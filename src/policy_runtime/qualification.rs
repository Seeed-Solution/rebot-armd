//! Device-issued qualification ledger for real-hardware policy runs.
//!
//! A `production_release` gate is authorized only by records that armd itself appended to
//! this ledger when a `supervised_first_run` live run on real hardware ended. Callers (the
//! release gate document, the WS request) cannot supply or edit these records; a
//! `supervised_real_run_record` inside a release gate is ignored.
//!
//! Format: one JSON object per line (`rebot-policy-qualification/v1`), appended with
//! `O_APPEND` and fsynced. Default path `/userdata/rebot-teleop/policy_qualification.jsonl`
//! (when `/userdata/rebot-teleop` exists) or `$ARMD_POLICY_LEDGER`; without a path no record
//! is written and production releases are refused.
//!
//! A run counts as passed when it ran until its `max_seconds` deadline (`stop_reason`
//! `MAX_SECONDS`, action `done`) without a stop, fault, hold or lease loss. This says the run
//! finished without a safety stop under supervision; it does not judge task success.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use super::release_gate::{AuthorizationTier, VerifiedReleaseGate};
use super::PolicyError;

pub const QUALIFICATION_SCHEMA: &str = "rebot-policy-qualification/v1";
pub const DEFAULT_LEDGER_DIR: &str = "/userdata/rebot-teleop";
pub const LEDGER_FILE_NAME: &str = "policy_qualification.jsonl";
/// Passed supervised runs with the same binding required before a production release.
pub const MIN_SUPERVISED_PASSES: usize = 3;
/// Larger ledgers are refused (fail closed) rather than read partially.
pub const MAX_LEDGER_BYTES: u64 = 1024 * 1024;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct QualificationRecord {
    pub schema: String,
    pub recorded_unix_s: u64,
    pub action_id: u64,
    pub policy_sha256: String,
    pub scene_id: String,
    pub tool_id: String,
    pub calibration_digest: String,
    pub profile_id: String,
    pub fps: u64,
    pub tier: String,
    pub max_speed_scale: f64,
    pub steps: u64,
    pub stop_reason: String,
    pub passed: bool,
    /// Per-step timing of the run (absent in older records).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub policy_stats: Option<crate::motion::PolicyStats>,
}

impl QualificationRecord {
    /// Record for a supervised run that used `gate`.
    pub fn for_run(gate: &VerifiedReleaseGate, action_id: u64, steps: u64, stop_reason: &str, passed: bool) -> Self {
        Self {
            schema: QUALIFICATION_SCHEMA.into(),
            recorded_unix_s: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |d| d.as_secs()),
            action_id,
            policy_sha256: gate.policy_sha256().to_ascii_lowercase(),
            scene_id: gate.scene_id().into(),
            tool_id: gate.tool_id().into(),
            calibration_digest: gate.calibration_digest().to_ascii_lowercase(),
            profile_id: gate.canonical_profile_id().into(),
            fps: gate.fps(),
            tier: gate.authorization_tier().as_str().into(),
            max_speed_scale: gate.max_speed_scale(),
            steps,
            stop_reason: stop_reason.into(),
            passed,
            policy_stats: None,
        }
    }

    pub fn with_policy_stats(mut self, stats: crate::motion::PolicyStats) -> Self {
        self.policy_stats = Some(stats);
        self
    }

    /// Same artifact, deployment (scene, tool, calibration), profile and rate as `gate`.
    fn binds(&self, gate: &VerifiedReleaseGate) -> bool {
        self.schema == QUALIFICATION_SCHEMA
            && self.tier == AuthorizationTier::SupervisedFirstRun.as_str()
            && self.max_speed_scale <= super::SUPERVISED_FIRST_RUN_MAX_SPEED_SCALE + 1e-12
            && self.policy_sha256.eq_ignore_ascii_case(gate.policy_sha256())
            && self.scene_id == gate.scene_id()
            && self.tool_id == gate.tool_id()
            && self.calibration_digest.eq_ignore_ascii_case(gate.calibration_digest())
            && self.profile_id == gate.canonical_profile_id()
            && self.fps == gate.fps()
    }
}

/// The ledger path: `$ARMD_POLICY_LEDGER`, else the default under `/userdata/rebot-teleop`
/// when that directory exists, else none.
pub fn ledger_path_from_env() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("ARMD_POLICY_LEDGER").filter(|p| !p.is_empty()) {
        return Some(PathBuf::from(p));
    }
    let dir = Path::new(DEFAULT_LEDGER_DIR);
    dir.is_dir().then(|| dir.join(LEDGER_FILE_NAME))
}

/// Append one record (one line, fsynced).
pub fn append_record(path: &Path, record: &QualificationRecord) -> std::io::Result<()> {
    let mut line = serde_json::to_vec(record).map_err(std::io::Error::other)?;
    line.push(b'\n');
    let mut f = OpenOptions::new().create(true).append(true).open(path)?;
    f.write_all(&line)?;
    f.sync_all()
}

/// All well-formed records; a missing file is an empty ledger. Malformed lines are skipped
/// (they never authorize anything).
pub fn load_records(path: &Path) -> Result<Vec<QualificationRecord>, PolicyError> {
    let file = match File::open(path) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(PolicyError::ReleaseGate(format!("cannot open qualification ledger: {e}"))),
    };
    let mut buf = Vec::new();
    file.take(MAX_LEDGER_BYTES + 1)
        .read_to_end(&mut buf)
        .map_err(|e| PolicyError::ReleaseGate(format!("cannot read qualification ledger: {e}")))?;
    if buf.len() as u64 > MAX_LEDGER_BYTES {
        return Err(PolicyError::ReleaseGate(format!(
            "qualification ledger exceeds {MAX_LEDGER_BYTES} bytes"
        )));
    }
    Ok(buf
        .split(|&b| b == b'\n')
        .filter_map(|l| serde_json::from_slice::<QualificationRecord>(l).ok())
        .collect())
}

/// A production release needs at least [`MIN_SUPERVISED_PASSES`] passed device records with
/// the gate's binding, and the most recent such record must be a pass.
pub fn check_production_qualified(records: &[QualificationRecord], gate: &VerifiedReleaseGate) -> Result<(), PolicyError> {
    let bound: Vec<&QualificationRecord> = records.iter().filter(|r| r.binds(gate)).collect();
    let passes = bound.iter().filter(|r| r.passed).count();
    if passes < MIN_SUPERVISED_PASSES {
        return Err(PolicyError::ReleaseGate(format!(
            "production_release needs {MIN_SUPERVISED_PASSES} passed supervised runs recorded by this device for this artifact and deployment; found {passes}"
        )));
    }
    if bound.last().is_some_and(|r| !r.passed) {
        return Err(PolicyError::ReleaseGate(
            "the most recent supervised run recorded for this artifact and deployment did not pass".into(),
        ));
    }
    Ok(())
}
