//! `POST /api/v1/episodes/import`: check an episode pushed from a PC (parquet + sidecar JSON)
//! against this arm before it enters the store. Pure (no I/O); the caller writes it.
//!
//! Refused (IMPORT_INVALID, `reason`):
//! `sidecar` (not a JSON object / bad field types), `schema` (parquet columns), `units`
//! (anything but `rad`), `joint_names` (other joints) / `joint_order` (same joints, other
//! order), `profile` (another profile id), `dof` (row length), `empty` (< 2 frames),
//! `timestamps` (not finite / decreasing), `too_long` (> [`MAX_EPISODE_S`]), `non_finite`
//! (NaN / inf joint values), `soft_limit` (a value outside the joint's soft limits).
//! A joint zero other than the current one is not refused: the episode is stored stale
//! (`stale: true`, DATA_STALE on replay without `allow_stale`); a sidecar without `joint_zero`
//! counts as stale.
use serde_json::Value;

use super::episode::RecordFrame;
use super::store::{EpisodeSidecar, JointZeroStamp, DEFAULT_DATASET, MAX_EPISODE_S};
use crate::model::JointLimit;

/// What the episode is checked against (taken under the supervisor lock).
#[derive(Debug, Clone)]
pub struct ImportContext {
    pub joint_names: Vec<String>,
    pub limits: Vec<JointLimit>,
    pub profile_id: Option<String>,
    pub arm_dof: usize,
    pub gripper_joint: Option<usize>,
    pub zero_offsets: Vec<f32>,
    pub fps: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ImportInvalid {
    pub reason: &'static str,
    pub message: String,
}

fn bad(reason: &'static str, message: impl Into<String>) -> ImportInvalid { ImportInvalid { reason, message: message.into() } }

/// Tolerance on the soft-limit check (rad), above float rounding of the stored f32 values.
pub const LIMIT_EPS: f32 = 1e-4;

/// A checked import: the sidecar to store (id set by the caller) and whether it is stale.
#[derive(Debug, Clone)]
pub struct CheckedImport {
    pub sidecar: EpisodeSidecar,
    pub stale: bool,
}

fn opt_str(v: &Value, key: &str) -> Result<Option<String>, ImportInvalid> {
    match v.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s.clone())),
        Some(_) => Err(bad("sidecar", format!("{key} must be a string"))),
    }
}

/// Check `sidecar` (JSON) and `frames` (read from the parquet) against `ctx`.
pub fn check(ctx: &ImportContext, sidecar: &Value, frames: &[RecordFrame]) -> Result<CheckedImport, ImportInvalid> {
    if !sidecar.is_object() { return Err(bad("sidecar", "the sidecar must be a JSON object")); }
    let units = opt_str(sidecar, "units")?.ok_or_else(|| bad("units", "units is required (rad)"))?;
    if units != "rad" {
        return Err(bad("units", format!("units is '{units}'; only rad is accepted (convert deg / so101_norm before the import)")));
    }
    let names: Vec<String> = match sidecar.get("joint_names") {
        Some(Value::Array(a)) => a.iter().map(|n| n.as_str().map(str::to_string).ok_or_else(|| bad("sidecar", "joint_names must be strings"))).collect::<Result<_, _>>()?,
        _ => return Err(bad("joint_names", "joint_names (array) is required")),
    };
    if names != ctx.joint_names {
        let mut a = names.clone();
        let mut b = ctx.joint_names.clone();
        a.sort();
        b.sort();
        return Err(if a == b {
            bad("joint_order", format!("joint order {names:?} differs from this arm's {:?}", ctx.joint_names))
        } else {
            bad("joint_names", format!("joints {names:?} are not this arm's {:?}", ctx.joint_names))
        });
    }
    if let (Some(p), Some(mine)) = (opt_str(sidecar, "profile_id")?, ctx.profile_id.as_ref()) {
        if &p != mine { return Err(bad("profile", format!("recorded on profile '{p}', this arm runs '{mine}'"))); }
    }
    let dof = ctx.joint_names.len();
    if frames.len() < 2 { return Err(bad("empty", format!("{} frame(s); an episode needs at least 2", frames.len()))); }
    for (k, f) in frames.iter().enumerate() {
        if f.observation_state.len() != dof || f.action.len() != dof {
            return Err(bad("dof", format!("frame {k} has {} / {} values; this arm has {dof} joints", f.observation_state.len(), f.action.len())));
        }
        if !f.timestamp.is_finite() || (k > 0 && f.timestamp < frames[k - 1].timestamp) {
            return Err(bad("timestamps", format!("timestamps are not finite and non-decreasing at frame {k}")));
        }
        for (what, values) in [("observation.state", &f.observation_state), ("action", &f.action)] {
            for (j, (&q, l)) in values.iter().zip(&ctx.limits).enumerate() {
                if !q.is_finite() { return Err(bad("non_finite", format!("frame {k} {what}[{j}] is not finite"))); }
                if q < l.min_position - LIMIT_EPS || q > l.max_position + LIMIT_EPS {
                    return Err(bad("soft_limit", format!("frame {k} {what} joint {} ({}) is {q:.4} rad, outside [{:.4}, {:.4}]",
                        j + 1, l.name, l.min_position, l.max_position)));
                }
            }
        }
    }
    let duration = frames[frames.len() - 1].timestamp - frames[0].timestamp;
    if duration > MAX_EPISODE_S { return Err(bad("too_long", format!("{duration:.1} s; the limit is {MAX_EPISODE_S} s"))); }
    let joint_zero = match sidecar.get("joint_zero") {
        None | Some(Value::Null) => JointZeroStamp { offsets_rad: Vec::new(), rev: "unknown".into() },
        Some(v) => serde_json::from_value::<JointZeroStamp>(v.clone()).map_err(|e| bad("sidecar", format!("joint_zero: {e}")))?,
    };
    if !joint_zero.offsets_rad.iter().all(|o| o.is_finite()) { return Err(bad("sidecar", "joint_zero offsets must be finite")); }
    let stale = joint_zero.stale_against(&ctx.zero_offsets);
    let recorded_at = opt_str(sidecar, "recorded_at")?
        .filter(|t| chrono::DateTime::parse_from_rfc3339(t).is_ok())
        .unwrap_or_else(|| chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true));
    let task = opt_str(sidecar, "task")?.unwrap_or_default();
    if task.chars().count() > 256 { return Err(bad("sidecar", "task is at most 256 characters")); }
    let sc = EpisodeSidecar {
        id: String::new(),
        dataset: opt_str(sidecar, "dataset")?.unwrap_or_else(|| DEFAULT_DATASET.into()),
        task, source: "import".into(), sequence_id: None, repeat_index: None,
        outcome: match opt_str(sidecar, "outcome")?.as_deref() { Some("aborted") => "aborted".into(), _ => "ok".into() },
        recorded_at, duration_s: duration, frames: frames.len(), joint_names: names, units: "rad".into(), joint_zero,
        profile_id: ctx.profile_id.clone(), armd_version: env!("CARGO_PKG_VERSION").into(),
        fps_nominal: sidecar.get("fps_nominal").and_then(Value::as_f64).filter(|f| f.is_finite() && *f > 0.0).unwrap_or(ctx.fps),
        truncated: false, stopped_by: None, arm_dof: Some(ctx.arm_dof), gripper_joint: ctx.gripper_joint, recovered: None,
    };
    Ok(CheckedImport { sidecar: sc, stale })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn ctx() -> ImportContext {
        let cfg = crate::model::ArmConfig::rebot_dm();
        ImportContext {
            joint_names: cfg.limits.iter().map(|l| l.name.clone()).collect(), limits: cfg.limits.clone(), profile_id: cfg.profile_id.clone(),
            arm_dof: cfg.arm_dof(), gripper_joint: cfg.gripper_index(), zero_offsets: vec![0.0; cfg.dof], fps: 50.0,
        }
    }

    fn frames(q: &[f32], n: usize) -> Vec<RecordFrame> {
        (0..n).map(|k| RecordFrame { timestamp: k as f64 * 0.02, observation_state: q.to_vec(), action: q.to_vec() }).collect()
    }

    #[test]
    fn rules() {
        let mut c = ctx();
        c.profile_id = Some("rebot-b601-dm".into());
        let q = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
        let side = json!({"units": "rad", "joint_names": c.joint_names, "joint_zero": {"offsets_rad": vec![0.0; 7], "rev": "x"}, "task": "t"});
        let ok = check(&c, &side, &frames(&q, 10)).unwrap();
        assert!(!ok.stale && ok.sidecar.source == "import" && ok.sidecar.frames == 10);
        let reason = |s: &Value, f: &[RecordFrame]| check(&c, s, f).unwrap_err().reason;
        // degrees refused
        let mut deg = side.clone();
        deg["units"] = json!("deg");
        assert_eq!(reason(&deg, &frames(&q, 10)), "units");
        let mut none = side.clone();
        none.as_object_mut().unwrap().remove("units");
        assert_eq!(reason(&none, &frames(&q, 10)), "units");
        // wrong joint order refused
        let mut order = side.clone();
        let mut names = c.joint_names.clone();
        names.swap(0, 1);
        order["joint_names"] = json!(names);
        assert_eq!(reason(&order, &frames(&q, 10)), "joint_order");
        let mut other = side.clone();
        other["joint_names"] = json!(["a", "b"]);
        assert_eq!(reason(&other, &frames(&q, 10)), "joint_names");
        // values: degrees passed off as rad leave the soft limits; NaN; length
        let deg_q: Vec<f32> = q.iter().map(|v| v.to_degrees()).collect();
        assert_eq!(reason(&side, &frames(&deg_q, 10)), "soft_limit");
        let mut nan = frames(&q, 10);
        nan[3].action[2] = f32::NAN;
        assert_eq!(reason(&side, &nan), "non_finite");
        assert_eq!(reason(&side, &frames(&q[..6], 10)), "dof");
        assert_eq!(reason(&side, &frames(&q, 1)), "empty");
        let mut long = frames(&q, 3);
        long[2].timestamp = 601.0;
        assert_eq!(reason(&side, &long), "too_long");
        let mut back = frames(&q, 3);
        back[2].timestamp = -1.0;
        assert_eq!(reason(&side, &back), "timestamps");
        let mut prof = side.clone();
        prof["profile_id"] = json!("someone-else");
        assert_eq!(reason(&prof, &frames(&q, 10)), "profile");
        // another joint zero: accepted, stale
        let mut z = side.clone();
        z["joint_zero"] = json!({"offsets_rad": [0.1, 0, 0, 0, 0, 0, 0], "rev": "y"});
        assert!(check(&c, &z, &frames(&q, 10)).unwrap().stale);
        let mut nz = side.clone();
        nz.as_object_mut().unwrap().remove("joint_zero");
        assert!(check(&c, &nz, &frames(&q, 10)).unwrap().stale);
    }
}
