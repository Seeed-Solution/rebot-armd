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

fn opt_bool(v: &Value, key: &str) -> Result<Option<bool>, ImportInvalid> {
    match v.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Bool(b)) => Ok(Some(*b)),
        Some(_) => Err(bad("sidecar", format!("{key} must be a boolean"))),
    }
}

/// Embedded tool provenance read from a Parquet file's schema key-value metadata.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EmbeddedProvenance {
    pub tool_id: Option<String>,
    pub tool_config_digest: Option<String>,
    pub has_gripper_motion: Option<bool>,
}

fn parse_kv_bool(raw: &str, key: &str) -> Result<bool, String> {
    match raw.trim() {
        "true" => Ok(true),
        "false" => Ok(false),
        other => Err(format!("parquet metadata '{key}' has invalid boolean {other:?}")),
    }
}

/// Extract and validate embedded tool provenance from Parquet schema key-value metadata
/// (checking top-level keys and optional `"rebot.episode"` JSON block for internal consistency).
pub fn extract_embedded_provenance(
    kv: &std::collections::HashMap<String, String>,
) -> Result<EmbeddedProvenance, String> {
    let mut top = EmbeddedProvenance::default();
    if let Some(raw) = kv.get("tool_id") {
        if raw.trim().is_empty() {
            return Err("parquet metadata 'tool_id' cannot be empty".into());
        }
        top.tool_id = Some(raw.clone());
    }
    if let Some(raw) = kv.get("tool_config_digest") {
        if raw.trim().is_empty() {
            return Err("parquet metadata 'tool_config_digest' cannot be empty".into());
        }
        top.tool_config_digest = Some(raw.clone());
    }
    if let Some(raw) = kv.get("has_gripper_motion") {
        top.has_gripper_motion = Some(parse_kv_bool(raw, "has_gripper_motion")?);
    }

    if let Some(ep_json) = kv.get("rebot.episode") {
        let val: Value = serde_json::from_str(ep_json)
            .map_err(|e| format!("parquet metadata 'rebot.episode' is invalid JSON: {e}"))?;
        if !val.is_object() {
            return Err("parquet metadata 'rebot.episode' must be a JSON object".into());
        }
        if let Some(v) = val.get("tool_id").filter(|v| !v.is_null()) {
            let tid = v
                .as_str()
                .ok_or_else(|| "parquet metadata 'rebot.episode.tool_id' must be a string".to_string())?;
            if tid.trim().is_empty() {
                return Err("parquet metadata 'rebot.episode.tool_id' cannot be empty".into());
            }
            if let Some(ref existing) = top.tool_id {
                if existing != tid {
                    return Err(format!("parquet metadata 'tool_id' ({existing}) conflicts with 'rebot.episode.tool_id' ({tid})"));
                }
            } else {
                top.tool_id = Some(tid.to_string());
            }
        }
        if let Some(v) = val.get("tool_config_digest").filter(|v| !v.is_null()) {
            let tdig = v
                .as_str()
                .ok_or_else(|| "parquet metadata 'rebot.episode.tool_config_digest' must be a string".to_string())?;
            if tdig.trim().is_empty() {
                return Err("parquet metadata 'rebot.episode.tool_config_digest' cannot be empty".into());
            }
            if let Some(ref existing) = top.tool_config_digest {
                if existing != tdig {
                    return Err(format!("parquet metadata 'tool_config_digest' ({existing}) conflicts with 'rebot.episode.tool_config_digest' ({tdig})"));
                }
            } else {
                top.tool_config_digest = Some(tdig.to_string());
            }
        }
        if let Some(v) = val.get("has_gripper_motion").filter(|v| !v.is_null()) {
            let gm = v.as_bool().ok_or_else(|| "parquet metadata 'rebot.episode.has_gripper_motion' must be boolean".to_string())?;
            if let Some(existing) = top.has_gripper_motion {
                if existing != gm {
                    return Err(format!("parquet metadata 'has_gripper_motion' ({existing}) conflicts with 'rebot.episode.has_gripper_motion' ({gm})"));
                }
            } else {
                top.has_gripper_motion = Some(gm);
            }
        }
    }
    Ok(top)
}

/// Check `sidecar` (JSON) and `frames` against `ctx` (without Parquet KV metadata).
pub fn check(ctx: &ImportContext, sidecar: &Value, frames: &[RecordFrame]) -> Result<CheckedImport, ImportInvalid> {
    check_with_provenance(ctx, sidecar, frames, None)
}

/// Check `sidecar` (JSON), `frames`, and optional Parquet KV metadata against `ctx`, enforcing
/// embedded tool provenance equality and rejecting sidecar erasure or conflicting fields.
pub fn check_with_provenance(
    ctx: &ImportContext,
    sidecar: &Value,
    frames: &[RecordFrame],
    parquet_kv: Option<&std::collections::HashMap<String, String>>,
) -> Result<CheckedImport, ImportInvalid> {
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
    let tool_id = opt_str(sidecar, "tool_id")?;
    if tool_id.as_deref().is_some_and(|s| s.trim().is_empty()) {
        return Err(bad("tool_provenance", "sidecar tool_id cannot be empty"));
    }
    let tool_config_digest = opt_str(sidecar, "tool_config_digest")?;
    if tool_config_digest.as_deref().is_some_and(|s| s.trim().is_empty()) {
        return Err(bad("tool_provenance", "sidecar tool_config_digest cannot be empty"));
    }
    let sidecar_has_gripper = opt_bool(sidecar, "has_gripper_motion")?;
    let emb = match parquet_kv {
        Some(kv) => extract_embedded_provenance(kv).map_err(|e| bad("tool_provenance", e))?,
        None => EmbeddedProvenance::default(),
    };
    if emb.tool_id != tool_id {
        return Err(bad(
            "tool_provenance",
            format!(
                "parquet embedded tool_id ({:?}) does not match sidecar tool_id ({:?})",
                emb.tool_id, tool_id
            ),
        ));
    }
    if emb.tool_config_digest != tool_config_digest {
        return Err(bad(
            "tool_provenance",
            format!(
                "parquet embedded tool_config_digest ({:?}) does not match sidecar tool_config_digest ({:?})",
                emb.tool_config_digest, tool_config_digest
            ),
        ));
    }
    if let Some(eb) = emb.has_gripper_motion {
        match sidecar_has_gripper {
            None => {
                return Err(bad(
                    "tool_provenance",
                    "sidecar omits embedded parquet has_gripper_motion provenance",
                ));
            }
            Some(sb) if sb != eb => {
                return Err(bad(
                    "tool_provenance",
                    format!(
                        "parquet embedded has_gripper_motion ({eb}) conflicts with sidecar ({sb})"
                    ),
                ));
            }
            Some(_) => {}
        }
    }
    let frames_has_gripper = super::store::episode_has_gripper_motion(frames, ctx.gripper_joint);
    if (emb.has_gripper_motion == Some(false) || sidecar_has_gripper == Some(false)) && frames_has_gripper {
        return Err(bad(
            "tool_provenance",
            "has_gripper_motion=false conflicts with gripper motion present in episode frames",
        ));
    }
    let has_gripper_motion = emb.has_gripper_motion.unwrap_or(false)
        || sidecar_has_gripper.unwrap_or(false)
        || frames_has_gripper;
    let sc = EpisodeSidecar {
        id: String::new(),
        dataset: opt_str(sidecar, "dataset")?.unwrap_or_else(|| DEFAULT_DATASET.into()),
        task, source: "import".into(), sequence_id: None, repeat_index: None,
        outcome: match opt_str(sidecar, "outcome")?.as_deref() { Some("aborted") => "aborted".into(), _ => "ok".into() },
        recorded_at, duration_s: duration, frames: frames.len(), joint_names: names, units: "rad".into(), joint_zero,
        profile_id: ctx.profile_id.clone(), armd_version: env!("CARGO_PKG_VERSION").into(),
        fps_nominal: sidecar.get("fps_nominal").and_then(Value::as_f64).filter(|f| f.is_finite() && *f > 0.0).unwrap_or(ctx.fps),
        truncated: false, stopped_by: None, arm_dof: Some(ctx.arm_dof), gripper_joint: ctx.gripper_joint, recovered: None,
        first_state: Some(frames[0].observation_state.clone()), last_state: Some(frames[frames.len() - 1].observation_state.clone()),
        tool_id, tool_config_digest, has_gripper_motion: Some(has_gripper_motion),
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

        // B05: embedded parquet provenance checks
        let mut kv = std::collections::HashMap::new();
        kv.insert("tool_id".into(), "soft_finray_gripper".into());
        kv.insert("tool_config_digest".into(), "v1:abc".into());
        kv.insert("has_gripper_motion".into(), "true".into());

        // 1. Sidecar omitting embedded tool provenance -> rejected
        let err_omit = check_with_provenance(&c, &side, &frames(&q, 10), Some(&kv)).unwrap_err();
        assert_eq!(err_omit.reason, "tool_provenance");

        // 2. Conflicting tool_id or digest -> rejected
        let mut bad_tid = side.clone();
        bad_tid["tool_id"] = json!("other_tool");
        bad_tid["tool_config_digest"] = json!("v1:abc");
        bad_tid["has_gripper_motion"] = json!(true);
        assert_eq!(check_with_provenance(&c, &bad_tid, &frames(&q, 10), Some(&kv)).unwrap_err().reason, "tool_provenance");

        // 3. Sidecar claims has_gripper_motion: false while frames move gripper -> rejected
        let mut gf = frames(&q, 10);
        gf[5].observation_state[6] = -1.5;
        let mut fake_false = side.clone();
        fake_false["has_gripper_motion"] = json!(false);
        assert_eq!(check_with_provenance(&c, &fake_false, &gf, None).unwrap_err().reason, "tool_provenance");

        // 4. Matching embedded + sidecar succeeds and preserves provenance
        let mut ok_s = side.clone();
        ok_s["tool_id"] = json!("soft_finray_gripper");
        ok_s["tool_config_digest"] = json!("v1:abc");
        ok_s["has_gripper_motion"] = json!(true);
        let ok_imp = check_with_provenance(&c, &ok_s, &frames(&q, 10), Some(&kv)).unwrap();
        assert_eq!(ok_imp.sidecar.tool_id.as_deref(), Some("soft_finray_gripper"));
        assert_eq!(ok_imp.sidecar.tool_config_digest.as_deref(), Some("v1:abc"));
        assert_eq!(ok_imp.sidecar.has_gripper_motion, Some(true));
    }
}
