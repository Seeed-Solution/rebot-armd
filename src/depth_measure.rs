//! Depth measurement without motion (`POST /api/v1/grasp/measure`, WS command of the same path
//! without a lease): one perception trigger at the current pose, the frame's table fit and, per
//! detection, the depth measurement the grasp would compute from it.
//!
//! The fit and the measurement are the grasp path's own functions
//! ([`grasp::calibrate_depth_why`], [`grasp::measure_detection_depth_with_cal_why`]) on the
//! camera model a grasp would build ([`grasp::CamModel::assumed`]) at the FK pose of the
//! commanded positions (the pose a grasp's orchestrator reads). Nothing moves, nothing is
//! written; the reply adds the fit's residuals and the calibration in effect so an evaluation
//! can tell which calibration a number came from.
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::{json, Map, Value};

use crate::grasp::{self, CamModel, DetectionFrame, DetectionSource, GraspSetup};
use crate::kinematics::{self, Pose};
use crate::motion::MotionError;
use crate::rt_core::publish::{ReadRequirements, ServiceReadError, ServiceSnapshot};
use crate::supervisor::{ArmError, ArmSupervisor};

/// Path of the REST route and the WS command.
pub const PATH: &str = "/api/v1/grasp/measure";
/// A joint faster than this (rad/s, measured) refuses the measurement with `ARM_MOVING`.
pub const MAX_JOINT_SPEED: f64 = 0.02;
/// Commanded positions that change by more than this (rad) during the capture refuse it.
const MOVED_RAD: f64 = 1e-3;
/// Relative depth error of a table sample that counts as an inlier (the RANSAC criterion of
/// `fit_oblique_ransac`).
const INLIER_REL: f64 = 0.08;
/// Maximum allowed hardware feedback age for driver-backed captures (matches `feedback_valid`).
const MAX_FEEDBACK_AGE: Duration = Duration::from_millis(250);
/// Bounded wait for a causally post-capture published snapshot on a ticking owner.
const POST_CAPTURE_WAIT: Duration = Duration::from_millis(250);

const YOLO_DEPTH: &[&str] = &["yolo", "depth"];
const DEPTH_ONLY: &[&str] = &["depth"];

#[derive(Debug, Clone, PartialEq)]
pub struct MeasureRequest {
    /// Only detections with this label (case-insensitive); None = all.
    pub label: Option<String>,
    pub min_score: f64,
    pub pipelines: &'static [&'static str],
}

fn invalid(msg: impl Into<String>) -> ArmError { MotionError::InvalidCommand(msg.into()).into() }

fn rejected(status: u16, code: &str, message: impl Into<String>) -> ArmError {
    ArmError::Rejected { status, code: code.into(), message: message.into() }
}

/// Body `{label?, min_score? (0..1, default 0.3), pipelines? (["yolo","depth"] default, or ["depth"])}`.
pub fn parse_request(body: &Value) -> Result<MeasureRequest, ArmError> {
    let empty = Map::new();
    let obj = match body {
        Value::Null => &empty,
        Value::Object(o) => o,
        _ => return Err(invalid("Command body must be an object")),
    };
    if let Some(k) = obj.keys().find(|k| !matches!(k.as_str(), "label" | "min_score" | "pipelines")) {
        return Err(invalid(format!("unknown field {k} (label, min_score, pipelines)")));
    }
    let label = match obj.get("label") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) if !s.is_empty() && s.len() <= 128 => Some(s.clone()),
        Some(_) => return Err(invalid("label must be a non-empty string (at most 128 characters)")),
    };
    let min_score = match obj.get("min_score") {
        None | Some(Value::Null) => 0.3,
        Some(v) => match v.as_f64() {
            Some(s) if (0.0..=1.0).contains(&s) => s,
            _ => return Err(invalid("min_score must be a number in [0, 1]")),
        },
    };
    let pipelines = match obj.get("pipelines") {
        None | Some(Value::Null) => YOLO_DEPTH,
        Some(Value::Array(a)) => {
            let mut names: Vec<&str> = a.iter().map(|v| v.as_str().unwrap_or("?")).collect();
            names.sort_unstable();
            match names.as_slice() {
                ["depth", "yolo"] => YOLO_DEPTH,
                ["depth"] => DEPTH_ONLY,
                _ => return Err(invalid("pipelines must be [\"yolo\", \"depth\"] or [\"depth\"]")),
            }
        }
        Some(_) => return Err(invalid("pipelines must be an array")),
    };
    Ok(MeasureRequest { label, min_score, pipelines })
}

/// What the measurement needs from the supervisor, taken from a coherent [`ServiceSnapshot`].
struct Snapshot {
    setup: GraspSetup,
    cam: CamModel,
    pose: Pose,
    tcp_offset: [f64; 3],
    commanded: Vec<f32>,
    measured: Vec<f32>,
    lens_known: bool,
    calibration: Value,
}

/// For driver-backed arms, enforce live feedback validity and freshness (no-driver mock arms
/// do not stamp hardware RX instants).
fn check_driver_feedback(svc: &ServiceSnapshot, now: Instant) -> Result<(), ArmError> {
    let Some(env) = svc.prep_env.as_ref() else {
        return Err(ArmError::from(ServiceReadError::PrepEnvUnavailable));
    };
    if !env.driver_present {
        return Ok(());
    }
    if !svc.feedback_valid {
        return Err(ArmError::from(ServiceReadError::FeedbackInvalid));
    }
    let arm_dof = svc.identity.arm_dof;
    if arm_dof == 0 || svc.snapshot.rt.feedback_instants.len() < arm_dof {
        return Err(ArmError::from(ServiceReadError::FeedbackStale { joint: 0 }));
    }
    for (joint, slot) in svc.snapshot.rt.feedback_instants[..arm_dof].iter().enumerate() {
        let Some(rx) = *slot else {
            return Err(ArmError::from(ServiceReadError::FeedbackStale { joint }));
        };
        if now.saturating_duration_since(rx) > MAX_FEEDBACK_AGE {
            return Err(ArmError::from(ServiceReadError::FeedbackStale { joint }));
        }
    }
    Ok(())
}

/// `ARM_MOVING` when a trajectory or servo stream runs or a joint (J1..J6) is faster than
/// [`MAX_JOINT_SPEED`] (or has non-finite measured velocity).
fn check_still(svc: &ServiceSnapshot) -> Result<(), ArmError> {
    let arm_dof = svc.identity.arm_dof;
    if svc.motion_active {
        return Err(rejected(409, "ARM_MOVING", "A motion is running; measure at rest"));
    }
    if let Some((i, v)) = svc
        .snapshot
        .rt
        .velocities
        .iter()
        .take(arm_dof)
        .enumerate()
        .find(|(_, v)| !v.is_finite() || f64::from(v.abs()) > MAX_JOINT_SPEED)
    {
        return Err(rejected(
            409,
            "ARM_MOVING",
            format!("Joint {} moves at {:.3} rad/s (limit {MAX_JOINT_SPEED})", i + 1, v.abs()),
        ));
    }
    Ok(())
}

fn snapshot_from_service(svc: &ServiceSnapshot) -> Result<Snapshot, ArmError> {
    let env = svc
        .prep_env
        .as_ref()
        .ok_or_else(|| ArmError::from(ServiceReadError::PrepEnvUnavailable))?;
    let Some(setup) = env.grasp_setup.clone() else {
        return Err(MotionError::Unsupported("Depth measurement needs a profile with kinematics, a gripper policy and a grasp setup".into()).into());
    };
    check_driver_feedback(svc, Instant::now())?;
    check_still(svc)?;
    let arm = &env.config.arm_model;
    let cam = CamModel::assumed(arm, &setup, &env.config.camera_mount);
    let pose = arm
        .fk_f32(&svc.snapshot.rt.commanded_positions)
        .ok_or_else(|| ArmError::Internal("no FK pose for the commanded positions".into()))?;
    Ok(Snapshot {
        cam,
        pose,
        tcp_offset: env.config.tcp_offset,
        commanded: svc.snapshot.rt.commanded_positions.clone(),
        measured: svc.snapshot.rt.positions.clone(),
        lens_known: env.config.camera_mount.offset_m.is_some(),
        calibration: fingerprint(svc, &setup),
        setup,
    })
}

/// FNV-1a 64 of a JSON value's text (a stable short id of the stored intrinsics).
fn fnv64(v: &Value) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in v.to_string().bytes() { h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3); }
    format!("{h:016x}")
}

/// The calibration a measurement used: the stored file's identity (`solved_at`, intrinsics and
/// their hash, stale), the live camera mount, the grasp setup's intrinsics and table, the
/// kinematic correction and its model level (`kinematics.fit.level`).
fn fingerprint(svc: &ServiceSnapshot, setup: &GraspSetup) -> Value {
    let env = svc
        .prep_env
        .as_ref()
        .expect("fingerprint requires ActionPrepEnv");
    let file = env.calibration.as_ref();
    let intrinsics = file.and_then(|c| c.intrinsics.clone());
    let kin = file.and_then(|c| c.kinematics.as_ref());
    json!({
        "calibrated": env.camera_calibrated,
        "solved_at": file.and_then(|c| c.solved_at.clone()),
        "stale": file.and_then(|c| c.stale.as_ref()).map(|s| json!({"reason": s.reason, "at": s.at})),
        "intrinsics_hash": intrinsics.as_ref().map(fnv64),
        "intrinsics": intrinsics,
        "camera_mount": crate::calibration::mount_summary(&env.config.camera_mount),
        "grasp": crate::calibration::grasp_fields(setup),
        "table_z_m": setup.table_z_m,
        "kinematics_corrected": env.config.arm_model.correction().is_some(),
        "kinematics_level": kin.and_then(|k| k.fit.as_ref()).and_then(|f| f.get("level")).cloned(),
        "calibration_revision": svc.revisions.calibration,
        // CamModel is a pinhole: k1 / k2 of the stored intrinsics are not applied to the samples
        "distortion_modelled": false,
    })
}

/// Residuals of a table fit `1 / z = a * disp + b` over `(1 / z_fk, disp, z_fk)` samples:
/// depth residual z_pred - z_fk in mm (rms, median |r|, max |r|) and the inliers within
/// [`INLIER_REL`] relative depth error.
pub fn fit_residuals(samples: &[(f64, f64, f64)], a: f64, b: f64) -> Value {
    let mut res: Vec<f64> = Vec::new();
    let mut inliers = 0;
    for &(_, disp, z_fk) in samples {
        let denom = a * disp + b;
        if denom <= 1e-4 { continue; }
        let r = 1.0 / denom - z_fk;
        if (r / z_fk).abs() <= INLIER_REL { inliers += 1; }
        res.push(r * 1000.0);
    }
    if res.is_empty() { return json!({"n": 0, "inliers": 0, "rms_mm": null, "median_abs_mm": null, "max_abs_mm": null}); }
    let rms = (res.iter().map(|r| r * r).sum::<f64>() / res.len() as f64).sqrt();
    let mut abs: Vec<f64> = res.iter().map(|r| r.abs()).collect();
    abs.sort_by(f64::total_cmp);
    let n = abs.len();
    let median = if n % 2 == 1 { abs[n / 2] } else { 0.5 * (abs[n / 2 - 1] + abs[n / 2]) };
    json!({"n": n, "inliers": inliers, "rms_mm": rms, "median_abs_mm": median, "max_abs_mm": abs[n - 1]})
}

/// Ordinary least squares `1 / z = a * disp + b` (no outlier rejection) and its R² in 1 / z:
/// a diagnostic next to armd's robust fit. None below 2 samples or without disparity spread.
pub fn least_squares(samples: &[(f64, f64, f64)]) -> Option<Value> {
    let n = samples.len() as f64;
    if samples.len() < 2 { return None; }
    let (sx, sy) = samples.iter().fold((0.0, 0.0), |(sx, sy), &(y, x, _)| (sx + x, sy + y));
    let (mx, my) = (sx / n, sy / n);
    let (sxx, sxy, syy) = samples.iter().fold((0.0, 0.0, 0.0), |(a, b, c), &(y, x, _)| (a + (x - mx) * (x - mx), b + (x - mx) * (y - my), c + (y - my) * (y - my)));
    if sxx < 1e-12 { return None; }
    let a = sxy / sxx;
    let b = my - a * mx;
    let ss_res: f64 = samples.iter().map(|&(y, x, _)| (y - (a * x + b)).powi(2)).sum();
    let r2 = if syy > 0.0 { 1.0 - ss_res / syy } else { 0.0 };
    Some(json!({"a": a, "b": b, "r2_inv_z": r2, "residual": fit_residuals(samples, a, b)}))
}

/// `table_fit` of the reply: armd's calibration of this frame (`source`, `a`, `b` of
/// `1 / z = a * disp + b`, or `why` it failed), the sample counts, residuals of the fit and the
/// plain least-squares diagnostic.
fn table_fit(snap: &Snapshot, frame: &DetectionFrame, cal: &Result<grasp::DepthCalibration, &'static str>) -> Value {
    let Some(depth) = frame.depth.as_ref() else {
        return json!({"ok": false, "why": "no_depth_frame", "n_samples": 0, "n_valid": 0});
    };
    let samples = grasp::table_fk_samples(&snap.cam, &snap.pose, depth, snap.setup.table_z_m);
    let inv_z_span = samples.iter().map(|s| s.0).fold(f64::NEG_INFINITY, f64::max) - samples.iter().map(|s| s.0).fold(f64::INFINITY, f64::min);
    let mut v = json!({
        "metric_model": depth.metric_model,
        "n_samples": depth.table_samples.len(),
        "n_valid": samples.len(),
        "inv_z_span": if samples.is_empty() { Value::Null } else { json!(inv_z_span) },
        "least_squares": least_squares(&samples),
    });
    match cal {
        Ok(c) => {
            v["ok"] = json!(true);
            v["source"] = json!(c.source);
            v["a"] = json!(c.a);
            v["b"] = json!(c.b);
            v["residual"] = fit_residuals(&samples, c.a, c.b);
        }
        Err(why) => {
            v["ok"] = json!(false);
            v["why"] = json!(why);
        }
    }
    v
}

fn opt(v: Option<f64>) -> Value { v.map_or(Value::Null, |x| json!(x)) }

/// The reply for one frame (pure: the tests call it with synthetic frames).
fn reply(snap: &Snapshot, req: &MeasureRequest, frame: &DetectionFrame) -> Value {
    let cal = grasp::calibrate_depth_why(&snap.cam, &snap.pose, frame.depth.as_ref(), None, snap.setup.table_z_m);
    let detections: Vec<Value> = frame.detections.iter()
        .filter(|d| d.score >= req.min_score && req.label.as_ref().is_none_or(|l| d.label.eq_ignore_ascii_case(l)))
        .map(|d| {
            // as the grasp's height source `depth`: no depth without the lens centre
            let meas = if !snap.lens_known { Err("no_lens_centre") } else {
                cal.and_then(|c| grasp::measure_detection_depth_with_cal_why(&snap.cam, &snap.pose, d, &c))
            };
            let (measurement, why) = match meas {
                Ok(m) => (json!({"height_m": m.height_m, "depth_m": m.depth_m, "width_m": m.width_m, "length_m": m.length_m,
                                 "source": m.source, "jaw_clear": m.jaw_clear}), Value::Null),
                Err(w) => (Value::Null, json!(w)),
            };
            json!({
                "label": d.label, "score": d.score, "box_norm": d.box_norm,
                "disp_fg": opt(d.disp_fg), "disp_bg": opt(d.disp_bg), "depth_conf": opt(d.depth_conf),
                "angle_deg": opt(d.angle_deg), "angle_conf": opt(d.angle_conf), "grasp_width_px": opt(d.grasp_width_px),
                "length_px": opt(d.length_px), "jaw_clear": d.jaw_clear,
                "depth_angle_deg": opt(d.depth_angle_deg), "depth_angle_conf": opt(d.depth_angle_conf), "depth_width_px": opt(d.depth_width_px),
                "measurement": measurement, "why": why,
            })
        })
        .collect();
    let arm_dof = 6.min(snap.commanded.len());
    let tcp_pose = crate::motion::end_to_tcp(&snap.pose, &snap.tcp_offset);
    json!({
        "frame_id": frame.frame_id,
        "age_ms": Instant::now().saturating_duration_since(frame.captured_at).as_secs_f64() * 1000.0,
        "image_size_px": frame.image_size_px,
        "pipelines": req.pipelines,
        "camera": {
            "pose_source": "commanded",
            "commanded_rad": &snap.commanded[..arm_dof],
            "measured_rad": &snap.measured[..arm_dof.min(snap.measured.len())],
            "end_pose": {"xyz_m": snap.pose.p, "quat_wxyz": kinematics::r_to_quat(&snap.pose.r)},
            "tcp": {"xyz_m": tcp_pose.p, "quat_wxyz": kinematics::r_to_quat(&tcp_pose.r)},
            "origin_m": snap.cam.origin(&snap.pose),
            "quat_wxyz": kinematics::r_to_quat(&snap.cam.rotation(&snap.pose)),
            "fx": snap.cam.fx, "fy": snap.cam.fy, "cx": snap.cam.cx, "cy": snap.cam.cy, "size": snap.cam.size,
            "lens_known": snap.lens_known,
        },
        "calibration": snap.calibration,
        "table_fit": table_fit(snap, frame, &cal),
        "detections": detections,
    })
}

async fn read_post_capture_service(
    supervisor: &Arc<ArmSupervisor>,
    svc0: &ServiceSnapshot,
    capture_done_at: Instant,
) -> Result<Arc<ServiceSnapshot>, ArmError> {
    let base_req = ReadRequirements {
        require_within_valid_until: true,
        require_prep_env: true,
        ..ReadRequirements::default()
    };
    if svc0.publish_seq == 0 && supervisor.owner_mode() == crate::supervisor::OWNER_LEGACY {
        // No-ticker legacy owner: read_services synthesizes a fresh snapshot directly under the
        // supervisor read lock at `Instant::now() >= capture_done_at`.
        return supervisor.read_services(base_req).await;
    }
    let deadline = Instant::now() + POST_CAPTURE_WAIT;
    let mut want_seq = svc0.publish_seq.saturating_add(1);
    loop {
        let req = ReadRequirements {
            min_publish_seq: Some(want_seq),
            ..base_req
        };
        match supervisor.read_services(req).await {
            Ok(svc1) => {
                if svc1.captured_at >= capture_done_at {
                    return Ok(svc1);
                }
                want_seq = svc1.publish_seq.saturating_add(1);
            }
            Err(ArmError::Rejected { status: 503, ref code, .. })
                if code == "OBSERVATION_UNAVAILABLE" && Instant::now() < deadline => {}
            Err(e) => return Err(e),
        }
        if Instant::now() >= deadline {
            return Err(ArmError::from(ServiceReadError::SequenceNotReached {
                required: want_seq,
                actual: supervisor.hub().published_seq(),
            }));
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
}

/// One measurement: refuse when the arm moves, trigger the request's pipelines once, refuse when
/// the commanded positions or calibration changed meanwhile, then fit and measure. No lease, no motion.
pub async fn measure(supervisor: &Arc<ArmSupervisor>, source: &dyn DetectionSource, body: &Value) -> Result<Value, ArmError> {
    let req = parse_request(body)?;
    // Mutually exclude detector switches (`POST /api/v1/vision/detector`) for the full capture.
    let _in_flight = crate::vision_detector::try_begin_measure()?;
    let svc0 = supervisor
        .read_services(ReadRequirements {
            require_within_valid_until: true,
            require_prep_env: true,
            ..ReadRequirements::default()
        })
        .await?;
    let snap = snapshot_from_service(&svc0)?;
    match source.per_request_pipelines().await {
        Ok(true) => {}
        Ok(false) => return Err(rejected(422, "UNSUPPORTED", "The vision provider does not take per-request pipelines (perception API 2)")),
        Err(e) => return Err(rejected(503, "DETECTION_UNAVAILABLE", format!("Cannot reach the vision provider: {e}"))),
    }
    let mut frame = match source.trigger(req.pipelines, grasp::TRIGGER_WAIT_MS).await {
        Ok(Some(f)) => f,
        Ok(None) => return Err(rejected(504, "DETECTION_UNAVAILABLE", "The vision provider returned no frame")),
        Err(e) => return Err(rejected(503, "DETECTION_UNAVAILABLE", format!("Trigger failed: {e}"))),
    };
    let capture_done_at = Instant::now();
    let svc1 = read_post_capture_service(supervisor, &svc0, capture_done_at).await?;
    check_driver_feedback(&svc1, Instant::now())?;
    check_still(&svc1)?;
    if svc1.boot != svc0.boot
        || svc1.revisions.calibration != svc0.revisions.calibration
        || svc1.revisions.grasp_setup != svc0.revisions.grasp_setup
        || svc1.revisions.tool_digest_hash != svc0.revisions.tool_digest_hash
    {
        return Err(rejected(
            409,
            "SNAPSHOT_STALE",
            "Arm calibration or configuration changed during the capture",
        ));
    }
    let moved = svc1.motion_generation != svc0.motion_generation
        || svc1
            .snapshot
            .rt
            .commanded_positions
            .iter()
            .zip(&snap.commanded)
            .take(svc1.identity.arm_dof)
            .any(|(a, b)| f64::from((a - b).abs()) > MOVED_RAD);
    if moved {
        return Err(rejected(409, "ARM_MOVING", "The arm moved during the capture"));
    }
    grasp::rescale_frame_to_cam(&mut frame, &snap.cam);
    Ok(reply(&snap, &req, &frame))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_parsing() {
        let r = parse_request(&Value::Null).unwrap();
        assert_eq!((r.label, r.min_score, r.pipelines), (None, 0.3, YOLO_DEPTH));
        let r = parse_request(&json!({"label": "cup", "min_score": 0.5, "pipelines": ["depth"]})).unwrap();
        assert_eq!((r.label.as_deref(), r.min_score, r.pipelines), (Some("cup"), 0.5, DEPTH_ONLY));
        assert_eq!(parse_request(&json!({"pipelines": ["depth", "yolo"]})).unwrap().pipelines, YOLO_DEPTH);
        for bad in [json!({"pipelines": ["yolo"]}), json!({"pipelines": ["aruco", "depth"]}), json!({"min_score": 2}), json!({"label": ""}),
                    json!({"foo": 1}), json!([1]), json!({"pipelines": "depth"})] {
            let e = crate::api::rest::ApiError::from(parse_request(&bad).unwrap_err());
            assert_eq!(e.code.as_deref(), Some("INVALID_COMMAND"), "{bad}");
        }
    }

    #[test]
    fn residuals_of_an_exact_and_an_offset_fit() {
        // 1/z = 2 * disp + 0.5
        let samples: Vec<(f64, f64, f64)> = [0.25, 0.3, 0.4].iter().map(|&z: &f64| (1.0 / z, (1.0 / z - 0.5) / 2.0, z)).collect();
        let r = fit_residuals(&samples, 2.0, 0.5);
        assert_eq!(r["inliers"], 3);
        assert!(r["rms_mm"].as_f64().unwrap() < 1e-9);
        let ls = least_squares(&samples).unwrap();
        assert!((ls["a"].as_f64().unwrap() - 2.0).abs() < 1e-9 && (ls["b"].as_f64().unwrap() - 0.5).abs() < 1e-9);
        assert!((ls["r2_inv_z"].as_f64().unwrap() - 1.0).abs() < 1e-12);
        // a 25 % scale error: every sample out of the 8 % inlier band
        let r = fit_residuals(&samples, 2.0 * 1.25, 0.5);
        assert_eq!(r["inliers"], 0);
        assert!(r["max_abs_mm"].as_f64().unwrap() > 20.0);
        assert!(least_squares(&samples[..1]).is_none());
    }
}
