//! Tap-to-grasp action (`/api/v1/action/grasp`, docs/agent_action_api.md).
//!
//! The user picks a detection box; an async orchestrator (outside the supervisor lock and
//! the control tick) runs: open → coarse approach (box-centre ray hits the table plane) →
//! closed-loop image centring → J6 alignment → descend → close → lift. Every motion is a
//! sub-motion of the one `grasp` action record, installed under the lock only while the
//! grasp is still running, so stop / E-stop / lease loss / supersede end it between or
//! during motions without leaving a half-issued motion.
//!
//! The closed loop drives the target's box centre onto `grip_pixel_norm` (where the table
//! point under the jaw tips appears at approach height). Pixel errors become metric steps
//! through rough pinhole intrinsics and the FK-known camera height, scaled by a
//! conservative gain, so wrong intrinsics or mount angles slow convergence but do not
//! break it.
//!
//! Camera-down acquisition (`acquire_camera_down`, for mounts pitched toward the jaws such
//! as the v8 holder): approach with the optical axis vertical and the camera centre above
//! the estimate, centre the box on the image centre, then one Cartesian move to the
//! tool-vertical pose above the table point of the centred ray (`transition`).
//!
//! Table clearance ([`TABLE_CLEARANCE_M`], [`envelope`]): every sub-motion, in both modes,
//! keeps the TCP, the gripper check points and the camera (lens, body box) above
//! `table_z_m + 0.005` along its path. The approach (and the camera-down transition) is
//! checked before the ack; every sub-motion again before it is installed.
use std::future::Future;
use std::pin::Pin;

pub type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

mod admission;
mod depth;
mod geometry;
mod orchestrator;
mod perception;
mod planning;
mod setup;
mod tasks;

pub use admission::{
    begin, begin_executor, begin_macro_spec, begin_with_stamp, spawn, spawn_with_deadline, start,
};
pub(crate) use admission::begin_executor_ws;

pub use depth::{
    calibrate_depth, calibrate_depth_why, extract_table_obs, measure_detection_depth,
    measure_detection_depth_with_cal, measure_detection_depth_with_cal_why, table_fk_samples,
    DepthCalibration, DepthMeasurement, TableObs,
};
use depth::{metric_table_contradicts_fk, DepthCache};

pub use geometry::{
    acquisition_z, camera_down, camera_down_mode, camera_down_pose, envelope,
    envelope_with_contact, look_down, table_violation, transition_pose, CamModel,
    TABLE_CLEARANCE_M,
};
#[cfg(test)]
use geometry::path_samples;

pub use orchestrator::{
    align_guard, border_free_aim, frames_angle, frames_angle_by, frames_width_px,
    parallax_distance, roll_across_long_side, scale_height, top_face_u, touches_border, yaw_chain,
    Centred, Lateral, Orchestrator, YawOutcome, BORDER_SHIFTS, HEIGHT_PROBE_TOL_M,
    LATERAL_MIN_SHIFT_PX, PROBE_FALLBACK_DZ_M,
};
use orchestrator::run;

pub use perception::{
    box_centre, box_changed_px, grasp_mode, iou, parse_contour, parse_detections,
    parse_gray_frame, parse_gray_frame_with_format, parse_markers, rescale_frame_to_cam,
    CameraHoldTicket, DepthFrameInfo, Detection, DetectionFrame, DetectionSource,
    GrayFormatDescriptor, GrayFrame, GrayFrameError, GrayTriggerResult, HttpDetectionSource,
    MaskFocus, ModeInfo, ProviderModels, ResidentModels, Tracker, VisionMode, GRAY_FRAME_ALIGN,
    GRAY_FRAME_MAGIC, GRAY_MAX_HEADER_BYTES, GRAY_V1_HEIGHT, GRAY_V1_LUMA, GRAY_V1_LUMA_RANGE,
    GRAY_V1_STRIDE, GRAY_V1_WIDTH, GRASP_PIPELINES, MAX_CONTOUR_VERTICES,
};
pub(crate) use perception::TRIGGER_WAIT_MS;
use perception::{f, median, refusal_message};

pub use planning::{
    dry_run, dry_run_task, expected_duration, parse_grasp, plan, prepare, prepare_from_snapshot,
    transition_estimate, DryRun, GraspRequest, PendingAdaptive, PendingGrasp, PlannedAdaptive,
    PlannedGrasp,
};
use planning::{approach_pose, grasp_plane};

pub use setup::{
    commit_setup_update, grasp_width_check_json, grasp_width_check_json_from_published, load_into,
    load_setup, load_setup_for_tool, precheck_setup_update, prepare_setup_update,
    refuse_calibrated_fields, serialize_setup_for_tool, setup_path_from_env, update_setup,
    AcquireCameraDown, GraspDeps, GraspSetup, HeightProbeMode, HeightSource, LoadOutcome, PreOpen,
    PreparedGraspSetup, WidthSource, YawSource, CALIBRATED_FIELDS, CAMERA_DOWN_AUTO_TILT_DEG,
    DEFAULT_SETUP_PATH,
};
pub(crate) use setup::log_grasp_width_check;
use setup::refuse_calibrated_request;

pub use tasks::{
    preflight, DepthReq, GraspTask, ModelKind, TaskModels, TaskRequires, TASK_UNSUPPORTED,
};
use tasks::{task_verdict, valid_task_name};

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::sync::Arc;
    use std::time::{Duration, Instant};
    use serde_json::Value;
    use crate::api::proxy::DetectorUpstream;
    use crate::kinematics::{self, Mat3, Pose, Vec3};
    use crate::macro_action::MacroCtx;
    use crate::model::CameraMount;
    use crate::motion::{self, MotionError};
    use crate::supervisor::{ArmSupervisor, SupervisorInner};
    use super::*;

    fn tasks_setup(tasks: Value) -> Result<GraspSetup, String> { GraspSetup::default().patched(&serde_json::json!({"tasks": tasks})) }

    #[test]
    fn task_registry_validation() {
        let ok = tasks_setup(serde_json::json!({"cup-2_a": {"description": "d", "requires": {"model_kind": "segment", "depth": "any"},
            "overrides": {"yaw_sources": ["mask"]}}, "x": {}})).unwrap();
        assert_eq!(ok.tasks["cup-2_a"].requires, TaskRequires { model_kind: Some(ModelKind::Segment), depth: Some(DepthReq::Any) });
        assert!(ok.tasks["x"].requires.is_empty() && ok.tasks["x"].overrides.is_empty());
        let obb = tasks_setup(serde_json::json!({"part": {"requires": {"model_kind": "obb"}}})).unwrap();
        assert_eq!(obb.tasks["part"].requires.model_kind, Some(ModelKind::Obb));
        assert_eq!(serde_json::to_value(&obb.tasks["part"]).unwrap()["requires"], serde_json::json!({"model_kind": "obb"}));
        let long = "a".repeat(33);
        for (tasks, msg) in [
            (serde_json::json!({"Cup": {}}), "name"),
            (serde_json::json!({"": {}}), "name"),
            (serde_json::json!({long: {}}), "name"),
            (serde_json::json!({"a b": {}}), "name"),
            (serde_json::json!({"a": {"description": "x".repeat(201)}}), "description"),
            (serde_json::json!({"a": {"overrides": {"tasks": {}}}}), "overrides cannot contain tasks"),
            (serde_json::json!({"a": {"overrides": {"bogus": 1}}}), "unknown grasp setup field: bogus"),
            (serde_json::json!({"a": {"overrides": {"grasp_height_m": 5.0}}}), "grasp_height_m"),
            (serde_json::json!({"a": {"overrides": {"yaw_sources": ["mask", "mask"]}}}), "repeat"),
            (serde_json::json!({"a": {"requires": {"model_kind": "pose"}}}), "invalid grasp setup"),
            (serde_json::json!({"a": {"requires": {"depth": "stereo"}}}), "invalid grasp setup"),
            (serde_json::json!({"a": {"requires": {"gpu": true}}}), "invalid grasp setup"),
            (serde_json::json!({"a": {"model": "x"}}), "invalid grasp setup"),
        ] {
            let e = tasks_setup(tasks.clone()).unwrap_err();
            assert!(e.contains(msg), "{tasks}: {e}");
        }
        // a task is checked against the setup it is stored with; a base change that breaks it is rejected
        let s = tasks_setup(serde_json::json!({"low": {"overrides": {"max_grasp_height_m": 0.05}}})).unwrap();
        assert!(GraspSetup::default().patched(&serde_json::json!({"grasp_height_m": 0.055})).is_ok());
        let e = s.patched(&serde_json::json!({"grasp_height_m": 0.055})).unwrap_err();
        assert!(e.starts_with("grasp task low: max_grasp_height_m"), "{e}");
        // replacing the map in the same patch fixes it; `{}` deletes all tasks
        assert!(s.patched(&serde_json::json!({"grasp_height_m": 0.055, "tasks": {}})).unwrap().tasks.is_empty());
        // tasks always serialise (first insertion through a patch of the default)
        assert_eq!(serde_json::to_value(GraspSetup::default()).unwrap()["tasks"], serde_json::json!({}));
        assert_eq!(serde_json::to_value(&s).unwrap()["tasks"], serde_json::json!({"low": {"overrides": {"max_grasp_height_m": 0.05}}}));
    }

    #[test]
    fn grasp_request_applies_base_then_task_then_overrides() {
        let base = tasks_setup(serde_json::json!({
            "t": {"requires": {"depth": "metric"}, "overrides": {"lift_height_m": 0.13, "approach_height_m": 0.2}},
            "low": {"overrides": {"max_grasp_height_m": 0.05}},
        })).unwrap();
        let body = |extra: Value| {
            let mut b = serde_json::json!({"label": "cup", "box_norm": [0.4, 0.4, 0.6, 0.6]});
            for (k, v) in extra.as_object().unwrap() { b[k] = v.clone(); }
            b
        };
        let r = parse_grasp(&body(serde_json::json!({"task": "t", "overrides": {"approach_height_m": 0.18}})), &base).unwrap();
        assert_eq!((r.setup.lift_height_m, r.setup.approach_height_m, r.setup.table_z_m), (0.13, 0.18, base.table_z_m), "request override wins");
        assert_eq!((r.task.as_deref(), r.requires.depth), (Some("t"), Some(DepthReq::Metric)));
        assert!(r.setup.tasks.is_empty(), "the effective setup carries no tasks");
        let r = parse_grasp(&body(serde_json::json!({})), &base).unwrap();
        assert_eq!((r.task, r.requires, r.setup), (None, TaskRequires::default(), base.without_tasks()));
        let err = |extra: Value| match parse_grasp(&body(extra), &base) { Err(MotionError::InvalidCommand(m)) => m, other => panic!("{other:?}") };
        assert_eq!(err(serde_json::json!({"task": "nope"})), "unknown grasp task: nope");
        assert_eq!(err(serde_json::json!({"task": ["t"]})), "task must be a string");
        assert_eq!(err(serde_json::json!({"overrides": {"tasks": {}}})), "tasks cannot be overridden per request");
        // only the final effective setup is validated: the request's grasp height against the task's max
        assert!(err(serde_json::json!({"task": "low", "overrides": {"grasp_height_m": 0.055}})).contains("max_grasp_height_m"));
        assert!(parse_grasp(&body(serde_json::json!({"overrides": {"grasp_height_m": 0.055}})), &base).is_ok(), "no task: the tasks do not constrain overrides");
    }

    #[test]
    fn mode_info_parse_and_task_requirement_check() {
        let legacy = ModeInfo::parse(None);
        assert!(!legacy.reported && legacy.vision == Ok(None));
        let full = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "enabled": true, "per_request_pipelines": true,
            "model": {"kind": "segment"}, "available": {"depth": true}, "depth_model": {"metric": true}})));
        assert_eq!(full, ModeInfo { reported: true, per_request_pipelines: true, yolo_available: true, aruco_available: false, depth_available: true, gray_available: false, gray_format: Ok(None), upstream_loopback: true, depth_metric: true,
            model_kind: Some("segment".into()), vision: Ok(Some(VisionMode { mode: "yolo".into(), enabled: true })), models: None });
        // available.depth is read independently of depth_model.metric
        let depth_off = ModeInfo::parse(Some(&serde_json::json!({"available": {"depth": false}, "depth_model": {"metric": true}})));
        assert!(!depth_off.depth_available && depth_off.depth_metric && depth_off.vision.is_err());
        let relative = ModeInfo { depth_metric: false, model_kind: Some("detect".into()), ..full.clone() };
        let req = |k: Option<ModelKind>, d: Option<DepthReq>| TaskRequires { model_kind: k, depth: d };
        assert_eq!(req(None, None).check("t", &legacy), Ok(()));
        assert_eq!(req(Some(ModelKind::Segment), Some(DepthReq::Metric)).check("t", &full), Ok(()));
        assert_eq!(req(Some(ModelKind::Segment), None).check("t", &relative).unwrap_err(), "task t needs model_kind segment, provider has detect");
        assert_eq!(req(Some(ModelKind::Detect), None).check("t", &relative), Ok(()));
        assert_eq!(req(None, Some(DepthReq::Any)).check("t", &relative), Ok(()));
        assert_eq!(req(None, Some(DepthReq::Metric)).check("t", &relative).unwrap_err(), "task t needs depth metric, provider depth model is not metric");
        assert_eq!(req(None, Some(DepthReq::Any)).check("t", &depth_off).unwrap_err(), "task t needs depth any, provider has no depth pipeline");
        assert_eq!(req(Some(ModelKind::Detect), None).check("t", &ModeInfo { model_kind: None, ..full.clone() }).unwrap_err(),
            "task t needs model_kind detect, provider does not report model.kind");
        assert_eq!(req(Some(ModelKind::Detect), None).check("t", &legacy).unwrap_err(), "task t needs model_kind detect, provider does not report model.kind");
        assert_eq!(req(None, Some(DepthReq::Any)).check("t", &legacy).unwrap_err(), "task t needs depth any, provider does not report depth");
        // detector slot empty: an older provider still reports model.kind "detect", a newer one null
        let mode = |kind: Value| ModeInfo::parse(Some(&serde_json::json!({"mode": "aruco", "per_request_pipelines": true,
            "available": {"yolo": false, "aruco": true, "depth": true}, "model": {"name": "", "kind": kind}})));
        for empty in [mode(serde_json::json!("detect")), mode(Value::Null)] {
            assert!(!empty.yolo_available);
            assert_eq!(req(Some(ModelKind::Detect), None).check("t", &empty).unwrap_err(), "task t needs model_kind detect, provider has no detector loaded");
            assert_eq!(req(None, Some(DepthReq::Any)).check("t", &empty), Ok(()), "depth alone needs no detector");
        }
        assert!(full.yolo_available, "available.yolo not reported: a detector is assumed");
        // obb: an oriented box model satisfies obb only; segment stays the segmentation model
        let obb = ModeInfo { model_kind: Some("obb".into()), ..full.clone() };
        assert_eq!(req(Some(ModelKind::Obb), None).check("t", &obb), Ok(()));
        assert_eq!(req(Some(ModelKind::Segment), None).check("t", &obb).unwrap_err(), "task t needs model_kind segment, provider has obb");
        assert_eq!(req(Some(ModelKind::Obb), None).check("t", &full).unwrap_err(), "task t needs model_kind obb, provider has segment");
    }

    #[test]
    fn task_models_are_tri_state_per_slot() {
        let s = tasks_setup(serde_json::json!({
            "a": {"models": {"detector": "seg"}}, "b": {"models": {"depth": null}}, "c": {"models": {"detector": null, "depth": "zd"}}})).unwrap();
        let m = |t: &str| s.tasks[t].models.clone().unwrap();
        assert_eq!(m("a"), TaskModels { detector: Some(Some("seg".into())), depth: None });
        assert_eq!(m("b"), TaskModels { detector: None, depth: Some(None) });
        assert_eq!(m("c"), TaskModels { detector: Some(None), depth: Some(Some("zd".into())) });
        // round trip: absent stays absent, null stays null
        assert_eq!(serde_json::to_value(&s.tasks["b"]).unwrap(), serde_json::json!({"overrides": {}, "models": {"depth": null}}));
        for bad in [serde_json::json!({"models": {}}), serde_json::json!({"models": {"detector": ""}}),
                    serde_json::json!({"models": {"detector": "x".repeat(65)}}), serde_json::json!({"models": {"gpu": "x"}}),
                    serde_json::json!({"models": {"detector": 3}})] {
            assert!(tasks_setup(serde_json::json!({"a": bad.clone()})).is_err(), "{bad}");
        }
        // the switch body names every slot of the task when the resident set differs
        let mode = |det: Option<&str>, depth: Option<&str>| ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo",
            "models": {"generation": 3, "swap": {"state": "idle", "generation": 3}, "resident": {"detector": det, "depth": depth},
                       "catalogue": [{"name": "seg", "kind": "segment", "ion_mb": 19.5}, {"name": "zd", "kind": "depth", "ion_mb": 20.5},
                                     {"name": "det", "kind": "detect"}]},
            "ion": {"base_mb": 21.5, "used_mb": 41.0, "total_mb": 62.9, "reserve_mb": 8.0}})));
        assert_eq!(m("a").switch_body(&mode(Some("seg"), Some("zd"))), None);
        assert_eq!(m("a").switch_body(&mode(Some("det"), None)), Some(serde_json::json!({"detector": "seg"})));
        assert_eq!(m("c").switch_body(&mode(Some("det"), None)), Some(serde_json::json!({"detector": null, "depth": "zd"})));
        assert_eq!(m("b").switch_body(&mode(Some("det"), None)), None);
        assert!(m("b").switch_body(&ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo"})))).is_some(), "no models: always switch");
        // provider state and the catalogue budget
        let pm = mode(Some("det"), None).models.unwrap();
        assert_eq!((pm.generation, pm.swap_state.as_str(), pm.ion), (3, "idle", Some([21.5, 62.9, 8.0])));
        assert_eq!(pm.catalogue.get("det"), Some(&None));
        let set = |d: &str, z: Option<&str>| ResidentModels { detector: Some(d.into()), depth: z.map(str::to_string) };
        assert_eq!(pm.budget(&set("seg", None)), Some(Ok(())));
        assert_eq!(pm.budget(&set("seg", Some("zd"))), Some(Err("ION_BUDGET (need 69.5 MB, limit 62.9 MB)".into())));
        assert_eq!(pm.budget(&set("det", Some("zd"))), None, "det has no ion_mb");
        let req = TaskRequires { model_kind: Some(ModelKind::Segment), depth: None };
        assert_eq!(task_verdict("a", &req, Some(&m("a")), &mode(Some("det"), Some("zd"))),
                   (false, Some("task a: model switch refused: ION_BUDGET (need 69.5 MB, limit 62.9 MB)".into())));
        assert_eq!(task_verdict("a", &req, Some(&m("a")), &mode(Some("det"), None)), (true, Some("needs model switch".into())));
        assert!(!task_verdict("a", &req, Some(&m("a")), &mode(Some("seg"), None)).0, "model.kind not reported");
    }

    #[test]
    fn patch_rejects_unknown_and_invalid_fields() {
        let s = GraspSetup::default();
        assert!(s.validate().is_ok());
        assert!(s.patched(&serde_json::json!({"nope": 1})).is_err());
        assert!(s.patched(&serde_json::json!({"grasp_height_m": 0.5})).is_err());
        let p = s.patched(&serde_json::json!({"table_z_m": 0.02, "grip_pixel_norm": [0.4, 0.7]})).unwrap();
        assert_eq!((p.table_z_m, p.grip_pixel_norm), (0.02, [0.4, 0.7]));
    }

    #[test]
    fn height_source_settings_default_validate_and_patch() {
        let d = GraspSetup::default();
        assert_eq!((d.height_sources.as_slice(), d.height_record.as_slice()), ([HeightSource::Probe].as_slice(), [HeightSource::Depth].as_slice()));
        assert!(d.height_priors.is_empty());
        // a setup file without the keys gets the defaults
        let old: GraspSetup = serde_json::from_value(serde_json::json!({"table_z_m": 0.01})).unwrap();
        assert_eq!((old.height_sources, old.height_record, old.height_priors), (d.height_sources.clone(), d.height_record.clone(), BTreeMap::new()));
        let p = d.patched(&serde_json::json!({"height_sources": ["depth", "probe", "prior"], "height_record": ["prior"], "height_priors": {"cup": 0.09}})).unwrap();
        assert_eq!(p.height_sources, [HeightSource::Depth, HeightSource::Probe, HeightSource::Prior]);
        assert_eq!(p.height_priors.get("cup"), Some(&0.09));
        // the default height_record may name a deciding source: it decides instead
        assert!(d.patched(&serde_json::json!({"height_sources": ["depth", "probe"]})).is_ok());
        for bad in [
            serde_json::json!({"height_sources": ["probe", "probe"]}),
            serde_json::json!({"height_record": ["depth", "depth"]}),
            serde_json::json!({"height_record": ["probe"]}),
            serde_json::json!({"height_sources": ["scale"]}),
            serde_json::json!({"height_priors": {"cup": 0.0}}),
            serde_json::json!({"height_priors": {"cup": 0.6}}),
            serde_json::json!({"height_priors": {"cup": 0.05, "Cup": 0.06}}),
        ] {
            assert!(d.patched(&bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn parses_obb_v2_detections() {
        let raw = serde_json::json!([{
            "class_id": 39,
            "format": "xywh_center",
            "class_name": "bottle",
            "confidence": 0.88,
            "x": 0.45,
            "y": 0.55,
            "w": 0.12,
            "h": 0.24,
            "obb_corners": [
                [0.40, 0.45],
                [0.50, 0.45],
                [0.50, 0.65],
                [0.40, 0.65]
            ],
            "angle_deg": -25.5,
            "angle_conf": 0.85,
            "grasp_width_px": 28.5,
            "length_px": 72.0
        }]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 1);
        let det = &d[0];
        assert_eq!(det.label, "bottle");
        assert!((det.score - 0.88).abs() < 1e-6);
        assert!((det.box_norm[0] - 0.39).abs() < 1e-6);
        assert!((det.box_norm[1] - 0.43).abs() < 1e-6);
        assert!((det.box_norm[2] - 0.51).abs() < 1e-6);
        assert!((det.box_norm[3] - 0.67).abs() < 1e-6);
        assert_eq!(det.angle_deg, Some(-25.5));
        assert_eq!(det.angle_conf, Some(0.85));
        assert_eq!(det.grasp_width_px, Some(28.5));
        assert_eq!(det.length_px, Some(72.0));
    }

    #[test]
    fn parses_detector_formats() {
        let raw = serde_json::json!([
            [320.0, 180.0, 64.0, 36.0, 0.9, "cup"],
            {"label": "bottle", "score": 0.8, "format": "xyxy", "bbox": [0.1, 0.2, 0.3, 0.4]},
            {"label": "x", "score": 0.5, "bbox": [1, 2, 3, 4]},
            {"label": "mouse", "score": 0.7, "box_norm": [0.5, 0.5, 0.6, 0.7]},
            {"class_id": 39, "class_name": "bottle", "confidence": 0.71, "x": 0.42, "y": 0.55, "w": 0.08, "h": 0.2}
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 4, "{d:?}");
        assert_eq!(d[3].label, "bottle", "rebot-vision 1.2 detections carry no format");
        assert!(d[3].box_norm.iter().zip([0.38, 0.45, 0.46, 0.65]).all(|(a, b)| (a - b).abs() < 1e-9), "{:?}", d[3].box_norm);
        assert_eq!(d[0].label, "cup");
        assert!((d[0].box_norm[0] - 0.45).abs() < 1e-9 && (d[0].box_norm[3] - 0.55).abs() < 1e-9);
        assert!(d[1].box_norm.iter().zip([0.1, 0.2, 0.3, 0.4]).all(|(a, b)| (a - b).abs() < 1e-9));
        assert_eq!(d[2].label, "mouse");
    }

    fn det(b: [f64; 4], angle: Option<(f64, f64)>) -> Detection {
        Detection { label: "cup".into(), score: 0.9, raw_score_valid: true, box_norm: b, angle_deg: angle.map(|a| a.0), angle_conf: angle.map(|a| a.1), grasp_width_px: None, mask_focus: false, disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None, depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None }
    }

    #[test]
    fn test_box_changed_and_iou() {
        let size = [640.0, 360.0];
        let b0 = [0.1, 0.1, 0.3, 0.3];
        // small shift <= 5 px (4 px in x: 4/640 = 0.00625)
        let b_small = [0.1 + 4.0 / 640.0, 0.1, 0.3 + 4.0 / 640.0, 0.3];
        assert!(!box_changed_px(&b0, &b_small, size));
        // large shift > 5 px (6 px in x: 6/640 = 0.009375)
        let b_large = [0.1 + 6.0 / 640.0, 0.1, 0.3 + 6.0 / 640.0, 0.3];
        assert!(box_changed_px(&b0, &b_large, size));

        // IoU matching unit test
        let b1 = [0.0, 0.0, 1.0, 1.0];
        let b2 = [0.5, 0.0, 1.5, 1.0];
        // intersection: 0.5 * 1.0 = 0.5; union: 1.0 + 1.0 - 0.5 = 1.5 -> iou = 1/3
        assert!((iou(&b1, &b2) - 1.0 / 3.0).abs() < 1e-6);
        let b_disjoint = [2.0, 2.0, 3.0, 3.0];
        assert_eq!(iou(&b1, &b_disjoint), 0.0);
    }

    #[test]
    fn align_guard_skips_border_and_occluded_boxes() {
        let size = [640.0, 360.0];
        let b = [0.3, 0.3, 0.5, 0.6];
        let frames = vec![det(b, None), det([0.31, 0.3, 0.51, 0.6], None), det(b, None)];
        assert_eq!(align_guard(&det(b, None), &frames, size), Ok(()));
        // an edge within 2 px of the border (left, bottom)
        assert_eq!(align_guard(&det([0.002, 0.3, 0.2, 0.6], None), &frames, size), Err("border"));
        assert_eq!(align_guard(&det([0.3, 0.3, 0.5, 0.995], None), &frames, size), Err("border"));
        // the last box shrank to half its height (jaws in front of it): area -50 %
        let shrunk = det([0.3, 0.3, 0.5, 0.45], None);
        let mut with_last = frames.clone();
        with_last.push(shrunk.clone());
        assert_eq!(align_guard(&shrunk, &with_last, size), Err("occluded"));
        // -30 % passes
        assert_eq!(align_guard(&det([0.3, 0.3, 0.5, 0.51], None), &frames, size), Ok(()));
    }

    #[test]
    fn border_free_aim_moves_a_cut_axis_inside() {
        let size = [640.0, 360.0];
        let n = |b: [f64; 4]| det([b[0] / 640.0, b[1] / 360.0, b[2] / 640.0, b[3] / 360.0], None);
        // uncut: no new aim
        assert_eq!(border_free_aim(&[n([280.0, 200.0, 360.0, 280.0])], [320.0, 240.0], size, 14.0, 0), None);
        // cut at the bottom, 80 px wide: v so that 1.25 x 80 px fits 14 px above the bottom, u kept
        let a = border_free_aim(&[n([280.0, 320.0, 360.0, 360.0])], [320.0, 340.0], size, 14.0, 0).unwrap();
        assert_eq!(a[0], 320.0);
        assert!((a[1] - (360.0 - 14.0 - 50.0)).abs() < 1e-9, "{a:?}");
        // the estimate grows with the attempt; the visible extent counts when it is the larger
        let b = border_free_aim(&[n([280.0, 200.0, 360.0, 360.0])], [320.0, 340.0], size, 14.0, 1).unwrap();
        assert!((b[1] - (360.0 - 14.0 - 160.0 * 1.5 / 2.0)).abs() < 1e-9, "{b:?}");
        // cut at the left: u moves right
        let c = border_free_aim(&[n([0.0, 150.0, 40.0, 210.0])], [10.0, 180.0], size, 14.0, 0).unwrap();
        assert!((c[0] - (14.0 + 75.0 / 2.0)).abs() < 1e-9 && c[1] == 180.0, "{c:?}");
        // cut on both sides of an axis, or too large to fit: none
        assert_eq!(border_free_aim(&[n([100.0, 0.0, 200.0, 360.0])], [150.0, 180.0], size, 14.0, 0), None);
        assert_eq!(border_free_aim(&[n([100.0, 200.0, 400.0, 360.0])], [250.0, 340.0], size, 14.0, 0), None);
        // the median edge decides: one cut frame of three does not
        let frames = [n([280.0, 200.0, 360.0, 300.0]), n([280.0, 200.0, 360.0, 301.0]), n([280.0, 200.0, 360.0, 359.0])];
        assert_eq!(border_free_aim(&frames, [320.0, 250.0], size, 14.0, 0), None);
    }

    #[test]
    fn tracker_widens_the_gate_for_a_box_at_the_border() {
        let setup = GraspSetup::default();
        let frame = |b: [f64; 4]| {
            let now = Instant::now();
            DetectionFrame { frame_id: 1, captured_at: now, policy_captured_at: Some(now), image_size_px: [640.0, 360.0], detections: vec![Detection { label: "cube".into(), ..det(b, None) }], depth: None, model_generation: None }
        };
        // predicted cut box (v 0.9..1.0) after a step; the box seen whole is 96 px higher
        let seen = frame([0.4, 300.0 / 360.0 - 0.3, 0.5, 300.0 / 360.0]);
        let cut = Tracker { label: "cube".into(), predicted: [0.4, 0.9, 0.5, 1.0], gate_px: setup.match_dist_px };
        assert!(cut.find(&seen, &setup).is_some());
        let uncut = Tracker { predicted: [0.4, 0.8, 0.5, 0.99], ..cut };
        assert!(uncut.find(&seen, &setup).is_none());
    }

    #[test]
    fn scale_height_rejects_noisy_sizes() {
        // lens 0.13 m above the top, 0.04 m up: sizes scale by 0.13 / 0.17
        let (s_low, s_high) = (100.0, 100.0 * 0.13 / 0.17);
        let d = scale_height(&[s_low; 3], &[s_high; 3], 0.04).unwrap();
        assert!((d - 0.13).abs() < 1e-9, "{d}");
        // 5 % size jitter: several cm of height uncertainty
        let e = scale_height(&[s_low * 0.95, s_low, s_low * 1.05], &[s_high * 1.05, s_high * 0.95, s_high], 0.04).unwrap_err();
        assert!(e.starts_with("scale_noise:"), "{e}");
        // 0.3 % jitter passes
        assert!(scale_height(&[s_low * 0.997, s_low, s_low * 1.003], &[s_high, s_high * 1.003, s_high], 0.04).is_ok());
        assert!(scale_height(&[s_low], &[s_low / 1.01], 0.04).unwrap_err().starts_with("scale_change:"));
    }

    #[test]
    fn parallax_distance_and_its_noise_gate() {
        // lens 0.13 m above the top, 0.04 m baseline, fx 600: the centre moves by 600 · 0.04 / 0.13 px
        let du = 600.0 * 0.04 / 0.13;
        let (un, up) = (320.0 + du / 2.0, 320.0 - du / 2.0);
        let d = parallax_distance(&[un; 3], &[up; 3], 600.0, 0.04).unwrap();
        assert!((d - 0.13).abs() < 1e-9, "{d}");
        // either direction of the shift and of the baseline gives the same distance
        assert!((parallax_distance(&[up; 3], &[un; 3], 600.0, -0.04).unwrap() - 0.13).abs() < 1e-9);
        // 3 px centre jitter: dD = 0.13 / 185 · √(3² + 3²) ≈ 3 mm, accepted
        assert!(parallax_distance(&[un - 3.0, un, un + 3.0], &[up + 3.0, up, up - 3.0], 600.0, 0.04).is_ok());
        // 10 px jitter on a 1 cm baseline (46 px shift): dD ≈ 40 mm, rejected
        let du1 = 600.0 * 0.01 / 0.13;
        let e = parallax_distance(&[320.0 + du1 / 2.0 - 10.0, 320.0 + du1 / 2.0 + 10.0], &[320.0 - du1 / 2.0 - 10.0, 320.0 - du1 / 2.0 + 10.0], 600.0, 0.01).unwrap_err();
        assert!(e.starts_with("parallax_noise:"), "{e}");
        assert!(parallax_distance(&[320.0], &[318.0], 600.0, 0.04).unwrap_err().starts_with("parallax_shift:"));
        assert!(parallax_distance(&[un], &[up], 600.0, 0.0005).unwrap_err().starts_with("parallax_baseline:"));
        assert_eq!(parallax_distance(&[], &[up], 600.0, 0.04).unwrap_err(), "no_frames");
    }

    #[test]
    fn top_face_feature_for_the_parallax() {
        let cx = 320.0;
        // straddling the principal point at both poses: the centre
        let (n, p, f) = top_face_u(&[[250.0, 0.0, 410.0, 1.0]], &[[200.0, 0.0, 360.0, 1.0]], cx).unwrap();
        assert_eq!((n, p, f), (vec![330.0], vec![280.0], "centre"));
        // left of it at one pose: the left edge (top face) in both
        let (n, p, f) = top_face_u(&[[200.0, 0.0, 330.0, 1.0]], &[[100.0, 0.0, 250.0, 1.0]], cx).unwrap();
        assert_eq!((n, p, f), (vec![200.0], vec![100.0], "left edge"));
        let (n, p, f) = top_face_u(&[[400.0, 0.0, 500.0, 1.0]], &[[300.0, 0.0, 420.0, 1.0]], cx).unwrap();
        assert_eq!((n, p, f), (vec![500.0], vec![420.0], "right edge"));
        // entirely right at one pose and entirely left at the other: no common top-face edge
        assert!(top_face_u(&[[330.0, 0.0, 380.0, 1.0]], &[[250.0, 0.0, 300.0, 1.0]], cx).is_none());
    }

    /// The former downward probe's estimator (lens-to-top distance at the start pose from the
    /// start sizes and the sizes `dz` closer), kept here only for the comparison below.
    fn down_probe_distance(start: &[f64], near: &[f64], dz: f64) -> f64 {
        let r = median(&mut near.to_vec()) / median(&mut start.to_vec());
        dz * r / (r - 1.0)
    }

    /// Same synthetic noisy sizes (Gaussian size jitter, 3 frames per pose) for the up probe
    /// (0.06 / 0.08 / 0.10 m) and the former down probe (0.04 m): distance errors without the
    /// noise gate, and how many estimates the gate accepts.
    #[test]
    fn up_and_down_probe_estimates_under_size_noise() {
        let mut seed = 12345u64;
        let mut gauss = || {
            let mut u = || { seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407); ((seed >> 11) as f64 + 0.5) / (1u64 << 53) as f64 };
            let (a, b) = (u(), u());
            (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
        };
        let n = 2000;
        let stat = |v: &mut Vec<f64>| { v.sort_by(|a, b| a.total_cmp(b)); (v.iter().sum::<f64>() / v.len() as f64, v[v.len() / 2], v[v.len() - 1]) };
        println!("probe     D (m)  jitter  mean / median / max error (mm)  accepted");
        for d in [0.13, 0.20] {
            for jitter in [0.01, 0.03, 0.05] {
                let mut medians = Vec::new();
                for (dir, dz) in [("down", 0.04), ("up", 0.06), ("up", 0.08), ("up", 0.10)] {
                    let (mut err, mut ok) = (Vec::new(), 0);
                    for _ in 0..n {
                        let mut frames = |dist: f64| -> Vec<f64> { (0..3).map(|_| 100.0 / dist * (1.0 + jitter * gauss())).collect() };
                        let start = frames(d);
                        let (est, accepted) = if dir == "down" {
                            let near = frames(d - dz);
                            (down_probe_distance(&start, &near, dz), scale_height(&near, &start, dz).is_ok())
                        } else {
                            let high = frames(d + dz);
                            (dz / (median(&mut start.clone()) / median(&mut high.clone()) - 1.0), scale_height(&start, &high, dz).is_ok())
                        };
                        err.push((est - d).abs());
                        if accepted { ok += 1; }
                    }
                    let (m, med, max) = stat(&mut err);
                    medians.push(med);
                    println!("{dir:>4} {dz:.2}  {d:.2}   {:>3.0} %   {:>6.1} / {:>6.1} / {:>7.1}            {ok:>4}/{n}", jitter * 100.0, m * 1e3, med * 1e3, max * 1e3);
                }
                // a longer raise reduces the error of the up probe
                assert!(medians[3] < medians[1], "{medians:?}");
            }
        }
    }

    #[test]
    fn detector_angle_parsing_and_doubled_mean() {
        let raw = serde_json::json!([{"label": "pen", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2], "angle_deg": 120.0, "angle_conf": 1.4},
                                     {"label": "pen", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2]}]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!((d[0].angle_deg, d[0].angle_conf), (Some(-60.0), Some(1.0)));
        assert_eq!((d[1].angle_deg, d[1].angle_conf), (None, None));
        // 89 and -89 are 2 deg apart: mean at the wrap, not 0
        let fr = [det([0.0; 4], Some((89.0, 0.9))), det([0.0; 4], Some((-89.0, 0.9))), det([0.0; 4], Some((10.0, 0.1)))];
        let (a, spread) = frames_angle(&fr, 0.3).unwrap();
        assert!(a.abs() > 89.9, "{a}");
        assert!(spread < 2.0, "spread for 89 and -89 should be small: {spread}");
        assert_eq!(frames_angle(&fr[2..], 0.3), None, "below align_min_angle_conf");

        // dispersion units: identical -> 0, ±40° -> ~40°, one frame -> 0
        let fr_ident = [det([0.0; 4], Some((30.0, 0.9))), det([0.0; 4], Some((30.0, 0.9)))];
        let (a_id, sp_id) = frames_angle(&fr_ident, 0.3).unwrap();
        assert!((a_id - 30.0).abs() < 1e-6);
        assert!(sp_id.abs() < 1e-6);

        let fr_pm = [det([0.0; 4], Some((40.0, 0.9))), det([0.0; 4], Some((-40.0, 0.9)))];
        let (a_pm, sp_pm) = frames_angle(&fr_pm, 0.3).unwrap();
        assert!(a_pm.abs() < 1e-6);
        assert!((sp_pm - 53.6).abs() < 1.0, "spread was {sp_pm}");

        let fr_single = [det([0.0; 4], Some((40.0, 0.9)))];
        let (_, sp_single) = frames_angle(&fr_single, 0.3).unwrap();
        assert_eq!(sp_single, 0.0);
        // the roll turns the jaw by -t about z
        let t = roll_across_long_side([0.0, 1.0, 0.0], [1.0, 0.0, 0.0]);
        assert!(t.abs() < 1e-12, "jaw already across a long side along x: {t}");
        let t = roll_across_long_side([1.0, 0.0, 0.0], [30f64.to_radians().cos(), 30f64.to_radians().sin(), 0.0]);
        assert!((t - (-120f64 + 180.0).to_radians()).abs() < 1e-9, "{}", t.to_degrees());
        assert!(GraspSetup::default().patched(&serde_json::json!({"align_min_angle_conf": 1.5})).is_err());
    }

    #[test]
    #[allow(clippy::approx_constant)]
    fn ray_and_projection_round_trip() {
        let setup = GraspSetup { camera_tilt_deg: 20.0, ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.05, 0.2]);
        assert!(kinematics::column(&pose.r, 0)[2] < -0.999999);
        let p = cam.plane_point(&pose, [300.0, 200.0], 0.01).unwrap();
        let px = cam.project(&pose, &p).unwrap();
        assert!((px[0] - 300.0).abs() < 1e-6 && (px[1] - 200.0).abs() < 1e-6, "{px:?}");
        // a ray pointing above the horizon never meets the table (tool axis up, optical axis
        // 20 deg off it)
        let up = Pose::new(kinematics::axis_r(&[0.0, 1.0, 0.0], -std::f64::consts::FRAC_PI_2), pose.p);
        assert!(cam.plane_point(&up, [320.0, 180.0], 0.0).is_none());
    }

    /// v8 holder: rot 0, tilt -38, lens centre behind and above the jaw tips.
    fn v8() -> (GraspSetup, CameraMount, CamModel) {
        let setup = GraspSetup { camera_tilt_deg: -38.0, ..Default::default() };
        let mount = CameraMount { rot_deg: 0.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
        (setup, mount, cam)
    }

    /// End rotation at the inspect preset (joint angles, J6 at its rest angle 0).
    fn inspect_r() -> Mat3 { crate::chain::ArmModel::REBOT_DEV_ARM.fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 0.0]).r }

    fn close(a: &Vec3, b: &Vec3, tol: f64) -> bool { kinematics::norm(&kinematics::sub(a, b)) < tol }

    #[test]
    fn camera_down_mode_follows_the_setting_tilt_and_lens_centre() {
        let (setup, mount, _) = v8();
        assert_eq!(camera_down_mode(&setup, &mount), Ok(true), "auto, tilted, lens centre known");
        let no_offset = CameraMount { offset_m: None, ..mount.clone() };
        assert_eq!(camera_down_mode(&setup, &no_offset), Ok(false), "auto without a lens centre");
        for tilt in [0.0, 4.9, -5.0] {
            assert_eq!(camera_down_mode(&GraspSetup { camera_tilt_deg: tilt, ..setup.clone() }, &mount), Ok(false), "auto at tilt {tilt}");
        }
        assert_eq!(camera_down_mode(&GraspSetup { camera_tilt_deg: -5.1, ..setup.clone() }, &mount), Ok(true));
        let off = GraspSetup { acquire_camera_down: AcquireCameraDown::Off, ..setup.clone() };
        assert_eq!(camera_down_mode(&off, &mount), Ok(false));
        let on = GraspSetup { acquire_camera_down: AcquireCameraDown::On, camera_tilt_deg: 0.0, ..setup.clone() };
        assert_eq!(camera_down_mode(&on, &mount), Ok(true));
        assert!(camera_down_mode(&on, &no_offset).is_err(), "on needs the lens centre");
        // setup field: lowercase strings, default auto, other values rejected
        let d = GraspSetup::default();
        assert_eq!(serde_json::to_value(&d).unwrap()["acquire_camera_down"], "auto");
        assert_eq!(d.patched(&serde_json::json!({"acquire_camera_down": "on"})).unwrap().acquire_camera_down, AcquireCameraDown::On);
        assert_eq!((d.height_probe_mode, d.lateral_probe_m), (HeightProbeMode::Up, 0.04));
        assert_eq!(d.patched(&serde_json::json!({"height_probe_mode": "lateral"})).unwrap().height_probe_mode, HeightProbeMode::Lateral);
        assert!(d.patched(&serde_json::json!({"height_probe_mode": "sideways"})).is_err());
        for (b, ok) in [(0.009, false), (0.01, true), (0.08, true), (0.081, false)] {
            assert_eq!(d.patched(&serde_json::json!({"lateral_probe_m": b})).is_ok(), ok, "{b}");
        }
        assert!(d.patched(&serde_json::json!({"recheck_tol_px": 0.0})).is_err() && d.patched(&serde_json::json!({"recheck_tol_px": 501.0})).is_err());
        assert_eq!(d.patched(&serde_json::json!({"recheck_tol_px": 25.0})).unwrap().recheck_tol_px, 25.0);
        for bad in [serde_json::json!("yes"), serde_json::json!(true), serde_json::json!(1)] {
            assert!(d.patched(&serde_json::json!({"acquire_camera_down": bad})).is_err(), "{bad}");
        }
    }

    #[test]
    fn camera_down_pose_puts_the_optical_axis_down_over_the_target() {
        let (setup, _, cam) = v8();
        let target = [0.28, -0.04, setup.table_z_m + setup.grasp_height_m];
        let r_now = inspect_r();
        let pose = camera_down_pose(&r_now, &target, &setup, &cam, &[]);
        let rc = cam.rotation(&pose);
        assert!(kinematics::column(&rc, 2)[2] < -1.0 + 1e-12, "optical axis down: {:?}", kinematics::column(&rc, 2));
        let o = cam.origin(&pose);
        assert!((o[0] - target[0]).abs() < 1e-12 && (o[1] - target[1]).abs() < 1e-12, "camera centre over the target: {o:?}");
        assert!((pose.p[2] - (setup.table_z_m + setup.approach_height_m)).abs() < 1e-12, "TCP at approach height");
        // the tool is pitched by the mount tilt about x_cam; heading (x_cam) kept
        let tool_from_down = kinematics::column(&pose.r, 0)[2].clamp(-1.0, 1.0).acos();
        assert!((std::f64::consts::PI - tool_from_down - 38f64.to_radians()).abs() < 1e-9, "tool pitched by 38 deg");
        let v = look_down(&r_now);
        let x_cam_v = kinematics::mat_vec(&v, &kinematics::column(&cam.rec, 0));
        assert!(close(&kinematics::column(&rc, 0), &x_cam_v, 1e-9), "pitch about x_cam");
        assert!(kinematics::angle_between(&look_down(&pose.r), &v) < 1e-9, "look_down undoes it");
        // the image centre ray meets the plane straight below the camera
        let hit = cam.plane_point(&pose, [cam.cx, cam.cy], target[2]).unwrap();
        assert!(close(&hit, &target, 1e-9), "{hit:?}");
        // tilt 0: the optical axis already is the tool axis, the orientation is look_down's
        let flat = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &GraspSetup::default(), &CameraMount::default());
        assert_eq!(camera_down(&v, &flat.rec), v);
    }

    #[test]
    fn tilted_ray_meets_the_table_ahead_by_the_tilt() {
        // tool vertical, camera tilted 38 deg: the image centre ray meets the plane h·tan(38)
        // from the point below the lens centre
        let (setup, _, cam) = v8();
        let pose = Pose::new(look_down(&inspect_r()), [0.25, 0.0, 0.15]);
        let o = cam.origin(&pose);
        let plane = setup.table_z_m + setup.grasp_height_m;
        let hit = cam.plane_point(&pose, [cam.cx, cam.cy], plane).unwrap();
        let ahead = (hit[0] - o[0]).hypot(hit[1] - o[1]);
        assert!((ahead - (o[2] - plane) * 38f64.to_radians().tan()).abs() < 1e-9, "ahead {ahead}");
        let px = cam.project(&pose, &hit).unwrap();
        assert!((px[0] - cam.cx).abs() < 1e-6 && (px[1] - cam.cy).abs() < 1e-6);
        // the table point under the jaw tips is outside the image at approach height (v < 0)
        let under = cam.project(&pose, &[pose.p[0], pose.p[1], setup.table_z_m]).map(|p| cam.to_norm(p));
        assert!(under.is_none_or(|n| !(0.0..=1.0).contains(&n[1])), "{under:?}");
    }

    #[test]
    fn acquisition_height_keeps_the_pitched_envelope_clear() {
        // rot 90, tilt -38: pitch about end_link z, the axis along which the kit check points
        // are 92.5 mm out
        let setup = GraspSetup { camera_tilt_deg: -38.0, table_z_m: 0.10, approach_height_m: 0.035, ..Default::default() };
        let mount = CameraMount { rot_deg: 90.0, tilt_deg: -38.0, offset_m: Some([-0.0731, 0.0005, 0.066]), ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
        let pts = envelope(crate::model::ArmConfig::rebot_dm().workspace.as_ref(), &mount, &cam.rec);
        assert_eq!(pts.len(), 18, "TCP, 8 check points, lens centre, 8 body corners");
        let target = [0.28, 0.0, setup.table_z_m + setup.grasp_height_m];
        let low = camera_down_pose(&inspect_r(), &target, &setup, &cam, &[]);
        assert!((low.p[2] - 0.135).abs() < 1e-12, "TCP only: approach height");
        let bound = setup.table_z_m + TABLE_CLEARANCE_M;
        let min_z = |pose: &Pose| pts.iter().map(|p| pose.transform_point(p)[2]).fold(f64::INFINITY, f64::min);
        assert!(min_z(&low) < setup.table_z_m - 0.02, "unraised: {:.4}", min_z(&low));
        let pose = camera_down_pose(&inspect_r(), &target, &setup, &cam, &pts);
        assert!(pose.p[2] > low.p[2] + 0.02 && (min_z(&pose) - bound).abs() < 1e-3, "raised to the bound: {:.4}", min_z(&pose));
        let tool = approach_pose(&pose.r, &target, &setup);
        assert!(path_samples(&pose, &tool).iter().all(|(_, p)| min_z(p) >= bound - 1e-3), "transition clear");
        assert!(table_violation(&pose, &tool, &pts, setup.table_z_m).is_none());
        let from = Pose::new(pose.r, [pose.p[0], pose.p[1], pose.p[2] + 0.1]);
        assert!(table_violation(&from, &low, &pts, setup.table_z_m).is_some(), "a path into the table is reported");
        // a path from below the bound may move sideways but not lower
        let side = Pose::new(low.r, [low.p[0] + 0.01, low.p[1], low.p[2]]);
        assert!(table_violation(&low, &side, &pts, setup.table_z_m).is_none());
    }

    #[test]
    fn below_clearance_exception_is_per_point() {
        // two points 0.1 m either side of the TCP along x, TCP 4 mm above the table. A roll
        // about y from -5.73 to +5.16 deg swaps which point is lowest: -x starts lowest at
        // -6 mm (below the table), +x starts at 14 mm and ends at -5 mm, through the table but
        // above the lowest start height. Only -x may sit below the bound, +x must stay above it.
        let table_z = 0.0;
        let pts = [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [-0.1, 0.0, 0.0]];
        let roll = |deg: f64| kinematics::exp_r(&[0.0, deg.to_radians(), 0.0]);
        let a = (0.01f64 / 0.1).asin().to_degrees();
        let from = Pose::new(roll(-a), [0.0, 0.0, 0.004]);
        let to = Pose::new(roll(0.9 * a), [0.0, 0.0, 0.004]);
        let z = |pose: &Pose, i: usize| pose.transform_point(&pts[i])[2];
        let bound = table_z + TABLE_CLEARANCE_M;
        assert!(z(&from, 2) < table_z && z(&from, 1) > bound, "-x starts lowest, below the table; +x above the bound");
        assert!(z(&to, 1) < table_z && z(&to, 1) > z(&from, 2) + 1e-4, "+x ends in the table, above the lowest start");
        assert!(path_samples(&from, &to).iter().all(|(_, p)| (0..3).all(|i| z(p, i) >= z(&from, 2) - 1e-6)),
            "no sample goes below the lowest start height, so a single-minimum check accepts the path");
        match table_violation(&from, &to, &pts, table_z) {
            Some(MotionError::WorkspaceViolation { point, value, .. }) => assert!(point == 1 && value < bound, "{point} {value}"),
            other => panic!("expected a violation of point 1, got {other:?}"),
        }
        // the lowest point may stay where it started
        assert!(table_violation(&from, &from, &pts, table_z).is_none());
    }

    #[test]
    fn envelope_has_the_camera_when_a_lens_or_body_is_set() {
        let ws = crate::model::ArmConfig::rebot_dm().workspace;
        let rec = kinematics::cam_mount(false, 0.0, -38.0);
        let n = |mount: CameraMount| envelope(ws.as_ref(), &mount, &rec).len();
        assert_eq!(n(CameraMount::default()), 9, "TCP and 8 check points");
        assert_eq!(n(CameraMount { offset_m: Some([-0.06, 0.0, 0.04]), ..Default::default() }), 18);
        // a body size without a lens centre: the lens at the end_link origin, as CamModel assumes
        let pts = envelope(ws.as_ref(), &CameraMount { body_size_m: Some([0.05, 0.05, 0.04]), ..Default::default() }, &rec);
        assert_eq!((pts.len(), pts[9]), (18, [0.0; 3]));
    }

    #[test]
    fn transition_goes_tool_vertical_above_the_centred_target() {
        let (setup, _, cam) = v8();
        let r_now = inspect_r();
        let target = [0.30, 0.03, setup.table_z_m + setup.grasp_height_m];
        let pose = camera_down_pose(&r_now, &target, &setup, &cam, &[]);
        let (t, goal) = transition_pose(&pose, [cam.cx, cam.cy], grasp_plane(&setup), &setup, &cam).unwrap();
        assert!(close(&t, &target, 1e-9), "{t:?}");
        assert!(close(&goal.p, &[target[0], target[1], setup.table_z_m + setup.approach_height_m], 1e-9), "{:?}", goal.p);
        assert!(kinematics::column(&goal.r, 0)[2] < -1.0 + 1e-12, "tool axis down");
        assert!(kinematics::angle_between(&goal.r, &look_down(&r_now)) < 1e-9, "heading of the approach kept");
        // a box centre off the image centre maps to its own table point
        let off_px = [cam.cx + 30.0, cam.cy - 20.0];
        let (t2, _) = transition_pose(&pose, off_px, grasp_plane(&setup), &setup, &cam).unwrap();
        let px = cam.project(&pose, &t2).unwrap();
        assert!((px[0] - off_px[0]).abs() < 1e-6 && (px[1] - off_px[1]).abs() < 1e-6);
        // expected duration of the move: rotation of 38 deg at 0.5 rad/s dominates here
        let est = transition_estimate(&pose, &target, &setup);
        let rot = 1.1 * 1.875 * 38f64.to_radians() / motion::DEFAULT_ANGULAR_RPS;
        assert!(est >= rot - 1e-9 && est < rot + 3.0, "{est} vs {rot}");
    }

    #[test]
    fn test_parse_detections_grasp_width_px() {
        let raw = serde_json::json!([
            {"label": "can", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.3], "grasp_width_px": 42.5},
            {"label": "cup", "score": 0.8, "box_norm": [0.2, 0.2, 0.4, 0.4], "width_px": 35.0},
            {"label": "box", "score": 0.7, "box_norm": [0.3, 0.3, 0.5, 0.5], "grasp_width_px": 0.0},
            {"label": "pen", "score": 0.6, "box_norm": [0.4, 0.4, 0.6, 0.6], "width_px": -5.0},
            {"label": "mug", "score": 0.5, "box_norm": [0.5, 0.5, 0.7, 0.7]},
            [320.0, 180.0, 64.0, 36.0, 0.9, "arr_obj"]
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 6);
        assert_eq!(d[0].grasp_width_px, Some(42.5));
        assert_eq!(d[1].grasp_width_px, Some(35.0));
        assert_eq!(d[2].grasp_width_px, None, "zero width rejected");
        assert_eq!(d[3].grasp_width_px, None, "negative width rejected");
        assert_eq!(d[4].grasp_width_px, None, "absent width");
        assert_eq!(d[5].grasp_width_px, None, "array format detection has no grasp_width_px");
    }

    #[test]
    fn test_frames_width_px() {
        let mut d1 = det([0.0; 4], Some((0.0, 0.8)));
        d1.grasp_width_px = Some(40.0);

        let mut d2 = det([0.0; 4], Some((0.0, 0.9)));
        d2.grasp_width_px = Some(50.0);

        let mut d3 = det([0.0; 4], Some((0.0, 0.7)));
        d3.grasp_width_px = Some(45.0);

        // Low confidence detection
        let mut d_low = det([0.0; 4], Some((0.0, 0.2)));
        d_low.grasp_width_px = Some(100.0);

        // No width detection
        let d_nowidth = det([0.0; 4], Some((0.0, 0.9)));

        // Non-positive width
        let mut d_neg = det([0.0; 4], Some((0.0, 0.9)));
        d_neg.grasp_width_px = Some(-10.0);

        let frames = vec![d1, d2, d3, d_low, d_nowidth, d_neg];

        // min_conf = 0.5 filters out d_low. Valid widths: [40, 50, 45]. Median of [40, 45, 50] is 45.0.
        let w = frames_width_px(&frames, 0.5);
        assert_eq!(w, Some(45.0));

        // Even count: only d1 and d2 (min_conf = 0.75). Widths: [40, 50]. Median is 45.0.
        let w_even = frames_width_px(&frames, 0.75);
        assert_eq!(w_even, Some(45.0));

        // min_conf too high: no frames qualify
        assert_eq!(frames_width_px(&frames, 0.95), None);

        // Empty slice
        assert_eq!(frames_width_px(&[], 0.5), None);
    }

    #[test]
    fn test_plane_point_width_conversion() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let plane_z = 0.0;

        let centre_px = [320.0, 180.0];
        let w_px = 60.0;
        let closing_deg: f64 = 90.0;
        let (u_x, u_y) = (closing_deg.to_radians().cos(), closing_deg.to_radians().sin());
        let half_w = w_px / 2.0;
        let p0_px = [centre_px[0] - half_w * u_x, centre_px[1] - half_w * u_y];
        let p1_px = [centre_px[0] + half_w * u_x, centre_px[1] + half_w * u_y];

        let p0 = cam.plane_point(&pose, p0_px, plane_z).expect("p0 meets plane");
        let p1 = cam.plane_point(&pose, p1_px, plane_z).expect("p1 meets plane");
        let width_m = kinematics::norm(&kinematics::sub(&p1, &p0));

        // Projected width: dx = w_px * height / fx = 60.0 * 0.20 / 600.0 = 0.02 m (20 mm)
        assert!((width_m - 0.02).abs() < 1e-4, "expected 0.02 m, got {width_m}");
    }

    #[test]
    fn test_parse_detections_depth_fields() {
        let raw = serde_json::json!([
            {
                "label": "cup",
                "score": 0.9,
                "box_norm": [0.1, 0.1, 0.3, 0.3],
                "disp_fg": 1.25,
                "disp_bg": 0.85,
                "depth_conf": 0.92,
                "length_px": 55.0,
                "jaw_clear": true,
            },
            {
                "label": "bottle",
                "score": 0.8,
                "box_norm": [0.2, 0.2, 0.4, 0.4],
                "disp_fg": 0.0,
                "disp_bg": -0.5,
                "depth_conf": 1.5,
                "length_px": -10.0,
                "jaw_clear": false,
            }
        ]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!(d.len(), 2);
        assert_eq!(d[0].disp_fg, Some(1.25));
        assert_eq!(d[0].disp_bg, Some(0.85));
        assert_eq!(d[0].depth_conf, Some(0.92));
        assert_eq!(d[0].length_px, Some(55.0));
        assert_eq!(d[0].jaw_clear, Some(true));

        assert_eq!(d[1].disp_fg, Some(0.0));
        assert_eq!(d[1].disp_bg, Some(-0.5));
        assert_eq!(d[1].depth_conf, Some(1.0), "clamped to 1.0");
        assert_eq!(d[1].length_px, None, "negative length rejected");
        assert_eq!(d[1].jaw_clear, Some(false));
    }

    #[test]
    fn test_camera_optical_depth_and_ray() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let table_z = 0.0;

        // Image centre ray straight down: optical axis aligned with -z
        let ray = cam.ray_world_per_z(&pose, [320.0, 180.0]);
        assert!((ray[0]).abs() < 1e-6 && (ray[1]).abs() < 1e-6);
        assert!((ray[2] - (-1.0)).abs() < 1e-6, "ray[2] should be -1.0 for vertical down camera");

        // Optical depth: z_cam = (0.0 - 0.20) / (-1.0) = 0.20 m
        let z_opt = cam.table_optical_depth(&pose, [320.0, 180.0], table_z).expect("optical depth valid");
        assert!((z_opt - 0.20).abs() < 1e-6, "expected 0.20 m, got {z_opt}");

        // Off-centre pixel: orthogonal depth to table plane is the same (0.20 m)
        let z_off = cam.table_optical_depth(&pose, [420.0, 180.0], table_z).expect("off-centre valid");
        assert!((z_off - 0.20).abs() < 1e-6, "expected 0.20 m, got {z_off}");
    }

    #[test]
    fn test_depth_calibration_and_measurement() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let table_z = 0.0;

        // 1. Metric model with visible table
        let z_table = 0.20;
        let disp_table = 1.0 / z_table; // 5.0
        let samples = vec![
            [320.0, 180.0, disp_table],
            [300.0, 160.0, disp_table * 1.01],
            [340.0, 200.0, disp_table * 0.99],
        ];
        let depth_info = DepthFrameInfo { metric_model: true, table_samples: samples };
        let cal = calibrate_depth(&cam, &pose, Some(&depth_info), None, table_z).expect("metric_table calibrated");
        assert_eq!(cal.source, "metric_table");
        assert!((cal.a - 1.0).abs() < 0.05);
        assert_eq!(cal.b, 0.0);

        // Object on table: top height 0.04 m -> z_fg = 0.16 m -> disp_fg = 1/0.16 = 6.25, disp_bg = 1/0.20 = 5.0
        let det = Detection {
            label: "cube".into(),
            score: 0.9,
            raw_score_valid: true,
            box_norm: [0.45, 0.45, 0.55, 0.55],
            angle_deg: None,
            angle_conf: None,
            grasp_width_px: Some(30.0),
            mask_focus: false,
            disp_fg: Some(6.25),
            disp_bg: Some(5.0),
            depth_conf: Some(0.95),
            length_px: Some(50.0),
            jaw_clear: Some(true),
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None, contour_norm: None,
        };
        let meas = measure_detection_depth(&cam, &pose, &det, Some(&depth_info), None, table_z).expect("measurement valid");
        assert!((meas.height_m - 0.04).abs() < 0.005, "height error: {}", (meas.height_m - 0.04).abs());
        assert_eq!(meas.source, "metric_table");
        assert_eq!(meas.jaw_clear, Some(true));

        // 2. Table occluded (0 samples): no calibration, the grasp probes instead
        let occluded_depth = DepthFrameInfo { metric_model: true, table_samples: Vec::new() };
        assert!(calibrate_depth(&cam, &pose, Some(&occluded_depth), None, table_z).is_none(), "occluded table must not calibrate");

        // 3. Metric model with table samples contradicting FK returns None
        let conflicting_samples = vec![
            [320.0, 180.0, disp_table * 2.0],
            [300.0, 160.0, disp_table],
            [340.0, 200.0, disp_table * 2.0],
            [360.0, 220.0, disp_table],
        ];
        let conflicting_depth = DepthFrameInfo { metric_model: true, table_samples: conflicting_samples };
        assert!(calibrate_depth(&cam, &pose, Some(&conflicting_depth), None, table_z).is_none(), "contradicting table samples should return None");
    }

    #[test]
    fn test_anisotropic_camera_sizing() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 400.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let z_fg = 0.16;
        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "metric_table" };
        let det = Detection {
            label: "obj".into(),
            score: 0.9,
            raw_score_valid: true,
            box_norm: [0.45, 0.45, 0.55, 0.55],
            angle_deg: Some(90.0),
            angle_conf: Some(0.9),
            grasp_width_px: Some(30.0),
            mask_focus: false,
            disp_fg: Some(1.0 / z_fg),
            disp_bg: Some(1.0 / 0.20),
            depth_conf: Some(0.95),
            length_px: Some(50.0),
            jaw_clear: Some(true),
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None, contour_norm: None,
        };
        let meas = measure_detection_depth_with_cal(&cam, &pose, &det, &cal).expect("valid measurement");
        let expected_w = 30.0 * z_fg / 600.0;
        let expected_l = 50.0 * z_fg / 400.0;
        assert!((meas.width_m.unwrap() - expected_w).abs() < 1e-6);
        assert!((meas.length_m.unwrap() - expected_l).abs() < 1e-6);
    }

    #[test]
    fn test_http_detection_source_rescaling() {
        let src = HttpDetectionSource {
            upstream: DetectorUpstream::parse("http://127.0.0.1:8090").unwrap(),
            image_size_px: [640.0, 360.0],
        };
        let raw = serde_json::json!({
            "frame_id": 1,
            "width": 1280,
            "height": 720,
            "depth": {
                "metric_model": true,
                "table_samples": [[100.0, 200.0, 0.85]],
            },
            "detections": [{
                "label": "cup",
                "score": 0.9,
                "format": "xywh_center",
                "x": 640.0,
                "y": 360.0,
                "w": 100.0,
                "h": 100.0,
                "length_px": 80.0,
                "grasp_width_px": 40.0,
            }]
        });
        let now = Instant::now();
        let mut frame = src.frame(&raw, None, now, now, 0).expect("frame parsed");
        assert_eq!(frame.image_size_px, [1280.0, 720.0]);
        assert_eq!(frame.depth.as_ref().unwrap().table_samples[0], [100.0, 200.0, 0.85]);
        assert_eq!(frame.detections[0].length_px, Some(80.0));
        assert_eq!(frame.detections[0].grasp_width_px, Some(40.0));

        let cam_iso = CamModel {
            rec: crate::kinematics::IDENTITY,
            offset: [0.0; 3],
            fx: 900.0,
            fy: 900.0,
            cx: 480.0,
            cy: 270.0,
            size: [960.0, 540.0],
        };
        rescale_frame_to_cam(&mut frame, &cam_iso);
        assert_eq!(frame.image_size_px, [960.0, 540.0]);
        assert_eq!(frame.depth.as_ref().unwrap().table_samples[0], [75.0, 150.0, 0.85]);
        assert_eq!(frame.detections[0].length_px, Some(60.0));
        assert_eq!(frame.detections[0].grasp_width_px, Some(30.0));

        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "test" };
        let pose = Pose::new(crate::kinematics::IDENTITY, [0.0; 3]);
        let mut det_iso = frame.detections[0].clone();
        det_iso.depth_conf = Some(0.9);
        det_iso.disp_fg = Some(1.0);
        det_iso.disp_bg = Some(0.5);
        let meas_iso = measure_detection_depth_with_cal(&cam_iso, &pose, &det_iso, &cal).expect("measured");
        let expected_l_iso = 80.0 / 1200.0;
        let expected_w_iso = 40.0 / 1200.0;
        assert!((meas_iso.length_m.unwrap() - expected_l_iso).abs() < 1e-9);
        assert!((meas_iso.width_m.unwrap() - expected_w_iso).abs() < 1e-9);

        // Anisotropic rescaling:
        let now_aniso = Instant::now();
        let mut frame_aniso = DetectionFrame {
            frame_id: 2,
            captured_at: now_aniso,
            policy_captured_at: Some(now_aniso),
            image_size_px: [640.0, 360.0],
            detections: vec![Detection {
                label: "cup".into(),
                score: 0.9,
                raw_score_valid: true,
                box_norm: [0.3, 0.3, 0.7, 0.7],
                angle_deg: Some(45.0),
                angle_conf: Some(0.9),
                grasp_width_px: Some(120.0),
                mask_focus: false,
                disp_fg: Some(1.0),
                disp_bg: Some(0.5),
                depth_conf: Some(0.9),
                length_px: Some(240.0),
                jaw_clear: Some(true),
                depth_angle_deg: None,
                depth_angle_conf: None,
                depth_width_px: None, contour_norm: None,
            }],
            depth: None,
            model_generation: None,
        };
        let cam_aniso = CamModel {
            rec: crate::kinematics::IDENTITY,
            offset: [0.0; 3],
            fx: 1200.0,
            fy: 1800.0,
            cx: 640.0,
            cy: 540.0,
            size: [1280.0, 1080.0],
        };
        rescale_frame_to_cam(&mut frame_aniso, &cam_aniso);
        assert_eq!(frame_aniso.image_size_px, [1280.0, 1080.0]);

        let meas_aniso = measure_detection_depth_with_cal(&cam_aniso, &pose, &frame_aniso.detections[0], &cal).expect("measured");
        let expected_l_aniso = 240.0 / 600.0;
        let expected_w_aniso = 120.0 / 600.0;
        assert!((meas_aniso.length_m.unwrap() - expected_l_aniso).abs() < 1e-9);
        assert!((meas_aniso.width_m.unwrap() - expected_w_aniso).abs() < 1e-9);
    }

    #[test]
    fn yaw_and_width_source_settings_default_validate_and_patch() {
        let d = GraspSetup::default();
        assert_eq!(d.yaw_sources, [YawSource::Mask, YawSource::Box]);
        assert_eq!(d.yaw_record, [YawSource::Depth]);
        assert_eq!(d.width_sources, [WidthSource::Mask, WidthSource::Depth]);
        assert!(d.width_record.is_empty());
        // a setup file without the keys loads the defaults
        let old: GraspSetup = serde_json::from_value(serde_json::json!({"table_z_m": 0.01})).unwrap();
        assert_eq!((old.yaw_sources, old.yaw_record, old.width_sources, old.width_record),
            (d.yaw_sources.clone(), d.yaw_record.clone(), d.width_sources.clone(), d.width_record.clone()));
        let p = d.patched(&serde_json::json!({"yaw_sources": ["depth", "box"], "yaw_record": ["mask"], "width_sources": ["depth"], "width_record": ["mask"]})).unwrap();
        assert_eq!(p.yaw_sources, [YawSource::Depth, YawSource::Box]);
        assert_eq!(p.width_record, [WidthSource::Mask]);
        // the default yaw_record may name a deciding source: it decides instead
        assert!(d.patched(&serde_json::json!({"yaw_sources": ["depth", "mask", "box"]})).is_ok());
        for bad in [
            serde_json::json!({"yaw_sources": ["mask", "mask"]}),
            serde_json::json!({"yaw_record": ["depth", "depth"]}),
            serde_json::json!({"width_sources": ["mask", "mask"]}),
            serde_json::json!({"width_record": ["depth", "depth"]}),
            serde_json::json!({"yaw_sources": ["probe"]}),
            serde_json::json!({"width_sources": ["box"]}),
        ] {
            assert!(d.patched(&bad).is_err(), "{bad} accepted");
        }
    }

    /// A centred, elongated box (aspect 2) with mask and / or depth angles per frame.
    fn yaw_frames(mask: &[f64], depth: &[f64]) -> (Detection, Vec<Detection>) {
        let n = mask.len().max(depth.len()).max(1);
        let frames: Vec<Detection> = (0..n).map(|i| Detection {
            angle_deg: mask.get(i).copied(), angle_conf: mask.get(i).map(|_| 0.9),
            depth_angle_deg: depth.get(i).copied(), depth_angle_conf: depth.get(i).map(|_| 0.8),
            ..det([0.4, 0.45, 0.6, 0.55], None)
        }).collect();
        (frames[n - 1].clone(), frames)
    }

    const SIZE: [f64; 2] = [640.0, 360.0];

    fn turn_by(t: f64) -> impl Fn(f64) -> Result<Option<f64>, &'static str> { move |_| Ok(Some(t)) }

    fn summary(o: &YawOutcome) -> Vec<(&str, bool, bool, Option<&str>)> {
        o.entries.iter().map(|e| (e.source.as_str(), e.used, e.record_only, e.reason.as_deref())).collect()
    }

    #[test]
    fn yaw_chain_default_mask_decides_and_depth_is_recorded() {
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[30.0], &[-40.0]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.4), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (Some(0.4), "turned:angle"));
        assert_eq!(summary(&o), [("depth", false, true, None), ("mask", true, false, None)]);
        assert_eq!(o.entries[0].value, Some(-40.0));
        assert_eq!(o.entries[1].value, Some(30.0));
        assert_eq!(o.angle_spread_deg, Some(0.0));
        // a turn below 1e-3 rad keeps J6
        let o = yaw_chain(&s, &last, &frames, SIZE, &|_| Ok(None), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "kept:angle"));
    }

    #[test]
    fn yaw_chain_depth_first_decides_with_both_present() {
        let s = GraspSetup { yaw_sources: vec![YawSource::Depth, YawSource::Mask, YawSource::Box], ..GraspSetup::default() };
        let (last, frames) = yaw_frames(&[30.0], &[-40.0]);
        let seen = std::cell::Cell::new(f64::NAN);
        let angle_turn = |alpha: f64| { seen.set(alpha); Ok(Some(0.2)) };
        let o = yaw_chain(&s, &last, &frames, SIZE, &angle_turn, &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (Some(0.2), "turned:depth"));
        assert_eq!(seen.get(), -40.0);
        // depth is a deciding source here (not recorded twice), the mask is not evaluated
        assert_eq!(summary(&o), [("depth", true, false, None)]);
        // the depth source never writes angle_spread_deg
        assert_eq!(o.angle_spread_deg, None);
        let o = yaw_chain(&s, &last, &frames, SIZE, &|_| Ok(None), &|_| Some(1.0));
        assert_eq!(o.align, "kept:depth");
    }

    #[test]
    fn yaw_chain_record_only_depth_never_turns_and_box_decides_without_a_mask() {
        let s = GraspSetup::default();
        // a plain detection model: no mask angle, a depth angle
        let (last, frames) = yaw_frames(&[], &[-40.0]);
        let angle_turn = |_: f64| -> Result<Option<f64>, &'static str> { panic!("no angle source decides") };
        let o = yaw_chain(&s, &last, &frames, SIZE, &angle_turn, &|short_x| { assert!(!short_x); Some(1.5) });
        assert_eq!((o.turn, o.align.as_str()), (Some(1.5), "turned:box"));
        assert_eq!(summary(&o), [("depth", false, true, None), ("mask", false, false, Some("no_angle")), ("box", true, false, None)]);
        assert_eq!(o.entries[0].value, Some(-40.0));
        assert_eq!(o.entries[2].value, Some(0.0), "box long side along image x");
        assert_eq!(o.angle_spread_deg, None);
        let o = yaw_chain(&s, &last, &frames, SIZE, &angle_turn, &|_| None);
        assert_eq!((o.turn, o.align.as_str()), (None, "kept:box"));
        // a square box: the aspect rejects, nothing else: skipped:aspect as before the chain
        let sq = Detection { box_norm: [0.45, 0.4, 0.55, 0.578], ..last.clone() };
        let o = yaw_chain(&s, &sq, std::slice::from_ref(&sq), SIZE, &angle_turn, &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "skipped:aspect"));
        assert_eq!(o.entries.last().unwrap().reason.as_deref(), Some("aspect"));
    }

    #[test]
    fn yaw_chain_spread_gate_is_terminal_per_angle_source() {
        // mask frames 0 / 60 deg disagree: skipped:angle_spread, the box does not decide
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[0.0, 60.0], &[]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "skipped:angle_spread"));
        assert!(o.angle_spread_deg.is_some_and(|a| a > 15.0));
        assert_eq!(summary(&o).last().unwrap(), &("mask", false, false, Some("angle_spread")));
        // the same for a deciding depth source, which does not write angle_spread_deg
        let s = GraspSetup { yaw_sources: vec![YawSource::Depth, YawSource::Box], yaw_record: vec![], ..GraspSetup::default() };
        let (last, frames) = yaw_frames(&[], &[0.0, 60.0]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str(), o.angle_spread_deg), (None, "skipped:angle_spread", None));
        // record-only: the spread only rejects the entry
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[10.0, 10.0], &[0.0, 60.0]);
        let o = yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!(o.align, "turned:angle");
        assert_eq!(summary(&o)[0], ("depth", false, true, Some("angle_spread")));
        // spread 0 disables the gate
        let s = GraspSetup { align_max_angle_spread_deg: 0.0, ..GraspSetup::default() };
        let (last, frames) = yaw_frames(&[0.0, 60.0], &[]);
        assert_eq!(yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0)).align, "turned:angle");
    }

    #[test]
    fn yaw_chain_guard_ray_and_limits_end_the_alignment() {
        let s = GraspSetup::default();
        let (last, frames) = yaw_frames(&[30.0], &[-40.0]);
        // the box at the image border: skipped:border before any source, the record is kept
        let cut = Detection { box_norm: [0.0, 0.45, 0.2, 0.55], ..last.clone() };
        let o = yaw_chain(&s, &cut, std::slice::from_ref(&cut), SIZE, &turn_by(0.3), &|_| Some(1.0));
        assert_eq!((o.turn, o.align.as_str()), (None, "skipped:border"));
        assert_eq!(summary(&o), [("depth", false, true, None)]);
        for why in ["ray", "j6_limits"] {
            let o = yaw_chain(&s, &last, &frames, SIZE, &|_| Err(why), &|_| Some(1.0));
            assert_eq!((o.turn, o.align), (None, format!("skipped:{why}")));
            assert_eq!(o.entries.last().unwrap().reason.as_deref(), Some(why));
        }
        // no deciding source
        let s = GraspSetup { yaw_sources: vec![], ..GraspSetup::default() };
        assert_eq!(yaw_chain(&s, &last, &frames, SIZE, &turn_by(0.3), &|_| Some(1.0)).align, "skipped:no_source");
    }

    #[test]
    fn depth_angle_fields_parse_and_rescale_like_the_mask_angle() {
        let raw = serde_json::json!([{"label": "block", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2],
            "depth_angle_deg": 120.0, "depth_angle_conf": 1.5, "depth_width_px": 20.0},
            {"label": "block", "score": 0.9, "box_norm": [0.1, 0.1, 0.3, 0.2], "depth_width_px": -1.0}]);
        let d = parse_detections(&raw, [640.0, 360.0]);
        assert_eq!((d[0].depth_angle_deg, d[0].depth_angle_conf, d[0].depth_width_px), (Some(-60.0), Some(1.0), Some(20.0)));
        assert_eq!((d[0].angle_deg, d[0].grasp_width_px), (None, None), "depth values stay separate");
        assert_eq!((d[1].depth_angle_deg, d[1].depth_width_px), (None, None));
        // the same rescale as angle_deg / grasp_width_px
        let setup = GraspSetup { fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let mk = |mask: bool| Detection {
            angle_deg: mask.then_some(30.0), grasp_width_px: mask.then_some(40.0), length_px: Some(80.0),
            depth_angle_deg: (!mask).then_some(30.0), depth_width_px: (!mask).then_some(40.0),
            ..det([0.4, 0.4, 0.6, 0.6], None)
        };
        let frame = |d: Detection| {
            let now = Instant::now();
            DetectionFrame { frame_id: 1, captured_at: now, policy_captured_at: Some(now), image_size_px: [1280.0, 540.0], detections: vec![d], depth: None, model_generation: None }
        };
        let (mut fm, mut fd) = (frame(mk(true)), frame(mk(false)));
        rescale_frame_to_cam(&mut fm, &cam);
        rescale_frame_to_cam(&mut fd, &cam);
        let (m, dd) = (&fm.detections[0], &fd.detections[0]);
        assert_eq!(m.angle_deg, dd.depth_angle_deg);
        assert_eq!(m.grasp_width_px, dd.depth_width_px);
        assert_eq!(m.length_px, dd.length_px, "the length follows the depth angle without a mask angle");
        assert!((m.angle_deg.unwrap() - 30.0).abs() > 1.0, "anisotropic rescale changed the angle");
    }

    #[test]
    fn depth_measurement_width_from_the_detector_else_the_depth_pipeline() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "metric_table" };
        let base = Detection { disp_fg: Some(6.25), disp_bg: Some(5.0), depth_conf: Some(0.95), length_px: Some(50.0), ..det([0.45, 0.45, 0.55, 0.55], None) };
        // a detection model: depth PCA width 30 px at 90 deg -> the numbers of grasp_width_px / angle_deg
        let old = Detection { angle_deg: Some(90.0), grasp_width_px: Some(30.0), ..base.clone() };
        let new = Detection { depth_angle_deg: Some(90.0), depth_width_px: Some(30.0), ..base.clone() };
        let (mo, mn) = (measure_detection_depth_with_cal(&cam, &pose, &old, &cal).unwrap(), measure_detection_depth_with_cal(&cam, &pose, &new, &cal).unwrap());
        assert_eq!((mo.width_m, mo.length_m), (mn.width_m, mn.length_m));
        assert!(mn.width_m.is_some());
        // a segmentation model: the mask width decides the measurement's width, as before
        let seg = Detection { grasp_width_px: Some(20.0), angle_deg: Some(90.0), ..new.clone() };
        let ms = measure_detection_depth_with_cal(&cam, &pose, &seg, &cal).unwrap();
        let only_mask = Detection { depth_angle_deg: None, depth_width_px: None, ..seg.clone() };
        assert_eq!(ms, measure_detection_depth_with_cal(&cam, &pose, &only_mask, &cal).unwrap());
    }

    #[test]
    fn depth_calibration_and_measurement_report_why_they_fail() {
        let setup = GraspSetup { camera_tilt_deg: 0.0, fx_px: 600.0, fy_px: 600.0, image_size_px: [640.0, 360.0], ..Default::default() };
        let cam = CamModel::assumed(&crate::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
        let pose = Pose::new(look_down(&inspect_r()), [0.3, 0.0, 0.20]);
        let why = |d: Option<&DepthFrameInfo>| calibrate_depth_why(&cam, &pose, d, None, 0.0).err();
        assert_eq!(why(None), Some("no_depth_frame"));
        let metric = |samples: Vec<[f64; 3]>| DepthFrameInfo { metric_model: true, table_samples: samples };
        assert_eq!(why(Some(&metric(vec![]))), Some("too_few_samples"));
        assert_eq!(why(Some(&metric(vec![[320.0, 180.0, 10.0], [300.0, 160.0, 5.0], [340.0, 200.0, 2.5]]))), Some("table_contradicts_fk"));
        // metric 50 % off the prior (the simulator's --depth-scale 1.5): contradicts FK
        assert_eq!(why(Some(&metric(vec![[320.0, 180.0, 5.0 / 1.5], [300.0, 160.0, 5.0 / 1.5], [340.0, 200.0, 5.0 / 1.5]]))), Some("table_contradicts_fk"));
        assert_eq!(why(Some(&metric(vec![[320.0, 180.0, 5.0], [300.0, 160.0, 5.0], [340.0, 200.0, 5.0]]))), None);
        // relative: a flat look-down table spans no inverse depth
        let grid: Vec<[f64; 3]> = (0..12).map(|i| [100.0 + 40.0 * i as f64, 100.0 + 15.0 * i as f64, 2.0]).collect();
        let rel = |samples: Vec<[f64; 3]>| DepthFrameInfo { metric_model: false, table_samples: samples };
        assert_eq!(why(Some(&rel(grid.clone()))), Some("no_oblique_spread"));
        assert_eq!(why(Some(&rel(grid[..5].to_vec()))), Some("too_few_samples"));
        // the boolean API is unchanged
        assert!(calibrate_depth(&cam, &pose, Some(&rel(grid)), None, 0.0).is_none());

        let cal = DepthCalibration { a: 1.0, b: 0.0, source: "metric_table" };
        let ok = Detection { disp_fg: Some(6.25), disp_bg: Some(5.0), depth_conf: Some(0.95), ..det([0.45, 0.45, 0.55, 0.55], None) };
        let m = |d: Detection| measure_detection_depth_with_cal_why(&cam, &pose, &d, &cal).err();
        assert_eq!(m(ok.clone()), None);
        assert_eq!(m(Detection { depth_conf: Some(0.2), ..ok.clone() }), Some("low_conf"));
        assert_eq!(m(Detection { disp_fg: None, ..ok.clone() }), Some("no_disparity"));
        assert_eq!(m(Detection { disp_fg: Some(4.0), ..ok.clone() }), Some("bad_disparity"));
        assert_eq!(m(Detection { disp_fg: Some(5.001), ..ok.clone() }), Some("too_flat"));
    }

    struct DummyDetectionSource;
    impl DetectionSource for DummyDetectionSource {
        fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> { Box::pin(async { Ok(true) }) }
        fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> { Box::pin(async { Ok(()) }) }
        fn next_frame(&self, _after: Option<u64>, _wait_ms: u64) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> { Box::pin(async { Ok(None) }) }
    }

    #[tokio::test]
    async fn test_orchestrator_holding_now_load_and_gap_on_6dof() {
        let config = crate::model::ArmConfig::from_profile_file("profiles/so_arm101.json").expect("loads");
        let policy = config.gripper_policy.clone().expect("gripper_policy");

        let mut setup = GraspSetup::default();
        setup.post_lift_settle_ms = 10;

        let sup = Arc::new(ArmSupervisor::new(config));
        let (id, stamp) = {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = false;
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(admitted).unwrap();
            inner.commanded_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
            inner.measured_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.5];
            inner.loads = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.35];
            let stamp = inner.current_admission_stamp();
            let plan_in = inner.plan_input().unwrap();
            let admitted = inner.check_admission(stamp).unwrap();
            (SupervisorInner::begin_macro(admitted, "grasp", &plan_in, None).unwrap(), stamp)
        };

        let source = Arc::new(DummyDetectionSource);
        let cam = CamModel::assumed(&sup.inner().read().await.config.arm_model, &setup, &CameraMount::default());
        let ctx = MacroCtx {
            sup: sup.clone(),
            id,
            stamp,
            deadline: Instant::now() + Duration::from_secs(10),
            phase_timeout_s: setup.phase_timeout_s,
            timeout_code: "GRASP_TIMEOUT",
            timeout_message: "timeout".into(),
        };
        let orch = Orchestrator {
            ctx,
            source,
            setup: setup.clone(),
            cam,
            envelope: vec![],
            tcp_offset: [0.0; 3],
            triggered: false,
            lens_known: false,
            depth_available: false,
            generation: None,
        };

        let (holding, gap) = orch.holding_now().await.expect("holding_now should succeed");
        assert!(holding);
        assert!((gap - 0.5).abs() < 1e-4);

        // Test held_width calculation on 6-DOF
        let held_width = {
            let inner = sup.inner().read().await;
            inner.config.gripper_policy.as_ref().map(|p| {
                let idx = inner.config.gripper_index().unwrap_or(6);
                let m = inner.measured_positions.get(idx).copied().unwrap_or(0.0);
                p.opening_m(m)
            })
        };
        assert_eq!(held_width, Some(policy.opening_m(0.5)));
    }

    #[test]
    fn markers_of_a_trigger_reply() {
        let v = serde_json::json!([
            {"id": 3, "corners": [[10.0, 20.0], [30.0, 20.0], [30.0, 40.0], [10.0, 40.0]]},
            {"id": 4, "corners": [[1, 2], [3, 4], [5, 6]]},
            {"corners": [[1, 2], [3, 4], [5, 6], [7, 8]]},
            {"id": 5, "corners": [[1, 2], [3, 4], [5, 6], [7, "x"]]},
        ]);
        let m = parse_markers(&v);
        assert_eq!(m.len(), 1);
        assert_eq!((m[0].id, m[0].corners[2]), (3, [30.0, 40.0]));
        assert!(parse_markers(&Value::Null).is_empty());
    }

    fn build_rgf1(header: &Value, pixels: &[u8]) -> Vec<u8> {
        let mut js = header.to_string().into_bytes();
        while !(8 + js.len()).is_multiple_of(GRAY_FRAME_ALIGN) {
            js.push(b' ');
        }
        let mut out = Vec::with_capacity(8 + js.len() + pixels.len());
        out.extend_from_slice(GRAY_FRAME_MAGIC);
        out.extend_from_slice(&(js.len() as u32).to_le_bytes());
        out.extend_from_slice(&js);
        out.extend_from_slice(pixels);
        out
    }

    #[test]
    fn parse_gray_frame_decodes_cpp_golden_and_rejects_malformed() {
        // 1. Independently decode the C++-generated golden RGF1 fixture.
        let golden = include_bytes!("../tests/fixtures/gray_frame_golden.rgf1");
        let g = parse_gray_frame(golden, Some("rv-golden-0001"), Some(7)).expect("golden RGF1");
        assert_eq!(g.instance_id, "rv-golden-0001");
        assert_eq!(g.frame_id, 7);
        assert_eq!((g.img.w, g.img.h), (640, 360));
        assert_eq!(g.ts_ms, 1763791234567);
        assert_eq!((g.frame_mono_ms, g.wait_mono_ms, g.grab_mono_ms, g.stored_mono_ms), (123450, 123401, 123452, 123459));
        assert!((g.convert_ms - 4.9).abs() < 0.05);
        for y in 0..360usize {
            for x in 0..640usize {
                let expect = ((x * 3 + y * 5 + 17) & 0xFF) as u8;
                assert_eq!(g.img.px[y * 640 + x], expect, "pixel ({x},{y})");
            }
        }

        // 2. Identity mismatches
        assert!(matches!(parse_gray_frame(golden, Some("rv-other"), Some(7)), Err(GrayFrameError::Protocol(_))));
        assert!(matches!(parse_gray_frame(golden, Some("rv-golden-0001"), Some(8)), Err(GrayFrameError::Protocol(_))));

        // 3. Truncated / bad magic / extra payload / oversized / unaligned / bad padding
        assert!(matches!(parse_gray_frame(&golden[..6], None, None), Err(GrayFrameError::Protocol(_))));
        let mut bad_magic = golden.to_vec();
        bad_magic[0] = b'X';
        assert!(matches!(parse_gray_frame(&bad_magic, None, None), Err(GrayFrameError::Protocol(_))));
        assert!(matches!(parse_gray_frame(&golden[..golden.len() - 1], None, None), Err(GrayFrameError::Protocol(_))));
        let mut extra = golden.to_vec();
        extra.push(0);
        assert!(matches!(parse_gray_frame(&extra, None, None), Err(GrayFrameError::Protocol(_))));

        let mut oversized = Vec::from(&b"RGF1"[..]);
        oversized.extend_from_slice(&((GRAY_MAX_HEADER_BYTES as u32) + 8).to_le_bytes());
        assert!(matches!(parse_gray_frame(&oversized, None, None), Err(GrayFrameError::Protocol(_))));

        let mut unaligned = Vec::from(&b"RGF1"[..]);
        unaligned.extend_from_slice(&5u32.to_le_bytes());
        unaligned.extend_from_slice(b"{}   ");
        assert!(matches!(parse_gray_frame(&unaligned, None, None), Err(GrayFrameError::Protocol(_))));

        let px = vec![0x42u8; 640 * 360];
        let base_hdr = || serde_json::json!({
            "api_version": 2, "encoding": "RGF1", "format": "gray8",
            "instance_id": "rv-test1", "frame_id": 1,
            "width": 640, "height": 360, "stride": 640,
            "frame_size": [640, 640], "frame_format": "RGB888",
            "crop": [0, 140, 640, 360], "channel": 0,
            "luma": GRAY_V1_LUMA, "luma_range": GRAY_V1_LUMA_RANGE,
            "ts_ms": 1763791234567u64, "frame_mono_ms": 100u64,
            "wait_mono_ms": 95u64, "grab_mono_ms": 101u64, "stored_mono_ms": 105u64,
            "convert_ms": 4.2, "model_generation": 0u64,
        });
        assert!(parse_gray_frame(&build_rgf1(&base_hdr(), &px), Some("rv-test1"), Some(1)).is_ok());

        // Non-space trailing byte inside padded header
        let mut bad_pad = build_rgf1(&base_hdr(), &px);
        let hlen = u32::from_le_bytes([bad_pad[4], bad_pad[5], bad_pad[6], bad_pad[7]]) as usize;
        bad_pad[8 + hlen - 1] = b'\t';
        assert!(matches!(parse_gray_frame(&bad_pad, None, None), Err(GrayFrameError::Protocol(_))));

        // Missing required v1 metadata fields must all be rejected
        for missing_key in [
            "encoding", "format", "frame_format", "frame_size", "crop", "channel",
            "luma", "luma_range", "ts_ms", "frame_mono_ms", "wait_mono_ms",
            "grab_mono_ms", "stored_mono_ms", "convert_ms", "model_generation",
        ] {
            let mut h = base_hdr();
            h.as_object_mut().unwrap().remove(missing_key);
            assert!(
                matches!(parse_gray_frame(&build_rgf1(&h, &px), None, None), Err(GrayFrameError::Protocol(_))),
                "missing {missing_key} must fail"
            );
        }

        // Incompatible metadata, crop out-of-bounds/mismatch, negative timing, bad stride/dims
        for mut h in [
            { let mut h = base_hdr(); h["encoding"] = serde_json::json!("RGF2"); h },
            { let mut h = base_hdr(); h["frame_format"] = serde_json::json!("BGR888"); h },
            { let mut h = base_hdr(); h["luma"] = serde_json::json!("limited"); h },
            { let mut h = base_hdr(); h["luma_range"] = serde_json::json!("limited_16_235"); h },
            { let mut h = base_hdr(); h["frame_size"] = serde_json::json!([0, 640]); h },
            { let mut h = base_hdr(); h["crop"] = serde_json::json!([0, 300, 640, 360]); h }, // 300+360 > 640
            { let mut h = base_hdr(); h["crop"] = serde_json::json!([0, 140, 640, 350]); h }, // ch != height
            { let mut h = base_hdr(); h["crop"] = serde_json::json!([0, 140, 640]); h },      // wrong arity
            { let mut h = base_hdr(); h["convert_ms"] = serde_json::json!(-1.0); h },
            { let mut h = base_hdr(); h["stored_mono_ms"] = serde_json::json!(-5); h },
            { let mut h = base_hdr(); h["stored_mono_ms"] = serde_json::json!(100u64); h }, // stored before grab (101)
            { let mut h = base_hdr(); h["stride"] = serde_json::json!(648); h },
            { let mut h = base_hdr(); h["width"] = serde_json::json!(320); h["stride"] = serde_json::json!(320); h },
            { let mut h = base_hdr(); h["height"] = serde_json::json!(u64::MAX); h },
            { let mut h = base_hdr(); h["api_version"] = serde_json::json!(1); h },
            { let mut h = base_hdr(); h["format"] = serde_json::json!("rgb888"); h },
        ] {
            let body = build_rgf1(&h, &px);
            assert!(matches!(parse_gray_frame(&body, None, None), Err(GrayFrameError::Protocol(_))), "{h}");
            h["width"] = serde_json::json!(0);
            assert!(matches!(parse_gray_frame(&build_rgf1(&h, &[]), None, None), Err(GrayFrameError::Protocol(_))));
        }

        // Negotiated format mismatch (frame_size / crop differs from negotiated /api/mode descriptor)
        let negotiated = GrayFormatDescriptor {
            encoding: "RGF1".into(), format: "gray8".into(),
            width: 640, height: 360, stride: 640, ring: 4, exposure_qualified: false,
            frame_size: Some([640, 640]), crop: Some([0, 140, 640, 360]),
            luma: Some(GRAY_V1_LUMA.into()), luma_range: Some(GRAY_V1_LUMA_RANGE.into()),
        };
        assert!(negotiated.is_supported_v1());
        assert!(parse_gray_frame_with_format(&build_rgf1(&base_hdr(), &px), Some("rv-test1"), Some(1), Some(&negotiated)).is_ok());
        let mut diff_crop = base_hdr();
        diff_crop["crop"] = serde_json::json!([0, 100, 640, 360]);
        assert!(matches!(
            parse_gray_frame_with_format(&build_rgf1(&diff_crop, &px), None, None, Some(&negotiated)),
            Err(GrayFrameError::Protocol(m)) if m.contains("crop")
        ));
        let mut diff_fs = base_hdr();
        diff_fs["frame_size"] = serde_json::json!([1280, 720]);
        assert!(matches!(
            parse_gray_frame_with_format(&build_rgf1(&diff_fs, &px), None, None, Some(&negotiated)),
            Err(GrayFrameError::Protocol(m)) if m.contains("frame_size")
        ));
    }

    #[test]
    fn mode_info_parses_gray_and_aruco_and_upstream_loopback() {
        let m = ModeInfo::parse(Some(&serde_json::json!({
            "mode": "yolo", "enabled": false, "per_request_pipelines": true,
            "available": {"yolo": true, "aruco": true, "depth": false, "gray": true},
            "gray_frames": {
                "encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640,
                "frame_size": [640, 640], "crop": [0, 140, 640, 360],
                "luma": GRAY_V1_LUMA, "luma_range": GRAY_V1_LUMA_RANGE,
                "ring": 4, "exposure_qualified": false
            }
        })));
        assert!(m.per_request_pipelines && m.aruco_available && m.gray_available);
        assert_eq!(m.gray_format, Ok(Some(GrayFormatDescriptor {
            encoding: "RGF1".into(), format: "gray8".into(), width: 640, height: 360, stride: 640, ring: 4, exposure_qualified: false,
            frame_size: Some([640, 640]), crop: Some([0, 140, 640, 360]),
            luma: Some(GRAY_V1_LUMA.into()), luma_range: Some(GRAY_V1_LUMA_RANGE.into()),
        })));
        assert!(m.gray_format.as_ref().unwrap().as_ref().unwrap().is_supported_v1());
        let old = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "per_request_pipelines": true, "available": {"yolo": true, "aruco": true}})));
        assert!(old.aruco_available && !old.gray_available && old.gray_format == Ok(None));
        // An advertised but malformed descriptor is an error, not "absent".
        let good = serde_json::json!({
            "encoding": "RGF1", "format": "gray8", "width": 640, "height": 360, "stride": 640,
            "frame_size": [640, 640], "crop": [0, 140, 640, 360], "ring": 4
        });
        for (key, bad) in [
            ("width", serde_json::json!("640")), ("crop", serde_json::json!([0, 140, 640])),
            ("frame_size", serde_json::json!(null)), ("luma", serde_json::json!(1)), ("ring", serde_json::json!(-1)),
            ("exposure_qualified", serde_json::json!("yes")), ("exposure_qualified", serde_json::json!(1)),
            ("luma_range", serde_json::json!(null)),
        ] {
            let mut g = good.clone();
            g[key] = bad;
            let m = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "per_request_pipelines": true,
                "available": {"gray": true}, "gray_frames": g})));
            assert!(m.gray_format.is_err(), "{key}: {:?}", m.gray_format);
        }
        let mut no_enc = good.clone();
        no_enc.as_object_mut().unwrap().remove("encoding");
        for g in [no_enc, serde_json::json!(null), serde_json::json!([1, 2])] {
            let m = ModeInfo::parse(Some(&serde_json::json!({"mode": "yolo", "per_request_pipelines": true,
                "available": {"gray": true}, "gray_frames": g})));
            assert!(m.gray_format.is_err(), "{:?}", m.gray_format);
        }

        for lo in ["http://127.0.0.1:8090", "http://127.0.0.2:8090", "http://localhost:8090", "http://[::1]:8090"] {
            assert!(DetectorUpstream::parse(lo).unwrap().is_loopback(), "{lo}");
        }
        for rem in ["http://192.168.1.50:8090", "http://10.0.0.2:8090", "http://recamera.local:8090"] {
            assert!(!DetectorUpstream::parse(rem).unwrap().is_loopback(), "{rem}");
        }
    }

    #[test]
    fn table_z_from_tcp_requires_vertical_closed_known_contact_and_measured_fk() {
        let mut cfg = crate::ArmConfig::rebot_dm();
        cfg.active_tool.is_stock = false;
        cfg.active_tool.id = "custom_tool".into();
        cfg.active_tool.config_digest = "v1:test".into();
        cfg.physical_contact_end_m = None;
        let mut sup = SupervisorInner::new(cfg);

        // 1. Missing physical_contact_end_m -> rejected
        let err = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .unwrap_err();
        assert!(format!("{err:?}").contains("physical_contact_end_m"));

        // 2. Vertical pose (+X_end pointing -Z_base) with open gripper -> rejected
        sup.config.physical_contact_end_m = Some([0.0288, 0.0, 0.0]);
        let target_r = kinematics::rpy_to_r(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let q_seed = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        let p_end = sup.config.arm_model.fk(&q_seed).p;
        let (lo, hi): ([f64; 6], [f64; 6]) = (
            std::array::from_fn(|i| f64::from(sup.config.limits[i].min_position)),
            std::array::from_fn(|i| f64::from(sup.config.limits[i].max_position)),
        );
        let sol = sup.config.arm_model.solve_dls(
            &kinematics::Pose::new(target_r, p_end),
            &q_seed,
            &lo,
            &hi,
            kinematics::PAR.w,
        );
        assert!(sol.ok);
        let q_vert: Vec<f32> = sol.q.iter().map(|&v| v as f32).chain([-3.0]).collect();
        sup.measured_positions = q_vert.clone();
        sup.commanded_positions = vec![0.0; 7];
        let err = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .unwrap_err();
        assert!(format!("{err:?}").contains("closed"));

        // 3. Tilted > 5 deg with closed gripper -> rejected
        sup.measured_positions = vec![0.0, -1.167, -1.565, 1.0, 0.0, 0.0, 0.0];
        let err = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .unwrap_err();
        assert!(format!("{err:?}").contains("5 deg"));

        // 4. Vertical + closed: uses measured_positions FK + physical_contact_end_m (not commanded_positions)
        let mut q_ok = q_vert;
        q_ok[6] = 0.0;
        sup.measured_positions = q_ok.clone();
        let meas_end = sup.config.arm_model.fk_f32(&q_ok).unwrap();
        let expected_z = (meas_end.transform_point(&[0.0288, 0.0, 0.0])[2] * 1e4).round() / 1e4;
        let prep = prepare_setup_update(&sup, &serde_json::json!({"table_z_from_tcp": true}), None, false)
            .expect("valid vertical closed pose must compute table_z_m");
        assert!((prep.setup.table_z_m - expected_z).abs() < 1e-6);
    }
}
