use std::collections::BTreeMap;
use std::time::Instant;

use serde_json::Value;

use armd::grasp::*;
use armd::motion::MotionError;

use crate::det;

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

