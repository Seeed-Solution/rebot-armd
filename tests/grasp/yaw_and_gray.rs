use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::Value;

use armd::api::proxy::DetectorUpstream;
use armd::grasp::*;
use armd::kinematics::{self, Pose};
use armd::macro_action::MacroCtx;
use armd::model::CameraMount;
use armd::supervisor::{ArmSupervisor, SupervisorInner};

use crate::{det, inspect_r};

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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let config = armd::model::ArmConfig::from_profile_file("profiles/so_arm101.json").expect("loads");
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
    let golden = include_bytes!("../fixtures/gray_frame_golden.rgf1");
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
    let mut cfg = armd::ArmConfig::rebot_dm();
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
