use armd::grasp::*;
use armd::kinematics::{self, Pose, Vec3};
use armd::model::CameraMount;

use crate::{det, inspect_r};

#[test]
#[allow(clippy::approx_constant)]
fn ray_and_projection_round_trip() {
    let setup = GraspSetup { camera_tilt_deg: 20.0, ..Default::default() };
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &mount);
    (setup, mount, cam)
}

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
    let flat = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &GraspSetup::default(), &CameraMount::default());
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
fn envelope_has_the_camera_when_a_lens_or_body_is_set() {
    let ws = armd::model::ArmConfig::rebot_dm().workspace;
    let rec = kinematics::cam_mount(false, 0.0, -38.0);
    let n = |mount: CameraMount| envelope(ws.as_ref(), &mount, &rec).len();
    assert_eq!(n(CameraMount::default()), 9, "TCP and 8 check points");
    assert_eq!(n(CameraMount { offset_m: Some([-0.06, 0.0, 0.04]), ..Default::default() }), 18);
    // a body size without a lens centre: the lens at the end_link origin, as CamModel assumes
    let pts = envelope(ws.as_ref(), &CameraMount { body_size_m: Some([0.05, 0.05, 0.04]), ..Default::default() }, &rec);
    assert_eq!((pts.len(), pts[9]), (18, [0.0; 3]));
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
    let cam = CamModel::assumed(&armd::chain::ArmModel::REBOT_DEV_ARM, &setup, &CameraMount::default());
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
