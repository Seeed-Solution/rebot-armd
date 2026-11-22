//! Upgrade compatibility with the state a B601-DM unit persisted under an older armd
//! (tests/fixtures/b601dm_devstate): no gripper jaw curve, a tuned grasp_setup.json, a camera
//! calibration. A missing or narrower jaw curve must not drop or block that state, and
//! persisted intrinsics an older armd accepted must not stop the start.
mod common;

use std::path::{Path, PathBuf};
use std::sync::Arc;

use armd::calibration;
use armd::gripper_calib;
use armd::grasp;
use armd::model::GraspWidthCheck;
use armd::{ArmConfig, ArmSupervisor};
use serde_json::json;

const PROFILE_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");
const DEVSTATE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/b601dm_devstate");

fn profile() -> ArmConfig { ArmConfig::from_profile_file(PROFILE_PATH).unwrap() }

/// Supervisor with a physical-like driver and follower `rebot-dm` (`requires_real_jaw_curve()`).
async fn real_supervisor() -> Arc<ArmSupervisor> {
    let sup = Arc::new(ArmSupervisor::new(profile()).with_driver(Box::new(common::TestArm::new())));
    sup.set_follower("rebot-dm").await;
    assert!(sup.inner().read().await.requires_real_jaw_curve());
    sup
}

fn devstate_copy(dir: &Path) -> (PathBuf, PathBuf) {
    let setup = dir.join("grasp_setup.json");
    let cal = dir.join("calibration.json");
    std::fs::copy(Path::new(DEVSTATE).join("grasp_setup.json"), &setup).unwrap();
    std::fs::copy(Path::new(DEVSTATE).join("calibration.json"), &cal).unwrap();
    (setup, cal)
}

#[tokio::test]
async fn device_grasp_setup_loads_on_a_real_follower_without_jaw_curve() {
    let dir = tempfile::tempdir().unwrap();
    let (setup_path, cal_path) = devstate_copy(dir.path());
    let sup = real_supervisor().await;
    grasp::load_into(&sup, &setup_path).await;
    calibration::load_into(&sup, &cal_path, Some(&setup_path)).await.unwrap();
    let inner = sup.inner().read().await;
    assert!(inner.config.gripper_policy.as_ref().unwrap().jaw_opening.is_none(), "the profile has no jaw curve");
    let g = inner.grasp_setup().unwrap();
    assert_eq!(g.table_z_m, 0.0098);
    assert!((g.camera_tilt_deg + 37.0).abs() < 0.01, "tilt {}", g.camera_tilt_deg);
    assert_eq!((g.grasp_height_m, g.max_grasp_width_m), (0.015, 0.075));
    assert_eq!(g.fx_px, 803.844739372464);
    assert_eq!(inner.grasp_width_check(g.max_grasp_width_m), Some(GraspWidthCheck::Unverified));
    let wc = grasp::grasp_width_check_json(&inner, g.max_grasp_width_m);
    assert_eq!((wc["status"].as_str(), wc["jaw_curve"].as_str()), (Some("unverified"), Some("missing")), "{wc}");
    assert_eq!(wc["effective_max_grasp_width_m"], json!(0.075));
}

#[tokio::test]
async fn speed_only_and_width_patches_are_accepted_without_jaw_curve() {
    let dir = tempfile::tempdir().unwrap();
    let (setup_path, _) = devstate_copy(dir.path());
    let sup = real_supervisor().await;
    grasp::load_into(&sup, &setup_path).await;
    let inner = sup.inner().read().await;
    let p = grasp::prepare_setup_update(&inner, &json!({"approach_speed_mps": 0.04}), Some(&setup_path), false).unwrap();
    assert_eq!((p.setup.approach_speed_mps, p.setup.table_z_m), (0.04, 0.0098));
    // without a curve a new width cannot be verified: accepted (unverified)
    assert!(grasp::prepare_setup_update(&inner, &json!({"max_grasp_width_m": 0.05}), None, false).is_ok());
}

fn write_curve(dir: &Path, max_open_m: f32) -> PathBuf {
    let p = dir.join("gripper_calib.json");
    let f = json!({"offset_rad": 0.02, "jaw_opening": {"points": [[-5.7, max_open_m], [0.0, 0.0]], "calibrated": true}});
    std::fs::write(&p, f.to_string()).unwrap();
    p
}

#[tokio::test]
async fn narrower_jaw_curve_keeps_calibration_and_setup_and_clamps_the_width() {
    let dir = tempfile::tempdir().unwrap();
    let (setup_path, _) = devstate_copy(dir.path());
    let calib_path = write_curve(dir.path(), 0.070);
    let sup = real_supervisor().await;
    gripper_calib::load_into(&sup, &calib_path).await;
    grasp::load_into(&sup, &setup_path).await;
    let inner = sup.inner().read().await;
    assert!(!inner.gripper_calib_invalid, "a 70 mm curve under a 75 mm setup keeps the calibration");
    assert_eq!(inner.gripper_calib_offset, Some(0.02));
    assert!(inner.gripper_calib_jaw_opening.is_some());
    let g = inner.grasp_setup().unwrap();
    assert_eq!((g.table_z_m, g.max_grasp_width_m), (0.0098, 0.075), "the setup file stays in effect");
    let check = inner.grasp_width_check(g.max_grasp_width_m).unwrap();
    let GraspWidthCheck::Clamped { curve_max_m } = check else { panic!("{check:?}") };
    assert!((curve_max_m - 0.070).abs() < 1e-6);
    assert!((check.effective_max_m(0.075) - 0.070).abs() < 1e-6);
    let wc = grasp::grasp_width_check_json(&inner, g.max_grasp_width_m);
    assert_eq!((wc["status"].as_str(), wc["jaw_curve"].as_str()), (Some("clamped"), Some("present")), "{wc}");
    // other edits are accepted; a new width beyond the curve is refused
    assert!(grasp::prepare_setup_update(&inner, &json!({"descend_speed_mps": 0.02}), None, false).is_ok());
    let err = grasp::prepare_setup_update(&inner, &json!({"max_grasp_width_m": 0.08}), None, false).unwrap_err();
    assert!(err.to_string().contains("max_grasp_width_m"), "{err}");
    assert!(grasp::prepare_setup_update(&inner, &json!({"max_grasp_width_m": 0.065}), None, false).is_ok());
    drop(inner);
    assert!(calib_path.exists() && setup_path.exists(), "neither file is discarded");
}
