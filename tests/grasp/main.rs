mod geometry_and_depth;
mod setup_and_probes;
mod yaw_and_gray;

use armd::grasp::Detection;
use armd::kinematics::Mat3;

pub(crate) fn det(b: [f64; 4], angle: Option<(f64, f64)>) -> Detection {
    Detection { label: "cup".into(), score: 0.9, raw_score_valid: true, box_norm: b, angle_deg: angle.map(|a| a.0), angle_conf: angle.map(|a| a.1), grasp_width_px: None, mask_focus: false, disp_fg: None, disp_bg: None, depth_conf: None, length_px: None, jaw_clear: None, depth_angle_deg: None, depth_angle_conf: None, depth_width_px: None, contour_norm: None }
}

/// End rotation at the inspect preset (joint angles, J6 at its rest angle 0).
pub(crate) fn inspect_r() -> Mat3 { armd::chain::ArmModel::REBOT_DEV_ARM.fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 0.0]).r }

/// Mirror of `test_rt_a1_stop`'s static admission-stamp audit across the split `src/grasp` tree
/// so no grasp child module can re-sample an admission stamp in production code.
#[test]
fn grasp_submodules_never_resample_admission_stamp() {
    fn collect_rs(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                collect_rs(&path, out);
            } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }

    let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut files = vec![manifest_dir.join("src/grasp.rs")];
    collect_rs(&manifest_dir.join("src/grasp"), &mut files);
    assert!(
        files.len() >= 10,
        "expected facade plus grasp submodules, found only {}",
        files.len()
    );
    for path in files {
        let src = std::fs::read_to_string(&path).unwrap();
        let prod = src.split("#[cfg(test)]").next().unwrap_or(&src);
        assert!(
            !prod.contains("current_admission_stamp") && !prod.contains("capture_admission_stamp"),
            "{} must never re-sample an admission stamp in production code",
            path.display()
        );
    }
}
