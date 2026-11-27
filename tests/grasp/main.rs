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
