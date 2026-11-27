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
