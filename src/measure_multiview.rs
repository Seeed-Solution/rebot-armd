//! Multi-view object measurement (motion stereo): action `/api/v1/action/measure_multiview` (WS,
//! lease, operator session when armd enforces the PIN; capability `measure_multiview`).
//!
//! From the current pose (the camera sees the object), the arm moves to view poses displaced
//! laterally around the object, captures a camera snapshot and a detection at each settled pose
//! and triangulates object points from the images ([`crate::mview_stereo`]). The geometry uses
//! only the camera intrinsics (grasp setup), the hand-eye mount and the kinematics (FK of the
//! measured joints at each capture, kinematic correction included); no depth model, no depth
//! calibration.
//!
//! - Body `{label?, min_score? (0..1, 0.3), target? {box_norm, frame_id?}, motion? ("orbit" |
//!   "translate", "orbit"), axes? ("x" | "xy"; "x" for orbit, "xy" for translate), views? (x: 2..5,
//!   3; xy: 3..5, 5), baseline_m? (0.03..0.12, 0.08), speed_scale? (0.1..0.5, 0.3),
//!   return_to_start? (true)}`.
//! - View offsets ([`view_offsets`]) in the image plane of the start camera: `axes` "x" spreads
//!   them evenly over [-baseline/2, +baseline/2] on the image x axis (`views` 3: -B/2, 0 (the start
//!   pose), +B/2); "xy" puts the start pose and `views - 1` offsets on the circle of radius
//!   baseline/2 (`views` 5: +x, +y, -x, -y), so edges of every direction cross an epipolar line.
//! - `motion` "orbit": the start pose turned about the look-at point X (ray through the target box
//!   centre at [`LOOK_AT_ABOVE_TABLE_M`] above the table) so that the camera centre moves by the
//!   chord |offset| in the offset's direction at the same distance from X; X stays at the same
//!   pixel. "translate": the camera keeps its orientation and moves by the offset parallel to the
//!   image plane; X moves by about fx * |offset| / Z in the image. Before moving, the start box
//!   shifted by the predicted shift of X must stay [`MIN_FRAME_MARGIN_PX`] inside the image (else
//!   `OUT_OF_FRAME`, a refusal with fallbacks).
//! - Motion: IK of the view pose ([`motion::solve_goal_joints`]), then the joint-space quintic of
//!   `path: "joint"` at `speed_scale` with every check of a joint move (swept workspace, keep-out
//!   boxes, link capsules, power budget, torque and collision holds). A view refused before it
//!   moves ([`PLAN_REFUSALS`]) is retried at the mirrored offset (when unused), then at 0.75 and
//!   0.5 of the offset on either side; when all are refused the action fails with the refusal code.
//! - Times: `measurement.stats` carries the stage times of the reconstruction and `decode_ms`
//!   (all views); every view its own `decode_ms`. `measurement.timing` and `views[].timing`: the
//!   wall-clock phases ([`RunTiming`], [`ViewTiming`]).
//! - Capture: after the motion settled, [`MIN_SETTLE_S`], then the settle rule ([`is_still`] over a
//!   sliding window, until [`MAX_SETTLE_S`], else `NOT_SETTLED`). The start view (the arm was not
//!   moved by the action) skips the minimum wait: the settle rule only, for up to
//!   [`MAX_SETTLE_S`] - [`MIN_SETTLE_S`]. Then one snapshot `s0`: with
//!   inference idle the camera runs at 1 fps, so `/snapshot.jpg` can still hold a frame taken while
//!   the arm moved (real B601-DM, 2026-11-22: the same frame for 0.7-2.2 s). A joint sample, a YOLO
//!   trigger (a frame newer than the request; it also refreshes the snapshot), then snapshots every
//!   [`REFRESH_POLL`] until one differs from `s0` (at most [`REFRESH_MAX`], else
//!   `DETECTION_UNAVAILABLE`), then a second joint sample. The view's pose is the FK of the two
//!   samples' mean joints; when they differ by more than [`BRACKET_RAD`] the trigger and snapshot
//!   are retried (`s0` = the last snapshot), at most [`SNAPSHOT_TRIES`] times, then `MOVED`. The
//!   target box comes from the trigger frame of the accepted try.
//! - A fault, collision, torque limit, stop, E-stop or lease loss ends the action where the arm is
//!   (the supervisor ends it; [`recheck`] between steps). A measurement failure after motion
//!   ([`RETURN_AFTER`], `TOO_FEW_POINTS`) returns to the start joints first when `return_to_start`.
//! - Pose refinement ([`mview_stereo::pose`]): before the reconstruction, the rotation of every
//!   view but the reference is refined from table-texture corners outside the target box and the
//!   gripper region ([`GRIPPER_REGION_NORM`]) matched in 2D (real B601-DM, 2026-11-22: 1-2 mrad
//!   per view, otherwise a flat sheet triangulated several mm above the table). Each view reports
//!   its correction, point count and reprojection rms before and after (`views[].pose_refine`,
//!   `pose_refine`); a view with too few points or a correction above the cap keeps its
//!   kinematic pose.
//! - Heights are measured from a table plane fitted to the points outside the target box
//!   ([`mview_stereo::fit_table_plane`], `table_plane`); the object threshold is max(5 mm, 3 x the
//!   MAD of the table residuals). Without a usable fit the calibrated table z stays
//!   (`table_plane` null).
//! - The image computation runs in `spawn_blocking`, while the arm returns to the start pose.
//! - Camera hold ([`CameraHold`], gray frames only): for the run armd holds the provider's camera
//!   gate open (rebot-vision `POST /api/subscribe` without pipelines, refreshed every
//!   [`CAMERA_HOLD_REFRESH`], deleted at the end of every outcome), so a trigger does not wait for
//!   the 1 fps idle gate. A provider without it (`measurement.camera_hold` "unsupported") runs as before.
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde_json::{json, Map, Value};

use crate::grasp::{self, CamModel, Detection, DetectionSource, GraspSetup};
use crate::kinematics::{self, Pose, Vec3};
use crate::macro_action::{arm_fail, fail, MacroCtx, Stop, POLL};
use crate::motion::{self, MotionError, PathOpts};
use crate::mview_stereo::{self, Camera, Gray, Params, View};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const PATH: &str = "/api/v1/action/measure_multiview";
pub const MIN_VIEWS: usize = 2;
pub const MAX_VIEWS: usize = 5;
pub const DEFAULT_VIEWS: usize = 3;
pub const BASELINE_M: [f64; 2] = [0.03, 0.12];
pub const DEFAULT_BASELINE_M: f64 = 0.08;
pub const SPEED_SCALE: [f64; 2] = [0.1, 0.5];
pub const DEFAULT_SPEED_SCALE: f64 = 0.3;
/// Wait after a view motion settled, before the first snapshot (s). Real B601-DM, 2026-11-22: J1
/// trails its POS_VEL command by up to 19 mrad at the end of a view move and the rest decays with
/// tau ~0.7 s, so after 1 s the trigger-and-snapshot bracket still moved 2.7-5.7 mrad.
pub const MIN_SETTLE_S: f64 = 2.0;
/// After the trigger: a snapshot equal to the one before the trigger is polled again every
/// [`REFRESH_POLL`], for at most [`REFRESH_MAX`] (real B601-DM, 2026-11-22: the snapshot changed
/// 0.03-0.08 s after the trigger returned, and up to 1.1 s later when it was not the trigger's frame;
/// idle frames come 0.7-2.2 s apart).
pub const REFRESH_POLL: Duration = Duration::from_millis(50);
pub const REFRESH_MAX: Duration = Duration::from_millis(3000);
/// Look-at point: the ray through the target box centre at this height above the table (m).
pub const LOOK_AT_ABOVE_TABLE_M: f64 = 0.02;
/// Fallback offsets as fractions of the refused one (after the mirrored offset).
pub const FALLBACK_SCALES: [f64; 2] = [0.75, 0.5];
/// Refusals before a view's motion (planning, or the target leaving the image): the view is
/// retried at a fallback offset.
pub const PLAN_REFUSALS: [&str; 5] = ["IK_UNREACHABLE", "IK_BRANCH_FLIP", "WORKSPACE_VIOLATION", "SOFT_LIMIT_EXCEEDED", "OUT_OF_FRAME"];
/// Failures after the arm moved that still return to the start joints (`return_to_start`).
pub const RETURN_AFTER: [&str; 8] = ["TARGET_LOST", "DETECTION_UNAVAILABLE", "UNSUPPORTED", "IK_UNREACHABLE", "IK_BRANCH_FLIP", "WORKSPACE_VIOLATION", "SOFT_LIMIT_EXCEEDED", "OUT_OF_FRAME"];
/// A view is refused (`OUT_OF_FRAME`) when the start target box, moved by the predicted shift of
/// the look-at point, would come closer than this to the image border (px).
pub const MIN_FRAME_MARGIN_PX: f64 = 8.0;
/// The two farthest view offsets must stay at least this far apart (m) after fallbacks.
pub const MIN_EFFECTIVE_BASELINE_M: f64 = 0.02;
/// Settle check after [`MIN_SETTLE_S`]: a window of [`STILL_WINDOW`] joint samples
/// [`STILL_SAMPLE`] apart is still when, for every arm joint, the median |velocity| is at most
/// [`STILL_VEL`] and the position range at most [`STILL_RANGE_RAD`]. Windows slide until
/// [`MAX_SETTLE_S`] after the motion, then `NOT_SETTLED`. (Real B601-DM, 2026-11-22: after a
/// joint move at speed_scale 0.3 the joints creep 2-4 mrad over seconds and J4 / J5 can hold a
/// limit cycle of a few mrad; single |velocity| samples reach 0.037 rad/s.)
pub const STILL_WINDOW: usize = 10;
pub const STILL_SAMPLE: Duration = Duration::from_millis(20);
pub const STILL_VEL: f32 = 0.03;
pub const STILL_RANGE_RAD: f32 = 0.003;
pub const MAX_SETTLE_S: f64 = 4.0;
/// The snapshot's pose: the joints sampled before the trigger request and right after the fresh
/// snapshot, averaged. When any arm joint differs by more than [`BRACKET_RAD`] between the two,
/// trigger and snapshot are retried [`SNAPSHOT_GAP_RETRY`] later, at most [`SNAPSHOT_TRIES`]
/// times, then `MOVED`. With `pose_refine` `off`; see [`BRACKET_REFINED_RAD`].
pub const BRACKET_RAD: f32 = 0.002;
/// [`BRACKET_RAD`] when the views' rotations are refined from the images (`pose_refine` `auto` or
/// `aruco`): the refinement absorbs a residual rotation (real B601-DM, rc25: 1.9-5.5 mrad per
/// view), a 5 mrad joint change moves the camera by well under 1 mm, and a drift of a few mrad
/// per second does not blur a ~30 ms exposure. Real B601-DM, rc25: brackets of 2.3-4.6 mrad over
/// the ~1 s trigger made 2 of 6 measurements fail `MOVED`.
pub const BRACKET_REFINED_RAD: f32 = 0.005;
pub const SNAPSHOT_TRIES: usize = 3;
pub const SNAPSHOT_GAP_RETRY: Duration = Duration::from_millis(500);
pub const FEEDBACK_MAX_AGE: Duration = Duration::from_millis(250);
/// Maximum time to wait after a gray frame fetch for every arm joint to receive fresh hardware
/// feedback (`ts >= fetch_completed`) before failing with `STALE_TELEMETRY`.
pub const FEEDBACK_WAIT_MAX: Duration = Duration::from_millis(500);
/// A view's target detection: centre at most this far (normalised image units) from where the
/// start view saw it, moved by the shift of the look-at point in that view.
pub const TARGET_MAX_SHIFT_NORM: f64 = 0.2;
/// The start detection matching `target.box_norm`: IoU at least this.
pub const TARGET_MIN_IOU: f64 = 0.3;
/// Feature region: the reference box grown by this fraction of its size on each side plus
/// [`REGION_PAD_PX`].
const REGION_GROW: f64 = 0.25;
const REGION_PAD_PX: f64 = 12.0;
const TRIGGER_WAIT_MS: u64 = 1500;
const TRIGGERS: usize = 3;
const YOLO: &[&str] = &["yolo"];
/// Pipelines of a capture trigger when marker corners are wanted (`pose_refine` `auto` with a
/// provider that runs ArUco, or `aruco`).
const YOLO_ARUCO: &[&str] = &["yolo", "aruco"];
/// Pipelines of a raw gray capture trigger (negotiated once per run when `available.gray` and
/// loopback).
const YOLO_GRAY: &[&str] = &["yolo", "gray"];
const YOLO_ARUCO_GRAY: &[&str] = &["yolo", "aruco", "gray"];
/// `pose_refine` `"aruco"`: marker corners each view must share with the reference view.
pub const MIN_MARKER_CORNERS: usize = 8;
/// Camera hold during a gray-frame run ([`CameraHold`]): the provider's subscription client, its
/// time to live and how often armd refreshes it. The hold ends at most this TTL after armd stops
/// refreshing it (a crash included).
pub const CAMERA_HOLD_CLIENT: &str = "armd-measure-multiview";
pub const CAMERA_HOLD_TTL_MS: u64 = 5000;
pub const CAMERA_HOLD_REFRESH: Duration = Duration::from_millis(1500);
const CAMERA_RELEASE_WAIT: Duration = Duration::from_secs(1);

/// The camera gate held open during one run (rebot-vision camera hold, a subscription without
/// pipelines): with inference idle the gate runs at 1 fps and a trigger waits for the gate; held,
/// the trigger is served by the next frame. Only for [`FrameSource::Gray`]: there the frame is
/// fetched by its id, while `/snapshot.jpg` would refresh with frames that are not the trigger's.
/// [`CameraHold::release`] at the end of the run (every outcome); on a panic `Drop` stops the
/// refresh and the hold expires after [`CAMERA_HOLD_TTL_MS`].
pub struct CameraHold {
    src: Arc<dyn DetectionSource>,
    id: Option<String>,
    refresh: Option<tokio::task::JoinHandle<()>>,
    /// `measurement.camera_hold`: "held", "unsupported" (the provider has none), "failed" (the
    /// request failed; the run goes on without), "off" (snapshot source).
    pub status: &'static str,
}

impl CameraHold {
    pub async fn start(src: Arc<dyn DetectionSource>, wanted: bool) -> Self {
        let mut h = Self { src, id: None, refresh: None, status: "off" };
        if !wanted { return h; }
        match h.src.camera_hold(CAMERA_HOLD_CLIENT, CAMERA_HOLD_TTL_MS).await {
            Ok(Some(id)) => {
                let src = h.src.clone();
                h.refresh = Some(tokio::spawn(async move {
                    loop {
                        tokio::time::sleep(CAMERA_HOLD_REFRESH).await;
                        if let Err(e) = src.camera_hold(CAMERA_HOLD_CLIENT, CAMERA_HOLD_TTL_MS).await {
                            tracing::warn!("measure_multiview: camera hold refresh failed: {e}");
                        }
                    }
                }));
                h.id = Some(id);
                h.status = "held";
            }
            Ok(None) => h.status = "unsupported",
            Err(e) => {
                tracing::warn!("measure_multiview: camera hold not taken ({e}); triggers wait for the idle camera gate");
                h.status = "failed";
            }
        }
        h
    }

    /// Stop refreshing (waiting for a refresh in flight) and end the hold.
    pub async fn release(mut self) {
        if let Some(r) = self.refresh.take() {
            r.abort();
            let _ = r.await;
        }
        if let Some(id) = self.id.take() {
            match tokio::time::timeout(CAMERA_RELEASE_WAIT, self.src.camera_release(&id)).await {
                Ok(Ok(())) => {}
                Ok(Err(e)) => tracing::warn!("measure_multiview: camera hold {id} not released ({e}); it expires in {CAMERA_HOLD_TTL_MS} ms"),
                Err(_) => tracing::warn!("measure_multiview: camera hold {id} release timed out; it expires in {CAMERA_HOLD_TTL_MS} ms"),
            }
        }
    }
}

impl Drop for CameraHold {
    fn drop(&mut self) {
        if let Some(r) = self.refresh.take() { r.abort(); }
    }
}

/// Negotiated image transport for one `measure_multiview` run (selected once, never switched mid-run).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameSource {
    /// Raw 8-bit grayscale via loopback `POST /api/trigger` `"gray"` + `GET /api/frame/gray` (RGF1).
    Gray,
    /// Legacy `/snapshot.jpg` polling + JPEG decode + letterbox center crop.
    Snapshot,
}

impl FrameSource {
    pub fn name(self) -> &'static str {
        match self {
            Self::Gray => "gray",
            Self::Snapshot => "snapshot",
        }
    }
}

fn invalid(m: impl Into<String>) -> ArmError { MotionError::InvalidCommand(m.into()).into() }

/// How the camera reaches a view (`motion`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Motion {
    /// Turned about the look-at point X: X stays at the same pixel, the orientation changes.
    Orbit,
    /// Moved parallel to the image plane, orientation kept: the object shifts in the image.
    Translate,
}

impl Motion {
    pub fn name(self) -> &'static str { match self { Self::Orbit => "orbit", Self::Translate => "translate" } }
}

/// Where the view offsets lie (`axes`): along the image x axis, or on a circle in the image plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axes { X, Xy }

/// Camera rotation refinement before the reconstruction (`pose_refine`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseRefine {
    /// Table texture outside the target box, plus ArUco marker corners when the provider returns them.
    Auto,
    /// ArUco marker corners only; every view needs [`MIN_MARKER_CORNERS`] shared with the
    /// reference, else `MARKERS_UNAVAILABLE`.
    Aruco,
    /// The kinematic camera poses.
    Off,
}

impl PoseRefine {
    pub fn name(self) -> &'static str { match self { Self::Auto => "auto", Self::Aruco => "aruco", Self::Off => "off" } }
    pub fn parse(s: &str) -> Option<Self> { match s { "auto" => Some(Self::Auto), "aruco" => Some(Self::Aruco), "off" => Some(Self::Off), _ => None } }
}

impl Axes {
    pub fn name(self) -> &'static str { match self { Self::X => "x", Self::Xy => "xy" } }
    /// Allowed `views` and the default.
    pub fn views(self) -> ([usize; 2], usize) { match self { Self::X => ([MIN_VIEWS, MAX_VIEWS], DEFAULT_VIEWS), Self::Xy => ([3, MAX_VIEWS], MAX_VIEWS) } }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    pub label: Option<String>,
    pub min_score: f64,
    pub target_box: Option<[f64; 4]>,
    pub target_frame_id: Option<u64>,
    pub motion: Motion,
    pub axes: Axes,
    pub views: usize,
    pub baseline_m: f64,
    pub speed_scale: f64,
    pub return_to_start: bool,
    /// Write a debug bundle ([`debug_dir`]).
    pub debug: bool,
    pub pose_refine: PoseRefine,
}

fn num(o: &Map<String, Value>, key: &str, default: f64, lo: f64, hi: f64) -> Result<f64, ArmError> {
    match o.get(key) {
        None | Some(Value::Null) => Ok(default),
        Some(v) => v.as_f64().filter(|x| x.is_finite() && (lo..=hi).contains(x)).ok_or_else(|| invalid(format!("{key} must be a number in [{lo}, {hi}]"))),
    }
}

/// The request body (see the module doc); unknown fields are refused.
pub fn parse_request(body: &Value) -> Result<Request, ArmError> {
    let empty = Map::new();
    let o = match body { Value::Null => &empty, Value::Object(o) => o, _ => return Err(invalid("Command body must be an object")) };
    const KEYS: [&str; 11] = ["label", "min_score", "target", "motion", "axes", "views", "baseline_m", "speed_scale", "return_to_start", "debug", "pose_refine"];
    if let Some(k) = o.keys().find(|k| !KEYS.contains(&k.as_str())) { return Err(invalid(format!("unknown field {k} ({})", KEYS.join(", ")))); }
    let label = match o.get("label") {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) if !s.is_empty() && s.len() <= 128 => Some(s.clone()),
        Some(_) => return Err(invalid("label must be a non-empty string (at most 128 characters)")),
    };
    let (target_box, target_frame_id) = match o.get("target") {
        None | Some(Value::Null) => (None, None),
        Some(Value::Object(t)) => {
            if let Some(k) = t.keys().find(|k| !matches!(k.as_str(), "box_norm" | "frame_id")) { return Err(invalid(format!("unknown target field {k} (box_norm, frame_id)"))); }
            let b = t.get("box_norm").and_then(Value::as_array).filter(|a| a.len() == 4)
                .and_then(|a| a.iter().map(|v| v.as_f64().filter(|x| x.is_finite() && (0.0..=1.0).contains(x))).collect::<Option<Vec<f64>>>())
                .filter(|b| b[2] > b[0] && b[3] > b[1])
                .ok_or_else(|| invalid("target.box_norm must be [x0, y0, x1, y1] in [0, 1] with x1 > x0, y1 > y0"))?;
            let fid = match t.get("frame_id") {
                None | Some(Value::Null) => None,
                Some(v) => Some(v.as_u64().ok_or_else(|| invalid("target.frame_id must be a non-negative integer"))?),
            };
            (Some([b[0], b[1], b[2], b[3]]), fid)
        }
        Some(_) => return Err(invalid("target must be an object {box_norm, frame_id?}")),
    };
    let motion = match o.get("motion") {
        None | Some(Value::Null) => Motion::Orbit,
        Some(Value::String(s)) if s == "orbit" => Motion::Orbit,
        Some(Value::String(s)) if s == "translate" => Motion::Translate,
        Some(_) => return Err(invalid("motion must be \"orbit\" or \"translate\"")),
    };
    let axes = match o.get("axes") {
        None | Some(Value::Null) => match motion { Motion::Orbit => Axes::X, Motion::Translate => Axes::Xy },
        Some(Value::String(s)) if s == "x" => Axes::X,
        Some(Value::String(s)) if s == "xy" => Axes::Xy,
        Some(_) => return Err(invalid("axes must be \"x\" or \"xy\"")),
    };
    let ([lo, hi], default_views) = axes.views();
    let views = match o.get("views") {
        None | Some(Value::Null) => default_views,
        Some(v) => v.as_u64().filter(|n| (lo as u64..=hi as u64).contains(n))
            .ok_or_else(|| invalid(format!("views must be an integer in [{lo}, {hi}] with axes \"{}\"", axes.name())))? as usize,
    };
    let return_to_start = match o.get("return_to_start") {
        None | Some(Value::Null) => true,
        Some(Value::Bool(b)) => *b,
        Some(_) => return Err(invalid("return_to_start must be a boolean")),
    };
    let debug = match o.get("debug") {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(_) => return Err(invalid("debug must be a boolean")),
    };
    let pose_refine = match o.get("pose_refine") {
        None | Some(Value::Null) => PoseRefine::Auto,
        Some(Value::String(s)) => PoseRefine::parse(s).ok_or_else(|| invalid("pose_refine must be \"auto\", \"aruco\" or \"off\""))?,
        Some(_) => return Err(invalid("pose_refine must be \"auto\", \"aruco\" or \"off\"")),
    };

    Ok(Request {
        label,
        min_score: num(o, "min_score", 0.3, 0.0, 1.0)?,
        target_box, target_frame_id, motion, axes, views,
        baseline_m: num(o, "baseline_m", DEFAULT_BASELINE_M, BASELINE_M[0], BASELINE_M[1])?,
        speed_scale: num(o, "speed_scale", DEFAULT_SPEED_SCALE, SPEED_SCALE[0], SPEED_SCALE[1])?,
        return_to_start, debug, pose_refine,
    })
}

/// View offsets [x, y] (m, image-plane axes of the start camera). `X`: `n` offsets spread evenly
/// over [-baseline/2, +baseline/2] on x. `Xy`: the start [0, 0] and `n - 1` offsets on the circle
/// of radius baseline/2 at angles 360 deg * k / (n - 1) from +x (`views` 5: +x, +y, -x, -y).
pub fn view_offsets(n: usize, baseline_m: f64, axes: Axes) -> Vec<[f64; 2]> {
    let clean = |v: f64| if v.abs() < 1e-9 { 0.0 } else { v };
    match axes {
        Axes::X => (0..n).map(|i| [clean(-baseline_m / 2.0 + baseline_m * i as f64 / (n - 1).max(1) as f64), 0.0]).collect(),
        Axes::Xy => std::iter::once([0.0, 0.0]).chain((0..n.saturating_sub(1)).map(|k| {
            let a = 2.0 * std::f64::consts::PI * k as f64 / (n - 1) as f64;
            [clean(baseline_m / 2.0 * a.cos()), clean(baseline_m / 2.0 * a.sin())]
        })).collect(),
    }
}

fn is_start(o: &[f64; 2]) -> bool { o[0] == 0.0 && o[1] == 0.0 }
fn dist2(a: &[f64; 2], b: &[f64; 2]) -> f64 { (a[0] - b[0]).hypot(a[1] - b[1]) }

/// The order the views are visited from the start pose: the start view first; then for offsets on
/// x the negative side outward and the positive side outward, for offsets on a circle by angle.
pub fn visit_order(offsets: &[[f64; 2]]) -> Vec<usize> {
    let mut idx: Vec<usize> = (0..offsets.len()).collect();
    let on_x = offsets.iter().all(|o| o[1] == 0.0);
    idx.sort_by(|&a, &b| {
        let key = |o: &[f64; 2]| -> (u8, f64) {
            if is_start(o) { (0, 0.0) } else if on_x { (if o[0] < 0.0 { 1 } else { 2 }, o[0].abs()) } else { (1, o[1].atan2(o[0]).rem_euclid(2.0 * std::f64::consts::PI)) }
        };
        let (ka, kb) = (key(&offsets[a]), key(&offsets[b]));
        ka.0.cmp(&kb.0).then(ka.1.total_cmp(&kb.1))
    });
    idx
}

/// Offsets to try after `o` was refused, given the offsets already planned or captured: the
/// mirrored one, then 0.75 and 0.5 of it on either side, each when no other view is within 4 mm.
pub fn fallback_offsets(o: [f64; 2], used: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::new();
    let free = |c: [f64; 2], out: &[[f64; 2]]| c[0].hypot(c[1]) >= 0.005 && used.iter().chain(out).all(|u| dist2(u, &c) > 0.004);
    let m = [-o[0], -o[1]];
    if free(m, &out) { out.push(m); }
    for s in FALLBACK_SCALES {
        for c in [[o[0] * s, o[1] * s], [-o[0] * s, -o[1] * s]] { if free(c, &out) { out.push(c); } }
    }
    out
}

/// Largest distance between two offsets (the effective baseline).
pub fn span(offsets: &[[f64; 2]]) -> f64 {
    offsets.iter().enumerate().flat_map(|(i, a)| offsets[i + 1..].iter().map(move |b| dist2(a, b))).fold(0.0, f64::max)
}

/// The image-plane axes of the start camera in the base frame: x, y (unit, orthogonal to the ray
/// from the camera to the pivot for `Orbit`; the camera's own axes for `Translate`).
pub fn plane_axes(cam: &Camera, pivot: &Vec3, motion: Motion) -> (Vec3, Vec3) {
    let (xc, yc) = (kinematics::column(&cam.r, 0), kinematics::column(&cam.r, 1));
    match motion {
        Motion::Translate => (xc, yc),
        Motion::Orbit => {
            let ray = kinematics::unit(&kinematics::sub(pivot, &cam.c));
            let orth = |v: &Vec3| { let k = kinematics::dot(&ray, v); kinematics::unit(&kinematics::sub(v, &[ray[0] * k, ray[1] * k, ray[2] * k])) };
            let lx = orth(&xc);
            (lx, kinematics::cross(&ray, &lx))
        }
    }
}

/// The view pose for `offset` (see [`Motion`]). `axes` from [`plane_axes`].
pub fn view_pose(pose: &Pose, cam_c: &Vec3, pivot: &Vec3, axes: &(Vec3, Vec3), offset: [f64; 2], motion: Motion) -> Pose {
    let d = [offset[0] * axes.0[0] + offset[1] * axes.1[0], offset[0] * axes.0[1] + offset[1] * axes.1[1], offset[0] * axes.0[2] + offset[1] * axes.1[2]];
    let mag = kinematics::norm(&d);
    if mag == 0.0 { return *pose; }
    match motion {
        Motion::Translate => Pose::new(pose.r, kinematics::add(&pose.p, &d)),
        Motion::Orbit => orbit_pose(pose, cam_c, pivot, &kinematics::unit(&d), mag),
    }
}

/// The pose `pose` turned about `pivot` so that the camera centre `cam_c` moves by the chord
/// `offset_m` along `lat` (unit, perpendicular to the ray from the camera to the pivot).
pub fn orbit_pose(pose: &Pose, cam_c: &Vec3, pivot: &Vec3, lat: &Vec3, offset_m: f64) -> Pose {
    if offset_m == 0.0 { return *pose; }
    let ray = kinematics::unit(&kinematics::sub(pivot, cam_c));
    let d = kinematics::norm(&kinematics::sub(pivot, cam_c));
    let axis = kinematics::unit(&kinematics::cross(lat, &ray));
    let theta = 2.0 * (offset_m / (2.0 * d)).clamp(-1.0, 1.0).asin();
    let r = kinematics::axis_r(&axis, theta);
    Pose::new(kinematics::mat_mul(&r, &pose.r), kinematics::add(pivot, &kinematics::mat_vec(&r, &kinematics::sub(&pose.p, pivot))))
}

/// Camera of the stereo geometry at the end_link pose `pose`.
pub fn camera_at(cam: &CamModel, pose: &Pose) -> Camera {
    Camera { r: cam.rotation(pose), c: cam.origin(pose), fx: cam.fx, fy: cam.fy, cx: cam.cx, cy: cam.cy }
}

/// Decode a snapshot JPEG to the camera model's grayscale frame. The snapshot is the frame
/// letterboxed into a square (rebot-vision: 640x640 with the 640x360 frame in rows 140..499): a
/// width equal to the model's and a taller image is cropped to the centred rows.
pub fn decode_frame(jpeg: &[u8], size: [f64; 2]) -> Result<Gray, String> {
    use zune_jpeg::zune_core::{bytestream::ZCursor, colorspace::ColorSpace, options::DecoderOptions};
    let opts = DecoderOptions::default().jpeg_set_out_colorspace(ColorSpace::Luma).set_max_width(4096).set_max_height(4096);
    let mut dec = zune_jpeg::JpegDecoder::new_with_options(ZCursor::new(jpeg), opts);
    let px = dec.decode().map_err(|e| format!("snapshot is not a JPEG: {e:?}"))?;
    let info = dec.info().ok_or("snapshot without a JPEG header")?;
    let (w, h) = (usize::from(info.width), usize::from(info.height));
    if px.len() != w * h { return Err(format!("snapshot decoded to {} bytes for {w}x{h} grayscale", px.len())); }
    let (cw, ch) = (size[0].round() as usize, size[1].round() as usize);
    if w != cw || h < ch {
        return Err(format!("snapshot is {w}x{h}; the camera model (grasp setup image_size_px) is {cw}x{ch}"));
    }
    let top = (h - ch) / 2;
    Gray::new(cw, ch, px[top * w..(top + ch) * w].to_vec()).ok_or_else(|| "snapshot crop failed".to_string())
}

/// A checked run, ready to begin.
#[derive(Debug, Clone)]
pub struct Plan {
    pub request: Request,
    pub epoch: u64,
    pub workspace_revision: u64,
    pub calibration_revision: u64,
    pub setup: GraspSetup,
    pub cam: CamModel,
    /// Commanded joints at the start (all joints).
    pub start: Vec<f32>,
    pub arm_dof: usize,
    pub deadline_s: f64,
}

/// Check the run under the lock.
pub fn prepare(inner: &SupervisorInner, body: &Value, epoch: u64) -> Result<Plan, ArmError> {
    let request = parse_request(body)?;
    if epoch != inner.safety.epoch_id() {
        return Err(crate::safety::SafetyError::EpochMismatch { expected: inner.safety.epoch_id(), received: epoch }.into());
    }
    let Some(setup) = inner.grasp_setup() else {
        return Err(MotionError::Unsupported("measure_multiview needs a profile with kinematics, a gripper policy and a grasp setup (camera intrinsics)".into()).into());
    };
    inner.plan_input()?;
    if !inner.is_torqued { return Err(crate::safety::SafetyError::NotArmed(inner.safety.state().clone()).into()); }
    let cam = CamModel::assumed(&inner.config.arm_model, &setup, &inner.config.camera_mount);
    // bound: per view one joint move (up to 2 rad at the slowest planning speed), settle, captures
    let vmin = inner.planning_limits().iter().take(inner.config.arm_dof()).map(|l| l.max_velocity).fold(f32::INFINITY, f32::min).max(0.05);
    let move_s = 2.0 / (f64::from(vmin) * request.speed_scale) + crate::motion::SETTLE_MAX_S;
    // a snapshot takes at most 3 s (3.5 s budgeted); per try: the triggers, the refresh polls
    // (the last one may start just before REFRESH_MAX) and the retry gap
    let try_s = SNAPSHOT_GAP_RETRY.as_secs_f64() + TRIGGERS as f64 * (TRIGGER_WAIT_MS as f64 / 1000.0 + 2.0)
        + 3.5 + REFRESH_MAX.as_secs_f64() + 3.5;
    let capture_s = MAX_SETTLE_S + 3.5 + SNAPSHOT_TRIES as f64 * try_s;
    let moves = request.views + 1 + 2 * FALLBACK_SCALES.len();
    let deadline_s = moves as f64 * move_s + request.views as f64 * capture_s + 30.0;
    Ok(Plan {
        request, epoch, workspace_revision: inner.workspace_revision, calibration_revision: inner.calibration_revision,
        setup, cam, start: inner.commanded_positions.clone(), arm_dof: inner.config.arm_dof(), deadline_s,
    })
}

/// Begin the action and spawn its orchestrator. Returns (action id, deadline bound s).
pub fn begin(inner: &mut SupervisorInner, sup: &Arc<ArmSupervisor>, plan: Plan, source: Arc<dyn DetectionSource>) -> Result<(u64, f64), ArmError> {
    let id = inner.begin_orchestrated_action("measure_multiview", None, plan.epoch)?;
    let deadline = plan.deadline_s;
    tracing::info!("measure_multiview {id} started ({} views, baseline {:.3} m, speed_scale {:.2}, bound {deadline:.0} s)",
        plan.request.views, plan.request.baseline_m, plan.request.speed_scale);
    let sup = sup.clone();
    tokio::spawn(async move {
        let worker = { let sup = sup.clone(); tokio::spawn(async move { run(sup, id, plan, source).await }) };
        if let Err(e) = worker.await {
            tracing::error!("measure_multiview {id} orchestrator failed: {e}");
            sup.inner().write().await.finish_macro(id, "failed", Some("INTERNAL"), Some(format!("orchestrator failed: {e}")), None, None);
        }
    });
    Ok((id, deadline))
}

/// Alive, nothing the run was checked against changed, telemetry fresh, torque on.
async fn recheck(ctx: &MacroCtx, plan: &Plan) -> Result<(), Stop> {
    let mut inner = ctx.sup.inner().write().await;
    if !inner.macro_alive(ctx.id) { return Err(Stop::Cancelled); }
    if inner.safety.epoch_id() != plan.epoch { return Err(fail("PLAN_STALE", "the epoch changed during the measurement (stop or e-stop)")); }
    if inner.workspace_revision != plan.workspace_revision { return Err(fail("PLAN_STALE", "the workspace, keep-out boxes or camera mount changed during the measurement")); }
    if inner.calibration_revision != plan.calibration_revision { return Err(fail("CALIBRATION_CHANGED", "the joint zero or the kinematic correction changed during the measurement")); }
    if inner.active_lease.as_ref().map(|l| l.generation) != inner.action_lease_generation {
        let id = ctx.id;
        inner.finish_macro(id, "cancelled", Some("LEASE_LOST"), Some("Control lease lost or released".into()), None, None);
        return Err(Stop::Cancelled);
    }
    if !inner.is_torqued { return Err(fail("NOT_ARMED", "torque went off during the measurement")); }
    if !inner.hardware_feedback_fresh(FEEDBACK_MAX_AGE) { return Err(fail("STALE_TELEMETRY", "joint feedback is stale")); }
    if Instant::now() > ctx.deadline { return Err(fail(ctx.timeout_code, ctx.timeout_message.clone())); }
    Ok(())
}

/// Measured joints (all) and velocities (arm joints).
async fn sample(ctx: &MacroCtx) -> (Vec<f32>, Vec<f32>) {
    let inner = ctx.sup.inner().read().await;
    let arm = inner.config.arm_joint_indices();
    (inner.measured_positions.clone(), arm.iter().map(|&i| inner.measured_velocities.get(i).copied().unwrap_or(0.0)).collect())
}

/// Measured joints (all), per-arm-joint feedback instants, and per-arm-joint monotonic feedback
/// sequence counters, read coherently under one lock.
async fn sample_with_timing(ctx: &MacroCtx) -> (Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
    let inner = ctx.sup.inner().read().await;
    let (pos, _, instants, seqs) = inner.arm_feedback_snapshot();
    (pos, instants, seqs)
}

/// Measured joints (all) and per-arm-joint feedback instants, read coherently under one lock.
async fn sample_with_instants(ctx: &MacroCtx) -> (Vec<f32>, Vec<Option<Instant>>) {
    let (pos, instants, _) = sample_with_timing(ctx).await;
    (pos, instants)
}

/// Maximum feedback age (ms) among arm joints at `now`.
fn max_feedback_age_ms(now: Instant, instants: &[Option<Instant>]) -> Option<f64> {
    if instants.is_empty() || instants.iter().any(Option::is_none) {
        return None;
    }
    instants
        .iter()
        .flatten()
        .map(|t| now.saturating_duration_since(*t).as_secs_f64() * 1000.0)
        .fold(None, |acc: Option<f64>, v| Some(acc.map_or(v, |a| a.max(v))))
}

/// Wait until every arm joint has a hardware feedback sample that was enqueued after `after`,
/// bounded by [`FEEDBACK_WAIT_MAX`] and the action deadline / `recheck` invariants.
///
/// Decode instants and sequence counters alone do not prove that: the Damiao driver stamps and
/// counts at decode, and its bounded per-tick drain can decode a pre-fence backlog (several old
/// replies per joint) ticks later. A sample therefore also needs a receive-queue bound at or
/// after `after` ([`ArmDriver::feedback_queue_bounds`]: the driver saw its RX queue empty at
/// that instant before reading the sample). Drivers without a receive queue (Feetech/SO-ARM101,
/// FashionStar request/response, sim, driverless mock) report none; their receive instant is
/// the bound. The sequence counter must still advance by at least 2 beyond `fence_seqs`.
/// Returns `(positions, arm_feedback_instants, wait_ms)`.
///
/// [`ArmDriver::feedback_queue_bounds`]: crate::driver::ArmDriver::feedback_queue_bounds
async fn sample_fresh_after(
    ctx: &MacroCtx,
    plan: &Plan,
    after: Instant,
    fence_seqs: &[u64],
) -> Result<(Vec<f32>, Vec<Option<Instant>>, f64), Stop> {
    let wait_start = Instant::now();
    loop {
        recheck(ctx, plan).await?;
        let (pos, instants, seqs, bounds) = {
            let inner = ctx.sup.inner().read().await;
            let (pos, _, instants, seqs) = inner.arm_feedback_snapshot();
            (pos, instants, seqs, inner.arm_feedback_queue_bounds())
        };
        let ready = !instants.is_empty()
            && instants.len() == fence_seqs.len()
            && seqs.len() == fence_seqs.len()
            && bounds.len() == fence_seqs.len()
            && instants
                .iter()
                .zip(bounds.iter())
                .zip(seqs.iter().zip(fence_seqs.iter()))
                .all(|((t, q), (&seq, &base))| {
                    t.is_some_and(|ts| ts >= after) && q.is_some_and(|qb| qb >= after) && seq.wrapping_sub(base) >= 2
                });
        if ready {
            return Ok((pos, instants, wait_start.elapsed().as_secs_f64() * 1000.0));
        }
        if wait_start.elapsed() >= FEEDBACK_WAIT_MAX {
            return Err(fail("STALE_TELEMETRY", "joint feedback did not refresh for every arm joint after capture"));
        }
        tokio::time::sleep(POLL).await;
    }
}

/// Wait until the arm is still (see [`STILL_WINDOW`]): `min_s`, then sliding windows until
/// `min_s` + ([`MAX_SETTLE_S`] - [`MIN_SETTLE_S`]). Returns the seconds it took. The start view
/// passes 0 (the arm stood at the start pose before the action; the still windows and the
/// bracket check still apply), the view poses [`MIN_SETTLE_S`].
async fn settle(ctx: &MacroCtx, plan: &Plan, min_s: f64) -> Result<f64, Stop> {
    let t0 = Instant::now();
    let until = t0 + Duration::from_secs_f64(min_s);
    let max_s = min_s + (MAX_SETTLE_S - MIN_SETTLE_S);
    while Instant::now() < until {
        ctx.check().await?;
        tokio::time::sleep(POLL).await;
    }
    recheck(ctx, plan).await?;
    let arm = { ctx.sup.inner().read().await.config.arm_joint_indices() };
    let mut win: std::collections::VecDeque<(Vec<f32>, Vec<f32>)> = std::collections::VecDeque::new();
    loop {
        ctx.check().await?;
        win.push_back(sample(ctx).await);
        if win.len() > STILL_WINDOW { win.pop_front(); }
        if win.len() == STILL_WINDOW && is_still(&win, &arm) { return Ok(t0.elapsed().as_secs_f64()); }
        if t0.elapsed().as_secs_f64() > max_s {
            return Err(fail("NOT_SETTLED", format!("the arm did not come to rest within {max_s} s (median |velocity| <= {STILL_VEL} rad/s and range <= {STILL_RANGE_RAD} rad over {STILL_WINDOW} samples)")));
        }
        tokio::time::sleep(STILL_SAMPLE).await;
    }
}

/// The settle rule over a window of (positions, arm velocities) samples.
pub fn is_still(win: &std::collections::VecDeque<(Vec<f32>, Vec<f32>)>, arm: &[usize]) -> bool {
    arm.iter().enumerate().all(|(k, &j)| {
        let mut v: Vec<f32> = win.iter().map(|s| s.1.get(k).copied().unwrap_or(0.0).abs()).collect();
        v.sort_by(f32::total_cmp);
        let (lo, hi) = win.iter().filter_map(|s| s.0.get(j)).fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), &x| (a.min(x), b.max(x)));
        v[v.len() / 2] <= STILL_VEL && hi - lo <= STILL_RANGE_RAD
    })
}

/// Largest difference of the arm joints between two samples (rad), and that joint's index.
pub fn bracket_spread(a: &[f32], b: &[f32], arm: &[usize]) -> (f32, usize) {
    arm.iter().map(|&i| ((a.get(i).copied().unwrap_or(0.0) - b.get(i).copied().unwrap_or(0.0)).abs(), i))
        .fold((0.0, arm.first().copied().unwrap_or(0)), |m, x| if x.0 > m.0 { x } else { m })
}

/// Wall-clock phases of one view (ms), `views[].timing`: the view motion including its planning
/// and refused fallback plans (0 for the start view), the settle wait, the time inside detection
/// triggers and inside frame fetches (gray frame, or the snapshot before the trigger plus the wait
/// for the refreshed one), all tries summed, and the capture from the end of the settle wait to the
/// accepted frame (triggers, fetches, bracket samples, retry gaps, decode).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ViewTiming {
    pub motion_ms: f64,
    pub settle_ms: f64,
    pub trigger_ms: f64,
    pub fetch_ms: f64,
    pub capture_ms: f64,
}

impl ViewTiming {
    pub fn to_json(&self) -> Value {
        let r1 = |x: f64| (x * 10.0).round() / 10.0;
        json!({"motion_ms": r1(self.motion_ms), "settle_ms": r1(self.settle_ms), "trigger_ms": r1(self.trigger_ms),
            "fetch_ms": r1(self.fetch_ms), "capture_ms": r1(self.capture_ms)})
    }
}

/// Wall-clock phases of the action (ms), `measurement.timing`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RunTiming {
    /// `GET /api/mode` of the vision provider (and the camera hold, when used).
    pub negotiate_ms: f64,
    /// The start view: settle check, trigger and fetch, until the target is picked.
    pub detect_ms: f64,
    /// From the start capture to the last view capture (motions, settles, captures).
    pub views_ms: f64,
    /// The return to the start joints (0 without `return_to_start`).
    pub return_ms: f64,
    /// The reconstruction (in `spawn_blocking`), and how much of it ran during the return.
    pub compute_ms: f64,
    pub overlap_ms: f64,
    /// From the start of the run until the measurement is complete (after the return and the
    /// reconstruction; the debug bundle is written afterwards).
    pub total_ms: f64,
}

/// Length (ms) of the intersection of two time intervals.
pub fn overlap_ms(a: (Instant, Instant), b: (Instant, Instant)) -> f64 {
    let (lo, hi) = (a.0.max(b.0), a.1.min(b.1));
    if hi > lo { (hi - lo).as_secs_f64() * 1000.0 } else { 0.0 }
}

/// `measurement.timing`: the action phases and the per-view phases summed over the views.
pub fn timing_value(run: &RunTiming, views: &[ViewTiming]) -> Value {
    let r1 = |x: f64| (x * 10.0).round() / 10.0;
    let sum = |f: fn(&ViewTiming) -> f64| r1(views.iter().map(f).sum::<f64>());
    json!({
        "negotiate_ms": r1(run.negotiate_ms), "detect_ms": r1(run.detect_ms), "views_ms": r1(run.views_ms),
        "motion_ms": sum(|v| v.motion_ms), "settle_ms": sum(|v| v.settle_ms), "trigger_ms": sum(|v| v.trigger_ms),
        "fetch_ms": sum(|v| v.fetch_ms), "capture_ms": sum(|v| v.capture_ms),
        "return_ms": r1(run.return_ms), "compute_ms": r1(run.compute_ms), "compute_return_overlap_ms": r1(run.overlap_ms),
        "total_ms": r1(run.total_ms),
    })
}

/// One view's capture.
struct Capture {
    offset: [f64; 2],
    pose: Pose,
    img: Gray,
    frame_source: FrameSource,
    instance_id: Option<String>,
    frame_id: u64,
    detections: Vec<Detection>,
    decode_ms: f64,
    frame_fetch_ms: f64,
    gray_convert_ms: Option<f64>,
    parse_ms: f64,
    /// Predicted shift of the look-at point from its start pixel, and the smallest distance of
    /// the predicted target box to the image border (px).
    shift_px: [f64; 2],
    margin_px: f64,
    /// Joints of the pose (bracket mean), the bracket samples, the bracket spread of every
    /// bracketed try (rad), bracket timing metrics, retry reasons, and the settle time (s).
    joints: Vec<f32>,
    bracket: [Vec<f32>; 2],
    snapshot_tries: Vec<f32>,
    retry_reasons: Vec<String>,
    bracket_duration_ms: f64,
    feedback_wait_b_ms: f64,
    feedback_age_a_ms: Option<f64>,
    feedback_age_b_ms: Option<f64>,
    settle_s: f64,
    /// Accepted try: the trigger's time, the time from its reply to the refreshed snapshot (ms)
    /// and the snapshots that still returned the frame from before the trigger (0 for gray).
    trigger_ms: f64,
    snapshot_wait_ms: f64,
    snapshot_polls: usize,
    /// The snapshot (debug bundles only).
    jpeg: Option<Vec<u8>>,
    /// ArUco markers of the trigger frame (camera model pixels); empty unless requested.
    markers: Vec<mview_stereo::Marker>,
    timing: ViewTiming,
}

fn source_fail(e: impl std::fmt::Display) -> Stop { fail("DETECTION_UNAVAILABLE", e.to_string()) }

fn rescale_markers_and_frame(plan: &Plan, frame: &mut grasp::DetectionFrame, marks: &mut [mview_stereo::Marker]) {
    // marker corners are pixels of the frame: to the camera model's image like the detections
    let fs = frame.image_size_px;
    if fs[0] > 0.0 && fs[1] > 0.0 {
        let (rx, ry) = (plan.cam.size[0] / fs[0], plan.cam.size[1] / fs[1]);
        for m in marks {
            for c in &mut m.corners {
                c[0] *= rx;
                c[1] *= ry;
            }
        }
    }
    grasp::rescale_frame_to_cam(frame, &plan.cam);
}

/// Settle, then capture one view via the run's negotiated [`FrameSource`] (never switched mid-run).
async fn capture(
    ctx: &MacroCtx,
    plan: &Plan,
    src: &dyn DetectionSource,
    frame_source: FrameSource,
    negotiated_gray: Option<&grasp::GrayFormatDescriptor>,
    offset: [f64; 2],
    shift_px: [f64; 2],
    margin_px: f64,
    markers: bool,
    min_settle_s: f64,
) -> Result<Capture, Stop> {
    match frame_source {
        FrameSource::Gray => capture_gray(ctx, plan, src, negotiated_gray, offset, shift_px, margin_px, markers, min_settle_s).await,
        FrameSource::Snapshot => capture_snapshot(ctx, plan, src, offset, shift_px, margin_px, markers, min_settle_s).await,
    }
}

/// Raw gray capture using the conservative bracket:
/// sample A is taken right before the trigger request, and sample B AFTER the gray frame has been
/// fetched, waiting for a fresh hardware joint feedback update acquired after the fetch completed
/// for every arm joint.
///
/// Rationale: the trigger returns a frame newer than the request; any pipeline latency before A is
/// at most ~1 frame and the arm has been settled for >= 2 s with drift of a few mrad/s, so the
/// residual is far below the bracket limit (5 mrad with pose_refine, 2 mrad off).
async fn capture_gray(
    ctx: &MacroCtx,
    plan: &Plan,
    src: &dyn DetectionSource,
    negotiated_gray: Option<&grasp::GrayFormatDescriptor>,
    offset: [f64; 2],
    shift_px: [f64; 2],
    margin_px: f64,
    markers: bool,
    min_settle_s: f64,
) -> Result<Capture, Stop> {
    let settle_s = settle(ctx, plan, min_settle_s).await?;
    let t_cap = Instant::now();
    let (mut trig_total_ms, mut fetch_total_ms) = (0.0, 0.0);
    let arm = { ctx.sup.inner().read().await.config.arm_joint_indices() };
    let bracket_limit = if plan.request.pose_refine == PoseRefine::Off { BRACKET_RAD } else { BRACKET_REFINED_RAD };
    let pipelines = if markers { YOLO_ARUCO_GRAY } else { YOLO_GRAY };
    let (cw, ch) = exact_camera_dims(plan.cam.size, "camera model (grasp setup image_size_px)")
        .map_err(|e| fail("DETECTION_UNAVAILABLE", e))?;
    let mut tries: Vec<(f32, usize)> = Vec::new();
    let mut retry_reasons: Vec<String> = Vec::new();
    let mut last_acq_err: Option<String> = None;
    let mut got = None;

    for k in 0..SNAPSHOT_TRIES {
        if k > 0 { tokio::time::sleep(SNAPSHOT_GAP_RETRY).await; }
        // Take sample A immediately before EACH trigger attempt so a timed-out trigger never
        // leaves an outdated sample A paired with a later attempt's frame and sample B.
        let mut trig = None;
        for _ in 0..TRIGGERS {
            recheck(ctx, plan).await?;
            let t_a = Instant::now();
            let (a, inst_a) = sample_with_instants(ctx).await;
            let age_a_ms = max_feedback_age_ms(t_a, &inst_a);
            let t_trigger = Instant::now();
            let res = src.trigger_gray(pipelines, TRIGGER_WAIT_MS).await.map_err(|e| source_fail(format!("trigger: {e}")))?;
            let call_ms = t_trigger.elapsed().as_secs_f64() * 1000.0;
            trig_total_ms += call_ms;
            if let Some(r) = res {
                trig = Some((r, t_a, a, age_a_ms, call_ms));
                break;
            }
        }
        let (trig, t_a, a, age_a_ms, trigger_ms) =
            trig.ok_or_else(|| fail("DETECTION_UNAVAILABLE", format!("no detection frame in {TRIGGERS} triggers")))?;
        let Some(instance_id) = trig.instance_id.clone().filter(|s| !s.is_empty()) else {
            return Err(fail("DETECTION_UNAVAILABLE", "gray trigger reply missing instance_id"));
        };
        if !trig.gray_ran {
            retry_reasons.push("gray_not_produced".into());
            last_acq_err = Some(format!("gray frame {}: provider did not produce gray image", trig.frame.frame_id));
            continue;
        }
        let t_fetch = Instant::now();
        let fetched = src.gray_frame(&instance_id, trig.frame.frame_id).await;
        fetch_total_ms += t_fetch.elapsed().as_secs_f64() * 1000.0;
        let gray = match fetched {
            Ok(g) => g,
            Err(e) if e.is_retryable() => {
                retry_reasons.push(e.retry_reason().into());
                last_acq_err = Some(e.to_string());
                continue;
            }
            Err(e) => return Err(fail("DETECTION_UNAVAILABLE", e.to_string())),
        };
        let frame_fetch_ms = gray.fetch_ms;
        if gray.img.w != cw || gray.img.h != ch {
            return Err(fail("DETECTION_UNAVAILABLE", format!(
                "gray frame is {}x{}; the camera model (grasp setup image_size_px) is {cw}x{ch}",
                gray.img.w, gray.img.h
            )));
        }
        if let Some(fmt) = negotiated_gray {
            if gray.img.w != fmt.width || gray.img.h != fmt.height {
                return Err(fail("DETECTION_UNAVAILABLE", format!(
                    "gray frame {}x{} does not match negotiated {}x{}",
                    gray.img.w, gray.img.h, fmt.width, fmt.height
                )));
            }
            if let Some(exp_fs) = fmt.frame_size {
                if gray.frame_size != exp_fs {
                    return Err(fail("DETECTION_UNAVAILABLE", format!(
                        "gray frame source size {:?} does not match negotiated {:?}",
                        gray.frame_size, exp_fs
                    )));
                }
            }
            if let Some(exp_crop) = fmt.crop {
                if gray.crop != exp_crop {
                    return Err(fail("DETECTION_UNAVAILABLE", format!(
                        "gray frame crop {:?} does not match negotiated {:?}",
                        gray.crop, exp_crop
                    )));
                }
            }
        }
        let t_fetch_done = Instant::now();
        let (_, _, fence_seqs) = sample_with_timing(ctx).await;
        let (b, inst_b, feedback_wait_b_ms) = sample_fresh_after(ctx, plan, t_fetch_done, &fence_seqs).await?;
        let t_b = Instant::now();
        let bracket_duration_ms = (t_b - t_a).as_secs_f64() * 1000.0;
        let age_b_ms = max_feedback_age_ms(t_b, &inst_b);
        let (spread, joint) = bracket_spread(&a, &b, &arm);
        tries.push((spread, joint));
        if spread <= bracket_limit {
            let mean: Vec<f32> = a.iter().zip(&b).map(|(x, y)| 0.5 * (x + y)).collect();
            got = Some((gray, instance_id, mean, a, b, trig, trigger_ms, frame_fetch_ms, bracket_duration_ms, feedback_wait_b_ms, age_a_ms, age_b_ms));
            break;
        }
        retry_reasons.push("moved".into());
        last_acq_err = None;
    }

    let Some((gray, instance_id, joints, bracket_a, bracket_b, mut trig, trigger_ms, frame_fetch_ms, bracket_duration_ms, feedback_wait_b_ms, feedback_age_a_ms, feedback_age_b_ms)) = got else {
        if let Some(err) = last_acq_err {
            return Err(fail("DETECTION_UNAVAILABLE", err));
        }
        let mrad: Vec<String> = tries.iter().map(|(t, j)| format!("J{} {:.1}", j + 1, t * 1000.0)).collect();
        return Err(fail("MOVED", format!("the arm moved during {SNAPSHOT_TRIES} captures (largest joint change across each trigger and gray fetch: {} mrad, limit {:.1})", mrad.join(", "), bracket_limit * 1000.0)));
    };
    recheck(ctx, plan).await?;
    let pose = { ctx.sup.inner().read().await.config.arm_model.fk_f32(&joints) }
        .ok_or_else(|| fail("INTERNAL", "no FK pose for the measured joints"))?;
    rescale_markers_and_frame(plan, &mut trig.frame, &mut trig.markers);
    let convert_ms = trig.gray_ms.or(Some(gray.convert_ms));
    Ok(Capture {
        offset, pose, img: gray.img, frame_source: FrameSource::Gray, instance_id: Some(instance_id),
        frame_id: trig.frame.frame_id, detections: trig.frame.detections, decode_ms: 0.0,
        frame_fetch_ms, gray_convert_ms: convert_ms, parse_ms: gray.parse_ms, shift_px, margin_px,
        joints, bracket: [bracket_a, bracket_b], snapshot_tries: tries.iter().map(|t| t.0).collect(),
        retry_reasons, bracket_duration_ms, feedback_wait_b_ms, feedback_age_a_ms, feedback_age_b_ms,
        settle_s, trigger_ms, snapshot_wait_ms: 0.0, snapshot_polls: 0, jpeg: None, markers: trig.markers,
        timing: ViewTiming { motion_ms: 0.0, settle_ms: settle_s * 1000.0, trigger_ms: trig_total_ms, fetch_ms: fetch_total_ms,
            capture_ms: t_cap.elapsed().as_secs_f64() * 1000.0 },
    })
}

/// Legacy JPEG snapshot path (unchanged behaviour for older or remote providers).
async fn capture_snapshot(
    ctx: &MacroCtx,
    plan: &Plan,
    src: &dyn DetectionSource,
    offset: [f64; 2],
    shift_px: [f64; 2],
    margin_px: f64,
    markers: bool,
    min_settle_s: f64,
) -> Result<Capture, Stop> {
    let settle_s = settle(ctx, plan, min_settle_s).await?;
    let t_cap = Instant::now();
    let snap = || async { src.snapshot_jpeg().await.map_err(|e| source_fail(format!("snapshot: {e}"))) };
    // possibly a frame from before the motion settled (the idle camera runs at 1 fps)
    let mut s0 = snap().await?;
    let (mut trig_total_ms, mut fetch_total_ms) = (0.0, t_cap.elapsed().as_secs_f64() * 1000.0);
    let arm = { ctx.sup.inner().read().await.config.arm_joint_indices() };
    let mut tries: Vec<(f32, usize)> = Vec::new();
    let mut retry_reasons: Vec<String> = Vec::new();
    let bracket_limit = if plan.request.pose_refine == PoseRefine::Off { BRACKET_RAD } else { BRACKET_REFINED_RAD };
    let mut got = None;
    for k in 0..SNAPSHOT_TRIES {
        if k > 0 { tokio::time::sleep(SNAPSHOT_GAP_RETRY).await; }
        let mut frame = None;
        for _ in 0..TRIGGERS {
            recheck(ctx, plan).await?;
            let t_a = Instant::now();
            let (a, inst_a) = sample_with_instants(ctx).await;
            let age_a_ms = max_feedback_age_ms(t_a, &inst_a);
            let t_trigger = Instant::now();
            let got = if markers { src.trigger_markers(YOLO_ARUCO, TRIGGER_WAIT_MS).await } else { src.trigger(YOLO, TRIGGER_WAIT_MS).await.map(|f| f.map(|f| (f, Vec::new()))) };
            trig_total_ms += t_trigger.elapsed().as_secs_f64() * 1000.0;
            if let Some(f) = got.map_err(|e| source_fail(format!("trigger: {e}")))? {
                let t_reply = Instant::now();
                frame = Some((f, t_a, a, age_a_ms, t_reply, (t_reply - t_trigger).as_secs_f64() * 1000.0));
                break;
            }
        }
        let (frame, t_a, a, age_a_ms, t_reply, trigger_ms) =
            frame.ok_or_else(|| fail("DETECTION_UNAVAILABLE", format!("no detection frame in {TRIGGERS} triggers")))?;
        let mut polls = 0;
        let jpeg = loop {
            let j = snap().await?;
            if j != s0 { break j; }
            polls += 1;
            if t_reply.elapsed() >= REFRESH_MAX {
                return Err(fail("DETECTION_UNAVAILABLE", format!("camera snapshot did not refresh within {} ms after a detection trigger ({polls} snapshots returned the frame from before the trigger)", REFRESH_MAX.as_millis())));
            }
            tokio::time::sleep(REFRESH_POLL).await;
            ctx.check().await?;
        };
        let wait_ms = t_reply.elapsed().as_secs_f64() * 1000.0;
        fetch_total_ms += wait_ms;
        let t_b = Instant::now();
        let (b, inst_b) = sample_with_instants(ctx).await;
        let bracket_duration_ms = (t_b - t_a).as_secs_f64() * 1000.0;
        let age_b_ms = max_feedback_age_ms(t_b, &inst_b);
        let (spread, joint) = bracket_spread(&a, &b, &arm);
        tries.push((spread, joint));
        if spread <= bracket_limit {
            let mean: Vec<f32> = a.iter().zip(&b).map(|(x, y)| 0.5 * (x + y)).collect();
            got = Some((jpeg, mean, a, b, frame, trigger_ms, wait_ms, polls, bracket_duration_ms, age_a_ms, age_b_ms));
            break;
        }
        retry_reasons.push("moved".into());
        s0 = jpeg;
    }
    let Some((jpeg, joints, bracket_a, bracket_b, (mut frame, mut marks), trigger_ms, snapshot_wait_ms, snapshot_polls, bracket_duration_ms, feedback_age_a_ms, feedback_age_b_ms)) = got else {
        let mrad: Vec<String> = tries.iter().map(|(t, j)| format!("J{} {:.1}", j + 1, t * 1000.0)).collect();
        return Err(fail("MOVED", format!("the arm moved during {SNAPSHOT_TRIES} snapshots (largest joint change across each trigger and snapshot: {} mrad, limit {:.1})", mrad.join(", "), bracket_limit * 1000.0)));
    };
    recheck(ctx, plan).await?;
    let pose = { ctx.sup.inner().read().await.config.arm_model.fk_f32(&joints) }
        .ok_or_else(|| fail("INTERNAL", "no FK pose for the measured joints"))?;
    rescale_markers_and_frame(plan, &mut frame, &mut marks);
    let size = plan.cam.size;
    let keep = plan.request.debug.then(|| jpeg.clone());
    let (img, decode_ms) = tokio::task::spawn_blocking(move || { let t0 = Instant::now(); let r = decode_frame(&jpeg, size); (r, t0.elapsed().as_secs_f64() * 1000.0) }).await
        .map_err(|e| fail("INTERNAL", format!("decode task failed: {e}")))?;
    let img = img.map_err(|e| fail("DETECTION_UNAVAILABLE", e))?;
    Ok(Capture {
        offset, pose, img, frame_source: FrameSource::Snapshot, instance_id: None,
        frame_id: frame.frame_id, detections: frame.detections, decode_ms, frame_fetch_ms: 0.0,
        gray_convert_ms: None, parse_ms: 0.0, shift_px, margin_px, joints,
        bracket: [bracket_a, bracket_b], snapshot_tries: tries.iter().map(|t| t.0).collect(),
        retry_reasons, bracket_duration_ms, feedback_wait_b_ms: 0.0, feedback_age_a_ms, feedback_age_b_ms,
        settle_s, trigger_ms, snapshot_wait_ms, snapshot_polls, jpeg: keep, markers: marks,
        timing: ViewTiming { motion_ms: 0.0, settle_ms: settle_s * 1000.0, trigger_ms: trig_total_ms, fetch_ms: fetch_total_ms,
            capture_ms: t_cap.elapsed().as_secs_f64() * 1000.0 },
    })
}

fn label_ok(d: &Detection, label: Option<&str>) -> bool { label.is_none_or(|l| d.label.eq_ignore_ascii_case(l)) }

/// The target at the start view: best IoU with `target_box`, else the best score.
fn pick_start(req: &Request, dets: &[Detection]) -> Option<Detection> {
    let c = dets.iter().filter(|d| d.score >= req.min_score && label_ok(d, req.label.as_deref()));
    match req.target_box {
        Some(t) => c.map(|d| (grasp::iou(&d.box_norm, &t), d)).filter(|(i, _)| *i >= TARGET_MIN_IOU).max_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, d)| d.clone()),
        None => c.max_by(|a, b| a.score.total_cmp(&b.score)).cloned(),
    }
}

/// The target in another view: same label, nearest to the start view's box centre moved by
/// `shift` (normalised image units).
fn pick_view(target: &Detection, min_score: f64, dets: &[Detection], shift: [f64; 2]) -> Option<Detection> {
    let c0 = grasp::box_centre(&target.box_norm);
    let c0 = [c0[0] + shift[0], c0[1] + shift[1]];
    dets.iter().filter(|d| d.score >= min_score && d.label.eq_ignore_ascii_case(&target.label))
        .map(|d| { let c = grasp::box_centre(&d.box_norm); ((c[0] - c0[0]).hypot(c[1] - c0[1]), d) })
        .filter(|(dist, _)| *dist <= TARGET_MAX_SHIFT_NORM)
        .min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, d)| d.clone())
}

/// Plan and run the joint-space move to the arm joints solving `goal`. The `bool` of the error
/// is true when the motion was installed (a failure after the arm moved).
async fn move_to_goal(ctx: &MacroCtx, plan: &Plan, phase: &str, goal: Pose) -> Result<(), (Stop, bool)> {
    let before = |s: Stop| (s, false);
    let (input, seeds) = {
        let inner = ctx.sup.inner().read().await;
        if !inner.macro_alive(ctx.id) { return Err(before(Stop::Cancelled)); }
        let input = inner.plan_input().map_err(arm_fail).map_err(before)?;
        let mut extra = inner.goal_seed_poses();
        extra.push(kinematics::q6(&plan.start[..plan.arm_dof.min(plan.start.len())]));
        let seeds = motion::goal_seeds(&input, &goal, &extra);
        (input, seeds)
    };
    let opts = PathOpts { yaw_free: plan.arm_dof < 6, yaw_outward: false };
    let q = tokio::task::spawn_blocking(move || motion::solve_goal_joints(&input, &goal, opts, &seeds)).await
        .map_err(|e| before(fail("INTERNAL", format!("IK task failed: {e}"))))?
        .map_err(|e| before(arm_fail(e.into())))?;
    move_joints(ctx, plan, phase, &q).await
}

/// Joint-space quintic sub-motion to the arm joints `q` (gripper kept).
async fn move_joints(ctx: &MacroCtx, plan: &Plan, phase: &str, q: &[f64; 6]) -> Result<(), (Stop, bool)> {
    let limit = {
        let mut inner = ctx.sup.inner().write().await;
        if !inner.macro_alive(ctx.id) { return Err((Stop::Cancelled, false)); }
        let target = inner.joint_goal_target(q);
        let limits = inner.joint_goal_limits(plan.request.speed_scale);
        inner.set_action_phase(ctx.id, phase, 0.0, None, None);
        match inner.install_macro_quintic(ctx.id, target, &limits) {
            Ok(l) => l,
            Err(e) => {
                if let Some(detail) = e.workspace_detail() { inner.set_action_workspace_violation(ctx.id, detail); }
                return Err((arm_fail(e), false));
            }
        }
    };
    ctx.wait_sub(phase, limit * 2.0 + 5.0).await.map(|_| ()).map_err(|s| (s, true))
}

fn is_refusal(s: &Stop) -> bool { matches!(s, Stop::Fail(code, _) if PLAN_REFUSALS.contains(&code.as_str())) }

fn r4(x: f64) -> f64 { (x * 1e4).round() / 1e4 }

/// The `measurement` value of the action (see docs/agent_action_api.md).
fn measurement_value(plan: &Plan, frame_source: FrameSource, caps: &[Capture], target: &Detection, result: &Result<mview_stereo::ObjectMeasure, &'static str>,
    table: &mview_stereo::TableCheck, stats: &mview_stereo::Stats, ncc_backend: &mview_stereo::BackendDiagnostics, fallbacks: &[Value], compute_ms: f64) -> Value {
    let r1 = |x: f64| (x * 10.0).round() / 10.0;
    let r2 = |x: f64| (x * 100.0).round() / 100.0;
    let views: Vec<Value> = caps.iter().map(|c| json!({
        "tcp": {"xyz_m": c.pose.p.map(r4), "quat_wxyz": kinematics::r_to_quat(&c.pose.r)},
        "frame_source": c.frame_source.name(),
        "instance_id": c.instance_id,
        "frame_id": c.frame_id, "offset_m": c.offset.map(r4),
        "predicted_shift_px": c.shift_px.map(r1), "frame_margin_px": r1(c.margin_px),
        "decode_ms": r2(c.decode_ms),
        "frame_fetch_ms": r2(c.frame_fetch_ms),
        "gray_convert_ms": c.gray_convert_ms.map(r2),
        "parse_ms": r2(c.parse_ms),
        "joints_rad": c.joints.iter().map(|q| (f64::from(*q) * 1e5).round() / 1e5).collect::<Vec<f64>>(),
        "snapshot_tries_mrad": c.snapshot_tries.iter().map(|t| r1(f64::from(*t) * 1000.0)).collect::<Vec<f64>>(),
        "retry_reasons": c.retry_reasons,
        "bracket_rad": c.bracket.iter().map(|b| b.iter().map(|q| (f64::from(*q) * 1e5).round() / 1e5).collect::<Vec<f64>>()).collect::<Vec<_>>(),
        "bracket_duration_ms": r2(c.bracket_duration_ms),
        "feedback_wait_b_ms": r2(c.feedback_wait_b_ms),
        "feedback_age_a_ms": c.feedback_age_a_ms.map(r2),
        "feedback_age_b_ms": c.feedback_age_b_ms.map(r2),
        "settle_s": r2(c.settle_s),
        "trigger_ms": c.trigger_ms.round(), "snapshot_wait_ms": c.snapshot_wait_ms.round(), "snapshot_polls": c.snapshot_polls,
        "timing": c.timing.to_json(),
    })).collect();
    let (objects, why) = match result {
        Ok(m) => (vec![json!({
            "label": target.label, "score": target.score,
            "centroid_m": m.centroid_m.map(r4), "top_z_m": r4(m.top_z_m), "height_m": r4(m.height_m),
            "width_m": r4(m.width_m), "length_m": r4(m.length_m), "yaw_deg": r1(m.yaw_deg),
            "n_points": m.n_points, "n_edge_points": m.n_edge_points, "n_top_points": m.n_top_points, "top_method": m.top_method,
            "reproj_rms_px": r2(m.reproj_rms_px), "views_used": caps.len(),
        })], Value::Null),
        Err(w) => (Vec::new(), json!(w)),
    };
    let mm = |v: Option<f64>| v.map_or(Value::Null, |x| json!(r2(x)));
    let mut st = serde_json::to_value(stats).unwrap_or(Value::Null);
    if let Some(o) = st.as_object_mut() {
        for v in o.values_mut() { if let Some(x) = v.as_f64().filter(|_| !v.is_u64()) { *v = json!(r2(x)); } }
        o.insert("decode_ms".into(), json!(r2(caps.iter().map(|c| c.decode_ms).sum::<f64>())));
        o.insert("frame_fetch_ms".into(), json!(r2(caps.iter().map(|c| c.frame_fetch_ms).sum::<f64>())));
        o.insert("parse_ms".into(), json!(r2(caps.iter().map(|c| c.parse_ms).sum::<f64>())));
        let conv_sum: f64 = caps.iter().filter_map(|c| c.gray_convert_ms).sum();
        if caps.iter().any(|c| c.gray_convert_ms.is_some()) {
            o.insert("gray_convert_ms".into(), json!(r2(conv_sum)));
        }
        o.insert("ncc_backend".into(), json!(ncc_backend.backend));
        o.insert("ncc_backend_reason".into(), json!(ncc_backend.reason));
    }
    let offsets: Vec<[f64; 2]> = caps.iter().map(|c| c.offset).collect();
    json!({
        "method": "motion_stereo",
        "frame_source": frame_source.name(),
        "motion": plan.request.motion.name(),
        "axes": plan.request.axes.name(),
        "objects": objects,
        "table_check": {"n_points": table.n_points, "median_abs_mm": mm(table.median_abs_mm), "p90_abs_mm": mm(table.p90_abs_mm), "median_mm": mm(table.median_mm)},
        "views": views,
        "why": why,
        "table_z_m": plan.setup.table_z_m,
        "baseline_m": r4(span(&offsets)),
        "fallbacks": fallbacks,
        "stats": st,
        "ncc_backend": ncc_backend,
        "compute_ms": compute_ms.round(),
    })
}

/// Predicted shift (px) of the look-at point in the view at `goal`, and the smallest distance
/// (px) of the start target box moved by it to the image border.
pub fn frame_prediction(cam: &CamModel, goal: &Pose, pivot: &Vec3, start_px: [f64; 2], box_px: [f64; 4]) -> ([f64; 2], f64) {
    let Some(q) = camera_at(cam, goal).project(pivot) else { return ([f64::NAN; 2], f64::NEG_INFINITY); };
    let s = [q[0] - start_px[0], q[1] - start_px[1]];
    let [w, h] = cam.size;
    let m = (box_px[0] + s[0]).min(box_px[1] + s[1]).min(w - (box_px[2] + s[0])).min(h - (box_px[3] + s[1]));
    (s, m)
}

/// The look-at point X: the ray through the target box centre at [`LOOK_AT_ABOVE_TABLE_M`]
/// above the table (0.35 m along the ray when it does not meet that plane).
fn look_at(plan: &Plan, pose: &Pose, target: &Detection) -> Vec3 {
    let centre = plan.cam.to_px(grasp::box_centre(&target.box_norm));
    plan.cam.plane_point(pose, centre, plan.setup.table_z_m + LOOK_AT_ABOVE_TABLE_M).unwrap_or_else(|| {
        let c = camera_at(&plan.cam, pose);
        let d = kinematics::unit(&c.ray(centre));
        kinematics::add(&c.c, &[0.35 * d[0], 0.35 * d[1], 0.35 * d[2]])
    })
}

/// Everything after the start capture, until the views are captured.
async fn run_views(ctx: &MacroCtx, plan: &Plan, src: &dyn DetectionSource, frame_source: FrameSource,
    negotiated_gray: Option<&grasp::GrayFormatDescriptor>, caps: &mut Vec<Capture>, target: &Detection,
    fallbacks: &mut Vec<Value>, pivot: &Vec3, markers: bool) -> Result<(), (Stop, bool)> {
    let pose0 = caps[0].pose;
    let cam0 = camera_at(&plan.cam, &pose0);
    let b = target.box_norm;
    let box_px = [b[0] * plan.cam.size[0], b[1] * plan.cam.size[1], b[2] * plan.cam.size[0], b[3] * plan.cam.size[1]];
    let start_px = cam0.project(pivot).unwrap_or_else(|| plan.cam.to_px(grasp::box_centre(&b)));
    let axes = plane_axes(&cam0, pivot, plan.request.motion);
    let offsets = view_offsets(plan.request.views, plan.request.baseline_m, plan.request.axes);
    let order = visit_order(&offsets);
    let mut used: Vec<[f64; 2]> = offsets.clone();
    let total = order.len() as f32;
    for (k, &i) in order.iter().enumerate() {
        let o = offsets[i];
        if is_start(&o) { continue; } // the start capture
        ctx.sup.inner().write().await.set_action_phase(ctx.id, &format!("view:{k}"), k as f32 / (total + 1.0), None, None);
        let t_view = Instant::now();
        let mut candidates = vec![o];
        let mut tried = 0;
        let mut last_refusal: Option<Stop> = None;
        let mut done = false;
        while tried < candidates.len() {
            let c = candidates[tried];
            tried += 1;
            recheck(ctx, plan).await.map_err(|s| (s, true))?;
            let goal = view_pose(&pose0, &cam0.c, pivot, &axes, c, plan.request.motion);
            let (shift, margin) = frame_prediction(&plan.cam, &goal, pivot, start_px, box_px);
            let moved = if margin < MIN_FRAME_MARGIN_PX {
                Err((fail("OUT_OF_FRAME", format!("the target would be {margin:.0} px from the image border at offset [{:+.3}, {:+.3}] m (minimum {MIN_FRAME_MARGIN_PX})", c[0], c[1])), false))
            } else {
                move_to_goal(ctx, plan, &format!("move:{k}"), goal).await
            };
            match moved {
                Ok(()) => {
                    let motion_ms = t_view.elapsed().as_secs_f64() * 1000.0;
                    if c != o {
                        fallbacks.push(json!({"offset_m": o.map(r4), "used_offset_m": c.map(r4), "code": last_refusal.as_ref().map(|s| match s { Stop::Fail(c, _) => c.clone(), Stop::Cancelled => String::new() })}));
                        used.retain(|u| dist2(u, &o) > 1e-9);
                        used.push(c);
                    }
                    ctx.sup.inner().write().await.set_action_phase(ctx.id, &format!("capture:{k}"), k as f32 / (total + 1.0), None, None);
                    let mut cap = capture(ctx, plan, src, frame_source, negotiated_gray, c, shift, margin, markers, MIN_SETTLE_S).await.map_err(|s| (s, true))?;
                    cap.timing.motion_ms = motion_ms;
                    // where the look-at point is in the captured view (measured joints)
                    let seen = camera_at(&plan.cam, &cap.pose).project(pivot).map_or(shift, |q| [q[0] - start_px[0], q[1] - start_px[1]]);
                    if pick_view(target, plan.request.min_score, &cap.detections, plan.cam.to_norm(seen)).is_none() {
                        return Err((fail("TARGET_LOST", format!("the {} is not detected at the view offset [{:+.3}, {:+.3}] m", target.label, c[0], c[1])), true));
                    }
                    caps.push(cap);
                    done = true;
                    break;
                }
                Err((s, false)) if is_refusal(&s) => {
                    if let Stop::Fail(code, msg) = &s { tracing::info!("measure_multiview {}: view offset [{:+.3}, {:+.3}] m refused ({code}: {msg})", ctx.id, c[0], c[1]); }
                    if tried == 1 { candidates.extend(fallback_offsets(o, &used)); }
                    last_refusal = Some(s);
                }
                Err(e) => return Err(e),
            }
        }
        if !done {
            let s = last_refusal.unwrap_or_else(|| fail("IK_UNREACHABLE", "no view pose"));
            // nothing moved for this view; earlier views may have moved the arm
            return Err((s, caps.len() > 1));
        }
    }
    let got: Vec<[f64; 2]> = caps.iter().map(|c| c.offset).collect();
    if span(&got) < MIN_EFFECTIVE_BASELINE_M {
        return Err((fail("IK_UNREACHABLE", format!("the reachable view poses span {:.3} m (minimum {MIN_EFFECTIVE_BASELINE_M})", span(&got))), true));
    }
    Ok(())
}

/// What [`compute`] returns.
pub struct Computed {
    pub measure: Result<mview_stereo::ObjectMeasure, &'static str>,
    pub table: mview_stereo::TableCheck,
    pub rec: mview_stereo::Reconstruction,
    pub region_px: [f64; 4],
    /// The per-view rotation refinement ([`mview_stereo::pose`]); its cameras were used.
    pub pose: Option<mview_stereo::PoseRefinement>,
    /// The fitted table plane the heights are measured from (None: z = table_z).
    pub plane: Option<mview_stereo::TablePlane>,
    /// The object threshold used (m).
    pub object_min_h_m: f64,
    /// Active NCC backend, selection reason, and kernel call counts for this run.
    pub ncc_backend: mview_stereo::BackendDiagnostics,
    pub ms: f64,
}

/// The feature region: the reference box grown by [`REGION_GROW`] + [`REGION_PAD_PX`].
pub fn feature_region(b: [f64; 4]) -> [f64; 4] {
    let (w, h) = (b[2] - b[0], b[3] - b[1]);
    [b[0] - REGION_GROW * w - REGION_PAD_PX, b[1] - REGION_GROW * h - REGION_PAD_PX, b[2] + REGION_GROW * w + REGION_PAD_PX, b[3] + REGION_GROW * h + REGION_PAD_PX]
}

/// Image region of the gripper jaws as fractions of the image [x0, y0, x1, y1]. A constant, not
/// computed: armd's configuration has no jaw geometry. Measured on the real B601-DM with the v8
/// camera holder (2026-11-21 debug bundles, 640x360): the dark jaws covered x 323..453, y 257..359
/// in every view; this region adds about 20 px on each side. The jaws move with the camera, so
/// their corners match at zero disparity; features in this region are used neither for the pose
/// refinement nor for the reconstruction.
pub const GRIPPER_REGION_NORM: [f64; 4] = [0.47, 0.66, 0.74, 1.0];

/// [`GRIPPER_REGION_NORM`] in pixels of an image of `size`.
pub fn gripper_region_px(size: [f64; 2]) -> [f64; 4] {
    let g = GRIPPER_REGION_NORM;
    [g[0] * size[0], g[1] * size[1], g[2] * size[0], g[3] * size[1]]
}

/// Marker corners of each view but `reference` shared with the reference view (by marker id
/// and corner index; ids seen twice in one view do not count).
pub fn shared_marker_corners(markers: &[Vec<mview_stereo::Marker>], reference: usize) -> Vec<usize> {
    let keys = |ms: &[mview_stereo::Marker]| -> std::collections::BTreeSet<i64> {
        ms.iter().map(|m| m.id).filter(|id| ms.iter().filter(|o| o.id == *id).count() == 1).collect()
    };
    let r = markers.get(reference).map(|m| keys(m)).unwrap_or_default();
    markers.iter().map(|m| 4 * keys(m).intersection(&r).count()).collect()
}

/// The reconstruction of the captures (blocking; spawn_blocking): the per-view rotation
/// refinement (`mode`; `markers` per view, camera model pixels), then the reconstruction with
/// the refined cameras.
pub fn compute(views: &[View], reference: usize, ref_box_px: [f64; 4], table_z: f64, mode: PoseRefine, markers: &[Vec<mview_stereo::Marker>]) -> Computed {
    let t0 = Instant::now();
    let region_px = feature_region(ref_box_px);
    let mut p = Params::new(table_z);
    p.exclude_px = views.get(reference).map(|v| gripper_region_px([v.img.w as f64, v.img.h as f64]));
    mview_stereo::reset_kernel_counts();
    let pose = (mode != PoseRefine::Off).then(|| {
        let mut pp = mview_stereo::PoseParams::default();
        let exclude = views.get(reference).map(|v| vec![gripper_region_px([v.img.w as f64, v.img.h as f64])]).unwrap_or_default();
        let corners = mview_stereo::marker_corrs(markers, reference, ref_box_px);
        if mode == PoseRefine::Aruco { pp.min_points = MIN_MARKER_CORNERS; }
        mview_stereo::refine_views(views, reference, ref_box_px, &exclude, table_z, &corners, mode == PoseRefine::Auto, &pp)
    });
    let rec = match &pose {
        Some(pr) => {
            let refined: Vec<View> = views.iter().zip(&pr.cams).map(|(v, c)| View { cam: *c, img: v.img.clone() }).collect();
            mview_stereo::reconstruct(&refined, reference, region_px, ref_box_px, &p)
        }
        None => mview_stereo::reconstruct(views, reference, region_px, ref_box_px, &p),
    };
    let ncc_backend = mview_stereo::backend_diagnostics();
    // heights from the table plane fitted to the points outside the box (else z = table_z)
    let cam0 = views.get(reference).map(|v| v.cam);
    let centre = cam0.and_then(|c| {
        let d = c.ray([0.5 * (ref_box_px[0] + ref_box_px[2]), 0.5 * (ref_box_px[1] + ref_box_px[3])]);
        (d[2] < -1e-3).then(|| { let t = (table_z - c.c[2]) / d[2]; [c.c[0] + t * d[0], c.c[1] + t * d[1]] })
    }).unwrap_or([0.0; 2]);
    let plane = mview_stereo::fit_table_plane(&rec.points, &p, centre);
    let mut pm = p;
    if let Some(t) = &plane {
        pm.plane = Some(t.plane);
        pm.object_min_h_m = t.threshold_mm / 1000.0;
    }
    let measure = mview_stereo::measure(&rec.points, &pm);
    let table = mview_stereo::table_check(&rec.points, &p);
    Computed { measure, table, rec, region_px, pose, plane, object_min_h_m: pm.object_min_h_m, ncc_backend, ms: t0.elapsed().as_secs_f64() * 1000.0 }
}

/// The `table_plane` value of the measurement: the fitted plane, or null (heights from table_z).
pub fn plane_value(plane: &Option<mview_stereo::TablePlane>) -> Value {
    let r2 = |x: f64| (x * 100.0).round() / 100.0;
    plane.as_ref().map_or(Value::Null, |t| json!({
        "n_points": t.n_points, "n_inliers": t.n_inliers, "offset_mm": r2(t.offset_mm), "tilt_deg": r2(t.tilt_deg),
        "mad_mm": r2(t.mad_mm), "threshold_mm": r2(t.threshold_mm),
    }))
}

/// The `pose_refine` value of the measurement (and of each view, `views`).
pub fn pose_value(mode: PoseRefine, pose: &Option<mview_stereo::PoseRefinement>) -> (Value, Vec<Value>) {
    let r2 = |x: f64| (x * 100.0).round() / 100.0;
    let Some(p) = pose else { return (json!({"mode": mode.name(), "applied": false}), Vec::new()); };
    let views: Vec<Value> = p.views.iter().map(|v| json!({
        "status": v.status, "correction_mrad": r2(v.correction_mrad), "rot_mrad": v.rot_mrad.map(r2),
        "n_points": v.n_points, "n_marker_corners": v.n_marker_corners,
        "rms_before_px": v.rms_before_px.map(r2), "rms_after_px": v.rms_after_px.map(r2),
    })).collect();
    (json!({
        "mode": mode.name(),
        "applied": p.views.iter().any(|v| v.status == "refined"),
        "n_points": p.n_corrs, "n_marker_points": p.n_marker_corrs, "ms": p.ms.round(),
        "views": views,
    }), views)
}

/// Triangulated points in the target box: count and height above the table (mm: min, median,
/// max), and how many lie above the object cut (5 mm).
pub fn box_heights(points: &[mview_stereo::Point], table_z: f64) -> Value {
    let p = Params::new(table_z);
    let mut h: Vec<f64> = points.iter().filter(|q| q.in_box).map(|q| (q.x[2] - table_z) * 1000.0).collect();
    h.sort_by(f64::total_cmp);
    let r1 = |x: f64| (x * 10.0).round() / 10.0;
    if h.is_empty() { return json!({"n": 0, "n_above": 0, "min_mm": null, "median_mm": null, "max_mm": null}); }
    json!({"n": h.len(), "n_above": h.iter().filter(|x| **x > p.object_min_h_m * 1000.0).count(),
        "min_mm": r1(h[0]), "median_mm": r1(mview_stereo::quantile(&h, 0.5)), "max_mm": r1(h[h.len() - 1])})
}

/// Feature outcomes counted by kind (`corner` / `edge`), place (`box` / `out`) and outcome.
pub fn outcome_counts(rec: &mview_stereo::Reconstruction, object_box: [f64; 4]) -> Value {
    let mut m: std::collections::BTreeMap<String, std::collections::BTreeMap<&str, usize>> = Default::default();
    for (f, o) in &rec.outcomes {
        let inside = (f.x as f64) >= object_box[0] && (f.x as f64) <= object_box[2] && (f.y as f64) >= object_box[1] && (f.y as f64) <= object_box[3];
        let key = format!("{}_{}", if f.normal.is_some() { "edge" } else { "corner" }, if inside { "box" } else { "out" });
        *m.entry(key).or_default().entry(o).or_default() += 1;
    }
    json!(m)
}

// ---- debug bundles ----

/// Where `debug: true` writes its bundles: `$ARMD_MVIEW_DEBUG_DIR`, else
/// `/userdata/rebot-teleop/mview-debug`. One directory per action id; the newest
/// [`MAX_DEBUG_BUNDLES`] are kept.
pub fn debug_dir() -> std::path::PathBuf {
    std::env::var_os("ARMD_MVIEW_DEBUG_DIR").map_or_else(|| std::path::PathBuf::from("/userdata/rebot-teleop/mview-debug"), std::path::PathBuf::from)
}

pub const MAX_DEBUG_BUNDLES: usize = 5;
pub const BUNDLE_VERSION: u32 = 2;

/// One view image stored in a debug bundle.
enum BundleImage {
    Jpeg(Vec<u8>),
    Gray(Gray),
}

impl BundleImage {
    fn ext(&self) -> &'static str {
        match self {
            Self::Jpeg(_) => "jpg",
            Self::Gray(_) => "pgm",
        }
    }
}

/// Validate exact integer pixel dimensions from a camera model `[w, h]` (no rounding of
/// fractional sizes).
fn exact_camera_dims(size: [f64; 2], label: &str) -> Result<(usize, usize), String> {
    let [w, h] = size;
    if !w.is_finite()
        || !h.is_finite()
        || w <= 0.0
        || h <= 0.0
        || w.fract() != 0.0
        || h.fract() != 0.0
        || w > usize::MAX as f64
        || h > usize::MAX as f64
    {
        return Err(format!("{label} must be positive integer pixels, got {w}x{h}"));
    }
    Ok((w as usize, h as usize))
}

/// Stream a grayscale image directly as canonical binary PGM (`P5\n<w> <h>\n255\n` + exact `w * h`
/// bytes) without allocating an extra copy of the pixel buffer.
pub fn write_pgm<W: std::io::Write>(mut out: W, img: &Gray) -> std::io::Result<()> {
    write!(out, "P5\n{} {}\n255\n", img.w, img.h)?;
    out.write_all(&img.px)
}

/// Encode a grayscale image as canonical binary PGM (`P5\n<w> <h>\n255\n` + exact `w * h` bytes).
pub fn encode_pgm(img: &Gray) -> Vec<u8> {
    let mut out = Vec::with_capacity(32 + img.px.len());
    let _ = write_pgm(&mut out, img);
    out
}

/// Parse a binary PGM (`P5`, maxval 255) into a [`Gray`] matching `expected_size` exactly (no
/// letterbox crop). Consumes **exactly one** ASCII whitespace byte after the maxval token so a
/// first pixel equal to whitespace (`0x20`, `0x0A`, `0x09`, `0x0D`) or `#` (`0x23`) survives.
pub fn parse_pgm(data: &[u8], expected_size: [f64; 2]) -> Result<Gray, String> {
    let mut pos = 0usize;
    let skip_ws_and_comments = |pos: &mut usize| -> Result<(), String> {
        loop {
            while *pos < data.len() && matches!(data[*pos], b' ' | b'\t' | b'\r' | b'\n') {
                *pos += 1;
            }
            if *pos < data.len() && data[*pos] == b'#' {
                while *pos < data.len() && data[*pos] != b'\n' {
                    *pos += 1;
                }
                if *pos < data.len() {
                    *pos += 1;
                } else {
                    return Err("PGM header truncated inside comment".into());
                }
            } else {
                return Ok(());
            }
        }
    };
    let next_token = |pos: &mut usize| -> Result<&[u8], String> {
        skip_ws_and_comments(pos)?;
        let start = *pos;
        while *pos < data.len() && !matches!(data[*pos], b' ' | b'\t' | b'\r' | b'\n' | b'#') {
            *pos += 1;
        }
        if *pos == start {
            return Err("PGM header missing token".into());
        }
        Ok(&data[start..*pos])
    };

    let magic = next_token(&mut pos)?;
    if magic != b"P5" {
        return Err("PGM magic must be P5".into());
    }
    let parse_dim = |tok: &[u8], what: &str| -> Result<usize, String> {
        let s = std::str::from_utf8(tok).map_err(|_| format!("PGM {what} is not ASCII"))?;
        s.parse::<usize>().map_err(|_| format!("PGM invalid {what}: {s}"))
    };
    let w = parse_dim(next_token(&mut pos)?, "width")?;
    let h = parse_dim(next_token(&mut pos)?, "height")?;
    let maxval = parse_dim(next_token(&mut pos)?, "maxval")?;
    if maxval != 255 {
        return Err(format!("PGM maxval must be 255, got {maxval}"));
    }
    // Consume EXACTLY one whitespace delimiter byte; do not skip subsequent bytes or '#'.
    if pos >= data.len() || !matches!(data[pos], b' ' | b'\t' | b'\r' | b'\n') {
        return Err("PGM missing single whitespace delimiter after maxval".into());
    }
    pos += 1;

    let (cw, ch) = exact_camera_dims(expected_size, "camera model (intrinsics size)")?;
    if w == 0 || h == 0 || w != cw || h != ch {
        return Err(format!("PGM is {w}x{h}; the camera model (intrinsics size) is {cw}x{ch}"));
    }
    let expected_bytes = w.checked_mul(h).ok_or_else(|| "PGM dimensions overflow".to_string())?;
    let payload = &data[pos..];
    if payload.len() != expected_bytes {
        return Err(format!("PGM payload length mismatch: expected {expected_bytes} bytes, got {}", payload.len()));
    }
    Gray::new(w, h, payload.to_vec()).ok_or_else(|| "PGM buffer creation failed".to_string())
}

fn mat_json(m: &kinematics::Mat3) -> Value { json!(m) }

/// Write the bundle of one run (blocking): `view_<k>.{jpg,pgm}` and `bundle.json`.
fn write_bundle(root: &std::path::Path, id: u64, json_doc: &Value, images: &[(usize, BundleImage)]) -> std::io::Result<std::path::PathBuf> {
    std::fs::create_dir_all(root)?;
    let dir = root.join(id.to_string());
    if dir.exists() { std::fs::remove_dir_all(&dir)?; }
    std::fs::create_dir_all(&dir)?;
    for (k, img) in images {
        let path = dir.join(format!("view_{k}.{}", img.ext()));
        match img {
            BundleImage::Jpeg(j) => std::fs::write(path, j)?,
            BundleImage::Gray(g) => write_pgm(std::fs::File::create(path)?, g)?,
        }
    }
    let tmp = dir.join(".bundle.json.tmp");
    std::fs::write(&tmp, serde_json::to_vec_pretty(json_doc).unwrap_or_default())?;
    std::fs::rename(tmp, dir.join("bundle.json"))?;
    // keep the newest MAX_DEBUG_BUNDLES directories
    let mut dirs: Vec<(std::time::SystemTime, std::path::PathBuf)> = std::fs::read_dir(root)?.flatten()
        .filter(|e| e.path().join("bundle.json").exists())
        .filter_map(|e| Some((e.metadata().ok()?.modified().ok()?, e.path()))).collect();
    dirs.sort_by_key(|d| std::cmp::Reverse(d.0));
    for (_, d) in dirs.into_iter().skip(MAX_DEBUG_BUNDLES) { let _ = std::fs::remove_dir_all(d); }
    Ok(dir)
}

/// The bundle document (see docs/agent_action_api.md, "Debug bundle").
fn bundle_doc(id: u64, plan: &Plan, frame_source: FrameSource, caps: &[Capture], reference: usize, ref_box_px: [f64; 4], c: &Computed, measurement: &Value) -> Value {
    let views: Vec<Value> = caps.iter().enumerate().map(|(k, v)| {
        let cam = camera_at(&plan.cam, &v.pose);
        let (ext, encoding) = match v.frame_source {
            FrameSource::Gray => ("pgm", "pgm"),
            FrameSource::Snapshot => ("jpg", "jpeg"),
        };
        json!({
            "file": format!("view_{k}.{ext}"), "encoding": encoding,
            "frame_source": v.frame_source.name(), "instance_id": v.instance_id,
            "offset_m": v.offset, "frame_id": v.frame_id,
            "joints_rad": v.joints, "bracket_rad": v.bracket,
            "snapshot_tries_mrad": v.snapshot_tries.iter().map(|t| t * 1000.0).collect::<Vec<f32>>(),
            "retry_reasons": v.retry_reasons,
            "bracket_duration_ms": v.bracket_duration_ms,
            "feedback_wait_b_ms": v.feedback_wait_b_ms,
            "feedback_age_a_ms": v.feedback_age_a_ms,
            "feedback_age_b_ms": v.feedback_age_b_ms,
            "settle_s": v.settle_s, "trigger_ms": v.trigger_ms,
            "frame_fetch_ms": v.frame_fetch_ms, "gray_convert_ms": v.gray_convert_ms, "parse_ms": v.parse_ms,
            "snapshot_wait_ms": v.snapshot_wait_ms, "snapshot_polls": v.snapshot_polls,
            "timing": v.timing.to_json(),
            "tcp": {"xyz_m": v.pose.p, "r": mat_json(&v.pose.r)},
            "camera": {"r_base_cam": mat_json(&cam.r), "c_m": cam.c},
            "detections": v.detections.iter().map(|d| json!({"label": d.label, "score": d.score, "box_norm": d.box_norm})).collect::<Vec<_>>(),
            "markers": v.markers.iter().map(|m| json!({"id": m.id, "corners": m.corners})).collect::<Vec<_>>(),
        })
    }).collect();
    let tz = plan.setup.table_z_m;
    let points: Vec<Value> = c.rec.points.iter().map(|q| json!({
        "x_m": q.x, "h_mm": (q.x[2] - tz) * 1000.0, "ref_px": q.ref_px, "in_box": q.in_box, "edge": q.edge,
        "consistent": q.consistent, "reproj_rms_px": q.reproj_rms_px, "views": q.views,
    })).collect();
    let outcomes: Vec<Value> = c.rec.outcomes.iter().map(|(f, o)| json!({"x": f.x, "y": f.y, "edge": f.normal.is_some(), "outcome": o})).collect();
    json!({
        "version": BUNDLE_VERSION, "action_id": id, "armd_version": env!("CARGO_PKG_VERSION"),
        "frame_source": frame_source.name(),
        "created_at": chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Millis, true),
        "request": {"motion": plan.request.motion.name(), "axes": plan.request.axes.name(), "views": plan.request.views,
            "baseline_m": plan.request.baseline_m, "speed_scale": plan.request.speed_scale, "label": plan.request.label, "min_score": plan.request.min_score,
            "pose_refine": plan.request.pose_refine.name()},
        "intrinsics": {"fx": plan.cam.fx, "fy": plan.cam.fy, "cx": plan.cam.cx, "cy": plan.cam.cy, "size": plan.cam.size},
        "cam_mount": {"r_end_cam": mat_json(&plan.cam.rec), "offset_m": plan.cam.offset},
        "table_z_m": tz, "reference": reference, "ref_box_px": ref_box_px, "region_px": c.region_px,
        "views": views, "stats": c.rec.stats, "ncc_backend": c.ncc_backend, "points": points, "outcomes": outcomes, "measurement": measurement,
    })
}

/// A bundle with only what a replay needs (for synthetic or converted captures): the JPEGs, the
/// cameras (shared intrinsics), the reference view, its box and the table height.
pub fn write_replay_bundle(dir: &std::path::Path, jpegs: &[Vec<u8>], cams: &[Camera], size: [f64; 2], reference: usize, ref_box_px: [f64; 4], table_z_m: f64) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let c0 = cams.first().copied().ok_or_else(|| std::io::Error::other("no camera"))?;
    let views: Vec<Value> = jpegs.iter().zip(cams).enumerate().map(|(k, (j, c))| {
        let _ = std::fs::write(dir.join(format!("view_{k}.jpg")), j);
        json!({"file": format!("view_{k}.jpg"), "encoding": "jpeg", "frame_source": "snapshot", "camera": {"r_base_cam": mat_json(&c.r), "c_m": c.c}})
    }).collect();
    let doc = json!({"version": BUNDLE_VERSION, "frame_source": "snapshot", "intrinsics": {"fx": c0.fx, "fy": c0.fy, "cx": c0.cx, "cy": c0.cy, "size": size},
        "table_z_m": table_z_m, "reference": reference, "ref_box_px": ref_box_px, "views": views});
    std::fs::write(dir.join("bundle.json"), serde_json::to_vec_pretty(&doc).unwrap_or_default())
}

/// Write a v2 replay bundle with raw binary PGM (`P5`) grayscale views.
pub fn write_replay_bundle_gray(dir: &std::path::Path, imgs: &[Gray], cams: &[Camera], size: [f64; 2], reference: usize, ref_box_px: [f64; 4], table_z_m: f64) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let c0 = cams.first().copied().ok_or_else(|| std::io::Error::other("no camera"))?;
    let mut views = Vec::with_capacity(imgs.len());
    for (k, (img, c)) in imgs.iter().zip(cams).enumerate() {
        write_pgm(std::fs::File::create(dir.join(format!("view_{k}.pgm")))?, img)?;
        views.push(json!({
            "file": format!("view_{k}.pgm"), "encoding": "pgm", "frame_source": "gray",
            "camera": {"r_base_cam": mat_json(&c.r), "c_m": c.c}
        }));
    }
    let doc = json!({"version": BUNDLE_VERSION, "frame_source": "gray", "intrinsics": {"fx": c0.fx, "fy": c0.fy, "cx": c0.cx, "cy": c0.cy, "size": size},
        "table_z_m": table_z_m, "reference": reference, "ref_box_px": ref_box_px, "views": views});
    std::fs::write(dir.join("bundle.json"), serde_json::to_vec_pretty(&doc).unwrap_or_default())
}

/// A bundle read back for a replay ([`replay_bundle`]).
pub struct Bundle {
    pub views: Vec<View>,
    pub reference: usize,
    pub ref_box_px: [f64; 4],
    pub table_z_m: f64,
    /// ArUco markers per view (camera model pixels; empty in older bundles).
    pub markers: Vec<Vec<mview_stereo::Marker>>,
    /// The run's `pose_refine` (`auto` when the bundle does not say).
    pub pose_refine: PoseRefine,
    pub doc: Value,
}

/// Read a bundle directory (v1 legacy JPEG bundles and v2 bundles with per-view `"encoding"`
/// `"jpeg"` or `"pgm"`): decode each view to the camera frame and rebuild its camera from the
/// stored pose. Rejects unsupported versions, unknown/missing v2 encodings, malformed lengths,
/// and geometry mismatches.
pub fn read_bundle(dir: &std::path::Path) -> Result<Bundle, String> {
    let doc: Value = serde_json::from_slice(&std::fs::read(dir.join("bundle.json")).map_err(|e| format!("bundle.json: {e}"))?)
        .map_err(|e| format!("bundle.json: {e}"))?;
    let version = doc.get("version").and_then(Value::as_u64).ok_or("bundle: version missing")?;
    if version != 1 && version != 2 {
        return Err(format!("bundle: unsupported version {version} (expected 1 or 2)"));
    }
    let f = |v: &Value, what: &str| v.as_f64().ok_or_else(|| format!("bundle: {what} missing"));
    let mat = |v: &Value, what: &str| -> Result<kinematics::Mat3, String> { serde_json::from_value(v.clone()).map_err(|e| format!("bundle: {what}: {e}")) };
    let vec3 = |v: &Value, what: &str| -> Result<Vec3, String> { serde_json::from_value(v.clone()).map_err(|e| format!("bundle: {what}: {e}")) };
    let i = &doc["intrinsics"];
    let (fx, fy, cx, cy) = (f(&i["fx"], "fx")?, f(&i["fy"], "fy")?, f(&i["cx"], "cx")?, f(&i["cy"], "cy")?);
    let size = [f(&i["size"][0], "size")?, f(&i["size"][1], "size")?];
    let mut views = Vec::new();
    let mut markers = Vec::new();
    for (k, v) in doc["views"].as_array().ok_or("bundle: views missing")?.iter().enumerate() {
        markers.push(grasp::parse_markers(&v["markers"]));
        let file = v["file"].as_str().ok_or("bundle: view file missing")?;
        let encoding = match (version, v.get("encoding").and_then(Value::as_str)) {
            (1, None) | (1, Some("jpeg")) => "jpeg",
            (1, Some(other)) => return Err(format!("bundle: views[{k}] unsupported v1 encoding \"{other}\"")),
            (2, Some(enc @ ("jpeg" | "pgm"))) => enc,
            (2, Some(other)) => return Err(format!("bundle: views[{k}] unknown encoding \"{other}\"")),
            (2, None) => return Err(format!("bundle: views[{k}] missing encoding")),
            _ => unreachable!(),
        };
        let bytes = std::fs::read(dir.join(file)).map_err(|e| format!("{file}: {e}"))?;
        let img = match encoding {
            "jpeg" => decode_frame(&bytes, size)?,
            "pgm" => parse_pgm(&bytes, size)?,
            _ => unreachable!(),
        };
        let cam = Camera { r: mat(&v["camera"]["r_base_cam"], &format!("views[{k}].camera"))?, c: vec3(&v["camera"]["c_m"], &format!("views[{k}].camera"))?, fx, fy, cx, cy };
        views.push(View { cam, img });
    }
    let b = &doc["ref_box_px"];
    Ok(Bundle {
        reference: doc["reference"].as_u64().ok_or("bundle: reference missing")? as usize,
        ref_box_px: [f(&b[0], "ref_box_px")?, f(&b[1], "ref_box_px")?, f(&b[2], "ref_box_px")?, f(&b[3], "ref_box_px")?],
        table_z_m: f(&doc["table_z_m"], "table_z_m")?,
        pose_refine: doc["request"]["pose_refine"].as_str().and_then(PoseRefine::parse).unwrap_or(PoseRefine::Auto),
        views, markers, doc,
    })
}

/// Re-run the reconstruction of a bundle (host replay; `cargo run --release --features mview-replay --bin mview-replay -- <dir>`).
pub fn replay_bundle(dir: &std::path::Path) -> Result<(Bundle, Computed), String> { replay_bundle_with(dir, None) }

/// [`replay_bundle`] with `pose_refine` `mode` instead of the run's.
pub fn replay_bundle_with(dir: &std::path::Path, mode: Option<PoseRefine>) -> Result<(Bundle, Computed), String> {
    let b = read_bundle(dir)?;
    let c = compute(&b.views, b.reference, b.ref_box_px, b.table_z_m, mode.unwrap_or(b.pose_refine), &b.markers);
    Ok((b, c))
}

async fn run(sup: Arc<ArmSupervisor>, id: u64, plan: Plan, src: Arc<dyn DetectionSource>) {
    let ctx = MacroCtx {
        sup: sup.clone(), id,
        deadline: Instant::now() + Duration::from_secs_f64(plan.deadline_s),
        phase_timeout_s: plan.deadline_s,
        timeout_code: "MEASURE_TIMEOUT",
        timeout_message: "the measurement did not finish within its time bound".into(),
    };
    let mut hold = None;
    let outcome = measure_run(&ctx, &plan, &src, &mut hold).await;
    // the camera gate goes back to the provider's state before the run, whatever the outcome
    if let Some(h) = hold { h.release().await; }
    let mut inner = sup.inner().write().await;
    match outcome {
        Ok(None) => inner.finish_macro(id, "done", None, None, None, None),
        Ok(Some((code, message))) => inner.finish_macro(id, "failed", Some(&code), Some(message), None, None),
        Err(Stop::Fail(code, message)) => inner.finish_macro(id, "failed", Some(&code), Some(message), None, None),
        Err(Stop::Cancelled) => {}
    }
    let a = inner.last_action.as_ref().filter(|a| a.id == id).map(|a| (a.state.clone(), a.code.clone()));
    drop(inner);
    tracing::info!("measure_multiview {id} ended: {a:?}");
}

/// The run's image source: gray when the provider advertises it on a loopback upstream and its
/// `gray_frames` descriptor is absent (fixed v1 contract) or a supported v1 descriptor. A
/// malformed or unsupported descriptor selects the JPEG snapshot path.
pub fn select_frame_source(mode: &grasp::ModeInfo) -> FrameSource {
    let descriptor_ok = match &mode.gray_format {
        Ok(None) => true,
        Ok(Some(fmt)) => fmt.is_supported_v1(),
        Err(_) => false,
    };
    if mode.gray_available && mode.upstream_loopback && descriptor_ok { FrameSource::Gray } else { FrameSource::Snapshot }
}

/// Return to the start joints when asked (a failure there ends the action with its own code).
async fn back_to_start(ctx: &MacroCtx, plan: &Plan) -> Result<(), Stop> {
    if !plan.request.return_to_start { return Ok(()); }
    recheck(ctx, plan).await?;
    let q = kinematics::q6(&plan.start[..plan.arm_dof.min(plan.start.len())]);
    move_joints(ctx, plan, "return", &q).await.map_err(|(s, _)| s)
}

/// The run; Ok(Some((code, message))): a measurement failure after the views (the action fails
/// with it, after the return to the start pose).
async fn measure_run(ctx: &MacroCtx, plan: &Plan, src_arc: &Arc<dyn DetectionSource>, hold: &mut Option<CameraHold>) -> Result<Option<(String, String)>, Stop> {
    let src: &dyn DetectionSource = src_arc.as_ref();
    let t_run = Instant::now();
    let mut timing = RunTiming::default();
    recheck(ctx, plan).await?;
    // Negotiate capabilities and image source ONCE for the whole run (never switch mid-run).
    let t_neg = Instant::now();
    let mode = src.mode_info().await.map_err(|e| source_fail(format!("cannot reach the vision provider: {e}")))?;
    if !mode.per_request_pipelines {
        return Err(fail("UNSUPPORTED", "the vision provider does not take per-request pipelines (perception API 2)"));
    }
    // marker corners for the pose refinement: requested when the provider runs ArUco
    let markers = match plan.request.pose_refine {
        PoseRefine::Off => false,
        PoseRefine::Auto => mode.aruco_available,
        PoseRefine::Aruco => {
            if mode.aruco_available {
                true
            } else {
                return Err(fail("MARKERS_UNAVAILABLE", "pose_refine \"aruco\" needs ArUco markers: the vision provider does not run the aruco pipeline"));
            }
        }
    };
    let frame_source = select_frame_source(&mode);
    let negotiated_gray_format = match (&mode.gray_format, frame_source) {
        (Ok(fmt), FrameSource::Gray) => fmt.clone(),
        _ => None,
    };
    let held = hold.insert(CameraHold::start(src_arc.clone(), frame_source == FrameSource::Gray).await);
    let camera_hold = held.status;
    timing.negotiate_ms = t_neg.elapsed().as_secs_f64() * 1000.0;
    ctx.sup.inner().write().await.set_action_phase(ctx.id, "capture:start", 0.0, None, None);
    let t_detect = Instant::now();
    // the start view: no minimum wait (the arm was not moved by this action), still windows only
    let first = capture(ctx, plan, src, frame_source, negotiated_gray_format.as_ref(), [0.0, 0.0], [0.0, 0.0], f64::NAN, markers, 0.0).await?;
    let Some(target) = pick_start(&plan.request, &first.detections) else {
        let what = plan.request.label.as_deref().map_or("an object".to_string(), |l| format!("a {l}"));
        return Err(fail("TARGET_LOST", format!("{what} (score >= {}) is not detected at the start pose{}", plan.request.min_score,
            if plan.request.target_box.is_some() { " overlapping target.box_norm" } else { "" })));
    };
    timing.detect_ms = t_detect.elapsed().as_secs_f64() * 1000.0;
    let mut first = first;
    let pivot = look_at(plan, &first.pose, &target);
    {
        let b = target.box_norm;
        let [w, h] = plan.cam.size;
        first.margin_px = (b[0] * w).min(b[1] * h).min(w - b[2] * w).min(h - b[3] * h);
    }
    let mut caps = vec![first];
    let mut fallbacks = Vec::new();
    let t_views = Instant::now();
    let views_res = run_views(ctx, plan, src, frame_source, negotiated_gray_format.as_ref(), &mut caps, &target, &mut fallbacks, &pivot, markers).await;
    timing.views_ms = t_views.elapsed().as_secs_f64() * 1000.0;
    if let Err((stop, moved)) = views_res {
        // a measurement failure goes back to the start first; a stop, fault, collision, torque
        // limit, lease loss or anything else ends the action where the arm is
        if let (Stop::Fail(code, message), true) = (&stop, moved) {
            if RETURN_AFTER.contains(&code.as_str()) && ctx.alive().await {
                if let Err(e) = back_to_start(ctx, plan).await {
                    tracing::warn!("measure_multiview {}: return to start failed: {e:?}", ctx.id);
                }
                return Ok(Some((code.clone(), message.clone())));
            }
        }
        return Err(stop);
    }
    // reference: the view closest to the start pose; its own detection box
    let reference = (0..caps.len()).min_by(|&a, &b| dist2(&caps[a].offset, &[0.0; 2]).total_cmp(&dist2(&caps[b].offset, &[0.0; 2]))).unwrap_or(0);
    let shift = plan.cam.to_norm(caps[reference].shift_px);
    let ref_det = pick_view(&target, plan.request.min_score, &caps[reference].detections, shift).unwrap_or_else(|| target.clone());
    let b = ref_det.box_norm;
    let ref_box_px = [b[0] * plan.cam.size[0], b[1] * plan.cam.size[1], b[2] * plan.cam.size[0], b[3] * plan.cam.size[1]];
    let marks: Vec<Vec<mview_stereo::Marker>> = caps.iter().map(|c| c.markers.clone()).collect();
    let mode = plan.request.pose_refine;
    if mode == PoseRefine::Aruco {
        let shared = shared_marker_corners(&marks, reference);
        let short: Vec<String> = shared.iter().enumerate().filter(|(k, n)| *k != reference && **n < MIN_MARKER_CORNERS).map(|(k, n)| format!("view {k}: {n}")).collect();
        if !short.is_empty() || marks.get(reference).is_none_or(|m| m.is_empty()) {
            back_to_start(ctx, plan).await?;
            return Ok(Some(("MARKERS_UNAVAILABLE".to_string(), format!("pose_refine \"aruco\": every view needs {MIN_MARKER_CORNERS} ArUco marker corners shared with the reference view (markers per view {:?}; shared corners {})",
                marks.iter().map(Vec::len).collect::<Vec<_>>(), if short.is_empty() { "none in the reference view".to_string() } else { short.join(", ") }))));
        }
    }
    let views: Vec<View> = caps.iter_mut().map(|c| View { cam: camera_at(&plan.cam, &c.pose), img: std::mem::replace(&mut c.img, Gray { w: 0, h: 0, px: Vec::new() }) }).collect();
    let table_z = plan.setup.table_z_m;
    let keep_views_for_debug = plan.request.debug && frame_source == FrameSource::Gray;
    ctx.sup.inner().write().await.set_action_phase(ctx.id, "compute", 0.9, None, None);
    let task = tokio::task::spawn_blocking(move || {
        let t0 = Instant::now();
        let c = compute(&views, reference, ref_box_px, table_z, mode, &marks);
        let span = (t0, Instant::now());
        let returned_views = if keep_views_for_debug { views } else { Vec::new() };
        (c, returned_views, span)
    });
    let t_ret = Instant::now();
    let back = back_to_start(ctx, plan).await;
    let ret_span = (t_ret, Instant::now());
    let (c, mut computed_views, compute_span) = task.await.map_err(|e| fail("INTERNAL", format!("compute task failed: {e}")))?;
    timing.return_ms = if plan.request.return_to_start { (ret_span.1 - ret_span.0).as_secs_f64() * 1000.0 } else { 0.0 };
    timing.compute_ms = (compute_span.1 - compute_span.0).as_secs_f64() * 1000.0;
    timing.overlap_ms = if plan.request.return_to_start { overlap_ms(compute_span, ret_span) } else { 0.0 };
    let stats = c.rec.stats;
    tracing::info!(
        "measure_multiview {}: {} views, ncc_backend={} ({}), {stats:?}, {:.0} ms, {:?}",
        ctx.id,
        caps.len(),
        c.ncc_backend.backend,
        c.ncc_backend.reason,
        c.ms,
        c.measure.as_ref().map(|m| m.height_m)
    );
    let in_box = box_heights(&c.rec.points, table_z);
    let mut value = measurement_value(plan, frame_source, &caps, &ref_det, &c.measure, &c.table, &stats, &c.ncc_backend, &fallbacks, c.ms);
    value["in_box"] = in_box.clone();
    value["outcomes"] = outcome_counts(&c.rec, ref_box_px);
    value["table_plane"] = plane_value(&c.plane);
    timing.total_ms = t_run.elapsed().as_secs_f64() * 1000.0;
    let view_timing: Vec<ViewTiming> = caps.iter().map(|c| c.timing).collect();
    value["timing"] = timing_value(&timing, &view_timing);
    value["camera_hold"] = json!(camera_hold);
    let (mut pose, per_view) = pose_value(mode, &c.pose);
    pose["markers_requested"] = json!(markers);
    value["pose_refine"] = pose;
    if let Some(vs) = value["views"].as_array_mut() { for (v, p) in vs.iter_mut().zip(per_view) { v["pose_refine"] = p; } }
    if plan.request.debug {
        let doc = bundle_doc(ctx.id, plan, frame_source, &caps, reference, ref_box_px, &c, &value);
        let images: Vec<(usize, BundleImage)> = match frame_source {
            FrameSource::Snapshot => caps.iter_mut().enumerate().filter_map(|(k, v)| v.jpeg.take().map(|j| (k, BundleImage::Jpeg(j)))).collect(),
            FrameSource::Gray => computed_views.drain(..).enumerate().map(|(k, v)| (k, BundleImage::Gray(v.img))).collect(),
        };
        let (root, id) = (debug_dir(), ctx.id);
        let written = tokio::task::spawn_blocking(move || write_bundle(&root, id, &doc, &images)).await;
        value["debug"] = match written {
            Ok(Ok(dir)) => json!({"dir": dir.to_string_lossy(), "written": true}),
            Ok(Err(e)) => { tracing::warn!("measure_multiview {}: debug bundle not written: {e}", ctx.id); json!({"dir": debug_dir().to_string_lossy(), "written": false, "error": e.to_string()}) }
            Err(e) => json!({"written": false, "error": e.to_string()}),
        };
    }
    ctx.sup.inner().write().await.update_action(ctx.id, |a| a.measurement = Some(value));
    back?;
    let p = Params::new(table_z);
    let thr_mm = c.object_min_h_m * 1000.0;
    Ok(c.measure.err().map(|_| ("TOO_FEW_POINTS".to_string(),
        format!("fewer than {} object points more than {thr_mm:.0} mm above the table in the target box: {} triangulated points in the box (height above table min {} / median {} / max {} mm, {} above {:.0} mm); {} points in all; features {}, no peak {}, ambiguous {}, refine {}, left-right {}, reprojection {}",
            p.min_object_points, in_box["n"], in_box["min_mm"], in_box["median_mm"], in_box["max_mm"], in_box["n_above"], p.object_min_h_m * 1000.0,
            stats.points, stats.features, stats.no_peak, stats.ambiguous, stats.refine_rejected, stats.lr_rejected, stats.reproj_rejected))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_parsing_and_bounds() {
        let r = parse_request(&Value::Null).unwrap();
        assert_eq!((r.views, r.baseline_m, r.speed_scale, r.return_to_start, r.min_score), (3, 0.08, 0.3, true, 0.3));
        assert_eq!((r.motion, r.axes), (Motion::Orbit, Axes::X));
        let r = parse_request(&json!({"motion": "translate"})).unwrap();
        assert_eq!((r.motion, r.axes, r.views), (Motion::Translate, Axes::Xy, 5));
        let r = parse_request(&json!({"motion": "orbit", "axes": "xy", "views": 3})).unwrap();
        assert_eq!((r.motion, r.axes, r.views), (Motion::Orbit, Axes::Xy, 3));
        let r = parse_request(&json!({"motion": "translate", "axes": "x"})).unwrap();
        assert_eq!((r.axes, r.views), (Axes::X, 3));
        let r = parse_request(&json!({"label": "cup", "target": {"box_norm": [0.1, 0.2, 0.3, 0.4], "frame_id": 7}, "views": 5, "baseline_m": 0.12, "speed_scale": 0.5, "return_to_start": false})).unwrap();
        assert_eq!((r.target_box, r.target_frame_id, r.views, r.return_to_start), (Some([0.1, 0.2, 0.3, 0.4]), Some(7), 5, false));
        for bad in [json!({"views": 1}), json!({"views": 6}), json!({"baseline_m": 0.02}), json!({"baseline_m": 0.13}), json!({"speed_scale": 0.6}),
                    json!({"speed_scale": 0.05}), json!({"target": {"box_norm": [0.3, 0.2, 0.1, 0.4]}}), json!({"target": {"box_norm": [0, 0, 1]}}),
                    json!({"target": [0, 0, 1, 1]}), json!({"foo": 1}), json!({"return_to_start": 1}), json!({"label": ""}), json!([1]),
                    json!({"motion": "spin"}), json!({"axes": "z"}), json!({"axes": "xy", "views": 2}), json!({"motion": 1})] {
            let e = crate::api::rest::ApiError::from(parse_request(&bad).unwrap_err());
            assert_eq!(e.code.as_deref(), Some("INVALID_COMMAND"), "{bad}");
        }
    }

    #[test]
    fn pose_refine_request_and_shared_marker_corners() {
        assert_eq!(parse_request(&Value::Null).unwrap().pose_refine, PoseRefine::Auto);
        for m in [PoseRefine::Auto, PoseRefine::Aruco, PoseRefine::Off] {
            assert_eq!(parse_request(&json!({"pose_refine": m.name()})).unwrap().pose_refine, m);
        }
        for bad in [json!({"pose_refine": "on"}), json!({"pose_refine": true})] {
            let e = crate::api::rest::ApiError::from(parse_request(&bad).unwrap_err());
            assert_eq!(e.code.as_deref(), Some("INVALID_COMMAND"), "{bad}");
        }
        let m = |id: i64| mview_stereo::Marker { id, corners: [[0.0; 2]; 4] };
        // view 1 shares ids 1, 2; view 2 sees id 2 twice (not counted) and 3
        let views = vec![vec![m(1), m(2), m(3)], vec![m(1), m(2), m(9)], vec![m(2), m(2), m(3)]];
        assert_eq!(shared_marker_corners(&views, 0), vec![12, 8, 4]);
    }

    #[test]
    fn offsets_order_and_fallbacks() {
        let x = |v: &[[f64; 2]]| v.iter().map(|o| { assert_eq!(o[1], 0.0); o[0] }).collect::<Vec<f64>>();
        assert_eq!(x(&view_offsets(3, 0.08, Axes::X)), vec![-0.04, 0.0, 0.04]);
        assert_eq!(x(&view_offsets(2, 0.08, Axes::X)), vec![-0.04, 0.04]);
        let o5 = view_offsets(5, 0.08, Axes::X);
        assert!((o5[1][0] + 0.02).abs() < 1e-12 && o5[2][0] == 0.0);
        assert_eq!(visit_order(&o5), vec![2, 1, 0, 3, 4]);
        // xy: the start, then +x, +y, -x, -y on the circle of radius B/2, visited by angle
        let c5 = view_offsets(5, 0.08, Axes::Xy);
        assert_eq!(c5, vec![[0.0, 0.0], [0.04, 0.0], [0.0, 0.04], [-0.04, 0.0], [0.0, -0.04]]);
        assert_eq!(visit_order(&c5), vec![0, 1, 2, 3, 4]);
        assert!((span(&c5) - 0.08).abs() < 1e-12 && (span(&view_offsets(3, 0.08, Axes::X)) - 0.08).abs() < 1e-12);
        let c4 = view_offsets(4, 0.08, Axes::Xy);
        assert_eq!(c4.len(), 4);
        assert!(c4[1..].iter().all(|o| (o[0].hypot(o[1]) - 0.04).abs() < 1e-12));
        let mm = |f: &[[f64; 2]]| f.iter().map(|o| ((o[0] * 1000.0).round() as i64, (o[1] * 1000.0).round() as i64)).collect::<Vec<_>>();
        // the mirror is in use: the smaller offsets on both sides
        let used = view_offsets(3, 0.08, Axes::X);
        assert_eq!(mm(&fallback_offsets([0.04, 0.0], &used)), vec![(30, 0), (-30, 0), (20, 0), (-20, 0)]);
        // views 2 with the mirror free (refused first)
        assert_eq!(mm(&fallback_offsets([-0.04, 0.0], &[[0.04, 0.0]])), vec![(-30, 0), (30, 0), (-20, 0), (20, 0)]);
        assert_eq!(mm(&fallback_offsets([-0.04, 0.0], &[[-0.04, 0.0]]))[0], (40, 0));
        // xy: +y refused, -y in use
        assert_eq!(mm(&fallback_offsets([0.0, 0.04], &c5)), vec![(0, 30), (0, -30), (0, 20), (0, -20)]);
    }

    #[test]
    fn orbit_keeps_the_pivot_on_the_same_pixel_and_the_distance() {
        let arm = crate::chain::ArmModel::default();
        let setup = GraspSetup::default();
        let cam = CamModel::assumed(&arm, &setup, &crate::model::CameraMount::default());
        let pose = arm.fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 0.0]);
        let c0 = camera_at(&cam, &pose);
        let centre = [cam.cx + 20.0, cam.cy + 10.0];
        let pivot = cam.plane_point(&pose, centre, 0.02).unwrap();
        let ray = kinematics::unit(&kinematics::sub(&pivot, &c0.c));
        let xc = kinematics::column(&c0.r, 0);
        let k = kinematics::dot(&ray, &xc);
        let lat = kinematics::unit(&kinematics::sub(&xc, &[ray[0] * k, ray[1] * k, ray[2] * k]));
        for o in [-0.06, -0.04, 0.03] {
            let p = orbit_pose(&pose, &c0.c, &pivot, &lat, o);
            let c = camera_at(&cam, &p);
            let px = c.project(&pivot).unwrap();
            assert!((px[0] - centre[0]).abs() < 1e-6 && (px[1] - centre[1]).abs() < 1e-6, "{px:?}");
            let chord = kinematics::norm(&kinematics::sub(&c.c, &c0.c));
            assert!((chord - o.abs()).abs() < 1e-9, "{chord}");
            let along = kinematics::dot(&kinematics::sub(&c.c, &c0.c), &lat);
            assert!(along * o > 0.0 && (along.abs() - o.abs()).abs() < 0.002, "{along}");
            assert!((kinematics::norm(&kinematics::sub(&c.c, &pivot)) - kinematics::norm(&kinematics::sub(&c0.c, &pivot))).abs() < 1e-9);
        }
    }

    #[test]
    fn view_poses_orbit_on_two_axes_and_translate() {
        let arm = crate::chain::ArmModel::default();
        let cam = CamModel::assumed(&arm, &GraspSetup::default(), &crate::model::CameraMount::default());
        let pose = arm.fk(&[0.0, -1.167, -1.565, 1.435, 0.0, 0.0]);
        let c0 = camera_at(&cam, &pose);
        let centre = [cam.cx, cam.cy];
        let pivot = cam.plane_point(&pose, centre, 0.02).unwrap();
        let z = kinematics::norm(&kinematics::sub(&pivot, &c0.c));
        for o in view_offsets(5, 0.08, Axes::Xy) {
            // orbit: the pivot stays on its pixel, the chord is |offset|
            let ax = plane_axes(&c0, &pivot, Motion::Orbit);
            let c = camera_at(&cam, &view_pose(&pose, &c0.c, &pivot, &ax, o, Motion::Orbit));
            let px = c.project(&pivot).unwrap();
            assert!((px[0] - centre[0]).abs() < 1e-6 && (px[1] - centre[1]).abs() < 1e-6, "{o:?} {px:?}");
            assert!((kinematics::norm(&kinematics::sub(&c.c, &c0.c)) - o[0].hypot(o[1])).abs() < 1e-9);
            // translate: same orientation, the camera moves by the offset along its own x, y; the
            // pivot shifts by about fx * offset / z the other way
            let ax = plane_axes(&c0, &pivot, Motion::Translate);
            let p = view_pose(&pose, &c0.c, &pivot, &ax, o, Motion::Translate);
            assert_eq!(p.r, pose.r);
            let c = camera_at(&cam, &p);
            let d = kinematics::mat_vec(&kinematics::transpose(&c0.r), &kinematics::sub(&c.c, &c0.c));
            assert!((d[0] - o[0]).abs() < 1e-12 && (d[1] - o[1]).abs() < 1e-12 && d[2].abs() < 1e-12, "{d:?}");
            let px = c.project(&pivot).unwrap();
            assert!((px[0] - centre[0] + cam.fx * o[0] / z).abs() < 0.5 && (px[1] - centre[1] + cam.fy * o[1] / z).abs() < 0.5, "{o:?} {px:?}");
            // frame margin of a 100 x 60 px box at the centre: 330 - 50 - |shift x|, 180 - 30 - |shift y|
            let bx = [centre[0] - 50.0, centre[1] - 30.0, centre[0] + 50.0, centre[1] + 30.0];
            let (sh, m) = frame_prediction(&cam, &p, &pivot, centre, bx);
            let expect = (cam.size[0] / 2.0 - 50.0 - sh[0].abs()).min(cam.size[1] / 2.0 - 30.0 - sh[1].abs());
            assert!((m - expect).abs() < 1e-6 && (sh[0].hypot(sh[1]) - cam.fx * o[0].hypot(o[1]) / z).abs() < 0.5, "{sh:?} {m}");
        }
    }

    /// How far a joint error moves the image (fx 804, a point 0.45 m in front of the camera on
    /// the optical axis) and the depth error that shift means at baseline 0.08 m.
    #[test]
    fn joint_errors_in_pixels() {
        let arm = crate::chain::ArmModel::default();
        let setup = GraspSetup { fx_px: 804.0, fy_px: 804.0, ..GraspSetup::default() };
        let cam = CamModel::assumed(&arm, &setup, &crate::model::CameraMount::default());
        let q0 = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];
        let c0 = camera_at(&cam, &arm.fk(&q0));
        let z = 0.45;
        let x = kinematics::add(&c0.c, &kinematics::mat_vec(&c0.r, &[0.0, 0.0, z]));
        let p0 = c0.project(&x).unwrap();
        let mut shifts = [0.0; 6];
        for (j, s) in shifts.iter_mut().enumerate() {
            let mut q = q0;
            q[j] += 0.0035;
            let p = camera_at(&cam, &arm.fk(&q)).project(&x).unwrap();
            *s = (p[0] - p0[0]).hypot(p[1] - p0[1]);
            println!("J{} +3.5 mrad: {:.2} px, depth error at B 0.08 m {:.1} mm", j + 1, *s, z * z * *s / (804.0 * 0.08) * 1000.0);
        }
        // a few mrad on a wrist joint is several px: the pose must be the one of the snapshot
        assert!(shifts[3] > 2.0 && shifts[3] < 6.0, "{shifts:?}");
    }

    #[test]
    fn timing_overlap_and_sums() {
        let t = Instant::now();
        let ms = |x: u64| t + Duration::from_millis(x);
        // compute 0..800 ms, return 300..2300 ms: 500 ms overlap; disjoint and nested intervals
        assert!((overlap_ms((ms(0), ms(800)), (ms(300), ms(2300))) - 500.0).abs() < 1e-6);
        assert_eq!(overlap_ms((ms(0), ms(100)), (ms(200), ms(300))), 0.0);
        assert!((overlap_ms((ms(100), ms(200)), (ms(0), ms(300))) - 100.0).abs() < 1e-6);
        let v = |m: f64, s: f64| ViewTiming { motion_ms: m, settle_ms: s, trigger_ms: 100.0, fetch_ms: 20.0, capture_ms: 150.0 };
        let run = RunTiming { negotiate_ms: 5.0, detect_ms: 2400.0, views_ms: 9000.0, return_ms: 3000.0, compute_ms: 800.0, overlap_ms: 500.0, total_ms: 14800.0 };
        let j = timing_value(&run, &[v(0.0, 2200.0), v(3000.0, 2200.0), v(3500.0, 2200.0)]);
        assert_eq!((j["motion_ms"].as_f64(), j["settle_ms"].as_f64(), j["trigger_ms"].as_f64(), j["fetch_ms"].as_f64()), (Some(6500.0), Some(6600.0), Some(300.0), Some(60.0)), "{j}");
        assert_eq!((j["compute_return_overlap_ms"].as_f64(), j["total_ms"].as_f64(), j["detect_ms"].as_f64()), (Some(500.0), Some(14800.0), Some(2400.0)), "{j}");
        assert_eq!(v(1.0, 2.0).to_json()["settle_ms"], 2.0);
    }

    #[test]
    fn settle_rule_and_bracket() {
        let arm = [0usize, 1, 2, 3, 4, 5];
        let win = |f: &dyn Fn(usize) -> (Vec<f32>, Vec<f32>)| -> std::collections::VecDeque<(Vec<f32>, Vec<f32>)> { (0..STILL_WINDOW).map(f).collect() };
        // creep of 2.7 mrad over 2 s (0.27 mrad per 200 ms window) and one velocity spike of 0.037: still
        let w = win(&|k| (vec![0.00027 * k as f32 / 10.0; 7], vec![if k == 3 { 0.037 } else { 0.005 }; 6]));
        assert!(is_still(&w, &arm));
        // a joint moving 4 mrad within the window, or a median |velocity| of 0.05: not still
        assert!(!is_still(&win(&|k| { let mut p = vec![0.0; 7]; p[3] = 0.0004 * k as f32; (p, vec![0.0; 6]) }), &arm));
        assert!(!is_still(&win(&|_| (vec![0.0; 7], vec![0.05; 6])), &arm));
        let a = vec![0.0f32; 7];
        let mut b = a.clone();
        b[4] = 0.0025;
        b[6] = 0.5; // the gripper is not an arm joint
        let (d, j) = bracket_spread(&a, &b, &arm);
        assert!((d - 0.0025).abs() < 1e-7);
        assert_eq!(j, 4, "the joint with the largest change");
    }

    #[test]
    fn letterboxed_snapshots_are_cropped_to_the_frame() {
        // a 64x64 JPEG with the 64x36 frame in rows 14..49 (grey 200), black bars
        let (w, h) = (64usize, 64usize);
        let mut img = vec![0u8; w * h];
        for y in 14..50 { for x in 0..w { img[y * w + x] = 200; } }
        let mut out = Vec::new();
        jpeg(&img, w, h, &mut out);
        let g = decode_frame(&out, [64.0, 36.0]).unwrap();
        assert_eq!((g.w, g.h), (64, 36));
        let mean = g.px.iter().map(|&v| f64::from(v)).sum::<f64>() / g.px.len() as f64;
        assert!((mean - 200.0).abs() < 6.0, "{mean}");
        assert!(decode_frame(&out, [32.0, 18.0]).unwrap_err().contains("64x64"));
        assert!(decode_frame(b"not a jpeg", [64.0, 36.0]).is_err());
    }

    fn jpeg(img: &[u8], w: usize, h: usize, out: &mut Vec<u8>) {
        jpeg_encoder::Encoder::new(out, 90).encode(img, w as u16, h as u16, jpeg_encoder::ColorType::Luma).unwrap();
    }

    #[test]
    fn pgm_roundtrip_and_delimiter_first_pixel_edge_cases() {
        // Every whitespace byte and '#' as pixel 0 must survive untouched after the single delimiter.
        for first in [b' ', b'\n', b'\t', b'\r', b'#', 0x00, 0xFF] {
            let mut px = vec![128u8; 64 * 36];
            px[0] = first;
            px[1] = b'#';
            px[64 * 36 - 1] = 255 - first;
            let img = Gray::new(64, 36, px.clone()).unwrap();
            let encoded = encode_pgm(&img);
            assert!(encoded.starts_with(b"P5\n64 36\n255\n"));
            assert_eq!(encoded.len(), "P5\n64 36\n255\n".len() + 64 * 36);
            let decoded = parse_pgm(&encoded, [64.0, 36.0]).unwrap();
            assert_eq!(decoded, img, "first pixel 0x{first:02X}");
        }
        // Comments between tokens before maxval are allowed
        let mut with_comment = b"P5\n# comment\n4 # w\n2\n# max\n255\n".to_vec();
        with_comment.extend_from_slice(&[b'#', b' ', b'\n', 4, 5, 6, 7, 8]);
        let g = parse_pgm(&with_comment, [4.0, 2.0]).unwrap();
        assert_eq!(g.px, vec![b'#', b' ', b'\n', 4, 5, 6, 7, 8]);

        // Reject bad magic, bad maxval, geometry mismatch (incl. fractional dims), truncated, extra byte
        assert!(parse_pgm(b"P2\n4 2\n255\n12345678", [4.0, 2.0]).is_err());
        assert!(parse_pgm(b"P5\n4 2\n65535\n12345678", [4.0, 2.0]).is_err());
        assert!(parse_pgm(b"P5\n4 2\n255\n12345678", [8.0, 2.0]).is_err());
        assert!(parse_pgm(b"P5\n4 2\n255\n12345678", [4.4, 2.0]).is_err());
        assert!(parse_pgm(b"P5\n4 2\n255\n12345678", [4.0, 2.2]).is_err());
        assert!(parse_pgm(b"P5\n4 2\n255\n1234567", [4.0, 2.0]).is_err());
        assert!(parse_pgm(b"P5\n4 2\n255\n123456789", [4.0, 2.0]).is_err());
    }

    #[test]
    fn read_bundle_supports_v1_jpeg_v2_pgm_and_rejects_bad_versions_and_encodings() {
        let dir = tempfile::tempdir().unwrap();
        let cam = Camera { r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], c: [0.0; 3], fx: 60.0, fy: 60.0, cx: 32.0, cy: 18.0 };
        let mut raw_px = vec![0u8; 64 * 36];
        for (i, p) in raw_px.iter_mut().enumerate() { *p = ((i * 7 + 13) & 0xFF) as u8; }
        raw_px[0] = b'#';
        let gray = Gray::new(64, 36, raw_px.clone()).unwrap();
        let mut sq = vec![0u8; 64 * 64];
        sq[14 * 64..50 * 64].copy_from_slice(&raw_px);
        let mut jpg_bytes = Vec::new();
        jpeg(&sq, 64, 64, &mut jpg_bytes);

        // 1. Legacy v1 JPEG bundle (no per-view encoding field)
        let v1_dir = dir.path().join("v1");
        std::fs::create_dir_all(&v1_dir).unwrap();
        std::fs::write(v1_dir.join("view_0.jpg"), &jpg_bytes).unwrap();
        let v1_doc = json!({
            "version": 1, "intrinsics": {"fx": 60.0, "fy": 60.0, "cx": 32.0, "cy": 18.0, "size": [64.0, 36.0]},
            "table_z_m": 0.0, "reference": 0, "ref_box_px": [10.0, 10.0, 30.0, 30.0],
            "views": [{"file": "view_0.jpg", "camera": {"r_base_cam": cam.r, "c_m": cam.c}}]
        });
        std::fs::write(v1_dir.join("bundle.json"), serde_json::to_vec(&v1_doc).unwrap()).unwrap();
        let b1 = read_bundle(&v1_dir).unwrap();
        assert_eq!((b1.views.len(), b1.views[0].img.w, b1.views[0].img.h), (1, 64, 36));

        // 2. v2 PGM bundle via write_replay_bundle_gray (exact pixel roundtrip!)
        let v2_dir = dir.path().join("v2_pgm");
        write_replay_bundle_gray(&v2_dir, std::slice::from_ref(&gray), &[cam], [64.0, 36.0], 0, [10.0, 10.0, 30.0, 30.0], 0.0).unwrap();
        let b2 = read_bundle(&v2_dir).unwrap();
        assert_eq!(b2.views.len(), 1);
        assert_eq!(b2.views[0].img, gray);

        // 3. Reject unknown version, missing encoding in v2, unknown encoding, and geometry mismatch
        let bad_dir = dir.path().join("bad");
        std::fs::create_dir_all(&bad_dir).unwrap();
        std::fs::write(bad_dir.join("view_0.pgm"), encode_pgm(&gray)).unwrap();
        for mut doc in [
            json!({"version": 3, "intrinsics": {"fx": 60.0, "fy": 60.0, "cx": 32.0, "cy": 18.0, "size": [64.0, 36.0]}, "table_z_m": 0.0, "reference": 0, "ref_box_px": [10.0, 10.0, 30.0, 30.0], "views": []}),
            json!({"version": 2, "intrinsics": {"fx": 60.0, "fy": 60.0, "cx": 32.0, "cy": 18.0, "size": [64.0, 36.0]}, "table_z_m": 0.0, "reference": 0, "ref_box_px": [10.0, 10.0, 30.0, 30.0], "views": [{"file": "view_0.pgm", "camera": {"r_base_cam": cam.r, "c_m": cam.c}}]}),
            json!({"version": 2, "intrinsics": {"fx": 60.0, "fy": 60.0, "cx": 32.0, "cy": 18.0, "size": [64.0, 36.0]}, "table_z_m": 0.0, "reference": 0, "ref_box_px": [10.0, 10.0, 30.0, 30.0], "views": [{"file": "view_0.pgm", "encoding": "png", "camera": {"r_base_cam": cam.r, "c_m": cam.c}}]}),
            json!({"version": 2, "intrinsics": {"fx": 60.0, "fy": 60.0, "cx": 32.0, "cy": 18.0, "size": [128.0, 72.0]}, "table_z_m": 0.0, "reference": 0, "ref_box_px": [10.0, 10.0, 30.0, 30.0], "views": [{"file": "view_0.pgm", "encoding": "pgm", "camera": {"r_base_cam": cam.r, "c_m": cam.c}}]}),
        ] {
            std::fs::write(bad_dir.join("bundle.json"), serde_json::to_vec(&doc).unwrap()).unwrap();
            assert!(read_bundle(&bad_dir).is_err(), "{doc}");
            doc["version"] = Value::Null;
            std::fs::write(bad_dir.join("bundle.json"), serde_json::to_vec(&doc).unwrap()).unwrap();
            assert!(read_bundle(&bad_dir).is_err());
        }
    }

    #[test]
    fn ncc_backend_diagnostics_reported_in_compute_measurement_and_bundle() {
        let (w, h) = (320usize, 240usize);
        let (fx, fy, cx, cy) = (400.0, 400.0, 160.0, 120.0);
        let r = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
        let tex = |wx: f64, wy: f64| -> u8 {
            let cx_i = (wx * 80.0).floor() as i32;
            let cy_i = (wy * 80.0).floor() as i32;
            let checker = if (cx_i + cy_i) & 1 == 0 { 190.0 } else { 65.0 };
            let fine = 25.0 * (wx * 310.0).sin() * (wy * 290.0).cos();
            (checker + fine).clamp(0.0, 255.0) as u8
        };
        let offsets = [-0.04, 0.0, 0.04];
        let views: Vec<View> = offsets.iter().map(|&ox| {
            let c = [ox, 0.0, 0.40];
            let mut px = vec![0u8; w * h];
            for v in 0..h {
                for u in 0..w {
                    let wx = ox + 0.40 * (u as f64 - cx) / fx;
                    let wy = -0.40 * (v as f64 - cy) / fy;
                    px[v * w + u] = tex(wx, wy);
                }
            }
            View {
                cam: Camera { r, c, fx, fy, cx, cy },
                img: Gray::new(w, h, px).unwrap(),
            }
        }).collect();

        let ref_box_px = [100.0, 70.0, 220.0, 170.0];
        let c = compute(&views, 1, ref_box_px, 0.0, PoseRefine::Off, &[]);

        let expected_backend = mview_stereo::backend_diagnostics();
        assert_eq!(c.ncc_backend.backend, expected_backend.backend);
        assert_eq!(c.ncc_backend.reason, expected_backend.reason);
        #[cfg(any(feature = "xthead-ncc", feature = "mview-profile"))]
        {
            assert!(!c.ncc_backend.reason.is_empty(), "reason should be non-empty");
            let total_kernel_calls = c.ncc_backend.counts.coarse_dots_scalar
                + c.ncc_backend.counts.coarse_dots_xthead071
                + c.ncc_backend.counts.patch_moments_scalar
                + c.ncc_backend.counts.patch_moments_xthead071;
            assert!(
                total_kernel_calls > 0,
                "expected NCC kernel counts > 0 across backends, got {:?}",
                c.ncc_backend.counts
            );
        }
        #[cfg(not(any(feature = "xthead-ncc", feature = "mview-profile")))]
        {
            assert_eq!(c.ncc_backend.reason, "scalar (built without xthead-ncc)");
            assert_eq!(
                c.ncc_backend.counts,
                mview_stereo::KernelCallCounts::default(),
                "expected zero kernel counts when built without xthead-ncc or mview-profile"
            );
        }

        let arm = crate::chain::ArmModel::default();
        let setup = GraspSetup { image_size_px: [w as f64, h as f64], fx_px: fx, fy_px: fy, cx_px: Some(cx), cy_px: Some(cy), ..GraspSetup::default() };
        let cam = CamModel::assumed(&arm, &setup, &crate::model::CameraMount::default());
        let plan = Plan {
            request: parse_request(&Value::Null).unwrap(),
            epoch: 1,
            workspace_revision: 1,
            calibration_revision: 1,
            setup,
            cam,
            start: vec![0.0; 6],
            arm_dof: 6,
            deadline_s: 30.0,
        };
        let caps: Vec<Capture> = offsets.iter().enumerate().map(|(i, &ox)| Capture {
            offset: [ox, 0.0],
            pose: Pose::new(r, [ox, 0.0, 0.40]),
            img: Gray { w: 0, h: 0, px: Vec::new() },
            frame_source: FrameSource::Snapshot,
            instance_id: None,
            frame_id: i as u64 + 1,
            detections: Vec::new(),
            decode_ms: if i == 0 { 1.236 } else { 1.0 },
            frame_fetch_ms: 0.0,
            gray_convert_ms: None,
            parse_ms: 0.0,
            shift_px: [0.0, 0.0],
            margin_px: 20.0,
            joints: vec![0.0; 6],
            bracket: [vec![0.0; 6], vec![0.0; 6]],
            snapshot_tries: vec![0.0],
            retry_reasons: Vec::new(),
            bracket_duration_ms: 0.0,
            feedback_wait_b_ms: 0.0,
            feedback_age_a_ms: None,
            feedback_age_b_ms: None,
            settle_s: 2.0,
            trigger_ms: 10.0,
            snapshot_wait_ms: 50.0,
            snapshot_polls: 1,
            jpeg: None,
            markers: Vec::new(),
            timing: ViewTiming { motion_ms: 1000.0, settle_ms: 2000.0, trigger_ms: 100.0, fetch_ms: 10.0, capture_ms: 150.0 },
        }).collect();
        let det = Detection {
            label: "box".into(),
            score: 0.9,
            raw_score_valid: true,
            box_norm: [100.0 / w as f64, 70.0 / h as f64, 220.0 / w as f64, 170.0 / h as f64],
            angle_deg: None,
            angle_conf: None,
            grasp_width_px: None,
            mask_focus: false,
            disp_fg: None,
            disp_bg: None,
            depth_conf: None,
            length_px: None,
            jaw_clear: None,
            depth_angle_deg: None,
            depth_angle_conf: None,
            depth_width_px: None,
        };
        let mut stats = c.rec.stats;
        stats.sweep_ms = 12.346;
        let val = measurement_value(&plan, FrameSource::Snapshot, &caps, &det, &c.measure, &c.table, &stats, &c.ncc_backend, &[], c.ms);
        assert_eq!(val["stats"]["ncc_backend"], json!(c.ncc_backend.backend));
        assert_eq!(val["stats"]["ncc_backend_reason"], json!(c.ncc_backend.reason));
        assert_eq!(val["stats"]["sweep_ms"], json!(12.35));
        assert_eq!(val["stats"]["decode_ms"], json!(3.24));
        let expected_diag = serde_json::to_value(&c.ncc_backend).unwrap();
        assert_eq!(val["ncc_backend"], expected_diag);
        assert!(val["ncc_backend"]["counts"].is_object());

        let bundle = bundle_doc(42, &plan, FrameSource::Snapshot, &caps, 1, ref_box_px, &c, &val);
        assert_eq!(bundle["ncc_backend"], expected_diag);
        assert_eq!(bundle["ncc_backend"]["backend"], json!(c.ncc_backend.backend));
        assert_eq!(bundle["ncc_backend"]["reason"], json!(c.ncc_backend.reason));
        assert_eq!(bundle["ncc_backend"]["counts"], serde_json::to_value(c.ncc_backend.counts).unwrap());
    }
}
