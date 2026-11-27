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
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::kinematics::{self, Mat3, Pose, Vec3};
use crate::model::HoldingMode;
use crate::estimate::{EstimateChain, EstimateEntry, SourceResult};
use crate::macro_action::{fail, ContactGuard, MacroCtx, Stop, SubDone};
use crate::motion::{self, GoalFrame, GoalSpec, GripperCommand, MotionError, RelFrame, Speed};
use crate::supervisor::{ArmError, ArmSupervisor};

pub type BoxFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;


mod setup;
mod tasks;

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

mod perception;

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

mod depth;
mod geometry;

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

mod admission;
mod planning;

pub use admission::{
    begin, begin_executor, begin_macro_spec, begin_with_stamp, spawn, spawn_with_deadline, start,
};
pub(crate) use admission::begin_executor_ws;

pub use planning::{
    dry_run, dry_run_task, expected_duration, parse_grasp, plan, prepare, prepare_from_snapshot,
    transition_estimate, DryRun, GraspRequest, PendingAdaptive, PendingGrasp, PlannedAdaptive,
    PlannedGrasp,
};
use planning::{approach_pose, grasp_plane};

// ---- orchestrator ----

const POLL: Duration = crate::macro_action::POLL;

/// Grasp phases on top of the shared macro-action primitives ([`MacroCtx`], via `Deref`).
pub struct Orchestrator {
    pub ctx: MacroCtx,
    pub source: Arc<dyn DetectionSource>,
    pub setup: GraspSetup,
    pub cam: CamModel,
    /// Points kept above the table on every sub-motion ([`envelope`]).
    pub envelope: Vec<Vec3>,
    /// Active tool TCP offset in `end_link` frame (m).
    pub tcp_offset: [f64; 3],
    /// Each centring step triggers its own frame (perception API 2) instead of reading the
    /// continuous results.
    pub triggered: bool,
    /// `camera_mount.offset_m` is set: the lens position (height probe) is known.
    pub lens_known: bool,
    /// Perception API depth pipeline is available.
    pub depth_available: bool,
    /// The provider's model generation at the start: a frame from another one fails the grasp.
    pub generation: Option<u64>,
}

/// A guarded grasp insertion ([`Orchestrator::insert`]): the TCP goal, the linear speed and
/// the same-side approach request of its final segment.
#[derive(Debug, Clone, Copy)]
struct Insertion {
    goal_tcp: Vec3,
    speed_mps: f64,
    approach: crate::backlash::Request,
}

impl std::ops::Deref for Orchestrator {
    type Target = MacroCtx;
    fn deref(&self) -> &MacroCtx { &self.ctx }
}

impl Orchestrator {
    /// Applies a diagnostic `ActionStatus` update on Legacy owner, or emits a stream patch via
    /// `rt_core::stream` on `OWNER_EXECUTOR` (0 `SupervisorLock`, 0 RT allocation).
    async fn update_action_if_legacy(&self, f: impl FnOnce(&mut crate::motion::ActionStatus)) {
        if !self.is_executor() {
            self.sup.inner().write().await.update_action(self.id, f);
        }
    }

    fn emit_diag(&self, kind: crate::rt_core::action_diag::ActionDiagKind) {
        if self.is_executor() {
            let _ = self.sup.hub().push_action_diag(
                crate::rt_core::action_diag::ActionDiagPatch {
                    action_id: self.id,
                    kind,
                },
            );
        }
    }

    fn emit_estimates(
        &self,
        list: crate::rt_core::action_diag::EstimateListKind,
        entries: &[crate::motion::EstimateEntry],
    ) {
        if self.is_executor() {
            self.sup
                .hub()
                .push_action_estimates(self.id, list, entries);
        }
    }

    async fn arm_dof(&self) -> usize {
        if self.is_executor() {
            self.sup
                .hub()
                .prep_env()
                .map_or(6, |e| e.config.arm_model.arm_dof())
        } else {
            self.sup.inner().read().await.config.arm_model.arm_dof()
        }
    }

    async fn gripper_policy(&self) -> Option<crate::model::GripperPolicy> {
        if self.is_executor() {
            self.sup
                .hub()
                .prep_env()
                .and_then(|e| e.config.gripper_policy.clone())
        } else {
            self.sup.inner().read().await.config.gripper_policy.clone()
        }
    }

    async fn held_width_m(&self) -> Option<f32> {
        if self.is_executor() {
            let env = self.sup.hub().prep_env()?;
            let snap = self.sup.hub().latest()?;
            let require_real = env.requires_real_jaw_curve;
            let p = env.config.gripper_policy.as_ref()?;
            if require_real && p.jaw_opening.is_none() {
                return None;
            }
            let idx = env.config.gripper_index().unwrap_or(6);
            let m = snap.rt.positions.get(idx).copied().unwrap_or(0.0);
            Some(p.opening_m(m))
        } else {
            let inner = self.sup.inner().read().await;
            let require_real = inner.requires_real_jaw_curve();
            inner.config.gripper_policy.as_ref().and_then(|p| {
                if require_real && p.jaw_opening.is_none() {
                    return None;
                }
                let idx = inner.config.gripper_index().unwrap_or(6);
                let m = inner.measured_positions.get(idx).copied().unwrap_or(0.0);
                Some(p.opening_m(m))
            })
        }
    }

    /// Every grasp sub-motion in `EndLink` frame: WORKSPACE_VIOLATION before anything is
    /// installed when its straight path takes the envelope below table clearance.
    async fn move_arm(&self, phase: &str, spec: GoalSpec, linear: f64) -> Result<SubDone, Stop> {
        self.move_arm_in(GoalFrame::EndLink, phase, spec, linear).await
    }

    async fn move_arm_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, linear: f64) -> Result<SubDone, Stop> {
        self.move_arm_final_in(frame, phase, spec, linear, None).await
    }

    /// [`move_arm_in`](Self::move_arm_in) with an optional same-side final approach.
    async fn move_arm_final_in(&self, frame: GoalFrame, phase: &str, spec: GoalSpec, linear: f64, approach: Option<crate::macro_action::FinalApproach>) -> Result<SubDone, Stop> {
        self.move_arm_final_in_guarded(frame, phase, spec, linear, approach, None)
            .await
            .map(|(done, _)| done)
    }

    /// [`move_arm_final_in`](Self::move_arm_final_in) folding an optional [`ContactGuard`]
    /// atomically into `ChildSpec.contact` at `InstallChild` time (0 lock pre-check on Executor).
    async fn move_arm_final_in_guarded(
        &self,
        frame: GoalFrame,
        phase: &str,
        spec: GoalSpec,
        linear: f64,
        approach: Option<crate::macro_action::FinalApproach>,
        contact: Option<ContactGuard>,
    ) -> Result<(SubDone, crate::macro_action::ContactWatch), Stop> {
        if !self.alive().await {
            return Err(Stop::Cancelled);
        }
        let snap = self.ctx.snapshot(self.deadline).await?;
        let input = snap.plan_input_in(frame).map_err(crate::macro_action::arm_fail)?;
        let spec = if input.arm.arm_dof() == 5 {
            match spec {
                GoalSpec::Absolute { p, r } => GoalSpec::AbsoluteEx { p, r, auto_yaw: true },
                other => other,
            }
        } else {
            spec
        };
        let (goal, _yaw_free) = motion::resolve_goal_ex(&input, &spec);
        let goal_end = motion::goal_to_end(&goal, &input);
        if let Some(e) = table_violation(&input.arm.fk(&input.q0), &goal_end, &self.envelope, self.setup.table_z_m) {
            if let MotionError::WorkspaceViolation { ref detail, .. } = e {
                if self.is_executor() {
                    self.sup
                        .hub()
                        .push_action_workspace_violation(self.id, (**detail).clone());
                } else {
                    self.sup.inner().write().await.set_action_workspace_violation(self.id, (**detail).clone());
                }
            }
            return Err(crate::macro_action::arm_fail(e.into()));
        }
        self.ctx.move_arm_staged_in_with_table_guarded(
            frame,
            phase,
            spec,
            Speed { linear, angular: motion::DEFAULT_ANGULAR_RPS },
            approach,
            Some((self.envelope.clone(), self.setup.table_z_m + TABLE_CLEARANCE_M)),
            contact,
        ).await.map_err(|(stop, _)| stop)
    }

    /// The frame ran with the model set the grasp started with (or either side does not report it).
    fn same_models(&self, frame: &DetectionFrame) -> bool {
        !matches!((self.generation, frame.model_generation), (Some(a), Some(b)) if a != b)
    }

    async fn set_mask_focus(&self, label: Option<String>, box_norm: [f64; 4], ttl_ms: u64) {
        if let Err(e) = self.source.set_mask_focus(label, box_norm, ttl_ms).await {
            tracing::debug!("Mask focus request failed (ignored): {e}");
        }
    }

    /// Next fresh frame (captured after `since` + settle delay) containing the target.
    /// TARGET_LOST when: frames keep arriving but all predate the settle time, `lost_timeout_s`
    /// after max(now, settle time) (stale frames do not extend it); no frame arrives for
    /// max(3 s, `lost_timeout_s`); fresh frames miss the target `lost_frames` times or for
    /// `lost_timeout_s` from the call.
    async fn fresh_match(&self, tracker: &Tracker, since: Instant, after: &mut Option<u64>) -> Result<Detection, Stop> {
        let not_before = since + Duration::from_secs_f64(self.setup.settle_delay_s);
        let started = Instant::now();
        let observe_until = started.max(not_before) + Duration::from_secs_f64(self.setup.lost_timeout_s);
        let mut last_frame = Instant::now();
        let mut unmatched = 0u32;
        loop {
            self.check().await?;
            let source = self.source.clone();
            let a = *after;
            let triggered = self.triggered;
            let focus = MaskFocus {
                label: Some(tracker.label.clone()),
                box_norm: tracker.predicted,
                ttl_ms: 2000,
            };
            let get = async move {
                if triggered {
                    // the triggered frame is taken once the arm has settled
                    tokio::time::sleep(not_before.saturating_duration_since(Instant::now())).await;
                    source.trigger_with_focus(GRASP_PIPELINES, TRIGGER_WAIT_MS, Some(&focus)).await
                } else {
                    source.next_frame(a, 200).await
                }
            };
            let next = tokio::select! {
                r = get => r,
                _ = async { loop { tokio::time::sleep(POLL).await; if !self.alive().await { break; } } } => return Err(Stop::Cancelled),
            };
            let mut frame = match next {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    if last_frame.elapsed().as_secs_f64() > self.setup.lost_timeout_s.max(3.0) {
                        return Err(fail("TARGET_LOST", "No detection frames arrived"));
                    }
                    continue;
                }
                Err(e) => return Err(fail("DETECTION_UNAVAILABLE", format!("Detection source failed: {e}"))),
            };
            if !self.same_models(&frame) { return Err(fail("DETECTION_UNAVAILABLE", "model changed during grasp")); }
            rescale_frame_to_cam(&mut frame, &self.cam);
            *after = Some(frame.frame_id);
            last_frame = Instant::now();
            if frame.captured_at < not_before {
                if last_frame > observe_until {
                    return Err(fail("TARGET_LOST", format!("No frame captured after the motion settled within {:.1} s", self.setup.lost_timeout_s)));
                }
                continue;
            }
            if let Some(d) = tracker.find(&frame, &self.setup) { return Ok(d); }
            unmatched += 1;
            if unmatched >= self.setup.lost_frames || started.elapsed().as_secs_f64() > self.setup.lost_timeout_s {
                return Err(fail("TARGET_LOST", format!("Target '{}' not seen for {unmatched} frames / {:.1} s", tracker.label, started.elapsed().as_secs_f64())));
            }
        }
    }

    /// Dispatches either Phase B adaptive execution or legacy visual-servo execution.
    async fn run_selected(&self, planned: &PlannedGrasp) -> Result<(bool, Option<f32>), Stop> {
        if let Some(pa) = planned.adaptive.as_ref() {
            self.run_adaptive(planned, pa).await
        } else {
            self.run(planned).await
        }
    }

    fn check_adaptive_revisions(
        phase: &str,
        env: &crate::rt_core::requests::actions::ActionPrepEnv,
        rev: crate::grasp_select::AdaptiveRevisions,
    ) -> Result<(), Stop> {
        if env.calibration_revision != rev.calibration_revision {
            return Err(fail(
                "CALIBRATION_CHANGED",
                format!("[{phase}] calibration changed during adaptive grasp"),
            ));
        }
        if env.workspace_revision != rev.workspace_revision
            || env.grasp_setup_revision != rev.grasp_setup_revision
            || env.gripper_calib_revision != rev.gripper_calib_revision
            || env.power_budget_revision != rev.power_budget_revision
        {
            return Err(fail(
                "PLAN_STALE",
                format!("[{phase}] configuration changed during adaptive grasp"),
            ));
        }
        Ok(())
    }

    async fn adaptive_snapshot_fenced(
        &self,
        phase: &str,
        rev: crate::grasp_select::AdaptiveRevisions,
    ) -> Result<crate::rt_core::requests::actions::ActionSnapshot, Stop> {
        let snap = self.ctx.snapshot(self.deadline).await?;
        Self::check_adaptive_revisions(phase, &snap.env, rev)?;
        Ok(snap)
    }

    /// Coherent, revision-fenced, epoch-checked, freshness-verified measured joint state and TCP
    /// pose on both Legacy and RT Executor owners (0 `SupervisorLock` on `OWNER_EXECUTOR`).
    /// Never falls back silently to commanded FK when physical driver feedback is stale or missing.
    async fn measured_state_fenced(
        &self,
        phase: &str,
        rev: crate::grasp_select::AdaptiveRevisions,
    ) -> Result<(Vec<f32>, Pose), Stop> {
        if self.is_executor() {
            let Some(env) = self.sup.hub().prep_env() else {
                return Err(fail("OBSERVATION_UNAVAILABLE", format!("[{phase}] no published planning environment")));
            };
            Self::check_adaptive_revisions(phase, &env, rev)?;
            let Some(snap) = self.sup.hub().latest() else {
                return Err(fail("OBSERVATION_UNAVAILABLE", format!("[{phase}] no published telemetry snapshot")));
            };
            if snap.rt.owner.authority.epoch != self.ctx.stamp.estop_epoch {
                return Err(Stop::Cancelled);
            }
            if !snap.rt.owner.macro_state.alive || snap.rt.owner.macro_state.active_id != self.id {
                return Err(Stop::Cancelled);
            }
            if env.driver_present {
                let now = Instant::now();
                let fresh = !snap.rt.feedback_instants.is_empty()
                    && snap.rt.feedback_instants.iter().all(|t| {
                        t.is_some_and(|ts| now.saturating_duration_since(ts) <= Duration::from_millis(250))
                    });
                if !fresh {
                    return Err(fail(
                        "FEEDBACK_STALE",
                        format!("[{phase}] measured joint feedback is stale or unavailable"),
                    ));
                }
            }
            if snap.rt.positions.iter().any(|v| !v.is_finite()) {
                return Err(fail(
                    "FEEDBACK_STALE",
                    format!("[{phase}] measured joint positions contain non-finite values"),
                ));
            }
            let Some(end) = env.config.arm_model.fk_f32(&snap.rt.positions) else {
                return Err(fail(
                    "OBSERVATION_UNAVAILABLE",
                    format!("[{phase}] cannot compute measured FK"),
                ));
            };
            Ok((snap.rt.positions.clone(), motion::end_to_tcp(&end, &self.tcp_offset)))
        } else {
            let inner = self.sup.inner().read().await;
            if !inner.macro_alive(self.id) || inner.safety.epoch_id() != self.ctx.stamp.estop_epoch {
                return Err(Stop::Cancelled);
            }
            if inner.calibration_revision != rev.calibration_revision {
                return Err(fail("CALIBRATION_CHANGED", format!("[{phase}] calibration changed during adaptive grasp")));
            }
            if inner.workspace_revision != rev.workspace_revision
                || inner.grasp_setup_revision != rev.grasp_setup_revision
                || inner.gripper_calib_revision != rev.gripper_calib_revision
                || inner.power_budget_revision != rev.power_budget_revision
            {
                return Err(fail("PLAN_STALE", format!("[{phase}] configuration changed during adaptive grasp")));
            }
            if inner.driver.is_some() && !inner.hardware_feedback_fresh(Duration::from_millis(250)) {
                return Err(fail(
                    "FEEDBACK_STALE",
                    format!("[{phase}] measured hardware feedback is stale or unavailable"),
                ));
            }
            if inner.measured_positions.iter().any(|v| !v.is_finite()) {
                return Err(fail(
                    "FEEDBACK_STALE",
                    format!("[{phase}] measured joint positions contain non-finite values"),
                ));
            }
            let Some(end) = inner.config.arm_model.fk_f32(&inner.measured_positions) else {
                return Err(fail(
                    "OBSERVATION_UNAVAILABLE",
                    format!("[{phase}] cannot compute measured FK"),
                ));
            };
            Ok((inner.measured_positions.clone(), motion::end_to_tcp(&end, &self.tcp_offset)))
        }
    }

    /// Strict post-settle vision revalidation for Phase B adaptive grasps:
    /// - Enforces strict `model_generation` equality when the provider reports generations
    ///   (`DETECTION_UNAVAILABLE` on mismatch or omission).
    /// - Rejects stale/cached frames (`frame_id <= after`, `captured_at < not_before`,
    ///   `policy_captured_at < not_before`, and in stream mode without `age_ms`, requires a
    ///   strictly newer frame past a post-settle barrier frame).
    /// - Verifies target label, rejects ambiguous same-label duplicates, checks pixel error,
    ///   base-frame XY displacement on the target mid-plane, projected box area ratio, and
    ///   orientation consistency (contour principal axis / angle / elongated box aspect).
    #[allow(clippy::too_many_arguments)]
    async fn fresh_match_adaptive(
        &self,
        phase: &str,
        label: &str,
        route: &crate::grasp_select::AdaptiveRoute,
        expected_gen: &mut Option<u64>,
        pose_end: &Pose,
        since: Instant,
        after: &mut Option<u64>,
    ) -> Result<Detection, Stop> {
        let s = &self.setup;
        let z_mid = route.target_table_z + (route.target_height[0] + route.target_height[1]) / 4.0;
        let target_mid = [route.target_centroid[0], route.target_centroid[1], z_mid];
        let Some(pred_px) = self.cam.project(pose_end, &target_mid) else {
            return Err(fail(
                "TARGET_LOST",
                format!("[{phase}] expected target centre is behind the camera"),
            ));
        };
        if !(pred_px[0] >= 0.0
            && pred_px[0] <= self.cam.size[0]
            && pred_px[1] >= 0.0
            && pred_px[1] <= self.cam.size[1])
        {
            return Err(fail(
                "TARGET_LOST",
                format!("[{phase}] expected target centre projects outside the camera view"),
            ));
        }

        // Expected projected bounding box at `pose_end` from the planned target footprint.
        let mut exp_min = [f64::INFINITY; 2];
        let mut exp_max = [f64::NEG_INFINITY; 2];
        let mut any_proj = false;
        for z in [route.target_table_z, route.target_table_z + route.target_height[1]] {
            for v in &route.target_poly {
                if let Some(px) = self.cam.project(pose_end, &[v[0], v[1], z]) {
                    exp_min[0] = exp_min[0].min(px[0]);
                    exp_min[1] = exp_min[1].min(px[1]);
                    exp_max[0] = exp_max[0].max(px[0]);
                    exp_max[1] = exp_max[1].max(px[1]);
                    any_proj = true;
                }
            }
        }
        let (exp_w, exp_h, exp_area, focus_box) = if any_proj {
            let w = (exp_max[0] - exp_min[0]).max(1.0);
            let h = (exp_max[1] - exp_min[1]).max(1.0);
            let lo = self.cam.to_norm([exp_min[0].clamp(0.0, self.cam.size[0]), exp_min[1].clamp(0.0, self.cam.size[1])]);
            let hi = self.cam.to_norm([exp_max[0].clamp(0.0, self.cam.size[0]), exp_max[1].clamp(0.0, self.cam.size[1])]);
            (w, h, w * h, [lo[0], lo[1], hi[0], hi[1]])
        } else {
            let c = self.cam.to_norm(pred_px);
            (0.0, 0.0, 0.0, [(c[0] - 0.08).clamp(0.0, 1.0), (c[1] - 0.08).clamp(0.0, 1.0), (c[0] + 0.08).clamp(0.0, 1.0), (c[1] + 0.08).clamp(0.0, 1.0)])
        };

        // Unified vision deadline matching `preflight_candidate_from_start::adaptive_vision_budget_s`.
        let vision_budget_s = crate::grasp_select::adaptive_vision_budget_s(s);
        let not_before = since + Duration::from_secs_f64(s.settle_delay_s.max(0.0));
        let started = Instant::now();
        let observe_until = (since + Duration::from_secs_f64(vision_budget_s)).min(self.deadline);
        let mut unmatched = 0u32;
        let mut stream_barrier_id: Option<u64> = None;

        loop {
            self.check().await?;
            let now_pre = Instant::now();
            if now_pre > observe_until {
                return Err(fail(
                    "TARGET_LOST",
                    format!("[{phase}] vision revalidation budget ({vision_budget_s:.2} s) exceeded"),
                ));
            }
            let source = self.source.clone();
            let prev_id = *after;
            let triggered = self.triggered;
            let focus = MaskFocus {
                label: Some(label.to_string()),
                box_norm: focus_box,
                ttl_ms: 2000,
            };
            let rem_ms = observe_until
                .saturating_duration_since(now_pre)
                .as_millis()
                .clamp(1, TRIGGER_WAIT_MS as u128) as u64;
            let get = async move {
                if triggered {
                    let wait = not_before.saturating_duration_since(Instant::now());
                    if !wait.is_zero() {
                        tokio::time::sleep(wait).await;
                    }
                    let trig_ms = observe_until
                        .saturating_duration_since(Instant::now())
                        .as_millis()
                        .clamp(1, TRIGGER_WAIT_MS as u128) as u64;
                    source
                        .trigger_with_focus(GRASP_PIPELINES, trig_ms, Some(&focus))
                        .await
                } else {
                    source.next_frame(prev_id, rem_ms.min(200)).await
                }
            };
            let next = tokio::select! {
                r = get => r,
                _ = async {
                    loop {
                        tokio::time::sleep(POLL).await;
                        if !self.alive().await || Instant::now() > observe_until {
                            break;
                        }
                    }
                } => {
                    self.check().await?;
                    return Err(fail(
                        "TARGET_LOST",
                        format!("[{phase}] no fresh detection frame within vision budget ({vision_budget_s:.2} s)"),
                    ));
                }
            };
            let mut frame = match next {
                Ok(Some(frame)) => frame,
                Ok(None) => {
                    if Instant::now() > observe_until {
                        return Err(fail(
                            "TARGET_LOST",
                            format!("[{phase}] no fresh detection frames arrived within {vision_budget_s:.2} s"),
                        ));
                    }
                    continue;
                }
                Err(e) => {
                    return Err(fail(
                        "DETECTION_UNAVAILABLE",
                        format!("[{phase}] detection source failed: {e}"),
                    ));
                }
            };
            let now = Instant::now();
            if now > observe_until {
                return Err(fail(
                    "TARGET_LOST",
                    format!("[{phase}] detection frame arrived past vision budget ({vision_budget_s:.2} s)"),
                ));
            }

            // Strict model generation provenance check (latching first observed generation if
            // `/api/mode` reported none so mid-grasp model swaps are still caught).
            if let Some(exp_g) = *expected_gen {
                if frame.model_generation != Some(exp_g) {
                    return Err(fail(
                        "DETECTION_UNAVAILABLE",
                        format!(
                            "[{phase}] vision model generation changed (expected {exp_g}, got {:?})",
                            frame.model_generation
                        ),
                    ));
                }
            } else {
                if !self.same_models(&frame) {
                    return Err(fail(
                        "DETECTION_UNAVAILABLE",
                        format!("[{phase}] vision model changed during adaptive grasp"),
                    ));
                }
                if let Some(g) = frame.model_generation {
                    *expected_gen = Some(g);
                }
            }

            rescale_frame_to_cam(&mut frame, &self.cam);

            // Monotonic frame ID check: never reuse the request frame or an earlier frame.
            if prev_id.is_some_and(|a| frame.frame_id <= a) {
                if now > observe_until {
                    return Err(fail(
                        "TARGET_LOST",
                        format!("[{phase}] no new frame arrived after frame_id {} within {vision_budget_s:.2} s", prev_id.unwrap_or(0)),
                    ));
                }
                tokio::time::sleep(POLL).await;
                continue;
            }
            *after = Some(frame.frame_id);

            // Freshness check: must be captured at or after settle deadline.
            let stale_time = frame.captured_at < not_before
                || frame.policy_captured_at.is_some_and(|pt| pt < not_before);
            if stale_time {
                if now > observe_until {
                    return Err(fail(
                        "TARGET_LOST",
                        format!("[{phase}] no frame captured after settle within {vision_budget_s:.2} s"),
                    ));
                }
                continue;
            }
            // In continuous stream mode without upstream `age_ms` (`policy_captured_at == None`),
            // only arm the post-settle barrier AFTER `not_before` has elapsed and require a
            // strictly newer frame past that barrier so pre-settle queued frames cannot pass.
            if !self.triggered && frame.policy_captured_at.is_none() {
                if now < not_before {
                    continue;
                }
                match stream_barrier_id {
                    None => {
                        stream_barrier_id = Some(frame.frame_id);
                        if now > observe_until {
                            return Err(fail(
                                "TARGET_LOST",
                                format!("[{phase}] no post-settle stream frame arrived within {vision_budget_s:.2} s"),
                            ));
                        }
                        continue;
                    }
                    Some(b_id) if frame.frame_id <= b_id => {
                        if now > observe_until {
                            return Err(fail(
                                "TARGET_LOST",
                                format!("[{phase}] stream frame did not advance past settle barrier within {vision_budget_s:.2} s"),
                            ));
                        }
                        continue;
                    }
                    Some(_) => {}
                }
            }

            // Collect same-label candidates within `acquire_dist_px` of expected projection.
            let mut matches: Vec<(&Detection, [f64; 2], f64)> = frame
                .detections
                .iter()
                .filter(|d| d.label.eq_ignore_ascii_case(label))
                .filter(|d| d.box_norm.iter().all(|v| v.is_finite()) && d.box_norm[2] > d.box_norm[0] && d.box_norm[3] > d.box_norm[1])
                .map(|d| {
                    let c = self.cam.to_px(box_centre(&d.box_norm));
                    let dist = (c[0] - pred_px[0]).hypot(c[1] - pred_px[1]);
                    (d, c, dist)
                })
                .filter(|(_, _, dist)| *dist <= s.acquire_dist_px)
                .collect();

            if matches.is_empty() {
                unmatched += 1;
                if unmatched >= s.lost_frames || now > observe_until {
                    return Err(fail(
                        "TARGET_LOST",
                        format!("[{phase}] target '{label}' not seen for {unmatched} frames / {:.1} s", started.elapsed().as_secs_f64()),
                    ));
                }
                continue;
            }
            matches.sort_by(|a, b| a.2.total_cmp(&b.2).then(b.0.score.total_cmp(&a.0.score)));
            let (best, best_c, best_dist) = matches[0];

            // Unconditionally reject ambiguous duplicate same-label detection inside dup_gate.
            let dup_gate = s.match_dist_px.max(s.recheck_tol_px);
            if let Some((_, _, second_dist)) = matches.get(1).copied() {
                if second_dist <= dup_gate {
                    return Err(fail(
                        "TARGET_DISPLACED",
                        format!("[{phase}] ambiguous duplicate '{label}' detection at {:.1} px vs {:.1} px", best_dist, second_dist),
                    ));
                }
            }

            // Pixel recheck tolerance.
            if best_dist > s.recheck_tol_px {
                return Err(fail(
                    "TARGET_DISPLACED",
                    format!("[{phase}] target '{label}' shifted {best_dist:.1} px (recheck_tol_px {:.1})", s.recheck_tol_px),
                ));
            }

            // Base-frame XY displacement strictly bounded by the selected candidate's opening reserve.
            if let Some(meas_pt) = self.cam.plane_point(pose_end, best_c, z_mid) {
                let dxy = (meas_pt[0] - route.target_centroid[0])
                    .hypot(meas_pt[1] - route.target_centroid[1]);
                let xy_tol = route.opening_reserve_m.clamp(0.005, 0.015);
                if dxy > xy_tol {
                    return Err(fail(
                        "TARGET_DISPLACED",
                        format!("[{phase}] target '{label}' moved {dxy:.3} m on table (tol {xy_tol:.3} m)"),
                    ));
                }
            }

            // Projected bounding box area sanity check.
            let meas_w = ((best.box_norm[2] - best.box_norm[0]) * self.cam.size[0]).max(1.0);
            let meas_h = ((best.box_norm[3] - best.box_norm[1]) * self.cam.size[1]).max(1.0);
            let meas_area = meas_w * meas_h;
            if exp_area > 16.0 {
                let ratio = meas_area / exp_area;
                if !(0.40..=2.20).contains(&ratio) {
                    return Err(fail(
                        "TARGET_DISPLACED",
                        format!("[{phase}] target '{label}' projected area ratio {ratio:.2} outside [0.40, 2.20]"),
                    ));
                }
            }

            // Orientation consistency when target long axis is non-ambiguous (tightened to 12 deg).
            if !route.target_axis_ambiguous {
                let exp_axis = route.target_axis;
                let axis_angle_deg = |obs: [f64; 2]| -> f64 {
                    let c = (obs[0] * exp_axis[0] + obs[1] * exp_axis[1]).abs().clamp(0.0, 1.0);
                    c.acos().to_degrees()
                };
                if let Some(Ok(ref pts)) = best.contour_norm {
                    if pts.len() >= 3 {
                        let world_pts: Option<Vec<[f64; 2]>> = pts
                            .iter()
                            .map(|v| self.cam.plane_point(pose_end, self.cam.to_px(*v), z_mid).map(|p| [p[0], p[1]]))
                            .collect();
                        if let Some(wpts) = world_pts {
                            let (obs_axis, obs_ratio) = crate::grasp_select::geometry::principal_axis(&wpts);
                            if obs_ratio >= 1.15 {
                                let deg = axis_angle_deg(obs_axis);
                                if deg > 12.0 {
                                    return Err(fail(
                                        "TARGET_DISPLACED",
                                        format!("[{phase}] target '{label}' contour rotated by {deg:.1} deg (limit 12.0 deg)"),
                                    ));
                                }
                            }
                        }
                    }
                } else if let (Some(a_deg), Some(conf)) = (best.angle_deg, best.angle_conf) {
                    if conf >= s.align_min_angle_conf {
                        let (ca, sa) = (a_deg.to_radians().cos(), a_deg.to_radians().sin());
                        if let (Some(p0), Some(p1)) = (
                            self.cam.plane_point(pose_end, best_c, z_mid),
                            self.cam.plane_point(pose_end, [best_c[0] + 20.0 * ca, best_c[1] + 20.0 * sa], z_mid),
                        ) {
                            let obs_axis = crate::grasp_select::geometry::unit2([p1[0] - p0[0], p1[1] - p0[1]]);
                            let deg = axis_angle_deg(obs_axis);
                            if deg > 12.0 {
                                return Err(fail(
                                    "TARGET_DISPLACED",
                                    format!("[{phase}] target '{label}' orientation rotated by {deg:.1} deg (limit 12.0 deg)"),
                                ));
                            }
                        }
                    }
                } else if route.has_oriented_evidence && exp_w > 1.0 && exp_h > 1.0 {
                    let exp_elong = exp_w.max(exp_h) / exp_w.min(exp_h);
                    if exp_elong >= 1.4 {
                        let ratio = (meas_w / meas_h) / (exp_w / exp_h);
                        if (ratio - 1.0).abs() > 0.30 {
                            return Err(fail(
                                "TARGET_DISPLACED",
                                format!("[{phase}] target '{label}' box aspect shifted by {:.0}% (limit 30%)", (ratio - 1.0).abs() * 100.0),
                            ));
                        }
                    }
                }
            }

            return Ok(best.clone());
        }
    }

    /// Best-effort safe straight-line TCP withdrawal along `-insertion_dir` back to `pregrasp`
    /// without changing J7 (closed-jaw after `NOT_HOLDING`, open-jaw after early `COLLISION`).
    /// Never moves blindly: rechecks table/workspace/branch from a fresh fenced snapshot and holds
    /// safely in place if withdrawal is unsafe. Always propagates `Stop::Cancelled`.
    async fn adaptive_withdraw(
        &self,
        phase: &str,
        route: &crate::grasp_select::AdaptiveRoute,
        rev: crate::grasp_select::AdaptiveRevisions,
    ) -> Result<(), Stop> {
        if !self.alive().await {
            return Err(Stop::Cancelled);
        }
        let snap = match self.adaptive_snapshot_fenced(phase, rev).await {
            Ok(s) => s,
            Err(Stop::Cancelled) => return Err(Stop::Cancelled),
            Err(e) => {
                tracing::warn!("Adaptive grasp {}: skipping withdrawal ({e:?})", self.id);
                return Ok(());
            }
        };
        let input = match snap.plan_input_in(GoalFrame::Tcp) {
            Ok(i) => i,
            Err(e) => {
                tracing::warn!("Adaptive grasp {}: withdrawal plan_input failed ({e:?}), holding in place", self.id);
                return Ok(());
            }
        };
        let cur_tcp = motion::end_to_tcp(&input.arm.fk(&input.q0), &self.tcp_offset);
        // Fixed-orientation straight TCP translation strictly along `-insertion_dir` from the
        // actual pose (never introducing lateral scrape or wrist rotation near the object).
        let retreat_along = kinematics::dot(
            &kinematics::sub(&cur_tcp.p, &route.plan.pregrasp.p),
            &route.plan.insertion_dir,
        )
        .clamp(0.0, route.plan.insertion_m + 0.01);
        if retreat_along < 1e-3 {
            return Ok(());
        }
        let withdraw_goal = Pose::new(
            cur_tcp.r,
            kinematics::sub(
                &cur_tcp.p,
                &route.plan.insertion_dir.map(|x| x * retreat_along),
            ),
        );
        let q_pre_ref = route.plan.q_pregrasp;
        let speed = Speed { linear: route.insert_mps, angular: motion::DEFAULT_ANGULAR_RPS };
        let env_pts = self.envelope.clone();
        let table_bound_z = self.setup.table_z_m + TABLE_CLEARANCE_M;
        let planned = tokio::task::spawn_blocking(move || {
            let path = motion::plan_cartesian_opts(&input, withdraw_goal, speed, motion::PathOpts::default())?;
            crate::backlash::check_table_clear(
                &input.arm,
                &env_pts,
                table_bound_z,
                path.q.iter().map(|q| q.as_slice()),
            )
            .map_err(|v| v.into_motion_error())?;
            let q_end: [f64; 6] = std::array::from_fn(|i| f64::from(path.last()[i]));
            let jd = crate::grasp_select::feasibility::max_joint_diff(
                &q_end,
                &q_pre_ref,
                input.arm.arm_dof(),
            );
            if jd > crate::grasp_select::feasibility::BRANCH_CONSISTENCY_TOL_RAD {
                let joint = (0..input.arm.arm_dof().min(6))
                    .max_by(|&a, &b| (q_end[a] - q_pre_ref[a]).abs().total_cmp(&(q_end[b] - q_pre_ref[b]).abs()))
                    .unwrap_or(0);
                return Err(MotionError::IkBranchFlip {
                    s: 1.0,
                    joint,
                    dq: jd,
                    yaw_off_deg: None,
                });
            }
            Ok((input, path))
        })
        .await;
        let (input, path) = match planned {
            Ok(Ok(pair)) => pair,
            Ok(Err(e)) => {
                tracing::warn!("Adaptive grasp {}: withdrawal unsafe ({e}), holding safely in place", self.id);
                return Ok(());
            }
            Err(e) => {
                tracing::warn!("Adaptive grasp {}: withdrawal planner join error ({e}), holding in place", self.id);
                return Ok(());
            }
        };
        let remaining_s = self.deadline.saturating_duration_since(Instant::now()).as_secs_f64();
        if path.duration + 0.5 > self.setup.phase_timeout_s || path.duration + 0.5 > remaining_s {
            tracing::warn!("Adaptive grasp {}: withdrawal exceeds remaining budget, holding safely in place", self.id);
            return Ok(());
        }
        match self
            .ctx
            .install_adaptive_path_child(phase, &snap, &input, withdraw_goal, path, rev, None)
            .await
        {
            Ok(_) => Ok(()),
            Err((Stop::Cancelled, _)) => Err(Stop::Cancelled),
            Err((e, _)) => {
                tracing::warn!("Adaptive grasp {}: withdrawal child failed ({e:?}), holding safely in place", self.id);
                Ok(())
            }
        }
    }

    /// Phase B adaptive grasp orchestrator (both `OWNER_LEGACY` and `OWNER_EXECUTOR`, 0
    /// `SupervisorLock` on `OWNER_EXECUTOR`).
    ///
    /// Executes the preflighted [`crate::grasp_select::AdaptiveRoute`]:
    /// 0. `revalidate_start`: fresh post-settle vision check at start pose.
    /// 1. `pregrasp` + `pre_open`: Cartesian or swept-joint verbatim `JointPath` (J7 locked,
    ///    never `yaw_outward`), then J7 intent `pre_open`.
    /// 2. `revalidate` + `insert`: recheck target identity/pose/orientation when visible, verify
    ///    pregrasp pose & IK branch, straight TCP line along `insertion_dir` at fixed orientation
    ///    with optional J2/J3 contact guard. Early contact outside `contact_window_m` triggers
    ///    safe open-jaw withdrawal (or safe hold) and returns `COLLISION`.
    /// 3. `close` -> `separation` -> `lift` -> `lift_confirm`: J7 pos-force close; `NOT_HOLDING`
    ///    triggers closed-jaw withdrawal along `-insertion_dir` (never opens jaws at contact);
    ///    loaded `separation` and `lift` recheck swept carried-object OBB SAT; `lift_confirm`
    ///    settles with live revision fencing and NO automatic regrip retry (`DROPPED`).
    async fn run_adaptive(
        &self,
        planned: &PlannedGrasp,
        pa: &PlannedAdaptive,
    ) -> Result<(bool, Option<f32>), Stop> {
        use crate::grasp_select::feasibility::{
            max_joint_diff, plan_swept_joint_path,
            BRANCH_CONSISTENCY_TOL_RAD, SEPARATION_M,
        };
        use crate::grasp_select::{
            visible_at_pose, PregraspMode, CONTACT_LATERAL_TOL_M, CONTACT_ROT_TOL_RAD,
        };

        let route = &pa.route;
        let rev = pa.revisions;
        let s = &self.setup;
        let table_bound_z = s.table_z_m + TABLE_CLEARANCE_M;
        let z_mid = route.target_table_z + (route.target_height[0] + route.target_height[1]) / 4.0;
        let mut expected_gen = pa.expected_generation.or(self.generation);
        let mut after = planned.request.frame_id;

        // ---- Stage 0: Revalidate target at start pose (with coherent measured start check) ----
        self.phase("revalidate_start", 0.05).await?;
        let snap0 = self.adaptive_snapshot_fenced("revalidate_start", rev).await?;
        for (a, b) in snap0.commanded_positions.iter().zip(pa.start_positions.iter()) {
            if (a - b).abs() > 0.01 {
                return Err(fail(
                    "PLAN_STALE",
                    "[revalidate_start] arm moved away from planned start pose before execution",
                ));
            }
        }
        let (meas0, _) = self.measured_state_fenced("revalidate_start", rev).await?;
        let arm_dof0 = snap0.env.config.arm_dof().min(meas0.len()).min(snap0.commanded_positions.len());
        for i in 0..arm_dof0 {
            if (meas0[i] - snap0.commanded_positions[i]).abs() > 0.05 {
                return Err(fail(
                    "TRACKING_STALL",
                    format!(
                        "[revalidate_start] measured J{} ({:.3} rad) diverges from commanded ({:.3} rad)",
                        i + 1, meas0[i], snap0.commanded_positions[i]
                    ),
                ));
            }
        }
        let input0 = snap0
            .plan_input_in(GoalFrame::Tcp)
            .map_err(crate::macro_action::arm_fail)?;
        let pose0_end = input0.arm.fk(&input0.q0);
        self.fresh_match_adaptive(
            "revalidate_start",
            &planned.request.label,
            route,
            &mut expected_gen,
            &pose0_end,
            Instant::now(),
            &mut after,
        )
        .await?;
        let confirmed_at = Instant::now();

        // ---- Stage 1: Pre-grasp (preserving jaw orientation & IK branch, J7 locked) + Pre-open ----
        self.phase("pregrasp", 0.20).await?;
        let snap_pre = self.adaptive_snapshot_fenced("pregrasp", rev).await?;
        let input_pre = snap_pre
            .plan_input_in(GoalFrame::Tcp)
            .map_err(crate::macro_action::arm_fail)?;
        let pre_goal = route.plan.pregrasp;
        let q_pre_target = route.plan.q_pregrasp;
        let pre_mode = route.pregrasp_mode;
        let pre_speed = Speed { linear: route.approach_mps, angular: motion::DEFAULT_ANGULAR_RPS };
        let env_pts = self.envelope.clone();
        let (input_pre, pre_path) = tokio::task::spawn_blocking(move || {
            let plan_res: Result<motion::JointPath, ArmError> = (|| {
                if pre_mode == PregraspMode::Cartesian {
                    if let Ok(p) = motion::plan_cartesian_opts(
                        &input_pre,
                        pre_goal,
                        pre_speed,
                        motion::PathOpts::default(),
                    ) {
                        let q_end: [f64; 6] = std::array::from_fn(|i| f64::from(p.last()[i]));
                        let table_ok = crate::backlash::check_table_clear(
                            &input_pre.arm,
                            &env_pts,
                            table_bound_z,
                            p.q.iter().map(|q| q.as_slice()),
                        )
                        .is_ok();
                        if table_ok
                            && max_joint_diff(&q_end, &q_pre_target, input_pre.arm.arm_dof())
                                <= BRANCH_CONSISTENCY_TOL_RAD
                        {
                            return Ok(p);
                        }
                    }
                }
                plan_swept_joint_path(&input_pre, q_pre_target, &env_pts, table_bound_z)
                    .map_err(ArmError::from)
            })();
            (input_pre, plan_res)
        })
        .await
        .map_err(|e| fail("INTERNAL", format!("[pregrasp] planner task failed: {e}")))?;
        let pre_path = pre_path.map_err(|e| {
            let s = crate::macro_action::arm_fail(e);
            match s {
                Stop::Fail(c, m) => Stop::Fail(c, format!("[pregrasp] {m}")),
                other => other,
            }
        })?;
        if pre_path.duration + 0.5 > s.phase_timeout_s {
            return Err(fail(
                "GRASP_TIMEOUT",
                format!("[pregrasp] planned duration {:.2} s exceeds phase_timeout_s {:.2} s", pre_path.duration, s.phase_timeout_s),
            ));
        }
        let _ = self
            .ctx
            .install_adaptive_path_child("pregrasp", &snap_pre, &input_pre, pre_goal, pre_path, rev, None)
            .await
            .map_err(|(stop, _)| stop)?;

        // Open jaws at pre-grasp strictly via J7 intent.
        let _ = self
            .ctx
            .run_gripper_fenced(
                "pre_open",
                0.35,
                route.pre_open_command,
                route.pre_open_fraction,
                false,
                rev,
            )
            .await?;
        let pre_open_done_at = Instant::now();

        // ---- Stage 2: Revalidate at pre-grasp (when visible) + Straight TCP insertion ----
        self.phase("revalidate", 0.45).await?;
        let snap_rev = self.adaptive_snapshot_fenced("revalidate", rev).await?;
        let input_rev = snap_rev
            .plan_input_in(GoalFrame::Tcp)
            .map_err(crate::macro_action::arm_fail)?;
        let actual_pre_end = input_rev.arm.fk(&input_rev.q0);
        let actual_pre_tcp = motion::end_to_tcp(&actual_pre_end, &self.tcp_offset);
        let pre_pos_err = kinematics::norm(&kinematics::sub(&actual_pre_tcp.p, &route.plan.pregrasp.p));
        let pre_rot_err = kinematics::angle_between(&actual_pre_tcp.r, &route.plan.pregrasp.r);
        let pre_j_err = max_joint_diff(&input_rev.q0, &route.plan.q_pregrasp, input_rev.arm.arm_dof());
        if pre_pos_err > 0.006
            || pre_rot_err > 2.5f64.to_radians()
            || pre_j_err > BRANCH_CONSISTENCY_TOL_RAD
        {
            return Err(fail(
                "PLAN_STALE",
                format!(
                    "[revalidate] arm drifted from pregrasp pose/branch (pos {pre_pos_err:.4} m, rot {:.2} deg, joint {pre_j_err:.3} rad)",
                    pre_rot_err.to_degrees()
                ),
            ));
        }
        if visible_at_pose(
            &self.cam,
            &self.tcp_offset,
            &route.target_poly,
            z_mid,
            &actual_pre_tcp,
        ) {
            self.fresh_match_adaptive(
                "revalidate",
                &planned.request.label,
                route,
                &mut expected_gen,
                &actual_pre_end,
                pre_open_done_at,
                &mut after,
            )
            .await?;
        } else {
            let max_obscured_s = (route.budget.pregrasp_s + route.budget.pre_open_s + 2.5).min(s.phase_timeout_s);
            if confirmed_at.elapsed().as_secs_f64() > max_obscured_s {
                return Err(fail(
                    "TARGET_LOST",
                    format!(
                        "[revalidate] target confirmation expired ({:.1} s > {:.1} s) while obscured at oblique pregrasp",
                        confirmed_at.elapsed().as_secs_f64(),
                        max_obscured_s
                    ),
                ));
            }
        }

        self.phase("insert", 0.55).await?;
        let snap_ins = self.adaptive_snapshot_fenced("insert", rev).await?;
        let input_ins = snap_ins
            .plan_input_in(GoalFrame::Tcp)
            .map_err(crate::macro_action::arm_fail)?;
        // Fixed-orientation straight TCP translation strictly along `insertion_dir` from the
        // actual pregrasp pose (zero lateral drift or wrist rotation during insertion).
        let actual_ins_start = motion::end_to_tcp(&input_ins.arm.fk(&input_ins.q0), &self.tcp_offset);
        let rem_ins = kinematics::dot(
            &kinematics::sub(&route.plan.grasp.p, &actual_ins_start.p),
            &route.plan.insertion_dir,
        )
        .clamp(0.0, route.plan.insertion_m + 0.006);
        let grasp_goal = Pose::new(
            actual_ins_start.r,
            kinematics::add(&actual_ins_start.p, &route.plan.insertion_dir.map(|x| x * rem_ins)),
        );
        let q_grasp_ref = route.plan.q_grasp;
        let ins_speed = Speed { linear: route.insert_mps, angular: motion::DEFAULT_ANGULAR_RPS };
        let env_pts = self.envelope.clone();
        let (input_ins, ins_path) = tokio::task::spawn_blocking(move || {
            let res: Result<motion::JointPath, ArmError> = (|| {
                let p = motion::plan_cartesian_opts(
                    &input_ins,
                    grasp_goal,
                    ins_speed,
                    motion::PathOpts::default(),
                )?;
                crate::backlash::check_table_clear(
                    &input_ins.arm,
                    &env_pts,
                    table_bound_z,
                    p.q.iter().map(|q| q.as_slice()),
                )
                .map_err(|v| ArmError::from(v.into_motion_error()))?;
                let q_end: [f64; 6] = std::array::from_fn(|i| f64::from(p.last()[i]));
                let jd = max_joint_diff(&q_end, &q_grasp_ref, input_ins.arm.arm_dof());
                if jd > BRANCH_CONSISTENCY_TOL_RAD {
                    let joint = (0..input_ins.arm.arm_dof().min(6))
                        .max_by(|&a, &b| (q_end[a] - q_grasp_ref[a]).abs().total_cmp(&(q_end[b] - q_grasp_ref[b]).abs()))
                        .unwrap_or(0);
                    return Err(ArmError::from(MotionError::IkBranchFlip {
                        s: 1.0,
                        joint,
                        dq: jd,
                        yaw_off_deg: None,
                    }));
                }
                Ok(p)
            })();
            (input_ins, res)
        })
        .await
        .map_err(|e| fail("INTERNAL", format!("[insert] planner task failed: {e}")))?;
        let ins_path = ins_path.map_err(|e| match crate::macro_action::arm_fail(e) {
            Stop::Fail(c, m) => Stop::Fail(c, format!("[insert] {m}")),
            other => other,
        })?;
        if ins_path.duration + 0.5 > s.phase_timeout_s {
            return Err(fail(
                "GRASP_TIMEOUT",
                format!("[insert] planned duration {:.2} s exceeds phase_timeout_s {:.2} s", ins_path.duration, s.phase_timeout_s),
            ));
        }
        let ins_res = self
            .ctx
            .install_adaptive_path_child(
                "insert",
                &snap_ins,
                &input_ins,
                grasp_goal,
                ins_path,
                rev,
                Some(route.contact_guard),
            )
            .await;
        let (_ins_done, watch) = match ins_res {
            Ok(pair) => pair,
            Err((stop, _)) => return Err(stop),
        };
        let contact_h = watch
            .contact_joint
            .map(|_| watch.contact_tcp_z_m.unwrap_or(f64::NAN) - s.table_z_m);
        if self.is_executor() {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::DescendContact {
                peak_residual_nm: watch.peak_residual_nm,
                peak_tracking_rad: watch.peak_tracking_rad,
                contact_height_m: contact_h.map(|h| h as f32),
            });
        } else {
            self.update_action_if_legacy(|a| {
                a.descend_peak_residual_nm = watch.peak_residual_nm;
                a.descend_peak_tracking_rad = watch.peak_tracking_rad;
                if let Some(h) = contact_h {
                    a.descend_contact = Some(true);
                    a.descend_contact_height_m = Some(h as f32);
                }
            })
            .await;
        }

        // Verify post-insertion coherent measured TCP pose against nominal / contact-window
        // envelope (never silently falling back to commanded FK on stale hardware feedback).
        let (_meas_post_ins, actual_contact_tcp) = self.measured_state_fenced("insert", rev).await?;
        let delta = kinematics::sub(&route.plan.grasp.p, &actual_contact_tcp.p);
        let rem_along = kinematics::dot(&delta, &route.plan.insertion_dir);
        let along = route.plan.insertion_dir.map(|x| x * rem_along);
        let perp = kinematics::sub(&delta, &along);
        let lat_err = kinematics::norm(&perp);
        let rot_err = kinematics::angle_between(&actual_contact_tcp.r, &route.plan.grasp.r);
        let contact_triggered = watch.contact_joint.is_some();
        let within_contact_window = rem_along >= -CONTACT_LATERAL_TOL_M
            && rem_along <= route.contact_window_m + 1e-4
            && lat_err <= CONTACT_LATERAL_TOL_M
            && rot_err <= CONTACT_ROT_TOL_RAD;
        let reached_nominal = rem_along.abs() <= CONTACT_LATERAL_TOL_M
            && lat_err <= CONTACT_LATERAL_TOL_M
            && rot_err <= CONTACT_ROT_TOL_RAD;

        if contact_triggered && !within_contact_window {
            // Unexpected early obstruction during open-jaw insertion: safe open-jaw withdrawal.
            self.adaptive_withdraw("withdrawal", route, rev).await?;
            return Err(fail(
                "COLLISION",
                format!(
                    "[insert] early contact {:.3} m before target (window {:.3} m, lat {:.3} m, rot {:.2} deg)",
                    rem_along,
                    route.contact_window_m,
                    lat_err,
                    rot_err.to_degrees()
                ),
            ));
        }
        if !contact_triggered && !reached_nominal {
            return Err(fail(
                "TRACKING_STALL",
                format!(
                    "[insert] insertion did not reach grasp pose (rem {:.3} m, lat {:.3} m, rot {:.2} deg)",
                    rem_along,
                    lat_err,
                    rot_err.to_degrees()
                ),
            ));
        }

        // ---- Stage 3: Close -> Separation -> Lift -> Post-lift holding confirmation ----
        let closed = self
            .ctx
            .run_gripper_fenced("close", 0.70, GripperCommand::Close, None, true, rev)
            .await?;
        if let Some(w) = self.held_width_m().await {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::HeldWidth { held_width_m: w });
            self.update_action_if_legacy(|a| a.held_width_m = Some(w)).await;
        }
        if closed.holding != Some(true) {
            // Empty grasp: NEVER open jaws at contact; withdraw closed along -insertion_dir.
            self.adaptive_withdraw("withdrawal", route, rev).await?;
            return Err(fail(
                "NOT_HOLDING",
                format!(
                    "[close] jaws closed without holding target (gap {:.2} rad)",
                    closed.gripper_gap_rad.unwrap_or(0.0)
                ),
            ));
        }

        // Compute actual conservative carried-object TCP box via full SE(3) relative transform
        // `actual_contact_tcp^{-1} * route.plan.grasp` (covers BOTH translation and rotation error).
        let rt_actual = kinematics::transpose(&actual_contact_tcp.r);
        let rel_r = kinematics::mat_mul(&rt_actual, &route.plan.grasp.r);
        let rel_p = kinematics::mat_vec(
            &rt_actual,
            &kinematics::sub(&route.plan.grasp.p, &actual_contact_tcp.p),
        );
        let rel_pose = Pose::new(rel_r, rel_p);
        let obj_box = route.obj_box_nominal_tcp.transformed_by_relative_pose(&rel_pose);

        // Separation (+SEPARATION_M straight up, fixed orientation, single converged loaded-leg validator)
        self.phase("separation", 0.80).await?;
        let snap_sep = self.adaptive_snapshot_fenced("separation", rev).await?;
        let input_sep = snap_sep
            .plan_input_in(GoalFrame::Tcp)
            .map_err(crate::macro_action::arm_fail)?;
        let cur_sep_tcp = motion::end_to_tcp(&input_sep.arm.fk(&input_sep.q0), &self.tcp_offset);
        let sep_goal = Pose::new(
            actual_contact_tcp.r,
            [
                cur_sep_tcp.p[0],
                cur_sep_tcp.p[1],
                (cur_sep_tcp.p[2] + SEPARATION_M).max(route.plan.separation.p[2]),
            ],
        );
        let loaded_speed = Speed { linear: route.loaded_mps, angular: motion::DEFAULT_ANGULAR_RPS };
        let env_pts = self.envelope.clone();
        let table_z_m = s.table_z_m;
        let phase_timeout_s = s.phase_timeout_s;
        let payload_kg = route.payload_kg;
        let torque_limits = route.torque_limits.clone();
        let has_gravity_model = route.has_gravity_model;
        let (input_sep, sep_path) = tokio::task::spawn_blocking(move || {
            let res: Result<motion::JointPath, ArmError> = (|| {
                let p = motion::plan_cartesian_opts(
                    &input_sep,
                    sep_goal,
                    loaded_speed,
                    motion::PathOpts::default(),
                )?;
                let leg_params = crate::grasp_select::feasibility::LoadedLegParams {
                    arm: &input_sep.arm,
                    tcp_offset: &input_sep.tcp_offset,
                    ws: input_sep.ws.as_ref(),
                    envelope: &env_pts,
                    table_z_m,
                    phase_timeout_s,
                    obj_box: &obj_box,
                    payload_kg,
                    torque_limits: torque_limits.as_deref(),
                    has_gravity_model,
                    fail_on_insufficient_evidence: true,
                };
                crate::grasp_select::feasibility::validate_loaded_leg(&leg_params, &p, "separation")
                    .map_err(|r| ArmError::Rejected {
                        status: 422,
                        code: r.code,
                        message: r.message,
                    })?;
                Ok(p)
            })();
            (input_sep, res)
        })
        .await
        .map_err(|e| fail("INTERNAL", format!("[separation] planner task failed: {e}")))?;
        let sep_path = sep_path.map_err(|e| match crate::macro_action::arm_fail(e) {
            Stop::Fail(c, m) => Stop::Fail(c, format!("[separation] {m}")),
            other => other,
        })?;
        let _ = self
            .ctx
            .install_adaptive_path_child("separation", &snap_sep, &input_sep, sep_goal, sep_path, rev, None)
            .await
            .map_err(|(stop, _)| stop)?;

        // Lift (to lift_z, fixed orientation, single converged loaded-leg validator)
        self.phase("lift", 0.88).await?;
        let snap_lift = self.adaptive_snapshot_fenced("lift", rev).await?;
        let input_lift = snap_lift
            .plan_input_in(GoalFrame::Tcp)
            .map_err(crate::macro_action::arm_fail)?;
        let cur_lift_tcp = motion::end_to_tcp(&input_lift.arm.fk(&input_lift.q0), &self.tcp_offset);
        let lift_z = route.plan.lift.p[2].max(cur_lift_tcp.p[2]);
        if lift_z > cur_lift_tcp.p[2] + 1e-4 {
            let lift_goal = Pose::new(actual_contact_tcp.r, [cur_lift_tcp.p[0], cur_lift_tcp.p[1], lift_z]);
            let q_lift_ref = route.plan.q_lift;
            let env_pts = self.envelope.clone();
            let torque_limits = route.torque_limits.clone();
            let (input_lift, lift_path) = tokio::task::spawn_blocking(move || {
                let res: Result<motion::JointPath, ArmError> = (|| {
                    let p = motion::plan_cartesian_opts(
                        &input_lift,
                        lift_goal,
                        loaded_speed,
                        motion::PathOpts::default(),
                    )?;
                    let leg_params = crate::grasp_select::feasibility::LoadedLegParams {
                        arm: &input_lift.arm,
                        tcp_offset: &input_lift.tcp_offset,
                        ws: input_lift.ws.as_ref(),
                        envelope: &env_pts,
                        table_z_m,
                        phase_timeout_s,
                        obj_box: &obj_box,
                        payload_kg,
                        torque_limits: torque_limits.as_deref(),
                        has_gravity_model,
                        fail_on_insufficient_evidence: true,
                    };
                    crate::grasp_select::feasibility::validate_loaded_leg(&leg_params, &p, "lift")
                        .map_err(|r| ArmError::Rejected {
                            status: 422,
                            code: r.code,
                            message: r.message,
                        })?;
                    let q_end: [f64; 6] = std::array::from_fn(|i| f64::from(p.last()[i]));
                    let jd = max_joint_diff(&q_end, &q_lift_ref, input_lift.arm.arm_dof());
                    if jd > BRANCH_CONSISTENCY_TOL_RAD {
                        let joint = (0..input_lift.arm.arm_dof().min(6))
                            .max_by(|&a, &b| (q_end[a] - q_lift_ref[a]).abs().total_cmp(&(q_end[b] - q_lift_ref[b]).abs()))
                            .unwrap_or(0);
                        return Err(ArmError::from(MotionError::IkBranchFlip {
                            s: 1.0,
                            joint,
                            dq: jd,
                            yaw_off_deg: None,
                        }));
                    }
                    Ok(p)
                })();
                (input_lift, res)
            })
            .await
            .map_err(|e| fail("INTERNAL", format!("[lift] planner task failed: {e}")))?;
            let lift_path = lift_path.map_err(|e| match crate::macro_action::arm_fail(e) {
                Stop::Fail(c, m) => Stop::Fail(c, format!("[lift] {m}")),
                other => other,
            })?;
            let _ = self
                .ctx
                .install_adaptive_path_child("lift", &snap_lift, &input_lift, lift_goal, lift_path, rev, None)
                .await
                .map_err(|(stop, _)| stop)?;
        }

        // Post-lift holding confirmation with live 5-revision fencing and NO automatic retry.
        self.phase("lift_confirm", 0.95).await?;
        let (held, gap) = self.holding_now_fenced(Some(rev)).await?;
        if held {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PostLift {
                kind: crate::rt_core::action_diag::PostLiftKind::Held,
                gripper_gap_rad: None,
            });
            self.update_action_if_legacy(|a| a.post_lift = Some("held".into())).await;
            return Ok((true, closed.gripper_gap_rad.or(Some(gap))));
        }
        self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PostLift {
            kind: crate::rt_core::action_diag::PostLiftKind::Dropped,
            gripper_gap_rad: Some(gap),
        });
        self.update_action_if_legacy(|a| {
            a.post_lift = Some("dropped".into());
            a.gripper_gap_rad = Some(gap);
        })
        .await;
        Err(fail(
            "DROPPED",
            format!("[lift_confirm] object no longer held after lift (gap {gap:.2} rad)"),
        ))
    }

    async fn run(&self, planned: &PlannedGrasp) -> Result<(bool, Option<f32>), Stop> {
        let s = &self.setup;
        let open = self.gripper_phase("open", 0.02, GripperCommand::Open).await?;
        let _ = open;

        // coarse approach: tool axis down, or (camera-down) optical axis down with the camera
        // centre above the estimate
        let camera_down = planned.camera_down;
        let pose0 = self.pose().await;
        let tcp0 = self.tcp_pose().await;
        // depth capture runs when depth is a height source or recorded (`height_record`), or
        // decides the yaw (`yaw_sources`); the depth yaw / width record and the depth width
        // source use it when it ran
        let depth = self.triggered && self.depth_available && self.lens_known
            && (s.height_sources.contains(&HeightSource::Depth) || s.height_record.contains(&HeightSource::Depth)
                || s.yaw_sources.contains(&YawSource::Depth));
        let depth_cache = if depth { self.depth_start_frame(planned, &pose0).await } else { DepthCache::default() };

        self.phase("approach", 0.1).await?;
        let target = planned.target;
        let is_5dof = self.arm_dof().await == 5;
        let r_ref_p = if camera_down { &pose0.p } else { &tcp0.p };
        let r_ref = if is_5dof { motion::apply_auto_yaw(r_ref_p, &target, pose0.r) } else { pose0.r };
        let goal = if camera_down { camera_down_pose(&r_ref, &target, s, &self.cam, &self.envelope) } else { approach_pose(&r_ref, &target, s) };
        if camera_down {
            let height = goal.p[2] - s.table_z_m;
            if height > s.approach_height_m + 1e-6 {
                tracing::info!("Grasp {}: camera-down acquisition at {height:.3} m (approach_height_m {:.3}) for table clearance", self.id, s.approach_height_m);
                if self.is_executor() {
                    self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::AcquireHeight {
                        height_m: height as f32,
                    });
                } else {
                    self.sup.inner().write().await.set_action_acquire_height(self.id, height as f32);
                }
            }
        }
        let approach_frame = if camera_down { GoalFrame::EndLink } else { GoalFrame::Tcp };
        let done = self.move_arm_in(approach_frame, "approach", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;

        // closed-loop centring onto the grip pixel, or (camera-down) the image centre
        self.phase("centring", 0.3).await?;
        self.set_mask_focus(Some(planned.request.label.clone()), planned.request.box_norm, 2000).await;
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let (aim_px, aim_name) = if camera_down { ([self.cam.cx, self.cam.cy], "the image centre") } else { (grip_px, "the grip point") };
        let plane = grasp_plane(s);
        let pose = self.pose().await;
        let predicted = self.cam.project(&pose, &target).map(|p| self.cam.to_norm(p)).unwrap_or(s.grip_pixel_norm);
        let mut tracker = Tracker { label: planned.request.label.clone(), predicted: planned.request.box_norm, gate_px: s.acquire_dist_px };
        tracker.predict_at(predicted);
        let mut after = None;
        let mut c = self.centre("centring", 0.3, &mut tracker, aim_px, aim_name, plane, done.at, &mut after, s.max_iterations, true).await?;
        // a box cut by the image border at the aim (look-down v5: the point under the jaws is
        // near the bottom edge) has its centre off the target's: centre the box uncut at an aim
        // inside the image; the later steps use those frames, and the `shift` phase moves the
        // target onto the aim
        let mut centred_at = aim_px;
        for attempt in 0..BORDER_SHIFTS {
            let Some(inner) = border_free_aim(&c.frames, centred_at, self.cam.size, s.centre_tol_px + 4.0, attempt) else { break };
            tracing::info!("Grasp {}: box cut by the image border at {aim_name}, centring at ({:.0}, {:.0}) px", self.id, inner[0], inner[1]);
            c = self.centre("centring", 0.3, &mut tracker, inner, "the border-free aim", plane, Instant::now(), &mut after, s.max_iterations, true).await?;
            centred_at = inner;
        }
        let shifted = centred_at != aim_px;
        let cut = c.frames.iter().any(|d| touches_border(&d.box_norm, self.cam.size));
        if shifted || cut {
            let shift_px = shifted.then(|| [(centred_at[0] - aim_px[0]) as f32, (centred_at[1] - aim_px[1]) as f32]);
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::Border {
                kind: if cut {
                    crate::rt_core::action_diag::BorderKind::Cut
                } else {
                    crate::rt_core::action_diag::BorderKind::Shifted
                },
                shift_px,
            });
            self.update_action_if_legacy(|a| {
                a.border = Some(if cut { "cut" } else { "shifted" }.into());
                a.border_shift_px = shift_px;
            }).await;
        }
        let last = c.last.clone();

        // the overhead depth trigger replaces `after` and `c.last` / `c.frames` with its frame
        // whenever it runs, also when depth is record-only (only the height selection ignores it)
        let (depth_meas, depth_why) = if depth {
            match self.depth_overhead(&planned.request.label, &mut c, &mut after, &pose0, depth_cache).await {
                Ok(m) => (Some(m), "no_measurement"),
                Err(why) => (None, why),
            }
        } else {
            (None, "unavailable")
        };
        let depth_res = depth_meas.as_ref().ok_or(depth_why);

        // object height: record-only sources first (they do not move the arm), then the
        // `height_sources` in order until one accepts; `grasp_height_m` when none does. Box
        // centres map onto the object's top plane, the jaws close part way up the object
        let label = &planned.request.label;
        let record: Vec<HeightSource> = s.height_record.iter().copied().filter(|x| !s.height_sources.contains(x)).collect();
        let mut chain = EstimateChain::new();
        for src in &record {
            let (name, value, r) = if *src == HeightSource::Depth { self.depth_source(depth, depth_res) } else { self.prior_source(label) };
            chain.offer(&name, value, r, true);
        }
        self.write_estimates(&chain).await;
        let mut reasons: Vec<String> = Vec::new();
        for src in &s.height_sources {
            if chain.done() { break; }
            // a prior does not override a measured too-tall result
            if *src == HeightSource::Prior && chain.vetoed() { continue; }
            let (name, value, r) = match src {
                HeightSource::Depth => self.depth_source(depth, depth_res),
                HeightSource::Prior => self.prior_source(label),
                // only a too-tall measurement is a result; stop / fault / other failures end the grasp
                HeightSource::Probe => match self.probe_height(&c, label, &mut after).await {
                    Ok(Ok((h, source))) => (source.to_string(), None, SourceResult::Accept(h)),
                    Ok(Err(reason)) => ("probe".to_string(), None, SourceResult::Reject(reason)),
                    Err(Stop::Fail(code, msg)) if code == "OBJECT_TOO_TALL" => ("probe".to_string(), None, SourceResult::Veto(Stop::Fail(code, msg))),
                    Err(stop) => return Err(stop),
                },
            };
            // `height_reason` keeps the reasons of the non-depth sources (depth's are in `height_estimates`)
            if let (SourceResult::Reject(reason), false) = (&r, *src == HeightSource::Depth) { reasons.push(reason.clone()); }
            chain.offer(&name, value, r, false);
            self.write_estimates(&chain).await;
        }
        let chosen = match chain.finish() { Ok((chosen, _)) => chosen, Err((stop, _)) => return Err(stop) };

        let mut close_h = s.grasp_height_m;
        let mut plane = plane;
        let depth_chosen = chosen.as_ref().is_some_and(|(_, source)| source.starts_with("depth"));
        match chosen {
            Some((h, source)) => {
                plane = s.table_z_m + s.grasp_height_m.max(h);
                close_h = (h * s.height_grasp_fraction).clamp(s.grasp_height_m, s.max_grasp_height_m);
                tracing::info!("Grasp {}: object top {h:.3} m above the table ({source}), closing at {close_h:.3} m", self.id);
                let src_label = if depth_chosen { "depth".to_string() } else { source };
                if self.is_executor() {
                    self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::HeightDecision {
                        source: crate::rt_core::action_diag::ShortLabel::new_truncated(&src_label),
                        object_height_m: Some(h as f32),
                        reason: None,
                    });
                } else {
                    self.update_action_if_legacy(|a| {
                        a.height_source = Some(src_label);
                        a.object_height_m = Some(h as f32);
                    }).await;
                }
            }
            None => {
                let joined = (!reasons.is_empty()).then(|| reasons.join(","));
                if self.is_executor() {
                    let reason = joined
                        .as_deref()
                        .map(|r| self.sup.hub().intern_action_diag_str(self.id, r));
                    self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::HeightDecision {
                        source: crate::rt_core::action_diag::ShortLabel::new_truncated("setup"),
                        object_height_m: None,
                        reason,
                    });
                } else {
                    self.update_action_if_legacy(|a| {
                        a.height_source = Some("setup".into());
                        a.height_reason = joined;
                    }).await;
                }
            }
        }
        // depth measurement fields: when depth decided or is record-only (diagnostic)
        if let Some(meas) = depth_meas.as_ref().filter(|_| depth_chosen || record.contains(&HeightSource::Depth)) {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::DepthMeasurement {
                anchor: crate::rt_core::action_diag::ShortLabel::new_truncated(meas.source),
                object_depth_m: meas.depth_m as f32,
                object_length_m: meas.length_m.map(|l| l as f32),
                jaw_clear: meas.jaw_clear,
            });
            self.update_action_if_legacy(|a| {
                a.depth_anchor = Some(meas.source.into());
                a.object_depth_m = Some(meas.depth_m as f32);
                a.object_length_m = meas.length_m.map(|l| l as f32);
                a.jaw_clear = meas.jaw_clear;
            }).await;
        }
        self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::CloseHeight {
            close_height_m: close_h as f32,
        });
        self.update_action_if_legacy(|a| a.close_height_m = Some(close_h as f32)).await;

        // look-down, centred at a border-free aim: one base move that puts the TCP over the box
        // centre's point on the plane z = `plane`, seen from the centring pose. The point under
        // the TCP on that plane is the grip pixel (the table point under the jaws) moved by the
        // model's parallax between the table and the plane (camera-down: the transition starts
        // from the box centre's ray anyway)
        if shifted && !camera_down {
            self.phase("shift", 0.5).await?;
            let mut xs: Vec<f64> = c.frames.iter().map(|d| self.cam.to_px(box_centre(&d.box_norm))[0]).collect();
            let mut ys: Vec<f64> = c.frames.iter().map(|d| self.cam.to_px(box_centre(&d.box_norm))[1]).collect();
            let centre = [median(&mut xs), median(&mut ys)];
            let tcp_c = motion::end_to_tcp(&c.pose, &self.tcp_offset);
            let under = |z: f64| self.cam.project(&c.pose, &[tcp_c.p[0], tcp_c.p[1], z]);
            let aim = match (under(plane), under(s.table_z_m)) {
                (Some(m), Some(m0)) => [aim_px[0] + m[0] - m0[0], aim_px[1] + m[1] - m0[1]],
                _ => aim_px,
            };
            let (Some(pt), Some(pg)) = (self.cam.plane_point(&c.pose, centre, plane), self.cam.plane_point(&c.pose, aim, plane)) else {
                return Err(fail("CENTRING_FAILED", "Border shift cannot be mapped onto the table plane"));
            };
            let d = [pt[0] - pg[0], pt[1] - pg[1], 0.0];
            self.move_arm_in(GoalFrame::Tcp, "shift", GoalSpec::Relative { frame: RelFrame::Base, d, dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None }, s.approach_speed_mps).await?;
        }

        // camera-down: re-orient to the tool-vertical pose above the target's table point
        if camera_down {
            self.phase("transition", 0.5).await?;
            // the ray of the last centring box from the pose it was seen at (before the probe)
            let Some((t, goal)) = transition_pose(&c.pose, self.cam.to_px(box_centre(&last.box_norm)), plane, s, &self.cam) else {
                return Err(fail("CENTRING_FAILED", "The target ray does not meet the table plane"));
            };
            let done = self.move_arm_in(GoalFrame::Tcp, "transition", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await?;
            // verify the target where the model expects it before descending
            self.recheck(&planned.request.label, &last, &t, grip_px, done.at, &mut after).await?;
        }

        // J6 alignment: close the jaws across the object's short side
        self.phase("align", 0.55).await?;
        let (turn, align) = if s.align_j6 { self.align_turn(&c, plane).await } else { (None, "off".to_string()) };
        self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::Align {
            align: crate::rt_core::action_diag::ShortLabel::new_truncated(&align),
            turn_rad: turn.map(|t| t as f32),
        });
        self.update_action_if_legacy(|a| { a.align = Some(align.clone()); a.align_turn_rad = turn.map(|t| t as f32); }).await;

        // grasp width: `width_record` sources first (reported only), then `width_sources` in
        // order until one accepts; one limit check on the chosen width. The width is measured
        // whatever the angle confidence: a disc or a square has a low angle_conf but a valid
        // width (any closing direction sees about the same width)
        let width_record: Vec<WidthSource> = s.width_record.iter().copied().filter(|x| !s.width_sources.contains(x)).collect();
        let mut wchain = EstimateChain::new();
        let width_of = |src: WidthSource| -> SourceResult {
            let r = match src {
                WidthSource::Mask => self.mask_width_m(&c, plane),
                WidthSource::Depth => match depth_res {
                    Ok(m) => m.width_m.ok_or("no_width"),
                    Err(why) => Err(if depth { why } else { "unavailable" }),
                },
            };
            match r { Ok(w) => SourceResult::Accept(w), Err(why) => SourceResult::Reject(why.into()) }
        };
        for src in &width_record {
            wchain.offer(src.name(), None, width_of(*src), true);
        }
        for src in &s.width_sources {
            if wchain.done() { break; }
            wchain.offer(src.name(), None, width_of(*src), false);
        }
        let (chosen_width, width_entries) = match wchain.finish() { Ok(r) => r, Err((_, e)) => (None, e) };
        let measured_width_m = chosen_width.map(|(w, _)| w);
        self.emit_estimates(
            crate::rt_core::action_diag::EstimateListKind::Width,
            &width_entries,
        );
        if let Some(w) = measured_width_m {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::GraspWidth {
                width_m: w as f32,
            });
        }
        self.update_action_if_legacy(|a| {
            a.width_estimates = width_entries;
            if let Some(w) = measured_width_m { a.grasp_width_m = Some(w as f32); }
        }).await;
        if let Some(w) = measured_width_m {
            let max_w = self.max_grasp_width().await;
            if max_w > 0.0 && w > max_w {
                return Err(fail("TOO_WIDE", format!("Object width {w:.3} m exceeds max_grasp_width_m {:.3} m", max_w)));
            }
        }

        if let Some(turn) = turn {
            self.move_arm_in(GoalFrame::Tcp, "align", GoalSpec::Relative { frame: RelFrame::Tool, d: [0.0; 3], dyaw: 0.0, dpitch: 0.0, droll: turn, level: false, auto_yaw: None }, s.approach_speed_mps).await?;
        }

        // width-based pre-open before descend
        let (pre_open_decision, pre_open_target) = {
            let policy = self.gripper_policy().await;
            match policy {
                None => (Some("skipped:no_policy".to_string()), None),
                Some(policy) => {
                    let calibrated = policy.is_calibrated();
                    if s.pre_open == PreOpen::Off {
                        (Some("off".to_string()), None)
                    } else if s.pre_open == PreOpen::Auto && !calibrated {
                        (Some("skipped:uncalibrated".to_string()), None)
                    } else if !matches!(align.as_str(), "turned:angle" | "kept:angle")
                        && frames_angle(&c.frames, s.align_min_angle_conf).is_some()
                    {
                        // an elongated mask (confident angle): its narrowest cross-section is the
                        // width across the jaws only once J6 is aligned to it; a round or square
                        // mask (no confident angle) has about the same width in every direction
                        (Some("skipped:not_aligned".to_string()), None)
                    } else if let Err(reason) = align_guard(&c.last, &c.frames, self.cam.size) {
                        (Some(format!("skipped:{reason}")), None)
                    } else if let Some(w) = measured_width_m {
                        let target_w = (w + s.pre_open_margin_m) as f32;
                        let target_j7 = policy.j7_for(target_w);
                        let full_open_w = policy.opening_m(policy.open_rad);
                        let at_or_beyond_open = target_w >= full_open_w
                            || (target_j7 - policy.open_rad).abs() < 1e-4
                            || if policy.open_rad > policy.closed_rad {
                                target_j7 >= policy.open_rad
                            } else {
                                target_j7 <= policy.open_rad
                            };
                        if at_or_beyond_open {
                            (Some("skipped:open_rad".to_string()), None)
                        } else {
                            let min_j7 = policy.closed_rad.min(policy.open_rad);
                            let max_j7 = policy.closed_rad.max(policy.open_rad);
                            let clamped_j7 = target_j7.clamp(min_j7, max_j7);
                            let span = policy.open_rad - policy.closed_rad;
                            let frac = if span.abs() < 1e-6 { 0.0 } else { ((clamped_j7 - policy.closed_rad) / span).clamp(0.0, 1.0) as f64 };
                            (None, Some((frac, target_w)))
                        }
                    } else {
                        (Some("skipped:no_width".to_string()), None)
                    }
                }
            }
        };

        if let Some((fraction, target_w)) = pre_open_target {
            self.phase("preopen", 0.6).await?;
            self.gripper_move("preopen", fraction).await?;
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PreOpen {
                decision: crate::rt_core::action_diag::DiagStr::Inline(
                    crate::rt_core::action_diag::ShortLabel::new_truncated("done"),
                ),
                pre_open_m: Some(target_w),
            });
            self.update_action_if_legacy(|a| {
                a.pre_open = Some("done".into());
                a.pre_open_m = Some(target_w);
            }).await;
        } else if let Some(reason) = pre_open_decision {
            if self.is_executor() {
                let d = self.sup.hub().intern_action_diag_str(self.id, &reason);
                self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PreOpen {
                    decision: d,
                    pre_open_m: None,
                });
            } else {
                self.update_action_if_legacy(|a| {
                    a.pre_open = Some(reason);
                }).await;
            }
        }

        // descend (look-down: re-centre at mid height on the way), close, check
        self.phase("descend", 0.65).await?;
        if !camera_down {
            let (recentre, iterations) = self.recentre(&c, plane, close_h, &planned.request.label, &mut after).await?;
            if self.is_executor() {
                let status = self.sup.hub().intern_action_diag_str(self.id, &recentre);
                self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::Recentre {
                    status,
                    iterations,
                });
            } else {
                self.update_action_if_legacy(|a| { a.recentre = Some(recentre); a.recentre_iterations = iterations; }).await;
            }
            self.phase("descend", 0.75).await?;
        }
        self.descend(close_h, planned.request.approach).await?;
        let closed = self.gripper_phase("close", 0.8, GripperCommand::Close).await?;
        if let Some(w) = self.held_width_m().await {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::HeldWidth { held_width_m: w });
            self.update_action_if_legacy(|a| a.held_width_m = Some(w)).await;
        }
        let holding = closed.holding == Some(true);
        let lift_to = |p: Vec3| GoalSpec::Absolute { p: [p[0], p[1], s.table_z_m + s.lift_height_m], r: None };
        if !holding {
            self.gripper_phase("release", 0.85, GripperCommand::Open).await?;
            self.phase("lift", 0.9).await?;
            let p = self.tcp_pose().await.p;
            self.move_arm_in(GoalFrame::Tcp, "lift", lift_to(p), s.descend_speed_mps).await?;
            return Err(fail("NOT_HOLDING", format!("Jaws closed without holding the object (gap {:.2} rad)", closed.gripper_gap_rad.unwrap_or(0.0))));
        }
        self.phase("lift", 0.9).await?;
        let p = self.tcp_pose().await.p;
        self.move_arm_in(GoalFrame::Tcp, "lift", lift_to(p), s.descend_speed_mps).await?;
        let r_lift = self.pose().await.r;
        let tool_z = kinematics::column(&r_lift, 2)[2];
        let tilt = (-tool_z).clamp(-1.0, 1.0).acos().to_degrees();
        tracing::info!("Grasp {}: lift tilt {:.2}°", self.id, tilt);
        // the object may slip out while lifting. Under position control the jaw settles on its
        // hold target and the grip torque fades when the object only shifted or turned in the
        // jaws, so a failed torque rule is decided by a re-grip: a close with grasp detection
        // stops on the object (holding) or runs to closed (dropped)
        let (held, gap) = self.holding_now().await?;
        if held {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PostLift {
                kind: crate::rt_core::action_diag::PostLiftKind::Held,
                gripper_gap_rad: None,
            });
            self.update_action_if_legacy(|a| a.post_lift = Some("held".into())).await;
            return Ok((true, closed.gripper_gap_rad));
        }
        tracing::info!("Grasp {}: grip torque gone after the lift (gap {gap:.2} rad), re-gripping", self.id);
        let regrip = self.gripper_phase("regrip", 0.95, GripperCommand::Close).await?;
        let gap = regrip.gripper_gap_rad.unwrap_or(gap);
        if regrip.holding == Some(true) {
            if let Some(w) = self.held_width_m().await {
                self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::HeldWidth { held_width_m: w });
                self.update_action_if_legacy(|a| a.held_width_m = Some(w)).await;
            }
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PostLift {
                kind: crate::rt_core::action_diag::PostLiftKind::Regripped,
                gripper_gap_rad: None,
            });
            self.update_action_if_legacy(|a| a.post_lift = Some("regripped".into())).await;
            return Ok((true, Some(gap)));
        }
        self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::PostLift {
            kind: crate::rt_core::action_diag::PostLiftKind::Dropped,
            gripper_gap_rad: Some(gap),
        });
        self.update_action_if_legacy(|a| { a.post_lift = Some("dropped".into()); a.gripper_gap_rad = Some(gap); }).await;
        Err(fail("DROPPED", format!("Object no longer held after the lift: the re-grip closed to {gap:.2} rad from closed")))
    }

    /// `holding` after `post_lift_settle_ms` from the measured gripper joint, with the gripper policy's rule
    /// (matching GripperMotion in motion.rs). Returns (holding, gap).
    async fn holding_now(&self) -> Result<(bool, f32), Stop> {
        self.holding_now_fenced(None).await
    }

    /// Revision-fenced variant of [`Self::holding_now`]: when `fence` is `Some(rev)`, verifies
    /// all five adaptive revisions on every settle poll iteration on both Legacy and RT Executor
    /// owners (0 `SupervisorLock` on `OWNER_EXECUTOR`).
    async fn holding_now_fenced(
        &self,
        fence: Option<crate::grasp_select::AdaptiveRevisions>,
    ) -> Result<(bool, f32), Stop> {
        let confirm_start = Instant::now();
        let until = confirm_start + Duration::from_millis(self.setup.post_lift_settle_ms);
        let strict_adaptive = fence.is_some();
        let mut taus = Vec::new();
        let mut loads = Vec::new();
        let mut vels = Vec::new();
        let mut last_j7_ts: Option<Instant> = None;
        let mut last_j7_seq: Option<u64> = None;
        let mut last_fresh_gap: Option<f32> = None;
        loop {
            self.check().await?;
            let (gap, tau, load, vel, policy, feedback_fresh, torque_feedback, load_feedback, driver_present, sample_ts, sample_seq) = if self.is_executor() {
                let Some(env) = self.sup.hub().prep_env() else {
                    return Err(fail("OBSERVATION_UNAVAILABLE", "No published planning environment"));
                };
                if let Some(rev) = fence {
                    Self::check_adaptive_revisions("lift_confirm", &env, rev)?;
                }
                let Some(snap) = self.sup.hub().latest() else {
                    return Err(fail("OBSERVATION_UNAVAILABLE", "No published telemetry snapshot"));
                };
                let Some(policy) = env.config.gripper_policy.clone() else {
                    return Err(fail("UNSUPPORTED", "No gripper policy"));
                };
                let g_idx = env.config.gripper_index().unwrap_or(6);
                let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                let m_g = snap.rt.positions.get(g_idx).copied().unwrap_or(f32::NAN);
                let gap = (policy.closed_rad - m_g) * dir;
                let tau = snap.obs.torques.get(g_idx).copied().unwrap_or(0.0).abs();
                let load = snap
                    .obs
                    .loads
                    .as_ref()
                    .and_then(|l| l.get(g_idx).copied())
                    .unwrap_or(0.0)
                    .abs();
                let vel = snap.rt.velocities.get(g_idx).copied().unwrap_or(0.0);
                let j7_ts = snap.rt.feedback_instants.get(g_idx).copied().flatten();
                let j7_seq = if env.driver_present {
                    snap.rt.feedback_seqs.get(g_idx).copied()
                } else {
                    Some(snap.meta.seq)
                };
                let fresh = if env.driver_present {
                    let now = Instant::now();
                    !snap.rt.feedback_instants.is_empty()
                        && snap.rt.feedback_instants.iter().all(|t| {
                            t.is_some_and(|ts| now.saturating_duration_since(ts) <= Duration::from_millis(250))
                        })
                } else {
                    true
                };
                (gap, tau, load, vel, policy, fresh, env.has_torque_feedback, env.has_load_feedback, env.driver_present, j7_ts, j7_seq)
            } else {
                let inner = self.sup.inner().read().await;
                if let Some(rev) = fence {
                    if inner.calibration_revision != rev.calibration_revision {
                        return Err(fail(
                            "CALIBRATION_CHANGED",
                            "[lift_confirm] calibration changed during adaptive grasp",
                        ));
                    }
                    if inner.workspace_revision != rev.workspace_revision
                        || inner.grasp_setup_revision != rev.grasp_setup_revision
                        || inner.gripper_calib_revision != rev.gripper_calib_revision
                        || inner.power_budget_revision != rev.power_budget_revision
                    {
                        return Err(fail(
                            "PLAN_STALE",
                            "[lift_confirm] configuration changed during adaptive grasp",
                        ));
                    }
                }
                let Some(policy) = inner.config.gripper_policy.clone() else {
                    return Err(fail("UNSUPPORTED", "No gripper policy"));
                };
                let g_idx = inner.config.gripper_index().unwrap_or(6);
                let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
                let m_g = inner.measured_positions.get(g_idx).copied().unwrap_or(f32::NAN);
                let gap = (policy.closed_rad - m_g) * dir;
                let tau = inner.torques.get(g_idx).copied().unwrap_or(0.0).abs();
                let load = inner.loads.get(g_idx).copied().unwrap_or(0.0).abs();
                let vel = inner.measured_velocities.get(g_idx).copied().unwrap_or(0.0);
                let j7_ts = inner.core.feedback().instants().get(g_idx).copied().flatten();
                let j7_seq = inner.core.feedback().seqs().get(g_idx).copied();

                let (torque_fb, load_fb) = if let Some(ref d) = inner.driver {
                    d.with_ref(|driver| {
                        let caps = driver.capabilities();
                        (caps.torque_feedback, caps.load_feedback)
                    })
                    .unwrap_or((false, false))
                } else {
                    (true, !inner.loads.is_empty())
                };
                let driver_present = inner.driver.is_some();
                let fresh = if driver_present {
                    inner.hardware_feedback_fresh(Duration::from_millis(250))
                } else {
                    true
                };

                (gap, tau, load, vel, policy, fresh, torque_fb, load_fb, driver_present, j7_ts, j7_seq)
            };
            let accept_sample = if strict_adaptive {
                if !feedback_fresh || !gap.is_finite() {
                    false
                } else if driver_present {
                    // Require fresh post-lift J7 observation distinct from previously recorded sample.
                    let post_lift = sample_ts.is_some_and(|ts| ts >= confirm_start);
                    let distinct_ts = sample_ts.is_some() && sample_ts != last_j7_ts;
                    let distinct_seq = sample_seq.is_some() && sample_seq != last_j7_seq;
                    post_lift && (distinct_ts || distinct_seq)
                } else {
                    sample_seq.is_none() || sample_seq != last_j7_seq
                }
            } else {
                true
            };
            if accept_sample {
                taus.push(f64::from(tau));
                loads.push(f64::from(load));
                vels.push(f64::from(vel));
                if sample_ts.is_some() {
                    last_j7_ts = sample_ts;
                }
                if sample_seq.is_some() {
                    last_j7_seq = sample_seq;
                }
                last_fresh_gap = Some(gap);
            }

            if Instant::now() >= until {
                let tau_median = if taus.is_empty() { 0.0 } else { median(&mut taus) };
                let load_median = if loads.is_empty() { 0.0 } else { median(&mut loads) };
                let vel_median = if vels.is_empty() { 0.0 } else { median(&mut vels) };
                let current_gap = if strict_adaptive {
                    if feedback_fresh && gap.is_finite() {
                        gap
                    } else {
                        last_fresh_gap.unwrap_or(f32::NAN)
                    }
                } else {
                    gap
                };
                let adaptive_fresh_ok = !strict_adaptive
                    || (feedback_fresh && !taus.is_empty() && current_gap.is_finite());

                let holding = adaptive_fresh_ok
                    && match policy.holding_mode {
                        HoldingMode::TorqueAndGap => {
                            torque_feedback
                                && current_gap >= policy.holding_gap_rad
                                && tau_median >= f64::from(policy.holding_torque_nm)
                        }
                        HoldingMode::PositionGap => {
                            feedback_fresh
                                && vel_median.abs() < motion::SETTLE_VEL
                                && current_gap >= policy.holding_gap_rad
                        }
                        HoldingMode::LoadAndGap => {
                            let req = policy.holding_load_fraction.unwrap_or(0.15) as f64;
                            feedback_fresh
                                && load_feedback
                                && current_gap >= policy.holding_gap_rad
                                && load_median >= req
                        }
                    };
                return Ok((holding, current_gap));
            }
            tokio::time::sleep(POLL).await;
        }
    }

    /// After the camera-down transition, when the model projects the target `t` into the
    /// image: one fresh frame must show it (else TARGET_LOST / DETECTION_UNAVAILABLE), within
    /// `recheck_tol_px` of the projection (else TARGET_DISPLACED). Records the box centre minus
    /// `grip_px` as `final_offset_px`. Nothing is checked when the target is not expected in view.
    async fn recheck(&self, label: &str, last: &Detection, t: &Vec3, grip_px: [f64; 2], since: Instant, after: &mut Option<u64>) -> Result<(), Stop> {
        let pose = self.pose().await;
        let Some(p) = self.cam.project(&pose, t) else { return Ok(()) };
        if !(p[0] >= 0.0 && p[0] <= self.cam.size[0] && p[1] >= 0.0 && p[1] <= self.cam.size[1]) { return Ok(()); }
        let mut tracker = Tracker { label: label.to_string(), predicted: last.box_norm, gate_px: self.setup.acquire_dist_px };
        tracker.predict_at(self.cam.to_norm(p));
        let d = self.fresh_match(&tracker, since, after).await?;
        let c = self.cam.to_px(box_centre(&d.box_norm));
        let off = [(c[0] - grip_px[0]) as f32, (c[1] - grip_px[1]) as f32];
        if self.is_executor() {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::FinalOffset { offset_px: off });
        } else {
            self.sup.inner().write().await.set_action_final_offset(self.id, off);
        }
        let moved = (c[0] - p[0]).hypot(c[1] - p[1]);
        if moved > self.setup.recheck_tol_px {
            return Err(fail("TARGET_DISPLACED", format!("Target {moved:.0} px from where it was expected after the transition (recheck_tol_px {:.0})", self.setup.recheck_tol_px)));
        }
        Ok(())
    }

    /// Closed-loop centring: drive the target's median box centre onto `aim_px` (pixel errors
    /// mapped onto the plane z = `plane`), at most `max_iter` corrections, else CENTRING_FAILED.
    /// `report`: publish `iteration` / `error_px` in the action status.
    #[allow(clippy::too_many_arguments)]
    async fn centre(&self, phase: &str, progress: f32, tracker: &mut Tracker, aim_px: [f64; 2], aim_name: &str, plane: f64,
                    mut since: Instant, after: &mut Option<u64>, max_iter: u32, report: bool) -> Result<Centred, Stop> {
        let s = &self.setup;
        let mut iteration = 0u32;
        let mut last_focus_box = tracker.predicted;
        loop {
            let pose = self.pose().await;
            let mut xs = Vec::new();
            let mut ys = Vec::new();
            let mut frames = Vec::new();
            let mut consecutive = 0u32;
            let decision = loop {
                let d = self.fresh_match(tracker, since, after).await?;
                tracker.gate_px = s.match_dist_px;
                tracker.predicted = d.box_norm;
                if self.triggered {
                    self.set_mask_focus(Some(tracker.label.clone()), d.box_norm, 2000).await;
                } else if box_changed_px(&d.box_norm, &last_focus_box, self.cam.size) {
                    self.set_mask_focus(Some(tracker.label.clone()), d.box_norm, 2000).await;
                    last_focus_box = d.box_norm;
                }
                let c = self.cam.to_px(box_centre(&d.box_norm));
                frames.push(d);
                xs.push(c[0]);
                ys.push(c[1]);
                let err = (c[0] - aim_px[0]).hypot(c[1] - aim_px[1]);
                if err <= s.centre_tol_px { consecutive += 1; } else { consecutive = 0; }
                let (it, e) = if report { (Some(iteration), Some(err as f32)) } else { (None, None) };
                if self.is_executor() {
                    self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::Phase {
                        phase: crate::rt_core::action_diag::ShortLabel::new_truncated(phase),
                        progress,
                        iteration: it,
                        error_px: e,
                        at: Instant::now(),
                    });
                } else {
                    self.sup.inner().write().await.set_action_phase(self.id, phase, progress, it, e);
                }
                if consecutive >= s.stable_frames { break None; }
                let m = s.median_frames as usize;
                if xs.len() >= m {
                    let med = [median(&mut xs[xs.len() - m..].to_vec()), median(&mut ys[ys.len() - m..].to_vec())];
                    if (med[0] - aim_px[0]).hypot(med[1] - aim_px[1]) > s.centre_tol_px { break Some(med); }
                    if xs.len() >= m + s.stable_frames as usize { break None; }
                }
            };
            let Some(med) = decision else {
                let last = frames.last().cloned().expect("at least one matched frame");
                return Ok(Centred { last, frames, pose, iterations: iteration });
            };
            if iteration >= max_iter {
                let err = (med[0] - aim_px[0]).hypot(med[1] - aim_px[1]);
                return Err(fail("CENTRING_FAILED", format!("Target still {err:.0} px from {aim_name} after {iteration} corrections")));
            }
            iteration += 1;
            let (Some(pt), Some(pg)) = (self.cam.plane_point(&pose, med, plane), self.cam.plane_point(&pose, aim_px, plane)) else {
                return Err(fail("CENTRING_FAILED", "Pixel error cannot be mapped onto the table plane"));
            };
            let mut step = [(pt[0] - pg[0]) * s.servo_gain, (pt[1] - pg[1]) * s.servo_gain, 0.0];
            let len = step[0].hypot(step[1]);
            if len > s.max_step_m { step = [step[0] * s.max_step_m / len, step[1] * s.max_step_m / len, 0.0]; }
            // predict where the target lands after the step (same rough model)
            let moved = Pose::new(pose.r, kinematics::add(&pose.p, &step));
            if let Some(p) = self.cam.project(&moved, &pt) { tracker.predict_at(self.cam.to_norm(p)); }
            let done = self.move_arm(phase, GoalSpec::Relative { frame: RelFrame::Base, d: step, dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None }, s.approach_speed_mps).await?;
            since = done.at;
        }
    }

    /// Start-frame depth: table samples (`two_height_fk` prior), the matched detection and its
    /// measurement at the start pose `pose0`.
    async fn depth_start_frame(&self, planned: &PlannedGrasp, pose0: &Pose) -> DepthCache {
        let s = &self.setup;
        let mut cache = DepthCache::default();
        let focus = MaskFocus {
            label: Some(planned.request.label.clone()),
            box_norm: planned.request.box_norm,
            ttl_ms: 2000,
        };
        // a frame of another model set is no depth frame (the next centring frame fails the grasp)
        if let Ok(Some(mut frame0)) = self.source.trigger_with_focus(&["yolo", "depth"], TRIGGER_WAIT_MS, Some(&focus)).await.map(|f| f.filter(|f| self.same_models(f))) {
            rescale_frame_to_cam(&mut frame0, &self.cam);
            if let Some(ref d) = frame0.depth {
                cache.prior = extract_table_obs(&self.cam, pose0, d, s.table_z_m);
                let cal0 = calibrate_depth_why(&self.cam, pose0, Some(d), None, s.table_z_m);
                let tracker0 = Tracker {
                    label: planned.request.label.clone(),
                    predicted: planned.request.box_norm,
                    gate_px: s.acquire_dist_px,
                };
                if let Some(det0) = tracker0.find(&frame0, s) {
                    cache.det0 = Some(det0.clone());
                    match cal0 {
                        Ok(ref cal) => match measure_detection_depth_with_cal_why(&self.cam, pose0, &det0, cal) {
                            Ok(m) => cache.meas = Some(m),
                            Err(why) => cache.why = Some(why),
                        },
                        Err(why) => cache.why = Some(why),
                    }
                } else {
                    cache.why = Some("target_lost");
                }
            } else {
                cache.why = Some("no_depth_frame");
            }
        } else {
            cache.why = Some("no_depth_frame");
        }
        cache
    }

    /// Overhead depth trigger at the centred pose: its frame becomes `after`, and its matched
    /// detection `c.last` (appended to `c.frames`). Returns the overhead measurement, else the
    /// start-frame one, else why there is none (the last failed attempt).
    async fn depth_overhead(&self, label: &str, c: &mut Centred, after: &mut Option<u64>, pose0: &Pose, cache: DepthCache) -> Result<DepthMeasurement, &'static str> {
        let s = &self.setup;
        let DepthCache { prior: cached_prior, meas: mut cached_meas, det0: cached_det0, why: cached_why } = cache;
        let mut depth_meas: Option<DepthMeasurement> = None;
        let mut why = cached_why.unwrap_or("no_depth_frame");
        let focus = MaskFocus {
            label: Some(label.to_string()),
            box_norm: c.last.box_norm,
            ttl_ms: 2000,
        };
        if let Ok(Some(mut overhead_frame)) = self.source.trigger_with_focus(&["yolo", "depth"], TRIGGER_WAIT_MS, Some(&focus)).await.map(|f| f.filter(|f| self.same_models(f))) {
            rescale_frame_to_cam(&mut overhead_frame, &self.cam);
            *after = Some(overhead_frame.frame_id);
            if metric_table_contradicts_fk(&self.cam, &c.pose, overhead_frame.depth.as_ref(), s.table_z_m) {
                tracing::debug!("Overhead table samples contradict FK; invalidating cached start-frame depth measurement");
                cached_meas = None;
                why = "table_contradicts_fk";
            }
            let tracker = Tracker {
                label: label.to_string(),
                predicted: c.last.box_norm,
                gate_px: s.match_dist_px,
            };
            if let Some(overhead_det) = tracker.find(&overhead_frame, s) {
                c.last = overhead_det.clone();
                c.frames.push(overhead_det.clone());

                match calibrate_depth_why(&self.cam, &c.pose, overhead_frame.depth.as_ref(), None, s.table_z_m)
                    .and_then(|cal| measure_detection_depth_with_cal_why(&self.cam, &c.pose, &overhead_det, &cal)) {
                    Ok(m) => depth_meas = Some(m),
                    Err(w) => why = w,
                }
                if depth_meas.is_none() {
                    depth_meas = cached_meas.clone();
                }
                if depth_meas.is_none() {
                    match calibrate_depth_why(&self.cam, &c.pose, overhead_frame.depth.as_ref(), cached_prior.as_ref(), s.table_z_m) {
                        Ok(cal) if cal.source == "two_height_fk" => {
                            if let Some(ref det0) = cached_det0 {
                                match measure_detection_depth_with_cal_why(&self.cam, &c.pose, &overhead_det, &cal) {
                                    Ok(m1) => match measure_detection_depth_with_cal_why(&self.cam, pose0, det0, &cal) {
                                        Ok(m0) => {
                                            if (m0.height_m - m1.height_m).abs() <= (0.20 * m1.height_m).max(0.008) {
                                                depth_meas = Some(m1);
                                            } else {
                                                tracing::debug!("two_height_fk rejected: height drifted between frames (m0={:.3}, m1={:.3})", m0.height_m, m1.height_m);
                                                why = "two_height_mismatch";
                                            }
                                        }
                                        Err(w) => why = w,
                                    },
                                    Err(w) => why = w,
                                }
                            }
                        }
                        Ok(cal) => match measure_detection_depth_with_cal_why(&self.cam, &c.pose, &overhead_det, &cal) {
                            Ok(m) => depth_meas = Some(m),
                            Err(w) => why = w,
                        },
                        Err(w) => why = w,
                    }
                }
            } else {
                why = "target_lost";
            }
        }
        depth_meas.or(cached_meas).ok_or(why)
    }

    /// Height source `depth` (entry name `depth:<anchor>`); `ran`: the depth capture ran.
    fn depth_source(&self, ran: bool, meas: Result<&DepthMeasurement, &'static str>) -> (String, Option<f64>, SourceResult) {
        let reject = |r: &str| ("depth".to_string(), None, SourceResult::Reject(r.into()));
        match meas {
            Ok(m) => (format!("depth:{}", m.source), Some(m.height_m), self.height_result(m.height_m)),
            Err(why) if ran => reject(why),
            Err(_) if !self.lens_known => reject("no_lens_centre"),
            Err(_) => reject("unavailable"),
        }
    }

    /// Height source `prior`: `height_priors` for the target's label (case-insensitive).
    fn prior_source(&self, label: &str) -> (String, Option<f64>, SourceResult) {
        match self.setup.height_priors.iter().find(|(k, _)| k.eq_ignore_ascii_case(label)) {
            Some((_, &h)) => ("prior".into(), Some(h), self.height_result(h)),
            None => ("prior".into(), None, SourceResult::Reject("no_prior".into())),
        }
    }

    fn height_result(&self, h: f64) -> SourceResult {
        match self.accept_height(h) {
            Ok(Ok(h)) => SourceResult::Accept(h),
            Ok(Err(reason)) => SourceResult::Reject(reason),
            Err(stop) => SourceResult::Veto(stop),
        }
    }

    async fn write_estimates(&self, chain: &EstimateChain) {
        let entries = chain.entries().to_vec();
        self.emit_estimates(
            crate::rt_core::action_diag::EstimateListKind::Height,
            &entries,
        );
        self.update_action_if_legacy(|a| a.height_estimates = entries).await;
    }

    /// Height probe after the centring (`height_probe_mode`): `Ok(Ok((h, source)))` with the
    /// object's top h above the table and `source` `scale` (up probe) or `parallax` (lateral
    /// probe); `Ok(Err(reason))` when the probe was skipped or its result is unreliable (the
    /// grasp goes on with `grasp_height_m`); OBJECT_TOO_TALL when h >= `approach_height_m` − 0.02.
    /// Both modes need the optical axis within 5 deg of vertical and the lens centre; the
    /// lateral probe gives way to the up probe (`lateral_fallback:<why>` in `height_reason`)
    /// when the shifted box would reach the image border, a lateral pose is unreachable, or the
    /// target was cut off or lost at a lateral pose.
    async fn probe_height(&self, c: &Centred, label: &str, after: &mut Option<u64>) -> Result<Result<(f64, &'static str), String>, Stop> {
        let s = &self.setup;
        let skip = |r: &str| Ok(Err(r.to_string()));
        if s.height_probe_m <= 0.0 { return skip("disabled"); }
        if !self.lens_known { return skip("no_lens_centre"); }
        if kinematics::column(&self.cam.rotation(&c.pose), 2)[2] > -(5f64.to_radians().cos()) { return skip("camera_not_vertical"); }
        if c.frames.iter().any(|d| touches_border(&d.box_norm, self.cam.size)) { return skip("border"); }
        let centre_px = self.cam.to_px(box_centre(&c.last.box_norm));
        let Some(est) = self.cam.plane_point(&c.pose, centre_px, grasp_plane(s)) else { return skip("ray"); };
        let mut fallback = None;
        if s.height_probe_mode == HeightProbeMode::Lateral {
            match self.probe_lateral(c, &est, label, after).await? {
                Lateral::Done(r) => return Ok(match r { Ok(h) => self.accept_height(h)?.map(|h| (h, "parallax")), Err(e) => Err(e) }),
                Lateral::Fallback(why) => {
                    tracing::info!("Grasp {}: lateral height probe falls back to up ({why})", self.id);
                    fallback = Some(format!("lateral_fallback:{why}"));
                }
            }
        }
        let up = match self.probe_up(c, &est, label, after).await? { Ok(h) => self.accept_height(h)?, Err(e) => Err(e) };
        Ok(match (up, fallback) {
            (Ok(h), Some(f)) => {
                if self.is_executor() {
                    let reason = self.sup.hub().intern_action_diag_str(self.id, &f);
                    self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::HeightReason { reason });
                } else {
                    self.update_action_if_legacy(|a| a.height_reason = Some(f)).await;
                }
                Ok((h, "scale"))
            }
            (Ok(h), None) => Ok((h, "scale")),
            (Err(e), Some(f)) => Err(format!("{f},{e}")),
            (Err(e), None) => Err(e),
        })
    }

    /// The acceptance rules shared by both probe modes: finite, not below the table, and
    /// OBJECT_TOO_TALL when h >= `approach_height_m` − 0.02.
    fn accept_height(&self, h: f64) -> Result<Result<f64, String>, Stop> {
        let s = &self.setup;
        if !h.is_finite() { return Ok(Err("not_finite".into())); }
        if h < 0.0 { return Ok(Err(format!("below_table:{h:.3}"))); }
        if h >= s.approach_height_m - 0.02 {
            return Err(fail("OBJECT_TOO_TALL", format!("Object top {h:.3} m above the table, approach_height_m {:.3} m leaves less than 0.02 m", s.approach_height_m)));
        }
        Ok(Ok(h))
    }

    /// Up probe: from the centred pose `c.pose` move straight up by `height_probe_m`, take
    /// `median_frames` fresh matched frames, and return to `c.pose`. The median box size
    /// s = sqrt(w·h) (px) of the centring frames (s_low) and of the raised frames (s_high) give
    /// the lens-to-top distance at the start pose D_low = dz · s_high / (s_low − s_high)
    /// ([`scale_height`]); top height = lens z − D_low − `table_z_m` (not yet accepted). Moving
    /// up keeps the jaws away from an object whose height is not known yet.
    async fn probe_up(&self, c: &Centred, est: &Vec3, label: &str, after: &mut Option<u64>) -> Result<Result<f64, String>, Stop> {
        let s = &self.setup;
        let pose = c.pose;
        let size = |d: &Detection| (((d.box_norm[2] - d.box_norm[0]) * self.cam.size[0]) * ((d.box_norm[3] - d.box_norm[1]) * self.cam.size[1])).sqrt();
        let low: Vec<f64> = c.frames.iter().map(size).collect();
        self.phase("probe", 0.45).await?;
        // height_probe_m, then the shorter fallback, when the planner rejects the raised pose
        let mut dzs = vec![s.height_probe_m];
        if s.height_probe_m > PROBE_FALLBACK_DZ_M + 1e-9 { dzs.push(PROBE_FALLBACK_DZ_M); }
        let mut moved = None;
        for dz in dzs {
            let up = Pose::new(pose.r, [pose.p[0], pose.p[1], pose.p[2] + dz]);
            if table_violation(&pose, &up, &self.envelope, s.table_z_m).is_some() { continue; }
            let spec = GoalSpec::Relative { frame: RelFrame::Base, d: [0.0, 0.0, dz], dyaw: 0.0, dpitch: 0.0, droll: 0.0, level: false, auto_yaw: None };
            match self.move_arm("probe", spec, s.approach_speed_mps).await {
                Ok(d) => { moved = Some((d, dz)); break; }
                // rejected by the planner before anything was installed: the arm is still at the start
                Err(Stop::Fail(code, _)) if code.starts_with("IK_") || code == "WORKSPACE_VIOLATION" => {}
                Err(e) => return Err(e),
            }
        }
        let Some((done, dz)) = moved else { return Ok(Err("probe_unreachable".into())); };
        self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::ProbeUp { dz_m: dz as f32 });
        self.update_action_if_legacy(|a| { a.probe_dz_m = Some(dz as f32); a.probe_mode = Some("up".into()); }).await;
        let now = self.pose().await;
        let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
        if let Some(p) = self.cam.project(&now, est) { tracker.predict_at(self.cam.to_norm(p)); }
        let mut high = Vec::new();
        let mut reason = None;
        while high.len() < s.median_frames as usize {
            let d = match self.fresh_match(&tracker, done.at, after).await {
                Ok(d) => d,
                Err(Stop::Fail(code, _)) if code == "TARGET_LOST" => { reason = Some("target_lost".to_string()); break; }
                Err(e) => return Err(e),
            };
            if touches_border(&d.box_norm, self.cam.size) { reason = Some("border".to_string()); break; }
            tracker.gate_px = s.match_dist_px;
            tracker.predicted = d.box_norm;
            high.push(size(&d));
        }
        // back to the start pose: the later steps start from it
        self.move_arm("probe", GoalSpec::Absolute { p: pose.p, r: Some(pose.r) }, s.approach_speed_mps).await?;
        if let Some(r) = reason { return Ok(Err(r)); }
        let (z_low, z_high) = (self.cam.origin(&pose)[2], self.cam.origin(&now)[2]);
        Ok(scale_height(&low, &high, z_high - z_low).map(|d_low| z_low - d_low - s.table_z_m))
    }

    /// Lateral probe: from the centred pose `c.pose` move horizontally (orientation kept) to
    /// −B/2 and then +B/2 along the base-frame direction of the image x axis (B =
    /// `lateral_probe_m`), take `median_frames` fresh matched frames at each pose, and return to
    /// `c.pose`. Before moving: [`Lateral::Fallback`] `border` when the box shifted by the
    /// predicted parallax fx · B/2 / D0 (D0 = lens height above the grasp plane) comes within
    /// 2 px of the image border at either pose, `unreachable` when either pose fails the table
    /// clearance or the planner (IK, workspace), `edge` when no box feature is on the top face at
    /// both predicted (or, after the moves, seen) poses ([`top_face_u`]). The distance D = fx · B_meas / |du|
    /// ([`parallax_distance`]) uses the median u of a top-face feature at each pose
    /// ([`top_face_u`]: box centre or outer edge) and B_meas, the image-x
    /// component of the lens displacement between the poses from FK of the measured joints;
    /// top height = mean measured lens z − D − `table_z_m` (not yet accepted).
    pub async fn probe_lateral(&self, c: &Centred, est: &Vec3, label: &str, after: &mut Option<u64>) -> Result<Lateral, Stop> {
        if self.arm_dof().await == 5 {
            return Ok(Lateral::Fallback("unreachable"));
        }
        let s = &self.setup;
        let pose = c.pose;
        let x_cam = kinematics::column(&self.cam.rotation(&pose), 0);
        let n = x_cam[0].hypot(x_cam[1]);
        // within 5 deg of a vertical optical axis the image x axis is nearly horizontal
        if n < 0.5 { return Ok(Lateral::Fallback("direction")); }
        let half = s.lateral_probe_m / 2.0;
        let at = |k: f64| Pose::new(pose.r, [pose.p[0] + k * half * x_cam[0] / n, pose.p[1] + k * half * x_cam[1] / n, pose.p[2]]);
        let (neg, pos) = (at(-1.0), at(1.0));
        // predicted shift: a point at D0 moves by −fx · dx / D0 when the camera moves dx along x_cam
        let d0 = self.cam.origin(&pose)[2] - grasp_plane(s);
        if d0.is_nan() || d0 <= 0.01 { return Ok(Lateral::Fallback("border")); }
        let shift_norm = self.cam.fx * half * n / d0 / self.cam.size[0];
        let shifted = |b: &[f64; 4], du: f64| [b[0] + du, b[1], b[2] + du, b[3]];
        if c.frames.iter().any(|d| [shift_norm, -shift_norm].iter().any(|du| touches_border(&shifted(&d.box_norm, *du), self.cam.size))) {
            return Ok(Lateral::Fallback("border"));
        }
        // a box feature on the object's top face at both predicted poses
        let px = |b: [f64; 4]| [b[0] * self.cam.size[0], b[1] * self.cam.size[1], b[2] * self.cam.size[0], b[3] * self.cam.size[1]];
        let predicted = |du: f64| c.frames.iter().map(|d| px(shifted(&d.box_norm, du))).collect::<Vec<_>>();
        if top_face_u(&predicted(shift_norm), &predicted(-shift_norm), self.cam.cx).is_none() { return Ok(Lateral::Fallback("edge")); }
        if [(&pose, &neg), (&neg, &pos), (&pos, &pose)].iter().any(|(a, b)| table_violation(a, b, &self.envelope, s.table_z_m).is_some()) {
            return Ok(Lateral::Fallback("unreachable"));
        }
        {
            let mut input = if self.is_executor() {
                self.ctx
                    .snapshot(self.deadline)
                    .await?
                    .plan_input_in(GoalFrame::EndLink)
                    .map_err(crate::macro_action::arm_fail)?
            } else {
                self.sup.inner().read().await.plan_input().map_err(crate::macro_action::arm_fail)?
            };
            let speed = Speed { linear: s.approach_speed_mps, angular: motion::DEFAULT_ANGULAR_RPS };
            let is_5dof = input.arm.arm_dof() == 5;
            for goal in [neg, pos] {
                match motion::plan_cartesian_ex(&input, goal, speed, is_5dof) {
                    Ok(path) => { let q = path.last(); input.q0 = std::array::from_fn(|i| f64::from(q[i])); }
                    Err(_) => return Ok(Lateral::Fallback("unreachable")),
                }
            }
        }
        self.phase("probe", 0.45).await?;
        // per pose: boxes (px), measured lens centres, measured camera rotation
        let mut sides: Vec<(Vec<[f64; 4]>, Vec<Vec3>, Mat3)> = Vec::new();
        let mut reason = None;
        let mut moved = false;
        'poses: for goal in [neg, pos] {
            let done = match self.move_arm("probe", GoalSpec::Absolute { p: goal.p, r: Some(goal.r) }, s.approach_speed_mps).await {
                Ok(d) => d,
                // rejected by the planner before anything was installed
                Err(Stop::Fail(code, _)) if code.starts_with("IK_") || code == "WORKSPACE_VIOLATION" => { reason = Some("unreachable"); break; }
                Err(e) => return Err(e),
            };
            if !moved {
                moved = true;
                self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::ProbeLateralStart);
                self.update_action_if_legacy(|a| a.probe_mode = Some("lateral".into())).await;
            }
            let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
            if let Some(p) = self.cam.project(&goal, est) { tracker.predict_at(self.cam.to_norm(p)); }
            let (mut boxes, mut lens, mut rot) = (Vec::new(), Vec::new(), None);
            while boxes.len() < s.median_frames as usize {
                let d = match self.fresh_match(&tracker, done.at, after).await {
                    Ok(d) => d,
                    Err(Stop::Fail(code, _)) if code == "TARGET_LOST" => { reason = Some("target_lost"); break 'poses; }
                    Err(e) => return Err(e),
                };
                if touches_border(&d.box_norm, self.cam.size) { reason = Some("border"); break 'poses; }
                tracker.gate_px = s.match_dist_px;
                tracker.predicted = d.box_norm;
                boxes.push(px(d.box_norm));
                let meas = if self.is_executor() {
                    match (self.sup.hub().prep_env(), self.sup.hub().latest()) {
                        (Some(env), Some(snap)) => env.config.arm_model.fk_f32(&snap.rt.positions),
                        _ => None,
                    }
                } else {
                    let inner = self.sup.inner().read().await;
                    inner.config.arm_model.fk_f32(&inner.measured_positions)
                };
                let meas = meas.unwrap_or(goal);
                lens.push(self.cam.origin(&meas));
                rot.get_or_insert(self.cam.rotation(&meas));
            }
            sides.push((boxes, lens, rot.unwrap_or_else(|| self.cam.rotation(&goal))));
        }
        // back to the start pose: the later steps start from it
        if moved { self.move_arm("probe", GoalSpec::Absolute { p: pose.p, r: Some(pose.r) }, s.approach_speed_mps).await?; }
        if let Some(r) = reason { return Ok(Lateral::Fallback(r)); }
        let mean = |v: &[Vec3]| { let k = v.len().max(1) as f64; v.iter().fold([0.0; 3], |a, p| kinematics::add(&a, p)).map(|x| x / k) };
        let (l_neg, l_pos) = (mean(&sides[0].1), mean(&sides[1].1));
        let x_meas = kinematics::column(&sides[0].2, 0);
        let b_meas = kinematics::dot(&kinematics::sub(&l_pos, &l_neg), &x_meas);
        // commanded: the image-x component of the commanded lens displacement (B · n)
        let b_cmd = kinematics::dot(&kinematics::sub(&self.cam.origin(&pos), &self.cam.origin(&neg)), &x_cam);
        self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::ProbeLateralBaseline {
            baseline_m: b_meas as f32,
            err_mm: ((b_meas - b_cmd) * 1000.0) as f32,
        });
        self.update_action_if_legacy(|a| {
            a.lateral_baseline_m = Some(b_meas as f32);
            a.lateral_baseline_err_mm = Some(((b_meas - b_cmd) * 1000.0) as f32);
        }).await;
        tracing::info!("Grasp {}: lateral probe baseline {:.4} m measured, {:.4} m commanded", self.id, b_meas, b_cmd);
        let z = (l_neg[2] + l_pos[2]) / 2.0;
        let Some((u_neg, u_pos, feature)) = top_face_u(&sides[0].0, &sides[1].0, self.cam.cx) else { return Ok(Lateral::Fallback("edge")); };
        tracing::info!("Grasp {}: lateral probe parallax of the box {feature}", self.id);
        Ok(Lateral::Done(parallax_distance(&u_neg, &u_pos, self.cam.fx, b_meas).map(|d| z - d - s.table_z_m)))
    }

    /// Look-down mid-height re-centre: descend to `recentre_height_m`, then centre the target
    /// on the grip pixel for that height: the model's projection of the point under the TCP on
    /// the plane z = `plane`, plus the calibration offset (`grip_pixel_norm` minus the model's
    /// projection of that point at the centring pose). Returns the `recentre` status and the
    /// corrections; target lost, table clearance or an unusable aim skip it (the descent goes on).
    async fn recentre(&self, c: &Centred, plane: f64, close_h: f64, label: &str, after: &mut Option<u64>) -> Result<(String, Option<u32>), Stop> {
        let s = &self.setup;
        let skip = |r: &str| Ok((format!("skipped:{r}"), None));
        if s.recentre_height_m <= 0.0 { return skip("disabled"); }
        if !self.lens_known { return skip("no_lens_centre"); }
        let pose = self.pose().await;
        let pose_tcp = self.tcp_pose().await;
        let z = s.table_z_m + s.recentre_height_m;
        if s.recentre_height_m <= close_h + 0.02 || z >= pose_tcp.p[2] - 1e-3 { return skip("height"); }
        let grip_px = self.cam.to_px(s.grip_pixel_norm);
        let tcp_c = motion::end_to_tcp(&c.pose, &self.tcp_offset);
        let Some(m0) = self.cam.project(&c.pose, &[tcp_c.p[0], tcp_c.p[1], plane]) else { return skip("projection"); };
        let Some(t3) = self.cam.plane_point(&c.pose, self.cam.to_px(box_centre(&c.last.box_norm)), plane) else { return skip("projection"); };
        let goal = Pose::new(pose_tcp.r, [pose_tcp.p[0], pose_tcp.p[1], z]);
        let goal_end = motion::tcp_to_end(&goal, &self.tcp_offset);
        if table_violation(&pose, &goal_end, &self.envelope, s.table_z_m).is_some() { return skip("table_clearance"); }
        let done = self.move_arm_in(GoalFrame::Tcp, "descend", GoalSpec::Absolute { p: goal.p, r: None }, s.descend_speed_mps).await?;
        let now = self.pose().await;
        let now_tcp = self.tcp_pose().await;
        let Some(m1) = self.cam.project(&now, &[now_tcp.p[0], now_tcp.p[1], plane]) else { return skip("projection"); };
        let aim = [m1[0] + grip_px[0] - m0[0], m1[1] + grip_px[1] - m0[1]];
        if !(aim[0] >= 0.0 && aim[0] <= self.cam.size[0] && aim[1] >= 0.0 && aim[1] <= self.cam.size[1]) { return skip("aim_outside_image"); }
        self.phase("recentre", 0.7).await?;
        let mut tracker = Tracker { label: label.to_string(), predicted: c.last.box_norm, gate_px: s.acquire_dist_px };
        if let Some(p) = self.cam.project(&now, &t3) { tracker.predict_at(self.cam.to_norm(p)); }
        match self.centre("recentre", 0.7, &mut tracker, aim, "the grip point", plane, done.at, after, s.recentre_max_iterations, false).await {
            Ok(r) => Ok(("done".into(), Some(r.iterations))),
            Err(Stop::Fail(code, _)) if code == "TARGET_LOST" => skip("target_lost"),
            Err(Stop::Fail(code, _)) if code == "WORKSPACE_VIOLATION" => skip("table_clearance"),
            Err(Stop::Fail(code, _)) if code == "CENTRING_FAILED" => Ok(("unconverged".into(), Some(s.recentre_max_iterations))),
            Err(e) => Err(e),
        }
    }

    /// Straight down to `close_h` above the table: the [`Insertion`] along -z that ends at the
    /// close height, run by [`insert`](Self::insert). With the same-side approach
    /// (`grasp_final`) it ends with the short segment, never rising towards the target from below.
    async fn descend(&self, close_h: f64, approach: crate::backlash::Request) -> Result<(), Stop> {
        let s = &self.setup;
        let p = self.tcp_pose().await.p;
        self.insert(Insertion { goal_tcp: [p[0], p[1], s.table_z_m + close_h], speed_mps: s.descend_speed_mps, approach }).await
    }

    /// One guarded insertion (phase `descend`): a TCP-frame path to `goal_tcp` whose final
    /// segment is the same-side approach `grasp_final`, with the table clearance enforced on
    /// the installed path. The control tick runs its contact guard ([`ContactGuard`]): it
    /// records the peak |residual| of J2 / J3 and the peak tracking error, and with
    /// `descend_contact_stop_nm` set a J2 or J3 residual above it ends the insertion at the
    /// measured position (the short segment is then not run). The straight-down descent is the
    /// insertion along -z; another approach direction is another `goal_tcp` from its pre-grasp.
    async fn insert(&self, ins: Insertion) -> Result<(), Stop> {
        let s = &self.setup;
        // the TCP never rises to the object from below; the envelope keeps the table clearance
        let rules = crate::backlash::DescentRules { table: Some((self.envelope.clone(), s.table_z_m + TABLE_CLEARANCE_M)), ..crate::backlash::DescentRules::tcp_at(self.tcp_offset) };
        let final_approach = crate::macro_action::FinalApproach { use_: "grasp_final", request: ins.approach, rules };
        // J2 / J3 bounded by descend_contact_stop_nm (none: peaks only); a contact ends the descent done
        let lim = s.descend_contact_stop_nm.map_or(0.0, |l| l as f32);
        let mut max_residual_nm = [0.0; crate::rt_core::mailbox::MAX_DOF];
        max_residual_nm[1] = lim;
        max_residual_nm[2] = lim;
        let guard = ContactGuard { max_residual_nm, stop_code: None };
        let result = self
            .move_arm_final_in_guarded(
                GoalFrame::Tcp,
                "descend",
                GoalSpec::Absolute { p: ins.goal_tcp, r: None },
                ins.speed_mps,
                Some(final_approach),
                Some(guard),
            )
            .await;
        if !self.is_executor() {
            let mut inner = self.sup.inner().write().await;
            let watch = match &result {
                Ok((_, w)) => *w,
                Err(_) => inner
                    .macro_run
                    .as_ref()
                    .filter(|m| m.id == self.id && m.child.guard == Some(guard))
                    .and_then(|m| m.ticket())
                    .and_then(|t| inner.macro_child_watch(t))
                    .unwrap_or_default(),
            };
            let contact_h = watch.contact_joint.map(|_| watch.contact_tcp_z_m.unwrap_or(f64::NAN) - s.table_z_m);
            inner.update_action(self.id, |a| {
                a.descend_peak_residual_nm = watch.peak_residual_nm;
                a.descend_peak_tracking_rad = watch.peak_tracking_rad;
                if let Some(h) = contact_h { a.descend_contact = Some(true); a.descend_contact_height_m = Some(h as f32); }
            });
            if let Some(h) = contact_h {
                tracing::info!("Grasp {}: contact during the descent at {h:.3} m above the table", self.id);
                // a contact stop ends the descent before the short segment
                inner.approach_interrupted(self.id, "contact");
            }
        } else {
            let watch = match &result {
                Ok((_, w)) => *w,
                Err(_) => self
                    .sup
                    .hub()
                    .latest()
                    .map(|s| s.rt.owner.macro_state)
                    .filter(|m| m.active_id == self.id)
                    .map(|m| m.child_watch)
                    .unwrap_or_default(),
            };
            let contact_h = watch.contact_joint.map(|_| watch.contact_tcp_z_m.unwrap_or(f64::NAN) - s.table_z_m);
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::DescendContact {
                peak_residual_nm: watch.peak_residual_nm,
                peak_tracking_rad: watch.peak_tracking_rad,
                contact_height_m: contact_h.map(|h| h as f32),
            });
            if let Some(h) = contact_h {
                tracing::info!("Grasp {}: contact during the descent at {h:.3} m above the table", self.id);
                self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::ApproachInterrupted {
                    reason: crate::rt_core::action_diag::ShortLabel::new_truncated("contact"),
                });
            }
        }
        result.map(|_| ())
    }

    /// J6 roll (about the tool axis) that closes the jaws across the object's short side, and
    /// the `align` status, from the yaw chain ([`yaw_chain`], `yaw_sources` default `mask`
    /// then `box`): the detector angle when the centring frames carry one with
    /// `angle_conf >= align_min_angle_conf`, else ±pi/2 from the box aspect. No turn when the
    /// last box is unreliable ([`align_guard`]) or no candidate fits J6's limits.
    async fn align_turn(&self, c: &Centred, plane: f64) -> (Option<f64>, String) {
        let pose = self.pose().await;
        let (q_roll, lim, sign) = if self.is_executor() {
            match (self.sup.hub().prep_env(), self.sup.hub().latest()) {
                (Some(env), Some(snap)) => {
                    let roll_idx = env.config.arm_dof().saturating_sub(1);
                    let q_roll = f64::from(snap.rt.commanded_positions.get(roll_idx).copied().unwrap_or(0.0));
                    let lim = env.config.limits.get(roll_idx).cloned().unwrap_or_else(|| crate::model::JointLimit::new("roll", -std::f32::consts::PI, std::f32::consts::PI, 1.0, 1.0));
                    let sign = env.config.arm_model.joint_map().get(roll_idx).map_or(1.0, |m| m.sign);
                    (q_roll, lim, sign)
                }
                _ => return (None, "skipped:no_snapshot".into()),
            }
        } else {
            let inner = self.sup.inner().read().await;
            let roll_idx = inner.config.arm_dof() - 1;
            let (q_roll, lim) = (f64::from(inner.commanded_positions[roll_idx]), inner.config.limits[roll_idx].clone());
            // a roll t about the tool axis moves the roll motor by sign * t
            let sign = inner.config.arm_model.joint_map().get(roll_idx).map_or(1.0, |m| m.sign);
            (q_roll, lim, sign)
        };
        let room = |t: f64| {
            let Some((lo, hi)) = lim.effective_planning_range() else { return false; };
            let q = q_roll + sign * t;
            q >= f64::from(lo) + 0.1 && q <= f64::from(hi) - 0.1
        };
        // an image angle: the long side on the plane, seen from the pose of the centring frames
        let angle_turn = |alpha: f64| -> Result<Option<f64>, &'static str> {
            let px = self.cam.to_px(box_centre(&c.last.box_norm));
            let (a, b) = (alpha.to_radians().cos(), alpha.to_radians().sin());
            let (Some(p0), Some(p1)) = (self.cam.plane_point(&c.pose, px, plane), self.cam.plane_point(&c.pose, [px[0] + 20.0 * a, px[1] + 20.0 * b], plane)) else {
                return Err("ray");
            };
            let jaw = kinematics::mat_vec(&pose.r, &[0.0, 1.0, 0.0]);
            let t0 = roll_across_long_side(jaw, kinematics::sub(&p1, &p0));
            let pi = std::f64::consts::PI;
            let Some(t) = [t0, t0 + pi, t0 - pi].into_iter().filter(|t| room(*t)).min_by(|x, y| x.abs().total_cmp(&y.abs())) else {
                return Err("j6_limits");
            };
            Ok((t.abs() >= 1e-3).then_some(t))
        };
        // the box: ±pi/2 when the short side is along the jaw closing direction (end_link y)
        // seen in the image
        let box_turn = |short_along_x: bool| -> Option<f64> {
            let jaw = kinematics::mat_vec(&kinematics::transpose(&self.cam.rec), &[0.0, 1.0, 0.0]);
            let jaw_along_x = jaw[0].abs() >= jaw[1].abs();
            if jaw_along_x == short_along_x { return None; }
            let half = std::f64::consts::FRAC_PI_2;
            // roll about +x_tool; J6 turns with it when its axis is the tool axis
            Some([half, -half].into_iter().find(|t| room(*t)).unwrap_or(half))
        };
        let out = yaw_chain(&self.setup, &c.last, &c.frames, self.cam.size, &angle_turn, &box_turn);
        if let Some(spread) = out.angle_spread_deg {
            self.emit_diag(crate::rt_core::action_diag::ActionDiagKind::AngleSpread {
                spread_deg: spread as f32,
            });
        }
        self.emit_estimates(
            crate::rt_core::action_diag::EstimateListKind::Yaw,
            &out.entries,
        );
        self.update_action_if_legacy(|a| {
            if let Some(spread) = out.angle_spread_deg { a.angle_spread_deg = Some(spread as f32); }
            a.yaw_estimates = out.entries.clone();
        }).await;
        (out.turn, out.align)
    }

    /// Grasp width source `mask`: the median `grasp_width_px` of the centring frames across the
    /// mask angle (when it is confident and consistent; else along the jaw axis), on the plane.
    fn mask_width_m(&self, c: &Centred, plane: f64) -> Result<f64, &'static str> {
        let s = &self.setup;
        let Some(w_px) = frames_width_px(&c.frames, 0.0) else { return Err("no_width") };
        let accepted_angle = frames_angle(&c.frames, s.align_min_angle_conf)
            .filter(|&(_alpha, spread)| s.align_max_angle_spread_deg <= 0.0 || spread <= s.align_max_angle_spread_deg);
        let (u_x, u_y) = if let Some((alpha, _spread)) = accepted_angle {
            let closing_deg = alpha + 90.0;
            (closing_deg.to_radians().cos(), closing_deg.to_radians().sin())
        } else {
            let jaw = kinematics::mat_vec(&kinematics::transpose(&self.cam.rec), &[0.0, 1.0, 0.0]);
            let n = jaw[0].hypot(jaw[1]);
            if n > 1e-6 { (jaw[0] / n, jaw[1] / n) } else { (0.0, 1.0) }
        };
        let centre_px = self.cam.to_px(box_centre(&c.last.box_norm));
        let half_w = w_px / 2.0;
        let p0_px = [centre_px[0] - half_w * u_x, centre_px[1] - half_w * u_y];
        let p1_px = [centre_px[0] + half_w * u_x, centre_px[1] + half_w * u_y];
        match (self.cam.plane_point(&c.pose, p0_px, plane), self.cam.plane_point(&c.pose, p1_px, plane)) {
            (Some(p0), Some(p1)) => Ok(kinematics::norm(&kinematics::sub(&p1, &p0))),
            _ => Err("ray"),
        }
    }

    /// TOO_WIDE limit (m; 0 = off): the calibrated full opening minus `pre_open_margin_m`, at
    /// most `max_grasp_width_m` (when > 0); `max_grasp_width_m` without a calibrated policy.
    async fn max_grasp_width(&self) -> f64 {
        let s = &self.setup;
        if self.is_executor() {
            let Some(env) = self.sup.hub().prep_env() else {
                return s.max_grasp_width_m;
            };
            match env.config.gripper_policy.as_ref() {
                Some(p) if p.is_calibrated() => {
                    let calib_limit = (p.opening_m(p.open_rad) as f64) - s.pre_open_margin_m;
                    if s.max_grasp_width_m > 0.0 { s.max_grasp_width_m.min(calib_limit) } else { calib_limit }
                }
                Some(p) => {
                    let check = crate::model::check_grasp_width(p, s.max_grasp_width_m, env.requires_real_jaw_curve);
                    check.effective_max_m(s.max_grasp_width_m)
                }
                None => s.max_grasp_width_m,
            }
        } else {
            let inner = self.sup.inner().read().await;
            match inner.config.gripper_policy.as_ref().filter(|p| p.is_calibrated()) {
                Some(p) => {
                    let calib_limit = (p.opening_m(p.open_rad) as f64) - s.pre_open_margin_m;
                    if s.max_grasp_width_m > 0.0 { s.max_grasp_width_m.min(calib_limit) } else { calib_limit }
                }
                // an uncalibrated (nominal) curve narrower than the setup clamps it
                None => inner.grasp_width_check(s.max_grasp_width_m)
                    .map_or(s.max_grasp_width_m, |c| c.effective_max_m(s.max_grasp_width_m)),
            }
        }
    }
}

/// Result of [`yaw_chain`]: the roll, the `align` status, the `yaw_estimates` entries and the
/// mask angle dispersion (`angle_spread_deg`, written only by a deciding `mask` source).
#[derive(Debug, Clone, PartialEq)]
pub struct YawOutcome {
    pub turn: Option<f64>,
    pub align: String,
    pub entries: Vec<EstimateEntry>,
    pub angle_spread_deg: Option<f64>,
}

/// Mean angle and dispersion of an angle source's frames (`mask`: `angle_deg` / `angle_conf`,
/// `depth`: `depth_angle_deg` / `depth_angle_conf`).
fn yaw_angle(src: YawSource, frames: &[Detection], min_conf: f64) -> Option<(f64, f64)> {
    match src {
        YawSource::Mask => frames_angle(frames, min_conf),
        YawSource::Depth => frames_angle_by(frames, min_conf, |d| (d.depth_angle_deg, d.depth_angle_conf)),
        YawSource::Box => None,
    }
}

/// The J6 yaw estimate chain (`yaw_sources`, `yaw_record`). Record-only sources are evaluated
/// first (value only, no turn). [`align_guard`] then decides for every source (`skipped:<why>`).
/// The deciding sources run in order: `mask` / `depth` without a confident angle pass to the
/// next source; with one, they decide: `skipped:angle_spread` (frames disagree), `skipped:ray`
/// / `skipped:j6_limits` (from `angle_turn`), else `kept:angle` / `turned:angle` (`mask`) or
/// `kept:depth` / `turned:depth` (`depth`). `box` passes on `aspect` (box ratio below
/// `align_min_aspect`), else `kept:box` / `turned:box` (`box_turn(short side along image x)`).
/// When every source passes: `skipped:<reason of the last one>`.
pub fn yaw_chain(
    s: &GraspSetup,
    last: &Detection,
    frames: &[Detection],
    size: [f64; 2],
    angle_turn: &dyn Fn(f64) -> Result<Option<f64>, &'static str>,
    box_turn: &dyn Fn(bool) -> Option<f64>,
) -> YawOutcome {
    let spread_ok = |spread: f64| s.align_max_angle_spread_deg <= 0.0 || spread <= s.align_max_angle_spread_deg;
    let b = last.box_norm;
    let (w, h) = ((b[2] - b[0]) * size[0], (b[3] - b[1]) * size[1]);
    let aspect_ok = w.max(h) / w.min(h).max(1e-6) >= s.align_min_aspect;
    // the box's long side in the image: along x (0 deg) or along y (-90 deg)
    let box_alpha = if w >= h { 0.0 } else { -90.0 };
    let mut chain = EstimateChain::new();
    let mut out = YawOutcome { turn: None, align: String::new(), entries: Vec::new(), angle_spread_deg: None };
    let finish = |chain: EstimateChain, mut out: YawOutcome, turn: Option<f64>, align: String| {
        out.turn = turn;
        out.align = align;
        out.entries = match chain.finish() { Ok((_, e)) | Err((_, e)) => e };
        out
    };
    for src in s.yaw_record.iter().filter(|x| !s.yaw_sources.contains(x)) {
        let r = match src {
            YawSource::Box if aspect_ok => (Some(box_alpha), SourceResult::Accept(box_alpha)),
            YawSource::Box => (Some(box_alpha), SourceResult::Reject("aspect".into())),
            _ => match yaw_angle(*src, frames, s.align_min_angle_conf) {
                Some((alpha, spread)) if spread_ok(spread) => (Some(alpha), SourceResult::Accept(alpha)),
                Some((alpha, _)) => (Some(alpha), SourceResult::Reject("angle_spread".into())),
                None => (None, SourceResult::Reject("no_angle".into())),
            },
        };
        chain.offer(src.name(), r.0, r.1, true);
    }
    if let Err(reason) = align_guard(last, frames, size) {
        return finish(chain, out, None, format!("skipped:{reason}"));
    }
    let mut passed = "no_source";
    for src in &s.yaw_sources {
        if *src == YawSource::Box {
            if !aspect_ok {
                chain.offer("box", Some(box_alpha), SourceResult::Reject("aspect".into()), false);
                passed = "aspect";
                continue;
            }
            chain.offer("box", Some(box_alpha), SourceResult::Accept(box_alpha), false);
            return match box_turn(w < h) {
                None => finish(chain, out, None, "kept:box".into()),
                Some(t) => finish(chain, out, Some(t), "turned:box".into()),
            };
        }
        let Some((alpha, spread)) = yaw_angle(*src, frames, s.align_min_angle_conf) else {
            chain.offer(src.name(), None, SourceResult::Reject("no_angle".into()), false);
            passed = "no_angle";
            continue;
        };
        if *src == YawSource::Mask { out.angle_spread_deg = Some(spread); }
        // a confident angle decides: a failure below ends the alignment without a turn
        if !spread_ok(spread) {
            chain.offer(src.name(), Some(alpha), SourceResult::Reject("angle_spread".into()), false);
            return finish(chain, out, None, "skipped:angle_spread".into());
        }
        let tag = if *src == YawSource::Mask { "angle" } else { "depth" };
        return match angle_turn(alpha) {
            Err(why) => {
                chain.offer(src.name(), Some(alpha), SourceResult::Reject(why.into()), false);
                finish(chain, out, None, format!("skipped:{why}"))
            }
            Ok(turn) => {
                chain.offer(src.name(), Some(alpha), SourceResult::Accept(alpha), false);
                let align = format!("{}:{tag}", if turn.is_some() { "turned" } else { "kept" });
                finish(chain, out, turn, align)
            }
        };
    }
    finish(chain, out, None, format!("skipped:{passed}"))
}

/// Lateral probe outcome: a height estimate (or why it is unreliable), or why the up probe runs instead.
#[derive(Debug, PartialEq)]
pub enum Lateral {
    Done(Result<f64, String>),
    Fallback(&'static str),
}

/// Matched frames of the last centring pose.
pub struct Centred {
    pub last: Detection,
    pub frames: Vec<Detection>,
    /// Commanded pose at which `frames` were taken.
    pub pose: Pose,
    pub iterations: u32,
}

/// Why the last box cannot decide the J6 roll: `border` when an edge is within 2 px of the
/// image border, `occluded` when its area differs by more than 40 % from the median area of
/// `frames` (the jaws or a hand cover part of it).
pub fn align_guard(last: &Detection, frames: &[Detection], size: [f64; 2]) -> Result<(), &'static str> {
    let b = last.box_norm;
    if touches_border(&b, size) { return Err("border"); }
    let area = |b: &[f64; 4]| (b[2] - b[0]) * (b[3] - b[1]);
    let mut areas: Vec<f64> = frames.iter().map(|d| area(&d.box_norm)).collect();
    if !areas.is_empty() {
        let m = median(&mut areas);
        if m > 0.0 && (area(&b) / m - 1.0).abs() > 0.4 { return Err("occluded"); }
    }
    Ok(())
}

/// Largest height uncertainty (m) the probe accepts: box size noise beyond it makes the
/// estimate fall back to `grasp_height_m`.
pub const HEIGHT_PROBE_TOL_M: f64 = 0.01;

/// Shorter upward probe tried when the `height_probe_m` pose is unreachable or outside the workspace.
pub const PROBE_FALLBACK_DZ_M: f64 = 0.06;

/// u (px) of a box feature that lies on the object's top face at both lateral-probe poses,
/// from the boxes [x0, y0, x1, y1] (px) at the −B/2 (`neg`) and +B/2 (`pos`) poses and the
/// principal point `cx`, with the feature's name. Looking down, the box edge on the far side of
/// `cx` is the top face's (a closer point projects farther from the principal point); an edge
/// on the near side of `cx` is the bottom of the far side face. So: the centre when every box
/// straddles `cx` (both edges on the top face), else the left edge when it is left of `cx` in
/// every box, else the right edge when it is right of `cx` in every box; None otherwise. With the
/// centre of a box seen off-centre the parallax measures a depth between top and bottom (MuJoCo
/// `axis_cube_lateral`, 0.04 m cube at u 0.3: 0.023 m from the centre).
pub fn top_face_u(neg: &[[f64; 4]], pos: &[[f64; 4]], cx: f64) -> Option<(Vec<f64>, Vec<f64>, &'static str)> {
    let all = || neg.iter().chain(pos);
    let pick = |f: fn(&[f64; 4]) -> f64| (neg.iter().map(f).collect(), pos.iter().map(f).collect());
    if all().all(|b| b[0] <= cx && b[2] >= cx) {
        let (n, p) = pick(|b| (b[0] + b[2]) / 2.0);
        return Some((n, p, "centre"));
    }
    if all().all(|b| b[0] <= cx) { let (n, p) = pick(|b| b[0]); return Some((n, p, "left edge")); }
    if all().all(|b| b[2] >= cx) { let (n, p) = pick(|b| b[2]); return Some((n, p, "right edge")); }
    None
}

/// Smallest box-centre shift (px) between the two lateral-probe poses the parallax estimate accepts.
pub const LATERAL_MIN_SHIFT_PX: f64 = 5.0;

/// Lens-to-top distance from the box-centre u (px) at the −B/2 pose (`u_neg`) and the +B/2
/// pose (`u_pos`) of the lateral probe ([`top_face_u`]) and the measured baseline along the image x axis
/// `baseline_m`: D = fx · |B| / |du|, du = median(u_pos) − median(u_neg). Err (fallback reason)
/// when |B| < 1 mm, |du| < [`LATERAL_MIN_SHIFT_PX`], or the frames' centre spread (half range
/// e of each set) makes D uncertain by more than [`HEIGHT_PROBE_TOL_M`]:
/// dD = D / |du| · √(e_neg² + e_pos²).
pub fn parallax_distance(u_neg: &[f64], u_pos: &[f64], fx: f64, baseline_m: f64) -> Result<f64, String> {
    let stats = |v: &[f64]| -> Option<(f64, f64)> {
        if v.is_empty() { return None; }
        let (lo, hi) = v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| (a.min(*x), b.max(*x)));
        Some((median(&mut v.to_vec()), (hi - lo) / 2.0))
    };
    let (Some((m_neg, e1)), Some((m_pos, e2))) = (stats(u_neg), stats(u_pos)) else { return Err("no_frames".into()); };
    let b = baseline_m.abs();
    if b.is_nan() || b < 1e-3 { return Err(format!("parallax_baseline:{baseline_m:.4}")); }
    let du = (m_pos - m_neg).abs();
    if du.is_nan() || du < LATERAL_MIN_SHIFT_PX { return Err(format!("parallax_shift:{du:.1}")); }
    let d = fx * b / du;
    let unc = d / du * e1.hypot(e2);
    if unc.is_nan() || unc > HEIGHT_PROBE_TOL_M { return Err(format!("parallax_noise:{unc:.3}")); }
    Ok(d)
}

/// Lens-to-top distance at the low pose from box sizes (px) there (`low`) and after moving
/// `dz` farther away (`high`): with r = s_low / s_high (medians), D_low = dz · s_high /
/// (s_low − s_high) = dz / (r − 1). Err (fallback reason) when r < 1.03 or when the frames'
/// size spread (half range / median of each set) makes D_low uncertain by more than
/// [`HEIGHT_PROBE_TOL_M`]: dD = dz / (r − 1)² · dr, dr = r · √(e_low² + e_high²).
pub fn scale_height(low: &[f64], high: &[f64], dz: f64) -> Result<f64, String> {
    let stats = |v: &[f64]| -> Option<(f64, f64)> {
        if v.is_empty() { return None; }
        let m = median(&mut v.to_vec());
        let (lo, hi) = v.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| (a.min(*x), b.max(*x)));
        (m > 0.0).then(|| (m, (hi - lo) / 2.0 / m))
    };
    let (Some((s_low, e1)), Some((s_high, e2))) = (stats(low), stats(high)) else { return Err("no_frames".into()); };
    let r = s_low / s_high;
    if r.is_nan() || r < 1.03 { return Err(format!("scale_change:{r:.3}")); }
    let unc = dz / (r - 1.0).powi(2) * r * e1.hypot(e2);
    if unc.is_nan() || unc > HEIGHT_PROBE_TOL_M { return Err(format!("scale_noise:{unc:.3}")); }
    Ok(dz / (r - 1.0))
}

/// Attempts at a border-free aim ([`border_free_aim`]) after the centring on the grip pixel.
pub const BORDER_SHIFTS: u32 = 3;

/// Aim pixel at which a target whose box, centred on `aim` (px), is cut by the image border
/// would be seen whole: on each axis where the median box edge of `frames` is within 2 px of
/// one border, the estimated full extent (the larger of the visible extent and the box's
/// extent along the other axis when that one is uncut, times 1 + 0.25 · (`attempt` + 1)) is
/// placed `margin` px inside that border; other axes keep `aim`. None when no axis is cut, an
/// axis is cut on both sides, or the estimate does not fit in the image.
pub fn border_free_aim(frames: &[Detection], aim: [f64; 2], size: [f64; 2], margin: f64, attempt: u32) -> Option<[f64; 2]> {
    if frames.is_empty() { return None; }
    let edge = |k: usize| median(&mut frames.iter().map(|d| d.box_norm[k] * size[k % 2]).collect::<Vec<_>>());
    let (lo, hi) = ([edge(0), edge(1)], [edge(2), edge(3)]);
    let cut = |k: usize| (lo[k] <= 2.0, hi[k] >= size[k] - 2.0);
    let mut out = aim;
    let mut any = false;
    for k in 0..2 {
        let (low, high) = cut(k);
        if !low && !high { continue; }
        if low && high { return None; }
        let o = 1 - k;
        let other = if cut(o) == (false, false) { hi[o] - lo[o] } else { 0.0 };
        let est = (hi[k] - lo[k]).max(other) * (1.0 + 0.25 * f64::from(attempt + 1));
        if est + 2.0 * margin > size[k] { return None; }
        out[k] = if high { size[k] - margin - est / 2.0 } else { margin + est / 2.0 };
        any = true;
    }
    any.then_some(out)
}

/// A box edge within 2 px of the image border (the box may be cut off).
pub fn touches_border(b: &[f64; 4], size: [f64; 2]) -> bool {
    let [w, h] = size;
    b[0] * w <= 2.0 || b[1] * h <= 2.0 || b[2] * w >= w - 2.0 || b[3] * h >= h - 2.0
}

/// Mean long-side angle (deg, [-90, 90)) and dispersion (deg) of the frames with
/// `angle_conf >= min_conf`, averaged on doubled angles (an axis, not a direction); None without such frames.
pub fn frames_angle(frames: &[Detection], min_conf: f64) -> Option<(f64, f64)> {
    frames_angle_by(frames, min_conf, |d| (d.angle_deg, d.angle_conf))
}

/// [`frames_angle`] over the (angle, confidence) pair `pick` returns per frame.
pub fn frames_angle_by(frames: &[Detection], min_conf: f64, pick: impl Fn(&Detection) -> (Option<f64>, Option<f64>)) -> Option<(f64, f64)> {
    let (mut c, mut s, mut n) = (0.0, 0.0, 0);
    for d in frames {
        if let (Some(a), Some(conf)) = pick(d) {
            if conf >= min_conf { c += (2.0 * a.to_radians()).cos(); s += (2.0 * a.to_radians()).sin(); n += 1; }
        }
    }
    if n == 0 || c.hypot(s) < 1e-9 { return None; }
    let r = (c.hypot(s) / n as f64).clamp(1e-12, 1.0);
    let spread = if n <= 1 { 0.0 } else { ((-2.0 * r.ln()).sqrt() / 2.0).to_degrees() };
    let a = (s.atan2(c) / 2.0).to_degrees();
    let mean = (a + 90.0).rem_euclid(180.0) - 90.0;
    Some((mean, spread))
}

/// Median grasp width (px) of the frames with `angle_conf >= min_conf` and a positive, finite
/// `grasp_width_px`; None without such frames.
pub fn frames_width_px(frames: &[Detection], min_conf: f64) -> Option<f64> {
    let mut widths: Vec<f64> = frames
        .iter()
        .filter(|d| d.angle_conf.unwrap_or(0.0) >= min_conf)
        .filter_map(|d| d.grasp_width_px)
        .filter(|w| *w > 0.0 && w.is_finite())
        .collect();
    if widths.is_empty() {
        None
    } else {
        Some(median(&mut widths))
    }
}

/// Roll about the (vertical, down) tool axis, in [-pi/2, pi/2), that turns the jaw closing
/// direction `jaw` (base frame) perpendicular to the long side `long` (base frame, horizontal
/// parts used). Rolling by t about the downward tool axis turns `jaw` by -t about base z.
pub fn roll_across_long_side(jaw: Vec3, long: Vec3) -> f64 {
    let pi = std::f64::consts::PI;
    let t = jaw[1].atan2(jaw[0]) - long[1].atan2(long[0]) - pi / 2.0;
    (t + pi / 2.0).rem_euclid(pi) - pi / 2.0
}

async fn run(
    sup: Arc<ArmSupervisor>,
    source: Arc<dyn DetectionSource>,
    id: u64,
    mut planned: PlannedGrasp,
    stamp: crate::supervisor::AdmissionStamp,
    deadline: Instant,
) {
    use crate::rt_core::requests::actions::{FinishState, HeldUpdate};
    let setup = planned.request.setup.clone();
    let cam = CamModel::assumed(&planned.input.arm, &setup, &planned.input.cam);
    // one mode reading per grasp: the preflight's, else read here
    let mode = match planned.mode.take() {
        Some(m) => m,
        None => source.mode_info().await,
    };
    let per_request = mode.as_ref().map(|m| m.per_request_pipelines).map_err(Clone::clone);
    let triggered = per_request == Ok(true);
    let depth_available = mode.as_ref().is_ok_and(|m| m.depth_available);
    let generation = mode.as_ref().ok().and_then(|m| m.models.as_ref()).map(|pm| pm.generation);
    let vision = mode.map_or(Ok(None), |m| m.vision);
    let ctx = MacroCtx {
        sup: sup.clone(), id, stamp, deadline, phase_timeout_s: setup.phase_timeout_s, timeout_code: "GRASP_TIMEOUT",
        timeout_message: format!("Grasp did not finish within {:.0} s", setup.timeout_s),
    };
    let contact_end_m = if ctx.is_executor() {
        sup.hub()
            .prep_env()
            .and_then(|e| e.config.effective_physical_contact_end_m())
    } else {
        sup.inner().read().await.config.effective_physical_contact_end_m()
    };
    let envelope = envelope_with_contact(planned.input.ws.as_ref(), &planned.input.cam, &cam.rec, contact_end_m);
    let lens_known = planned.input.cam.offset_m.is_some();
    let orch = Orchestrator { ctx, source: source.clone(), setup, cam, envelope, tcp_offset: planned.input.tcp_offset, triggered, lens_known, depth_available, generation };
    if orch.is_executor() {
        orch.emit_diag(crate::rt_core::action_diag::ActionDiagKind::Phase {
            phase: crate::rt_core::action_diag::ShortLabel::new_truncated("start"),
            progress: 0.0,
            iteration: None,
            error_px: None,
            at: Instant::now(),
        });
    }
    let result = if let Err(e) = per_request {
        Err(fail("DETECTION_UNAVAILABLE", format!("Cannot reach the vision provider: {e}")))
    } else if triggered {
        // perception API 2: a ["yolo"] trigger per centring step, no mode change to restore
        orch.run_selected(&planned).await
    } else { match vision {
        // perception API 1 (rebot-vision 1.2): YOLO (both when ArUco was running) with inference
        // on for the duration, then the previous mode
        Ok(Some(prev)) => {
            let want = VisionMode { mode: grasp_mode(&prev.mode).to_string(), enabled: true };
            if want == prev {
                orch.run_selected(&planned).await
            } else {
                match source.set_vision_mode(&want).await {
                    Ok(()) => {
                        let r = orch.run_selected(&planned).await;
                        if let Err(e) = source.set_vision_mode(&prev).await {
                            tracing::warn!("Grasp could not restore vision mode {} (enabled {}): {e}", prev.mode, prev.enabled);
                        }
                        r
                    }
                    Err(e) => Err(fail("DETECTION_UNAVAILABLE", format!("Cannot switch the camera to {}: {e}", want.mode))),
                }
            }
        }
        // older detectors: inference on for the duration
        Ok(None) => {
            let enabled_by_us = match source.inference_enabled().await {
                Ok(true) => Ok(false),
                Ok(false) => source.set_inference(true).await.map(|_| true),
                Err(e) => Err(e),
            };
            let r = match &enabled_by_us {
                Ok(_) => orch.run_selected(&planned).await,
                Err(e) => Err(fail("DETECTION_UNAVAILABLE", format!("Cannot read the detection state: {e}"))),
            };
            if enabled_by_us == Ok(true) {
                if let Err(e) = source.set_inference(false).await { tracing::warn!("Grasp could not restore detection off: {e}"); }
            }
            r
        }
        Err(e) => Err(fail("DETECTION_UNAVAILABLE", format!("Cannot read the vision mode: {e}"))),
    } };
    let _ = source.set_mask_focus(None, [0.0; 4], 0).await;
    match result {
        Ok((holding, gap)) => {
            let held = if holding {
                let spill_safe = planned
                    .request
                    .setup
                    .spill_safe_labels
                    .iter()
                    .any(|s| s.eq_ignore_ascii_case(&planned.request.label));
                HeldUpdate::Set {
                    label: crate::model::HolderLabel::new(&planned.request.label),
                    spill_safe,
                }
            } else {
                HeldUpdate::Unchanged
            };
            let adaptive_rev = planned.adaptive.as_ref().map(|pa| pa.revisions);
            if let Err(refusal_code) = orch
                .ctx
                .finish_fenced(
                    FinishState::Done,
                    None,
                    None,
                    Some(holding),
                    gap,
                    held,
                    adaptive_rev,
                )
                .await
            {
                let msg = format!(
                    "[lift_confirm] finish refused on live parent before completion ({refusal_code})"
                );
                for _ in 0..5 {
                    orch.ctx
                        .finish(
                            FinishState::Failed,
                            Some(refusal_code),
                            Some(msg.clone()),
                            Some(false),
                            None,
                            HeldUpdate::Clear,
                        )
                        .await;
                    if !orch.ctx.alive().await {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            }
        }
        Err(Stop::Fail(code, message)) => {
            let holding = if code == "NOT_HOLDING" || code == "DROPPED" {
                Some(false)
            } else {
                None
            };
            let held = if holding == Some(false) {
                HeldUpdate::Clear
            } else {
                HeldUpdate::Unchanged
            };
            orch.ctx
                .finish(
                    FinishState::Failed,
                    Some(&code),
                    Some(message),
                    holding,
                    None,
                    held,
                )
                .await;
        }
        Err(Stop::Cancelled) => {
            orch.ctx
                .finish(
                    FinishState::Cancelled,
                    None,
                    None,
                    None,
                    None,
                    HeldUpdate::Unchanged,
                )
                .await;
        }
    }
    if !orch.ctx.is_executor() {
        let inner = sup.inner().read().await;
        if let Some(a) = inner.last_action.as_ref().filter(|a| a.id == id) {
            let code = a.code.as_deref().unwrap_or(if a.state == "done" { "OK" } else { &a.state });
            let worst = a.motions.iter().max_by(|x, y| x.track_mm.total_cmp(&y.track_mm));
            tracing::info!("Grasp {id} {code}: phase_ms {:?}, worst motion {:?}", a.phase_ms, worst);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use serde_json::Value;
    use crate::api::proxy::DetectorUpstream;
    use crate::model::CameraMount;
    use crate::supervisor::SupervisorInner;
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
