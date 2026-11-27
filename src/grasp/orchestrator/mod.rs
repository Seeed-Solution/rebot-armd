mod adaptive;
mod probes;

use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::estimate::{EstimateChain, SourceResult};
use crate::kinematics::{self, Vec3};
use crate::macro_action::{fail, ContactGuard, MacroCtx, Stop, SubDone};
use crate::model::HoldingMode;
use crate::motion::{self, GoalFrame, GoalSpec, GripperCommand, MotionError, RelFrame, Speed};
use crate::supervisor::ArmSupervisor;

use super::{
    approach_pose, box_centre, camera_down_pose, envelope_with_contact, grasp_mode, grasp_plane,
    median, rescale_frame_to_cam, table_violation, transition_pose, CamModel, DepthCache,
    Detection, DetectionFrame, DetectionSource, GraspSetup, HeightSource, MaskFocus, PlannedGrasp,
    PreOpen, Tracker, VisionMode, WidthSource, YawSource, GRASP_PIPELINES, TABLE_CLEARANCE_M,
    TRIGGER_WAIT_MS,
};

pub use probes::{
    align_guard, border_free_aim, frames_angle, frames_angle_by, frames_width_px,
    parallax_distance, roll_across_long_side, scale_height, top_face_u, touches_border, yaw_chain,
    Centred, Lateral, YawOutcome, BORDER_SHIFTS, HEIGHT_PROBE_TOL_M, LATERAL_MIN_SHIFT_PX,
    PROBE_FALLBACK_DZ_M,
};

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
    pub(in crate::grasp) async fn holding_now(&self) -> Result<(bool, f32), Stop> {
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
}

pub(in crate::grasp) async fn run(
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
