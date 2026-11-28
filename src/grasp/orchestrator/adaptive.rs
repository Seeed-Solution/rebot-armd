use std::time::{Duration, Instant};

use crate::kinematics::{self, Pose};
use crate::macro_action::{fail, Stop};
use crate::motion::{self, GoalFrame, GripperCommand, MotionError, Speed};
use crate::supervisor::ArmError;

use super::{Orchestrator, POLL};
use crate::grasp::{
    box_centre, rescale_frame_to_cam, Detection, MaskFocus, PlannedAdaptive, PlannedGrasp,
    GRASP_PIPELINES, TABLE_CLEARANCE_M, TRIGGER_WAIT_MS,
};

/// Fresh-frame jaw-gate evidence cached from `fresh_match_adaptive` so `insert` can re-gate the
/// exact insertion `grasp_goal` built from `snap_ins` against the same interval jaw predicate.
struct AdaptiveJawEvidence {
    poly: Vec<[f64; 2]>,
    cut_evidence: &'static str,
    cut_tol: f64,
    open_w: f64,
    cap: Option<f64>,
}

impl Orchestrator {
    /// Evaluate the shared interval jaw predicate on `contact_pose` against cached fresh-frame
    /// footprint evidence `ev` (used by both `fresh_match_adaptive` on `snap_rev` and `insert` on
    /// `snap_ins` so wrist settle between the two snapshots can never bypass the jaw gate).
    fn gate_adaptive_jaw_pose(
        phase: &str,
        label: &str,
        route: &crate::grasp_select::AdaptiveRoute,
        contact_pose: &Pose,
        ev: &AdaptiveJawEvidence,
    ) -> Result<(), Stop> {
        let y = kinematics::column(&contact_pose.r, 1);
        let cos_elev = y[0].hypot(y[1]).clamp(1e-6, 1.0);
        let u = crate::grasp_select::geometry::unit2([y[0], y[1]]);
        let p = [
            route.contact_centre[0] + (contact_pose.p[0] - route.plan.grasp.p[0]),
            route.contact_centre[1] + (contact_pose.p[1] - route.plan.grasp.p[1]),
        ];
        let Some((lo_h, hi_h)) =
            crate::grasp_select::candidates::cut_interval_near(&ev.poly, p, u, Some(ev.cut_tol))
        else {
            return Err(fail(
                "TARGET_DISPLACED",
                format!("[{phase}] fresh footprint of '{label}' no longer covers the contact along the closing axis"),
            ));
        };
        let cut = crate::grasp_select::admission::JawCut::new(
            lo_h / cos_elev,
            hi_h / cos_elev,
            ev.cut_evidence,
        );
        if let Err(r) =
            crate::grasp_select::admission::jaw_check(&cut, route.target_hard_error_m, ev.open_w, ev.cap)
        {
            return Err(fail(&r.code, format!("[{phase}] fresh evidence of '{label}': {}", r.message)));
        }
        Ok(())
    }
    pub(super) fn check_adaptive_revisions(
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
        contact_pose: &Pose,
        expected_gen: &mut Option<u64>,
        pose_end: &Pose,
        since: Instant,
        after: &mut Option<u64>,
    ) -> Result<(Detection, AdaptiveJawEvidence), Stop> {
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

            // The shared interval jaw predicate on fresh evidence at the actual insertion contact
            // pose and closing axis (consistent with the insertion goal at actual settled wrist
            // orientation), with jaw-elevation tilt correction and actual insertion opening.
            let cut_tol = route.opening_reserve_m.clamp(0.005, 0.015);
            let (fresh_poly, cut_evidence): (Vec<[f64; 2]>, &'static str) = match &best.contour_norm {
                Some(Ok(pts)) => {
                    let world: Option<Vec<[f64; 2]>> = pts
                        .iter()
                        .map(|v| self.cam.plane_point(pose_end, self.cam.to_px(*v), z_mid).map(|q| [q[0], q[1]]))
                        .collect();
                    let Some(mut wpts) = world.filter(|w| w.len() >= 3) else {
                        return Err(fail(
                            "NO_WIDTH_EVIDENCE",
                            format!("[{phase}] fresh contour of '{label}' could not be projected onto the target plane"),
                        ));
                    };
                    crate::grasp_select::geometry::ensure_ccw(&mut wpts);
                    (wpts, "contour")
                }
                Some(Err(why)) => {
                    return Err(fail(
                        "NO_WIDTH_EVIDENCE",
                        format!("[{phase}] fresh contour of '{label}' malformed: {why}"),
                    ));
                }
                None if route.has_contour_evidence => {
                    return Err(fail(
                        "NO_WIDTH_EVIDENCE",
                        format!("[{phase}] fresh detection of '{label}' is missing its contour"),
                    ));
                }
                None => {
                    let b = best.box_norm;
                    let corners = [[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]];
                    let world: Option<Vec<[f64; 2]>> = corners
                        .iter()
                        .map(|v| self.cam.plane_point(pose_end, self.cam.to_px(*v), z_mid).map(|q| [q[0], q[1]]))
                        .collect();
                    let Some(mut wpts) = world.filter(|w| w.len() >= 3) else {
                        return Err(fail(
                            "NO_WIDTH_EVIDENCE",
                            format!("[{phase}] fresh box of '{label}' could not be projected onto the target plane"),
                        ));
                    };
                    crate::grasp_select::geometry::ensure_ccw(&mut wpts);
                    (wpts, "box")
                }
            };
            let cap = self.effective_max_grasp_width_cap().await;
            let open_w = match (route.pre_open_fraction, self.gripper_policy().await) {
                (Some(frac), Some(pol)) => {
                    let j7 = pol.closed_rad + (frac as f32) * (pol.open_rad - pol.closed_rad);
                    f64::from(pol.opening_m(j7))
                }
                _ => route.plan.jaw_opening_m,
            };
            let jaw_ev = AdaptiveJawEvidence {
                poly: fresh_poly,
                cut_evidence,
                cut_tol,
                open_w,
                cap,
            };
            Self::gate_adaptive_jaw_pose(phase, label, route, contact_pose, &jaw_ev)?;

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

            return Ok((best.clone(), jaw_ev));
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
    pub(super) async fn run_adaptive(
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
        let (_, mut jaw_ev) = self
            .fresh_match_adaptive(
                "revalidate_start",
                &planned.request.label,
                route,
                &route.plan.grasp,
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
            // Actual insertion contact pose matching the insertion goal below (settled wrist
            // orientation `actual_pre_tcp.r` and translation along `insertion_dir`).
            let rem_rev = kinematics::dot(
                &kinematics::sub(&route.plan.grasp.p, &actual_pre_tcp.p),
                &route.plan.insertion_dir,
            )
            .clamp(0.0, route.plan.insertion_m + 0.006);
            let actual_contact_pose = Pose::new(
                actual_pre_tcp.r,
                kinematics::add(&actual_pre_tcp.p, &route.plan.insertion_dir.map(|x| x * rem_rev)),
            );
            let (_, rev_jaw_ev) = self
                .fresh_match_adaptive(
                    "revalidate",
                    &planned.request.label,
                    route,
                    &actual_contact_pose,
                    &mut expected_gen,
                    &actual_pre_end,
                    pre_open_done_at,
                    &mut after,
                )
                .await?;
            jaw_ev = rev_jaw_ev;
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
        let ins_pos_err = kinematics::norm(&kinematics::sub(&actual_ins_start.p, &route.plan.pregrasp.p));
        let ins_rot_err = kinematics::angle_between(&actual_ins_start.r, &route.plan.pregrasp.r);
        let ins_j_err = max_joint_diff(&input_ins.q0, &route.plan.q_pregrasp, input_ins.arm.arm_dof());
        if ins_pos_err > 0.006
            || ins_rot_err > 2.5f64.to_radians()
            || ins_j_err > BRANCH_CONSISTENCY_TOL_RAD
        {
            return Err(fail(
                "PLAN_STALE",
                format!(
                    "[insert] arm drifted from pregrasp pose/branch before insertion (pos {ins_pos_err:.4} m, rot {:.2} deg, joint {ins_j_err:.3} rad)",
                    ins_rot_err.to_degrees()
                ),
            ));
        }
        let rem_ins = kinematics::dot(
            &kinematics::sub(&route.plan.grasp.p, &actual_ins_start.p),
            &route.plan.insertion_dir,
        )
        .clamp(0.0, route.plan.insertion_m + 0.006);
        let grasp_goal = Pose::new(
            actual_ins_start.r,
            kinematics::add(&actual_ins_start.p, &route.plan.insertion_dir.map(|x| x * rem_ins)),
        );
        // Re-gate the exact insertion `grasp_goal` built from `snap_ins` so any wrist settle
        // between `snap_rev` and `snap_ins` cannot bypass the interval jaw predicate.
        Self::gate_adaptive_jaw_pose("insert", &planned.request.label, route, &grasp_goal, &jaw_ev)?;
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
}
