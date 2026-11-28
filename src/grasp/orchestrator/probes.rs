use std::time::Instant;

use crate::estimate::{EstimateChain, EstimateEntry, SourceResult};
use crate::kinematics::{self, Mat3, Pose, Vec3};
use crate::macro_action::{fail, ContactGuard, Stop};
use crate::motion::{self, GoalFrame, GoalSpec, RelFrame, Speed};

use super::{Insertion, Orchestrator};
use crate::grasp::{
    box_centre, box_changed_px, calibrate_depth_why, extract_table_obs, grasp_plane,
    measure_detection_depth_with_cal_why, median, metric_table_contradicts_fk,
    rescale_frame_to_cam, table_violation, DepthCache, DepthMeasurement, Detection, GraspSetup,
    HeightProbeMode, MaskFocus, PlannedGrasp, Tracker, YawSource, TABLE_CLEARANCE_M,
    TRIGGER_WAIT_MS,
};

impl Orchestrator {
    /// After the camera-down transition, when the model projects the target `t` into the
    /// image: one fresh frame must show it (else TARGET_LOST / DETECTION_UNAVAILABLE), within
    /// `recheck_tol_px` of the projection (else TARGET_DISPLACED). Records the box centre minus
    /// `grip_px` as `final_offset_px`. Nothing is checked when the target is not expected in view.
    pub(super) async fn recheck(&self, label: &str, last: &Detection, t: &Vec3, grip_px: [f64; 2], since: Instant, after: &mut Option<u64>) -> Result<(), Stop> {
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
    pub(super) async fn centre(&self, phase: &str, progress: f32, tracker: &mut Tracker, aim_px: [f64; 2], aim_name: &str, plane: f64,
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
    pub(super) async fn depth_start_frame(&self, planned: &PlannedGrasp, pose0: &Pose) -> DepthCache {
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
    pub(super) async fn depth_overhead(&self, label: &str, c: &mut Centred, after: &mut Option<u64>, pose0: &Pose, cache: DepthCache) -> Result<DepthMeasurement, &'static str> {
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
    pub(super) fn depth_source(&self, ran: bool, meas: Result<&DepthMeasurement, &'static str>) -> (String, Option<f64>, SourceResult) {
        let reject = |r: &str| ("depth".to_string(), None, SourceResult::Reject(r.into()));
        match meas {
            Ok(m) => (format!("depth:{}", m.source), Some(m.height_m), self.height_result(m.height_m)),
            Err(why) if ran => reject(why),
            Err(_) if !self.lens_known => reject("no_lens_centre"),
            Err(_) => reject("unavailable"),
        }
    }

    /// Height source `prior`: `height_priors` for the target's label (case-insensitive).
    pub(super) fn prior_source(&self, label: &str) -> (String, Option<f64>, SourceResult) {
        match self.setup.height_priors.iter().find(|(k, _)| k.eq_ignore_ascii_case(label)) {
            Some((_, &h)) => ("prior".into(), Some(h), self.height_result(h)),
            None => ("prior".into(), None, SourceResult::Reject("no_prior".into())),
        }
    }

    pub(super) fn height_result(&self, h: f64) -> SourceResult {
        match self.accept_height(h) {
            Ok(Ok(h)) => SourceResult::Accept(h),
            Ok(Err(reason)) => SourceResult::Reject(reason),
            Err(stop) => SourceResult::Veto(stop),
        }
    }

    pub(super) async fn write_estimates(&self, chain: &EstimateChain) {
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
    pub(super) async fn probe_height(&self, c: &Centred, label: &str, after: &mut Option<u64>) -> Result<Result<(f64, &'static str), String>, Stop> {
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
    pub(super) async fn recentre(&self, c: &Centred, plane: f64, close_h: f64, label: &str, after: &mut Option<u64>) -> Result<(String, Option<u32>), Stop> {
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
    pub(super) async fn descend(&self, close_h: f64, approach: crate::backlash::Request) -> Result<(), Stop> {
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
    pub(super) async fn insert(&self, ins: Insertion) -> Result<(), Stop> {
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
    pub(super) async fn align_turn(&self, c: &Centred, plane: f64) -> (Option<f64>, String) {
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
    pub(super) fn mask_width_m(&self, c: &Centred, plane: f64) -> Result<f64, &'static str> {
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
    pub(super) async fn max_grasp_width(&self) -> f64 {
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
