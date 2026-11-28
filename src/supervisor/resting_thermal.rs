//! Resting-pose validation, thermal cooldown tracking, and E-stop consumption.

#[allow(unused_imports)]
use super::*;

impl ThermalCooldown {
    pub(crate) fn reset(&mut self) {
        self.first_sample_at = None;
        self.last_oldest_sample_at = None;
        self.last_joint_samples.fill(None);
        self.joint_count = 0;
    }

    pub(crate) fn elapsed_ms(&self) -> u64 {
        match (self.first_sample_at, self.last_oldest_sample_at) {
            (Some(first), Some(last)) => last
                .saturating_duration_since(first)
                .as_millis()
                .min(u128::from(u64::MAX)) as u64,
            _ => 0,
        }
    }

    pub(crate) fn record_fresh_cool_samples(&mut self, samples: &[Instant]) {
        let (Some(oldest), Some(newest)) = (samples.iter().copied().min(), samples.iter().copied().max()) else {
            self.reset();
            return;
        };
        let track_joints = samples.len() <= crate::model::MAX_MOTION_DOF;
        match (self.first_sample_at, self.last_oldest_sample_at) {
            (Some(first), Some(prev_oldest)) => {
                let oldest_gapped =
                    oldest.saturating_duration_since(prev_oldest) > THERMAL_FEEDBACK_FRESHNESS_LIMIT;
                let joint_gapped = track_joints
                    && self.joint_count == samples.len()
                    && samples.iter().enumerate().any(|(i, &t)| {
                        self.last_joint_samples[i].is_none_or(|prev| {
                            t.saturating_duration_since(prev) > THERMAL_FEEDBACK_FRESHNESS_LIMIT
                        })
                    });
                let gapped = oldest_gapped || joint_gapped;
                if gapped {
                    // Continuity across all joints is only proven from the newest sample on: a joint
                    // that resumed late must not credit dwell from an older joint's timestamp.
                    self.first_sample_at = Some(newest);
                    self.last_oldest_sample_at = Some(oldest);
                } else {
                    self.first_sample_at = Some(first);
                    self.last_oldest_sample_at = Some(prev_oldest.max(oldest));
                }
                if track_joints {
                    if gapped || self.joint_count != samples.len() {
                        self.last_joint_samples.fill(None);
                        for (slot, &t) in self.last_joint_samples.iter_mut().zip(samples.iter()) {
                            *slot = Some(t);
                        }
                    } else {
                        for (slot, &t) in self.last_joint_samples.iter_mut().zip(samples.iter()) {
                            *slot = Some(slot.map_or(t, |prev| prev.max(t)));
                        }
                    }
                    self.joint_count = samples.len();
                } else {
                    self.last_joint_samples.fill(None);
                    self.joint_count = 0;
                }
            }
            _ => {
                self.first_sample_at = Some(oldest);
                self.last_oldest_sample_at = Some(oldest);
                self.last_joint_samples.fill(None);
                if track_joints {
                    for (slot, &t) in self.last_joint_samples.iter_mut().zip(samples.iter()) {
                        *slot = Some(t);
                    }
                    self.joint_count = samples.len();
                } else {
                    self.joint_count = 0;
                }
            }
        }
    }

    pub(crate) fn record_fresh_cool_at(&mut self, sample_at: Instant) {
        self.record_fresh_cool_samples(std::slice::from_ref(&sample_at));
    }
}

#[cfg(test)]
mod thermal_cooldown_tests {
    use super::{ThermalCooldown, THERMAL_FEEDBACK_FRESHNESS_LIMIT};
    use std::time::{Duration, Instant};

    /// A joint that resumes after a stale gap restarts the dwell from the newest sample: dwell is
    /// never credited from an older joint's timestamp (2026-11-24 review: J1 resumes at 300 ms,
    /// J2's last sample was at 240 ms, both sample at 330 ms -> dwell 30 ms, not 90 ms).
    #[test]
    fn staggered_gap_restarts_dwell_from_newest_sample() {
        let t0 = Instant::now();
        let ms = |v: u64| t0 + Duration::from_millis(v);
        let mut c = ThermalCooldown::default();
        c.record_fresh_cool_samples(&[ms(0), ms(0)]);
        c.record_fresh_cool_samples(&[ms(40), ms(240)]);
        assert!(Duration::from_millis(300 - 40) > THERMAL_FEEDBACK_FRESHNESS_LIMIT);
        c.record_fresh_cool_samples(&[ms(300), ms(240)]); // J1 gap 260 ms > 250 ms
        assert_eq!(c.elapsed_ms(), 0);
        c.record_fresh_cool_samples(&[ms(330), ms(330)]);
        assert_eq!(c.elapsed_ms(), 30);
    }
}

impl SupervisorInner {
    pub fn is_at_safe_resting_pose(&self) -> bool {
        self.resting_pose_violation().is_none()
    }

    /// The resting-pose conditions that fail on the current sample, or `None` when the arm
    /// may be released. Velocity is judged on this one sample.
    pub(super) fn resting_pose_violation(&self) -> Option<String> {
        self.resting_pose_static_violation().or_else(|| {
            let policy = self.resting_policy()?;
            let moving: Vec<String> = self.measured_velocities.iter().enumerate()
                .filter(|(i, v)| !v.is_finite() || v.abs() > policy.max_velocity_of(*i))
                .map(|(i, v)| format!("joint {} moving at {v:.4} rad/s (limit {})", i + 1, policy.max_velocity_of(i)))
                .collect();
            (!moving.is_empty()).then(|| moving.join("; "))
        })
    }

    /// Zero-allocation, non-blocking RT resting-pose check (M4): uses `driver.try_lock()`
    /// (never blocks `armd-control`) and returns `LockContended` on `TryLockError::WouldBlock`.
    pub(crate) fn check_resting_pose_rt(&mut self, stillness_verified: bool) -> RtRestingCheck {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return RtRestingCheck::OffRest;
        }
        let Some(policy) = self.resting_policy() else {
            return RtRestingCheck::OffRest;
        };
        let feedback_max_age = Duration::from_millis(policy.feedback_max_age_ms);
        let has_support_torque = !policy.support_torque_limits.is_empty();
        let has_support_load = !policy.support_load_limits.is_empty();
        if let Some(ref d) = self.driver {
            let check = d.try_with_ref(|drv| {
                let caps = drv.capabilities();
                if !caps.velocity_feedback || !caps.enable_feedback {
                    return RtRestingCheck::OffRest;
                }
                if has_support_torque && !caps.torque_feedback {
                    return RtRestingCheck::OffRest;
                }
                if has_support_load && !caps.load_feedback {
                    return RtRestingCheck::OffRest;
                }
                if self.safety.state().is_stopped() {
                    return RtRestingCheck::OffRest;
                }
                let fresh = self.feedback_numeric_valid
                    && drv.feedback_all_within(dof, feedback_max_age);
                if !fresh {
                    return RtRestingCheck::OffRest;
                }
                RtRestingCheck::AtRest
            });
            match check {
                Ok(RtRestingCheck::AtRest) => {}
                Ok(other) => return other,
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    return RtRestingCheck::LockContended;
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    return RtRestingCheck::OffRest;
                }
            }
        }
        let rest = self.config.resting_positions();
        if rest.len() < dof {
            return RtRestingCheck::OffRest;
        }
        let mut total = 0.0f32;
        for (i, &tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > tolerance {
                return RtRestingCheck::OffRest;
            }
            total += dev;
        }
        if total > policy.max_total_deviation {
            return RtRestingCheck::OffRest;
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof {
                    return RtRestingCheck::OffRest;
                }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    return RtRestingCheck::OffRest;
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof {
                    return RtRestingCheck::OffRest;
                }
                let load = self.loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    return RtRestingCheck::OffRest;
                }
            }
        }
        if !stillness_verified {
            for (i, &v) in self.measured_velocities.iter().enumerate() {
                if !v.is_finite() || v.abs() > policy.max_velocity_of(i) {
                    return RtRestingCheck::OffRest;
                }
            }
        }
        let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
        let ws_bad = self
            .workspace_violation_for_positions_bounded(&self.measured_positions, &mut ws_bufs)
            .is_some();
        self.ws_eval_bufs = ws_bufs;
        if ws_bad {
            RtRestingCheck::OffRest
        } else {
            RtRestingCheck::AtRest
        }
    }

    /// Zero-allocation RT equivalent of `resting_pose_violation(stillness_verified).is_some()`.
    pub(crate) fn has_resting_pose_violation_rt(&mut self, stillness_verified: bool) -> bool {
        self.check_resting_pose_rt(stillness_verified) != RtRestingCheck::AtRest
    }

    pub(super) fn default_rebot_dm_resting_pose() -> &'static RestingPosePolicy {
        static DEFAULT_REBOT_DM: std::sync::OnceLock<RestingPosePolicy> = std::sync::OnceLock::new();
        DEFAULT_REBOT_DM.get_or_init(RestingPosePolicy::rebot_dm)
    }

    pub(super) fn resting_policy(&self) -> Option<&RestingPosePolicy> {
        match &self.config.resting_pose {
            Some(policy) => Some(policy),
            None if self.driver.is_none() => Some(Self::default_rebot_dm_resting_pose()),
            None => None,
        }
    }

    /// Every resting-pose condition except stationarity: feedback, enable state, position,
    /// support torque. Lists every failing joint with its value.
    pub fn resting_pose_static_violation(&self) -> Option<String> {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return Some("feedback length does not match dof".into());
        }
        let Some(policy) = self.resting_policy() else { return Some("no resting pose configured".into()); };
        if let Some(ref d) = self.driver {
            let pre_violation = d
                .with_ref(|drv| {
                    let caps = drv.capabilities();
                    if !caps.velocity_feedback || !caps.enable_feedback {
                        return Some("driver lacks velocity/enable feedback".into());
                    }
                    if !policy.support_torque_limits.is_empty() && !caps.torque_feedback {
                        return Some("driver lacks torque feedback for support torque limits".into());
                    }
                    if !policy.support_load_limits.is_empty() && !caps.load_feedback {
                        return Some("driver lacks load feedback for support load limits".into());
                    }
                    if self.safety.state().is_stopped() { return Some(format!("safety state {:?}", self.safety.state())); }
                    None
                })
                .unwrap_or_else(|| Some("driver lock poisoned".into()));
            if let Some(msg) = pre_violation {
                return Some(msg);
            }
            if !self.hardware_feedback_fresh(Duration::from_millis(policy.feedback_max_age_ms)) {
                return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
            }
        }
        let mut failures = Vec::new();
        let mut total = 0.0;
        let rest = self.config.resting_positions();
        for (i, tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > *tolerance {
                failures.push(format!("joint {} off home by {dev:.4} rad (limit {tolerance})", i + 1));
            }
            total += dev;
        }
        if failures.is_empty() && total > policy.max_total_deviation {
            failures.push(format!("total deviation {total:.4} rad (limit {})", policy.max_total_deviation));
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof { return Some(format!("support torque index {i} out of range")); }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    failures.push(format!("joint {} torque {tau:.2} Nm (limit {max_tau})", i + 1));
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof { return Some(format!("support load index {i} out of range")); }
                let load = self.loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    failures.push(format!("joint {} load {load:.2} (limit {max_load})", i + 1));
                }
            }
        }
        if let Some(msg) = self.measured_workspace_violation("park_release") {
            failures.push(msg);
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    /// Resting-pose position tolerances and support torque/load limits without requiring
    /// operational state (!is_stopped()) or enabled driver. Used to guard disabling
    /// supporting joints during fault clearing.
    pub fn resting_pose_support_violation_with_driver(&self, drv: Option<&dyn ArmDriver>) -> Option<String> {
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return Some("feedback length does not match dof".into());
        }
        let Some(policy) = self.resting_policy() else {
            return Some("no resting pose configured".into());
        };
        if self.driver.is_some() {
            let Some(driver) = drv else {
                return Some("driver feedback unavailable".into());
            };
            let caps = driver.capabilities();
            if !caps.velocity_feedback {
                return Some("driver lacks velocity feedback".into());
            }
            if !policy.support_torque_limits.is_empty() && !caps.torque_feedback {
                return Some("driver lacks torque feedback for support torque limits".into());
            }
            if !policy.support_load_limits.is_empty() && !caps.load_feedback {
                return Some("driver lacks load feedback for support load limits".into());
            }
            let max_age = Duration::from_millis(policy.feedback_max_age_ms);
            let ages = driver.feedback_ages_snapshot();
            if !self.feedback_numeric_valid
                || ages.len() != dof
                || !ages.iter().all(|a| a.is_some_and(|age| age <= max_age))
            {
                return Some(format!("feedback older than {} ms", policy.feedback_max_age_ms));
            }
        }
        let mut failures = Vec::new();
        for (i, &v) in self.measured_velocities.iter().enumerate().take(dof) {
            let limit = policy.max_velocity_of(i);
            if !v.is_finite() || v.abs() > limit {
                failures.push(format!("joint {} moving at {v:.4} rad/s (limit {limit})", i + 1));
            }
        }
        let mut total = 0.0;
        let rest = self.config.resting_positions();
        for (i, tolerance) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > *tolerance {
                failures.push(format!("joint {} off home by {dev:.4} rad (limit {tolerance})", i + 1));
            }
            total += dev;
        }
        if failures.is_empty() && total > policy.max_total_deviation {
            failures.push(format!("total deviation {total:.4} rad (limit {})", policy.max_total_deviation));
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof {
                    return Some(format!("support torque index {i} out of range"));
                }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    failures.push(format!("joint {} torque {tau:.2} Nm (limit {max_tau})", i + 1));
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof {
                    return Some(format!("support load index {i} out of range"));
                }
                let load = self.loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    failures.push(format!("joint {} load {load:.2} (limit {max_load})", i + 1));
                }
            }
        }
        if let Some(msg) = self.measured_workspace_violation("clear_fault") {
            failures.push(msg);
        }
        if failures.is_empty() {
            None
        } else if failures.len() > crate::safety::MAX_RESTING_SUPPORT_ITEMS {
            let overflowed = failures.len() - crate::safety::MAX_RESTING_SUPPORT_ITEMS;
            let mut joined = failures[..crate::safety::MAX_RESTING_SUPPORT_ITEMS].join("; ");
            let _ = core::fmt::write(&mut joined, format_args!(" (+{overflowed} more)"));
            Some(joined)
        } else {
            Some(failures.join("; "))
        }
    }

    /// Stationary check over the settle hold window: per joint, the position span, the
    /// mean velocity over the window and over its last [`UNLOAD_STILL_RECENT`], and every
    /// velocity sample. A single quantized velocity sample above
    /// the resting-pose limit is noise when the position does not move and the mean holds.
    /// Zero-allocation RT equivalent of `unload_still_violation(settle).is_none()` (M4).
    pub(super) fn unload_still_ok_rt(&self, settle: &UnloadSettle) -> bool {
        let Some(policy) = self.resting_policy() else {
            return false;
        };
        let hold = UNLOAD_SETTLE_HOLD.as_secs_f64();
        if settle.held_s + 1e-6 < hold || settle.samples.is_empty() {
            return false;
        }
        let n = settle.samples.len() as f32;
        let recent_from = settle.pending_s - UNLOAD_STILL_RECENT.as_secs_f64() - 1e-6;
        for i in 0..self.config.dof {
            let (mut lo, mut hi, mut sum, mut peak) =
                (f32::INFINITY, f32::NEG_INFINITY, 0.0f32, 0.0f32);
            let (mut recent_sum, mut recent_n) = (0.0f32, 0u32);
            for sample in &settle.samples {
                let (q, v) = if i < sample.dof {
                    (sample.positions[i], sample.velocities[i])
                } else {
                    (f32::NAN, f32::NAN)
                };
                if !q.is_finite() || !v.is_finite() {
                    return false;
                }
                lo = lo.min(q);
                hi = hi.max(q);
                sum += v;
                peak = peak.max(v.abs());
                if sample.t >= recent_from {
                    recent_sum += v;
                    recent_n += 1;
                }
            }
            let limit = policy.max_velocity_of(i);
            let (span, mean) = (hi - lo, sum / n);
            let recent = recent_sum / recent_n.max(1) as f32;
            if recent.abs() > limit
                || span > UNLOAD_STILL_POSITION_SPAN_RAD
                || mean.abs() > limit
                || peak > limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR
            {
                return false;
            }
        }
        true
    }

    pub(super) fn unload_still_violation(&self, settle: &UnloadSettle) -> Option<String> {
        let policy = self.resting_policy()?;
        let hold = UNLOAD_SETTLE_HOLD.as_secs_f64();
        if settle.held_s + 1e-6 < hold || settle.samples.is_empty() {
            return Some(format!("held {:.2} s of {hold:.2} s", settle.held_s));
        }
        let n = settle.samples.len() as f32;
        let recent_from = settle.pending_s - UNLOAD_STILL_RECENT.as_secs_f64() - 1e-6;
        let mut failures = Vec::new();
        for i in 0..self.config.dof {
            let (mut lo, mut hi, mut sum, mut peak) = (f32::INFINITY, f32::NEG_INFINITY, 0.0f32, 0.0f32);
            let (mut recent_sum, mut recent_n) = (0.0f32, 0u32);
            for sample in &settle.samples {
                let (q, v) = if i < sample.dof {
                    (sample.positions[i], sample.velocities[i])
                } else {
                    (f32::NAN, f32::NAN)
                };
                if !q.is_finite() || !v.is_finite() { return Some(format!("joint {} non-finite feedback", i + 1)); }
                lo = lo.min(q); hi = hi.max(q); sum += v; peak = peak.max(v.abs());
                if sample.t >= recent_from { recent_sum += v; recent_n += 1; }
            }
            let limit = policy.max_velocity_of(i);
            let (span, mean) = (hi - lo, sum / n);
            let recent = recent_sum / recent_n.max(1) as f32;
            if recent.abs() > limit {
                failures.push(format!("joint {} mean velocity over the last {} ms {recent:.4} rad/s (limit {limit})",
                    i + 1, UNLOAD_STILL_RECENT.as_millis()));
            }
            if span > UNLOAD_STILL_POSITION_SPAN_RAD {
                failures.push(format!("joint {} moved {span:.4} rad in {hold:.1} s (limit {UNLOAD_STILL_POSITION_SPAN_RAD})", i + 1));
            }
            if mean.abs() > limit {
                failures.push(format!("joint {} mean velocity {mean:.4} rad/s (limit {limit})", i + 1));
            }
            if peak > limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR {
                failures.push(format!("joint {} velocity peak {peak:.4} rad/s (limit {})", i + 1, limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR));
            }
        }
        (!failures.is_empty()).then(|| failures.join("; "))
    }

    pub fn bump_motion_config_revision(&mut self) {
        self.motion_config_revision = self.motion_config_revision.wrapping_add(1).max(1);
    }

    pub fn park_continuation(&self) -> Option<ParkContinuation> { self.park_continuation }

    /// Non-zero boot identity of this supervisor/executor instance.
    pub fn boot_id(&self) -> crate::rt_core::broker::BootId { self.boot_id }

    /// Overrides the bounded planning deadline for legacy off-lock jog/home/park requests.
    pub fn set_motion_plan_deadline(&mut self, deadline: Duration) {
        self.motion_plan_deadline = deadline;
    }

    pub(crate) fn notify_park_continuation(&self) {
        if !self.rt_executor_owned { self.park_continuation_notify.notify_waiters(); }
    }

    /// True when replacing `self.active_trajectory` will not drop heap storage on RT.
    pub(crate) fn can_replace_active_trajectory(&self) -> bool {
        !self.rt_executor_owned
            || !self.active_trajectory.as_ref().is_some_and(ActiveTrajectory::has_heap_storage)
            || self.retired_trajectories.len() < self.retired_trajectories.capacity()
    }

    /// Preallocates a reusable `Box<GripperMotion>` off-RT so a 7-DOF park carrying an object
    /// runs Opening and Closing release moves with strictly zero heap allocations/frees on RT.
    pub(crate) fn prepare_park_release_spare(&mut self) {
        let (Some(g_idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.clone()) else {
            self.park_release_spare = None;
            return;
        };
        let Some(limit) = self.config.limits.get(g_idx).cloned() else {
            self.park_release_spare = None;
            return;
        };
        let req = GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: PARK_RELEASE_SPEED, timeout_s: None, tag: None };
        let start = self.commanded_positions.get(g_idx).copied().unwrap_or(policy.closed_rad);
        self.park_release_spare = Some(Box::new(GripperMotion::new(req, policy, limit, start, g_idx)));
    }

    /// Recycles a clean park-release `Box<GripperMotion>` into `park_release_spare` (0 free on RT),
    /// otherwise returns `Some(old)` iff dropping `old` would touch the heap.
    pub(crate) fn recycle_or_keep_heap_trajectory(&mut self, old: ActiveTrajectory) -> Option<ActiveTrajectory> {
        if old.action_id.is_none() && old.motion_limits.is_none() && old.then.is_none() && self.park_release_spare.is_none() {
            if let Reference::Gripper(gm) = &old.reference {
                if gm.is_park_release_clean() {
                    if let Reference::Gripper(gm) = old.reference {
                        self.park_release_spare = Some(gm);
                        return None;
                    }
                }
            }
        }
        old.has_heap_storage().then_some(old)
    }

    /// Zero-allocation, zero-driver-lock static resting-pose check for RT park settle/disable.
    pub(crate) fn is_resting_pose_static_ok(&mut self) -> bool {
        let dof = self.config.dof;
        let Some(policy) = self.resting_policy() else { return false; };
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof { return false; }
        if self.driver.is_some() {
            let c = self.driver_caps;
            let max_age = Duration::from_millis(policy.feedback_max_age_ms);
            if !c.velocity_feedback || !c.enable_feedback
                || (!policy.support_torque_limits.is_empty() && !c.torque_feedback)
                || (!policy.support_load_limits.is_empty() && !c.load_feedback)
                || (self.last_poll_enable_state != EnableState::Enabled && !self.last_poll_partial_enable)
                || self.safety.state().is_stopped()
                || !self.feedback_numeric_valid
                || !self.last_poll_oldest_age.is_some_and(|age| age <= max_age)
            { return false; }
        }
        let rest = self.config.resting_positions();
        let mut total = 0.0f32;
        for (i, &tol) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > tol { return false; }
            total += dev;
        }
        if total > policy.max_total_deviation { return false; }
        if self.driver.is_some()
            && (policy.support_torque_limits.iter().any(|&(i, m)| i >= dof || self.torques.get(i).is_none_or(|t| !t.is_finite() || t.abs() > m))
                || policy.support_load_limits.iter().any(|&(i, m)| i >= dof || self.loads.get(i).is_none_or(|l| !l.is_finite() || l.abs() > m)))
        { return false; }
        let mut bufs = std::mem::take(&mut self.ws_eval_bufs);
        let ok = self.measured_workspace_violation_bounded(&mut bufs).is_none();
        self.ws_eval_bufs = bufs;
        ok
    }

    /// Zero-allocation stationary check over the settle hold window (matches [`Self::unload_still_violation`]).
    pub(super) fn is_unload_still_ok(&self, settle: &UnloadSettle) -> bool {
        let Some(policy) = self.resting_policy() else { return false; };
        if settle.held_s + 1e-6 < UNLOAD_SETTLE_HOLD.as_secs_f64() || settle.samples.is_empty() { return false; }
        let (n, recent_from) = (settle.samples.len() as f32, settle.pending_s - UNLOAD_STILL_RECENT.as_secs_f64() - 1e-6);
        for i in 0..self.config.dof {
            let (mut lo, mut hi, mut sum, mut peak, mut r_sum, mut r_n) = (f32::INFINITY, f32::NEG_INFINITY, 0.0f32, 0.0f32, 0.0f32, 0u32);
            for s in &settle.samples {
                if i >= s.dof { return false; }
                let (q, v) = (s.positions[i], s.velocities[i]);
                if !q.is_finite() || !v.is_finite() { return false; }
                lo = lo.min(q); hi = hi.max(q); sum += v; peak = peak.max(v.abs());
                if s.t >= recent_from { r_sum += v; r_n += 1; }
            }
            let limit = policy.max_velocity_of(i);
            if (r_sum / r_n.max(1) as f32).abs() > limit || (hi - lo) > UNLOAD_STILL_POSITION_SPAN_RAD
                || (sum / n).abs() > limit || peak > limit * UNLOAD_STILL_VELOCITY_PEAK_FACTOR
            { return false; }
        }
        true
    }

    /// Automatic unload requires a configured resting pose and observable
    /// velocity, supporting torque and enable state. This is a capability check,
    /// not a claim that the arm is currently resting or ready to disable.
    pub fn supports_automatic_unload(&self) -> bool {
        if self.driver.is_none() {
            return true;
        }
        self.config.resting_pose.as_ref().is_some_and(|policy| {
            let capabilities = self.driver_caps;
            capabilities.velocity_feedback
                && capabilities.enable_feedback
                && (capabilities.torque_feedback || policy.support_torque_limits.is_empty())
                && (capabilities.load_feedback || policy.support_load_limits.is_empty())
        })
    }

    pub(crate) fn consume_pending_estop(&mut self) -> bool {
        let req = self.estop_state.requested_count();
        if req > self.applied_estop_count {
            let first_gen = self.applied_estop_count + 1;
            self.reset_hold_tx_joint_progress();
            self.estop_state.latch();
            // Non-blocking driver abort on E-stop (never stall behind a contended driver mutex
            // before latching EStop and publishing `estop_applied`); zero-lock on `DriverSlot::Exclusive`
            // and retains `DetachedCancel` on `WouldBlock` so the next release drains `drv.abort_enable()`.
            let mut estop_cancel_was_torqued: Option<bool> = None;
            let mut estop_legacy_ws: Option<(u64, u8)> = None;
            if let Some(driver_transitions::TorqueTransition::Enable(ref reducer)) =
                self.torque_transition
            {
                let was_t = reducer.was_torqued;
                if let driver_transitions::EnableOrigin::LegacyWs { token, slot, .. } =
                    reducer.origin
                {
                    estop_legacy_ws = Some((token, slot));
                }
                if let Some(d) = self.driver.as_mut() {
                    match d.try_with_mut(|drv| drv.abort_enable()) {
                        Ok(()) => {}
                        Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                            estop_cancel_was_torqued = Some(was_t);
                        }
                        Err(crate::supervisor_lock::DriverTryError::Poisoned) => {}
                    }
                }
            }
            self.apply_stop_policy(StopPolicy::EStopLatch);
            // Retain `DetachedCancel` (setting `detached_torque_pending_atomic`) BEFORE settling
            // the legacy WS slot so the waking waiter always observes the pending drain flag (R5 M4).
            if let Some(was_t) = estop_cancel_was_torqued {
                self.retain_detached_enable_cancel_if_clear(was_t);
            }
            // Settle any in-flight legacy WS enable immediately as `ESTOP_ACTIVE` so a concurrent
            // or subsequent request timeout can never beat an already-requested E-stop (R4 M5).
            if let Some((token, slot)) = estop_legacy_ws {
                self.settle_legacy_ws_failed(
                    token,
                    slot,
                    driver_transitions::LegacyTorqueFailure::EStopActive,
                );
            }
            self.applied_estop_count = req;
            self.estop_applied.store(req, Ordering::SeqCst);
            if self.pending_estop_acks.len() < crate::rt_core::executor::ESTOP_ACK_CAPACITY {
                self.pending_estop_acks.push_back(crate::rt_core::executor::EstopAck {
                    first_gen,
                    last_gen: req,
                    state: self.safety.state_arc(),
                    epoch: self.safety.epoch_id(),
                });
            } else {
                self.estop_ack_overflow_gen = self.estop_ack_overflow_gen.max(req);
            }
            true
        } else {
            false
        }
    }

    #[doc(hidden)]
    pub fn request_and_consume_estop_for_test(&mut self) -> bool {
        let _ = self.estop_state.request_and_latch();
        self.consume_pending_estop()
    }

    /// Pure, allocation-free evaluation of the next [`ThermalCooldown`] state from fresh driver
    /// feedback timestamps without mutating `self`. Any gap between consecutive fresh samples
    /// exceeding [`THERMAL_FEEDBACK_FRESHNESS_LIMIT`] restarts the dwell from the new sample.
    pub(crate) fn evaluate_thermal_cooldown_with_driver(
        &self,
        drv: &dyn ArmDriver,
        ages: &[Option<Duration>],
        thermal_below: bool,
        rx_scratch: &mut Vec<Option<Instant>>,
    ) -> ThermalCooldown {
        Self::evaluate_thermal_cooldown_fields(
            self.config.dof,
            self.thermal_cooldown,
            drv,
            ages,
            thermal_below,
            rx_scratch,
        )
    }

    pub(crate) fn evaluate_thermal_cooldown_fields(
        dof: usize,
        current_cooldown: ThermalCooldown,
        drv: &dyn ArmDriver,
        ages: &[Option<Duration>],
        thermal_below: bool,
        rx_scratch: &mut Vec<Option<Instant>>,
    ) -> ThermalCooldown {
        if !thermal_below {
            return ThermalCooldown::default();
        }
        if dof == 0 {
            return ThermalCooldown::default();
        }
        let mut next = current_cooldown;
        drv.feedback_receive_instants_into(rx_scratch);
        if rx_scratch.len() != dof || rx_scratch.iter().any(Option::is_none) {
            drv.feedback_instants_into(rx_scratch);
        }
        if rx_scratch.len() == dof && rx_scratch.iter().all(Option::is_some) {
            if dof <= crate::model::MAX_MOTION_DOF {
                let mut buf = [rx_scratch[0].unwrap(); crate::model::MAX_MOTION_DOF];
                for (dst, src) in buf.iter_mut().take(dof).zip(rx_scratch.iter()) {
                    *dst = src.unwrap();
                }
                next.record_fresh_cool_samples(&buf[..dof]);
            } else if let Some(oldest) = rx_scratch.iter().filter_map(|t| *t).min() {
                next.record_fresh_cool_at(oldest);
            } else {
                next.reset();
            }
            return next;
        }
        if ages.len() == dof && ages.iter().all(Option::is_some) {
            let drv_now = drv.clock_now();
            if dof <= crate::model::MAX_MOTION_DOF {
                let mut buf = [drv_now; crate::model::MAX_MOTION_DOF];
                for (dst, age) in buf.iter_mut().take(dof).zip(ages.iter()) {
                    let a = age.unwrap();
                    *dst = drv_now.checked_sub(a).unwrap_or(drv_now);
                }
                next.record_fresh_cool_samples(&buf[..dof]);
            } else if let Some(oldest_age) = ages.iter().copied().try_fold(Duration::ZERO, |m, a| a.map(|t| m.max(t))) {
                let oldest = drv_now.checked_sub(oldest_age).unwrap_or(drv_now);
                next.record_fresh_cool_at(oldest);
            } else {
                next.reset();
            }
            return next;
        }
        ThermalCooldown::default()
    }

    /// Evaluates continuous thermal cooldown tracking from driverless simulation temperatures at
    /// `now` without mutating `self.thermal_cooldown`.
    pub(crate) fn evaluate_thermal_cooldown_driverless(&self, now: Instant) -> ThermalCooldown {
        let mut next = self.thermal_cooldown;
        let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
        if !self.temperatures.is_empty()
            && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold)
        {
            next.record_fresh_cool_at(now);
        } else {
            next.reset();
        }
        next
    }

    /// Updates continuous thermal cooldown tracking from driverless simulation temperatures at `now`.
    pub(crate) fn update_thermal_cooldown_driverless(&mut self, now: Instant) {
        self.thermal_cooldown = self.evaluate_thermal_cooldown_driverless(now);
    }

    /// Returns the first measured joint violating configured limits plus
    /// [`JOINT_LIMIT_CLEAR_MARGIN_RAD`] (zero heap allocations), or `None` when every joint is
    /// finite and within clear bounds.
    pub(crate) fn joint_limit_clear_violation(&self) -> Option<crate::safety::JointLimitClearViolation> {
        use crate::safety::{BoundedText, JointLimitClearViolation as V};
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.config.limits.len() != dof {
            return Some(V::FeedbackIncomplete {
                measured_len: self.measured_positions.len(),
                dof,
            });
        }
        for (i, (&q, l)) in self.measured_positions.iter().zip(&self.config.limits).enumerate() {
            if !q.is_finite() {
                return Some(V::NonFinite {
                    joint_idx: i as u16,
                    joint_name: BoundedText::new(&l.name),
                    value: q,
                });
            }
            if q < l.min_position - JOINT_LIMIT_CLEAR_MARGIN_RAD
                || q > l.max_position + JOINT_LIMIT_CLEAR_MARGIN_RAD
            {
                return Some(V::OutOfBounds {
                    joint_idx: i as u16,
                    joint_name: BoundedText::new(&l.name),
                    value: q,
                    min: l.min_position,
                    max: l.max_position,
                    margin: JOINT_LIMIT_CLEAR_MARGIN_RAD,
                });
            }
        }
        None
    }

    /// True when every measured joint position is finite and within configured limits plus
    /// [`JOINT_LIMIT_CLEAR_MARGIN_RAD`].
    #[allow(dead_code)]
    pub(crate) fn joints_within_clear_limits(&self) -> bool {
        self.joint_limit_clear_violation().is_none()
    }

    /// Thermal protection is active whenever motors are torqued, a torque disable is
    /// unconfirmed, or an enable transition is in flight (`Enabling`).
    #[inline]
    pub(crate) fn thermal_protection_active(&self) -> bool {
        self.is_torqued
            || self.disable_pending
            || self.enable_lifecycle == EnableLifecycle::Enabling
            || matches!(
                self.torque_transition,
                Some(driver_transitions::TorqueTransition::Enable(_))
            )
    }

    /// Thermal protection remains active while torqued, while enabling, or while torque
    /// disable is unconfirmed. `was_active` preserves thermal protection when a torque
    /// transition confirmed disable or exited `Enabling` on the exact same Step 2 poll that
    /// delivered an over-temperature sample.
    ///
    /// Thermal cooldown accrues ONLY across consecutive fresh temperature samples; stale,
    /// unavailable, or gapped (> [`THERMAL_FEEDBACK_FRESHNESS_LIMIT`]) feedback resets or
    /// restarts the cooldown dwell.
    pub(crate) fn check_thermal_protection(&mut self, was_active: bool) {
        self.check_thermal_protection_inner(
            was_active,
            self.rt_executor_owned || self.nonblocking_driver_release,
        );
    }

    pub(crate) fn check_thermal_protection_inner(&mut self, was_active: bool, nonblocking: bool) {
        let temperature_available = match self.driver.as_ref() {
            None => {
                self.update_thermal_cooldown_driverless(Instant::now());
                true
            }
            Some(d) => {
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
                let eval = |drv: &dyn ArmDriver| {
                    let avail = drv.capabilities().temperature_feedback;
                    drv.feedback_ages_into(&mut scratch_ages);
                    let fresh = avail
                        && self.feedback_numeric_valid
                        && drv.is_feedback_valid()
                        && scratch_ages.len() == self.config.dof
                        && scratch_ages
                            .iter()
                            .all(|a| a.is_some_and(|t| t <= THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                    let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                    let thermal_below = fresh
                        && !self.temperatures.is_empty()
                        && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
                    let cd = Self::evaluate_thermal_cooldown_fields(
                        self.config.dof,
                        self.thermal_cooldown,
                        drv,
                        &scratch_ages,
                        thermal_below,
                        &mut scratch_rx,
                    );
                    (avail, cd)
                };
                let (avail, next_cd) = if nonblocking {
                    match d.try_with_ref(eval) {
                        Ok(res) => res,
                        Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                            (false, self.thermal_cooldown)
                        }
                        Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                            (false, ThermalCooldown::default())
                        }
                    }
                } else {
                    d.with_ref(eval)
                        .unwrap_or((false, ThermalCooldown::default()))
                };
                self.scratch.ages = scratch_ages;
                self.scratch.rx = scratch_rx;
                self.thermal_cooldown = next_cd;
                avail
            }
        };
        // Protective-cut exception (decision b, 2026-11-24): thermal trip is NOT a user
        // torque-off request; it intentionally bypasses `may_disable_torque_while_stopped` and
        // disables the motors in any pose (leaving an overheating motor energized is worse than
        // the arm dropping).
        if (self.thermal_protection_active() || was_active)
            && temperature_available
            && self
                .temperatures
                .iter()
                .any(|t| t.is_finite() && *t > self.config.max_temperature)
        {
            let thermal_already_latched =
                self.safety.has_latched_fault_kind(crate::safety::FaultKind::ThermalTrip);
            // R12 B4: The thermal latch must NOT suppress its own redispatch if torque is still
            // on / intent is still On and no protective disable retry is already armed or exhausted.
            let needs_thermal_redispatch = thermal_already_latched
                && (self.is_torqued || self.torque_intent == TorqueIntent::On)
                && !matches!(
                    self.background_disable_retry,
                    Some(driver_transitions::BackgroundDisableRetry::Active {
                        kind: crate::driver::DisableKind::DisableProtective { .. },
                        ..
                    }) | Some(driver_transitions::BackgroundDisableRetry::Exhausted)
                );
            if !thermal_already_latched {
                self.apply_stop_policy(StopPolicy::ThermalTrip);
            } else if needs_thermal_redispatch {
                self.thermal_cooldown.reset();
                self.redispatch_thermal_protective_disable();
            }
        }
    }
}
