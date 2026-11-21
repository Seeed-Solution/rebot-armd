//! Torque-saturation guard: a joint whose measured |torque| stays at or above `ratio` x the
//! motor's torque mapping range (TMAX) for `persist_ms` stops the motion and holds the
//! measured pose with torque on (no Fault).
//!
//! Motivation (real arm, 2026-11-12, black box at 5 Hz): J4 (DM4310, TMAX 10 N·m) stopped at
//! -1.60 rad against a physical stop while the wrist was driven down; its torque went
//! -2.0 -> -6.8 -> +8.8 -> +9.5 N·m in 0.4 s with no position change, and ~0.4 s later all
//! seven motors stopped replying and came back Disabled (power reset). The servo blocked hold
//! (< 0.01 rad progress for 500 ms at the lead cap) had not triggered by then.
//!
//! Watched: J1..J6 listed in `joints`, while torqued in position control with fresh torque
//! feedback. Not watched: the gripper (its own stall handling; gripping is a normal stall),
//! gravity compensation (it clamps its own feed-forward torques), the unload settle phase
//! (the arm rests on its supports and torque is cut at its end).
//!
//! Per tick the guard costs O(6) and allocates nothing; strings are built only on a trip.

use serde::{Deserialize, Serialize};

use crate::model::ArmConfig;
#[cfg(not(target_arch = "wasm32"))]
use crate::model::MotionLimitReason;
#[cfg(not(target_arch = "wasm32"))]
use crate::supervisor::SupervisorInner;

/// A torque hold ends once the held joint's measured position is at least this far (rad)
/// from the hold position and its |torque| is below the threshold.
pub const TORQUE_HOLD_RELEASE_RAD: f32 = 0.05;

/// Retreat allowance: after a trip, a motion that moves the held joint away from the load
/// (opposite to the hold torque sign) is not tripped on that joint for at most this long
/// (control-loop time, s). 1.5 s is the shortest planned park (`NormalUnload` plans with a
/// 1.5 s minimum); in the elastic-obstacle reproduction the torque fell below the threshold
/// 0.18 s into the park. After it the normal rule applies again.
pub const TORQUE_RETREAT_WINDOW_S: f64 = 1.5;
/// Retreat allowance: |torque| of the held joint may not exceed max(|torque at the trip|,
/// threshold) + this fraction of the threshold (1.68 N·m on the B601-DM J1-J3, 0.6 N·m on
/// J4-J6). A load that relaxes while the joint backs out does not rise; a rise by a tenth of
/// the threshold means the motion is loading the joint further.
pub const TORQUE_RETREAT_MARGIN_RATIO: f32 = 0.10;
/// Retreat allowance: the measured position of the held joint must follow the commanded
/// retreat within this lag (rad) beyond the lag at the start of the retreat. A joint that is
/// wedged and does not come back loses the allowance after 10 mrad of commanded retreat.
pub const TORQUE_RETREAT_LAG_RAD: f32 = 0.01;
/// Retreat allowance: the commanded position may turn back toward the load by at most this
/// much (rad) from the furthest retreat reached; a path that turns around loses the allowance.
pub const TORQUE_RETREAT_TURN_RAD: f32 = 0.001;
/// While a hold is in force, a commanded position more than this far (rad) past the trip
/// position in the push direction trips at once, without `persist_ms` (a planned path, park
/// or Cartesian move that pushes into the load again). At the 650 N·m/rad contact of the
/// reproduction 2 mrad is 1.2 N·m.
pub const TORQUE_PUSH_IN_RAD: f32 = 0.002;
/// After a trip the held joint is commanded this far (rad) back from the measured trip
/// position, in the relieving direction, so that it does not keep pressing at the trip
/// torque. Bounded: a repeated trip on the same joint and sign never backs off further than
/// this from the first trip position, and never moves a joint that was pushed back further
/// than this toward the load. 5 mrad is 1.5 mm at 0.3 m from J2; position control stays
/// on, so it is a bounded position step, not a release (the trip itself steps the command
/// back by the tracking lag, 65 mrad in the 2026-11-21 incident).
pub const TORQUE_HOLD_BACKOFF_RAD: f32 = 0.005;

/// Accepted range of `torque_guard.ratio`.
pub const MIN_RATIO: f32 = 0.3;
pub const MAX_RATIO: f32 = 0.95;

/// Profile section `torque_guard`. Absent: guard off. Missing fields take the defaults below
/// (ratio 0.85: the value intended with the recommended 24 V 14.6 A supply; the B601-DM
/// profile sets 0.60 for the 6.5 A supply in use, see `ArmConfig::rebot_dm`).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct TorqueGuardPolicy {
    pub enabled: bool,
    /// Fraction of the joint's TMAX (`damiao_ranges[j].t_max`) that counts as saturated.
    pub ratio: f32,
    /// Time |torque| must stay at or above the threshold (control-loop time, ms).
    pub persist_ms: u64,
    /// J1..J6 watched. The gripper joint is never watched.
    pub joints: [bool; 6],
}

impl Default for TorqueGuardPolicy {
    fn default() -> Self {
        Self { enabled: true, ratio: 0.85, persist_ms: 100, joints: [true; 6] }
    }
}

impl TorqueGuardPolicy {
    pub fn validate(&self) -> Result<(), String> {
        if !self.ratio.is_finite() || !(MIN_RATIO..=MAX_RATIO).contains(&self.ratio) {
            return Err(format!("torque_guard.ratio must be in [{MIN_RATIO}, {MAX_RATIO}]"));
        }
        if self.persist_ms == 0 || self.persist_ms > 2000 {
            return Err("torque_guard.persist_ms must be in 1..=2000".into());
        }
        Ok(())
    }

    /// Threshold (N·m) per joint of `config`; None = not watched.
    pub fn thresholds(&self, config: &ArmConfig) -> Vec<Option<f32>> {
        let gripper = config.gripper_index();
        (0..config.dof).map(|j| {
            if !self.enabled || j >= 6 || !self.joints[j] || Some(j) == gripper { return None; }
            config.damiao_ranges.as_ref().and_then(|r| r.get(j)).map(|r| self.ratio * r.t_max)
        }).collect()
    }
}

/// `schema.torque_guard`: the policy in force.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TorqueGuardInfo {
    pub ratio: f32,
    pub persist_ms: u64,
    /// Per joint: |torque| threshold (N·m), null = not watched.
    pub thresholds_nm: Vec<Option<f32>>,
    /// Back-out distance that ends a hold (rad).
    pub release_rad: f32,
}

impl TorqueGuardInfo {
    /// None when the profile has no enabled guard or no joint has a threshold.
    pub fn from_config(config: &ArmConfig) -> Option<Self> {
        let p = config.torque_guard.filter(|p| p.enabled)?;
        let thresholds_nm = p.thresholds(config);
        thresholds_nm.iter().any(Option::is_some).then_some(Self {
            ratio: p.ratio,
            persist_ms: p.persist_ms,
            thresholds_nm,
            release_rad: TORQUE_HOLD_RELEASE_RAD,
        })
    }
}

/// The hold in force after a trip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueHold {
    pub joint: usize,
    /// Commanded position of the joint at the trip (measured, clamped to the limits).
    pub hold_q: f32,
    /// Sign of the measured torque at the trip: the push direction.
    pub sign: f32,
    pub torque_nm: f32,
    pub threshold_nm: f32,
    /// Observation-clock time of the trip.
    pub since_s: f64,
    /// Measured position of the joint at the first trip of this hold (same joint and sign);
    /// reference for the push-in check and the back-off bound.
    pub anchor_q: f32,
    /// This hold was entered because a motion pushed past the trip position.
    pub push_in: bool,
    /// Retreat allowance of this hold.
    pub retreat: Retreat,
}

/// Retreat allowance state of a hold (see [`TORQUE_RETREAT_WINDOW_S`]).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Retreat {
    /// No retreat motion seen yet.
    #[default]
    Unused,
    /// A retreat motion runs; values are in the relief coordinate (rad, negative = away
    /// from the load, relative to `hold_q`).
    /// `elapsed_s` counts control-loop time.
    Active { elapsed_s: f64, best_cmd: f32, lag0: f32 },
    /// The allowance ended (time bound, torque rise, lag, turn-around or motion end).
    Spent,
}

impl TorqueHold {
    /// Relief coordinate of `q`: (q - hold_q) x sign; negative = away from the load.
    pub fn relief(&self, q: f32) -> f32 { (q - self.hold_q) * self.sign }

    /// Hold command of the held joint after a trip at measured position `q_meas`:
    /// [`TORQUE_HOLD_BACKOFF_RAD`] back from it in the relieving direction, never further
    /// than that from `anchor_q`, and never toward the load from `q_meas`.
    pub fn backed_off(&self, q_meas: f32) -> f32 {
        let r = (q_meas - self.anchor_q) * self.sign;
        let r_hold = (r - TORQUE_HOLD_BACKOFF_RAD).max(-TORQUE_HOLD_BACKOFF_RAD).min(r);
        self.anchor_q + r_hold * self.sign
    }
}

/// Runtime guard state kept in the supervisor.
#[derive(Debug, Clone, Default)]
pub struct TorqueGuard {
    /// Continuous time at or above the threshold (s), J1..J6.
    over_s: [f64; 6],
    pub hold: Option<TorqueHold>,
    /// Trips since start (logs).
    pub trips_total: u64,
}

impl TorqueGuard {
    /// Deterministic saturation transition. Returns the joint that newly entered a hold.
    /// The gripper is excluded by limiting the input to the first six joints.
    /// `q_cmd` is this tick's commanded position (before the reference is stepped);
    /// `motion_target` the target of the running motion, None when none runs.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, torques_nm: &[f32], thresholds_nm: &[Option<f32>], dt_s: f64, now_s: f64,
        policy: &TorqueGuardPolicy, operational: bool, watching: bool, q_meas: &[f32],
        q_cmd: &[f32], motion_target: Option<&[f32]>) -> Option<usize> {
        let previous = self.hold;
        if let Some(h) = self.hold {
            let q = q_meas.get(h.joint).copied().unwrap_or(h.hold_q);
            let tau = torques_nm.get(h.joint).copied().unwrap_or(f32::NAN);
            if !operational || ((q - h.hold_q).abs() >= TORQUE_HOLD_RELEASE_RAD && tau.abs() < h.threshold_nm) {
                self.hold = None;
            }
        }
        if !operational || !policy.enabled || !watching { self.over_s = [0.0; 6]; return None; }
        // Held joint: push-in check and retreat allowance.
        let mut exempt = None;
        if let Some(h) = self.hold.as_mut() {
            let j = h.joint;
            let cmd = q_cmd.get(j).copied().unwrap_or(h.hold_q);
            if (cmd - h.anchor_q) * h.sign > TORQUE_PUSH_IN_RAD {
                let tau = torques_nm.get(j).copied().unwrap_or(f32::NAN);
                self.over_s = [0.0; 6];
                self.trips_total += 1;
                let anchor_q = h.anchor_q;
                self.hold = Some(TorqueHold { joint: j, hold_q: q_meas.get(j).copied().unwrap_or(cmd), sign: h.sign,
                    torque_nm: if tau.is_finite() { tau } else { h.torque_nm }, threshold_nm: h.threshold_nm, since_s: now_s,
                    anchor_q, push_in: true, retreat: Retreat::Unused });
                return Some(j);
            }
            if Self::retreat_allowed(h, torques_nm, dt_s, q_meas, cmd, motion_target) { exempt = Some(j); }
        }
        let persist = policy.persist_ms as f64 / 1000.0 - 1e-6;
        let mut worst = None;
        for j in 0..6 {
            let Some(threshold) = thresholds_nm.get(j).copied().flatten() else { self.over_s[j] = 0.0; continue; };
            let tau = torques_nm.get(j).copied().unwrap_or(f32::NAN);
            if tau.is_finite() && tau.abs() >= threshold {
                self.over_s[j] += dt_s;
                let ratio = tau.abs() / threshold;
                if exempt != Some(j) && self.over_s[j] >= persist && worst.is_none_or(|(_, r): (usize, f32)| ratio > r) { worst = Some((j, ratio)); }
            } else { self.over_s[j] = 0.0; }
        }
        if let Some((j, _)) = worst {
            let threshold = thresholds_nm[j].unwrap_or(f32::INFINITY);
            let tau = torques_nm[j];
            let sign = if tau >= 0.0 { 1.0 } else { -1.0 };
            let q = q_meas.get(j).copied().unwrap_or(0.0);
            // a repeated trip on the same joint and sign keeps the first trip position
            let anchor_q = previous.filter(|p| p.joint == j && p.sign == sign).map_or(q, |p| p.anchor_q);
            self.over_s = [0.0; 6];
            self.trips_total += 1;
            self.hold = Some(TorqueHold { joint: j, hold_q: q, sign, torque_nm: tau, threshold_nm: threshold, since_s: now_s,
                anchor_q, push_in: false, retreat: Retreat::Unused });
            return Some(j);
        }
        None
    }

    /// Advance the retreat allowance of `h`; true while the held joint is exempt from a
    /// re-trip this tick.
    fn retreat_allowed(h: &mut TorqueHold, torques_nm: &[f32], dt_s: f64, q_meas: &[f32], cmd: f32,
        motion_target: Option<&[f32]>) -> bool {
        const EPS: f32 = 1e-6;
        let j = h.joint;
        let r_cmd = h.relief(cmd);
        let r_meas = q_meas.get(j).map_or(f32::NAN, |q| h.relief(*q));
        let target_relief = motion_target.and_then(|t| t.get(j)).map(|t| h.relief(*t));
        match h.retreat {
            Retreat::Spent => false,
            Retreat::Unused => {
                // a running motion that moves the held joint away from the load
                let retreating = target_relief.is_some_and(|t| t < -EPS) && r_cmd <= EPS;
                if !retreating || !r_meas.is_finite() { return false; }
                h.retreat = Retreat::Active { elapsed_s: 0.0, best_cmd: r_cmd.min(0.0), lag0: r_meas - r_cmd };
                true
            }
            Retreat::Active { elapsed_s, best_cmd, lag0 } => {
                let elapsed_s = elapsed_s + dt_s;
                let tau = torques_nm.get(j).copied().unwrap_or(f32::NAN);
                let limit = h.torque_nm.abs().max(h.threshold_nm) + TORQUE_RETREAT_MARGIN_RATIO * h.threshold_nm;
                let ok = motion_target.is_some()
                    && elapsed_s <= TORQUE_RETREAT_WINDOW_S + 1e-6
                    && tau.is_finite() && tau.abs() <= limit
                    && r_cmd <= best_cmd + TORQUE_RETREAT_TURN_RAD
                    && r_meas.is_finite() && r_meas - r_cmd <= lag0 + TORQUE_RETREAT_LAG_RAD;
                h.retreat = if ok { Retreat::Active { elapsed_s, best_cmd: best_cmd.min(r_cmd), lag0 } } else { Retreat::Spent };
                ok
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl SupervisorInner {
    /// Per-joint thresholds (N·m) from the profile; empty / all None when the guard is off.
    pub fn torque_guard_thresholds(&self) -> Vec<Option<f32>> {
        self.config.torque_guard.map(|p| p.thresholds(&self.config)).unwrap_or_default()
    }

    /// One control tick, after this tick's feedback and before the motion reference is
    /// stepped. `dt` is the control-loop step (s).
    pub(crate) fn torque_guard_tick(&mut self, dt: f64) {
        let Some(p) = self.config.torque_guard.filter(|p| p.enabled) else { return; };
        let operational = self.is_torqued && self.safety.state().is_operational() && self.gravity_comp.is_none()
            && !self.disable_pending && !self.pending_rearm;
        let thr = p.thresholds(&self.config);
        let settling = self.pending_normal_unload && self.active_trajectory.is_none();
        let watching = operational && !settling && self.driver.is_some() && self.collision.torque_feedback
            && self.feedback_numeric_valid && self.hardware_feedback_fresh(std::time::Duration::from_millis(100));
        let now = self.start_time.elapsed().as_secs_f64();
        let previous_hold = self.torque_guard.hold;
        let had_hold = previous_hold.is_some();
        let motion_target = if let Some(t) = self.servo_target.as_deref() { Some(t) }
            else if self.active_trajectory.is_some() { Some(&self.target_positions[..]) } else { None };
        if let Some(j) = self.torque_guard.step(&self.torques, &thr, dt, now, &p, operational, watching, &self.measured_positions,
            &self.commanded_positions, motion_target) {
            let threshold = thr[j].unwrap_or(f32::INFINITY);
            self.torque_trip(j, self.torques[j], threshold, &p, previous_hold.is_some_and(|h| h.joint == j));
        }
        if had_hold && self.torque_guard.hold.is_none()
            && self.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Torque) {
            self.motion_limit = None;
        }
    }

    /// Trip on joint `j`: fail the running action with `TORQUE_LIMIT`, end every motion and
    /// hold the measured pose with torque on (gripper keeps its command), the held joint
    /// backed off by [`TORQUE_HOLD_BACKOFF_RAD`] in the relieving direction.
    fn torque_trip(&mut self, j: usize, torque: f32, threshold: f32, p: &TorqueGuardPolicy, again: bool) {
        let tmax = threshold / p.ratio;
        let push_in = self.torque_guard.hold.is_some_and(|h| h.push_in);
        let message = if push_in {
            let past = self.torque_guard.hold.map_or(0.0, |h| (self.commanded_positions[j] - h.anchor_q) * h.sign);
            format!("Torque limit on joint {}: the motion pushes {past:.3} rad past the position of the torque trip, into the load ({torque:.2} N·m, threshold {threshold:.2} N·m)", j + 1)
        } else {
            format!("Torque limit on joint {}: {torque:.2} N·m at or above {threshold:.2} N·m ({:.0}% of {tmax:.1} N·m) for {} ms",
                j + 1, p.ratio * 100.0, p.persist_ms)
        };
        let running = self.last_action.as_ref().filter(|a| a.is_running()).map(|a| a.id);
        if again && !push_in && running.is_none() {
            tracing::debug!("{message}; holding the new measured position");
        } else {
            tracing::warn!("{message} (cmd {:.3} meas {:.3} rad); motion stopped, holding position",
                self.commanded_positions[j], self.measured_positions[j]);
        }
        if let Some(id) = running {
            self.finish_action(id, "failed", Some("TORQUE_LIMIT"), Some(message), None, None, None);
        }
        self.macro_run = None;
        self.hold_measured_pose();
        self.note_motion_limit(MotionLimitReason::Torque, Some(j));
        if let Some(h) = self.torque_guard.hold.as_mut() {
            let l = &self.config.limits[j];
            let q = self.measured_positions[j];
            if q.is_finite() {
                self.commanded_positions[j] = h.backed_off(q).clamp(l.min_position, l.max_position);
                self.target_positions[j] = self.commanded_positions[j];
            }
            h.hold_q = self.commanded_positions[j];
        }
    }

    /// While a torque hold is in force: a servo / jog target that moves the held joint
    /// past the hold position in the push direction is clamped to the hold position.
    /// Targets that keep it there or move it the other way (reducing the load) pass.
    pub(crate) fn torque_hold_filter(&mut self, targets: &mut [f32]) {
        let Some(h) = self.torque_guard.hold else { return; };
        let Some(t) = targets.get_mut(h.joint) else { return; };
        if (*t - h.hold_q) * h.sign > 0.0 {
            *t = h.hold_q;
            self.note_motion_limit(MotionLimitReason::Torque, Some(h.joint));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn persistence_and_measured_position_release() {
        let policy = TorqueGuardPolicy { enabled: true, persist_ms: 100, ..Default::default() };
        let thresholds = [Some(1.0), None, None, None, None, None];
        let mut g = TorqueGuard::default();
        assert!(g.step(&[2.0], &thresholds, 0.05, 1.0, &policy, true, true, &[0.0], &[0.0], None).is_none());
        assert_eq!(g.step(&[2.0], &thresholds, 0.05, 1.05, &policy, true, true, &[0.0], &[0.0], None), Some(0));
        assert!(g.hold.is_some());
        assert_eq!(g.step(&[], &thresholds, 0.01, 1.055, &policy, true, false, &[0.1], &[0.0], None), None);
        assert!(g.hold.is_some(), "missing torque feedback must not release a hold");
        assert_eq!(g.step(&[0.1], &thresholds, 0.01, 1.06, &policy, true, true, &[0.1], &[0.0], None), None);
        assert!(g.hold.is_none());
    }
}
