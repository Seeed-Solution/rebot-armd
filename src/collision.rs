//! Collision guard: contact detection from the residual torques (`observation.residual_torques`)
//! with a protective stop, auto-resume and escalation to Fault `COLLISION`.
//!
//! This is contact detection on a gravity-model residual, not a certified safety function.
//! The default thresholds are placeholders until they are measured on a real arm; the guard
//! is off unless the profile or the runtime setting enables it. Thresholds must come from a
//! rebot-sysid measurement of the arm: on the B601-DM at the folded home pose the residuals
//! measured J1 ±1.1, J2 ~1.2, J3 ~7.0-7.5, J4 ~2.2 (4.4 during a J4 move), J5 ~0.9 N·m, above
//! several placeholder thresholds.
//!
//! Joints listed in `resting_pose.support_torque_limits` are not watched while every joint is
//! within `resting_pose.position_tolerances` of home: they rest on their mechanical support and
//! the support force shows up as residual.
//!
//! Per tick the guard costs O(6) and allocates nothing; strings are built only on a trip.

use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

use crate::motion::MotionError;
use crate::persist::{self, Loaded, PendingWrite};
use crate::safety::{FaultCode, FaultReason, SafetyError};
use crate::supervisor::{ArmError, ArmSupervisor, SupervisorInner};

pub const DEFAULT_COLLISION_PATH: &str = "/userdata/rebot-teleop/collision.json";
/// Trip times kept for the escalation count; bounds `max_auto_resumes`.
pub const MAX_TRIP_HISTORY: usize = 16;

/// `ARMD_COLLISION` or [`DEFAULT_COLLISION_PATH`].
pub fn path_from_env() -> PathBuf {
    persist::path_from_env("ARMD_COLLISION", DEFAULT_COLLISION_PATH)
}

/// Profile section `collision`. Every field is optional; the values below are placeholders
/// until measured on a real arm.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct CollisionPolicy {
    pub enabled: bool,
    /// |residual| above which J1..J6 count as in contact (N·m), before the sensitivity factor.
    pub thresholds_nm: [f32; 6],
    /// Consecutive ticks above the threshold on one joint before a trip.
    pub persist_ticks: u32,
    /// Auto-resume needs every |residual| below `clear_ratio` x threshold.
    pub clear_ratio: f32,
    /// ... continuously for this long (control-loop time).
    pub resume_after_s: f64,
    /// Trips within `escalate_window_s` that still auto-resume; the next one latches Fault.
    pub max_auto_resumes: u32,
    pub escalate_window_s: f64,
}

impl Default for CollisionPolicy {
    fn default() -> Self {
        Self {
            enabled: false,
            thresholds_nm: [6.0, 6.0, 6.0, 2.5, 2.5, 2.5],
            persist_ticks: 4,
            clear_ratio: 0.5,
            resume_after_s: 2.0,
            max_auto_resumes: 3,
            escalate_window_s: 60.0,
        }
    }
}

impl CollisionPolicy {
    pub fn validate(&self) -> Result<(), String> {
        if self.thresholds_nm.iter().any(|t| !t.is_finite() || *t <= 0.0) {
            return Err("collision.thresholds_nm: six positive finite values".into());
        }
        if self.persist_ticks == 0 || self.persist_ticks > 1000 {
            return Err("collision.persist_ticks must be in 1..=1000".into());
        }
        if !self.clear_ratio.is_finite() || self.clear_ratio <= 0.0 || self.clear_ratio >= 1.0 {
            return Err("collision.clear_ratio must be in (0, 1)".into());
        }
        if !self.resume_after_s.is_finite() || self.resume_after_s <= 0.0 || self.resume_after_s > 600.0 {
            return Err("collision.resume_after_s must be in (0, 600]".into());
        }
        if self.max_auto_resumes as usize >= MAX_TRIP_HISTORY {
            return Err(format!("collision.max_auto_resumes must be below {MAX_TRIP_HISTORY}"));
        }
        if !self.escalate_window_s.is_finite() || self.escalate_window_s <= 0.0 || self.escalate_window_s > 86_400.0 {
            return Err("collision.escalate_window_s must be in (0, 86400]".into());
        }
        Ok(())
    }
}

/// Threshold scale chosen at runtime: low x1.5, medium x1.0, high x0.7.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Sensitivity {
    Low,
    #[default]
    Medium,
    High,
}

impl Sensitivity {
    pub fn factor(self) -> f32 {
        match self { Self::Low => 1.5, Self::Medium => 1.0, Self::High => 0.7 }
    }
}

/// `observation.collision` (null while the guard is disabled or unavailable).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct CollisionObservation {
    pub active: bool,
    /// 1-based joint that tripped.
    pub joint: Option<u8>,
    pub residual_nm: Option<f32>,
    pub threshold_nm: Option<f32>,
    /// Seconds since the trip.
    pub since_s: Option<f64>,
    /// Quiet time still needed before the hold ends by itself.
    pub resume_in_s: Option<f64>,
    pub trips_in_window: u32,
    pub enabled: bool,
    pub sensitivity: Sensitivity,
}

/// The protective stop in force.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionHold {
    pub joint: usize,
    pub residual_nm: f32,
    pub threshold_nm: f32,
    /// Observation-clock time of the trip.
    pub since_s: f64,
    /// Continuous time with every residual below the clear level.
    pub quiet_s: f64,
}

/// Runtime guard state kept in the supervisor.
#[derive(Debug, Clone, Default)]
pub struct CollisionGuard {
    pub enabled: bool,
    pub sensitivity: Sensitivity,
    /// True once a runtime setting replaced the profile's.
    pub runtime: bool,
    /// File the runtime setting is written to; None = not persisted.
    pub path: Option<PathBuf>,
    /// The driver reports torque (refreshed every tick).
    pub torque_feedback: bool,
    pub hold: Option<CollisionHold>,
    /// Consecutive ticks above the threshold, J1..J6.
    over: [u32; 6],
    /// Ring of the most recent trip times (observation clock).
    trips: [f64; MAX_TRIP_HISTORY],
    trip_count: usize,
    /// Trips since start (black box / logs).
    pub trips_total: u64,
}

impl CollisionGuard {
    pub fn from_policy(policy: &CollisionPolicy) -> Self {
        Self { enabled: policy.enabled, ..Self::default() }
    }

    fn record_trip(&mut self, now: f64) {
        self.trips[self.trip_count % MAX_TRIP_HISTORY] = now;
        self.trip_count += 1;
        self.trips_total += 1;
    }

    /// Trips within `window` seconds before `now`.
    pub fn trips_in_window(&self, now: f64, window: f64) -> u32 {
        let n = self.trip_count.min(MAX_TRIP_HISTORY);
        self.trips[..n].iter().filter(|t| now - **t <= window).count() as u32
    }
}

/// `GET /api/v1/safety/collision` body and WS ack.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct CollisionConfig {
    /// Profile gravity model and driver torque feedback (capability `collision_guard`).
    pub available: bool,
    pub enabled: bool,
    pub sensitivity: Sensitivity,
    /// Effective thresholds: profile thresholds x sensitivity factor.
    pub thresholds_nm: [f32; 6],
    pub profile_thresholds_nm: [f32; 6],
    pub persist_ticks: u32,
    pub clear_ratio: f32,
    pub resume_after_s: f64,
    pub max_auto_resumes: u32,
    pub escalate_window_s: f64,
    /// `profile` or `runtime`.
    pub source: &'static str,
    pub active: bool,
}

/// File body.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct CollisionSetting {
    pub enabled: bool,
    pub sensitivity: Sensitivity,
}

impl SupervisorInner {
    pub fn collision_policy(&self) -> CollisionPolicy {
        self.config.collision.unwrap_or_default()
    }

    /// Gravity model on a 6/7-DOF profile and a driver that reports torque.
    pub fn collision_available(&self) -> bool {
        self.config.gravity_model.is_some() && self.config.arm_dof() == 6 && matches!(self.config.dof, 6 | 7) && self.collision.torque_feedback
    }

    pub fn collision_thresholds(&self) -> [f32; 6] {
        let p = self.collision_policy();
        let f = self.collision.sensitivity.factor();
        std::array::from_fn(|j| p.thresholds_nm[j] * f)
    }

    /// Motion gate: `COLLISION_HOLD` while the protective stop is in force.
    pub fn collision_hold_check(&self) -> Result<(), SafetyError> {
        match &self.collision.hold {
            Some(h) => Err(SafetyError::CollisionHold(format!(
                "Collision hold on joint {} (residual {:.2} N·m); waiting for auto-resume", h.joint + 1, h.residual_nm))),
            None => Ok(()),
        }
    }

    pub fn collision_observation(&self) -> Option<CollisionObservation> {
        if !self.collision.enabled || !self.collision_available() { return None; }
        let p = self.collision_policy();
        let now = self.start_time.elapsed().as_secs_f64();
        let h = self.collision.hold.as_ref();
        Some(CollisionObservation {
            active: h.is_some(),
            joint: h.map(|h| (h.joint + 1) as u8),
            residual_nm: h.map(|h| h.residual_nm),
            threshold_nm: h.map(|h| h.threshold_nm),
            since_s: h.map(|h| (now - h.since_s).max(0.0)),
            resume_in_s: h.map(|h| (p.resume_after_s - h.quiet_s).max(0.0)),
            trips_in_window: self.collision.trips_in_window(now, p.escalate_window_s),
            enabled: true,
            sensitivity: self.collision.sensitivity,
        })
    }

    pub fn collision_config(&self) -> CollisionConfig {
        let p = self.collision_policy();
        CollisionConfig {
            available: self.collision_available(),
            enabled: self.collision.enabled,
            sensitivity: self.collision.sensitivity,
            thresholds_nm: self.collision_thresholds(),
            profile_thresholds_nm: p.thresholds_nm,
            persist_ticks: p.persist_ticks,
            clear_ratio: p.clear_ratio,
            resume_after_s: p.resume_after_s,
            max_auto_resumes: p.max_auto_resumes,
            escalate_window_s: p.escalate_window_s,
            source: if self.collision.runtime { "runtime" } else { "profile" },
            active: self.collision.hold.is_some(),
        }
    }

    /// One control tick of the guard, after the residuals of this tick are known and
    /// before the motion reference is stepped. `dt` is the control-loop step (s).
    /// J1..J6 resting on their support: listed in `resting_pose.support_torque_limits` and
    /// every arm joint (`ArmConfig::resting_arm_joints`) within `resting_pose.position_tolerances`
    /// of home (measured positions).
    pub fn collision_supported_joints(&self) -> [bool; 6] {
        let mut out = [false; 6];
        let Some(policy) = self.config.resting_pose.as_ref() else { return out; };
        let home = &self.config.home_positions;
        // Every arm joint needs a tolerance and a reading within it (a short tolerance vector
        // would leave the joints past its end unchecked).
        let n = self.config.resting_arm_joints();
        let at_rest = n > 0 && (0..n).all(|i| {
            match (self.measured_positions.get(i), home.get(i), policy.position_tolerances.get(i)) {
                (Some(q), Some(h), Some(tol)) => (q - h).abs() <= *tol,
                _ => false,
            }
        });
        if at_rest {
            for &(i, _) in &policy.support_torque_limits { if i < 6 { out[i] = true; } }
        }
        out
    }

    pub(crate) fn collision_tick(&mut self, dt: f64) {
        let p = self.collision_policy();
        let operational = self.is_torqued && self.safety.state().is_operational() && self.gravity_comp.is_none()
            && !self.disable_pending && !self.pending_rearm;
        if let Some(mut hold) = self.collision.hold {
            // A stop, torque off, gravity-comp or switching the guard off ends the hold;
            // each of these has its own way back to motion.
            if !self.collision.enabled || !operational {
                tracing::info!("Collision hold on joint {} ended (guard off, stop or torque off)", hold.joint + 1);
                self.collision.hold = None;
                self.collision.over = [0; 6];
                return;
            }
            let thr = self.collision_thresholds();
            let supported = self.collision_supported_joints();
            let quiet = self.residual_torques.is_some_and(|r| (0..6).all(|j| supported[j] || r[j].abs() < p.clear_ratio * thr[j]));
            hold.quiet_s = if quiet { hold.quiet_s + dt } else { 0.0 };
            if hold.quiet_s >= p.resume_after_s {
                tracing::info!("Collision hold on joint {} ended: residuals below {:.0}% of the thresholds for {:.1} s",
                    hold.joint + 1, p.clear_ratio * 100.0, p.resume_after_s);
                self.collision.hold = None;
                self.collision.over = [0; 6];
            } else {
                self.collision.hold = Some(hold);
            }
            return;
        }
        let watching = self.collision.enabled && self.collision_available() && operational && self.unload_settle.is_none();
        let Some(r) = self.residual_torques.filter(|_| watching) else {
            self.collision.over = [0; 6];
            return;
        };
        let thr = self.collision_thresholds();
        let supported = self.collision_supported_joints();
        let mut worst: Option<(usize, f32)> = None;
        for j in 0..6 {
            if !supported[j] && r[j].abs() > thr[j] {
                self.collision.over[j] = self.collision.over[j].saturating_add(1);
                let ratio = r[j].abs() / thr[j];
                if self.collision.over[j] >= p.persist_ticks && worst.is_none_or(|(_, w)| ratio > w) {
                    worst = Some((j, ratio));
                }
            } else {
                self.collision.over[j] = 0;
            }
        }
        if let Some((j, _)) = worst {
            self.collision_trip(j, r[j], thr[j], &p);
        }
    }

    /// Trip on joint `j`: cancel the running action, hold the measured pose with torque on,
    /// and latch Fault `COLLISION` when this trip exceeds `max_auto_resumes` in the window.
    fn collision_trip(&mut self, j: usize, residual: f32, threshold: f32, p: &CollisionPolicy) {
        let now = self.start_time.elapsed().as_secs_f64();
        self.collision.record_trip(now);
        self.collision.over = [0; 6];
        let trips = self.collision.trips_in_window(now, p.escalate_window_s);
        let message = format!("Collision on joint {}: residual {residual:.2} N·m above threshold {threshold:.2} N·m", j + 1);
        tracing::warn!("{message} (trip {trips} within {:.0} s); protective stop", p.escalate_window_s);
        if let Some(id) = self.last_action.as_ref().filter(|a| a.is_running()).map(|a| a.id) {
            self.finish_action(id, "failed", Some("COLLISION"), Some(message.clone()), None, None, None);
        }
        self.macro_run = None;
        // Hold commanded = measured; the gripper keeps its command so a held object stays held.
        let gripper_idx = self.config.gripper_index();
        let grip_cmd = gripper_idx.map(|g| self.commanded_positions[g]);
        self.stop_motion();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            for (i, (c, m)) in self.commanded_positions.iter_mut().zip(&self.measured_positions).enumerate() {
                let l = &self.config.limits[i];
                *c = m.clamp(l.min_position, l.max_position);
            }
        }
        if let (Some(g), Some(c)) = (gripper_idx, grip_cmd) { self.commanded_positions[g] = c; }
        self.target_positions.clone_from(&self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.servo_lead_stall.fill(None);
        self.safety.mark_motion_finished();
        if trips > p.max_auto_resumes {
            tracing::error!("Collision guard: {trips} trips within {:.0} s; latching Fault COLLISION", p.escalate_window_s);
            self.safety.trigger_fault_coded(FaultCode::Collision, FaultReason::Other(format!(
                "COLLISION: {message}; {trips} trips within {:.0} s", p.escalate_window_s)));
            return;
        }
        self.collision.hold = Some(CollisionHold { joint: j, residual_nm: residual, threshold_nm: threshold, since_s: now, quiet_s: 0.0 });
    }

    /// Apply a runtime setting (WS command / file). Enabling needs the guard to be available
    /// when `check` is set (the startup load runs before the driver is attached).
    fn set_collision(&mut self, enabled: Option<bool>, sensitivity: Option<Sensitivity>, check: bool) -> Result<(), ArmError> {
        if check && enabled == Some(true) && !self.collision_available() {
            return Err(MotionError::Unsupported(
                "The collision guard needs a profile gravity model and a driver that reports torque".into()).into());
        }
        if let Some(e) = enabled { self.collision.enabled = e; }
        if !self.collision.enabled {
            // Switching the guard off ends a hold at once (the lease holder decided).
            self.collision.hold = None;
            self.collision.over = [0; 6];
        }
        if let Some(s) = sensitivity { self.collision.sensitivity = s; }
        self.collision.runtime = true;
        Ok(())
    }
}

/// WS command `/api/v1/safety/collision` `{"enabled": bool?, "sensitivity": "low"|"medium"|"high"?}`.
/// Returns the new effective config and the file write to run after the lock is released.
pub fn update(inner: &mut SupervisorInner, body: &serde_json::Value) -> Result<(CollisionConfig, Option<PendingWrite>), ArmError> {
    #[derive(Deserialize)]
    #[serde(deny_unknown_fields)]
    struct Body { enabled: Option<bool>, sensitivity: Option<Sensitivity> }
    let b: Body = serde_json::from_value(body.clone()).map_err(|e| MotionError::InvalidCommand(format!(
        "collision body must be {{\"enabled\": bool?, \"sensitivity\": \"low\"|\"medium\"|\"high\"?}}: {e}")))?;
    if b.enabled.is_none() && b.sensitivity.is_none() {
        return Err(MotionError::InvalidCommand("collision body needs enabled and/or sensitivity".into()).into());
    }
    inner.set_collision(b.enabled, b.sensitivity, true)?;
    let setting = CollisionSetting { enabled: inner.collision.enabled, sensitivity: inner.collision.sensitivity };
    let write = inner.collision.path.clone().map(|path| PendingWrite::new(path, serde_json::to_string_pretty(&setting).unwrap_or_default()));
    Ok((inner.collision_config(), write))
}

/// Load the persisted runtime setting at startup (logs the outcome). An invalid file is
/// left untouched and the profile setting stays in force.
pub async fn load_into(supervisor: &ArmSupervisor, path: &Path) {
    let mut inner = supervisor.inner().write().await;
    inner.collision.path = Some(path.to_path_buf());
    match persist::load::<CollisionSetting>(path, |_| Ok(())) {
        Loaded::Loaded(s) => {
            tracing::info!("Collision guard setting loaded from {} (enabled {}, sensitivity {:?})", path.display(), s.enabled, s.sensitivity);
            let _ = inner.set_collision(Some(s.enabled), Some(s.sensitivity), false);
        }
        Loaded::Missing => {}
        Loaded::Invalid(e) => tracing::warn!("Ignoring collision guard file: {e}; using the profile setting"),
    }
}
