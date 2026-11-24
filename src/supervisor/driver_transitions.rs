//! Driver transitions of the supervisor: driver attachment, feedback intake, driver limits, torque enable/disable and fault clearing (track A).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

use crate::driver::handshake::Deadline;

/// Supervisor-side disable confirmation after the driver's own window: fresh valid feedback
/// and `EnableState::Disabled`, within this time.
pub const DISABLE_CONFIRM_WINDOW: Duration = Duration::from_millis(25);
/// Interval between services of [`DisableConfirm`] in the blocking `request_disable`.
pub const DISABLE_CONFIRM_SERVICE: Duration = Duration::from_millis(2);
/// Upper wall-clock bound for last-chance hardware disable attempts on panic or forced teardown.
pub const PANIC_DISABLE_TIMEOUT: Duration = Duration::from_millis(500);

/// Bounded stack-only validated driver limits probed read-only before `clear_check` and committed
/// only after `clear_check` passes.
#[derive(Debug, Clone, Copy)]
pub(crate) struct CandidateDriverLimits {
    dof: usize,
    min: [f32; crate::rt_core::mailbox::MAX_DOF],
    max: [f32; crate::rt_core::mailbox::MAX_DOF],
    vel: [f32; crate::rt_core::mailbox::MAX_DOF],
}

/// Bounded, inline error returned by [`SupervisorInner::request_disable_bounded`] (0 heap allocs).
pub(crate) struct RequestDisableError {
    pub(crate) code: &'static str,
    pub(crate) message: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
}

/// Result of a bounded last-chance hardware disable attempt on a panic or forced teardown path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PanicDisableOutcome {
    /// No hardware driver attached (driverless mock).
    NoDriver,
    /// Disable command sent and fresh hardware feedback confirmed `EnableState::Disabled`.
    Confirmed,
    /// Disable command sent (or driver lacks enable feedback), but confirmation was not observed within the bound.
    Unconfirmed,
    /// Owner lock acquisition or blocking driver call exceeded the time bound.
    TimedOut,
    /// Disable transmission or teardown raised an error or secondary panic.
    Failed(String),
}

impl PanicDisableOutcome {
    pub fn is_confirmed(&self) -> bool {
        matches!(self, Self::Confirmed | Self::NoDriver)
    }

    pub fn log(&self, context: &str, timeout: Duration) {
        match self {
            Self::NoDriver => {
                tracing::warn!("{context}: no hardware driver attached (driverless); torque-off confirmed");
            }
            Self::Confirmed => {
                tracing::warn!("{context}: bounded hardware torque disable CONFIRMED");
            }
            Self::Unconfirmed => {
                tracing::error!(
                    "{context}: bounded hardware torque disable sent but NOT CONFIRMED within {} ms; torque may still be ON",
                    timeout.as_millis()
                );
            }
            Self::TimedOut => {
                tracing::error!(
                    "{context}: bounded hardware torque disable TIMED OUT after {} ms; torque may still be ON",
                    timeout.as_millis()
                );
            }
            Self::Failed(err) => {
                tracing::error!(
                    "{context}: bounded hardware torque disable FAILED ({err}); torque may still be ON"
                );
            }
        }
    }
}

/// Runs `work` on the bus owner thread (strictly preserving single-bus-writer ownership) while
/// an independent deadline observer thread waits up to `timeout` without touching the bus or
/// driver. If `work` blocks past `timeout` (e.g. a driver implementing only blocking `set_torque`
/// or a stalled transport syscall), the observer fires `on_timeout` at the deadline and this
/// function returns `None` if `work` later unblocks.
pub fn run_with_panic_deadline_observer(
    timeout: Duration,
    work: impl FnOnce() -> PanicDisableOutcome,
    on_timeout: impl FnOnce() + Send + 'static,
) -> Option<PanicDisableOutcome> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        #[derive(Clone, Copy, PartialEq, Eq)]
        enum ObserverState {
            Running,
            Completed,
            TimedOut,
        }

        let state = Arc::new((std::sync::Mutex::new(ObserverState::Running), std::sync::Condvar::new()));
        let on_timeout_slot: Arc<std::sync::Mutex<Option<Box<dyn FnOnce() + Send>>>> =
            Arc::new(std::sync::Mutex::new(Some(Box::new(on_timeout))));
        let state_obs = Arc::clone(&state);
        let on_timeout_obs = Arc::clone(&on_timeout_slot);
        let deadline = Instant::now() + timeout;
        let observer = match std::thread::Builder::new()
            .name("armd-panic-watchdog".into())
            .spawn(move || {
                let mut g = state_obs.0.lock().unwrap_or_else(|e| e.into_inner());
                while *g == ObserverState::Running {
                    let rem = deadline.saturating_duration_since(Instant::now());
                    if rem.is_zero() {
                        *g = ObserverState::TimedOut;
                        drop(g);
                        if let Some(cb) = on_timeout_obs.lock().unwrap_or_else(|e| e.into_inner()).take() {
                            cb();
                        }
                        return;
                    }
                    let (next, res) = state_obs.1.wait_timeout(g, rem).unwrap_or_else(|e| e.into_inner());
                    g = next;
                    if res.timed_out() && *g == ObserverState::Running {
                        *g = ObserverState::TimedOut;
                        drop(g);
                        if let Some(cb) = on_timeout_obs.lock().unwrap_or_else(|e| e.into_inner()).take() {
                            cb();
                        }
                        return;
                    }
                }
            }) {
            Ok(jh) => jh,
            Err(e) => {
                // Never fall back to an unbounded inline driver call without a deadline observer.
                tracing::error!("Failed to spawn panic deadline observer ({e}); aborting without unbounded bus call");
                if let Some(cb) = on_timeout_slot.lock().unwrap_or_else(|e| e.into_inner()).take() {
                    cb();
                }
                return None;
            }
        };

        let outcome = work();
        let won = {
            let mut g = state.0.lock().unwrap_or_else(|e| e.into_inner());
            if *g == ObserverState::Running {
                *g = ObserverState::Completed;
                true
            } else {
                false
            }
        };
        if won {
            state.1.notify_all();
            let _ = observer.join();
            Some(outcome)
        } else {
            None
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (timeout, on_timeout);
        Some(work())
    }
}

fn format_joint_list(joints: &[usize]) -> String {
    joints.iter().map(|i| format!("J{}", i + 1)).collect::<Vec<_>>().join(", ")
}

/// Attempts bounded hardware torque disable directly through the bus owner's driver using
/// budgeted non-blocking driver operations. Within `timeout`, retries disable for every motor
/// not yet confirmed; a transient send error on one motor never stops attempts on the others
/// and never exits before the budget is spent. Logs which motors remain unconfirmed when the
/// budget expires. Callers on panic paths wrap this with [`run_with_panic_deadline_observer`]
/// to bound drivers whose fallback `begin_torque` or transport I/O may block inside a call.
pub fn bounded_panic_disable_driver(drv: &mut dyn ArmDriver, timeout: Duration) -> PanicDisableOutcome {
    let wall_deadline = Instant::now() + timeout;
    let enable_fb = drv.capabilities().enable_feedback;

    // Preferred per-motor selective retry path (e.g. Damiao CAN bus): sends disable to every
    // unconfirmed motor on each pass, never stops on a single motor's TX error, and keeps
    // retrying until all motors confirm or the budget is spent.
    let mut first_step = true;
    loop {
        let rem = wall_deadline.saturating_duration_since(Instant::now());
        let drv_deadline = drv.clock_now() + rem;
        let step_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drv.panic_disable_step(first_step, drv_deadline)
        }));
        match step_res {
            Err(_) => return PanicDisableOutcome::Failed("driver panicked during panic_disable_step".into()),
            Ok(None) => break, // Driver uses the generic retry loop below.
            Ok(Some(Ok(unconfirmed))) => {
                first_step = false;
                if !enable_fb {
                    return PanicDisableOutcome::Unconfirmed;
                }
                if unconfirmed.is_empty() && DisableConfirm::confirmed(drv) {
                    return PanicDisableOutcome::Confirmed;
                }
                let rem = wall_deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    let final_unconfirmed = drv.unconfirmed_disable_joints();
                    tracing::error!(
                        "Bounded panic disable budget ({} ms) exhausted; unconfirmed motors: [{}]",
                        timeout.as_millis(),
                        format_joint_list(&final_unconfirmed)
                    );
                    return PanicDisableOutcome::Unconfirmed;
                }
                std::thread::sleep(rem.min(DISABLE_CONFIRM_SERVICE));
            }
            Ok(Some(Err(e))) => {
                first_step = false;
                tracing::warn!("Panic disable step error (retrying within budget): {e}");
                let rem = wall_deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    let final_unconfirmed = drv.unconfirmed_disable_joints();
                    tracing::error!(
                        "Bounded panic disable budget ({} ms) exhausted after error ({e}); unconfirmed motors: [{}]",
                        timeout.as_millis(),
                        format_joint_list(&final_unconfirmed)
                    );
                    return PanicDisableOutcome::Unconfirmed;
                }
                std::thread::sleep(rem.min(DISABLE_CONFIRM_SERVICE));
            }
        }
    }

    // Generic retry loop for drivers that do not implement `panic_disable_step`: retry disable
    // sends and feedback confirmation until confirmed or `wall_deadline` is spent.
    let mut states_buf = Vec::new();
    let mut any_send_ok = false;
    let mut last_err: Option<String> = None;
    let mut attempt = 0u32;
    loop {
        attempt += 1;
        let begin_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drv.begin_torque_budgeted(false, 0)
        }));
        match begin_res {
            Err(_) => return PanicDisableOutcome::Failed("driver panicked during begin_torque(false)".into()),
            Ok(Err(e)) => {
                tracing::warn!("Panic disable begin_torque(false) attempt {attempt} failed (retrying within budget): {e}");
                last_err = Some(e.to_string());
            }
            Ok(Ok(mut progress)) => {
                any_send_ok = true;
                while matches!(progress, crate::driver::TorqueProgress::Pending { .. }) {
                    let rem = wall_deadline.saturating_duration_since(Instant::now());
                    if rem.is_zero() {
                        break;
                    }
                    let drv_deadline = drv.clock_now() + rem;
                    let poll_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        drv.poll_torque_budgeted(drv_deadline, 64)
                    }));
                    match poll_res {
                        Ok(Ok(p)) => progress = p,
                        Ok(Err(e)) => {
                            tracing::warn!("Panic disable poll_torque failed (retrying within budget): {e}");
                            last_err = Some(e.to_string());
                            break;
                        }
                        Err(_) => {
                            return PanicDisableOutcome::Failed(
                                "driver panicked during poll_torque(disable)".into(),
                            );
                        }
                    }
                    if matches!(progress, crate::driver::TorqueProgress::Pending { .. }) {
                        let rem = wall_deadline.saturating_duration_since(Instant::now());
                        if rem.is_zero() {
                            break;
                        }
                        std::thread::sleep(rem.min(DISABLE_CONFIRM_SERVICE));
                    }
                }
            }
        }

        if !enable_fb && any_send_ok {
            return PanicDisableOutcome::Unconfirmed;
        }
        if enable_fb && DisableConfirm::confirmed(drv) {
            return PanicDisableOutcome::Confirmed;
        }
        let rem = wall_deadline.saturating_duration_since(Instant::now());
        if rem.is_zero() {
            break;
        }
        let drv_deadline = drv.clock_now() + rem;
        let read_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drv.read_joint_states_budgeted(&mut states_buf, drv_deadline, 64)
        }));
        if read_res.is_err() {
            return PanicDisableOutcome::Failed("driver panicked while confirming disable".into());
        }
        if enable_fb && DisableConfirm::confirmed(drv) {
            return PanicDisableOutcome::Confirmed;
        }
        let rem = wall_deadline.saturating_duration_since(Instant::now());
        if rem.is_zero() {
            break;
        }
        std::thread::sleep(rem.min(DISABLE_CONFIRM_SERVICE));
    }

    if enable_fb && DisableConfirm::confirmed(drv) {
        return PanicDisableOutcome::Confirmed;
    }
    let unconfirmed = drv.unconfirmed_disable_joints();
    tracing::error!(
        "Bounded panic disable budget ({} ms) exhausted; unconfirmed motors: [{}]{}",
        timeout.as_millis(),
        format_joint_list(&unconfirmed),
        last_err.as_deref().map(|e| format!(" (last error: {e})")).unwrap_or_default()
    );
    if any_send_ok {
        PanicDisableOutcome::Unconfirmed
    } else {
        PanicDisableOutcome::Failed(last_err.unwrap_or_else(|| "panic disable send failed".into()))
    }
}

/// Result of one budgeted [`DisableConfirm::serve_budgeted`] call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DisableServe {
    pub outcome: Option<bool>,
    pub did_rx: bool,
    pub rx_frames: u16,
    pub budget_exhausted: bool,
}

/// Outcome of one [`SupervisorInner::service_torque_transition`] call, with the exact
/// RX work performed by this call (None when no bus receive pass ran).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TransitionPoll {
    pub progress: Result<crate::rt_core::ops::TorqueProgress, &'static str>,
    pub rx: Option<(u16, bool)>,
}

/// The supervisor's disable confirmation as an absolute-deadline wait (no sleep inside): served
/// once per call, it reads feedback once and reports `Some(confirmed)` when finished.
#[derive(Clone, Copy, Debug)]
pub struct DisableConfirm { deadline: Deadline }

impl DisableConfirm {
    pub fn start(now: Instant) -> Self { Self { deadline: Deadline::after(now, DISABLE_CONFIRM_WINDOW) } }

    /// End of the confirmation window (driver clock).
    pub fn deadline(&self) -> Instant { self.deadline.at }

    /// Fresh valid feedback reports every motor disabled.
    pub fn confirmed(drv: &dyn ArmDriver) -> bool { drv.is_feedback_valid() && drv.enable_state() == EnableState::Disabled }

    /// One service: past the deadline -> `Some(false)` without a read (unless already confirmed);
    /// else one receive pass, `Some(true)` once confirmed, `None` to serve again.
    pub fn serve(&mut self, drv: &mut dyn ArmDriver) -> Option<bool> {
        if self.deadline.passed(drv.clock_now()) { return Some(Self::confirmed(drv)); }
        let _ = drv.read_joint_states();
        Self::confirmed(drv).then_some(true)
    }

    /// Budgeted service for the RT executor: skips bus RX once the deadline has passed,
    /// otherwise performs a single budgeted receive pass into `buf`.
    pub fn serve_budgeted(
        &mut self,
        drv: &mut dyn ArmDriver,
        deadline: Instant,
        max_rx_frames: u16,
        buf: &mut Vec<crate::model::JointState>,
    ) -> DisableServe {
        if self.deadline.passed(drv.clock_now()) {
            return DisableServe {
                outcome: Some(Self::confirmed(drv)),
                did_rx: false,
                rx_frames: 0,
                budget_exhausted: false,
            };
        }
        let (rx_frames, budget_exhausted) = drv
            .read_joint_states_budgeted(buf, deadline, max_rx_frames)
            .unwrap_or_default();
        DisableServe {
            outcome: Self::confirmed(drv).then_some(true),
            did_rx: true,
            rx_frames,
            budget_exhausted,
        }
    }
}

/// A torque transition the RT started ([`crate::rt_core::ops::CoreOps::commit_transition`])
/// and serves once per [`crate::rt_core::ops::CoreOps::poll_hardware`]: the driver's
/// handshake (`ArmDriver::begin_torque` / `poll_torque`), then, for a disable, the
/// supervisor's own confirmation ([`DisableConfirm`]). Its outcome is applied to the
/// supervisor state when it ends (torque flags, safety state, fault on failure), exactly as
/// the blocking `set_torque` applies it.
#[derive(Debug, Clone, Copy)]
pub enum TorqueTransition {
    /// Waiting for the driver's enable confirmation.
    Enable { was_torqued: bool, prev_gripper_cmd: Option<f32> },
    /// Waiting for the driver's disable handshake (`confirm` None), then for fresh feedback
    /// reporting every motor disabled.
    Disable { confirm: Option<DisableConfirm> },
}

/// The terminal code of a refused or failed enable / disable step.
fn torque_error_code(e: &SafetyError) -> &'static str {
    match e {
        SafetyError::EStopActive => "ESTOP_ACTIVE",
        SafetyError::FeedbackUnavailable => "FEEDBACK_UNAVAILABLE",
        SafetyError::FaultActive(_) => "FAULT_ACTIVE",
        SafetyError::NotArmed(_) => "NOT_ARMED",
        SafetyError::TorqueOffWouldDrop { .. } => "TORQUE_OFF_WOULD_DROP",
        _ => "ENABLE_FAILED",
    }
}

impl SupervisorInner {
    pub fn with_driver(mut self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_abort_flag(self.e_stop_latched.clone());
        driver.set_estop_gate(self.estop_gate());
        driver.set_gripper_index(self.config.gripper_index());
        driver.set_strict_mode_handover(self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
        if driver.capabilities().load_feedback {
            self.loads = vec![f32::NAN; self.config.dof];
        }
        let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &self.joint_zero_offsets));
        self.apply_driver_limits(driver.as_ref());
        self.driver = Some(std::sync::Mutex::new(driver));
        self
    }

    pub fn sync_driver_feedback(
        &mut self,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        Self::apply_feedback(
            &mut self.measured_positions,
            &mut self.measured_velocities,
            &mut self.torques,
            &mut self.temperatures,
            &mut self.loads,
            self.config.dof,
            states,
            loads,
        );
    }

    pub(crate) fn apply_feedback(
        measured_positions: &mut [f32],
        measured_velocities: &mut [f32],
        torques: &mut [f32],
        temperatures: &mut [f32],
        loads_vec: &mut Vec<f32>,
        dof: usize,
        states: &[crate::model::JointState],
        loads: Option<&[f32]>,
    ) {
        for (i, state) in states.iter().enumerate().take(dof) {
            measured_positions[i] = state.position;
            measured_velocities[i] = state.velocity;
            torques[i] = state.torque;
            temperatures[i] = state.temperature;
        }
        if let Some(loads) = loads {
            if loads.len() == dof && loads.iter().all(|x| x.is_finite()) {
                copy_into(loads_vec, loads);
            }
        }
    }

    pub fn apply_driver_limits(&mut self, driver: &dyn ArmDriver) {
        let vel: Vec<_> = (0..self.config.dof).map(|i| driver.max_command_velocity(i)).collect();
        let pos: Vec<_> = (0..self.config.dof).map(|i| driver.position_limits(i)).collect();
        self.apply_extracted_driver_limits(&vel, &pos);
    }

    pub fn apply_extracted_driver_limits(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
    ) {
        self.apply_extracted_driver_limits_inner(vel_limits, pos_limits, true);
    }

    pub fn apply_extracted_driver_limits_inner(
        &mut self,
        vel_limits: &[Option<f32>],
        pos_limits: &[Option<(f32, f32)>],
        trigger_fault: bool,
    ) {
        self.driver_config_error = None;
        let mut candidate = self.config.clone();

        for (i, limit) in candidate.limits.iter_mut().enumerate() {
            if let Some(Some(max)) = vel_limits.get(i) {
                if max.is_finite() && *max > 0.0 {
                    limit.max_velocity = limit.max_velocity.min(*max);
                }
            }
            if let Some(Some((d_min, d_max))) = pos_limits.get(i).copied() {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' are invalid",
                        d_min, d_max, limit.name
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.latch(
                            crate::safety::FaultKind::JointLimit,
                            crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                buf.push_str(&reason);
                            }),
                        );
                    }
                    return;
                }
                let new_min = limit.min_position.max(d_min);
                let new_max = limit.max_position.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    let reason = format!(
                        "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                        d_min, d_max, limit.name, limit.min_position, limit.max_position
                    );
                    self.driver_config_error = Some(reason.clone());
                    if trigger_fault {
                        self.safety.latch(
                            crate::safety::FaultKind::JointLimit,
                            crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                buf.push_str(&reason);
                            }),
                        );
                    }
                    return;
                }
                if let Some(pos) = candidate.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if pos < new_min || pos > new_max {
                        let reason = format!(
                            "Driver position limits [{:.4}, {:.4}] for joint '{}' conflict with resting pose position {:.4}",
                            d_min, d_max, limit.name, pos
                        );
                        self.driver_config_error = Some(reason.clone());
                        if trigger_fault {
                            self.safety.latch(
                                crate::safety::FaultKind::JointLimit,
                                crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                                    buf.push_str(&reason);
                                }),
                            );
                        }
                        return;
                    }
                }
                if (new_min - limit.min_position).abs() > 1e-4 || (new_max - limit.max_position).abs() > 1e-4 {
                    tracing::info!(
                        "Intersected joint '{}' position limits with driver range: profile=[{:.4}, {:.4}], hw=[{:.4}, {:.4}] -> [{:.4}, {:.4}]",
                        limit.name, limit.min_position, limit.max_position, d_min, d_max, new_min, new_max
                    );
                }
                limit.min_position = new_min;
                limit.max_position = new_max;
            }
        }

        if candidate.validate().is_err()
            && self.base_gripper_policy.is_some()
            && (self.gripper_calib_offset.is_some() || self.gripper_calib_jaw_opening.is_some())
        {
            let mut base_test = candidate.clone();
            base_test.gripper_policy = self.base_gripper_policy.clone();
            if base_test.validate().is_ok() {
                tracing::warn!("Driver limits invalidated outdated calibrated gripper policy; reverting to base uncalibrated policy");
                self.gripper_calib_invalid = true;
                self.gripper_calib_offset = None;
                self.gripper_calib_jaw_opening = None;
                self.config.gripper_policy = self.base_gripper_policy.clone();
                candidate = base_test;
            }
        }

        if let Err(err) = candidate.validate() {
            let reason = format!("Driver limits violated config invariants: {err}");
            self.driver_config_error = Some(reason.clone());
            if trigger_fault {
                self.safety.latch(
                    crate::safety::FaultKind::JointLimit,
                    crate::safety::FaultDetail::formatted(FaultCode::JointLimit, |buf| {
                        buf.push_str(&reason);
                    }),
                );
            }
            return;
        }

        self.config = candidate;
    }

    pub fn reapply_driver_limits(&mut self) -> bool {
        const MAX_DOF: usize = crate::rt_core::mailbox::MAX_DOF;
        let dof = self.config.dof;
        if dof <= MAX_DOF {
            let mut vel = [None::<f32>; MAX_DOF];
            let mut pos = [None::<(f32, f32)>; MAX_DOF];
            let locked = self.driver.as_ref().is_some_and(|d| {
                d.lock().is_ok_and(|drv| {
                    for i in 0..dof {
                        vel[i] = drv.max_command_velocity(i);
                        pos[i] = drv.position_limits(i);
                    }
                    true
                })
            });
            if locked {
                self.apply_extracted_driver_limits_inner(&vel[..dof], &pos[..dof], false);
                self.driver_config_error.is_none()
            } else {
                false
            }
        } else {
            let limits = if let Some(ref d) = self.driver {
                if let Ok(drv) = d.lock() {
                    let vel: Vec<_> = (0..dof).map(|i| drv.max_command_velocity(i)).collect();
                    let pos: Vec<_> = (0..dof).map(|i| drv.position_limits(i)).collect();
                    Some((vel, pos))
                } else {
                    None
                }
            } else {
                None
            };
            if let Some((vel_limits, pos_limits)) = limits {
                self.apply_extracted_driver_limits_inner(&vel_limits, &pos_limits, false);
                self.driver_config_error.is_none()
            } else {
                false
            }
        }
    }

    /// Pure read-only validation of driver position/velocity limits into bounded stack storage
    /// (zero allocations, zero mutations to `self`).
    #[allow(clippy::result_large_err)] // inline bounded text avoids RT heap allocation
    pub(crate) fn probe_driver_limits_bounded(
        &self,
    ) -> Result<CandidateDriverLimits, crate::safety::BoundedText<256>> {
        use crate::safety::BoundedText;
        const MAX_DOF: usize = crate::rt_core::mailbox::MAX_DOF;
        let dof = self.config.dof;
        if dof > MAX_DOF || self.config.limits.len() != dof || self.config.home_positions.len() != dof {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver limit retry exceeds bounded RT DOF")));
        }
        let mut vel_limits = [None::<f32>; MAX_DOF];
        let mut pos_limits = [None::<(f32, f32)>; MAX_DOF];
        let locked = self.driver.as_ref().is_some_and(|d| {
            d.lock().is_ok_and(|drv| {
                for i in 0..dof {
                    vel_limits[i] = drv.max_command_velocity(i);
                    pos_limits[i] = drv.position_limits(i);
                }
                true
            })
        });
        if !locked {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver unavailable while revalidating limits")));
        }

        let mut cand_min = [0.0f32; MAX_DOF];
        let mut cand_max = [0.0f32; MAX_DOF];
        let mut cand_vel = [0.0f32; MAX_DOF];
        for i in 0..dof {
            let limit = &self.config.limits[i];
            let mut new_vel = limit.max_velocity;
            if let Some(max) = vel_limits[i] {
                if max.is_finite() && max > 0.0 {
                    new_vel = new_vel.min(max);
                }
            }
            if !new_vel.is_finite() || new_vel <= 0.0 {
                let mut msg = BoundedText::<256>::empty();
                let _ = core::fmt::write(&mut msg, format_args!("Driver limits violated config invariants: Joint {i} max_velocity must be positive finite"));
                return Err(msg);
            }
            let mut new_min = limit.min_position;
            let mut new_max = limit.max_position;
            if let Some((d_min, d_max)) = pos_limits[i] {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    let mut msg = BoundedText::<256>::empty();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!("Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' are invalid", limit.name),
                    );
                    return Err(msg);
                }
                new_min = new_min.max(d_min);
                new_max = new_max.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    let mut msg = BoundedText::<256>::empty();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!(
                            "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                            limit.name, limit.min_position, limit.max_position
                        ),
                    );
                    return Err(msg);
                }
                if let Some(pos) = self.config.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if !pos.is_finite() || pos < new_min || pos > new_max {
                        let mut msg = BoundedText::<256>::empty();
                        let _ = core::fmt::write(
                            &mut msg,
                            format_args!(
                                "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with resting pose position {pos:.4}",
                                limit.name
                            ),
                        );
                        return Err(msg);
                    }
                }
            }
            let home = self.config.home_positions[i];
            if !home.is_finite() || home < new_min || home > new_max {
                return Err(BoundedText::new(
                    "Driver limits violated config invariants: Home positions must be finite and within joint limits",
                ));
            }
            cand_min[i] = new_min;
            cand_max[i] = new_max;
            cand_vel[i] = new_vel;
        }
        if let (Some(g), Some(dg)) = (self.config.gripper_index(), &self.config.damiao_gripper) {
            if dg.mode == crate::model::DamiaoGripperMode::PosForce
                && crate::model::DamiaoGripperDrive::quantize_vel_raw(cand_vel[g]) == 0
            {
                let mut msg = BoundedText::<256>::empty();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!("Driver limits violated config invariants: Joint {g} max_velocity encodes to 0 on the pos-force wire (must quantize to >= 0.01 rad/s)"),
                );
                return Err(msg);
            }
        }
        if let Some(gp) = &self.config.gripper_policy {
            let ok = self.config.gripper_index().is_some_and(|g_idx| {
                let (lo, hi) = (cand_min[g_idx], cand_max[g_idx]);
                let in_lim = |v: f32| v.is_finite() && v >= lo && v <= hi;
                in_lim(gp.closed_rad)
                    && in_lim(gp.open_rad)
                    && gp.jaw_opening.as_ref().is_none_or(|jo| jo.points.iter().all(|&[q, _]| in_lim(q)))
            });
            if !ok {
                return Err(BoundedText::new(
                    "Driver limits violated config invariants: Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)",
                ));
            }
        }
        if self.driver_config_error.is_some() && self.retired_driver_config_error.is_some() {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver config error reclamation pending")));
        }

        Ok(CandidateDriverLimits {
            dof,
            min: cand_min,
            max: cand_max,
            vel: cand_vel,
        })
    }

    /// Commits validated candidate driver limits in place and retires any previous error string
    /// off-thread (zero heap ops; invoked ONLY after `clear_check` passes).
    pub(crate) fn commit_driver_limits_bounded(&mut self, cand: &CandidateDriverLimits) {
        for i in 0..cand.dof {
            self.config.limits[i].min_position = cand.min[i];
            self.config.limits[i].max_position = cand.max[i];
            self.config.limits[i].max_velocity = cand.vel[i];
        }
        if let Some(prev) = self.driver_config_error.take() {
            if self.retired_driver_config_error.is_none() {
                self.retired_driver_config_error = Some(prev);
            } else {
                self.driver_config_error = Some(prev);
            }
        }
    }

    /// Pure read-only check of `positions` against either validated candidate limits (if
    /// `driver_config_error` just resolved) or current `self.config.limits` (zero heap allocations).
    fn joint_limit_clear_violation_for_positions(
        &self,
        positions: &[f32],
        cand: Option<&CandidateDriverLimits>,
    ) -> Option<crate::safety::JointLimitClearViolation> {
        use crate::safety::{BoundedText, JointLimitClearViolation as V};
        let dof = cand.map_or(self.config.dof, |c| c.dof);
        if positions.len() != dof || self.config.limits.len() != dof {
            return Some(V::FeedbackIncomplete {
                measured_len: positions.len(),
                dof,
            });
        }
        for i in 0..dof {
            let q = positions[i];
            let joint_name = BoundedText::new(&self.config.limits[i].name);
            if !q.is_finite() {
                return Some(V::NonFinite {
                    joint_idx: i as u16,
                    joint_name,
                    value: q,
                });
            }
            let (min, max) = match cand {
                Some(c) => (c.min[i], c.max[i]),
                None => (self.config.limits[i].min_position, self.config.limits[i].max_position),
            };
            if q < min - JOINT_LIMIT_CLEAR_MARGIN_RAD || q > max + JOINT_LIMIT_CLEAR_MARGIN_RAD {
                return Some(V::OutOfBounds {
                    joint_idx: i as u16,
                    joint_name,
                    value: q,
                    min,
                    max,
                    margin: JOINT_LIMIT_CLEAR_MARGIN_RAD,
                });
            }
        }
        None
    }

    /// Pure read-only check of measured positions against either validated candidate limits (if
    /// `driver_config_error` just resolved) or current `self.config.limits` (zero heap allocations).
    fn joint_limit_clear_violation_with_candidates(
        &self,
        cand: Option<&CandidateDriverLimits>,
    ) -> Option<crate::safety::JointLimitClearViolation> {
        self.joint_limit_clear_violation_for_positions(&self.measured_positions, cand)
    }

    /// Bounded, zero-allocation retry of driver limits for the RT executor `clear_fault` path.
    #[allow(dead_code)]
    pub(crate) fn reapply_driver_limits_bounded(&mut self) -> Option<crate::safety::BoundedText<256>> {
        match self.probe_driver_limits_bounded() {
            Ok(cand) => {
                self.commit_driver_limits_bounded(&cand);
                None
            }
            Err(err) => Some(err),
        }
    }

    /// Store the motor error codes, logging each per-joint change once.
    pub(super) fn update_motor_errors(&mut self, errors: Vec<Option<crate::model::MotorError>>) {
        if errors == self.motor_errors { return; }
        for (i, e) in errors.iter().enumerate() {
            let prev = self.motor_errors.get(i).cloned().flatten();
            match (prev, e) {
                (p, Some(e)) if p.as_ref() != Some(e) =>
                    tracing::warn!("Joint {} motor reports error code 0x{:X} ({})", i + 1, e.code, e.name),
                (Some(p), None) => tracing::info!("Joint {} motor error 0x{:X} ({}) cleared", i + 1, p.code, p.name),
                _ => {}
            }
        }
        self.motor_errors = errors;
    }

    pub(crate) fn states_valid(
        states: &[crate::model::JointState],
        caps: DriverCapabilities,
        dof: usize,
        loads: Option<&[f32]>,
    ) -> bool {
        states.len() == dof
            && states.iter().all(|s| crate::driver::joint_state_valid(s, caps))
            && (!caps.load_feedback
                || loads.is_some_and(|l| l.len() == dof && l.iter().all(|x| x.is_finite())))
    }

    /// Per-joint subset numeric validity (e.g., J1..J6 arm joints excluding the gripper).
    #[allow(dead_code)]
    pub(crate) fn states_valid_joints(
        states: &[crate::model::JointState],
        caps: DriverCapabilities,
        joints: &[usize],
        loads: Option<&[f32]>,
    ) -> bool {
        !joints.is_empty()
            && joints.iter().all(|&i| {
                states.get(i).is_some_and(|s| crate::driver::joint_state_valid(s, caps))
                    && (!caps.load_feedback
                        || loads.and_then(|l| l.get(i)).is_some_and(|x| x.is_finite()))
            })
    }

    pub fn hardware_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        // Ages alone decide: drivers' is_feedback_valid() hard-codes 250 ms,
        // which would silently cap callers asking for a longer bound.
        self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.feedback_all_within(self.config.dof, max_age)))
    }

    /// Per arm joint, an instant before which its last sample cannot have been enqueued: the
    /// driver's receive-queue bound, or the receive instant for drivers without a receive queue.
    pub fn arm_feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.config.arm_joint_indices().iter().map(|&i| match self.joint_feedback_queue_bounds.get(i) {
            Some(b) => *b,
            None => self.core.feedback().instant(i),
        }).collect()
    }

    /// Snapshot of `(measured_positions, arm_velocities, arm_feedback_instants, arm_feedback_seqs)`
    /// read coherently under the supervisor lock.
    pub fn arm_feedback_snapshot(&self) -> (Vec<f32>, Vec<f32>, Vec<Option<Instant>>, Vec<u64>) {
        let arm = self.config.arm_joint_indices();
        let vels = arm.iter().map(|&i| self.measured_velocities.get(i).copied().unwrap_or(0.0)).collect();
        let instants = arm
            .iter()
            .map(|&i| self.core.feedback().instant(i))
            .collect();
        let seqs = arm
            .iter()
            .map(|&i| self.core.feedback().seq(i))
            .collect();
        (self.measured_positions.clone(), vels, instants, seqs)
    }

    /// Bounded, zero-allocation error from [`Self::request_disable_bounded`].
    #[allow(clippy::result_large_err)] // inline bounded error keeps RT disable retry zero-alloc
    pub(crate) fn request_disable_bounded(&mut self) -> Result<(), RequestDisableError> {
        // a blocking disable (park, thermal trip, fault path) replaces a transition in flight
        // (the driver's disable replaces its pending enable): that transition ends SUPERSEDED
        if self.torque_transition.take().is_some() { self.core.begin_transition(Err("SUPERSEDED")); }
        self.disable_requested();
        if let Some(ref d) = self.driver {
            let mut drv = d.lock().map_err(|_| RequestDisableError {
                code: "FEEDBACK_UNAVAILABLE",
                message: crate::safety::BoundedText::new("Feedback unavailable"),
            })?;
            if self.rt_executor_owned {
                let res = drv.begin_torque_budgeted(false, 0).map_err(|e| {
                    let mut message = crate::safety::BoundedText::empty();
                    message.push_str("Torque disable transmission failed: ");
                    e.write_bounded(&mut message);
                    let code = crate::driver::handshake::rt_progress(false, Err(e))
                        .err()
                        .unwrap_or("HARDWARE_FAULT");
                    RequestDisableError { code, message }
                })?;
                let confirmed_now = matches!(res, crate::driver::TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed))
                    && DisableConfirm::confirmed(&**drv);
                drop(drv);
                if confirmed_now {
                    self.disable_confirmed();
                } else {
                    self.is_torqued = true;
                }
                return Ok(());
            }
            drv.set_torque(false).map_err(|e| {
                let mut message = crate::safety::BoundedText::empty();
                message.push_str("Torque disable transmission failed: ");
                e.write_bounded(&mut message);
                let code = crate::driver::handshake::rt_progress(false, Err(e))
                    .err()
                    .unwrap_or("HARDWARE_FAULT");
                RequestDisableError { code, message }
            })?;
            let mut confirm = DisableConfirm::start(drv.clock_now());
            while confirm.serve(&mut **drv).is_none() {
                drv.clock_pause(DISABLE_CONFIRM_SERVICE);
            }
            if !DisableConfirm::confirmed(&**drv) {
                return Err(RequestDisableError {
                    code: "FEEDBACK_UNCONFIRMED",
                    message: crate::safety::BoundedText::new(
                        "Torque disable has not been confirmed by fresh hardware feedback",
                    ),
                });
            }
        }
        self.disable_confirmed();
        Ok(())
    }

    /// A successful transport write is not confirmation that torque was removed.
    pub(super) fn request_disable(&mut self) -> Result<(), SafetyError> {
        self.request_disable_bounded().map_err(|e| {
            if e.code == "FEEDBACK_UNAVAILABLE" {
                SafetyError::FeedbackUnavailable
            } else {
                SafetyError::ClearFaultRejected(e.message.as_str().to_string())
            }
        })
    }

    /// State at the first disable frame: MIT off, `disable_pending` until confirmed.
    fn disable_requested(&mut self) {
        self.mit_torque_off();
        self.disable_pending = true;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.last_disable_attempt = Some(Instant::now());
    }

    /// State once fresh feedback confirmed the disable.
    pub(super) fn disable_confirmed(&mut self) {
        self.is_torqued = false;
        self.pending_rearm = false;
        self.disable_pending = false;
        self.gripper_hold_vel_ceiling = None;
    }

    pub fn set_torque(&mut self, enable: bool) -> Result<SafetyState, SafetyError> {
        if self.torque_transition.is_some() {
            return Err(SafetyError::ClearFaultRejected("A torque transition is pending".into()));
        }
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
        if enable {
            let (was_torqued, prev_gripper_cmd) = self.enable_prepare()?;
            if let Some(ref d) = self.driver {
                let r = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?.set_torque(true);
                if let Err(e) = r { return Err(self.enable_failed(was_torqued, &e)); }
            }
            self.enable_finish(was_torqued, prev_gripper_cmd)
        } else {
            self.torque_off(false)
        }
    }

    /// Enable, before the driver's handshake: every refusal (gravity compensation, driver
    /// limits, handover, pending disable, E-stop, fault, MIT gate), then the feedback read
    /// that seeds the commanded state. Returns (torque was on, gripper command to keep).
    fn enable_prepare(&mut self) -> Result<(bool, Option<f32>), SafetyError> {
        self.enable_prepare_inner(true)
    }

    fn enable_prepare_inner(&mut self, allow_bus_read: bool) -> Result<(bool, Option<f32>), SafetyError> {
        if self.shutdown_committed {
            return Err(SafetyError::EStopActive);
        }
        if let Some(ref err) = self.driver_config_error {
            return Err(SafetyError::MotionForbiddenInFaultState(format!("Driver limit configuration error: {err}")));
        }
        if self.handover_settling_since.is_some() {
            return Err(SafetyError::HandoverSettling);
        }
        if self.disable_pending { return Err(SafetyError::ClearFaultRejected("Torque disable is still pending".into())); }
        // Atomic e-stop check
        if self.estop_pending() {
            return Err(SafetyError::EStopActive);
        }

        // Strict pre-check: If stopped (E-Stop or Fault), reject IMMEDIATELY before touching driver!
        if self.safety.state().is_stopped() {
            if let SafetyState::Fault(ref reason) = self.safety.state() {
                return Err(SafetyError::FaultActive(reason.clone()));
            }
            return Err(SafetyError::EStopActive);
        }

        // RT executor step 1 does not support synchronous MIT register qualification/handover or
        // gravity-compensation mode switches; fail closed before touching any sync driver helpers.
        if (!allow_bus_read || self.rt_executor_owned)
            && (self.gravity_comp.is_some()
                || self.mit_active()
                || self.mit.revoke_pending
                || !crate::mit::mit_joints(&self.config).is_empty())
        {
            return Err(SafetyError::ClearFaultRejected(
                "UNSUPPORTED: MIT mode qualification/handover and gravity compensation are not supported under synchronous RT executor step 1".into(),
            ));
        }

        // MIT tracking (Phase 2): every arm and rearm rechecks the qualification against a fresh
        // register readback (nonzero TIMEOUT, identity, ranges); unqualified hardware refuses
        // MIT before any motion. MIT not requested: no readback, no change.
        // A rearm while MIT owns joints hands them back first (at rest), so neither the
        // register readback below nor the driver's enable runs under an active MIT owner
        // (no dynamic output during the reads, no silent mode restore).
        if self.mit_active() {
            self.mit.armed_ok = false;
            if let Err(e) = self.mit_hand_back("rearm") {
                // not at rest: the authorization is gone, so the motion brakes to rest and MIT
                // hands back there (the tick); no motion starts meanwhile
                if self.mit_active() {
                    self.active_trajectory = None;
                    self.servo_target = None;
                    self.mit.revoke_pending = true;
                }
                return Err(SafetyError::ClearFaultRejected(e));
            }
        }
        if let Err(e) = self.mit_gate() {
            // revoked before returning: no handover until a later arm passes the gate
            self.mit.armed_ok = false;
            let msg = format!("MIT_UNQUALIFIED: MIT tracking refused: {e}");
            tracing::error!("{msg}");
            self.mit_gate_error = Some(msg.clone());
            return Err(SafetyError::ClearFaultRejected(msg));
        }
        self.mit_gate_error = None;

        // Enable hardware torque so bus devices wake up and publish state, then verify physical feedback
        let any_driver_enabled = self.driver.as_ref().is_some_and(|d| d.lock().is_ok_and(|drv| drv.has_any_joint_enabled()));
        let was_torqued = self.is_torqued || any_driver_enabled;
        let prev_gripper_cmd = if was_torqued {
            self.config.gripper_index().and_then(|i| self.commanded_positions.get(i).copied())
        } else {
            None
        };

        if let Some(ref d) = self.driver {
            // Secondary double-check latch right before hardware actuation
            if self.estop_pending() {
                return Err(SafetyError::EStopActive);
            }
            let mut driver = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
            if self.estop_pending() {
                return Err(SafetyError::EStopActive);
            }
            if allow_bus_read && !driver.is_feedback_valid() {
                if let Ok(states) = driver.read_joint_states() {
                    let loads = driver.joint_loads();
                    if !states.is_empty() && driver.is_feedback_valid() && SupervisorInner::states_valid(&states, driver.capabilities(), self.config.dof, loads.as_deref()) {
                        Self::apply_feedback(
                            &mut self.measured_positions,
                            &mut self.measured_velocities,
                            &mut self.torques,
                            &mut self.temperatures,
                            &mut self.loads,
                            self.config.dof,
                            &states,
                            loads.as_deref(),
                        );
                        for (i, s) in states.iter().enumerate().take(self.config.dof) {
                            if s.position.is_finite() {
                                self.commanded_positions[i] = s.position;
                                self.target_positions[i] = s.position;
                            }
                        }
                    }
                }
            }
        }
        Ok((was_torqued, prev_gripper_cmd))
    }

    /// The driver's enable failed with `e`: an arm that was not torqued is disabled again
    /// (confirmed or left `disable_pending`), and the fault `ENABLE_FAILED` latches.
    fn enable_failed(&mut self, was_torqued: bool, e: &dyn std::fmt::Display) -> SafetyError {
        if !was_torqued {
            if let Some(ref d) = self.driver {
                if let Ok(mut driver) = d.lock() {
                    let sent = driver.set_torque(false).is_ok();
                    let _ = driver.read_joint_states();
                    let confirmed_off = sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled;
                    self.is_torqued = !confirmed_off;
                    self.disable_pending = !confirmed_off;
                    self.last_disable_attempt = Some(Instant::now());
                }
            }
        }
        let err_msg = format!("Hardware torque enable failed: {}", e);
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::formatted(FaultCode::EnableFailed, |buf| {
                buf.push_str(&err_msg);
            }),
        );
        SafetyError::ClearFaultRejected(err_msg)
    }

    /// Non-blocking enable failure handler for RT transitions: sends disable frames with zero RX
    /// and zero sleep when the arm was not previously torqued, leaving confirmation to Step 2.
    fn enable_failed_budgeted(&mut self, was_torqued: bool, e: &dyn std::fmt::Display) -> SafetyError {
        if !was_torqued {
            let disable_outcome = self.driver.as_ref().and_then(|d| {
                d.lock().ok().map(|mut driver| {
                    driver.abort_enable();
                    let sent = driver.begin_torque_budgeted(false, 0);
                    matches!(
                        sent,
                        Ok(crate::driver::TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed))
                    ) && driver.is_feedback_valid()
                        && driver.enable_state() == EnableState::Disabled
                })
            });
            if let Some(confirmed_off) = disable_outcome {
                if confirmed_off {
                    self.disable_confirmed();
                } else {
                    self.is_torqued = true;
                    self.disable_requested();
                }
            }
        }
        let err_msg = format!("Hardware torque enable failed: {}", e);
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::formatted(FaultCode::EnableFailed, |buf| {
                buf.push_str(&err_msg);
            }),
        );
        SafetyError::ClearFaultRejected(err_msg)
    }

    /// Enable, after the driver confirmed it: validates fresh feedback (a failure disables
    /// an arm that was not torqued again), seeds commanded / target / tracking state from it,
    /// and records torque on.
    fn enable_finish(&mut self, was_torqued: bool, prev_gripper_cmd: Option<f32>) -> Result<SafetyState, SafetyError> {
        self.enable_finish_inner(was_torqued, prev_gripper_cmd, false)
    }

    fn enable_finish_inner(&mut self, was_torqued: bool, prev_gripper_cmd: Option<f32>, use_cached: bool) -> Result<SafetyState, SafetyError> {
        if let Some(ref d) = self.driver {
            let mut scratch_states = std::mem::take(&mut self.scratch.states);
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_loads = std::mem::take(&mut self.scratch.loads);
            let mut driver = match d.lock() {
                Ok(g) => g,
                Err(_) => {
                    self.scratch.states = scratch_states;
                    self.scratch.ages = scratch_ages;
                    self.scratch.loads = scratch_loads;
                    return Err(SafetyError::FeedbackUnavailable);
                }
            };
            let read_res = if use_cached {
                if driver.read_cached_joint_states_into(&mut scratch_states).is_some() {
                    Ok(())
                } else {
                    driver.read_joint_states_into(&mut scratch_states)
                }
            } else {
                driver.read_joint_states_into(&mut scratch_states)
            };
            if read_res.is_err() {
                let disable_outcome = if !was_torqued {
                    if use_cached {
                        let sent = driver.begin_torque_budgeted(false, 0);
                        Some(
                            matches!(
                                sent,
                                Ok(crate::driver::TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed))
                            ) && driver.is_feedback_valid()
                                && driver.enable_state() == EnableState::Disabled,
                        )
                    } else {
                        let sent = driver.set_torque(false).is_ok();
                        let _ = driver.read_joint_states();
                        Some(sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled)
                    }
                } else {
                    None
                };
                drop(driver);
                self.scratch.states = scratch_states;
                self.scratch.ages = scratch_ages;
                self.scratch.loads = scratch_loads;
                if let Some(confirmed_off) = disable_outcome {
                    if use_cached {
                        if confirmed_off {
                            self.disable_confirmed();
                        } else {
                            self.is_torqued = true;
                            self.disable_requested();
                        }
                    } else {
                        self.is_torqued = !confirmed_off;
                        self.disable_pending = !confirmed_off;
                        self.last_disable_attempt = Some(Instant::now());
                    }
                }
                return Err(SafetyError::FeedbackUnavailable);
            }
            let has_loads = driver.joint_loads_into(&mut scratch_loads);
            let loads_slice = has_loads.then_some(scratch_loads.as_slice());
            if !driver.is_feedback_valid()
                || !Self::states_valid(&scratch_states, driver.capabilities(), self.config.dof, loads_slice)
                || (driver.capabilities().enable_feedback && driver.enable_state() != EnableState::Enabled)
            {
                let disable_outcome = if !was_torqued {
                    if use_cached {
                        let sent = driver.begin_torque_budgeted(false, 0);
                        Some(
                            matches!(
                                sent,
                                Ok(crate::driver::TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed))
                            ) && driver.is_feedback_valid()
                                && driver.enable_state() == EnableState::Disabled,
                        )
                    } else {
                        let sent = driver.set_torque(false).is_ok();
                        let _ = driver.read_joint_states();
                        Some(sent && driver.is_feedback_valid() && driver.enable_state() == EnableState::Disabled)
                    }
                } else {
                    None
                };
                drop(driver);
                self.scratch.states = scratch_states;
                self.scratch.ages = scratch_ages;
                self.scratch.loads = scratch_loads;
                if let Some(confirmed_off) = disable_outcome {
                    if use_cached {
                        if confirmed_off {
                            self.disable_confirmed();
                        } else {
                            self.is_torqued = true;
                            self.disable_requested();
                        }
                    } else {
                        self.is_torqued = !confirmed_off;
                        self.disable_pending = !confirmed_off;
                        self.last_disable_attempt = Some(Instant::now());
                    }
                }
                return Err(SafetyError::FeedbackUnavailable);
            }
            driver.feedback_ages_into(&mut scratch_ages);
            let oldest_age = if scratch_ages.iter().all(Option::is_some) {
                scratch_ages.iter().flatten().copied().max()
            } else {
                None
            };
            self.feedback_numeric_valid = true;
            self.last_can_feedback_instant = oldest_age.and_then(|age| Instant::now().checked_sub(age));
            self.core.poll_feedback(&mut self.joint_feedback_queue_bounds, self.config.dof, driver.as_ref(), &mut self.scratch.fb);
            Self::apply_feedback(
                &mut self.measured_positions,
                &mut self.measured_velocities,
                &mut self.torques,
                &mut self.temperatures,
                &mut self.loads,
                self.config.dof,
                &scratch_states,
                loads_slice,
            );
            drop(driver);
            self.scratch.states = scratch_states;
            self.scratch.ages = scratch_ages;
            self.scratch.loads = scratch_loads;
        }

        // Initialize commanded_positions to verified measured_positions to prevent sudden jumps
        copy_into(&mut self.commanded_positions, &self.measured_positions);
        copy_into(&mut self.target_positions, &self.measured_positions);
        // Preserve gripper holding position when re-arming an already torqued arm so grasped objects are not released
        if let Some(g_cmd) = prev_gripper_cmd {
            if let Some(grip_idx) = self.config.gripper_index() {
                self.commanded_positions[grip_idx] = g_cmd;
                self.target_positions[grip_idx] = g_cmd;
            }
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        if self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled) {
            let arm_indices = self.config.arm_joints();
            let has_vel_fb = self.driver.as_ref().is_none_or(|d| {
                d.lock().is_ok_and(|drv| drv.capabilities().velocity_feedback)
            });
            // Positive evidence only: without velocity feedback a single sample cannot show
            // rest; the history stays Unknown until two samples do (plain output meanwhile).
            let arm_stationary = has_vel_fb
                && arm_indices.iter().all(|&j| {
                    self.measured_velocities
                        .get(j)
                        .is_some_and(|v| v.is_finite() && v.abs() <= crate::tracking::STATIONARY_VELOCITY_RAD_S)
                });
            let aligned_with_issued = !self
                .tracking
                .has_residual_lead(&self.commanded_positions, &arm_indices);
            if was_torqued && self.tracking.last_issued_positions.is_some() {
                if arm_stationary && aligned_with_issued {
                    self.tracking.seed(&self.commanded_positions);
                } else {
                    // Already torqued and moving or holding active lead: preserve real
                    // issued history so the next tick ramps smoothly instead of jumping.
                    self.tracking.ws_min_depths = None;
                }
            } else if arm_stationary {
                self.tracking.seed(&self.commanded_positions);
            } else {
                self.tracking.invalidate();
            }
        } else {
            self.tracking.invalidate();
        }
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        self.active_trajectory = None;
        self.pending_normal_unload = false;
        self.unload_settle = None;
        self.watchdog_park = None;
        self.servo_target = None;
        self.servo_stream_active = false;
        self.servo_ws_min_depths = None;
        self.gripper_hold_vel_ceiling = None;

        if self.estop_pending() || self.safety.state().is_stopped() {
            // Hardware confirmed enabled right as E-stop/fault arrived: preserve honest physical
            // torque state so hold output is not suppressed and shutdown stays gated.
            self.is_torqued = true;
            self.pending_rearm = true;
            return Err(match self.safety.state() {
                SafetyState::Fault(reason) if !self.estop_pending() => SafetyError::FaultActive(reason.clone()),
                _ => SafetyError::EStopActive,
            });
        }
        let state = self.safety.set_torque(true)?;
        self.is_torqued = true;
        self.pending_rearm = false;
        self.disable_pending = false;
        self.mit.armed_ok = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        self.mit.handover_error = None;
        let now = Instant::now();
        self.last_heartbeat = now;
        self.last_motion_frame = now;
        Ok(state)
    }

    /// Torque off at the resting pose. `stillness_verified` is set by the unload settle,
    /// which judged stationarity over its hold window; otherwise the current velocity
    /// sample must be within the resting-pose limit.
    pub(super) fn torque_off(&mut self, stillness_verified: bool) -> Result<SafetyState, SafetyError> {
        self.torque_off_in_place(stillness_verified)?;
        Ok(self.safety.state().clone())
    }

    /// True when E-stop, a Fault state, any latched fault kind bit, or a pending E-stop
    /// keeps the arm stopped: torque-off must apply Rule A′ and must never clear or mutate
    /// the fault record or epoch.
    #[inline]
    fn torque_off_stopped(&self) -> bool {
        self.safety.state().is_stopped()
            || self.safety.has_latched_fault()
            || self.estop_pending()
    }

    pub(crate) fn torque_off_rt_immediate(&mut self) -> Result<(), &'static str> {
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        self.gravity_comp_servo_busy()
            .map_err(|e| crate::rt_core::ops::static_code(crate::supervisor::ArmError::Safety(e)))?;
        self.torque_off_check_rt()?;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable_bounded() {
            if !self.torque_off_stopped() {
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                        buf.push_str(error.message.as_str());
                    }),
                );
            }
            return Err(error.code);
        }
        self.torque_off_finish_in_place()
            .map_err(|e| crate::rt_core::ops::static_code(crate::supervisor::ArmError::Safety(e)))
    }

    fn torque_off_in_place(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        self.torque_off_check(stillness_verified)?;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable() {
            if !self.torque_off_stopped() {
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                        let _ = core::fmt::write(buf, format_args!("{error}"));
                    }),
                );
            }
            return Err(error);
        }
        self.torque_off_finish_in_place()
    }

    /// Zero-allocation stopped torque-off gate for the RT executor (evaluates
    /// [`Self::may_disable_torque_while_stopped`] once); non-stopped paths fall through to
    /// [`Self::torque_off_check`].
    fn torque_off_check_rt(&mut self) -> Result<(), &'static str> {
        if self.torque_off_stopped() {
            return self
                .eval_stopped_torque_off()
                .map_err(|_| "TORQUE_OFF_WOULD_DROP");
        }
        self.torque_off_check(false)
            .map_err(|e| crate::rt_core::ops::static_code(crate::supervisor::ArmError::Safety(e)))
    }

    /// The refusals of a torque off: E-stop / fault (rest-pose gated), not at the resting pose.
    fn torque_off_check(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        if self.torque_off_stopped() {
            return self
                .eval_stopped_torque_off()
                .map_err(|r| r.into_safety_error(&self.config.joint_names));
        }

        let violation = if stillness_verified { self.resting_pose_static_violation() } else { self.resting_pose_violation() };
        if let Some(reason) = violation {
            tracing::warn!("Torque off refused, not at resting pose: {reason}");
            return Err(SafetyError::ArmNotAtRestingPose {
                current: self.measured_positions.clone(),
                expected: self.config.resting_positions().to_vec(),
                reason: Some(reason),
            });
        }
        Ok(())
    }

    /// Torque off after the confirmed disable without cloning `SafetyState` (zero-alloc on RT).
    fn torque_off_finish_in_place(&mut self) -> Result<(), SafetyError> {
        if !self.torque_off_stopped() {
            let _ = self.safety.set_torque(false)?;
        }
        self.reset_gripper_stall();
        self.gripper_hold_vel_ceiling = None;
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        Ok(())
    }

    /// A transition is in flight ([`TorqueTransition`]): ordinary output and blocking torque
    /// changes wait for it.
    pub fn torque_transition_pending(&self) -> bool { self.torque_transition.is_some() }

    /// Starts a torque transition without waiting for its confirmation (the RT's
    /// `commit_transition`). Err: refused before anything was sent (the same refusals as
    /// `set_torque(enable)`), the commit is refused with the code. Ok: started; its progress,
    /// or its failure code once a step failed (the failure is applied as `set_torque` applies
    /// it: fault `ENABLE_FAILED` / `DRIVER_WRITE_FAILED`). Without a driver it completes now.
    pub(crate) fn begin_torque_transition(&mut self, enable: bool) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        use crate::rt_core::ops::{static_code, TorqueProgress as Rt};
        let to_code = |e: SafetyError| static_code(crate::supervisor::ArmError::Safety(e));
        if self.torque_transition.is_some() { return Err("TRANSITION_BUSY"); }
        self.gravity_comp_servo_busy().map_err(to_code)?;
        if enable {
            let (was_torqued, prev_gripper_cmd) = self.enable_prepare_inner(false).map_err(to_code)?;
            let started = match self.driver.as_ref() {
                None => None,
                Some(d) => {
                    let mut drv = d.lock().map_err(|_| "FEEDBACK_UNAVAILABLE")?;
                    let drv_now = drv.clock_now();
                    Some((drv.begin_torque_budgeted(true, 0), drv_now))
                }
            };
            Ok(match started {
                None | Some((Ok(crate::driver::TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed)), _)) =>
                    self.enable_finish_inner(was_torqued, prev_gripper_cmd, true).map(|_| Rt::Done).map_err(|e| torque_error_code(&e)),
                Some((Ok(crate::driver::TorqueProgress::Pending { .. }), now)) => {
                    self.torque_transition = Some(TorqueTransition::Enable { was_torqued, prev_gripper_cmd });
                    Ok(Rt::Pending { wake: now })
                }
                Some((Err(ref e), _)) if crate::driver::is_estop_inhibited(e) =>
                {
                    let still_enabled = self.driver.as_ref().is_some_and(|d| {
                        d.lock().is_ok_and(|drv| {
                            drv.enable_state() != EnableState::Disabled
                                || drv.has_any_joint_enabled()
                                || drv.has_known_partial_enable()
                        })
                    });
                    self.is_torqued = was_torqued || still_enabled;
                    self.pending_rearm = true;
                    return Err("ESTOP_ACTIVE");
                }
                Some((r, _)) => self.enable_step_failed(was_torqued, r),
            })
        } else {
            self.torque_off_check_rt()?;
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            self.disable_requested();
            let started = match self.driver.as_ref() {
                None => None,
                Some(d) => {
                    let mut drv = d.lock().map_err(|_| "FEEDBACK_UNAVAILABLE")?;
                    let drv_now = drv.clock_now();
                    Some((drv.begin_torque_budgeted(false, 0), drv_now))
                }
            };
            Ok(match started {
                None => {
                    self.disable_confirmed();
                    self.torque_off_finish_in_place().map(|()| Rt::Done).map_err(|e| torque_error_code(&e))
                }
                Some((Ok(crate::driver::TorqueProgress::Pending { .. }), now)) => {
                    self.torque_transition = Some(TorqueTransition::Disable { confirm: None });
                    Ok(Rt::Pending { wake: now })
                }
                // the frames went out: the supervisor's confirmation follows (as in request_disable)
                Some((Ok(crate::driver::TorqueProgress::Done(_)), now)) => {
                    self.torque_transition = Some(TorqueTransition::Disable { confirm: Some(DisableConfirm::start(now)) });
                    Ok(Rt::Pending { wake: now })
                }
                Some((Err(e), _)) => Err(self.disable_step_failed(e)),
            })
        }
    }

    /// An enable step that is not a confirmation: the code of the RT contract
    /// (`handshake::rt_progress`), with non-blocking budgeted cleanup.
    fn enable_step_failed(&mut self, was_torqued: bool, r: Result<crate::driver::TorqueProgress, crate::driver::ArmDriverError>)
        -> Result<crate::rt_core::ops::TorqueProgress, &'static str> {
        let message = match &r { Err(e) => e.to_string(), Ok(_) => "enable not confirmed".to_string() };
        let code = crate::driver::handshake::rt_progress(true, r).err().unwrap_or("ENABLE_FAILED");
        let _ = self.enable_failed_budgeted(was_torqued, &message);
        Err(code)
    }

    /// A disable step failed: `disable_pending` stays set and `DRIVER_WRITE_FAILED` latches
    /// (as in `torque_off`).
    fn disable_step_failed(&mut self, e: crate::driver::ArmDriverError) -> &'static str {
        let already_stopped = self.torque_off_stopped();
        let detail = (!already_stopped).then(|| {
            crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                buf.push_str("Torque disable transmission failed: ");
                e.write_bounded(buf);
            })
        });
        let code = crate::driver::handshake::rt_progress(false, Err(e)).err().unwrap_or("HARDWARE_FAULT");
        if let Some(detail) = detail {
            self.safety.latch(crate::safety::FaultKind::Other, detail);
        }
        code
    }

    /// Serves any background (non-broker) disable handshake started by `enable_failed_budgeted`
    /// or `request_disable` under RT executor ownership, sharing Step 2's single RX budget.
    pub(crate) fn service_background_disable(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
    ) -> Option<(u16, bool)> {
        if !self.disable_pending || self.torque_transition.is_some() {
            return None;
        }
        let d = self.driver.as_ref()?;
        let mut drv = d.lock().ok()?;
        if !drv.torque_transition_pending() {
            return None;
        }
        let drv_now = drv.clock_now();
        let span = budget.deadline.saturating_duration_since(budget.now);
        let _ = drv.poll_torque_budgeted(drv_now + span, budget.max_rx_frames);
        drv.take_last_poll_rx()
    }

    /// Serves the transition in flight once, within `budget` (the executor's clock; the
    /// driver's receive pass gets the same span on the driver's clock). None: none in flight.
    /// `Pending { wake }` is on the executor's clock.
    pub(crate) fn service_torque_transition(&mut self, budget: &crate::rt_core::ops::ServiceBudget)
        -> Option<TransitionPoll> {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        use crate::rt_core::ops::TorqueProgress as Rt;
        let t = self.torque_transition.take()?;
        let Some(d) = self.driver.as_ref() else {
            return Some(TransitionPoll { progress: Err("INTERNAL"), rx: None });
        };
        let Ok(mut drv) = d.lock() else {
            return Some(TransitionPoll { progress: Err("FEEDBACK_UNAVAILABLE"), rx: None });
        };
        let drv_now = drv.clock_now();
        let span = budget.deadline.saturating_duration_since(budget.now);
        let to_exec = |wake: Instant| budget.now + wake.saturating_duration_since(drv_now);
        match t {
            TorqueTransition::Enable { was_torqued, prev_gripper_cmd } => {
                if self.estop_pending() || self.safety.state().is_stopped() {
                    drv.abort_enable();
                    let still_enabled = drv.enable_state() != EnableState::Disabled
                        || drv.has_any_joint_enabled()
                        || drv.has_known_partial_enable();
                    drop(drv);
                    self.is_torqued = was_torqued || still_enabled;
                    self.pending_rearm = true;
                    let code = if self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop) {
                        "ESTOP_ACTIVE"
                    } else {
                        "FAULT_ACTIVE"
                    };
                    return Some(TransitionPoll { progress: Err(code), rx: None });
                }
                let r = drv.poll_torque_budgeted(drv_now + span, budget.max_rx_frames);
                let rx = drv.take_last_poll_rx();
                let estop_inhibited = matches!(&r, Err(e) if crate::driver::is_estop_inhibited(e));
                let still_enabled = if estop_inhibited {
                    drv.enable_state() != EnableState::Disabled
                        || drv.has_any_joint_enabled()
                        || drv.has_known_partial_enable()
                } else {
                    false
                };
                drop(drv);
                if estop_inhibited {
                    self.is_torqued = was_torqued || still_enabled;
                    self.pending_rearm = true;
                    return Some(TransitionPoll { progress: Err("ESTOP_ACTIVE"), rx });
                }
                let progress = match r {
                    Ok(Drv::Pending { wake }) => {
                        self.torque_transition = Some(t);
                        Ok(Rt::Pending { wake: to_exec(wake) })
                    }
                    Ok(Drv::Done(TorqueOutcome::Confirmed)) =>
                        self.enable_finish_inner(was_torqued, prev_gripper_cmd, true).map(|_| Rt::Done).map_err(|e| torque_error_code(&e)),
                    r => self.enable_step_failed(was_torqued, r),
                };
                Some(TransitionPoll { progress, rx })
            }
            TorqueTransition::Disable { confirm: None } => {
                let r = drv.poll_torque_budgeted(drv_now + span, budget.max_rx_frames);
                let rx = drv.take_last_poll_rx();
                match r {
                    Ok(Drv::Pending { wake }) => {
                        drop(drv);
                        self.torque_transition = Some(t);
                        Some(TransitionPoll { progress: Ok(Rt::Pending { wake: to_exec(wake) }), rx })
                    }
                    // confirmed or not by the driver's window: the supervisor's confirmation decides
                    // (an unconfirmed disable is never success; `request_disable` does the same)
                    Ok(Drv::Done(_)) => {
                        let now = drv.clock_now();
                        drop(drv);
                        self.torque_transition = Some(TorqueTransition::Disable { confirm: Some(DisableConfirm::start(now)) });
                        Some(TransitionPoll { progress: Ok(Rt::Pending { wake: budget.now }), rx })
                    }
                    Err(e) => {
                        drop(drv);
                        Some(TransitionPoll { progress: Err(self.disable_step_failed(e)), rx })
                    }
                }
            }
            TorqueTransition::Disable { confirm: Some(mut c) } => {
                let mut states = std::mem::take(&mut self.scratch.states);
                let served = c.serve_budgeted(&mut **drv, drv_now + span, budget.max_rx_frames, &mut states);
                self.scratch.states = states;
                drop(drv);
                let rx = served.did_rx.then_some((served.rx_frames, served.budget_exhausted));
                let progress = match served.outcome {
                    None => {
                        self.torque_transition = Some(TorqueTransition::Disable { confirm: Some(c) });
                        let wake = (drv_now + DISABLE_CONFIRM_SERVICE).min(c.deadline());
                        Ok(Rt::Pending { wake: to_exec(wake) })
                    }
                    Some(true) => {
                        self.disable_confirmed();
                        self.torque_off_finish_in_place().map(|()| Rt::Done).map_err(|e| torque_error_code(&e))
                    }
                    Some(false) => {
                        if !self.torque_off_stopped() {
                            self.safety.latch(
                                crate::safety::FaultKind::Other,
                                crate::safety::FaultDetail::Static(crate::safety::StaticFault::DisableUnconfirmed),
                            );
                        }
                        Err("FEEDBACK_UNCONFIRMED")
                    }
                };
                Some(TransitionPoll { progress, rx })
            }
        }
    }

    /// In Fault / EStop with torque on: fresh feedback from every motor reporting Disabled
    /// (Damiao: all fresh 0xCC / feedback states 0) means the motors lost torque, e.g. a
    /// motor power reset during a bus loss (real arm, 2026-11-12: all seven motors came
    /// back Disabled after BusLoss and armd kept `is_torqued: true`). armd then records
    /// torque off (`is_torqued` false, re-arm needs the full enable handshake) and keeps the
    /// Fault until `clear_fault`. Not applied during gravity compensation (its exit path
    /// owns the motor modes) or while a disable is pending.
    pub(super) fn sync_torque_state_after_stop(&mut self) {
        if !self.is_torqued || self.disable_pending || self.gravity_comp.is_some() || !self.safety.state().is_stopped() {
            return;
        }
        let Some(d) = self.driver.as_ref() else { return; };
        let off = d.lock().is_ok_and(|drv| drv.capabilities().enable_feedback && drv.is_feedback_valid()
            && drv.enable_state() == EnableState::Disabled);
        if !off || !self.hardware_feedback_fresh(Duration::from_millis(250)) { return; }
        if let Ok(mut drv) = d.lock() { drv.set_torque_flag(false); }
        tracing::warn!("Motors report Disabled with fresh feedback in {:?}: torque is off (is_torqued = false); the fault stays latched until clear_fault",
            self.safety.state());
        self.is_torqued = false;
        self.pending_rearm = true;
        self.output_suspended = false;
        self.gripper_stalled = false;
        self.gripper_stall_sign = 0.0;
        self.gripper_prev_hw_overload = false;
        self.gripper_hold_vel_ceiling = None;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        if self.measured_positions.iter().all(|q| q.is_finite()) {
            self.commanded_positions = self.measured_positions.clone();
            self.target_positions = self.measured_positions.clone();
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
    }

    /// Builds the common zero-allocation [`crate::safety::ClearContext`] baseline from current
    /// supervisor state. Callers fill in live driver freshness, thermal capability, hardware error,
    /// enable state, and resting-pose checks before invoking [`SafetyManager::clear_check`].
    pub(crate) fn build_clear_context_base(
        &self,
        driver_config_error_override: Option<crate::safety::BoundedText<256>>,
    ) -> crate::safety::ClearContext {
        use crate::safety::{BoundedText, ClearContext};
        let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
        let thermal_below = !self.temperatures.is_empty()
            && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
        let cooldown_elapsed_ms = if thermal_below {
            self.thermal_cooldown.elapsed_ms()
        } else {
            0
        };
        // An expired lease counts as NO lease for the WatchdogTimeout heartbeat condition.
        let has_active_lease = self.is_lease_active();
        let lease_heartbeat_fresh =
            has_active_lease && self.last_heartbeat.elapsed() <= self.lease_timeout;
        ClearContext {
            expected_epoch: None,
            pending_estop: self.estop_request_unapplied(),
            shutdown_committed: self.shutdown_committed,
            driver_config_error: driver_config_error_override
                .or_else(|| self.driver_config_error.as_deref().map(BoundedText::new)),
            gravity_comp_unconfirmed: self.gravity_comp_blocks_clear(),
            disable_pending: self.disable_pending,
            has_driver: self.driver.is_some(),
            feedback_fresh: self.driver.is_none(),
            thermal_sensor_available: self.driver.is_none(),
            thermal_below_threshold: thermal_below,
            thermal_cooldown_elapsed_ms: cooldown_elapsed_ms,
            thermal_cooldown_required_ms: self.thermal_cooldown_duration.as_millis().min(u128::from(u64::MAX)) as u64,
            has_active_lease,
            lease_heartbeat_fresh,
            joint_limit_violation: self.joint_limit_clear_violation(),
            enable_state_known: self.driver.is_none(),
            hardware_reset_required: false,
            hardware_reset_sent: false,
            hardware_error: None,
            resting_pose_violation: None,
            resting_feedback_missing: false,
            mit_modes_uncertain: false,
            mit_lost_unsupported: false,
        }
    }

    /// Builds [`crate::safety::ClearContext`] for the MIT expiry recovery path (checks every
    /// recorded kind including `ThermalTrip`, `BusLoss`, `WatchdogTimeout`, `JointLimit`,
    /// `CollisionLatched`, `Estop`, `Other`, while deferring motor-timeout disable to recovery).
    pub(crate) fn build_mit_recovery_clear_context(&self) -> crate::safety::ClearContext {
        let mut ctx = self.build_clear_context_base(None);
        if let Some(ref d) = self.driver {
            if let Ok(drv) = d.lock() {
                ctx.mit_modes_uncertain = self.mit.owner == crate::mit::MitOwner::Lost && drv.joint_modes_uncertain();
                ctx.thermal_sensor_available = drv.capabilities().temperature_feedback;
                ctx.feedback_fresh = self.feedback_numeric_valid
                    && drv.is_feedback_valid()
                    && drv.feedback_all_within(self.config.dof, Duration::from_millis(250));
                ctx.enable_state_known = true;
            }
        }
        ctx
    }

    /// Pure read-only probe of driver limits for legacy `clear_fault` (`dof > MAX_DOF` fallback
    /// clones candidate config without mutating `self`; `dof <= MAX_DOF` uses the bounded stack probe).
    #[allow(clippy::result_large_err)] // inline bounded text avoids RT heap allocation
    fn probe_driver_limits_legacy(
        &self,
    ) -> Result<Option<CandidateDriverLimits>, crate::safety::BoundedText<256>> {
        if self.driver_config_error.is_none() {
            return Ok(None);
        }
        if self.config.dof <= crate::rt_core::mailbox::MAX_DOF {
            return self.probe_driver_limits_bounded().map(Some);
        }
        use crate::safety::BoundedText;
        let dof = self.config.dof;
        let Some(ref d) = self.driver else {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver unavailable while revalidating limits")));
        };
        let Ok(drv) = d.lock() else {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("driver unavailable while revalidating limits")));
        };
        let mut candidate = self.config.clone();
        for (i, limit) in candidate.limits.iter_mut().enumerate().take(dof) {
            if let Some(max) = drv.max_command_velocity(i) {
                if max.is_finite() && max > 0.0 {
                    limit.max_velocity = limit.max_velocity.min(max);
                }
            }
            if let Some((d_min, d_max)) = drv.position_limits(i) {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    return Err(BoundedText::new(&format!(
                        "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' are invalid",
                        limit.name
                    )));
                }
                let new_min = limit.min_position.max(d_min);
                let new_max = limit.max_position.min(d_max);
                if !new_min.is_finite() || !new_max.is_finite() || new_max - new_min < 1e-2 {
                    return Err(BoundedText::new(&format!(
                        "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with profile limits [{:.4}, {:.4}] (intersected range < 0.01)",
                        limit.name, limit.min_position, limit.max_position
                    )));
                }
                if let Some(pos) = candidate.resting_pose.as_ref().and_then(|r| r.positions.as_ref()).and_then(|p| p.get(i).copied()) {
                    if pos < new_min || pos > new_max {
                        return Err(BoundedText::new(&format!(
                            "Driver position limits [{d_min:.4}, {d_max:.4}] for joint '{}' conflict with resting pose position {pos:.4}",
                            limit.name
                        )));
                    }
                }
                limit.min_position = new_min;
                limit.max_position = new_max;
            }
        }
        drop(drv);
        if let Err(err) = candidate.validate() {
            return Err(BoundedText::new(&format!("Driver limits violated config invariants: {err}")));
        }
        Ok(None)
    }

    pub fn clear_fault(&mut self) -> Result<SafetyState, SafetyError> {
        self.clear_fault_for_epoch(None)
    }

    /// Clear fault, optionally requiring an exact safety epoch (`expected_epoch`).
    /// `SafetyManager::clear_check` runs FIRST before any motor reset, driver limit mutation,
    /// concurrent E-stop consumption, or other side effect.
    pub fn clear_fault_for_epoch(&mut self, expected_epoch: Option<u64>) -> Result<SafetyState, SafetyError> {
        use crate::safety::ClearRefusal;
        let had_driver_config_error = self.driver_config_error.is_some();
        let (candidate_limits, driver_err_override, clear_driver_err_field) = match self.probe_driver_limits_legacy() {
            Ok(cand) => (cand, None, had_driver_config_error),
            Err(err) => (None, Some(err), false),
        };
        let apply_estop_refusal = |this: &mut Self| -> SafetyError {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return SafetyError::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    };
                }
            }
            SafetyError::EStopActive
        };
        let commit_probed_limits = |this: &mut Self| {
            if let Some(ref cand) = candidate_limits {
                this.commit_driver_limits_bounded(cand);
            } else if clear_driver_err_field {
                this.reapply_driver_limits();
            }
        };

        // Run `clear_check` FIRST on universal and supervisor-known blockers (pending E-stop,
        // epoch, shutdown, driver config error, gravity comp, disable_pending, uncertain MIT
        // modes, active-lease watchdog heartbeat) before ANY side effect or bus poll (no
        // `consume_pending_estop`, no driver limit mutation, no motor reset, no MIT fence ack).
        let mit_modes_uncertain = self.mit.owner == crate::mit::MitOwner::Lost
            && self
                .driver
                .as_ref()
                .and_then(|d| d.lock().ok())
                .is_some_and(|drv| drv.joint_modes_uncertain());
        let mut pre_ctx = self.build_clear_context_base(driver_err_override);
        if clear_driver_err_field && driver_err_override.is_none() {
            pre_ctx.driver_config_error = None;
        }
        pre_ctx.expected_epoch = expected_epoch;
        pre_ctx.pending_estop = self.estop_request_unapplied();
        pre_ctx.mit_modes_uncertain = mit_modes_uncertain;
        // Bus/telemetry-derived conditions (freshness, live temperature/cooldown, live joint
        // position, enable state) are evaluated immediately below after updating/reading live
        // telemetry; universal blockers and supervisor-known checks (including `has_active_lease`
        // / `lease_heartbeat_fresh`) run FIRST here before any bus poll or state change.
        pre_ctx.feedback_fresh = true;
        pre_ctx.thermal_sensor_available = true;
        pre_ctx.thermal_below_threshold = true;
        pre_ctx.thermal_cooldown_elapsed_ms = pre_ctx.thermal_cooldown_required_ms;
        pre_ctx.joint_limit_violation = None;
        pre_ctx.enable_state_known = true;
        if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
            if refusal == ClearRefusal::EStopActive {
                return Err(apply_estop_refusal(self));
            }
            return Err(refusal.into_safety_error());
        }
        if !self.safety.state().is_stopped() && !self.safety.has_latched_fault() {
            commit_probed_limits(self);
            return Ok(self.safety.state().clone());
        }

        if self.mit.owner == crate::mit::MitOwner::Lost {
            // Do NOT acknowledge the MIT fence trip or mutate supervisor state before the full
            // `clear_check` passes: `mit_fresh_status()` enforces the quiet period directly while
            // leaving `mit_fence_acked == false` so autonomous CAN refreshes remain inhibited.
            let fresh = match self.driver.as_ref().and_then(|d| d.lock().ok()) {
                Some(mut drv) => drv.mit_fresh_status(),
                None => Err("no driver".into()),
            };
            let st = fresh.map_err(|e| SafetyError::ClearFaultRejected(format!(
                "MIT tracking: fresh feedback is required before clearing ({e}); nothing was sent")))?;
            if st.iter().any(|&c| c >= 8) || (st.contains(&0) && st.contains(&1)) {
                // Evaluate the full MIT recovery `clear_check` read-only from the fresh status
                // snapshot — ZERO fence ack and ZERO supervisor state mutation until `clear_check`
                // and operator confirmation both pass.
                let snapshot = self.driver.as_ref().and_then(|d| {
                    d.lock().ok().and_then(|mut drv| {
                        let states = drv.read_joint_states().ok()?;
                        let ages = drv.feedback_ages();
                        let loads = drv.joint_loads();
                        let caps = drv.capabilities();
                        let modes_uncertain = drv.joint_modes_uncertain();
                        let fb_fresh = drv.is_feedback_valid()
                            && ages.len() == self.config.dof
                            && ages.iter().all(|a| a.is_some_and(|t| t <= THERMAL_FEEDBACK_FRESHNESS_LIMIT))
                            && Self::states_valid(&states, caps, self.config.dof, loads.as_deref());
                        let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                        let thermal_below = fb_fresh
                            && caps.temperature_feedback
                            && !states.is_empty()
                            && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                        let mut rx_buf = Vec::new();
                        let next_cd = self.evaluate_thermal_cooldown_with_driver(
                            drv.as_ref(),
                            &ages,
                            thermal_below,
                            &mut rx_buf,
                        );
                        Some((
                            states,
                            loads,
                            caps.temperature_feedback,
                            fb_fresh,
                            thermal_below,
                            next_cd,
                            modes_uncertain,
                        ))
                    })
                });
                let mut mit_ctx = self.build_clear_context_base(driver_err_override);
                if clear_driver_err_field && driver_err_override.is_none() {
                    mit_ctx.driver_config_error = None;
                }
                mit_ctx.expected_epoch = expected_epoch;
                mit_ctx.pending_estop = self.estop_request_unapplied();
                if let Some((ref states, _, temp_fb, fb_fresh, thermal_below, next_cd, modes_uncertain)) = snapshot {
                    mit_ctx.mit_modes_uncertain = modes_uncertain;
                    mit_ctx.thermal_sensor_available = temp_fb;
                    mit_ctx.feedback_fresh = fb_fresh;
                    mit_ctx.thermal_below_threshold = thermal_below;
                    mit_ctx.thermal_cooldown_elapsed_ms = if thermal_below {
                        next_cd.elapsed_ms()
                    } else {
                        0
                    };
                    mit_ctx.enable_state_known = true;
                    let positions: Vec<f32> = states.iter().take(self.config.dof).map(|s| s.position).collect();
                    mit_ctx.joint_limit_violation =
                        self.joint_limit_clear_violation_for_positions(&positions, candidate_limits.as_ref());
                } else {
                    mit_ctx.feedback_fresh = false;
                    mit_ctx.thermal_sensor_available = false;
                    mit_ctx.thermal_below_threshold = false;
                    mit_ctx.thermal_cooldown_elapsed_ms = 0;
                    mit_ctx.enable_state_known = false;
                    mit_ctx.joint_limit_violation =
                        self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
                }
                if let Err(refusal) = self.safety.clear_check(&mit_ctx) {
                    if refusal == ClearRefusal::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(refusal.into_safety_error());
                }
                // MIT expiry recovery disables every motor on the bus. Unless every arm joint
                // (J1..J6) is already disabled or its own watchdog has already expired
                // (`status == 0 || status >= 8`), this is an operator torque-off request and must
                // pass the single Rule A′ rest-pose decision with no override.
                let arm_joints = self.config.arm_joints();
                let all_arm_already_disabled = !arm_joints.is_empty()
                    && arm_joints
                        .iter()
                        .all(|&i| st.get(i).copied().is_some_and(|c| c == 0 || c >= 8));
                if !all_arm_already_disabled {
                    self.eval_stopped_torque_off()
                        .map_err(|r| r.into_safety_error(&self.config.joint_names))?;
                }
                if !self.mit.recovery_confirmed {
                    return Err(SafetyError::ClearFaultRejected(format!(
                        "{}: {}",
                        crate::mit::RECOVERY_CONFIRM_CODE,
                        crate::mit::RECOVERY_WARNING
                    )));
                }
                // Full `clear_check` and operator confirmation passed: commit synced state now.
                self.mit.recovery_confirmed = false;
                if let Some((states, loads, _, fb_fresh, _, next_cd, _)) = snapshot {
                    if fb_fresh {
                        self.feedback_numeric_valid = true;
                        Self::apply_feedback(
                            &mut self.measured_positions,
                            &mut self.measured_velocities,
                            &mut self.torques,
                            &mut self.temperatures,
                            &mut self.loads,
                            self.config.dof,
                            &states,
                            loads.as_deref(),
                        );
                        self.thermal_cooldown = next_cd;
                    }
                }
                commit_probed_limits(self);
                return self.mit_expiry_recovery_for_epoch(expected_epoch, all_arm_already_disabled);
            }
        }

        let mut ctx;
        if let Some(ref d) = self.driver {
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let mut drv = match d.lock() {
                Ok(g) => g,
                Err(_) => {
                    self.ws_eval_bufs = ws_bufs;
                    return Err(SafetyError::FeedbackUnavailable);
                }
            };
            // Pure read-only telemetry & status snapshot FIRST — ZERO motor resets or side effects.
            let states = match drv.read_joint_states() {
                Ok(s) => s,
                Err(e) => {
                    drop(drv);
                    self.ws_eval_bufs = ws_bufs;
                    return Err(SafetyError::ClearFaultRejected(e.to_string()));
                }
            };
            let ages = drv.feedback_ages();
            let loads = drv.joint_loads();
            let caps = drv.capabilities();
            let disabling = drv.clear_fault_disables_joints();
            let hw_fault = drv.check_hardware_status().is_err();
            let mut state = drv.enable_state();
            let mut partial = drv.has_known_partial_enable();
            let mut any_enabled = drv.has_any_joint_enabled();
            let fb_valid = drv.is_feedback_valid();
            let states_ok = Self::states_valid(&states, caps, self.config.dof, loads.as_deref());
            let fb_fresh = fb_valid
                && ages.len() == self.config.dof
                && !ages.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)))
                && states_ok;
            let needs_resting_check = (self.is_torqued || any_enabled || state != EnableState::Disabled)
                && disabling.iter().any(|&j| j < self.config.arm_dof());
            let max_rest_age = self
                .resting_policy()
                .map(|p| Duration::from_millis(p.feedback_max_age_ms))
                .unwrap_or(Duration::from_millis(250));
            let resting_fb_ok = fb_valid
                && ages.len() == self.config.dof
                && ages.iter().all(|a| a.is_some_and(|t| t <= max_rest_age))
                && states_ok;

            let defer_mit_commit = self.mit.owner == crate::mit::MitOwner::Lost;
            if fb_fresh && !defer_mit_commit {
                self.feedback_numeric_valid = true;
                self.last_can_feedback_instant = ages
                    .iter()
                    .copied()
                    .try_fold(Duration::ZERO, |m, a| a.map(|t| m.max(t)))
                    .and_then(|age| Instant::now().checked_sub(age));
                self.core.poll_feedback(
                    &mut self.joint_feedback_queue_bounds,
                    self.config.dof,
                    drv.as_ref(),
                    &mut self.scratch.fb,
                );
                Self::apply_feedback(
                    &mut self.measured_positions,
                    &mut self.measured_velocities,
                    &mut self.torques,
                    &mut self.temperatures,
                    &mut self.loads,
                    self.config.dof,
                    &states,
                    loads.as_deref(),
                );
            }
            let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
            let thermal_below = fb_fresh
                && caps.temperature_feedback
                && !states.is_empty()
                && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
            // Cooldown accrues ONLY across consecutive fresh cool samples; stale/unavailable/gapped temp resets it.
            let mut rx_buf = Vec::new();
            let next_cd = self.evaluate_thermal_cooldown_with_driver(
                drv.as_ref(),
                &ages,
                thermal_below,
                &mut rx_buf,
            );
            if !defer_mit_commit {
                self.thermal_cooldown = next_cd;
            }

            let mut resting_feedback_missing = false;
            let mut resting_pose_violation = None;
            if needs_resting_check {
                if !resting_fb_ok {
                    resting_feedback_missing = true;
                } else {
                    resting_pose_violation =
                        self.resting_pose_support_violation_bounded(Some(drv.as_ref()), &ages, &mut ws_bufs);
                }
            }
            let enable_known = state != EnableState::Unknown || partial;
            drop(drv);
            self.ws_eval_bufs = ws_bufs;

            ctx = self.build_clear_context_base(driver_err_override);
            if clear_driver_err_field && driver_err_override.is_none() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.feedback_fresh = fb_fresh;
            ctx.thermal_sensor_available = caps.temperature_feedback;
            ctx.thermal_below_threshold = thermal_below;
            if defer_mit_commit {
                ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
                let positions: Vec<f32> = states.iter().take(self.config.dof).map(|s| s.position).collect();
                ctx.joint_limit_violation = if fb_fresh {
                    self.joint_limit_clear_violation_for_positions(&positions, candidate_limits.as_ref())
                } else {
                    self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref())
                };
            } else {
                ctx.joint_limit_violation =
                    self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            }
            ctx.resting_feedback_missing = resting_feedback_missing;
            ctx.resting_pose_violation = resting_pose_violation;
            ctx.hardware_reset_required = hw_fault;
            ctx.enable_state_known = enable_known;

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any motor reset or
            // `refresh_enable_state` side effect.
            match self.safety.clear_check(&ctx) {
                Ok(()) => {}
                Err(ClearRefusal::EStopActive) => return Err(apply_estop_refusal(self)),
                Err(ClearRefusal::HardwareResetRequired) => {
                    // Explicitly modeled exception: every OTHER recorded kind's condition passed,
                    // so transmitting the motor fault reset is now permitted; re-evaluate
                    // `clear_check` immediately after the reset.
                    let reset_res = {
                        let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                        drv.clear_fault()
                    };
                    ctx.hardware_reset_required = false;
                    ctx.pending_estop = self.estop_request_unapplied();
                    match reset_res {
                        Ok(()) => ctx.hardware_reset_sent = true,
                        Err(e) => ctx.hardware_error = Some(clear_fault_driver_error(&e)),
                    }
                    let post_refusal = self.safety.clear_check(&ctx).unwrap_err();
                    if post_refusal == ClearRefusal::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(post_refusal.into_safety_error());
                }
                Err(ClearRefusal::EnableStateUnknown) if fb_fresh && !hw_fault && !partial => {
                    // Every other condition passed; probe enable state once and re-run clear_check.
                    {
                        let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                        let _ = drv.refresh_enable_state();
                        state = drv.enable_state();
                        partial = drv.has_known_partial_enable();
                        any_enabled = drv.has_any_joint_enabled();
                    }
                    ctx.enable_state_known = state != EnableState::Unknown || partial;
                    ctx.pending_estop = self.estop_request_unapplied();
                    if let Err(refusal) = self.safety.clear_check(&ctx) {
                        if refusal == ClearRefusal::EStopActive {
                            return Err(apply_estop_refusal(self));
                        }
                        return Err(refusal.into_safety_error());
                    }
                }
                Err(refusal) => return Err(refusal.into_safety_error()),
            }
            if defer_mit_commit && fb_fresh {
                self.feedback_numeric_valid = true;
                self.last_can_feedback_instant = ages
                    .iter()
                    .copied()
                    .try_fold(Duration::ZERO, |m, a| a.map(|t| m.max(t)))
                    .and_then(|age| Instant::now().checked_sub(age));
                if let Ok(drv) = d.lock() {
                    self.core.poll_feedback(
                        &mut self.joint_feedback_queue_bounds,
                        self.config.dof,
                        drv.as_ref(),
                        &mut self.scratch.fb,
                    );
                }
                Self::apply_feedback(
                    &mut self.measured_positions,
                    &mut self.measured_velocities,
                    &mut self.torques,
                    &mut self.temperatures,
                    &mut self.loads,
                    self.config.dof,
                    &states,
                    loads.as_deref(),
                );
                self.thermal_cooldown = next_cd;
            }
            commit_probed_limits(self);
            self.is_torqued = state == EnableState::Enabled || partial || any_enabled;
        } else {
            self.update_thermal_cooldown_driverless(Instant::now());
            ctx = self.build_clear_context_base(driver_err_override);
            if clear_driver_err_field && driver_err_override.is_none() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == ClearRefusal::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                return Err(refusal.into_safety_error());
            }
            commit_probed_limits(self);
        }
        self.clear_fault_finish(&ctx)
    }

    fn clear_fault_finish(&mut self, ctx: &crate::safety::ClearContext) -> Result<SafetyState, SafetyError> {
        self.clear_fault_finish_bounded(ctx).map_err(SafetyError::from)
    }

    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    fn clear_fault_finish_bounded(
        &mut self,
        ctx: &crate::safety::ClearContext,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        use crate::safety::{ClearFaultError, ClearRefusal};
        if self.shutdown_committed {
            return Err(ClearFaultError::Rejected(ClearRefusal::ShutdownCommitted));
        }
        // Single authoritative clear_check FIRST before mutating/resetting state.
        self.safety.clear_check(ctx).map_err(ClearFaultError::Rejected)?;
        let was_mit_lost = self.mit.owner == crate::mit::MitOwner::Lost;
        self.pending_rearm = true;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        // Reset at the measured position; never retain a residual old trajectory target.
        copy_into(&mut self.commanded_positions, &self.measured_positions);
        copy_into(&mut self.target_positions, &self.measured_positions);
        self.commanded_velocities.fill(0.0);
        self.last_commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.bus_loss_instant = None;
        self.reset_gripper_stall();
        if self.consume_pending_estop() {
            if let Some(received) = ctx.expected_epoch {
                if received != self.safety.epoch_id() {
                    return Err(ClearFaultError::EpochMismatch {
                        expected: self.safety.epoch_id(),
                        received,
                    });
                }
            }
            return Err(ClearFaultError::EStopActive);
        }
        let was_estop = *self.safety.state() == SafetyState::EStop;
        let state = self.safety.clear_with_check(ctx).map_err(ClearFaultError::from)?;
        self.handover_settling_since = None;
        if !was_estop && self.e_stop_latched.load(Ordering::SeqCst) {
            self.apply_stop_policy(StopPolicy::EStopLatch);
            self.last_heartbeat = Instant::now();
            self.last_motion_frame = Instant::now();
            return Ok(self.safety.state().clone());
        }
        self.e_stop_latched.store(false, Ordering::SeqCst);
        self.shutdown_refused_unknown = false;
        if self.consume_pending_estop() {
            if let Some(received) = ctx.expected_epoch {
                if received != self.safety.epoch_id() {
                    return Err(ClearFaultError::EpochMismatch {
                        expected: self.safety.epoch_id(),
                        received,
                    });
                }
            }
            return Err(ClearFaultError::EStopActive);
        }
        if was_mit_lost {
            // Only after full clear passed: release MIT lost ownership and allow read-only polls.
            self.mit.owner = crate::mit::MitOwner::Off;
            self.mit.joints.clear();
            self.mit.armed_ok = false;
            if let Some(mut drv) = self.driver.as_ref().and_then(|d| d.lock().ok()) {
                drv.acknowledge_mit_fence_trip();
            }
        }
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        Ok(state)
    }

    fn measured_workspace_violation_bounded(
        &self,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::WorkspaceSupportViolation> {
        use crate::safety::{BoundedText, WorkspaceSupportViolation};
        let (ws, cam) = self.workspace_policy()?;
        if self.config.arm_model.frames_f32(&self.measured_positions).is_none() {
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason: BoundedText::new("non-finite or incomplete joint positions"),
            });
        }
        if ws.self_collision_enabled && !ws.link_capsules.is_empty() && !ws.self_collision_pairs.is_empty()
            && (!ws.self_collision_min_clearance_m.is_finite() || ws.self_collision_min_clearance_m < 0.0)
        {
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason: BoundedText::new("invalid self_collision_min_clearance_m"),
            });
        }
        if let Some(cap) = ws.link_capsules.iter().find(|c| self.config.arm_model.resolve_frame(&c.frame).is_none()) {
            let mut reason = BoundedText::<128>::empty();
            let _ = core::fmt::write(&mut reason, format_args!("unknown frame '{}' for capsule '{}'", cap.frame, cap.id));
            return Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new("invalid"),
                reason,
            });
        }
        let eval = crate::chain::WorkspaceEvaluator::new_with_tcp_in(
            &self.config.arm_model, ws, cam, self.config.tcp_offset, bufs,
        );
        let mut buf = eval.alloc_buf_in(bufs);
        let violation = match eval.eval_into_in(&self.measured_positions, &mut buf, &mut bufs.world_caps, "clear_fault") {
            Err(f) => Some(WorkspaceSupportViolation::EvalFailed {
                limit: BoundedText::new(&f.limit),
                reason: BoundedText::new(f.detail.reason.as_deref().unwrap_or("invalid joint positions")),
            }),
            Ok(()) => buf.as_slice().iter().enumerate()
                .find(|(_, clr)| !clr.is_finite() || **clr < -crate::kinematics::WORKSPACE_EPS)
                .map(|(slot, &clr)| describe_workspace_slot_bounded(&eval, ws, cam, self.config.tcp_offset, slot, clr)),
        };
        crate::chain::WorkspaceEvaluator::recycle_buf(buf, bufs);
        eval.recycle(bufs);
        violation
    }

    /// Allocation-free bounded equivalent of [`Self::resting_pose_support_violation_with_driver`].
    pub(crate) fn resting_pose_support_violation_bounded(
        &self,
        drv: Option<&dyn ArmDriver>,
        ages: &[Option<Duration>],
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::RestingSupportViolation> {
        use crate::safety::{RestingSupportJointItem as Item, RestingSupportViolation as V, MAX_RESTING_SUPPORT_ITEMS};
        let dof = self.config.dof;
        if self.measured_positions.len() != dof || self.measured_velocities.len() != dof {
            return Some(V::FeedbackLengthMismatch);
        }
        let Some(policy) = self.config.resting_pose.as_ref() else { return Some(V::NoRestingPoseConfigured); };
        if self.driver.is_some() {
            let Some(driver) = drv else { return Some(V::DriverFeedbackUnavailable); };
            let caps = driver.capabilities();
            if !caps.velocity_feedback { return Some(V::DriverLacksVelocityFeedback); }
            if !policy.support_torque_limits.is_empty() && !caps.torque_feedback { return Some(V::DriverLacksTorqueFeedback); }
            if !policy.support_load_limits.is_empty() && !caps.load_feedback { return Some(V::DriverLacksLoadFeedback); }
            let max_age = Duration::from_millis(policy.feedback_max_age_ms);
            if !self.feedback_numeric_valid || ages.len() != dof || !ages.iter().all(|a| a.is_some_and(|age| age <= max_age)) {
                return Some(V::FeedbackTooOld { max_age_ms: policy.feedback_max_age_ms });
            }
        }
        let mut items = [None; MAX_RESTING_SUPPORT_ITEMS];
        let mut len = 0usize;
        let mut overflowed = 0u16;
        for (i, &v) in self.measured_velocities.iter().enumerate().take(dof) {
            let limit = policy.max_velocity_of(i);
            if !v.is_finite() || v.abs() > limit {
                if len < MAX_RESTING_SUPPORT_ITEMS {
                    items[len] = Some(Item::JointMoving { joint: i as u16, velocity: v, limit });
                    len += 1;
                } else {
                    overflowed = overflowed.saturating_add(1);
                }
            }
        }
        let mut total = 0.0;
        let rest = self.config.resting_positions();
        for (i, &tol) in policy.position_tolerances.iter().enumerate().take(dof) {
            let dev = (self.measured_positions[i] - rest[i]).abs();
            if !dev.is_finite() || dev > tol {
                if len < MAX_RESTING_SUPPORT_ITEMS {
                    items[len] = Some(Item::JointOffHome { joint: i as u16, deviation: dev, limit: tol });
                    len += 1;
                } else {
                    overflowed = overflowed.saturating_add(1);
                }
            }
            total += dev;
        }
        if len == 0 && total > policy.max_total_deviation {
            items[0] = Some(Item::TotalDeviation { total, limit: policy.max_total_deviation });
            len = 1;
        }
        if self.driver.is_some() {
            for &(i, max_tau) in &policy.support_torque_limits {
                if i >= dof { return Some(V::SupportTorqueIndexOutOfRange { index: i }); }
                let tau = self.torques.get(i).copied().unwrap_or(f32::NAN);
                if !tau.is_finite() || tau.abs() > max_tau {
                    if len < MAX_RESTING_SUPPORT_ITEMS {
                        items[len] = Some(Item::JointTorque { joint: i as u16, torque: tau, limit: max_tau });
                        len += 1;
                    } else {
                        overflowed = overflowed.saturating_add(1);
                    }
                }
            }
            for &(i, max_load) in &policy.support_load_limits {
                if i >= dof { return Some(V::SupportLoadIndexOutOfRange { index: i }); }
                let load = self.loads.get(i).copied().unwrap_or(f32::NAN);
                if !load.is_finite() || load.abs() > max_load {
                    if len < MAX_RESTING_SUPPORT_ITEMS {
                        items[len] = Some(Item::JointLoad { joint: i as u16, load, limit: max_load });
                        len += 1;
                    } else {
                        overflowed = overflowed.saturating_add(1);
                    }
                }
            }
        }
        let mut workspace = self.measured_workspace_violation_bounded(bufs);
        if workspace.is_some() && len >= MAX_RESTING_SUPPORT_ITEMS {
            workspace = None;
            overflowed = overflowed.saturating_add(1);
        }
        (len > 0 || workspace.is_some()).then_some(V::Failures { items, len: len as u8, overflowed, workspace })
    }

    /// Bounded, allocation-free fault clearing for the RT executor; refusal text is rendered off-thread.
    /// `SafetyManager::clear_check` runs FIRST before any concurrent-E-stop mutation, driver limit
    /// commit, motor reset, or other side effect.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_executor_bounded(
        &mut self,
        deadline: Instant,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        self.clear_fault_executor_bounded_for_epoch(deadline, None)
    }

    /// Bounded, allocation-free fault clearing for the RT executor enforcing exact `expected_epoch`
    /// parity with [`Self::clear_fault_for_epoch`].
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_executor_bounded_for_epoch(
        &mut self,
        deadline: Instant,
        expected_epoch: Option<u64>,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        use crate::safety::{ClearFaultError as E, ClearRefusal as R};
        let apply_estop_refusal = |this: &mut Self| -> E {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return E::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    };
                }
            }
            E::EStopActive
        };
        let had_driver_config_error = self.driver_config_error.is_some();
        let (candidate_limits, driver_err_override) = if had_driver_config_error {
            match self.probe_driver_limits_bounded() {
                Ok(cand) => (Some(cand), None),
                Err(err) => (None, Some(err)),
            }
        } else {
            (None, None)
        };
        let is_stopped_or_latched = self.safety.state().is_stopped() || self.safety.has_latched_fault();

        if !is_stopped_or_latched {
            let mut ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                return Err(E::from(refusal));
            }
            if let Some(ref cand) = candidate_limits {
                self.commit_driver_limits_bounded(cand);
            }
            return Ok(self.safety.state().clone());
        }

        let mut ctx;
        if let Some(ref d) = self.driver {
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let drv = match d.lock() {
                Ok(g) => g,
                Err(_) => {
                    self.scratch.ages = scratch_ages;
                    self.scratch.rx = scratch_rx;
                    self.ws_eval_bufs = ws_bufs;
                    return Err(E::FeedbackUnavailable);
                }
            };
            // Pure read-only driver snapshot FIRST — ZERO motor resets or mutations.
            drv.feedback_ages_into(&mut scratch_ages);
            let ages_valid = scratch_ages.len() == self.config.dof
                && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
            let fb_valid = drv.is_feedback_valid();
            let hw_fault = drv.has_hardware_fault();
            let temp_fb = drv.capabilities().temperature_feedback;
            let state = drv.enable_state();
            let partial = drv.has_known_partial_enable();
            let any_enabled = drv.has_any_joint_enabled();
            let needs_resting_check = (self.is_torqued || any_enabled || state != EnableState::Disabled)
                && drv.clear_fault_disables_arm_joint(self.config.arm_dof());
            let mut resting_feedback_missing = false;
            let mut resting_pose_violation = None;
            if needs_resting_check {
                let max_age = self
                    .config
                    .resting_pose
                    .as_ref()
                    .map(|p| Duration::from_millis(p.feedback_max_age_ms))
                    .unwrap_or(Duration::from_millis(250));
                if !self.feedback_numeric_valid
                    || !fb_valid
                    || scratch_ages.len() != self.config.dof
                    || !scratch_ages.iter().all(|a| a.is_some_and(|t| t <= max_age))
                {
                    resting_feedback_missing = true;
                } else {
                    resting_pose_violation = self.resting_pose_support_violation_bounded(
                        Some(drv.as_ref()),
                        &scratch_ages,
                        &mut ws_bufs,
                    );
                }
            }
            let fb_fresh = self.feedback_numeric_valid && fb_valid && ages_valid;
            // Cooldown accrues ONLY across consecutive fresh cool samples; stale/unavailable/gapped temp resets it.
            let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
            let thermal_below = fb_fresh
                && temp_fb
                && !self.temperatures.is_empty()
                && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
            let next_cd = self.evaluate_thermal_cooldown_with_driver(
                drv.as_ref(),
                &scratch_ages,
                thermal_below,
                &mut scratch_rx,
            );
            drop(drv);
            self.scratch.ages = scratch_ages;
            self.scratch.rx = scratch_rx;
            self.ws_eval_bufs = ws_bufs;
            self.thermal_cooldown = next_cd;

            ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            ctx.feedback_fresh = fb_fresh;
            ctx.thermal_sensor_available = temp_fb;
            ctx.thermal_below_threshold = thermal_below;
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            ctx.resting_feedback_missing = resting_feedback_missing;
            ctx.resting_pose_violation = resting_pose_violation;
            ctx.hardware_reset_required = hw_fault;
            ctx.enable_state_known = state != EnableState::Unknown || partial;

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any motor reset or
            // concurrent E-stop / config mutation.
            match self.safety.clear_check(&ctx) {
                Ok(()) => {}
                Err(R::EStopActive) => return Err(apply_estop_refusal(self)),
                Err(R::HardwareResetRequired) => {
                    // Explicitly modeled exception: every OTHER recorded kind's condition passed,
                    // so a budgeted motor reset is now permitted; re-evaluate `clear_check` after.
                    let now = Instant::now();
                    if now >= deadline {
                        return Err(E::Rejected(R::BudgetExpired));
                    }
                    let reset_res = {
                        let mut drv = d.lock().map_err(|_| E::FeedbackUnavailable)?;
                        let drv_deadline = drv.clock_now() + deadline.saturating_duration_since(now);
                        drv.clear_fault_budgeted(drv_deadline)
                    };
                    ctx.hardware_reset_required = false;
                    ctx.pending_estop = self.estop_request_unapplied();
                    match reset_res {
                        Ok(()) => ctx.hardware_reset_sent = true,
                        Err(e) => ctx.hardware_error = Some(clear_fault_driver_error(&e)),
                    }
                    let post_refusal = self.safety.clear_check(&ctx).unwrap_err();
                    if post_refusal == R::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(E::from(post_refusal));
                }
                Err(refusal) => return Err(E::from(refusal)),
            }
            if let Some(ref cand) = candidate_limits {
                self.commit_driver_limits_bounded(cand);
            }
            self.is_torqued = state == EnableState::Enabled || partial || any_enabled;
        } else {
            // Driverless simulation mode: update cooldown clock in place (0 alloc).
            self.update_thermal_cooldown_driverless(Instant::now());
            ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                return Err(E::from(refusal));
            }
            if let Some(ref cand) = candidate_limits {
                self.commit_driver_limits_bounded(cand);
            }
        }
        self.clear_fault_finish_bounded(&ctx)
    }

    /// Bounded fault clearing for the RT executor rendered into [`SafetyError`] for direct callers.
    pub fn clear_fault_executor(&mut self, deadline: Instant) -> Result<SafetyState, SafetyError> {
        self.clear_fault_executor_for_epoch(deadline, None)
    }

    /// Bounded fault clearing for the RT executor with exact epoch check rendered into [`SafetyError`].
    pub fn clear_fault_executor_for_epoch(
        &mut self,
        deadline: Instant,
        expected_epoch: Option<u64>,
    ) -> Result<SafetyState, SafetyError> {
        self.clear_fault_executor_bounded_for_epoch(deadline, expected_epoch)
            .map_err(SafetyError::from)
    }

    /// Bounded last-chance disable on a panic or forced teardown path owned by this `SupervisorInner`.
    /// Uses bounded `try_lock()` so a contended driver mutex cannot block indefinitely past `timeout`.
    pub fn attempt_panic_disable(&mut self, timeout: Duration) -> PanicDisableOutcome {
        self.e_stop_latched.store(true, Ordering::SeqCst);
        self.disable_requested();
        let Some(ref d) = self.driver else {
            self.disable_confirmed();
            return PanicDisableOutcome::NoDriver;
        };
        let wall_deadline = Instant::now() + timeout;
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut drv = loop {
                match d.try_lock() {
                    Ok(g) => break g,
                    Err(std::sync::TryLockError::Poisoned(p)) => break p.into_inner(),
                    Err(std::sync::TryLockError::WouldBlock) => {
                        let rem = wall_deadline.saturating_duration_since(Instant::now());
                        if rem.is_zero() {
                            return PanicDisableOutcome::TimedOut;
                        }
                        std::thread::sleep(rem.min(Duration::from_millis(1)));
                    }
                }
            };
            let rem = wall_deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                return PanicDisableOutcome::TimedOut;
            }
            bounded_panic_disable_driver(&mut **drv, rem)
        }))
        .unwrap_or_else(|_| PanicDisableOutcome::Failed("panic during driver panic-disable".into()));
        if outcome == PanicDisableOutcome::Confirmed {
            self.disable_confirmed();
        }
        outcome
    }

    pub fn driver_feedback_confirmed(&self) -> bool {
        match &self.driver {
            None => false,
            Some(d) => d.lock().is_ok_and(|drv| drv.is_feedback_valid()),
        }
    }
}

fn clear_fault_driver_error(e: &crate::driver::ArmDriverError) -> crate::safety::ClearFaultDriverError {
    e.to_clear_fault_error()
}

fn describe_workspace_slot_bounded(
    eval: &crate::chain::WorkspaceEvaluator<'_>,
    ws: &crate::model::WorkspacePolicy,
    cam: &crate::model::CameraMount,
    tcp_offset: [f64; 3],
    slot: usize,
    clr: f64,
) -> crate::safety::WorkspaceSupportViolation {
    use crate::kinematics as k;
    use crate::safety::{BoundedText, WorkspaceSupportViolation as W};
    use core::fmt::Write as _;

    let depth = -clr;
    if slot >= eval.self_collision_slot_start {
        let pi = slot - eval.self_collision_slot_start;
        let (id_a, id_b) = ws.self_collision_pairs.get(pi).map_or(("?", "?"), |[a, b]| (a.as_str(), b.as_str()));
        let (mut cid, mut gid, mut limit) = (BoundedText::empty(), BoundedText::empty(), BoundedText::empty());
        let _ = write!(&mut cid, "self_collision:{id_a}:{id_b}");
        let _ = write!(&mut gid, "capsule:{id_a}");
        let _ = write!(&mut limit, "self_collision '{id_a}' vs '{id_b}'");
        let ia = ws.link_capsules.iter().position(|c| c.id == id_a);
        let ib = ws.link_capsules.iter().position(|c| c.id == id_b);
        let point = match (ia, ib) { (Some(a), Some(b)) if a != b => eval.num_end_pts + a, _ => 0 };
        let bound = f64::from(ws.self_collision_min_clearance_m);
        return W::Constraint { limit, point, cid, gid, depth, value: bound + clr, bound };
    }
    if eval.has_reach && slot >= eval.num_geoms * eval.slots_per_geom {
        let reach_idx = slot - eval.num_geoms * eval.slots_per_geom;
        let bound = f64::from(ws.max_reach_m.unwrap_or(0.0));
        let (point, gid) = if reach_idx == 0 {
            (0, BoundedText::new("end_origin"))
        } else {
            let tcp_f32 = tcp_offset.map(|v| v as f32);
            let pt = ws.check_points_end_m.iter()
                .position(|c| (c[0] - tcp_f32[0]).abs() <= 1e-6 && (c[1] - tcp_f32[1]).abs() <= 1e-6 && (c[2] - tcp_f32[2]).abs() <= 1e-6)
                .map_or(0, |i| i + 1);
            (pt, BoundedText::new("tcp_origin"))
        };
        return W::Constraint { limit: BoundedText::new("reach"), point, cid: BoundedText::new("max_reach"), gid, depth, value: bound - clr, bound };
    }
    let (gi, ci) = (slot / eval.slots_per_geom, slot % eval.slots_per_geom);
    let mut gid = BoundedText::empty();
    if gi == 0 {
        gid.push_str("end_origin");
    } else if gi < eval.num_end_pts {
        if cam.offset_m.is_some() && gi + 1 == eval.num_end_pts { gid.push_str("camera_mount"); }
        else { let _ = write!(&mut gid, "end_point:{gi}"); }
    } else {
        let _ = write!(&mut gid, "capsule:{}", ws.link_capsules[gi - eval.num_end_pts].id);
    }
    let (mut limit, mut cid) = (BoundedText::empty(), BoundedText::empty());
    let (value, bound) = if ci == 0 {
        limit.push_str("floor"); cid.push_str("floor_z");
        let b = f64::from(ws.floor_z_m); (b + clr, b)
    } else if ci <= 3 {
        let ax = ci - 1;
        limit.push_str(k::BOX_LIMITS[ax][0]); cid.push_str(k::BOX_CONSTRAINT_IDS[ax][0]);
        let b = f64::from(ws.box_min_m[ax]); (b + clr, b)
    } else if ci <= 6 {
        let ax = ci - 4;
        limit.push_str(k::BOX_LIMITS[ax][1]); cid.push_str(k::BOX_CONSTRAINT_IDS[ax][1]);
        let b = f64::from(ws.box_max_m[ax]); (b - clr, b)
    } else {
        let name = ws.keepout_boxes.get(ci - 7).map_or("?", |b| b.name.as_str());
        let _ = write!(&mut limit, "keepout '{name}'"); let _ = write!(&mut cid, "keepout:{name}");
        (depth, 0.0)
    };
    W::Constraint { limit, point: gi, cid, gid, depth, value, bound }
}
