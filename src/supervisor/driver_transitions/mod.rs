//! Driver transitions of the supervisor: driver attachment, feedback intake, driver limits, torque enable/disable and fault clearing (track A).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

mod limits_feedback;
mod disable_request;
mod measured_hold;
mod enable_prepare;
mod enable_poll;
mod torque_off_gate;
mod legacy_torque;
mod park_disable;
mod service_transition;
mod clear_fault_legacy;
mod clear_fault_support;
mod clear_fault_executor;
mod panic_disable;

/// Interval between service polls in bounded panic-disable teardown.
pub const PANIC_DISABLE_SERVICE: Duration = Duration::from_millis(2);
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
    raw_pos: [Option<(f32, f32)>; crate::rt_core::mailbox::MAX_DOF],
}

/// Side effects staged during `clear_fault` preflight and applied ONLY AFTER
/// `SafetyManager::clear_with_check` commits (zero state mutations on refusal).
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct ClearFaultCommit {
    pub(crate) candidate_limits: Option<CandidateDriverLimits>,
    pub(crate) reapply_legacy_limits: bool,
    pub(crate) is_torqued: Option<bool>,
    pub(crate) reset_disarmed_hold: bool,
    pub(crate) thermal_cooldown: Option<ThermalCooldown>,
}

/// Bounded, inline error returned by [`SupervisorInner::request_disable_bounded`] (0 heap allocs).
pub(crate) struct RequestDisableError {
    pub(crate) code: &'static str,
    pub(crate) message: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    /// True when the driver transmit choke point refused `DisableRequested` (Rule A′ or 1 s
    /// request lifetime expiry), so the pending disable request ended cleanly without TX.
    pub(crate) refused_by_chokepoint: bool,
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

/// Splits a panic-disable `timeout` anchored at `origin` into `(coop_deadline, hard_deadline)`.
///
/// `hard_deadline` stays strictly `origin + timeout` (preserving `PANIC_DISABLE_TIMEOUT = 500 ms`
/// and every blocking-driver bound). `coop_deadline` stops cooperative retries slightly earlier
/// (`10%`, capped at `25 ms`) so post-loop logging, lock release, and channel/condvar handoff
/// complete before the hard watchdog fires `TimedOut`.
pub(crate) fn panic_disable_deadlines(origin: Instant, timeout: Duration) -> (Instant, Instant) {
    let hard = origin + timeout;
    let reserve = (timeout / 10).min(Duration::from_millis(25));
    let coop = origin + timeout.saturating_sub(reserve);
    (coop, hard)
}

/// Runs `work` on the bus owner thread (strictly preserving single-bus-writer ownership) while
/// an independent deadline observer thread waits up to `hard_deadline` without touching the bus or
/// driver. If `work` blocks past `hard_deadline` (e.g. a driver implementing only blocking
/// `set_torque` or a stalled transport syscall), the observer fires `on_timeout` at the deadline
/// and this function returns `None` if `work` later unblocks.
pub(crate) fn run_with_panic_deadline_observer_until(
    hard_deadline: Instant,
    work: impl FnOnce() -> PanicDisableOutcome,
    on_complete: impl FnOnce(&PanicDisableOutcome) -> bool,
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
        let deadline = hard_deadline;
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
        let (won, fire_timeout_inline) = {
            let mut g = state.0.lock().unwrap_or_else(|e| e.into_inner());
            if *g == ObserverState::Running {
                if Instant::now() >= hard_deadline || !on_complete(&outcome) {
                    *g = ObserverState::TimedOut;
                    (false, true)
                } else {
                    *g = ObserverState::Completed;
                    (true, false)
                }
            } else {
                (false, false)
            }
        };
        if won {
            state.1.notify_all();
            let _ = observer.join();
            Some(outcome)
        } else {
            if fire_timeout_inline {
                state.1.notify_all();
                if let Some(cb) = on_timeout_slot.lock().unwrap_or_else(|e| e.into_inner()).take() {
                    cb();
                }
            }
            // Always join observer once `work()` has returned so any in-flight `on_timeout`
            // callback (including `timeout_hook`) completes before the caller proceeds to exit.
            let _ = observer.join();
            None
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (hard_deadline, on_timeout);
        let outcome = work();
        let _ = on_complete(&outcome);
        Some(outcome)
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
    run_with_panic_deadline_observer_until(Instant::now() + timeout, work, |_| true, on_timeout)
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
#[allow(dead_code)]
pub fn bounded_panic_disable_driver(drv: &mut dyn ArmDriver, timeout: Duration) -> PanicDisableOutcome {
    let deadline = Instant::now() + timeout;
    bounded_panic_disable_driver_until(drv, deadline, deadline, timeout)
}

pub(crate) fn bounded_panic_disable_driver_until(
    drv: &mut dyn ArmDriver,
    coop_deadline: Instant,
    hard_deadline: Instant,
    coop_budget: Duration,
) -> PanicDisableOutcome {
    let enable_fb = drv.capabilities().enable_feedback;

    // Preferred per-motor selective retry path (e.g. Damiao CAN bus): sends disable to every
    // unconfirmed motor on each pass, never stops on a single motor's TX error, and keeps
    // retrying until all motors confirm or the cooperative budget is spent.
    let mut first_step = true;
    let mut last_step_err: Option<String> = None;
    let mut use_generic = false;
    let drv_coop_deadline = drv.clock_now() + coop_budget;
    loop {
        let drv_now = drv.clock_now();
        let now = Instant::now();
        if now >= hard_deadline {
            if first_step {
                return PanicDisableOutcome::TimedOut;
            }
            break;
        }
        let rem = coop_deadline.saturating_duration_since(now);
        let drv_rem = drv_coop_deadline.saturating_duration_since(drv_now);
        if !first_step && (rem.is_zero() || drv_rem.is_zero()) {
            break;
        }
        let hard_rem = hard_deadline.saturating_duration_since(now);
        let step_budget = if rem.is_zero() {
            hard_rem.min(PANIC_DISABLE_SERVICE)
        } else {
            rem.min(hard_rem).min(drv_rem.max(PANIC_DISABLE_SERVICE))
        };
        let drv_deadline = drv_now + step_budget;
        let step_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drv.bus_recovery_mut().panic_disable_step(first_step, drv_deadline)
        }));
        match step_res {
            Err(_) => return PanicDisableOutcome::Failed("driver panicked during panic_disable_step".into()),
            Ok(None) => {
                use_generic = true;
                break; // Driver uses the generic retry loop below.
            }
            Ok(Some(Ok(unconfirmed))) => {
                first_step = false;
                last_step_err = None;
                if !enable_fb {
                    return PanicDisableOutcome::Unconfirmed;
                }
                if unconfirmed.is_empty() {
                    return PanicDisableOutcome::Confirmed;
                }
            }
            Ok(Some(Err(e))) => {
                first_step = false;
                tracing::warn!("Panic disable step error (retrying within budget): {e}");
                last_step_err = Some(e.to_string());
            }
        }
        let rem = coop_deadline.saturating_duration_since(Instant::now());
        let drv_rem = drv_coop_deadline.saturating_duration_since(drv.clock_now());
        if rem.is_zero() || drv_rem.is_zero() {
            break;
        }
        drv.clock_pause(rem.min(drv_rem).min(PANIC_DISABLE_SERVICE));
    }

    if !use_generic {
        let final_unconfirmed = drv.unconfirmed_disable_joints();
        tracing::error!(
            "Bounded panic disable cooperative budget ({} ms) exhausted{}; unconfirmed motors: [{}]",
            coop_budget.as_millis(),
            last_step_err.as_deref().map(|e| format!(" after error ({e})")).unwrap_or_default(),
            format_joint_list(&final_unconfirmed)
        );
        return PanicDisableOutcome::Unconfirmed;
    }

    // Generic retry loop for drivers that do not implement `panic_disable_step`: transmit
    // bounded protective disable frames until the driver finishes or `coop_deadline` is spent.
    let mut any_send_ok = false;
    let mut last_err: Option<String> = None;
    let mut attempt = 0u32;
    loop {
        let now = Instant::now();
        if now >= hard_deadline {
            if attempt == 0 {
                return PanicDisableOutcome::TimedOut;
            }
            break;
        }
        if attempt > 0 && coop_deadline.saturating_duration_since(now).is_zero() {
            break;
        }
        attempt += 1;
        let begin_res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let now = Instant::now();
            if now >= hard_deadline || (attempt > 1 && now >= coop_deadline) {
                return Err(None);
            }
            drv.begin_disable_torque_budgeted(
                crate::driver::DisableKind::DisableProtective {
                    reason: crate::driver::DisableProtectiveReason::ProcessExit,
                },
                0,
            )
            .map_err(Some)
        }));
        match begin_res {
            Err(_) => return PanicDisableOutcome::Failed("driver panicked during begin_torque(false)".into()),
            Ok(Err(None)) => {
                if attempt == 1 {
                    return PanicDisableOutcome::TimedOut;
                }
                break;
            }
            Ok(Err(Some(e))) => {
                tracing::warn!("Panic disable begin_torque(false) attempt {attempt} failed (retrying within budget): {e}");
                last_err = Some(e.to_string());
            }
            Ok(Ok(mut progress)) => {
                any_send_ok = true;
                while matches!(progress, crate::driver::TorqueProgress::Pending { .. }) {
                    let drv_now = drv.clock_now();
                    let now = Instant::now();
                    let rem = coop_deadline
                        .saturating_duration_since(now)
                        .min(hard_deadline.saturating_duration_since(now));
                    if rem.is_zero() {
                        break;
                    }
                    let drv_deadline = drv_now + rem;
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
                        let rem = coop_deadline.saturating_duration_since(Instant::now());
                        if rem.is_zero() {
                            break;
                        }
                        std::thread::sleep(rem.min(PANIC_DISABLE_SERVICE));
                    }
                }
                if matches!(
                    progress,
                    crate::driver::TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed)
                ) {
                    return if enable_fb {
                        PanicDisableOutcome::Confirmed
                    } else {
                        PanicDisableOutcome::Unconfirmed
                    };
                }
            }
        }

        if !enable_fb && any_send_ok {
            return PanicDisableOutcome::Unconfirmed;
        }
        let rem = coop_deadline.saturating_duration_since(Instant::now());
        if rem.is_zero() {
            break;
        }
        std::thread::sleep(rem.min(PANIC_DISABLE_SERVICE));
    }

    let unconfirmed = drv.unconfirmed_disable_joints();
    tracing::error!(
        "Bounded panic disable budget ({} ms) exhausted; unconfirmed motors: [{}]{}",
        coop_budget.as_millis(),
        format_joint_list(&unconfirmed),
        last_err.as_deref().map(|e| format!(" (last error: {e})")).unwrap_or_default()
    );
    if any_send_ok {
        PanicDisableOutcome::Unconfirmed
    } else {
        PanicDisableOutcome::Failed(last_err.unwrap_or_else(|| "panic disable send failed".into()))
    }
}

/// Outcome of one [`SupervisorInner::service_torque_transition`] call, with the exact
/// RX work performed by this call (None when no bus receive pass ran).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TransitionPoll {
    pub progress: Result<crate::rt_core::ops::TorqueProgress, &'static str>,
    pub rx: Option<(u16, bool)>,
    pub broker_owned: bool,
}

/// Owner-owned torque decision state (never inferred from motor feedback).
///
/// Holds are the default while stopped whenever `torque_intent == On`. Transitions to `Off`
/// ONLY by an accepted explicit disable (passing the rest-pose gate when stopped) or
/// `DisableProtective` (thermal / process exit / panic). A later enable sets `On`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TorqueIntent {
    Off,
    On,
}

/// Persistent enable-lifecycle state owned by the single enable transition reducer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnableLifecycle {
    /// Torque is off by accepted disable (or driverless startup).
    Disarmed,
    /// Enable transition reducer is active (`Enable` / `Validate`).
    Enabling,
    /// Enable and post-enable validation succeeded; arm is armed.
    Armed,
    /// Enable failed or was interrupted at any stage: queued enable work cancelled,
    /// `ENABLE_FAILED` latched, and measured-position holds are serviced for EVERY joint.
    FailedHold,
}

/// Maximum time an energized joint may remain with an unverified control mode before latching a
/// non-recoverable fault naming the joint (L2).
pub const UNVERIFIED_MODE_READBACK_TIMEOUT: Duration = Duration::from_millis(200);

/// Consecutive stopped-hold `driver.try_lock()` skips (0.5 s at 50 Hz) before escalating with
/// a counted warning and latching a non-recoverable [`FaultCode::HardwareFault`].
pub const STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS: u32 = 25;

/// Inline per-joint measured-position hold state (0 heap allocations on RT).
#[derive(Debug, Clone, Copy)]
pub struct MeasuredHoldTracker {
    /// Fixed captured target position per joint while feedback stays fresh (<= 100 ms).
    /// Cleared to `None` when feedback goes stale/missing, continuity breaks across a `> 100 ms`
    /// gap, or mode becomes unverified, and recaptured when fresh feedback returns.
    pub captured_positions: [Option<f32>; crate::rt_core::mailbox::MAX_DOF],
    /// Driver-reported per-joint feedback continuity epoch at capture time (detects `> 100 ms`
    /// gaps even if feedback resumed before the next service pass).
    pub captured_continuity_epoch: [Option<u64>; crate::rt_core::mailbox::MAX_DOF],
    /// Driver-clock timestamp when this joint's capture was last confirmed continuously fresh.
    pub last_seen_fresh_at: [Option<Instant>; crate::rt_core::mailbox::MAX_DOF],
    /// Driver-clock timestamp when a joint was first seen with an unverified hold mode
    /// (L2: per-tick `RID_CTRL_MODE` readback, non-recoverable fault at 200 ms).
    pub unverified_mode_since: [Option<Instant>; crate::rt_core::mailbox::MAX_DOF],
    /// Bitmask of joints that have already latched the 200 ms unverified-mode fault.
    pub unverified_mode_faulted_mask: u64,
    /// Owner-held bitmask of joints with pending/queued stopped-hold TX frames on the driver (L3).
    pub owner_pending_hold_mask: u64,
    /// Owner-held active stopped-hold mask for the current pass (L3).
    pub owner_hold_mask: u64,
    /// Owner-held inline stopped-hold target queue for the current pass (L3; zero alloc).
    pub owner_hold_targets: [f32; crate::rt_core::mailbox::MAX_DOF],
    /// Most recent per-joint hold status observed during hold evaluation/servicing.
    pub joint_status: [crate::driver::JointHoldStatus; crate::rt_core::mailbox::MAX_DOF],
    /// Original failure cause stored on `FailedHold` entry so live `ENABLE_FAILED` messages
    /// reflect updated per-joint hold status without relatching or bumping `epoch_id`.
    pub enable_failed_cause: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    /// Consecutive stopped-hold TX ticks skipped due to `driver.try_lock()` contention.
    pub consecutive_lock_contended: u32,
    /// True once the current contiguous contention streak has escalated (>= 25 skips).
    pub lock_contended_escalated: bool,
    pub dof: usize,
}

impl MeasuredHoldTracker {
    pub fn new(dof: usize) -> Self {
        Self {
            captured_positions: [None; crate::rt_core::mailbox::MAX_DOF],
            captured_continuity_epoch: [None; crate::rt_core::mailbox::MAX_DOF],
            last_seen_fresh_at: [None; crate::rt_core::mailbox::MAX_DOF],
            unverified_mode_since: [None; crate::rt_core::mailbox::MAX_DOF],
            unverified_mode_faulted_mask: 0,
            owner_pending_hold_mask: 0,
            owner_hold_mask: 0,
            owner_hold_targets: [0.0; crate::rt_core::mailbox::MAX_DOF],
            joint_status: [crate::driver::JointHoldStatus::SkippedOff; crate::rt_core::mailbox::MAX_DOF],
            enable_failed_cause: crate::safety::BoundedText::empty(),
            consecutive_lock_contended: 0,
            lock_contended_escalated: false,
            dof: dof.min(crate::rt_core::mailbox::MAX_DOF),
        }
    }

    pub fn clear(&mut self) {
        self.captured_positions.fill(None);
        self.captured_continuity_epoch.fill(None);
        self.last_seen_fresh_at.fill(None);
        self.unverified_mode_since.fill(None);
        self.unverified_mode_faulted_mask = 0;
        self.owner_pending_hold_mask = 0;
        self.owner_hold_mask = 0;
        self.owner_hold_targets.fill(0.0);
        self.joint_status.fill(crate::driver::JointHoldStatus::SkippedOff);
        self.enable_failed_cause = crate::safety::BoundedText::empty();
        self.consecutive_lock_contended = 0;
        self.lock_contended_escalated = false;
    }

    /// Append concise per-joint hold status (`[J1:held,...]`) into a bounded text
    /// buffer without heap allocation.
    pub fn write_summary(&self, buf: &mut crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>) {
        if self.dof == 0 {
            return;
        }
        buf.push_str("[");
        for i in 0..self.dof {
            if i > 0 {
                buf.push_str(",");
            }
            let _ = core::fmt::write(
                buf,
                format_args!(
                    "J{}:{}",
                    i + 1,
                    self.joint_status[i].short_tag()
                ),
            );
        }
        buf.push_str("]");
    }

    /// Format the live `ENABLE_FAILED` message (`Enable failed [J1:...]: <cause>`) without heap
    /// allocation so observations reflect current per-joint hold status.
    pub fn write_fault_message(
        &self,
        buf: &mut crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    ) {
        *buf = crate::safety::BoundedText::empty();
        buf.push_str("Enable failed ");
        self.write_summary(buf);
        if !self.enable_failed_cause.is_empty() {
            buf.push_str(": ");
            buf.push_str(self.enable_failed_cause.as_str());
        }
    }
}

/// Inline owner-held driver snapshot refreshed whenever the owner already holds the driver lock
/// (`poll_hardware_rx`, torque transitions, `enter_failed_hold`, `apply_estop_latch_with_driver`).
/// Allows stopped-hold capture and MIT stopped-hold preparation to run with zero heap allocation
/// and zero driver mutex acquisition (Rule R1).
#[derive(Debug, Clone, Copy)]
pub(crate) struct OwnerDriverSnapshot {
    pub(crate) valid: bool,
    pub(crate) captured_at: Option<Instant>,
    pub(crate) fresh_samples: [Option<(f32, u64)>; crate::rt_core::mailbox::MAX_DOF],
    pub(crate) feedback_ages: [Option<Duration>; crate::rt_core::mailbox::MAX_DOF],
    pub(crate) hold_mode_verified: [bool; crate::rt_core::mailbox::MAX_DOF],
}

impl OwnerDriverSnapshot {
    pub(crate) fn new(_dof: usize) -> Self {
        Self {
            valid: false,
            captured_at: None,
            fresh_samples: [None; crate::rt_core::mailbox::MAX_DOF],
            feedback_ages: [None; crate::rt_core::mailbox::MAX_DOF],
            hold_mode_verified: [false; crate::rt_core::mailbox::MAX_DOF],
        }
    }

    #[inline]
    pub(crate) fn invalidate(&mut self) {
        self.valid = false;
        self.captured_at = None;
        self.fresh_samples.fill(None);
        self.feedback_ages.fill(None);
        self.hold_mode_verified.fill(false);
    }

    /// Captures all driver-owned state needed for stopped-hold evaluation and MIT stopped-hold
    /// preparation using caller-provided preallocated scratch buffers (zero heap allocations).
    pub(crate) fn capture_from_driver(
        drv: &dyn crate::driver::ArmDriver,
        dof: usize,
        feedback_numeric_valid: bool,
        measured_positions: &[f32],
        ages_buf: &mut Vec<Option<Duration>>,
        bounds_buf: &mut Vec<Option<Instant>>,
    ) -> Self {
        use crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE;
        let dof = dof.min(crate::rt_core::mailbox::MAX_DOF);
        let mut snap = Self::new(dof);
        let drv_now = drv.clock_now();
        snap.captured_at = Some(drv_now);
        let has_drv_fresh = drv.has_driver_fresh_position_api();
        drv.feedback_ages_into(ages_buf);
        for j in 0..dof {
            snap.feedback_ages[j] = ages_buf.get(j).copied().flatten();
        }
        let mut has_bounds = false;
        if !has_drv_fresh {
            drv.feedback_queue_bounds_into(bounds_buf);
            has_bounds = !bounds_buf.is_empty();
        }
        for j in 0..dof {
            snap.fresh_samples[j] = if has_drv_fresh {
                drv.fresh_measured_sample_for(j, STOPPED_DISABLE_FEEDBACK_MAX_AGE)
            } else {
                let age_ok = snap.feedback_ages[j]
                    .is_some_and(|a| a <= STOPPED_DISABLE_FEEDBACK_MAX_AGE);
                let bound_ok = !has_bounds
                    || bounds_buf.get(j).copied().flatten().is_some_and(|b| {
                        drv_now.saturating_duration_since(b) <= STOPPED_DISABLE_FEEDBACK_MAX_AGE
                    });
                if feedback_numeric_valid && age_ok && bound_ok {
                    measured_positions
                        .get(j)
                        .copied()
                        .filter(|p| p.is_finite())
                        .map(|p| (p, 0u64))
                } else {
                    None
                }
            };
            snap.hold_mode_verified[j] = drv.joint_mode().joint_hold_mode_verified(j);
        }
        snap.valid = true;
        snap
    }
}

/// Origin of a [`TorqueTransition::Enable`] handshake so broker torque commands,
/// legacy WS torque commands, and park `holding_after_clear` rearm share the single bounded
/// enable authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnableOrigin {
    BrokerTransition,
    ParkRearm { action_id: u64, stop_gen: u64 },
    LegacyWs {
        token: u64,
        slot: u8,
        stop_gen: u64,
        prev_torque_intent: TorqueIntent,
        deadline: Instant,
    },
    /// Enable cancellation retained in the reducer after Stop, timeout, or owner transfer when
    /// `driver.try_lock()` was contended; drains `drv.abort_enable()` with zero enable TX.
    DetachedCancel,
}

/// Origin of a [`TorqueTransition::Disable`] handshake so broker torque-off, legacy WS
/// torque-off, and park torque-off share the single bounded, rest-pose-gated disable authority.
/// Per-motor disable evidence extends this single choke point rather than introducing
/// a parallel disable state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisableOrigin {
    BrokerTransition,
    Park {
        watchdog: bool,
        action_id: Option<u64>,
        expected_stop_gen: Option<u64>,
    },
    LegacyWs {
        token: u64,
        slot: u8,
        deadline: Instant,
    },
    /// Legacy WS disable whose waiter timed out or was detached on owner handoff.
    /// Carries `deadline: Some(dl)` while undispatched so an expired undispatched request aborts
    /// cleanly under a single driver lock; `deadline: None` once frames are dispatched or protective.
    Detached { deadline: Option<Instant> },
}

/// Number of preallocated legacy WS torque completion slots on [`SupervisorInner`].
/// Allocated once at construction so settling a transition on the control release performs
/// strictly zero heap allocations and zero deallocations.
pub(crate) const LEGACY_TORQUE_SLOTS: usize = 4;

/// Typed, copyable refusal captured atomically at transition completion on the control release
/// (zero heap allocation) and formatted into the wire-compatible `ApiError` off-lock.
#[derive(Debug, Clone, Copy)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum LegacyTorqueFailure {
    TorqueOffWouldDrop(crate::supervisor::StoppedTorqueOffRefusal),
    EnableFailed {
        detail: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
        feedback_unavailable: bool,
        estop_interrupted: bool,
    },
    DisableFailed {
        detail: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
    },
    Stopped,
    LeaseLost,
    EpochMismatch,
    FeedbackUnavailable,
    EStopActive,
    StaticCode(&'static str),
}

/// Lossless terminal outcome of a legacy WS `/api/v1/torque` transition captured atomically at
/// completion time inside the owner (zero heap allocation on the control release).
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)] // inline bounded failure keeps control-release settling zero-alloc
pub(crate) enum LegacyTorqueOutcome {
    Accepted {
        epoch_id: u64,
        safety_state: std::sync::Arc<SafetyState>,
        is_torqued: bool,
        control_mode_gravity_comp: bool,
    },
    Failed {
        failure: LegacyTorqueFailure,
        epoch_id: u64,
        safety_state: std::sync::Arc<SafetyState>,
    },
    OwnerTransferred,
}

const SLOT_IDLE: u64 = 0;
const SLOT_IN_FLIGHT: u64 = 1;
const SLOT_SETTLED: u64 = 2;
const SLOT_ABANDONED: u64 = 3;
const SLOT_STATE_MASK: u64 = 0b11;

#[inline]
const fn pack_slot(token: u64, state: u64) -> u64 {
    (token << 2) | (state & SLOT_STATE_MASK)
}

/// Preallocated, strictly wait-free per-request completion slot shared between a legacy WS waiter
/// and the control tick. Packs `(token << 2) | state` into a single `AtomicU64` with zero mutexes,
/// zero `Notify` waiter locks, and zero heap allocs/frees on the control release.
/// Off-lock waiter wakeup is performed by `ArmSupervisor::legacy_torque_notify` AFTER dropping
/// `SupervisorLock::write`.
pub(crate) struct LegacyTorqueSlot {
    packed: std::sync::atomic::AtomicU64,
    outcome: std::cell::UnsafeCell<Option<LegacyTorqueOutcome>>,
}

// SAFETY: Access to `outcome` is strictly serialized by the single-word atomic state machine:
// - The owner (serialized by `SupervisorLock::write`) writes `outcome` ONLY while
//   `packed == pack_slot(token, SLOT_IN_FLIGHT)` and publishes via
//   `SLOT_IN_FLIGHT -> SLOT_SETTLED` (Release); on concurrent waiter abandon (`Err(SLOT_ABANDONED)`),
//   the waiter never touches `outcome` and the owner clears it before storing `SLOT_IDLE` (Release).
// - The single WS waiter owning `token` reads `outcome` ONLY after observing
//   `packed == pack_slot(token, SLOT_SETTLED)` (Acquire) and releases to `SLOT_IDLE` (Release).
unsafe impl Sync for LegacyTorqueSlot {}

impl LegacyTorqueSlot {
    pub(crate) fn new() -> Self {
        Self {
            packed: std::sync::atomic::AtomicU64::new(SLOT_IDLE),
            outcome: std::cell::UnsafeCell::new(None),
        }
    }

    pub(crate) fn try_reserve(&self, token: u64) -> bool {
        debug_assert!(token != 0 && token < (1u64 << 62));
        self.packed
            .compare_exchange(
                SLOT_IDLE,
                pack_slot(token, SLOT_IN_FLIGHT),
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    pub(crate) fn release_unstarted(&self, token: u64) {
        let _ = self
            .packed
            .compare_exchange(
                pack_slot(token, SLOT_IN_FLIGHT),
                SLOT_IDLE,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .or_else(|_| {
                self.packed.compare_exchange(
                    pack_slot(token, SLOT_ABANDONED),
                    SLOT_IDLE,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
            });
    }

    /// Wait-free settlement on the control release (zero mutexes, zero allocs).
    /// Returns `true` iff an outcome was published for a live waiter (`SLOT_IN_FLIGHT -> SLOT_SETTLED`).
    /// When the waiter already abandoned the slot, skips invoking `make` and returns the slot to `Idle`.
    pub(crate) fn settle_with(&self, token: u64, make: impl FnOnce() -> LegacyTorqueOutcome) -> bool {
        let in_flight = pack_slot(token, SLOT_IN_FLIGHT);
        let settled = pack_slot(token, SLOT_SETTLED);
        let abandoned = pack_slot(token, SLOT_ABANDONED);
        let cur = self.packed.load(Ordering::Acquire);
        if cur == abandoned {
            let _ = self
                .packed
                .compare_exchange(abandoned, SLOT_IDLE, Ordering::AcqRel, Ordering::Acquire);
            return false;
        }
        if cur != in_flight {
            return false;
        }
        // SAFETY: owner has exclusive write access to `outcome` while in `SLOT_IN_FLIGHT`.
        unsafe { *self.outcome.get() = Some(make()) };
        match self
            .packed
            .compare_exchange(in_flight, settled, Ordering::Release, Ordering::Acquire)
        {
            Ok(_) => true,
            Err(v) if v == abandoned => {
                // Waiter abandoned concurrently; waiter will not read `outcome`.
                // SAFETY: exclusive ownership stays with owner until `SLOT_IDLE`.
                unsafe { *self.outcome.get() = None };
                self.packed.store(SLOT_IDLE, Ordering::Release);
                false
            }
            Err(_) => false,
        }
    }

    /// Returns `true` iff an `OwnerTransferred` outcome was published for a live waiter.
    pub(crate) fn settle_owner_transferred(&self) -> bool {
        let cur = self.packed.load(Ordering::Acquire);
        let st = cur & SLOT_STATE_MASK;
        let tok_bits = cur & !SLOT_STATE_MASK;
        match st {
            SLOT_ABANDONED => {
                let _ = self
                    .packed
                    .compare_exchange(cur, SLOT_IDLE, Ordering::AcqRel, Ordering::Acquire);
                false
            }
            SLOT_IN_FLIGHT => {
                // SAFETY: owner has exclusive write access to `outcome` while in `SLOT_IN_FLIGHT`.
                unsafe { *self.outcome.get() = Some(LegacyTorqueOutcome::OwnerTransferred) };
                match self.packed.compare_exchange(
                    cur,
                    tok_bits | SLOT_SETTLED,
                    Ordering::Release,
                    Ordering::Acquire,
                ) {
                    Ok(_) => true,
                    Err(v) if v == (tok_bits | SLOT_ABANDONED) => {
                        // SAFETY: waiter abandoned concurrently and will not read `outcome`.
                        unsafe { *self.outcome.get() = None };
                        self.packed.store(SLOT_IDLE, Ordering::Release);
                        false
                    }
                    Err(_) => false,
                }
            }
            _ => false,
        }
    }

    pub(crate) fn take_settled(&self, token: u64) -> Option<LegacyTorqueOutcome> {
        if self.packed.load(Ordering::Acquire) == pack_slot(token, SLOT_SETTLED) {
            // SAFETY: single waiter owning `token` observed `SLOT_SETTLED` with Acquire.
            let out = unsafe { (*self.outcome.get()).take() };
            self.packed.store(SLOT_IDLE, Ordering::Release);
            out
        } else {
            None
        }
    }

    pub(crate) fn abandon(&self, token: u64) {
        let in_flight = pack_slot(token, SLOT_IN_FLIGHT);
        let settled = pack_slot(token, SLOT_SETTLED);
        let abandoned = pack_slot(token, SLOT_ABANDONED);
        match self
            .packed
            .compare_exchange(in_flight, abandoned, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => {}
            Err(v) if v == settled => {
                // SAFETY: owner already published `SLOT_SETTLED` and will not touch `outcome`.
                unsafe {
                    let _ = (*self.outcome.get()).take();
                }
                self.packed.store(SLOT_IDLE, Ordering::Release);
            }
            Err(_) => {}
        }
    }
}

/// Immediate start result of a legacy WS `/api/v1/torque` command under the short write guard.
#[allow(clippy::large_enum_variant)] // inline bounded failure returned under short write guard
pub(crate) enum LegacyTorqueStart {
    ImmediateAccepted {
        epoch_id: u64,
        safety_state: SafetyState,
        is_torqued: bool,
        control_mode_gravity_comp: bool,
    },
    ImmediateRejected {
        failure: LegacyTorqueFailure,
        epoch_id: u64,
        safety_state: SafetyState,
    },
    Pending {
        token: u64,
        slot_idx: u8,
        slot: std::sync::Arc<LegacyTorqueSlot>,
    },
}

/// Immediate outcome of starting a park `holding_after_clear` rearm transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParkRearmStart {
    /// Confirmed immediately (driverless or instant-confirm hardware); `commanded_positions`
    /// is seeded from fresh `measured_positions` and ready for off-lock park planning.
    ConfirmedNow,
    /// Driver enable handshake is pending confirmation via `TorqueTransition::Enable`.
    Pending,
}

/// Maximum retries of a background hardware disable before latching `DisableUnconfirmed`.
pub const BACKGROUND_DISABLE_MAX_RETRIES: u8 = 8;
/// Spacing between bounded background hardware disable retries (driver clock).
pub const BACKGROUND_DISABLE_RETRY_INTERVAL: Duration = Duration::from_millis(25);
/// Upper time bound for bounded background hardware disable retries before latching a fault.
pub const BACKGROUND_DISABLE_MAX_WINDOW: Duration = Duration::from_millis(500);

/// Single executor-owned bounded retry state machine for a background hardware disable whose
/// initial disable attempt failed on transport TX.
///
/// Every retry is funneled strictly through the driver's single `begin_disable_torque_budgeted(kind, 0)`
/// (`send(TxKind::DisableRequested | DisableProtective, ...)`) choke point carrying the ORIGINAL
/// `DisableKind` (`started_at`) so the rest-pose gate and single 1 s lifetime apply uniformly.
/// Once [`Self::Exhausted`] is entered, retries stop permanently until an accepted disable or
/// explicit recovery clears the fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BackgroundDisableRetry {
    Active {
        attempts: u8,
        kind: crate::driver::DisableKind,
        started_at: Instant,
        next_retry_at: Instant,
        /// When `Some(expected_stop_gen)`, this retry belongs to a client-initiated park and MUST
        /// dispatch via `begin_client_park_disable_with_kind` (cancelling immediately if Stop arrived).
        client_park: Option<Option<u64>>,
    },
    Exhausted,
}

/// Authority tuple captured from [`AdmittedProof`] when a `TorqueTransition::Enable` starts,
/// re-verified before every subsequent enable retry TX and before arming so a displaced lease,
/// disconnect, E-stop, fault, or epoch change aborts immediately into `FailedHold`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EnableAuthority {
    pub lease_generation: u64,
    pub owner_conn_id: u64,
    pub admitted_epoch: u64,
    pub admission_request_id: u64,
    pub stop_gen: u64,
}

/// Phase of the single budgeted enable transition reducer (`Enable -> Validate -> Armed | FailedHold`,
/// or `Cancelling` while retrying non-blocking `drv.abort_enable()` after Stop/timeout/handoff).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnableReducerPhase {
    /// Driving the driver's enable handshake (`begin_torque` / `poll_torque`).
    Enable,
    /// Driver enable completed; validating fresh feedback and enable state before arming.
    Validate,
    /// Cancellation in progress: zero enable TX allowed; retries `drv.abort_enable()` until
    /// `driver.try_lock()` succeeds so the driver never retains a live `torque_hs` blocking hold TX.
    Cancelling,
}

/// State of the single enable transition reducer shared by blocking `set_torque(true)`, RT
/// `begin_enable_torque_transition` / `service_torque_transition`, and park `holding_after_clear`
/// rearm (`EnableOrigin::ParkRearm`).
#[derive(Debug, Clone, Copy)]
pub struct EnableReducer {
    pub origin: EnableOrigin,
    pub phase: EnableReducerPhase,
    pub was_torqued: bool,
    pub prev_gripper_cmd: Option<f32>,
    pub authority: Option<EnableAuthority>,
    pub budgeted: bool,
    /// True iff this exact enable admission passed MIT qualification (sync `mit_gate` on legacy
    /// or admission-bound `MitGateProof` consumed on RT executor). Only this reducer's successful
    /// arming may set `mit.armed_ok`.
    pub mit_qualified: bool,
}

/// Result of one step of the unified enable reducer (stack-only, 0 heap allocs on RT).
#[allow(clippy::large_enum_variant)] // inline bounded text avoids RT heap allocation
enum EnableStepOutcome {
    Pending {
        wake: Instant,
    },
    Armed(SafetyState),
    Failed {
        rt_code: &'static str,
        message: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
        feedback_unavailable: bool,
        estop_interrupted: bool,
    },
}

/// A torque transition in flight: either the single budgeted enable reducer
/// (`Enable -> Validate -> Armed | FailedHold`) or an explicit/park/protective disable handshake
/// (rest-pose gate defer + bounded disable repeats within the 1 s request lifetime).
#[derive(Debug, Clone, Copy)]
pub enum TorqueTransition {
    /// Single budgeted enable transition reducer (`Enable -> Validate -> Armed | FailedHold`).
    Enable(EnableReducer),
    /// Driver disable handshake in flight (rest-pose gate wait or bounded disable repeats),
    /// carrying the `origin` (broker vs park) and original `DisableKind` (`started_at`) so any
    /// transport-failure background retry keeps the request's original timestamp and rest-pose gate.
    Disable {
        origin: DisableOrigin,
        kind: crate::driver::DisableKind,
    },
}

/// RT terminal code for a driver error during enable without moving the error (zero alloc).
fn enable_driver_err_code(e: &crate::driver::ArmDriverError) -> &'static str {
    use crate::driver::ArmDriverError as E;
    if crate::driver::is_estop_inhibited(e) {
        "ESTOP_ACTIVE"
    } else {
        match e {
            E::TorqueOffWouldDrop | E::DisableRequestExpired => "TORQUE_OFF_WOULD_DROP",
            E::LeaseFenced | E::EnableAuthorityExpired => "LEASE_LOST",
            E::Transport(_) => "DRIVER_WRITE_FAILED",
            _ => "ENABLE_FAILED",
        }
    }
}

/// Static protocol code for an admission refusal on RT torque paths (zero allocations).
fn torque_admission_error_code(e: &SafetyError) -> &'static str {
    match e {
        SafetyError::EStopActive => "ESTOP_ACTIVE",
        SafetyError::Stopped => "STOPPED",
        SafetyError::LeaseLost => "LEASE_LOST",
        SafetyError::EpochMismatch { .. } => "EPOCH_MISMATCH",
        SafetyError::StampAlreadyConsumed => "STAMP_REUSED",
        SafetyError::OwnerTransferred => "OWNER_TRANSFERRED",
        SafetyError::FaultActive(_) => "FAULT_ACTIVE",
        _ => "REJECTED",
    }
}

/// The terminal code of a refused or failed enable / disable step.
#[allow(dead_code)]
fn torque_error_code(e: &SafetyError) -> &'static str {
    match e {
        SafetyError::EStopActive => "ESTOP_ACTIVE",
        SafetyError::FeedbackUnavailable => "FEEDBACK_UNAVAILABLE",
        SafetyError::FaultActive(_) => "FAULT_ACTIVE",
        SafetyError::NotArmed(_) => "NOT_ARMED",
        SafetyError::TorqueOffWouldDrop { .. } => "TORQUE_OFF_WOULD_DROP",
        SafetyError::ClearFaultRejected(msg) if msg.starts_with("WATCHDOG_BUDGET_INSUFFICIENT") => {
            "WATCHDOG_BUDGET_INSUFFICIENT"
        }
        _ => "ENABLE_FAILED",
    }
}

/// Zero-allocation error returned by RT `clear_fault_executor_bounded_for_release_with_authority`.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // inline ClearFaultError keeps RT clear_fault strictly zero-alloc
pub(crate) enum ClearFaultWithAuthError {
    Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
    Clear(crate::safety::ClearFaultError),
}

/// Legacy internal error distinguishing M3 live authority revocation from `SafetyError`.
#[derive(Debug)]
pub(crate) enum ClearFaultLegacyInnerError {
    Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
    Safety(SafetyError),
}

/// Legacy confirmed clear_fault error distinguishing M3 live authority revocation from `ArmError`.
#[derive(Debug)]
pub(crate) enum ClearFaultLegacyWithAuthError {
    Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
    Arm(crate::supervisor::ArmError),
}

impl From<SafetyError> for ClearFaultLegacyInnerError {
    fn from(e: SafetyError) -> Self {
        Self::Safety(e)
    }
}

impl From<crate::safety::ClearFaultError> for ClearFaultWithAuthError {
    fn from(e: crate::safety::ClearFaultError) -> Self {
        Self::Clear(e)
    }
}

impl SupervisorInner {

    /// Single unified failure transition of the enable reducer (shared by begin errors, poll
    /// errors, interruption, and post-enable validation failures; zero heap allocations on RT):
    /// 1. Preserves any unapplied pending E-stop latch before snapshotting holds.
    /// 2. Cancels queued enable work on the driver (`abort_enable`) without sending any rollback
    ///    disable frame at any pose.
    /// 3. Evaluates per-motor torque evidence and per-joint measured-position holds independently
    ///    (`snapshot_measured_hold`) and drops queued driver holds outside the eligible hold mask.
    /// 4. Transitions `enable_lifecycle` to [`EnableLifecycle::FailedHold`], keeps `torque_intent = On`
    ///    and `is_torqued = true` (every joint held), and latches non-recoverable
    ///    [`FaultCode::EnableFailed`] carrying per-joint hold status while preserving
    ///    any existing E-stop or thermal latch.
    fn enter_failed_hold(
        &mut self,
        _was_torqued: bool,
        write_cause: impl FnOnce(&mut crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>),
    ) -> crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }> {
        // 1. Preserve any pending E-stop before snapshotting holds (consume_pending_estop clears
        //    the previous hold snapshot via apply_stop_policy).
        if self.estop_request_unapplied() {
            self.consume_pending_estop();
        }
        // 2 & 3. Cancel queued enable work (NEVER send automatic rollback disable frames),
        // refresh owner-held driver snapshot in the SAME single borrow, capture per-joint holds from
        // owner data (no extra driver lock), and drop queued driver holds outside `hold_mask`.
        // Zero-lock on `DriverSlot::Exclusive`; on `DriverSlot::Legacy` mutex contention
        // (`WouldBlock`), retain `DetachedCancel` so a subsequent control tick drains
        // `drv.abort_enable()` with zero enable TX and unblocks measured-hold writes.
        let mut cancel_contended = false;
        if let Some(mut d) = self.driver.take() {
            match d.try_with_mut(|drv| {
                drv.abort_enable();
                self.refresh_owner_driver_snapshot_from_ref(&*drv);
                let (hold_mask, _) = self.capture_measured_hold_from_owner();
                drv.tx_mut().drop_pending_holds_outside_mask(hold_mask);
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.drain_from_driver(drv);
            }) {
                Ok(()) => {}
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    cancel_contended = true;
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
            }
            self.driver = Some(d);
        } else {
            let _ = self.snapshot_measured_hold();
        }
        if cancel_contended {
            self.detached_cancel_restore_intent = None;
            self.retain_detached_enable_cancel_if_clear(_was_torqued);
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue(crate::torque::shadow::ShadowInputKind::HandshakeResult {
                live_op_id: self.shadow_enable_op_id,
                confirmed_mask: crate::torque::ChannelMask::EMPTY,
                outcome: crate::torque::HandshakeOutcome::Failed,
                live_intent: self.shadow_live_intent(),
                live_is_torqued: self.is_torqued,
                live_stopped: self.safety.state().is_stopped(),
            });
        // 4. Record captured measured positions and enter FailedHold (no cached-command fallback).
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        for j in 0..dof {
            if let Some(pos) = self.measured_hold.captured_positions[j] {
                if j < self.measured_positions.len() {
                    self.measured_positions[j] = pos;
                }
                if j < self.commanded_positions.len() {
                    self.commanded_positions[j] = pos;
                }
                if j < self.target_positions.len() {
                    self.target_positions[j] = pos;
                }
            }
        }
        self.commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.enable_lifecycle = EnableLifecycle::FailedHold;
        self.torque_intent = TorqueIntent::On;
        self.is_torqued = true;
        self.pending_rearm = true;
        self.disable_pending = false;
        self.active_disable_kind = None;
        self.last_disable_attempt = None;

        // 5. Store cause and latch non-recoverable ENABLE_FAILED with per-joint torque & hold
        //    status first so the joint report is never truncated by a verbose driver error string.
        let mut cause = crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
        write_cause(&mut cause);
        self.measured_hold.enable_failed_cause = cause;
        let mut msg = crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
        self.measured_hold.write_fault_message(&mut msg);
        let msg_copy = msg;
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::formatted(FaultCode::EnableFailed, |buf| {
                buf.push_str(msg_copy.as_str());
            }),
        );
        self.record_applied_qualifying_fault_stop();
        msg
    }

    fn settle_legacy_ws_disable_failed(
        &mut self,
        token: u64,
        slot: u8,
        code: &'static str,
        detail: Option<crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>>,
    ) {
        let failure = match code {
            "TORQUE_OFF_WOULD_DROP" => {
                let refusal = self
                    .last_stopped_torque_off_refusal
                    .or_else(|| {
                        self.may_disable_torque_while_stopped_nonblocking()
                            .ok()
                            .and_then(Result::err)
                    })
                    .unwrap_or(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                self.last_stopped_torque_off_refusal = Some(refusal);
                LegacyTorqueFailure::TorqueOffWouldDrop(refusal)
            }
            "ESTOP_ACTIVE" => LegacyTorqueFailure::EStopActive,
            "FEEDBACK_UNAVAILABLE" => LegacyTorqueFailure::FeedbackUnavailable,
            "DEADLINE_EXPIRED" | "DRIVER_BUSY" | "SUPERSEDED" | "STOPPED" | "MOTION_BUSY" => {
                LegacyTorqueFailure::StaticCode(code)
            }
            _ => {
                if let Some(detail) = detail {
                    LegacyTorqueFailure::DisableFailed { detail }
                } else {
                    LegacyTorqueFailure::StaticCode(code)
                }
            }
        };
        self.settle_legacy_ws_failed(token, slot, failure);
    }

    fn classify_legacy_enable_failure(
        rt_code: &'static str,
        detail: crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
        feedback_unavailable: bool,
        estop_interrupted: bool,
    ) -> LegacyTorqueFailure {
        if feedback_unavailable {
            LegacyTorqueFailure::FeedbackUnavailable
        } else if estop_interrupted || rt_code == "ESTOP_ACTIVE" {
            LegacyTorqueFailure::EStopActive
        } else if matches!(rt_code, "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED") {
            LegacyTorqueFailure::LeaseLost
        } else if rt_code == "EPOCH_MISMATCH" {
            LegacyTorqueFailure::EpochMismatch
        } else if rt_code == "STOPPED" {
            LegacyTorqueFailure::Stopped
        } else if rt_code == "DEADLINE_EXPIRED" {
            LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED")
        } else {
            LegacyTorqueFailure::EnableFailed {
                detail,
                feedback_unavailable,
                estop_interrupted,
            }
        }
    }

}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DisableExpiryDecision {
    Aborted,
    KeepDetachedDispatched,
    KeepDetachedWithDeadline,
    ContendedRetry,
}

impl SupervisorInner {

    /// Handles an aborted multi-tick disable after >= 1 joint already received a disable frame.
    /// Does NOT arm background disable retries: remaining energized joints stay held (`is_torqued =
    /// true`, `torque_intent = On`), and a fault naming exact `applied` / `uncertain` / `not
    /// attempted` joints is latched unless E-stop is already active.
    fn handle_partial_disable_abort(
        &mut self,
        e: crate::driver::ArmDriverError,
    ) -> (
        crate::safety::BoundedText<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>,
        &'static str,
    ) {
        self.clear_active_disable_retry_state();
        self.is_torqued = true;
        self.torque_intent = TorqueIntent::On;
        let mut err_detail =
            crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
        e.write_bounded(&mut err_detail);
        if !self.e_stop_latched.load(Ordering::SeqCst)
            && !matches!(self.safety.state(), crate::safety::SafetyState::EStop)
        {
            let detail =
                crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                    buf.push_str(err_detail.as_str());
                });
            self.safety.latch(crate::safety::FaultKind::Other, detail);
        }
        let _ = self.capture_measured_hold_from_owner();
        (err_detail, "DRIVER_WRITE_FAILED")
    }

    /// A disable step failed: `disable_pending` stays set for transport errors or committed
    /// partial dispatches (`committed_partial`), while an uncommitted choke-point refusal
    /// (`TorqueOffWouldDrop` / `DisableRequestExpired`) cleanly ends the request.
    fn disable_step_failed_inner(
        &mut self,
        e: crate::driver::ArmDriverError,
        committed_partial: bool,
    ) -> &'static str {
        let refused = matches!(
            e,
            crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired
        );
        if refused && !committed_partial {
            self.disable_pending = false;
            self.active_disable_kind = None;
            self.last_disable_attempt = None;
            self.background_disable_retry = None;
            if self.torque_off_stopped() {
                // Non-blocking refresh of refusal details so refusal handling never stalls on a
                // contended driver mutex under `SupervisorLock::write()` or `armd-control`.
                let _ = self.eval_stopped_torque_off_nonblocking();
            }
        }
        let already_stopped = self.torque_off_stopped();
        let detail = (!already_stopped && !refused).then(|| {
            crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                buf.push_str("Torque disable transmission failed: ");
                e.write_bounded(buf);
            })
        });
        let code = crate::driver::handshake::rt_progress(false, Err(e))
            .err()
            .unwrap_or("HARDWARE_FAULT");
        if let Some(detail) = detail {
            self.safety.latch(crate::safety::FaultKind::Other, detail);
        }
        code
    }

}

fn clear_fault_driver_error(e: &crate::driver::ArmDriverError) -> crate::safety::ClearFaultDriverError {
    e.to_clear_fault_error()
}

#[cfg(test)]
mod panic_disable_tests;
