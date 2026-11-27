//! Driver transitions of the supervisor: driver attachment, feedback intake, driver limits, torque enable/disable and fault clearing (track A).
//!
//! Moved from `supervisor.rs` without changes (armd 1.6 RT-core foundation); private
//! methods are `pub(super)` so the rest of the supervisor keeps calling them.

#[allow(unused_imports)]
use super::*;

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
    pub fn with_driver(mut self, mut driver: Box<dyn ArmDriver>) -> Self {
        driver.set_estop_state(self.estop_state.clone());
        driver.set_estop_gate(self.estop_gate());
        driver.set_gripper_index(self.config.gripper_index());
        driver.joint_mode_mut().set_strict_mode_handover(self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
        if driver.capabilities().load_feedback {
            self.loads = vec![f32::NAN; self.config.dof];
        }
        let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &self.joint_zero_offsets));
        self.driver_caps = driver.capabilities();
        self.apply_driver_limits(driver.as_ref());
        self.install_driver(driver);
        self.bump_motion_config_revision();
        self.refresh_schema_cache();
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
        for (i, slot) in self.cached_driver_pos_limits.iter_mut().enumerate() {
            *slot = pos_limits.get(i).copied().flatten();
        }
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

        if candidate.validate_runtime_narrowed().is_err()
            && self.base_gripper_policy.is_some()
            && (self.gripper_calib_offset.is_some() || self.gripper_calib_jaw_opening.is_some())
        {
            let mut base_test = candidate.clone();
            base_test.gripper_policy = self.base_gripper_policy.clone();
            if base_test.validate_runtime_narrowed().is_ok() {
                tracing::warn!("Driver limits invalidated outdated calibrated gripper policy; reverting to base uncalibrated policy");
                self.gripper_calib_invalid = true;
                self.gripper_calib_offset = None;
                self.gripper_calib_jaw_opening = None;
                self.config.gripper_policy = self.base_gripper_policy.clone();
                candidate = base_test;
            }
        }

        if let Err(err) = candidate.validate_runtime_narrowed() {
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
        self.bump_motion_config_revision();
        if !self.rt_executor_owned {
            self.refresh_schema_cache();
        }
    }

    pub fn reapply_driver_limits(&mut self) -> bool {
        const MAX_DOF: usize = crate::rt_core::mailbox::MAX_DOF;
        let dof = self.config.dof;
        if dof <= MAX_DOF {
            let mut vel = [None::<f32>; MAX_DOF];
            let mut pos = [None::<(f32, f32)>; MAX_DOF];
            let locked = self.driver.as_ref().is_some_and(|d| {
                d.with_ref(|drv| {
                    for i in 0..dof {
                        vel[i] = drv.max_command_velocity(i);
                        pos[i] = drv.position_limits(i);
                    }
                })
                .is_some()
            });
            if locked {
                self.apply_extracted_driver_limits_inner(&vel[..dof], &pos[..dof], false);
                self.driver_config_error.is_none()
            } else {
                false
            }
        } else {
            let limits = self.driver.as_ref().and_then(|d| {
                d.with_ref(|drv| {
                    let vel: Vec<_> = (0..dof).map(|i| drv.max_command_velocity(i)).collect();
                    let pos: Vec<_> = (0..dof).map(|i| drv.position_limits(i)).collect();
                    (vel, pos)
                })
            });
            if let Some((vel_limits, pos_limits)) = limits {
                self.apply_extracted_driver_limits_inner(&vel_limits, &pos_limits, false);
                self.driver_config_error.is_none()
            } else {
                false
            }
        }
    }

    /// Zero-allocation check whether a candidate `GripperPolicy` satisfies both profile and
    /// driver hardware position limits without mutating `self`.
    #[allow(clippy::result_large_err)]
    pub(crate) fn validate_gripper_policy_against_driver_bounded(
        &self,
        gp: &crate::model::GripperPolicy,
    ) -> Result<(), crate::safety::BoundedText<256>> {
        use crate::safety::BoundedText;
        let Some(g_idx) = self.config.gripper_index() else {
            return Err(BoundedText::new("Gripper calibration requires a gripper joint"));
        };
        let Some(lim) = self.config.limits.get(g_idx) else {
            return Err(BoundedText::new("Gripper calibration requires gripper joint limits"));
        };
        let mut lo = lim.min_position;
        let mut hi = lim.max_position;
        if self.rt_executor_owned {
            if let Some((d_min, d_max)) = self.cached_driver_pos_limits.get(g_idx).copied().flatten() {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    return Err(BoundedText::new("Driver position limits for gripper joint are invalid"));
                }
                lo = lo.max(d_min);
                hi = hi.min(d_max);
            }
        } else if let Some(d) = self.driver.as_ref() {
            let pos_lim = d
                .with_ref(|drv| drv.position_limits(g_idx))
                .ok_or_else(|| BoundedText::new("driver unavailable while validating gripper calibration"))?;
            if let Some((d_min, d_max)) = pos_lim {
                if !d_min.is_finite() || !d_max.is_finite() || d_min >= d_max {
                    return Err(BoundedText::new("Driver position limits for gripper joint are invalid"));
                }
                lo = lo.max(d_min);
                hi = hi.min(d_max);
            }
        }
        if self.driver_config_error.is_some() && self.driver.is_none() {
            return Err(self
                .driver_config_error
                .as_deref()
                .map(BoundedText::new)
                .unwrap_or_else(|| BoundedText::new("Driver limit configuration error prevents gripper calibration")));
        }
        let in_lim = |v: f32| v.is_finite() && v >= lo && v <= hi;
        if !(in_lim(gp.closed_rad)
            && in_lim(gp.open_rad)
            && gp.jaw_opening.as_ref().is_none_or(|jo| jo.points.iter().all(|&[q, _]| in_lim(q))))
        {
            return Err(BoundedText::new(
                "Driver limits violated config invariants: Invalid gripper policy (open/closed inside the gripper joint limits, 0 <= squeeze_rad <= 0.5)",
            ));
        }
        Ok(())
    }

    /// Pure read-only validation of driver position/velocity limits into bounded stack storage
    /// (zero allocations, zero mutations to `self`).
    #[allow(clippy::result_large_err)] // inline bounded text avoids RT heap allocation
    pub(crate) fn probe_driver_limits_bounded(
        &self,
    ) -> Result<CandidateDriverLimits, crate::safety::BoundedText<256>> {
        self.probe_driver_limits_with_offset_shift_bounded(None)
    }

    /// Probes driver limits for candidate `new_offsets` BEFORE mutating `ZeroOffsetDriver`
    /// (zero heap allocations).
    #[allow(clippy::result_large_err)]
    pub(crate) fn probe_driver_limits_for_offsets_bounded(
        &self,
        new_offsets: &[f32],
    ) -> Result<CandidateDriverLimits, crate::safety::BoundedText<256>> {
        self.probe_driver_limits_with_offset_shift_bounded(Some(new_offsets))
    }

    #[allow(clippy::result_large_err)]
    fn probe_driver_limits_with_offset_shift_bounded(
        &self,
        candidate_offsets: Option<&[f32]>,
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
            d.with_ref(|drv| {
                for i in 0..dof {
                    vel_limits[i] = drv.max_command_velocity(i);
                    pos_limits[i] = drv.position_limits(i).map(|(lo, hi)| {
                        if let Some(new_off) = candidate_offsets {
                            let old = self.joint_zero_offsets.get(i).copied().unwrap_or(0.0);
                            let next = new_off.get(i).copied().unwrap_or(old);
                            let shift = old - next;
                            (lo + shift, hi + shift)
                        } else {
                            (lo, hi)
                        }
                    });
                }
            })
            .is_some()
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
            raw_pos: pos_limits,
        })
    }

    /// Commits validated candidate driver limits in place and retires any previous error string
    /// off-thread (zero heap ops; invoked ONLY after `clear_check` passes).
    pub(crate) fn commit_driver_limits_bounded(&mut self, cand: &CandidateDriverLimits) {
        for i in 0..cand.dof {
            self.config.limits[i].min_position = cand.min[i];
            self.config.limits[i].max_position = cand.max[i];
            self.config.limits[i].max_velocity = cand.vel[i];
            self.cached_driver_pos_limits[i] = cand.raw_pos[i];
        }
        self.bump_motion_config_revision();
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
    pub(crate) fn joint_limit_clear_violation_for_positions(
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
    pub(crate) fn joint_limit_clear_violation_with_candidates(
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
        if self.driver.is_none() {
            return true;
        }
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.feedback_all_within(self.config.dof, max_age))
    }

    /// True when every arm joint (excluding a `PosForce` Damiao gripper) has valid numeric
    /// feedback within `max_age`. Used by gravity compensation so transient J7 pos-force
    /// staleness does not prematurely abort or wedge arm-only handback before
    /// `feedback_stale_fault_ms` latches `J7_FEEDBACK_STALE`.
    pub fn hardware_arm_feedback_fresh(&self, max_age: Duration) -> bool {
        if !self.feedback_numeric_valid { return false; }
        if !self.config.damiao_gripper.as_ref().is_some_and(|g| g.mode == crate::model::DamiaoGripperMode::PosForce) {
            return self.hardware_feedback_fresh(max_age);
        }
        let arm = self.config.arm_joints();
        if self.driver.is_none() {
            return true;
        }
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.feedback_joints_within(&arm, max_age))
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
    pub(crate) fn request_disable_bounded(
        &mut self,
        kind: crate::driver::DisableKind,
    ) -> Result<(), RequestDisableError> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        // T2: Never silently discard a pending broker/park `Disable` transition when upgrading to
        // protective disable (e.g. thermal trip). Non-disable transitions (such as an uncancelled
        // `Enable`) end `SUPERSEDED` / `UNLOAD_FAILED`; a pending `Disable` stays attached to the
        // upgraded driver disable handshake (or completes/fails here) so its request receives a
        // terminal delivered exactly once.
        let pending_disable_origin = match self.torque_transition {
            Some(TorqueTransition::Disable { origin, .. }) => Some(origin),
            _ => None,
        };
        let pending_disable = pending_disable_origin.is_some();
        let is_protective = matches!(kind, crate::driver::DisableKind::DisableProtective { .. });
        let prev_protective = matches!(
            self.active_disable_kind,
            Some(crate::driver::DisableKind::DisableProtective { .. })
        );
        if is_protective && !prev_protective {
            // Upgrading from a non-protective disable to thermal/exit protective disable clears
            // any prior non-protective retry exhaustion so the protective cut always runs.
            self.background_disable_retry = None;
        }
        if !pending_disable {
            if let Some(t) = self.torque_transition.take() {
                match t {
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::BrokerTransition,
                        ..
                    }) => {
                        self.core.begin_transition(Err("SUPERSEDED"));
                    }
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::ParkRearm { action_id, .. },
                        ..
                    }) => {
                        self.finish_park_rearm_failed(action_id, "UNLOAD_FAILED");
                    }
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::LegacyWs { token, slot, .. },
                        ..
                    }) => {
                        self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::StaticCode("SUPERSEDED"));
                    }
                    TorqueTransition::Enable(EnableReducer {
                        origin: EnableOrigin::DetachedCancel,
                        ..
                    }) => {}
                    TorqueTransition::Disable { .. } => {}
                }
            }
        }
        if self.rt_executor_owned
            && !pending_disable
            && matches!(self.background_disable_retry, Some(BackgroundDisableRetry::Exhausted))
        {
            return Err(RequestDisableError {
                code: "DISABLE_UNCONFIRMED",
                message: crate::safety::BoundedText::new("Background torque disable retries exhausted"),
                refused_by_chokepoint: false,
            });
        }
        if is_protective {
            self.mit_torque_off();
            self.active_disable_client_park = None;
        }
        self.active_disable_kind = Some(kind);
        self.disable_requested();
        #[cfg(not(target_arch = "wasm32"))]
        if let crate::driver::DisableKind::DisableRequested { started_at } = kind {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            self.torque_shadow.enqueue_at(
                started_at,
                crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        }
        if self.driver.is_some() {
            if self.rt_executor_owned {
                let dof = self.config.dof;
                let feedback_numeric_valid = self.feedback_numeric_valid;
                let mut ages_buf = std::mem::take(&mut self.scratch.ages);
                let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
                let outcome = {
                    #[cfg(not(target_arch = "wasm32"))]
                    let shadow_rt = &mut self.torque_shadow;
                    let d = self.driver.as_mut().expect("checked is_some");
                    match d.try_with_mut(|drv| {
                        let drv_now = drv.clock_now();
                        let res = drv.begin_disable_torque_budgeted(kind, 0);
                        let post_now = drv.clock_now();
                        let frames_dispatched = drv.disable_frames_dispatched();
                        let dispatched_mask = drv.disable_dispatched_mask();
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            &self.measured_positions,
                            &mut ages_buf,
                            &mut bounds_buf,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        shadow_rt.drain_from_driver(drv);
                        (res, drv_now, post_now, frames_dispatched, dispatched_mask, snap)
                    }) {
                        Ok(v) => Ok(v),
                        Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                        Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                    }
                };
                self.scratch.ages = ages_buf;
                self.scratch.fb.bounds = bounds_buf;
                let (res, drv_now, post_now, frames_dispatched, dispatched_mask, snap) = match outcome {
                    Ok(v) => {
                        self.reset_disable_lock_contention();
                        v
                    }
                    Err(true) => {
                        self.record_disable_lock_contention();
                        if let Some(origin) = pending_disable_origin {
                            self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        } else if is_protective {
                            // R12 B4: A contended first protective (thermal / exit) cut MUST arm
                            // a bounded per-release retry (`next_retry_at: now`) on the driver's
                            // lock-free clock so `service_background_disable` retries every
                            // release until dispatched.
                            let retry_now = self.driver_clock_now_or(Instant::now());
                            let _ = self.arm_background_disable_retry_immediate_with_kind(
                                kind,
                                retry_now,
                            );
                        }
                        return Err(RequestDisableError {
                            code: "DRIVER_BUSY",
                            message: crate::safety::BoundedText::new("Driver mutex contended on RT disable"),
                            refused_by_chokepoint: false,
                        });
                    }
                    Err(false) => {
                        if let Some(origin) = pending_disable_origin {
                            self.torque_transition = None;
                            match origin {
                                DisableOrigin::BrokerTransition => {
                                    self.core.begin_transition(Err("FEEDBACK_UNAVAILABLE"));
                                }
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        "FEEDBACK_UNAVAILABLE",
                                        "Feedback unavailable",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_failed(
                                        token,
                                        slot,
                                        LegacyTorqueFailure::FeedbackUnavailable,
                                    );
                                }
                                DisableOrigin::Detached { .. } => {}
                            }
                        }
                        return Err(RequestDisableError {
                            code: "FEEDBACK_UNAVAILABLE",
                            message: crate::safety::BoundedText::new("Feedback unavailable"),
                            refused_by_chokepoint: false,
                        });
                    }
                };
                let res = match res {
                    Ok(p) => p,
                    Err(e) => {
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        let mut message = crate::safety::BoundedText::empty();
                        message.push_str("Torque disable transmission failed: ");
                        e.write_bounded(&mut message);
                        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                        let partial_retry = self.commit_partial_disable_retry_kind(
                            kind,
                            post_now,
                            frames_dispatched,
                            partial_remaining,
                        );
                        let code = if let Some(origin) = pending_disable_origin {
                            self.torque_transition = None;
                            if let Some(rk) = partial_retry {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else if is_protective {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(kind, post_now);
                            } else if !refused {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            }
                            let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                            match origin {
                                DisableOrigin::BrokerTransition => {
                                    self.core.begin_transition(Err(code));
                                }
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        code,
                                        "Torque disable transmission failed",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_disable_failed(
                                        token,
                                        slot,
                                        code,
                                        Some(message),
                                    );
                                }
                                DisableOrigin::Detached { .. } => {}
                            }
                            code
                        } else {
                            if let Some(rk) = partial_retry {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else if is_protective {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(kind, post_now);
                            } else if refused {
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                                self.last_disable_attempt = None;
                                self.background_disable_retry = None;
                            } else {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            }
                            crate::driver::handshake::rt_progress(false, Err(e))
                                .err()
                                .unwrap_or("HARDWARE_FAULT")
                        };
                        return Err(RequestDisableError {
                            code,
                            message,
                            refused_by_chokepoint: refused && !frames_dispatched,
                        });
                    }
                };
                self.owner_driver_snapshot = snap;
                let done_now = matches!(res, crate::driver::TorqueProgress::Done(_));
                if done_now {
                    self.disable_accepted();
                    if let Some(origin) = pending_disable_origin {
                        self.torque_transition = None;
                        match origin {
                            DisableOrigin::BrokerTransition => {
                                let terminal = self.torque_off_finish_rt().map(|()| Rt::Done);
                                self.core.begin_transition(terminal);
                            }
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_after_accepted(watchdog, action_id);
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                match self.torque_off_finish_rt() {
                                    Ok(()) => self.settle_legacy_ws_accepted(token, slot),
                                    Err(code) => {
                                        self.settle_legacy_ws_disable_failed(token, slot, code, None)
                                    }
                                }
                            }
                            DisableOrigin::Detached { .. } => {
                                let _ = self.torque_off_finish_rt();
                            }
                        }
                    }
                } else {
                    if frames_dispatched {
                        self.disable_first_frame_accepted();
                    }
                    if let Some(origin) = pending_disable_origin {
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                    } else {
                        let _ = self.arm_background_disable_retry_with_kind(kind, drv_now);
                    }
                }
                return Ok(());
            }
            if self
                .driver
                .as_ref()
                .expect("checked is_some")
                .as_legacy_lock()
                .is_some_and(|l| l.is_poisoned())
            {
                if let Some(origin) = pending_disable_origin {
                    self.torque_transition = None;
                    match origin {
                        DisableOrigin::BrokerTransition => {
                            self.core.begin_transition(Err("FEEDBACK_UNAVAILABLE"));
                        }
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "FEEDBACK_UNAVAILABLE",
                                "Feedback unavailable",
                            );
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.settle_legacy_ws_failed(
                                token,
                                slot,
                                LegacyTorqueFailure::FeedbackUnavailable,
                            );
                        }
                        DisableOrigin::Detached { .. } => {}
                    }
                }
                return Err(RequestDisableError {
                    code: "FEEDBACK_UNAVAILABLE",
                    message: crate::safety::BoundedText::new("Feedback unavailable"),
                    refused_by_chokepoint: false,
                });
            }
            let d = self.driver.as_mut().expect("checked is_some");
            let mut drv = match d.write() {
                Some(g) => g,
                None => {
                    return Err(RequestDisableError {
                        code: "FEEDBACK_UNAVAILABLE",
                        message: crate::safety::BoundedText::new("Feedback unavailable"),
                        refused_by_chokepoint: false,
                    });
                }
            };
            // R12 B1: Never hold the driver mutex while waiting/sleeping for K=3 rest-pose
            // evidence on legacy owners. Drivers supporting deferred per-release rest-gate wait
            // (`FeetechArmDriver`, `DamiaoArmDriver`) begin non-blockingly: if awaiting K=3
            // (`Pending` with `!frames_dispatched`), release the mutex immediately so subsequent
            // legacy releases advance one bounded poll step while holds keep running between
            // releases.
            if drv.supports_budgeted_disable_await() {
                match drv.begin_disable_torque(kind) {
                    Ok(crate::driver::TorqueProgress::Done(_)) => {}
                    Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                        let frames_dispatched = drv.disable_frames_dispatched();
                        if !frames_dispatched {
                            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
                            let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
                            self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                                drv.as_ref(),
                                self.config.dof,
                                self.feedback_numeric_valid,
                                &self.measured_positions,
                                &mut ages_buf,
                                &mut bounds_buf,
                            );
                            self.scratch.ages = ages_buf;
                            self.scratch.fb.bounds = bounds_buf;
                            #[cfg(not(target_arch = "wasm32"))]
                            self.torque_shadow.drain_from_driver(&mut **drv);
                            drop(drv);
                            if let Some(origin) = pending_disable_origin {
                                self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                            }
                            return Ok(());
                        }
                        // Frame 1 already dispatched (`torque_intent = Off`): complete remaining
                        // bounded disable repeats without re-sending repeat 1 or starving holds.
                        if let Err(e) = drv.bus_recovery_mut().finish_dispatched_disable() {
                            let refused = matches!(
                                e,
                                crate::driver::ArmDriverError::TorqueOffWouldDrop
                                    | crate::driver::ArmDriverError::DisableRequestExpired
                            );
                            let post_now = drv.clock_now();
                            let frames_dispatched = drv.disable_frames_dispatched() || true;
                            let dispatched_mask = drv.disable_dispatched_mask();
                            let mut message = crate::safety::BoundedText::empty();
                            message.push_str("Torque disable transmission failed: ");
                            e.write_bounded(&mut message);
                            #[cfg(not(target_arch = "wasm32"))]
                            self.torque_shadow.drain_from_driver(&mut **drv);
                            drop(drv);
                            let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                            let partial_retry = self.commit_partial_disable_retry_kind(
                                kind,
                                post_now,
                                frames_dispatched,
                                partial_remaining,
                            );
                            let code = if let Some(origin) = pending_disable_origin {
                                self.torque_transition = None;
                                let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                                match origin {
                                    DisableOrigin::BrokerTransition => {
                                        self.core.begin_transition(Err(code));
                                    }
                                    DisableOrigin::Park { watchdog, action_id, .. } => {
                                        self.finish_park_disable_failed(
                                            watchdog,
                                            action_id,
                                            code,
                                            "Torque disable transmission failed",
                                        );
                                    }
                                    DisableOrigin::LegacyWs { token, slot, .. } => {
                                        self.settle_legacy_ws_disable_failed(
                                            token,
                                            slot,
                                            code,
                                            Some(message),
                                        );
                                    }
                                    DisableOrigin::Detached { .. } => {}
                                }
                                code
                            } else {
                                if refused && partial_retry.is_none() {
                                    self.disable_pending = false;
                                    self.active_disable_kind = None;
                                    self.last_disable_attempt = None;
                                    self.background_disable_retry = None;
                                }
                                crate::driver::handshake::rt_progress(false, Err(e))
                                    .err()
                                    .unwrap_or("HARDWARE_FAULT")
                            };
                            return Err(RequestDisableError {
                                code,
                                message,
                                refused_by_chokepoint: refused && !frames_dispatched,
                            });
                        }
                    }
                    Err(e) => {
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        let post_now = drv.clock_now();
                        let frames_dispatched = drv.disable_frames_dispatched();
                        let dispatched_mask = drv.disable_dispatched_mask();
                        let mut message = crate::safety::BoundedText::empty();
                        message.push_str("Torque disable transmission failed: ");
                        e.write_bounded(&mut message);
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.drain_from_driver(&mut **drv);
                        drop(drv);
                        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                        let partial_retry = self.commit_partial_disable_retry_kind(
                            kind,
                            post_now,
                            frames_dispatched,
                            partial_remaining,
                        );
                        let code = if let Some(origin) = pending_disable_origin {
                            self.torque_transition = None;
                            let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                            match origin {
                                DisableOrigin::BrokerTransition => {
                                    self.core.begin_transition(Err(code));
                                }
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        code,
                                        "Torque disable transmission failed",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_disable_failed(
                                        token,
                                        slot,
                                        code,
                                        Some(message),
                                    );
                                }
                                DisableOrigin::Detached { .. } => {}
                            }
                            code
                        } else {
                            if refused && partial_retry.is_none() {
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                                self.last_disable_attempt = None;
                                self.background_disable_retry = None;
                            }
                            crate::driver::handshake::rt_progress(false, Err(e))
                                .err()
                                .unwrap_or("HARDWARE_FAULT")
                        };
                        return Err(RequestDisableError {
                            code,
                            message,
                            refused_by_chokepoint: refused && !frames_dispatched,
                        });
                    }
                }
            } else if let Err(e) = drv.disable_torque(kind) {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let post_now = drv.clock_now();
                let frames_dispatched = drv.disable_frames_dispatched();
                let dispatched_mask = drv.disable_dispatched_mask();
                let mut message = crate::safety::BoundedText::empty();
                message.push_str("Torque disable transmission failed: ");
                e.write_bounded(&mut message);
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.drain_from_driver(&mut **drv);
                drop(drv);
                let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                let partial_retry = self.commit_partial_disable_retry_kind(
                    kind,
                    post_now,
                    frames_dispatched,
                    partial_remaining,
                );
                let code = if let Some(origin) = pending_disable_origin {
                    self.torque_transition = None;
                    let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                    match origin {
                        DisableOrigin::BrokerTransition => {
                            self.core.begin_transition(Err(code));
                        }
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                code,
                                "Torque disable transmission failed",
                            );
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.settle_legacy_ws_disable_failed(
                                token,
                                slot,
                                code,
                                Some(message),
                            );
                        }
                        DisableOrigin::Detached { .. } => {}
                    }
                    code
                } else {
                    if refused && partial_retry.is_none() {
                        self.disable_pending = false;
                        self.active_disable_kind = None;
                        self.last_disable_attempt = None;
                        self.background_disable_retry = None;
                    }
                    crate::driver::handshake::rt_progress(false, Err(e))
                        .err()
                        .unwrap_or("HARDWARE_FAULT")
                };
                return Err(RequestDisableError {
                    code,
                    message,
                    refused_by_chokepoint: refused && !frames_dispatched,
                });
            }
            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
            let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
            self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                drv.as_ref(),
                self.config.dof,
                self.feedback_numeric_valid,
                &self.measured_positions,
                &mut ages_buf,
                &mut bounds_buf,
            );
            self.scratch.ages = ages_buf;
            self.scratch.fb.bounds = bounds_buf;
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.drain_from_driver(&mut **drv);
            drop(drv);
        }
        self.disable_accepted();
        if let Some(origin) = pending_disable_origin {
            self.torque_transition = None;
            match origin {
                DisableOrigin::BrokerTransition => {
                    let terminal = self
                        .torque_off_finish_in_place()
                        .map(|()| Rt::Done)
                        .map_err(|e| torque_error_code(&e));
                    self.core.begin_transition(terminal);
                }
                DisableOrigin::Park { watchdog, action_id, .. } => {
                    self.finish_park_disable_after_accepted(watchdog, action_id);
                }
                DisableOrigin::LegacyWs { token, slot, .. } => {
                    match self.torque_off_finish_rt() {
                        Ok(()) => self.settle_legacy_ws_accepted(token, slot),
                        Err(code) => {
                            self.settle_legacy_ws_disable_failed(token, slot, code, None)
                        }
                    }
                }
                DisableOrigin::Detached { .. } => {
                    let _ = self.torque_off_finish_rt();
                }
            }
        }
        Ok(())
    }

    /// A successful transport write is not confirmation that torque was removed.
    pub(super) fn request_disable(&mut self, kind: crate::driver::DisableKind) -> Result<(), SafetyError> {
        self.request_disable_bounded(kind).map_err(|e| {
            if e.code == "FEEDBACK_UNAVAILABLE" {
                SafetyError::FeedbackUnavailable
            } else if e.code == "TORQUE_OFF_WOULD_DROP" {
                let r = self
                    .may_disable_torque_while_stopped()
                    .err()
                    .unwrap_or(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                self.last_stopped_torque_off_refusal = Some(r);
                r.into_safety_error(&self.config.joint_names)
            } else {
                SafetyError::ClearFaultRejected(e.message.as_str().to_string())
            }
        })
    }

    /// State when a disable request is in flight: `disable_pending` blocks new motion and
    /// enable/clear-fault transitions, while holds and MIT hold context stay active until the
    /// rest gate passes and the first disable frame is actually dispatched.
    fn disable_requested(&mut self) {
        self.disable_requested_at(Instant::now());
    }

    fn disable_requested_at(&mut self, now: Instant) {
        self.disable_pending = true;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.last_disable_attempt = Some(now);
    }

    /// State once the rest-pose gate has passed and the first disable frame has been dispatched
    /// (while bounded repeat frames are still finishing): records `torque_intent = Off` and stops
    /// holds while keeping `disable_pending = true` until the handshake completes.
    pub(crate) fn disable_first_frame_accepted(&mut self) {
        self.mit_torque_off();
        self.torque_intent = TorqueIntent::Off;
        self.measured_hold.clear();
    }

    pub(crate) fn reset_reported_torque_loss(&mut self) {
        self.reported_torque_loss_count = 0;
        self.reported_torque_loss_last_release = 0;
        self.reported_torque_loss_last_sample_seq = None;
        self.reported_torque_loss_from_motion = false;
    }

    /// State once an explicit disable (passing the rest-pose gate) or protective disable has been
    /// accepted and transmitted: records `torque_intent = Off` by decision and stops holding.
    pub(crate) fn disable_accepted(&mut self) {
        self.disable_accepted_inner(true);
    }

    pub(crate) fn disable_accepted_inner(&mut self, emit_mit_shadow: bool) {
        self.mit_torque_off_inner(emit_mit_shadow);
        self.torque_intent = TorqueIntent::Off;
        self.is_torqued = false;
        self.pending_rearm = false;
        self.disable_pending = false;
        self.active_disable_kind = None;
        self.active_disable_client_park = None;
        self.background_disable_retry = None;
        self.gripper_hold_vel_ceiling = None;
        self.enable_lifecycle = EnableLifecycle::Disarmed;
        self.measured_hold.clear();
        self.reset_reported_torque_loss();
        self.last_poll_enable_state = EnableState::Disabled;
        self.last_poll_partial_enable = false;
    }

    /// Persistent enable lifecycle owned by the transition reducer.
    pub fn enable_lifecycle(&self) -> EnableLifecycle {
        self.enable_lifecycle
    }

    /// Owner-owned torque decision state (`Off` only after an accepted disable or protective cut).
    pub fn torque_intent(&self) -> TorqueIntent {
        self.torque_intent
    }

    /// Latest per-joint hold status observed by the stopped/failed-hold tracker.
    pub fn joint_hold_status(&self) -> &[crate::driver::JointHoldStatus] {
        &self.measured_hold.joint_status[..self.measured_hold.dof]
    }

    /// Single shared clear-fault enable context for BOTH legacy and RT clear owners:
    /// reported enable state from feedback is telemetry only (never blocks recovery).
    #[inline]
    fn populate_enable_clear_context(&self, ctx: &mut crate::safety::ClearContext, enable_fb: bool) {
        ctx.enable_state_known = true;
        ctx.failed_hold_recovery_allowed = self.enable_lifecycle == EnableLifecycle::FailedHold
            && enable_fb
            && self.safety.has_latched_fault_code(FaultCode::EnableFailed)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::BusLoss)
            && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::ThermalTrip);
    }

    /// Stopped-hold eligibility derived strictly from the owner's decision state (`torque_intent`):
    /// holds are the default after any stop, fault, E-stop, failed/interrupted enable, lease loss,
    /// or watchdog event, and stop ONLY after an accepted disable records `torque_intent = Off`.
    #[inline]
    pub(crate) fn stopped_hold_eligible(&self) -> bool {
        self.torque_intent == TorqueIntent::On
            && !matches!(
                self.active_disable_kind,
                Some(crate::driver::DisableKind::DisableProtective { .. })
            )
    }

    /// Refreshes [`Self::owner_driver_snapshot`] from an already-locked driver reference without
    /// acquiring `self.driver` and without heap allocations (Rule R1).
    pub(crate) fn refresh_owner_driver_snapshot_from_ref(&mut self, drv: &dyn crate::driver::ArmDriver) {
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
            drv,
            self.config.dof,
            self.feedback_numeric_valid,
            &self.measured_positions,
            &mut ages_buf,
            &mut bounds_buf,
        );
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
    }

    /// L2: For every joint while torqued or holding while stopped (`torqued_or_stopped`) whose
    /// control mode is unverified, request a `RID_CTRL_MODE` readback every tick while already
    /// holding the driver lock in `poll_hardware_rx`. If still unverified after 200 ms, latch a
    /// non-recoverable fault naming the joint(s) and report the joint unheld
    /// (`UnheldNoVerifiedMode`). Zero heap allocations.
    pub(crate) fn enforce_unverified_mode_readback_parts(
        measured_hold: &mut MeasuredHoldTracker,
        safety: &mut crate::safety::SafetyManager,
        snap: &OwnerDriverSnapshot,
        transition_pending: bool,
        torqued_or_stopped: bool,
        config_dof: usize,
        driver: &mut dyn crate::driver::ArmDriver,
    ) {
        use crate::driver::JointHoldStatus;
        if !snap.valid || transition_pending {
            return;
        }
        let drv_now = snap.captured_at.unwrap_or_else(|| driver.clock_now());
        let dof = config_dof.min(crate::rt_core::mailbox::MAX_DOF);
        measured_hold.dof = dof;
        let mut unverified_mask = 0u64;
        let mut timed_out_mask = 0u64;
        for j in 0..dof {
            if !torqued_or_stopped || snap.hold_mode_verified[j] {
                measured_hold.unverified_mode_since[j] = None;
                measured_hold.unverified_mode_faulted_mask &= !(1u64 << j);
                continue;
            }
            unverified_mask |= 1u64 << j;
            measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
            let since = *measured_hold.unverified_mode_since[j].get_or_insert(drv_now);
            if drv_now.saturating_duration_since(since) >= UNVERIFIED_MODE_READBACK_TIMEOUT
                && (measured_hold.unverified_mode_faulted_mask & (1u64 << j)) == 0
            {
                timed_out_mask |= 1u64 << j;
            }
        }
        if unverified_mask != 0 {
            let _ = driver.joint_mode_mut().request_mode_readback_budgeted(unverified_mask);
        }
        if timed_out_mask != 0 {
            measured_hold.unverified_mode_faulted_mask |= timed_out_mask;
            let detail = crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                let mut first = true;
                for j in 0..dof {
                    if (timed_out_mask & (1u64 << j)) != 0 {
                        if !first {
                            buf.push_str(", ");
                        }
                        first = false;
                        let _ = core::fmt::write(buf, format_args!("J{}", j + 1));
                    }
                }
                buf.push_str(" control mode unverified after 200 ms readback timeout; joint unheld");
            });
            safety.latch(crate::safety::FaultKind::Other, detail);
        }
    }

    /// Single shared stopped-hold capture kernel reading strictly from the owner's inline
    /// [`Self::owner_driver_snapshot`] with ZERO driver mutex acquisition and ZERO heap
    /// allocations (Rule R1). Commands holds to EVERY joint with fresh (`<= 100 ms`) finite
    /// measured position and verified hold mode, regardless of reported motor enable telemetry.
    /// Returns `(hold_mask, invalidated_mask)`.
    pub(crate) fn capture_measured_hold_from_owner(&mut self) -> (u64, u64) {
        use crate::driver::{JointHoldStatus, STOPPED_DISABLE_FEEDBACK_MAX_AGE};
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        self.measured_hold.dof = dof;
        if !self.owner_driver_snapshot.valid {
            let mut hold_mask = 0u64;
            let mut invalidated_mask = 0u64;
            for j in 0..dof {
                if self.measured_hold.captured_positions[j].is_some() {
                    if self.owner_driver_snapshot.hold_mode_verified[j] {
                        self.measured_hold.joint_status[j] = JointHoldStatus::HeldStaleTarget;
                        hold_mask |= 1u64 << j;
                    } else {
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
                    }
                } else {
                    self.measured_hold.captured_continuity_epoch[j] = None;
                    self.measured_hold.last_seen_fresh_at[j] = None;
                    self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                    invalidated_mask |= 1u64 << j;
                }
            }
            return (hold_mask, invalidated_mask);
        }
        let snap = self.owner_driver_snapshot;
        let drv_now = snap.captured_at.unwrap_or_else(Instant::now);
        let mut hold_mask = 0u64;
        let mut invalidated_mask = 0u64;
        for j in 0..dof {
            let Some((pos, epoch)) = snap.fresh_samples[j] else {
                // B2: Never drop an already-captured hold while torque_intent == On.
                // Keep the last captured target and report `HeldStaleTarget` ("hold target stale")
                // once mode is verified; if mode is unverified, follow L2 (200 ms readback -> fault)
                // while preserving the captured hold target.
                if self.measured_hold.captured_positions[j].is_some() {
                    if snap.hold_mode_verified[j] {
                        self.measured_hold.joint_status[j] = JointHoldStatus::HeldStaleTarget;
                        hold_mask |= 1u64 << j;
                    } else {
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
                    }
                } else {
                    self.measured_hold.captured_continuity_epoch[j] = None;
                    self.measured_hold.last_seen_fresh_at[j] = None;
                    self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                    invalidated_mask |= 1u64 << j;
                }
                continue;
            };
            if !snap.hold_mode_verified[j] {
                // L2: 200 ms readback -> else fault; preserve any already-captured hold target.
                self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoVerifiedMode;
                invalidated_mask |= 1u64 << j;
                continue;
            }
            let continuity_broken = self.measured_hold.captured_positions[j].is_none()
                || self.measured_hold.captured_continuity_epoch[j] != Some(epoch)
                || self.measured_hold.last_seen_fresh_at[j].is_some_and(|prev| {
                    drv_now.saturating_duration_since(prev) > STOPPED_DISABLE_FEEDBACK_MAX_AGE
                });
            if continuity_broken {
                self.measured_hold.captured_positions[j] = Some(pos);
                self.measured_hold.captured_continuity_epoch[j] = Some(epoch);
                self.measured_hold.joint_status[j] = JointHoldStatus::Queued;
                invalidated_mask |= 1u64 << j;
            } else if !matches!(
                self.measured_hold.joint_status[j],
                JointHoldStatus::Held | JointHoldStatus::Queued
            ) {
                self.measured_hold.joint_status[j] = JointHoldStatus::Queued;
            }
            self.measured_hold.last_seen_fresh_at[j] = Some(drv_now);
            hold_mask |= 1u64 << j;
        }
        (hold_mask, invalidated_mask)
    }

    /// Evaluates each joint independently from fresh (`<= 100 ms`) finite measured position with
    /// transport provenance and continuity tracking, and verified hold mode without acquiring the
    /// driver mutex (Rule R1; zero heap allocations).
    pub fn snapshot_measured_hold_with_invalidated(&mut self) -> (u64, u64) {
        use crate::driver::JointHoldStatus;
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        self.measured_hold.dof = dof;
        if self.driver.is_none() {
            for j in 0..dof {
                if self.is_torqued || self.torque_intent == TorqueIntent::On {
                    if let Some(p) = self.measured_positions.get(j).copied().filter(|p| p.is_finite()) {
                        let _ = self.measured_hold.captured_positions[j].get_or_insert(p);
                        self.measured_hold.joint_status[j] = JointHoldStatus::Held;
                    } else {
                        self.measured_hold.captured_positions[j] = None;
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                    }
                } else {
                    self.measured_hold.captured_positions[j] = None;
                    self.measured_hold.joint_status[j] = JointHoldStatus::SkippedOff;
                }
            }
            return (0, 0);
        }
        self.capture_measured_hold_from_owner()
    }

    pub(crate) fn snapshot_measured_hold(&mut self) -> u64 {
        self.snapshot_measured_hold_with_invalidated().0
    }

    /// Test-only wrapper around [`Self::service_stopped_measured_hold`] returning `true` when the
    /// stopped-hold pass completed without exposing private `TickOutput`.
    #[doc(hidden)]
    pub fn service_stopped_measured_hold_complete(
        &mut self,
        dt: f64,
        gravity_output_enabled: bool,
    ) -> bool {
        matches!(
            self.service_stopped_measured_hold(dt, gravity_output_enabled),
            crate::supervisor::TickOutput::Complete { is_hold: true, .. }
        )
    }

    #[doc(hidden)]
    pub fn owner_hold_mask(&self) -> u64 {
        self.measured_hold.owner_hold_mask
    }

    #[doc(hidden)]
    pub fn owner_pending_hold_mask(&self) -> u64 {
        self.measured_hold.owner_pending_hold_mask
    }

    #[doc(hidden)]
    pub fn invalidate_owner_driver_snapshot_for_test(&mut self) {
        self.owner_driver_snapshot.invalidate();
    }

    /// Test-only helper (L3): holds `self.driver`'s mutex on a separate OS thread while running
    /// `f(self)` (which must not replace or drop `self.driver`), then releases and joins the
    /// holder thread before returning.
    #[doc(hidden)]
    pub fn with_contended_driver_for_test<R>(&mut self, f: impl FnOnce(&mut Self) -> R) -> R {
        let Some(driver_lock) = self
            .driver
            .as_ref()
            .and_then(crate::supervisor_lock::DriverSlot::as_legacy_lock)
        else {
            return f(self);
        };
        let addr =
            (driver_lock.raw_mutex() as *const std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>)
                as usize;
        let (locked_tx, locked_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let holder = std::thread::spawn(move || {
            // SAFETY: `self.driver` stays in place for the duration of `f(self)` and this thread
            // is strictly joined before `with_contended_driver_for_test` returns (even on panic).
            let mutex = unsafe { &*(addr as *const std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>) };
            let _guard = mutex.lock().unwrap();
            let _ = locked_tx.send(());
            let _ = release_rx.recv_timeout(Duration::from_secs(5));
        });
        locked_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("holder acquired driver mutex");
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(self)));
        let _ = release_tx.send(());
        let _ = holder.join();
        match res {
            Ok(v) => v,
            Err(payload) => std::panic::resume_unwind(payload),
        }
    }

    /// Services one masked measured-position hold pass while stopped or in `FailedHold`
    /// (zero heap allocation on RT; hold capture and MIT preparation read owner-held data without
    /// locking the driver mutex; never falls back to cached `commanded_positions`).
    pub(crate) fn service_stopped_measured_hold(
        &mut self,
        dt: f64,
        gravity_output_enabled: bool,
    ) -> crate::supervisor::TickOutput {
        use crate::driver::{JointHoldStatus, TxBatchOutcome};
        use crate::supervisor::TickOutput;
        if self.urgent_stop_unapplied() {
            return TickOutput::None;
        }
        if self.driver.is_none() {
            return TickOutput::None;
        }
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let (mut hold_mask, invalidated_mask) = self.snapshot_measured_hold_with_invalidated();

        let mut targets = std::mem::take(&mut self.scratch.targets);
        targets.clear();
        for i in 0..self.config.dof {
            let pos = if i < dof {
                self.measured_hold.captured_positions[i].unwrap_or(0.0)
            } else {
                0.0
            };
            targets.push(crate::driver::JointTarget {
                position: pos,
                velocity: Some(0.0),
                torque: if gravity_output_enabled {
                    self.gravity_torques_ramped.get(i).copied()
                } else {
                    Some(0.0)
                },
                kp: None,
                kd: None,
            });
        }

        // Per-joint masked MIT hold preparation (Rule R1: zero alloc, no feedback-age Vec,
        // no `last_ff.clone()`, and no driver mutex lock).
        let mut ff_buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let mit_prepared = if self.mit.owner == crate::mit::MitOwner::Active && hold_mask != 0 {
            let before_mask = hold_mask;
            let (filtered_mask, any_mit) =
                self.mit_prepare_masked_into(hold_mask, &mut targets, dt, &mut ff_buf);
            let rejected = before_mask & !filtered_mask;
            if rejected != 0 {
                for j in 0..dof {
                    if (rejected & (1u64 << j)) != 0 {
                        self.measured_hold.joint_status[j] = if self.measured_hold.captured_positions[j].is_some() {
                            JointHoldStatus::HeldStaleTarget
                        } else {
                            JointHoldStatus::UnheldNoFreshFeedback
                        };
                    }
                }
            }
            hold_mask = filtered_mask;
            any_mit
        } else {
            false
        };

        // L3: The stopped-hold queue and mask are owner state. Record them inline before TX.
        self.measured_hold.owner_hold_mask = hold_mask;
        for (slot, target) in self.measured_hold.owner_hold_targets[..dof]
            .iter_mut()
            .zip(targets.iter())
        {
            *slot = target.position;
        }

        // L3: An empty mask with no owner-tracked queued holds takes NO driver lock at all.
        let keep_queued_mask = hold_mask & !invalidated_mask;
        let needs_prune = (self.measured_hold.owner_pending_hold_mask & !keep_queued_mask) != 0;
        if hold_mask == 0 && !needs_prune {
            self.measured_hold.consecutive_lock_contended = 0;
            self.measured_hold.lock_contended_escalated = false;
            self.scratch.targets = targets;
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue_at(
                self.cached_driver_clock_now(),
                crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                    hold_mask: crate::torque::ChannelMask::EMPTY,
                    stale_held_mask: crate::torque::ChannelMask::EMPTY,
                    contended: false,
                    escalated: false,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
            return TickOutput::None;
        }

        // L3: Non-blocking `try_lock` for stopped-hold TX: on contention, skip this tick's send
        // and increment the diagnostics counter; escalate after `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS`
        // (25 consecutive skips, 0.5 s at 50 Hz) with a counted warning and a non-recoverable fault
        // (zero heap allocation on RT).
        let mit_lost = self.mit.owner == crate::mit::MitOwner::Lost;
        let mut lock_contended = false;
        #[cfg(not(target_arch = "wasm32"))]
        let shadow_rt = &mut self.torque_shadow;
        let write_result = match self.driver.as_mut() {
            Some(d) => match d.try_with_mut(|drv| {
                drv.mit_mut().set_mit_ownership_lost(mit_lost);
                let dispatched_mask = drv.disable_dispatched_mask();
                let effective_hold_mask = hold_mask & !dispatched_mask;
                let effective_keep_mask = keep_queued_mask & !dispatched_mask;
                drv.tx_mut().drop_pending_holds_outside_mask(effective_keep_mask);
                drv.pos_force_mut().set_pending_pos_force_hold_hint(true);
                let r = drv.write_measured_hold_masked(effective_hold_mask, &targets);
                let now = drv.clock_now();
                let max_age = drv.tx().tx_frame_max_age();
                let oldest_pending = drv.tx().oldest_pending_tx_age();
                let last_expired = drv.tx().last_tx_expired_age();
                let sent_any_fallback = drv.tx().last_hold_tx_sent_any();
                let due_mask = drv.tx().last_hold_tx_due_mask();
                let sent_mask = drv.tx().last_hold_tx_sent_mask();
                let pf_status = drv.pos_force_mut().take_last_pos_force_tx_status();
                let has_pending_holds = drv.tx().pending_tx_len_total() > 0;
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(drv);
                (
                    r,
                    now,
                    max_age,
                    oldest_pending,
                    last_expired,
                    sent_any_fallback,
                    due_mask,
                    sent_mask,
                    pf_status,
                    has_pending_holds,
                    effective_hold_mask,
                )
            }) {
                Ok(v) => Some(v),
                Err(_) => {
                    lock_contended = true;
                    None
                }
            },
            None => None,
        };
        self.scratch.targets = targets;

        if lock_contended {
            crate::diagnostics::tick::record_stopped_hold_lock_contended();
            self.measured_hold.consecutive_lock_contended =
                self.measured_hold.consecutive_lock_contended.saturating_add(1);
            if self.measured_hold.consecutive_lock_contended >= STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS
                && !self.measured_hold.lock_contended_escalated
            {
                self.measured_hold.lock_contended_escalated = true;
                crate::diagnostics::tick::record_stopped_hold_contention_escalation();
                let skips = self.measured_hold.consecutive_lock_contended;
                let detail = crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                    let _ = core::fmt::write(
                        buf,
                        format_args!(
                            "Stopped-hold TX skipped {} consecutive ticks due to driver mutex contention",
                            skips
                        ),
                    );
                });
                self.safety.latch(crate::safety::FaultKind::Other, detail);
            }
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                    hold_mask: crate::torque::ChannelMask::EMPTY,
                    stale_held_mask: crate::torque::ChannelMask::EMPTY,
                    contended: true,
                    escalated: self.measured_hold.lock_contended_escalated,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
            return TickOutput::None;
        }
        self.measured_hold.consecutive_lock_contended = 0;
        self.measured_hold.lock_contended_escalated = false;

        let Some((
            write_res,
            now,
            max_age,
            oldest_pending,
            last_expired,
            sent_any_fallback,
            due_mask,
            sent_mask,
            pf_status,
            has_pending_holds,
            effective_hold_mask,
        )) = write_result
        else {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue_at(
                self.cached_driver_clock_now(),
                crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                    hold_mask: crate::torque::ChannelMask::EMPTY,
                    stale_held_mask: crate::torque::ChannelMask::EMPTY,
                    contended: false,
                    escalated: false,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
            return TickOutput::None;
        };

        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_hm = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut shadow_stale_hm = crate::torque::ChannelMask::EMPTY;

        let out = match write_res {
            Ok(report) => {
                let n = report.dof.min(dof);
                let mut next_pending_mask = 0u64;
                for j in 0..n {
                    self.measured_hold.joint_status[j] = report.joint_status[j];
                    #[cfg(not(target_arch = "wasm32"))]
                    if j < crate::torque::MAX_CHANNELS {
                        match report.joint_status[j] {
                            JointHoldStatus::Held | JointHoldStatus::Queued => {
                                shadow_hm.insert(j as u8);
                            }
                            JointHoldStatus::HeldStaleTarget => {
                                shadow_hm.insert(j as u8);
                                shadow_stale_hm.insert(j as u8);
                            }
                            _ => {}
                        }
                    }
                    if report.joint_status[j] == JointHoldStatus::Queued
                        || (report.joint_status[j] == JointHoldStatus::HeldStaleTarget
                            && has_pending_holds)
                    {
                        next_pending_mask |= 1u64 << j;
                    }
                    // B2: Preserve captured hold targets while torque_intent == On; only clear
                    // when explicitly SkippedOff (UnheldNoVerifiedMode preserves captured_positions).
                    if report.joint_status[j] == JointHoldStatus::SkippedOff {
                        self.measured_hold.captured_positions[j] = None;
                        self.measured_hold.captured_continuity_epoch[j] = None;
                        self.measured_hold.last_seen_fresh_at[j] = None;
                    }
                }
                #[cfg(not(target_arch = "wasm32"))]
                if shadow_hm.is_empty() && report.sent_any {
                    for j in 0..n.min(crate::torque::MAX_CHANNELS) {
                        if (effective_hold_mask & (1u64 << j)) != 0 {
                            shadow_hm.insert(j as u8);
                        }
                    }
                }
                if has_pending_holds && next_pending_mask == 0 {
                    next_pending_mask = hold_mask;
                }
                self.measured_hold.owner_pending_hold_mask = next_pending_mask;
                if hold_mask == 0 && !report.sent_any && matches!(report.outcome, TxBatchOutcome::Complete) {
                    TickOutput::None
                } else {
                    match report.outcome {
                        TxBatchOutcome::Complete => TickOutput::Complete {
                            is_hold: true,
                            now,
                            max_age,
                            due_mask,
                            sent_mask,
                            pf_status,
                        },
                        TxBatchOutcome::Deferred { pending } => TickOutput::Deferred {
                            pending,
                            now,
                            max_age,
                            oldest_age: oldest_pending,
                            due_mask,
                            is_hold: true,
                            sent_mask,
                            pf_status,
                        },
                        TxBatchOutcome::Expired { dropped, pending } => TickOutput::Expired {
                            dropped,
                            pending,
                            now,
                            max_age,
                            oldest_age: last_expired,
                            due_mask,
                            sent_mask,
                            is_hold: true,
                            pf_status,
                        },
                    }
                }
            }
            Err(crate::driver::ArmDriverError::EstopInhibited) => {
                TickOutput::EstopInhibited { is_hold: true }
            }
            Err(e) => {
                let mut buf = crate::mit::OutputErrorText::empty();
                e.write_bounded(&mut buf);
                TickOutput::Error {
                    message: buf,
                    now,
                    max_age,
                    is_hold: true,
                    sent_any: sent_any_fallback,
                    due_mask,
                    sent_mask,
                    pf_status,
                }
            }
        };

        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow.enqueue_at(
            now,
            crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                hold_mask: shadow_hm,
                stale_held_mask: shadow_stale_hm,
                contended: false,
                escalated: false,
                live_intent: self.shadow_live_intent(),
                live_is_torqued: self.is_torqued,
                live_stopped: self.safety.state().is_stopped(),
            },
        );

        if mit_prepared && matches!(out, TickOutput::Complete { .. }) {
            let ff_n = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
            if self.mit.last_ff.len() == ff_n {
                self.mit.last_ff.copy_from_slice(&ff_buf[..ff_n]);
            } else {
                self.mit.last_ff.clear();
                self.mit.last_ff.extend_from_slice(&ff_buf[..ff_n]);
            }
            self.mit.last_output = Some(now);
        }
        out
    }

    #[inline]
    fn all_joints_mask(&self) -> u64 {
        let dof = self.config.dof.min(crate::rt_core::mailbox::MAX_DOF);
        if dof >= 64 {
            u64::MAX
        } else {
            (1u64 << dof) - 1
        }
    }

    #[inline]
    fn is_partial_disable_remaining(&self, dispatched_mask: u64) -> bool {
        let all = self.all_joints_mask();
        dispatched_mask != 0 && (dispatched_mask & all) != all
    }

    /// R12 B3: Once ANY motor has received a disable frame (`frames_dispatched == true`), the
    /// disable is committed (`torque_intent = Off`, holds stopped). When undispatched motors
    /// remain (`partial_remaining`), preserve `disable_pending = true` and return the retry
    /// `DisableKind` so the retry path finishes the remaining motors (keeping original `started_at`
    /// while within 1 s, or starting a bounded continuation pass if expiry fired mid-batch after
    /// at least one motor was already disabled).
    fn commit_partial_disable_retry_kind(
        &mut self,
        kind: crate::driver::DisableKind,
        post_now: Instant,
        frames_dispatched: bool,
        partial_remaining: bool,
    ) -> Option<crate::driver::DisableKind> {
        if !frames_dispatched {
            return None;
        }
        self.disable_first_frame_accepted();
        if !partial_remaining {
            return None;
        }
        let retry_kind = match kind {
            crate::driver::DisableKind::DisableRequested { started_at }
                if post_now.saturating_duration_since(started_at)
                    >= crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME =>
            {
                crate::driver::DisableKind::DisableRequested { started_at: post_now }
            }
            other => other,
        };
        self.disable_pending = true;
        self.active_disable_kind = Some(retry_kind);
        Some(retry_kind)
    }

    #[inline]
    pub(crate) fn client_park_stop_invalidated(&self, expected_stop_gen: Option<u64>) -> bool {
        self.e_stop_latched.load(Ordering::SeqCst)
            || self.estop_pending()
            || self.urgent_stop_unapplied()
            || expected_stop_gen.is_some_and(|g| self.current_stop_gen() != g)
    }

    #[inline]
    pub(crate) fn clear_active_disable_retry_state(&mut self) {
        self.disable_pending = false;
        self.active_disable_kind = None;
        self.active_disable_client_park = None;
        self.last_disable_attempt = None;
        self.background_disable_retry = None;
    }

    /// Arms bounded [`BackgroundDisableRetry`] retrying immediately on the next release
    /// (`next_retry_at: drv_now`), used for committed partial disables (R12 B3) and contended
    /// protective thermal cuts (R12 B4).
    fn arm_background_disable_retry_immediate_with_kind(
        &mut self,
        kind: crate::driver::DisableKind,
        drv_now: Instant,
    ) -> bool {
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            return false;
        }
        let client_park = if matches!(kind, crate::driver::DisableKind::DisableProtective { .. }) {
            None
        } else {
            self.active_disable_client_park
        };
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                return false;
            }
        }
        let origin = match kind {
            crate::driver::DisableKind::DisableRequested { started_at, .. } => started_at,
            crate::driver::DisableKind::DisableProtective { .. } => drv_now,
        };
        let window = self.background_disable_window_for(kind);
        if drv_now.saturating_duration_since(origin) >= window {
            self.exhaust_background_disable_retry();
            return false;
        }
        if self.background_disable_retry.is_none() {
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: 0,
                kind,
                started_at: origin,
                next_retry_at: drv_now,
                client_park,
            });
        }
        true
    }

    /// Arms bounded [`BackgroundDisableRetry`] preserving the original [`crate::driver::DisableKind`]
    /// (`started_at` on the driver clock for `DisableRequested`) so every retry passes through the
    /// driver's single `begin_disable_torque_budgeted(kind, 0)` choke point and rest-pose gate
    /// within the original request's lifetime.
    fn arm_background_disable_retry_with_kind(
        &mut self,
        kind: crate::driver::DisableKind,
        drv_now: Instant,
    ) -> bool {
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            return false;
        }
        let client_park = if matches!(kind, crate::driver::DisableKind::DisableProtective { .. }) {
            None
        } else {
            self.active_disable_client_park
        };
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                return false;
            }
        }
        let origin = match kind {
            crate::driver::DisableKind::DisableRequested { started_at, .. } => started_at,
            crate::driver::DisableKind::DisableProtective { .. } => drv_now,
        };
        let window = self.background_disable_window_for(kind);
        if drv_now.saturating_duration_since(origin) >= window {
            self.exhaust_background_disable_retry();
            return false;
        }
        if self.background_disable_retry.is_none() {
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: 0,
                kind,
                started_at: origin,
                next_retry_at: drv_now + BACKGROUND_DISABLE_RETRY_INTERVAL,
                client_park,
            });
        }
        true
    }

    #[inline]
    fn background_disable_window_for(&self, kind: crate::driver::DisableKind) -> Duration {
        match kind {
            crate::driver::DisableKind::DisableRequested { .. } => {
                crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME
            }
            crate::driver::DisableKind::DisableProtective { .. } => BACKGROUND_DISABLE_MAX_WINDOW,
        }
    }

    fn exhaust_background_disable_retry(&mut self) {
        self.background_disable_retry = Some(BackgroundDisableRetry::Exhausted);
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::Static(crate::safety::StaticFault::DisableUnconfirmed),
        );
    }

    /// R12 B1: Advances an in-flight deferred legacy disable (`FeetechArmDriver::pending_disable`
    /// or `DamiaoArmDriver::AwaitRestGate` / `Disable`) by ONE bounded poll step per control
    /// release, releasing the driver mutex immediately afterward so `service_stopped_measured_hold`
    /// keeps servicing holds between releases.
    ///
    /// Returns `true` when the driver had a pending disable transition that was serviced this
    /// release (suppressing the 250 ms fallback restart on the same tick).
    pub(crate) fn service_legacy_pending_disable(&mut self) -> bool {
        if self.rt_executor_owned || !self.disable_pending || self.torque_transition.is_some() {
            return false;
        }
        if self.driver.is_none() {
            return false;
        }
        let control_release_seq = self.control_release_seq;
        let retry_kind = self.active_disable_kind;
        let client_park = self.active_disable_client_park;
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                self.clear_active_disable_retry_state();
                return true;
            }
        }
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let period = Duration::from_secs_f64(
            1.0 / f64::from(self.config.control_frequency_hz.max(1)),
        );
        let per_tick_budget = period.saturating_sub(self.tx_release_publish_margin());
        let step = {
            let d = self.driver.as_mut().expect("checked is_some");
            d.with_mut(|drv| {
                let pending_hs = drv.torque_transition_pending();
                let partial_dispatched = drv.disable_dispatched_mask() != 0;
                if !pending_hs && !partial_dispatched {
                    return None;
                }
                if control_release_seq > 0 {
                    drv.begin_control_release(control_release_seq);
                }
                let drv_now = drv.clock_now();
                let poll_res = if pending_hs {
                    drv.poll_torque(drv_now + per_tick_budget)
                } else {
                    let k = retry_kind.unwrap_or(crate::driver::DisableKind::DisableRequested {
                        started_at: drv_now,
                    });
                    if let Some(expected_stop_gen) = client_park {
                        drv.begin_client_park_disable_with_kind(k, None, expected_stop_gen)
                    } else {
                        drv.begin_disable_torque(k)
                    }
                };
                let post_now = drv.clock_now();
                let frames_dispatched = drv.disable_frames_dispatched();
                let dispatched_mask = drv.disable_dispatched_mask();
                let snap = OwnerDriverSnapshot::capture_from_driver(
                    &*drv,
                    dof,
                    feedback_numeric_valid,
                    &self.measured_positions,
                    &mut ages_buf,
                    &mut bounds_buf,
                );
                Some((poll_res, post_now, frames_dispatched, dispatched_mask, snap))
            })
            .flatten()
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        let Some(step) = step else {
            return false;
        };
        let (poll_res, post_now, frames_dispatched, dispatched_mask, snap) = step;
        self.owner_driver_snapshot = snap;
        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
        let active_kind = self.active_disable_kind.unwrap_or(
            crate::driver::DisableKind::DisableRequested { started_at: post_now },
        );
        match poll_res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.disable_accepted();
                let _ = self.torque_off_finish_in_place();
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
            }
            Err(
                crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired,
            ) => {
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                    if !partial_remaining {
                        self.disable_pending = false;
                        self.active_disable_kind = None;
                        self.last_disable_attempt = None;
                    } else if let crate::driver::DisableKind::DisableRequested { started_at } = active_kind {
                        if post_now.saturating_duration_since(started_at)
                            >= crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME
                        {
                            if self.background_disable_retry.is_none() {
                                let rk = crate::driver::DisableKind::DisableRequested {
                                    started_at: post_now,
                                };
                                self.active_disable_kind = Some(rk);
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else {
                                self.exhaust_background_disable_retry();
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                            }
                        }
                    }
                } else {
                    self.disable_pending = false;
                    self.active_disable_kind = None;
                    self.last_disable_attempt = None;
                    self.background_disable_retry = None;
                }
            }
            Err(e) => {
                if let Some(expected_stop_gen) = client_park {
                    if crate::driver::is_estop_inhibited(&e)
                        || self.client_park_stop_invalidated(expected_stop_gen)
                    {
                        self.clear_active_disable_retry_state();
                        return true;
                    }
                }
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
                if !self.safety.deferred_logging() {
                    tracing::warn!("Legacy deferred torque disable step failed: {e}");
                }
            }
        }
        true
    }

    pub fn enable_torque(admitted: Admitted<'_>) -> Result<SafetyState, SafetyError> {
        let (this, proof) = admitted.consume()?;
        this.enable_torque_inner(&proof)
    }

    pub(crate) fn enable_torque_inner(&mut self, _proof: &crate::supervisor::AdmittedProof) -> Result<SafetyState, SafetyError> {
        if self.torque_transition.is_some() {
            return Err(SafetyError::ClearFaultRejected("A torque transition is pending".into()));
        }
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
        let (was_torqued, prev_gripper_cmd) = self.enable_prepare()?;
        let mit_qualified = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        let mut outcome = self.start_enable_reducer(
            EnableOrigin::BrokerTransition,
            was_torqued,
            prev_gripper_cmd,
            None,
            false,
            mit_qualified,
        );
        let mut first = true;
        let res = loop {
            match outcome {
                EnableStepOutcome::Armed(state) => break Ok(state),
                EnableStepOutcome::Failed {
                    message,
                    feedback_unavailable,
                    estop_interrupted,
                    ..
                } => {
                    break Err(if feedback_unavailable {
                        SafetyError::FeedbackUnavailable
                    } else if estop_interrupted {
                        SafetyError::EStopActive
                    } else {
                        SafetyError::ClearFaultRejected(message.as_str().to_string())
                    });
                }
                EnableStepOutcome::Pending { .. } => {
                    if !first {
                        if let Some(ref d) = self.driver {
                            let _ = d.with_ref(|drv| drv.clock_pause(Duration::from_micros(500)));
                        }
                    }
                    first = false;
                    let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
                        break Err(SafetyError::ClearFaultRejected(
                            "Enable transition reducer state lost".into(),
                        ));
                    };
                    let (next, _) = self.poll_enable_reducer(reducer, None);
                    #[cfg(not(target_arch = "wasm32"))]
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                    outcome = next;
                }
            }
        };
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
        res
    }

    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check (still
    /// subject to resting-pose and torque-off safety checks).
    pub fn disable_torque(&mut self) -> Result<SafetyState, SafetyError> {
        let res = (|| {
            if self.torque_transition.is_some() {
                return Err(SafetyError::ClearFaultRejected("A torque transition is pending".into()));
            }
            // Torque changes and re-arming are refused in gravity compensation: exit first.
            self.gravity_comp_servo_busy()?;
            self.torque_off(false)
        })();
        #[cfg(not(target_arch = "wasm32"))]
        if !self.rt_executor_owned {
            self.flush_torque_shadow();
        }
        res
    }

    /// Enable, before the driver's handshake: every refusal (gravity compensation, driver
    /// limits, handover, pending disable, E-stop, fault, MIT gate), then the feedback read
    /// that seeds the commanded state. Returns `(was_torqued, prev_gripper_cmd)`.
    fn enable_prepare(&mut self) -> Result<(bool, Option<f32>), SafetyError> {
        self.enable_prepare_inner(true, true, false)
    }

    fn enable_prepare_inner(
        &mut self,
        allow_bus_read: bool,
        allow_sync_mit: bool,
        nonblocking_lock: bool,
    ) -> Result<(bool, Option<f32>), SafetyError> {
        if self.internal_fault_lane.is_fenced() || self.shutdown_committed {
            return Err(SafetyError::EStopActive);
        }
        if self.core.motion_inhibited_by_config() {
            return Err(SafetyError::ClearFaultRejected(
                "CONFIG_BUSY: configuration or hardware commissioning reconciliation is active; rearm is blocked".into(),
            ));
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
        if let Some(d) = self.driver.as_ref() {
            if let Some(driver) = d.read() {
                if let Err((max_frames, required)) = driver.tx().check_watchdog_tx_budget_for_enable() {
                    return Err(SafetyError::ClearFaultRejected(format!(
                        "WATCHDOG_BUDGET_INSUFFICIENT: tx_budget.max_frames ({max_frames}) < required ({required}) for energized watchdog continuity"
                    )));
                }
            }
        }

        // RT executor step 1 does not support synchronous MIT register qualification/handover or
        // gravity-compensation mode switches; fail closed before touching any sync driver helpers.
        if (!allow_sync_mit || self.rt_executor_owned)
            && (self.gravity_comp.is_some()
                || self.mit_active()
                || self.mit.revoke_pending
                || crate::mit::mit_joint_mask(&self.config) != 0)
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
            #[cfg(not(target_arch = "wasm32"))]
            if self.driver.is_some() && self.driver_supports_nonblocking_mode_switch() {
                self.mit.armed_ok = false;
                self.active_trajectory = None;
                self.servo_target = None;
                self.mit.revoke_pending = true;
                return Err(SafetyError::ClearFaultRejected(
                    "MIT_HAND_BACK_REQUIRED: MIT tracking must hand back via PosVelHandback before rearm".into(),
                ));
            }
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

        // Enable hardware torque so bus devices wake up and publish state, then verify physical feedback.
        if let Some(mut d) = self.driver.take() {
            let prep_res: Result<(), SafetyError> = (|| {
                // Secondary double-check latch right before hardware actuation
                if self.estop_pending() {
                    return Err(SafetyError::EStopActive);
                }
                let run_prep = |driver: &mut dyn crate::driver::ArmDriver| -> Result<(), SafetyError> {
                    if self.estop_pending() {
                        return Err(SafetyError::EStopActive);
                    }
                    if allow_bus_read && !driver.is_feedback_valid() {
                        if let Ok(states) = driver.read_joint_states() {
                            let loads = driver.joint_loads();
                            if !states.is_empty()
                                && driver.is_feedback_valid()
                                && SupervisorInner::states_valid(
                                    &states,
                                    driver.capabilities(),
                                    self.config.dof,
                                    loads.as_deref(),
                                )
                            {
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
                    self.refresh_owner_driver_snapshot_from_ref(&*driver);
                    Ok(())
                };
                if nonblocking_lock {
                    match d.try_with_mut(run_prep) {
                        Ok(res) => res,
                        Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                            Err(SafetyError::ClearFaultRejected("DRIVER_BUSY".into()))
                        }
                        Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                            self.owner_driver_snapshot.invalidate();
                            Err(SafetyError::FeedbackUnavailable)
                        }
                    }
                } else {
                    match d.with_mut(run_prep) {
                        Some(res) => res,
                        None => {
                            self.owner_driver_snapshot.invalidate();
                            Err(SafetyError::FeedbackUnavailable)
                        }
                    }
                }
            })();
            self.driver = Some(d);
            prep_res?;
        }
        let was_torqued = self.is_torqued;
        let prev_gripper_cmd = if was_torqued {
            self.config.gripper_index().and_then(|i| self.commanded_positions.get(i).copied())
        } else {
            None
        };
        Ok((was_torqued, prev_gripper_cmd))
    }

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

    /// Enforces thermal protection inside the enable reducer (`Enabling` and post-enable
    /// `Validate` before arming): any finite joint temperature above `max_temperature` aborts
    /// the enable through `enter_failed_hold`, latches `FaultKind::ThermalTrip`, and starts
    /// `DisableProtective { Thermal }` without ever arming.
    fn check_enable_thermal_trip(
        &mut self,
        was_torqued: bool,
        temperature_available: bool,
    ) -> Option<EnableStepOutcome> {
        if !temperature_available
            || !self
                .temperatures
                .iter()
                .any(|t| t.is_finite() && *t > self.config.max_temperature)
        {
            return None;
        }
        // Run `enter_failed_hold` BEFORE `StopPolicy::ThermalTrip` because `enter_failed_hold`
        // clears `disable_pending` / `active_disable_kind`; `ThermalTrip` then latches
        // highest-severity `FaultKind::ThermalTrip` and starts `DisableProtective { Thermal }`.
        let message = self.enter_failed_hold(was_torqued, |buf| {
            buf.push_str("thermal trip during enable");
        });
        self.apply_stop_policy(StopPolicy::ThermalTrip);
        Some(EnableStepOutcome::Failed {
            rt_code: "THERMAL_TRIP",
            message,
            feedback_unavailable: false,
            estop_interrupted: false,
        })
    }

    /// Reducer-owned cancellation of an in-flight enable when `StopPolicy::ThermalTrip` takes
    /// over: cancels queued enable work, records the `ENABLE_FAILED` diagnostic with per-joint
    /// status (while `ThermalTrip` retains highest display precedence), and ends any pending RT
    /// `TorqueOn` command with `THERMAL_TRIP` before thermal protection starts `DisableProtective`.
    pub(crate) fn interrupt_enable_for_thermal(&mut self) {
        if !matches!(self.torque_transition, Some(TorqueTransition::Enable(_))) {
            return;
        }
        let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
            return;
        };
        match reducer.origin {
            EnableOrigin::BrokerTransition => {
                self.core.begin_transition(Err("THERMAL_TRIP"));
            }
            EnableOrigin::ParkRearm { action_id, .. } => {
                self.finish_park_rearm_failed(action_id, "THERMAL_TRIP");
            }
            EnableOrigin::LegacyWs { token, slot, .. } => {
                let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
                    buf.push_str("interrupted by thermal trip");
                });
                self.settle_legacy_ws_failed(
                    token,
                    slot,
                    LegacyTorqueFailure::StaticCode("THERMAL_TRIP"),
                );
                return;
            }
            EnableOrigin::DetachedCancel => {}
        }
        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
            buf.push_str("interrupted by thermal trip");
        });
    }

    /// Zero-allocation RT preflight for torque enable (all refusal paths return static codes).
    fn enable_prepare_rt_precheck(&mut self, authority: Option<EnableAuthority>) -> Result<(), &'static str> {
        #[cfg(not(target_arch = "wasm32"))]
        if (self.calibration_busy && self.motor_zero_busy)
            || self.commissioning.is_active()
            || self.sessions.blocks_ordinary_output(None)
        {
            return Err("MOTION_BUSY");
        }
        #[cfg(target_arch = "wasm32")]
        if (self.calibration_busy && self.motor_zero_busy) || self.commissioning.is_active() {
            return Err("MOTION_BUSY");
        }
        if self.core.motion_inhibited_by_config() {
            return Err("CONFIG_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if self.internal_fault_lane.is_fenced() {
            return Err("INTERNAL_FAULT");
        }
        if self.shutdown_committed {
            return Err("ESTOP_ACTIVE");
        }
        if self.driver_config_error.is_some() {
            return Err("MOTION_FORBIDDEN_IN_FAULT_STATE");
        }
        if self.handover_settling_since.is_some() {
            return Err("HANDOVER_SETTLING");
        }
        if self.disable_pending {
            return Err("DISABLE_PENDING");
        }
        if self.estop_pending() {
            return Err("ESTOP_ACTIVE");
        }
        if self.safety.state().is_stopped() {
            return Err(if matches!(self.safety.state(), SafetyState::Fault(_)) {
                "FAULT_ACTIVE"
            } else {
                "ESTOP_ACTIVE"
            });
        }
        if self.mit_active() || self.mit.revoke_pending {
            return Err("MOTION_BUSY");
        }
        if crate::mit::mit_joint_mask(&self.config) != 0 {
            #[cfg(not(target_arch = "wasm32"))]
            {
                if crate::rt_core::sched::mit_refusal().is_some() {
                    return Err("MIT_UNQUALIFIED");
                }
                let Some(auth) = authority else {
                    return Err("MIT_UNQUALIFIED");
                };
                let Some(proof) = self.mit_gate_proof.take() else {
                    return Err("MIT_UNQUALIFIED");
                };
                let now = Instant::now();
                let matches_admission = proof.admission_request_id == auth.admission_request_id
                    && proof.connection_id == auth.owner_conn_id
                    && proof.lease_generation == auth.lease_generation
                    && proof.estop_epoch == auth.admitted_epoch
                    && proof.estop_epoch == self.safety.epoch_id()
                    && proof.stop_gen == auth.stop_gen
                    && proof.stop_gen == self.current_stop_gen()
                    && proof.calibration_revision == self.calibration_revision
                    && proof.motion_config_revision == self.motion_config_revision
                    && proof.zero_offsets_hash
                        == crate::mit::zero_offsets_hash(&self.joint_zero_offsets)
                    && now.saturating_duration_since(proof.verified_at) <= Duration::from_secs(5);
                if !matches_admission {
                    return Err("MIT_UNQUALIFIED");
                }
            }
            #[cfg(target_arch = "wasm32")]
            {
                return Err("UNSUPPORTED");
            }
        }
        Ok(())
    }

    /// Zero-allocation RT enable prepare (runs after [`Self::enable_prepare_rt_precheck`]).
    /// Never infers `was_torqued` from feedback-reported enable state (R10 decision-owned torque).
    /// Returns `(was_torqued, prev_gripper_cmd, mit_qualified)`.
    fn enable_prepare_rt(
        &mut self,
        authority: Option<EnableAuthority>,
    ) -> Result<(bool, Option<f32>, bool), &'static str> {
        self.enable_prepare_rt_precheck(authority)?;
        let mit_qualified = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        let was_torqued = self.is_torqued;
        let prev_gripper_cmd = if was_torqued {
            self.config
                .gripper_index()
                .and_then(|i| self.commanded_positions.get(i).copied())
        } else {
            None
        };
        if let Some(d) = self.driver.take() {
            if self.estop_pending() {
                self.driver = Some(d);
                return Err("ESTOP_ACTIVE");
            }
            let Some(driver) = d.read() else {
                self.owner_driver_snapshot.invalidate();
                self.driver = Some(d);
                return Err("FEEDBACK_UNAVAILABLE");
            };
            if self.estop_pending() {
                drop(driver);
                self.driver = Some(d);
                return Err("ESTOP_ACTIVE");
            }
            if driver.tx().check_watchdog_tx_budget_for_enable().is_err() {
                drop(driver);
                self.driver = Some(d);
                return Err("WATCHDOG_BUDGET_INSUFFICIENT");
            }
            self.refresh_owner_driver_snapshot_from_ref(driver.as_ref());
            drop(driver);
            self.driver = Some(d);
        }
        Ok((was_torqued, prev_gripper_cmd, mit_qualified))
    }

    /// Starts the single budgeted enable reducer (`Enable -> Validate -> Armed | FailedHold`).
    fn start_enable_reducer(
        &mut self,
        origin: EnableOrigin,
        was_torqued: bool,
        prev_gripper_cmd: Option<f32>,
        authority: Option<EnableAuthority>,
        budgeted: bool,
        mit_qualified: bool,
    ) -> EnableStepOutcome {
        self.start_enable_reducer_inner(
            origin,
            was_torqued,
            prev_gripper_cmd,
            authority,
            budgeted,
            false,
            mit_qualified,
        )
    }

    /// Computes the owner-clock authority ceiling (`min(lease_expiry, request_deadline)`) so
    /// Damiao/Feetech enforce authority immediately before every `CMD_ENABLE` / torque-on frame.
    fn compute_enable_owner_deadline(
        &self,
        owner_now: Instant,
        auth: Option<EnableAuthority>,
        req_deadline: Option<Instant>,
    ) -> Option<Instant> {
        let lease_exp = auth.and_then(|a| {
            if self.lease_enabled && a.lease_generation != 0 {
                Some(
                    self.active_lease
                        .as_ref()
                        .filter(|l| l.generation == a.lease_generation)
                        .map_or(owner_now, |l| l.acquired_at + self.lease_timeout),
                )
            } else {
                None
            }
        });
        match (lease_exp, req_deadline) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }

    fn start_enable_reducer_inner(
        &mut self,
        origin: EnableOrigin,
        was_torqued: bool,
        prev_gripper_cmd: Option<f32>,
        authority: Option<EnableAuthority>,
        budgeted: bool,
        retry_unverified_ranges: bool,
        mit_qualified: bool,
    ) -> EnableStepOutcome {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        let reducer = EnableReducer {
            origin,
            phase: EnableReducerPhase::Enable,
            was_torqued,
            prev_gripper_cmd,
            authority,
            budgeted,
            mit_qualified,
        };
        self.torque_intent = TorqueIntent::On;
        self.enable_lifecycle = EnableLifecycle::Enabling;
        #[cfg(not(target_arch = "wasm32"))]
        {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            self.shadow_enable_op_id = live_op_id;
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::EnableStart {
                    live_op_id,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
        }
        if self.driver.is_none() {
            return self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            );
        }
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let owner_now = Instant::now();
        let req_dl = match origin {
            EnableOrigin::LegacyWs { deadline, .. } => Some(deadline),
            _ => None,
        };
        let owner_dl = self.compute_enable_owner_deadline(owner_now, authority, req_dl);
        let expected_lease_gen = authority
            .and_then(|a| (a.lease_generation != 0).then_some(a.lease_generation));
        let measured_positions = &self.measured_positions;
        let pending_bus_loss_notify = &mut self.pending_driver_bus_loss_notify;
        let res = {
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            let run_start = |drv: &mut dyn crate::driver::ArmDriver| {
                if *pending_bus_loss_notify {
                    drv.bus_recovery_mut().notify_bus_loss();
                    *pending_bus_loss_notify = false;
                }
                let enable_fb = drv.capabilities().enable_feedback;
                let drv_now = drv.clock_now();
                let tx_dl = owner_dl.map(|dl| drv_now + dl.saturating_duration_since(owner_now));
                drv.tx_mut().set_enable_authority(tx_dl, expected_lease_gen);
                let started = if budgeted && retry_unverified_ranges {
                    drv.begin_torque_budgeted_with_range_retry(true, 0)
                } else if budgeted {
                    drv.begin_torque_budgeted(true, 0)
                } else {
                    drv.begin_torque(true)
                };
                let snap = OwnerDriverSnapshot::capture_from_driver(
                    &*drv,
                    dof,
                    feedback_numeric_valid,
                    measured_positions,
                    &mut ages_buf,
                    &mut bounds_buf,
                );
                #[cfg(not(target_arch = "wasm32"))]
                shadow_rt.drain_from_driver(drv);
                (started, enable_fb, drv_now, snap)
            };
            if budgeted {
                d.try_with_mut(run_start).ok()
            } else {
                d.with_mut(run_start)
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        let Some((started, enable_fb, drv_now, snap)) = res else {
            self.owner_driver_snapshot.invalidate();
            let message = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str("driver mutex unavailable during enable");
            });
            return EnableStepOutcome::Failed {
                rt_code: "FEEDBACK_UNAVAILABLE",
                message,
                feedback_unavailable: true,
                estop_interrupted: false,
            };
        };
        self.owner_driver_snapshot = snap;
        match started {
            Ok(Drv::Pending { wake }) => {
                self.torque_transition = Some(TorqueTransition::Enable(reducer));
                EnableStepOutcome::Pending {
                    wake: if budgeted { drv_now } else { wake },
                }
            }
            Ok(Drv::Done(TorqueOutcome::Confirmed)) => self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            ),
            Ok(Drv::Done(TorqueOutcome::Unconfirmed)) if !enable_fb => self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            ),
            Ok(Drv::Done(TorqueOutcome::Unconfirmed)) => {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable not confirmed");
                });
                EnableStepOutcome::Failed {
                    rt_code: "FEEDBACK_UNCONFIRMED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                }
            }
            Err(e) => {
                let rt_code = enable_driver_err_code(&e);
                let estop_interrupted = crate::driver::is_estop_inhibited(&e);
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    e.write_bounded(buf);
                });
                EnableStepOutcome::Failed {
                    rt_code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted,
                }
            }
        }
    }

    /// Drains an in-flight enable cancellation (`EnableReducerPhase::Cancelling`) with zero enable
    /// TX (zero-lock on `DriverSlot::Exclusive`, nonblocking `try_with_mut` on `DriverSlot::Legacy`)
    /// until `drv.abort_enable()` clears the driver's `torque_hs` so stopped-hold writes can never
    /// stay blocked by `TorqueTransitionPending`.
    fn step_enable_cancel(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
    ) -> EnableStepOutcome {
        let mut drv_any_enabled = false;
        let mut contended = false;
        if let Some(mut d) = self.driver.take() {
            match d.try_with_mut(|drv| {
                let enable_dispatched = drv.enable_frames_dispatched();
                drv.abort_enable();
                self.refresh_owner_driver_snapshot_from_ref(&*drv);
                let any = enable_dispatched
                    || drv.enable_state() != EnableState::Disabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable();
                if reducer.was_torqued || any {
                    let (hold_mask, _) = self.capture_measured_hold_from_owner();
                    drv.tx_mut().drop_pending_holds_outside_mask(hold_mask);
                }
                any
            }) {
                Ok(any) => {
                    drv_any_enabled = any;
                }
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    contended = true;
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    self.owner_driver_snapshot.invalidate();
                    drv_any_enabled = true;
                    let _ = self.capture_measured_hold_from_owner();
                }
            }
            self.driver = Some(d);
        }
        if contended {
            self.torque_transition = Some(TorqueTransition::Enable(reducer));
            let wake = budget.map_or_else(Instant::now, |b| b.now);
            return EnableStepOutcome::Pending { wake };
        }
        let restore_intent = self.detached_cancel_restore_intent.take();
        if reducer.was_torqued || drv_any_enabled {
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
            self.is_torqued = true;
            self.torque_intent = TorqueIntent::On;
            self.pending_rearm = true;
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::HandshakeResult {
                    live_op_id: self.shadow_enable_op_id,
                    confirmed_mask: crate::torque::ChannelMask::EMPTY,
                    outcome: crate::torque::HandshakeOutcome::Failed,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        } else if let Some(prev_intent) = restore_intent {
            // Reconcile provisional FailedHold from contended clean Stop once driver confirms
            // zero enable frames were dispatched and zero joints are energized (R5 M5).
            if !self.safety.state().is_stopped() && !self.estop_pending() {
                self.enable_lifecycle = EnableLifecycle::Disarmed;
                self.is_torqued = false;
                self.torque_intent = prev_intent;
                self.pending_rearm = false;
                self.measured_hold.clear();
                #[cfg(not(target_arch = "wasm32"))]
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::EnableCancelledPreTx {
                        live_op_id: self.shadow_enable_op_id,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    },
                );
            }
        }
        EnableStepOutcome::Failed {
            rt_code: "CANCELLED",
            message: crate::safety::BoundedText::new("enable cancellation drained"),
            feedback_unavailable: false,
            estop_interrupted: false,
        }
    }

    /// True when a protective (thermal or process-exit) hardware disable is currently pending.
    #[inline]
    pub(crate) fn is_protective_disable_active(&self) -> bool {
        self.disable_pending
            && matches!(
                self.active_disable_kind,
                Some(crate::driver::DisableKind::DisableProtective { .. })
            )
    }

    pub(crate) fn sync_detached_torque_flag(&self) {
        self.detached_torque_pending_atomic
            .store(self.has_detached_torque_transition(), Ordering::Release);
    }

    /// Retains `EnableOrigin::DetachedCancel` (`Cancelling` phase) when a driver mutex was
    /// contended during an enable abort, provided no newer transition or protective disable owns
    /// the slot. A later release drains `drv.abort_enable()` with zero enable TX.
    pub(crate) fn retain_detached_enable_cancel_if_clear(&mut self, was_torqued: bool) {
        if self.torque_transition.is_none() && !self.is_protective_disable_active() {
            self.torque_transition = Some(TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::DetachedCancel,
                phase: EnableReducerPhase::Cancelling,
                was_torqued,
                prev_gripper_cmd: None,
                authority: None,
                budgeted: true,
                mit_qualified: false,
            }));
        }
        self.sync_detached_torque_flag();
    }

    /// Single non-blocking enable-abort helper shared across Stop, ParkRearm abort, timeout, and
    /// E-stop paths. Returns `(drv_any_enabled, contended)`. When `retain_on_contention` is true
    /// and `driver.try_lock()` returns `WouldBlock`, installs `DetachedCancel` so a later release
    /// drains `drv.abort_enable()` with zero enable TX.
    pub(crate) fn abort_enable_driver_nonblocking(
        &mut self,
        was_torqued: bool,
        retain_on_contention: bool,
    ) -> (bool, bool) {
        let mut drv_any_enabled = false;
        let mut contended = false;
        if let Some(mut d) = self.driver.take() {
            match d.try_with_mut(|drv| {
                let enable_dispatched = drv.enable_frames_dispatched();
                drv.abort_enable();
                self.refresh_owner_driver_snapshot_from_ref(&*drv);
                let any = enable_dispatched
                    || drv.enable_state() != EnableState::Disabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable();
                if was_torqued || any {
                    let (hold_mask, _) = self.capture_measured_hold_from_owner();
                    drv.tx_mut().drop_pending_holds_outside_mask(hold_mask);
                }
                any
            }) {
                Ok(any) => {
                    drv_any_enabled = any;
                }
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                    contended = true;
                    drv_any_enabled = true;
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                    drv_any_enabled = true;
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
            }
            self.driver = Some(d);
        }
        if contended && retain_on_contention {
            self.retain_detached_enable_cancel_if_clear(was_torqued);
        }
        (drv_any_enabled, contended)
    }

    /// Cancels an in-flight `EnableOrigin::ParkRearm` cleanly on client Stop without latching
    /// `EnableFailed`, preserving holding torque on any energized joints (non-blocking `try_lock`,
    /// retaining `DetachedCancel` on contention).
    fn cancel_park_rearm_on_stop(&mut self, action_id: u64, was_torqued: bool) {
        let (drv_any_enabled, _) = self.abort_enable_driver_nonblocking(was_torqued, true);
        self.is_torqued = was_torqued || drv_any_enabled;
        self.pending_rearm = true;
        if self.park_continuation.is_some_and(|c| c.action_id == action_id) {
            self.park_continuation = None;
            self.notify_park_continuation();
        }
        if self
            .last_action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.is_running())
        {
            self.finish_action(action_id, "cancelled", Some("STOPPED"), None, None, None, None);
        }
    }

    /// Advances the single budgeted enable reducer by one poll step (`Enable -> Validate -> Armed | FailedHold`).
    /// Re-verifies live [`EnableAuthority`] BEFORE any subsequent enable retry TX so a displaced
    /// lease, disconnect, E-stop, fault, or epoch change transitions immediately to `FailedHold`
    /// without sending rollback disable frames.
    fn poll_enable_reducer(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
    ) -> (EnableStepOutcome, Option<(u16, bool)>) {
        let entry_wall = Instant::now();
        let base = budget.map_or(entry_wall, |b| b.now);
        let mut live_now = || base + Instant::now().saturating_duration_since(entry_wall);
        self.poll_enable_reducer_with_clock(reducer, budget, &mut live_now)
    }

    fn poll_enable_reducer_with_clock(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> (EnableStepOutcome, Option<(u16, bool)>) {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        if matches!(reducer.phase, EnableReducerPhase::Cancelling) {
            return (self.step_enable_cancel(reducer, budget), None);
        }
        if let Some(auth) = reducer.authority {
            let check_now = match reducer.origin {
                EnableOrigin::LegacyWs { .. } => Instant::now(),
                _ => budget.map_or_else(Instant::now, |b| b.now),
            };
            if let Some(code) = self.enable_authority_revoked(auth, check_now) {
                let estop = code == "ESTOP_ACTIVE";
                let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                    buf.push_str("enable authority revoked: ");
                    buf.push_str(code);
                });
                return (
                    EnableStepOutcome::Failed {
                        rt_code: code,
                        message,
                        feedback_unavailable: false,
                        estop_interrupted: estop,
                    },
                    None,
                );
            }
        }
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            ..
        } = reducer.origin
        {
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    reducer.was_torqued,
                    prev_torque_intent,
                );
                return (
                    EnableStepOutcome::Failed {
                        rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                        message: crate::safety::BoundedText::new(if estop {
                            "interrupted by E-stop"
                        } else {
                            "enable cancelled by stop"
                        }),
                        feedback_unavailable: false,
                        estop_interrupted: estop,
                    },
                    None,
                );
            }
        }
        if let EnableOrigin::ParkRearm { action_id, stop_gen } = reducer.origin {
            if self.urgent_stop_unapplied()
                || self.current_stop_gen() != stop_gen
                || !self
                    .park_continuation
                    .is_some_and(|c| c.action_id == action_id && c.stop_gen == stop_gen)
            {
                self.cancel_park_rearm_on_stop(action_id, reducer.was_torqued);
                return (
                    EnableStepOutcome::Failed {
                        rt_code: "STOPPED",
                        message: crate::safety::BoundedText::new("park rearm cancelled by stop"),
                        feedback_unavailable: false,
                        estop_interrupted: false,
                    },
                    None,
                );
            }
        }
        if self.estop_pending() || self.safety.state().is_stopped() {
            let estop = self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop);
            let rt_code = if estop { "ESTOP_ACTIVE" } else { "FAULT_ACTIVE" };
            let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                buf.push_str(if estop {
                    "interrupted by E-stop"
                } else {
                    "interrupted by active fault"
                });
            });
            return (
                EnableStepOutcome::Failed {
                    rt_code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                },
                None,
            );
        }
        match reducer.phase {
            EnableReducerPhase::Cancelling => (self.step_enable_cancel(reducer, budget), None),
            EnableReducerPhase::Validate => (
                self.step_enable_validate(reducer, budget, budget.is_some()),
                None,
            ),
            EnableReducerPhase::Enable => {
                if self.driver.is_none() {
                    let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                        buf.push_str("driver missing during enable poll");
                    });
                    return (
                        EnableStepOutcome::Failed {
                            rt_code: "INTERNAL",
                            message,
                            feedback_unavailable: false,
                            estop_interrupted: false,
                        },
                        None,
                    );
                }
                let mut scratch_states = std::mem::take(&mut self.scratch.states);
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let mut scratch_bounds = std::mem::take(&mut self.scratch.fb.bounds);
                let dof = self.config.dof;
                let feedback_numeric_valid = self.feedback_numeric_valid;
                let control_release_seq = self.control_release_seq;
                let owner_now = match reducer.origin {
                    EnableOrigin::LegacyWs { .. } => Instant::now(),
                    _ => budget.map_or_else(Instant::now, |b| b.now),
                };
                let req_dl = match reducer.origin {
                    EnableOrigin::LegacyWs { deadline, .. } => Some(deadline),
                    _ => None,
                };
                let owner_dl = self.compute_enable_owner_deadline(owner_now, reducer.authority, req_dl);
                let expected_lease_gen = reducer
                    .authority
                    .and_then(|a| (a.lease_generation != 0).then_some(a.lease_generation));
                let measured_positions = &self.measured_positions;
                let conservative_cap = self.pending_conservative_tx_cap;
                let mut cons_rx_delta = 0u16;
                let mut cons_tx_delta = 0u16;
                let lock_res = {
                    #[cfg(not(target_arch = "wasm32"))]
                    let shadow_rt = &mut self.torque_shadow;
                    let d = self.driver.as_mut().expect("checked is_some");
                    let run_poll = |drv: &mut dyn crate::driver::ArmDriver| {
                        if control_release_seq > 0 {
                            drv.begin_control_release(control_release_seq);
                        }
                        let caps = drv.capabilities();
                        let enable_fb = caps.enable_feedback;
                        let temp_fb = caps.temperature_feedback;
                        let (drv_now, r, rx, expired_defer) = if let Some(b) = budget {
                            match crate::supervisor::translate_driver_cutoff_conservative(
                                drv,
                                b.deadline,
                                now_fn,
                            ) {
                                Ok((drv_now, live_owner_now, drv_deadline)) => {
                                    let tx_dl = owner_dl.map(|dl| {
                                        drv_now + dl.saturating_duration_since(live_owner_now)
                                    });
                                    drv.tx_mut().set_enable_authority(tx_dl, expected_lease_gen);
                                    let before_cons = if conservative_cap {
                                        let bc = drv.tx().release_budget_consumed();
                                        drv.tx_mut().sync_release_budget_limits(0, 0);
                                        Some(bc)
                                    } else {
                                        None
                                    };
                                    let r = if reducer.budgeted {
                                        drv.poll_torque_budgeted(drv_deadline, b.max_rx_frames)
                                    } else {
                                        drv.poll_torque(drv_deadline)
                                    };
                                    if let Some(bc) = before_cons {
                                        let after = drv.tx().release_budget_consumed();
                                        cons_rx_delta = cons_rx_delta.saturating_add(after.0.saturating_sub(bc.0));
                                        cons_tx_delta = cons_tx_delta.saturating_add(after.1.saturating_sub(bc.1));
                                    }
                                    let rx = drv.take_last_poll_rx();
                                    (drv_now, r, rx, None)
                                }
                                Err((drv_now, live_owner_now)) => (
                                    drv_now,
                                    Ok(Drv::Pending { wake: drv_now }),
                                    Some((0, true)),
                                    Some(live_owner_now),
                                ),
                            }
                        } else {
                            let drv_now = drv.clock_now();
                            let tx_dl = owner_dl.map(|dl| drv_now + dl.saturating_duration_since(owner_now));
                            drv.tx_mut().set_enable_authority(tx_dl, expected_lease_gen);
                            let r = drv.poll_torque(drv_now + crate::driver::DAMIAO_RX_PASS);
                            (drv_now, r, None, None)
                        };
                        let mut cached_temps_read = false;
                        if expired_defer.is_none()
                            && temp_fb
                            && drv.read_cached_joint_states_into(&mut scratch_states).is_some()
                        {
                            drv.feedback_ages_into(&mut scratch_ages);
                            cached_temps_read = true;
                        }
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            measured_positions,
                            &mut scratch_ages,
                            &mut scratch_bounds,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        shadow_rt.drain_from_driver(drv);
                        (r, rx, enable_fb, temp_fb, drv_now, cached_temps_read, snap, expired_defer)
                    };
                    if budget.is_some() || reducer.budgeted {
                        match d.try_with_mut(run_poll) {
                            Ok(v) => Ok(v),
                            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                            Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                        }
                    } else {
                        d.with_mut(run_poll).ok_or(false)
                    }
                };
                if conservative_cap {
                    self.conservative_step_rx_delta =
                        self.conservative_step_rx_delta.saturating_add(cons_rx_delta);
                    self.conservative_step_tx_delta =
                        self.conservative_step_tx_delta.saturating_add(cons_tx_delta);
                }
                let (poll_res, rx, enable_fb, temp_fb, drv_now, cached_temps_read, snap, expired_defer) = match lock_res {
                    Ok(v) => v,
                    Err(true) => {
                        self.scratch.states = scratch_states;
                        self.scratch.ages = scratch_ages;
                        self.scratch.fb.bounds = scratch_bounds;
                        self.torque_transition = Some(TorqueTransition::Enable(reducer));
                        let wake = budget.map_or_else(Instant::now, |b| b.now);
                        return (EnableStepOutcome::Pending { wake }, None);
                    }
                    Err(false) => {
                        self.scratch.states = scratch_states;
                        self.scratch.ages = scratch_ages;
                        self.scratch.fb.bounds = scratch_bounds;
                        self.owner_driver_snapshot.invalidate();
                        let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            buf.push_str("driver mutex poisoned during enable poll");
                        });
                        return (
                            EnableStepOutcome::Failed {
                                rt_code: "FEEDBACK_UNAVAILABLE",
                                message,
                                feedback_unavailable: true,
                                estop_interrupted: false,
                            },
                            None,
                        );
                    }
                };
                self.owner_driver_snapshot = snap;
                if let Some(wake) = expired_defer {
                    self.scratch.states = scratch_states;
                    self.scratch.ages = scratch_ages;
                    self.scratch.fb.bounds = scratch_bounds;
                    self.torque_transition = Some(TorqueTransition::Enable(reducer));
                    return (EnableStepOutcome::Pending { wake }, rx);
                }
                if cached_temps_read {
                    let n = self.config.dof.min(self.temperatures.len()).min(scratch_states.len());
                    for i in 0..n {
                        if scratch_ages.get(i).copied().flatten().is_some()
                            && scratch_states[i].temperature.is_finite()
                        {
                            self.temperatures[i] = scratch_states[i].temperature;
                        }
                    }
                }
                self.scratch.states = scratch_states;
                self.scratch.ages = scratch_ages;
                self.scratch.fb.bounds = scratch_bounds;
                if let Some(failed) = self.check_enable_thermal_trip(reducer.was_torqued, temp_fb) {
                    return (failed, rx);
                }
                match poll_res {
                    Ok(Drv::Pending { wake }) => {
                        self.torque_transition = Some(TorqueTransition::Enable(reducer));
                        let mapped_wake = match budget {
                            Some(b) => b.now + wake.saturating_duration_since(drv_now),
                            None => wake,
                        };
                        (EnableStepOutcome::Pending { wake: mapped_wake }, rx)
                    }
                    Ok(Drv::Done(TorqueOutcome::Confirmed)) => (
                        self.step_enable_validate(
                            EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                            budget,
                            budget.is_some(),
                        ),
                        rx,
                    ),
                    Ok(Drv::Done(TorqueOutcome::Unconfirmed)) if !enable_fb => (
                        self.step_enable_validate(
                            EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                            budget,
                            budget.is_some(),
                        ),
                        rx,
                    ),
                    Ok(Drv::Done(TorqueOutcome::Unconfirmed)) => {
                        let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            buf.push_str("enable not confirmed");
                        });
                        (
                            EnableStepOutcome::Failed {
                                rt_code: "FEEDBACK_UNCONFIRMED",
                                message,
                                feedback_unavailable: false,
                                estop_interrupted: false,
                            },
                            rx,
                        )
                    }
                    Err(e) => {
                        // Major 1: If the driver stopped an enable poll due to a stop-stamp change
                        // (`EstopInhibited`) on `EnableOrigin::LegacyWs`, resolve live E-stop vs
                        // ordinary client Stop so an ordinary Stop aborts cleanly without latching
                        // `EnableFailed` or falsely reporting `ESTOP_ACTIVE`.
                        if crate::driver::is_estop_inhibited(&e) {
                            if let EnableOrigin::LegacyWs {
                                token,
                                slot,
                                stop_gen,
                                prev_torque_intent,
                                ..
                            } = reducer.origin
                            {
                                let is_estop = self.estop_pending()
                                    || self.e_stop_latched.load(Ordering::SeqCst)
                                    || matches!(self.safety.state(), SafetyState::EStop);
                                if !is_estop
                                    && (self.urgent_stop_unapplied()
                                        || self.current_stop_gen() != stop_gen)
                                {
                                    let estop = self.cancel_legacy_enable_on_stop(
                                        token,
                                        slot,
                                        reducer.was_torqued,
                                        prev_torque_intent,
                                    );
                                    return (
                                        EnableStepOutcome::Failed {
                                            rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                                            message: crate::safety::BoundedText::new(if estop {
                                                "interrupted by E-stop"
                                            } else {
                                                "enable cancelled by stop"
                                            }),
                                            feedback_unavailable: false,
                                            estop_interrupted: estop,
                                        },
                                        rx,
                                    );
                                }
                            }
                        }
                        let mut rt_code = enable_driver_err_code(&e);
                        if matches!(e, crate::driver::ArmDriverError::EnableAuthorityExpired) {
                            if let EnableOrigin::LegacyWs { deadline, .. } = reducer.origin {
                                let now = Instant::now();
                                let auth_revoked = reducer
                                    .authority
                                    .and_then(|a| self.enable_authority_revoked(a, now));
                                if let Some(code) = auth_revoked {
                                    rt_code = code;
                                } else if now >= deadline {
                                    rt_code = "DEADLINE_EXPIRED";
                                }
                            }
                        }
                        let live_estop = self.estop_pending()
                            || self.e_stop_latched.load(Ordering::SeqCst)
                            || matches!(self.safety.state(), SafetyState::EStop);
                        let estop_interrupted =
                            crate::driver::is_estop_inhibited(&e) || live_estop;
                        if live_estop {
                            rt_code = "ESTOP_ACTIVE";
                        }
                        let message = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            e.write_bounded(buf);
                        });
                        (
                            EnableStepOutcome::Failed {
                                rt_code,
                                message,
                                feedback_unavailable: false,
                                estop_interrupted,
                            },
                            rx,
                        )
                    }
                }
            }
        }
    }

    /// Post-enable validation phase of the single enable reducer (`Validate -> Armed | FailedHold`).
    /// Any validation failure or interruption routes through [`Self::enter_failed_hold`] (zero
    /// automatic rollback at any pose).
    fn step_enable_validate(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
        use_cached: bool,
    ) -> EnableStepOutcome {
        let was_torqued = reducer.was_torqued;
        let prev_gripper_cmd = reducer.prev_gripper_cmd;
        let check_now = match reducer.origin {
            EnableOrigin::LegacyWs { .. } => Instant::now(),
            _ => budget.map_or_else(Instant::now, |b| b.now),
        };
        if let Some(auth) = reducer.authority {
            if let Some(code) = self.enable_authority_revoked(auth, check_now) {
                let estop = code == "ESTOP_ACTIVE";
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable authority revoked before validation: ");
                    buf.push_str(code);
                });
                return EnableStepOutcome::Failed {
                    rt_code: code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            deadline,
        } = reducer.origin
        {
            if self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("interrupted by E-stop before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "ESTOP_ACTIVE",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: true,
                };
            }
            if Instant::now() >= deadline {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable request deadline expired before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "DEADLINE_EXPIRED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    was_torqued,
                    prev_torque_intent,
                );
                return EnableStepOutcome::Failed {
                    rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                    message: crate::safety::BoundedText::new(if estop {
                        "interrupted by E-stop before arming"
                    } else {
                        "enable cancelled by stop before arming"
                    }),
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if self.estop_pending() || self.safety.state().is_stopped() {
            let estop = self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop);
            let rt_code = if estop { "ESTOP_ACTIVE" } else { "FAULT_ACTIVE" };
            let message = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str(if estop {
                    "interrupted by E-stop before validation"
                } else {
                    "interrupted by active fault before validation"
                });
            });
            return EnableStepOutcome::Failed {
                rt_code,
                message,
                feedback_unavailable: false,
                estop_interrupted: estop,
            };
        }

        if let Some(mut d) = self.driver.take() {
            let mut scratch_states = std::mem::take(&mut self.scratch.states);
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_loads = std::mem::take(&mut self.scratch.loads);
            let run_validate = |driver: &mut dyn crate::driver::ArmDriver| {
                let temp_fb = driver.capabilities().temperature_feedback;
                let read_res: Result<(), ()> = if use_cached {
                    if driver.read_cached_joint_states_into(&mut scratch_states).is_some() {
                        Ok(())
                    } else if let Some(b) = budget {
                        let entry_wall = Instant::now();
                        let mut live_now =
                            || b.now + Instant::now().saturating_duration_since(entry_wall);
                        match crate::supervisor::translate_driver_cutoff_conservative(
                            driver,
                            b.deadline,
                            &mut live_now,
                        ) {
                            Ok((_, _, drv_deadline)) => driver
                                .read_joint_states_budgeted(
                                    &mut scratch_states,
                                    drv_deadline,
                                    b.max_rx_frames,
                                )
                                .map(|_| ())
                                .map_err(|_| ()),
                            Err(_) => Err(()),
                        }
                    } else {
                        Err(())
                    }
                } else {
                    driver.read_joint_states_into(&mut scratch_states).map_err(|_| ())
                };
                if read_res.is_err() {
                    (Err("post-enable feedback unavailable"), temp_fb)
                } else {
                    let has_loads = driver.joint_loads_into(&mut scratch_loads);
                    let loads_slice = has_loads.then_some(scratch_loads.as_slice());
                    if !driver.is_feedback_valid()
                        || !Self::states_valid(&scratch_states, driver.capabilities(), self.config.dof, loads_slice)
                        || (driver.capabilities().enable_feedback && driver.enable_state() != EnableState::Enabled)
                    {
                        // Preserve any finite received temperatures for thermal enforcement
                        // even when another joint failed post-enable validation.
                        if temp_fb {
                            driver.feedback_ages_into(&mut scratch_ages);
                            let n = self.config.dof.min(self.temperatures.len()).min(scratch_states.len());
                            for i in 0..n {
                                if scratch_ages.get(i).copied().flatten().is_some()
                                    && scratch_states[i].temperature.is_finite()
                                {
                                    self.temperatures[i] = scratch_states[i].temperature;
                                }
                            }
                        }
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.drain_from_driver(driver);
                        (Err("post-enable feedback invalid or enable unconfirmed"), temp_fb)
                    } else {
                        driver.feedback_ages_into(&mut scratch_ages);
                        let oldest_age = if scratch_ages.iter().all(Option::is_some) {
                            scratch_ages.iter().flatten().copied().max()
                        } else {
                            None
                        };
                        self.driver_caps = driver.capabilities();
                        self.feedback_numeric_valid = true;
                        self.last_can_feedback_instant = oldest_age.and_then(|age| Instant::now().checked_sub(age));
                        self.last_poll_enable_state = driver.enable_state();
                        self.last_poll_partial_enable = driver.has_known_partial_enable();
                        self.last_poll_feedback_valid = true;
                        self.last_poll_oldest_age = oldest_age;
                        self.core.poll_feedback(&mut self.joint_feedback_queue_bounds, self.config.dof, &*driver, &mut self.scratch.fb);
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
                        self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                            &*driver,
                            self.config.dof,
                            self.feedback_numeric_valid,
                            &self.measured_positions,
                            &mut scratch_ages,
                            &mut self.scratch.fb.bounds,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.drain_from_driver(driver);
                        (Ok(()), temp_fb)
                    }
                }
            };
            let lock_out = if budget.is_some() || reducer.budgeted {
                match d.try_with_mut(run_validate) {
                    Ok(v) => Ok(v),
                    Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                    Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                }
            } else {
                d.with_mut(run_validate).ok_or(false)
            };
            self.driver = Some(d);
            let (validate_res, temp_fb) = match lock_out {
                Ok(v) => v,
                Err(true) => {
                    self.scratch.states = scratch_states;
                    self.scratch.ages = scratch_ages;
                    self.scratch.loads = scratch_loads;
                    self.torque_transition = Some(TorqueTransition::Enable(reducer));
                    return EnableStepOutcome::Pending { wake: check_now };
                }
                Err(false) => (Err("driver mutex poisoned during validation"), false),
            };
            self.scratch.states = scratch_states;
            self.scratch.ages = scratch_ages;
            self.scratch.loads = scratch_loads;
            if let Some(failed) = self.check_enable_thermal_trip(was_torqued, temp_fb) {
                return failed;
            }
            if let Err(cause) = validate_res {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str(cause);
                });
                return EnableStepOutcome::Failed {
                    rt_code: "FEEDBACK_UNAVAILABLE",
                    message,
                    feedback_unavailable: true,
                    estop_interrupted: false,
                };
            }
        } else if let Some(failed) = self.check_enable_thermal_trip(was_torqued, true) {
            return failed;
        }

        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            deadline,
        } = reducer.origin
        {
            if self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("interrupted by E-stop before arming");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "ESTOP_ACTIVE",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: true,
                };
            }
            if Instant::now() >= deadline {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable request deadline expired before arming");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "DEADLINE_EXPIRED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    was_torqued,
                    prev_torque_intent,
                );
                return EnableStepOutcome::Failed {
                    rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                    message: crate::safety::BoundedText::new(if estop {
                        "interrupted by E-stop"
                    } else {
                        "enable cancelled by stop"
                    }),
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if let EnableOrigin::ParkRearm { action_id, stop_gen } = reducer.origin {
            let cont_lost = budget.is_some()
                && !self
                    .park_continuation
                    .is_some_and(|c| c.action_id == action_id && c.stop_gen == stop_gen);
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen || cont_lost {
                self.cancel_park_rearm_on_stop(action_id, was_torqued);
                return EnableStepOutcome::Failed {
                    rt_code: "STOPPED",
                    message: crate::safety::BoundedText::new("park rearm cancelled by stop"),
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
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
            let has_vel_fb = self.driver.is_none() || self.driver_caps.velocity_feedback;
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
        self.replace_active_trajectory(None);
        self.pending_normal_unload = false;
        self.clear_unload_settle();
        self.watchdog_park = None;
        self.watchdog_park_pending = None;
        self.servo_target = None;
        self.servo_stream_active = false;
        self.clear_servo_ws_min_depths_rt();
        self.gripper_hold_vel_ceiling = None;

        if let Some(auth) = reducer.authority {
            let post_now = match reducer.origin {
                EnableOrigin::LegacyWs { .. } => Instant::now(),
                _ => budget.map_or_else(Instant::now, |b| b.now),
            };
            if let Some(code) = self.enable_authority_revoked(auth, post_now) {
                let estop = code == "ESTOP_ACTIVE";
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable authority revoked before arming: ");
                    buf.push_str(code);
                });
                return EnableStepOutcome::Failed {
                    rt_code: code,
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            stop_gen,
            prev_torque_intent,
            deadline,
        } = reducer.origin
        {
            if self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop)
            {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("interrupted by E-stop before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "ESTOP_ACTIVE",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: true,
                };
            }
            if Instant::now() >= deadline {
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("enable request deadline expired before validation");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "DEADLINE_EXPIRED",
                    message,
                    feedback_unavailable: false,
                    estop_interrupted: false,
                };
            }
            if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
                let estop = self.cancel_legacy_enable_on_stop(
                    token,
                    slot,
                    was_torqued,
                    prev_torque_intent,
                );
                return EnableStepOutcome::Failed {
                    rt_code: if estop { "ESTOP_ACTIVE" } else { "STOPPED" },
                    message: crate::safety::BoundedText::new(if estop {
                        "interrupted by E-stop before arming"
                    } else {
                        "enable cancelled by stop before arming"
                    }),
                    feedback_unavailable: false,
                    estop_interrupted: estop,
                };
            }
        }
        if self.estop_pending() || self.safety.state().is_stopped() {
            let estop = self.estop_pending() || matches!(self.safety.state(), SafetyState::EStop);
            let rt_code = if estop { "ESTOP_ACTIVE" } else { "FAULT_ACTIVE" };
            let message = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str(if estop {
                    "interrupted by E-stop before arming"
                } else {
                    "interrupted by active fault before arming"
                });
            });
            return EnableStepOutcome::Failed {
                rt_code,
                message,
                feedback_unavailable: false,
                estop_interrupted: estop,
            };
        }
        let armed_now = budget.map_or_else(Instant::now, |b| b.now);
        if use_cached {
            match self.safety.set_torque_rt(true) {
                Ok(()) => {
                    self.enable_lifecycle = EnableLifecycle::Armed;
                    self.measured_hold.clear();
                    self.is_torqued = true;
                    self.pending_rearm = false;
                    self.disable_pending = false;
                    self.background_disable_retry = None;
                    self.mit.armed_ok = reducer.mit_qualified;
                    self.mit.handback_suppress_until = None;
                    self.mit.handover_error_rt = None;
                    if !self.rt_executor_owned {
                        self.mit.handover_error = None;
                    }
                    self.last_heartbeat = armed_now;
                    self.last_motion_frame = armed_now;
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let dof = self.config.dof.min(crate::torque::MAX_CHANNELS) as u8;
                        self.torque_shadow
                            .enqueue(crate::torque::shadow::ShadowInputKind::HandshakeResult {
                                live_op_id: self.shadow_enable_op_id,
                                confirmed_mask: crate::torque::ChannelMask::from_count(dof),
                                outcome: crate::torque::HandshakeOutcome::Confirmed,
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            });
                    }
                    EnableStepOutcome::Armed(SafetyState::Armed)
                }
                Err(rt_code) => {
                    let estop_interrupted = rt_code == "ESTOP_ACTIVE";
                    let message = self.enter_failed_hold(was_torqued, |buf| {
                        buf.push_str(rt_code);
                    });
                    EnableStepOutcome::Failed {
                        rt_code,
                        message,
                        feedback_unavailable: false,
                        estop_interrupted,
                    }
                }
            }
        } else {
            match self.safety.set_torque(true) {
                Ok(state) => {
                    self.enable_lifecycle = EnableLifecycle::Armed;
                    self.measured_hold.clear();
                    self.is_torqued = true;
                    self.pending_rearm = false;
                    self.disable_pending = false;
                    self.background_disable_retry = None;
                    self.mit.armed_ok = reducer.mit_qualified;
                    self.mit.handback_suppress_until = None;
                    self.mit.handover_error_rt = None;
                    if !self.rt_executor_owned {
                        self.mit.handover_error = None;
                    }
                    self.last_heartbeat = armed_now;
                    self.last_motion_frame = armed_now;
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let dof = self.config.dof.min(crate::torque::MAX_CHANNELS) as u8;
                        self.torque_shadow
                            .enqueue(crate::torque::shadow::ShadowInputKind::HandshakeResult {
                                live_op_id: self.shadow_enable_op_id,
                                confirmed_mask: crate::torque::ChannelMask::from_count(dof),
                                outcome: crate::torque::HandshakeOutcome::Confirmed,
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            });
                    }
                    EnableStepOutcome::Armed(state)
                }
                Err(e) => {
                    let rt_code = torque_error_code(&e);
                    let estop_interrupted = matches!(e, SafetyError::EStopActive);
                    let message = self.enter_failed_hold(was_torqued, |buf| {
                        buf.push_str(rt_code);
                    });
                    EnableStepOutcome::Failed {
                        rt_code,
                        message,
                        feedback_unavailable: false,
                        estop_interrupted,
                    }
                }
            }
        }
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

    fn torque_off_in_place(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        if let Err(err) = self.torque_off_check(stillness_verified) {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let live_op_id = self.torque_shadow.alloc_live_op_id();
                let live_refusal = match &err {
                    SafetyError::TorqueOffWouldDrop { .. } => {
                        crate::torque::shadow::LiveRefusalKind::TorqueOffWouldDrop
                    }
                    SafetyError::ArmNotAtRestingPose { .. } => {
                        crate::torque::shadow::LiveRefusalKind::ArmNotAtRestingPose
                    }
                    SafetyError::EStopActive => {
                        crate::torque::shadow::LiveRefusalKind::EStopActive
                    }
                    SafetyError::FaultActive(_) => {
                        crate::torque::shadow::LiveRefusalKind::FaultLatched
                    }
                    _ => crate::torque::shadow::LiveRefusalKind::Other,
                };
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: Some(live_refusal),
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    });
            }
            return Err(err);
        }
        let started_at = self.driver_clock_now();
        let disable_kind = crate::driver::DisableKind::DisableRequested { started_at };
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable(disable_kind) {
            let refused = matches!(error, SafetyError::TorqueOffWouldDrop { .. });
            if !refused && !self.torque_off_stopped() {
                self.safety.latch(
                    crate::safety::FaultKind::Other,
                    crate::safety::FaultDetail::formatted(FaultCode::DriverWriteFailed, |buf| {
                        let _ = core::fmt::write(buf, format_args!("{error}"));
                    }),
                );
            }
            return Err(error);
        }
        if !self.disable_pending {
            self.torque_off_finish_in_place()?;
        }
        Ok(())
    }

    /// Record one contended `driver.try_lock()` skip on an RT disable dispatch or poll path (M4).
    /// Escalates at `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS` (25 consecutive skips, 0.5 s at 50 Hz)
    /// with a counted diagnostic escalation and a non-recoverable `HardwareFault` (zero allocation).
    pub(crate) fn record_disable_lock_contention(&mut self) {
        crate::diagnostics::tick::record_disable_lock_contended();
        self.disable_lock_contended_skips = self.disable_lock_contended_skips.saturating_add(1);
        if self.disable_lock_contended_skips >= STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS
            && !self.disable_lock_contended_escalated
        {
            self.disable_lock_contended_escalated = true;
            crate::diagnostics::tick::record_disable_contention_escalation();
            let skips = self.disable_lock_contended_skips;
            let detail = crate::safety::FaultDetail::formatted(FaultCode::HardwareFault, |buf| {
                let _ = core::fmt::write(
                    buf,
                    format_args!(
                        "RT torque-disable path skipped {} consecutive ticks due to driver mutex contention",
                        skips
                    ),
                );
            });
            self.safety.latch(crate::safety::FaultKind::Other, detail);
        }
    }

    #[inline]
    pub(crate) fn reset_disable_lock_contention(&mut self) {
        self.disable_lock_contended_skips = 0;
        self.disable_lock_contended_escalated = false;
    }

    /// True when the driver can keep a stopped disable pending inside its 1 s lifetime while an
    /// exhausted RX drain, incomplete K=3 rest-pose window, or stale/missing evidence on a drained
    /// bus runs the probe loop (L1/M3). Non-blocking variant for `armd-control` (M4).
    fn can_defer_stopped_disable_to_driver_nonblocking(&self) -> Result<bool, ()> {
        let Some(d) = self.driver.as_ref() else {
            return Ok(false);
        };
        match d.try_with_ref(|drv| {
            drv.supports_budgeted_disable_await()
                && matches!(
                    drv.rest_pose_gate_status(),
                    crate::driver::RestPoseGateStatus::PendingDrainOrWindow
                        | crate::driver::RestPoseGateStatus::StaleOrMissing
                )
        }) {
            Ok(can) => Ok(can),
            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(()),
            Err(crate::supervisor_lock::DriverTryError::Poisoned) => Ok(false),
        }
    }

    /// True when the driver can keep a stopped disable pending inside its 1 s lifetime while an
    /// exhausted RX drain, incomplete K=3 rest-pose window, or stale/missing evidence on a drained
    /// bus runs the probe loop (L1/M3). Zero heap allocations.
    fn can_defer_stopped_disable_to_driver(&self) -> bool {
        if self.rt_executor_owned {
            return self.can_defer_stopped_disable_to_driver_nonblocking().unwrap_or(false);
        }
        self.driver.as_ref().is_some_and(|d| {
            d.with_ref(|drv| {
                drv.supports_budgeted_disable_await()
                    && matches!(
                        drv.rest_pose_gate_status(),
                        crate::driver::RestPoseGateStatus::PendingDrainOrWindow
                            | crate::driver::RestPoseGateStatus::StaleOrMissing
                    )
            })
            .unwrap_or(false)
        })
    }

    /// Zero-allocation RT torque-off precheck (static protocol error codes): enforces the
    /// unified rest-pose gate in EVERY safety state (operational and stopped alike), plus the
    /// operational stillness/support-torque/workspace check when not stopped.
    fn torque_off_check_rt(&mut self, stillness_verified: bool) -> Result<(), &'static str> {
        if self.calibration_busy && self.motor_zero_busy {
            return Err("MOTION_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if !self.torque_off_stopped() && self.has_resting_pose_violation_rt(stillness_verified) {
            return Err("ARM_NOT_AT_RESTING_POSE");
        }
        self.eval_stopped_torque_off()
            .map_err(|_| "TORQUE_OFF_WOULD_DROP")
    }

    /// Zero-allocation, non-blocking rest-pose gate for ALL RT budgeted torque-off transitions
    /// (operational and stopped alike): uses `driver.try_lock()` with counted escalation on
    /// contention, checks operational stillness/support-torque/workspace when not stopped,
    /// defers incomplete drains / K=3 windows / stale-or-missing evidence to `AwaitRestGate`,
    /// and refuses off-rest poses with zero heap allocations.
    fn torque_off_check_rt_transition(&mut self) -> Result<(), &'static str> {
        if self.calibration_busy && self.motor_zero_busy {
            return Err("MOTION_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
        }
        if !self.torque_off_stopped() {
            match self.check_resting_pose_rt(false) {
                super::RtRestingCheck::AtRest => {}
                super::RtRestingCheck::OffRest => return Err("ARM_NOT_AT_RESTING_POSE"),
                super::RtRestingCheck::LockContended => {
                    self.record_disable_lock_contention();
                    return Err("DRIVER_BUSY");
                }
            }
        }
        match self.may_disable_torque_while_stopped_nonblocking() {
            Err(()) => {
                self.record_disable_lock_contention();
                Err("DRIVER_BUSY")
            }
            Ok(Ok(())) => {
                self.last_stopped_torque_off_refusal = None;
                Ok(())
            }
            Ok(Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)) => {
                match self.can_defer_stopped_disable_to_driver_nonblocking() {
                    Err(()) => {
                        self.record_disable_lock_contention();
                        Err("DRIVER_BUSY")
                    }
                    Ok(true) => {
                        self.last_stopped_torque_off_refusal = None;
                        Ok(())
                    }
                    Ok(false) => {
                        self.last_stopped_torque_off_refusal =
                            Some(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                        Err("TORQUE_OFF_WOULD_DROP")
                    }
                }
            }
            Ok(Err(r)) => {
                self.last_stopped_torque_off_refusal = Some(r);
                Err("TORQUE_OFF_WOULD_DROP")
            }
        }
    }

    /// Unified torque-off refusals for operational and stopped states alike: when operational,
    /// checks the resting-pose stillness/support-torque/workspace policy first, then ALWAYS
    /// enforces the authoritative K=3 rest-pose gate (`eval_stopped_torque_off`).
    fn torque_off_check(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        if !self.torque_off_stopped() {
            let violation = if stillness_verified {
                self.resting_pose_static_violation()
            } else {
                self.resting_pose_violation()
            };
            if let Some(reason) = violation {
                tracing::warn!("Torque off refused, not at resting pose: {reason}");
                return Err(SafetyError::ArmNotAtRestingPose {
                    current: self.measured_positions.clone(),
                    expected: self.config.resting_positions().to_vec(),
                    reason: Some(reason),
                });
            }
        }
        match self.eval_stopped_torque_off() {
            Ok(()) => Ok(()),
            Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)
                if self.can_defer_stopped_disable_to_driver() =>
            {
                self.last_stopped_torque_off_refusal = None;
                Ok(())
            }
            Err(r) => Err(r.into_safety_error(&self.config.joint_names)),
        }
    }

    /// Torque off after the accepted disable without cloning `SafetyState` (zero-alloc on RT).
    fn torque_off_finish_in_place(&mut self) -> Result<(), SafetyError> {
        if !self.torque_off_stopped() {
            let _ = self.safety.set_torque(false)?;
        }
        self.reset_gripper_stall();
        self.gripper_hold_vel_ceiling = None;
        if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        Ok(())
    }

    /// Zero-allocation RT torque-off finish after accepted disable (preserves stopped Fault/EStop).
    fn torque_off_finish_rt(&mut self) -> Result<(), &'static str> {
        if !self.torque_off_stopped() {
            self.safety.set_torque_rt(false)?;
        }
        self.reset_gripper_stall();
        self.gripper_hold_vel_ceiling = None;
        if self.driver.is_none() {
            self.measured_velocities.fill(0.0);
        }
        Ok(())
    }

    /// Re-checks live authority for an in-flight `TorqueTransition::Enable` BEFORE any retry TX.
    pub(crate) fn enable_authority_revoked(
        &self,
        auth: EnableAuthority,
        now: Instant,
    ) -> Option<&'static str> {
        if self.shutdown_committed
            || self.estop_pending()
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            return Some("ESTOP_ACTIVE");
        }
        if self.safety.state().is_stopped() {
            return Some("FAULT_ACTIVE");
        }
        if auth.admitted_epoch != 0 && self.safety.epoch_id() != auth.admitted_epoch {
            return Some("EPOCH_MISMATCH");
        }
        if auth.owner_conn_id != 0
            && self.check_client_connection_live(auth.owner_conn_id).is_err()
        {
            return Some("DISCONNECTED");
        }
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return Some("LEASE_LOST");
        }
        let fenced_gen = self.effective_fenced_lease_generation();
        if auth.lease_generation != 0 {
            if auth.lease_generation <= fenced_gen {
                return Some("LEASE_LOST");
            }
            let owner_matches = auth.owner_conn_id == 0
                || self.active_lease_conn_id == auth.owner_conn_id;
            let lease_matches = owner_matches
                && self.active_lease.as_ref().is_some_and(|a| {
                    a.generation == auth.lease_generation
                        && a.generation > fenced_gen
                        && (self.active_lease_conn_id == 0
                            || !self.is_connection_closed(self.active_lease_conn_id))
                        && now.saturating_duration_since(a.acquired_at) < self.lease_timeout
                });
            if !lease_matches {
                return Some("LEASE_LOST");
            }
        }
        None
    }

    /// Aborts any in-flight `TorqueTransition::Enable` owned by `displaced_gen` (called by
    /// `fence_displaced_lease_generation` on release, expiry, preemption, or disconnect),
    /// transitioning through [`Self::enter_failed_hold`] (no rollback disable frames at any pose).
    pub(crate) fn abort_enable_transition_for_fenced_lease_impl(&mut self, displaced_gen: u64) {
        let matches = matches!(
            self.torque_transition,
            Some(TorqueTransition::Enable(reducer))
                if displaced_gen != 0
                    && reducer
                        .authority
                        .is_some_and(|a| a.lease_generation == displaced_gen)
        );
        if !matches {
            return;
        }
        let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
            return;
        };
        let closed = reducer.authority.is_some_and(|a| {
            a.owner_conn_id != 0 && self.is_connection_closed(a.owner_conn_id)
        });
        let code = if closed { "DISCONNECTED" } else { "LEASE_LOST" };
        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
            buf.push_str(if closed {
                "interrupted by disconnect"
            } else {
                "interrupted by lease loss"
            });
        });
        match reducer.origin {
            EnableOrigin::BrokerTransition => self.core.begin_transition(Err(code)),
            EnableOrigin::ParkRearm { action_id, .. } => self.finish_park_rearm_failed(action_id, code),
            EnableOrigin::LegacyWs { token, slot, .. } => {
                self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::LeaseLost);
            }
            EnableOrigin::DetachedCancel => {}
        }
    }

    /// A transition is in flight ([`TorqueTransition`]): ordinary output and blocking torque
    /// changes wait for it.
    pub fn torque_transition_pending(&self) -> bool { self.torque_transition.is_some() }

    pub(crate) fn park_disable_in_flight(&self) -> bool {
        matches!(self.torque_transition, Some(TorqueTransition::Disable { origin: DisableOrigin::Park { .. }, .. }))
    }

    pub(crate) fn park_disable_in_flight_for(&self, id: u64) -> bool {
        matches!(self.torque_transition, Some(TorqueTransition::Disable { origin: DisableOrigin::Park { action_id: Some(aid), .. }, .. }) if aid == id)
    }

    pub(crate) fn is_watchdog_park_disable_in_flight(&self) -> bool {
        matches!(self.torque_transition, Some(TorqueTransition::Disable { origin: DisableOrigin::Park { watchdog: true, .. }, .. }))
    }

    pub(crate) fn has_non_broker_torque_transition(&self) -> bool {
        matches!(
            self.torque_transition,
            Some(
                TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::ParkRearm { .. }
                        | EnableOrigin::LegacyWs { .. }
                        | EnableOrigin::DetachedCancel,
                    ..
                })
                    | TorqueTransition::Disable {
                        origin: DisableOrigin::Park { .. }
                            | DisableOrigin::LegacyWs { .. }
                            | DisableOrigin::Detached { .. },
                        ..
                    }
            )
        )
    }

    pub(crate) fn has_detached_torque_transition(&self) -> bool {
        matches!(
            self.torque_transition,
            Some(
                TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::DetachedCancel,
                    ..
                })
                    | TorqueTransition::Disable {
                        origin: DisableOrigin::Detached { .. },
                        ..
                    }
            )
        )
    }

    pub(crate) fn has_legacy_torque_transition_token(&self, token: u64) -> bool {
        matches!(
            self.torque_transition,
            Some(
                TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::LegacyWs { token: t, .. },
                    ..
                })
                    | TorqueTransition::Disable {
                        origin: DisableOrigin::LegacyWs { token: t, .. },
                        ..
                    }
            ) if t == token
        )
    }

    #[allow(dead_code)]
    fn driver_any_enabled(&self) -> bool {
        self.driver.as_ref().is_some_and(|d| {
            d.with_ref(|drv| {
                drv.enable_state() != EnableState::Disabled
                    || drv.has_any_joint_enabled()
                    || drv.has_known_partial_enable()
            })
            .unwrap_or(false)
        })
    }

    /// Starts a torque-enable transition without waiting for its confirmation (the RT's
    /// `commit_transition`). Requires a valid [`AdmissionStamp`] checked first under the owner lock
    /// and drives the single authoritative budgeted enable reducer (`Enable -> Validate -> Armed | FailedHold`)
    /// carrying live [`EnableAuthority`] checks (lease generation, owner conn, epoch) into `FailedHold`.
    pub(crate) fn begin_enable_torque_transition(
        admitted: Admitted<'_>,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        let req_id = admitted.stamp().request_id;
        Self::begin_enable_torque_transition_with_seq(admitted, req_id)
    }

    pub(crate) fn begin_enable_torque_transition_with_seq(
        admitted: Admitted<'_>,
        client_seq: u64,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| torque_admission_error_code(&e))?;
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: client_seq,
            stop_gen: s.stop_gen,
        };
        this.begin_enable_torque_transition_inner(
            EnableOrigin::BrokerTransition,
            authority,
            false,
            false,
        )
    }

    /// Driverless immediate torque-enable on RT (zero-alloc, consumes A1's [`Admitted`]).
    pub(crate) fn enable_torque_immediate_rt(
        admitted: Admitted<'_>,
        now: Instant,
    ) -> Result<(), &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| torque_admission_error_code(&e))?;
        if this.driver.is_some() {
            return Err("UNSUPPORTED");
        }
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: s.request_id,
            stop_gen: s.stop_gen,
        };
        let (was_torqued, prev_gripper_cmd, mit_qualified) = this.enable_prepare_rt(Some(authority))?;
        let _ = now;
        match this.start_enable_reducer(
            EnableOrigin::BrokerTransition,
            was_torqued,
            prev_gripper_cmd,
            Some(authority),
            true,
            mit_qualified,
        ) {
            EnableStepOutcome::Armed(_) => Ok(()),
            EnableStepOutcome::Failed { rt_code, .. } => Err(rt_code),
            EnableStepOutcome::Pending { .. } => Err("INTERNAL"),
        }
    }

    /// Driverless immediate torque-disable on RT (zero-alloc, exempt from `Admitted`).
    pub(crate) fn disable_torque_immediate_rt(&mut self, now: Instant) -> Result<(), &'static str> {
        if self.driver.is_some() {
            return Err("UNSUPPORTED");
        }
        self.torque_off_check_rt(false)?;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        self.disable_requested_at(now);
        #[cfg(not(target_arch = "wasm32"))]
        {
            let live_op_id = self.torque_shadow.alloc_live_op_id();
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
        }
        self.disable_accepted();
        self.torque_off_finish_rt()
    }

    fn begin_enable_torque_transition_inner(
        &mut self,
        origin: EnableOrigin,
        authority: EnableAuthority,
        allow_sync_mit: bool,
        allow_legacy_fallback: bool,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        let use_rt_path = self.rt_executor_owned || matches!(origin, EnableOrigin::BrokerTransition);
        let prep_res: Result<(bool, Option<f32>, bool), &'static str> = if use_rt_path {
            self.enable_prepare_rt(Some(authority))
        } else {
            (|| {
                if self.torque_transition.is_some() {
                    return Err("TRANSITION_BUSY");
                }
                self.gravity_comp_servo_busy().map_err(|_| "MOTION_BUSY")?;
                let (w, g) = self.enable_prepare_inner(false, allow_sync_mit, false)
                    .map_err(|e| torque_error_code(&e))?;
                let mit_q = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
                Ok((w, g, mit_q))
            })()
        };
        let (was_torqued, prev_gripper_cmd, mit_qualified) = match prep_res {
            Ok(v) => v,
            Err(code) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let live_op_id = self.torque_shadow.alloc_live_op_id();
                    let live_refusal = match code {
                        "ESTOP_ACTIVE" => crate::torque::shadow::LiveRefusalKind::EStopActive,
                        "FAULT_ACTIVE" => crate::torque::shadow::LiveRefusalKind::FaultLatched,
                        "DISABLE_PENDING" => crate::torque::shadow::LiveRefusalKind::DisableInProgress,
                        "MOTION_BUSY" | "TRANSITION_BUSY" | "HANDOVER_SETTLING" => {
                            crate::torque::shadow::LiveRefusalKind::BusyOrTransition
                        }
                        "UNSUPPORTED" => crate::torque::shadow::LiveRefusalKind::Unsupported,
                        _ => crate::torque::shadow::LiveRefusalKind::EnableFailed,
                    };
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::EnableStart {
                            live_op_id,
                            live_refusal: Some(live_refusal),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        });
                }
                return Err(code);
            }
        };
        let budgeted = !allow_legacy_fallback
            || self.rt_executor_owned
            || self.driver.is_none()
            || self.driver_supports_rt_executor_nonblocking();
        Ok(
            match self.start_enable_reducer(
                origin,
                was_torqued,
                prev_gripper_cmd,
                Some(authority),
                budgeted,
                mit_qualified,
            ) {
                EnableStepOutcome::Armed(_) => Ok(Rt::Done),
                EnableStepOutcome::Pending { wake } => Ok(Rt::Pending { wake }),
                EnableStepOutcome::Failed { rt_code, .. } => Err(rt_code),
            },
        )
    }

    /// Starts a bounded, confirmed `TorqueTransition::Enable` inside park `action_id` when
    /// parking from `holding_after_clear`. Requires the already-consumed [`AdmittedProof`].
    /// Zero heap allocations on RT failure paths.
    pub(crate) fn begin_park_rearm_transition(
        &mut self,
        proof: &crate::supervisor::AdmittedProof,
        action_id: u64,
        stop_gen: u64,
    ) -> Result<ParkRearmStart, crate::rt_core::requests::motion::MotionInstallError> {
        use crate::rt_core::requests::motion::MotionInstallError;
        if self.urgent_stop_unapplied() || self.current_stop_gen() != stop_gen {
            return Err(MotionInstallError::Stopped);
        }
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: s.request_id,
            stop_gen: s.stop_gen,
        };
        let map_code = |c: &'static str| match c {
            "ESTOP_ACTIVE" => MotionInstallError::EStopActive,
            "FAULT_ACTIVE" => MotionInstallError::FaultActive,
            "HANDOVER_SETTLING" => MotionInstallError::HandoverSettling,
            "FEEDBACK_UNAVAILABLE" | "FEEDBACK_STALE" => MotionInstallError::FeedbackStale,
            "CONFIG_BUSY" => MotionInstallError::ConfigBusy,
            "MOTION_BUSY" | "TRANSITION_BUSY" | "DISABLE_PENDING" => MotionInstallError::MotionBusy,
            _ => MotionInstallError::EnableFailed,
        };
        match self
            .begin_enable_torque_transition_inner(
                EnableOrigin::ParkRearm { action_id, stop_gen },
                authority,
                !self.rt_executor_owned,
                true,
            )
            .map_err(map_code)?
        {
            Ok(crate::rt_core::ops::TorqueProgress::Done) => Ok(ParkRearmStart::ConfirmedNow),
            Ok(crate::rt_core::ops::TorqueProgress::Pending { .. }) => Ok(ParkRearmStart::Pending),
            Err(_) => Err(MotionInstallError::EnableFailed),
        }
    }

    /// Aborts a pending `ParkRearm` enable transition safely (non-blocking `try_lock`, preserving
    /// holding torque and retaining `DetachedCancel` on driver mutex contention).
    pub(crate) fn abort_park_rearm_transition(&mut self) {
        let Some(TorqueTransition::Enable(EnableReducer {
            origin: EnableOrigin::ParkRearm { .. },
            was_torqued,
            ..
        })) = self.torque_transition
        else {
            return;
        };
        self.torque_transition = None;
        let (drv_any_enabled, _) = self.abort_enable_driver_nonblocking(was_torqued, true);
        self.is_torqued = was_torqued || drv_any_enabled;
        self.pending_rearm = true;
        #[cfg(not(target_arch = "wasm32"))]
        if !was_torqued && !drv_any_enabled {
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::EnableCancelledPreTx {
                    live_op_id: self.shadow_enable_op_id,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        } else {
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::HandshakeResult {
                    live_op_id: self.shadow_enable_op_id,
                    confirmed_mask: crate::torque::ChannelMask::EMPTY,
                    outcome: crate::torque::HandshakeOutcome::Failed,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        }
    }

    /// Starts a torque-disable transition without waiting for its confirmation.
    ///
    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check (still
    /// governed by resting-pose and torque-off safety checks, zero-alloc on RT).
    pub(crate) fn begin_disable_torque_transition(
        &mut self,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        if let Err(code) = self.torque_off_check_rt_transition() {
            #[cfg(not(target_arch = "wasm32"))]
            {
                let live_op_id = self.torque_shadow.alloc_live_op_id();
                let live_refusal = match code {
                    "TORQUE_OFF_WOULD_DROP" | "ARM_NOT_AT_RESTING_POSE" => {
                        crate::torque::shadow::LiveRefusalKind::TorqueOffWouldDrop
                    }
                    "DRIVER_BUSY" => crate::torque::shadow::LiveRefusalKind::DriverBusy,
                    "MOTION_BUSY" | "TRANSITION_BUSY" => {
                        crate::torque::shadow::LiveRefusalKind::BusyOrTransition
                    }
                    _ => crate::torque::shadow::LiveRefusalKind::Other,
                };
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: Some(live_refusal),
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    });
            }
            return Err(code);
        }
        #[cfg(not(target_arch = "wasm32"))]
        let live_op_id = self.torque_shadow.alloc_live_op_id();
        let started = if self.driver.is_none() {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                    live_op_id,
                    explicit_release_authorized: false,
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            self.background_disable_retry = None;
            self.disable_requested();
            None
        } else {
            let dof = self.config.dof;
            let feedback_numeric_valid = self.feedback_numeric_valid;
            let active_lease_gen = (self.lease_enabled && !self.safety.state().is_stopped())
                .then(|| self.current_active_lease_gen())
                .filter(|&g| g > 0);
            let mut ages_buf = std::mem::take(&mut self.scratch.ages);
            let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
            let locked = {
                #[cfg(not(target_arch = "wasm32"))]
                let live_intent = self.shadow_live_intent();
                #[cfg(not(target_arch = "wasm32"))]
                let live_is_torqued = self.is_torqued;
                #[cfg(not(target_arch = "wasm32"))]
                let live_stopped = self.safety.state().is_stopped();
                #[cfg(not(target_arch = "wasm32"))]
                let shadow_rt = &mut self.torque_shadow;
                let d = self.driver.as_mut().expect("checked is_some");
                match d.try_with_mut(|drv| {
                    let requested_at = drv.clock_now();
                    // Enqueue accepted DisableRequest at `requested_at` BEFORE draining the
                    // driver's DisableRequested WriteResult so causal order is exact AND a
                    // failed try_lock never records a spurious accepted request!
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.enqueue_at(
                        requested_at,
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: None,
                            live_intent,
                            live_is_torqued,
                            live_stopped,
                        },
                    );
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                    };
                    drv.tx_mut().set_disable_authority(None, active_lease_gen);
                    let res = drv.begin_disable_torque_budgeted(disable_kind, 0);
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    let post_now = drv.clock_now();
                    let snap = OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        &self.measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    );
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    (res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap)
                }) {
                    Ok(v) => Ok(v),
                    Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                    Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
                }
            };
            self.scratch.ages = ages_buf;
            self.scratch.fb.bounds = bounds_buf;
            match locked {
                Ok((res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap)) => {
                    self.reset_disable_lock_contention();
                    self.owner_driver_snapshot = snap;
                    self.stop_motion();
                    self.commanded_velocities.fill(0.0);
                    self.background_disable_retry = None;
                    self.disable_requested_at(requested_at);
                    Some((res, post_now, disable_kind, frames_dispatched, dispatched_mask))
                }
                Err(true) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(crate::torque::shadow::LiveRefusalKind::DriverBusy),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        });
                    self.record_disable_lock_contention();
                    return Err("DRIVER_BUSY");
                }
                Err(false) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    self.torque_shadow
                        .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(crate::torque::shadow::LiveRefusalKind::Other),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        });
                    return Err("FEEDBACK_UNAVAILABLE");
                }
            }
        };
        if let Some((_, _, disable_kind, _, _)) = started {
            self.active_disable_kind = Some(disable_kind);
            self.active_disable_client_park = None;
        }
        Ok(match started {
            None => {
                self.disable_accepted();
                self.torque_off_finish_rt().map(|()| Rt::Done)
            }
            Some((Ok(crate::driver::TorqueProgress::Pending { .. }), post_now, disable_kind, frames_dispatched, dispatched_mask)) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::BrokerTransition,
                    kind: disable_kind,
                });
                Ok(Rt::Pending { wake: post_now })
            }
            Some((Ok(crate::driver::TorqueProgress::Done(_)), _, _, _, _)) => {
                self.disable_accepted();
                self.torque_off_finish_rt().map(|()| Rt::Done)
            }
            Some((Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }), _, _, _, _)) => {
                let (_, code) = self.handle_partial_disable_abort(e);
                Err(code)
            }
            Some((Err(crate::driver::ArmDriverError::LeaseFenced), _, _, _, _)) => {
                self.clear_active_disable_retry_state();
                Err("LEASE_LOST")
            }
            Some((Err(e), post_now, disable_kind, frames_dispatched, dispatched_mask)) => {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                let partial_retry = self.commit_partial_disable_retry_kind(
                    disable_kind,
                    post_now,
                    frames_dispatched,
                    partial_remaining,
                );
                if let Some(rk) = partial_retry {
                    let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                } else if !refused {
                    let _ = self.arm_background_disable_retry_with_kind(disable_kind, post_now);
                }
                Err(self.disable_step_failed_inner(e, partial_retry.is_some()))
            }
        })
    }

    fn reserve_legacy_torque_slot(
        &mut self,
    ) -> Option<(u64, u8, std::sync::Arc<LegacyTorqueSlot>)> {
        let token = self.legacy_torque_seq.wrapping_add(1).max(1);
        self.legacy_torque_seq = token;
        for (idx, slot) in self.legacy_torque_slots.iter().enumerate() {
            if slot.try_reserve(token) {
                return Some((token, idx as u8, std::sync::Arc::clone(slot)));
            }
        }
        None
    }

    fn settle_legacy_ws_accepted(&mut self, token: u64, slot: u8) {
        let epoch_id = self.safety.epoch_id();
        let safety_state = self.safety.state_arc();
        let is_torqued = self.is_torqued;
        let control_mode_gravity_comp = self.gravity_comp_active();
        if let Some(s) = self.legacy_torque_slots.get(usize::from(slot)) {
            if s.settle_with(token, || LegacyTorqueOutcome::Accepted {
                epoch_id,
                safety_state,
                is_torqued,
                control_mode_gravity_comp,
            }) {
                self.legacy_torque_settled_this_tick = true;
            }
        }
    }

    pub(crate) fn settle_legacy_ws_failed(&mut self, token: u64, slot: u8, failure: LegacyTorqueFailure) {
        let epoch_id = self.safety.epoch_id();
        let safety_state = self.safety.state_arc();
        if let Some(s) = self.legacy_torque_slots.get(usize::from(slot)) {
            if s.settle_with(token, || LegacyTorqueOutcome::Failed {
                failure,
                epoch_id,
                safety_state,
            }) {
                self.legacy_torque_settled_this_tick = true;
            }
        }
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

    /// Cancels an in-flight `EnableOrigin::LegacyWs` on client Stop without latching a
    /// non-recoverable `EnableFailed` fault (unless E-stop is also pending/latched, in which case
    /// E-stop takes precedence). Returns `true` when E-stop won.
    fn cancel_legacy_enable_on_stop(
        &mut self,
        token: u64,
        slot: u8,
        was_torqued: bool,
        prev_torque_intent: TorqueIntent,
    ) -> bool {
        if self.estop_pending()
            || self.e_stop_latched.load(Ordering::SeqCst)
            || matches!(self.safety.state(), SafetyState::EStop)
        {
            let _ = self.enter_failed_hold(was_torqued, |buf| {
                buf.push_str("interrupted by E-stop");
            });
            self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::EStopActive);
            return true;
        }
        let (drv_any_enabled, contended) = self.abort_enable_driver_nonblocking(was_torqued, true);
        if contended && !was_torqued {
            self.detached_cancel_restore_intent = Some(prev_torque_intent);
        } else {
            self.detached_cancel_restore_intent = None;
        }
        let any_energized = was_torqued || drv_any_enabled;
        if !any_energized {
            self.enable_lifecycle = EnableLifecycle::Disarmed;
            self.is_torqued = false;
            self.torque_intent = prev_torque_intent;
            self.pending_rearm = false;
            self.measured_hold.clear();
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::EnableCancelledPreTx {
                    live_op_id: self.shadow_enable_op_id,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        } else {
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
            // Use FailedHold (without latching EnableFailed) so tick_control always holds
            // partially enabled / interrupted-rearm motors via service_stopped_measured_hold
            // rather than ordinary dynamic output, while allowing immediate operator recovery.
            self.enable_lifecycle = EnableLifecycle::FailedHold;
            self.is_torqued = true;
            self.torque_intent = TorqueIntent::On;
            self.pending_rearm = true;
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow.enqueue(
                crate::torque::shadow::ShadowInputKind::HandshakeResult {
                    live_op_id: self.shadow_enable_op_id,
                    confirmed_mask: crate::torque::ChannelMask::EMPTY,
                    outcome: crate::torque::HandshakeOutcome::Failed,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                },
            );
        }
        self.settle_legacy_ws_failed(token, slot, LegacyTorqueFailure::Stopped);
        false
    }

    /// Immediately aborts an in-flight `EnableOrigin::LegacyWs` when `authorized_stop` runs.
    pub(crate) fn abort_legacy_enable_on_authorized_stop(&mut self) {
        let is_legacy_enable = matches!(
            self.torque_transition,
            Some(TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::LegacyWs { .. },
                ..
            }))
        );
        if !is_legacy_enable {
            return;
        }
        let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
            return;
        };
        if let EnableOrigin::LegacyWs {
            token,
            slot,
            prev_torque_intent,
            ..
        } = reducer.origin
        {
            let _ = self.cancel_legacy_enable_on_stop(
                token,
                slot,
                reducer.was_torqued,
                prev_torque_intent,
            );
        }
    }

    /// Starts a bounded, ticked legacy WS torque-enable transition under a short write guard
    /// without blocking or sleeping on CAN/serial confirmation (consumes A1's [`Admitted`] by value).
    pub(crate) fn begin_enable_torque_transition_legacy(
        admitted: Admitted<'_>,
        deadline: Instant,
    ) -> Result<LegacyTorqueStart, SafetyError> {
        let (this, proof) = admitted.consume()?;
        if this.calibration_busy && this.motor_zero_busy {
            return Err(SafetyError::MotionBusy);
        }
        if this.torque_transition.is_some() {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        }
        this.gravity_comp_servo_busy()?;
        let (was_torqued, prev_gripper_cmd) = this.enable_prepare_inner(false, true, true)?;
        let mit_qualified = this.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
        let Some((token, slot_idx, slot)) = this.reserve_legacy_torque_slot() else {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        };
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
            admission_request_id: s.request_id,
            stop_gen: s.stop_gen,
        };
        let prev_torque_intent = this.torque_intent;
        let budgeted = this.driver.is_none() || this.driver_supports_rt_executor_nonblocking();
        let origin = EnableOrigin::LegacyWs {
            token,
            slot: slot_idx,
            stop_gen: s.stop_gen,
            prev_torque_intent,
            deadline,
        };
        match this.start_enable_reducer_inner(
            origin,
            was_torqued,
            prev_gripper_cmd,
            Some(authority),
            budgeted,
            budgeted,
            mit_qualified,
        ) {
            EnableStepOutcome::Armed(state) => {
                slot.release_unstarted(token);
                Ok(LegacyTorqueStart::ImmediateAccepted {
                    epoch_id: this.safety.epoch_id(),
                    safety_state: state,
                    is_torqued: this.is_torqued,
                    control_mode_gravity_comp: this.gravity_comp_active(),
                })
            }
            EnableStepOutcome::Pending { .. } => Ok(LegacyTorqueStart::Pending {
                token,
                slot_idx,
                slot,
            }),
            EnableStepOutcome::Failed {
                rt_code,
                message,
                feedback_unavailable,
                estop_interrupted,
            } => {
                slot.release_unstarted(token);
                Ok(LegacyTorqueStart::ImmediateRejected {
                    failure: Self::classify_legacy_enable_failure(
                        rt_code,
                        message,
                        feedback_unavailable,
                        estop_interrupted,
                    ),
                    epoch_id: this.safety.epoch_id(),
                    safety_state: this.safety.state().clone(),
                })
            }
        }
    }

    /// Non-blocking torque-off preflight for legacy WS admission (never blocks on `driver.lock()`).
    fn torque_off_check_legacy_nonblocking(&mut self) -> Result<Result<(), SafetyError>, ()> {
        if !self.torque_off_stopped() {
            if let Some(reason) = self.resting_pose_violation() {
                tracing::warn!("Torque off refused, not at resting pose: {reason}");
                return Ok(Err(SafetyError::ArmNotAtRestingPose {
                    current: self.measured_positions.clone(),
                    expected: self.config.resting_positions().to_vec(),
                    reason: Some(reason),
                }));
            }
        }
        let stopped_res = match self.may_disable_torque_while_stopped_nonblocking() {
            Ok(r) => r,
            Err(()) => return Err(()),
        };
        match stopped_res {
            Ok(()) => {
                self.last_stopped_torque_off_refusal = None;
                Ok(Ok(()))
            }
            Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback) => {
                match self.can_defer_stopped_disable_to_driver_nonblocking() {
                    Ok(true) => {
                        self.last_stopped_torque_off_refusal = None;
                        Ok(Ok(()))
                    }
                    Ok(false) => {
                        let r = crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback;
                        self.last_stopped_torque_off_refusal = Some(r);
                        Ok(Err(r.into_safety_error(&self.config.joint_names)))
                    }
                    Err(()) => Err(()),
                }
            }
            Err(r) => {
                self.last_stopped_torque_off_refusal = Some(r);
                Ok(Err(r.into_safety_error(&self.config.joint_names)))
            }
        }
    }

    /// Starts a bounded, ticked legacy WS torque-disable transition under a short write guard
    /// without blocking on rest-gate probes or disable repeat sleeps. Exempt from `Admitted`.
    pub(crate) fn begin_disable_torque_transition_legacy(
        &mut self,
        deadline: Instant,
    ) -> Result<LegacyTorqueStart, SafetyError> {
        if Instant::now() >= deadline {
            return Ok(LegacyTorqueStart::ImmediateRejected {
                failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state().clone(),
            });
        }
        if self.calibration_busy && self.motor_zero_busy {
            return Err(SafetyError::MotionBusy);
        }
        if self.torque_transition.is_some()
            || matches!(
                self.active_disable_kind,
                Some(crate::driver::DisableKind::DisableProtective { .. })
            )
            || matches!(
                self.background_disable_retry,
                Some(BackgroundDisableRetry::Active { .. })
            )
        {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        }
        self.gravity_comp_servo_busy()?;
        match self.torque_off_check_legacy_nonblocking() {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let live_op_id = self.torque_shadow.alloc_live_op_id();
                    let live_refusal = match &e {
                        SafetyError::TorqueOffWouldDrop { .. }
                        | SafetyError::ArmNotAtRestingPose { .. } => {
                            crate::torque::shadow::LiveRefusalKind::TorqueOffWouldDrop
                        }
                        SafetyError::MotionBusy | SafetyError::ClearFaultRejected(_) => {
                            crate::torque::shadow::LiveRefusalKind::BusyOrTransition
                        }
                        _ => crate::torque::shadow::LiveRefusalKind::Other,
                    };
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(live_refusal),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        },
                    );
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                }
                return Err(e);
            }
            Err(()) => {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    let live_op_id = self.torque_shadow.alloc_live_op_id();
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: Some(crate::torque::shadow::LiveRefusalKind::DriverBusy),
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: self.safety.state().is_stopped(),
                        },
                    );
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                }
                self.record_disable_lock_contention();
                return Ok(LegacyTorqueStart::ImmediateRejected {
                    failure: LegacyTorqueFailure::StaticCode("DRIVER_BUSY"),
                    epoch_id: self.safety.epoch_id(),
                    safety_state: self.safety.state().clone(),
                });
            }
        }

        #[cfg(not(target_arch = "wasm32"))]
        let live_op_id = self.torque_shadow.alloc_live_op_id();
        if self.driver.is_none() {
            #[cfg(not(target_arch = "wasm32"))]
            {
                self.torque_shadow
                    .enqueue(crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: None,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    });
                if !self.rt_executor_owned {
                    self.flush_torque_shadow();
                }
            }
            self.stop_motion();
            self.commanded_velocities.fill(0.0);
            self.background_disable_retry = None;
            self.disable_requested();
            self.disable_accepted();
            self.torque_off_finish_in_place()?;
            return Ok(LegacyTorqueStart::ImmediateAccepted {
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state().clone(),
                is_torqued: self.is_torqued,
                control_mode_gravity_comp: self.gravity_comp_active(),
            });
        }

        let Some((token, slot_idx, slot)) = self.reserve_legacy_torque_slot() else {
            return Err(SafetyError::ClearFaultRejected(
                "A torque transition is pending".into(),
            ));
        };
        let now_wall = Instant::now();
        if now_wall >= deadline {
            slot.release_unstarted(token);
            return Ok(LegacyTorqueStart::ImmediateRejected {
                failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state().clone(),
            });
        }

        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let period = Duration::from_secs_f64(
            1.0 / f64::from(self.config.control_frequency_hz.max(1)),
        );
        let per_tick_tx_budget = period.saturating_sub(self.tx_release_publish_margin());
        let active_lease_gen = (self.lease_enabled && !self.safety.state().is_stopped())
            .then(|| self.current_active_lease_gen())
            .filter(|&g| g > 0);
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let locked = {
            #[cfg(not(target_arch = "wasm32"))]
            let live_intent = self.shadow_live_intent();
            #[cfg(not(target_arch = "wasm32"))]
            let live_is_torqued = self.is_torqued;
            #[cfg(not(target_arch = "wasm32"))]
            let live_stopped = self.safety.state().is_stopped();
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            match d.try_with_mut(|drv| {
                if drv.disable_frames_dispatched() || drv.torque_transition_pending() {
                    Err(2u8)
                } else if Instant::now() >= deadline {
                    Err(3u8)
                } else {
                    let requested_at = drv.clock_now();
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.enqueue_at(
                        requested_at,
                        crate::torque::shadow::ShadowInputKind::DisableRequest {
                            live_op_id,
                            explicit_release_authorized: false,
                            live_refusal: None,
                            live_intent,
                            live_is_torqued,
                            live_stopped,
                        },
                    );
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                    };
                    let req_rem = deadline.saturating_duration_since(Instant::now());
                    let step_budget = req_rem.min(per_tick_tx_budget);
                    let use_budgeted = drv.supports_rt_executor().is_ok();
                    drv.tx_mut().set_disable_authority(Some(requested_at + req_rem), active_lease_gen);
                    drv.set_tx_release_deadline(Some(requested_at + step_budget));
                    let res = if use_budgeted {
                        drv.begin_disable_torque_budgeted(disable_kind, 0)
                    } else {
                        drv.begin_disable_torque(disable_kind)
                    };
                    drv.set_tx_release_deadline(None);
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    let post_now = drv.clock_now();
                    let snap = OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    );
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    Ok((
                        res,
                        requested_at,
                        post_now,
                        disable_kind,
                        frames_dispatched,
                        dispatched_mask,
                        snap,
                    ))
                }
            }) {
                Ok(inner_res) => inner_res,
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(1u8),
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(0u8),
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;

        let (res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap) =
            match locked {
                Ok(v) => {
                    self.reset_disable_lock_contention();
                    #[cfg(not(target_arch = "wasm32"))]
                    if !self.rt_executor_owned {
                        self.flush_torque_shadow();
                    }
                    v
                }
                Err(3u8) => {
                    slot.release_unstarted(token);
                    return Ok(LegacyTorqueStart::ImmediateRejected {
                        failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                        epoch_id: self.safety.epoch_id(),
                        safety_state: self.safety.state().clone(),
                    });
                }
                Err(2u8) => {
                    slot.release_unstarted(token);
                    return Err(SafetyError::ClearFaultRejected(
                        "A torque transition is pending".into(),
                    ));
                }
                Err(1u8) => {
                    slot.release_unstarted(token);
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.torque_shadow.enqueue(
                            crate::torque::shadow::ShadowInputKind::DisableRequest {
                                live_op_id,
                                explicit_release_authorized: false,
                                live_refusal: Some(
                                    crate::torque::shadow::LiveRefusalKind::DriverBusy,
                                ),
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            },
                        );
                        if !self.rt_executor_owned {
                            self.flush_torque_shadow();
                        }
                    }
                    self.record_disable_lock_contention();
                    return Ok(LegacyTorqueStart::ImmediateRejected {
                        failure: LegacyTorqueFailure::StaticCode("DRIVER_BUSY"),
                        epoch_id: self.safety.epoch_id(),
                        safety_state: self.safety.state().clone(),
                    });
                }
                Err(_) => {
                    slot.release_unstarted(token);
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        self.torque_shadow.enqueue(
                            crate::torque::shadow::ShadowInputKind::DisableRequest {
                                live_op_id,
                                explicit_release_authorized: false,
                                live_refusal: Some(crate::torque::shadow::LiveRefusalKind::Other),
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            },
                        );
                        if !self.rt_executor_owned {
                            self.flush_torque_shadow();
                        }
                    }
                    return Err(SafetyError::FeedbackUnavailable);
                }
            };

        self.owner_driver_snapshot = snap;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        self.background_disable_retry = None;
        self.disable_requested_at(requested_at);
        self.active_disable_kind = Some(disable_kind);
        self.active_disable_client_park = None;

        match res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                slot.release_unstarted(token);
                self.disable_accepted();
                self.torque_off_finish_in_place()?;
                Ok(LegacyTorqueStart::ImmediateAccepted {
                    epoch_id: self.safety.epoch_id(),
                    safety_state: self.safety.state().clone(),
                    is_torqued: self.is_torqued,
                    control_mode_gravity_comp: self.gravity_comp_active(),
                })
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::LegacyWs {
                        token,
                        slot: slot_idx,
                        deadline,
                    },
                    kind: disable_kind,
                });
                Ok(LegacyTorqueStart::Pending {
                    token,
                    slot_idx,
                    slot,
                })
            }
            Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }) => {
                // Never return ImmediateRejected once any disable frame was dispatched:
                // settle the slot with a single terminal fault detailing applied/uncertain/not attempted.
                let (err_detail, _) = self.handle_partial_disable_abort(e);
                self.settle_legacy_ws_failed(
                    token,
                    slot_idx,
                    LegacyTorqueFailure::DisableFailed { detail: err_detail },
                );
                Ok(LegacyTorqueStart::Pending {
                    token,
                    slot_idx,
                    slot,
                })
            }
            Err(e) => {
                let mut err_detail =
                    crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
                err_detail.push_str("Torque disable transmission failed: ");
                e.write_bounded(&mut err_detail);
                if frames_dispatched {
                    let refused = matches!(
                        e,
                        crate::driver::ArmDriverError::TorqueOffWouldDrop
                            | crate::driver::ArmDriverError::DisableRequestExpired
                    );
                    let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                    let partial_retry = self.commit_partial_disable_retry_kind(
                        disable_kind,
                        post_now,
                        frames_dispatched,
                        partial_remaining,
                    );
                    if let Some(rk) = partial_retry {
                        let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                    } else if !refused {
                        let _ = self.arm_background_disable_retry_with_kind(disable_kind, post_now);
                    }
                    let _ = self.disable_step_failed_inner(e, partial_retry.is_some());
                    self.settle_legacy_ws_failed(
                        token,
                        slot_idx,
                        LegacyTorqueFailure::DisableFailed { detail: err_detail },
                    );
                    return Ok(LegacyTorqueStart::Pending {
                        token,
                        slot_idx,
                        slot,
                    });
                }
                slot.release_unstarted(token);
                self.clear_active_disable_retry_state();
                let code = self.disable_step_failed_inner(e, false);
                let failure = match code {
                    "TORQUE_OFF_WOULD_DROP" => {
                        let refusal = self
                            .may_disable_torque_while_stopped_nonblocking()
                            .ok()
                            .and_then(Result::err)
                            .or(self.last_stopped_torque_off_refusal)
                            .unwrap_or(
                                crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                            );
                        self.last_stopped_torque_off_refusal = Some(refusal);
                        LegacyTorqueFailure::TorqueOffWouldDrop(refusal)
                    }
                    "ESTOP_ACTIVE" => LegacyTorqueFailure::EStopActive,
                    "LEASE_LOST" => LegacyTorqueFailure::LeaseLost,
                    "FEEDBACK_UNAVAILABLE" => LegacyTorqueFailure::FeedbackUnavailable,
                    _ => LegacyTorqueFailure::DisableFailed { detail: err_detail },
                };
                Ok(LegacyTorqueStart::ImmediateRejected {
                    failure,
                    epoch_id: self.safety.epoch_id(),
                    safety_state: self.safety.state().clone(),
                })
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
    /// Single locked decision for an expired non-protective disable (`LegacyWs` or `Detached`):
    /// - Protective disable -> never aborted (`KeepDetachedDispatched`).
    /// - Missing driver or poisoned lock -> clears transition + retry state (`Aborted`).
    /// - Contended `try_with_mut` -> keeps `Detached { deadline: Some(dl) }` pending without clearing
    ///   retry state (`ContendedRetry`).
    /// - Acquired single driver borrow -> clears `deadline` ONLY when `disable_frames_dispatched()`
    ///   proves at least one disable frame was already sent; otherwise attempts
    ///   `abort_pending_disable()` and keeps `deadline` intact if abort is refused.
    fn try_abort_expired_undispatched_disable(
        &mut self,
        kind: crate::driver::DisableKind,
        deadline: Option<Instant>,
    ) -> DisableExpiryDecision {
        let is_protective =
            matches!(kind, crate::driver::DisableKind::DisableProtective { .. });
        if is_protective {
            self.torque_transition = Some(TorqueTransition::Disable {
                origin: DisableOrigin::Detached { deadline: None },
                kind,
            });
            return DisableExpiryDecision::KeepDetachedDispatched;
        }
        let Some(ref mut d) = self.driver else {
            self.clear_active_disable_retry_state();
            return DisableExpiryDecision::Aborted;
        };
        let decision = match d.try_with_mut(|drv| {
            if drv.disable_frames_dispatched() {
                DisableExpiryDecision::KeepDetachedDispatched
            } else if drv.abort_pending_disable() {
                DisableExpiryDecision::Aborted
            } else {
                // Driver refused abort without proving dispatch; keep deadline intact so no first
                // disable frame can ever transmit after request expiry.
                DisableExpiryDecision::KeepDetachedWithDeadline
            }
        }) {
            Ok(dec) => dec,
            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                DisableExpiryDecision::ContendedRetry
            }
            Err(crate::supervisor_lock::DriverTryError::Poisoned) => DisableExpiryDecision::Aborted,
        };
        match decision {
            DisableExpiryDecision::Aborted => self.clear_active_disable_retry_state(),
            DisableExpiryDecision::ContendedRetry
            | DisableExpiryDecision::KeepDetachedWithDeadline => {
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Detached { deadline },
                    kind,
                });
            }
            DisableExpiryDecision::KeepDetachedDispatched => {
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Detached { deadline: None },
                    kind,
                });
            }
        }
        self.sync_detached_torque_flag();
        decision
    }

    /// Resolves an expired legacy WS torque transition under the owner write guard.
    /// - When `settle_slot` is `false` (waiter-driven timeout): checks for an already-settled
    ///   outcome first, releases the slot, cleans up/detaches the transition, and returns the
    ///   terminal outcome directly to the waiter.
    /// - When `settle_slot` is `true` (control-tick authoritative deadline fence): cleans up/detaches
    ///   the transition and publishes the terminal outcome into the waiter slot so the waiter wakes.
    fn expire_legacy_torque_transition_inner(
        &mut self,
        token: u64,
        slot_idx: u8,
        enabled: bool,
        settle_slot: bool,
    ) -> LegacyTorqueOutcome {
        if !settle_slot {
            if let Some(settled) = self
                .legacy_torque_slots
                .get(usize::from(slot_idx))
                .and_then(|s| s.take_settled(token))
            {
                return settled;
            }
            if let Some(s) = self.legacy_torque_slots.get(usize::from(slot_idx)) {
                s.release_unstarted(token);
            }
        }
        if enabled {
            let matches_ours = matches!(
                self.torque_transition,
                Some(TorqueTransition::Enable(EnableReducer {
                    origin: EnableOrigin::LegacyWs { token: t, .. },
                    ..
                })) if t == token
            );
            if matches_ours {
                if let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() {
                    let prev_torque_intent = match reducer.origin {
                        EnableOrigin::LegacyWs { prev_torque_intent, .. } => prev_torque_intent,
                        _ => TorqueIntent::Off,
                    };
                    let (drv_any_enabled, contended) =
                        self.abort_enable_driver_nonblocking(reducer.was_torqued, false);
                    if !reducer.was_torqued && !drv_any_enabled {
                        self.enable_lifecycle = EnableLifecycle::Disarmed;
                        self.is_torqued = false;
                        self.torque_intent = prev_torque_intent;
                        self.pending_rearm = false;
                        self.measured_hold.clear();
                        #[cfg(not(target_arch = "wasm32"))]
                        self.torque_shadow.enqueue(
                            crate::torque::shadow::ShadowInputKind::EnableCancelledPreTx {
                                live_op_id: self.shadow_enable_op_id,
                                live_intent: self.shadow_live_intent(),
                                live_is_torqued: self.is_torqued,
                                live_stopped: self.safety.state().is_stopped(),
                            },
                        );
                    } else {
                        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
                            buf.push_str("enable transition timed out");
                        });
                    }
                    if contended {
                        self.retain_detached_enable_cancel_if_clear(reducer.was_torqued);
                    }
                }
            }
            let is_estop = self.estop_pending()
                || self.e_stop_latched.load(Ordering::SeqCst)
                || matches!(self.safety.state(), SafetyState::EStop);
            let failure = if is_estop {
                LegacyTorqueFailure::EStopActive
            } else {
                LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED")
            };
            if settle_slot {
                self.settle_legacy_ws_failed(token, slot_idx, failure);
            }
            LegacyTorqueOutcome::Failed {
                failure,
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state_arc(),
            }
        } else {
            let matches_ours = matches!(
                self.torque_transition,
                Some(TorqueTransition::Disable {
                    origin: DisableOrigin::LegacyWs { token: t, .. },
                    ..
                }) if t == token
            );
            let mut aborted_before_dispatch = false;
            if matches_ours {
                if let Some(TorqueTransition::Disable {
                    origin: DisableOrigin::LegacyWs { deadline, .. },
                    kind,
                }) = self.torque_transition.take()
                {
                    if matches!(
                        self.try_abort_expired_undispatched_disable(kind, Some(deadline)),
                        DisableExpiryDecision::Aborted
                    ) {
                        aborted_before_dispatch = true;
                    }
                }
            }
            let failure = if aborted_before_dispatch {
                LegacyTorqueFailure::TorqueOffWouldDrop(
                    crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                )
            } else {
                LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED")
            };
            if settle_slot {
                self.settle_legacy_ws_failed(token, slot_idx, failure);
            }
            LegacyTorqueOutcome::Failed {
                failure,
                epoch_id: self.safety.epoch_id(),
                safety_state: self.safety.state_arc(),
            }
        }
    }

    /// Resolves a waiter deadline atomically under the owner write guard against any concurrent
    /// completion. Never cancels a protective disable or an already-dispatched disable.
    pub(crate) fn timeout_legacy_torque_transition(
        &mut self,
        token: u64,
        slot_idx: u8,
        enabled: bool,
    ) -> LegacyTorqueOutcome {
        self.expire_legacy_torque_transition_inner(token, slot_idx, enabled, false)
    }

    /// Single park terminal torque-off function routed strictly through the torque-off rest-pose
    /// gate (`may_disable_torque_while_stopped_nonblocking` / `eval_stopped_torque_off` + driver
    /// `DisableRequested` K=3 rest-pose gate) and the single `TorqueTransition::Disable` authority
    /// (zero-alloc on RT). Client Stop is linearized against every motor's disable transmit at the
    /// driver boundary; watchdog park is exempt from client Stop while still rest-pose gated.
    pub(crate) fn begin_park_disable_transition(&mut self, watchdog: bool, action_id: Option<u64>) {
        if !watchdog && self.urgent_stop_unapplied() {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                    live_op_id: action_id.unwrap_or(0),
                    admitted: false,
                    settled_ok: Some(false),
                });
            self.cancel_park_before_disable(action_id);
            return;
        }
        if self.torque_transition.is_some()
            || self.safety.state().is_stopped()
            || self.e_stop_latched.load(Ordering::SeqCst)
            || self.estop_pending()
        {
            #[cfg(not(target_arch = "wasm32"))]
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                    live_op_id: action_id.unwrap_or(0),
                    admitted: false,
                    settled_ok: Some(false),
                });
            self.finish_park_disable_failed(
                watchdog,
                action_id,
                "UNLOAD_FAILED",
                "Park torque-off refused at resting pose",
            );
            return;
        }
        // Route park disable through the torque-off rest-pose gate (both RT non-blocking and legacy).
        if self.rt_executor_owned {
            match self.may_disable_torque_while_stopped_nonblocking() {
                Err(()) => {
                    self.record_disable_lock_contention();
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "DRIVER_BUSY",
                        "Driver busy during park torque-off",
                    );
                    return;
                }
                Ok(Ok(())) => {
                    self.last_stopped_torque_off_refusal = None;
                }
                Ok(Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)) => {
                    match self.can_defer_stopped_disable_to_driver_nonblocking() {
                        Err(()) => {
                            self.record_disable_lock_contention();
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "DRIVER_BUSY",
                                "Driver busy during park torque-off",
                            );
                            return;
                        }
                        Ok(true) => {
                            self.last_stopped_torque_off_refusal = None;
                        }
                        Ok(false) => {
                            self.last_stopped_torque_off_refusal = Some(
                                crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                            );
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "TORQUE_OFF_WOULD_DROP",
                                "Park torque-off refused by rest-pose gate",
                            );
                            return;
                        }
                    }
                }
                Ok(Err(r)) => {
                    self.last_stopped_torque_off_refusal = Some(r);
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "TORQUE_OFF_WOULD_DROP",
                        "Park torque-off refused by rest-pose gate",
                    );
                    return;
                }
            }
            match self.check_resting_pose_rt(true) {
                super::RtRestingCheck::AtRest => {}
                super::RtRestingCheck::OffRest => {
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "ARM_NOT_AT_RESTING_POSE",
                        "Park torque-off refused at resting pose",
                    );
                    return;
                }
                super::RtRestingCheck::LockContended => {
                    self.record_disable_lock_contention();
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "DRIVER_BUSY",
                        "Driver busy during park torque-off",
                    );
                    return;
                }
            }
        } else {
            match self.eval_stopped_torque_off() {
                Ok(()) => {}
                Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)
                    if self.can_defer_stopped_disable_to_driver() =>
                {
                    self.last_stopped_torque_off_refusal = None;
                }
                Err(_) => {
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "TORQUE_OFF_WOULD_DROP",
                        "Park torque-off refused by rest-pose gate",
                    );
                    return;
                }
            }
            if !self.is_resting_pose_static_ok() {
                self.finish_park_disable_failed(
                    watchdog,
                    action_id,
                    "ARM_NOT_AT_RESTING_POSE",
                    "Park torque-off refused at resting pose",
                );
                return;
            }
        }
        if self.driver.is_none() {
            if !watchdog && self.urgent_stop_unapplied() {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                            live_op_id: action_id.unwrap_or(0),
                            admitted: false,
                            settled_ok: Some(false),
                        },
                    );
                    self.torque_shadow.enqueue(
                        crate::torque::shadow::ShadowInputKind::Stop {
                            kind: crate::torque::StopKind::Stop,
                            live_intent: self.shadow_live_intent(),
                            live_is_torqued: self.is_torqued,
                            live_stopped: true,
                        },
                    );
                }
                self.cancel_park_before_disable(action_id);
                return;
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
                let live_op_id = action_id
                    .unwrap_or_else(|| self.torque_shadow.alloc_live_op_id());
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                        live_op_id,
                        admitted: true,
                        settled_ok: None,
                    },
                );
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::DisableRequest {
                        live_op_id,
                        explicit_release_authorized: false,
                        live_refusal: None,
                        live_intent: self.shadow_live_intent(),
                        live_is_torqued: self.is_torqued,
                        live_stopped: self.safety.state().is_stopped(),
                    },
                );
            }
            self.retire_motion_payloads();
            self.pending_deferred_done = None;
            self.end_gripper_stream();
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
            self.commanded_velocities.fill(0.0);
            self.disable_requested();
            self.complete_park_disable_accepted(watchdog, action_id);
            return;
        }
        #[allow(clippy::large_enum_variant)] // inline zero-alloc RT stack enum
        enum ParkDisableStart {
            CancelBeforeDisable,
            Contended,
            Poisoned,
            Outcome {
                res: Result<crate::driver::TorqueProgress, crate::driver::ArmDriverError>,
                requested_at: Instant,
                post_now: Instant,
                disable_kind: crate::driver::DisableKind,
                frames_dispatched: bool,
                dispatched_mask: u64,
                snap: OwnerDriverSnapshot,
            },
        }

        let stop_before_write = self.e_stop_latched.clone();
        let expected_stop_gen = self.stop_requests.as_ref().map(|s| s.applied());
        let estop_state = &self.estop_state;
        let applied_estop_count = self.applied_estop_count;
        let stop_gen_atomic = &self.stop_gen;
        let stop_requests = self.stop_requests.as_ref();
        let safety_stopped = self.safety.state().is_stopped();
        let rt_executor_owned = self.rt_executor_owned;
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let estop_pending_now = || {
            let (req, latched) = estop_state.snapshot();
            latched || req != applied_estop_count
        };
        let urgent_stop_unapplied_now = || {
            estop_state.requested_count() != applied_estop_count
                || stop_requests.is_some_and(crate::rt_core::urgent::StopRequests::pending)
        };
        let current_stop_gen_now = || {
            Self::compute_current_stop_gen(stop_gen_atomic, stop_requests)
        };
        #[cfg(not(target_arch = "wasm32"))]
        let pre_live_intent = self.shadow_live_intent();
        #[cfg(not(target_arch = "wasm32"))]
        let pre_live_torqued = self.is_torqued;
        let start = {
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            let run_start = |drv: &mut dyn crate::driver::ArmDriver| -> ParkDisableStart {
                // Linearize client Stop and hardware E-stop against EVERY motor's disable transmit
                // both inside the driver slot AND inside the driver's per-motor gate
                // (`DamiaoArmDriver::send(TxKind::ClientParkDisable)` /
                // `FeetechArmDriver::write_torque_off_checked_budgeted`).
                if !watchdog && urgent_stop_unapplied_now() {
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        shadow_rt.drain_from_driver(drv);
                        let now = drv.clock_now();
                        shadow_rt.enqueue_at(
                            now,
                            crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                                live_op_id: action_id.unwrap_or(0),
                                admitted: false,
                                settled_ok: Some(false),
                            },
                        );
                        shadow_rt.enqueue_at(
                            now,
                            crate::torque::shadow::ShadowInputKind::Stop {
                                kind: crate::torque::StopKind::Stop,
                                live_intent: pre_live_intent,
                                live_is_torqued: pre_live_torqued,
                                live_stopped: true,
                            },
                        );
                    }
                    ParkDisableStart::CancelBeforeDisable
                } else if stop_before_write.load(Ordering::SeqCst) || estop_pending_now() {
                    let now = drv.clock_now();
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    let disable_kind =
                        crate::driver::DisableKind::DisableRequested { started_at: now };
                    let snap = OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    );
                    ParkDisableStart::Outcome {
                        res: Err(crate::driver::ArmDriverError::EstopInhibited),
                        requested_at: now,
                        post_now: now,
                        disable_kind,
                        frames_dispatched: false,
                        dispatched_mask: 0,
                        snap,
                    }
                } else {
                    let requested_at = drv.clock_now();
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                    };
                    // Run the live disable I/O BEFORE any shadow enqueue/drain so shadow work can
                    // never shift release-cutoff TX deadlines (Major #1).
                    let res = if !watchdog {
                        let max_rx = if rt_executor_owned || drv.supports_rt_executor().is_ok() {
                            Some(0)
                        } else {
                            None
                        };
                        drv.begin_client_park_disable(max_rx, expected_stop_gen)
                    } else if rt_executor_owned || drv.supports_rt_executor().is_ok() {
                        drv.begin_disable_torque_budgeted(disable_kind, 0)
                    } else {
                        drv.begin_disable_torque(disable_kind)
                    };
                    #[cfg(not(target_arch = "wasm32"))]
                    {
                        let live_op_id =
                            action_id.unwrap_or_else(|| shadow_rt.alloc_live_op_id());
                        shadow_rt.enqueue_at(
                            requested_at,
                            crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                                live_op_id,
                                admitted: true,
                                settled_ok: None,
                            },
                        );
                        shadow_rt.enqueue_at(
                            requested_at,
                            crate::torque::shadow::ShadowInputKind::DisableRequest {
                                live_op_id,
                                explicit_release_authorized: false,
                                live_refusal: None,
                                live_intent: pre_live_intent,
                                live_is_torqued: pre_live_torqued,
                                live_stopped: safety_stopped,
                            },
                        );
                        shadow_rt.drain_from_driver(drv);
                    }
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    let post_now = drv.clock_now();
                    if !watchdog
                        && !frames_dispatched
                        && matches!(&res, Err(e) if crate::driver::is_estop_inhibited(e))
                        && !stop_before_write.load(Ordering::SeqCst)
                        && !estop_pending_now()
                        && !safety_stopped
                        && (urgent_stop_unapplied_now()
                            || expected_stop_gen.is_some_and(|g| current_stop_gen_now() != g))
                    {
                        #[cfg(not(target_arch = "wasm32"))]
                        {
                            shadow_rt.enqueue_at(
                                post_now,
                                crate::torque::shadow::ShadowInputKind::Stop {
                                    kind: crate::torque::StopKind::Stop,
                                    live_intent: pre_live_intent,
                                    live_is_torqued: pre_live_torqued,
                                    live_stopped: true,
                                },
                            );
                            shadow_rt.enqueue_at(
                                post_now,
                                crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                                    live_op_id: action_id.unwrap_or(0),
                                    admitted: false,
                                    settled_ok: Some(false),
                                },
                            );
                        }
                        // Driver stop-gated client park before any disable frame reached the wire:
                        // zero disable frames were transmitted, so cancel cleanly and hold torque.
                        ParkDisableStart::CancelBeforeDisable
                    } else {
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            measured_positions,
                            &mut ages_buf,
                            &mut bounds_buf,
                        );
                        ParkDisableStart::Outcome {
                            res,
                            requested_at,
                            post_now,
                            disable_kind,
                            frames_dispatched,
                            dispatched_mask,
                            snap,
                        }
                    }
                }
            };
            if rt_executor_owned || self.nonblocking_driver_release {
                match d.try_with_mut(run_start) {
                    Ok(v) => v,
                    Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                        ParkDisableStart::Contended
                    }
                    Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                        ParkDisableStart::Poisoned
                    }
                }
            } else {
                d.with_mut(run_start).unwrap_or(ParkDisableStart::Poisoned)
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;

        let (res, requested_at, post_now, disable_kind, frames_dispatched, dispatched_mask, snap) =
            match start {
                ParkDisableStart::CancelBeforeDisable => {
                    self.cancel_park_before_disable(action_id);
                    return;
                }
                ParkDisableStart::Contended => {
                    self.record_disable_lock_contention();
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "DRIVER_BUSY",
                        "Driver busy during park disable",
                    );
                    return;
                }
                ParkDisableStart::Poisoned => {
                    self.finish_park_disable_failed(
                        watchdog,
                        action_id,
                        "FEEDBACK_UNAVAILABLE",
                        "Driver unavailable during park disable",
                    );
                    return;
                }
                ParkDisableStart::Outcome {
                    res,
                    requested_at,
                    post_now,
                    disable_kind,
                    frames_dispatched,
                    dispatched_mask,
                    snap,
                } => {
                    self.reset_disable_lock_contention();
                    (
                        res,
                        requested_at,
                        post_now,
                        disable_kind,
                        frames_dispatched,
                        dispatched_mask,
                        snap,
                    )
                }
            };

        self.owner_driver_snapshot = snap;
        self.retire_motion_payloads();
        self.pending_deferred_done = None;
        self.end_gripper_stream();
        crate::rt_core::scratch::copy_into(&mut self.target_positions, &self.commanded_positions);
        self.commanded_velocities.fill(0.0);
        self.background_disable_retry = None;
        self.active_disable_kind = Some(disable_kind);
        self.active_disable_client_park = (!watchdog).then_some(expected_stop_gen);
        self.disable_requested_at(requested_at);

        match res {
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                    self.disable_first_frame_accepted();
                }
                self.torque_transition = Some(TorqueTransition::Disable {
                    origin: DisableOrigin::Park {
                        watchdog,
                        action_id,
                        expected_stop_gen,
                    },
                    kind: disable_kind,
                });
            }
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.complete_park_disable_accepted(watchdog, action_id);
            }
            Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }) => {
                let (err_detail, code) = self.handle_partial_disable_abort(e);
                self.finish_park_disable_failed(watchdog, action_id, code, err_detail.as_str());
            }
            Err(ref e) if crate::driver::is_estop_inhibited(e) => {
                // Keep `disable_pending` set when E-stop intervened and do not latch `DriverWriteFailed`.
                if let Some(id) = action_id {
                    if self
                        .last_action
                        .as_ref()
                        .is_some_and(|a| a.id == id && a.is_running())
                    {
                        let mut msg = self.take_action_msg_buf(id, 64);
                        msg.clear();
                        msg.push_str("E-stop latched during park disable");
                        self.finish_action(
                            id,
                            "failed",
                            Some("ESTOP_ACTIVE"),
                            Some(msg),
                            None,
                            None,
                            None,
                        );
                    }
                }
            }
            Err(e) => {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let client_park_stopped =
                    !watchdog && self.client_park_stop_invalidated(expected_stop_gen);
                let (code, text) = if client_park_stopped {
                    // Stop arrived mid-batch after partial motor disable: NEVER hand the remaining
                    // undispatched motors to an unstamped retry; hold remaining energized motors
                    // and latch `DriverWriteFailed`.
                    self.clear_active_disable_retry_state();
                    let code = self.disable_step_failed_inner(e, false);
                    (code, "Client park disable stopped mid-batch")
                } else {
                    let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                    let partial_retry = self.commit_partial_disable_retry_kind(
                        disable_kind,
                        post_now,
                        frames_dispatched,
                        partial_remaining,
                    );
                    if let Some(rk) = partial_retry {
                        let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                    } else if !refused {
                        let _ = self.arm_background_disable_retry_with_kind(disable_kind, post_now);
                    }
                    let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                    let text = if refused && !frames_dispatched {
                        "Park torque-off refused by rest-pose gate"
                    } else {
                        "Torque disable transmission failed"
                    };
                    (code, text)
                };
                self.finish_park_disable_failed(watchdog, action_id, code, text);
            }
        }
    }

    fn cancel_park_before_disable(&mut self, action_id: Option<u64>) {
        self.safety.mark_motion_finished();
        if let Some(id) = action_id {
            if self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running()) {
                self.finish_action(id, "cancelled", Some("STOPPED"), None, None, None, None);
            }
        }
    }

    /// Zero-allocation completion when park torque-off is accepted through the rest-pose gate:
    /// records `torque_intent = Off` and preserves any latched E-stop/Fault safety state.
    fn complete_park_disable_accepted(&mut self, watchdog: bool, action_id: Option<u64>) {
        self.disable_accepted();
        self.finish_park_disable_after_accepted(watchdog, action_id);
    }

    fn finish_park_disable_after_accepted(&mut self, watchdog: bool, action_id: Option<u64>) {
        if !self.safety.state().is_stopped() {
            let _ = self.torque_off_finish_rt();
        } else {
            self.reset_gripper_stall();
            self.gripper_hold_vel_ceiling = None;
            if self.driver.is_none() { self.measured_velocities.fill(0.0); }
        }
        self.finish_park_disable_ok(watchdog, action_id);
    }

    fn finish_park_rearm_failed(&mut self, action_id: u64, code: &'static str) {
        if self.park_continuation.is_some_and(|c| c.action_id == action_id) { self.park_continuation = None; }
        let mut msg = self.take_action_msg_buf(action_id, 64);
        msg.clear(); msg.push_str("Park rearm failed");
        self.finish_action(action_id, "failed", Some(code), Some(msg), None, None, None);
        self.notify_park_continuation();
    }

    fn finish_park_disable_ok(&mut self, watchdog: bool, action_id: Option<u64>) {
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                live_op_id: action_id.unwrap_or(0),
                admitted: false,
                settled_ok: Some(true),
            });
        if watchdog && !self.rt_executor_owned { tracing::info!("Watchdog park complete: torque off at resting pose"); }
        if let Some(id) = action_id { if !self.safety.state().is_stopped() { self.finish_action(id, "done", None, None, None, None, None); } }
    }

    fn finish_park_disable_failed(&mut self, watchdog: bool, action_id: Option<u64>, code: &'static str, text: &str) {
        #[cfg(not(target_arch = "wasm32"))]
        self.torque_shadow
            .enqueue(crate::torque::shadow::ShadowInputKind::ParkLifecycle {
                live_op_id: action_id.unwrap_or(0),
                admitted: false,
                settled_ok: Some(false),
            });
        self.safety.mark_motion_finished();
        if watchdog && (self.is_torqued || self.torque_intent == TorqueIntent::On) && !self.safety.state().is_stopped() {
            self.latch_watchdog_fault();
        }
        if let Some(id) = action_id {
            if self.last_action.as_ref().is_some_and(|a| a.id == id && a.is_running()) {
                let mut msg = self.take_action_msg_buf(id, 96);
                msg.clear(); msg.push_str(text);
                self.finish_action(id, "failed", Some(code), Some(msg), None, None, None);
            }
        }
    }

    #[inline]
    fn all_joints_dispatched_for_disable(
        &self,
        frames_dispatched: bool,
        dispatched_mask: u64,
    ) -> bool {
        let all = self.all_joints_mask();
        if dispatched_mask != 0 {
            (dispatched_mask & all) == all
        } else {
            frames_dispatched
        }
    }

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

    /// Serves any background (non-broker) disable handshake and bounded [`BackgroundDisableRetry`]
    /// started by `request_disable_bounded` or a transport-failed broker disable under RT executor
    /// ownership, sharing Step 2's single RX budget.
    ///
    /// Every retry routes strictly through the driver's single
    /// `begin_disable_torque_budgeted(kind, 0)` (`send(DisableRequested | DisableProtective, ...)`)
    /// choke point with the ORIGINAL `DisableKind` (`started_at`) and rest-pose gate within the
    /// single request lifetime. Exhausting retries or the window latches
    /// `StaticFault::DisableUnconfirmed` with zero heap allocations; choke-point refusals
    /// (`TorqueOffWouldDrop` / `DisableRequestExpired`) terminate cleanly without retrying.
    pub(crate) fn service_background_disable_with_clock(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Option<(u16, bool)> {
        if !self.disable_pending || self.torque_transition.is_some() {
            return None;
        }
        self.driver.as_ref()?;
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            return None;
        }
        let control_release_seq = self.control_release_seq;
        // Probe / poll phase under non-blocking try_lock (M4).
        enum BgPollStep {
            Contended,
            Poisoned,
            BudgetExpired,
            Polled {
                poll_res: Result<crate::driver::TorqueProgress, crate::driver::ArmDriverError>,
                rx: Option<(u16, bool)>,
                post_now: Instant,
                still_pending: bool,
                frames_dispatched: bool,
                dispatched_mask: u64,
            },
            Idle {
                drv_now: Instant,
            },
        }
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let conservative_cap = self.pending_conservative_tx_cap;
        let mut cons_rx_delta = 0u16;
        let mut cons_tx_delta = 0u16;
        let mut new_snap = None;
        let step = {
            #[cfg(not(target_arch = "wasm32"))]
            let shadow_rt = &mut self.torque_shadow;
            let d = self.driver.as_mut().expect("checked is_some");
            match d.try_with_mut(|drv| {
                if control_release_seq > 0 {
                    drv.begin_control_release(control_release_seq);
                }
                if drv.torque_transition_pending() {
                    let Ok((_, _, drv_deadline)) =
                        crate::supervisor::translate_driver_cutoff_conservative(
                            drv,
                            budget.deadline,
                            now_fn,
                        )
                    else {
                        return BgPollStep::BudgetExpired;
                    };
                    let before_cons = if conservative_cap {
                        let bc = drv.tx().release_budget_consumed();
                        drv.tx_mut().sync_release_budget_limits(0, 0);
                        Some(bc)
                    } else {
                        None
                    };
                    let poll_res = drv.poll_torque_budgeted(drv_deadline, budget.max_rx_frames);
                    if let Some(bc) = before_cons {
                        let after = drv.tx().release_budget_consumed();
                        cons_rx_delta = cons_rx_delta.saturating_add(after.0.saturating_sub(bc.0));
                        cons_tx_delta = cons_tx_delta.saturating_add(after.1.saturating_sub(bc.1));
                    }
                    let rx = drv.take_last_poll_rx();
                    let post_now = drv.clock_now();
                    new_snap = Some(OwnerDriverSnapshot::capture_from_driver(
                        &*drv,
                        dof,
                        feedback_numeric_valid,
                        measured_positions,
                        &mut ages_buf,
                        &mut bounds_buf,
                    ));
                    let still_pending = drv.torque_transition_pending();
                    let frames_dispatched = drv.disable_frames_dispatched();
                    let dispatched_mask = drv.disable_dispatched_mask();
                    #[cfg(not(target_arch = "wasm32"))]
                    shadow_rt.drain_from_driver(drv);
                    BgPollStep::Polled {
                        poll_res,
                        rx,
                        post_now,
                        still_pending,
                        frames_dispatched,
                        dispatched_mask,
                    }
                } else {
                    BgPollStep::Idle { drv_now: drv.clock_now() }
                }
            }) {
                Ok(v) => v,
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => BgPollStep::Contended,
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => BgPollStep::Poisoned,
            }
        };
        if conservative_cap {
            self.conservative_step_rx_delta =
                self.conservative_step_rx_delta.saturating_add(cons_rx_delta);
            self.conservative_step_tx_delta =
                self.conservative_step_tx_delta.saturating_add(cons_tx_delta);
        }
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        if let Some(snap) = new_snap {
            self.owner_driver_snapshot = snap;
        }
        let drv_now = match step {
            BgPollStep::Contended => {
                self.record_disable_lock_contention();
                return None;
            }
            BgPollStep::Poisoned => return None,
            BgPollStep::BudgetExpired => return Some((0, true)),
            BgPollStep::Polled {
                poll_res,
                rx,
                post_now,
                still_pending,
                frames_dispatched,
                dispatched_mask,
            } => {
                self.reset_disable_lock_contention();
                let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                match poll_res {
                    Ok(crate::driver::TorqueProgress::Done(_)) => {
                        self.disable_accepted();
                        let _ = self.torque_off_finish_rt();
                    }
                    Err(
                        crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired,
                    ) => {
                        if frames_dispatched {
                            self.disable_first_frame_accepted();
                        }
                        if partial_remaining {
                            match self.background_disable_retry {
                                None => {
                                    let base_kind = self.active_disable_kind.unwrap_or(
                                        crate::driver::DisableKind::DisableRequested {
                                            started_at: post_now,
                                        },
                                    );
                                    if let Some(rk) = self.commit_partial_disable_retry_kind(
                                        base_kind,
                                        post_now,
                                        true,
                                        true,
                                    ) {
                                        let _ = self.arm_background_disable_retry_immediate_with_kind(
                                            rk,
                                            post_now,
                                        );
                                    }
                                }
                                Some(BackgroundDisableRetry::Active {
                                    attempts,
                                    kind,
                                    started_at,
                                    client_park,
                                    ..
                                }) => {
                                    if let Some(expected_stop_gen) = client_park {
                                        if self.client_park_stop_invalidated(expected_stop_gen) {
                                            self.clear_active_disable_retry_state();
                                            return rx;
                                        }
                                    }
                                    let window = self.background_disable_window_for(kind);
                                    if post_now.saturating_duration_since(started_at) >= window
                                        || attempts >= BACKGROUND_DISABLE_MAX_RETRIES
                                    {
                                        self.exhaust_background_disable_retry();
                                    } else {
                                        self.background_disable_retry =
                                            Some(BackgroundDisableRetry::Active {
                                                attempts,
                                                kind,
                                                started_at,
                                                next_retry_at: post_now,
                                                client_park,
                                            });
                                    }
                                }
                                Some(BackgroundDisableRetry::Exhausted) => {}
                            }
                        } else {
                            self.disable_pending = false;
                            self.active_disable_kind = None;
                            self.last_disable_attempt = None;
                            self.background_disable_retry = None;
                        }
                    }
                    Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                        if frames_dispatched {
                            self.disable_first_frame_accepted();
                        }
                        if let Some(BackgroundDisableRetry::Active {
                            kind,
                            started_at,
                            ..
                        }) = self.background_disable_retry
                        {
                            let window = self.background_disable_window_for(kind);
                            if post_now.saturating_duration_since(started_at) >= window {
                                if matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
                                    && self.torque_off_stopped()
                                    && !frames_dispatched
                                {
                                    self.disable_pending = false;
                                    self.active_disable_kind = None;
                                    self.last_disable_attempt = None;
                                    self.background_disable_retry = None;
                                } else {
                                    self.exhaust_background_disable_retry();
                                }
                            }
                        }
                    }
                    Err(_) => {
                        if frames_dispatched {
                            self.disable_first_frame_accepted();
                        }
                        match self.background_disable_retry {
                            Some(BackgroundDisableRetry::Active {
                                attempts,
                                kind,
                                started_at,
                                client_park,
                                ..
                            }) => {
                                if let Some(expected_stop_gen) = client_park {
                                    if self.client_park_stop_invalidated(expected_stop_gen) {
                                        self.clear_active_disable_retry_state();
                                        return rx;
                                    }
                                }
                                let window = self.background_disable_window_for(kind);
                                let window_expired =
                                    post_now.saturating_duration_since(started_at) >= window;
                                if window_expired
                                    && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
                                    && self.torque_off_stopped()
                                    && !frames_dispatched
                                {
                                    self.clear_active_disable_retry_state();
                                } else if window_expired
                                    || (!still_pending && attempts >= BACKGROUND_DISABLE_MAX_RETRIES)
                                {
                                    self.exhaust_background_disable_retry();
                                } else if !still_pending {
                                    let is_protective = matches!(
                                        kind,
                                        crate::driver::DisableKind::DisableProtective { .. }
                                    );
                                    self.background_disable_retry =
                                        Some(BackgroundDisableRetry::Active {
                                            attempts,
                                            kind,
                                            started_at,
                                            next_retry_at: if partial_remaining || is_protective {
                                                post_now
                                            } else {
                                                post_now + BACKGROUND_DISABLE_RETRY_INTERVAL
                                            },
                                            client_park,
                                        });
                                }
                            }
                            Some(BackgroundDisableRetry::Exhausted) => {}
                            None => {
                                let kind = self.active_disable_kind.unwrap_or(
                                    crate::driver::DisableKind::DisableRequested {
                                        started_at: post_now,
                                    },
                                );
                                let is_protective = matches!(
                                    kind,
                                    crate::driver::DisableKind::DisableProtective { .. }
                                );
                                if let Some(rk) = self.commit_partial_disable_retry_kind(
                                    kind,
                                    post_now,
                                    frames_dispatched,
                                    partial_remaining,
                                ) {
                                    let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                                } else if is_protective {
                                    let _ = self.arm_background_disable_retry_immediate_with_kind(kind, post_now);
                                } else {
                                    let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                                }
                            }
                        }
                    }
                }
                return rx;
            }
            BgPollStep::Idle { drv_now } => {
                self.reset_disable_lock_contention();
                drv_now
            }
        };

        let Some(BackgroundDisableRetry::Active {
            attempts,
            kind,
            started_at,
            next_retry_at,
            client_park,
        }) = self.background_disable_retry
        else {
            return None;
        };
        if let Some(expected_stop_gen) = client_park {
            if self.client_park_stop_invalidated(expected_stop_gen) {
                self.clear_active_disable_retry_state();
                return None;
            }
        }
        let window = self.background_disable_window_for(kind);
        let window_expired = drv_now.saturating_duration_since(started_at) >= window;
        if window_expired
            && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
            && self.torque_off_stopped()
            && self.torque_intent != TorqueIntent::Off
        {
            self.clear_active_disable_retry_state();
            return None;
        }
        if attempts >= BACKGROUND_DISABLE_MAX_RETRIES || window_expired {
            self.exhaust_background_disable_retry();
            return None;
        }
        if drv_now < next_retry_at {
            return None;
        }

        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let dof = self.config.dof;
        let feedback_numeric_valid = self.feedback_numeric_valid;
        let measured_positions = &self.measured_positions;
        let retry_out = {
            let d = self.driver.as_mut().expect("checked is_some");
            match d.try_with_mut(|drv| {
                let res = if let Some(expected_stop_gen) = client_park {
                    drv.begin_client_park_disable_with_kind(kind, Some(0), expected_stop_gen)
                } else {
                    drv.begin_disable_torque_budgeted(kind, 0)
                };
                let handshake_pending = drv.torque_transition_pending();
                let frames_dispatched = drv.disable_frames_dispatched();
                let dispatched_mask = drv.disable_dispatched_mask();
                let post_now = drv.clock_now();
                let snap = OwnerDriverSnapshot::capture_from_driver(
                    &*drv,
                    dof,
                    feedback_numeric_valid,
                    measured_positions,
                    &mut ages_buf,
                    &mut bounds_buf,
                );
                (res, handshake_pending, frames_dispatched, dispatched_mask, post_now, snap)
            }) {
                Ok(v) => Ok(v),
                Err(crate::supervisor_lock::DriverTryError::WouldBlock) => Err(true),
                Err(crate::supervisor_lock::DriverTryError::Poisoned) => Err(false),
            }
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
        let (res, handshake_pending, frames_dispatched, dispatched_mask, post_now, snap) = match retry_out {
            Ok(v) => {
                self.reset_disable_lock_contention();
                v
            }
            Err(true) => {
                self.record_disable_lock_contention();
                return None;
            }
            Err(false) => return None,
        };
        self.owner_driver_snapshot = snap;
        let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);

        match res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.disable_accepted();
                let _ = self.torque_off_finish_rt();
                return None;
            }
            Err(
                crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired,
            ) => {
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
                if !partial_remaining {
                    self.disable_pending = false;
                    self.active_disable_kind = None;
                    self.last_disable_attempt = None;
                    self.background_disable_retry = None;
                    return None;
                }
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) => {
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
            }
            Err(e) => {
                if let Some(expected_stop_gen) = client_park {
                    if crate::driver::is_estop_inhibited(&e)
                        || self.client_park_stop_invalidated(expected_stop_gen)
                    {
                        self.clear_active_disable_retry_state();
                        return None;
                    }
                }
                if frames_dispatched {
                    self.disable_first_frame_accepted();
                }
            }
        }

        let next_attempts = attempts.saturating_add(1);
        let window_expired = post_now.saturating_duration_since(started_at) >= window;
        if window_expired
            && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
            && self.torque_off_stopped()
            && !frames_dispatched
        {
            self.clear_active_disable_retry_state();
        } else if window_expired || (!handshake_pending && next_attempts >= BACKGROUND_DISABLE_MAX_RETRIES)
        {
            self.exhaust_background_disable_retry();
        } else {
            let is_protective = matches!(
                kind,
                crate::driver::DisableKind::DisableProtective { .. }
            );
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: next_attempts,
                kind,
                started_at,
                next_retry_at: if partial_remaining || is_protective {
                    post_now
                } else {
                    post_now + BACKGROUND_DISABLE_RETRY_INTERVAL
                },
                client_park,
            });
        }
        None
    }

    /// Serves the transition in flight once, within `budget` (the executor's clock; the
    /// driver's receive pass gets the same span on the driver's clock). None: none in flight.
    /// `Pending { wake }` is on the executor's clock.
    pub(crate) fn service_torque_transition(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
    ) -> Option<TransitionPoll> {
        let mut live_now = Instant::now;
        self.service_torque_transition_with_clock(budget, &mut live_now)
    }

    pub(crate) fn service_torque_transition_with_clock(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Option<TransitionPoll> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        let t = self.torque_transition.take()?;
        let ws_origin_dir = match t {
            TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::LegacyWs { .. } | EnableOrigin::DetachedCancel,
                ..
            }) => Some("on"),
            TorqueTransition::Disable {
                origin: DisableOrigin::LegacyWs { .. } | DisableOrigin::Detached { .. },
                ..
            } => Some("off"),
            _ => None,
        };
        self.last_tick_torque_ws_direction = ws_origin_dir;
        // Enforce authoritative request deadline BEFORE dispatch, retry, or arming (Major 1 / B4).
        match t {
            TorqueTransition::Enable(EnableReducer {
                origin: EnableOrigin::LegacyWs { token, slot, deadline, .. },
                ..
            }) if Instant::now() >= deadline => {
                self.torque_transition = Some(t);
                let _ = self.expire_legacy_torque_transition_inner(token, slot, true, true);
                return Some(TransitionPoll {
                    progress: Err("DEADLINE_EXPIRED"),
                    rx: None,
                    broker_owned: false,
                });
            }
            TorqueTransition::Disable {
                origin: DisableOrigin::LegacyWs { token, slot, deadline },
                ..
            } if Instant::now() >= deadline => {
                self.torque_transition = Some(t);
                let _ = self.expire_legacy_torque_transition_inner(token, slot, false, true);
                return Some(TransitionPoll {
                    progress: Err("DEADLINE_EXPIRED"),
                    rx: None,
                    broker_owned: false,
                });
            }
            TorqueTransition::Disable {
                origin: DisableOrigin::Detached { deadline: Some(dl) },
                kind,
            } if Instant::now() >= dl => {
                match self.try_abort_expired_undispatched_disable(kind, Some(dl)) {
                    DisableExpiryDecision::Aborted => {
                        return Some(TransitionPoll {
                            progress: Err("DEADLINE_EXPIRED"),
                            rx: None,
                            broker_owned: false,
                        });
                    }
                    DisableExpiryDecision::ContendedRetry
                    | DisableExpiryDecision::KeepDetachedWithDeadline => {
                        return Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: budget.now }),
                            rx: None,
                            broker_owned: false,
                        });
                    }
                    DisableExpiryDecision::KeepDetachedDispatched => {
                        // Already dispatched (or protective): let the next line poll to completion
                        // with `deadline: None`.
                        let next = self.torque_transition.take()?;
                        return self.service_torque_transition_post_deadline(next, budget, now_fn);
                    }
                }
            }
            _ => {}
        }
        let out = self.service_torque_transition_post_deadline(t, budget, now_fn);
        self.sync_detached_torque_flag();
        out
    }

    fn service_torque_transition_post_deadline(
        &mut self,
        t: TorqueTransition,
        budget: &crate::rt_core::ops::ServiceBudget,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Option<TransitionPoll> {
        use crate::driver::TorqueProgress as Drv;
        use crate::rt_core::ops::TorqueProgress as Rt;
        let broker_owned = matches!(
            t,
            TorqueTransition::Enable(EnableReducer { origin: EnableOrigin::BrokerTransition, .. })
                | TorqueTransition::Disable { origin: DisableOrigin::BrokerTransition, .. }
        );
        match t {
            TorqueTransition::Enable(reducer) => {
                let origin = reducer.origin;
                let (outcome, rx) = self.poll_enable_reducer_with_clock(reducer, Some(budget), now_fn);
                let progress = match outcome {
                    EnableStepOutcome::Armed(_) => {
                        match origin {
                            EnableOrigin::ParkRearm { action_id, stop_gen } => {
                                if let Some(c) = self
                                    .park_continuation
                                    .as_mut()
                                    .filter(|c| c.action_id == action_id && c.stop_gen == stop_gen)
                                {
                                    c.phase = super::ParkContinuationPhase::ReadyForPlan;
                                }
                                self.notify_park_continuation();
                            }
                            EnableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_accepted(token, slot);
                            }
                            EnableOrigin::BrokerTransition | EnableOrigin::DetachedCancel => {}
                        }
                        Ok(Rt::Done)
                    }
                    EnableStepOutcome::Pending { wake } => Ok(Rt::Pending { wake }),
                    EnableStepOutcome::Failed {
                        rt_code,
                        message,
                        feedback_unavailable,
                        estop_interrupted,
                    } => {
                        match origin {
                            EnableOrigin::ParkRearm { action_id, .. } => {
                                if rt_code != "STOPPED" {
                                    self.finish_park_rearm_failed(action_id, rt_code);
                                }
                            }
                            EnableOrigin::LegacyWs { token, slot, .. } => {
                                let failure = Self::classify_legacy_enable_failure(
                                    rt_code,
                                    message,
                                    feedback_unavailable,
                                    estop_interrupted,
                                );
                                self.settle_legacy_ws_failed(token, slot, failure);
                            }
                            EnableOrigin::BrokerTransition | EnableOrigin::DetachedCancel => {}
                        }
                        Err(rt_code)
                    }
                };
                Some(TransitionPoll { progress, rx, broker_owned })
            }
            TorqueTransition::Disable { origin, kind } => {
                if self.driver.is_none() {
                    match origin {
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.finish_park_disable_failed(
                                watchdog,
                                action_id,
                                "INTERNAL",
                                "Driver missing during park disable",
                            );
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.settle_legacy_ws_failed(
                                token,
                                slot,
                                LegacyTorqueFailure::StaticCode("INTERNAL"),
                            );
                        }
                        DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                    }
                    return Some(TransitionPoll { progress: Err("INTERNAL"), rx: None, broker_owned });
                }
                let req_deadline = match origin {
                    DisableOrigin::LegacyWs { deadline, .. } => Some(deadline),
                    DisableOrigin::Detached { deadline } => deadline,
                    _ => None,
                };
                let is_protective =
                    matches!(kind, crate::driver::DisableKind::DisableProtective { .. });
                let control_release_seq = self.control_release_seq;
                let rt_executor_owned = self.rt_executor_owned;
                let nonblocking_lock = rt_executor_owned
                    || self.nonblocking_driver_release
                    || broker_owned
                    || matches!(origin, DisableOrigin::LegacyWs { .. } | DisableOrigin::Detached { .. });
                let dof = self.config.dof;
                let feedback_numeric_valid = self.feedback_numeric_valid;
                let measured_positions = &self.measured_positions;
                let conservative_cap = self.pending_conservative_tx_cap;
                let mut cons_rx_delta = 0u16;
                let mut cons_tx_delta = 0u16;
                let mut ages_buf = std::mem::take(&mut self.scratch.ages);
                let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
                #[allow(clippy::large_enum_variant)] // RT stack-only enum; avoids heap Box
                enum DisablePollOutcome {
                    Ran(
                        Instant,
                        Result<Drv, crate::driver::ArmDriverError>,
                        bool,
                        u64,
                        Option<(u16, bool)>,
                        Instant,
                        OwnerDriverSnapshot,
                    ),
                    BudgetExpired(Instant),
                    Contended,
                    Poisoned,
                }
                let poll_out = {
                    #[cfg(not(target_arch = "wasm32"))]
                    let shadow_rt = &mut self.torque_shadow;
                    let d = self.driver.as_mut().expect("checked is_some");
                    let run_poll = |drv: &mut dyn crate::driver::ArmDriver| {
                        if control_release_seq > 0 {
                            drv.begin_control_release(control_release_seq);
                        }
                        let (drv_now, live_owner_now, mut drv_deadline) =
                            match crate::supervisor::translate_driver_cutoff_conservative(
                                drv,
                                budget.deadline,
                                now_fn,
                            ) {
                                Ok(translated) => translated,
                                Err((_, owner_now)) => {
                                    return DisablePollOutcome::BudgetExpired(owner_now);
                                }
                            };
                        // Bound any undispatched non-protective WS/Detached disable by its
                        // remaining request deadline so first dispatch never crosses expiry (R5 B2/M2).
                        let mut applied_ws_dl = false;
                        if let Some(dl) = req_deadline {
                            if !is_protective && !drv.disable_frames_dispatched() {
                                let req_rem = dl.saturating_duration_since(live_owner_now);
                                drv_deadline = drv_deadline.min(drv_now + req_rem);
                                drv.set_tx_release_deadline(Some(drv_deadline));
                                applied_ws_dl = true;
                            }
                        }
                        let use_budgeted = broker_owned
                            || rt_executor_owned
                            || drv.supports_rt_executor().is_ok();
                        let before_cons = if conservative_cap {
                            let bc = drv.tx().release_budget_consumed();
                            drv.tx_mut().sync_release_budget_limits(0, 0);
                            Some(bc)
                        } else {
                            None
                        };
                        let r = if use_budgeted {
                            drv.poll_torque_budgeted(drv_deadline, budget.max_rx_frames)
                        } else {
                            drv.poll_torque(drv_deadline)
                        };
                        if let Some(bc) = before_cons {
                            let after = drv.tx().release_budget_consumed();
                            cons_rx_delta = cons_rx_delta.saturating_add(after.0.saturating_sub(bc.0));
                            cons_tx_delta = cons_tx_delta.saturating_add(after.1.saturating_sub(bc.1));
                        }
                        if applied_ws_dl {
                            drv.set_tx_release_deadline(None);
                        }
                        let frames_dispatched = drv.disable_frames_dispatched();
                        let dispatched_mask = drv.disable_dispatched_mask();
                        let rx = drv.take_last_poll_rx();
                        let post_now = drv.clock_now();
                        let snap = OwnerDriverSnapshot::capture_from_driver(
                            &*drv,
                            dof,
                            feedback_numeric_valid,
                            measured_positions,
                            &mut ages_buf,
                            &mut bounds_buf,
                        );
                        #[cfg(not(target_arch = "wasm32"))]
                        shadow_rt.drain_from_driver(drv);
                        DisablePollOutcome::Ran(
                            drv_now,
                            r,
                            frames_dispatched,
                            dispatched_mask,
                            rx,
                            post_now,
                            snap,
                        )
                    };
                    if nonblocking_lock {
                        match d.try_with_mut(run_poll) {
                            Ok(v) => v,
                            Err(crate::supervisor_lock::DriverTryError::WouldBlock) => {
                                DisablePollOutcome::Contended
                            }
                            Err(crate::supervisor_lock::DriverTryError::Poisoned) => {
                                DisablePollOutcome::Poisoned
                            }
                        }
                    } else {
                        d.with_mut(run_poll).unwrap_or(DisablePollOutcome::Poisoned)
                    }
                };
                if conservative_cap {
                    self.conservative_step_rx_delta =
                        self.conservative_step_rx_delta.saturating_add(cons_rx_delta);
                    self.conservative_step_tx_delta =
                        self.conservative_step_tx_delta.saturating_add(cons_tx_delta);
                }
                self.scratch.ages = ages_buf;
                self.scratch.fb.bounds = bounds_buf;
                let (drv_now, r, frames_dispatched, dispatched_mask, rx, post_now, snap) = match poll_out {
                    DisablePollOutcome::Ran(
                        drv_now,
                        r,
                        frames_dispatched,
                        dispatched_mask,
                        rx,
                        post_now,
                        snap,
                    ) => {
                        self.reset_disable_lock_contention();
                        (drv_now, r, frames_dispatched, dispatched_mask, rx, post_now, snap)
                    }
                    DisablePollOutcome::BudgetExpired(owner_now) => {
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        return Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: owner_now }),
                            rx: Some((0, true)),
                            broker_owned,
                        });
                    }
                    DisablePollOutcome::Contended => {
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        self.record_disable_lock_contention();
                        return Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: budget.now }),
                            rx: None,
                            broker_owned,
                        });
                    }
                    DisablePollOutcome::Poisoned => {
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(
                                    watchdog,
                                    action_id,
                                    "FEEDBACK_UNAVAILABLE",
                                    "Driver unavailable during park disable",
                                );
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_failed(
                                    token,
                                    slot,
                                    LegacyTorqueFailure::FeedbackUnavailable,
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        return Some(TransitionPoll {
                            progress: Err("FEEDBACK_UNAVAILABLE"),
                            rx: None,
                            broker_owned,
                        });
                    }
                };
                self.owner_driver_snapshot = snap;
                let to_exec = |wake: Instant| budget.now + wake.saturating_duration_since(drv_now);
                match r {
                    Ok(Drv::Pending { wake }) => {
                        if self.all_joints_dispatched_for_disable(frames_dispatched, dispatched_mask) {
                            self.disable_first_frame_accepted();
                        }
                        self.torque_transition = Some(TorqueTransition::Disable { origin, kind });
                        Some(TransitionPoll {
                            progress: Ok(Rt::Pending { wake: to_exec(wake) }),
                            rx,
                            broker_owned,
                        })
                    }
                    Ok(Drv::Done(_)) => match origin {
                        DisableOrigin::BrokerTransition => {
                            self.disable_accepted();
                            let progress = self.torque_off_finish_rt().map(|()| Rt::Done);
                            Some(TransitionPoll { progress, rx, broker_owned })
                        }
                        DisableOrigin::Park { watchdog, action_id, .. } => {
                            self.complete_park_disable_accepted(watchdog, action_id);
                            Some(TransitionPoll { progress: Ok(Rt::Done), rx, broker_owned })
                        }
                        DisableOrigin::LegacyWs { token, slot, .. } => {
                            self.disable_accepted();
                            let progress = match self.torque_off_finish_rt() {
                                Ok(()) => {
                                    self.settle_legacy_ws_accepted(token, slot);
                                    Ok(Rt::Done)
                                }
                                Err(code) => {
                                    self.settle_legacy_ws_disable_failed(token, slot, code, None);
                                    Err(code)
                                }
                            };
                            Some(TransitionPoll { progress, rx, broker_owned })
                        }
                        DisableOrigin::Detached { .. } => {
                            self.disable_accepted();
                            let progress = self.torque_off_finish_rt().map(|()| Rt::Done);
                            Some(TransitionPoll { progress, rx, broker_owned })
                        }
                    },
                    Err(e @ crate::driver::ArmDriverError::PartialDisable { .. }) => {
                        let (err_detail, code) = self.handle_partial_disable_abort(e);
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(
                                    watchdog,
                                    action_id,
                                    code,
                                    err_detail.as_str(),
                                );
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_failed(
                                    token,
                                    slot,
                                    LegacyTorqueFailure::DisableFailed { detail: err_detail },
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        Some(TransitionPoll {
                            progress: Err(code),
                            rx,
                            broker_owned,
                        })
                    }
                    Err(crate::driver::ArmDriverError::LeaseFenced) => {
                        self.clear_active_disable_retry_state();
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(
                                    watchdog,
                                    action_id,
                                    "LEASE_LOST",
                                    "Lease lost before park disable dispatched",
                                );
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_failed(
                                    token,
                                    slot,
                                    LegacyTorqueFailure::LeaseLost,
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        Some(TransitionPoll {
                            progress: Err("LEASE_LOST"),
                            rx,
                            broker_owned,
                        })
                    }
                    Err(e) => {
                        if let DisableOrigin::Park {
                            watchdog: false,
                            action_id,
                            ..
                        } = origin
                        {
                            if !frames_dispatched
                                && crate::driver::is_estop_inhibited(&e)
                                && !self.e_stop_latched.load(Ordering::SeqCst)
                                && !self.estop_pending()
                                && !self.safety.state().is_stopped()
                            {
                                self.clear_active_disable_retry_state();
                                self.cancel_park_before_disable(action_id);
                                return Some(TransitionPoll {
                                    progress: Err("STOPPED"),
                                    rx,
                                    broker_owned,
                                });
                            }
                        }
                        if crate::driver::is_estop_inhibited(&e) {
                            match origin {
                                DisableOrigin::Park { watchdog, action_id, .. } => {
                                    self.finish_park_disable_failed(
                                        watchdog,
                                        action_id,
                                        "ESTOP_ACTIVE",
                                        "E-stop inhibited park disable",
                                    );
                                }
                                DisableOrigin::LegacyWs { token, slot, .. } => {
                                    self.settle_legacy_ws_failed(
                                        token,
                                        slot,
                                        LegacyTorqueFailure::EStopActive,
                                    );
                                }
                                DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                            }
                            return Some(TransitionPoll {
                                progress: Err("ESTOP_ACTIVE"),
                                rx,
                                broker_owned,
                            });
                        }
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        let client_park_stopped = matches!(
                            origin,
                            DisableOrigin::Park {
                                watchdog: false,
                                expected_stop_gen,
                                ..
                            } if self.client_park_stop_invalidated(expected_stop_gen)
                        );
                        let mut err_detail =
                            crate::safety::BoundedText::<{ crate::safety::SAFETY_LOG_TEXT_CAPACITY }>::empty();
                        err_detail.push_str("Torque disable transmission failed: ");
                        e.write_bounded(&mut err_detail);
                        let (code, text) = if client_park_stopped {
                            self.clear_active_disable_retry_state();
                            let code = self.disable_step_failed_inner(e, false);
                            (code, "Client park disable stopped mid-batch")
                        } else {
                            if let DisableOrigin::Park {
                                watchdog: false,
                                expected_stop_gen,
                                ..
                            } = origin
                            {
                                self.active_disable_client_park = Some(expected_stop_gen);
                            }
                            let partial_remaining = self.is_partial_disable_remaining(dispatched_mask);
                            let partial_retry = self.commit_partial_disable_retry_kind(
                                kind,
                                post_now,
                                frames_dispatched,
                                partial_remaining,
                            );
                            let allow_undispatched_retry = !matches!(
                                origin,
                                DisableOrigin::LegacyWs { .. }
                                    | DisableOrigin::Detached { deadline: Some(_), .. }
                            );
                            if let Some(rk) = partial_retry {
                                let _ = self.arm_background_disable_retry_immediate_with_kind(rk, post_now);
                            } else if !refused
                                && (frames_dispatched || is_protective || allow_undispatched_retry)
                            {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            } else if !frames_dispatched && !is_protective && !allow_undispatched_retry {
                                self.clear_active_disable_retry_state();
                            }
                            let code = self.disable_step_failed_inner(e, partial_retry.is_some());
                            let text = if refused && !frames_dispatched {
                                "Park torque-off refused by rest-pose gate"
                            } else {
                                "Torque disable transmission failed"
                            };
                            (code, text)
                        };
                        match origin {
                            DisableOrigin::Park { watchdog, action_id, .. } => {
                                self.finish_park_disable_failed(watchdog, action_id, code, text);
                            }
                            DisableOrigin::LegacyWs { token, slot, .. } => {
                                self.settle_legacy_ws_disable_failed(
                                    token,
                                    slot,
                                    code,
                                    Some(err_detail),
                                );
                            }
                            DisableOrigin::BrokerTransition | DisableOrigin::Detached { .. } => {}
                        }
                        Some(TransitionPoll {
                            progress: Err(code),
                            rx,
                            broker_owned,
                        })
                    }
                }
            }
        }
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
            config_reconcile_fault: self.config_reconcile_fault,
            gravity_comp_unconfirmed: self.gravity_comp_blocks_clear(),
            disable_pending: self.disable_pending,
            has_driver: self.driver.is_some(),
            feedback_fresh: self.driver.is_none(),
            j7_feedback_fresh: self.driver.is_none()
                && !self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale),
            tx_queue_drained: self.driver.is_none(),
            tx_probe_confirmed: self.driver.is_none(),
            tx_probe_sent: false,
            thermal_sensor_available: self.driver.is_none(),
            thermal_below_threshold: thermal_below,
            thermal_cooldown_elapsed_ms: cooldown_elapsed_ms,
            thermal_cooldown_required_ms: self.thermal_cooldown_duration.as_millis().min(u128::from(u64::MAX)) as u64,
            has_active_lease,
            lease_heartbeat_fresh,
            joint_limit_violation: self.joint_limit_clear_violation(),
            enable_state_known: self.driver.is_none(),
            failed_hold_recovery_allowed: false,
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
            let _ = d.with_ref(|drv| {
                ctx.mit_modes_uncertain = self.mit.owner == crate::mit::MitOwner::Lost && drv.joint_mode().joint_modes_uncertain();
                ctx.thermal_sensor_available = drv.capabilities().temperature_feedback;
                ctx.feedback_fresh = self.feedback_numeric_valid
                    && drv.is_feedback_valid()
                    && drv.feedback_all_within(self.config.dof, Duration::from_millis(250));
                ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                ctx.tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
                ctx.tx_probe_confirmed = drv.tx().tx_probe_confirmed();
                ctx.tx_probe_sent = drv.tx().tx_probe_pending();
                ctx.enable_state_known = true;
            });
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
        let Some(drv) = d.read() else {
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
        if let Err(err) = candidate.validate_runtime_narrowed() {
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
        match self.clear_fault_for_epoch_with_authority(expected_epoch, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultLegacyInnerError::Safety(e)) => Err(e),
            Err(ClearFaultLegacyInnerError::Authority(_)) => {
                unreachable!("authority is None on direct clear_fault_for_epoch")
            }
        }
    }

    pub(crate) fn clear_fault_for_epoch_with_authority(
        &mut self,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
    ) -> Result<SafetyState, ClearFaultLegacyInnerError> {
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly;
        use crate::safety::ClearRefusal;
        let had_driver_config_error = self.driver_config_error.is_some();
        let (candidate_limits, driver_err_override, clear_driver_err_field) = match self.probe_driver_limits_legacy() {
            Ok(cand) => (cand, None, had_driver_config_error),
            Err(err) => (None, Some(err), false),
        };
        let apply_estop_refusal = |this: &mut Self| -> ClearFaultLegacyInnerError {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return ClearFaultLegacyInnerError::Safety(SafetyError::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    });
                }
            }
            ClearFaultLegacyInnerError::Safety(SafetyError::EStopActive)
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
                .and_then(|d| d.with_ref(|drv| drv.joint_mode().joint_modes_uncertain()))
                .unwrap_or(false);
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
        pre_ctx.j7_feedback_fresh = true;
        pre_ctx.tx_queue_drained = true;
        pre_ctx.tx_probe_confirmed = true;
        pre_ctx.thermal_sensor_available = true;
        pre_ctx.thermal_below_threshold = true;
        pre_ctx.thermal_cooldown_elapsed_ms = pre_ctx.thermal_cooldown_required_ms;
        pre_ctx.joint_limit_violation = None;
        pre_ctx.enable_state_known = true;
        if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
            if refusal == ClearRefusal::EStopActive {
                return Err(apply_estop_refusal(self));
            }
            return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
        }
        if !self.safety.state().is_stopped() && !self.safety.has_latched_fault() {
            let has_motor_errors = self.has_reported_motor_errors()
                || self
                    .driver
                    .as_ref()
                    .and_then(|d| d.read().map(|drv| drv.has_hardware_fault()))
                    .unwrap_or(false);
            if has_motor_errors {
                let mut wall_now = Instant::now;
                let state = self
                    .clear_carried_over_motor_errors_inner(
                        &mut wall_now,
                        None,
                        expected_epoch,
                        authority,
                        None,
                    )
                    .map_err(|e| match e {
                        ClearFaultWithAuthError::Authority(a) => {
                            ClearFaultLegacyInnerError::Authority(a)
                        }
                        ClearFaultWithAuthError::Clear(c) => {
                            ClearFaultLegacyInnerError::Safety(SafetyError::from(c))
                        }
                    })?;
                commit_probed_limits(self);
                return Ok(state);
            }
            commit_probed_limits(self);
            return Ok(self.safety.state().clone());
        }

        if self.mit.owner == crate::mit::MitOwner::Lost {
            // Do NOT acknowledge the MIT fence trip or mutate supervisor state before the full
            // `clear_check` passes: `mit_fresh_status()` enforces the quiet period directly while
            // leaving `mit_fence_acked == false` so autonomous CAN refreshes remain inhibited.
            let fresh = match self.driver.as_mut().and_then(|d| d.with_mut(|drv| drv.mit_mut().mit_fresh_status())) {
                Some(r) => r,
                None => Err("no driver".into()),
            };
            let st = fresh.map_err(|e| SafetyError::ClearFaultRejected(format!(
                "MIT tracking: fresh feedback is required before clearing ({e}); nothing was sent")))?;
            if st.iter().any(|&c| c >= 8) || (st.contains(&0) && st.contains(&1)) {
                // Evaluate the full MIT recovery `clear_check` read-only from the fresh status
                // snapshot — ZERO fence ack and ZERO supervisor state mutation until `clear_check`
                // and operator confirmation both pass.
                let dof = self.config.dof;
                let max_temp = self.config.max_temperature;
                let thermal_cooldown = self.thermal_cooldown;
                let snapshot = self.driver.as_mut().and_then(|d| {
                    d.with_mut(|drv| {
                        let states = drv.read_joint_states().ok()?;
                        let ages = drv.feedback_ages();
                        let loads = drv.joint_loads();
                        let caps = drv.capabilities();
                        let modes_uncertain = drv.joint_mode().joint_modes_uncertain();
                        let tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
                        let tx_probe_confirmed = drv.tx().tx_probe_confirmed();
                        let tx_probe_sent = drv.tx().tx_probe_pending();
                        let fb_fresh = drv.is_feedback_valid()
                            && ages.len() == dof
                            && ages.iter().all(|a| a.is_some_and(|t| t <= THERMAL_FEEDBACK_FRESHNESS_LIMIT))
                            && Self::states_valid(&states, caps, dof, loads.as_deref());
                        let j7_fb_fresh = drv.pos_force().pos_force_feedback_fresh();
                        let threshold = max_temp - THERMAL_CLEAR_MARGIN_C;
                        let thermal_below = fb_fresh
                            && caps.temperature_feedback
                            && !states.is_empty()
                            && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                        let mut rx_buf = Vec::new();
                        let next_cd = Self::evaluate_thermal_cooldown_fields(
                            dof,
                            thermal_cooldown,
                            drv,
                            &ages,
                            thermal_below,
                            &mut rx_buf,
                        );
                        Some((
                            states,
                            caps.temperature_feedback,
                            fb_fresh,
                            j7_fb_fresh,
                            tx_queue_drained,
                            tx_probe_confirmed,
                            tx_probe_sent,
                            thermal_below,
                            next_cd,
                            modes_uncertain,
                        ))
                    })
                    .flatten()
                });
                let mut mit_ctx = self.build_clear_context_base(driver_err_override);
                if clear_driver_err_field && driver_err_override.is_none() {
                    mit_ctx.driver_config_error = None;
                }
                mit_ctx.expected_epoch = expected_epoch;
                mit_ctx.pending_estop = self.estop_request_unapplied();
                if let Some((ref states, temp_fb, fb_fresh, j7_fb_fresh, tx_queue_drained, tx_probe_confirmed, tx_probe_sent, thermal_below, next_cd, modes_uncertain)) = snapshot {
                    mit_ctx.mit_modes_uncertain = modes_uncertain;
                    mit_ctx.thermal_sensor_available = temp_fb;
                    mit_ctx.feedback_fresh = fb_fresh;
                    mit_ctx.j7_feedback_fresh = j7_fb_fresh;
                    mit_ctx.tx_queue_drained = tx_queue_drained;
                    mit_ctx.tx_probe_confirmed = tx_probe_confirmed;
                    mit_ctx.tx_probe_sent = tx_probe_sent;
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
                    mit_ctx.j7_feedback_fresh = false;
                    mit_ctx.tx_queue_drained = false;
                    mit_ctx.tx_probe_confirmed = false;
                    mit_ctx.tx_probe_sent = false;
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
                    return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                }
                // MIT expiry recovery disables every motor on the bus and therefore always passes
                // the single Rule A′ rest-pose decision with no override or already-off bypass.
                self.eval_stopped_torque_off()
                    .map_err(|r| r.into_safety_error(&self.config.joint_names))?;
                if !self.mit.recovery_confirmed {
                    return Err(ClearFaultLegacyInnerError::Safety(SafetyError::ClearFaultRejected(format!(
                        "{}: {}",
                        crate::mit::RECOVERY_CONFIRM_CODE,
                        crate::mit::RECOVERY_WARNING
                    ))));
                }
                // Keep operator confirmation, feedback, cooldown, and probed limits staged through
                // `mit_expiry_recovery_for_epoch`'s final `clear_with_check` decision (fail-closed:
                // zero premature commits if pre- or post-recovery clear_check refuses).
                let mit_commit = ClearFaultCommit {
                    candidate_limits,
                    reapply_legacy_limits: clear_driver_err_field && candidate_limits.is_none(),
                    is_torqued: None,
                    reset_disarmed_hold: false,
                    thermal_cooldown: None,
                };
                if let Some(auth) = authority {
                    recheck_clear_fault_authority_readonly(self, auth, Instant::now())
                        .map_err(ClearFaultLegacyInnerError::Authority)?;
                }
                return self.mit_expiry_recovery_for_epoch(
                    expected_epoch,
                    mit_ctx,
                    mit_commit,
                    driver_err_override,
                    authority,
                );
            }
        }

        let mut ctx;
        let mut commit = ClearFaultCommit {
            candidate_limits,
            reapply_legacy_limits: clear_driver_err_field && candidate_limits.is_none(),
            is_torqued: None,
            reset_disarmed_hold: false,
            thermal_cooldown: None,
        };
        let mut staged_feedback = None;
        if self.driver.is_some() {
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let dof = self.config.dof;
            let arm_dof = self.config.arm_dof();
            let max_temp = self.config.max_temperature;
            let max_rest_age = self
                .resting_policy()
                .map(|p| Duration::from_millis(p.feedback_max_age_ms))
                .unwrap_or(Duration::from_millis(250));
            let is_torqued = self.is_torqued;
            let torque_intent = self.torque_intent;
            let thermal_cooldown = self.thermal_cooldown;
            let fallback_loads = &self.loads;
            let (
                states,
                ages,
                loads,
                caps,
                hw_fault,
                mut j7_fb_fresh,
                tx_queue_drained,
                tx_probe_confirmed,
                tx_probe_sent,
                fb_fresh,
                needs_resting_check,
                resting_fb_ok,
                staged_positions,
                staged_velocities,
                staged_torques,
                staged_loads_vec,
                thermal_below,
                next_cd,
            ) = {
                let Some(ref mut d) = self.driver else { unreachable!() };
                let Some(mut drv) = d.write() else {
                    self.ws_eval_bufs = ws_bufs;
                    return Err(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable));
                };
                // Pure read-only telemetry & status snapshot FIRST — ZERO motor resets or side effects.
                let states = match drv.read_joint_states() {
                    Ok(s) => s,
                    Err(e) => {
                        drop(drv);
                        self.ws_eval_bufs = ws_bufs;
                        return Err(ClearFaultLegacyInnerError::Safety(SafetyError::ClearFaultRejected(e.to_string())));
                    }
                };
                let ages = drv.feedback_ages();
                let loads = drv.joint_loads();
                let caps = drv.capabilities();
                let disabling = drv.clear_fault_disables_joints();
                let hw_fault = drv.check_hardware_status().is_err();
                let fb_valid = drv.is_feedback_valid();
                let j7_fb_fresh = drv.pos_force().pos_force_feedback_fresh();
                let tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
                let tx_probe_confirmed = drv.tx().tx_probe_confirmed();
                let tx_probe_sent = drv.tx().tx_probe_pending();
                let states_ok = Self::states_valid(&states, caps, dof, loads.as_deref());
                let fb_fresh = fb_valid
                    && ages.len() == dof
                    && !ages.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)))
                    && states_ok;
                // M1: Resting-pose check necessity follows owner decision state (`torque_intent == On`
                // or `is_torqued`), never feedback-reported `enable_state()`.
                let needs_resting_check = (is_torqued || torque_intent == TorqueIntent::On)
                    && disabling.iter().any(|&j| j < arm_dof);
                let resting_fb_ok = fb_valid
                    && ages.len() == dof
                    && ages.iter().all(|a| a.is_some_and(|t| t <= max_rest_age))
                    && states_ok;

                // Stage sampled feedback and thermal cooldown WITHOUT mutating supervisor state
                // before `clear_check` / `clear_with_check` succeeds.
                let staged_positions: Vec<f32> = states.iter().take(dof).map(|s| s.position).collect();
                let staged_velocities: Vec<f32> = states.iter().take(dof).map(|s| s.velocity).collect();
                let staged_torques: Vec<f32> = states.iter().take(dof).map(|s| s.torque).collect();
                let staged_loads_vec: Vec<f32> = match loads.as_deref() {
                    Some(ls) => ls.iter().take(dof).copied().collect(),
                    None => fallback_loads.clone(),
                };
                let threshold = max_temp - THERMAL_CLEAR_MARGIN_C;
                let thermal_below = fb_fresh
                    && caps.temperature_feedback
                    && !states.is_empty()
                    && states.iter().all(|s| s.temperature.is_finite() && s.temperature <= threshold);
                // Cooldown accrues ONLY across consecutive fresh cool samples; stale/unavailable/gapped temp resets it.
                let mut rx_buf = Vec::new();
                let next_cd = Self::evaluate_thermal_cooldown_fields(
                    dof,
                    thermal_cooldown,
                    drv.as_ref(),
                    &ages,
                    thermal_below,
                    &mut rx_buf,
                );
                (
                    states,
                    ages,
                    loads,
                    caps,
                    hw_fault,
                    j7_fb_fresh,
                    tx_queue_drained,
                    tx_probe_confirmed,
                    tx_probe_sent,
                    fb_fresh,
                    needs_resting_check,
                    resting_fb_ok,
                    staged_positions,
                    staged_velocities,
                    staged_torques,
                    staged_loads_vec,
                    thermal_below,
                    next_cd,
                )
            };

            let mut resting_feedback_missing = false;
            let mut resting_pose_violation = None;
            if needs_resting_check {
                if !resting_fb_ok {
                    resting_feedback_missing = true;
                } else {
                    resting_pose_violation = self.resting_pose_support_violation_for_feedback_caps(
                        Some(caps),
                        &ages,
                        &staged_positions,
                        &staged_velocities,
                        &staged_torques,
                        &staged_loads_vec,
                        true,
                        &mut ws_bufs,
                    );
                }
            }
            self.ws_eval_bufs = ws_bufs;

            ctx = self.build_clear_context_base(driver_err_override);
            if clear_driver_err_field && driver_err_override.is_none() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.feedback_fresh = fb_fresh;
            ctx.j7_feedback_fresh = j7_fb_fresh;
            ctx.tx_queue_drained = tx_queue_drained;
            ctx.tx_probe_confirmed = tx_probe_confirmed;
            ctx.tx_probe_sent = tx_probe_sent;
            ctx.thermal_sensor_available = caps.temperature_feedback;
            ctx.thermal_below_threshold = thermal_below;
            ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
            ctx.joint_limit_violation = if fb_fresh {
                self.joint_limit_clear_violation_for_positions(&staged_positions, candidate_limits.as_ref())
            } else {
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref())
            };
            ctx.resting_feedback_missing = resting_feedback_missing;
            ctx.resting_pose_violation = resting_pose_violation;
            ctx.hardware_reset_required = hw_fault;
            self.populate_enable_clear_context(&mut ctx, caps.enable_feedback);

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any TX probe or motor
            // reset side effect.
            let mut clear_res = self.safety.clear_check(&ctx);
            if clear_res == Err(ClearRefusal::TxProbeRequired) {
                // Every OTHER recorded kind's condition passed and the software TX queue is empty:
                // run one read-only refresh/status TX probe and verify fresh post-probe feedback.
                let (probe_res, drained_after, probe_ok, probe_pending, hw_fault_after, fb_valid_after, j7_fresh_after, ages_after) =
                    self.with_driver_and_owner_ref(|owner, drv| {
                        if let Some(auth) = authority {
                            recheck_clear_fault_authority_readonly(owner, auth, Instant::now())?;
                        }
                        let res = drv.tx_mut().probe_tx_ready();
                        Ok((
                            res,
                            drv.tx().pending_tx_len_total() == 0,
                            drv.tx().tx_probe_confirmed(),
                            drv.tx().tx_probe_pending(),
                            drv.check_hardware_status().is_err(),
                            drv.is_feedback_valid(),
                            drv.pos_force().pos_force_feedback_fresh(),
                            drv.feedback_ages(),
                        ))
                    })
                    .ok_or(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable))?
                    .map_err(ClearFaultLegacyInnerError::Authority)?;
                j7_fb_fresh = j7_fresh_after;
                ctx.tx_queue_drained = drained_after;
                self.populate_enable_clear_context(&mut ctx, caps.enable_feedback);
                ctx.hardware_reset_required = hw_fault_after;
                ctx.j7_feedback_fresh = j7_fb_fresh;
                ctx.pending_estop = self.estop_request_unapplied();
                match probe_res {
                    Ok(confirmed) => {
                        ctx.tx_probe_sent = true;
                        ctx.tx_probe_confirmed = confirmed && probe_ok;
                        let fresh_after = fb_valid_after
                            && ages_after.len() == self.config.dof
                            && !ages_after.iter().any(|a| a.is_none_or(|t| t > Duration::from_millis(250)));
                        ctx.feedback_fresh = fb_fresh && fresh_after;
                    }
                    Err(e) => {
                        ctx.tx_probe_confirmed = false;
                        ctx.tx_probe_sent = probe_pending;
                        ctx.hardware_error = Some(clear_fault_driver_error(&e));
                    }
                }
                clear_res = self.safety.clear_check(&ctx);
            }
            match clear_res {
                Ok(()) => {}
                Err(ClearRefusal::EStopActive) => return Err(apply_estop_refusal(self)),
                Err(ClearRefusal::HardwareResetRequired) => {
                    // Explicitly modeled exception: every OTHER recorded kind's condition passed,
                    // so transmitting the motor fault reset is now permitted. Recheck live grant,
                    // lease, operator session (M3), J7 freshness, Rule A′ rest-pose gate (when the
                    // reset itself disables an arm joint, e.g. Feetech STS3215), and pending E-stop
                    // under the SAME driver lock guard / exclusive borrow before sending the reset (zero gap).
                    #[allow(clippy::large_enum_variant)]
                    enum ResetAttempt {
                        Ran(Result<(), crate::driver::ArmDriverError>),
                        Refusal(ClearRefusal),
                        StoppedTorqueOff(crate::supervisor::StoppedTorqueOffRefusal),
                        Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                    }
                    let reset_outcome = self
                        .with_driver_and_owner_ref(|owner, drv| {
                            if let Some(auth) = authority {
                                if let Err(auth_err) =
                                    recheck_clear_fault_authority_readonly(owner, auth, Instant::now())
                                {
                                    return ResetAttempt::Authority(auth_err);
                                }
                            }
                            if needs_resting_check {
                                if let Err(r) =
                                    Self::check_stopped_torque_off_with_driver(&owner.config, &*drv)
                                {
                                    return ResetAttempt::StoppedTorqueOff(r);
                                }
                            }
                            ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                            ctx.feedback_fresh = ctx.feedback_fresh && drv.is_feedback_valid();
                            ctx.pending_estop = owner.estop_request_unapplied();
                            if let Err(refusal) = owner.safety.clear_check(&ctx) {
                                if refusal != ClearRefusal::HardwareResetRequired {
                                    return ResetAttempt::Refusal(refusal);
                                }
                            }
                            ResetAttempt::Ran(drv.clear_fault())
                        })
                        .ok_or(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable))?;
                    let reset_res = match reset_outcome {
                        ResetAttempt::Ran(res) => res,
                        ResetAttempt::Refusal(ClearRefusal::EStopActive) => {
                            return Err(apply_estop_refusal(self));
                        }
                        ResetAttempt::Refusal(refusal) => {
                            return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                        }
                        ResetAttempt::StoppedTorqueOff(r) => {
                            self.last_stopped_torque_off_refusal = Some(r);
                            return Err(ClearFaultLegacyInnerError::Safety(
                                r.into_safety_error(&self.config.joint_names),
                            ));
                        }
                        ResetAttempt::Authority(e) => {
                            return Err(ClearFaultLegacyInnerError::Authority(e));
                        }
                    };
                    ctx.hardware_reset_required = false;
                    ctx.pending_estop = self.estop_request_unapplied();
                    match reset_res {
                        Ok(()) => ctx.hardware_reset_sent = true,
                        Err(
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                            | crate::driver::ArmDriverError::DisableRequestExpired,
                        ) => {
                            let r = self
                                .may_disable_torque_while_stopped()
                                .err()
                                .unwrap_or(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                            self.last_stopped_torque_off_refusal = Some(r);
                            return Err(ClearFaultLegacyInnerError::Safety(
                                r.into_safety_error(&self.config.joint_names),
                            ));
                        }
                        Err(e) => ctx.hardware_error = Some(clear_fault_driver_error(&e)),
                    }
                    let post_refusal = self.safety.clear_check(&ctx).unwrap_err();
                    if post_refusal == ClearRefusal::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(ClearFaultLegacyInnerError::Safety(post_refusal.into_safety_error()));
                }
                Err(refusal) => {
                    if matches!(refusal, ClearRefusal::ThermalCooldownNotElapsed { .. })
                        && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                            || ctx.j7_feedback_fresh)
                    {
                        self.thermal_cooldown = next_cd;
                    }
                    return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                }
            }
            if fb_fresh {
                staged_feedback = Some((states, loads, ages));
            }
            if self.torque_intent == TorqueIntent::Off {
                commit.is_torqued = Some(false);
                commit.reset_disarmed_hold = true;
            } else if self.enable_lifecycle == EnableLifecycle::FailedHold {
                commit.is_torqued = Some(true);
            }
            commit.thermal_cooldown = Some(next_cd);
        } else {
            let next_cd = self.evaluate_thermal_cooldown_driverless(Instant::now());
            ctx = self.build_clear_context_base(driver_err_override);
            if clear_driver_err_field && driver_err_override.is_none() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.thermal_cooldown_elapsed_ms = if ctx.thermal_below_threshold {
                next_cd.elapsed_ms()
            } else {
                0
            };
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == ClearRefusal::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                if matches!(refusal, ClearRefusal::ThermalCooldownNotElapsed { .. })
                    && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                        || ctx.j7_feedback_fresh)
                {
                    self.thermal_cooldown = next_cd;
                }
                return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
            }
            commit.thermal_cooldown = Some(next_cd);
        }
        self.clear_fault_finish_bounded_with_authority(&ctx, commit, authority, None, move |this| {
            if let Some((states, loads, ages)) = staged_feedback {
                this.feedback_numeric_valid = true;
                this.last_can_feedback_instant = ages
                    .iter()
                    .copied()
                    .try_fold(Duration::ZERO, |m, a| a.map(|t| m.max(t)))
                    .and_then(|age| Instant::now().checked_sub(age));
                if let Some(ref d) = this.driver {
                    let _ = d.with_ref(|drv| {
                        this.core.poll_feedback(
                            &mut this.joint_feedback_queue_bounds,
                            this.config.dof,
                            drv,
                            &mut this.scratch.fb,
                        );
                    });
                }
                Self::apply_feedback(
                    &mut this.measured_positions,
                    &mut this.measured_velocities,
                    &mut this.torques,
                    &mut this.temperatures,
                    &mut this.loads,
                    this.config.dof,
                    &states,
                    loads.as_deref(),
                );
            }
        })
        .map_err(|e| match e {
            ClearFaultWithAuthError::Authority(a) => ClearFaultLegacyInnerError::Authority(a),
            ClearFaultWithAuthError::Clear(c) => ClearFaultLegacyInnerError::Safety(SafetyError::from(c)),
        })
    }

    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    fn clear_fault_finish_bounded_with_authority(
        &mut self,
        ctx: &crate::safety::ClearContext,
        commit: ClearFaultCommit,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
        now_override: Option<Instant>,
        on_committed: impl FnOnce(&mut Self),
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly;
        use crate::safety::{ClearFaultError, ClearRefusal, FaultKind};
        if self.internal_fault_lane.is_fenced() || self.shutdown_committed {
            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::Rejected(ClearRefusal::ShutdownCommitted)));
        }
        // Build live pre-commit ClearContext right before committing so any concurrent E-stop,
        // authority revocation (M3), or J7 freshness expiry between preflight/probe and commit
        // fails closed with ZERO mutations.
        let mut final_ctx = *ctx;
        let was_estop = *self.safety.state() == SafetyState::EStop;
        let state = if let Some(ref d) = self.driver {
            let drv = match d.read() {
                Some(g) => g,
                None => {
                    final_ctx.j7_feedback_fresh = false;
                    final_ctx.feedback_fresh = false;
                    final_ctx.pending_estop = self.estop_request_unapplied();
                    return Err(ClearFaultWithAuthError::Clear(
                        self.safety
                            .clear_check(&final_ctx)
                            .err()
                            .map_or(ClearFaultError::FeedbackUnavailable, ClearFaultError::from),
                    ));
                }
            };
            if let Some(auth) = authority {
                let check_now = now_override.unwrap_or_else(Instant::now);
                if let Err(auth_err) = recheck_clear_fault_authority_readonly(self, auth, check_now) {
                    drop(drv);
                    return Err(ClearFaultWithAuthError::Authority(auth_err));
                }
            }
            final_ctx.pending_estop = self.estop_request_unapplied();
            if self.safety.has_latched_fault_kind(FaultKind::J7FeedbackStale) {
                final_ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                final_ctx.feedback_fresh = final_ctx.feedback_fresh && drv.is_feedback_valid();
            }
            if let Err(refusal) = self.safety.clear_check(&final_ctx) {
                drop(drv);
                if refusal == ClearRefusal::EStopActive {
                    self.consume_pending_estop();
                    if let Some(received) = final_ctx.expected_epoch {
                        if received != self.safety.epoch_id() {
                            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                                expected: self.safety.epoch_id(),
                                received,
                            }));
                        }
                    }
                    return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::from(refusal)));
            }
            if self.estop_request_unapplied() {
                drop(drv);
                self.consume_pending_estop();
                if let Some(received) = final_ctx.expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        }));
                    }
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
            }
            // Commit the safety clear under the SAME driver lock guard before applying staged
            // limits, torque flag, or cleanup (zero check-to-commit gap).
            self.safety
                .clear_with_check(&final_ctx)
                .map_err(|r| ClearFaultWithAuthError::Clear(ClearFaultError::from(r)))?
        } else {
            if let Some(auth) = authority {
                let check_now = now_override.unwrap_or_else(Instant::now);
                recheck_clear_fault_authority_readonly(self, auth, check_now)
                    .map_err(ClearFaultWithAuthError::Authority)?;
            }
            final_ctx.pending_estop = self.estop_request_unapplied();
            if self.safety.has_latched_fault_kind(FaultKind::J7FeedbackStale) {
                final_ctx.j7_feedback_fresh = false;
            }
            if let Err(refusal) = self.safety.clear_check(&final_ctx) {
                if refusal == ClearRefusal::EStopActive {
                    self.consume_pending_estop();
                    if let Some(received) = final_ctx.expected_epoch {
                        if received != self.safety.epoch_id() {
                            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                                expected: self.safety.epoch_id(),
                                received,
                            }));
                        }
                    }
                    return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::from(refusal)));
            }
            if self.consume_pending_estop() {
                if let Some(received) = final_ctx.expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        }));
                    }
                }
                return Err(ClearFaultWithAuthError::Clear(ClearFaultError::EStopActive));
            }
            self.safety
                .clear_with_check(&final_ctx)
                .map_err(|r| ClearFaultWithAuthError::Clear(ClearFaultError::from(r)))?
        };

        let clear_estop_gen = self.applied_estop_count;
        on_committed(self);
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.torque_shadow
                .enqueue(crate::torque::shadow::ShadowInputKind::FaultClear {
                    live_refusal: None,
                    live_intent: self.shadow_live_intent(),
                    live_is_torqued: self.is_torqued,
                    live_stopped: self.safety.state().is_stopped(),
                });
            if !self.rt_executor_owned {
                self.flush_torque_shadow();
            }
        }
        if let Some(ref cand) = commit.candidate_limits {
            self.commit_driver_limits_bounded(cand);
        } else if commit.reapply_legacy_limits {
            self.reapply_driver_limits();
        }
        if let Some(is_torqued) = commit.is_torqued {
            self.is_torqued = is_torqued;
        }
        if commit.reset_disarmed_hold {
            self.enable_lifecycle = EnableLifecycle::Disarmed;
            self.measured_hold.clear();
        }
        if let Some(cd) = commit.thermal_cooldown {
            self.thermal_cooldown = cd;
        }
        let was_mit_lost = self.mit.owner == crate::mit::MitOwner::Lost;
        self.pending_rearm = true;
        self.stop_motion();
        self.servo_stream_active = false;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.reset_tx_joint_progress();
        self.reset_hold_tx_joint_progress();
        self.hold_tx_stalled_latched = false;
        self.last_hold_tx_stall_log = None;
        self.j7_stale_since = None;
        // Preserve `self.last_hold_tx_error_log` across fault clears so rapid clear/re-fault
        // cycles cannot bypass the 1/s stopped-hold error log rate limit.
        self.pending_deferred_done = None;
        let mut synced_status: Option<(EnableState, [Option<&'static str>; crate::rt_core::mailbox::MAX_DOF])> = None;
        let mut scratch_status = std::mem::take(&mut self.scratch.status);
        if let Some(ref mut d) = self.driver {
            let _ = d.with_mut(|drv| {
                drv.tx_mut().discard_pending_tx();
                drv.tx_mut().clear_tx_probe();
                if drv.motor_status_into(&mut scratch_status) {
                    let mut names = [None; crate::rt_core::mailbox::MAX_DOF];
                    for (slot, &code) in names.iter_mut().zip(scratch_status.iter()) {
                        *slot = drv.decode_motor_error_static(code);
                    }
                    synced_status = Some((drv.enable_state(), names));
                }
            });
        }
        if let Some((en_state, names)) = synced_status {
            self.apply_synced_motor_status_bounded(&scratch_status, &names, en_state);
        }
        self.scratch.status = scratch_status;
        // Reset at the measured position; never retain a residual old trajectory target.
        copy_into(&mut self.commanded_positions, &self.measured_positions);
        copy_into(&mut self.target_positions, &self.measured_positions);
        self.commanded_velocities.fill(0.0);
        self.last_commanded_velocities.fill(0.0);
        self.commanded_accelerations.fill(0.0);
        self.following_error_since = None;
        self.watchdog_park_blocked_reason = None;
        self.bus_loss_instant = None;
        self.output_suspended = false;
        self.reset_gripper_stall();
        self.handover_settling_since = None;
        if let Some(hook) = self.before_estop_unlatch_hook.take() {
            hook();
        }
        if was_mit_lost {
            // Only after full clear_with_check passed: release MIT lost ownership and acknowledge
            // the driver MIT fence BEFORE any E-stop-latch early return so a cleared MIT fault
            // never re-latches `MIT_LOST` over `SafetyState::EStop` on the next control tick.
            self.mit.owner = crate::mit::MitOwner::Off;
            self.mit.joints.clear();
            self.mit.last_ff.clear();
            self.mit.last_output = None;
            self.mit.armed_ok = false;
            self.mit.revoke_pending = false;
            self.mit.recovery_confirmed = false;
            if let Some(ref mut d) = self.driver {
                if self.rt_executor_owned {
                    let _ = d.try_with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                } else {
                    let _ = d.with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                }
            }
        }
        if !was_estop {
            // Clearing a non-E-stop fault must never overwrite a prior or concurrent E-stop latch.
            if self.consume_pending_estop() || self.estop_state.is_latched() {
                if self.safety.state() != &SafetyState::EStop {
                    self.apply_stop_policy(StopPolicy::EStopLatch);
                }
                self.last_heartbeat = Instant::now();
                self.last_motion_frame = Instant::now();
                return Ok(self.safety.state().clone());
            }
            self.shutdown_refused_unknown = false;
        } else {
            // Atomically clear bit 0 ONLY if no newer E-stop has advanced `generation` past
            // `clear_estop_gen`. A concurrent newer E-stop causes CAS failure without ever
            // transiently clearing the latch bit.
            match self.estop_state.try_clear_if_generation(clear_estop_gen) {
                Ok(()) => {
                    self.shutdown_refused_unknown = false;
                    if self.consume_pending_estop() {
                        // A concurrent E-stop arrived strictly AFTER the CAS unlatched the prior
                        // E-stop; keep the newly applied E-stop state and return Ok(EStop).
                        self.last_heartbeat = Instant::now();
                        self.last_motion_frame = Instant::now();
                        return Ok(self.safety.state().clone());
                    }
                }
                Err(_newer_gen) => {
                    // A newer E-stop arrived before unlatch: latch bit stayed true throughout
                    // (zero transient-false window). Apply the newer E-stop immediately.
                    let _ = self.consume_pending_estop();
                    if self.safety.state() != &SafetyState::EStop {
                        self.apply_stop_policy(StopPolicy::EStopLatch);
                    }
                    self.last_heartbeat = Instant::now();
                    self.last_motion_frame = Instant::now();
                    return Ok(self.safety.state().clone());
                }
            }
        }
        // If a valid lease was acquired while E-stop was latched, its servo stream could not be
        // authorized until E-stop cleared; authorize it now when no policy stream owns targets.
        if self.core.target_authority().stream.is_none() {
            let check_now = now_override.unwrap_or_else(Instant::now);
            let active_gen = self.active_lease_gen_at(check_now);
            if active_gen != 0 {
                self.authorize_servo_stream_for_committed_lease(active_gen);
            }
        }
        self.last_heartbeat = Instant::now();
        self.last_motion_frame = Instant::now();
        Ok(state)
    }

    pub(super) fn measured_workspace_violation_bounded(
        &self,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::WorkspaceSupportViolation> {
        self.workspace_violation_for_positions_bounded(&self.measured_positions, bufs)
    }

    pub(super) fn workspace_violation_for_positions_bounded(
        &self,
        positions: &[f32],
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::WorkspaceSupportViolation> {
        use crate::safety::{BoundedText, WorkspaceSupportViolation};
        let (ws, cam) = self.workspace_policy()?;
        if self.config.arm_model.frames_f32(positions).is_none() {
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
        let violation = match eval.eval_into_in(positions, &mut buf, &mut bufs.world_caps, "clear_fault") {
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
        self.resting_pose_support_violation_for_feedback(
            drv,
            ages,
            &self.measured_positions,
            &self.measured_velocities,
            &self.torques,
            &self.loads,
            self.feedback_numeric_valid,
            bufs,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn resting_pose_support_violation_for_feedback(
        &self,
        drv: Option<&dyn ArmDriver>,
        ages: &[Option<Duration>],
        positions: &[f32],
        velocities: &[f32],
        torques: &[f32],
        loads: &[f32],
        feedback_numeric_valid: bool,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::RestingSupportViolation> {
        self.resting_pose_support_violation_for_feedback_caps(
            drv.map(ArmDriver::capabilities),
            ages,
            positions,
            velocities,
            torques,
            loads,
            feedback_numeric_valid,
            bufs,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn resting_pose_support_violation_for_feedback_caps(
        &self,
        drv_caps: Option<DriverCapabilities>,
        ages: &[Option<Duration>],
        positions: &[f32],
        velocities: &[f32],
        torques: &[f32],
        loads: &[f32],
        feedback_numeric_valid: bool,
        bufs: &mut crate::chain::EvalBufs,
    ) -> Option<crate::safety::RestingSupportViolation> {
        use crate::safety::{RestingSupportJointItem as Item, RestingSupportViolation as V, MAX_RESTING_SUPPORT_ITEMS};
        let dof = self.config.dof;
        if positions.len() != dof || velocities.len() != dof {
            return Some(V::FeedbackLengthMismatch);
        }
        let Some(policy) = self.config.resting_pose.as_ref() else { return Some(V::NoRestingPoseConfigured); };
        if self.driver.is_some() {
            let Some(caps) = drv_caps else { return Some(V::DriverFeedbackUnavailable); };
            if !caps.velocity_feedback { return Some(V::DriverLacksVelocityFeedback); }
            if !policy.support_torque_limits.is_empty() && !caps.torque_feedback { return Some(V::DriverLacksTorqueFeedback); }
            if !policy.support_load_limits.is_empty() && !caps.load_feedback { return Some(V::DriverLacksLoadFeedback); }
            let max_age = Duration::from_millis(policy.feedback_max_age_ms);
            if !feedback_numeric_valid || ages.len() != dof || !ages.iter().all(|a| a.is_some_and(|age| age <= max_age)) {
                return Some(V::FeedbackTooOld { max_age_ms: policy.feedback_max_age_ms });
            }
        }
        let mut items = [None; MAX_RESTING_SUPPORT_ITEMS];
        let mut len = 0usize;
        let mut overflowed = 0u16;
        for (i, &v) in velocities.iter().enumerate().take(dof) {
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
            let dev = (positions[i] - rest[i]).abs();
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
                let tau = torques.get(i).copied().unwrap_or(f32::NAN);
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
                let load = loads.get(i).copied().unwrap_or(f32::NAN);
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
        let mut workspace = self.workspace_violation_for_positions_bounded(positions, bufs);
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
        let mut wall_now = Instant::now;
        match self.clear_fault_executor_inner(&mut wall_now, deadline, expected_epoch, None, None, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_epoch")
            }
        }
    }

    /// Release-bounded, allocation-free fault clearing for Step 2b of an RT release: caps every
    /// TX (0xCC probe and motor fault reset) at `release_deadline - tx_release_publish_margin()`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub fn clear_fault_executor_bounded_for_release(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        expected_epoch: Option<u64>,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let mut live_now = || now.max(Instant::now());
        self.clear_fault_executor_bounded_for_release_with_clock(
            now,
            release_deadline,
            expected_epoch,
            &mut live_now,
        )
    }

    /// Like [`Self::clear_fault_executor_bounded_for_release`], threading a live owner clock
    /// (`now_fn`) into under-guard TX deadline translation so wait time or driver-clock sampling
    /// can never extend TX beyond `release_deadline - tx_release_publish_margin()`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(crate) fn clear_fault_executor_bounded_for_release_with_clock(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        expected_epoch: Option<u64>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let tx_deadline = release_deadline
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        match self.clear_fault_executor_inner(now_fn, tx_deadline, expected_epoch, None, None, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_release")
            }
        }
    }

    /// Release-ledger-threaded form of [`Self::clear_fault_executor_bounded_for_release_with_clock`]:
    /// enforces and debits the shared [`crate::rt_core::budget::ReleaseBudget`] across RX drain,
    /// `0xCC` TX probes, and motor fault resets so maintenance never regains TX/RX spent earlier
    /// in the same release.
    #[allow(clippy::result_large_err)]
    pub fn clear_fault_executor_bounded_for_release_with_budget(
        &mut self,
        now: Instant,
        release_budget: &mut crate::rt_core::budget::ReleaseBudget,
        expected_epoch: Option<u64>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, crate::safety::ClearFaultError> {
        let tx_deadline = release_budget
            .deadline()
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        match self.clear_fault_executor_inner(
            now_fn,
            tx_deadline,
            expected_epoch,
            None,
            Some(release_budget),
            None,
        ) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_release_with_budget")
            }
        }
    }

    /// Release-bounded, allocation-free fault clearing carrying M3 live owner authority through
    /// driver preflight, hardware reset (`drv.clear_fault_budgeted`), and final `safety.clear_with_check`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(crate) fn clear_fault_executor_bounded_for_release_with_authority_clock(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        authority: crate::rt_core::requests::lease::ClearFaultAuthority,
        mit_recovery_started_at: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        let tx_deadline = release_deadline
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        self.clear_fault_executor_inner(
            now_fn,
            tx_deadline,
            Some(authority.expected_epoch),
            Some(authority),
            None,
            mit_recovery_started_at,
        )
    }

    /// Release-ledger-threaded form of [`Self::clear_fault_executor_bounded_for_release_with_authority_clock`].
    #[allow(clippy::result_large_err)]
    pub(crate) fn clear_fault_executor_bounded_for_release_with_authority_budget(
        &mut self,
        now: Instant,
        release_budget: &mut crate::rt_core::budget::ReleaseBudget,
        authority: crate::rt_core::requests::lease::ClearFaultAuthority,
        mit_recovery_started_at: Option<Instant>,
        now_fn: &mut dyn FnMut() -> Instant,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        let tx_deadline = release_budget
            .deadline()
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        self.clear_fault_executor_inner(
            now_fn,
            tx_deadline,
            Some(authority.expected_epoch),
            Some(authority),
            Some(release_budget),
            mit_recovery_started_at,
        )
    }

    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    fn clear_fault_executor_inner(
        &mut self,
        now_fn: &mut dyn FnMut() -> Instant,
        deadline: Instant,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
        mut release_budget: Option<&mut crate::rt_core::budget::ReleaseBudget>,
        mit_recovery_started_at: Option<Instant>,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly;
        use crate::safety::{ClearFaultError as E, ClearRefusal as R};
        let apply_estop_refusal = |this: &mut Self| -> ClearFaultWithAuthError {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return ClearFaultWithAuthError::Clear(E::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    });
                }
            }
            ClearFaultWithAuthError::Clear(E::EStopActive)
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

        // Zero-allocation, release-bounded MIT expiry recovery on RT executor when `mit.owner == Lost`.
        // Enforces explicit confirmation (`mit_recovery == true`), Rule A′ rest-pose gate at send
        // time (`TxKind::DisableRequested { started_at }`), live authority + new-estop-since baseline
        // checks, physical disable sync on Done, and causal all-axis disabled feedback before
        // clearing `MitOwner::Lost` and the fault latch.
        #[cfg(not(target_arch = "wasm32"))]
        if self.mit.owner == crate::mit::MitOwner::Lost && is_stopped_or_latched {
            let mut pre_ctx = self.build_mit_recovery_clear_context();
            pre_ctx.driver_config_error = if candidate_limits.is_some() {
                None
            } else {
                driver_err_override.or(pre_ctx.driver_config_error)
            };
            pre_ctx.expected_epoch = expected_epoch;
            pre_ctx.pending_estop = self.estop_request_unapplied();
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            if let Some(drv) = self.driver.as_ref().and_then(|d| d.read()) {
                drv.feedback_ages_into(&mut scratch_ages);
                let ages_valid = scratch_ages.len() == self.config.dof
                    && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                let caps = drv.capabilities();
                pre_ctx.feedback_fresh = ages_valid && drv.is_feedback_valid();
                pre_ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                pre_ctx.thermal_sensor_available = caps.temperature_feedback;
                let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                let thermal_below = pre_ctx.feedback_fresh
                    && caps.temperature_feedback
                    && !self.temperatures.is_empty()
                    && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold);
                let next_cd = Self::evaluate_thermal_cooldown_fields(
                    self.config.dof,
                    self.thermal_cooldown,
                    drv.as_ref(),
                    &scratch_ages,
                    thermal_below,
                    &mut scratch_rx,
                );
                pre_ctx.thermal_below_threshold = thermal_below;
                pre_ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
                pre_ctx.enable_state_known = true;
                pre_ctx.joint_limit_violation =
                    self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            } else {
                pre_ctx.feedback_fresh = false;
                pre_ctx.j7_feedback_fresh = false;
                pre_ctx.enable_state_known = false;
            }
            self.scratch.ages = scratch_ages;
            self.scratch.rx = scratch_rx;

            if let Err(refusal) = self.safety.clear_check(&pre_ctx) {
                self.mit.recovery_baseline = None;
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
            }
            // Rule A′ rest-pose check BEFORE checking confirmation or issuing any CAN TX.
            let rest_check = {
                let drv_check = self
                    .driver
                    .as_ref()
                    .and_then(|d| d.read())
                    .map(|drv| Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref()));
                match drv_check {
                    Some(res) => res,
                    None => self.eval_stopped_torque_off(),
                }
            };
            if let Err(refusal) = rest_check {
                self.mit.recovery_baseline = None;
                self.last_stopped_torque_off_refusal = Some(refusal);
                return Err(ClearFaultWithAuthError::Clear(refusal.into_clear_fault_error()));
            }
            if !self.mit.recovery_confirmed {
                self.mit.recovery_baseline = None;
                return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                    R::MitRecoveryConfirmRequired,
                )));
            }
            if now_fn() >= deadline {
                self.mit.recovery_baseline = None;
                return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
            }
            // Run bounded, zero-alloc clear-error + Rule A′ DisableRequested + causal confirmation.
            #[allow(clippy::large_enum_variant)] // stack-only RT enum avoids heap allocation
            enum MitRecStep {
                Done,
                Pending,
                TorqueOffWouldDrop(crate::supervisor::StoppedTorqueOffRefusal),
                Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                WatchdogBudgetInsufficient { max_frames: u16, required: u16 },
                WatchdogContinuityMissed { joint: u8 },
                Refusal(R),
            }
            let attempt_started_at = mit_recovery_started_at.unwrap_or_else(&mut *now_fn);
            let is_new_attempt = self.mit.recovery_attempt_started_at != Some(attempt_started_at);
            self.mit.recovery_attempt_started_at = Some(attempt_started_at);
            let recovery_baseline = match self.mit.recovery_baseline {
                Some(b) if !is_new_attempt => b,
                _ => {
                    let b = self.capture_baseline();
                    self.mit.recovery_baseline = Some(b);
                    b
                }
            };
            let mit_budget_rem = release_budget
                .as_deref()
                .map(|b| (b.rx_remaining(), b.tx_remaining()));
            let mut mit_rx_delta = 0u16;
            let mut mit_tx_delta = 0u16;
            let rec_step_opt = self.with_driver_and_owner_ref(|owner, drv| {
                if is_new_attempt {
                    drv.mit_mut().mit_expiry_recover_reset();
                }
                if let Some(auth) = authority {
                    if let Err(auth_err) =
                        recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                    {
                        drv.mit_mut().mit_expiry_recover_reset();
                        return MitRecStep::Authority(auth_err);
                    }
                }
                if let Err(refusal) =
                    Self::check_stopped_torque_off_with_driver(&owner.config, &*drv)
                {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::TorqueOffWouldDrop(refusal);
                }
                if let Err((max_frames, required)) = drv.tx().check_watchdog_tx_budget_for_energized() {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::WatchdogBudgetInsufficient {
                        max_frames: max_frames.min(u16::MAX as usize) as u16,
                        required: required.min(u16::MAX as usize) as u16,
                    };
                }
                let Ok((drv_now, owner_now, drv_deadline)) =
                    crate::supervisor::translate_driver_cutoff_conservative(
                        drv, deadline, now_fn,
                    )
                else {
                    return MitRecStep::Refusal(R::BudgetExpired);
                };
                let elapsed_since_start =
                    owner_now.saturating_duration_since(attempt_started_at);
                let drv_started_at = drv_now
                    .checked_sub(elapsed_since_start)
                    .unwrap_or(drv_now);
                let mut auth_abort: Option<crate::rt_core::requests::lease::ClearFaultAuthorityError> = None;
                let mut estop_abort = false;
                let mut pre_tx_gate = || -> Result<(), &'static str> {
                    if owner.new_estop_since(recovery_baseline) {
                        estop_abort = true;
                        return Err("ESTOP_ACTIVE");
                    }
                    if let Some(auth) = authority {
                        if let Err(auth_err) =
                            recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                        {
                            auth_abort = Some(auth_err);
                            return Err("AUTH_REVOKED");
                        }
                    }
                    Ok(())
                };
                let before_budget = drv.tx().release_budget_consumed();
                if let Some((rx_rem, tx_rem)) = mit_budget_rem {
                    drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                }
                drv.set_tx_release_deadline(Some(drv_deadline));
                let r = drv.mit_mut().mit_expiry_recover_budgeted(
                    drv_deadline,
                    drv_started_at,
                    &mut pre_tx_gate,
                );
                drv.set_tx_release_deadline(None);
                let after_budget = drv.tx().release_budget_consumed();
                mit_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                mit_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                if let Some(auth_err) = auth_abort {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::Authority(auth_err);
                }
                if estop_abort {
                    drv.mit_mut().mit_expiry_recover_reset();
                    return MitRecStep::Refusal(R::EStopActive);
                }
                match r {
                    Ok(true) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::Done
                    }
                    Ok(false) => MitRecStep::Pending,
                    Err(crate::driver::ArmDriverError::DisableRequestExpired) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::Refusal(R::BudgetExpired)
                    }
                    Err(crate::driver::ArmDriverError::TorqueOffWouldDrop) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        let ref_err = Self::check_stopped_torque_off_with_driver(
                            &owner.config,
                            &*drv,
                        )
                        .err()
                        .unwrap_or(
                            crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback,
                        );
                        MitRecStep::TorqueOffWouldDrop(ref_err)
                    }
                    Err(crate::driver::ArmDriverError::WatchdogBudgetInsufficient {
                        max_frames,
                        required,
                    }) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::WatchdogBudgetInsufficient {
                            max_frames: max_frames.min(u16::MAX as usize) as u16,
                            required: required.min(u16::MAX as usize) as u16,
                        }
                    }
                    Err(crate::driver::ArmDriverError::WatchdogContinuityMissed { joint }) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::WatchdogContinuityMissed {
                            joint: joint.min(u8::MAX as usize) as u8,
                        }
                    }
                    Err(e) if crate::driver::is_estop_inhibited(&e) => {
                        drv.mit_mut().mit_expiry_recover_reset();
                        MitRecStep::Refusal(R::EStopActive)
                    }
                    Err(_) => MitRecStep::Pending,
                }
            });
            if let Some(b) = release_budget.as_deref_mut() {
                let rx_exhausted = mit_rx_delta > 0
                    && mit_budget_rem.is_some_and(|(rx_rem, _)| mit_rx_delta >= rx_rem);
                if mit_rx_delta > 0 || rx_exhausted {
                    b.charge_rx(mit_rx_delta, rx_exhausted);
                }
                if mit_tx_delta > 0 {
                    b.charge_tx(mit_tx_delta);
                }
            }
            let Some(rec_step) = rec_step_opt else {
                self.mit.recovery_baseline = None;
                return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
            };
            match rec_step {
                MitRecStep::Done => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    // Synchronize confirmed physical disable immediately after bus disable confirmation;
                    // logical fault clear and MIT-off state clear wait for final authority/clear_check below.
                    self.stop_motion();
                    self.servo_stream_active = false;
                    self.tracking.invalidate();
                    self.tracking.clear_recovery_suspension();
                    self.disable_accepted();
                    let _ = self.safety.set_torque_rt(false);
                    self.is_torqued = false;
                }
                MitRecStep::Pending => {
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::MitRecoveryIncomplete,
                    )));
                }
                MitRecStep::TorqueOffWouldDrop(r) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    self.last_stopped_torque_off_refusal = Some(r);
                    return Err(ClearFaultWithAuthError::Clear(r.into_clear_fault_error()));
                }
                MitRecStep::Authority(e) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(ClearFaultWithAuthError::Authority(e));
                }
                MitRecStep::WatchdogBudgetInsufficient {
                    max_frames,
                    required,
                } => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::WatchdogBudgetInsufficient {
                            max_frames,
                            required,
                        },
                    )));
                }
                MitRecStep::WatchdogContinuityMissed { joint } => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    #[cfg(not(target_arch = "wasm32"))]
                    if !crate::rt_core::executor::core_is_confirmed_disarmed(self) {
                        self.estop_state.request_and_latch();
                        self.internal_fault_lane.latch_control_safe_hold_fence();
                        self.sessions.request_safe_hold();
                    }
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::MitRecoveryWatchdogAbort { joint },
                    )));
                }
                MitRecStep::Refusal(R::EStopActive) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(apply_estop_refusal(self));
                }
                MitRecStep::Refusal(r) => {
                    self.mit.recovery_attempt_started_at = None;
                    self.mit.recovery_baseline = None;
                    return Err(ClearFaultWithAuthError::Clear(E::from(r)));
                }
            }
            // All motors causally confirmed disabled after req_boundary: rebuild the final clear
            // context directly from the fresh post-disable driver snapshot (0 RT allocs) so any
            // out-of-limit position, overtemperature sample, or carried hardware fault observed on
            // the disable confirmation gates the clear identically to legacy.
            let mut post_ctx = self.build_mit_recovery_clear_context();
            post_ctx.driver_config_error = if candidate_limits.is_some() {
                None
            } else {
                driver_err_override.or(post_ctx.driver_config_error)
            };
            post_ctx.expected_epoch = expected_epoch;
            post_ctx.pending_estop = self.estop_request_unapplied();
            post_ctx.hardware_reset_required = false;
            let mut post_cd = self.thermal_cooldown;
            let mut post_thermal_trip: Option<(usize, f32)> = None;
            let mut post_positions = [0.0f32; crate::model::MAX_MOTION_DOF];
            let mut post_pos_len = 0usize;
            let mut scratch_states = std::mem::take(&mut self.scratch.states);
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            if let Some(drv) = self.driver.as_ref().and_then(|d| d.read()) {
                drv.feedback_ages_into(&mut scratch_ages);
                let ages_valid = scratch_ages.len() == self.config.dof
                    && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                let caps = drv.capabilities();
                let cached_ok = drv.read_cached_joint_states_into(&mut scratch_states).is_some()
                    && scratch_states.len() >= self.config.dof;
                let states_finite = cached_ok
                    && scratch_states[..self.config.dof]
                        .iter()
                        .all(|s| s.position.is_finite() && (!caps.temperature_feedback || s.temperature.is_finite()));
                let fb_fresh = ages_valid && drv.is_feedback_valid() && (!cached_ok || states_finite);
                post_ctx.feedback_fresh = fb_fresh;
                post_ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                post_ctx.thermal_sensor_available = caps.temperature_feedback;
                post_ctx.enable_state_known = true;
                post_ctx.hardware_reset_required = drv.has_hardware_fault();
                post_ctx.hardware_error = None;
                let threshold = self.config.max_temperature - THERMAL_CLEAR_MARGIN_C;
                let thermal_below = if cached_ok {
                    if fb_fresh && caps.temperature_feedback {
                        post_thermal_trip = scratch_states[..self.config.dof]
                            .iter()
                            .map(|s| s.temperature)
                            .enumerate()
                            .filter(|(_, t)| t.is_finite() && *t > self.config.max_temperature)
                            .max_by(|a, b| a.1.total_cmp(&b.1));
                    }
                    fb_fresh
                        && caps.temperature_feedback
                        && self.config.dof > 0
                        && scratch_states[..self.config.dof]
                            .iter()
                            .all(|s| s.temperature.is_finite() && s.temperature <= threshold)
                } else {
                    fb_fresh
                        && caps.temperature_feedback
                        && !self.temperatures.is_empty()
                        && self.temperatures.iter().all(|t| t.is_finite() && *t <= threshold)
                };
                post_cd = Self::evaluate_thermal_cooldown_fields(
                    self.config.dof,
                    self.thermal_cooldown,
                    drv.as_ref(),
                    &scratch_ages,
                    thermal_below,
                    &mut scratch_rx,
                );
                post_ctx.thermal_below_threshold = thermal_below;
                post_ctx.thermal_cooldown_elapsed_ms = if thermal_below { post_cd.elapsed_ms() } else { 0 };
                if cached_ok && fb_fresh && self.config.dof <= crate::model::MAX_MOTION_DOF {
                    for (dst, s) in post_positions[..self.config.dof]
                        .iter_mut()
                        .zip(&scratch_states[..self.config.dof])
                    {
                        *dst = s.position;
                    }
                    post_pos_len = self.config.dof;
                }
            } else {
                post_ctx.feedback_fresh = false;
                post_ctx.j7_feedback_fresh = false;
                post_ctx.enable_state_known = false;
            }
            self.scratch.states = scratch_states;
            self.scratch.ages = scratch_ages;
            self.scratch.rx = scratch_rx;
            if let Some((joint_idx, temperature)) = post_thermal_trip {
                self.safety.latch(
                    crate::safety::FaultKind::ThermalTrip,
                    crate::safety::FaultDetail::ThermalTrip {
                        joint_idx,
                        temperature,
                        limit: self.config.max_temperature,
                    },
                );
            }
            post_ctx.joint_limit_violation = if post_pos_len == self.config.dof {
                self.joint_limit_clear_violation_for_positions(
                    &post_positions[..post_pos_len],
                    candidate_limits.as_ref(),
                )
            } else {
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref())
            };
            if self.estop_request_unapplied() {
                self.consume_pending_estop();
            }
            if self.e_stop_latched.load(std::sync::atomic::Ordering::SeqCst) {
                if let Some(auth) = authority {
                    recheck_clear_fault_authority_readonly(self, auth, now_fn())
                        .map_err(ClearFaultWithAuthError::Authority)?;
                }
                if let Some(received) = expected_epoch {
                    if received != self.safety.epoch_id() {
                        return Err(ClearFaultWithAuthError::Clear(E::EpochMismatch {
                            expected: self.safety.epoch_id(),
                            received,
                        }));
                    }
                }
                self.mit.owner = crate::mit::MitOwner::Off;
                self.mit.joints.clear();
                self.mit.last_ff.clear();
                self.mit.last_output = None;
                self.mit.armed_ok = false;
                self.mit.revoke_pending = false;
                self.mit.recovery_confirmed = false;
                if let Some(d) = self.driver.as_mut() {
                    let _ = d.try_with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                }
                let _ = self.safety.clear_with_check(&post_ctx);
                self.apply_stop_policy(crate::safety::StopPolicy::EStopLatch);
                return Err(ClearFaultWithAuthError::Clear(E::EStopActive));
            }
            let commit = ClearFaultCommit {
                candidate_limits,
                reapply_legacy_limits: false,
                is_torqued: Some(false),
                reset_disarmed_hold: true,
                thermal_cooldown: Some(post_cd),
            };
            let state = self.clear_fault_finish_bounded_with_authority(
                &post_ctx,
                commit,
                authority,
                Some(now_fn()),
                |this| {
                    this.mit.owner = crate::mit::MitOwner::Off;
                    this.mit.joints.clear();
                    this.mit.last_ff.clear();
                    this.mit.last_output = None;
                    this.mit.armed_ok = false;
                    this.mit.revoke_pending = false;
                    this.mit.recovery_confirmed = false;
                    if let Some(d) = this.driver.as_mut() {
                        let _ = d.try_with_mut(|drv| drv.mit_mut().acknowledge_mit_fence_trip());
                    }
                },
            )?;
            return Ok(state);
        }

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
                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
            }
            let has_motor_errors = self.has_reported_motor_errors()
                || self
                    .driver
                    .as_ref()
                    .and_then(|d| d.read().map(|drv| drv.has_hardware_fault()))
                    .unwrap_or(false);
            if has_motor_errors {
                let state = self.clear_carried_over_motor_errors_inner(
                    now_fn,
                    Some(deadline),
                    expected_epoch,
                    authority,
                    release_budget,
                )?;
                if let Some(ref cand) = candidate_limits {
                    self.commit_driver_limits_bounded(cand);
                }
                return Ok(state);
            }
            if let Some(ref cand) = candidate_limits {
                self.commit_driver_limits_bounded(cand);
            }
            return Ok(self.safety.state().clone());
        }

        let mut ctx;
        let mut commit = ClearFaultCommit {
            candidate_limits,
            reapply_legacy_limits: false,
            is_torqued: None,
            reset_disarmed_hold: false,
            thermal_cooldown: None,
        };
        if self.driver.is_some() {
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_rx = std::mem::take(&mut self.scratch.rx);
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let drv = match self.driver.as_ref().and_then(|d| d.read()) {
                Some(g) => g,
                None => {
                    self.scratch.ages = scratch_ages;
                    self.scratch.rx = scratch_rx;
                    self.ws_eval_bufs = ws_bufs;
                    return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
                }
            };
            // Pure read-only driver snapshot FIRST — ZERO motor resets or mutations.
            drv.feedback_ages_into(&mut scratch_ages);
            let ages_valid = scratch_ages.len() == self.config.dof
                && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
            let fb_valid = drv.is_feedback_valid();
            let j7_fb_fresh = drv.pos_force().pos_force_feedback_fresh();
            let tx_queue_drained = drv.tx().pending_tx_len_total() == 0;
            let tx_probe_confirmed = drv.tx().tx_probe_confirmed();
            let tx_probe_sent = drv.tx().tx_probe_pending();
            let hw_fault = drv.has_hardware_fault();
            let caps = drv.capabilities();
            let temp_fb = caps.temperature_feedback;
            let enable_fb = caps.enable_feedback;
            // M1: Resting-pose check necessity follows owner decision state (`torque_intent == On`
            // or `is_torqued`), never feedback-reported `enable_state()`.
            let needs_resting_check = (self.is_torqued || self.torque_intent == TorqueIntent::On)
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
            // Cooldown accrues ONLY across consecutive fresh cool samples; evaluate without
            // mutating `self.thermal_cooldown` until clear commits (or ThermalCooldownNotElapsed).
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

            ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            ctx.feedback_fresh = fb_fresh;
            ctx.j7_feedback_fresh = j7_fb_fresh;
            ctx.tx_queue_drained = tx_queue_drained;
            ctx.tx_probe_confirmed = tx_probe_confirmed;
            ctx.tx_probe_sent = tx_probe_sent;
            ctx.thermal_sensor_available = temp_fb;
            ctx.thermal_below_threshold = thermal_below;
            ctx.thermal_cooldown_elapsed_ms = if thermal_below { next_cd.elapsed_ms() } else { 0 };
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            ctx.resting_feedback_missing = resting_feedback_missing;
            ctx.resting_pose_violation = resting_pose_violation;
            ctx.hardware_reset_required = hw_fault;
            self.populate_enable_clear_context(&mut ctx, enable_fb);

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any TX probe, motor
            // reset, or concurrent E-stop / config mutation.
            let mut clear_res = self.safety.clear_check(&ctx);
            if clear_res == Err(R::TxProbeRequired) {
                if now_fn() >= deadline {
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                }
                enum BudgetedProbeAttempt<T> {
                    Ran(T),
                    BudgetExpired,
                    Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                }
                let budget_rem = release_budget
                    .as_deref()
                    .map(|b| (b.rx_remaining(), b.tx_remaining()));
                let mut probe_rx_delta = 0u16;
                let mut probe_tx_delta = 0u16;
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let probe_outcome = self.with_driver_and_owner_ref(|owner, drv| {
                    if let Some(auth) = authority {
                        if let Err(auth_err) =
                            recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                        {
                            return BudgetedProbeAttempt::Authority(auth_err);
                        }
                    }
                    // Sample driver clock FIRST, then live owner clock, and refuse expired budgets
                    // before staging any driver TX deadline so guard wait or `clock_now()` latency
                    // can never add time back to the caller's cutoff.
                    let Ok((_, _, drv_deadline)) =
                        crate::supervisor::translate_driver_cutoff_conservative(drv, deadline, now_fn)
                    else {
                        return BudgetedProbeAttempt::BudgetExpired;
                    };
                    let before_budget = drv.tx().release_budget_consumed();
                    if let Some((rx_rem, tx_rem)) = budget_rem {
                        drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                    } else {
                        // Standalone off-release clear_fault call: exit any prior managed release
                        // and open a fresh TX pass before installing the release-margin deadline
                        // so a backpressured/expired pass from a prior release never blocks 0xCC.
                        drv.tx_mut().reset_standalone_tx_pass();
                    }
                    drv.set_tx_release_deadline(Some(drv_deadline));
                    let res = drv.tx_mut().probe_tx_ready_budgeted(drv_deadline);
                    drv.set_tx_release_deadline(None);
                    let after_budget = drv.tx().release_budget_consumed();
                    probe_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    probe_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    drv.feedback_ages_into(&mut scratch_ages);
                    let ages_ok = scratch_ages.len() == owner.config.dof
                        && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                    BudgetedProbeAttempt::Ran((
                        res,
                        drv.tx().pending_tx_len_total() == 0,
                        drv.tx().tx_probe_confirmed(),
                        drv.tx().tx_probe_pending(),
                        drv.has_hardware_fault(),
                        drv.is_feedback_valid(),
                        drv.pos_force().pos_force_feedback_fresh(),
                        ages_ok,
                    ))
                });
                self.scratch.ages = scratch_ages;
                if let Some(b) = release_budget.as_deref_mut() {
                    if probe_rx_delta > 0 {
                        b.charge_rx(probe_rx_delta, false);
                    }
                    if probe_tx_delta > 0 {
                        b.charge_tx(probe_tx_delta);
                    }
                }
                let (probe_res, drained_after, probe_ok, probe_pending, hw_fault_after, fb_valid_after, j7_fresh_after, ages_valid_after) =
                    match probe_outcome
                        .ok_or(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable))?
                    {
                        BudgetedProbeAttempt::Ran(v) => v,
                        BudgetedProbeAttempt::BudgetExpired => {
                            return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                        }
                        BudgetedProbeAttempt::Authority(e) => {
                            return Err(ClearFaultWithAuthError::Authority(e));
                        }
                    };
                ctx.tx_queue_drained = drained_after;
                self.populate_enable_clear_context(&mut ctx, enable_fb);
                ctx.hardware_reset_required = hw_fault_after;
                ctx.j7_feedback_fresh = j7_fresh_after;
                ctx.pending_estop = self.estop_request_unapplied();
                match probe_res {
                    Ok(confirmed) => {
                        ctx.tx_probe_sent = true;
                        ctx.tx_probe_confirmed = confirmed && probe_ok;
                        ctx.feedback_fresh = fb_fresh && fb_valid_after && ages_valid_after;
                    }
                    Err(e) => {
                        ctx.tx_probe_confirmed = false;
                        ctx.tx_probe_sent = probe_pending;
                        ctx.hardware_error = Some(clear_fault_driver_error(&e));
                    }
                }
                clear_res = self.safety.clear_check(&ctx);
            }
            match clear_res {
                Ok(()) => {}
                Err(R::EStopActive) => return Err(apply_estop_refusal(self)),
                Err(R::HardwareResetRequired) => {
                    // Explicitly modeled exception: every OTHER recorded kind's condition passed,
                    // so a budgeted motor reset is now permitted. Recheck live grant, lease,
                    // operator session (M3), J7 freshness, Rule A′ rest-pose gate (when the reset
                    // itself disables an arm joint, e.g. Feetech STS3215), and pending E-stop under
                    // the SAME driver lock guard / exclusive borrow before sending the reset (zero gap, zero alloc).
                    if now_fn() >= deadline {
                        return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                    }
                    #[allow(clippy::large_enum_variant)]
                    enum BudgetedResetAttempt {
                        Ran(Result<(), crate::driver::ArmDriverError>),
                        Refusal(R),
                        StoppedTorqueOff(crate::supervisor::StoppedTorqueOffRefusal),
                        Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
                    }
                    let budget_rem = release_budget
                        .as_deref()
                        .map(|b| (b.rx_remaining(), b.tx_remaining()));
                    let mut reset_rx_delta = 0u16;
                    let mut reset_tx_delta = 0u16;
                    let reset_outcome = self
                        .with_driver_and_owner_ref(|owner, drv| {
                            if let Some(auth) = authority {
                                if let Err(auth_err) =
                                    recheck_clear_fault_authority_readonly(owner, auth, now_fn())
                                {
                                    return BudgetedResetAttempt::Authority(auth_err);
                                }
                            }
                            if needs_resting_check {
                                if let Err(refusal) =
                                    Self::check_stopped_torque_off_with_driver(&owner.config, &*drv)
                                {
                                    return BudgetedResetAttempt::StoppedTorqueOff(refusal);
                                }
                            }
                            ctx.j7_feedback_fresh = drv.pos_force().pos_force_feedback_fresh();
                            ctx.feedback_fresh = ctx.feedback_fresh && drv.is_feedback_valid();
                            ctx.pending_estop = owner.estop_request_unapplied();
                            if let Err(refusal) = owner.safety.clear_check(&ctx) {
                                if refusal != R::HardwareResetRequired {
                                    return BudgetedResetAttempt::Refusal(refusal);
                                }
                            }
                            // Sample driver clock FIRST, then live owner clock, and refuse expired
                            // budgets before staging any driver TX deadline.
                            let Ok((_, _, drv_deadline)) =
                                crate::supervisor::translate_driver_cutoff_conservative(
                                    drv, deadline, now_fn,
                                )
                            else {
                                return BudgetedResetAttempt::Refusal(R::BudgetExpired);
                            };
                            let before_budget = drv.tx().release_budget_consumed();
                            if let Some((rx_rem, tx_rem)) = budget_rem {
                                drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                            }
                            drv.set_tx_release_deadline(Some(drv_deadline));
                            let res = drv.clear_fault_budgeted(drv_deadline);
                            drv.set_tx_release_deadline(None);
                            let after_budget = drv.tx().release_budget_consumed();
                            reset_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                            reset_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                            BudgetedResetAttempt::Ran(res)
                        })
                        .ok_or(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable))?;
                    if let Some(b) = release_budget {
                        if reset_rx_delta > 0 {
                            b.charge_rx(reset_rx_delta, false);
                        }
                        if reset_tx_delta > 0 {
                            b.charge_tx(reset_tx_delta);
                        }
                    }
                    let reset_res = match reset_outcome {
                        BudgetedResetAttempt::Ran(res) => res,
                        BudgetedResetAttempt::Refusal(R::EStopActive) => {
                            return Err(apply_estop_refusal(self));
                        }
                        BudgetedResetAttempt::Refusal(refusal) => {
                            return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
                        }
                        BudgetedResetAttempt::StoppedTorqueOff(refusal) => {
                            self.last_stopped_torque_off_refusal = Some(refusal);
                            return Err(ClearFaultWithAuthError::Clear(
                                refusal.into_clear_fault_error(),
                            ));
                        }
                        BudgetedResetAttempt::Authority(e) => {
                            return Err(ClearFaultWithAuthError::Authority(e));
                        }
                    };
                    ctx.hardware_reset_required = false;
                    ctx.pending_estop = self.estop_request_unapplied();
                    match reset_res {
                        Ok(()) => ctx.hardware_reset_sent = true,
                        Err(
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                            | crate::driver::ArmDriverError::DisableRequestExpired,
                        ) => {
                            let r = self
                                .may_disable_torque_while_stopped()
                                .err()
                                .unwrap_or(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback);
                            self.last_stopped_torque_off_refusal = Some(r);
                            return Err(ClearFaultWithAuthError::Clear(r.into_clear_fault_error()));
                        }
                        Err(e) => ctx.hardware_error = Some(clear_fault_driver_error(&e)),
                    }
                    let post_refusal = self.safety.clear_check(&ctx).unwrap_err();
                    if post_refusal == R::EStopActive {
                        return Err(apply_estop_refusal(self));
                    }
                    return Err(ClearFaultWithAuthError::Clear(E::from(post_refusal)));
                }
                Err(refusal) => {
                    if matches!(refusal, R::ThermalCooldownNotElapsed { .. })
                        && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                            || ctx.j7_feedback_fresh)
                    {
                        self.thermal_cooldown = next_cd;
                    }
                    return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
                }
            }
            if self.torque_intent == TorqueIntent::Off {
                commit.is_torqued = Some(false);
                commit.reset_disarmed_hold = true;
            } else if self.enable_lifecycle == EnableLifecycle::FailedHold {
                commit.is_torqued = Some(true);
            }
            commit.thermal_cooldown = Some(next_cd);
        } else {
            // Driverless simulation mode: evaluate cooldown without pre-committing (0 alloc).
            let next_cd = self.evaluate_thermal_cooldown_driverless(Instant::now());
            ctx = self.build_clear_context_base(driver_err_override);
            if candidate_limits.is_some() {
                ctx.driver_config_error = None;
            }
            ctx.expected_epoch = expected_epoch;
            ctx.pending_estop = self.estop_request_unapplied();
            ctx.mit_lost_unsupported = self.mit.owner == crate::mit::MitOwner::Lost;
            ctx.thermal_cooldown_elapsed_ms = if ctx.thermal_below_threshold {
                next_cd.elapsed_ms()
            } else {
                0
            };
            ctx.joint_limit_violation =
                self.joint_limit_clear_violation_with_candidates(candidate_limits.as_ref());
            if let Err(refusal) = self.safety.clear_check(&ctx) {
                if refusal == R::EStopActive {
                    return Err(apply_estop_refusal(self));
                }
                if matches!(refusal, R::ThermalCooldownNotElapsed { .. })
                    && (!self.safety.has_latched_fault_kind(crate::safety::FaultKind::J7FeedbackStale)
                        || ctx.j7_feedback_fresh)
                {
                    self.thermal_cooldown = next_cd;
                }
                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
            }
            commit.thermal_cooldown = Some(next_cd);
        }
        self.clear_fault_finish_bounded_with_authority(&ctx, commit, authority, Some(now_fn()), |_| {})
    }

    /// Bounded fault clearing for the RT executor rendered into [`SafetyError`] for direct callers.
    pub fn clear_fault_executor(&mut self, deadline: Instant) -> Result<SafetyState, SafetyError> {
        self.clear_fault_executor_for_epoch(deadline, None)
    }

    /// Zero-allocation rest-pose verification on fresh feedback for J1–J6 (`config.arm_joints()`)
    /// before clearing carried-over motor hardware errors on a disarmed arm.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    fn check_carryover_rest_pose_with_driver(
        config: &crate::model::ArmConfig,
        drv: &dyn crate::driver::ArmDriver,
    ) -> Result<(), crate::safety::ClearRefusal> {
        use crate::safety::{
            ClearRefusal as R, RestingSupportJointItem as Item, RestingSupportViolation as V,
            MAX_RESTING_SUPPORT_ITEMS,
        };
        let Some(policy) = config.resting_pose.as_ref() else {
            return Err(R::MotorErrorNotAtRestPose(V::NoRestingPoseConfigured));
        };
        let tol = config.rest_pose_tolerance_rad;
        if !tol.is_finite() || tol <= 0.0 {
            return Err(R::FreshFeedbackRequiredForRestingSupport);
        }
        let max_age = Duration::from_millis(policy.feedback_max_age_ms);
        let rest = config.resting_positions();
        let arm_joints = config.arm_joints();
        let mut arm_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        if arm_joints.is_empty()
            || arm_joints.len() > arm_positions.len()
            || rest.len() < config.dof
        {
            return Err(R::FreshFeedbackRequiredForRestingSupport);
        }
        if !drv.fresh_joint_positions_into(
            &arm_joints,
            max_age,
            &mut arm_positions[..arm_joints.len()],
        ) {
            return Err(R::FreshFeedbackRequiredForRestingSupport);
        }
        let mut items = [None; MAX_RESTING_SUPPORT_ITEMS];
        let mut len = 0usize;
        let mut overflowed = 0u16;
        for (&i, &pos) in arm_joints.iter().zip(arm_positions[..arm_joints.len()].iter()) {
            let rest_pos = rest[i];
            let dev = (pos - rest_pos).abs();
            if !pos.is_finite() || !rest_pos.is_finite() || !dev.is_finite() || dev > tol {
                if len < MAX_RESTING_SUPPORT_ITEMS {
                    items[len] = Some(Item::JointOffHome {
                        joint: i as u16,
                        deviation: dev,
                        limit: tol,
                    });
                    len += 1;
                } else {
                    overflowed = overflowed.saturating_add(1);
                }
            }
        }
        if len > 0 {
            return Err(R::MotorErrorNotAtRestPose(V::Failures {
                items,
                len: len as u8,
                overflowed,
                workspace: None,
            }));
        }
        Ok(())
    }

    /// Zero-allocation in-place sync of `self.motor_errors` and cached enable state from a
    /// pre-captured driver status snapshot.
    fn apply_synced_motor_status_bounded(
        &mut self,
        status: &[u8],
        names: &[Option<&'static str>; crate::rt_core::mailbox::MAX_DOF],
        enable_state: EnableState,
    ) {
        self.last_poll_enable_state = enable_state;
        self.capture_driver_cache.enable_state = enable_state;
        let len = status.len().min(names.len());
        if self.rt_executor_owned && self.motor_errors.capacity() < len {
            // Never grow on the RT thread; truncate to existing capacity if pre-reserve missed.
            let cap = self.motor_errors.capacity().min(len);
            self.motor_errors.resize_with(cap, || None);
        } else {
            self.motor_errors.resize_with(len, || None);
        }
        for i in 0..self.motor_errors.len().min(len) {
            self.motor_errors[i] = names[i].map(|name| crate::model::MotorError {
                code: status[i],
                name: std::borrow::Cow::Borrowed(name),
            });
        }
    }

    /// Clear carried-over motor hardware errors on a disarmed arm when no armd fault is latched.
    ///
    /// Shared single-authority path for both legacy and RT executor `clear_fault`:
    /// - requires disarmed supervisor + driver (`Disabled` or `Unknown`, no joint enabled, no torque);
    /// - requires J1–J6 at the resting pose on fresh feedback;
    /// - requires operator authority (when invoked via WS/lease authority);
    /// - transmits `CMD_CLEAR_FAULTS` to affected motors only, then verifies on a fresh post-clear
    ///   status reply that the error bit is cleared and the motor stayed disabled (never enables);
    /// - zero heap allocations on the RT executor path.
    #[allow(clippy::result_large_err)]
    fn clear_carried_over_motor_errors_inner(
        &mut self,
        now_fn: &mut dyn FnMut() -> Instant,
        deadline: Option<Instant>,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
        release_budget: Option<&mut crate::rt_core::budget::ReleaseBudget>,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        use crate::driver::CarryoverMotorClearOutcome;
        use crate::rt_core::requests::lease::recheck_clear_fault_authority_readonly_with_op;
        use crate::safety::{ClearFaultError as E, ClearRefusal as R};

        let apply_estop_refusal = |this: &mut Self| -> ClearFaultWithAuthError {
            this.consume_pending_estop();
            if let Some(received) = expected_epoch {
                if received != this.safety.epoch_id() {
                    return ClearFaultWithAuthError::Clear(E::EpochMismatch {
                        expected: this.safety.epoch_id(),
                        received,
                    });
                }
            }
            ClearFaultWithAuthError::Clear(E::EStopActive)
        };

        let supervisor_disarmed = !self.is_torqued
            && !self.disable_pending
            && self.gravity_comp.is_none()
            && !self.torque_transition_pending()
            && self.enable_lifecycle == EnableLifecycle::Disarmed
            && !self.safety.state().is_operational()
            && self.active_trajectory.is_none()
            && self.servo_target.is_none()
            && !self.last_action.as_ref().is_some_and(|a| a.is_running())
            && self.macro_run.is_none();
        if !supervisor_disarmed {
            return Err(ClearFaultWithAuthError::Clear(E::Rejected(
                R::MotorErrorClearRequiresDisarmed,
            )));
        }

        if let Some(auth) = authority {
            recheck_clear_fault_authority_readonly_with_op(self, auth, now_fn(), true)
                .map_err(ClearFaultWithAuthError::Authority)?;
        }

        if self.driver.is_none() {
            return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
        }

        if let Some(dl) = deadline {
            if now_fn() >= dl {
                return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
            }
        }

        #[allow(clippy::large_enum_variant)]
        enum CarryoverAttempt {
            Ran {
                outcome: Result<CarryoverMotorClearOutcome, crate::driver::ArmDriverError>,
                persist_name: Option<&'static str>,
                enable_state: EnableState,
                has_status: bool,
                names: [Option<&'static str>; crate::rt_core::mailbox::MAX_DOF],
            },
            Refusal(R),
            Authority(crate::rt_core::requests::lease::ClearFaultAuthorityError),
        }

        let budget_rem = release_budget
            .as_deref()
            .map(|b| (b.rx_remaining(), b.tx_remaining()));
        let mut co_rx_delta = 0u16;
        let mut co_tx_delta = 0u16;
        let mut scratch_status = std::mem::take(&mut self.scratch.status);
        let attempt = self
            .with_driver_and_owner_ref(|owner, drv| {
                let before_budget = drv.tx().release_budget_consumed();
                if let Some((rx_rem, tx_rem)) = budget_rem {
                    drv.tx_mut().sync_release_budget_limits(rx_rem, tx_rem);
                }
                let drv_deadline = if let Some(dl) = deadline {
                    let Ok((_, _, translated)) =
                        crate::supervisor::translate_driver_cutoff_conservative(drv, dl, now_fn)
                    else {
                        return CarryoverAttempt::Refusal(R::BudgetExpired);
                    };
                    drv.set_tx_release_deadline(Some(translated));
                    Some(translated)
                } else {
                    None
                };

                // 1. Drain any queued RX backlog FIRST so newer off-rest or enabled-motor frames
                //    are never missed by the rest-pose or disarm interlocks.
                if let Err(e) = drv.bus_recovery_mut().drain_rx_only_bounded(drv_deadline) {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::DriverError(clear_fault_driver_error(&e)));
                }

                // 2. Re-check live authority and pending E-stop after RX drain.
                if let Some(auth) = authority {
                    if let Err(auth_err) =
                        recheck_clear_fault_authority_readonly_with_op(owner, auth, now_fn(), true)
                    {
                        if deadline.is_some() {
                            drv.set_tx_release_deadline(None);
                        }
                        let after_budget = drv.tx().release_budget_consumed();
                        co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                        co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                        return CarryoverAttempt::Authority(auth_err);
                    }
                }
                if owner.estop_request_unapplied() {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::EStopActive);
                }

                // 3. Require fresh disarm evidence across ALL motors (including J7) so a stale
                //    enabled motor can never satisfy `enable_state == Unknown`.
                let has_all_fresh_feedback = drv.is_feedback_valid();
                let has_status_pre = drv.motor_status_into(&mut scratch_status);
                let any_status_enabled = has_status_pre && scratch_status.contains(&1);
                if !matches!(drv.enable_state(), EnableState::Disabled | EnableState::Unknown)
                    || drv.has_any_joint_enabled()
                    || any_status_enabled
                {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::MotorErrorClearRequiresDisarmed);
                }
                if !has_all_fresh_feedback {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(R::FreshFeedbackRequiredForRestingSupport);
                }

                // 4. Evaluate J1–J6 rest-pose gate strictly AFTER the RX drain.
                if let Err(refusal) = Self::check_carryover_rest_pose_with_driver(&owner.config, &*drv)
                {
                    if deadline.is_some() {
                        drv.set_tx_release_deadline(None);
                    }
                    let after_budget = drv.tx().release_budget_consumed();
                    co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                    co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                    return CarryoverAttempt::Refusal(refusal);
                }

                // 5. Submit targeted `CMD_CLEAR_FAULTS` with a live pre-TX authority & E-stop gate
                //    checked immediately before every CAN frame write.
                let mut revoked_auth: Option<crate::rt_core::requests::lease::ClearFaultAuthorityError> =
                    None;
                let mut pre_tx_gate = || -> Result<(), &'static str> {
                    if let Some(auth) = authority {
                        if let Err(auth_err) = recheck_clear_fault_authority_readonly_with_op(
                            owner,
                            auth,
                            now_fn(),
                            true,
                        ) {
                            revoked_auth = Some(auth_err);
                            return Err("AUTHORITY_REVOKED");
                        }
                    }
                    if owner.estop_request_unapplied() {
                        return Err("ESTOP_PENDING");
                    }
                    Ok(())
                };
                let outcome = drv.bus_recovery_mut().clear_carried_over_motor_errors(drv_deadline, &mut pre_tx_gate);
                if deadline.is_some() {
                    drv.set_tx_release_deadline(None);
                }
                let after_budget = drv.tx().release_budget_consumed();
                co_rx_delta = after_budget.0.saturating_sub(before_budget.0);
                co_tx_delta = after_budget.1.saturating_sub(before_budget.1);
                if let Ok(CarryoverMotorClearOutcome::Revoked(code)) = outcome {
                    if let Some(auth_err) = revoked_auth {
                        return CarryoverAttempt::Authority(auth_err);
                    }
                    if code == "ESTOP_PENDING" {
                        return CarryoverAttempt::Refusal(R::EStopActive);
                    }
                    return CarryoverAttempt::Refusal(R::BudgetExpired);
                }
                if owner.estop_request_unapplied() {
                    return CarryoverAttempt::Refusal(R::EStopActive);
                }
                let persist_name = match outcome {
                    Ok(CarryoverMotorClearOutcome::ErrorPersists { code, .. }) => {
                        drv.decode_motor_error_static(code)
                    }
                    _ => None,
                };
                let has_status = drv.motor_status_into(&mut scratch_status);
                let mut names = [None; crate::rt_core::mailbox::MAX_DOF];
                if has_status {
                    for (slot, &code) in names.iter_mut().zip(scratch_status.iter()) {
                        *slot = drv.decode_motor_error_static(code);
                    }
                }
                CarryoverAttempt::Ran {
                    outcome,
                    persist_name,
                    enable_state: drv.enable_state(),
                    has_status,
                    names,
                }
            })
            .ok_or(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
        let attempt = match attempt {
            Ok(a) => a,
            Err(e) => {
                self.scratch.status = scratch_status;
                return Err(e);
            }
        };
        if let Some(b) = release_budget {
            if co_rx_delta > 0 {
                b.charge_rx(co_rx_delta, false);
            }
            if co_tx_delta > 0 {
                b.charge_tx(co_tx_delta);
            }
        }

        let result = match attempt {
            CarryoverAttempt::Ran {
                outcome,
                persist_name,
                enable_state,
                has_status,
                names,
            } => {
                if has_status {
                    self.apply_synced_motor_status_bounded(&scratch_status, &names, enable_state);
                }
                match outcome {
                    Ok(CarryoverMotorClearOutcome::NoErrors | CarryoverMotorClearOutcome::Cleared) => {
                        if self.estop_request_unapplied() {
                            Err(apply_estop_refusal(self))
                        } else {
                            Ok(self.safety.state().clone())
                        }
                    }
                    Ok(CarryoverMotorClearOutcome::ErrorPersists { joint, code }) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(
                            R::MotorErrorPersists {
                                joint: joint.min(usize::from(u8::MAX)) as u8,
                                code,
                                name: persist_name,
                            },
                        )))
                    }
                    Ok(CarryoverMotorClearOutcome::MotorEnabled { joint }) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(
                            R::MotorErrorClearMotorEnabled {
                                joint: joint.min(usize::from(u8::MAX)) as u8,
                            },
                        )))
                    }
                    Ok(CarryoverMotorClearOutcome::Unconfirmed { joint }) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(
                            R::MotorErrorClearUnconfirmed {
                                joint: joint.min(usize::from(u8::MAX)) as u8,
                            },
                        )))
                    }
                    Ok(CarryoverMotorClearOutcome::Revoked(_)) => {
                        Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)))
                    }
                    Err(crate::driver::ArmDriverError::EstopInhibited) => {
                        Err(apply_estop_refusal(self))
                    }
                    Err(e) => Err(ClearFaultWithAuthError::Clear(E::Rejected(
                        R::DriverError(clear_fault_driver_error(&e)),
                    ))),
                }
            }
            CarryoverAttempt::Refusal(R::EStopActive) => Err(apply_estop_refusal(self)),
            CarryoverAttempt::Refusal(refusal) => {
                Err(ClearFaultWithAuthError::Clear(E::from(refusal)))
            }
            CarryoverAttempt::Authority(e) => Err(ClearFaultWithAuthError::Authority(e)),
        };
        self.scratch.status = scratch_status;
        result
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

    fn mark_panic_disable_requested(&mut self) {
        self.estop_state.latch();
        self.active_disable_kind = Some(crate::driver::DisableKind::DisableProtective {
            reason: crate::driver::DisableProtectiveReason::ProcessExit,
        });
        self.disable_requested();
    }

    /// Explicit test-only ProcessExit authorization on `SupervisorInner` (DRV-F1 B7).
    #[doc(hidden)]
    pub fn authorize_process_exit_for_test(&mut self) {
        self.process_exit_authorized = true;
    }

    /// Bounded last-chance disable reserved for explicit `ProcessExit` paths owned by this
    /// `SupervisorInner`. Refuses protective cuts unless explicit owner-bound ProcessExit
    /// authorization is set (`process_exit_authorized` or `shutdown_committed`, DRV-F1 B7).
    #[doc(hidden)]
    pub fn attempt_panic_disable(&mut self, timeout: Duration) -> PanicDisableOutcome {
        if !self.process_exit_authorized && !self.shutdown_committed {
            return PanicDisableOutcome::Failed(
                "protective disable requires explicit ProcessExit authorization".into(),
            );
        }
        self.attempt_bounded_teardown_disable(timeout, true)
    }

    /// Bounded protective disable on repeated shutdown (drains driver observations without
    /// emitting a premature `ProcessExit` before `commit_shutdown`).
    pub(crate) fn attempt_shutdown_protective_disable(&mut self, timeout: Duration) -> PanicDisableOutcome {
        self.mark_panic_disable_requested();
        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        let outcome = run_with_panic_deadline_observer_until(
            hard_deadline,
            || self.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget),
            |_| true,
            move || {
                PanicDisableOutcome::TimedOut.log("Bounded panic disable", timeout);
            },
        )
        .unwrap_or(PanicDisableOutcome::TimedOut);
        #[cfg(not(target_arch = "wasm32"))]
        if self.torque_shadow.is_enabled() {
            let deferred_mit = self.torque_shadow.take_deferred_mit_off();
            if !deferred_mit.is_empty() {
                self.torque_shadow.enqueue(
                    crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                        channels: deferred_mit,
                        target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                        device_wide: false,
                    },
                );
            }
            match &outcome {
                PanicDisableOutcome::Confirmed
                | PanicDisableOutcome::Unconfirmed
                | PanicDisableOutcome::NoDriver => {
                    if let Some(ref mut d) = self.driver {
                        let shadow_rt = &mut self.torque_shadow;
                        match d {
                            crate::supervisor_lock::DriverSlot::Exclusive(drv) => {
                                shadow_rt.drain_from_driver(drv.get_mut().as_mut());
                            }
                            crate::supervisor_lock::DriverSlot::Legacy(_)
                                if crate::rt_core::executor::on_rt_control_thread() =>
                            {
                                shadow_rt.record_external_staging_loss(1);
                            }
                            crate::supervisor_lock::DriverSlot::Legacy(_) => {
                                if d.try_with_mut(|drv| shadow_rt.drain_from_driver(drv)).is_err() {
                                    shadow_rt.record_external_staging_loss(1);
                                }
                            }
                        }
                    }
                }
                PanicDisableOutcome::TimedOut | PanicDisableOutcome::Failed(_) => {
                    self.torque_shadow.record_external_staging_loss(1);
                }
            }
            self.flush_torque_shadow();
        }
        outcome
    }

    fn attempt_bounded_teardown_disable(
        &mut self,
        timeout: Duration,
        is_caught_panic: bool,
    ) -> PanicDisableOutcome {
        #[cfg(not(target_arch = "wasm32"))]
        let (pre_intent, pre_torqued, pre_stopped, pre_ts) = (
            self.shadow_live_intent(),
            self.is_torqued,
            self.safety.state().is_stopped(),
            self.cached_driver_clock_now(),
        );
        self.mark_panic_disable_requested();
        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        let outcome = run_with_panic_deadline_observer_until(
            hard_deadline,
            || self.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget),
            |_| true,
            move || {
                PanicDisableOutcome::TimedOut.log("Bounded panic disable", timeout);
            },
        )
        .unwrap_or(PanicDisableOutcome::TimedOut);
        #[cfg(not(target_arch = "wasm32"))]
        self.record_post_panic_disable_shadow_at(
            is_caught_panic,
            &outcome,
            pre_intent,
            pre_torqued,
            pre_stopped,
            pre_ts,
        );
        outcome
    }

    /// Record panic or forced-teardown shadow state AFTER the live deadline observer has sealed
    /// the live `PanicDisableOutcome`. Signals incomplete staging loss if the driver timed out
    /// or panicked so `interval_unverifiable` latches cleanly. No-op (zero locks) when disabled.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn record_post_panic_disable_shadow(
        &mut self,
        is_caught_panic: bool,
        outcome: &PanicDisableOutcome,
    ) {
        let pre_intent = self.shadow_live_intent();
        let pre_torqued = self.is_torqued;
        let pre_stopped = self.safety.state().is_stopped();
        let pre_ts = self.cached_driver_clock_now();
        self.record_post_panic_disable_shadow_at(
            is_caught_panic,
            outcome,
            pre_intent,
            pre_torqued,
            pre_stopped,
            pre_ts,
        );
    }

    /// Record panic or forced-teardown shadow state using the pre-disable live state and causal
    /// timestamp captured BEFORE `attempt_panic_disable_until` ran.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn record_post_panic_disable_shadow_at(
        &mut self,
        is_caught_panic: bool,
        outcome: &PanicDisableOutcome,
        pre_intent: crate::torque::TorqueIntent,
        pre_torqued: bool,
        pre_stopped: bool,
        pre_ts: Instant,
    ) {
        if !self.torque_shadow.is_enabled() {
            return;
        }
        // Emit any MIT ownership release deferred during `attempt_panic_disable_until` strictly
        // AFTER the live panic deadline observer has sealed `outcome`.
        let deferred_mit = self.torque_shadow.take_deferred_mit_off();
        if !deferred_mit.is_empty() {
            self.torque_shadow.enqueue_at(
                pre_ts,
                crate::torque::shadow::ShadowInputKind::MitOwnershipChange {
                    channels: deferred_mit,
                    target_mode: crate::motor_mode::ControlMode::PositionVelocity,
                    device_wide: false,
                },
            );
        }
        if is_caught_panic {
            self.torque_shadow.enqueue_at(
                pre_ts,
                crate::torque::shadow::ShadowInputKind::Panic {
                    tier: crate::torque::PanicTier::CaughtControlPanic,
                    live_intent: pre_intent,
                    live_is_torqued: pre_torqued,
                    live_stopped: pre_stopped,
                },
            );
        } else {
            self.torque_shadow.enqueue_at(
                pre_ts,
                crate::torque::shadow::ShadowInputKind::ProcessExit {
                    live_intent: pre_intent,
                    live_is_torqued: pre_torqued,
                    live_stopped: pre_stopped,
                },
            );
        }
        match outcome {
            PanicDisableOutcome::Confirmed
            | PanicDisableOutcome::Unconfirmed
            | PanicDisableOutcome::NoDriver => {
                if let Some(ref mut d) = self.driver {
                    // Hard-RT invariant: NEVER attempt a driver mutex lock from `armd-control`
                    // (even `try_lock`) on emergency teardown; exclusive RT driver drains
                    // lock-free, while legacy slots on `armd-control` latch loss lock-free.
                    let shadow_rt = &mut self.torque_shadow;
                    match d {
                        crate::supervisor_lock::DriverSlot::Exclusive(drv) => {
                            shadow_rt.drain_from_driver(drv.get_mut().as_mut());
                        }
                        crate::supervisor_lock::DriverSlot::Legacy(_)
                            if crate::rt_core::executor::on_rt_control_thread() =>
                        {
                            shadow_rt.record_external_staging_loss(1);
                        }
                        crate::supervisor_lock::DriverSlot::Legacy(_) => {
                            if d.try_with_mut(|drv| shadow_rt.drain_from_driver(drv)).is_err() {
                                shadow_rt.record_external_staging_loss(1);
                            }
                        }
                    }
                }
            }
            PanicDisableOutcome::TimedOut | PanicDisableOutcome::Failed(_) => {
                self.torque_shadow.record_external_staging_loss(1);
            }
        }
        self.flush_torque_shadow();
    }

    pub(crate) fn attempt_panic_disable_until(
        &mut self,
        coop_deadline: Instant,
        hard_deadline: Instant,
        coop_budget: Duration,
    ) -> PanicDisableOutcome {
        self.mark_panic_disable_requested();
        let Some(ref mut d) = self.driver else {
            self.disable_accepted_inner(false);
            return PanicDisableOutcome::NoDriver;
        };
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| match d {
            crate::supervisor_lock::DriverSlot::Exclusive(drv) => {
                if Instant::now() >= hard_deadline {
                    return PanicDisableOutcome::TimedOut;
                }
                bounded_panic_disable_driver_until(drv.get_mut().as_mut(), coop_deadline, hard_deadline, coop_budget)
            }
            crate::supervisor_lock::DriverSlot::Legacy(lock) => {
                let mut drv = loop {
                    match lock.try_lock() {
                        Ok(g) => break g,
                        Err(std::sync::TryLockError::Poisoned(p)) => break p.into_inner(),
                        Err(std::sync::TryLockError::WouldBlock) => {
                            let rem = hard_deadline.saturating_duration_since(Instant::now());
                            if rem.is_zero() {
                                return PanicDisableOutcome::TimedOut;
                            }
                            std::thread::sleep(rem.min(Duration::from_millis(1)));
                        }
                    }
                };
                if Instant::now() >= hard_deadline {
                    return PanicDisableOutcome::TimedOut;
                }
                bounded_panic_disable_driver_until(&mut **drv, coop_deadline, hard_deadline, coop_budget)
            }
        }))
        .unwrap_or_else(|_| PanicDisableOutcome::Failed("panic during driver panic-disable".into()));
        if outcome == PanicDisableOutcome::Confirmed {
            self.disable_accepted_inner(false);
        }
        outcome
    }

    pub fn driver_feedback_confirmed(&self) -> bool {
        self.read_driver_for_tick()
            .is_some_and(|drv| drv.is_feedback_valid())
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

#[cfg(test)]
mod panic_disable_tests {
    use super::*;
    use crate::driver::{ArmDriverError, TorqueOutcome, TorqueProgress};
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::{Arc, Condvar, Mutex};

    struct SelectiveMockDriver {
        selective_calls: Arc<AtomicUsize>,
        generic_begin_calls: Arc<AtomicUsize>,
        sleep_on_step: Duration,
        step_result: Result<Vec<usize>, ArmDriverError>,
    }

    impl ArmDriver for SelectiveMockDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                enable_feedback: true,
                ..DriverCapabilities::default()
            }
        }
        fn is_enabled(&self) -> bool { true }
        fn enable_state(&self) -> EnableState { EnableState::Unknown }
        fn is_feedback_valid(&self) -> bool { false }
        fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> {
            Ok(Vec::new())
        }
        fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn begin_torque_budgeted(
            &mut self,
            _: bool,
            _: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
            Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
        }
        fn unconfirmed_disable_joints(&self) -> Vec<usize> { vec![2] }
        crate::ext_accessors!(bus_recovery);
    }
    impl crate::driver::BusRecoveryExt for SelectiveMockDriver {
        fn panic_disable_step(
            &mut self,
            _first: bool,
            _deadline: Instant,
        ) -> Option<Result<Vec<usize>, ArmDriverError>> {
            self.selective_calls.fetch_add(1, Ordering::SeqCst);
            if !self.sleep_on_step.is_zero() {
                std::thread::sleep(self.sleep_on_step);
            }
            Some(match &self.step_result {
                Ok(v) => Ok(v.clone()),
                Err(e) => Err(ArmDriverError::HardwareFault(e.to_string())),
            })
        }
    }

    struct GenericMockDriver {
        generic_begin_calls: Arc<AtomicUsize>,
        sleep_on_begin: Duration,
    }

    impl ArmDriver for GenericMockDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                enable_feedback: true,
                ..DriverCapabilities::default()
            }
        }
        fn is_enabled(&self) -> bool { true }
        fn enable_state(&self) -> EnableState { EnableState::Unknown }
        fn is_feedback_valid(&self) -> bool { false }
        fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> {
            Ok(Vec::new())
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<crate::model::JointState>,
            _deadline: Instant,
            _max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            out.clear();
            Ok((0, false))
        }
        fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn begin_torque_budgeted(
            &mut self,
            _: bool,
            _: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
            if !self.sleep_on_begin.is_zero() {
                std::thread::sleep(self.sleep_on_begin);
            }
            Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
        }
        fn unconfirmed_disable_joints(&self) -> Vec<usize> {
            vec![0]
        }
    }

    struct ProbeThenGenericMockDriver {
        selective_calls: Arc<AtomicUsize>,
        generic_begin_calls: Arc<AtomicUsize>,
        sleep_on_probe: Duration,
    }

    impl ArmDriver for ProbeThenGenericMockDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                enable_feedback: true,
                ..DriverCapabilities::default()
            }
        }
        fn is_enabled(&self) -> bool { true }
        fn enable_state(&self) -> EnableState { EnableState::Unknown }
        fn is_feedback_valid(&self) -> bool { false }
        fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> { Ok(Vec::new()) }
        fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn begin_torque_budgeted(
            &mut self,
            _: bool,
            _: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
            Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
        }
        crate::ext_accessors!(bus_recovery);
    }
    impl crate::driver::BusRecoveryExt for ProbeThenGenericMockDriver {
        fn panic_disable_step(
            &mut self,
            _first: bool,
            _deadline: Instant,
        ) -> Option<Result<Vec<usize>, ArmDriverError>> {
            self.selective_calls.fetch_add(1, Ordering::SeqCst);
            if !self.sleep_on_probe.is_zero() {
                std::thread::sleep(self.sleep_on_probe);
            }
            None
        }
    }

    #[test]
    fn panic_disable_deadlines_reserves_tail_within_hard_bound() {
        let origin = Instant::now();
        assert_eq!(
            panic_disable_deadlines(origin, Duration::from_millis(500)),
            (origin + Duration::from_millis(475), origin + Duration::from_millis(500))
        );
        assert_eq!(
            panic_disable_deadlines(origin, Duration::from_millis(80)),
            (origin + Duration::from_millis(72), origin + Duration::from_millis(80))
        );
        assert_eq!(
            panic_disable_deadlines(origin, Duration::from_millis(60)),
            (origin + Duration::from_millis(54), origin + Duration::from_millis(60))
        );
        assert_eq!(
            panic_disable_deadlines(origin, Duration::ZERO),
            (origin, origin)
        );
    }

    #[test]
    fn bounded_panic_disable_driver_no_extra_step_after_deadline_and_no_generic_fallthrough() {
        let selective_calls = Arc::new(AtomicUsize::new(0));
        let generic_begin_calls = Arc::new(AtomicUsize::new(0));
        let mut drv = SelectiveMockDriver {
            selective_calls: Arc::clone(&selective_calls),
            generic_begin_calls: Arc::clone(&generic_begin_calls),
            sleep_on_step: Duration::from_millis(25),
            step_result: Ok(vec![2]),
        };
        let origin = Instant::now();
        let coop_budget = Duration::from_millis(15);
        let coop_deadline = origin + coop_budget;
        let hard_deadline = origin + Duration::from_millis(40);
        let outcome = bounded_panic_disable_driver_until(
            &mut drv,
            coop_deadline,
            hard_deadline,
            coop_budget,
        );
        assert_eq!(outcome, PanicDisableOutcome::Unconfirmed);
        assert_eq!(selective_calls.load(Ordering::SeqCst), 1);
        assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

        // Top-of-loop sleep wakeup case: step 1 completes immediately, sleep wakes after
        // `coop_budget` (1ms), and top-of-loop check exits without calling step 2.
        let selective_calls_wake = Arc::new(AtomicUsize::new(0));
        let generic_begin_calls_wake = Arc::new(AtomicUsize::new(0));
        let mut drv_wake = SelectiveMockDriver {
            selective_calls: Arc::clone(&selective_calls_wake),
            generic_begin_calls: Arc::clone(&generic_begin_calls_wake),
            sleep_on_step: Duration::ZERO,
            step_result: Ok(vec![2]),
        };
        let origin_wake = Instant::now();
        let coop_budget_wake = Duration::from_millis(1);
        let outcome_wake = bounded_panic_disable_driver_until(
            &mut drv_wake,
            origin_wake + coop_budget_wake,
            origin_wake + Duration::from_millis(50),
            coop_budget_wake,
        );
        assert_eq!(outcome_wake, PanicDisableOutcome::Unconfirmed);
        assert_eq!(selective_calls_wake.load(Ordering::SeqCst), 1);
        assert_eq!(generic_begin_calls_wake.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn bounded_panic_disable_driver_expired_at_entry_and_generic_probe_expiry() {
        // Expired at entry: zero selective or generic calls, returns TimedOut.
        let selective_calls = Arc::new(AtomicUsize::new(0));
        let generic_begin_calls = Arc::new(AtomicUsize::new(0));
        let mut drv = SelectiveMockDriver {
            selective_calls: Arc::clone(&selective_calls),
            generic_begin_calls: Arc::clone(&generic_begin_calls),
            sleep_on_step: Duration::ZERO,
            step_result: Ok(vec![2]),
        };
        let expired = Instant::now();
        let outcome = bounded_panic_disable_driver_until(&mut drv, expired, expired, Duration::ZERO);
        assert_eq!(outcome, PanicDisableOutcome::TimedOut);
        assert_eq!(selective_calls.load(Ordering::SeqCst), 0);
        assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

        // Selective probe returns `None` after sleeping past `hard_deadline`: generic fallback
        // must see `now >= hard_deadline` and return `TimedOut` with zero `begin_torque_budgeted` calls.
        let probe_calls = Arc::new(AtomicUsize::new(0));
        let generic_calls = Arc::new(AtomicUsize::new(0));
        let mut drv_probe = ProbeThenGenericMockDriver {
            selective_calls: Arc::clone(&probe_calls),
            generic_begin_calls: Arc::clone(&generic_calls),
            sleep_on_probe: Duration::from_millis(25),
        };
        let origin = Instant::now();
        let outcome_probe = bounded_panic_disable_driver_until(
            &mut drv_probe,
            origin + Duration::from_millis(10),
            origin + Duration::from_millis(15),
            Duration::from_millis(10),
        );
        assert_eq!(outcome_probe, PanicDisableOutcome::TimedOut);
        assert_eq!(probe_calls.load(Ordering::SeqCst), 1);
        assert_eq!(generic_calls.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn bounded_panic_disable_driver_generic_no_extra_attempt_after_deadline() {
        let generic_begin_calls = Arc::new(AtomicUsize::new(0));
        let mut drv = GenericMockDriver {
            generic_begin_calls: Arc::clone(&generic_begin_calls),
            sleep_on_begin: Duration::from_millis(25),
        };
        let origin = Instant::now();
        let coop_budget = Duration::from_millis(15);
        let coop_deadline = origin + coop_budget;
        let hard_deadline = origin + Duration::from_millis(40);
        let outcome = bounded_panic_disable_driver_until(
            &mut drv,
            coop_deadline,
            hard_deadline,
            coop_budget,
        );
        assert_eq!(outcome, PanicDisableOutcome::Unconfirmed);
        assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn attempt_panic_disable_until_rejects_new_hardware_tx_after_hard_deadline() {
        let selective_calls = Arc::new(AtomicUsize::new(0));
        let generic_begin_calls = Arc::new(AtomicUsize::new(0));
        let drv = SelectiveMockDriver {
            selective_calls: Arc::clone(&selective_calls),
            generic_begin_calls: Arc::clone(&generic_begin_calls),
            sleep_on_step: Duration::ZERO,
            step_result: Ok(vec![2]),
        };
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv));
        let expired = Instant::now();
        let outcome = inner.attempt_panic_disable_until(expired, expired, Duration::ZERO);
        assert_eq!(outcome, PanicDisableOutcome::TimedOut);
        assert_eq!(selective_calls.load(Ordering::SeqCst), 0);
        assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

        // Contended driver mutex held across `hard_deadline` returns TimedOut with 0 hardware TX.
        let drv_held = SelectiveMockDriver {
            selective_calls: Arc::clone(&selective_calls),
            generic_begin_calls: Arc::clone(&generic_begin_calls),
            sleep_on_step: Duration::ZERO,
            step_result: Ok(vec![2]),
        };
        let mut inner_held = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_held));
        // Safe Rust cannot hold a borrow of `inner_held.driver` across `&mut inner_held`;
        // leaking the guard keeps `try_lock()` contended until `hard_deadline` expires.
        let leaked_guard = inner_held
            .driver
            .as_ref()
            .unwrap()
            .as_legacy_lock()
            .unwrap()
            .lock()
            .unwrap();
        std::mem::forget(leaked_guard);
        let origin_held = Instant::now();
        let outcome_held = inner_held.attempt_panic_disable_until(
            origin_held + Duration::from_millis(5),
            origin_held + Duration::from_millis(10),
            Duration::from_millis(5),
        );
        assert_eq!(outcome_held, PanicDisableOutcome::TimedOut);
        assert_eq!(selective_calls.load(Ordering::SeqCst), 0);
        assert_eq!(generic_begin_calls.load(Ordering::SeqCst), 0);

        // Legacy lock contention clearing after `hard_deadline`: `inner` write lock held past
        // `timeout` and then released -> `attempt_legacy_panic_disable` returns `TimedOut`
        // with 0 hardware TX.
        let sup_calls = Arc::new(AtomicUsize::new(0));
        let sup_generic = Arc::new(AtomicUsize::new(0));
        let drv_sup = SelectiveMockDriver {
            selective_calls: Arc::clone(&sup_calls),
            generic_begin_calls: Arc::clone(&sup_generic),
            sleep_on_step: Duration::ZERO,
            step_result: Ok(vec![2]),
        };
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_sup)));
        let inner_arc = Arc::clone(sup.inner());
        let (locked_tx, locked_rx) = std::sync::mpsc::channel();
        let (rel_tx, rel_rx) = std::sync::mpsc::channel();
        let holder = std::thread::spawn(move || {
            let _guard = inner_arc.blocking_write();
            let _ = locked_tx.send(());
            let _ = rel_rx.recv();
        });
        locked_rx.recv().unwrap();
        sup.authorize_process_exit_for_test();
        let legacy_outcome = sup.attempt_legacy_panic_disable(Duration::from_millis(12));
        rel_tx.send(()).unwrap();
        holder.join().unwrap();
        assert_eq!(legacy_outcome, PanicDisableOutcome::TimedOut);
        assert_eq!(sup_calls.load(Ordering::SeqCst), 0);
        assert_eq!(sup_generic.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn run_with_panic_deadline_observer_until_completes_before_hard_deadline_and_fires_when_blocked() {
        let complete_a = Arc::new(AtomicBool::new(false));
        let complete_a_cb = Arc::clone(&complete_a);
        let timeout_a = Arc::new(AtomicBool::new(false));
        let timeout_a_cb = Arc::clone(&timeout_a);
        let res_a = run_with_panic_deadline_observer_until(
            Instant::now() + Duration::from_millis(100),
            || PanicDisableOutcome::Unconfirmed,
            |outcome| {
                assert_eq!(*outcome, PanicDisableOutcome::Unconfirmed);
                complete_a_cb.store(true, Ordering::SeqCst);
                true
            },
            move || {
                timeout_a_cb.store(true, Ordering::SeqCst);
            },
        );
        assert_eq!(res_a, Some(PanicDisableOutcome::Unconfirmed));
        assert!(complete_a.load(Ordering::SeqCst));
        assert!(!timeout_a.load(Ordering::SeqCst));

        let gate = Arc::new((Mutex::new(false), Condvar::new()));
        let gate_cb = Arc::clone(&gate);
        let timeout_b = Arc::new(AtomicBool::new(false));
        let timeout_b_cb = Arc::clone(&timeout_b);
        let complete_b = Arc::new(AtomicBool::new(false));
        let complete_b_cb = Arc::clone(&complete_b);
        let res_b = run_with_panic_deadline_observer_until(
            Instant::now() + Duration::from_millis(15),
            || {
                let mut done = gate.0.lock().unwrap();
                while !*done {
                    done = gate.1.wait(done).unwrap();
                }
                PanicDisableOutcome::Unconfirmed
            },
            |_| {
                complete_b_cb.store(true, Ordering::SeqCst);
                true
            },
            move || {
                timeout_b_cb.store(true, Ordering::SeqCst);
                let mut done = gate_cb.0.lock().unwrap();
                *done = true;
                gate_cb.1.notify_all();
            },
        );
        assert_eq!(res_b, None);
        assert!(!complete_b.load(Ordering::SeqCst));
        assert!(timeout_b.load(Ordering::SeqCst));

        // Subcase (c): work completes slightly past `hard_deadline` (racing observer wake):
        // single-winner arbitration must return `None`, NOT fire `on_complete`, and fire `on_timeout` once.
        let complete_c = Arc::new(AtomicBool::new(false));
        let complete_c_cb = Arc::clone(&complete_c);
        let timeout_c_count = Arc::new(AtomicUsize::new(0));
        let timeout_c_cb = Arc::clone(&timeout_c_count);
        let res_c = run_with_panic_deadline_observer_until(
            Instant::now(),
            || {
                std::thread::sleep(Duration::from_millis(2));
                PanicDisableOutcome::Unconfirmed
            },
            |_| {
                complete_c_cb.store(true, Ordering::SeqCst);
                true
            },
            move || {
                timeout_c_cb.fetch_add(1, Ordering::SeqCst);
            },
        );
        assert_eq!(res_c, None);
        assert!(!complete_c.load(Ordering::SeqCst));
        assert_eq!(timeout_c_count.load(Ordering::SeqCst), 1);

        // Subcase (d): `on_timeout` blocks on a barrier while `work()` unblocks and returns;
        // `run_with_panic_deadline_observer_until` must NOT return `None` until `on_timeout` finishes.
        let (started_tx, started_rx) = std::sync::mpsc::channel();
        let (work_done_tx, work_done_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let timeout_finished = Arc::new(AtomicBool::new(false));
        let timeout_finished_cb = Arc::clone(&timeout_finished);
        let runner_returned = Arc::new(AtomicBool::new(false));
        let runner_returned_th = Arc::clone(&runner_returned);
        let runner = std::thread::spawn(move || {
            let res = run_with_panic_deadline_observer_until(
                Instant::now() + Duration::from_millis(5),
                || {
                    started_rx.recv().unwrap();
                    let _ = work_done_tx.send(());
                    PanicDisableOutcome::Unconfirmed
                },
                |_| true,
                move || {
                    let _ = started_tx.send(());
                    release_rx.recv().unwrap();
                    timeout_finished_cb.store(true, Ordering::SeqCst);
                },
            );
            runner_returned_th.store(true, Ordering::SeqCst);
            res
        });
        work_done_rx.recv().unwrap();
        std::thread::sleep(Duration::from_millis(5));
        assert!(
            !runner_returned.load(Ordering::SeqCst),
            "observer_until must wait for in-flight on_timeout before returning None"
        );
        release_tx.send(()).unwrap();
        let res_d = runner.join().unwrap();
        assert_eq!(res_d, None);
        assert!(timeout_finished.load(Ordering::SeqCst));
    }

    #[test]
    fn legacy_timeout_aborts_enable_and_detaches_dispatched_disable_without_aborting_protective() {
        use crate::driver::{ArmDriver, ArmDriverError, DisableKind, DisableProtectiveReason, DriverCapabilities, EnableState, JointTarget, TorqueOutcome, TorqueProgress};
        use crate::model::JointState;

        struct StubPendingDriver {
            dof: usize,
            pending: bool,
            dispatched_mask: u64,
            dispatch_mask_on_begin: u64,
            protective: bool,
            abort_enable_calls: usize,
            abort_disable_calls: usize,
            complete_disable_on_poll: Arc<AtomicBool>,
            enabled: bool,
        }

        impl ArmDriver for StubPendingDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    enable_feedback: true,
                    ..DriverCapabilities::default()
                }
            }
            fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(vec![JointState::default(); self.dof])
            }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
                out.clear();
                out.resize(self.dof, JointState::default());
                Some(())
            }
            fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
                self.enabled = enabled;
                Ok(())
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn feedback_all_within(&self, _dof: usize, _max_age: Duration) -> bool { true }
            fn feedback_ages(&self) -> Vec<Option<Duration>> {
                vec![Some(Duration::ZERO); self.dof]
            }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::ZERO));
            }
            fn has_driver_fresh_position_api(&self) -> bool { true }
            fn fresh_measured_position_for(&self, _joint: usize, _max_age: Duration) -> Option<f32> {
                Some(0.0)
            }
            fn fresh_joint_positions_into(
                &self,
                joints: &[usize],
                _max_age: Duration,
                out: &mut [f32],
            ) -> bool {
                for slot in out.iter_mut().take(joints.len()) {
                    *slot = 0.0;
                }
                true
            }
            fn rest_pose_gate_status(&self) -> crate::driver::RestPoseGateStatus {
                crate::driver::RestPoseGateStatus::Confirmed
            }
            fn enable_state(&self) -> EnableState {
                if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
            }
            fn begin_torque_budgeted(&mut self, _enable: bool, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
                self.pending = true;
                Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
            }
            fn begin_torque_budgeted_with_range_retry(&mut self, enable: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
                self.begin_torque_budgeted(enable, max_rx_frames)
            }
            fn begin_disable_torque_budgeted(&mut self, kind: DisableKind, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
                self.pending = true;
                self.protective = matches!(kind, DisableKind::DisableProtective { .. });
                self.dispatched_mask = self.dispatch_mask_on_begin;
                Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
            }
            fn poll_torque_budgeted(&mut self, _deadline: Instant, _max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
                if !self.pending {
                    return Err(ArmDriverError::ProtocolStatic("no torque transition pending"));
                }
                if self.complete_disable_on_poll.load(Ordering::Acquire) {
                    self.pending = false;
                    self.enabled = false;
                    self.dispatched_mask = 0;
                    return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                Ok(TorqueProgress::Pending { wake: Instant::now() + Duration::from_millis(5) })
            }
            fn torque_transition_pending(&self) -> bool { self.pending }
            fn disable_frames_dispatched(&self) -> bool { self.dispatched_mask != 0 }
            fn disable_dispatched_mask(&self) -> u64 { self.dispatched_mask }
            fn abort_enable(&mut self) {
                self.abort_enable_calls += 1;
                self.pending = false;
            }
            fn abort_pending_disable(&mut self) -> bool {
                self.abort_disable_calls += 1;
                if self.protective || self.dispatched_mask != 0 {
                    return false;
                }
                self.pending = false;
                true
            }
        }

        // 1. Stalled-ticker enable timeout aborts driver enable (clearing driver pending state),
        //    clears supervisor transition, and settles slot.
        {
            let mut inner = SupervisorInner::new(crate::ArmConfig::rebot_dm()).with_driver(Box::new(StubPendingDriver {
                dof: 7,
                pending: false,
                dispatched_mask: 0,
                dispatch_mask_on_begin: 0,
                protective: false,
                abort_enable_calls: 0,
                abort_disable_calls: 0,
                complete_disable_on_poll: Arc::new(AtomicBool::new(false)),
                enabled: false,
            }));
            inner.lease_enabled = false;
            inner.tick_control(0.02);
            let stamp = inner.current_admission_stamp();
            let start = inner
                .check_admission(stamp)
                .and_then(|a| {
                    SupervisorInner::begin_enable_torque_transition_legacy(
                        a,
                        Instant::now() + Duration::from_secs(5),
                    )
                })
                .expect("start enable");
            let (token, slot_idx, slot) = match start {
                LegacyTorqueStart::Pending { token, slot_idx, slot } => (token, slot_idx, slot),
                _ => panic!("expected Pending enable"),
            };
            assert!(inner.torque_transition.is_some());
            assert!(inner.driver.as_ref().unwrap().read().unwrap().torque_transition_pending());
            let out = inner.timeout_legacy_torque_transition(token, slot_idx, true);
            assert!(matches!(out, LegacyTorqueOutcome::Failed { .. }), "{out:?}");
            assert!(inner.torque_transition.is_none(), "timed-out enable must not leave orphan transition");
            assert!(
                !inner.driver.as_ref().unwrap().read().unwrap().torque_transition_pending(),
                "driver handshake must actually be cleared on enable timeout"
            );
            assert!(slot.take_settled(token).is_none(), "slot already consumed/released");
        }

        // 2. Dispatched disable timeout does NOT abort driver disable; retags to Detached and
        //    completes to `is_torqued == false` once remaining motors confirm.
        {
            let complete_flag = Arc::new(AtomicBool::new(false));
            let mut inner = SupervisorInner::new(crate::ArmConfig::rebot_dm()).with_driver(Box::new(StubPendingDriver {
                dof: 7,
                pending: false,
                dispatched_mask: 0,
                dispatch_mask_on_begin: 0b0000_0111, // 3 joints dispatched during begin_disable
                protective: false,
                abort_enable_calls: 0,
                abort_disable_calls: 0,
                complete_disable_on_poll: Arc::clone(&complete_flag),
                enabled: true,
            }));
            inner.lease_enabled = false;
            inner.is_torqued = true;
            inner.tick_control(0.02);
            let start = inner
                .begin_disable_torque_transition_legacy(Instant::now() + Duration::from_secs(5))
                .expect("start disable");
            let (token, slot_idx, _slot) = match start {
                LegacyTorqueStart::Pending { token, slot_idx, slot } => (token, slot_idx, slot),
                _ => panic!("expected Pending disable"),
            };
            let out = inner.timeout_legacy_torque_transition(token, slot_idx, false);
            assert!(matches!(
                out,
                LegacyTorqueOutcome::Failed {
                    failure: LegacyTorqueFailure::StaticCode("DEADLINE_EXPIRED"),
                    ..
                }
            ));
            assert!(
                matches!(
                    inner.torque_transition,
                    Some(TorqueTransition::Disable {
                        origin: DisableOrigin::Detached { .. },
                        ..
                    })
                ),
                "dispatched disable must remain active as DisableOrigin::Detached, got {:?}",
                inner.torque_transition
            );
            // Flip stub to confirm remaining joints and verify Detached disable completes cleanly.
            // Use a generous wall-clock budget so CPU contention under parallel test runs cannot
            // expire `budget.deadline` before `StubPendingDriver::poll_torque_budgeted` runs.
            complete_flag.store(true, Ordering::Release);
            let now = Instant::now();
            let budget = crate::rt_core::ops::ServiceBudget {
                now,
                deadline: now + Duration::from_secs(1),
                max_rx_frames: 64,
            };
            let poll = inner
                .service_torque_transition(&budget)
                .expect("expected Detached disable poll result");
            assert!(
                matches!(poll.progress, Ok(crate::rt_core::ops::TorqueProgress::Done)),
                "Detached disable must complete cleanly: {:?}",
                poll.progress
            );
            assert!(inner.torque_transition.is_none());
            assert!(!inner.is_torqued);
        }

        // 3. Protective disable is NEVER aborted by `abort_pending_disable`.
        {
            let mut drv = StubPendingDriver {
                dof: 7,
                pending: false,
                dispatched_mask: 0,
                dispatch_mask_on_begin: 0,
                protective: false,
                abort_enable_calls: 0,
                abort_disable_calls: 0,
                complete_disable_on_poll: Arc::new(AtomicBool::new(false)),
                enabled: true,
            };
            let _ = drv.begin_disable_torque_budgeted(
                DisableKind::DisableProtective {
                    reason: DisableProtectiveReason::Thermal,
                },
                0,
            );
            assert!(
                !drv.abort_pending_disable(),
                "protective disable must NEVER be aborted"
            );
            assert!(drv.torque_transition_pending(), "protective disable must remain live");
        }
    }
}
