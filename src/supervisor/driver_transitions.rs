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
            drv.panic_disable_step(first_step, drv_deadline)
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
            snap.hold_mode_verified[j] = drv.joint_hold_mode_verified(j);
        }
        snap.valid = true;
        snap
    }
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
}

/// Phase of the single budgeted enable transition reducer (`Enable -> Validate -> Armed | FailedHold`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnableReducerPhase {
    /// Driving the driver's enable handshake (`begin_torque` / `poll_torque`).
    Enable,
    /// Driver enable completed; validating fresh feedback and enable state before arming.
    Validate,
}

/// State of the single enable transition reducer shared by blocking `set_torque(true)` and RT
/// `begin_enable_torque_transition` / `service_torque_transition`.
#[derive(Debug, Clone, Copy)]
pub struct EnableReducer {
    pub phase: EnableReducerPhase,
    pub was_torqued: bool,
    pub prev_gripper_cmd: Option<f32>,
    pub authority: Option<EnableAuthority>,
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
/// (`Enable -> Validate -> Armed | FailedHold`) or an explicit/protective disable handshake
/// (rest-pose gate defer + bounded disable repeats within the 1 s request lifetime).
#[derive(Debug, Clone, Copy)]
pub enum TorqueTransition {
    /// Single budgeted enable transition reducer (`Enable -> Validate -> Armed | FailedHold`).
    Enable(EnableReducer),
    /// Driver disable handshake in flight (rest-pose gate wait or bounded disable repeats),
    /// carrying the original `DisableKind` (`started_at`) so any transport-failure background
    /// retry keeps the request's original timestamp and rest-pose gate.
    Disable {
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
        driver.set_strict_mode_handover(self.config.joint_tracking.as_ref().is_some_and(|jt| jt.enabled));
        if driver.capabilities().load_feedback {
            self.loads = vec![f32::NAN; self.config.dof];
        }
        let driver: Box<dyn ArmDriver> = Box::new(crate::joint_zero::ZeroOffsetDriver::new(driver, &self.joint_zero_offsets));
        self.apply_driver_limits(driver.as_ref());
        self.install_driver(driver);
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
            let drv = d
                .lock()
                .map_err(|_| BoundedText::new("driver unavailable while validating gripper calibration"))?;
            if let Some((d_min, d_max)) = drv.position_limits(g_idx) {
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
        self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.feedback_all_within(self.config.dof, max_age)))
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
        self.driver.as_ref().is_none_or(|d| d.lock().is_ok_and(|drv| drv.feedback_joints_within(&arm, max_age)))
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
        // T2: Never silently discard a pending broker `Disable` transition when upgrading to
        // protective disable (e.g. thermal trip). Non-disable transitions (such as an uncancelled
        // `Enable`) end `SUPERSEDED`; a pending `Disable` stays attached to the upgraded driver
        // disable handshake (or completes/fails here) so its broker request receives a terminal
        // delivered exactly once.
        let pending_disable = matches!(self.torque_transition, Some(TorqueTransition::Disable { .. }));
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
        if !pending_disable && self.torque_transition.take().is_some() {
            self.core.begin_transition(Err("SUPERSEDED"));
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
        self.active_disable_kind = Some(kind);
        self.disable_requested();
        if let Some(ref d) = self.driver {
            let mut drv = match d.lock() {
                Ok(g) => g,
                Err(_) => {
                    if pending_disable {
                        self.torque_transition = None;
                        self.core.begin_transition(Err("FEEDBACK_UNAVAILABLE"));
                    }
                    return Err(RequestDisableError {
                        code: "FEEDBACK_UNAVAILABLE",
                        message: crate::safety::BoundedText::new("Feedback unavailable"),
                        refused_by_chokepoint: false,
                    });
                }
            };
            if self.rt_executor_owned {
                let drv_now = drv.clock_now();
                let res = match drv.begin_disable_torque_budgeted(kind, 0) {
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
                        let post_now = drv.clock_now();
                        drop(drv);
                        let code = if pending_disable {
                            self.torque_transition = None;
                            if !refused {
                                let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                            }
                            let code = self.disable_step_failed(e);
                            self.core.begin_transition(Err(code));
                            code
                        } else {
                            if refused {
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
                            refused_by_chokepoint: refused,
                        });
                    }
                };
                let done_now = matches!(res, crate::driver::TorqueProgress::Done(_));
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
                drop(drv);
                if done_now {
                    self.disable_accepted();
                    if pending_disable {
                        self.torque_transition = None;
                        let terminal = self.torque_off_finish_rt().map(|()| Rt::Done);
                        self.core.begin_transition(terminal);
                    }
                } else if pending_disable {
                    self.torque_transition = Some(TorqueTransition::Disable { kind });
                } else {
                    let _ = self.arm_background_disable_retry_with_kind(kind, drv_now);
                }
                return Ok(());
            }
            if let Err(e) = drv.disable_torque(kind) {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                let mut message = crate::safety::BoundedText::empty();
                message.push_str("Torque disable transmission failed: ");
                e.write_bounded(&mut message);
                drop(drv);
                let code = if pending_disable {
                    self.torque_transition = None;
                    let code = self.disable_step_failed(e);
                    self.core.begin_transition(Err(code));
                    code
                } else {
                    if refused {
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
                    refused_by_chokepoint: refused,
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
            drop(drv);
        }
        self.disable_accepted();
        if pending_disable {
            self.torque_transition = None;
            let terminal = self
                .torque_off_finish_in_place()
                .map(|()| Rt::Done)
                .map_err(|e| torque_error_code(&e));
            self.core.begin_transition(terminal);
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

    /// State at the first disable frame: MIT off, `disable_pending` until confirmed.
    fn disable_requested(&mut self) {
        self.disable_requested_at(Instant::now());
    }

    fn disable_requested_at(&mut self, now: Instant) {
        self.mit_torque_off();
        self.disable_pending = true;
        self.tracking.invalidate();
        self.tracking.clear_recovery_suspension();
        self.last_disable_attempt = Some(now);
    }

    /// State once an explicit disable (passing the rest-pose gate) or protective disable has been
    /// accepted and transmitted: records `torque_intent = Off` by decision and stops holding.
    pub(crate) fn disable_accepted(&mut self) {
        self.torque_intent = TorqueIntent::Off;
        self.is_torqued = false;
        self.pending_rearm = false;
        self.disable_pending = false;
        self.active_disable_kind = None;
        self.background_disable_retry = None;
        self.gripper_hold_vel_ceiling = None;
        self.enable_lifecycle = EnableLifecycle::Disarmed;
        self.measured_hold.clear();
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
    fn populate_enable_clear_context(
        &self,
        ctx: &mut crate::safety::ClearContext,
        _state: EnableState,
        _partial: bool,
        enable_fb: bool,
    ) {
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
        self.torque_intent == TorqueIntent::On || self.is_torqued
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
            let _ = driver.request_mode_readback_budgeted(unverified_mask);
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
            for j in 0..dof {
                self.measured_hold.captured_positions[j] = None;
                self.measured_hold.captured_continuity_epoch[j] = None;
                self.measured_hold.last_seen_fresh_at[j] = None;
                self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
            }
            return (0, u64::MAX);
        }
        let snap = self.owner_driver_snapshot;
        let drv_now = snap.captured_at.unwrap_or_else(Instant::now);
        let mut hold_mask = 0u64;
        let mut invalidated_mask = 0u64;
        for j in 0..dof {
            let Some((pos, epoch)) = snap.fresh_samples[j] else {
                self.measured_hold.captured_positions[j] = None;
                self.measured_hold.captured_continuity_epoch[j] = None;
                self.measured_hold.last_seen_fresh_at[j] = None;
                self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
                invalidated_mask |= 1u64 << j;
                continue;
            };
            if !snap.hold_mode_verified[j] {
                self.measured_hold.captured_positions[j] = None;
                self.measured_hold.captured_continuity_epoch[j] = None;
                self.measured_hold.last_seen_fresh_at[j] = None;
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
        let Some(driver_ref) = self.driver.as_ref() else {
            return f(self);
        };
        let addr = (driver_ref as *const std::sync::Mutex<Box<dyn crate::driver::ArmDriver>>) as usize;
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
        if self.mit.owner == crate::mit::MitOwner::Lost {
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
                        self.measured_hold.captured_positions[j] = None;
                        self.measured_hold.captured_continuity_epoch[j] = None;
                        self.measured_hold.last_seen_fresh_at[j] = None;
                        self.measured_hold.joint_status[j] = JointHoldStatus::UnheldNoFreshFeedback;
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
            return TickOutput::None;
        }

        // L3: Non-blocking `try_lock` for stopped-hold TX: on contention, skip this tick's send
        // and increment the diagnostics counter; escalate after `STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS`
        // (25 consecutive skips, 0.5 s at 50 Hz) with a counted warning and a non-recoverable fault
        // (zero heap allocation on RT).
        let mut lock_contended = false;
        let write_result = match self.driver.as_ref() {
            Some(d) => match d.try_lock() {
                Ok(mut drv) => {
                    drv.drop_pending_holds_outside_mask(keep_queued_mask);
                    drv.set_pending_pos_force_hold_hint(true);
                    let r = drv.write_measured_hold_masked(hold_mask, &targets);
                    let now = drv.clock_now();
                    let max_age = drv.tx_frame_max_age();
                    let oldest_pending = drv.oldest_pending_tx_age();
                    let last_expired = drv.last_tx_expired_age();
                    let sent_any_fallback = drv.last_hold_tx_sent_any();
                    let due_mask = drv.last_hold_tx_due_mask();
                    let sent_mask = drv.last_hold_tx_sent_mask();
                    let pf_status = drv.take_last_pos_force_tx_status();
                    let has_pending_holds = drv.pending_tx_len_total() > 0;
                    Some((
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
                    ))
                }
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
        )) = write_result
        else {
            return TickOutput::None;
        };

        let out = match write_res {
            Ok(report) => {
                let n = report.dof.min(dof);
                let mut next_pending_mask = 0u64;
                for j in 0..n {
                    self.measured_hold.joint_status[j] = report.joint_status[j];
                    if report.joint_status[j] == JointHoldStatus::Queued {
                        next_pending_mask |= 1u64 << j;
                    }
                    if !matches!(report.joint_status[j], JointHoldStatus::Held | JointHoldStatus::Queued) {
                        self.measured_hold.captured_positions[j] = None;
                        self.measured_hold.captured_continuity_epoch[j] = None;
                        self.measured_hold.last_seen_fresh_at[j] = None;
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
            });
        }
        true
    }

    #[inline]
    fn background_disable_window_for(&self, kind: crate::driver::DisableKind) -> Duration {
        match kind {
            crate::driver::DisableKind::DisableRequested { .. } if self.torque_off_stopped() => {
                crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME
            }
            _ => BACKGROUND_DISABLE_MAX_WINDOW,
        }
    }

    fn exhaust_background_disable_retry(&mut self) {
        self.background_disable_retry = Some(BackgroundDisableRetry::Exhausted);
        self.safety.latch(
            crate::safety::FaultKind::Other,
            crate::safety::FaultDetail::Static(crate::safety::StaticFault::DisableUnconfirmed),
        );
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
        let mut outcome = self.start_enable_reducer(was_torqued, prev_gripper_cmd, None, false);
        let mut first = true;
        loop {
            match outcome {
                EnableStepOutcome::Armed(state) => return Ok(state),
                EnableStepOutcome::Failed {
                    message,
                    feedback_unavailable,
                    estop_interrupted,
                    ..
                } => {
                    return Err(if feedback_unavailable {
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
                            if let Ok(drv) = d.lock() {
                                drv.clock_pause(Duration::from_micros(500));
                            }
                        }
                    }
                    first = false;
                    let Some(TorqueTransition::Enable(reducer)) = self.torque_transition.take() else {
                        return Err(SafetyError::ClearFaultRejected(
                            "Enable transition reducer state lost".into(),
                        ));
                    };
                    let (next, _) = self.poll_enable_reducer(reducer, None);
                    outcome = next;
                }
            }
        }
    }

    /// Exempt from `Admitted` (Invariant 1c): torque disable is a stop-direction action that
    /// removes actuator output and must never be blocked by a Stop or admission check (still
    /// subject to resting-pose and torque-off safety checks).
    pub fn disable_torque(&mut self) -> Result<SafetyState, SafetyError> {
        if self.torque_transition.is_some() {
            return Err(SafetyError::ClearFaultRejected("A torque transition is pending".into()));
        }
        // Torque changes and re-arming are refused in gravity compensation: exit first.
        self.gravity_comp_servo_busy()?;
        self.torque_off(false)
    }

    /// Enable, before the driver's handshake: every refusal (gravity compensation, driver
    /// limits, handover, pending disable, E-stop, fault, MIT gate), then the feedback read
    /// that seeds the commanded state. Returns `(was_torqued, prev_gripper_cmd)`.
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
        if let Some(d) = self.driver.take() {
            // Secondary double-check latch right before hardware actuation
            if self.estop_pending() {
                self.driver = Some(d);
                return Err(SafetyError::EStopActive);
            }
            let Ok(mut driver) = d.lock() else {
                self.owner_driver_snapshot.invalidate();
                self.driver = Some(d);
                return Err(SafetyError::FeedbackUnavailable);
            };
            if self.estop_pending() {
                drop(driver);
                self.driver = Some(d);
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
            self.refresh_owner_driver_snapshot_from_ref(driver.as_ref());
            drop(driver);
            self.driver = Some(d);
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
        // refresh owner-held driver snapshot in the SAME single lock, capture per-joint holds from
        // owner data (no extra driver lock), and drop queued driver holds outside `hold_mask`.
        if let Some(d) = self.driver.take() {
            match d.lock() {
                Ok(mut drv) => {
                    drv.abort_enable();
                    self.refresh_owner_driver_snapshot_from_ref(drv.as_ref());
                    let (hold_mask, _) = self.capture_measured_hold_from_owner();
                    drv.drop_pending_holds_outside_mask(hold_mask);
                }
                Err(_) => {
                    self.owner_driver_snapshot.invalidate();
                    let _ = self.capture_measured_hold_from_owner();
                }
            }
            self.driver = Some(d);
        } else {
            let _ = self.snapshot_measured_hold();
        }
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
        self.core.begin_transition(Err("THERMAL_TRIP"));
        let _ = self.enter_failed_hold(reducer.was_torqued, |buf| {
            buf.push_str("interrupted by thermal trip");
        });
    }

    /// Zero-allocation RT preflight for torque enable (all refusal paths return static codes).
    fn enable_prepare_rt_precheck(&self) -> Result<(), &'static str> {
        if self.calibration_busy && self.motor_zero_busy {
            return Err("MOTION_BUSY");
        }
        if self.torque_transition.is_some() {
            return Err("TRANSITION_BUSY");
        }
        if self.gravity_comp.is_some() {
            return Err("MOTION_BUSY");
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
        if self.mit_active()
            || self.mit.revoke_pending
            || crate::mit::mit_joint_mask(&self.config) != 0
        {
            return Err("UNSUPPORTED");
        }
        Ok(())
    }

    /// Zero-allocation RT enable prepare (runs after [`Self::enable_prepare_rt_precheck`]).
    /// Never infers `was_torqued` from feedback-reported enable state (R10 decision-owned torque).
    fn enable_prepare_rt(&mut self) -> Result<(bool, Option<f32>), &'static str> {
        self.enable_prepare_rt_precheck()?;
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
            let Ok(driver) = d.lock() else {
                self.owner_driver_snapshot.invalidate();
                self.driver = Some(d);
                return Err("FEEDBACK_UNAVAILABLE");
            };
            if self.estop_pending() {
                drop(driver);
                self.driver = Some(d);
                return Err("ESTOP_ACTIVE");
            }
            self.refresh_owner_driver_snapshot_from_ref(driver.as_ref());
            drop(driver);
            self.driver = Some(d);
        }
        Ok((was_torqued, prev_gripper_cmd))
    }

    /// Starts the single budgeted enable reducer (`Enable -> Validate -> Armed | FailedHold`).
    fn start_enable_reducer(
        &mut self,
        was_torqued: bool,
        prev_gripper_cmd: Option<f32>,
        authority: Option<EnableAuthority>,
        budgeted: bool,
    ) -> EnableStepOutcome {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        let reducer = EnableReducer {
            phase: EnableReducerPhase::Enable,
            was_torqued,
            prev_gripper_cmd,
            authority,
        };
        self.torque_intent = TorqueIntent::On;
        self.enable_lifecycle = EnableLifecycle::Enabling;
        let Some(ref d) = self.driver else {
            return self.step_enable_validate(
                EnableReducer { phase: EnableReducerPhase::Validate, ..reducer },
                None,
                budgeted,
            );
        };
        let mut ages_buf = std::mem::take(&mut self.scratch.ages);
        let mut bounds_buf = std::mem::take(&mut self.scratch.fb.bounds);
        let (started, enable_fb, drv_now, snap) = {
            let Ok(mut drv) = d.lock() else {
                self.scratch.ages = ages_buf;
                self.scratch.fb.bounds = bounds_buf;
                self.owner_driver_snapshot.invalidate();
                let message = self.enter_failed_hold(was_torqued, |buf| {
                    buf.push_str("driver mutex poisoned during enable");
                });
                return EnableStepOutcome::Failed {
                    rt_code: "FEEDBACK_UNAVAILABLE",
                    message,
                    feedback_unavailable: true,
                    estop_interrupted: false,
                };
            };
            let enable_fb = drv.capabilities().enable_feedback;
            let drv_now = drv.clock_now();
            let started = if budgeted {
                drv.begin_torque_budgeted(true, 0)
            } else {
                drv.begin_torque(true)
            };
            let snap = OwnerDriverSnapshot::capture_from_driver(
                drv.as_ref(),
                self.config.dof,
                self.feedback_numeric_valid,
                &self.measured_positions,
                &mut ages_buf,
                &mut bounds_buf,
            );
            (started, enable_fb, drv_now, snap)
        };
        self.scratch.ages = ages_buf;
        self.scratch.fb.bounds = bounds_buf;
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

    /// Advances the single budgeted enable reducer by one poll step (`Enable -> Validate -> Armed | FailedHold`).
    /// Re-verifies live [`EnableAuthority`] BEFORE any subsequent enable retry TX so a displaced
    /// lease, disconnect, E-stop, fault, or epoch change transitions immediately to `FailedHold`
    /// without sending rollback disable frames.
    fn poll_enable_reducer(
        &mut self,
        reducer: EnableReducer,
        budget: Option<&crate::rt_core::ops::ServiceBudget>,
    ) -> (EnableStepOutcome, Option<(u16, bool)>) {
        use crate::driver::{TorqueOutcome, TorqueProgress as Drv};
        if let Some(auth) = reducer.authority {
            let check_now = budget.map_or_else(Instant::now, |b| b.now);
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
            EnableReducerPhase::Validate => (
                self.step_enable_validate(reducer, budget, budget.is_some()),
                None,
            ),
            EnableReducerPhase::Enable => {
                let Some(ref d) = self.driver else {
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
                };
                let mut scratch_states = std::mem::take(&mut self.scratch.states);
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let mut scratch_bounds = std::mem::take(&mut self.scratch.fb.bounds);
                let lock_res = {
                    let Ok(mut drv) = d.lock() else {
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
                    };
                    let caps = drv.capabilities();
                    let enable_fb = caps.enable_feedback;
                    let temp_fb = caps.temperature_feedback;
                    let drv_now = drv.clock_now();
                    let (r, rx) = if let Some(b) = budget {
                        let span = b.deadline.saturating_duration_since(b.now);
                        let r = drv.poll_torque_budgeted(drv_now + span, b.max_rx_frames);
                        let rx = drv.take_last_poll_rx();
                        (r, rx)
                    } else {
                        let r = drv.poll_torque(drv_now + crate::driver::DAMIAO_RX_PASS);
                        (r, None)
                    };
                    let mut cached_temps_read = false;
                    if temp_fb && drv.read_cached_joint_states_into(&mut scratch_states).is_some() {
                        drv.feedback_ages_into(&mut scratch_ages);
                        cached_temps_read = true;
                    }
                    let snap = OwnerDriverSnapshot::capture_from_driver(
                        drv.as_ref(),
                        self.config.dof,
                        self.feedback_numeric_valid,
                        &self.measured_positions,
                        &mut scratch_ages,
                        &mut scratch_bounds,
                    );
                    (r, rx, enable_fb, temp_fb, drv_now, cached_temps_read, snap)
                };
                let (poll_res, rx, enable_fb, temp_fb, drv_now, cached_temps_read, snap) = lock_res;
                self.owner_driver_snapshot = snap;
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
                        let rt_code = enable_driver_err_code(&e);
                        let estop_interrupted = crate::driver::is_estop_inhibited(&e);
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
        let check_now = budget.map_or_else(Instant::now, |b| b.now);
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

        if self.driver.is_some() {
            let mut scratch_states = std::mem::take(&mut self.scratch.states);
            let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
            let mut scratch_loads = std::mem::take(&mut self.scratch.loads);
            let (validate_res, temp_fb) = {
                let d = self.driver.as_ref().expect("checked is_some");
                match d.lock() {
                    Err(_) => (Err("driver mutex poisoned during validation"), false),
                    Ok(mut driver) => {
                        let temp_fb = driver.capabilities().temperature_feedback;
                        let read_res: Result<(), ()> = if use_cached {
                            if driver.read_cached_joint_states_into(&mut scratch_states).is_some() {
                                Ok(())
                            } else if let Some(b) = budget {
                                let span = b.deadline.saturating_duration_since(b.now);
                                let drv_deadline = driver.clock_now() + span;
                                driver
                                    .read_joint_states_budgeted(&mut scratch_states, drv_deadline, b.max_rx_frames)
                                    .map(|_| ())
                                    .map_err(|_| ())
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
                                (Err("post-enable feedback invalid or enable unconfirmed"), temp_fb)
                            } else {
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
                                self.owner_driver_snapshot = OwnerDriverSnapshot::capture_from_driver(
                                    driver.as_ref(),
                                    self.config.dof,
                                    self.feedback_numeric_valid,
                                    &self.measured_positions,
                                    &mut scratch_ages,
                                    &mut self.scratch.fb.bounds,
                                );
                                (Ok(()), temp_fb)
                            }
                        }
                    }
                }
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

        if let Some(auth) = reducer.authority {
            let post_now = budget.map_or_else(Instant::now, |b| b.now);
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
                    self.mit.armed_ok = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
                    self.mit.handover_error = None;
                    self.last_heartbeat = armed_now;
                    self.last_motion_frame = armed_now;
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
                    self.mit.armed_ok = self.config.joint_tracking.as_ref().is_some_and(|jt| jt.mit_requested());
                    self.mit.handover_error = None;
                    self.last_heartbeat = armed_now;
                    self.last_motion_frame = armed_now;
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
        self.torque_off_check(stillness_verified)?;
        let started_at = self.driver_clock_now();
        let disable_kind = crate::driver::DisableKind::DisableRequested {
            started_at,
            rollback: false,
        };
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        if let Err(error) = self.request_disable(disable_kind) {
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

    /// True when the driver can keep a stopped disable pending inside its 1 s lifetime while an
    /// exhausted RX drain or incomplete K=3 rest-pose window completes (L1). Zero heap allocations.
    fn can_defer_stopped_disable_to_driver(&self) -> bool {
        self.driver.as_ref().is_some_and(|d| {
            d.lock().is_ok_and(|drv| {
                drv.supports_budgeted_disable_await()
                    && drv.rest_pose_gate_status()
                        == crate::driver::RestPoseGateStatus::PendingDrainOrWindow
            })
        })
    }

    /// Zero-allocation RT torque-off precheck (static protocol error codes): enforces Rule A′
    /// (`eval_stopped_torque_off`) when stopped and zero-alloc resting-pose check otherwise.
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
        if self.torque_off_stopped() {
            return self
                .eval_stopped_torque_off()
                .map_err(|_| "TORQUE_OFF_WOULD_DROP");
        }
        if self.has_resting_pose_violation_rt(stillness_verified) {
            return Err("ARM_NOT_AT_RESTING_POSE");
        }
        Ok(())
    }

    /// Zero-allocation stopped torque-off gate for RT budgeted transitions (L1): defers to the
    /// driver's `AwaitRestGate` when the ONLY blocker is an incomplete RX drain or incomplete K=3
    /// rest-pose window; immediately refuses non-rest poses, missing rest-pose policies, or
    /// stale/silent buses.
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
        if self.torque_off_stopped() {
            match self.eval_stopped_torque_off() {
                Ok(()) => return Ok(()),
                Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)
                    if self.can_defer_stopped_disable_to_driver() =>
                {
                    self.last_stopped_torque_off_refusal = None;
                    return Ok(());
                }
                Err(_) => return Err("TORQUE_OFF_WOULD_DROP"),
            }
        }
        if self.has_resting_pose_violation_rt(false) {
            return Err("ARM_NOT_AT_RESTING_POSE");
        }
        Ok(())
    }

    /// The refusals of a torque off: E-stop / fault (rest-pose gated), not at the resting pose.
    fn torque_off_check(&mut self, stillness_verified: bool) -> Result<(), SafetyError> {
        if self.torque_off_stopped() {
            match self.eval_stopped_torque_off() {
                Ok(()) => return Ok(()),
                Err(crate::supervisor::StoppedTorqueOffRefusal::StaleOrMissingFeedback)
                    if self.can_defer_stopped_disable_to_driver() =>
                {
                    self.last_stopped_torque_off_refusal = None;
                    return Ok(());
                }
                Err(r) => return Err(r.into_safety_error(&self.config.joint_names)),
            }
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
        self.core.begin_transition(Err(code));
    }

    /// A transition is in flight ([`TorqueTransition`]): ordinary output and blocking torque
    /// changes wait for it.
    pub fn torque_transition_pending(&self) -> bool { self.torque_transition.is_some() }

    /// Starts a torque-enable transition without waiting for its confirmation (the RT's
    /// `commit_transition`). Requires a valid [`AdmissionStamp`] checked first under the owner lock
    /// and drives the single authoritative budgeted enable reducer (`Enable -> Validate -> Armed | FailedHold`)
    /// carrying live [`EnableAuthority`] checks (lease generation, owner conn, epoch) into `FailedHold`.
    pub(crate) fn begin_enable_torque_transition(
        admitted: Admitted<'_>,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        let (this, proof) = admitted
            .consume()
            .map_err(|e| torque_admission_error_code(&e))?;
        let s = proof.stamp();
        let authority = EnableAuthority {
            lease_generation: s.lease_gen,
            owner_conn_id: s.connection_id,
            admitted_epoch: s.estop_epoch,
        };
        this.begin_enable_torque_transition_inner(authority)
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
        };
        let (was_torqued, prev_gripper_cmd) = this.enable_prepare_rt()?;
        let _ = now;
        match this.start_enable_reducer(was_torqued, prev_gripper_cmd, Some(authority), true) {
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
        self.disable_accepted();
        self.torque_off_finish_rt()
    }

    fn begin_enable_torque_transition_inner(
        &mut self,
        authority: EnableAuthority,
    ) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
        use crate::rt_core::ops::TorqueProgress as Rt;
        let (was_torqued, prev_gripper_cmd) = self.enable_prepare_rt()?;
        Ok(match self.start_enable_reducer(was_torqued, prev_gripper_cmd, Some(authority), true) {
            EnableStepOutcome::Armed(_) => Ok(Rt::Done),
            EnableStepOutcome::Pending { wake } => Ok(Rt::Pending { wake }),
            EnableStepOutcome::Failed { rt_code, .. } => Err(rt_code),
        })
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
        self.torque_off_check_rt_transition()?;
        self.stop_motion();
        self.commanded_velocities.fill(0.0);
        self.background_disable_retry = None;
        let started = match self.driver.as_ref() {
            None => {
                self.disable_requested();
                None
            }
            Some(d) => {
                let (res, requested_at, post_now, disable_kind) = {
                    let mut drv = d.lock().map_err(|_| "FEEDBACK_UNAVAILABLE")?;
                    let requested_at = drv.clock_now();
                    let disable_kind = crate::driver::DisableKind::DisableRequested {
                        started_at: requested_at,
                        rollback: false,
                    };
                    let res = drv.begin_disable_torque_budgeted(disable_kind, 0);
                    let post_now = drv.clock_now();
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
                    (res, requested_at, post_now, disable_kind)
                };
                self.disable_requested_at(requested_at);
                Some((res, post_now, disable_kind))
            }
        };
        if let Some((_, _, disable_kind)) = started {
            self.active_disable_kind = Some(disable_kind);
        }
        Ok(match started {
            None => {
                self.disable_accepted();
                self.torque_off_finish_rt().map(|()| Rt::Done)
            }
            Some((Ok(crate::driver::TorqueProgress::Pending { .. }), post_now, disable_kind)) => {
                self.torque_transition = Some(TorqueTransition::Disable { kind: disable_kind });
                Ok(Rt::Pending { wake: post_now })
            }
            Some((Ok(crate::driver::TorqueProgress::Done(_)), _, _)) => {
                self.disable_accepted();
                self.torque_off_finish_rt().map(|()| Rt::Done)
            }
            Some((Err(e), post_now, disable_kind)) => {
                let refused = matches!(
                    e,
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                        | crate::driver::ArmDriverError::DisableRequestExpired
                );
                if !refused {
                    let _ = self.arm_background_disable_retry_with_kind(disable_kind, post_now);
                }
                Err(self.disable_step_failed(e))
            }
        })
    }

    /// A disable step failed: `disable_pending` stays set for transport errors (and
    /// `DRIVER_WRITE_FAILED` latches when not already stopped), while a choke-point refusal
    /// (`TorqueOffWouldDrop` / `DisableRequestExpired`) cleanly ends the request.
    fn disable_step_failed(&mut self, e: crate::driver::ArmDriverError) -> &'static str {
        let refused = matches!(
            e,
            crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired
        );
        if refused {
            self.disable_pending = false;
            self.active_disable_kind = None;
            self.last_disable_attempt = None;
            self.background_disable_retry = None;
            if self.torque_off_stopped() {
                let _ = self.eval_stopped_torque_off();
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
    pub(crate) fn service_background_disable(
        &mut self,
        budget: &crate::rt_core::ops::ServiceBudget,
    ) -> Option<(u16, bool)> {
        if !self.disable_pending || self.torque_transition.is_some() {
            return None;
        }
        let d = self.driver.as_ref()?;
        let mut drv = d.lock().ok()?;
        if self.control_release_seq > 0 {
            drv.begin_control_release(self.control_release_seq);
        }
        if matches!(
            self.background_disable_retry,
            Some(BackgroundDisableRetry::Exhausted)
        ) {
            drop(drv);
            return None;
        }
        let drv_now = drv.clock_now();
        let span = budget.deadline.saturating_duration_since(budget.now);

        if drv.torque_transition_pending() {
            let poll_res = drv.poll_torque_budgeted(drv_now + span, budget.max_rx_frames);
            let rx = drv.take_last_poll_rx();
            let post_now = drv.clock_now();
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
            let still_pending = drv.torque_transition_pending();
            drop(drv);
            match poll_res {
                Ok(crate::driver::TorqueProgress::Done(_)) => {
                    self.disable_accepted();
                }
                Err(
                    crate::driver::ArmDriverError::TorqueOffWouldDrop
                    | crate::driver::ArmDriverError::DisableRequestExpired,
                ) => {
                    self.disable_pending = false;
                    self.active_disable_kind = None;
                    self.last_disable_attempt = None;
                    self.background_disable_retry = None;
                }
                Ok(crate::driver::TorqueProgress::Pending { .. }) => {
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
                    match self.background_disable_retry {
                        Some(BackgroundDisableRetry::Active {
                            attempts,
                            kind,
                            started_at,
                            ..
                        }) => {
                            let window = self.background_disable_window_for(kind);
                            let window_expired =
                                post_now.saturating_duration_since(started_at) >= window;
                            if window_expired
                                && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
                                && self.torque_off_stopped()
                            {
                                self.disable_pending = false;
                                self.active_disable_kind = None;
                                self.last_disable_attempt = None;
                                self.background_disable_retry = None;
                            } else if window_expired
                                || (!still_pending && attempts >= BACKGROUND_DISABLE_MAX_RETRIES)
                            {
                                self.exhaust_background_disable_retry();
                            } else if !still_pending {
                                self.background_disable_retry =
                                    Some(BackgroundDisableRetry::Active {
                                        attempts,
                                        kind,
                                        started_at,
                                        next_retry_at: post_now + BACKGROUND_DISABLE_RETRY_INTERVAL,
                                    });
                            }
                        }
                        Some(BackgroundDisableRetry::Exhausted) => {}
                        None => {
                            let kind = self.active_disable_kind.unwrap_or(
                                crate::driver::DisableKind::DisableRequested {
                                    started_at: post_now,
                                    rollback: false,
                                },
                            );
                            let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                        }
                    }
                }
            }
            return rx;
        }

        let Some(BackgroundDisableRetry::Active {
            attempts,
            kind,
            started_at,
            next_retry_at,
        }) = self.background_disable_retry
        else {
            drop(drv);
            return None;
        };
        let window = self.background_disable_window_for(kind);
        let window_expired = drv_now.saturating_duration_since(started_at) >= window;
        if window_expired
            && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
            && self.torque_off_stopped()
        {
            drop(drv);
            self.disable_pending = false;
            self.active_disable_kind = None;
            self.last_disable_attempt = None;
            self.background_disable_retry = None;
            return None;
        }
        if attempts >= BACKGROUND_DISABLE_MAX_RETRIES || window_expired {
            drop(drv);
            self.exhaust_background_disable_retry();
            return None;
        }
        if drv_now < next_retry_at {
            drop(drv);
            return None;
        }

        let res = drv.begin_disable_torque_budgeted(kind, 0);
        let handshake_pending = drv.torque_transition_pending();
        let post_now = drv.clock_now();
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
        drop(drv);

        match res {
            Ok(crate::driver::TorqueProgress::Done(_)) => {
                self.disable_accepted();
                return None;
            }
            Err(
                crate::driver::ArmDriverError::TorqueOffWouldDrop
                | crate::driver::ArmDriverError::DisableRequestExpired,
            ) => {
                self.disable_pending = false;
                self.active_disable_kind = None;
                self.last_disable_attempt = None;
                self.background_disable_retry = None;
                return None;
            }
            Ok(crate::driver::TorqueProgress::Pending { .. }) | Err(_) => {}
        }

        let next_attempts = attempts.saturating_add(1);
        let window_expired = post_now.saturating_duration_since(started_at) >= window;
        if window_expired
            && matches!(kind, crate::driver::DisableKind::DisableRequested { .. })
            && self.torque_off_stopped()
        {
            self.disable_pending = false;
            self.active_disable_kind = None;
            self.last_disable_attempt = None;
            self.background_disable_retry = None;
        } else if window_expired || (!handshake_pending && next_attempts >= BACKGROUND_DISABLE_MAX_RETRIES)
        {
            self.exhaust_background_disable_retry();
        } else {
            self.background_disable_retry = Some(BackgroundDisableRetry::Active {
                attempts: next_attempts,
                kind,
                started_at,
                next_retry_at: post_now + BACKGROUND_DISABLE_RETRY_INTERVAL,
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
        use crate::driver::TorqueProgress as Drv;
        use crate::rt_core::ops::TorqueProgress as Rt;
        let t = self.torque_transition.take()?;
        match t {
            TorqueTransition::Enable(reducer) => {
                let (outcome, rx) = self.poll_enable_reducer(reducer, Some(budget));
                let progress = match outcome {
                    EnableStepOutcome::Armed(_) => Ok(Rt::Done),
                    EnableStepOutcome::Pending { wake } => Ok(Rt::Pending { wake }),
                    EnableStepOutcome::Failed { rt_code, .. } => Err(rt_code),
                };
                Some(TransitionPoll { progress, rx })
            }
            TorqueTransition::Disable { kind } => {
                let Some(d) = self.driver.as_ref() else {
                    return Some(TransitionPoll { progress: Err("INTERNAL"), rx: None });
                };
                let Ok(mut drv) = d.lock() else {
                    return Some(TransitionPoll { progress: Err("FEEDBACK_UNAVAILABLE"), rx: None });
                };
                if self.control_release_seq > 0 {
                    drv.begin_control_release(self.control_release_seq);
                }
                let drv_now = drv.clock_now();
                let span = budget.deadline.saturating_duration_since(budget.now);
                let to_exec = |wake: Instant| budget.now + wake.saturating_duration_since(drv_now);
                let r = drv.poll_torque_budgeted(drv_now + span, budget.max_rx_frames);
                let rx = drv.take_last_poll_rx();
                let post_now = drv.clock_now();
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
                drop(drv);
                match r {
                    Ok(Drv::Pending { wake }) => {
                        self.torque_transition = Some(TorqueTransition::Disable { kind });
                        Some(TransitionPoll { progress: Ok(Rt::Pending { wake: to_exec(wake) }), rx })
                    }
                    Ok(Drv::Done(_)) => {
                        self.disable_accepted();
                        let progress = self.torque_off_finish_rt().map(|()| Rt::Done);
                        Some(TransitionPoll { progress, rx })
                    }
                    Err(e) => {
                        let refused = matches!(
                            e,
                            crate::driver::ArmDriverError::TorqueOffWouldDrop
                                | crate::driver::ArmDriverError::DisableRequestExpired
                        );
                        if !refused {
                            let _ = self.arm_background_disable_retry_with_kind(kind, post_now);
                        }
                        Some(TransitionPoll { progress: Err(self.disable_step_failed(e)), rx })
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
            if let Ok(drv) = d.lock() {
                ctx.mit_modes_uncertain = self.mit.owner == crate::mit::MitOwner::Lost && drv.joint_modes_uncertain();
                ctx.thermal_sensor_available = drv.capabilities().temperature_feedback;
                ctx.feedback_fresh = self.feedback_numeric_valid
                    && drv.is_feedback_valid()
                    && drv.feedback_all_within(self.config.dof, Duration::from_millis(250));
                ctx.j7_feedback_fresh = drv.pos_force_feedback_fresh();
                ctx.tx_queue_drained = drv.pending_tx_len_total() == 0;
                ctx.tx_probe_confirmed = drv.tx_probe_confirmed();
                ctx.tx_probe_sent = drv.tx_probe_pending();
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
                        let tx_queue_drained = drv.pending_tx_len_total() == 0;
                        let tx_probe_confirmed = drv.tx_probe_confirmed();
                        let tx_probe_sent = drv.tx_probe_pending();
                        let fb_fresh = drv.is_feedback_valid()
                            && ages.len() == self.config.dof
                            && ages.iter().all(|a| a.is_some_and(|t| t <= THERMAL_FEEDBACK_FRESHNESS_LIMIT))
                            && Self::states_valid(&states, caps, self.config.dof, loads.as_deref());
                        let j7_fb_fresh = drv.pos_force_feedback_fresh();
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
                ).map_err(ClearFaultLegacyInnerError::Safety);
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
        if let Some(ref d) = self.driver {
            let mut ws_bufs = std::mem::take(&mut self.ws_eval_bufs);
            let mut drv = match d.lock() {
                Ok(g) => g,
                Err(_) => {
                    self.ws_eval_bufs = ws_bufs;
                    return Err(ClearFaultLegacyInnerError::Safety(SafetyError::FeedbackUnavailable));
                }
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
            let mut state = drv.enable_state();
            let mut partial = drv.has_known_partial_enable();
            let mut any_enabled = drv.has_any_joint_enabled();
            let fb_valid = drv.is_feedback_valid();
            let mut j7_fb_fresh = drv.pos_force_feedback_fresh();
            let tx_queue_drained = drv.pending_tx_len_total() == 0;
            let tx_probe_confirmed = drv.tx_probe_confirmed();
            let tx_probe_sent = drv.tx_probe_pending();
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

            // Stage sampled feedback and thermal cooldown WITHOUT mutating supervisor state
            // before `clear_check` / `clear_with_check` succeeds.
            let dof = self.config.dof;
            let staged_positions: Vec<f32> = states.iter().take(dof).map(|s| s.position).collect();
            let staged_velocities: Vec<f32> = states.iter().take(dof).map(|s| s.velocity).collect();
            let staged_torques: Vec<f32> = states.iter().take(dof).map(|s| s.torque).collect();
            let staged_loads_vec: Vec<f32> = match loads.as_deref() {
                Some(ls) => ls.iter().take(dof).copied().collect(),
                None => self.loads.clone(),
            };
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

            let mut resting_feedback_missing = false;
            let mut resting_pose_violation = None;
            if needs_resting_check {
                if !resting_fb_ok {
                    resting_feedback_missing = true;
                } else {
                    resting_pose_violation = self.resting_pose_support_violation_for_feedback(
                        Some(drv.as_ref()),
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
            drop(drv);
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
            self.populate_enable_clear_context(&mut ctx, state, partial, caps.enable_feedback);

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any TX probe, motor
            // reset, or `refresh_enable_state` side effect.
            let mut clear_res = self.safety.clear_check(&ctx);
            if clear_res == Err(ClearRefusal::TxProbeRequired) {
                // Every OTHER recorded kind's condition passed and the software TX queue is empty:
                // run one read-only refresh/status TX probe and verify fresh post-probe feedback.
                let (probe_res, drained_after, probe_ok, probe_pending, hw_fault_after, st_after, partial_after, any_en_after, fb_valid_after, j7_fresh_after, ages_after) = {
                    let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                    if let Some(auth) = authority {
                        if let Err(auth_err) =
                            recheck_clear_fault_authority_readonly(self, auth, Instant::now())
                        {
                            drop(drv);
                            return Err(ClearFaultLegacyInnerError::Authority(auth_err));
                        }
                    }
                    let res = drv.probe_tx_ready();
                    (
                        res,
                        drv.pending_tx_len_total() == 0,
                        drv.tx_probe_confirmed(),
                        drv.tx_probe_pending(),
                        drv.check_hardware_status().is_err(),
                        drv.enable_state(),
                        drv.has_known_partial_enable(),
                        drv.has_any_joint_enabled(),
                        drv.is_feedback_valid(),
                        drv.pos_force_feedback_fresh(),
                        drv.feedback_ages(),
                    )
                };
                state = st_after;
                partial = partial_after;
                any_enabled = any_en_after;
                j7_fb_fresh = j7_fresh_after;
                ctx.tx_queue_drained = drained_after;
                self.populate_enable_clear_context(&mut ctx, state, partial, caps.enable_feedback);
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
                    // under the SAME driver lock guard before sending the reset (zero gap).
                    let reset_res = {
                        let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                        if let Some(auth) = authority {
                            if let Err(auth_err) =
                                recheck_clear_fault_authority_readonly(self, auth, Instant::now())
                            {
                                drop(drv);
                                return Err(ClearFaultLegacyInnerError::Authority(auth_err));
                            }
                        }
                        if needs_resting_check {
                            if let Err(r) =
                                Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref())
                            {
                                drop(drv);
                                self.last_stopped_torque_off_refusal = Some(r);
                                return Err(ClearFaultLegacyInnerError::Safety(
                                    r.into_safety_error(&self.config.joint_names),
                                ));
                            }
                        }
                        ctx.j7_feedback_fresh = drv.pos_force_feedback_fresh();
                        ctx.feedback_fresh = ctx.feedback_fresh && drv.is_feedback_valid();
                        ctx.pending_estop = self.estop_request_unapplied();
                        if let Err(refusal) = self.safety.clear_check(&ctx) {
                            if refusal != ClearRefusal::HardwareResetRequired {
                                drop(drv);
                                if refusal == ClearRefusal::EStopActive {
                                    return Err(apply_estop_refusal(self));
                                }
                                return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                            }
                        }
                        drv.clear_fault()
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
                Err(ClearRefusal::EnableStateUnknown) if fb_fresh && !hw_fault && !partial => {
                    // Every other condition passed; probe enable state once and re-run clear_check.
                    {
                        let mut drv = d.lock().map_err(|_| SafetyError::FeedbackUnavailable)?;
                        if let Some(auth) = authority {
                            if let Err(auth_err) =
                                recheck_clear_fault_authority_readonly(self, auth, Instant::now())
                            {
                                drop(drv);
                                return Err(ClearFaultLegacyInnerError::Authority(auth_err));
                            }
                        }
                        let _ = drv.refresh_enable_state();
                        state = drv.enable_state();
                        partial = drv.has_known_partial_enable();
                        any_enabled = drv.has_any_joint_enabled();
                        j7_fb_fresh = drv.pos_force_feedback_fresh();
                    }
                    self.populate_enable_clear_context(&mut ctx, state, partial, caps.enable_feedback);
                    ctx.j7_feedback_fresh = j7_fb_fresh;
                    ctx.pending_estop = self.estop_request_unapplied();
                    if let Err(refusal) = self.safety.clear_check(&ctx) {
                        if refusal == ClearRefusal::EStopActive {
                            return Err(apply_estop_refusal(self));
                        }
                        return Err(ClearFaultLegacyInnerError::Safety(refusal.into_safety_error()));
                    }
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
            let _ = any_enabled;
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
                    if let Ok(drv) = d.lock() {
                        this.core.poll_feedback(
                            &mut this.joint_feedback_queue_bounds,
                            this.config.dof,
                            drv.as_ref(),
                            &mut this.scratch.fb,
                        );
                    }
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
        if self.shutdown_committed {
            return Err(ClearFaultWithAuthError::Clear(ClearFaultError::Rejected(ClearRefusal::ShutdownCommitted)));
        }
        // Build live pre-commit ClearContext right before committing so any concurrent E-stop,
        // authority revocation (M3), or J7 freshness expiry between preflight/probe and commit
        // fails closed with ZERO mutations.
        let mut final_ctx = *ctx;
        let was_estop = *self.safety.state() == SafetyState::EStop;
        let state = if let Some(ref d) = self.driver {
            let drv = match d.lock() {
                Ok(g) => g,
                Err(_) => {
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
                final_ctx.j7_feedback_fresh = drv.pos_force_feedback_fresh();
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
        if let Some(ref d) = self.driver {
            if let Ok(mut drv) = d.lock() {
                drv.discard_pending_tx();
                drv.clear_tx_probe();
            }
        }
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
        use crate::safety::{RestingSupportJointItem as Item, RestingSupportViolation as V, MAX_RESTING_SUPPORT_ITEMS};
        let dof = self.config.dof;
        if positions.len() != dof || velocities.len() != dof {
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
        match self.clear_fault_executor_inner(None, deadline, expected_epoch, None) {
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
        let tx_deadline = release_deadline
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        match self.clear_fault_executor_inner(Some(now), tx_deadline, expected_epoch, None) {
            Ok(state) => Ok(state),
            Err(ClearFaultWithAuthError::Clear(e)) => Err(e),
            Err(ClearFaultWithAuthError::Authority(_)) => {
                unreachable!("authority is None on clear_fault_executor_bounded_for_release")
            }
        }
    }

    /// Release-bounded, allocation-free fault clearing carrying M3 live owner authority through
    /// driver preflight, hardware reset (`drv.clear_fault_budgeted`), and final `safety.clear_with_check`.
    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    pub(crate) fn clear_fault_executor_bounded_for_release_with_authority(
        &mut self,
        now: Instant,
        release_deadline: Instant,
        authority: crate::rt_core::requests::lease::ClearFaultAuthority,
    ) -> Result<SafetyState, ClearFaultWithAuthError> {
        let tx_deadline = release_deadline
            .checked_sub(self.tx_release_publish_margin())
            .unwrap_or(now);
        self.clear_fault_executor_inner(
            Some(now),
            tx_deadline,
            Some(authority.expected_epoch),
            Some(authority),
        )
    }

    #[allow(clippy::result_large_err)] // inline refusal keeps RT clear_fault zero-alloc
    fn clear_fault_executor_inner(
        &mut self,
        now_override: Option<Instant>,
        deadline: Instant,
        expected_epoch: Option<u64>,
        authority: Option<crate::rt_core::requests::lease::ClearFaultAuthority>,
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
                    return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
                }
            };
            // Pure read-only driver snapshot FIRST — ZERO motor resets or mutations.
            drv.feedback_ages_into(&mut scratch_ages);
            let ages_valid = scratch_ages.len() == self.config.dof
                && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
            let fb_valid = drv.is_feedback_valid();
            let j7_fb_fresh = drv.pos_force_feedback_fresh();
            let tx_queue_drained = drv.pending_tx_len_total() == 0;
            let tx_probe_confirmed = drv.tx_probe_confirmed();
            let tx_probe_sent = drv.tx_probe_pending();
            let hw_fault = drv.has_hardware_fault();
            let caps = drv.capabilities();
            let temp_fb = caps.temperature_feedback;
            let enable_fb = caps.enable_feedback;
            let mut state = drv.enable_state();
            let mut partial = drv.has_known_partial_enable();
            let mut any_enabled = drv.has_any_joint_enabled();
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
            self.populate_enable_clear_context(&mut ctx, state, partial, enable_fb);

            // Evaluate `clear_check` FIRST over EVERY recorded kind before any TX probe, motor
            // reset, or concurrent E-stop / config mutation.
            let mut clear_res = self.safety.clear_check(&ctx);
            if clear_res == Err(R::TxProbeRequired) {
                let now = now_override.unwrap_or_else(Instant::now);
                if now >= deadline {
                    return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                }
                let mut scratch_ages = std::mem::take(&mut self.scratch.ages);
                let (probe_res, drained_after, probe_ok, probe_pending, hw_fault_after, st_after, partial_after, any_en_after, fb_valid_after, j7_fresh_after, ages_valid_after) = {
                    let mut drv = match d.lock() {
                        Ok(g) => g,
                        Err(_) => {
                            self.scratch.ages = scratch_ages;
                            return Err(ClearFaultWithAuthError::Clear(E::FeedbackUnavailable));
                        }
                    };
                    if let Some(auth) = authority {
                        if let Err(auth_err) = recheck_clear_fault_authority_readonly(self, auth, now) {
                            drop(drv);
                            self.scratch.ages = scratch_ages;
                            return Err(ClearFaultWithAuthError::Authority(auth_err));
                        }
                    }
                    let drv_deadline = drv.clock_now() + deadline.saturating_duration_since(now);
                    drv.set_tx_release_deadline(Some(drv_deadline));
                    let res = drv.probe_tx_ready_budgeted(drv_deadline);
                    drv.set_tx_release_deadline(None);
                    drv.feedback_ages_into(&mut scratch_ages);
                    let ages_ok = scratch_ages.len() == self.config.dof
                        && !scratch_ages.iter().any(|a| a.is_none_or(|t| t > THERMAL_FEEDBACK_FRESHNESS_LIMIT));
                    (
                        res,
                        drv.pending_tx_len_total() == 0,
                        drv.tx_probe_confirmed(),
                        drv.tx_probe_pending(),
                        drv.has_hardware_fault(),
                        drv.enable_state(),
                        drv.has_known_partial_enable(),
                        drv.has_any_joint_enabled(),
                        drv.is_feedback_valid(),
                        drv.pos_force_feedback_fresh(),
                        ages_ok,
                    )
                };
                self.scratch.ages = scratch_ages;
                state = st_after;
                partial = partial_after;
                any_enabled = any_en_after;
                ctx.tx_queue_drained = drained_after;
                self.populate_enable_clear_context(&mut ctx, state, partial, enable_fb);
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
                    // the SAME driver lock guard before sending the reset (zero gap, zero alloc).
                    let now = now_override.unwrap_or_else(Instant::now);
                    if now >= deadline {
                        return Err(ClearFaultWithAuthError::Clear(E::Rejected(R::BudgetExpired)));
                    }
                    let reset_res = {
                        let mut drv = d.lock().map_err(|_| ClearFaultWithAuthError::Clear(E::FeedbackUnavailable))?;
                        if let Some(auth) = authority {
                            if let Err(auth_err) =
                                recheck_clear_fault_authority_readonly(self, auth, now)
                            {
                                drop(drv);
                                return Err(ClearFaultWithAuthError::Authority(auth_err));
                            }
                        }
                        if needs_resting_check {
                            if let Err(refusal) =
                                Self::check_stopped_torque_off_with_driver(&self.config, drv.as_ref())
                            {
                                drop(drv);
                                self.last_stopped_torque_off_refusal = Some(refusal);
                                return Err(ClearFaultWithAuthError::Clear(
                                    refusal.into_clear_fault_error(),
                                ));
                            }
                        }
                        ctx.j7_feedback_fresh = drv.pos_force_feedback_fresh();
                        ctx.feedback_fresh = ctx.feedback_fresh && drv.is_feedback_valid();
                        ctx.pending_estop = self.estop_request_unapplied();
                        if let Err(refusal) = self.safety.clear_check(&ctx) {
                            if refusal != R::HardwareResetRequired {
                                drop(drv);
                                if refusal == R::EStopActive {
                                    return Err(apply_estop_refusal(self));
                                }
                                return Err(ClearFaultWithAuthError::Clear(E::from(refusal)));
                            }
                        }
                        let drv_deadline = drv.clock_now() + deadline.saturating_duration_since(now);
                        drv.set_tx_release_deadline(Some(drv_deadline));
                        let res = drv.clear_fault_budgeted(drv_deadline);
                        drv.set_tx_release_deadline(None);
                        res
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
            let _ = any_enabled;
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
        self.clear_fault_finish_bounded_with_authority(&ctx, commit, authority, now_override, |_| {})
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

    fn mark_panic_disable_requested(&mut self) {
        self.estop_state.latch();
        self.active_disable_kind = Some(crate::driver::DisableKind::DisableProtective {
            reason: crate::driver::DisableProtectiveReason::ProcessExit,
        });
        self.disable_requested();
    }

    /// Bounded last-chance disable on a panic or forced teardown path owned by this `SupervisorInner`.
    /// Uses bounded `try_lock()` so a contended driver mutex cannot block indefinitely past `timeout`.
    /// Runs synchronously on the caller holding `&mut SupervisorInner` (preserving exclusive driver
    /// ownership) while logging timeout independently at `hard_deadline`.
    pub fn attempt_panic_disable(&mut self, timeout: Duration) -> PanicDisableOutcome {
        self.mark_panic_disable_requested();
        let origin = Instant::now();
        let (coop_deadline, hard_deadline) = panic_disable_deadlines(origin, timeout);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        run_with_panic_deadline_observer_until(
            hard_deadline,
            || self.attempt_panic_disable_until(coop_deadline, hard_deadline, coop_budget),
            |_| true,
            move || {
                PanicDisableOutcome::TimedOut.log("Bounded panic disable", timeout);
            },
        )
        .unwrap_or(PanicDisableOutcome::TimedOut)
    }

    pub(crate) fn attempt_panic_disable_until(
        &mut self,
        coop_deadline: Instant,
        hard_deadline: Instant,
        coop_budget: Duration,
    ) -> PanicDisableOutcome {
        self.mark_panic_disable_requested();
        let Some(ref d) = self.driver else {
            self.disable_accepted();
            return PanicDisableOutcome::NoDriver;
        };
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut drv = loop {
                match d.try_lock() {
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
        }))
        .unwrap_or_else(|_| PanicDisableOutcome::Failed("panic during driver panic-disable".into()));
        if outcome == PanicDisableOutcome::Confirmed {
            self.disable_accepted();
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
        fn unconfirmed_disable_joints(&self) -> Vec<usize> {
            vec![2]
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
        let leaked_guard = inner_held.driver.as_ref().unwrap().lock().unwrap();
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
}
