//! Preallocated, lock-free, zero-allocation SAFE-HOLD loop shared by BOTH the RT executor
//! (`armd-control`) and the legacy control thread (`armd-control`).
//!
//! Governing DRV-F1 safety contract:
//! - Never disable torque on teardown away from rest.
//! - `DisableProtective` is permitted ONLY for `Thermal` and `ProcessExit`.
//! - Bus loss alone latches a non-recoverable fault and marks holding unconfirmed; it NEVER
//!   issues `DisableProtective`.
//! - Runs from preallocated state only: zero planner calls, zero new commands, zero heap
//!   allocations, zero frees, and zero mutex acquisitions on `armd-control`.

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::driver::{
    ArmDriver, DisableKind, DisableProtectiveReason, JointTarget, PosForceIntentStamp,
    TorqueProgress, STOPPED_DISABLE_FEEDBACK_MAX_AGE,
};
use crate::model::{ArmConfig, DamiaoGripperMode, JointState};
use crate::rt_core::mailbox::MAX_DOF;
use crate::rt_core::ops::RX_FRAME_BUDGET;
use crate::rt_core::urgent::{EstopState, StopRequests};
use crate::supervisor::{
    bounded_panic_disable_driver_until, panic_disable_deadlines,
    translate_driver_cutoff_conservative, PanicDisableOutcome, SupervisorInner, BUS_LOSS_TIMEOUT,
    PANIC_DISABLE_TIMEOUT, THERMAL_FEEDBACK_FRESHNESS_LIMIT,
};

/// Why the control thread entered SAFE-HOLD.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafeHoldReason {
    None = 0,
    ControlPanic = 1,
    TeardownWhileEnergized = 2,
    StartupTimeoutLoser = 3,
    HandbackStalled = 4,
    SessionUncertainState = 5,
}

impl SafeHoldReason {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::ControlPanic,
            2 => Self::TeardownWhileEnergized,
            3 => Self::StartupTimeoutLoser,
            4 => Self::HandbackStalled,
            5 => Self::SessionUncertainState,
            _ => Self::None,
        }
    }
}

/// Shared lock-free control and observability block for a SAFE-HOLD control thread.
///
/// `armd-control` ONLY touches the atomic fields here (zero mutexes, zero heap allocations).
/// Callers off-control read status atomics and may request an explicit `ProcessExit` protective
/// disable through [`Self::request_process_exit_disable`].
pub struct SafeHoldControl {
    epoch: Instant,
    active: AtomicBool,
    reason: AtomicU8,
    ticks: AtomicU64,
    hold_tx_count: AtomicU64,
    j7_tx_count: AtomicU64,
    thermal_tripped: AtomicBool,
    thermal_disable_confirmed: AtomicBool,
    bus_loss_latched: AtomicBool,
    holding_unconfirmed: AtomicBool,
    process_exit_requested: AtomicBool,
    process_exit_deadline_ns: AtomicU64,
    exited: AtomicBool,
    /// Off-control outcome storage for explicit `ProcessExit` protective disable (written ONLY
    /// once on SAFE-HOLD exit after process exit was requested, or read by callers).
    process_exit_outcome_code: AtomicU8,
    process_exit_outcome: Mutex<Option<PanicDisableOutcome>>,
}

const OUTCOME_NONE: u8 = 0;
const OUTCOME_CONFIRMED: u8 = 1;
const OUTCOME_UNCONFIRMED: u8 = 2;
const OUTCOME_TIMED_OUT: u8 = 3;
const OUTCOME_NO_DRIVER: u8 = 4;
const OUTCOME_FAILED: u8 = 5;

impl Default for SafeHoldControl {
    fn default() -> Self {
        Self::new()
    }
}

impl SafeHoldControl {
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
            active: AtomicBool::new(false),
            reason: AtomicU8::new(SafeHoldReason::None as u8),
            ticks: AtomicU64::new(0),
            hold_tx_count: AtomicU64::new(0),
            j7_tx_count: AtomicU64::new(0),
            thermal_tripped: AtomicBool::new(false),
            thermal_disable_confirmed: AtomicBool::new(false),
            bus_loss_latched: AtomicBool::new(false),
            holding_unconfirmed: AtomicBool::new(false),
            process_exit_requested: AtomicBool::new(false),
            process_exit_deadline_ns: AtomicU64::new(0),
            exited: AtomicBool::new(false),
            process_exit_outcome_code: AtomicU8::new(OUTCOME_NONE),
            process_exit_outcome: Mutex::new(None),
        }
    }

    #[inline]
    pub fn mark_active(&self, reason: SafeHoldReason) {
        let _ = self.reason.compare_exchange(
            SafeHoldReason::None as u8,
            reason as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
        self.active.store(true, Ordering::Release);
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Acquire)
    }

    #[inline]
    pub fn reason(&self) -> SafeHoldReason {
        SafeHoldReason::from_u8(self.reason.load(Ordering::Acquire))
    }

    #[inline]
    pub fn ticks(&self) -> u64 {
        self.ticks.load(Ordering::Acquire)
    }

    #[inline]
    pub fn hold_tx_count(&self) -> u64 {
        self.hold_tx_count.load(Ordering::Acquire)
    }

    #[inline]
    pub fn j7_tx_count(&self) -> u64 {
        self.j7_tx_count.load(Ordering::Acquire)
    }

    #[inline]
    pub fn thermal_tripped(&self) -> bool {
        self.thermal_tripped.load(Ordering::Acquire)
    }

    #[inline]
    pub fn thermal_disable_confirmed(&self) -> bool {
        self.thermal_disable_confirmed.load(Ordering::Acquire)
    }

    #[inline]
    pub fn bus_loss_latched(&self) -> bool {
        self.bus_loss_latched.load(Ordering::Acquire)
    }

    #[inline]
    pub fn holding_unconfirmed(&self) -> bool {
        self.holding_unconfirmed.load(Ordering::Acquire)
    }

    #[inline]
    pub fn is_exited(&self) -> bool {
        self.exited.load(Ordering::Acquire)
    }

    #[inline]
    pub fn process_exit_requested(&self) -> bool {
        self.process_exit_requested.load(Ordering::Acquire)
    }

    /// Requests deliberate `DisableProtective { ProcessExit }` on the SAFE-HOLD writer with one
    /// absolute `deadline`. Lock-free for `armd-control`; callers wait via [`Self::wait_process_exit_until`].
    pub fn request_process_exit_disable(&self, deadline: Instant) {
        let rel_ns = deadline
            .saturating_duration_since(self.epoch)
            .as_nanos()
            .min(u128::from(u64::MAX)) as u64;
        // First writer wins the earliest absolute deadline if multiple process-exit calls race.
        let _ = self.process_exit_deadline_ns.fetch_update(
            Ordering::AcqRel,
            Ordering::Acquire,
            |cur| Some(if cur == 0 { rel_ns.max(1) } else { cur.min(rel_ns.max(1)) }),
        );
        self.process_exit_requested.store(true, Ordering::Release);
    }

    #[inline]
    pub fn process_exit_deadline(&self) -> Instant {
        let rel_ns = self.process_exit_deadline_ns.load(Ordering::Acquire);
        if rel_ns == 0 {
            Instant::now() + PANIC_DISABLE_TIMEOUT
        } else {
            self.epoch + Duration::from_nanos(rel_ns)
        }
    }

    /// Records the process-exit outcome lock-freely via atomic code (zero allocations/locks on
    /// `armd-control`).
    #[inline]
    pub fn record_process_exit_outcome_lockfree(&self, outcome: &PanicDisableOutcome) {
        let code = match outcome {
            PanicDisableOutcome::Confirmed => OUTCOME_CONFIRMED,
            PanicDisableOutcome::Unconfirmed => OUTCOME_UNCONFIRMED,
            PanicDisableOutcome::TimedOut => OUTCOME_TIMED_OUT,
            PanicDisableOutcome::NoDriver => OUTCOME_NO_DRIVER,
            PanicDisableOutcome::Failed(_) => OUTCOME_FAILED,
        };
        let _ = self.process_exit_outcome_code.compare_exchange(
            OUTCOME_NONE,
            code,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
    }

    /// Off-control caller waits up to `deadline` for SAFE-HOLD to finish `ProcessExit` disable.
    pub fn wait_process_exit_until(&self, deadline: Instant) -> PanicDisableOutcome {
        while !self.exited.load(Ordering::Acquire) {
            let rem = deadline.saturating_duration_since(Instant::now());
            if rem.is_zero() {
                let _ = self.process_exit_outcome_code.compare_exchange(
                    OUTCOME_NONE,
                    OUTCOME_TIMED_OUT,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                );
                return PanicDisableOutcome::TimedOut;
            }
            std::thread::sleep(rem.min(Duration::from_millis(1)));
        }
        if let Some(out) = self
            .process_exit_outcome
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
        {
            return out;
        }
        match self.process_exit_outcome_code.load(Ordering::Acquire) {
            OUTCOME_CONFIRMED => PanicDisableOutcome::Confirmed,
            OUTCOME_NO_DRIVER => PanicDisableOutcome::NoDriver,
            OUTCOME_UNCONFIRMED => PanicDisableOutcome::Unconfirmed,
            OUTCOME_FAILED => {
                PanicDisableOutcome::Failed("safe-hold process-exit disable failed".into())
            }
            _ => PanicDisableOutcome::TimedOut,
        }
    }

    #[inline]
    pub fn mark_exited(&self) {
        self.exited.store(true, Ordering::Release);
    }
}

/// Preallocated, stack/fixed-capacity working state for SAFE-HOLD ticks.
///
/// Constructed ONCE before the control loop starts. Every method invoked inside SAFE-HOLD
/// performs zero heap allocations, zero frees, and zero mutex locks.
pub struct SafeHoldBuffers {
    pub dof: usize,
    pub period: Duration,
    pub tx_publish_margin: Duration,
    pub max_temperature: f32,
    pub gripper_is_pos_force: bool,
    pub gripper_index: Option<usize>,
    pub last_fresh_positions: [Option<f32>; MAX_DOF],
    hold_targets: [JointTarget; MAX_DOF],
    rx_states: Vec<JointState>,
    ages_buf: Vec<Option<Duration>>,
    pos_buf: [f32; 1],
    thermal_disable_started: bool,
    j7_stamp: PosForceIntentStamp,
    estop_state: Option<std::sync::Arc<EstopState>>,
    estop_applied: Option<std::sync::Arc<AtomicU64>>,
    stop_requests: Option<StopRequests>,
}

impl SafeHoldBuffers {
    pub fn from_config(config: &ArmConfig, period: Duration) -> Self {
        let dof = config.dof.min(MAX_DOF);
        let tx_publish_margin = crate::driver::DAMIAO_TX_RELEASE_PUBLISH_MARGIN.min(period / 2);
        let gripper_is_pos_force = config
            .damiao_gripper
            .as_ref()
            .is_some_and(|g| g.mode == DamiaoGripperMode::PosForce);
        let gripper_index = config.gripper_index();
        let mut rx_states = Vec::with_capacity(MAX_DOF);
        rx_states.resize(dof, JointState::default());
        let mut ages_buf = Vec::with_capacity(MAX_DOF);
        ages_buf.resize(dof, None);
        Self {
            dof,
            period,
            tx_publish_margin,
            max_temperature: config.max_temperature,
            gripper_is_pos_force,
            gripper_index,
            last_fresh_positions: [None; MAX_DOF],
            hold_targets: std::array::from_fn(|_| JointTarget::position(0.0)),
            rx_states,
            ages_buf,
            pos_buf: [0.0; 1],
            thermal_disable_started: false,
            j7_stamp: PosForceIntentStamp {
                stop_gen: 0,
                estop_epoch: 0,
                lease_gen: 0,
            },
            estop_state: None,
            estop_applied: None,
            stop_requests: None,
        }
    }

    /// Attaches the owner's lock-free E-stop/stop mirrors once at control-thread startup (off RT)
    /// so SAFE-HOLD acknowledges any pending external E-stop/stop generations without allocating
    /// or locking — preventing `EstopGate::pending()` from starving `write_measured_hold_masked`
    /// while keeping `EstopState::LATCH_BIT` permanently latched.
    pub fn attach_stop_mirrors(
        &mut self,
        estop_state: std::sync::Arc<EstopState>,
        estop_applied: std::sync::Arc<AtomicU64>,
        stop_requests: Option<StopRequests>,
    ) {
        self.estop_state = Some(estop_state);
        self.estop_applied = Some(estop_applied);
        self.stop_requests = stop_requests;
    }

    #[inline]
    fn acknowledge_stops_for_safe_hold(&self) {
        if let (Some(es), Some(applied)) = (self.estop_state.as_ref(), self.estop_applied.as_ref())
        {
            es.latch();
            applied.fetch_max(es.requested_count(), Ordering::SeqCst);
        }
        if let Some(stops) = self.stop_requests.as_ref() {
            stops.mark_applied(stops.current());
        }
    }

    /// Mirrors the latest fresh measured hold positions from a healthy `SupervisorInner` tick
    /// (zero allocations, zero locks). Called after every clean tick so SAFE-HOLD always starts
    /// from the last verified measured pose even if a later tick panics mid-mutation.
    #[inline]
    pub fn mirror_from_core(&mut self, core: &SupervisorInner) {
        let dof = core.config.dof.min(MAX_DOF);
        self.dof = dof;
        self.max_temperature = core.config.max_temperature;
        self.gripper_is_pos_force = core
            .config
            .damiao_gripper
            .as_ref()
            .is_some_and(|g| g.mode == DamiaoGripperMode::PosForce);
        self.gripper_index = core.config.gripper_index();
        self.j7_stamp = core.current_j7_intent_stamp();
        for j in 0..dof {
            if let Some((pos, _)) = core.owner_driver_snapshot.fresh_samples[j] {
                if pos.is_finite() {
                    self.last_fresh_positions[j] = Some(pos);
                    continue;
                }
            }
            if let Some(pos) = core.measured_hold.captured_positions[j] {
                if pos.is_finite() {
                    self.last_fresh_positions[j] = Some(pos);
                    continue;
                }
            }
            if core.feedback_numeric_valid {
                if let Some(&pos) = core.measured_positions.get(j) {
                    if pos.is_finite() {
                        self.last_fresh_positions[j] = Some(pos);
                    }
                }
            }
        }
    }

    /// Best-effort zero-allocation capture on entry to SAFE-HOLD (preserves any already-mirrored
    /// fresh positions if `core` was partially mutated before unwinding).
    #[inline]
    pub fn seed_on_entry(&mut self, core: &SupervisorInner) {
        let dof = core.config.dof.min(MAX_DOF);
        self.dof = dof;
        self.max_temperature = core.config.max_temperature;
        self.j7_stamp = core.current_j7_intent_stamp();
        for j in 0..dof {
            if let Some((pos, _)) = core.owner_driver_snapshot.fresh_samples[j] {
                if pos.is_finite() {
                    self.last_fresh_positions[j] = Some(pos);
                }
            } else if self.last_fresh_positions[j].is_none() {
                if let Some(pos) = core.measured_hold.captured_positions[j] {
                    if pos.is_finite() {
                        self.last_fresh_positions[j] = Some(pos);
                    }
                }
            }
        }
    }

    /// Prepares the exclusively owned driver on SAFE-HOLD entry:
    /// - Aborts any interrupted torque-enable handshake without sending rollback disable frames
    ///   so Damiao hold output is never rejected by a pending handshake (M4).
    /// - Switches the driver to standalone PosForce J7 emitter mode (`set_external_pos_force_emitter(false)`)
    ///   so `write_measured_hold_masked` routes J7 directly through the driver's single
    ///   `pos_force_emitter::emit_tick` (`PosForceIntent::stop`: no output until fresh, freeze on
    ///   stale, release-capped).
    #[inline]
    pub fn prepare_driver_on_entry(&self, drv: &mut dyn ArmDriver) {
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drv.prepare_safe_hold();
        }));
    }

    /// Executes ONE zero-allocation, zero-lock SAFE-HOLD tick on an exclusively owned driver.
    pub fn step_driver(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        release_deadline: Instant,
        now_fn: &mut dyn FnMut() -> Instant,
    ) {
        self.step_driver_with_shadow(drv, ctrl, release_deadline, now_fn, None);
    }

    /// Executes ONE zero-allocation, zero-lock SAFE-HOLD tick on an exclusively owned driver,
    /// emitting causal shadow events (`Release`, driver observations, `Thermal`, `BusLoss`,
    /// `LiveHoldCheckpoint`) and flushing at the outer post-write boundary when `shadow` is active.
    pub fn step_driver_with_shadow(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        release_deadline: Instant,
        now_fn: &mut dyn FnMut() -> Instant,
        mut shadow: Option<&mut crate::torque::shadow::ShadowRuntime>,
    ) {
        let dof = self.dof.min(MAX_DOF);
        // 1. Conservative driver-clock-first release cap; reject expired cutoffs before staging.
        let Some(owner_cap) = release_deadline.checked_sub(self.tx_publish_margin) else {
            ctrl.ticks.fetch_add(1, Ordering::Release);
            return;
        };
        let Ok((drv_now, _, drv_cap)) =
            translate_driver_cutoff_conservative(drv, owner_cap, now_fn)
        else {
            ctrl.ticks.fetch_add(1, Ordering::Release);
            return;
        };
        let release_seq = ctrl.ticks.load(Ordering::Acquire).saturating_add(1);
        drv.begin_control_release(0x8000_0000_0000_0000u64 | release_seq);
        drv.set_tx_release_deadline(Some(drv_cap));
        let thermal_confirmed = ctrl.thermal_disable_confirmed.load(Ordering::Acquire);
        if let Some(ref mut sh) = shadow {
            sh.enqueue(crate::torque::shadow::ShadowInputKind::Release {
                release_seq,
                live_intent: crate::torque::TorqueIntent::On,
                live_is_torqued: !thermal_confirmed,
                live_stopped: true,
            });
        }

        // 2. Zero-alloc live RX intake across all supported drivers (Damiao, RobStride, Feetech).
        let rx_deadline = drv_cap.min(drv_now + crate::rt_core::ops::RX_TIME_BUDGET);
        let rx_ok = drv
            .read_joint_states_safe_hold(&mut self.rx_states, rx_deadline, RX_FRAME_BUDGET)
            .is_ok()
            || drv.read_cached_joint_states_into(&mut self.rx_states).is_some();
        if let Some(ref mut sh) = shadow {
            sh.drain_from_driver(drv);
        }

        // 3. Zero-alloc feedback ages and fresh measured position capture.
        drv.feedback_ages_into(&mut self.ages_buf);
        let has_fresh_api = drv.has_driver_fresh_position_api();
        for j in 0..dof {
            let age_fresh = self
                .ages_buf
                .get(j)
                .copied()
                .flatten()
                .is_some_and(|age| age <= STOPPED_DISABLE_FEEDBACK_MAX_AGE);
            if age_fresh && rx_ok {
                if let Some(st) = self.rx_states.get(j) {
                    if st.position.is_finite() {
                        self.last_fresh_positions[j] = Some(st.position);
                        continue;
                    }
                }
            }
            if has_fresh_api
                && drv.fresh_joint_positions_into(
                    &[j],
                    STOPPED_DISABLE_FEEDBACK_MAX_AGE,
                    &mut self.pos_buf,
                )
                && self.pos_buf[0].is_finite()
            {
                self.last_fresh_positions[j] = Some(self.pos_buf[0]);
            }
        }

        // 4. Thermal check BEFORE hold TX: over-temperature is an approved protective disable.
        let temp_fb = drv.capabilities().temperature_feedback;
        if temp_fb && rx_ok && !ctrl.thermal_tripped.load(Ordering::Acquire) {
            let mut hot: Option<(usize, f32)> = None;
            for j in 0..dof {
                let fresh = self
                    .ages_buf
                    .get(j)
                    .copied()
                    .flatten()
                    .is_some_and(|age| age <= THERMAL_FEEDBACK_FRESHNESS_LIMIT);
                if fresh {
                    if let Some(st) = self.rx_states.get(j) {
                        if st.temperature.is_finite() && st.temperature > self.max_temperature {
                            hot = Some((j, st.temperature));
                            break;
                        }
                    }
                }
            }
            if let Some((hot_j, hot_temp)) = hot {
                ctrl.thermal_tripped.store(true, Ordering::Release);
                if let Some(ref mut sh) = shadow {
                    sh.enqueue(crate::torque::shadow::ShadowInputKind::Thermal {
                        channel: hot_j as u8,
                        temperature_c: hot_temp,
                        live_intent: crate::torque::TorqueIntent::On,
                        live_is_torqued: true,
                        live_stopped: true,
                    });
                }
            }
        }

        if ctrl.thermal_tripped.load(Ordering::Acquire) {
            if !ctrl.thermal_disable_confirmed.load(Ordering::Acquire) {
                let kind = DisableKind::DisableProtective {
                    reason: DisableProtectiveReason::Thermal,
                };
                if !self.thermal_disable_started {
                    match drv.begin_disable_torque_budgeted(kind, 0) {
                        Ok(TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed))
                            if drv.is_hardware_disarmed_confirmed() =>
                        {
                            self.thermal_disable_started = true;
                            ctrl.thermal_disable_confirmed.store(true, Ordering::Release);
                        }
                        Ok(TorqueProgress::Done(_)) => {
                            // Never falsely mark unconfirmed thermal disable as confirmed; retry.
                            self.thermal_disable_started = false;
                        }
                        Ok(TorqueProgress::Pending { .. }) => {
                            self.thermal_disable_started = true;
                        }
                        Err(_) => {
                            self.thermal_disable_started = false;
                        }
                    }
                }
                if self.thermal_disable_started
                    && !ctrl.thermal_disable_confirmed.load(Ordering::Acquire)
                {
                    match drv.poll_torque_budgeted(drv_cap, RX_FRAME_BUDGET) {
                        Ok(TorqueProgress::Done(crate::driver::TorqueOutcome::Confirmed))
                            if drv.is_hardware_disarmed_confirmed() =>
                        {
                            ctrl.thermal_disable_confirmed.store(true, Ordering::Release);
                        }
                        Ok(TorqueProgress::Done(_)) => {
                            self.thermal_disable_started = false;
                        }
                        Ok(TorqueProgress::Pending { .. }) => {}
                        Err(_) => {
                            if !drv.torque_transition_pending() {
                                self.thermal_disable_started = false;
                            }
                        }
                    }
                }
                if let Some(ref mut sh) = shadow {
                    sh.drain_from_driver(drv);
                }
            }
            drv.set_tx_release_deadline(None);
            if let Some(ref mut sh) = shadow {
                sh.flush_deferred_steps();
            }
            ctrl.ticks.fetch_add(1, Ordering::Release);
            return;
        }

        // 5. Freshness / bus-loss check BEFORE hold TX:
        // DRV-F1 hard rule: Bus loss alone latches fault + marks holding unconfirmed; NEVER
        // issues `DisableProtective`.
        let numeric_ok = !rx_ok
            || self
                .rx_states
                .iter()
                .take(dof)
                .all(|st| st.position.is_finite());
        let all_bus_fresh =
            numeric_ok && drv.is_feedback_valid() && drv.feedback_all_within(dof, BUS_LOSS_TIMEOUT);
        if !all_bus_fresh {
            ctrl.holding_unconfirmed.store(true, Ordering::Release);
            let was_latched = ctrl.bus_loss_latched.swap(true, Ordering::AcqRel);
            if !was_latched {
                if let Some(ref mut sh) = shadow {
                    sh.enqueue(crate::torque::shadow::ShadowInputKind::BusLoss {
                        device: 0,
                        multi_device_gap: false,
                        live_intent: crate::torque::TorqueIntent::On,
                        live_is_torqued: true,
                        live_stopped: true,
                    });
                }
            }
        } else {
            ctrl.holding_unconfirmed.store(false, Ordering::Release);
        }

        // 6. Masked measured-position hold TX (zero planner calls; only joints with captured
        // fresh measured positions; never invents targets). When `gripper_is_pos_force`, the
        // driver's standalone single emitter (`set_external_pos_force_emitter(false)`) runs
        // inside `write_measured_hold_masked` under the same release cap (no output until fresh).
        let mut hold_mask = 0u64;
        for j in 0..dof {
            if let Some(pos) = self.last_fresh_positions[j] {
                if pos.is_finite() {
                    self.hold_targets[j] = JointTarget::position(pos);
                    hold_mask |= 1u64 << j;
                }
            }
        }
        let mut sent_hold_mask = 0u64;
        let mut unverified_mask = 0u64;
        if hold_mask != 0 || self.gripper_is_pos_force {
            // Acknowledge any pending external E-stop/stop generations while keeping LATCH_BIT set
            // so `EstopGate::pending()` never suppresses SAFE-HOLD measured-position hold frames.
            self.acknowledge_stops_for_safe_hold();
            drv.set_pending_pos_force_hold_hint(true);
            if let Ok(report) = drv.write_measured_hold_masked(hold_mask, &self.hold_targets[..dof])
            {
                if report.sent_any {
                    ctrl.hold_tx_count.fetch_add(1, Ordering::Release);
                }
                for j in 0..dof.min(report.dof) {
                    match report.joint_status[j] {
                        crate::driver::JointHoldStatus::Held
                        | crate::driver::JointHoldStatus::HeldStaleTarget => {
                            sent_hold_mask |= 1u64 << j;
                        }
                        crate::driver::JointHoldStatus::UnheldNoVerifiedMode if j < 64 => {
                            unverified_mask |= 1u64 << j;
                        }
                        _ => {}
                    }
                }
                if sent_hold_mask == 0 && report.sent_any {
                    sent_hold_mask = hold_mask;
                }
                if let Some(j7) = self.gripper_index {
                    if self.gripper_is_pos_force
                        && j7 < report.dof
                        && matches!(
                            report.joint_status[j7],
                            crate::driver::JointHoldStatus::Held
                                | crate::driver::JointHoldStatus::HeldStaleTarget
                        )
                    {
                        ctrl.j7_tx_count.fetch_add(1, Ordering::Release);
                        sent_hold_mask |= 1u64 << j7;
                    }
                }
            }
            if unverified_mask != 0 {
                // Causally probe CTRL_MODE for any mode-uncertain joint (e.g. lost REG_WRITE ACK)
                // so SAFE-HOLD can resume holding it in its verified mode without ever sending a
                // guessed-mode frame or letting the 100 ms motor watchdog expire.
                let _ = drv.request_mode_readback_budgeted(unverified_mask);
            }
            if let Some(ref mut sh) = shadow {
                sh.drain_from_driver(drv);
            }
        }
        if let Some(ref mut sh) = shadow {
            sh.enqueue(crate::torque::shadow::ShadowInputKind::LiveHoldCheckpoint {
                hold_mask: crate::torque::ChannelMask(sent_hold_mask as u32),
                contended: false,
                escalated: false,
                live_intent: crate::torque::TorqueIntent::On,
                live_is_torqued: true,
                live_stopped: true,
            });
            sh.flush_deferred_steps();
        }

        drv.set_tx_release_deadline(None);
        ctrl.ticks.fetch_add(1, Ordering::Release);
    }

    /// Runs the periodic SAFE-HOLD loop on the exclusively owned driver until an explicit
    /// `ProcessExit` protective disable is requested via [`SafeHoldControl::request_process_exit_disable`],
    /// returning the requested `hard_deadline`.
    ///
    /// Performs zero heap allocations, zero frees, and zero mutex locks per tick.
    pub fn run_loop_until_process_exit<FNow, FSleep>(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        reason: SafeHoldReason,
        now_fn: FNow,
        sleep_dur: FSleep,
    ) -> Instant
    where
        FNow: FnMut() -> Instant,
        FSleep: FnMut(Duration),
    {
        self.run_loop_until_process_exit_with_shadow(drv, ctrl, reason, now_fn, sleep_dur, None)
    }

    /// Shadow-aware variant of [`Self::run_loop_until_process_exit`]: emits causal SAFE-HOLD
    /// shadow events and flushes at every tick's outer post-write boundary with zero allocs/locks.
    pub fn run_loop_until_process_exit_with_shadow<FNow, FSleep>(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        reason: SafeHoldReason,
        mut now_fn: FNow,
        mut sleep_dur: FSleep,
        mut shadow: Option<&mut crate::torque::shadow::ShadowRuntime>,
    ) -> Instant
    where
        FNow: FnMut() -> Instant,
        FSleep: FnMut(Duration),
    {
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.prepare_driver_on_entry(drv);
        }));
        ctrl.mark_active(reason);
        let period = self.period.max(Duration::from_millis(1));
        loop {
            if ctrl.process_exit_requested.load(Ordering::Acquire) {
                return ctrl.process_exit_deadline();
            }
            let tick_start = std::panic::catch_unwind(std::panic::AssertUnwindSafe(&mut now_fn))
                .unwrap_or_else(|_| Instant::now());
            let release_deadline = tick_start + period;
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                self.step_driver_with_shadow(
                    drv,
                    ctrl,
                    release_deadline,
                    &mut now_fn,
                    shadow.as_deref_mut(),
                );
            }));
            if ctrl.process_exit_requested.load(Ordering::Acquire) {
                continue;
            }
            let tick_end = std::panic::catch_unwind(std::panic::AssertUnwindSafe(&mut now_fn))
                .unwrap_or_else(|_| Instant::now());
            let elapsed = tick_end.saturating_duration_since(tick_start);
            if let Some(rem) = period.checked_sub(elapsed) {
                if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sleep_dur(rem)))
                    .is_err()
                {
                    std::thread::sleep(rem);
                }
            }
        }
    }

    /// Runs the periodic SAFE-HOLD loop on the exclusively owned driver until an explicit
    /// `ProcessExit` protective disable is requested via [`SafeHoldControl::request_process_exit_disable`],
    /// then executes bounded `DisableProtective { ProcessExit }` and marks SAFE-HOLD exited.
    ///
    /// Performs zero heap allocations, zero frees, and zero mutex locks per SAFE-HOLD tick.
    pub fn run_loop<FNow, FSleep>(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        reason: SafeHoldReason,
        now_fn: FNow,
        sleep_dur: FSleep,
    ) -> PanicDisableOutcome
    where
        FNow: FnMut() -> Instant,
        FSleep: FnMut(Duration),
    {
        self.run_loop_with_shadow(drv, ctrl, reason, now_fn, sleep_dur, None)
    }

    /// Runs the periodic SAFE-HOLD loop indefinitely after a failed/unconfirmed/timed-out
    /// `ProcessExit` disable so an energized or unconfirmed driver is NEVER dropped (DRV-F1 B5).
    pub fn run_post_exit_hold_forever<FNow, FSleep>(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        mut now_fn: FNow,
        mut sleep_dur: FSleep,
        mut shadow: Option<&mut crate::torque::shadow::ShadowRuntime>,
    ) -> !
    where
        FNow: FnMut() -> Instant,
        FSleep: FnMut(Duration),
    {
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.prepare_driver_on_entry(drv);
        }));
        let period = self.period.max(Duration::from_millis(1));
        loop {
            let tick_start = std::panic::catch_unwind(std::panic::AssertUnwindSafe(&mut now_fn))
                .unwrap_or_else(|_| Instant::now());
            let release_deadline = tick_start + period;
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                self.step_driver_with_shadow(
                    drv,
                    ctrl,
                    release_deadline,
                    &mut now_fn,
                    shadow.as_deref_mut(),
                );
            }));
            let tick_end = std::panic::catch_unwind(std::panic::AssertUnwindSafe(&mut now_fn))
                .unwrap_or_else(|_| Instant::now());
            let elapsed = tick_end.saturating_duration_since(tick_start);
            if let Some(rem) = period.checked_sub(elapsed) {
                if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sleep_dur(rem)))
                    .is_err()
                {
                    std::thread::sleep(rem);
                }
            }
        }
    }

    /// Shadow-aware variant of [`Self::run_loop`] (used by Legacy SAFE-HOLD extraction).
    pub fn run_loop_with_shadow<FNow, FSleep>(
        &mut self,
        drv: &mut dyn ArmDriver,
        ctrl: &SafeHoldControl,
        reason: SafeHoldReason,
        mut now_fn: FNow,
        mut sleep_dur: FSleep,
        mut shadow: Option<&mut crate::torque::shadow::ShadowRuntime>,
    ) -> PanicDisableOutcome
    where
        FNow: FnMut() -> Instant,
        FSleep: FnMut(Duration),
    {
        let hard_deadline = self.run_loop_until_process_exit_with_shadow(
            drv,
            ctrl,
            reason,
            &mut now_fn,
            &mut sleep_dur,
            shadow.as_deref_mut(),
        );
        let origin = Instant::now();
        let remaining = hard_deadline.saturating_duration_since(origin);
        let (coop_deadline, _) = panic_disable_deadlines(origin, remaining);
        let coop_budget = coop_deadline.saturating_duration_since(origin);
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            bounded_panic_disable_driver_until(drv, coop_deadline, hard_deadline, coop_budget)
        }))
        .unwrap_or_else(|_| {
            PanicDisableOutcome::Failed("panic during safe-hold process-exit disable".into())
        });
        if let Some(ref mut sh) = shadow {
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                sh.enqueue(crate::torque::shadow::ShadowInputKind::ProcessExit {
                    live_intent: crate::torque::TorqueIntent::On,
                    live_is_torqued: !ctrl.thermal_disable_confirmed.load(Ordering::Acquire),
                    live_stopped: true,
                });
                match &outcome {
                    PanicDisableOutcome::Confirmed
                    | PanicDisableOutcome::Unconfirmed
                    | PanicDisableOutcome::NoDriver => {
                        sh.drain_from_driver(drv);
                    }
                    PanicDisableOutcome::TimedOut | PanicDisableOutcome::Failed(_) => {
                        sh.record_external_staging_loss(1);
                    }
                }
                sh.flush_deferred_steps();
            }));
        }
        // Publish outcome and wake caller FIRST so failed/unconfirmed/timed-out ProcessExit never
        // hangs caller; retain sole driver ownership in SAFE-HOLD on any non-confirmed outcome (B5).
        ctrl.record_process_exit_outcome_lockfree(&outcome);
        ctrl.mark_exited();
        let disarmed = matches!(
            outcome,
            PanicDisableOutcome::Confirmed | PanicDisableOutcome::NoDriver
        ) || std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            drv.is_hardware_disarmed_confirmed()
        }))
        .unwrap_or(false);
        if !disarmed {
            self.run_post_exit_hold_forever(drv, ctrl, now_fn, sleep_dur, shadow);
        }
        outcome
    }
}
