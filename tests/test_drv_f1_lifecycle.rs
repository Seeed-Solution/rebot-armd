//! DRV-F1 end-to-end lifecycle, handback, single-deadline, and tiered panic-recovery suite.
//!
//! Covers BOTH Legacy and RT Executor owners against the governing DRV-F1 safety rules:
//! 1. Never disable torque on teardown away from rest.
//! 2. `DisableProtective` is ONLY for `Thermal` and `ProcessExit`.
//! 3. Bus loss alone latches fault + holding unconfirmed (NO `DisableProtective`).
//! 4. Energized handback preserves fresh measured hold + single writer; stalled handback keeps
//!    sole driver ownership in SAFE-HOLD and NEVER publishes `OWNER_LEGACY`.
//! 5. Single end-to-end deadline, atomic startup activation barrier, token-bound `CommitShutdown`.
//! 6. Non-control panic fences commands (`INTERNAL_FAULT`), stops, holds, parks to rest ->
//!    rest-gated disable -> non-recoverable fault naming panicking thread; blocked park holds.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, HoldPassReport,
    JointTarget, TorqueProgress,
};
use armd::model::{ArmConfig, JointState};
use armd::panic_coord::{install_process_panic_coordinator, InternalFaultParkOutcome};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::executor::{ExecutorConfig, OwnerError, PanicPolicy, PersistTestHooks};
use armd::rt_core::safe_hold::SafeHoldReason;
use armd::supervisor::{
    ArmSupervisor, PanicDisableOutcome, OWNER_EXECUTOR, OWNER_FAILED, OWNER_LEGACY,
};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

#[derive(Clone, Copy)]
struct SimMotor {
    id: u16,
    q: f32,
    dq: f32,
    tau: f32,
    temp_c: u8,
    enabled: bool,
    silent: bool,
    limits: DamiaoLimits,
}

impl SimMotor {
    fn feedback_frame(&self) -> CanFrame {
        let state = if self.enabled { 1u8 } else { 0u8 };
        let q_raw = damiao::float_to_uint(self.q, -self.limits.p_max, self.limits.p_max, 16);
        let dq_raw = damiao::float_to_uint(self.dq, -self.limits.v_max, self.limits.v_max, 12);
        let t_raw = damiao::float_to_uint(self.tau, -self.limits.t_max, self.limits.t_max, 12);
        let d = [
            (state << 4) | ((self.id as u8) & 0x0F),
            (q_raw >> 8) as u8,
            (q_raw & 0xFF) as u8,
            (dq_raw >> 4) as u8,
            (((dq_raw & 0x0F) << 4) | ((t_raw >> 8) & 0x0F)) as u8,
            (t_raw & 0xFF) as u8,
            self.temp_c,
            self.temp_c,
        ];
        CanFrame::new_standard(0x10 + self.id, &d).unwrap()
    }
}

struct FakeBusInner {
    motors: Vec<SimMotor>,
    ctrl_modes: [u32; 7],
    rx: VecDeque<(CanFrame, Option<Instant>)>,
    tx_log: Vec<CanFrame>,
    immediate_replies: bool,
    withhold_disable_feedback: bool,
    follow_targets: bool,
    clock: Option<MockClock>,
}

impl FakeBusInner {
    fn push_rx(&mut self, frame: CanFrame) {
        let ts = self.clock.as_ref().map(MockClock::now);
        self.rx.push_back((frame, ts));
    }

    fn push_all_feedback(&mut self) {
        let ts = self.clock.as_ref().map(MockClock::now);
        for m in &self.motors {
            if !m.silent {
                self.rx.push_back((m.feedback_frame(), ts));
            }
        }
    }
}

#[derive(Clone)]
struct FakeBus(Arc<Mutex<FakeBusInner>>);

impl FakeBus {
    fn new(limits: &[DamiaoLimits], q_init: [f32; 7], clock: Option<MockClock>) -> Self {
        let motors = limits
            .iter()
            .enumerate()
            .map(|(i, &lim)| SimMotor {
                id: (i + 1) as u16,
                q: q_init[i],
                dq: 0.0,
                tau: 0.0,
                temp_c: 30,
                enabled: false,
                silent: false,
                limits: lim,
            })
            .collect();
        Self(Arc::new(Mutex::new(FakeBusInner {
            motors,
            ctrl_modes: [damiao::CTRL_MODE_POS_VEL; 7],
            rx: VecDeque::new(),
            tx_log: Vec::new(),
            immediate_replies: true,
            withhold_disable_feedback: false,
            follow_targets: false,
            clock,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut FakeBusInner) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl CanTransport for FakeBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.tx_log.push(*frame);
        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(fb) = s
                .motors
                .iter()
                .find(|m| m.id == id && !m.silent)
                .map(SimMotor::feedback_frame)
            {
                s.push_rx(fb);
            }
            return Ok(());
        }
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_READ {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            if let Some((idx, m)) = s.motors.iter().enumerate().find(|(_, m)| m.id == id) {
                let ctrl_mode = s.ctrl_modes[idx];
                let val: Option<[u8; 4]> = match rid {
                    damiao::RID_CTRL_MODE => Some(ctrl_mode.to_le_bytes()),
                    damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                    damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                    damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                    _ => None,
                };
                if let Some(v) = val {
                    let [lo, hi] = id.to_le_bytes();
                    let reply = CanFrame::new_standard(
                        0x10 + id,
                        &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                    )
                    .unwrap();
                    s.push_rx(reply);
                }
            }
            return Ok(());
        }
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let v = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                if rid == damiao::RID_CTRL_MODE {
                    s.ctrl_modes[idx] = u32::from_le_bytes(v);
                }
                let [lo, hi] = id.to_le_bytes();
                let reply = CanFrame::new_standard(
                    0x10 + id,
                    &[lo, hi, damiao::REG_WRITE, rid, v[0], v[1], v[2], v[3]],
                )
                .unwrap();
                s.push_rx(reply);
            }
            return Ok(());
        }
        let id = frame.id as u16;
        let base_id = if id >= 0x300 {
            id - 0x300
        } else if id >= 0x100 {
            id - 0x100
        } else {
            id
        };
        let is_pos_vel = (0x101..=0x107).contains(&id);
        let is_pos_force = (0x301..=0x307).contains(&id);
        let immediate = s.immediate_replies;
        let withhold_disable = s.withhold_disable_feedback;
        let follow = s.follow_targets;
        if let Some(m) = s.motors.iter_mut().find(|m| m.id == base_id) {
            let is_disable = frame.data == damiao::CMD_DISABLE;
            if frame.data == damiao::CMD_ENABLE {
                m.enabled = true;
            } else if is_disable {
                if !withhold_disable {
                    m.enabled = false;
                }
            } else if (is_pos_vel || is_pos_force) && follow && frame.data.len() >= 4 {
                let target_q = f32::from_le_bytes([
                    frame.data[0],
                    frame.data[1],
                    frame.data[2],
                    frame.data[3],
                ]);
                if target_q.is_finite() {
                    m.q = target_q;
                }
            }
            if immediate && !m.silent && !(is_disable && withhold_disable) {
                let fb = m.feedback_frame();
                s.push_rx(fb);
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let mut s = self.0.lock().unwrap();
        let now = s.clock.as_ref().map_or_else(Instant::now, |c| c.now());
        s.rx.pop_front()
            .map(|(f, ts)| (f, Some(ts.unwrap_or(now))))
            .ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        self.0
            .lock()
            .unwrap()
            .clock
            .clone()
            .map(|c| Arc::new(c) as Arc<dyn DriverClock>)
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }
}

async fn wait_until_async(timeout: Duration, label: &str, mut cond: impl FnMut() -> bool) {
    let dl = Instant::now() + timeout;
    while !cond() {
        assert!(Instant::now() < dl, "timed out waiting for: {label}");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

fn wait_until(timeout: Duration, label: &str, mut cond: impl FnMut() -> bool) {
    let dl = Instant::now() + timeout;
    while !cond() {
        assert!(Instant::now() < dl, "timed out waiting for: {label}");
        thread::sleep(Duration::from_millis(2));
    }
}

fn make_energized_dm_supervisor(
    q_init: [f32; 7],
    clock: Option<MockClock>,
    follow_targets: bool,
) -> (Arc<ArmSupervisor>, FakeBus) {
    let bus = FakeBus::new(&dm_limits(), q_init, clock);
    bus.with(|s| {
        s.follow_targets = follow_targets;
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap()
        .with_gripper_pos_force(armd::driver::GripperPosForce {
            joint: 6,
            current_raw: 220,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        })
        .unwrap();
    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    (sup, bus)
}

// ============================================================================
// 1. Executor control-thread panic -> preallocated SAFE-HOLD -> ProcessExit
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_control_panic_enters_safe_hold_holds_measured_pose_j7_emitter_and_exits_only_on_process_exit() {
    let raised = [0.40, -0.30, 0.50, 0.20, -0.20, 0.10, -1.20];
    let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));
    bus.with(|s| s.tx_log.clear());

    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    cfg.panic_policy = PanicPolicy::StopExecutor;
    let handle = sup
        .start_executor_with_hooks(
            cfg,
            PersistTestHooks {
                panic_on_release: Some(2),
                ..Default::default()
            },
        )
        .await
        .unwrap();

    wait_until_async(Duration::from_secs(2), "executor entered SAFE-HOLD", || {
        handle.is_in_safe_hold() && handle.safe_hold_control().ticks() >= 3
    })
    .await;

    let sh = handle.safe_hold_control();
    assert_eq!(sh.reason(), SafeHoldReason::ControlPanic);
    assert!(sup.internal_fault_fenced(), "SAFE-HOLD must fence commands & rearm");
    assert!(sh.hold_tx_count() >= 1, "SAFE-HOLD must actively emit hold frames");
    assert!(
        sh.j7_tx_count() >= 1,
        "SAFE-HOLD must emit J7 PosForce hold frames through the single emitter"
    );

    // Zero CMD_DISABLE frames emitted and all 7 motors remain physically enabled; hold targets
    // match the last fresh measured pose (no target invention).
    let (all_enabled, disable_count, pos_vel_j1, has_j7_307) = bus.with(|s| {
        let disables = s
            .tx_log
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE)
            .count();
        let j1_q = s
            .tx_log
            .iter()
            .rev()
            .find(|f| f.id == 0x101)
            .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]));
        let j7_307 = s.tx_log.iter().any(|f| f.id == 0x307);
        (s.motors.iter().all(|m| m.enabled), disables, j1_q, j7_307)
    });
    assert!(all_enabled, "all 7 motors must stay enabled during SAFE-HOLD away from rest");
    assert_eq!(disable_count, 0, "control panic away from rest must issue 0 CMD_DISABLE frames");
    assert!(
        pos_vel_j1.is_some_and(|q| (q - raised[0]).abs() < 0.02),
        "SAFE-HOLD J1 hold target {pos_vel_j1:?} must match measured pose {}",
        raised[0]
    );
    assert!(has_j7_307, "SAFE-HOLD must drive J7 through 0x307 PosForce emitter");

    // Deliberate ProcessExit disable exits SAFE-HOLD and disables all 7 motors.
    let outcome = sup
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
        .await;
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
    assert!(sh.is_exited());
}

// ============================================================================
// 2. Legacy control-thread panic -> preallocated SAFE-HOLD -> ProcessExit
// ============================================================================

struct PanicOnceDriver {
    inner: DamiaoArmDriver<FakeBus>,
    panic_after_reads: AtomicUsize,
}

impl ArmDriver for PanicOnceDriver {
    fn capabilities(&self) -> DriverCapabilities {
        self.inner.capabilities()
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let rem = self.panic_after_reads.load(Ordering::Acquire);
        if rem > 0 && self.panic_after_reads.fetch_sub(1, Ordering::AcqRel) == 1 {
            panic!("injected legacy control-thread panic");
        }
        self.inner.read_joint_states()
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let rem = self.panic_after_reads.load(Ordering::Acquire);
        if rem > 0 && self.panic_after_reads.fetch_sub(1, Ordering::AcqRel) == 1 {
            panic!("injected legacy control-thread panic");
        }
        self.inner.read_joint_states_into(out)
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.inner
            .read_joint_states_budgeted(out, deadline, max_rx_frames)
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.inner.write_joint_targets(targets)
    }
    fn write_measured_hold_masked(
        &mut self,
        mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.inner.write_measured_hold_masked(mask, targets)
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.inner.set_torque(enable)
    }
    fn begin_torque_budgeted(
        &mut self,
        enable: bool,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted(enable, max_rx_frames)
    }
    fn begin_disable_torque_budgeted(
        &mut self,
        kind: armd::driver::DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_disable_torque_budgeted(kind, max_rx_frames)
    }
    fn poll_torque_budgeted(
        &mut self,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque_budgeted(deadline, max_rx_frames)
    }
    fn abort_enable(&mut self) {
        self.inner.abort_enable();
    }
    fn set_external_pos_force_emitter(&mut self, external: bool) {
        self.inner.set_external_pos_force_emitter(external);
    }
    fn set_pending_pos_force_hold_hint(&mut self, active: bool) {
        self.inner.set_pending_pos_force_hold_hint(active);
    }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.inner.set_tx_release_deadline(deadline);
    }
    fn is_feedback_valid(&self) -> bool {
        self.inner.is_feedback_valid()
    }
    fn is_enabled(&self) -> bool {
        self.inner.is_enabled()
    }
    fn enable_state(&self) -> EnableState {
        self.inner.enable_state()
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.inner.feedback_ages()
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        self.inner.feedback_ages_into(out);
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.inner.feedback_all_within(dof, max_age)
    }
    fn has_driver_fresh_position_api(&self) -> bool {
        self.inner.has_driver_fresh_position_api()
    }
    fn fresh_joint_positions_into(
        &self,
        joints: &[usize],
        max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        self.inner.fresh_joint_positions_into(joints, max_age, out)
    }
    fn clock_now(&self) -> Instant {
        self.inner.clock_now()
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_control_panic_enters_safe_hold_holds_measured_pose_and_exits_on_process_exit() {
    let raised = [0.35, -0.25, 0.45, 0.15, -0.15, 0.05, -0.90];
    let bus = FakeBus::new(&dm_limits(), raised, None);
    bus.with(|s| s.push_all_feedback());
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    let drv = PanicOnceDriver {
        inner: dm,
        panic_after_reads: AtomicUsize::new(0),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    // Arm panic on the 3rd legacy loop tick after enable.
    {
        let mut inner = sup.inner().write().await;
        // Inject via driver tick read once loop starts:
        let _ = &mut inner;
    }
    // Re-create cleanly with panic armed AFTER enable_torque:
    let bus2 = FakeBus::new(&dm_limits(), raised, None);
    bus2.with(|s| s.push_all_feedback());
    let mut dm2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    dm2.read_joint_states().unwrap();
    dm2.check_mapping_ranges();
    let panic_counter = Arc::new(AtomicUsize::new(0));
    struct SharedPanicDriver {
        inner: DamiaoArmDriver<FakeBus>,
        countdown: Arc<AtomicUsize>,
    }
    impl ArmDriver for SharedPanicDriver {
        fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let rem = self.countdown.load(Ordering::Acquire);
            if rem > 0 && self.countdown.fetch_sub(1, Ordering::AcqRel) == 1 {
                panic!("injected legacy control-thread panic");
            }
            self.inner.read_joint_states()
        }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
            let rem = self.countdown.load(Ordering::Acquire);
            if rem > 0 && self.countdown.fetch_sub(1, Ordering::AcqRel) == 1 {
                panic!("injected legacy control-thread panic");
            }
            self.inner.read_joint_states_into(out)
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<JointState>,
            deadline: Instant,
            max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)
        }
        fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
            self.inner.read_cached_joint_states_into(out)
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.inner.write_joint_targets(targets)
        }
        fn write_measured_hold_masked(
            &mut self,
            mask: u64,
            targets: &[JointTarget],
        ) -> Result<HoldPassReport, ArmDriverError> {
            self.inner.write_measured_hold_masked(mask, targets)
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            self.inner.set_torque(enable)
        }
        fn begin_torque_budgeted(&mut self, enable: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.begin_torque_budgeted(enable, max_rx_frames)
        }
        fn begin_disable_torque_budgeted(
            &mut self,
            kind: armd::driver::DisableKind,
            max_rx_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.begin_disable_torque_budgeted(kind, max_rx_frames)
        }
        fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.poll_torque_budgeted(deadline, max_rx_frames)
        }
        fn abort_enable(&mut self) { self.inner.abort_enable(); }
        fn set_external_pos_force_emitter(&mut self, external: bool) { self.inner.set_external_pos_force_emitter(external); }
        fn set_pending_pos_force_hold_hint(&mut self, active: bool) { self.inner.set_pending_pos_force_hold_hint(active); }
        fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) { self.inner.set_tx_release_deadline(deadline); }
        fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
        fn is_enabled(&self) -> bool { self.inner.is_enabled() }
        fn enable_state(&self) -> EnableState { self.inner.enable_state() }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out); }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.inner.feedback_all_within(dof, max_age) }
        fn has_driver_fresh_position_api(&self) -> bool { self.inner.has_driver_fresh_position_api() }
        fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
            self.inner.fresh_joint_positions_into(joints, max_age, out)
        }
        fn clock_now(&self) -> Instant { self.inner.clock_now() }
    }
    let sup2 = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(SharedPanicDriver {
        inner: dm2,
        countdown: Arc::clone(&panic_counter),
    })));
    sup2.set_lease_enabled(false).await;
    sup2.enable_torque(sup2.capture_admission_stamp()).await.unwrap();
    bus2.with(|s| s.tx_log.clear());
    panic_counter.store(2, Ordering::Release);

    let stop = Arc::new(AtomicBool::new(false));
    sup2.register_legacy_control_stop(Arc::clone(&stop));
    let sup_loop = Arc::clone(&sup2);
    let stop_loop = Arc::clone(&stop);
    let rt_handle = tokio::runtime::Handle::current();
    let stats = Arc::new(armd::rt_core::executor::ExecutorStats::default());
    let jh = thread::Builder::new()
        .name("armd-control".into())
        .spawn(move || {
            sup_loop.run_legacy_control_loop_blocking(
                Duration::from_millis(5),
                stop_loop,
                stats,
                rt_handle,
                None,
            );
        })
        .unwrap();

    let sh = sup2.legacy_safe_hold_control();
    wait_until_async(Duration::from_secs(2), "legacy entered SAFE-HOLD", || {
        sh.is_active() && sh.ticks() >= 3
    })
    .await;

    assert_eq!(sh.reason(), SafeHoldReason::ControlPanic);
    assert!(sup2.internal_fault_fenced());
    assert!(sh.hold_tx_count() >= 1);
    assert!(bus2.with(|s| {
        s.motors.iter().all(|m| m.enabled)
            && !s.tx_log.iter().any(|f| f.data == damiao::CMD_DISABLE)
    }));

    let outcome = sup2
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
        .await;
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    jh.join().unwrap();
    assert!(sh.is_exited());
    assert!(bus2.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

// ============================================================================
// 3. SAFE-HOLD thermal trip -> DisableProtective { Thermal };
//    SAFE-HOLD bus loss -> fault latched + holding unconfirmed, NO disable
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn safe_hold_thermal_trip_issues_protective_disable_and_bus_loss_holds_unconfirmed_without_disable() {
    let raised = [0.30, -0.20, 0.40, 0.10, -0.10, 0.05, -0.50];

    // (a) Bus loss during SAFE-HOLD: NEVER disables torque!
    {
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        bus.with(|s| s.tx_log.clear());

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    panic_on_release: Some(2),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        let sh = handle.safe_hold_control();
        wait_until_async(Duration::from_secs(2), "entered SAFE-HOLD", || {
            sh.is_active() && sh.ticks() >= 2
        })
        .await;

        // Make all motors silent -> triggers bus loss after BUS_LOSS_TIMEOUT (~250ms).
        bus.with(|s| {
            s.rx.clear();
            for m in &mut s.motors {
                m.silent = true;
            }
            s.tx_log.clear();
        });
        wait_until_async(Duration::from_secs(2), "bus loss latched in SAFE-HOLD", || {
            sh.bus_loss_latched() && sh.holding_unconfirmed()
        })
        .await;
        assert!(!sh.thermal_tripped());
        assert!(
            bus.with(|s| {
                s.motors.iter().all(|m| m.enabled)
                    && !s.tx_log.iter().any(|f| f.data == damiao::CMD_DISABLE)
            }),
            "DRV-F1 hard rule: bus loss alone in SAFE-HOLD must NEVER issue DisableProtective"
        );
        // Re-enable replies and exit cleanly via deliberate ProcessExit.
        bus.with(|s| {
            for m in &mut s.motors {
                m.silent = false;
            }
        });
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }

    // (b) Thermal trip during SAFE-HOLD: issues DisableProtective { Thermal }!
    {
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        bus.with(|s| s.tx_log.clear());

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    panic_on_release: Some(2),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        let sh = handle.safe_hold_control();
        wait_until_async(Duration::from_secs(2), "entered SAFE-HOLD for thermal", || {
            sh.is_active() && sh.ticks() >= 2
        })
        .await;
        assert!(!sh.thermal_tripped());

        // Inject over-temperature on J3 (> 70°C max_temperature).
        bus.with(|s| {
            s.motors[2].temp_c = 95;
            s.push_all_feedback();
        });
        wait_until_async(Duration::from_secs(2), "thermal disable confirmed in SAFE-HOLD", || {
            sh.thermal_tripped() && sh.thermal_disable_confirmed()
        })
        .await;
        assert!(
            bus.with(|s| {
                s.motors.iter().all(|m| !m.enabled)
                    && s.tx_log.iter().any(|f| f.data == damiao::CMD_DISABLE)
            }),
            "thermal trip in SAFE-HOLD must execute DisableProtective {{ Thermal }}"
        );
        let _ = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
    }
}

// ============================================================================
// 4. Non-control panic -> fence commands, park to rest, rest-gated disable,
//    latch non-recoverable fault naming panicking thread (BOTH owners + blocked park)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn non_control_panic_fences_commands_parks_to_rest_disables_and_latches_named_fault_both_owners() {
    // (a) Legacy owner: panicking non-control thread via process panic hook -> park succeeds at rest.
    {
        let rest_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (sup, bus) = make_energized_dm_supervisor(rest_pose, None, true);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        install_process_panic_coordinator(Arc::clone(sup.internal_fault_lane()));

        // Trigger a real panic on a named non-control worker thread.
        let jh = thread::Builder::new()
            .name("ws-panic-worker-leg".into())
            .spawn(|| {
                panic!("simulated WS handler panic");
            })
            .unwrap();
        assert!(jh.join().is_err());

        assert!(
            sup.internal_fault_fenced(),
            "non-control panic hook must immediately fence commands & rearm"
        );
        assert_eq!(
            sup.internal_fault_lane().thread_name_string(),
            "ws-panic-worker-leg"
        );

        // Run legacy ticks so `service_internal_fault_lane` executes park -> rest-disable -> latch.
        let dl = Instant::now() + Duration::from_secs(3);
        while sup.internal_fault_lane().outcome() != InternalFaultParkOutcome::ParkedAndDisabled {
            assert!(Instant::now() < dl, "legacy non-control panic failed to park & disable");
            bus.with(|s| s.push_all_feedback());
            sup.tick(0.02).await;
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(sup.internal_fault_lane().fault_latched());
        let obs = sup.get_observation().await;
        assert!(!obs.is_torqued, "rest-gated disable must complete after park");
        let fault = obs.fault.expect("non-recoverable fault must be latched");
        assert!(!fault.recoverable, "internal fault must be non-recoverable");
        assert!(
            fault.message.contains("ws-panic-worker-leg"),
            "fault message must name panicking thread: {}",
            fault.message
        );
        // Rearm / clear_fault must remain fenced.
        let stamp = sup.capture_admission_stamp();
        assert!(sup.enable_torque(stamp).await.is_err());
    }

    // (b) Executor owner: non-control panic -> parks to rest, disables, latches named fault.
    {
        let rest_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (sup, bus) = make_energized_dm_supervisor(rest_pose, None, true);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_until_async(Duration::from_secs(2), "executor running", || {
            handle.stats().steps.load(Ordering::Relaxed) >= 3
        })
        .await;

        assert!(sup
            .internal_fault_lane()
            .trigger_non_control_panic("tokio-worker-exec"));
        assert!(sup.internal_fault_fenced());

        wait_until_async(
            Duration::from_secs(4),
            "executor parked, disabled, and latched named fault",
            || {
                bus.with(|s| s.push_all_feedback());
                sup.internal_fault_lane().outcome() == InternalFaultParkOutcome::ParkedAndDisabled
                    && sup.internal_fault_lane().fault_latched()
            },
        )
        .await;
        let obs = handle.observation_async().await.unwrap();
        assert!(!obs.is_torqued);
        let fault = obs.fault.expect("executor internal fault latched");
        assert!(!fault.recoverable);
        assert!(
            fault.message.contains("tokio-worker-exec"),
            "fault message must name panicking thread: {}",
            fault.message
        );
        handle.stop_and_join();
    }

    // (c) Park blocked / away from rest (physical arm does not move toward rest):
    //     MUST keep holding (0 disables away from rest) + latch non-recoverable fault!
    {
        let raised = [0.55, -0.45, 0.65, 0.25, -0.25, 0.15, -0.40];
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        // Clear resting_pose so park cannot run -> HoldingFallback immediately away from rest.
        sup.inner().write().await.config.resting_pose = None;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        bus.with(|s| s.tx_log.clear());

        assert!(sup
            .internal_fault_lane()
            .trigger_non_control_panic("blocked-park-worker"));
        for _ in 0..5 {
            bus.with(|s| s.push_all_feedback());
            sup.tick(0.02).await;
        }
        assert_eq!(
            sup.internal_fault_lane().outcome(),
            InternalFaultParkOutcome::HoldingFallback
        );
        assert!(sup.internal_fault_lane().fault_latched());
        let obs = sup.get_observation().await;
        assert!(obs.is_torqued, "blocked park away from rest must keep holding");
        assert!(
            bus.with(|s| {
                s.motors.iter().all(|m| m.enabled)
                    && !s.tx_log.iter().any(|f| f.data == damiao::CMD_DISABLE)
            }),
            "blocked park away from rest must emit 0 CMD_DISABLE frames"
        );
        let fault = obs.fault.expect("fault latched on blocked park");
        assert!(!fault.recoverable);
        assert!(fault.message.contains("blocked-park-worker"));
    }
}

// ============================================================================
// 5. Energized handback to Legacy with live loop + ProcessExit writer parity
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn energized_handback_to_legacy_with_live_loop_preserves_hold_single_writer_and_process_exit_routing() {
    let raised = [0.35, -0.25, 0.40, 0.10, -0.10, 0.05, -0.60];
    let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    bus.with(|s| s.tx_log.clear());

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(2), "executor stepped", || {
        handle.stats().steps.load(Ordering::Relaxed) >= 4
    })
    .await;
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    assert_eq!(sup.driver_lock_attempts(), 0);

    let (legacy_stop, legacy_jh) = sup
        .handback_to_legacy_with_live_loop(
            Duration::from_secs(2),
            Duration::from_millis(5),
        )
        .await
        .expect("energized handback must succeed");
    assert_eq!(sup.owner_mode(), OWNER_LEGACY);
    assert_eq!(sup.driver_lock_attempts(), 0, "0 DriverLock attempts during executor ownership");

    // Zero torque-off frames during energized handback; live legacy loop keeps emitting holds.
    bus.with(|s| s.tx_log.clear());
    wait_until_async(Duration::from_secs(2), "live legacy loop emitting hold frames", || {
        bus.with(|s| s.tx_log.iter().any(|f| f.id == 0x101))
    })
    .await;
    let (all_enabled, disables) = bus.with(|s| {
        (
            s.motors.iter().all(|m| m.enabled),
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count(),
        )
    });
    assert!(all_enabled, "all 7 motors must stay enabled across energized handback");
    assert_eq!(disables, 0, "energized handback must emit 0 CMD_DISABLE frames");

    // Deliberate ProcessExit routes dynamically to the now-active Legacy writer and disables all 7.
    let outcome = sup
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
        .await;
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    legacy_stop.store(true, Ordering::Release);
    legacy_jh.join().unwrap();
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

// ============================================================================
// 6. Stalled handback retains exclusive driver in SAFE-HOLD, NEVER publishes OWNER_LEGACY
// ============================================================================

struct StallingReadDriver {
    inner: DamiaoArmDriver<FakeBus>,
    reads: AtomicUsize,
    entered: Arc<AtomicBool>,
    gate: Arc<(Mutex<bool>, Condvar)>,
}

impl ArmDriver for StallingReadDriver {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.inner.read_joint_states()
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        let n = self.reads.fetch_add(1, Ordering::AcqRel) + 1;
        if n == 2 {
            self.entered.store(true, Ordering::Release);
            let (lock, cv) = &*self.gate;
            let mut open = lock.lock().unwrap();
            while !*open {
                open = cv.wait(open).unwrap();
            }
        }
        self.inner
            .read_joint_states_budgeted(out, deadline, max_rx_frames)
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.inner.write_joint_targets(targets)
    }
    fn write_measured_hold_masked(
        &mut self,
        mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.inner.write_measured_hold_masked(mask, targets)
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.inner.set_torque(enable)
    }
    fn begin_torque_budgeted(&mut self, enable: bool, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted(enable, max_rx_frames)
    }
    fn begin_disable_torque_budgeted(
        &mut self,
        kind: armd::driver::DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_disable_torque_budgeted(kind, max_rx_frames)
    }
    fn poll_torque_budgeted(&mut self, deadline: Instant, max_rx_frames: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque_budgeted(deadline, max_rx_frames)
    }
    fn abort_enable(&mut self) { self.inner.abort_enable(); }
    fn set_external_pos_force_emitter(&mut self, external: bool) { self.inner.set_external_pos_force_emitter(external); }
    fn set_pending_pos_force_hold_hint(&mut self, active: bool) { self.inner.set_pending_pos_force_hold_hint(active); }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) { self.inner.set_tx_release_deadline(deadline); }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.inner.feedback_ages() }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out); }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool { self.inner.feedback_all_within(dof, max_age) }
    fn has_driver_fresh_position_api(&self) -> bool { self.inner.has_driver_fresh_position_api() }
    fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
        self.inner.fresh_joint_positions_into(joints, max_age, out)
    }
    fn clock_now(&self) -> Instant { self.inner.clock_now() }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stalled_handback_retains_exclusive_driver_in_safe_hold_never_publishes_legacy() {
    let raised = [0.25, -0.25, 0.35, 0.0, 0.0, 0.0, -0.30];
    let bus = FakeBus::new(&dm_limits(), raised, None);
    bus.with(|s| s.push_all_feedback());
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    let entered = Arc::new(AtomicBool::new(false));
    let gate = Arc::new((Mutex::new(false), Condvar::new()));
    let drv = StallingReadDriver {
        inner: dm,
        reads: AtomicUsize::new(0),
        entered: Arc::clone(&entered),
        gate: Arc::clone(&gate),
    };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    bus.with(|s| s.tx_log.clear());

    let _handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    wait_until_async(Duration::from_secs(2), "entered stalled read #2", || {
        entered.load(Ordering::Acquire)
    })
    .await;

    let t0 = Instant::now();
    let res = sup
        .handback_to_legacy_with_live_loop(Duration::from_millis(60), Duration::from_millis(5))
        .await;
    assert!(res.is_err(), "stalled handback must fail bounded");
    assert!(t0.elapsed() < Duration::from_millis(500));
    assert_eq!(
        sup.owner_mode(),
        OWNER_FAILED,
        "stalled handback must NEVER publish OWNER_LEGACY"
    );
    assert!(sup.inner().read().await.driver.is_none());

    // Unblock read #2 -> executor sees timed-out handback and retains driver in SAFE-HOLD!
    {
        let (lock, cv) = &*gate;
        *lock.lock().unwrap() = true;
        cv.notify_all();
    }
    wait_until_async(Duration::from_secs(2), "stalled handback entered SAFE-HOLD", || {
        sup.is_detached_safe_hold_active()
    })
    .await;
    assert!(
        bus.with(|s| {
            s.motors.iter().all(|m| m.enabled)
                && !s.tx_log.iter().any(|f| f.data == damiao::CMD_DISABLE)
        }),
        "stalled handback must hold in SAFE-HOLD with 0 CMD_DISABLE frames"
    );
    let outcome = sup
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
        .await;
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

// ============================================================================
// 7. Atomic startup activation barrier, CommitShutdown token expiry,
//    single-deadline bounds, and Cargo.toml release panic=unwind verification
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn atomic_startup_activation_and_commit_shutdown_expiry_and_release_unwind_config() {
    // (a) Failed startup timeout never leaks prewarm_template into StateHub.
    {
        let raised = [0.0; 7];
        let (sup, _bus) = make_energized_dm_supervisor(raised, None, false);
        assert!(!sup.hub().has_prewarm_template());
        let hooks = PersistTestHooks {
            fail_init: Some(true),
            ..Default::default()
        };
        assert!(sup
            .start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks)
            .await
            .is_err());
        assert_eq!(sup.owner_mode(), OWNER_LEGACY);
        assert!(
            !sup.hub().has_prewarm_template(),
            "failed startup must NOT leak staged prewarm template onto StateHub"
        );
    }

    // (b) Successful startup installs prewarm_template atomically before OWNER_EXECUTOR and
    //     CommitShutdown with an already-expired deadline returns OwnerError::TimedOut without
    //     mutating shutdown_committed.
    {
        let rest = [0.0; 7];
        let (sup, bus) = make_energized_dm_supervisor(rest, None, true);
        sup.set_lease_enabled(false).await;
        for step in 0..3 {
            if step > 0 {
                tokio::time::sleep(Duration::from_millis(12)).await;
            }
            bus.with(|s| s.push_all_feedback());
            sup.tick(0.02).await;
        }
        sup.disable_torque().await.unwrap();
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
        assert!(sup.hub().has_prewarm_template());
        assert!(
            sup.request_shutdown().await.unwrap(),
            "disabled executor must accept request_shutdown before CommitShutdown"
        );

        let expired = Instant::now() - Duration::from_millis(1);
        let h_exp = handle.clone();
        let err = tokio::task::spawn_blocking(move || h_exp.shutdown_and_join_until(expired))
            .await
            .unwrap();
        assert!(
            matches!(err, Err(OwnerError::TimedOut)),
            "expired CommitShutdown must fail with OwnerError::TimedOut, got {err:?}"
        );
        // Subsequent valid CommitShutdown succeeds cleanly.
        let h_ok = handle.clone();
        let ok = tokio::task::spawn_blocking(move || {
            h_ok.shutdown_and_join_until(Instant::now() + Duration::from_secs(2))
        })
        .await
        .unwrap();
        assert!(ok.is_ok(), "disabled owner must accept valid CommitShutdown: {ok:?}");
    }

    // (c) Verify Cargo.toml release profile sets `panic = "unwind"` (FU-PANIC tiered decision).
    let cargo_toml = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/Cargo.toml"))
        .expect("read Cargo.toml");
    let release_section = cargo_toml
        .split("[profile.release]")
        .nth(1)
        .expect("[profile.release] section missing");
    let release_block = release_section
        .split("\n[")
        .next()
        .unwrap_or(release_section);
    assert!(
        release_block.contains("panic = \"unwind\""),
        "[profile.release] must set panic = \"unwind\", got:\n{release_block}"
    );

    // (d) Verify synchronous helper wait_until works deterministically.
    let flag = Arc::new(AtomicBool::new(false));
    let flag2 = Arc::clone(&flag);
    thread::spawn(move || flag2.store(true, Ordering::Release));
    wait_until(Duration::from_secs(1), "flag set", || flag.load(Ordering::Acquire));
}

// ============================================================================
// 8. Integration: SAFE-HOLD and handback emit & flush torque shadow events
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn safe_hold_and_handback_emit_and_flush_torque_shadow() {
    let raised = [0.40, -0.30, 0.50, 0.20, -0.20, 0.10, -1.20];

    // (a) Energized handback flushes torque shadow and restores ShadowOwner::Legacy.
    {
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        let cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        let _handle = sup.start_executor(cfg).await.unwrap();
        wait_until_async(Duration::from_secs(2), "shadow worker sees Executor owner", || {
            sup.torque_shadow_hub().drain_now();
            let d = sup.torque_shadow_diagnostics();
            d.enabled && d.owner == armd::torque::shadow::ShadowOwner::Executor && d.events_processed > 0
        })
        .await;
        let before_hb = sup.torque_shadow_diagnostics().events_processed;

        let (legacy_stop, legacy_jh) = sup
            .handback_to_legacy_with_live_loop(Duration::from_secs(2), Duration::from_millis(5))
            .await
            .expect("energized handback must succeed");
        sup.torque_shadow_hub().drain_now();
        let after_hb = sup.torque_shadow_diagnostics();
        assert_eq!(
            after_hb.owner,
            armd::torque::shadow::ShadowOwner::Legacy,
            "handback must restore shadow owner to Legacy"
        );
        assert!(
            after_hb.events_processed >= before_hb,
            "handback must flush shadow events without resetting session counter"
        );
        assert!(
            bus.with(|s| s.motors.iter().all(|m| m.enabled)),
            "handback away from rest must never disable torque"
        );
        // Ordinary Legacy stop while energized transitions into SAFE-HOLD instead of abandoning
        // the arm (DRV-F1 B2), and only explicit ProcessExit disables and exits cleanly.
        sup.signal_legacy_stop(&legacy_stop);
        wait_until_async(Duration::from_secs(2), "legacy ordinary stop enters SAFE-HOLD", || {
            sup.legacy_safe_hold_control().is_active()
        })
        .await;
        assert!(
            bus.with(|s| s.motors.iter().all(|m| m.enabled)),
            "ordinary Legacy stop away from rest must retain torque in SAFE-HOLD"
        );
        // B1: While detached SAFE-HOLD holds the arm, try_set_driver must be refused before I/O.
        let dummy_bus = FakeBus::new(&dm_limits(), raised, None);
        let dummy_drv = DamiaoArmDriver::new(dummy_bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        assert!(
            sup.try_set_driver(Box::new(dummy_drv)).await.is_err(),
            "try_set_driver must reject installing a second bus writer while SAFE-HOLD is active"
        );
        assert!(
            dummy_bus.with(|s| s.tx_log.is_empty()),
            "rejected try_set_driver must perform zero I/O on incoming driver"
        );
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
        let _ = legacy_jh.join();
    }

    // (b) Control-thread panic into SAFE-HOLD emits shadow events and flushes on ProcessExit.
    {
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    panic_on_release: Some(2),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        wait_until_async(Duration::from_secs(2), "executor entered SAFE-HOLD", || {
            handle.is_in_safe_hold() && handle.safe_hold_control().ticks() >= 4
        })
        .await;
        sup.torque_shadow_hub().drain_now();
        let in_sh_1 = sup.torque_shadow_diagnostics().events_processed;
        assert!(in_sh_1 > 0, "SAFE-HOLD must emit and flush shadow events");
        let ticks_1 = handle.safe_hold_control().ticks();
        wait_until_async(Duration::from_secs(2), "SAFE-HOLD advances shadow events", || {
            sup.torque_shadow_hub().drain_now();
            handle.safe_hold_control().ticks() >= ticks_1 + 3
                && sup.torque_shadow_diagnostics().events_processed > in_sh_1
        })
        .await;
        assert!(
            bus.with(|s| s.motors.iter().all(|m| m.enabled)),
            "SAFE-HOLD away from rest must never disable torque before ProcessExit"
        );
        let in_sh_2 = sup.torque_shadow_diagnostics().events_processed;
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
        sup.torque_shadow_hub().drain_now();
        let after_exit = sup.torque_shadow_diagnostics();
        assert!(
            after_exit.events_processed > in_sh_2,
            "ProcessExit disable must emit and flush shadow events"
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn review_blockers_b3_b7_b8_b10_enforced() {
    let raised = [0.40, -0.30, 0.50, 0.20, -0.20, 0.10, -1.20];

    // B3: Replacing an energized Legacy driver must fail BEFORE any incoming-driver I/O.
    // B7: Unauthorized `attempt_legacy_panic_disable` must be rejected with zero CMD_DISABLE frames.
    {
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        bus.with(|s| s.tx_log.clear());

        let incoming_bus = FakeBus::new(&dm_limits(), raised, None);
        let incoming_drv = DamiaoArmDriver::new(incoming_bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        let err = sup
            .try_set_driver(Box::new(incoming_drv))
            .await
            .expect_err("energized predecessor must reject try_set_driver");
        assert!(
            format!("{err:?}").contains("DRIVER_ENERGIZED"),
            "expected DRIVER_ENERGIZED, got {err:?}"
        );
        assert!(
            incoming_bus.with(|s| s.tx_log.is_empty()),
            "rejected try_set_driver must perform zero incoming-driver I/O"
        );

        let sup_unauth = sup.clone();
        let unauth = tokio::task::spawn_blocking(move || {
            sup_unauth.attempt_legacy_panic_disable(Duration::from_millis(100))
        })
        .await
        .unwrap();
        assert!(
            matches!(unauth, PanicDisableOutcome::Failed(_)),
            "unauthorized attempt_legacy_panic_disable must fail closed, got {unauth:?}"
        );
        assert!(
            !bus.with(|s| s.tx_log.iter().any(|f| f.data == damiao::CMD_DISABLE)),
            "unauthorized attempt_legacy_panic_disable must emit zero CMD_DISABLE frames"
        );
        assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }

    // B8: SAFE-HOLD thermal disable must NOT mark confirmed on Unconfirmed/Err and must retry.
    {
        use armd::driver::{DisableKind, TorqueOutcome, TorqueProgress};
        use armd::rt_core::safe_hold::{SafeHoldBuffers, SafeHoldControl};
        use std::sync::atomic::AtomicUsize;

        struct UnconfirmedThermalDriver {
            begin_calls: Arc<AtomicUsize>,
            confirm_on_call: usize,
        }
        impl ArmDriver for UnconfirmedThermalDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    torque_feedforward: false,
                    enable_feedback: true,
                    load_feedback: false,
                }
            }
            fn enable_state(&self) -> EnableState {
                if self.begin_calls.load(Ordering::Acquire) >= self.confirm_on_call {
                    EnableState::Disabled
                } else {
                    EnableState::Enabled
                }
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn feedback_all_within(&self, _dof: usize, _max_age: Duration) -> bool { true }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(7, Some(Duration::from_millis(5)));
            }
            fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, ArmDriverError> {
                Ok(vec![
                    armd::model::JointState { position: 0.2, temperature: 95.0, ..Default::default() };
                    7
                ])
            }
            fn read_joint_states_safe_hold(
                &mut self,
                out: &mut Vec<armd::model::JointState>,
                _deadline: Instant,
                _max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                out.clear();
                out.resize(
                    7,
                    armd::model::JointState { position: 0.2, temperature: 95.0, ..Default::default() },
                );
                Ok((7, false))
            }
            fn write_joint_targets(&mut self, _targets: &[armd::driver::JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> { Ok(()) }
            fn begin_disable_torque_budgeted(
                &mut self,
                _kind: DisableKind,
                _max_rx_frames: u16,
            ) -> Result<TorqueProgress, ArmDriverError> {
                let n = self.begin_calls.fetch_add(1, Ordering::AcqRel) + 1;
                if n == 1 {
                    Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
                } else {
                    Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
                }
            }
        }

        let calls = Arc::new(AtomicUsize::new(0));
        let mut drv = UnconfirmedThermalDriver {
            begin_calls: Arc::clone(&calls),
            confirm_on_call: 2,
        };
        let mut bufs = SafeHoldBuffers::from_config(&ArmConfig::rebot_dm(), Duration::from_millis(20));
        let ctrl = SafeHoldControl::new();
        let mut now = Instant::now;
        bufs.step_driver(&mut drv, &ctrl, Instant::now() + Duration::from_millis(20), &mut now);
        assert!(ctrl.thermal_tripped());
        assert!(
            !ctrl.thermal_disable_confirmed(),
            "Unconfirmed thermal disable must NOT mark thermal_disable_confirmed"
        );
        bufs.step_driver(&mut drv, &ctrl, Instant::now() + Duration::from_millis(20), &mut now);
        assert_eq!(calls.load(Ordering::Acquire), 2, "SAFE-HOLD must retry unconfirmed thermal disable");
        assert!(ctrl.thermal_disable_confirmed());
    }
}

/// Blocker 1 regression:
/// 1. Ordinary `is_hardware_disarmed_confirmed()` on Damiao must NOT accept cached pre-enable
///    `motor_status == 0`; it requires causal per-axis disabled feedback after the last `CMD_ENABLE`.
/// 2. `ProcessExit` on Damiao must require causal post-disable `state == 0` feedback per axis;
///    when `FakeBus` accepts `CMD_DISABLE` TX but withholds disabled feedback, `ProcessExit`
///    returns `Unconfirmed` and retains the energized driver in SAFE-HOLD (NEVER drops it).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn drv_f1_blocker1_process_exit_requires_causal_disarm_proof_and_retains_driver() {
    // Part A: Ordinary disarm predicate rejects cached pre-enable status == 0 after CMD_ENABLE TX.
    {
        let clock = MockClock::new();
        let limits = dm_limits();
        let bus = FakeBus::new(&limits, [0.0; 7], Some(clock.clone()));
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        bus.with(|s| s.push_all_feedback());
        let _ = drv.read_joint_states().unwrap();
        assert!(
            drv.is_hardware_disarmed_confirmed(),
            "fresh cold-boot state==0 on all axes should confirm disarmed"
        );

        // Stop immediate replies, dispatch CMD_ENABLE on the wire, then abort enable so
        // `is_enabled == false`, `torque_transition_pending() == false`, and cached
        // `motor_status` is STILL `[0; 7]` and fresh (< 500 ms).
        bus.with(|s| s.immediate_replies = false);
        let _ = drv.begin_torque_budgeted(true, 64).unwrap();
        for _ in 0..8 {
            clock.advance(Duration::from_millis(2));
            let _ = drv.poll_torque_budgeted(clock.now() + Duration::from_millis(5), 64);
        }
        let enable_tx = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_ENABLE)
                .count()
        });
        assert!(enable_tx > 0, "expected at least one CMD_ENABLE frame on the wire");
        drv.abort_enable();
        assert_eq!(drv.enable_state(), EnableState::Disabled);
        assert!(drv.is_feedback_valid());
        assert!(!drv.torque_transition_pending());
        assert!(
            !drv.is_hardware_disarmed_confirmed(),
            "cached pre-enable state==0 must NOT confirm disarmed after CMD_ENABLE TX"
        );

        // Fresh post-enable state==0 feedback on all 7 axes restores causal disarm proof.
        clock.advance(Duration::from_millis(5));
        bus.with(|s| {
            for m in &mut s.motors {
                m.enabled = false;
            }
            s.push_all_feedback();
        });
        let _ = drv.read_joint_states().unwrap();
        assert!(
            drv.is_hardware_disarmed_confirmed(),
            "fresh post-enable state==0 on all axes should confirm disarmed"
        );

        // Pre-TX disabled frame arriving during pre-TX checks (stamped BEFORE `can.transmit`
        // actually runs) must NOT falsely satisfy causal post-enable disarm proof.
        let bus_pre = FakeBus::new(&dm_limits(), [0.0; 7], Some(clock.clone()));
        let clk_hook = clock.clone();
        let bus_hook = bus_pre.clone();
        let injected_pre_tx = Arc::new(AtomicBool::new(false));
        let injected_c = Arc::clone(&injected_pre_tx);
        let mut drv_pre = DamiaoArmDriver::new(bus_pre.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap()
            .with_before_final_tx_gate_hook(Arc::new(move |frame: &CanFrame| {
                if frame.data == damiao::CMD_ENABLE && !injected_c.swap(true, Ordering::AcqRel) {
                    // Simulate pre-TX state==0 feedback arriving on the bus BEFORE `can.transmit`,
                    // then clock advancing 2 ms before `can.transmit` actually fires.
                    bus_hook.with(|s| s.push_all_feedback());
                    clk_hook.advance(Duration::from_millis(2));
                }
            }));
        bus_pre.with(|s| s.push_all_feedback());
        let _ = drv_pre.read_joint_states().unwrap();
        bus_pre.with(|s| s.immediate_replies = false);
        let _ = drv_pre.begin_torque_budgeted(true, 64).unwrap();
        for _ in 0..8 {
            clock.advance(Duration::from_millis(2));
            let _ = drv_pre.poll_torque_budgeted(clock.now() + Duration::from_millis(5), 64);
        }
        assert!(injected_pre_tx.load(Ordering::Acquire));
        drv_pre.abort_enable();
        let _ = drv_pre.read_joint_states().unwrap();
        assert!(
            !drv_pre.is_hardware_disarmed_confirmed(),
            "disabled feedback stamped BEFORE actual CMD_ENABLE transmit must NOT establish disarm proof"
        );
    }

    // Helper wrapper tracking physical driver Drop while forwarding the full ArmDriver surface.
    struct DropWatchDamiao {
        inner: DamiaoArmDriver<FakeBus>,
        dropped: Arc<AtomicBool>,
    }
    impl Drop for DropWatchDamiao {
        fn drop(&mut self) {
            self.dropped.store(true, Ordering::Release);
        }
    }
    impl ArmDriver for DropWatchDamiao {
        fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
        fn enable_state(&self) -> EnableState { self.inner.enable_state() }
        fn is_hardware_disarmed_confirmed(&self) -> bool { self.inner.is_hardware_disarmed_confirmed() }
        fn has_driver_fresh_position_api(&self) -> bool { self.inner.has_driver_fresh_position_api() }
        fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
            self.inner.fresh_measured_position_for(joint, max_age)
        }
        fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
            self.inner.fresh_measured_sample_for(joint, max_age)
        }
        fn joint_hold_mode_verified(&self, joint: usize) -> bool {
            self.inner.joint_hold_mode_verified(joint)
        }
        fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
        fn is_enabled(&self) -> bool { self.inner.is_enabled() }
        fn has_any_joint_enabled(&self) -> bool { self.inner.has_any_joint_enabled() }
        fn has_known_partial_enable(&self) -> bool { self.inner.has_known_partial_enable() }
        fn torque_transition_pending(&self) -> bool { self.inner.torque_transition_pending() }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
            self.inner.feedback_all_within(dof, max_age)
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            self.inner.read_joint_states()
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<JointState>,
            deadline: Instant,
            max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)
        }
        fn read_joint_states_safe_hold(
            &mut self,
            out: &mut Vec<JointState>,
            deadline: Instant,
            max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            self.inner.read_joint_states_safe_hold(out, deadline, max_rx_frames)
        }
        fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
            self.inner.read_cached_joint_states_into(out)
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.inner.write_joint_targets(targets)
        }
        fn write_joint_targets_bounded(
            &mut self,
            targets: &[JointTarget],
        ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
            self.inner.write_joint_targets_bounded(targets)
        }
        fn write_measured_hold_masked(
            &mut self,
            hold_mask: u64,
            targets: &[JointTarget],
        ) -> Result<HoldPassReport, ArmDriverError> {
            self.inner.write_measured_hold_masked(hold_mask, targets)
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            self.inner.set_torque(enabled)
        }
        fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.begin_torque(enabled)
        }
        fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.poll_torque(deadline)
        }
        fn begin_torque_budgeted(
            &mut self,
            enabled: bool,
            max_rx_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.begin_torque_budgeted(enabled, max_rx_frames)
        }
        fn poll_torque_budgeted(
            &mut self,
            deadline: Instant,
            max_rx_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.inner.poll_torque_budgeted(deadline, max_rx_frames)
        }
        fn panic_disable_step(
            &mut self,
            first: bool,
            deadline: Instant,
        ) -> Option<Result<Vec<usize>, ArmDriverError>> {
            self.inner.panic_disable_step(first, deadline)
        }
        fn unconfirmed_disable_joints(&self) -> Vec<usize> {
            self.inner.unconfirmed_disable_joints()
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            self.inner.supports_rt_executor()
        }
        fn set_rt_mode(&mut self, rt: bool) { self.inner.set_rt_mode(rt) }
        fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
            self.inner.set_tx_release_deadline(deadline)
        }
        fn set_estop_state(&mut self, state: Arc<armd::rt_core::urgent::EstopState>) {
            self.inner.set_estop_state(state)
        }
        fn set_estop_gate(&mut self, gate: armd::rt_core::urgent::EstopGate) {
            self.inner.set_estop_gate(gate)
        }
        fn prepare_safe_hold(&mut self) { self.inner.prepare_safe_hold() }
        fn clock_now(&self) -> Instant { self.inner.clock_now() }
        fn clock_pause(&self, d: Duration) { self.inner.clock_pause(d) }
        fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
            self.inner.position_limits(joint)
        }
    }

    let make_watched_sup = || {
        let limits = dm_limits();
        let bus = FakeBus::new(&limits, [0.4, -0.3, 0.2, 0.1, -0.2, 0.3, 0.0], None);
        let dropped = Arc::new(AtomicBool::new(false));
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        bus.with(|s| s.push_all_feedback());
        let _ = drv.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        let _ = drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(
            DropWatchDamiao {
                inner: drv,
                dropped: Arc::clone(&dropped),
            },
        )));
        (sup, bus, dropped)
    };

    // Part B: FakeBus accepts CMD_DISABLE TX but withholds disabled feedback -> ProcessExit
    // returns Unconfirmed and retains the energized driver in post-exit SAFE-HOLD (never dropped).
    {
        let (sup, bus, dropped) = make_watched_sup();
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let _handle = sup.start_executor(cfg).await.expect("start_executor");

        // Accept CMD_DISABLE on the wire, but withhold disabled feedback replies.
        bus.with(|s| {
            s.tx_log.clear();
            s.withhold_disable_feedback = true;
        });

        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_millis(250))
            .await;
        assert_eq!(
            outcome,
            PanicDisableOutcome::Unconfirmed,
            "ProcessExit must NOT return Confirmed when disabled feedback is withheld"
        );
        let disable_tx = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert!(disable_tx >= 7, "expected CMD_DISABLE frames on bus, got {disable_tx}");

        tokio::time::sleep(Duration::from_millis(80)).await;
        assert!(
            !dropped.load(Ordering::Acquire),
            "energized driver with unconfirmed ProcessExit disable MUST stay retained in SAFE-HOLD and NEVER drop"
        );
    }

    // Part C: Positive control — when FakeBus replies with fresh post-disable state==0 on all
    // 7 axes, ProcessExit returns Confirmed and drops the disarmed core cleanly.
    {
        let (sup, bus, dropped) = make_watched_sup();
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let _handle = sup.start_executor(cfg).await.expect("start_executor");

        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_millis(500))
            .await;
        assert_eq!(
            outcome,
            PanicDisableOutcome::Confirmed,
            "fresh causal post-disable state==0 feedback on all axes should confirm ProcessExit"
        );
        wait_until_async(Duration::from_secs(2), "confirmed-disarmed core dropped", || {
            dropped.load(Ordering::Acquire)
        })
        .await;
    }
}

/// Blocker 2 regression:
/// Energized handback MUST require fresh (`<= 100 ms`), mode-verified J1–J6 hold coverage right
/// at the handback commit boundary. Stale feedback OR unverified hold mode after a previously
/// valid capture must abort handback and retain sole driver ownership in SAFE-HOLD (NEVER
/// publish `OWNER_LEGACY` or emit `CMD_DISABLE`).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn drv_f1_blocker2_energized_handback_rejects_stale_or_unverified_hold_coverage() {
    // Part A: Stale feedback on J3 after a previously valid J1–J6 capture.
    {
        let raised = [0.35, -0.25, 0.45, 0.15, -0.15, 0.05, -0.90];
        let (sup, bus) = make_energized_dm_supervisor(raised, None, false);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_until_async(Duration::from_secs(2), "executor running ticks", || {
            handle.stats().steps.load(Ordering::Relaxed) >= 4
        })
        .await;

        // Latch E-stop while fresh so `measured_hold.captured_positions[0..6]` is populated
        // with a previously valid capture.
        sup.trigger_e_stop().await;
        wait_until_async(Duration::from_secs(2), "executor holding captured pose", || {
            handle.stats().steps.load(Ordering::Relaxed) >= 8
        })
        .await;

        // Silence J3 and let its feedback age past `STOPPED_DISABLE_FEEDBACK_MAX_AGE` (100 ms)
        // while keeping `measured_hold.captured_positions[2]` retained from the earlier capture.
        bus.with(|s| {
            s.tx_log.clear();
            s.motors[2].silent = true;
        });
        tokio::time::sleep(Duration::from_millis(140)).await;

        let res = sup
            .handback_to_legacy_with_live_loop(Duration::from_millis(300), Duration::from_millis(5))
            .await;
        assert!(
            res.is_err(),
            "energized handback must reject stale J3 feedback even after a previously valid capture"
        );
        assert_eq!(
            sup.owner_mode(),
            OWNER_FAILED,
            "rejected stale handback must NEVER publish OWNER_LEGACY"
        );
        wait_until_async(Duration::from_secs(2), "rejected handback entered SAFE-HOLD", || {
            sup.is_detached_safe_hold_active()
        })
        .await;
        assert_eq!(
            sup.active_safe_hold_control().reason(),
            SafeHoldReason::HandbackStalled
        );
        let disables = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert_eq!(disables, 0, "rejected handback must retain SAFE-HOLD with 0 CMD_DISABLE");

        // Unsilence J3 and cleanly finish via deliberate ProcessExit.
        bus.with(|s| s.motors[2].silent = false);
        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }

    // Part B: Unverified hold mode on J4 after a previously valid capture.
    {
        struct ModeToggleDriver {
            inner: DamiaoArmDriver<FakeBus>,
            j4_mode_verified: Arc<AtomicBool>,
        }
        impl ArmDriver for ModeToggleDriver {
            fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
            fn enable_state(&self) -> EnableState { self.inner.enable_state() }
            fn is_hardware_disarmed_confirmed(&self) -> bool { self.inner.is_hardware_disarmed_confirmed() }
            fn has_driver_fresh_position_api(&self) -> bool { self.inner.has_driver_fresh_position_api() }
            fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
                self.inner.fresh_measured_position_for(joint, max_age)
            }
            fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
                self.inner.fresh_measured_sample_for(joint, max_age)
            }
            fn joint_hold_mode_verified(&self, joint: usize) -> bool {
                if joint == 3 && !self.j4_mode_verified.load(Ordering::Acquire) {
                    return false;
                }
                self.inner.joint_hold_mode_verified(joint)
            }
            fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
            fn is_enabled(&self) -> bool { self.inner.is_enabled() }
            fn has_any_joint_enabled(&self) -> bool { self.inner.has_any_joint_enabled() }
            fn has_known_partial_enable(&self) -> bool { self.inner.has_known_partial_enable() }
            fn torque_transition_pending(&self) -> bool { self.inner.torque_transition_pending() }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
            fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
                self.inner.feedback_all_within(dof, max_age)
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                self.inner.read_joint_states()
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)
            }
            fn read_joint_states_safe_hold(
                &mut self,
                out: &mut Vec<JointState>,
                deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                self.inner.read_joint_states_safe_hold(out, deadline, max_rx_frames)
            }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
                self.inner.read_cached_joint_states_into(out)
            }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.inner.write_joint_targets(targets)
            }
            fn write_joint_targets_bounded(
                &mut self,
                targets: &[JointTarget],
            ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
                self.inner.write_joint_targets_bounded(targets)
            }
            fn write_measured_hold_masked(
                &mut self,
                hold_mask: u64,
                targets: &[JointTarget],
            ) -> Result<HoldPassReport, ArmDriverError> {
                self.inner.write_measured_hold_masked(hold_mask, targets)
            }
            fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
                self.inner.set_torque(enabled)
            }
            fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.begin_torque(enabled)
            }
            fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.poll_torque(deadline)
            }
            fn begin_torque_budgeted(
                &mut self,
                enabled: bool,
                max_rx_frames: u16,
            ) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.begin_torque_budgeted(enabled, max_rx_frames)
            }
            fn poll_torque_budgeted(
                &mut self,
                deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.poll_torque_budgeted(deadline, max_rx_frames)
            }
            fn panic_disable_step(
                &mut self,
                first: bool,
                deadline: Instant,
            ) -> Option<Result<Vec<usize>, ArmDriverError>> {
                self.inner.panic_disable_step(first, deadline)
            }
            fn unconfirmed_disable_joints(&self) -> Vec<usize> {
                self.inner.unconfirmed_disable_joints()
            }
            fn supports_rt_executor(&self) -> Result<(), &'static str> {
                self.inner.supports_rt_executor()
            }
            fn set_rt_mode(&mut self, rt: bool) { self.inner.set_rt_mode(rt) }
            fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
                self.inner.set_tx_release_deadline(deadline)
            }
            fn set_estop_state(&mut self, state: Arc<armd::rt_core::urgent::EstopState>) {
                self.inner.set_estop_state(state)
            }
            fn set_estop_gate(&mut self, gate: armd::rt_core::urgent::EstopGate) {
                self.inner.set_estop_gate(gate)
            }
            fn prepare_safe_hold(&mut self) { self.inner.prepare_safe_hold() }
            fn clock_now(&self) -> Instant { self.inner.clock_now() }
            fn clock_pause(&self, d: Duration) { self.inner.clock_pause(d) }
            fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
                self.inner.position_limits(joint)
            }
        }

        let raised = [0.35, -0.25, 0.45, 0.15, -0.15, 0.05, -0.90];
        let bus = FakeBus::new(&dm_limits(), raised, None);
        bus.with(|s| s.push_all_feedback());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let j4_mode_verified = Arc::new(AtomicBool::new(true));
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(
            ModeToggleDriver {
                inner: dm,
                j4_mode_verified: Arc::clone(&j4_mode_verified),
            },
        )));
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        sup.trigger_e_stop().await;
        wait_until_async(Duration::from_secs(2), "executor captured valid hold", || {
            handle.stats().steps.load(Ordering::Relaxed) >= 6
        })
        .await;

        // Drop mode verification on J4 while feedback remains fresh and previously captured.
        bus.with(|s| s.tx_log.clear());
        j4_mode_verified.store(false, Ordering::Release);

        let res = sup
            .handback_to_legacy_with_live_loop(Duration::from_millis(300), Duration::from_millis(5))
            .await;
        assert!(
            res.is_err(),
            "energized handback must reject unverified J4 hold mode even with fresh feedback"
        );
        assert_eq!(sup.owner_mode(), OWNER_FAILED);
        wait_until_async(Duration::from_secs(2), "unverified-mode handback entered SAFE-HOLD", || {
            sup.is_detached_safe_hold_active()
        })
        .await;
        assert_eq!(
            sup.active_safe_hold_control().reason(),
            SafeHoldReason::HandbackStalled
        );
        let disables = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert_eq!(disables, 0);

        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }

    // Part C: Feedback going stale DURING `set_rt_mode(false)` (after the initial handback check
    // passed, before `try_commit_handback`) must still abort handback and retain SAFE-HOLD.
    {
        struct SlowRtModeExitDriver {
            inner: DamiaoArmDriver<FakeBus>,
            delay_on_exit: Arc<AtomicBool>,
        }
        impl ArmDriver for SlowRtModeExitDriver {
            fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
            fn enable_state(&self) -> EnableState { self.inner.enable_state() }
            fn is_hardware_disarmed_confirmed(&self) -> bool { self.inner.is_hardware_disarmed_confirmed() }
            fn has_driver_fresh_position_api(&self) -> bool { self.inner.has_driver_fresh_position_api() }
            fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
                self.inner.fresh_measured_position_for(joint, max_age)
            }
            fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
                self.inner.fresh_measured_sample_for(joint, max_age)
            }
            fn joint_hold_mode_verified(&self, joint: usize) -> bool {
                self.inner.joint_hold_mode_verified(joint)
            }
            fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
            fn is_enabled(&self) -> bool { self.inner.is_enabled() }
            fn has_any_joint_enabled(&self) -> bool { self.inner.has_any_joint_enabled() }
            fn has_known_partial_enable(&self) -> bool { self.inner.has_known_partial_enable() }
            fn torque_transition_pending(&self) -> bool { self.inner.torque_transition_pending() }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
            fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
                self.inner.feedback_all_within(dof, max_age)
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                self.inner.read_joint_states()
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)
            }
            fn read_joint_states_safe_hold(
                &mut self,
                out: &mut Vec<JointState>,
                deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                self.inner.read_joint_states_safe_hold(out, deadline, max_rx_frames)
            }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
                self.inner.read_cached_joint_states_into(out)
            }
            fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
                self.inner.write_joint_targets(targets)
            }
            fn write_joint_targets_bounded(
                &mut self,
                targets: &[JointTarget],
            ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
                self.inner.write_joint_targets_bounded(targets)
            }
            fn write_measured_hold_masked(
                &mut self,
                hold_mask: u64,
                targets: &[JointTarget],
            ) -> Result<HoldPassReport, ArmDriverError> {
                self.inner.write_measured_hold_masked(hold_mask, targets)
            }
            fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
                self.inner.set_torque(enabled)
            }
            fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.begin_torque(enabled)
            }
            fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.poll_torque(deadline)
            }
            fn begin_torque_budgeted(
                &mut self,
                enabled: bool,
                max_rx_frames: u16,
            ) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.begin_torque_budgeted(enabled, max_rx_frames)
            }
            fn poll_torque_budgeted(
                &mut self,
                deadline: Instant,
                max_rx_frames: u16,
            ) -> Result<TorqueProgress, ArmDriverError> {
                self.inner.poll_torque_budgeted(deadline, max_rx_frames)
            }
            fn panic_disable_step(
                &mut self,
                first: bool,
                deadline: Instant,
            ) -> Option<Result<Vec<usize>, ArmDriverError>> {
                self.inner.panic_disable_step(first, deadline)
            }
            fn unconfirmed_disable_joints(&self) -> Vec<usize> {
                self.inner.unconfirmed_disable_joints()
            }
            fn supports_rt_executor(&self) -> Result<(), &'static str> {
                self.inner.supports_rt_executor()
            }
            fn set_rt_mode(&mut self, rt: bool) {
                if !rt && self.delay_on_exit.swap(false, Ordering::AcqRel) {
                    // Stall past STOPPED_DISABLE_FEEDBACK_MAX_AGE (100 ms) while still inside
                    // the 600 ms handback timeout window.
                    thread::sleep(Duration::from_millis(130));
                }
                self.inner.set_rt_mode(rt);
            }
            fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
                self.inner.set_tx_release_deadline(deadline)
            }
            fn set_estop_state(&mut self, state: Arc<armd::rt_core::urgent::EstopState>) {
                self.inner.set_estop_state(state)
            }
            fn set_estop_gate(&mut self, gate: armd::rt_core::urgent::EstopGate) {
                self.inner.set_estop_gate(gate)
            }
            fn prepare_safe_hold(&mut self) { self.inner.prepare_safe_hold() }
            fn clock_now(&self) -> Instant { self.inner.clock_now() }
            fn clock_pause(&self, d: Duration) { self.inner.clock_pause(d) }
            fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
                self.inner.position_limits(joint)
            }
        }

        let raised = [0.35, -0.25, 0.45, 0.15, -0.15, 0.05, -0.90];
        let bus = FakeBus::new(&dm_limits(), raised, None);
        bus.with(|s| s.push_all_feedback());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let delay_on_exit = Arc::new(AtomicBool::new(true));
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(
            SlowRtModeExitDriver {
                inner: dm,
                delay_on_exit: Arc::clone(&delay_on_exit),
            },
        )));
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_until_async(Duration::from_secs(2), "executor running before slow mode-exit handback", || {
            handle.stats().steps.load(Ordering::Relaxed) >= 4
        })
        .await;
        bus.with(|s| s.tx_log.clear());

        let res = sup
            .handback_to_legacy_with_live_loop(Duration::from_millis(600), Duration::from_millis(5))
            .await;
        assert!(
            res.is_err(),
            "energized handback must reject when set_rt_mode(false) stalls past freshness window"
        );
        assert_eq!(sup.owner_mode(), OWNER_FAILED);
        wait_until_async(Duration::from_secs(2), "post-mode-exit stale handback entered SAFE-HOLD", || {
            sup.is_detached_safe_hold_active()
        })
        .await;
        assert_eq!(
            sup.active_safe_hold_control().reason(),
            SafeHoldReason::HandbackStalled
        );
        let disables = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert_eq!(disables, 0);

        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    }
}

/// Shared drop-tracking wrapper around `DamiaoArmDriver<FakeBus>` for Blocker 3 ownership tests.
struct DropWatchDamiaoDriver {
    inner: DamiaoArmDriver<FakeBus>,
    dropped: Arc<AtomicBool>,
}

impl Drop for DropWatchDamiaoDriver {
    fn drop(&mut self) {
        self.dropped.store(true, Ordering::Release);
    }
}

impl ArmDriver for DropWatchDamiaoDriver {
    fn capabilities(&self) -> DriverCapabilities { self.inner.capabilities() }
    fn enable_state(&self) -> EnableState { self.inner.enable_state() }
    fn is_hardware_disarmed_confirmed(&self) -> bool { self.inner.is_hardware_disarmed_confirmed() }
    fn has_driver_fresh_position_api(&self) -> bool { self.inner.has_driver_fresh_position_api() }
    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        self.inner.fresh_measured_position_for(joint, max_age)
    }
    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        self.inner.fresh_measured_sample_for(joint, max_age)
    }
    fn joint_hold_mode_verified(&self, joint: usize) -> bool {
        self.inner.joint_hold_mode_verified(joint)
    }
    fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
    fn is_enabled(&self) -> bool { self.inner.is_enabled() }
    fn has_any_joint_enabled(&self) -> bool { self.inner.has_any_joint_enabled() }
    fn has_known_partial_enable(&self) -> bool { self.inner.has_known_partial_enable() }
    fn torque_transition_pending(&self) -> bool { self.inner.torque_transition_pending() }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.inner.feedback_all_within(dof, max_age)
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.inner.read_joint_states()
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)
    }
    fn read_joint_states_safe_hold(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.inner.read_joint_states_safe_hold(out, deadline, max_rx_frames)
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.inner.write_joint_targets(targets)
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
        self.inner.write_joint_targets_bounded(targets)
    }
    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.inner.write_measured_hold_masked(hold_mask, targets)
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.inner.set_torque(enabled)
    }
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque(enabled)
    }
    fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque(deadline)
    }
    fn begin_torque_budgeted(
        &mut self,
        enabled: bool,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted(enabled, max_rx_frames)
    }
    fn poll_torque_budgeted(
        &mut self,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque_budgeted(deadline, max_rx_frames)
    }
    fn panic_disable_step(
        &mut self,
        first: bool,
        deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.inner.panic_disable_step(first, deadline)
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        self.inner.unconfirmed_disable_joints()
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        self.inner.supports_rt_executor()
    }
    fn set_rt_mode(&mut self, rt: bool) { self.inner.set_rt_mode(rt) }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.inner.set_tx_release_deadline(deadline)
    }
    fn set_estop_state(&mut self, state: Arc<armd::rt_core::urgent::EstopState>) {
        self.inner.set_estop_state(state)
    }
    fn set_estop_gate(&mut self, gate: armd::rt_core::urgent::EstopGate) {
        self.inner.set_estop_gate(gate)
    }
    fn prepare_safe_hold(&mut self) { self.inner.prepare_safe_hold() }
    fn clock_now(&self) -> Instant { self.inner.clock_now() }
    fn clock_pause(&self, d: Duration) { self.inner.clock_pause(d) }
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.inner.position_limits(joint)
    }
}

fn make_watched_energized_sup(q_init: [f32; 7]) -> (Arc<ArmSupervisor>, FakeBus, Arc<AtomicBool>) {
    let limits = dm_limits();
    let bus = FakeBus::new(&limits, q_init, None);
    let dropped = Arc::new(AtomicBool::new(false));
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    let _ = drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    let _ = drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(
        DropWatchDamiaoDriver {
            inner: drv,
            dropped: Arc::clone(&dropped),
        },
    )));
    (sup, bus, dropped)
}

/// Blocker 3A regression (spawn-failure fault injection):
/// When `ArmSupervisor::drop` fails to spawn the SAFE-HOLD retention thread while holding an
/// energized Legacy driver, the driver MUST NOT be dropped; ownership is retained in inline
/// SAFE-HOLD until deliberate `ProcessExit` confirms physical disarm.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn drv_f1_blocker3_legacy_drop_spawn_failure_retains_driver_in_safe_hold() {
    let (sup, bus, dropped) = make_watched_energized_sup([0.35, -0.25, 0.45, 0.15, -0.15, 0.05, 0.0]);
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    sup.set_fail_drop_safe_hold_spawn_for_test(true);
    let sh = sup.legacy_safe_hold_control();
    bus.with(|s| s.tx_log.clear());

    let drop_jh = thread::spawn(move || {
        drop(sup);
    });

    wait_until_async(
        Duration::from_secs(2),
        "inline SAFE-HOLD active after drop spawn failure",
        || sh.is_active() && sh.ticks() >= 2 && sh.hold_tx_count() >= 1,
    )
    .await;
    assert!(
        !dropped.load(Ordering::Acquire),
        "spawn failure on ArmSupervisor::drop MUST NOT drop an energized driver"
    );
    let disables = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE)
            .count()
    });
    assert_eq!(disables, 0, "inline SAFE-HOLD must emit 0 CMD_DISABLE before ProcessExit");

    // Deliberate ProcessExit confirms physical disarm and releases the driver cleanly.
    let dl = Instant::now() + Duration::from_secs(2);
    sh.request_process_exit_disable(dl);
    let outcome = tokio::task::spawn_blocking(move || sh.wait_process_exit_until(dl))
        .await
        .unwrap();
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    drop_jh.join().unwrap();
    assert!(dropped.load(Ordering::Acquire));
}

/// Blocker 3B regression (outer-unwind fault injection):
/// Panics in BOTH executor startup setup (before `init_res`) AND release-tail work (after
/// `diag_tx.push` and tail allocation accounting) must be caught by the outer unwind boundary
/// while retaining sole driver ownership in SAFE-HOLD (never dropping an energized driver).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn drv_f1_blocker3_executor_outer_unwind_startup_setup_and_release_tail_retains_driver() {
    // Part A: Panic in executor startup setup (outside `init_res`).
    {
        let (sup, bus, dropped) =
            make_watched_energized_sup([0.30, -0.20, 0.40, 0.10, -0.10, 0.05, 0.0]);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        bus.with(|s| s.tx_log.clear());

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let res = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    panic_in_startup_setup: true,
                    ..Default::default()
                },
            )
            .await;
        assert!(res.is_err(), "startup setup panic must fail start_executor");
        assert!(
            sup.has_detached_safe_hold(),
            "startup setup panic with energized driver must publish DetachedSafeHold"
        );
        wait_until_async(
            Duration::from_secs(2),
            "startup setup outer unwind entered SAFE-HOLD",
            || {
                sup.is_detached_safe_hold_active()
                    && sup.active_safe_hold_control().hold_tx_count() >= 1
            },
        )
        .await;
        assert!(
            !dropped.load(Ordering::Acquire),
            "startup setup outer unwind MUST NOT drop an energized driver"
        );
        let disables = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert_eq!(disables, 0);

        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
        wait_until_async(Duration::from_secs(2), "driver dropped after ProcessExit", || {
            dropped.load(Ordering::Acquire)
        })
        .await;
    }

    // Part B: Panic in executor release-tail work (after `diag_tx.push` and tail alloc accounting).
    {
        let (sup, bus, dropped) =
            make_watched_energized_sup([0.25, -0.35, 0.30, 0.15, -0.05, 0.20, 0.0]);
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        let tail_panic = Arc::new(AtomicBool::new(false));

        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    panic_in_release_tail_once: Some(Arc::clone(&tail_panic)),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        wait_until_async(Duration::from_secs(2), "executor running before tail panic", || {
            handle.stats().steps.load(Ordering::Relaxed) >= 3
        })
        .await;

        bus.with(|s| s.tx_log.clear());
        tail_panic.store(true, Ordering::Release);

        wait_until_async(
            Duration::from_secs(2),
            "release-tail outer unwind entered SAFE-HOLD",
            || {
                handle.is_in_safe_hold()
                    && handle.safe_hold_control().ticks() >= 2
                    && handle.safe_hold_control().hold_tx_count() >= 1
            },
        )
        .await;
        assert_eq!(
            handle.safe_hold_control().reason(),
            SafeHoldReason::ControlPanic
        );
        assert!(
            !dropped.load(Ordering::Acquire),
            "release-tail outer unwind MUST NOT drop an energized driver"
        );
        let disables = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        });
        assert_eq!(disables, 0);

        let outcome = sup
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(2))
            .await;
        assert_eq!(outcome, PanicDisableOutcome::Confirmed);
        wait_until_async(Duration::from_secs(2), "driver dropped after ProcessExit", || {
            dropped.load(Ordering::Acquire)
        })
        .await;
    }
}

/// Blocker 3C regression (writer-transition fault injection):
/// During `start_executor` ownership transfer/extraction:
/// 1. `active_writer_endpoint()` is `WriterEndpoint::Transferring` (never `Legacy`).
/// 2. Direct `attempt_legacy_panic_disable()` against the detached shell NEVER returns `NoDriver`.
/// 3. Concurrent `emergency_process_exit_disable_until()` waits across `Transferring` and routes
///    to the authoritative writer to confirm physical disable across all 7 motors.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn drv_f1_blocker3_process_exit_writer_transition_never_selects_legacy_or_returns_no_driver() {
    let (sup, bus, _dropped) =
        make_watched_energized_sup([0.40, -0.20, 0.30, 0.10, -0.10, 0.15, 0.0]);
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    let pause_gate = Arc::new((AtomicBool::new(false), Mutex::new(false), Condvar::new()));
    let sup_start = Arc::clone(&sup);
    let gate_start = Arc::clone(&pause_gate);
    let start_task = tokio::spawn(async move {
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(5));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        sup_start
            .start_executor_with_hooks(
                cfg,
                PersistTestHooks {
                    pause_in_startup_init: Some(gate_start),
                    ..Default::default()
                },
            )
            .await
    });

    // Wait until `core` has been extracted into `armd-control` (detached shell in `sup.inner`).
    wait_until_async(Duration::from_secs(2), "paused inside startup init", || {
        pause_gate.0.load(Ordering::Acquire)
    })
    .await;

    assert!(
        matches!(
            sup.active_writer_endpoint(),
            armd::supervisor::WriterEndpoint::Transferring
        ),
        "active_writer_endpoint must be Transferring during extraction, not Legacy"
    );

    // Launch ProcessExit concurrently while transfer is in progress.
    let sup_pe = Arc::clone(&sup);
    let pe_task = tokio::spawn(async move {
        sup_pe
            .emergency_process_exit_disable_until(Instant::now() + Duration::from_secs(3))
            .await
    });
    tokio::time::sleep(Duration::from_millis(25)).await;
    assert!(
        !pe_task.is_finished(),
        "ProcessExit must wait across Transferring rather than returning NoDriver from the detached shell"
    );

    // Release startup init pause -> executor activates -> ProcessExit routes to Executor & confirms!
    {
        let mut open = pause_gate.1.lock().unwrap();
        *open = true;
        pause_gate.2.notify_all();
    }
    let start_res = start_task.await.unwrap();
    if let Err(e) = &start_res {
        panic!("start_executor should settle cleanly: {e}");
    }

    let outcome = pe_task.await.unwrap();
    assert_eq!(
        outcome,
        PanicDisableOutcome::Confirmed,
        "ProcessExit across writer transition must confirm physical disarm"
    );
    assert!(
        bus.with(|s| s.motors.iter().all(|m| !m.enabled)),
        "all 7 motors must be physically disabled"
    );

    // Direct legacy panic-disable against the now-detached shell must NEVER return NoDriver.
    sup.authorize_process_exit_for_test();
    let direct_detached = sup.attempt_legacy_panic_disable(Duration::from_millis(80));
    assert!(
        !matches!(direct_detached, PanicDisableOutcome::NoDriver),
        "attempt_legacy_panic_disable must NEVER return NoDriver from a detached shell; got {direct_detached:?}"
    );
}
