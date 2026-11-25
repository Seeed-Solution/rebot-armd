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
    rx: VecDeque<CanFrame>,
    tx_log: Vec<CanFrame>,
    immediate_replies: bool,
    follow_targets: bool,
    clock: Option<MockClock>,
}

impl FakeBusInner {
    fn push_all_feedback(&mut self) {
        for m in &self.motors {
            if !m.silent {
                self.rx.push_back(m.feedback_frame());
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
                s.rx.push_back(fb);
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
                    s.rx.push_back(reply);
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
                s.rx.push_back(reply);
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
        let follow = s.follow_targets;
        if let Some(m) = s.motors.iter_mut().find(|m| m.id == base_id) {
            if frame.data == damiao::CMD_ENABLE {
                m.enabled = true;
            } else if frame.data == damiao::CMD_DISABLE {
                m.enabled = false;
            } else if is_pos_vel && follow && frame.data.len() >= 4 {
                let target_q = f32::from_le_bytes([
                    frame.data[0],
                    frame.data[1],
                    frame.data[2],
                    frame.data[3],
                ]);
                if target_q.is_finite() {
                    m.q = target_q;
                }
            } else if is_pos_force && follow && frame.data.len() >= 4 {
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
            if immediate && !m.silent {
                let fb = m.feedback_frame();
                s.rx.push_back(fb);
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
            .map(|f| (f, Some(now)))
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
        .handback_to_legacy(Duration::from_millis(60))
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
