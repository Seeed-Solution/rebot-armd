//! Zero-allocation proof for stopped torque-off on `armd-control` (RT invariant #5):
//! verifies 0 heap allocations and 0 frees across:
//! 1. Stopped `TorqueOff` refused (`TORQUE_OFF_WOULD_DROP` — raised & stale)
//! 2. Stopped `TorqueOff` allowed and confirmed (`Outcome::Completed`)
//! 3. Stopped `TorqueOff` allowed with TX failure (`disable_step_failed`)
//! 4. Stopped `TorqueOff` allowed with confirmation timeout (`FEEDBACK_UNCONFIRMED`)

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::driver::{ArmDriver, DamiaoArmDriver, StoppedDisableGate};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    Authority, BootId, Broker, Capacities, ClientId, Outcome, Submit, TransitionOp,
};
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture};
use armd::safety::{FaultCode, FaultReason, SafetyState};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
use armd::{ArmConfig, SupervisorInner};

thread_local! {
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn bump(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) {
        let _ = c.try_with(|n| n.set(n.get() + 1));
    }
}

// SAFETY: forwards to System; thread-local counters are const-initialized without destructors.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { bump(&ALLOCS); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { bump(&ALLOCS); unsafe { System.alloc_zeroed(l) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { bump(&FREES); unsafe { System.dealloc(p, l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        bump(&ALLOCS);
        bump(&FREES);
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

#[derive(Clone, Copy)]
struct Motor {
    id: u16,
    q: f32,
    enabled: bool,
    fault_state: Option<u8>,
    limits: DamiaoLimits,
}

impl Motor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = self.fault_state.unwrap_or(u8::from(self.enabled));
        let data = [
            (state << 4) | (self.id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            30,
            30,
        ];
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

struct BusState {
    motors: [Motor; 7],
    ctrl_modes: [u32; 7],
    rx: VecDeque<(CanFrame, Option<std::time::Instant>)>,
    clock: Arc<MockClock>,
    tx_would_block: bool,
    silent_disable: bool,
    drop_enable_mask: u8,
    mit_tx_count: u64,
}

impl BusState {
    fn push_all_feedback(&mut self) {
        let ts = self.clock.now();
        for m in &self.motors {
            self.rx.push_back((m.feedback_frame(), Some(ts)));
        }
    }
}

#[derive(Clone)]
struct ZeroAllocBus(Arc<Mutex<BusState>>);

impl ZeroAllocBus {
    fn new(clock: Arc<MockClock>) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| Motor {
            id: (i + 1) as u16,
            q: 0.0,
            enabled: false,
            fault_state: None,
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(BusState {
            motors,
            ctrl_modes: [damiao::CTRL_MODE_POS_VEL; 7],
            rx: VecDeque::with_capacity(256),
            clock,
            tx_would_block: false,
            silent_disable: false,
            drop_enable_mask: 0,
            mit_tx_count: 0,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut BusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl CanTransport for ZeroAllocBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone() as Arc<dyn DriverClock>)
    }
    fn supports_bounded_flush(&self) -> bool { true }
    fn supports_nonblocking_rt(&self) -> bool { true }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        if s.tx_would_block {
            return Err(TransportError::WouldBlock);
        }
        let silent_disable = s.silent_disable;
        let drop_enable_mask = s.drop_enable_mask;
        let now = s.clock.now();
        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(f) = (!silent_disable)
                .then(|| s.motors.iter().find(|m| m.id == id).map(Motor::feedback_frame))
                .flatten()
            {
                s.rx.push_back((f, Some(now)));
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
                    s.rx.push_back((reply, Some(now)));
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
                s.rx.push_back((reply, Some(now)));
            }
            return Ok(());
        }
        if let Some((idx, m)) = s.motors.iter_mut().enumerate().find(|(_, m)| u32::from(m.id) == frame.id) {
            if frame.data == damiao::CMD_ENABLE {
                if (drop_enable_mask & (1 << idx)) == 0 {
                    m.enabled = true;
                    let f = m.feedback_frame();
                    s.rx.push_back((f, Some(now)));
                }
            } else if frame.data == damiao::CMD_DISABLE && !silent_disable {
                m.enabled = false;
                let f = m.feedback_frame();
                s.rx.push_back((f, Some(now)));
            } else if frame.data != damiao::CMD_DISABLE {
                // MIT control / hold frame on standard motor CAN ID.
                let f = m.feedback_frame();
                s.mit_tx_count += 1;
                s.rx.push_back((f, Some(now)));
            }
            return Ok(());
        }
        if let Some(m) = s.motors.iter().find(|m| u32::from(m.id) + 0x100 == frame.id) {
            let f = m.feedback_frame();
            s.rx.push_back((f, Some(now)));
        }
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }
    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        self.0
            .lock()
            .unwrap()
            .rx
            .pop_front()
            .ok_or(TransportError::WouldBlock)
    }
    fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
}

const BOOT: BootId = BootId(77);
const RELEASE: Duration = Duration::from_millis(20);

fn build_armed_fixture(q_j2: f32) -> (ReleaseFixture<SupervisorInner>, Broker<PlanSlot>, ZeroAllocBus, Arc<MockClock>) {
    let clock = Arc::new(MockClock::new());
    let bus = ZeroAllocBus::new(clock.clone());
    let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| {
        s.motors[1].q = q_j2;
        s.push_all_feedback();
    });
    drv.read_joint_states().unwrap();
    drv.check_mapping_ranges();
    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.register_live_connection(1).unwrap();
    inner.tick(0.02);

    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
        inner,
        BOOT,
        Capacities { commands: 8, plans: 1, replies: 16 },
    );

    // Arm through the RT transition path.
    bus.with(|s| s.push_all_feedback());
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(ClientId(1), 1, auth, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    for _ in 0..20 {
        bus.with(|s| s.push_all_feedback());
        let rep = fx.step();
        clock.advance(RELEASE);
        if rep.transition_ended.is_some() { break; }
    }
    assert!(fx.core.is_torqued);

    (fx, b, bus, clock)
}

fn build_armed_faulted_fixture(q_j2: f32) -> (ReleaseFixture<SupervisorInner>, Broker<PlanSlot>, ZeroAllocBus, Arc<MockClock>) {
    let (mut fx, b, bus, clock) = build_armed_fixture(q_j2);
    // Latch Fault and warm up all triple-buffer slots so steady-state publish buffers are settled.
    fx.core.safety.trigger_fault_coded(
        FaultCode::DriverWriteFailed,
        FaultReason::Other("latched fault".into()),
    );
    for _ in 0..5 {
        bus.with(|s| s.push_all_feedback());
        fx.step();
        clock.advance(RELEASE);
    }

    (fx, b, bus, clock)
}

fn build_armed_estopped_fixture(q_j2: f32) -> (ReleaseFixture<SupervisorInner>, Broker<PlanSlot>, ZeroAllocBus, Arc<MockClock>) {
    let (mut fx, b, bus, clock) = build_armed_fixture(q_j2);
    fx.core.apply_stop_policy(armd::safety::StopPolicy::EStopLatch);
    for _ in 0..5 {
        bus.with(|s| s.push_all_feedback());
        fx.step();
        clock.advance(RELEASE);
    }
    (fx, b, bus, clock)
}

fn submit_torque_off(fx: &ReleaseFixture<SupervisorInner>, b: &mut Broker<PlanSlot>, seq: u64) {
    let auth: Authority = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(
        ClientId(1),
        seq,
        auth,
        fx.now() + Duration::from_secs(5),
        Submit::Transition(TransitionOp::TorqueOff),
    )
    .unwrap();
}

#[test]
fn stopped_torque_off_refused_zero_alloc() {
    let (mut fx, mut b, bus, clock) = build_armed_estopped_fixture(0.35);
    let epoch_before = fx.core.safety.epoch_id();
    let kinds_before = fx.core.safety.latched_fault_kinds();
    let fault_before = fx.core.safety.fault();
    bus.with(|s| s.push_all_feedback());
    submit_torque_off(&fx, &mut b, 2);

    let (rep, allocs, frees) = counted(|| fx.step());
    clock.advance(RELEASE);
    assert_eq!(rep.command.unwrap().1, Err("TORQUE_OFF_WOULD_DROP"));
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert_eq!((allocs, frees), (0, 0), "refused stopped TorqueOff must allocate 0");

    // Also verify zero-alloc direct CoreOps::commit_transition while in Fault(Other(String)).
    let (mut fx_f, _b_f, _bus_f, _clock_f) = build_armed_faulted_fixture(0.35);
    let f_epoch_before = fx_f.core.safety.epoch_id();
    let f_kinds_before = fx_f.core.safety.latched_fault_kinds();
    let f_fault_before = fx_f.core.safety.fault();
    let ctx = fx_f.ctx(fx_f.now());
    let d = armd::rt_core::broker::Descriptor {
        stamp: armd::rt_core::broker::Stamp {
            arm: ctx.arm,
            boot: ctx.boot,
            request: armd::rt_core::broker::RequestId(99),
            client: ClientId(1),
            client_seq: 99,
            authority: fx_f.core.authority(&ctx),
            deadline: ctx.now + Duration::from_secs(5),
        },
        kind: armd::rt_core::broker::CommandKind::Transition(TransitionOp::TorqueOff),
    };
    let (res, a, f) = counted(|| fx_f.core.commit_transition(&d, TransitionOp::TorqueOff, &ctx));
    assert_eq!(res.err(), Some("TORQUE_OFF_WOULD_DROP"));
    assert_eq!(fx_f.core.safety.epoch_id(), f_epoch_before);
    assert_eq!(fx_f.core.safety.latched_fault_kinds(), f_kinds_before);
    assert_eq!(fx_f.core.safety.fault(), f_fault_before);
    assert_eq!((a, f), (0, 0), "refused commit_transition in Fault(Other) must allocate 0");
}

#[test]
fn stopped_torque_off_confirmed_zero_alloc() {
    let (mut fx, mut b, bus, clock) = build_armed_estopped_fixture(0.0);
    let epoch_before = fx.core.safety.epoch_id();
    let kinds_before = fx.core.safety.latched_fault_kinds();
    let fault_before = fx.core.safety.fault();
    bus.with(|s| s.push_all_feedback());
    submit_torque_off(&fx, &mut b, 2);

    let mut total_allocs = 0u64;
    let mut total_frees = 0u64;
    let mut ended = None;
    for _ in 0..20 {
        bus.with(|s| s.push_all_feedback());
        let (rep, a, f) = counted(|| fx.step());
        total_allocs += a;
        total_frees += f;
        clock.advance(RELEASE);
        if let Some((_, o)) = rep.transition_ended {
            ended = Some(o);
            break;
        }
    }
    assert_eq!(ended, Some(Outcome::Completed));
    assert!(!fx.core.is_torqued);
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert_eq!((total_allocs, total_frees), (0, 0), "confirmed stopped TorqueOff must allocate 0");
}

#[test]
fn stopped_torque_off_tx_failure_zero_alloc() {
    let (mut fx, mut b, bus, clock) = build_armed_estopped_fixture(0.0);
    let epoch_before = fx.core.safety.epoch_id();
    let kinds_before = fx.core.safety.latched_fault_kinds();
    let fault_before = fx.core.safety.fault();
    bus.with(|s| {
        s.push_all_feedback();
        s.tx_would_block = true;
    });
    submit_torque_off(&fx, &mut b, 2);

    let (rep, a1, f1) = counted(|| fx.step());
    clock.advance(RELEASE);
    assert!(rep.command.unwrap().1.is_ok(), "commit starts transition and records terminal failure");

    let (ended, a2, f2) = if rep.transition_ended.is_some() {
        (rep.transition_ended, 0u64, 0u64)
    } else {
        let (rep2, a2, f2) = counted(|| fx.step());
        clock.advance(RELEASE);
        (rep2.transition_ended, a2, f2)
    };
    assert!(matches!(ended, Some((_, Outcome::Failed(_)))), "{rep:?}");
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert!(fx.core.disable_pending, "sticky disable_pending stays set after TX failure");
    assert_eq!(
        (a1 + a2, f1 + f2),
        (0, 0),
        "TX failure in disable_step_failed while already stopped must allocate 0"
    );

    // Trigger the periodic (>= 250 ms) background RT disable retry (`supervisor.rs:2946` ->
    // `request_disable_bounded`) while TX still fails: MUST allocate 0 and preserve EStop/epoch.
    fx.core.last_disable_attempt = Some(std::time::Instant::now() - Duration::from_millis(300));
    bus.with(|s| s.push_all_feedback());
    let (retry_rep, a3, f3) = counted(|| fx.step());
    clock.advance(RELEASE);
    assert!(retry_rep.command.is_none());
    assert!(fx.core.disable_pending);
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert_eq!(
        (a3, f3),
        (0, 0),
        "failed periodic RT background disable retry must allocate 0"
    );

    // Also verify `ImmediateOp::TorqueOff` (`torque_off_rt_immediate` -> `request_disable_bounded`)
    // TX failure allocates 0 while already stopped.
    bus.with(|s| s.push_all_feedback());
    let ctx = fx.ctx(fx.now());
    let d_imm = armd::rt_core::broker::Descriptor {
        stamp: armd::rt_core::broker::Stamp {
            arm: ctx.arm,
            boot: ctx.boot,
            request: armd::rt_core::broker::RequestId(101),
            client: ClientId(1),
            client_seq: 101,
            authority: fx.core.authority(&ctx),
            deadline: ctx.now + Duration::from_secs(5),
        },
        kind: armd::rt_core::broker::CommandKind::Immediate(
            armd::rt_core::broker::ImmediateOp::TorqueOff,
        ),
    };
    let (imm_res, a4, f4) = counted(|| {
        fx.core
            .commit_immediate(&d_imm, armd::rt_core::broker::ImmediateOp::TorqueOff, &ctx)
    });
    assert!(imm_res.is_err());
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert_eq!(
        (a4, f4),
        (0, 0),
        "failed ImmediateOp::TorqueOff while already stopped must allocate 0"
    );

    // Expiring the single 1 s request lifetime checked inside `send` must end retries with
    // 0 allocs and resume normal E-stop hold without sending disable frames or mutating EStop/epoch.
    // Refresh the empty-queue RX bound at t+980 ms before crossing the 1 s boundary so feedback
    // itself is provably fresh (<= 25 ms) and ONLY the 1 s request lifetime triggers refusal.
    clock.advance(Duration::from_millis(980));
    fx.core.last_disable_attempt = Some(std::time::Instant::now());
    bus.with(|s| s.push_all_feedback());
    let _ = fx.step();
    clock.advance(Duration::from_millis(25));
    fx.core.last_disable_attempt = Some(std::time::Instant::now() - Duration::from_millis(300));
    bus.with(|s| {
        s.tx_would_block = false;
        s.push_all_feedback();
    });
    let (_exp_rep, a5, f5) = counted(|| fx.step());
    clock.advance(RELEASE);
    assert!(!fx.core.disable_pending, "1 s lifetime expiry in send must clear disable_pending");
    assert!(fx.core.active_disable_kind().is_none());
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!((a5, f5), (0, 0), "expiring 1 s stopped-disable request lifetime in send must allocate 0");
}

#[test]
fn stopped_torque_off_confirm_timeout_zero_alloc() {
    let (mut fx, mut b, bus, clock) = build_armed_estopped_fixture(0.0);
    let epoch_before = fx.core.safety.epoch_id();
    let kinds_before = fx.core.safety.latched_fault_kinds();
    let fault_before = fx.core.safety.fault();
    bus.with(|s| {
        s.push_all_feedback();
        s.silent_disable = true;
    });
    submit_torque_off(&fx, &mut b, 2);

    let mut total_allocs = 0u64;
    let mut total_frees = 0u64;
    let mut ended = None;
    for _ in 0..80 {
        let (rep, a, f) = counted(|| fx.step());
        total_allocs += a;
        total_frees += f;
        clock.advance(RELEASE);
        if let Some((_, o)) = rep.transition_ended {
            ended = Some(o);
            break;
        }
    }
    assert_eq!(
        ended,
        Some(Outcome::Completed),
        "R10: accepted disable at rest finishes Completed after bounded repeats even when motors are silent"
    );
    assert_eq!(fx.core.torque_intent, armd::TorqueIntent::Off);
    assert!(!fx.core.is_torqued);
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert_eq!(
        (total_allocs, total_frees),
        (0, 0),
        "bounded disable repeats with silent motors while stopped must allocate 0"
    );
}

/// Review M-05 regression: RT enable failures (enable timeout AND motor hardware fault 0x0A)
/// and subsequent `FailedHold` hold ticks (including repeated `check_hardware_status` while the
/// motor fault persists) MUST perform strictly 0 heap allocations and 0 frees.
#[test]
fn rt_enable_failure_with_motor_fault_and_failed_hold_ticks_zero_alloc() {
    let build_disarmed = |q_j2: f32| {
        let clock = Arc::new(MockClock::new());
        let bus = ZeroAllocBus::new(clock.clone());
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        for i in 0..6 {
            arm_joints[i] = i;
        }
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 6,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        drv.read_joint_states().unwrap();
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(12));
            }
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        drv.check_mapping_ranges();
        drv.set_torque(false).unwrap();
        bus.with(|s| {
            s.motors[1].q = q_j2;
            s.push_all_feedback();
        });
        drv.read_joint_states().unwrap();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.safety.set_off_thread_reclaim(true);
        inner.tick(0.02);

        let (mut fx, b, _tw, _stop) = ReleaseFixture::new(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 1, replies: 16 },
        );
        // Warm up steady-state observation triple-buffer slots before counting allocations.
        for _ in 0..4 {
            bus.with(|s| s.push_all_feedback());
            fx.step();
            clock.advance(RELEASE);
        }
        (fx, b, bus, clock)
    };

    // Case A: Enable timeout (J4 drops enable frames) -> FailedHold + ENABLE_FAILED + hold ticks: (0, 0).
    {
        let (mut fx, mut b, bus, clock) = build_disarmed(0.35);
        bus.with(|s| {
            s.drop_enable_mask = 1 << 3;
            s.push_all_feedback();
        });
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();

        let mut total_a = 0u64;
        let mut total_f = 0u64;
        let mut ended = None;
        for _ in 0..25 {
            bus.with(|s| s.push_all_feedback());
            let (rep, a, f) = counted(|| fx.step());
            total_a += a;
            total_f += f;
            clock.advance(RELEASE);
            if let Some((_, o)) = rep.transition_ended {
                ended = Some(o);
                break;
            }
        }
        assert_eq!(ended, Some(Outcome::Failed("ENABLE_FAILED")));
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(fx.core.safety.fault().map(|f| f.code), Some(FaultCode::EnableFailed));
        assert!(!fx.core.safety.recoverable());
        assert_eq!((total_a, total_f), (0, 0), "RT enable timeout into FailedHold must allocate 0");

        for _ in 0..5 {
            bus.with(|s| s.push_all_feedback());
            let (_, a, f) = counted(|| fx.step());
            clock.advance(RELEASE);
            assert_eq!((a, f), (0, 0), "FailedHold hold tick after enable timeout must allocate 0");
        }
    }

    // Case B: Motor hardware fault (0x0A overcurrent on J3) during RT enable -> FailedHold +
    // subsequent hold ticks with persistent 0x0A fault status: strictly (0, 0) allocs/frees!
    // (Pre-warm observation triple-buffer `motor_errors` slots with the pre-existing motor fault
    // before submitting `TorqueOn` so the allocator guard isolates the RT enable failure +
    // `check_hardware_status` + `FailedHold` hold path.)
    {
        let (mut fx, mut b, bus, clock) = build_disarmed(0.35);
        bus.with(|s| {
            s.motors[2].fault_state = Some(0x0A);
        });
        for _ in 0..4 {
            bus.with(|s| s.push_all_feedback());
            fx.step();
            clock.advance(RELEASE);
        }
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();

        let mut total_a = 0u64;
        let mut total_f = 0u64;
        let mut ended = None;
        for _ in 0..25 {
            bus.with(|s| s.push_all_feedback());
            let (rep, a, f) = counted(|| fx.step());
            total_a += a;
            total_f += f;
            clock.advance(RELEASE);
            if let Some((_, o)) = rep.transition_ended {
                ended = Some(o);
                break;
            }
        }
        assert_eq!(ended, Some(Outcome::Failed("ENABLE_FAILED")));
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(fx.core.safety.fault().map(|f| f.code), Some(FaultCode::EnableFailed));
        assert!(!fx.core.safety.recoverable());
        assert_eq!(
            (total_a, total_f),
            (0, 0),
            "RT motor-fault enable failure into FailedHold must allocate 0"
        );

        for _ in 0..5 {
            bus.with(|s| s.push_all_feedback());
            let (_, a, f) = counted(|| fx.step());
            clock.advance(RELEASE);
            assert_eq!(
                (a, f),
                (0, 0),
                "FailedHold hold tick with persistent motor fault must allocate 0"
            );
        }
    }
}

/// Rule R1 (part 1): Hold capture uses data the owner already has, NOT the driver mutex, and
/// performs strictly 0 heap allocations and 0 frees.
/// Proves lock-freedom by poisoning `inner.driver`'s mutex after `poll_hardware_rx` populates
/// `owner_driver_snapshot` — if `snapshot_measured_hold_with_invalidated()` acquired `inner.driver`,
/// `PoisonError` would return mask 0 instead of `0b111_1111`.
#[test]
fn r1_hold_capture_uses_owner_snapshot_without_driver_mutex_zero_alloc() {
    let (mut fx, _b, bus, clock) = build_armed_fixture(0.35);
    bus.with(|s| s.push_all_feedback());
    fx.step();
    clock.advance(RELEASE);

    // Poison `inner.driver` so any attempt to lock `self.driver` during hold capture would fail.
    let _ = std::thread::scope(|s| {
        let drv = fx
            .core
            .driver
            .as_ref()
            .expect("driver attached")
            .as_legacy_lock()
            .expect("legacy driver");
        s.spawn(move || {
            let _guard = drv.lock().unwrap();
            panic!("intentional poison to prove hold capture never locks driver mutex");
        })
        .join()
    });
    assert!(
        fx.core
            .driver
            .as_ref()
            .unwrap()
            .as_legacy_lock()
            .unwrap()
            .is_poisoned(),
        "driver mutex must be poisoned before testing lock-free hold capture"
    );

    let ((hold_mask, _invalidated), a, f) =
        counted(|| fx.core.snapshot_measured_hold_with_invalidated());
    assert_eq!(
        hold_mask, 0b111_1111,
        "R1: hold capture must use owner-held snapshot (all 7 joints eligible without locking driver mutex)"
    );
    assert_eq!(
        (a, f),
        (0, 0),
        "R1: lock-free owner hold capture must perform strictly 0 allocs and 0 frees"
    );
}

fn feetech_sync_read_reply_bytes(ids: &[u8], steps: &[u16]) -> Vec<u8> {
    let mut out = Vec::new();
    for (&id, &pos) in ids.iter().zip(steps.iter()) {
        let [lo, hi] = pos.to_le_bytes();
        // 8-byte status payload: pos_l, pos_h, vel_l, vel_h, load_l, load_h, volt, temp
        let params = [lo, hi, 0, 0, 0, 0, 120, 30];
        let length = (params.len() + 2) as u8;
        let checksum = armd::protocol::feetech::calculate_checksum(id, length, 0, &params);
        out.extend_from_slice(&armd::protocol::feetech::FEETECH_HEADER);
        out.push(id);
        out.push(length);
        out.push(0); // error = 0
        out.extend_from_slice(&params);
        out.push(checksum);
    }
    out
}

/// Rule R1 (part 2): Feetech `FailedHold` uses preallocated buffers for positions and the
/// `SYNC_WRITE` packet and performs strictly 0 heap allocations and 0 frees.
#[test]
fn r1_feetech_failed_hold_zero_alloc() {
    use armd::driver::{FeetechArmDriver, JointHoldStatus, JointTarget};

    let clock = Arc::new(MockClock::new());

    // 6-DOF zero-alloc Feetech transport that answers chunk 1 ([1,2,3]) and chunk 2 ([4,5,6])
    // from pre-encoded fixed buffers on SYNC_READ without any heap allocation.
    #[derive(Clone)]
    struct ZeroAllocFeetechBus {
        clock: Arc<MockClock>,
        state: Arc<Mutex<ZeroAllocFeetechBusState>>,
    }
    struct ZeroAllocFeetechBusState {
        chunk1_reply: [u8; 64],
        chunk1_len: usize,
        chunk2_reply: [u8; 64],
        chunk2_len: usize,
        rx: [u8; 128],
        rx_head: usize,
        rx_tail: usize,
        last_tx: [u8; 128],
        last_tx_len: usize,
        sync_write_count: u64,
    }
    impl armd::transport::Transport for ZeroAllocFeetechBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(self.clock.clone() as Arc<dyn DriverClock>)
        }
        fn write(&mut self, data: &[u8]) -> Result<usize, TransportError> {
            self.write_all(data)?;
            Ok(data.len())
        }
        fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
            let mut s = self.state.lock().unwrap();
            let copy = data.len().min(s.last_tx.len());
            s.last_tx[..copy].copy_from_slice(&data[..copy]);
            s.last_tx_len = copy;
            if data.len() >= 5 && data[4] == armd::protocol::feetech::INST_SYNC_WRITE {
                s.sync_write_count += 1;
            } else if data.len() >= 8 && data[4] == armd::protocol::feetech::INST_SYNC_READ {
                // First servo ID in SYNC_READ params is at byte 7.
                let first_id = data[7];
                let (src_len, is_chunk1) = if first_id == 1 {
                    (s.chunk1_len, true)
                } else {
                    (s.chunk2_len, false)
                };
                s.rx_head = 0;
                s.rx_tail = src_len;
                for i in 0..src_len {
                    s.rx[i] = if is_chunk1 { s.chunk1_reply[i] } else { s.chunk2_reply[i] };
                }
            }
            Ok(())
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            let mut s = self.state.lock().unwrap();
            let avail = s.rx_tail.saturating_sub(s.rx_head);
            if avail == 0 {
                drop(s);
                self.clock.advance(Duration::from_millis(1));
                return Ok(0);
            }
            let n = avail.min(buf.len());
            buf[..n].copy_from_slice(&s.rx[s.rx_head..s.rx_head + n]);
            s.rx_head += n;
            Ok(n)
        }
        fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
        fn clear_input_buffer(&mut self) {
            let mut s = self.state.lock().unwrap();
            s.rx_head = 0;
            s.rx_tail = 0;
        }
    }

    let c1 = feetech_sync_read_reply_bytes(&[1, 2, 3], &[2048, 2100, 1980]);
    let c2 = feetech_sync_read_reply_bytes(&[4, 5, 6], &[2000, 2050, 2090]);
    let mut c1_arr = [0u8; 64];
    c1_arr[..c1.len()].copy_from_slice(&c1);
    let mut c2_arr = [0u8; 64];
    c2_arr[..c2.len()].copy_from_slice(&c2);

    let bus = ZeroAllocFeetechBus {
        clock: clock.clone(),
        state: Arc::new(Mutex::new(ZeroAllocFeetechBusState {
            chunk1_reply: c1_arr,
            chunk1_len: c1.len(),
            chunk2_reply: c2_arr,
            chunk2_len: c2.len(),
            rx: [0; 128],
            rx_head: 0,
            rx_tail: 0,
            last_tx: [0; 128],
            last_tx_len: 0,
            sync_write_count: 0,
        })),
    };
    let mut drv = FeetechArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6])
        .with_calibrated(true)
        .with_tx_echo(Some(false));
    drv.read_joint_states().unwrap();
    assert!(drv.is_feedback_valid());

    // 1. Direct `FeetechArmDriver::write_measured_hold_masked` (6-DOF): strictly (0, 0).
    let targets = vec![JointTarget::position(0.0); 6];
    let (report, a, f) = counted(|| drv.write_measured_hold_masked(0b11_1111, &targets).unwrap());
    assert!(report.sent_any, "Feetech FailedHold must transmit SYNC_WRITE");
    assert_eq!(report.joint_status[..6], [JointHoldStatus::Held; 6]);
    assert_eq!(
        (a, f),
        (0, 0),
        "R1: FeetechArmDriver::write_measured_hold_masked must use preallocated buffers (0 allocs/frees)"
    );
    {
        let st = bus.state.lock().unwrap();
        assert_eq!(st.sync_write_count, 1);
        assert_eq!(&st.last_tx[0..2], &[0xFF, 0xFF]);
        assert_eq!(st.last_tx[4], armd::protocol::feetech::INST_SYNC_WRITE);
        assert_eq!(st.last_tx_len, 8 + 6 * 3);
    }

    // 2. Full supervisor `FailedHold` stopped-hold pass with FeetechArmDriver: strictly (0, 0).
    let mut drv2 = FeetechArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6])
        .with_calibrated(true)
        .with_tx_echo(Some(false));
    drv2.read_joint_states().unwrap();
    let cfg2 = ArmConfig {
        dof: 6,
        lease_timeout_ms: 600_000,
        ..ArmConfig::default()
    };
    let mut inner2 = SupervisorInner::new(cfg2).with_driver(Box::new(drv2));
    inner2.lease_enabled = false;
    inner2.safety.set_off_thread_reclaim(true);
    let stamp = inner2.current_admission_stamp();
    let admitted = inner2.check_admission(stamp).unwrap();
    assert!(SupervisorInner::enable_torque(admitted).is_err());
    assert_eq!(inner2.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);

    // Advance past Feetech's 20 ms poisoned-line quiet window after enable readback timeout,
    // refresh fresh feedback + owner_driver_snapshot via one FailedHold tick, then prove
    // zero-allocation stopped-hold service passes (no Vec alloc for positions or SYNC_WRITE pkt).
    clock.advance(Duration::from_millis(25));
    inner2.tick(0.02);
    bus.state.lock().unwrap().sync_write_count = 0;
    for _ in 0..3 {
        let (ok, a, f) = counted(|| inner2.service_stopped_measured_hold_complete(0.02, false));
        assert!(ok, "R1: Feetech FailedHold service pass must complete hold TX");
        assert_eq!(
            (a, f),
            (0, 0),
            "R1: Supervisor Feetech FailedHold service_stopped_measured_hold must allocate 0"
        );
    }
    assert_eq!(bus.state.lock().unwrap().sync_write_count, 3);
}

/// Rule R1 (part 3): MIT stopped hold uses no feedback-age `Vec`, no `mit_gravity` `Vec`,
/// no `mit_joints` `Vec`, and no `last_ff.clone()` — strictly 0 heap allocations and 0 frees
/// both in `service_stopped_measured_hold` AND across full stopped RT releases (`fx.step()`).
#[test]
fn r1_mit_stopped_hold_zero_alloc() {
    use armd::mit::MitOwner;
    use armd::tracking::{JointTrackingConfig, JointTrackingJoint, TrackingStrategy};

    let clock = Arc::new(MockClock::new());
    let bus = ZeroAllocBus::new(clock.clone());
    bus.with(|s| {
        for j in 0..6 {
            s.ctrl_modes[j] = damiao::CTRL_MODE_MIT;
            s.motors[j].q = 0.25;
            s.motors[j].enabled = true;
        }
        s.ctrl_modes[6] = damiao::CTRL_MODE_POS_VEL;
        s.motors[6].q = 0.0;
        s.motors[6].enabled = true;
        s.push_all_feedback();
    });

    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.joint_tracking = Some(JointTrackingConfig {
        enabled: true,
        joints: (0..6)
            .map(|_| JointTrackingJoint {
                strategy: TrackingStrategy::Mit,
                kp: 4.0,
                kd: 0.15,
                hold_kp: Some(4.0),
                hold_kd: Some(0.15),
                gravity_scale: 1.0,
                effort_limit_nm: 3.0,
                ..Default::default()
            })
            .collect(),
    });
    let limits: Vec<DamiaoLimits> = cfg.damiao_ranges.as_ref().unwrap().iter().map(|r| r.limits()).collect();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    drv.read_joint_states().unwrap();
    // Verify CTRL_MODE readback and switch J1..J6 to verified MIT mode (J7 stays POS_VEL).
    drv.check_mapping_ranges();
    let hold_targets: Vec<armd::driver::JointTarget> = (0..7)
        .map(|j| {
            if j < 6 {
                armd::driver::JointTarget::mit(0.25, 0.0, 25.0, 1.5, 0.0)
            } else {
                armd::driver::JointTarget::position(0.0)
            }
        })
        .collect();
    let modes = [
        armd::driver::JointMode::Mit,
        armd::driver::JointMode::Mit,
        armd::driver::JointMode::Mit,
        armd::driver::JointMode::Mit,
        armd::driver::JointMode::Mit,
        armd::driver::JointMode::Mit,
        armd::driver::JointMode::Position,
    ];
    drv.set_joint_modes_confirmed(&modes, &hold_targets).unwrap();
    for j in 0..7 {
        assert!(drv.joint_hold_mode_verified(j), "J{} must have verified hold mode", j + 1);
    }

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.safety.set_off_thread_reclaim(true);
    inner.is_torqued = true;
    inner.mit.owner = MitOwner::Active;
    inner.mit.joints = vec![0, 1, 2, 3, 4, 5];
    inner.mit.last_ff = vec![0.0; 7];
    inner.mit.last_output = Some(clock.now());
    inner.safety.latch(
        armd::safety::FaultKind::Estop,
        armd::safety::FaultDetail::Estop,
    );

    let (mut fx, _b, _tw, _stop) = ReleaseFixture::new(
        inner,
        BOOT,
        Capacities { commands: 8, plans: 1, replies: 16 },
    );
    // Warm up observation triple-buffer slots before counting allocations.
    for _ in 0..4 {
        bus.with(|s| s.push_all_feedback());
        fx.step();
        clock.advance(RELEASE);
    }
    assert_eq!(fx.core.mit.owner, MitOwner::Active, "MIT owner must remain Active during stopped hold");

    // 1. Direct `service_stopped_measured_hold` after a Step 2 poll with MIT active: strictly (0, 0).
    bus.with(|s| {
        s.mit_tx_count = 0;
        s.push_all_feedback();
    });
    let step2_budget = armd::rt_core::ops::ServiceBudget::for_release(fx.now(), fx.now() + RELEASE);
    let _ = fx.core.poll_hardware(&step2_budget);
    let (ok, a, f) = counted(|| fx.core.service_stopped_measured_hold_complete(0.02, false));
    assert!(ok, "R1: MIT stopped hold pass must complete hold TX");
    assert_eq!(
        (a, f),
        (0, 0),
        "R1: MIT service_stopped_measured_hold must perform 0 allocs and 0 frees (no age Vec, no last_ff clone)"
    );
    assert!(
        bus.with(|s| s.mit_tx_count) >= 6,
        "R1: MIT stopped hold must transmit MIT hold frames for all 6 owned joints"
    );
    clock.advance(RELEASE);

    // 2. Full RT stopped releases (`fx.step()`) with MIT active: strictly (0, 0) every release!
    bus.with(|s| s.mit_tx_count = 0);
    for tick in 0..5 {
        bus.with(|s| s.push_all_feedback());
        let (_, a, f) = counted(|| fx.step());
        clock.advance(RELEASE);
        assert_eq!(
            (a, f),
            (0, 0),
            "R1: full RT stopped MIT hold release #{tick} must allocate 0 and free 0"
        );
    }
    assert_eq!(fx.core.mit.owner, MitOwner::Active);
    assert!(
        bus.with(|s| s.mit_tx_count) >= 30,
        "R1: 5 MIT stopped hold releases must transmit >= 30 MIT hold frames"
    );
}

/// L3: The stopped-hold queue and mask are owner state; TX uses non-blocking `try_lock` (skips
/// the tick and increments the diagnostics counter on contention, never blocks, 0 alloc), and an
/// empty mask takes NO lock at all (0 contention, 0 alloc).
#[test]
fn l3_stopped_hold_try_lock_contention_and_empty_mask_zero_alloc() {
    use armd::diagnostics::tick::{
        reset_stopped_hold_contention_escalations, reset_stopped_hold_lock_contended,
        stopped_hold_contention_escalations_count, stopped_hold_lock_contended_count,
    };
    use std::time::Instant;

    let (mut fx, _b, bus, clock) = build_armed_estopped_fixture(0.30);
    // Step 2 poll populates owner_driver_snapshot with all 7 joints fresh & verified -> non-empty mask.
    bus.with(|s| s.push_all_feedback());
    let step2_budget = armd::rt_core::ops::ServiceBudget::for_release(fx.now(), fx.now() + RELEASE);
    let _ = fx.core.poll_hardware(&step2_budget);
    assert!(fx.core.service_stopped_measured_hold_complete(0.02, false));
    assert_ne!(fx.core.owner_hold_mask(), 0);
    assert_eq!(fx.core.owner_pending_hold_mask(), 0);

    // Hold the driver mutex on another OS thread throughout the contention + escalation + empty-mask checks.
    reset_stopped_hold_lock_contended();
    reset_stopped_hold_contention_escalations();
    fx.core.with_contended_driver_for_test(|core| {
        // 1. Non-empty hold mask while driver mutex is contended across all N=25 consecutive skips:
        //    MUST NOT block, skips send (`false`), increments `stopped_hold_lock_contended_count`,
        //    escalates on tick N=25 with a counted warning + non-recoverable HardwareFault, and
        //    allocates (0, 0) on EVERY tick including the escalation tick!
        for skip in 1..=armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
            let t0 = Instant::now();
            let (sent, a1, f1) = counted(|| core.service_stopped_measured_hold_complete(0.02, false));
            let elapsed1 = t0.elapsed();
            assert!(!sent, "L3: contended driver mutex must skip stopped-hold TX (skip {skip})");
            assert!(
                elapsed1 < Duration::from_millis(20),
                "L3: contended try_lock must not block on skip {skip}, took {elapsed1:?}"
            );
            assert_eq!(
                stopped_hold_lock_contended_count(),
                u64::from(skip),
                "L3: contended try_lock must increment diagnostics counter on skip {skip}"
            );
            assert_eq!(
                (a1, f1),
                (0, 0),
                "L3: contended stopped-hold skip #{skip} (incl. N=25 escalation) must allocate 0 and free 0"
            );
            if skip < armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
                assert_eq!(stopped_hold_contention_escalations_count(), 0);
            } else {
                assert_eq!(stopped_hold_contention_escalations_count(), 1);
                assert!(core.safety.has_latched_fault_code(armd::FaultCode::HardwareFault));
            }
        }

        // 2. Empty hold mask (no eligible joints, no pending holds) while driver mutex is STILL
        //    held: MUST take NO lock at all -> counter stays unchanged at N=25 and allocates (0, 0).
        core.invalidate_owner_driver_snapshot_for_test();
        let t1 = Instant::now();
        let (sent_empty, a2, f2) =
            counted(|| core.service_stopped_measured_hold_complete(0.02, false));
        let elapsed2 = t1.elapsed();
        assert!(!sent_empty);
        assert_eq!(core.owner_hold_mask(), 0);
        assert!(
            elapsed2 < Duration::from_millis(20),
            "L3: empty mask must return immediately without touching lock, took {elapsed2:?}"
        );
        assert_eq!(
            stopped_hold_lock_contended_count(),
            u64::from(armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS),
            "L3: empty mask MUST NOT call try_lock or touch driver mutex"
        );
        assert_eq!(
            (a2, f2),
            (0, 0),
            "L3: empty-mask stopped-hold pass must allocate 0 and free 0"
        );
    });

    // 3. Once uncontended again, refreshing Step 2 resumes normal zero-alloc stopped-hold TX.
    clock.advance(RELEASE);
    bus.with(|s| s.push_all_feedback());
    let step2_budget2 = armd::rt_core::ops::ServiceBudget::for_release(fx.now(), fx.now() + RELEASE);
    let _ = fx.core.poll_hardware(&step2_budget2);
    let (sent_ok, a3, f3) = counted(|| fx.core.service_stopped_measured_hold_complete(0.02, false));
    assert!(sent_ok, "L3: uncontended stopped-hold TX must succeed");
    assert_eq!((a3, f3), (0, 0));
    assert_eq!(
        stopped_hold_lock_contended_count(),
        u64::from(armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS)
    );
}

/// M4: `armd-control` RT off-rest refusal (operational & stopped) allocates strictly (0, 0)
/// without cloning positions or allocating the expected-pose Vec, and RT disable dispatch &
/// polling use non-blocking `try_lock` with counted escalation at N=25 and (0, 0) allocs.
#[test]
fn r11_m4_rt_off_rest_refusal_and_disable_try_lock_escalation_zero_alloc() {
    use armd::diagnostics::tick::{
        disable_contention_escalations_count, disable_lock_contended_count,
        reset_disable_contention_escalations, reset_disable_lock_contended,
    };
    use armd::rt_core::ids::ArmId;
    use std::time::Instant;

    // 1. Operational off-rest TorqueOff refusal (`ARM_NOT_AT_RESTING_POSE`): (0, 0) allocs/frees!
    let (mut fx, mut b, bus, clock) = build_armed_fixture(0.35);
    // Warm up triple-buffer slots so steady-state publish buffers are settled.
    for _ in 0..5 {
        bus.with(|s| s.push_all_feedback());
        fx.step();
        clock.advance(RELEASE);
    }
    assert!(fx.core.is_torqued);
    assert!(fx.core.safety.state().is_operational());

    let auth = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(
        ClientId(1),
        2,
        auth,
        fx.now() + Duration::from_secs(5),
        Submit::Transition(TransitionOp::TorqueOff),
    )
    .unwrap();
    bus.with(|s| s.push_all_feedback());
    let (rep_refuse, a_ref, f_ref) = counted(|| fx.step());
    assert_eq!(
        rep_refuse.command.map(|(_, r)| r),
        Some(Err("ARM_NOT_AT_RESTING_POSE")),
        "M4: operational off-rest TorqueOff must refuse with ARM_NOT_AT_RESTING_POSE"
    );
    assert_eq!(
        (a_ref, f_ref),
        (0, 0),
        "M4: operational off-rest TorqueOff refusal must allocate 0 and free 0"
    );

    // 2. Contended RT disable DISPATCH: try_lock returns DRIVER_BUSY without blocking,
    //    escalates at N=25 consecutive skips, and allocates (0, 0) on every tick including escalation!
    reset_disable_lock_contended();
    reset_disable_contention_escalations();
    fx.core.with_contended_driver_for_test(|core| {
        let now = clock.now();
        let cctx = armd::rt_core::ops::CommitCtx {
            arm: ArmId(1),
            boot: BOOT,
            now,
            stop_generation: 0,
        };
        let ep = core.safety.epoch_id();
        for skip in 1..=armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
            let desc = armd::rt_core::broker::Descriptor {
                stamp: armd::rt_core::broker::Stamp {
                    arm: ArmId(1),
                    boot: BOOT,
                    request: armd::rt_core::broker::RequestId(100 + u64::from(skip)),
                    client: ClientId(1),
                    client_seq: 100 + u64::from(skip),
                    authority: Authority {
                        lease_generation: 0,
                        epoch: ep,
                        stop_generation: 0,
                        config_revision: 0,
                        parent_generation: 0,
                    },
                    deadline: now + Duration::from_secs(5),
                },
                kind: armd::rt_core::broker::CommandKind::Transition(TransitionOp::TorqueOff),
            };
            let t0 = Instant::now();
            let (res, a1, f1) =
                counted(|| core.commit_transition(&desc, TransitionOp::TorqueOff, &cctx));
            assert!(
                t0.elapsed() < Duration::from_millis(20),
                "M4: dispatch must never block"
            );
            assert_eq!(res, Err("DRIVER_BUSY"), "M4: contended dispatch skip #{skip}");
            assert_eq!(
                (a1, f1),
                (0, 0),
                "M4: contended dispatch skip #{skip} must allocate (0, 0)"
            );
            assert_eq!(disable_lock_contended_count(), u64::from(skip));
            if skip < armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
                assert_eq!(disable_contention_escalations_count(), 0);
            } else {
                assert_eq!(disable_contention_escalations_count(), 1);
                assert!(core.safety.has_latched_fault_code(FaultCode::HardwareFault));
            }
        }
    });

    // 3. Contended RT disable POLLING: start a real TorqueOff at rest (Pending in repeat window),
    //    then hold driver mutex across N=25 poll ticks -> never blocks, stays Pending, escalates at N=25, (0, 0) allocs!
    let (mut fx_poll, mut b_poll, bus_poll, clock_poll) = build_armed_estopped_fixture(0.0);
    // Clear 20 ms post-motion settle and accumulate K=3 fresh rest samples so TorqueOff enters Pending Disable repeats.
    clock_poll.advance(Duration::from_millis(25));
    for _ in 0..3 {
        bus_poll.with(|s| s.push_all_feedback());
        let _ = fx_poll.step();
        clock_poll.advance(RELEASE);
    }
    let a_p = fx_poll.core.authority(&fx_poll.ctx(fx_poll.now()));
    b_poll
        .submit(
            ClientId(1),
            10,
            a_p,
            fx_poll.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();
    bus_poll.with(|s| s.push_all_feedback());
    let rep_start = fx_poll.step();
    assert!(matches!(rep_start.command, Some((_, Ok(_)))));
    assert!(
        fx_poll.core.torque_transition_pending(),
        "precondition: disable repeats must be pending"
    );

    reset_disable_lock_contended();
    reset_disable_contention_escalations();
    fx_poll.core.with_contended_driver_for_test(|core| {
        let budget = armd::rt_core::ops::ServiceBudget::for_release(
            clock_poll.now(),
            clock_poll.now() + RELEASE,
        );
        for skip in 1..=armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
            let t0 = Instant::now();
            let (poll, a2, f2) = counted(|| core.poll_hardware(&budget));
            assert!(
                t0.elapsed() < Duration::from_millis(20),
                "M4: disable poll must never block"
            );
            assert!(
                matches!(poll.torque, Some(Ok(armd::rt_core::ops::TorqueProgress::Pending { .. }))),
                "M4: contended disable poll skip #{skip} must keep transition Pending, got {:?}",
                poll.torque
            );
            assert_eq!(
                (a2, f2),
                (0, 0),
                "M4: contended disable poll skip #{skip} must allocate (0, 0)"
            );
            assert_eq!(disable_lock_contended_count(), u64::from(skip));
            if skip < armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS {
                assert_eq!(disable_contention_escalations_count(), 0);
            } else {
                assert_eq!(disable_contention_escalations_count(), 1);
                assert!(core.safety.has_latched_fault_code(FaultCode::HardwareFault));
            }
        }
    });
}

// =========================================================================
// R12 M3: Zero-allocation Feetech RT disable path + RX budget & deadline proof
// =========================================================================

struct ZeroAllocFeetechState {
    clock: Arc<MockClock>,
    rx: [u8; 4096],
    rx_head: usize,
    rx_tail: usize,
    tx: [u8; 4096],
    tx_len: usize,
    read_calls: u64,
    // Fixed-size staged replies (zero heap alloc during `write_all`):
    stage_req: [[u8; 16]; 10],
    stage_req_len: [usize; 10],
    stage_rep: [[u8; 64]; 10],
    stage_rep_len: [usize; 10],
    stage_used: [bool; 10],
}

impl ZeroAllocFeetechState {
    fn new(clock: Arc<MockClock>) -> Self {
        Self {
            clock,
            rx: [0u8; 4096],
            rx_head: 0,
            rx_tail: 0,
            tx: [0u8; 4096],
            tx_len: 0,
            read_calls: 0,
            stage_req: [[0u8; 16]; 10],
            stage_req_len: [0; 10],
            stage_rep: [[0u8; 64]; 10],
            stage_rep_len: [0; 10],
            stage_used: [false; 10],
        }
    }

    fn clear_tx(&mut self) {
        self.tx_len = 0;
    }

    fn clear_rx(&mut self) {
        self.rx_head = 0;
        self.rx_tail = 0;
    }

    fn push_rx(&mut self, bytes: &[u8]) {
        assert!(self.rx_tail + bytes.len() <= self.rx.len());
        self.rx[self.rx_tail..self.rx_tail + bytes.len()].copy_from_slice(bytes);
        self.rx_tail += bytes.len();
    }

    fn stage_reply(&mut self, slot: usize, req: &[u8], rep: &[u8]) {
        self.stage_req[slot][..req.len()].copy_from_slice(req);
        self.stage_req_len[slot] = req.len();
        self.stage_rep[slot][..rep.len()].copy_from_slice(rep);
        self.stage_rep_len[slot] = rep.len();
        self.stage_used[slot] = true;
    }

    fn clear_staged(&mut self) {
        self.stage_used = [false; 10];
    }

    fn count_torque_off_writes(&self) -> usize {
        let mut count = 0;
        let data = &self.tx[..self.tx_len];
        let mut i = 0;
        while i + 8 <= data.len() {
            if data[i] == 0xFF
                && data[i + 1] == 0xFF
                && data[i + 3] == 0x04
                && data[i + 4] == 0x03
                && data[i + 5] == armd::protocol::feetech::REG_TORQUE_ENABLE
                && data[i + 6] == 0x00
            {
                count += 1;
                i += 8;
            } else {
                i += 1;
            }
        }
        count
    }
}

#[derive(Clone)]
struct ZeroAllocFeetechTransport(Arc<Mutex<ZeroAllocFeetechState>>);

impl ZeroAllocFeetechTransport {
    fn new(clock: Arc<MockClock>) -> Self {
        Self(Arc::new(Mutex::new(ZeroAllocFeetechState::new(clock))))
    }
    fn with<R>(&self, f: impl FnOnce(&mut ZeroAllocFeetechState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl armd::transport::Transport for ZeroAllocFeetechTransport {
    fn write(&mut self, data: &[u8]) -> Result<usize, TransportError> {
        self.write_all(data)?;
        Ok(data.len())
    }

    fn write_all(&mut self, data: &[u8]) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        assert!(s.tx_len + data.len() <= s.tx.len());
        let start = s.tx_len;
        s.tx[start..start + data.len()].copy_from_slice(data);
        s.tx_len += data.len();
        for slot in 0..10 {
            if s.stage_used[slot] && s.stage_req_len[slot] == data.len() && &s.stage_req[slot][..data.len()] == data {
                let rlen = s.stage_rep_len[slot];
                let mut rep_copy = [0u8; 64];
                rep_copy[..rlen].copy_from_slice(&s.stage_rep[slot][..rlen]);
                s.stage_used[slot] = false;
                s.push_rx(&rep_copy[..rlen]);
                break;
            }
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        let mut s = self.0.lock().unwrap();
        s.read_calls += 1;
        let avail = s.rx_tail.saturating_sub(s.rx_head);
        if avail == 0 {
            s.clock.advance(Duration::from_millis(1));
            return Err(TransportError::WouldBlock);
        }
        let n = avail.min(buf.len());
        buf[..n].copy_from_slice(&s.rx[s.rx_head..s.rx_head + n]);
        s.rx_head += n;
        if s.rx_head == s.rx_tail {
            s.rx_head = 0;
            s.rx_tail = 0;
        }
        Ok(n)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn clear_input_buffer(&mut self) {
        self.0.lock().unwrap().clear_rx();
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone() as Arc<dyn DriverClock>)
    }
}

fn build_feetech_pos_replies(ids: &[u8], steps: &[u16], out: &mut [u8]) -> usize {
    let mut off = 0;
    for (&id, &pos) in ids.iter().zip(steps.iter()) {
        let [p_lo, p_hi] = pos.to_le_bytes();
        let params = [p_lo, p_hi, 0, 0, 0, 0, 120, 30];
        let cs = armd::protocol::feetech::calculate_checksum(id, 10, 0, &params);
        out[off] = 0xFF;
        out[off + 1] = 0xFF;
        out[off + 2] = id;
        out[off + 3] = 10;
        out[off + 4] = 0;
        out[off + 5..off + 13].copy_from_slice(&params);
        out[off + 13] = cs;
        off += 14;
    }
    off
}

#[test]
fn r12_m3_feetech_rt_disable_path_zero_alloc_and_honours_rx_budget_and_deadline() {
    use armd::driver::{
        ArmDriverError, DisableKind, DisableProtectiveReason, FeetechArmDriver, TorqueOutcome,
        TorqueProgress,
    };
    use armd::protocol::feetech;

    let clock = Arc::new(MockClock::new());
    let transport = ZeroAllocFeetechTransport::new(clock.clone());
    let servo_ids: [u8; 6] = [1, 2, 3, 4, 5, 6];
    let rest_steps: [u16; 6] = [2048; 6];
    let raised_steps: [u16; 6] = [2400, 2048, 2048, 2048, 2048, 2048];
    let mut drv = FeetechArmDriver::new(transport.clone(), servo_ids.to_vec())
        .with_calibrated(true)
        .with_tx_echo(Some(false));
    drv.set_stopped_disable_gate(StoppedDisableGate {
        safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        has_resting_pose: true,
        arm_joints: {
            let mut a = [0usize; armd::rt_core::mailbox::MAX_DOF];
            for i in 0..5 {
                a[i] = i;
            }
            a
        },
        arm_joints_len: 5,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.15,
    });

    let mut req1 = [0u8; 16];
    let req1_len = feetech::write_sync_read_slice(&servo_ids[..3], feetech::REG_PRESENT_POSITION_L, 8, &mut req1).unwrap();
    let mut req2 = [0u8; 16];
    let req2_len = feetech::write_sync_read_slice(&servo_ids[3..], feetech::REG_PRESENT_POSITION_L, 8, &mut req2).unwrap();
    let mut rep_rest1 = [0u8; 64];
    let rep_rest1_len = build_feetech_pos_replies(&servo_ids[..3], &rest_steps[..3], &mut rep_rest1);
    let mut rep_rest2 = [0u8; 64];
    let rep_rest2_len = build_feetech_pos_replies(&servo_ids[3..], &rest_steps[3..], &mut rep_rest2);
    let mut rep_raised1 = [0u8; 64];
    let rep_raised1_len = build_feetech_pos_replies(&servo_ids[..3], &raised_steps[..3], &mut rep_raised1);

    // 1. Unsupported budgeted entry points & idle poll_torque return static error codes with (0, 0) allocs.
    let mut states_buf = Vec::with_capacity(6);
    let dl0 = clock.now() + Duration::from_millis(8);
    let (r_rx, a, f) = counted(|| drv.read_joint_states_budgeted(&mut states_buf, dl0, 6));
    assert!(matches!(r_rx, Err(ArmDriverError::ProtocolStatic(_))));
    assert_eq!((a, f), (0, 0), "M3: read_joint_states_budgeted unsupported must allocate (0, 0)");

    let (r_bt, a, f) = counted(|| drv.begin_torque_budgeted(true, 6));
    assert!(matches!(r_bt, Err(ArmDriverError::ProtocolStatic(_))));
    assert_eq!((a, f), (0, 0), "M3: begin_torque_budgeted unsupported must allocate (0, 0)");

    let (r_pb, a, f) = counted(|| drv.poll_torque_budgeted(dl0, 6));
    assert!(matches!(r_pb, Err(ArmDriverError::ProtocolStatic(_))));
    assert_eq!((a, f), (0, 0), "M3: idle poll_torque_budgeted must allocate (0, 0)");

    let (r_pt, a, f) = counted(|| drv.poll_torque(dl0));
    assert!(matches!(r_pt, Err(ArmDriverError::ProtocolStatic(_))));
    assert_eq!((a, f), (0, 0), "M3: idle poll_torque must allocate (0, 0)");

    // 2. Off-rest refusal: begin_disable_torque_budgeted(DisableRequested, 0) refuses with (0, 0) allocs & 0 bus I/O.
    transport.with(|s| {
        s.stage_reply(0, &req1[..req1_len], &rep_raised1[..rep_raised1_len]);
        s.stage_reply(1, &req2[..req2_len], &rep_rest2[..rep_rest2_len]);
    });
    drv.read_joint_states().unwrap();
    transport.with(|s| {
        s.clear_tx();
        s.read_calls = 0;
    });
    let started_raised = clock.now();
    let (r_refuse, a, f) = counted(|| {
        drv.begin_disable_torque_budgeted(
            DisableKind::DisableRequested { started_at: started_raised },
            0,
        )
    });
    assert!(matches!(r_refuse, Err(ArmDriverError::TorqueOffWouldDrop)));
    assert_eq!((a, f), (0, 0), "M3: off-rest refusal must allocate (0, 0)");
    assert_eq!(transport.with(|s| (s.tx_len, s.read_calls)), (0, 0));

    // 3. Move to rest pose (no K=3 window yet): begin_disable_torque_budgeted(kind, 0) defers with (0, 0) allocs.
    clock.advance(Duration::from_millis(120)); // age out raised sample so gate sees stale/missing -> defer
    transport.with(|s| {
        s.clear_tx();
        s.clear_staged();
        s.read_calls = 0;
    });
    let started_rest = clock.now();
    let (r_defer, a, f) = counted(|| {
        drv.begin_disable_torque_budgeted(
            DisableKind::DisableRequested { started_at: started_rest },
            0,
        )
    });
    assert!(matches!(r_defer, Ok(TorqueProgress::Pending { .. })));
    assert_eq!((a, f), (0, 0), "M3: deferred begin_disable_torque_budgeted must allocate (0, 0)");
    assert_eq!(transport.with(|s| (s.tx_len, s.read_calls)), (0, 0));

    // 4. Expired deadline (deadline <= now): poll_torque_budgeted performs 0 bus TX/RX with (0, 0) allocs.
    transport.with(|s| {
        s.clear_tx();
        s.read_calls = 0;
        s.stage_reply(0, &req1[..req1_len], &rep_rest1[..rep_rest1_len]);
        s.stage_reply(1, &req2[..req2_len], &rep_rest2[..rep_rest2_len]);
    });
    let expired_dl = clock.now();
    let (r_exp_dl, a, f) = counted(|| drv.poll_torque_budgeted(expired_dl, 6));
    assert!(matches!(r_exp_dl, Ok(TorqueProgress::Pending { .. })));
    assert_eq!(drv.take_last_poll_rx(), Some((0, false)));
    assert_eq!((a, f), (0, 0), "M3: expired-deadline poll must allocate (0, 0)");
    assert_eq!(
        transport.with(|s| (s.tx_len, s.read_calls)),
        (0, 0),
        "M3: expired deadline must perform 0 bus TX/RX"
    );

    // 5. Zero RX budget (max_rx_frames == 0): poll_torque_budgeted performs 0 bus TX/RX with (0, 0) allocs.
    let valid_dl = clock.now() + Duration::from_millis(8);
    let (r_zero_rx, a, f) = counted(|| drv.poll_torque_budgeted(valid_dl, 0));
    assert!(matches!(r_zero_rx, Ok(TorqueProgress::Pending { .. })));
    assert_eq!(drv.take_last_poll_rx(), Some((0, true)));
    assert_eq!((a, f), (0, 0), "M3: max_rx_frames=0 poll must allocate (0, 0)");
    assert_eq!(
        transport.with(|s| (s.tx_len, s.read_calls)),
        (0, 0),
        "M3: max_rx_frames=0 must perform 0 bus TX/RX"
    );

    // 5b. Tight mid-poll deadline (2 ms < CHUNK_TIMEOUT 4 ms < MAX_READ_DURATION 8 ms) with no
    //     replies staged: poll_torque_budgeted MUST stop at the caller deadline, not 4/8 ms, and allocate (0, 0).
    transport.with(|s| {
        s.clear_tx();
        s.clear_rx();
        s.clear_staged();
    });
    let t_before_tight = clock.now();
    let tight_dl = t_before_tight + Duration::from_millis(2);
    let (r_tight, a, f) = counted(|| drv.poll_torque_budgeted(tight_dl, 6));
    assert!(matches!(r_tight, Ok(TorqueProgress::Pending { .. })));
    assert_eq!((a, f), (0, 0), "M3: tight-deadline poll must allocate (0, 0)");
    let elapsed_tight = clock.now().saturating_duration_since(t_before_tight);
    assert!(
        elapsed_tight <= Duration::from_millis(3),
        "M3: poll_torque_budgeted must honour caller 2 ms deadline (elapsed {elapsed_tight:?} > 3 ms)"
    );
    // Clear poisoned line from the intentional silent-bus timeout (20 ms idle).
    clock.advance(Duration::from_millis(25));

    // 6. Partial RX budget (max_rx_frames == 2 on 6-servo arm): decodes at most 2 frames, marks
    //    budget_exhausted = true, does NOT poll chunk 2, and allocates (0, 0)!
    transport.with(|s| {
        s.clear_tx();
        s.clear_rx();
        s.clear_staged();
        s.stage_reply(0, &req1[..req1_len], &rep_rest1[..rep_rest1_len]);
        s.stage_reply(1, &req2[..req2_len], &rep_rest2[..rep_rest2_len]);
    });
    let (r_cap2, a, f) = counted(|| drv.poll_torque_budgeted(clock.now() + Duration::from_millis(8), 2));
    assert!(matches!(r_cap2, Ok(TorqueProgress::Pending { .. })));
    assert_eq!(
        drv.take_last_poll_rx(),
        Some((2, true)),
        "M3: poll_torque_budgeted(max_rx_frames=2) must stop at 2 decoded frames with exhausted=true"
    );
    assert_eq!((a, f), (0, 0), "M3: partial RX budget poll must allocate (0, 0)");
    assert!(
        transport.with(|s| s.stage_used[0] ^ s.stage_used[1]),
        "M3: exactly one chunk may be requested before exhausting max_rx_frames=2; second chunk must NOT be requested"
    );

    // 7. Complete K=3 rest confirmation & disable dispatch across spaced polls with max_rx_frames=6:
    //    every single poll (including post-disable REG_TORQUE_ENABLE readback!) allocates (0, 0),
    //    stays <= 6 RX frames, and transmits all 6 TORQUE_ENABLE=0 writes!
    let mut done = false;
    for poll_idx in 0..5 {
        clock.advance(Duration::from_millis(15));
        transport.with(|s| {
            s.clear_tx();
            s.clear_rx();
            s.clear_staged();
            s.stage_reply(0, &req1[..req1_len], &rep_rest1[..rep_rest1_len]);
            s.stage_reply(1, &req2[..req2_len], &rep_rest2[..rep_rest2_len]);
            for (idx, &id) in servo_ids.iter().enumerate() {
                let rd = feetech::build_read_array(id, feetech::REG_TORQUE_ENABLE, 1);
                let cs = feetech::calculate_checksum(id, 3, 0, &[0]);
                let rep = [0xFF, 0xFF, id, 3, 0, 0, cs];
                s.stage_reply(2 + idx, &rd, &rep);
            }
        });
        let dl = clock.now() + Duration::from_millis(8);
        let (res, a, f) = counted(|| drv.poll_torque_budgeted(dl, 6));
        assert_eq!((a, f), (0, 0), "M3: poll #{poll_idx} must allocate (0, 0)");
        let (rx_used, _exhausted) = drv.take_last_poll_rx().unwrap();
        assert!(rx_used <= 6, "M3: poll #{poll_idx} exceeded max_rx_frames=6 (got {rx_used})");
        match res.unwrap() {
            TorqueProgress::Done(TorqueOutcome::Confirmed) => {
                done = true;
                break;
            }
            TorqueProgress::Done(TorqueOutcome::Unconfirmed) => {
                panic!("M3: unexpected TorqueOutcome::Unconfirmed");
            }
            TorqueProgress::Pending { .. } => {}
        }
    }
    assert!(done, "M3: deferred Feetech RT disable must complete on K=3");
    assert_eq!(
        transport.with(|s| s.count_torque_off_writes()),
        6,
        "M3: all 6 servos must receive TORQUE_ENABLE=0"
    );

    // 8. M1 + M3 interaction: when K=3 is ALREADY confirmed and a queued non-rest position reply
    //    sits in transport RX, begin_disable_torque_budgeted(DisableRequested, 0) honours
    //    max_rx_frames=0 (0 RX reads, 0 TX writes, (0, 0) allocs) AND DEFERS instead of discarding
    //    the queued reply; the following poll_torque_budgeted(dl, 6) drains+decodes it within
    //    budget and refuses with TorqueOffWouldDrop (0 torque-off writes, (0, 0) allocs)!
    transport.with(|s| {
        s.clear_tx();
        s.clear_rx();
        s.push_rx(&rep_raised1[..rep_raised1_len]);
        s.read_calls = 0;
    });
    let started_already_confirmed = clock.now();
    let (r_imm, a, f) = counted(|| {
        drv.begin_disable_torque_budgeted(
            DisableKind::DisableRequested { started_at: started_already_confirmed },
            0,
        )
    });
    assert!(
        matches!(r_imm, Ok(TorqueProgress::Pending { .. })),
        "M1+M3: begin_disable_torque_budgeted(0) must defer rather than discard queued RX or violate rx_budget=0"
    );
    assert_eq!((a, f), (0, 0), "M3: begin_disable_torque_budgeted(0) must allocate (0, 0)");
    assert_eq!(
        transport.with(|s| (s.tx_len, s.read_calls)),
        (0, 0),
        "M3: begin_disable_torque_budgeted(0) must perform 0 bus TX/RX"
    );

    let dl_check = clock.now() + Duration::from_millis(8);
    let (r_refuse_queued, a, f) = counted(|| drv.poll_torque_budgeted(dl_check, 6));
    assert!(
        matches!(r_refuse_queued, Err(ArmDriverError::TorqueOffWouldDrop)),
        "M1+M3: budgeted poll must decode the queued raised reply before clearing and refuse, got {r_refuse_queued:?}"
    );
    assert_eq!((a, f), (0, 0), "M3: queued-reply refusal on poll_torque_budgeted must allocate (0, 0)");
    assert_eq!(
        transport.with(|s| s.count_torque_off_writes()),
        0,
        "M1+M3: 0 torque-off writes after queued non-rest reply"
    );

    // 9. DisableProtective with max_rx_frames=0: dispatches all 6 writes with 0 RX reads and (0, 0) allocs.
    transport.with(|s| {
        s.clear_tx();
        s.clear_rx();
        s.read_calls = 0;
    });
    let (r_prot, a, f) = counted(|| {
        drv.begin_disable_torque_budgeted(
            DisableKind::DisableProtective { reason: DisableProtectiveReason::Thermal },
            0,
        )
    });
    assert!(matches!(r_prot, Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))));
    assert_eq!((a, f), (0, 0), "M3: DisableProtective begin_disable_torque_budgeted(0) must allocate (0, 0)");
    assert_eq!(
        transport.with(|s| (s.count_torque_off_writes(), s.read_calls)),
        (6, 0),
        "M3: DisableProtective(0) must write all 6 disable frames with 0 RX reads"
    );
}
