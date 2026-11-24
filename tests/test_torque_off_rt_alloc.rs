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

use armd::driver::{ArmDriver, DamiaoArmDriver};
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
    rx: VecDeque<(CanFrame, Option<std::time::Instant>)>,
    clock: Arc<MockClock>,
    tx_would_block: bool,
    silent_disable: bool,
    drop_enable_mask: u8,
}

impl BusState {
    fn push_all_feedback(&mut self) {
        for m in &self.motors {
            self.rx.push_back((m.feedback_frame(), None));
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
            rx: VecDeque::with_capacity(256),
            clock,
            tx_would_block: false,
            silent_disable: false,
            drop_enable_mask: 0,
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
    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(drv));
    inner.lease_enabled = false;
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

    let (rep2, a2, f2) = counted(|| fx.step());
    clock.advance(RELEASE);
    assert!(matches!(rep2.transition_ended, Some((_, Outcome::Failed(_)))), "{rep2:?}");
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
    assert_eq!(ended, Some(Outcome::Failed("FEEDBACK_UNCONFIRMED")));
    assert_eq!(*fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(fx.core.safety.epoch_id(), epoch_before);
    assert_eq!(fx.core.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(fx.core.safety.fault(), fault_before);
    assert_eq!(
        (total_allocs, total_frees),
        (0, 0),
        "confirmation timeout while already stopped must allocate 0"
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
        bus.with(|s| {
            s.motors[1].q = q_j2;
            s.push_all_feedback();
        });
        drv.read_joint_states().unwrap();
        drv.set_torque(false).unwrap();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
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
