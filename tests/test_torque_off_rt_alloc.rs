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
    limits: DamiaoLimits,
}

impl Motor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = u8::from(self.enabled);
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
    rx: VecDeque<CanFrame>,
    clock: Arc<MockClock>,
    tx_would_block: bool,
    silent_disable: bool,
}

impl BusState {
    fn push_all_feedback(&mut self) {
        for m in &self.motors {
            self.rx.push_back(m.feedback_frame());
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
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(BusState {
            motors,
            rx: VecDeque::with_capacity(256),
            clock,
            tx_would_block: false,
            silent_disable: false,
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
        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(f) = (!silent_disable)
                .then(|| s.motors.iter().find(|m| m.id == id).map(Motor::feedback_frame))
                .flatten()
            {
                s.rx.push_back(f);
            }
            return Ok(());
        }
        if let Some(m) = s.motors.iter_mut().find(|m| u32::from(m.id) == frame.id) {
            if frame.data == damiao::CMD_ENABLE {
                m.enabled = true;
                let f = m.feedback_frame();
                s.rx.push_back(f);
            } else if frame.data == damiao::CMD_DISABLE && !silent_disable {
                m.enabled = false;
                let f = m.feedback_frame();
                s.rx.push_back(f);
            }
            return Ok(());
        }
        if let Some(m) = s.motors.iter().find(|m| u32::from(m.id) + 0x100 == frame.id) {
            let f = m.feedback_frame();
            s.rx.push_back(f);
        }
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
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
    assert_eq!(
        (a1 + a2, f1 + f2),
        (0, 0),
        "TX failure in disable_step_failed while already stopped must allocate 0"
    );
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
