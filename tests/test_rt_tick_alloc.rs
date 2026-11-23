//! The steady-state control tick neither allocates nor frees on the control thread: a
//! counting allocator records every allocation and deallocation of the calling thread
//! (armd, std and crate internals) during whole ticks of the supervisor (feedback, control,
//! output, observation build and publication, recording sample).
//!
//! Scenarios: torqued idle hold, a joint quintic move, a servo stream (a new target every
//! tick), a gripper close, and a recording; driverless (mock follower) and with an
//! allocation-free driver double. "Steady state" excludes the tick that starts or ends a
//! motion (a transition may allocate once; the design's step 3 removes that too): each
//! scenario warms up, then counts ticks in the middle of the motion.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

use std::collections::VecDeque;

use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, CanTransport, TransportError};
use armd::model::JointState;
use armd::motion::{GripperCommand, GripperRequest};
use armd::{ArmConfig, SupervisorInner};

thread_local! {
    /// The scenarios run with the device kit's profile.
    static DEVICE: Cell<bool> = const { Cell::new(false) };
    /// The driver is the Damiao driver on [`FakeDamiaoBus`] (else [`EchoDriver`]).
    static DAMIAO: Cell<bool> = const { Cell::new(false) };
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn count(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) { let _ = c.try_with(|n| n.set(n.get() + 1)); }
}

// SAFETY: forwards to System; the counters are const-initialized thread-locals without
// destructors and do not allocate.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { count(&ALLOCS); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { count(&ALLOCS); unsafe { System.alloc_zeroed(l) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { count(&FREES); unsafe { System.dealloc(p, l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 { count(&ALLOCS); count(&FREES); unsafe { System.realloc(p, l, n) } }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// (allocations, deallocations) of this thread inside `f`.
fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

/// A driver whose feedback follows the last target, without allocating on the hot calls.
struct EchoDriver {
    dof: usize,
    q: Vec<f32>,
    v: Vec<f32>,
    enabled: bool,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
}

impl EchoDriver {
    fn new(dof: usize) -> Self {
        Self { dof, q: vec![0.0; dof], v: vec![0.0; dof], enabled: false, rx_at: vec![None; dof], seqs: vec![0; dof] }
    }
}

impl ArmDriver for EchoDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true, torque_feedforward: false, enable_feedback: true, load_feedback: false }
    }
    fn enable_state(&self) -> EnableState { if self.enabled { EnableState::Enabled } else { EnableState::Disabled } }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.rx_at.iter().map(|t| t.map(|t| t.elapsed())).collect() }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
    fn feedback_seqs(&self) -> Vec<u64> { self.seqs.clone() }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.dof);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = Instant::now();
        out.clear();
        for i in 0..self.dof {
            out.push(JointState { position: self.q[i], velocity: self.v[i], torque: 0.0, temperature: 30.0 });
            self.rx_at[i] = Some(now);
            self.seqs[i] += 1;
        }
        Ok(())
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.extend(self.rx_at.iter().map(|t| t.map(|t| t.elapsed())));
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.rx_at.len() == dof && self.rx_at.iter().all(|t| t.is_some_and(|t| t.elapsed() <= max_age))
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.extend(self.rx_at.iter().map(|t| t.map(|t| t.elapsed().as_millis() as u64)));
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.rx_at); }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); out.extend_from_slice(&self.seqs); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.rx_at); }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(self.dof) {
            self.q[i] = t.position;
            self.v[i] = t.velocity.unwrap_or(0.0);
        }
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }
}

const DEVICE_PROFILE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/kits/recamera-b601dm/profile/rebot-b601dm.json");

/// The built-in B601-DM config, or the device kit's profile (workspace with link capsules and
/// self-collision pairs, power budget, torque guard).
/// Damiao motors on a bus that answers every command frame with a feedback frame at once
/// (the reply queue is preallocated: the bus itself does not allocate per frame).
struct FakeDamiaoBus {
    motors: Vec<(u16, DamiaoLimits, f32, bool)>,
    rx: VecDeque<CanFrame>,
}

impl FakeDamiaoBus {
    fn new(limits: &[DamiaoLimits]) -> Self {
        Self { motors: limits.iter().enumerate().map(|(i, l)| (i as u16 + 1, *l, 0.0, false)).collect(), rx: VecDeque::with_capacity(1024) }
    }

    fn reply(&mut self, k: usize) {
        let (id, l, q, enabled) = self.motors[k];
        let p = damiao::float_to_uint(q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let data = [(u8::from(enabled) << 4) | (id as u8 & 0x0f), (p >> 8) as u8, p as u8, (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8, t as u8, 30, 30];
        if self.rx.len() < self.rx.capacity() { self.rx.push_back(CanFrame::new_standard(0x10 + id, &data).unwrap()); }
    }
}

impl CanTransport for FakeDamiaoBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let id = frame.id as u16;
        let find = |m: &[(u16, DamiaoLimits, f32, bool)], id: u16| m.iter().position(|x| x.0 == id);
        if id == 0x7FF {
            let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(k) = find(&self.motors, mid) { if frame.data[2] == 0xCC { self.reply(k); } }
        } else if let Some(k) = find(&self.motors, id) {
            if frame.data == damiao::CMD_ENABLE { self.motors[k].3 = true; }
            if frame.data == damiao::CMD_DISABLE { self.motors[k].3 = false; }
            self.reply(k);
        } else if let Some(k) = find(&self.motors, id.wrapping_sub(0x100)).or_else(|| find(&self.motors, id.wrapping_sub(0x300))) {
            self.motors[k].2 = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.reply(k);
        }
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> { self.rx.pop_front().ok_or(TransportError::WouldBlock) }
    fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
}

fn config(device: bool) -> ArmConfig {
    let base = if device {
        let mut c: ArmConfig = serde_json::from_reader(std::fs::File::open(DEVICE_PROFILE).unwrap()).unwrap();
        c.validate().unwrap();
        c
    } else {
        ArmConfig::rebot_dm()
    };
    // no lease watchdog in these single-threaded scenarios
    ArmConfig { lease_timeout_ms: 600_000, ..base }
}

/// One control-loop tick (tick, observation filled into the publication slot, publish) as
/// `ArmSupervisor::control_tick` runs it under the write guard, counted; the displaced
/// observation (none in steady state) is freed outside the count.
fn tick_counted(inner: &mut SupervisorInner) -> (u64, u64) {
    let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    // a reader between ticks (its copy is the reader's allocation, not the tick's)
    let _ = inner.core.hub().latest();
    (a, f)
}

fn warm(inner: &mut SupervisorInner, n: usize) {
    for _ in 0..n { let _ = inner.tick_publish(0.02, 1, false); let _ = inner.core.hub().latest(); }
}

fn supervisor(with_driver: bool) -> SupervisorInner {
    let cfg = config(DEVICE.with(Cell::get));
    let dof = cfg.dof;
    let mut inner = SupervisorInner::new(cfg);
    if with_driver {
        inner = if DAMIAO.with(Cell::get) {
            let limits: Vec<DamiaoLimits> = inner.config.damiao_ranges.as_ref().unwrap().iter().map(|r| r.limits()).collect();
            let bus = FakeDamiaoBus::new(&limits);
            let driver = DamiaoArmDriver::new(bus, (1..=dof as u16).collect()).with_joint_limits(limits).unwrap();
            inner.with_driver(Box::new(driver))
        } else {
            inner.with_driver(Box::new(EchoDriver::new(dof)))
        };
    }
    inner.lease_enabled = false;
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    warm(&mut inner, 5);
    inner
}

/// Allocation and deallocation counts of `n` ticks, each counted on its own.
fn count_ticks(inner: &mut SupervisorInner, n: usize, mut before: impl FnMut(&mut SupervisorInner, usize)) -> Vec<(u64, u64)> {
    (0..n).map(|i| {
        before(inner, i);
        tick_counted(inner)
    }).collect()
}

fn assert_zero(label: &str, counts: &[(u64, u64)]) {
    let bad: Vec<_> = counts.iter().enumerate().filter(|(_, c)| **c != (0, 0)).collect();
    assert!(bad.is_empty(), "{label}: ticks with (allocations, deallocations): {bad:?}");
}

fn scenarios(with_driver: bool) {
    let tag = match (with_driver, DEVICE.with(Cell::get), DAMIAO.with(Cell::get)) {
        (false, false, _) => "driverless", (true, false, _) => "driver",
        (false, true, _) => "device profile driverless", (true, true, false) => "device profile driver",
        (true, true, true) => "device profile Damiao driver",
    };

    // torqued idle hold
    let mut inner = supervisor(with_driver);
    assert_zero(&format!("{tag} idle"), &count_ticks(&mut inner, 50, |_, _| {}));

    // joint quintic move: 2 s; count ticks 10..60 of it
    let mut inner = supervisor(with_driver);
    let mut q = inner.commanded_positions.clone();
    q[0] += 0.4;
    q[2] -= 0.3;
    let epoch = inner.safety.epoch_id();
    inner.jog(q, Some(2.0), Some(epoch)).expect("jog");
    warm(&mut inner, 10);
    assert!(inner.active_trajectory.is_some());
    assert_zero(&format!("{tag} quintic"), &count_ticks(&mut inner, 50, |_, _| {}));
    assert!(inner.active_trajectory.is_some(), "still moving: the counted ticks were mid-motion");

    // servo stream: a new target every tick (the target vector is built outside the tick)
    let mut inner = supervisor(with_driver);
    let base = inner.commanded_positions.clone();
    let epoch = inner.safety.epoch_id();
    for i in 0..10 {
        let mut t = base.clone();
        t[0] += 0.002 * i as f32;
        inner.servo(t, epoch).unwrap();
        warm(&mut inner, 1);
    }
    let targets: Vec<Vec<f32>> = (10..60).map(|i| { let mut t = base.clone(); t[0] += 0.002 * i as f32; t }).collect();
    let mut targets = targets.into_iter();
    let mut servo_counts = Vec::new();
    for _ in 0..50 {
        inner.servo(targets.next().unwrap(), epoch).unwrap();
        servo_counts.push(tick_counted(&mut inner));
    }
    assert_zero(&format!("{tag} servo stream"), &servo_counts);

    // gripper close (from fully open, slowly)
    let mut inner = supervisor(with_driver);
    let epoch = inner.safety.epoch_id();
    inner.start_gripper(GripperRequest { command: GripperCommand::Open, fraction: None, grasp: false, speed: 2.0, timeout_s: None, tag: None }, epoch)
        .expect("gripper open");
    for _ in 0..200 { if inner.active_trajectory.is_none() { break; } warm(&mut inner, 1); }
    assert!(inner.active_trajectory.is_none(), "gripper open finished");
    inner.start_gripper(GripperRequest { command: GripperCommand::Close, fraction: None, grasp: false, speed: 0.3, timeout_s: None, tag: None }, epoch)
        .expect("gripper close");
    warm(&mut inner, 5);
    assert!(inner.active_trajectory.is_some());
    // counted while the jaw moves (the tick that ends the action is a transition)
    let mut counts = Vec::new();
    while inner.active_trajectory.is_some() && counts.len() < 200 {
        let before = inner.last_action.as_ref().map(|a| a.state.clone());
        let c = tick_counted(&mut inner);
        if inner.active_trajectory.is_some() && inner.last_action.as_ref().map(|a| a.state.clone()) == before { counts.push(c); }
    }
    assert!(counts.len() >= 10, "gripper close: {} ticks counted", counts.len());
    assert_zero(&format!("{tag} gripper close"), &counts);

    // recording (teach session ring) during a quintic move
    let mut inner = supervisor(with_driver);
    let dir = tempfile::tempdir().unwrap();
    inner.recorder.start(Some("ep".into()), Some(dir.path().join("ep.parquet"))).unwrap();
    inner.core.begin_recording();
    let mut q = inner.commanded_positions.clone();
    q[1] += 0.3;
    let epoch = inner.safety.epoch_id();
    inner.jog(q, Some(2.0), Some(epoch)).expect("jog");
    warm(&mut inner, 10);
    let counts = count_ticks(&mut inner, 50, |inner, i| {
        // the dispatcher drains the ring off the control thread
        if i % 20 == 0 { inner.core.record.ring().unwrap().drain(); }
    });
    assert_zero(&format!("{tag} recording"), &counts);
    assert!(inner.core.record.pushed() >= 60);
}

#[test]
fn steady_state_ticks_do_not_touch_the_allocator_driverless() {
    scenarios(false);
}

#[test]
fn steady_state_ticks_do_not_touch_the_allocator_with_a_driver() {
    scenarios(true);
}

#[test]
fn steady_state_ticks_do_not_touch_the_allocator_device_profile() {
    DEVICE.with(|d| d.set(true));
    scenarios(false);
    scenarios(true);
}

/// The device's driver path: the Damiao driver (behind the joint-zero boundary) on a CAN bus.
#[test]
fn steady_state_ticks_do_not_touch_the_allocator_damiao_driver() {
    DEVICE.with(|d| d.set(true));
    DAMIAO.with(|d| d.set(true));
    scenarios(true);
}
