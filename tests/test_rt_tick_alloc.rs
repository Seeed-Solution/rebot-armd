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
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
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
    clock: std::sync::Arc<MockClock>,
}

impl FakeDamiaoBus {
    fn new(limits: &[DamiaoLimits]) -> Self {
        Self {
            motors: limits.iter().enumerate().map(|(i, l)| (i as u16 + 1, *l, 0.0, false)).collect(),
            rx: VecDeque::with_capacity(1024),
            clock: std::sync::Arc::new(MockClock::new()),
        }
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
    fn clock(&self) -> Option<std::sync::Arc<dyn DriverClock>> { Some(self.clock.clone()) }
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
        let before = inner.last_action.as_ref().map(|a| a.state);
        let c = tick_counted(&mut inner);
        if inner.active_trajectory.is_some() && inner.last_action.as_ref().map(|a| a.state) == before { counts.push(c); }
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
        if i % 20 == 0 { inner.core.record_ring().unwrap().drain(); }
    });
    assert_zero(&format!("{tag} recording"), &counts);
    assert!(inner.core.record_pushed() >= 60);
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

/// Stalled readers and dispatcher: no reader refreshes the hub and the recording ring's
/// consumer is held (a preempted dispatcher) for longer than the retire ring and the
/// recording ring hold. The tick still neither allocates nor frees: it fills the slot's own
/// observation in place (readers copy it, never take it), so nothing is displaced, and a
/// full recording ring drops and counts samples. Holding both mutexes on this thread also
/// shows the tick never takes them.
#[test]
fn ticks_do_not_touch_the_allocator_with_stalled_readers_and_dispatcher() {
    DEVICE.with(|d| d.set(true));
    DAMIAO.with(|d| d.set(true));
    let mut inner = supervisor(true);
    let dir = tempfile::tempdir().unwrap();
    inner.recorder.start(Some("ep".into()), Some(dir.path().join("ep.parquet"))).unwrap();
    inner.core.begin_recording();
    let mut q = inner.commanded_positions.clone();
    q[1] += 0.3;
    let epoch = inner.safety.epoch_id();
    inner.jog(q, Some(8.0), Some(epoch)).expect("jog");
    warm(&mut inner, 10);
    let hub = inner.core.hub().clone();
    let ring = inner.core.record_ring().unwrap().clone();
    let (_reader, _dispatcher) = (hub.reader_guard(), ring.consumer_guard());
    let n = armd::rt_core::record_ring::RECORD_RING_CAPACITY + 2 * armd::rt_core::publish::RETIRE_CAPACITY + 20;
    let counts: Vec<(u64, u64)> = (0..n).map(|_| {
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        (a, f)
    }).collect();
    assert_zero("stalled readers and dispatcher", &counts);
    assert!(inner.active_trajectory.is_some(), "still moving");
    assert_eq!(hub.tick_frees(), 0);
    assert!(inner.core.record_dropped() > 0, "the recording ring overflowed");
}

/// Lease transitions in the observation: the holder label is set while a lease is valid
/// and cleared when it expires; neither the tick that publishes the new holder nor the one
/// that clears it allocates or frees (the label is stored inline).
#[test]
fn lease_holder_set_and_clear_ticks_do_not_touch_the_allocator() {
    DEVICE.with(|d| d.set(true));
    DAMIAO.with(|d| d.set(true));
    let mut inner = supervisor(true);
    // no holder -> holder
    let generation = inner.acquire_lease("conn-1").unwrap();
    inner.set_lease_label("conn-1", generation, "a-fairly-long-client-label-that-would-need-a-heap-buffer");
    let set = tick_counted(&mut inner);
    assert_eq!(inner.core.hub().latest().unwrap().obs.lease_holder.as_deref(), Some("a-fairly-long-client-label-that-would-need-a-heap-buffer"));
    // holder -> none (expiry; the stored lease stays until the watchdog runs)
    inner.active_lease.as_mut().unwrap().acquired_at -= inner.lease_timeout;
    let cleared = tick_counted(&mut inner);
    let after = count_ticks(&mut inner, 5, |_, _| {});
    assert_eq!(inner.core.hub().latest().unwrap().obs.lease_holder, None);
    assert_zero("lease holder set / clear", &[set, cleared]);
    assert_zero("after the lease cleared", &after);
}

/// The retire ring full: publications that displaced observations (not the tick: the
/// displacing API) filled it while no reader ran; the tick still neither allocates nor
/// frees, because it publishes its slot's own observation back in place.
#[test]
fn ticks_with_a_full_retire_ring_do_not_touch_the_allocator() {
    DEVICE.with(|d| d.set(true));
    DAMIAO.with(|d| d.set(true));
    let mut inner = supervisor(true);
    let hub = inner.core.hub().clone();
    let _reader = hub.reader_guard();
    let template = inner.current_observation();
    let q = inner.commanded_positions.clone();
    let mut handed_back = 0;
    for i in 0..armd::rt_core::publish::RETIRE_CAPACITY + 3 {
        let now = Instant::now();
        match inner.core.publish_displacing(100 + i as u64, now, now, template.clone(), &q, &q) {
            Ok(Some(back)) => { handed_back += 1; drop(back); }
            Ok(None) => {}
            Err(_) => panic!("no buffer is out"),
        }
    }
    assert!(handed_back > 0, "the retire ring did not fill");
    let counts = count_ticks_stalled(&mut inner, 2 * armd::rt_core::publish::RETIRE_CAPACITY);
    assert_zero("full retire ring", &counts);
    // the displacing API is refused while the tick's buffer is out
    let buf = inner.core.take_observation_buffer();
    let now = Instant::now();
    assert!(inner.core.publish_displacing(1, now, now, template.clone(), &q, &q).is_err());
    inner.core.publish(buf, 1, now, now, &q, &q);
}

/// Like [`count_ticks`] without the reader between ticks (the hub reader is held).
fn count_ticks_stalled(inner: &mut SupervisorInner, n: usize) -> Vec<(u64, u64)> {
    (0..n).map(|_| { let (p, a, f) = counted(|| inner.tick_publish(0.02, 1, false)); drop(p); (a, f) }).collect()
}

/// The observation's inline labels: a policy run's stop reason and the gravity-compensation
/// phase change (set, replaced, cleared) without allocating or freeing.
#[cfg(feature = "policy-runtime")]
#[test]
fn observation_stop_reason_and_phase_transitions_do_not_touch_the_allocator() {
    use armd::model::{GravityCompObservation, HolderLabel, PolicyObservation};
    use armd::policy_runtime::{PolicyMode, PolicyRunStart};
    let mut inner = supervisor(false);
    let gen = inner.acquire_lease("alloc").expect("lease");
    let epoch = inner.safety.epoch_id();
    inner.begin_policy_run(PolicyRunStart {
        epoch_id: epoch, connection_id: "alloc".into(), lease_generation: gen, policy_id: "p".into(), artifact_digest: "d".into(),
        backend: "mlp".into(), mode: PolicyMode::Shadow, sim_only: false, max_first_step_rad: None, max_step_rad: None, tag: None,
        i_am_at_the_arm: false, verified_release_gate: None,
    }).expect("run");
    let mut run = inner.policy_run.take().expect("run");
    let mut out = PolicyObservation::default();
    run.to_observation_into(true, &mut out);
    let reasons = [Some("MAX_SECONDS".to_string()), Some("LEASE_LOST".to_string()), None, Some("VISION_STALE".to_string()), None];
    for r in reasons {
        run.stop_reason = r.clone();
        let ((), a, f) = counted(|| run.to_observation_into(false, &mut out));
        assert_eq!((a, f), (0, 0), "stop reason {r:?}");
        assert_eq!(out.stop_reason.as_deref(), r.as_deref());
    }
    // the phase: set, replaced, cleared (the gravity-comp observation without a run clears it)
    let mut g = GravityCompObservation::default();
    inner.gravity_comp_observation_into(&mut g);
    for phase in [Some("ramp_in"), Some("active"), Some("exiting"), None] {
        let ((), a, f) = counted(|| g.phase = phase.map(HolderLabel::new));
        assert_eq!((a, f), (0, 0), "phase {phase:?}");
    }
    g.phase = Some(HolderLabel::new("active"));
    let ((), a, f) = counted(|| inner.gravity_comp_observation_into(&mut g));
    assert_eq!((a, f, g.phase), (0, 0, None), "clearing the phase on the tick");
}

/// Plan-slot memory: the 8 default slots are write-touched (pre-faulted) at construction,
/// reserve exactly 655,360 payload bytes, fill to full capacity without touching the
/// allocator, and refuse a 9th concurrent stage (`PlansFull` / `None`).
#[test]
fn plan_slots_are_pre_faulted_and_reserve_655360_payload_bytes() {
    use armd::rt_core::broker::{plan_slots, PLAN_SLOTS};
    use armd::rt_core::mailbox::MAX_DOF;
    use armd::rt_core::ops::{
        PlanSlot, ALL_PLAN_SLOTS_PAYLOAD_BYTES, MAX_PATH_SAMPLES, PLAN_SLOT_PAYLOAD_BYTES,
    };

    assert_eq!(PLAN_SLOTS, 8);
    assert_eq!(MAX_PATH_SAMPLES, 2048);
    assert_eq!(MAX_DOF, 8);
    assert_eq!(PLAN_SLOT_PAYLOAD_BYTES, 81_920);
    assert_eq!(ALL_PLAN_SLOTS_PAYLOAD_BYTES, 655_360);

    let sample = [0.25f32; MAX_DOF];
    let mut handles = [None; PLAN_SLOTS];
    let mut total_payload_bytes = 0usize;

    // Warm up the staging/pin/retire/reclaim code pages on a 1-slot scratch pool so first-call
    // `.text` / stack page faults are settled before we construct and measure the 8-slot pool.
    {
        let (mut w0, mut r0) = plan_slots(1, || PlanSlot::with_path_capacity(1));
        let ((), a0, f0) = counted(|| {
            let h = w0.stage(|s| { s.path.clear(); s.path.push(&sample, 0.0).unwrap(); }).unwrap();
            let p = r0.pin(h).unwrap();
            assert_eq!(p.path.len(), 1);
            assert_eq!(p.path.capacity(), 1);
            assert!(p.payload_bytes() > 0);
            assert_eq!(p.path.samples()[0][0], 0.25);
            drop(p);
            assert_eq!(w0.free(), 0);
            assert!(w0.stage(|_| {}).is_none());
            assert!(r0.retire(h));
            assert_eq!(w0.reclaim(), 1);
        });
        assert_eq!((a0, f0), (0, 0));
    }

    let (mut writer, mut reader) = plan_slots(PLAN_SLOTS, PlanSlot::default);
    assert_eq!((writer.capacity(), reader.capacity()), (PLAN_SLOTS, PLAN_SLOTS));

    #[cfg(target_os = "linux")]
    let thread_faults = || -> (i64, i64) {
        let mut ru: libc::rusage = unsafe { std::mem::zeroed() };
        let rc = unsafe { libc::getrusage(libc::RUSAGE_THREAD, &mut ru) };
        assert_eq!(rc, 0, "getrusage(RUSAGE_THREAD) failed");
        (ru.ru_minflt, ru.ru_majflt)
    };
    #[cfg(target_os = "linux")]
    let (min_before, maj_before) = {
        let _ = thread_faults();
        thread_faults()
    };

    // Filling all 8 pre-faulted slots to MAX_PATH_SAMPLES, pinning, reading, retiring and
    // reclaiming must perform zero allocations, zero frees, and zero page faults.
    let (exhausted_none, allocs, frees) = counted(|| {
        for (idx, slot_h) in handles.iter_mut().enumerate() {
            let h = writer
                .stage(|slot| {
                    slot.path.clear();
                    for k in 0..MAX_PATH_SAMPLES {
                        slot.path.push(&sample, k as f64 / (MAX_PATH_SAMPLES - 1) as f64).unwrap();
                    }
                })
                .expect("slot available");
            let pinned = reader.pin(h).expect("slot live");
            assert_eq!(pinned.path.len(), MAX_PATH_SAMPLES);
            assert_eq!(pinned.path.capacity(), MAX_PATH_SAMPLES);
            assert_eq!(pinned.payload_bytes(), PLAN_SLOT_PAYLOAD_BYTES);
            assert_eq!(pinned.path.samples()[0][0], 0.25);
            assert_eq!(pinned.path.samples()[MAX_PATH_SAMPLES - 1][MAX_DOF - 1], 0.25);
            total_payload_bytes += pinned.payload_bytes();
            drop(pinned);
            *slot_h = Some(h);
            assert_eq!(writer.free(), PLAN_SLOTS - 1 - idx);
        }
        let exhausted = writer.stage(|_| {}).is_none();
        for h in handles.iter().flatten().copied() {
            assert!(reader.retire(h));
        }
        assert_eq!(writer.reclaim(), PLAN_SLOTS);
        exhausted
    });

    #[cfg(target_os = "linux")]
    {
        let (min_after, maj_after) = thread_faults();
        assert_eq!(maj_after - maj_before, 0, "pre-faulted plan slots triggered major page faults");
        assert_eq!(min_after - min_before, 0, "pre-faulted plan slots triggered minor page faults on write");
    }

    assert!(exhausted_none, "9th concurrent stage must fail when all 8 slots are held");
    assert_eq!(total_payload_bytes, 655_360);
    assert_eq!((allocs, frees), (0, 0), "staging/pinning/retiring 8 full plan slots touched the allocator");
}

fn watchdog_park_supervisor() -> SupervisorInner {
    use armd::model::{JointLimit, RestingPosePolicy};

    let mut cfg = ArmConfig::default();
    cfg.dof = 2;
    cfg.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];
    cfg.home_positions = vec![0.0, 0.0];
    cfg.lease_timeout_ms = 1_000;
    cfg.resting_pose = Some(RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.08; 2],
        max_total_deviation: 0.16,
        max_velocity: 0.015,
        joint_max_velocities: vec![],
        support_torque_limits: vec![(0, 1.0)],
        support_load_limits: vec![],
        feedback_max_age_ms: 100,
    });
    let mut inner = SupervisorInner::new(cfg);
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    let _gen = inner.acquire_lease("client").expect("lease");
    let epoch = inner.safety.epoch_id();
    inner.jog(vec![0.5, 0.2], Some(0.2), Some(epoch)).expect("jog");
    for _ in 0..40 {
        inner.active_lease.as_mut().unwrap().acquired_at = Instant::now();
        inner.last_heartbeat = Instant::now();
        inner.tick(0.02);
    }
    // Expire lease so the watchdog starts a park.
    let past = Duration::from_millis(1_100);
    inner.active_lease.as_mut().unwrap().acquired_at = Instant::now() - past;
    inner.last_heartbeat = Instant::now() - past;
    inner.tick(0.02);
    assert!(inner.watchdog_park.is_some() && inner.pending_normal_unload);
    inner
}

static STOP_DIAG_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// `authorized_stop` during a watchdog park records fields into the preallocated diagnostic
/// slot without tracing, allocating or freeing; an off-thread caller drains and logs it.
#[test]
fn authorized_stop_during_watchdog_park_does_not_allocate_or_free() {
    use armd::rt_core::ops::stop_diag;
    let _guard = STOP_DIAG_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let mut inner = watchdog_park_supervisor();

    let mut seen = stop_diag().count();
    let (stopped, allocs, frees) = counted(|| inner.authorized_stop());
    assert!(!stopped, "watchdog park continues");
    assert_eq!((allocs, frees), (0, 0), "authorized_stop during watchdog park touched allocator");

    let rec = stop_diag().drain_and_log(&mut seen).expect("diagnostic recorded");
    assert_eq!(rec.arm, inner.core.id());
    assert!(rec.count >= 1);
    assert!(rec.park_budget_us > 0);
    assert!(stop_diag().drain_and_log(&mut seen).is_none(), "drained once");

    // Normal authorized_stop when idle / servo active also performs zero allocations and frees.
    let mut idle = supervisor(false);
    let q = idle.commanded_positions.clone();
    let ep = idle.safety.epoch_id();
    idle.servo(q, ep).expect("servo");
    let (stopped_idle, a2, f2) = counted(|| idle.authorized_stop());
    assert!(stopped_idle);
    assert_eq!((a2, f2), (0, 0), "idle/servo authorized_stop touched allocator");
}

/// Normal-priority periodic publication refresh (`StateHub::latest` / `try_latest`, called by
/// the supervisor dispatcher loop at 50 Hz) drains deferred RT stop diagnostics off the RT
/// thread and logs each recorded watchdog-park stop event exactly once.
#[test]
fn watchdog_park_stop_diagnostic_is_drained_and_logged_once_off_rt_thread() {
    use std::io::Write;
    use std::sync::{Arc, Mutex};
    let _guard = STOP_DIAG_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    #[derive(Clone, Default)]
    struct BufWriter(Arc<Mutex<Vec<u8>>>);
    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for BufWriter {
        type Writer = BufGuard;
        fn make_writer(&'a self) -> Self::Writer { BufGuard(self.0.clone()) }
    }
    struct BufGuard(Arc<Mutex<Vec<u8>>>);
    impl Write for BufGuard {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }

    let mut inner = watchdog_park_supervisor();

    // Clear any prior un-drained count before capturing logs for this test.
    let _ = inner.core.hub().latest();

    let buf = BufWriter::default();
    let sub = tracing_subscriber::fmt()
        .with_writer(buf.clone())
        .with_ansi(false)
        .with_max_level(tracing::Level::INFO)
        .finish();
    let captured = || String::from_utf8(buf.0.lock().unwrap().clone()).unwrap();

    tracing::subscriber::with_default(sub, || {
        // 1. RT authorized_stop records into the lock-free slot without logging or allocating.
        let (stopped, a, f) = counted(|| inner.authorized_stop());
        assert!(!stopped);
        assert_eq!((a, f), (0, 0), "authorized_stop during watchdog park must not touch allocator");
        assert_eq!(captured().matches("Stop during a watchdog park").count(), 0, "must not log on RT call");

        // 2. RT tick publication does NOT drain/log the stop diagnostic.
        let _ = inner.tick_publish(0.02, 1, false);
        assert_eq!(captured().matches("Stop during a watchdog park").count(), 0, "must not log on RT tick");

        // 3. Normal-priority StateHub::latest() (dispatcher loop path) drains and logs once.
        let _ = inner.core.hub().latest();
        let text = captured();
        assert_eq!(text.matches("Stop during a watchdog park").count(), 1, "expected 1 log line: {text}");
        assert!(text.contains("the park continues"), "missing park-continues message: {text}");

        // 4. Subsequent normal-priority polls do not re-log the same event.
        let _ = inner.core.hub().latest();
        let _ = inner.core.hub().try_latest();
        assert_eq!(captured().matches("Stop during a watchdog park").count(), 1, "must log record only once");

        // 5. A second watchdog-park stop event is drained once more.
        assert!(!inner.authorized_stop());
        let _ = inner.core.hub().latest();
        let _ = inner.core.hub().latest();
        assert_eq!(captured().matches("Stop during a watchdog park").count(), 2, "second event logged once");
    });
}

/// Commit-time RT boundaries for path, replay, jog, and config reservation:
/// - Refused commits (stale boot, zero boot, overtaken stop, stale plan, config busy) across
///   path, replay, jog and config perform zero allocations and zero frees.
/// - In-place path/replay target activation (`start_path_motion_slice`, `start_replay_motion`)
///   and slot-borrowed `ReplayMotion` sampling + steady-state stepping perform zero allocations
///   and zero frees.
#[test]
fn commit_boundaries_for_path_replay_jog_and_config_do_not_allocate_or_free() {
    use armd::motion::Reference;
    use armd::rt_core::broker::{
        Authority, BootId, Capacities, ClientId, CommandKind, Descriptor, RequestId, Stamp, Submit,
    };
    use armd::rt_core::config_iface::{ConfigAuthority, ConfigDomain, Reserved};
    use armd::rt_core::ids::ArmId;
    use armd::rt_core::mailbox::MAX_DOF;
    use armd::rt_core::ops::{
        tool_digest_hash, CommitCtx, CoreOps, EpisodeRef, PathKind, PlanSlot, Primitive, ReleaseFixture,
    };
    use armd::rt_core::urgent::StopRequests;

    let boot = BootId::fresh();
    let stale_boot = BootId::fresh();
    assert_ne!(boot, stale_boot);

    let (mut fx, mut b, _tw, stop) = ReleaseFixture::new(supervisor(false), boot, Capacities::default());
    let now = fx.now();
    let deadline = now + Duration::from_secs(30);
    let auth = fx.core.authority(&fx.ctx(now));
    let dof = fx.core.commanded_positions.len();
    let mut start = [0.0f32; MAX_DOF];
    start[..dof].copy_from_slice(&fx.core.commanded_positions);
    let mut goal = start;
    goal[0] += 0.05;

    // 1. Pre-stage a Cartesian path slot and a Replay slot (off-RT).
    let tool = tool_digest_hash(&fx.core.config.active_tool.config_digest);
    let mut cart_slot = PlanSlot::default();
    for k in 0..=10 {
        let f = k as f32 / 10.0;
        let mut q = start;
        q[0] += 0.05 * f;
        cart_slot.path.push(&q[..dof], f64::from(f)).unwrap();
    }
    cart_slot.primitive = Primitive::Prepared {
        child: None,
        dof: dof as u8,
        start,
        kind: PathKind::Cartesian,
        duration_s: 0.5,
        power_scale: None,
    };

    let mut replay_slot = PlanSlot::default();
    for k in 0..=20 {
        let f = k as f32 / 20.0;
        let mut q = start;
        q[0] += 0.05 * f;
        replay_slot.path.push(&q[..dof], k as f64).unwrap();
    }
    replay_slot.primitive = Primitive::Prepared {
        child: None,
        dof: dof as u8,
        start,
        kind: PathKind::Replay {
            episode: EpisodeRef::new("alloc_ep").unwrap(),
            speed: 1.0,
            gripper: false,
            sample_dt: 0.02,
            tool_digest: tool,
        },
        duration_s: 0.4,
        power_scale: None,
    };

    let jog_prim = Primitive::JointQuintic { dof: dof as u8, start, q: goal, duration_s: 0.5 };

    // 2. Direct commit checks for stale boot id, zero boot id, overtaken stop, stale start,
    //    invalid prepared path, and mismatched replay tool digest across jog, path, and replay
    //    must perform (0, 0) allocs/frees.
    let mut bad_path_slot = PlanSlot::default();
    bad_path_slot.path.push(&start[..dof], 0.0).unwrap();
    bad_path_slot.primitive = cart_slot.primitive;

    let mut bad_tool_replay = replay_slot.clone();
    bad_tool_replay.primitive = Primitive::Prepared {
        child: None,
        dof: dof as u8,
        start,
        kind: PathKind::Replay {
            episode: EpisodeRef::new("alloc_ep").unwrap(),
            speed: 1.0,
            gripper: false,
            sample_dt: 0.02,
            tool_digest: tool ^ 1,
        },
        duration_s: 0.4,
        power_scale: None,
    };

    let (mut pw, mut pr) = armd::rt_core::broker::plan_slots(4, PlanSlot::default);
    let h_cart = pw.stage(|s| s.copy_from(&cart_slot).unwrap()).unwrap();
    let h_rep = pw.stage(|s| s.copy_from(&replay_slot).unwrap()).unwrap();
    let h_bad_path = pw.stage(|s| s.copy_from(&bad_path_slot).unwrap()).unwrap();
    let h_bad_tool = pw.stage(|s| s.copy_from(&bad_tool_replay).unwrap()).unwrap();

    let ctx_ok = CommitCtx { arm: ArmId::LOCAL, boot, now, stop_generation: 0 };
    let ctx_stopped = CommitCtx { stop_generation: 1, ..ctx_ok };
    let desc = |b_id: BootId, stop_gen: u64, kind: CommandKind| Descriptor {
        stamp: Stamp {
            arm: ArmId::LOCAL,
            boot: b_id,
            request: RequestId(1),
            client: ClientId(1),
            client_seq: 1,
            authority: Authority { stop_generation: stop_gen, ..auth },
            deadline,
        },
        kind,
    };
    let mut stale_start = start;
    stale_start[0] += 0.01;
    let stale_jog = Primitive::JointQuintic { dof: dof as u8, start: stale_start, q: goal, duration_s: 0.5 };

    let (codes, allocs, frees) = counted(|| {
        let d_stale_boot = desc(stale_boot, 0, CommandKind::Install { plan: h_cart });
        let d_zero_boot = desc(BootId(0), 0, CommandKind::Install { plan: h_cart });
        let d_ok = desc(boot, 0, CommandKind::Install { plan: h_cart });
        let d_rep = desc(boot, 0, CommandKind::Install { plan: h_rep });
        let d_bad_path = desc(boot, 0, CommandKind::Install { plan: h_bad_path });
        let d_bad_tool = desc(boot, 0, CommandKind::Install { plan: h_bad_tool });

        let c1 = fx.core.commit_primitive(&d_stale_boot, &jog_prim, &ctx_ok).unwrap_err();
        let c2 = fx.core.commit_primitive(&d_zero_boot, &jog_prim, &ctx_ok).unwrap_err();
        let c3 = fx.core.commit_primitive(&d_ok, &jog_prim, &ctx_stopped).unwrap_err();
        let c4 = fx.core.commit_primitive(&d_ok, &stale_jog, &ctx_ok).unwrap_err();

        let p1 = fx.core.commit_prepared_slot(&d_stale_boot, pr.pin(h_cart).unwrap(), &ctx_ok).unwrap_err();
        let p2 = fx.core.commit_prepared_slot(&d_ok, pr.pin(h_cart).unwrap(), &ctx_stopped).unwrap_err();
        let p3 = fx.core.commit_prepared_slot(&d_bad_path, pr.pin(h_bad_path).unwrap(), &ctx_ok).unwrap_err();

        let r1 = fx.core.commit_prepared_slot(&d_stale_boot, pr.pin(h_rep).unwrap(), &ctx_ok).unwrap_err();
        let r2 = fx.core.commit_prepared_slot(&d_rep, pr.pin(h_rep).unwrap(), &ctx_stopped).unwrap_err();
        let r3 = fx.core.commit_prepared_slot(&d_bad_tool, pr.pin(h_bad_tool).unwrap(), &ctx_ok).unwrap_err();

        [c1, c2, c3, c4, p1, p2, p3, r1, r2, r3]
    });
    assert_eq!(
        codes,
        [
            "WRONG_TARGET", "WRONG_TARGET", "STOPPED", "PLAN_STALE",
            "WRONG_TARGET", "STOPPED", "INVALID_COMMAND",
            "WRONG_TARGET", "STOPPED", "TOOL_MISMATCH",
        ]
    );
    assert_eq!((allocs, frees), (0, 0), "refused commits touched the allocator");
    assert!(pr.retire(h_cart) && pr.retire(h_rep) && pr.retire(h_bad_path) && pr.retire(h_bad_tool));
    assert_eq!(pw.reclaim(), 4, "no leaked plan slot pins after refused commits");

    // 3. In-place path and replay target activation perform (0, 0) allocs/frees.
    let epoch = fx.core.safety.epoch_id();
    let (res, a_act, f_act) = counted(|| {
        let d1 = fx.core.start_path_motion_slice(epoch, &goal[..dof], 0.5).unwrap();
        fx.core.start_replay_motion(&start[..dof]).unwrap();
        d1
    });
    assert_eq!(res, 0.5);
    assert_eq!((a_act, f_act), (0, 0), "in-place path/replay target activation touched the allocator");
    fx.core.stop_motion();

    // 4. Config reserve (refused for stale/zero boot and overtaken stop, and granted+released)
    //    performs (0, 0) allocs/frees.
    let live_stops = StopRequests::new();
    let cfg_ctx = fx.core.config_ctx_with(boot, &live_stops, now);
    let prop = fx.core.config_proposal(ConfigDomain::Keepout, fx.core.revision(ConfigDomain::Keepout), 0, &cfg_ctx);
    let mut bad_boot_prop = prop;
    bad_boot_prop.stamp.boot = stale_boot;
    let mut zero_boot_prop = prop;
    zero_boot_prop.stamp.boot = BootId(0);
    let zero_ctx = CommitCtx { boot: BootId(0), ..cfg_ctx };

    let (cfg_ok, a_cfg, f_cfg) = counted(|| {
        let r_bad = matches!(fx.core.reserve(&bad_boot_prop, &cfg_ctx), Reserved::Refused { code: "WRONG_TARGET", .. });
        let r_zero = matches!(fx.core.reserve(&zero_boot_prop, &zero_ctx), Reserved::Refused { code: "WRONG_TARGET", .. });
        live_stops.request();
        let after_stop = fx.core.config_ctx_with(boot, &live_stops, now);
        let r_stop = matches!(fx.core.reserve(&prop, &after_stop), Reserved::Refused { code: "STOPPED", .. });
        let fresh_prop = fx.core.config_proposal(ConfigDomain::Keepout, fx.core.revision(ConfigDomain::Keepout), 0, &after_stop);
        let granted = match fx.core.reserve(&fresh_prop, &after_stop) {
            Reserved::Granted(r) => {
                fx.core.release(r);
                true
            }
            _ => false,
        };
        r_bad && r_zero && r_stop && granted
    });
    assert!(cfg_ok);
    assert_eq!((a_cfg, f_cfg), (0, 0), "config reserve/release touched the allocator");

    // 5. Accepted prepared replay commit (via `fx.step()`) performs (0, 0) allocs/frees,
    //    borrows the immutable plan slot (no sample vector copy, no `Box`, inline approach),
    //    and runs approach + follow ticks with (0, 0) allocs/frees.
    // Warm publication triple-buffer slots before counting full releases.
    for _ in 0..4 { fx.step(); }
    let _ = fx.core.core.hub().latest();

    let rep_copy = replay_slot.clone();
    b.submit(
        ClientId(1),
        1,
        fx.core.authority(&fx.ctx(fx.now())),
        deadline,
        Submit::Install(&move |s: &mut PlanSlot| s.copy_from(&rep_copy).unwrap()),
    )
    .unwrap();
    let (step_rep, a_rep_commit, f_rep_commit) = counted(|| fx.step());
    assert!(step_rep.command.unwrap().1.is_ok(), "prepared replay committed");
    assert_eq!(
        (a_rep_commit, f_rep_commit),
        (0, 0),
        "accepted prepared replay commit + tick touched the allocator"
    );

    let mut sample_buf = Vec::with_capacity(dof);
    let mut step_out = armd::motion::StepOut {
        positions: Vec::with_capacity(dof),
        velocities: Vec::with_capacity(dof),
        ..Default::default()
    };
    let cmd_q = fx.core.commanded_positions.clone();
    let cmd_v = fx.core.commanded_velocities.clone();
    let meas_q = fx.core.measured_positions.clone();
    let meas_v = fx.core.measured_velocities.clone();
    let torques = fx.core.torques.clone();
    let loads = fx.core.loads.clone();
    let ((borrows_slot, phase_after, t_after), a_samp, f_samp) = counted(|| match fx.core.active_trajectory.as_mut().map(|a| &mut a.reference) {
        Some(Reference::Replay(rm)) => {
            rm.sample_at_into(0.1, &mut sample_buf);
            rm.sample_at_into(0.25, &mut sample_buf);
            let ctx = armd::motion::StepCtx {
                dt: 0.02,
                time_scale: 1.0,
                measured: &meas_q,
                measured_velocities: &meas_v,
                commanded: &cmd_q,
                commanded_velocities: &cmd_v,
                torques: &torques,
                loads: &loads,
                feedback_fresh: true,
                torque_feedback: true,
                load_feedback: false,
                gripper_stalled: false,
                gripper_stall_sign: 0.0,
                gripper_force_capped: false,
            };
            // Step through approach into slot-borrowed follow phase without allocating or freeing.
            for _ in 0..10 {
                rm.step_into(&ctx, &mut step_out);
            }
            (rm.borrows_plan_slot(), rm.phase_name(), rm.t_s())
        }
        _ => (false, "", 0.0),
    });
    assert!(borrows_slot, "prepared replay must borrow directly from the immutable plan slot");
    assert_eq!(phase_after, "replay", "must transition from inline approach to slot-borrowed follow phase");
    assert!(t_after > 0.0, "follow phase must advance recording time");
    assert_eq!((a_samp, f_samp), (0, 0), "sampling/stepping slot-borrowed ReplayMotion touched the allocator");
    assert_eq!(sample_buf.len(), dof);
    assert_eq!(step_out.positions.len(), dof);

    // Steady-state replay ticks through `fx.core.tick_publish` also perform (0, 0) allocs/frees.
    assert_zero("prepared replay steady ticks", &count_ticks(&mut fx.core, 10, |_, _| {}));

    // Overtaken stop ends the replay cleanly and retires its pinned slot.
    stop.request();
    let (stopped_rep, a_stop, f_stop) = counted(|| fx.step());
    assert!(stopped_rep.stopped);
    assert_eq!((a_stop, f_stop), (0, 0), "urgent stop ending slot-borrowed replay touched the allocator");
    assert!(fx.core.active_trajectory.is_none());
    b.poll(|_, _| true);
    assert_eq!(b.stats().retired, 1, "slot reclaimed after replay terminal");
    // Let post-stop deceleration reach rest before preparing the next Cartesian path.
    for _ in 0..20 {
        if fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-3) { break; }
        fx.step();
    }
    let _ = fx.core.core.hub().latest();

    // 6. Accepted prepared Cartesian path commit (via `fx.step()`) performs (0, 0) allocs/frees,
    //    borrows the immutable plan slot, and executes steady-state ticks with (0, 0) allocs/frees.
    let mut cart_now = cart_slot.clone();
    let mut cur_start = [0.0f32; MAX_DOF];
    cur_start[..dof].copy_from_slice(&fx.core.commanded_positions);
    cart_now.path.clear();
    for k in 0..=10 {
        let f = k as f32 / 10.0;
        let mut q = cur_start;
        q[0] += 0.05 * f;
        cart_now.path.push(&q[..dof], f64::from(f)).unwrap();
    }
    cart_now.primitive = Primitive::Prepared {
        child: None,
        dof: dof as u8,
        start: cur_start,
        kind: PathKind::Cartesian,
        duration_s: 1.0,
        power_scale: None,
    };
    b.submit(
        ClientId(1),
        2,
        fx.core.authority(&fx.ctx(fx.now())),
        deadline,
        Submit::Install(&move |s: &mut PlanSlot| s.copy_from(&cart_now).unwrap()),
    )
    .unwrap();
    let (step_cart, a_cart_commit, f_cart_commit) = counted(|| fx.step());
    assert!(step_cart.command.unwrap().1.is_ok(), "prepared path committed");
    assert_eq!(
        (a_cart_commit, f_cart_commit),
        (0, 0),
        "accepted prepared Cartesian path commit + tick touched the allocator"
    );
    assert!(
        matches!(fx.core.active_trajectory.as_ref().map(|a| &a.reference), Some(Reference::Path(p)) if p.borrows_plan_slot()),
        "prepared Cartesian path must borrow directly from the immutable plan slot"
    );
    assert_zero("prepared Cartesian path steady ticks", &count_ticks(&mut fx.core, 10, |_, _| {}));
    stop.request();
    let (stopped_cart, a_stop2, f_stop2) = counted(|| fx.step());
    assert!(stopped_cart.stopped);
    assert_eq!((a_stop2, f_stop2), (0, 0), "urgent stop ending slot-borrowed path touched the allocator");
    b.poll(|_, _| true);
    assert_eq!(b.stats().retired, 2, "slot reclaimed after Cartesian path terminal");
    for _ in 0..20 {
        if fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-3) { break; }
        fx.step();
    }
    let _ = fx.core.core.hub().latest();

    // 7. Accepted Jog commit (`Primitive::JointQuintic` via `fx.step()`) performs (0, 0)
    //    allocs/frees (including inline quintic planner + stack workspace sweep) and runs
    //    steady-state ticks with (0, 0) allocs/frees.
    let mut jog_start = [0.0f32; MAX_DOF];
    jog_start[..dof].copy_from_slice(&fx.core.commanded_positions);
    let mut jog_goal = jog_start;
    jog_goal[0] -= 0.05;
    let jog_after = Primitive::JointQuintic { dof: dof as u8, start: jog_start, q: jog_goal, duration_s: 1.0 };
    b.submit(
        ClientId(1),
        3,
        fx.core.authority(&fx.ctx(fx.now())),
        deadline,
        Submit::Install(&move |s: &mut PlanSlot| s.set(jog_after)),
    )
    .unwrap();
    let (step_jog, a_jog_commit, f_jog_commit) = counted(|| fx.step());
    assert!(step_jog.command.unwrap().1.is_ok(), "jog committed via jog_action_slice");
    assert_eq!(
        (a_jog_commit, f_jog_commit),
        (0, 0),
        "accepted jog commit + tick touched the allocator"
    );
    assert!(fx.core.active_trajectory.is_some());
    assert_zero("RT-committed jog steady ticks", &count_ticks(&mut fx.core, 10, |_, _| {}));
}

/// Accepted commits for prepared Cartesian path, prepared replay, and jog also perform
/// `(0, 0)` allocations/frees under the real device profile (`rebot-b601dm.json`:
/// workspace policy + link capsules + self-collision pairs + power-budget planning enabled).
#[test]
fn accepted_commits_on_device_profile_with_workspace_and_power_budget_do_not_allocate_or_free() {
    use armd::rt_core::broker::{BootId, Capacities, ClientId, Submit};
    use armd::rt_core::mailbox::MAX_DOF;
    use armd::rt_core::ops::{
        tool_digest_hash, CoreOps, EpisodeRef, PathKind, PlanSlot, Primitive, ReleaseFixture,
    };

    DEVICE.with(|d| d.set(true));
    let mut inner = supervisor(false);
    DEVICE.with(|d| d.set(false));
    assert!(inner.workspace_policy().is_some(), "device profile must exercise workspace sweep");
    assert!(inner.power_budget().is_some_and(|b| b.planning.enabled), "device profile must exercise power planning");
    warm(&mut inner, 4);

    let boot = BootId::fresh();
    let (mut fx, mut b, _tw, stop) = ReleaseFixture::new(inner, boot, Capacities::default());
    for _ in 0..4 { fx.step(); }
    let _ = fx.core.core.hub().latest();
    let deadline = fx.now() + Duration::from_secs(30);
    let dof = fx.core.commanded_positions.len();
    let tool = tool_digest_hash(&fx.core.config.active_tool.config_digest);

    // 1. Accepted prepared Cartesian path commit on device profile.
    let mut start = [0.0f32; MAX_DOF];
    start[..dof].copy_from_slice(&fx.core.commanded_positions);
    let mut cart = PlanSlot::default();
    for k in 0..=10 {
        let f = k as f32 / 10.0;
        let mut q = start;
        q[0] += 0.04 * f;
        cart.path.push(&q[..dof], f64::from(f)).unwrap();
    }
    cart.primitive = Primitive::Prepared {
        child: None,
        dof: dof as u8,
        start,
        kind: PathKind::Cartesian,
        duration_s: 0.8,
        power_scale: None,
    };
    b.submit(
        ClientId(1),
        1,
        fx.core.authority(&fx.ctx(fx.now())),
        deadline,
        Submit::Install(&move |s: &mut PlanSlot| s.copy_from(&cart).unwrap()),
    )
    .unwrap();
    let (rep, a, f) = counted(|| fx.step());
    assert!(rep.command.unwrap().1.is_ok(), "device profile prepared path committed");
    assert_eq!((a, f), (0, 0), "device profile prepared path commit touched allocator");
    stop.request();
    fx.step();
    b.poll(|_, _| true);

    // 2. Accepted prepared replay commit on device profile (exercises approach quintic + power planning + workspace sweep).
    start[..dof].copy_from_slice(&fx.core.commanded_positions);
    let mut replay = PlanSlot::default();
    for k in 0..=20 {
        let f = k as f32 / 20.0;
        let mut q = start;
        q[0] += 0.04 * f;
        replay.path.push(&q[..dof], k as f64).unwrap();
    }
    replay.primitive = Primitive::Prepared {
        child: None,
        dof: dof as u8,
        start,
        kind: PathKind::Replay {
            episode: EpisodeRef::new("dev_ep").unwrap(),
            speed: 1.0,
            gripper: false,
            sample_dt: 0.02,
            tool_digest: tool,
        },
        duration_s: 0.4,
        power_scale: None,
    };
    b.submit(
        ClientId(1),
        2,
        fx.core.authority(&fx.ctx(fx.now())),
        deadline,
        Submit::Install(&move |s: &mut PlanSlot| s.copy_from(&replay).unwrap()),
    )
    .unwrap();
    let (rep, a, f) = counted(|| fx.step());
    assert!(rep.command.unwrap().1.is_ok(), "device profile prepared replay committed");
    assert_eq!((a, f), (0, 0), "device profile prepared replay commit touched allocator");
    stop.request();
    fx.step();
    b.poll(|_, _| true);

    // 3. Accepted jog commit on device profile (exercises quintic planner + power planning + workspace sweep).
    start[..dof].copy_from_slice(&fx.core.commanded_positions);
    let mut goal = start;
    goal[0] -= 0.04;
    let jog = Primitive::JointQuintic { dof: dof as u8, start, q: goal, duration_s: 0.8 };
    b.submit(
        ClientId(1),
        3,
        fx.core.authority(&fx.ctx(fx.now())),
        deadline,
        Submit::Install(&move |s: &mut PlanSlot| s.set(jog)),
    )
    .unwrap();
    let (rep, a, f) = counted(|| fx.step());
    assert!(rep.command.unwrap().1.is_ok(), "device profile jog committed");
    assert_eq!((a, f), (0, 0), "device profile jog commit touched allocator");
}

/// I16 Item 3: Zero heap allocations and zero frees on `armd-control` for:
/// 1. Short-stall error formatting in `GripperMotion::step_into` on an obstructed calibration
///    stroke (and `SubState::Failed` handoff in `apply_action_step` on `macro_sub`).
/// 2. `GripPhase::Squeeze` entering squeeze, collecting samples across both normal settle windows
///    and >256-sample decimation windows, and computing medians in-place at settle completion.
#[test]
fn gripper_short_stall_error_and_grasp_squeeze_settle_are_zero_alloc() {
    use armd::motion::{
        GripperCommand, GripperMotion, GripperRequest, LearnMode, StepCtx, StepOut, StepStatus,
        GRIPPER_SQUEEZE_SAMPLE_CAP,
    };
    use armd::model::ArmConfig;

    let cfg = ArmConfig::rebot_dm();
    let dof = cfg.dof;
    let g_idx = cfg.gripper_index().unwrap();
    let policy = cfg.gripper_policy.clone().unwrap();
    let limit = cfg.limits[g_idx].clone();

    // 1. Obstructed calibration stroke short-stall in `step_into`:
    //    Closing from -5.4 toward 5% (-0.285), stalled at -3.0 (>= 0.25 rad short of target).
    let mut cal_gm = GripperMotion::try_with_close_speed_limit(
        GripperRequest {
            command: GripperCommand::Move,
            fraction: Some(0.05),
            grasp: false,
            speed: 0.30,
            timeout_s: None,
            tag: None,
        },
        policy.clone(),
        limit.clone(),
        -5.4,
        g_idx,
        Some(0.30),
    )
    .unwrap()
    .with_baseline(None, LearnMode::Calibration);

    let mut out = StepOut {
        positions: Vec::with_capacity(dof),
        velocities: Vec::with_capacity(dof),
        ..Default::default()
    };
    let mut commanded = vec![0.0f32; dof];
    let mut measured = vec![0.0f32; dof];
    let cmd_vel = vec![0.0f32; dof];
    let meas_vel = vec![0.0f32; dof];
    let mut torques = vec![0.0f32; dof];
    let loads = vec![0.0f32; dof];
    commanded[g_idx] = -2.8;
    measured[g_idx] = -3.0;
    torques[g_idx] = 0.70;

    let stalled_ctx = StepCtx {
        dt: 0.02,
        time_scale: 1.0,
        measured: &measured,
        measured_velocities: &meas_vel,
        commanded: &commanded,
        commanded_velocities: &cmd_vel,
        torques: &torques,
        loads: &loads,
        feedback_fresh: true,
        torque_feedback: true,
        load_feedback: false,
        gripper_stalled: true,
        gripper_stall_sign: 1.0,
        gripper_force_capped: false,
    };

    let ((), allocs, frees) = counted(|| cal_gm.step_into(&stalled_ctx, &mut out));
    assert_eq!((allocs, frees), (0, 0), "obstructed calibration short-stall step_into touched allocator");
    match &out.status {
        StepStatus::Failed { code, message } => {
            assert_eq!(*code, "GRIPPER_TIMEOUT");
            assert!(
                message.as_str().contains("Gripper stalled short of target")
                    && message.as_str().contains("-3.000"),
                "unexpected short-stall message: {message}"
            );
        }
        other => panic!("expected GRIPPER_TIMEOUT short-stall failure, got {other:?}"),
    }

    // 2. Grasp squeeze: enter squeeze, accumulate >256 samples (exercising both normal and
    //    bounded 2x decimation paths) and finish settle with in-place median sorting, all (0, 0).
    let mut long_policy = policy.clone();
    long_policy.settle_ms = 5_000; // 5 s; with dt=0.01 s -> 500 samples (> GRIPPER_SQUEEZE_SAMPLE_CAP = 256)
    let mut grasp_gm = GripperMotion::try_with_close_speed_limit(
        GripperRequest {
            command: GripperCommand::Close,
            fraction: None,
            grasp: true,
            speed: 0.30,
            timeout_s: Some(30.0),
            tag: None,
        },
        long_policy,
        limit,
        -5.4,
        g_idx,
        Some(0.30),
    )
    .unwrap();

    commanded[g_idx] = -2.0;
    measured[g_idx] = -2.0;
    torques[g_idx] = 0.72;
    let enter_ctx = StepCtx {
        dt: 0.01,
        time_scale: 1.0,
        measured: &measured,
        measured_velocities: &meas_vel,
        commanded: &commanded,
        commanded_velocities: &cmd_vel,
        torques: &torques,
        loads: &loads,
        feedback_fresh: true,
        torque_feedback: true,
        load_feedback: false,
        gripper_stalled: true,
        gripper_stall_sign: 1.0,
        gripper_force_capped: false,
    };
    let ((), a_enter, f_enter) = counted(|| grasp_gm.step_into(&enter_ctx, &mut out));
    assert_eq!((a_enter, f_enter), (0, 0), "entering GripPhase::Squeeze touched allocator");
    assert_eq!(out.status, StepStatus::Running);

    let mut settle_ticks = 0usize;
    let mut settled_done = None;
    let mut total_a = 0u64;
    let mut total_f = 0u64;
    for tick in 0..800 {
        commanded[g_idx] = out.positions[g_idx];
        torques[g_idx] = 0.65 + ((tick % 7) as f32) * 0.02;
        let sq_ctx = StepCtx {
            dt: 0.01,
            time_scale: 1.0,
            measured: &measured,
            measured_velocities: &meas_vel,
            commanded: &commanded,
            commanded_velocities: &cmd_vel,
            torques: &torques,
            loads: &loads,
            feedback_fresh: true,
            torque_feedback: true,
            load_feedback: false,
            gripper_stalled: true,
            gripper_stall_sign: 1.0,
            gripper_force_capped: false,
        };
        let ((), a, f) = counted(|| grasp_gm.step_into(&sq_ctx, &mut out));
        total_a += a;
        total_f += f;
        settle_ticks += 1;
        if let StepStatus::Done { holding, gripper_gap_rad, .. } = out.status {
            settled_done = Some((holding, gripper_gap_rad));
            break;
        }
    }
    assert!(
        settle_ticks > GRIPPER_SQUEEZE_SAMPLE_CAP,
        "expected > {GRIPPER_SQUEEZE_SAMPLE_CAP} squeeze ticks to cover decimation, got {settle_ticks}"
    );
    assert_eq!(
        (total_a, total_f),
        (0, 0),
        "grasp squeeze sample collection / settle median touched allocator"
    );
    let (holding, gap) = settled_done.expect("grasp squeeze did not finish settle");
    assert_eq!(holding, Some(true));
    assert!((gap.unwrap() - 2.0).abs() < 1e-3);

    // 3. Full `SupervisorInner::tick_publish` coverage:
    //    3a. Grasp squeeze entry and all steady squeeze ticks prior to final `Box<GripperMotion>`
    //        retirement perform `(0, 0)` allocs/frees, and the settle completion tick performs `0`
    //        allocations (only retiring the boxed `GripperMotion` on `active_trajectory = None`).
    {
        let mut inner = supervisor(false);
        inner.commanded_positions[g_idx] = -2.0;
        inner.measured_positions[g_idx] = -2.0;
        inner.target_positions[g_idx] = -2.0;
        let ep = inner.safety.epoch_id();
        let (grasp_id, _) = inner
            .start_gripper(
                GripperRequest {
                    command: GripperCommand::Close,
                    fraction: None,
                    grasp: true,
                    speed: 0.30,
                    timeout_s: Some(10.0),
                    tag: None,
                },
                ep,
            )
            .unwrap();
        warm(&mut inner, 3);
        // Latch stall so the next tick enters `GripPhase::Squeeze`, then step through all settle ticks.
        inner.gripper_stalled = true;
        inner.gripper_stall_sign = 1.0;
        let mut steady_squeeze_ticks = 0usize;
        let mut done_status = None;
        for _ in 0..80 {
            let (a, f) = tick_counted(&mut inner);
            let act = inner.last_action.as_ref().unwrap();
            assert_eq!(act.id, grasp_id);
            if act.is_running() {
                assert_eq!((a, f), (0, 0), "grasp squeeze tick_publish touched allocator");
                steady_squeeze_ticks += 1;
            } else {
                assert_eq!(a, 0, "grasp squeeze settle completion tick allocated on control thread");
                done_status = Some(act.clone());
                break;
            }
        }
        assert!(steady_squeeze_ticks >= 10, "expected >= 10 steady squeeze ticks, got {steady_squeeze_ticks}");
        let done = done_status.expect("grasp squeeze did not finish via tick_publish");
        assert_eq!(done.state.as_str(), "done", "{done:?}");
    }

    //    3b. Obstructed calibration macro stroke short-stall tick performs `0` allocations
    //        (only retiring `Box<GripperMotion>` + `Box<MoveSamples>` on `active_trajectory = None`).
    {
        let mut inner = supervisor(false);
        inner.commanded_positions[g_idx] = -2.0;
        inner.measured_positions[g_idx] = -2.0;
        inner.target_positions[g_idx] = -2.0;
        let ep = inner.safety.epoch_id();
        let cal_id = inner.begin_orchestrated_action("gripper_calibrate", None, ep).unwrap();
        inner
            .start_macro_gripper_learning_within(
                cal_id,
                GripperRequest {
                    command: GripperCommand::Move,
                    fraction: Some(0.05),
                    grasp: false,
                    speed: 0.30,
                    timeout_s: None,
                    tag: None,
                },
                LearnMode::Calibration,
                None,
                false,
            )
            .unwrap();
        warm(&mut inner, 3);
        inner.gripper_stalled = true;
        inner.gripper_stall_sign = 1.0;
        let (a_fail, _f_box_retire) = tick_counted(&mut inner);
        assert_eq!(a_fail, 0, "obstructed calibration short-stall tick allocated on control thread");
        match inner.macro_run.as_ref().map(|m| &m.sub) {
            Some(armd::macro_action::SubState::Failed { code, message }) => {
                assert_eq!(code.as_str(), "GRIPPER_TIMEOUT");
                assert!(message.as_str().contains("Gripper stalled short of target"), "{message}");
            }
            other => panic!("expected macro SubState::Failed(GRIPPER_TIMEOUT), got {other:?}"),
        }
    }

    //    3c. Non-macro direct gripper short-stall tick (`apply_action_step` -> `finish_action_bounded`
    //        + `ActionStatus.message` publication) ALSO performs `0` allocations on `armd-control`.
    {
        let mut inner = supervisor(false);
        inner.commanded_positions[g_idx] = -2.0;
        inner.measured_positions[g_idx] = -2.0;
        inner.target_positions[g_idx] = -2.0;
        let ep = inner.safety.epoch_id();
        let (act_id, _) = inner
            .start_gripper(
                GripperRequest {
                    command: GripperCommand::Move,
                    fraction: Some(0.0),
                    grasp: false,
                    speed: 0.30,
                    timeout_s: None,
                    tag: None,
                },
                ep,
            )
            .unwrap();
        warm(&mut inner, 3);
        inner.gripper_stalled = true;
        inner.gripper_stall_sign = 1.0;
        let (a_fail, _f_box_retire) = tick_counted(&mut inner);
        assert_eq!(a_fail, 0, "non-macro gripper short-stall tick allocated on control thread");
        let act = inner.last_action.as_ref().unwrap();
        assert_eq!(act.id, act_id);
        assert_eq!(act.state.as_str(), "failed");
        assert_eq!(act.code.as_deref(), Some("GRIPPER_TIMEOUT"));
        assert!(
            act.message.as_deref().is_some_and(|m| m.contains("Gripper stalled short of target")),
            "{:?}",
            act.message
        );
    }
}
