//! Zero-allocation verification for the bounded CAN TX send path and per-frame TX
//! instrumentation (counting global allocator harness matching `tests/test_rt_tick_alloc.rs`).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::VecDeque;
use std::time::Duration;

use armd::diagnostics::tick::{
    record_tx_attempt, record_tx_deferred, record_tx_dropped_stale, record_tx_dropped_stopped,
    reset_tx_metrics, thread_usage, TickSample, TickStats, TxAttemptKind, UsageMark,
};
use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, JointTarget, TxBatchOutcome, TxBudget,
};
use armd::protocol::damiao::{self, DamiaoLimits};
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
enum Mode {
    Ok,
    WouldBlockAfter(usize),
    EnobufsAfter(usize),
}

/// Pre-allocated Damiao CAN bus fake (zero allocations during `transmit` and `receive`).
struct NoAllocDamiaoBus {
    motors: Vec<(u16, DamiaoLimits, f32, bool)>,
    rx: VecDeque<CanFrame>,
    clock: std::sync::Arc<MockClock>,
    mode: Mode,
    tx_count: usize,
}

impl NoAllocDamiaoBus {
    fn new(limits: &[DamiaoLimits]) -> Self {
        Self {
            motors: limits.iter().enumerate().map(|(i, l)| (i as u16 + 1, *l, 0.0, false)).collect(),
            rx: VecDeque::with_capacity(1024),
            clock: std::sync::Arc::new(MockClock::new()),
            mode: Mode::Ok,
            tx_count: 0,
        }
    }

    fn reply(&mut self, k: usize) {
        let (id, l, q, enabled) = self.motors[k];
        let p = damiao::float_to_uint(q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let data = [
            (u8::from(enabled) << 4) | (id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            30,
            30,
        ];
        if self.rx.len() < self.rx.capacity() {
            self.rx.push_back(CanFrame::new_standard(0x10 + id, &data).unwrap());
        }
    }
}

impl CanTransport for NoAllocDamiaoBus {
    fn clock(&self) -> Option<std::sync::Arc<dyn DriverClock>> {
        Some(self.clock.clone())
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        match self.mode {
            Mode::Ok => {}
            Mode::WouldBlockAfter(n) if self.tx_count >= n => return Err(TransportError::WouldBlock),
            Mode::EnobufsAfter(n) if self.tx_count >= n => return Err(TransportError::NoBufferSpace),
            _ => {}
        }
        self.tx_count += 1;
        let id = frame.id as u16;
        let find = |m: &[(u16, DamiaoLimits, f32, bool)], id: u16| m.iter().position(|x| x.0 == id);
        if id == 0x7FF {
            let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(k) = find(&self.motors, mid) {
                if frame.data[2] == 0xCC {
                    self.reply(k);
                }
            }
        } else if let Some(k) = find(&self.motors, id) {
            if frame.data == damiao::CMD_ENABLE {
                self.motors[k].3 = true;
            }
            if frame.data == damiao::CMD_DISABLE {
                self.motors[k].3 = false;
            }
            self.reply(k);
        } else if let Some(k) =
            find(&self.motors, id.wrapping_sub(0x100)).or_else(|| find(&self.motors, id.wrapping_sub(0x300)))
        {
            self.motors[k].2 = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.reply(k);
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.rx.pop_front().ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

#[test]
fn bounded_send_path_does_not_allocate_on_healthy_backpressure_carry_and_expiry() {
    let limits = b601_limits();
    let bus = NoAllocDamiaoBus::new(&limits);
    let clock = bus.clock.clone();
    let mut driver = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    driver.set_torque(true).expect("enable");
    let targets_a: Vec<JointTarget> = (0..7).map(|i| JointTarget::position(0.1 * i as f32)).collect();
    let targets_b: Vec<JointTarget> = (0..7).map(|i| JointTarget::position(-0.1 * i as f32)).collect();
    // Warm up thread-locals once before counting.
    let _ = driver.write_joint_targets_bounded(&targets_a).unwrap();
    reset_tx_metrics();

    // 1. Healthy 7-joint bounded send: 0 allocs, 0 frees.
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_a));
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert_eq!((a, f), (0, 0), "healthy bounded send touched allocator");

    // 2. WouldBlock mid-batch deferral: 0 allocs, 0 frees.
    driver.can_mut().mode = Mode::WouldBlockAfter(3);
    driver.can_mut().tx_count = 0;
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_a));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 4 });
    assert_eq!((a, f), (0, 0), "WouldBlock deferral touched allocator");

    // 3. ENOBUFS mid-batch deferral with opposite-target coalescing: 0 allocs, 0 frees.
    driver.can_mut().mode = Mode::EnobufsAfter(1);
    driver.can_mut().tx_count = 0;
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_b));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 6 });
    assert_eq!((a, f), (0, 0), "ENOBUFS deferral + coalescing touched allocator");

    // 4. Carry-over drain when queue opens: 0 allocs, 0 frees.
    driver.can_mut().mode = Mode::Ok;
    let (res, a, f) = counted(|| driver.drain_pending_tx());
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert_eq!((a, f), (0, 0), "pending TX drain touched allocator");

    // 5. Budget exhaustion (max_frames = 4) + inclusive 45 ms stale expiry drop: 0 allocs, 0 frees.
    driver.set_tx_budget(TxBudget {
        max_frames: 4,
        max_duration: Duration::from_millis(5),
        frame_max_age: Duration::from_millis(45),
    });
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_a));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 3 });
    assert_eq!((a, f), (0, 0), "budget exhaustion deferral touched allocator");
    clock.advance(Duration::from_millis(45));
    let (res, a, f) = counted(|| driver.drain_pending_tx());
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert_eq!((a, f), (0, 0), "45 ms stale expiry drop touched allocator");

    // 6. Legacy write_joint_targets returning Err(TxDeferred) + discarding queue: 0 allocs, 0 frees.
    let (res, a, f) = counted(|| driver.write_joint_targets(&targets_b));
    assert!(matches!(res, Err(ArmDriverError::TxDeferred { pending: 3 })), "{res:?}");
    assert_eq!((a, f), (0, 0), "legacy TxDeferred error path touched allocator");

    // 7. Explicit discard_pending_tx: 0 allocs, 0 frees.
    let _ = driver.write_joint_targets_bounded(&targets_a).unwrap();
    let (dropped, a, f) = counted(|| driver.discard_pending_tx());
    assert_eq!(dropped, 3);
    assert_eq!((a, f), (0, 0), "discard_pending_tx touched allocator");
}

#[test]
fn diagnostics_tick_tx_recording_and_usage_mark_do_not_allocate() {
    let mut stats = TickStats::default();
    // Warm up thread-local once.
    reset_tx_metrics();
    let _ = UsageMark::now();

    let ((), a, f) = counted(|| {
        for _ in 0..50 {
            let start = UsageMark::now();
            let locked = UsageMark::now();
            record_tx_attempt(TxAttemptKind::Sent, 120_000, 30_000);
            record_tx_attempt(TxAttemptKind::WouldBlock, 40_000, 10_000);
            record_tx_attempt(TxAttemptKind::Enobufs, 50_000, 15_000);
            record_tx_deferred(2, 2);
            record_tx_dropped_stale(1);
            record_tx_dropped_stopped(1);
            let released = UsageMark::now();
            let usage = thread_usage(&start, &locked, &released);
            stats.record(&TickSample { tick_us: 1_000, lock_us: 500, usage, ..Default::default() });
        }
    });
    assert_eq!((a, f), (0, 0), "tick TX diagnostics recording touched allocator");
    let snap = stats.snapshot();
    assert_eq!(snap.tx.frames_total, 50);
    assert_eq!(snap.tx.would_block_total, 50);
    assert_eq!(snap.tx.enobufs_total, 50);
}

#[test]
fn supervisor_tick_publish_with_damiao_tx_instrumentation_does_not_allocate() {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits = b601_limits();
    let bus = NoAllocDamiaoBus::new(&limits);
    let driver = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    let mut q = inner.commanded_positions.clone();
    q[0] += 0.3;
    let epoch = inner.safety.epoch_id();
    inner.jog(q, Some(2.0), Some(epoch)).expect("jog");
    for _ in 0..5 {
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    let mut stats = TickStats::default();
    for _ in 0..20 {
        let (published, a, f) = counted(|| {
            let start = UsageMark::now();
            let locked = UsageMark::now();
            let p = inner.tick_publish(0.02, 1, false);
            let released = UsageMark::now();
            let usage = thread_usage(&start, &locked, &released);
            stats.record(&TickSample { tick_us: 1_000, lock_us: 500, usage, ..Default::default() });
            p
        });
        assert!(published.kept.is_none());
        drop(published);
        let _ = inner.core.hub().latest();
        assert_eq!((a, f), (0, 0), "steady-state tick with Damiao TX instrumentation touched allocator");
    }
    let snap = stats.snapshot();
    assert_eq!(snap.tx.frames_total, 20 * 7, "7 frames per tick over 20 ticks");
}
