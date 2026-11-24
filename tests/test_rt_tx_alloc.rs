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
    is_estop_inhibited, ArmDriver, ArmDriverError, DamiaoArmDriver, JointMode, JointTarget,
    TxBatchOutcome, TxBudget,
};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::urgent::{EstopGate, StopRequests};
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
    HardError,
    #[cfg(target_os = "linux")]
    SocketCanOsAfter(usize, i32),
}

/// Pre-allocated Damiao CAN bus fake (zero allocations during `transmit` and `receive`).
struct NoAllocDamiaoBus {
    motors: Vec<(u16, DamiaoLimits, f32, bool, u32)>,
    rx: VecDeque<(CanFrame, Option<std::time::Instant>)>,
    /// Post-transmit replies promoted to `rx` once `rx` drains empty (models SocketCAN causality
    /// without heap allocation so post-write probe drains separate pre-probe from post-probe RX).
    pending_rx: VecDeque<(CanFrame, Option<std::time::Instant>)>,
    clock: std::sync::Arc<MockClock>,
    mode: Mode,
    mute_j1: bool,
    mute_j7: bool,
    tx_count: usize,
    tx_step: Duration,
    flush_err: Option<Mode>,
    on_tx_stop: Option<(usize, StopRequests)>,
    on_flush_stop: Option<StopRequests>,
    pf_tx_count: u64,
}

impl NoAllocDamiaoBus {
    fn new(limits: &[DamiaoLimits]) -> Self {
        Self {
            motors: limits
                .iter()
                .enumerate()
                .map(|(i, l)| (i as u16 + 1, *l, 0.0, false, damiao::CTRL_MODE_POS_VEL))
                .collect(),
            rx: VecDeque::with_capacity(1024),
            pending_rx: VecDeque::with_capacity(64),
            clock: std::sync::Arc::new(MockClock::new()),
            mode: Mode::Ok,
            mute_j1: false,
            mute_j7: false,
            tx_count: 0,
            tx_step: Duration::ZERO,
            flush_err: None,
            on_tx_stop: None,
            on_flush_stop: None,
            pf_tx_count: 0,
        }
    }

    fn build_reply_frame(&self, k: usize) -> CanFrame {
        let (id, l, q, enabled, _) = self.motors[k];
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
        CanFrame::new_standard(0x10 + id, &data).unwrap()
    }

    fn reply(&mut self, k: usize) {
        if (k == 0 && self.mute_j1) || (k == 6 && self.mute_j7) {
            return;
        }
        let frame = self.build_reply_frame(k);
        if self.rx.len() < self.rx.capacity() {
            self.rx.push_back((frame, Some(self.clock.now())));
        }
    }

    fn reply_pending(&mut self, k: usize) {
        if (k == 0 && self.mute_j1) || (k == 6 && self.mute_j7) {
            return;
        }
        let frame = self.build_reply_frame(k);
        if self.pending_rx.len() < self.pending_rx.capacity() {
            self.pending_rx.push_back((frame, Some(self.clock.now())));
        }
    }

    fn push_all_feedback(&mut self) {
        for k in 0..self.motors.len() {
            self.reply(k);
        }
    }
}

impl CanTransport for NoAllocDamiaoBus {
    fn clock(&self) -> Option<std::sync::Arc<dyn DriverClock>> {
        Some(self.clock.clone())
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        if !self.tx_step.is_zero() {
            self.clock.advance(self.tx_step);
        }
        if let Some((at, ref stops)) = self.on_tx_stop {
            if self.tx_count == at {
                let _ = stops.request();
            }
        }
        match self.mode {
            Mode::Ok => {}
            Mode::WouldBlockAfter(n) if self.tx_count >= n => return Err(TransportError::WouldBlock),
            Mode::EnobufsAfter(n) if self.tx_count >= n => return Err(TransportError::NoBufferSpace),
            Mode::HardError => return Err(TransportError::NotOpen),
            #[cfg(target_os = "linux")]
            Mode::SocketCanOsAfter(n, errno) if self.tx_count >= n => {
                return Err(armd::transport::socketcan::map_socketcan_tx_error(
                    std::io::Error::from_raw_os_error(errno),
                ));
            }
            _ => {}
        }
        self.tx_count += 1;
        let id = frame.id as u16;
        let find = |m: &[(u16, DamiaoLimits, f32, bool, u32)], id: u16| m.iter().position(|x| x.0 == id);
        if id == 0x7FF {
            let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(k) = find(&self.motors, mid) {
                match frame.data[2] {
                    0xCC => self.reply_pending(k),
                    damiao::REG_READ => {
                        let (_, l, _, _, mode) = self.motors[k];
                        let val = match frame.data[3] {
                            damiao::RID_CTRL_MODE => mode.to_le_bytes(),
                            damiao::RID_PMAX => l.p_max.to_le_bytes(),
                            damiao::RID_VMAX => l.v_max.to_le_bytes(),
                            damiao::RID_TMAX => l.t_max.to_le_bytes(),
                            damiao::RID_MST_ID => (0x10u32 + u32::from(mid)).to_le_bytes(),
                            damiao::RID_ESC_ID => u32::from(mid).to_le_bytes(),
                            damiao::RID_TIMEOUT => 2000u32.to_le_bytes(),
                            damiao::RID_SW_VER => 0x0001_0000u32.to_le_bytes(),
                            _ => 0u32.to_le_bytes(),
                        };
                        let [lo, hi] = mid.to_le_bytes();
                        if self.rx.len() < self.rx.capacity() {
                            self.rx.push_back((
                                CanFrame::new_standard(
                                    0x10 + mid,
                                    &[lo, hi, damiao::REG_READ, frame.data[3], val[0], val[1], val[2], val[3]],
                                )
                                .unwrap(),
                                Some(self.clock.now()),
                            ));
                        }
                    }
                    damiao::REG_WRITE => {
                        let val = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
                        if frame.data[3] == damiao::RID_CTRL_MODE {
                            self.motors[k].4 = u32::from_le_bytes(val);
                        }
                        let [lo, hi] = mid.to_le_bytes();
                        if self.rx.len() < self.rx.capacity() {
                            self.rx.push_back((
                                CanFrame::new_standard(
                                    0x10 + mid,
                                    &[lo, hi, damiao::REG_WRITE, frame.data[3], val[0], val[1], val[2], val[3]],
                                )
                                .unwrap(),
                                Some(self.clock.now()),
                            ));
                        }
                    }
                    _ => {}
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
            if id == 0x307 {
                self.pf_tx_count += 1;
            }
            self.motors[k].2 = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.reply(k);
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        match self.rx.pop_front() {
            Some(f) => Ok(f),
            None => {
                while let Some(f) = self.pending_rx.pop_front() {
                    if self.rx.len() < self.rx.capacity() {
                        self.rx.push_back(f);
                    }
                }
                Err(TransportError::WouldBlock)
            }
        }
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        if let Some(ref stops) = self.on_flush_stop {
            let _ = stops.request();
        }
        match self.flush_err {
            None | Some(Mode::Ok) => Ok(()),
            Some(Mode::WouldBlockAfter(_)) => Err(TransportError::WouldBlock),
            Some(Mode::EnobufsAfter(_)) => Err(TransportError::NoBufferSpace),
            Some(Mode::HardError) => Err(TransportError::NotOpen),
            #[cfg(target_os = "linux")]
            Some(Mode::SocketCanOsAfter(_, errno)) => {
                Err(armd::transport::socketcan::map_socketcan_tx_error(
                    std::io::Error::from_raw_os_error(errno),
                ))
            }
        }
    }

    fn supports_bounded_flush(&self) -> bool { true }
    fn flush_is_noop(&self) -> bool { true }
    fn supports_nonblocking_rt(&self) -> bool { true }
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
    driver.begin_tx_tick();
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_a));
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert_eq!((a, f), (0, 0), "healthy bounded send touched allocator");

    // 2. WouldBlock mid-batch deferral: 0 allocs, 0 frees.
    driver.begin_tx_tick();
    driver.can_mut().mode = Mode::WouldBlockAfter(3);
    driver.can_mut().tx_count = 0;
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_a));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 4 });
    assert_eq!((a, f), (0, 0), "WouldBlock deferral touched allocator");

    // 3. ENOBUFS mid-batch deferral with opposite-target coalescing: 0 allocs, 0 frees.
    driver.begin_tx_tick();
    driver.can_mut().mode = Mode::EnobufsAfter(1);
    driver.can_mut().tx_count = 0;
    let (res, a, f) = counted(|| driver.write_joint_targets_bounded(&targets_b));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 6 });
    assert_eq!((a, f), (0, 0), "ENOBUFS deferral + coalescing touched allocator");

    // 4. Carry-over drain when queue opens: 0 allocs, 0 frees.
    driver.begin_tx_tick();
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
    driver.begin_tx_tick();
    let (res, a, f) = counted(|| driver.drain_pending_tx());
    assert_eq!(res.unwrap(), TxBatchOutcome::Expired { dropped: 3, pending: 0 });
    assert_eq!((a, f), (0, 0), "45 ms stale expiry drop touched allocator");

    // 6. Legacy write_joint_targets returning Err(TxDeferred) + discarding queue: 0 allocs, 0 frees.
    driver.begin_tx_tick();
    let (res, a, f) = counted(|| driver.write_joint_targets(&targets_b));
    assert!(matches!(res, Err(ArmDriverError::TxDeferred { pending: 3 })), "{res:?}");
    assert_eq!((a, f), (0, 0), "legacy TxDeferred error path touched allocator");

    // 7. Explicit discard_pending_tx: 0 allocs, 0 frees.
    driver.begin_tx_tick();
    let _ = driver.write_joint_targets_bounded(&targets_a).unwrap();
    let (dropped, a, f) = counted(|| driver.discard_pending_tx());
    assert_eq!(dropped, 3);
    assert_eq!((a, f), (0, 0), "discard_pending_tx touched allocator");
}

#[test]
fn safety_and_fenced_mit_failure_paths_do_not_allocate_or_free() {
    use std::sync::atomic::AtomicU64;
    use std::sync::Arc;

    let limits = b601_limits();
    let targets: Vec<JointTarget> = (0..7).map(|i| JointTarget::position(0.05 * i as f32)).collect();
    let modes = [
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
        JointMode::Mit,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
    ];
    let mk_fenced = || {
        let bus = NoAllocDamiaoBus::new(&limits);
        let clock = bus.clock.clone();
        let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap();
        d.can_mut().push_all_feedback();
        d.read_joint_states().unwrap();
        d.set_torque(true).expect("enable");
        d.set_joint_modes(&modes, &targets).expect("switch J4 to MIT");
        d.set_mit_deadline_fence(Some(Duration::from_millis(45)), Duration::from_millis(300));
        d.can_mut().tx_count = 0;
        let stops = StopRequests::new();
        d.set_estop_gate(
            EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
                .with_stops(stops.clone()),
        );
        reset_tx_metrics();
        (d, clock, stops)
    };

    // 1. Stop / E-stop before batch (fenced MIT) — 0 allocs, 0 frees.
    let (mut d, _, stops) = mk_fenced();
    let _ = stops.request();
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(is_estop_inhibited(&res.unwrap_err()));
    assert_eq!((a, f), (0, 0), "fenced pre-batch stop touched allocator");

    // 2. Stop / E-stop during final flush (fenced MIT) — 0 allocs, 0 frees.
    let (mut d, _, stops) = mk_fenced();
    d.can_mut().on_flush_stop = Some(stops);
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(is_estop_inhibited(&res.unwrap_err()));
    assert_eq!((a, f), (0, 0), "fenced post-flush stop touched allocator");

    // 3. Fenced MIT preflight budget exceeded — 0 allocs, 0 frees.
    let (mut d, _, _) = mk_fenced();
    d.set_tx_budget(TxBudget {
        max_frames: 5,
        max_duration: Duration::from_millis(5),
        frame_max_age: Duration::from_millis(45),
    });
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(matches!(res, Err(ArmDriverError::MitFenceTripped(_))), "{res:?}");
    assert_eq!((a, f), (0, 0), "fenced preflight budget trip touched allocator");

    // 4. Fenced MIT mid-batch ENOBUFS backpressure — 0 allocs, 0 frees.
    let (mut d, _, _) = mk_fenced();
    d.can_mut().mode = Mode::EnobufsAfter(3);
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(matches!(res, Err(ArmDriverError::Transport(TransportError::NoBufferSpace))), "{res:?}");
    assert_eq!((a, f), (0, 0), "fenced ENOBUFS trip touched allocator");

    // 5. Fenced MIT deadline expiry at 45 ms inclusive — 0 allocs, 0 frees.
    let (mut d, clock, _) = mk_fenced();
    clock.advance(Duration::from_millis(45));
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(matches!(res, Err(ArmDriverError::MitFenceTripped(_))), "{res:?}");
    assert_eq!((a, f), (0, 0), "fenced 45 ms deadline trip touched allocator");

    // 6. Fenced MIT flush WouldBlock — 0 allocs, 0 frees.
    let (mut d, _, _) = mk_fenced();
    d.can_mut().flush_err = Some(Mode::WouldBlockAfter(0));
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(matches!(res, Err(ArmDriverError::Transport(TransportError::WouldBlock))), "{res:?}");
    assert_eq!((a, f), (0, 0), "fenced flush WouldBlock trip touched allocator");

    // 7. Fenced MIT preflight wrong joint count — 0 allocs, 0 frees.
    let (mut d, _, _) = mk_fenced();
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets[..3]));
    assert!(matches!(res, Err(ArmDriverError::JointCountMismatch { expected: 7, actual: 3 })), "{res:?}");
    assert_eq!((a, f), (0, 0), "fenced preflight count mismatch touched allocator");

    // 8. Unfenced mid-batch stop + mid-pass stale Timeout + flush ENOBUFS — 0 allocs, 0 frees.
    let bus = NoAllocDamiaoBus::new(&limits);
    let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.clone())
        .unwrap();
    d.set_torque(true).expect("enable");
    let stops = StopRequests::new();
    d.set_estop_gate(
        EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
            .with_stops(stops.clone()),
    );
    d.can_mut().tx_count = 0;
    d.can_mut().on_tx_stop = Some((2, stops.clone()));
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(is_estop_inhibited(&res.unwrap_err()));
    assert_eq!((a, f), (0, 0), "unfenced mid-batch stop touched allocator");

    stops.mark_applied(stops.current());
    d.can_mut().on_tx_stop = None;
    d.begin_tx_tick();
    d.set_tx_budget(TxBudget {
        max_frames: 16,
        max_duration: Duration::from_millis(100),
        frame_max_age: Duration::from_millis(45),
    });
    d.can_mut().tx_step = Duration::from_millis(45);
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Expired { dropped: 6, pending: 0 });
    assert_eq!((a, f), (0, 0), "unfenced mid-pass stale drop Expired touched allocator");

    d.can_mut().tx_step = Duration::ZERO;
    d.can_mut().flush_err = Some(Mode::EnobufsAfter(0));
    d.begin_tx_tick();
    let (res, a, f) = counted(|| d.write_joint_targets_bounded(&targets));
    assert!(matches!(res, Err(ArmDriverError::Transport(TransportError::NoBufferSpace))), "{res:?}");
    assert_eq!((a, f), (0, 0), "unfenced flush ENOBUFS touched allocator");

    // 8b. Drain after an applied stop (requested == applied) invalidates stamped frames via set_estop_gate — 0 allocs, 0 frees.
    d.can_mut().flush_err = None;
    d.set_estop_gate(
        EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
            .with_stops(stops.clone()),
    );
    d.set_tx_budget(TxBudget {
        max_frames: 4,
        max_duration: Duration::from_millis(5),
        frame_max_age: Duration::from_millis(45),
    });
    assert_eq!(
        d.write_joint_targets_bounded(&targets).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let g = stops.request();
    stops.mark_applied(g);
    d.begin_tx_tick();
    let (res, a, f) = counted(|| d.drain_pending_tx());
    assert!(is_estop_inhibited(&res.unwrap_err()));
    assert_eq!((a, f), (0, 0), "applied-stop invalidation on drain touched allocator");

    // 9. Production SocketCAN hard OS error (ENETDOWN) construction + drop through fenced,
    // unfenced, recovery, and refresh paths — 0 allocs, 0 frees.
    #[cfg(target_os = "linux")]
    {
        // Direct mapper construction and disposal.
        let (is_os_err, a, f) = counted(|| {
            let err = armd::transport::socketcan::map_socketcan_tx_error(
                std::io::Error::from_raw_os_error(libc::ENETDOWN),
            );
            let ok = matches!(
                err,
                TransportError::SocketCanOs { op: "send", errno: Some(libc::ENETDOWN), .. }
            );
            drop(err);
            ok
        });
        assert!(is_os_err);
        assert_eq!((a, f), (0, 0), "SocketCAN ENETDOWN map + drop touched allocator");

        // Fenced MIT hard OS error + error drop.
        let (mut d, _, _) = mk_fenced();
        d.can_mut().mode = Mode::SocketCanOsAfter(2, libc::ENETDOWN);
        let (matched, a, f) = counted(|| {
            let res = d.write_joint_targets_bounded(&targets);
            let ok = matches!(
                res,
                Err(ArmDriverError::Transport(TransportError::SocketCanOs {
                    op: "send",
                    errno: Some(libc::ENETDOWN),
                    ..
                }))
            );
            drop(res);
            ok
        });
        assert!(matched);
        assert_eq!((a, f), (0, 0), "fenced MIT SocketCAN ENETDOWN + drop touched allocator");

        // Unfenced target write hard OS error + error drop.
        let bus = NoAllocDamiaoBus::new(&limits);
        let clock = bus.clock.clone();
        let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        d.can_mut().push_all_feedback();
        d.read_joint_states().unwrap();
        d.set_torque(true).expect("enable");
        d.can_mut().tx_count = 0;
        d.can_mut().mode = Mode::SocketCanOsAfter(1, libc::ENETDOWN);
        let (matched, a, f) = counted(|| {
            let res = d.write_joint_targets_bounded(&targets);
            let ok = matches!(
                res,
                Err(ArmDriverError::Transport(TransportError::SocketCanOs {
                    op: "send",
                    errno: Some(libc::ENETDOWN),
                    ..
                }))
            );
            drop(res);
            ok
        });
        assert!(matched);
        assert_eq!((a, f), (0, 0), "unfenced SocketCAN ENETDOWN + drop touched allocator");

        // Receive-side recovery and 0xCC refresh discarding hard OS errors inside the tick.
        clock.advance(Duration::from_millis(120));
        d.can_mut().tx_count = 0;
        d.can_mut().mode = Mode::SocketCanOsAfter(0, libc::ENETDOWN);
        let mut scratch = Vec::with_capacity(7);
        let (res, a, f) = counted(|| d.read_joint_states_into(&mut scratch));
        assert!(res.is_ok(), "{res:?}");
        assert_eq!((a, f), (0, 0), "recovery/refresh SocketCAN ENETDOWN disposal touched allocator");
    }
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
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
    let mut q = inner.commanded_positions.clone();
    q[0] += 0.3;
    let epoch = inner.safety.epoch_id();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog(__a, q, Some(2.0), Some(epoch))) } }.expect("jog");
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

#[test]
fn first_recovery_write_does_not_allocate_or_free() {
    let limits = b601_limits();
    let bus = NoAllocDamiaoBus::new(&limits);
    let clock = bus.clock.clone();
    let mut driver = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    driver.can_mut().push_all_feedback();
    driver.read_joint_states().unwrap();
    driver.set_torque(true).expect("enable");
    // Drain replies from enable and warm up TX thread-local without triggering recovery.
    let mut states = Vec::with_capacity(7);
    let mut recovered = Vec::with_capacity(7);
    driver.read_joint_states_into(&mut states).unwrap();
    reset_tx_metrics();

    // First autonomous recovery across all 7 joints (50 ms >= 45 ms recovery threshold, < 100 ms refresh):
    // recovery_written_joints must be preallocated so pushing all 7 indices does 0 allocs, 0 frees.
    clock.advance(Duration::from_millis(50));
    driver.can_mut().rx.clear();
    let (res, a, f) = counted(|| {
        driver.read_joint_states_into(&mut states)?;
        driver.take_recovery_writes_into(&mut recovered);
        Ok::<(), ArmDriverError>(())
    });
    res.unwrap();
    assert_eq!(recovered, vec![0, 1, 2, 3, 4, 5, 6]);
    assert_eq!((a, f), (0, 0), "first recovery write touched allocator");

    // Owned take_recovery_writes() must preserve capacity so subsequent recoveries stay zero-alloc.
    clock.advance(Duration::from_millis(50));
    driver.can_mut().rx.clear();
    driver.read_joint_states_into(&mut states).unwrap();
    let taken = driver.take_recovery_writes();
    assert_eq!(taken, vec![0, 1, 2, 3, 4, 5, 6]);

    clock.advance(Duration::from_millis(50));
    driver.can_mut().rx.clear();
    let (res, a, f) = counted(|| {
        driver.read_joint_states_into(&mut states)?;
        driver.take_recovery_writes_into(&mut recovered);
        Ok::<(), ArmDriverError>(())
    });
    res.unwrap();
    assert_eq!(recovered, vec![0, 1, 2, 3, 4, 5, 6]);
    assert_eq!((a, f), (0, 0), "recovery write after take_recovery_writes() touched allocator");
}

#[test]
fn budgeted_clear_fault_and_pre_enable_hold_errors_do_not_allocate_or_free() {
    // Major 4 regression: clear_fault_budgeted preserves zero-alloc TransportError on TX/flush
    // failure (WouldBlock, ENOBUFS, SocketCanOs ENETDOWN), and finish_pos_force_readback reports
    // pre-enable hold TX budget exhaustion without heap allocation.
    let limits = b601_limits();
    let bus = NoAllocDamiaoBus::new(&limits);
    let clock = bus.clock.clone();
    let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.clone())
        .unwrap();
    d.can_mut().push_all_feedback();
    d.read_joint_states().unwrap();
    reset_tx_metrics();

    // 1. clear_fault_budgeted TX WouldBlock -> 0 allocs, 0 frees.
    d.can_mut().mode = Mode::WouldBlockAfter(0);
    d.can_mut().tx_count = 0;
    let dl = clock.now() + Duration::from_millis(50);
    let (ok, a, f) = counted(|| {
        let res = d.clear_fault_budgeted(dl);
        matches!(res, Err(ArmDriverError::Transport(TransportError::WouldBlock)))
    });
    assert!(ok, "expected typed Transport(WouldBlock)");
    assert_eq!((a, f), (0, 0), "clear_fault_budgeted WouldBlock touched allocator");

    // 2. clear_fault_budgeted flush ENOBUFS -> 0 allocs, 0 frees.
    d.begin_tx_tick();
    d.can_mut().mode = Mode::Ok;
    d.can_mut().flush_err = Some(Mode::EnobufsAfter(0));
    let dl = clock.now() + Duration::from_millis(50);
    let (ok, a, f) = counted(|| {
        let res = d.clear_fault_budgeted(dl);
        matches!(res, Err(ArmDriverError::Transport(TransportError::NoBufferSpace)))
    });
    assert!(ok, "expected typed Transport(NoBufferSpace)");
    assert_eq!((a, f), (0, 0), "clear_fault_budgeted flush ENOBUFS touched allocator");

    // 3. clear_fault_budgeted SocketCAN ENETDOWN -> 0 allocs, 0 frees.
    #[cfg(target_os = "linux")]
    {
        d.begin_tx_tick();
        d.can_mut().flush_err = None;
        d.can_mut().mode = Mode::SocketCanOsAfter(0, libc::ENETDOWN);
        d.can_mut().tx_count = 0;
        let dl = clock.now() + Duration::from_millis(50);
        let (ok, a, f) = counted(|| {
            let res = d.clear_fault_budgeted(dl);
            matches!(
                res,
                Err(ArmDriverError::Transport(TransportError::SocketCanOs {
                    op: "send",
                    errno: Some(libc::ENETDOWN),
                    ..
                }))
            )
        });
        assert!(ok, "expected typed Transport(SocketCanOs ENETDOWN)");
        assert_eq!((a, f), (0, 0), "clear_fault_budgeted ENETDOWN touched allocator");
    }

    // 4. finish_pos_force_readback pre-enable hold TX budget exhaustion -> 0 allocs, 0 frees.
    let bus_pf = NoAllocDamiaoBus::new(&limits);
    let clock_pf = bus_pf.clock.clone();
    let mut d_pf = DamiaoArmDriver::new(bus_pf, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(armd::driver::GripperPosForce {
            joint: 6,
            current_raw: 460,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        })
        .unwrap();
    d_pf.can_mut().motors[6].2 = -1.0;
    d_pf.read_joint_states().unwrap();
    d_pf.can_mut().push_all_feedback();
    d_pf.read_joint_states().unwrap();
    // Advance through mode restore and readback request under default budget.
    d_pf.begin_torque(true).unwrap();
    // Poll restore reply -> transitions to Readback.
    clock_pf.advance(Duration::from_millis(1));
    d_pf.poll_torque(clock_pf.now() + Duration::from_millis(10)).unwrap();
    // Poll readback -> sends RID_CTRL_MODE read request and queues reply in NoAllocDamiaoBus!
    // NoAllocDamiaoBus doesn't simulate REG_READ replies unless we push one directly.
    clock_pf.advance(Duration::from_millis(1));
    d_pf.poll_torque(clock_pf.now() + Duration::from_millis(10)).unwrap();
    // Push CTRL_MODE=4 read reply on expected reply ID 0x17 + fresh J7 feedback, then set max_frames = 0.
    let [lo, hi] = 7u16.to_le_bytes();
    let val = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
    let reply = CanFrame::new_standard(
        0x17,
        &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, val[0], val[1], val[2], val[3]],
    )
    .unwrap();
    d_pf.can_mut().rx.push_back((reply, None));
    d_pf.can_mut().push_all_feedback();
    d_pf.set_tx_budget(TxBudget {
        max_frames: 0,
        max_duration: Duration::from_millis(5),
        frame_max_age: Duration::from_millis(45),
    });
    let dl = clock_pf.now() + Duration::from_millis(10);
    let (ok, a, f) = counted(|| {
        let res = d_pf.poll_torque(dl);
        matches!(res, Err(ArmDriverError::PosForcePreEnableHoldBudgetExhausted { joint: 6 }))
    });
    assert!(ok, "expected PosForcePreEnableHoldBudgetExhausted {{ joint: 6 }}");
    assert_eq!((a, f), (0, 0), "pre-enable hold budget exhaustion touched allocator");
}

#[derive(Clone)]
struct SharedNoAllocBus(std::sync::Arc<std::sync::Mutex<NoAllocDamiaoBus>>);

impl SharedNoAllocBus {
    fn new(limits: &[DamiaoLimits]) -> Self {
        Self(std::sync::Arc::new(std::sync::Mutex::new(NoAllocDamiaoBus::new(limits))))
    }
}

impl CanTransport for SharedNoAllocBus {
    fn clock(&self) -> Option<std::sync::Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone())
    }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        self.0.lock().unwrap().transmit(frame)
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.0.lock().unwrap().receive()
    }
    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        self.0.lock().unwrap().receive_timestamped()
    }
    fn flush(&mut self) -> Result<(), TransportError> {
        self.0.lock().unwrap().flush()
    }
    fn supports_bounded_flush(&self) -> bool {
        true
    }
    fn flush_is_noop(&self) -> bool {
        true
    }
    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
}

#[test]
fn bounded_hold_and_zero_offset_paths_do_not_allocate_or_free() {
    let limits = b601_limits();
    let targets: Vec<JointTarget> = (0..7).map(|i| JointTarget::position(0.05 * i as f32)).collect();

    // 1. DamiaoArmDriver::write_estop_hold_bounded: Complete, partial Deferred (max_frames = 4),
    // and Expired (tx_step = 45 ms) — 0 allocs, 0 frees.
    let bus = NoAllocDamiaoBus::new(&limits);
    let mut driver = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.clone())
        .unwrap();
    driver.can_mut().push_all_feedback();
    driver.read_joint_states().unwrap();
    driver.set_torque(true).expect("enable");
    driver.set_rt_mode(true);
    // Warm up thread-local TX metrics once.
    assert_eq!(driver.write_estop_hold_bounded(&targets).unwrap(), TxBatchOutcome::Complete);
    reset_tx_metrics();

    // Complete
    driver.begin_tx_tick();
    let (res, a, f) = counted(|| driver.write_estop_hold_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert!(driver.last_hold_tx_sent_any());
    assert_eq!((a, f), (0, 0), "write_estop_hold_bounded Complete touched allocator");

    // Partial Deferred with max_frames = 4
    driver.set_tx_budget(TxBudget {
        max_frames: 4,
        max_duration: Duration::from_millis(5),
        frame_max_age: Duration::from_millis(45),
    });
    driver.begin_tx_tick();
    let (res, a, f) = counted(|| driver.write_estop_hold_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 3 });
    assert!(driver.last_hold_tx_sent_any());
    assert_eq!((a, f), (0, 0), "write_estop_hold_bounded Deferred touched allocator");

    // Drain carried-over hold frames cleanly before testing Expired.
    driver.set_tx_budget(TxBudget {
        max_frames: 16,
        max_duration: Duration::from_millis(100),
        frame_max_age: Duration::from_millis(45),
    });
    driver.begin_tx_tick();
    assert_eq!(driver.write_estop_hold_bounded(&targets).unwrap(), TxBatchOutcome::Complete);

    // Expired with tx_step = 45 ms (frame 0 sends and hits 45 ms age; remaining 6 expire in pass).
    driver.can_mut().tx_step = Duration::from_millis(45);
    driver.begin_tx_tick();
    let (res, a, f) = counted(|| driver.write_estop_hold_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Expired { dropped: 6, pending: 0 });
    assert!(driver.last_hold_tx_sent_any());
    assert_eq!((a, f), (0, 0), "write_estop_hold_bounded Expired touched allocator");

    // 2. ZeroOffsetDriver with nonzero offsets: warm up once, then verify Complete + Deferred for
    // both write_joint_targets_bounded and write_estop_hold_bounded — 0 allocs, 0 frees.
    let shared_bus = SharedNoAllocBus::new(&limits);
    let mut inner_driver = DamiaoArmDriver::new(shared_bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    shared_bus.0.lock().unwrap().push_all_feedback();
    inner_driver.read_joint_states().unwrap();
    inner_driver.set_torque(true).expect("enable");
    inner_driver.set_rt_mode(true);
    let mut wrapped = armd::joint_zero::ZeroOffsetDriver::new(
        Box::new(inner_driver),
        &[0.1, -0.05, 0.02, 0.0, 0.0, 0.0, 0.03],
    );
    let mut scratch = Vec::with_capacity(7);

    // Warm up once before counting (read_joint_states_into opens a fresh per-tick TX budget).
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    assert_eq!(wrapped.write_joint_targets_bounded(&targets).unwrap(), TxBatchOutcome::Complete);
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    assert_eq!(wrapped.write_estop_hold_bounded(&targets).unwrap(), TxBatchOutcome::Complete);
    reset_tx_metrics();

    // write_joint_targets_bounded: Complete
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    let (res, a, f) = counted(|| wrapped.write_joint_targets_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert_eq!((a, f), (0, 0), "ZeroOffsetDriver write_joint_targets_bounded Complete touched allocator");

    // write_joint_targets_bounded: Deferred
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    {
        let mut guard = shared_bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(3);
        guard.tx_count = 0;
    }
    let (res, a, f) = counted(|| wrapped.write_joint_targets_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 4 });
    assert_eq!((a, f), (0, 0), "ZeroOffsetDriver write_joint_targets_bounded Deferred touched allocator");

    // Drain target queue cleanly before hold tests.
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    {
        let mut guard = shared_bus.0.lock().unwrap();
        guard.mode = Mode::Ok;
        guard.tx_count = 0;
    }
    assert_eq!(wrapped.write_joint_targets_bounded(&targets).unwrap(), TxBatchOutcome::Complete);

    // write_estop_hold_bounded: Complete
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    let (res, a, f) = counted(|| wrapped.write_estop_hold_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Complete);
    assert!(wrapped.last_hold_tx_sent_any());
    assert_eq!((a, f), (0, 0), "ZeroOffsetDriver write_estop_hold_bounded Complete touched allocator");

    // write_estop_hold_bounded: Deferred
    wrapped.read_joint_states_into(&mut scratch).unwrap();
    {
        let mut guard = shared_bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(3);
        guard.tx_count = 0;
    }
    let (res, a, f) = counted(|| wrapped.write_estop_hold_bounded(&targets));
    assert_eq!(res.unwrap(), TxBatchOutcome::Deferred { pending: 4 });
    assert!(wrapped.last_hold_tx_sent_any());
    assert_eq!((a, f), (0, 0), "ZeroOffsetDriver write_estop_hold_bounded Deferred touched allocator");
}

#[test]
fn rt_release_deferral_can_tx_stalled_reclaim_saturation_and_gravity_tick_zero_alloc() {
    use armd::gravity_comp::GravityCompPhase;
    use armd::safety::{FaultCode, FaultReason, SafetyState, TxStallDetail, RETIRED_STATE_CAPACITY};

    let limits = b601_limits();
    let mk_rt_supervisor = || {
        let cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            joint_tracking: None,
            ..ArmConfig::rebot_dm()
        };
        let bus = SharedNoAllocBus::new(&limits);
        let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap();
        driver.check_mapping_ranges();
        driver.set_rt_mode(true);
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
        inner.lease_enabled = false;
        inner.tick(0.02);
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).expect("arm");
        inner.rt_executor_owned = true;
        inner.safety.set_off_thread_reclaim(true);
        for _ in 0..5 {
            let _ = inner.tick_publish(0.02, 1, false);
            let _ = inner.core.hub().latest();
        }
        reset_tx_metrics();
        (inner, bus)
    };

    let (mut inner, bus) = mk_rt_supervisor();

    // (a) Steady-state tick_publish(0.02, 1, false) (Complete): 0 allocs, 0 frees.
    let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    let _ = inner.core.hub().latest();
    assert_eq!((a, f), (0, 0), "steady-state tick_publish touched allocator");

    // (b) Transient Deferred tick_publish(0.02, 1, false) under Mode::EnobufsAfter(1):
    // must stay operational (!is_stopped()) with 0 allocs, 0 frees while J5..J7 accumulate
    // per-motor lack of progress (1, 2, 3 < 4 at 50 Hz) and latch on exact tick 4.
    {
        let mut guard = bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(1);
        guard.tx_count = 0;
    }
    let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    let _ = inner.core.hub().latest();
    assert!(!inner.safety.state().is_stopped(), "transient deferral must not fault: {:?}", inner.safety.state());
    assert_eq!((a, f), (0, 0), "transient Deferred tick_publish touched allocator");

    // Second and third transient Deferred ticks (streak = 2, 3 < 4 at 50 Hz): still operational, 0 allocs, 0 frees.
    for tick_idx in 2..=3 {
        bus.0.lock().unwrap().tx_count = 0;
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        let _ = inner.core.hub().latest();
        assert!(
            !inner.safety.state().is_stopped(),
            "deferred tick {tick_idx} (< min_tx_stall_ticks = 4 at 50 Hz) must stay operational: {:?}",
            inner.safety.state()
        );
        assert_eq!((a, f), (0, 0), "deferred tick {tick_idx} tick_publish touched allocator");
    }

    // (c) Stall-threshold tick (streak = 4 == ceil(45/20) + 1) latches CAN_TX_STALLED and publishes
    // non-empty CAN_TX_STALLED message with 0 allocs, 0 frees.
    bus.0.lock().unwrap().tx_count = 0;
    let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    assert_eq!((a, f), (0, 0), "CAN_TX_STALLED latch tick_publish touched allocator");
    assert_eq!(inner.safety.fault().map(|info| info.code), Some(FaultCode::CanTxStalled));
    let snap = inner.core.hub().latest().expect("published observation after stall");
    let obs_fault = snap.obs.fault.as_ref().expect("published fault info");
    assert_eq!(obs_fault.code, FaultCode::CanTxStalled);
    assert!(!obs_fault.message.is_empty(), "CAN_TX_STALLED observation message must be non-empty");
    drop(snap);

    // Subsequent stalled tick in Fault (write_estop_hold_bounded under EnobufsAfter(0)): 0 allocs, 0 frees.
    {
        let mut guard = bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(0);
        guard.tx_count = 0;
    }
    // Round-2 Item 2 zero-alloc check: empty software queue + fresh feedback + persistent ENOBUFS
    // -> RT executor clear_fault_executor_bounded runs TX probe, fails with ENOBUFS, 0 allocs/0 frees.
    {
        let dl = std::time::Instant::now() + Duration::from_millis(200);
        let (refused_by_probe, a, f) = counted(|| {
            matches!(
                inner.clear_fault_executor_bounded(dl),
                Err(armd::safety::ClearFaultError::Rejected(armd::safety::ClearRefusal::DriverError(_)))
            )
        });
        assert!(refused_by_probe, "ENOBUFS TX probe must refuse clear_fault_executor_bounded");
        assert_eq!((a, f), (0, 0), "ENOBUFS TX probe clear refusal touched allocator");
    }
    let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    let _ = inner.core.hub().latest();
    assert!(matches!(inner.safety.state(), SafetyState::Fault(FaultReason::CanTxStalled)));
    assert_eq!((a, f), (0, 0), "subsequent stalled hold tick_publish touched allocator");

    // Round-2 Item 2 zero-alloc check: when ENOBUFS clears and the queue drains, RT executor
    // clear_fault_executor_bounded sends the 0xCC TX probe, receives post-probe fresh feedback,
    // and clears CAN_TX_STALLED to Idle with 0 allocs / 0 frees.
    {
        let mut guard = bus.0.lock().unwrap();
        guard.mode = Mode::Ok;
        guard.tx_count = 0;
        guard.push_all_feedback();
    }
    let _ = inner.tick_publish(0.02, 1, false);
    let _ = inner.core.hub().latest();
    {
        let dl = std::time::Instant::now() + Duration::from_millis(200);
        let (cleared_to_idle, a, f) = counted(|| inner.clear_fault_executor_bounded(dl) == Ok(SafetyState::Idle));
        assert!(cleared_to_idle, "healthy TX probe must clear CAN_TX_STALLED to Idle");
        assert_eq!((a, f), (0, 0), "healthy TX probe clear_fault_executor_bounded touched allocator");
    }

    // (d) Off-thread reclamation saturation: fill RETIRED_STATE_CAPACITY (16) retired
    // fault transitions without draining, leave a non-cached Fault(Other) active, then latch
    // CanTxStalled via SafetyManager::latch inside counted -> records kind & detail and advances
    // epoch with 0 allocs / 0 frees without overflowing or dropping the retired fault ring.
    for _ in 0..RETIRED_STATE_CAPACITY {
        inner.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fill".into()));
        inner.safety.clear_fault_bounded().expect("retire fault arc");
    }
    inner.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("active-full".into()));
    assert!(inner.safety.retired_state_arcs_full());
    assert!(!inner.safety.can_accept_fault_transition());
    let stall_detail = TxStallDetail {
        queued: 5,
        dropped: 0,
        age_ms: 60,
        limit_ms: 45,
        consecutive_ticks: 3,
        is_hold: false,
    };
    let ep_before = inner.safety.epoch_id();
    let (ep_after, a, f) = counted(|| {
        inner.safety.latch(
            armd::safety::FaultKind::CanTxStalled,
            armd::safety::FaultDetail::CanTxStalled(Some(stall_detail)),
        )
    });
    assert_eq!(ep_after, ep_before + 1, "CanTxStalled latch must advance epoch_id");
    assert!(inner.safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled));
    assert_eq!(inner.safety.fault_code(), Some(FaultCode::CanTxStalled));
    assert_eq!(inner.safety.tx_stall_detail(), Some(stall_detail));
    assert_eq!((a, f), (0, 0), "saturated SafetyManager::latch(CanTxStalled) touched allocator");

    // (e) Active gravity-comp tick on a fresh warmed-up SupervisorInner with DamiaoArmDriver<SharedNoAllocBus>.
    let (mut gc_inner, _gc_bus) = mk_rt_supervisor();
    let stamp = gc_inner.current_admission_stamp();
    let admitted = gc_inner.check_admission(stamp).unwrap();
    SupervisorInner::enter_gravity_comp(admitted, Default::default()).expect("enter gravity comp");
    // Advance past ramp_in_s (1.0 s) and warm up one Active tick across all triple-buffer slots.
    for _ in 0..60 {
        let _ = gc_inner.tick_publish(0.02, 1, false);
        let _ = gc_inner.core.hub().latest();
    }
    assert!(matches!(
        gc_inner.gravity_comp.as_ref().map(|g| &g.phase),
        Some(GravityCompPhase::Active)
    ));
    let (published, a, f) = counted(|| gc_inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    let _ = gc_inner.core.hub().latest();
    assert!(gc_inner.gravity_comp.is_some());
    assert_eq!((a, f), (0, 0), "Active gravity-comp tick_publish touched allocator");

    // (f) Major 6A: Running jog action stalls into CAN_TX_STALLED (3 deferred ticks), terminates
    //     the action (`failed` / `CAN_TX_STALLED` with non-empty message) and publishes observation
    //     — whole release must do 0 allocs, 0 frees!
    let (mut jog_inner, jog_bus) = mk_rt_supervisor();
    let epoch = jog_inner.safety.epoch_id();
    let stamp = jog_inner.current_admission_stamp();
    let admitted = jog_inner.check_admission(stamp).unwrap();
    let (_, jog_id, _) = SupervisorInner::jog_action(
        admitted,
        vec![0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
        Some(1.0),
        Some(epoch),
        "joint_move",
    )
    .expect("start jog");
    // Warm up running action across all 3 triple-buffer slots before counting.
    for _ in 0..4 {
        let _ = jog_inner.tick_publish(0.02, 1, false);
        let _ = jog_inner.core.hub().latest();
    }
    assert_eq!(jog_inner.last_action.as_ref().map(|a| (a.id, a.state.as_str())), Some((jog_id, "running")));
    {
        let mut guard = jog_bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(1);
        guard.tx_count = 0;
    }
    for tick in 1..=4 {
        jog_bus.0.lock().unwrap().tx_count = 0;
        let (published, a, f) = counted(|| jog_inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!(
            (a, f),
            (0, 0),
            "running-jog deferral/stall tick {tick} touched allocator"
        );
    }
    assert_eq!(jog_inner.safety.fault_code(), Some(FaultCode::CanTxStalled));
    let snap = jog_inner.core.hub().latest().expect("published observation after jog stall");
    let act = snap.obs.action.as_ref().expect("published action after jog stall");
    assert_eq!(act.id, jog_id);
    assert_eq!(act.state.as_str(), "failed");
    assert_eq!(act.code.as_deref(), Some("CAN_TX_STALLED"));
    assert!(
        act.message.as_deref().is_some_and(|m| m.contains("CAN TX stalled")),
        "expected CAN TX stalled action message, got {:?}",
        act.message
    );
    drop(snap);

    // (g) Major 6B: Escalating a stopped hold from an existing heap-backed `Fault(Other(String))`
    //     to `Fault(CanTxStalled)` retires the old state arc and recycles `safety_reason` spare
    //     with 0 allocs, 0 frees across the whole release.
    let (mut esc_inner, esc_bus) = mk_rt_supervisor();
    esc_inner
        .safety
        .trigger_fault_coded(FaultCode::Other, FaultReason::Other("existing heap-backed fault".into()));
    // Warm up the existing heap-backed Fault(Other) across all 3 triple-buffer slots.
    for _ in 0..4 {
        let _ = esc_inner.tick_publish(0.02, 1, false);
        let _ = esc_inner.core.hub().latest();
    }
    {
        let mut guard = esc_bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(0);
        guard.tx_count = 0;
    }
    // Advance clock >= 45 ms across ticks so stopped hold escalates from Fault(Other) -> CanTxStalled.
    for tick in 0..4 {
        {
            let mut guard = esc_bus.0.lock().unwrap();
            guard.clock.advance(Duration::from_millis(20));
            guard.push_all_feedback();
            guard.tx_count = 0;
        }
        let (published, a, f) = counted(|| esc_inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!(
            (a, f),
            (0, 0),
            "Fault(Other) -> CanTxStalled stopped-hold escalation tick {tick} touched allocator"
        );
    }
    assert_eq!(esc_inner.safety.fault_code(), Some(FaultCode::CanTxStalled));
    assert!(
        esc_inner.safety.retired_state_arcs_len() >= 1,
        "prior heap-backed Fault(Other) state arc must be retired off-thread"
    );
    let esc_snap = esc_inner.core.hub().latest().expect("published observation after escalation");
    assert_eq!(esc_snap.obs.safety_state, SafetyState::Fault(FaultReason::CanTxStalled));
    assert_eq!(esc_snap.obs.fault.as_ref().map(|f| f.code), Some(FaultCode::CanTxStalled));
    drop(esc_snap);

    // (h) Major 7: Park `unload_settle` populated with accepted stationary samples followed by a
    //     transient `Deferred` output-gap release clearing `settle.samples` — 0 allocs, 0 frees!
    let (mut pk_inner, pk_bus) = mk_rt_supervisor();
    pk_inner.pending_normal_unload = true;
    // Populate settle history and warm all 3 triple-buffer slots.
    for _ in 0..6 {
        let _ = pk_inner.tick_publish(0.02, 1, false);
        let _ = pk_inner.core.hub().latest();
    }
    assert!(
        pk_inner.unload_settle.as_ref().is_some_and(|s| !s.samples.is_empty()),
        "unload_settle should hold accepted stationary samples before the output gap"
    );
    {
        let mut guard = pk_bus.0.lock().unwrap();
        guard.mode = Mode::EnobufsAfter(2);
        guard.tx_count = 0;
    }
    let (published, a, f) = counted(|| pk_inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    assert_eq!(
        (a, f),
        (0, 0),
        "park unload_settle Deferred output-gap release touched allocator"
    );
    let settle = pk_inner.unload_settle.as_ref().expect("settle remains active after transient gap");
    assert_eq!(settle.held_s, 0.0);
    assert!(settle.samples.is_empty(), "output gap must clear settle samples without freeing");
}

/// Item 1 + Item 3 + Item 4 RT zero-alloc/zero-free verification:
/// - Full RT release across transient J7 staleness (skip + `0xCC` refresh), sustained J7
///   staleness latching `J7_FEEDBACK_STALE`, `halt_motion_for_fault_rt(6)`, `reconcile_action`
///   terminating a real running RT `MacroRun` (retiring it off-thread without freeing on RT),
///   and `tick_publish`: strictly 0 allocs, 0 frees.
/// - Subsequent rate-limited `"Stopped hold output failed"` (`SafetyLogEvent::StoppedHoldOutputFailed`)
///   emission across 1.1 s (55 ticks: emit 0 suppressed -> 49 suppressed -> emit 49 suppressed):
///   strictly 0 allocs, 0 frees on every control release.
#[test]
fn rt_j7_pos_force_staleness_macro_retire_and_throttled_hold_error_zero_alloc() {
    use armd::safety::{FaultCode, FaultReason, SafetyLogEvent, SafetyState};

    let limits = b601_limits();
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        joint_tracking: None,
        ..ArmConfig::rebot_dm()
    };
    let bus = SharedNoAllocBus::new(&limits);
    bus.0.lock().unwrap().motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
    let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap();
    driver.read_joint_states().unwrap();
    driver.read_joint_states().unwrap();
    driver.check_mapping_ranges();
    driver.set_rt_mode(true);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.tick(0.02);
    {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit arm");
        SupervisorInner::enable_torque(admitted).expect("arm pos-force");
    }
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);

    // Start a real running RT macro (`MacroRun`) so its retirement on J7_FEEDBACK_STALE is
    // proven zero-alloc and zero-free on the control thread.
    let ep = inner.safety.epoch_id();
    let macro_id = {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit macro");
        SupervisorInner::begin_sequence_action(admitted, Some("rt-j7-macro".into()), ep)
            .expect("begin macro")
    };
    assert!(inner.macro_run.is_some());

    // Warm up triple-buffer observation slots with running macro before counting.
    for _ in 0..5 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    inner.safety.drain_log_events_off_thread();
    reset_tx_metrics();

    // 1. Mute J7 and advance 20 ms per tick across transient skip/0xCC ticks (120..240 ms)
    //    and through the 250 ms latch tick (260 ms): strictly 0 allocs, 0 frees on EVERY tick!
    bus.0.lock().unwrap().mute_j7 = true;
    let mut latched_on_tick = None;
    for tick in 1..=16 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!(
            (a, f),
            (0, 0),
            "J7 stale skip/refresh/latch tick {tick} touched allocator"
        );
        if inner.safety.fault_code() == Some(FaultCode::J7FeedbackStale) && latched_on_tick.is_none() {
            latched_on_tick = Some(tick);
            break;
        }
    }
    assert!(latched_on_tick.is_some(), "expected J7_FEEDBACK_STALE to latch within 16 ticks");
    assert_eq!(*inner.safety.state(), SafetyState::Fault(FaultReason::J7FeedbackStale));
    assert!(inner.macro_run.is_none(), "MacroRun must be removed from active slot");
    assert_eq!(
        inner.retired_macro_runs_len(),
        1,
        "MacroRun must be queued in retired_macro_runs for off-thread drop (0 free on RT)"
    );
    let snap = inner.core.hub().latest().expect("published observation after J7_FEEDBACK_STALE");
    let act = snap.obs.action.as_ref().expect("published action after J7_FEEDBACK_STALE");
    assert_eq!(act.id, macro_id);
    assert_eq!(act.state.as_str(), "failed");
    assert_eq!(act.code.as_deref(), Some("J7_FEEDBACK_STALE"));
    drop(snap);
    // Reclaim off-thread (outside counted).
    assert!(inner.pop_retired_macro_run().is_some());
    inner.safety.drain_log_events_off_thread();

    // 2. Rate-limited `"Stopped hold output failed"` via deferred safety log ring across 55 ticks
    //    (1.1 s on driver clock): unmute J7 so feedback remains fresh (< 500 ms BUS_LOSS) while
    //    every stopped-hold TX returns Mode::HardError — strictly 0 allocs, 0 frees, exact counts!
    {
        let mut g = bus.0.lock().unwrap();
        g.mute_j7 = false;
        g.mode = Mode::HardError;
    }
    let mut hold_log_events = Vec::new();
    for tick in 0..55 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
            g.tx_count = 0;
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!(
            (a, f),
            (0, 0),
            "rate-limited stopped hold error tick {tick} touched allocator"
        );
        while let Some(ev) = inner.safety.pop_log_event() {
            if let SafetyLogEvent::StoppedHoldOutputFailed { partial, suppressed, .. } = ev {
                hold_log_events.push((partial, suppressed));
            }
        }
    }
    assert_eq!(
        hold_log_events,
        vec![(false, 0), (false, 49)],
        "expected exactly 2 deferred StoppedHoldOutputFailed events across 1.1 s with exact suppressed counts"
    );
    assert_eq!(inner.hold_tx_error_suppressed(), 4);
    assert_eq!(
        inner.safety.fault_code(),
        Some(FaultCode::CanTxStalled),
        "sustained hard-error stopped-hold streak with live RX must escalate to CAN_TX_STALLED zero-alloc"
    );
}

/// Major 4 RT zero-alloc/zero-free regression:
/// Saturating `retired_macro_runs` guarantees the final admitted heap-backed `MacroRun` retires
/// on `armd-control` with strictly 0 allocs and 0 frees, and subsequent macro admissions apply
/// backpressure (`MotionError::MotionBusy`) until an off-thread drain frees a retirement slot.
#[test]
fn rt_macro_retirement_saturation_applies_admission_backpressure_with_zero_alloc_and_zero_free() {
    use armd::macro_action::SubState;
    use armd::motion::MotionError;
    use armd::safety::FaultCode;
    use armd::supervisor::ArmError;

    let limits = b601_limits();
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        joint_tracking: None,
        ..ArmConfig::rebot_dm()
    };
    let bus = SharedNoAllocBus::new(&limits);
    bus.0.lock().unwrap().motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
    let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap();
    driver.read_joint_states().unwrap();
    driver.read_joint_states().unwrap();
    driver.check_mapping_ranges();
    driver.set_rt_mode(true);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.tick(0.02);
    {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit arm");
        SupervisorInner::enable_torque(admitted).expect("arm pos-force");
    }
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);

    // Fill 7 of 8 `retired_macro_runs` slots via real admission + macro stop (no off-thread drain).
    for i in 0..7 {
        let ep = inner.safety.epoch_id();
        let _id = {
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit fill");
            SupervisorInner::begin_sequence_action(admitted, Some(format!("fill-{i}")), ep)
                .expect("admit while slots available")
        };
        if let Some(m) = inner.macro_run.as_mut() {
            m.phase = Some("approach-heap-phase".into());
            m.sub = SubState::Failed {
                code: "TEST_SUB_CODE".into(),
                message: "heap-backed sub-state message".into(),
            };
        }
        inner.request_macro_stop();
    }
    assert_eq!(inner.retired_macro_runs_len(), 7);
    assert!(inner.can_admit_macro_run(), "1 free slot must still admit 1 macro");

    // Admit the 8th (final-capacity) heap-backed MacroRun and warm triple-buffer slots.
    let ep = inner.safety.epoch_id();
    let _last_id = {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit 8th");
        SupervisorInner::begin_sequence_action(admitted, Some("slot-8-macro".into()), ep)
            .expect("admit 8th macro")
    };
    // Expire the legacy-populated `previous_action` (with `phase_ms` BTreeMap from `request_macro_stop`)
    // before measuring RT control-thread allocs/frees for the 8th macro's retirement.
    inner.previous_action = None;
    if let Some(m) = inner.macro_run.as_mut() {
        m.phase = Some("grasp-descend-heap-phase".into());
        m.sub = SubState::Failed {
            code: "HEAP_SUB_CODE".into(),
            message: "must not be freed on armd-control when retiring into final slot".into(),
        };
    }
    assert!(!inner.can_admit_macro_run(), "active macro + 7 retired == capacity 8 -> no more room");

    for _ in 0..5 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    inner.safety.drain_log_events_off_thread();

    // Trigger J7_FEEDBACK_STALE so the 8th heap-backed MacroRun retires on the RT control thread
    // into slot 8/8: MUST be strictly (0 allocs, 0 frees) on EVERY tick!
    bus.0.lock().unwrap().mute_j7 = true;
    let mut latched = false;
    for tick in 1..=16 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!(
            (a, f),
            (0, 0),
            "saturated final-slot macro retirement tick {tick} touched allocator"
        );
        if inner.safety.fault_code() == Some(FaultCode::J7FeedbackStale) {
            latched = true;
            break;
        }
    }
    assert!(latched);
    assert!(inner.macro_run.is_none());
    assert_eq!(inner.retired_macro_runs_len(), 8);

    // Recover & re-arm WITHOUT draining `retired_macro_runs`: 9th admission MUST backpressure!
    {
        let mut g = bus.0.lock().unwrap();
        g.mute_j7 = false;
        g.push_all_feedback();
    }
    let _ = inner.tick_publish(0.02, 1, false);
    {
        let mut g = bus.0.lock().unwrap();
        g.clock.advance(Duration::from_millis(20));
        g.push_all_feedback();
    }
    let _ = inner.tick_publish(0.02, 1, false);
    inner.clear_fault().expect("clear J7 fault");
    {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit re-arm");
        SupervisorInner::enable_torque(admitted).expect("re-arm");
    }
    assert!(!inner.can_admit_macro_run());
    let ep_full = inner.safety.epoch_id();
    let err = {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit overflow");
        SupervisorInner::begin_sequence_action(admitted, Some("overflow-macro".into()), ep_full)
            .expect_err("saturated retired_macro_runs must reject macro admission")
    };
    assert!(
        matches!(err, ArmError::Motion(MotionError::MotionBusy(_))),
        "expected MotionBusy backpressure, got {err:?}"
    );
    assert!(inner.macro_run.is_none());

    // Drain 1 retired MacroRun off-thread -> admission immediately succeeds again!
    let _reclaimed = inner.pop_retired_macro_run().expect("pop 1 retired macro");
    assert_eq!(inner.retired_macro_runs_len(), 7);
    assert!(inner.can_admit_macro_run());
    let ep_ok = inner.safety.epoch_id();
    let _ = {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit post-drain");
        SupervisorInner::begin_sequence_action(admitted, Some("post-drain-macro".into()), ep_ok)
            .expect("admission must resume once a slot drains")
    };
}

/// R2 Blocker 2 + R2 Major 1 + R2 Major 3 RT zero-alloc/zero-free regression:
/// 1. Full RT release (`tick_publish`) latching `J7_FEEDBACK_STALE` with completely exhausted
///    `SafetyManager` fault pools AND then upgrading to `ThermalTrip` with pools still exhausted:
///    strictly (0 allocs, 0 frees) AND all 3 fault views (`state()`, `fault_code()`, `fault()`)
///    resolve the winning `ThermalTrip` (never leaving stale `Fault(J7FeedbackStale)` in `state()`).
/// 2. Counted `(0, 0)` alloc/free verification on BOTH refused (`J7FeedbackStale`) and successful
///    (`Ok(Idle)`) `clear_fault_executor_bounded` paths.
/// 3. Counted `(0, 0)` alloc/free verification on the production `forward_deferred_safety_logs`
///    path across full downstream `rtrb` ring rejection, delayed recovery at `t + 1.2 s`, and
///    throttled follow-up at `t + 1.22 s`.
#[test]
fn rt_j7_exhausted_pool_latch_bounded_clear_and_production_log_forwarding_zero_alloc() {
    use armd::rt_core::executor::{forward_deferred_safety_logs, ExecutorStats};
    use armd::safety::{
        ClearFaultError, ClearRefusal, FaultCode, FaultReason, SafetyLogEvent, SafetyState,
    };

    let limits = b601_limits();
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        joint_tracking: None,
        resting_pose: None,
        ..ArmConfig::rebot_dm()
    };
    cfg.max_temperature = 75.0;
    let bus = SharedNoAllocBus::new(&limits);
    bus.0.lock().unwrap().motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
    let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap();
    driver.read_joint_states().unwrap();
    driver.read_joint_states().unwrap();
    driver.check_mapping_ranges();
    driver.set_rt_mode(true);

    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.tick(0.02);
    {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit arm");
        SupervisorInner::enable_torque(admitted).expect("arm pos-force");
    }
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);

    for _ in 0..4 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    inner.safety.drain_log_events_off_thread();

    // 1. Exhaust ALL SafetyManager fault pools, then latch J7_FEEDBACK_STALE via live RT ticks.
    let held_pools = inner.safety.drain_fault_arc_pool_for_test();
    bus.0.lock().unwrap().mute_j7 = true;
    let mut j7_latched = false;
    for tick in 1..=16 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "empty-pool J7 latch tick {tick} touched allocator");
        if inner.safety.fault_code() == Some(FaultCode::J7FeedbackStale) {
            j7_latched = true;
            break;
        }
    }
    assert!(j7_latched);
    assert_eq!(*inner.safety.state(), SafetyState::Fault(FaultReason::J7FeedbackStale));

    // With pools STILL exhausted, latch higher-severity ThermalTrip:
    // MUST be (0, 0) and follow the baseline pool-exhaustion contract (`test_rt_b3_alloc.rs:680`):
    // existing stopped `state_arc` stays pinned without allocating while `fault_code()`,
    // `fault()`, and `recoverable()` reflect the higher-severity `ThermalTrip`.
    {
        let mut g = bus.0.lock().unwrap();
        g.clock.advance(Duration::from_millis(20));
        g.push_all_feedback();
    }
    inner.temperatures[0] = 90.0;
    let ((), a, f) = counted(|| {
        inner.safety.latch(
            armd::safety::FaultKind::ThermalTrip,
            armd::safety::FaultDetail::ThermalTrip { joint_idx: 0, temperature: 90.0, limit: 75.0 },
        );
    });
    assert_eq!((a, f), (0, 0), "empty-pool ThermalTrip upgrade over J7 touched allocator");
    assert_eq!(*inner.safety.state(), SafetyState::Fault(FaultReason::J7FeedbackStale));
    assert!(inner.safety.has_latched_fault_kind(armd::safety::FaultKind::J7FeedbackStale));
    assert!(inner.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip));
    assert_eq!(inner.safety.fault_code(), Some(FaultCode::ThermalTrip));
    assert_eq!(inner.safety.fault().unwrap().code, FaultCode::ThermalTrip);
    assert!(!inner.safety.recoverable());
    assert!(!inner.safety.fault().unwrap().recoverable);
    drop(held_pools);

    // 2. Counted (0, 0) alloc/free on BOTH refused and successful `clear_fault_executor_bounded`
    //    for `J7_FEEDBACK_STALE`.
    let bus2 = SharedNoAllocBus::new(&b601_limits());
    bus2.0.lock().unwrap().motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
    let grip2 = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
    let mut driver2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits())
        .unwrap()
        .with_gripper_drive(grip2)
        .unwrap();
    driver2.read_joint_states().unwrap();
    driver2.read_joint_states().unwrap();
    driver2.check_mapping_ranges();
    driver2.set_rt_mode(true);
    let mut inner2 = SupervisorInner::new(cfg).with_driver(Box::new(driver2));
    inner2.lease_enabled = false;
    inner2.tick(0.02);
    {
        let stamp = inner2.current_admission_stamp();
        let admitted = inner2.check_admission(stamp).expect("admit arm");
        SupervisorInner::enable_torque(admitted).expect("arm");
    }
    inner2.rt_executor_owned = true;
    inner2.safety.set_off_thread_reclaim(true);
    bus2.0.lock().unwrap().mute_j7 = true;
    for _ in 0..16 {
        {
            let mut g = bus2.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner2.tick_publish(0.02, 1, false);
        if inner2.safety.fault_code() == Some(FaultCode::J7FeedbackStale) {
            break;
        }
    }
    assert_eq!(inner2.safety.fault_code(), Some(FaultCode::J7FeedbackStale));
    // Refused bounded clear while J7 is still stale -> (0, 0) allocs/frees!
    let dl_refuse = std::time::Instant::now() + Duration::from_millis(50);
    let (refused_j7, a, f) = counted(|| {
        matches!(
            inner2.clear_fault_executor_bounded(dl_refuse),
            Err(ClearFaultError::Rejected(ClearRefusal::J7FeedbackStale))
        )
    });
    assert_eq!((a, f), (0, 0), "refused J7 clear_fault_executor_bounded touched allocator");
    assert!(refused_j7, "expected ClearRefusal::J7FeedbackStale");

    // Restore fresh J7 feedback across empty RX boundary -> successful bounded clear -> (0, 0)!
    for _ in 0..2 {
        {
            let mut g = bus2.0.lock().unwrap();
            g.mute_j7 = false;
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner2.tick_publish(0.02, 1, false);
    }
    let dl_ok = std::time::Instant::now() + Duration::from_millis(50);
    let (cleared_idle, a, f) = counted(|| {
        matches!(inner2.clear_fault_executor_bounded(dl_ok), Ok(SafetyState::Idle))
    });
    assert_eq!((a, f), (0, 0), "successful J7 clear_fault_executor_bounded touched allocator");
    assert!(cleared_idle, "expected Ok(SafetyState::Idle) after fresh J7 feedback");

    // 3. Counted (0, 0) alloc/free on the production `forward_deferred_safety_logs` forwarding
    //    path across downstream ring saturation, delayed retry at t + 1.2 s, and next-tick throttle.
    inner2.safety.latch(
        armd::safety::FaultKind::J7FeedbackStale,
        armd::safety::FaultDetail::J7FeedbackStale(None),
    );
    inner2.safety.drain_log_events_off_thread();
    bus2.0.lock().unwrap().mode = Mode::HardError;
    let (mut log_tx, mut log_rx) = rtrb::RingBuffer::<SafetyLogEvent>::new(2);
    log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
    log_tx.push(SafetyLogEvent::GravityCompRampComplete).unwrap();
    let stats = ExecutorStats::default();

    // Tick at t0: hold error hits full downstream `log_tx` -> rejected, (0, 0) allocs/frees!
    let t0 = {
        let mut g = bus2.0.lock().unwrap();
        g.clock.advance(Duration::from_millis(20));
        g.push_all_feedback();
        g.clock.now()
    };
    let (published, a, f) = counted(|| {
        let p = inner2.tick_publish(0.02, 1, false);
        let _ = forward_deferred_safety_logs(&mut inner2, &mut log_tx, &stats, t0);
        p
    });
    assert!(published.kept.is_none());
    drop(published);
    assert_eq!((a, f), (0, 0), "downstream-full rejection in production forwarding touched allocator");
    assert_eq!(inner2.hold_tx_error_suppressed(), 1);

    // Consumer drains `log_rx` 1.2 s later with no intervening error -> retry delivers with (0, 0)!
    assert!(log_rx.pop().is_ok());
    assert!(log_rx.pop().is_ok());
    let t_recover = {
        let g = bus2.0.lock().unwrap();
        g.clock.advance(Duration::from_millis(1200));
        g.clock.now()
    };
    let (did_forward, a, f) = counted(|| {
        forward_deferred_safety_logs(&mut inner2, &mut log_tx, &stats, t_recover)
    });
    assert!(did_forward);
    assert_eq!((a, f), (0, 0), "delayed retry in production forwarding touched allocator");
    assert_eq!(inner2.hold_tx_error_suppressed(), 0);
    match log_rx.pop().expect("retry delivered") {
        SafetyLogEvent::StoppedHoldOutputFailed { suppressed, logged_at, .. } => {
            assert_eq!(suppressed, 0);
            assert_eq!(logged_at, t0);
        }
        other => panic!("unexpected event: {other:?}"),
    }

    // Immediate next hold error 20 ms after retry acceptance -> MUST be throttled with (0, 0)!
    let t_next = {
        let mut g = bus2.0.lock().unwrap();
        g.clock.advance(Duration::from_millis(20));
        g.push_all_feedback();
        g.clock.now()
    };
    let (published, a, f) = counted(|| {
        let p = inner2.tick_publish(0.02, 1, false);
        let _ = forward_deferred_safety_logs(&mut inner2, &mut log_tx, &stats, t_next);
        p
    });
    assert!(published.kept.is_none());
    drop(published);
    assert_eq!((a, f), (0, 0), "post-retry throttled tick touched allocator");
    // Hard-error streak past 45 ms may emit a one-shot CAN_TX_STALLED escalation event,
    // while `StoppedHoldOutputFailed` itself MUST remain throttled for 1 s after retry acceptance!
    while let Ok(ev) = log_rx.pop() {
        assert!(
            !matches!(ev, SafetyLogEvent::StoppedHoldOutputFailed { .. }),
            "StoppedHoldOutputFailed 20 ms after delayed retry acceptance must stay throttled, got {ev:?}"
        );
    }
    assert_eq!(inner2.hold_tx_error_suppressed(), 1);
}

/// Round 5 zero-allocation proof on `armd-control` covering every new Round-5 path:
/// - **B1**: `MitOwner::Lost` stopped-hold emitting J7 freeze/hold via `write_pos_force_stopped_hold_only()` -> (0, 0).
/// - **M1**: decode-time live freshness predicate + freeze frame emission -> (0, 0).
/// - **M2**: stale J1 (`BuildRefused`) + J2 `WouldBlock` streak -> `CAN_TX_STALLED` -> (0, 0).
/// - **M3**: running `gripper=true` replay terminating with `J7_FEEDBACK_STALE` on stale J7 -> (0, 0).
#[test]
fn rt_round5_b1_m1_m2_m3_paths_zero_alloc_and_zero_free() {
    use armd::mit::MitOwner;
    use armd::recording::{EpisodeStore, RecordFrame};
    use armd::replay::{plan_frames, ReplayInput, ReplayRequest};
    use armd::safety::{FaultCode, FaultReason};

    let limits = b601_limits();
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        joint_tracking: None,
        ..ArmConfig::rebot_dm()
    };
    let bus = SharedNoAllocBus::new(&limits);
    bus.0.lock().unwrap().motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
    bus.0.lock().unwrap().motors[6].2 = -2.5;
    let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap();
    driver.read_joint_states().unwrap();
    driver.read_joint_states().unwrap();
    driver.check_mapping_ranges();
    driver.set_rt_mode(true);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.tick(0.02);
    {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit arm");
        SupervisorInner::enable_torque(admitted).expect("arm pos-force");
    }
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);

    // Pre-warm observation triple buffer and start a `gripper=true` replay for M3.
    let dir = tempfile::tempdir().unwrap();
    let store = EpisodeStore::new(Some(dir.path().to_path_buf()));
    for _ in 0..4 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    let q0 = inner.commanded_positions.clone();
    let mut q1 = q0.clone();
    q1[6] = 0.0;
    let frames = vec![
        RecordFrame { timestamp: 0.0, observation_state: q0.clone(), action: q0.clone() },
        RecordFrame { timestamp: 0.2, observation_state: q0.clone(), action: q0.clone() },
        RecordFrame { timestamp: 3.0, observation_state: q1.clone(), action: q1 },
    ];
    let plan = plan_frames(
        ReplayInput {
            request: ReplayRequest {
                episode: "r5_alloc_ep".into(),
                speed: 1.0,
                gripper: true,
                tag: None,
                allow_stale: false,
            },
            path: dir.path().join("r5_alloc_ep.parquet"),
            profile_id: inner.config.profile_id.clone(),
            dof: inner.config.dof,
            arm_dof: inner.config.arm_dof(),
            gripper_idx: inner.config.gripper_index(),
            joint_names: inner.config.limits.iter().map(|l| l.name.clone()).collect(),
            limits: inner.config.limits.clone(),
            speed_limits: inner.config.limits.clone(),
            ws: inner.workspace_policy().map(|(ws, cam)| (ws.clone(), cam.clone())),
            arm: inner.config.arm_model.clone(),
            start: q0,
            control_hz: inner.config.control_frequency_hz,
            epoch: inner.safety.epoch_id(),
            workspace_revision: inner.workspace_revision,
            power: None,
            store,
            zero_offsets: inner.joint_zero_offsets.clone(),
            tcp_offset: inner.config.tcp_offset,
            active_tool: inner.config.active_tool.clone(),
        },
        &frames,
    )
    .expect("plan replay");
    let (replay_id, _) = {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit replay");
        SupervisorInner::install_replay(admitted, plan).expect("install replay")
    };
    for _ in 0..3 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    inner.safety.drain_log_events_off_thread();
    reset_tx_metrics();

    // 1. M3 + M1: J7 goes stale (> 100 ms) while `gripper=true` replay is running -> terminates
    //    with `J7_FEEDBACK_STALE` + retires trajectory off-thread + emits freeze frame with (0, 0)!
    {
        let mut g = bus.0.lock().unwrap();
        g.rx.clear();
        g.pending_rx.clear();
        g.mute_j7 = true;
        g.clock.advance(Duration::from_millis(115));
        g.push_all_feedback();
    }
    let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(published.kept.is_none());
    drop(published);
    assert_eq!((a, f), (0, 0), "M3 running gripper=true replay stale termination touched allocator");
    assert_eq!(inner.last_action.as_ref().map(|a| a.id), Some(replay_id));
    assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("J7_FEEDBACK_STALE"));
    while let Some(retired) = inner.pop_retired_trajectory() {
        drop(retired);
    }

    // 2. B1: `MitOwner::Lost` stopped-hold emits J7 freeze frame via `write_pos_force_stopped_hold_only()`
    //    with strictly (0, 0) allocs/frees!
    inner.mit.owner = MitOwner::Lost;
    inner.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::Other("B1 alloc test".into()));
    inner.safety.drain_log_events_off_thread();
    for tick in 0..3 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "B1 MitOwner::Lost J7 freeze tick {tick} touched allocator");
    }

    // 3. M2: Stale J1 (`BuildRefused`) + J2 `WouldBlock` streak -> `CAN_TX_STALLED` with (0, 0)!
    inner.mit.owner = MitOwner::Off;
    {
        let mut g = bus.0.lock().unwrap();
        g.mute_j7 = false;
        g.mute_j1 = true;
        g.rx.clear();
        g.pending_rx.clear();
        g.clock.advance(Duration::from_millis(120));
        g.push_all_feedback();
    }
    let _ = inner.tick_publish(0.02, 1, false);
    inner.safety.drain_log_events_off_thread();
    bus.0.lock().unwrap().mode = Mode::WouldBlockAfter(0);
    let mut stalled = false;
    for tick in 0..6 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "M2 stale J1 + WouldBlock tick {tick} touched allocator");
        if inner.safety.has_latched_fault_kind(armd::safety::FaultKind::CanTxStalled) {
            stalled = true;
            break;
        }
    }
    assert!(stalled, "M2 stale J1 + WouldBlock must latch CAN_TX_STALLED");
    assert_eq!(inner.hold_tx_no_progress_ticks()[0], 0);
}

/// R2 + R1 zero-allocation proof on `armd-control`:
/// Verifies strictly `(0, 0)` allocs and frees across EVERY RT stale-J7 rejection path and the
/// unified R1 J7 tick-epilogue emitter:
/// 1. R1 tick epilogue (`emit_pos_force_j7`): fresh Target, fresh Hold/Stop, and stale Freeze;
/// 2. `consume_target_frame` / `consume_policy_target` -> `policy_step_slice` while J7 is stale
///    (`streams.rs:1886, 1251`) -> `Refusal::code("J7_FEEDBACK_STALE")`, in-place policy obs update,
///    preallocated action message, off-thread `PolicyRun` retirement, and `tick_publish`;
/// 3. `commit_prepared_slot` standalone `gripper=true` replay while J7 is stale (`ops.rs:918`);
/// 4. `commit_prepared` standalone `gripper=true` replay while J7 is stale (`ops.rs:860`);
/// 5. `commit_primitive` `JointQuintic` moving J7 while J7 is stale (`ops.rs:809` -> `motion.rs:1051`);
/// 6. Macro children while J7 is stale:
///    - `commit_prepared_slot` child `gripper=true` replay (`ops.rs:924`),
///    - `commit_prepared` child `gripper=true` replay (`ops.rs:864`),
///    - `commit_primitive` child `Gripper`, `JointQuinticLimited` moving J7, and `Replay { gripper: true }`;
/// 7. `reconcile_action` terminating a live `PolicyRun` when `J7_FEEDBACK_STALE` latches at 250 ms.
#[test]
fn rt_r2_stale_j7_refusals_and_r1_epilogue_zero_alloc_and_zero_free() {
    use armd::rt_core::broker::{
        plan_slots, ActionId, Authority, BootId, ClientId, CommandKind, Descriptor, RequestId, Stamp,
    };
    use armd::rt_core::ids::ArmId;
    use armd::rt_core::mailbox::{Refusal, TargetFrame, MAX_DOF};
    use armd::rt_core::ops::{
        tool_digest_hash, ChildSpec, ChildTicket, CommitCtx, CoreOps, EpisodeRef, PathKind,
        PlanSlot, Primitive,
    };
    use armd::safety::FaultCode;

    let limits = b601_limits();
    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        joint_tracking: None,
        ..ArmConfig::rebot_dm()
    };
    let bus = SharedNoAllocBus::new(&limits);
    bus.0.lock().unwrap().motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
    bus.0.lock().unwrap().motors[6].2 = -2.5;
    let grip = armd::driver::gripper_drive_from_profile(&cfg).expect("pos-force drive");
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap();
    driver.read_joint_states().unwrap();
    driver.read_joint_states().unwrap();
    driver.check_mapping_ranges();
    driver.set_rt_mode(true);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.register_live_connection(1).unwrap();
    inner.tick(0.02);
    {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit arm");
        SupervisorInner::enable_torque(admitted).expect("arm pos-force");
    }
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);
    inner.prewarm_rt_buffers();
    for _ in 0..4 {
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();
    }
    inner.safety.drain_log_events_off_thread();

    // 1. R1 tick epilogue: fresh Target motion tick -> (0, 0).
    {
        let mut g = bus.0.lock().unwrap();
        g.clock.advance(Duration::from_millis(20));
        g.push_all_feedback();
    }
    let (pub_fresh, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
    assert!(pub_fresh.kept.is_none());
    drop(pub_fresh);
    assert_eq!((a, f), (0, 0), "R1 fresh tick epilogue touched allocator");
    let _ = inner.core.hub().latest();

    // 2. Policy target refusal on `consume_target_frame` (`streams.rs:1886, 1251`) while J7 stale.
    #[cfg(feature = "policy-runtime")]
    {
        use armd::policy_runtime::{PolicyMode, PolicyRunStart};
        inner.follower = Some("sim".into());
        inner.lease_enabled = true;
        let lease_gen = inner.acquire_lease("r2-policy-conn").expect("lease");
        let ep_pol = inner.safety.epoch_id();
        let pol_id = {
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit policy");
            SupervisorInner::begin_policy_run(
                admitted,
                PolicyRunStart {
                    epoch_id: ep_pol,
                    connection_id: "r2-policy-conn".into(),
                    lease_generation: lease_gen,
                    policy_id: "r2-pol".into(),
                    artifact_digest: "0".repeat(64),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: None,
                },
            )
            .expect("begin live policy")
        };
        // Warm up 1 accepted policy step + triple-buffer slots so steady-state buffers are primed.
        let mut q_step = [0.0f32; MAX_DOF];
        q_step[..7].copy_from_slice(&inner.commanded_positions);
        inner.policy_step_slice(pol_id, &q_step[..7], Some(5), None, None).expect("warm step");
        for _ in 0..4 {
            {
                let mut g = bus.0.lock().unwrap();
                g.clock.advance(Duration::from_millis(20));
                g.push_all_feedback();
            }
            let _ = inner.tick_publish(0.02, 1, false);
            let _ = inner.core.hub().latest();
        }
        inner.safety.drain_log_events_off_thread();

        // Make J7 stale (> 100 ms, still < 250 ms fault).
        {
            let mut g = bus.0.lock().unwrap();
            g.rx.clear();
            g.pending_rx.clear();
            g.mute_j7 = true;
            g.clock.advance(Duration::from_millis(120));
            g.push_all_feedback();
        }
        let (pub_stale, a_ep, f_ep) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(pub_stale.kept.is_none());
        drop(pub_stale);
        assert_eq!((a_ep, f_ep), (0, 0), "R1 stale J7 freeze tick epilogue touched allocator");
        let _ = inner.core.hub().latest();

        // Build live policy TargetFrame and consume it on `armd-control` inside `counted(...)`.
        let now = std::time::Instant::now();
        let boot = BootId(1);
        let ctx = CommitCtx {
            arm: ArmId::LOCAL,
            boot,
            now,
            stop_generation: inner.current_stop_gen(),
        };
        let stamp_pol = inner
            .policy_target_stamp(pol_id, ctx.stop_generation)
            .expect("policy stamp");
        let mut q_close = [0.0f32; MAX_DOF];
        q_close[..7].copy_from_slice(&inner.commanded_positions);
        q_close[6] = -2.45;
        let frame = TargetFrame {
            arm: ArmId::LOCAL,
            boot,
            stream: armd::supervisor::targets::policy_stream(pol_id),
            seq: 1,
            authority: stamp_pol.authority,
            policy_generation: stamp_pol.authorization.max(1),
            input_at: now,
            vision_at: None,
            vision_frame_id: None,
            deadline: now + Duration::from_secs(1),
            dof: 7,
            q: q_close,
            replaceable: false,
        };

        let (res, a_pol, f_pol) = counted(|| inner.consume_target_frame(&frame, &ctx));
        assert_eq!(res, Err(Refusal::code("J7_FEEDBACK_STALE")));
        assert_eq!(
            (a_pol, f_pol),
            (0, 0),
            "R2 consume_policy_target stale-J7 refusal touched allocator"
        );
        assert!(inner.policy_run.is_none(), "policy run must terminate on stale J7");
        assert_eq!(
            inner.retired_policy_runs_len(),
            1,
            "terminated PolicyRun must retire off-thread (0 frees on RT)"
        );
        assert_eq!(
            inner.last_policy_obs.as_ref().and_then(|o| o.stop_reason.as_deref()),
            Some("J7_FEEDBACK_STALE")
        );
        assert_eq!(
            inner.last_action.as_ref().and_then(|a| a.code.as_deref()),
            Some("J7_FEEDBACK_STALE")
        );

        // Publishing the terminated policy observation + action on the next tick is also (0, 0)!
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (pub_post, a_pub, f_pub) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(pub_post.kept.is_none());
        drop(pub_post);
        assert_eq!((a_pub, f_pub), (0, 0), "post-termination policy tick_publish touched allocator");
        let _ = inner.core.hub().latest();

        // Off-thread reclaim (outside counted).
        assert!(inner.pop_retired_policy_run().is_some());
        let _ = inner.release_lease("r2-policy-conn", lease_gen);
        inner.lease_enabled = false;
        inner.follower = None;
    }

    // Ensure J7 is still stale (> 100 ms, operational, not yet 250 ms fault) — reset freshness cleanly
    // so we have full headroom before the 250 ms fault while testing all commit paths!
    {
        let mut g = bus.0.lock().unwrap();
        g.mute_j7 = false;
        g.clock.advance(Duration::from_millis(20));
        g.push_all_feedback();
    }
    let _ = inner.tick_publish(0.02, 1, false);
    let _ = inner.core.hub().latest();
    {
        let mut g = bus.0.lock().unwrap();
        g.rx.clear();
        g.pending_rx.clear();
        g.mute_j7 = true;
        g.clock.advance(Duration::from_millis(120));
        g.push_all_feedback();
    }
    let _ = inner.tick_publish(0.02, 1, false);
    let _ = inner.core.hub().latest();
    assert!(inner.safety.state().is_operational(), "must stay operational at 120 ms J7 age");

    // 3-5. Standalone `commit_prepared_slot`, `commit_prepared`, and `commit_primitive` (`JointQuintic`)
    //      while J7 is stale: strictly (0, 0) allocs/frees!
    let boot = BootId(7);
    let now = std::time::Instant::now();
    let deadline = now + Duration::from_secs(5);
    let ctx = CommitCtx {
        arm: ArmId::LOCAL,
        boot,
        now,
        stop_generation: inner.current_stop_gen(),
    };
    let auth = Authority {
        epoch: inner.safety.epoch_id(),
        stop_generation: ctx.stop_generation,
        lease_generation: inner.current_active_lease_gen(),
        config_revision: inner.workspace_revision,
        parent_generation: 0,
    };
    let tool = tool_digest_hash(&inner.config.active_tool.config_digest);
    let mut start = [0.0f32; MAX_DOF];
    start[..7].copy_from_slice(&inner.commanded_positions);
    let mut goal_j7 = start;
    goal_j7[6] = 0.0;

    let mut replay_slot = PlanSlot::default();
    for k in 0..=10 {
        let f = k as f32 / 10.0;
        let mut q = start;
        q[6] = start[6] + (0.0 - start[6]) * f;
        replay_slot.path.push(&q[..7], k as f64 * 0.02).unwrap();
    }
    replay_slot.primitive = Primitive::Prepared {
        child: None,
        dof: 7,
        start,
        kind: PathKind::Replay {
            episode: EpisodeRef::new("r2_standalone_ep").unwrap(),
            speed: 1.0,
            gripper: true,
            sample_dt: 0.02,
            approach_duration_s: 0.5,
            approach_power_scale: None,
            tool_digest: tool,
        },
        duration_s: 0.2,
        power_scale: None,
    };
    let jog_j7_prim = Primitive::JointQuintic {
        dof: 7,
        start,
        q: goal_j7,
        duration_s: 0.5,
    };

    let (mut pw, mut pr) = plan_slots(4, PlanSlot::default);
    let h_rep = pw.stage(|s| s.copy_from(&replay_slot).unwrap()).unwrap();
    let make_desc = |req_id: u64, parent_gen: u64, kind: CommandKind| Descriptor {
        stamp: Stamp {
            arm: ArmId::LOCAL,
            boot,
            request: RequestId(req_id),
            client: ClientId(1),
            client_seq: req_id,
            authority: Authority { parent_generation: parent_gen, ..auth },
            deadline,
        },
        kind,
    };

    let (standalone_codes, a_st, f_st) = counted(|| {
        let d1 = make_desc(101, 0, CommandKind::Install { plan: h_rep });
        let d2 = make_desc(102, 0, CommandKind::Install { plan: h_rep });
        let d3 = make_desc(103, 0, CommandKind::Install { plan: h_rep });
        let c_slot = inner.commit_prepared_slot(&d1, pr.pin(h_rep).unwrap(), &ctx).unwrap_err();
        let c_prep = inner
            .commit_prepared(&d2, &replay_slot.primitive, &replay_slot.path, &ctx)
            .unwrap_err();
        let c_jog = inner.commit_primitive(&d3, &jog_j7_prim, &ctx).unwrap_err();
        [c_slot, c_prep, c_jog]
    });
    assert_eq!(
        standalone_codes,
        ["J7_FEEDBACK_STALE", "J7_FEEDBACK_STALE", "J7_FEEDBACK_STALE"]
    );
    assert_eq!(
        (a_st, f_st),
        (0, 0),
        "R2 standalone replay/jog stale-J7 refusals touched allocator"
    );
    assert!(pr.retire(h_rep));
    assert_eq!(pw.reclaim(), 1);

    // 6. Macro children while J7 is stale: `commit_prepared_slot` child replay (`ops.rs:924`),
    //    `commit_prepared` child replay (`ops.rs:864`), and `commit_primitive` child `Gripper`,
    //    `JointQuinticLimited` (moving J7), and `Replay { gripper: true }` (`actions.rs:793-825`)
    //    — ALL strictly (0, 0) allocs/frees!
    let ep_seq = inner.safety.epoch_id();
    let macro_id = {
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).expect("admit macro");
        SupervisorInner::begin_sequence_action(admitted, Some("r2-child-seq".into()), ep_seq)
            .expect("begin sequence macro")
    };
    let child_spec = ChildSpec {
        ticket: ChildTicket { parent: ActionId(macro_id), child_seq: 1 },
        timeout_s: 5.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        contact: None,
    };
    let mut child_replay_slot = replay_slot.clone();
    child_replay_slot.primitive = Primitive::Prepared {
        child: Some(child_spec),
        dof: 7,
        start,
        kind: PathKind::Replay {
            episode: EpisodeRef::new("r2_child_ep").unwrap(),
            speed: 1.0,
            gripper: true,
            sample_dt: 0.02,
            approach_duration_s: 0.5,
            approach_power_scale: None,
            tool_digest: tool,
        },
        duration_s: 0.2,
        power_scale: None,
    };
    let h_child_rep = pw.stage(|s| s.copy_from(&child_replay_slot).unwrap()).unwrap();
    let child_grip_prim = Primitive::Gripper {
        child: Some(child_spec),
        dof: 7,
        start,
        fraction: 0.0,
        speed: 0.30,
        grasp: true,
    };
    let child_quintic_prim = Primitive::JointQuinticLimited {
        child: Some(child_spec),
        dof: 7,
        start,
        q: goal_j7,
        duration_s: 0.5,
        max_velocity: [0.0; MAX_DOF],
        settle_s: 0.1,
    };
    let child_raw_replay_prim = Primitive::Replay {
        child: Some(child_spec),
        dof: 7,
        start,
        episode: EpisodeRef::new("r2_child_raw_ep").unwrap(),
        speed: 1.0,
        gripper: true,
    };

    let (child_codes, a_ch, f_ch) = counted(|| {
        let dc1 = make_desc(201, macro_id, CommandKind::Install { plan: h_child_rep });
        let dc2 = make_desc(202, macro_id, CommandKind::Install { plan: h_child_rep });
        let dc3 = make_desc(203, macro_id, CommandKind::Install { plan: h_child_rep });
        let dc4 = make_desc(204, macro_id, CommandKind::Install { plan: h_child_rep });
        let dc5 = make_desc(205, macro_id, CommandKind::Install { plan: h_child_rep });
        let c_slot = inner
            .commit_prepared_slot(&dc1, pr.pin(h_child_rep).unwrap(), &ctx)
            .unwrap_err();
        let c_prep = inner
            .commit_prepared(&dc2, &child_replay_slot.primitive, &child_replay_slot.path, &ctx)
            .unwrap_err();
        let c_grip = inner.commit_primitive(&dc3, &child_grip_prim, &ctx).unwrap_err();
        let c_qlim = inner.commit_primitive(&dc4, &child_quintic_prim, &ctx).unwrap_err();
        let c_raw = inner.commit_primitive(&dc5, &child_raw_replay_prim, &ctx).unwrap_err();
        [c_slot, c_prep, c_grip, c_qlim, c_raw]
    });
    assert_eq!(
        child_codes,
        [
            "J7_FEEDBACK_STALE",
            "J7_FEEDBACK_STALE",
            "J7_FEEDBACK_STALE",
            "J7_FEEDBACK_STALE",
            "J7_FEEDBACK_STALE",
        ]
    );
    assert_eq!(
        (a_ch, f_ch),
        (0, 0),
        "R2 macro-child stale-J7 refusals touched allocator"
    );
    assert!(pr.retire(h_child_rep));
    assert_eq!(pw.reclaim(), 1);
    inner.finish_macro(macro_id, "cancelled", Some("STOPPED"), None, None, None);
    // Clear legacy-populated `phase_ms` BTreeMap from `finish_macro` before RT Part 7 measurement.
    inner.previous_action = None;
    if let Some(a) = inner.last_action.as_mut() {
        a.phase_ms.clear();
    }
    while inner.pop_retired_macro_run().is_some() {}

    // 7. `reconcile_action` terminating a live `PolicyRun` when `J7_FEEDBACK_STALE` latches at 250 ms
    //    (without any incoming policy frame): strictly (0, 0) allocs/frees across the latch tick!
    #[cfg(feature = "policy-runtime")]
    {
        {
            let mut g = bus.0.lock().unwrap();
            g.mute_j7 = false;
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let _ = inner.tick_publish(0.02, 1, false);
        let _ = inner.core.hub().latest();

        use armd::policy_runtime::{PolicyMode, PolicyRunStart};
        inner.follower = Some("sim".into());
        inner.lease_enabled = true;
        let lease_gen = inner.acquire_lease("r2-fault-pol").expect("lease");
        let ep_pol = inner.safety.epoch_id();
        let pol_id = {
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).expect("admit policy");
            SupervisorInner::begin_policy_run(
                admitted,
                PolicyRunStart {
                    epoch_id: ep_pol,
                    connection_id: "r2-fault-pol".into(),
                    lease_generation: lease_gen,
                    policy_id: "r2-fault-pol".into(),
                    artifact_digest: "0".repeat(64),
                    backend: "mlp".into(),
                    mode: PolicyMode::Live,
                    sim_only: false,
                    max_first_step_rad: None,
                    max_step_rad: None,
                    tag: None,
                    i_am_at_the_arm: true,
                    verified_release_gate: None,
                },
            )
            .expect("begin live policy")
        };
        let mut q_step = [0.0f32; MAX_DOF];
        q_step[..7].copy_from_slice(&inner.commanded_positions);
        inner.policy_step_slice(pol_id, &q_step[..7], Some(5), None, None).expect("warm step");
        for _ in 0..4 {
            {
                let mut g = bus.0.lock().unwrap();
                g.clock.advance(Duration::from_millis(20));
                g.push_all_feedback();
            }
            let _ = inner.tick_publish(0.02, 1, false);
            let _ = inner.core.hub().latest();
        }
        inner.safety.drain_log_events_off_thread();

        bus.0.lock().unwrap().mute_j7 = true;
        let mut latched = false;
        for tick in 1..=16 {
            {
                let mut g = bus.0.lock().unwrap();
                g.clock.advance(Duration::from_millis(20));
                g.push_all_feedback();
            }
            let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
            assert!(published.kept.is_none());
            drop(published);
            assert_eq!(
                (a, f),
                (0, 0),
                "policy active J7_FEEDBACK_STALE latch tick {tick} touched allocator"
            );
            if inner.safety.fault_code() == Some(FaultCode::J7FeedbackStale) {
                latched = true;
                break;
            }
        }
        assert!(latched, "expected J7_FEEDBACK_STALE to latch while policy active");
        assert!(inner.policy_run.is_none());
        assert_eq!(inner.retired_policy_runs_len(), 1);
        assert!(inner.pop_retired_policy_run().is_some());
    }
}

/// Round-7 M3 zero-allocation proof on `armd-control`:
/// MIT steady-state ticks, MIT batch write failures (`Error`, `MitFenceTripped`, `EstopInhibited`,
/// `Deferred`, `Expired`), `mit_prepare_inline` refusal, `mit_supervise` deadline loss, and
/// post-loss `MitOwner::Lost` ticks all perform strictly 0 allocations and 0 frees while
/// recording bounded `mit.lost_reason`, latching `BusLoss`, queuing deferred `SafetyLogEvent`,
/// and still emitting J7 hold/freeze through the tick epilogue.
#[test]
fn rt_m3_mit_batch_error_and_mit_lose_zero_alloc_and_zero_free() {
    use armd::driver::{
        gripper_drive_from_profile, DriverCapabilities, EnableState, GripperPosForceInfo,
        PosForceEmitOutcome, PosForceIntent, PosForceIntentStamp, PosForceTxStatus,
    };
    use armd::mit::{self, MitJointQualification, MitOwner, MitQualificationFile};
    use armd::model::JointState;
    use armd::safety::FaultCode;
    use armd::tracking::{JointTrackingConfig, TrackingStrategy};

    let limits = b601_limits();

    let mit_cfg = || {
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints {
            j.strategy = TrackingStrategy::Position;
        }
        let j4 = &mut jt.joints[3];
        j4.strategy = TrackingStrategy::Mit;
        j4.kp = 10.0;
        j4.kd = 0.3;
        j4.hold_kp = Some(12.0);
        j4.hold_kd = Some(0.4);
        j4.effort_limit_nm = 3.0;
        j4.gravity_scale = 1.0;
        ArmConfig {
            lease_timeout_ms: 600_000,
            workspace: None,
            joint_tracking: Some(jt),
            ..ArmConfig::rebot_dm()
        }
    };

    let mk_qualified_active_mit = || {
        let cfg = mit_cfg();
        let l4 = limits[3];
        let rec = MitQualificationFile {
            version: mit::QUALIFICATION_VERSION,
            joints: vec![MitJointQualification {
                profile_id: "rebot_b601dm".into(),
                joint: 4,
                joint_name: cfg.joint_names[3].clone(),
                zero_fingerprint: mit::zero_fingerprint(0.0),
                motor_id: 4,
                reply_id: 0x14,
                esc_id: 4,
                mst_id: 0x14,
                sw_ver: 0x0001_0000,
                pmax: l4.p_max,
                vmax: l4.v_max,
                tmax: l4.t_max,
                timeout_raw: 2000,
                timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
                timeout_unit_verified: true,
                tested_response: "disabled".into(),
                measured_expiry_ms: 101.0,
                worst_latency_ms: 8.0,
                load_envelope: "unloaded, supported".into(),
                max_kp: 15.0,
                max_kd: 0.5,
                effort_ceiling_nm: 4.0,
                date: "2026-11-24T00:00:00Z".into(),
                trace_id: "m3-zero-alloc".into(),
            }],
            invalidated: None,
        };
        let bus = SharedNoAllocBus::new(&limits);
        {
            let mut g = bus.0.lock().unwrap();
            g.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
            g.push_all_feedback();
        }
        let grip = gripper_drive_from_profile(&cfg).expect("pos-force drive");
        let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_drive(grip)
            .unwrap();
        driver.read_joint_states().unwrap();
        driver.check_mapping_ranges();
        driver.set_rt_mode(true);
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
        inner.lease_enabled = false;
        inner.set_calibration(Some(armd::calibration::CalibrationFile {
            mit_qualification: serde_json::to_value(&rec).ok(),
            ..Default::default()
        }));
        inner.tick(0.02);
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).expect("arm with MIT qualification");
        inner.rt_executor_owned = true;
        inner.safety.set_off_thread_reclaim(true);
        // Step stationary ticks until MIT handover completes and triple-buffer slots warm up.
        for _ in 0..25 {
            {
                let mut g = bus.0.lock().unwrap();
                g.clock.advance(Duration::from_millis(20));
                g.push_all_feedback();
            }
            let _ = inner.tick_publish(0.02, 1, false);
            let _ = inner.core.hub().latest();
        }
        assert_eq!(
            inner.mit.owner,
            MitOwner::Active,
            "MIT must be active after handover (handover_error={:?}, lost_reason={:?})",
            inner.mit.handover_error,
            inner.mit.lost_reason
        );
        // Disable Phase-1 position-lead tracking (`src/tracking.rs`, out of scope for M3) after
        // real MIT handover completes so `tick_publish` exercises the full MIT `mit_supervise` ->
        // `write_output_scratch` -> `mit_prepare_inline` -> `write_joint_targets_bounded` ->
        // `mit_lose_bounded` -> J7 epilogue -> `publish_at` path with strictly (0, 0) alloc/free.
        if let Some(jt) = inner.config.joint_tracking.as_mut() {
            jt.enabled = false;
        }
        // Warm up one post-handover tick across triple-buffer slots before counting.
        for _ in 0..4 {
            {
                let mut g = bus.0.lock().unwrap();
                g.clock.advance(Duration::from_millis(20));
                g.push_all_feedback();
            }
            let _ = inner.tick_publish(0.02, 1, false);
            let _ = inner.core.hub().latest();
        }
        inner.safety.drain_log_events_off_thread();
        reset_tx_metrics();
        (inner, bus)
    };

    // 1. Steady-state Active MIT tick + HardError (TickOutput::Error -> mit_lose_bounded)
    //    + subsequent MitOwner::Lost tick (mit_supervise with tripped fence + J7 hold epilogue).
    {
        let (mut inner, bus) = mk_qualified_active_mit();
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        let _ = inner.core.hub().latest();
        assert_eq!(inner.mit.owner, MitOwner::Active);
        assert_eq!((a, f), (0, 0), "steady-state Active MIT tick_publish touched allocator");

        // Trigger TickOutput::Error via Mode::HardError during MIT batch write.
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
            g.mode = Mode::HardError;
            g.tx_count = 0;
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "MIT HardError batch failure + mit_lose touched allocator");
        assert_eq!(inner.mit.owner, MitOwner::Lost);
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::BusLoss));
        let reason = inner.mit.lost_reason.expect("lost_reason must be recorded");
        assert!(
            reason.as_str().contains("MIT batch write failed"),
            "unexpected lost_reason: {reason}"
        );
        assert!(
            inner.safety.log_events_len() >= 1,
            "deferred MitTrackingCancelled safety log must be queued"
        );
        let _ = inner.core.hub().latest();

        // Recover bus and run one post-loss tick: MitOwner::Lost + tripped fence + J7 hold epilogue.
        {
            let mut g = bus.0.lock().unwrap();
            g.mode = Mode::Ok;
            g.tx_count = 0;
            g.pf_tx_count = 0;
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "post-loss MitOwner::Lost tick_publish touched allocator");
        assert_eq!(
            bus.0.lock().unwrap().pf_tx_count,
            1,
            "J7 hold/freeze 0x307 must still emit on post-loss fenced tick"
        );
    }

    // 2. MIT fence trip on WouldBlock during batch write (MitFenceTripped -> TickOutput::Error).
    {
        let (mut inner, bus) = mk_qualified_active_mit();
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
            g.mode = Mode::WouldBlockAfter(0);
            g.tx_count = 0;
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "MIT WouldBlock/MitFenceTripped batch failure touched allocator");
        assert_eq!(inner.mit.owner, MitOwner::Lost);
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::BusLoss));
        assert!(inner
            .mit
            .lost_reason
            .is_some_and(|r| r.as_str().contains("MIT batch write failed")));
    }

    // 3. Pending STOP arriving mid-batch (TickOutput::EstopInhibited -> mit_lose_bounded).
    {
        let (mut inner, bus) = mk_qualified_active_mit();
        let stops = StopRequests::new();
        inner.attach_stop_requests(stops.clone());
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
            g.tx_count = 0;
            g.pf_tx_count = 0;
            g.on_tx_stop = Some((1, stops));
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "MIT mid-batch STOP EstopInhibited + mit_lose touched allocator");
        assert_eq!(inner.mit.owner, MitOwner::Lost);
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::BusLoss));
        assert!(inner
            .mit
            .lost_reason
            .is_some_and(|r| r.as_str().contains("MIT batch write failed")));
        assert_eq!(
            bus.0.lock().unwrap().pf_tx_count,
            1,
            "J7 stop/hold 0x307 must still emit after mid-batch STOP"
        );
    }

    // 4. Stale feedback >= 45 ms hitting mit_supervise -> mit_lose_bounded (0 alloc, 0 free).
    {
        let (mut inner, bus) = mk_qualified_active_mit();
        {
            let mut g = bus.0.lock().unwrap();
            g.rx.clear();
            g.pending_rx.clear();
            g.clock.advance(Duration::from_millis(50));
            g.pf_tx_count = 0;
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "mit_supervise stale-feedback mit_lose touched allocator");
        assert_eq!(inner.mit.owner, MitOwner::Lost);
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::BusLoss));
        assert!(
            inner.mit.lost_reason.is_some_and(|r| r.as_str().contains("feedback age")),
            "expected 'feedback age' in {:?}",
            inner.mit.lost_reason
        );
        assert_eq!(bus.0.lock().unwrap().pf_tx_count, 1);
    }

    // 5. mit_prepare_inline refusal (missing gravity model while gravity_scale > 0): 0 alloc, 0 free.
    {
        let (mut inner, bus) = mk_qualified_active_mit();
        inner.config.gravity_model = None;
        {
            let mut g = bus.0.lock().unwrap();
            g.clock.advance(Duration::from_millis(20));
            g.push_all_feedback();
            g.pf_tx_count = 0;
        }
        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!((a, f), (0, 0), "mit_prepare_inline refusal tick_publish touched allocator");
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::DriverWriteFailed));
        assert_eq!(
            bus.0.lock().unwrap().pf_tx_count,
            1,
            "J7 hold/freeze 0x307 must still emit when mit_prepare_inline refuses"
        );
    }

    // 6. Direct coverage of TickOutput::Deferred, TickOutput::Expired, and TickOutput::EstopInhibited
    //    arms inside write_output_slice (`mit.rs`) through full `tick_publish`: strictly (0, 0).
    #[derive(Clone, Copy)]
    enum StubOutcome {
        Ok,
        Deferred(usize),
        Expired(usize),
        EstopInhibited,
    }
    struct StubMitBatchDriver {
        outcome: StubOutcome,
        pf_emits: u64,
        clock: std::sync::Arc<MockClock>,
        states: [JointState; 7],
    }
    impl StubMitBatchDriver {
        fn new() -> Self {
            Self {
                outcome: StubOutcome::Ok,
                pf_emits: 0,
                clock: std::sync::Arc::new(MockClock::new()),
                states: std::array::from_fn(|_| JointState {
                    position: 0.0,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                }),
            }
        }
    }
    impl ArmDriver for StubMitBatchDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                temperature_feedback: true,
                torque_feedback: true,
                ..DriverCapabilities::default()
            }
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(self.states.to_vec())
        }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
            out.clear();
            out.extend_from_slice(&self.states);
            Ok(())
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn write_joint_targets_bounded(
            &mut self,
            _targets: &[JointTarget],
        ) -> Result<TxBatchOutcome, ArmDriverError> {
            match self.outcome {
                StubOutcome::Ok => Ok(TxBatchOutcome::Complete),
                StubOutcome::Deferred(pending) => Ok(TxBatchOutcome::Deferred { pending }),
                StubOutcome::Expired(dropped) => Ok(TxBatchOutcome::Expired { dropped, pending: 0 }),
                StubOutcome::EstopInhibited => Err(ArmDriverError::EstopInhibited),
            }
        }
        fn write_estop_hold_bounded(
            &mut self,
            _targets: &[JointTarget],
        ) -> Result<TxBatchOutcome, ArmDriverError> {
            Ok(TxBatchOutcome::Complete)
        }
        fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear();
            out.extend([Some(Duration::from_millis(5)); 7]);
        }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
            dof == 7 && Duration::from_millis(5) <= max_age
        }
        fn clock_now(&self) -> std::time::Instant { self.clock.now() }
        fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
            Some(GripperPosForceInfo {
                joint: 6,
                active: true,
                close_speed_rad_s: 0.3,
                ceiling_torque_estimate_nm: None,
            })
        }
        fn emit_pos_force_j7(
            &mut self,
            _intent: PosForceIntent,
            _current: PosForceIntentStamp,
        ) -> Result<PosForceEmitOutcome, ArmDriverError> {
            self.pf_emits += 1;
            Ok(PosForceEmitOutcome::Sent {
                joint: 6,
                freeze: false,
                pf_status: PosForceTxStatus::Fresh,
            })
        }
    }

    for (outcome, expected_substr) in [
        (StubOutcome::Deferred(2), "MIT batch write deferred (2 frames queued)"),
        (StubOutcome::Expired(3), "MIT batch write expired (3 frames dropped)"),
        (StubOutcome::EstopInhibited, "MIT batch write failed:"),
    ] {
        let mut cfg = mit_cfg();
        cfg.joint_tracking.as_mut().unwrap().enabled = false;
        let mut drv = StubMitBatchDriver::new();
        drv.outcome = StubOutcome::Ok;
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.tick(0.02);
        inner.is_torqued = true;
        inner.safety.set_torque(true).expect("armed");
        inner.mit.owner = MitOwner::Active;
        inner.mit.joints = vec![3];
        inner.mit.last_ff = vec![0.0; 7];
        inner.mit.last_output = Some(inner.driver.as_ref().unwrap().lock().unwrap().clock_now());
        inner.rt_executor_owned = true;
        inner.safety.set_off_thread_reclaim(true);
        // Warm up steady-state MIT ticks across triple-buffer slots.
        for _ in 0..5 {
            let _ = inner.tick_publish(0.02, 1, false);
            let _ = inner.core.hub().latest();
        }
        inner.safety.drain_log_events_off_thread();

        // Inject the specific batch outcome by swapping the stub driver's mode.
        let mut failing = StubMitBatchDriver::new();
        failing.outcome = outcome;
        inner.driver = Some(std::sync::Mutex::new(Box::new(failing)));
        inner.mit.last_output = Some(inner.driver.as_ref().unwrap().lock().unwrap().clock_now());

        let (published, a, f) = counted(|| inner.tick_publish(0.02, 1, false));
        assert!(published.kept.is_none());
        drop(published);
        assert_eq!(
            (a, f),
            (0, 0),
            "MIT batch outcome '{expected_substr}' touched allocator"
        );
        assert_eq!(inner.mit.owner, MitOwner::Lost);
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::BusLoss));
        let reason = inner.mit.lost_reason.expect("lost_reason recorded");
        assert!(
            reason.as_str().contains(expected_substr),
            "expected '{expected_substr}' in '{reason}'"
        );
    }
}
