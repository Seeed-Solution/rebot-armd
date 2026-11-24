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
    #[cfg(target_os = "linux")]
    SocketCanOsAfter(usize, i32),
}

/// Pre-allocated Damiao CAN bus fake (zero allocations during `transmit` and `receive`).
struct NoAllocDamiaoBus {
    motors: Vec<(u16, DamiaoLimits, f32, bool, u32)>,
    rx: VecDeque<CanFrame>,
    /// Post-transmit replies promoted to `rx` once `rx` drains empty (models SocketCAN causality
    /// without heap allocation so post-write probe drains separate pre-probe from post-probe RX).
    pending_rx: VecDeque<CanFrame>,
    clock: std::sync::Arc<MockClock>,
    mode: Mode,
    tx_count: usize,
    tx_step: Duration,
    flush_err: Option<Mode>,
    on_tx_stop: Option<(usize, StopRequests)>,
    on_flush_stop: Option<StopRequests>,
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
            tx_count: 0,
            tx_step: Duration::ZERO,
            flush_err: None,
            on_tx_stop: None,
            on_flush_stop: None,
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
        let frame = self.build_reply_frame(k);
        if self.rx.len() < self.rx.capacity() {
            self.rx.push_back(frame);
        }
    }

    fn reply_pending(&mut self, k: usize) {
        let frame = self.build_reply_frame(k);
        if self.pending_rx.len() < self.pending_rx.capacity() {
            self.pending_rx.push_back(frame);
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
                            _ => 0u32.to_le_bytes(),
                        };
                        let [lo, hi] = mid.to_le_bytes();
                        if self.rx.len() < self.rx.capacity() {
                            self.rx.push_back(
                                CanFrame::new_standard(
                                    0x10 + mid,
                                    &[lo, hi, damiao::REG_READ, frame.data[3], val[0], val[1], val[2], val[3]],
                                )
                                .unwrap(),
                            );
                        }
                    }
                    damiao::REG_WRITE => {
                        let val = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
                        if frame.data[3] == damiao::RID_CTRL_MODE {
                            self.motors[k].4 = u32::from_le_bytes(val);
                        }
                        let [lo, hi] = mid.to_le_bytes();
                        if self.rx.len() < self.rx.capacity() {
                            self.rx.push_back(
                                CanFrame::new_standard(
                                    0x10 + mid,
                                    &[lo, hi, damiao::REG_WRITE, frame.data[3], val[0], val[1], val[2], val[3]],
                                )
                                .unwrap(),
                            );
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
            self.motors[k].2 = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.reply(k);
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
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
    d_pf.can_mut().rx.push_back(reply);
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
        driver.set_rt_mode(true);
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
        inner.lease_enabled = false;
        inner.tick(0.02);
        inner.set_torque(true).expect("arm");
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
    gc_inner.enter_gravity_comp(Default::default()).expect("enter gravity comp");
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
    let (_, jog_id, _) = jog_inner
        .jog_action(vec![0.15, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), Some(epoch), "joint_move")
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
