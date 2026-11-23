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
            clock: std::sync::Arc::new(MockClock::new()),
            mode: Mode::Ok,
            tx_count: 0,
            tx_step: Duration::ZERO,
            flush_err: None,
            on_tx_stop: None,
            on_flush_stop: None,
        }
    }

    fn reply(&mut self, k: usize) {
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
        if self.rx.len() < self.rx.capacity() {
            self.rx.push_back(CanFrame::new_standard(0x10 + id, &data).unwrap());
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
                    0xCC => self.reply(k),
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
        self.rx.pop_front().ok_or(TransportError::WouldBlock)
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
    assert!(matches!(res, Err(ArmDriverError::Transport(TransportError::Timeout))), "{res:?}");
    assert_eq!((a, f), (0, 0), "unfenced mid-pass stale drop Timeout touched allocator");

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
