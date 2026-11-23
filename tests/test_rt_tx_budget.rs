//! Virtual-bus tests for bounded per-tick CAN TX and per-frame TX instrumentation
//! (armd 1.6 stage 3 §4):
//! - Queue-full injection (`WouldBlock` and `ENOBUFS` / `NoBufferSpace`), zero retry sleeps
//! - Mid-batch E-stop and authorized stop dropping pending motion frames
//! - Inclusive 45 ms expiry on both carried-over queue frames and the MIT deadline fence
//! - Budget exhaustion (`max_frames` and absolute `max_duration` cutoff) carrying fresh frames,
//!   coalescing opposite consecutive targets, and dropping stale ones
//! - Shared tick budget across RX recovery / 0xCC refresh and target writes (early backpressure
//!   closes the entire pass)
//! - MIT partial-batch trip, oversized-batch preflight, and fail-closed flush handling
//! - Per-frame wall/CPU timing exposed through `GET /api/v1/diagnostics/tick`
//! - Fail-closed legacy supervisor bridge on a backpressured final release batch

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::api::{create_router_full, proxy::DetectorUpstream};
use armd::diagnostics::tick::{reset_tx_metrics, take_tx_metrics};
use armd::diagnostics::DiagnosticsState;
use armd::driver::{
    is_estop_inhibited, ArmDriver, ArmDriverError, DamiaoArmDriver, JointMode, JointTarget,
    TxBatchOutcome, TxBudget,
};
use armd::motion::{GripperCommand, GripperRequest};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::urgent::{EstopGate, StopRequests};
use armd::safety::{FaultCode, SafetyState};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};

fn ms(n: u64) -> Duration { Duration::from_millis(n) }
fn us(n: u64) -> Duration { Duration::from_micros(n) }

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

#[derive(Clone, Copy, Debug)]
enum TxRule {
    Ok,
    WouldBlock,
    Enobufs,
    HardError,
}

struct VirtualBusState {
    motors: Vec<(u16, DamiaoLimits, f32, bool, u32)>, // (id, limits, q, enabled, ctrl_mode)
    rx: VecDeque<CanFrame>,
    tx_log: Vec<CanFrame>,
    tx_accepted: Vec<CanFrame>,
    tx_script: VecDeque<(TxRule, Duration)>,
    default_rule: TxRule,
    default_step: Duration,
    flush_error: Option<TxRule>,
    clock: MockClock,
    on_tx: Option<Arc<dyn Fn(usize, &CanFrame) + Send + Sync>>,
    on_flush: Option<Arc<dyn Fn() + Send + Sync>>,
    tx_attempts: usize,
    /// Optional busy-work loop per transmit to produce nonzero CLOCK_THREAD_CPUTIME_ID on Linux.
    cpu_spin_iters: u64,
}

impl VirtualBusState {
    fn new(limits: &[DamiaoLimits], clock: MockClock) -> Self {
        let motors = limits
            .iter()
            .enumerate()
            .map(|(i, l)| (i as u16 + 1, *l, 0.0, false, damiao::CTRL_MODE_POS_VEL))
            .collect();
        Self {
            motors,
            rx: VecDeque::new(),
            tx_log: Vec::new(),
            tx_accepted: Vec::new(),
            tx_script: VecDeque::new(),
            default_rule: TxRule::Ok,
            default_step: Duration::ZERO,
            flush_error: None,
            clock,
            on_tx: None,
            on_flush: None,
            tx_attempts: 0,
            cpu_spin_iters: 0,
        }
    }

    fn push_feedback(&mut self, idx: usize) {
        let (id, l, q, enabled, _) = self.motors[idx];
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
        self.rx.push_back(CanFrame::new_standard(0x10 + id, &data).unwrap());
    }

    fn push_all_feedback(&mut self) {
        for i in 0..self.motors.len() {
            self.push_feedback(i);
        }
    }

    fn handle_frame(&mut self, frame: &CanFrame) {
        let id = frame.id as u16;
        if id == 0x7FF {
            let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let (op, rid) = (frame.data[2], frame.data[3]);
            if let Some(k) = self.motors.iter().position(|m| m.0 == mid) {
                match op {
                    0xCC => self.push_feedback(k),
                    damiao::REG_READ => {
                        let (_, l, _, _, mode) = self.motors[k];
                        let val = match rid {
                            damiao::RID_CTRL_MODE => mode.to_le_bytes(),
                            damiao::RID_PMAX => l.p_max.to_le_bytes(),
                            damiao::RID_VMAX => l.v_max.to_le_bytes(),
                            damiao::RID_TMAX => l.t_max.to_le_bytes(),
                            _ => 0u32.to_le_bytes(),
                        };
                        let [lo, hi] = mid.to_le_bytes();
                        let reply = CanFrame::new_standard(
                            0x10 + mid,
                            &[lo, hi, damiao::REG_READ, rid, val[0], val[1], val[2], val[3]],
                        )
                        .unwrap();
                        self.rx.push_back(reply);
                    }
                    damiao::REG_WRITE => {
                        let val = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
                        if rid == damiao::RID_CTRL_MODE {
                            self.motors[k].4 = u32::from_le_bytes(val);
                        }
                        let [lo, hi] = mid.to_le_bytes();
                        let reply = CanFrame::new_standard(
                            0x10 + mid,
                            &[lo, hi, damiao::REG_WRITE, rid, val[0], val[1], val[2], val[3]],
                        )
                        .unwrap();
                        self.rx.push_back(reply);
                    }
                    _ => {}
                }
            }
            return;
        }
        if let Some(k) = self.motors.iter().position(|m| m.0 == id) {
            if frame.data == damiao::CMD_ENABLE {
                self.motors[k].3 = true;
            } else if frame.data == damiao::CMD_DISABLE {
                self.motors[k].3 = false;
            }
            self.push_feedback(k);
        } else if let Some(k) = self
            .motors
            .iter()
            .position(|m| m.0 == id.wrapping_sub(0x100) || m.0 == id.wrapping_sub(0x300))
        {
            self.motors[k].2 =
                f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.push_feedback(k);
        }
    }
}

#[derive(Clone)]
struct VirtualBus(Arc<Mutex<VirtualBusState>>);

impl VirtualBus {
    fn new(limits: &[DamiaoLimits]) -> (Self, MockClock) {
        let clock = MockClock::new();
        (Self(Arc::new(Mutex::new(VirtualBusState::new(limits, clock.clone())))), clock)
    }

    fn with<R>(&self, f: impl FnOnce(&mut VirtualBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn clear(&self) {
        self.with(|s| {
            s.tx_log.clear();
            s.tx_accepted.clear();
            s.tx_attempts = 0;
        });
    }
}

impl CanTransport for VirtualBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(Arc::new(self.0.lock().unwrap().clock.clone()))
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let (rule, step, spin, cb, attempt_idx) = {
            let mut s = self.0.lock().unwrap();
            let (rule, step) = s
                .tx_script
                .pop_front()
                .unwrap_or((s.default_rule, s.default_step));
            let idx = s.tx_attempts;
            s.tx_attempts += 1;
            s.tx_log.push(*frame);
            (rule, step, s.cpu_spin_iters, s.on_tx.clone(), idx)
        };
        if spin > 0 {
            let mut acc = 0u64;
            for i in 0..spin {
                acc = acc.wrapping_add(std::hint::black_box(i));
            }
            std::hint::black_box(acc);
        }
        {
            let s = self.0.lock().unwrap();
            if !step.is_zero() {
                s.clock.advance(step);
            }
        }
        if let Some(cb) = cb {
            cb(attempt_idx, frame);
        }
        match rule {
            TxRule::Ok => {
                let mut s = self.0.lock().unwrap();
                s.tx_accepted.push(*frame);
                s.handle_frame(frame);
                Ok(())
            }
            TxRule::WouldBlock => Err(TransportError::WouldBlock),
            TxRule::Enobufs => Err(TransportError::NoBufferSpace),
            TxRule::HardError => Err(TransportError::Can("injected bus fault".into())),
        }
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.0
            .lock()
            .unwrap()
            .rx
            .pop_front()
            .ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        let (err, cb) = {
            let mut s = self.0.lock().unwrap();
            (s.flush_error.take(), s.on_flush.clone())
        };
        if let Some(cb) = cb {
            cb();
        }
        match err {
            None | Some(TxRule::Ok) => Ok(()),
            Some(TxRule::WouldBlock) => Err(TransportError::WouldBlock),
            Some(TxRule::Enobufs) => Err(TransportError::NoBufferSpace),
            Some(TxRule::HardError) => Err(TransportError::Can("injected flush error".into())),
        }
    }
}

fn enabled_driver() -> (DamiaoArmDriver<VirtualBus>, VirtualBus, MockClock) {
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("enable");
    bus.clear();
    reset_tx_metrics();
    (d, bus, clock)
}

fn targets_const(pos: f32) -> Vec<JointTarget> {
    (0..7).map(|_| JointTarget::position(pos)).collect()
}

fn pos_vel_target_rad(frame: &CanFrame) -> f32 {
    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]])
}

#[test]
fn queue_full_would_block_and_enobufs_defer_and_count_without_sleeping() {
    let (mut d, bus, clock) = enabled_driver();
    // 3 frames succeed, then J4 hits ENOBUFS; on tick 2 J4 hits WouldBlock; on tick 3 bus clears.
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Ok, us(50)));
        s.tx_script.push_back((TxRule::Ok, us(50)));
        s.tx_script.push_back((TxRule::Ok, us(50)));
        s.tx_script.push_back((TxRule::Enobufs, us(20)));
        s.tx_script.push_back((TxRule::WouldBlock, us(25)));
    });
    let t0 = clock.now();
    reset_tx_metrics();
    let out1 = d.write_joint_targets_bounded(&targets_const(0.25)).unwrap();
    assert_eq!(out1, TxBatchOutcome::Deferred { pending: 4 });
    // No 100 us retry sleep: virtual time advanced strictly by the 4 attempted frames (170 us).
    assert_eq!(clock.now().saturating_duration_since(t0), us(170));
    let m1 = take_tx_metrics();
    assert_eq!((m1.frames, m1.enobufs, m1.would_block, m1.deferred, m1.pending), (3, 1, 0, 4, 4));
    assert_eq!(bus.with(|s| s.tx_accepted.iter().map(|f| f.id).collect::<Vec<_>>()), vec![0x101, 0x102, 0x103]);

    // Tick 2 (20 ms later): J4 hits WouldBlock, stops immediately with no sleep, does not double-count deferred.
    clock.advance(ms(20));
    d.begin_tx_tick();
    let t1 = clock.now();
    let out2 = d.drain_pending_tx().unwrap();
    assert_eq!(out2, TxBatchOutcome::Deferred { pending: 4 });
    assert_eq!(clock.now().saturating_duration_since(t1), us(25));
    let m2 = take_tx_metrics();
    assert_eq!((m2.frames, m2.enobufs, m2.would_block, m2.deferred, m2.pending), (0, 0, 1, 0, 4));

    // Tick 3 (10 ms later, total age 30 ms < 45 ms): queue clears, J4..J7 delivered.
    clock.advance(ms(10));
    d.begin_tx_tick();
    let out3 = d.drain_pending_tx().unwrap();
    assert_eq!(out3, TxBatchOutcome::Complete);
    let m3 = take_tx_metrics();
    assert_eq!((m3.frames, m3.enobufs, m3.would_block, m3.deferred, m3.pending), (4, 0, 0, 0, 0));
    assert_eq!(
        bus.with(|s| s.tx_accepted.iter().map(|f| f.id).collect::<Vec<_>>()),
        vec![0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107]
    );

    // Hard transport error still returns Err (never treated as backpressure) and clears pending queue.
    d.begin_tx_tick();
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::HardError, Duration::ZERO));
    });
    let err = d.write_joint_targets_bounded(&targets_const(0.3)).unwrap_err();
    assert!(matches!(err, ArmDriverError::Transport(TransportError::Can(_))), "{err:?}");
    assert_eq!(d.pending_tx_len(), 0);
}

#[test]
fn mid_batch_stop_and_estop_drop_pending_motion_frames() {
    // Part A: authorized stop (StopRequests via set_stop_gate) arriving mid-batch drops pending frames.
    let (mut d, bus, clock) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let stops = StopRequests::new();
    d.set_stop_gate(req.clone(), app.clone(), Some(stops.clone()));
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });

    // Tick 1 defers J5..J7.
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.4)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    assert_eq!(d.pending_tx_len(), 3);
    bus.clear();
    reset_tx_metrics();

    // Tick 2: after J5 goes out, an authorized stop is raised; J6 and J7 must be dropped.
    let stops_hook = stops.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move |idx, _| {
            if idx == 0 {
                let _ = stops_hook.request();
            }
        }));
    });
    clock.advance(ms(10));
    d.begin_tx_tick();
    let err = d.drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "{err:?}");
    assert_eq!(d.pending_tx_len(), 0, "stop must drop remaining pending motion frames");
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.dropped_stale), (1, 2, 0));

    // After the stop is marked applied, no old motion frame remains queued.
    stops.mark_applied(stops.current());
    bus.with(|s| s.on_tx = None);
    bus.clear();
    d.begin_tx_tick();
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
    assert!(bus.with(|s| s.tx_log.is_empty()));

    // Part B: E-stop raised during an ENOBUFS attempt returns ESTOP_INHIBITED and clears pending queue.
    let (mut d, bus, _) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    d.set_stop_gate(req.clone(), app, None);
    let req_hook = req.clone();
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::Enobufs, Duration::ZERO));
        s.on_tx = Some(Arc::new(move |idx, _| {
            if idx == 1 {
                req_hook.fetch_add(1, Ordering::SeqCst);
            }
        }));
    });
    reset_tx_metrics();
    let err = d.write_joint_targets_bounded(&targets_const(0.5)).unwrap_err();
    assert!(is_estop_inhibited(&err), "{err:?}");
    assert_eq!(d.pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.enobufs, m.dropped_stopped), (1, 1, 6));

    // Part C: deferred targets MUST NOT survive an already-applied ordinary stop (requested == applied).
    let (mut d, bus, clock) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let stops = StopRequests::new();
    d.set_stop_gate(req.clone(), app.clone(), Some(stops.clone()));
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });

    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.45)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    assert_eq!(d.pending_tx_len(), 3);
    // Request AND immediately apply the stop BEFORE the next driver call (requested == applied).
    let stop_gen = stops.request();
    stops.mark_applied(stop_gen);
    assert!(!stops.pending(), "stop is already applied (requested == applied)");
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(10)); // still fresh (< 45 ms)
    d.begin_tx_tick();
    let err = d.drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "applied stop must invalidate deferred frames: {err:?}");
    assert!(bus.with(|s| s.tx_log.is_empty()), "no deferred frame may reach the bus after applied stop");
    assert_eq!(d.pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.dropped_stale, m.pending), (0, 3, 0, 0));

    // Part D: deferred targets MUST NOT survive an already-applied E-stop (requested == applied).
    let (mut d, bus, clock) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    d.set_stop_gate(req.clone(), app.clone(), None);
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });

    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.35)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let estop_gen = req.fetch_add(1, Ordering::SeqCst) + 1;
    app.store(estop_gen, Ordering::SeqCst);
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(10)); // still fresh (< 45 ms)
    d.begin_tx_tick();
    let err = d.drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "applied E-stop must invalidate deferred frames: {err:?}");
    assert!(bus.with(|s| s.tx_log.is_empty()), "no deferred frame may reach the bus after applied E-stop");
    assert_eq!(d.pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.pending), (0, 3, 0));

    // Part E: stop requested AND applied mid-batch during J5 transmit drops J6..J7 immediately.
    let (mut d, bus, clock) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let stops = StopRequests::new();
    d.set_stop_gate(req, app, Some(stops.clone()));
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.25)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let stops_hook = stops.clone();
    bus.clear();
    reset_tx_metrics();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move |idx, _| {
            if idx == 0 {
                let g = stops_hook.request();
                stops_hook.mark_applied(g);
            }
        }));
    });
    clock.advance(ms(10));
    d.begin_tx_tick();
    let err = d.drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "{err:?}");
    assert_eq!(bus.with(|s| s.tx_log.len()), 1, "only J5 sent before applied stop");
    assert_eq!(d.pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.pending), (1, 2, 0));
}

#[test]
fn inclusive_45ms_expiry_carries_fresh_and_drops_stale() {
    // 45 ms - 1 ns: carried-over frames stay fresh and transmit.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(45) - Duration::from_nanos(1));
    d.begin_tx_tick();
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.pending), (3, 0, 0));
    assert_eq!(bus.with(|s| s.tx_accepted.iter().map(|f| f.id).collect::<Vec<_>>()), vec![0x105, 0x106, 0x107]);

    // Exactly 45 ms (inclusive): carried-over frames are dropped as stale, zero transmitted,
    // and drain_pending_tx MUST report Expired (never Complete).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(45));
    d.begin_tx_tick();
    assert_eq!(
        d.drain_pending_tx().unwrap(),
        TxBatchOutcome::Expired { dropped: 3, pending: 0 }
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.dropped_stopped, m.pending), (0, 3, 0, 0));
    assert!(bus.with(|s| s.tx_log.is_empty()));

    // Mid-drain expiry: J5 transmits at 44 ms taking 1 ms -> J6 and J7 hit 45 ms and expire;
    // outcome MUST be Expired { dropped: 2, pending: 0 }, never Complete.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    bus.clear();
    bus.with(|s| s.tx_script.push_back((TxRule::Ok, ms(1))));
    reset_tx_metrics();
    clock.advance(ms(44));
    d.begin_tx_tick();
    assert_eq!(
        d.drain_pending_tx().unwrap(),
        TxBatchOutcome::Expired { dropped: 2, pending: 0 }
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.pending), (1, 2, 0));
    assert_eq!(bus.with(|s| s.tx_accepted.iter().map(|f| f.id).collect::<Vec<_>>()), vec![0x105]);

    // A stalled single transmit pushing the current batch past 45 ms must fail closed (Timeout),
    // never falsely reporting Complete after dropping the remaining frames as stale.
    let (mut d, bus, _) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(100), frame_max_age: ms(45) });
    bus.with(|s| s.tx_script.push_back((TxRule::Ok, ms(45))));
    reset_tx_metrics();
    let err = d.write_joint_targets_bounded(&targets_const(0.3)).unwrap_err();
    assert!(matches!(err, ArmDriverError::Transport(TransportError::Timeout)), "{err:?}");
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.pending), (1, 6, 0));
}

#[test]
fn budget_exhaustion_carries_fresh_coalesces_opposite_targets_and_separates_counters() {
    // Part A: opposite consecutive targets under max_frames = 4.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });

    // Tick 1: command +0.5 rad -> J1..J4 sent (+0.5), J5..J7 deferred.
    reset_tx_metrics();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.5)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let m1 = take_tx_metrics();
    assert_eq!((m1.frames, m1.deferred, m1.pending), (4, 3, 3));
    bus.clear();

    // Tick 2 (20 ms later): opposite command -0.5 rad.
    // J5..J7 stay at the front of the FIFO (no starvation), updated IN PLACE to -0.5 rad (never replaying +0.5!).
    clock.advance(ms(20));
    d.begin_tx_tick();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(-0.5)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let sent2: Vec<(u32, f32)> = bus.with(|s| {
        s.tx_accepted
            .iter()
            .map(|f| (f.id, pos_vel_target_rad(f)))
            .collect()
    });
    assert_eq!(
        sent2,
        vec![(0x105, -0.5), (0x106, -0.5), (0x107, -0.5), (0x101, -0.5)],
        "unsent joints sent first with the new -0.5 rad target, never obsolete +0.5 rad"
    );
    let m2 = take_tx_metrics();
    assert_eq!((m2.frames, m2.deferred, m2.pending), (4, 3, 3));

    // Part B: replacement deferral identity on max_frames = 0 (replacement resets command deferral count).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 0, max_duration: ms(5), frame_max_age: ms(45) });
    reset_tx_metrics();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.5)).unwrap(),
        TxBatchOutcome::Deferred { pending: 7 }
    );
    let r1 = take_tx_metrics();
    assert_eq!((r1.deferred, r1.pending), (7, 7));

    // Same untouched pending frames on tick 2 (drain_pending_tx): deferred == 0, pending == 7.
    clock.advance(ms(10));
    d.begin_tx_tick();
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Deferred { pending: 7 });
    let r2 = take_tx_metrics();
    assert_eq!((r2.deferred, r2.pending), (0, 7));

    // New opposite targets replace the 7 slots: each new command instance is deferred once -> deferred == 7, pending == 7.
    clock.advance(ms(10));
    d.begin_tx_tick();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(-0.25)).unwrap(),
        TxBatchOutcome::Deferred { pending: 7 }
    );
    let r3 = take_tx_metrics();
    assert_eq!((r3.deferred, r3.pending), (7, 7));

    // Pre-expiry invalidation by E-stop counts as dropped_stopped (NOT dropped_stale).
    let req = Arc::new(AtomicU64::new(1));
    let app = Arc::new(AtomicU64::new(0));
    d.set_estop_gate(EstopGate::new(req, app));
    clock.advance(ms(10)); // 10 ms < 45 ms
    d.begin_tx_tick();
    assert!(is_estop_inhibited(&d.drain_pending_tx().unwrap_err()));
    let r4 = take_tx_metrics();
    assert_eq!((r4.dropped_stale, r4.dropped_stopped, r4.pending), (0, 7, 0));
    assert!(bus.with(|s| s.tx_log.is_empty()));

    // Part C: absolute deadline budget cutoff (max_duration = 1 ms, tx_step = 400 us -> 3 frames start at 0, 400, 800 us).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(1), frame_max_age: ms(45) });
    bus.with(|s| s.default_step = us(400));
    reset_tx_metrics();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.2)).unwrap(),
        TxBatchOutcome::Deferred { pending: 4 }
    );
    let md = take_tx_metrics();
    assert_eq!((md.frames, md.deferred, md.pending), (3, 4, 4));
    // Next tick (10 ms later, fresh): remaining 4 frames complete when deadline allows.
    bus.with(|s| s.default_step = us(100));
    clock.advance(ms(10));
    d.begin_tx_tick();
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
    assert_eq!(bus.with(|s| s.tx_accepted.len()), 7);
}

#[test]
fn shared_tick_budget_refresh_and_early_recovery_backpressure_close_pass() {
    // Part A: 0xCC refreshes in receive_feedback() share max_frames with write_joint_targets_bounded().
    let (mut d, bus, clock) = enabled_driver();
    d.set_torque(false).unwrap(); // disabled -> stale >= 100 ms sends 0xCC refreshes (no recovery holds)
    bus.clear();
    d.set_tx_budget(TxBudget { max_frames: 6, max_duration: ms(5), frame_max_age: ms(45) });
    // Advance 120 ms with only J1..J3 refreshed -> J4..J7 (4 motors) are stale >= 100 ms.
    clock.advance(ms(120));
    bus.with(|s| {
        s.push_feedback(0);
        s.push_feedback(1);
        s.push_feedback(2);
    });
    reset_tx_metrics();
    d.read_joint_states().unwrap();
    let refresh_count = bus.with(|s| s.tx_accepted.iter().filter(|f| f.id == 0x7FF).count());
    assert_eq!(refresh_count, 4, "4 stale motors refreshed in receive_feedback");

    // In the same tick, only 2 of the 6 attempts remain for target writes.
    let out = d.write_joint_targets_bounded(&targets_const(0.1)).unwrap();
    assert_eq!(out, TxBatchOutcome::Deferred { pending: 5 });
    let m = take_tx_metrics();
    assert_eq!(m.frames, 6, "combined refresh (4) + target (2) bounded by max_frames = 6");
    assert_eq!(m.pending, 5);

    // Part B: early recovery ENOBUFS closes the ENTIRE shared pass (zero subsequent recovery,
    // refresh, or target transmit attempts even with spare attempts and time).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(5), frame_max_age: ms(45) });
    // Advance 120 ms (within 250 ms enable validity, >= 45 ms recovery threshold and >= 100 ms refresh threshold).
    clock.advance(ms(120));
    bus.clear();
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Enobufs, us(10)));
    });
    reset_tx_metrics();
    d.read_joint_states().unwrap();
    let attempts_after_rx = bus.with(|s| s.tx_log.len());
    assert_eq!(attempts_after_rx, 1, "first recovery ENOBUFS stops further recovery AND skips 0xCC refresh");

    let out = d.write_joint_targets_bounded(&targets_const(0.2)).unwrap();
    assert_eq!(out, TxBatchOutcome::Deferred { pending: 7 });
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        1,
        "zero target transmit attempts after early recovery backpressure in the same tick"
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.enobufs, m.deferred, m.pending), (0, 1, 7, 7));
}

#[test]
fn mit_partial_batch_budget_backpressure_and_oversized_preflight_trip_group() {
    fn mit_driver() -> (DamiaoArmDriver<VirtualBus>, VirtualBus, MockClock) {
        let (mut d, bus, clock) = enabled_driver();
        let hold = targets_const(0.0);
        let modes = [
            JointMode::Position,
            JointMode::Position,
            JointMode::Position,
            JointMode::Mit,
            JointMode::Position,
            JointMode::Position,
            JointMode::Position,
        ];
        d.set_joint_modes(&modes, &hold).unwrap();
        d.set_mit_deadline_fence(Some(ms(45)), ms(300));
        bus.clear();
        reset_tx_metrics();
        (d, bus, clock)
    }

    // A: oversized MIT batch preflights BEFORE frame 0 (0 frames sent, fence tripped).
    let (mut d, bus, _) = mit_driver();
    d.set_tx_budget(TxBudget { max_frames: 5, max_duration: ms(5), frame_max_age: ms(45) });
    let err = d.write_joint_targets_bounded(&targets_const(0.1)).unwrap_err();
    assert!(err.to_string().contains("MIT deadline fence tripped"), "{err}");
    assert!(bus.with(|s| s.tx_log.is_empty()), "preflight must reject oversized MIT batch before frame 0");
    assert!(d.mit_fence_trip().is_some());

    // B: ENOBUFS on J4 trips group and invalidates cached frames (J5..J7 never sent).
    let (mut d, bus, clock) = mit_driver();
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::Enobufs, Duration::ZERO));
    });
    assert!(d.write_joint_targets_bounded(&targets_const(0.1)).is_err());
    assert_eq!(bus.with(|s| s.tx_log.len()), 4);
    assert!(d.mit_fence_trip().is_some());
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.enobufs), (3, 1));
    // Cached frames invalidated: later reads/writes send nothing.
    bus.clear();
    for _ in 0..3 {
        clock.advance(ms(50));
        let _ = d.read_joint_states();
        assert!(d.write_joint_targets_bounded(&targets_const(0.1)).is_err());
    }
    assert!(bus.with(|s| s.tx_log.is_empty()));

    // C: deadline exhaustion mid-batch trips group and invalidates cached frames.
    let (mut d, bus, _) = mit_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: us(500), frame_max_age: ms(45) });
    bus.with(|s| s.default_step = us(300));
    assert!(d.write_joint_targets_bounded(&targets_const(0.1)).is_err());
    assert_eq!(bus.with(|s| s.tx_log.len()), 2, "frame 0 at 0 us, frame 1 at 300 us -> 600 us >= 500 us cutoff");
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("TX budget exhausted")));
}

#[test]
fn flush_backpressure_and_failure_are_fail_closed_without_replay() {
    for flush_rule in [TxRule::WouldBlock, TxRule::Enobufs, TxRule::HardError] {
        let (mut d, bus, _) = enabled_driver();
        bus.with(|s| s.flush_error = Some(flush_rule));
        let res = d.write_joint_targets_bounded(&targets_const(0.2));
        assert!(res.is_err(), "flush {flush_rule:?} must fail closed, got {res:?}");
        assert_eq!(d.pending_tx_len(), 0, "submitted frames must not be re-queued after flush failure");
        bus.clear();
        d.begin_tx_tick();
        assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
        assert!(bus.with(|s| s.tx_log.is_empty()), "no replay of already-submitted frames");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn per_frame_timing_unequal_durations_exposed_in_diagnostics_tick_http_json() {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits = b601_limits();
    let (bus, _clock) = VirtualBus::new(&limits);
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    driver.read_joint_states().unwrap();
    driver.set_torque(true).expect("enable");
    bus.clear();

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(driver)));
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        inner.tick(0.02);
        inner.set_torque(true).expect("arm");
    }
    bus.clear();
    // Warm up one tick.
    bus.with(|s| s.push_all_feedback());
    sup.tick(0.02).await;

    // Script deliberately unequal per-frame wall times (150 us, 900 us, 200 us, 100 us, 100 us, 100 us, 100 us)
    // plus CPU spin work so CLOCK_THREAD_CPUTIME_ID registers nonzero time on Linux.
    bus.with(|s| {
        s.push_all_feedback();
        s.cpu_spin_iters = 50_000;
        for d in [150u64, 900, 200, 100, 100, 100, 100] {
            s.tx_script.push_back((TxRule::Ok, us(d)));
        }
    });
    sup.tick(0.02).await;

    // Next tick: inject 1 ENOBUFS on J2 (250 us) to verify enobufs counter in HTTP JSON.
    bus.with(|s| {
        s.push_all_feedback();
        s.cpu_spin_iters = 0;
        s.tx_script.push_back((TxRule::Ok, us(80)));
        s.tx_script.push_back((TxRule::Enobufs, us(250)));
    });
    sup.tick(0.02).await;

    let grasp = armd::grasp::GraspDeps::http(DetectorUpstream::from_env(), None);
    let router = create_router_full(
        sup.clone(),
        DetectorUpstream::from_env(),
        grasp,
        DiagnosticsState::default(),
        armd::OperatorAuth::disabled(),
    );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });

    let body: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();

    // Per-frame max must equal the 900 us frame (distinct from the 1650 us sum).
    assert_eq!(body["tx"]["frame_max_us"]["max"], 900, "{body}");
    assert_eq!(body["tx"]["send_us"]["max"], 1650, "{body}");
    assert_eq!(body["tx"]["frames"]["max"], 7, "{body}");
    assert_eq!(body["tx"]["enobufs_total"], 1, "{body}");
    assert_eq!(body["tx"]["since_boot_enobufs"], 1, "{body}");
    assert!(body["tx"]["frames_total"].as_u64().unwrap() >= 8, "{body}");
    assert_eq!(body["max_tx"]["tx_frame_max_us"], 900, "{body}");
    assert_eq!(body["max_tx"]["tx_send_us"], 1650, "{body}");
    assert_eq!(body["max_tx"]["tx_frames"], 7, "{body}");
    assert!(body["tx"]["send_cpu_us"]["max"].is_u64(), "{body}");
    assert!(body["tx"]["frame_cpu_max_us"]["max"].is_u64(), "{body}");
    #[cfg(target_os = "linux")]
    assert!(
        body["tx"]["send_cpu_us"]["max"].as_u64().unwrap() > 0,
        "expected nonzero CLOCK_THREAD_CPUTIME_ID send time on Linux: {body}"
    );
}

#[test]
fn legacy_supervisor_fails_closed_on_backpressured_final_release_batch() {
    // Verify that untouched legacy SupervisorInner never marks a gripper release action done
    // or clears held/releasing when the final tick's CAN TX hits ENOBUFS.
    let mut cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    cfg.joint_tracking = Some(armd::tracking::JointTrackingConfig {
        enabled: true,
        ..armd::tracking::JointTrackingConfig::rebot_dm()
    });
    let open_rad = cfg.gripper_policy.as_ref().unwrap().open_rad;
    let limits = b601_limits();
    let (bus, _clock) = VirtualBus::new(&limits);
    // Start J7 right next to open_rad so the gripper Open step finishes on the very next tick
    // (producing DeferredActionDone with clear_releasing = true).
    bus.with(|s| {
        s.motors[6].2 = open_rad;
        s.push_all_feedback();
    });
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    driver.read_joint_states().unwrap();
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    for _ in 0..3 {
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }

    // Start a gripper Open (release) while holding an object; inject ENOBUFS on the final tick.
    let epoch = inner.safety.epoch_id();
    inner.releasing = true;
    inner
        .start_gripper(
            GripperRequest {
                command: GripperCommand::Open,
                fraction: None,
                grasp: false,
                speed: 5.0,
                timeout_s: None,
                tag: None,
            },
            epoch,
        )
        .expect("start gripper open");

    // Inject ENOBUFS on every transmit on the final tick where the gripper open reaches Done.
    bus.with(|s| {
        s.push_all_feedback();
        s.default_rule = TxRule::Enobufs;
    });
    inner.tick(0.02);

    assert!(
        matches!(inner.safety.state(), SafetyState::Fault(_)),
        "legacy bridge must fail closed on deferred TX: {:?}",
        inner.safety.state()
    );
    assert_eq!(inner.safety.fault().map(|f| f.code), Some(FaultCode::DriverWriteFailed));
    let action = inner.last_action.as_ref().unwrap();
    assert_ne!(action.state, "done", "backpressured final release must not report done: {action:?}");
    assert!(inner.releasing, "backpressured final release must not clear releasing flag");
}

#[test]
fn wrapper_default_bounded_write_fails_closed_and_flush_estop_drops_pending() {
    // Part A: ZeroOffsetDriver uses ArmDriver::write_joint_targets_bounded default impl.
    // Because DamiaoArmDriver::write_joint_targets discards pending frames on TxDeferred,
    // the default impl must propagate Err(TxDeferred) instead of falsely returning Ok(Deferred).
    let (mut d, bus, _clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 3, max_duration: ms(5), frame_max_age: ms(45) });
    let mut wrapped = armd::joint_zero::ZeroOffsetDriver::new(Box::new(d), &[0.0; 7]);
    reset_tx_metrics();
    let err = wrapped.write_joint_targets_bounded(&targets_const(0.2)).unwrap_err();
    assert!(matches!(err, ArmDriverError::TxDeferred { pending: 4 }), "{err:?}");
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.deferred, m.dropped_stopped, m.pending), (3, 4, 4, 0));
    assert_eq!(bus.with(|s| s.tx_accepted.len()), 3);

    // Part B: E-stop arriving during flush() after partial send + WouldBlock drops remaining pending frames.
    let (mut d, bus, _) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    d.set_estop_gate(EstopGate::new(req.clone(), app));
    let req_hook = req.clone();
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::WouldBlock, Duration::ZERO));
        s.on_flush = Some(Arc::new(move || {
            req_hook.fetch_add(1, Ordering::SeqCst);
        }));
    });
    reset_tx_metrics();
    let err = d.write_joint_targets_bounded(&targets_const(0.3)).unwrap_err();
    assert!(is_estop_inhibited(&err), "{err:?}");
    assert_eq!(d.pending_tx_len(), 0, "E-stop during flush must clear pending queue");
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.would_block, m.dropped_stopped, m.pending), (1, 1, 6, 0));

    // Part C: clear_fault() drops pending motion frames; read_joint_states() preserves tx_pending gauge.
    let (mut d, bus, _clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.2)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    reset_tx_metrics();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    let m_rx = take_tx_metrics();
    assert_eq!(m_rx.pending, 3, "RX-only tick keeps accurate tx_pending gauge");
    assert_eq!(m_rx.deferred, 0);

    reset_tx_metrics();
    d.clear_fault().unwrap();
    assert_eq!(d.pending_tx_len(), 0);
    let m_cf = take_tx_metrics();
    assert_eq!((m_cf.dropped_stopped, m_cf.pending), (3, 0));
}

#[test]
fn same_tick_budget_persists_across_multiple_calls_and_enobufs_retry() {
    // Part A: within a single receive tick, write_joint_targets_bounded + drain_pending_tx +
    // another write share the exact same max_frames and backpressure closure.
    let (mut d, bus, _) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    bus.clear();
    reset_tx_metrics();

    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.2)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    assert_eq!(bus.with(|s| s.tx_log.len()), 4);

    // Same tick (no read_joint_states / begin_tx_tick): drain_pending_tx and a second write must
    // make ZERO additional transmit attempts because max_frames (4) is already exhausted.
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Deferred { pending: 3 });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(-0.2)).unwrap(),
        TxBatchOutcome::Deferred { pending: 7 }
    );
    assert_eq!(bus.with(|s| s.tx_log.len()), 4, "no extra attempts in the same tick");

    // Part B: immediate retry after ENOBUFS in the same tick stays closed (0 extra attempts),
    // even when max_frames and max_duration still have spare headroom.
    let (mut d, bus, _) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(5), frame_max_age: ms(45) });
    bus.with(|s| {
        s.push_all_feedback();
        s.tx_script.push_back((TxRule::Ok, us(10)));
        s.tx_script.push_back((TxRule::Enobufs, us(10)));
    });
    d.read_joint_states().unwrap();
    bus.clear();
    reset_tx_metrics();

    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Deferred { pending: 6 }
    );
    assert_eq!(bus.with(|s| s.tx_log.len()), 2);

    // Immediate drain in the same tick must NOT probe the bus again.
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Deferred { pending: 6 });
    assert_eq!(bus.with(|s| s.tx_log.len()), 2, "ENOBUFS closure must persist within the tick");

    // Next receive tick opens a fresh pass and delivers the remaining 6 frames.
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
    assert_eq!(bus.with(|s| s.tx_accepted.len()), 7);
}

#[test]
fn stop_during_final_send_or_flush_fails_closed_for_unfenced_and_fenced_mit() {
    // Part A: unfenced batch fully transmitted, stop raised during flush() -> Err(EstopInhibited), never Complete.
    let (mut d, bus, _) = enabled_driver();
    let stops = StopRequests::new();
    d.set_estop_gate(
        EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
            .with_stops(stops.clone()),
    );
    let stops_hook = stops.clone();
    bus.with(|s| {
        s.on_flush = Some(Arc::new(move || {
            let _ = stops_hook.request();
        }));
    });
    let err = d.write_joint_targets_bounded(&targets_const(0.25)).unwrap_err();
    assert!(is_estop_inhibited(&err), "stop during final flush must not return Complete: {err:?}");
    assert_eq!(bus.with(|s| s.tx_accepted.len()), 7);
    assert_eq!(d.pending_tx_len(), 0);

    // Part B: fenced MIT batch fully transmitted, stop raised during final transmit -> trips fence,
    // invalidates cached frames, and returns Err(EstopInhibited), never Complete.
    let (mut d, bus, clock) = enabled_driver();
    let hold = targets_const(0.0);
    let modes = [
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
        JointMode::Mit,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
    ];
    d.set_joint_modes(&modes, &hold).unwrap();
    d.set_mit_deadline_fence(Some(ms(45)), ms(300));
    let stops = StopRequests::new();
    d.set_estop_gate(
        EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
            .with_stops(stops.clone()),
    );
    bus.clear();
    let stops_hook = stops.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move |idx, _| {
            if idx == 6 {
                let _ = stops_hook.request();
            }
        }));
    });
    let err = d.write_joint_targets_bounded(&targets_const(0.1)).unwrap_err();
    assert!(is_estop_inhibited(&err), "fenced stop during final frame must fail closed: {err:?}");
    assert!(
        d.mit_fence_trip().is_some_and(|r| r.contains("E-stop")),
        "fenced MIT stop during final frame must trip fence: {:?}",
        d.mit_fence_trip()
    );
    // Cached output invalidated: later ticks send nothing even after stop applied.
    stops.mark_applied(stops.current());
    bus.with(|s| s.on_tx = None);
    bus.clear();
    clock.advance(ms(50));
    let _ = d.read_joint_states();
    assert!(d.write_joint_targets_bounded(&targets_const(0.1)).is_err());
    assert!(bus.with(|s| s.tx_log.is_empty()));
}

#[test]
fn receive_recovery_obeys_stop_gate_and_refresh_flush_failure_propagates() {
    // Part A: receive-side recovery with cached MIT output obeys stop raised during first recovery send.
    let (mut d, bus, clock) = enabled_driver();
    let hold = targets_const(0.0);
    let modes = [
        JointMode::Mit,
        JointMode::Mit,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
    ];
    d.set_joint_modes(&modes, &hold).unwrap();
    // Stage a pending frame on a partial write so we can also confirm recovery stop drops pending motion.
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.15)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let stops = StopRequests::new();
    d.set_estop_gate(
        EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
            .with_stops(stops.clone()),
    );
    // Advance 50 ms (>= 45 ms recovery threshold, < 100 ms refresh threshold) with all feedback stale.
    clock.advance(ms(50));
    bus.clear();
    let stops_hook = stops.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move |idx, _| {
            if idx == 0 {
                let _ = stops_hook.request();
            }
        }));
    });
    d.read_joint_states().unwrap();
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        1,
        "stop raised during first MIT recovery send must inhibit remaining recovery frames"
    );
    assert_eq!(d.pending_tx_len(), 0, "stop during recovery must clear pending motion queue");

    // Already-pending stop inhibits ALL recovery frames (0 sent).
    bus.with(|s| s.on_tx = None);
    bus.clear();
    clock.advance(ms(50));
    d.read_joint_states().unwrap();
    assert!(
        bus.with(|s| s.tx_log.iter().all(|f| f.id == 0x7FF)),
        "already-pending stop must inhibit every recovery command: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // Part B: 0xCC refresh flush failure propagates Err, closes the shared pass, and drops pending motion.
    for flush_rule in [TxRule::WouldBlock, TxRule::Enobufs, TxRule::HardError] {
        let (mut d, bus, clock) = enabled_driver();
        d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
        assert_eq!(
            d.write_joint_targets_bounded(&targets_const(0.1)).unwrap(),
            TxBatchOutcome::Deferred { pending: 3 }
        );
        d.set_torque(false).unwrap(); // disabled -> stale >= 100 ms sends 0xCC refreshes only
        // Re-queue pending motion frames isn't needed while disabled, verify pass closure + Err propagation.
        clock.advance(ms(120));
        bus.clear();
        bus.with(|s| s.flush_error = Some(flush_rule));
        let err = d.read_joint_states().unwrap_err();
        match flush_rule {
            TxRule::WouldBlock => assert!(matches!(err, ArmDriverError::Transport(TransportError::WouldBlock)), "{err:?}"),
            TxRule::Enobufs => assert!(matches!(err, ArmDriverError::Transport(TransportError::NoBufferSpace)), "{err:?}"),
            TxRule::HardError => assert!(matches!(err, ArmDriverError::Transport(TransportError::Can(_))), "{err:?}"),
            TxRule::Ok => unreachable!(),
        }
        // Pass must be closed in the same tick: a subsequent target write makes 0 new TX attempts.
        let attempts_after_rx = bus.with(|s| s.tx_log.len());
        let out = d.write_joint_targets_bounded(&targets_const(0.2)).unwrap();
        assert_eq!(out, TxBatchOutcome::Deferred { pending: 7 });
        assert_eq!(
            bus.with(|s| s.tx_log.len()),
            attempts_after_rx,
            "refresh flush failure must close the tick pass against subsequent target writes"
        );
    }
}

#[test]
fn preflight_rejection_trips_mit_fence_and_clears_pending_queue() {
    // Part A: with MIT fence armed, wrong joint count trips fence & clears cached output while
    // preserving JointCountMismatch.
    let (mut d, bus, _) = enabled_driver();
    let hold = targets_const(0.0);
    let modes = [
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
        JointMode::Mit,
        JointMode::Position,
        JointMode::Position,
        JointMode::Position,
    ];
    d.set_joint_modes(&modes, &hold).unwrap();
    d.set_mit_deadline_fence(Some(ms(45)), ms(300));
    bus.clear();
    let err = d.write_joint_targets_bounded(&targets_const(0.1)[..3]).unwrap_err();
    assert!(matches!(err, ArmDriverError::JointCountMismatch { expected: 7, actual: 3 }), "{err:?}");
    assert!(d.mit_fence_trip().is_some(), "preflight rejection under MIT fence must trip the group");
    assert!(bus.with(|s| s.tx_log.is_empty()));

    // Part B: deferred bounded batch followed by legacy write_joint_targets with wrong count
    // discards the previously queued pending frames while returning JointCountMismatch.
    let (mut d, bus, _) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.2)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    assert_eq!(d.pending_tx_len(), 3);
    d.begin_tx_tick();
    reset_tx_metrics();
    let err = d.write_joint_targets(&targets_const(0.2)[..2]).unwrap_err();
    assert!(matches!(err, ArmDriverError::JointCountMismatch { expected: 7, actual: 2 }), "{err:?}");
    assert_eq!(d.pending_tx_len(), 0, "preflight rejection must invalidate carried pending frames");
    let m = take_tx_metrics();
    assert_eq!((m.dropped_stopped, m.pending), (3, 0));
    bus.clear();
    d.begin_tx_tick();
    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
    assert!(bus.with(|s| s.tx_log.is_empty()));
}

#[test]
fn frames_expiring_during_flush_drop_stale_and_return_timeout() {
    // Frame 0 succeeds, frame 1 hits WouldBlock, then successful flush() advances clock to 45 ms:
    // remaining 6 current-batch frames expire during flush -> dropped as stale, queue empty,
    // and write_joint_targets_bounded fails closed with Timeout (never returning Deferred).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(5), frame_max_age: ms(45) });
    let clock_hook = clock.clone();
    bus.with(|s| {
        s.tx_script.push_back((TxRule::Ok, Duration::ZERO));
        s.tx_script.push_back((TxRule::WouldBlock, Duration::ZERO));
        s.on_flush = Some(Arc::new(move || {
            clock_hook.advance(ms(45));
        }));
    });
    reset_tx_metrics();
    let err = d.write_joint_targets_bounded(&targets_const(0.3)).unwrap_err();
    assert!(matches!(err, ArmDriverError::Transport(TransportError::Timeout)), "{err:?}");
    assert_eq!(d.pending_tx_len(), 0, "expired frames must not remain queued after flush");
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.would_block, m.dropped_stale, m.pending), (1, 1, 6, 0));
}
