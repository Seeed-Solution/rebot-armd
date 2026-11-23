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
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::api::{create_router_full, proxy::DetectorUpstream};
use armd::diagnostics::tick::{reset_tx_metrics, take_tx_metrics};
use armd::diagnostics::DiagnosticsState;
use armd::driver::{
    is_estop_inhibited, ArmDriver, ArmDriverError, DamiaoArmDriver, GripperPosForce, JointMode,
    JointTarget, TxBatchOutcome, TxBudget,
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
    // Part A: authorized stop (StopRequests via EstopGate::with_stops) arriving mid-batch drops pending frames.
    let (mut d, bus, clock) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let stops = StopRequests::new();
    d.set_estop_gate(EstopGate::new(req.clone(), app.clone()).with_stops(stops.clone()));
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
    d.set_estop_gate(EstopGate::new(req.clone(), app));
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

    // Part C: deferred targets MUST NOT survive an already-applied ordinary stop (requested == applied),
    // wired strictly through set_estop_gate (same path as production).
    let (mut d, bus, clock) = enabled_driver();
    let req = Arc::new(AtomicU64::new(0));
    let app = Arc::new(AtomicU64::new(0));
    let stops = StopRequests::new();
    d.set_estop_gate(EstopGate::new(req.clone(), app.clone()).with_stops(stops.clone()));
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
    d.set_estop_gate(EstopGate::new(req.clone(), app.clone()));
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
    d.set_estop_gate(EstopGate::new(req, app).with_stops(stops.clone()));
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
fn production_set_estop_gate_through_zero_offset_driver_invalidates_deferred_on_applied_stop() {
    // Thin Arc<Mutex<DamiaoArmDriver>> adapter so ZeroOffsetDriver wraps the real DamiaoArmDriver
    // and forwards set_estop_gate exactly as production does (`SupervisorInner::with_driver` +
    // `driver.set_estop_gate(inner.estop_gate())`).
    struct SharedDamiao(Arc<Mutex<DamiaoArmDriver<VirtualBus>>>);
    impl ArmDriver for SharedDamiao {
        fn capabilities(&self) -> armd::driver::DriverCapabilities { self.0.lock().unwrap().capabilities() }
        fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, ArmDriverError> {
            self.0.lock().unwrap().read_joint_states()
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.0.lock().unwrap().write_joint_targets(targets)
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            self.0.lock().unwrap().set_torque(enabled)
        }
        fn set_estop_gate(&mut self, gate: EstopGate) {
            self.0.lock().unwrap().set_estop_gate(gate);
        }
    }

    // Case 1: SupervisorInner::estop_gate() style (E-stop requested + applied, no StopRequests).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    let shared = Arc::new(Mutex::new(d));
    let mut wrapped = armd::joint_zero::ZeroOffsetDriver::new(Box::new(SharedDamiao(shared.clone())), &[0.0; 7]);
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let mut inner = SupervisorInner::new(cfg);
    // Wire the exact EstopGate built by SupervisorInner::estop_gate() through ZeroOffsetDriver.
    wrapped.set_estop_gate(inner.estop_gate());

    assert_eq!(
        shared.lock().unwrap().write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    // Request AND apply E-stop on SupervisorInner the exact way ArmSupervisor::trigger_e_stop +
    // tick does so requested == applied (gate.pending() == false, generations == (1, 0)).
    inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    assert!(inner.estop_gate().pending(), "E-stop pending before tick applies it");
    inner.tick(0.02);
    assert!(!inner.estop_gate().pending(), "E-stop is already applied (requested == applied)");
    assert_eq!(inner.estop_gate().generations(), (1, 0));
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(10));
    shared.lock().unwrap().begin_tx_tick();
    let err = shared.lock().unwrap().drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "production-wired E-stop must invalidate deferred frames: {err:?}");
    assert!(bus.with(|s| s.tx_log.is_empty()), "no target frame may be sent after applied E-stop");
    assert_eq!(shared.lock().unwrap().pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.pending), (0, 3, 0));

    // Case 2: SupervisorInner::estop_gate().with_stops(stops) through ZeroOffsetDriver.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    let shared = Arc::new(Mutex::new(d));
    let mut wrapped = armd::joint_zero::ZeroOffsetDriver::new(Box::new(SharedDamiao(shared.clone())), &[0.0; 7]);
    let stops = StopRequests::new();
    let inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    let gate = inner.estop_gate().with_stops(stops.clone());
    wrapped.set_estop_gate(gate.clone());

    assert_eq!(
        shared.lock().unwrap().write_joint_targets_bounded(&targets_const(0.35)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let g = stops.request();
    stops.mark_applied(g);
    assert!(!gate.pending(), "authorized stop is already applied (requested == applied)");
    assert_eq!(gate.generations(), (0, 1));
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(10));
    shared.lock().unwrap().begin_tx_tick();
    let err = shared.lock().unwrap().drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "production-wired StopRequests must invalidate deferred frames: {err:?}");
    assert!(bus.with(|s| s.tx_log.is_empty()), "no target frame may be sent after applied stop");
    assert_eq!(shared.lock().unwrap().pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.pending), (0, 3, 0));
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
fn expired_carryover_on_fresh_write_returns_expired_never_complete() {
    // Part A: defer 3 frames, advance 45 ms, raise budget, submit a fresh full 7-joint batch ->
    // write_joint_targets_bounded MUST return Expired { dropped: 3, pending: 0 }, never Complete.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.25)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(45));
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(5), frame_max_age: ms(45) });
    d.begin_tx_tick();
    let out = d.write_joint_targets_bounded(&targets_const(-0.25)).unwrap();
    assert_eq!(
        out,
        TxBatchOutcome::Expired { dropped: 3, pending: 0 },
        "fresh write after carried-over expiry must not report Complete"
    );
    assert_eq!(d.pending_tx_len(), 0);
    let sent_ids: Vec<(u32, f32)> = bus.with(|s| {
        s.tx_accepted
            .iter()
            .map(|f| (f.id, pos_vel_target_rad(f)))
            .collect()
    });
    assert_eq!(
        sent_ids,
        vec![
            (0x101, -0.25),
            (0x102, -0.25),
            (0x103, -0.25),
            (0x104, -0.25),
            (0x105, -0.25),
            (0x106, -0.25),
            (0x107, -0.25),
        ],
        "fresh batch transmits all 7 current targets after dropping expired carry-over"
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.deferred, m.pending), (7, 3, 0, 0));

    // Part B: same expired carry-over followed by a fresh batch under constrained budget (max_frames = 4) ->
    // reports Expired { dropped: 3, pending: 3 } so both stale loss and remaining pending count are visible.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.25)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(45));
    d.begin_tx_tick();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(-0.25)).unwrap(),
        TxBatchOutcome::Expired { dropped: 3, pending: 3 }
    );
    assert_eq!(d.pending_tx_len(), 3);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.deferred, m.pending), (4, 3, 3, 3));

    // Part C: under legacy write_joint_targets, expired carry-over followed by a fresh full batch
    // maps Expired -> Err(TransportError::Timeout) (never Ok(())).
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.25)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(45));
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(5), frame_max_age: ms(45) });
    d.begin_tx_tick();
    let err = d.write_joint_targets(&targets_const(-0.25)).unwrap_err();
    assert!(
        matches!(err, ArmDriverError::Transport(TransportError::Timeout)),
        "legacy write_joint_targets must map Expired carry-over to Timeout: {err:?}"
    );
    assert_eq!(d.pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.pending), (7, 3, 0));
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

#[test]
fn refresh_round_robin_covers_all_stale_motors_without_starving_j7() {
    // All 7 motors stay silent (stale >= 100 ms) while TX budget allows only 1 refresh per pass.
    // Within 7 consecutive passes every motor 1..=7 (including J7) must receive a 0xCC refresh.
    let (mut d, bus, clock) = enabled_driver();
    d.set_torque(false).unwrap();
    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });

    // Suppress automatic 0xCC replies by making transmit drop them? VirtualBus pushes feedback
    // inside handle_frame on 0xCC; clear rx before each pass so all 7 motors stay stale!
    clock.advance(ms(120));
    bus.clear();
    let mut refreshed_ids = Vec::new();
    for pass in 0..7 {
        bus.with(|s| {
            s.rx.clear();
            s.tx_log.clear();
            s.tx_accepted.clear();
        });
        d.read_joint_states().unwrap();
        let pass_ids: Vec<u16> = bus.with(|s| {
            s.tx_accepted
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                .map(|f| u16::from_le_bytes([f.data[0], f.data[1]]))
                .collect()
        });
        assert_eq!(pass_ids.len(), 1, "pass {pass} must send exactly 1 refresh under max_frames=1: {pass_ids:?}");
        refreshed_ids.extend(pass_ids);
        clock.advance(ms(20));
    }
    assert_eq!(
        refreshed_ids,
        vec![1, 2, 3, 4, 5, 6, 7],
        "7 constrained passes must refresh every stale motor J1..J7 without starving J7"
    );

    // Immediately after the full 7-motor sweep finishes (20 ms < 100 ms cooldown), no extra 0xCC spam.
    bus.with(|s| {
        s.rx.clear();
        s.tx_log.clear();
        s.tx_accepted.clear();
    });
    d.read_joint_states().unwrap();
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "100 ms cooldown must apply once a full refresh sweep completes"
    );

    // After 100 ms cooldown, the next sweep starts again at J1 and again covers J1..J7 in 7 passes
    // even when passes are spaced 120 ms apart.
    clock.advance(ms(100));
    let mut second_round = Vec::new();
    for _ in 0..7 {
        bus.with(|s| {
            s.rx.clear();
            s.tx_log.clear();
            s.tx_accepted.clear();
        });
        d.read_joint_states().unwrap();
        let pass_ids: Vec<u16> = bus.with(|s| {
            s.tx_accepted
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
                .map(|f| u16::from_le_bytes([f.data[0], f.data[1]]))
                .collect()
        });
        assert_eq!(pass_ids.len(), 1);
        second_round.extend(pass_ids);
        clock.advance(ms(120));
    }
    assert_eq!(second_round, vec![1, 2, 3, 4, 5, 6, 7]);
}

#[test]
fn merge_check_production_supervisor_estop_gate_with_stops_via_zero_offset_driver() {
    // Merge-check test: wire the real DamiaoArmDriver through the production
    // `SupervisorInner::with_driver` (which wraps in `ZeroOffsetDriver`) and
    // `SupervisorInner::attach_stop_requests` so `SupervisorInner::estop_gate()` carries
    // `with_stops` without any manual `.with_stops(...)` in the test.
    struct SharedDamiao(Arc<Mutex<DamiaoArmDriver<VirtualBus>>>);
    impl ArmDriver for SharedDamiao {
        fn capabilities(&self) -> armd::driver::DriverCapabilities { self.0.lock().unwrap().capabilities() }
        fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, ArmDriverError> {
            self.0.lock().unwrap().read_joint_states()
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.0.lock().unwrap().write_joint_targets(targets)
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            self.0.lock().unwrap().set_torque(enabled)
        }
        fn set_estop_gate(&mut self, gate: EstopGate) {
            self.0.lock().unwrap().set_estop_gate(gate);
        }
    }

    // Path A: production `SupervisorInner::with_driver` (wraps in `ZeroOffsetDriver`) +
    // `SupervisorInner::attach_stop_requests` -> forwards `inner.estop_gate()` through `ZeroOffsetDriver`.
    let (mut d, bus, clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    let shared = Arc::new(Mutex::new(d));
    let stops = StopRequests::new();
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(SharedDamiao(shared.clone())));
    inner.attach_stop_requests(stops.clone());

    assert_eq!(
        shared.lock().unwrap().write_joint_targets_bounded(&targets_const(0.35)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    // Request and apply an ORDINARY authorized stop using SupervisorInner::authorized_stop()
    // before advancing the applied generation (matching src/supervisor.rs:3576-3577; E-stop
    // stays at generation 0 and unlatched).
    let g = stops.request();
    assert!(inner.estop_gate().pending(), "ordinary stop must be pending before applied");
    let _ = inner.authorized_stop();
    stops.mark_applied(g);
    assert!(!inner.estop_gate().pending(), "ordinary stop is applied (requested == applied)");
    assert_eq!(inner.estop_gate().generations(), (0, 1));
    assert_eq!(inner.requested_estop_count.load(Ordering::SeqCst), 0);

    bus.clear();
    reset_tx_metrics();
    clock.advance(ms(10));
    shared.lock().unwrap().begin_tx_tick();
    let err = shared.lock().unwrap().drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err), "applied ordinary stop via production SupervisorInner::estop_gate must invalidate deferred frames: {err:?}");
    assert!(bus.with(|s| s.tx_log.is_empty()), "no target frame may reach the bus after applied ordinary stop");
    assert_eq!(shared.lock().unwrap().pending_tx_len(), 0);
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stopped, m.pending), (0, 3, 0));

    // Path B: direct `ZeroOffsetDriver::set_estop_gate(inner.estop_gate())` using the gate
    // returned by `SupervisorInner::estop_gate()` (carrying `with_stops` from §1).
    let (mut d2, bus2, clock2) = enabled_driver();
    d2.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    let shared2 = Arc::new(Mutex::new(d2));
    let mut wrapped = armd::joint_zero::ZeroOffsetDriver::new(Box::new(SharedDamiao(shared2.clone())), &[0.0; 7]);
    wrapped.set_estop_gate(inner.estop_gate());

    assert_eq!(
        shared2.lock().unwrap().write_joint_targets_bounded(&targets_const(-0.35)).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    let g2 = stops.request();
    assert!(inner.estop_gate().pending());
    stops.mark_applied(g2);
    assert!(!inner.estop_gate().pending());
    assert_eq!(inner.estop_gate().generations(), (0, 2));

    bus2.clear();
    reset_tx_metrics();
    clock2.advance(ms(10));
    shared2.lock().unwrap().begin_tx_tick();
    let err2 = shared2.lock().unwrap().drain_pending_tx().unwrap_err();
    assert!(is_estop_inhibited(&err2), "ZeroOffsetDriver forwarded SupervisorInner::estop_gate must invalidate deferred frames: {err2:?}");
    assert!(bus2.with(|s| s.tx_log.is_empty()), "no target frame may reach the bus on drain");
    assert_eq!(shared2.lock().unwrap().pending_tx_len(), 0);
    let m2 = take_tx_metrics();
    assert_eq!(m2.frames, 0);
    assert_eq!((m2.frames, m2.dropped_stopped, m2.pending), (0, 3, 0));
}

#[test]
fn j7_pos_force_autonomous_recovery_obeys_estop_and_handshake_obeys_tx_budget() {
    fn pf_cfg() -> GripperPosForce {
        GripperPosForce {
            joint: 6,
            current_raw: 460,
            close_speed_rad_s: 0.30,
            closed_rad: 0.0,
            open_rad: -5.7,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        }
    }

    fn enabled_pos_force_driver() -> (DamiaoArmDriver<VirtualBus>, VirtualBus, MockClock) {
        let limits = b601_limits();
        let (bus, clock) = VirtualBus::new(&limits);
        bus.with(|s| {
            s.motors[6].2 = -1.0;
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_gripper_pos_force(pf_cfg())
            .unwrap();
        // Establish empty-queue boundary, then fresh J7 feedback so feedback_queue_bound[6] is fresh.
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.set_torque(true).expect("enable pos-force driver");
        // Establish fresh post-enable feedback within the 100 ms pos-force bound.
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        bus.clear();
        reset_tx_metrics();
        (d, bus, clock)
    }

    // Part A1 (positive control): at 55 ms (>= 45 ms recovery, <= 100 ms pos-force freshness),
    // autonomous recovery emits a capped 0x307 J7 recovery hold.
    let (mut d, bus, clock) = enabled_pos_force_driver();
    clock.advance(ms(55));
    d.read_joint_states().unwrap();
    let j7_holds: Vec<_> = bus.with(|s| s.tx_accepted.iter().filter(|f| f.id == 0x307).copied().collect());
    assert_eq!(j7_holds.len(), 1, "positive control must emit capped J7 0x307 recovery hold: {j7_holds:?}");

    // Part A2 (pending E-stop and pending ordinary stop via SupervisorInner::estop_gate()):
    // autonomous recovery MUST emit zero 0x307 (and zero 0x101..0x106) recovery frames.
    for trigger_estop in [true, false] {
        let (mut d, bus, clock) = enabled_pos_force_driver();
        let stops = StopRequests::new();
        let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
        inner.attach_stop_requests(stops.clone());
        d.set_estop_gate(inner.estop_gate());
        if trigger_estop {
            inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        } else {
            let _ = stops.request();
        }
        assert!(inner.estop_gate().pending());
        clock.advance(ms(55));
        bus.clear();
        d.read_joint_states().unwrap();
        let sent = bus.with(|s| s.tx_log.clone());
        assert!(
            sent.is_empty(),
            "trigger_estop={trigger_estop}: pending stop must inhibit ALL recovery frames including J7 0x307: {sent:?}"
        );
    }

    // Part A3 (mid-recovery stop raised AND applied on J1 recovery TX before reaching J7):
    // stop-stamp change breaks the recovery loop before J7 so zero 0x307 reaches the bus.
    let (mut d, bus, clock) = enabled_pos_force_driver();
    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());
    let stops_hook = stops.clone();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move |idx, _| {
            if idx == 0 {
                let g = stops_hook.request();
                stops_hook.mark_applied(g);
            }
        }));
    });
    clock.advance(ms(55));
    d.read_joint_states().unwrap();
    let sent = bus.with(|s| s.tx_log.clone());
    assert_eq!(sent.len(), 1, "applied stop during J1 recovery must stop J2..J7 immediately: {sent:?}");
    assert_eq!(sent[0].id, 0x101);
    assert!(!sent.iter().any(|f| f.id == 0x307), "J7 0x307 recovery hold must not be sent after mid-recovery stop");

    // Part B: TX budget bounds J7 handshake frames (`0xCC` refresh, `CTRL_MODE=4` restore, readback,
    // and pre-enable `0x307` hold). Under `max_frames = 0`, zero J7 handshake frames are sent; once
    // budget is restored, `set_torque(true)` completes cleanly.
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    bus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(pf_cfg())
        .unwrap();
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    // Age J7 past 100 ms so enable requires a J7 0xCC refresh handshake before CTRL_MODE=4 write.
    clock.advance(ms(120));
    d.set_tx_budget(TxBudget { max_frames: 0, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    let p = d.begin_torque(true).expect("begin_torque enters RefreshBeforeRestore pending");
    assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "max_frames = 0 must bound J7 0xCC handshake refresh to 0 frames"
    );
    // Poll once still under max_frames = 0: still 0 frames sent.
    clock.advance(ms(10));
    let p2 = d.poll_torque(clock.now() + ms(5)).expect("still pending within 50 ms refresh deadline");
    assert!(matches!(p2, armd::driver::TorqueProgress::Pending { .. }));
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "max_frames = 0 must keep J7 handshake frames bounded across polls"
    );
    // Restore budget within the refresh deadline -> handshake sends 0xCC, restores CTRL_MODE=4,
    // reads back, sends 0x307 pre-enable hold, and confirms enable.
    d.set_tx_budget(TxBudget::default());
    let mut confirmed = false;
    for _ in 0..20 {
        clock.advance(ms(2));
        match d.poll_torque(clock.now() + ms(5)).expect("poll_torque with restored budget") {
            armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed) => {
                confirmed = true;
                break;
            }
            armd::driver::TorqueProgress::Pending { .. } => {}
            other => panic!("unexpected torque outcome: {other:?}"),
        }
    }
    assert!(confirmed, "J7 handshake should complete once TX budget is restored");
    assert!(bus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC)));
    assert!(bus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)));
}

#[test]
fn deferred_j7_opening_command_rebuilds_with_close_speed_cap_when_feedback_passes_target() {
    // Blocker regression: defer an opening J7 command (allowed higher speed = 2.0 rad/s, raw 200),
    // then deliver fresh feedback placing J7 past the target so the command is now CLOSING.
    // Draining MUST rebuild the J7 frame from live feedback and cap wire speed at close_speed_rad_s
    // (0.30 rad/s -> raw 30), never replaying the cached opening-speed bytes.
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    bus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
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
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("enable");
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();

    // Admission at q = -1.0 toward target = -2.0 (opening toward open_rad = -5.7 at 2.0 rad/s).
    d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    let mut t = targets_const(0.0);
    t[6] = JointTarget { position: -2.0, velocity: Some(2.0), ..JointTarget::position(-2.0) };
    assert_eq!(
        d.write_joint_targets_bounded(&t).unwrap(),
        TxBatchOutcome::Deferred { pending: 3 }
    );
    assert!(!bus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)), "J7 must be deferred");

    // Before draining, fresh feedback arrives showing J7 moved to -2.5 (past target -2.0 -> now CLOSING).
    clock.advance(ms(10));
    bus.with(|s| {
        s.motors[6].2 = -2.5;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    bus.clear();

    assert_eq!(d.drain_pending_tx().unwrap(), TxBatchOutcome::Complete);
    let j7_frames: Vec<_> = bus.with(|s| s.tx_accepted.iter().filter(|f| f.id == 0x307).copied().collect());
    assert_eq!(j7_frames.len(), 1, "expected 1 J7 0x307 frame on drain: {j7_frames:?}");
    let wire_vel_raw = u16::from_le_bytes([j7_frames[0].data[4], j7_frames[0].data[5]]);
    assert_eq!(
        wire_vel_raw, 30,
        "deferred J7 frame must rebuild from latest feedback with close_speed_rad_s cap (raw 30, not opening 200)"
    );
}

#[test]
fn enable_and_retry_frames_obey_tx_budget_across_polls() {
    // Major 1 regression: with max_frames = 1 throughout the entire PosForce enable handshake
    // (CTRL_MODE=4 restore, readback, 0x307 pre-enable hold, 7 initial enable frames, and retries),
    // NO single poll may transmit more than 1 frame, and enable still completes across polls.
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    bus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
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
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();

    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    let p = d.begin_torque(true).expect("begin_torque under max_frames=1");
    assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
    assert!(
        bus.with(|s| s.tx_log.len()) <= 1,
        "begin_torque exceeded max_frames=1: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    let mut confirmed = false;
    let mut total_enables = 0usize;
    for step in 0..30 {
        bus.clear();
        clock.advance(ms(1));
        match d.poll_torque(clock.now() + ms(5)).expect("poll_torque under max_frames=1") {
            armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed) => {
                assert!(
                    bus.with(|s| s.tx_log.len()) <= 1,
                    "step {step} exceeded max_frames=1"
                );
                confirmed = true;
                break;
            }
            armd::driver::TorqueProgress::Pending { .. } => {
                let sent = bus.with(|s| s.tx_log.clone());
                assert!(
                    sent.len() <= 1,
                    "step {step} exceeded max_frames=1 (sent {} frames): {sent:?}",
                    sent.len()
                );
                total_enables += sent.iter().filter(|f| f.data == damiao::CMD_ENABLE).count();
            }
            other => panic!("unexpected torque outcome: {other:?}"),
        }
    }
    assert!(confirmed, "enable handshake must carry remaining enable frames across max_frames=1 polls and confirm");
    assert_eq!(total_enables, 7, "all 7 motors enabled 1 frame per poll");
}

#[test]
fn mode_handover_preflights_ctrl_mode_plus_hold_atomically_against_tx_budget() {
    // Major 2 regression: when only 1 TX slot remains (enough for CTRL_MODE=4 but NOT its
    // required 0x307 hold), BOTH set_joint_modes and set_joint_modes_confirmed MUST refuse
    // BEFORE transmitting CTRL_MODE=4 (0 frames on the bus).
    let limits = b601_limits();
    let (bus, _clock) = VirtualBus::new(&limits);
    bus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
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
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("enable");
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();

    let mut mit_modes = [JointMode::Position; 7];
    mit_modes[6] = JointMode::Mit;
    let hold = targets_const(-1.0);
    d.set_joint_modes(&mit_modes, &hold).expect("switch J7 to MIT");

    let pos_modes = [JointMode::Position; 7];
    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });

    bus.clear();
    assert!(d.set_joint_modes(&pos_modes, &hold).is_err());
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "set_joint_modes must not send CTRL_MODE=4 without TX budget room for 0x307 hold: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // Restore J7 mode tracking back to Mit and verify set_joint_modes_confirmed too.
    d.set_tx_budget(TxBudget::default());
    d.set_joint_modes(&mit_modes, &hold).expect("re-enter MIT");
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    assert!(d.set_joint_modes_confirmed(&pos_modes, &hold).is_err());
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "set_joint_modes_confirmed must not send CTRL_MODE=4 without TX budget room for 0x307 hold: {:?}",
        bus.with(|s| s.tx_log.clone())
    );
}

#[test]
fn pending_stop_inhibits_round_robin_refresh_and_confirmed_mode_handover() {
    // Major 3 regression:
    // Part A: round-robin 0xCC refresh does not transmit while a stop is pending, and resumes
    // once the stop is applied.
    let (mut d, bus, clock) = enabled_driver();
    d.set_torque(false).unwrap();
    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());

    let g = stops.request();
    assert!(inner.estop_gate().pending());
    clock.advance(ms(120));
    bus.clear();
    d.read_joint_states().unwrap();
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "pending stop must inhibit 0xCC refresh sweep: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // Once applied (requested == applied), read-only 0xCC telemetry refresh resumes.
    let _ = inner.authorized_stop();
    stops.mark_applied(g);
    assert!(!inner.estop_gate().pending());
    bus.clear();
    d.read_joint_states().unwrap();
    assert!(
        bus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC)),
        "0xCC refresh should resume once stop is applied"
    );

    // Part B: set_joint_modes_confirmed under pending stop fails closed with EstopInhibited and 0 frames.
    let (mut d, bus, _) = enabled_driver();
    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());
    let _ = stops.request();
    let mut mit_modes = [JointMode::Position; 7];
    mit_modes[3] = JointMode::Mit;
    bus.clear();
    let err = d.set_joint_modes_confirmed(&mit_modes, &targets_const(0.0)).unwrap_err();
    assert!(is_estop_inhibited(&err), "confirmed mode handover must obey pending stop: {err:?}");
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "no frames may be sent during confirmed mode handover under pending stop: {:?}",
        bus.with(|s| s.tx_log.clone())
    );
}

#[test]
fn plain_mode_switch_refuses_pending_or_latched_stop_and_rollback_requires_fresh_measured_hold() {
    // Part A: Pending stop + plain set_joint_modes forward switch -> Err(EstopInhibited) and 0 frames.
    let (mut d, bus, _) = enabled_driver();
    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());
    let _ = stops.request();
    let mut mit_modes = [JointMode::Position; 7];
    mit_modes[3] = JointMode::Mit;
    bus.clear();
    let err = d.set_joint_modes(&mit_modes, &targets_const(0.0)).unwrap_err();
    assert!(is_estop_inhibited(&err), "pending stop must refuse plain set_joint_modes: {err:?}");
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "pending stop + set_joint_modes must send 0 frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // Part B: Latched E-stop (abort_flag = true, estop_gate NOT pending) + forward switch ->
    // BOTH plain set_joint_modes and set_joint_modes_confirmed refuse with 0 frames.
    let (mut d, bus, _) = enabled_driver();
    d.set_abort_flag(Arc::new(AtomicBool::new(true)));
    bus.clear();
    let err_plain = d.set_joint_modes(&mit_modes, &targets_const(0.0)).unwrap_err();
    assert!(is_estop_inhibited(&err_plain), "latched E-stop must refuse plain forward switch: {err_plain:?}");
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "latched E-stop + plain forward switch must send 0 frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );
    let err_conf = d.set_joint_modes_confirmed(&mit_modes, &targets_const(0.0)).unwrap_err();
    assert!(is_estop_inhibited(&err_conf), "latched E-stop must refuse confirmed forward switch: {err_conf:?}");
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "latched E-stop + confirmed forward switch must send 0 frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // Part C: Base-mode rollback while stopped/latched:
    // (C1) Stale feedback (> 100 ms) -> sends 0 frames and fails closed.
    // (C2) Fresh feedback at measured q = 0.42 while caller passes bogus hold 1.75 ->
    //      allowed, and the transmitted POS_VEL hold targets the fresh measured 0.42 (NOT 1.75).
    let (mut d, bus, clock) = enabled_driver();
    // Establish empty-queue boundary + fresh feedback, then switch J4 into MIT.
    d.read_joint_states().unwrap();
    bus.with(|s| {
        s.motors[3].2 = 0.42;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    d.set_joint_modes(&mit_modes, &targets_const(0.42)).expect("enter MIT on J4");

    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());
    d.set_abort_flag(Arc::new(AtomicBool::new(true)));
    let _ = stops.request();

    // C1: Age J4 feedback past 100 ms -> base-mode rollback MUST send 0 frames and fail closed.
    clock.advance(ms(120));
    bus.clear();
    let pos_modes = [JointMode::Position; 7];
    let err_stale = d.set_joint_modes(&pos_modes, &targets_const(1.75)).unwrap_err();
    assert!(
        err_stale.to_string().contains("fresh feedback"),
        "expected stale rollback failure, got {err_stale:?}"
    );
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "base-mode rollback without fresh feedback must send 0 frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // C2: Deliver fresh feedback at measured q = 0.42 -> base-mode rollback succeeds and overrides
    // caller's stale/arbitrary 1.75 hold with the fresh measured 0.42 position.
    d.read_joint_states().unwrap(); // refresh empty-queue boundary (no 0xCC sent while stop pending)
    bus.with(|s| {
        s.motors[3].2 = 0.42;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    bus.clear();
    d.set_joint_modes(&pos_modes, &targets_const(1.75))
        .expect("base-mode rollback with fresh feedback must succeed");
    let tx = bus.with(|s| s.tx_accepted.clone());
    let hold_frames: Vec<_> = tx.iter().filter(|f| f.id == 0x104).copied().collect();
    assert_eq!(hold_frames.len(), 1, "expected 1 POS_VEL hold frame for J4: {tx:?}");
    let wire_pos = f32::from_le_bytes([
        hold_frames[0].data[0],
        hold_frames[0].data[1],
        hold_frames[0].data[2],
        hold_frames[0].data[3],
    ]);
    assert!(
        (wire_pos - 0.42).abs() < 1e-2,
        "rollback hold must use fresh measured position ~0.42, not caller 1.75 (got {wire_pos})"
    );
}

#[test]
fn confirmed_handover_and_mit_fresh_status_obey_per_send_stop_gate_and_tx_budget() {
    // Part A: Stop raised during the FIRST confirmation refresh send in `write_modes_confirmed`
    // (`set_joint_modes_confirmed`) -> NO further refresh frames sent, fails closed with EstopInhibited.
    // Wrap `VirtualBus` to drop immediate feedback frames produced by the MIT hold frames for J1 & J2
    // so `write_modes_confirmed` enters its confirmation 0xCC refresh loop with `missing = [0, 1]`.
    struct DropHoldFeedbackBus {
        inner: VirtualBus,
        drop_mit_feedback_for: Vec<u32>,
    }
    impl CanTransport for DropHoldFeedbackBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn supports_nonblocking_rt(&self) -> bool { true }
        fn supports_bounded_flush(&self) -> bool { true }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let rx_before = self.inner.with(|s| s.rx.len());
            let r = self.inner.transmit(frame);
            if self.drop_mit_feedback_for.contains(&frame.id)
                && frame.data != damiao::CMD_ENABLE
                && frame.data != damiao::CMD_DISABLE
                && frame.data != damiao::CMD_CLEAR_FAULTS
            {
                self.inner.with(|s| s.rx.truncate(rx_before));
            }
            r
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    }

    let limits = b601_limits();
    let (vbus, _clock) = VirtualBus::new(&limits);
    let wrapper = DropHoldFeedbackBus { inner: vbus.clone(), drop_mit_feedback_for: vec![1, 2] };
    let mut d = DamiaoArmDriver::new(wrapper, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    vbus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("enable");
    d.check_mapping_ranges();
    vbus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();

    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());

    let stops_hook = stops.clone();
    vbus.clear();
    vbus.with(|s| {
        s.on_tx = Some(Arc::new(move |_, frame| {
            if frame.id == 0x7FF && frame.data[2] == 0xCC {
                let _ = stops_hook.request();
            }
        }));
    });
    let mut mit_two = [JointMode::Position; 7];
    mit_two[0] = JointMode::Mit;
    mit_two[1] = JointMode::Mit;
    let err = d.set_joint_modes_confirmed(&mit_two, &targets_const(0.0)).unwrap_err();
    assert!(is_estop_inhibited(&err), "stop during first confirmation refresh must fail closed with EstopInhibited: {err:?}");
    let refreshes: Vec<_> = vbus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .copied()
            .collect()
    });
    assert_eq!(
        refreshes.len(),
        1,
        "stop raised during first confirmation refresh must send 0 further refresh frames: {refreshes:?}"
    );
    assert!(d.joint_modes_uncertain(), "aborted confirmed handover must leave modes uncertain");

    // Part B: Stop raised during FIRST refresh send in `mit_fresh_status` -> NO further refresh frames.
    let (mut d, bus, _) = enabled_driver();
    let stops = StopRequests::new();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.attach_stop_requests(stops.clone());
    d.set_estop_gate(inner.estop_gate());
    let stops_hook = stops.clone();
    bus.clear();
    bus.with(|s| {
        s.on_tx = Some(Arc::new(move |_, frame| {
            if frame.id == 0x7FF && frame.data[2] == 0xCC {
                let _ = stops_hook.request();
            }
        }));
    });
    let err = d.mit_fresh_status().unwrap_err();
    assert!(
        err.contains(armd::driver::ESTOP_INHIBITED),
        "mit_fresh_status must stop on mid-loop stop: {err}"
    );
    let refreshes: Vec<_> = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == 0xCC)
            .copied()
            .collect()
    });
    assert_eq!(
        refreshes.len(),
        1,
        "stop raised during first mit_fresh_status refresh must send 0 further refresh frames: {refreshes:?}"
    );

    // Part C: TX budget bounds BOTH non-pos-force holds in confirmed handover AND `mit_fresh_status`.
    let (mut d, bus, _) = enabled_driver();
    d.check_mapping_ranges();
    d.set_tx_budget(TxBudget { max_frames: 2, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    let err = d.mit_fresh_status().unwrap_err();
    assert!(err.contains("TX budget exhausted"), "mit_fresh_status must respect TX budget: {err}");
    assert_eq!(
        bus.with(|s| s.tx_log.len()),
        2,
        "mit_fresh_status must stop after max_frames = 2"
    );

    // Non-pos-force confirmed handover with max_frames = 1 (room for CTRL_MODE but not its hold)
    // must send 0 frames and fail closed.
    let (mut d, bus, _) = enabled_driver();
    d.check_mapping_ranges();
    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    let mut mit_j4 = [JointMode::Position; 7];
    mit_j4[3] = JointMode::Mit;
    assert!(d.set_joint_modes_confirmed(&mit_j4, &targets_const(0.0)).is_err());
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "confirmed handover for non-PF joint must respect TX budget (0 frames under max_frames=1): {:?}",
        bus.with(|s| s.tx_log.clone())
    );
}

#[test]
fn budget_1_completes_multi_joint_restore_readback_and_enable_within_deadline() {
    // Item 3a regression: with max_frames = 1 and a joint (J4 in MIT) needing CTRL_MODE + hold
    // frames alongside J7 PosForce restore + readback + pre-enable hold + 7 enable frames,
    // budget-limited restore frames are deferred (not marked unsent/failed), so enable completes
    // across several ticks within the existing deadline with <= 1 TX frame per poll.
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    bus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
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
    // Establish fresh feedback, enable once, switch J4 to MIT, then disable so J4 is stray (MIT)
    // and needs CTRL_MODE=POS_VEL + dual holds on the next enable alongside J7 PosForce!
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_torque(true).expect("initial enable");
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    let mut mit_j4 = [JointMode::Position; 7];
    mit_j4[3] = JointMode::Mit;
    d.set_joint_modes(&mit_j4, &targets_const(0.0)).expect("switch J4 to MIT");
    d.set_torque(false).expect("disable while J4 is in MIT");
    // Refresh feedback (torque off, state 0, fresh <= 100 ms).
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();

    // Now set max_frames = 1: J4 needs CTRL_MODE + 2 dual hold frames (3 frames > 1).
    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
    bus.clear();
    let p = d.begin_torque(true).expect("begin_torque under max_frames=1 with stray MIT joint");
    assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
    assert!(
        bus.with(|s| s.tx_log.len()) <= 1,
        "begin_torque exceeded max_frames=1: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    let mut confirmed = false;
    let mut saw_j4_ctrl_mode = false;
    let mut saw_j4_hold = false;
    let mut total_enables = 0usize;
    for step in 0..40 {
        let sent_prev = bus.with(|s| s.tx_log.clone());
        saw_j4_ctrl_mode |= sent_prev.iter().any(|f| {
            f.id == 0x7FF && f.data[0] == 4 && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_CTRL_MODE
        });
        saw_j4_hold |= sent_prev.iter().any(|f| f.id == 0x104);
        total_enables += sent_prev.iter().filter(|f| f.data == damiao::CMD_ENABLE).count();
        bus.clear();
        clock.advance(ms(1));
        match d.poll_torque(clock.now() + ms(5)).expect("poll_torque must defer, not fail, under max_frames=1") {
            armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed) => {
                let sent = bus.with(|s| s.tx_log.clone());
                assert!(sent.len() <= 1, "step {step} exceeded max_frames=1: {sent:?}");
                total_enables += sent.iter().filter(|f| f.data == damiao::CMD_ENABLE).count();
                confirmed = true;
                break;
            }
            armd::driver::TorqueProgress::Pending { .. } => {
                let sent = bus.with(|s| s.tx_log.clone());
                assert!(sent.len() <= 1, "step {step} exceeded max_frames=1: {sent:?}");
            }
            other => panic!("unexpected torque outcome: {other:?}"),
        }
    }
    assert!(confirmed, "healthy arm with stray MIT joint must complete enable across ticks under max_frames=1");
    assert!(saw_j4_ctrl_mode, "expected J4 CTRL_MODE=POS_VEL restore write");
    assert!(saw_j4_hold, "expected J4 POS_VEL hold frame during restore");
    assert_eq!(total_enables, 7, "all 7 motors enabled 1 frame per poll");
    assert!(d.is_enabled());
}

#[test]
fn carried_and_retried_j7_enable_fails_closed_when_j7_feedback_goes_stale() {
    // Item 3b regression:
    // Part A: Carried J7 CMD_ENABLE (under tight TX budget) whose J7 feedback ages > 100 ms
    // after the pre-enable 0x307 hold MUST NOT transmit J7 CMD_ENABLE and MUST fail closed.
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    bus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits.clone())
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
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
    d.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();

    // Run under max_frames = 1 until the 0x307 pre-enable hold is sent (leaving all 7 enable
    // frames carried in `unsent_mask`).
    d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
    let _ = d.begin_torque(true).expect("begin");
    let mut saw_pre_enable_hold = false;
    for _ in 0..10 {
        bus.clear();
        clock.advance(ms(1));
        let p = d.poll_torque(clock.now() + ms(5)).expect("poll before hold");
        assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        if bus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
            saw_pre_enable_hold = true;
            break;
        }
    }
    assert!(saw_pre_enable_hold, "expected 0x307 pre-enable hold before carried enable");
    // Drop the immediate feedback frame generated by VirtualBus in response to 0x307 so no newer
    // J7 feedback arrives, then advance clock across carried J1..J6 polls past 100 ms before J7.
    bus.with(|s| s.rx.clear());
    clock.advance(ms(105));
    bus.clear();
    // Increase budget so J1..J6 and the stale check for J7 happen in one poll_enable pass.
    d.set_tx_budget(TxBudget::default());
    let err = d.poll_torque(clock.now() + ms(5)).unwrap_err();
    assert!(
        matches!(err, ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: 6 }),
        "carried stale J7 enable must fail closed with PosForceFeedbackStaleBeforeTransmit: {err:?}"
    );
    assert!(
        !bus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
        "stale carried J7 enable frame MUST NOT be transmitted: {:?}",
        bus.with(|s| s.tx_log.clone())
    );
    assert!(!d.torque_transition_pending() && !d.is_enabled());

    // Part B: Retried J7 CMD_ENABLE (at 75 ms retry after initial J7 enable reply was lost)
    // when J7 feedback has aged > 100 ms MUST NOT retransmit J7 CMD_ENABLE and MUST fail closed.
    struct DropJ7EnableReplyBus {
        inner: VirtualBus,
        drop_j7_enable: Arc<AtomicBool>,
    }
    impl CanTransport for DropJ7EnableReplyBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn supports_nonblocking_rt(&self) -> bool { true }
        fn supports_bounded_flush(&self) -> bool { true }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let rx_before = self.inner.with(|s| s.rx.len());
            let r = self.inner.transmit(frame);
            if self.drop_j7_enable.load(Ordering::SeqCst)
                && ((frame.id == 7 && frame.data == damiao::CMD_ENABLE) || frame.id == 0x307)
            {
                self.inner.with(|s| s.rx.truncate(rx_before));
            }
            r
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    }

    let (vbus, clock2) = VirtualBus::new(&limits);
    vbus.with(|s| {
        s.motors[6].2 = -1.0;
    });
    let drop_j7 = Arc::new(AtomicBool::new(false));
    let wrapper = DropJ7EnableReplyBus { inner: vbus.clone(), drop_j7_enable: drop_j7.clone() };
    let mut d2 = DamiaoArmDriver::new(wrapper, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
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
    d2.read_joint_states().unwrap();
    vbus.with(|s| s.push_all_feedback());
    d2.read_joint_states().unwrap();
    // Age J7 to 40 ms before enable, drop J7 enable reply + 0x307 reply so initial enable sends at
    // age ~42 ms (fresh), retry 1 at +25 ms (age ~67 ms, fresh), retry 2 at +75 ms (age ~117 ms > 100 ms -> fail closed!).
    clock2.advance(ms(40));
    drop_j7.store(true, Ordering::SeqCst);
    let _ = d2.begin_torque(true).expect("begin");
    let mut saw_enable_batch = false;
    for _ in 0..10 {
        clock2.advance(ms(1));
        let p = d2.poll_torque(clock2.now() + ms(5)).expect("poll to enable");
        assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)) {
            saw_enable_batch = true;
            break;
        }
    }
    assert!(saw_enable_batch, "expected initial J7 CMD_ENABLE while J7 feedback is still fresh");
    // Drop the 0x307 feedback reply too if still sitting in RX so J7 stays aged from t=0.
    vbus.with(|s| s.rx.retain(|f| f.id != 0x17));
    // Advance +76 ms (past the 75 ms retry point and past 100 ms J7 feedback age, still < 150 ms enable confirm).
    clock2.advance(ms(76));
    vbus.clear();
    let err2 = d2.poll_torque(clock2.now() + ms(5)).unwrap_err();
    assert!(
        matches!(err2, ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: 6 }),
        "retried J7 enable after feedback aged > 100 ms must fail closed: {err2:?}"
    );
    assert!(
        !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
        "stale retried J7 enable frame MUST NOT be transmitted: {:?}",
        vbus.with(|s| s.tx_log.clone())
    );
    assert!(!d2.torque_transition_pending() && !d2.is_enabled());
}
