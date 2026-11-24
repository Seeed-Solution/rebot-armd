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
    motor_faults: Vec<u8>, // non-zero overrides feedback state nibble (e.g. 0x0A overcurrent)
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
        let motor_faults = vec![0u8; limits.len()];
        Self {
            motors,
            motor_faults,
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
        let state = if self.motor_faults[idx] != 0 {
            self.motor_faults[idx]
        } else {
            u8::from(enabled)
        };
        let p = damiao::float_to_uint(q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let data = [
            ((state & 0x0f) << 4) | (id as u8 & 0x0f),
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
            } else if frame.data == damiao::CMD_CLEAR_FAULTS {
                self.motor_faults[k] = 0;
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

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn supports_bounded_flush(&self) -> bool {
        true
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
        { let __s = inner.current_admission_stamp(); inner.enable_torque(__s) }.expect("arm");
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
    { let __s = inner.current_admission_stamp(); inner.enable_torque(__s) }.expect("arm");
    for _ in 0..3 {
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }

    // Start a gripper Open (release) while holding an object; inject ENOBUFS on the final tick.
    let epoch = inner.safety.epoch_id();
    inner.releasing = true;
    let stamp = inner.current_admission_stamp();
    inner
        .start_gripper(
            stamp,
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
    bus.clear();
    // clear_fault() obeys the per-tick TX budget (max_frames = 4 on 7 motors -> 4 sent, 3 deferred)
    // while still dropping all pending motion frames immediately.
    let cf_err = d.clear_fault().unwrap_err();
    assert!(
        matches!(cf_err, ArmDriverError::TxDeferred { pending: 3 }),
        "clear_fault under max_frames=4 must defer remaining 3 motors: {cf_err:?}"
    );
    assert_eq!(bus.with(|s| s.tx_log.len()), 4, "clear_fault must respect max_frames=4");
    assert_eq!(d.pending_tx_len(), 0);
    let m_cf = take_tx_metrics();
    assert_eq!((m_cf.dropped_stopped, m_cf.pending), (3, 0));

    // With sufficient TX budget, clear_fault transmits all 7 clear-fault frames and succeeds.
    d.set_tx_budget(TxBudget::default());
    bus.clear();
    d.clear_fault().unwrap();
    assert_eq!(bus.with(|s| s.tx_log.len()), 7);
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

    // Part C: Latched E-stop + rollback:
    // - CTRL_MODE writes (both `set_joint_modes` and `set_joint_modes_confirmed` rollbacks) are
    //   `ModeWrite`: refused while pending OR latched with ZERO CTRL_MODE frames sent.
    // - Only a stationary hold frame (`write_estop_hold` -> `RollbackHold`) built from fresh
    //   (<= 100 ms) measured feedback may transmit while latched (stale feedback -> 0 frames).
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
    let pending_gen = stops.request();

    // C0: While a stop is PENDING (not yet applied), both mode rollback and estop hold are refused (0 frames).
    bus.clear();
    let pos_modes = [JointMode::Position; 7];
    let err_pending = d.set_joint_modes(&pos_modes, &targets_const(1.75)).unwrap_err();
    assert!(
        is_estop_inhibited(&err_pending),
        "pending stop must refuse base-mode rollback: {err_pending:?}"
    );
    let err_hold_pending = d.write_estop_hold(&targets_const(1.75)).unwrap_err();
    assert!(
        is_estop_inhibited(&err_hold_pending),
        "pending stop must refuse estop hold: {err_hold_pending:?}"
    );
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "pending stop must send 0 frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // Apply the stop so the gate is no longer pending while E-stop remains LATCHED.
    stops.mark_applied(pending_gen);

    // C1: Even with fresh feedback while LATCHED, rollback CTRL_MODE writes (`set_joint_modes` and
    // `set_joint_modes_confirmed`) are refused and transmit ZERO CTRL_MODE frames!
    d.read_joint_states().unwrap();
    bus.with(|s| {
        s.motors[3].2 = 0.42;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    bus.clear();
    let err_rb_plain = d.set_joint_modes(&pos_modes, &targets_const(0.42)).unwrap_err();
    assert!(
        is_estop_inhibited(&err_rb_plain),
        "latched E-stop must refuse rollback set_joint_modes: {err_rb_plain:?}"
    );
    let err_rb_conf = d.set_joint_modes_confirmed(&pos_modes, &targets_const(0.42)).unwrap_err();
    assert!(
        is_estop_inhibited(&err_rb_conf),
        "latched E-stop must refuse rollback set_joint_modes_confirmed: {err_rb_conf:?}"
    );
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "latched E-stop + rollback must send 0 CTRL_MODE frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // C2: Stale feedback (> 100 ms) while latched -> `write_estop_hold` sends 0 frames and fails closed.
    clock.advance(ms(120));
    bus.clear();
    assert!(
        d.write_estop_hold(&targets_const(1.75)).is_err(),
        "stale feedback must refuse write_estop_hold while latched"
    );
    assert!(
        bus.with(|s| s.tx_log.is_empty()),
        "stale write_estop_hold must send 0 frames: {:?}",
        bus.with(|s| s.tx_log.clone())
    );

    // C3: Fresh feedback at measured q = 0.42 while latched -> `write_estop_hold` succeeds,
    // emits 0 CTRL_MODE frames, and holds J4 at fresh measured ~0.42 (not caller's bogus 1.75).
    d.read_joint_states().unwrap(); // refresh empty-queue boundary (read-only 0xCC allowed while latched)
    bus.with(|s| {
        s.motors[3].2 = 0.42;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();
    bus.clear();
    d.write_estop_hold(&targets_const(1.75))
        .expect("write_estop_hold with fresh feedback while latched must succeed");
    let tx = bus.with(|s| s.tx_accepted.clone());
    assert!(
        !tx.iter().any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE),
        "latched hold must emit 0 CTRL_MODE register writes: {tx:?}"
    );
    let j4_mit: Vec<_> = tx.iter().filter(|f| f.id == 4).copied().collect();
    assert_eq!(j4_mit.len(), 1, "expected 1 MIT hold frame for J4: {tx:?}");
    let p_u = ((j4_mit[0].data[0] as u16) << 8) | (j4_mit[0].data[1] as u16);
    let lim = b601_limits()[3];
    let wire_pos = damiao::uint_to_float(p_u, lim.p_min, lim.p_max, 16);
    assert!(
        (wire_pos - 0.42).abs() < 1e-2,
        "latched hold must use fresh measured position ~0.42, not caller 1.75 (got {wire_pos})"
    );
}

#[test]
fn gravity_comp_recovery_rebuilds_mit_frame_fresh_and_refuses_stale() {
    let (mut d, bus, clock) = enabled_driver();
    let mut modes = [JointMode::Position; 7];
    modes[3] = JointMode::Mit; // J4 in gravity-comp MIT mode
    d.set_joint_modes(&modes, &targets_const(0.0)).unwrap();

    // Command J4 to target pos=1.25 with explicit MIT gains/torque (kp=12, kd=0.8, tau=1.5),
    // while the motor's actual measured feedback is at pos=0.30.
    let mut cmd = targets_const(0.0);
    cmd[3] = JointTarget {
        position: 1.25,
        velocity: Some(0.5),
        kp: Some(12.0),
        kd: Some(0.8),
        torque: Some(1.5),
    };
    d.write_joint_targets(&cmd).unwrap();
    // Establish an observed-empty receive boundary and fresh feedback at measured pos=0.30.
    d.read_joint_states().unwrap();
    bus.with(|s| {
        s.motors[3].2 = 0.30;
        s.push_all_feedback();
    });
    d.read_joint_states().unwrap();

    // Advance 50 ms (>= 45 ms recovery threshold, <= 100 ms freshness limit) with silent motors.
    clock.advance(ms(50));
    bus.clear();
    d.read_joint_states().unwrap();
    let j4_rec: Vec<_> = bus.with(|s| {
        s.tx_accepted.iter().filter(|f| f.id == 4).copied().collect()
    });
    assert_eq!(
        j4_rec.len(),
        1,
        "fresh gravity-comp recovery must send 1 rebuilt MIT hold frame for J4: {j4_rec:?}"
    );
    let data = j4_rec[0].data;
    let lim = b601_limits()[3];
    let p_u = ((data[0] as u16) << 8) | (data[1] as u16);
    let v_u = ((data[2] as u16) << 4) | ((data[3] as u16) >> 4);
    let kp_u = (((data[3] & 0x0f) as u16) << 8) | (data[4] as u16);
    let kd_u = ((data[5] as u16) << 4) | ((data[6] as u16) >> 4);
    let t_u = (((data[6] & 0x0f) as u16) << 8) | (data[7] as u16);
    let wire_p = damiao::uint_to_float(p_u, lim.p_min, lim.p_max, 16);
    let wire_v = damiao::uint_to_float(v_u, lim.v_min, lim.v_max, 12);
    let wire_kp = damiao::uint_to_float(kp_u, 0.0, 500.0, 12);
    let wire_kd = damiao::uint_to_float(kd_u, 0.0, 5.0, 12);
    let wire_t = damiao::uint_to_float(t_u, lim.t_min, lim.t_max, 12);
    assert!(
        (wire_p - 0.30).abs() < 1e-2,
        "recovery MIT frame must rebuild at fresh measured pos ~0.30, NOT cached target 1.25 (got {wire_p})"
    );
    assert!(
        wire_v.abs() < 2e-2,
        "recovery MIT hold must zero velocity, NOT replay cached 0.5 (got {wire_v})"
    );
    assert!((wire_kp - 12.0).abs() < 0.3, "expected preserved kp ~12.0, got {wire_kp}");
    assert!((wire_kd - 0.8).abs() < 0.05, "expected preserved kd ~0.8, got {wire_kd}");
    assert!((wire_t - 1.5).abs() < 0.05, "expected preserved torque ~1.5, got {wire_t}");

    // Advance another 60 ms (feedback age = 110 ms > 100 ms freshness limit) with silent motors:
    // recovery MUST fail closed and send ZERO MIT frames for J4.
    clock.advance(ms(60));
    bus.clear();
    d.read_joint_states().unwrap();
    let j4_stale_mit: Vec<_> = bus.with(|s| {
        s.tx_log.iter().filter(|f| f.id == 4).copied().collect()
    });
    assert!(
        j4_stale_mit.is_empty(),
        "stale (> 100 ms) gravity-comp recovery must send 0 MIT frames for J4: {j4_stale_mit:?}"
    );
}

#[test]
fn motor_fault_plus_latched_estop_clears_via_operator_clear_fault_and_refuses_pending_stop() {
    // Helper to build a SupervisorInner + DamiaoArmDriver in SafetyState::EStop (latched, !pending)
    // with a live hardware motor fault (0x0A overcurrent) on J3.
    let make_faulted_estop_supervisor = || {
        let limits = b601_limits();
        let (bus, clock) = VirtualBus::new(&limits);
        bus.with(|s| s.push_all_feedback());
        let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap();
        driver.read_joint_states().unwrap();
        let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
        inner.lease_enabled = false;
        // Initial healthy idle tick, then inject J3 fault 0x0A + latch an E-stop and apply it.
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        bus.with(|s| {
            s.motor_faults[2] = 0x0A; // J3 overcurrent fault
            s.push_all_feedback();
        });
        inner.e_stop_latched.store(true, Ordering::SeqCst);
        inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
        inner.tick(0.02);
        assert_eq!(*inner.safety.state(), SafetyState::EStop);
        assert!(inner.e_stop_latched.load(Ordering::SeqCst));
        assert!(!inner.estop_gate().pending(), "E-stop must be applied (latched, not pending)");
        // Ensure J3 fault feedback is freshly ingested in the driver.
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        (inner, bus, clock)
    };

    // Case A: Legacy `SupervisorInner::clear_fault()` resets motor fault while latched, then clears latch.
    let (mut inner, bus, _clock) = make_faulted_estop_supervisor();
    // Negative check: while a NEW stop is PENDING (requested > applied), clear_fault MUST refuse
    // and emit 0 CMD_CLEAR_FAULTS frames.
    bus.clear();
    inner.requested_estop_count.fetch_add(1, Ordering::SeqCst);
    assert!(inner.estop_gate().pending());
    let pending_err = inner.clear_fault().unwrap_err();
    assert!(
        bus.with(|s| !s.tx_log.iter().any(|f| f.data == damiao::CMD_CLEAR_FAULTS)),
        "pending stop must refuse CMD_CLEAR_FAULTS: err={pending_err:?}, tx={:?}",
        bus.with(|s| s.tx_log.clone())
    );
    // Apply the pending E-stop so gate is latched & !pending again.
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    assert!(!inner.estop_gate().pending());
    assert!(inner.e_stop_latched.load(Ordering::SeqCst));

    // Step 1: operator clear_fault while latched sends CMD_CLEAR_FAULTS to motors (including J3)
    // and keeps the fault/E-stop latched until fresh fault-free feedback is observed.
    bus.clear();
    let step1_err = inner.clear_fault().unwrap_err();
    assert!(
        step1_err.to_string().contains("Hardware reset sent"),
        "expected first clear_fault to send hardware reset and require fresh feedback, got {step1_err:?}"
    );
    let clear_frames: Vec<_> = bus.with(|s| {
        s.tx_accepted
            .iter()
            .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
            .copied()
            .collect()
    });
    assert_eq!(
        clear_frames.len(),
        7,
        "operator clear_fault while latched must transmit CMD_CLEAR_FAULTS to all 7 motors: {clear_frames:?}"
    );
    assert!(
        bus.with(|s| s.motor_faults[2] == 0),
        "motor J3 fault must be cleared by CMD_CLEAR_FAULTS"
    );
    assert!(
        inner.e_stop_latched.load(Ordering::SeqCst),
        "latch must remain set until fresh fault-free feedback is verified on retry"
    );

    // Next supervisor tick ingests the fault-free feedback replies from the reset while E-stop remains latched.
    inner.tick(0.02);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert!(inner.e_stop_latched.load(Ordering::SeqCst));

    // Step 2: retry operator clear_fault now that fresh fault-free feedback arrived -> Idle & unlatched!
    let final_state = inner.clear_fault().expect("second clear_fault after reset must clear E-stop latch");
    assert_eq!(final_state, SafetyState::Idle);
    assert_eq!(*inner.safety.state(), SafetyState::Idle);
    assert!(
        !inner.e_stop_latched.load(Ordering::SeqCst),
        "E-stop latch must be cleared after operator clear_fault succeeds"
    );

    // Case B: RT Executor `SupervisorInner::clear_fault_executor(deadline)` resets motor fault while
    // latched via `clear_fault_budgeted`, then clears the latch on the next call after Step 2 feedback.
    let (mut inner_rt, bus_rt, _clock_rt) = make_faulted_estop_supervisor();
    bus_rt.clear();
    let step1_rt = inner_rt
        .clear_fault_executor(std::time::Instant::now() + ms(50))
        .unwrap_err();
    assert!(
        step1_rt.to_string().contains("Hardware reset sent"),
        "expected clear_fault_executor to send budgeted hardware reset while latched, got {step1_rt:?}"
    );
    let clear_frames_rt: Vec<_> = bus_rt.with(|s| {
        s.tx_accepted
            .iter()
            .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
            .copied()
            .collect()
    });
    assert_eq!(
        clear_frames_rt.len(),
        7,
        "clear_fault_executor while latched must transmit CMD_CLEAR_FAULTS to all 7 motors: {clear_frames_rt:?}"
    );
    assert!(inner_rt.e_stop_latched.load(Ordering::SeqCst));
    // Step 2 tick ingests fresh fault-free feedback from all 7 motors.
    inner_rt.tick(0.02);
    let final_rt = inner_rt
        .clear_fault_executor(std::time::Instant::now() + ms(50))
        .expect("second clear_fault_executor after fresh feedback must clear E-stop latch");
    assert_eq!(final_rt, SafetyState::Idle);
    assert_eq!(*inner_rt.safety.state(), SafetyState::Idle);
    assert!(!inner_rt.e_stop_latched.load(Ordering::SeqCst));

    // Direct driver negative check: `clear_fault_budgeted` refuses while a stop is PENDING (0 TX).
    let (mut drv, bus_d, clock_d) = enabled_driver();
    let req = Arc::new(AtomicU64::new(1));
    let app = Arc::new(AtomicU64::new(0));
    drv.set_estop_gate(EstopGate::new(req, app));
    drv.set_abort_flag(Arc::new(AtomicBool::new(true)));
    bus_d.clear();
    let err = drv.clear_fault_budgeted(clock_d.now() + ms(10)).unwrap_err();
    assert!(is_estop_inhibited(&err), "pending stop must refuse clear_fault_budgeted: {err:?}");
    assert!(bus_d.with(|s| s.tx_log.is_empty()), "pending stop must emit 0 clear-fault frames");
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

/// Strip Rust `//` line comments, nested `/* ... */` block comments, string/raw-string contents,
/// and character literal contents while keeping all code tokens and braces intact.
fn strip_rust_comments_and_literals(src: &str) -> String {
    let bytes = src.as_bytes();
    let len = bytes.len();
    let mut out = String::with_capacity(len);
    let mut i = 0usize;
    while i < len {
        // Line comment: // ... \n
        if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'/') {
            i += 2;
            while i < len && bytes[i] != b'\n' {
                i += 1;
            }
            if i < len {
                out.push('\n');
                i += 1;
            }
            continue;
        }
        // Nested block comment: /* ... */
        if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'*') {
            let mut depth = 1usize;
            i += 2;
            while i < len && depth > 0 {
                if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'*') {
                    depth += 1;
                    i += 2;
                } else if bytes[i] == b'*' && bytes.get(i + 1) == Some(&b'/') {
                    depth -= 1;
                    i += 2;
                } else {
                    i += 1;
                }
            }
            out.push(' ');
            continue;
        }
        // Raw string literal: r"...", r#"..."#, br#"..."#
        let raw_prefix = if bytes[i] == b'b' && bytes.get(i + 1) == Some(&b'r') {
            Some(i + 2)
        } else if bytes[i] == b'r'
            && !i
                .checked_sub(1)
                .is_some_and(|p| bytes[p].is_ascii_alphanumeric() || bytes[p] == b'_')
        {
            Some(i + 1)
        } else {
            None
        };
        if let Some(mut j) = raw_prefix {
            let mut hashes = 0usize;
            while j < len && bytes[j] == b'#' {
                hashes += 1;
                j += 1;
            }
            if j < len && bytes[j] == b'"' {
                j += 1;
                while j < len {
                    if bytes[j] == b'"'
                        && j + hashes < len
                        && bytes[j + 1..j + 1 + hashes].iter().all(|&b| b == b'#')
                    {
                        j += 1 + hashes;
                        break;
                    }
                    j += 1;
                }
                out.push_str("\"\"");
                i = j;
                continue;
            }
        }
        // Regular string literal: "..." (or b"...")
        if bytes[i] == b'"' {
            i += 1;
            while i < len {
                if bytes[i] == b'\\' {
                    i += 2;
                    continue;
                }
                if bytes[i] == b'"' {
                    i += 1;
                    break;
                }
                i += 1;
            }
            out.push_str("\"\"");
            continue;
        }
        // Char literal (distinguished from lifetime `'a`): `'...'`
        if bytes[i] == b'\'' {
            if bytes.get(i + 1) == Some(&b'\\') {
                i += 2;
                while i < len {
                    if bytes[i] == b'\\' {
                        i += 2;
                        continue;
                    }
                    if bytes[i] == b'\'' {
                        i += 1;
                        break;
                    }
                    i += 1;
                }
                out.push_str("''");
                continue;
            }
            let mut chars = src[i + 1..].char_indices();
            if let (Some((_, _ch)), Some((end_idx, '\''))) = (chars.next(), chars.next()) {
                out.push_str("''");
                i = i + 1 + end_idx + 1;
                continue;
            }
        }
        let ch = src[i..].chars().next().unwrap();
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

/// Partition `dm_section` into `(outside_send_compact, send_body_compact)` after stripping
/// comments/literals and removing all ASCII whitespace so multi-line calls such as
/// `self.can\n    .transmit(&frame)` or `CanTransport::\n    transmit(...)` are always caught.
fn partition_damiao_send_body_compact(dm_section: &str) -> (String, String) {
    let clean = strip_rust_comments_and_literals(dm_section);
    let send_matches: Vec<usize> = clean.match_indices("fn send(").map(|(idx, _)| idx).collect();
    assert_eq!(
        send_matches.len(),
        1,
        "expected exactly one `fn send(` definition in DamiaoArmDriver, found {}",
        send_matches.len()
    );
    let send_start = send_matches[0];
    let body_open = clean[send_start..]
        .find('{')
        .map(|rel| send_start + rel)
        .expect("fn send body opening brace");
    let mut depth = 0usize;
    let mut body_end = None;
    for (rel, ch) in clean[body_open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    body_end = Some(body_open + rel);
                    break;
                }
            }
            _ => {}
        }
    }
    let body_end = body_end.expect("fn send body closing brace");
    let send_body = &clean[body_open..=body_end];
    let outside = format!("{}{}", &clean[..send_start], &clean[body_end + 1..]);
    let compact = |s: &str| -> String { s.chars().filter(|c| !c.is_ascii_whitespace()).collect() };
    (compact(&outside), compact(send_body))
}

#[test]
fn damiao_has_single_can_transmit_chokepoint_in_driver_rs() {
    // Self-check: verify the whitespace/comment normalizer catches multi-line `.transmit(` and
    // UFCS `::transmit(` splits while ignoring comments and string literals.
    let synthetic_split = r#"
        impl Foo {
            // self.can.transmit(&commented_out);
            /* self.can
                .transmit(&block_commented); */
            fn send(&mut self) {
                let _s = "self.can.transmit(&in_string)";
                self.can
                    .transmit(&frame);
            }
            fn sneaky_dot(&mut self) {
                self
                    .can
                    .transmit(&bad);
            }
            fn sneaky_ufcs(&mut self) {
                CanTransport::
                    transmit(&mut self.can, &bad);
            }
        }
    "#;
    let (syn_out, syn_in) = partition_damiao_send_body_compact(synthetic_split);
    assert_eq!(syn_in.matches(".transmit(").count(), 1);
    assert_eq!(syn_out.matches(".transmit(").count(), 1, "must catch multi-line .transmit(: {syn_out}");
    assert_eq!(syn_out.matches("::transmit(").count(), 1, "must catch multi-line ::transmit(: {syn_out}");

    // Production check over the entire Damiao driver section of src/driver.rs.
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/driver.rs"))
        .expect("read src/driver.rs");
    let dm_start = src
        .find("// 3. Damiao Brushless Arm Driver")
        .expect("Damiao driver section header");
    let rs_start = src
        .find("// 4. RobStride Actuator Arm Driver")
        .expect("RobStride driver section header");
    assert!(dm_start < rs_start);
    let dm_section = &src[dm_start..rs_start];
    let (outside_compact, send_compact) = partition_damiao_send_body_compact(dm_section);

    assert_eq!(
        outside_compact.matches(".transmit(").count(),
        0,
        "DamiaoArmDriver must have ZERO `.transmit(` calls outside `fn send`"
    );
    assert_eq!(
        outside_compact.matches("::transmit(").count(),
        0,
        "DamiaoArmDriver must have ZERO `::transmit(` UFCS calls outside `fn send`"
    );
    assert_eq!(
        send_compact.matches(".transmit(").count(),
        1,
        "DamiaoArmDriver::send must contain EXACTLY ONE `.transmit(` call"
    );
    assert_eq!(
        send_compact.matches("::transmit(").count(),
        0,
        "DamiaoArmDriver::send must not contain UFCS `::transmit(` calls"
    );
    assert!(
        send_compact.contains("self.can.transmit(&frame)"),
        "DamiaoArmDriver::send must transmit via `self.can.transmit(&frame)`"
    );

    // Ensure no legacy bypass or raw-frame helper exists in DamiaoArmDriver.
    let clean_dm = strip_rust_comments_and_literals(dm_section);
    for forbidden in [
        "fn transmit_instrumented",
        "fn transmit_budgeted_gated",
        "fn transmit_checked",
        "fn tx_mit_gated",
        "fn transmit_with_retry_gated",
        "fn transmit_with_retry",
        "fn retransmit(",
    ] {
        assert!(
            !clean_dm.contains(forbidden),
            "forbidden legacy/raw TX helper `{forbidden}` found in DamiaoArmDriver"
        );
    }
}

#[test]
fn production_damiao_operations_under_latched_estop_table() {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Outcome {
        RefusedZeroTx,
        AllowedTx,
    }

    let latch_driver = |d: &mut DamiaoArmDriver<VirtualBus>| {
        // Applied, latched E-stop: requested == applied == 1 (!pending), abort_flag == true.
        let req = Arc::new(AtomicU64::new(1));
        let app = Arc::new(AtomicU64::new(1));
        d.set_estop_gate(EstopGate::new(req, app));
        d.set_abort_flag(Arc::new(AtomicBool::new(true)));
    };

    let disabled_confirmed_driver = || {
        let (mut d, bus, clock) = enabled_driver();
        d.set_torque(false).expect("disable before test");
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        assert_eq!(d.enable_state(), armd::driver::EnableState::Disabled);
        (d, bus, clock)
    };

    type CaseFn = fn(
        &dyn Fn(&mut DamiaoArmDriver<VirtualBus>),
        &dyn Fn() -> (DamiaoArmDriver<VirtualBus>, VirtualBus, MockClock),
    ) -> (bool, Vec<CanFrame>);

    // Declarative table over every production public `DamiaoArmDriver` / `ArmDriver` operation
    // that can emit CAN frames, exercised under a latched E-stop (no synthetic `send()` calls).
    let table: &[(&str, Outcome, CaseFn, fn(&[CanFrame]))] = &[
        (
            "write_joint_targets",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let res = d.write_joint_targets(&targets_const(0.2));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "write_joint_targets_bounded (unfenced)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let res = d.write_joint_targets_bounded(&targets_const(0.2));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "write_joint_targets_bounded (fenced MIT)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                let mut modes = [JointMode::Position; 7];
                modes[3] = JointMode::Mit;
                d.set_joint_modes_confirmed(&modes, &targets_const(0.0)).unwrap();
                d.set_mit_deadline_fence(Some(ms(45)), ms(250));
                latch(&mut d);
                bus.clear();
                let res = d.write_joint_targets_bounded(&targets_const(0.2));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                assert!(d.mit_fence_trip().is_some(), "fenced MIT under latched E-stop must trip fence");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "drain_pending_tx (deferred targets queued before latch)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                d.set_tx_budget(TxBudget { max_frames: 4, max_duration: ms(5), frame_max_age: ms(45) });
                assert_eq!(
                    d.write_joint_targets_bounded(&targets_const(0.25)).unwrap(),
                    TxBatchOutcome::Deferred { pending: 3 }
                );
                latch(&mut d);
                bus.clear();
                d.begin_tx_tick();
                let res = d.drain_pending_tx();
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                assert_eq!(d.pending_tx_len(), 0, "latched drain must drop staged frames");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "begin_torque(true)",
            Outcome::RefusedZeroTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let res = d.begin_torque(true);
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "begin_torque_budgeted(true, 16)",
            Outcome::RefusedZeroTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let res = d.begin_torque_budgeted(true, 16);
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "set_joint_modes (forward PosVel->MIT)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let mut modes = [JointMode::Position; 7];
                modes[3] = JointMode::Mit;
                let res = d.set_joint_modes(&modes, &targets_const(0.0));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "set_joint_modes (rollback MIT->PosVel, fresh feedback)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                let mut modes = [JointMode::Position; 7];
                modes[3] = JointMode::Mit;
                d.set_joint_modes(&modes, &targets_const(0.0)).unwrap();
                latch(&mut d);
                d.read_joint_states().unwrap();
                bus.with(|s| s.push_all_feedback());
                d.read_joint_states().unwrap();
                bus.clear();
                let res = d.set_joint_modes(&[JointMode::Position; 7], &targets_const(0.0));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "set_joint_modes_confirmed (forward PosVel->MIT)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let mut modes = [JointMode::Position; 7];
                modes[3] = JointMode::Mit;
                let res = d.set_joint_modes_confirmed(&modes, &targets_const(0.0));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "set_joint_modes_confirmed (rollback MIT->PosVel, fresh feedback)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                let mut modes = [JointMode::Position; 7];
                modes[3] = JointMode::Mit;
                d.set_joint_modes_confirmed(&modes, &targets_const(0.0)).unwrap();
                latch(&mut d);
                d.read_joint_states().unwrap();
                bus.with(|s| s.push_all_feedback());
                d.read_joint_states().unwrap();
                bus.clear();
                let res = d.set_joint_modes_confirmed(&[JointMode::Position; 7], &targets_const(0.0));
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "set_zero_position",
            Outcome::RefusedZeroTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let res = d.set_zero_position(0);
                assert!(res.as_ref().is_err_and(is_estop_inhibited), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "commission_motor_timeout",
            Outcome::RefusedZeroTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let res = d.commission_motor_timeout(&[0], 2000);
                assert!(
                    res.as_ref().is_err_and(|e| is_estop_inhibited(&e.error)),
                    "{res:?}"
                );
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "read_joint_states (50 ms recovery window)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, clock) = enabled_driver();
                latch(&mut d);
                clock.advance(ms(50)); // >= 45 ms recovery, < 100 ms refresh
                bus.clear();
                let res = d.read_joint_states();
                assert!(res.is_ok(), "{res:?}");
                // Recovery TX is refused while latched (0 frames transmitted).
                (false, bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "recovery must emit 0 frames while latched: {tx:?}"),
        ),
        (
            "write_estop_hold (stale feedback > 100 ms)",
            Outcome::RefusedZeroTx,
            |latch, _| {
                let (mut d, bus, clock) = enabled_driver();
                latch(&mut d);
                clock.advance(ms(120));
                bus.clear();
                let res = d.write_estop_hold(&targets_const(0.5));
                assert!(res.is_err(), "stale write_estop_hold must fail closed: {res:?}");
                (res.is_ok(), bus.with(|s| s.tx_log.clone()))
            },
            |tx| assert!(tx.is_empty(), "{tx:?}"),
        ),
        (
            "write_estop_hold (fresh feedback <= 100 ms)",
            Outcome::AllowedTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                d.read_joint_states().unwrap();
                bus.with(|s| {
                    s.motors[0].2 = 0.33;
                    s.push_all_feedback();
                });
                d.read_joint_states().unwrap();
                bus.clear();
                let res = d.write_estop_hold(&targets_const(1.9));
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 stationary hold frames: {tx:?}");
                assert!(
                    tx.iter().all(|f| (0x101..=0x107).contains(&f.id)),
                    "expected POS_VEL hold frames only (0 CTRL_MODE): {tx:?}"
                );
                let j1_pos = f32::from_le_bytes([tx[0].data[0], tx[0].data[1], tx[0].data[2], tx[0].data[3]]);
                assert!((j1_pos - 0.33).abs() < 1e-3, "expected fresh measured ~0.33, got {j1_pos}");
            },
        ),
        (
            "read_joint_states (120 ms 0xCC refresh window)",
            Outcome::AllowedTx,
            |latch, mk_dis| {
                let (mut d, bus, clock) = mk_dis();
                latch(&mut d);
                clock.advance(ms(120));
                bus.clear();
                let res = d.read_joint_states();
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 refresh frames: {tx:?}");
                assert!(
                    tx.iter().all(|f| f.id == 0x7FF && f.data[2] == 0xCC),
                    "expected read-only 0xCC refresh frames only: {tx:?}"
                );
            },
        ),
        (
            "mit_fresh_status",
            Outcome::AllowedTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let res = d.mit_fresh_status();
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 status poll frames: {tx:?}");
                assert!(
                    tx.iter().all(|f| f.id == 0x7FF && f.data[2] == 0xCC),
                    "expected read-only 0xCC frames only: {tx:?}"
                );
            },
        ),
        (
            "read_motor_identity",
            Outcome::AllowedTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let res = d.read_motor_identity(&[0]);
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 8, "expected 8 identity register reads: {tx:?}");
                assert!(
                    tx.iter().all(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ),
                    "expected read-only REG_READ frames only: {tx:?}"
                );
            },
        ),
        (
            "check_mapping_ranges",
            Outcome::AllowedTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                latch(&mut d);
                bus.clear();
                let _ = d.check_mapping_ranges();
                (true, bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 28, "expected 4 register reads x 7 motors: {tx:?}");
                assert!(
                    tx.iter().all(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ),
                    "expected read-only REG_READ frames only: {tx:?}"
                );
            },
        ),
        (
            "clear_fault",
            Outcome::AllowedTx,
            |latch, mk_dis| {
                let (mut d, bus, _) = mk_dis();
                bus.with(|s| {
                    s.motor_faults[2] = 0x0A;
                    s.push_all_feedback();
                });
                d.read_joint_states().unwrap();
                latch(&mut d);
                bus.clear();
                let res = d.clear_fault();
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 CMD_CLEAR_FAULTS frames: {tx:?}");
                assert!(
                    tx.iter().all(|f| f.data == damiao::CMD_CLEAR_FAULTS),
                    "expected CMD_CLEAR_FAULTS frames only: {tx:?}"
                );
            },
        ),
        (
            "clear_fault_budgeted",
            Outcome::AllowedTx,
            |latch, mk_dis| {
                let (mut d, bus, clock) = mk_dis();
                bus.with(|s| {
                    s.motor_faults[2] = 0x0A;
                    s.push_all_feedback();
                });
                d.read_joint_states().unwrap();
                latch(&mut d);
                bus.clear();
                let res = d.clear_fault_budgeted(clock.now() + ms(10));
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 CMD_CLEAR_FAULTS frames: {tx:?}");
                assert!(
                    tx.iter().all(|f| f.data == damiao::CMD_CLEAR_FAULTS),
                    "expected CMD_CLEAR_FAULTS frames only: {tx:?}"
                );
            },
        ),
        (
            "mit_expiry_recover",
            Outcome::AllowedTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                bus.with(|s| {
                    s.motor_faults[3] = 0x0E;
                    s.push_all_feedback();
                });
                d.read_joint_states().unwrap();
                latch(&mut d);
                bus.clear();
                let res = d.mit_expiry_recover();
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert!(
                    tx.iter().any(|f| f.id == 4 && f.data == damiao::CMD_CLEAR_FAULTS),
                    "expected CMD_CLEAR_FAULTS for faulted J4: {tx:?}"
                );
                assert_eq!(
                    tx.iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
                    7,
                    "expected 7 CMD_DISABLE frames: {tx:?}"
                );
                assert!(
                    tx.iter().all(|f| f.data == damiao::CMD_CLEAR_FAULTS || f.data == damiao::CMD_DISABLE),
                    "mit_expiry_recover must only transmit CMD_CLEAR_FAULTS and CMD_DISABLE: {tx:?}"
                );
            },
        ),
        (
            "begin_torque(false)",
            Outcome::AllowedTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let res = d.begin_torque(false);
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 CMD_DISABLE frames: {tx:?}");
                assert!(tx.iter().all(|f| f.data == damiao::CMD_DISABLE), "{tx:?}");
            },
        ),
        (
            "begin_torque_budgeted(false, 16)",
            Outcome::AllowedTx,
            |latch, _| {
                let (mut d, bus, _) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let res = d.begin_torque_budgeted(false, 16);
                assert!(res.is_ok(), "{res:?}");
                (res.is_ok(), bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 CMD_DISABLE frames: {tx:?}");
                assert!(tx.iter().all(|f| f.data == damiao::CMD_DISABLE), "{tx:?}");
            },
        ),
        (
            "panic_disable_step",
            Outcome::AllowedTx,
            |latch, _| {
                let (mut d, bus, clock) = enabled_driver();
                latch(&mut d);
                bus.clear();
                let _ = d.panic_disable_step(true, clock.now() + ms(10));
                (true, bus.with(|s| s.tx_accepted.clone()))
            },
            |tx| {
                assert_eq!(tx.len(), 7, "expected 7 CMD_DISABLE frames: {tx:?}");
                assert!(tx.iter().all(|f| f.data == damiao::CMD_DISABLE), "{tx:?}");
            },
        ),
    ];

    let mut refused_checked = 0usize;
    let mut allowed_checked = 0usize;
    for &(name, expected, run, verify_tx) in table {
        let (ok, tx) = run(&latch_driver, &disabled_confirmed_driver);
        match expected {
            Outcome::RefusedZeroTx => {
                assert!(!ok, "production op `{name}` must refuse/emit no TX while E-stop is latched");
                assert!(tx.is_empty(), "production op `{name}` transmitted while latched: {tx:?}");
                refused_checked += 1;
            }
            Outcome::AllowedTx => {
                assert!(ok, "production op `{name}` must succeed while E-stop is latched");
                assert!(!tx.is_empty(), "production op `{name}` expected TX while latched");
                allowed_checked += 1;
            }
        }
        verify_tx(&tx);
    }
    assert_eq!(refused_checked + allowed_checked, table.len());
    assert_eq!(table.len(), 25, "expected all 25 production Damiao TX operations verified");
}
