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
use std::time::{Duration, Instant};

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
    rx: VecDeque<(CanFrame, Option<Instant>)>,
    tx_log: Vec<CanFrame>,
    tx_accepted: Vec<CanFrame>,
    tx_script: VecDeque<(TxRule, Duration)>,
    default_rule: TxRule,
    default_step: Duration,
    /// Optional periodic TX spike: every Nth transmit (when > 0) adds `tx_spike_step` to clock.
    tx_spike_every: usize,
    tx_spike_step: Duration,
    flush_error: Option<TxRule>,
    clock: MockClock,
    on_tx: Option<Arc<dyn Fn(usize, &CanFrame) + Send + Sync>>,
    on_flush: Option<Arc<dyn Fn() + Send + Sync>>,
    tx_attempts: usize,
    /// Optional busy-work loop per transmit to produce nonzero CLOCK_THREAD_CPUTIME_ID on Linux.
    cpu_spin_iters: u64,
    /// When true, `flush_is_noop()` returns `false` to model transports with a real flush.
    real_flush: bool,
    flush_calls: usize,
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
            tx_spike_every: 0,
            tx_spike_step: Duration::ZERO,
            flush_error: None,
            clock,
            on_tx: None,
            on_flush: None,
            tx_attempts: 0,
            cpu_spin_iters: 0,
            real_flush: false,
            flush_calls: 0,
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
        self.rx.push_back((CanFrame::new_standard(0x10 + id, &data).unwrap(), Some(self.clock.now())));
    }

    fn push_unsolicited_feedback(&mut self, i: usize) {
        self.push_feedback(i);
    }

    fn push_all_feedback(&mut self) {
        for i in 0..self.motors.len() {
            self.push_unsolicited_feedback(i);
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
                        self.rx.push_back((reply, Some(self.clock.now())));
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
                        self.rx.push_back((reply, Some(self.clock.now())));
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
            let (rule, mut step) = s
                .tx_script
                .pop_front()
                .unwrap_or((s.default_rule, s.default_step));
            let idx = s.tx_attempts;
            s.tx_attempts += 1;
            if s.tx_spike_every > 0 && (idx + 1).is_multiple_of(s.tx_spike_every) {
                step += s.tx_spike_step;
            }
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
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
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
            s.flush_calls += 1;
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

    fn flush_is_noop(&self) -> bool {
        !self.0.lock().unwrap().real_flush
    }
}

fn enabled_driver() -> (DamiaoArmDriver<VirtualBus>, VirtualBus, MockClock) {
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        arm_joints[i] = i;
    }
    d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
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

    // A stalled single transmit pushing the current batch past 45 ms must fail closed uniformly as
    // Expired { dropped: 6, pending: 0 } (never falsely reporting Complete), while legacy
    // write_joint_targets maps Expired to Timeout.
    let (mut d, bus, _) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(100), frame_max_age: ms(45) });
    bus.with(|s| s.tx_script.push_back((TxRule::Ok, ms(45))));
    reset_tx_metrics();
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Expired { dropped: 6, pending: 0 }
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.pending), (1, 6, 0));

    let (mut d, bus, _) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(100), frame_max_age: ms(45) });
    bus.with(|s| s.tx_script.push_back((TxRule::Ok, ms(45))));
    let err = d.write_joint_targets(&targets_const(0.3)).unwrap_err();
    assert!(matches!(err, ArmDriverError::Transport(TransportError::Timeout)), "{err:?}");
}

#[test]
fn expired_carryover_on_fresh_write_returns_expired_never_complete() {
    // Part A: defer 3 frames, advance 45 ms, raise budget, submit a fresh full 7-joint batch ->
    // write_joint_targets_bounded fails closed immediately with Expired { dropped: 3, pending: 0 }
    // and sends ZERO replacement motion frames in the same tick.
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
        "fresh write after carried-over expiry must fail closed as Expired"
    );
    assert_eq!(d.pending_tx_len(), 0);
    assert!(
        bus.with(|s| s.tx_accepted.is_empty()),
        "expired motion carry-over must not transmit replacement motion targets"
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.deferred, m.pending), (0, 3, 0, 0));

    // Part B: same expired carry-over under constrained budget (max_frames = 4) ->
    // fails closed with Expired { dropped: 3, pending: 0 } and zero replacement sends.
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
        TxBatchOutcome::Expired { dropped: 3, pending: 0 }
    );
    assert_eq!(d.pending_tx_len(), 0);
    assert!(bus.with(|s| s.tx_accepted.is_empty()));
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.deferred, m.pending), (0, 3, 0, 0));

    // Part C: under legacy write_joint_targets, expired carry-over maps Expired -> Timeout
    // with zero replacement sends.
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
    assert!(bus.with(|s| s.tx_accepted.is_empty()));
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.dropped_stale, m.pending), (0, 3, 0));
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
        { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
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
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
    for _ in 0..3 {
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }

    // Start a gripper Open (release) while holding an object; inject ENOBUFS on the final tick.
    let epoch = inner.safety.epoch_id();
    inner.releasing = true;
    let stamp = inner.current_admission_stamp();
    let admitted = inner.check_admission(stamp).expect("admit gripper");
    armd::SupervisorInner::start_gripper(admitted,
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
    // Ticks 1..=3 defer without faulting and MUST NOT mark the release done or clear releasing;
    // Tick 4 hits the 4-tick stall threshold (50 Hz default = ceil(45/20) + 1) and latches CAN_TX_STALLED.
    bus.with(|s| {
        s.push_all_feedback();
        s.default_rule = TxRule::Enobufs;
    });
    for tick in 1..=3 {
        if tick > 1 {
            bus.with(|s| s.push_all_feedback());
        }
        inner.tick(0.02);
        assert!(
            !inner.safety.state().is_stopped(),
            "transient ENOBUFS deferral on tick {tick} (< 4 at 50 Hz) must carry over without faulting: {:?}",
            inner.safety.state()
        );
        assert_eq!(inner.last_action.as_ref().unwrap().state, "running");
        assert!(inner.releasing, "deferred final release must not clear releasing flag");
    }

    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    assert!(
        matches!(inner.safety.state(), SafetyState::Fault(armd::safety::FaultReason::CanTxStalled)),
        "sustained deferral past stall threshold must latch CAN_TX_STALLED: {:?}",
        inner.safety.state()
    );
    assert_eq!(inner.safety.fault().map(|f| f.code), Some(FaultCode::CanTxStalled));
    let action = inner.last_action.as_ref().unwrap();
    assert_ne!(action.state, "done", "backpressured final release must not report done: {action:?}");
    assert_eq!(action.code.as_deref(), Some("CAN_TX_STALLED"));
    assert!(inner.releasing, "backpressured final release must not clear releasing flag");
}

#[test]
fn wrapper_default_bounded_write_fails_closed_and_flush_estop_drops_pending() {
    // Part A1: ZeroOffsetDriver overrides write_joint_targets_bounded and preserves carryover cleanly.
    let (mut d, bus, _clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 3, max_duration: ms(5), frame_max_age: ms(45) });
    let mut wrapped = armd::joint_zero::ZeroOffsetDriver::new(Box::new(d), &[0.0; 7]);
    reset_tx_metrics();
    assert_eq!(
        wrapped.write_joint_targets_bounded(&targets_const(0.2)).unwrap(),
        TxBatchOutcome::Deferred { pending: 4 }
    );
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.deferred, m.dropped_stopped, m.pending), (3, 4, 0, 4));
    assert_eq!(bus.with(|s| s.tx_accepted.len()), 3);

    // Part A2: A wrapper that does NOT override ArmDriver::write_joint_targets_bounded falls back
    // to write_joint_targets, which discards pending frames and propagates Err(TxDeferred).
    struct DefaultBoundedWrapper(Box<dyn ArmDriver>);
    impl ArmDriver for DefaultBoundedWrapper {
        fn capabilities(&self) -> armd::driver::DriverCapabilities { self.0.capabilities() }
        fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, ArmDriverError> { self.0.read_joint_states() }
        fn write_joint_targets(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> { self.0.write_joint_targets(t) }
        fn set_torque(&mut self, e: bool) -> Result<(), ArmDriverError> { self.0.set_torque(e) }
        fn enable_state(&self) -> armd::driver::EnableState { self.0.enable_state() }
        fn clear_fault(&mut self) -> Result<(), ArmDriverError> { self.0.clear_fault() }
    }
    let (mut d, bus, _clock) = enabled_driver();
    d.set_tx_budget(TxBudget { max_frames: 3, max_duration: ms(5), frame_max_age: ms(45) });
    let mut default_wrapped = DefaultBoundedWrapper(Box::new(d));
    reset_tx_metrics();
    let err = default_wrapped.write_joint_targets_bounded(&targets_const(0.2)).unwrap_err();
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
            d.write_joint_targets_bounded(&targets_const(0.02)).unwrap(),
            TxBatchOutcome::Deferred { pending: 3 }
        );
        for step in 0..3 {
            if step > 0 {
                clock.advance(ms(10));
            }
            bus.with(|s| s.push_all_feedback());
            d.read_joint_states().unwrap();
        }
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
    // and write_joint_targets_bounded reports Expired { dropped: 6, pending: 0 } uniformly
    // (never Deferred), while legacy write_joint_targets maps Expired to Timeout.
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
    assert_eq!(
        d.write_joint_targets_bounded(&targets_const(0.3)).unwrap(),
        TxBatchOutcome::Expired { dropped: 6, pending: 0 }
    );
    assert_eq!(d.pending_tx_len(), 0, "expired frames must not remain queued after flush");
    let m = take_tx_metrics();
    assert_eq!((m.frames, m.would_block, m.dropped_stale, m.pending), (1, 1, 6, 0));

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
    let err = d.write_joint_targets(&targets_const(0.3)).unwrap_err();
    assert!(matches!(err, ArmDriverError::Transport(TransportError::Timeout)), "{err:?}");
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
    for _ in 0..50 {
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
    for step in 0..120 {
        bus.clear();
        clock.advance(ms(1));
        match d.poll_torque(clock.now() + ms(5)).expect("poll_torque under max_frames=1") {
            armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed) => {
                let sent = bus.with(|s| s.tx_log.clone());
                assert!(
                    sent.len() <= 1,
                    "step {step} exceeded max_frames=1"
                );
                total_enables += sent.iter().filter(|f| f.data == damiao::CMD_ENABLE).count();
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

    // Advance another 60 ms (feedback age = 110 ms > 100 ms freshness limit) with silent motors
    // (drop the synchronous mock reply to the t = 50 ms recovery frame so the motor truly stayed silent):
    // recovery MUST fail closed and send ZERO MIT frames for J4.
    bus.with(|s| s.rx.clear());
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
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        for (i, slot) in arm_joints.iter_mut().enumerate().take(6) {
            *slot = i;
        }
        driver.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(false)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 6,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        driver.read_joint_states().unwrap();
        for _ in 0..2 {
            clock.advance(ms(10));
            bus.with(|s| s.push_all_feedback());
            driver.read_joint_states().unwrap();
        }
        driver.set_torque(false).unwrap();
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
    let (mut inner, bus, clock) = make_faulted_estop_supervisor();
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

    // Next supervisor ticks ingest K=3 fault-free feedback replies across >= 20 ms while E-stop remains latched.
    for step in 0..3 {
        if step > 0 {
            clock.advance(ms(10));
            bus.with(|s| s.push_all_feedback());
        }
        inner.tick(0.02);
    }
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
    let (mut inner_rt, bus_rt, clock_rt) = make_faulted_estop_supervisor();
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
    // Step 2 ticks ingest K=3 fresh fault-free feedback from all 7 motors across >= 20 ms.
    for step in 0..3 {
        if step > 0 {
            clock_rt.advance(ms(10));
            bus_rt.with(|s| s.push_all_feedback());
        }
        inner_rt.tick(0.02);
    }
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
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> { self.inner.receive_timestamped() }
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
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for (i, slot) in arm_joints.iter_mut().enumerate().take(6) {
        *slot = i;
    }
    d.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
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
    for step in 0..3 {
        if step > 0 {
            clock.advance(ms(10));
        }
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
    }
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
    for step in 0..120 {
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
    // Helper wrapper supporting:
    // - dropping J7 0x307 / CMD_ENABLE / 0xCC replies (silent J7),
    // - delayed delivery of J7 0xCC refresh replies (e.g. 30-60 ms) to test causal verification
    //   across 20 ms releases.
    struct J7ScriptedBus {
        inner: VirtualBus,
        clock: MockClock,
        drop_j7_0x307: Arc<AtomicBool>,
        drop_j7_enable: Arc<AtomicBool>,
        drop_j7_refresh: Arc<AtomicBool>,
        j7_refresh_delay: Arc<Mutex<Duration>>,
        delayed_rx: Arc<Mutex<VecDeque<(Instant, CanFrame)>>>,
    }
    impl J7ScriptedBus {
        fn new(limits: &[DamiaoLimits]) -> (Self, VirtualBus, MockClock) {
            let (inner, clock) = VirtualBus::new(limits);
            let bus = Self {
                inner: inner.clone(),
                clock: clock.clone(),
                drop_j7_0x307: Arc::new(AtomicBool::new(false)),
                drop_j7_enable: Arc::new(AtomicBool::new(false)),
                drop_j7_refresh: Arc::new(AtomicBool::new(false)),
                j7_refresh_delay: Arc::new(Mutex::new(Duration::ZERO)),
                delayed_rx: Arc::new(Mutex::new(VecDeque::new())),
            };
            (bus, inner, clock)
        }
        fn release_due_delayed(&self) {
            let now = self.clock.now();
            let mut delayed = self.delayed_rx.lock().unwrap();
            let mut remaining = VecDeque::new();
            while let Some((due_at, frame)) = delayed.pop_front() {
                if due_at <= now {
                    self.inner.with(|s| s.rx.push_back((frame, Some(due_at))));
                } else {
                    remaining.push_back((due_at, frame));
                }
            }
            *delayed = remaining;
        }
    }
    impl CanTransport for J7ScriptedBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn supports_nonblocking_rt(&self) -> bool { true }
        fn supports_bounded_flush(&self) -> bool { true }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let rx_before = self.inner.with(|s| s.rx.len());
            let r = self.inner.transmit(frame);
            let is_j7_refresh = frame.id == 0x7FF
                && frame.data[2] == 0xCC
                && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 7;
            let is_j7_enable = frame.id == 7 && frame.data == damiao::CMD_ENABLE;
            let is_j7_307 = frame.id == 0x307;
            if (is_j7_307 && self.drop_j7_0x307.load(Ordering::SeqCst))
                || (is_j7_enable && self.drop_j7_enable.load(Ordering::SeqCst))
                || (is_j7_refresh && self.drop_j7_refresh.load(Ordering::SeqCst))
            {
                self.inner.with(|s| s.rx.truncate(rx_before));
            } else if is_j7_refresh {
                let delay = *self.j7_refresh_delay.lock().unwrap();
                if !delay.is_zero() {
                    let due_at = self.clock.now() + delay;
                    let popped: Vec<CanFrame> = self.inner.with(|s| {
                        let mut out = Vec::new();
                        while s.rx.len() > rx_before {
                            if let Some((f, _)) = s.rx.pop_back() {
                                out.push(f);
                            }
                        }
                        out.reverse();
                        out
                    });
                    let mut delayed = self.delayed_rx.lock().unwrap();
                    for f in popped {
                        delayed.push_back((due_at, f));
                    }
                }
            }
            r
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.release_due_delayed();
            self.inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.release_due_delayed();
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    }

    let make_pf = || GripperPosForce {
        joint: 6,
        current_raw: 460,
        close_speed_rad_s: 0.30,
        closed_rad: 0.0,
        open_rad: -5.7,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
    };
    let limits = b601_limits();

    // Part A: Silent J7 on carried initial CMD_ENABLE (under tight TX budget) whose J7 feedback
    // ages > 100 ms MUST attempt bounded 0xCC refresh, NEVER transmit stale J7 CMD_ENABLE, and
    // fail closed with PosForceFeedbackStaleBeforeTransmit.
    {
        let (bus_wrap, vbus, clock) = J7ScriptedBus::new(&limits);
        vbus.with(|s| { s.motors[6].2 = -1.0; });
        let drop_307 = bus_wrap.drop_j7_0x307.clone();
        let drop_cc = bus_wrap.drop_j7_refresh.clone();
        let mut d = DamiaoArmDriver::new(bus_wrap, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_pos_force(make_pf())
            .unwrap();
        d.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();

        d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
        drop_307.store(true, Ordering::SeqCst);
        let _ = d.begin_torque(true).expect("begin");
        let mut saw_pre_enable_hold = false;
        for _ in 0..10 {
            vbus.clear();
            clock.advance(ms(1));
            let p = d.poll_torque(clock.now() + ms(5)).expect("poll before hold");
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                saw_pre_enable_hold = true;
                break;
            }
        }
        assert!(saw_pre_enable_hold, "expected 0x307 pre-enable hold before carried enable");

        // Silence J7 0xCC refreshes and advance across 20 ms releases past 100 ms J7 feedback age.
        drop_cc.store(true, Ordering::SeqCst);
        vbus.with(|s| s.rx.clear());
        clock.advance(ms(105));
        vbus.clear();
        d.set_tx_budget(TxBudget::default());

        let mut saw_j7_cc = false;
        let mut final_err = None;
        for step in 0..8 {
            if step > 0 {
                clock.advance(ms(20));
            }
            match d.poll_torque(clock.now() + ms(5)) {
                Ok(armd::driver::TorqueProgress::Pending { .. }) => {
                    saw_j7_cc |= vbus.with(|s| {
                        s.tx_log.iter().any(|f| {
                            f.id == 0x7FF
                                && f.data[2] == 0xCC
                                && u16::from_le_bytes([f.data[0], f.data[1]]) == 7
                        })
                    });
                    assert!(
                        !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
                        "stale carried J7 enable frame MUST NOT be transmitted while J7 is silent"
                    );
                }
                Err(e) => {
                    final_err = Some(e);
                    break;
                }
                Ok(other) => panic!("silent J7 must not confirm enable: {other:?}"),
            }
        }
        assert!(saw_j7_cc, "expected bounded J7 0xCC refresh attempt before failing closed");
        let err = final_err.expect("silent J7 carried enable must fail closed");
        assert!(
            matches!(err, ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: 6 }),
            "silent carried stale J7 enable must fail closed with PosForceFeedbackStaleBeforeTransmit: {err:?}"
        );
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
            "stale carried J7 enable frame MUST NOT be transmitted: {:?}",
            vbus.with(|s| s.tx_log.clone())
        );
        assert!(!d.torque_transition_pending() && !d.is_enabled());
    }

    // Part B: Silent J7 on retried CMD_ENABLE (at 75 ms retry after initial J7 enable reply was
    // lost) when J7 feedback aged > 100 ms MUST try 0xCC refresh, NEVER retransmit stale J7
    // CMD_ENABLE, and fail closed with PosForceFeedbackStaleBeforeTransmit.
    {
        let (bus_wrap, vbus, clock2) = J7ScriptedBus::new(&limits);
        vbus.with(|s| { s.motors[6].2 = -1.0; });
        let drop_307 = bus_wrap.drop_j7_0x307.clone();
        let drop_en = bus_wrap.drop_j7_enable.clone();
        let drop_cc = bus_wrap.drop_j7_refresh.clone();
        let mut d2 = DamiaoArmDriver::new(bus_wrap, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_pos_force(make_pf())
            .unwrap();
        d2.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d2.read_joint_states().unwrap();
        clock2.advance(ms(40));
        drop_307.store(true, Ordering::SeqCst);
        drop_en.store(true, Ordering::SeqCst);
        drop_cc.store(true, Ordering::SeqCst);
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
        vbus.with(|s| s.rx.retain(|(f, _)| f.id != 0x17));
        clock2.advance(ms(76));
        vbus.clear();

        let mut saw_j7_cc = false;
        let mut final_err = None;
        for step in 0..8 {
            if step > 0 {
                clock2.advance(ms(20));
            }
            match d2.poll_torque(clock2.now() + ms(5)) {
                Ok(armd::driver::TorqueProgress::Pending { .. }) => {
                    saw_j7_cc |= vbus.with(|s| {
                        s.tx_log.iter().any(|f| {
                            f.id == 0x7FF
                                && f.data[2] == 0xCC
                                && u16::from_le_bytes([f.data[0], f.data[1]]) == 7
                        })
                    });
                    assert!(
                        !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
                        "stale retried J7 enable MUST NOT be transmitted while silent"
                    );
                }
                Err(e) => {
                    final_err = Some(e);
                    break;
                }
                Ok(other) => panic!("silent J7 retry must not confirm: {other:?}"),
            }
        }
        assert!(saw_j7_cc, "expected bounded J7 0xCC refresh attempt on stale retry");
        let err2 = final_err.expect("silent retried J7 enable must fail closed");
        assert!(
            matches!(err2, ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint: 6 }),
            "retried J7 enable after feedback aged > 100 ms must fail closed when silent: {err2:?}"
        );
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
            "stale retried J7 enable frame MUST NOT be transmitted: {:?}",
            vbus.with(|s| s.tx_log.clone())
        );
        assert!(!d2.torque_transition_pending() && !d2.is_enabled());
    }

    // Part C: Carried initial J7 enable across 20 ms releases with constrained TX (max_frames=2)
    // and delayed J7 0xCC refresh replies (30 ms and 60 ms). Poll transmission alone MUST NOT
    // transmit J7 CMD_ENABLE; once the verified post-boundary reply arrives, J7 CMD_ENABLE
    // revalidates and transmits in that SAME release, and enable succeeds!
    for reply_delay_ms in [30u64, 60u64] {
        let (bus_wrap, vbus, clock3) = J7ScriptedBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            // Pre-configure J7 in PosForce so enable goes straight to readback -> 0x307 -> enable.
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let drop_307 = bus_wrap.drop_j7_0x307.clone();
        let delay_slot = bus_wrap.j7_refresh_delay.clone();
        let mut d3 = DamiaoArmDriver::new(bus_wrap, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_pos_force(make_pf())
            .unwrap();
        d3.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d3.read_joint_states().unwrap();

        // Age J7 to 85 ms before begin_torque so pre-enable 0x307 is still fresh, drop 0x307 reply,
        // and carry all 7 enable frames under max_frames=1 so J7 crosses 100 ms on the next 20 ms release.
        clock3.advance(ms(85));
        drop_307.store(true, Ordering::SeqCst);
        *delay_slot.lock().unwrap() = ms(reply_delay_ms);
        d3.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
        let _ = d3.begin_torque(true).expect("begin");
        let mut saw_pre_enable_hold = false;
        for _ in 0..10 {
            vbus.clear();
            clock3.advance(ms(1));
            let p = d3.poll_torque(clock3.now() + ms(5)).expect("poll before hold");
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                saw_pre_enable_hold = true;
                break;
            }
        }
        assert!(saw_pre_enable_hold, "delay={reply_delay_ms}ms: expected 0x307 pre-enable hold");

        // Switch to constrained TX (max_frames = 2 per 20 ms release).
        d3.set_tx_budget(TxBudget { max_frames: 2, max_duration: ms(5), frame_max_age: ms(45) });
        let mut cc_sent_at: Option<Instant> = None;
        let mut j7_enable_sent_at: Option<Instant> = None;
        let mut confirmed = false;
        for _step in 0..12 {
            vbus.clear();
            clock3.advance(ms(20));
            let now = clock3.now();
            match d3.poll_torque(now + ms(5)).expect("poll during delayed refresh must not fail") {
                armd::driver::TorqueProgress::Pending { .. } => {
                    let sent_cc = vbus.with(|s| {
                        s.tx_accepted.iter().any(|f| {
                            f.id == 0x7FF
                                && f.data[2] == 0xCC
                                && u16::from_le_bytes([f.data[0], f.data[1]]) == 7
                        })
                    });
                    let sent_j7_en = vbus.with(|s| {
                        s.tx_accepted.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)
                    });
                    if sent_cc && cc_sent_at.is_none() {
                        cc_sent_at = Some(now);
                        assert!(
                            !sent_j7_en,
                            "delay={reply_delay_ms}ms: 0xCC transmit alone must NEVER allow J7 CMD_ENABLE in same release"
                        );
                    }
                    if sent_j7_en && j7_enable_sent_at.is_none() {
                        let cc_t = cc_sent_at.expect("J7 CMD_ENABLE must follow 0xCC refresh");
                        assert!(
                            now.saturating_duration_since(cc_t) >= ms(reply_delay_ms),
                            "delay={reply_delay_ms}ms: J7 CMD_ENABLE sent before delayed 0xCC reply arrived"
                        );
                        j7_enable_sent_at = Some(now);
                    }
                }
                armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed) => {
                    confirmed = true;
                    break;
                }
                other => panic!("unexpected torque progress: {other:?}"),
            }
        }
        assert!(cc_sent_at.is_some(), "delay={reply_delay_ms}ms: expected J7 0xCC refresh");
        let cc_t = cc_sent_at.unwrap();
        let en_t = j7_enable_sent_at.expect("expected J7 CMD_ENABLE after fresh reply");
        // Verify J7 CMD_ENABLE went out on the EXACT release that accepted the delayed reply
        // (first 20 ms tick >= cc_t + reply_delay_ms).
        let expected_accept_offset = reply_delay_ms.div_ceil(20) * 20;
        assert_eq!(
            en_t.saturating_duration_since(cc_t),
            ms(expected_accept_offset),
            "delay={reply_delay_ms}ms: J7 CMD_ENABLE must transmit in the exact release accepting the fresh reply"
        );
        assert!(confirmed && d3.is_enabled(), "delay={reply_delay_ms}ms: enable must succeed after verified refresh");
    }

    // Part D: Retried J7 enable across 20 ms releases with delayed 40 ms J7 0xCC refresh reply -> MUST succeed.
    {
        let (bus_wrap, vbus, clock4) = J7ScriptedBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let drop_307 = bus_wrap.drop_j7_0x307.clone();
        let drop_en = bus_wrap.drop_j7_enable.clone();
        let delay_slot = bus_wrap.j7_refresh_delay.clone();
        let mut d4 = DamiaoArmDriver::new(bus_wrap, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_pos_force(make_pf())
            .unwrap();
        d4.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d4.read_joint_states().unwrap();

        // Age J7 to 65 ms so initial CMD_ENABLE sends fresh, its reply is dropped, and by the 40 ms
        // tick (after 25 ms retry cursor) J7 age is 105 ms > 100 ms -> triggers 0xCC refresh with 40 ms reply delay.
        clock4.advance(ms(65));
        drop_307.store(true, Ordering::SeqCst);
        drop_en.store(true, Ordering::SeqCst);
        *delay_slot.lock().unwrap() = ms(40);
        d4.set_tx_budget(TxBudget { max_frames: 7, max_duration: ms(5), frame_max_age: ms(45) });
        let _ = d4.begin_torque(true).expect("begin");
        let mut saw_initial_j7 = false;
        for _ in 0..10 {
            clock4.advance(ms(1));
            let p = d4.poll_torque(clock4.now() + ms(5)).expect("poll to initial enable");
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)) {
                saw_initial_j7 = true;
                break;
            }
        }
        assert!(saw_initial_j7, "expected initial J7 CMD_ENABLE");
        // Allow the retried J7 CMD_ENABLE to get its confirmation reply once refreshed.
        drop_en.store(false, Ordering::SeqCst);
        d4.set_tx_budget(TxBudget { max_frames: 2, max_duration: ms(5), frame_max_age: ms(45) });

        let mut saw_retry_cc = false;
        let mut confirmed = false;
        for _ in 0..10 {
            vbus.clear();
            clock4.advance(ms(20));
            match d4.poll_torque(clock4.now() + ms(5)).expect("poll during retried J7 refresh") {
                armd::driver::TorqueProgress::Pending { .. } => {
                    saw_retry_cc |= vbus.with(|s| {
                        s.tx_accepted.iter().any(|f| {
                            f.id == 0x7FF
                                && f.data[2] == 0xCC
                                && u16::from_le_bytes([f.data[0], f.data[1]]) == 7
                        })
                    });
                }
                armd::driver::TorqueProgress::Done(armd::driver::TorqueOutcome::Confirmed) => {
                    confirmed = true;
                    break;
                }
                other => panic!("unexpected outcome: {other:?}"),
            }
        }
        assert!(saw_retry_cc, "expected bounded J7 0xCC refresh during retried enable");
        assert!(confirmed && d4.is_enabled(), "retried J7 enable must succeed after verified 40 ms refresh");
    }

    // Part E: Fair J7 TX scheduling — a carried J7 enable MUST NOT starve behind earlier joints
    // across more than one release when TX budget allows.
    {
        let (vbus, clock5) = VirtualBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let mut d5 = DamiaoArmDriver::new(vbus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_gripper_pos_force(make_pf())
            .unwrap();
        d5.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d5.read_joint_states().unwrap();
        d5.set_tx_budget(TxBudget { max_frames: 2, max_duration: ms(5), frame_max_age: ms(45) });
        let _ = d5.begin_torque(true).expect("begin");
        let mut saw_pre_enable_hold = false;
        for _ in 0..10 {
            vbus.clear();
            clock5.advance(ms(1));
            let p = d5.poll_torque(clock5.now() + ms(5)).expect("poll before hold");
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                saw_pre_enable_hold = true;
                break;
            }
        }
        assert!(saw_pre_enable_hold);
        // On the VERY NEXT release (carrying J2..J7), J7 MUST transmit first (never starved behind J2..J6).
        vbus.clear();
        clock5.advance(ms(20));
        let _ = d5.poll_torque(clock5.now() + ms(5)).expect("next release after carried J7");
        let sent_ids: Vec<u32> = vbus.with(|s| {
            s.tx_accepted
                .iter()
                .filter(|f| f.data == damiao::CMD_ENABLE)
                .map(|f| f.id)
                .collect()
        });
        assert_eq!(
            sent_ids.first().copied(),
            Some(7),
            "carried J7 enable must be prioritized on the next release instead of starving behind earlier joints: {sent_ids:?}"
        );
    }
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
                    7 * usize::from(armd::driver::DAMIAO_DISABLE_REPEATS),
                    "expected 21 CMD_DISABLE frames: {tx:?}"
                );
                assert!(
                    tx.iter().all(|f| {
                        f.data == damiao::CMD_CLEAR_FAULTS
                            || f.data == damiao::CMD_DISABLE
                            || (f.id == 0x7FF && f.data[2] == 0xCC)
                    }),
                    "mit_expiry_recover must only transmit CMD_CLEAR_FAULTS, CMD_DISABLE, and read-only 0xCC status probes: {tx:?}"
                );
            },
        ),
        (
            "begin_torque(false)",
            Outcome::AllowedTx,
            |latch, _| {
                let (mut d, bus, clock) = enabled_driver();
                for i in 0..3 {
                    if i > 0 {
                        clock.advance(ms(11));
                    }
                    bus.with(|s| s.push_all_feedback());
                    d.read_joint_states().unwrap();
                }
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
                let (mut d, bus, clock) = enabled_driver();
                for i in 0..3 {
                    if i > 0 {
                        clock.advance(ms(11));
                    }
                    bus.with(|s| s.push_all_feedback());
                    d.read_joint_states().unwrap();
                }
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

/// Major 3 (part 1): Tick cutoff elapsing between Step 2 and Step 5 (during `build`) must refuse
/// frame submission (`Deferred`) with 0 CAN frames transmitted.
#[test]
fn cutoff_elapsing_during_send_build_refuses_submission_without_transmitting() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    struct JumpingClock {
        inner: MockClock,
        armed: Arc<AtomicBool>,
        calls: Arc<AtomicUsize>,
        jump_on_call: usize,
        jump_by: Duration,
    }
    impl DriverClock for JumpingClock {
        fn clock_id(&self) -> u64 { self.inner.clock_id() }
        fn now(&self) -> std::time::Instant {
            if self.armed.load(Ordering::SeqCst) {
                let n = self.calls.fetch_add(1, Ordering::SeqCst) + 1;
                if n == self.jump_on_call {
                    self.inner.advance(self.jump_by);
                }
            }
            self.inner.now()
        }
        fn sleep(&self, d: Duration) { self.inner.sleep(d); }
        fn yield_now(&self) { self.inner.yield_now(); }
    }

    struct CustomClockBus {
        inner: VirtualBus,
        clock: Arc<dyn DriverClock>,
    }
    impl CanTransport for CustomClockBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { Some(Arc::clone(&self.clock)) }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> { self.inner.transmit(frame) }
        fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> { self.inner.receive_timestamped() }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
        fn supports_bounded_flush(&self) -> bool { true }
        fn flush_is_noop(&self) -> bool { true }
        fn supports_nonblocking_rt(&self) -> bool { true }
    }

    let limits = b601_limits();
    let (vbus, base_clock) = VirtualBus::new(&limits);
    let armed = Arc::new(AtomicBool::new(false));
    let calls = Arc::new(AtomicUsize::new(0));
    // During `write_joint_targets_bounded` on an empty queue:
    //   1: drop_stale -> now
    //   2: batch_ts -> now
    //   3: ensure_tx_pass -> now (deadline = t0 + 10 ms)
    //   4: send Step 1 deadline check -> now
    //   5: send Step 2 can_start_tick_tx -> now (still < deadline!)
    //   6: send Step 5 deadline check -> now (still < deadline, or jumps to +10 ms right here!)
    //   7: send Step 5 can_start_tick_tx -> now (>= deadline -> Deferred, 0 frames sent!)
    // Test every jump call index in 4..=7: calls 4..=5 trip Step 2, calls 6..=7 trip Step 5
    // (after `build`, immediately before `can.transmit`), and NONE may transmit any frame.
    let mut hit_post_step2_deferral = false;
    for jump_on_call in 4..=7 {
        armed.store(false, Ordering::SeqCst);
        calls.store(0, Ordering::SeqCst);
        let jclock: Arc<dyn DriverClock> = Arc::new(JumpingClock {
            inner: base_clock.clone(),
            armed: Arc::clone(&armed),
            calls: Arc::clone(&calls),
            jump_on_call,
            jump_by: ms(10),
        });
        let bus = CustomClockBus { inner: vbus.clone(), clock: jclock };
        let mut d = DamiaoArmDriver::new(bus, vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_tx_budget(TxBudget {
                max_frames: 16,
                max_duration: ms(10),
                frame_max_age: ms(45),
            });
        vbus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.set_torque(true).unwrap();
        vbus.clear();
        d.begin_tx_tick();
        armed.store(true, Ordering::SeqCst);
        let out = d.write_joint_targets_bounded(&targets_const(0.1)).unwrap();
        armed.store(false, Ordering::SeqCst);
        assert_eq!(
            out,
            TxBatchOutcome::Deferred { pending: 7 },
            "jump_on_call={jump_on_call}: cutoff elapsed before transmit must defer all 7 frames"
        );
        assert!(
            vbus.with(|s| s.tx_log.is_empty()),
            "jump_on_call={jump_on_call}: zero frames may be transmitted after cutoff elapses"
        );
        if jump_on_call >= 6 {
            hit_post_step2_deferral = true;
        }
    }
    assert!(hit_post_step2_deferral);
}

/// Major 3 (part 2): When a transport has a real flush (`!flush_is_noop()`) and the tick TX
/// cutoff elapses during `transmit()`, all four bounded paths (unfenced motion, unfenced hold,
/// fenced motion, fenced hold) fail closed with `Timeout` without starting `flush()`.
#[test]
fn real_flush_cutoff_elapsed_fails_closed_across_all_four_paths() {
    // 1. Unfenced motion (`write_joint_targets_bounded`)
    {
        let (mut d, bus, _) = enabled_driver();
        d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(10), frame_max_age: ms(45) });
        bus.with(|s| {
            s.real_flush = true;
            s.flush_calls = 0;
            // Frame 0 starts at t=0 (< 10 ms) and completes at t=11 ms (>= 10 ms).
            s.default_step = ms(11);
        });
        d.begin_tx_tick();
        let err = d.write_joint_targets_bounded(&targets_const(0.1)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::Transport(TransportError::Timeout)),
            "unfenced motion real flush past cutoff must fail closed with Timeout, got {err:?}"
        );
        assert_eq!(bus.with(|s| s.flush_calls), 0, "unfenced motion must not start flush past cutoff");
        assert_eq!(d.pending_tx_len(), 0, "unfenced motion must discard pending queue on flush refusal");
    }

    // 2. Unfenced stopped hold (`write_estop_hold_bounded`)
    {
        let (mut d, bus, _) = enabled_driver();
        d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(10), frame_max_age: ms(45) });
        bus.with(|s| {
            s.real_flush = true;
            s.flush_calls = 0;
            s.default_step = ms(11);
        });
        d.begin_tx_tick();
        let err = d.write_estop_hold_bounded(&targets_const(0.0)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::Transport(TransportError::Timeout)),
            "unfenced hold real flush past cutoff must fail closed with Timeout, got {err:?}"
        );
        assert_eq!(bus.with(|s| s.flush_calls), 0, "unfenced hold must not start flush past cutoff");
        assert_eq!(d.pending_hold_tx_len(), 0, "unfenced hold must discard hold queue on flush refusal");
        assert!(!d.last_hold_tx_sent_any(), "unflushed hold must not report wire progress");
    }

    // 3. Fenced motion (`write_joint_targets_bounded` under armed MIT fence)
    {
        let (mut d, bus, _) = enabled_driver();
        d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(10), frame_max_age: ms(45) });
        d.set_mit_deadline_fence(Some(ms(45)), ms(250));
        bus.with(|s| {
            s.real_flush = true;
            s.flush_calls = 0;
            // 7 frames * 1.5 ms = 10.5 ms -> last frame starts at 9.0 ms (< 10 ms), ends at 10.5 ms (>= 10 ms).
            s.default_step = us(1_500);
        });
        d.begin_tx_tick();
        let err = d.write_joint_targets_bounded(&targets_const(0.1)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::Transport(TransportError::Timeout)),
            "fenced motion real flush past cutoff must fail closed with Timeout, got {err:?}"
        );
        assert_eq!(bus.with(|s| s.flush_calls), 0, "fenced motion must not start flush past cutoff");
        assert!(d.mit_fence_trip().is_some(), "fenced motion flush cutoff refusal must trip MIT fence");
    }

    // 4. Fenced stopped hold (`write_estop_hold_bounded` under armed MIT fence)
    {
        let (mut d, bus, _) = enabled_driver();
        d.set_tx_budget(TxBudget { max_frames: 16, max_duration: ms(10), frame_max_age: ms(45) });
        d.set_mit_deadline_fence(Some(ms(45)), ms(250));
        bus.with(|s| {
            s.real_flush = true;
            s.flush_calls = 0;
            s.default_step = us(1_500);
        });
        d.begin_tx_tick();
        let err = d.write_estop_hold_bounded(&targets_const(0.0)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::Transport(TransportError::Timeout)),
            "fenced hold real flush past cutoff must fail closed with Timeout, got {err:?}"
        );
        assert_eq!(bus.with(|s| s.flush_calls), 0, "fenced hold must not start flush past cutoff");
        assert!(d.mit_fence_trip().is_some(), "fenced hold flush cutoff refusal must trip MIT fence");
        assert!(!d.last_hold_tx_sent_any(), "unflushed fenced hold must not report wire progress");
    }
}

/// Major 4: An unconfirmed non-POS_FORCE joint stages BOTH POS_VEL (`0x100 + id`) and MIT (`id`).
/// Under slow cadence (90 ms > 45 ms frame max age) and `max_frames = 1`, `write_estop_hold_bounded`
/// must rotate across frame keys `(joint, slot)` so BOTH the POS_VEL and MIT hold frames are
/// transmitted instead of starving the MIT slot.
#[test]
fn slow_cadence_unconfirmed_joint_rotates_both_posvel_and_mit_hold_slots() {
    let limits = b601_limits();
    let (bus, clock) = VirtualBus::new(&limits);
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.set_torque(true).unwrap();

    // Force J1 into unconfirmed mode (`joint_modes[0] == None`):
    // 1. Switch J1 to MIT cleanly first (verifies MIT limits via REG_READ).
    // 2. Inject HardError and switch J1 back to Position -> forward write_modes & rollback both fail,
    //    leaving J1 unconfirmed (`joint_modes[0] == None`).
    let mut modes_mit = vec![armd::driver::JointMode::Position; 7];
    modes_mit[0] = armd::driver::JointMode::Mit;
    d.set_joint_modes(&modes_mit, &targets_const(0.0)).expect("switch J1 to MIT");
    bus.with(|s| {
        s.default_rule = TxRule::HardError;
    });
    let modes_pos = vec![armd::driver::JointMode::Position; 7];
    let _ = d.set_joint_modes(&modes_pos, &targets_const(0.0));
    assert!(d.joint_modes_uncertain(), "J1 must be unconfirmed after failed mode switch + rollback");

    // Restore healthy bus, set max_frames = 1 and slow 90 ms cadence (> 45 ms expiry, <= 100 ms fresh).
    bus.with(|s| {
        s.default_rule = TxRule::Ok;
        s.tx_log.clear();
        s.tx_accepted.clear();
    });
    d.set_tx_budget(TxBudget {
        max_frames: 1,
        max_duration: ms(10),
        frame_max_age: ms(45),
    });

    // 8 total frame keys (J1 POS_VEL 0x101, J1 MIT 0x001, J2..J7 POS_VEL 0x102..=0x107).
    // Across 8 slow-cadence releases (1 frame/release), every single key must be sent once.
    for _ in 0..8 {
        clock.advance(ms(90));
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        let out = d.write_estop_hold_bounded(&targets_const(0.0)).unwrap();
        assert!(
            matches!(out, TxBatchOutcome::Deferred { .. } | TxBatchOutcome::Expired { .. }),
            "expected partial progress under max_frames=1, got {out:?}"
        );
        assert!(d.last_hold_tx_sent_any(), "each slow release must send 1 hold frame");
    }

    let sent_ids: Vec<u32> = bus.with(|s| {
        s.tx_accepted
            .iter()
            .filter(|f| f.id != 0x7FF)
            .map(|f| f.id)
            .collect()
    });
    assert!(
        sent_ids.contains(&0x101),
        "unconfirmed J1 must receive POS_VEL hold (0x101): {sent_ids:#x?}"
    );
    assert!(
        sent_ids.contains(&0x001),
        "unconfirmed J1 must receive MIT hold (0x001) without starvation: {sent_ids:#x?}"
    );
    for j in 2u32..=7 {
        assert!(
            sent_ids.contains(&(0x100 + j)),
            "joint {j} must receive POS_VEL hold: {sent_ids:#x?}"
        );
    }
}

/// Remove every `#[cfg(test)]` item (module, function, or `mod foo;` declaration) from a
/// comment/literal-stripped Rust source string so static production-code proofs never trip on
/// unit tests while preserving all non-test production tokens.
fn strip_cfg_test_items(clean: &str) -> String {
    let bytes = clean.as_bytes();
    let len = bytes.len();
    let mut out = String::with_capacity(len);
    let mut i = 0usize;
    while i < len {
        if bytes[i] == b'#' {
            // Match `#[cfg(test)]` allowing arbitrary whitespace inside the attribute.
            let mut j = i + 1;
            while j < len && bytes[j].is_ascii_whitespace() {
                j += 1;
            }
            if j < len && bytes[j] == b'[' {
                let mut depth = 0usize;
                let attr_start = j;
                while j < len {
                    match bytes[j] {
                        b'[' => depth += 1,
                        b']' => {
                            depth -= 1;
                            j += 1;
                            if depth == 0 {
                                break;
                            }
                            continue;
                        }
                        _ => {}
                    }
                    j += 1;
                }
                let attr_compact: String = clean[attr_start..j]
                    .chars()
                    .filter(|c| !c.is_ascii_whitespace())
                    .collect();
                if attr_compact == "[cfg(test)]" {
                    // Skip any chained outer attributes (`#[path = "..."]`, `#[allow(...)]`, ...),
                    // then skip either the item's balanced `{ ... }` block or terminating `;`.
                    loop {
                        while j < len && bytes[j].is_ascii_whitespace() {
                            j += 1;
                        }
                        if j < len && bytes[j] == b'#' {
                            j += 1;
                            while j < len && bytes[j].is_ascii_whitespace() {
                                j += 1;
                            }
                            if j < len && bytes[j] == b'[' {
                                let mut ad = 0usize;
                                while j < len {
                                    match bytes[j] {
                                        b'[' => ad += 1,
                                        b']' => {
                                            ad -= 1;
                                            j += 1;
                                            if ad == 0 {
                                                break;
                                            }
                                            continue;
                                        }
                                        _ => {}
                                    }
                                    j += 1;
                                }
                                continue;
                            }
                        }
                        break;
                    }
                    while j < len {
                        if bytes[j] == b';' {
                            j += 1;
                            break;
                        }
                        if bytes[j] == b'{' {
                            let mut bd = 0usize;
                            while j < len {
                                match bytes[j] {
                                    b'{' => bd += 1,
                                    b'}' => {
                                        bd -= 1;
                                        j += 1;
                                        if bd == 0 {
                                            break;
                                        }
                                        continue;
                                    }
                                    _ => {}
                                }
                                j += 1;
                            }
                            break;
                        }
                        j += 1;
                    }
                    i = j;
                    continue;
                }
            }
        }
        let ch = clean[i..].chars().next().unwrap();
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

/// Partition `clean` (already comment/literal/test-stripped) around the unique function definition
/// matching `fn_prefix` (e.g. `"fn emit_pos_force_frame("`) into `(outside_compact, body_compact)`.
fn partition_unique_fn_compact(clean: &str, fn_prefix: &str) -> (String, String) {
    let matches: Vec<usize> = clean.match_indices(fn_prefix).map(|(idx, _)| idx).collect();
    assert_eq!(
        matches.len(),
        1,
        "expected exactly one `{fn_prefix}` definition, found {}",
        matches.len()
    );
    let fn_start = matches[0];
    let body_open = clean[fn_start..]
        .find('{')
        .map(|rel| fn_start + rel)
        .unwrap_or_else(|| panic!("missing opening brace for `{fn_prefix}`"));
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
    let body_end = body_end.unwrap_or_else(|| panic!("missing closing brace for `{fn_prefix}`"));
    let body = &clean[body_open..=body_end];
    let outside = format!("{}{}", &clean[..fn_start], &clean[body_end + 1..]);
    let compact = |s: &str| -> String { s.chars().filter(|c| !c.is_ascii_whitespace()).collect() };
    (compact(&outside), compact(body))
}

fn collect_rs_files(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()))
        .map(|e| e.expect("dir entry").path())
        .collect();
    entries.sort();
    for path in entries {
        if path.is_dir() {
            collect_rs_files(&path, out);
        } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
            out.push(path);
        }
    }
}

/// True iff `compact_lower` (whitespace-stripped, lowercased production source) contains a hex
/// literal token equal to `hex_token` (e.g. `"0x307"` or `"0x300"`) not extended by another hex
/// digit (so unrelated CJK codepoints like `0x3005`/`0x3007` are ignored while `0x300`, `0x300u32`,
/// `0x300+id`, and `0x307` are all caught).
fn has_hex_literal_token(compact_lower: &str, hex_token: &str) -> bool {
    let bytes = compact_lower.as_bytes();
    for (idx, _) in compact_lower.match_indices(hex_token) {
        let after = idx + hex_token.len();
        let next_is_hex = bytes.get(after).is_some_and(|b| b.is_ascii_hexdigit());
        if !next_is_hex {
            return true;
        }
    }
    false
}

/// Strip the balanced `{ ... }` bodies (and headers from `fn_prefix`) of every allow-listed
/// function from `clean` (comment/literal/`#[cfg(test)]`-stripped source), asserting each
/// allow-listed function exists once, and return `(outside_all_compact, per_fn_body_compact)`.
fn partition_allowlisted_fns_compact<'a>(
    clean: &str,
    allowlist: &[(&'a str, &str)],
) -> (String, Vec<(&'a str, String)>) {
    let mut ranges: Vec<(usize, usize, &'a str, String)> = Vec::with_capacity(allowlist.len());
    let compact = |s: &str| -> String { s.chars().filter(|c| !c.is_ascii_whitespace()).collect() };
    for &(fn_prefix, _why_safe) in allowlist {
        let matches: Vec<usize> = clean.match_indices(fn_prefix).map(|(idx, _)| idx).collect();
        assert_eq!(
            matches.len(),
            1,
            "expected exactly one allow-listed `{fn_prefix}` definition, found {}",
            matches.len()
        );
        let fn_start = matches[0];
        let body_open = clean[fn_start..]
            .find('{')
            .map(|rel| fn_start + rel)
            .unwrap_or_else(|| panic!("missing opening brace for allow-listed `{fn_prefix}`"));
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
        let body_end =
            body_end.unwrap_or_else(|| panic!("missing closing brace for allow-listed `{fn_prefix}`"));
        ranges.push((fn_start, body_end, fn_prefix, compact(&clean[body_open..=body_end])));
    }
    ranges.sort_by_key(|(start, _, _, _)| *start);
    let mut outside = String::with_capacity(clean.len());
    let mut cursor = 0usize;
    let mut bodies = Vec::with_capacity(ranges.len());
    for (start, end, name, body) in ranges {
        assert!(start >= cursor, "overlapping allow-listed function `{name}`");
        outside.push_str(&clean[cursor..start]);
        cursor = end + 1;
        bodies.push((name, body));
    }
    outside.push_str(&clean[cursor..]);
    (compact(&outside), bodies)
}

#[test]
fn pos_force_0x307_frame_construction_exists_only_in_single_emitter() {
    // Self-check: verify multi-line splits, raw 0x307 / 0x300 / CanFrame literals, and unlisted
    // `emit_pos_force_frame(` callers inside driver.rs are ALL caught, while comments / literals /
    // `#[cfg(test)]` items are ignored.
    let synthetic = r#"
        impl Demo {
            // damiao::build_pos_force_frame(0x300, 7, 0.0, 0, 0);
            fn emit_pos_force_frame(&mut self) {
                let _s = "build_pos_force_frame(0x307)";
                damiao::
                    build_pos_force_frame(0x300, 7, 0.0, 0, 0);
            }
            fn sneaky_build(&mut self) {
                build_pos_force_frame
                    (0x300, 7, 0.0, 0, 0);
            }
            fn ok_ret(&self) -> CanFrame {
                damiao::build_pos_vel_frame(0x100, 1, 0.0, 1.0)
            }
            fn sneaky_raw(&mut self) {
                let _raw = CanFrame { id: 0x307, dlc: 8 };
                let _ctor = CanFrame::new(0x307, &[]);
                let _base = 0x300 + 7;
                self.emit_pos_force_frame();
            }
        }
        #[cfg(test)]
        mod tests {
            fn t() { let _ = (0x307, CanFrame { id: 0x300 }, build_pos_force_frame(0x300, 7, 0.0, 0, 0)); }
        }
    "#;
    let syn_prod = strip_cfg_test_items(&strip_rust_comments_and_literals(synthetic));
    let (syn_out, syn_in) = partition_unique_fn_compact(&syn_prod, "fn emit_pos_force_frame(");
    assert_eq!(syn_in.matches("build_pos_force_frame(").count(), 1);
    assert_eq!(
        syn_out.matches("build_pos_force_frame(").count(),
        1,
        "must catch multi-line build_pos_force_frame( outside emitter: {syn_out}"
    );
    let syn_out_lower = syn_out.to_ascii_lowercase();
    assert!(has_hex_literal_token(&syn_out_lower, "0x307"), "must catch raw 0x307 in driver.rs");
    assert!(has_hex_literal_token(&syn_out_lower, "0x300"), "must catch raw 0x300 in driver.rs");
    assert!(!has_hex_literal_token("0x3005|0x3007", "0x300"), "must not trip on 0x3005/0x3007 CJK ranges");
    let count_raw_can_frame_ctors = |compact_src: &str| -> usize {
        let struct_lits = compact_src
            .match_indices("CanFrame{")
            .filter(|&(idx, _)| !compact_src[..idx].ends_with("->"))
            .count();
        struct_lits + compact_src.matches("CanFrame::new").count()
    };
    assert_eq!(count_raw_can_frame_ctors(&syn_out), 2, "must catch raw CanFrame literal + CanFrame::new (ignoring `-> CanFrame {{`)");
    assert_eq!(
        syn_out.matches("emit_pos_force_frame(").count(),
        1,
        "must catch unlisted emit_pos_force_frame( caller in driver.rs"
    );

    let src_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    collect_rs_files(&src_root, &mut files);
    assert!(!files.is_empty(), "expected Rust source files under src/");

    let mut total_driver_build_sites = 0usize;
    let mut total_owner_emit_j7_calls = 0usize;

    for path in &files {
        let rel = path
            .strip_prefix(&src_root)
            .expect("relative under src/")
            .to_string_lossy()
            .replace('\\', "/");
        let raw = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
        let prod = strip_cfg_test_items(&strip_rust_comments_and_literals(&raw));
        let compact: String = prod.chars().filter(|c| !c.is_ascii_whitespace()).collect();
        let compact_lower = compact.to_ascii_lowercase();

        // Raw CAN ID `0x307` is forbidden across ALL production source files, INCLUDING driver.rs
        // and protocol/damiao.rs.
        assert!(
            !has_hex_literal_token(&compact_lower, "0x307"),
            "production file src/{rel} must not hardcode CAN ID 0x307"
        );

        if rel == "protocol/damiao.rs" {
            // `build_pos_force_frame` MUST be `pub(crate)` and require `&crate::driver::PosForceSendToken`
            // so only private `driver::pos_force_emitter` can invoke it at compile time.
            assert_eq!(
                compact.matches("build_pos_force_frame(").count(),
                1,
                "src/protocol/damiao.rs must contain only the build_pos_force_frame definition"
            );
            assert!(
                compact.contains("pub(crate)fnbuild_pos_force_frame(_token:&crate::driver::PosForceSendToken,"),
                "build_pos_force_frame in src/protocol/damiao.rs must be pub(crate) and sealed by &PosForceSendToken"
            );
            assert!(
                !compact.contains("pubfnbuild_pos_force_frame("),
                "build_pos_force_frame must not be publicly callable without PosForceSendToken"
            );
            continue;
        }

        // Raw base ID `0x300` is forbidden across ALL production source files outside protocol/damiao.rs,
        // INCLUDING src/driver.rs!
        assert!(
            !has_hex_literal_token(&compact_lower, "0x300"),
            "production file src/{rel} must not hardcode PosForce base CAN ID 0x300"
        );

        if rel == "driver.rs" {
            // 1. No raw `CanFrame { ... }` struct literal or `CanFrame::new*` constructor in
            //    production driver.rs (prevents bypassing `damiao::build_pos_force_frame` with
            //    hand-built CAN frames).
            assert_eq!(
                count_raw_can_frame_ctors(&compact),
                0,
                "src/driver.rs must not hand-construct raw CanFrame literals/constructors in production code"
            );

            // 2a. `build_pos_force_frame(` exists ONLY inside private `pos_force_emitter::build_wire_frame`.
            let (outside_build, inside_build) =
                partition_unique_fn_compact(&prod, "fn build_wire_frame(");
            assert_eq!(
                inside_build.matches("build_pos_force_frame(").count(),
                1,
                "pos_force_emitter::build_wire_frame must contain EXACTLY ONE build_pos_force_frame( call"
            );
            assert_eq!(
                outside_build.matches("build_pos_force_frame(").count(),
                0,
                "src/driver.rs must have ZERO build_pos_force_frame( calls outside pos_force_emitter::build_wire_frame"
            );
            total_driver_build_sites += inside_build.matches("build_pos_force_frame(").count();

            // 2b. `PosForceSendToken(` is defined and constructed ONLY inside private `mod pos_force_emitter`
            //     (`struct PosForceSendToken(())` + `PosForceSendToken(())` inside `fn send_frame(`).
            let (outside_send_frame, inside_send_frame) =
                partition_unique_fn_compact(&prod, "fn send_frame<");
            assert_eq!(
                inside_send_frame.matches("PosForceSendToken(").count(),
                1,
                "pos_force_emitter::send_frame must construct PosForceSendToken(()) once"
            );
            assert_eq!(
                outside_send_frame.matches("PosForceSendToken(").count(),
                1,
                "outside send_frame, only `struct PosForceSendToken(())` may contain `PosForceSendToken(`"
            );
            assert!(
                outside_send_frame.contains("pub(crate)structPosForceSendToken(())"),
                "PosForceSendToken must be a private-field tuple struct inside private mod pos_force_emitter"
            );
            assert!(
                !compact.contains("pubmodpos_force_emitter"),
                "mod pos_force_emitter must remain private"
            );

            // 3. Explicit allow-list for EVERY `POS_FORCE_BASE_ID` reference in production driver.rs:
            //    all non-emitter sites are strictly read-only CAN ID predicates or key lookups.
            const POS_FORCE_BASE_ID_ALLOWLIST: &[(&str, &str)] = &[
                (
                    "fn build_wire_frame(",
                    "Sole production constructor inside private mod pos_force_emitter calling damiao::build_pos_force_frame",
                ),
                (
                    "fn frame_pos_force_joint(",
                    "Read-only CAN ID predicate used by DamiaoArmDriver::send submission guard",
                ),
                (
                    "fn is_pos_force_freeze_frame(",
                    "Read-only bit-for-bit freeze payload predicate (constructs no frame)",
                ),
                (
                    "fn validate_pos_force_target_static(",
                    "Pure static validation returning expected CAN ID (constructs no frame)",
                ),
                (
                    "fn process_rx_frame(",
                    "Read-only SocketCAN local-echo RX filter (drops echoed 0x300+id TX frames)",
                ),
                (
                    "fn receive_feedback(",
                    "Read-only last_target_frames CAN ID match for standalone recovery speed lookup",
                ),
            ];
            let (outside_base_id, _) =
                partition_allowlisted_fns_compact(&prod, POS_FORCE_BASE_ID_ALLOWLIST);
            assert_eq!(
                outside_base_id.matches("POS_FORCE_BASE_ID").count(),
                0,
                "src/driver.rs references POS_FORCE_BASE_ID outside its explicit allow-list"
            );

            // 4. Secondary text check on EVERY caller of `pos_force_emitter::send_frame` and the
            //    narrow `pos_force_emitter::emit_*` APIs in production driver.rs.
            const NARROW_EMITTER_CALLERS: &[(&str, &str, &str)] = &[
                (
                    "fn finish_pos_force_readback(",
                    "pos_force_emitter::emit_pre_enable_hold(",
                    "Pre-enable stationary hold during torque-on handshake right after CTRL_MODE readback while motor is still disabled (allow_freeze = false)",
                ),
                (
                    "fn write_modes_send(",
                    "pos_force_emitter::emit_mode_switch_hold(",
                    "Immediate hold frame directly following a CTRL_MODE register switch (allow_freeze = false)",
                ),
                (
                    "fn write_modes_confirmed(",
                    "pos_force_emitter::emit_mode_switch_hold(",
                    "Immediate hold frame directly following a confirmed CTRL_MODE register switch (allow_freeze = false)",
                ),
                (
                    "fn receive_feedback(",
                    "pos_force_emitter::emit_standalone_recovery_hold(",
                    "Standalone-driver recovery only (refuses with BuildRefused whenever external_pos_force_emitter == true; allow_freeze = false)",
                ),
            ];
            let narrow_allowlist: Vec<(&str, &str)> = NARROW_EMITTER_CALLERS
                .iter()
                .map(|&(fn_sig, _, why)| (fn_sig, why))
                .collect();
            let (outside_narrow, narrow_bodies) =
                partition_allowlisted_fns_compact(&prod, &narrow_allowlist);
            for &(_, api, _) in NARROW_EMITTER_CALLERS {
                assert_eq!(
                    outside_narrow.matches(api).count(),
                    0,
                    "src/driver.rs calls {api} outside its explicit allow-list"
                );
            }
            for (fn_name, body) in &narrow_bodies {
                let &(_, api, _) = NARROW_EMITTER_CALLERS
                    .iter()
                    .find(|&&(sig, _, _)| sig == *fn_name)
                    .unwrap_or_else(|| panic!("unexpected narrow allow-list entry {fn_name}"));
                assert_eq!(
                    body.matches(api).count(),
                    1,
                    "allow-listed {fn_name} expected exactly 1 {api} call"
                );
            }
            // Partition `DamiaoArmDriver` vs the rest of `driver.rs` (trait default methods and
            // other arm drivers) so `EMIT_TICK_ALLOWLIST` matches Damiao's unique trait impls and
            // proves ZERO `pos_force_emitter::` calls exist anywhere outside Damiao.
            let dm_impl_start = prod
                .find("impl<C: CanTransport> DamiaoArmDriver<C>")
                .expect("DamiaoArmDriver impl block");
            let rs_impl_start = prod
                .find("pub struct RobStrideArmDriver")
                .expect("RobStrideArmDriver struct");
            assert!(dm_impl_start < rs_impl_start);
            let outside_dm_compact: String = format!("{}{}", &prod[..dm_impl_start], &prod[rs_impl_start..])
                .chars()
                .filter(|c| !c.is_ascii_whitespace())
                .collect();
            assert_eq!(
                outside_dm_compact.matches("pos_force_emitter::").count(),
                0,
                "src/driver.rs must have ZERO pos_force_emitter:: calls outside DamiaoArmDriver"
            );
            let dm_prod = &prod[dm_impl_start..rs_impl_start];
            const EMIT_TICK_ALLOWLIST: &[(&str, &str)] = &[
                (
                    "fn emit_pos_force_j7(",
                    "ArmDriver trait entrypoint called once per tick from SupervisorInner tick epilogue (Legacy & RT Executor)",
                ),
                (
                    "fn write_estop_hold_bounded(",
                    "Direct-driver fallback when external_pos_force_emitter == false (returns early when true)",
                ),
                (
                    "fn write_joint_targets_bounded(",
                    "Direct-driver fallback when external_pos_force_emitter == false (returns early when true)",
                ),
                (
                    "fn drain_pending_tx(",
                    "Direct-driver fallback when external_pos_force_emitter == false (returns early when true)",
                ),
            ];
            let (outside_emit_tick, emit_tick_bodies) =
                partition_allowlisted_fns_compact(dm_prod, EMIT_TICK_ALLOWLIST);
            assert_eq!(
                outside_emit_tick.matches("pos_force_emitter::emit_tick(").count(),
                0,
                "src/driver.rs calls pos_force_emitter::emit_tick( outside its explicit allow-list"
            );
            for (fn_name, body) in emit_tick_bodies {
                assert!(
                    body.matches("pos_force_emitter::emit_tick(").count() >= 1,
                    "allow-listed {fn_name} expected at least 1 pos_force_emitter::emit_tick( call"
                );
            }
            continue;
        }

        // Every other production source file in src/: ZERO POS_FORCE_BASE_ID / build_pos_force_frame.
        assert_eq!(
            compact.matches("build_pos_force_frame(").count(),
            0,
            "production file src/{rel} must not call build_pos_force_frame("
        );
        assert_eq!(
            compact.matches("POS_FORCE_BASE_ID").count(),
            0,
            "production file src/{rel} must not reference POS_FORCE_BASE_ID"
        );

        // Outside driver.rs and joint_zero.rs (trait + zero-offset wrapper), `emit_pos_force_j7(`
        // may be invoked ONLY inside `SupervisorInner::emit_and_merge_pos_force_j7` in supervisor.rs,
        // which itself is called ONLY from `emit_j7_tick_epilogue`, which is called ONLY once in
        // `tick_control_inner` (shared by BOTH Legacy and RT Executor owners).
        if rel == "supervisor.rs" {
            let (outside_merge, inside_merge) =
                partition_unique_fn_compact(&prod, "fn emit_and_merge_pos_force_j7(");
            assert_eq!(
                inside_merge.matches("emit_pos_force_j7(").count(),
                1,
                "SupervisorInner::emit_and_merge_pos_force_j7 must call emit_pos_force_j7( once"
            );
            assert_eq!(
                outside_merge.matches("emit_pos_force_j7(").count(),
                0,
                "src/supervisor.rs must have ZERO emit_pos_force_j7( calls outside emit_and_merge_pos_force_j7"
            );
            total_owner_emit_j7_calls += inside_merge.matches("emit_pos_force_j7(").count();

            let (outside_epilogue, inside_epilogue) =
                partition_unique_fn_compact(&prod, "fn emit_j7_tick_epilogue(");
            assert_eq!(
                inside_epilogue.matches("emit_and_merge_pos_force_j7(").count(),
                1,
                "emit_j7_tick_epilogue must call emit_and_merge_pos_force_j7( once"
            );
            // Outside `emit_j7_tick_epilogue`, only the definition `fn emit_and_merge_pos_force_j7(` exists.
            assert_eq!(
                outside_epilogue.matches("emit_and_merge_pos_force_j7(").count(),
                1,
                "emit_and_merge_pos_force_j7 must have no callers outside emit_j7_tick_epilogue"
            );

            let (outside_tick, inside_tick) =
                partition_unique_fn_compact(&prod, "fn tick_control_inner(");
            assert_eq!(
                inside_tick.matches("emit_j7_tick_epilogue(").count(),
                1,
                "tick_control_inner must call emit_j7_tick_epilogue( exactly once"
            );
            // Outside `tick_control_inner`, only the definition `fn emit_j7_tick_epilogue(` exists.
            assert_eq!(
                outside_tick.matches("emit_j7_tick_epilogue(").count(),
                1,
                "emit_j7_tick_epilogue must have no callers outside tick_control_inner"
            );
        } else if rel != "joint_zero.rs" {
            assert_eq!(
                compact.matches("emit_pos_force_j7(").count(),
                0,
                "production file src/{rel} must not call emit_pos_force_j7("
            );
            assert_eq!(
                compact.matches("emit_and_merge_pos_force_j7(").count(),
                0,
                "production file src/{rel} must not call emit_and_merge_pos_force_j7("
            );
        }
    }

    assert_eq!(
        total_driver_build_sites, 1,
        "expected exactly 1 production build_pos_force_frame call site across src/"
    );
    assert_eq!(
        total_owner_emit_j7_calls, 1,
        "expected exactly 1 owner-side emit_pos_force_j7 call site across src/"
    );
}

/// Round 7 M2 regression: under tight per-tick budgets (`max_frames = 1, 2, 3`), J1–J6 stopped
/// hold frames ALWAYS take strict priority over J7 `0xCC` refresh and J7 `0x307` freeze/hold; J7
/// is served only after all due J1–J6 holds have been queued/sent and remaining capacity allows.
#[test]
fn j1_j6_stopped_holds_prioritized_over_j7_under_tight_budget_1_2_3() {
    let pf_cfg = armd::driver::GripperPosForce {
        joint: 6,
        current_raw: 460,
        close_speed_rad_s: 0.30,
        closed_rad: 0.0,
        open_rad: -5.7,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
    };

    // Part A: 7-motor arm (6 arm joints + PosForce J7), J7 stale (> 100 ms, refresh pending) while
    // J1–J6 stay fresh. Under max_frames = 1, 2, 3 across 12 consecutive stopped-hold ticks:
    // - EVERY tick transmits exact `max_frames` J1–J6 arm holds (0 starvation ticks!);
    // - Round-robin services all 6 arm joints J1–J6;
    // - Neither J7 0xCC refresh nor J7 0x307 freeze steals a slot while J1–J6 holds remain due.
    for max_frames in [1usize, 2, 3] {
        let (bus, clock) = VirtualBus::new(&b601_limits());
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(b601_limits())
            .unwrap()
            .with_gripper_pos_force(pf_cfg.clone())
            .unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).unwrap();
        bus.with(|s| {
            s.rx.clear();
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();

        // First issue one motion pass so all 7 joints (J1-J6 + J7) are executing motion (`vel > 0`).
        d.begin_tx_tick();
        assert_eq!(d.write_joint_targets_bounded(&targets_const(0.2)).unwrap(), TxBatchOutcome::Complete);
        clock.advance(ms(20));
        bus.with(|s| {
            s.rx.clear();
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();

        // Make J7 stale (> 100 ms) so both J7 freeze (Class 1 -> Class 3) AND 0xCC refresh (Class 4)
        // want slots, then tighten budget to `max_frames` ∈ {1, 2, 3}.
        clock.advance(ms(115));
        bus.with(|s| {
            s.rx.clear();
            for i in 0..6 {
                s.push_feedback(i);
            }
        });
        d.read_joint_states().unwrap();
        assert!(!d.pos_force_feedback_fresh(), "J7 must be stale");

        d.set_tx_budget(TxBudget {
            max_frames,
            max_duration: ms(10),
            frame_max_age: ms(45),
        });

        let bound_ticks = 7usize.div_ceil(max_frames);
        let mut seen_arm_ids = std::collections::BTreeSet::new();
        let mut j7_freeze_first_tick: Option<usize> = None;
        let mut total_j7_freezes = 0usize;
        for tick in 1..=14 {
            clock.advance(ms(20));
            bus.with(|s| {
                s.rx.clear();
                for i in 0..6 {
                    s.push_feedback(i);
                }
                s.tx_log.clear();
                s.tx_accepted.clear();
            });
            d.read_joint_states().unwrap();
            let out = d.write_estop_hold_bounded(&targets_const(0.0)).unwrap();
            assert!(
                matches!(out, TxBatchOutcome::Deferred { .. } | TxBatchOutcome::Expired { .. }),
                "max_frames={max_frames} tick={tick}: 7 joints cannot complete in {max_frames} frames: {out:?}"
            );
            let tx = bus.with(|s| s.tx_accepted.clone());
            let arm_holds: Vec<u32> = tx
                .iter()
                .filter(|f| (0x101..=0x106).contains(&f.id))
                .map(|f| f.id)
                .collect();
            let j7_frames: Vec<_> = tx.iter().filter(|f| f.id == 0x307).collect();
            let j7_refresh: Vec<_> = tx
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC)
                .collect();
            assert_eq!(
                arm_holds.len() + j7_frames.len(),
                max_frames,
                "max_frames={max_frames} tick={tick}: Class 1/3 holds+freeze must fill all {max_frames} slots: tx={tx:?}"
            );
            assert!(
                j7_refresh.is_empty(),
                "max_frames={max_frames} tick={tick}: Class-4 0xCC refresh must not steal slot from Class 1/3: tx={tx:?}"
            );
            for id in arm_holds {
                seen_arm_ids.insert(id);
            }
            if !j7_frames.is_empty() {
                assert_eq!(u16::from_le_bytes([j7_frames[0].data[4], j7_frames[0].data[5]]), 0);
                j7_freeze_first_tick.get_or_insert(tick);
                total_j7_freezes += 1;
            }
        }
        assert_eq!(
            seen_arm_ids,
            (0x101u32..=0x106).collect(),
            "max_frames={max_frames}: all 6 arm joints J1-J6 must receive holds via round-robin"
        );
        assert!(
            j7_freeze_first_tick.is_some_and(|t| t <= bound_ticks),
            "max_frames={max_frames}: J7 Class-1 freeze must transmit within ceil(7/{max_frames})={bound_ticks} ticks, got {j7_freeze_first_tick:?}"
        );
        assert!(
            total_j7_freezes >= 2,
            "max_frames={max_frames}: J7 must also rotate fairly in Class 3 across 14 ticks, got {total_j7_freezes}"
        );

        // Part B1: Once capacity allows on the same driver (max_frames = 7 -> 6 arm holds + 1 J7 freeze;
        // max_frames = 8 -> 6 arm holds + 1 J7 freeze + 1 J7 0xCC refresh), all 6 arm holds transmit
        // FIRST, then J7 freeze, then 0xCC refresh!
        for (budget, expect_refresh) in [(7usize, false), (8usize, true)] {
            d.set_tx_budget(TxBudget {
                max_frames: budget,
                max_duration: ms(10),
                frame_max_age: ms(45),
            });
            clock.advance(ms(20));
            bus.with(|s| {
                s.rx.clear();
                for i in 0..6 {
                    s.push_feedback(i);
                }
                s.tx_log.clear();
                s.tx_accepted.clear();
            });
            d.read_joint_states().unwrap();
            let out = d.write_estop_hold_bounded(&targets_const(0.0)).unwrap();
            assert!(
                matches!(out, TxBatchOutcome::Complete | TxBatchOutcome::Expired { pending: 0, .. }),
                "budget={budget}: all 6 arm holds + J7 freeze must drain with pending=0, got {out:?}"
            );
            let tx = bus.with(|s| s.tx_accepted.clone());
            let arm_indices: Vec<usize> = tx
                .iter()
                .enumerate()
                .filter(|(_, f)| (0x101..=0x106).contains(&f.id))
                .map(|(idx, _)| idx)
                .collect();
            let j7_idx = tx
                .iter()
                .position(|f| f.id == 0x307)
                .unwrap_or_else(|| panic!("budget={budget}: expected J7 freeze once capacity allows: {tx:?}"));
            assert_eq!(arm_indices.len(), 6, "budget={budget}: all 6 arm holds must send: {tx:?}");
            assert!(
                arm_indices.iter().all(|&i| i < j7_idx),
                "budget={budget}: all 6 arm holds must precede J7 freeze: {tx:?}"
            );
            assert_eq!(u16::from_le_bytes([tx[j7_idx].data[4], tx[j7_idx].data[5]]), 0);
            let refresh_idx = tx
                .iter()
                .position(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC);
            if expect_refresh {
                let r_idx = refresh_idx.unwrap_or_else(|| panic!("budget={budget}: expected 0xCC refresh after J7 freeze: {tx:?}"));
                assert!(r_idx > j7_idx, "budget={budget}: 0xCC refresh must follow J7 freeze: {tx:?}");
            } else {
                assert!(refresh_idx.is_none(), "budget={budget}: 0xCC must not steal slot from 6 arm holds + J7 freeze: {tx:?}");
            }
        }
    }

    // Part B2: With max_frames = 2 on a 1-arm + J7 bus (or max_frames = 3 on a 2-arm + J7 bus),
    // all due arm holds transmit FIRST and J7 freeze takes the remaining slot (0xCC does not steal
    // J7's freeze slot)!
    for (max_frames, arm_count) in [(2usize, 1usize), (3usize, 2usize)] {
        let mut motor_ids: Vec<u16> = (1..=arm_count as u16).collect();
        motor_ids.push(7);
        let mut lims: Vec<DamiaoLimits> = b601_limits()[..arm_count].to_vec();
        lims.push(b601_limits()[6]);
        let mut sub_pf = pf_cfg.clone();
        sub_pf.joint = arm_count;

        let (bus, clock) = VirtualBus::new(&lims);
        // Set the gripper motor's ID in VirtualBus to 7 so 0x307 matches cleanly.
        bus.with(|s| {
            s.motors[arm_count].0 = 7;
        });
        let mut d = DamiaoArmDriver::new(bus.clone(), motor_ids)
            .with_joint_limits(lims)
            .unwrap()
            .with_gripper_pos_force(sub_pf)
            .unwrap();
        d.read_joint_states().unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        d.set_torque(true).unwrap();
        bus.with(|s| {
            s.rx.clear();
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();

        // Age J7 past 100 ms while keeping all arm joints fresh (with fresh empty-queue bound).
        clock.advance(ms(115));
        bus.with(|s| s.rx.clear());
        d.read_joint_states().unwrap();
        bus.with(|s| {
            for i in 0..arm_count {
                s.push_feedback(i);
            }
        });
        d.set_tx_budget(TxBudget {
            max_frames,
            max_duration: ms(10),
            frame_max_age: ms(45),
        });
        bus.with(|s| {
            s.tx_log.clear();
            s.tx_accepted.clear();
        });
        d.read_joint_states().unwrap();
        let hold_tgts = vec![JointTarget::position(0.0); arm_count + 1];
        let out = d.write_estop_hold_bounded(&hold_tgts).unwrap();
        assert_eq!(out, TxBatchOutcome::Complete);
        let tx = bus.with(|s| s.tx_accepted.clone());
        let arm_indices: Vec<usize> = tx
            .iter()
            .enumerate()
            .filter(|(_, f)| (0x101..=0x106).contains(&f.id))
            .map(|(idx, _)| idx)
            .collect();
        let j7_idx = tx
            .iter()
            .position(|f| f.id == 0x307)
            .unwrap_or_else(|| panic!("max_frames={max_frames}: expected J7 freeze in spare slot: {tx:?}"));
        assert_eq!(arm_indices.len(), arm_count, "expected {arm_count} arm holds: {tx:?}");
        assert!(
            arm_indices.iter().all(|&i| i < j7_idx),
            "due arm holds must precede J7 freeze in spare slot: {tx:?}"
        );
        assert_eq!(u16::from_le_bytes([tx[j7_idx].data[4], tx[j7_idx].data[5]]), 0);
        assert!(
            !tx.iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
            "0xCC refresh must not steal the J7 freeze slot when budget equals arm_count + 1: {tx:?}"
        );
    }
}

/// Sim-before-real gate for real-arm enable (dev18 regression): end-to-end legacy WS torque-on
/// (`ArmSupervisor::torque_ws`) through the 50 Hz (20 ms) ticked transition with 7 Damiao motors
/// (J7 in PosForce mode), constrained per-tick TX budget + SPI-CAN frame time, realistic Damiao
/// reply latencies (including delayed J7 0xCC refreshes and occasional dropped initial replies),
/// across 10 deterministic seeds. Every seed (10/10) must confirm enable cleanly.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_ws_torque_on_ticked_enable_7_motors_realistic_latency_10_seeded_runs() {
    use armd::auth::OperatorGrant;
    use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};

    struct SeedRng(u64);
    impl SeedRng {
        fn new(seed: u64) -> Self {
            Self(seed.wrapping_add(0x9E37_79B9_7F4A_7C15))
        }
        fn next_u64(&mut self) -> u64 {
            let mut z = self.0;
            self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        }
        fn range(&mut self, lo: u64, hi_inclusive: u64) -> u64 {
            lo + (self.next_u64() % (hi_inclusive - lo + 1))
        }
    }

    struct RealisticDamiaoState {
        inner: VirtualBus,
        clock: MockClock,
        rng: SeedRng,
        /// When false (startup `check_mapping_ranges`), replies are delivered immediately.
        /// When true (50 Hz ticked control loop), realistic per-frame delays and drops apply.
        delayed_mode: bool,
        tx_frame_us: u64,
        j7_refresh_delay_ms: u64,
        drop_first_j7_307: bool,
        drop_first_j7_enable: bool,
        dropped_j7_307: bool,
        dropped_j7_enable: bool,
        delayed_rx: VecDeque<(Instant, CanFrame)>,
        j7_cc_sent: usize,
        enable_frames_sent: usize,
    }

    #[derive(Clone)]
    struct RealisticDamiaoBus(Arc<Mutex<RealisticDamiaoState>>);

    impl RealisticDamiaoBus {
        fn new(
            limits: &[DamiaoLimits],
            seed: u64,
            j7_starts_pos_vel: bool,
            tx_frame_us: u64,
            j7_refresh_delay_ms: u64,
            drop_first_j7_307: bool,
            drop_first_j7_enable: bool,
        ) -> (Self, VirtualBus, MockClock) {
            let (inner, clock) = VirtualBus::new(limits);
            inner.with(|s| {
                s.motors[6].2 = -1.2;
                s.motors[6].4 = if j7_starts_pos_vel {
                    damiao::CTRL_MODE_POS_VEL
                } else {
                    damiao::CTRL_MODE_POS_FORCE
                };
            });
            let bus = Self(Arc::new(Mutex::new(RealisticDamiaoState {
                inner: inner.clone(),
                clock: clock.clone(),
                rng: SeedRng::new(seed ^ 0xA5A5_5A5A),
                delayed_mode: false,
                tx_frame_us,
                j7_refresh_delay_ms,
                drop_first_j7_307,
                drop_first_j7_enable,
                dropped_j7_307: false,
                dropped_j7_enable: false,
                delayed_rx: VecDeque::new(),
                j7_cc_sent: 0,
                enable_frames_sent: 0,
            })));
            (bus, inner, clock)
        }

        fn flush_due(&self) {
            let mut st = self.0.lock().unwrap();
            let now = st.clock.now();
            let mut keep = VecDeque::new();
            while let Some((due_at, frame)) = st.delayed_rx.pop_front() {
                if due_at <= now {
                    st.inner.with(|s| s.rx.push_back((frame, Some(due_at))));
                } else {
                    keep.push_back((due_at, frame));
                }
            }
            st.delayed_rx = keep;
        }
    }

    impl CanTransport for RealisticDamiaoBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.0.lock().unwrap().inner.clock()
        }
        fn supports_nonblocking_rt(&self) -> bool { true }
        fn supports_bounded_flush(&self) -> bool { true }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let mut st = self.0.lock().unwrap();
            if !st.delayed_mode {
                return st.inner.transmit(frame);
            }
            st.clock.advance(us(st.tx_frame_us));
            let rx_before = st.inner.with(|s| s.rx.len());
            let res = st.inner.transmit(frame);
            let popped: Vec<CanFrame> = st.inner.with(|s| {
                let mut out = Vec::new();
                while s.rx.len() > rx_before {
                    if let Some((f, _)) = s.rx.pop_back() {
                        out.push(f);
                    }
                }
                out.reverse();
                out
            });
            let is_j7_307 = frame.id == 0x307;
            let is_j7_enable = frame.id == 7 && frame.data == damiao::CMD_ENABLE;
            let is_j7_cc = frame.id == 0x7FF
                && frame.data[2] == 0xCC
                && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 7;
            if frame.data == damiao::CMD_ENABLE {
                st.enable_frames_sent += 1;
            }
            if is_j7_cc {
                st.j7_cc_sent += 1;
            }
            if is_j7_307 && st.drop_first_j7_307 && !st.dropped_j7_307 {
                st.dropped_j7_307 = true;
                return res;
            }
            if is_j7_enable && st.drop_first_j7_enable && !st.dropped_j7_enable {
                st.dropped_j7_enable = true;
                return res;
            }
            let delay = if is_j7_cc {
                ms(st.j7_refresh_delay_ms)
            } else if frame.id == 0x7FF {
                ms(st.rng.range(1, 10))
            } else {
                ms(st.rng.range(1, 12))
            };
            let due_at = st.clock.now() + delay;
            for f in popped {
                st.delayed_rx.push_back((due_at, f));
            }
            res
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.flush_due();
            self.0.lock().unwrap().inner.receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            self.flush_due();
            self.0.lock().unwrap().inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.0.lock().unwrap().inner.flush()
        }
    }

    let limits = b601_limits();
    let mut succeeded = 0usize;
    let mut seeds_with_j7_refresh = 0usize;

    for seed in 0..10u64 {
        let mut rng = SeedRng::new(seed);
        // Realistic SG2002 SPI-CAN frame time (250-450 us/frame): inside the 1 ms legacy
        // transition step budget (`LEGACY_TORQUE_STEP_BUDGET`), each 20 ms release sends 2-4
        // frames before carrying the remaining enable frames to subsequent releases, while
        // keeping `max_frames = 7` (default) for post-enable steady-state ticks.
        let tx_frame_us = rng.range(250, 450);
        let max_frames = 7usize;
        let j7_refresh_delay_ms = rng.range(10, 38);
        let j7_starts_pos_vel = (seed % 2) == 0;
        let drop_first_j7_307 = (seed % 3) != 0;
        let drop_first_j7_enable = (seed % 3) == 1;
        let pre_age_ms = if drop_first_j7_307 { rng.range(50, 80) } else { rng.range(10, 40) };

        let (bus, vbus, clock) = RealisticDamiaoBus::new(
            &limits,
            seed,
            j7_starts_pos_vel,
            tx_frame_us,
            j7_refresh_delay_ms,
            drop_first_j7_307,
            drop_first_j7_enable,
        );
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
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
        // Run startup mapping-range verification synchronously (just as `armd` does at boot),
        // then switch to realistic delayed replies and constrained per-tick TX budget.
        vbus.with(|s| s.push_all_feedback());
        dm.read_joint_states().unwrap();
        let rc = dm.check_mapping_ranges();
        assert_eq!(rc.status, armd::driver::RangeCheckStatus::Passed);
        dm.set_tx_budget(TxBudget {
            max_frames,
            max_duration: ms(5),
            frame_max_age: ms(45),
        });
        bus.0.lock().unwrap().delayed_mode = true;

        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 600_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let conn = sup.try_claim_ws_slot().unwrap();
        sup.register_ws_connection_for_slot(conn.id()).await.unwrap();
        let (lease_gen, epoch) = {
            let mut inner = sup.inner().write().await;
            let g = inner
                .acquire_lease(&format!("connection:{}", conn.id()))
                .unwrap();
            vbus.with(|s| s.push_all_feedback());
            inner.tick_control(0.02);
            (g, inner.safety.epoch_id())
        };
        // Age J7 feedback prior to WS torque-on so constrained ticked enable exercises J7 refresh.
        clock.advance(ms(pre_age_ms));

        let sup_ws = sup.clone();
        let ws_task = tokio::spawn(async move {
            let stamp = sup_ws.inner().read().await.current_admission_stamp();
            sup_ws
                .torque_ws(
                    conn,
                    lease_gen,
                    Some(epoch),
                    true,
                    seed + 1,
                    Ok(true),
                    OwnerAuthPolicy::None,
                    OperatorGrant::unenforced(),
                    stamp,
                )
                .await
        });

        // Wait until the legacy WS torque-on transition is registered on the supervisor.
        let wait_dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < wait_dl {
            if sup.inner().read().await.torque_transition_pending() || ws_task.is_finished() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(1)).await;
        }

        // Drive 50 Hz (20 ms) legacy control releases until the transition settles.
        let mut ticks = 0usize;
        for _ in 0..40 {
            if ws_task.is_finished() {
                break;
            }
            ticks += 1;
            clock.advance(ms(20));
            let done = {
                let mut inner = sup.inner().write().await;
                inner.tick_control(0.02);
                !inner.torque_transition_pending()
            };
            if done {
                break;
            }
            tokio::task::yield_now().await;
        }

        let outcome = tokio::time::timeout(Duration::from_secs(2), ws_task)
            .await
            .unwrap_or_else(|_| panic!("seed={seed}: torque_ws timed out after {ticks} ticks"))
            .expect("ws_task join")
            .expect("torque_ws Ok");
        match outcome {
            TorqueWsOutcome::Accepted { is_torqued, .. } => {
                assert!(is_torqued, "seed={seed}: expected is_torqued=true");
            }
            other => {
                let st = sup.inner().read().await.safety.state().clone();
                panic!("seed={seed}: legacy WS torque-on failed (ticks={ticks}, state={st:?}): {other:?}");
            }
        }
        {
            let inner = sup.inner().read().await;
            assert!(inner.is_torqued, "seed={seed}: supervisor must be torqued");
            assert!(
                inner.safety.state().is_operational(),
                "seed={seed}: supervisor must be operational: {:?}",
                inner.safety.state()
            );
        }
        vbus.with(|s| {
            for (i, m) in s.motors.iter().enumerate() {
                assert!(m.3, "seed={seed}: motor J{} must be enabled on bus", i + 1);
            }
            assert_eq!(
                s.motors[6].4,
                damiao::CTRL_MODE_POS_FORCE,
                "seed={seed}: J7 must be in CTRL_MODE_POS_FORCE"
            );
        });
        let j7_cc = bus.0.lock().unwrap().j7_cc_sent;
        if j7_cc > 0 {
            seeds_with_j7_refresh += 1;
        }
        eprintln!(
            "seed={seed}: PASS ticks={ticks} max_frames={max_frames} tx_us={tx_frame_us} j7_cc={j7_cc} j7_cc_delay_ms={j7_refresh_delay_ms}"
        );
        succeeded += 1;
    }

    assert_eq!(succeeded, 10, "expected 10/10 seeded legacy WS enable runs to succeed");
    assert!(
        seeds_with_j7_refresh > 0,
        "expected seeded runs to exercise J7 0xCC refresh during ticked enable"
    );
    eprintln!(
        "legacy_ws_torque_on_ticked_enable: {succeeded}/10 seeded runs PASSED (j7_refresh_exercised={seeds_with_j7_refresh}/10)"
    );
}

/// Regressions for all 4 hardware-safety Blockers identified by the read-only codex-rescue audit:
/// 1. RX budget expiring after a plausible J7 reply with a fault queued behind it MUST keep J7
///    ineligible for `CMD_ENABLE` until the queue drains and reveals the fault.
/// 2. A fresh timestamped J7 hardware fault (`0x0A`) arriving during a deferred (`boundary=None`)
///    refresh MUST fail closed with `MotorHardwareFault` instead of resolving the refresh as fresh.
/// 3. Crossing either the 100 ms J7 freshness bound or the 150 ms confirmation deadline after
///    frame construction (at final submission `t0`) MUST refuse `CMD_ENABLE` before `can.transmit`.
/// 4. Lease revocation occurring during `drain_rx_for_boundary` MUST block `0xCC` transmit and
///    flush and fail closed with `LeaseFenced`.
#[test]
fn j7_enable_refresh_review_blocker_regressions() {
    use armd::rt_core::urgent::LeaseGate;

    let limits = b601_limits();
    let make_pf = || GripperPosForce {
        joint: 6,
        current_raw: 460,
        close_speed_rad_s: 0.30,
        closed_rad: 0.0,
        open_rad: -5.7,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
    };
    let build_j7_fb = |limits: &[DamiaoLimits], state: u8, pos: f32| -> CanFrame {
        let l = limits[6];
        let p = damiao::float_to_uint(pos, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        CanFrame::new_standard(
            0x17,
            &[
                ((state & 0x0f) << 4) | 7,
                (p >> 8) as u8,
                p as u8,
                (v >> 4) as u8,
                (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
                t as u8,
                30,
                30,
            ],
        )
        .unwrap()
    };

    // --- Blocker 1: Plausible J7 reply at RX budget boundary with J7 fault right behind it ---
    {
        struct SuppressJ7RefreshBus {
            inner: VirtualBus,
            suppress_j7_cc: Arc<AtomicBool>,
            suppress_j7_307: Arc<AtomicBool>,
        }
        impl CanTransport for SuppressJ7RefreshBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
            fn supports_nonblocking_rt(&self) -> bool { true }
            fn supports_bounded_flush(&self) -> bool { true }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                let rx_before = self.inner.with(|s| s.rx.len());
                let r = self.inner.transmit(frame);
                let is_j7_cc = frame.id == 0x7FF
                    && frame.data[2] == 0xCC
                    && u16::from_le_bytes([frame.data[0], frame.data[1]]) == 7;
                if (frame.id == 0x307 && self.suppress_j7_307.load(Ordering::SeqCst))
                    || (is_j7_cc && self.suppress_j7_cc.load(Ordering::SeqCst))
                {
                    self.inner.with(|s| s.rx.truncate(rx_before));
                }
                r
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
        }

        let (vbus, clock) = VirtualBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let sup_307 = Arc::new(AtomicBool::new(true));
        let sup_cc = Arc::new(AtomicBool::new(true));
        let mut d = DamiaoArmDriver::new(
            SuppressJ7RefreshBus {
                inner: vbus.clone(),
                suppress_j7_cc: sup_cc.clone(),
                suppress_j7_307: sup_307.clone(),
            },
            vec![1, 2, 3, 4, 5, 6, 7],
        )
        .with_joint_limits(limits.clone())
        .unwrap()
        .with_gripper_pos_force(make_pf())
        .unwrap();
        d.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();

        // Age to 85 ms, carry enable under max_frames=1 through 0x307, then cross 100 ms to arm 0xCC.
        clock.advance(ms(85));
        d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
        let _ = d.begin_torque(true).unwrap();
        for _ in 0..10 {
            vbus.clear();
            clock.advance(ms(1));
            let _ = d.poll_torque(clock.now() + ms(5)).unwrap();
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                break;
            }
        }
        clock.advance(ms(20)); // J7 is now stale (> 100 ms); next poll sends 0xCC and leaves refresh pending.
        d.set_tx_budget(TxBudget::default());
        vbus.clear();
        let p = d.poll_torque(clock.now() + ms(5)).expect("0xCC refresh armed");
        assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        assert!(vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC)));

        // Inject TWO post-boundary J7 frames: benign state=0 FIRST, overcurrent fault 0x0A SECOND.
        clock.advance(ms(10));
        let rx_t = clock.now();
        vbus.with(|s| {
            s.rx.push_back((build_j7_fb(&limits, 0, -1.0), Some(rx_t)));
            s.rx.push_back((build_j7_fb(&limits, 0x0A, -1.0), Some(rx_t)));
        });
        vbus.clear();
        // RX budget = 1 decodes ONLY the benign frame (updating pf_last_fresh_feedback) without
        // draining the queue! J7 MUST remain held out of CMD_ENABLE while pf_refresh is pending.
        let p2 = d
            .poll_torque_budgeted(clock.now() + ms(5), 1)
            .expect("undrained refresh must remain pending");
        assert!(matches!(p2, armd::driver::TorqueProgress::Pending { .. }));
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
            "Blocker 1: J7 CMD_ENABLE MUST NOT transmit before refresh observes an empty RX queue: {:?}",
            vbus.with(|s| s.tx_log.clone())
        );

        // Next release drains the queued 0x0A fault and MUST fail closed without enabling J7.
        clock.advance(ms(10));
        vbus.clear();
        let err = d.poll_torque(clock.now() + ms(5)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::MotorHardwareFault { joint: 6, status: 0x0A, .. }),
            "Blocker 1: queued J7 fault behind benign reply must veto enable: {err:?}"
        );
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
            "Blocker 1: faulted J7 must never receive CMD_ENABLE"
        );
    }

    // --- Blocker 2: Deferred (boundary=None) refresh receives a fresh J7 fault frame ---
    {
        let (vbus, clock) = VirtualBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let mut d = DamiaoArmDriver::new(vbus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits.clone())
            .unwrap()
            .with_gripper_pos_force(make_pf())
            .unwrap();
        d.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();

        clock.advance(ms(85));
        d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
        let _ = d.begin_torque(true).unwrap();
        for _ in 0..10 {
            vbus.clear();
            clock.advance(ms(1));
            let _ = d.poll_torque(clock.now() + ms(5)).unwrap();
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                break;
            }
        }
        vbus.with(|s| s.rx.clear());
        // Exhaust TX budget (`max_frames = 0`) while stale so `pf_refresh` is armed with `boundary: None`.
        clock.advance(ms(20));
        d.set_tx_budget(TxBudget { max_frames: 0, max_duration: ms(5), frame_max_age: ms(45) });
        let p = d.poll_torque(clock.now() + ms(5)).expect("deferred refresh pending");
        assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));

        // Restore TX budget and push a fresh timestamped J7 overcurrent fault (0x0A) with finite pos.
        d.set_tx_budget(TxBudget::default());
        clock.advance(ms(10));
        let rx_t = clock.now();
        vbus.with(|s| s.rx.push_back((build_j7_fb(&limits, 0x0A, -1.0), Some(rx_t))));
        vbus.clear();
        let err = d.poll_torque(clock.now() + ms(5)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::MotorHardwareFault { joint: 6, status: 0x0A, .. }),
            "Blocker 2: fresh J7 hardware fault during deferred refresh must fail closed: {err:?}"
        );
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
            "Blocker 2: faulted J7 must NEVER receive CMD_ENABLE"
        );
    }

    // --- Blocker 3: Clock crosses 100 ms freshness (3a) or 150 ms confirm deadline (3b) after build ---
    for case in ["freshness_100ms", "confirm_150ms"] {
        struct JumpBeforeTxClock {
            inner: MockClock,
            arm_jump: Arc<AtomicBool>,
            /// Number of `now()` calls remaining after arming before injecting `jump_ms`.
            countdown: Arc<std::sync::atomic::AtomicUsize>,
            jump_ms: u64,
            jumped: Arc<AtomicBool>,
        }
        impl DriverClock for JumpBeforeTxClock {
            fn clock_id(&self) -> u64 { self.inner.clock_id() }
            fn sleep(&self, d: Duration) { self.inner.sleep(d) }
            fn yield_now(&self) { self.inner.yield_now() }
            fn now(&self) -> Instant {
                if self.arm_jump.load(Ordering::SeqCst) && !self.jumped.load(Ordering::SeqCst) {
                    let prev = self.countdown.fetch_sub(1, Ordering::SeqCst);
                    if prev == 0 {
                        self.jumped.store(true, Ordering::SeqCst);
                        self.inner.advance(ms(self.jump_ms));
                    }
                }
                self.inner.now()
            }
        }
        struct CustomClockBus {
            inner: VirtualBus,
            clk: Arc<JumpBeforeTxClock>,
        }
        impl CanTransport for CustomClockBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> {
                Some(self.clk.clone() as Arc<dyn DriverClock>)
            }
            fn supports_nonblocking_rt(&self) -> bool { true }
            fn supports_bounded_flush(&self) -> bool { true }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> { self.inner.receive() }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.inner.receive_timestamped()
            }
            fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
        }

        let (vbus, base_clock) = VirtualBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let arm_jump = Arc::new(AtomicBool::new(false));
        let countdown = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let jumped = Arc::new(AtomicBool::new(false));
        let jump_ms = if case == "freshness_100ms" { 10 } else { 10 };
        let clk = Arc::new(JumpBeforeTxClock {
            inner: base_clock.clone(),
            arm_jump: arm_jump.clone(),
            countdown: countdown.clone(),
            jump_ms,
            jumped: jumped.clone(),
        });
        let mut d = DamiaoArmDriver::new(
            CustomClockBus { inner: vbus.clone(), clk },
            vec![1, 2, 3, 4, 5, 6, 7],
        )
        .with_joint_limits(limits.clone())
        .unwrap()
        .with_gripper_pos_force(make_pf())
        .unwrap();
        d.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();

        // Carry J7 into `poll_enable` with J1..J6 already enabled so the next poll attempts J7 CMD_ENABLE first.
        d.set_tx_budget(TxBudget { max_frames: 7, max_duration: ms(5), frame_max_age: ms(45) });
        let _ = d.begin_torque(true).unwrap();
        for _ in 0..10 {
            vbus.clear();
            base_clock.advance(ms(1));
            let _ = d.poll_torque(base_clock.now() + ms(5)).unwrap();
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                break;
            }
        }
        // Drain J1..J6 confirmations while holding J7 unsent by advancing near the target boundary.
        vbus.with(|s| s.rx.retain(|(f, _)| f.id != 0x17));
        if case == "freshness_100ms" {
            // Advance to age 95 ms (fresh at `build` and `submit_now`, stale by 5 ms at final `t0`!).
            base_clock.advance(ms(95));
        } else {
            // Push fresh J7 at +100 ms so J7 stays fresh, then advance to +145 ms (5 ms before 150 ms confirm deadline).
            base_clock.advance(ms(100));
            let t_mid = base_clock.now();
            vbus.with(|s| s.rx.push_back((build_j7_fb(&limits, 0, -1.0), Some(t_mid))));
            base_clock.advance(ms(45));
        }
        // Find the exact `now()` call between `submit_now` and `t0` inside `send()`:
        // Sweep countdown values? Instead, inside `poll_enable` (with empty RX for freshness_100ms
        // or 1 frame for confirm_150ms), `send()` calls `now()`:
        // 1: deadline check, 2: ensure_tx_pass/can_start, 3: build (pf_enable_feedback_fresh),
        // 4: submit_now, 5: can_start_tick_tx, 6: t0 (final submission timestamp!).
        // Arm jump to fire right at `t0` so `build` and `submit_now` pass, and ONLY `t0` catches it!
        // Use a dedicated `on_tx` or simply verify that jumping on `t0` refuses TX:
        let pre_now = base_clock.now();
        // Count `now()` calls up to `t0`: poll_enable (expire:1, receive_feedback: rx_over+attempt+...=4,
        // ensure_enable_pf_refresh:1, send_enable_masked->send: deadline:1, ensure_tx_pass:1,
        // can_start_tick_tx:1, build->pf_enable_feedback_fresh:1, submit_now:1, can_start_tick_tx:1, t0:1).
        // Even cleaner: set countdown so jump triggers strictly AFTER `build` and BEFORE `can.transmit`,
        // and verify `jumped == true` AND J7 `CMD_ENABLE` was NOT transmitted!
        let _ = pre_now;
        vbus.clear();
        // Try countdown targeting `t0`: for `freshness_100ms` (0 RX frames) ~11; for `confirm_150ms` (1 RX frame) ~14.
        // Regardless of exact offset inside `send()`, as long as jump occurs after `poll_enable`'s initial
        // `window.expire`, the final `t0` gate must block J7 `CMD_ENABLE` when crossed!
        countdown.store(if case == "freshness_100ms" { 10 } else { 13 }, Ordering::SeqCst);
        arm_jump.store(true, Ordering::SeqCst);
        let res = d.poll_torque(base_clock.now() + ms(50));
        arm_jump.store(false, Ordering::SeqCst);
        assert!(jumped.load(Ordering::SeqCst), "Blocker 3 ({case}): clock jump must have fired");
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)),
            "Blocker 3 ({case}): J7 CMD_ENABLE MUST NOT transmit once {case} crossed at final t0 (res={res:?})"
        );
    }

    // --- Blocker 4: Lease revocation during `drain_rx_for_boundary` blocks 0xCC TX and flush ---
    {
        struct RevokeLeaseOnRxBus {
            inner: VirtualBus,
            fenced_gen: Arc<AtomicU64>,
            arm_revoke: Arc<AtomicBool>,
            /// Counts RX calls after arming so revocation fires on the 2nd RX pass
            /// (inside `drain_rx_for_boundary`, right before `0xCC` transmit).
            rx_calls_after_arm: Arc<AtomicU64>,
            flush_after_arm: Arc<AtomicU64>,
        }
        impl RevokeLeaseOnRxBus {
            fn on_rx(&self) {
                if self.arm_revoke.load(Ordering::SeqCst)
                    && self.rx_calls_after_arm.fetch_add(1, Ordering::SeqCst) >= 1
                {
                    self.fenced_gen.store(1, Ordering::SeqCst);
                }
            }
        }
        impl CanTransport for RevokeLeaseOnRxBus {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
            fn supports_nonblocking_rt(&self) -> bool { true }
            fn supports_bounded_flush(&self) -> bool { true }
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
                self.inner.transmit(frame)
            }
            fn receive(&mut self) -> Result<CanFrame, TransportError> {
                self.on_rx();
                self.inner.receive()
            }
            fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
                self.on_rx();
                self.inner.receive_timestamped()
            }
            fn flush(&mut self) -> Result<(), TransportError> {
                if self.arm_revoke.load(Ordering::SeqCst) {
                    self.flush_after_arm.fetch_add(1, Ordering::SeqCst);
                }
                self.inner.flush()
            }
        }

        let (vbus, clock) = VirtualBus::new(&limits);
        vbus.with(|s| {
            s.motors[6].2 = -1.0;
            s.motors[6].4 = damiao::CTRL_MODE_POS_FORCE;
        });
        let cur_gen = Arc::new(AtomicU64::new(1));
        let fenced_gen = Arc::new(AtomicU64::new(0));
        let disc_hold = Arc::new(AtomicU64::new(0));
        let lease_gate = LeaseGate::new(cur_gen.clone(), fenced_gen.clone(), disc_hold);
        let estop_gate = EstopGate::new(Arc::new(AtomicU64::new(0)), Arc::new(AtomicU64::new(0)))
            .with_lease_gate(lease_gate);
        let arm_revoke = Arc::new(AtomicBool::new(false));
        let rx_calls_after_arm = Arc::new(AtomicU64::new(0));
        let flush_after_arm = Arc::new(AtomicU64::new(0));

        let mut d = DamiaoArmDriver::new(
            RevokeLeaseOnRxBus {
                inner: vbus.clone(),
                fenced_gen: fenced_gen.clone(),
                arm_revoke: arm_revoke.clone(),
                rx_calls_after_arm: rx_calls_after_arm.clone(),
                flush_after_arm: flush_after_arm.clone(),
            },
            vec![1, 2, 3, 4, 5, 6, 7],
        )
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_pos_force(make_pf())
        .unwrap();
        d.set_estop_gate(estop_gate);
        d.read_joint_states().unwrap();
        vbus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();

        clock.advance(ms(85));
        d.set_tx_budget(TxBudget { max_frames: 1, max_duration: ms(5), frame_max_age: ms(45) });
        d.set_enable_authority(Some(clock.now() + ms(500)), Some(1));
        let _ = d.begin_torque(true).unwrap();
        for _ in 0..10 {
            vbus.clear();
            clock.advance(ms(1));
            d.set_enable_authority(Some(clock.now() + ms(500)), Some(1));
            let _ = d.poll_torque(clock.now() + ms(5)).unwrap();
            if vbus.with(|s| s.tx_accepted.iter().any(|f| f.id == 0x307)) {
                break;
            }
        }
        vbus.with(|s| s.rx.clear());
        // Cross 100 ms with empty TX budget so `pf_refresh` starts deferred (`boundary: None`).
        clock.advance(ms(20));
        d.set_tx_budget(TxBudget { max_frames: 0, max_duration: ms(5), frame_max_age: ms(45) });
        d.set_enable_authority(Some(clock.now() + ms(500)), Some(1));
        let _ = d.poll_torque(clock.now() + ms(5)).unwrap();

        // Now restore TX budget, advance 1 ms, and arm lease revocation during RX drain!
        d.set_tx_budget(TxBudget::default());
        clock.advance(ms(1));
        d.set_enable_authority(Some(clock.now() + ms(500)), Some(1));
        vbus.clear();
        arm_revoke.store(true, Ordering::SeqCst);
        let err = d.poll_torque(clock.now() + ms(5)).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::LeaseFenced),
            "Blocker 4: lease revocation during refresh drain must fail with LeaseFenced: {err:?}"
        );
        assert!(
            !vbus.with(|s| s.tx_log.iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC)),
            "Blocker 4: 0xCC refresh MUST NOT transmit after lease revocation during RX drain: {:?}",
            vbus.with(|s| s.tx_log.clone())
        );
        assert_eq!(
            flush_after_arm.load(Ordering::SeqCst),
            0,
            "Blocker 4: flush() MUST NOT be called after lease revocation"
        );
        assert!(
            rx_calls_after_arm.load(Ordering::SeqCst) >= 2,
            "Blocker 4: revocation must fire during drain_rx_for_boundary (2nd RX pass), got {}",
            rx_calls_after_arm.load(Ordering::SeqCst)
        );
        assert!(!d.torque_transition_pending() && !d.is_enabled());
    }
}
