//! Lock-proof, positive-control, Stop/E-stop cancellation, and RangeCheck recovery tests for
//! legacy WS `/api/v1/torque` routed through the ticked transition service.

mod common;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, DamiaoArmDriver, RangeCheckStatus};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::supervisor::EnableLifecycle;
use armd::supervisor_lock::HoldKind;
use armd::transport::{CanFrame, CanTransport, TransportError};
use armd::{create_router, ArmConfig, ArmSupervisor, FaultCode, OperatorAuth};
use armd::safety::FaultKind;
use serde_json::json;
use tokio::net::TcpListener;

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

#[derive(Clone)]
struct WallMotor {
    id: u16,
    q: f32,
    enabled: bool,
    limits: DamiaoLimits,
    pmax_override: Option<f32>,
    silent: bool,
    drop_reg_read: bool,
    drop_vmax_read: bool,
    drop_enable: bool,
    enable_reply_delay: Duration,
    disable_reply_delay: Duration,
    torque_ready_at: Option<Instant>,
}

impl WallMotor {
    fn new(id: u16, limits: DamiaoLimits) -> Self {
        Self {
            id,
            q: 0.0,
            enabled: false,
            limits,
            pmax_override: None,
            silent: false,
            drop_reg_read: false,
            drop_vmax_read: false,
            drop_enable: false,
            enable_reply_delay: Duration::ZERO,
            disable_reply_delay: Duration::ZERO,
            torque_ready_at: None,
        }
    }

    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = u8::from(self.enabled);
        let data = [
            (state << 4) | (self.id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            30,
            30,
        ];
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

struct WallBusState {
    motors: [WallMotor; 7],
    rx: VecDeque<(CanFrame, Option<Instant>)>,
    delayed: Vec<(Instant, CanFrame)>,
    tx_log: Vec<CanFrame>,
    /// When true, `receive_timestamped` continuously supplies extra background feedback frames
    /// to test bounded RX pass discipline under sustained CAN RX backlog.
    sustained_rx_backlog: bool,
    /// When true, `CMD_DISABLE` transmits fail with `TransportError::WouldBlock`.
    fail_disable_tx: bool,
    /// One-shot sleep injected inside `receive_timestamped` (testing mid-RX authority expiry
    /// before retry `CMD_ENABLE` TX inside `poll_torque_budgeted`).
    rx_delay_once: Duration,
    /// Timestamps of every `CMD_ENABLE` frame actually transmitted on the bus.
    enable_tx_instants: Vec<Instant>,
}

impl WallBusState {
    fn push_all_feedback(&mut self) {
        let now = Instant::now();
        for m in &self.motors {
            if !m.silent {
                self.rx.push_back((m.feedback_frame(), Some(now)));
            }
        }
    }

    fn drain_matured(&mut self, now: Instant) {
        let mut i = 0;
        while i < self.delayed.len() {
            if self.delayed[i].0 <= now {
                let (at, frame) = self.delayed.swap_remove(i);
                self.rx.push_back((frame, Some(at)));
            } else {
                i += 1;
            }
        }
    }
}

/// Real wall-clock Damiao CAN bus fake (`clock() -> None`) so `DamiaoArmDriver` uses real
/// `Instant::now()` and real `std::thread::sleep` in synchronous paths, while withholding
/// enable/disable confirmation replies for a real wall-clock delay (e.g. 35 ms).
#[derive(Clone)]
struct WallClockDelayedBus(Arc<Mutex<WallBusState>>);

impl WallClockDelayedBus {
    fn new() -> Self {
        let limits = dm_limits();
        let motors = std::array::from_fn(|i| WallMotor::new((i + 1) as u16, limits[i]));
        Self(Arc::new(Mutex::new(WallBusState {
            motors,
            rx: VecDeque::with_capacity(256),
            delayed: Vec::new(),
            tx_log: Vec::new(),
            sustained_rx_backlog: false,
            fail_disable_tx: false,
            rx_delay_once: Duration::ZERO,
            enable_tx_instants: Vec::new(),
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut WallBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn enable_tx_count(&self) -> usize {
        self.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_ENABLE)
                .count()
        })
    }

    fn disable_tx_count(&self) -> usize {
        self.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count()
        })
    }
}

impl CanTransport for WallClockDelayedBus {
    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let now = Instant::now();
        let mut s = self.0.lock().unwrap();
        s.drain_matured(now);
        s.tx_log.push(*frame);

        if frame.id == 0x7FF {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let (op, rid) = (frame.data[2], frame.data[3]);
            let Some(k) = s.motors.iter().position(|m| m.id == id) else {
                return Ok(());
            };
            if s.motors[k].silent {
                return Ok(());
            }
            match op {
                0xCC => {
                    let suppressed = s.motors[k].torque_ready_at.is_some_and(|ready| now < ready);
                    if !suppressed {
                        let f = s.motors[k].feedback_frame();
                        s.rx.push_back((f, Some(now)));
                    }
                }
                damiao::REG_READ => {
                    if s.motors[k].drop_reg_read {
                        return Ok(());
                    }
                    if s.motors[k].drop_vmax_read && rid == damiao::RID_VMAX {
                        return Ok(());
                    }
                    let m = &s.motors[k];
                    let val: Option<[u8; 4]> = match rid {
                        damiao::RID_CTRL_MODE => Some(damiao::CTRL_MODE_POS_VEL.to_le_bytes()),
                        damiao::RID_PMAX => {
                            Some(m.pmax_override.unwrap_or(m.limits.p_max).to_le_bytes())
                        }
                        damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                        damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                        _ => None,
                    };
                    if let Some(v) = val {
                        let [lo, hi] = id.to_le_bytes();
                        let reply = CanFrame::new_standard(
                            0x10 + id,
                            &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                        )
                        .unwrap();
                        s.rx.push_back((reply, Some(now)));
                    }
                }
                _ => {}
            }
            return Ok(());
        }

        if let Some(k) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if s.motors[k].silent {
                return Ok(());
            }
            if frame.data == damiao::CMD_ENABLE {
                s.enable_tx_instants.push(now);
                if s.motors[k].drop_enable {
                    return Ok(());
                }
                s.motors[k].enabled = true;
                let delay = s.motors[k].enable_reply_delay;
                if !delay.is_zero() {
                    let ready_at = match s.motors[k].torque_ready_at {
                        Some(prev) if prev > now => prev,
                        _ => {
                            let at = now + delay;
                            s.motors[k].torque_ready_at = Some(at);
                            at
                        }
                    };
                    let fb = s.motors[k].feedback_frame();
                    s.delayed.push((ready_at, fb));
                } else {
                    s.motors[k].torque_ready_at = None;
                    let fb = s.motors[k].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
                return Ok(());
            }
            if frame.data == damiao::CMD_DISABLE {
                if s.fail_disable_tx {
                    return Err(TransportError::WouldBlock);
                }
                s.motors[k].enabled = false;
                let delay = s.motors[k].disable_reply_delay;
                if !delay.is_zero() {
                    let ready_at = match s.motors[k].torque_ready_at {
                        Some(prev) if prev > now => prev,
                        _ => {
                            let at = now + delay;
                            s.motors[k].torque_ready_at = Some(at);
                            at
                        }
                    };
                    let fb = s.motors[k].feedback_frame();
                    s.delayed.push((ready_at, fb));
                } else {
                    s.motors[k].torque_ready_at = None;
                    let fb = s.motors[k].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
                return Ok(());
            }
            let fb = s.motors[k].feedback_frame();
            s.rx.push_back((fb, Some(now)));
            return Ok(());
        }

        if let Some(k) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id)
        {
            if !s.motors[k].silent {
                let suppressed = s.motors[k].torque_ready_at.is_some_and(|ready| now < ready);
                if !suppressed {
                    let fb = s.motors[k].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let sleep_once = {
            let mut s = self.0.lock().unwrap();
            std::mem::replace(&mut s.rx_delay_once, Duration::ZERO)
        };
        if !sleep_once.is_zero() {
            std::thread::sleep(sleep_once);
        }
        let now = Instant::now();
        let mut s = self.0.lock().unwrap();
        s.drain_matured(now);
        if let Some(frame) = s.rx.pop_front() {
            return Ok(frame);
        }
        if s.sustained_rx_backlog {
            // Simulate a continuously readable CAN RX queue (always another frame ready).
            let fb = s.motors[0].feedback_frame();
            return Ok((fb, Some(now)));
        }
        Err(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

async fn spawn_legacy_rig(
    bus: WallClockDelayedBus,
    start_ticker: bool,
) -> (
    std::net::SocketAddr,
    Arc<ArmSupervisor>,
    Arc<AtomicBool>,
    Arc<AtomicU64>,
) {
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();

    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm)),
    );
    sup.control_tick(0.02).await;

    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    let stop_flag = Arc::new(AtomicBool::new(false));
    let ticks = Arc::new(AtomicU64::new(0));
    if start_ticker {
        let sup_tick = sup.clone();
        let stop_tick = stop_flag.clone();
        let ticks_bg = ticks.clone();
        tokio::spawn(async move {
            // Production 50 Hz (20 ms) control cadence.
            let mut interval = tokio::time::interval(Duration::from_millis(20));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            while !stop_tick.load(Ordering::Relaxed) {
                interval.tick().await;
                sup_tick.control_tick(0.02).await;
                ticks_bg.fetch_add(1, Ordering::Release);
            }
        });
    }
    (addr, sup, stop_flag, ticks)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn lock_proof_legacy_ws_enable_withheld_35ms_bounds_every_guard_hold_under_2ms() {
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, ticks) = spawn_legacy_rig(bus.clone(), true).await;
    let (mut ws, gen) = common::controller(addr).await;

    // Wait for ticker to warm up, then arm per-hold history recording.
    let t0 = ticks.load(Ordering::Acquire);
    while ticks.load(Ordering::Acquire) < t0 + 3 {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Withhold motor 3 enable confirmation for 35 ms of real wall-clock time.
    bus.with(|s| s.motors[2].enable_reply_delay = Duration::from_millis(35));
    let epoch = sup.get_observation().await.epoch_id;

    sup.inner().set_hold_history_enabled(true);
    let ticks_before = ticks.load(Ordering::Acquire);
    let samples_before = sup.tick_samples().len();
    let started = Instant::now();
    let ack = common::rpc(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
    let elapsed = started.elapsed();
    let ticks_after = ticks.load(Ordering::Acquire);
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);
    stop_flag.store(true, Ordering::Relaxed);

    assert_eq!(ack["type"], "command_ack", "expected command_ack: {ack}");
    assert_eq!(ack["is_torqued"], true, "expected torqued=true: {ack}");
    assert!(
        elapsed >= Duration::from_millis(30),
        "bus must actually withhold confirmation for ~35 ms (got {elapsed:?})"
    );
    assert!(
        ticks_after >= ticks_before + 2,
        "control loop must continue releasing during the 35 ms enable ({ticks_before} -> {ticks_after})"
    );
    assert!(!holds.is_empty(), "expected recorded SupervisorLock holds");

    let max_hold = holds.iter().map(|h| h.hold()).max().unwrap_or(Duration::ZERO);
    let max_write_hold = holds
        .iter()
        .filter(|h| h.kind == HoldKind::Write)
        .map(|h| h.hold())
        .max()
        .unwrap_or(Duration::ZERO);
    let torque_holds: Vec<_> = holds
        .iter()
        .filter(|h| h.command == Some("/api/v1/torque"))
        .collect();
    assert!(
        !torque_holds.is_empty(),
        "expected /api/v1/torque hold attribution in history: {holds:?}"
    );
    for h in &torque_holds {
        assert_eq!(h.direction, Some("on"), "expected direction=\"on\": {h:?}");
        let io = h.io_timing.expect("expected HoldIoTiming on /api/v1/torque hold");
        assert_eq!(
            io.sleep_us, 0,
            "ticked legacy WS enable must never sleep under SupervisorLock: {h:?}"
        );
    }

    let samples = sup.tick_samples();
    let window_samples = &samples[samples_before.min(samples.len())..];
    let max_tick_wait_us = window_samples.iter().map(|s| s.wait_us).max().unwrap_or(0);
    // Unconditionally assert BOTH:
    // (a) Holder JSON from WS admission guard, AND
    // (b) Live control-tick `torque_step` attribution JSON on ticks that serviced the handshake.
    let torque_sample_holder = window_samples
        .iter()
        .find_map(|s| {
            s.longest_hold
                .as_ref()
                .filter(|h| h.command == Some("/api/v1/torque"))
                .or_else(|| {
                    s.waited_for
                        .as_ref()
                        .filter(|h| h.command == Some("/api/v1/torque"))
                })
        })
        .expect("expected /api/v1/torque Holder attribution in tick_samples window");
    let v = serde_json::to_value(torque_sample_holder).unwrap();
    assert_eq!(v["direction"], "on", "Holder JSON must serialize direction: {v}");
    assert!(v.get("tx_us").is_some(), "Holder JSON must include tx_us: {v}");
    assert!(v.get("rx_us").is_some(), "Holder JSON must include rx_us: {v}");
    assert_eq!(v["sleep_us"], 0, "Holder JSON must report sleep_us=0: {v}");

    let tick_torque_step = window_samples
        .iter()
        .find_map(|s| s.torque_step)
        .expect("expected live control-tick torque_step attribution during 35 ms enable");
    let ts_json = serde_json::to_value(tick_torque_step).unwrap();
    assert_eq!(ts_json["command"], "/api/v1/torque", "{ts_json}");
    assert_eq!(ts_json["direction"], "on", "{ts_json}");
    assert_eq!(ts_json["sleep_us"], 0, "{ts_json}");
    assert!(
        torque_holds.len() >= 2,
        "hold history must record both WS admission and live control-tick torque holds (got {})",
        torque_holds.len()
    );

    let tq_max = torque_holds.iter().map(|h| h.hold()).max().unwrap();
    let tq_io = torque_holds[0].io_timing.unwrap();
    println!(
        "[LOCK-PROOF enable 35ms] elapsed_ms={:.2} releases_during_enable={} holds={} max_guard_hold_us={} max_write_hold_us={} torque_hold_us={} max_tick_wait_us={} direction={:?} tx_us={} rx_us={} sleep_us={}",
        elapsed.as_secs_f64() * 1e3,
        ticks_after - ticks_before,
        holds.len(),
        max_hold.as_micros(),
        max_write_hold.as_micros(),
        tq_max.as_micros(),
        max_tick_wait_us,
        torque_holds[0].direction,
        tq_io.tx_us,
        tq_io.rx_us,
        tq_io.sleep_us,
    );

    assert_all_holds_bounded_2ms(&holds, "35 ms enable");
    assert!(
        max_tick_wait_us <= 2_000,
        "control tick lock wait during 35 ms enable must stay <= 2000 us (got {max_tick_wait_us} us)"
    );
}

fn assert_all_holds_bounded_2ms(holds: &[armd::supervisor_lock::HoldRecord], label: &str) {
    assert!(!holds.is_empty(), "expected recorded SupervisorLock holds during {label}");
    for h in holds {
        assert_eq!(
            h.io_timing.map_or(0, |io| io.sleep_us),
            0,
            "SupervisorLock guard must never sleep during {label}: {h:?}"
        );
        if let Some(cpu) = h.cpu_hold {
            assert!(
                cpu <= Duration::from_millis(2),
                "EVERY SupervisorLock guard on-CPU hold during {label} must stay <= 2 ms (cpu={cpu:?}, wall={:?}, hold={h:?})",
                h.hold(),
            );
        }
        if h.involuntary_csw == 0 {
            assert!(
                h.hold() <= Duration::from_millis(2),
                "EVERY unpreempted SupervisorLock guard hold during {label} must stay <= 2 ms (wall={:?}, hold={h:?})",
                h.hold(),
            );
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn positive_control_synchronous_enable_under_write_lock_fails_2ms_bound_and_reports_sleep() {
    let bus = WallClockDelayedBus::new();
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();

    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm)),
    );
    {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
    }
    sup.control_tick(0.02).await;

    // Exact same 35 ms withheld enable confirmation as the lock-proof test above.
    bus.with(|s| s.motors[2].enable_reply_delay = Duration::from_millis(35));
    sup.inner().set_hold_history_enabled(true);
    {
        // Old synchronous path: holds `SupervisorLock::write()` across `enable_torque`.
        let mut inner = sup.inner().write().await;
        let io_scope = armd::diagnostics::tick::HoldIoScope::enter();
        inner.set_command("/api/v1/torque");
        inner.set_direction("on");
        let stamp = inner.current_admission_stamp();
        let res = inner
            .check_admission(stamp)
            .and_then(armd::SupervisorInner::enable_torque);
        inner.set_io_timing(io_scope.finish());
        assert!(res.is_ok(), "synchronous enable should succeed: {res:?}");
    }
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);

    let sync_hold = holds
        .iter()
        .find(|h| h.command == Some("/api/v1/torque"))
        .expect("positive control hold must be recorded");
    let io = sync_hold
        .io_timing
        .expect("positive control must record HoldIoTiming");
    println!(
        "[POSITIVE-CONTROL sync enable 35ms] hold_us={} direction={:?} tx_us={} rx_us={} sleep_us={}",
        sync_hold.hold().as_micros(),
        sync_hold.direction,
        io.tx_us,
        io.rx_us,
        io.sleep_us,
    );
    assert!(
        sync_hold.hold() > Duration::from_millis(2),
        "positive control MUST violate the <= 2 ms bound (got {:?})",
        sync_hold.hold()
    );
    assert!(
        sync_hold.hold() >= Duration::from_millis(30),
        "positive control should hold lock for ~35 ms (got {:?})",
        sync_hold.hold()
    );
    assert_eq!(sync_hold.direction, Some("on"));
    assert!(
        io.sleep_us >= 20_000,
        "positive control must attribute >= 20 ms of sleep_us under the write lock (got {io:?})"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn lock_proof_legacy_ws_disable_delayed_35ms_bounds_every_guard_hold_under_2ms() {
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, ticks) = spawn_legacy_rig(bus.clone(), true).await;
    let (mut ws, gen) = common::controller(addr).await;

    // First arm at rest pose (q = 0.0).
    let ep0 = sup.get_observation().await.epoch_id;
    let ack_on = common::rpc(&mut ws, gen, ep0, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack_on["type"], "command_ack", "{ack_on}");

    // Ensure K=3 rest-pose ticks have settled before testing delayed disable.
    let t0 = ticks.load(Ordering::Acquire);
    while ticks.load(Ordering::Acquire) < t0 + 5 {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    bus.with(|s| s.motors[4].disable_reply_delay = Duration::from_millis(35));
    let ep1 = sup.get_observation().await.epoch_id;

    sup.inner().set_hold_history_enabled(true);
    let ticks_before = ticks.load(Ordering::Acquire);
    let started = Instant::now();
    let ack_off =
        common::rpc(&mut ws, gen, ep1, "/api/v1/torque", json!({"enable": false})).await;
    let elapsed = started.elapsed();
    let ticks_after = ticks.load(Ordering::Acquire);
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);
    stop_flag.store(true, Ordering::Relaxed);

    assert_eq!(ack_off["type"], "command_ack", "{ack_off}");
    assert_eq!(ack_off["is_torqued"], false, "{ack_off}");
    assert!(
        elapsed >= Duration::from_millis(30),
        "disable confirmation delay must actually span ~35 ms (got {elapsed:?})"
    );
    assert!(
        ticks_after >= ticks_before + 2,
        "control loop must keep releasing during 35 ms disable"
    );

    let max_hold = holds.iter().map(|h| h.hold()).max().unwrap_or(Duration::ZERO);
    let torque_holds: Vec<_> = holds
        .iter()
        .filter(|h| h.command == Some("/api/v1/torque"))
        .collect();
    assert!(!torque_holds.is_empty());
    for h in &torque_holds {
        assert_eq!(h.direction, Some("off"), "{h:?}");
        let io = h.io_timing.expect("expected HoldIoTiming on disable hold");
        assert_eq!(io.sleep_us, 0, "disable hold must never sleep: {h:?}");
    }
    println!(
        "[LOCK-PROOF disable 35ms] elapsed_ms={:.2} releases_during_disable={} holds={} max_guard_hold_us={} direction={:?}",
        elapsed.as_secs_f64() * 1e3,
        ticks_after - ticks_before,
        holds.len(),
        max_hold.as_micros(),
        torque_holds[0].direction,
    );
    assert_all_holds_bounded_2ms(&holds, "35 ms disable");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_stop_during_enable_aborts_cleanly_pre_and_post_partial_and_estop_wins() {
    // Subcase A: Stop arrives BEFORE any CMD_ENABLE frame is sent (during RangeCheck recovery
    // while motor 2 drops REG_READ but still reports fresh state=0 feedback) -> 0 enable frames
    // ever sent, Disarmed, is_torqued=false, no fault.
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| {
            s.motors[1].drop_reg_read = true;
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        assert_eq!(dm.check_mapping_ranges().status, RangeCheckStatus::Unverified);

        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });
        let stop_flag = Arc::new(AtomicBool::new(false));
        let sup_tick = sup.clone();
        let stop_tick = stop_flag.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(10));
            while !stop_tick.load(Ordering::Relaxed) {
                interval.tick().await;
                sup_tick.control_tick(0.01).await;
            }
        });

        let (mut ws, gen) = common::controller(addr).await;
        // Motor 2 stays silent so RangeCheck remains in-flight without sending CMD_ENABLE.
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "command", "request_id": 101, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
        )
        .await;

        let wait_dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < wait_dl {
            if sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        assert!(
            sup.inner().read().await.torque_transition_pending(),
            "enable transition must be in flight during RangeCheck before Stop"
        );
        // Client Stop arrives before any CMD_ENABLE frame was sent.
        {
            let mut inner = sup.inner().write().await;
            inner.authorized_stop();
        }
        let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
        tokio::time::sleep(Duration::from_millis(40)).await;
        let tx_after = bus.enable_tx_count();
        stop_flag.store(true, Ordering::Relaxed);

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(reply["code"], "STOPPED", "{reply}");
        assert_eq!(
            tx_after, 0,
            "zero enable frames may be sent when Stop aborts before enable dispatch"
        );
        let inner = sup.inner().read().await;
        assert!(!inner.torque_transition_pending());
        assert!(!inner.is_torqued);
        assert_eq!(inner.enable_lifecycle(), EnableLifecycle::Disarmed);
        assert!(
            !inner.safety.has_latched_fault_code(FaultCode::EnableFailed),
            "ordinary Stop during enable must NOT latch EnableFailed"
        );
    }

    // Subcase B: Partial enable (motors 1..6 enabled, motor 7 drops enable) then Stop.
    {
        let bus = WallClockDelayedBus::new();
        let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), true).await;
        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| {
            s.motors[6].drop_enable = true;
        });
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "command", "request_id": 102, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
        )
        .await;

        let wait_dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < wait_dl {
            if sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let tx_at_stop = {
            let mut inner = sup.inner().write().await;
            inner.authorized_stop();
            bus.enable_tx_count()
        };
        let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
        tokio::time::sleep(Duration::from_millis(40)).await;
        let tx_after = bus.enable_tx_count();
        let disables = bus.disable_tx_count();
        stop_flag.store(true, Ordering::Relaxed);

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(reply["code"], "STOPPED", "{reply}");
        assert_eq!(tx_after, tx_at_stop, "no further enable frames after Stop");
        assert_eq!(disables, 0, "no rollback disable frames on partial-enable Stop");
        let inner = sup.inner().read().await;
        assert!(inner.is_torqued, "energized joints must stay held");
        assert_eq!(inner.enable_lifecycle(), EnableLifecycle::FailedHold);
        assert!(inner.pending_rearm);
        assert!(
            !inner.safety.has_latched_fault_code(FaultCode::EnableFailed),
            "Stop during partial enable must NOT latch EnableFailed"
        );
    }

    // Subcase C: E-stop during enable wins with ESTOP_ACTIVE and preserves E-stop latch.
    {
        let bus = WallClockDelayedBus::new();
        let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), true).await;
        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| s.motors[3].drop_enable = true);
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "command", "request_id": 103, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
        )
        .await;

        let wait_dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < wait_dl {
            if sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let _ = sup.trigger_e_stop().await;
        let tx_at_estop = bus.enable_tx_count();
        let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
        tokio::time::sleep(Duration::from_millis(40)).await;
        let tx_after = bus.enable_tx_count();
        stop_flag.store(true, Ordering::Relaxed);

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(reply["code"], "ESTOP_ACTIVE", "{reply}");
        assert_eq!(tx_after, tx_at_estop, "no enable frames after E-stop");
        let inner = sup.inner().read().await;
        assert!(
            inner.safety.has_latched_fault_kind(FaultKind::Estop),
            "E-stop fault kind must remain latched"
        );
        assert!(
            inner.safety.state().is_stopped(),
            "safety state must remain stopped after E-stop during enable: {:?}",
            inner.safety.state()
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_unverified_range_check_recovers_and_mismatch_refuses_with_zero_enables() {
    // Subcase A: motor 2 silent at startup (`Unverified`), answers later -> ticked recovery passes
    // and arms with every guard hold <= 2 ms.
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| {
            s.motors[1].silent = true;
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        let _ = dm.read_joint_states();
        let rc = dm.check_mapping_ranges();
        assert_eq!(rc.status, RangeCheckStatus::Unverified);

        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });
        let stop_flag = Arc::new(AtomicBool::new(false));
        let sup_tick = sup.clone();
        let stop_tick = stop_flag.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(10));
            while !stop_tick.load(Ordering::Relaxed) {
                interval.tick().await;
                sup_tick.control_tick(0.01).await;
            }
        });

        let (mut ws, gen) = common::controller(addr).await;
        // Motor 2 comes back online before legacy WS torque enable.
        bus.with(|s| {
            s.motors[1].silent = false;
            s.push_all_feedback();
        });
        tokio::time::sleep(Duration::from_millis(30)).await;
        let ep = sup.get_observation().await.epoch_id;

        sup.inner().set_hold_history_enabled(true);
        let ack = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let holds = sup.inner().take_hold_history();
        sup.inner().set_hold_history_enabled(false);
        stop_flag.store(true, Ordering::Relaxed);

        assert_eq!(ack["type"], "command_ack", "{ack}");
        assert_eq!(ack["is_torqued"], true);
        assert_all_holds_bounded_2ms(&holds, "RangeCheck recovery + enable");
    }

    // Subcase B: motor 2 silent at startup (`Unverified`), later answers with mismatched PMAX ->
    // ticked range check refuses with ENABLE_FAILED and sends ZERO CMD_ENABLE frames.
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| {
            s.motors[1].silent = true;
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        let _ = dm.read_joint_states();
        assert_eq!(dm.check_mapping_ranges().status, RangeCheckStatus::Unverified);

        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });
        let stop_flag = Arc::new(AtomicBool::new(false));
        let sup_tick = sup.clone();
        let stop_tick = stop_flag.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(10));
            while !stop_tick.load(Ordering::Relaxed) {
                interval.tick().await;
                sup_tick.control_tick(0.01).await;
            }
        });

        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| {
            s.motors[1].silent = false;
            s.motors[1].pmax_override = Some(s.motors[1].limits.p_max * 2.0 + 3.0);
            s.push_all_feedback();
            s.tx_log.clear();
        });
        tokio::time::sleep(Duration::from_millis(30)).await;
        let ep = sup.get_observation().await.epoch_id;

        let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        stop_flag.store(true, Ordering::Relaxed);

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(
            reply["code"], "CLEAR_FAULT_REJECTED",
            "legacy WS wire code for hardware enable failure must remain CLEAR_FAULT_REJECTED: {reply}"
        );
        assert_eq!(reply["safety_state"], "Fault", "{reply}");
        assert!(
            sup.inner()
                .read()
                .await
                .safety
                .has_latched_fault_code(FaultCode::EnableFailed),
            "EnableFailed fault code must be latched on range mismatch"
        );
        assert_eq!(
            bus.enable_tx_count(),
            0,
            "mismatched range check must refuse with ZERO CMD_ENABLE frames"
        );
    }

    // Subcase C (Major 8): partial range reply where PMAX mismatches AND VMAX is withheld ->
    // MUST still reject as Mismatch with ZERO CMD_ENABLE frames (never fall through as Unverified).
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| {
            s.motors[1].silent = true;
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        let _ = dm.read_joint_states();
        assert_eq!(dm.check_mapping_ranges().status, RangeCheckStatus::Unverified);

        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });
        let stop_flag = Arc::new(AtomicBool::new(false));
        let sup_tick = sup.clone();
        let stop_tick = stop_flag.clone();
        tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(20));
            while !stop_tick.load(Ordering::Relaxed) {
                interval.tick().await;
                sup_tick.control_tick(0.02).await;
            }
        });

        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| {
            s.motors[1].silent = false;
            s.motors[1].pmax_override = Some(s.motors[1].limits.p_max * 2.0 + 3.0);
            s.motors[1].drop_vmax_read = true;
            s.push_all_feedback();
            s.tx_log.clear();
        });
        tokio::time::sleep(Duration::from_millis(40)).await;
        let ep = sup.get_observation().await.epoch_id;

        let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        stop_flag.store(true, Ordering::Relaxed);

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(reply["code"], "CLEAR_FAULT_REJECTED", "{reply}");
        assert_eq!(
            bus.enable_tx_count(),
            0,
            "partial range reply with mismatched PMAX + missing VMAX must refuse with ZERO CMD_ENABLE frames"
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_sustained_rx_backlog_at_50hz_stays_under_2ms() {
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, ticks) = spawn_legacy_rig(bus.clone(), true).await;
    let (mut ws, gen) = common::controller(addr).await;

    let t0 = ticks.load(Ordering::Acquire);
    while ticks.load(Ordering::Acquire) < t0 + 2 {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    bus.with(|s| {
        s.motors[2].enable_reply_delay = Duration::from_millis(35);
        s.sustained_rx_backlog = true;
    });
    let t_before = ticks.load(Ordering::Acquire);
    let ep = sup.get_observation().await.epoch_id;
    sup.inner().set_hold_history_enabled(true);
    let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);
    let t_after = ticks.load(Ordering::Acquire);
    stop_flag.store(true, Ordering::Relaxed);

    // Under continuous RX flood, transition may succeed or fail cleanly (RxDrainTimeout),
    // but EVERY SupervisorLock guard hold (WS + 50 Hz ticks) must strictly remain <= 2 ms.
    assert!(
        reply["type"] == "command_ack" || reply["type"] == "command_reject",
        "{reply}"
    );
    assert!(
        !holds.is_empty(),
        "hold history must be non-empty during sustained RX backlog test"
    );
    assert!(
        t_after > t_before,
        "live 50 Hz control loop must have executed ticks during the transition (before={t_before}, after={t_after})"
    );
    assert!(
        holds.iter().any(|h| h.command == Some("/api/v1/torque")),
        "hold history must include attributed /api/v1/torque guards"
    );
    assert_all_holds_bounded_2ms(&holds, "sustained RX backlog at 50 Hz");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_stalled_ticker_real_deadline_cleans_up_enable_and_detaches_dispatched_disable() {
    use armd::auth::OperatorGrant;
    use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};

    // 1. Real WS deadline with paused ticker on Damiao enable: waiter gets DEADLINE_EXPIRED,
    //    driver's `torque_hs` is actually cancelled (not pending), and measured hold write succeeds.
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| s.push_all_feedback());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let conn = sup.try_claim_ws_slot().unwrap();
        sup.register_ws_connection_for_slot(conn.id()).await.unwrap();
        let gen = {
            let mut inner = sup.inner().write().await;
            let gen = inner
                .acquire_lease(&format!("connection:{}", conn.id()))
                .unwrap();
            inner.tick_control(0.02);
            gen
        };
        // Withhold motor 3 longer than our 90 ms WS deadline, and keep ticker paused.
        bus.with(|s| s.motors[2].enable_reply_delay = Duration::from_millis(400));
        let ep = sup.inner().read().await.safety.epoch_id();
        let stamp = sup.inner().read().await.current_admission_stamp();
        let out = sup
            .torque_ws_with_deadline(
                conn,
                gen,
                Some(ep),
                true,
                201,
                Ok(true),
                OwnerAuthPolicy::None,
                OperatorGrant::unenforced(),
                stamp,
                Duration::from_millis(90),
            )
            .await
            .unwrap();
        assert!(
            matches!(out, TorqueWsOutcome::Rejected { ref error, .. } if error.code.as_deref() == Some("DEADLINE_EXPIRED")),
            "expected DEADLINE_EXPIRED on stalled ticker enable: {out:?}"
        );
        let mut inner = sup.inner().write().await;
        assert!(!inner.torque_transition_pending(), "enable transition must be cleaned up");
        let drv_pending = inner
            .driver
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .torque_transition_pending();
        assert!(
            !drv_pending,
            "Damiao driver torque_hs must be cancelled on deadline so measured hold writes succeed"
        );
        // Verify subsequent control tick runs measured hold cleanly (no TorqueTransitionPending).
        inner.tick_control(0.02);
    }

    // 2. Real WS deadline during partially dispatched disable: waiter gets DEADLINE_EXPIRED,
    //    disable stays Detached (NOT aborted), and resuming ticks finishes disabling hardware!
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| s.push_all_feedback());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        let conn = sup.try_claim_ws_slot().unwrap();
        sup.register_ws_connection_for_slot(conn.id()).await.unwrap();
        let gen = {
            let mut inner = sup.inner().write().await;
            let gen = inner
                .acquire_lease(&format!("connection:{}", conn.id()))
                .unwrap();
            let stamp = inner.current_admission_stamp();
            inner
                .check_admission(stamp)
                .and_then(armd::SupervisorInner::enable_torque)
                .unwrap();
            for _ in 0..4 {
                inner.tick_control(0.02);
            }
            gen
        };
        // Motor 5 delays disable confirmation by 220 ms; WS deadline is 75 ms (after tick_helper
        // dispatches CMD_DISABLE at 12..48 ms, strictly before Damiao's 3-repeat fallback window).
        bus.with(|s| s.motors[4].disable_reply_delay = Duration::from_millis(220));
        let ep = sup.inner().read().await.safety.epoch_id();
        let stamp = sup.inner().read().await.current_admission_stamp();
        let sup_bg = sup.clone();
        // Run four ticks right after disable starts so RX drains, K=3 rest-pose window confirms,
        // and CMD_DISABLE frames are dispatched to motors 1..7, then pause the ticker so the
        // 75 ms deadline fires while motor 5's confirmation is still withheld.
        let tick_helper = tokio::spawn(async move {
            for _ in 0..4 {
                tokio::time::sleep(Duration::from_millis(12)).await;
                sup_bg.control_tick(0.02).await;
            }
        });
        let out = sup
            .torque_ws_with_deadline(
                conn,
                gen,
                Some(ep),
                true,
                202,
                Ok(false),
                OwnerAuthPolicy::None,
                OperatorGrant::unenforced(),
                stamp,
                Duration::from_millis(75),
            )
            .await
            .unwrap();
        let _ = tick_helper.await;
        assert!(
            matches!(out, TorqueWsOutcome::Rejected { ref error, .. } if error.code.as_deref() == Some("DEADLINE_EXPIRED")),
            "expected DEADLINE_EXPIRED on delayed dispatched disable: {out:?}"
        );
        assert!(
            sup.inner().read().await.torque_transition_pending(),
            "dispatched disable must remain active as Detached after waiter timeout"
        );
        // Wait for motor 5's delayed reply to mature, then resume ticks and assert full disable!
        tokio::time::sleep(Duration::from_millis(180)).await;
        for _ in 0..10 {
            sup.control_tick(0.02).await;
            if !sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let inner = sup.inner().read().await;
        assert!(
            !inner.torque_transition_pending(),
            "detached disable must complete once control ticks resume"
        );
        assert!(!inner.is_torqued, "arm must reach torqued=false after detached disable completes");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_lease_expiry_during_enable_poll_is_fenced() {
    let bus = WallClockDelayedBus::new();
    bus.with(|s| s.push_all_feedback());
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    // Short 70 ms lease timeout; motor 3 withholds enable reply for 140 ms so lease expires mid-enable.
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 70,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm)),
    );
    sup.control_tick(0.02).await;
    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });
    let stop_flag = Arc::new(AtomicBool::new(false));
    let sup_tick = sup.clone();
    let stop_tick = stop_flag.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        while !stop_tick.load(Ordering::Relaxed) {
            interval.tick().await;
            sup_tick.control_tick(0.02).await;
        }
    });

    let (mut ws, gen) = common::controller(addr).await;
    bus.with(|s| s.motors[2].enable_reply_delay = Duration::from_millis(140));
    let ep = sup.get_observation().await.epoch_id;
    let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    stop_flag.store(true, Ordering::Relaxed);

    assert_eq!(
        reply["type"], "command_reject",
        "lease expiring mid-enable must NOT arm: {reply}"
    );
    assert!(
        reply["code"] == "LEASE_LOST" || reply["code"] == "NO_LEASE",
        "expected LEASE_LOST rejection when lease expired during enable: {reply}"
    );
    assert!(
        !sup.inner().read().await.safety.state().is_operational(),
        "must not enter Armed after lease expired mid-enable"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_contended_stop_drains_cancelling_phase_and_allows_measured_hold() {
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), false).await;
    let (mut ws, gen) = common::controller(addr).await;
    // Motors 1..6 enable immediately; motor 7 drops enable so handshake stays pending.
    bus.with(|s| s.motors[6].drop_enable = true);
    let ep = sup.get_observation().await.epoch_id;
    common::send(
        &mut ws,
        json!({"type": "command", "request_id": 301, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
    )
    .await;

    let dl = Instant::now() + Duration::from_secs(2);
    while Instant::now() < dl {
        if sup.inner().read().await.torque_transition_pending() && bus.enable_tx_count() > 0 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
    }
    assert!(sup.inner().read().await.torque_transition_pending());
    assert!(bus.enable_tx_count() > 0, "expected partial enable TX before contended Stop");

    // Trigger Stop WHILE the driver mutex is contended -> must settle STOPPED immediately AND
    // retain `Cancelling` phase instead of dropping the reducer while Damiao still holds torque_hs.
    let tx_at_stop = {
        let mut inner = sup.inner().write().await;
        inner.with_contended_driver_for_test(|s| {
            s.authorized_stop();
        });
        assert!(
            inner.torque_transition_pending(),
            "contended Stop must keep Cancelling reducer until drv.abort_enable() runs"
        );
        bus.enable_tx_count()
    };
    let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "STOPPED", "{reply}");

    // Once contention clears, the next control tick drains Cancelling -> calls abort_enable()
    // with ZERO enable TX and clears Damiao's torque_hs so measured hold works cleanly.
    sup.control_tick(0.02).await;
    let tx_after = bus.enable_tx_count();
    stop_flag.store(true, Ordering::Relaxed);
    assert_eq!(tx_after, tx_at_stop, "Cancelling phase must emit ZERO enable TX");
    let inner = sup.inner().read().await;
    assert!(!inner.torque_transition_pending(), "Cancelling must clear once driver lock is free");
    assert!(
        !inner
            .driver
            .as_ref()
            .unwrap()
            .read()
            .unwrap()
            .torque_transition_pending(),
        "Damiao torque_hs must be cleared so subsequent measured holds are never rejected"
    );
    assert!(inner.is_torqued);
    assert_eq!(inner.enable_lifecycle(), EnableLifecycle::FailedHold);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_torque_off_cannot_clobber_active_protective_disable() {
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), false).await;
    let (mut ws, gen) = common::controller(addr).await;

    // Arm, then trigger a thermal protective disable while disable TX returns WouldBlock so
    // `active_disable_kind = Some(DisableProtective { Thermal })` and `disable_pending = true`
    // remain armed without a `torque_transition` (the exact Blocker 3 window).
    {
        let mut inner = sup.inner().write().await;
        let stamp = inner.current_admission_stamp();
        inner
            .check_admission(stamp)
            .and_then(armd::SupervisorInner::enable_torque)
            .unwrap();
        for _ in 0..4 {
            inner.tick_control(0.02);
        }
        bus.with(|s| s.fail_disable_tx = true);
        inner.apply_stop_policy(armd::StopPolicy::ThermalTrip);
        bus.with(|s| s.fail_disable_tx = false);
        assert!(
            inner.disable_pending,
            "failed ThermalTrip TX must leave protective disable_pending armed"
        );
    }
    let ep = sup.get_observation().await.epoch_id;
    let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": false})).await;
    stop_flag.store(true, Ordering::Relaxed);

    assert_eq!(
        reply["type"], "command_reject",
        "WS torque-off must not replace active protective disable: {reply}"
    );
    // Verify the active protective disable was NOT replaced or cleared.
    let inner = sup.inner().read().await;
    assert!(
        inner.disable_pending,
        "protective disable work must remain active and unreplaced"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_real_socket_deadline_cleans_up_enable_and_detaches_dispatched_disable() {
    // Subcase 1 (R2 Major 1 + Major 5): Real WebSocket `/api/v1/torque {"enable": true}` over
    // TCP with a 120 ms deadline override and paused ticker -> serialized JSON `command_reject`
    // has `code == "DEADLINE_EXPIRED"` AND Damiao's `torque_hs` is cancelled so holds succeed.
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| s.push_all_feedback());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        sup.set_legacy_torque_deadline_for_test(Some(Duration::from_millis(120)));
        sup.control_tick(0.02).await;

        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });

        let (mut ws, gen) = common::controller(addr).await;
        // Withhold motor 3 longer than our 120 ms WS deadline, and keep control loop paused.
        bus.with(|s| s.motors[2].enable_reply_delay = Duration::from_millis(450));
        let ep = sup.get_observation().await.epoch_id;
        let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(
            reply["code"], "DEADLINE_EXPIRED",
            "real WS socket must serialize DEADLINE_EXPIRED on stalled enable: {reply}"
        );
        let mut inner = sup.inner().write().await;
        assert!(!inner.torque_transition_pending(), "enable transition must be cleaned up");
        assert!(
            !inner
                .driver
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .torque_transition_pending(),
            "Damiao driver torque_hs must be cancelled on real WS deadline"
        );
        inner.tick_control(0.02);
    }

    // Subcase 2: Real WebSocket `/api/v1/torque {"enable": false}` over TCP with 120 ms deadline
    // during partially dispatched disable -> JSON `DEADLINE_EXPIRED`, disable stays Detached,
    // and resuming control ticks completes hardware torque-off.
    {
        let bus = WallClockDelayedBus::new();
        bus.with(|s| s.push_all_feedback());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let sup = Arc::new(
            ArmSupervisor::new(ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(dm)),
        );
        sup.set_legacy_torque_deadline_for_test(Some(Duration::from_millis(75)));
        {
            let mut inner = sup.inner().write().await;
            let stamp = inner.current_admission_stamp();
            inner
                .check_admission(stamp)
                .and_then(armd::SupervisorInner::enable_torque)
                .unwrap();
            for _ in 0..4 {
                inner.tick_control(0.02);
            }
        }
        let router = create_router(sup.clone(), OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            axum::serve(listener, router).await.unwrap();
        });

        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| s.motors[4].disable_reply_delay = Duration::from_millis(260));
        let ep = sup.get_observation().await.epoch_id;
        let sup_bg = sup.clone();
        let tick_helper = tokio::spawn(async move {
            for _ in 0..4 {
                tokio::time::sleep(Duration::from_millis(12)).await;
                sup_bg.control_tick(0.02).await;
            }
        });
        let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": false})).await;
        let _ = tick_helper.await;

        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(
            reply["code"], "DEADLINE_EXPIRED",
            "real WS socket must serialize DEADLINE_EXPIRED on delayed dispatched disable: {reply}"
        );
        assert!(
            sup.inner().read().await.torque_transition_pending(),
            "dispatched disable must remain active as Detached after real WS deadline"
        );
        tokio::time::sleep(Duration::from_millis(220)).await;
        for _ in 0..10 {
            sup.control_tick(0.02).await;
            if !sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let inner = sup.inner().read().await;
        assert!(!inner.torque_transition_pending());
        assert!(!inner.is_torqued);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_damiao_mid_rx_lease_expiry_emits_zero_post_expiry_enable_tx() {
    // R2 Blocker 1: If the lease expires DURING RX intake inside `poll_torque_budgeted` right
    // before Damiao's 50 ms retry `send_enable_masked`, the driver's pre-TX `enable_tx_deadline`
    // fence MUST abort with ZERO post-expiry `CMD_ENABLE` frames transmitted on the bus.
    let bus = WallClockDelayedBus::new();
    bus.with(|s| s.push_all_feedback());
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 80,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm)),
    );
    sup.control_tick(0.02).await;
    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    let (mut ws, gen) = common::controller(addr).await;
    // Capture the exact lease expiry instant from the live owner.
    let lease_expiry = {
        let inner = sup.inner().read().await;
        let lease = inner.active_lease.as_ref().expect("controller holds lease");
        lease.acquired_at + Duration::from_millis(80)
    };
    // Motor 3 drops enable so Damiao arms its 50 ms retry timer.
    bus.with(|s| {
        s.motors[2].drop_enable = true;
        s.enable_tx_instants.clear();
    });
    let ep = sup.get_observation().await.epoch_id;
    common::send(
        &mut ws,
        json!({"type": "command", "request_id": 401, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
    )
    .await;

    // Wait for WS admission to install the pending transition and dispatch the first enable pass.
    let wait_dl = Instant::now() + Duration::from_secs(1);
    while Instant::now() < wait_dl {
        if sup.inner().read().await.torque_transition_pending() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    assert!(sup.inner().read().await.torque_transition_pending());
    // Step once immediately so the initial CMD_ENABLE wave goes out while lease is valid.
    sup.control_tick(0.02).await;
    assert!(bus.enable_tx_count() > 0, "initial CMD_ENABLE wave must have been sent");

    // Wait until ~55 ms after lease acquisition (past Damiao's 50 ms retry interval, still ~25 ms
    // before the 80 ms lease expiry), then inject a 40 ms one-shot delay inside `receive_timestamped`
    // so the lease expires MID-RX right before `send_enable_masked` runs inside `poll_torque_budgeted`!
    let target_tick_at = lease_expiry.checked_sub(Duration::from_millis(25)).unwrap_or_else(Instant::now);
    let now = Instant::now();
    if target_tick_at > now {
        tokio::time::sleep(target_tick_at - now).await;
    }
    bus.with(|s| {
        s.rx_delay_once = Duration::from_millis(40);
        s.push_all_feedback();
    });
    sup.control_tick(0.02).await;

    let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert!(
        reply["code"] == "LEASE_LOST" || reply["code"] == "NO_LEASE",
        "expected LEASE_LOST rejection when lease expired mid-RX: {reply}"
    );
    let post_expiry_tx = bus.with(|s| {
        s.enable_tx_instants
            .iter()
            .filter(|&&t| t >= lease_expiry)
            .count()
    });
    assert_eq!(
        post_expiry_tx, 0,
        "driver pre-TX authority fence must emit ZERO CMD_ENABLE frames at/after lease expiry"
    );
    assert!(
        !sup.inner().read().await.safety.state().is_operational(),
        "arm must not enter Armed after mid-RX lease expiry"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_feetech_synchronous_enable_preserves_calibration_and_safety_checks() {
    use armd::driver::FeetechArmDriver;
    use armd::protocol::feetech::{
        calculate_checksum, INST_PING, INST_SYNC_READ, INST_WRITE, REG_TORQUE_ENABLE,
    };
    use armd::transport::Transport;

    struct CheckingFeetechState {
        rx: VecDeque<u8>,
        torque_on_writes: usize,
    }

    #[derive(Clone)]
    struct CheckingFeetechTransport(Arc<Mutex<CheckingFeetechState>>);

    impl CheckingFeetechTransport {
        fn new() -> Self {
            Self(Arc::new(Mutex::new(CheckingFeetechState {
                rx: VecDeque::with_capacity(256),
                torque_on_writes: 0,
            })))
        }

        fn push_status(rx: &mut VecDeque<u8>, id: u8, params: &[u8]) {
            let len = (params.len() + 2) as u8;
            let cksum = calculate_checksum(id, len, 0, params);
            rx.push_back(0xFF);
            rx.push_back(0xFF);
            rx.push_back(id);
            rx.push_back(len);
            rx.push_back(0);
            for &b in params {
                rx.push_back(b);
            }
            rx.push_back(cksum);
        }
    }

    impl Transport for CheckingFeetechTransport {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            let mut s = self.0.lock().unwrap();
            if s.rx.is_empty() {
                return Err(TransportError::WouldBlock);
            }
            let n = buf.len().min(s.rx.len());
            for slot in buf.iter_mut().take(n) {
                *slot = s.rx.pop_front().unwrap();
            }
            Ok(n)
        }

        fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
            let mut s = self.0.lock().unwrap();
            if buf.len() >= 6 && buf[0] == 0xFF && buf[1] == 0xFF {
                let id = buf[2];
                let inst = buf[4];
                match inst {
                    INST_PING => Self::push_status(&mut s.rx, id, &[]),
                    INST_SYNC_READ => {
                        let data_len = buf.get(6).copied().unwrap_or(8) as usize;
                        let ids = &buf[7..buf.len().saturating_sub(1)];
                        for &sid in ids {
                            let mut payload = vec![0u8; data_len];
                            if data_len >= 2 {
                                payload[0] = 0x00;
                                payload[1] = 0x08;
                            }
                            if data_len >= 8 {
                                payload[6] = 74;
                                payload[7] = 25;
                            }
                            Self::push_status(&mut s.rx, sid, &payload);
                        }
                    }
                    INST_WRITE if buf.len() >= 8 && buf[5] == REG_TORQUE_ENABLE && buf[6] == 1 => {
                        s.torque_on_writes += 1;
                    }
                    _ => {}
                }
            }
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }

        fn clears_input_buffer(&self) -> bool {
            true
        }

        fn clear_input_buffer(&mut self) {
            self.0.lock().unwrap().rx.clear();
        }
    }

    // Serial Feetech driver stays on the validated synchronous enable path: uncalibrated arm MUST
    // be refused BEFORE any `REG_TORQUE_ENABLE = 1` packet is written, and `begin_torque_budgeted(true)`
    // returns `UNSUPPORTED`.
    let transport = CheckingFeetechTransport::new();
    let mut cfg = armd::protocol::feetech::FeetechConfig::default();
    cfg.calibrated = false;
    let mut driver =
        FeetechArmDriver::new(transport.clone(), vec![1, 2, 3, 4, 5, 6]).with_config(cfg);
    driver.read_joint_states().unwrap();
    assert!(matches!(
        driver.begin_torque_budgeted(true, 6),
        Err(armd::driver::ArmDriverError::ProtocolStatic(_))
    ));

    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::default()
        })
        .with_driver(Box::new(driver)),
    );
    sup.control_tick(0.02).await;

    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    let (mut ws, gen) = common::controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;

    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "CLEAR_FAULT_REJECTED", "{reply}");
    assert!(
        reply["message"].as_str().unwrap_or("").contains("uncalibrated"),
        "expected uncalibrated refusal detail: {reply}"
    );
    assert_eq!(
        transport.0.lock().unwrap().torque_on_writes,
        0,
        "uncalibrated Feetech arm must emit ZERO torque-on writes"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn rest_torque_endpoint_preserves_409_lease_rejection_without_write_guard() {
    use axum::body::Body;
    use axum::http::{Request, StatusCode};
    use tower::ServiceExt;

    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), true).await;
    let app = create_router(sup.clone(), OperatorAuth::disabled());

    // 1. Without a controller lease -> 409 LEASE_REQUIRED (no write guard hold).
    sup.inner().set_hold_history_enabled(true);
    let resp = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/api/v1/torque")
                .header("content-type", "application/json")
                .body(Body::from(r#"{"enable":true}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp.status(), StatusCode::CONFLICT);
    let body = axum::body::to_bytes(resp.into_body(), 4096).await.unwrap();
    let json_no_lease: serde_json::Value = serde_json::from_slice(&body).unwrap();
    assert_eq!(json_no_lease["code"], "LEASE_REQUIRED", "{json_no_lease}");

    // 2. With an active WS controller lease -> 409 LEASE_ACTIVE.
    let (_ws, _gen) = common::controller(addr).await;
    let resp2 = app
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/api/v1/torque")
                .header("content-type", "application/json")
                .body(Body::from(r#"{"enable":true}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp2.status(), StatusCode::CONFLICT);
    let body2 = axum::body::to_bytes(resp2.into_body(), 4096).await.unwrap();
    let json_with_lease: serde_json::Value = serde_json::from_slice(&body2).unwrap();
    assert_eq!(json_with_lease["code"], "LEASE_ACTIVE", "{json_with_lease}");
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);
    stop_flag.store(true, Ordering::Relaxed);
    assert!(
        holds.iter().all(|h| h.command != Some("/api/v1/torque")),
        "REST /api/v1/torque rejection must never enter the torque write guard"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_contended_estop_and_lease_revocation_never_stall_and_drain_cancelling() {
    // Subcase 1 (R3 B3): E-stop arriving while driver mutex is contended during pending enable
    // MUST NOT block on `d.lock()`, latches EStop immediately, retains `DetachedCancel`, and
    // drains `drv.abort_enable()` with ZERO enable TX once contention clears.
    {
        let bus = WallClockDelayedBus::new();
        let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), false).await;
        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| s.motors[6].drop_enable = true);
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "command", "request_id": 501, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
        )
        .await;
        let dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < dl {
            if sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let tx_at_estop = {
            let mut inner = sup.inner().write().await;
            let t0 = Instant::now();
            inner.with_contended_driver_for_test(|s| {
                assert!(s.request_and_consume_estop_for_test());
            });
            let elapsed = t0.elapsed();
            assert!(
                elapsed <= Duration::from_millis(5),
                "contended E-stop must not block on driver mutex (elapsed={elapsed:?})"
            );
            assert!(
                inner.torque_transition_pending(),
                "contended E-stop must retain DetachedCancel until drv.abort_enable() runs"
            );
            bus.enable_tx_count()
        };
        let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert_eq!(reply["code"], "ESTOP_ACTIVE", "{reply}");

        // Next tick drains DetachedCancel with ZERO enable TX and clears Damiao torque_hs.
        sup.control_tick(0.02).await;
        stop_flag.store(true, Ordering::Relaxed);
        assert_eq!(bus.enable_tx_count(), tx_at_estop);
        let inner = sup.inner().read().await;
        assert!(!inner.torque_transition_pending());
        assert!(!inner.driver.as_ref().unwrap().read().unwrap().torque_transition_pending());
    }

    // Subcase 2 (R3 M2): Lease revocation while driver mutex is contended during pending enable
    // MUST settle LEASE_LOST, retain DetachedCancel, and drain `drv.abort_enable()` on next tick.
    {
        let bus = WallClockDelayedBus::new();
        let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), false).await;
        let (mut ws, gen) = common::controller(addr).await;
        bus.with(|s| s.motors[6].drop_enable = true);
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "command", "request_id": 502, "generation": gen, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
        )
        .await;
        let dl = Instant::now() + Duration::from_secs(2);
        while Instant::now() < dl {
            if sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let tx_at_revoke = {
            let mut inner = sup.inner().write().await;
            inner.with_contended_driver_for_test(|s| {
                let (cid, g) = {
                    let lease = s.active_lease.as_ref().expect("active lease");
                    (lease.client_id.clone(), lease.generation)
                };
                let _ = s.release_lease(&cid, g);
            });
            assert!(
                inner.torque_transition_pending(),
                "contended lease revocation must retain DetachedCancel until drv.abort_enable() runs"
            );
            bus.enable_tx_count()
        };
        let reply = common::next_of(&mut ws, &["command_ack", "command_reject"]).await;
        assert_eq!(reply["type"], "command_reject", "{reply}");
        assert!(
            reply["code"] == "LEASE_LOST" || reply["code"] == "NO_LEASE",
            "{reply}"
        );
        sup.control_tick(0.02).await;
        stop_flag.store(true, Ordering::Relaxed);
        assert_eq!(bus.enable_tx_count(), tx_at_revoke);
        let inner = sup.inner().read().await;
        assert!(!inner.torque_transition_pending());
        assert!(!inner.driver.as_ref().unwrap().read().unwrap().torque_transition_pending());
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_disable_transport_error_preserves_clear_fault_rejected_wire_code() {
    // R2 Major 4 + R3 Major 9: Legacy WS torque-off transport failure must preserve exact
    // `CLEAR_FAULT_REJECTED` wire code + `"Torque disable transmission failed: "` prefix.
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, ticks) = spawn_legacy_rig(bus.clone(), true).await;
    let (mut ws, gen) = common::controller(addr).await;

    // Arm cleanly first and let K=3 rest-pose samples accumulate on live 50 Hz ticks.
    let ep = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let t0 = ticks.load(Ordering::Acquire);
    while ticks.load(Ordering::Acquire) < t0 + 4 {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    // Inject disable TX failure so CMD_DISABLE fails on transport.
    bus.with(|s| s.fail_disable_tx = true);
    let ep = sup.get_observation().await.epoch_id;
    let reply = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": false})).await;
    stop_flag.store(true, Ordering::Relaxed);

    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(
        reply["code"], "CLEAR_FAULT_REJECTED",
        "legacy WS disable transport failure must map to CLEAR_FAULT_REJECTED: {reply}"
    );
    let msg = reply["message"].as_str().unwrap_or("");
    assert!(
        msg.starts_with("Torque disable transmission failed: "),
        "disable transport failure reply must preserve exact base prefix: {reply}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_preflight_rejection_still_tags_torque_direction() {
    // R2 Minor 2: A syntactically valid `{"enable": false}` rejected in preflight (stale epoch)
    // must still tag its `/api/v1/torque` guard hold with `direction == Some("off")`.
    let bus = WallClockDelayedBus::new();
    let (addr, sup, stop_flag, _) = spawn_legacy_rig(bus.clone(), false).await;
    let (mut ws, gen) = common::controller(addr).await;

    let ep = sup.get_observation().await.epoch_id;
    sup.inner().set_hold_history_enabled(true);
    let reply = common::rpc(
        &mut ws,
        gen,
        ep.wrapping_add(999),
        "/api/v1/torque",
        json!({"enable": false}),
    )
    .await;
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);
    stop_flag.store(true, Ordering::Relaxed);

    assert_eq!(reply["type"], "command_reject", "{reply}");
    assert_eq!(reply["code"], "EPOCH_MISMATCH", "{reply}");
    assert!(
        holds
            .iter()
            .any(|h| h.command == Some("/api/v1/torque") && h.direction == Some("off")),
        "preflight-rejected valid torque-off must still record direction='off': {holds:?}"
    );
}
