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
        let now = Instant::now();
        let mut s = self.0.lock().unwrap();
        s.drain_matured(now);
        s.rx.pop_front().ok_or(TransportError::WouldBlock)
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
            let mut interval = tokio::time::interval(Duration::from_millis(10));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            while !stop_tick.load(Ordering::Relaxed) {
                interval.tick().await;
                sup_tick.control_tick(0.01).await;
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
    let torque_sample_holder = window_samples.iter().find_map(|s| {
        s.longest_hold
            .as_ref()
            .filter(|h| h.command == Some("/api/v1/torque"))
            .or_else(|| {
                s.waited_for
                    .as_ref()
                    .filter(|h| h.command == Some("/api/v1/torque"))
            })
    });
    if let Some(holder) = torque_sample_holder {
        let v = serde_json::to_value(holder).unwrap();
        assert_eq!(v["direction"], "on", "Holder JSON must serialize direction: {v}");
        assert!(v.get("tx_us").is_some(), "Holder JSON must include tx_us: {v}");
        assert!(v.get("rx_us").is_some(), "Holder JSON must include rx_us: {v}");
        assert_eq!(v["sleep_us"], 0, "Holder JSON must report sleep_us=0: {v}");
    }

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

    assert!(
        max_hold <= Duration::from_millis(2),
        "EVERY SupervisorLock guard hold during 35 ms enable must stay <= 2 ms (max_hold={max_hold:?}, holds={holds:?})"
    );
    assert!(
        max_tick_wait_us <= 2_000,
        "control tick lock wait during 35 ms enable must stay <= 2000 us (got {max_tick_wait_us} us)"
    );
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
    assert!(
        max_hold <= Duration::from_millis(2),
        "EVERY guard hold during 35 ms disable must stay <= 2 ms (got {max_hold:?})"
    );
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
        let max_hold = holds.iter().map(|h| h.hold()).max().unwrap_or(Duration::ZERO);
        assert!(
            max_hold <= Duration::from_millis(2),
            "RangeCheck recovery + enable must keep every guard hold <= 2 ms (got {max_hold:?})"
        );
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
        assert_eq!(reply["code"], "ENABLE_FAILED", "{reply}");
        assert_eq!(
            bus.enable_tx_count(),
            0,
            "mismatched range check must refuse with ZERO CMD_ENABLE frames"
        );
    }
}
