//! SG2002-like multi-tick rest-pose torque disable tests.
//!
//! Models a 7-motor Damiao B601-DM arm over `mcp251xfd` (~0.5 ms per CAN frame TX,
//! 1–3 ms reply latency, 1 ms per-tick disable step window) and verifies:
//! 1. Multi-tick 7-motor rest disable completes on both Legacy WS and RT Executor with a single
//!    client success, holds preserved on still-enabled joints each tick, all motors `Disabled`,
//!    `is_torqued = false`, watchdog quiet after lease drop, and 0 shadow divergences.
//! 2. Mid-sequence E-stop and Stop keep remaining enabled joints receiving holds and report a
//!    single partial-disable fault naming exact `applied` / `uncertain` / `not attempted` joints.
//! 3. Mid-sequence motion beyond rest tolerance on a still-enabled arm joint aborts remaining
//!    disables, keeps remaining joints held, and reports the partial disable cleanly.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::driver::{ArmDriver, DamiaoArmDriver, EnableState};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    BootId, Capacities, ClientId, Outcome, Reply, Submit, TransitionOp,
};
use armd::rt_core::ops::{CoreOps, ReleaseFixture};
use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};
use armd::safety::StopPolicy;
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
};
use armd::{ArmConfig, ArmSupervisor, SafetyState, SupervisorInner, TorqueIntent};

const BOOT: BootId = BootId(0xB601_0001);
const RELEASE_PERIOD: Duration = Duration::from_millis(20);
const SG2002_TX_COST: Duration = Duration::from_micros(500);

#[derive(Clone, Copy)]
struct SgMotor {
    id: u16,
    q: f32,
    v: f32,
    temp_c: u8,
    enabled: bool,
    reply_delay: Duration,
    limits: DamiaoLimits,
}

impl SgMotor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = u8::from(self.enabled);
        let data = [
            (state << 4) | (self.id as u8 & 0x0F),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0F) << 4) | ((t >> 8) & 0x0F)) as u8,
            t as u8,
            self.temp_c,
            self.temp_c,
        ];
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

#[derive(Clone, Copy, Debug)]
struct TxRecord {
    frame: CanFrame,
}

struct Sg2002BusState {
    clock: Arc<MockClock>,
    motors: [SgMotor; 7],
    ctrl_modes: [u32; 7],
    tx_step: Duration,
    /// Optional per-tick limit on `CMD_DISABLE` frames (models 1 ms disable burst window at
    /// 0.5 ms/frame = 2 disable frames/tick while leaving the rest of the 20 ms tick for holds).
    max_disables_per_tick: Option<usize>,
    disables_this_tick: usize,
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    delayed: Vec<(Instant, CanFrame)>,
    tx_log: Vec<TxRecord>,
}

impl Sg2002BusState {
    fn push_all_feedback_now(&mut self) {
        self.flush_due_delayed();
        let now = self.clock.now();
        for m in &self.motors {
            self.rx.push_back((m.feedback_frame(), Some(CanRxTime::Monotonic(now))));
        }
    }

    fn flush_due_delayed(&mut self) {
        let now = self.clock.now();
        let mut due = Vec::new();
        let mut i = 0;
        while i < self.delayed.len() {
            if self.delayed[i].0 <= now {
                due.push(self.delayed.swap_remove(i));
            } else {
                i += 1;
            }
        }
        due.sort_by_key(|(ready_at, _)| *ready_at);
        for (ready_at, frame) in due {
            self.rx.push_back((frame, Some(CanRxTime::Monotonic(ready_at))));
        }
    }
}

#[derive(Clone)]
struct Sg2002Bus(Arc<Mutex<Sg2002BusState>>);

impl Sg2002Bus {
    fn new(clock: Arc<MockClock>, tx_step: Duration) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        // Realistic SG2002 reply latency: 1, 2, or 3 ms across motors 1..=7.
        let motors = std::array::from_fn(|i| SgMotor {
            id: (i + 1) as u16,
            q: 0.0,
            v: 0.0,
            temp_c: 32,
            enabled: false,
            reply_delay: Duration::from_millis(1 + (i as u64 % 3)),
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(Sg2002BusState {
            clock,
            motors,
            ctrl_modes: [damiao::CTRL_MODE_POS_VEL; 7],
            tx_step,
            max_disables_per_tick: None,
            disables_this_tick: 0,
            rx: VecDeque::with_capacity(512),
            delayed: Vec::new(),
            tx_log: Vec::new(),
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut Sg2002BusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn begin_tick(&self) {
        self.with(|s| s.disables_this_tick = 0);
    }

    fn clear_log(&self) {
        self.with(|s| s.tx_log.clear());
    }

    fn tx_log(&self) -> Vec<TxRecord> {
        self.with(|s| s.tx_log.clone())
    }
}

impl CanTransport for Sg2002Bus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone() as Arc<dyn DriverClock>)
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn rx_clock_sample(&self) -> Option<RxClockSample> {
        let s = self.0.lock().unwrap();
        Some(RxClockSample {
            rx_now: CanRxTime::Monotonic(s.clock.now()),
            realtime_minus_monotonic_ns: None,
        })
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();

        // Enforce optional per-tick disable burst limit (simulating 1 ms step window / TX FIFO
        // backpressure at 0.5 ms/frame = 2 disable frames/tick) BEFORE recording or advancing.
        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_DISABLE {
            if let Some(limit) = s.max_disables_per_tick {
                if s.disables_this_tick >= limit {
                    return Err(TransportError::WouldBlock);
                }
                s.disables_this_tick += 1;
            }
        }

        s.tx_log.push(TxRecord { frame: *frame });
        if !s.tx_step.is_zero() {
            s.clock.advance(s.tx_step);
        }
        let now = s.clock.now();

        // Register write replies (CTRL_MODE switching on J7 PosForce / MIT).
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let raw = u32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]);
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                if rid == damiao::RID_CTRL_MODE {
                    s.ctrl_modes[idx] = raw;
                }
                let reply = CanFrame::new_standard(0x10 + id, &frame.data).unwrap();
                s.rx.push_back((reply, Some(CanRxTime::Monotonic(now))));
            }
            return Ok(());
        }

        // Register read replies (for enable mode/range verification).
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_READ {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            if let Some((idx, m)) = s.motors.iter().enumerate().find(|(_, m)| m.id == id) {
                let ctrl_mode = s.ctrl_modes[idx];
                let val: Option<[u8; 4]> = match rid {
                    damiao::RID_CTRL_MODE => Some(ctrl_mode.to_le_bytes()),
                    damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                    damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                    damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                    damiao::RID_TIMEOUT => Some(1000u32.to_le_bytes()),
                    _ => None,
                };
                if let Some(v) = val {
                    let [lo, hi] = id.to_le_bytes();
                    let reply = CanFrame::new_standard(
                        0x10 + id,
                        &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                    )
                    .unwrap();
                    s.rx.push_back((reply, Some(CanRxTime::Monotonic(now))));
                }
            }
            return Ok(());
        }

        // 0xCC refresh/status poll.
        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(m) = s.motors.iter().find(|m| m.id == id).copied() {
                let ready_at = now + m.reply_delay;
                s.delayed.push((ready_at, m.feedback_frame()));
            }
            return Ok(());
        }

        // CMD_ENABLE
        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_ENABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                s.motors[idx].enabled = true;
                let m = s.motors[idx];
                s.rx.push_back((m.feedback_frame(), Some(CanRxTime::Monotonic(now))));
            }
            return Ok(());
        }

        // CMD_DISABLE (replies after realistic 1–3 ms delay with state=0).
        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_DISABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                s.motors[idx].enabled = false;
                let m = s.motors[idx];
                let ready_at = now + m.reply_delay;
                s.delayed.push((ready_at, m.feedback_frame()));
            }
            return Ok(());
        }

        // Motion/hold frames (MIT 0x001..0x007, PosVel 0x101..0x107, PosForce 0x307):
        // reply after realistic 1–3 ms latency.
        if !frame.is_extended && frame.dlc == 8 {
            if let Some(idx) = s.motors.iter().position(|m| {
                frame.id == u32::from(m.id)
                    || frame.id == 0x100 + u32::from(m.id)
                    || frame.id == 0x300 + u32::from(m.id)
            }) {
                let m = s.motors[idx];
                let ready_at = now + m.reply_delay;
                s.delayed.push((ready_at, m.feedback_frame()));
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_with_rx_time().map(|(f, _)| f)
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.flush_due_delayed();
        s.rx.pop_front().ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn build_dm_config(shadow_enabled: bool, lease_timeout_ms: u64) -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.torque_shadow_enabled = shadow_enabled;
    cfg.lease_timeout_ms = lease_timeout_ms;
    // Align gripper mode with DamiaoArmDriver::new (PosVel across all 7 joints).
    cfg.damiao_gripper = None;
    cfg.resting_pose = Some(armd::model::RestingPosePolicy {
        positions: Some(vec![0.0; 7]),
        ..armd::model::RestingPosePolicy::rebot_dm()
    });
    cfg
}

fn build_dm_driver(bus: Sg2002Bus, cfg: &ArmConfig) -> DamiaoArmDriver<Sg2002Bus> {
    let limits: Vec<DamiaoLimits> = cfg
        .damiao_ranges
        .as_ref()
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let mut drv = DamiaoArmDriver::new(bus, (1..=7).collect())
        .with_joint_limits(limits)
        .unwrap();
    drv.check_mapping_ranges();
    drv
}

fn is_hold_frame_for_motor(frame: &CanFrame, motor_id: u16) -> bool {
    let id = u32::from(motor_id);
    if frame.data == damiao::CMD_DISABLE || frame.data == damiao::CMD_ENABLE {
        return false;
    }
    frame.id == id || frame.id == 0x100 + id || frame.id == 0x300 + id
}

fn is_disable_frame_for_motor(frame: &CanFrame, motor_id: u16) -> bool {
    frame.id == u32::from(motor_id) && frame.data == damiao::CMD_DISABLE
}

/// Sets up an armed 7-motor Damiao arm on an SG2002-like bus (0.5 ms/frame TX, 1–3 ms replies,
/// 2 disable frames per tick = 1 ms disable burst window, 20 ms tick budget for holds).
async fn setup_legacy_ws_sg2002_arm(
    shadow_enabled: bool,
) -> (
    Arc<ArmSupervisor>,
    Sg2002Bus,
    Arc<MockClock>,
    armd::rt_core::requests::lease::ConnectionHandle,
    u64,
) {
    let clock = Arc::new(MockClock::new());
    let bus = Sg2002Bus::new(clock.clone(), Duration::ZERO);
    let cfg = build_dm_config(shadow_enabled, 500);
    let mut drv = build_dm_driver(bus.clone(), &cfg);
    bus.with(|s| s.push_all_feedback_now());
    drv.read_joint_states().unwrap();

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let conn = sup.try_claim_ws_slot().unwrap();
    sup.register_ws_connection_for_slot(conn.id()).await.unwrap();

    let gen = {
        let mut inner = sup.inner().write().await;
        // Run one initial tick after installing the driver so shadow observes baseline feedback
        // (matching daemon startup before torque enable).
        bus.with(|s| s.push_all_feedback_now());
        {
            let mut now_fn = || clock.now();
            let dl = clock.now() + RELEASE_PERIOD;
            inner.tick_control_with_clock(0.02, &mut now_fn, dl);
        }
        let gen = inner
            .acquire_lease(&format!("connection:{}", conn.id()))
            .unwrap();
        let stamp = inner.current_admission_stamp();
        inner
            .check_admission(stamp)
            .and_then(SupervisorInner::enable_torque)
            .unwrap();
        for _ in 0..2 {
            clock.advance(RELEASE_PERIOD);
            bus.with(|s| s.push_all_feedback_now());
            let mut now_fn = || clock.now();
            let dl = clock.now() + RELEASE_PERIOD;
            inner.tick_control_with_clock(0.02, &mut now_fn, dl);
        }
        gen
    };

    // Enable SG2002 0.5 ms/frame TX timing + 2 disable frames/tick (1 ms disable burst window).
    bus.with(|s| {
        s.tx_step = SG2002_TX_COST;
        s.max_disables_per_tick = Some(2);
        s.disables_this_tick = 0;
    });
    (sup, bus, clock, conn, gen)
}

/// Steps the legacy control loop until K=3 establishes and the first 2 disable frames (J1+J2)
/// are dispatched in a single 1 ms burst, returning the tick index at which J1+J2 went out.
async fn step_until_j1_j2_dispatched(
    sup: &Arc<ArmSupervisor>,
    bus: &Sg2002Bus,
    clock: &Arc<MockClock>,
) -> usize {
    for tick in 1..=10 {
        bus.begin_tick();
        clock.advance(RELEASE_PERIOD);
        let mask = {
            let mut inner = sup.inner().write().await;
            let mut now_fn = || clock.now();
            let dl = clock.now() + RELEASE_PERIOD;
            inner.tick_control_with_clock(0.02, &mut now_fn, dl);
            let m = inner
                .driver
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .disable_dispatched_mask();
            m
        };
        if mask != 0 {
            assert_eq!(
                mask, 0b0000_0011,
                "tick {tick}: expected J1+J2 (0x3) dispatched in 1ms step window, got {mask:#x}"
            );
            return tick;
        }
    }
    panic!("J1+J2 were not dispatched within 10 ticks");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sg2002_multitick_7motor_rest_disable_succeeds_and_shadow_zero_divergences() {
    // Part A: Legacy WS path with 0.5 ms/frame TX, 1–3 ms replies, 1 ms disable step window
    // (2 disable frames/tick), normal 20 ms tick budget for holds, and torque shadow ENABLED.
    let (sup, bus, clock, conn, gen) = setup_legacy_ws_sg2002_arm(true).await;
    let conn_id = conn.id();

    let (ep, stamp) = {
        let inner = sup.inner().read().await;
        (inner.safety.epoch_id(), inner.current_admission_stamp())
    };
    bus.clear_log();

    let sup_ws = sup.clone();
    let ws_task = tokio::spawn(async move {
        sup_ws
            .torque_ws_with_deadline(
                conn,
                gen,
                Some(ep),
                true,
                101,
                Ok(false),
                OwnerAuthPolicy::None,
                OperatorGrant::unenforced(),
                stamp,
                Duration::from_secs(5),
            )
            .await
            .unwrap()
    });

    for _ in 0..200 {
        if sup.inner().read().await.torque_transition_pending() {
            break;
        }
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    assert!(
        sup.inner().read().await.torque_transition_pending(),
        "torque(false) must enter pending multi-tick disable transition"
    );

    let mut ticks_used = 0usize;
    let mut first_disable_tick: [Option<usize>; 7] = [None; 7];
    let mut disable_order: Vec<usize> = Vec::new();
    let mut per_tick_first_disables: Vec<(usize, Vec<usize>)> = Vec::new();
    let mut hold_checks_during_partial = 0usize;

    for tick in 1..=40 {
        bus.begin_tick();
        clock.advance(RELEASE_PERIOD);
        let log_before = bus.tx_log().len();

        let (still_pending, drv_enable_state, dispatched_mask) = {
            let mut inner = sup.inner().write().await;
            let mut now_fn = || clock.now();
            let dl = clock.now() + RELEASE_PERIOD;
            inner.tick_control_with_clock(0.02, &mut now_fn, dl);
            let drv_enable_state = inner
                .driver
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .enable_state();
            let dispatched_mask = inner
                .driver
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .disable_dispatched_mask();
            (inner.torque_transition_pending(), drv_enable_state, dispatched_mask)
        };

        let tx_now = bus.tx_log();
        let slice = &tx_now[log_before..];
        let mut newly_disabled = Vec::new();
        for rec in slice {
            for j in 0..7 {
                if is_disable_frame_for_motor(&rec.frame, (j + 1) as u16)
                    && first_disable_tick[j].is_none()
                {
                    first_disable_tick[j] = Some(tick);
                    disable_order.push(j + 1);
                    newly_disabled.push(j + 1);
                }
            }
        }
        if !newly_disabled.is_empty() {
            per_tick_first_disables.push((tick, newly_disabled));
        }

        // While initial dispatch is partially complete (0 < dispatched_mask < 0x7F),
        // driver must NOT report `Disabled` yet and EVERY still-enabled joint MUST keep receiving holds.
        if dispatched_mask != 0 && dispatched_mask != 0x7F {
            assert_ne!(
                drv_enable_state,
                EnableState::Disabled,
                "tick {tick}: driver enable_state must not be Disabled while only {dispatched_mask:#x} dispatched"
            );
            for j in 0..7 {
                let held = slice
                    .iter()
                    .any(|r| is_hold_frame_for_motor(&r.frame, (j + 1) as u16));
                if (dispatched_mask & (1u64 << j)) != 0 {
                    assert!(
                        !held,
                        "tick {tick}: already-disabled J{} must not receive hold frames",
                        j + 1
                    );
                } else {
                    assert!(
                        held,
                        "tick {tick}: still-enabled J{} must keep receiving hold frames while disable spans ticks (dispatched={dispatched_mask:#x})",
                        j + 1
                    );
                }
            }
            hold_checks_during_partial += 1;
        }

        ticks_used = tick;
        if !still_pending {
            break;
        }
    }

    let ws_outcome = tokio::time::timeout(Duration::from_secs(2), ws_task)
        .await
        .expect("WS torque(false) must settle once transition completes")
        .expect("WS task must not panic");

    assert!(
        matches!(ws_outcome, TorqueWsOutcome::Accepted { is_torqued: false, .. }),
        "expected single Accepted(is_torqued=false), got {ws_outcome:?}"
    );
    assert_eq!(
        per_tick_first_disables.len(),
        4,
        "expected 7 motors dispatched across 4 ticks (2+2+2+1), got {per_tick_first_disables:?}"
    );
    assert_eq!(
        hold_checks_during_partial, 3,
        "expected holds verified on all 3 partial-dispatch ticks (after J1..2, J1..4, J1..6)"
    );
    assert_eq!(disable_order, vec![1, 2, 3, 4, 5, 6, 7]);
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));

    // Verify final supervisor/driver state and shadow diagnostics (0 divergences!).
    let (final_is_torqued, final_enable_state, final_disarmed_confirmed, final_safety, shadow_diag) = {
        let mut inner = sup.inner().write().await;
        inner.test_flush_and_drain_torque_shadow();
        let drv = inner.driver.as_ref().unwrap().read().unwrap();
        let es = drv.enable_state();
        let disarmed_confirmed = drv.is_hardware_disarmed_confirmed();
        drop(drv);
        let diag = inner.torque_shadow_diagnostics();
        (inner.is_torqued, es, disarmed_confirmed, inner.safety.state().clone(), diag)
    };
    assert!(!final_is_torqued, "supervisor.is_torqued must be false");
    assert_eq!(final_enable_state, EnableState::Disabled);
    assert!(
        final_disarmed_confirmed,
        "multi-tick ordinary rest disable must satisfy causal per-axis is_hardware_disarmed_confirmed()"
    );
    assert_eq!(final_safety, SafetyState::Idle);
    assert_eq!(
        shadow_diag.unexplained_divergences_total, 0,
        "torque shadow must record 0 unexplained divergences on multi-tick rest disable: last_state={:?} taxonomy={:?} gaps={:?} recent={:?}",
        shadow_diag.last_state,
        shadow_diag.taxonomy,
        shadow_diag.coverage_gaps_by_kind,
        shadow_diag.recent_divergences
    );
    assert_eq!(shadow_diag.taxonomy.invariant_violation, 0);
    assert!(shadow_diag.comparisons_total > 0);

    // Drop lease and advance well past lease_timeout_ms (500 ms): watchdog MUST stay quiet!
    let safety_after_lease_drop = {
        let mut inner = sup.inner().write().await;
        let _ = inner.release_lease(&format!("connection:{conn_id}"), gen);
        for _ in 0..10 {
            clock.advance(Duration::from_millis(100));
            let mut now_fn = || clock.now();
            let dl = clock.now() + RELEASE_PERIOD;
            inner.tick_control_with_clock(0.02, &mut now_fn, dl);
        }
        inner.safety.state().clone()
    };
    assert_eq!(
        safety_after_lease_drop,
        SafetyState::Idle,
        "watchdog must not latch WatchdogTimeout after lease drop on disarmed arm"
    );

    println!(
        "[SG2002-MULTITICK-LEGACY] ticks_used={} initial_dispatch_ticks={:?} disable_order={:?} partial_hold_ticks={} final_is_torqued={} final_enable_state={:?} final_safety={:?} after_lease_drop={:?} shadow_comparisons={} shadow_divergences={}",
        ticks_used,
        per_tick_first_disables,
        disable_order,
        hold_checks_during_partial,
        final_is_torqued,
        final_enable_state,
        final_safety,
        safety_after_lease_drop,
        shadow_diag.comparisons_total,
        shadow_diag.unexplained_divergences_total,
    );

    // Part B: RT Executor / Broker path driven end-to-end via `ReleaseFixture::step_with_clock`
    // with SG2002 0.5 ms/frame TX, 1–3 ms replies, 1 ms disable burst window (2 disables/tick),
    // normal 20 ms tick budget for holds, torque shadow ENABLED, and verified single Broker
    // `Reply::Terminal { outcome: Outcome::Completed }`.
    {
        let rt_clock = Arc::new(MockClock::new());
        let rt_bus = Sg2002Bus::new(rt_clock.clone(), Duration::ZERO);
        let rt_cfg = build_dm_config(true, 500);
        let mut rt_drv = build_dm_driver(rt_bus.clone(), &rt_cfg);
        rt_bus.with(|s| s.push_all_feedback_now());
        rt_drv.read_joint_states().unwrap();
        let mut inner = SupervisorInner::new(rt_cfg).with_driver(Box::new(rt_drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
            inner,
            BOOT,
            Capacities {
                commands: 8,
                plans: 1,
                replies: 16,
            },
        );
        // Baseline feedback tick so shadow sees fresh feedback before TorqueOn.
        rt_bus.with(|s| s.push_all_feedback_now());
        let _ = fx.step_with_clock(|| rt_clock.now());
        // Arm the arm.
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step_with_clock(|| rt_clock.now()).command.unwrap().1.is_ok());
        for _ in 0..15 {
            rt_clock.advance(RELEASE_PERIOD);
            rt_bus.with(|s| s.push_all_feedback_now());
            let r = fx.step_with_clock(|| rt_clock.now());
            if let Some((_, Outcome::Completed)) = r.transition_ended {
                break;
            }
        }
        assert!(fx.core.is_torqued);
        // Drain TorqueOn replies before submitting TorqueOff.
        b.poll(|_, _| true);

        // Switch bus to SG2002 0.5 ms/frame + 2 disable frames/tick (1 ms disable burst window)
        // while leaving the normal 20 ms tick budget for holds on remaining joints.
        rt_bus.with(|s| {
            s.tx_step = SG2002_TX_COST;
            s.max_disables_per_tick = Some(2);
            s.disables_this_tick = 0;
        });
        rt_bus.clear_log();

        let auth2 = fx.core.authority(&fx.ctx(fx.now()));
        let req_id = b
            .submit(
                ClientId(1),
                2,
                auth2,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();

        let mut rt_ended = None;
        let mut rt_ticks = 0usize;
        let mut rt_order = Vec::new();
        let mut rt_per_tick: Vec<(usize, Vec<usize>)> = Vec::new();
        let mut rt_seen = [false; 7];
        let mut rt_partial_hold_ticks = 0usize;
        let mut prev_dispatched_mask = 0u64;

        for tick in 1..=40 {
            rt_bus.begin_tick();
            rt_clock.advance(RELEASE_PERIOD);
            let log_before = rt_bus.tx_log().len();

            let rep = fx.step_with_clock(|| rt_clock.now());
            if let Some((_, o)) = rep.transition_ended {
                rt_ended = Some(o);
            }

            let (drv_enable_state, dispatched_mask) = {
                let drv = fx.core.driver.as_ref().unwrap().read().unwrap();
                (drv.enable_state(), drv.disable_dispatched_mask())
            };
            let tx_now = rt_bus.tx_log();
            let slice = &tx_now[log_before..];
            let mut newly = Vec::new();
            for rec in slice {
                for j in 0..7 {
                    if is_disable_frame_for_motor(&rec.frame, (j + 1) as u16) && !rt_seen[j] {
                        rt_seen[j] = true;
                        rt_order.push(j + 1);
                        newly.push(j + 1);
                    }
                }
            }
            if !newly.is_empty() {
                rt_per_tick.push((tick, newly));
            }
            if dispatched_mask != 0 && dispatched_mask != 0x7F {
                assert_ne!(
                    drv_enable_state,
                    EnableState::Disabled,
                    "RT tick {tick}: enable_state must not be Disabled while partial"
                );
                for j in 0..7 {
                    let mid = (j + 1) as u16;
                    if (prev_dispatched_mask & (1u64 << j)) != 0 {
                        // Already disabled before this release -> ZERO hold frames in this release.
                        assert!(
                            !slice.iter().any(|r| is_hold_frame_for_motor(&r.frame, mid)),
                            "RT tick {tick}: already-disabled J{} must not get holds",
                            j + 1
                        );
                    } else if (dispatched_mask & (1u64 << j)) != 0 {
                        // Newly disabled during this release (e.g. on sub-release wake after Step 5)
                        // -> ZERO hold frames AFTER its CMD_DISABLE frame.
                        if let Some(d_pos) =
                            slice.iter().position(|r| is_disable_frame_for_motor(&r.frame, mid))
                        {
                            assert!(
                                !slice[d_pos + 1..]
                                    .iter()
                                    .any(|r| is_hold_frame_for_motor(&r.frame, mid)),
                                "RT tick {tick}: newly-disabled J{} must not get holds after CMD_DISABLE",
                                j + 1
                            );
                        }
                    } else {
                        assert!(
                            slice.iter().any(|r| is_hold_frame_for_motor(&r.frame, mid)),
                            "RT tick {tick}: still-enabled J{} must get holds",
                            j + 1
                        );
                    }
                }
                rt_partial_hold_ticks += 1;
            }
            prev_dispatched_mask = dispatched_mask;

            rt_ticks = tick;
            if rt_ended.is_some() {
                break;
            }
        }
        assert_eq!(rt_ended, Some(Outcome::Completed));
        assert_eq!(
            rt_per_tick.len(),
            4,
            "RT multi-tick disable must dispatch 7 motors across 4 ticks: {rt_per_tick:?}"
        );
        assert_eq!(rt_partial_hold_ticks, 3);
        assert_eq!(rt_order, vec![1, 2, 3, 4, 5, 6, 7]);
        assert!(!fx.core.is_torqued);
        assert_eq!(fx.core.torque_intent, TorqueIntent::Off);
        assert_eq!(
            fx.core.driver.as_ref().unwrap().read().unwrap().enable_state(),
            EnableState::Disabled
        );
        assert!(
            fx.core
                .driver
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .is_hardware_disarmed_confirmed(),
            "RT multi-tick ordinary rest disable must satisfy causal per-axis is_hardware_disarmed_confirmed()"
        );
        let mut replies = Vec::new();
        b.poll(|_, r: Reply| {
            replies.push(r);
            true
        });
        let terminals: Vec<_> = replies
            .iter()
            .filter(|r| matches!(r, Reply::Terminal { request, .. } if *request == req_id))
            .collect();
        assert_eq!(
            terminals.len(),
            1,
            "RT client must see exactly one terminal reply for TorqueOff: {replies:?}"
        );
        assert!(
            matches!(terminals[0], Reply::Terminal { outcome: Outcome::Completed, .. }),
            "RT terminal reply must be Outcome::Completed: {:?}",
            terminals[0]
        );
        fx.core.test_flush_and_drain_torque_shadow();
        let rt_diag = fx.core.torque_shadow_diagnostics();
        assert_eq!(
            rt_diag.unexplained_divergences_total, 0,
            "RT multi-tick disable must have 0 shadow divergences: {:?}",
            rt_diag.recent_divergences
        );
        println!(
            "[SG2002-MULTITICK-RT] ticks_used={} initial_dispatch_ticks={:?} disable_order={:?} partial_hold_ticks={} final_is_torqued={} final_enable_state={:?} broker_terminals={} shadow_comparisons={} shadow_divergences={}",
            rt_ticks,
            rt_per_tick,
            rt_order,
            rt_partial_hold_ticks,
            fx.core.is_torqued,
            fx.core.driver.as_ref().unwrap().read().unwrap().enable_state(),
            terminals.len(),
            rt_diag.comparisons_total,
            rt_diag.unexplained_divergences_total,
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sg2002_multitick_estop_mid_sequence_holds_remaining_and_reports_partial() {
    let (sup, bus, clock, conn, gen) = setup_legacy_ws_sg2002_arm(false).await;
    let (ep, stamp) = {
        let inner = sup.inner().read().await;
        (inner.safety.epoch_id(), inner.current_admission_stamp())
    };
    bus.clear_log();

    let sup_ws = sup.clone();
    let ws_task = tokio::spawn(async move {
        sup_ws
            .torque_ws_with_deadline(
                conn,
                gen,
                Some(ep),
                true,
                201,
                Ok(false),
                OwnerAuthPolicy::None,
                OperatorGrant::unenforced(),
                stamp,
                Duration::from_secs(5),
            )
            .await
            .unwrap()
    });
    for _ in 0..200 {
        if sup.inner().read().await.torque_transition_pending() {
            break;
        }
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(1)).await;
    }

    // Advance ticks until K=3 confirms and J1+J2 are dispatched in the 1 ms burst.
    let _t_partial = step_until_j1_j2_dispatched(&sup, &bus, &clock).await;

    // Latch E-stop mid-sequence before the next tick!
    {
        let mut inner = sup.inner().write().await;
        inner.apply_stop_policy(StopPolicy::EStopLatch);
    }

    // Next tick aborts remaining disables cleanly without dispatching J3..J7.
    bus.begin_tick();
    clock.advance(RELEASE_PERIOD);
    {
        let mut inner = sup.inner().write().await;
        let mut now_fn = || clock.now();
        let dl = clock.now() + RELEASE_PERIOD;
        inner.tick_control_with_clock(0.02, &mut now_fn, dl);
        assert!(!inner.torque_transition_pending());
        assert!(inner.is_torqued, "remaining joints stay held after partial disable abort");
    }

    let ws_outcome = tokio::time::timeout(Duration::from_secs(2), ws_task)
        .await
        .expect("WS task must settle with single partial-disable fault")
        .unwrap();
    let msg = match &ws_outcome {
        TorqueWsOutcome::Rejected { error, .. } => error.message.clone(),
        other => panic!("expected Rejected(partial disable fault), got {other:?}"),
    };
    assert!(msg.contains("applied [J1, J2]"), "missing applied list: {msg}");
    assert!(msg.contains("uncertain []"), "missing uncertain list: {msg}");
    assert!(
        msg.contains("not attempted [J3, J4, J5, J6, J7]"),
        "missing not-attempted list: {msg}"
    );

    // Zero CMD_DISABLE frames ever reached J3..J7.
    let log = bus.tx_log();
    for j in 3..=7u16 {
        assert!(
            !log.iter().any(|r| is_disable_frame_for_motor(&r.frame, j)),
            "J{j} must never receive CMD_DISABLE after mid-sequence E-stop"
        );
    }

    // Subsequent ticks MUST keep holding J3..J7 (and skip disabled J1..J2).
    bus.clear_log();
    for _ in 0..3 {
        bus.begin_tick();
        clock.advance(RELEASE_PERIOD);
        bus.with(|s| s.push_all_feedback_now());
        let mut inner = sup.inner().write().await;
        let mut now_fn = || clock.now();
        let dl = clock.now() + RELEASE_PERIOD;
        inner.tick_control_with_clock(0.02, &mut now_fn, dl);
    }
    let post_log = bus.tx_log();
    for j in 3..=7u16 {
        assert!(
            post_log.iter().any(|r| is_hold_frame_for_motor(&r.frame, j)),
            "J{j} must keep receiving holds after mid-sequence E-stop abort"
        );
    }
    for j in 1..=2u16 {
        assert!(
            !post_log.iter().any(|r| is_hold_frame_for_motor(&r.frame, j)),
            "already-disabled J{j} must not receive holds"
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sg2002_multitick_stop_mid_sequence_holds_remaining_and_reports_partial() {
    let (sup, bus, clock, conn, gen) = setup_legacy_ws_sg2002_arm(false).await;
    let (ep, stamp) = {
        let inner = sup.inner().read().await;
        (inner.safety.epoch_id(), inner.current_admission_stamp())
    };
    bus.clear_log();

    let sup_ws = sup.clone();
    let ws_task = tokio::spawn(async move {
        sup_ws
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
                Duration::from_secs(5),
            )
            .await
            .unwrap()
    });
    for _ in 0..200 {
        if sup.inner().read().await.torque_transition_pending() {
            break;
        }
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(1)).await;
    }

    let _t_partial = step_until_j1_j2_dispatched(&sup, &bus, &clock).await;

    // Client Stop (advances stop_gen) mid-sequence before the next tick!
    {
        let mut inner = sup.inner().write().await;
        let _ = inner.advance_stop_generation();
    }

    // Next tick aborts remaining disables cleanly without dispatching J3..J7.
    bus.begin_tick();
    clock.advance(RELEASE_PERIOD);
    {
        let mut inner = sup.inner().write().await;
        let mut now_fn = || clock.now();
        let dl = clock.now() + RELEASE_PERIOD;
        inner.tick_control_with_clock(0.02, &mut now_fn, dl);
        assert!(!inner.torque_transition_pending());
        assert!(inner.is_torqued);
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(_)),
            "partial disable abort on Stop must latch a fault reporting disabled joints"
        );
    }

    let ws_outcome = tokio::time::timeout(Duration::from_secs(2), ws_task)
        .await
        .expect("WS task must settle with single partial-disable fault")
        .unwrap();
    let msg = match &ws_outcome {
        TorqueWsOutcome::Rejected { error, .. } => error.message.clone(),
        other => panic!("expected Rejected(partial disable fault), got {other:?}"),
    };
    assert!(msg.contains("applied [J1, J2]"), "missing applied list: {msg}");
    assert!(msg.contains("uncertain []"), "missing uncertain list: {msg}");
    assert!(
        msg.contains("not attempted [J3, J4, J5, J6, J7]"),
        "missing not-attempted list: {msg}"
    );

    let log = bus.tx_log();
    for j in 3..=7u16 {
        assert!(!log.iter().any(|r| is_disable_frame_for_motor(&r.frame, j)));
    }

    // Subsequent ticks MUST keep holding J3..J7 (and skip disabled J1..J2).
    bus.clear_log();
    for _ in 0..3 {
        bus.begin_tick();
        clock.advance(RELEASE_PERIOD);
        bus.with(|s| s.push_all_feedback_now());
        let mut inner = sup.inner().write().await;
        let mut now_fn = || clock.now();
        let dl = clock.now() + RELEASE_PERIOD;
        inner.tick_control_with_clock(0.02, &mut now_fn, dl);
    }
    let post_log = bus.tx_log();
    for j in 3..=7u16 {
        assert!(
            post_log.iter().any(|r| is_hold_frame_for_motor(&r.frame, j)),
            "J{j} must keep receiving holds after mid-sequence Stop abort"
        );
    }
    for j in 1..=2u16 {
        assert!(!post_log.iter().any(|r| is_hold_frame_for_motor(&r.frame, j)));
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sg2002_multitick_motion_beyond_rest_mid_sequence_aborts_and_holds_remaining() {
    let (sup, bus, clock, conn, gen) = setup_legacy_ws_sg2002_arm(false).await;
    let (ep, stamp) = {
        let inner = sup.inner().read().await;
        (inner.safety.epoch_id(), inner.current_admission_stamp())
    };
    bus.clear_log();

    let sup_ws = sup.clone();
    let ws_task = tokio::spawn(async move {
        sup_ws
            .torque_ws_with_deadline(
                conn,
                gen,
                Some(ep),
                true,
                203,
                Ok(false),
                OwnerAuthPolicy::None,
                OperatorGrant::unenforced(),
                stamp,
                Duration::from_secs(5),
            )
            .await
            .unwrap()
    });
    for _ in 0..200 {
        if sup.inner().read().await.torque_transition_pending() {
            break;
        }
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(1)).await;
    }

    let _t_partial = step_until_j1_j2_dispatched(&sup, &bus, &clock).await;

    // Before the next tick, still-enabled arm joint J4 moves beyond 0.05 rad rest tolerance!
    bus.begin_tick();
    clock.advance(RELEASE_PERIOD);
    bus.with(|s| {
        s.motors[3].q = 0.25; // J4 moves 0.25 rad away from rest (0.0)
        s.push_all_feedback_now();
    });

    {
        let mut inner = sup.inner().write().await;
        let mut now_fn = || clock.now();
        let dl = clock.now() + RELEASE_PERIOD;
        inner.tick_control_with_clock(0.02, &mut now_fn, dl);
        assert!(!inner.torque_transition_pending());
        assert!(inner.is_torqued);
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(_)),
            "mid-sequence motion beyond rest must latch fault reporting partial disable"
        );
    }

    let ws_outcome = tokio::time::timeout(Duration::from_secs(2), ws_task)
        .await
        .expect("WS task must settle with single partial-disable fault")
        .unwrap();
    let msg = match &ws_outcome {
        TorqueWsOutcome::Rejected { error, .. } => error.message.clone(),
        other => panic!("expected Rejected(partial disable fault), got {other:?}"),
    };
    assert!(
        msg.contains("J4 moved beyond rest tolerance"),
        "expected J4 motion reason: {msg}"
    );
    assert!(msg.contains("applied [J1, J2]"), "missing applied list: {msg}");
    assert!(msg.contains("uncertain []"), "missing uncertain list: {msg}");
    assert!(
        msg.contains("not attempted [J3, J4, J5, J6, J7]"),
        "missing not-attempted list: {msg}"
    );

    // Zero CMD_DISABLE frames ever reached J3..J7.
    let log = bus.tx_log();
    for j in 3..=7u16 {
        assert!(!log.iter().any(|r| is_disable_frame_for_motor(&r.frame, j)));
    }

    // Subsequent ticks MUST keep holding J3..J7 (and skip disabled J1..J2).
    bus.clear_log();
    for _ in 0..3 {
        bus.begin_tick();
        clock.advance(RELEASE_PERIOD);
        bus.with(|s| s.push_all_feedback_now());
        let mut inner = sup.inner().write().await;
        let mut now_fn = || clock.now();
        let dl = clock.now() + RELEASE_PERIOD;
        inner.tick_control_with_clock(0.02, &mut now_fn, dl);
    }
    let post_log = bus.tx_log();
    for j in 3..=7u16 {
        assert!(
            post_log.iter().any(|r| is_hold_frame_for_motor(&r.frame, j)),
            "J{j} must keep receiving holds after mid-sequence motion-beyond-rest abort"
        );
    }
    for j in 1..=2u16 {
        assert!(!post_log.iter().any(|r| is_hold_frame_for_motor(&r.frame, j)));
    }
}
