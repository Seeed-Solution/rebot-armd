//! SG2002-like FakeBus integration tests for pre-enable bounded `0xCC` FRESHEN.
//!
//! Models a real 7-motor B601-DM arm on SG2002 (`mcp251xfd` SPI-CAN, ~0.45–0.5 ms/frame TX,
//! 1–3 ms per-motor reply latency, J7 in `PosForce` CTRL_MODE 4, 50 Hz 20 ms releases) and
//! verifies:
//! 1. Enable after a long disabled idle (stale ages 100–200 ms) succeeds 50/50 seeded runs across
//!    both Legacy WS and RT Executor owners, with `0xCC` freshen preceding readback, zero-velocity
//!    `0x307` pre-enable hold, and `CMD_ENABLE`.
//! 2. A silent J7 (after stale idle or from boot under `NoOutputUntilFresh`) and a silent arm
//!    joint still fail closed within the bounded freshen window, naming the silent joint and
//!    emitting zero `0x307` / `CMD_ENABLE` frames.
//! 3. Repeating the dev20 soak cycle 30× (enable -> J1 motion -> stop -> park -> rest torque-off
//!    -> disabled idle with RefreshScheduler) completes with 0 faults.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::driver::{PosForceExt,
    gripper_drive_from_profile, ArmDriver, ArmDriverError, DamiaoArmDriver, EnableState,
};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    BootId, Capacities, ClientId, Outcome, Submit, TransitionOp,
};
use armd::rt_core::ops::{CoreOps, ReleaseFixture, VirtualClock};
use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
};
use armd::{ArmConfig, ArmSupervisor, SafetyState, SupervisorInner};

const BOOT: BootId = BootId(0xB601_EF01);
const RELEASE_PERIOD: Duration = Duration::from_millis(20);
const DT: f64 = 0.02;

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

#[derive(Clone, Copy)]
struct SgMotor {
    id: u16,
    q: f32,
    v: f32,
    temp_c: u8,
    enabled: bool,
    silent: bool,
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

struct SgBusState {
    clock: Arc<MockClock>,
    motors: [SgMotor; 7],
    ctrl_modes: [u32; 7],
    tx_step: Duration,
    /// When true (startup `check_mapping_ranges`), replies are immediate with 0 tx_step.
    immediate_init: bool,
    follow_commands: bool,
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    delayed: Vec<(Instant, CanFrame)>,
    inject_after_empty: VecDeque<CanFrame>,
    tx_log: Vec<CanFrame>,
}

impl SgBusState {
    fn push_all_feedback_now(&mut self) {
        self.flush_due_delayed();
        let now = self.clock.now();
        for m in &self.motors {
            if !m.silent {
                self.rx
                    .push_back((m.feedback_frame(), Some(CanRxTime::Monotonic(now))));
            }
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
            self.rx
                .push_back((frame, Some(CanRxTime::Monotonic(ready_at))));
        }
    }

    fn enqueue_reply(&mut self, motor_idx: usize, frame: CanFrame, now: Instant) {
        if self.motors[motor_idx].silent {
            return;
        }
        if self.immediate_init {
            self.rx
                .push_back((frame, Some(CanRxTime::Monotonic(now))));
        } else {
            let ready_at = now + self.motors[motor_idx].reply_delay;
            self.delayed.push((ready_at, frame));
        }
    }
}

#[derive(Clone)]
struct Sg2002Bus(Arc<Mutex<SgBusState>>);

impl Sg2002Bus {
    fn new(
        clock: Arc<MockClock>,
        tx_step: Duration,
        reply_delays_ms: [u64; 7],
        j7_already_mode_4: bool,
    ) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| SgMotor {
            id: (i + 1) as u16,
            q: if i == 6 { -0.14 } else { 0.0 },
            v: 0.0,
            temp_c: 31,
            enabled: false,
            silent: false,
            reply_delay: Duration::from_millis(reply_delays_ms[i]),
            limits: limits[i],
        });
        let mut ctrl_modes = [damiao::CTRL_MODE_POS_VEL; 7];
        if j7_already_mode_4 {
            ctrl_modes[6] = damiao::CTRL_MODE_POS_FORCE;
        }
        Self(Arc::new(Mutex::new(SgBusState {
            clock,
            motors,
            ctrl_modes,
            tx_step,
            immediate_init: true,
            follow_commands: true,
            rx: VecDeque::with_capacity(512),
            delayed: Vec::new(),
            inject_after_empty: VecDeque::new(),
            tx_log: Vec::new(),
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut SgBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn clear_log(&self) {
        self.with(|s| s.tx_log.clear());
    }

    fn tx_log(&self) -> Vec<CanFrame> {
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
        s.tx_log.push(*frame);
        if !s.immediate_init && !s.tx_step.is_zero() {
            s.clock.advance(s.tx_step);
        }
        let now = s.clock.now();

        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let raw = u32::from_le_bytes([
                frame.data[4],
                frame.data[5],
                frame.data[6],
                frame.data[7],
            ]);
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                if !s.motors[idx].silent {
                    if rid == damiao::RID_CTRL_MODE {
                        s.ctrl_modes[idx] = raw;
                    }
                    let reply = CanFrame::new_standard(0x10 + id, &frame.data).unwrap();
                    s.enqueue_reply(idx, reply, now);
                }
            }
            return Ok(());
        }

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
                    s.enqueue_reply(idx, reply, now);
                }
            }
            return Ok(());
        }

        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                let fb = s.motors[idx].feedback_frame();
                s.enqueue_reply(idx, fb, now);
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_ENABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                if !s.motors[idx].silent {
                    s.motors[idx].enabled = true;
                    let fb = s.motors[idx].feedback_frame();
                    s.enqueue_reply(idx, fb, now);
                }
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_DISABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                if !s.motors[idx].silent {
                    s.motors[idx].enabled = false;
                    s.motors[idx].v = 0.0;
                    let fb = s.motors[idx].feedback_frame();
                    s.enqueue_reply(idx, fb, now);
                }
            }
            return Ok(());
        }

        // PosVel (0x100 + id) or PosForce (0x300 + id) or MIT (id) motion/hold frames.
        if !frame.is_extended && frame.dlc == 8 {
            if let Some(idx) = s.motors.iter().position(|m| {
                frame.id == u32::from(m.id)
                    || frame.id == 0x100 + u32::from(m.id)
                    || frame.id == 0x300 + u32::from(m.id)
            }) {
                if s.follow_commands && s.motors[idx].enabled {
                    if frame.id == 0x100 + u32::from(s.motors[idx].id) {
                        let target_q = f32::from_le_bytes(frame.data[0..4].try_into().unwrap());
                        let delta = target_q - s.motors[idx].q;
                        let step = delta.clamp(-0.04, 0.04);
                        s.motors[idx].q += step;
                        s.motors[idx].v = if delta.abs() <= 1e-4 { 0.0 } else { step / DT as f32 };
                    } else if frame.id == 0x300 + u32::from(s.motors[idx].id) {
                        let target_q = f32::from_le_bytes(frame.data[0..4].try_into().unwrap());
                        let vel_raw = u16::from_le_bytes([frame.data[4], frame.data[5]]);
                        if vel_raw == 0 {
                            s.motors[idx].v = 0.0;
                        } else {
                            let max_step = (f32::from(vel_raw) / 100.0) * DT as f32;
                            let delta = target_q - s.motors[idx].q;
                            let step = delta.clamp(-max_step, max_step);
                            s.motors[idx].q += step;
                            s.motors[idx].v =
                                if delta.abs() <= 1e-4 { 0.0 } else { step / DT as f32 };
                        }
                    }
                }
                let fb = s.motors[idx].feedback_frame();
                s.enqueue_reply(idx, fb, now);
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
        if let Some(item) = s.rx.pop_front() {
            return Ok(item);
        }
        if !s.inject_after_empty.is_empty() {
            let now = s.clock.now();
            while let Some(f) = s.inject_after_empty.pop_front() {
                s.rx.push_back((f, Some(CanRxTime::Monotonic(now))));
            }
        }
        Err(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn build_b601_config() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.resting_pose = Some(armd::model::RestingPosePolicy {
        positions: Some(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.14]),
        ..armd::model::RestingPosePolicy::rebot_dm()
    });
    cfg
}

fn build_b601_driver(bus: Sg2002Bus, cfg: &ArmConfig) -> DamiaoArmDriver<Sg2002Bus> {
    let limits: Vec<DamiaoLimits> = cfg
        .damiao_ranges
        .as_ref()
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let grip = gripper_drive_from_profile(cfg).expect("B601-DM PosForce gripper drive");
    let tx_budget = cfg.damiao_tx_budget();
    let mut drv = DamiaoArmDriver::new(bus.clone(), (1..=7).collect())
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap()
        .with_tx_budget(tx_budget);
    // Establish initial empty-queue boundary + baseline feedback, then run startup range check.
    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback_now());
    drv.read_joint_states().unwrap();
    let rc = drv.check_mapping_ranges();
    assert_eq!(rc.status, armd::driver::RangeCheckStatus::Passed);
    bus.with(|s| s.immediate_init = false);
    drv
}

fn is_cc_for_joint(frame: &CanFrame, motor_id: u16) -> bool {
    frame.id == 0x7FF
        && frame.data[2] == 0xCC
        && u16::from_le_bytes([frame.data[0], frame.data[1]]) == motor_id
}

fn is_reg_read_ctrl_mode(frame: &CanFrame, motor_id: u16) -> bool {
    frame.id == 0x7FF
        && frame.data[2] == damiao::REG_READ
        && frame.data[3] == damiao::RID_CTRL_MODE
        && u16::from_le_bytes([frame.data[0], frame.data[1]]) == motor_id
}

fn assert_enable_freshen_wire_order(log: &[CanFrame], label: &str) {
    for motor_id in 1u16..=7 {
        assert!(
            log.iter().any(|f| is_cc_for_joint(f, motor_id)),
            "{label}: expected 0xCC freshen poll for motor {motor_id}"
        );
    }
    let last_cc_j7 = log
        .iter()
        .position(|f| is_cc_for_joint(f, 7))
        .expect("J7 0xCC freshen");
    let read_j7 = log
        .iter()
        .position(|f| is_reg_read_ctrl_mode(f, 7))
        .expect("J7 CTRL_MODE readback");
    let hold_j7 = log
        .iter()
        .position(|f| f.id == 0x307)
        .expect("J7 0x307 pre-enable hold");
    let enable_j7 = log
        .iter()
        .position(|f| f.id == 7 && f.data == damiao::CMD_ENABLE)
        .expect("J7 CMD_ENABLE");
    assert!(
        last_cc_j7 < read_j7 && read_j7 < hold_j7 && hold_j7 < enable_j7,
        "{label}: expected freshen({last_cc_j7}) < readback({read_j7}) < hold({hold_j7}) < enable({enable_j7})"
    );
    let hold_vel = u16::from_le_bytes([log[hold_j7].data[4], log[hold_j7].data[5]]);
    assert_eq!(
        hold_vel, 0,
        "{label}: first 0x307 pre-enable hold must be zero-velocity (vel_raw=0), got {hold_vel}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn enable_after_long_disabled_idle_succeeds_50_of_50_seeded_runs() {
    let mut legacy_pass = 0usize;
    let mut rt_pass = 0usize;
    let mut min_max_stale_ms = u64::MAX;
    let mut max_stale_ms = 0u64;
    let mut max_enable_ticks = 0usize;

    for seed in 0u64..50 {
        let mut rng = SeedRng::new(seed ^ 0xEF50_1234);
        let tx_step = Duration::from_micros(rng.range(400, 600));
        let reply_delays_ms: [u64; 7] = std::array::from_fn(|_| rng.range(1, 3));
        let already_mode_4 = (seed % 3) != 0;
        let idle_ticks = rng.range(45, 90) as usize;
        // Target stale age window 100–200 ms across seeds (plus natural RefreshScheduler stagger).
        let extra_stale_ms = rng.range(0, 55);
        let use_legacy = (seed % 2) == 0;

        let clock = Arc::new(MockClock::new());
        let bus = Sg2002Bus::new(clock.clone(), tx_step, reply_delays_ms, already_mode_4);
        let cfg = build_b601_config();
        let drv = build_b601_driver(bus.clone(), &cfg);

        if use_legacy {
            let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
            let conn = sup.try_claim_ws_slot().unwrap();
            sup.register_ws_connection_for_slot(conn.id()).await.unwrap();
            let gen = {
                let mut inner = sup.inner().write().await;
                inner
                    .acquire_lease(&format!("connection:{}", conn.id()))
                    .unwrap()
            };

            // Simulate disabled idle with RefreshScheduler running at 50 Hz.
            for _ in 0..idle_ticks {
                clock.advance(RELEASE_PERIOD);
                let mut inner = sup.inner().write().await;
                let mut now_fn = || clock.now();
                let dl = clock.now() + RELEASE_PERIOD;
                inner.tick_control_with_clock(DT, &mut now_fn, dl);
            }
            if extra_stale_ms > 0 {
                clock.advance(Duration::from_millis(extra_stale_ms));
            }

            let ages_before = {
                let inner = sup.inner().read().await;
                inner.current_observation().feedback_age_ms
            };
            let max_age_before = ages_before.iter().flatten().copied().max().unwrap_or(0);
            assert!(
                max_age_before >= 100,
                "seed={seed} (legacy): expected stale ages >= 100 ms before enable, got {ages_before:?}"
            );
            min_max_stale_ms = min_max_stale_ms.min(max_age_before);
            max_stale_ms = max_stale_ms.max(max_age_before);

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
                        seed + 1,
                        Ok(true),
                        OwnerAuthPolicy::None,
                        OperatorGrant::unenforced(),
                        stamp,
                        Duration::from_secs(5),
                    )
                    .await
                    .unwrap()
            });

            for _ in 0..200 {
                if sup.inner().read().await.torque_transition_pending() || ws_task.is_finished() {
                    break;
                }
                tokio::task::yield_now().await;
                tokio::time::sleep(Duration::from_millis(1)).await;
            }

            let mut ticks_used = 0usize;
            for tick in 1..=35 {
                if ws_task.is_finished() {
                    break;
                }
                ticks_used = tick;
                clock.advance(RELEASE_PERIOD);
                let done = {
                    let mut inner = sup.inner().write().await;
                    let mut now_fn = || clock.now();
                    let dl = clock.now() + RELEASE_PERIOD;
                    inner.tick_control_with_clock(DT, &mut now_fn, dl);
                    !inner.torque_transition_pending()
                };
                if done {
                    break;
                }
                tokio::task::yield_now().await;
            }

            let outcome = tokio::time::timeout(Duration::from_secs(2), ws_task)
                .await
                .unwrap_or_else(|_| panic!("seed={seed} (legacy): torque_ws timed out"))
                .expect("ws join");
            match outcome {
                TorqueWsOutcome::Accepted { is_torqued, .. } => {
                    assert!(is_torqued, "seed={seed} (legacy): expected is_torqued=true");
                }
                other => {
                    let st = sup.inner().read().await.safety.state().clone();
                    panic!("seed={seed} (legacy): enable failed (ages_before={ages_before:?}, state={st:?}): {other:?}");
                }
            }
            {
                let inner = sup.inner().read().await;
                assert!(inner.is_torqued);
                assert!(
                    inner.safety.state().is_operational(),
                    "seed={seed} (legacy): {:?}",
                    inner.safety.state()
                );
                let cap = inner
                    .driver
                    .as_ref()
                    .unwrap()
                    .read()
                    .unwrap()
                    .motor_current_cap()
                    .unwrap();
                assert!(cap.active, "seed={seed} (legacy): J7 cap inactive");
            }
            bus.with(|s| {
                for (i, m) in s.motors.iter().enumerate() {
                    assert!(m.enabled, "seed={seed} (legacy): motor {} not enabled", i + 1);
                }
                assert_eq!(s.ctrl_modes[6], damiao::CTRL_MODE_POS_FORCE);
            });
            assert_enable_freshen_wire_order(&bus.tx_log(), &format!("seed={seed} legacy"));
            max_enable_ticks = max_enable_ticks.max(ticks_used);
            legacy_pass += 1;
        } else {
            let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
            inner.lease_enabled = false;
            inner.rt_executor_owned = true;
            inner.register_live_connection(1).unwrap();
            let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
                inner,
                BOOT,
                Capacities {
                    commands: 8,
                    plans: 2,
                    replies: 16,
                },
                VirtualClock::from_mock_clock((*clock).clone()),
            );

            // Idle ticks at 50 Hz with RefreshScheduler running.
            for _ in 0..idle_ticks {
                let _ = fx.step();
            }
            if extra_stale_ms > 0 {
                clock.advance(Duration::from_millis(extra_stale_ms));
            }

            let ages_before = fx.core.current_observation().feedback_age_ms;
            let max_age_before = ages_before.iter().flatten().copied().max().unwrap_or(0);
            assert!(
                max_age_before >= 100,
                "seed={seed} (rt): expected stale ages >= 100 ms before enable, got {ages_before:?}"
            );
            min_max_stale_ms = min_max_stale_ms.min(max_age_before);
            max_stale_ms = max_stale_ms.max(max_age_before);

            bus.clear_log();
            let auth = fx.core.authority(&fx.ctx(fx.now()));
            broker
                .submit(
                    ClientId(1),
                    seed + 1,
                    auth,
                    fx.now() + Duration::from_secs(5),
                    Submit::Transition(TransitionOp::TorqueOn),
                )
                .unwrap();
            let commit_rep = fx.step();
            assert!(
                commit_rep.command.as_ref().is_some_and(|(_, r)| r.is_ok()),
                "seed={seed} (rt): TorqueOn commit failed: {commit_rep:?}"
            );

            let mut ended = None;
            let mut ticks_used = 0usize;
            for tick in 1..=35 {
                ticks_used = tick;
                let rep = fx.step();
                if let Some((_, o)) = rep.transition_ended {
                    ended = Some(o);
                    break;
                }
            }
            assert_eq!(
                ended,
                Some(Outcome::Completed),
                "seed={seed} (rt): enable failed (ages_before={ages_before:?}, state={:?})",
                fx.core.safety.state()
            );
            assert!(fx.core.is_torqued && fx.core.safety.state().is_operational());
            let cap = fx
                .core
                .driver
                .as_ref()
                .unwrap()
                .read()
                .unwrap()
                .motor_current_cap()
                .unwrap();
            assert!(cap.active, "seed={seed} (rt): J7 cap inactive");
            bus.with(|s| {
                for (i, m) in s.motors.iter().enumerate() {
                    assert!(m.enabled, "seed={seed} (rt): motor {} not enabled", i + 1);
                }
                assert_eq!(s.ctrl_modes[6], damiao::CTRL_MODE_POS_FORCE);
            });
            assert_enable_freshen_wire_order(&bus.tx_log(), &format!("seed={seed} rt"));
            max_enable_ticks = max_enable_ticks.max(ticks_used);
            rt_pass += 1;
        }
    }

    let total = legacy_pass + rt_pass;
    assert_eq!(total, 50);
    println!(
        "enable_after_long_disabled_idle: {total}/50 seeded runs PASSED (legacy={legacy_pass}, rt_executor={rt_pass}, max_stale_age_range_ms={min_max_stale_ms}..={max_stale_ms}, max_enable_ticks={max_enable_ticks})"
    );
}

#[test]
fn silent_j7_and_silent_arm_joint_fail_closed_in_freshen() {
    // 1. RT Executor: after disabled idle (stale ages >= 100 ms), mute J7 -> fails closed with
    //    ENABLE_FAILED naming joint 7, 0xCC attempted, zero 0x307, zero CMD_ENABLE.
    {
        let clock = Arc::new(MockClock::new());
        let bus = Sg2002Bus::new(
            clock.clone(),
            Duration::from_micros(500),
            [1, 2, 3, 1, 2, 3, 2],
            true,
        );
        let cfg = build_b601_config();
        let drv = build_b601_driver(bus.clone(), &cfg);
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.rt_executor_owned = true;
        inner.register_live_connection(1).unwrap();
        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities {
                commands: 8,
                plans: 2,
                replies: 16,
            },
            VirtualClock::from_mock_clock((*clock).clone()),
        );
        for _ in 0..40 {
            let _ = fx.step();
        }
        clock.advance(Duration::from_millis(40));
        bus.with(|s| s.motors[6].silent = true);
        bus.clear_log();

        let auth = fx.core.authority(&fx.ctx(fx.now()));
        broker
            .submit(
                ClientId(1),
                1,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());

        let mut outcome = None;
        for _ in 0..20 {
            let rep = fx.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(outcome, Some(Outcome::Failed("ENABLE_FAILED")));
        let fault = fx.core.safety.fault().expect("ENABLE_FAILED fault expected");
        assert!(
            fault.message.contains("joint 7"),
            "silent J7 failure must name joint 7: {}",
            fault.message
        );
        let log = bus.tx_log();
        assert!(log.iter().any(|f| is_cc_for_joint(f, 7)), "expected 0xCC poll for J7");
        assert!(!log.iter().any(|f| f.id == 0x307), "must emit zero 0x307 when J7 is silent");
        assert!(
            !log.iter().any(|f| f.data == damiao::CMD_ENABLE),
            "must emit zero CMD_ENABLE when J7 is silent"
        );
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
    }

    // 2. Boot under NoOutputUntilFresh with silent J7: read-only 0xCC allowed, zero 0x307,
    //    zero CMD_ENABLE, fails closed with EnableFreshenTimeout { joint: 6 }.
    {
        let clock = Arc::new(MockClock::new());
        let bus = Sg2002Bus::new(
            clock.clone(),
            Duration::from_micros(500),
            [1, 2, 3, 1, 2, 3, 2],
            true,
        );
        let cfg = build_b601_config();
        let mut drv = build_b601_driver(bus.clone(), &cfg);
        drv.reset_pos_force_boot_state();
        bus.with(|s| s.motors[6].silent = true);
        bus.clear_log();

        let err = drv.set_torque(true).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::EnableFreshenTimeout { joint: 6 }),
            "expected EnableFreshenTimeout {{ joint: 6 }}, got {err:?}"
        );
        let log = bus.tx_log();
        assert!(log.iter().any(|f| is_cc_for_joint(f, 7)));
        assert!(!log.iter().any(|f| f.id == 0x307));
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!drv.is_enabled());
    }

    // 3. Silent arm joint (J4) after stale idle fails closed naming joint 4 with zero CMD_ENABLE.
    {
        let clock = Arc::new(MockClock::new());
        let bus = Sg2002Bus::new(
            clock.clone(),
            Duration::from_micros(500),
            [1, 2, 3, 1, 2, 3, 2],
            true,
        );
        let cfg = build_b601_config();
        let mut drv = build_b601_driver(bus.clone(), &cfg);
        clock.advance(Duration::from_millis(140));
        bus.with(|s| s.motors[3].silent = true);
        bus.clear_log();

        let err = drv.set_torque(true).unwrap_err();
        assert!(
            matches!(err, ArmDriverError::EnableFreshenTimeout { joint: 3 }),
            "expected EnableFreshenTimeout {{ joint: 3 }}, got {err:?}"
        );
        assert!(err.to_string().contains("joint 4"), "{err}");
        let log = bus.tx_log();
        assert!(log.iter().any(|f| is_cc_for_joint(f, 4)));
        assert!(!log.iter().any(|f| f.id == 0x307));
        assert!(!log.iter().any(|f| f.data == damiao::CMD_ENABLE));
        assert!(!drv.is_enabled());
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dev20_soak_cycle_30x_enable_j1_motion_stop_park_rest_torque_off_idle_zero_faults() {
    let clock = Arc::new(MockClock::new());
    let bus = Sg2002Bus::new(
        clock.clone(),
        Duration::from_micros(500),
        [1, 2, 3, 1, 2, 3, 2],
        true,
    );
    let cfg = build_b601_config();
    let drv = build_b601_driver(bus.clone(), &cfg);

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let conn = sup.try_claim_ws_slot().unwrap();
    sup.register_ws_connection_for_slot(conn.id()).await.unwrap();
    let gen = {
        let mut inner = sup.inner().write().await;
        inner
            .acquire_lease(&format!("connection:{}", conn.id()))
            .unwrap()
    };

    let tick_once = |sup: &Arc<ArmSupervisor>, clock: &Arc<MockClock>| {
        let sup = sup.clone();
        let clock = clock.clone();
        async move {
            clock.advance(RELEASE_PERIOD);
            let mut inner = sup.inner().write().await;
            let mut now_fn = || clock.now();
            let dl = clock.now() + RELEASE_PERIOD;
            inner.tick_control_with_clock(DT, &mut now_fn, dl);
        }
    };

    // Initial disabled idle before cycle 1 (mirrors dev20 boot -> idle -> cycle 1).
    for _ in 0..50 {
        tick_once(&sup, &clock).await;
    }

    let mut max_idle_age_seen = 0u64;
    let mut min_j7_age_at_enable = u64::MAX;
    let mut max_j7_age_at_enable = 0u64;
    let mut req_id = 1000u64;

    for cycle in 1usize..=30 {
        // Record feedback ages right before enable.
        let ages_before = {
            let inner = sup.inner().read().await;
            assert!(
                matches!(inner.safety.state(), SafetyState::Idle),
                "cycle {cycle}: expected Idle before enable, got {:?}",
                inner.safety.state()
            );
            assert!(!inner.is_torqued, "cycle {cycle}: expected disarmed before enable");
            inner.current_observation().feedback_age_ms
        };
        if let Some(m) = ages_before.iter().flatten().copied().max() {
            max_idle_age_seen = max_idle_age_seen.max(m);
        }
        if let Some(j7_age) = ages_before[6] {
            min_j7_age_at_enable = min_j7_age_at_enable.min(j7_age);
            max_j7_age_at_enable = max_j7_age_at_enable.max(j7_age);
        }

        // 1. Enable (Legacy WS budgeted ticked enable).
        req_id += 1;
        let (ep, stamp) = {
            let inner = sup.inner().read().await;
            (inner.safety.epoch_id(), inner.current_admission_stamp())
        };
        let sup_ws = sup.clone();
        let c_en = conn;
        let en_task = tokio::spawn(async move {
            sup_ws
                .torque_ws_with_deadline(
                    c_en,
                    gen,
                    Some(ep),
                    true,
                    req_id,
                    Ok(true),
                    OwnerAuthPolicy::None,
                    OperatorGrant::unenforced(),
                    stamp,
                    Duration::from_secs(600),
                )
                .await
                .unwrap()
        });
        for _ in 0..200 {
            if sup.inner().read().await.torque_transition_pending() || en_task.is_finished() {
                break;
            }
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        for _ in 0..35 {
            if en_task.is_finished() {
                break;
            }
            tick_once(&sup, &clock).await;
            if !sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::task::yield_now().await;
        }
        match tokio::time::timeout(Duration::from_secs(2), en_task)
            .await
            .unwrap_or_else(|_| panic!("cycle {cycle}: enable timed out"))
            .expect("enable join")
        {
            TorqueWsOutcome::Accepted { is_torqued: true, .. } => {}
            other => {
                let st = sup.inner().read().await.safety.state().clone();
                panic!(
                    "cycle {cycle}: enable failed (ages_before={ages_before:?}, state={st:?}): {other:?}"
                );
            }
        }
        {
            let inner = sup.inner().read().await;
            assert!(
                inner.is_torqued && inner.safety.state().is_operational(),
                "cycle {cycle}: expected operational after enable, got {:?}",
                inner.safety.state()
            );
        }
        // Run one operational hold tick so post-enable operational TX refreshes all joints.
        tick_once(&sup, &clock).await;
        {
            let inner = sup.inner().read().await;
            let ages_op = inner.current_observation().feedback_age_ms;
            assert!(
                ages_op.iter().all(|a| a.is_some_and(|ms| ms <= 100)),
                "cycle {cycle}: expected fresh feedback on all joints once operational, got {ages_op:?}"
            );
        }

        // 2. J1 motion away from resting pose (+0.25 rad on odd cycles, -0.25 rad on even cycles).
        let j1_goal = if cycle % 2 == 1 { 0.25_f32 } else { -0.25_f32 };
        {
            let mut inner = sup.inner().write().await;
            let j7_hold = inner.commanded_positions[6];
            let target = vec![j1_goal, 0.0, 0.0, 0.0, 0.0, 0.0, j7_hold];
            let ep = inner.safety.epoch_id();
            let stamp = inner.current_admission_stamp();
            let admitted = inner
                .check_admission(stamp)
                .unwrap_or_else(|e| panic!("cycle {cycle}: J1 motion admission failed: {e:?}"));
            SupervisorInner::jog(admitted, target, Some(0.20), Some(ep))
                .unwrap_or_else(|e| panic!("cycle {cycle}: J1 motion target failed: {e:?}"));
        }
        for _ in 0..80 {
            tick_once(&sup, &clock).await;
            let j1_q = bus.with(|s| s.motors[0].q);
            if j1_q.abs() > 0.15 || sup.inner().read().await.active_trajectory.is_none() {
                break;
            }
        }
        let j1_after_move = bus.with(|s| s.motors[0].q);
        assert!(
            j1_after_move.abs() > 0.15,
            "cycle {cycle}: J1 should have moved away from 0, got {j1_after_move}"
        );

        // 3. Stop.
        {
            let mut inner = sup.inner().write().await;
            inner.authorized_stop();
        }
        for _ in 0..3 {
            tick_once(&sup, &clock).await;
        }
        {
            let inner = sup.inner().read().await;
            assert!(
                inner.is_torqued && inner.safety.state().is_operational(),
                "cycle {cycle}: stop should stay operational and torqued, got {:?}",
                inner.safety.state()
            );
        }

        // 4. Park back to resting pose (0.0 on J1–J6) and settle stationary.
        {
            let mut inner = sup.inner().write().await;
            let j7_hold = inner.commanded_positions[6];
            let rest = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, j7_hold];
            let ep = inner.safety.epoch_id();
            let stamp = inner.current_admission_stamp();
            let admitted = inner
                .check_admission(stamp)
                .unwrap_or_else(|e| panic!("cycle {cycle}: park admission failed: {e:?}"));
            SupervisorInner::jog(admitted, rest, Some(0.20), Some(ep))
                .unwrap_or_else(|e| panic!("cycle {cycle}: park target failed: {e:?}"));
        }
        for _ in 0..120 {
            tick_once(&sup, &clock).await;
            let (q, v) = bus.with(|s| (s.motors[0].q, s.motors[0].v));
            if sup.inner().read().await.active_trajectory.is_none()
                && q.abs() < 0.01
                && v.abs() <= 0.005
            {
                break;
            }
        }
        // Settle 4 extra hold ticks so K=3 rest-pose evidence is fresh at rest.
        for _ in 0..4 {
            tick_once(&sup, &clock).await;
        }
        let (j1_parked, j1_vel) = bus.with(|s| (s.motors[0].q, s.motors[0].v));
        assert!(
            j1_parked.abs() < 0.01 && j1_vel.abs() <= 0.005,
            "cycle {cycle}: expected parked & settled at 0, got q={j1_parked} v={j1_vel}"
        );

        // 5. Rest torque-off (through real K=3 rest-pose gate).
        {
            let inner = sup.inner().read().await;
            assert!(
                inner.safety.fault().is_none() && inner.safety.state().is_operational(),
                "cycle {cycle}: already faulted BEFORE rest torque-off! state={:?}, ages={:?}, q={:?}",
                inner.safety.state(),
                inner.current_observation().feedback_age_ms,
                inner.measured_positions
            );
        }
        req_id += 1;
        let (ep, stamp) = {
            let inner = sup.inner().read().await;
            (inner.safety.epoch_id(), inner.current_admission_stamp())
        };
        let sup_ws = sup.clone();
        let c_dis = conn;
        let dis_task = tokio::spawn(async move {
            sup_ws
                .torque_ws_with_deadline(
                    c_dis,
                    gen,
                    Some(ep),
                    true,
                    req_id,
                    Ok(false),
                    OwnerAuthPolicy::None,
                    OperatorGrant::unenforced(),
                    stamp,
                    Duration::from_secs(600),
                )
                .await
                .unwrap()
        });
        for _ in 0..200 {
            if sup.inner().read().await.torque_transition_pending() || dis_task.is_finished() {
                break;
            }
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        for _ in 0..50 {
            if dis_task.is_finished() {
                break;
            }
            tick_once(&sup, &clock).await;
            if !sup.inner().read().await.torque_transition_pending() {
                break;
            }
            tokio::task::yield_now().await;
        }
        match tokio::time::timeout(Duration::from_secs(2), dis_task)
            .await
            .unwrap_or_else(|_| panic!("cycle {cycle}: torque-off timed out"))
            .expect("disable join")
        {
            TorqueWsOutcome::Accepted { is_torqued: false, .. } => {}
            other => {
                let inner = sup.inner().read().await;
                let st = inner.safety.state().clone();
                let ages = inner.current_observation().feedback_age_ms;
                panic!("cycle {cycle}: rest torque-off failed (state={st:?}, ages={ages:?}): {other:?}");
            }
        }
        {
            let inner = sup.inner().read().await;
            assert!(!inner.is_torqued, "cycle {cycle}: expected disarmed after rest torque-off");
            assert!(
                matches!(inner.safety.state(), SafetyState::Idle),
                "cycle {cycle}: expected Idle after rest torque-off, got {:?}",
                inner.safety.state()
            );
            assert_eq!(
                inner
                    .driver
                    .as_ref()
                    .unwrap()
                    .read()
                    .unwrap()
                    .enable_state(),
                EnableState::Disabled
            );
            assert!(
                inner.safety.fault().is_none(),
                "cycle {cycle}: unexpected fault: {:?}",
                inner.safety.fault()
            );
        }
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));

        // 6. Idle with torque off: vary tick count across all 7 RefreshScheduler phases
        //    (including J7 right around 100–140 ms and arm joints 100–160 ms!).
        let idle_ticks = 42 + (cycle % 7) + (cycle % 3);
        for _ in 0..idle_ticks {
            tick_once(&sup, &clock).await;
        }
        // On every 5th cycle, add small sub-tick jitter (1–12 ms) to stress the exact 99–105 ms boundary.
        if cycle % 5 == 0 {
            clock.advance(Duration::from_millis(1 + (cycle as u64 % 12)));
        }
        {
            let inner = sup.inner().read().await;
            assert!(
                inner.safety.fault().is_none(),
                "cycle {cycle}: unexpected fault during idle: {:?}",
                inner.safety.fault()
            );
        }
    }

    println!(
        "dev20_soak_cycle_30x: 30/30 cycles PASSED, faults=0 (max_idle_age_ms={max_idle_age_seen}, j7_age_at_enable_ms={min_j7_age_at_enable}..={max_j7_age_at_enable})"
    );
}

/// Blocker 1 regression: a delayed idle-poll reply arriving AFTER `send_plain` starts (e.g.,
/// during pre-TX clock sampling / hook) but BEFORE `can.transmit` MUST NOT satisfy FRESHEN's
/// causal `rx_at >= tx_at` provenance check when J7 never replies to the FRESHEN poll itself.
#[test]
fn freshen_rejects_delayed_pre_tx_reply_arriving_before_can_transmit() {
    let clock = Arc::new(MockClock::new());
    let bus = Sg2002Bus::new(clock.clone(), Duration::from_micros(500), [1, 1, 1, 1, 1, 1, 1], true);
    let cfg = build_b601_config();
    let limits: Vec<DamiaoLimits> = cfg
        .damiao_ranges
        .as_ref()
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let grip = gripper_drive_from_profile(&cfg).expect("pos-force gripper");

    let injected = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let bus_hook = bus.clone();
    let clk_hook = clock.clone();
    let inj_hook = injected.clone();
    let mut drv = DamiaoArmDriver::new(bus.clone(), (1..=7).collect())
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(grip)
        .unwrap()
        .with_tx_budget(cfg.damiao_tx_budget())
        .with_before_final_tx_gate_hook(Arc::new(move |frame| {
            if is_cc_for_joint(frame, 7)
                && !inj_hook.swap(true, std::sync::atomic::Ordering::SeqCst)
            {
                // Inject a delayed idle-poll J7 feedback frame at the hook instant, then advance
                // the clock by 500 us before `send_inner` captures `tx_started_mark` and calls
                // `can.transmit(&frame)`.
                let pre_tx_ts = clk_hook.now();
                bus_hook.with(|s| {
                    let fb = s.motors[6].feedback_frame();
                    s.rx.push_back((fb, Some(CanRxTime::Monotonic(pre_tx_ts))));
                });
                clk_hook.advance(Duration::from_micros(500));
            }
        }));

    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback_now());
    drv.read_joint_states().unwrap();
    assert_eq!(drv.check_mapping_ranges().status, armd::driver::RangeCheckStatus::Passed);
    bus.with(|s| {
        s.immediate_init = false;
        s.motors[6].silent = true;
    });

    // Age past FRESHEN entry threshold, then start budgeted enable.
    clock.advance(Duration::from_millis(140));
    bus.clear_log();
    let mut err = None;
    let first = drv.begin_torque_budgeted(true, 64);
    match first {
        Err(e) => err = Some(e),
        Ok(armd::driver::TorqueProgress::Done(_)) => {
            panic!("enable must not succeed when J7 only delivered a pre-TX delayed reply")
        }
        Ok(armd::driver::TorqueProgress::Pending { .. }) => {
            for _ in 0..20 {
                clock.advance(RELEASE_PERIOD);
                let dl = clock.now() + RELEASE_PERIOD;
                match drv.poll_torque_budgeted(dl, 64) {
                    Ok(armd::driver::TorqueProgress::Pending { .. }) => {}
                    Ok(armd::driver::TorqueProgress::Done(_)) => {
                        panic!("enable must not succeed on pre-TX delayed J7 reply")
                    }
                    Err(e) => {
                        err = Some(e);
                        break;
                    }
                }
            }
        }
    }
    assert!(
        injected.load(std::sync::atomic::Ordering::SeqCst),
        "before_final_tx_gate_hook must have injected the pre-TX J7 frame"
    );
    assert!(
        matches!(err, Some(ArmDriverError::EnableFreshenTimeout { joint: 6 })),
        "expected EnableFreshenTimeout for silent J7 despite pre-TX reply, got {err:?}"
    );
    let log = bus.tx_log();
    assert!(log.iter().all(|f| f.id != 0x307), "must not emit 0x307");
    assert!(
        log.iter().all(|f| f.data != damiao::CMD_ENABLE),
        "must not emit CMD_ENABLE"
    );
}

/// Blocker 2 regression: when J7 ages past `ENABLE_FRESHEN_EXIT_PF_MAX_AGE` (60 ms) while waiting
/// on a slower arm joint, `refresh_capture` MUST stay armed across the replacement poll's
/// `drain_rx_for_boundary` so an unsafe J7 frame (motion/fault/state!=0) arriving right before the
/// replacement poll cannot be swallowed and masked by a later stationary state-0 reply.
#[test]
fn freshen_re_stale_replacement_drain_preserves_j7_unsafe_veto() {
    let clock = Arc::new(MockClock::new());
    let bus = Sg2002Bus::new(clock.clone(), Duration::from_micros(400), [1, 1, 1, 1, 1, 1, 1], true);
    let cfg = build_b601_config();
    let mut drv = build_b601_driver(bus.clone(), &cfg);

    // Hold J1 silent initially so J7 confirms early at ~20 ms while J1 waits for retry at ~60 ms.
    bus.with(|s| s.motors[0].silent = true);
    clock.advance(Duration::from_millis(140));
    bus.clear_log();

    let mut prog = drv.begin_torque_budgeted(true, 64).unwrap();
    assert!(matches!(prog, armd::driver::TorqueProgress::Pending { .. }));

    // Ticks at +20, +40, +60 ms: J2..J7 confirm at +20 ms; retry for J1 fires at +60 ms.
    for tick in 1..=3 {
        if tick == 3 {
            // Re-enable J1 right before the 60 ms retry so J1 replies at ~61 ms and confirms at +80 ms,
            // when J7's sample is already ~79 ms old (> 60 ms exit bound -> re_stale for J7).
            bus.with(|s| s.motors[0].silent = false);
        }
        clock.advance(RELEASE_PERIOD);
        let dl = clock.now() + RELEASE_PERIOD;
        prog = drv.poll_torque_budgeted(dl, 64).unwrap();
        assert!(matches!(prog, armd::driver::TorqueProgress::Pending { .. }));
    }

    // Queue `[bad_fb, good_fb]` into `inject_after_empty` so that on tick 4 (+80 ms):
    //   1) `poll_freshen`'s initial `receive_feedback()` drains J1's reply and observes empty RX
    //      (`WouldBlock` -> `last_read_drained = true`), confirming all 7 joints and triggering
    //      `re_stale` for J7 (age ~79 ms > 60 ms exit bound);
    //   2) the empty observation immediately arms `[bad_fb, good_fb]` in `bus.rx` right before
    //      `send_freshen_polls` calls `drain_rx_for_boundary` for J7's replacement `0xCC` poll;
    //   3) `drain_rx_for_boundary` consumes both frames with `refresh_capture` STILL armed, so
    //      `check_freshen_j7_unsafe_capture()` vetoes fail-closed instead of swallowing `bad_fb`.
    bus.with(|s| {
        let mut bad_m = s.motors[6];
        bad_m.enabled = true;
        bad_m.v = 0.6;
        let bad_fb = bad_m.feedback_frame();
        let good_fb = s.motors[6].feedback_frame();
        s.inject_after_empty.push_back(bad_fb);
        s.inject_after_empty.push_back(good_fb);
    });

    clock.advance(RELEASE_PERIOD);
    let dl = clock.now() + RELEASE_PERIOD;
    let res = drv.poll_torque_budgeted(dl, 64);
    assert!(
        matches!(
            res,
            Err(ArmDriverError::EnableFreshenUnsafeGripper {
                joint: 6,
                still: false,
                off: false,
                status: 1,
            })
        ),
        "unsafe J7 frame across re_stale replacement must veto with EnableFreshenUnsafeGripper, got {res:?}"
    );
    let log = bus.tx_log();
    assert!(log.iter().all(|f| f.id != 0x307), "must not emit 0x307 after veto");
    assert!(
        log.iter().all(|f| f.data != damiao::CMD_ENABLE),
        "must not emit CMD_ENABLE after veto"
    );
}
