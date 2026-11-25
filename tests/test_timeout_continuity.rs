//! Watchdog hold continuity and off-lock `mit_timeout` integration tests (B601-DM SG2002 timing).
//!
//! Emulates Damiao's hardware `TIMEOUT` watchdog (`RID_TIMEOUT = 2000` -> 100 ms at 50 µs/count on
//! J1–J6; J7 `PosForce` keeps `TIMEOUT = 0` by design; `0xCC` status polls do NOT refresh the
//! watchdog; expiry disables the motor and latches state `0xD` CAN Communication Loss) under
//! SG2002-like SPI-CAN timing (`tx_step` ~0.45–0.5 ms/frame, 1–3 ms per-motor reply delay, 50 Hz
//! 20 ms releases).

mod common;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::calibration::CalibrationDeps;
use armd::driver::{gripper_drive_from_profile, ArmDriver, DamiaoArmDriver};
use armd::grasp::GraspDeps;
use armd::mit;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{BootId, Capacities, ClientId, Outcome, Submit, TransitionOp};
use armd::rt_core::ops::{CoreOps, ReleaseFixture, VirtualClock};
use armd::rt_core::requests::config::CALIBRATION_PENDING_MARKER_NAME;
use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};
use armd::rt_core::safe_hold::{SafeHoldBuffers, SafeHoldControl};
use armd::supervisor::EnableLifecycle;
use armd::supervisor_lock::HoldKind;
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
};
use armd::{ArmConfig, ArmSupervisor, OperatorAuth, SafetyState, SupervisorInner};
use serde_json::json;
use tokio::net::TcpListener;

fn pending_commit_marker_path(calib_path: &std::path::Path) -> std::path::PathBuf {
    calib_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join(CALIBRATION_PENDING_MARKER_NAME)
}

const BOOT: BootId = BootId(0xB601_7001);
const RELEASE_PERIOD: Duration = Duration::from_millis(20);
const DT: f64 = 0.02;
const TIMEOUT_2000_COUNTS: u32 = 2000;
const MAX_ALLOWED_CTRL_GAP: Duration = Duration::from_millis(40);

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
struct TimeoutMotor {
    id: u16,
    q: f32,
    v: f32,
    temp_c: u8,
    enabled: bool,
    fault_state: u8,
    timeout_counts: u32,
    last_ctrl_at: Option<Instant>,
    max_ctrl_gap: Duration,
    watchdog_expiries: usize,
    drop_enable_count: usize,
    reply_delay: Duration,
    limits: DamiaoLimits,
}

impl TimeoutMotor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = if self.fault_state != 0 {
            self.fault_state
        } else {
            u8::from(self.enabled)
        };
        let data = [
            ((state & 0x0F) << 4) | (self.id as u8 & 0x0F),
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

    fn watchdog_limit(&self) -> Option<Duration> {
        (self.timeout_counts > 0).then(|| {
            Duration::from_micros(
                u64::from(self.timeout_counts) * u64::from(damiao::TIMEOUT_UNIT_US_DOCUMENTED),
            )
        })
    }

    fn check_watchdog(&mut self, now: Instant) {
        if !self.enabled {
            return;
        }
        let Some(limit) = self.watchdog_limit() else {
            return;
        };
        if let Some(last) = self.last_ctrl_at {
            let elapsed = now.saturating_duration_since(last);
            if elapsed >= limit {
                self.enabled = false;
                self.fault_state = 0xD;
                self.last_ctrl_at = None;
                self.watchdog_expiries += 1;
            }
        }
    }

    fn record_ctrl_refresh(&mut self, now: Instant) {
        if !self.enabled {
            return;
        }
        if let Some(prev) = self.last_ctrl_at {
            let gap = now.saturating_duration_since(prev);
            if gap > self.max_ctrl_gap {
                self.max_ctrl_gap = gap;
            }
        }
        self.last_ctrl_at = Some(now);
    }
}

struct SgTimeoutBusState {
    clock: Arc<MockClock>,
    motors: [TimeoutMotor; 7],
    ctrl_modes: [u32; 7],
    tx_step: Duration,
    immediate_init: bool,
    follow_commands: bool,
    /// When true, every `transmit()` fails with `TransportError::WouldBlock` (TX squeeze/blackout).
    block_tx: bool,
    /// Optional per-tick limit on `CMD_DISABLE` frames (models 1 ms disable burst window).
    max_disables_per_tick: Option<usize>,
    disables_this_tick: usize,
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    delayed: Vec<(Instant, CanFrame)>,
    tx_log: Vec<(Instant, CanFrame)>,
}

impl SgTimeoutBusState {
    fn run_watchdogs(&mut self) {
        let now = self.clock.now();
        for m in &mut self.motors {
            m.check_watchdog(now);
        }
    }

    fn flush_due_delayed(&mut self) {
        self.run_watchdogs();
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

    fn push_all_feedback_now(&mut self) {
        self.flush_due_delayed();
        let now = self.clock.now();
        for m in &self.motors {
            self.rx
                .push_back((m.feedback_frame(), Some(CanRxTime::Monotonic(now))));
        }
    }

    fn enqueue_reply(&mut self, idx: usize, frame: CanFrame, now: Instant) {
        if self.immediate_init {
            self.rx
                .push_back((frame, Some(CanRxTime::Monotonic(now))));
        } else {
            let ready_at = now + self.motors[idx].reply_delay;
            self.delayed.push((ready_at, frame));
        }
    }

    fn total_expiries(&self) -> usize {
        self.motors.iter().map(|m| m.watchdog_expiries).sum()
    }

    fn max_arm_ctrl_gap(&self) -> Duration {
        self.motors[..6]
            .iter()
            .map(|m| m.max_ctrl_gap)
            .max()
            .unwrap_or(Duration::ZERO)
    }
}

#[derive(Clone)]
struct SgTimeoutBus(Arc<Mutex<SgTimeoutBusState>>);

impl SgTimeoutBus {
    fn new(
        clock: Arc<MockClock>,
        tx_step: Duration,
        reply_delays_ms: [u64; 7],
        j7_already_mode_4: bool,
        arm_timeout_counts: u32,
    ) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| TimeoutMotor {
            id: (i + 1) as u16,
            q: if i == 6 { -0.14 } else { 0.0 },
            v: 0.0,
            temp_c: 31,
            enabled: false,
            fault_state: 0,
            // J1-J6 have TIMEOUT = arm_timeout_counts (2000 -> 100 ms); J7 TIMEOUT stays 0 by design.
            timeout_counts: if i < 6 { arm_timeout_counts } else { 0 },
            last_ctrl_at: None,
            max_ctrl_gap: Duration::ZERO,
            watchdog_expiries: 0,
            drop_enable_count: 0,
            reply_delay: Duration::from_millis(reply_delays_ms[i]),
            limits: limits[i],
        });
        let mut ctrl_modes = [damiao::CTRL_MODE_POS_VEL; 7];
        if j7_already_mode_4 {
            ctrl_modes[6] = damiao::CTRL_MODE_POS_FORCE;
        }
        Self(Arc::new(Mutex::new(SgTimeoutBusState {
            clock,
            motors,
            ctrl_modes,
            tx_step,
            immediate_init: true,
            follow_commands: true,
            block_tx: false,
            max_disables_per_tick: None,
            disables_this_tick: 0,
            rx: VecDeque::with_capacity(512),
            delayed: Vec::new(),
            tx_log: Vec::new(),
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut SgTimeoutBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn begin_tick(&self) {
        self.with(|s| {
            s.disables_this_tick = 0;
            s.run_watchdogs();
        });
    }

    fn clear_log(&self) {
        self.with(|s| s.tx_log.clear());
    }

    fn total_expiries(&self) -> usize {
        self.with(|s| {
            s.run_watchdogs();
            s.total_expiries()
        })
    }

    fn max_arm_ctrl_gap(&self) -> Duration {
        self.with(|s| {
            s.run_watchdogs();
            s.max_arm_ctrl_gap()
        })
    }
}

impl CanTransport for SgTimeoutBus {
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
        s.run_watchdogs();
        if s.block_tx {
            return Err(TransportError::WouldBlock);
        }
        if !frame.is_extended
            && frame.dlc == 8
            && frame.data == damiao::CMD_DISABLE
            && s.max_disables_per_tick
                .is_some_and(|cap| s.disables_this_tick >= cap)
        {
            return Err(TransportError::WouldBlock);
        }
        if !s.immediate_init && !s.tx_step.is_zero() {
            s.clock.advance(s.tx_step);
        }
        s.run_watchdogs();
        let now = s.clock.now();
        s.tx_log.push((now, *frame));

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
                if rid == damiao::RID_CTRL_MODE {
                    s.ctrl_modes[idx] = raw;
                } else if rid == damiao::RID_TIMEOUT {
                    s.motors[idx].timeout_counts = raw;
                }
                let reply = CanFrame::new_standard(0x10 + id, &frame.data).unwrap();
                s.enqueue_reply(idx, reply, now);
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
                    damiao::RID_TIMEOUT => Some(m.timeout_counts.to_le_bytes()),
                    damiao::RID_ESC_ID => Some(u32::from(id).to_le_bytes()),
                    damiao::RID_MST_ID => Some((0x10 + u32::from(id)).to_le_bytes()),
                    damiao::RID_SW_VER => Some(0x3831_3037u32.to_le_bytes()),
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

        // 0xCC status poll: does NOT refresh Damiao TIMEOUT watchdog!
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
                if s.motors[idx].drop_enable_count > 0 {
                    s.motors[idx].drop_enable_count -= 1;
                    return Ok(());
                }
                s.motors[idx].enabled = true;
                s.motors[idx].fault_state = 0;
                s.motors[idx].last_ctrl_at = Some(now);
                let fb = s.motors[idx].feedback_frame();
                s.enqueue_reply(idx, fb, now);
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_DISABLE {
            s.disables_this_tick += 1;
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                s.motors[idx].enabled = false;
                s.motors[idx].last_ctrl_at = None;
                let fb = s.motors[idx].feedback_frame();
                s.enqueue_reply(idx, fb, now);
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_CLEAR_FAULTS {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                s.motors[idx].fault_state = 0;
                let fb = s.motors[idx].feedback_frame();
                s.enqueue_reply(idx, fb, now);
            }
            return Ok(());
        }

        // POS_VEL control frame (0x100 + id): refreshes watchdog when enabled.
        if let Some(idx) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id)
        {
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_POS_VEL {
                let target_q =
                    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                if s.follow_commands && target_q.is_finite() {
                    s.motors[idx].q = target_q;
                    s.motors[idx].v = 0.0;
                }
                s.motors[idx].record_ctrl_refresh(now);
            }
            let fb = s.motors[idx].feedback_frame();
            s.enqueue_reply(idx, fb, now);
            return Ok(());
        }

        // POS_FORCE control frame (0x300 + id): refreshes watchdog when enabled.
        if let Some(idx) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x300 == frame.id)
        {
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_POS_FORCE {
                let target_q =
                    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                let vel_raw = u16::from_le_bytes([frame.data[4], frame.data[5]]);
                if s.follow_commands && target_q.is_finite() && vel_raw > 0 {
                    s.motors[idx].q = target_q;
                    s.motors[idx].v = 0.0;
                }
                s.motors[idx].record_ctrl_refresh(now);
            }
            let fb = s.motors[idx].feedback_frame();
            s.enqueue_reply(idx, fb, now);
            return Ok(());
        }

        // MIT control frame (base id): refreshes watchdog when enabled in MIT mode.
        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_MIT {
                s.motors[idx].record_ctrl_refresh(now);
            }
            let fb = s.motors[idx].feedback_frame();
            s.enqueue_reply(idx, fb, now);
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

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let (f, t) = self.receive_with_rx_time()?;
        let inst = t.map(|rt| match rt {
            CanRxTime::Monotonic(i) => i,
            CanRxTime::KernelRealtimeNs(_) => self.0.lock().unwrap().clock.now(),
        });
        Ok((f, inst))
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.run_watchdogs();
        Ok(())
    }
}

fn build_b601_config() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.resting_pose = Some(armd::model::RestingPosePolicy {
        positions: Some(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.14]),
        ..armd::model::RestingPosePolicy::rebot_dm()
    });
    cfg
}

fn build_b601_driver(bus: SgTimeoutBus, cfg: &ArmConfig) -> DamiaoArmDriver<SgTimeoutBus> {
    let limits: Vec<DamiaoLimits> = cfg
        .damiao_ranges
        .as_ref()
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let gripper = gripper_drive_from_profile(cfg).unwrap();
    let tx_budget = cfg.damiao_tx_budget();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(gripper)
        .unwrap()
        .with_tx_budget(tx_budget);
    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback_now());
    drv.read_joint_states().unwrap();
    let rc = drv.check_mapping_ranges();
    assert_eq!(rc.status, armd::driver::RangeCheckStatus::Passed);
    bus.with(|s| s.immediate_init = false);
    drv
}

#[test]
fn damiao_timeout_emulation_positive_control_expires_on_0xcc_only() {
    let clock = Arc::new(MockClock::new());
    let mut bus = SgTimeoutBus::new(
        clock.clone(),
        Duration::from_micros(450),
        [1, 2, 2, 1, 3, 2, 2],
        true,
        TIMEOUT_2000_COUNTS,
    );
    bus.with(|s| s.immediate_init = false);

    // Enable J1 (TIMEOUT=2000 -> 100 ms), then send ONLY 0xCC polls for 110 ms.
    bus.transmit(&damiao::build_enable_frame(1)).unwrap();
    assert!(bus.with(|s| s.motors[0].enabled));

    for _ in 0..11 {
        clock.advance(Duration::from_millis(10));
        bus.transmit(&damiao::build_refresh_frame(1)).unwrap();
        while bus.receive_with_rx_time().is_ok() {}
    }

    assert_eq!(
        bus.total_expiries(),
        1,
        "positive control: 0xCC MUST NOT refresh Damiao TIMEOUT=2000"
    );
    bus.with(|s| {
        assert!(!s.motors[0].enabled, "expired motor must self-disable");
        assert_eq!(
            s.motors[0].fault_state, 0xD,
            "expired motor must report state 0xD (CAN Communication Loss)"
        );
    });
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn seeded_50_enables_from_stale_disarmed_idle_zero_expiries() {
    let mut legacy_pass = 0usize;
    let mut rt_pass = 0usize;
    let mut worst_gap = Duration::ZERO;

    for seed in 0u64..50 {
        let mut rng = SeedRng::new(seed ^ 0x7001_5050);
        let tx_step = Duration::from_micros(rng.range(450, 550));
        let reply_delays_ms: [u64; 7] = std::array::from_fn(|_| rng.range(1, 3));
        let already_mode_4 = (seed % 3) != 0;
        let idle_ticks = rng.range(45, 90) as usize;
        let extra_stale_ms = rng.range(0, 60);
        let use_legacy = (seed % 2) == 0;

        let clock = Arc::new(MockClock::new());
        let bus = SgTimeoutBus::new(
            clock.clone(),
            tx_step,
            reply_delays_ms,
            already_mode_4,
            TIMEOUT_2000_COUNTS,
        );
        let cfg = build_b601_config();
        let drv = build_b601_driver(bus.clone(), &cfg);

        if use_legacy {
            let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
            let conn = sup.try_claim_ws_slot().unwrap();
            let conn1_id = conn.id();
            sup.register_ws_connection_for_slot(conn1_id).await.unwrap();
            let gen = {
                let mut inner = sup.inner().write().await;
                inner
                    .acquire_lease(&format!("connection:{conn1_id}"))
                    .unwrap()
            };

            for _ in 0..idle_ticks {
                clock.advance(RELEASE_PERIOD);
                bus.begin_tick();
                let mut inner = sup.inner().write().await;
                let mut now_fn = || clock.now();
                let dl = clock.now() + RELEASE_PERIOD;
                inner.tick_control_with_clock(DT, &mut now_fn, dl);
            }
            if extra_stale_ms > 0 {
                clock.advance(Duration::from_millis(extra_stale_ms));
            }

            let (ep, mut stamp) = {
                let inner = sup.inner().read().await;
                (inner.safety.epoch_id(), inner.current_admission_stamp())
            };
            stamp.request_id = 1;
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

            for _ in 1..=35 {
                if ws_task.is_finished() {
                    break;
                }
                clock.advance(RELEASE_PERIOD);
                bus.begin_tick();
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
            assert!(
                matches!(outcome, TorqueWsOutcome::Accepted { is_torqued: true, .. }),
                "seed={seed} (legacy): expected Accepted(is_torqued=true), got {outcome:?}"
            );
            // Run 5 post-enable armed ticks to confirm clean handoff.
            for _ in 0..5 {
                clock.advance(RELEASE_PERIOD);
                bus.begin_tick();
                let mut inner = sup.inner().write().await;
                let mut now_fn = || clock.now();
                let dl = clock.now() + RELEASE_PERIOD;
                inner.tick_control_with_clock(DT, &mut now_fn, dl);
            }
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

            for _ in 0..idle_ticks {
                bus.begin_tick();
                let _ = fx.step();
            }
            if extra_stale_ms > 0 {
                clock.advance(Duration::from_millis(extra_stale_ms));
            }

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

            let mut ended = None;
            for _ in 1..=35 {
                bus.begin_tick();
                let rep = fx.step();
                if let Some((_, o)) = rep.transition_ended {
                    ended = Some(o);
                    break;
                }
            }
            assert_eq!(
                ended,
                Some(Outcome::Completed),
                "seed={seed} (rt): expected Outcome::Completed, state={:?}",
                fx.core.safety.state()
            );
            for _ in 0..5 {
                bus.begin_tick();
                let _ = fx.step();
            }
            rt_pass += 1;
        }

        assert_eq!(
            bus.total_expiries(),
            0,
            "seed={seed}: expected 0 Damiao TIMEOUT expiries"
        );
        let gap = bus.max_arm_ctrl_gap();
        assert!(
            gap <= MAX_ALLOWED_CTRL_GAP,
            "seed={seed}: arm control frame gap {gap:?} exceeded 40 ms bound"
        );
        worst_gap = worst_gap.max(gap);
        bus.with(|s| {
            for (i, m) in s.motors.iter().enumerate() {
                assert!(m.enabled, "seed={seed}: motor {} must be enabled", i + 1);
                assert_eq!(m.fault_state, 0, "seed={seed}: motor {} faulted", i + 1);
            }
        });
    }

    assert_eq!(legacy_pass + rt_pass, 50);
    println!(
        "[TIMEOUT-CONT 50-enables] legacy_pass={legacy_pass} rt_pass={rt_pass} expiries=0 worst_arm_ctrl_gap_ms={:.2}",
        worst_gap.as_secs_f64() * 1e3
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn enable_60s_hold_and_multitick_rest_disable_zero_expiries() {
    for use_legacy in [true, false] {
        let label = if use_legacy { "legacy" } else { "rt" };
        let clock = Arc::new(MockClock::new());
        let bus = SgTimeoutBus::new(
            clock.clone(),
            Duration::from_micros(500),
            [1, 2, 3, 1, 2, 3, 2],
            true,
            TIMEOUT_2000_COUNTS,
        );
        // Model 1 ms disable burst window (2 CMD_DISABLE frames per 20 ms tick -> 4-tick Phase A).
        bus.with(|s| s.max_disables_per_tick = Some(2));
        let cfg = build_b601_config();
        let drv = build_b601_driver(bus.clone(), &cfg);

        if use_legacy {
            let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
            let conn = sup.try_claim_ws_slot().unwrap();
            let conn1_id = conn.id();
            sup.register_ws_connection_for_slot(conn1_id).await.unwrap();
            let gen = {
                let mut inner = sup.inner().write().await;
                inner
                    .acquire_lease(&format!("connection:{conn1_id}"))
                    .unwrap()
            };

            // Stale idle (120 ms) -> enable.
            clock.advance(Duration::from_millis(120));
            let (ep, mut stamp) = {
                let inner = sup.inner().read().await;
                (inner.safety.epoch_id(), inner.current_admission_stamp())
            };
            stamp.request_id = 1;
            let sup_ws = sup.clone();
            let conn_en = conn;
            let en_task = tokio::spawn(async move {
                sup_ws
                    .torque_ws_with_deadline(
                        conn_en,
                        gen,
                        Some(ep),
                        true,
                        1,
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
                if sup.inner().read().await.torque_transition_pending() || en_task.is_finished() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            for _ in 0..35 {
                if en_task.is_finished() {
                    break;
                }
                clock.advance(RELEASE_PERIOD);
                bus.begin_tick();
                let mut inner = sup.inner().write().await;
                let mut now_fn = || clock.now();
                let dl = clock.now() + RELEASE_PERIOD;
                inner.tick_control_with_clock(DT, &mut now_fn, dl);
            }
            let en_out = en_task.await.unwrap();
            assert!(
                matches!(en_out, TorqueWsOutcome::Accepted { is_torqued: true, .. }),
                "{label}: enable failed: {en_out:?}"
            );

            // 60 s hold = 3000 ticks at 50 Hz.
            for _ in 0..3000 {
                clock.advance(RELEASE_PERIOD);
                bus.begin_tick();
                let mut inner = sup.inner().write().await;
                let mut now_fn = || clock.now();
                let dl = clock.now() + RELEASE_PERIOD;
                inner.tick_control_with_clock(DT, &mut now_fn, dl);
            }
            assert_eq!(bus.total_expiries(), 0, "{label}: 0 expiries during 60s hold");

            // Multi-tick rest disable using the same live connection/lease.
            // (`MockClock` is 60 s ahead of wall clock after 3000 simulated ticks, so size the
            // legacy WS request deadline above the 60 s virtual clock offset.)
            let (ep2, mut stamp2) = {
                let inner = sup.inner().read().await;
                (inner.safety.epoch_id(), inner.current_admission_stamp())
            };
            stamp2.request_id = 2;
            let sup_ws2 = sup.clone();
            let dis_task = tokio::spawn(async move {
                sup_ws2
                    .torque_ws_with_deadline(
                        conn,
                        gen,
                        Some(ep2),
                        true,
                        2,
                        Ok(false),
                        OwnerAuthPolicy::None,
                        OperatorGrant::unenforced(),
                        stamp2,
                        Duration::from_secs(120),
                    )
                    .await
                    .unwrap()
            });
            for _ in 0..200 {
                if sup.inner().read().await.torque_transition_pending() || dis_task.is_finished() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            for _ in 0..60 {
                if dis_task.is_finished() {
                    break;
                }
                clock.advance(RELEASE_PERIOD);
                bus.begin_tick();
                let mut inner = sup.inner().write().await;
                let mut now_fn = || clock.now();
                let dl = clock.now() + RELEASE_PERIOD;
                inner.tick_control_with_clock(DT, &mut now_fn, dl);
            }
            let dis_out = dis_task.await.unwrap();
            assert!(
                matches!(dis_out, TorqueWsOutcome::Accepted { is_torqued: false, .. }),
                "{label}: rest disable failed: {dis_out:?}"
            );
            assert!(!sup.inner().read().await.is_torqued);
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

            clock.advance(Duration::from_millis(120));
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
            let mut en_ended = None;
            for _ in 0..35 {
                bus.begin_tick();
                if let Some((_, o)) = fx.step().transition_ended {
                    en_ended = Some(o);
                    break;
                }
            }
            assert_eq!(en_ended, Some(Outcome::Completed), "{label}: enable failed");

            // 60 s hold = 3000 ticks at 50 Hz.
            for _ in 0..3000 {
                bus.begin_tick();
                let _ = fx.step();
            }
            assert_eq!(bus.total_expiries(), 0, "{label}: 0 expiries during 60s hold");

            // Multi-tick rest disable.
            let auth2 = fx.core.authority(&fx.ctx(fx.now()));
            broker
                .submit(
                    ClientId(1),
                    2,
                    auth2,
                    fx.now() + Duration::from_secs(5),
                    Submit::Transition(TransitionOp::TorqueOff),
                )
                .unwrap();
            let mut dis_ended = None;
            for _ in 0..60 {
                bus.begin_tick();
                if let Some((_, o)) = fx.step().transition_ended {
                    dis_ended = Some(o);
                    break;
                }
            }
            assert_eq!(dis_ended, Some(Outcome::Completed), "{label}: rest disable failed");
            assert!(!fx.core.is_torqued);
        }

        assert_eq!(bus.total_expiries(), 0, "{label}: expected 0 expiries overall");
        let gap = bus.max_arm_ctrl_gap();
        assert!(
            gap <= MAX_ALLOWED_CTRL_GAP,
            "{label}: max arm control gap {gap:?} > 40 ms"
        );
        bus.with(|s| {
            for (i, m) in s.motors.iter().enumerate() {
                assert!(!m.enabled, "{label}: motor {} still enabled after disable", i + 1);
                assert_eq!(m.fault_state, 0, "{label}: motor {} faulted", i + 1);
            }
        });
        println!(
            "[TIMEOUT-CONT 60s-hold {label}] ticks=3000 expiries=0 max_arm_ctrl_gap_ms={:.2}",
            gap.as_secs_f64() * 1e3
        );
    }
}

#[test]
fn injected_1tick_tx_budget_squeeze_and_multi_tick_blackout_abort_zero_expiries() {
    // Part A: 1-tick TX budget squeeze right after initial CMD_ENABLE frames -> still succeeds with 0 expiries.
    {
        let clock = Arc::new(MockClock::new());
        let bus = SgTimeoutBus::new(
            clock.clone(),
            Duration::from_micros(500),
            [2, 2, 2, 2, 2, 2, 2],
            true,
            TIMEOUT_2000_COUNTS,
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

        // Drop first enable frame on J7 so enable spans multiple ticks after J1-J6 energize.
        bus.with(|s| s.motors[6].drop_enable_count = 1);
        clock.advance(Duration::from_millis(120));
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

        let mut squeezed = false;
        let mut ended = None;
        for _ in 0..35 {
            let any_arm_enabled = bus.with(|s| s.motors[..6].iter().any(|m| m.enabled));
            let squeeze_this_tick = !squeezed && any_arm_enabled;
            if squeeze_this_tick {
                squeezed = true;
                bus.with(|s| s.block_tx = true);
            }
            bus.begin_tick();
            let rep = fx.step();
            if squeeze_this_tick {
                bus.with(|s| s.block_tx = false);
            }
            if let Some((_, o)) = rep.transition_ended {
                ended = Some(o);
                break;
            }
        }
        assert!(squeezed, "must have injected the 1-tick TX squeeze after arm motors energized");
        assert_eq!(
            ended,
            Some(Outcome::Completed),
            "1-tick TX squeeze during enable must still complete cleanly: state={:?}",
            fx.core.safety.state()
        );
        assert_eq!(bus.total_expiries(), 0, "1-tick TX squeeze must produce 0 expiries");
        let squeeze_gap = bus.max_arm_ctrl_gap();
        assert!(
            squeeze_gap <= MAX_ALLOWED_CTRL_GAP,
            "1-tick TX squeeze arm control gap {squeeze_gap:?} exceeded 40 ms bound"
        );
        println!(
            "[TIMEOUT-CONT 1-tick-squeeze] completed=true expiries=0 max_arm_ctrl_gap_ms={:.2}",
            squeeze_gap.as_secs_f64() * 1e3
        );
    }

    // Part B: Severe multi-tick TX blackout (4 ticks = 80 ms) during enable -> deliberate
    // fail-closed WatchdogContinuityAbort BEFORE 100 ms TIMEOUT expiry, entering FailedHold with 0 expiries.
    {
        let clock = Arc::new(MockClock::new());
        let bus = SgTimeoutBus::new(
            clock.clone(),
            Duration::from_micros(500),
            [2, 2, 2, 2, 2, 2, 2],
            true,
            TIMEOUT_2000_COUNTS,
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

        bus.with(|s| s.motors[6].drop_enable_count = 10);
        clock.advance(Duration::from_millis(120));
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        broker
            .submit(
                ClientId(1),
                2,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();

        let mut blackout_ticks = 0usize;
        let mut aborted = None;
        for _ in 0..35 {
            let any_arm_enabled = bus.with(|s| s.motors[..6].iter().any(|m| m.enabled));
            if any_arm_enabled && blackout_ticks < 4 {
                bus.with(|s| s.block_tx = true);
                blackout_ticks += 1;
            }
            bus.begin_tick();
            let rep = fx.step();
            if let Some((_, o)) = rep.transition_ended {
                aborted = Some(o);
                break;
            }
        }
        // Clear blackout immediately upon abort so FailedHold takes over before 100 ms TIMEOUT.
        bus.with(|s| s.block_tx = false);
        assert!(
            matches!(aborted, Some(Outcome::Failed(_))),
            "sustained multi-tick blackout must fail closed before TIMEOUT expiry, got {aborted:?}"
        );
        assert_eq!(
            fx.core.enable_lifecycle(),
            EnableLifecycle::FailedHold,
            "deliberate watchdog continuity abort must enter FailedHold"
        );
        assert_eq!(
            bus.total_expiries(),
            0,
            "fail-closed abort MUST happen BEFORE any Damiao 100 ms TIMEOUT expiry"
        );
        // Once bus recovers, FailedHold holds J1-J6 for 50 ticks (1 s) with 0 expiries.
        for _ in 0..50 {
            bus.begin_tick();
            let _ = fx.step();
        }
        assert_eq!(bus.total_expiries(), 0, "FailedHold after abort must have 0 expiries");
        println!(
            "[TIMEOUT-CONT blackout-abort] blackout_ticks={blackout_ticks} outcome={aborted:?} expiries=0"
        );
    }
}

#[test]
fn dev20_soak_loop_30_cycles_with_timeout_2000_zero_expiries() {
    let clock = Arc::new(MockClock::new());
    let bus = SgTimeoutBus::new(
        clock.clone(),
        Duration::from_micros(480),
        [1, 2, 2, 1, 3, 2, 2],
        true,
        TIMEOUT_2000_COUNTS,
    );
    bus.with(|s| s.max_disables_per_tick = Some(2));
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
            commands: 64,
            plans: 8,
            replies: 128,
        },
        VirtualClock::from_mock_clock((*clock).clone()),
    );

    let mut seq = 1u64;
    for cycle in 0..30 {
        let _ = broker.try_poll(|_, _| armd::rt_core::broker::DeliveryDecision::Delivered);
        // 1. Stale disabled idle (RefreshScheduler active, 8 ticks = 160 ms).
        for _ in 0..8 {
            bus.begin_tick();
            let _ = fx.step();
        }

        // 2. Enable.
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        broker
            .submit(
                ClientId(1),
                seq,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        seq += 1;
        let mut en_ended = None;
        let mut last_cmd = None;
        for _ in 0..35 {
            bus.begin_tick();
            let rep = fx.step();
            if rep.command.is_some() {
                last_cmd = rep.command;
            }
            if let Some((_, o)) = rep.transition_ended {
                en_ended = Some(o);
                break;
            }
        }
        assert_eq!(
            en_ended,
            Some(Outcome::Completed),
            "cycle {cycle}: enable failed (cmd={last_cmd:?}, state={:?}, torqued={})",
            fx.core.safety.state(),
            fx.core.is_torqued
        );

        // 3. Motion away from rest + stop + return to rest.
        {
            let mut target = fx.core.commanded_positions.clone();
            target[0] = 0.25;
            target[1] = -0.15;
            fx.core.servo_target = Some(target);
        }
        for _ in 0..10 {
            bus.begin_tick();
            let _ = fx.step();
        }
        fx.core.stop_motion();
        for _ in 0..3 {
            bus.begin_tick();
            let _ = fx.step();
        }
        // Return smoothly to rest pose and settle K=3 ticks.
        {
            let rest = fx.core.config.resting_positions();
            fx.core.servo_target = Some(rest.to_vec());
        }
        for _ in 0..12 {
            bus.begin_tick();
            let _ = fx.step();
        }
        fx.core.stop_motion();
        for _ in 0..5 {
            bus.begin_tick();
            let _ = fx.step();
        }

        // 4. Multi-tick rest disable.
        let auth2 = fx.core.authority(&fx.ctx(fx.now()));
        broker
            .submit(
                ClientId(1),
                seq,
                auth2,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        seq += 1;
        let mut dis_ended = None;
        for _ in 0..60 {
            bus.begin_tick();
            if let Some((_, o)) = fx.step().transition_ended {
                dis_ended = Some(o);
                break;
            }
        }
        assert_eq!(dis_ended, Some(Outcome::Completed), "cycle {cycle}: rest disable failed");
        assert!(!fx.core.is_torqued, "cycle {cycle}: expected disarmed");
        assert!(
            matches!(fx.core.safety.state(), SafetyState::Idle),
            "cycle {cycle}: unexpected safety state {:?}",
            fx.core.safety.state()
        );
        assert_eq!(bus.total_expiries(), 0, "cycle {cycle}: expected 0 expiries");
    }

    let gap = bus.max_arm_ctrl_gap();
    assert!(
        gap <= MAX_ALLOWED_CTRL_GAP,
        "dev20 30x soak: max arm control gap {gap:?} > 40 ms"
    );
    println!(
        "[TIMEOUT-CONT dev20-soak-30x] cycles=30 expiries=0 max_arm_ctrl_gap_ms={:.2}",
        gap.as_secs_f64() * 1e3
    );
}

#[test]
fn failed_hold_and_safe_hold_continuity_with_timeout_2000_zero_expiries() {
    // 1. FailedHold from partial enable (J7 drops enable, J1-J6 enable) -> 1 s hold: 0 expiries.
    {
        let clock = Arc::new(MockClock::new());
        let bus = SgTimeoutBus::new(
            clock.clone(),
            Duration::from_micros(480),
            [1, 2, 2, 1, 2, 1, 2],
            true,
            TIMEOUT_2000_COUNTS,
        );
        bus.with(|s| s.motors[6].drop_enable_count = 20);
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
        let mut ended = None;
        for _ in 0..30 {
            bus.begin_tick();
            if let Some((_, o)) = fx.step().transition_ended {
                ended = Some(o);
                break;
            }
        }
        assert!(matches!(ended, Some(Outcome::Failed(_))));
        assert_eq!(fx.core.enable_lifecycle(), EnableLifecycle::FailedHold);
        for _ in 0..50 {
            bus.begin_tick();
            let _ = fx.step();
        }
        assert_eq!(bus.total_expiries(), 0, "FailedHold must have 0 expiries");
        assert!(
            bus.max_arm_ctrl_gap() <= MAX_ALLOWED_CTRL_GAP,
            "FailedHold gap {:?} > 40 ms",
            bus.max_arm_ctrl_gap()
        );
    }

    // 2. SAFE-HOLD entered while armed with TIMEOUT = 2000 -> 2 s hold (100 ticks): 0 expiries.
    {
        let clock = Arc::new(MockClock::new());
        let bus = SgTimeoutBus::new(
            clock.clone(),
            Duration::from_micros(480),
            [1, 2, 2, 1, 2, 1, 2],
            true,
            TIMEOUT_2000_COUNTS,
        );
        let cfg = build_b601_config();
        let mut drv = build_b601_driver(bus.clone(), &cfg);
        drv.set_torque(true).unwrap();

        let mut inner = SupervisorInner::new(cfg);
        inner.is_torqued = true;
        inner.measured_positions = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.14];
        inner.feedback_numeric_valid = true;

        let mut sh = SafeHoldBuffers::from_config(&inner.config, RELEASE_PERIOD);
        sh.mirror_from_core(&inner);
        sh.seed_on_entry(&inner);
        sh.prepare_driver_on_entry(&mut drv);
        let ctrl = SafeHoldControl::new();

        for _ in 0..100 {
            clock.advance(RELEASE_PERIOD);
            bus.begin_tick();
            let dl = clock.now() + RELEASE_PERIOD;
            let mut now_fn = || clock.now();
            sh.step_driver(&mut drv, &ctrl, dl, &mut now_fn);
        }
        assert_eq!(bus.total_expiries(), 0, "SAFE-HOLD must have 0 expiries");
        assert!(
            bus.max_arm_ctrl_gap() <= MAX_ALLOWED_CTRL_GAP,
            "SAFE-HOLD gap {:?} > 40 ms",
            bus.max_arm_ctrl_gap()
        );
        println!(
            "[TIMEOUT-CONT FailedHold+SAFE-HOLD] expiries=0 max_arm_ctrl_gap_ms={:.2}",
            bus.max_arm_ctrl_gap().as_secs_f64() * 1e3
        );
    }
}

/// Real wall-clock Damiao CAN bus for testing legacy WS `/api/v1/calibration/mit_timeout`
/// off-lock guard holds (`<= 2 ms`) and `0` skipped 50 Hz releases while register replies carry
/// real wall-clock latency (10 ms per register -> multi-tick transaction spanning > 150 ms).
struct WallMitBusState {
    motors: [TimeoutMotor; 7],
    ctrl_modes: [u32; 7],
    rx: VecDeque<(CanFrame, Option<Instant>)>,
    delayed: Vec<(Instant, CanFrame)>,
    tx_log: Vec<CanFrame>,
    reg_reply_delay: Duration,
}

impl WallMitBusState {
    fn drain_matured(&mut self, now: Instant) {
        let mut i = 0;
        while i < self.delayed.len() {
            if self.delayed[i].0 <= now {
                let (at, f) = self.delayed.swap_remove(i);
                self.rx.push_back((f, Some(at)));
            } else {
                i += 1;
            }
        }
    }

    fn push_all_feedback(&mut self) {
        let now = Instant::now();
        for m in &self.motors {
            self.rx.push_back((m.feedback_frame(), Some(now)));
        }
    }
}

#[derive(Clone)]
struct WallMitBus(Arc<Mutex<WallMitBusState>>);

impl WallMitBus {
    fn new(reg_reply_delay: Duration) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| TimeoutMotor {
            id: (i + 1) as u16,
            q: 0.0,
            v: 0.0,
            temp_c: 30,
            enabled: false,
            fault_state: 0,
            timeout_counts: 0,
            last_ctrl_at: None,
            max_ctrl_gap: Duration::ZERO,
            watchdog_expiries: 0,
            drop_enable_count: 0,
            reply_delay: Duration::ZERO,
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(WallMitBusState {
            motors,
            ctrl_modes: [damiao::CTRL_MODE_POS_VEL; 7],
            rx: VecDeque::with_capacity(256),
            delayed: Vec::new(),
            tx_log: Vec::new(),
            reg_reply_delay,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut WallMitBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl CanTransport for WallMitBus {
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
            let delay = s.reg_reply_delay;
            match op {
                0xCC => {
                    let fb = s.motors[k].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
                damiao::REG_READ => {
                    let m = s.motors[k];
                    let cm = s.ctrl_modes[k];
                    let val: Option<[u8; 4]> = match rid {
                        damiao::RID_CTRL_MODE => Some(cm.to_le_bytes()),
                        damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                        damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                        damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                        damiao::RID_TIMEOUT => Some(m.timeout_counts.to_le_bytes()),
                        damiao::RID_ESC_ID => Some(u32::from(id).to_le_bytes()),
                        damiao::RID_MST_ID => Some((0x10 + u32::from(id)).to_le_bytes()),
                        damiao::RID_SW_VER => Some(0x3831_3037u32.to_le_bytes()),
                        _ => None,
                    };
                    if let Some(v) = val {
                        let [lo, hi] = id.to_le_bytes();
                        let reply = CanFrame::new_standard(
                            0x10 + id,
                            &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                        )
                        .unwrap();
                        if delay.is_zero() {
                            s.rx.push_back((reply, Some(now)));
                        } else {
                            s.delayed.push((now + delay, reply));
                        }
                    }
                }
                damiao::REG_WRITE => {
                    let raw = u32::from_le_bytes([
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7],
                    ]);
                    if rid == damiao::RID_CTRL_MODE {
                        s.ctrl_modes[k] = raw;
                    } else if rid == damiao::RID_TIMEOUT {
                        s.motors[k].timeout_counts = raw;
                    }
                    let reply = CanFrame::new_standard(0x10 + id, &frame.data).unwrap();
                    if delay.is_zero() {
                        s.rx.push_back((reply, Some(now)));
                    } else {
                        s.delayed.push((now + delay, reply));
                    }
                }
                _ => {}
            }
            return Ok(());
        }
        if let Some(k) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if frame.data == damiao::CMD_ENABLE {
                s.motors[k].enabled = true;
            } else if frame.data == damiao::CMD_DISABLE {
                s.motors[k].enabled = false;
            }
            let fb = s.motors[k].feedback_frame();
            s.rx.push_back((fb, Some(now)));
        } else if let Some(k) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id)
        {
            let fb = s.motors[k].feedback_frame();
            s.rx.push_back((fb, Some(now)));
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

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_ws_mit_timeout_off_lock_bounds_guard_hold_under_2ms_and_zero_skipped_releases() {
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("calibration.json");
    let setup_path = dir.path().join("grasp_setup.json");

    let bus = WallMitBus::new(Duration::from_millis(8));
    let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| {
        s.reg_reply_delay = Duration::ZERO;
        s.push_all_feedback();
    });
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    // Restore 8 ms wall-clock register latency after startup range check.
    bus.with(|s| s.reg_reply_delay = Duration::from_millis(8));

    let cfg = ArmConfig {
        lease_timeout_ms: 15_000,
        workspace: None,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(dm)));
    sup.control_tick(0.02).await;

    let det_up = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let grasp_deps = GraspDeps::http(det_up.clone(), Some(setup_path));
    let cal_deps = CalibrationDeps::new(calib_path.clone(), None, det_up.clone());
    let router = armd::api::create_router_calibration(
        sup.clone(),
        det_up,
        grasp_deps,
        armd::diagnostics::DiagnosticsState::global(),
        cal_deps,
        OperatorAuth::disabled(),
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let server_jh = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });

    // Background feedback feeder + production 50 Hz (20 ms) control ticker measuring skipped releases.
    let stop_flag = Arc::new(AtomicBool::new(false));
    let ticks = Arc::new(AtomicU64::new(0));
    let skipped_releases = Arc::new(AtomicU64::new(0));
    let max_release_gap_us = Arc::new(AtomicU64::new(0));
    let measure_window = Arc::new(AtomicBool::new(false));

    let feeder_stop = stop_flag.clone();
    let feeder_bus = bus.clone();
    let feeder_jh = tokio::spawn(async move {
        while !feeder_stop.load(Ordering::Relaxed) {
            feeder_bus.with(|s| {
                if s.rx.len() < 16 {
                    s.push_all_feedback();
                }
            });
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    });

    let tick_sup = sup.clone();
    let tick_stop = stop_flag.clone();
    let tick_cnt = ticks.clone();
    let tick_skip = skipped_releases.clone();
    let tick_gap = max_release_gap_us.clone();
    let tick_measure = measure_window.clone();
    let ticker_jh = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Burst);
        let mut prev_at: Option<Instant> = None;
        while !tick_stop.load(Ordering::Relaxed) {
            interval.tick().await;
            let now = Instant::now();
            if tick_measure.load(Ordering::Acquire) {
                if let Some(prev) = prev_at {
                    let gap = now.saturating_duration_since(prev);
                    let gap_us = gap.as_micros().min(u128::from(u64::MAX)) as u64;
                    tick_gap.fetch_max(gap_us, Ordering::AcqRel);
                    // A 50 Hz release skipped means >= 2 periods (>= 38 ms) between consecutive releases.
                    if gap >= Duration::from_millis(38) {
                        tick_skip.fetch_add(1, Ordering::AcqRel);
                    }
                }
            }
            prev_at = Some(now);
            tick_sup.control_tick(0.02).await;
            tick_cnt.fetch_add(1, Ordering::Release);
        }
    });

    let (mut ws, gen) = common::controller(addr).await;
    // Let 50 Hz ticker warm up & establish K=3 causal rest-pose evidence.
    let t0 = ticks.load(Ordering::Acquire);
    while ticks.load(Ordering::Acquire) < t0 + 6 {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    let epoch = sup.get_observation().await.epoch_id;
    sup.inner().set_hold_history_enabled(true);
    measure_window.store(true, Ordering::Release);
    let ticks_before = ticks.load(Ordering::Acquire);
    let samples_before = sup.tick_samples().len();
    let started = Instant::now();

    let ack = common::rpc(
        &mut ws,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;

    let elapsed = started.elapsed();
    measure_window.store(false, Ordering::Release);
    let ticks_after = ticks.load(Ordering::Acquire);
    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);

    assert_eq!(ack["type"], "command_ack", "expected command_ack: {ack}");
    assert_eq!(ack["dry_run"], false, "{ack}");
    assert_eq!(ack["timeout_counts"], 2000, "{ack}");
    assert_eq!(ack["commissioning"].as_array().map(Vec::len), Some(6), "{ack}");
    assert!(
        !pending_commit_marker_path(&calib_path).exists(),
        ".calibration_commit.pending must be cleaned up after all 6 axes applied"
    );
    bus.with(|s| {
        for j in 0..6 {
            assert_eq!(
                s.motors[j].timeout_counts, 2000,
                "motor {} RID_TIMEOUT must be 2000",
                j + 1
            );
        }
        assert!(
            !s.tx_log
                .iter()
                .any(|f| f.id == 0x7FF && f.data[2] == 0xAA),
            "RAM-only commissioning must emit 0 flash save frames (0xAA)"
        );
    });

    let max_write_hold = holds
        .iter()
        .filter(|h| h.kind == HoldKind::Write)
        .map(|h| h.hold())
        .max()
        .unwrap_or(Duration::ZERO);
    let samples = sup.tick_samples();
    let window_samples = &samples[samples_before.min(samples.len())..];
    let max_tick_wait_us = window_samples.iter().map(|s| s.wait_us).max().unwrap_or(0);
    let skipped = skipped_releases.load(Ordering::Acquire);
    let max_gap_us = max_release_gap_us.load(Ordering::Acquire);

    println!(
        "[MIT-TIMEOUT OFF-LOCK 6-axis] elapsed_ms={:.2} releases={} skipped_releases={} max_write_hold_us={} max_tick_wait_us={} max_release_gap_us={}",
        elapsed.as_secs_f64() * 1e3,
        ticks_after - ticks_before,
        skipped,
        max_write_hold.as_micros(),
        max_tick_wait_us,
        max_gap_us,
    );

    assert!(
        ticks_after >= ticks_before + 5,
        "50 Hz control loop must keep releasing during off-lock mit_timeout ({ticks_before} -> {ticks_after})"
    );
    assert!(
        max_write_hold <= Duration::from_millis(2),
        "max SupervisorLock::write guard hold MUST stay <= 2 ms (got {max_write_hold:?})"
    );
    assert_eq!(
        skipped, 0,
        "expected 0 skipped 50 Hz releases during mit_timeout (got {skipped}, max_gap_us={max_gap_us})"
    );

    // Verify C-F1 armed refusal still holds on Legacy WS.
    let ep_arm = sup.get_observation().await.epoch_id;
    let arm_ack = common::rpc(&mut ws, gen, ep_arm, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(arm_ack["type"], "command_ack", "{arm_ack}");
    let ep_post = sup.get_observation().await.epoch_id;
    let armed_rej = common::rpc(
        &mut ws,
        gen,
        ep_post,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    assert_eq!(armed_rej["type"], "command_reject", "{armed_rej}");
    assert_eq!(armed_rej["code"], "MIT_TIMEOUT_ARMED", "{armed_rej}");

    stop_flag.store(true, Ordering::Relaxed);
    let _ = feeder_jh.await;
    let _ = ticker_jh.await;
    server_jh.abort();
}
