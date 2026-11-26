//! SG2002 timing simulation and TX coalescing / optional-work deferral / bounded RX recovery tests.
//!
//! Models SG2002 B601-DM timing on a deterministic virtual bus:
//! - TX: 0.5 ms per CAN frame
//! - Separate SPI/IRQ interference model (`mcp251xfd` FIFO 80 preempting `armd-control` FIFO 79)
//! - RX drain: 1.0–1.5 ms per non-empty RX drain pass
//! - Damiao hardware watchdog `TIMEOUT = 2000` (100 ms at 50 µs/count on J1–J6; J7 `TIMEOUT = 0`;
//!   `0xCC` status polls do NOT refresh the watchdog)

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::diagnostics::tick::{
    reset_tx_metrics, take_tx_metrics, thread_usage, LockFreeTickStats, TickSample,
    TxFrameKindCounts, TxFrameKindTotals, UsageMark,
};
use armd::driver::{gripper_drive_from_profile, ArmDriver, DamiaoArmDriver};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{BootId, Capacities, ClientId, DeliveryDecision, Outcome, Submit, TransitionOp};
use armd::rt_core::ops::{CoreOps, ReleaseFixture, VirtualClock};
use armd::rt_core::requests::lease::{OwnerAuthPolicy, TorqueWsOutcome};
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
};
use armd::{ArmConfig, ArmSupervisor, SafetyState, SupervisorInner};

const BOOT: BootId = BootId(0xB601_9001);
const RELEASE_PERIOD: Duration = Duration::from_millis(20);
const MAX_SIM_RELEASE: Duration = Duration::from_millis(15);
const DT: f64 = 0.02;
const TIMEOUT_2000_COUNTS: u32 = 2000;
const MAX_ALLOWED_CTRL_GAP: Duration = Duration::from_millis(40);
const TX_STEP: Duration = Duration::from_micros(500);

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

/// Separate SG2002 SPI/IRQ interference model (`mcp251xfd` IRQ thread at SCHED_FIFO 80 preempting
/// `armd-control` at SCHED_FIFO 79).
struct IrqInterferenceModel {
    rng: SeedRng,
    tick_index: u64,
    tx_in_tick: u32,
    rx_drain_charged_this_tick: bool,
}

impl IrqInterferenceModel {
    fn new(seed: u64) -> Self {
        Self {
            rng: SeedRng::new(seed ^ 0x51A0_F1F0),
            tick_index: 0,
            tx_in_tick: 0,
            rx_drain_charged_this_tick: false,
        }
    }

    fn begin_release(&mut self) {
        self.tick_index = self.tick_index.wrapping_add(1);
        self.tx_in_tick = 0;
        self.rx_drain_charged_this_tick = false;
    }

    /// Per-frame SPI/IRQ preemption penalty on top of the 500 µs base CAN TX step.
    /// Models realistic FIFO-80 preemption (100–280 µs/frame, with periodic SPI IRQ bursts
    /// adding 400–650 µs when TX frame bursts stack up in one release).
    fn on_tx_frame(&mut self) -> Duration {
        self.tx_in_tick = self.tx_in_tick.saturating_add(1);
        let base_irq_us = self.rng.range(100, 260);
        let burst_us = if self.tick_index % 37 == 0 && self.tx_in_tick == 4 {
            self.rng.range(450, 750)
        } else if self.tx_in_tick > 8 {
            // Stacked extra frames (> 8 in one release) trigger additional SPI FIFO IRQ cycles.
            self.rng.range(220, 420)
        } else {
            0
        };
        Duration::from_micros(base_irq_us + burst_us)
    }

    /// Per-frame RX drain cost on SG2002 (~150–215 µs/frame -> 1.05–1.50 ms for a 7-motor RX drain
    /// pass, capped so a full pass models the 1.0–1.5 ms SG2002 RX drain window faithfully without
    /// starving a 1-frame 1 ms legacy handshake poll).
    fn on_rx_frame(&mut self) -> Duration {
        if !self.rx_drain_charged_this_tick {
            self.rx_drain_charged_this_tick = true;
            Duration::from_micros(self.rng.range(160, 220))
        } else {
            Duration::from_micros(self.rng.range(150, 210))
        }
    }
}

#[derive(Clone, Copy)]
struct SimMotor {
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

impl SimMotor {
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
            if now.saturating_duration_since(last) >= limit {
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

struct SgPerfBusState {
    clock: Arc<MockClock>,
    motors: [SimMotor; 7],
    ctrl_modes: [u32; 7],
    irq: IrqInterferenceModel,
    immediate_init: bool,
    max_disables_per_tick: Option<usize>,
    disables_this_tick: usize,
    /// Suppress feedback replies for joints in this bitmask (exercises staleness & recovery).
    suppress_reply_mask: u8,
    /// Per-motor control frame counts within the current release for already-enabled motors
    /// (and all control/enable frames on arm motors J1-J6). Protocol-required J7 pre-enable
    /// 0x307 hold while J7 is still disabled precedes J7 CMD_ENABLE by design.
    release_motor_ctrl_counts: [u32; 7],
    release_tx_total: u32,
    max_motor_ctrl_in_any_release: u32,
    max_tx_frames_in_any_release: u32,
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    delayed: Vec<(Instant, CanFrame)>,
}

impl SgPerfBusState {
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
        if !self.immediate_init && (self.suppress_reply_mask & (1u8 << idx)) != 0 {
            return;
        }
        if self.immediate_init {
            self.rx
                .push_back((frame, Some(CanRxTime::Monotonic(now))));
        } else {
            let ready_at = now + self.motors[idx].reply_delay;
            self.delayed.push((ready_at, frame));
        }
    }

    fn note_motor_ctrl(&mut self, idx: usize) {
        self.release_motor_ctrl_counts[idx] = self.release_motor_ctrl_counts[idx].saturating_add(1);
        if self.release_motor_ctrl_counts[idx] > self.max_motor_ctrl_in_any_release {
            self.max_motor_ctrl_in_any_release = self.release_motor_ctrl_counts[idx];
        }
    }
}

#[derive(Clone)]
struct SgPerfBus(Arc<Mutex<SgPerfBusState>>);

impl SgPerfBus {
    fn new(clock: Arc<MockClock>, seed: u64, reply_delays_ms: [u64; 7]) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| SimMotor {
            id: (i + 1) as u16,
            q: if i == 6 { -0.14 } else { 0.0 },
            v: 0.0,
            temp_c: 31,
            enabled: false,
            fault_state: 0,
            timeout_counts: if i < 6 { TIMEOUT_2000_COUNTS } else { 0 },
            last_ctrl_at: None,
            max_ctrl_gap: Duration::ZERO,
            watchdog_expiries: 0,
            drop_enable_count: 0,
            reply_delay: Duration::from_millis(reply_delays_ms[i]),
            limits: limits[i],
        });
        let mut ctrl_modes = [damiao::CTRL_MODE_POS_VEL; 7];
        ctrl_modes[6] = damiao::CTRL_MODE_POS_FORCE;
        Self(Arc::new(Mutex::new(SgPerfBusState {
            clock,
            motors,
            ctrl_modes,
            irq: IrqInterferenceModel::new(seed),
            immediate_init: true,
            max_disables_per_tick: None,
            disables_this_tick: 0,
            suppress_reply_mask: 0,
            release_motor_ctrl_counts: [0; 7],
            release_tx_total: 0,
            max_motor_ctrl_in_any_release: 0,
            max_tx_frames_in_any_release: 0,
            rx: VecDeque::with_capacity(512),
            delayed: Vec::new(),
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut SgPerfBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn begin_release(&self) {
        self.with(|s| {
            s.disables_this_tick = 0;
            s.release_motor_ctrl_counts = [0; 7];
            s.release_tx_total = 0;
            s.irq.begin_release();
            s.run_watchdogs();
        });
    }

    fn total_expiries(&self) -> usize {
        self.with(|s| {
            s.run_watchdogs();
            s.motors.iter().map(|m| m.watchdog_expiries).sum()
        })
    }

    fn max_arm_ctrl_gap(&self) -> Duration {
        self.with(|s| {
            s.run_watchdogs();
            s.motors[..6]
                .iter()
                .map(|m| m.max_ctrl_gap)
                .max()
                .unwrap_or(Duration::ZERO)
        })
    }
}

impl CanTransport for SgPerfBus {
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
        if !frame.is_extended
            && frame.dlc == 8
            && frame.data == damiao::CMD_DISABLE
            && s.max_disables_per_tick
                .is_some_and(|cap| s.disables_this_tick >= cap)
        {
            return Err(TransportError::WouldBlock);
        }
        if !s.immediate_init {
            let irq_extra = s.irq.on_tx_frame();
            s.clock.advance(TX_STEP + irq_extra);
            s.release_tx_total = s.release_tx_total.saturating_add(1);
            if s.release_tx_total > s.max_tx_frames_in_any_release {
                s.max_tx_frames_in_any_release = s.release_tx_total;
            }
        }
        s.run_watchdogs();
        let now = s.clock.now();

        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let raw = u32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]);
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

        // 0xCC status poll: does NOT refresh Damiao TIMEOUT watchdog.
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
                if !s.immediate_init {
                    s.note_motor_ctrl(idx);
                }
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

        // POS_VEL control frame (0x100 + id)
        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) + 0x100 == frame.id) {
            if !s.immediate_init {
                s.note_motor_ctrl(idx);
            }
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_POS_VEL {
                let target_q = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                if target_q.is_finite() {
                    s.motors[idx].q = target_q;
                    s.motors[idx].v = 0.0;
                }
                s.motors[idx].record_ctrl_refresh(now);
            }
            let fb = s.motors[idx].feedback_frame();
            s.enqueue_reply(idx, fb, now);
            return Ok(());
        }

        // POS_FORCE control frame (0x300 + id)
        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) + 0x300 == frame.id) {
            // While J7 is disabled (`!enabled`), Damiao protocol requires one pre-enable 0x307 hold
            // immediately before `CMD_ENABLE`. Once enabled, count every 0x307 control frame.
            if !s.immediate_init && s.motors[idx].enabled {
                s.note_motor_ctrl(idx);
            }
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_POS_FORCE {
                let target_q = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
                let vel_raw = u16::from_le_bytes([frame.data[4], frame.data[5]]);
                if target_q.is_finite() && vel_raw > 0 {
                    s.motors[idx].q = target_q;
                    s.motors[idx].v = 0.0;
                }
                s.motors[idx].record_ctrl_refresh(now);
            }
            let fb = s.motors[idx].feedback_frame();
            s.enqueue_reply(idx, fb, now);
            return Ok(());
        }

        // MIT control frame (base id)
        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if !s.immediate_init {
                s.note_motor_ctrl(idx);
            }
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
        if let Some(item) = s.rx.pop_front() {
            if !s.immediate_init {
                let rx_cost = s.irq.on_rx_frame();
                if !rx_cost.is_zero() {
                    s.clock.advance(rx_cost);
                    s.run_watchdogs();
                }
            }
            Ok(item)
        } else {
            Err(TransportError::WouldBlock)
        }
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

fn build_sg2002_config() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    cfg.resting_pose = Some(armd::model::RestingPosePolicy {
        positions: Some(vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.14]),
        ..armd::model::RestingPosePolicy::rebot_dm()
    });
    cfg
}

fn build_sg2002_driver(bus: SgPerfBus, cfg: &ArmConfig) -> DamiaoArmDriver<SgPerfBus> {
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

#[derive(Default)]
struct SimStats {
    releases: u64,
    skips: u64,
    max_release: Duration,
    max_tx_in_release: u32,
    by_kind: TxFrameKindTotals,
    total_frames: u64,
    max_recovery_in_release: u32,
}

impl SimStats {
    fn record_release(&mut self, elapsed: Duration) {
        self.releases += 1;
        if elapsed > self.max_release {
            self.max_release = elapsed;
        }
        if elapsed > RELEASE_PERIOD {
            self.skips += 1;
        }
        let tx = take_tx_metrics();
        assert_eq!(
            tx.frames,
            tx.by_kind.total(),
            "per-kind TX counter sum must equal tx.frames (got {:?} vs {})",
            tx.by_kind,
            tx.frames
        );
        self.by_kind.add_counts(tx.by_kind);
        self.total_frames += u64::from(tx.frames);
        if tx.frames > self.max_tx_in_release {
            self.max_tx_in_release = tx.frames;
        }
        if tx.by_kind.recovery > self.max_recovery_in_release {
            self.max_recovery_in_release = tx.by_kind.recovery;
        }
    }
}

/// Measure one simulated executor release including Window 1 + Window 2 virtual time before advancing
/// to the next release slot.
fn step_executor_timed(
    fx: &mut ReleaseFixture<SupervisorInner>,
    clock: &Arc<MockClock>,
    bus: &SgPerfBus,
    stats: &mut SimStats,
) -> armd::rt_core::ops::ReleaseReport {
    reset_tx_metrics();
    bus.begin_release();
    let release_start = clock.now();
    // Model ~1.0 ms control/obs/publish overhead on SG2002 (measured p50 0.82 ms, p99 1.67 ms).
    let work_overhead = Duration::from_micros(900);
    let rep = {
        // Measure work time inside the release before ReleaseFixture advances to `release_deadline`.
        // We wrap `step_with_clock` by capturing the peak clock timestamp observed at output/wake.
        let peak_t = Arc::new(Mutex::new(release_start));
        let peak_ref = peak_t.clone();
        let clk = clock.clone();
        let r = fx.step_with_clock(move || {
            let now = clk.now();
            let mut p = peak_ref.lock().unwrap();
            if now > *p {
                *p = now;
            }
            now
        });
        let peak = *peak_t.lock().unwrap();
        let simulated_work = peak.saturating_duration_since(release_start) + work_overhead;
        stats.record_release(simulated_work);
        r
    };
    rep
}

/// Measure one simulated legacy release.
fn step_legacy_timed(
    inner: &mut SupervisorInner,
    clock: &Arc<MockClock>,
    bus: &SgPerfBus,
    stats: &mut SimStats,
) {
    reset_tx_metrics();
    bus.begin_release();
    let release_start = clock.now();
    let dl = release_start + RELEASE_PERIOD;
    let mut now_fn = || clock.now();
    inner.tick_control_with_clock(DT, &mut now_fn, dl);
    let work_elapsed = clock.now().saturating_duration_since(release_start) + Duration::from_micros(900);
    stats.record_release(work_elapsed);
    let rem = dl.saturating_duration_since(clock.now());
    if !rem.is_zero() {
        clock.advance(rem);
    }
}

#[test]
fn executor_j1_motion_180s_sg2002_sim() {
    let clock = Arc::new(MockClock::new());
    let bus = SgPerfBus::new(clock.clone(), 0x180_ECEC, [1, 2, 2, 1, 3, 2, 2]);
    let cfg = build_sg2002_config();
    let drv = build_sg2002_driver(bus.clone(), &cfg);
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    inner.register_live_connection(1).unwrap();
    let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities { commands: 16, plans: 4, replies: 32 },
        VirtualClock::from_mock_clock((*clock).clone()),
    );

    let mut stats = SimStats::default();

    // Start from stale idle and drop J7's first enable frame to exercise multi-tick enable.
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

    let mut enabled = false;
    for _ in 0..35 {
        let rep = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        if let Some((_, Outcome::Completed)) = rep.transition_ended {
            enabled = true;
            break;
        }
    }
    assert!(enabled, "executor enable must complete cleanly");

    // 180 s equivalent at 50 Hz = 9,000 releases of continuous J1 motion.
    for tick in 0u64..9_000 {
        let phase = (tick as f32) * 0.02 * std::f32::consts::TAU * 0.12;
        let j1_target = 0.35 * phase.sin();
        let mut target = fx.core.commanded_positions.clone();
        target[0] = j1_target;
        fx.core.servo_target = Some(target);

        // Inject brief 3-tick feedback suppression on J1..J6 every 1,500 releases to exercise
        // bounded RX staleness recovery without bus-loss fault (< 250 ms).
        let cycle_pos = tick % 1_500;
        if cycle_pos == 750 || cycle_pos == 751 || cycle_pos == 752 {
            bus.with(|s| s.suppress_reply_mask = 0x3F);
        } else {
            bus.with(|s| s.suppress_reply_mask = 0);
        }

        let _ = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        if tick % 100 == 0 {
            let _ = broker.try_poll(|_, _| DeliveryDecision::Delivered);
        }
    }

    let max_per_motor = bus.with(|s| s.max_motor_ctrl_in_any_release);
    let max_gap = bus.max_arm_ctrl_gap();
    let expiries = bus.total_expiries();

    println!(
        "[PERF-SIM executor-j1-180s] releases={} skips={} max_release_ms={:.2} max_tx_per_rel={} max_per_motor_ctrl={} max_gap_ms={:.2} expiries={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_tx_in_release,
        max_per_motor,
        max_gap.as_secs_f64() * 1e3,
        expiries,
        stats.by_kind,
    );

    assert_eq!(expiries, 0, "TIMEOUT=2000 must have 0 expiries over 180 s J1 motion");
    assert!(
        max_gap <= MAX_ALLOWED_CTRL_GAP,
        "max arm watchdog gap {max_gap:?} exceeded 40 ms"
    );
    assert!(
        max_per_motor <= 1,
        "duplicate control frames sent to the same motor within one release: max={max_per_motor}"
    );
    assert_eq!(stats.skips, 0, "expected 0 skipped releases");
    assert!(
        stats.max_release <= MAX_SIM_RELEASE,
        "max simulated release {:?} exceeded 15 ms",
        stats.max_release
    );
    assert!(
        stats.max_recovery_in_release <= 2,
        "RX recovery frames per release must be bounded <= 2, got {}",
        stats.max_recovery_in_release
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_j1_motion_180s_sg2002_sim() {
    let clock = Arc::new(MockClock::new());
    let bus = SgPerfBus::new(clock.clone(), 0x180_1E6A, [1, 2, 2, 1, 3, 2, 2]);
    let cfg = build_sg2002_config();
    let drv = build_sg2002_driver(bus.clone(), &cfg);
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let conn = sup.try_claim_ws_slot().unwrap();
    let conn_id = conn.id();
    sup.register_ws_connection_for_slot(conn_id).await.unwrap();
    let lease_gen = {
        let mut inner = sup.inner().write().await;
        inner.acquire_lease(&format!("connection:{conn_id}")).unwrap()
    };

    let mut stats = SimStats::default();
    bus.with(|s| s.motors[6].drop_enable_count = 1);
    clock.advance(Duration::from_millis(120));

    let (ep, mut stamp) = {
        let inner = sup.inner().read().await;
        (inner.safety.epoch_id(), inner.current_admission_stamp())
    };
    stamp.request_id = 1;
    let sup_ws = sup.clone();
    let conn_clone = conn.clone();
    let ws_task = tokio::spawn(async move {
        sup_ws
            .torque_ws_with_deadline(
                conn_clone,
                lease_gen,
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
        if sup.inner().read().await.torque_transition_pending() || ws_task.is_finished() {
            break;
        }
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    for _ in 0..40 {
        if ws_task.is_finished() {
            break;
        }
        {
            let mut inner = sup.inner().write().await;
            step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
        }
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(1)).await;
    }
    let outcome = ws_task.await.unwrap();
    assert!(
        matches!(outcome, TorqueWsOutcome::Accepted { .. }),
        "legacy enable must succeed: {outcome:?}"
    );

    let mut inner = sup.inner().write().await;
    inner.lease_enabled = false;
    for tick in 0u64..9_000 {
        let phase = (tick as f32) * 0.02 * std::f32::consts::TAU * 0.12;
        let j1_target = 0.35 * phase.sin();
        let mut target = inner.commanded_positions.clone();
        target[0] = j1_target;
        inner.servo_target = Some(target);

        let cycle_pos = tick % 1_500;
        if cycle_pos == 750 || cycle_pos == 751 || cycle_pos == 752 {
            bus.with(|s| s.suppress_reply_mask = 0x3F);
        } else {
            bus.with(|s| s.suppress_reply_mask = 0);
        }

        step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
    }

    let max_per_motor = bus.with(|s| s.max_motor_ctrl_in_any_release);
    let max_gap = bus.max_arm_ctrl_gap();
    let expiries = bus.total_expiries();

    println!(
        "[PERF-SIM legacy-j1-180s] releases={} skips={} max_release_ms={:.2} max_tx_per_rel={} max_per_motor_ctrl={} max_gap_ms={:.2} expiries={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_tx_in_release,
        max_per_motor,
        max_gap.as_secs_f64() * 1e3,
        expiries,
        stats.by_kind,
    );

    assert_eq!(expiries, 0, "legacy TIMEOUT=2000 must have 0 expiries over 180 s J1 motion");
    assert!(max_gap <= MAX_ALLOWED_CTRL_GAP, "legacy max gap {max_gap:?} > 40 ms");
    assert!(max_per_motor <= 1, "legacy duplicate per-motor ctrl frame: {max_per_motor}");
    assert_eq!(stats.skips, 0, "legacy expected 0 skips");
    assert!(
        stats.max_release <= MAX_SIM_RELEASE,
        "legacy max simulated release {:?} exceeded 15 ms",
        stats.max_release
    );
}

#[test]
fn executor_enable_hold_rest_disable_cycle_sg2002_sim() {
    let clock = Arc::new(MockClock::new());
    let bus = SgPerfBus::new(clock.clone(), 0xC1C1_ECEC, [1, 2, 2, 1, 3, 2, 2]);
    bus.with(|s| s.max_disables_per_tick = Some(2));
    let cfg = build_sg2002_config();
    let drv = build_sg2002_driver(bus.clone(), &cfg);
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    inner.register_live_connection(1).unwrap();
    let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities { commands: 64, plans: 8, replies: 128 },
        VirtualClock::from_mock_clock((*clock).clone()),
    );

    let mut stats = SimStats::default();
    let mut seq = 1u64;

    for cycle in 0..10u64 {
        let _ = broker.try_poll(|_, _| DeliveryDecision::Delivered);

        // Stale disarmed idle.
        for _ in 0..8 {
            let _ = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        }

        // Enable (drop J7 first enable on odd cycles to force multi-tick enable).
        if cycle % 2 == 1 {
            bus.with(|s| s.motors[6].drop_enable_count = 1);
        }
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

        let mut en_ok = false;
        for _ in 0..35 {
            let rep = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
            if let Some((_, Outcome::Completed)) = rep.transition_ended {
                en_ok = true;
                break;
            }
        }
        assert!(en_ok, "cycle {cycle}: executor enable failed");

        // Hold for 100 ticks (2 s).
        for _ in 0..100 {
            let _ = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        }

        // Move away and return to rest pose, then settle.
        {
            let mut target = fx.core.commanded_positions.clone();
            target[0] = 0.20;
            fx.core.servo_target = Some(target);
        }
        for _ in 0..8 {
            let _ = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        }
        {
            let rest = fx.core.config.resting_positions();
            fx.core.servo_target = Some(rest.to_vec());
        }
        for _ in 0..16 {
            let _ = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        }
        fx.core.stop_motion();
        for _ in 0..6 {
            let _ = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
        }

        // Multi-tick rest disable.
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

        let mut dis_outcome = None;
        for _ in 0..60 {
            let rep = step_executor_timed(&mut fx, &clock, &bus, &mut stats);
            if let Some((_, o)) = rep.transition_ended {
                dis_outcome = Some(o);
                break;
            }
        }
        assert_eq!(
            dis_outcome,
            Some(Outcome::Completed),
            "cycle {cycle}: executor rest-disable failed (state={:?}, torqued={})",
            fx.core.safety.state(),
            fx.core.is_torqued
        );
        assert!(!fx.core.is_torqued);
        assert!(matches!(fx.core.safety.state(), SafetyState::Idle));
    }

    let max_per_motor = bus.with(|s| s.max_motor_ctrl_in_any_release);
    let max_gap = bus.max_arm_ctrl_gap();
    let expiries = bus.total_expiries();

    println!(
        "[PERF-SIM executor-cycle-10x] releases={} skips={} max_release_ms={:.2} max_tx_per_rel={} max_per_motor_ctrl={} max_gap_ms={:.2} expiries={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_tx_in_release,
        max_per_motor,
        max_gap.as_secs_f64() * 1e3,
        expiries,
        stats.by_kind,
    );

    assert_eq!(expiries, 0);
    assert!(max_gap <= MAX_ALLOWED_CTRL_GAP, "max gap {max_gap:?} > 40 ms");
    assert!(max_per_motor <= 1, "duplicate per-motor ctrl frame: {max_per_motor}");
    assert_eq!(stats.skips, 0);
    assert!(
        stats.max_release <= MAX_SIM_RELEASE,
        "max simulated release {:?} exceeded 15 ms",
        stats.max_release
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_enable_hold_rest_disable_cycle_sg2002_sim() {
    let clock = Arc::new(MockClock::new());
    let bus = SgPerfBus::new(clock.clone(), 0xC1C1_1E6A, [1, 2, 2, 1, 3, 2, 2]);
    bus.with(|s| s.max_disables_per_tick = Some(2));
    let cfg = build_sg2002_config();
    let drv = build_sg2002_driver(bus.clone(), &cfg);
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let conn = sup.try_claim_ws_slot().unwrap();
    let conn_id = conn.id();
    sup.register_ws_connection_for_slot(conn_id).await.unwrap();
    let lease_gen = {
        let mut inner = sup.inner().write().await;
        inner.acquire_lease(&format!("connection:{conn_id}")).unwrap()
    };

    let mut stats = SimStats::default();
    let mut req_id = 1u64;

    for cycle in 0..10u64 {
        // Stale idle.
        for _ in 0..8 {
            let mut inner = sup.inner().write().await;
            step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
        }

        if cycle % 2 == 1 {
            bus.with(|s| s.motors[6].drop_enable_count = 1);
        }
        let (ep, mut stamp) = {
            let inner = sup.inner().read().await;
            (inner.safety.epoch_id(), inner.current_admission_stamp())
        };
        let req_en = req_id;
        stamp.request_id = req_en;
        req_id += 1;
        let sup_ws = sup.clone();
        let conn_en = conn.clone();
        let ws_en = tokio::spawn(async move {
            sup_ws
                .torque_ws_with_deadline(
                    conn_en,
                    lease_gen,
                    Some(ep),
                    true,
                    req_en,
                    Ok(true),
                    OwnerAuthPolicy::None,
                    OperatorGrant::unenforced(),
                    stamp,
                    Duration::from_secs(120),
                )
                .await
                .unwrap()
        });
        for _ in 0..200 {
            if sup.inner().read().await.torque_transition_pending() || ws_en.is_finished() {
                break;
            }
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        for _ in 0..40 {
            if ws_en.is_finished() {
                break;
            }
            {
                let mut inner = sup.inner().write().await;
                step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
            }
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        assert!(
            matches!(ws_en.await.unwrap(), TorqueWsOutcome::Accepted { .. }),
            "cycle {cycle}: legacy enable failed"
        );

        // Hold for 100 ticks (2 s) + settle at rest.
        {
            let mut inner = sup.inner().write().await;
            for _ in 0..100 {
                step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
            }
            let rest = inner.config.resting_positions();
            inner.servo_target = Some(rest.to_vec());
            for _ in 0..5 {
                step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
            }
            inner.stop_motion();
            for _ in 0..5 {
                step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
            }
        }

        // Rest disable via legacy WS guarded path.
        let (ep2, mut stamp2) = {
            let inner = sup.inner().read().await;
            (inner.safety.epoch_id(), inner.current_admission_stamp())
        };
        let req_dis = req_id;
        stamp2.request_id = req_dis;
        req_id += 1;
        let sup_ws2 = sup.clone();
        let conn_dis = conn.clone();
        let ws_dis = tokio::spawn(async move {
            sup_ws2
                .torque_ws_with_deadline(
                    conn_dis,
                    lease_gen,
                    Some(ep2),
                    true,
                    req_dis,
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
            if sup.inner().read().await.torque_transition_pending() || ws_dis.is_finished() {
                break;
            }
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        for _ in 0..60 {
            if ws_dis.is_finished() {
                break;
            }
            {
                let mut inner = sup.inner().write().await;
                step_legacy_timed(&mut inner, &clock, &bus, &mut stats);
            }
            tokio::task::yield_now().await;
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        let dis_res = ws_dis.await.unwrap();
        assert!(
            matches!(dis_res, TorqueWsOutcome::Accepted { .. }),
            "cycle {cycle}: legacy rest-disable failed: {dis_res:?}"
        );
    }

    let max_per_motor = bus.with(|s| s.max_motor_ctrl_in_any_release);
    let max_gap = bus.max_arm_ctrl_gap();
    let expiries = bus.total_expiries();

    println!(
        "[PERF-SIM legacy-cycle-10x] releases={} skips={} max_release_ms={:.2} max_tx_per_rel={} max_per_motor_ctrl={} max_gap_ms={:.2} expiries={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_tx_in_release,
        max_per_motor,
        max_gap.as_secs_f64() * 1e3,
        expiries,
        stats.by_kind,
    );

    assert_eq!(expiries, 0);
    assert!(max_gap <= MAX_ALLOWED_CTRL_GAP, "legacy max gap {max_gap:?} > 40 ms");
    assert!(max_per_motor <= 1, "legacy duplicate per-motor ctrl frame: {max_per_motor}");
    assert_eq!(stats.skips, 0);
    assert!(
        stats.max_release <= MAX_SIM_RELEASE,
        "legacy max simulated release {:?} exceeded 15 ms",
        stats.max_release
    );
}

#[test]
fn coalesced_fallback_optional_deferral_and_bounded_rx_recovery() {
    // 1. Bounded per-release RX staleness recovery (<= 2 per release, rotating round-robin)
    //    + optional 0xCC deferral when remaining release budget < 6 ms.
    let clock = Arc::new(MockClock::new());
    let bus = SgPerfBus::new(clock.clone(), 0xFEED_1234, [1, 1, 1, 1, 1, 1, 1]);
    let cfg = build_sg2002_config();
    let mut drv = build_sg2002_driver(bus.clone(), &cfg);
    drv.set_torque(true).unwrap();
    // Drain enable replies so all joints start fresh.
    clock.advance(Duration::from_millis(5));
    drv.read_joint_states().unwrap();

    // Advance 50 ms so all 6 arm joints cross the 45 ms recovery threshold.
    clock.advance(Duration::from_millis(50));
    bus.with(|s| s.suppress_reply_mask = 0x7F);

    let mut seen_recovered = [false; 6];
    for rel in 1u64..=3 {
        reset_tx_metrics();
        bus.begin_release();
        drv.begin_control_release(rel);
        // Full 20 ms release -> tx_release_deadline = now + 18 ms (>= 4 ms extra margin).
        drv.set_tx_release_deadline(Some(clock.now() + Duration::from_millis(18)));
        let mut states = Vec::with_capacity(7);
        drv.read_joint_states_budgeted(&mut states, clock.now() + Duration::from_millis(5), 64)
            .unwrap();
        let tx = take_tx_metrics();
        assert!(
            tx.by_kind.recovery <= 2,
            "release {rel}: recovery frames must be bounded <= 2, got {}",
            tx.by_kind.recovery
        );
        let rec = drv.take_recovery_writes();
        assert_eq!(rec.len(), tx.by_kind.recovery as usize);
        for j in rec {
            if j < 6 {
                seen_recovered[j] = true;
            }
        }
    }
    assert!(
        seen_recovered.iter().all(|&v| v),
        "round-robin recovery over 3 releases must cover all 6 arm joints: {seen_recovered:?}"
    );

    // Tight release budget (< 6 ms remaining -> tx_release_deadline = now + 2 ms): optional 0xCC
    // and recovery must defer cleanly (0 frames sent).
    clock.advance(Duration::from_millis(60));
    reset_tx_metrics();
    bus.begin_release();
    drv.begin_control_release(4);
    drv.set_tx_release_deadline(Some(clock.now() + Duration::from_millis(2)));
    let mut states = Vec::with_capacity(7);
    drv.read_joint_states_budgeted(&mut states, clock.now() + Duration::from_millis(2), 64)
        .unwrap();
    let tight_tx = take_tx_metrics();
    assert_eq!(tight_tx.by_kind.refresh_0xcc, 0, "optional 0xCC must defer when remaining < 6 ms");
    assert_eq!(tight_tx.by_kind.recovery, 0, "optional RX recovery must defer when remaining < 6 ms");

    // Watchdog fallback is NEVER deferred by normal output absence: after >= 18 ms with no normal
    // output in release 5, `service_release_watchdog_fallback` emits UrgentHold for unserviced motors.
    bus.with(|s| s.suppress_reply_mask = 0);
    clock.advance(Duration::from_millis(20));
    reset_tx_metrics();
    bus.begin_release();
    drv.begin_control_release(5);
    drv.set_tx_release_deadline(Some(clock.now() + Duration::from_millis(18)));
    drv.service_release_watchdog_fallback();
    let fb_tx = take_tx_metrics();
    assert_eq!(fb_tx.by_kind.urgent_hold, 6, "all 6 unserviced arm motors must receive urgent_hold");
    assert_eq!(fb_tx.by_kind.normal, 0);

    // Calling fallback a second time in the SAME release emits 0 duplicate holds (coalesced!).
    reset_tx_metrics();
    drv.service_release_watchdog_fallback();
    let fb_dup = take_tx_metrics();
    assert_eq!(fb_dup.by_kind.urgent_hold, 0, "coalescing must prevent duplicate urgent holds in same release");

    // 2. Verify LockFreeTickStats + TickSnapshot preserve per-kind TX counts cleanly.
    let lf = LockFreeTickStats::new();
    let sample_counts = TxFrameKindCounts {
        normal: 6,
        hold: 1,
        urgent_hold: 0,
        refresh_0xcc: 1,
        recovery: 2,
        enable_disable: 0,
    };
    reset_tx_metrics();
    for _ in 0..6 {
        armd::diagnostics::tick::record_tx_attempt(armd::diagnostics::tick::TxAttemptKind::Sent, 500_000, 100_000);
        armd::diagnostics::tick::record_tx_frame_kind(armd::diagnostics::tick::TxFrameKind::Normal);
    }
    armd::diagnostics::tick::record_tx_attempt(armd::diagnostics::tick::TxAttemptKind::Sent, 500_000, 100_000);
    armd::diagnostics::tick::record_tx_frame_kind(armd::diagnostics::tick::TxFrameKind::Hold);
    armd::diagnostics::tick::record_tx_attempt(armd::diagnostics::tick::TxAttemptKind::Sent, 500_000, 100_000);
    armd::diagnostics::tick::record_tx_frame_kind(armd::diagnostics::tick::TxFrameKind::Refresh0xCc);
    for _ in 0..2 {
        armd::diagnostics::tick::record_tx_attempt(armd::diagnostics::tick::TxAttemptKind::Sent, 500_000, 100_000);
        armd::diagnostics::tick::record_tx_frame_kind(armd::diagnostics::tick::TxFrameKind::Recovery);
    }
    let start = UsageMark::default();
    let end = UsageMark::now();
    let usage = thread_usage(&start, &start, &end);
    assert_eq!(usage.tx_by_kind, sample_counts);
    assert_eq!(usage.tx_frames, 10);
    lf.record(&TickSample { tick_us: 8_000, release_us: 8_500, usage, ..Default::default() });
    let snap = lf.snapshot();
    assert_eq!(
        snap.tx.by_kind_total,
        TxFrameKindTotals {
            normal: 6,
            hold: 1,
            urgent_hold: 0,
            refresh_0xcc: 1,
            recovery: 2,
            enable_disable: 0,
        }
    );
    assert_eq!(snap.tx.since_boot_by_kind, snap.tx.by_kind_total);
}
