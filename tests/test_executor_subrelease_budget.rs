//! SG2002 timing simulation and regression tests for executor sub-release wake budget guarding.
//!
//! Verifies:
//! 1. Sub-release wakes (sleep + poll) run only if the release can complete before
//!    `release_deadline - margin` (default `margin = 4 ms` on 50 Hz / 20 ms SG2002 releases,
//!    configurable via `ExecutorConfig::sub_release_margin`, `ReleaseFixture::sub_release_margin`,
//!    and `ARMD_SUB_RELEASE_MARGIN_MS`).
//! 2. At most `MAX_SERVICE_WAKES == 1` sub-release wake runs per release.
//! 3. Stale Step-2 `urgent_wake` is refreshed after Step 5 control/hold TX so already-refreshed
//!    motors never trigger a spurious Window-2 sleep + rest-gate probe pass.
//! 4. Across 20 enable -> hold -> immediate rest-disable cycles and J1 motion with stops:
//!    - max release <= 16 ms;
//!    - 0 skipped releases;
//!    - arm watchdog gap <= 40 ms with 0 expiries (`TIMEOUT = 2000`);
//!    - every transition (FRESHEN, readback, enable confirm, multi-tick rest-disable) completes
//!      within its deadline.
//! 5. Positive control: without the sub-release guard, the exact same SG2002 transition workload
//!    reproduces a > 20 ms release overrun and skipped release.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::AllocCounts;
use armd::diagnostics::tick::{reset_tx_metrics, take_tx_metrics, TxFrameKindTotals};
use armd::driver::{
    gripper_drive_from_profile, ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities,
    EnableState, JointTarget, TorqueOutcome, TorqueProgress, DAMIAO_TX_RELEASE_PUBLISH_MARGIN,
};
use armd::model::JointState;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    Authority, BootId, Capacities, ClientId, DeliveryDecision, Outcome, Submit, TransitionOp,
};
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks};
use armd::rt_core::ops::{
    parse_sub_release_margin_ms, CoreOps, ReleaseEngine, ReleaseFixture, SubReleaseBudget,
    SubReleaseWakeKind, VirtualClock, DEFAULT_SUB_RELEASE_MARGIN, MAX_SERVICE_WAKES,
    SUB_RELEASE_SHADOW_MARGIN,
};
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
};
use armd::{ArmConfig, ArmSupervisor, SafetyState, SupervisorInner};

const BOOT: BootId = BootId(0xB601_5B01);
const RELEASE_PERIOD: Duration = Duration::from_millis(20);
const MAX_GUARDED_RELEASE: Duration = Duration::from_millis(16);
const DT: f64 = 0.02;
const TIMEOUT_2000_COUNTS: u32 = 2000;
const MAX_ALLOWED_CTRL_GAP: Duration = Duration::from_millis(40);
const TX_STEP: Duration = Duration::from_micros(500);
/// Measured SG2002 Window-1 non-CAN control/obs/publish + shadow overhead (~1.0 ms).
const SG2002_CPU_OVERHEAD: Duration = Duration::from_micros(950);

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

/// SG2002 SPI/IRQ interference model (`mcp251xfd` FIFO 80 preempting `armd-control` FIFO 79):
/// TX 0.5 ms/frame + IRQ interference (100-280 us/frame, bursts up to 750 us), RX 1.0-1.5 ms/pass.
struct IrqInterferenceModel {
    rng: SeedRng,
    tick_index: u64,
    tx_in_tick: u32,
}

impl IrqInterferenceModel {
    fn new(seed: u64) -> Self {
        Self {
            rng: SeedRng::new(seed ^ 0x51A0_F1F0),
            tick_index: 0,
            tx_in_tick: 0,
        }
    }

    fn begin_release(&mut self) {
        self.tick_index = self.tick_index.wrapping_add(1);
        self.tx_in_tick = 0;
    }

    fn on_tx_frame(&mut self) -> Duration {
        self.tx_in_tick = self.tx_in_tick.saturating_add(1);
        let base_irq_us = self.rng.range(110, 270);
        let burst_us = if self.tick_index.is_multiple_of(31) && self.tx_in_tick == 4 {
            self.rng.range(450, 750)
        } else if self.tx_in_tick > 8 {
            self.rng.range(240, 440)
        } else {
            0
        };
        Duration::from_micros(base_irq_us + burst_us)
    }

    fn on_rx_frame(&mut self) -> Duration {
        // ~155-215 us/frame -> ~1.08-1.50 ms for a 7-motor RX drain pass.
        Duration::from_micros(self.rng.range(155, 215))
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

struct SgBusState {
    clock: Arc<MockClock>,
    motors: [SimMotor; 7],
    ctrl_modes: [u32; 7],
    irq: IrqInterferenceModel,
    immediate_init: bool,
    max_disables_per_tick: Option<usize>,
    disables_this_tick: usize,
    suppress_reply_mask: u8,
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    delayed: Vec<(Instant, CanFrame)>,
}

impl SgBusState {
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
}

#[derive(Clone)]
struct SgBus(Arc<Mutex<SgBusState>>);

impl SgBus {
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
        Self(Arc::new(Mutex::new(SgBusState {
            clock,
            motors,
            ctrl_modes,
            irq: IrqInterferenceModel::new(seed),
            immediate_init: true,
            max_disables_per_tick: None,
            disables_this_tick: 0,
            suppress_reply_mask: 0,
            rx: VecDeque::with_capacity(512),
            delayed: Vec::new(),
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut SgBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn begin_release(&self) {
        self.with(|s| {
            s.disables_this_tick = 0;
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

impl CanTransport for SgBus {
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

        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) + 0x100 == frame.id) {
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_POS_VEL {
                let target_q =
                    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
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

        if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) + 0x300 == frame.id) {
            if s.motors[idx].enabled && s.ctrl_modes[idx] == damiao::CTRL_MODE_POS_FORCE {
                let target_q =
                    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
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
                s.clock.advance(rx_cost);
                s.run_watchdogs();
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

fn build_sg2002_driver(bus: SgBus, cfg: &ArmConfig) -> DamiaoArmDriver<SgBus> {
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
    assert_eq!(
        drv.check_mapping_ranges().status,
        armd::driver::RangeCheckStatus::Passed
    );
    bus.with(|s| s.immediate_init = false);
    drv
}

#[derive(Default)]
struct SimStats {
    releases: u64,
    skips: u64,
    over_16ms: u64,
    max_release: Duration,
    max_tx_in_release: u32,
    max_wakes_in_release: u32,
    total_wakes: u64,
    by_kind: TxFrameKindTotals,
}

impl SimStats {
    fn record(&mut self, elapsed: Duration, wakes: u32, skipped_periods: u64) {
        self.releases += 1;
        if elapsed > self.max_release {
            self.max_release = elapsed;
        }
        if elapsed > MAX_GUARDED_RELEASE {
            self.over_16ms += 1;
        }
        self.skips += skipped_periods;
        if wakes > self.max_wakes_in_release {
            self.max_wakes_in_release = wakes;
        }
        self.total_wakes += u64::from(wakes);
        let tx = take_tx_metrics();
        self.by_kind.add_counts(tx.by_kind);
        if tx.frames > self.max_tx_in_release {
            self.max_tx_in_release = tx.frames;
        }
    }
}

/// Real periodic release scheduler shared identically by guarded and unguarded simulations.
/// Counts skips directly from periodic deadline advancement (`while clock.now() > next_release_at`)
/// rather than deriving skips from a separate elapsed-duration heuristic.
struct PeriodicScheduler {
    next_release_at: Instant,
    period: Duration,
}

impl PeriodicScheduler {
    fn new(start: Instant, period: Duration) -> Self {
        Self {
            next_release_at: start,
            period,
        }
    }

    fn resync(&mut self, now: Instant) {
        self.next_release_at = now;
    }

    fn finish_release(&mut self, clock: &Arc<MockClock>) -> u64 {
        let mut next = self.next_release_at + self.period;
        let mut skipped = 0u64;
        while clock.now() > next {
            skipped += 1;
            next += self.period;
        }
        let rem = next.saturating_duration_since(clock.now());
        if !rem.is_zero() {
            clock.advance(rem);
        }
        self.next_release_at = next;
        skipped
    }
}

#[derive(Clone, Copy)]
enum Window2Policy {
    /// Production guarded policy (`SubReleaseBudget` + `MAX_SERVICE_WAKES = 1` + post-Step-5 refresh).
    Guarded(Option<Duration>),
    /// Pre-fix unguarded policy on the same `ReleaseEngine`, `SgBus`, and `PeriodicScheduler`:
    /// stale Step-2 `urgent_wake` + `MAX_SERVICE_WAKES = 8` + unmargined `release_deadline` +
    /// 2 ms urgent cutoff.
    UnguardedPreFix,
}

fn step_executor_with_policy(
    core: &mut SupervisorInner,
    engine: &mut ReleaseEngine,
    clock: &Arc<MockClock>,
    bus: &SgBus,
    sched: &mut PeriodicScheduler,
    policy: Window2Policy,
    stats: &mut SimStats,
) -> armd::rt_core::ops::ReleaseReport {
    reset_tx_metrics();
    bus.begin_release();
    let release_start = clock.now();
    let release_deadline = release_start + RELEASE_PERIOD;
    let next_gen = engine.releases() + 1;
    let mut now_fn = || clock.now();

    let pre_step5_urgent_wake = if matches!(policy, Window2Policy::UnguardedPreFix)
        && engine.transition().is_some()
    {
        core.post_output_urgent_wake(true, clock.now(), None)
    } else {
        None
    };

    let mut report = engine.step(core, &mut now_fn, DT, release_deadline, next_gen);

    match policy {
        Window2Policy::Guarded(margin_override) => {
            let margin = margin_override
                .unwrap_or_else(|| SubReleaseBudget::default_margin_for_period(RELEASE_PERIOD));
            let wake_budget =
                SubReleaseBudget::for_release(release_start, release_deadline, RELEASE_PERIOD, margin);
            if report.wakes < MAX_SERVICE_WAKES {
                if let Some((wake, kind)) = wake_budget.admit_next_wake(
                    clock.now(),
                    engine.has_pending_urgent(core),
                    &report.poll,
                ) {
                    let now = clock.now();
                    let urgent_stop_pending =
                        kind == SubReleaseWakeKind::UrgentStop || engine.has_pending_urgent(core);
                    let exec_cutoff = if urgent_stop_pending {
                        wake_budget.urgent_stop_cutoff
                    } else {
                        wake_budget.transition_start_cutoff
                    };
                    let target_wake = if urgent_stop_pending { now } else { wake };
                    if now < exec_cutoff && target_wake <= exec_cutoff {
                        let target = target_wake.min(wake_budget.wake_deadline);
                        if target > now {
                            clock.advance(target.saturating_duration_since(now));
                        }
                        let post_now = clock.now();
                        let urgent_stop_now =
                            urgent_stop_pending || engine.has_pending_urgent(core);
                        if let Some(allow_transition_poll) =
                            wake_budget.post_sleep_allow_transition_poll(post_now, urgent_stop_now)
                        {
                            report.wakes += 1;
                            let _ = engine.service_wake_with_clock_opts(
                                core,
                                &mut now_fn,
                                wake_budget.poll_deadline,
                                next_gen,
                                allow_transition_poll,
                                &mut report,
                            );
                        }
                    }
                }
            }
        }
        Window2Policy::UnguardedPreFix => {
            if pre_step5_urgent_wake.is_some() {
                report.poll.urgent_wake = pre_step5_urgent_wake;
            }
            let optional_cutoff = release_deadline
                .checked_sub(Duration::from_millis(9))
                .unwrap_or(release_start);
            let urgent_cutoff = release_deadline
                .checked_sub(DAMIAO_TX_RELEASE_PUBLISH_MARGIN)
                .unwrap_or(release_start);
            let mut next = report
                .poll
                .admitted_wake_before(optional_cutoff, urgent_cutoff);
            while let Some((wake, is_urgent)) = next {
                if report.wakes >= 8 {
                    break;
                }
                let exec_cutoff = if is_urgent {
                    urgent_cutoff
                } else {
                    optional_cutoff
                };
                if clock.now() >= exec_cutoff || wake > exec_cutoff {
                    break;
                }
                if wake > clock.now() {
                    clock.advance(wake.saturating_duration_since(clock.now()));
                }
                report.wakes += 1;
                let _ = engine.service_wake_with_clock(
                    core,
                    &mut now_fn,
                    release_deadline,
                    next_gen,
                    &mut report,
                );
                next = report
                    .poll
                    .admitted_wake_before(optional_cutoff, urgent_cutoff);
            }
        }
    }

    // Account for measured SG2002 control-loop CPU + publish + shadow-flush overhead in the
    // real mock clock before the periodic scheduler advances to the next release boundary.
    clock.advance(SG2002_CPU_OVERHEAD);
    let elapsed = clock.now().saturating_duration_since(release_start);
    let skipped = sched.finish_release(clock);
    stats.record(elapsed, report.wakes, skipped);
    report
}

fn step_executor_guarded(
    fx: &mut ReleaseFixture<SupervisorInner>,
    clock: &Arc<MockClock>,
    bus: &SgBus,
    stats: &mut SimStats,
) -> armd::rt_core::ops::ReleaseReport {
    reset_tx_metrics();
    bus.begin_release();
    let release_start = clock.now();
    let peak_t = Arc::new(Mutex::new(release_start));
    let peak_ref = peak_t.clone();
    let clk = clock.clone();
    let rep = fx.step_with_clock(move || {
        let now = clk.now();
        let mut p = peak_ref.lock().unwrap();
        if now > *p {
            *p = now;
        }
        now
    });
    let peak = *peak_t.lock().unwrap();
    let simulated_work = peak.saturating_duration_since(release_start) + SG2002_CPU_OVERHEAD;
    let skipped = u64::from(simulated_work > RELEASE_PERIOD);
    stats.record(simulated_work, rep.wakes, skipped);
    rep
}

struct CycleSimOutcome {
    stats: SimStats,
    max_gap: Duration,
    expiries: usize,
    max_en_ticks: u32,
    max_dis_ticks: u32,
}

/// Runs the exact same SG2002 enable -> hold -> immediate rest-disable workload on one shared
/// `ReleaseEngine` + `PeriodicScheduler`, varying ONLY the Window-2 wake guard policy.
fn run_enable_hold_rest_disable_workload(
    seed: u64,
    cycles: u64,
    guarded: bool,
    margin_override: Option<Duration>,
) -> CycleSimOutcome {
    let clock = Arc::new(MockClock::new());
    let bus = SgBus::new(clock.clone(), seed, [1, 2, 2, 1, 3, 2, 2]);
    bus.with(|s| s.max_disables_per_tick = Some(2));
    let cfg = build_sg2002_config();
    let drv = build_sg2002_driver(bus.clone(), &cfg);
    let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    core.register_live_connection(1).unwrap();
    core.set_boot_id(BOOT);

    let (mut engine, mut broker, _tw, _stop) = ReleaseEngine::new(
        BOOT,
        Capacities {
            commands: 64,
            plans: 8,
            replies: 128,
        },
    );
    core.attach_closed_bitset(broker.closed_bitset());

    let policy = if guarded {
        Window2Policy::Guarded(margin_override)
    } else {
        Window2Policy::UnguardedPreFix
    };
    let mut sched = PeriodicScheduler::new(clock.now(), RELEASE_PERIOD);
    let mut stats = SimStats::default();
    let mut seq = 1u64;
    let mut max_en_ticks = 0u32;
    let mut max_dis_ticks = 0u32;

    for cycle in 0..cycles {
        let _ = broker.try_poll(|_, _| DeliveryDecision::Delivered);

        // Exercise stale idle -> FRESHEN on every 4th cycle.
        if cycle % 4 == 0 {
            clock.advance(Duration::from_millis(120));
            sched.resync(clock.now());
        } else {
            for _ in 0..5 {
                let _ = step_executor_with_policy(
                    &mut core,
                    &mut engine,
                    &clock,
                    &bus,
                    &mut sched,
                    policy,
                    &mut stats,
                );
            }
        }

        // Drop enable frames on odd/every-3rd cycles to force multi-tick enable retries.
        if cycle % 2 == 1 {
            bus.with(|s| s.motors[6].drop_enable_count = 1);
        }
        if cycle % 3 == 0 {
            bus.with(|s| s.motors[0].drop_enable_count = 1);
        }

        let auth = core.authority(&engine.ctx(clock.now()));
        broker
            .submit(
                ClientId(1),
                seq,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        seq += 1;

        let mut en_ok = false;
        let mut en_ticks = 0u32;
        for _ in 0..35 {
            en_ticks += 1;
            let rep = step_executor_with_policy(
                &mut core,
                &mut engine,
                &clock,
                &bus,
                &mut sched,
                policy,
                &mut stats,
            );
            if let Some((_, Outcome::Completed)) = rep.transition_ended {
                en_ok = true;
                break;
            }
        }
        assert!(en_ok, "cycle {cycle} (guarded={guarded}): enable failed/timed out");
        max_en_ticks = max_en_ticks.max(en_ticks);

        // Hold for 25 ticks (0.5 s).
        for _ in 0..25 {
            let _ = step_executor_with_policy(
                &mut core,
                &mut engine,
                &clock,
                &bus,
                &mut sched,
                policy,
                &mut stats,
            );
        }

        // Brief motion away and back to rest, then immediate stop + TorqueOff on arrival
        // (0 settle ticks -> unconfirmed K=3 rest window -> enters AwaitRestGate + multi-tick Disable).
        {
            let mut target = core.commanded_positions.clone();
            target[0] = 0.12;
            core.servo_target = Some(target);
        }
        for _ in 0..6 {
            let _ = step_executor_with_policy(
                &mut core,
                &mut engine,
                &clock,
                &bus,
                &mut sched,
                policy,
                &mut stats,
            );
        }
        {
            let rest = core.config.resting_positions();
            core.servo_target = Some(rest.to_vec());
        }
        for _ in 0..24 {
            let _ = step_executor_with_policy(
                &mut core,
                &mut engine,
                &clock,
                &bus,
                &mut sched,
                policy,
                &mut stats,
            );
            if core.measured_positions[0].abs() < 0.005 {
                break;
            }
        }
        core.stop_motion();

        let auth = core.authority(&engine.ctx(clock.now()));
        broker
            .submit(
                ClientId(1),
                seq,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        seq += 1;

        let mut dis_outcome = None;
        let mut dis_ticks = 0u32;
        for _ in 0..50 {
            dis_ticks += 1;
            let rep = step_executor_with_policy(
                &mut core,
                &mut engine,
                &clock,
                &bus,
                &mut sched,
                policy,
                &mut stats,
            );
            if let Some((_, o)) = rep.transition_ended {
                dis_outcome = Some(o);
                break;
            }
        }
        assert_eq!(
            dis_outcome,
            Some(Outcome::Completed),
            "cycle {cycle} (guarded={guarded}): rest-disable failed (state={:?}, torqued={})",
            core.safety.state(),
            core.is_torqued
        );
        max_dis_ticks = max_dis_ticks.max(dis_ticks);
        assert!(!core.is_torqued);
        assert!(matches!(core.safety.state(), SafetyState::Idle));
    }

    CycleSimOutcome {
        stats,
        max_gap: bus.max_arm_ctrl_gap(),
        expiries: bus.total_expiries(),
        max_en_ticks,
        max_dis_ticks,
    }
}

#[test]
fn executor_enable_hold_rest_disable_20x_sg2002_sim() {
    assert_eq!(
        SubReleaseBudget::default_margin_for_period(RELEASE_PERIOD),
        DEFAULT_SUB_RELEASE_MARGIN
    );
    let out = run_enable_hold_rest_disable_workload(0x5B20_2026, 20, true, None);
    let stats = &out.stats;

    println!(
        "[SUBRELEASE-SIM guarded-cycle-20x] releases={} skips={} over_16ms={} max_release_ms={:.2} max_wakes_per_rel={} total_wakes={} max_tx={} max_gap_ms={:.2} expiries={} max_en_ticks={} max_dis_ticks={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.over_16ms,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_wakes_in_release,
        stats.total_wakes,
        stats.max_tx_in_release,
        out.max_gap.as_secs_f64() * 1e3,
        out.expiries,
        out.max_en_ticks,
        out.max_dis_ticks,
        stats.by_kind,
    );

    assert_eq!(out.expiries, 0, "TIMEOUT=2000 must have 0 watchdog expiries");
    assert!(
        out.max_gap <= MAX_ALLOWED_CTRL_GAP,
        "max arm watchdog gap {:?} > 40 ms",
        out.max_gap
    );
    assert_eq!(stats.skips, 0, "expected 0 skipped releases");
    assert!(
        stats.max_release <= MAX_GUARDED_RELEASE,
        "max release {:?} exceeded 16 ms budget",
        stats.max_release
    );
    assert!(
        stats.max_wakes_in_release <= MAX_SERVICE_WAKES,
        "wakes per release {} exceeded MAX_SERVICE_WAKES ({})",
        stats.max_wakes_in_release,
        MAX_SERVICE_WAKES
    );

    // Also prove all transition deadlines (FRESHEN, Readback, Enable Confirm, Multi-tick Rest-Disable)
    // still succeed under 100% sub-release wake deferral (`sub_release_margin = 20 ms`, 0 wakes!).
    let deferred = run_enable_hold_rest_disable_workload(
        0x5B20_2026,
        5,
        true,
        Some(RELEASE_PERIOD),
    );
    println!(
        "[SUBRELEASE-SIM full-deferral-5x] releases={} skips={} over_16ms={} max_release_ms={:.2} total_wakes={} max_gap_ms={:.2} expiries={} max_en_ticks={} max_dis_ticks={}",
        deferred.stats.releases,
        deferred.stats.skips,
        deferred.stats.over_16ms,
        deferred.stats.max_release.as_secs_f64() * 1e3,
        deferred.stats.total_wakes,
        deferred.max_gap.as_secs_f64() * 1e3,
        deferred.expiries,
        deferred.max_en_ticks,
        deferred.max_dis_ticks,
    );
    assert_eq!(deferred.stats.total_wakes, 0, "margin=20ms must defer every transition wake");
    assert_eq!(deferred.stats.skips, 0);
    assert!(deferred.stats.max_release <= MAX_GUARDED_RELEASE);
    assert_eq!(deferred.expiries, 0);
    assert!(deferred.max_gap <= MAX_ALLOWED_CTRL_GAP);
}

#[test]
fn executor_j1_motion_with_stops_sg2002_sim() {
    let clock = Arc::new(MockClock::new());
    let bus = SgBus::new(clock.clone(), 0x5B11_0006, [1, 2, 2, 1, 3, 2, 2]);
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
        Capacities {
            commands: 64,
            plans: 8,
            replies: 128,
        },
        VirtualClock::from_mock_clock((*clock).clone()),
    );

    let mut stats = SimStats::default();
    let mut seq = 1u64;

    // 6 full J1 motion cycles with mid-motion stops, return-to-rest, and immediate rest-disable
    // (mirroring rc3 full run's idle_tail_start transition phase).
    for cycle in 0..6u64 {
        let _ = broker.try_poll(|_, _| DeliveryDecision::Delivered);
        clock.advance(Duration::from_millis(120));
        bus.with(|s| s.motors[6].drop_enable_count = usize::from(cycle % 2 == 0));

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
            let rep = step_executor_guarded(&mut fx, &clock, &bus, &mut stats);
            if let Some((_, Outcome::Completed)) = rep.transition_ended {
                en_ok = true;
                break;
            }
        }
        assert!(en_ok, "J1 cycle {cycle}: enable failed");

        // 150 ticks (3 s) sinusoidal J1 motion with 2 mid-motion stops.
        for tick in 0u64..150 {
            if tick == 50 || tick == 100 {
                fx.core.stop_motion();
                for _ in 0..5 {
                    let _ = step_executor_guarded(&mut fx, &clock, &bus, &mut stats);
                }
            }
            let phase = (tick as f32) * 0.02 * std::f32::consts::TAU * 0.4;
            let mut target = fx.core.commanded_positions.clone();
            target[0] = 0.30 * phase.sin();
            fx.core.servo_target = Some(target);
            let _ = step_executor_guarded(&mut fx, &clock, &bus, &mut stats);
        }

        // Return to rest, stop immediately upon arrival, and trigger rest-disable (`idle_tail_start` phase).
        let rest = fx.core.config.resting_positions();
        fx.core.servo_target = Some(rest.to_vec());
        for _ in 0..40 {
            let _ = step_executor_guarded(&mut fx, &clock, &bus, &mut stats);
            if fx.core.measured_positions[0].abs() < 0.005 {
                break;
            }
        }
        fx.core.stop_motion();

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
        let mut dis_ok = false;
        for _ in 0..50 {
            let rep = step_executor_guarded(&mut fx, &clock, &bus, &mut stats);
            if let Some((_, Outcome::Completed)) = rep.transition_ended {
                dis_ok = true;
                break;
            }
        }
        assert!(dis_ok, "J1 cycle {cycle}: idle_tail_start rest-disable failed");
    }

    let max_gap = bus.max_arm_ctrl_gap();
    let expiries = bus.total_expiries();

    println!(
        "[SUBRELEASE-SIM guarded-j1-stops] releases={} skips={} over_16ms={} max_release_ms={:.2} max_wakes_per_rel={} total_wakes={} max_tx={} max_gap_ms={:.2} expiries={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.over_16ms,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_wakes_in_release,
        stats.total_wakes,
        stats.max_tx_in_release,
        max_gap.as_secs_f64() * 1e3,
        expiries,
        stats.by_kind,
    );

    assert_eq!(expiries, 0, "TIMEOUT=2000 must have 0 watchdog expiries");
    assert!(
        max_gap <= MAX_ALLOWED_CTRL_GAP,
        "max arm watchdog gap {:?} > 40 ms",
        max_gap
    );
    assert_eq!(stats.skips, 0, "expected 0 skipped releases");
    assert!(
        stats.max_release <= MAX_GUARDED_RELEASE,
        "max release {:?} exceeded 16 ms budget",
        stats.max_release
    );
    assert!(stats.max_wakes_in_release <= MAX_SERVICE_WAKES);
}

#[test]
fn positive_control_without_subrelease_guard_reproduces_overrun() {
    // Run the EXACT SAME 20-cycle SG2002 workload and timing model (zero extra synthetic timing
    // penalty!) without the sub-release guard and prove it reproduces > 20 ms release overruns
    // and skipped releases!
    let out = run_enable_hold_rest_disable_workload(0x5B20_2026, 20, false, None);
    let stats = &out.stats;

    println!(
        "[SUBRELEASE-SIM unguarded-cycle-20x] releases={} skips={} over_16ms={} max_release_ms={:.2} max_wakes_per_rel={} total_wakes={} max_tx={} max_gap_ms={:.2} expiries={} max_en_ticks={} max_dis_ticks={} by_kind={:?}",
        stats.releases,
        stats.skips,
        stats.over_16ms,
        stats.max_release.as_secs_f64() * 1e3,
        stats.max_wakes_in_release,
        stats.total_wakes,
        stats.max_tx_in_release,
        out.max_gap.as_secs_f64() * 1e3,
        out.expiries,
        out.max_en_ticks,
        out.max_dis_ticks,
        stats.by_kind,
    );

    assert!(
        stats.max_release > RELEASE_PERIOD && stats.skips >= 1,
        "positive control MUST reproduce > 20 ms overrun and >= 1 skipped release on identical workload without guard (got max={:?}, skips={})",
        stats.max_release,
        stats.skips
    );
}

/// Mock driver that schedules an immediate sub-release wake chain (`wake = now + offset`) to
/// verify `MAX_SERVICE_WAKES == 1` and configurable `sub_release_margin`.
struct WakeProbeDriver {
    clock: Arc<MockClock>,
    q: Vec<f32>,
    enabled: bool,
    pending_polls_left: u32,
    wake_offset: Duration,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
}

impl WakeProbeDriver {
    fn new(clock: Arc<MockClock>, pending_polls: u32, wake_offset: Duration) -> Self {
        let now = clock.now();
        Self {
            clock,
            q: vec![0.0; 7],
            enabled: false,
            pending_polls_left: pending_polls,
            wake_offset,
            rx_at: vec![Some(now); 7],
            seqs: vec![1; 7],
        }
    }
}

impl ArmDriver for WakeProbeDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }
    fn clock_now(&self) -> Instant {
        self.clock.now()
    }
    fn enable_state(&self) -> EnableState {
        if self.enabled {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let now = self.clock.now();
        self.rx_at
            .iter()
            .map(|t| t.map(|t| now.saturating_duration_since(t)))
            .collect()
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.rx_at.clone()
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        self.seqs.clone()
    }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.rx_at.clone()
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let now = self.clock.now();
        out.clear();
        out.extend(
            self.rx_at
                .iter()
                .map(|t| t.map(|t| now.saturating_duration_since(t))),
        );
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let now = self.clock.now();
        self.rx_at.len() == dof
            && self
                .rx_at
                .iter()
                .all(|t| t.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let now = self.clock.now();
        out.clear();
        out.extend(
            self.rx_at
                .iter()
                .map(|t| t.map(|t| now.saturating_duration_since(t).as_millis() as u64)),
        );
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        out.extend_from_slice(&self.seqs);
    }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(7);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = self.clock.now();
        out.clear();
        for i in 0..7 {
            out.push(JointState {
                position: self.q[i],
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            });
            self.rx_at[i] = Some(now);
            self.seqs[i] += 1;
        }
        Ok(())
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((7, false))
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(7) {
            self.q[i] = t.position;
        }
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }
    fn begin_torque_budgeted(
        &mut self,
        _enable: bool,
        _max_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        Ok(TorqueProgress::Pending {
            wake: self.clock.now() + self.wake_offset,
        })
    }
    fn poll_torque_budgeted(
        &mut self,
        _deadline: Instant,
        _max_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        if self.pending_polls_left > 1 {
            self.pending_polls_left -= 1;
            Ok(TorqueProgress::Pending {
                wake: self.clock.now() + self.wake_offset,
            })
        } else {
            self.pending_polls_left = 0;
            self.enabled = true;
            Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
        }
    }
    fn abort_enable(&mut self) {
        self.pending_polls_left = 0;
    }
    fn torque_transition_pending(&self) -> bool {
        self.pending_polls_left > 0
    }
}

#[test]
fn sub_release_budget_configurable_margin_and_single_wake_bound() {
    // 1. Env parser validation (default 4 ms at 50 Hz, custom >= 4 ms, fail-closed on < 4 ms / invalid).
    assert_eq!(
        parse_sub_release_margin_ms(None, RELEASE_PERIOD).unwrap(),
        Duration::from_millis(4)
    );
    assert_eq!(
        parse_sub_release_margin_ms(Some("4"), RELEASE_PERIOD).unwrap(),
        Duration::from_millis(4)
    );
    assert_eq!(
        parse_sub_release_margin_ms(Some("6"), RELEASE_PERIOD).unwrap(),
        Duration::from_millis(6)
    );
    assert!(
        parse_sub_release_margin_ms(Some("0"), RELEASE_PERIOD).is_err(),
        "ARMD_SUB_RELEASE_MARGIN_MS=0 must fail closed below the 4 ms floor at 50 Hz"
    );
    assert!(
        parse_sub_release_margin_ms(Some("3"), RELEASE_PERIOD).is_err(),
        "ARMD_SUB_RELEASE_MARGIN_MS=3 must fail closed below the 4 ms floor at 50 Hz"
    );
    assert!(
        parse_sub_release_margin_ms(Some("1"), Duration::from_millis(6)).is_err(),
        "fractional-ms floor (6 ms / 5 = 1.2 ms) must reject 1 ms without truncating down"
    );
    assert!(parse_sub_release_margin_ms(Some("abc"), RELEASE_PERIOD).is_err());
    assert!(parse_sub_release_margin_ms(Some("-2"), RELEASE_PERIOD).is_err());

    // Direct config floor enforcement: even if ExecutorConfig sets sub_release_margin = 0 ms,
    // SubReleaseBudget::for_release clamps effective_margin to >= 4 ms at 20 ms period.
    let t0 = Instant::now();
    let ecfg_zero = ExecutorConfig::for_tests(RELEASE_PERIOD).with_sub_release_margin(Duration::ZERO);
    let clamped = SubReleaseBudget::for_release(
        t0,
        t0 + RELEASE_PERIOD,
        RELEASE_PERIOD,
        ecfg_zero.sub_release_margin,
    );
    assert_eq!(
        clamped.wake_deadline,
        t0 + Duration::from_millis(16),
        "direct zero margin override must still enforce 4 ms completion floor (<= 16 ms)"
    );
    assert_eq!(
        clamped.poll_deadline,
        t0 + Duration::from_millis(15),
        "poll_deadline must reserve 1 ms tail headroom before 16 ms wake_deadline"
    );
    assert_eq!(
        clamped.transition_start_cutoff,
        t0 + Duration::from_millis(11),
        "transition_start_cutoff must remain <= 11 ms when zero margin is requested"
    );
    assert_eq!(
        clamped.urgent_stop_cutoff,
        t0 + Duration::from_micros(15_500),
        "urgent_stop_cutoff must stay strictly below 16 ms (15.5 ms) so urgent wakes finish <= 16 ms"
    );

    // 2. Single-wake bound (`MAX_SERVICE_WAKES == 1`): even when a transition asks for another
    //    sub-release wake 2 ms later, at most 1 wake runs in the release and the rest defers.
    assert_eq!(MAX_SERVICE_WAKES, 1);
    {
        let clock = Arc::new(MockClock::new());
        let drv = WakeProbeDriver::new(clock.clone(), 4, Duration::from_millis(2));
        let mut inner = SupervisorInner::new(build_sg2002_config()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.rt_executor_owned = true;
        inner.register_live_connection(1).unwrap();
        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities {
                commands: 16,
                plans: 4,
                replies: 32,
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
        // Release 1 commits TorqueOn; Release 2 polls in Step 2 (pending_polls 4->3, wake=+2 ms)
        // and runs AT MOST 1 Window-2 wake (pending_polls 3->2, wake=+4 ms deferred to Release 3!).
        let _r1 = fx.step();
        let r2 = fx.step();
        assert_eq!(
            r2.wakes, 1,
            "release must execute at most MAX_SERVICE_WAKES (1) sub-release wake"
        );
        assert!(
            r2.transition_ended.is_none(),
            "remaining polls must defer to the next release"
        );
        let r3 = fx.step();
        assert_eq!(r3.wakes, 1);
        assert_eq!(
            r3.transition_ended.map(|(_, o)| o),
            Some(Outcome::Completed)
        );
    }

    // 3. Configurable `sub_release_margin`: a wake at t + 10 ms on a 20 ms period is ADMITTED
    //    with the default 4 ms margin (`transition_start_cutoff = t + 11 ms`) and DEFERRED when
    //    the configured margin is raised to 6 ms (`transition_start_cutoff = t + 9 ms`).
    for (margin_ms, expect_wake_on_r2) in [(4u64, true), (6u64, false)] {
        let clock = Arc::new(MockClock::new());
        let drv = WakeProbeDriver::new(clock.clone(), 2, Duration::from_millis(10));
        let mut inner = SupervisorInner::new(build_sg2002_config()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.rt_executor_owned = true;
        inner.register_live_connection(1).unwrap();
        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities {
                commands: 16,
                plans: 4,
                replies: 32,
            },
            VirtualClock::from_mock_clock((*clock).clone()),
        );
        fx = fx.with_sub_release_margin(Duration::from_millis(margin_ms));
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
        let _r1 = fx.step();
        let r2 = fx.step();
        if expect_wake_on_r2 {
            assert_eq!(
                r2.wakes, 1,
                "margin={margin_ms} ms should admit wake at t + 10 ms"
            );
            assert_eq!(
                r2.transition_ended.map(|(_, o)| o),
                Some(Outcome::Completed)
            );
        } else {
            assert_eq!(
                r2.wakes, 0,
                "margin={margin_ms} ms must defer wake at t + 10 ms to the next release"
            );
            assert!(r2.transition_ended.is_none());
            let r3 = fx.step();
            assert_eq!(
                r3.transition_ended.map(|(_, o)| o),
                Some(Outcome::Completed),
                "deferred transition must complete on the next release"
            );
        }
    }

    // 4. `ExecutorConfig::with_sub_release_margin` carries the configured value cleanly.
    let ecfg = ExecutorConfig::for_tests(RELEASE_PERIOD)
        .with_sub_release_margin(Duration::from_millis(6));
    assert_eq!(ecfg.sub_release_margin, Duration::from_millis(6));
    let _ = PersistTestHooks::default();
}

/// Blocker 2 regression: phase-offset missed refresh under full Window-2 deferral (`margin = 20 ms`)
/// AND exhausted motion TX frame cap (`tx_budget.max_frames`).
///
/// Scenario:
/// - Release 0 (`t = 0 ms`): arm motors refreshed late in release 0 at `t = 10 ms`.
/// - Release 1 (`t = 20 ms`): normal Step-5 output is skipped for J2..J6 (only J1 refreshed),
///   non-hold TX attempts exhaust `tx_budget.max_frames`, and `service_release_watchdog_fallback`
///   runs at `t = 27 ms` (when J2..J6 age is only `17 ms < 18 ms`!).
/// - Window 2 defers 100% of wakes (`sub_release_margin = 20 ms`).
/// - Release 2 (`t = 40 ms`): next normal output runs late at `t = 51 ms`.
///
/// Without `Duration::ZERO` + Window-1 fallback reservation, J2..J6 would suffer a `51 - 10 = 41 ms`
/// watchdog expiry. With the fix, fallback refreshes J2..J6 at `t = 27 ms` (gap `17 ms << 40 ms`).
#[test]
fn phase_offset_missed_refresh_under_full_deferral_and_tx_cap_stays_under_40ms() {
    let clock = Arc::new(MockClock::new());
    let bus = SgBus::new(clock.clone(), 0x51A5_E001, [1, 1, 1, 1, 1, 1, 1]);
    let cfg = build_sg2002_config();
    let mut drv = build_sg2002_driver(bus.clone(), &cfg);

    // Bootstrap & enable all motors cleanly.
    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback_now());
    drv.read_joint_states().unwrap();
    assert_eq!(
        drv.check_mapping_ranges().status,
        armd::driver::RangeCheckStatus::Passed
    );
    bus.with(|s| s.immediate_init = false);
    drv.set_torque(true).unwrap();

    // Release 0 (t = 0 ms): refresh all 6 arm motors late at t = 10 ms.
    let t0 = clock.now();
    clock.advance(Duration::from_millis(10));
    bus.begin_release();
    drv.begin_control_release(1);
    drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(18)));
    let zeros: Vec<_> = (0..7).map(|_| JointTarget::position(0.0)).collect();
    drv.write_joint_targets_bounded(&zeros).unwrap();
    bus.with(|s| {
        for m in &mut s.motors {
            m.max_ctrl_gap = Duration::ZERO;
            m.watchdog_expiries = 0;
        }
    });

    // Release 1 (t = 20 ms): advance to t = 27 ms (17 ms since t = 10 ms refresh!),
    // exhaust the motion TX frame cap (12 frames) with 0xCC refreshes/holds on J1 only,
    // leaving J2..J6 unrefreshed in release 1 with age = 17 ms (< 18 ms).
    let t1 = t0 + RELEASE_PERIOD;
    clock.advance(t1.saturating_duration_since(clock.now()));
    let mut tight_budget = cfg.damiao_tx_budget();
    tight_budget.max_frames = 1;
    drv.set_tx_budget(tight_budget);
    bus.begin_release();
    drv.begin_control_release(2);
    drv.set_tx_release_deadline(Some(t1 + Duration::from_millis(18)));
    // Refresh ONLY J1 (consuming the 1 motion TX frame slot!).
    let _ = drv.write_measured_hold_masked(1u64 << 0, &zeros).unwrap();
    clock.advance((t1 + Duration::from_millis(7)).saturating_duration_since(clock.now()));

    // Fallback runs at t = 27 ms: MUST refresh J2..J6 immediately despite age = 17 ms and max_frames = 1!
    reset_tx_metrics();
    drv.service_release_watchdog_fallback(u64::MAX, None);
    let fb = take_tx_metrics();
    assert_eq!(
        fb.by_kind.urgent_hold, 5,
        "phase-offset missed joints (age 17 ms) must be refreshed in Window 1 fallback even when motion tx_budget.max_frames is exhausted"
    );

    // Calling fallback again in the SAME release must still coalesce to 0 frames.
    reset_tx_metrics();
    drv.service_release_watchdog_fallback(u64::MAX, None);
    assert_eq!(take_tx_metrics().by_kind.urgent_hold, 0);

    // Window 2 defers completely; Release 2 runs late normal output at t = 51 ms.
    let t2_late = t0 + Duration::from_millis(51);
    clock.advance(t2_late.saturating_duration_since(clock.now()));
    drv.set_tx_budget(cfg.damiao_tx_budget());
    bus.begin_release();
    drv.begin_control_release(3);
    drv.set_tx_release_deadline(Some(t0 + Duration::from_millis(58)));
    drv.write_joint_targets_bounded(&zeros).unwrap();

    let (max_gap, expiries) = (bus.max_arm_ctrl_gap(), bus.total_expiries());
    assert!(
        max_gap <= Duration::from_millis(40) && expiries == 0,
        "phase-offset missed refresh under full deferral exceeded watchdog bound: max_gap={max_gap:?}, expiries={expiries}"
    );
}

/// Blocker 3 + Major 4 regression:
/// 1. Urgent Stop / E-Stop / Disconnect arriving AFTER Window-2 CAN wake classification (when
///    `allow_transition_poll == true`) is consumed by `service_wake_with_clock_opts` and MUST
///    suppress CAN `poll_torque_budgeted` in that wake (0 CAN polls).
/// 2. Non-urgent Window-2 torque-shadow flush MUST defer when remaining headroom before
///    `wake_deadline` is `< SUB_RELEASE_SHADOW_MARGIN` (including at or after `wake_deadline`).
#[test]
fn mid_wake_urgent_arrival_suppresses_can_poll_and_shadow_requires_headroom() {
    struct MidWakeDriver {
        clock: Arc<MockClock>,
        poll_count: Arc<AtomicUsize>,
        pending: bool,
    }
    impl ArmDriver for MidWakeDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
        fn enable_state(&self) -> EnableState {
            EnableState::Enabled
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::ZERO); 7]
        }
        fn feedback_instants(&self) -> Vec<Option<Instant>> {
            vec![Some(self.clock.now()); 7]
        }
        fn feedback_seqs(&self) -> Vec<u64> {
            vec![1; 7]
        }
        fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
            vec![Some(self.clock.now()); 7]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![
                JointState {
                    position: 0.0,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                };
                7
            ])
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn begin_torque_budgeted(
            &mut self,
            _enable: bool,
            _max_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.pending = true;
            Ok(TorqueProgress::Pending {
                wake: self.clock.now() + Duration::from_millis(2),
            })
        }
        fn poll_torque_budgeted(
            &mut self,
            _deadline: Instant,
            _max_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.poll_count.fetch_add(1, Ordering::AcqRel);
            Ok(TorqueProgress::Pending {
                wake: self.clock.now() + Duration::from_millis(2),
            })
        }
        fn abort_enable(&mut self) {
            self.pending = false;
        }
        fn torque_transition_pending(&self) -> bool {
            self.pending
        }
    }

    // Exercise Stop, E-Stop, and Disconnect arriving between classification (`allow_transition_poll = true`)
    // and `service_wake_with_clock_opts` execution.
    for kind in ["stop", "estop", "disconnect"] {
        let clock = Arc::new(MockClock::new());
        let poll_count = Arc::new(AtomicUsize::new(0));
        let drv = MidWakeDriver {
            clock: clock.clone(),
            poll_count: poll_count.clone(),
            pending: false,
        };
        let mut core = SupervisorInner::new(build_sg2002_config()).with_driver(Box::new(drv));
        core.lease_enabled = false;
        core.rt_executor_owned = true;
        core.register_live_connection(1).unwrap();
        core.set_boot_id(BOOT);

        let (mut engine, mut broker, _tw, stops) = ReleaseEngine::new(
            BOOT,
            Capacities {
                commands: 16,
                plans: 4,
                replies: 32,
            },
        );
        let (lifecycle_client, lifecycle_intake) =
            armd::rt_core::requests::lease::conn_lifecycle_channel(Arc::new(AtomicBool::new(false)));
        lifecycle_client.seed_live_connections([1]);
        engine.attach_conn_lifecycle(lifecycle_intake);
        core.attach_closed_bitset(broker.closed_bitset());

        // Start a pending transition in Release 1 so a CAN poll would otherwise execute.
        let auth = core.authority(&engine.ctx(clock.now()));
        broker
            .submit(
                ClientId(1),
                1,
                auth,
                clock.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        let dl1 = clock.now() + RELEASE_PERIOD;
        let mut report = engine.step(&mut core, || clock.now(), DT, dl1, 1);
        let polls_before = poll_count.load(Ordering::Acquire);

        // Inject the urgent event AFTER Window 2 would have classified `allow_transition_poll = true`.
        match kind {
            "stop" => {
                let _ = stops.request();
            }
            "estop" => {
                let _ = core.estop_state.request_and_latch();
            }
            "disconnect" => {
                lifecycle_client.enqueue_disconnect_detached(
                    armd::rt_core::requests::lease::ConnectionHandle::from_ws_id(1).unwrap(),
                );
            }
            _ => unreachable!(),
        }

        let published = engine.service_wake_with_clock_opts(
            &mut core,
            || clock.now(),
            clock.now() + Duration::from_millis(15),
            2,
            true,
            &mut report,
        );
        assert!(
            published,
            "mid-wake urgent {kind} must still publish state immediately"
        );
        assert_eq!(
            poll_count.load(Ordering::Acquire) - polls_before,
            0,
            "mid-wake urgent {kind} consumed in service_wake_with_clock_opts must suppress CAN poll_torque_budgeted"
        );
    }

    // 2. Verify `SUB_RELEASE_SHADOW_MARGIN` is strictly positive and non-urgent shadow flush
    //    with a POPULATED shadow queue defers at/past `wake_deadline` and flushes when headroom >= 500 us.
    assert!(SUB_RELEASE_SHADOW_MARGIN > Duration::ZERO);
    let clock = Arc::new(MockClock::new());
    let bus = SgBus::new(clock.clone(), 0x5AD0_0001, [1, 1, 1, 1, 1, 1, 1]);
    let cfg = build_sg2002_config();
    let drv = build_sg2002_driver(bus.clone(), &cfg);
    let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    core.lease_enabled = false;
    core.rt_executor_owned = true;
    core.register_live_connection(1).unwrap();
    core.set_boot_id(BOOT);
    let (mut engine, mut broker, _tw, _stop) = ReleaseEngine::new(
        BOOT,
        Capacities {
            commands: 16,
            plans: 4,
            replies: 32,
        },
    );
    core.attach_closed_bitset(broker.closed_bitset());
    let auth = core.authority(&engine.ctx(clock.now()));
    broker
        .submit(
            ClientId(1),
            1,
            auth,
            clock.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
    let _ = engine.step(&mut core, || clock.now(), DT, clock.now() + RELEASE_PERIOD, 1);
    let _ = engine.step(&mut core, || clock.now(), DT, clock.now() + RELEASE_PERIOD, 2);
    let now = Instant::now();
    let wake_deadline = now;
    assert!(
        !core.flush_torque_shadow_if_budget(false, now, wake_deadline, SUB_RELEASE_SHADOW_MARGIN),
        "non-urgent shadow flush with populated FIFO must NOT run at wake_deadline"
    );
    assert!(
        !core.flush_torque_shadow_if_budget(
            false,
            now,
            wake_deadline + Duration::from_micros(200),
            SUB_RELEASE_SHADOW_MARGIN,
        ),
        "non-urgent shadow flush must NOT run when headroom < SUB_RELEASE_SHADOW_MARGIN"
    );
    assert!(
        core.flush_torque_shadow_if_budget(
            false,
            now,
            wake_deadline + SUB_RELEASE_SHADOW_MARGIN,
            SUB_RELEASE_SHADOW_MARGIN,
        ),
        "populated shadow FIFO must flush once remaining headroom >= SUB_RELEASE_SHADOW_MARGIN"
    );
}

/// Production-path `RtExecutor` test:
/// Exercises Stop, E-Stop, and Disconnect on the real `RtExecutor` with `sub_release_margin = 20 ms`
/// (100% CAN transition poll deferral):
/// - Verifies late arrival (between Window 1 end and Window 2 dispatch / post-Step-5) is applied in
///   the SAME release via a Window-2 urgent wake.
/// - Verifies 0 Window-2 CAN transition polls (`poll_torque_budgeted`), 0 extra CAN RX/TX calls,
///   0 heap allocations, and 0 `SupervisorLock` attempts.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn production_executor_urgent_stop_bypass_zero_alloc_and_zero_locks() {
    struct CountingPollDriver {
        q: Vec<f32>,
        enabled: bool,
        pending: bool,
        rx_at: Vec<Option<Instant>>,
        seqs: Vec<u64>,
        rx_count: Arc<AtomicUsize>,
        tx_count: Arc<AtomicUsize>,
        poll_count: Arc<AtomicUsize>,
        inject_mode: Arc<AtomicUsize>, // 0=none, 1=stop, 2=estop, 3=disconnect
        injected_release: Arc<std::sync::atomic::AtomicU64>,
        stats_handle: Arc<Mutex<Option<Arc<armd::rt_core::executor::ExecutorStats>>>>,
        stop_handle: Arc<Mutex<Option<armd::rt_core::urgent::StopRequests>>>,
        sup_handle: Arc<Mutex<Option<Arc<ArmSupervisor>>>>,
    }
    impl ArmDriver for CountingPollDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn supports_rt_executor(&self) -> Result<(), &'static str> {
            Ok(())
        }
        fn enable_state(&self) -> EnableState {
            if self.enabled {
                EnableState::Enabled
            } else {
                EnableState::Disabled
            }
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            let now = Instant::now();
            self.rx_at
                .iter()
                .map(|t| t.map(|t| now.saturating_duration_since(t)))
                .collect()
        }
        fn feedback_instants(&self) -> Vec<Option<Instant>> {
            self.rx_at.clone()
        }
        fn feedback_seqs(&self) -> Vec<u64> {
            self.seqs.clone()
        }
        fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
            self.rx_at.clone()
        }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            let now = Instant::now();
            out.clear();
            out.extend(
                self.rx_at
                    .iter()
                    .map(|t| t.map(|t| now.saturating_duration_since(t))),
            );
        }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
            let now = Instant::now();
            self.rx_at.len() == dof
                && self
                    .rx_at
                    .iter()
                    .all(|t| t.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
        }
        fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
            let now = Instant::now();
            out.clear();
            out.extend(
                self.rx_at
                    .iter()
                    .map(|t| t.map(|t| now.saturating_duration_since(t).as_millis() as u64)),
            );
        }
        fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            out.clear();
            out.extend_from_slice(&self.rx_at);
        }
        fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
            out.clear();
            out.extend_from_slice(&self.seqs);
        }
        fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
            out.clear();
            out.extend_from_slice(&self.rx_at);
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            let mut out = Vec::with_capacity(7);
            self.read_joint_states_into(&mut out)?;
            Ok(out)
        }
        fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
            self.rx_count.fetch_add(1, Ordering::AcqRel);
            let now = Instant::now();
            out.clear();
            for i in 0..7 {
                out.push(JointState {
                    position: self.q[i],
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                });
                self.rx_at[i] = Some(now);
                self.seqs[i] += 1;
            }
            Ok(())
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<JointState>,
            _deadline: Instant,
            _max_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            self.read_joint_states_into(out)?;
            Ok((7, false))
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.tx_count.fetch_add(1, Ordering::AcqRel);
            for (i, t) in targets.iter().enumerate().take(7) {
                self.q[i] = t.position;
            }
            Ok(())
        }
        fn service_release_watchdog_fallback(
            &mut self,
            _authorized_mask: u64,
            _owner_hold_targets: Option<&[Option<f32>]>,
        ) {
            // Inject the urgent request strictly after Step 5's `stops.mark_applied()` checkpoint
            // (src/supervisor.rs:11598) so it is pending strictly upon Window 2 entry.
            let mode = self.inject_mode.swap(0, Ordering::AcqRel);
            if mode != 0 {
                if let Some(st) = self.stats_handle.lock().unwrap().as_ref() {
                    // Record the 1-based release index currently in Window 1 so the test can wait
                    // until THIS exact release's `completed_releases` finishes (after Window 2).
                    let rel = st.completed_releases.load(Ordering::Acquire) + 1;
                    self.injected_release.store(rel, Ordering::Release);
                }
            }
            match mode {
                1 => {
                    if let Some(stops) = self.stop_handle.lock().unwrap().as_ref() {
                        let _ = stops.request();
                    }
                }
                2 => {
                    if let Some(sup) = self.sup_handle.lock().unwrap().as_ref() {
                        let _ = sup.estop_state.request_and_latch();
                    }
                }
                3 => {
                    if let Some(sup) = self.sup_handle.lock().unwrap().as_ref() {
                        sup.disconnect_ws_connection_detached(
                            armd::rt_core::requests::lease::ConnectionHandle::from_ws_id(1)
                                .unwrap(),
                        );
                    }
                }
                _ => {}
            }
        }
        fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
            self.enabled = enable;
            Ok(())
        }
        fn begin_torque_budgeted(
            &mut self,
            _enable: bool,
            _max_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.pending = true;
            Ok(TorqueProgress::Pending {
                wake: Instant::now() + Duration::from_millis(2),
            })
        }
        fn poll_torque_budgeted(
            &mut self,
            _deadline: Instant,
            _max_frames: u16,
        ) -> Result<TorqueProgress, ArmDriverError> {
            self.poll_count.fetch_add(1, Ordering::AcqRel);
            Ok(TorqueProgress::Pending {
                wake: Instant::now() + Duration::from_millis(2),
            })
        }
        fn abort_enable(&mut self) {
            self.pending = false;
        }
        fn torque_transition_pending(&self) -> bool {
            self.pending
        }
    }

    for (mode, label) in [(1usize, "stop"), (2usize, "estop"), (3usize, "disconnect")] {
        let rx_count = Arc::new(AtomicUsize::new(0));
        let tx_count = Arc::new(AtomicUsize::new(0));
        let poll_count = Arc::new(AtomicUsize::new(0));
        let inject_mode = Arc::new(AtomicUsize::new(0));
        let injected_release = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let stats_handle: Arc<Mutex<Option<Arc<armd::rt_core::executor::ExecutorStats>>>> =
            Arc::new(Mutex::new(None));
        let stop_handle: Arc<Mutex<Option<armd::rt_core::urgent::StopRequests>>> =
            Arc::new(Mutex::new(None));
        let sup_handle: Arc<Mutex<Option<Arc<ArmSupervisor>>>> = Arc::new(Mutex::new(None));
        let now = Instant::now();
        let drv = CountingPollDriver {
            q: vec![0.0; 7],
            enabled: true,
            pending: false,
            rx_at: vec![Some(now); 7],
            seqs: vec![1; 7],
            rx_count: rx_count.clone(),
            tx_count: tx_count.clone(),
            poll_count: poll_count.clone(),
            inject_mode: inject_mode.clone(),
            injected_release: injected_release.clone(),
            stats_handle: stats_handle.clone(),
            stop_handle: stop_handle.clone(),
            sup_handle: sup_handle.clone(),
        };

        let sup = Arc::new(ArmSupervisor::new(build_sg2002_config()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.inner().write().await.register_live_connection(1).unwrap();
        {
            let mut inner = sup.inner().write().await;
            let _ = inner.tick(DT);
            let stamp = inner.current_admission_stamp();
            let auth = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(auth).unwrap();
            assert!(inner.is_torqued);
        }

        let mut ecfg =
            ExecutorConfig::for_tests(RELEASE_PERIOD).with_sub_release_margin(RELEASE_PERIOD);
        ecfg.alloc_timing = true;
        let release_delay_once = Arc::new(Mutex::new(None));
        let handle = sup
            .start_executor_with_hooks(
                ecfg,
                PersistTestHooks {
                    release_delay_once: Some(release_delay_once.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        *stats_handle.lock().unwrap() = Some(handle.stats().clone());
        *stop_handle.lock().unwrap() = Some(handle.stop_requests().clone());
        *sup_handle.lock().unwrap() = Some(sup.clone());

        // Hold the detached legacy `SupervisorLock` write guard to prove 0 RT lock touches.
        let _legacy_write_guard = sup.inner().write().await;

        let dl = Instant::now() + Duration::from_secs(2);
        while handle.stats().completed_releases.load(Ordering::Acquire) < 2 && Instant::now() < dl {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let wakes_before = handle.stats().wakes.load(Ordering::Acquire);
        let allocs_before = handle.stats().total_allocs();
        let locks_before = sup.inner().control_thread_attempts();
        let polls_before = poll_count.load(Ordering::Acquire);
        let completed_before = handle.stats().completed_releases.load(Ordering::Acquire);
        let rx_before = rx_count.load(Ordering::Acquire);
        let tx_before = tx_count.load(Ordering::Acquire);

        let auth = Authority {
            lease_generation: 0,
            epoch: sup.hub().latest().map_or(1, |s| s.obs.epoch_id),
            stop_generation: handle.stop_requests().current(),
            config_revision: 0,
            parent_generation: 0,
        };
        handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                1,
                auth,
                Instant::now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        // 11.5 ms Window-1 delay: strictly past `transition_start_cutoff` (11 ms) and `wake_deadline`
        // (0 ms under margin = 20 ms), with 4 ms Linux CFS jitter margin below `urgent_stop_cutoff` (15.5 ms).
        *release_delay_once.lock().unwrap() = Some(Duration::from_micros(11_500));
        inject_mode.store(mode, Ordering::Release);

        // Wait until the EXACT injected release finishes (including its Window-2 wake and post-wake
        // seal), so counter sampling never races an in-flight wake.
        let dl2 = Instant::now() + Duration::from_secs(2);
        while (injected_release.load(Ordering::Acquire) == 0
            || handle.stats().completed_releases.load(Ordering::Acquire)
                < injected_release.load(Ordering::Acquire))
            && Instant::now() < dl2
        {
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let target_rel = injected_release.load(Ordering::Acquire);
        let completed_after = handle.stats().completed_releases.load(Ordering::Acquire);
        let wakes_after = handle.stats().wakes.load(Ordering::Acquire);
        let rx_after = rx_count.load(Ordering::Acquire);
        let tx_after = tx_count.load(Ordering::Acquire);
        let polls_after = poll_count.load(Ordering::Acquire);
        let allocs_delta = handle.stats().total_allocs().delta_since(allocs_before);
        let locks_delta = sup.inner().control_thread_attempts() - locks_before;

        assert!(
            target_rel > 0 && completed_after >= target_rel,
            "{label}: targeted release must have completed (target_rel={target_rel}, completed={completed_after})"
        );
        assert!(
            wakes_after > wakes_before,
            "{label}: urgent event must execute in Window 2 even with sub_release_margin = 20 ms (wakes={wakes_after} before={wakes_before})"
        );
        if mode == 1 {
            assert!(
                handle.stop_requests().current() >= 1
                    && handle.stop_requests().applied() == handle.stop_requests().current(),
                "stop must be applied in Window 2"
            );
        } else if mode == 2 {
            assert!(
                sup.hub()
                    .latest()
                    .is_some_and(|s| s.obs.safety_state == SafetyState::EStop),
                "estop must be applied and published in Window 2"
            );
        } else if mode == 3 {
            // Re-opening connection 1 returns `Ok(Err("DISCONNECTED"))` ONLY after Window 2
            // closed and tombstoned connection 1 (if still live it would fail `AlreadyClosed`).
            assert_eq!(
                handle.open_connection_async(1).await,
                Ok(Err("DISCONNECTED")),
                "disconnect must close and tombstone connection 1 inside the targeted release's Window 2 wake"
            );
        }
        assert_eq!(
            polls_after - polls_before,
            0,
            "{label}: urgent-only Window 2 wake must NOT invoke hardware poll_torque_budgeted"
        );
        // Each completed release performs at most 1 Window-1 RX and 1 Window-1 TX; Window 2 urgent
        // wake performs strictly 0 CAN RX/TX.
        let releases_run = (completed_after.saturating_sub(completed_before)) as usize;
        assert!(
            rx_after - rx_before <= releases_run + 1,
            "{label}: Window 2 urgent wake must add 0 extra CAN RX reads (rx_delta={}, releases_run={})",
            rx_after - rx_before,
            releases_run
        );
        assert!(
            tx_after - tx_before <= releases_run + 1,
            "{label}: Window 2 urgent wake must add 0 extra CAN TX writes (tx_delta={}, releases_run={})",
            tx_after - tx_before,
            releases_run
        );
        assert_eq!(
            allocs_delta,
            AllocCounts::ZERO,
            "{label}: production RT executor Window 1 + urgent Window 2 wake must perform 0 heap allocs/frees"
        );
        assert_eq!(
            locks_delta, 0,
            "{label}: production RT executor must perform 0 SupervisorLock attempts"
        );
        handle.stop_and_join();
    }
}
