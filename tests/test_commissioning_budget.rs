//! SG2002-timing regression tests for off-lock legacy and executor hardware commissioning
//! (`/api/v1/calibration/mit_timeout`).
//!
//! Models SG2002 SPI-CAN timing directly on wall-clock time:
//! - RX drain: 1.0–1.5 ms per release (real-arm rc2 measured `rx_us` p50 1.14 ms, p99 1.64 ms)
//! - TX: 0.4–0.6 ms/frame
//! - Motor CAN register reply latency: 0.5–1.5 ms
//!
//! Verifies:
//! 1. Legacy dry-run readback of J1–J6 completes in < 1 s with 0 skipped releases, max write
//!    guard hold <= 2 ms, and commissioning progress at least every 2 releases.
//! 2. Legacy write 0 -> 2000 of J1–J6 completes all Applied in < 2 s with 0 skipped releases,
//!    max write guard hold <= 2 ms, 0 flash writes (`0xAA`), and progress at least every 2 releases.
//! 3. Positive control: with the pre-fix `tick_start + 1 ms` budget cap, commissioning starves
//!    (0 register TX frames, never completes); with the release-deadline budget, the same bus
//!    completes all 6 joints.
//! 4. Tick publication precedes commissioning TX, an already-elapsed release cutoff emits 0 TX,
//!    and per-release register TX frames stay bounded (<= 4).
//! 5. Executor commissioning path does not starve under the same SG2002 RX/TX timing.

mod common;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::calibration::CalibrationDeps;
use armd::driver::{ArmDriver, DamiaoArmDriver, DAMIAO_TX_RELEASE_PUBLISH_MARGIN};
use armd::grasp::GraspDeps;
use armd::mit::{
    self, CommissioningAuthority, CommissioningOpKind, CommissioningOpState,
    CommissioningStartRequest, JointCommissioningDisposition,
};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::executor::ExecutorConfig;
use armd::rt_core::requests::config::{
    ConfigReply, ConfigRequest, CALIBRATION_PENDING_MARKER_NAME,
};
use armd::supervisor_lock::HoldKind;
use armd::transport::{CanFrame, CanRxTime, CanTransport, TransportError};
use armd::{ArmSupervisor, OperatorAuth};
use serde_json::json;
use tokio::net::TcpListener;

const SW_VER: u32 = 0x3831_3037;

fn pending_commit_marker_path(calib_path: &std::path::Path) -> std::path::PathBuf {
    calib_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join(CALIBRATION_PENDING_MARKER_NAME)
}

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn spin_for(dur: Duration) {
    if dur.is_zero() {
        return;
    }
    let start = Instant::now();
    while start.elapsed() < dur {
        std::hint::spin_loop();
    }
}

fn median_u64(samples: &[u64]) -> u64 {
    if samples.is_empty() {
        return 0;
    }
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    sorted[sorted.len() / 2]
}

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
    timeout_counts: u32,
    ctrl_mode: u32,
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
}

struct Sg2002WallBusState {
    rng: SeedRng,
    motors: [SgMotor; 7],
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    delayed: Vec<(Instant, CanFrame)>,
    tx_log: Vec<CanFrame>,
    timing_active: bool,
    rx_drain_min_us: u64,
    rx_drain_max_us: u64,
    tx_min_us: u64,
    tx_max_us: u64,
    reply_min_us: u64,
    reply_max_us: u64,
    /// Armed once per control release so Step 2's RX drain spends `rx_drain_min_us..=rx_drain_max_us`.
    rx_drain_armed: bool,
    rx_drain_samples_us: Vec<u64>,
    tx_frame_samples_us: Vec<u64>,
    /// Commissioning TX frames (`0x7FF` op `0x33`/`0x55`/`0xCC`) counted per control release.
    comm_tx_by_release: Vec<usize>,
    /// Register TX frames (`0x7FF` op `0x33`/`0x55`) counted per control release.
    reg_tx_by_release: Vec<usize>,
    /// Optional hook invoked on the first commissioning TX frame (used to verify ordering after publish).
    on_first_comm_tx: Option<Box<dyn FnMut() + Send>>,
}

impl Sg2002WallBusState {
    fn drain_matured(&mut self, now: Instant) {
        let mut i = 0;
        while i < self.delayed.len() {
            if self.delayed[i].0 <= now {
                let (at, f) = self.delayed.swap_remove(i);
                self.rx.push_back((f, Some(CanRxTime::Monotonic(at))));
            } else {
                i += 1;
            }
        }
    }

    fn push_all_feedback(&mut self) {
        let now = Instant::now();
        for m in &self.motors {
            self.rx
                .push_back((m.feedback_frame(), Some(CanRxTime::Monotonic(now))));
        }
    }

    fn begin_release(&mut self) {
        self.rx_drain_armed = true;
        self.comm_tx_by_release.push(0);
        self.reg_tx_by_release.push(0);
        if self.rx.len() < 14 {
            self.push_all_feedback();
        }
    }
}

#[derive(Clone)]
struct Sg2002WallBus(Arc<Mutex<Sg2002WallBusState>>);

impl Sg2002WallBus {
    fn new(seed: u64, reply_min_us: u64, reply_max_us: u64) -> Self {
        let cfg = ArmConfig::rebot_dm();
        let rest_pos = cfg.resting_positions();
        let limits = b601_limits();
        let motors = std::array::from_fn(|i| SgMotor {
            id: (i + 1) as u16,
            q: rest_pos.get(i).copied().unwrap_or(0.0),
            v: 0.0,
            temp_c: 31,
            enabled: false,
            timeout_counts: 0,
            ctrl_mode: damiao::CTRL_MODE_POS_VEL,
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(Sg2002WallBusState {
            rng: SeedRng::new(seed),
            motors,
            rx: VecDeque::with_capacity(256),
            delayed: Vec::new(),
            tx_log: Vec::new(),
            timing_active: false,
            rx_drain_min_us: 1_050,
            rx_drain_max_us: 1_400,
            tx_min_us: 400,
            tx_max_us: 600,
            reply_min_us,
            reply_max_us,
            rx_drain_armed: false,
            rx_drain_samples_us: Vec::new(),
            tx_frame_samples_us: Vec::new(),
            comm_tx_by_release: Vec::new(),
            reg_tx_by_release: Vec::new(),
            on_first_comm_tx: None,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut Sg2002WallBusState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn begin_release(&self) {
        self.with(|s| s.begin_release());
    }

    fn clear_metrics(&self) {
        self.with(|s| {
            s.tx_log.clear();
            s.rx_drain_samples_us.clear();
            s.tx_frame_samples_us.clear();
            s.comm_tx_by_release.clear();
            s.reg_tx_by_release.clear();
        });
    }
}

impl CanTransport for Sg2002WallBus {
    fn supports_bounded_flush(&self) -> bool {
        true
    }
    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let tx_dur = {
            let mut s = self.0.lock().unwrap();
            if s.timing_active {
                let (lo, hi) = (s.tx_min_us, s.tx_max_us);
                Duration::from_micros(s.rng.range(lo, hi))
            } else {
                Duration::ZERO
            }
        };
        let tx_start = Instant::now();
        spin_for(tx_dur);
        let tx_elapsed_us = tx_start.elapsed().as_micros() as u64;

        let now = Instant::now();
        let mut s = self.0.lock().unwrap();
        if s.timing_active {
            s.tx_frame_samples_us.push(tx_elapsed_us);
        }
        s.drain_matured(now);
        s.tx_log.push(*frame);

        if frame.id == 0x7FF {
            let op = frame.data[2];
            if matches!(op, damiao::REG_READ | damiao::REG_WRITE | 0xCC) {
                if let Some(last) = s.comm_tx_by_release.last_mut() {
                    *last += 1;
                }
                if matches!(op, damiao::REG_READ | damiao::REG_WRITE) {
                    if let Some(last) = s.reg_tx_by_release.last_mut() {
                        *last += 1;
                    }
                }
                if let Some(mut cb) = s.on_first_comm_tx.take() {
                    cb();
                }
            }
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let Some(k) = s.motors.iter().position(|m| m.id == id) else {
                return Ok(());
            };
            let reply_delay = if s.timing_active && s.reply_max_us > 0 {
                let (lo, hi) = (s.reply_min_us, s.reply_max_us);
                Duration::from_micros(s.rng.range(lo, hi))
            } else {
                Duration::ZERO
            };
            match op {
                0xCC => {
                    let fb = s.motors[k].feedback_frame();
                    if reply_delay.is_zero() {
                        s.rx.push_back((fb, Some(CanRxTime::Monotonic(now))));
                    } else {
                        s.delayed.push((now + reply_delay, fb));
                    }
                }
                damiao::REG_READ => {
                    let m = s.motors[k];
                    let val: Option<[u8; 4]> = match rid {
                        damiao::RID_CTRL_MODE => Some(m.ctrl_mode.to_le_bytes()),
                        damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                        damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                        damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                        damiao::RID_TIMEOUT => Some(m.timeout_counts.to_le_bytes()),
                        damiao::RID_ESC_ID => Some(u32::from(id).to_le_bytes()),
                        damiao::RID_MST_ID => Some((0x10 + u32::from(id)).to_le_bytes()),
                        damiao::RID_SW_VER => Some(SW_VER.to_le_bytes()),
                        _ => None,
                    };
                    if let Some(v) = val {
                        let [lo, hi] = id.to_le_bytes();
                        let reply = CanFrame::new_standard(
                            0x10 + id,
                            &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                        )
                        .unwrap();
                        if reply_delay.is_zero() {
                            s.rx.push_back((reply, Some(CanRxTime::Monotonic(now))));
                        } else {
                            s.delayed.push((now + reply_delay, reply));
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
                        s.motors[k].ctrl_mode = raw;
                    } else if rid == damiao::RID_TIMEOUT {
                        s.motors[k].timeout_counts = raw;
                    }
                    let reply = CanFrame::new_standard(0x10 + id, &frame.data).unwrap();
                    if reply_delay.is_zero() {
                        s.rx.push_back((reply, Some(CanRxTime::Monotonic(now))));
                    } else {
                        s.delayed.push((now + reply_delay, reply));
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
            s.rx.push_back((fb, Some(CanRxTime::Monotonic(now))));
        } else if let Some(k) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id)
        {
            let fb = s.motors[k].feedback_frame();
            s.rx.push_back((fb, Some(CanRxTime::Monotonic(now))));
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_with_rx_time().map(|(f, _)| f)
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        // Charge the SG2002 per-release RX drain latency (1.0–1.5 ms) once per control release
        // on the first RX drain call of that release.
        let drain_dur = {
            let mut s = self.0.lock().unwrap();
            if s.timing_active && s.rx_drain_armed {
                s.rx_drain_armed = false;
                let (lo, hi) = (s.rx_drain_min_us, s.rx_drain_max_us);
                Some(Duration::from_micros(s.rng.range(lo, hi)))
            } else {
                None
            }
        };
        if let Some(dur) = drain_dur {
            let drain_start = Instant::now();
            spin_for(dur);
            let actual_us = drain_start.elapsed().as_micros() as u64;
            self.0.lock().unwrap().rx_drain_samples_us.push(actual_us);
        }

        let now = Instant::now();
        let mut s = self.0.lock().unwrap();
        s.drain_matured(now);
        s.rx.pop_front().ok_or(TransportError::WouldBlock)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let (f, t) = self.receive_with_rx_time()?;
        let inst = t.map(|rt| match rt {
            CanRxTime::Monotonic(i) => i,
            CanRxTime::KernelRealtimeNs(_) => Instant::now(),
        });
        Ok((f, inst))
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn max_zero_progress_gap(per_release: &[usize]) -> usize {
    let Some(first) = per_release.iter().position(|&c| c > 0) else {
        return per_release.len();
    };
    let Some(last) = per_release.iter().rposition(|&c| c > 0) else {
        return per_release.len();
    };
    let mut cur = 0usize;
    let mut best = 0usize;
    for &c in &per_release[first..=last] {
        if c == 0 {
            cur += 1;
            best = best.max(cur);
        } else {
            cur = 0;
        }
    }
    best
}

fn build_sg2002_driver(bus: Sg2002WallBus) -> DamiaoArmDriver<Sg2002WallBus> {
    let limits = b601_limits();
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    bus.with(|s| {
        s.timing_active = false;
        s.push_all_feedback();
    });
    dm.read_joint_states().unwrap();
    let rc = dm.check_mapping_ranges();
    assert_eq!(rc.status, armd::driver::RangeCheckStatus::Passed);
    bus.with(|s| {
        s.timing_active = true;
    });
    dm
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn sg2002_legacy_mit_timeout_dry_run_and_write_meet_latency_and_guard_bounds() {
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("calibration.json");
    let setup_path = dir.path().join("grasp_setup.json");

    let bus = Sg2002WallBus::new(0x5602_0001, 500, 1_500);
    let dm = build_sg2002_driver(bus.clone());

    let cfg = ArmConfig {
        lease_timeout_ms: 15_000,
        workspace: None,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(dm)));

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

    // 50 Hz (20 ms) legacy control loop with scheduled release deadlines and skipped-release tracking.
    let stop_flag = Arc::new(AtomicBool::new(false));
    let ticks = Arc::new(AtomicU64::new(0));
    let skipped_releases = Arc::new(AtomicU64::new(0));
    let max_release_gap_us = Arc::new(AtomicU64::new(0));
    let measure_window = Arc::new(AtomicBool::new(false));

    let tick_sup = sup.clone();
    let tick_bus = bus.clone();
    let tick_stop = stop_flag.clone();
    let tick_cnt = ticks.clone();
    let tick_skip = skipped_releases.clone();
    let tick_gap = max_release_gap_us.clone();
    let tick_measure = measure_window.clone();
    let ticker_jh = tokio::spawn(async move {
        let period = Duration::from_millis(20);
        let mut next_release = Instant::now();
        let mut prev_at: Option<Instant> = None;
        while !tick_stop.load(Ordering::Relaxed) {
            let now = Instant::now();
            if now < next_release {
                tokio::time::sleep(next_release - now).await;
            }
            let release_start = Instant::now();
            let release_deadline = next_release + period;
            if tick_measure.load(Ordering::Acquire) {
                if let Some(prev) = prev_at {
                    let gap = release_start.saturating_duration_since(prev);
                    let gap_us = gap.as_micros().min(u128::from(u64::MAX)) as u64;
                    tick_gap.fetch_max(gap_us, Ordering::AcqRel);
                    if gap >= Duration::from_millis(38) {
                        tick_skip.fetch_add(1, Ordering::AcqRel);
                    }
                }
            }
            prev_at = Some(release_start);
            tick_bus.begin_release();
            tick_sup
                .control_tick_with_deadline(0.02, release_deadline)
                .await;
            let after = Instant::now();
            if after >= release_deadline && tick_measure.load(Ordering::Acquire) {
                tick_skip.fetch_add(1, Ordering::AcqRel);
            }
            next_release = release_deadline;
            while next_release + period <= after {
                next_release += period;
            }
            tick_cnt.fetch_add(1, Ordering::Release);
        }
    });

    let (mut ws, gen) = common::controller(addr).await;
    // Warm up 6 releases so K=3 causal rest-pose evidence is fresh.
    let t0 = ticks.load(Ordering::Acquire);
    while ticks.load(Ordering::Acquire) < t0 + 6 {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    // --- Phase 1: Dry-run readback of J1–J6 (must complete in < 1 s) ---
    let epoch = sup.get_observation().await.epoch_id;
    bus.clear_metrics();
    skipped_releases.store(0, Ordering::Release);
    max_release_gap_us.store(0, Ordering::Release);
    sup.inner().set_hold_history_enabled(true);
    measure_window.store(true, Ordering::Release);
    let ticks_before_dry = ticks.load(Ordering::Acquire);
    let dry_start = Instant::now();

    let dry_ack = common::rpc(
        &mut ws,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "dry_run": true }),
    )
    .await;

    let dry_elapsed = dry_start.elapsed();
    measure_window.store(false, Ordering::Release);
    let ticks_after_dry = ticks.load(Ordering::Acquire);
    let dry_holds = sup.inner().take_hold_history();
    let dry_skipped = skipped_releases.load(Ordering::Acquire);
    let dry_max_gap_us = max_release_gap_us.load(Ordering::Acquire);
    let dry_max_write_hold = dry_holds
        .iter()
        .filter(|h| h.kind == HoldKind::Write)
        .map(|h| h.hold())
        .max()
        .unwrap_or(Duration::ZERO);
    let (dry_rx_p50, dry_tx_p50, dry_progress_gap, dry_max_reg_tx) = bus.with(|s| {
        (
            median_u64(&s.rx_drain_samples_us),
            median_u64(&s.tx_frame_samples_us),
            max_zero_progress_gap(&s.comm_tx_by_release),
            s.reg_tx_by_release.iter().copied().max().unwrap_or(0),
        )
    });

    println!(
        "[SG2002-COMM dry_run J1-J6] elapsed_ms={:.2} releases={} skipped={} max_write_hold_us={} rx_p50_us={} tx_p50_us={} max_zero_progress_gap={} max_reg_tx_per_release={} max_gap_us={}",
        dry_elapsed.as_secs_f64() * 1e3,
        ticks_after_dry - ticks_before_dry,
        dry_skipped,
        dry_max_write_hold.as_micros(),
        dry_rx_p50,
        dry_tx_p50,
        dry_progress_gap,
        dry_max_reg_tx,
        dry_max_gap_us,
    );

    assert_eq!(dry_ack["type"], "command_ack", "expected dry_run command_ack: {dry_ack}");
    assert_eq!(dry_ack["dry_run"], true, "{dry_ack}");
    let rb = dry_ack["readback"].as_array().expect("readback array");
    assert_eq!(rb.len(), 6, "{dry_ack}");
    for (idx, item) in rb.iter().enumerate() {
        assert_eq!(item["joint"].as_u64(), Some((idx + 1) as u64), "{item}");
        assert_eq!(item["timeout_raw"].as_u64(), Some(0), "{item}");
    }
    assert!(
        (1_000..=1_500).contains(&dry_rx_p50),
        "SG2002 RX drain p50 must be 1.0–1.5 ms (got {dry_rx_p50} us)"
    );
    assert!(
        (400..=600).contains(&dry_tx_p50),
        "SG2002 TX frame p50 must be 0.4–0.6 ms (got {dry_tx_p50} us)"
    );
    assert!(
        dry_elapsed < Duration::from_secs(1),
        "SG2002 dry-run J1–J6 MUST complete in < 1 s (got {dry_elapsed:?})"
    );
    assert_eq!(dry_skipped, 0, "expected 0 skipped releases during dry-run");
    assert!(
        dry_max_write_hold <= Duration::from_millis(2),
        "max write guard hold during dry-run MUST stay <= 2 ms (got {dry_max_write_hold:?})"
    );
    assert!(
        dry_progress_gap < 2,
        "commissioning must make progress at least every 2 releases (max gap={dry_progress_gap})"
    );
    assert!(
        dry_max_reg_tx <= 4,
        "per-release register TX must stay bounded <= 4 (got {dry_max_reg_tx})"
    );

    // --- Phase 2: Write 0 -> 2000 for J1–J6 (must complete all Applied in < 2 s) ---
    let epoch = sup.get_observation().await.epoch_id;
    bus.clear_metrics();
    skipped_releases.store(0, Ordering::Release);
    max_release_gap_us.store(0, Ordering::Release);
    sup.inner().set_hold_history_enabled(true);
    measure_window.store(true, Ordering::Release);
    let ticks_before_wr = ticks.load(Ordering::Acquire);
    let wr_start = Instant::now();

    let wr_ack = common::rpc(
        &mut ws,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;

    let wr_elapsed = wr_start.elapsed();
    measure_window.store(false, Ordering::Release);
    let ticks_after_wr = ticks.load(Ordering::Acquire);
    let wr_holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);
    let wr_skipped = skipped_releases.load(Ordering::Acquire);
    let wr_max_gap_us = max_release_gap_us.load(Ordering::Acquire);
    let wr_max_write_hold = wr_holds
        .iter()
        .filter(|h| h.kind == HoldKind::Write)
        .map(|h| h.hold())
        .max()
        .unwrap_or(Duration::ZERO);
    let (wr_rx_p50, wr_tx_p50, wr_progress_gap, wr_max_reg_tx, flash_writes) = bus.with(|s| {
        (
            median_u64(&s.rx_drain_samples_us),
            median_u64(&s.tx_frame_samples_us),
            max_zero_progress_gap(&s.comm_tx_by_release),
            s.reg_tx_by_release.iter().copied().max().unwrap_or(0),
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == 0xAA)
                .count(),
        )
    });

    println!(
        "[SG2002-COMM write 0->2000 J1-J6] elapsed_ms={:.2} releases={} skipped={} max_write_hold_us={} rx_p50_us={} tx_p50_us={} max_zero_progress_gap={} max_reg_tx_per_release={} max_gap_us={} flash_writes={}",
        wr_elapsed.as_secs_f64() * 1e3,
        ticks_after_wr - ticks_before_wr,
        wr_skipped,
        wr_max_write_hold.as_micros(),
        wr_rx_p50,
        wr_tx_p50,
        wr_progress_gap,
        wr_max_reg_tx,
        wr_max_gap_us,
        flash_writes,
    );

    assert_eq!(wr_ack["type"], "command_ack", "expected write command_ack: {wr_ack}");
    assert_eq!(wr_ack["dry_run"], false, "{wr_ack}");
    assert_eq!(wr_ack["timeout_counts"], 2000, "{wr_ack}");
    let comm = wr_ack["commissioning"].as_array().expect("commissioning array");
    assert_eq!(comm.len(), 6, "{wr_ack}");
    for (idx, item) in comm.iter().enumerate() {
        assert_eq!(item["before"]["joint"].as_u64(), Some((idx + 1) as u64), "{item}");
        assert_eq!(item["readback"]["joint"].as_u64(), Some((idx + 1) as u64), "{item}");
        assert_eq!(item["written_raw"].as_u64(), Some(2000), "{item}");
        assert_eq!(item["ack_raw"].as_u64(), Some(2000), "{item}");
        assert_eq!(item["before"]["timeout_raw"].as_u64(), Some(0), "{item}");
        assert_eq!(item["readback"]["timeout_raw"].as_u64(), Some(2000), "{item}");
    }
    bus.with(|s| {
        for j in 0..6 {
            assert_eq!(
                s.motors[j].timeout_counts, 2000,
                "motor {} RID_TIMEOUT must be 2000",
                j + 1
            );
        }
    });
    assert_eq!(flash_writes, 0, "RAM-only commissioning must emit 0 flash writes");
    assert!(
        !pending_commit_marker_path(&calib_path).exists(),
        ".calibration_commit.pending must be cleaned up after all 6 joints applied"
    );
    assert!(
        (1_000..=1_500).contains(&wr_rx_p50),
        "SG2002 RX drain p50 must be 1.0–1.5 ms (got {wr_rx_p50} us)"
    );
    assert!(
        (400..=600).contains(&wr_tx_p50),
        "SG2002 TX frame p50 must be 0.4–0.6 ms (got {wr_tx_p50} us)"
    );
    assert!(
        wr_elapsed < Duration::from_secs(2),
        "SG2002 write 0->2000 J1–J6 MUST complete in < 2 s (got {wr_elapsed:?})"
    );
    assert_eq!(wr_skipped, 0, "expected 0 skipped releases during write");
    assert!(
        wr_max_write_hold <= Duration::from_millis(2),
        "max write guard hold during write MUST stay <= 2 ms (got {wr_max_write_hold:?})"
    );
    assert!(
        wr_progress_gap < 2,
        "commissioning must make progress at least every 2 releases (max gap={wr_progress_gap})"
    );
    assert!(
        wr_max_reg_tx <= 4,
        "per-release register TX must stay bounded <= 4 (got {wr_max_reg_tx})"
    );

    stop_flag.store(true, Ordering::Relaxed);
    let _ = ticker_jh.await;
    server_jh.abort();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sg2002_positive_control_old_1ms_tick_start_cap_starves() {
    let bus = Sg2002WallBus::new(0x5602_0002, 0, 0);
    let dm = build_sg2002_driver(bus.clone());

    let cfg = ArmConfig {
        lease_timeout_ms: 15_000,
        workspace: None,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(dm)));

    // Warm up feedback & acquire a direct lease so StartCommissioning succeeds.
    let (lease_gen, epoch, stop_gen) = {
        let mut inner = sup.inner().write().await;
        let gen = inner.acquire_lease("connection:1").unwrap();
        bus.begin_release();
        let mut sys_now = Instant::now;
        inner.tick_control_with_clock(0.02, &mut sys_now, Instant::now() + Duration::from_millis(20));
        (gen, inner.safety.epoch_id(), inner.current_stop_gen())
    };

    let mut joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        joints[i] = i;
    }
    let start_req = CommissioningStartRequest {
        kind: CommissioningOpKind::ReadIdentity,
        joints,
        joint_len: 6,
        timeout_counts: 0,
        references: [0.0; armd::rt_core::mailbox::MAX_DOF],
        authority: CommissioningAuthority {
            conn_id: 1,
            lease_holder: Some((armd::model::HolderLabel::new("connection:1"), lease_gen)),
            operator_grant: OperatorGrant::unenforced(),
            expected_epoch: epoch,
            expected_stop_gen: stop_gen,
            grant_id: None,
            txn_id: 0,
        },
        txn_deadline: Instant::now() + Duration::from_secs(15),
        prior_disk_mutated: false,
    };
    let op_id = match sup
        .dispatch_config_request(ConfigRequest::StartCommissioning(start_req))
        .await
        .expect("StartCommissioning")
    {
        ConfigReply::CommissioningStarted(Ok(id)) => id,
        _ => panic!("unexpected StartCommissioning reply"),
    };

    // Part A (Positive control): replicate the old `tick_wall_start + 1 ms` cap by passing
    // `release_deadline = tick_start + 1 ms + DAMIAO_TX_RELEASE_PUBLISH_MARGIN` (so
    // `comm_cutoff = release_deadline - margin = tick_start + 1 ms`). Because SG2002 RX drain
    // alone takes 1.05–1.40 ms, commissioning MUST starve (0 register TX across 20 releases).
    bus.clear_metrics();
    {
        let mut inner = sup.inner().write().await;
        for _ in 0..20 {
            bus.begin_release();
            let tick_start = Instant::now();
            let old_cap_release_deadline =
                tick_start + Duration::from_millis(1) + DAMIAO_TX_RELEASE_PUBLISH_MARGIN;
            let mut sys_now = Instant::now;
            inner.tick_control_with_clock(0.02, &mut sys_now, old_cap_release_deadline);
        }
    }
    let (old_tx_count, old_rx_p50) = bus.with(|s| {
        (
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ)
                .count(),
            median_u64(&s.rx_drain_samples_us),
        )
    });
    let old_snap = match sup
        .dispatch_config_request(ConfigRequest::PollCommissioning { op_id })
        .await
        .unwrap()
    {
        ConfigReply::CommissioningPolled(Some(s)) => s,
        _ => panic!("unexpected poll reply"),
    };
    println!(
        "[SG2002-COMM positive-control old-1ms-cap] releases=20 rx_p50_us={} reg_tx_sent={} state={:?}",
        old_rx_p50, old_tx_count, old_snap.state
    );
    assert_eq!(
        old_tx_count, 0,
        "positive control: old 1 ms cap MUST starve commissioning under SG2002 RX drain ({old_rx_p50} us)"
    );
    assert_eq!(
        old_snap.state,
        CommissioningOpState::Running,
        "starved commissioning must still be stuck Running under old 1 ms cap"
    );

    // Part B (Fixed release-deadline budget): on the exact same SupervisorInner and SG2002 bus,
    // pass the true 20 ms release deadline (`comm_cutoff = release_deadline - 2 ms`); commissioning
    // immediately makes progress every release and completes all 6 joints.
    bus.clear_metrics();
    let mut fixed_releases = 0usize;
    for _ in 0..40 {
        fixed_releases += 1;
        {
            let mut inner = sup.inner().write().await;
            bus.begin_release();
            let tick_start = Instant::now();
            let mut sys_now = Instant::now;
            inner.tick_control_with_clock(
                0.02,
                &mut sys_now,
                tick_start + Duration::from_millis(20),
            );
        }
        let snap = match sup
            .dispatch_config_request(ConfigRequest::PollCommissioning { op_id })
            .await
            .unwrap()
        {
            ConfigReply::CommissioningPolled(Some(s)) => s,
            _ => panic!("unexpected poll reply"),
        };
        if snap.state != CommissioningOpState::Running {
            break;
        }
    }
    let final_snap = match sup
        .dispatch_config_request(ConfigRequest::PollCommissioning { op_id })
        .await
        .unwrap()
    {
        ConfigReply::CommissioningPolled(Some(s)) => s,
        _ => panic!("unexpected poll reply"),
    };
    let (fixed_tx_count, fixed_rx_p50, fixed_tx_p50) = bus.with(|s| {
        (
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ)
                .count(),
            median_u64(&s.rx_drain_samples_us),
            median_u64(&s.tx_frame_samples_us),
        )
    });
    println!(
        "[SG2002-COMM fixed-release-budget] releases={} rx_p50_us={} tx_p50_us={} reg_tx_sent={} state={:?}",
        fixed_releases, fixed_rx_p50, fixed_tx_p50, fixed_tx_count, final_snap.state
    );
    assert_eq!(final_snap.state, CommissioningOpState::Done);
    assert_eq!(final_snap.joint_len, 6);
    for i in 0..6 {
        assert_eq!(
            final_snap.outcomes[i].disposition,
            JointCommissioningDisposition::Applied
        );
    }
    assert!(
        fixed_releases <= 20,
        "48 register reads at 4 frames/release must complete within 20 releases (got {fixed_releases})"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sg2002_legacy_tick_output_precedes_commissioning_and_release_cutoff_is_enforced() {
    let bus = Sg2002WallBus::new(0x5602_0003, 0, 0);
    let dm = build_sg2002_driver(bus.clone());
    let cfg = ArmConfig {
        lease_timeout_ms: 15_000,
        workspace: None,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(dm)));

    // Establish initial published observation and lease.
    bus.begin_release();
    sup.control_tick_with_deadline(0.02, Instant::now() + Duration::from_millis(20))
        .await;
    let seq_before = sup.hub().latest().expect("published obs").meta.seq;
    let (lease_gen, epoch, stop_gen) = {
        let mut inner = sup.inner().write().await;
        let gen = inner.acquire_lease("connection:1").unwrap();
        (gen, inner.safety.epoch_id(), inner.current_stop_gen())
    };

    let mut joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    joints[0] = 0;
    let start_req = CommissioningStartRequest {
        kind: CommissioningOpKind::ReadIdentity,
        joints,
        joint_len: 1,
        timeout_counts: 0,
        references: [0.0; armd::rt_core::mailbox::MAX_DOF],
        authority: CommissioningAuthority {
            conn_id: 1,
            lease_holder: Some((armd::model::HolderLabel::new("connection:1"), lease_gen)),
            operator_grant: OperatorGrant::unenforced(),
            expected_epoch: epoch,
            expected_stop_gen: stop_gen,
            grant_id: None,
            txn_id: 0,
        },
        txn_deadline: Instant::now() + Duration::from_secs(15),
        prior_disk_mutated: false,
    };
    let _op_id = match sup
        .dispatch_config_request(ConfigRequest::StartCommissioning(start_req))
        .await
        .unwrap()
    {
        ConfigReply::CommissioningStarted(Ok(id)) => id,
        _ => panic!("unexpected StartCommissioning reply"),
    };

    // 1. Expired release cutoff (`release_deadline - margin <= now`) must emit 0 commissioning TX.
    bus.clear_metrics();
    bus.begin_release();
    sup.control_tick_with_deadline(0.02, Instant::now() + Duration::from_millis(1))
        .await;
    let expired_tx = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ)
            .count()
    });
    assert_eq!(
        expired_tx, 0,
        "expired release cutoff must emit 0 commissioning TX (never delay control release)"
    );

    // 2. Normal release: tick observation publication (`hub.latest().seq`) MUST already have
    //    advanced BEFORE the first commissioning TX hits the wire, and register TX is capped <= 4.
    let seq_at_first_tx = Arc::new(AtomicU64::new(0));
    let seq_before_normal = sup.hub().latest().unwrap().meta.seq;
    let hub_clone = sup.hub().clone();
    let seq_out = seq_at_first_tx.clone();
    bus.with(|s| {
        s.tx_log.clear();
        s.on_first_comm_tx = Some(Box::new(move || {
            let seq = hub_clone.latest().map_or(0, |o| o.meta.seq);
            seq_out.store(seq, Ordering::Release);
        }));
    });
    bus.begin_release();
    sup.control_tick_with_deadline(0.02, Instant::now() + Duration::from_millis(20))
        .await;
    let observed_seq = seq_at_first_tx.load(Ordering::Acquire);
    let sent_in_one_release = bus.with(|s| {
        s.tx_log
            .iter()
            .filter(|f| f.id == 0x7FF && f.data[2] == damiao::REG_READ)
            .count()
    });
    assert!(
        observed_seq > seq_before_normal,
        "tick output & publish MUST precede commissioning TX (before={seq_before_normal}, at_first_tx={observed_seq}, init={seq_before})"
    );
    assert!(
        (1..=4).contains(&sent_in_one_release),
        "normal SG2002 release must send 1..=4 bounded commissioning frames (got {sent_in_one_release})"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn sg2002_executor_mit_timeout_dry_run_and_write_no_starvation() {
    let dir = tempfile::tempdir().unwrap();
    let calib_path = dir.path().join("calibration.json");
    let setup_path = dir.path().join("grasp_setup.json");

    // Executor path runs Step 2b synchronously on `armd-control` with zero sleep; model SG2002
    // RX drain (1.05–1.40 ms) and TX (0.4–0.6 ms/frame) with immediate CAN replies so each 20 ms
    // release processes bounded frames (up to 4) without starvation or skipped releases.
    let bus = Sg2002WallBus::new(0x5602_0004, 0, 0);
    let dm = build_sg2002_driver(bus.clone());

    let cfg = ArmConfig {
        lease_timeout_ms: 15_000,
        workspace: None,
        ..ArmConfig::rebot_dm()
    };
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(dm)));

    // Keep feeding fresh feedback and arming the per-release SG2002 RX drain at 50 Hz.
    let stop_flag = Arc::new(AtomicBool::new(false));
    let feeder_stop = stop_flag.clone();
    let feeder_bus = bus.clone();
    let feeder_jh = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        while !feeder_stop.load(Ordering::Relaxed) {
            interval.tick().await;
            feeder_bus.begin_release();
        }
    });

    let ecfg = ExecutorConfig::for_tests(Duration::from_millis(20));
    let handle = sup.start_executor(ecfg).await.unwrap();
    let dl = Instant::now() + Duration::from_secs(3);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl, "executor never published initial snapshot");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    tokio::time::sleep(Duration::from_millis(120)).await;

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

    let (mut ws, gen) = common::controller(addr).await;
    let epoch = sup.get_observation().await.epoch_id;
    let skipped_before = handle.stats().skipped_releases.load(Ordering::Acquire);
    bus.clear_metrics();

    let dry_start = Instant::now();
    let dry_ack = common::rpc(
        &mut ws,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "dry_run": true }),
    )
    .await;
    let dry_elapsed = dry_start.elapsed();
    assert_eq!(dry_ack["type"], "command_ack", "{dry_ack}");
    assert_eq!(dry_ack["readback"].as_array().map(Vec::len), Some(6), "{dry_ack}");
    assert!(
        dry_elapsed < Duration::from_secs(1),
        "executor SG2002 dry-run J1–J6 must complete in < 1 s (got {dry_elapsed:?})"
    );

    let wr_start = Instant::now();
    let wr_ack = common::rpc(
        &mut ws,
        gen,
        epoch,
        mit::TIMEOUT_PATH,
        json!({ "joints": [1, 2, 3, 4, 5, 6], "timeout_counts": 2000, "dry_run": false }),
    )
    .await;
    let wr_elapsed = wr_start.elapsed();
    let skipped_delta = handle.stats().skipped_releases.load(Ordering::Acquire) - skipped_before;
    let (rx_p50, tx_p50) = bus.with(|s| {
        (
            median_u64(&s.rx_drain_samples_us),
            median_u64(&s.tx_frame_samples_us),
        )
    });
    println!(
        "[SG2002-COMM executor J1-J6] dry_ms={:.2} write_ms={:.2} skipped_delta={} rx_p50_us={} tx_p50_us={}",
        dry_elapsed.as_secs_f64() * 1e3,
        wr_elapsed.as_secs_f64() * 1e3,
        skipped_delta,
        rx_p50,
        tx_p50,
    );
    assert_eq!(wr_ack["type"], "command_ack", "{wr_ack}");
    let comm = wr_ack["commissioning"].as_array().unwrap();
    assert_eq!(comm.len(), 6, "{wr_ack}");
    for (idx, item) in comm.iter().enumerate() {
        assert_eq!(item["readback"]["joint"].as_u64(), Some((idx + 1) as u64), "{item}");
        assert_eq!(item["written_raw"].as_u64(), Some(2000), "{item}");
        assert_eq!(item["ack_raw"].as_u64(), Some(2000), "{item}");
        assert_eq!(item["readback"]["timeout_raw"].as_u64(), Some(2000), "{item}");
    }
    assert!(
        wr_elapsed < Duration::from_secs(2),
        "executor SG2002 write 0->2000 J1–J6 must complete in < 2 s (got {wr_elapsed:?})"
    );
    assert_eq!(skipped_delta, 0, "executor must have 0 skipped releases");

    stop_flag.store(true, Ordering::Relaxed);
    let _ = feeder_jh.await;
    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .unwrap();
    server_jh.abort();
}
