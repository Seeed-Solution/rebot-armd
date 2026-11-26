//! Observation-only `TorqueController` SHADOW mode runtime, bounded driver staging,
//! lock-free SPSC ring transport, and off-thread comparator/diagnostics/blackbox sink.
//!
//! # Guarantees (Reducer PR1)
//! - **Zero live behavioral change**: live owner/driver logic remains the sole actuator authority.
//! - **Zero heap ops & zero lock attempts on RT**: normal releases, transition wakes, driver
//!   staging overflow, owner FIFO overflow, and outgoing SPSC ring saturation perform 0 heap
//!   allocations, 0 frees, and 0 mutex/lock attempts.
//! - **Single ordered input stream & outer-boundary flush**: driver observations are drained into
//!   `pending_inputs` before dropping any driver guard; owner events append to the same FIFO with
//!   their live edge timestamps; `flush_deferred_steps()` runs strictly at outer post-write
//!   boundaries (never mid-release before deadline-sensitive live TX).
//! - **Monotonic first-loss sequence cutoffs**: staging overflow and ring overflow publish
//!   monotonic `first_*_loss_seq` watermarks; records strictly before the cutoff are credited
//!   normally, while the cutoff and all subsequent records latch `interval_unverifiable = true`
//!   permanently for the session.

use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use rtrb::{Consumer, Producer, PushError, RingBuffer};
use serde::{Deserialize, Serialize};

use crate::diagnostics::rotating::RotatingFile;
use crate::model::ArmConfig;
use crate::motor_mode::ControlMode;
use crate::transport::DriverClock;

use super::invariants::{check_invariants, Violation};
use super::reducer::step;
use super::types::*;

// =========================================================================
// Constants
// =========================================================================

/// Fixed capacity of each driver's inline observation staging buffer (sized to absorb both
/// budgeted RT RX passes and up to a 1 s legacy blocking disable poll without spurious overflow).
pub const DRIVER_SHADOW_STAGING_CAP: usize = 1024;

/// Fixed capacity of the owner's unified ordered input FIFO per release/wake (`>= DRIVER_SHADOW_STAGING_CAP`).
pub const OWNER_PENDING_INPUTS_CAP: usize = 1024;

/// Preallocated capacity of the SPSC ring from producer (`SupervisorInner`) to off-thread comparator.
pub const SHADOW_RING_CAPACITY: usize = 1024;

/// Bounded recent-divergence ring exposed in `/api/v1/diagnostics` (`torque_shadow`).
pub const RECENT_DIVERGENCES_CAP: usize = 32;

/// Monotonic origin bias (µs, ~2.77 hours) so valid pre-origin timestamps stay strictly `< now_us`
/// without saturating to 0.
pub const SHADOW_ORIGIN_BIAS_US: u64 = 10_000_000_000;

/// Shadow blackbox rotating file name (kept strictly separate from primary `bb.jsonl`).
pub const SHADOW_BB_FILE: &str = "bb_shadow.jsonl";
/// Shadow blackbox rotation size (512 KiB).
pub const SHADOW_BB_MAX_BYTES: u64 = 512 * 1024;
/// Number of rotated shadow blackbox backups (`bb_shadow.jsonl.1`, `bb_shadow.jsonl.2`).
pub const SHADOW_BB_KEEP: usize = 2;

/// Environment variable overriding `ArmConfig::torque_shadow_enabled` at daemon startup.
pub const ENV_TORQUE_SHADOW: &str = "ARMD_TORQUE_SHADOW";

// =========================================================================
// Kill Switch & Config Resolution
// =========================================================================

/// Resolve the effective startup shadow-mode enable flag from profile config and optional
/// `ARMD_TORQUE_SHADOW` environment value.
///
/// Precedence: environment variable overrides profile flag when present and non-empty.
/// Accepted values (case-insensitive): `"1" | "true" | "on" | "yes"` => `true`,
/// `"0" | "false" | "off" | "no"` => `false`. Any other non-empty string fails startup.
pub fn resolve_shadow_enabled(profile_flag: bool, env_val: Option<&str>) -> Result<bool, String> {
    let Some(raw) = env_val.map(str::trim).filter(|s| !s.is_empty()) else {
        return Ok(profile_flag);
    };
    match raw.to_ascii_lowercase().as_str() {
        "1" | "true" | "on" | "yes" => Ok(true),
        "0" | "false" | "off" | "no" => Ok(false),
        _ => Err(format!(
            "Invalid {ENV_TORQUE_SHADOW}='{raw}': expected 1/0, true/false, on/off, or yes/no"
        )),
    }
}

/// Derive a validated [`TorqueConfig`] from an [`ArmConfig`] and mounted driver vendor family.
pub fn torque_config_from_arm_config(
    cfg: &ArmConfig,
    vendor: VendorKind,
    _has_driver: bool,
) -> TorqueConfig {
    let mut tc = TorqueConfig::default();
    let dof = cfg.dof.clamp(1, MAX_CHANNELS) as u8;
    tc.channel_count = dof;
    tc.recipient_count = dof;
    tc.group_count = 1;
    tc.device_count = 1;

    let poison_silence_us = match vendor {
        VendorKind::Feetech | VendorKind::FashionStar => FEETECH_POISON_SILENCE_US,
        _ => 0,
    };

    tc.devices[0] = DeviceConfig {
        id: 0,
        bus_id: 0,
        tx_quota_per_release: 16,
        reserved_safety_tx: 8,
        deadline_us: DEFAULT_DISABLE_DEADLINE_US,
        device_wide_mode_switch: false,
        poison_silence_us,
        panic_bus_loss_limit_us: DEFAULT_PANIC_BUS_LOSS_LIMIT_US,
    };

    let disable_repeats = match vendor {
        VendorKind::Damiao => DAMIAO_DISABLE_REPEATS,
        _ => 1,
    };

    tc.groups[0] = GroupConfig {
        id: 0,
        channels: ChannelMask::from_count(dof),
        recipients: ChannelMask::from_count(dof),
        devices: DeviceMask::bit(0),
        disable_repeats,
        disable_repeat_interval_us: DAMIAO_DISABLE_REPEAT_INTERVAL_US,
        disable_deadline_us: DEFAULT_DISABLE_DEADLINE_US,
        enable_deadline_us: DEFAULT_ENABLE_DEADLINE_US,
        has_park_path: cfg.resting_pose.is_some(),
    };

    let gripper_idx = cfg.gripper_index();
    let rest_positions = cfg.resting_positions();
    let rest_tol = if cfg.rest_pose_tolerance_rad.is_finite() && cfg.rest_pose_tolerance_rad > 0.0 {
        cfg.rest_pose_tolerance_rad
    } else {
        0.05
    };
    let trip_c = if cfg.max_temperature.is_finite() && cfg.max_temperature > 0.0 {
        cfg.max_temperature
    } else {
        70.0
    };
    let rest_qualified = !matches!(vendor, VendorKind::FashionStar);
    let has_pos_force_j7 = cfg
        .damiao_gripper
        .as_ref()
        .is_some_and(|g| g.mode == crate::model::DamiaoGripperMode::PosForce);

    for i in 0..dof {
        let idx = i as usize;
        let is_gripper = gripper_idx == Some(idx);
        let role = if is_gripper {
            JointRole::Gripper(GripperReleasePolicy::LegacyJ7)
        } else {
            JointRole::LoadBearing
        };
        let desired_mode = match vendor {
            VendorKind::Damiao => {
                if is_gripper && has_pos_force_j7 {
                    ControlMode::PositionForce
                } else {
                    ControlMode::PositionVelocity
                }
            }
            VendorKind::Feetech | VendorKind::FashionStar => ControlMode::Position,
            VendorKind::RobStride | VendorKind::GenericGrouped => ControlMode::PositionVelocity,
        };
        let rest_pos = rest_positions
            .get(idx)
            .copied()
            .filter(|v| v.is_finite())
            .unwrap_or(0.0);
        let max_vel = cfg
            .resting_pose
            .as_ref()
            .map(|rp| rp.max_velocity_of(idx))
            .filter(|v| v.is_finite() && *v > 0.0);

        tc.channels[idx] = ChannelConfig {
            id: i,
            device: 0,
            group: 0,
            recipient: i,
            role,
            active_tool_qualification: ActiveToolConfig::NotActiveTool,
            vendor,
            desired_mode,
            rest_position_rad: rest_pos,
            rest_tolerance_rad: if is_gripper { 0.0 } else { rest_tol },
            rest_max_velocity_rad_s: max_vel,
            supports_mit_damping: matches!(vendor, VendorKind::Damiao) && !is_gripper,
            damping: DampingConfig::default(),
            thermal: ThermalConfig {
                derate_start_c: None,
                trip_c,
            },
            damiao_timeout_reg: None,
            max_capture_age_us: DEFAULT_CAPTURE_MAX_AGE_US,
        };
        tc.recipients[idx] = RecipientConfig {
            id: i,
            device: 0,
            group: 0,
            channels: ChannelMask::bit(i),
            grouped_packet_mask: ChannelMask::bit(i),
            role,
            vendor,
            rest_evidence_qualified: rest_qualified,
        };
    }

    tc
}

// =========================================================================
// Owner, Clock & Provenance Types
// =========================================================================

/// Control owner producing shadow records.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ShadowOwner {
    #[default]
    Legacy,
    Executor,
}

/// Provenance of a sample or mode verification (never disguises native/mock assumptions as causal readback).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceProvenance {
    #[default]
    HardwareCausal,
    DriverNativeFixed,
    SimDriver,
    MockSynthetic,
}

/// All 5 live RX timestamp outcomes preserved into shadow normalization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxStampKind {
    /// Mapped monotonic RX instant with optional live clock offset drift in nanoseconds
    /// and live settle-boundary status.
    Valid {
        rx_at: Instant,
        drift_ns: u64,
        live_post_settle: bool,
    },
    /// Future-dated RX timestamp (`rx_at > decode_now` or rejected by kernel realtime future check).
    Future,
    /// Missing kernel/transport RX timestamp (`rx_time == None`).
    Missing { reset_window: bool },
    /// Present RX timestamp that could not be mapped to the monotonic write clock.
    Unmappable,
    /// Live clock-offset drift `> 1 ms` detected (triggers rest-window reset; CAN drivers also rebase settle).
    OffsetDrifted { rebased_settle: bool },
    /// Valid monotonic RX stamp, but live rest-evidence continuity prerequisites failed
    /// (e.g., unknown CAN queue bound, incomplete RX boundary, or invalid joint numeric state).
    /// Must NOT be credited to the K=3 rest window and must reset the rest window.
    UnqualifiedRestEvidence { rx_at: Instant, drift_ns: u64 },
}

/// Lock-free monotonic clock adapter anchored once at initialization.
pub struct ShadowClock {
    clock: Option<Arc<dyn DriverClock>>,
    origin: Instant,
    last_now_us: u64,
}

impl ShadowClock {
    pub fn new(clock: Option<Arc<dyn DriverClock>>) -> Self {
        let origin = clock.as_ref().map_or_else(Instant::now, |c| c.now());
        Self {
            clock,
            origin,
            last_now_us: SHADOW_ORIGIN_BIAS_US,
        }
    }

    /// Lock-free monotonic `Instant` in the driver's clock domain (or `Instant::now()` when driverless).
    #[inline]
    pub fn now_instant(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, |c| c.now())
    }

    /// Convert any `Instant` (including pre-origin instants) into biased monotonic µs.
    #[inline]
    pub fn instant_to_us(&self, ts: Instant) -> u64 {
        if ts >= self.origin {
            let delta = ts.duration_since(self.origin).as_micros().min(u128::from(u64::MAX)) as u64;
            SHADOW_ORIGIN_BIAS_US.saturating_add(delta)
        } else {
            let delta = self.origin.duration_since(ts).as_micros().min(u128::from(u64::MAX)) as u64;
            SHADOW_ORIGIN_BIAS_US.saturating_sub(delta)
        }
    }

    /// Advance monotonic owner `now_us` from an `Instant`.
    #[inline]
    pub fn advance_now_us(&mut self, ts: Instant) -> u64 {
        let raw = self.instant_to_us(ts);
        let mono = self.last_now_us.max(raw);
        self.last_now_us = mono;
        mono
    }

    /// Normalize a valid RX instant relative to `decode_now` and optional `last_motion_write_us`,
    /// preserving sub-microsecond future rejection (`rx_at > decode_now` => `now_us + 1`) and
    /// sub-microsecond strict `> 20 ms` post-motion settle boundaries.
    #[inline]
    pub fn normalize_valid_rx_us(
        &mut self,
        rx_at: Instant,
        decode_now: Instant,
        last_motion_write_us: Option<u64>,
        live_post_settle: bool,
    ) -> (u64, u64) {
        let mut now_us = self.advance_now_us(decode_now);
        if rx_at > decode_now {
            // Sub-microsecond future RX must never collapse onto `<= now_us`.
            return (now_us.saturating_add(1), now_us);
        }
        let mut rx_us = self.instant_to_us(rx_at);
        if rx_us > now_us {
            now_us = rx_us;
            self.last_now_us = now_us;
        }
        if live_post_settle {
            if let Some(mw_us) = last_motion_write_us {
                let min_post_settle = mw_us.saturating_add(REST_SETTLE_US).saturating_add(1);
                if rx_us < min_post_settle {
                    // Only bump across a sub-µs flooring edge (difference <= 1 µs); never fabricate settle.
                    if min_post_settle.saturating_sub(rx_us) <= 1 {
                        rx_us = min_post_settle;
                        if now_us < rx_us {
                            now_us = rx_us;
                            self.last_now_us = now_us;
                        }
                    }
                }
            }
        }
        (rx_us, now_us)
    }
}

// =========================================================================
// Driver-Side Bounded Inline Staging (Zero Alloc, Zero Lock)
// =========================================================================

/// Fixed-size `Copy` observation staged inline by drivers while already holding `&mut self`.
#[derive(Debug, Clone, Copy)]
pub enum ShadowDriverObs {
    CausalSample {
        channel: u8,
        sample_id: u64,
        stamp: RxStampKind,
        decode_now: Instant,
        position_rad: f32,
        velocity_rad_s: Option<f32>,
        temperature_c: Option<f32>,
        reported_enabled: Option<bool>,
        correlated_after_write_done: Option<Instant>,
        poll_issued: Option<Instant>,
        provenance: EvidenceProvenance,
    },
    WriteResult {
        device: u8,
        kind: WriteKind,
        write_done: Instant,
        now: Instant,
        target_mask: ChannelMask,
        outcomes: [MotorWriteOutcome; MAX_CHANNELS],
        /// True when a post-enqueue/flush error left wire delivery uncertain (`Ambiguous`).
        ambiguous_gap: bool,
    },
    ModeVerified {
        channel: u8,
        mode: ControlMode,
        now: Instant,
        provenance: EvidenceProvenance,
    },
    TimeoutReadback {
        channel: u8,
        raw_reg: u32,
        now: Instant,
    },
    SerialTimeout {
        device: u8,
        now: Instant,
    },
}

/// Heap-backed driver staging buffer with intrusive Treiber stack link so if an `ArmDriver` is
/// dropped on `armd-control` (executor exit/unwind), its staging allocation is retired lock-free
/// and freed off-thread (0 allocs, 0 frees, 0 locks on RT teardown).
struct DriverStagingBuf {
    buf: Box<[Option<ShadowDriverObs>]>,
    next_retired: *mut DriverStagingBuf,
}

// SAFETY: uniquely owned inside `ShadowDriverStaging` until retired via atomic Treiber stack.
unsafe impl Send for DriverStagingBuf {}
unsafe impl Sync for DriverStagingBuf {}

static RETIRED_DRIVER_STAGING_HEAD: AtomicPtr<DriverStagingBuf> =
    AtomicPtr::new(std::ptr::null_mut());

#[inline]
fn retire_driver_staging_buf(boxed: Box<DriverStagingBuf>) {
    let raw = Box::into_raw(boxed);
    loop {
        let head = RETIRED_DRIVER_STAGING_HEAD.load(Ordering::Acquire);
        // SAFETY: `raw` is uniquely owned before atomic publication.
        unsafe {
            (*raw).next_retired = head;
        }
        if RETIRED_DRIVER_STAGING_HEAD
            .compare_exchange_weak(head, raw, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            break;
        }
    }
}

fn reclaim_retired_driver_staging() {
    let mut cur = RETIRED_DRIVER_STAGING_HEAD.swap(std::ptr::null_mut(), Ordering::AcqRel);
    while !cur.is_null() {
        // SAFETY: exclusively reclaimed from the Treiber stack; created by `Box::into_raw`.
        let node = unsafe { Box::from_raw(cur) };
        cur = node.next_retired;
        drop(node);
    }
}

/// Bounded preallocated staging ring embedded in each driver struct (0 RT allocs, 0 locks, 0 RT frees).
pub struct ShadowDriverStaging {
    enabled: bool,
    len: usize,
    dropped: u64,
    sample_seq: u64,
    /// Deduplicates identical pre-transport refusal spins within a single driver guard call.
    last_refusal: Option<(WriteKind, ChannelMask, MotorWriteOutcome)>,
    /// Preallocated fixed-capacity heap slice when enabled (`None` when disabled -> 0 bytes allocated).
    buf: Option<Box<DriverStagingBuf>>,
}

impl Drop for ShadowDriverStaging {
    fn drop(&mut self) {
        if let Some(boxed) = self.buf.take() {
            if crate::rt_core::executor::on_rt_control_thread() {
                retire_driver_staging_buf(boxed);
            }
        }
    }
}

impl Default for ShadowDriverStaging {
    fn default() -> Self {
        Self::new(true)
    }
}

impl ShadowDriverStaging {
    pub fn new(enabled: bool) -> Self {
        let buf = if enabled {
            Some(Box::new(DriverStagingBuf {
                buf: vec![None; DRIVER_SHADOW_STAGING_CAP].into_boxed_slice(),
                next_retired: std::ptr::null_mut(),
            }))
        } else {
            None
        };
        Self {
            enabled,
            len: 0,
            dropped: 0,
            sample_seq: 0,
            last_refusal: None,
            buf,
        }
    }

    /// Allocated staging slot count (`0` when disabled).
    #[inline]
    pub fn allocated_capacity(&self) -> usize {
        self.buf.as_ref().map_or(0, |b| b.buf.len())
    }

    /// Enable or disable staging (called off-RT at driver install). Preallocates fixed buffer once.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.last_refusal = None;
        if enabled {
            if self.buf.as_ref().is_none_or(|b| b.buf.len() != DRIVER_SHADOW_STAGING_CAP) {
                self.buf = Some(Box::new(DriverStagingBuf {
                    buf: vec![None; DRIVER_SHADOW_STAGING_CAP].into_boxed_slice(),
                    next_retired: std::ptr::null_mut(),
                }));
            }
        } else {
            self.len = 0;
            self.dropped = 0;
        }
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    #[inline]
    pub fn next_sample_id(&mut self) -> u64 {
        self.sample_seq = self.sample_seq.wrapping_add(1).max(1);
        self.sample_seq
    }

    #[inline]
    pub fn push(&mut self, obs: ShadowDriverObs) {
        if !self.enabled {
            return;
        }
        if matches!(obs, ShadowDriverObs::WriteResult { .. }) {
            self.last_refusal = None;
        }
        let Some(ref mut inner) = self.buf else {
            self.dropped = self.dropped.saturating_add(1);
            return;
        };
        if self.len < inner.buf.len() {
            inner.buf[self.len] = Some(obs);
            self.len += 1;
        } else {
            self.dropped = self.dropped.saturating_add(1);
        }
    }

    /// Push a pre-transport write refusal, deduplicating repeated identical refusals within one
    /// driver guard call (e.g. tight blocking retry/poll loops) while always recording the first
    /// occurrence and any change in `(kind, mask, outcome)`.
    #[inline]
    pub fn push_dedup_refusal(
        &mut self,
        kind: WriteKind,
        mask: ChannelMask,
        outcome: MotorWriteOutcome,
        obs: ShadowDriverObs,
    ) {
        if !self.enabled {
            return;
        }
        let key = (kind, mask, outcome);
        if self.last_refusal == Some(key) {
            return;
        }
        self.last_refusal = Some(key);
        let Some(ref mut inner) = self.buf else {
            self.dropped = self.dropped.saturating_add(1);
            return;
        };
        if self.len < inner.buf.len() {
            inner.buf[self.len] = Some(obs);
            self.len += 1;
        } else {
            self.dropped = self.dropped.saturating_add(1);
        }
    }

    /// Copy staged observations out into `out`, returning `(copied, dropped_count)`.
    /// Resets the staging buffer cleanly with zero heap allocation.
    #[inline]
    pub fn drain_into(&mut self, out: &mut [Option<ShadowDriverObs>]) -> (usize, u64) {
        let Some(ref mut inner) = self.buf else {
            let dropped = self.dropped;
            self.len = 0;
            self.dropped = 0;
            self.last_refusal = None;
            return (0, dropped);
        };
        let count = self.len.min(out.len()).min(inner.buf.len());
        let overflow_from_small_out = self.len.saturating_sub(count) as u64;
        for (dst, src) in out.iter_mut().zip(inner.buf.iter_mut()).take(count) {
            *dst = src.take();
        }
        for slot in inner.buf.iter_mut().take(self.len).skip(count) {
            *slot = None;
        }
        let dropped = self.dropped.saturating_add(overflow_from_small_out);
        self.len = 0;
        self.dropped = 0;
        self.last_refusal = None;
        (count, dropped)
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

// =========================================================================
// Lock-Free Overflow & Blackbox Summary Atomics
// =========================================================================

/// Lock-free atomic loss watermarks & live summary counters shared between producer (`ShadowRuntime`),
/// off-thread comparator (`TorqueShadowHub`), and `BlackBox` piggyback summary.
pub struct ShadowOverflowAtomics {
    /// Monotonic highest input sequence allocated by `ShadowRuntime` (for lock-free contention cutoffs).
    pub last_allocated_seq: AtomicU64,
    /// Monotonic first input sequence lost to driver/owner staging overflow (`u64::MAX` = none).
    pub first_staging_loss_seq: AtomicU64,
    /// Monotonic first record sequence dropped on full outgoing SPSC ring (`u64::MAX` = none).
    pub first_ring_loss_seq: AtomicU64,
    pub staging_dropped_records: AtomicU64,
    pub staging_overflow_events: AtomicU64,
    pub ring_dropped_records: AtomicU64,
    pub ring_overflow_events: AtomicU64,
    /// Lock-free mirror of unexplained divergences total (written by comparator, read by `bb.jsonl`).
    pub unexplained_divergences_mirror: AtomicU64,
    /// Lock-free mirror of invariant violations total (written by comparator, read by `bb.jsonl`).
    pub invariant_violations_mirror: AtomicU64,
    /// Lock-free mirror of latched unverifiable status (written by producer on loss & comparator).
    pub interval_unverifiable_mirror: AtomicBool,
}

impl Default for ShadowOverflowAtomics {
    fn default() -> Self {
        Self::new()
    }
}

impl ShadowOverflowAtomics {
    pub fn new() -> Self {
        Self {
            last_allocated_seq: AtomicU64::new(0),
            first_staging_loss_seq: AtomicU64::new(u64::MAX),
            first_ring_loss_seq: AtomicU64::new(u64::MAX),
            staging_dropped_records: AtomicU64::new(0),
            staging_overflow_events: AtomicU64::new(0),
            ring_dropped_records: AtomicU64::new(0),
            ring_overflow_events: AtomicU64::new(0),
            unexplained_divergences_mirror: AtomicU64::new(0),
            invariant_violations_mirror: AtomicU64::new(0),
            interval_unverifiable_mirror: AtomicBool::new(false),
        }
    }

    #[inline]
    pub fn record_staging_loss(&self, first_lost_seq: u64, dropped_count: u64) {
        self.first_staging_loss_seq
            .fetch_min(first_lost_seq, Ordering::Release);
        self.staging_dropped_records
            .fetch_add(dropped_count, Ordering::Relaxed);
        self.staging_overflow_events.fetch_add(1, Ordering::Relaxed);
        self.interval_unverifiable_mirror
            .store(true, Ordering::Release);
    }

    #[inline]
    pub fn record_ring_loss(&self, seq: u64) {
        self.first_ring_loss_seq.fetch_min(seq, Ordering::Release);
        self.ring_dropped_records.fetch_add(1, Ordering::Relaxed);
        self.ring_overflow_events.fetch_add(1, Ordering::Relaxed);
        self.interval_unverifiable_mirror
            .store(true, Ordering::Release);
    }

    /// Compact summary piggybacked onto existing `bb.jsonl` lines ONLY when non-zero
    /// (`None` when clean, keeping clean `bb.jsonl` lines byte-identical).
    #[inline]
    pub fn blackbox_compact(&self) -> Option<ShadowBbCompact> {
        let d = self.unexplained_divergences_mirror.load(Ordering::Acquire);
        let inv = self.invariant_violations_mirror.load(Ordering::Acquire);
        let u = self.interval_unverifiable_mirror.load(Ordering::Acquire);
        let drop = self
            .ring_dropped_records
            .load(Ordering::Relaxed)
            .saturating_add(self.staging_dropped_records.load(Ordering::Relaxed));
        if d == 0 && inv == 0 && !u && drop == 0 {
            None
        } else {
            Some(ShadowBbCompact { d, inv, u, drop })
        }
    }
}

/// Compact shadow summary (~40 B) piggybacked onto `bb.jsonl` lines when non-clean.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShadowBbCompact {
    /// Unexplained divergences total.
    pub d: u64,
    /// Invariant violations total.
    #[serde(skip_serializing_if = "is_zero_u64")]
    pub inv: u64,
    /// Interval unverifiable (ring or staging overflow latched).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub u: bool,
    /// Total dropped records (ring + staging).
    #[serde(skip_serializing_if = "is_zero_u64")]
    pub drop: u64,
}

fn is_zero_u64(v: &u64) -> bool {
    *v == 0
}

// =========================================================================
// Compact Comparison, Taxonomy & SPSC Record Types
// =========================================================================

/// Compact summary of one `torque::step` `ActionSet` (fixed-size `Copy`, no heap).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionSummary {
    pub hold_mask: ChannelMask,
    pub damp_mask: ChannelMask,
    pub enable_mask: ChannelMask,
    pub enable_attempt_id: u64,
    pub disable_kind: Option<DisableKind>,
    pub disable_recipients: ChannelMask,
    pub disable_dispatch_id: u64,
    pub disable_deadline_us: u64,
    pub disable_repeat_index: u8,
    pub disable_completed: bool,
    pub disable_partial: bool,
    pub disable_timed_out: bool,
    pub refusal: Option<RefusalCode>,
    pub motion_allowed: Option<ChannelMask>,
    pub park_requested: bool,
    pub park_id: u64,
    pub contention_escalated: DeviceMask,
    pub unheld_mask: ChannelMask,
}

impl ActionSummary {
    pub fn from_action_set(actions: &ActionSet) -> Self {
        let mut hold_mask = ChannelMask::EMPTY;
        let mut damp_mask = ChannelMask::EMPTY;
        for c in 0..MAX_CHANNELS {
            if actions.holds[c].is_some() {
                hold_mask.insert(c as u8);
            }
            if actions.damps[c].is_some() {
                damp_mask.insert(c as u8);
            }
        }
        let (enable_mask, enable_attempt_id) = actions.enables[0]
            .map_or((ChannelMask::EMPTY, 0), |e| (e.channels, e.attempt_id));
        let (
            disable_kind,
            disable_recipients,
            disable_dispatch_id,
            disable_deadline_us,
            disable_repeat_index,
        ) = actions.disables[0].map_or((None, ChannelMask::EMPTY, 0, 0, 0), |d| {
            (
                Some(d.kind),
                d.recipients,
                d.dispatch_id,
                d.deadline_us,
                d.repeat_index,
            )
        });
        let (disable_completed, disable_partial, disable_timed_out) = actions.disable_completions[0]
            .map_or((false, false, false), |c| {
                (true, c.partial_or_ambiguous, c.timed_out)
            });
        let refusal = actions.refusals[0]
            .or(actions.invalid_group_refusal)
            .map(|r| r.code);
        let motion_allowed = actions.motion_allowed[0].map(|m| m.channels);
        let (park_requested, park_id) = actions.park_requests[0]
            .map_or((false, 0), |p| (true, p.park_id));

        Self {
            hold_mask,
            damp_mask,
            enable_mask,
            enable_attempt_id,
            disable_kind,
            disable_recipients,
            disable_dispatch_id,
            disable_deadline_us,
            disable_repeat_index,
            disable_completed,
            disable_partial,
            disable_timed_out,
            refusal,
            motion_allowed,
            park_requested,
            park_id,
            contention_escalated: actions.contention_escalations,
            unheld_mask: actions.telemetry.unheld_mask,
        }
    }
}

/// Live refusal classification observed at owner entry points.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LiveRefusalKind {
    TorqueOffWouldDrop,
    ArmNotAtRestingPose,
    EnableFailed,
    DisableInProgress,
    FaultLatched,
    EStopActive,
    BusyOrTransition,
    DriverBusy,
    Unsupported,
    Other,
}

/// Live owner decision captured at the tap edge for comparator join.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LiveDecisionSummary {
    /// True when this record carries a live decision checkpoint to compare against reducer output.
    pub comparable: bool,
    pub live_intent: TorqueIntent,
    pub live_is_torqued: bool,
    pub live_stopped: bool,
    pub live_hold_mask: ChannelMask,
    /// Joints held by live using previously-captured stale targets (`JointHoldStatus::HeldStaleTarget`).
    #[serde(default)]
    pub live_stale_held_mask: ChannelMask,
    pub live_enable_started: bool,
    pub live_disable_kind: Option<DisableKind>,
    pub live_disable_recipients: ChannelMask,
    pub live_refusal: Option<LiveRefusalKind>,
    pub live_motion_allowed: Option<bool>,
    pub live_partial_disable: bool,
    pub live_contention_escalated: bool,
}

/// Capability / operation join metadata attached to each `ShadowRecord`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct JoinMeta {
    pub live_op_id: u64,
    pub capability_id: u64,
    pub generation: u64,
    pub group: u8,
    pub device: u8,
    pub recipient_mask: ChannelMask,
    pub unmatched_live_write: bool,
}

/// Explicit coverage gap tags surfaced in diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CoverageGapKind {
    MissingRxTimestamp,
    UnmappableRxTimestamp,
    TimeoutReadbackUnavailable,
    TimeoutReadbackOutOfRange,
    NativeModeAssumed,
    GraspForceCeilingMissing,
    WorkerPanicHookAbsent,
    BusLossDeviceAttribution,
    AmbiguousWriteOutcomeUnavailable,
    MockSyntheticSample,
}

/// Explicitly reviewed PR2-only differences (visible in diagnostics, not unexplained mismatches).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReviewedPr2Kind {
    ProtectiveDamping,
    ThermalDerating,
    TimeoutReadbackGate,
    FuPanicBaselineMismatch,
    ProcessExitAlreadyDisabled,
    LiveDriftSettleRebase,
    DriverPreflightVsAwaitRestGate,
    /// Live continued holding a previously-captured joint (`HeldStaleTarget`) while its feedback
    /// briefly aged beyond `max_capture_age_us`, verified against reducer `had_valid_capture`.
    LiveHeldStaleTarget,
}

/// Primary divergence taxonomy from `reducer-pr1-shadow-spec.md`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DivergenceCategory {
    ExtraAction,
    MissingAction,
    AuthorityRecipientMismatch,
    CausalTimingOrder,
    OutcomeRefusalMismatch,
    InvariantViolation,
    CoverageGap,
    ReviewedPr2Only,
}

/// Discriminant for SPSC records sent to the off-thread comparator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ShadowRecordKind {
    Step,
    JointZeroRecalibrated,
    CoverageGapOnly,
}

/// Compact fixed-size `Copy` record pushed over the preallocated SPSC ring (`rtrb`).
#[derive(Debug, Clone, Copy)]
pub struct ShadowRecord {
    pub seq: u64,
    pub owner: ShadowOwner,
    pub release_id: u64,
    pub now_us: u64,
    pub kind: ShadowRecordKind,
    pub event: Option<Event>,
    pub rt_actions: ActionSummary,
    /// Predicted hold mask from the most recent `Release` step, carried separately from
    /// `rt_actions` so `LockContended` replay comparisons never see a mutated action summary.
    pub predicted_hold_mask: ChannelMask,
    pub live: LiveDecisionSummary,
    pub join: JoinMeta,
    pub post_drive_state: DriveState,
    pub post_intent: TorqueIntent,
    pub coverage_gap: Option<CoverageGapKind>,
    pub reviewed_pr2: Option<ReviewedPr2Kind>,
}

/// Test-only fault injection hook (never exposed on HTTP/WS/CLI/env/profile JSON).
#[doc(hidden)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ShadowFaultInjection {
    #[default]
    None,
    /// Inject a spurious disable command into RT `ActionSummary` on the next step.
    InjectSpuriousDisable { recipients: ChannelMask },
    /// Suppress predicted holds on the next release comparison.
    SuppressHolds,
    /// Flip refusal to `TorqueOffWouldDrop` on the next comparable decision.
    InjectRefusal(RefusalCode),
    /// Corrupt recipient mask on the next disable command.
    CorruptDisableRecipients(ChannelMask),
}

// =========================================================================
// Unified Ordered Input FIFO (`pending_inputs`) & Inline `ShadowRuntime`
// =========================================================================

/// One ordered input item captured at its exact live edge (`driver` or `owner`) and reduced
/// strictly at the outer post-write boundary.
#[derive(Debug, Clone, Copy)]
pub struct ShadowInput {
    pub seq: u64,
    pub edge_instant: Instant,
    pub kind: ShadowInputKind,
}

#[derive(Debug, Clone, Copy)]
pub enum ShadowInputKind {
    DriverObs(ShadowDriverObs),
    Release {
        release_seq: u64,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    TransitionWake {
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    EnableStart {
        live_op_id: u64,
        live_refusal: Option<LiveRefusalKind>,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    EnableCancelledPreTx {
        live_op_id: u64,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    HandshakeResult {
        live_op_id: u64,
        confirmed_mask: ChannelMask,
        outcome: HandshakeOutcome,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    DisableRequest {
        live_op_id: u64,
        explicit_release_authorized: bool,
        live_refusal: Option<LiveRefusalKind>,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    Stop {
        kind: StopKind,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    FaultClear {
        live_refusal: Option<LiveRefusalKind>,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    Thermal {
        channel: u8,
        temperature_c: f32,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    BusLoss {
        device: u8,
        multi_device_gap: bool,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    MotionCommand {
        channels: ChannelMask,
        live_allowed: bool,
        live_refusal: Option<LiveRefusalKind>,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    GraspStateUpdate {
        channel: u8,
        setpoint_rad: f32,
        force_limit_raw: Option<u16>,
    },
    MitOwnershipChange {
        channels: ChannelMask,
        target_mode: ControlMode,
        device_wide: bool,
    },
    ProcessExit {
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    Panic {
        tier: PanicTier,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    LiveHoldCheckpoint {
        hold_mask: ChannelMask,
        stale_held_mask: ChannelMask,
        contended: bool,
        escalated: bool,
        live_intent: TorqueIntent,
        live_is_torqued: bool,
        live_stopped: bool,
    },
    ParkLifecycle {
        live_op_id: u64,
        admitted: bool,
        settled_ok: Option<bool>,
    },
    JointZeroRecalibrated,
    CoverageGap {
        gap: CoverageGapKind,
    },
}

#[derive(Debug, Clone, Copy, Default)]
struct EnableBinding {
    active: bool,
    live_op_id: u64,
    attempt_id: u64,
    generation: u64,
    channels: ChannelMask,
}

#[derive(Debug, Clone, Copy, Default)]
struct DisableBinding {
    active: bool,
    live_op_id: u64,
    tx_id: u64,
    dispatch_id: u64,
    generation: u64,
    kind: Option<DisableKind>,
}

#[derive(Debug, Clone, Copy, Default)]
struct ParkBinding {
    active: bool,
    is_tier1_panic: bool,
    park_id: Option<u64>,
    generation: u64,
}

/// Zero-cost wrapper making `rtrb::Producer<T>` `Sync` by exposing ONLY `&mut self` access.
///
/// SAFETY: `Producer<T>` is `Send` (for `T: Send`) and is never accessed through `&self`
/// (shared references cannot call any method or touch the inner `Cell`), so sharing
/// `&ExclusiveProducer<T>` across threads cannot introduce data races.
struct ExclusiveProducer<T>(Producer<T>);

// SAFETY: `ExclusiveProducer<T>` provides no `&self` access to the inner `Producer<T>`,
// and all mutation requires `&mut self` (exclusive ownership).
unsafe impl<T: Send> Sync for ExclusiveProducer<T> {}

impl<T> ExclusiveProducer<T> {
    #[inline]
    fn new(tx: Producer<T>) -> Self {
        Self(tx)
    }

    #[inline]
    fn push(&mut self, value: T) -> Result<(), PushError<T>> {
        self.0.push(value)
    }
}

/// Heap-backed producer state bundled behind a single `Option<Box<_>>` so:
/// 1. Disabled runtimes allocate ZERO bytes of reducer state, FIFO, scratch, or SPSC ring.
/// 2. On RT control-thread teardown or unwind, `ShadowRuntime::drop` retires this box
///    lock-free into `ShadowHubCore::retired_head` (0 allocs, 0 frees, 0 locks on RT) so
///    off-thread reclamation frees it outside `armd-control`.
struct ShadowRuntimeInner {
    state: Box<TorqueState>,
    pending_inputs: Box<[Option<ShadowInput>]>,
    driver_scratch: Box<[Option<ShadowDriverObs>]>,
    ring_tx: ExclusiveProducer<ShadowRecord>,
    /// Intrusive lock-free Treiber stack link for off-thread drop reclamation.
    next_retired: *mut ShadowRuntimeInner,
}

// SAFETY: `ShadowRuntimeInner` is uniquely owned by `ShadowRuntime` (access requires `&mut self`
// on `ExclusiveProducer`), and `next_retired` is only written while uniquely owned or popped
// exclusively by off-thread reclamation.
unsafe impl Send for ShadowRuntimeInner {}
unsafe impl Sync for ShadowRuntimeInner {}

/// Inline shadow runtime owned by `SupervisorInner` (one per active owner).
///
/// Zero heap allocations and zero locks after initialization, and zero frees on RT teardown.
pub struct ShadowRuntime {
    enabled: bool,
    owner: ShadowOwner,
    vendor: VendorKind,
    clock: ShadowClock,
    next_input_seq: u64,
    next_live_op_id: u64,
    last_release_hold_mask: ChannelMask,
    enable_binding: EnableBinding,
    disable_binding: DisableBinding,
    park_binding: ParkBinding,
    pending_len: usize,
    /// `None` when disabled (0 bytes allocated) or after retiring to off-thread drop.
    inner: Option<Box<ShadowRuntimeInner>>,
    overflow: Arc<ShadowOverflowAtomics>,
    /// Retained handle to the paired off-thread hub (used by direct `SupervisorInner` tests and
    /// cloned into `ArmSupervisor` at startup; never locked on RT!).
    hub: Option<Arc<TorqueShadowHub>>,
    /// MIT ownership mask cleared during a timed panic disable; emitted strictly after outcome seal.
    deferred_mit_off_mask: ChannelMask,
    #[doc(hidden)]
    injected_fault: ShadowFaultInjection,
}

impl Drop for ShadowRuntime {
    fn drop(&mut self) {
        if let Some(boxed) = self.inner.take() {
            if let Some(ref hub) = self.hub {
                hub.retire_runtime_inner(boxed);
            }
        }
    }
}

impl ShadowRuntime {
    /// Disabled runtime with zero reducer/ring/buffer allocation (`torque_shadow_enabled = false` or `detached_shell`).
    pub fn disabled(_cfg: &ArmConfig) -> Self {
        Self {
            enabled: false,
            owner: ShadowOwner::Legacy,
            vendor: VendorKind::Damiao,
            clock: ShadowClock::new(None),
            next_input_seq: 1,
            next_live_op_id: 1,
            last_release_hold_mask: ChannelMask::EMPTY,
            enable_binding: EnableBinding::default(),
            disable_binding: DisableBinding::default(),
            park_binding: ParkBinding::default(),
            pending_len: 0,
            inner: None,
            overflow: Arc::new(ShadowOverflowAtomics::new()),
            hub: None,
            deferred_mit_off_mask: ChannelMask::EMPTY,
            injected_fault: ShadowFaultInjection::None,
        }
    }

    /// Construct a new inline `ShadowRuntime` and its paired [`TorqueShadowHub`].
    /// When `enabled == false`, skips `TorqueState`, `rtrb::RingBuffer`, and FIFO allocation completely.
    pub fn new_with_hub(
        cfg: &ArmConfig,
        enabled: bool,
        vendor: VendorKind,
        has_driver: bool,
        driver_clock: Option<Arc<dyn DriverClock>>,
    ) -> (Self, Arc<TorqueShadowHub>) {
        if !enabled {
            let overflow = Arc::new(ShadowOverflowAtomics::new());
            let hub = Arc::new(TorqueShadowHub::disabled(Arc::clone(&overflow)));
            let rt = Self {
                enabled: false,
                owner: ShadowOwner::Legacy,
                vendor,
                clock: ShadowClock::new(driver_clock),
                next_input_seq: 1,
                next_live_op_id: 1,
                last_release_hold_mask: ChannelMask::EMPTY,
                enable_binding: EnableBinding::default(),
                disable_binding: DisableBinding::default(),
                park_binding: ParkBinding::default(),
                pending_len: 0,
                inner: None,
                overflow,
                hub: Some(Arc::clone(&hub)),
                deferred_mit_off_mask: ChannelMask::EMPTY,
                injected_fault: ShadowFaultInjection::None,
            };
            return (rt, hub);
        }
        let tc = torque_config_from_arm_config(cfg, vendor, has_driver);
        Self::from_torque_config(tc, true, vendor, driver_clock)
    }

    /// Construct from an explicit [`TorqueConfig`] (also used for synthetic grouped-hand tests).
    pub fn from_torque_config(
        tc: TorqueConfig,
        enabled: bool,
        vendor: VendorKind,
        driver_clock: Option<Arc<dyn DriverClock>>,
    ) -> (Self, Arc<TorqueShadowHub>) {
        let overflow = Arc::new(ShadowOverflowAtomics::new());
        if !enabled {
            let hub = Arc::new(TorqueShadowHub::disabled(Arc::clone(&overflow)));
            let rt = Self {
                enabled: false,
                owner: ShadowOwner::Legacy,
                vendor,
                clock: ShadowClock::new(driver_clock),
                next_input_seq: 1,
                next_live_op_id: 1,
                last_release_hold_mask: ChannelMask::EMPTY,
                enable_binding: EnableBinding::default(),
                disable_binding: DisableBinding::default(),
                park_binding: ParkBinding::default(),
                pending_len: 0,
                inner: None,
                overflow,
                hub: Some(Arc::clone(&hub)),
                deferred_mit_off_mask: ChannelMask::EMPTY,
                injected_fault: ShadowFaultInjection::None,
            };
            return (rt, hub);
        }

        let mut state = Box::new(TorqueState::new(tc).expect("valid TorqueConfig"));
        let has_mode_rb = has_causal_mode_readback(vendor);
        if !has_mode_rb {
            seed_state_native_modes(&mut state);
        }
        let (tx, rx) = RingBuffer::<ShadowRecord>::new(SHADOW_RING_CAPACITY);
        let consumer_state = state.clone();
        let hub = Arc::new(TorqueShadowHub::enabled(
            rx,
            consumer_state,
            Arc::clone(&overflow),
            ShadowOwner::Legacy,
        ));
        let inner = Box::new(ShadowRuntimeInner {
            state,
            pending_inputs: vec![None; OWNER_PENDING_INPUTS_CAP].into_boxed_slice(),
            driver_scratch: vec![None; DRIVER_SHADOW_STAGING_CAP].into_boxed_slice(),
            ring_tx: ExclusiveProducer::new(tx),
            next_retired: std::ptr::null_mut(),
        });
        let rt = Self {
            enabled: true,
            owner: ShadowOwner::Legacy,
            vendor,
            clock: ShadowClock::new(driver_clock),
            next_input_seq: 1,
            next_live_op_id: 1,
            last_release_hold_mask: ChannelMask::EMPTY,
            enable_binding: EnableBinding::default(),
            disable_binding: DisableBinding::default(),
            park_binding: ParkBinding::default(),
            pending_len: 0,
            inner: Some(inner),
            overflow,
            hub: Some(Arc::clone(&hub)),
            deferred_mit_off_mask: ChannelMask::EMPTY,
            injected_fault: ShadowFaultInjection::None,
        };
        (rt, hub)
    }

    /// Record MIT channels cleared inside a timed panic disable without sampling the clock or
    /// enqueueing until after the live panic deadline observer seals the outcome.
    #[inline]
    pub fn note_deferred_mit_off(&mut self, mask: ChannelMask) {
        self.deferred_mit_off_mask = ChannelMask(self.deferred_mit_off_mask.raw() | mask.raw());
    }

    /// Consume and clear any deferred MIT-off mask recorded during a timed panic disable.
    #[inline]
    pub fn take_deferred_mit_off(&mut self) -> ChannelMask {
        let m = self.deferred_mit_off_mask;
        self.deferred_mit_off_mask = ChannelMask::EMPTY;
        m
    }
}

/// Live supervisor state snapshot synchronized into both inline and off-thread `TorqueState`
/// when mounting or remounting a driver (`reinit_for_driver_synced`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LiveBootSync {
    pub live_is_torqued: bool,
    pub live_stopped: bool,
    pub estop_latched: bool,
    pub has_fault: bool,
    pub failed_hold: bool,
}

impl ShadowRuntime {
    /// Re-initialize inline `TorqueState`, clock, and consumer mirror when a driver is mounted.
    ///
    /// Preserves session-wide overflow latches (`first_*_loss_seq`, `interval_unverifiable`),
    /// monotonic sequence numbers, divergence counters, and `TorqueShadowHub` identity across
    /// driver remounts. Bypasses all allocation when disabled.
    pub fn reinit_for_driver(
        &mut self,
        cfg: &ArmConfig,
        vendor: VendorKind,
        driver_clock: Option<Arc<dyn DriverClock>>,
    ) {
        self.reinit_for_driver_synced(cfg, vendor, driver_clock, LiveBootSync::default());
    }

    /// Re-initialize inline `TorqueState`, clock, and consumer mirror when a driver is mounted or
    /// remounted, seeding the initial group state from `sync` so boot/remount starts in sync with
    /// the live supervisor (cold boot => `SwitchOnDisabled/Off`; remount while armed/stopped/faulted
    /// => matching state).
    pub fn reinit_for_driver_synced(
        &mut self,
        cfg: &ArmConfig,
        vendor: VendorKind,
        driver_clock: Option<Arc<dyn DriverClock>>,
        sync: LiveBootSync,
    ) {
        // Flush any queued old-driver FIFO inputs in the PREVIOUS driver's clock domain before
        // installing the new driver clock and reinitializing the state mirrors.
        self.flush_deferred_steps();
        self.vendor = vendor;
        self.clock = ShadowClock::new(driver_clock);
        if !self.enabled {
            return;
        }
        // Flush any already-queued pre-remount records cleanly before switching driver topology.
        self.flush_deferred_steps();
        if let Some(ref hub) = self.hub {
            hub.drain_now();
        }
        let tc = torque_config_from_arm_config(cfg, vendor, true);
        let mut new_state = Box::new(TorqueState::new(tc).expect("valid derived TorqueConfig"));
        let has_mode_rb = has_causal_mode_readback(vendor);
        if !has_mode_rb {
            seed_state_native_modes(&mut new_state);
        }
        apply_live_boot_sync(&mut new_state, sync);
        self.pending_len = 0;
        self.enable_binding = EnableBinding::default();
        self.disable_binding = DisableBinding::default();
        self.park_binding = ParkBinding::default();
        self.last_release_hold_mask = ChannelMask::EMPTY;

        let (tx, rx) = RingBuffer::<ShadowRecord>::new(SHADOW_RING_CAPACITY);
        let consumer_state = new_state.clone();
        if let Some(ref mut inner) = self.inner {
            *inner.state = *new_state;
            for slot in inner.pending_inputs.iter_mut() {
                *slot = None;
            }
            inner.ring_tx = ExclusiveProducer::new(tx);
        } else {
            self.inner = Some(Box::new(ShadowRuntimeInner {
                state: new_state,
                pending_inputs: vec![None; OWNER_PENDING_INPUTS_CAP].into_boxed_slice(),
                driver_scratch: vec![None; DRIVER_SHADOW_STAGING_CAP].into_boxed_slice(),
                ring_tx: ExclusiveProducer::new(tx),
                next_retired: std::ptr::null_mut(),
            }));
        }
        // Preserve `next_input_seq` and all session overflow watermarks/counters across remount!
        if let Some(ref hub) = self.hub {
            hub.reinit_consumer_preserving_session(rx, consumer_state, self.next_input_seq);
        }
    }

    /// Record staging loss when a driver guard could not drain normally (e.g. panic unwind or
    /// abandoned timed-out driver call), latching session `interval_unverifiable` immediately.
    #[inline]
    pub fn record_external_staging_loss(&mut self, dropped: u64) {
        if !self.enabled || dropped == 0 {
            return;
        }
        let first_lost = self.alloc_seq();
        self.next_input_seq = self
            .next_input_seq
            .saturating_add(dropped.saturating_sub(1));
        self.overflow
            .last_allocated_seq
            .fetch_max(self.next_input_seq.saturating_sub(1), Ordering::Release);
        self.overflow.record_staging_loss(first_lost, dropped);
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    #[inline]
    pub fn owner(&self) -> ShadowOwner {
        self.owner
    }

    /// Update active owner tag lock-free across handoff or rollback WITHOUT resetting state,
    /// sequence, or loss latches, and WITHOUT touching any comparator or disk I/O lock.
    pub fn set_owner(&mut self, owner: ShadowOwner) {
        self.owner = owner;
        if let Some(ref hub) = self.hub {
            hub.set_owner(owner);
        }
    }

    #[inline]
    pub fn hub(&self) -> Option<&Arc<TorqueShadowHub>> {
        self.hub.as_ref()
    }

    #[inline]
    pub fn overflow_atomics(&self) -> &Arc<ShadowOverflowAtomics> {
        &self.overflow
    }

    #[inline]
    pub fn state(&self) -> Option<&TorqueState> {
        self.inner.as_deref().map(|i| i.state.as_ref())
    }

    #[inline]
    pub fn alloc_live_op_id(&mut self) -> u64 {
        let id = self.next_live_op_id;
        self.next_live_op_id = self.next_live_op_id.wrapping_add(1).max(1);
        id
    }

    /// Test-only fault injection setter (never exposed via HTTP/WS/CLI/env/profile JSON).
    #[doc(hidden)]
    pub fn set_injected_fault(&mut self, fault: ShadowFaultInjection) {
        self.injected_fault = fault;
    }

    #[inline]
    fn alloc_seq(&mut self) -> u64 {
        let s = self.next_input_seq;
        self.next_input_seq = self.next_input_seq.saturating_add(1);
        self.overflow.last_allocated_seq.fetch_max(s, Ordering::Release);
        s
    }

    /// Enqueue an owner or driver input into the single ordered FIFO with its exact edge timestamp.
    /// Zero heap allocation, zero locks. On overflow, assigns a sequence position and publishes
    /// `first_staging_loss_seq` atomically.
    #[inline]
    pub fn enqueue(&mut self, kind: ShadowInputKind) {
        if !self.enabled {
            return;
        }
        let edge_instant = self.clock.now_instant();
        self.enqueue_at(edge_instant, kind);
    }

    #[inline]
    pub fn enqueue_at(&mut self, edge_instant: Instant, kind: ShadowInputKind) {
        if !self.enabled {
            return;
        }
        let seq = self.alloc_seq();
        let Some(ref mut inner) = self.inner else {
            return;
        };
        if self.pending_len < inner.pending_inputs.len() {
            inner.pending_inputs[self.pending_len] = Some(ShadowInput {
                seq,
                edge_instant,
                kind,
            });
            self.pending_len += 1;
        } else {
            self.overflow.record_staging_loss(seq, 1);
        }
    }

    /// Copy staged observations out of a driver (while the driver lock is ALREADY held) into
    /// `pending_inputs` in exact arrival order. Zero reducer steps, zero SPSC pushes, zero allocs.
    #[inline]
    pub fn drain_driver_staging_into_fifo(
        &mut self,
        drain_fn: impl FnOnce(&mut [Option<ShadowDriverObs>]) -> (usize, u64),
    ) {
        if !self.enabled {
            return;
        }
        let Some(ref mut inner) = self.inner else {
            return;
        };
        let (count, driver_dropped) = drain_fn(&mut inner.driver_scratch);
        for i in 0..count {
            let obs = match self.inner.as_mut() {
                Some(inner) => inner.driver_scratch[i].take(),
                None => None,
            };
            if let Some(obs) = obs {
                let edge_ts = driver_obs_edge_instant(&obs);
                self.enqueue_at(edge_ts, ShadowInputKind::DriverObs(obs));
            }
        }
        if driver_dropped > 0 {
            let first_lost = self.alloc_seq();
            // Advance sequence counter past all dropped items so gaps are exact.
            self.next_input_seq = self
                .next_input_seq
                .saturating_add(driver_dropped.saturating_sub(1));
            self.overflow
                .last_allocated_seq
                .fetch_max(self.next_input_seq.saturating_sub(1), Ordering::Release);
            self.overflow.record_staging_loss(first_lost, driver_dropped);
        }
    }

    /// Convenience wrapper draining staged observations directly from a locked `ArmDriver`.
    #[inline]
    pub fn drain_from_driver(&mut self, drv: &mut dyn crate::driver::ArmDriver) {
        if !self.enabled {
            return;
        }
        self.drain_driver_staging_into_fifo(|buf| drv.drain_shadow_observations(buf));
    }

    /// Outer post-write boundary flush: drains `pending_inputs` in exact causal FIFO order,
    /// steps the single inline `TorqueState` (zero `prev_state` copy, zero `check_invariants` on RT),
    /// and pushes compact `ShadowRecord`s into the preallocated SPSC ring (`rtrb`).
    ///
    /// Zero heap allocations and zero locks, including under full ring saturation.
    pub fn flush_deferred_steps(&mut self) {
        if !self.enabled || self.pending_len == 0 || self.inner.is_none() {
            return;
        }
        let count = self.pending_len;
        self.pending_len = 0;
        for i in 0..count {
            let input = self
                .inner
                .as_mut()
                .and_then(|inner| inner.pending_inputs[i].take());
            if let Some(input) = input {
                self.process_one_input(input);
            }
        }
    }

    /// Test / synchronous helper: flush pending steps and wake/drain the off-thread hub.
    pub fn flush_and_drain_sync(&mut self) {
        self.flush_deferred_steps();
        if let Some(ref hub) = self.hub {
            hub.drain_now();
        }
    }

    fn push_record(&mut self, rec: ShadowRecord) {
        let Some(ref mut inner) = self.inner else {
            return;
        };
        if let Err(PushError::Full(_)) = inner.ring_tx.push(rec) {
            self.overflow.record_ring_loss(rec.seq);
        }
    }

    #[inline]
    fn state_mut(&mut self) -> &mut TorqueState {
        self.inner
            .as_mut()
            .expect("enabled ShadowRuntime has inner")
            .state
            .as_mut()
    }

    #[inline]
    fn state_ref(&self) -> &TorqueState {
        self.inner
            .as_ref()
            .expect("enabled ShadowRuntime has inner")
            .state
            .as_ref()
    }

    fn apply_fault_injection(&mut self, summary: &mut ActionSummary) {
        match self.injected_fault {
            ShadowFaultInjection::None => {}
            ShadowFaultInjection::InjectSpuriousDisable { recipients } => {
                self.injected_fault = ShadowFaultInjection::None;
                summary.disable_kind = Some(DisableKind::Requested);
                summary.disable_recipients = recipients;
            }
            ShadowFaultInjection::SuppressHolds => {
                self.injected_fault = ShadowFaultInjection::None;
                summary.hold_mask = ChannelMask::EMPTY;
            }
            ShadowFaultInjection::InjectRefusal(code) => {
                self.injected_fault = ShadowFaultInjection::None;
                summary.refusal = Some(code);
            }
            ShadowFaultInjection::CorruptDisableRecipients(mask) => {
                self.injected_fault = ShadowFaultInjection::None;
                if summary.disable_kind.is_some() {
                    summary.disable_recipients = mask;
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn emit_record(
        &mut self,
        seq: u64,
        now_us: u64,
        kind: ShadowRecordKind,
        event: Option<Event>,
        rt_actions: ActionSummary,
        live: LiveDecisionSummary,
        join: JoinMeta,
        coverage_gap: Option<CoverageGapKind>,
        reviewed_pr2: Option<ReviewedPr2Kind>,
    ) {
        let st = self.state_ref();
        let rec = ShadowRecord {
            seq,
            owner: self.owner,
            release_id: st.current_release_id,
            now_us,
            kind,
            event,
            rt_actions,
            predicted_hold_mask: self.last_release_hold_mask,
            live,
            join,
            post_drive_state: st.groups[0].drive_state,
            post_intent: st.groups[0].torque_intent,
            coverage_gap,
            reviewed_pr2,
        };
        self.push_record(rec);
    }

    fn update_bindings_from_actions(
        &mut self,
        actions: &ActionSet,
        live_op_id: u64,
        is_tier1_panic: bool,
    ) {
        if let Some(ecmd) = actions.enables[0] {
            self.enable_binding = EnableBinding {
                active: true,
                live_op_id,
                attempt_id: ecmd.attempt_id,
                generation: ecmd.generation,
                channels: ecmd.channels,
            };
        }
        if self.state_ref().groups[0].drive_state != DriveState::ReadyToSwitchOn
            || self.state_ref().groups[0].enable_tx.is_none()
        {
            self.enable_binding.active = false;
        }
        if let Some(dcmd) = actions.disables[0] {
            self.disable_binding = DisableBinding {
                active: true,
                live_op_id: if self.disable_binding.active && self.disable_binding.tx_id == dcmd.tx_id {
                    self.disable_binding.live_op_id
                } else {
                    live_op_id
                },
                tx_id: dcmd.tx_id,
                dispatch_id: dcmd.dispatch_id,
                generation: dcmd.generation,
                kind: Some(dcmd.kind),
            };
        }
        // Clear disable_binding immediately once no pending wire disable dispatch is unresolved
        // so between-repeat unpredicted writes cannot match a stale dispatch_id.
        if self.state_ref().groups[0].pending_disable_dispatch.is_none() {
            self.disable_binding.active = false;
        }
        if let Some(pcmd) = actions.park_requests[0] {
            self.park_binding = ParkBinding {
                active: true,
                is_tier1_panic,
                park_id: Some(pcmd.park_id),
                generation: pcmd.generation,
            };
        }
    }

    /// Out-of-line step + summary helper so `ActionSet` (~4.5 KB) occupies only a single stack
    /// slot rather than N separate unoptimized match-arm slots inside `process_one_input` and
    /// `process_driver_obs` (keeps debug/RT stack usage bounded to < 8 KB).
    #[inline(never)]
    fn run_step_summarized(
        &mut self,
        event: Event,
        update_bindings: bool,
        live_op_id: u64,
        is_tier1_panic: bool,
        apply_fault: bool,
    ) -> ActionSummary {
        let actions = step(self.state_mut(), event);
        if update_bindings {
            self.update_bindings_from_actions(&actions, live_op_id, is_tier1_panic);
        }
        let mut summary = ActionSummary::from_action_set(&actions);
        if apply_fault {
            self.apply_fault_injection(&mut summary);
        }
        summary
    }

    #[inline(never)]
    fn process_one_input(&mut self, input: ShadowInput) {
        let seq = input.seq;

        match input.kind {
            ShadowInputKind::JointZeroRecalibrated => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                apply_joint_zero_recalibration(self.state_mut());
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::JointZeroRecalibrated,
                    None,
                    ActionSummary::default(),
                    LiveDecisionSummary::default(),
                    JoinMeta::default(),
                    None,
                    None,
                );
            }
            ShadowInputKind::CoverageGap { gap } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::CoverageGapOnly,
                    None,
                    ActionSummary::default(),
                    LiveDecisionSummary::default(),
                    JoinMeta::default(),
                    Some(gap),
                    None,
                );
            }
            ShadowInputKind::DriverObs(obs) => self.process_driver_obs(seq, obs),
            ShadowInputKind::Release {
                release_seq: _,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::Release { now_us };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                self.last_release_hold_mask = rt_actions.hold_mask;
                let reviewed_pr2 = if !rt_actions.damp_mask.is_empty() {
                    Some(ReviewedPr2Kind::ProtectiveDamping)
                } else {
                    None
                };
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: false, // Holds are compared at LiveHoldCheckpoint after Step 5
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    reviewed_pr2,
                );
            }
            ShadowInputKind::TransitionWake {
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::TransitionWake { now_us };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: false,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        capability_id: self.disable_binding.dispatch_id,
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::EnableStart {
                live_op_id,
                live_refusal,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::EnableStart { group: 0, now_us };
                let rt_actions = self.run_step_summarized(event, true, live_op_id, false, true);
                let reviewed_pr2 = if rt_actions.refusal == Some(RefusalCode::TimeoutMismatch) {
                    Some(ReviewedPr2Kind::TimeoutReadbackGate)
                } else {
                    None
                };
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        live_enable_started: live_refusal.is_none(),
                        live_refusal,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        live_op_id,
                        capability_id: self.enable_binding.attempt_id,
                        generation: gen,
                        recipient_mask: self.enable_binding.channels,
                        ..JoinMeta::default()
                    },
                    None,
                    reviewed_pr2,
                );
            }
            ShadowInputKind::EnableCancelledPreTx {
                live_op_id,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let attempt_id = if self.enable_binding.active
                    && (live_op_id == 0 || self.enable_binding.live_op_id == live_op_id)
                {
                    self.enable_binding.attempt_id
                } else {
                    0
                };
                let event = Event::EnableCancelledPreTx {
                    group: 0,
                    attempt_id,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, true, live_op_id, false, false);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: false,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        live_op_id,
                        capability_id: attempt_id,
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::HandshakeResult {
                live_op_id,
                confirmed_mask,
                outcome,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                // Bind strictly by `live_op_id` whenever non-zero so a stale result cannot attach
                // to a newer enable attempt.
                let attempt_id = if self.enable_binding.active
                    && ((live_op_id != 0 && self.enable_binding.live_op_id == live_op_id)
                        || (live_op_id == 0 && self.enable_binding.live_op_id == 0))
                {
                    self.enable_binding.attempt_id
                } else {
                    0
                };
                let event = Event::HandshakeResult {
                    group: 0,
                    attempt_id,
                    now_us,
                    confirmed_mask,
                    outcome,
                };
                let rt_actions = self.run_step_summarized(event, true, live_op_id, false, true);
                let live_refusal = match outcome {
                    HandshakeOutcome::Failed | HandshakeOutcome::FaultReported { .. } => {
                        Some(LiveRefusalKind::EnableFailed)
                    }
                    HandshakeOutcome::Pending | HandshakeOutcome::Confirmed => None,
                };
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: !matches!(outcome, HandshakeOutcome::Pending),
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        live_refusal,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        live_op_id,
                        capability_id: attempt_id,
                        generation: gen,
                        recipient_mask: confirmed_mask,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::DisableRequest {
                live_op_id,
                explicit_release_authorized,
                live_refusal,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::DisableRequest {
                    group: 0,
                    now_us,
                    explicit_release_authorized,
                    recipient_mask_override: None,
                };
                let rt_actions = self.run_step_summarized(event, true, live_op_id, false, true);
                // Exact PR2 reviewed exception: ONLY live rest-pose preflight refusals
                // (`TorqueOffWouldDrop` / `ArmNotAtRestingPose`) differ from reducer `AwaitRestGate`.
                // Unrelated refusals like `DriverBusy` or `BusyOrTransition` MUST NOT be masked!
                let reviewed_pr2 = if matches!(
                    live_refusal,
                    Some(LiveRefusalKind::TorqueOffWouldDrop | LiveRefusalKind::ArmNotAtRestingPose)
                ) && rt_actions.refusal.is_none()
                    && self.state_ref().groups[0]
                        .disable_tx
                        .is_some_and(|tx| matches!(tx.phase, DisablePhase::AwaitRestGate))
                {
                    Some(ReviewedPr2Kind::DriverPreflightVsAwaitRestGate)
                } else {
                    None
                };
                let gen = self.state_ref().groups[0].stop_generation;
                let recipients = self.state_ref().config.groups[0].recipients;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        live_refusal,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        live_op_id,
                        capability_id: self.disable_binding.dispatch_id,
                        generation: gen,
                        recipient_mask: recipients,
                        ..JoinMeta::default()
                    },
                    None,
                    reviewed_pr2,
                );
            }
            ShadowInputKind::Stop {
                kind,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::Stop {
                    groups: GroupMask::bit(0),
                    kind,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: false,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::FaultClear {
                live_refusal,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::FaultClear {
                    groups: GroupMask::bit(0),
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        live_refusal,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::Thermal {
                channel,
                temperature_c,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::Thermal {
                    channel,
                    temperature_c,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let reviewed_pr2 = if !rt_actions.damp_mask.is_empty() {
                    Some(ReviewedPr2Kind::ProtectiveDamping)
                } else {
                    None
                };
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: false,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        capability_id: self.disable_binding.dispatch_id,
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    reviewed_pr2,
                );
            }
            ShadowInputKind::BusLoss {
                device,
                multi_device_gap,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::BusLoss { device, now_us };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: false,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        device,
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    multi_device_gap.then_some(CoverageGapKind::BusLossDeviceAttribution),
                    None,
                );
            }
            ShadowInputKind::MotionCommand {
                channels,
                live_allowed,
                live_refusal,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::MotionCommand {
                    group: 0,
                    channels,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, false, 0, false, true);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        live_motion_allowed: Some(live_allowed),
                        live_refusal,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        generation: gen,
                        recipient_mask: channels,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::GraspStateUpdate {
                channel,
                setpoint_rad,
                force_limit_raw,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::GraspStateUpdate {
                    channel,
                    setpoint_rad,
                    force_limit_raw,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, false, 0, false, false);
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary::default(),
                    JoinMeta {
                        recipient_mask: ChannelMask::bit(channel),
                        ..JoinMeta::default()
                    },
                    force_limit_raw
                        .is_none()
                        .then_some(CoverageGapKind::GraspForceCeilingMissing),
                    None,
                );
            }
            ShadowInputKind::MitOwnershipChange {
                channels,
                target_mode,
                device_wide,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::MitOwnershipChange {
                    group: 0,
                    channels,
                    target_mode,
                    device_wide,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, true, 0, false, false);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary::default(),
                    JoinMeta {
                        generation: gen,
                        recipient_mask: channels,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::ProcessExit {
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::ProcessExit { now_us };
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let reviewed_pr2 = if live_intent == TorqueIntent::Off
                    && rt_actions.disable_kind == Some(DisableKind::Protective)
                {
                    Some(ReviewedPr2Kind::ProcessExitAlreadyDisabled)
                } else {
                    None
                };
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        capability_id: self.disable_binding.dispatch_id,
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    reviewed_pr2,
                );
            }
            ShadowInputKind::Panic {
                tier,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                let event = Event::Panic { tier, now_us };
                let rt_actions = self.run_step_summarized(
                    event,
                    true,
                    0,
                    matches!(tier, PanicTier::WorkerPanic { .. }),
                    true,
                );
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    None,
                    Some(ReviewedPr2Kind::FuPanicBaselineMismatch),
                );
            }
            ShadowInputKind::LiveHoldCheckpoint {
                hold_mask,
                stale_held_mask,
                contended,
                escalated,
                live_intent,
                live_is_torqued,
                live_stopped,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                if contended {
                    let target = self.state_ref().config.groups[0].channels;
                    let gen = self.state_ref().groups[0].stop_generation;
                    let wr = WriteResult::uniform(
                        0,
                        0,
                        WriteKind::Hold,
                        0,
                        gen,
                        now_us,
                        now_us,
                        target,
                        MotorWriteOutcome::LockContended,
                    );
                    let event = Event::WriteResult(wr);
                    // Keep `step_sum` strictly identical to `ActionSummary::from_action_set(&actions)`
                    // so off-thread replay never gets a false `ExtraAction` from `last_release_hold_mask`.
                    let step_sum = self.run_step_summarized(event, false, 0, false, true);
                    self.emit_record(
                        seq,
                        now_us,
                        ShadowRecordKind::Step,
                        Some(event),
                        step_sum,
                        LiveDecisionSummary {
                            comparable: true,
                            live_intent,
                            live_is_torqued,
                            live_stopped,
                            live_hold_mask: hold_mask,
                            live_stale_held_mask: stale_held_mask,
                            live_contention_escalated: escalated,
                            ..LiveDecisionSummary::default()
                        },
                        JoinMeta {
                            generation: gen,
                            recipient_mask: hold_mask,
                            ..JoinMeta::default()
                        },
                        None,
                        None,
                    );
                    return;
                }
                // Evaluate the reducer's predicted hold mask at the checkpoint itself (after
                // Step-2 transition service and RX intake have already applied to `self.state_ref`)
                // so comparison aligns with live measured-hold selection within the same release.
                let mut pred_hold =
                    crate::torque::reducer::predict_group_hold_mask(self.state_ref(), 0, now_us);
                if pred_hold.is_empty() {
                    if let Some(pend) = self.state_ref().groups[0].pending_disable_dispatch {
                        if pend.kind == DisableKind::Requested
                            && !pend.remaining_recipients.is_empty()
                        {
                            // During a multi-tick requested disable, still-enabled joints keep
                            // receiving measured holds until their disable frame goes out.
                            let st = self.state_ref();
                            for r in pend.remaining_recipients.iter() {
                                if let Some(rcfg) = st.config.recipients.get(r as usize) {
                                    for ch in rcfg.channels.iter() {
                                        if st
                                            .joints
                                            .get(ch as usize)
                                            .is_some_and(|j| j.captured_position_rad.is_some())
                                        {
                                            pred_hold.insert(ch);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                let mut rt_actions = ActionSummary {
                    hold_mask: pred_hold,
                    ..ActionSummary::default()
                };
                self.apply_fault_injection(&mut rt_actions);
                self.last_release_hold_mask = rt_actions.hold_mask;
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::CoverageGapOnly,
                    None,
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: true,
                        live_intent,
                        live_is_torqued,
                        live_stopped,
                        live_hold_mask: hold_mask,
                        live_stale_held_mask: stale_held_mask,
                        live_contention_escalated: escalated,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        generation: gen,
                        recipient_mask: hold_mask,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
            ShadowInputKind::ParkLifecycle {
                live_op_id,
                admitted,
                settled_ok,
            } => {
                let now_us = self.clock.advance_now_us(input.edge_instant);
                if self.park_binding.active
                    && self.park_binding.is_tier1_panic
                    && self.park_binding.park_id.is_some()
                {
                    let park_id = self.park_binding.park_id.unwrap_or(0);
                    let event_opt = if let Some(succeeded) = settled_ok {
                        self.park_binding.active = false;
                        Some(Event::ParkSettled {
                            group: 0,
                            park_id,
                            succeeded,
                            now_us,
                        })
                    } else if admitted {
                        Some(Event::ParkAccepted {
                            group: 0,
                            park_id,
                            now_us,
                        })
                    } else {
                        None
                    };
                    if let Some(event) = event_opt {
                        let rt_actions = self.run_step_summarized(event, false, 0, false, false);
                        self.emit_record(
                            seq,
                            now_us,
                            ShadowRecordKind::Step,
                            Some(event),
                            rt_actions,
                            LiveDecisionSummary::default(),
                            JoinMeta {
                                live_op_id,
                                capability_id: park_id,
                                generation: self.park_binding.generation,
                                ..JoinMeta::default()
                            },
                            None,
                            None,
                        );
                    } else {
                        // Always emit a sequence-preserving record for every allocated `seq`.
                        self.emit_record(
                            seq,
                            now_us,
                            ShadowRecordKind::CoverageGapOnly,
                            None,
                            ActionSummary::default(),
                            LiveDecisionSummary::default(),
                            JoinMeta {
                                live_op_id,
                                capability_id: park_id,
                                generation: self.park_binding.generation,
                                ..JoinMeta::default()
                            },
                            None,
                            None,
                        );
                    }
                } else {
                    let gen = self.state_ref().groups[0].stop_generation;
                    if settled_ok.is_some() {
                        self.park_binding.active = false;
                    } else if admitted {
                        self.park_binding = ParkBinding {
                            active: true,
                            is_tier1_panic: false,
                            park_id: None,
                            generation: gen,
                        };
                    }
                    // Always emit a sequence-preserving record for normal park lifecycle markers
                    // so `expected_seq` never develops a false unverifiable seq gap.
                    self.emit_record(
                        seq,
                        now_us,
                        ShadowRecordKind::CoverageGapOnly,
                        None,
                        ActionSummary::default(),
                        LiveDecisionSummary::default(),
                        JoinMeta {
                            live_op_id,
                            generation: gen,
                            ..JoinMeta::default()
                        },
                        None,
                        None,
                    );
                }
            }
        }
    }

    #[inline(never)]
    fn process_driver_obs(&mut self, seq: u64, obs: ShadowDriverObs) {
        match obs {
            ShadowDriverObs::CausalSample {
                channel,
                sample_id,
                stamp,
                decode_now,
                position_rad,
                velocity_rad_s,
                temperature_c,
                reported_enabled,
                correlated_after_write_done,
                poll_issued,
                provenance,
            } => {
                let last_mw_us = self
                    .state_ref()
                    .joints
                    .get(channel as usize)
                    .and_then(|j| j.last_motion_write_us);
                let (rx_timestamp_us, now_us, drift_us, coverage_gap, reviewed_pr2) = match stamp {
                    RxStampKind::Valid {
                        rx_at,
                        drift_ns,
                        live_post_settle,
                    } => {
                        let (rx_us, now_us) = self.clock.normalize_valid_rx_us(
                            rx_at,
                            decode_now,
                            last_mw_us,
                            live_post_settle,
                        );
                        let drift_us = if drift_ns > CLOCK_OFFSET_MAX_DRIFT_US * 1_000 {
                            (drift_ns / 1_000).max(CLOCK_OFFSET_MAX_DRIFT_US + 1)
                        } else {
                            drift_ns / 1_000
                        };
                        let gap = match provenance {
                            EvidenceProvenance::MockSynthetic => {
                                Some(CoverageGapKind::MockSyntheticSample)
                            }
                            _ => None,
                        };
                        (rx_us, now_us, drift_us, gap, None)
                    }
                    RxStampKind::Future => {
                        let now_us = self.clock.advance_now_us(decode_now);
                        (now_us.saturating_add(1), now_us, 0, None, None)
                    }
                    RxStampKind::Missing { reset_window } => {
                        let now_us = self.clock.advance_now_us(decode_now);
                        if !reset_window {
                            self.emit_record(
                                seq,
                                now_us,
                                ShadowRecordKind::CoverageGapOnly,
                                None,
                                ActionSummary::default(),
                                LiveDecisionSummary::default(),
                                JoinMeta {
                                    recipient_mask: ChannelMask::bit(channel),
                                    ..JoinMeta::default()
                                },
                                Some(CoverageGapKind::MissingRxTimestamp),
                                None,
                            );
                            return;
                        }
                        (
                            now_us.saturating_add(1),
                            now_us,
                            0,
                            Some(CoverageGapKind::MissingRxTimestamp),
                            None,
                        )
                    }
                    RxStampKind::Unmappable => {
                        let now_us = self.clock.advance_now_us(decode_now);
                        (
                            now_us.saturating_add(1),
                            now_us,
                            0,
                            Some(CoverageGapKind::UnmappableRxTimestamp),
                            None,
                        )
                    }
                    RxStampKind::OffsetDrifted { rebased_settle } => {
                        let now_us = self.clock.advance_now_us(decode_now);
                        (
                            now_us,
                            now_us,
                            CLOCK_OFFSET_MAX_DRIFT_US + 1,
                            None,
                            rebased_settle.then_some(ReviewedPr2Kind::LiveDriftSettleRebase),
                        )
                    }
                    RxStampKind::UnqualifiedRestEvidence { .. } => {
                        // Live rest gate refused credit (unknown queue bound, awaiting RX boundary,
                        // or invalid numeric data): reject sample so reducer resets the rest window
                        // and revokes unconsumed rest auth identically to live `record_disagree()`.
                        let now_us = self.clock.advance_now_us(decode_now);
                        (now_us.saturating_add(1), now_us, 0, None, None)
                    }
                };

                let correlated_after_write_done_us =
                    correlated_after_write_done.map(|t| self.clock.instant_to_us(t));
                let poll_issued_us = poll_issued.map(|t| self.clock.instant_to_us(t));

                let sample = CausalSample {
                    channel,
                    sample_id,
                    rx_timestamp_us,
                    now_us,
                    clock_offset_drift_us: drift_us,
                    position_rad,
                    velocity_rad_s,
                    temperature_c,
                    reported_enabled,
                    correlated_after_write_done_us,
                    poll_issued_us,
                    release_evidence: None,
                    is_handshake_ack_only: false,
                };
                let event = Event::CausalSample(sample);
                let rt_actions = self.run_step_summarized(event, true, 0, false, true);
                let gen = self.state_ref().groups[0].stop_generation;
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary::default(),
                    JoinMeta {
                        recipient_mask: ChannelMask::bit(channel),
                        generation: gen,
                        ..JoinMeta::default()
                    },
                    coverage_gap,
                    reviewed_pr2,
                );
            }
            ShadowDriverObs::WriteResult {
                device,
                kind,
                write_done,
                now,
                target_mask,
                outcomes,
                ambiguous_gap,
            } => {
                let now_us = self.clock.advance_now_us(now);
                let write_done_us = self.clock.instant_to_us(write_done).min(now_us);
                let cur_gen = self.state_ref().groups[0].stop_generation;
                let (dispatch_id, generation, unmatched, live_op_id) = match kind {
                    WriteKind::DisableRequested | WriteKind::DisableProtective => {
                        let expected_dk = match kind {
                            WriteKind::DisableRequested => DisableKind::Requested,
                            WriteKind::DisableProtective => DisableKind::Protective,
                            _ => unreachable!(),
                        };
                        if self.disable_binding.active
                            && self.disable_binding.kind == Some(expected_dk)
                        {
                            (
                                self.disable_binding.dispatch_id,
                                self.disable_binding.generation,
                                false,
                                self.disable_binding.live_op_id,
                            )
                        } else {
                            (0, cur_gen, true, 0)
                        }
                    }
                    WriteKind::Enable => {
                        if self.enable_binding.active {
                            (
                                self.enable_binding.attempt_id,
                                self.enable_binding.generation,
                                false,
                                self.enable_binding.live_op_id,
                            )
                        } else {
                            (0, cur_gen, true, 0)
                        }
                    }
                    WriteKind::Hold
                    | WriteKind::Motion
                    | WriteKind::Damp
                    | WriteKind::Refresh
                    | WriteKind::ModeSwitch => (0, cur_gen, false, 0),
                };

                let wr = WriteResult {
                    group: 0,
                    device,
                    kind,
                    dispatch_id,
                    generation,
                    write_done_us,
                    now_us,
                    target_mask,
                    outcomes,
                };
                let event = Event::WriteResult(wr);
                let rt_actions = self.run_step_summarized(event, true, live_op_id, false, true);

                let live_disable_kind = match kind {
                    WriteKind::DisableRequested => Some(DisableKind::Requested),
                    WriteKind::DisableProtective => Some(DisableKind::Protective),
                    _ => None,
                };
                let mut sent_or_ambig = ChannelMask::EMPTY;
                for c in target_mask.iter() {
                    if matches!(
                        outcomes[c as usize],
                        MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous
                    ) {
                        sent_or_ambig.insert(c);
                    }
                }
                let live_partial = live_disable_kind.is_some()
                    && !sent_or_ambig.is_empty()
                    && sent_or_ambig != target_mask;
                let live_intent = self.state_ref().groups[0].torque_intent;
                // Always mark disable/enable write results comparable so matched writes are
                // reconciled against predicted recipient/kind obligations (never silently hidden).
                let is_comparable = live_disable_kind.is_some() || kind == WriteKind::Enable;

                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary {
                        comparable: is_comparable,
                        live_intent,
                        live_enable_started: kind == WriteKind::Enable && !sent_or_ambig.is_empty(),
                        live_disable_kind,
                        live_disable_recipients: sent_or_ambig,
                        live_partial_disable: live_partial,
                        ..LiveDecisionSummary::default()
                    },
                    JoinMeta {
                        live_op_id,
                        capability_id: dispatch_id,
                        generation,
                        group: 0,
                        device,
                        recipient_mask: target_mask,
                        unmatched_live_write: unmatched && !sent_or_ambig.is_empty(),
                    },
                    ambiguous_gap.then_some(CoverageGapKind::AmbiguousWriteOutcomeUnavailable),
                    None,
                );
            }
            ShadowDriverObs::ModeVerified {
                channel,
                mode,
                now,
                provenance,
            } => {
                let now_us = self.clock.advance_now_us(now);
                let event = Event::ModeVerified {
                    channel,
                    mode,
                    now_us,
                };
                let rt_actions = self.run_step_summarized(event, false, 0, false, false);
                let gap = match provenance {
                    EvidenceProvenance::DriverNativeFixed => {
                        Some(CoverageGapKind::NativeModeAssumed)
                    }
                    _ => None,
                };
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary::default(),
                    JoinMeta {
                        recipient_mask: ChannelMask::bit(channel),
                        ..JoinMeta::default()
                    },
                    gap,
                    None,
                );
            }
            ShadowDriverObs::TimeoutReadback {
                channel,
                raw_reg,
                now,
            } => {
                let now_us = self.clock.advance_now_us(now);
                match u16::try_from(raw_reg).ok().filter(|&v| v > 0 && v <= DAMIAO_TIMEOUT_MAX_REG) {
                    Some(timeout_reg) => {
                        let event = Event::TimeoutReadback {
                            channel,
                            timeout_reg,
                            now_us,
                        };
                        let rt_actions = self.run_step_summarized(event, false, 0, false, false);
                        self.emit_record(
                            seq,
                            now_us,
                            ShadowRecordKind::Step,
                            Some(event),
                            rt_actions,
                            LiveDecisionSummary::default(),
                            JoinMeta {
                                recipient_mask: ChannelMask::bit(channel),
                                ..JoinMeta::default()
                            },
                            None,
                            None,
                        );
                    }
                    None => {
                        self.emit_record(
                            seq,
                            now_us,
                            ShadowRecordKind::CoverageGapOnly,
                            None,
                            ActionSummary::default(),
                            LiveDecisionSummary::default(),
                            JoinMeta {
                                recipient_mask: ChannelMask::bit(channel),
                                ..JoinMeta::default()
                            },
                            Some(CoverageGapKind::TimeoutReadbackOutOfRange),
                            None,
                        );
                    }
                }
            }
            ShadowDriverObs::SerialTimeout { device, now } => {
                let now_us = self.clock.advance_now_us(now);
                let event = Event::SerialTimeout { device, now_us };
                let rt_actions = self.run_step_summarized(event, false, 0, false, false);
                self.emit_record(
                    seq,
                    now_us,
                    ShadowRecordKind::Step,
                    Some(event),
                    rt_actions,
                    LiveDecisionSummary::default(),
                    JoinMeta {
                        device,
                        ..JoinMeta::default()
                    },
                    None,
                    None,
                );
            }
        }
    }
}

#[inline]
fn driver_obs_edge_instant(obs: &ShadowDriverObs) -> Instant {
    match *obs {
        ShadowDriverObs::CausalSample { decode_now, .. } => decode_now,
        ShadowDriverObs::WriteResult { now, .. } => now,
        ShadowDriverObs::ModeVerified { now, .. } => now,
        ShadowDriverObs::TimeoutReadback { now, .. } => now,
        ShadowDriverObs::SerialTimeout { now, .. } => now,
    }
}

#[inline]
fn has_causal_mode_readback(vendor: VendorKind) -> bool {
    matches!(vendor, VendorKind::Damiao)
}

fn seed_state_native_modes(state: &mut TorqueState) {
    for c in 0..state.config.channel_count {
        let ci = c as usize;
        if !matches!(state.config.channels[ci].role, JointRole::PassiveSensor) {
            state.joints[ci].verified_mode = Some(state.joints[ci].desired_mode);
        }
    }
}

fn apply_live_boot_sync(state: &mut TorqueState, sync: LiveBootSync) {
    if state.config.group_count == 0 {
        return;
    }
    let grp = &mut state.groups[0];
    if sync.estop_latched {
        grp.faults.insert(TorqueFaultKind::EStop);
    } else if sync.has_fault {
        grp.faults.insert(TorqueFaultKind::Other);
    }
    if sync.failed_hold {
        grp.torque_intent = TorqueIntent::On;
        grp.drive_state = DriveState::FailedHold;
        grp.enable_phase = EnablePhase::FailedHold;
        grp.faults.insert(TorqueFaultKind::EnableFailed);
    } else if sync.live_is_torqued {
        grp.torque_intent = TorqueIntent::On;
        grp.enable_phase = EnablePhase::Armed;
        grp.drive_state = if !grp.faults.is_empty() {
            DriveState::Fault
        } else if sync.live_stopped {
            DriveState::QuickStopActive
        } else {
            DriveState::OperationEnabled
        };
    } else if !grp.faults.is_empty() {
        grp.torque_intent = TorqueIntent::Off;
        grp.drive_state = DriveState::Fault;
        grp.enable_phase = EnablePhase::Idle;
    } else {
        grp.torque_intent = TorqueIntent::Off;
        grp.drive_state = DriveState::SwitchOnDisabled;
        grp.enable_phase = EnablePhase::Idle;
    }
}

/// Replayable recalibration reset applied identically on both inline `TorqueState` and off-thread
/// comparator `TorqueState` when joint zero offsets change (`src/supervisor/config.rs:84`).
pub fn apply_joint_zero_recalibration(state: &mut TorqueState) {
    for c in 0..state.config.channel_count {
        let ci = c as usize;
        let rt = &mut state.joints[ci];
        rt.captured_position_rad = None;
        rt.captured_at_us = None;
        rt.last_sample_us = None;
        rt.last_measured_pos_rad = None;
        rt.last_measured_vel_rad_s = None;
        rt.last_window_sample_id = None;
        rt.last_window_release_id = None;
        rt.had_valid_capture = false;
    }
    for r in 0..state.config.recipient_count {
        state.rest_windows[r as usize].reset();
    }
    for g in 0..state.config.group_count {
        let gi = g as usize;
        if let Some(tx) = state.groups[gi].disable_tx.as_mut() {
            if let DisablePhase::AuthorizedForRelease { dispatch_id, .. } = tx.phase {
                if !tx.committed_off {
                    tx.phase = DisablePhase::AwaitRestGate;
                    if let Some(pend) = state.groups[gi].pending_disable_dispatch.as_mut() {
                        if pend.dispatch_id == dispatch_id {
                            pend.tx_still_active = false;
                        }
                    }
                }
            }
        }
    }
}

// =========================================================================
// Off-Thread Comparator, Diagnostics Snapshot & Decoupled Blackbox Sink
// =========================================================================

/// Taxonomy counters exposed in `/api/v1/diagnostics` (`torque_shadow.taxonomy`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DivergenceTaxonomyCounts {
    pub extra_action: u64,
    pub missing_action: u64,
    pub authority_recipient_mismatch: u64,
    pub causal_timing_order: u64,
    pub outcome_refusal_mismatch: u64,
    pub invariant_violation: u64,
    pub coverage_gap: u64,
    pub reviewed_pr2_only: u64,
}

impl DivergenceTaxonomyCounts {
    pub fn unexplained_total(&self) -> u64 {
        self.extra_action
            .saturating_add(self.missing_action)
            .saturating_add(self.authority_recipient_mismatch)
            .saturating_add(self.causal_timing_order)
            .saturating_add(self.outcome_refusal_mismatch)
            .saturating_add(self.invariant_violation)
    }
}

/// Per-kind coverage gap counters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoverageGapCounts {
    pub missing_rx_timestamp: u64,
    pub unmappable_rx_timestamp: u64,
    pub timeout_readback_unavailable: u64,
    pub timeout_readback_out_of_range: u64,
    pub native_mode_assumed: u64,
    pub grasp_force_ceiling_missing: u64,
    pub worker_panic_hook_absent: u64,
    pub bus_loss_device_attribution: u64,
    pub ambiguous_write_outcome_unavailable: u64,
    pub mock_synthetic_sample: u64,
}

impl CoverageGapCounts {
    fn bump(&mut self, kind: CoverageGapKind) {
        match kind {
            CoverageGapKind::MissingRxTimestamp => self.missing_rx_timestamp += 1,
            CoverageGapKind::UnmappableRxTimestamp => self.unmappable_rx_timestamp += 1,
            CoverageGapKind::TimeoutReadbackUnavailable => self.timeout_readback_unavailable += 1,
            CoverageGapKind::TimeoutReadbackOutOfRange => self.timeout_readback_out_of_range += 1,
            CoverageGapKind::NativeModeAssumed => self.native_mode_assumed += 1,
            CoverageGapKind::GraspForceCeilingMissing => self.grasp_force_ceiling_missing += 1,
            CoverageGapKind::WorkerPanicHookAbsent => self.worker_panic_hook_absent += 1,
            CoverageGapKind::BusLossDeviceAttribution => self.bus_loss_device_attribution += 1,
            CoverageGapKind::AmbiguousWriteOutcomeUnavailable => {
                self.ambiguous_write_outcome_unavailable += 1
            }
            CoverageGapKind::MockSyntheticSample => self.mock_synthetic_sample += 1,
        }
    }
}

/// Ring & staging overflow diagnostic report.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShadowRingDiagnostics {
    pub capacity: usize,
    pub ring_dropped_records: u64,
    pub ring_overflow_events: u64,
    pub staging_dropped_records: u64,
    pub staging_overflow_events: u64,
    pub first_unverifiable_seq: Option<u64>,
    pub interval_unverifiable: bool,
    pub blackbox_written: u64,
    pub blackbox_dropped: u64,
}

/// Last reducer state snapshot exposed in diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShadowLastState {
    pub drive_state: DriveState,
    pub torque_intent: TorqueIntent,
    pub release_id: u64,
    pub now_us: u64,
}

/// One structured divergence / violation / overflow entry kept in diagnostics and written to `bb_shadow.jsonl`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShadowDivergenceEntry {
    pub seq: u64,
    pub release_id: u64,
    pub now_us: u64,
    pub owner: ShadowOwner,
    pub category: DivergenceCategory,
    pub detail: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub violation: Option<Violation>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage_gap: Option<CoverageGapKind>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reviewed_pr2: Option<ReviewedPr2Kind>,
}

/// JSON payload returned under `"torque_shadow"` in `GET /api/v1/diagnostics`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TorqueShadowDiagnostics {
    pub enabled: bool,
    pub owner: ShadowOwner,
    pub events_processed: u64,
    pub comparisons_total: u64,
    pub unexplained_divergences_total: u64,
    pub taxonomy: DivergenceTaxonomyCounts,
    pub coverage_gaps_by_kind: CoverageGapCounts,
    pub ring: ShadowRingDiagnostics,
    pub last_state: ShadowLastState,
    pub recent_divergences: Vec<ShadowDivergenceEntry>,
}

impl TorqueShadowDiagnostics {
    pub fn disabled(owner: ShadowOwner) -> Self {
        Self {
            enabled: false,
            owner,
            events_processed: 0,
            comparisons_total: 0,
            unexplained_divergences_total: 0,
            taxonomy: DivergenceTaxonomyCounts::default(),
            coverage_gaps_by_kind: CoverageGapCounts::default(),
            ring: ShadowRingDiagnostics {
                capacity: 0,
                ..ShadowRingDiagnostics::default()
            },
            last_state: ShadowLastState::default(),
            recent_divergences: Vec::new(),
        }
    }
}

/// Dedicated sink for shadow blackbox JSONL records (`bb_shadow.jsonl`), strictly isolated from
/// the primary `"armd-blackbox"` writer thread and 1-slot observation queue.
pub trait ShadowBlackBoxSink: Send {
    fn write_shadow_line(&mut self, line: &[u8]) -> io::Result<()>;
}

impl ShadowBlackBoxSink for RotatingFile {
    fn write_shadow_line(&mut self, line: &[u8]) -> io::Result<()> {
        self.write_record(line)
    }
}

/// Bounded queue capacity for decoupled `bb_shadow.jsonl` disk writes.
const SHADOW_BB_QUEUE_CAP: usize = 256;

/// Decoupled non-blocking handle to the dedicated `"armd-shadow-bb"` file writer thread.
///
/// The comparator mutex is NEVER held across synchronous disk I/O: formatting + `try_send`
/// completes in nanoseconds, and a stalled disk sink drops excess shadow lines rather than
/// blocking owner handoff, diagnostics reads, or ring draining.
struct ShadowBbQueue {
    tx: mpsc::SyncSender<String>,
    boot: u64,
    start: Instant,
    dropped: Arc<AtomicU64>,
    done: Arc<(Mutex<bool>, Condvar)>,
}

impl ShadowBbQueue {
    fn spawn(
        mut sink: Box<dyn ShadowBlackBoxSink>,
        boot: u64,
        start: Instant,
        written: Arc<AtomicU64>,
        dropped: Arc<AtomicU64>,
    ) -> io::Result<Self> {
        let (tx, rx) = mpsc::sync_channel::<String>(SHADOW_BB_QUEUE_CAP);
        let w = written;
        let d = Arc::clone(&dropped);
        let done = Arc::new((Mutex::new(false), Condvar::new()));
        let done_bg = Arc::clone(&done);
        thread::Builder::new()
            .name("armd-shadow-bb".into())
            .spawn(move || {
                while let Ok(line) = rx.recv() {
                    match sink.write_shadow_line(line.as_bytes()) {
                        Ok(()) => {
                            w.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(_) => {
                            d.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
                let mut g = done_bg.0.lock().unwrap_or_else(|e| e.into_inner());
                *g = true;
                done_bg.1.notify_all();
            })?;
        Ok(Self {
            tx,
            boot,
            start,
            dropped,
            done,
        })
    }

    /// Spawn the decoupled `"armd-shadow-bb"` writer thread and open `bb_shadow.jsonl` INSIDE the
    /// writer thread so startup/primary `bb.jsonl` never blocks on shadow disk open or rotation.
    fn spawn_dir(
        path: std::path::PathBuf,
        boot: u64,
        start: Instant,
        written: Arc<AtomicU64>,
        dropped: Arc<AtomicU64>,
    ) -> io::Result<Self> {
        let (tx, rx) = mpsc::sync_channel::<String>(SHADOW_BB_QUEUE_CAP);
        let w = written;
        let d = Arc::clone(&dropped);
        let done = Arc::new((Mutex::new(false), Condvar::new()));
        let done_bg = Arc::clone(&done);
        thread::Builder::new()
            .name("armd-shadow-bb".into())
            .spawn(move || {
                let mut sink_opt = RotatingFile::open(&path, SHADOW_BB_MAX_BYTES, SHADOW_BB_KEEP).ok();
                while let Ok(line) = rx.recv() {
                    if let Some(ref mut sink) = sink_opt {
                        match sink.write_shadow_line(line.as_bytes()) {
                            Ok(()) => {
                                w.fetch_add(1, Ordering::Relaxed);
                            }
                            Err(_) => {
                                d.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                    } else {
                        d.fetch_add(1, Ordering::Relaxed);
                    }
                }
                let mut g = done_bg.0.lock().unwrap_or_else(|e| e.into_inner());
                *g = true;
                done_bg.1.notify_all();
            })?;
        Ok(Self {
            tx,
            boot,
            start,
            dropped,
            done,
        })
    }

    #[inline]
    fn try_send(&self, line: String) -> bool {
        if self.tx.try_send(line).is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            false
        } else {
            true
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct PendingDisableExpectation {
    kind: DisableKind,
    generation: u64,
    dispatch_id: u64,
    deadline_us: u64,
    last_progress_release_id: u64,
    expected_recipients: ChannelMask,
    remaining_recipients: ChannelMask,
    owner: ShadowOwner,
}

struct TorqueShadowConsumer {
    enabled: bool,
    rx: Option<Consumer<ShadowRecord>>,
    state: Option<Box<TorqueState>>,
    prev_state: Option<Box<TorqueState>>,
    expected_seq: u64,
    first_unverifiable_seq: Option<u64>,
    overflow_logged_bb: bool,
    events_processed: u64,
    comparisons_total: u64,
    taxonomy: DivergenceTaxonomyCounts,
    coverage_gaps: CoverageGapCounts,
    last_state: ShadowLastState,
    recent_divergences: std::collections::VecDeque<ShadowDivergenceEntry>,
    pending_disable: Option<PendingDisableExpectation>,
    bb_queue: Option<ShadowBbQueue>,
}

impl TorqueShadowConsumer {
    fn new_disabled() -> Self {
        Self {
            enabled: false,
            rx: None,
            state: None,
            prev_state: None,
            expected_seq: 1,
            first_unverifiable_seq: None,
            overflow_logged_bb: false,
            events_processed: 0,
            comparisons_total: 0,
            taxonomy: DivergenceTaxonomyCounts::default(),
            coverage_gaps: CoverageGapCounts::default(),
            last_state: ShadowLastState::default(),
            recent_divergences: std::collections::VecDeque::new(),
            pending_disable: None,
            bb_queue: None,
        }
    }

    fn new_enabled(rx: Consumer<ShadowRecord>, state: Box<TorqueState>) -> Self {
        let prev_state = state.clone();
        Self {
            enabled: true,
            rx: Some(rx),
            state: Some(state),
            prev_state: Some(prev_state),
            expected_seq: 1,
            first_unverifiable_seq: None,
            overflow_logged_bb: false,
            events_processed: 0,
            comparisons_total: 0,
            taxonomy: DivergenceTaxonomyCounts::default(),
            coverage_gaps: CoverageGapCounts::default(),
            last_state: ShadowLastState::default(),
            recent_divergences: std::collections::VecDeque::with_capacity(RECENT_DIVERGENCES_CAP),
            pending_disable: None,
            bb_queue: None,
        }
    }

    fn effective_atomic_cutoff(atomics: &ShadowOverflowAtomics) -> Option<u64> {
        let s = atomics.first_staging_loss_seq.load(Ordering::Acquire);
        let r = atomics.first_ring_loss_seq.load(Ordering::Acquire);
        let m = s.min(r);
        (m != u64::MAX).then_some(m)
    }

    fn update_cutoff(
        &mut self,
        candidate: Option<u64>,
        atomics: &ShadowOverflowAtomics,
        active_owner: ShadowOwner,
    ) {
        if let Some(c) = candidate {
            self.first_unverifiable_seq = Some(match self.first_unverifiable_seq {
                Some(prev) => prev.min(c),
                None => c,
            });
            atomics
                .interval_unverifiable_mirror
                .store(true, Ordering::Release);
            if !self.overflow_logged_bb
                && self.write_bb_overflow(c, atomics, active_owner)
            {
                self.overflow_logged_bb = true;
            }
        }
    }

    fn write_bb_overflow(
        &self,
        cutoff_seq: u64,
        atomics: &ShadowOverflowAtomics,
        active_owner: ShadowOwner,
    ) -> bool {
        let Some(ref q) = self.bb_queue else {
            return false;
        };
        let t_ms = q.start.elapsed().as_millis() as u64;
        let line = format!(
            "{{\"boot\":{},\"t_ms\":{},\"kind\":\"torque_shadow\",\"event\":\"overflow\",\"owner\":\"{:?}\",\"cutoff_seq\":{},\"ring_dropped\":{},\"staging_dropped\":{},\"interval_unverifiable\":true}}\n",
            q.boot,
            t_ms,
            active_owner,
            cutoff_seq,
            atomics.ring_dropped_records.load(Ordering::Relaxed),
            atomics.staging_dropped_records.load(Ordering::Relaxed),
        );
        q.try_send(line)
    }

    fn write_bb_divergence(&self, entry: &ShadowDivergenceEntry) {
        let Some(ref q) = self.bb_queue else {
            return;
        };
        let t_ms = q.start.elapsed().as_millis() as u64;
        let payload = serde_json::json!({
            "boot": q.boot,
            "t_ms": t_ms,
            "kind": "torque_shadow",
            "event": "divergence",
            "owner": entry.owner,
            "seq": entry.seq,
            "release_id": entry.release_id,
            "now_us": entry.now_us,
            "category": entry.category,
            "detail": entry.detail,
            "violation": entry.violation,
            "coverage_gap": entry.coverage_gap,
            "reviewed_pr2": entry.reviewed_pr2,
        });
        if let Ok(mut s) = serde_json::to_string(&payload) {
            s.push('\n');
            q.try_send(s);
        }
    }

    fn push_recent(&mut self, entry: ShadowDivergenceEntry) {
        if self.recent_divergences.len() >= RECENT_DIVERGENCES_CAP {
            self.recent_divergences.pop_front();
        }
        self.recent_divergences.push_back(entry);
    }

    fn push_divergence(&mut self, entry: ShadowDivergenceEntry, atomics: &ShadowOverflowAtomics) {
        match entry.category {
            DivergenceCategory::ExtraAction => self.taxonomy.extra_action += 1,
            DivergenceCategory::MissingAction => self.taxonomy.missing_action += 1,
            DivergenceCategory::AuthorityRecipientMismatch => {
                self.taxonomy.authority_recipient_mismatch += 1
            }
            DivergenceCategory::CausalTimingOrder => self.taxonomy.causal_timing_order += 1,
            DivergenceCategory::OutcomeRefusalMismatch => {
                self.taxonomy.outcome_refusal_mismatch += 1
            }
            DivergenceCategory::InvariantViolation => self.taxonomy.invariant_violation += 1,
            DivergenceCategory::CoverageGap => self.taxonomy.coverage_gap += 1,
            DivergenceCategory::ReviewedPr2Only => self.taxonomy.reviewed_pr2_only += 1,
        }
        if let Some(gap) = entry.coverage_gap {
            self.coverage_gaps.bump(gap);
        }
        atomics
            .unexplained_divergences_mirror
            .store(self.taxonomy.unexplained_total(), Ordering::Release);
        atomics
            .invariant_violations_mirror
            .store(self.taxonomy.invariant_violation, Ordering::Release);

        let should_log_bb = !matches!(
            entry.category,
            DivergenceCategory::CoverageGap | DivergenceCategory::ReviewedPr2Only
        );
        if should_log_bb {
            self.write_bb_divergence(&entry);
        }
        self.push_recent(entry);
    }

    fn drain(&mut self, atomics: &ShadowOverflowAtomics, active_owner: ShadowOwner) -> usize {
        if !self.enabled {
            return 0;
        }
        let pre_cutoff = Self::effective_atomic_cutoff(atomics);
        self.update_cutoff(pre_cutoff, atomics, active_owner);

        let mut drained = 0usize;
        while let Some(rec) = self.rx.as_mut().and_then(|rx| rx.pop().ok()) {
            drained += 1;
            self.events_processed += 1;

            if rec.seq != self.expected_seq {
                let gap_seq = self.expected_seq.min(rec.seq);
                self.update_cutoff(Some(gap_seq), atomics, active_owner);
            }
            self.expected_seq = rec.seq.saturating_add(1);

            let live_cutoff = Self::effective_atomic_cutoff(atomics);
            self.update_cutoff(live_cutoff, atomics, active_owner);

            if self
                .first_unverifiable_seq
                .is_some_and(|cut| rec.seq >= cut)
            {
                self.last_state = ShadowLastState {
                    drive_state: rec.post_drive_state,
                    torque_intent: rec.post_intent,
                    release_id: rec.release_id,
                    now_us: rec.now_us,
                };
                continue;
            }

            self.consume_verifiable_record(rec, atomics);
        }

        let post_cutoff = Self::effective_atomic_cutoff(atomics);
        self.update_cutoff(post_cutoff, atomics, active_owner);
        drained
    }

    fn consume_verifiable_record(&mut self, rec: ShadowRecord, atomics: &ShadowOverflowAtomics) {
        self.last_state = ShadowLastState {
            drive_state: rec.post_drive_state,
            torque_intent: rec.post_intent,
            release_id: rec.release_id,
            now_us: rec.now_us,
        };

        // 1. Coverage gap accounting (preserve structured entry in recent_divergences).
        if let Some(gap) = rec.coverage_gap {
            self.taxonomy.coverage_gap += 1;
            self.coverage_gaps.bump(gap);
            self.push_recent(ShadowDivergenceEntry {
                seq: rec.seq,
                release_id: rec.release_id,
                now_us: rec.now_us,
                owner: rec.owner,
                category: DivergenceCategory::CoverageGap,
                detail: format!("coverage gap: {gap:?}"),
                violation: None,
                coverage_gap: Some(gap),
                reviewed_pr2: None,
            });
        }

        // 2. Expire unresolved predicted disable obligations on release / transition-wake boundaries
        //    using the replayed reducer dispatch lifecycle and exact effective deadline (never
        //    prematurely dropping an active budget-paused dispatch that is still within deadline).
        if matches!(
            rec.event,
            Some(Event::Release { .. } | Event::TransitionWake { .. })
        ) {
            if let Some(pending) = self.pending_disable {
                let dispatch_gone_in_reducer = self.state.as_ref().is_some_and(|st| {
                    st.groups[0]
                        .pending_disable_dispatch
                        .is_none_or(|p| p.dispatch_id != pending.dispatch_id)
                });
                let past_deadline = pending.deadline_us > 0 && rec.now_us >= pending.deadline_us;
                let stale_after_gone = dispatch_gone_in_reducer
                    && rec.release_id > pending.last_progress_release_id.saturating_add(1);
                if !pending.remaining_recipients.is_empty() && (past_deadline || stale_after_gone) {
                    self.pending_disable = None;
                    self.push_divergence(
                        ShadowDivergenceEntry {
                            seq: rec.seq,
                            release_id: rec.release_id,
                            now_us: rec.now_us,
                            owner: pending.owner,
                            category: DivergenceCategory::ExtraAction,
                            detail: format!(
                                "predicted disable {:?} (dispatch_id={}, gen={}) unfulfilled for recipients {:#x} by release {}",
                                pending.kind,
                                pending.dispatch_id,
                                pending.generation,
                                pending.remaining_recipients.raw(),
                                rec.release_id
                            ),
                            violation: None,
                            coverage_gap: None,
                            reviewed_pr2: None,
                        },
                        atomics,
                    );
                }
            }
        }

        // 3. Replay off-thread reducer & run independent machine-checked invariants.
        match rec.kind {
            ShadowRecordKind::JointZeroRecalibrated => {
                if let Some(ref mut st) = self.state {
                    apply_joint_zero_recalibration(st);
                }
                self.pending_disable = None;
                return;
            }
            ShadowRecordKind::Step => {
                if let Some(ev) = rec.event {
                    let (violation_opt, off_summary) = if let (Some(st), Some(prev)) =
                        (self.state.as_mut(), self.prev_state.as_mut())
                    {
                        **prev = **st;
                        let off_actions = step(st, ev);
                        let v = check_invariants(st, prev, &ev, &off_actions).err();
                        (v, ActionSummary::from_action_set(&off_actions))
                    } else {
                        (None, ActionSummary::default())
                    };
                    if let Some(v) = violation_opt {
                        self.push_divergence(
                            ShadowDivergenceEntry {
                                seq: rec.seq,
                                release_id: rec.release_id,
                                now_us: rec.now_us,
                                owner: rec.owner,
                                category: DivergenceCategory::InvariantViolation,
                                detail: format!("reducer invariant violation: {:?}", v.kind),
                                violation: Some(v),
                                coverage_gap: None,
                                reviewed_pr2: None,
                            },
                            atomics,
                        );
                    }
                    if rec.rt_actions != off_summary {
                        let cat = classify_rt_vs_off_mismatch(&rec.rt_actions, &off_summary);
                        self.push_divergence(
                            ShadowDivergenceEntry {
                                seq: rec.seq,
                                release_id: rec.release_id,
                                now_us: rec.now_us,
                                owner: rec.owner,
                                category: cat,
                                detail: format!(
                                    "RT shadow action summary diverged from reference reducer (rt={:?}, ref={:?})",
                                    rec.rt_actions, off_summary
                                ),
                                violation: None,
                                coverage_gap: None,
                                reviewed_pr2: None,
                            },
                            atomics,
                        );
                    }
                }
            }
            ShadowRecordKind::CoverageGapOnly => {}
        }

        // Track new reducer disable predictions for receipt reconciliation (skip PR2-exempt ProcessExit).
        if let Some(dk) = rec.rt_actions.disable_kind {
            if !rec.rt_actions.disable_recipients.is_empty()
                && !matches!(
                    rec.reviewed_pr2,
                    Some(
                        ReviewedPr2Kind::FuPanicBaselineMismatch
                            | ReviewedPr2Kind::ProcessExitAlreadyDisabled
                    )
                )
            {
                if let Some(prev_pending) = self.pending_disable {
                    if prev_pending.dispatch_id != rec.rt_actions.disable_dispatch_id
                        && !prev_pending.remaining_recipients.is_empty()
                    {
                        self.push_divergence(
                            ShadowDivergenceEntry {
                                seq: rec.seq,
                                release_id: rec.release_id,
                                now_us: rec.now_us,
                                owner: prev_pending.owner,
                                category: DivergenceCategory::ExtraAction,
                                detail: format!(
                                    "predicted disable {:?} (dispatch_id={}, gen={}) unfulfilled for recipients {:#x} before superseded by dispatch_id={}",
                                    prev_pending.kind,
                                    prev_pending.dispatch_id,
                                    prev_pending.generation,
                                    prev_pending.remaining_recipients.raw(),
                                    rec.rt_actions.disable_dispatch_id
                                ),
                                violation: None,
                                coverage_gap: None,
                                reviewed_pr2: None,
                            },
                            atomics,
                        );
                    }
                }
                self.pending_disable = Some(PendingDisableExpectation {
                    kind: dk,
                    generation: rec.join.generation,
                    dispatch_id: rec.rt_actions.disable_dispatch_id,
                    deadline_us: rec.rt_actions.disable_deadline_us,
                    last_progress_release_id: rec.release_id,
                    expected_recipients: rec.rt_actions.disable_recipients,
                    remaining_recipients: rec.rt_actions.disable_recipients,
                    owner: rec.owner,
                });
            }
        }

        // 4. Reviewed PR2-only classification (visible with structured detail, single comparison count).
        let mut comparison_counted = false;
        if let Some(pr2) = rec.reviewed_pr2 {
            self.taxonomy.reviewed_pr2_only += 1;
            self.push_recent(ShadowDivergenceEntry {
                seq: rec.seq,
                release_id: rec.release_id,
                now_us: rec.now_us,
                owner: rec.owner,
                category: DivergenceCategory::ReviewedPr2Only,
                detail: format!("reviewed PR2 difference: {pr2:?}"),
                violation: None,
                coverage_gap: None,
                reviewed_pr2: Some(pr2),
            });
            if matches!(
                pr2,
                ReviewedPr2Kind::FuPanicBaselineMismatch
                    | ReviewedPr2Kind::ProcessExitAlreadyDisabled
                    | ReviewedPr2Kind::DriverPreflightVsAwaitRestGate
            ) {
                if rec.live.comparable {
                    self.comparisons_total += 1;
                }
                return;
            }
        }

        // 5. Compare reducer prediction against live decision when comparable (count once).
        if !rec.live.comparable {
            return;
        }
        if !comparison_counted {
            self.comparisons_total += 1;
            comparison_counted = true;
        }
        let _ = comparison_counted;
        self.compare_live_vs_predicted(&rec, atomics);
    }

    fn compare_live_vs_predicted(&mut self, rec: &ShadowRecord, atomics: &ShadowOverflowAtomics) {
        let pred = &rec.rt_actions;
        let live = &rec.live;

        // 5a. Unmatched live hardware write (live sent disable/enable with no reducer capability).
        if rec.join.unmatched_live_write {
            self.push_divergence(
                ShadowDivergenceEntry {
                    seq: rec.seq,
                    release_id: rec.release_id,
                    now_us: rec.now_us,
                    owner: rec.owner,
                    category: DivergenceCategory::MissingAction,
                    detail: format!(
                        "unmatched live hardware write: kind={:?} recipients={:#x}",
                        live.live_disable_kind,
                        live.live_disable_recipients.raw()
                    ),
                    violation: None,
                    coverage_gap: None,
                    reviewed_pr2: None,
                },
                atomics,
            );
            return;
        }

        // 5b. Reconcile live disable WriteResult against pending predicted disable obligations.
        if let (Some(Event::WriteResult(wr)), Some(lk)) = (rec.event, live.live_disable_kind) {
            if let Some(mut exp) = self.pending_disable {
                if lk != exp.kind || rec.join.generation != exp.generation {
                    self.push_divergence(
                        ShadowDivergenceEntry {
                            seq: rec.seq,
                            release_id: rec.release_id,
                            now_us: rec.now_us,
                            owner: rec.owner,
                            category: DivergenceCategory::AuthorityRecipientMismatch,
                            detail: format!(
                                "live disable write authority mismatch: pred=({:?}, gen={}) live=({lk:?}, gen={})",
                                exp.kind, exp.generation, rec.join.generation
                            ),
                            violation: None,
                            coverage_gap: None,
                            reviewed_pr2: None,
                        },
                        atomics,
                    );
                } else {
                    let unexpected = ChannelMask(
                        live.live_disable_recipients.raw() & !exp.expected_recipients.raw(),
                    );
                    if !unexpected.is_empty() {
                        self.push_divergence(
                            ShadowDivergenceEntry {
                                seq: rec.seq,
                                release_id: rec.release_id,
                                now_us: rec.now_us,
                                owner: rec.owner,
                                category: DivergenceCategory::AuthorityRecipientMismatch,
                                detail: format!(
                                    "live disable targeted unexpected recipients {:#x} (expected subset of {:#x})",
                                    unexpected.raw(),
                                    exp.expected_recipients.raw()
                                ),
                                violation: None,
                                coverage_gap: None,
                                reviewed_pr2: None,
                            },
                            atomics,
                        );
                    }
                    // Mirror the reducer's single-repeat (`disable_repeats == 1`) retry rule:
                    // when >= 1 recipient has already been sent/ambiguous and other recipients
                    // reported `Deferred`/`LockContended`, keep those deferred recipients in
                    // `exp.remaining_recipients` so the driver's subsequent retry `Sent` reconciles
                    // cleanly against the same `dispatch_id`.
                    let single_repeat = self
                        .state
                        .as_ref()
                        .is_some_and(|st| st.config.groups[0].disable_repeats == 1);
                    let mut deferred_in_wr = ChannelMask::EMPTY;
                    let mut any_sent_or_ambiguous_in_wr = false;
                    for r in live.live_disable_recipients.iter() {
                        match wr.outcomes[r as usize] {
                            MotorWriteOutcome::Sent | MotorWriteOutcome::Ambiguous => {
                                any_sent_or_ambiguous_in_wr = true;
                            }
                            MotorWriteOutcome::Deferred | MotorWriteOutcome::LockContended => {
                                deferred_in_wr.insert(r);
                            }
                            _ => {}
                        }
                    }
                    let already_had_sent =
                        exp.remaining_recipients != exp.expected_recipients;
                    let settled_mask = if single_repeat
                        && (any_sent_or_ambiguous_in_wr || already_had_sent)
                    {
                        live.live_disable_recipients.difference(deferred_in_wr)
                    } else {
                        live.live_disable_recipients
                    };
                    let prev_remaining = exp.remaining_recipients;
                    exp.remaining_recipients = exp.remaining_recipients.difference(settled_mask);
                    if exp.remaining_recipients != prev_remaining {
                        exp.last_progress_release_id = rec.release_id;
                    }
                    self.pending_disable = if exp.remaining_recipients.is_empty() {
                        None
                    } else {
                        Some(exp)
                    };
                }
            }
            if pred.disable_completed && live.live_partial_disable != pred.disable_partial {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: DivergenceCategory::OutcomeRefusalMismatch,
                        detail: format!(
                            "disable partiality mismatch: live_partial={} pred_partial={}",
                            live.live_partial_disable, pred.disable_partial
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
            return;
        }

        // 5c. Direct disable recipient/kind mismatch or extra disable on non-WriteResult records.
        if let (Some(pk), Some(lk)) = (pred.disable_kind, live.live_disable_kind) {
            if pk != lk || pred.disable_recipients != live.live_disable_recipients {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: DivergenceCategory::AuthorityRecipientMismatch,
                        detail: format!(
                            "disable mismatch: pred=({pk:?}, {:#x}) live=({lk:?}, {:#x})",
                            pred.disable_recipients.raw(),
                            live.live_disable_recipients.raw()
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
        } else if pred.disable_kind.is_some()
            && live.live_disable_kind.is_none()
            && !matches!(rec.event, Some(Event::DisableRequest { .. }))
        {
            self.push_divergence(
                ShadowDivergenceEntry {
                    seq: rec.seq,
                    release_id: rec.release_id,
                    now_us: rec.now_us,
                    owner: rec.owner,
                    category: DivergenceCategory::ExtraAction,
                    detail: format!(
                        "reducer predicted disable {:?} ({:#x}) with no live disable",
                        pred.disable_kind,
                        pred.disable_recipients.raw()
                    ),
                    violation: None,
                    coverage_gap: None,
                    reviewed_pr2: None,
                },
                atomics,
            );
        }

        // 5d. Refusal vs acceptance parity AND refusal-code equivalence.
        match (pred.refusal, live.live_refusal) {
            (Some(pr), Some(lr)) => {
                if !refusal_matches_live(pr, lr) {
                    self.push_divergence(
                        ShadowDivergenceEntry {
                            seq: rec.seq,
                            release_id: rec.release_id,
                            now_us: rec.now_us,
                            owner: rec.owner,
                            category: DivergenceCategory::OutcomeRefusalMismatch,
                            detail: format!(
                                "refusal code mismatch: pred={pr:?} live={lr:?} event={:?}",
                                rec.event
                            ),
                            violation: None,
                            coverage_gap: None,
                            reviewed_pr2: None,
                        },
                        atomics,
                    );
                }
            }
            (Some(pr), None) => {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: DivergenceCategory::OutcomeRefusalMismatch,
                        detail: format!(
                            "reducer refused ({pr:?}) while live accepted event={:?}",
                            rec.event
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
            (None, Some(lr)) => {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: DivergenceCategory::OutcomeRefusalMismatch,
                        detail: format!(
                            "live refused ({lr:?}) while reducer accepted event={:?}",
                            rec.event
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
            (None, None) => {}
        }

        // 5e. EnableStart live vs reducer authorization parity.
        if matches!(rec.event, Some(Event::EnableStart { .. })) {
            let pred_started = !pred.enable_mask.is_empty();
            if pred_started != live.live_enable_started && pred.refusal.is_none() && live.live_refusal.is_none() {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: if pred_started {
                            DivergenceCategory::ExtraAction
                        } else {
                            DivergenceCategory::MissingAction
                        },
                        detail: format!(
                            "enable start mismatch: pred_started={pred_started} live_started={}",
                            live.live_enable_started
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
        }

        // 5f. MotionCommand live vs reducer authorization parity.
        if let Some(live_allowed) = live.live_motion_allowed {
            let pred_allowed = pred.motion_allowed.is_some_and(|m| !m.is_empty());
            if pred_allowed != live_allowed && pred.refusal.is_none() && live.live_refusal.is_none() {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: DivergenceCategory::OutcomeRefusalMismatch,
                        detail: format!(
                            "motion admission mismatch: pred_allowed={pred_allowed} live_allowed={live_allowed}"
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
        }

        // 5g. Stopped-hold contention escalation parity: the reducer evaluates streak escalation
        //     on the next `Release` boundary (`reducer.rs:483`) rather than on the individual
        //     `WriteResult(Hold)` receipt, so compare escalation against the reducer's device
        //     `contention_escalated` latch state rather than only the single-step `ActionSummary`.
        if matches!(
            rec.event,
            Some(Event::WriteResult(WriteResult {
                kind: WriteKind::Hold,
                ..
            }))
        ) {
            let pred_escalated = !pred.contention_escalated.is_empty()
                || self
                    .state
                    .as_ref()
                    .is_some_and(|st| st.devices[0].contention_escalated);
            if live.live_contention_escalated != pred_escalated {
                self.push_divergence(
                    ShadowDivergenceEntry {
                        seq: rec.seq,
                        release_id: rec.release_id,
                        now_us: rec.now_us,
                        owner: rec.owner,
                        category: DivergenceCategory::OutcomeRefusalMismatch,
                        detail: format!(
                            "hold contention escalation mismatch: pred={pred_escalated} live={}",
                            live.live_contention_escalated
                        ),
                        violation: None,
                        coverage_gap: None,
                        reviewed_pr2: None,
                    },
                    atomics,
                );
            }
        }

        // 5h. Stopped hold mask comparison (LiveHoldCheckpoint — compares non-empty AND empty live masks).
        if matches!(rec.kind, ShadowRecordKind::CoverageGapOnly)
            && rec.event.is_none()
            && live.live_stopped
            && live.live_intent == TorqueIntent::On
        {
            let pred_hold = rec.predicted_hold_mask;
            let live_hold = live.live_hold_mask;
            if pred_hold != live_hold {
                let pred_only = pred_hold.difference(live_hold);
                let live_only = live_hold.difference(pred_hold);
                let all_live_only_verified_stale = pred_only.is_empty()
                    && !live_only.is_empty()
                    && live_only.is_subset_of(live.live_stale_held_mask)
                    && self.state.as_ref().is_some_and(|st| {
                        live_only.iter().all(|c| {
                            if c >= st.config.channel_count {
                                return false;
                            }
                            let ch_cfg = &st.config.channels[c as usize];
                            let rt = &st.joints[c as usize];
                            rt.had_valid_capture
                                && rt.last_sample_us.is_none_or(|ts| {
                                    !crate::torque::reducer::is_sample_fresh(
                                        ts,
                                        rec.now_us,
                                        ch_cfg.max_capture_age_us,
                                    )
                                })
                        })
                    });
                if all_live_only_verified_stale {
                    self.push_divergence(
                        ShadowDivergenceEntry {
                            seq: rec.seq,
                            release_id: rec.release_id,
                            now_us: rec.now_us,
                            owner: rec.owner,
                            category: DivergenceCategory::ReviewedPr2Only,
                            detail: format!(
                                "reviewed PR2 difference: live held stale target(s) {:#x} beyond capture age (pred={:#x} live={:#x})",
                                live_only.raw(),
                                pred_hold.raw(),
                                live_hold.raw()
                            ),
                            violation: None,
                            coverage_gap: None,
                            reviewed_pr2: Some(ReviewedPr2Kind::LiveHeldStaleTarget),
                        },
                        atomics,
                    );
                } else {
                    let cat = classify_mask_diff(pred_hold, live_hold);
                    self.push_divergence(
                        ShadowDivergenceEntry {
                            seq: rec.seq,
                            release_id: rec.release_id,
                            now_us: rec.now_us,
                            owner: rec.owner,
                            category: cat,
                            detail: format!(
                                "hold mask mismatch: pred={:#x} live={:#x}",
                                pred_hold.raw(),
                                live_hold.raw()
                            ),
                            violation: None,
                            coverage_gap: None,
                            reviewed_pr2: None,
                        },
                        atomics,
                    );
                }
            }
        }
    }

    fn snapshot(
        &self,
        atomics: &ShadowOverflowAtomics,
        active_owner: ShadowOwner,
        bb_written: u64,
        bb_dropped: u64,
    ) -> TorqueShadowDiagnostics {
        if !self.enabled {
            return TorqueShadowDiagnostics::disabled(active_owner);
        }
        let ring_dropped = atomics.ring_dropped_records.load(Ordering::Relaxed);
        let ring_ov = atomics.ring_overflow_events.load(Ordering::Relaxed);
        let staging_dropped = atomics.staging_dropped_records.load(Ordering::Relaxed);
        let staging_ov = atomics.staging_overflow_events.load(Ordering::Relaxed);
        let atomic_cutoff = Self::effective_atomic_cutoff(atomics);
        let first_unverifiable_seq = match (self.first_unverifiable_seq, atomic_cutoff) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        };
        let interval_unverifiable = first_unverifiable_seq.is_some()
            || atomics.interval_unverifiable_mirror.load(Ordering::Acquire);

        TorqueShadowDiagnostics {
            enabled: true,
            owner: active_owner,
            events_processed: self.events_processed,
            comparisons_total: self.comparisons_total,
            unexplained_divergences_total: self.taxonomy.unexplained_total(),
            taxonomy: self.taxonomy,
            coverage_gaps_by_kind: self.coverage_gaps,
            ring: ShadowRingDiagnostics {
                capacity: SHADOW_RING_CAPACITY,
                ring_dropped_records: ring_dropped,
                ring_overflow_events: ring_ov,
                staging_dropped_records: staging_dropped,
                staging_overflow_events: staging_ov,
                first_unverifiable_seq,
                interval_unverifiable,
                blackbox_written: bb_written,
                blackbox_dropped: bb_dropped,
            },
            last_state: self.last_state,
            recent_divergences: self.recent_divergences.iter().cloned().collect(),
        }
    }
}

/// Set-difference mask divergence classifier:
/// - `pred` has bits missing in `live` AND `live` has extra bits not in `pred` -> `AuthorityRecipientMismatch`
/// - `pred` is a strict superset of `live` (`pred` predicted extra holds/actions) -> `ExtraAction`
/// - `live` is a strict superset of `pred` (`pred` missed actions live executed) -> `MissingAction`
#[inline]
fn classify_mask_diff(pred: ChannelMask, live: ChannelMask) -> DivergenceCategory {
    let pred_only = pred.raw() & !live.raw();
    let live_only = live.raw() & !pred.raw();
    if pred_only != 0 && live_only != 0 {
        DivergenceCategory::AuthorityRecipientMismatch
    } else if pred_only != 0 {
        DivergenceCategory::ExtraAction
    } else {
        DivergenceCategory::MissingAction
    }
}

#[inline]
fn refusal_matches_live(pred: RefusalCode, live: LiveRefusalKind) -> bool {
    matches!(
        (pred, live),
        (
            RefusalCode::TorqueOffWouldDrop
                | RefusalCode::RestGateTimeout
                | RefusalCode::RestEvidenceUnqualified
                | RefusalCode::HandReleaseNotAuthorized
                | RefusalCode::ReleaseEvidenceTimeout
                | RefusalCode::ActiveToolReleaseNotAuthorized,
            LiveRefusalKind::TorqueOffWouldDrop | LiveRefusalKind::ArmNotAtRestingPose,
        ) | (
            RefusalCode::EnableFailed | RefusalCode::ModeUnverified | RefusalCode::TimeoutMismatch,
            LiveRefusalKind::EnableFailed | LiveRefusalKind::Other,
        ) | (
            RefusalCode::DisableInProgress,
            LiveRefusalKind::DisableInProgress | LiveRefusalKind::BusyOrTransition,
        ) | (
            RefusalCode::FaultLatched
                | RefusalCode::ProtectiveLatched
                | RefusalCode::ThermalActive
                | RefusalCode::InternalFault
                | RefusalCode::ControlPanicSafeHold,
            LiveRefusalKind::FaultLatched | LiveRefusalKind::EStopActive,
        ) | (
            RefusalCode::PreemptedByStop | RefusalCode::PreemptedByProtective,
            LiveRefusalKind::EStopActive
                | LiveRefusalKind::FaultLatched
                | LiveRefusalKind::BusyOrTransition,
        ) | (
            RefusalCode::AlreadyEnabled | RefusalCode::NotEnabled | RefusalCode::ZeroWritesRefused,
            LiveRefusalKind::BusyOrTransition | LiveRefusalKind::Other,
        ) | (
            RefusalCode::RecipientMaskInvalid | RefusalCode::GroupOutOfRange,
            LiveRefusalKind::Unsupported | LiveRefusalKind::Other,
        )
    )
}

fn classify_rt_vs_off_mismatch(rt: &ActionSummary, reference: &ActionSummary) -> DivergenceCategory {
    if rt.disable_kind != reference.disable_kind {
        if rt.disable_kind.is_some() && reference.disable_kind.is_none() {
            DivergenceCategory::ExtraAction
        } else if rt.disable_kind.is_none() && reference.disable_kind.is_some() {
            DivergenceCategory::MissingAction
        } else {
            DivergenceCategory::AuthorityRecipientMismatch
        }
    } else if rt.disable_recipients != reference.disable_recipients {
        DivergenceCategory::AuthorityRecipientMismatch
    } else if rt.refusal != reference.refusal {
        DivergenceCategory::OutcomeRefusalMismatch
    } else if rt.hold_mask != reference.hold_mask {
        classify_mask_diff(rt.hold_mask, reference.hold_mask)
    } else {
        DivergenceCategory::CausalTimingOrder
    }
}

#[inline]
fn owner_to_u8(owner: ShadowOwner) -> u8 {
    match owner {
        ShadowOwner::Legacy => 0,
        ShadowOwner::Executor => 1,
    }
}

#[inline]
fn owner_from_u8(v: u8) -> ShadowOwner {
    match v {
        1 => ShadowOwner::Executor,
        _ => ShadowOwner::Legacy,
    }
}

/// Shared worker core separated from [`TorqueShadowHub`] so `"armd-torque-shadow"` holds only
/// `Arc<ShadowHubCore>` and NEVER keeps `TorqueShadowHub` alive via an `Arc` cycle.
struct ShadowHubCore {
    enabled: bool,
    active_owner: AtomicU8,
    overflow: Arc<ShadowOverflowAtomics>,
    bb_written: Arc<AtomicU64>,
    bb_dropped: Arc<AtomicU64>,
    consumer: Mutex<TorqueShadowConsumer>,
    published: Mutex<TorqueShadowDiagnostics>,
    wake_mu: Mutex<bool>,
    wake_cv: Condvar,
    stop: AtomicBool,
    retired_head: AtomicPtr<ShadowRuntimeInner>,
    retired_reclaimed: AtomicU64,
}

impl ShadowHubCore {
    #[inline]
    fn active_owner(&self) -> ShadowOwner {
        owner_from_u8(self.active_owner.load(Ordering::Acquire))
    }

    fn reclaim_retired(&self) {
        reclaim_retired_driver_staging();
        let mut cur = self
            .retired_head
            .swap(std::ptr::null_mut(), Ordering::AcqRel);
        while !cur.is_null() {
            // SAFETY: each node was pushed once via `Box::into_raw` by `ShadowRuntime::drop`
            // and exclusively unlinked by the atomic `swap` above.
            let node = unsafe { Box::from_raw(cur) };
            cur = node.next_retired;
            drop(node);
            self.retired_reclaimed.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn drain_and_publish(&self) -> usize {
        self.reclaim_retired();
        if !self.enabled {
            return 0;
        }
        let mut c = self.consumer.lock().unwrap_or_else(|e| e.into_inner());
        let owner = self.active_owner();
        let n = c.drain(&self.overflow, owner);
        let snap = c.snapshot(
            &self.overflow,
            owner,
            self.bb_written.load(Ordering::Relaxed),
            self.bb_dropped.load(Ordering::Relaxed),
        );
        // Linearize publication strictly under the `consumer` lock so concurrent drains can never
        // overwrite a newer snapshot with an older one.
        *self.published.lock().unwrap_or_else(|e| e.into_inner()) = snap;
        n
    }

    /// Non-blocking drain variant for bounded shutdown paths: if a detached timed-out worker
    /// still holds `consumer`, returns immediately rather than blocking past the shutdown deadline.
    fn try_drain_and_publish(&self) -> usize {
        self.reclaim_retired();
        if !self.enabled {
            return 0;
        }
        let Ok(mut c) = self.consumer.try_lock() else {
            return 0;
        };
        let owner = self.active_owner();
        let n = c.drain(&self.overflow, owner);
        let snap = c.snapshot(
            &self.overflow,
            owner,
            self.bb_written.load(Ordering::Relaxed),
            self.bb_dropped.load(Ordering::Relaxed),
        );
        if let Ok(mut pub_snap) = self.published.try_lock() {
            *pub_snap = snap;
        }
        n
    }
}

impl Drop for ShadowHubCore {
    fn drop(&mut self) {
        self.reclaim_retired();
    }
}

/// Shared off-thread hub owned by `ArmSupervisor` (never locked on `armd-control`!).
///
/// Manages the dedicated `"armd-torque-shadow"` consumer thread, independently publishes
/// [`TorqueShadowDiagnostics`] for `GET /api/v1/diagnostics` without draining or touching disk,
/// and owns the decoupled `"armd-shadow-bb"` rotating writer channel.
pub struct TorqueShadowHub {
    core: Arc<ShadowHubCore>,
    worker: Mutex<Option<(JoinHandle<()>, Arc<(Mutex<bool>, Condvar)>)>>,
    /// Latched when `stop_and_flush_bounded` times out waiting for a worker so subsequent
    /// shutdown/Drop calls never mistake the empty worker slot for a completed worker.
    worker_timed_out: AtomicBool,
}

impl TorqueShadowHub {
    fn disabled(overflow: Arc<ShadowOverflowAtomics>) -> Self {
        Self {
            core: Arc::new(ShadowHubCore {
                enabled: false,
                active_owner: AtomicU8::new(owner_to_u8(ShadowOwner::Legacy)),
                overflow,
                bb_written: Arc::new(AtomicU64::new(0)),
                bb_dropped: Arc::new(AtomicU64::new(0)),
                consumer: Mutex::new(TorqueShadowConsumer::new_disabled()),
                published: Mutex::new(TorqueShadowDiagnostics::disabled(ShadowOwner::Legacy)),
                wake_mu: Mutex::new(false),
                wake_cv: Condvar::new(),
                stop: AtomicBool::new(false),
                retired_head: AtomicPtr::new(std::ptr::null_mut()),
                retired_reclaimed: AtomicU64::new(0),
            }),
            worker: Mutex::new(None),
            worker_timed_out: AtomicBool::new(false),
        }
    }

    fn enabled(
        rx: Consumer<ShadowRecord>,
        state: Box<TorqueState>,
        overflow: Arc<ShadowOverflowAtomics>,
        owner: ShadowOwner,
    ) -> Self {
        let consumer = TorqueShadowConsumer::new_enabled(rx, state);
        let initial_snap = consumer.snapshot(&overflow, owner, 0, 0);
        Self {
            core: Arc::new(ShadowHubCore {
                enabled: true,
                active_owner: AtomicU8::new(owner_to_u8(owner)),
                overflow,
                bb_written: Arc::new(AtomicU64::new(0)),
                bb_dropped: Arc::new(AtomicU64::new(0)),
                consumer: Mutex::new(consumer),
                published: Mutex::new(initial_snap),
                wake_mu: Mutex::new(false),
                wake_cv: Condvar::new(),
                stop: AtomicBool::new(false),
                retired_head: AtomicPtr::new(std::ptr::null_mut()),
                retired_reclaimed: AtomicU64::new(0),
            }),
            worker: Mutex::new(None),
            worker_timed_out: AtomicBool::new(false),
        }
    }

    fn retire_runtime_inner(&self, boxed: Box<ShadowRuntimeInner>) {
        let raw = Box::into_raw(boxed);
        loop {
            let head = self.core.retired_head.load(Ordering::Acquire);
            // SAFETY: `raw` is uniquely owned here before publishing into the Treiber stack.
            unsafe {
                (*raw).next_retired = head;
            }
            if self
                .core
                .retired_head
                .compare_exchange_weak(head, raw, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                break;
            }
        }
        // Strictly lock-free: do NOT call `wake()` (which attempts `wake_mu.try_lock()`) from
        // `Drop` on `armd-control`. The off-thread worker reclaims on its next poll or shutdown.
    }

    /// Test-only counter of retired `ShadowRuntimeInner` allocations reclaimed off-thread.
    #[doc(hidden)]
    pub fn test_retired_reclaimed_count(&self) -> u64 {
        self.core.retired_reclaimed.load(Ordering::Relaxed)
    }

    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.core.enabled
    }

    #[inline]
    pub fn overflow_atomics(&self) -> &Arc<ShadowOverflowAtomics> {
        &self.core.overflow
    }

    /// Lock-free staging-loss latch usable without holding `SupervisorInner` (e.g. when a
    /// post-panic helper thread cannot acquire `inner.try_write()` or finds `detached`).
    #[inline]
    pub fn record_contended_staging_loss(&self, dropped: u64) {
        if !self.core.enabled || dropped == 0 {
            return;
        }
        let cutoff = self
            .core
            .overflow
            .last_allocated_seq
            .load(Ordering::Acquire)
            .saturating_add(1);
        self.core.overflow.record_staging_loss(cutoff, dropped);
    }

    /// Rebind consumer SPSC endpoint and reducer `TorqueState` on driver mount/remount while
    /// preserving session-wide overflow latches, sequence continuity, counters, and blackbox sink.
    pub fn reinit_consumer_preserving_session(
        &self,
        rx: Consumer<ShadowRecord>,
        state: Box<TorqueState>,
        next_expected_seq: u64,
    ) {
        if !self.core.enabled {
            return;
        }
        let mut c = self.core.consumer.lock().unwrap_or_else(|e| e.into_inner());
        let prev_state = state.clone();
        c.rx = Some(rx);
        c.state = Some(state);
        c.prev_state = Some(prev_state);
        c.expected_seq = next_expected_seq;
        c.pending_disable = None;
        let owner = self.core.active_owner();
        let snap = c.snapshot(
            &self.core.overflow,
            owner,
            self.core.bb_written.load(Ordering::Relaxed),
            self.core.bb_dropped.load(Ordering::Relaxed),
        );
        *self.core.published.lock().unwrap_or_else(|e| e.into_inner()) = snap;
    }

    /// Lock-free active owner update (never acquires comparator mutex or waits on disk I/O).
    #[inline]
    pub fn set_owner(&self, owner: ShadowOwner) {
        self.core
            .active_owner
            .store(owner_to_u8(owner), Ordering::Release);
    }

    /// Attach a dedicated `bb_shadow.jsonl` file inside `dir` (opens the file inside the
    /// decoupled `"armd-shadow-bb"` thread so startup/primary `bb.jsonl` never waits on shadow disk I/O).
    pub fn attach_blackbox_dir(&self, dir: &Path, boot: u64, start: Instant) -> io::Result<()> {
        if !self.core.enabled {
            return Ok(());
        }
        let q = ShadowBbQueue::spawn_dir(
            dir.join(SHADOW_BB_FILE),
            boot,
            start,
            Arc::clone(&self.core.bb_written),
            Arc::clone(&self.core.bb_dropped),
        )?;
        let mut c = self.core.consumer.lock().unwrap_or_else(|e| e.into_inner());
        c.bb_queue = Some(q);
        Ok(())
    }

    /// Attach a custom [`ShadowBlackBoxSink`] behind the decoupled `"armd-shadow-bb"` writer thread.
    pub fn attach_blackbox_sink(
        &self,
        sink: Box<dyn ShadowBlackBoxSink>,
        boot: u64,
        start: Instant,
    ) {
        if !self.core.enabled {
            return;
        }
        if let Ok(q) = ShadowBbQueue::spawn(
            sink,
            boot,
            start,
            Arc::clone(&self.core.bb_written),
            Arc::clone(&self.core.bb_dropped),
        ) {
            let mut c = self.core.consumer.lock().unwrap_or_else(|e| e.into_inner());
            c.bb_queue = Some(q);
        }
    }

    /// Spawn the dedicated `"armd-torque-shadow"` off-thread consumer (idempotent; no-op when disabled).
    /// Holds only `Arc<ShadowHubCore>` so dropping `TorqueShadowHub` stops & joins cleanly.
    pub fn start_worker(self: &Arc<Self>) -> io::Result<()> {
        if !self.core.enabled {
            return Ok(());
        }
        let mut slot = self.worker.lock().unwrap_or_else(|e| e.into_inner());
        if slot.is_some() {
            return Ok(());
        }
        let core = Arc::clone(&self.core);
        let done = Arc::new((Mutex::new(false), Condvar::new()));
        let done_bg = Arc::clone(&done);
        let jh = thread::Builder::new()
            .name("armd-torque-shadow".into())
            .spawn(move || {
                while !core.stop.load(Ordering::Acquire) {
                    core.drain_and_publish();
                    let guard = core.wake_mu.lock().unwrap_or_else(|e| e.into_inner());
                    if !core.stop.load(Ordering::Acquire) {
                        let _ = core.wake_cv.wait_timeout(guard, Duration::from_millis(20));
                    }
                }
                core.drain_and_publish();
                let mut g = done_bg.0.lock().unwrap_or_else(|e| e.into_inner());
                *g = true;
                done_bg.1.notify_all();
            })?;
        *slot = Some((jh, done));
        self.worker_timed_out.store(false, Ordering::Release);
        Ok(())
    }

    /// True if the dedicated background consumer thread has been started.
    pub fn has_worker(&self) -> bool {
        self.worker
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some()
    }

    /// Non-blocking wakeup signal for the dedicated consumer thread.
    pub fn wake(&self) {
        if !self.core.enabled {
            return;
        }
        if let Ok(mut flag) = self.core.wake_mu.try_lock() {
            *flag = true;
            self.core.wake_cv.notify_one();
        }
    }

    /// Drain all queued SPSC records and publish the updated [`TorqueShadowDiagnostics`] snapshot.
    /// Safe to call off the control thread (worker loop, shutdown drain, or tests); NEVER blocks on disk I/O.
    pub fn drain_now(&self) -> usize {
        self.core.drain_and_publish()
    }

    /// Read the latest published diagnostics snapshot WITHOUT acquiring the comparator lock,
    /// WITHOUT draining the ring on the caller thread, and WITHOUT performing disk I/O, while
    /// overlaying fresh lock-free atomic owner & overflow/loss watermarks.
    pub fn snapshot(&self) -> TorqueShadowDiagnostics {
        let active_owner = self.core.active_owner();
        if !self.core.enabled {
            return TorqueShadowDiagnostics::disabled(active_owner);
        }
        let mut snap = self
            .core
            .published
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        snap.owner = active_owner;

        let atomics = &self.core.overflow;
        snap.ring.ring_dropped_records = atomics.ring_dropped_records.load(Ordering::Relaxed);
        snap.ring.ring_overflow_events = atomics.ring_overflow_events.load(Ordering::Relaxed);
        snap.ring.staging_dropped_records = atomics.staging_dropped_records.load(Ordering::Relaxed);
        snap.ring.staging_overflow_events = atomics.staging_overflow_events.load(Ordering::Relaxed);
        snap.ring.blackbox_written = self.core.bb_written.load(Ordering::Relaxed);
        snap.ring.blackbox_dropped = self.core.bb_dropped.load(Ordering::Relaxed);
        let atomic_cutoff = TorqueShadowConsumer::effective_atomic_cutoff(atomics);
        snap.ring.first_unverifiable_seq = match (snap.ring.first_unverifiable_seq, atomic_cutoff) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        };
        snap.ring.interval_unverifiable = snap.ring.interval_unverifiable
            || snap.ring.first_unverifiable_seq.is_some()
            || atomics.interval_unverifiable_mirror.load(Ordering::Acquire);
        snap
    }

    /// Stop the consumer worker and flush queued shadow records + `bb_shadow.jsonl` under ONE
    /// single overall wall-clock deadline (`timeout`). Never blocks beyond `timeout` even if a
    /// comparator or disk sink is delayed.
    pub fn stop_and_flush_bounded(&self, timeout: Duration) {
        let deadline = Instant::now() + timeout;
        self.core.stop.store(true, Ordering::Release);
        {
            let mut flag = self.core.wake_mu.lock().unwrap_or_else(|e| e.into_inner());
            *flag = true;
            self.core.wake_cv.notify_all();
        }
        let worker_entry = self
            .worker
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        let mut worker_finished = true;
        if let Some((jh, worker_done)) = worker_entry {
            let mut g = worker_done.0.lock().unwrap_or_else(|e| e.into_inner());
            while !*g {
                let rem = deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    break;
                }
                let (next, res) = worker_done
                    .1
                    .wait_timeout(g, rem)
                    .unwrap_or_else(|e| e.into_inner());
                g = next;
                if res.timed_out() {
                    break;
                }
            }
            worker_finished = *g;
            drop(g);
            if worker_finished {
                let _ = jh.join();
            } else {
                self.worker_timed_out.store(true, Ordering::Release);
            }
        }
        if worker_finished && !self.worker_timed_out.load(Ordering::Acquire) {
            self.core.try_drain_and_publish();
        }
        let done_opt = if let Ok(mut c) = self.core.consumer.try_lock() {
            c.bb_queue.take().map(|q| q.done)
        } else {
            None
        };
        if let Some(done) = done_opt {
            let mut g = done.0.lock().unwrap_or_else(|e| e.into_inner());
            while !*g {
                let rem = deadline.saturating_duration_since(Instant::now());
                if rem.is_zero() {
                    break;
                }
                let (next, res) = done
                    .1
                    .wait_timeout(g, rem)
                    .unwrap_or_else(|e| e.into_inner());
                g = next;
                if res.timed_out() {
                    break;
                }
            }
        }
    }

    /// Request bounded stop of the background worker (never blocks on a slow disk sink).
    pub fn stop_worker(&self) {
        self.stop_and_flush_bounded(Duration::ZERO);
    }
}

impl Drop for TorqueShadowHub {
    fn drop(&mut self) {
        self.stop_and_flush_bounded(Duration::ZERO);
    }
}

// =========================================================================
// Unit Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::MockClock;

    #[test]
    fn resolve_shadow_enabled_precedence_and_spellings() {
        assert!(resolve_shadow_enabled(true, None).unwrap());
        assert!(!resolve_shadow_enabled(false, None).unwrap());
        assert!(resolve_shadow_enabled(true, Some("")).unwrap());
        for s in ["1", "true", "TRUE", "on", "On", "yes", " YES "] {
            assert!(resolve_shadow_enabled(false, Some(s)).unwrap(), "{s}");
        }
        for s in ["0", "false", "FALSE", "off", "Off", "no", " NO "] {
            assert!(!resolve_shadow_enabled(true, Some(s)).unwrap(), "{s}");
        }
        assert!(resolve_shadow_enabled(true, Some("maybe")).is_err());
    }

    #[test]
    fn clock_normalization_handles_pre_origin_sub_us_future_and_settle() {
        let mc = Arc::new(MockClock::new());
        let t_pre = mc.now();
        mc.advance(Duration::from_millis(50));
        let mut sc = ShadowClock::new(Some(mc.clone() as Arc<dyn DriverClock>));

        // Valid pre-origin timestamp stays strictly below origin bias (no saturation to 0).
        let pre_us = sc.instant_to_us(t_pre);
        assert_eq!(pre_us, SHADOW_ORIGIN_BIAS_US - 50_000);
        assert!(pre_us < SHADOW_ORIGIN_BIAS_US);

        // Sub-microsecond future RX (1 ns ahead of decode_now) must map strictly > now_us.
        let decode_now = mc.now();
        let future_1ns = decode_now + Duration::from_nanos(1);
        let (rx_us, now_us) = sc.normalize_valid_rx_us(future_1ns, decode_now, None, false);
        assert!(rx_us > now_us, "1ns future must stay > now_us: rx={rx_us} now={now_us}");

        // Sub-microsecond strict > 20ms settle boundary: write_done at +0.2us, rx at +20ms+0.8us
        // floors to exact +20000us unless preserved by live_post_settle.
        let mw = mc.now() + Duration::from_nanos(200);
        let mw_us = sc.instant_to_us(mw);
        let rx_past_20ms = mw + Duration::from_millis(20) + Duration::from_nanos(600);
        let decode_at = rx_past_20ms + Duration::from_nanos(100);
        let (rx_us, now_us) =
            sc.normalize_valid_rx_us(rx_past_20ms, decode_at, Some(mw_us), true);
        assert!(
            rx_us > mw_us + REST_SETTLE_US,
            "strict >20ms ns settle must survive µs conversion: rx={rx_us} mw={mw_us}"
        );
        assert!(rx_us <= now_us);
    }

    #[test]
    fn overflow_cutoffs_credit_pre_loss_backlog_and_latch_terminal_loss() {
        let cfg = ArmConfig::rebot_dm();
        let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);

        // Push 5 clean releases -> all verifiable and credited.
        for r in 1..=5u64 {
            rt.enqueue(ShadowInputKind::Release {
                release_seq: r,
                live_intent: TorqueIntent::Off,
                live_is_torqued: false,
                live_stopped: false,
            });
        }
        rt.flush_and_drain_sync();
        let snap1 = hub.snapshot();
        assert_eq!(snap1.events_processed, 5);
        assert!(!snap1.ring.interval_unverifiable);
        assert_eq!(snap1.ring.first_unverifiable_seq, None);

        // Enqueue 2 valid items, then trigger a terminal staging drop with no subsequent flush records.
        rt.enqueue(ShadowInputKind::Release {
            release_seq: 6,
            live_intent: TorqueIntent::Off,
            live_is_torqued: false,
            live_stopped: false,
        });
        rt.enqueue(ShadowInputKind::Release {
            release_seq: 7,
            live_intent: TorqueIntent::Off,
            live_is_torqued: false,
            live_stopped: false,
        });
        // Simulate driver staging overflow of 3 dropped obs right after seq 7.
        rt.drain_driver_staging_into_fifo(|_scratch| (0, 3));
        rt.flush_and_drain_sync();

        let snap2 = hub.snapshot();
        assert_eq!(snap2.events_processed, 7);
        assert!(snap2.ring.interval_unverifiable);
        assert_eq!(snap2.ring.first_unverifiable_seq, Some(8));
        assert_eq!(snap2.ring.staging_dropped_records, 3);
        assert_eq!(snap2.ring.staging_overflow_events, 1);

        // Subsequent clean records NEVER clear the latched unverifiable status.
        rt.enqueue(ShadowInputKind::Release {
            release_seq: 8,
            live_intent: TorqueIntent::Off,
            live_is_torqued: false,
            live_stopped: false,
        });
        rt.flush_and_drain_sync();
        let snap3 = hub.snapshot();
        assert!(snap3.ring.interval_unverifiable);
        assert_eq!(snap3.ring.first_unverifiable_seq, Some(8));
    }

    #[test]
    fn ring_saturation_marks_unverifiable_and_writes_blackbox() {
        struct MemSink(Arc<Mutex<Vec<String>>>);
        impl ShadowBlackBoxSink for MemSink {
            fn write_shadow_line(&mut self, line: &[u8]) -> io::Result<()> {
                self.0
                    .lock()
                    .unwrap()
                    .push(String::from_utf8_lossy(line).into_owned());
                Ok(())
            }
        }

        let cfg = ArmConfig::rebot_dm();
        let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);
        let lines = Arc::new(Mutex::new(Vec::new()));
        hub.attach_blackbox_sink(Box::new(MemSink(lines.clone())), 1, Instant::now());

        // Overfill SPSC ring (capacity 1024) without draining!
        for r in 1..=(SHADOW_RING_CAPACITY as u64 + 20) {
            rt.enqueue(ShadowInputKind::Release {
                release_seq: r,
                live_intent: TorqueIntent::Off,
                live_is_torqued: false,
                live_stopped: false,
            });
            rt.flush_deferred_steps();
        }
        // Even before draining, atomic watermark & blackbox compact reflect loss!
        assert!(hub.overflow_atomics().blackbox_compact().is_some_and(|c| c.u && c.drop == 20));
        hub.drain_now();
        let snap = hub.snapshot();
        assert!(snap.ring.interval_unverifiable);
        assert_eq!(snap.ring.ring_dropped_records, 20);
        assert_eq!(snap.ring.first_unverifiable_seq, Some(SHADOW_RING_CAPACITY as u64 + 1));
        assert!(
            lines.lock().unwrap().iter().any(|l| l.contains("\"event\":\"overflow\"")),
            "expected overflow blackbox line: {:?}",
            lines.lock().unwrap()
        );
    }

    #[test]
    fn fault_injection_is_counted_classified_and_logged() {
        struct MemSink(Arc<Mutex<Vec<String>>>);
        impl ShadowBlackBoxSink for MemSink {
            fn write_shadow_line(&mut self, line: &[u8]) -> io::Result<()> {
                self.0
                    .lock()
                    .unwrap()
                    .push(String::from_utf8_lossy(line).into_owned());
                Ok(())
            }
        }

        let cfg = ArmConfig::rebot_dm();
        let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);
        let lines = Arc::new(Mutex::new(Vec::new()));
        hub.attach_blackbox_sink(Box::new(MemSink(lines.clone())), 2, Instant::now());

        rt.set_injected_fault(ShadowFaultInjection::InjectSpuriousDisable {
            recipients: ChannelMask::from_count(7),
        });
        rt.enqueue(ShadowInputKind::Release {
            release_seq: 1,
            live_intent: TorqueIntent::Off,
            live_is_torqued: false,
            live_stopped: false,
        });
        rt.flush_and_drain_sync();

        let snap = hub.snapshot();
        assert_eq!(snap.taxonomy.extra_action, 1);
        assert_eq!(snap.unexplained_divergences_total, 1);
        assert_eq!(snap.recent_divergences.len(), 1);
        assert_eq!(
            snap.recent_divergences[0].category,
            DivergenceCategory::ExtraAction
        );
        assert!(lines
            .lock()
            .unwrap()
            .iter()
            .any(|l| l.contains("\"event\":\"divergence\"") && l.contains("extra_action")));
        assert!(hub.overflow_atomics().blackbox_compact().is_some_and(|c| c.d == 1));
    }

    #[test]
    fn grouped_hand_topology_runs_through_shadow_on_both_owners() {
        for owner in [ShadowOwner::Legacy, ShadowOwner::Executor] {
            let tc = TorqueConfig::arm_plus_underactuated_hand();
            let (mut rt, hub) =
                ShadowRuntime::from_torque_config(tc, true, VendorKind::GenericGrouped, None);
            rt.set_owner(owner);
            let op = rt.alloc_live_op_id();
            rt.enqueue(ShadowInputKind::Release {
                release_seq: 1,
                live_intent: TorqueIntent::Off,
                live_is_torqued: false,
                live_stopped: false,
            });
            rt.enqueue(ShadowInputKind::EnableStart {
                live_op_id: op,
                live_refusal: None,
                live_intent: TorqueIntent::On,
                live_is_torqued: true,
                live_stopped: false,
            });
            rt.flush_and_drain_sync();
            let snap = hub.snapshot();
            assert_eq!(snap.owner, owner);
            assert_eq!(snap.taxonomy.invariant_violation, 0);
            assert_eq!(snap.unexplained_divergences_total, 0);
            assert!(!snap.ring.interval_unverifiable);
        }
    }
}
