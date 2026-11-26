//! Torque-saturation guard: a joint whose measured |torque| stays at or above `ratio` x the
//! motor's torque mapping range (TMAX) for `persist_ms` stops the motion and holds the
//! measured pose with torque on (no Fault).
//!
//! Motivation (real arm, 2026-11-12, black box at 5 Hz): J4 (DM4310, TMAX 10 N·m) stopped at
//! -1.60 rad against a physical stop while the wrist was driven down; its torque went
//! -2.0 -> -6.8 -> +8.8 -> +9.5 N·m in 0.4 s with no position change, and ~0.4 s later all
//! seven motors stopped replying and came back Disabled (power reset). The servo blocked hold
//! (< 0.01 rad progress for 500 ms at the lead cap) had not triggered by then.
//!
//! Watched: J1..J6 listed in `joints`, while torqued in position control with fresh torque
//! feedback. Not watched: the gripper (its own stall handling; gripping is a normal stall),
//! gravity compensation (it clamps its own feed-forward torques), the unload settle phase
//! (the arm rests on its supports and torque is cut at its end).
//!
//! Per tick the guard costs O(6) and allocates nothing; strings are built only on a trip.

use serde::{Deserialize, Serialize};

use crate::model::ArmConfig;
#[cfg(not(target_arch = "wasm32"))]
use crate::model::MotionLimitReason;
#[cfg(not(target_arch = "wasm32"))]
use crate::supervisor::SupervisorInner;

/// A torque hold ends once the held joint's measured position is at least this far (rad)
/// from the hold position and its |torque| is below the threshold.
pub const TORQUE_HOLD_RELEASE_RAD: f32 = 0.05;

/// Retreat allowance: after a trip, a motion that moves the held joint away from the load
/// (opposite to the hold torque sign) is not tripped on that joint for at most this long
/// (control-loop time, s). 1.5 s is the shortest planned park (`NormalUnload` plans with a
/// 1.5 s minimum); in the elastic-obstacle reproduction the torque fell below the threshold
/// 0.18 s into the park. After it the normal rule applies again.
pub const TORQUE_RETREAT_WINDOW_S: f64 = 1.5;
/// Retreat allowance: |torque| of the held joint may not exceed max(|torque at the trip|,
/// threshold) + this fraction of the threshold (1.68 N·m on the B601-DM J1-J3, 0.6 N·m on
/// J4-J6). A load that relaxes while the joint backs out does not rise; a rise by a tenth of
/// the threshold means the motion is loading the joint further.
pub const TORQUE_RETREAT_MARGIN_RATIO: f32 = 0.10;
/// Retreat allowance: the measured position of the held joint must follow the commanded
/// retreat within this lag (rad) beyond the lag at the start of the retreat. A joint that is
/// wedged and does not come back loses the allowance after 10 mrad of commanded retreat.
pub const TORQUE_RETREAT_LAG_RAD: f32 = 0.01;
/// Retreat allowance: the commanded position may turn back toward the load by at most this
/// much (rad) from the furthest retreat reached; a path that turns around loses the allowance.
pub const TORQUE_RETREAT_TURN_RAD: f32 = 0.001;
/// While a hold is in force, a commanded position more than this far (rad) past the trip
/// position in the push direction trips at once, without `persist_ms` (a planned path, park
/// or Cartesian move that pushes into the load again). At the 650 N·m/rad contact of the
/// reproduction 2 mrad is 1.2 N·m.
pub const TORQUE_PUSH_IN_RAD: f32 = 0.002;
/// After a trip the held joint is commanded this far (rad) back from the measured trip
/// position, in the relieving direction, so that it does not keep pressing at the trip
/// torque. Bounded: a repeated trip on the same joint and sign never backs off further than
/// this from the first trip position, and never moves a joint that was pushed back further
/// than this toward the load. 5 mrad is 1.5 mm at 0.3 m from J2; position control stays
/// on, so it is a bounded position step, not a release (the trip itself steps the command
/// back by the tracking lag, 65 mrad in the 2026-11-21 incident).
pub const TORQUE_HOLD_BACKOFF_RAD: f32 = 0.005;

/// Accepted range of `torque_guard.ratio`.
pub const MIN_RATIO: f32 = 0.3;
pub const MAX_RATIO: f32 = 0.95;

/// Profile section `torque_guard`. Absent: guard off. Missing fields take the defaults below
/// (ratio 0.85: the value intended with the recommended 24 V 14.6 A supply; the B601-DM
/// profile sets 0.60 for the 6.5 A supply in use, see `ArmConfig::rebot_dm`).
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct TorqueGuardPolicy {
    pub enabled: bool,
    /// Fraction of the joint's TMAX (`damiao_ranges[j].t_max`) that counts as saturated.
    pub ratio: f32,
    /// Time |torque| must stay at or above the threshold (control-loop time, ms).
    pub persist_ms: u64,
    /// J1..J6 watched. The gripper joint is never watched.
    pub joints: [bool; 6],
}

impl Default for TorqueGuardPolicy {
    fn default() -> Self {
        Self { enabled: true, ratio: 0.85, persist_ms: 100, joints: [true; 6] }
    }
}

impl TorqueGuardPolicy {
    pub fn validate(&self) -> Result<(), String> {
        if !self.ratio.is_finite() || !(MIN_RATIO..=MAX_RATIO).contains(&self.ratio) {
            return Err(format!("torque_guard.ratio must be in [{MIN_RATIO}, {MAX_RATIO}]"));
        }
        if self.persist_ms == 0 || self.persist_ms > 2000 {
            return Err("torque_guard.persist_ms must be in 1..=2000".into());
        }
        Ok(())
    }

    /// Threshold (N·m) per joint of `config`; None = not watched.
    pub fn thresholds(&self, config: &ArmConfig) -> Vec<Option<f32>> {
        (0..config.dof).map(|j| self.threshold_of(config, j)).collect()
    }

    /// The threshold of joint `j` ([`Self::thresholds`] without the vector).
    pub fn threshold_of(&self, config: &ArmConfig, j: usize) -> Option<f32> {
        if !self.enabled || j >= 6 || !self.joints[j] || config.gripper_index() == Some(j) { return None; }
        config.damiao_ranges.as_ref().and_then(|r| r.get(j)).map(|r| self.ratio * r.t_max)
    }
}

/// `schema.torque_guard`: the policy in force.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TorqueGuardInfo {
    pub ratio: f32,
    pub persist_ms: u64,
    /// Per joint: |torque| threshold (N·m), null = not watched.
    pub thresholds_nm: Vec<Option<f32>>,
    /// Back-out distance that ends a hold (rad).
    pub release_rad: f32,
}

impl TorqueGuardInfo {
    /// None when the profile has no enabled guard or no joint has a threshold.
    pub fn from_config(config: &ArmConfig) -> Option<Self> {
        let p = config.torque_guard.filter(|p| p.enabled)?;
        let thresholds_nm = p.thresholds(config);
        thresholds_nm.iter().any(Option::is_some).then_some(Self {
            ratio: p.ratio,
            persist_ms: p.persist_ms,
            thresholds_nm,
            release_rad: TORQUE_HOLD_RELEASE_RAD,
        })
    }
}

/// The hold in force after a trip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueHold {
    pub joint: usize,
    /// Commanded position of the joint at the trip (measured, clamped to the limits).
    pub hold_q: f32,
    /// Sign of the measured torque at the trip: the push direction.
    pub sign: f32,
    pub torque_nm: f32,
    pub threshold_nm: f32,
    /// Observation-clock time of the trip.
    pub since_s: f64,
    /// Measured position of the joint at the first trip of this hold (same joint and sign);
    /// reference for the push-in check and the back-off bound.
    pub anchor_q: f32,
    /// This hold was entered because a motion pushed past the trip position.
    pub push_in: bool,
    /// Retreat allowance of this hold.
    pub retreat: Retreat,
}

/// Retreat allowance state of a hold (see [`TORQUE_RETREAT_WINDOW_S`]).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Retreat {
    /// No retreat motion seen yet.
    #[default]
    Unused,
    /// A retreat motion runs; values are in the relief coordinate (rad, negative = away
    /// from the load, relative to `hold_q`).
    /// `elapsed_s` counts control-loop time.
    Active { elapsed_s: f64, best_cmd: f32, lag0: f32 },
    /// The allowance ended (time bound, torque rise, lag, turn-around or motion end).
    Spent,
}

/// Number of recent torque-guard back-off records kept in fixed, zero-allocation storage.
pub const TORQUE_BACKOFF_DIAG_CAPACITY: usize = 8;
/// Number of fresh post-trip feedback ticks sampled to measure post-trip position & torque.
pub const TORQUE_BACKOFF_POST_SAMPLES: u16 = 5;
/// Bounded post-trip capture window (s) after which the record closes even if fewer samples arrived.
pub const TORQUE_BACKOFF_CAPTURE_WINDOW_S: f64 = 0.5;

/// Bounded per-trip back-off measurement record captured in fixed storage on the RT thread
/// and published off-thread via [`crate::rt_core::publish::StateHub`].
///
/// Coordinate system: `anchor_q`, `measured_at_trip_q`, `requested_q`, `transmitted_q`, and
/// `post_trip_measured_q` are all in user-space radians (zero-offset removed; `transmitted_q`
/// reflects the actual wire-quantized frame committed at the CAN flush boundary).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TorqueBackoffRecord {
    pub trip_seq: u64,
    pub joint: u8,
    pub since_s: f64,
    pub sign: f32,
    pub torque_at_trip_nm: f32,
    pub threshold_nm: f32,
    pub anchor_q: f32,
    pub measured_at_trip_q: f32,
    /// Backed-off target position requested by `torque_trip` (user-space rad, clamped to limits).
    pub requested_q: f32,
    /// Actual wire-quantized position (converted back to user-space rad) committed at the CAN
    /// flush boundary for the tripped joint on the first non-skipped control frame after the trip.
    pub transmitted_q: Option<f32>,
    /// Post-trip measured position (user-space rad) observed on new post-TX feedback frames.
    pub post_trip_measured_q: Option<f32>,
    /// Post-trip measured torque (N·m) observed on new post-TX feedback frames.
    pub post_trip_torque_nm: Option<f32>,
    /// Count of distinct fresh post-TX feedback frames observed (capped at [`TORQUE_BACKOFF_POST_SAMPLES`]).
    pub post_trip_samples: u16,
    pub push_in: bool,
    /// Driver wire-commit sequence snapshot at the moment of `torque_trip` (gates `transmitted_q`).
    #[serde(skip)]
    pub(crate) tx_seq_at_trip: u64,
    /// Pre-TX driver clock instant at the first confirmed post-trip wire commit on `joint`;
    /// gates `record_post_trip_feedback` so pre-TX queued frames decoded after TX are ignored.
    #[serde(skip)]
    pub(crate) transmitted_at: Option<std::time::Instant>,
    /// Last per-axis feedback sequence counted into `post_trip_*` (prevents duplicate cached reads).
    #[serde(skip)]
    pub(crate) last_post_fb_seq: u64,
    /// True once the post-trip capture window closed (sample cap, window expiry, or disarm/stop).
    #[serde(skip)]
    pub(crate) capture_closed: bool,
}

impl TorqueBackoffRecord {
    /// Construct a fresh trip record awaiting wire TX confirmation and post-trip feedback samples.
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        trip_seq: u64,
        joint: u8,
        since_s: f64,
        sign: f32,
        torque_at_trip_nm: f32,
        threshold_nm: f32,
        anchor_q: f32,
        measured_at_trip_q: f32,
        requested_q: f32,
        push_in: bool,
    ) -> Self {
        Self {
            trip_seq,
            joint,
            since_s,
            sign,
            torque_at_trip_nm,
            threshold_nm,
            anchor_q,
            measured_at_trip_q,
            requested_q,
            transmitted_q: None,
            post_trip_measured_q: None,
            post_trip_torque_nm: None,
            post_trip_samples: 0,
            push_in,
            tx_seq_at_trip: 0,
            transmitted_at: None,
            last_post_fb_seq: 0,
            capture_closed: false,
        }
    }
}

/// Fixed-capacity, `Copy`, zero-allocation ring buffer of recent [`TorqueBackoffRecord`]s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueBackoffHistory {
    records: [Option<TorqueBackoffRecord>; TORQUE_BACKOFF_DIAG_CAPACITY],
    head: usize,
    len: usize,
    pub trips_total: u64,
    /// Monotonic revision bumped whenever a trip or post-trip sample mutates the history,
    /// allowing `StateHub` publication to skip copying when unchanged (0 bytes copied on steady ticks).
    pub revision: u64,
}

impl Default for TorqueBackoffHistory {
    fn default() -> Self {
        Self {
            records: [None; TORQUE_BACKOFF_DIAG_CAPACITY],
            head: 0,
            len: 0,
            trips_total: 0,
            revision: 0,
        }
    }
}

impl TorqueBackoffHistory {
    #[inline]
    pub fn push(&mut self, record: TorqueBackoffRecord) {
        self.records[self.head] = Some(record);
        self.head = (self.head + 1) % TORQUE_BACKOFF_DIAG_CAPACITY;
        if self.len < TORQUE_BACKOFF_DIAG_CAPACITY {
            self.len += 1;
        }
        self.trips_total = record.trip_seq;
        self.revision = self.revision.wrapping_add(1).max(1);
    }

    #[inline]
    pub fn latest(&self) -> Option<&TorqueBackoffRecord> {
        if self.len == 0 {
            return None;
        }
        let idx = (self.head + TORQUE_BACKOFF_DIAG_CAPACITY - 1) % TORQUE_BACKOFF_DIAG_CAPACITY;
        self.records[idx].as_ref()
    }

    #[inline]
    fn latest_mut(&mut self) -> Option<&mut TorqueBackoffRecord> {
        if self.len == 0 {
            return None;
        }
        let idx = (self.head + TORQUE_BACKOFF_DIAG_CAPACITY - 1) % TORQUE_BACKOFF_DIAG_CAPACITY;
        self.records[idx].as_mut()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Closes any open post-trip capture record (e.g. on disarm, fault, or non-operational state)
    /// so a later rearm cannot resume writing samples into an older trip record.
    #[inline]
    pub fn close_open_capture(&mut self) {
        if let Some(rec) = self.latest_mut() {
            if !rec.capture_closed {
                rec.capture_closed = true;
            }
        }
    }

    /// Records the first wire-committed position for the most recent trip on `joint`, gated by
    /// `wire_tx_seq > rec.tx_seq_at_trip` so skipped/mode-uncertain axes never fabricate TX.
    #[inline]
    pub fn record_transmitted(
        &mut self,
        joint: usize,
        wire_tx_seq: u64,
        tx_at: Option<std::time::Instant>,
        transmitted_q: f32,
        post_fb_seq_baseline: u64,
    ) {
        if !transmitted_q.is_finite() {
            return;
        }
        let mut changed = false;
        if let Some(rec) = self.latest_mut() {
            if usize::from(rec.joint) == joint
                && !rec.capture_closed
                && rec.transmitted_q.is_none()
                && wire_tx_seq > rec.tx_seq_at_trip
            {
                rec.transmitted_q = Some(transmitted_q);
                rec.transmitted_at = tx_at;
                rec.last_post_fb_seq = rec.last_post_fb_seq.max(post_fb_seq_baseline);
                changed = true;
            }
        }
        if changed {
            self.revision = self.revision.wrapping_add(1).max(1);
        }
    }

    /// Updates post-trip measured position and torque for the most recent trip up to
    /// [`TORQUE_BACKOFF_POST_SAMPLES`] distinct post-TX feedback frames within
    /// [`TORQUE_BACKOFF_CAPTURE_WINDOW_S`]. Never counts cached feedback or pre-TX frames
    /// (`fb_rx_at < tx_at`).
    #[inline]
    pub fn record_post_trip_feedback(
        &mut self,
        joint: usize,
        fb_seq: u64,
        fb_rx_at: Option<std::time::Instant>,
        now_s: f64,
        measured_q: f32,
        torque_nm: f32,
    ) {
        let mut changed = false;
        if let Some(rec) = self.latest_mut() {
            if rec.capture_closed || usize::from(rec.joint) != joint {
                return;
            }
            if !now_s.is_finite() || now_s - rec.since_s > TORQUE_BACKOFF_CAPTURE_WINDOW_S {
                rec.capture_closed = true;
                return;
            }
            if rec.transmitted_q.is_none() || fb_seq <= rec.last_post_fb_seq {
                return;
            }
            if let Some(tx_at) = rec.transmitted_at {
                if fb_rx_at.is_none_or(|rx| rx < tx_at) {
                    return;
                }
            }
            if rec.post_trip_samples < TORQUE_BACKOFF_POST_SAMPLES {
                if measured_q.is_finite() {
                    rec.post_trip_measured_q = Some(measured_q);
                }
                if torque_nm.is_finite() {
                    rec.post_trip_torque_nm = Some(torque_nm);
                }
                rec.post_trip_samples = rec.post_trip_samples.saturating_add(1);
                rec.last_post_fb_seq = fb_seq;
                if rec.post_trip_samples >= TORQUE_BACKOFF_POST_SAMPLES {
                    rec.capture_closed = true;
                }
                changed = true;
            }
        }
        if changed {
            self.revision = self.revision.wrapping_add(1).max(1);
        }
    }

    /// Off-thread snapshot for diagnostics JSON publication.
    pub fn to_diagnostics(&self) -> TorqueBackoffDiagnostics {
        let mut recent = Vec::with_capacity(self.len);
        let start = if self.len == TORQUE_BACKOFF_DIAG_CAPACITY {
            self.head
        } else {
            0
        };
        for offset in 0..self.len {
            let idx = (start + offset) % TORQUE_BACKOFF_DIAG_CAPACITY;
            if let Some(rec) = self.records[idx] {
                recent.push(rec);
            }
        }
        TorqueBackoffDiagnostics {
            backoff_rad: TORQUE_HOLD_BACKOFF_RAD,
            trips_total: self.trips_total,
            latest: self.latest().copied(),
            recent,
        }
    }
}

/// Off-thread serializable view of [`TorqueBackoffHistory`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TorqueBackoffDiagnostics {
    pub backoff_rad: f32,
    pub trips_total: u64,
    pub latest: Option<TorqueBackoffRecord>,
    pub recent: Vec<TorqueBackoffRecord>,
}

impl TorqueHold {
    /// Relief coordinate of `q`: (q - hold_q) x sign; negative = away from the load.
    pub fn relief(&self, q: f32) -> f32 { (q - self.hold_q) * self.sign }

    /// Hold command of the held joint after a trip at measured position `q_meas`:
    /// [`TORQUE_HOLD_BACKOFF_RAD`] back from it in the relieving direction, never further
    /// than that from `anchor_q`, and never toward the load from `q_meas`.
    pub fn backed_off(&self, q_meas: f32) -> f32 {
        let r = (q_meas - self.anchor_q) * self.sign;
        let r_hold = (r - TORQUE_HOLD_BACKOFF_RAD).max(-TORQUE_HOLD_BACKOFF_RAD).min(r);
        self.anchor_q + r_hold * self.sign
    }
}

/// Runtime guard state kept in the supervisor.
#[derive(Debug, Clone, Default)]
pub struct TorqueGuard {
    /// Continuous time at or above the threshold (s), J1..J6.
    over_s: [f64; 6],
    pub hold: Option<TorqueHold>,
    /// Trips since start (logs).
    pub trips_total: u64,
    /// Bounded fixed-storage back-off diagnostics captured at trip/TX/post-trip ticks (0 alloc).
    pub backoff_history: TorqueBackoffHistory,
}

impl TorqueGuard {
    /// Deterministic saturation transition. Returns the joint that newly entered a hold.
    /// The gripper is excluded by limiting the input to the first six joints.
    /// `q_cmd` is this tick's commanded position (before the reference is stepped);
    /// `motion_target` the target of the running motion, None when none runs.
    #[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, torques_nm: &[f32], thresholds_nm: &[Option<f32>], dt_s: f64, now_s: f64,
        policy: &TorqueGuardPolicy, operational: bool, watching: bool, q_meas: &[f32],
        q_cmd: &[f32], motion_target: Option<&[f32]>) -> Option<usize> {
        let previous = self.hold;
        if let Some(h) = self.hold {
            let q = q_meas.get(h.joint).copied().unwrap_or(h.hold_q);
            let tau = torques_nm.get(h.joint).copied().unwrap_or(f32::NAN);
            if !operational || ((q - h.hold_q).abs() >= TORQUE_HOLD_RELEASE_RAD && tau.abs() < h.threshold_nm) {
                self.hold = None;
            }
        }
        if !operational || !policy.enabled || !watching { self.over_s = [0.0; 6]; return None; }
        // Held joint: push-in check and retreat allowance.
        let mut exempt = None;
        if let Some(h) = self.hold.as_mut() {
            let j = h.joint;
            let cmd = q_cmd.get(j).copied().unwrap_or(h.hold_q);
            if (cmd - h.anchor_q) * h.sign > TORQUE_PUSH_IN_RAD {
                let tau = torques_nm.get(j).copied().unwrap_or(f32::NAN);
                self.over_s = [0.0; 6];
                self.trips_total += 1;
                let anchor_q = h.anchor_q;
                self.hold = Some(TorqueHold { joint: j, hold_q: q_meas.get(j).copied().unwrap_or(cmd), sign: h.sign,
                    torque_nm: if tau.is_finite() { tau } else { h.torque_nm }, threshold_nm: h.threshold_nm, since_s: now_s,
                    anchor_q, push_in: true, retreat: Retreat::Unused });
                return Some(j);
            }
            if Self::retreat_allowed(h, torques_nm, dt_s, q_meas, cmd, motion_target) { exempt = Some(j); }
        }
        let persist = policy.persist_ms as f64 / 1000.0 - 1e-6;
        let mut worst = None;
        for j in 0..6 {
            let Some(threshold) = thresholds_nm.get(j).copied().flatten() else { self.over_s[j] = 0.0; continue; };
            let tau = torques_nm.get(j).copied().unwrap_or(f32::NAN);
            if tau.is_finite() && tau.abs() >= threshold {
                self.over_s[j] += dt_s;
                let ratio = tau.abs() / threshold;
                if exempt != Some(j) && self.over_s[j] >= persist && worst.is_none_or(|(_, r): (usize, f32)| ratio > r) { worst = Some((j, ratio)); }
            } else { self.over_s[j] = 0.0; }
        }
        if let Some((j, _)) = worst {
            let threshold = thresholds_nm[j].unwrap_or(f32::INFINITY);
            let tau = torques_nm[j];
            let sign = if tau >= 0.0 { 1.0 } else { -1.0 };
            let q = q_meas.get(j).copied().unwrap_or(0.0);
            // a repeated trip on the same joint and sign keeps the first trip position
            let anchor_q = previous.filter(|p| p.joint == j && p.sign == sign).map_or(q, |p| p.anchor_q);
            self.over_s = [0.0; 6];
            self.trips_total += 1;
            self.hold = Some(TorqueHold { joint: j, hold_q: q, sign, torque_nm: tau, threshold_nm: threshold, since_s: now_s,
                anchor_q, push_in: false, retreat: Retreat::Unused });
            return Some(j);
        }
        None
    }

    /// Advance the retreat allowance of `h`; true while the held joint is exempt from a
    /// re-trip this tick.
    fn retreat_allowed(h: &mut TorqueHold, torques_nm: &[f32], dt_s: f64, q_meas: &[f32], cmd: f32,
        motion_target: Option<&[f32]>) -> bool {
        const EPS: f32 = 1e-6;
        let j = h.joint;
        let r_cmd = h.relief(cmd);
        let r_meas = q_meas.get(j).map_or(f32::NAN, |q| h.relief(*q));
        let target_relief = motion_target.and_then(|t| t.get(j)).map(|t| h.relief(*t));
        match h.retreat {
            Retreat::Spent => false,
            Retreat::Unused => {
                // a running motion that moves the held joint away from the load
                let retreating = target_relief.is_some_and(|t| t < -EPS) && r_cmd <= EPS;
                if !retreating || !r_meas.is_finite() { return false; }
                h.retreat = Retreat::Active { elapsed_s: 0.0, best_cmd: r_cmd.min(0.0), lag0: r_meas - r_cmd };
                true
            }
            Retreat::Active { elapsed_s, best_cmd, lag0 } => {
                let elapsed_s = elapsed_s + dt_s;
                let tau = torques_nm.get(j).copied().unwrap_or(f32::NAN);
                let limit = h.torque_nm.abs().max(h.threshold_nm) + TORQUE_RETREAT_MARGIN_RATIO * h.threshold_nm;
                let ok = motion_target.is_some()
                    && elapsed_s <= TORQUE_RETREAT_WINDOW_S + 1e-6
                    && tau.is_finite() && tau.abs() <= limit
                    && r_cmd <= best_cmd + TORQUE_RETREAT_TURN_RAD
                    && r_meas.is_finite() && r_meas - r_cmd <= lag0 + TORQUE_RETREAT_LAG_RAD;
                h.retreat = if ok { Retreat::Active { elapsed_s, best_cmd: best_cmd.min(r_cmd), lag0 } } else { Retreat::Spent };
                ok
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl SupervisorInner {
    /// Per-joint thresholds (N·m) from the profile; empty / all None when the guard is off.
    pub fn torque_guard_thresholds(&self) -> Vec<Option<f32>> {
        self.config.torque_guard.map(|p| p.thresholds(&self.config)).unwrap_or_default()
    }

    /// One control tick, after this tick's feedback and before the motion reference is
    /// stepped. `dt` is the control-loop step (s).
    #[doc(hidden)]
    pub fn torque_guard_tick(&mut self, dt: f64) {
        let Some(p) = self.config.torque_guard.filter(|p| p.enabled) else { return; };
        let operational = self.is_torqued && self.safety.state().is_operational() && self.gravity_comp.is_none()
            && !self.disable_pending && !self.pending_rearm;
        // on the stack: no allocation in the tick (joints beyond the array are never watched)
        let mut thr_buf = [None; 16];
        let n = self.config.dof.min(thr_buf.len());
        for (j, t) in thr_buf.iter_mut().enumerate().take(n) { *t = p.threshold_of(&self.config, j); }
        let thr = &thr_buf[..n];
        let settling = self.pending_normal_unload && self.active_trajectory.is_none();
        let watching = operational && !settling && self.driver.is_some() && self.collision.torque_feedback
            && self.feedback_numeric_valid && self.hardware_feedback_fresh(std::time::Duration::from_millis(100));
        let now = self.start_time.elapsed().as_secs_f64();
        if !operational {
            self.torque_guard.backoff_history.close_open_capture();
        } else if watching {
            if let Some(j) = self
                .torque_guard
                .backoff_history
                .latest()
                .filter(|rec| !rec.capture_closed && rec.post_trip_samples < TORQUE_BACKOFF_POST_SAMPLES)
                .map(|rec| usize::from(rec.joint))
            {
                let fb_seq = self.core.feedback().seqs().get(j).copied().unwrap_or(0);
                let fb_rx_at = self.core.feedback().instants().get(j).copied().flatten();
                if let (Some(&q_meas), Some(&tau)) =
                    (self.measured_positions.get(j), self.torques.get(j))
                {
                    self.torque_guard
                        .backoff_history
                        .record_post_trip_feedback(j, fb_seq, fb_rx_at, now, q_meas, tau);
                }
            }
        }
        let previous_hold = self.torque_guard.hold;
        let had_hold = previous_hold.is_some();
        let motion_target = if let Some(t) = self.servo_target.as_deref() { Some(t) }
            else if self.active_trajectory.is_some() { Some(&self.target_positions[..]) } else { None };
        if let Some(j) = self.torque_guard.step(&self.torques, thr, dt, now, &p, operational, watching, &self.measured_positions,
            &self.commanded_positions, motion_target) {
            let threshold = thr[j].unwrap_or(f32::INFINITY);
            self.torque_trip(j, self.torques[j], threshold, &p, previous_hold.is_some_and(|h| h.joint == j));
        }
        if had_hold && self.torque_guard.hold.is_none()
            && self.motion_limit.as_ref().is_some_and(|l| l.reason == MotionLimitReason::Torque) {
            self.motion_limit = None;
        }
    }

    /// Trip on joint `j`: fail the running action with `TORQUE_LIMIT`, end every motion and
    /// hold the measured pose with torque on (gripper keeps its command), the held joint
    /// backed off by [`TORQUE_HOLD_BACKOFF_RAD`] in the relieving direction.
    /// Zero heap allocations on the control thread (B8).
    fn torque_trip(&mut self, j: usize, torque: f32, threshold: f32, p: &TorqueGuardPolicy, again: bool) {
        let tmax = threshold / p.ratio;
        let push_in = self.torque_guard.hold.is_some_and(|h| h.push_in);
        let past = self.torque_guard.hold.map_or(0.0, |h| (self.commanded_positions[j] - h.anchor_q) * h.sign);
        let running = self.last_action.as_ref().filter(|a| a.is_running()).map(|a| a.id);
        let write_trip_msg = |buf: &mut dyn core::fmt::Write| {
            if push_in {
                let _ = write!(
                    buf,
                    "Torque limit on joint {}: the motion pushes {past:.3} rad past the position of the torque trip, into the load ({torque:.2} N·m, threshold {threshold:.2} N·m)",
                    j + 1
                );
            } else {
                let _ = write!(
                    buf,
                    "Torque limit on joint {}: {torque:.2} N·m at or above {threshold:.2} N·m ({:.0}% of {tmax:.1} N·m) for {} ms",
                    j + 1,
                    p.ratio * 100.0,
                    p.persist_ms
                );
            }
        };
        let action_msg = running.map(|id| {
            let mut msg = self.take_action_msg_buf(id, 256);
            msg.clear();
            write_trip_msg(&mut msg);
            msg
        });
        if !self.safety.deferred_logging() {
            let mut log_buf = crate::safety::BoundedText::<256>::empty();
            let text = match action_msg.as_deref() {
                Some(s) => s,
                None => {
                    write_trip_msg(&mut log_buf);
                    log_buf.as_str()
                }
            };
            if again && !push_in && running.is_none() && self.watchdog_park.is_none() {
                tracing::debug!("{text}; holding the new measured position");
            } else {
                tracing::warn!(
                    "{text} (cmd {:.3} meas {:.3} rad); motion stopped, holding position",
                    self.commanded_positions[j],
                    self.measured_positions[j]
                );
            }
        }
        if let Some(id) = running {
            self.finish_action(id, "failed", Some("TORQUE_LIMIT"), action_msg, None, None, None);
        }
        self.drop_macro_run("failed", Some("TORQUE_LIMIT"));
        // M1 + B8: preserve previous tracking state across `hold_measured_pose` in place
        // (0 heap allocs, 0 moves, 0 Vec clones) so active lead ramps smoothly in the relieving
        // direction instead of dropping in one tick.
        let keep_tracking = self
            .config
            .joint_tracking
            .as_ref()
            .is_some_and(|jt| jt.enabled);
        self.hold_measured_pose_inner(keep_tracking);
        self.note_motion_limit(MotionLimitReason::Torque, Some(j));
        let tx_seq_at_trip = self
            .read_driver_for_tick()
            .and_then(|drv| drv.last_committed_wire_position(j))
            .map_or(0, |(seq, _, _)| seq);
        let fb_seq_at_trip = self.core.feedback().seqs().get(j).copied().unwrap_or(0);
        if let Some(h) = self.torque_guard.hold.as_mut() {
            let l = &self.config.limits[j];
            let q = self.measured_positions[j];
            if q.is_finite() {
                self.commanded_positions[j] = h.backed_off(q).clamp(l.min_position, l.max_position);
                self.target_positions[j] = self.commanded_positions[j];
            }
            h.hold_q = self.commanded_positions[j];
            let rec = TorqueBackoffRecord {
                trip_seq: self.torque_guard.trips_total,
                joint: j as u8,
                since_s: h.since_s,
                sign: h.sign,
                torque_at_trip_nm: torque,
                threshold_nm: threshold,
                anchor_q: h.anchor_q,
                measured_at_trip_q: q,
                requested_q: self.commanded_positions[j],
                transmitted_q: None,
                post_trip_measured_q: None,
                post_trip_torque_nm: None,
                post_trip_samples: 0,
                push_in,
                tx_seq_at_trip,
                transmitted_at: None,
                last_post_fb_seq: fb_seq_at_trip,
                capture_closed: false,
            };
            self.torque_guard.backoff_history.push(rec);
        }
        // A watchdog park stopped by the guard: the park path loads the held joint (it pushes
        // into the load, or the retreat allowance ended). Hold torque at the backed-off pose
        // and latch WatchdogTimeout, naming the cause (torque off would drop the arm).
        if self.watchdog_park.is_some() {
            if !self.safety.deferred_logging() {
                tracing::warn!(
                    "Watchdog park stopped by the torque guard on joint {} ({}); holding torque at the backed-off pose and latching WatchdogTimeout",
                    j + 1,
                    if push_in { "the park path pushes into the load" } else { "the load did not relax during the park" }
                );
            }
            let mut hold = [0.0f32; crate::model::MAX_MOTION_DOF];
            let n = self.commanded_positions.len().min(hold.len());
            hold[..n].copy_from_slice(&self.commanded_positions[..n]);
            self.latch_watchdog_fault();
            crate::rt_core::scratch::copy_into(&mut self.target_positions, &hold[..n]);
            crate::rt_core::scratch::copy_into(&mut self.commanded_positions, &hold[..n]);
        }
    }

    /// Side-effect-free preview of [`Self::torque_hold_filter`]: clamps `targets` to the active
    /// torque hold without mutating `motion_limit` or clearing `workspace_violation`.
    pub(crate) fn preview_torque_hold_filter(&self, targets: &mut [f32]) -> Option<usize> {
        let h = self.torque_guard.hold?;
        let t = targets.get_mut(h.joint)?;
        if (*t - h.hold_q) * h.sign > 0.0 {
            *t = h.hold_q;
            return Some(h.joint);
        }
        None
    }

    /// While a torque hold is in force: a servo / jog target that moves the held joint
    /// past the hold position in the push direction is clamped to the hold position.
    /// Targets that keep it there or move it the other way (reducing the load) pass.
    pub(crate) fn torque_hold_filter(&mut self, targets: &mut [f32]) {
        if let Some(j) = self.preview_torque_hold_filter(targets) {
            self.note_motion_limit(MotionLimitReason::Torque, Some(j));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn persistence_and_measured_position_release() {
        let policy = TorqueGuardPolicy { enabled: true, persist_ms: 100, ..Default::default() };
        let thresholds = [Some(1.0), None, None, None, None, None];
        let mut g = TorqueGuard::default();
        assert!(g.step(&[2.0], &thresholds, 0.05, 1.0, &policy, true, true, &[0.0], &[0.0], None).is_none());
        assert_eq!(g.step(&[2.0], &thresholds, 0.05, 1.05, &policy, true, true, &[0.0], &[0.0], None), Some(0));
        assert!(g.hold.is_some());
        assert_eq!(g.step(&[], &thresholds, 0.01, 1.055, &policy, true, false, &[0.1], &[0.0], None), None);
        assert!(g.hold.is_some(), "missing torque feedback must not release a hold");
        assert_eq!(g.step(&[0.1], &thresholds, 0.01, 1.06, &policy, true, true, &[0.1], &[0.0], None), None);
        assert!(g.hold.is_none());
    }

    #[test]
    fn back_off_is_bounded_by_the_first_trip_position() {
        let h = TorqueHold { joint: 0, hold_q: 0.033, sign: 1.0, torque_nm: 20.0, threshold_nm: 16.8, since_s: 0.0,
            anchor_q: 0.033, push_in: false, retreat: Retreat::Unused };
        let b = TORQUE_HOLD_BACKOFF_RAD;
        assert!((h.backed_off(0.033) - (0.033 - b)).abs() < 1e-6, "first trip: back off");
        assert!((h.backed_off(0.033 - b) - (0.033 - b)).abs() < 1e-6, "repeated trip: no creep");
        assert!((h.backed_off(0.033 - 0.002) - (0.033 - b)).abs() < 1e-6, "partly followed: up to the bound");
        assert!((h.backed_off(0.020) - 0.020).abs() < 1e-6, "pushed back by the load: hold where it is");
        assert!((h.backed_off(0.040) - (0.040 - b)).abs() < 1e-6, "further in: back off from there");
        let neg = TorqueHold { sign: -1.0, anchor_q: -0.5, hold_q: -0.5, ..h };
        assert!((neg.backed_off(-0.5) - (-0.5 + b)).abs() < 1e-6);
    }

    #[test]
    fn push_in_past_the_trip_position_trips_at_once() {
        let policy = TorqueGuardPolicy { enabled: true, persist_ms: 100, ..Default::default() };
        let thresholds = [Some(1.0), None, None, None, None, None];
        let mut g = TorqueGuard::default();
        g.step(&[2.0], &thresholds, 0.05, 1.0, &policy, true, true, &[0.0], &[0.0], None);
        assert_eq!(g.step(&[2.0], &thresholds, 0.05, 1.05, &policy, true, true, &[0.0], &[0.0], None), Some(0));
        let trips = g.trips_total;
        // a command within TORQUE_PUSH_IN_RAD of the trip position (torque below the threshold) does not trip
        assert_eq!(g.step(&[0.5], &thresholds, 0.02, 1.07, &policy, true, true, &[0.0], &[TORQUE_PUSH_IN_RAD * 0.9], Some(&[0.1])), None);
        // past it: trips on the same tick
        assert_eq!(g.step(&[0.5], &thresholds, 0.02, 1.09, &policy, true, true, &[0.0], &[TORQUE_PUSH_IN_RAD * 1.1], Some(&[0.1])), Some(0));
        assert!(g.hold.unwrap().push_in && g.trips_total == trips + 1);
    }
}
