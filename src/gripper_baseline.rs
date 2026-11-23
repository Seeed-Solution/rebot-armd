//! Learned free-motion torque of the gripper jaw (`gripper_baseline.json`, next to
//! `gripper_calib.json`).
//!
//! The jaw needs torque to move even with nothing between the fingers (friction of the finger
//! guides and the gear). It depends on the position (B601-DM, new finger, 2026-11-23, |v| > 0.3
//! rad/s: median 0.46-0.51 N·m near open, 0.12 N·m near closed), on the direction and on the
//! finger, so one absolute contact threshold either misses contacts (old finger: free 0.3-0.45
//! N·m) or reads free motion as contact (new finger: 0.5-0.69 N·m, the close crept at the contact
//! speed and timed out). The baseline is learned per position bin, direction and speed band from
//! unloaded moves (opening moves, closes that reach the target without a contact verdict); a
//! grasp contact is |τ| above baseline + margin. The absolute torque cap
//! (`gripper_policy.max_close_torque_nm`) stays independent of it.
//!
//! The baseline belongs to one tool (tool id + tool config digest); another tool starts unlearned.

use serde::{Deserialize, Serialize};

/// Position bins over the jaw stroke [min(closed, open), max(closed, open)].
pub const BINS: usize = 12;
/// Below this |v| (rad/s) a sample belongs to the slow band (the grasp contact speed is 0.3).
pub const SLOW_BAND_MAX_RAD_S: f32 = 0.6;
/// Samples taken while the jaw moves slower than this (rad/s) are not used for learning.
pub const LEARN_MIN_SPEED_RAD_S: f32 = 0.15;
/// A bin needs at least this many samples of one move to be updated.
pub const LEARN_MIN_BIN_SAMPLES: usize = 4;
/// Samples within this distance of the move's start (acceleration, releasing a held object) or
/// end (the fingers meeting at closed, the mechanical open stop) are not used for learning (rad).
pub const LEARN_START_GUARD_RAD: f32 = 0.3;
pub const LEARN_END_GUARD_RAD: f32 = 0.3;
/// Contact margin above the baseline: max(MARGIN_FLOOR_NM, MARGIN_SPREADS x spread) (N·m).
pub const MARGIN_FLOOR_NM: f32 = 0.15;
pub const MARGIN_SPREADS: f32 = 2.5;
/// Extra margin when the estimate comes from the other direction (N·m).
pub const OTHER_DIRECTION_EXTRA_NM: f32 = 0.08;
/// Lowest contact threshold from a learned baseline (N·m).
pub const MIN_CONTACT_NM: f32 = 0.2;
/// The contact threshold stays below this fraction of the torque cap.
pub const CONTACT_CAP_FRACTION: f32 = 0.85;
/// Bounded change of a bin median per accepted move (N·m): rising slower than falling, so an
/// object brushed during a move cannot lift the baseline quickly (a falling baseline, e.g. after
/// lubrication, only makes the contact detection more sensitive).
pub const MAX_RISE_PER_MOVE_NM: f32 = 0.05;
pub const MAX_FALL_PER_MOVE_NM: f32 = 0.10;
/// A move is learned only when every bin it covers is within the learned profile: its median
/// at most max(ACCEPT_FLOOR_NM, ACCEPT_SPREADS x spread) above the learned median (the contact
/// margin). One bin above it rejects the whole move (a dragging open, a compliant object): a
/// higher free-motion torque is taken only by a calibration (`gripper_calibrate`).
pub const ACCEPT_FLOOR_NM: f32 = MARGIN_FLOOR_NM;
pub const ACCEPT_SPREADS: f32 = MARGIN_SPREADS;
/// EWMA weight floor of a new move (1 / (moves + 1) until it reaches this).
pub const EWMA_MIN_WEIGHT: f32 = 0.25;
/// ... for a move below the learned median.
pub const EWMA_MIN_WEIGHT_FALL: f32 = 0.5;
/// Spread (robust sigma) bounds (N·m).
pub const MIN_SPREAD_NM: f32 = 0.02;
pub const MAX_SPREAD_NM: f32 = 0.3;
/// The closing direction counts as learned after this many accepted moves in a bin.
pub const LEARNED_MOVES: u32 = 2;
/// Samples kept per move (decimated when full).
pub const MOVE_SAMPLE_CAP: usize = 512;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction { Close, Open }

impl Direction {
    fn other(self) -> Self { match self { Direction::Close => Direction::Open, Direction::Open => Direction::Close } }
}

/// One table: one direction and speed band, BINS position bins.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Table {
    /// Median |τ| (N·m) per bin.
    pub median_nm: [f32; BINS],
    /// Robust spread (1.4826 x MAD, with the move-to-move change) per bin (N·m).
    pub spread_nm: [f32; BINS],
    /// Accepted moves per bin (0: not learned).
    pub moves: [u32; BINS],
}

/// Persisted free-motion torque baseline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FreeTorqueBaseline {
    pub version: u32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_config_digest: Option<String>,
    /// `--follower` armd ran with (a sim or mock baseline never applies to the real arm).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub follower: Option<String>,
    /// Stroke the bins cover (rad).
    pub lo_rad: f32,
    pub hi_rad: f32,
    pub close_fast: Table,
    pub close_slow: Table,
    pub open_fast: Table,
    pub open_slow: Table,
    /// Accepted moves per direction.
    pub close_moves: u64,
    pub open_moves: u64,
    /// Moves with at least one bin rejected as an outlier.
    pub rejected_moves: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ts: Option<u64>,
}

/// Baseline at one position: median and spread (N·m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimate {
    pub baseline_nm: f32,
    pub spread_nm: f32,
    /// Taken from the other direction's tables.
    pub other_direction: bool,
}

impl Estimate {
    /// Contact margin above the baseline (N·m).
    pub fn margin_nm(&self) -> f32 {
        MARGIN_FLOOR_NM.max(MARGIN_SPREADS * self.spread_nm) + if self.other_direction { OTHER_DIRECTION_EXTRA_NM } else { 0.0 }
    }

    /// Contact threshold: baseline + margin, at least MIN_CONTACT_NM, below
    /// CONTACT_CAP_FRACTION of the cap.
    pub fn threshold_nm(&self, cap_nm: Option<f32>) -> f32 {
        let t = (self.baseline_nm + self.margin_nm()).max(MIN_CONTACT_NM);
        match cap_nm { Some(cap) => t.min(CONTACT_CAP_FRACTION * cap), None => t }
    }
}

/// Samples of one gripper move: (position, |τ|, |v|), fixed capacity, decimated when full.
#[derive(Debug, Clone)]
pub struct MoveSamples {
    buf: Box<[(f32, f32, f32); MOVE_SAMPLE_CAP]>,
    len: usize,
    /// Keep every `stride`-th offered sample.
    stride: usize,
    offered: usize,
}

impl Default for MoveSamples {
    fn default() -> Self { Self { buf: Box::new([(0.0, 0.0, 0.0); MOVE_SAMPLE_CAP]), len: 0, stride: 1, offered: 0 } }
}

impl MoveSamples {
    pub fn push(&mut self, q: f32, tau_abs: f32, v_abs: f32) {
        if !(q.is_finite() && tau_abs.is_finite() && v_abs.is_finite()) { return; }
        self.offered += 1;
        if !(self.offered - 1).is_multiple_of(self.stride) { return; }
        if self.len == MOVE_SAMPLE_CAP {
            // keep every other sample, then take every other offered one
            for i in 0..MOVE_SAMPLE_CAP / 2 { self.buf[i] = self.buf[2 * i]; }
            self.len = MOVE_SAMPLE_CAP / 2;
            self.stride *= 2;
        }
        self.buf[self.len] = (q, tau_abs, v_abs);
        self.len += 1;
    }

    pub fn len(&self) -> usize { self.len }
    pub fn is_empty(&self) -> bool { self.len == 0 }
    pub fn iter(&self) -> impl Iterator<Item = &(f32, f32, f32)> { self.buf[..self.len].iter() }
}

/// One finished, unloaded move offered for learning.
#[derive(Debug, Clone)]
pub struct MoveRecord {
    pub direction: Direction,
    pub start_rad: f32,
    pub end_rad: f32,
    pub samples: MoveSamples,
    /// A calibration stroke (nothing in the jaws, checked): its first value per bin is not
    /// limited to the absolute rule's free-motion torque.
    pub calibration: bool,
}

/// What [`FreeTorqueBaseline::learn`] did with a move.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LearnOutcome {
    pub bins_updated: usize,
    pub bins_rejected: usize,
}

fn median(v: &mut [f32]) -> f32 {
    v.sort_by(|a, b| a.total_cmp(b));
    let n = v.len();
    if n % 2 == 1 { v[n / 2] } else { 0.5 * (v[n / 2 - 1] + v[n / 2]) }
}

/// (median, 1.4826 x MAD) of `v` (non-empty).
pub fn robust_stats(v: &mut [f32]) -> (f32, f32) {
    let m = median(v);
    for x in v.iter_mut() { *x = (*x - m).abs(); }
    (m, 1.4826 * median(v))
}

impl FreeTorqueBaseline {
    pub const VERSION: u32 = 1;

    /// Empty baseline over the stroke between `closed_rad` and `open_rad`.
    pub fn new(closed_rad: f32, open_rad: f32, tool_id: Option<String>, tool_config_digest: Option<String>, follower: Option<String>) -> Self {
        Self {
            version: Self::VERSION,
            tool_id,
            tool_config_digest,
            follower,
            lo_rad: closed_rad.min(open_rad),
            hi_rad: closed_rad.max(open_rad),
            close_fast: Table::default(),
            close_slow: Table::default(),
            open_fast: Table::default(),
            open_slow: Table::default(),
            close_moves: 0,
            open_moves: 0,
            rejected_moves: 0,
            ts: None,
        }
    }

    /// The baseline was learned with this tool (id and config digest) on this follower; every
    /// identity field must be present and equal.
    pub fn belongs_to(&self, tool_id: &str, tool_config_digest: &str, follower: Option<&str>) -> bool {
        self.tool_id.as_deref() == Some(tool_id)
            && self.tool_config_digest.as_deref() == Some(tool_config_digest)
            && follower.is_some() && self.follower.as_deref() == follower
    }

    /// Structural checks of a loaded file.
    pub fn validate(&self) -> Result<(), String> {
        if self.version != Self::VERSION { return Err(format!("unsupported baseline version {}", self.version)); }
        if !(self.lo_rad.is_finite() && self.hi_rad.is_finite() && self.hi_rad - self.lo_rad > 0.1) {
            return Err(format!("invalid stroke [{}, {}]", self.lo_rad, self.hi_rad));
        }
        for t in [&self.close_fast, &self.close_slow, &self.open_fast, &self.open_slow] {
            for b in 0..BINS {
                let (m, s) = (t.median_nm[b], t.spread_nm[b]);
                if t.moves[b] > 0 && !(m.is_finite() && (0.0..20.0).contains(&m) && s.is_finite() && (0.0..=MAX_SPREAD_NM).contains(&s)) {
                    return Err(format!("invalid bin {b}: median {m}, spread {s}"));
                }
            }
        }
        Ok(())
    }

    fn table(&self, dir: Direction, slow: bool) -> &Table {
        match (dir, slow) {
            (Direction::Close, false) => &self.close_fast,
            (Direction::Close, true) => &self.close_slow,
            (Direction::Open, false) => &self.open_fast,
            (Direction::Open, true) => &self.open_slow,
        }
    }

    fn table_mut(&mut self, dir: Direction, slow: bool) -> &mut Table {
        match (dir, slow) {
            (Direction::Close, false) => &mut self.close_fast,
            (Direction::Close, true) => &mut self.close_slow,
            (Direction::Open, false) => &mut self.open_fast,
            (Direction::Open, true) => &mut self.open_slow,
        }
    }

    /// Fractional bin coordinate of `q` (bin centres at 0, 1, ...).
    fn coord(&self, q: f32) -> f32 {
        let f = (q - self.lo_rad) / (self.hi_rad - self.lo_rad);
        (f.clamp(0.0, 1.0) * BINS as f32 - 0.5).clamp(0.0, (BINS - 1) as f32)
    }

    fn bin(&self, q: f32) -> usize { (self.coord(q) + 0.5).floor().clamp(0.0, (BINS - 1) as f32) as usize }

    /// Interpolated (median, spread) of one table at `q`; None when no learned bin is within one bin.
    fn table_at(t: &Table, c: f32) -> Option<(f32, f32)> {
        let i0 = c.floor() as usize;
        let i1 = (i0 + 1).min(BINS - 1);
        let w = c - i0 as f32;
        match (t.moves[i0] > 0, t.moves[i1] > 0) {
            (true, true) => Some((
                t.median_nm[i0] * (1.0 - w) + t.median_nm[i1] * w,
                t.spread_nm[i0].max(t.spread_nm[i1]),
            )),
            (true, false) => Some((t.median_nm[i0], t.spread_nm[i0])),
            (false, true) => Some((t.median_nm[i1], t.spread_nm[i1])),
            (false, false) => {
                // nearest learned bin one further out on either side
                let near = [i0.checked_sub(1), Some(i1 + 1).filter(|&i| i < BINS)];
                near.into_iter().flatten().filter(|&i| t.moves[i] > 0)
                    .map(|i| (t.median_nm[i], t.spread_nm[i]))
                    .reduce(|a, b| (a.0.max(b.0), a.1.max(b.1)))
            }
        }
    }

    /// Baseline at `q` moving in `dir` at |v| `speed`: the matching table, else the other speed
    /// band, else the other direction (with extra margin). None: nothing learned near `q`.
    pub fn estimate(&self, q: f32, dir: Direction, speed: f32) -> Option<Estimate> {
        if !q.is_finite() { return None; }
        let c = self.coord(q);
        let slow = speed.abs() < SLOW_BAND_MAX_RAD_S;
        for (d, s, other) in [(dir, slow, false), (dir, !slow, false), (dir.other(), slow, true), (dir.other(), !slow, true)] {
            if let Some((m, sp)) = Self::table_at(self.table(d, s), c) {
                return Some(Estimate { baseline_nm: m, spread_nm: sp, other_direction: other });
            }
        }
        None
    }

    /// Learned moves of a direction (any bin).
    pub fn moves(&self, dir: Direction) -> u64 { match dir { Direction::Close => self.close_moves, Direction::Open => self.open_moves } }

    /// `"unlearned"` (no data), `"learning"` (some data, the closing direction not yet over
    /// LEARNED_MOVES moves in most bins), `"learned"`.
    pub fn state(&self) -> &'static str {
        if self.close_moves == 0 && self.open_moves == 0 { return "unlearned"; }
        let learned_bins = (0..BINS).filter(|&b| self.close_fast.moves[b].max(self.close_slow.moves[b]) >= LEARNED_MOVES).count();
        if learned_bins * 2 >= BINS { "learned" } else { "learning" }
    }

    /// Learn from one unloaded move. Samples near the start and the end of the move and samples
    /// slower than LEARN_MIN_SPEED_RAD_S are dropped; each bin with at least
    /// LEARN_MIN_BIN_SAMPLES samples gets the move's robust median, bounded; a move with a bin
    /// above the learned profile (ACCEPT_*) is rejected as a whole.
    /// A bin's first value is at most `initial_max_nm` (the free-motion torque the absolute
    /// contact rule allows for: `contact_torque_nm` − MARGIN_FLOOR_NM); it then rises by at most
    /// MAX_RISE_PER_MOVE_NM per move.
    pub fn learn(&mut self, rec: &MoveRecord, initial_max_nm: f32) -> LearnOutcome {
        let mut out = LearnOutcome::default();
        let travel = rec.end_rad - rec.start_rad;
        let sign = if travel >= 0.0 { 1.0 } else { -1.0 };
        let mut groups: Vec<Vec<f32>> = vec![Vec::new(); 2 * BINS];
        for &(q, tau, v) in rec.samples.iter() {
            let from_start = (q - rec.start_rad) * sign;
            let to_end = (rec.end_rad - q) * sign;
            if from_start < LEARN_START_GUARD_RAD || to_end < LEARN_END_GUARD_RAD || v < LEARN_MIN_SPEED_RAD_S { continue; }
            let slow = v < SLOW_BAND_MAX_RAD_S;
            groups[usize::from(slow) * BINS + self.bin(q)].push(tau);
        }
        // per bin (median, spread) of this move; any bin above the learned profile rejects it
        let mut stats: Vec<(usize, bool, f32, f32)> = Vec::new();
        for (gi, g) in groups.iter_mut().enumerate() {
            if g.len() < LEARN_MIN_BIN_SAMPLES { continue; }
            let (m, s) = robust_stats(g);
            stats.push((gi % BINS, gi >= BINS, m, s));
        }
        let above = stats.iter().filter(|&&(b, slow, m, _)| {
            let t = self.table(rec.direction, slow);
            t.moves[b] > 0 && m - t.median_nm[b] > ACCEPT_FLOOR_NM.max(ACCEPT_SPREADS * t.spread_nm[b])
        }).count();
        if above > 0 {
            out.bins_rejected = above;
            self.rejected_moves += 1;
            return out;
        }
        for (b, slow, m, s) in stats {
            let t = self.table_mut(rec.direction, slow);
            if t.moves[b] == 0 {
                t.median_nm[b] = m.min(initial_max_nm.max(0.0));
                t.spread_nm[b] = s.clamp(MIN_SPREAD_NM, MAX_SPREAD_NM);
                t.moves[b] = 1;
                out.bins_updated += 1;
                continue;
            }
            let dev = m - t.median_nm[b];
            // a lower torque is followed faster (it only makes the detection more sensitive)
            let floor = if dev < 0.0 { EWMA_MIN_WEIGHT_FALL } else { EWMA_MIN_WEIGHT };
            let w = (1.0 / (t.moves[b] as f32 + 1.0)).max(floor);
            t.median_nm[b] += (w * dev).clamp(-MAX_FALL_PER_MOVE_NM, MAX_RISE_PER_MOVE_NM);
            // the spread covers the noise within a move and the change from move to move
            let s_move = s.max(dev.abs()).clamp(MIN_SPREAD_NM, MAX_SPREAD_NM);
            t.spread_nm[b] = (t.spread_nm[b] + w * (s_move - t.spread_nm[b])).clamp(MIN_SPREAD_NM, MAX_SPREAD_NM);
            t.moves[b] = t.moves[b].saturating_add(1);
            out.bins_updated += 1;
        }
        if out.bins_updated > 0 {
            match rec.direction { Direction::Close => self.close_moves += 1, Direction::Open => self.open_moves += 1 }
        }
        out
    }
}

/// Contact verdict details of the last grasp contact (observation `gripper.baseline.last_contact`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ContactInfo {
    /// Measured jaw at the contact (rad).
    pub at_rad: f32,
    /// |τ| at the verdict (N·m).
    pub torque_nm: f32,
    /// Torque threshold in force (N·m); None when torque contact was off at that point.
    pub threshold_nm: Option<f32>,
    /// Free-motion baseline at that position (N·m); None when not learned.
    pub baseline_nm: Option<f32>,
    /// |τ| minus the baseline (N·m).
    pub margin_nm: Option<f32>,
    pub source: ContactSource,
    pub threshold_source: ThresholdSource,
}

/// What made the contact verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContactSource {
    /// |τ| above the threshold.
    Torque,
    /// The jaw fell behind the command and slowed down.
    Lag,
}

/// Where the contact torque threshold came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThresholdSource {
    /// The learned baseline + margin.
    Baseline,
    /// No baseline near this position: `gripper_policy.contact_torque_nm` (the absolute rule).
    ContactTorqueNm,
    /// Torque contact off (no `contact_torque_nm`).
    None,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(dir: Direction, from: f32, to: f32, f: impl Fn(f32) -> f32, speed: f32) -> MoveRecord {
        let mut samples = MoveSamples::default();
        let n = 300;
        for i in 0..=n {
            let q = from + (to - from) * i as f32 / n as f32;
            samples.push(q, f(q), speed);
        }
        MoveRecord { direction: dir, start_rad: from, end_rad: to, samples, calibration: false }
    }

    #[test]
    fn learns_position_dependent_torque_and_bounds_changes() {
        let mut b = FreeTorqueBaseline::new(0.0, -5.7, None, None, None);
        assert_eq!(b.state(), "unlearned");
        assert!(b.estimate(-3.0, Direction::Close, 1.0).is_none());
        let shape = |q: f32| 0.12 + 0.08 * (-q);
        b.learn(&record(Direction::Open, 0.0, -5.7, shape, 1.0), 2.0);
        // closing falls back on the opening tables with extra margin
        let e = b.estimate(-3.0, Direction::Close, 1.0).unwrap();
        assert!(e.other_direction && (e.baseline_nm - shape(-3.0)).abs() < 0.03, "{e:?}");
        b.learn(&record(Direction::Close, -5.7, 0.0, shape, 1.0), 2.0);
        b.learn(&record(Direction::Close, -5.7, 0.0, shape, 1.0), 2.0);
        assert_eq!(b.state(), "learned");
        let e = b.estimate(-3.0, Direction::Close, 1.0).unwrap();
        assert!(!e.other_direction && (e.baseline_nm - shape(-3.0)).abs() < 0.03, "{e:?}");
        // a move with an object pressing (+0.5 N·m) is rejected, however often it comes; a move
        // pressing in one bin only is rejected as a whole
        let before = b.clone();
        for _ in 0..5 {
            let o = b.learn(&record(Direction::Close, -5.7, 0.0, |q| shape(q) + 0.5, 1.0), 2.0);
            assert!(o.bins_rejected > 0 && o.bins_updated == 0);
        }
        let o = b.learn(&record(Direction::Close, -5.7, 0.0, |q| shape(q) + if (-3.2..-2.8).contains(&q) { 0.4 } else { 0.0 }, 1.0), 2.0);
        assert!(o.bins_rejected > 0 && o.bins_updated == 0, "{o:?}");
        assert_eq!(b.estimate(-3.0, Direction::Close, 1.0), before.estimate(-3.0, Direction::Close, 1.0));
        assert_eq!(b.rejected_moves, before.rejected_moves + 6);
        // within the profile (+0.1 N·m): taken, at most MAX_RISE_PER_MOVE_NM
        let before = b.estimate(-3.0, Direction::Close, 1.0).unwrap().baseline_nm;
        b.learn(&record(Direction::Close, -5.7, 0.0, |q| shape(q) + 0.1, 1.0), 2.0);
        let after = b.estimate(-3.0, Direction::Close, 1.0).unwrap().baseline_nm;
        assert!(after > before && after - before <= MAX_RISE_PER_MOVE_NM + 1e-4, "{before} -> {after}");
        // a lower baseline (lubrication) is followed, at most MAX_FALL_PER_MOVE_NM per move
        let mut prev = after;
        for _ in 0..12 {
            b.learn(&record(Direction::Close, -5.7, 0.0, |q| 0.5 * shape(q), 1.0), 2.0);
            let now = b.estimate(-3.0, Direction::Close, 1.0).unwrap().baseline_nm;
            assert!(prev - now <= MAX_FALL_PER_MOVE_NM + 1e-4);
            prev = now;
        }
        assert!((prev - 0.5 * shape(-3.0)).abs() < 0.04, "{prev}");
    }

    #[test]
    fn first_value_is_bounded_then_rises_slowly() {
        let mut b = FreeTorqueBaseline::new(0.0, -5.7, None, None, None);
        // a compressed object on the first move cannot set a high baseline at once
        b.learn(&record(Direction::Open, 0.0, -5.7, |_| 0.9, 1.0), 0.45);
        assert!((b.estimate(-3.0, Direction::Open, 1.0).unwrap().baseline_nm - 0.45).abs() < 1e-4);
        b.learn(&record(Direction::Open, 0.0, -5.7, |_| 0.6, 1.0), 0.45);
        let e = b.estimate(-3.0, Direction::Open, 1.0).unwrap().baseline_nm;
        assert!(e <= 0.45 + MAX_RISE_PER_MOVE_NM + 1e-4, "{e}");
    }

    #[test]
    fn identity_needs_every_field() {
        let b = FreeTorqueBaseline::new(0.0, -5.7, Some("stock".into()), Some("d".into()), Some("damiao".into()));
        assert!(b.belongs_to("stock", "d", Some("damiao")));
        assert!(!b.belongs_to("stock", "d", Some("sim")));
        assert!(!b.belongs_to("stock", "d", None));
        assert!(!b.belongs_to("stock", "e", Some("damiao")));
        let legacy = FreeTorqueBaseline::new(0.0, -5.7, None, None, None);
        assert!(!legacy.belongs_to("stock", "d", Some("damiao")));
    }

    #[test]
    fn guards_drop_the_ends_of_a_move() {
        let mut b = FreeTorqueBaseline::new(0.0, -5.7, None, None, None);
        // torque spike at the end (fingers meet at closed): not learned
        b.learn(&record(Direction::Close, -5.7, 0.0, |q| if q > -0.25 { 3.0 } else { 0.3 }, 1.0), 2.0);
        let e = b.estimate(-0.1, Direction::Close, 1.0).unwrap();
        assert!(e.baseline_nm < 0.35, "{e:?}");
    }

    #[test]
    fn sample_buffer_decimates() {
        let mut s = MoveSamples::default();
        for i in 0..5000 { s.push(i as f32, 0.1, 1.0); }
        assert!(s.len() <= MOVE_SAMPLE_CAP && s.len() > MOVE_SAMPLE_CAP / 4);
        let qs: Vec<f32> = s.iter().map(|x| x.0).collect();
        assert!(qs.windows(2).all(|w| w[1] > w[0]));
        assert!(*qs.last().unwrap() > 4000.0);
    }

    #[test]
    fn serde_round_trip() {
        let mut b = FreeTorqueBaseline::new(0.0, -5.7, Some("stock".into()), Some("abc".into()), Some("sim".into()));
        b.learn(&record(Direction::Open, 0.0, -5.7, |_| 0.4, 1.0), 2.0);
        let text = serde_json::to_string(&b).unwrap();
        let back: FreeTorqueBaseline = serde_json::from_str(&text).unwrap();
        assert_eq!(back, b);
        back.validate().unwrap();
    }
}
