//! Adaptive grasp pose selection (`strategy: "adaptive"` of the grasp dry run), phase A:
//! selection and validation only, no actuation.
//!
//! One immutable [`Snapshot`] (target evidence bound to its capture pose, camera, table,
//! tool, opening, workspace, start joints) is captured under the supervisor lock; [`select`]
//! then runs outside it: contact chords along the footprint ([`candidates::chords`]),
//! approach directions over the upper hemisphere with both jaw half-turns
//! ([`candidates::hypotheses`]), cheap geometric rejection, seeded endpoint IK, a shortlist
//! with a reserved radial-oblique quota, the connected path check
//! ([`feasibility::check`]), robustness and scoring ([`score`]). Fixed enumeration order,
//! quotas and tie-breaks make it deterministic; a wall-time abort returns
//! `budget_exhausted` and never a partially checked winner.
//!
//! The selected candidate carries what an executor needs (phase B): pre-grasp, insertion
//! along `a`, contact, separation and lift TCP poses with their joint solutions.

pub mod candidates;
pub mod feasibility;
pub mod geometry;
pub mod score;
pub mod snapshot;

use std::collections::BTreeMap;
use std::time::Instant;

use serde_json::{json, Value};

pub use snapshot::{capture, Snapshot};
use candidates::{Chord, DirClass, Hypothesis, Rejection};
use feasibility::{Checked, Work};
use score::{Robustness, Scores};
use crate::kinematics::{self, Pose};
use crate::motion::MotionError;

/// Grasp request `strategy`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Strategy { #[default] Legacy, Adaptive }

/// `strategy`: absent / null / `"legacy"` → legacy, `"adaptive"` → adaptive.
pub fn parse_strategy(body: &Value) -> Result<Strategy, MotionError> {
    match body.get("strategy") {
        None | Some(Value::Null) => Ok(Strategy::Legacy),
        Some(Value::String(s)) if s == "legacy" => Ok(Strategy::Legacy),
        Some(Value::String(s)) if s == "adaptive" => Ok(Strategy::Adaptive),
        Some(_) => Err(MotionError::InvalidCommand("strategy must be \"legacy\", \"adaptive\" or null".into())),
    }
}

/// Refusal of an executed (not dry-run) grasp with `strategy: "adaptive"`: execution with a
/// selected candidate is not available yet.
pub fn refuse_execution(body: &Value) -> Result<(), MotionError> {
    match parse_strategy(body)? {
        Strategy::Legacy => Ok(()),
        Strategy::Adaptive => Err(MotionError::Unsupported("strategy \"adaptive\" is available for dry_run only; execution uses the legacy strategy".into())),
    }
}

/// Shortlist size (candidates given the full path check).
pub const SHORTLIST: usize = 8;
/// Shortlist places reserved for radial obliques.
pub const RESERVED_RADIAL_OBLIQUE: usize = 2;
/// Most shortlist places per direction class.
pub const MAX_PER_CLASS: usize = 3;
/// Refinement candidates around the best feasible oblique.
pub const REFINE: usize = 4;
/// Representative alternatives in the reply.
pub const MAX_ALTERNATIVES: usize = 6;

/// Outcome of a selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome { Selected, InsufficientEvidence, BudgetExhausted, Infeasible }

impl Outcome {
    pub fn name(self) -> &'static str {
        match self { Self::Selected => "selected", Self::InsufficientEvidence => "insufficient_evidence", Self::BudgetExhausted => "budget_exhausted", Self::Infeasible => "infeasible" }
    }
}

/// A fully checked candidate.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub id: String,
    pub hy: Hypothesis,
    pub chord: Chord,
    pub checked: Checked,
    pub robustness: Robustness,
    pub scores: Scores,
    pub visible_at_pregrasp: bool,
}

/// What an executor needs to run the selected candidate (phase B). TCP poses, base frame.
#[derive(Debug, Clone)]
pub struct ExecutionPlan {
    pub pregrasp: Pose,
    /// Unit insertion direction `a` (pre-grasp → contact).
    pub insertion_dir: [f64; 3],
    pub insertion_m: f64,
    pub grasp: Pose,
    pub separation: Pose,
    pub lift: Pose,
    pub jaw_opening_m: f64,
    pub chord_width_m: f64,
    pub q_pregrasp: [f64; 6],
    pub q_grasp: [f64; 6],
    pub q_lift: [f64; 6],
}

/// Ordered macro child step produced from a selected [`ExecutionPlan`] for Phase B execution.
///
/// Phase A keeps live `strategy: "adaptive"` refused via [`refuse_execution`] and serves
/// dry-run selection only. Phase B installs [`SelectedGraspMacro::begin`] as an A7 macro parent
/// and feeds each [`SelectedMacroStep`] through `MacroCtx` / `InstallChild` without changing
/// the A7 broker or owner contract.
#[derive(Debug, Clone, PartialEq)]
pub enum SelectedMacroStep {
    /// Linear TCP sub-motion with optional contact guard (for insertion).
    MoveTcp {
        phase: &'static str,
        goal: Pose,
        linear_mps: f64,
        contact: Option<crate::rt_core::ops::ContactGuard>,
    },
    /// Gripper open / close sub-motion.
    Gripper {
        phase: &'static str,
        command: crate::motion::GripperCommand,
        fraction: Option<f64>,
        grasp: bool,
    },
}

/// Phase B handoff bundle translating a selected [`ExecutionPlan`] into an A7
/// [`crate::rt_core::requests::actions::BeginMacroSpec`] plus its ordered child sequence.
#[derive(Debug, Clone)]
pub struct SelectedGraspMacro {
    pub begin: crate::rt_core::requests::actions::BeginMacroSpec,
    pub steps: Vec<SelectedMacroStep>,
    pub expected_duration_s: f64,
}

impl ExecutionPlan {
    /// Builds the Phase B A7 macro bundle (`BeginMacroSpec` + ordered `pregrasp` -> `pre_open`
    /// -> guarded `insert` -> `close` -> `separation` -> `lift` child steps) ready to install
    /// through the shared A7 macro lifecycle when Phase B enables adaptive execution.
    pub fn into_macro_plan(
        &self,
        prepared_boot: crate::rt_core::broker::BootId,
        start: &[f32],
        deadline: Instant,
        tag: Option<&str>,
        contact_guard: Option<crate::rt_core::ops::ContactGuard>,
    ) -> Result<SelectedGraspMacro, &'static str> {
        use crate::motion::GripperCommand;
        use crate::rt_core::requests::actions::{BeginMacroSpec, MacroKind};

        let mut begin = BeginMacroSpec::new(MacroKind::Grasp, prepared_boot).with_start(start)?;
        begin.check_workspace_revision = true;
        begin.macro_deadline = Some(deadline);
        begin.timeout_code = crate::safety::BoundedText::new("GRASP_TIMEOUT");
        if let Some(tag) = tag {
            begin.tag = crate::model::HolderLabel::new(tag);
        }

        let approach_mps = crate::motion::DEFAULT_LINEAR_MPS;
        let insert_mps = (crate::motion::DEFAULT_LINEAR_MPS * 0.5).clamp(0.02, 0.15);
        let steps = vec![
            SelectedMacroStep::MoveTcp {
                phase: "pregrasp",
                goal: self.pregrasp,
                linear_mps: approach_mps,
                contact: None,
            },
            SelectedMacroStep::Gripper {
                phase: "pre_open",
                command: GripperCommand::Open,
                fraction: None,
                grasp: false,
            },
            SelectedMacroStep::MoveTcp {
                phase: "insert",
                goal: self.grasp,
                linear_mps: insert_mps,
                contact: contact_guard,
            },
            SelectedMacroStep::Gripper {
                phase: "close",
                command: GripperCommand::Close,
                fraction: None,
                grasp: true,
            },
            SelectedMacroStep::MoveTcp {
                phase: "separation",
                goal: self.separation,
                linear_mps: insert_mps,
                contact: None,
            },
            SelectedMacroStep::MoveTcp {
                phase: "lift",
                goal: self.lift,
                linear_mps: approach_mps,
                contact: None,
            },
        ];
        let move_est = (self.insertion_m / insert_mps).clamp(0.2, 10.0);
        let expected_duration_s = move_est * 3.0 + 6.0;
        Ok(SelectedGraspMacro {
            begin,
            steps,
            expected_duration_s,
        })
    }
}

impl Candidate {
    pub fn plan(&self, s: &Snapshot) -> ExecutionPlan {
        ExecutionPlan {
            pregrasp: self.hy.pregrasp_pose(), insertion_dir: self.hy.a, insertion_m: self.hy.insertion_m, grasp: self.hy.grasp_pose(),
            separation: self.checked.separation, lift: self.checked.lift, jaw_opening_m: s.tool.open_width_m, chord_width_m: self.hy.width_m,
            q_pregrasp: self.checked.q_pre, q_grasp: self.checked.q_grasp, q_lift: self.checked.q_lift,
        }
    }
}

/// A rejected hypothesis kept for the reply.
#[derive(Debug, Clone)]
pub struct Rejected { pub id: String, pub hy: Hypothesis, pub rejection: Rejection, pub shortlisted: bool }

/// Search counters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    pub chords: usize,
    pub generated: usize,
    pub geometric_survivors: usize,
    pub ik_survivors: usize,
    pub shortlisted: usize,
    pub refined: usize,
    pub fully_checked: usize,
    pub lattice_tests: usize,
    pub work: Work,
}

/// Result of [`select`].
#[derive(Debug, Clone)]
pub struct Selection {
    pub snapshot_id: String,
    pub outcome: Outcome,
    pub code: Option<String>,
    pub message: Option<String>,
    /// Ranked fully checked candidates (best first); the first is selected when the outcome is
    /// `selected` (candidates needing the assumed payload bound only under `insufficient_evidence`).
    pub ranked: Vec<Candidate>,
    pub rejected: Vec<Rejected>,
    pub rejections_by_phase: BTreeMap<&'static str, usize>,
    /// Rejections by `phase:code`.
    pub rejections_by_code: BTreeMap<String, usize>,
    pub evidence: Value,
    pub stats: Stats,
    pub elapsed_ms: f64,
    pub budget_ms: u64,
    /// Phase B input of the selected candidate.
    pub plan: Option<ExecutionPlan>,
}

impl Selection {
    pub fn reachable(&self) -> bool { self.outcome == Outcome::Selected }
    pub fn selected(&self) -> Option<&Candidate> { (self.outcome == Outcome::Selected).then(|| self.ranked.first()).flatten() }

    /// Phase B hook: converts this selection's winning [`ExecutionPlan`] into a
    /// [`SelectedGraspMacro`] ready for A7 `BeginMacro` / `InstallChild` execution.
    /// Returns `None` when the selection did not pick a winner.
    pub fn phase_b_macro_plan(
        &self,
        prepared_boot: crate::rt_core::broker::BootId,
        start: &[f32],
        deadline: Instant,
        tag: Option<&str>,
        contact_guard: Option<crate::rt_core::ops::ContactGuard>,
    ) -> Option<Result<SelectedGraspMacro, &'static str>> {
        self.plan
            .as_ref()
            .map(|p| p.into_macro_plan(prepared_boot, start, deadline, tag, contact_guard))
    }
}

/// Order of the phases, for the deepest failure.
const PHASE_ORDER: [&str; 12] = ["evidence", "geometry", "table", "pre_open", "insertion", "closure", "endpoint_ik", "acquisition", "transition", "separation", "lift", "withdrawal"];

fn phase_rank(p: &str, path: bool) -> usize {
    // an insertion failure of the path check is deeper than one of the cheap stage
    let base = PHASE_ORDER.iter().position(|x| *x == p).unwrap_or(0);
    if path && p == "insertion" { 8 } else { base }
}

struct Search<'a> {
    s: &'a Snapshot,
    start: Instant,
    stats: Stats,
    rejected: Vec<Rejected>,
    by_phase: BTreeMap<&'static str, usize>,
    by_code: BTreeMap<String, usize>,
}

impl Search<'_> {
    fn out_of_budget(&self) -> bool {
        self.start.elapsed().as_secs_f64() * 1000.0 > self.s.budget.wall_ms as f64
            || self.s.budget.cancel.as_ref().is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed))
    }
    fn id(&self, hy: &Hypothesis) -> String { format!("{}-{:03}", self.s.id, hy.index) }
    fn reject(&mut self, hy: &Hypothesis, r: Rejection, shortlisted: bool) {
        *self.by_phase.entry(r.phase).or_default() += 1;
        *self.by_code.entry(format!("{}:{}", r.phase, r.code)).or_default() += 1;
        self.rejected.push(Rejected { id: self.id(hy), hy: hy.clone(), rejection: r, shortlisted });
    }
    fn finish(self, outcome: Outcome, code: Option<String>, message: Option<String>, ranked: Vec<Candidate>, evidence: Value) -> Selection {
        let plan = (outcome == Outcome::Selected).then(|| ranked.first().map(|c| c.plan(self.s))).flatten();
        Selection {
            snapshot_id: self.s.id.clone(), outcome, code, message, ranked, rejected: self.rejected, rejections_by_phase: self.by_phase, rejections_by_code: self.by_code,
            evidence, stats: self.stats, elapsed_ms: self.start.elapsed().as_secs_f64() * 1000.0, budget_ms: self.s.budget.wall_ms, plan,
        }
    }
    fn exhausted(self, ranked_so_far: usize, evidence: Value) -> Selection {
        let m = format!("the search exceeded its {} ms budget ({} candidates fully checked); no candidate is returned", self.s.budget.wall_ms, ranked_so_far);
        self.finish(Outcome::BudgetExhausted, Some("BUDGET_EXHAUSTED".into()), Some(m), Vec::new(), evidence)
    }
}

fn evidence_json(s: &Snapshot, t: Option<&snapshot::Target>) -> Value {
    let mut v = json!({
        "frame_id": s.frame_id,
        "box_norm": s.box_norm,
        "camera_down_acquisition": s.camera_down,
        "tool_volumes": s.tool.source,
        "jaw_open_width_m": s.tool.open_width_m,
        "max_grasp_width_m": s.tool.max_grasp_width_m,
        "torque_limits": s.torque_limits.as_ref().map(|(_, src)| *src),
        "payload_bound_kg": s.evidence.payload_kg.unwrap_or(feasibility::ASSUMED_PAYLOAD_KG),
        "payload_bound_source": if s.evidence.payload_kg.is_some() { "evidence" } else { "assumed" },
        "contour_error": s.evidence.contour_error,
    });
    if let Some(t) = t {
        v["footprint_source"] = json!(t.source.name());
        v["footprint_fallback_reason"] = json!(t.fallback_reason);
        v["footprint_vertices"] = json!(t.poly.len());
        v["footprint_centroid_m"] = json!(t.centroid);
        v["long_axis"] = json!(t.axis);
        v["aspect"] = json!(if t.aspect.is_finite() { Some(t.aspect) } else { None });
        v["axis_ambiguous"] = json!(t.axis_ambiguous);
        v["height_m"] = json!(t.height);
        v["height_source"] = json!(t.height_source);
        v["uncertainty"] = json!({
            "hard_error_m": t.hard_error_m, "scale_error_m": t.scale_error_m, "hand_eye_error_m": t.hand_eye_error_m,
            "contour_error_m": t.contour_error_m, "projection_spread_m": t.projection_spread_m, "table_error_m": snapshot::TABLE_ERROR_M,
            "view_distance_m": t.view_distance_m,
        });
    }
    v
}

fn prescore(s: &Snapshot, t: &snapshot::Target, ch: &Chord, hy: &Hypothesis, q_g: &[f64; 6]) -> f64 {
    score::contact_quality(s, t, ch, hy) - score::preference_penalty(&s.prefs, hy) + 0.2 * feasibility::joint_margin(&s.input, q_g)
}

fn visible(s: &Snapshot, t: &snapshot::Target, tcp: &Pose) -> bool {
    let end = crate::motion::tcp_to_end(tcp, &s.input.tcp_offset);
    let z = t.table_z + (t.height[0] + t.height[1]) / 4.0;
    let m = 4.0;
    t.poly.iter().all(|v| s.cam.project(&end, &[v[0], v[1], z]).is_some_and(|p| p[0] >= m && p[1] >= m && p[0] <= s.cam.size[0] - m && p[1] <= s.cam.size[1] - m))
}

/// Run the selection on a snapshot (outside the supervisor lock).
pub fn select(s: &Snapshot) -> Selection {
    let mut se = Search { s, start: Instant::now(), stats: Stats::default(), rejected: Vec::new(), by_phase: BTreeMap::new(), by_code: BTreeMap::new() };
    let t = match snapshot::target(s) {
        Ok(t) => t,
        Err(m) => {
            *se.by_phase.entry("evidence").or_default() += 1;
            let ev = evidence_json(s, None);
            return se.finish(Outcome::Infeasible, Some("INVALID_COMMAND".into()), Some(format!("evidence: {m}")), Vec::new(), ev);
        }
    };
    let ev = evidence_json(s, Some(&t));
    let (chords, chord_rej) = candidates::chords(&t, &s.tool);
    se.stats.chords = chords.len();
    for r in chord_rej {
        *se.by_phase.entry(r.phase).or_default() += 1;
        *se.by_code.entry(format!("{}:{}", r.phase, r.code)).or_default() += 1;
    }
    let mut cheap = candidates::CheapStats::default();
    let (hyps, hyp_rej) = candidates::hypotheses(s, &t, &chords, &mut cheap);
    se.stats.generated = cheap.generated;
    se.stats.geometric_survivors = hyps.len();
    for (hy, r) in hyp_rej { se.reject(&hy, r, false); }
    if se.out_of_budget() { se.stats.lattice_tests = cheap.lattice_tests; return se.exhausted(0, ev); }
    // the acquisition prefix every candidate shares
    let mut work = Work::default();
    let acq = match feasibility::acquisition(s, &mut work) {
        Ok(a) => a,
        Err(r) => {
            se.stats.work = work;
            se.stats.lattice_tests = cheap.lattice_tests;
            *se.by_phase.entry("acquisition").or_default() += 1;
            let m = format!("acquisition: {} (a different contact pose cannot fix the observation pose)", r.message);
            return se.finish(Outcome::Infeasible, Some(r.code), Some(m), Vec::new(), ev);
        }
    };
    // seeded endpoint IK, neighbours of the same class and half-turn seeding each other
    let mut seed_cache: BTreeMap<(DirClass, bool), [f64; 6]> = BTreeMap::new();
    let mut screened: Vec<(Hypothesis, [f64; 6], f64)> = Vec::new();
    for hy in hyps {
        if se.out_of_budget() { se.stats.work = work; se.stats.lattice_tests = cheap.lattice_tests; return se.exhausted(0, ev); }
        let mut seeds = vec![acq.q_end];
        if let Some(q) = seed_cache.get(&(hy.class, hy.half_turn)) { seeds.insert(0, *q); }
        let q_g = match feasibility::endpoint_ik(&s.input, &hy.grasp_pose(), &seeds, &mut work) {
            Ok(q) => q,
            Err((ep, er)) => { se.reject(&hy, Rejection::new("endpoint_ik", "IK_UNREACHABLE", format!("contact pose: best IK error {:.4} m / {:.2} deg", ep, er.to_degrees())).with(ep, kinematics::PAR.tol_pos), false); continue; }
        };
        if let Err((ep, er)) = feasibility::endpoint_ik(&s.input, &hy.pregrasp_pose(), &[q_g, acq.q_end], &mut work) {
            se.reject(&hy, Rejection::new("endpoint_ik", "IK_UNREACHABLE", format!("pre-grasp pose: best IK error {:.4} m / {:.2} deg", ep, er.to_degrees())).with(ep, kinematics::PAR.tol_pos), false);
            continue;
        }
        seed_cache.insert((hy.class, hy.half_turn), q_g);
        let p = prescore(s, &t, &chords[hy.chord], &hy, &q_g);
        screened.push((hy, q_g, p));
    }
    se.stats.ik_survivors = screened.len();
    screened.sort_by(|a, b| b.2.total_cmp(&a.2).then(a.0.index.cmp(&b.0.index)));
    // shortlist: the reserved radial obliques first, then by prescore with a cap per class
    let mut short: Vec<usize> = screened.iter().enumerate().filter(|(_, x)| x.0.class.is_radial_oblique()).map(|(i, _)| i).take(RESERVED_RADIAL_OBLIQUE).collect();
    for (i, x) in screened.iter().enumerate() {
        if short.len() >= SHORTLIST { break; }
        if short.contains(&i) { continue; }
        let n_class = short.iter().filter(|&&j| screened[j].0.class == x.0.class).count();
        if n_class < MAX_PER_CLASS { short.push(i); }
    }
    short.sort_unstable();
    se.stats.shortlisted = short.len();
    let mut ranked: Vec<Candidate> = Vec::new();
    let mut deepest: Option<(usize, Rejection)> = None;
    let run = |se: &mut Search, hy: &Hypothesis, work: &mut Work, ranked: &mut Vec<Candidate>, deepest: &mut Option<(usize, Rejection)>| {
        match feasibility::check(s, &t, &acq, hy, work) {
            Ok(c) => {
                let r = score::robustness(s, &t, hy, &c, work);
                let ch = chords[hy.chord].clone();
                let scores = score::score(s, &t, &ch, hy, &c, &r);
                let vis = visible(s, &t, &hy.pregrasp_pose());
                ranked.push(Candidate { id: se.id(hy), hy: hy.clone(), chord: ch, checked: c, robustness: r, scores, visible_at_pregrasp: vis });
            }
            Err(r) => {
                let rank = phase_rank(r.phase, true);
                if deepest.as_ref().is_none_or(|(d, _)| rank > *d) { *deepest = Some((rank, r.clone())); }
                se.reject(hy, r, true);
            }
        }
    };
    for &i in &short {
        if se.out_of_budget() { let n = ranked.len(); se.stats.work = work; se.stats.lattice_tests = cheap.lattice_tests; return se.exhausted(n, ev); }
        let hy = screened[i].0.clone();
        run(&mut se, &hy, &mut work, &mut ranked, &mut deepest);
    }
    // refinement around the best feasible oblique: tilt ±10 deg, then azimuth ±20 deg
    let best_oblique = ranked.iter().filter(|c| c.hy.tilt_deg > 0.0).max_by(|a, b| a.scores.total.total_cmp(&b.scores.total).then(b.hy.index.cmp(&a.hy.index))).map(|c| c.hy.clone());
    if let Some(b) = best_oblique {
        let ch = &chords[b.chord];
        let h0 = crate::grasp_select::geometry::unit2([b.a[0], b.a[1]]);
        let rot = |h: geometry::V2, deg: f64| { let (sn, cs) = deg.to_radians().sin_cos(); [h[0] * cs - h[1] * sn, h[0] * sn + h[1] * cs] };
        let variants = [(b.tilt_deg + 10.0, h0), ((b.tilt_deg - 10.0).max(10.0), h0), (b.tilt_deg, rot(h0, 20.0)), (b.tilt_deg, rot(h0, -20.0))];
        let mut next = 10_000 + b.index * 10;
        for (tilt, h) in variants.into_iter().take(REFINE) {
            if se.out_of_budget() { let n = ranked.len(); se.stats.work = work; se.stats.lattice_tests = cheap.lattice_tests; return se.exhausted(n, ev); }
            next += 1;
            se.stats.refined += 1;
            match candidates::make(s, &t, ch, b.chord, (b.class, tilt.min(80.0), h), b.half_turn, b.deep, next, &mut cheap) {
                Ok(hy) => {
                    let seeds = [ranked.iter().find(|c| c.hy.index == b.index).map_or(acq.q_end, |c| c.checked.q_grasp), acq.q_end];
                    match feasibility::endpoint_ik(&s.input, &hy.grasp_pose(), &seeds, &mut work) {
                        Ok(_) => run(&mut se, &hy, &mut work, &mut ranked, &mut deepest),
                        Err((ep, _)) => se.reject(&hy, Rejection::new("endpoint_ik", "IK_UNREACHABLE", format!("refined contact pose: IK error {ep:.4} m")).with(ep, kinematics::PAR.tol_pos), true),
                    }
                }
                Err(b) => { let (hy, r) = *b; se.reject(&hy, r, true) }
            }
        }
    }
    se.stats.work = work;
    se.stats.lattice_tests = cheap.lattice_tests;
    se.stats.fully_checked = ranked.len();
    ranked.sort_by(|a, b| {
        // candidates resting on the assumed payload bound rank after the others
        let ia = a.checked.insufficient_evidence.is_some();
        let ib = b.checked.insufficient_evidence.is_some();
        ia.cmp(&ib).then(b.scores.total.total_cmp(&a.scores.total)).then(a.hy.index.cmp(&b.hy.index))
    });
    match ranked.first() {
        Some(c) if c.checked.insufficient_evidence.is_none() => se.finish(Outcome::Selected, None, None, ranked, ev),
        Some(c) => {
            let m = format!("insufficient evidence: {}", c.checked.insufficient_evidence.clone().unwrap_or_default());
            se.finish(Outcome::InsufficientEvidence, Some("INSUFFICIENT_EVIDENCE".into()), Some(m), ranked, ev)
        }
        None => {
            // the deepest failure of the path checks, else the most frequent cheap one
            let (code, m) = match deepest {
                Some((_, r)) => (r.code.clone(), format!("no candidate passed; deepest failure in {}: {}", r.phase, r.message)),
                None => {
                    let top = se.rejected.iter().max_by(|a, b| phase_rank(a.rejection.phase, false).cmp(&phase_rank(b.rejection.phase, false)).then(b.hy.index.cmp(&a.hy.index)));
                    match top {
                        Some(r) => (r.rejection.code.clone(), format!("no candidate passed; deepest failure in {}: {}", r.rejection.phase, r.rejection.message)),
                        None => ("NO_CANDIDATE".into(), "no contact chord on the footprint".into()),
                    }
                }
            };
            se.finish(Outcome::Infeasible, Some(code), Some(m), ranked, ev)
        }
    }
}

fn pose_json(p: &Pose) -> Value {
    json!({"p": p.p, "quat_wxyz": kinematics::r_to_quat(&p.r), "rpy": kinematics::r_to_rpy(&p.r)})
}

fn r6(q: &[f64; 6]) -> Vec<f64> { q.iter().map(|v| (v * 1e6).round() / 1e6).collect() }

fn candidate_json(s: &Snapshot, c: &Candidate, with_plan: bool) -> Value {
    let hy = &c.hy;
    let mut v = json!({
        "id": c.id,
        "approach_class": hy.class.name(),
        "tilt_deg": hy.tilt_deg,
        "jaw_half_turn": hy.half_turn,
        "station": c.chord.station,
        "depth": if hy.deep { "deep" } else { "shallow" },
        "chord": {"centre_m": c.chord.centre, "dir": c.chord.dir, "width_m": hy.width_m, "opposition": c.chord.opposition, "moment_q": c.chord.moment_q, "pad_support": c.chord.pad_support},
        "contact": {"pad_overlap_m": hy.pad_overlap_m, "opening_reserve_m": hy.opening_reserve_m, "jaw_elevation_deg": hy.jaw_elevation_deg},
        "scores": {"contact": c.scores.contact, "robustness": c.scores.robustness, "margins": c.scores.margins, "tie_break": c.scores.tie_break, "preference": c.scores.preference, "total": c.scores.total},
        "robustness": {"worst_opening_reserve_m": c.robustness.worst_opening_reserve_m, "worst_pad_overlap_m": c.robustness.worst_pad_overlap_m,
            "worst_table_clearance_m": c.robustness.worst_table_clearance_m, "perturbed_ik_ok": c.robustness.ik_ok, "perturbed_ik_tried": c.robustness.ik_tried},
        "margins": {"joint_limit": c.checked.margins.joint_limit, "sigma_min_m": c.checked.margins.sigma_min, "jacobian_char_length_m": feasibility::JACOBIAN_CHAR_LENGTH_M,
            "table_clearance_m": c.checked.margins.table_clearance_m, "payload_capacity_kg": c.checked.margins.payload_capacity_kg},
        "duration_s": c.checked.duration_s,
        "joint_travel_rad": c.checked.joint_travel_rad,
        "visible_at_pregrasp": c.visible_at_pregrasp,
        "insufficient_evidence": c.checked.insufficient_evidence,
    });
    if with_plan {
        let p = c.plan(s);
        v["plan"] = json!({
            "frame": "tcp",
            "pregrasp": pose_json(&p.pregrasp),
            "insertion_dir": p.insertion_dir,
            "insertion_m": p.insertion_m,
            "grasp": pose_json(&p.grasp),
            "jaw_opening_m": p.jaw_opening_m,
            "chord_width_m": p.chord_width_m,
            "separation": {"tcp": pose_json(&p.separation), "distance_m": feasibility::SEPARATION_M, "direction": [0.0, 0.0, 1.0]},
            "lift": pose_json(&p.lift),
            "withdrawal": {"to": "pregrasp", "direction": p.insertion_dir.map(|x| -x)},
            "joints": {"pregrasp": r6(&p.q_pregrasp), "grasp": r6(&p.q_grasp), "lift": r6(&p.q_lift)},
        });
    }
    v
}

impl Selection {
    /// Reply fields of the adaptive dry run (merged into the command ack).
    pub fn to_json(&self, s: &Snapshot) -> Value {
        let selected = self.selected().map(|c| {
            let mut v = candidate_json(s, c, true);
            v["checked_phases"] = json!(c.checked.phases);
            v
        });
        // alternatives: the best other candidate per class, then the shortlisted rejections
        let mut alts: Vec<Value> = Vec::new();
        let mut classes = Vec::new();
        for c in self.ranked.iter().skip(usize::from(self.outcome == Outcome::Selected)) {
            if alts.len() >= MAX_ALTERNATIVES / 2 { break; }
            if classes.contains(&c.hy.class) { continue; }
            classes.push(c.hy.class);
            let mut v = candidate_json(s, c, false);
            v["status"] = json!("feasible");
            alts.push(v);
        }
        for r in self.rejected.iter().filter(|r| r.shortlisted) {
            if alts.len() >= MAX_ALTERNATIVES { break; }
            alts.push(json!({"id": r.id, "status": "rejected", "approach_class": r.hy.class.name(), "tilt_deg": r.hy.tilt_deg, "jaw_half_turn": r.hy.half_turn,
                "phase": r.rejection.phase, "code": r.rejection.code, "message": r.rejection.message, "value": r.rejection.value, "bound": r.rejection.bound,
                "geometry": r.rejection.geometry, "s": r.rejection.s}));
        }
        json!({
            "strategy": "adaptive",
            "outcome": self.outcome.name(),
            "snapshot_id": self.snapshot_id,
            "selected_candidate": selected,
            "checked_phases": self.selected().map(|c| c.checked.phases.clone()).unwrap_or_default(),
            "margins": self.selected().map(|c| json!({"joint_limit": c.checked.margins.joint_limit, "sigma_min_m": c.checked.margins.sigma_min,
                "table_clearance_m": c.checked.margins.table_clearance_m, "opening_reserve_m": c.robustness.worst_opening_reserve_m,
                "pad_overlap_m": c.robustness.worst_pad_overlap_m, "payload_capacity_kg": c.checked.margins.payload_capacity_kg})),
            "evidence": self.evidence,
            "rejections": self.rejections_by_phase,
            "rejection_codes": self.rejections_by_code,
            "alternatives": alts,
            "search": {
                "chords": self.stats.chords, "generated": self.stats.generated, "geometric_survivors": self.stats.geometric_survivors,
                "ik_survivors": self.stats.ik_survivors, "shortlisted": self.stats.shortlisted, "refined": self.stats.refined,
                "fully_checked": self.stats.fully_checked, "ik_iterations": self.stats.work.ik_iterations, "endpoint_solves": self.stats.work.endpoint_solves,
                "path_checks": self.stats.work.path_checks, "path_samples": self.stats.work.path_samples, "lattice_tests": self.stats.lattice_tests,
                "elapsed_ms": self.elapsed_ms, "budget_ms": self.budget_ms,
                "quotas": {"shortlist": SHORTLIST, "reserved_radial_oblique": RESERVED_RADIAL_OBLIQUE, "max_per_class": MAX_PER_CLASS, "refine": REFINE},
            },
        })
    }
}
