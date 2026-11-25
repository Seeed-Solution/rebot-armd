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

pub use snapshot::{capture, capture_from_env, Snapshot};
use candidates::{Chord, DirClass, Hypothesis, Rejection};
pub use feasibility::{
    CarriedObjectBox, Checked, Margins, Work, BRANCH_CONSISTENCY_TOL_RAD, CONTACT_LATERAL_TOL_M,
    CONTACT_ROT_TOL_RAD, CONTACT_WINDOW_M,
};
use score::{Robustness, Scores};
use crate::kinematics::{self, Pose};
use crate::motion::{self, GripperCommand, MotionError};

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

/// Snapshot of all five owner configuration revisions captured at adaptive grasp admission and
/// fenced on `BeginMacro`, every `InstallChild`, post-lift holding observation, and final
/// `FinishMacro(Done, HeldUpdate::Set)` commit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdaptiveRevisions {
    pub workspace_revision: u64,
    pub calibration_revision: u64,
    pub grasp_setup_revision: u64,
    pub gripper_calib_revision: u64,
    pub power_budget_revision: u64,
}

/// How the adaptive route transitions from actual start `q0` to `pregrasp`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PregraspMode {
    /// Straight Cartesian TCP path from `q0` to `pregrasp` staying on the selected IK branch.
    Cartesian,
    /// Swept joint-space quintic `JointPath` from `q0` to `q_pregrasp` (used when a straight
    /// Cartesian move would flip the IK branch or fail IK), J7 strictly locked.
    SweptJoint,
}

impl PregraspMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Cartesian => "cartesian",
            Self::SweptJoint => "swept_joint",
        }
    }
}

/// Scheduled time budget (s) derived from the exact checked legs and shared gripper estimator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteBudget {
    pub pregrasp_s: f64,
    pub start_vision_s: f64,
    pub pre_open_s: f64,
    pub pregrasp_vision_s: f64,
    pub insert_s: f64,
    pub close_s: f64,
    pub close_timeout_s: f64,
    pub is_pos_force_close: bool,
    pub separation_s: f64,
    pub lift_s: f64,
    pub withdrawal_s: f64,
    pub post_lift_settle_s: f64,
    pub nominal_total_s: f64,
    pub recovery_total_s: f64,
}

/// Shared gripper duration / timeout estimator using the EXACT same
/// [`motion::GripperMotion::try_with_close_speed_limit`] calculation installed by the owner.
/// Returns `(expected_duration_s, motion_timeout_s, is_pos_force_close)`.
pub fn estimate_gripper_duration_s(
    policy: &crate::model::GripperPolicy,
    limit: &crate::model::JointLimit,
    start_j7: f32,
    close_speed_limit: Option<f32>,
    command: GripperCommand,
    fraction: Option<f64>,
    grasp: bool,
) -> Result<(f64, f64, bool), MotionError> {
    let closes = match command {
        GripperCommand::Close | GripperCommand::Seek => true,
        GripperCommand::Open => false,
        GripperCommand::Move => {
            let target =
                policy.closed_rad + fraction.unwrap_or(0.0) as f32 * (policy.open_rad - policy.closed_rad);
            let dir = if policy.closed_rad >= policy.open_rad { 1.0 } else { -1.0 };
            (target - start_j7) * dir > 1e-3
        }
    };
    let (speed, timeout_s) = if closes {
        (policy.close_speed(), None)
    } else {
        (1.0, Some(8.0))
    };
    let req = motion::GripperRequest {
        command,
        fraction,
        grasp,
        speed,
        timeout_s,
        tag: None,
    };
    let mut mot = motion::GripperMotion::try_with_close_speed_limit(
        req,
        policy.clone(),
        limit.clone(),
        start_j7,
        0,
        close_speed_limit,
    )?;
    let is_pf = close_speed_limit.is_some() && mot.is_closing();
    if is_pf && mot.request.timeout_s.is_some() {
        mot.use_default_timeout();
    }
    Ok((mot.expected_duration(), mot.timeout_s, is_pf))
}

/// Single authoritative executable route for Phase B adaptive grasp: drives adaptive dry-run
/// reporting, pre-`BeginMacro` whole-route preflight & budget admission, [`SelectedGraspMacro`]
/// construction, and runtime `Orchestrator::run_adaptive` execution.
#[derive(Debug, Clone)]
pub struct AdaptiveRoute {
    pub candidate_id: String,
    pub approach_class: &'static str,
    pub tilt_deg: f64,
    pub plan: ExecutionPlan,
    pub pregrasp_mode: PregraspMode,
    /// Conservative TCP-frame object volume (includes horizontal `hard_error_m`, vertical
    /// `TABLE_ERROR_M`, and early-contact offset when `descend_contact_stop_nm` is armed).
    pub obj_box_tcp: CarriedObjectBox,
    pub target_centroid: [f64; 2],
    pub target_table_z: f64,
    pub target_height: [f64; 2],
    pub target_poly: Vec<[f64; 2]>,
    pub target_axis: [f64; 2],
    pub target_axis_ambiguous: bool,
    pub target_hard_error_m: f64,
    pub opening_reserve_m: f64,
    pub contact_window_m: f64,
    pub visible_at_pregrasp: bool,
    pub pre_open_command: GripperCommand,
    pub pre_open_fraction: Option<f64>,
    pub budget: RouteBudget,
    pub contact_guard: crate::rt_core::ops::ContactGuard,
    pub contact_stop_armed: bool,
    pub checked_phases: Vec<&'static str>,
    pub margins: Margins,
    pub robustness: Robustness,
    pub scores: Scores,
    pub approach_mps: f64,
    pub insert_mps: f64,
    pub loaded_mps: f64,
    pub has_oriented_evidence: bool,
}

impl AdaptiveRoute {
    /// Builds P's [`SelectedGraspMacro`] directly from this authoritative route, binding all
    /// five captured revisions and exact `expected_duration_s`.
    #[allow(clippy::too_many_arguments)]
    pub fn into_selected_macro(
        &self,
        prepared_boot: crate::rt_core::broker::BootId,
        start: &[f32],
        deadline: Instant,
        tag: Option<&str>,
        task: Option<&str>,
        revisions: AdaptiveRevisions,
        admission_request_id: u64,
    ) -> Result<SelectedGraspMacro, &'static str> {
        use crate::rt_core::requests::actions::{BeginMacroSpec, MacroKind};

        let mut begin = BeginMacroSpec::new(MacroKind::Grasp, prepared_boot).with_start(start)?;
        begin.admission_request_id = admission_request_id;
        begin.check_workspace_revision = true;
        begin.calibration_revision = Some(revisions.calibration_revision);
        begin.grasp_setup_revision = Some(revisions.grasp_setup_revision);
        begin.gripper_calib_revision = Some(revisions.gripper_calib_revision);
        begin.power_budget_revision = Some(revisions.power_budget_revision);
        begin.macro_deadline = Some(deadline);
        begin.timeout_code = crate::safety::BoundedText::new("GRASP_TIMEOUT");
        if let Some(tag) = tag {
            begin.tag = crate::model::HolderLabel::new(tag);
        }
        if let Some(task) = task {
            begin.task = crate::model::HolderLabel::new(task);
        }
        let steps = vec![
            SelectedMacroStep::MoveTcp {
                phase: "pregrasp",
                goal: self.plan.pregrasp,
                linear_mps: self.approach_mps,
                contact: None,
            },
            SelectedMacroStep::Gripper {
                phase: "pre_open",
                command: self.pre_open_command,
                fraction: self.pre_open_fraction,
                grasp: false,
            },
            SelectedMacroStep::MoveTcp {
                phase: "insert",
                goal: self.plan.grasp,
                linear_mps: self.insert_mps,
                contact: Some(self.contact_guard),
            },
            SelectedMacroStep::Gripper {
                phase: "close",
                command: GripperCommand::Close,
                fraction: None,
                grasp: true,
            },
            SelectedMacroStep::MoveTcp {
                phase: "separation",
                goal: self.plan.separation,
                linear_mps: self.loaded_mps,
                contact: None,
            },
            SelectedMacroStep::MoveTcp {
                phase: "lift",
                goal: self.plan.lift,
                linear_mps: self.loaded_mps,
                contact: None,
            },
        ];
        Ok(SelectedGraspMacro {
            begin,
            steps,
            expected_duration_s: self.budget.nominal_total_s,
        })
    }
}

/// Result of Phase B actual-start selection + whole-route preflight ([`select_adaptive_route`]).
#[derive(Debug, Clone)]
pub struct AdaptiveSelection {
    pub selection: Selection,
    pub route: Option<AdaptiveRoute>,
    pub failing_phase: Option<&'static str>,
    /// Checked-alternative hint populated ONLY when an actually checked alternative exists
    /// (e.g. an `InsufficientEvidence` candidate with a verified payload capacity bound).
    pub checked_hint: Option<String>,
}

impl AdaptiveSelection {
    pub fn reachable(&self) -> bool {
        self.selection.outcome == Outcome::Selected && self.route.is_some()
    }

    /// Adaptive dry-run reply JSON (parity with Phase A fields plus authoritative Phase B
    /// route summary, `failing_phase`, and checked-alternative `hint` only when verified).
    pub fn to_json(
        &self,
        s: &Snapshot,
        legacy_verdict: Option<(bool, Option<&str>, Option<&str>)>,
    ) -> Value {
        let mut v = self.selection.to_json(s);
        if let Some(route) = &self.route {
            v["expected_duration_s"] = json!(route.budget.nominal_total_s);
            v["pregrasp_mode"] = json!(route.pregrasp_mode.name());
            v["checked_phases"] = json!(route.checked_phases);
            if let Some(sc) = v.get_mut("selected_candidate").and_then(Value::as_object_mut) {
                sc.insert("checked_phases".into(), json!(route.checked_phases));
                sc.insert("pregrasp_mode".into(), json!(route.pregrasp_mode.name()));
                sc.insert("expected_duration_s".into(), json!(route.budget.nominal_total_s));
            }
        }
        v["failing_phase"] = json!(self.failing_phase);
        let hint = self.checked_hint.clone().or_else(|| {
            if !self.reachable() {
                if let Some((true, _, _)) = legacy_verdict {
                    return Some("legacy strategy passed dry_run for this target".to_string());
                }
            }
            None
        });
        v["hint"] = json!(hint);
        v
    }
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
const PHASE_ORDER: [&str; 13] = ["evidence", "geometry", "table", "pre_open", "insertion", "closure", "endpoint_ik", "acquisition", "transition", "separation", "lift", "withdrawal", "budget"];

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

/// Whether every vertex of `poly` (base xy) at mid-height `z_mid` projects inside the camera
/// image with a 4 px inset when the TCP is at `tcp`.
pub fn visible_at_pose(
    cam: &crate::grasp::CamModel,
    tcp_offset: &[f64; 3],
    poly: &[[f64; 2]],
    z_mid: f64,
    tcp: &Pose,
) -> bool {
    let end = crate::motion::tcp_to_end(tcp, tcp_offset);
    let m = 4.0;
    poly.iter().all(|v| {
        cam.project(&end, &[v[0], v[1], z_mid])
            .is_some_and(|p| p[0] >= m && p[1] >= m && p[0] <= cam.size[0] - m && p[1] <= cam.size[1] - m)
    })
}

fn visible(s: &Snapshot, t: &snapshot::Target, tcp: &Pose) -> bool {
    let z = t.table_z + (t.height[0] + t.height[1]) / 4.0;
    visible_at_pose(&s.cam, &s.input.tcp_offset, &t.poly, z, tcp)
}

/// Phase B actual-start selection + whole-route preflight (shared single authority for live
/// adaptive `grasp` AND adaptive `grasp_dry_run`).
///
/// Shares candidate generation and scoring with Phase A [`select`], while evaluating actual-start
/// feasibility directly from `s.input.q0` (direct Cartesian or swept-joint `pregrasp`, calibrated
/// `pre_open`, straight-line TCP `insert`, capped `close`, nominal + contact-window `separation`
/// & `lift` with conservative swept OBB SAT + 8-corner payload torque, closed & open `withdrawal`,
/// and unified per-child + parent deadline admission) under ONE absolute `budget_ms` deadline.
pub fn select_adaptive_route(
    s: &Snapshot,
    gripper_policy: &crate::model::GripperPolicy,
    gripper_limit: &crate::model::JointLimit,
    close_speed_limit: Option<f32>,
    has_gravity_model: bool,
) -> AdaptiveSelection {
    let mut se = Search {
        s,
        start: Instant::now(),
        stats: Stats::default(),
        rejected: Vec::new(),
        by_phase: BTreeMap::new(),
        by_code: BTreeMap::new(),
    };
    let t = match snapshot::target(s) {
        Ok(t) => t,
        Err(m) => {
            *se.by_phase.entry("evidence").or_default() += 1;
            let ev = evidence_json(s, None);
            let sel = se.finish(
                Outcome::Infeasible,
                Some("INVALID_COMMAND".into()),
                Some(format!("evidence: {m}")),
                Vec::new(),
                ev,
            );
            return AdaptiveSelection {
                selection: sel,
                route: None,
                failing_phase: Some("evidence"),
                checked_hint: None,
            };
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
    for (hy, r) in hyp_rej {
        se.reject(&hy, r, false);
    }
    if se.out_of_budget() {
        se.stats.lattice_tests = cheap.lattice_tests;
        return AdaptiveSelection {
            selection: se.exhausted(0, ev),
            route: None,
            failing_phase: Some("budget"),
            checked_hint: None,
        };
    }

    // Check acquisition as a reference seed / diagnostic only — NEVER hard-blocking direct-from-q0
    // candidates when acquisition itself is blocked (e.g. overhead keep-out box above target).
    let mut work = Work::default();
    let acq_res = feasibility::acquisition(s, &mut work);
    let acq_seed = acq_res.as_ref().ok().map(|a| a.q_end);

    // Seed endpoint IK from cache, actual start q0, and optional acq_seed.
    let mut seed_cache: BTreeMap<(DirClass, bool), [f64; 6]> = BTreeMap::new();
    let mut screened: Vec<(Hypothesis, [f64; 6], [f64; 6], f64)> = Vec::new();
    for hy in hyps {
        if se.out_of_budget() {
            se.stats.work = work;
            se.stats.lattice_tests = cheap.lattice_tests;
            return AdaptiveSelection {
                selection: se.exhausted(0, ev),
                route: None,
                failing_phase: Some("budget"),
                checked_hint: None,
            };
        }
        let mut seeds = Vec::with_capacity(3);
        if let Some(q) = seed_cache.get(&(hy.class, hy.half_turn)) {
            seeds.push(*q);
        }
        seeds.push(s.input.q0);
        if let Some(qa) = acq_seed {
            seeds.push(qa);
        }
        let q_g = match feasibility::endpoint_ik(&s.input, &hy.grasp_pose(), &seeds, &mut work) {
            Ok(q) => q,
            Err((ep, er)) => {
                se.reject(
                    &hy,
                    Rejection::new(
                        "endpoint_ik",
                        "IK_UNREACHABLE",
                        format!("contact pose: best IK error {:.4} m / {:.2} deg", ep, er.to_degrees()),
                    )
                    .with(ep, kinematics::PAR.tol_pos),
                    false,
                );
                continue;
            }
        };
        let mut pre_seeds = vec![q_g, s.input.q0];
        if let Some(qa) = acq_seed {
            pre_seeds.push(qa);
        }
        let q_pre_seed = match feasibility::endpoint_ik(&s.input, &hy.pregrasp_pose(), &pre_seeds, &mut work) {
            Ok(q) => q,
            Err((ep, er)) => {
                se.reject(
                    &hy,
                    Rejection::new(
                        "endpoint_ik",
                        "IK_UNREACHABLE",
                        format!("pre-grasp pose: best IK error {:.4} m / {:.2} deg", ep, er.to_degrees()),
                    )
                    .with(ep, kinematics::PAR.tol_pos),
                    false,
                );
                continue;
            }
        };
        seed_cache.insert((hy.class, hy.half_turn), q_g);
        let p = prescore(s, &t, &chords[hy.chord], &hy, &q_g);
        screened.push((hy, q_g, q_pre_seed, p));
    }
    se.stats.ik_survivors = screened.len();

    // If acquisition failed AND no candidate even passed endpoint IK (wholly unreachable target,
    // e.g. table 0.30 m below base), preserve the exact acquisition failure diagnostics.
    if screened.is_empty() {
        if let Err(r) = acq_res {
            se.stats.work = work;
            se.stats.lattice_tests = cheap.lattice_tests;
            *se.by_phase.entry("acquisition").or_default() += 1;
            let m = format!(
                "acquisition: {} (a different contact pose cannot fix the observation pose)",
                r.message
            );
            let sel = se.finish(Outcome::Infeasible, Some(r.code), Some(m), Vec::new(), ev);
            return AdaptiveSelection {
                selection: sel,
                route: None,
                failing_phase: Some("acquisition"),
                checked_hint: None,
            };
        }
    }

    screened.sort_by(|a, b| b.3.total_cmp(&a.3).then(a.0.index.cmp(&b.0.index)));
    let mut short: Vec<usize> = screened
        .iter()
        .enumerate()
        .filter(|(_, x)| x.0.class.is_radial_oblique())
        .map(|(i, _)| i)
        .take(RESERVED_RADIAL_OBLIQUE)
        .collect();
    for (i, x) in screened.iter().enumerate() {
        if short.len() >= SHORTLIST {
            break;
        }
        if short.contains(&i) {
            continue;
        }
        let n_class = short
            .iter()
            .filter(|&&j| screened[j].0.class == x.0.class)
            .count();
        if n_class < MAX_PER_CLASS {
            short.push(i);
        }
    }
    short.sort_unstable();
    se.stats.shortlisted = short.len();

    let acq_ok = acq_res.is_ok();
    let mut entries: Vec<(Candidate, AdaptiveRoute)> = Vec::new();
    let mut deepest: Option<(usize, Rejection)> = None;

    let evaluate = |se: &mut Search,
                        hy: &Hypothesis,
                        q_pre_seed: [f64; 6],
                        work: &mut Work,
                        entries: &mut Vec<(Candidate, AdaptiveRoute)>,
                        deepest: &mut Option<(usize, Rejection)>| {
        match preflight_candidate_from_start(
            s,
            &t,
            hy,
            q_pre_seed,
            acq_ok,
            gripper_policy,
            gripper_limit,
            close_speed_limit,
            has_gravity_model,
            work,
        ) {
            Ok((checked, mut route)) => {
                let rob = score::robustness(s, &t, hy, &checked, work);
                let ch = chords[hy.chord].clone();
                let scores = score::score(s, &t, &ch, hy, &checked, &rob);
                let vis = visible(s, &t, &hy.pregrasp_pose());
                let cid = se.id(hy);
                route.candidate_id = cid.clone();
                route.robustness = rob;
                route.scores = scores;
                route.visible_at_pregrasp = vis;
                let cand = Candidate {
                    id: cid,
                    hy: hy.clone(),
                    chord: ch,
                    checked,
                    robustness: rob,
                    scores,
                    visible_at_pregrasp: vis,
                };
                entries.push((cand, route));
            }
            Err(r) => {
                let rank = phase_rank(r.phase, true);
                if deepest.as_ref().is_none_or(|(d, _)| rank > *d) {
                    *deepest = Some((rank, r.clone()));
                }
                se.reject(hy, r, true);
            }
        }
    };

    for &i in &short {
        if se.out_of_budget() {
            let n = entries.len();
            se.stats.work = work;
            se.stats.lattice_tests = cheap.lattice_tests;
            return AdaptiveSelection {
                selection: se.exhausted(n, ev),
                route: None,
                failing_phase: Some("budget"),
                checked_hint: None,
            };
        }
        let (hy, _, q_pre_seed, _) = &screened[i];
        evaluate(&mut se, hy, *q_pre_seed, &mut work, &mut entries, &mut deepest);
    }

    // Refinement around best feasible oblique (same schedule as Phase A).
    let best_oblique = entries
        .iter()
        .filter(|(c, _)| c.hy.tilt_deg > 0.0)
        .max_by(|a, b| {
            a.0.scores
                .total
                .total_cmp(&b.0.scores.total)
                .then(b.0.hy.index.cmp(&a.0.hy.index))
        })
        .map(|(c, _)| (c.hy.clone(), c.checked.q_grasp, c.checked.q_pre));
    if let Some((b, b_qg, b_qp)) = best_oblique {
        let ch = &chords[b.chord];
        let h0 = geometry::unit2([b.a[0], b.a[1]]);
        let rot = |h: geometry::V2, deg: f64| {
            let (sn, cs) = deg.to_radians().sin_cos();
            [h[0] * cs - h[1] * sn, h[0] * sn + h[1] * cs]
        };
        let variants = [
            (b.tilt_deg + 10.0, h0),
            ((b.tilt_deg - 10.0).max(10.0), h0),
            (b.tilt_deg, rot(h0, 20.0)),
            (b.tilt_deg, rot(h0, -20.0)),
        ];
        let mut next = 10_000 + b.index * 10;
        for (tilt, h) in variants.into_iter().take(REFINE) {
            if se.out_of_budget() {
                let n = entries.len();
                se.stats.work = work;
                se.stats.lattice_tests = cheap.lattice_tests;
                return AdaptiveSelection {
                    selection: se.exhausted(n, ev),
                    route: None,
                    failing_phase: Some("budget"),
                    checked_hint: None,
                };
            }
            next += 1;
            se.stats.refined += 1;
            match candidates::make(
                s,
                &t,
                ch,
                b.chord,
                (b.class, tilt.min(80.0), h),
                b.half_turn,
                b.deep,
                next,
                &mut cheap,
            ) {
                Ok(hy) => {
                    let seeds = [b_qg, s.input.q0];
                    match feasibility::endpoint_ik(&s.input, &hy.grasp_pose(), &seeds, &mut work) {
                        Ok(q_g) => {
                            let pre_seeds = [b_qp, q_g, s.input.q0];
                            match feasibility::endpoint_ik(
                                &s.input,
                                &hy.pregrasp_pose(),
                                &pre_seeds,
                                &mut work,
                            ) {
                                Ok(q_pre_seed) => evaluate(
                                    &mut se,
                                    &hy,
                                    q_pre_seed,
                                    &mut work,
                                    &mut entries,
                                    &mut deepest,
                                ),
                                Err((ep, _)) => se.reject(
                                    &hy,
                                    Rejection::new(
                                        "endpoint_ik",
                                        "IK_UNREACHABLE",
                                        format!("refined pre-grasp pose: IK error {ep:.4} m"),
                                    )
                                    .with(ep, kinematics::PAR.tol_pos),
                                    true,
                                ),
                            }
                        }
                        Err((ep, _)) => se.reject(
                            &hy,
                            Rejection::new(
                                "endpoint_ik",
                                "IK_UNREACHABLE",
                                format!("refined contact pose: IK error {ep:.4} m"),
                            )
                            .with(ep, kinematics::PAR.tol_pos),
                            true,
                        ),
                    }
                }
                Err(b) => {
                    let (hy, r) = *b;
                    se.reject(&hy, r, true);
                }
            }
        }
    }

    se.stats.work = work;
    se.stats.lattice_tests = cheap.lattice_tests;
    se.stats.fully_checked = entries.len();
    entries.sort_by(|a, b| {
        let ia = a.0.checked.insufficient_evidence.is_some();
        let ib = b.0.checked.insufficient_evidence.is_some();
        ia.cmp(&ib)
            .then(b.0.scores.total.total_cmp(&a.0.scores.total))
            .then(a.0.hy.index.cmp(&b.0.hy.index))
    });
    let ranked: Vec<Candidate> = entries.iter().map(|(c, _)| c.clone()).collect();
    match entries.first() {
        Some((c, route)) if c.checked.insufficient_evidence.is_none() => {
            let route = route.clone();
            let sel = se.finish(Outcome::Selected, None, None, ranked, ev);
            AdaptiveSelection {
                selection: sel,
                route: Some(route),
                failing_phase: None,
                checked_hint: None,
            }
        }
        Some((c, _)) => {
            let m = format!(
                "insufficient evidence: {}",
                c.checked.insufficient_evidence.clone().unwrap_or_default()
            );
            let hint = c.checked.margins.payload_capacity_kg.filter(|cap| *cap >= 0.0).map(|cap| {
                format!(
                    "checked candidate {} supports payload up to {:.3} kg; pass evidence.payload_kg <= {:.3}",
                    c.id, cap, cap
                )
            });
            let sel = se.finish(
                Outcome::InsufficientEvidence,
                Some("INSUFFICIENT_EVIDENCE".into()),
                Some(m),
                ranked,
                ev,
            );
            AdaptiveSelection {
                selection: sel,
                route: None,
                failing_phase: Some("lift"),
                checked_hint: hint,
            }
        }
        None => {
            let (phase, code, m) = match deepest {
                Some((_, r)) => (
                    r.phase,
                    r.code.clone(),
                    format!("no candidate passed; deepest failure in {}: {}", r.phase, r.message),
                ),
                None => {
                    let top = se.rejected.iter().max_by(|a, b| {
                        phase_rank(a.rejection.phase, false)
                            .cmp(&phase_rank(b.rejection.phase, false))
                            .then(b.hy.index.cmp(&a.hy.index))
                    });
                    match top {
                        Some(r) => (
                            r.rejection.phase,
                            r.rejection.code.clone(),
                            format!(
                                "no candidate passed; deepest failure in {}: {}",
                                r.rejection.phase, r.rejection.message
                            ),
                        ),
                        None => (
                            "geometry",
                            "NO_CANDIDATE".into(),
                            "no contact chord on the footprint".into(),
                        ),
                    }
                }
            };
            let sel = se.finish(Outcome::Infeasible, Some(code), Some(m), ranked, ev);
            AdaptiveSelection {
                selection: sel,
                route: None,
                failing_phase: Some(phase),
                checked_hint: None,
            }
        }
    }
}

/// Whole-route actual-start preflight of one hypothesis `hy` directly from `s.input.q0`.
#[allow(clippy::too_many_arguments)]
fn preflight_candidate_from_start(
    s: &Snapshot,
    t: &snapshot::Target,
    hy: &Hypothesis,
    q_pre_seed: [f64; 6],
    acq_ok: bool,
    gripper_policy: &crate::model::GripperPolicy,
    gripper_limit: &crate::model::JointLimit,
    close_speed_limit: Option<f32>,
    has_gravity_model: bool,
    work: &mut Work,
) -> Result<(Checked, AdaptiveRoute), Rejection> {
    let st = &s.setup;
    let pre = hy.pregrasp_pose();
    let grasp_pose = hy.grasp_pose();
    let table_bound_z = st.table_z_m + crate::grasp::TABLE_CLEARANCE_M;
    let q6_of = |q: &[f32; 7]| std::array::from_fn::<f64, 6, _>(|i| f64::from(q[i]));

    // 1. Pregrasp from actual start q0 (J7 locked at s.input.j7): direct Cartesian if branch-
    //    consistent, else swept joint-space quintic JointPath to q_pre_seed.
    let (pregrasp_path, pregrasp_mode) = match feasibility::phase(
        s,
        "transition",
        s.input.q0,
        s.input.j7,
        pre,
        st.approach_speed_mps,
        work,
    ) {
        Ok(p) => {
            let q_end = q6_of(&p.last());
            // Verify insertion from this Cartesian end succeeds without branch flip; otherwise try swept joint.
            if feasibility::phase(s, "insertion", q_end, s.open_rad, grasp_pose, st.descend_speed_mps, work).is_ok() {
                (p, PregraspMode::Cartesian)
            } else {
                let inp = feasibility::input_at(&s.input, s.input.q0, s.input.j7);
                work.path_checks += 1;
                let jp = feasibility::plan_swept_joint_path(&inp, q_pre_seed, &s.envelope, table_bound_z)
                    .map_err(|e| feasibility::motion_rejection("transition", &e))?;
                work.path_samples += jp.q.len();
                (jp, PregraspMode::SweptJoint)
            }
        }
        Err(cart_rej) => {
            let inp = feasibility::input_at(&s.input, s.input.q0, s.input.j7);
            work.path_checks += 1;
            match feasibility::plan_swept_joint_path(&inp, q_pre_seed, &s.envelope, table_bound_z) {
                Ok(jp) => {
                    work.path_samples += jp.q.len();
                    (jp, PregraspMode::SweptJoint)
                }
                Err(_) => return Err(cart_rej),
            }
        }
    };
    if pregrasp_path.duration + 0.5 > st.phase_timeout_s {
        return Err(Rejection::new(
            "transition",
            "GRASP_TIMEOUT",
            format!(
                "pregrasp duration {:.2} s + settle exceeds phase_timeout_s {:.2} s",
                pregrasp_path.duration, st.phase_timeout_s
            ),
        ));
    }
    let q_pre = q6_of(&pregrasp_path.last());

    // 2. Pre-open budget (shared GripperMotion estimator)
    let (pre_open_cmd, pre_open_frac) = (GripperCommand::Open, None);
    let (pre_open_s, _, _) = estimate_gripper_duration_s(
        gripper_policy,
        gripper_limit,
        s.input.j7,
        close_speed_limit,
        pre_open_cmd,
        pre_open_frac,
        false,
    )
    .map_err(|e| feasibility::motion_rejection("pre_open", &e))?;
    if pre_open_s > st.phase_timeout_s {
        return Err(Rejection::new(
            "pre_open",
            "GRASP_TIMEOUT",
            format!("pre-open duration {:.2} s exceeds phase_timeout_s {:.2} s", pre_open_s, st.phase_timeout_s),
        ));
    }

    // 3. Insertion: straight TCP line along hy.a at fixed orientation, reduced speed
    let insertion = feasibility::phase(
        s,
        "insertion",
        q_pre,
        s.open_rad,
        grasp_pose,
        st.descend_speed_mps,
        work,
    )?;
    if insertion.duration + 0.5 > st.phase_timeout_s {
        return Err(Rejection::new(
            "insertion",
            "GRASP_TIMEOUT",
            format!(
                "insertion duration {:.2} s + settle exceeds phase_timeout_s {:.2} s",
                insertion.duration, st.phase_timeout_s
            ),
        ));
    }
    let q_grasp = q6_of(&insertion.last());

    // 4. Close: shared GripperMotion capped pos-force estimator
    let (close_s, close_timeout_s, is_pos_force_close) = estimate_gripper_duration_s(
        gripper_policy,
        gripper_limit,
        s.open_rad,
        close_speed_limit,
        GripperCommand::Close,
        None,
        true,
    )
    .map_err(|e| feasibility::motion_rejection("closure", &e))?;
    if !is_pos_force_close && close_s > st.phase_timeout_s {
        return Err(Rejection::new(
            "closure",
            "GRASP_TIMEOUT",
            format!("close duration {:.2} s exceeds phase_timeout_s {:.2} s", close_s, st.phase_timeout_s),
        ));
    }

    // Contact window + conservative carried-object TCP box
    let contact_stop_armed = st.descend_contact_stop_nm.is_some();
    let contact_window_m = CONTACT_WINDOW_M.min(hy.insertion_m * 0.5);
    let mut obj_box = CarriedObjectBox::from_target_grasp(t, &grasp_pose);
    if contact_stop_armed {
        // When jaws stop up to `contact_window_m` early along -a (TCP -x), the object sits up to
        // `+contact_window_m` further forward along TCP +x, plus accepted lateral error.
        obj_box = obj_box
            .expanded_by_tcp_offset(&[contact_window_m, CONTACT_LATERAL_TOL_M, CONTACT_LATERAL_TOL_M])
            .expanded_by_tcp_offset(&[0.0, -CONTACT_LATERAL_TOL_M, -CONTACT_LATERAL_TOL_M]);
    }

    // 5. Separation + Lift (nominal pose) with conservative swept OBB SAT across BOTH legs
    let sep = Pose::new(hy.r, kinematics::add(&hy.g, &[0.0, 0.0, feasibility::SEPARATION_M]));
    let separation = feasibility::phase(
        s,
        "separation",
        q_grasp,
        s.closed_rad,
        sep,
        st.descend_speed_mps,
        work,
    )?;
    if separation.duration + 0.5 > st.phase_timeout_s {
        return Err(Rejection::new(
            "separation",
            "GRASP_TIMEOUT",
            format!(
                "separation duration {:.2} s + settle exceeds phase_timeout_s {:.2} s",
                separation.duration, st.phase_timeout_s
            ),
        ));
    }
    if let Some(ws) = s.input.ws.as_ref() {
        feasibility::check_carried_object_volume(
            &s.input.arm,
            &s.input.tcp_offset,
            ws,
            &obj_box,
            &separation,
            "separation",
        )?;
    }
    let q_sep = q6_of(&separation.last());
    let lift_z = (st.table_z_m + st.lift_height_m).max(sep.p[2]);
    let lift = Pose::new(hy.r, [hy.g[0], hy.g[1], lift_z]);
    let (lift_path, q_lift) = if lift_z > sep.p[2] + 1e-4 {
        let lp = feasibility::phase(
            s,
            "lift",
            q_sep,
            s.closed_rad,
            lift,
            st.descend_speed_mps,
            work,
        )?;
        if lp.duration + 0.5 > st.phase_timeout_s {
            return Err(Rejection::new(
                "lift",
                "GRASP_TIMEOUT",
                format!(
                    "lift duration {:.2} s + settle exceeds phase_timeout_s {:.2} s",
                    lp.duration, st.phase_timeout_s
                ),
            ));
        }
        if let Some(ws) = s.input.ws.as_ref() {
            feasibility::check_carried_object_volume(
                &s.input.arm,
                &s.input.tcp_offset,
                ws,
                &obj_box,
                &lp,
                "lift",
            )?;
        }
        let q = q6_of(&lp.last());
        (Some(lp), q)
    } else {
        (None, q_sep)
    };

    // 6. Loaded payload torque across initial loaded pose + separation + lift using exact 8-corner CoM box
    let mut loaded: Vec<[f64; 6]> = Vec::with_capacity(1 + separation.q.len() + lift_path.as_ref().map_or(0, |p| p.q.len()));
    loaded.push(q_grasp);
    loaded.extend(separation.q.iter().map(q6_of));
    if let Some(lp) = &lift_path {
        loaded.extend(lp.q.iter().map(q6_of));
    }
    let capacity = feasibility::payload_capacity_box(s, &loaded, &obj_box);
    let mut payload_assumed = false;
    let mut insufficient = None;
    match capacity {
        Some((cap, joint)) => {
            let (bound, given) = match s.evidence.payload_kg {
                Some(m) => (m, true),
                None => (feasibility::ASSUMED_PAYLOAD_KG, false),
            };
            if cap < 0.0 {
                return Err(Rejection::new(
                    "lift",
                    "LIFT_TORQUE",
                    format!("J{} holding torque exceeds its derated limit without payload", joint + 1),
                )
                .with(cap, 0.0));
            }
            if cap < bound {
                if given {
                    return Err(Rejection::new(
                        "lift",
                        "LIFT_TORQUE",
                        format!("J{} allows {cap:.3} kg of payload across CoM envelope, bound is {bound:.3} kg", joint + 1),
                    )
                    .with(cap, bound));
                }
                insufficient = Some(format!(
                    "J{} allows {cap:.3} kg across CoM envelope; payload bound is unknown (assumed {bound:.2} kg)",
                    joint + 1
                ));
            }
            payload_assumed = !given;
        }
        None if has_gravity_model => {
            insufficient = Some("gravity model is configured without derated torque limits; payload capacity cannot be bounded".into());
        }
        None => {}
    }

    // 7. Closed-jaw withdrawal (empty-grasp NOT_HOLDING recovery: jaws stay closed, straight back along -a)
    let withdrawal = feasibility::phase(
        s,
        "withdrawal",
        q_grasp,
        s.closed_rad,
        pre,
        st.descend_speed_mps,
        work,
    )?;
    if withdrawal.duration + 0.5 > st.phase_timeout_s {
        return Err(Rejection::new(
            "withdrawal",
            "GRASP_TIMEOUT",
            format!(
                "withdrawal duration {:.2} s + settle exceeds phase_timeout_s {:.2} s",
                withdrawal.duration, st.phase_timeout_s
            ),
        ));
    }

    // 8. When contact guard is armed, preflight contact-window stations along -a for separation,
    //    lift, closed-jaw withdrawal, AND open-jaw obstruction withdrawal.
    let mut worst_sep_s = separation.duration;
    let mut worst_lift_s = lift_path.as_ref().map_or(0.0, |p| p.duration);
    let mut worst_withdraw_s = withdrawal.duration;
    if contact_stop_armed {
        let open_withdraw = feasibility::phase(
            s,
            "withdrawal",
            q_grasp,
            s.open_rad,
            pre,
            st.descend_speed_mps,
            work,
        )?;
        worst_withdraw_s = worst_withdraw_s.max(open_withdraw.duration);
        for d in [0.005_f64, 0.010, contact_window_m] {
            if d > contact_window_m + 1e-9 || d <= 1e-4 {
                continue;
            }
            let frac = (1.0 - d / hy.insertion_m.max(1e-6)).clamp(0.0, 1.0);
            let idx = ((frac * (insertion.q.len().saturating_sub(1)) as f64).round() as usize)
                .min(insertion.q.len().saturating_sub(1));
            let q_cw = q6_of(&insertion.q[idx]);
            let p_cw = motion::end_to_tcp(&s.input.arm.fk(&q_cw), &s.input.tcp_offset).p;
            let sep_cw = Pose::new(hy.r, [p_cw[0], p_cw[1], (p_cw[2] + feasibility::SEPARATION_M).max(sep.p[2])]);
            let sp = feasibility::phase(s, "separation", q_cw, s.closed_rad, sep_cw, st.descend_speed_mps, work)?;
            if let Some(ws) = s.input.ws.as_ref() {
                feasibility::check_carried_object_volume(&s.input.arm, &s.input.tcp_offset, ws, &obj_box, &sp, "separation")?;
            }
            worst_sep_s = worst_sep_s.max(sp.duration);
            let q_sp = q6_of(&sp.last());
            let lift_cw_z = (st.table_z_m + st.lift_height_m).max(sep_cw.p[2]);
            if lift_cw_z > sep_cw.p[2] + 1e-4 {
                let lift_cw = Pose::new(hy.r, [p_cw[0], p_cw[1], lift_cw_z]);
                let lp = feasibility::phase(s, "lift", q_sp, s.closed_rad, lift_cw, st.descend_speed_mps, work)?;
                if let Some(ws) = s.input.ws.as_ref() {
                    feasibility::check_carried_object_volume(&s.input.arm, &s.input.tcp_offset, ws, &obj_box, &lp, "lift")?;
                }
                worst_lift_s = worst_lift_s.max(lp.duration);
            }
            let wd = feasibility::phase(s, "withdrawal", q_cw, s.closed_rad, pre, st.descend_speed_mps, work)?;
            worst_withdraw_s = worst_withdraw_s.max(wd.duration);
        }
    }

    // 9. Unified parent deadline budget check (transition + vision + open + insert + capped close + loaded/recovery + settle)
    let vis = visible(s, t, &pre);
    let vision_step_s = st.settle_delay_s + st.lost_timeout_s.min(1.5);
    let start_vision_s = vision_step_s;
    let pregrasp_vision_s = if vis { vision_step_s } else { 0.0 };
    let post_lift_settle_s = st.post_lift_settle_ms as f64 / 1000.0;
    let arm_settle_nom = 0.5 * (3.0 + if lift_path.is_some() { 1.0 } else { 0.0 });
    let arm_settle_rec = 0.5 * 3.0;
    let nominal_total_s = pregrasp_path.duration
        + start_vision_s
        + pre_open_s
        + pregrasp_vision_s
        + insertion.duration
        + close_s
        + worst_sep_s
        + worst_lift_s
        + post_lift_settle_s
        + arm_settle_nom;
    let recovery_total_s = pregrasp_path.duration
        + start_vision_s
        + pre_open_s
        + pregrasp_vision_s
        + insertion.duration
        + close_s
        + worst_withdraw_s
        + arm_settle_rec;
    let required_s = nominal_total_s.max(recovery_total_s);
    if required_s > st.timeout_s {
        return Err(Rejection::new(
            "budget",
            "GRASP_TIMEOUT",
            format!(
                "scheduled route budget {:.1} s (nominal {:.1} s, recovery {:.1} s, close {:.1} s) exceeds timeout_s {:.1} s",
                required_s, nominal_total_s, recovery_total_s, close_s, st.timeout_s
            ),
        )
        .with(required_s, st.timeout_s));
    }

    // Margins & travel summary
    let paths: Vec<&motion::JointPath> = [
        Some(&pregrasp_path),
        Some(&insertion),
        Some(&separation),
        lift_path.as_ref(),
    ]
    .into_iter()
    .flatten()
    .collect();
    let mut jm = 1.0f64;
    let mut sig = f64::INFINITY;
    for p in paths.iter().copied().chain(std::iter::once(&withdrawal)) {
        for (k, q) in p.q.iter().enumerate() {
            let q = q6_of(q);
            jm = jm.min(feasibility::joint_margin(&s.input, &q));
            if k % 3 == 0 || k + 1 == p.q.len() {
                sig = sig.min(feasibility::sigma_min(&s.input, &q));
            }
        }
    }
    let duration_s = paths.iter().map(|p| p.duration).sum::<f64>();
    let joint_travel_rad: f64 = paths
        .iter()
        .map(|p| {
            p.q.windows(2)
                .map(|w| (0..p.arm_dof).map(|i| f64::from((w[1][i] - w[0][i]).abs())).sum::<f64>())
                .sum::<f64>()
        })
        .sum();
    let mut phases = Vec::with_capacity(8);
    if acq_ok {
        phases.push("acquisition");
    }
    phases.extend([
        "transition",
        "pre_open",
        "insertion",
        "closure",
        "separation",
        "lift",
        "withdrawal",
    ]);
    let margins = Margins {
        joint_limit: jm,
        sigma_min: sig,
        table_clearance_m: hy.table_clearance_m,
        payload_capacity_kg: capacity.map(|c| c.0),
    };
    let checked = Checked {
        q_pre,
        q_grasp,
        q_lift,
        separation: sep,
        lift,
        duration_s,
        joint_travel_rad,
        margins,
        phases: phases.clone(),
        payload_assumed,
        insufficient_evidence: insufficient,
    };
    let lim = st.descend_contact_stop_nm.map_or(0.0, |l| l as f32);
    let mut max_residual_nm = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
    max_residual_nm[1] = lim;
    max_residual_nm[2] = lim;
    let contact_guard = crate::rt_core::ops::ContactGuard {
        max_residual_nm,
        stop_code: None,
    };
    let plan = ExecutionPlan {
        pregrasp: pre,
        insertion_dir: hy.a,
        insertion_m: hy.insertion_m,
        grasp: grasp_pose,
        separation: sep,
        lift,
        jaw_opening_m: s.tool.open_width_m,
        chord_width_m: hy.width_m,
        q_pregrasp: q_pre,
        q_grasp,
        q_lift,
    };
    let has_oriented_evidence = s.evidence.contour_norm.is_some()
        || s.evidence.obb_corners_norm.is_some()
        || (s.evidence.angle_deg.is_some()
            && s.evidence.angle_conf.is_some_and(|c| c >= st.align_min_angle_conf));
    let route = AdaptiveRoute {
        candidate_id: String::new(),
        approach_class: hy.class.name(),
        tilt_deg: hy.tilt_deg,
        plan,
        pregrasp_mode,
        obj_box_tcp: obj_box,
        target_centroid: t.centroid,
        target_table_z: t.table_z,
        target_height: t.height,
        target_poly: t.poly.clone(),
        target_axis: t.axis,
        target_axis_ambiguous: t.axis_ambiguous,
        target_hard_error_m: t.hard_error_m,
        opening_reserve_m: hy.opening_reserve_m,
        contact_window_m,
        visible_at_pregrasp: vis,
        pre_open_command: pre_open_cmd,
        pre_open_fraction: pre_open_frac,
        budget: RouteBudget {
            pregrasp_s: pregrasp_path.duration,
            start_vision_s,
            pre_open_s,
            pregrasp_vision_s,
            insert_s: insertion.duration,
            close_s,
            close_timeout_s,
            is_pos_force_close,
            separation_s: worst_sep_s,
            lift_s: worst_lift_s,
            withdrawal_s: worst_withdraw_s,
            post_lift_settle_s,
            nominal_total_s,
            recovery_total_s,
        },
        contact_guard,
        contact_stop_armed,
        checked_phases: phases,
        margins,
        robustness: Robustness::default(),
        scores: Scores::default(),
        approach_mps: st.approach_speed_mps,
        insert_mps: st.descend_speed_mps,
        loaded_mps: st.descend_speed_mps,
        has_oriented_evidence,
    };
    Ok((checked, route))
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
