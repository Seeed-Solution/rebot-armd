//! Episode replay (action kind `replay`): load a saved episode, resample its measured
//! joint angles (`observation.state`) to the control rate, check it, then drive it as an
//! action reference: a planned joint move to the first frame, then the frames.
//!
//! Loading and the checks run without the supervisor lock ([`plan`]); the install
//! ([`SupervisorInner::install_replay`]) re-checks everything that may have changed.
use serde_json::Value;

use crate::chain::ArmModel;
use crate::model::{CameraMount, JointLimit, WorkspacePolicy};
use crate::motion::{self, MotionError, Settle, StepCtx, StepOut, StepStatus, LAG_FULL, LAG_STOP, STALL_TIMEOUT_S};
use crate::recording::RecordFrame;
use crate::safety::SafetyError;
use crate::supervisor::{ArmError, SupervisorInner, MAX_KEEPOUT_EVALS_PER_PLAN};
use crate::trajectory::{MultiJointQuinticTrajectory, TrajectoryPlanner};

/// Playback speeds accepted by `/api/v1/action/replay` (fraction of the recorded speed).
pub const REPLAY_SPEEDS: [f64; 2] = [0.5, 1.0];
pub const DEFAULT_REPLAY_SPEED: f64 = 0.5;
/// Longest episode that can be replayed (recorded time, s).
pub const MAX_REPLAY_DURATION_S: f64 = 600.0;
/// Tolerance on the per-joint speed check (rad/s), above float rounding.
pub const SPEED_CHECK_EPS: f32 = 1e-3;
/// Relative tolerance on the per-joint speed check. Recorded timestamps jitter against the
/// control tick that produced the positions (15..25 ms on a 20 ms period in a mock-follower
/// run), so a recording made at the planned-motion limit (a jog) shows single samples up to
/// ~0.3 % above it; without this margin such an episode is refused at 1x.
pub const SPEED_CHECK_REL: f32 = 0.02;
/// Playback rate (fraction of the chosen speed) at or below which the replay counts as
/// stalled; [`STALL_TIMEOUT_S`] of that fails it with TRACKING_STALL.
pub const STALL_RATE: f64 = 0.1;

/// Body of `/api/v1/action/replay`.
#[derive(Debug, Clone, PartialEq)]
pub struct ReplayRequest {
    pub episode: String,
    pub speed: f64,
    /// Replay the gripper joint; false holds it where it is.
    pub gripper: bool,
    pub tag: Option<String>,
    /// Replay an episode recorded with another joint zero (otherwise DATA_STALE).
    pub allow_stale: bool,
}

/// `{episode, speed: 0.5 | 1.0 (default 0.5), gripper: bool (default true), allow_stale?: false, tag?}`.
pub fn parse_replay(body: &Value) -> Result<ReplayRequest, MotionError> {
    let episode = body.get("episode").and_then(Value::as_str)
        .ok_or_else(|| MotionError::InvalidCommand("episode must be an episode id string".into()))?;
    if !crate::recording::valid_episode_id(episode) {
        return Err(MotionError::InvalidCommand(format!("episode '{episode}' is not a valid id (1-64 characters from A-Z a-z 0-9 _ -)")));
    }
    let speed = motion::num(body, "speed")?.unwrap_or(DEFAULT_REPLAY_SPEED);
    if !REPLAY_SPEEDS.iter().any(|s| (s - speed).abs() < 1e-9) {
        return Err(MotionError::InvalidCommand(format!("speed must be one of {REPLAY_SPEEDS:?}")));
    }
    Ok(ReplayRequest { episode: episode.into(), speed, gripper: motion::flag(body, "gripper", true)?, tag: motion::tag(body)?,
        allow_stale: motion::flag(body, "allow_stale", false)? })
}

/// Supervisor state a replay is checked against (taken under the lock).
#[derive(Debug, Clone)]
pub struct ReplayInput {
    pub request: ReplayRequest,
    pub path: std::path::PathBuf,
    pub profile_id: Option<String>,
    pub dof: usize,
    pub arm_dof: usize,
    pub gripper_idx: Option<usize>,
    pub joint_names: Vec<String>,
    /// Joint soft limits (`config.limits`).
    pub limits: Vec<JointLimit>,
    /// Planned-motion limits (`planning_limits()`): the speed bound of the check.
    pub speed_limits: Vec<JointLimit>,
    pub ws: Option<(WorkspacePolicy, CameraMount)>,
    pub arm: ArmModel,
    /// Commanded positions when the replay was requested (workspace rule baseline).
    pub start: Vec<f32>,
    pub control_hz: u32,
    pub epoch: u64,
    pub workspace_revision: u64,
    /// Power budget whose `planning` section time-scales the playback; None = none.
    pub power: Option<crate::power::PowerBudget>,
    /// Episode store the episode is read from, and the joint zero in effect (stale check).
    pub store: crate::recording::EpisodeStore,
    pub zero_offsets: Vec<f32>,
    /// Active tool TCP offset in `end_link` frame (m).
    pub tcp_offset: [f64; 3],
    /// Active tool identity (gripper provenance check).
    pub active_tool: crate::tool::ActiveToolInfo,
}

/// A checked replay ready to install.
#[derive(Debug, Clone)]
pub struct ReplayPlan {
    pub input: ReplayInput,
    /// Recording resampled at `sample_dt` (recorded time); `samples[k]` is at `k * sample_dt`.
    pub samples: Vec<Vec<f32>>,
    pub sample_dt: f64,
    /// Recorded duration (s).
    pub duration: f64,
    /// Power-budget time scale of the playback (1 = at the requested speed); None without a
    /// planning budget.
    pub power_scale: Option<f64>,
    pub has_gripper_motion: bool,
    pub tool_provenance_ok: bool,
    pub planned_tool_digest: String,
    pub recorded_tool_id: Option<String>,
    pub recorded_tool_digest: Option<String>,
}

fn rejected(status: u16, code: &str, message: String) -> ArmError {
    ArmError::Rejected { status, code: code.into(), message }
}

/// Resample `frames` (by timestamp) at `dt`, linear between frames. Timestamps must be
/// finite and non-decreasing; frames with the same timestamp keep the last one.
pub fn resample(frames: &[RecordFrame], dt: f64) -> Result<(Vec<Vec<f32>>, f64), ArmError> {
    if frames.len() < 2 {
        return Err(rejected(422, "REPLAY_EMPTY", format!("episode has {} frame(s); replay needs at least 2", frames.len())));
    }
    let t0 = frames[0].timestamp;
    for (k, w) in frames.windows(2).enumerate() {
        if !w[0].timestamp.is_finite() || !w[1].timestamp.is_finite() || w[1].timestamp < w[0].timestamp {
            return Err(rejected(422, "REPLAY_INVALID_EPISODE", format!("timestamps are not finite and non-decreasing at frame {}", k + 1)));
        }
    }
    let duration = frames[frames.len() - 1].timestamp - t0;
    if duration <= 0.0 {
        return Err(rejected(422, "REPLAY_EMPTY", "episode has no duration (all frames at one timestamp)".into()));
    }
    if duration > MAX_REPLAY_DURATION_S {
        return Err(rejected(422, "REPLAY_TOO_LONG", format!("episode lasts {duration:.1} s; replay is limited to {MAX_REPLAY_DURATION_S} s")));
    }
    let n = (duration / dt + 1e-9).floor() as usize;
    // a tail shorter than a quarter period is dropped (the last grid sample interpolates
    // into it), so the last segment is never so short that rounding inflates its speed
    let mut duration = duration;
    let tail = duration - n as f64 * dt;
    if tail > 1e-9 && tail <= 0.25 * dt { duration = n as f64 * dt; }
    let mut out = Vec::with_capacity(n + 2);
    let mut i = 0;
    for k in 0..=n {
        let t = t0 + (k as f64 * dt).min(duration);
        while i + 2 < frames.len() && frames[i + 1].timestamp <= t { i += 1; }
        out.push(lerp_frames(&frames[i], &frames[i + 1], t));
    }
    // the last frame itself, when the grid stops short of it
    if (n as f64) * dt < duration - 1e-9 { out.push(frames[frames.len() - 1].observation_state.clone()); }
    Ok((out, duration))
}

fn lerp_frames(a: &RecordFrame, b: &RecordFrame, t: f64) -> Vec<f32> {
    let span = b.timestamp - a.timestamp;
    let f = if span > 0.0 { ((t - a.timestamp) / span).clamp(0.0, 1.0) as f32 } else { 1.0 };
    a.observation_state.iter().zip(&b.observation_state).map(|(x, y)| x + (y - x) * f).collect()
}

/// Load, resample and check an episode for replay. Pure (file read, no lock).
/// Worker-side preparation of a replay for the RT broker (E3): the episode `plan` read and
/// resampled (file I/O here, on the worker) goes into a plan slot as joint samples, so the
/// commit installs it without reading the file. The worker checks the tool provenance the
/// legacy install checks (`TOOL_MISMATCH`) and names the active tool it checked against;
/// the commit refuses when that tool changed. `start` is the commanded position the episode
/// was loaded from. `PLAN_TOO_LARGE` beyond `MAX_PATH_SAMPLES` samples.
pub fn prepare_replay(plan: &ReplayPlan, active_tool_digest: &str, child: Option<crate::rt_core::ops::ChildSpec>)
    -> Result<crate::rt_core::ops::PlanSlot, &'static str> {
    use crate::rt_core::{mailbox::MAX_DOF, ops::{tool_digest_hash, EpisodeRef, PathKind, PlanSlot, Primitive, MAX_PATH_SAMPLES}};
    let input = &plan.input;
    let dof = input.start.len();
    if dof == 0 || dof > MAX_DOF || plan.samples.is_empty() || plan.samples.iter().any(|q| q.len() != dof) { return Err("INVALID_COMMAND"); }
    if plan.samples.len() > MAX_PATH_SAMPLES { return Err("PLAN_TOO_LARGE"); }
    if plan.planned_tool_digest != active_tool_digest { return Err("TOOL_MISMATCH"); }
    let approach_moves_gripper = input.gripper_idx.is_some_and(|g| input.request.gripper && (plan.samples[0][g] - input.start[g]).abs() > 1e-3);
    if input.request.gripper && (plan.has_gripper_motion || approach_moves_gripper) && !plan.tool_provenance_ok { return Err("TOOL_MISMATCH"); }
    let episode = EpisodeRef::new(&input.request.episode).ok_or("INVALID_COMMAND")?;
    let mut slot = PlanSlot::with_path_capacity(plan.samples.len());
    for (k, q) in plan.samples.iter().enumerate() { slot.path.push(q, k as f64)?; }
    let mut start = [0.0; MAX_DOF];
    start[..dof].copy_from_slice(&input.start);
    slot.primitive = Primitive::Prepared {
        child, dof: dof as u8, start,
        kind: PathKind::Replay { episode, speed: input.request.speed as f32, gripper: input.request.gripper, sample_dt: plan.sample_dt,
            tool_digest: tool_digest_hash(active_tool_digest) },
        duration_s: plan.duration, power_scale: plan.power_scale,
    };
    Ok(slot)
}

pub fn plan(input: ReplayInput) -> Result<ReplayPlan, ArmError> {
    let id = &input.request.episode;
    let sidecar = match input.store.get_or_adopt(id, &input.zero_offsets, &input.joint_names, input.profile_id.as_deref(), f64::from(input.control_hz)) {
        Ok(s) => s,
        Err(crate::recording::StoreError::NotFound(_)) => return Err(rejected(404, "EPISODE_NOT_FOUND", format!("Episode '{id}' not found"))),
        Err(e) => return Err(rejected(422, "REPLAY_INVALID_EPISODE", format!("Failed to read episode '{id}': {e}"))),
    };
    if sidecar.units != "rad" {
        return Err(rejected(422, "REPLAY_INVALID_EPISODE", format!("Episode '{id}' is stored in '{}', not rad", sidecar.units)));
    }
    if !input.request.allow_stale && sidecar.joint_zero.stale_against(&input.zero_offsets) {
        return Err(ArmError::detailed(409, "DATA_STALE", format!(
            "Episode '{id}' was recorded with another joint zero (rev {}); replay it with allow_stale: true only after checking it", sidecar.joint_zero.rev),
            serde_json::json!({"episode_id": id})));
    }
    let meta = Some(sidecar.metadata());
    if input.arm_dof == 5 {
        let has_arm_dof = meta.as_ref().and_then(|m| m.arm_dof).is_some();
        if !has_arm_dof {
            return Err(rejected(400, "REPLAY_DOF_MISMATCH", format!(
                "Episode '{}' missing arm_dof on 5-DOF profile", input.request.episode
            )));
        }
    }
    if let Some(ref m) = meta {
        if let (Some(ref stored_pid), Some(ref active_pid)) = (&m.profile_id, &input.profile_id) {
            if stored_pid != active_pid {
                return Err(rejected(400, "REPLAY_DOF_MISMATCH", format!(
                    "Episode '{}' profile_id '{stored_pid}' does not match active profile_id '{active_pid}'", input.request.episode
                )));
            }
        }
        if let Some(ad) = m.arm_dof {
            if ad != input.arm_dof {
                return Err(rejected(400, "REPLAY_DOF_MISMATCH", format!(
                    "Episode '{}' arm_dof {ad} does not match active config arm_dof {}", input.request.episode, input.arm_dof
                )));
            }
        }
        if let Some(gj) = m.gripper_joint {
            if Some(gj) != input.gripper_idx {
                return Err(rejected(400, "REPLAY_DOF_MISMATCH", format!(
                    "Episode '{}' gripper_joint {gj} does not match active config gripper_joint {:?}", input.request.episode, input.gripper_idx
                )));
            }
        }
        if let Some(ref jn) = m.joint_names {
            if jn != &input.joint_names {
                return Err(rejected(400, "REPLAY_DOF_MISMATCH", format!(
                    "Episode '{}' joint_names do not match active config joint_names", input.request.episode
                )));
            }
        }
    }
    let frames = match crate::recording::read_frames_from_parquet(&input.path) {
        Ok(f) => f,
        Err(crate::recording::ParquetRecordingError::Io(e)) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(rejected(404, "EPISODE_NOT_FOUND", format!("Episode '{}' not found", input.request.episode)));
        }
        Err(e) => return Err(rejected(422, "REPLAY_INVALID_EPISODE", format!("episode '{}' cannot be read: {e}", input.request.episode))),
    };
    let parquet_kv = crate::recording::read_parquet_kv(&input.path)
        .map_err(|e| rejected(422, "REPLAY_INVALID_EPISODE", format!("episode '{}' parquet metadata cannot be read: {e}", input.request.episode)))?;
    let emb = crate::recording::import::extract_embedded_provenance(&parquet_kv)
        .map_err(|e| rejected(422, "REPLAY_INVALID_EPISODE", format!("episode '{}' invalid embedded metadata: {e}", input.request.episode)))?;
    if (emb.tool_id.is_some() && emb.tool_id != sidecar.tool_id)
        || (emb.tool_config_digest.is_some() && emb.tool_config_digest != sidecar.tool_config_digest)
        || (emb.has_gripper_motion.is_some() && sidecar.has_gripper_motion.is_some() && emb.has_gripper_motion != sidecar.has_gripper_motion)
    {
        return Err(rejected(422, "REPLAY_INVALID_EPISODE", format!(
            "Episode '{}' sidecar provenance ({:?}/{:?}, gripper={:?}) conflicts with parquet embedded metadata ({:?}/{:?}, gripper={:?})",
            input.request.episode,
            sidecar.tool_id,
            sidecar.tool_config_digest,
            sidecar.has_gripper_motion,
            emb.tool_id,
            emb.tool_config_digest,
            emb.has_gripper_motion
        )));
    }
    let eff_tool_id = emb.tool_id.or(sidecar.tool_id.clone());
    let eff_tool_digest = emb.tool_config_digest.or(sidecar.tool_config_digest.clone());
    let frames_gripper = crate::recording::store::episode_has_gripper_motion(&frames, input.gripper_idx);
    let sidecar_has_gripper = emb.has_gripper_motion == Some(true)
        || sidecar.has_gripper_motion == Some(true)
        || frames_gripper;
    let approach_moves_gripper = input.gripper_idx.is_some_and(|g| {
        frames.first().and_then(|f| f.observation_state.get(g).copied())
            .zip(input.start.get(g).copied())
            .is_some_and(|(first_g, start_g)| (first_g - start_g).abs() > 1e-3)
    });
    let provenance_ok = input
        .active_tool
        .matches_record(eff_tool_id.as_deref(), eff_tool_digest.as_deref());
    if input.request.gripper && (sidecar_has_gripper || approach_moves_gripper) && !provenance_ok {
        return Err(rejected(409, "TOOL_MISMATCH", format!(
            "Episode '{}' contains gripper motion from tool {:?} ({:?}), which does not match active tool '{}' ({}); pass gripper: false for arm-only playback",
            input.request.episode, eff_tool_id, eff_tool_digest, input.active_tool.id, input.active_tool.config_digest
        )));
    }
    let mut plan = plan_frames_with_provenance(input, &frames, eff_tool_id, eff_tool_digest)?;
    plan.has_gripper_motion = sidecar_has_gripper;
    plan.tool_provenance_ok = provenance_ok;
    Ok(plan)
}

/// [`plan`] for frames already in memory (legacy unkeyed provenance `(None, None)`).
pub fn plan_frames(input: ReplayInput, frames: &[RecordFrame]) -> Result<ReplayPlan, ArmError> {
    plan_frames_with_provenance(input, frames, None, None)
}

fn plan_frames_with_provenance(
    input: ReplayInput,
    frames: &[RecordFrame],
    recorded_tool_id: Option<String>,
    recorded_tool_digest: Option<String>,
) -> Result<ReplayPlan, ArmError> {
    if frames.is_empty() {
        return Err(rejected(422, "REPLAY_EMPTY", "episode has no frames".into()));
    }
    if let Some((k, f)) = frames.iter().enumerate().find(|(_, f)| f.observation_state.len() != input.dof) {
        return Err(rejected(422, "REPLAY_JOINT_COUNT", format!(
            "frame {k} has {} joints; this arm has {}", f.observation_state.len(), input.dof)));
    }
    if let Some(k) = frames.iter().position(|f| f.observation_state.iter().any(|q| !q.is_finite())) {
        return Err(rejected(422, "REPLAY_INVALID_EPISODE", format!("frame {k} has a non-finite joint angle")));
    }
    let sample_dt = 1.0 / f64::from(input.control_hz.max(1));
    let (samples, duration) = resample(frames, sample_dt)?;
    let replayed = |j: usize| input.request.gripper || Some(j) != input.gripper_idx;

    // joint soft limits, every frame
    for f in frames {
        for (j, (&q, l)) in f.observation_state.iter().zip(&input.limits).enumerate() {
            if replayed(j) && (q < l.min_position || q > l.max_position) {
                return Err(rejected(400, "SOFT_LIMIT_EXCEEDED", format!(
                    "joint {} ({}) is {q:.4} rad at t={:.2} s, outside [{:.4}, {:.4}]",
                    j + 1, l.name, f.timestamp - frames[0].timestamp, l.min_position, l.max_position)));
            }
        }
    }

    // per-joint speed at the chosen playback speed, against the planned-motion limits
    let speed = input.request.speed as f32;
    let mut worst: Option<(usize, f32, usize)> = None; // (joint, ratio, sample)
    for (k, w) in samples.windows(2).enumerate() {
        let h = if k + 2 == samples.len() { (duration - k as f64 * sample_dt).max(1e-9) } else { sample_dt };
        for j in (0..input.dof).filter(|&j| replayed(j)) {
            let v = (w[1][j] - w[0][j]).abs() / h as f32 * speed;
            let limit = input.speed_limits[j].max_velocity;
            if v > limit * (1.0 + SPEED_CHECK_REL) + SPEED_CHECK_EPS {
                let ratio = v / limit;
                if worst.is_none_or(|(_, r, _)| ratio > r) { worst = Some((j, ratio, k)); }
            }
        }
    }
    if let Some((j, ratio, k)) = worst {
        let limit = input.speed_limits[j].max_velocity;
        let max_speed = input.request.speed / f64::from(ratio);
        return Err(rejected(422, "REPLAY_TOO_FAST", format!(
            "joint {} moves at {:.3} rad/s at {}x (t={:.2} s); the planned-motion limit is {limit:.3} rad/s, so this episode needs a playback speed of at most {max_speed:.2}x",
            j + 1, limit * ratio, input.request.speed, k as f64 * sample_dt)));
    }

    // workspace and keep-out boxes over every resampled frame, with the planned-motion rule
    if let Some((ws, cam)) = input.ws.as_ref() {
        let held = |q: &[f32]| -> Vec<f32> {
            let mut q = q.to_vec();
            if let (false, Some(g)) = (input.request.gripper, input.gripper_idx) { q[g] = input.start[g]; }
            q
        };
        let path: Vec<Vec<f32>> = samples.iter().map(|q| held(q)).collect();
        crate::supervisor::check_path_with_tcp_tol(&input.arm, ws, cam, input.tcp_offset, crate::kinematics::WORKSPACE_EPS, &input.start, path.iter().map(Vec::as_slice))
            .map_err(workspace_reject)?;
        let at = |t: f64| -> Vec<f32> {
            let k = (t.floor() as usize).min(path.len() - 1);
            let b = (k + 1).min(path.len() - 1);
            let f = (t - k as f64).clamp(0.0, 1.0) as f32;
            path[k].iter().zip(&path[b]).map(|(x, y)| x + (y - x) * f).collect()
        };
        let travel = |t0: f64, t1: f64| -> Vec<f64> {
            // linear segments: per-joint travel is the sum over the segments crossed
            let (a, b) = (t0.min(t1).max(0.0), t0.max(t1).min((path.len() - 1) as f64));
            let mut tr = vec![0.0f64; input.dof];
            let mut k = a.floor() as usize;
            while (k as f64) < b && k + 1 < path.len() {
                let lo = a.max(k as f64);
                let hi = b.min(k as f64 + 1.0);
                for (j, t) in tr.iter_mut().enumerate() { *t += f64::from((path[k + 1][j] - path[k][j]).abs()) * (hi - lo); }
                k += 1;
            }
            tr
        };
        let knots: Vec<f64> = (0..path.len()).map(|k| k as f64).collect();
        crate::supervisor::keepout_sweep_with_tcp_tol(&input.arm, ws, cam, input.tcp_offset, crate::kinematics::WORKSPACE_EPS, &at, &travel, &knots, MAX_KEEPOUT_EVALS_PER_PLAN)
            .map_err(workspace_reject)?;
    }
    let has_gripper_motion = crate::recording::store::episode_has_gripper_motion(frames, input.gripper_idx);
    let approach_moves_gripper = input.gripper_idx.is_some_and(|g| {
        frames.first().and_then(|f| f.observation_state.get(g).copied())
            .zip(input.start.get(g).copied())
            .is_some_and(|(first_g, start_g)| (first_g - start_g).abs() > 1e-3)
    });
    let tool_provenance_ok = input
        .active_tool
        .matches_record(recorded_tool_id.as_deref(), recorded_tool_digest.as_deref());
    if input.request.gripper && (has_gripper_motion || approach_moves_gripper) && !tool_provenance_ok {
        return Err(rejected(409, "TOOL_MISMATCH", format!(
            "Episode '{}' contains gripper motion from tool {:?} ({:?}), which does not match active tool '{}' ({}); pass gripper: false for arm-only playback",
            input.request.episode, recorded_tool_id, recorded_tool_digest, input.active_tool.id, input.active_tool.config_digest
        )));
    }
    // power budget: the playback (at the chosen speed) is slowed to fit, or refused
    let power_scale = match input.power.as_ref().filter(|b| b.enabled && b.planning.enabled) {
        None => None,
        Some(b) => {
            let t0 = std::time::Instant::now();
            let dof = input.arm.arm_dof().min(input.dof);
            let played = duration / input.request.speed;
            let n = crate::power_plan::Samples::count(&b.planning, played);
            let at = |t: f64| -> [f64; crate::chain::MAX_CHAIN_JOINTS] {
                let x = (t * input.request.speed / sample_dt).max(0.0);
                let k = (x.floor() as usize).min(samples.len() - 1);
                let nb = (k + 1).min(samples.len() - 1);
                let f = (x - k as f64).clamp(0.0, 1.0);
                std::array::from_fn(|j| if j < dof { f64::from(samples[k][j]) + (f64::from(samples[nb][j]) - f64::from(samples[k][j])) * f } else { 0.0 })
            };
            let ps = crate::power_plan::Samples::from_positions(played, n, dof, at);
            let fit = crate::power_plan::plan_scale(b, &input.arm, &ps).map_err(|e| rejected(422, e.code(), format!("episode '{}': {e}", input.request.episode)))?;
            tracing::debug!("Power planning (replay): scale {:.3}, peak {:.2} A (full {:.2} A), {} samples, {} µs",
                fit.scale, fit.peak_a, fit.peak_full_a, fit.samples, t0.elapsed().as_micros());
            Some(fit.scale)
        }
    };
    let planned_tool_digest = input.active_tool.config_digest.clone();
    Ok(ReplayPlan {
        input,
        samples,
        sample_dt,
        duration,
        power_scale,
        has_gripper_motion,
        tool_provenance_ok,
        planned_tool_digest,
        recorded_tool_id,
        recorded_tool_digest,
    })
}

fn workspace_reject(e: SafetyError) -> ArmError {
    let detail = e.workspace_detail().map(|d| d.with_phase("replay")).unwrap_or_default();
    ArmError::WorkspaceRejected {
        message: format!("episode leaves the allowed workspace: {e}"),
        detail: Box::new(detail),
    }
}

// `MultiJointQuinticTrajectory` stores up to 8 joint polynomials inline so transitioning
// from `Approach` to `Follow` never frees heap memory on the RT thread.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
enum Phase {
    Approach(MultiJointQuinticTrajectory),
    Follow { t: f64, stalled_for: f64 },
    Settle(Settle),
}

/// Backing storage for [`ReplayMotion`] samples: owned (legacy `ReplayPlan`) or a pinned
/// immutable plan slot borrowed until terminal retirement (RT prepared commit, zero copies).
#[derive(Debug, Clone)]
enum ReplaySamples {
    Owned(Vec<Vec<f32>>),
    Slot {
        slot: crate::rt_core::broker::PlanSlotRef<crate::rt_core::ops::PlanSlot>,
        dof: usize,
    },
}

impl ReplaySamples {
    fn len(&self) -> usize {
        match self {
            Self::Owned(v) => v.len(),
            Self::Slot { slot, .. } => slot.path.len(),
        }
    }

    fn sample(&self, k: usize) -> &[f32] {
        match self {
            Self::Owned(v) => &v[k],
            Self::Slot { slot, dof } => &slot.path.samples()[k][..*dof],
        }
    }
}

/// Episode id carried by [`ReplayMotion`]: owned `String` (legacy) or inline `EpisodeRef` (RT).
#[derive(Debug, Clone)]
enum ReplayEpisode {
    Owned(String),
    Inline(crate::rt_core::ops::EpisodeRef),
}

impl ReplayEpisode {
    fn as_str(&self) -> &str {
        match self {
            Self::Owned(s) => s.as_str(),
            Self::Inline(e) => e.as_str(),
        }
    }
}

/// Action reference of a replay: approach quintic, then the resampled frames.
#[derive(Debug, Clone)]
pub struct ReplayMotion {
    episode: ReplayEpisode,
    pub speed: f64,
    pub gripper: bool,
    pub gripper_idx: Option<usize>,
    samples: ReplaySamples,
    pub sample_dt: f64,
    pub duration: f64,
    /// Gripper command held when `gripper` is false.
    hold_gripper: Option<f32>,
    /// Request tag (standalone replay action).
    pub tag: Option<String>,
    approach_duration: f64,
    phase: Phase,
}

impl ReplayMotion {
    /// Episode identifier of this replay.
    pub fn episode_str(&self) -> &str { self.episode.as_str() }

    /// Number of resampled frames in this replay.
    pub fn sample_count(&self) -> usize { self.samples.len() }

    /// True when samples are borrowed directly from an immutable plan slot (no vector copy).
    pub fn borrows_plan_slot(&self) -> bool { matches!(self.samples, ReplaySamples::Slot { .. }) }

    /// True when dropping this replay motion would free heap memory.
    pub fn has_heap_storage(&self) -> bool {
        matches!(self.episode, ReplayEpisode::Owned(_))
            || matches!(self.samples, ReplaySamples::Owned(_))
            || self.tag.is_some()
            || matches!(&self.phase, Phase::Approach(q) if q.polynomials.is_heap())
    }

    /// Recording time reached (s): 0 during the approach.
    pub fn t_s(&self) -> f64 {
        match &self.phase { Phase::Approach(_) => 0.0, Phase::Follow { t, .. } => *t, Phase::Settle(_) => self.duration }
    }

    pub fn phase_name(&self) -> &'static str {
        match self.phase { Phase::Approach(_) => "approach", Phase::Follow { .. } => "replay", Phase::Settle(_) => "settle" }
    }

    /// Approach + playback at `speed` + settle allowance (s).
    pub fn expected_duration(&self) -> f64 { self.approach_duration + self.duration / self.speed + 0.5 }

    /// The recording at `t` into `out` (reused).
    pub fn sample_at_into(&self, t: f64, out: &mut Vec<f32>) {
        let len = self.samples.len();
        let x = (t / self.sample_dt).max(0.0);
        let k = (x.floor() as usize).min(len - 1);
        let b = (k + 1).min(len - 1);
        // the last segment may be shorter than sample_dt
        let seg = if b + 1 == len { (self.duration - k as f64 * self.sample_dt).max(1e-9) } else { self.sample_dt };
        let f = ((t - k as f64 * self.sample_dt) / seg).clamp(0.0, 1.0) as f32;
        let sa = self.samples.sample(k);
        let sb = self.samples.sample(b);
        out.clear();
        out.extend(sa.iter().zip(sb).map(|(x, y)| x + (y - x) * f));
    }

    /// Gripper joint of `q`: held when not replayed; while the stall latch holds it, a
    /// command further into the stall direction is held and a reversal releases the latch
    /// (the servo / jog rule). Returns whether the latch is to be cleared.
    fn apply_gripper(&self, q: &mut [f32], ctx: &StepCtx) -> bool {
        let Some(g) = self.gripper_idx.filter(|g| *g < q.len()) else { return false; };
        if let Some(h) = self.hold_gripper { q[g] = h; return false; }
        if !ctx.gripper_stalled { return false; }
        let delta = q[g] - ctx.commanded[g];
        let tau = ctx.torques.get(g).copied().unwrap_or(0.0);
        if delta * tau < -1e-4 || delta * ctx.gripper_stall_sign < -1e-4 { return true; }
        q[g] = ctx.commanded[g];
        false
    }

    crate::motion::step_into_via_with!();

    /// One step, written into `positions` / `velocities` (their memory is reused).
    fn step_with(&mut self, ctx: &StepCtx, mut positions: Vec<f32>, mut velocities: Vec<f32>) -> StepOut {
        let n = ctx.commanded.len();
        let progress = (self.t_s() / self.duration).clamp(0.0, 1.0) as f32;
        match &mut self.phase {
            Phase::Approach(traj) => {
                let (mut q, mut v) = (positions, velocities);
                traj.step_pv_into(ctx.dt * ctx.time_scale, &mut q, &mut v);
                let finished = traj.is_finished();
                for x in v.iter_mut() { *x *= ctx.time_scale as f32; }
                let clear = self.apply_gripper(&mut q, ctx);
                if let Some(g) = self.gripper_idx { if q[g] == ctx.commanded[g] { v[g] = 0.0; } }
                if finished { self.phase = Phase::Follow { t: 0.0, stalled_for: 0.0 }; }
                StepOut { positions: q, velocities: v, status: StepStatus::Running, progress: 0.0, clear_gripper_stall: clear }
            }
            Phase::Follow { t, stalled_for } => {
                // lag-aware time scaling, as for planned paths
                let lag = (0..n).filter(|&i| Some(i) != self.gripper_idx)
                    .map(|i| f64::from((ctx.commanded[i] - ctx.measured[i]).abs())).fold(0.0, f64::max);
                let lag = if lag.is_finite() { lag } else { f64::INFINITY };
                let rate = ((LAG_STOP - lag) / (LAG_STOP - LAG_FULL)).clamp(0.0, 1.0);
                // the rate approaches 0 as the lag approaches LAG_STOP; below STALL_RATE the
                // replay counts as not progressing
                if rate <= STALL_RATE {
                    *stalled_for += ctx.dt;
                    if *stalled_for >= STALL_TIMEOUT_S - 1e-9 {
                        positions.clear();
                        positions.extend_from_slice(ctx.commanded);
                        velocities.clear();
                        velocities.resize(n, 0.0);
                        return StepOut {
                            positions, velocities, progress, clear_gripper_stall: false,
                            status: StepStatus::Failed { code: "TRACKING_STALL", message: format!(
                                "Arm did not follow the replay for {STALL_TIMEOUT_S} s (lag {lag:.3} rad); motion stopped") },
                        };
                    }
                } else {
                    *stalled_for = 0.0;
                }
                *t = (*t + ctx.dt * self.speed * rate * ctx.time_scale).min(self.duration);
                let t_now = *t;
                let mut q = positions;
                self.sample_at_into(t_now, &mut q);
                let clear = self.apply_gripper(&mut q, ctx);
                velocities.clear();
                velocities.extend((0..n).map(|i| ((q[i] - ctx.commanded[i]) as f64 / ctx.dt) as f32));
                if t_now >= self.duration { self.phase = Phase::Settle(Settle::default()); }
                StepOut { positions: q, velocities, status: StepStatus::Running, progress: (t_now / self.duration).min(1.0) as f32, clear_gripper_stall: clear }
            }
            Phase::Settle(settle) => {
                let status = match settle.step(ctx.dt, ctx.commanded, ctx.measured, ctx.measured_velocities) {
                    Some(settled) => StepStatus::Done { settled: Some(settled), holding: None, gripper_gap_rad: None },
                    None => StepStatus::Running,
                };
                positions.clear();
                positions.extend_from_slice(ctx.commanded);
                velocities.clear();
                velocities.resize(n, 0.0);
                StepOut { positions, velocities, status, progress: 1.0, clear_gripper_stall: false }
            }
        }
    }
}

impl SupervisorInner {
    /// Busy rules of a replay, in addition to [`action_preconditions`](Self::action_preconditions).
    fn replay_preconditions(&self) -> Result<(), ArmError> {
        // a sequence run that records itself replays its episode steps into its own recording
        let own_recording = self.recording.as_ref().is_some_and(|r| matches!(r.owner, crate::teach::RecorderOwner::Sequence { action_id }
            if self.macro_run.as_ref().is_some_and(|m| m.id == action_id)));
        if !own_recording && (self.recorder.state() == &crate::recording::EpisodeState::Recording
            || self.recorder.state() == &crate::recording::EpisodeState::Finalizing) {
            return Err(MotionError::MotionBusy("an episode recording is running; save or discard it first".into()).into());
        }
        self.gravity_comp_motion_busy()?;
        self.action_preconditions()
    }

    /// Snapshot for [`plan`] (under the lock, after the lease / epoch checks).
    pub fn replay_input(&self, request: ReplayRequest, epoch: u64) -> Result<ReplayInput, ArmError> {
        self.check_commissioning_for_ordinary_motion("replay")?;
        if epoch != self.safety.epoch_id() {
            return Err(SafetyError::EpochMismatch { expected: self.safety.epoch_id(), received: epoch }.into());
        }
        let path = self.episodes.path_of(&request.episode)?;
        self.replay_preconditions()?;
        Ok(ReplayInput {
            path,
            profile_id: self.config.profile_id.clone(),
            dof: self.config.dof,
            arm_dof: self.config.arm_dof(),
            gripper_idx: self.config.gripper_index(),
            joint_names: self.config.limits.iter().map(|l| l.name.clone()).collect(),
            limits: self.config.limits.clone(),
            speed_limits: self.planning_limits(),
            ws: self.workspace_policy().map(|(ws, cam)| (ws.clone(), cam.clone())),
            arm: self.config.arm_model.clone(),
            start: self.commanded_positions.clone(),
            control_hz: self.config.control_frequency_hz,
            epoch,
            workspace_revision: self.workspace_revision,
            power: self.power_budget().cloned(),
            store: self.episodes.clone(),
            zero_offsets: self.joint_zero_offsets.clone(),
            tcp_offset: self.config.tcp_offset,
            active_tool: self.config.active_tool.clone(),
            request,
        })
    }

    /// Install a checked replay: plan the approach to the first frame at the planned-motion
    /// limits, then start the action. Returns (action id, expected duration).
    pub fn install_replay(&mut self, stamp: crate::supervisor::AdmissionStamp, plan: ReplayPlan) -> Result<(u64, f64), ArmError> {
        let admitted = self.check_admission_stamp(stamp)?;
        let (motion, first, scales) = self.replay_motion(plan)?;
        self.install_replay_motion(admitted, motion, &first, scales, true)
    }

    fn install_replay_motion(&mut self, admitted: crate::supervisor::AdmittedMotion, motion: ReplayMotion, first: &[f32], scales: [Option<f64>; 2], log_start: bool) -> Result<(u64, f64), ArmError> {
        let expected = motion.expected_duration();
        let tag = motion.tag.clone();
        self.start_replay_motion(admitted, first)?;
        let id = if log_start { self.begin_action("replay", tag) } else { self.begin_action_rt("replay") };
        for s in scales.into_iter().flatten() { self.note_action_power_scale(id, s); }
        if let Some(a) = self.last_action.as_mut() {
            a.episode = Some(motion.episode_str().into());
            a.phase = Some("approach".into());
            a.t_s = Some(0.0);
        }
        if log_start {
            tracing::info!("Replay of '{}' started (action {id}, {}x, gripper {}, {:.1} s recorded)", motion.episode_str(), motion.speed, motion.gripper, motion.duration);
        }
        self.active_trajectory = Some(crate::motion::ActiveTrajectory::new(Some(id), crate::motion::Reference::Replay(motion)));
        Ok((id, expected))
    }

    /// Install a checked replay as a sub-motion of macro action `id` (sequence replay step).
    /// Returns its expected duration.
    pub fn install_macro_replay(&mut self, stamp: crate::supervisor::AdmissionStamp, id: u64, plan: ReplayPlan) -> Result<f64, ArmError> {
        let admitted = self.check_admission_stamp(stamp)?;
        self.macro_sub_ready_pub(id)?;
        let (motion, first, scales) = self.replay_motion(plan)?;
        self.install_macro_replay_motion(admitted, id, motion, &first, scales)
    }

    fn install_macro_replay_motion(&mut self, admitted: crate::supervisor::AdmittedMotion, id: u64, motion: ReplayMotion, first: &[f32], scales: [Option<f64>; 2]) -> Result<f64, ArmError> {
        let expected = motion.expected_duration();
        self.start_replay_motion(admitted, first)?;
        for s in scales.into_iter().flatten() { self.note_action_power_scale(id, s); }
        let ep: crate::model::HolderLabel = motion.episode_str().into();
        self.update_action(id, |a| {
            a.episode = Some(ep);
            a.t_s = Some(0.0);
        });
        self.quintic_settle = None;
        self.active_trajectory = Some(crate::motion::ActiveTrajectory::new(Some(id), crate::motion::Reference::Replay(motion)));
        self.set_macro_sub_running(id);
        Ok(expected)
    }

    #[doc(hidden)]
    pub fn start_replay_motion(&mut self, _admitted: crate::supervisor::AdmittedMotion, first: &[f32]) -> Result<(), ArmError> {
        self.release_servo_target();
        self.servo_settled_ticks.fill(0);
        self.servo_ws_min_depths = None;
        crate::rt_core::scratch::copy_into(&mut self.target_positions, first);
        self.safety.mark_motion_started()?;
        self.last_heartbeat = std::time::Instant::now();
        Ok(())
    }

    /// The approach to the first frame: validated, planned from the commanded state, power
    /// fitted and checked against the workspace (no IK, inline polynomials and sweep).
    fn replay_approach(&mut self, first: &[f32], epoch: u64) -> Result<(MultiJointQuinticTrajectory, Option<f64>), ArmError> {
        self.safety.validate_motion_command(Some(epoch), first, &self.config.limits)?;
        let zero_buf = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        let zero_vec;
        let zeros: &[f32] = if self.config.dof <= zero_buf.len() {
            &zero_buf[..self.config.dof]
        } else {
            zero_vec = vec![0.0; self.config.dof];
            &zero_vec
        };
        let inline_limits = self.planning_limits_inline(None);
        let fallback_limits;
        let limits: &[JointLimit] = match &inline_limits {
            Some((buf, n)) => &buf[..*n],
            None => {
                fallback_limits = self.planning_limits();
                &fallback_limits
            }
        };
        let traj = TrajectoryPlanner::plan(&self.commanded_positions, &self.commanded_velocities, first,
            zeros, None, limits)?;
        let (traj, approach_scale) = self.power_fit_quintic(traj, &self.commanded_positions, &self.commanded_velocities, first, limits, true)?;
        self.check_workspace_quintic_inline(&traj)?;
        Ok((traj, approach_scale))
    }

    /// Zero-allocation validation of a prepared replay preserving the exact refusal precedence
    /// of [`Self::check_prepared_replay`] (`INVALID_COMMAND`, `PLAN_STALE`, preconditions, `TOOL_MISMATCH`).
    pub(crate) fn validate_prepared_replay(&self, dof: usize, path: &crate::rt_core::ops::PreparedPath, kind: crate::rt_core::ops::PathKind,
        duration: f64, epoch: u64) -> Result<(crate::rt_core::ops::EpisodeRef, f32, bool, f64), &'static str> {
        let crate::rt_core::ops::PathKind::Replay { episode, speed, gripper, sample_dt, tool_digest } = kind else {
            return Err("INVALID_COMMAND");
        };
        if path.is_empty() || dof != self.config.dof || dof > crate::rt_core::mailbox::MAX_DOF || !(sample_dt.is_finite() && sample_dt > 0.0) || !(duration.is_finite() && duration >= 0.0) {
            return Err("INVALID_COMMAND");
        }
        if self.safety.epoch_id() != epoch {
            return Err("PLAN_STALE");
        }
        self.replay_preconditions().map_err(crate::rt_core::ops::static_code)?;
        if crate::rt_core::ops::tool_digest_hash(&self.config.active_tool.config_digest) != tool_digest {
            return Err("TOOL_MISMATCH");
        }
        Ok((episode, speed, gripper, sample_dt))
    }

    /// Shared checks for a prepared replay (from `&PreparedPath` or pinned `PlanSlotRef`).
    fn check_prepared_replay(&self, dof: usize, path: &crate::rt_core::ops::PreparedPath, kind: crate::rt_core::ops::PathKind,
        duration: f64, epoch: u64) -> Result<(crate::rt_core::ops::EpisodeRef, f32, bool, f64), ArmError> {
        let crate::rt_core::ops::PathKind::Replay { episode, speed, gripper, sample_dt, tool_digest } = kind else {
            return Err(MotionError::InvalidCommand("not a prepared replay".into()).into());
        };
        if path.is_empty() || dof != self.config.dof || dof > crate::rt_core::mailbox::MAX_DOF || !(sample_dt.is_finite() && sample_dt > 0.0) || !(duration.is_finite() && duration >= 0.0) {
            return Err(MotionError::InvalidCommand("prepared replay without samples".into()).into());
        }
        if self.safety.epoch_id() != epoch {
            return Err(MotionError::PlanStale("epoch changed while loading the episode".into()).into());
        }
        self.replay_preconditions()?;
        if crate::rt_core::ops::tool_digest_hash(&self.config.active_tool.config_digest) != tool_digest {
            return Err(rejected(409, "TOOL_MISMATCH", format!(
                "Replay for episode '{}' was prepared for another tool than the active '{}'", episode.as_str(), self.config.active_tool.id)));
        }
        Ok((episode, speed, gripper, sample_dt))
    }

    /// The motion of a prepared replay borrowing directly from an immutable pinned plan slot
    /// (`PlanSlotRef`): zero sample vector copies and zero episode `String` allocation on commit.
    pub(crate) fn prepared_replay_from_slot(&mut self, dof: usize, slot: crate::rt_core::broker::PlanSlotRef<crate::rt_core::ops::PlanSlot>,
        kind: crate::rt_core::ops::PathKind, duration: f64, power_scale: Option<f64>, epoch: u64)
        -> Result<(ReplayMotion, [f32; crate::rt_core::mailbox::MAX_DOF], usize, [Option<f64>; 2]), ArmError> {
        let checked = self.check_prepared_replay(dof, &slot.path, kind, duration, epoch)?;
        self.prepared_replay_from_slot_checked(dof, slot, checked, duration, power_scale, epoch)
    }

    /// [`Self::prepared_replay_from_slot`] after zero-allocation [`Self::validate_prepared_replay`].
    pub(crate) fn prepared_replay_from_slot_checked(&mut self, dof: usize, slot: crate::rt_core::broker::PlanSlotRef<crate::rt_core::ops::PlanSlot>,
        checked: (crate::rt_core::ops::EpisodeRef, f32, bool, f64), duration: f64, power_scale: Option<f64>, epoch: u64)
        -> Result<(ReplayMotion, [f32; crate::rt_core::mailbox::MAX_DOF], usize, [Option<f64>; 2]), ArmError> {
        let (episode, speed, gripper, sample_dt) = checked;
        let gripper_idx = self.config.gripper_index();
        let hold_gripper = gripper_idx.filter(|_| !gripper).map(|g| self.commanded_positions[g]);
        let mut first = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        first[..dof].copy_from_slice(&slot.path.samples()[0][..dof]);
        if let (Some(g), Some(h)) = (gripper_idx, hold_gripper) { first[g] = h; }
        let (traj, approach_scale) = self.replay_approach(&first[..dof], epoch)?;
        let approach_duration = traj.duration;
        let motion = ReplayMotion {
            episode: ReplayEpisode::Inline(episode),
            speed: f64::from(speed) * power_scale.unwrap_or(1.0),
            gripper,
            gripper_idx,
            samples: ReplaySamples::Slot { slot, dof },
            sample_dt,
            duration,
            hold_gripper,
            tag: None,
            approach_duration,
            phase: Phase::Approach(traj),
        };
        Ok((motion, first, dof, [power_scale, approach_scale]))
    }

    /// Fallback prepared replay when called through `CoreOps::commit_prepared(&PreparedPath)`
    /// without a pinned [`crate::rt_core::broker::PlanSlotRef`].
    pub(crate) fn prepared_replay_motion(&mut self, dof: usize, path: &crate::rt_core::ops::PreparedPath, kind: crate::rt_core::ops::PathKind,
        duration: f64, power_scale: Option<f64>, epoch: u64)
        -> Result<(ReplayMotion, [f32; crate::rt_core::mailbox::MAX_DOF], usize, [Option<f64>; 2]), ArmError> {
        let (episode, speed, gripper, sample_dt) = self.check_prepared_replay(dof, path, kind, duration, epoch)?;
        let gripper_idx = self.config.gripper_index();
        let hold_gripper = gripper_idx.filter(|_| !gripper).map(|g| self.commanded_positions[g]);
        let samples: Vec<Vec<f32>> = path.samples().iter().map(|q| q[..dof].to_vec()).collect();
        let mut first = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        first[..dof].copy_from_slice(&samples[0]);
        if let (Some(g), Some(h)) = (gripper_idx, hold_gripper) { first[g] = h; }
        let (traj, approach_scale) = self.replay_approach(&first[..dof], epoch)?;
        let motion = ReplayMotion {
            episode: ReplayEpisode::Inline(episode), speed: f64::from(speed) * power_scale.unwrap_or(1.0), gripper,
            gripper_idx, samples: ReplaySamples::Owned(samples), sample_dt, duration, hold_gripper, tag: None,
            approach_duration: traj.duration, phase: Phase::Approach(traj),
        };
        Ok((motion, first, dof, [power_scale, approach_scale]))
    }

    /// Installs a prepared replay as a `replay` action (no RT `tracing::info!`). Returns (action id, expected duration).
    pub(crate) fn install_prepared_replay(&mut self, stamp: crate::supervisor::AdmissionStamp, prepared: (ReplayMotion, [f32; crate::rt_core::mailbox::MAX_DOF], usize, [Option<f64>; 2])) -> Result<(u64, f64), ArmError> {
        let admitted = self.check_admission_stamp(stamp)?;
        let (motion, first, dof, scales) = prepared;
        self.install_replay_motion(admitted, motion, &first[..dof], scales, false)
    }

    /// Installs a prepared replay as a sub-motion of macro action `id`. Returns its expected duration.
    pub(crate) fn install_macro_prepared_replay(&mut self, stamp: crate::supervisor::AdmissionStamp, id: u64, prepared: (ReplayMotion, [f32; crate::rt_core::mailbox::MAX_DOF], usize, [Option<f64>; 2])) -> Result<f64, ArmError> {
        let admitted = self.check_admission_stamp(stamp)?;
        self.macro_sub_ready_pub(id)?;
        let (motion, first, dof, scales) = prepared;
        self.install_macro_replay_motion(admitted, id, motion, &first[..dof], scales)
    }

    /// Checks of an install and the motion: (motion, first frame, power scales).
    fn replay_motion(&mut self, plan: ReplayPlan) -> Result<(ReplayMotion, Vec<f32>, [Option<f64>; 2]), ArmError> {
        let input = &plan.input;
        if self.safety.epoch_id() != input.epoch {
            return Err(MotionError::PlanStale("epoch changed while loading the episode".into()).into());
        }
        if self.workspace_revision != input.workspace_revision {
            return Err(MotionError::PlanStale("workspace or keep-out boxes changed while loading the episode".into()).into());
        }
        self.replay_preconditions()?;
        if self.commanded_positions.iter().zip(&input.start).any(|(a, b)| (a - b).abs() >= 1e-4) {
            return Err(MotionError::PlanStale("commanded positions changed while loading the episode".into()).into());
        }
        if plan.planned_tool_digest != self.config.active_tool.config_digest {
            return Err(rejected(409, "TOOL_MISMATCH", format!(
                "Replay for episode '{}' was planned for tool digest '{}' which does not match active tool '{}' ({})",
                input.request.episode, plan.planned_tool_digest, self.config.active_tool.id, self.config.active_tool.config_digest
            )));
        }
        let gripper_idx = input.gripper_idx;
        let hold_gripper = gripper_idx.filter(|_| !input.request.gripper).map(|g| self.commanded_positions[g]);
        let mut first = plan.samples[0].clone();
        if let (Some(g), Some(h)) = (gripper_idx, hold_gripper) { first[g] = h; }
        let approach_moves_gripper = gripper_idx.is_some_and(|g| {
            input.request.gripper && (first[g] - self.commanded_positions[g]).abs() > 1e-3
        });
        let tool_provenance_ok = self
            .config
            .active_tool
            .matches_record(plan.recorded_tool_id.as_deref(), plan.recorded_tool_digest.as_deref());
        if input.request.gripper && (plan.has_gripper_motion || approach_moves_gripper) && !tool_provenance_ok {
            return Err(rejected(409, "TOOL_MISMATCH", format!(
                "Episode '{}' contains gripper motion from tool {:?} ({:?}), which does not match active tool '{}' ({})",
                input.request.episode, plan.recorded_tool_id, plan.recorded_tool_digest, self.config.active_tool.id, self.config.active_tool.config_digest
            )));
        }
        let (traj, approach_scale) = self.replay_approach(&first, input.epoch)?;

        let ReplayPlan { input, samples, sample_dt, duration, power_scale, .. } = plan;
        let motion = ReplayMotion {
            episode: ReplayEpisode::Owned(input.request.episode.clone()), speed: input.request.speed * power_scale.unwrap_or(1.0), gripper: input.request.gripper,
            gripper_idx, samples: ReplaySamples::Owned(samples), sample_dt, duration, hold_gripper, tag: input.request.tag.clone(),
            approach_duration: traj.duration, phase: Phase::Approach(traj),
        };
        Ok((motion, first, [power_scale, approach_scale]))
    }

    /// Check and start a replay under the caller's lock (tests, in-process callers). The WS
    /// command loads and checks the episode without the lock.
    pub fn start_replay(&mut self, stamp: crate::supervisor::AdmissionStamp, request: ReplayRequest, epoch: u64) -> Result<(u64, f64), ArmError> {
        let _admitted = self.check_admission_stamp(stamp)?;
        let input = self.replay_input(request, epoch)?;
        let plan = plan(input)?;
        self.install_replay(stamp, plan)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(t: f64, q: &[f32]) -> RecordFrame { RecordFrame { timestamp: t, observation_state: q.to_vec(), action: q.to_vec() } }

    #[test]
    fn parse_rules() {
        let r = parse_replay(&serde_json::json!({"episode": "ep_1"})).unwrap();
        assert_eq!((r.speed, r.gripper), (0.5, true));
        let r = parse_replay(&serde_json::json!({"episode": "ep_1", "speed": 1.0, "gripper": false, "tag": "x", "allow_stale": true})).unwrap();
        assert_eq!((r.speed, r.gripper, r.tag.as_deref(), r.allow_stale), (1.0, false, Some("x"), true));
        for bad in [serde_json::json!({}), serde_json::json!({"episode": "../a"}), serde_json::json!({"episode": "a", "speed": 2.0}),
                    serde_json::json!({"episode": "a", "speed": 0.7}), serde_json::json!({"episode": "a", "gripper": 1})] {
            assert_eq!(parse_replay(&bad).unwrap_err().code(), "INVALID_COMMAND", "{bad}");
        }
    }

    #[test]
    fn resample_by_timestamp() {
        // 3 frames with uneven spacing, resampled at 0.02 s
        let frames = vec![frame(5.0, &[0.0]), frame(5.03, &[0.3]), frame(5.05, &[0.3]), frame(5.05, &[0.5]), frame(5.07, &[0.5])];
        let (s, d) = resample(&frames, 0.02).unwrap();
        assert!((d - 0.07).abs() < 1e-12);
        let got: Vec<f32> = s.iter().map(|q| q[0]).collect();
        // t = 0, 0.02, 0.04, 0.06, then the last frame (0.07)
        let want = [0.0, 0.2, 0.3, 0.5, 0.5];
        assert_eq!(got.len(), want.len(), "{got:?}");
        for (g, w) in got.iter().zip(want) { assert!((g - w).abs() < 1e-5, "{got:?}"); }
        // exact grid: no extra sample
        let frames: Vec<_> = (0..5).map(|k| frame(k as f64 * 0.02, &[k as f32])).collect();
        assert_eq!(resample(&frames, 0.02).unwrap().0.len(), 5);
        // errors
        let code = |e: ArmError| match e { ArmError::Rejected { code, .. } => code, e => panic!("{e:?}") };
        assert_eq!(code(resample(&frames[..1], 0.02).unwrap_err()), "REPLAY_EMPTY");
        assert_eq!(code(resample(&[frame(1.0, &[0.0]), frame(1.0, &[0.0])], 0.02).unwrap_err()), "REPLAY_EMPTY");
        assert_eq!(code(resample(&[frame(1.0, &[0.0]), frame(0.5, &[0.0])], 0.02).unwrap_err()), "REPLAY_INVALID_EPISODE");
        assert_eq!(code(resample(&[frame(0.0, &[0.0]), frame(f64::NAN, &[0.0])], 0.02).unwrap_err()), "REPLAY_INVALID_EPISODE");
        assert_eq!(code(resample(&[frame(0.0, &[0.0]), frame(601.0, &[0.0])], 0.02).unwrap_err()), "REPLAY_TOO_LONG");
    }

    #[test]
    fn r3_m5_plan_frames_enforces_active_tool_provenance_for_gripper_motion() {
        let dir = tempfile::tempdir().unwrap();
        let store = crate::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
        let mut cfg = crate::model::ArmConfig::rebot_dm();
        let finray_pack: crate::tool::ToolPackConfig = serde_json::from_value(serde_json::json!({
            "id": "soft_finray_gripper",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.0288, 0.0, 0.0]
        })).unwrap();
        let non_stock_cfg = crate::tool::apply_tool_overlay(&cfg, &finray_pack).unwrap();
        cfg.validate().unwrap();

        let q0 = vec![0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -1.0];
        let mut q1 = q0.clone();
        q1[6] = -0.2;
        let frames = vec![
            frame(0.0, &q0),
            frame(0.5, &q0),
            frame(1.0, &q1),
        ];

        let make_input = |c: &crate::model::ArmConfig, gripper: bool| ReplayInput {
            request: ReplayRequest { episode: "mem_ep".into(), speed: 1.0, gripper, tag: None, allow_stale: false },
            path: dir.path().join("unused.parquet"),
            profile_id: c.profile_id.clone(),
            dof: c.dof,
            arm_dof: c.arm_dof(),
            gripper_idx: c.gripper_index(),
            joint_names: c.limits.iter().map(|l| l.name.clone()).collect(),
            limits: c.limits.clone(),
            speed_limits: c.limits.clone(),
            ws: None,
            arm: c.arm_model.clone(),
            start: q0.clone(),
            control_hz: 50,
            epoch: 0,
            workspace_revision: 0,
            power: None,
            store: store.clone(),
            zero_offsets: vec![0.0; c.dof],
            tcp_offset: c.tcp_offset,
            active_tool: c.active_tool.clone(),
        };

        // 1. Non-stock active tool + gripper: true -> rejected with 409 TOOL_MISMATCH
        let err = plan_frames(make_input(&non_stock_cfg, true), &frames).unwrap_err();
        match err {
            ArmError::Rejected { status: 409, code, .. } => assert_eq!(code, "TOOL_MISMATCH"),
            other => panic!("expected 409 TOOL_MISMATCH, got {other:?}"),
        }

        // 2. Non-stock active tool + gripper: false (arm-only) -> allowed
        let arm_only = plan_frames(make_input(&non_stock_cfg, false), &frames).unwrap();
        assert!(!arm_only.tool_provenance_ok);

        // 3. Stock active tool + gripper: true -> allowed (legacy provenance)
        let stock_plan = plan_frames(make_input(&cfg, true), &frames).unwrap();
        assert!(stock_plan.tool_provenance_ok);
    }

    #[test]
    fn r4_m6_install_replay_rechecks_receiving_supervisor_active_tool() {
        let dir = tempfile::tempdir().unwrap();
        let store = crate::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
        let mut stock_cfg = crate::model::ArmConfig::rebot_dm();
        stock_cfg.validate().unwrap();
        let finray_pack: crate::tool::ToolPackConfig = serde_json::from_value(serde_json::json!({
            "id": "soft_finray_gripper",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.0288, 0.0, 0.0]
        })).unwrap();
        let mut non_stock_cfg = crate::tool::apply_tool_overlay(&stock_cfg, &finray_pack).unwrap();
        non_stock_cfg.active_tool.commissioning_required = false;

        let q0 = vec![0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -1.0];
        let mut q1 = q0.clone();
        q1[6] = -0.2;
        let mut recv = SupervisorInner::new(non_stock_cfg);
        recv.lease_enabled = false;
        recv.set_torque(true).unwrap();
        recv.commanded_positions = q0.clone();
        recv.measured_positions = q0.clone();
        recv.target_positions = q0.clone();
        let recv_epoch = recv.safety.epoch_id();

        let frames = vec![frame(0.0, &q0), frame(0.5, &q0), frame(1.0, &q1)];
        let stock_plan = plan_frames(ReplayInput {
            request: ReplayRequest { episode: "stock_mem".into(), speed: 1.0, gripper: true, tag: None, allow_stale: false },
            path: dir.path().join("unused.parquet"),
            profile_id: stock_cfg.profile_id.clone(),
            dof: stock_cfg.dof,
            arm_dof: stock_cfg.arm_dof(),
            gripper_idx: stock_cfg.gripper_index(),
            joint_names: stock_cfg.limits.iter().map(|l| l.name.clone()).collect(),
            limits: stock_cfg.limits.clone(),
            speed_limits: stock_cfg.limits.clone(),
            ws: None,
            arm: stock_cfg.arm_model.clone(),
            start: q0,
            control_hz: 50,
            epoch: recv_epoch,
            workspace_revision: 0,
            power: None,
            store,
            zero_offsets: vec![0.0; stock_cfg.dof],
            tcp_offset: stock_cfg.tcp_offset,
            active_tool: stock_cfg.active_tool.clone(),
        }, &frames).unwrap();
        assert!(stock_plan.tool_provenance_ok);

        let stamp = recv.current_admission_stamp();
        let err = recv.install_replay(stamp, stock_plan).unwrap_err();
        match err {
            ArmError::Rejected { status: 409, code, .. } => assert_eq!(code, "TOOL_MISMATCH"),
            other => panic!("expected 409 TOOL_MISMATCH on cross-tool install_replay, got {other:?}"),
        }
    }

    #[test]
    fn r5_m2_arm_only_plan_cannot_install_under_different_active_tool_digest() {
        let dir = tempfile::tempdir().unwrap();
        let store = crate::recording::EpisodeStore::new(Some(dir.path().to_path_buf()));
        let mut stock_cfg = crate::model::ArmConfig::rebot_dm();
        stock_cfg.validate().unwrap();
        let finray_pack: crate::tool::ToolPackConfig = serde_json::from_value(serde_json::json!({
            "id": "soft_finray_gripper",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.045, 0.0, 0.0]
        })).unwrap();
        let mut longer_cfg = crate::tool::apply_tool_overlay(&stock_cfg, &finray_pack).unwrap();
        longer_cfg.active_tool.commissioning_required = false;

        let q0 = vec![0.0, -0.5, -0.5, 0.5, 0.0, 0.0, -1.0];
        let mut q1 = q0.clone();
        q1[0] = 0.2;
        let mut recv = SupervisorInner::new(longer_cfg);
        recv.lease_enabled = false;
        recv.set_torque(true).unwrap();
        recv.commanded_positions = q0.clone();
        recv.measured_positions = q0.clone();
        recv.target_positions = q0.clone();
        let recv_epoch = recv.safety.epoch_id();

        let frames = vec![frame(0.0, &q0), frame(0.5, &q0), frame(1.0, &q1)];
        let stock_arm_only_plan = plan_frames(ReplayInput {
            request: ReplayRequest { episode: "stock_arm_only".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
            path: dir.path().join("unused.parquet"),
            profile_id: stock_cfg.profile_id.clone(),
            dof: stock_cfg.dof,
            arm_dof: stock_cfg.arm_dof(),
            gripper_idx: stock_cfg.gripper_index(),
            joint_names: stock_cfg.limits.iter().map(|l| l.name.clone()).collect(),
            limits: stock_cfg.limits.clone(),
            speed_limits: stock_cfg.limits.clone(),
            ws: None,
            arm: stock_cfg.arm_model.clone(),
            start: q0,
            control_hz: 50,
            epoch: recv_epoch,
            workspace_revision: 0,
            power: None,
            store,
            zero_offsets: vec![0.0; stock_cfg.dof],
            tcp_offset: stock_cfg.tcp_offset,
            active_tool: stock_cfg.active_tool.clone(),
        }, &frames).unwrap();
        assert!(!stock_arm_only_plan.has_gripper_motion);
        assert_eq!(stock_arm_only_plan.planned_tool_digest, stock_cfg.active_tool.config_digest);

        let stamp = recv.current_admission_stamp();
        let err = recv.install_replay(stamp, stock_arm_only_plan).unwrap_err();
        match err {
            ArmError::Rejected { status: 409, code, .. } => assert_eq!(code, "TOOL_MISMATCH"),
            other => panic!("expected 409 TOOL_MISMATCH for cross-tool arm-only replay plan, got {other:?}"),
        }
    }
}
