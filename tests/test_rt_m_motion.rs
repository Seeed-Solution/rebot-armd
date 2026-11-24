//! PR M integration, lock-hold, owner-parity, park-disable, holding_after_clear rearm,
//! legacy MIT moving-park exclusion, and B1/B2 zero-lock/zero-alloc RT proofs.

mod common;
mod mit_support;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget,
    TorqueOutcome, TorqueProgress,
};
use armd::mit::{self, MitJointQualification, MitQualificationFile};
use armd::model::{ArmConfig, JointLimit, JointState, KeepoutBox, RestingPosePolicy, WorkspacePolicy};
use armd::motion::MotionError;
use armd::power::PowerBudget;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{Authority, ClientId, Outcome, Reply, Submit, TransitionOp};
use armd::rt_core::executor::{ExecutorClock, ExecutorConfig, PersistTestHooks, RtExecutorHandle};
use armd::rt_core::ops::PlanSlot;
use armd::rt_core::requests::motion::{
    self as req, MotionInstallError, MotionInstalled, MotionPlanSnapshot, MotionRequest,
};
use armd::safety::{FaultCode, SafetyError, SafetyState};
use armd::supervisor::{ParkContinuationPhase, SupervisorInner};
use armd::tracking::{JointTrackingConfig, TrackingStrategy};
use armd::{create_router, ArmError, ArmSupervisor, OperatorAuth};
use mit_support::{FakeBus, SW_VER};
use serde_json::json;

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

// ============================================================================
// Deterministic Scripted Clock & Allocation-Free Configurable Test Driver
// ============================================================================

struct ScriptedClock {
    base: Instant,
    state: Mutex<ClockState>,
    cv: Condvar,
}

struct ClockState {
    now_ns: u64,
    limit_ns: u64,
    waiting_for_ns: Option<u64>,
}

impl ScriptedClock {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            base: Instant::now(),
            state: Mutex::new(ClockState { now_ns: 0, limit_ns: 0, waiting_for_ns: None }),
            cv: Condvar::new(),
        })
    }

    fn run_steps(&self, period_ns: u64, steps: u64) {
        let mut s = self.state.lock().unwrap_or_else(|e| e.into_inner());
        let target = s.limit_ns + period_ns * steps;
        s.limit_ns = target;
        self.cv.notify_all();
        let deadline = Instant::now() + Duration::from_secs(10);
        while !s.waiting_for_ns.is_some_and(|w| w > target) {
            let rem = deadline.saturating_duration_since(Instant::now());
            assert!(!rem.is_zero(), "clock park timeout past {target} ns (now={}, wait={:?})", s.now_ns, s.waiting_for_ns);
            let (next, _) = self.cv.wait_timeout(s, rem).unwrap_or_else(|e| e.into_inner());
            s = next;
        }
    }
}

impl ExecutorClock for ScriptedClock {
    fn now(&self) -> Instant { self.base + Duration::from_nanos(self.state.lock().unwrap_or_else(|e| e.into_inner()).now_ns) }
    fn now_ns(&self) -> u64 { self.state.lock().unwrap_or_else(|e| e.into_inner()).now_ns }
    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
        let mut s = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if s.now_ns >= target_ns { self.cv.notify_all(); return false; }
        while !stop.load(Ordering::Acquire) && s.now_ns < target_ns && s.limit_ns < target_ns {
            s.waiting_for_ns = Some(target_ns);
            self.cv.notify_all();
            s = self.cv.wait(s).unwrap_or_else(|e| e.into_inner());
        }
        s.waiting_for_ns = None;
        if !stop.load(Ordering::Acquire) && s.now_ns < target_ns { s.now_ns = target_ns; }
        self.cv.notify_all();
        true
    }
    fn sleep(&self, dur: Duration, stop: &AtomicBool) {
        if stop.load(Ordering::Acquire) { return; }
        let mut s = self.state.lock().unwrap_or_else(|e| e.into_inner());
        s.now_ns = s.now_ns.saturating_add(dur.as_nanos().min(u128::from(u64::MAX)) as u64);
        self.cv.notify_all();
    }
    fn wake(&self) { let _g = self.state.lock().unwrap_or_else(|e| e.into_inner()); self.cv.notify_all(); }
}

#[derive(Clone)]
struct DriverKnobs {
    confirm_disable: Arc<AtomicBool>,
    fail_next_enable: Arc<AtomicBool>,
    estop_inhibit_disable: Arc<AtomicBool>,
    disable_started: Arc<AtomicBool>,
    nudge_q0_on_enable: Arc<Mutex<Option<f32>>>,
}

impl DriverKnobs {
    fn new() -> Self {
        Self {
            confirm_disable: Arc::new(AtomicBool::new(true)),
            fail_next_enable: Arc::new(AtomicBool::new(false)),
            estop_inhibit_disable: Arc::new(AtomicBool::new(false)),
            disable_started: Arc::new(AtomicBool::new(false)),
            nudge_q0_on_enable: Arc::new(Mutex::new(None)),
        }
    }
}

struct MotionTestDriver {
    dof: usize,
    q: Vec<f32>,
    enabled: bool,
    caps: DriverCapabilities,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
    pending: Option<(bool, Instant)>,
    enable_delay: Duration,
    disable_delay: Duration,
    clock: Option<Arc<ScriptedClock>>,
    knobs: DriverKnobs,
}

impl MotionTestDriver {
    fn new(q: Vec<f32>, enable_delay: Duration, disable_delay: Duration, clock: Option<Arc<ScriptedClock>>) -> (Self, DriverKnobs) {
        let dof = q.len();
        let now = clock.as_ref().map_or_else(Instant::now, |c| c.now());
        let knobs = DriverKnobs::new();
        let drv = Self {
            dof, q, enabled: false,
            caps: DriverCapabilities {
                velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
                torque_feedforward: false, enable_feedback: true, load_feedback: false,
            },
            rx_at: vec![Some(now); dof], seqs: vec![1; dof], pending: None,
            enable_delay, disable_delay, clock, knobs: knobs.clone(),
        };
        (drv, knobs)
    }

    fn now(&self) -> Instant { self.clock.as_ref().map_or_else(Instant::now, |c| c.now()) }
}

impl ArmDriver for MotionTestDriver {
    fn capabilities(&self) -> DriverCapabilities { self.caps }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn clock_now(&self) -> Instant { self.now() }
    fn enable_state(&self) -> EnableState { if self.enabled { EnableState::Enabled } else { EnableState::Disabled } }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { let n = self.now(); self.rx_at.iter().map(|t| t.map(|t| n.saturating_duration_since(t))).collect() }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
    fn feedback_seqs(&self) -> Vec<u64> { self.seqs.clone() }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let n = self.now(); out.clear();
        for t in &self.rx_at { out.push(t.map(|t| n.saturating_duration_since(t))); }
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let n = self.now();
        self.rx_at.len() == dof && self.rx_at.iter().all(|t| t.is_some_and(|t| n.saturating_duration_since(t) <= max_age))
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let n = self.now(); out.clear();
        for t in &self.rx_at { out.push(t.map(|t| n.saturating_duration_since(t).as_millis() as u64)); }
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.rx_at); }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { out.clear(); out.extend_from_slice(&self.seqs); }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { out.clear(); out.extend_from_slice(&self.rx_at); }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.dof);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = self.now();
        out.clear();
        for i in 0..self.dof {
            out.push(JointState { position: self.q[i], velocity: 0.0, torque: 0.0, temperature: 30.0 });
            self.rx_at[i] = Some(now);
            self.seqs[i] += 1;
        }
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((self.dof as u16, false))
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(self.dof) { self.q[i] = t.position; }
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> { self.enabled = enable; Ok(()) }
    fn begin_torque_budgeted(&mut self, enable: bool, _: u16) -> Result<TorqueProgress, ArmDriverError> {
        if !enable {
            self.knobs.disable_started.store(true, Ordering::Release);
            if self.knobs.estop_inhibit_disable.load(Ordering::Acquire) {
                return Err(ArmDriverError::EstopInhibited);
            }
        }
        let delay = if enable { self.enable_delay } else { self.disable_delay };
        let wake = self.now() + delay;
        self.pending = Some((enable, wake));
        Ok(TorqueProgress::Pending { wake })
    }
    fn poll_torque_budgeted(&mut self, _: Instant, _: u16) -> Result<TorqueProgress, ArmDriverError> {
        let Some((target, wake)) = self.pending else { return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed)); };
        if self.now() < wake { return Ok(TorqueProgress::Pending { wake }); }
        self.pending = None;
        if target {
            if self.knobs.fail_next_enable.swap(false, Ordering::AcqRel) {
                return Err(ArmDriverError::Timeout(1));
            }
            if let Some(q0) = self.knobs.nudge_q0_on_enable.lock().unwrap().take() {
                self.q[0] = q0;
            }
            self.enabled = true;
            return Ok(TorqueProgress::Done(TorqueOutcome::Confirmed));
        }
        if self.knobs.estop_inhibit_disable.load(Ordering::Acquire) {
            return Err(ArmDriverError::EstopInhibited);
        }
        if self.knobs.confirm_disable.load(Ordering::Acquire) {
            self.enabled = false;
        }
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn abort_enable(&mut self) { if matches!(self.pending, Some((true, _))) { self.pending = None; } }
    fn torque_transition_pending(&self) -> bool { self.pending.is_some() }
}

fn parkable_2dof_config() -> ArmConfig {
    let mut cfg = ArmConfig::default();
    cfg.dof = 2;
    cfg.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0), JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0)];
    cfg.home_positions = vec![0.0, 0.0];
    cfg.lease_timeout_ms = 60_000;
    cfg.resting_pose = Some(RestingPosePolicy {
        positions: None, position_tolerances: vec![0.08; 2], max_total_deviation: 0.16,
        max_velocity: 0.015, joint_max_velocities: vec![], support_torque_limits: vec![(0, 1.0)],
        support_load_limits: vec![], feedback_max_age_ms: 200,
    });
    cfg
}

fn dm_config_with_planning_load() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 60_000;
    let mut ws = WorkspacePolicy::rebot_dm();
    ws.keepout_boxes.push(KeepoutBox {
        name: "side_box".into(), min_m: [0.40, 0.30, 0.05], max_m: [0.60, 0.50, 0.25],
    });
    cfg.workspace = Some(ws);
    cfg.power_budget = Some(PowerBudget::rebot_dm());
    cfg
}

async fn start_scripted_executor(sup: &Arc<ArmSupervisor>, period: Duration, clock: Arc<ScriptedClock>, alloc_timing: bool) -> RtExecutorHandle {
    sup.set_lease_enabled(false).await;
    let mut cfg = ExecutorConfig::for_tests(period);
    cfg.alloc_timing = alloc_timing;
    let mut hooks = PersistTestHooks::default();
    hooks.clock = Some(clock);
    sup.start_executor_with_hooks(cfg, hooks).await.expect("start executor")
}

fn auth_from_hub(sup: &ArmSupervisor, handle: &RtExecutorHandle) -> Authority {
    let snap = sup.hub().latest().expect("published snapshot");
    Authority {
        lease_generation: snap.rt.lease_generation,
        epoch: snap.obs.epoch_id,
        stop_generation: handle.stop_requests().applied(),
        config_revision: snap.rt.workspace_revision,
        parent_generation: 0,
    }
}

fn drain_replies(handle: &RtExecutorHandle) -> Vec<Reply> {
    let mut out = Vec::new();
    handle.broker().lock().unwrap().poll(|_, r| { out.push(r); true });
    out
}

fn submit_enable_and_wait(sup: &ArmSupervisor, handle: &RtExecutorHandle, clock: &ScriptedClock, period_ns: u64, seq: u64) -> Authority {
    let auth = auth_from_hub(sup, handle);
    let req_id = handle.broker().lock().unwrap()
        .submit(ClientId(1), seq, auth, clock.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn))
        .unwrap();
    for _ in 0..40 {
        clock.run_steps(period_ns, 2);
        let rs = drain_replies(handle);
        if rs.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req_id)) {
            return auth_from_hub(sup, handle);
        }
    }
    panic!("enable transition did not finish");
}

// ============================================================================
// 1. Priority Lock-Hold Proof (<= 2 ms write-lock hold, zero in-lock fallback)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_home_and_jog_write_lock_hold_under_2ms_with_no_in_lock_fallback() {
    let sup = Arc::new(ArmSupervisor::new(dm_config_with_planning_load()));
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions[0] = 0.25;
        inner.measured_positions[0] = 0.25;
    }
    let _ = tokio::task::spawn_blocking(|| {}).await;
    let _ = sup.inner().take_log();

    let epoch = sup.get_observation().await.epoch_id;
    let stamp = sup.capture_admission_stamp();
    sup.home(stamp, Some(1.0), Some(epoch)).await.expect("off-lock home succeeds");
    let hold = sup.inner().take_log().expect("lock log available");
    assert!(
        hold.writer_hold <= Duration::from_millis(2),
        "ArmSupervisor::home write lock held {:?} (> 2 ms budget); longest={:?}",
        hold.writer_hold, hold.longest
    );
    assert_eq!(sup.inner().read().await.prepared_quintics_used, 1);

    // Off-lock StartDrifted retry: nudge commanded_positions[0] before Lock #2 -> succeeds via retry, still <= 2 ms.
    sup.stop_motion().await;
    {
        let mut inner = sup.inner().write().await;
        inner.before_install_hook = Some(Box::new(|i| { i.commanded_positions[0] += 0.01; }));
    }
    let _ = sup.inner().take_log();
    let stamp = sup.capture_admission_stamp();
    let epoch = sup.get_observation().await.epoch_id;
    sup.home(stamp, Some(1.0), Some(epoch)).await.expect("home retries off-lock after StartDrifted");
    let hold2 = sup.inner().take_log().expect("lock log available");
    assert!(hold2.writer_hold <= Duration::from_millis(2), "retry write lock held {:?}", hold2.writer_hold);
    assert_eq!(sup.inner().read().await.prepared_quintics_used, 2);

    // PlanStale has ZERO in-lock planning fallback: bump workspace_revision before Lock #2 -> PlanStale & no trajectory.
    sup.stop_motion().await;
    {
        let mut inner = sup.inner().write().await;
        inner.before_install_hook = Some(Box::new(|i| { i.workspace_revision += 1; }));
    }
    let stamp = sup.capture_admission_stamp();
    let epoch = sup.get_observation().await.epoch_id;
    let err = sup.home(stamp, Some(1.0), Some(epoch)).await.unwrap_err();
    assert!(matches!(err, ArmError::Motion(MotionError::PlanStale(_))), "expected PlanStale, got {err:?}");
    let inner = sup.inner().read().await;
    assert!(inner.active_trajectory.is_none(), "must never fall back to in-lock planning");
    assert_eq!(inner.prepared_quintics_used, 2);
}

// ============================================================================
// 2. Validation Precedence, Commissioning, motion_config_revision, Zero Mutation
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn validation_precedence_commissioning_motion_config_revision_and_zero_mutation() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 60_000;
    cfg.active_tool.is_stock = false;
    cfg.active_tool.id = "custom_commissioning_tool".into();
    cfg.active_tool.config_digest = "v1:test".into();
    cfg.active_tool.commissioning_required = true;
    let mut start_q = ArmConfig::rebot_dm().home_positions;
    let g_idx = cfg.gripper_index().unwrap();
    let (open_rad, closed_rad) = {
        let gp = cfg.gripper_policy.as_ref().unwrap();
        (gp.open_rad, gp.closed_rad)
    };
    start_q[g_idx] = open_rad;
    let (mut drv, _) = MotionTestDriver::new(start_q.clone(), Duration::ZERO, Duration::ZERO, None);
    drv.caps.torque_feedback = false;
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let (mut ws, gen, _, sup, _) = common::torqued_rig(sup, &start_q).await;
    let epoch = sup.get_observation().await.epoch_id;

    // 2a. calibration_busy beats malformed preset (409 MOTION_BUSY).
    sup.inner().write().await.calibration_busy = true;
    let r = common::ws_reply(
        &mut ws,
        json!({"type":"preset","name":"custom","positions":"not-an-array","generation":gen,"epoch_id":epoch}),
        &["preset_ack", "preset_reject"],
    ).await;
    assert_eq!((&r["type"], &r["status"], &r["code"]), (&json!("preset_reject"), &json!(409), &json!("MOTION_BUSY")), "{r}");
    sup.inner().write().await.calibration_busy = false;

    // 2b. Commissioning rules: home refused, speed_scale > 0.2 refused, gripper close without torque_feedback refused.
    let home = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/home", json!({})).await;
    assert_eq!((&home["type"], &home["code"]), (&json!("command_reject"), &json!("COMMISSIONING_REQUIRED")), "{home}");
    let mut target = sup.get_observation().await.commanded_positions;
    target[0] += 0.1;
    let fast = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/jog", json!({"target_positions": target, "speed_scale": 0.25})).await;
    assert_eq!((&fast["type"], &fast["code"]), (&json!("command_reject"), &json!("COMMISSIONING_REQUIRED")), "{fast}");
    let bad_scale = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/jog", json!({"target_positions": target, "speed_scale": 0.01})).await;
    assert_eq!((&bad_scale["type"], &bad_scale["code"]), (&json!("command_reject"), &json!("INVALID_COMMAND")), "{bad_scale}");

    let mut close_target = sup.get_observation().await.commanded_positions;
    close_target[g_idx] = closed_rad;
    let g_err = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/jog", json!({"target_positions": close_target, "speed_scale": 0.2})).await;
    assert_eq!((&g_err["type"], &g_err["code"]), (&json!("command_reject"), &json!("UNSUPPORTED")), "{g_err}");

    let ok = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/jog", json!({"target_positions": target, "speed_scale": 0.2})).await;
    assert_eq!(ok["type"], "command_ack", "{ok}");
    assert!(sup.inner().read().await.active_trajectory.as_ref().and_then(|t| t.motion_vel_caps).is_some());
    sup.stop_motion().await;

    // 2c. motion_config_revision invalidation with unchanged workspace_revision + zero mutation on refusal.
    let mut inner = sup.inner().write().await;
    inner.config.active_tool.commissioning_required = false;
    inner.bump_motion_config_revision();
    let snap = MotionPlanSnapshot::from_supervisor(&inner);
    let mut jog_t = inner.commanded_positions.clone();
    jog_t[0] += 0.05;
    let plan = req::prepare_motion(&snap, &MotionRequest::jog(jog_t, Some(0.5), None)).unwrap();
    let ws_rev_before = inner.workspace_revision;
    inner.config.planned_max_velocity = Some(0.5);
    inner.bump_motion_config_revision();
    assert_eq!(inner.workspace_revision, ws_rev_before);
    let stamp = inner.current_admission_stamp();
    let ep = inner.safety.epoch_id();
    let ml_before = inner.motion_limit.clone();
    let cmd_before = inner.commanded_positions.clone();
    let err = req::install_jog(&mut inner, stamp, &plan.slot, None, Some(ep), false).unwrap_err();
    assert_eq!(err, MotionInstallError::PlanStale);
    assert!(inner.active_trajectory.is_none());
    assert_eq!(inner.motion_limit, ml_before);
    assert_eq!(inner.commanded_positions, cmd_before);

    // 2d. Unsupported unload (torque_feedback == false with support_torque_limits configured).
    let park_snap = MotionPlanSnapshot::from_supervisor(&inner);
    let park_plan = req::prepare_motion(&park_snap, &MotionRequest::park()).unwrap();
    let stamp = inner.current_admission_stamp();
    let ep = inner.safety.epoch_id();
    assert_eq!(
        req::install_park(&mut inner, stamp, &park_plan.slot, None, Some(ep), false).unwrap_err(),
        MotionInstallError::UnsupportedUnload
    );

    // 2e. Review regressions:
    //   - Blocker #1: refresh_for_drift_retry refuses stale config after workspace/motion revision bump.
    let stale_cfg = snap.config.clone();
    assert!(MotionPlanSnapshot::refresh_for_drift_retry(&inner, stale_cfg).is_none());
    //   - Major #1: torque-hold clamped jog fails PlanStale if hold clears before install;
    //     preserves clamp attribution (`motion_limit`) when hold stays active.
    let q0 = inner.commanded_positions[0];
    inner.torque_guard.hold = Some(armd::torque_guard::TorqueHold {
        joint: 0, sign: 1.0, hold_q: q0 + 0.05, anchor_q: q0 + 0.06,
        torque_nm: 10.0, threshold_nm: 8.0, since_s: 0.0, push_in: false, retreat: armd::torque_guard::Retreat::Unused,
    });
    let hold_snap = MotionPlanSnapshot::from_supervisor(&inner);
    let mut clamped_t = inner.commanded_positions.clone();
    clamped_t[0] = q0 + 0.20;
    let clamped_plan = req::prepare_motion(&hold_snap, &MotionRequest::jog(clamped_t, Some(0.5), None)).unwrap();
    assert!((clamped_plan.slot.target[0] - (q0 + 0.05)).abs() < 1e-5);
    assert!((clamped_plan.slot.requested_target[0] - (q0 + 0.20)).abs() < 1e-5);
    inner.torque_guard.hold = None;
    let stamp = inner.current_admission_stamp();
    let ep = inner.safety.epoch_id();
    assert_eq!(req::install_jog(&mut inner, stamp, &clamped_plan.slot, None, Some(ep), false).unwrap_err(), MotionInstallError::PlanStale);
    //   - Major #3: live calibration_busy blocks commit even after Lock #1 preflight passed.
    let clean_snap = MotionPlanSnapshot::from_supervisor(&inner);
    let mut t2 = inner.commanded_positions.clone();
    t2[0] += 0.05;
    let plan2 = req::prepare_motion(&clean_snap, &MotionRequest::jog(t2, Some(0.5), None)).unwrap();
    inner.calibration_busy = true;
    let stamp = inner.current_admission_stamp();
    assert_eq!(req::install_jog(&mut inner, stamp, &plan2.slot, None, Some(ep), false).unwrap_err(), MotionInstallError::MotionBusy);
    inner.calibration_busy = false;
}

// ============================================================================
// 3. Executor Broker Jog/Home/Park, Supersession, and B1/B2 Zero-Alloc Proof
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_broker_jog_home_park_supersession_and_zero_alloc_b1_b2() {
    let clock = ScriptedClock::new();
    let (drv, _) = MotionTestDriver::new(vec![0.4, 0.2], Duration::from_millis(4), Duration::from_millis(4), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
    let period = Duration::from_millis(5);
    let period_ns = period.as_nanos() as u64;
    let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;

    clock.run_steps(period_ns, 2);
    let auth = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);

    let hub_cfg = sup.hub().motion_config().expect("motion_config cached on start_executor");
    let plan_snap = MotionPlanSnapshot::from_published(&sup.hub().latest().unwrap(), hub_cfg.clone()).expect("from_published");
    let jog_plan = req::prepare_motion(&plan_snap, &MotionRequest::jog(vec![0.6, 0.3], Some(0.4), Some(0.5))).unwrap();
    let mut stale_slot = jog_plan.slot;
    stale_slot.start[0] += 0.2;
    let mut bad_rev_slot = jog_plan.slot;
    bad_rev_slot.motion_config_revision += 99;

    let dl = clock.now() + Duration::from_secs(5);
    let r_stale = handle.broker().lock().unwrap().submit(ClientId(1), 2, auth, dl, Submit::Install(&|s: &mut PlanSlot| req::stage_jog(s, &stale_slot))).unwrap();
    let r_rev = handle.broker().lock().unwrap().submit(ClientId(1), 3, auth, dl, Submit::Install(&|s: &mut PlanSlot| req::stage_jog(s, &bad_rev_slot))).unwrap();
    let r_ep = handle.broker().lock().unwrap().submit(ClientId(1), 4, Authority { epoch: auth.epoch + 7, ..auth }, dl, Submit::Install(&|s: &mut PlanSlot| req::stage_jog(s, &jog_plan.slot))).unwrap();
    let r_jog = handle.broker().lock().unwrap().submit(ClientId(1), 5, auth, dl, Submit::Install(&|s: &mut PlanSlot| req::stage_jog(s, &jog_plan.slot))).unwrap();
    clock.run_steps(period_ns, 5); // 1 descriptor per release: drain 3 refusals + install jog + advance 1 tick

    let snap2 = MotionPlanSnapshot::from_published(&sup.hub().latest().unwrap(), hub_cfg.clone()).unwrap();
    let home_plan = req::prepare_motion(&snap2, &MotionRequest::home(Some(0.2))).unwrap();
    let r_home = handle.broker().lock().unwrap().submit(ClientId(1), 6, auth, dl, Submit::InstallHome(&|s: &mut PlanSlot| req::stage_home(s, &home_plan.slot))).unwrap();
    let home_ticks = ((home_plan.slot.duration / 0.005).ceil() as u64) + 200;
    clock.run_steps(period_ns, home_ticks);

    let replies = drain_replies(&handle);
    assert!(replies.iter().any(|r| matches!(r, Reply::Refused { request, code: "PLAN_STALE" } if *request == r_stale)), "{replies:?}");
    assert!(replies.iter().any(|r| matches!(r, Reply::Refused { request, code: "PLAN_STALE" } if *request == r_rev)), "{replies:?}");
    assert!(replies.iter().any(|r| matches!(r, Reply::Refused { request, code: "EPOCH_MISMATCH" } if *request == r_ep)), "{replies:?}");
    let sup_idx = replies.iter().position(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Superseded, .. } if *request == r_jog)).expect("jog superseded");
    let home_ack_idx = replies.iter().position(|r| matches!(r, Reply::Ack { request, .. } if *request == r_home)).expect("home ack");
    assert!(sup_idx < home_ack_idx, "Superseded must precede new Ack");
    assert!(replies.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_home)), "{replies:?}");

    // InstallPark -> moves to rest, settles 150ms, disables through single TorqueTransition::Disable + DisableConfirm.
    let snap3 = MotionPlanSnapshot::from_published(&sup.hub().latest().unwrap(), hub_cfg).unwrap();
    let park_plan = req::prepare_motion(&snap3, &MotionRequest::park()).unwrap();
    let r_park = handle.broker().lock().unwrap().submit(ClientId(1), 7, auth, clock.now() + Duration::from_secs(10), Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park_plan.slot))).unwrap();
    let mut park_done = false;
    for _ in 0..320 {
        clock.run_steps(period_ns, 2);
        let rs = drain_replies(&handle);
        if rs.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_park)) {
            park_done = true;
            break;
        }
    }
    assert!(park_done, "InstallPark did not reach Outcome::Completed");
    let final_snap = sup.hub().latest().unwrap();
    assert!(!final_snap.obs.is_torqued);
    assert_eq!(final_snap.obs.safety_state, SafetyState::Idle);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "B2 violation: RT thread allocated during jog/home/park/disable");
    assert_eq!(sup.inner().control_thread_attempts(), 0, "B1 violation: RT thread touched SupervisorLock");

    // Executor WS entry stays at 503 EXECUTOR_ROUTING_PENDING until PR L lands.
    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let r = common::rpc(&mut ws, 0, final_snap.obs.epoch_id, "/api/v1/motion/jog", json!({"target_positions": [0.0, 0.0]})).await;
    assert_eq!((&r["type"], &r["status"], &r["code"]), (&json!("command_reject"), &json!(503), &json!("EXECUTOR_ROUTING_PENDING")), "{r}");
    drop(handle);
}

// ============================================================================
// 4. Park Disable Unconfirmed, Stop Before/After Disable Write, E-Stop Inhibit, Workspace
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn park_disable_confirm_failure_stop_boundary_and_estop_inhibit() {
    let period = Duration::from_millis(5);
    let period_ns = period.as_nanos() as u64;

    // (a) Missing disable confirmation -> FEEDBACK_UNCONFIRMED + StaticFault::DisableUnconfirmed + 0 alloc.
    {
        let clock = ScriptedClock::new();
        let (drv, knobs) = MotionTestDriver::new(vec![0.0, 0.0], Duration::from_millis(2), Duration::from_millis(2), Some(clock.clone()));
        knobs.confirm_disable.store(false, Ordering::Release);
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
        clock.run_steps(period_ns, 2);
        let auth = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);
        let snap = MotionPlanSnapshot::from_published(&sup.hub().latest().unwrap(), sup.hub().motion_config().unwrap()).unwrap();
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let r_park = handle.broker().lock().unwrap().submit(ClientId(1), 2, auth, clock.now() + Duration::from_secs(10), Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park.slot))).unwrap();
        let mut failed_code = None;
        let mut all_rs = Vec::new();
        for _ in 0..320 {
            clock.run_steps(period_ns, 2);
            for r in drain_replies(&handle) {
                all_rs.push(r);
                if let Reply::Terminal { request, outcome: Outcome::Failed(code), .. } = r {
                    if request == r_park { failed_code = Some(code); }
                }
            }
            if failed_code.is_some() { break; }
        }
        assert_eq!(failed_code, Some("FEEDBACK_UNCONFIRMED"), "replies={all_rs:?} obs={:?}", sup.hub().latest().map(|s| (s.obs.safety_state.clone(), s.obs.is_torqued, s.obs.action.clone())));
        let snap_end = sup.hub().latest().unwrap();
        assert!(snap_end.obs.is_torqued, "torque must remain reported ON when disable unconfirmed");
        assert!(matches!(&snap_end.obs.safety_state, SafetyState::Fault(armd::safety::FaultReason::Other(m)) if m.contains("not been confirmed")), "{:?}", snap_end.obs.safety_state);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
        drop(handle);
    }

    // (b) Stop BEFORE first disable write cancels park & holds torque; Stop AFTER first disable write completes disable.
    for stop_after_write in [false, true] {
        let clock = ScriptedClock::new();
        let (drv, knobs) = MotionTestDriver::new(vec![0.0, 0.0], Duration::from_millis(2), Duration::from_millis(25), Some(clock.clone()));
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
        clock.run_steps(period_ns, 2);
        let auth = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);
        let snap = MotionPlanSnapshot::from_published(&sup.hub().latest().unwrap(), sup.hub().motion_config().unwrap()).unwrap();
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let r_park = handle.broker().lock().unwrap().submit(ClientId(1), 2, auth, clock.now() + Duration::from_secs(10), Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park.slot))).unwrap();
        if !stop_after_write {
            clock.run_steps(period_ns, 5);
            assert!(!knobs.disable_started.load(Ordering::Acquire));
            let sg = handle.stop_requests().request();
            clock.run_steps(period_ns, 4);
            assert_eq!(handle.stop_requests().applied(), sg);
            let rs = drain_replies(&handle);
            assert!(rs.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Stopped, .. } if *request == r_park)), "{rs:?}");
            let s = sup.hub().latest().unwrap();
            assert!(s.obs.is_torqued && s.obs.safety_state == SafetyState::Armed);
        } else {
            for _ in 0..520 {
                clock.run_steps(period_ns, 1);
                if knobs.disable_started.load(Ordering::Acquire) { break; }
            }
            assert!(knobs.disable_started.load(Ordering::Acquire), "park disable write did not begin");
            let sg = handle.stop_requests().request();
            clock.run_steps(period_ns, 20);
            assert_eq!(handle.stop_requests().applied(), sg);
            let s = sup.hub().latest().unwrap();
            assert!(!s.obs.is_torqued && s.obs.safety_state == SafetyState::Idle, "physical disable must complete after first write: {:?}", s.obs.safety_state);
        }
        drop(handle);
    }

    // (c) E-stop inhibit during park disable does not latch DriverWriteFailed and preserves disable_pending.
    {
        let (drv, knobs) = MotionTestDriver::new(vec![0.0, 0.0], Duration::ZERO, Duration::ZERO, None);
        knobs.estop_inhibit_disable.store(true, Ordering::Release);
        let mut inner = SupervisorInner::new(parkable_2dof_config()).with_driver(Box::new(drv));
        let stamp = inner.current_admission_stamp();
        SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
        inner.tick(0.01);
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let stamp = inner.current_admission_stamp();
        let ep = inner.safety.epoch_id();
        req::install_park(&mut inner, stamp, &park.slot, None, Some(ep), false).unwrap();
        for _ in 0..260 {
            inner.tick(0.01);
            if knobs.disable_started.load(Ordering::Acquire) { break; }
        }
        assert!(knobs.disable_started.load(Ordering::Acquire));
        assert!(inner.is_torqued && inner.disable_pending);
        assert_ne!(inner.safety.fault_code(), Some(FaultCode::DriverWriteFailed));
        assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("ESTOP_ACTIVE"));
    }

    // (d) Park resting-pose workspace violation fails off-lock with phase="park".
    {
        let mut cfg = ArmConfig::rebot_dm();
        let mut ws = WorkspacePolicy::rebot_dm();
        ws.keepout_boxes.push(KeepoutBox { name: "block_rest".into(), min_m: [-2.0, -2.0, -2.0], max_m: [2.0, 2.0, 2.0] });
        cfg.workspace = Some(ws);
        let inner = SupervisorInner::new(cfg);
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let err = req::prepare_motion(&snap, &MotionRequest::park()).unwrap_err();
        match err {
            ArmError::Safety(SafetyError::WorkspaceViolation { detail, .. }) => {
                assert_eq!(detail.violation_phase.as_deref(), Some("park"));
            }
            other => panic!("expected park WorkspaceViolation, got {other:?}"),
        }
    }
}

// ============================================================================
// 5. holding_after_clear Bounded Rearm + Post-Confirm Planning + ContinuePark
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn holding_after_clear_bounded_rearm_and_continue_park() {
    let period = Duration::from_millis(5);
    let period_ns = period.as_nanos() as u64;
    let clock = ScriptedClock::new();
    let (drv, knobs) = MotionTestDriver::new(vec![0.4, 0.2], Duration::from_millis(12), Duration::from_millis(4), Some(clock.clone()));
    let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
    let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
    clock.run_steps(period_ns, 2);
    let _ = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);

    // E-stop while torqued, then ClearFault -> Idle + is_torqued + holding_after_clear.
    // Step ScriptedClock in the background so both trigger_e_stop and clear_fault_async receive RT acks.
    let maint_done = Arc::new(AtomicBool::new(false));
    let maint_done_bg = maint_done.clone();
    let clock_step = clock.clone();
    let step_jh = tokio::task::spawn_blocking(move || {
        while !maint_done_bg.load(Ordering::Acquire) {
            clock_step.run_steps(period_ns, 1);
            std::thread::sleep(Duration::from_millis(1));
        }
    });
    let _ = sup.trigger_e_stop().await;
    handle.clear_fault_async().await.unwrap().expect("clear_fault succeeds");
    maint_done.store(true, Ordering::Release);
    step_jh.await.unwrap();
    clock.run_steps(period_ns, 2);
    let held_snap = sup.hub().latest().unwrap();
    assert!(held_snap.rt.holding_after_clear && held_snap.obs.is_torqued && held_snap.obs.safety_state == SafetyState::Idle);

    // Nudge physical joint 0 upon rearm confirmation to prove continuation plans from post-confirm measured state.
    *knobs.nudge_q0_on_enable.lock().unwrap() = Some(0.55);
    let hub_cfg = sup.hub().motion_config().unwrap();
    let p_snap = MotionPlanSnapshot::from_published(&held_snap, hub_cfg.clone()).unwrap();
    let park_rearm = req::prepare_motion(&p_snap, &MotionRequest::park()).unwrap();
    assert!(park_rearm.slot.for_holding_after_clear);
    let auth_clear = auth_from_hub(&sup, &handle);
    let r_park = handle.broker().lock().unwrap().submit(
        ClientId(1), 3, auth_clear, clock.now() + Duration::from_secs(10),
        Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park_rearm.slot)),
    ).unwrap();
    clock.run_steps(period_ns, 1);
    let ack_rs = drain_replies(&handle);
    let park_action = ack_rs.iter().find_map(|r| match r {
        Reply::Ack { request, action: Some(a), .. } if *request == r_park => Some(a.0),
        _ => None,
    }).unwrap_or_else(|| panic!("InstallPark ACK with action_id; replies={ack_rs:?}"));

    // Advance until multi-tick ParkRearm confirms and StateHub publishes holding_after_clear == false + q0 == 0.55.
    let mut ready_snap = None;
    for _ in 0..20 {
        clock.run_steps(period_ns, 1);
        let s = sup.hub().latest().unwrap();
        if !s.rt.holding_after_clear && s.obs.safety_state == SafetyState::Armed {
            ready_snap = Some(s);
            break;
        }
    }
    let ready_snap = ready_snap.expect("ParkRearm confirmed on StateHub");
    assert!((ready_snap.obs.commanded_positions[0] - 0.55).abs() < 1e-4, "must seed from post-rearm measured state");

    // Plan continuation from post-rearm measured state and submit ContinuePark.
    let cont_snap = MotionPlanSnapshot::from_published(&ready_snap, hub_cfg).unwrap();
    let cont_plan = req::prepare_motion(&cont_snap, &MotionRequest::park()).unwrap();
    assert!((cont_plan.slot.start[0] - 0.55).abs() < 1e-4);
    let r_cont = handle.broker().lock().unwrap().submit(
        ClientId(1), 4, auth_from_hub(&sup, &handle), clock.now() + Duration::from_secs(10),
        Submit::ContinuePark(&|s: &mut PlanSlot| req::stage_park_continuation(s, park_action, &cont_plan.slot)),
    ).unwrap();
    let mut cont_acked = false;
    let mut park_terminals = 0usize;
    let mut cont_terminals = 0usize;
    for _ in 0..320 {
        clock.run_steps(period_ns, 2);
        for r in drain_replies(&handle) {
            match r {
                Reply::Ack { request, .. } if request == r_cont => cont_acked = true,
                Reply::Terminal { request, outcome: Outcome::Completed, .. } if request == r_park => park_terminals += 1,
                Reply::Terminal { request, .. } if request == r_cont => cont_terminals += 1,
                _ => {}
            }
        }
        if park_terminals > 0 { break; }
    }
    assert!(cont_acked && park_terminals == 1 && cont_terminals == 0, "single action + single terminal");
    let end_snap = sup.hub().latest().unwrap();
    assert!(!end_snap.obs.is_torqued && end_snap.obs.safety_state == SafetyState::Idle);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    drop(handle);

    // 5b. Stop during WaitingRearm aborts ParkRearm, holds torque, and ends park Stopped;
    //     failed ParkRearm ends park Failed while keeping holding torque intact.
    for fail_enable in [false, true] {
        let clock = ScriptedClock::new();
        let (drv, knobs) = MotionTestDriver::new(vec![0.4, 0.2], Duration::from_millis(20), Duration::from_millis(4), Some(clock.clone()));
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
        clock.run_steps(period_ns, 2);
        let _ = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);
        let maint_done = Arc::new(AtomicBool::new(false));
        let maint_done_bg = maint_done.clone();
        let clock_step = clock.clone();
        let step_jh = tokio::task::spawn_blocking(move || {
            while !maint_done_bg.load(Ordering::Acquire) {
                clock_step.run_steps(period_ns, 1);
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        let _ = sup.trigger_e_stop().await;
        handle.clear_fault_async().await.unwrap().unwrap();
        maint_done.store(true, Ordering::Release);
        step_jh.await.unwrap();
        clock.run_steps(period_ns, 2);
        if fail_enable { knobs.fail_next_enable.store(true, Ordering::Release); }
        let held_snap = sup.hub().latest().unwrap();
        let p_snap = MotionPlanSnapshot::from_published(&held_snap, sup.hub().motion_config().unwrap()).unwrap();
        let park_rearm = req::prepare_motion(&p_snap, &MotionRequest::park()).unwrap();
        let r_park = handle.broker().lock().unwrap().submit(
            ClientId(1), 3, auth_from_hub(&sup, &handle), clock.now() + Duration::from_secs(10),
            Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park_rearm.slot)),
        ).unwrap();
        clock.run_steps(period_ns, 2);
        if !fail_enable {
            let sg = handle.stop_requests().request();
            clock.run_steps(period_ns, 4);
            assert_eq!(handle.stop_requests().applied(), sg);
            let rs = drain_replies(&handle);
            assert!(rs.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Stopped, .. } if *request == r_park)), "{rs:?}");
            let s = sup.hub().latest().unwrap();
            assert!(s.obs.is_torqued && s.rt.holding_after_clear);
        } else {
            clock.run_steps(period_ns, 10);
            let rs = drain_replies(&handle);
            assert!(rs.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Failed(_), .. } if *request == r_park)), "{rs:?}");
            let s = sup.hub().latest().unwrap();
            assert!(s.obs.is_torqued);
            // Blocker #4: failed ParkRearm on RT performs strictly zero heap allocs/frees.
            assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
        }
        drop(handle);
    }

    // 5c. Blocker #3 & Blocker #5 regressions:
    //   - 7-DOF park carrying an object (gripper contact -> Opening -> Closing -> DisableConfirm)
    //     completes with AllocCounts::ZERO on armd-control.
    //   - E-stop arriving AFTER first disable write preserves SafetyState::EStop, marks !is_torqued
    //     once confirmed, and performs AllocCounts::ZERO (no heap SafetyError constructed).
    {
        let mut dm_cfg = ArmConfig::rebot_dm();
        dm_cfg.workspace = None;
        dm_cfg.lease_timeout_ms = 60_000;
        let g_idx = dm_cfg.gripper_index().unwrap();
        let gp = dm_cfg.gripper_policy.as_ref().unwrap().clone();
        let mut q0 = dm_cfg.resting_positions().to_vec();
        q0[g_idx] = gp.closed_rad + (gp.open_rad - gp.closed_rad) * 0.5;
        let close_dir = if gp.closed_rad >= gp.open_rad { 1.0_f32 } else { -1.0_f32 };
        let clock = ScriptedClock::new();
        let (mut drv, knobs) = MotionTestDriver::new(q0, Duration::from_millis(2), Duration::from_millis(20), Some(clock.clone()));
        drv.caps.torque_feedback = true;
        let sup = Arc::new(ArmSupervisor::new(dm_cfg).with_driver(Box::new(drv)));
        {
            // Simulate an object stalled in the jaw so 7-DOF park triggers step_park_release (Opening -> Closing) on RT.
            let mut inner = sup.inner().write().await;
            inner.gripper_stalled = true;
            inner.gripper_stall_sign = close_dir;
        }
        let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
        clock.run_steps(period_ns, 2);
        let auth = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);
        let snap = MotionPlanSnapshot::from_published(&sup.hub().latest().unwrap(), sup.hub().motion_config().unwrap()).unwrap();
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let _r_park = handle.broker().lock().unwrap().submit(
            ClientId(1), 2, auth, clock.now() + Duration::from_secs(15),
            Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park.slot)),
        ).unwrap();
        for _ in 0..1400 {
            clock.run_steps(period_ns, 1);
            if knobs.disable_started.load(Ordering::Acquire) { break; }
        }
        assert!(knobs.disable_started.load(Ordering::Acquire), "7-DOF park with gripper release did not reach disable");
        // Trigger E-stop AFTER first disable write while disable confirmation is in flight!
        let estop_done = Arc::new(AtomicBool::new(false));
        let estop_done_bg = estop_done.clone();
        let clock_step = clock.clone();
        let step_jh = tokio::task::spawn_blocking(move || {
            while !estop_done_bg.load(Ordering::Acquire) {
                clock_step.run_steps(period_ns, 1);
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        let _ = sup.trigger_e_stop().await;
        estop_done.store(true, Ordering::Release);
        step_jh.await.unwrap();
        clock.run_steps(period_ns, 15);
        let end_snap = sup.hub().latest().unwrap();
        assert_eq!(end_snap.obs.safety_state, SafetyState::EStop, "must preserve latched EStop");
        assert!(!end_snap.obs.is_torqued, "confirmed disable must clear is_torqued even under EStop");
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Blocker #5: stopped disable confirm must be 0 alloc");
        drop(handle);
    }
}

// ============================================================================
// 6. Legacy MIT Moving-Park Exclusion + Executor MIT Gate
// ============================================================================

fn mit_config_j4() -> ArmConfig {
    let mut jt = JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
    let j4 = &mut jt.joints[3];
    j4.strategy = TrackingStrategy::Mit;
    j4.kp = 10.0; j4.kd = 0.3; j4.hold_kp = Some(12.0); j4.hold_kd = Some(0.4);
    j4.effort_limit_nm = 3.0; j4.gravity_scale = 1.0;
    ArmConfig { lease_timeout_ms: 600_000, workspace: None, joint_tracking: Some(jt), ..ArmConfig::rebot_dm() }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_mit_moving_park_off_lock_exclusion_and_executor_mit_gate() {
    // 6a. Executor start_executor keeps rejecting MIT through the existing gate.
    let mit_sup = Arc::new(ArmSupervisor::new(mit_config_j4()));
    let gate_err = mit_sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(5))).await;
    assert!(gate_err.is_err(), "executor must keep rejecting MIT config");

    // 6b. Legacy MIT moving-park: single-action exclusion across WaitingMitHandback AND ReadyForPlan.
    let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect();
    let bus = FakeBus::with_clock(&limits, armd::transport::MockClock::new());
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(limits.clone()).unwrap();
    bus.with(|s| { s.motors[3].set_reg_u32(damiao::RID_TIMEOUT, 2000); s.push_all_feedback(); });
    drv.read_joint_states().unwrap();
    let mut inner = SupervisorInner::new(mit_config_j4()).with_driver(Box::new(drv));
    let l = limits[3];
    let rec = MitQualificationFile {
        version: mit::QUALIFICATION_VERSION,
        joints: vec![MitJointQualification {
            profile_id: "rebot_b601dm".into(), joint: 4, joint_name: ArmConfig::rebot_dm().joint_names[3].clone(),
            zero_fingerprint: mit::zero_fingerprint(0.0), motor_id: 4, reply_id: 0x14, esc_id: 4, mst_id: 0x14, sw_ver: SW_VER,
            pmax: l.p_max, vmax: l.v_max, tmax: l.t_max, timeout_raw: 2000, timeout_unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED,
            timeout_unit_verified: true, tested_response: "disabled".into(), measured_expiry_ms: 101.0, worst_latency_ms: 8.0,
            load_envelope: "unloaded, supported".into(), max_kp: 15.0, max_kd: 0.5, effort_ceiling_nm: 4.0,
            date: "2026-11-22T00:00:00Z".into(), trace_id: "test".into(),
        }],
        invalidated: None,
    };
    inner.set_calibration(Some(armd::calibration::CalibrationFile { mit_qualification: serde_json::to_value(&rec).ok(), ..Default::default() }));
    let stamp = inner.current_admission_stamp();
    SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
    inner.tick(0.01);
    assert!(inner.mit_active());

    let snap = MotionPlanSnapshot::from_supervisor(&inner);
    let mut jog_t = inner.commanded_positions.clone();
    jog_t[3] -= 0.3;
    let jog_plan = req::prepare_motion(&snap, &MotionRequest::jog(jog_t.clone(), Some(0.5), None)).unwrap();
    let stamp = inner.current_admission_stamp();
    let ep = inner.safety.epoch_id();
    req::install_jog(&mut inner, stamp, &jog_plan.slot, None, Some(ep), false).unwrap();
    for _ in 0..15 { inner.tick(0.01); }
    assert!(inner.commanded_velocities[3].abs() > 0.01);

    let park_snap = MotionPlanSnapshot::from_supervisor(&inner);
    let park_plan = req::prepare_motion(&park_snap, &MotionRequest::park()).unwrap();
    let stamp = inner.current_admission_stamp();
    let ep = inner.safety.epoch_id();
    let installed = req::install_park(&mut inner, stamp, &park_plan.slot, None, Some(ep), false).unwrap();
    let park_id = match installed {
        MotionInstalled::DeferredContinuation { action_id, .. } => action_id,
        other => panic!("expected DeferredContinuation, got {other:?}"),
    };
    assert_eq!(inner.park_continuation().unwrap().phase, ParkContinuationPhase::WaitingMitHandback);
    assert!(inner.mit_motion_blocker().is_some());
    assert_eq!(req::verify_motion_authority(&inner, req::MotionKind::Jog, None, &jog_t, Some(inner.safety.epoch_id()), false), Err(MotionInstallError::MotionBusy));

    for _ in 0..200 {
        inner.tick(0.01);
        if inner.park_continuation().is_some_and(|c| c.phase == ParkContinuationPhase::ReadyForPlan) { break; }
    }
    assert_eq!(inner.park_continuation().unwrap().phase, ParkContinuationPhase::ReadyForPlan);
    assert!(!inner.mit_active(), "MIT must stay inactive during ReadyForPlan window");
    inner.tick(0.01);
    assert!(!inner.mit_active(), "mit_maybe_activate must not reactivate MIT before continuation installs");
    assert_eq!(req::verify_motion_authority(&inner, req::MotionKind::Jog, None, &jog_t, Some(inner.safety.epoch_id()), false), Err(MotionInstallError::MotionBusy));

    let ready_snap = MotionPlanSnapshot::from_supervisor(&inner);
    let cont_plan = req::prepare_motion(&ready_snap, &MotionRequest::park()).unwrap();
    let saved_ws = inner.workspace_revision;
    inner.workspace_revision += 1;
    let ep = inner.safety.epoch_id();
    assert_eq!(
        req::install_park_continuation(&mut inner, park_id, &cont_plan.slot, None, Some(ep), false),
        Err(MotionInstallError::PlanStale)
    );
    inner.workspace_revision = saved_ws;
    req::install_park_continuation(&mut inner, park_id, &cont_plan.slot, None, Some(ep), false).unwrap();
    assert!(inner.park_continuation().is_none() && inner.pending_normal_unload);
}

// ============================================================================
// 7. Round 2 B1: Legacy WS Stop cancels ParkRearm & continuation before ACK
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b1_legacy_ws_stop_cancels_park_rearm_and_continuation_before_ack() {
    // 7a. Legacy WS Stop during WaitingRearm cancels park_continuation + ParkRearm BEFORE ACK;
    //     no motion occurs after the Stop ACK.
    {
        let (drv, _knobs) = MotionTestDriver::new(vec![0.8, 0.4], Duration::from_millis(250), Duration::ZERO, None);
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let (mut ws, gen, _, sup, addr) = common::torqued_rig(sup, &[0.8, 0.4]).await;
        assert!(reqwest::Client::new().post(format!("http://{addr}/api/v1/safety/e_stop")).send().await.unwrap().status().is_success());
        let ep = sup.get_observation().await.epoch_id;
        assert_eq!(common::rpc(&mut ws, gen, ep, "/api/v1/safety/clear_fault", json!({})).await["type"], "command_ack");
        assert!(sup.inner().read().await.holding_after_clear());

        let ep = sup.get_observation().await.epoch_id;
        let park = common::rpc(&mut ws, gen, ep, "/api/v1/motion/unload", json!({})).await;
        assert_eq!(park["type"], "command_ack", "{park}");
        assert_eq!(sup.inner().read().await.park_continuation().map(|c| c.phase), Some(ParkContinuationPhase::WaitingRearm));

        let stop = common::rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop["type"], "command_ack", "{stop}");
        {
            let inner = sup.inner().read().await;
            assert!(inner.park_continuation().is_none(), "Stop must cancel park_continuation before ACK");
            assert!(!inner.torque_transition_pending(), "Stop must abort ParkRearm before ACK");
            assert!(inner.active_trajectory.is_none());
            assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("STOPPED"));
        }
        let q_after_stop = sup.get_observation().await.commanded_positions;
        tokio::time::sleep(Duration::from_millis(350)).await;
        let obs_later = sup.get_observation().await;
        assert!(obs_later.is_torqued && obs_later.commanded_positions == q_after_stop, "no motion after Stop ACK");
        assert!(sup.inner().read().await.active_trajectory.is_none());
    }

    // 7b. Legacy WS Stop while continuation worker is paused mid-planning (ReadyForPlan):
    //     Stop ACKs cleanly; unpausing continuation worker produces ZERO motion after Stop ACK.
    {
        struct PauseGuard(String);
        impl Drop for PauseGuard {
            fn drop(&mut self) {
                armd::api::ws::set_test_ws_blocking_plan_paused(&self.0, false);
            }
        }
        let (drv, _knobs) = MotionTestDriver::new(vec![0.8, 0.4], Duration::from_millis(60), Duration::ZERO, None);
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let (mut ws, gen, _, sup, addr) = common::torqued_rig(sup, &[0.8, 0.4]).await;
        assert!(reqwest::Client::new().post(format!("http://{addr}/api/v1/safety/e_stop")).send().await.unwrap().status().is_success());
        let ep = sup.get_observation().await.epoch_id;
        assert_eq!(common::rpc(&mut ws, gen, ep, "/api/v1/safety/clear_fault", json!({})).await["type"], "command_ack");
        let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();

        let ep = sup.get_observation().await.epoch_id;
        let park = common::rpc(&mut ws, gen, ep, "/api/v1/motion/unload", json!({})).await;
        assert_eq!(park["type"], "command_ack", "{park}");
        assert_eq!(sup.inner().read().await.park_continuation().map(|c| c.phase), Some(ParkContinuationPhase::WaitingRearm));

        // Pause continuation spawn_blocking before ParkRearm confirms, then wait until the
        // continuation worker has entered spawn_blocking in ReadyForPlan.
        armd::api::ws::set_test_ws_blocking_plan_paused(&conn_id, true);
        let _unpause = PauseGuard(conn_id.clone());
        let wait_until = tokio::time::Instant::now() + Duration::from_secs(3);
        while armd::api::ws::test_ws_blocking_plan_blocked_count(&conn_id) < 1 {
            assert!(tokio::time::Instant::now() < wait_until, "continuation worker never reached spawn_blocking");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert_eq!(sup.inner().read().await.park_continuation().map(|c| c.phase), Some(ParkContinuationPhase::ReadyForPlan));

        let stop = common::rpc(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
        assert_eq!(stop["type"], "command_ack", "{stop}");
        {
            let inner = sup.inner().read().await;
            assert!(inner.park_continuation().is_none(), "Stop must cancel ReadyForPlan continuation before ACK");
            assert!(inner.active_trajectory.is_none());
            assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("STOPPED"));
        }
        let q_after_stop = sup.get_observation().await.commanded_positions;

        // Release the paused continuation worker and prove no motion starts after Stop ACK.
        drop(_unpause);
        tokio::time::sleep(Duration::from_millis(200)).await;
        let obs_later = sup.get_observation().await;
        assert!(obs_later.is_torqued && obs_later.commanded_positions == q_after_stop, "no motion after Stop ACK");
        let inner = sup.inner().read().await;
        assert!(inner.active_trajectory.is_none() && !inner.pending_normal_unload && inner.park_continuation().is_none());
    }

    // 7c. Executor Stop during continuation planning (ReadyForPlan) cancels park_continuation
    //     before ACK, and a subsequent continuation install is refused with no motion.
    {
        let period = Duration::from_millis(5);
        let period_ns = period.as_nanos() as u64;
        let clock = ScriptedClock::new();
        let (drv, _knobs) = MotionTestDriver::new(vec![0.8, 0.4], Duration::from_millis(10), Duration::ZERO, Some(clock.clone()));
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
        clock.run_steps(period_ns, 2);
        let _ = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);
        let done = Arc::new(AtomicBool::new(false));
        let done_bg = done.clone();
        let clock_bg = clock.clone();
        let stepper = tokio::task::spawn_blocking(move || {
            while !done_bg.load(Ordering::Acquire) {
                clock_bg.run_steps(period_ns, 1);
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        let _ = sup.trigger_e_stop().await;
        let _ = handle.clear_fault_async().await.unwrap().unwrap();
        done.store(true, Ordering::Release);
        stepper.await.unwrap();
        clock.run_steps(period_ns, 2);

        let held_snap = sup.hub().latest().unwrap();
        assert!(held_snap.rt.holding_after_clear);
        let p_snap = MotionPlanSnapshot::from_published(&held_snap, sup.hub().motion_config().unwrap()).unwrap();
        let park = req::prepare_motion(&p_snap, &MotionRequest::park()).unwrap();
        let r_park = handle.broker().lock().unwrap().submit(
            ClientId(1), 2, auth_from_hub(&sup, &handle), clock.now() + Duration::from_secs(10),
            Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park.slot)),
        ).unwrap();
        clock.run_steps(period_ns, 2);
        let ack_rs = drain_replies(&handle);
        let action_id = ack_rs.iter().find_map(|r| match r {
            Reply::Ack { request, action: Some(a), .. } if *request == r_park => Some(a.0),
            _ => None,
        }).unwrap_or_else(|| panic!("expected InstallPark ACK with action_id; replies={ack_rs:?}"));
        // Advance until ParkRearm confirms and continuation enters ReadyForPlan.
        let mut ready_snap = None;
        for _ in 0..40 {
            clock.run_steps(period_ns, 1);
            let s = sup.hub().latest().unwrap();
            if !s.rt.holding_after_clear && s.obs.safety_state == SafetyState::Armed {
                ready_snap = Some(s);
                break;
            }
        }
        let ready_snap = ready_snap.expect("ParkRearm confirmed on StateHub");
        // Prepare continuation plan BEFORE Stop arrives (simulating continuation mid-planning).
        let cont_plan = req::prepare_motion(
            &MotionPlanSnapshot::from_published(&ready_snap, sup.hub().motion_config().unwrap()).unwrap(),
            &MotionRequest::park(),
        ).unwrap();
        // Client Stop arrives while continuation is being planned.
        let sg = handle.stop_requests().request();
        clock.run_steps(period_ns, 3);
        assert_eq!(handle.stop_requests().applied(), sg);
        let rs = drain_replies(&handle);
        assert!(rs.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Stopped, .. } if *request == r_park)), "{rs:?}");
        let snap_after_stop = sup.hub().latest().unwrap();
        let q_after_stop = snap_after_stop.obs.commanded_positions.clone();

        // Late continuation install must be refused and produce zero motion.
        let r_late = handle.broker().lock().unwrap().submit(
            ClientId(1), 3, auth_from_hub(&sup, &handle), clock.now() + Duration::from_secs(10),
            Submit::ContinuePark(&|s: &mut PlanSlot| req::stage_park_continuation(s, action_id, &cont_plan.slot)),
        ).unwrap();
        clock.run_steps(period_ns, 10);
        let rs = drain_replies(&handle);
        assert!(rs.iter().any(|r| matches!(r, Reply::Refused { request, .. } if *request == r_late)), "{rs:?}");
        let end_snap = sup.hub().latest().unwrap();
        assert_eq!(end_snap.obs.commanded_positions, q_after_stop, "no motion after Stop ACK on executor");
        drop(handle);
    }

    // 7d. Direct unit check: install_park_continuation rejects any advanced stop_gen with Stopped.
    {
        let mut inner = SupervisorInner::new(parkable_2dof_config());
        let stamp = inner.current_admission_stamp();
        SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
        inner.commanded_positions = vec![0.8, 0.4];
        inner.measured_positions = vec![0.8, 0.4];
        inner.target_positions = vec![0.8, 0.4];
        inner.execute_stop(armd::safety::StopPolicy::EStopLatch);
        inner.clear_fault().unwrap();
        assert!(inner.holding_after_clear());
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let stamp = inner.current_admission_stamp();
        let ep = inner.safety.epoch_id();
        let action_id = match req::install_park(&mut inner, stamp, &park.slot, None, Some(ep), false).unwrap() {
            MotionInstalled::DeferredContinuation { action_id, .. } => action_id,
            other => panic!("expected DeferredContinuation, got {other:?}"),
        };
        assert_eq!(inner.park_continuation().unwrap().phase, ParkContinuationPhase::ReadyForPlan);
        let cont_plan = req::prepare_motion(&MotionPlanSnapshot::from_supervisor(&inner), &MotionRequest::park()).unwrap();
        inner.advance_stop_generation();
        assert_eq!(
            req::install_park_continuation(&mut inner, action_id, &cont_plan.slot, None, Some(ep), false),
            Err(MotionInstallError::Stopped),
            "install_park_continuation must verify original request stop_gen"
        );
    }
}
