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
    reported_vel_j0_bits: Arc<std::sync::atomic::AtomicU32>,
    override_q0_bits: Arc<std::sync::atomic::AtomicU32>,
}

impl DriverKnobs {
    fn new() -> Self {
        Self {
            confirm_disable: Arc::new(AtomicBool::new(true)),
            fail_next_enable: Arc::new(AtomicBool::new(false)),
            estop_inhibit_disable: Arc::new(AtomicBool::new(false)),
            disable_started: Arc::new(AtomicBool::new(false)),
            nudge_q0_on_enable: Arc::new(Mutex::new(None)),
            reported_vel_j0_bits: Arc::new(std::sync::atomic::AtomicU32::new(0.0f32.to_bits())),
            override_q0_bits: Arc::new(std::sync::atomic::AtomicU32::new(f32::NAN.to_bits())),
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
    fn fresh_joint_positions_into(&self, indices: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
        if out.len() < indices.len() { return false; }
        let n = self.now();
        let ov_q0 = f32::from_bits(self.knobs.override_q0_bits.load(Ordering::Relaxed));
        for (slot, &i) in out.iter_mut().zip(indices.iter()) {
            let fresh = self.rx_at.get(i).copied().flatten().is_some_and(|t| n.saturating_duration_since(t) <= max_age);
            if !fresh { return false; }
            let pos = if i == 0 && ov_q0.is_finite() { ov_q0 } else { *self.q.get(i).unwrap_or(&f32::NAN) };
            if !pos.is_finite() { return false; }
            *slot = pos;
        }
        true
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
        let vel0 = f32::from_bits(self.knobs.reported_vel_j0_bits.load(Ordering::Relaxed));
        let ov_q0 = f32::from_bits(self.knobs.override_q0_bits.load(Ordering::Relaxed));
        out.clear();
        for i in 0..self.dof {
            let position = if i == 0 && ov_q0.is_finite() { ov_q0 } else { self.q[i] };
            let velocity = if i == 0 { vel0 } else { 0.0 };
            out.push(JointState { position, velocity, torque: 0.0, temperature: 30.0 });
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
        if !self.knobs.confirm_disable.load(Ordering::Acquire) {
            return Err(ArmDriverError::TorqueOffWouldDrop);
        }
        self.enabled = false;
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
    sup.inner().write().await.register_live_connection(1).unwrap();
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
    drop(inner);

    // 1b. M2 extension: real WS dispatch path (/api/v1/motion/jog + /api/v1/motion/home) with a
    //     concurrent 50 Hz tick loop on the legacy owner -> write-lock holds stay <= 2 ms.
    {
        let cfg = dm_config_with_planning_load();
        let mut q0 = cfg.home_positions.clone();
        q0[0] += 0.20;
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let (mut ws, gen, _, sup, _) = common::torqued_rig(sup, &q0).await;
        let before_seq = sup.hub().published_seq();
        let ep = sup.get_observation().await.epoch_id;
        let mut jog_t = q0.clone();
        jog_t[0] -= 0.08;
        let jog_ack = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/motion/jog",
            json!({"target_positions": jog_t, "duration_secs": 0.4}),
        ).await;
        assert_eq!(jog_ack["type"], "command_ack", "{jog_ack}");
        tokio::time::sleep(Duration::from_millis(80)).await;
        let ep = sup.get_observation().await.epoch_id;
        let home_ack = common::rpc(&mut ws, gen, ep, "/api/v1/motion/home", json!({"duration_secs": 0.4})).await;
        assert_eq!(home_ack["type"], "command_ack", "{home_ack}");
        tokio::time::sleep(Duration::from_millis(80)).await;
        let stats = sup.tick_stats();
        assert!(sup.hub().published_seq() >= before_seq + 5, "concurrent 50 Hz ticks must advance during WS motion");
        assert!(
            stats.writer_hold_us.max <= 2_000,
            "WS jog/home writer_hold_us.max = {} us (> 2 ms budget)",
            stats.writer_hold_us.max
        );
        assert!(
            stats.lock_us.max <= 2_000,
            "concurrent tick lock_us.max = {} us (> 2 ms budget)",
            stats.lock_us.max
        );
        assert!(sup.inner().read().await.prepared_quintics_used >= 2);
    }
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
    assert!(MotionPlanSnapshot::refresh_for_drift_retry(&inner, &snap).is_none());
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

    // Executor WS entry routes `/api/v1/motion/jog` through executor admission (409 LEASE_LOST
    // without a lease), while unmigrated `/api/v1/mode/gravity_comp` stays at 503 EXECUTOR_ROUTING_PENDING.
    let router = create_router(sup.clone(), OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let r = common::rpc(&mut ws, 0, final_snap.obs.epoch_id, "/api/v1/motion/jog", json!({"target_positions": [0.0, 0.0]})).await;
    assert_eq!((&r["type"], &r["status"], &r["code"]), (&json!("command_reject"), &json!(409), &json!("LEASE_LOST")), "{r}");
    let r_gc = common::rpc(&mut ws, 0, final_snap.obs.epoch_id, "/api/v1/mode/gravity_comp", json!({"enable": true})).await;
    assert_eq!((&r_gc["type"], &r_gc["status"], &r_gc["code"]), (&json!("command_reject"), &json!(503), &json!("EXECUTOR_ROUTING_PENDING")), "{r_gc}");
    drop(handle);
}

// ============================================================================
// 4. Park Disable Unconfirmed, Stop Before/After Disable Write, E-Stop Inhibit, Workspace
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn park_disable_confirm_failure_stop_boundary_and_estop_inhibit() {
    let period = Duration::from_millis(5);
    let period_ns = period.as_nanos() as u64;

    // (a) Rest-pose gate refusal during park disable -> TORQUE_OFF_WOULD_DROP, torque stays ON, 0 alloc.
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
        assert_eq!(failed_code, Some("TORQUE_OFF_WOULD_DROP"), "replies={all_rs:?} obs={:?}", sup.hub().latest().map(|s| (s.obs.safety_state.clone(), s.obs.is_torqued, s.obs.action.clone())));
        let snap_end = sup.hub().latest().unwrap();
        assert!(snap_end.obs.is_torqued, "torque must remain reported ON when park disable is refused by rest-pose gate");
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
        // Park trajectory (~3.2 s) + Opening 2.0 rad at 1.0 rad/s (~2.0 s) +
        // Closing 4.85 rad at 1.0 rad/s (~4.85 s) + UnloadSettle (0.8 s) ~= 10.9 s (~2180 ticks).
        for _ in 0..2600 {
            clock.run_steps(period_ns, 1);
            if knobs.disable_started.load(Ordering::Acquire) { break; }
        }
        assert!(
            knobs.disable_started.load(Ordering::Acquire),
            "7-DOF park with gripper release (Opening -> Closing -> UnloadSettle) did not reach disable; replies={:?}, obs={:?}",
            drain_replies(&handle),
            sup.hub().latest().map(|s| (s.obs.safety_state.clone(), s.obs.is_torqued, s.obs.action.clone(), s.obs.positions.clone()))
        );
        let pre_estop_j7 = sup.hub().latest().unwrap().obs.positions[g_idx];
        assert!(
            (pre_estop_j7 - gp.closed_rad).abs() < 0.05,
            "Opening -> Closing must close J7 before disable, got j7={pre_estop_j7} (closed={})",
            gp.closed_rad
        );
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

// ============================================================================
// 8. Round 2 M1: Prepared plan authority (boot, epoch, lease, stop_gen, deadline)
// ============================================================================

#[test]
fn m1_prepared_plan_authority_and_deadline_enforced_at_install() {
    // 8a. ArmSupervisor::home queued behind a saturated Tokio blocking pool past its bounded
    //     deadline is refused with DEADLINE_EXPIRED and produces zero motion.
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .max_blocking_threads(2)
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(async {
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()));
        sup.set_lease_enabled(false).await;
        sup.set_motion_plan_deadline(Duration::from_millis(45)).await;
        {
            let mut inner = sup.inner().write().await;
            let stamp = inner.current_admission_stamp();
            SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
            inner.commanded_positions = vec![0.8, 0.4];
            inner.measured_positions = vec![0.8, 0.4];
            inner.target_positions = vec![0.8, 0.4];
        }
        let ep = sup.get_observation().await.epoch_id;
        // Saturate the 2-thread blocking pool past the 45ms motion deadline.
        let entered = Arc::new(std::sync::Barrier::new(3));
        let mut blockers = Vec::new();
        for _ in 0..2 {
            let b = entered.clone();
            blockers.push(tokio::task::spawn_blocking(move || {
                b.wait();
                std::thread::sleep(Duration::from_millis(160));
            }));
        }
        entered.wait();
        let stamp = sup.capture_admission_stamp();
        let err = sup.home(stamp, None, Some(ep)).await.unwrap_err();
        for h in blockers { h.await.unwrap(); }
        match err {
            armd::ArmError::Rejected { code, .. } => assert_eq!(code, "DEADLINE_EXPIRED"),
            other => panic!("expected DEADLINE_EXPIRED behind saturated blocking pool, got {other:?}"),
        }
        assert!(sup.inner().read().await.active_trajectory.is_none(), "expired home must not start motion");
        assert_eq!(sup.get_observation().await.commanded_positions, vec![0.8, 0.4]);

        // 8b. Legacy WS /api/v1/motion/home delayed past bounded deadline is refused DEADLINE_EXPIRED.
        {
            struct PauseGuard(String);
            impl Drop for PauseGuard {
                fn drop(&mut self) { armd::api::ws::set_test_ws_blocking_plan_paused(&self.0, false); }
            }
            let (drv, _) = MotionTestDriver::new(vec![0.8, 0.4], Duration::ZERO, Duration::ZERO, None);
            let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
            sup.set_motion_plan_deadline(Duration::from_millis(50)).await;
            let (mut ws, gen, ep, sup, _addr) = common::torqued_rig(sup, &[0.8, 0.4]).await;
            let conn_id = sup.inner().read().await.active_lease.as_ref().unwrap().client_id.clone();
            armd::api::ws::set_test_ws_blocking_plan_paused(&conn_id, true);
            let _guard = PauseGuard(conn_id.clone());
            let unpause_conn = conn_id.clone();
            tokio::spawn(async move {
                let wait_until = tokio::time::Instant::now() + Duration::from_secs(2);
                while armd::api::ws::test_ws_blocking_plan_blocked_count(&unpause_conn) < 1
                    && tokio::time::Instant::now() < wait_until
                {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                tokio::time::sleep(Duration::from_millis(90)).await;
                armd::api::ws::set_test_ws_blocking_plan_paused(&unpause_conn, false);
            });
            let resp = common::rpc(&mut ws, gen, ep, "/api/v1/motion/home", json!({})).await;
            assert_eq!((&resp["type"], &resp["code"]), (&json!("command_reject"), &json!("DEADLINE_EXPIRED")), "{resp}");
            assert!(sup.inner().read().await.active_trajectory.is_none());
        }

        // 8c. Executor broker: a pre-Stop prepared plan staged into a post-Stop descriptor is refused
        //     with STOPPED inside the single install path (no motion starts).
        {
            let period = Duration::from_millis(5);
            let period_ns = period.as_nanos() as u64;
            let clock = ScriptedClock::new();
            let (drv, _) = MotionTestDriver::new(vec![0.8, 0.4], Duration::from_millis(4), Duration::from_millis(4), Some(clock.clone()));
            let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
            let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
            clock.run_steps(period_ns, 2);
            let _ = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);

            let pre_stop_snap = MotionPlanSnapshot::from_published(
                &sup.hub().latest().unwrap(),
                sup.hub().motion_config().unwrap(),
            ).unwrap();
            let pre_stop_home = req::prepare_motion(&pre_stop_snap, &MotionRequest::home(Some(0.5))).unwrap();
            let pre_stop_jog = req::prepare_motion(&pre_stop_snap, &MotionRequest::jog(vec![0.5, 0.2], Some(0.5), None)).unwrap();

            // Client Stop advances stop_generation on the executor.
            let sg = handle.stop_requests().request();
            clock.run_steps(period_ns, 2);
            assert_eq!(handle.stop_requests().applied(), sg);
            let _ = drain_replies(&handle);

            // Stage pre-Stop prepared plans under the NEW post-Stop broker authority!
            let post_stop_auth = auth_from_hub(&sup, &handle);
            assert_eq!(post_stop_auth.stop_generation, sg);
            let r_home = handle.broker().lock().unwrap().submit(
                ClientId(1), 2, post_stop_auth, clock.now() + Duration::from_secs(10),
                Submit::InstallHome(&|s: &mut PlanSlot| req::stage_home(s, &pre_stop_home.slot)),
            ).unwrap();
            clock.run_steps(period_ns, 2);
            let rs = drain_replies(&handle);
            assert!(rs.iter().any(|r| matches!(r, Reply::Refused { request, code: "STOPPED", .. } if *request == r_home)), "{rs:?}");

            let r_jog = handle.broker().lock().unwrap().submit(
                ClientId(1), 3, post_stop_auth, clock.now() + Duration::from_secs(10),
                Submit::Install(&|s: &mut PlanSlot| req::stage_jog(s, &pre_stop_jog.slot)),
            ).unwrap();
            clock.run_steps(period_ns, 2);
            let rs = drain_replies(&handle);
            assert!(rs.iter().any(|r| matches!(r, Reply::Refused { request, code: "STOPPED", .. } if *request == r_jog)), "{rs:?}");
            assert_eq!(sup.hub().latest().unwrap().obs.commanded_positions, vec![0.8, 0.4]);
            assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
            drop(handle);
        }

        // 8d. Single install function verifies ALL 5 PlanAuthority fields (boot, deadline,
        //     stop_generation, epoch, lease_generation) and refresh_for_drift_retry preserves them.
        {
            let mut inner = SupervisorInner::new(parkable_2dof_config());
            let stamp = inner.current_admission_stamp();
            SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
            inner.commanded_positions = vec![0.8, 0.4];
            inner.measured_positions = vec![0.8, 0.4];
            inner.target_positions = vec![0.8, 0.4];
            let lease_gen = inner.acquire_lease("conn-a").unwrap();
            let snap = MotionPlanSnapshot::from_supervisor(&inner);
            let base = req::prepare_motion(&snap, &MotionRequest::home(Some(0.4))).unwrap();
            let ep = inner.safety.epoch_id();

            // Wrong boot + BootId(0) fail-closed.
            let mut wrong_boot = base.slot;
            wrong_boot.authority.boot = armd::rt_core::broker::BootId(inner.boot_id().0.wrapping_add(1).max(1));
            let s = inner.current_admission_stamp();
            assert_eq!(req::install_home(&mut inner, s, &wrong_boot, None, Some(ep), false), Err(MotionInstallError::WrongTarget));
            let mut zero_boot = base.slot;
            zero_boot.authority.boot = armd::rt_core::broker::BootId(0);
            let s = inner.current_admission_stamp();
            assert_eq!(req::install_home(&mut inner, s, &zero_boot, None, Some(ep), false), Err(MotionInstallError::WrongTarget));

            // Expired deadline.
            let mut expired = base.slot;
            expired.authority.deadline = std::time::Instant::now() - Duration::from_millis(1);
            let s = inner.current_admission_stamp();
            assert_eq!(req::install_home(&mut inner, s, &expired, None, Some(ep), false), Err(MotionInstallError::DeadlineExpired));

            // Stale stop_generation.
            let mut stale_stop = base.slot;
            stale_stop.authority.stop_generation = inner.current_stop_gen() + 1;
            let s = inner.current_admission_stamp();
            assert_eq!(req::install_home(&mut inner, s, &stale_stop, None, Some(ep), false), Err(MotionInstallError::Stopped));

            // Stale epoch.
            let mut stale_epoch = base.slot;
            stale_epoch.authority.epoch = ep + 1;
            let s = inner.current_admission_stamp();
            assert_eq!(req::install_home(&mut inner, s, &stale_epoch, None, Some(ep), false), Err(MotionInstallError::EpochMismatch));

            // Stale lease_generation.
            let mut stale_lease = base.slot;
            stale_lease.authority.lease_generation = lease_gen + 1;
            let s = inner.current_admission_stamp();
            assert_eq!(req::install_home(&mut inner, s, &stale_lease, None, Some(ep), false), Err(MotionInstallError::LeaseLost));

            // refresh_for_drift_retry preserves original authority & deadline without extending.
            let orig_auth = snap.authority;
            inner.commanded_positions[0] += 0.05;
            let refreshed = MotionPlanSnapshot::refresh_for_drift_retry(&inner, &snap).unwrap();
            assert_eq!(refreshed.authority, orig_auth, "drift retry must preserve original request authority and deadline");
        }
    });
}

// ============================================================================
// 9. Round 2 M2: Off-thread lease-expiry watchdog park (0 alloc), bare JointQuintic
//    refusal on RT, legacy watchdog park off SupervisorLock, and MIT park off WS lock
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn m2_off_thread_watchdog_park_bare_quintic_refusal_mit_handback_and_zero_alloc() {
    let period = Duration::from_millis(5);
    let period_ns = period.as_nanos() as u64;

    // 9a. Bare broker `Primitive::JointQuintic` without `PreparedMotionSlot` is refused on
    //     armd-control without planning and with AllocCounts::ZERO.
    {
        let clock = ScriptedClock::new();
        let (drv, _) = MotionTestDriver::new(vec![0.4, 0.2], Duration::from_millis(4), Duration::from_millis(4), Some(clock.clone()));
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let handle = start_scripted_executor(&sup, period, clock.clone(), true).await;
        clock.run_steps(period_ns, 2);
        let auth = submit_enable_and_wait(&sup, &handle, &clock, period_ns, 1);
        let prim = armd::rt_core::ops::Primitive::joint_quintic(&[0.4, 0.2], &[0.1, 0.1], 0.5).unwrap();
        let req_id = handle.broker().lock().unwrap().submit(
            ClientId(1), 2, auth, clock.now() + Duration::from_secs(10),
            Submit::Install(&move |s: &mut PlanSlot| s.set(prim)),
        ).unwrap();
        clock.run_steps(period_ns, 3);
        let rs = drain_replies(&handle);
        assert!(rs.iter().any(|r| matches!(r, Reply::Refused { request, code: "INVALID_COMMAND", .. } if *request == req_id)), "{rs:?}");
        assert_eq!(sup.hub().latest().unwrap().obs.commanded_positions, vec![0.4, 0.2]);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
        drop(handle);
    }

    // 9b. Executor lease-expiry NormalUnload: holds at measured position while off-RT armd-persist
    //     prepares the park plan, installs via shared `install_watchdog_park`, ignores client Stop,
    //     parks to rest, disables torque, and runs with strictly AllocCounts::ZERO on armd-control!
    {
        let mut cfg = parkable_2dof_config();
        cfg.lease_timeout_ms = 60;
        let clock = ScriptedClock::new();
        let (drv, knobs) = MotionTestDriver::new(vec![0.6, 0.3], Duration::from_millis(4), Duration::from_millis(4), Some(clock.clone()));
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = true;
            let _ = inner.acquire_lease("client-wd").unwrap();
            let s = inner.current_admission_stamp();
            SupervisorInner::enable_torque(inner.check_admission(s).unwrap()).unwrap();
            inner.commanded_positions = vec![0.6, 0.3];
            inner.measured_positions = vec![0.6, 0.3];
            inner.target_positions = vec![0.6, 0.3];
            // Pre-expire heartbeat so release #1 triggers lease-expiry watchdog immediately.
            inner.last_heartbeat = std::time::Instant::now() - Duration::from_millis(200);
        }
        let mut exec_cfg = ExecutorConfig::for_tests(period);
        exec_cfg.alloc_timing = true;
        let mut hooks = PersistTestHooks::default();
        hooks.clock = Some(clock.clone());
        let handle = sup.start_executor_with_hooks(exec_cfg, hooks).await.unwrap();

        // Release #1: watchdog triggers -> holds at measured [0.6, 0.3] & publishes watchdog_park_pending_gen != 0.
        clock.run_steps(period_ns, 1);
        let snap1 = sup.hub().latest().unwrap();
        assert_ne!(snap1.rt.watchdog_park_pending_gen, 0, "must defer watchdog park planning off armd-control");
        assert_eq!(snap1.obs.commanded_positions, vec![0.6, 0.3], "must hold at measured position while preparing");
        assert!(snap1.obs.is_torqued);

        // Client Stop during the off-thread prepare window must NOT cancel the watchdog park.
        let sg = handle.stop_requests().request();
        clock.run_steps(period_ns, 1);
        assert_eq!(handle.stop_requests().applied(), sg);

        // Let armd-persist prepare off-thread and push the Copy WatchdogParkPlan, then run to Idle!
        let mut reached_idle = false;
        for _ in 0..800 {
            clock.run_steps(period_ns, 2);
            let s = sup.hub().latest().unwrap();
            if s.obs.safety_state == SafetyState::Idle && !s.obs.is_torqued {
                reached_idle = true;
                break;
            }
            if s.rt.watchdog_park_pending_gen != 0 {
                std::thread::sleep(Duration::from_millis(2));
            }
        }
        assert!(reached_idle, "executor off-thread watchdog park did not reach Idle: {:?}", sup.hub().latest().map(|s| s.obs.safety_state.clone()));
        assert!(knobs.disable_started.load(Ordering::Acquire));
        let end_snap = sup.hub().latest().unwrap();
        assert!(end_snap.obs.positions.iter().all(|p| p.abs() < 1e-2), "ended at rest: {:?}", end_snap.obs.positions);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "lease-expiry watchdog park must perform zero alloc/free on armd-control");
        drop(handle);
    }

    // 9c. Legacy watchdog park off SupervisorLock: heavy workspace + resting pose, expires under
    //     control_tick -> parks to rest and disables torque with write-lock hold <= 2 ms.
    {
        let mut cfg = dm_config_with_planning_load();
        cfg.lease_timeout_ms = 40;
        let mut q0 = cfg.resting_positions().to_vec();
        q0[0] += 0.25;
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        {
            let mut inner = sup.inner().write().await;
            let _ = inner.acquire_lease("client-legacy-wd").unwrap();
            inner.commanded_positions = q0.clone();
            inner.measured_positions = q0.clone();
            inner.target_positions = q0.clone();
            inner.last_heartbeat = std::time::Instant::now() - Duration::from_millis(120);
        }
        let _ = tokio::task::spawn_blocking(|| {}).await;
        let _ = sup.inner().take_log();
        let mut reached_idle = false;
        for _ in 0..400 {
            sup.control_tick(0.02).await;
            let obs = sup.get_observation().await;
            if obs.safety_state == SafetyState::Idle && !obs.is_torqued {
                reached_idle = true;
                break;
            }
            if sup.inner().read().await.active_trajectory.is_none() {
                tokio::time::sleep(Duration::from_millis(2)).await;
            }
        }
        assert!(reached_idle, "legacy off-lock watchdog park did not complete");
        let stats = sup.tick_stats();
        assert!(stats.lock_us.max <= 2_000, "legacy watchdog tick lock_us.max = {} us (> 2 ms)", stats.lock_us.max);
        assert!(stats.writer_hold_us.max <= 2_000, "legacy watchdog install writer_hold_us.max = {} us (> 2 ms)", stats.writer_hold_us.max);
    }

    // 9d. Legacy MIT stationary park: install_park defers to WaitingMitHandback WITHOUT calling
    //     blocking `set_joint_modes_confirmed` under the WS write lock; tick hands back off WS lock.
    {
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
        assert!(inner.commanded_velocities.iter().all(|&v| v == 0.0), "stationary MIT arm");

        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let park_plan = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        assert!(park_plan.slot.for_mit_deferral);
        let stamp = inner.current_admission_stamp();
        let ep = inner.safety.epoch_id();
        let installed = req::install_park(&mut inner, stamp, &park_plan.slot, None, Some(ep), false).unwrap();
        let park_id = match installed {
            MotionInstalled::DeferredContinuation { action_id, .. } => action_id,
            other => panic!("stationary MIT park must defer handback off WS write lock, got {other:?}"),
        };
        assert_eq!(inner.park_continuation().unwrap().phase, ParkContinuationPhase::WaitingMitHandback);
        assert!(inner.mit_active(), "MIT handback must NOT have executed synchronously under WS write lock");

        inner.tick(0.01);
        assert!(!inner.mit_active(), "control tick completes MIT handback off WS write lock");
        assert_eq!(inner.park_continuation().unwrap().phase, ParkContinuationPhase::ReadyForPlan);
        let cont_plan = req::prepare_motion(&MotionPlanSnapshot::from_supervisor(&inner), &MotionRequest::park()).unwrap();
        req::install_park_continuation(&mut inner, park_id, &cont_plan.slot, None, Some(ep), false).unwrap();
        assert!(inner.park_continuation().is_none() && inner.pending_normal_unload);
    }

    // 9e. Round 3 Item 3: RT watchdog park settle-timeout branches (`at_pose` grace extension +
    //     grace expiry `WatchdogTimeout`, AND `!at_pose` immediate `WatchdogTimeout`) must execute
    //     on `armd-control` with strictly `AllocCounts::ZERO` (no diagnostic `String` allocations).
    for test_at_pose_grace in [true, false] {
        let step_period = Duration::from_millis(20);
        let step_period_ns = step_period.as_nanos() as u64;
        let mut cfg = parkable_2dof_config();
        cfg.lease_timeout_ms = 60;
        let clock = ScriptedClock::new();
        let (drv, knobs) = MotionTestDriver::new(vec![0.12, 0.05], Duration::from_millis(4), Duration::from_millis(4), Some(clock.clone()));
        if test_at_pose_grace {
            // Reach resting pose [0.0, 0.0] (`at_pose == true`), but report velocity > max_velocity
            // (0.015 rad/s) so the stationary settle window never completes -> exercises grace grant
            // AND post-grace WatchdogTimeout latch.
            knobs.reported_vel_j0_bits.store(0.05f32.to_bits(), Ordering::Relaxed);
        } else {
            // Keep measured joint 0 outside resting_pose tolerance (0.08 rad) yet well below
            // following_error bound so `at_pose == false` -> immediate WatchdogTimeout without grace.
            knobs.override_q0_bits.store(0.12f32.to_bits(), Ordering::Relaxed);
        }
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        {
            let mut inner = sup.inner().write().await;
            inner.lease_enabled = true;
            let _ = inner.acquire_lease("client-wd-settle-timeout").unwrap();
            let s = inner.current_admission_stamp();
            SupervisorInner::enable_torque(inner.check_admission(s).unwrap()).unwrap();
            inner.commanded_positions = vec![0.12, 0.05];
            inner.measured_positions = vec![0.12, 0.05];
            inner.target_positions = vec![0.12, 0.05];
            inner.last_heartbeat = std::time::Instant::now() - Duration::from_millis(200);
        }
        let mut exec_cfg = ExecutorConfig::for_tests(step_period);
        exec_cfg.alloc_timing = true;
        let mut hooks = PersistTestHooks::default();
        hooks.clock = Some(clock.clone());
        let handle = sup.start_executor_with_hooks(exec_cfg, hooks).await.unwrap();

        // Step 1 triggers lease expiry and defers watchdog park planning off-thread.
        clock.run_steps(step_period_ns, 1);
        assert_ne!(sup.hub().latest().unwrap().rt.watchdog_park_pending_gen, 0);

        // Wait for off-thread watchdog park to be planned and installed by armd-persist.
        let mut installed = false;
        for _ in 0..80 {
            clock.run_steps(step_period_ns, 1);
            let s = sup.hub().latest().unwrap();
            if s.rt.watchdog_park_pending_gen == 0 && s.obs.safety_state.is_operational() {
                installed = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(installed, "off-thread watchdog park must install before settle-timeout check");

        // Advance past WATCHDOG_PARK_TIMEOUT (10 s) + WATCHDOG_PARK_SETTLE_GRACE (5 s) = ~16 s.
        let mut saw_grace_still_operational = false;
        let mut latched_watchdog_fault = false;
        let mut fault_step = 0usize;
        for step in 1..=950 {
            clock.run_steps(step_period_ns, 1);
            let s = sup.hub().latest().unwrap();
            if test_at_pose_grace && step == 560 {
                // ~11.2 s: initial 10 s budget expired, 5 s grace must keep the arm operational.
                assert!(
                    s.obs.safety_state.is_operational(),
                    "grace window must keep arm operational past initial 10 s budget; got {:?}",
                    s.obs.safety_state
                );
                saw_grace_still_operational = true;
            }
            if matches!(s.obs.safety_state, SafetyState::Fault(armd::safety::FaultReason::WatchdogTimeout)) {
                assert!(s.obs.is_torqued, "torque must remain held on WatchdogTimeout");
                latched_watchdog_fault = true;
                fault_step = step;
                break;
            }
        }
        if test_at_pose_grace {
            assert!(saw_grace_still_operational, "at-pose settle timeout must grant grace window first");
            assert!(fault_step > 560, "at-pose fault must occur only after 5 s grace expires (step={fault_step})");
        } else {
            assert!(fault_step < 560, "off-pose timeout must latch immediately without 5 s grace (step={fault_step})");
        }
        assert!(latched_watchdog_fault, "settle timeout (test_at_pose_grace={test_at_pose_grace}) must latch WatchdogTimeout");
        assert!(!knobs.disable_started.load(Ordering::Acquire), "must not cut torque when settle timed out");
        assert_eq!(
            handle.stats().total_allocs(),
            AllocCounts::ZERO,
            "RT settle-timeout branch (test_at_pose_grace={test_at_pose_grace}) must perform zero heap alloc/free on armd-control"
        );
        drop(handle);
    }
}

// ============================================================================
// 10. Round 2 M3: Park's first disable frame rechecks pending Stop / stop_gen
//     at the driver's transmit gate (`DamiaoArmDriver::send`)
// ============================================================================

#[test]
fn m3_client_park_first_disable_frame_rechecks_stop_at_driver_tx_gate() {
    use armd::driver::FeetechArmDriver;
    use armd::protocol::feetech;
    use armd::rt_core::urgent::{EstopGate, StopRequests};
    use armd::transport::{CanFrame, CanTransport, DriverClock, Transport, TransportError};

    struct RaceStopCan {
        inner: FakeBus,
        nonblocking_rt: bool,
        fire_stop_on_receive: Arc<AtomicBool>,
        fire_stop_on_first_disable_tx: Arc<AtomicBool>,
        would_block_and_stop_on_first_disable_tx: Arc<AtomicBool>,
        stops: StopRequests,
        disable_tx_count: Arc<std::sync::atomic::AtomicUsize>,
    }

    impl CanTransport for RaceStopCan {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> { self.inner.clock() }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            let is_disable = frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
            if is_disable && self.would_block_and_stop_on_first_disable_tx.swap(false, Ordering::SeqCst) {
                let _ = self.stops.request();
                return Err(TransportError::WouldBlock);
            }
            if is_disable {
                self.disable_tx_count.fetch_add(1, Ordering::SeqCst);
            }
            let r = self.inner.transmit(frame);
            if is_disable && self.fire_stop_on_first_disable_tx.swap(false, Ordering::SeqCst) {
                let _ = self.stops.request();
            }
            r
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.receive_timestamped().map(|(f, _)| f)
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
            if self.fire_stop_on_receive.swap(false, Ordering::SeqCst) {
                let _ = self.stops.request();
            }
            self.inner.receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
        fn supports_bounded_flush(&self) -> bool { true }
        fn supports_nonblocking_rt(&self) -> bool { self.nonblocking_rt }
    }

    let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm().damiao_ranges.unwrap()[..2].iter().map(|r| r.limits()).collect();

    // 10a. Direct driver-boundary checks on `DamiaoArmDriver::begin_client_park_disable`.
    {
        let clock = armd::transport::MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        let stops = StopRequests::default();
        let req_estop = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let app_estop = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let fire_rx = Arc::new(AtomicBool::new(false));
        let fire_tx = Arc::new(AtomicBool::new(false));
        let wb_stop = Arc::new(AtomicBool::new(false));
        let disable_tx = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let can = RaceStopCan {
            inner: bus.clone(),
            nonblocking_rt: true,
            fire_stop_on_receive: fire_rx.clone(),
            fire_stop_on_first_disable_tx: fire_tx.clone(),
            would_block_and_stop_on_first_disable_tx: wb_stop.clone(),
            stops: stops.clone(),
            disable_tx_count: disable_tx.clone(),
        };
        let mut drv = DamiaoArmDriver::new(can, vec![1, 2]).with_joint_limits(limits.clone()).unwrap();
        drv.set_estop_gate(EstopGate::new(req_estop, app_estop).with_stops(stops.clone()));
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        arm_joints[1] = 1;
        drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(false)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 2,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        bus.with(|s| s.push_all_feedback());
        drv.read_joint_states().unwrap();
        drv.set_torque(true).unwrap();
        // Accumulate K=3 causal rest samples so the rest-pose gate is confirmed before testing
        // client-park Stop linearization at the driver TX choke point.
        for _ in 0..3 {
            clock.advance(Duration::from_millis(12));
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        assert_eq!(drv.enable_state(), EnableState::Enabled);
        assert_eq!(disable_tx.load(Ordering::SeqCst), 0);

        // (1) Client Stop fires inside `begin_client_park_disable`'s RX drain RIGHT BEFORE motor 0 TX:
        //     Step 3/Step 5 inside `send(TxKind::ClientParkDisable)` inhibits motor 0 -> 0 frames sent,
        //     driver stays Enabled with no torque transition mutation.
        fire_rx.store(true, Ordering::SeqCst);
        let err = drv.begin_client_park_disable(None, Some(0)).unwrap_err();
        assert!(armd::driver::is_estop_inhibited(&err), "expected EstopInhibited, got {err:?}");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 0, "zero disable frames may transmit when Stop races motor 0");
        assert_eq!(drv.enable_state(), EnableState::Enabled);
        assert!(!drv.torque_transition_pending());

        // (2) Stop generation already applied (`pending() == false`), but caller passes stale `expected_stop_gen`:
        //     Step 5 `current_stop_stamp() != expected_stamp` still refuses motor 0 -> 0 frames sent.
        let sg = stops.current();
        assert_eq!(sg, 1);
        stops.mark_applied(sg);
        let err2 = drv.begin_client_park_disable(Some(0), Some(0)).unwrap_err();
        assert!(armd::driver::is_estop_inhibited(&err2), "stale stop_gen must be refused: {err2:?}");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 0);
        assert_eq!(drv.enable_state(), EnableState::Enabled);

        // (3) WouldBlock on motor 0 followed by client Stop BEFORE motor 1:
        //     per-motor recheck refuses motor 1 -> 0 disable frames transmitted, driver stays Enabled.
        wb_stop.store(true, Ordering::SeqCst);
        let err3 = drv.begin_client_park_disable(Some(0), Some(sg)).unwrap_err();
        assert!(armd::driver::is_estop_inhibited(&err3), "WouldBlock + Stop before motor 1 must return EstopInhibited, got {err3:?}");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 0, "motor 1 disable must NOT transmit after motor 0 WouldBlock + Stop");
        assert_eq!(drv.enable_state(), EnableState::Enabled);
        assert!(!drv.torque_transition_pending());

        // (4) Motor 0 transmits disable, then client Stop arrives BEFORE motor 1:
        //     per-motor recheck blocks motor 1's disable write (only motor 0 transmitted) and reports error.
        let sg_wb = stops.current();
        assert!(sg_wb > sg && stops.pending());
        stops.mark_applied(sg_wb);
        fire_tx.store(true, Ordering::SeqCst);
        let err4 = drv.begin_client_park_disable(Some(0), Some(sg_wb)).unwrap_err();
        assert!(!armd::driver::is_estop_inhibited(&err4), "partial disable must not be masked as clean pre-tx inhibit: {err4:?}");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 1, "motor 1 must NOT transmit once Stop arrives after motor 0");

        // (5) Watchdog park (`begin_torque_budgeted(false, 0)`) ignores pending client Stop and disables all motors.
        let sg2 = stops.current();
        assert!(sg2 > sg_wb && stops.pending());
        stops.mark_applied(sg2);
        for _ in 0..3 {
            clock.advance(Duration::from_millis(12));
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        drv.set_torque(false).unwrap();
        drv.set_torque(true).unwrap();
        for _ in 0..3 {
            clock.advance(Duration::from_millis(12));
            bus.with(|s| s.push_all_feedback());
            drv.read_joint_states().unwrap();
        }
        let _ = stops.request();
        assert!(stops.pending());
        disable_tx.store(0, Ordering::SeqCst);
        drv.begin_torque_budgeted(false, 0).expect("watchdog disable must ignore client Stop");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 2);
    }

    // 10b. End-to-end `SupervisorInner::begin_park_disable_transition` on Damiao:
    //      motor 0 hits WouldBlock and fires client Stop right before motor 1 -> zero disable frames
    //      transmitted, torque stays ON, !disable_pending, park cancelled STOPPED.
    {
        let clock = armd::transport::MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        let stops = StopRequests::default();
        let fire_rx = Arc::new(AtomicBool::new(false));
        let fire_tx = Arc::new(AtomicBool::new(false));
        let wb_stop = Arc::new(AtomicBool::new(false));
        let disable_tx = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let can = RaceStopCan {
            inner: bus.clone(),
            nonblocking_rt: false,
            fire_stop_on_receive: fire_rx,
            fire_stop_on_first_disable_tx: fire_tx,
            would_block_and_stop_on_first_disable_tx: wb_stop.clone(),
            stops: stops.clone(),
            disable_tx_count: disable_tx.clone(),
        };
        let drv = DamiaoArmDriver::new(can, vec![1, 2]).with_joint_limits(limits.clone()).unwrap();
        let mut inner = SupervisorInner::new(parkable_2dof_config()).with_driver(Box::new(drv));
        inner.attach_stop_requests(stops.clone());
        bus.with(|s| s.push_all_feedback());
        let stamp = inner.current_admission_stamp();
        SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
        inner.tick(0.01);

        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let stamp = inner.current_admission_stamp();
        let ep = inner.safety.epoch_id();
        req::install_park(&mut inner, stamp, &park.slot, None, Some(ep), false).unwrap();

        let mut armed = false;
        for _ in 0..320 {
            clock.advance(Duration::from_millis(10));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.01);
            if !armed && inner.unload_settle.as_ref().is_some_and(|u| u.held_s >= 0.70) {
                armed = true;
                wb_stop.store(true, Ordering::SeqCst);
            }
            if !inner.pending_normal_unload && !inner.torque_transition_pending() {
                break;
            }
        }
        assert!(armed, "Damiao unload_settle must reach >= 0.70s");
        assert!(!wb_stop.load(Ordering::SeqCst), "motor 0 WouldBlock + Stop race must have fired during park disable");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 0, "zero disable frames transmitted on motor 0 WouldBlock + Stop");
        assert!(inner.is_torqued && !inner.disable_pending, "torque must remain held and disable_pending false");
        assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("STOPPED"));
        assert_ne!(inner.safety.fault_code(), Some(FaultCode::DriverWriteFailed));
    }

    // 10b2. Regression (client park Stop fence across partial retry):
    //       motor 0 transmits disable, then client Stop arrives BEFORE motor 1's first write ->
    //       owner must NEVER hand motor 1 to an unstamped background/periodic retry across
    //       subsequent ticks (disable_tx stays == 1, torque_intent stays On, DriverWriteFailed latched).
    for rt_owned in [false, true] {
        let clock = armd::transport::MockClock::new();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        let stops = StopRequests::default();
        let fire_rx = Arc::new(AtomicBool::new(false));
        let fire_tx = Arc::new(AtomicBool::new(false));
        let wb_stop = Arc::new(AtomicBool::new(false));
        let disable_tx = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let can = RaceStopCan {
            inner: bus.clone(),
            nonblocking_rt: rt_owned,
            fire_stop_on_receive: fire_rx,
            fire_stop_on_first_disable_tx: fire_tx.clone(),
            would_block_and_stop_on_first_disable_tx: wb_stop,
            stops: stops.clone(),
            disable_tx_count: disable_tx.clone(),
        };
        let drv = DamiaoArmDriver::new(can, vec![1, 2]).with_joint_limits(limits.clone()).unwrap();
        let mut inner = SupervisorInner::new(parkable_2dof_config()).with_driver(Box::new(drv));
        inner.attach_stop_requests(stops.clone());
        bus.with(|s| s.push_all_feedback());
        let stamp = inner.current_admission_stamp();
        SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
        inner.tick(0.01);

        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let stamp = inner.current_admission_stamp();
        let ep = inner.safety.epoch_id();
        req::install_park(&mut inner, stamp, &park.slot, None, Some(ep), false).unwrap();
        if rt_owned {
            inner.rt_executor_owned = true;
        }

        let mut armed = false;
        for _ in 0..320 {
            clock.advance(Duration::from_millis(10));
            bus.with(|s| s.push_all_feedback());
            if rt_owned {
                let b = armd::rt_core::ops::ServiceBudget {
                    now: clock.now(),
                    deadline: clock.now() + Duration::from_millis(2),
                    max_rx_frames: 32,
                };
                let _ = armd::rt_core::ops::CoreOps::poll_hardware(&mut inner, &b);
            }
            inner.tick(0.01);
            if !armed && inner.unload_settle.as_ref().is_some_and(|u| u.held_s >= 0.70) {
                armed = true;
                fire_tx.store(true, Ordering::SeqCst);
            }
            if !inner.pending_normal_unload && !inner.torque_transition_pending() {
                break;
            }
        }
        assert!(armed, "rt_owned={rt_owned}: unload_settle must reach >= 0.70s");
        assert!(!fire_tx.load(Ordering::SeqCst), "rt_owned={rt_owned}: Stop after motor 0 must have fired");
        assert_eq!(disable_tx.load(Ordering::SeqCst), 1, "rt_owned={rt_owned}: only motor 0 disabled before Stop");
        assert_eq!(inner.safety.fault_code(), Some(FaultCode::DriverWriteFailed));
        assert_eq!(inner.torque_intent, armd::TorqueIntent::On);

        // Advance > 600 ms (past both immediate RT retry and 250 ms legacy fallback) with fresh rest feedback:
        // motor 1 must NEVER receive a first disable write from any background retry after Stop.
        for _ in 0..35 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            if rt_owned {
                let b = armd::rt_core::ops::ServiceBudget {
                    now: clock.now(),
                    deadline: clock.now() + Duration::from_millis(2),
                    max_rx_frames: 32,
                };
                let _ = armd::rt_core::ops::CoreOps::poll_hardware(&mut inner, &b);
            }
            inner.tick(0.02);
        }
        assert_eq!(
            disable_tx.load(Ordering::SeqCst),
            1,
            "rt_owned={rt_owned}: motor 1 must NEVER receive its first disable from retry after Stop"
        );
        assert_eq!(inner.torque_intent, armd::TorqueIntent::On);
    }

    // 10c. SO-ARM101 / Feetech (`FeetechArmDriver`) per-servo Stop / stop_gen gate:
    //      direct driver checks (pending Stop, stale stop_gen, WouldBlock on servo 1 + Stop before
    //      servo 2, Stop after servo 1, watchdog fallback) + end-to-end `begin_park_disable_transition`.
    {
        struct RaceStopFeetechTransport {
            clock: Arc<armd::transport::MockClock>,
            rx: std::collections::VecDeque<u8>,
            torque_enabled: [bool; 2],
            disable_writes: Arc<std::sync::atomic::AtomicUsize>,
            wb_and_stop_on_first_disable: Arc<AtomicBool>,
            stop_after_first_disable: Arc<AtomicBool>,
            stops: StopRequests,
        }

        impl RaceStopFeetechTransport {
            fn feetech_resp(id: u8, params: &[u8]) -> Vec<u8> {
                let len = (params.len() + 2) as u8;
                let mut sum: u32 = u32::from(id) + u32::from(len);
                for &b in params { sum += u32::from(b); }
                let checksum = !(sum as u8);
                let mut pkt = vec![0xFF, 0xFF, id, len, 0x00];
                pkt.extend_from_slice(params);
                pkt.push(checksum);
                pkt
            }
        }

        impl Transport for RaceStopFeetechTransport {
            fn clock(&self) -> Option<Arc<dyn DriverClock>> { Some(self.clock.clone()) }
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
                if self.rx.is_empty() {
                    return Err(TransportError::WouldBlock);
                }
                let n = buf.len().min(self.rx.len());
                for b in buf.iter_mut().take(n) {
                    *b = self.rx.pop_front().unwrap();
                }
                Ok(n)
            }
            fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
                // Feetech WRITE packet: [0xFF, 0xFF, id, 4, INST_WRITE(3), REG_TORQUE_ENABLE(40), val, csum]
                if buf.len() >= 8
                    && buf[0] == 0xFF
                    && buf[1] == 0xFF
                    && buf[4] == feetech::INST_WRITE
                    && buf[5] == feetech::REG_TORQUE_ENABLE
                {
                    let id = buf[2];
                    let val = buf[6];
                    if val == 0 {
                        if self.wb_and_stop_on_first_disable.swap(false, Ordering::SeqCst) {
                            let _ = self.stops.request();
                            return Err(TransportError::WouldBlock);
                        }
                        self.disable_writes.fetch_add(1, Ordering::SeqCst);
                        if (1..=2).contains(&id) {
                            self.torque_enabled[(id - 1) as usize] = false;
                        }
                        if self.stop_after_first_disable.swap(false, Ordering::SeqCst) {
                            let _ = self.stops.request();
                        }
                    } else if (1..=2).contains(&id) {
                        self.torque_enabled[(id - 1) as usize] = true;
                    }
                } else if buf.len() >= 8
                    && buf[0] == 0xFF
                    && buf[1] == 0xFF
                    && buf[4] == feetech::INST_READ
                    && buf[5] == feetech::REG_TORQUE_ENABLE
                {
                    let id = buf[2];
                    let val = if (1..=2).contains(&id) && self.torque_enabled[(id - 1) as usize] { 1u8 } else { 0u8 };
                    self.rx.extend(Self::feetech_resp(id, &[val]));
                } else if buf.len() >= 8
                    && buf[0] == 0xFF
                    && buf[1] == 0xFF
                    && buf[2] == feetech::BROADCAST_ID
                    && buf[4] == feetech::INST_SYNC_READ
                {
                    let reg = buf[5];
                    let data_len = buf[6];
                    let ids = &buf[7..buf.len() - 1];
                    for &id in ids {
                        if reg == feetech::REG_TORQUE_ENABLE && data_len == 1 {
                            let val = if (1..=2).contains(&id) && self.torque_enabled[(id - 1) as usize] { 1u8 } else { 0u8 };
                            self.rx.extend(Self::feetech_resp(id, &[val]));
                        } else if reg == feetech::REG_PRESENT_POSITION_L && data_len == 8 {
                            // Neutral position (2048 steps = 0 rad), velocity 0, load 0, voltage 120, temp 25.
                            let pos = 2048u16.to_le_bytes();
                            self.rx.extend(Self::feetech_resp(id, &[pos[0], pos[1], 0, 0, 0, 0, 120, 25]));
                        }
                    }
                }
                Ok(buf.len())
            }
            fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
            fn clear_input_buffer(&mut self) { self.rx.clear(); }
        }

        // Direct FeetechArmDriver checks
        {
            let clock = Arc::new(armd::transport::MockClock::new());
            let stops = StopRequests::default();
            let req_estop = Arc::new(std::sync::atomic::AtomicU64::new(0));
            let app_estop = Arc::new(std::sync::atomic::AtomicU64::new(0));
            let disable_writes = Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let wb_stop = Arc::new(AtomicBool::new(false));
            let stop_after = Arc::new(AtomicBool::new(false));
            let transport = RaceStopFeetechTransport {
                clock: clock.clone(),
                rx: std::collections::VecDeque::new(),
                torque_enabled: [false, false],
                disable_writes: disable_writes.clone(),
                wb_and_stop_on_first_disable: wb_stop.clone(),
                stop_after_first_disable: stop_after.clone(),
                stops: stops.clone(),
            };
            let mut drv = FeetechArmDriver::new(transport, vec![1, 2])
                .with_calibrated(true)
                .with_tx_echo(Some(false));
            drv.set_estop_gate(EstopGate::new(req_estop, app_estop).with_stops(stops.clone()));
            let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
            arm_joints[0] = 0;
            arm_joints[1] = 1;
            drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
                safety_stopped: Arc::new(AtomicBool::new(false)),
                has_resting_pose: true,
                arm_joints,
                arm_joints_len: 2,
                rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
                rest_pose_tolerance_rad: 0.05,
            });
            drv.read_joint_states().unwrap();
            drv.set_torque(true).unwrap();
            for _ in 0..3 {
                clock.advance(Duration::from_millis(12));
                drv.read_joint_states().unwrap();
            }
            assert_eq!(drv.enable_state(), EnableState::Enabled);
            assert_eq!(disable_writes.load(Ordering::SeqCst), 0);

            // (1) Pending client Stop before servo 1 -> EstopInhibited, 0 writes, stays Enabled.
            let sg1 = stops.request();
            let err1 = drv.begin_client_park_disable(None, Some(0)).unwrap_err();
            assert!(armd::driver::is_estop_inhibited(&err1), "expected EstopInhibited, got {err1:?}");
            assert_eq!(disable_writes.load(Ordering::SeqCst), 0);
            assert_eq!(drv.enable_state(), EnableState::Enabled);

            // (2) Applied Stop with stale expected_stop_gen -> EstopInhibited, 0 writes.
            stops.mark_applied(sg1);
            let err2 = drv.begin_client_park_disable(None, Some(0)).unwrap_err();
            assert!(armd::driver::is_estop_inhibited(&err2), "stale stop_gen must be refused: {err2:?}");
            assert_eq!(disable_writes.load(Ordering::SeqCst), 0);
            assert_eq!(drv.enable_state(), EnableState::Enabled);

            // (3) WouldBlock on servo 1 followed by Stop before servo 2 -> EstopInhibited, 0 writes!
            wb_stop.store(true, Ordering::SeqCst);
            let err3 = drv.begin_client_park_disable(None, Some(sg1)).unwrap_err();
            assert!(armd::driver::is_estop_inhibited(&err3), "WouldBlock + Stop before servo 2 must return EstopInhibited, got {err3:?}");
            assert_eq!(disable_writes.load(Ordering::SeqCst), 0, "servo 2 disable must NOT write after servo 1 WouldBlock + Stop");
            assert_eq!(drv.enable_state(), EnableState::Enabled);

            // (4) Servo 1 succeeds, then Stop arrives before servo 2 -> servo 2 MUST NOT write!
            let sg2 = stops.current();
            assert!(sg2 > sg1 && stops.pending());
            stops.mark_applied(sg2);
            stop_after.store(true, Ordering::SeqCst);
            let err4 = drv.begin_client_park_disable(None, Some(sg2)).unwrap_err();
            assert!(!armd::driver::is_estop_inhibited(&err4), "partial servo disable must report error: {err4:?}");
            assert_eq!(disable_writes.load(Ordering::SeqCst), 1, "servo 2 must NOT be disabled after Stop arrives mid-batch");

            // (5) Safety/watchdog `set_torque(false)` ignores pending client Stop and disables all servos.
            let sg3 = stops.current();
            assert!(sg3 > sg2 && stops.pending());
            stops.mark_applied(sg3);
            for _ in 0..3 {
                clock.advance(Duration::from_millis(12));
                drv.read_joint_states().unwrap();
            }
            drv.set_torque(false).unwrap();
            drv.read_joint_states().unwrap();
            drv.set_torque(true).unwrap();
            for _ in 0..3 {
                clock.advance(Duration::from_millis(12));
                drv.read_joint_states().unwrap();
            }
            let _ = stops.request();
            assert!(stops.pending());
            disable_writes.store(0, Ordering::SeqCst);
            drv.set_torque(false).expect("watchdog/safety disable must ignore client Stop");
            assert_eq!(disable_writes.load(Ordering::SeqCst), 2);
            assert_eq!(drv.enable_state(), EnableState::Disabled);
        }

        // End-to-end `SupervisorInner::begin_park_disable_transition` on FeetechArmDriver:
        // servo 1 WouldBlock + Stop before servo 2 -> 0 writes, torque stays ON, park cancelled STOPPED.
        {
            let clock = Arc::new(armd::transport::MockClock::new());
            let stops = StopRequests::default();
            let disable_writes = Arc::new(std::sync::atomic::AtomicUsize::new(0));
            let wb_stop = Arc::new(AtomicBool::new(false));
            let stop_after = Arc::new(AtomicBool::new(false));
            let transport = RaceStopFeetechTransport {
                clock: clock.clone(),
                rx: std::collections::VecDeque::new(),
                torque_enabled: [false, false],
                disable_writes: disable_writes.clone(),
                wb_and_stop_on_first_disable: wb_stop.clone(),
                stop_after_first_disable: stop_after,
                stops: stops.clone(),
            };
            let drv = FeetechArmDriver::new(transport, vec![1, 2])
                .with_calibrated(true)
                .with_tx_echo(Some(false));
            let mut cfg = parkable_2dof_config();
            if let Some(rp) = cfg.resting_pose.as_mut() {
                rp.support_torque_limits.clear();
                rp.support_load_limits = vec![(0, 1.0)];
            }
            let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
            inner.attach_stop_requests(stops.clone());
            inner.tick(0.01);
            let stamp = inner.current_admission_stamp();
            SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
            inner.tick(0.01);

            let snap = MotionPlanSnapshot::from_supervisor(&inner);
            let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
            let stamp = inner.current_admission_stamp();
            let ep = inner.safety.epoch_id();
            req::install_park(&mut inner, stamp, &park.slot, None, Some(ep), false).unwrap();

            let mut armed = false;
            for _ in 0..320 {
                clock.advance(Duration::from_millis(10));
                inner.tick(0.01);
                if !armed && inner.unload_settle.as_ref().is_some_and(|u| u.held_s >= 0.70) {
                    armed = true;
                    wb_stop.store(true, Ordering::SeqCst);
                }
                if !inner.pending_normal_unload && !inner.torque_transition_pending() {
                    break;
                }
            }
            assert!(armed, "Feetech unload_settle must reach >= 0.70s");
            assert!(!wb_stop.load(Ordering::SeqCst), "Feetech servo 1 WouldBlock + Stop race must have fired");
            assert_eq!(disable_writes.load(Ordering::SeqCst), 0, "zero Feetech disable writes on servo 1 WouldBlock + Stop");
            assert!(inner.is_torqued && !inner.disable_pending, "Feetech torque must stay held");
            assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("STOPPED"));
            assert_ne!(inner.safety.fault_code(), Some(FaultCode::DriverWriteFailed));
        }
    }
}

// ============================================================================
// 11. M4: 9-DOF Profile (8-joint URDF chain + gripper) Inline Motion & >9 Rejection
// ============================================================================

struct TempUrdfFile(std::path::PathBuf);
impl Drop for TempUrdfFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

fn write_eight_joint_urdf() -> TempUrdfFile {
    let mut xml = String::from("<robot name=\"arm8\">\n  <link name=\"base_link\"/>\n");
    for i in 1..=8 {
        let parent = if i == 1 { "base_link".to_string() } else { format!("link{}", i - 1) };
        let child = format!("link{i}");
        xml.push_str(&format!(
            "  <link name=\"{child}\"><inertial><origin xyz=\"0.025 0 0\" rpy=\"0 0 0\"/><mass value=\"0.1\"/><inertia ixx=\"1e-4\" ixy=\"0\" ixz=\"0\" iyy=\"1e-4\" iyz=\"0\" izz=\"1e-4\"/></inertial></link>\n\
             \x20 <joint name=\"joint{i}\" type=\"revolute\">\n\
             \x20   <parent link=\"{parent}\"/>\n\
             \x20   <child link=\"{child}\"/>\n\
             \x20   <origin xyz=\"0.05 0 0\" rpy=\"0 0 0\"/>\n\
             \x20   <axis xyz=\"0 0 1\"/>\n\
             \x20   <limit lower=\"-3.14\" upper=\"3.14\" effort=\"10\" velocity=\"2.0\"/>\n\
             \x20 </joint>\n"
        ));
    }
    xml.push_str("</robot>\n");
    let pid = std::process::id();
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let path = std::env::temp_dir().join(format!("armd_m4_8dof_{pid}_{nanos}.urdf"));
    std::fs::write(&path, xml).expect("write temp 8-joint URDF");
    TempUrdfFile(path)
}

fn nine_dof_urdf_plus_gripper_config(urdf_path: &std::path::Path) -> ArmConfig {
    let dm = ArmConfig::rebot_dm();
    let mut v = serde_json::to_value(&dm).unwrap();
    let o = v.as_object_mut().unwrap();
    o.insert("profile_id".into(), "urdf_8dof_plus_gripper".into());
    o.insert("kinematics".into(), "Urdf".into());
    o.insert("gravity_model".into(), "Urdf".into());
    o.insert("urdf".into(), urdf_path.to_str().unwrap().into());
    o.remove("urdf_sha256");
    o.remove("model_offsets_rad");
    o.insert(
        "chain".into(),
        json!({
            "base_link": "base_link",
            "tip_link": "link8",
            "joints": (1..=8).map(|i| json!({
                "urdf_joint": format!("joint{i}"),
                "offset_rad": 0.0
            })).collect::<Vec<_>>(),
        }),
    );
    o.remove("workspace");
    let mut cfg: ArmConfig = serde_json::from_value(v).unwrap();
    cfg.dof = 9;
    cfg.zero_reference = None;
    cfg.joint_names = (1..=8).map(|i| format!("j{i}")).chain(std::iter::once("gripper".into())).collect();
    cfg.limits = (0..9)
        .map(|i| JointLimit {
            name: if i < 8 { format!("j{}", i + 1) } else { "gripper".into() },
            min_position: -3.0,
            max_position: 3.0,
            max_velocity: if i < 8 { 2.0 } else { 3.0 },
            max_acceleration: 5.0,
        })
        .collect();
    cfg.home_positions = vec![0.0; 9];
    cfg.gripper_joint = Some(8);
    cfg.gripper_policy = None;
    cfg.grasp = None;
    cfg.gravity_comp = None;
    cfg.torque_guard = None;
    cfg.damiao_ranges = None;
    cfg.servo_max_velocity = None;
    cfg.planned_max_velocity = None;
    cfg.speed_cap = None;
    cfg.power_budget = None;
    cfg.joint_tracking = None;
    cfg.lease_timeout_ms = 60_000;
    cfg.resting_pose = Some(RestingPosePolicy {
        positions: Some(vec![0.0; 9]),
        position_tolerances: vec![0.08; 9],
        max_total_deviation: 0.72,
        max_velocity: 0.05,
        joint_max_velocities: vec![],
        support_torque_limits: vec![(0, 1.0)],
        support_load_limits: vec![],
        feedback_max_age_ms: 200,
    });
    cfg.load_arm_model(None).expect("load 8-joint URDF chain");
    cfg.validate().expect("9-DOF profile (8-joint URDF + gripper) must validate");
    cfg
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn m4_nine_dof_urdf_chain_plus_gripper_motion_and_over_limit_rejection() {
    let urdf = write_eight_joint_urdf();
    let cfg = nine_dof_urdf_plus_gripper_config(&urdf.0);
    assert_eq!(cfg.arm_dof(), 8);
    assert_eq!(cfg.dof, 9);
    assert_eq!(cfg.gripper_index(), Some(8));

    // 11a. >9 DOF profile is refused at profile load with a clear error (no silent drop).
    let mut bad_cfg = ArmConfig::default();
    bad_cfg.dof = armd::model::MAX_MOTION_DOF + 1;
    bad_cfg.limits = (0..bad_cfg.dof)
        .map(|i| JointLimit::new(format!("j{}", i + 1), -3.0, 3.0, 2.0, 5.0))
        .collect();
    bad_cfg.home_positions = vec![0.0; bad_cfg.dof];
    let err = bad_cfg.validate().expect_err("DOF > MAX_MOTION_DOF must fail validation");
    assert!(
        err.contains("exceeds maximum supported"),
        "expected clear over-limit error, got: {err}"
    );

    // 11b. RT executor refuses 9-DOF explicitly (503 UNSUPPORTED_RT_CONFIG) instead of truncating.
    let (drv_rt, _) = MotionTestDriver::new(vec![0.0; 9], Duration::ZERO, Duration::ZERO, None);
    let sup_rt = Arc::new(ArmSupervisor::new(cfg.clone()).with_driver(Box::new(drv_rt)));
    let rt_err = sup_rt
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .err()
        .expect("RT executor must reject 9-DOF profile explicitly");
    assert!(
        matches!(rt_err, ArmError::Rejected { status: 503, ref code, .. } if code == "UNSUPPORTED_RT_CONFIG"),
        "unexpected RT executor rejection: {rt_err:?}"
    );

    // 11c. Legacy WS jog, home, unload (park) + ArmSupervisor::{jog,home} move ALL 9 joints
    //      (including joint 8 and gripper at index 8) without silent drop.
    let (drv, _) = MotionTestDriver::new(vec![0.0; 9], Duration::ZERO, Duration::ZERO, None);
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    let (mut ws, gen, _, sup, _) = common::torqued_rig(sup, &[0.0; 9]).await;
    let epoch = sup.get_observation().await.epoch_id;

    let target9 = vec![0.10, -0.10, 0.15, -0.15, 0.20, -0.20, 0.25, -0.35, 0.45];
    let jog_ack = common::rpc(
        &mut ws,
        gen,
        epoch,
        "/api/v1/motion/jog",
        json!({ "target_positions": target9, "duration_secs": 0.2, "speed_scale": 0.5 }),
    )
    .await;
    assert_eq!(jog_ack["type"], "command_ack", "{jog_ack}");
    {
        let inner = sup.inner().read().await;
        let (caps, cap_dof) = inner
            .active_trajectory
            .as_ref()
            .and_then(|t| t.motion_vel_caps)
            .expect("9-DOF inline motion_vel_caps populated");
        assert_eq!(cap_dof, 9);
        assert!((caps[7] - 1.0).abs() < 1e-5, "arm joint 8 cap scaled by 0.5: {}", caps[7]);
        assert!((caps[8] - 3.0).abs() < 1e-5, "gripper cap unscaled: {}", caps[8]);
    }
    for _ in 0..200 {
        let mut inner = sup.inner().write().await;
        inner.tick(0.01);
        if inner.active_trajectory.is_none() {
            break;
        }
    }
    assert!(sup.inner().read().await.active_trajectory.is_none(), "9-DOF WS jog trajectory must finish");
    let obs_after_jog = sup.get_observation().await;
    assert_eq!(obs_after_jog.commanded_positions.len(), 9);
    for (i, (&got, &want)) in obs_after_jog.commanded_positions.iter().zip(&target9).enumerate() {
        assert!((got - want).abs() < 1e-3, "joint {i} did not reach target {want}: got {got}");
    }

    // Home returns all 9 joints (including j8 and gripper) back to 0.0.
    let home_ack = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/home", json!({ "duration_secs": 0.2 })).await;
    assert_eq!(home_ack["type"], "command_ack", "{home_ack}");
    for _ in 0..200 {
        let mut inner = sup.inner().write().await;
        inner.tick(0.01);
        if inner.active_trajectory.is_none() {
            break;
        }
    }
    assert!(sup.inner().read().await.active_trajectory.is_none(), "9-DOF WS home trajectory must finish");
    let obs_after_home = sup.get_observation().await;
    for (i, &got) in obs_after_home.commanded_positions.iter().enumerate() {
        assert!(got.abs() < 1e-3, "joint {i} did not home to 0.0: got {got}");
    }

    // Direct ArmSupervisor::jog and ArmSupervisor::home on 9-DOF profile.
    let direct_target = vec![0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.22, -0.33];
    let stamp = sup.capture_admission_stamp();
    sup.jog(stamp, direct_target.clone(), Some(0.15), Some(epoch)).await.expect("direct 9-DOF jog");
    for _ in 0..200 {
        let mut inner = sup.inner().write().await;
        inner.tick(0.01);
        if inner.active_trajectory.is_none() {
            break;
        }
    }
    assert!(sup.inner().read().await.active_trajectory.is_none(), "9-DOF direct jog must finish");
    let obs_direct = sup.get_observation().await;
    assert!((obs_direct.commanded_positions[7] - 0.22).abs() < 1e-3);
    assert!((obs_direct.commanded_positions[8] - (-0.33)).abs() < 1e-3);

    // Unload (park) moves all 9 joints back to resting pose, settles all 9 in UnloadSettleSample,
    // and disables torque cleanly to SafetyState::Idle.
    let unload_ack = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/unload", json!({})).await;
    assert_eq!(unload_ack["type"], "command_ack", "{unload_ack}");
    let mut saw_9dof_settle = false;
    for _ in 0..300 {
        let mut inner = sup.inner().write().await;
        inner.tick(0.01);
        if let Some(sample) = inner.unload_settle.as_ref().and_then(|u| u.samples.back()) {
            assert_eq!(sample.dof, 9, "UnloadSettleSample must track all 9 joints");
            assert!(sample.positions[8].is_finite(), "gripper settle sample must be finite");
            saw_9dof_settle = true;
        }
        if !inner.pending_normal_unload && !inner.is_torqued {
            break;
        }
    }
    assert!(saw_9dof_settle, "park settle must record 9-DOF UnloadSettleSample");
    let final_obs = sup.get_observation().await;
    assert_eq!(final_obs.safety_state, SafetyState::Idle);
    assert!(!final_obs.is_torqued);
    for (i, &got) in final_obs.commanded_positions.iter().enumerate() {
        assert!(got.abs() < 1e-3, "joint {i} did not reach resting pose 0.0: got {got}");
    }
}

// ============================================================================
// 12. Round 3 Item 2: Deferred park carries ORIGINAL request deadline on BOTH owners
//     (no fresh 15 s timeout, no separately timed ContinuePark past original deadline)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r3_deferred_park_enforces_original_deadline_on_both_owners() {
    // 12a. Legacy WS owner: deferred park continuation uses the ORIGINAL request deadline
    //      (80 ms here, NOT a fresh 15 s timeout); when rearm outlives the original deadline,
    //      continuation fails DEADLINE_EXPIRED with no motion, and direct install past
    //      `cont.deadline` is refused even with a freshly-extended slot deadline.
    {
        let (drv, _knobs) = MotionTestDriver::new(
            vec![0.4, 0.2],
            Duration::from_millis(200), // longer than the 80 ms original request deadline
            Duration::from_millis(5),
            None,
        );
        let sup = Arc::new(ArmSupervisor::new(parkable_2dof_config()).with_driver(Box::new(drv)));
        let (mut ws, gen, _, sup, _) = common::torqued_rig(sup, &[0.4, 0.2]).await;

        // Enter holding_after_clear via E-stop + clear_fault.
        let _ = sup.trigger_e_stop().await;
        sup.inner().write().await.tick(0.01);
        sup.clear_fault().await.expect("clear_fault -> holding_after_clear");
        assert!(sup.inner().read().await.holding_after_clear());

        sup.set_motion_plan_deadline(Duration::from_millis(80)).await;
        let epoch = sup.get_observation().await.epoch_id;
        let req_started = Instant::now();
        let ack = common::rpc(&mut ws, gen, epoch, "/api/v1/motion/unload", json!({})).await;
        assert_eq!(ack["type"], "command_ack", "{ack}");

        let cont = sup
            .inner()
            .read()
            .await
            .park_continuation()
            .expect("deferred park continuation installed");
        assert_eq!(cont.phase, ParkContinuationPhase::WaitingRearm);
        assert!(
            cont.deadline <= req_started + Duration::from_millis(500),
            "continuation deadline must reflect the original ~80 ms request deadline, not a fresh 15 s timeout"
        );

        // Wait past the original 80 ms deadline (well under 15 s) without letting rearm complete first.
        tokio::time::sleep(Duration::from_millis(140)).await;
        for _ in 0..10 {
            sup.inner().write().await.tick(0.01);
            tokio::time::sleep(Duration::from_millis(10)).await;
            if sup.inner().read().await.park_continuation().is_none() {
                break;
            }
        }
        let inner = sup.inner().read().await;
        assert!(inner.park_continuation().is_none(), "expired park continuation must be cleared");
        assert!(inner.active_trajectory.is_none(), "no motion may start after original deadline");
        assert_eq!(inner.last_action.as_ref().and_then(|a| a.code.as_deref()), Some("DEADLINE_EXPIRED"));
        assert!(inner.is_torqued && !inner.pending_normal_unload);
    }

    // 12b. Direct `install_park_continuation_at`: even when `slot.authority.deadline` is freshly
    //      extended far into the future, `now > cont.deadline` is refused with `DeadlineExpired`.
    {
        let (drv, _) = MotionTestDriver::new(vec![0.4, 0.2], Duration::ZERO, Duration::ZERO, None);
        let mut inner = SupervisorInner::new(parkable_2dof_config()).with_driver(Box::new(drv));
        let stamp = inner.current_admission_stamp();
        SupervisorInner::enable_torque(inner.check_admission(stamp).unwrap()).unwrap();
        inner.tick(0.01);
        inner.execute_stop(armd::safety::StopPolicy::EStopLatch);
        inner.tick(0.01);
        inner.clear_fault().unwrap();
        assert!(inner.holding_after_clear());

        let t0 = Instant::now();
        let orig_deadline = t0 + Duration::from_millis(50);
        let snap = MotionPlanSnapshot::from_supervisor(&inner).with_deadline(orig_deadline);
        let park = req::prepare_motion(&snap, &MotionRequest::park()).unwrap();
        let stamp = inner.current_admission_stamp();
        let ep = inner.safety.epoch_id();
        let boot = inner.boot_id();
        let installed = req::install_park_at(&mut inner, stamp, &park.slot, None, Some(ep), false, boot, t0).unwrap();
        let action_id = installed.action_id();
        assert_eq!(inner.park_continuation().unwrap().deadline, orig_deadline);
        inner.tick(0.01);
        assert_eq!(inner.park_continuation().unwrap().phase, ParkContinuationPhase::ReadyForPlan);

        let fresh_snap = MotionPlanSnapshot::from_supervisor(&inner)
            .with_deadline(t0 + Duration::from_secs(60));
        let cont_plan = req::prepare_motion(&fresh_snap, &MotionRequest::park()).unwrap();
        let ep2 = inner.safety.epoch_id();
        let err = req::install_park_continuation_at(
            &mut inner,
            action_id,
            &cont_plan.slot,
            None,
            Some(ep2),
            false,
            boot,
            orig_deadline + Duration::from_millis(1),
        )
        .unwrap_err();
        assert_eq!(err, MotionInstallError::DeadlineExpired);
        assert!(inner.active_trajectory.is_none());
    }

    // 12c. RT executor owner: `Submit::InstallPark` records original deadline on `ParkContinuation`;
    //      advancing `ScriptedClock` past it causes `Submit::ContinuePark` (even with a fresh 10 s
    //      broker deadline and fresh slot deadline) to be refused `DEADLINE_EXPIRED` with 0 alloc.
    {
        let period = Duration::from_millis(5);
        let period_ns = period.as_nanos() as u64;
        let clock = ScriptedClock::new();
        let (drv, _) = MotionTestDriver::new(
            vec![0.4, 0.2],
            Duration::from_millis(10),
            Duration::from_millis(4),
            Some(clock.clone()),
        );
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
        handle.clear_fault_async().await.unwrap().expect("clear_fault succeeds");
        maint_done.store(true, Ordering::Release);
        step_jh.await.unwrap();
        clock.run_steps(period_ns, 2);

        let held_snap = sup.hub().latest().unwrap();
        assert!(held_snap.rt.holding_after_clear);
        let hub_cfg = sup.hub().motion_config().unwrap();
        let orig_deadline = clock.now() + Duration::from_millis(40);
        let p_snap = MotionPlanSnapshot::from_published(&held_snap, hub_cfg.clone())
            .unwrap()
            .with_deadline(orig_deadline);
        let park_rearm = req::prepare_motion(&p_snap, &MotionRequest::park()).unwrap();
        let auth_clear = auth_from_hub(&sup, &handle);
        let r_park = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                3,
                auth_clear,
                orig_deadline,
                Submit::InstallPark(&|s: &mut PlanSlot| req::stage_park(s, &park_rearm.slot)),
            )
            .unwrap();
        clock.run_steps(period_ns, 1);
        let park_action = drain_replies(&handle)
            .iter()
            .find_map(|r| match r {
                Reply::Ack { request, action: Some(a), .. } if *request == r_park => Some(a.0),
                _ => None,
            })
            .expect("InstallPark ACK with action_id");

        // Wait for ParkRearm to confirm (10 ms), then advance clock PAST original 40 ms deadline.
        for _ in 0..20 {
            clock.run_steps(period_ns, 1);
            if clock.now() > orig_deadline + Duration::from_millis(10) {
                break;
            }
        }
        assert!(clock.now() > orig_deadline);
        let ready_snap = sup.hub().latest().unwrap();
        assert!(!ready_snap.rt.holding_after_clear && ready_snap.obs.safety_state == SafetyState::Armed);
        let pos_before = ready_snap.obs.commanded_positions.clone();

        // Attempt a separately timed `ContinuePark` with a fresh 10 s deadline on BOTH slot and broker.
        let fresh_deadline = clock.now() + Duration::from_secs(10);
        let cont_snap = MotionPlanSnapshot::from_published(&ready_snap, hub_cfg)
            .unwrap()
            .with_deadline(fresh_deadline);
        let cont_plan = req::prepare_motion(&cont_snap, &MotionRequest::park()).unwrap();
        let r_cont = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                4,
                auth_from_hub(&sup, &handle),
                fresh_deadline,
                Submit::ContinuePark(&|s: &mut PlanSlot| req::stage_park_continuation(s, park_action, &cont_plan.slot)),
            )
            .unwrap();
        clock.run_steps(period_ns, 4);
        let rs = drain_replies(&handle);
        assert!(
            rs.iter().any(|r| matches!(r, Reply::Refused { request, code: "DEADLINE_EXPIRED", .. } if *request == r_cont)),
            "ContinuePark past original deadline must be refused DEADLINE_EXPIRED; got {rs:?}"
        );
        let snap_after = sup.hub().latest().unwrap();
        assert_eq!(snap_after.obs.commanded_positions, pos_before, "no motion may start after deadline expiry");
        assert!(snap_after.obs.is_torqued);
        assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
        drop(handle);
    }
}

// ============================================================================
// 11. Per-Command Lock Attribution in /api/v1/diagnostics/tick (Zero-Alloc Hot Path)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_lock_attribution_records_static_command_and_action_id_in_tick_diagnostics_zero_alloc() {
    use armd::diagnostics::alloc::{snapshot_counts, time_this_thread};
    use armd::diagnostics::tick::{Holder, LockFreeTickStats, TickSample};
    use armd::supervisor_lock::SupervisorLock;

    // 11a. Strictly zero heap allocations/frees on the lock-attribution hot path:
    //      set_command + set_action_id + guard drop -> HoldLog -> take_log -> LockFreeTickStats::record.
    {
        let (counts, longest) = std::thread::spawn(|| {
            let lock = SupervisorLock::new(0u64);
            let lf = LockFreeTickStats::new();
            let loc = std::panic::Location::caller();
            time_this_thread();
            let before = snapshot_counts();
            {
                let mut w = lock.try_write().expect("uncontended try_write");
                w.set_command("/api/v1/motion/jog");
                w.set_action_id(123);
                *w += 1;
            }
            let log = lock.take_log().expect("take_log");
            let r = log.last_release.expect("recorded hold");
            let h = Holder {
                kind: r.kind.as_str(),
                location: loc,
                command: r.command,
                direction: r.direction,
                action_id: r.action_id,
                io_timing: r.io_timing,
                hold_us: 10,
                acquire_us: 1,
                acquired_before_wait_us: 0,
                released_before_lock_us: 0,
            };
            let ok = lf.record(&TickSample {
                tick_us: 50,
                lock_us: 20,
                waited_for: Some(h),
                longest_hold: Some(h),
                ..Default::default()
            });
            let delta = snapshot_counts().delta_since(before);
            assert!(ok);
            let snap = lf.snapshot();
            (delta, snap.max_tick.unwrap().sample.longest_hold.unwrap())
        })
        .join()
        .unwrap();
        assert_eq!(counts, AllocCounts::ZERO, "lock attribution hot path must allocate 0 bytes");
        assert_eq!(longest.command, Some("/api/v1/motion/jog"));
        assert_eq!(longest.action_id, Some(123));
    }

    // 11b. Real WS + HTTP GET /api/v1/diagnostics/tick: verify lock breakdowns name the WS
    //      command path and action_id when the tick observes/waits on a WS write hold.
    {
        let cfg = dm_config_with_planning_load();
        let mut q0 = cfg.home_positions.clone();
        q0[0] += 0.20;
        let sup = Arc::new(ArmSupervisor::new(cfg));
        let (mut ws, gen, _, sup, addr) = common::torqued_rig(sup, &q0).await;
        // Inject a 25 ms hold (> 20 ms tick period) inside Lock #2 of `/api/v1/motion/jog` so
        // the 50 Hz control tick deterministically blocks on this write guard, guaranteeing that
        // `max_tick` in `GET /api/v1/diagnostics/tick` names `/api/v1/motion/jog` and its action_id
        // in both `waited_for` and `longest_hold`.
        {
            let mut inner = sup.inner().write().await;
            inner.before_install_hook = Some(Box::new(|_| {
                std::thread::sleep(Duration::from_millis(25));
            }));
        }
        let ep = sup.get_observation().await.epoch_id;
        let mut jog_t = q0.clone();
        jog_t[0] -= 0.08;
        let ack = common::rpc(
            &mut ws,
            gen,
            ep,
            "/api/v1/motion/jog",
            json!({"target_positions": jog_t, "duration_secs": 0.4}),
        )
        .await;
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let action_id = ack["action_id"].as_u64().expect("jog action_id");
        // Wait for at least two 50 Hz ticks so the hold log is drained into `LockFreeTickStats`.
        tokio::time::sleep(Duration::from_millis(80)).await;

        let resp = reqwest::get(format!("http://{addr}/api/v1/diagnostics/tick"))
            .await
            .expect("GET /api/v1/diagnostics/tick");
        assert!(resp.status().is_success());
        let body: serde_json::Value = resp.json().await.expect("tick diagnostics JSON");

        // Verify raw tick samples captured BOTH command path and action_id on the jog Lock #2 hold.
        let samples = sup.tick_samples();
        let jog_holder = samples
            .iter()
            .flat_map(|s| s.longest_hold.into_iter().chain(s.waited_for))
            .find(|h| h.command == Some("/api/v1/motion/jog") && h.action_id == Some(action_id));
        assert!(
            jog_holder.is_some(),
            "expected tick_samples to attribute /api/v1/motion/jog action_id={action_id}; samples={:?}",
            samples
                .iter()
                .filter_map(|s| s.longest_hold.map(|h| (h.command, h.action_id, h.hold_us)))
                .collect::<Vec<_>>()
        );

        // Verify the HTTP JSON lock breakdowns (`max_tick` / `max_release`) expose `command` and `action_id`.
        let mt = &body["max_tick"];
        let breakdown_holder = mt
            .get("longest_hold")
            .filter(|h| h.get("command").and_then(|c| c.as_str()) == Some("/api/v1/motion/jog"))
            .or_else(|| {
                mt.get("waited_for")
                    .filter(|h| h.get("command").and_then(|c| c.as_str()) == Some("/api/v1/motion/jog"))
            })
            .or_else(|| {
                body["max_release"]
                    .get("longest_hold")
                    .filter(|h| h.get("command").and_then(|c| c.as_str()) == Some("/api/v1/motion/jog"))
            });
        let h_json = breakdown_holder.unwrap_or_else(|| {
            panic!("expected /api/v1/diagnostics/tick lock breakdown to name /api/v1/motion/jog; body={body}")
        });
        assert_eq!(h_json["command"], "/api/v1/motion/jog", "{h_json}");
        assert_eq!(h_json["action_id"], action_id, "{h_json}");
        assert_eq!(h_json["kind"], "write", "{h_json}");
    }
}

// ============================================================================
// 12. Exhaustive <= 2 ms Legacy Lock-Hold Audit Across ALL WS Ordinary Owner Handlers
// ============================================================================

struct StubDetectionSource;

impl armd::grasp::DetectionSource for StubDetectionSource {
    fn inference_enabled(&self) -> armd::grasp::BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn set_inference(&self, _enabled: bool) -> armd::grasp::BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn next_frame(
        &self,
        after: Option<u64>,
        _wait_ms: u64,
    ) -> armd::grasp::BoxFuture<'_, Result<Option<armd::grasp::DetectionFrame>, String>> {
        Box::pin(async move {
            Ok(Some(armd::grasp::DetectionFrame {
                frame_id: after.unwrap_or(0) + 1,
                captured_at: Instant::now(),
                policy_captured_at: Some(Instant::now()),
                image_size_px: [640.0, 360.0],
                detections: Vec::new(),
                depth: None,
                model_generation: None,
            }))
        })
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_ws_ordinary_owner_handlers_lock_hold_under_2ms_with_real_planning() {
    Box::pin(run_legacy_ws_ordinary_owner_handlers_lock_hold_audit()).await;
}

async fn reset_test_arm_state(arm: &common::TestArm, sup: &Arc<ArmSupervisor>, q: [f32; 7]) {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        sup.stop_motion().await;
        {
            let mut s = arm.0.lock().unwrap();
            s.q = q;
            s.mit = false;
        }
        let idle = {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = q.to_vec();
            inner.measured_positions = q.to_vec();
            inner.target_positions = q.to_vec();
            inner.commanded_velocities.fill(0.0);
            inner.measured_velocities.fill(0.0);
            inner.pending_normal_unload = false;
            inner.park_release = None;
            inner.park_gripper_contact = false;
            inner.active_trajectory.is_none()
                && inner.macro_run.is_none()
                && inner.gripper_calibration.is_none()
                && !inner.gravity_comp_active()
                && !inner.last_action.as_ref().is_some_and(|a| a.is_running())
        };
        tokio::time::sleep(Duration::from_millis(45)).await;
        if idle || Instant::now() >= deadline {
            break;
        }
    }
}

async fn measure_ws_rpc(
    ws: &mut common::TestSocket,
    gen: u64,
    sup: &Arc<ArmSupervisor>,
    label: &'static str,
    path: &str,
    body: serde_json::Value,
    expect_ack: bool,
    measurements: &mut Vec<(&'static str, u32, String)>,
) -> serde_json::Value {
    let before = sup.tick_samples().len();
    let ep = sup.get_observation().await.epoch_id;
    let r = common::rpc(ws, gen, ep, path, body).await;
    if expect_ack {
        assert_eq!(r["type"], "command_ack", "{label}: {r}");
    }
    tokio::time::sleep(Duration::from_millis(45)).await;
    let samples = sup.tick_samples();
    let slice = &samples[before.min(samples.len())..];
    let mut max_hold = 0u32;
    let mut max_holder_desc = String::new();
    for s in slice {
        if s.writer_hold_us >= max_hold {
            max_hold = s.writer_hold_us;
            if let Some(h) = s.longest_hold.filter(|h| h.kind == "write").or(s.waited_for.filter(|h| h.kind == "write")) {
                max_holder_desc = format!(
                    "{}:{} cmd={:?} action={:?}",
                    h.location.file(),
                    h.location.line(),
                    h.command,
                    h.action_id
                );
            }
        }
    }
    measurements.push((label, max_hold, format!("{} ({})", r["type"], max_holder_desc)));
    r
}

async fn run_legacy_ws_ordinary_owner_handlers_lock_hold_audit() {
    use armd::api::create_router_calibration;
    use armd::api::proxy::DetectorUpstream;
    use armd::calibration::CalibrationDeps;
    use armd::diagnostics::DiagnosticsState;
    use armd::grasp::GraspDeps;
    use armd::recording::{write_frames_to_parquet, RecordFrame};
    use common::{TestArm, INSPECT};
    use tokio::net::TcpListener;

    let tmp = tempfile::tempdir().expect("tempdir");
    let keepout_path = tmp.path().join("keepout.json");
    let collision_path = tmp.path().join("collision.json");
    let gripper_calib_path = tmp.path().join("gripper_calib.json");
    let poses_path = tmp.path().join("poses.json");
    let sequences_path = tmp.path().join("sequences.json");
    let setup_path = tmp.path().join("grasp_setup.json");
    let commissioning_path = tmp.path().join("tool_commissioning.json");
    let calib_path = tmp.path().join("calibration.json");
    let episodes_dir = tmp.path().join("episodes");
    std::fs::create_dir_all(&episodes_dir).unwrap();

    // Seed a real 4 s 50 Hz episode parquet (201 frames) for /api/v1/action/replay & sequences.
    let ep_frames: Vec<RecordFrame> = (0..=200)
        .map(|k| {
            let t = k as f64 * 0.02;
            let s = (1.0 - (2.0 * std::f64::consts::PI * t / 4.0).cos()) as f32 / 2.0;
            let mut q = INSPECT.to_vec();
            q[0] += 0.25 * s;
            q[3] += 0.15 * s;
            RecordFrame { timestamp: 100.0 + t, observation_state: q.clone(), action: q }
        })
        .collect();
    write_frames_to_parquet(&episodes_dir.join("ep_wave.parquet"), &ep_frames).unwrap();

    // Real planning load: B601-DM 7-DOF profile + URDF kinematics + workspace + 2 extra keepout
    // boxes + power budget + resting pose policy + TestArm driver (torque/velocity/MIT caps).
    let mut cfg = dm_config_with_planning_load();
    if let Some(ws) = cfg.workspace.as_mut() {
        ws.keepout_boxes.push(KeepoutBox {
            name: "rear_box".into(),
            min_m: [-0.55, -0.45, 0.05],
            max_m: [-0.35, -0.25, 0.25],
        });
    }
    let arm = TestArm::new();
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(arm.clone())));
    sup.set_episodes_dir(Some(episodes_dir.clone())).await;
    {
        let inner = sup.inner().read().await;
        let _ = inner.episodes.get_or_adopt(
            "ep_wave",
            &inner.joint_zero_offsets,
            &inner.config.joint_names,
            inner.config.profile_id.as_deref(),
            50.0,
        ).expect("adopt ep_wave parquet");
    }
    armd::keepout::load_into(&sup, &keepout_path).await.unwrap();
    armd::collision::load_into(&sup, &collision_path).await;
    armd::gripper_calib::load_into(&sup, &gripper_calib_path).await;
    armd::poses::load_into(&sup, &poses_path).await;
    armd::sequences::load_into(&sup, &sequences_path).await;
    armd::grasp::load_into(&sup, &setup_path).await;
    armd::tool::load_commissioning_into(&sup, &commissioning_path).await;
    armd::calibration::load_into(&sup, &calib_path, None).await.unwrap();

    let up = DetectorUpstream::parse("http://127.0.0.1:9").unwrap();
    let grasp_deps = GraspDeps {
        source: Arc::new(StubDetectionSource),
        setup_path: Some(setup_path.clone()),
    };
    let mut calib_deps = CalibrationDeps::new(calib_path.clone(), Some(setup_path.clone()), up.clone());
    calib_deps.backup_dir = tmp.path().join("backups");
    let router = create_router_calibration(
        sup.clone(),
        up,
        grasp_deps,
        DiagnosticsState::global(),
        calib_deps,
        OperatorAuth::disabled(),
    );
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });

    // Concurrent 50 Hz legacy control tick loop.
    let ticker_stop = Arc::new(AtomicBool::new(false));
    let ticker_stop_bg = ticker_stop.clone();
    let ticker_sup = sup.clone();
    let ticker_jh = tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(20));
        while !ticker_stop_bg.load(Ordering::Acquire) {
            interval.tick().await;
            ticker_sup.tick(0.02).await;
        }
    });

    // Warm up Tokio blocking pool & first-call dispatch tables before measuring holds.
    let _ = tokio::task::spawn_blocking(|| {}).await;
    let (mut ws, mut gen) = common::controller(addr).await;
    let ep = sup.get_observation().await.epoch_id;
    let ack = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    reset_test_arm_state(&arm, &sup, INSPECT).await;

    // Seed an initial grasp_setup + saved pose + sequence so grasp/place/move_to_pose/measure_multiview/run_sequence
    // execute REAL planning rather than rejecting with MISSING_SETUP / POSE_NOT_FOUND.
    let ep = sup.get_observation().await.epoch_id;
    let setup_ack = common::rpc(&mut ws, gen, ep, "/api/v1/grasp/setup", json!({"table_z_m": 0.01})).await;
    assert_eq!(setup_ack["type"], "command_ack", "seed grasp_setup: {setup_ack}");
    let mut pose_q = INSPECT.to_vec();
    pose_q[0] += 0.18;
    pose_q[3] -= 0.12;
    let pose_ack = common::rpc(
        &mut ws,
        gen,
        sup.get_observation().await.epoch_id,
        "/api/v1/poses/save",
        json!({"name": "seed_pose", "positions": pose_q}),
    )
    .await;
    assert_eq!(pose_ack["type"], "command_ack", "seed pose: {pose_ack}");
    let seed_pose_id = pose_ack["pose"]["id"].as_str().unwrap().to_string();
    let seq_ack = common::rpc(
        &mut ws,
        gen,
        sup.get_observation().await.epoch_id,
        "/api/v1/sequences/save",
        json!({
            "name": "seed_seq",
            "steps": [
                {"type": "pose", "pose_id": seed_pose_id},
                {"type": "gripper", "action": "open"},
                {"type": "replay", "episode_id": "ep_wave", "speed": 1.0, "gripper": true}
            ]
        }),
    )
    .await;
    assert_eq!(seq_ack["type"], "command_ack", "seed sequence: {seq_ack}");
    let seed_seq_id = seq_ack["sequence"]["id"].as_str().unwrap().to_string();
    let tcp = sup.inner().read().await.config.tcp_pose_f32(&INSPECT).expect("FK TCP pose at INSPECT");
    let tcp_pos = tcp.p;
    let tcp_rpy = armd::kinematics::r_to_rpy(&tcp.r);

    let mut measurements: Vec<(&'static str, u32, String)> = Vec::new();
    tokio::time::sleep(Duration::from_millis(45)).await;

    // --- 1. Motion family (jog/move_joints, home, unload, presets, gravity_comp) ---
    let mut jog_t = INSPECT.to_vec();
    jog_t[0] += 0.22;
    jog_t[1] += 0.12;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/motion/jog", "/api/v1/motion/jog", json!({"target_positions": jog_t, "duration_secs": 0.6}), true, &mut measurements).await;
    reset_test_arm_state(&arm, &sup, INSPECT).await;

    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/motion/home", "/api/v1/motion/home", json!({"duration_secs": 0.6}), true, &mut measurements).await;
    reset_test_arm_state(&arm, &sup, INSPECT).await;

    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/motion/unload", "/api/v1/motion/unload", json!({}), true, &mut measurements).await;
    reset_test_arm_state(&arm, &sup, INSPECT).await;
    if !sup.get_observation().await.is_torqued {
        let ep = sup.get_observation().await.epoch_id;
        let _ = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        reset_test_arm_state(&arm, &sup, INSPECT).await;
    }

    for (label, name, pos_opt) in [
        ("preset:home", "home", None),
        ("preset:park", "park", None),
        ("preset:custom", "custom_pose", Some({ let mut q = INSPECT.to_vec(); q[0] -= 0.20; q })),
    ] {
        let before = sup.tick_samples().len();
        let ep = sup.get_observation().await.epoch_id;
        let r = common::preset(&mut ws, gen, ep, name, pos_opt).await;
        assert_eq!(r["type"], "preset_ack", "{label}: {r}");
        tokio::time::sleep(Duration::from_millis(45)).await;
        let samples = sup.tick_samples();
        let max_hold = samples[before.min(samples.len())..].iter().map(|s| s.writer_hold_us).max().unwrap_or(0);
        measurements.push((label, max_hold, r["type"].to_string()));
        reset_test_arm_state(&arm, &sup, INSPECT).await;
        if !sup.get_observation().await.is_torqued {
            let ep = sup.get_observation().await.epoch_id;
            let _ = common::rpc(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
            reset_test_arm_state(&arm, &sup, INSPECT).await;
        }
    }

    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/mode/gravity_comp(on)", "/api/v1/mode/gravity_comp", json!({"enable": true}), false, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/mode/gravity_comp(off)", "/api/v1/mode/gravity_comp", json!({"enable": false}), false, &mut measurements).await;
    reset_test_arm_state(&arm, &sup, INSPECT).await;

    // --- 2. Servo family (FrameKind::Servo, FrameKind::ImplicitServo, /api/v1/motion/servo) ---
    {
        let mut s_q = INSPECT;
        s_q[0] += 0.02;
        let before = sup.tick_samples().len();
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "servo", "generation": gen, "epoch_id": ep, "sequence_id": 100, "positions": s_q}),
        )
        .await;
        let rej = common::servo_reject_within(&mut ws, 50).await;
        assert!(rej.is_none(), "frame:servo rejected: {rej:?}");
        let samples = sup.tick_samples();
        let max_hold = samples[before.min(samples.len())..].iter().map(|s| s.writer_hold_us).max().unwrap_or(0);
        measurements.push(("frame:servo", max_hold, "accepted".into()));
        reset_test_arm_state(&arm, &sup, INSPECT).await;
    }
    {
        let mut s_q = INSPECT;
        s_q[0] += 0.015;
        let before = sup.tick_samples().len();
        let ep = sup.get_observation().await.epoch_id;
        common::send(
            &mut ws,
            json!({"type": "joint_positions", "generation": gen, "epoch_id": ep, "sequence_id": 101, "positions": s_q}),
        )
        .await;
        let rej = common::servo_reject_within(&mut ws, 50).await;
        assert!(rej.is_none(), "frame:implicit_servo rejected: {rej:?}");
        let samples = sup.tick_samples();
        let max_hold = samples[before.min(samples.len())..].iter().map(|s| s.writer_hold_us).max().unwrap_or(0);
        measurements.push(("frame:implicit_servo", max_hold, "accepted".into()));
        reset_test_arm_state(&arm, &sup, INSPECT).await;
    }
    {
        let mut s_q = INSPECT;
        s_q[0] += 0.015;
        measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/motion/servo", "/api/v1/motion/servo", json!({"targets": s_q}), true, &mut measurements).await;
        reset_test_arm_state(&arm, &sup, INSPECT).await;
    }

    // --- 3. Actions family (all 12 DispatchHandler::Owner entries with real planning) ---
    let action_cases: Vec<(&'static str, &'static str, serde_json::Value, bool)> = vec![
        ("/api/v1/action/move_to(cartesian)", "/api/v1/action/move_to", json!({"position": [tcp_pos[0], tcp_pos[1] + 0.03, tcp_pos[2] + 0.02], "rpy": tcp_rpy, "speed": {"linear": 0.10, "angular": 0.5}}), true),
        ("/api/v1/action/move_to(joint)", "/api/v1/action/move_to", json!({"position": [tcp_pos[0], tcp_pos[1] - 0.03, tcp_pos[2] + 0.02], "rpy": tcp_rpy, "path": "joint", "speed_scale": 0.5}), true),
        ("/api/v1/action/move_relative", "/api/v1/action/move_relative", json!({"frame": "base", "d": [0.0, 0.02, 0.02], "speed": {"linear": 0.10, "angular": 0.5}}), true),
        ("/api/v1/action/move_to_pose", "/api/v1/action/move_to_pose", json!({"name": "seed_pose", "speed_scale": 0.5}), true),
        ("/api/v1/action/gripper", "/api/v1/action/gripper", json!({"command": "open", "speed": 1.0}), true),
        ("/api/v1/action/gripper_zero_check", "/api/v1/action/gripper_zero_check", json!({"store_offset": false}), true),
        ("/api/v1/action/grasp(dry_run)", "/api/v1/action/grasp", json!({"dry_run": true, "label": "cup", "box_norm": [0.45, 0.70, 0.55, 0.80], "overrides": {"table_z_m": 0.10}}), true),
        ("/api/v1/action/grasp", "/api/v1/action/grasp", json!({"label": "cup", "box_norm": [0.45, 0.70, 0.55, 0.80], "overrides": {"table_z_m": 0.10}}), true),
        ("/api/v1/action/place", "/api/v1/action/place", json!({"position": [0.30, 0.0, 0.05]}), true),
        ("/api/v1/action/replay", "/api/v1/action/replay", json!({"episode": "ep_wave", "speed": 1.0, "gripper": true}), true),
        ("/api/v1/action/run_sequence", "/api/v1/action/run_sequence", json!({"id": seed_seq_id}), true),
        (armd::calib_hover::PATH, armd::calib_hover::PATH, json!({"candidate_fingerprint": "cafe0123", "points": [{"id": "p0", "approach_m": [0.30, 0.0, 0.08], "hover_m": [0.30, 0.0, 0.06]}], "rpy_rad": [std::f64::consts::PI, 0.0, 0.0], "approach_speed_mps": 0.05, "descent_speed_mps": 0.015, "settle_s": 2.5}), true),
        (armd::gripper_calib::CALIBRATE_PATH, armd::gripper_calib::CALIBRATE_PATH, json!({}), true),
        (armd::measure_multiview::PATH, armd::measure_multiview::PATH, json!({"motion": "orbit", "axes": "x", "views": 3, "baseline_m": 0.08}), true),
    ];
    for (label, path, body, expect_ack) in action_cases {
        let pq_before = sup.inner().read().await.prepared_quintics_used;
        let samples_before = sup.tick_samples().len();
        measure_ws_rpc(&mut ws, gen, &sup, label, path, body, expect_ack, &mut measurements).await;
        if label == "/api/v1/action/run_sequence" {
            let wait_deadline = Instant::now() + Duration::from_millis(500);
            while sup.inner().read().await.prepared_quintics_used == pq_before
                && Instant::now() < wait_deadline
            {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            assert!(
                sup.inner().read().await.prepared_quintics_used > pq_before,
                "run_sequence pose child must install via off-lock PreparedQuintic (zero in-lock planning fallback)"
            );
            tokio::time::sleep(Duration::from_millis(25)).await;
            let samples = sup.tick_samples();
            let post_max = samples[samples_before.min(samples.len())..]
                .iter()
                .map(|s| s.writer_hold_us)
                .max()
                .unwrap_or(0);
            if let Some(last) = measurements.last_mut() {
                last.1 = last.1.max(post_max);
            }
        }
        reset_test_arm_state(&arm, &sup, INSPECT).await;
    }

    // --- 4. Config family (all 10 DispatchHandler::Owner entries with real persistence + EvalBufs) ---
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/workspace/keepout", "/api/v1/workspace/keepout", json!({"boxes": [{"name": "dyn_box", "min_m": [0.45, -0.45, 0.05], "max_m": [0.55, -0.35, 0.20]}]}), true, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/safety/collision", "/api/v1/safety/collision", json!({"enabled": true, "sensitivity": "medium"}), true, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/gripper/calibration", "/api/v1/gripper/calibration", json!({"jaw_opening": {"points": [[0.0, 0.0], [-5.7, 0.08]], "calibrated": true}, "offset_rad": 0.02}), true, &mut measurements).await;
    let saved_pose = measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/poses/save", "/api/v1/poses/save", json!({"name": "tmp_pose", "positions": INSPECT}), true, &mut measurements).await;
    let tmp_pose_id = saved_pose["pose"]["id"].as_str().unwrap().to_string();
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/poses/rename", "/api/v1/poses/rename", json!({"id": tmp_pose_id, "name": "tmp_pose_renamed"}), true, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/poses/delete", "/api/v1/poses/delete", json!({"name": "tmp_pose_renamed"}), true, &mut measurements).await;
    let saved_seq = measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/sequences/save", "/api/v1/sequences/save", json!({"name": "tmp_seq", "steps": [{"type": "wait", "s": 0.1}]}), true, &mut measurements).await;
    let tmp_seq_id = saved_seq["sequence"]["id"].as_str().unwrap().to_string();
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/sequences/delete", "/api/v1/sequences/delete", json!({"id": tmp_seq_id}), true, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/grasp/setup", "/api/v1/grasp/setup", json!({"table_z_m": 0.015}), true, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/tool/commissioning/confirm", "/api/v1/tool/commissioning/confirm", json!({"operator_confirmed": true}), false, &mut measurements).await;

    // --- 5. Policy family (when policy-runtime feature is enabled) ---
    #[cfg(feature = "policy-runtime")]
    {
        measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/action/policy/start", "/api/v1/action/policy/start", json!({"policy_id": "probe_missing_policy", "mode": "shadow"}), false, &mut measurements).await;
        measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/action/policy/stop", "/api/v1/action/policy/stop", json!({}), true, &mut measurements).await;
    }

    // --- 6. Torque off + Calibration / MotorZero / MIT family (all 6 DispatchHandler::Owner entries) ---
    arm.set_residual(2, 7.62);
    reset_test_arm_state(&arm, &sup, [0.0; 7]).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/torque(off)", "/api/v1/torque", json!({"enable": false}), true, &mut measurements).await;
    arm.clear();
    let calib_cases: Vec<(&'static str, &'static str, serde_json::Value)> = vec![
        ("/api/v1/calibration/apply", "/api/v1/calibration/apply", json!({
            "intrinsics": {"fx": 831.5, "fy": 829.0, "cx": 318.2, "cy": 183.4, "k1": 0.05, "k2": -0.1, "p1": 0.0, "p2": 0.0, "k3": 0.0, "width": 640, "height": 360, "marker_size": 0.0495},
            "camera_mount": {"offset_m": [-0.0728, 0.0012, 0.0655], "q_end_cam": [0.0, 0.0, 0.0, 1.0]},
            "table_z_m": 0.012,
            "rms": {"intrinsics_px": 0.21, "hand_eye_mm": 1.4, "hand_eye_deg": 0.3},
            "dry_run": true
        })),
        ("/api/v1/calibration/restore", "/api/v1/calibration/restore", json!({"backup": "none"})),
        (armd::motor_zero::PATH, armd::motor_zero::PATH, json!({"joints": [1, 2, 3], "dry_run": true})),
        (armd::motor_zero::RESTORE_PATH, armd::motor_zero::RESTORE_PATH, json!({"backup": "2026-01-01T00-00-00Z"})),
        (armd::mit::TIMEOUT_PATH, armd::mit::TIMEOUT_PATH, json!({})),
        (armd::mit::QUALIFICATION_PATH, armd::mit::QUALIFICATION_PATH, json!({"joints": [1]})),
    ];
    for (label, path, body) in calib_cases {
        measure_ws_rpc(&mut ws, gen, &sup, label, path, body, false, &mut measurements).await;
    }

    // --- 7. SessionAsync & CommandAsync ordinary worker handlers ---
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/arm", "/api/v1/arm", json!({"enable": true}), true, &mut measurements).await;
    measure_ws_rpc(&mut ws, gen, &sup, "/api/v1/safety/clear_fault", "/api/v1/safety/clear_fault", json!({}), false, &mut measurements).await;
    {
        let before = sup.tick_samples().len();
        let r = common::ws_reply(&mut ws, json!({"type": "release_lease", "generation": gen}), &["lease_released"]).await;
        assert_eq!(r["success"], json!(true), "release_lease: {r}");
        tokio::time::sleep(Duration::from_millis(45)).await;
        let samples = sup.tick_samples();
        let max_hold = samples[before.min(samples.len())..].iter().map(|s| s.writer_hold_us).max().unwrap_or(0);
        measurements.push(("release_lease", max_hold, "lease_released".into()));
    }
    {
        let before = sup.tick_samples().len();
        let r = common::ws_reply(&mut ws, json!({"type": "acquire_lease", "client_id": "reacquire_probe"}), &["lease_granted"]).await;
        gen = r["generation"].as_u64().unwrap();
        let _ = gen;
        tokio::time::sleep(Duration::from_millis(45)).await;
        let samples = sup.tick_samples();
        let max_hold = samples[before.min(samples.len())..].iter().map(|s| s.writer_hold_us).max().unwrap_or(0);
        measurements.push(("acquire_lease", max_hold, "lease_granted".into()));
    }

    ticker_stop.store(true, Ordering::Release);
    let _ = ticker_jh.await;

    let mut over_2ms = Vec::new();
    for (label, hold_us, tag) in &measurements {
        eprintln!("[WS_LOCK_HOLD] {label:<36} writer_hold_us={hold_us:>6} reply={tag}");
        if *hold_us > 2_000 {
            over_2ms.push((*label, *hold_us));
        }
    }
    assert!(
        over_2ms.is_empty(),
        "WS ordinary owner handlers exceeded 2 ms write-lock hold budget: {over_2ms:?}\nAll measurements: {measurements:?}"
    );
}
