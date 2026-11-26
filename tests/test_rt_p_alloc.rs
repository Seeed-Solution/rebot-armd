//! Stage 3 PR P B2 zero-allocation proof (`TimedAlloc`):
//! Proves strictly ZERO heap operations (`alloc`, `alloc_zeroed`, `realloc`, `dealloc`) on
//! `armd-control` across EVERY new PR P request family:
//! - A6 `InstallPrepared` (`Cartesian`, `Joint`, `Gripper`, `Replay`) commit, tick, completion,
//!   and retirement;
//! - A7 `BeginMacro` -> `InstallChild` (`Cartesian`, `Joint`, `Gripper`) -> `FinishMacro(Done, HeldUpdate::Set)`;
//! - Refused prepared actions / children / delayed finishes;
//! - Forced `FinishMacro(Failed)` during active gripper motion (measured-position hold + deferred
//!   bus acceptance `MacroFinish`);
//! - Owner-enforced `MacroRun::deadline` expiry.

mod common;

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use serde_json::json;

use armd::diagnostics::alloc::{AllocCounts, TimedAlloc};
use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueOutcome,
    TorqueProgress,
};
use armd::macro_action::{ChildSpec, ChildTicket, SubState};
use armd::model::{HolderLabel, JointState};
use armd::motion::{GoalFrame, GoalSpec, GripperCommand, Speed};
use armd::rt_core::broker::{Authority, BootId, ClientId, Outcome, Reply, Submit};
use armd::rt_core::executor::{
    ExecutorClock, ExecutorConfig, PersistTestHooks, RtExecutorHandle,
};
use armd::rt_core::mailbox::MAX_DOF;
use armd::rt_core::ops::{EpisodeRef, PlanSlot, Primitive};
use armd::rt_core::requests::actions::{
    prepare_child_cartesian_slot, prepare_child_cartesian_slot_with_options,
    prepare_child_gripper_slot, prepare_child_joint_slot, prepare_gripper, prepare_move_to,
    ActionSnapshot, ActionSubmit, BeginMacroSpec, FinishMacroSpec, HeldUpdate, MacroKind,
    PreparedActionKind, StagedAction, DEFAULT_ACTION_DEADLINE, DEFAULT_FINISH_DEADLINE,
};
use armd::supervisor::GripperBaselineCmd;
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use common::INSPECT;

#[global_allocator]
static ALLOC: TimedAlloc = TimedAlloc;

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
            state: Mutex::new(ClockState {
                now_ns: 0,
                limit_ns: 0,
                waiting_for_ns: None,
            }),
            cv: Condvar::new(),
        })
    }

    fn run_steps(&self, period_ns: u64, steps: u64) {
        let mut s = self.state.lock().unwrap();
        let target = s.limit_ns + period_ns * steps;
        s.limit_ns = target;
        let expect_park = target + period_ns;
        self.cv.notify_all();
        let deadline = Instant::now() + Duration::from_secs(5);
        while s.waiting_for_ns != Some(expect_park) {
            let rem = deadline.saturating_duration_since(Instant::now());
            assert!(!rem.is_zero(), "timed out waiting for scripted clock park");
            let (next, _) = self.cv.wait_timeout(s, rem).unwrap();
            s = next;
        }
    }
}

impl ExecutorClock for ScriptedClock {
    fn now(&self) -> Instant {
        self.base + Duration::from_nanos(self.state.lock().unwrap().now_ns)
    }

    fn now_ns(&self) -> u64 {
        self.state.lock().unwrap().now_ns
    }

    fn sleep_until_ns(&self, target_ns: u64, stop: &AtomicBool) -> bool {
        let mut s = self.state.lock().unwrap();
        if s.now_ns >= target_ns {
            self.cv.notify_all();
            return false;
        }
        while !stop.load(Ordering::Acquire) && s.now_ns < target_ns && s.limit_ns < target_ns {
            s.waiting_for_ns = Some(target_ns);
            self.cv.notify_all();
            s = self.cv.wait(s).unwrap();
        }
        s.waiting_for_ns = None;
        if !stop.load(Ordering::Acquire) && s.now_ns < target_ns {
            s.now_ns = target_ns;
        }
        self.cv.notify_all();
        true
    }

    fn sleep(&self, dur: Duration, stop: &AtomicBool) {
        if stop.load(Ordering::Acquire) {
            return;
        }
        let mut s = self.state.lock().unwrap();
        s.now_ns = s.now_ns.saturating_add(dur.as_nanos().min(u128::from(u64::MAX)) as u64);
        self.cv.notify_all();
    }

    fn wake(&self) {
        let _g = self.state.lock().unwrap();
        self.cv.notify_all();
    }
}

struct ZeroAllocDriver {
    dof: usize,
    q: Vec<f32>,
    enabled: bool,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
    clock: Arc<ScriptedClock>,
    reject_writes: Arc<AtomicBool>,
    has_torque_feedback: bool,
    pos_force_close_speed: Option<f32>,
    pf_target_count: Arc<AtomicUsize>,
    pf_hold_count: Arc<AtomicUsize>,
}

impl ZeroAllocDriver {
    fn new(init_q: &[f32], clock: Arc<ScriptedClock>) -> Self {
        let dof = init_q.len();
        let now = clock.now();
        Self {
            dof,
            q: init_q.to_vec(),
            enabled: true,
            rx_at: vec![Some(now); dof],
            seqs: vec![1; dof],
            clock,
            reject_writes: Arc::new(AtomicBool::new(false)),
            has_torque_feedback: true,
            pos_force_close_speed: None,
            pf_target_count: Arc::new(AtomicUsize::new(0)),
            pf_hold_count: Arc::new(AtomicUsize::new(0)),
        }
    }
}

impl ArmDriver for ZeroAllocDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: self.has_torque_feedback,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn clock_now(&self) -> Instant { self.clock.now() }
    fn enable_state(&self) -> EnableState {
        if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let now = self.clock.now();
        self.rx_at.iter().map(|t| t.map(|t| now.saturating_duration_since(t))).collect()
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
    fn feedback_seqs(&self) -> Vec<u64> { self.seqs.clone() }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> { self.rx_at.clone() }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let now = self.clock.now();
        out.clear();
        for t in &self.rx_at {
            out.push(t.map(|t| now.saturating_duration_since(t)));
        }
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let now = self.clock.now();
        self.rx_at.len() == dof
            && self.rx_at.iter().all(|t| t.is_some_and(|t| now.saturating_duration_since(t) <= max_age))
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let now = self.clock.now();
        out.clear();
        for t in &self.rx_at {
            out.push(t.map(|t| now.saturating_duration_since(t).as_millis() as u64));
        }
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
        let mut out = Vec::with_capacity(self.dof);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = self.clock.now();
        out.clear();
        for i in 0..self.dof {
            out.push(JointState { position: self.q[i], velocity: 0.0, torque: 0.0, temperature: 30.0 });
            self.rx_at[i] = Some(now);
            self.seqs[i] += 1;
        }
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _dl: Instant, _max: u16) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((self.dof as u16, false))
    }
    fn gripper_pos_force_info(&self) -> Option<armd::driver::GripperPosForceInfo> {
        self.pos_force_close_speed.map(|s| armd::driver::GripperPosForceInfo {
            joint: 6,
            active: true,
            close_speed_rad_s: s,
            ceiling_torque_estimate_nm: Some(1.5),
        })
    }
    fn pos_force_feedback_fresh(&self) -> bool { true }
    fn pos_force_last_fresh_position(&self) -> Option<f32> { self.q.get(6).copied() }
    fn emit_pos_force_j7(
        &mut self,
        intent: armd::driver::PosForceIntent,
        current: armd::driver::PosForceIntentStamp,
    ) -> Result<armd::driver::PosForceEmitOutcome, ArmDriverError> {
        if self.pos_force_close_speed.is_none() {
            return Ok(armd::driver::PosForceEmitOutcome::NotApplicable);
        }
        let stale = intent.stamp.is_stale_against(current);
        match intent.kind {
            armd::driver::PosForceIntentKind::Target { position, .. } if !stale => {
                self.pf_target_count.fetch_add(1, Ordering::Relaxed);
                if self.q.len() > 6 {
                    self.q[6] = position;
                }
            }
            _ => {
                self.pf_hold_count.fetch_add(1, Ordering::Relaxed);
            }
        }
        Ok(armd::driver::PosForceEmitOutcome::Sent {
            joint: 6,
            freeze: false,
            pf_status: armd::driver::PosForceTxStatus::Fresh,
        })
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        for (i, t) in targets.iter().enumerate().take(self.dof) {
            self.q[i] = t.position;
        }
        Ok(())
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
        if self.reject_writes.load(Ordering::Acquire) {
            return Ok(armd::driver::TxBatchOutcome::Deferred { pending: 1 });
        }
        self.write_joint_targets(targets)?;
        Ok(armd::driver::TxBatchOutcome::Complete)
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }
    fn begin_torque_budgeted(&mut self, enable: bool, _max: u16) -> Result<TorqueProgress, ArmDriverError> {
        self.enabled = enable;
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn poll_torque_budgeted(&mut self, _dl: Instant, _max: u16) -> Result<TorqueProgress, ArmDriverError> {
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn abort_enable(&mut self) {}
    fn torque_transition_pending(&self) -> bool { false }
}

fn drain(handle: &RtExecutorHandle) -> Vec<Reply> {
    let mut out = Vec::new();
    let mut b = handle.broker().lock().unwrap();
    b.poll(|_, r| {
        out.push(r);
        true
    });
    b.reclaim_plans();
    out
}

fn latest_snap(sup: &ArmSupervisor, dl: Instant) -> ActionSnapshot {
    let s = sup.hub().latest().expect("published snapshot");
    let env = sup.hub().prep_env().expect("published prep_env");
    ActionSnapshot::from_published(&s, env, sup.capture_admission_stamp(), dl).unwrap()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b2_pr_p_prepared_actions_and_macro_lifecycle_are_zero_alloc_on_rt() {
    let clock = ScriptedClock::new();
    let drv = ZeroAllocDriver::new(&INSPECT, clock.clone());
    let reject_writes = drv.reject_writes.clone();
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv)),
    );
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        // Major 7: seed a pre-existing held object BEFORE handover to prove clearing & re-setting
        // a held object on RT performs strictly 0 allocs and 0 frees!
        inner.held = Some(armd::supervisor::HeldObject {
            label: "pre_seeded_object".into(),
            spill_safe: true,
        });
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
    }

    let period_ns = 20_000_000u64;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
    cfg.alloc_timing = true;
    let handle = sup
        .start_executor_with_hooks(
            cfg,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    let boot = handle.boot();
    clock.run_steps(period_ns, 3);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    let submit_prep = |seq: u64, kind: PreparedActionKind, slot: PlanSlot, auth: Authority, check_ws: bool| {
        let dl = clock.now() + DEFAULT_ACTION_DEADLINE;
        let write = |dst: &mut PlanSlot| *dst = slot.clone();
        handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(1),
                seq,
                auth,
                dl,
                Submit::Action(ActionSubmit::InstallPrepared {
                    write: &write,
                    kind,
                    prepared_boot: boot,
                    admission_request_id: 0,
                    check_workspace_revision: check_ws,
                    poses_revision: None,
                    calibration_revision: None,
                    gripper_calib_revision: None,
                    power_budget_revision: None,
                }),
            )
            .unwrap()
    };

    // 1) A6 Prepared Cartesian move
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let p0 = armd::motion::start_pose(&snap.plan_input_in(GoalFrame::Tcp).unwrap()).p;
    let prep = prepare_move_to(&snap, &json!({"position": [p0[0] + 0.02, p0[1], p0[2]]})).unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let r1 = submit_prep(1, PreparedActionKind::MoveToLinear, slot, snap.owner.authority, true);
    clock.run_steps(period_ns, 80);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r1)), "{reps:?}");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Cartesian prepared action must be 0-alloc");

    // 2) A6 Prepared Joint move
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let p0 = armd::motion::start_pose(&snap.plan_input_in(GoalFrame::Tcp).unwrap()).p;
    let prep = prepare_move_to(&snap, &json!({"position": [p0[0] - 0.015, p0[1], p0[2]], "path": "joint", "speed_scale": 0.8})).unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let r2 = submit_prep(2, PreparedActionKind::MoveToJoint, slot, snap.owner.authority, true);
    clock.run_steps(period_ns, 80);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r2)), "{reps:?}");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Joint prepared action must be 0-alloc");

    // 3) A6 Prepared Gripper move
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let prep = prepare_gripper(&snap, &json!({"command": "move", "fraction": 0.5, "speed": 1.0}), GripperBaselineCmd::None).unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let r3 = submit_prep(3, PreparedActionKind::Gripper, slot, snap.owner.authority, false);
    clock.run_steps(period_ns, 250);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r3)), "{reps:?}");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Gripper prepared action must be 0-alloc");

    // 4) A6 Prepared Replay
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let mut replay_slot = PlanSlot::default();
    for k in 0..=20 {
        let mut q = snap.commanded_positions.clone();
        q[0] += 0.02 * (k as f32 / 20.0);
        replay_slot.path.push(&q, k as f64).unwrap();
    }
    let mut s_arr = [0.0f32; MAX_DOF];
    s_arr[..snap.commanded_positions.len()].copy_from_slice(&snap.commanded_positions);
    replay_slot.replay_approach = Some(
        armd::replay::PreparedReplayApproach::from_endpoints(
            &snap.commanded_positions,
            &snap.commanded_positions,
            0.2,
            None,
            true,
        )
        .unwrap(),
    );
    replay_slot.primitive = Primitive::Action(StagedAction::Replay {
        child: None,
        dof: snap.commanded_positions.len() as u8,
        start: s_arr,
        episode: EpisodeRef::new("ep_b2").unwrap(),
        speed: 1.0,
        gripper: false,
        sample_dt: 0.02,
        duration_s: 0.4,
        power_scale: None,
        approach_duration_s: 0.2,
        approach_power_scale: None,
        tool_digest: snap.env.tool_digest_hash,
    });
    let r4 = submit_prep(4, PreparedActionKind::Replay, replay_slot, snap.owner.authority, true);
    clock.run_steps(period_ns, 80);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r4)), "{reps:?}");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Replay prepared action must be 0-alloc");

    // 5) Full A7 Macro Lifecycle: BeginMacro -> Cartesian child -> Joint child -> Gripper child -> FinishMacro(Done, Set held)
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let begin = BeginMacroSpec::new(MacroKind::Grasp, boot);
    let r_b = handle.broker().lock().unwrap()
        .submit(ClientId(1), 5, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE, Submit::Action(ActionSubmit::BeginMacro(begin)))
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    let parent = reps.iter().find_map(|r| match r {
        Reply::Ack { request, action: Some(a), .. } if *request == r_b => Some(*a),
        _ => None,
    }).expect("BeginMacro ACK");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "BeginMacro must be 0-alloc");

    // Child 1: Cartesian
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let input = snap.plan_input_in(GoalFrame::Tcp).unwrap();
    let p0 = armd::motion::start_pose(&input).p;
    let path = armd::motion::plan_cartesian(&input, armd::motion::resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.015, p0[1], p0[2]], r: None }), Speed::default()).unwrap();
    let c1 = ChildSpec { ticket: ChildTicket { parent, child_seq: 1 }, timeout_s: 10.0, timeout_code: "GRASP_TIMEOUT", contact: None };
    let c1_slot = prepare_child_cartesian_slot(&snap.commanded_positions, &path, None, c1).unwrap();
    {
        let write = |dst: &mut PlanSlot| *dst = c1_slot.clone();
        handle.broker().lock().unwrap().submit(ClientId(1), 6, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: true, calibration_revision: None, grasp_setup_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
    }
    clock.run_steps(period_ns, 60);
    let _ = drain(&handle);
    assert!(matches!(sup.hub().latest().unwrap().rt.owner.macro_state.child_sub, SubState::Done(_)));
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Cartesian child must be 0-alloc");

    // Child 2: Joint
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] -= 0.02;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c2 = ChildSpec { ticket: ChildTicket { parent, child_seq: 2 }, timeout_s: 10.0, timeout_code: "GRASP_TIMEOUT", contact: None };
    let c2_slot = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.2, None, c2).unwrap();
    {
        let write = |dst: &mut PlanSlot| *dst = c2_slot.clone();
        handle.broker().lock().unwrap().submit(ClientId(1), 7, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: true, calibration_revision: None, grasp_setup_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
    }
    clock.run_steps(period_ns, 60);
    let _ = drain(&handle);
    assert!(matches!(sup.hub().latest().unwrap().rt.owner.macro_state.child_sub, SubState::Done(_)));
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Joint child must be 0-alloc");

    // Child 3: Gripper
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let c3 = ChildSpec { ticket: ChildTicket { parent, child_seq: 3 }, timeout_s: 10.0, timeout_code: "GRASP_TIMEOUT", contact: None };
    let c3_slot = prepare_child_gripper_slot(&snap.commanded_positions, GripperCommand::Open, None, false, 1.0, Some(8.0), None, false, c3).unwrap();
    {
        let write = |dst: &mut PlanSlot| *dst = c3_slot.clone();
        handle.broker().lock().unwrap().submit(ClientId(1), 8, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: false, calibration_revision: None, grasp_setup_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
    }
    clock.run_steps(period_ns, 250);
    let _ = drain(&handle);
    assert!(matches!(sup.hub().latest().unwrap().rt.owner.macro_state.child_sub, SubState::Done(_)));
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "Gripper child must be 0-alloc");

    // FinishMacro(Done, HeldUpdate::Set)
    let snap = latest_snap(&sup, clock.now() + DEFAULT_FINISH_DEADLINE);
    let mut fin = FinishMacroSpec::done(parent, boot);
    fin.holding = Some(true);
    fin.held = HeldUpdate::Set { label: HolderLabel::new("mug"), spill_safe: true };
    handle.broker().lock().unwrap().submit(ClientId(1), 9, snap.owner.authority, clock.now() + DEFAULT_FINISH_DEADLINE,
        Submit::Action(ActionSubmit::FinishMacro(fin))).unwrap();
    clock.run_steps(period_ns, 3);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, action, outcome: Outcome::Completed, children: 3 } if *request == r_b && *action == parent)), "{reps:?}");
    assert_eq!(
        sup.hub().latest().unwrap().rt.owner.held.map(|h| (h.label, h.spill_safe)),
        Some((HolderLabel::new("mug"), true))
    );
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "FinishMacro(Done) must be 0-alloc");

    // 6) Forced FinishMacro(Failed) during active gripper under REJECTED bus writes + duplicate
    //    finish -> PARENT_REPLACED + later acceptance (Blocker 2 + Major 7 pre-seeded held clear)!
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let mut begin2 = BeginMacroSpec::new(MacroKind::Place, boot);
    begin2.macro_deadline = Some(clock.now() + Duration::from_secs(10));
    begin2.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
    let r_b2 = handle.broker().lock().unwrap()
        .submit(ClientId(1), 10, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE, Submit::Action(ActionSubmit::BeginMacro(begin2)))
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    let parent2 = reps.iter().find_map(|r| match r {
        Reply::Ack { request, action: Some(a), .. } if *request == r_b2 => Some(*a),
        _ => None,
    }).unwrap();
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let cg = ChildSpec { ticket: ChildTicket { parent: parent2, child_seq: 1 }, timeout_s: 10.0, timeout_code: "PLACE_TIMEOUT", contact: None };
    let cg_slot = prepare_child_gripper_slot(&snap.commanded_positions, GripperCommand::Move, Some(0.1), false, 0.4, Some(8.0), None, false, cg).unwrap();
    {
        let write = |dst: &mut PlanSlot| *dst = cg_slot.clone();
        handle.broker().lock().unwrap().submit(ClientId(1), 11, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: false, calibration_revision: None, grasp_setup_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
    }
    clock.run_steps(period_ns, 3);
    // Reject bus writes for 3 ticks while forced FinishMacro + duplicate FinishMacro arrive!
    reject_writes.store(true, Ordering::Release);
    let snap = latest_snap(&sup, clock.now() + DEFAULT_FINISH_DEADLINE);
    let mut fail = FinishMacroSpec::failed(parent2, boot, "TARGET_LOST", "forced abort");
    fail.held = HeldUpdate::Clear;
    handle.broker().lock().unwrap().submit(ClientId(1), 12, snap.owner.authority, clock.now() + DEFAULT_FINISH_DEADLINE,
        Submit::Action(ActionSubmit::FinishMacro(fail))).unwrap();
    clock.run_steps(period_ns, 2);
    // Duplicate forced finish while bus still rejecting -> PARENT_REPLACED with 0 alloc!
    let mut dup_auth = snap.owner.authority;
    dup_auth.parent_generation = parent2.0;
    handle.broker().lock().unwrap().submit(ClientId(1), 13, dup_auth, clock.now() + DEFAULT_FINISH_DEADLINE,
        Submit::Action(ActionSubmit::FinishMacro(fail))).unwrap();
    clock.run_steps(period_ns, 2);
    let reps_rej = drain(&handle);
    assert!(reps_rej.iter().any(|r| matches!(r, Reply::Refused { code: "PARENT_REPLACED", .. })), "{reps_rej:?}");
    assert!(!reps_rej.iter().any(|r| matches!(r, Reply::Terminal { .. })), "must not emit terminal while bus rejects: {reps_rej:?}");
    // Allow bus acceptance -> terminal commits with 0 alloc and clears held with 0 free!
    reject_writes.store(false, Ordering::Release);
    clock.run_steps(period_ns, 3);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, action, outcome: Outcome::Failed("TARGET_LOST"), .. } if *request == r_b2 && *action == parent2)), "{reps:?}");
    assert!(sup.hub().latest().unwrap().rt.owner.held.is_none(), "forced FinishMacro(HeldUpdate::Clear) must clear held with 0 dealloc");

    // 7) Refused prepared action (wrong boot) -> 0 alloc!
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let bad_slot = PlanSlot::default();
    let write = |dst: &mut PlanSlot| *dst = bad_slot.clone();
    handle.broker().lock().unwrap().submit(ClientId(1), 14, snap.owner.authority, clock.now() + DEFAULT_ACTION_DEADLINE,
        Submit::Action(ActionSubmit::InstallPrepared {
            write: &write,
            kind: PreparedActionKind::Gripper,
            prepared_boot: BootId(999),
            admission_request_id: 0,
            check_workspace_revision: false,
            poses_revision: None,
            calibration_revision: None,
            gripper_calib_revision: None,
            power_budget_revision: None,
        })).unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Refused { code: "WRONG_TARGET", .. })), "{reps:?}");

    // 8) Major 15: qualifying gripper open + MaintenanceOp::TakeBaselinePending replenishment -> 0 alloc!
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let prep_open = prepare_gripper(&snap, &json!({"command": "open", "speed": 1.0}), GripperBaselineCmd::None).unwrap();
    let open_slot = prep_open.to_plan_slot(None).unwrap();
    let r_open = submit_prep(15, PreparedActionKind::Gripper, open_slot, snap.owner.authority, false);
    clock.run_steps(period_ns, 350);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_open)), "{reps:?}");
    let maint_handle = handle.clone();
    let maint_task = tokio::task::spawn_blocking(move || {
        maint_handle.call_maintenance(armd::rt_core::executor::MaintenanceOp::TakeBaselinePending {
            fresh_pending: Vec::with_capacity(8),
            fresh_samples: vec![armd::gripper_baseline::MoveSamples::default(); 2],
            clear_dirty: false,
        })
    });
    // Drive the scripted clock on the main test task while the maintenance op drains on RT!
    while !maint_task.is_finished() {
        clock.run_steps(period_ns, 1);
        tokio::task::yield_now().await;
    }
    let maint_res = maint_task.await.unwrap().expect("TakeBaselinePending must succeed");
    match maint_res {
        armd::rt_core::executor::MaintenanceResult::BaselineTaken(reply) => {
            assert!(
                !reply.moves.is_empty(),
                "Major 15: qualifying RT gripper opening must yield baseline move records via TakeBaselinePending"
            );
        }
        _ => panic!("expected BaselineTaken reply"),
    }

    // Final hard B2 invariant: 0 allocs across ALL releases and sub-release wakes!
    assert_eq!(
        handle.stats().release_allocs.snapshot(),
        AllocCounts::ZERO,
        "PR P release allocs must be strictly zero"
    );
    assert_eq!(
        handle.stats().wake_allocs.snapshot(),
        AllocCounts::ZERO,
        "PR P wake allocs must be strictly zero"
    );

    handle.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b2_pr_p_gripper_refusals_missing_torque_feedback_and_zero_quantized_speed_are_zero_alloc() {
    // Major 6: prove strictly 0 allocs/frees on RT when:
    // (a) driver lacks torque_feedback while max_close_torque_nm is configured -> UNSUPPORTED
    // (b) pos_force close speed quantizes to 0 -> INVALID_COMMAND
    let clock = ScriptedClock::new();
    let mut drv = ZeroAllocDriver::new(&INSPECT, clock.clone());
    drv.has_torque_feedback = false;
    let mut cfg = ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    if let Some(gp) = cfg.gripper_policy.as_mut() {
        gp.max_close_torque_nm = Some(2.0);
    }
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
    }
    let period_ns = 20_000_000u64;
    let mut ec = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
    ec.alloc_timing = true;
    let handle = sup
        .start_executor_with_hooks(
            ec,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    let boot = handle.boot();
    clock.run_steps(period_ns, 2);

    // (a) Standalone RT gripper close with missing torque feedback -> UNSUPPORTED, 0 alloc!
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let mut s_arr = [0.0f32; MAX_DOF];
    s_arr[..snap.commanded_positions.len()].copy_from_slice(&snap.commanded_positions);
    let mut slot = PlanSlot::default();
    slot.set(Primitive::Action(StagedAction::Gripper {
        child: None,
        dof: snap.commanded_positions.len() as u8,
        start: s_arr,
        command: armd::rt_core::requests::actions::GripperCommandKind::Close,
        fraction: None,
        grasp: true,
        speed: 0.5,
        timeout_s: Some(4.0),
        max_budget_s: None,
        extend_pos_force_close_timeout: false,
        speed_scale: None,
    }));
    let w = |dst: &mut PlanSlot| *dst = slot.clone();
    let r_a = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            1,
            snap.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w,
                kind: PreparedActionKind::Gripper,
                prepared_boot: boot,
                admission_request_id: 0,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    assert!(
        reps.iter().any(|r| matches!(r, Reply::Refused { request, code: "UNSUPPORTED", .. } if *request == r_a)),
        "expected UNSUPPORTED for missing torque feedback, got {reps:?}"
    );
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);
    handle.stop_and_join();

    // (b) Pos-force close speed quantizing to zero -> INVALID_COMMAND, 0 alloc!
    let clock2 = ScriptedClock::new();
    let mut drv2 = ZeroAllocDriver::new(&INSPECT, clock2.clone());
    drv2.pos_force_close_speed = Some(0.5);
    let mut cfg2 = ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg2.damiao_gripper = Some(armd::model::DamiaoGripperDrive {
        mode: armd::model::DamiaoGripperMode::PosForce,
        current_ceiling_raw: 1000,
        close_speed_rad_s: 0.5,
        calibrated: false,
        ceiling_torque_nm: None,
        torque_reference: None,
        feedback_stale_fault_ms: None,
    });
    let sup2 = Arc::new(ArmSupervisor::new(cfg2).with_driver(Box::new(drv2)));
    sup2.set_lease_enabled(false).await;
    {
        let mut inner = sup2.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
    }
    let mut ec2 = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
    ec2.alloc_timing = true;
    let handle2 = sup2
        .start_executor_with_hooks(
            ec2,
            PersistTestHooks {
                clock: Some(clock2.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    let boot2 = handle2.boot();
    clock2.run_steps(period_ns, 2);
    let snap2 = latest_snap(&sup2, clock2.now() + DEFAULT_ACTION_DEADLINE);
    let mut s_arr2 = [0.0f32; MAX_DOF];
    s_arr2[..snap2.commanded_positions.len()].copy_from_slice(&snap2.commanded_positions);
    let mut slot2 = PlanSlot::default();
    slot2.set(Primitive::Action(StagedAction::Gripper {
        child: None,
        dof: snap2.commanded_positions.len() as u8,
        start: s_arr2,
        command: armd::rt_core::requests::actions::GripperCommandKind::Close,
        fraction: None,
        grasp: true,
        speed: 1e-6, // quantizes to 0 on Damiao pos_force wire!
        timeout_s: Some(4.0),
        max_budget_s: None,
        extend_pos_force_close_timeout: false,
        speed_scale: None,
    }));
    let w2 = |dst: &mut PlanSlot| *dst = slot2.clone();
    let r_b = handle2
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            1,
            snap2.owner.authority,
            clock2.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w2,
                kind: PreparedActionKind::Gripper,
                prepared_boot: boot2,
                admission_request_id: 0,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    clock2.run_steps(period_ns, 2);
    let reps2 = drain(&handle2);
    assert!(
        reps2.iter().any(|r| matches!(r, Reply::Refused { request, code: "INVALID_COMMAND", .. } if *request == r_b)),
        "expected INVALID_COMMAND for 0-quantized pos_force close speed, got {reps2:?}"
    );
    assert_eq!(handle2.stats().total_allocs(), AllocCounts::ZERO);
    handle2.stop_and_join();
}

#[test]
fn b2_r2_m4_mit_motion_blocker_is_zero_alloc() {
    // M4: `mit_joint_mask`, `mit_motion_blocked`, unblocked `mit_motion_blocker`, and
    // prepared-action install refusal on an MIT-configured arm must perform strictly
    // (0, 0, 0, 0) heap operations (no `Vec` allocation in `mit_joints` / `mit_motion_blocker`).
    let mut jt = armd::tracking::JointTrackingConfig::rebot_dm();
    jt.enabled = true;
    for j in &mut jt.joints {
        j.strategy = armd::tracking::TrackingStrategy::Position;
    }
    jt.joints[3].strategy = armd::tracking::TrackingStrategy::Mit;
    jt.joints[3].kp = 10.0;
    jt.joints[3].kd = 0.3;
    jt.joints[3].hold_kp = Some(12.0);
    jt.joints[3].hold_kd = Some(0.4);
    jt.joints[3].effort_limit_nm = 3.0;
    jt.joints[3].gravity_scale = 1.0;

    let cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        workspace: None,
        joint_tracking: None,
        ..ArmConfig::rebot_dm()
    };
    let mut inner = SupervisorInner::new(cfg);
    inner.lease_enabled = false;
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    inner.config.joint_tracking = Some(jt);
    inner.register_live_connection(1).unwrap();
    inner.tick(0.02);

    // Run on a dedicated thread so `TimedAlloc` counts ONLY our checked calls, exercising
    // `mit_joint_mask`, `mit_motion_blocked`, unblocked `mit_motion_blocker`, AND the real
    // `ReleaseFixture` + `Broker` prepared-install refusal path with an MIT-configured arm.
    let boot = BootId(7);
    let mut s_arr = [0.0f32; MAX_DOF];
    let mut t_arr = [0.0f32; MAX_DOF];
    let mv_arr = [1.0f32; MAX_DOF];
    s_arr[..INSPECT.len()].copy_from_slice(&INSPECT);
    t_arr[..INSPECT.len()].copy_from_slice(&INSPECT);
    t_arr[0] += 0.04;
    let mut slot = PlanSlot::default();
    slot.set(Primitive::Action(StagedAction::Joint {
        child: None,
        kind: PreparedActionKind::MoveToJoint,
        dof: INSPECT.len() as u8,
        start: s_arr,
        v0: [0.0; MAX_DOF],
        target: t_arr,
        duration_s: 0.2,
        max_velocity: mv_arr,
        power_scale: None,
        settle: true,
        approach_then: None,
        checked_joints: None,
    }));

    let (mut fx, mut b, _tw, _stop) = armd::rt_core::ops::ReleaseFixture::new(
        inner,
        boot,
        armd::rt_core::broker::Capacities::default(),
    );
    // Prewarm triple-buffer observation slots before entering the zero-alloc window.
    for _ in 0..4 {
        let _ = fx.step();
    }
    let auth = Authority {
        epoch: fx.core.safety.epoch_id(),
        stop_generation: fx.core.stop_gen.load(Ordering::Acquire),
        lease_generation: 0,
        config_revision: fx.core.workspace_revision,
        parent_generation: 0,
    };
    let w = |dst: &mut PlanSlot| *dst = slot.clone();
    let req = b
        .submit(
            ClientId(1),
            1,
            auth,
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w,
                kind: PreparedActionKind::MoveToJoint,
                prepared_boot: boot,
                admission_request_id: 0,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();

    std::thread::spawn(move || {
        armd::diagnostics::alloc::time_this_thread();
        let before = armd::diagnostics::alloc::snapshot_counts();

        assert_eq!(armd::mit::mit_joint_mask(&fx.core.config), 1u64 << 3);

        // MitOwner::Off -> blocked, real RT ReleaseFixture prepared install refused PLAN_STALE, 0 alloc!
        fx.core.mit.owner = armd::mit::MitOwner::Off;
        assert!(fx.core.mit_motion_blocked());
        let rep = fx.step();
        assert_eq!(rep.command, Some((req, Err("PLAN_STALE"))), "{rep:?}");

        // MitOwner::Lost, revoke_pending, park_after_hand_back -> all blocked with 0 allocs!
        fx.core.mit.owner = armd::mit::MitOwner::Lost;
        assert!(fx.core.mit_motion_blocked());
        fx.core.mit.owner = armd::mit::MitOwner::Active;
        assert!(!fx.core.mit_motion_blocked());
        assert!(fx.core.mit_motion_blocker().is_none());
        fx.core.mit.revoke_pending = true;
        assert!(fx.core.mit_motion_blocked());
        fx.core.mit.revoke_pending = false;
        fx.core.mit.park_after_hand_back = true;
        assert!(fx.core.mit_motion_blocked());
        fx.core.mit.park_after_hand_back = false;
        assert!(!fx.core.mit_motion_blocked());
        assert!(fx.core.mit_motion_blocker().is_none());

        let after = armd::diagnostics::alloc::snapshot_counts();
        assert_eq!(
            after.delta_since(before),
            AllocCounts::ZERO,
            "mit_joint_mask / mit_motion_blocked / unblocked mit_motion_blocker / MIT refusal must be strictly 0-alloc"
        );
        // Drop heap-owning values outside the measured window.
        drop(b);
        drop(fx);
    })
    .join()
    .unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn b2_r2_m4_prepared_replay_approach_planned_off_thread_is_zero_alloc_on_rt() {
    // M4: replay approach-to-first-frame (TrajectoryPlanner::plan + power_budget fit +
    // workspace sweep) is planned OFF-thread at prepare time; on `armd-control`, install
    // only verifies start position and constructs the inline quintic with strictly 0 allocs
    // and 0 frees across BOTH standalone and macro-child replays.
    let tmp = tempfile::tempdir().unwrap();
    let mut frame0 = INSPECT;
    frame0[0] += 0.22; // displaced from INSPECT -> non-trivial approach quintic!
    frame0[1] -= 0.10;
    let mut frames = Vec::new();
    for k in 0..=25 {
        let f = k as f32 / 25.0;
        let mut q = frame0.to_vec();
        q[0] -= 0.08 * f;
        frames.push(armd::recording::RecordFrame {
            timestamp: 10.0 + k as f64 * 0.02,
            observation_state: q.clone(),
            action: q,
        });
    }
    armd::recording::write_frames_to_parquet(&tmp.path().join("ep_displaced.parquet"), &frames)
        .unwrap();

    let clock = ScriptedClock::new();
    let drv = ZeroAllocDriver::new(&INSPECT, clock.clone());
    let mut cfg = ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    if let Some(pb) = cfg.power_budget.as_mut() {
        pb.enabled = true;
        pb.planning.enabled = true;
    }
    assert!(cfg.workspace.is_some(), "rebot_dm must have workspace enabled to prove off-thread sweep");
    assert!(
        cfg.power_budget.as_ref().is_some_and(|p| p.enabled && p.planning.enabled),
        "rebot_dm must have planning power budget enabled to prove off-thread power fit"
    );

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.episodes = armd::recording::EpisodeStore::new(Some(tmp.path().to_path_buf()));
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
        let env = armd::rt_core::requests::actions::ActionPrepEnv::from_inner(&inner);
        sup.hub().update_prep_env(Arc::new(env));
    }

    let period_ns = 20_000_000u64;
    let mut ec = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
    ec.alloc_timing = true;
    let handle = sup
        .start_executor_with_hooks(
            ec,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    let boot = handle.boot();
    clock.run_steps(period_ns, 2);

    // 1) Off-thread prepare_replay plans, power-fits, and workspace-sweeps the approach once!
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let prep = armd::rt_core::requests::actions::prepare_replay(
        &snap,
        &json!({"episode": "ep_displaced", "speed": 1.0, "gripper": false}),
    )
    .expect("off-thread prepare_replay must succeed");
    assert!(
        prep.plan.approach.duration > 0.15,
        "displaced first frame must produce a non-trivial pre-planned approach, got {}",
        prep.plan.approach.duration
    );

    // 2) Start-drift refusal on RT -> PLAN_STALE, 0 alloc!
    let mut drifted_slot = prep.to_plan_slot(None).unwrap();
    if let Primitive::Action(StagedAction::Replay { ref mut start, .. }) = drifted_slot.primitive {
        start[0] += 0.05;
    }
    let w_drift = |dst: &mut PlanSlot| *dst = drifted_slot.clone();
    let r_drift = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            1,
            snap.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w_drift,
                kind: PreparedActionKind::Replay,
                prepared_boot: boot,
                admission_request_id: 0,
                check_workspace_revision: true,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    assert!(
        reps.iter().any(|r| matches!(r, Reply::Refused { request, code: "PLAN_STALE", .. } if *request == r_drift)),
        "expected PLAN_STALE for drifted replay start, got {reps:?}"
    );
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    // 3) Invalid preplanned approach duration (0.0) -> INVALID_COMMAND, 0 alloc!
    let mut bad_dur_slot = prep.to_plan_slot(None).unwrap();
    if let Primitive::Action(StagedAction::Replay { ref mut approach_duration_s, .. }) = bad_dur_slot.primitive {
        *approach_duration_s = 0.0;
    }
    let w_bad = |dst: &mut PlanSlot| *dst = bad_dur_slot.clone();
    let r_bad = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            2,
            snap.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w_bad,
                kind: PreparedActionKind::Replay,
                prepared_boot: boot,
                admission_request_id: 0,
                check_workspace_revision: true,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    assert!(
        reps.iter().any(|r| matches!(r, Reply::Refused { request, code: "INVALID_COMMAND", .. } if *request == r_bad)),
        "expected INVALID_COMMAND for 0 approach duration, got {reps:?}"
    );
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    // 4) Accepted standalone replay with displaced pre-planned approach -> Completed, 0 alloc!
    let ok_slot = prep.to_plan_slot(None).unwrap();
    let w_ok = |dst: &mut PlanSlot| *dst = ok_slot.clone();
    let r_ok = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            3,
            snap.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w_ok,
                kind: PreparedActionKind::Replay,
                prepared_boot: boot,
                admission_request_id: 0,
                check_workspace_revision: true,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    clock.run_steps(period_ns, 250);
    let reps = drain(&handle);
    assert!(
        reps.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == r_ok)),
        "expected Completed standalone replay with displaced approach, got {reps:?}"
    );
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    // 5) Accepted macro-child replay with displaced pre-planned approach -> Done + Completed, 0 alloc!
    let snap2 = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let begin = BeginMacroSpec::new(MacroKind::Sequence, boot);
    let r_b = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            4,
            snap2.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(begin)),
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    let parent = reps
        .iter()
        .find_map(|r| match r {
            Reply::Ack { request, action: Some(a), .. } if *request == r_b => Some(*a),
            _ => None,
        })
        .expect("BeginMacro ACK");

    let snap3 = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let prep_child = armd::rt_core::requests::actions::prepare_replay(
        &snap3,
        &json!({"episode": "ep_displaced", "speed": 1.0, "gripper": false}),
    )
    .expect("off-thread child prepare_replay must succeed");
    let c_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 15.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        contact: None,
    };
    let c_slot = armd::rt_core::requests::actions::prepare_child_replay_slot(
        &prep_child.plan,
        snap3.env.tool_digest_hash,
        c_spec,
    )
    .unwrap();
    let w_c = |dst: &mut PlanSlot| *dst = c_slot.clone();
    handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            5,
            snap3.owner.authority,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild {
                write: &w_c,
                prepared_boot: boot,
                check_workspace_revision: true,
                calibration_revision: None,
                grasp_setup_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    clock.run_steps(period_ns, 250);
    let _ = drain(&handle);
    assert!(matches!(
        sup.hub().latest().unwrap().rt.owner.macro_state.child_sub,
        SubState::Done(_)
    ));
    let snap4 = latest_snap(&sup, clock.now() + DEFAULT_FINISH_DEADLINE);
    let fin = FinishMacroSpec::done(parent, boot);
    handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            6,
            snap4.owner.authority,
            clock.now() + DEFAULT_FINISH_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(fin)),
        )
        .unwrap();
    clock.run_steps(period_ns, 3);
    let reps = drain(&handle);
    assert!(
        reps.iter().any(|r| matches!(r, Reply::Terminal { request, action, outcome: Outcome::Completed, children: 1 } if *request == r_b && *action == parent)),
        "expected macro parent Completed, got {reps:?}"
    );
    assert_eq!(handle.stats().release_allocs.snapshot(), AllocCounts::ZERO);
    assert_eq!(handle.stats().wake_allocs.snapshot(), AllocCounts::ZERO);
    handle.stop_and_join();
}

#[test]
fn b2_r2_m5_1000_refused_rt_gripper_installs_preserve_learning_pool_and_are_zero_alloc() {
    // M5: 1,000 refused qualifying-open RT gripper installs across BOTH standalone
    // (`install_gripper_rt`) and macro-child (`install_macro_child_slot_rt`) paths must
    // leave the RT `MoveSamples` spare pool completely unchanged AND perform strictly
    // (0, 0, 0, 0) heap operations on `armd-control`; unused spares passed to
    // `GripperMotion::with_baseline_data_and_samples` on a non-learning move are returned
    // untouched rather than dropped.
    let cfg = ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    let gp = cfg.gripper_policy.clone().expect("rebot_dm has gripper_policy");
    let g_idx = cfg.gripper_index().expect("rebot_dm has gripper_index");
    let mut start_q = INSPECT;
    start_q[g_idx] = gp.closed_rad; // closed -> Open is a qualifying LearnMode::Opening move!

    // (a) Direct unit check on `with_baseline_data_and_samples`: closing move returns spare untouched.
    {
        let mut lim = cfg.limits[g_idx].clone();
        lim.name = String::new();
        let mut gp_zero = gp.clone();
        gp_zero.jaw_opening = None;
        let req = armd::motion::GripperRequest {
            command: GripperCommand::Close,
            fraction: None,
            grasp: true,
            speed: 1.0,
            timeout_s: Some(4.0),
            tag: None,
        };
        let mot = armd::motion::GripperMotion::try_with_close_speed_limit_static(
            req,
            gp_zero,
            lim,
            gp.open_rad,
            g_idx,
            None,
        )
        .unwrap();
        assert!(!mot.will_record_learning(armd::motion::LearnMode::Opening));
        let spare = armd::gripper_baseline::MoveSamples::default();
        let (mot2, returned) = mot.with_baseline_data_and_samples(
            None,
            armd::motion::LearnMode::Opening,
            Some(spare),
        );
        assert!(
            returned.is_some(),
            "with_baseline_data_and_samples must return unused MoveSamples spare instead of dropping it"
        );
        assert!(!mot2.has_inner_heap_storage());
    }

    // (b) 1,000 refused RT gripper installs (500 standalone + 500 macro-child) on ReleaseFixture!
    let mut inner = SupervisorInner::new(cfg);
    inner.lease_enabled = false;
    inner.commanded_positions = start_q.to_vec();
    inner.measured_positions = start_q.to_vec();
    inner.target_positions = start_q.to_vec();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    inner.tick(0.02);
    inner.register_live_connection(1).unwrap();
    // Prewarm RT buffers before constructing `ReleaseFixture` (which also prewarms `move_samples_spares`).
    inner.prewarm_rt_buffers();

    let boot = BootId(11);
    let (mut fx, mut b, _tw, _stop) = armd::rt_core::ops::ReleaseFixture::new(
        inner,
        boot,
        armd::rt_core::broker::Capacities::default(),
    );
    let initial_pool = fx.core.test_move_samples_spare_count();
    assert!(initial_pool >= 2, "expected prewarmed move_samples_spares >= 2, got {initial_pool}");
    // Prewarm triple-buffer observation slots before entering timed zero-alloc thread.
    for _ in 0..4 {
        let _ = fx.step();
    }

    let mut s_arr = [0.0f32; MAX_DOF];
    s_arr[..start_q.len()].copy_from_slice(&start_q);
    let mut standalone_slot = PlanSlot::default();
    standalone_slot.set(Primitive::Action(StagedAction::Gripper {
        child: None,
        dof: start_q.len() as u8,
        start: s_arr,
        command: armd::rt_core::requests::actions::GripperCommandKind::Open,
        fraction: None,
        grasp: false,
        speed: 1.0,
        timeout_s: Some(4.0),
        max_budget_s: None,
        extend_pos_force_close_timeout: false,
        speed_scale: None,
    }));

    // Consume admission_request_id = 42 once via a real no-op or mark it already consumed
    // by first running a standalone BeginMacro (which consumes request_id = 42 -> later
    // requests with admission_request_id = 42 fail inside `consume_action_admission` AFTER
    // `GripperMotion::try_with_close_speed_limit_static`!).
    let auth_now = |fx: &armd::rt_core::ops::ReleaseFixture<SupervisorInner>| Authority {
        epoch: fx.core.safety.epoch_id(),
        stop_generation: fx.core.stop_gen.load(Ordering::Acquire),
        lease_generation: 0,
        config_revision: fx.core.workspace_revision,
        parent_generation: 0,
    };

    // Start a macro parent with admission_request_id = 42 (valid, single-use) so we can test
    // BOTH standalone STAMP_REUSED refusals (id=42) AND macro-child max_budget_s / STAMP_REUSED
    // refusals under the active parent!
    let mut begin = BeginMacroSpec::new(MacroKind::Grasp, boot);
    begin.admission_request_id = 42;
    let r_b = b
        .submit(
            ClientId(1),
            1,
            auth_now(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(begin)),
        )
        .unwrap();
    let rep_b = fx.step();
    assert!(matches!(rep_b.command, Some((r, Ok(_))) if r == r_b), "{rep_b:?}");
    let parent = fx.active().expect("macro parent active");
    let mut seq = 2u64;
    b.poll(|_, _| true);

    let child_bad_budget = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let mut child_bad_budget_slot = PlanSlot::default();
    child_bad_budget_slot.set(Primitive::Action(StagedAction::Gripper {
        child: Some(child_bad_budget),
        dof: start_q.len() as u8,
        start: s_arr,
        command: armd::rt_core::requests::actions::GripperCommandKind::Open,
        fraction: None,
        grasp: false,
        speed: 1.0,
        timeout_s: Some(4.0),
        max_budget_s: Some(0.0), // fails post-GripperMotion max_budget_s check (INVALID_COMMAND)
        extend_pos_force_close_timeout: false,
        speed_scale: None,
    }));

    let mut child_ok_slot = PlanSlot::default();
    child_ok_slot.set(Primitive::Action(StagedAction::Gripper {
        child: Some(child_bad_budget),
        dof: start_q.len() as u8,
        start: s_arr,
        command: armd::rt_core::requests::actions::GripperCommandKind::Open,
        fraction: None,
        grasp: false,
        speed: 1.0,
        timeout_s: Some(4.0),
        max_budget_s: None,
        extend_pos_force_close_timeout: false,
        speed_scale: None,
    }));

    let mut child_nan_slot = child_bad_budget_slot.clone();
    if let Primitive::Action(StagedAction::Gripper { ref mut max_budget_s, .. }) =
        child_nan_slot.primitive
    {
        *max_budget_s = Some(f64::NAN);
    }

    // Run all 1,000 refusals on a dedicated `TimedAlloc`-measured thread!
    std::thread::spawn(move || {
        armd::diagnostics::alloc::time_this_thread();
        let before = armd::diagnostics::alloc::snapshot_counts();

        // 500 standalone qualifying-open gripper installs refused at `consume_action_admission`
        // (`STAMP_REUSED` via reused `admission_request_id: 42`, AFTER `GripperMotion` construction!).
        let w_standalone = |dst: &mut PlanSlot| dst.copy_from(&standalone_slot).unwrap();
        for _ in 0..500 {
            let a = auth_now(&fx);
            let req = b
                .submit(
                    ClientId(1),
                    seq,
                    a,
                    fx.now() + DEFAULT_ACTION_DEADLINE,
                    Submit::Action(ActionSubmit::InstallPrepared {
                        write: &w_standalone,
                        kind: PreparedActionKind::Gripper,
                        prepared_boot: boot,
                        admission_request_id: 42,
                        check_workspace_revision: false,
                        poses_revision: None,
                        calibration_revision: None,
                        gripper_calib_revision: None,
                        power_budget_revision: None,
                    }),
                )
                .unwrap();
            seq += 1;
            let rep = fx.step();
            assert_eq!(rep.command, Some((req, Err("STAMP_REUSED"))));
            b.poll(|_, _| true);
        }

        // 250 macro-child qualifying-open gripper installs refused at post-GripperMotion `max_budget_s` (`INVALID_COMMAND`)!
        let w_child_bad = |dst: &mut PlanSlot| dst.copy_from(&child_bad_budget_slot).unwrap();
        for _ in 0..250 {
            let mut a = auth_now(&fx);
            a.parent_generation = parent.0;
            let req = b
                .submit(
                    ClientId(1),
                    seq,
                    a,
                    fx.now() + DEFAULT_ACTION_DEADLINE,
                    Submit::Action(ActionSubmit::InstallChild {
                        write: &w_child_bad,
                        prepared_boot: boot,
                        check_workspace_revision: false,
                        calibration_revision: None,
                        grasp_setup_revision: None,
                        gripper_calib_revision: None,
                        power_budget_revision: None,
                    }),
                )
                .unwrap();
            seq += 1;
            let rep = fx.step();
            assert_eq!(rep.command, Some((req, Err("INVALID_COMMAND"))));
            b.poll(|_, _| true);
        }

        // 250 macro-child qualifying-open gripper installs refused with NaN `max_budget_s` (`INVALID_COMMAND`)!
        let w_child_nan = |dst: &mut PlanSlot| dst.copy_from(&child_nan_slot).unwrap();
        for _ in 0..250 {
            let mut a = auth_now(&fx);
            a.parent_generation = parent.0;
            let req = b
                .submit(
                    ClientId(1),
                    seq,
                    a,
                    fx.now() + DEFAULT_ACTION_DEADLINE,
                    Submit::Action(ActionSubmit::InstallChild {
                        write: &w_child_nan,
                        prepared_boot: boot,
                        check_workspace_revision: false,
                        calibration_revision: None,
                        grasp_setup_revision: None,
                        gripper_calib_revision: None,
                        power_budget_revision: None,
                    }),
                )
                .unwrap();
            seq += 1;
            let rep = fx.step();
            assert_eq!(rep.command, Some((req, Err("INVALID_COMMAND"))));
            b.poll(|_, _| true);
        }

        // Pool MUST be completely unchanged after 1,000 refusals AND 0 heap ops on RT!
        assert_eq!(
            fx.core.test_move_samples_spare_count(),
            initial_pool,
            "1,000 refused RT gripper installs must leave move_samples_spares unchanged"
        );
        let after_refusals = armd::diagnostics::alloc::snapshot_counts();
        assert_eq!(
            after_refusals.delta_since(before),
            AllocCounts::ZERO,
            "1,000 refused RT gripper installs must perform strictly 0 allocs and 0 frees on RT"
        );

        // 1,001st install: ACCEPTED macro-child qualifying opening -> consumes 1 spare with 0 alloc!
        let w_child_ok = |dst: &mut PlanSlot| dst.copy_from(&child_ok_slot).unwrap();
        let mut a = auth_now(&fx);
        a.parent_generation = parent.0;
        let req_ok = b
            .submit(
                ClientId(1),
                seq,
                a,
                fx.now() + DEFAULT_ACTION_DEADLINE,
                Submit::Action(ActionSubmit::InstallChild {
                    write: &w_child_ok,
                    prepared_boot: boot,
                    check_workspace_revision: false,
                    calibration_revision: None,
                    grasp_setup_revision: None,
                    gripper_calib_revision: None,
                    power_budget_revision: None,
                }),
            )
            .unwrap();
        let rep_ok = fx.step();
        assert!(matches!(rep_ok.command, Some((r, Ok(_))) if r == req_ok), "{rep_ok:?}");
        assert_eq!(
            fx.core.test_move_samples_spare_count(),
            initial_pool - 1,
            "accepted qualifying opening after 1,000 refusals must take 1 spare from the intact pool"
        );
        let after_accept = armd::diagnostics::alloc::snapshot_counts();
        assert_eq!(
            after_accept.delta_since(before),
            AllocCounts::ZERO,
            "accepted qualifying opening must also perform strictly 0 allocs and 0 frees on RT"
        );

        drop(b);
        drop(fx);
        drop(standalone_slot);
        drop(child_bad_budget_slot);
        drop(child_nan_slot);
        drop(child_ok_slot);
    })
    .join()
    .unwrap();
}

/// Phase B adaptive grasp RT zero-alloc proof:
/// - `BeginMacro` carrying all 5 captured revisions (`workspace`, `calibration`, `grasp_setup`,
///   `gripper_calib`, `power_budget`)
/// - `InstallChild` for a verbatim swept-joint `JointPath` with `lock_j7_hold: true` + 5 revisions
///   (verifies J7 stays identically locked across every tick AND 0 allocs)
/// - `InstallChild` for a fenced J7 gripper move + 5 revisions
/// - `FinishMacro(Done, HeldUpdate::Set)` with stale revision refusal (`PLAN_STALE`, `held` NOT set,
///   0 allocs) followed by matching revision commit (`Done`, `held` set, 0 allocs)
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_adaptive_macro_fenced_and_j7_locked_is_zero_alloc_on_rt() {
    let clock = ScriptedClock::new();
    let mut drv = ZeroAllocDriver::new(&INSPECT, clock.clone());
    drv.pos_force_close_speed = Some(2.0);
    let pf_targets = Arc::clone(&drv.pf_target_count);
    let pf_holds = Arc::clone(&drv.pf_hold_count);
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv)),
    );
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
    }

    let period_ns = 20_000_000u64;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_nanos(period_ns));
    cfg.alloc_timing = true;
    let handle = sup
        .start_executor_with_hooks(
            cfg,
            PersistTestHooks {
                clock: Some(clock.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    let boot = handle.boot();
    clock.run_steps(period_ns, 3);
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO);

    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let rev_ws = snap.env.workspace_revision;
    let rev_cal = snap.env.calibration_revision;
    let rev_gs = snap.env.grasp_setup_revision;
    let rev_gc = snap.env.gripper_calib_revision;
    let rev_pb = snap.env.power_budget_revision;
    let j7_locked_val = snap.commanded_positions[6];
    pf_targets.store(0, Ordering::Relaxed);
    pf_holds.store(0, Ordering::Relaxed);

    // 1) Adaptive BeginMacro with `j7_intent_only = true`, start fence, and all 5 revisions
    let mut begin = BeginMacroSpec::new(MacroKind::Grasp, boot)
        .with_start(&snap.commanded_positions)
        .unwrap();
    begin.j7_intent_only = true;
    begin.check_workspace_revision = true;
    begin.calibration_revision = Some(rev_cal);
    begin.grasp_setup_revision = Some(rev_gs);
    begin.gripper_calib_revision = Some(rev_gc);
    begin.power_budget_revision = Some(rev_pb);
    let mut auth = snap.owner.authority;
    auth.config_revision = rev_ws;
    let r_begin = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            1,
            auth,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(begin)),
        )
        .unwrap();
    clock.run_steps(period_ns, 4);
    let reps = drain(&handle);
    let parent = reps
        .iter()
        .find_map(|r| match r {
            Reply::Ack { request, action: Some(a), .. } if *request == r_begin => Some(*a),
            _ => None,
        })
        .expect("fenced BeginMacro ACK");
    assert_eq!(pf_targets.load(Ordering::Relaxed), 0, "BeginMacro(j7_intent_only) must emit 0 PosForce Target");
    assert!(pf_holds.load(Ordering::Relaxed) >= 4, "BeginMacro(j7_intent_only) must emit PosForce Hold");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "fenced BeginMacro must be 0-alloc");

    // 2) Swept-joint verbatim JointPath child with `lock_j7_hold: true` + all 5 revisions
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let mut q_goal = [0.0f64; 6];
    for i in 0..6 {
        q_goal[i] = f64::from(snap.commanded_positions[i]);
    }
    q_goal[0] += 0.06;
    let inp = snap.plan_input_in(GoalFrame::Tcp).unwrap();
    let swept = armd::grasp_select::feasibility::plan_swept_joint_path(
        &inp,
        q_goal,
        &[[0.0, 0.0, 0.0]],
        -1.0,
    )
    .expect("swept joint path");
    let c1_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let c1_slot = prepare_child_cartesian_slot_with_options(
        &snap.commanded_positions,
        &swept,
        None,
        None,
        true,
        c1_spec,
    )
    .expect("stage lock_j7_hold slot");
    let mut c1_auth = snap.owner.authority;
    c1_auth.config_revision = rev_ws;
    c1_auth.parent_generation = parent.0;
    let w1 = |dst: &mut PlanSlot| *dst = c1_slot.clone();
    let r_c1 = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            2,
            c1_auth,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild {
                write: &w1,
                prepared_boot: boot,
                check_workspace_revision: true,
                calibration_revision: Some(rev_cal),
                grasp_setup_revision: Some(rev_gs),
                gripper_calib_revision: Some(rev_gc),
                power_budget_revision: Some(rev_pb),
            }),
        )
        .unwrap();
    for _ in 0..80 {
        clock.run_steps(period_ns, 1);
        let cur = sup.hub().latest().unwrap().rt.commanded_positions[6];
        assert!(
            (cur - j7_locked_val).abs() < 1e-6,
            "J7 must stay identically locked during lock_j7_hold path: cur={cur} expected={j7_locked_val}"
        );
    }
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Ack { request, .. } if *request == r_c1)), "{reps:?}");
    assert_eq!(pf_targets.load(Ordering::Relaxed), 0, "arm-only child & inter-child gap must emit 0 PosForce Target");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "lock_j7_hold swept-joint child must be 0-alloc");

    // 3) Fenced J7 gripper child with all 5 revisions (ONLY here may PosForce Target fire!)
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let c2_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 2 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let c2_slot = prepare_child_gripper_slot(
        &snap.commanded_positions,
        GripperCommand::Move,
        Some(0.4),
        false,
        1.0,
        Some(8.0),
        Some(20.0),
        true,
        c2_spec,
    )
    .expect("stage fenced gripper slot");
    let mut c2_auth = snap.owner.authority;
    c2_auth.config_revision = rev_ws;
    c2_auth.parent_generation = parent.0;
    let w2 = |dst: &mut PlanSlot| *dst = c2_slot.clone();
    let r_c2 = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            3,
            c2_auth,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallChild {
                write: &w2,
                prepared_boot: boot,
                check_workspace_revision: true,
                calibration_revision: Some(rev_cal),
                grasp_setup_revision: Some(rev_gs),
                gripper_calib_revision: Some(rev_gc),
                power_budget_revision: Some(rev_pb),
            }),
        )
        .unwrap();
    clock.run_steps(period_ns, 200);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Ack { request, .. } if *request == r_c2)), "{reps:?}");
    assert!(pf_targets.load(Ordering::Relaxed) >= 1, "GripperMotion child must emit PosForce Target");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "fenced gripper child must be 0-alloc");

    // 4a) FinishMacro(Done, HeldUpdate::Set) with STALE grasp_setup_revision -> refused PLAN_STALE,
    //     held NOT set, and strictly 0 allocs!
    let snap = latest_snap(&sup, clock.now() + DEFAULT_ACTION_DEADLINE);
    let mut stale_fin = FinishMacroSpec::done(parent, boot);
    stale_fin.holding = Some(true);
    stale_fin.held = HeldUpdate::Set {
        label: HolderLabel::new("banana"),
        spill_safe: false,
    };
    stale_fin.check_workspace_revision = true;
    stale_fin.calibration_revision = Some(rev_cal);
    stale_fin.grasp_setup_revision = Some(rev_gs + 99);
    stale_fin.gripper_calib_revision = Some(rev_gc);
    stale_fin.power_budget_revision = Some(rev_pb);
    let mut fin_auth = snap.owner.authority;
    fin_auth.config_revision = rev_ws;
    fin_auth.parent_generation = parent.0;
    let r_stale = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            4,
            fin_auth,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(stale_fin)),
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    assert!(
        reps.iter().any(|r| matches!(r, Reply::Refused { request, code: "PLAN_STALE", .. } if *request == r_stale)),
        "stale FinishMacro(Done) must refuse PLAN_STALE: {reps:?}"
    );
    assert!(sup.hub().latest().unwrap().rt.owner.held.is_none(), "refused FinishMacro must NOT set held");
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "stale FinishMacro refusal must be 0-alloc");

    // 4b) Matching 5-revision FinishMacro(Done, HeldUpdate::Set) -> commits held, 0 allocs!
    let mut ok_fin = stale_fin;
    ok_fin.grasp_setup_revision = Some(rev_gs);
    let r_ok = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(1),
            5,
            fin_auth,
            clock.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(ok_fin)),
        )
        .unwrap();
    clock.run_steps(period_ns, 2);
    let reps = drain(&handle);
    assert!(reps.iter().any(|r| matches!(r, Reply::Ack { request, .. } if *request == r_ok)), "{reps:?}");
    assert_eq!(
        sup.hub().latest().unwrap().rt.owner.held.map(|h| h.label.as_str().to_string()),
        Some("banana".to_string())
    );
    assert_eq!(handle.stats().total_allocs(), AllocCounts::ZERO, "fenced FinishMacro commit must be 0-alloc");

    handle.stop_and_join();
}
