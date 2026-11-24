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

use std::sync::atomic::{AtomicBool, Ordering};
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
    prepare_child_cartesian_slot, prepare_child_gripper_slot, prepare_child_joint_slot,
    prepare_gripper, prepare_move_to, ActionSnapshot, ActionSubmit, BeginMacroSpec,
    FinishMacroSpec, HeldUpdate, MacroKind, PreparedActionKind, StagedAction,
    DEFAULT_ACTION_DEADLINE, DEFAULT_FINISH_DEADLINE,
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
            ceiling_torque_estimate_nm: None,
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
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: true, calibration_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
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
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: true, calibration_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
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
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: false, calibration_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
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
            Submit::Action(ActionSubmit::InstallChild { write: &write, prepared_boot: boot, check_workspace_revision: false, calibration_revision: None, gripper_calib_revision: None, power_budget_revision: None })).unwrap();
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
        reps.iter().any(|r| matches!(r, Reply::Refused { request, code: "UNSUPPORTED" } if *request == r_a)),
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
        reps2.iter().any(|r| matches!(r, Reply::Refused { request, code: "INVALID_COMMAND" } if *request == r_b)),
        "expected INVALID_COMMAND for 0-quantized pos_force close speed, got {reps2:?}"
    );
    assert_eq!(handle2.stats().total_allocs(), AllocCounts::ZERO);
    handle2.stop_and_join();
}
