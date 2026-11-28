//! Decision Item 5: RT workspace return paths reuse `ws_eval_bufs` and existing scratch with
//! zero heap allocations and zero frees on acceptance, refusal, and fail-closed oversized/undersized
//! inputs on `armd-control`.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

use armd::chain::ArmModel;
use armd::model::{ArmObservation, KeepoutBox};
use armd::rt_core::broker::{Authority, BootId};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetFrame, MAX_DOF};
use armd::rt_core::ops::{CommitCtx, CoreOps};
use armd::{ArmConfig, SupervisorInner};

thread_local! {
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn count(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) {
        let _ = c.try_with(|n| n.set(n.get() + 1));
    }
}

// SAFETY: forwards to System; thread-local counters are const-initialized and do not allocate.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        count(&ALLOCS);
        unsafe { System.alloc(l) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        count(&ALLOCS);
        unsafe { System.alloc_zeroed(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        count(&FREES);
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        count(&ALLOCS);
        count(&FREES);
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

const BOOT: BootId = BootId(7);
const CONN: &str = "conn-ws-return-alloc";
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];

fn ctx(now: Instant) -> CommitCtx {
    CommitCtx {
        arm: ArmId::LOCAL,
        boot: BOOT,
        now,
        stop_generation: 0,
    }
}

fn make_frame(
    stream: StreamId,
    authority: Authority,
    q_vec: &[f32],
    seq: u64,
    now: Instant,
) -> TargetFrame {
    let mut q = [0.0; MAX_DOF];
    q[..q_vec.len()].copy_from_slice(q_vec);
    TargetFrame {
        arm: ArmId::LOCAL,
        boot: BOOT,
        stream,
        seq,
        authority,
        policy_generation: 0,
        input_at: now,
        vision_at: None,
        vision_frame_id: None,
        deadline: now + Duration::from_secs(1),
        dof: q_vec.len() as u8,
        q,
        replaceable: true,
    }
}

/// Zero-alloc acceptance and refusal on RT during outside-workspace recovery (including
/// running-minimum anti-ratcheting refusal and hard-slot deepening refusal).
#[test]
fn rt_workspace_return_acceptance_and_refusal_zero_alloc() {
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg.collision = None;
    cfg.power_budget = None;
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    let check_pts = ws.check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts
            .iter()
            .map(|c| p.transform_point(&c.map(f64::from))[0])
            .fold(p.p[0], f64::max)
    };
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let start_max_x = max_x_at(&q_start);
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;

    let mut q_probe = q_start;
    q_probe[1] += 0.01;
    let fwd_sign = if max_x_at(&q_probe) > start_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = (max_x_at(&{
        let mut q = q_start;
        q[1] += 0.01 * fwd_sign;
        q
    }) - start_max_x)
        / 0.01;
    let mut q_step1 = q_start;
    q_step1[1] += (0.003 / dx_per_rad) as f32 * fwd_sign;
    let mut q_step2 = q_start;
    q_step2[1] += (0.0062 / dx_per_rad) as f32 * fwd_sign;
    let mut q_retreat = q_start;
    q_retreat[1] -= (0.010 / dx_per_rad) as f32 * fwd_sign;

    let mut inner = SupervisorInner::new(cfg);
    inner.rt_executor_owned = true;
    inner.commanded_positions = q_start.to_vec();
    inner.measured_positions = q_start.to_vec();
    inner.target_positions = q_start.to_vec();
    inner.prewarm_rt_buffers();
    inner.tick(0.02);
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::enable_torque(a))
            .unwrap();
    }
    let gen = inner.acquire_lease(CONN).unwrap();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::authorize_servo_stream(a, gen).unwrap();
    }
    let stream = inner.core.target_authority().stream.unwrap();
    let authority = Authority {
        lease_generation: gen,
        epoch: inner.safety.epoch_id(),
        config_revision: inner.workspace_revision,
        ..Authority::default()
    };
    let mut obs = ArmObservation::blank_rt();
    let now = Instant::now();
    // Warm up servo_target buffer once off the counted region.
    let f_warm = make_frame(stream, authority, &q_start, 1, now);
    inner.consume_target(&f_warm, &ctx(now)).expect("warmup");
    inner.tick_control(0.02);
    inner.fill_observation(0.02, &mut obs);

    let f_step1 = make_frame(stream, authority, &q_step1, 2, now);
    let f_step2 = make_frame(stream, authority, &q_step2, 3, now);
    let f_retreat = make_frame(stream, authority, &q_retreat, 4, now);

    // 1. Accepted +3 mm frame + ticks + settle: 0 allocs / 0 frees.
    let (ok1, a1, fr1) = counted(|| {
        let r = inner.consume_target(&f_step1, &ctx(now));
        for k in 0..20 {
            inner.tick_control(0.02);
            inner.fill_observation(0.04 + 0.02 * k as f64, &mut obs);
        }
        r
    });
    assert!(ok1.is_ok(), "step1 must be accepted: {ok1:?}");
    assert_eq!((a1, fr1), (0, 0), "step1 recovery ticks allocated/freed");

    // 2. Refused ratcheting frame (+6.2 mm total from start): 0 allocs / 0 frees.
    let (rej2, a2, fr2) = counted(|| inner.consume_target(&f_step2, &ctx(now)));
    let err2 = rej2.expect_err("ratcheting frame 2 must be refused on RT");
    assert_eq!(err2.code, "WORKSPACE_VIOLATION");
    assert_eq!((a2, fr2), (0, 0), "ratcheting refusal allocated/freed");

    // 3. Accepted retreat frame + tick: 0 allocs / 0 frees.
    let (ok3, a3, fr3) = counted(|| {
        let r = inner.consume_target(&f_retreat, &ctx(now));
        inner.tick_control(0.02);
        inner.fill_observation(1.0, &mut obs);
        r
    });
    assert!(ok3.is_ok(), "retreat frame must be accepted: {ok3:?}");
    assert_eq!((a3, fr3), (0, 0), "retreat acceptance allocated/freed");
}

/// Decision Item 5: undersized scratch / oversized workspace on RT MUST fail closed with 0 allocs / 0 frees.
#[test]
fn rt_oversized_workspace_and_undersized_scratch_fail_closed_zero_alloc() {
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg.collision = None;
    cfg.power_budget = None;
    let mut inner = SupervisorInner::new(cfg);
    inner.rt_executor_owned = true;
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    inner.prewarm_rt_buffers();
    inner.tick(0.02);
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::enable_torque(a))
            .unwrap();
    }
    let gen = inner.acquire_lease(CONN).unwrap();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::authorize_servo_stream(a, gen).unwrap();
    }
    let stream = inner.core.target_authority().stream.unwrap();
    let authority = Authority {
        lease_generation: gen,
        epoch: inner.safety.epoch_id(),
        config_revision: inner.workspace_revision,
        ..Authority::default()
    };
    let now = Instant::now();
    let mut obs = ArmObservation::blank_rt();
    // Warm up servo_target buffer first.
    let mut q_ok = INSPECT;
    q_ok[0] += 0.01;
    inner
        .consume_target(&make_frame(stream, authority, &q_ok, 1, now), &ctx(now))
        .expect("warmup");
    inner.tick_control(0.02);
    inner.fill_observation(0.02, &mut obs);

    // Mutate workspace to exceed pre-sized scratch WITHOUT calling prepare_workspace_eval_bufs()!
    // On RT (`rt_executor_owned == true`), both servo intake and servo step MUST fail closed with (0, 0) allocs/frees!
    let ws = inner.config.workspace.as_mut().unwrap();
    for i in 0..8 {
        ws.keepout_boxes.push(KeepoutBox {
            name: format!("extra_{i}"),
            min_m: [1.0, 1.0, 1.0],
            max_m: [1.1, 1.1, 1.1],
        });
    }

    let mut q_next = q_ok;
    q_next[0] += 0.01;
    let f_next = make_frame(stream, authority, &q_next, 2, now);

    // 1. RT servo intake with undersized scratch -> fail closed with WORKSPACE_VIOLATION and (0, 0) allocs/frees!
    let (res_intake, a1, fr1) = counted(|| inner.consume_target(&f_next, &ctx(now)));
    let err = res_intake.expect_err("RT intake with undersized scratch must fail closed");
    assert_eq!(err.code, "WORKSPACE_VIOLATION");
    assert_eq!(
        (a1, fr1),
        (0, 0),
        "RT intake fail-closed on undersized scratch must not allocate or free"
    );

    // 2. RT servo step with undersized scratch -> fail closed into workspace hold with (0, 0) allocs/frees!
    // Re-install a servo_target directly (without allocating) so tick_control runs servo_step_workspace_rt.
    if let Some(t) = inner.servo_target.as_mut() {
        t[0] += 0.05;
    } else {
        inner.servo_target = Some(q_next.to_vec());
    }
    let (_, a2, fr2) = counted(|| {
        inner.tick_control(0.02);
        inner.fill_observation(0.04, &mut obs);
    });
    assert!(
        inner.servo_blocked_hold,
        "RT step with undersized scratch must fail closed into workspace hold"
    );
    assert_eq!(
        (a2, fr2),
        (0, 0),
        "RT step fail-closed on undersized scratch must not allocate or free"
    );
}

struct NoAllocTrackingDriver {
    q: [f32; 7],
    enabled: bool,
    writes: std::sync::Arc<std::sync::atomic::AtomicUsize>,
}

impl armd::driver::ArmDriver for NoAllocTrackingDriver {
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }
    fn capabilities(&self) -> armd::driver::DriverCapabilities {
        armd::driver::DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }
    fn enable_state(&self) -> armd::driver::EnableState {
        if self.enabled { armd::driver::EnableState::Enabled } else { armd::driver::EnableState::Disabled }
    }
    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.enabled }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::ZERO); 7] }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        for _ in 0..7 { out.push(Some(Duration::ZERO)); }
    }
    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        dof == 7
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        for _ in 0..7 { out.push(Some(0)); }
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let now = Instant::now();
        out.clear();
        for _ in 0..7 { out.push(Some(now)); }
    }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        for _ in 0..7 { out.push(1); }
    }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        for _ in 0..7 { out.push(None); }
    }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let now = Instant::now();
        out.clear();
        for _ in 0..7 { out.push(Some(now)); }
    }
    fn fresh_joint_positions_into(&self, joints: &[usize], _max_age: Duration, out: &mut [f32]) -> bool {
        if out.len() < joints.len() { return false; }
        for (dst, &j) in out.iter_mut().zip(joints) {
            let Some(&v) = self.q.get(j) else { return false };
            *dst = v;
        }
        true
    }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
        Ok(self.q.iter().map(|&position| armd::model::JointState { position, velocity: 0.0, torque: 0.0, temperature: 30.0 }).collect())
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<armd::model::JointState>) -> Result<(), armd::driver::ArmDriverError> {
        out.clear();
        for &position in &self.q {
            out.push(armd::model::JointState { position, velocity: 0.0, torque: 0.0, temperature: 30.0 });
        }
        Ok(())
    }
    fn write_joint_targets(&mut self, _targets: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
        self.writes.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), armd::driver::ArmDriverError> {
        self.enabled = enable;
        Ok(())
    }
}

/// R2 Item 4 (Major): Both `compute_position_lead_output_inline` (`tracking_owned`) AND
/// `compute_lead_ramp_out_inline` (`ramp_out_handover`) MUST check `can_eval_rt` before building
/// `WorkspaceEvaluator` and fail closed with (0, 0) heap allocs/frees on undersized scratch.
#[test]
fn rt_tracking_lead_and_ramp_out_undersized_scratch_fail_closed_zero_alloc() {
    use armd::safety::SafetyState;
    use armd::tracking::{JointTrackingConfig, TrackingStrategy};
    use std::sync::atomic::AtomicUsize;

    for is_ramp_out in [false, true] {
        let label = if is_ramp_out { "ramp_out" } else { "tracking_lead" };
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        cfg.collision = None;
        cfg.power_budget = None;
        cfg.torque_guard = None;
        let mut jt = JointTrackingConfig::rebot_dm();
        jt.enabled = true;
        for j in &mut jt.joints { j.strategy = TrackingStrategy::Position; }
        jt.joints[0].strategy = TrackingStrategy::PositionLead;
        jt.joints[0].lag_s = 0.68;
        jt.joints[0].max_lead_rad = 0.03;
        cfg.joint_tracking = Some(jt);

        let writes = std::sync::Arc::new(AtomicUsize::new(0));
        let drv = NoAllocTrackingDriver { q: INSPECT, enabled: true, writes: writes.clone() };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.rt_executor_owned = true;
        inner.safety.set_off_thread_reclaim(true);
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.prewarm_rt_buffers();
        {
            let s = inner.current_admission_stamp();
            inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
        }
        // Warm up steady-state RT buffers (driver read vecs, fallback spares) on normal ticks.
        let mut obs = ArmObservation::blank_rt();
        inner.tick_control(0.02);
        inner.fill_observation(0.02, &mut obs);
        let (_, warm_a, warm_f) = counted(|| inner.tick_control(0.02));
        assert_eq!((warm_a, warm_f), (0, 0), "{label}: warmup steady tick must be (0, 0)");

        // Set up either active tracking lead (trajectory moving J1) or Park ramp-out with residual lead.
        if is_ramp_out {
            let mut q_prev = INSPECT;
            q_prev[0] += 0.015;
            inner.tracking.record_issued(q_prev.to_vec(), vec![0.0; 7]);
            inner.pending_normal_unload = true;
        } else {
            let mut q_goal = INSPECT;
            q_goal[0] += 0.15;
            let traj = armd::TrajectoryPlanner::plan(&INSPECT, &[0.0; 7], &q_goal, &[0.0; 7], Some(1.0), &inner.config.limits).unwrap();
            inner.tracking.record_issued(INSPECT.to_vec(), vec![0.0; 7]);
            let _ = inner.safety.mark_motion_started();
            inner.active_trajectory = Some(armd::motion::ActiveTrajectory::quintic(traj, None));
        }

        // Grow workspace past prewarmed scratch capacity WITHOUT calling prepare_workspace_eval_bufs().
        let ws = inner.config.workspace.as_mut().unwrap();
        for i in 0..8 {
            ws.keepout_boxes.push(KeepoutBox {
                name: format!("rt_trk_extra_{i}"),
                min_m: [1.0, 1.0, 1.0],
                max_m: [1.1, 1.1, 1.1],
            });
        }

        let (_, tick_a, tick_f) = counted(|| inner.tick_control(0.02));
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(_)),
            "{label}: undersized RT scratch must fail closed into Fault, got {:?}",
            inner.safety.state()
        );
        assert_eq!(
            (tick_a, tick_f),
            (0, 0),
            "{label}: undersized RT scratch fail-closed tick_control must perform (0, 0) allocs/frees"
        );
    }
}

/// R2 Item 6 (Minor): RT inline quintic refusal recording its structured witness into
/// `workspace_violation` MUST perform (0, 0) heap allocations / deallocations.
#[test]
fn rt_inline_quintic_refusal_witness_zero_alloc() {
    use armd::rt_core::requests::actions::{
        install_move_joint, ActionSnapshot, JointInstallPayload, OwnerCheckCtx, PreparedActionKind,
    };

    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg.collision = None;
    cfg.power_budget = None;
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    ws.keepout_boxes.clear();
    let check_pts = ws.check_points_end_m.clone();
    let start_max_x = {
        let pose = armd::chain::ArmModel::REBOT_DEV_ARM.fk_f32(&q_start).unwrap();
        check_pts.iter().map(|c| pose.transform_point(&c.map(f64::from))[0]).fold(pose.p[0], f64::max)
    };
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;

    let mut q_neutral = q_start;
    q_neutral[6] += 0.05;
    let mut inner = SupervisorInner::new(cfg.clone());
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);
    {
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
    }
    inner.commanded_positions = q_start.to_vec();
    inner.measured_positions = q_start.to_vec();
    inner.target_positions = q_start.to_vec();
    inner.prewarm_rt_buffers();

    let s = inner.current_admission_stamp();
    let snap = ActionSnapshot::from_inner(&inner, s, Instant::now() + Duration::from_secs(5));
    let ctx = OwnerCheckCtx::for_legacy(&inner);
    let mut start_arr = [0.0f32; 8];
    let mut tgt_arr = [0.0f32; 8];
    let mut vel_arr = [1.0f32; 8];
    start_arr[..7].copy_from_slice(&q_start);
    tgt_arr[..7].copy_from_slice(&q_neutral);
    for (i, l) in cfg.limits.iter().enumerate() { vel_arr[i] = l.max_velocity; }

    // Invoke via `CoreOps::commit_action` (the exact RT thread entrypoint that returns `CommitRefusal`,
    // avoiding `install_move_joint`'s off-RT `ArmError` String formatting).
    use armd::rt_core::broker::{plan_slots, ClientId, CommandKind, Descriptor, RequestId, Stamp};
    use armd::rt_core::ops::{CommitCtx, CoreOps, PlanSlot, Primitive};
    use armd::rt_core::requests::actions::{ActionCommand, StagedAction};
    let mut slot = PlanSlot::default();
    slot.primitive = Primitive::Action(StagedAction::Joint {
        child: None,
        kind: PreparedActionKind::MoveToJoint,
        dof: 7,
        start: start_arr,
        v0: [0.0; 8],
        target: tgt_arr,
        duration_s: 0.4,
        max_velocity: vel_arr,
        power_scale: None,
        settle: false,
        approach_then: None,
        checked_joints: None,
    });
    let (mut pw, pr) = plan_slots(1, PlanSlot::default);
    let h = pw.stage(|dst| *dst = slot).unwrap();
    let slot_ref = pr.pin(h).unwrap();
    let now = Instant::now();
    let cmd = ActionCommand::InstallPrepared {
        kind: PreparedActionKind::MoveToJoint,
        plan: h,
        prepared_boot: snap.stamp.boot,
        admission_request_id: snap.stamp.admission.request_id,
        check_workspace_revision: true,
        poses_revision: snap.stamp.poses_revision,
        calibration_revision: snap.stamp.calibration_revision,
        gripper_calib_revision: snap.stamp.gripper_calib_revision,
        power_budget_revision: snap.stamp.power_budget_revision,
    };
    let d = Descriptor {
        stamp: Stamp {
            arm: inner.core.id(),
            boot: snap.stamp.boot,
            client: ClientId(0),
            client_seq: 1,
            request: RequestId(snap.stamp.admission.request_id),
            authority: Authority {
                lease_generation: 0,
                epoch: snap.stamp.admission.estop_epoch,
                stop_generation: snap.stamp.admission.stop_gen,
                config_revision: snap.stamp.config_revision,
                parent_generation: 0,
            },
            deadline: now + Duration::from_secs(5),
        },
        kind: CommandKind::Action(cmd),
    };
    let cctx = CommitCtx {
        arm: inner.core.id(),
        boot: ctx.boot,
        now,
        stop_generation: ctx.stop_generation,
    };
    let _ = (install_move_joint, JointInstallPayload::Rt {
        kind: PreparedActionKind::MoveToJoint, dof: 7, start: start_arr, v0: [0.0; 8], target: tgt_arr,
        duration_s: 0.4, max_velocity: vel_arr, power_scale: None, settle: false, approach_then: None, checked_joints: None,
    });

    let (res, a, fr) = counted(|| CoreOps::commit_action(&mut inner, &d, cmd, Some(slot_ref), &cctx));
    let refusal = res.expect_err("RT commit_staged_action must refuse neutral move");
    assert_eq!(refusal.code, "WORKSPACE_VIOLATION");
    assert_eq!((a, fr), (0, 0), "RT inline quintic refusal witness must perform (0, 0) allocs/frees");
    let detail = inner.workspace_violation.as_ref().expect("structured witness populated");
    assert_eq!(detail.constraint_id.as_deref(), Some("box_max_x"));
    assert_eq!(detail.violation_phase.as_deref(), Some("sweep"));
    assert!(detail.reason.as_deref().unwrap_or("").contains("move makes no net return toward the workspace"));
}

/// R3 Item 1 (Blocker): install-time live measured terminal re-check MUST perform (0, 0) heap
/// allocations / deallocations on RT on BOTH acceptance (within 20 mrad jitter) and refusal
/// (inward drift makes endpoint deeper than live start -> WORKSPACE_VIOLATION with witness).
#[test]
fn rt_measured_start_terminal_recheck_zero_alloc() {
    use armd::rt_core::requests::motion::{
        install_jog, prepare_motion, MotionInstallError, MotionPlanSnapshot, MotionRequest,
    };

    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg.collision = None;
    cfg.power_budget = None;
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    ws.keepout_boxes.clear();
    let check_pts = ws.check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let pose = armd::chain::ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts.iter().map(|c| pose.transform_point(&c.map(f64::from))[0]).fold(pose.p[0], f64::max)
    };
    let start_max_x = max_x_at(&q_start);
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;
    let mut q_probe = q_start;
    q_probe[1] += 0.01;
    let inward_sign = if max_x_at(&q_probe) < start_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = (start_max_x - { let mut q = q_start; q[1] += 0.01 * inward_sign; max_x_at(&q) }) / 0.01;
    let mut q_jog_2mm = q_start;
    q_jog_2mm[1] += (0.0015 / dx_per_rad) as f32 * inward_sign;
    let mut q_meas_jitter = q_start;
    q_meas_jitter[0] += 0.002;
    let mut q_meas_inward = q_start;
    q_meas_inward[1] += (0.0030 / dx_per_rad) as f32 * inward_sign;

    let mut inner = SupervisorInner::new(cfg.clone());
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    inner.safety.set_off_thread_reclaim(true);
    {
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
    }
    inner.commanded_positions = q_start.to_vec();
    inner.measured_positions = q_start.to_vec();
    inner.target_positions = q_start.to_vec();
    inner.prewarm_rt_buffers();

    let snap = MotionPlanSnapshot::from_supervisor(&inner);
    let plan = prepare_motion(&snap, &MotionRequest::jog(q_jog_2mm.to_vec(), Some(0.5), None))
        .expect("prepare jog");

    // 1. Inward drift (<= 20 mrad) makes endpoint deeper than live start -> WORKSPACE_VIOLATION, (0, 0) allocs/frees.
    inner.measured_positions.copy_from_slice(&q_meas_inward);
    let s1 = inner.current_admission_stamp();
    let (res_refuse, a1, fr1) = counted(|| install_jog(&mut inner, s1, &plan.slot, None, None, false));
    assert_eq!(res_refuse.unwrap_err(), MotionInstallError::WorkspaceViolation);
    assert_eq!((a1, fr1), (0, 0), "RT measured-start terminal refusal must perform (0, 0) allocs/frees");
    assert_eq!(
        inner.workspace_violation.as_ref().and_then(|d| d.constraint_id.as_deref()),
        Some("box_max_x")
    );

    // 2. ±2 mrad jitter leaving terminal rule valid -> ADMITTED, (0, 0) allocs/frees.
    inner.measured_positions.copy_from_slice(&q_meas_jitter);
    let s2 = inner.current_admission_stamp();
    let (res_admit, a2, fr2) = counted(|| install_jog(&mut inner, s2, &plan.slot, None, None, false));
    assert!(res_admit.is_ok(), "±2 mrad jitter must admit: {res_admit:?}");
    assert_eq!((a2, fr2), (0, 0), "RT measured-start terminal admission must perform (0, 0) allocs/frees");
}
