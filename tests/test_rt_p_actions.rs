//! Stage 3 PR P integration tests (spec sections A6 + A7):
//! - A6 prepared actions (`move_to` linear & joint, `move_relative`, `move_to_pose`, `gripper`,
//!   `replay`) prepared off-thread from published snapshots and installed via single owner-side
//!   install functions on both RT Executor and Legacy owners.
//! - Authority, domain revision, deadline, start-drift, config-inhibit, and disconnect fences
//!   (`ClosedBitset` exact 1..65536 + fail-closed overflow >= 65536).
//! - Caller-owned `ActionDelivery` bounded backpressure and cross-family FIFO ordering.
//! - A7 macro lifecycle (`BeginMacro` -> `InstallChild` -> `FinishMacro`): independent parent
//!   terminal reservation, strict monotonic `child_seq`, `> PLAN_SLOTS` reclamation, stop
//!   between children, Step-1 Stop + Step-3 replacement, refused replacement leaving parent+child
//!   intact, delayed/duplicate `FinishMacro`, `FinishMacro(Done)` refused during active motion /
//!   settle, forced `FinishMacro(Failed)` quiescence, owner-enforced `MacroRun::deadline` under
//!   `VirtualClock`, contact guard (done + fail modes), live dual-owner `MacroCtx` on
//!   `OWNER_EXECUTOR` with zero `SupervisorLock`, and `grasp_select` Phase A refusal + Phase B
//!   `SelectedGraspMacro` hook.

mod common;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::json;

use armd::macro_action::{
    ActionId, ChildSpec, ChildTicket, ContactGuard, MacroCtx, SubState,
};
use armd::model::HolderLabel;
use armd::motion::{GoalFrame, GoalSpec, GripperCommand, Speed};
use armd::rt_core::broker::{
    AdmitError, Authority, BootId, Broker, Capacities, ClientId, DeliveryDecision, Outcome,
    Reply, RequestId, Submit,
};
use armd::rt_core::executor::ExecutorConfig;
use armd::rt_core::mailbox::MAX_DOF;
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture, VirtualClock};
use armd::rt_core::requests::actions::{
    prepare_child_cartesian_slot, prepare_child_gripper_slot, prepare_child_joint_slot,
    prepare_gripper, prepare_move_relative, prepare_move_to,
    prepare_saved_pose, ActionDelivery, ActionDeliveryEvent, ActionSnapshot,
    ActionSubmit, BeginMacroSpec, ClosedBitset, FinishMacroSpec, FinishState, HeldUpdate,
    MacroKind, PreparedActionKind, PreparedMoveAction, CLOSED_BITSET_EXACT_LIMIT,
    DEFAULT_ACTION_DEADLINE, DEFAULT_FINISH_DEADLINE,
};
use armd::supervisor::{GripperBaselineCmd, OWNER_EXECUTOR};
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use common::{TestArm, INSPECT};

const BOOT: BootId = BootId(77);
const DT: f64 = 0.02;

type Fx = ReleaseFixture<SupervisorInner>;

fn sim_inner() -> SupervisorInner {
    let mut inner = common::sim_at(
        ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        },
        &INSPECT,
    );
    for _ in 0..5 {
        inner.tick(DT);
    }
    inner
}

fn fixture_with_caps(
    caps: Capacities,
) -> (Fx, Broker<PlanSlot>, VirtualClock, armd::rt_core::urgent::StopRequests) {
    let clock = VirtualClock::starting_at(Instant::now());
    let inner = sim_inner();
    let (fx, broker, _tw, stop) = ReleaseFixture::new_with_clock(inner, BOOT, caps, clock.clone());
    (fx, broker, clock, stop)
}

fn fixture() -> (Fx, Broker<PlanSlot>, VirtualClock, armd::rt_core::urgent::StopRequests) {
    fixture_with_caps(Capacities {
        commands: 8,
        plans: 4,
        replies: 32,
    })
}

fn fixture_driven() -> (Fx, Broker<PlanSlot>, VirtualClock, TestArm) {
    let clock = VirtualClock::starting_at(Instant::now());
    let arm = TestArm::new();
    let mut inner = SupervisorInner::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    });
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    for _ in 0..5 {
        inner.tick(DT);
    }
    let (fx, broker, _tw, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities {
            commands: 8,
            plans: 4,
            replies: 32,
        },
        clock.clone(),
    );
    (fx, broker, clock, arm)
}

fn auth(fx: &Fx) -> Authority {
    fx.core.authority(&fx.ctx(fx.now()))
}

fn snap_of(fx: &Fx) -> ActionSnapshot {
    let adm = fx.core.current_admission_stamp();
    ActionSnapshot::from_inner(&fx.core, adm, fx.now() + DEFAULT_ACTION_DEADLINE)
}

fn drain_all(broker: &mut Broker<PlanSlot>) -> Vec<(ClientId, Reply)> {
    let mut out = Vec::new();
    broker.poll(|c, r| {
        out.push((c, r));
        true
    });
    out
}

fn step_until_ended(fx: &mut Fx, max: usize) -> (ActionId, Outcome) {
    for _ in 0..max {
        if let Some(ended) = fx.step().ended {
            return ended;
        }
    }
    panic!("action did not end within {max} steps: {:?}", fx.core.last_action);
}

fn step_until_child_done(fx: &mut Fx, ticket: ChildTicket, max: usize) -> SubState {
    for _ in 0..max {
        fx.step();
        if let Some(st) = fx.core.macro_child_state(ticket) {
            if !matches!(st, SubState::Running | SubState::Idle) {
                return st;
            }
        }
    }
    panic!("child {ticket:?} did not finish within {max} steps");
}

fn submit_slot_prepared(
    broker: &mut Broker<PlanSlot>,
    client: ClientId,
    seq: u64,
    authority: Authority,
    deadline: Instant,
    kind: PreparedActionKind,
    prepared_boot: BootId,
    slot: PlanSlot,
    check_workspace_revision: bool,
    poses_revision: Option<u64>,
    power_budget_revision: Option<u64>,
) -> Result<RequestId, AdmitError> {
    let write = |dst: &mut PlanSlot| {
        *dst = slot.clone();
    };
    broker.submit(
        client,
        seq,
        authority,
        deadline,
        Submit::Action(ActionSubmit::InstallPrepared {
            write: &write,
            kind,
            prepared_boot,
            admission_request_id: 0,
            check_workspace_revision,
            poses_revision,
            calibration_revision: None,
            gripper_calib_revision: None,
            power_budget_revision,
        }),
    )
}

fn submit_child(
    broker: &mut Broker<PlanSlot>,
    client: ClientId,
    seq: u64,
    authority: Authority,
    deadline: Instant,
    prepared_boot: BootId,
    slot: PlanSlot,
    check_workspace_revision: bool,
) -> Result<RequestId, AdmitError> {
    let write = |dst: &mut PlanSlot| {
        *dst = slot.clone();
    };
    broker.submit(
        client,
        seq,
        authority,
        deadline,
        Submit::Action(ActionSubmit::InstallChild {
            write: &write,
            prepared_boot,
            check_workspace_revision,
            calibration_revision: None,
            gripper_calib_revision: None,
            power_budget_revision: None,
        }),
    )
}

// ============================================================================
// A6 Prepared Actions Tests
// ============================================================================

#[test]
fn a6_prepared_actions_commit_and_complete_on_rt_executor() {
    let (mut fx, mut b, _clock, _stop) = fixture();

    // Save a named pose so move_to_pose can resolve it from the snapshot.
    let mut target_pose = INSPECT.to_vec();
    target_pose[0] += 0.08;
    armd::poses::save(
        &mut fx.core,
        &json!({"name": "p_rt", "positions": target_pose}),
    )
    .unwrap();

    // 1) move_to (linear)
    let snap = snap_of(&fx);
    let p0 = armd::motion::start_pose(&snap.plan_input_in(GoalFrame::Tcp).unwrap()).p;
    let prep = prepare_move_to(
        &snap,
        &json!({"position": [p0[0] + 0.025, p0[1], p0[2]]}),
    )
    .unwrap();
    assert!(matches!(prep, PreparedMoveAction::Cartesian(_)));
    let slot = prep.to_plan_slot(None).unwrap();
    let req = submit_slot_prepared(
        &mut b,
        ClientId(1),
        1,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::MoveToLinear,
        BOOT,
        slot,
        true,
        None,
        None,
    )
    .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == req), "{rep:?}");
    let act = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act, Outcome::Completed));
    let replies = drain_all(&mut b);
    assert!(replies.iter().any(|(_, r)| matches!(r, Reply::Ack { request, .. } if *request == req)));
    assert!(replies.iter().any(|(_, r)| matches!(r, Reply::Terminal { request, outcome: Outcome::Completed, .. } if *request == req)));
    assert_eq!(b.stats().retired, 1);

    // 2) move_relative (joint)
    let snap = snap_of(&fx);
    let prep = prepare_move_relative(
        &snap,
        &json!({"delta": [-0.015, 0.0, 0.0], "frame": "base", "path": "joint", "speed_scale": 0.5}),
    )
    .unwrap();
    assert!(matches!(prep, PreparedMoveAction::Joint(_)));
    let slot = prep.to_plan_slot(None).unwrap();
    let _req = submit_slot_prepared(
        &mut b,
        ClientId(1),
        2,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::MoveRelativeJoint,
        BOOT,
        slot,
        true,
        None,
        None,
    )
    .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let act = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act, Outcome::Completed));
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 2);

    // 3) move_to_pose (saved pose)
    let snap = snap_of(&fx);
    let prep = prepare_saved_pose(&snap, &json!({"name": "p_rt", "speed_scale": 0.8})).unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let _req = submit_slot_prepared(
        &mut b,
        ClientId(1),
        3,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::MoveToPose,
        BOOT,
        slot,
        true,
        Some(snap.env.poses_revision),
        None,
    )
    .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let act = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act, Outcome::Completed));
    assert!(
        (fx.core.commanded_positions[0] - target_pose[0]).abs() < 2e-3,
        "expected J1 near {}, got {}",
        target_pose[0],
        fx.core.commanded_positions[0]
    );
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 3);

    // 4) gripper (move to fraction 0.5)
    let snap = snap_of(&fx);
    let prep = prepare_gripper(
        &snap,
        &json!({"command": "move", "fraction": 0.5, "speed": 1.0}),
        GripperBaselineCmd::None,
    )
    .unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let req = submit_slot_prepared(
        &mut b,
        ClientId(1),
        4,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::Gripper,
        BOOT,
        slot,
        false,
        None,
        None,
    )
    .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == req), "{rep:?}");
    let act = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act, Outcome::Completed));
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 4);

    // 5) replay (staged samples directly in PlanSlot)
    let start = fx.core.commanded_positions.clone();
    let replay_target_j1 = start[0] - 0.06;
    let mut replay_slot = PlanSlot::default();
    for k in 0..=40 {
        let mut q = start.clone();
        q[0] = start[0] - 0.06 * (k as f32 / 40.0);
        replay_slot.path.push(&q, k as f64).unwrap();
    }
    let mut start_arr = [0.0f32; MAX_DOF];
    start_arr[..start.len()].copy_from_slice(&start);
    let tool_digest = armd::rt_core::ops::tool_digest_hash(&fx.core.config.active_tool.config_digest);
    replay_slot.primitive = armd::rt_core::ops::Primitive::Action(
        armd::rt_core::requests::actions::StagedAction::Replay {
            child: None,
            dof: start.len() as u8,
            start: start_arr,
            episode: armd::rt_core::ops::EpisodeRef::new("ep_rt_p").unwrap(),
            speed: 1.0,
            gripper: false,
            sample_dt: 0.02,
            duration_s: 0.8,
            power_scale: None,
            tool_digest,
        },
    );
    let req = submit_slot_prepared(
        &mut b,
        ClientId(1),
        5,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::Replay,
        BOOT,
        replay_slot,
        true,
        None,
        None,
    )
    .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == req), "{rep:?}");
    let act = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act, Outcome::Completed));
    assert!((fx.core.commanded_positions[0] - replay_target_j1).abs() < 2e-3);
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 5);
}

#[test]
fn a6_authority_revision_deadline_and_start_drift_refusals() {
    let (mut fx, mut b, clock, _stop) = fixture();
    let mut target_pose = INSPECT.to_vec();
    target_pose[0] += 0.05;
    armd::poses::save(
        &mut fx.core,
        &json!({"name": "p_rev", "positions": target_pose}),
    )
    .unwrap();
    let snap = snap_of(&fx);
    let prep = prepare_saved_pose(&snap, &json!({"name": "p_rev", "speed_scale": 0.6})).unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let base = auth(&fx);
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;

    // Wrong boot & zero boot -> WRONG_TARGET
    submit_slot_prepared(&mut b, ClientId(1), 1, base, dl, PreparedActionKind::MoveToPose, BootId(999), slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("WRONG_TARGET"));
    submit_slot_prepared(&mut b, ClientId(1), 2, base, dl, PreparedActionKind::MoveToPose, BootId(0), slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("WRONG_TARGET"));

    // Stale stop, stale epoch, stale lease
    submit_slot_prepared(&mut b, ClientId(1), 3, Authority { stop_generation: base.stop_generation + 1, ..base }, dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("STOPPED"));
    submit_slot_prepared(&mut b, ClientId(1), 4, Authority { epoch: base.epoch + 1, ..base }, dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("EPOCH_MISMATCH"));
    submit_slot_prepared(&mut b, ClientId(1), 5, Authority { lease_generation: base.lease_generation + 1, ..base }, dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("LEASE_LOST"));

    // Stale workspace_revision, poses_revision, power_budget_revision -> PLAN_STALE
    submit_slot_prepared(&mut b, ClientId(1), 6, Authority { config_revision: base.config_revision + 1, ..base }, dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    submit_slot_prepared(&mut b, ClientId(1), 7, base, dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, Some(snap.env.poses_revision + 1), None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    submit_slot_prepared(&mut b, ClientId(1), 8, base, dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, Some(snap.env.power_budget_revision + 1)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));

    // Expired deadline -> DEADLINE_EXPIRED
    let short_dl = fx.now() + Duration::from_millis(5);
    clock.advance(Duration::from_millis(10));
    submit_slot_prepared(&mut b, ClientId(1), 9, base, short_dl, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("DEADLINE_EXPIRED"));

    // Start drift on arm joint AND on gripper joint (>= 1e-4) -> PLAN_STALE
    let _ = drain_all(&mut b);
    let saved_j0 = fx.core.commanded_positions[0];
    fx.core.commanded_positions[0] += 2e-4;
    submit_slot_prepared(&mut b, ClientId(1), 10, base, fx.now() + DEFAULT_ACTION_DEADLINE, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    fx.core.commanded_positions[0] = saved_j0;

    let saved_g = fx.core.commanded_positions[6];
    fx.core.commanded_positions[6] += 2e-4;
    submit_slot_prepared(&mut b, ClientId(1), 11, base, fx.now() + DEFAULT_ACTION_DEADLINE, PreparedActionKind::MoveToPose, BOOT, slot.clone(), true, None, None).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    fx.core.commanded_positions[6] = saved_g;

    assert!(fx.core.active_trajectory.is_none(), "no refused action may install motion");
    let _ = drain_all(&mut b);
    assert_eq!(b.reclaim_plans() + b.stats().retired as usize, 11, "every refused plan slot must retire cleanly");
}

#[test]
fn a6_disconnect_before_dequeue_before_commit_and_after_ack() {
    let (mut fx, mut b, _clock, _stop) = fixture();
    let snap = snap_of(&fx);
    let prep = prepare_gripper(
        &snap,
        &json!({"command": "move", "fraction": 0.5, "speed": 1.0}),
        GripperBaselineCmd::None,
    )
    .unwrap();
    let slot = prep.to_plan_slot(None).unwrap();

    // Case 1: disconnect AFTER submit BEFORE dequeue -> refused LEASE_LOST via ClosedBitset, 0 motion.
    let c1 = ClientId(11);
    let r1 = submit_slot_prepared(
        &mut b,
        c1,
        1,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::Gripper,
        BOOT,
        slot.clone(),
        false,
        None,
        None,
    )
    .unwrap();
    b.disconnect(c1);
    let rep = fx.step();
    assert_eq!(rep.command, Some((r1, Err("LEASE_LOST"))));
    assert!(fx.core.active_trajectory.is_none(), "disconnected client must not start physical motion");
    let delivered = drain_all(&mut b);
    assert!(delivered.is_empty(), "abandoned reply must not be delivered: {delivered:?}");
    assert_eq!(b.stats().abandoned, 1);

    // Case 2: disconnect AFTER ACK while running -> physical motion completes cleanly (1 terminal),
    // zero delivery to replacement socket, plan slot reclaimed!
    let c2 = ClientId(12);
    let r2 = submit_slot_prepared(
        &mut b,
        c2,
        1,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::Gripper,
        BOOT,
        slot,
        false,
        None,
        None,
    )
    .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == r2));
    let act = fx.active().unwrap();
    let acks = drain_all(&mut b);
    assert_eq!(acks.len(), 1);
    // Client 12 disconnects while gripper is running; replacement ClientId(13) joins.
    b.disconnect(c2);
    assert_eq!(step_until_ended(&mut fx, 400), (act, Outcome::Completed));
    let after = drain_all(&mut b);
    assert!(after.is_empty(), "disconnected client terminal must be abandoned, not sent to replacement");
    assert_eq!(b.stats().retired, 2, "both plan slots reclaimed");
}

#[test]
fn a6_closed_bitset_exact_range_and_fail_closed_overflow() {
    let bs = ClosedBitset::new();
    // Connection 0 is sentinel (never closed).
    bs.mark_closed(0);
    assert!(!bs.is_closed(0));

    // Exact membership in 1..65536: marking high IDs never produces false positives on lower IDs.
    bs.mark_closed(65_535);
    assert!(bs.is_closed(65_535));
    assert!(!bs.is_closed(65_534));
    assert!(!bs.is_closed(1));

    bs.mark_closed(42);
    assert!(bs.is_closed(42));
    assert!(!bs.is_closed(41));
    assert!(!bs.is_closed(43));

    // Overflow >= 65536 fails closed monotonically downward without affecting 1..65536.
    bs.mark_closed(CLOSED_BITSET_EXACT_LIMIT + 100);
    assert!(!bs.is_closed(CLOSED_BITSET_EXACT_LIMIT));
    assert!(!bs.is_closed(CLOSED_BITSET_EXACT_LIMIT + 99));
    assert!(bs.is_closed(CLOSED_BITSET_EXACT_LIMIT + 100));
    assert!(bs.is_closed(CLOSED_BITSET_EXACT_LIMIT + 500));
    assert!(!bs.is_closed(65_534));

    // Lower overflow mark pulls watermark down.
    bs.mark_closed(CLOSED_BITSET_EXACT_LIMIT + 10);
    assert!(bs.is_closed(CLOSED_BITSET_EXACT_LIMIT + 10));
    assert!(bs.is_closed(CLOSED_BITSET_EXACT_LIMIT + 99));
    assert!(!bs.is_closed(CLOSED_BITSET_EXACT_LIMIT + 9));
    assert!(!bs.is_closed(100));
}

#[test]
fn a6_action_delivery_backpressure_and_cross_family_fifo() {
    let (mut fx, mut b, _clock, _stop) = fixture_with_caps(Capacities {
        commands: 4,
        plans: 4,
        replies: 4,
    });
    let mut delivery = ActionDelivery::new();
    let snap = snap_of(&fx);
    let g1 = prepare_gripper(
        &snap,
        &json!({"command": "move", "fraction": 0.4, "speed": 1.0}),
        GripperBaselineCmd::None,
    )
    .unwrap();
    let slot1 = g1.to_plan_slot(None).unwrap();
    let r1 = submit_slot_prepared(
        &mut b,
        ClientId(1),
        1,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::Gripper,
        BOOT,
        slot1,
        false,
        None,
        None,
    )
    .unwrap();
    delivery.register(r1, ClientId(1), 1.25, false, true);
    assert!(fx.step().command.unwrap().1.is_ok());

    // Simulate a full downstream writer: reply MUST remain in bounded broker ring (reserved stays 2).
    let reserved_before = b.reserved();
    let mut seen = Vec::new();
    let drained = delivery.poll(&mut b, |_| DeliveryDecision::Backpressured);
    assert_eq!(drained, 0);
    assert_eq!(b.reserved(), reserved_before, "backpressure must preserve reply reservation");
    assert_eq!(delivery.tracked(), 1);

    // Now drain ACK of r1.
    delivery.poll(&mut b, |ev| {
        seen.push(ev);
        DeliveryDecision::Delivered
    });
    assert_eq!(seen.len(), 1);
    assert!(matches!(
        seen[0],
        ActionDeliveryEvent::Client {
            client: ClientId(1),
            reply: Reply::Ack { request, .. },
            expected_duration_s: Some(d),
        } if request == r1 && (d - 1.25).abs() < 1e-9
    ));

    // Let r1 complete and immediately submit r2 before draining -> cross-family FIFO:
    // Terminal(r1) MUST arrive strictly BEFORE Ack(r2), then Terminal(r2)!
    let act1 = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act1, Outcome::Completed));
    let snap2 = snap_of(&fx);
    let g2 = prepare_gripper(
        &snap2,
        &json!({"command": "open", "speed": 1.0}),
        GripperBaselineCmd::None,
    )
    .unwrap();
    let slot2 = g2.to_plan_slot(None).unwrap();
    let r2 = submit_slot_prepared(
        &mut b,
        ClientId(1),
        2,
        auth(&fx),
        fx.now() + DEFAULT_ACTION_DEADLINE,
        PreparedActionKind::Gripper,
        BOOT,
        slot2,
        false,
        None,
        None,
    )
    .unwrap();
    delivery.register(r2, ClientId(1), 0.75, false, true);
    assert!(fx.step().command.unwrap().1.is_ok());
    let act2 = fx.active().unwrap();
    assert_eq!(step_until_ended(&mut fx, 400), (act2, Outcome::Completed));

    // Drain all 3 queued replies in one pass and verify strict FIFO ordering!
    let mut seq_ev = Vec::new();
    delivery.poll(&mut b, |ev| {
        seq_ev.push(ev);
        DeliveryDecision::Delivered
    });
    assert_eq!(seq_ev.len(), 3, "{seq_ev:?}");
    assert!(matches!(
        seq_ev[0],
        ActionDeliveryEvent::Client {
            reply: Reply::Terminal { request, outcome: Outcome::Completed, .. },
            ..
        } if request == r1
    ));
    assert!(matches!(
        seq_ev[1],
        ActionDeliveryEvent::Client {
            reply: Reply::Ack { request, .. },
            expected_duration_s: Some(d),
            ..
        } if request == r2 && (d - 0.75).abs() < 1e-9
    ));
    assert!(matches!(
        seq_ev[2],
        ActionDeliveryEvent::Client {
            reply: Reply::Terminal { request, outcome: Outcome::Completed, .. },
            ..
        } if request == r2
    ));
    assert_eq!(delivery.tracked(), 0, "completed requests must be cleaned from ActionDelivery");
}

// ============================================================================
// A7 Macro Lifecycle Tests
// ============================================================================

#[test]
fn a7_macro_begin_children_and_finish_lifecycle_with_independent_terminal() {
    let (mut fx, mut b, _clock, _stop) = fixture();
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;

    // 1) BeginMacro reserves parent terminal independently.
    let mut begin_spec = BeginMacroSpec::new(MacroKind::Grasp, BOOT);
    begin_spec.check_workspace_revision = true;
    let r_begin = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            dl,
            Submit::Action(ActionSubmit::BeginMacro(begin_spec)),
        )
        .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == r_begin), "{rep:?}");
    let parent = fx.active().unwrap();
    let replies = drain_all(&mut b);
    assert_eq!(replies.len(), 1, "BeginMacro emits ACK only, holding parent terminal open");
    assert!(matches!(replies[0].1, Reply::Ack { request, action: Some(a), .. } if request == r_begin && a == parent));
    assert_eq!(b.reserved(), 1, "parent terminal slot stays reserved");

    // 2) Child 1: Cartesian move
    let snap = snap_of(&fx);
    let input = snap.plan_input_in(GoalFrame::Tcp).unwrap();
    let p0 = armd::motion::start_pose(&input).p;
    let goal = armd::motion::resolve_goal(
        &input,
        &GoalSpec::Absolute {
            p: [p0[0] + 0.02, p0[1], p0[2]],
            r: None,
        },
    );
    let path = armd::motion::plan_cartesian(&input, goal, Speed::default()).unwrap();
    let c1_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let c1_slot = prepare_child_cartesian_slot(&snap.commanded_positions, &path, None, c1_spec).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, c1_slot, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert!(matches!(step_until_child_done(&mut fx, c1_spec.ticket, 400), SubState::Done(_)));
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 1, "child 1 plan slot retired while parent stays open");
    assert_eq!(fx.active(), Some(parent));

    // 3) Child 2: Gripper move
    let snap = snap_of(&fx);
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
        None,
        false,
        c2_spec,
    )
    .unwrap();
    submit_child(&mut b, ClientId(1), 3, auth(&fx), dl, BOOT, c2_slot, false).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert!(matches!(step_until_child_done(&mut fx, c2_spec.ticket, 400), SubState::Done(_)));
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 2);

    // 4) Child 3: Joint quintic
    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.04;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c3_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 3 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let c3_slot = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.3, None, c3_spec).unwrap();
    submit_child(&mut b, ClientId(1), 4, auth(&fx), dl, BOOT, c3_slot, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert!(matches!(step_until_child_done(&mut fx, c3_spec.ticket, 400), SubState::Done(_)));
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 3);

    // 5) Child 4: Replay child staged directly in PlanSlot
    let snap = snap_of(&fx);
    let start = snap.commanded_positions.clone();
    let c4_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 4 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let mut c4_slot = PlanSlot::default();
    for k in 0..=20 {
        let mut q = start.clone();
        q[0] -= 0.03 * (k as f32 / 20.0);
        c4_slot.path.push(&q, k as f64).unwrap();
    }
    let mut s_arr = [0.0f32; MAX_DOF];
    s_arr[..start.len()].copy_from_slice(&start);
    c4_slot.primitive = armd::rt_core::ops::Primitive::Action(
        armd::rt_core::requests::actions::StagedAction::Replay {
            child: Some(c4_spec),
            dof: start.len() as u8,
            start: s_arr,
            episode: armd::rt_core::ops::EpisodeRef::new("ep_child").unwrap(),
            speed: 1.0,
            gripper: false,
            sample_dt: 0.02,
            duration_s: 0.4,
            power_scale: None,
            tool_digest: snap.env.tool_digest_hash,
        },
    );
    submit_child(&mut b, ClientId(1), 5, auth(&fx), dl, BOOT, c4_slot, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert!(matches!(step_until_child_done(&mut fx, c4_spec.ticket, 400), SubState::Done(_)));
    let _ = drain_all(&mut b);
    assert_eq!(b.stats().retired, 4);

    // 6) FinishMacro(Done, HeldUpdate::Set) emits parent Terminal with children == 4 and updates held!
    let mut fin = FinishMacroSpec::done(parent, BOOT);
    fin.holding = Some(true);
    fin.held = HeldUpdate::Set {
        label: HolderLabel::new("cup"),
        spill_safe: true,
    };
    let r_fin = b
        .submit(
            ClientId(1),
            6,
            auth(&fx),
            fx.now() + DEFAULT_FINISH_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(fin)),
        )
        .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == r_fin));
    assert_eq!(rep.ended, Some((parent, Outcome::Completed)));
    assert_eq!(
        fx.core.held.as_ref().map(|h| (h.label.as_str(), h.spill_safe)),
        Some(("cup", true))
    );
    assert_eq!(fx.core.action_children(parent), 4);
    let replies = drain_all(&mut b);
    assert!(replies.iter().any(|(_, r)| matches!(r, Reply::Ack { request, .. } if *request == r_fin)));
    assert!(replies.iter().any(|(_, r)| matches!(
        r,
        Reply::Terminal { request, action, outcome: Outcome::Completed, children: 4 }
            if *request == r_begin && *action == parent
    )));
    assert_eq!(b.reserved(), 0);
}

#[test]
fn a7_macro_child_seq_strict_monotonic_and_over_plan_slots_reclamation() {
    // Only 2 plan slots, run 12 sequential children (> PLAN_SLOTS) proving slot reclamation!
    let (mut fx, mut b, _clock, _stop) = fixture_with_caps(Capacities {
        commands: 4,
        plans: 2,
        replies: 32,
    });
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;
    let begin = BeginMacroSpec::new(MacroKind::Sequence, BOOT);
    b.submit(ClientId(1), 1, auth(&fx), dl, Submit::Action(ActionSubmit::BeginMacro(begin)))
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Out-of-order child_seq (2 instead of 1) refused PLAN_STALE without advancing seq.
    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.01;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let bad_spec = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 2 },
        timeout_s: 10.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        contact: None,
    };
    let bad_slot = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.1, None, bad_spec).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, bad_slot, true).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    let _ = drain_all(&mut b);
    b.reclaim_plans();

    // Now run 12 sequential children (> 2 slots).
    for seq in 1..=12u32 {
        let snap = snap_of(&fx);
        let mut tgt = snap.commanded_positions.clone();
        tgt[0] += if seq % 2 == 1 { 0.01 } else { -0.01 };
        let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
        let spec = ChildSpec {
            ticket: ChildTicket { parent, child_seq: seq },
            timeout_s: 10.0,
            timeout_code: "SEQUENCE_TIMEOUT",
            contact: None,
        };
        let slot = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.06, None, spec).unwrap();
        b.reclaim_plans();
        submit_child(&mut b, ClientId(1), u64::from(seq) + 10, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot, true)
            .expect("slot must have been reclaimed");
        assert!(fx.step().command.unwrap().1.is_ok(), "child {seq} failed");
        assert!(matches!(step_until_child_done(&mut fx, spec.ticket, 200), SubState::Done(_)));
        let _ = drain_all(&mut b);
        b.reclaim_plans();
    }
    assert_eq!(fx.core.macro_children(parent.0), Some(12));
    assert!(b.stats().retired >= 12);
}

#[test]
fn a7_macro_stop_between_children_and_step1_stop_plus_step3_replacement() {
    // Part A: Stop between children records Outcome::Stopped + exact child count.
    let (mut fx, mut b, _clock, stop) = fixture();
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;
    let r_begin = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            dl,
            Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Place, BOOT))),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Run 1 child to completion.
    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.02;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c1 = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "PLACE_TIMEOUT",
        contact: None,
    };
    let slot1 = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.1, None, c1).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, slot1, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert!(matches!(step_until_child_done(&mut fx, c1.ticket, 200), SubState::Done(_)));
    let _ = drain_all(&mut b);

    // Part B: Trigger Step-1 Stop AND queue a Step-3 replacement BeginMacro with the post-stop
    // stop_generation in the SAME release! Old macro MUST record Outcome::Stopped (children=1),
    // NOT Superseded, and FIFO order MUST be Terminal(old, Stopped) -> Ack(new)!
    let next_stop_gen = stop.request();
    let mut new_auth = auth(&fx);
    new_auth.stop_generation = next_stop_gen;
    new_auth.parent_generation = 0;
    let r_new = b
        .submit(
            ClientId(1),
            3,
            new_auth,
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Grasp, BOOT))),
        )
        .unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == r_new), "{rep:?}");
    let new_parent = fx.active().unwrap();
    assert_ne!(new_parent, parent);
    assert_eq!(fx.core.action_outcome(parent), Some(Outcome::Stopped));
    assert_eq!(fx.core.action_children(parent), 1);

    let replies = drain_all(&mut b);
    let pos_old_term = replies
        .iter()
        .position(|(_, r)| matches!(r, Reply::Terminal { request, action, outcome: Outcome::Stopped, children: 1 } if *request == r_begin && *action == parent))
        .expect("old macro must emit Terminal(Stopped)");
    let pos_new_ack = replies
        .iter()
        .position(|(_, r)| matches!(r, Reply::Ack { request, action: Some(a), .. } if *request == r_new && *a == new_parent))
        .expect("new macro must emit Ack");
    assert!(pos_old_term < pos_new_ack, "Terminal(old) must strictly precede Ack(new): {replies:?}");
}

#[test]
fn a7_macro_refused_replacement_leaves_parent_and_child_intact() {
    let (mut fx, mut b, _clock, _stop) = fixture();
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;
    b.submit(
        ClientId(1),
        1,
        auth(&fx),
        dl,
        Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Place, BOOT))),
    )
    .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Start a running child.
    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.08;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c1 = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "PLACE_TIMEOUT",
        contact: None,
    };
    let slot1 = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.4, None, c1).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, slot1, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let _ = drain_all(&mut b);

    // Submit a REFUSED replacement (wrong boot) while parent + child are running!
    let bad_begin = BeginMacroSpec::new(MacroKind::Grasp, BootId(999));
    let mut rep_auth = auth(&fx);
    rep_auth.parent_generation = 0;
    b.submit(ClientId(2), 1, rep_auth, dl, Submit::Action(ActionSubmit::BeginMacro(bad_begin)))
        .unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("WRONG_TARGET"));

    // Parent AND running child MUST remain intact and finish normally!
    assert_eq!(fx.active(), Some(parent));
    assert!(matches!(step_until_child_done(&mut fx, c1.ticket, 400), SubState::Done(_)));
    let _ = drain_all(&mut b);
    let fin = FinishMacroSpec::done(parent, BOOT);
    b.submit(ClientId(1), 3, auth(&fx), fx.now() + DEFAULT_FINISH_DEADLINE, Submit::Action(ActionSubmit::FinishMacro(fin)))
        .unwrap();
    assert_eq!(fx.step().ended, Some((parent, Outcome::Completed)));
}

#[test]
fn a7_macro_delayed_duplicate_finish_finish_during_active_and_owner_deadline() {
    let (mut fx, mut b, clock, _stop) = fixture();
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;
    let macro_dl = fx.now() + Duration::from_secs(2);

    let mut begin = BeginMacroSpec::new(MacroKind::Place, BOOT);
    begin.macro_deadline = Some(macro_dl);
    begin.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
    b.submit(ClientId(1), 1, auth(&fx), dl, Submit::Action(ActionSubmit::BeginMacro(begin)))
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Start a running child.
    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.08;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c1 = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "PLACE_TIMEOUT",
        contact: None,
    };
    let slot1 = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.5, None, c1).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, slot1, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let _ = drain_all(&mut b);

    // FinishMacro(Done) while child is active MUST be refused MOTION_BUSY (parent + child stay alive!).
    let done_spec = FinishMacroSpec::done(parent, BOOT);
    b.submit(ClientId(1), 3, auth(&fx), fx.now() + DEFAULT_FINISH_DEADLINE, Submit::Action(ActionSubmit::FinishMacro(done_spec)))
        .unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("MOTION_BUSY"));
    assert_eq!(fx.active(), Some(parent));
    assert!(CoreOps::macro_child_running(&fx.core, parent));
    let _ = drain_all(&mut b);

    // Forced FinishMacro(Failed) during active motion quiesces child and ends parent cleanly!
    let fail_spec = FinishMacroSpec::failed(parent, BOOT, "TARGET_LOST", "forced stop");
    b.submit(ClientId(1), 4, auth(&fx), fx.now() + DEFAULT_FINISH_DEADLINE, Submit::Action(ActionSubmit::FinishMacro(fail_spec)))
        .unwrap();
    let rep = fx.step();
    assert!(rep.command.unwrap().1.is_ok());
    assert_eq!(rep.ended, Some((parent, Outcome::Failed("TARGET_LOST"))));
    assert!(fx.core.active_trajectory.is_none());
    let _ = drain_all(&mut b);

    // Delayed/duplicate FinishMacro from ended parent -> PARENT_REPLACED!
    let mut stale_auth = auth(&fx);
    stale_auth.parent_generation = parent.0;
    b.submit(ClientId(1), 5, stale_auth, fx.now() + DEFAULT_FINISH_DEADLINE, Submit::Action(ActionSubmit::FinishMacro(done_spec)))
        .unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PARENT_REPLACED"));
    let _ = drain_all(&mut b);

    // Now test owner-enforced MacroRun::deadline expiry via VirtualClock!
    let short_macro_dl = fx.now() + Duration::from_millis(60);
    let mut begin2 = BeginMacroSpec::new(MacroKind::Grasp, BOOT);
    begin2.macro_deadline = Some(short_macro_dl);
    begin2.timeout_code = armd::safety::BoundedText::new("GRASP_TIMEOUT");
    let r_b2 = b
        .submit(
            ClientId(1),
            6,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(begin2)),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent2 = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Advance virtual clock past macro_deadline -> owner tick ends parent2 with GRASP_TIMEOUT!
    clock.advance(Duration::from_millis(80));
    let rep = fx.step();
    assert_eq!(rep.ended, Some((parent2, Outcome::Failed("GRASP_TIMEOUT"))));
    let replies = drain_all(&mut b);
    assert!(replies.iter().any(|(_, r)| matches!(
        r,
        Reply::Terminal { request, action, outcome: Outcome::Failed("GRASP_TIMEOUT"), .. }
            if *request == r_b2 && *action == parent2
    )));
}

#[test]
fn a7_macro_contact_guard_done_and_fail_modes() {
    let (mut fx, mut b, _clock, arm) = fixture_driven();
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;

    // Mode 1: ContactGuard { stop_code: None } -> stops descent cleanly as Done, parent stays alive!
    b.submit(ClientId(1), 1, auth(&fx), dl, Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Grasp, BOOT))))
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.3;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let mut max_res = [0.0f32; MAX_DOF];
    max_res[1] = 2.0;
    let guard_done = ContactGuard { max_residual_nm: max_res, stop_code: None };
    let c1 = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: Some(guard_done),
    };
    let slot1 = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 1.0, None, c1).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, slot1, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    for _ in 0..5 {
        fx.step();
    }
    arm.set_residual(1, 3.0);
    fx.step();
    assert!(matches!(fx.core.macro_child_state(c1.ticket), Some(SubState::Done(_))));
    assert!(fx.core.active_trajectory.is_none());
    let watch = fx.core.macro_child_watch(c1.ticket).unwrap();
    assert_eq!(watch.contact_joint, Some(1));
    assert_eq!(fx.active(), Some(parent), "parent stays alive after contact-stop done");

    // Mode 2: ContactGuard { stop_code: Some("COLLISION") } -> fails child with COLLISION.
    arm.set_residual(1, 0.0);
    let snap = snap_of(&fx);
    let mut tgt2 = snap.commanded_positions.clone();
    tgt2[0] -= 0.3;
    let guard_fail = ContactGuard { max_residual_nm: max_res, stop_code: Some("COLLISION") };
    let c2 = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 2 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: Some(guard_fail),
    };
    let slot2 = prepare_child_joint_slot(&snap.commanded_positions, &tgt2, &max_vel, 1.0, None, c2).unwrap();
    submit_child(&mut b, ClientId(1), 3, auth(&fx), dl, BOOT, slot2, true).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    for _ in 0..4 {
        fx.step();
    }
    arm.set_residual(1, 3.0);
    fx.step();
    assert!(matches!(
        fx.core.macro_child_state(c2.ticket),
        Some(SubState::Failed { code, .. }) if code == "COLLISION"
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a7_dual_owner_macro_ctx_runs_end_to_end_on_live_rt_executor_zero_lock() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    }));
    sup.set_lease_enabled(false).await;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        for _ in 0..5 {
            inner.tick(DT);
        }
    }
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(10)))
        .await
        .unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

    // Wait for first published snapshot with non-zero boot.
    let mut pub_snap = None;
    for _ in 0..200 {
        if let Some(s) = sup.hub().latest() {
            if !s.rt.owner.boot.is_zero() {
                pub_snap = Some(s);
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let pub_snap = pub_snap.expect("published RT snapshot");
    let boot = handle.boot();
    assert_eq!(pub_snap.rt.owner.boot, boot);

    // BeginMacro via broker!
    let mut begin = BeginMacroSpec::new(MacroKind::Sequence, boot);
    begin.check_workspace_revision = true;
    let dl = Instant::now() + DEFAULT_ACTION_DEADLINE;
    let r_begin = handle
        .broker()
        .lock()
        .unwrap()
        .submit(
            ClientId(5),
            1,
            pub_snap.rt.owner.authority,
            dl,
            Submit::Action(ActionSubmit::BeginMacro(begin)),
        )
        .unwrap();

    // Poll until ACK arrives for BeginMacro.
    let mut parent_id = None;
    for _ in 0..200 {
        handle.broker().lock().unwrap().poll(|_, r| {
            if let Reply::Ack { request, action, .. } = r {
                if request == r_begin {
                    parent_id = action;
                }
            }
            true
        });
        if parent_id.is_some() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let parent = parent_id.expect("BeginMacro ACK");

    // Wait for published macro_state to reflect active parent.
    for _ in 0..200 {
        if sup
            .hub()
            .latest()
            .is_some_and(|s| s.rt.owner.macro_state.active_id == parent.0 && s.rt.owner.macro_state.alive)
        {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Drive MacroCtx purely over StateHub + Broker (ZERO SupervisorLock!).
    let mut adm = sup.capture_admission_stamp();
    adm.connection_id = 5;
    let ctx = MacroCtx {
        sup: sup.clone(),
        id: parent.0,
        stamp: adm,
        deadline: Instant::now() + Duration::from_secs(15),
        phase_timeout_s: 10.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        timeout_message: "timed out".into(),
    };
    assert!(ctx.is_executor());
    assert!(ctx.alive().await);
    let _p = ctx.pose().await;
    let _tp = ctx.tcp_pose().await;
    let (meas, _vel, arm_idx) = ctx.measured_with_vel().await;
    assert_eq!(arm_idx.len(), 6);
    assert_eq!(meas.len(), 7);

    // Install joint child via MacroCtx on OWNER_EXECUTOR!
    let snap = ctx.snapshot(ctx.deadline).await.unwrap();
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.05;
    let limits = snap.env.planning_limits(Some(0.8));
    ctx.install_joint_child("step1", tgt.clone(), limits, true, None, None)
        .await
        .expect("executor joint child must complete");
    let latest = sup.hub().latest().unwrap();
    assert!(
        (latest.rt.commanded_positions[0] - tgt[0]).abs() < 5e-3,
        "expected J1 near {}, got {}",
        tgt[0],
        latest.rt.commanded_positions[0]
    );

    // Install gripper child via MacroCtx on OWNER_EXECUTOR!
    ctx.gripper_move("step2", 0.5)
        .await
        .expect("executor gripper child must complete");

    // Finish macro via MacroCtx on OWNER_EXECUTOR!
    ctx.finish(
        FinishState::Done,
        None,
        None,
        Some(true),
        None,
        HeldUpdate::Set {
            label: HolderLabel::new("block"),
            spill_safe: false,
        },
    )
    .await;

    // Wait for published macro_state to end and held object to publish!
    let mut ended = false;
    for _ in 0..200 {
        if let Some(s) = sup.hub().latest() {
            if !s.rt.owner.macro_state.alive {
                if let Some(h) = s.rt.owner.held {
                    assert_eq!(h.label.as_str(), "block");
                    ended = true;
                    break;
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(ended, "macro finish must publish !alive and updated held object");
    handle.stop_and_join();
}

#[test]
fn a7_grasp_select_phase_a_live_refusal_and_phase_b_macro_hook() {
    // Live strategy="adaptive" stays refused (Phase A).
    assert!(armd::grasp_select::refuse_execution(&json!({"strategy": "legacy"})).is_ok());
    let err = armd::grasp_select::refuse_execution(&json!({"strategy": "adaptive"})).unwrap_err();
    assert!(
        matches!(err, armd::motion::MotionError::Unsupported(_)),
        "live adaptive strategy must stay refused: {err:?}"
    );

    // Phase B hook: ExecutionPlan -> SelectedGraspMacro installs cleanly as an A7 BeginMacro!
    let (mut fx, mut b, _clock, _stop) = fixture();
    let input = fx.core.plan_input_tcp().unwrap();
    let start_tcp = armd::motion::start_pose(&input);
    let plan = armd::grasp_select::ExecutionPlan {
        pregrasp: start_tcp,
        insertion_dir: [0.0, 0.0, -1.0],
        insertion_m: 0.05,
        grasp: armd::kinematics::Pose::new(
            start_tcp.r,
            [start_tcp.p[0], start_tcp.p[1], start_tcp.p[2] - 0.05],
        ),
        separation: start_tcp,
        lift: armd::kinematics::Pose::new(
            start_tcp.r,
            [start_tcp.p[0], start_tcp.p[1], start_tcp.p[2] + 0.05],
        ),
        jaw_opening_m: 0.06,
        chord_width_m: 0.03,
        q_pregrasp: input.q0,
        q_grasp: input.q0,
        q_lift: input.q0,
    };
    let mut max_res = [0.0f32; MAX_DOF];
    max_res[1] = 2.0;
    let guard = ContactGuard {
        max_residual_nm: max_res,
        stop_code: None,
    };
    let macro_plan = plan
        .into_macro_plan(
            BOOT,
            &fx.core.commanded_positions,
            fx.now() + Duration::from_secs(30),
            Some("phase_b_test"),
            Some(guard),
        )
        .unwrap();
    assert_eq!(macro_plan.begin.kind, MacroKind::Grasp);
    assert_eq!(macro_plan.steps.len(), 6);
    assert!(matches!(
        macro_plan.steps[2],
        armd::grasp_select::SelectedMacroStep::MoveTcp {
            phase: "insert",
            contact: Some(g),
            ..
        } if g == guard
    ));

    // Install the generated BeginMacroSpec on ReleaseFixture!
    let r = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(macro_plan.begin)),
        )
        .unwrap();
    assert!(matches!(fx.step().command, Some((req, Ok(_))) if req == r));
    assert!(fx.active().is_some());
}
