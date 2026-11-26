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
mod mit_support;

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
    inner.register_live_connection(1).unwrap();
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
    inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(arm.clone())));
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
    inner.register_live_connection(1).unwrap();
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
            grasp_setup_revision: None,
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
    replay_slot.replay_approach = Some(
        armd::replay::PreparedReplayApproach::from_endpoints(&start, &start, 0.2, None, true)
            .unwrap(),
    );
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
            approach_duration_s: 0.2,
            approach_power_scale: None,
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
    fx.core.register_live_connection(11).unwrap();
    fx.core.register_live_connection(12).unwrap();
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
    c4_slot.replay_approach = Some(
        armd::replay::PreparedReplayApproach::from_endpoints(&start, &start, 0.2, None, true)
            .unwrap(),
    );
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
            approach_duration_s: 0.2,
            approach_power_scale: None,
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
    fx.core.register_live_connection(2).unwrap();
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
    sup.register_ws_connection(5).await.unwrap();

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
fn a7_grasp_select_phase_b_capability_and_macro_hook() {
    // Phase B capability gating: 6-DOF rebot_dm advertises grasp_select_exec; 5-DOF does not.
    let dm_caps = armd::model::Capabilities::from_config(&ArmConfig::rebot_dm());
    assert!(dm_caps.grasp_select && dm_caps.grasp_select_exec);
    let so101 = armd::tool::baseline_arm_config_for_robot("so101").unwrap();
    let so_caps = armd::model::Capabilities::from_config(&so101);
    assert!(!so_caps.grasp_select && !so_caps.grasp_select_exec);

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

// ============================================================================
// Independent Review Coverage Tests (Blockers 1-3, Majors 1-15, Minors 1-2)
// ============================================================================

#[test]
fn review_b1_descent_rules_refuse_table_violation_on_direct_and_fitted_final_leg() {
    use armd::backlash::{DescentRules, Pre, Request as BacklashRequest};
    use armd::motion::{plan_cartesian_opts, resolve_goal_opts};
    use armd::rt_core::requests::actions::resolve_cartesian_legs_with_rules;

    let (fx, _b, _clock, _stop) = fixture();
    let snap = snap_of(&fx);
    let input = snap.plan_input_in(GoalFrame::Tcp).unwrap();
    let start_tcp = armd::motion::start_pose(&input);
    let goal_p = [start_tcp.p[0], start_tcp.p[1], start_tcp.p[2] - 0.04];
    let (goal, opts) = resolve_goal_opts(
        &input,
        &GoalSpec::Absolute {
            p: goal_p,
            r: None,
        },
    );
    let direct = plan_cartesian_opts(&input, goal, Speed::default(), opts).unwrap();
    let end_tcp_z = input
        .arm
        .fk_f32(direct.last_slice())
        .map(|e| armd::motion::end_to_tcp(&e, &input.tcp_offset).p[2])
        .unwrap();
    let start_tcp_z = start_tcp.p[2];
    assert!(end_tcp_z < start_tcp_z - 0.02);

    // 1) Direct path crossing DescentRules::table MUST be rejected with WorkspaceViolation
    //    (even when Pre::Off or Pre::Direct!).
    let bound_above_end = end_tcp_z + 0.015;
    let strict_rules = DescentRules {
        table: Some((vec![input.tcp_offset], bound_above_end)),
        ..DescentRules::tcp_at(input.tcp_offset)
    };
    let err_off = resolve_cartesian_legs_with_rules(
        &input,
        &snap.env.config.limits,
        direct.clone(),
        Pre::Off,
        None,
        Some(&strict_rules),
        false,
        false,
    )
    .expect_err("direct path descending below rules.table must be refused");
    assert!(
        matches!(err_off, armd::ArmError::Motion(armd::motion::MotionError::WorkspaceViolation { .. })),
        "expected WorkspaceViolation, got {err_off:?}"
    );

    let err_direct = resolve_cartesian_legs_with_rules(
        &input,
        &snap.env.config.limits,
        direct.clone(),
        Pre::Direct("request".into()),
        None,
        Some(&strict_rules),
        false,
        false,
    )
    .expect_err("Pre::Direct fallback crossing rules.table must be refused");
    assert!(
        matches!(err_direct, armd::ArmError::Motion(armd::motion::MotionError::WorkspaceViolation { .. })),
        "expected WorkspaceViolation, got {err_direct:?}"
    );

    // 2) Pre::Ready whose fitted final quintic violates table clearance:
    //    build a same-side plan, then construct a table bound that passes `direct` (start and end
    //    are both above `bound`), while a synthetic pre-target dips a tool point below `bound` on
    //    the fitted quintic leg -> selector MUST NOT return the violating quintic leg!
    let plan = snap
        .env
        .approach_plan("place_final", BacklashRequest::SameSide)
        .expect("same_side approach plan");
    let pre = armd::backlash::plan_cartesian_pre(
        &input,
        &direct,
        &plan,
        Some(&DescentRules::tcp_at(input.tcp_offset)),
        Speed::default(),
        opts,
    );
    let Pre::Ready(mut cp) = pre else {
        panic!("expected Pre::Ready for same_side test move, got {pre:?}");
    };
    // Perturb the final quintic target slightly downward in J2/J3 via a second tool corner
    // so `direct` stays above `bound` while the fitted quintic dips below `bound`.
    let corner_point = [0.0, 0.0, -0.05];
    let direct_min_z = direct
        .q
        .iter()
        .filter_map(|q| input.arm.fk_f32(q).map(|e| e.transform_point(&corner_point)[2]))
        .fold(f64::INFINITY, f64::min);
    // Make cp.path end at a joint config where corner_point is strictly lower than direct_min_z.
    let mut dipped_end = cp.path.last_slice().to_vec();
    dipped_end[1] += 0.25;
    let dipped_z = input
        .arm
        .fk_f32(&dipped_end)
        .map(|e| e.transform_point(&corner_point)[2])
        .unwrap();
    let bound = direct_min_z - 1e-4;
    if dipped_z < bound - 1e-4 {
        let mut last = [0.0f32; 7];
        last[..dipped_end.len()].copy_from_slice(&dipped_end);
        cp.path.q.push(last);
        cp.path.s.push(1.0);
        let rules_clear_direct = DescentRules {
            table: Some((vec![corner_point], bound)),
            ..DescentRules::tcp_at(input.tcp_offset)
        };
        let resolved = resolve_cartesian_legs_with_rules(
            &input,
            &snap.env.config.limits,
            direct.clone(),
            Pre::Ready(cp.clone()),
            Some(&plan),
            Some(&rules_clear_direct),
            false,
            false,
        )
        .expect("direct clears bound, so fallback should succeed with direct path");
        assert!(
            resolved.approach_leg.is_none(),
            "fitted final leg violating table bound must NOT be selected"
        );
        assert_eq!(
            resolved.outcome.as_ref().and_then(|o| o.approach.as_deref()),
            Some("direct")
        );
    }

    // 3) Pre::Ready whose fitted final leg fails AND direct ALSO violates rules.table -> WorkspaceViolation!
    let err_both = resolve_cartesian_legs_with_rules(
        &input,
        &snap.env.config.limits,
        direct,
        Pre::Ready(cp),
        Some(&plan),
        Some(&strict_rules),
        false,
        false,
    )
    .expect_err("when both fitted final leg and direct fallback cross rules.table, must refuse");
    assert!(
        matches!(err_both, armd::ArmError::Motion(armd::motion::MotionError::WorkspaceViolation { .. })),
        "expected WorkspaceViolation, got {err_both:?}"
    );
}

#[derive(Clone)]
struct RejectingBusDriver {
    q: Arc<Mutex<Vec<f32>>>,
    reject_writes: Arc<std::sync::atomic::AtomicBool>,
    last_written: Arc<Mutex<Vec<f32>>>,
    has_torque_feedback: bool,
    clock: VirtualClock,
}

impl RejectingBusDriver {
    fn new(init_q: &[f32], clock: VirtualClock) -> Self {
        Self {
            q: Arc::new(Mutex::new(init_q.to_vec())),
            reject_writes: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            last_written: Arc::new(Mutex::new(init_q.to_vec())),
            has_torque_feedback: true,
            clock,
        }
    }
}

impl armd::driver::ArmDriver for RejectingBusDriver {
    fn capabilities(&self) -> armd::driver::DriverCapabilities {
        armd::driver::DriverCapabilities {
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
    fn enable_state(&self) -> armd::driver::EnableState { armd::driver::EnableState::Enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        self.q.lock().unwrap().len() == dof
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        vec![Some(self.clock.now()); self.q.lock().unwrap().len()]
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        vec![10; self.q.lock().unwrap().len()]
    }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
        let q = self.q.lock().unwrap();
        Ok(q.iter()
            .map(|&p| armd::model::JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            })
            .collect())
    }
    fn write_joint_targets(&mut self, targets: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
        let pos: Vec<f32> = targets.iter().map(|t| t.position).collect();
        *self.last_written.lock().unwrap() = pos;
        Ok(())
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[armd::driver::JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, armd::driver::ArmDriverError> {
        let pos: Vec<f32> = targets.iter().map(|t| t.position).collect();
        *self.last_written.lock().unwrap() = pos;
        if self.reject_writes.load(std::sync::atomic::Ordering::SeqCst) {
            return Ok(armd::driver::TxBatchOutcome::Deferred { pending: 1 });
        }
        Ok(armd::driver::TxBatchOutcome::Complete)
    }
    fn set_torque(&mut self, _enable: bool) -> Result<(), armd::driver::ArmDriverError> { Ok(()) }
}

#[test]
fn review_b2_forced_finish_and_timeout_under_rejected_bus_holds_jaw_and_refuses_duplicates() {
    use std::sync::atomic::Ordering;

    let clock = VirtualClock::starting_at(Instant::now());
    let drv = RejectingBusDriver::new(&INSPECT, clock.clone());
    let mut inner = SupervisorInner::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    });
    inner.lease_enabled = false;
    inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv.clone())));
    inner.register_live_connection(1).unwrap();
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    for _ in 0..3 {
        inner.tick(DT);
    }
    inner.register_live_connection(1).unwrap();
    inner.register_live_connection(2).unwrap();
    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities {
            commands: 8,
            plans: 4,
            replies: 32,
        },
        clock.clone(),
    );

    // Part A: Explicit forced finish during active gripper close while bus rejects writes!
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;
    let r_begin = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            dl,
            Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Grasp, BOOT))),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Start gripper child moving closed; measured jaw lags behind commanded at -0.35.
    let snap = snap_of(&fx);
    let c1 = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "GRASP_TIMEOUT",
        contact: None,
    };
    let slot = prepare_child_gripper_slot(
        &snap.commanded_positions,
        GripperCommand::Move,
        Some(0.2),
        false,
        0.5,
        Some(8.0),
        None,
        false,
        c1,
    )
    .unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), dl, BOOT, slot, false).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    fx.step();
    let _ = drain_all(&mut b);

    // Pin measured jaw to -0.35 and make bus reject TX writes!
    let held_jaw = -0.35f32;
    drv.q.lock().unwrap()[6] = held_jaw;
    fx.core.measured_positions[6] = held_jaw;
    drv.reject_writes.store(true, Ordering::SeqCst);

    // Submit forced FinishMacro(Failed) with a diagnostic message (Minor 1).
    let fail_spec = FinishMacroSpec::failed(parent, BOOT, "TARGET_LOST", "aborted while closing");
    let r_fin = b
        .submit(
            ClientId(1),
            3,
            auth(&fx),
            fx.now() + DEFAULT_FINISH_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(fail_spec)),
        )
        .unwrap();
    let rep1 = fx.step();
    assert!(matches!(rep1.command, Some((r, Ok(_))) if r == r_fin), "{rep1:?}");
    assert!(rep1.ended.is_none(), "terminal MUST NOT emit while bus rejects hold TX");
    assert!(
        (drv.last_written.lock().unwrap()[6] - held_jaw).abs() < 1e-4,
        "jaw target must hold measured {held_jaw}, got {}",
        drv.last_written.lock().unwrap()[6]
    );
    let reps1 = drain_all(&mut b);
    assert!(reps1.iter().any(|(_, r)| matches!(r, Reply::Ack { request, .. } if *request == r_fin)));
    assert!(!reps1.iter().any(|(_, r)| matches!(r, Reply::Terminal { .. })));

    // Several more rejected releases:
    // - Duplicate FinishMacro -> PARENT_REPLACED (no premature terminal!)
    let mut dup_auth = auth(&fx);
    dup_auth.parent_generation = parent.0;
    let r_dup = b
        .submit(
            ClientId(1),
            4,
            dup_auth,
            fx.now() + DEFAULT_FINISH_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(fail_spec)),
        )
        .unwrap();
    let rep_dup = fx.step();
    assert_eq!(rep_dup.command, Some((r_dup, Err("PARENT_REPLACED"))));
    assert!(rep_dup.ended.is_none());
    assert!((drv.last_written.lock().unwrap()[6] - held_jaw).abs() < 1e-4);

    // - Replacement BeginMacro while finishing hold is pending bus accept -> MOTION_BUSY!
    let mut repl_auth = auth(&fx);
    repl_auth.parent_generation = 0;
    let r_repl = b
        .submit(
            ClientId(2),
            1,
            repl_auth,
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Place, BOOT))),
        )
        .unwrap();
    let rep_repl = fx.step();
    assert_eq!(rep_repl.command, Some((r_repl, Err("MOTION_BUSY"))));
    assert!(rep_repl.ended.is_none());
    assert!((drv.last_written.lock().unwrap()[6] - held_jaw).abs() < 1e-4);
    let _ = drain_all(&mut b);

    // Now allow the bus to accept the hold write -> exact single Terminal arrives with message!
    drv.reject_writes.store(false, Ordering::SeqCst);
    let rep_ok = fx.step();
    assert_eq!(rep_ok.ended, Some((parent, Outcome::Failed("TARGET_LOST"))));
    assert!((drv.last_written.lock().unwrap()[6] - held_jaw).abs() < 1e-4);
    assert_eq!(
        fx.core.last_action.as_ref().and_then(|a| a.message.as_deref()),
        Some("aborted while closing"),
        "Minor 1: deferred MacroFinish must preserve finish diagnostic message"
    );
    let reps_ok = drain_all(&mut b);
    assert_eq!(
        reps_ok
            .iter()
            .filter(|(_, r)| matches!(
                r,
                Reply::Terminal { request, action, outcome: Outcome::Failed("TARGET_LOST"), children: 1 }
                    if *request == r_begin && *action == parent
            ))
            .count(),
        1
    );

    // Part B: Parent deadline expiry wins over child expiry AND defers until bus accepts jaw hold!
    let mut b2 = BeginMacroSpec::new(MacroKind::Grasp, BOOT);
    b2.macro_deadline = Some(fx.now() + Duration::from_millis(80));
    b2.timeout_code = armd::safety::BoundedText::new("GRASP_TIMEOUT");
    let r_b2 = b
        .submit(
            ClientId(1),
            10,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(b2)),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent2 = fx.active().unwrap();
    let _ = drain_all(&mut b);

    let snap2 = snap_of(&fx);
    let c2 = ChildSpec {
        ticket: ChildTicket { parent: parent2, child_seq: 1 },
        timeout_s: 0.04, // shorter than parent deadline!
        timeout_code: "CHILD_TIMEOUT",
        contact: None,
    };
    let slot2 = prepare_child_gripper_slot(
        &snap2.commanded_positions,
        GripperCommand::Move,
        Some(0.8),
        false,
        0.4,
        Some(8.0),
        None,
        false,
        c2,
    )
    .unwrap();
    submit_child(&mut b, ClientId(1), 11, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot2, false).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let _ = drain_all(&mut b);

    let held_jaw2 = -0.22f32;
    drv.q.lock().unwrap()[6] = held_jaw2;
    fx.core.measured_positions[6] = held_jaw2;
    drv.reject_writes.store(true, Ordering::SeqCst);

    // Jump past BOTH child deadline (40ms) and parent deadline (80ms) at once!
    clock.advance(Duration::from_millis(120));
    let rep_exp = fx.step();
    assert!(rep_exp.ended.is_none(), "parent timeout must defer terminal until bus accepts jaw hold");
    assert!((fx.core.commanded_positions[6] - held_jaw2).abs() < 1e-4);
    // One more rejected tick -> still no terminal, jaw still held!
    let rep_exp2 = fx.step();
    assert!(rep_exp2.ended.is_none());
    assert!((drv.last_written.lock().unwrap()[6] - held_jaw2).abs() < 1e-4);

    // Accept on bus -> parent emits GRASP_TIMEOUT (NOT child timeout!).
    drv.reject_writes.store(false, Ordering::SeqCst);
    let rep_exp3 = fx.step();
    assert_eq!(rep_exp3.ended, Some((parent2, Outcome::Failed("GRASP_TIMEOUT"))));
    let reps_exp = drain_all(&mut b);
    assert!(reps_exp.iter().any(|(_, r)| matches!(
        r,
        Reply::Terminal { request, action, outcome: Outcome::Failed("GRASP_TIMEOUT"), .. }
            if *request == r_b2 && *action == parent2
    )));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn review_b3_live_gripper_zero_check_refreshes_prep_env_for_subsequent_actions() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("gripper_calib.json");
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    }));
    let initial_rev;
    let zc_id = {
        let mut inner = sup.inner().write().await;
        inner.lease_enabled = false;
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.measured_positions = INSPECT.to_vec();
        inner.commanded_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        inner.measured_positions[6] = -0.05;
        inner.commanded_positions[6] = -0.05;
        inner.target_positions[6] = -0.05;
        inner.gripper_calib_path = Some(path.clone());
        inner.set_mock_gripper_stall_offset(Some(0.05));
        initial_rev = inner.gripper_calib_revision;
        let epoch = inner.safety.epoch_id();
        let s2 = inner.current_admission_stamp();
        let a2 = inner.check_admission(s2).unwrap();
        SupervisorInner::start_gripper_zero_check(
            a2,
            &json!({"threshold_rad": 0.10, "store_offset": true}),
            epoch,
        )
        .unwrap()
        .0
    };

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    sup.register_ws_connection(1).await.unwrap();

    // Wait for zero_check(store_offset=true) to complete and armd-persist to activate + refresh prep_env!
    let wait_dl = Instant::now() + Duration::from_secs(5);
    let mut refreshed = false;
    while Instant::now() < wait_dl {
        if let (Some(pub_snap), Some(env)) = (sup.hub().latest(), sup.hub().prep_env()) {
            let done = pub_snap
                .obs
                .action
                .as_ref()
                .is_some_and(|a| a.id == zc_id && a.state == "done");
            if done
                && pub_snap.rt.owner.gripper_calib_revision > initial_rev
                && env.gripper_calib_revision == pub_snap.rt.owner.gripper_calib_revision
            {
                refreshed = true;
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(refreshed, "live zero_check activation must advance and republish ActionPrepEnv");

    // Immediately capture fresh snapshot -> MUST NOT return PLAN_STALE!
    let dl = Instant::now() + DEFAULT_ACTION_DEADLINE;
    let snap = ActionSnapshot::capture(&sup, sup.capture_admission_stamp(), dl)
        .await
        .expect("fresh ActionSnapshot after live calibration must succeed (not PLAN_STALE)");
    assert!(snap.env.gripper_calib_revision > initial_rev);

    // Register a per-connection reply sink for ClientId(1) so InstallPrepared commits through P-F1's router.
    let conn1_terminals = Arc::new(Mutex::new(Vec::<Reply>::new()));
    {
        let out = Arc::clone(&conn1_terminals);
        handle
            .register_connection_reply_sink(
                ClientId(1),
                Arc::new(move |_, r| {
                    out.lock().unwrap().push(r);
                    DeliveryDecision::Delivered
                }),
                None,
            )
            .unwrap();
    }

    // 1) Fresh standalone gripper succeeds!
    let g_prep = prepare_gripper(&snap, &json!({"command": "move", "fraction": 0.2, "speed": 1.0}), GripperBaselineCmd::None).unwrap();
    let g_slot = g_prep.to_plan_slot(None).unwrap();
    let write_g = |dst: &mut PlanSlot| *dst = g_slot.clone();
    let (_, mut rx_g, _guard_g) = handle
        .submit_external_action(
            ClientId(1),
            1,
            snap.owner.authority,
            dl,
            ActionSubmit::InstallPrepared {
                write: &write_g,
                kind: PreparedActionKind::Gripper,
                prepared_boot: handle.boot(),
                admission_request_id: 0,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: g_prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            },
            armd::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    let mut ack_g = None;
    for _ in 0..200 {
        handle.pump_broker_replies();
        if let Ok(Ok(r)) = rx_g.try_recv() {
            ack_g = Some(r);
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(matches!(ack_g, Some(Reply::Ack { .. })), "fresh gripper after calib failed: {ack_g:?}");
    let g_action_id = match ack_g {
        Some(Reply::Ack { action: Some(a), .. }) => a,
        other => panic!("expected Ack with ActionId, got {other:?}"),
    };
    let mut g_ended = false;
    let g_wait_dl = Instant::now() + Duration::from_secs(10);
    while Instant::now() < g_wait_dl {
        handle.pump_broker_replies();
        if conn1_terminals
            .lock()
            .unwrap()
            .iter()
            .any(|r| matches!(r, Reply::Terminal { action, .. } if *action == g_action_id))
        {
            g_ended = true;
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(g_ended, "fresh gripper action after calib must complete");
    tokio::time::sleep(Duration::from_millis(15)).await;

    // 2) Fresh standalone move_to succeeds!
    let snap2 = ActionSnapshot::capture(&sup, sup.capture_admission_stamp(), Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let p0 = armd::motion::start_pose(&snap2.plan_input_in(GoalFrame::Tcp).unwrap()).p;
    let m_prep = prepare_move_to(&snap2, &json!({"position": [p0[0] + 0.02, p0[1], p0[2]]})).unwrap();
    let m_slot = m_prep.to_plan_slot(None).unwrap();
    let write_m = |dst: &mut PlanSlot| *dst = m_slot.clone();
    let (_, mut rx_m, _guard_m) = handle
        .submit_external_action(
            ClientId(1),
            2,
            snap2.owner.authority,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
            ActionSubmit::InstallPrepared {
                write: &write_m,
                kind: PreparedActionKind::MoveToLinear,
                prepared_boot: handle.boot(),
                admission_request_id: 0,
                check_workspace_revision: true,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: Some(snap2.env.power_budget_revision),
            },
            armd::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOnly,
            false,
            None,
        )
        .unwrap();
    let mut ack_m = None;
    for _ in 0..200 {
        handle.pump_broker_replies();
        if let Ok(Ok(r)) = rx_m.try_recv() {
            ack_m = Some(r);
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(matches!(ack_m, Some(Reply::Ack { .. })), "fresh move_to after calib failed: {ack_m:?}");
    let m_action_id = match ack_m {
        Some(Reply::Ack { action: Some(a), .. }) => a,
        other => panic!("expected Ack with ActionId, got {other:?}"),
    };
    let mut m_ended = false;
    for _ in 0..600 {
        handle.pump_broker_replies();
        if conn1_terminals
            .lock()
            .unwrap()
            .iter()
            .any(|r| matches!(r, Reply::Terminal { action, .. } if *action == m_action_id))
        {
            m_ended = true;
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(m_ended, "fresh move_to action after calib must complete");
    tokio::time::sleep(Duration::from_millis(15)).await;

    // 3) Fresh A7 BeginMacro -> FinishMacro succeeds once an external reply sink is attached!
    handle.attach_external_reply_sink(Arc::new(|_, _| DeliveryDecision::Delivered));
    let snap3 = ActionSnapshot::capture(&sup, sup.capture_admission_stamp(), Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let begin = BeginMacroSpec::new(MacroKind::Place, handle.boot());
    let (mid, _) = armd::macro_action::begin_macro_executor(
        &sup,
        &snap3.stamp,
        begin,
        Instant::now() + DEFAULT_ACTION_DEADLINE,
    )
    .await
    .expect("fresh BeginMacro after live calib must succeed");
    assert!(mid > 0);
    handle.stop_and_join();
}

#[test]
fn review_m1_m2_m3_deadline_precedence_parent_authority_and_single_use_admission() {
    let (mut fx, mut b, clock, _stop) = fixture();

    // --- Major 1: FinishMacro(Done) submitted after parent expiry with valid request deadline ---
    let mut begin = BeginMacroSpec::new(MacroKind::Place, BOOT);
    begin.macro_deadline = Some(fx.now() + Duration::from_millis(80));
    begin.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
    let r_b1 = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(begin)),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent1 = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Advance virtual clock PAST parent macro_deadline while FinishMacro request deadline is still valid!
    clock.advance(Duration::from_millis(120));
    let done_spec = FinishMacroSpec::done(parent1, BOOT);
    let r_done = b
        .submit(
            ClientId(1),
            2,
            auth(&fx),
            fx.now() + DEFAULT_FINISH_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(done_spec)),
        )
        .unwrap();
    let rep = fx.step();
    assert_eq!(rep.command, Some((r_done, Err("DEADLINE_EXPIRED"))));
    assert_eq!(rep.ended, Some((parent1, Outcome::Failed("PLACE_TIMEOUT"))));
    let reps = drain_all(&mut b);
    assert!(reps.iter().any(|(_, r)| matches!(
        r,
        Reply::Terminal { request, action, outcome: Outcome::Failed("PLACE_TIMEOUT"), .. }
            if *request == r_b1 && *action == parent1
    )));

    // --- Major 2: Two-client parent authority with leases disabled ---
    assert!(!fx.core.lease_enabled);
    fx.core.register_live_connection(11).unwrap();
    fx.core.register_live_connection(22).unwrap();
    let r_b2 = b
        .submit(
            ClientId(11),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Sequence, BOOT))),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent2 = fx.active().unwrap();
    let _ = drain_all(&mut b);

    // Client 22 (different client!) tries InstallChild with parent2's ID -> MUST refuse LEASE_LOST!
    let snap = snap_of(&fx);
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.03;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c1 = ChildSpec {
        ticket: ChildTicket { parent: parent2, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        contact: None,
    };
    let slot = prepare_child_joint_slot(&snap.commanded_positions, &tgt, &max_vel, 0.2, None, c1).unwrap();
    let r_hijack_child = submit_child(&mut b, ClientId(22), 1, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot, true).unwrap();
    assert_eq!(fx.step().command, Some((r_hijack_child, Err("LEASE_LOST"))));
    assert_eq!(fx.active(), Some(parent2));

    // Client 22 tries FinishMacro(Done) with parent2's ID -> MUST refuse LEASE_LOST!
    let r_hijack_fin = b
        .submit(
            ClientId(22),
            2,
            auth(&fx),
            fx.now() + DEFAULT_FINISH_DEADLINE,
            Submit::Action(ActionSubmit::FinishMacro(FinishMacroSpec::done(parent2, BOOT))),
        )
        .unwrap();
    assert_eq!(fx.step().command, Some((r_hijack_fin, Err("LEASE_LOST"))));
    assert_eq!(fx.active(), Some(parent2));
    let _ = drain_all(&mut b);

    // Original owner Client 11 finishes parent2 cleanly!
    b.submit(
        ClientId(11),
        2,
        auth(&fx),
        fx.now() + DEFAULT_FINISH_DEADLINE,
        Submit::Action(ActionSubmit::FinishMacro(FinishMacroSpec::done(parent2, BOOT))),
    )
    .unwrap();
    assert_eq!(fx.step().ended, Some((parent2, Outcome::Completed)));
    let _ = r_b2;
    let _ = drain_all(&mut b);

    // --- Major 3: Single-use original admission_request_id across InstallPrepared & BeginMacro ---
    let snap = snap_of(&fx);
    let prep = prepare_gripper(&snap, &json!({"command": "open", "speed": 1.0}), GripperBaselineCmd::None).unwrap();
    let slot1 = prep.to_plan_slot(None).unwrap();
    let slot2 = prep.to_plan_slot(None).unwrap();
    let orig_req_id = 4242u64;
    let w1 = |dst: &mut PlanSlot| *dst = slot1.clone();
    let r_first = b
        .submit(
            ClientId(1),
            10,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w1,
                kind: PreparedActionKind::Gripper,
                prepared_boot: BOOT,
                admission_request_id: orig_req_id,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    assert!(matches!(fx.step().command, Some((r, Ok(_))) if r == r_first));
    let _ = step_until_ended(&mut fx, 400);
    let _ = drain_all(&mut b);

    // Re-submit the SAME original admission_request_id with a brand new broker seq/RequestId -> STALE_ADMISSION!
    let w2 = |dst: &mut PlanSlot| *dst = slot2.clone();
    let r_second = b
        .submit(
            ClientId(1),
            11,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w2,
                kind: PreparedActionKind::Gripper,
                prepared_boot: BOOT,
                admission_request_id: orig_req_id,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    assert_eq!(fx.step().command, Some((r_second, Err("STAMP_REUSED"))));

    // Same for BeginMacro with reused admission_request_id = 4343!
    let mut bm = BeginMacroSpec::new(MacroKind::Place, BOOT);
    bm.admission_request_id = 4343;
    let r_bm1 = b
        .submit(ClientId(1), 12, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, Submit::Action(ActionSubmit::BeginMacro(bm)))
        .unwrap();
    assert!(matches!(fx.step().command, Some((r, Ok(_))) if r == r_bm1));
    let p_bm = fx.active().unwrap();
    b.submit(ClientId(1), 13, auth(&fx), fx.now() + DEFAULT_FINISH_DEADLINE, Submit::Action(ActionSubmit::FinishMacro(FinishMacroSpec::done(p_bm, BOOT)))).unwrap();
    assert_eq!(fx.step().ended, Some((p_bm, Outcome::Completed)));
    let _ = drain_all(&mut b);

    let r_bm2 = b
        .submit(ClientId(1), 14, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, Submit::Action(ActionSubmit::BeginMacro(bm)))
        .unwrap();
    assert_eq!(fx.step().command, Some((r_bm2, Err("STAMP_REUSED"))));
}

#[test]
fn review_m4_m5_gripper_calib_and_power_budget_revision_fencing() {
    let (mut fx, mut b, _clock, _stop) = fixture();
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;

    // --- Major 4: gripper_calib_revision change between prepare and commit -> PLAN_STALE ---
    let snap = snap_of(&fx);
    let g_prep = prepare_gripper(&snap, &json!({"command": "open", "speed": 1.0}), GripperBaselineCmd::None).unwrap();
    assert_eq!(g_prep.stamp.gripper_calib_revision, Some(fx.core.gripper_calib_revision));
    let g_slot = g_prep.to_plan_slot(None).unwrap();
    // Bump gripper_calib_revision before commit!
    fx.core.gripper_calib_revision += 1;
    let wg = |dst: &mut PlanSlot| *dst = g_slot.clone();
    let r_g = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            dl,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &wg,
                kind: PreparedActionKind::Gripper,
                prepared_boot: BOOT,
                admission_request_id: 0,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: g_prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    assert_eq!(fx.step().command, Some((r_g, Err("PLAN_STALE"))));
    let _ = drain_all(&mut b);

    // Same for child gripper and child Cartesian power_budget_revision (Major 5)!
    b.submit(ClientId(1), 2, auth(&fx), dl, Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Place, BOOT)))).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    let snap2 = snap_of(&fx);
    let old_g_rev = snap2.env.gripper_calib_revision;
    let old_p_rev = snap2.env.power_budget_revision;

    // Stale child gripper_calib_revision -> PLAN_STALE!
    fx.core.gripper_calib_revision += 1;
    let cg = ChildSpec { ticket: ChildTicket { parent, child_seq: 1 }, timeout_s: 10.0, timeout_code: "PLACE_TIMEOUT", contact: None };
    let cg_slot = prepare_child_gripper_slot(&snap2.commanded_positions, GripperCommand::Open, None, false, 1.0, Some(8.0), None, false, cg).unwrap();
    let wcg = |dst: &mut PlanSlot| *dst = cg_slot.clone();
    let r_cg = b
        .submit(
            ClientId(1),
            3,
            auth(&fx),
            dl,
            Submit::Action(ActionSubmit::InstallChild {
                write: &wcg,
                prepared_boot: BOOT,
                check_workspace_revision: false,
                calibration_revision: None,
                grasp_setup_revision: None,
                gripper_calib_revision: Some(old_g_rev),
                power_budget_revision: None,
            }),
        )
        .unwrap();
    assert_eq!(fx.step().command, Some((r_cg, Err("PLAN_STALE"))));

    // Stale child power_budget_revision -> PLAN_STALE (Major 5)!
    fx.core.power_budget_revision += 1;
    let input = snap2.plan_input_in(GoalFrame::Tcp).unwrap();
    let p0 = armd::motion::start_pose(&input).p;
    let path = armd::motion::plan_cartesian(&input, armd::motion::resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.015, p0[1], p0[2]], r: None }), Speed::default()).unwrap();
    let cc = ChildSpec { ticket: ChildTicket { parent, child_seq: 1 }, timeout_s: 10.0, timeout_code: "PLACE_TIMEOUT", contact: None };
    let cc_slot = armd::rt_core::requests::actions::prepare_child_cartesian_slot(&snap2.commanded_positions, &path, None, cc).unwrap();
    let wcc = |dst: &mut PlanSlot| *dst = cc_slot.clone();
    let r_cc = b
        .submit(
            ClientId(1),
            4,
            auth(&fx),
            dl,
            Submit::Action(ActionSubmit::InstallChild {
                write: &wcc,
                prepared_boot: BOOT,
                check_workspace_revision: true,
                calibration_revision: None,
                grasp_setup_revision: None,
                gripper_calib_revision: None,
                power_budget_revision: Some(old_p_rev),
            }),
        )
        .unwrap();
    assert_eq!(fx.step().command, Some((r_cc, Err("PLAN_STALE"))));
}

#[test]
fn review_m8_m9_mixed_legacy_child_refused_and_gripper_to_jog_replacement_order() {
    use armd::rt_core::ops::Primitive;
    use armd::safety::SafetyState;

    let clock = VirtualClock::starting_at(Instant::now());
    let drv = RejectingBusDriver::new(&INSPECT, clock.clone());
    let mut inner = SupervisorInner::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    });
    inner.lease_enabled = false;
    inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv.clone())));
    inner.register_live_connection(1).unwrap();
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    for _ in 0..3 {
        inner.tick(DT);
    }
    let (mut fx, mut b, _clock, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities {
            commands: 8,
            plans: 4,
            replies: 32,
        },
        clock,
    );
    let dl = fx.now() + DEFAULT_ACTION_DEADLINE;

    // --- Major 8: Legacy Submit::Install child against A7 ActiveSlot::Macro -> UNSUPPORTED ---
    let r_begin = b
        .submit(ClientId(1), 1, auth(&fx), dl, Submit::Action(ActionSubmit::BeginMacro(BeginMacroSpec::new(MacroKind::Sequence, BOOT))))
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    let mut q_arr = [0.0f32; MAX_DOF];
    q_arr[..INSPECT.len()].copy_from_slice(&INSPECT);
    let mut tgt_arr = q_arr;
    tgt_arr[0] += 0.03;
    let legacy_child_prim = Primitive::JointQuinticLimited {
        dof: INSPECT.len() as u8,
        start: q_arr,
        q: tgt_arr,
        max_velocity: [1.0; MAX_DOF],
        duration_s: 0.2,
        settle_s: 0.1,
        child: Some(ChildSpec {
            ticket: ChildTicket { parent, child_seq: 1 },
            timeout_s: 10.0,
            timeout_code: "SEQUENCE_TIMEOUT",
            contact: None,
        }),
    };
    let w_leg = |dst: &mut PlanSlot| dst.set(legacy_child_prim);
    let r_leg = b
        .submit(ClientId(1), 2, auth(&fx), dl, Submit::Install(&w_leg))
        .unwrap();
    assert_eq!(fx.step().command, Some((r_leg, Err("UNSUPPORTED"))));
    assert_eq!(fx.active(), Some(parent), "refused mixed legacy child must keep A7 parent intact");
    let reps_m8 = drain_all(&mut b);
    assert!(!reps_m8.iter().any(|(_, r)| matches!(r, Reply::Terminal { .. })));

    // --- Major 9: Active macro gripper child replaced by RT jog -> post-quiesce start, target, SafetyState::Moving, Terminal(old) < Ack(new) ---
    let snap = snap_of(&fx);
    let cg = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        contact: None,
    };
    let cg_slot = prepare_child_gripper_slot(&snap.commanded_positions, GripperCommand::Move, Some(0.1), false, 0.5, Some(8.0), None, false, cg).unwrap();
    submit_child(&mut b, ClientId(1), 3, auth(&fx), dl, BOOT, cg_slot, false).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    fx.step();
    let _ = drain_all(&mut b);

    // Diverge measured jaw (-0.25) from commanded jaw (-0.55) to verify post-quiesce start!
    let meas_jaw = -0.25f32;
    drv.q.lock().unwrap()[6] = meas_jaw;
    fx.core.measured_positions[6] = meas_jaw;
    fx.core.commanded_positions[6] = -0.55;

    let mut jog_start = [0.0f32; MAX_DOF];
    jog_start[..INSPECT.len()].copy_from_slice(&fx.core.commanded_positions);
    jog_start[6] = meas_jaw; // post-quiesce jaw position!
    let mut jog_target = jog_start;
    jog_target[0] += 0.12;
    let mut jog_auth = auth(&fx);
    jog_auth.parent_generation = 0;
    let mut jog_snap = armd::rt_core::requests::motion::MotionPlanSnapshot::from_supervisor(&fx.core);
    jog_snap.commanded_positions = jog_start[..INSPECT.len()].to_vec();
    jog_snap.commanded_velocities[6] = 0.0;
    jog_snap.authority = armd::rt_core::requests::motion::PlanAuthority {
        boot: BOOT,
        epoch: jog_auth.epoch,
        lease_generation: jog_auth.lease_generation,
        stop_generation: jog_auth.stop_generation,
        deadline: dl,
    };
    let jog_plan = armd::rt_core::requests::motion::prepare_motion(
        &jog_snap,
        &armd::rt_core::requests::motion::MotionRequest::jog(
            jog_target[..INSPECT.len()].to_vec(),
            Some(0.4),
            None,
        ),
    )
    .unwrap();
    let w_jog = |dst: &mut PlanSlot| armd::rt_core::requests::motion::stage_jog(dst, &jog_plan.slot);
    let r_jog = b
        .submit(ClientId(1), 4, jog_auth, dl, Submit::Install(&w_jog))
        .unwrap();
    let rep_jog = fx.step();
    assert!(matches!(rep_jog.command, Some((r, Ok(_))) if r == r_jog), "{rep_jog:?}");
    let jog_id = fx.active().expect("jog must be active");
    assert_ne!(jog_id, parent);
    // Verify target_positions is the JOG's target (NOT overwritten by old quiesce!),
    // jaw snapped to meas_jaw, and safety is SafetyState::Moving (NOT reset to Armed!).
    assert!(
        (fx.core.target_positions[0] - jog_target[0]).abs() < 1e-4,
        "jog target J1 was overwritten: expected {}, got {}",
        jog_target[0],
        fx.core.target_positions[0]
    );
    assert!(
        (fx.core.target_positions[6] - meas_jaw).abs() < 1e-4,
        "jog target J7 must preserve post-quiesce jaw {meas_jaw}, got {}",
        fx.core.target_positions[6]
    );
    assert_eq!(fx.core.safety.state(), &SafetyState::Moving, "safety state must remain Moving after replacement");
    let reps_jog = drain_all(&mut b);
    let pos_term = reps_jog
        .iter()
        .position(|(_, r)| matches!(r, Reply::Terminal { request, action, outcome: Outcome::Superseded, .. } if *request == r_begin && *action == parent))
        .expect("old parent must emit exactly one Terminal(Superseded)");
    let pos_ack = reps_jog
        .iter()
        .position(|(_, r)| matches!(r, Reply::Ack { request, action: Some(a), .. } if *request == r_jog && *a == jog_id))
        .expect("new jog must emit Ack");
    assert!(pos_term < pos_ack, "Terminal(old) must strictly precede Ack(new): {reps_jog:?}");
    // Verify old parent emitted EXACTLY ONE terminal!
    assert_eq!(
        reps_jog
            .iter()
            .filter(|(_, r)| matches!(r, Reply::Terminal { action, .. } if *action == parent))
            .count(),
        1
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn review_m10_reply_router_interleaved_external_and_backpressure() {
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
        for _ in 0..3 {
            inner.tick(DT);
        }
    }
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    sup.register_ws_connection(99).await.unwrap();
    sup.register_ws_connection(7).await.unwrap();
    for _ in 0..100 {
        if sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Submit 3 external refused commands whose replies sit at the head of the broker reply ring!
    let pub_snap = sup.hub().latest().unwrap();
    let dl = Instant::now() + DEFAULT_ACTION_DEADLINE;
    let mut ext_reqs = Vec::new();
    for seq in 1..=3u64 {
        let mut bad_begin = BeginMacroSpec::new(MacroKind::Place, BootId(9999));
        bad_begin.admission_request_id = 0;
        let r = handle
            .broker()
            .lock()
            .unwrap()
            .submit(
                ClientId(99),
                seq,
                pub_snap.rt.owner.authority,
                dl,
                Submit::Action(ActionSubmit::BeginMacro(bad_begin)),
            )
            .unwrap();
        ext_reqs.push(r);
    }
    // Wait until all 3 external replies are in the broker ring / external queue, then attach
    // a backpressurable ExternalReplySink so they remain buffered while BeginMacro/MacroCtx run!
    tokio::time::sleep(Duration::from_millis(40)).await;
    let bp_gate = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let drained_sink = Arc::new(Mutex::new(Vec::<(ClientId, Reply)>::new()));
    {
        let gate = Arc::clone(&bp_gate);
        let out = Arc::clone(&drained_sink);
        handle.attach_external_reply_sink(Arc::new(move |c, r| {
            if gate.load(std::sync::atomic::Ordering::Acquire) {
                DeliveryDecision::Backpressured
            } else {
                out.lock().unwrap().push((c, r));
                DeliveryDecision::Delivered
            }
        }));
    }
    assert_eq!(handle.pending_external_replies(), 3, "external replies must buffer in FIFO without dropping");

    // While external consumer is backpressured, drive BeginMacro + MacroCtx::install_joint_child + finish!
    let mut adm = sup.capture_admission_stamp();
    adm.connection_id = 7;
    let snap = ActionSnapshot::capture(&sup, adm, dl).await.unwrap();
    let mut begin = BeginMacroSpec::new(MacroKind::Sequence, handle.boot());
    begin.admission_request_id = adm.request_id;
    let (parent_id, r_begin) = armd::macro_action::begin_macro_executor(&sup, &snap.stamp, begin, dl)
        .await
        .expect("BeginMacro must NOT deadlock behind backpressured external replies");
    let ctx = MacroCtx {
        sup: sup.clone(),
        id: parent_id,
        stamp: adm,
        deadline: Instant::now() + Duration::from_secs(10),
        phase_timeout_s: 8.0,
        timeout_code: "SEQUENCE_TIMEOUT",
        timeout_message: "timeout".into(),
    };
    let mut tgt = snap.commanded_positions.clone();
    tgt[0] += 0.04;
    ctx.install_joint_child("s1", tgt, snap.env.planning_limits(Some(0.8)), true, None, None)
        .await
        .expect("MacroCtx child must succeed despite backpressured external replies");
    ctx.finish(FinishState::Done, None, None, None, None, HeldUpdate::Unchanged).await;
    tokio::time::sleep(Duration::from_millis(40)).await;

    // Now release sink backpressure and pump -> assert EXACT FIFO order into the attached sink:
    // ext_reqs[0..3] Refused followed by BeginMacro Terminal(Completed), and ZERO internal child/finish ACKs!
    bp_gate.store(false, std::sync::atomic::Ordering::Release);
    handle.pump_broker_replies();
    let drained = drained_sink.lock().unwrap().clone();
    assert_eq!(drained.len(), 4, "expected 3 external Refused + 1 BeginMacro Terminal, got {drained:?}");
    for (idx, req_id) in ext_reqs.iter().enumerate() {
        assert!(
            matches!(drained[idx], (ClientId(99), Reply::Refused { request, code: "WRONG_TARGET" }) if request == *req_id),
            "FIFO mismatch at index {idx}: {:?}",
            drained[idx]
        );
    }
    assert!(
        matches!(
            drained[3],
            (ClientId(7), Reply::Terminal { request, action, outcome: Outcome::Completed, children: 1 })
                if request == r_begin && action == ActionId(parent_id)
        ),
        "expected BeginMacro Terminal(Completed, children=1) at tail, got {:?}",
        drained[3]
    );
    handle.stop_and_join();
}

struct DetachedMockVision {
    next_id: std::sync::atomic::AtomicU64,
}

impl DetachedMockVision {
    fn new() -> Self {
        Self {
            next_id: std::sync::atomic::AtomicU64::new(1),
        }
    }
}

impl armd::grasp::DetectionSource for DetachedMockVision {
    fn inference_enabled(&self) -> armd::grasp::BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn set_inference(&self, _enabled: bool) -> armd::grasp::BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn next_frame(
        &self,
        _after: Option<u64>,
        _wait_ms: u64,
    ) -> armd::grasp::BoxFuture<'_, Result<Option<armd::grasp::DetectionFrame>, String>> {
        self.trigger_with_focus(&["yolo"], 100, None)
    }
    fn per_request_pipelines(&self) -> armd::grasp::BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn trigger_with_focus(
        &self,
        _pipelines: &'static [&'static str],
        _wait_ms: u64,
        _focus: Option<&armd::grasp::MaskFocus>,
    ) -> armd::grasp::BoxFuture<'_, Result<Option<armd::grasp::DetectionFrame>, String>> {
        let id = self.next_id.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        Box::pin(async move {
            Ok(Some(armd::grasp::DetectionFrame {
                frame_id: id,
                captured_at: Instant::now(),
                policy_captured_at: None,
                image_size_px: [640.0, 480.0],
                depth: None,
                model_generation: Some(1),
                detections: vec![armd::grasp::Detection {
                    label: "cube".into(),
                    score: 0.95,
                    raw_score_valid: true,
                    box_norm: [0.46, 0.46, 0.54, 0.54],
                    angle_deg: Some(15.0),
                    angle_conf: Some(0.9),
                    grasp_width_px: Some(36.0),
                    mask_focus: false,
                    disp_fg: None,
                    disp_bg: None,
                    depth_conf: None,
                    length_px: None,
                    jaw_clear: None,
                    depth_angle_deg: None,
                    depth_angle_conf: None,
                    depth_width_px: None,
                    contour_norm: None,
                }],
            }))
        })
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn review_m11_m12_m13_m14_production_adapters_detached_shell_and_same_side_joint_parity() {
    // --- Part A (Major 14): Same-side joint parity across Legacy & RT Executor ---
    {
        let mut cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        cfg.backlash_approach = Some(armd::backlash::BacklashApproach::default());
        let mut leg_inner = common::sim_at(cfg.clone(), &INSPECT);
        let mut target_pose = INSPECT.to_vec();
        target_pose[0] += 0.12;
        armd::poses::save(&mut leg_inner, &json!({"name": "ss_pose", "positions": target_pose})).unwrap();
        let adm = leg_inner.current_admission_stamp();
        let snap = ActionSnapshot::from_inner(&leg_inner, adm, Instant::now() + DEFAULT_ACTION_DEADLINE);
        let prep = prepare_saved_pose(&snap, &json!({"name": "ss_pose", "speed_scale": 1.0})).unwrap();
        assert!(prep.approach_leg.is_some(), "same-side joint move must prepare approach_leg off-thread");
        let slot = prep.to_plan_slot(None).unwrap();
        match slot.primitive {
            armd::rt_core::ops::Primitive::Action(armd::rt_core::requests::actions::StagedAction::Joint {
                approach_then,
                target,
                ..
            }) => {
                assert!(approach_then.is_some(), "RT joint slot must stage approach_then leg");
                assert!(
                    (target[0] - target_pose[0]).abs() > 1e-3,
                    "leg 1 target must be pre-target, not final target"
                );
                assert!(
                    (approach_then.unwrap().target[0] - target_pose[0]).abs() < 1e-5,
                    "leg 2 target must equal final target"
                );
            }
            other => panic!("expected StagedAction::Joint, got {other:?}"),
        }
        // Install on Legacy and run to completion.
        let ctx = armd::rt_core::requests::actions::OwnerCheckCtx::for_legacy(&leg_inner);
        let (leg_id, leg_dur) = armd::rt_core::requests::actions::install_move_joint(
            &mut leg_inner,
            &prep.stamp,
            &ctx,
            armd::rt_core::requests::actions::JointInstallPayload::Legacy(Box::new(prep.clone())),
        )
        .unwrap();
        assert!(leg_dur > 0.5);
        for _ in 0..300 {
            leg_inner.tick(DT);
            if leg_inner.last_action.as_ref().is_some_and(|a| a.id == leg_id && !a.is_running()) {
                break;
            }
        }
        assert_eq!(leg_inner.last_action.as_ref().unwrap().state, "done");
        assert_eq!(leg_inner.last_action.as_ref().unwrap().approach.as_deref(), Some("same_side"));

        // Install the SAME prepared slot on RT Executor and run to completion.
        let (mut fx, mut b, _clk, _st) = fixture();
        fx.core.config.backlash_approach = Some(armd::backlash::BacklashApproach::default());
        armd::poses::save(&mut fx.core, &json!({"name": "ss_pose", "positions": target_pose})).unwrap();
        let snap_rt = snap_of(&fx);
        let prep_rt = prepare_saved_pose(&snap_rt, &json!({"name": "ss_pose", "speed_scale": 1.0})).unwrap();
        let slot_rt = prep_rt.to_plan_slot(None).unwrap();
        let r = submit_slot_prepared(
            &mut b,
            ClientId(1),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            PreparedActionKind::MoveToPose,
            BOOT,
            slot_rt,
            true,
            prep_rt.stamp.poses_revision,
            prep_rt.stamp.power_budget_revision,
        )
        .unwrap();
        assert!(matches!(fx.step().command, Some((req, Ok(_))) if req == r));
        let (_, outcome) = step_until_ended(&mut fx, 300);
        assert_eq!(outcome, Outcome::Completed);
        for i in 0..6 {
            assert!(
                (fx.core.commanded_positions[i] - leg_inner.commanded_positions[i]).abs() < 1e-3,
                "joint {i} parity mismatch: rt={} legacy={}",
                fx.core.commanded_positions[i],
                leg_inner.commanded_positions[i]
            );
        }
    }

    // --- Part B (Majors 11, 12, 13 + Minor 2): Real production adapters on live OWNER_EXECUTOR ---
    let mut exec_cfg = ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    // Use fast valid close speed on both gripper_policy and damiao_gripper (pos_force clamps
    // to min(gripper_policy.close_speed_rad_s, damiao_gripper.close_speed_rad_s)) and start
    // the jaw open so the authentic rebot_dm grasp lifecycle finishes in ~12s wall-clock.
    if let Some(gp) = exec_cfg.gripper_policy.as_mut() {
        gp.close_speed_rad_s = Some(2.0);
    }
    if let Some(dg) = exec_cfg.damiao_gripper.as_mut() {
        dg.close_speed_rad_s = 2.0;
    }
    let open_j7 = exec_cfg.gripper_policy.as_ref().map_or(0.0, |g| g.open_rad);
    let sup = Arc::new(ArmSupervisor::new(exec_cfg));
    sup.set_lease_enabled(false).await;
    let mut start_q = INSPECT.to_vec();
    start_q[6] = open_j7;
    let mut pose_q = start_q.clone();
    pose_q[0] += 0.05;
    let seq_id;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = start_q.clone();
        inner.measured_positions = start_q.clone();
        inner.target_positions = start_q.clone();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        for _ in 0..3 {
            inner.tick(DT);
        }
        let (saved_pose, _) = armd::poses::save(&mut inner, &json!({"name": "seq_p1", "positions": pose_q})).unwrap();
        let (saved_seq, _) = armd::sequences::save(
            &mut inner,
            &json!({
                "name": "exec_seq",
                "steps": [
                    {"type": "pose", "pose_id": saved_pose.id, "speed_scale": 1.0, "gripper": "keep"},
                    {"type": "wait", "s": 0.02}
                ]
            }),
        )
        .unwrap();
        seq_id = saved_seq.id;
    }

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    for id in 31..=36 {
        sup.register_ws_connection(id).await.unwrap();
    }
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    // Verify legacy shell rejects lock-path admission (`OWNER_TRANSFERRED`).
    assert!(
        sup.inner().write().await.acquire_lease("probe").is_err(),
        "legacy shell MUST be detached"
    );
    for _ in 0..100 {
        if sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Plug in a test external reply sink via PR L's single hook so macro terminals are delivered.
    let sink_replies = Arc::new(Mutex::new(Vec::<(ClientId, Reply)>::new()));
    {
        let sink_out = Arc::clone(&sink_replies);
        handle.attach_external_reply_sink(Arc::new(move |c, r| {
            sink_out.lock().unwrap().push((c, r));
            DeliveryDecision::Delivered
        }));
    }

    let wait_macro_end = |sup: Arc<ArmSupervisor>, handle: armd::rt_core::executor::RtExecutorHandle, action_id: u64| {
        let sink_replies = Arc::clone(&sink_replies);
        async move {
            let dl = Instant::now() + Duration::from_secs(45);
            while Instant::now() < dl {
                handle.pump_broker_replies();
                if let Some(o) = sink_replies.lock().unwrap().iter().find_map(|(_, r)| match r {
                    Reply::Terminal { action, outcome, .. } if *action == ActionId(action_id) => Some(*outcome),
                    _ => None,
                }) {
                    return o;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            let latest = sup.hub().latest();
            panic!(
                "macro {action_id} did not emit terminal in time; macro={:?} motion_active={:?} safety={:?}",
                latest.as_ref().map(|s| s.rt.owner.macro_state),
                latest.as_ref().map(|s| s.rt.owner.motion_active),
                latest.as_ref().map(|s| &s.obs.safety_state),
            );
        }
    };

    // 1) Adapter 2: sequences::prepare_run_from_snapshot refuses a missing recording dataset
    //    (404 DATASET_NOT_FOUND) BEFORE starting any parent macro on executor!
    {
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 31;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
        let err = armd::sequences::prepare_run_from_snapshot(
            &snap,
            &json!({"id": seq_id, "record": {"dataset": "missing_demo"}}),
            snap.owner.authority.epoch,
            true,
        )
        .expect_err("missing recording dataset must be refused BEFORE BeginMacro on executor");
        let api = armd::api::rest::ApiError::from(err);
        assert_eq!(api.code.as_deref(), Some("DATASET_NOT_FOUND"), "{api:?}");
        assert!(!sup.hub().latest().unwrap().rt.owner.macro_state.alive, "no parent macro may start");
    }

    // 2) Major 11: Real sequences::begin_executor end-to-end on detached shell!
    {
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 32;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
        let plan = armd::sequences::prepare_run_from_snapshot(&snap, &json!({"id": seq_id}), snap.owner.authority.epoch, true).unwrap();
        let (id, _) = armd::sequences::begin_executor(&sup, adm, plan).await.unwrap();
        let outcome = wait_macro_end(sup.clone(), handle.clone(), id).await;
        assert_eq!(outcome, Outcome::Completed, "sequences::begin_executor must complete");
        let q_end = sup.hub().latest().unwrap().rt.commanded_positions[0];
        assert!((q_end - pose_q[0]).abs() < 5e-3, "expected J1={}, got {q_end}", pose_q[0]);
    }

    // 3) Major 11: Real place::begin_executor end-to-end on detached shell!
    {
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 33;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
        let tcp0 = armd::motion::start_pose(&snap.plan_input_in(GoalFrame::Tcp).unwrap()).p;
        let pending = armd::place::prepare_from_snapshot(
            &snap,
            &json!({"position": [tcp0[0] + 0.02, tcp0[1], tcp0[2]], "approach_height_m": 0.02, "release": false, "retreat": false}),
            snap.owner.authority.epoch,
        )
        .unwrap();
        let planned = armd::place::plan(pending).unwrap();
        let (id, _) = armd::place::begin_executor(&sup, adm, planned).await.unwrap();
        let outcome = wait_macro_end(sup.clone(), handle.clone(), id).await;
        assert_eq!(outcome, Outcome::Completed, "place::begin_executor must complete");
    }

    // 4) Majors 11 & 12: Real grasp::begin_executor on detached shell (runs approach/centring/align/preopen/descend/close/lift without touching sup.inner()!)
    {
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 34;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
        let setup = snap.env.grasp_setup.clone().unwrap();
        let pose_end = snap.env.config.arm_model.fk_f32(&snap.commanded_positions).unwrap();
        let cam = armd::grasp::CamModel::assumed(&snap.env.config.arm_model, &setup, &snap.env.config.camera_mount);
        // Project a reachable table point directly below the current end-link so approach is always IK-reachable!
        let target_pt = [pose_end.p[0], pose_end.p[1], setup.table_z_m + setup.grasp_height_m];
        let proj = cam.project(&pose_end, &target_pt).unwrap_or([cam.cx, cam.cy]);
        let uv = cam.to_norm(proj);
        let box_norm = [
            (uv[0] - 0.04).clamp(0.05, 0.90),
            (uv[1] - 0.04).clamp(0.05, 0.90),
            (uv[0] + 0.04).clamp(0.10, 0.95),
            (uv[1] + 0.04).clamp(0.10, 0.95),
        ];
        let vision: Arc<dyn armd::grasp::DetectionSource> = Arc::new(DetachedMockVision::new());
        let pending = armd::grasp::prepare_from_snapshot(
            &snap,
            &json!({
                "label": "cube",
                "box_norm": box_norm,
                "approach": "direct",
                "overrides": {
                    "approach_speed_mps": 0.15,
                    "descend_speed_mps": 0.15,
                    "settle_delay_s": 0.05,
                    "height_probe_m": 0.0,
                    "recentre_height_m": 0.0,
                    "post_lift_settle_ms": 50
                }
            }),
            snap.owner.authority.epoch,
        )
        .unwrap();
        let planned = armd::grasp::plan(pending).unwrap();
        let (id, _) = armd::grasp::begin_executor(&sup, adm, vision, planned).await.unwrap();
        let outcome = wait_macro_end(sup.clone(), handle.clone(), id).await;
        // In sim without a physical cube in the jaws, grasp runs through approach/centring/align/
        // preopen/descend/close/held_width_m -> NOT_HOLDING (or CENTRING_FAILED if mock detection
        // stays centered); crucially it MUST NOT fail with OWNER_TRANSFERRED or INTERNAL!
        assert!(
            matches!(
                outcome,
                Outcome::Completed
                    | Outcome::Failed("NOT_HOLDING")
                    | Outcome::Failed("DROPPED")
                    | Outcome::Failed("CENTRING_FAILED")
            ),
            "grasp::begin_executor must run cleanly on detached shell without touching SupervisorLock, got {outcome:?}"
        );
    }

    // 5) Majors 11 & 12: Real measure_multiview::begin_executor on detached shell!
    {
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 35;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
        let vision: Arc<dyn armd::grasp::DetectionSource> = Arc::new(DetachedMockVision::new());
        let plan = armd::measure_multiview::prepare_from_snapshot(
            &snap,
            &json!({"label": "cube", "views": 2, "settle_min_s": 0.5}),
            snap.owner.authority.epoch,
        )
        .unwrap();
        let (id, _) = armd::measure_multiview::begin_executor(&sup, adm, plan, vision).await.unwrap();
        let outcome = wait_macro_end(sup.clone(), handle.clone(), id).await;
        // Mock vision has no snapshot JPEG server, so settle completes cleanly and capture stops
        // cleanly at SNAPSHOT_FAILED / DETECTION_UNAVAILABLE (or completes with measurement stored
        // via owner-aware `sup.action_measurement(id)` — never touching detached `sup.inner()`).
        assert!(
            !matches!(outcome, Outcome::Failed("INTERNAL") | Outcome::Failed("OWNER_TRANSFERRED")),
            "measure_multiview::begin_executor failed with unexpected detached-shell error: {outcome:?}"
        );
        // Verify owner-aware record_action_measurement / action_measurement works with detached shell!
        sup.record_action_measurement(id, json!({"ok": true, "height_m": 0.042})).await;
        let m = sup.action_measurement(id).await.expect("owner-aware measurement readback");
        assert_eq!(m["height_m"], json!(0.042));
    }

    // 6) Major 11 + Minor 2: Real calib_hover::begin_executor + cancellation readback!
    {
        let dir = tempfile::tempdir().unwrap();
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 36;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
        let tcp0 = armd::motion::start_pose(&snap.plan_input_in(GoalFrame::EndLink).unwrap()).p;
        let plan = armd::calib_hover::prepare_from_snapshot(
            &snap,
            &[],
            &json!({
                "candidate_fingerprint": "fp-exec-test",
                "rpy_rad": [0.0, 1.57, 0.0],
                "points": [{
                    "id": "p1",
                    "approach_m": [tcp0[0], tcp0[1], tcp0[2] + 0.04],
                    "hover_m": [tcp0[0], tcp0[1], tcp0[2] + 0.01]
                }]
            }),
            snap.owner.authority.epoch,
        )
        .unwrap();
        let upstream = armd::api::proxy::DetectorUpstream::parse("http://127.0.0.1:1").unwrap();
        let (id, _, job_id) = armd::calib_hover::begin_executor(
            &sup,
            adm,
            plan,
            upstream,
            dir.path().to_path_buf(),
        )
        .await
        .unwrap();
        // Cancel via Step-1 StopRequests and wait for hover worker to persist & update job readback.
        tokio::time::sleep(Duration::from_millis(50)).await;
        let _ = handle.stop_requests().request();
        let outcome = wait_macro_end(sup.clone(), handle.clone(), id).await;
        assert_eq!(outcome, Outcome::Stopped);
        let mut found = None;
        for _ in 0..100 {
            let jobs = sup.hover_jobs_snapshot().await;
            if let Some(j) = jobs.into_iter().find(|j| j.id == job_id && j.state != "running") {
                found = Some(j);
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let job = found.expect("Minor 2: calib_hover job must be updated on executor cancellation");
        assert_eq!(job.state, "cancelled");
        assert!(job.end.is_some(), "Minor 2: cancelled hover job must record end arm state");
        assert!(job.code.is_some(), "Minor 2: cancelled hover job must record owner cancellation code");
    }

    handle.stop_and_join();
}

#[test]
fn review_m15_rt_gripper_opening_records_baseline_samples_and_recycles_spares() {
    let (mut fx, mut b, _clock, _stop) = fixture();
    let g_idx = fx.core.config.gripper_index().unwrap();
    let policy = fx.core.config.gripper_policy.clone().unwrap();
    // Start closed, open fully so opening travel qualifies for free-torque baseline learning.
    fx.core.commanded_positions[g_idx] = policy.closed_rad;
    fx.core.measured_positions[g_idx] = policy.closed_rad;
    fx.core.target_positions[g_idx] = policy.closed_rad;
    assert!(fx.core.gripper_baseline_pending.is_empty());

    let snap = snap_of(&fx);
    let prep = prepare_gripper(&snap, &json!({"command": "open", "speed": 1.0}), GripperBaselineCmd::None).unwrap();
    let slot = prep.to_plan_slot(None).unwrap();
    let w = |dst: &mut PlanSlot| *dst = slot.clone();
    let r = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::InstallPrepared {
                write: &w,
                kind: PreparedActionKind::Gripper,
                prepared_boot: BOOT,
                admission_request_id: 0,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: None,
                gripper_calib_revision: prep.stamp.gripper_calib_revision,
                power_budget_revision: None,
            }),
        )
        .unwrap();
    assert!(matches!(fx.step().command, Some((req, Ok(_))) if req == r));
    let (_, outcome) = step_until_ended(&mut fx, 400);
    assert_eq!(outcome, Outcome::Completed);
    assert!(
        !fx.core.gripper_baseline_pending.is_empty(),
        "Major 15: qualifying RT gripper opening must record MoveSamples into gripper_baseline_pending"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r2_m1_executor_macro_refused_503_without_reply_sink_and_commits_with_hook() {
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
        for _ in 0..3 {
            inner.tick(DT);
        }
    }
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    sup.register_ws_connection(41).await.unwrap();
    for _ in 0..100 {
        if sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(!handle.has_external_reply_sink(), "fresh executor must start without a reply sink");

    // 1) Without a reply sink attached, both begin_macro_executor and place::begin_executor MUST
    //    fail at admission with 503 EXECUTOR_ROUTING_PENDING and commit NOTHING.
    let mut adm = sup.capture_admission_stamp();
    adm.connection_id = 41;
    let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let begin = BeginMacroSpec::new(MacroKind::Place, handle.boot());
    let err = armd::macro_action::begin_macro_executor(
        &sup,
        &snap.stamp,
        begin,
        Instant::now() + DEFAULT_ACTION_DEADLINE,
    )
    .await
    .expect_err("BeginMacro must refuse at admission when no external reply sink is attached");
    assert!(
        matches!(err, armd::ArmError::Rejected { status: 503, ref code, .. } if code == "EXECUTOR_ROUTING_PENDING"),
        "expected 503 EXECUTOR_ROUTING_PENDING, got {err:?}"
    );
    let tcp0 = armd::motion::start_pose(&snap.plan_input_in(GoalFrame::Tcp).unwrap()).p;
    let pending = armd::place::prepare_from_snapshot(
        &snap,
        &json!({"position": [tcp0[0] + 0.02, tcp0[1], tcp0[2]], "approach_height_m": 0.02, "release": false, "retreat": false}),
        snap.owner.authority.epoch,
    )
    .unwrap();
    let planned = armd::place::plan(pending).unwrap();
    let err_place = armd::place::begin_executor(&sup, adm, planned)
        .await
        .expect_err("place::begin_executor must refuse 503 without reply sink");
    assert!(
        matches!(err_place, armd::ArmError::Rejected { status: 503, ref code, .. } if code == "EXECUTOR_ROUTING_PENDING"),
        "expected 503 EXECUTOR_ROUTING_PENDING, got {err_place:?}"
    );
    tokio::time::sleep(Duration::from_millis(25)).await;
    let ms = sup.hub().latest().unwrap().rt.owner.macro_state;
    assert!(!ms.alive && ms.active_id == 0, "refused macro must not commit: {ms:?}");

    // 2) Attach PR L's reply sink hook -> place::begin_executor admits, commits, and delivers
    //    Reply::Terminal directly to the attached sink.
    let delivered = Arc::new(Mutex::new(Vec::<(ClientId, Reply)>::new()));
    {
        let out = Arc::clone(&delivered);
        handle.attach_external_reply_sink(Arc::new(move |c, r| {
            out.lock().unwrap().push((c, r));
            DeliveryDecision::Delivered
        }));
    }
    assert!(handle.has_external_reply_sink());
    let pending2 = armd::place::prepare_from_snapshot(
        &snap,
        &json!({"position": [tcp0[0] + 0.02, tcp0[1], tcp0[2]], "approach_height_m": 0.02, "release": false, "retreat": false}),
        snap.owner.authority.epoch,
    )
    .unwrap();
    let planned2 = armd::place::plan(pending2).unwrap();
    let (id, _) = armd::place::begin_executor(&sup, adm, planned2).await.unwrap();
    let wait_dl = Instant::now() + Duration::from_secs(10);
    let mut found_outcome = None;
    while Instant::now() < wait_dl {
        handle.pump_broker_replies();
        if let Some(o) = delivered.lock().unwrap().iter().find_map(|(c, r)| match r {
            Reply::Terminal { action, outcome, .. } if *c == ClientId(41) && *action == ActionId(id) => Some(*outcome),
            _ => None,
        }) {
            found_outcome = Some(o);
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(found_outcome, Some(Outcome::Completed));

    // 3) Detaching the sink restores fail-closed 503 EXECUTOR_ROUTING_PENDING.
    handle.detach_external_reply_sink();
    assert!(!handle.has_external_reply_sink());
    let snap2 = ActionSnapshot::capture(&sup, sup.capture_admission_stamp(), Instant::now() + DEFAULT_ACTION_DEADLINE)
        .await
        .unwrap();
    let err_after = armd::macro_action::begin_macro_executor(
        &sup,
        &snap2.stamp,
        BeginMacroSpec::new(MacroKind::Place, handle.boot()),
        Instant::now() + DEFAULT_ACTION_DEADLINE,
    )
    .await
    .expect_err("detached sink must refuse 503 again");
    assert!(
        matches!(err_after, armd::ArmError::Rejected { status: 503, ref code, .. } if code == "EXECUTOR_ROUTING_PENDING"),
        "{err_after:?}"
    );
    handle.stop_and_join();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r2_m2_legacy_gripper_and_move_to_pose_ack_precedes_stop_terminal() {
    use futures_util::StreamExt;
    use serde_json::Value;
    struct ResetDelay;
    impl Drop for ResetDelay {
        fn drop(&mut self) {
            armd::api::ws::TEST_DEFERRED_ACK_DELAY_MS.store(0, std::sync::atomic::Ordering::SeqCst);
        }
    }
    let _reset = ResetDelay;

    let (addr, sup) = common::spawn_server(ArmConfig {
        collision: None,
        lease_timeout_ms: 60_000,
        ..ArmConfig::rebot_dm()
    })
    .await;
    sup.start_dispatcher().unwrap();
    let mut pose_q = INSPECT.to_vec();
    pose_q[0] += 0.08;
    {
        let mut inner = sup.inner().write().await;
        inner.commanded_positions = INSPECT.to_vec();
        inner.measured_positions = INSPECT.to_vec();
        inner.target_positions = INSPECT.to_vec();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        armd::poses::save(&mut inner, &json!({"name": "m2_pose", "positions": pose_q})).unwrap();
    }
    sup.tick(DT).await;

    let (mut ws, gen) = common::controller(addr).await;
    common::drain(&mut ws, 40).await;

    armd::api::ws::TEST_DEFERRED_ACK_DELAY_MS.store(120, std::sync::atomic::Ordering::SeqCst);

    for (req_id, path, body, expected_kind) in [
        (101u64, "/api/v1/action/gripper", json!({"command": "open", "speed": 1.0}), "gripper"),
        (102u64, "/api/v1/action/move_to_pose", json!({"name": "m2_pose", "speed_scale": 1.0}), "move_to_pose"),
    ] {
        let epoch = sup.inner().read().await.safety.epoch_id();
        let prev_id = sup
            .inner()
            .read()
            .await
            .last_action
            .as_ref()
            .map_or(0, |a| a.id);

        common::send(
            &mut ws,
            json!({
                "type": "command",
                "request_id": req_id,
                "generation": gen,
                "epoch_id": epoch,
                "path": path,
                "body": body,
            }),
        )
        .await;

        // Wait until the deferred install commits the new action under the supervisor lock,
        // while TEST_DEFERRED_ACK_DELAY_MS holds back the ACK from ordinary_tx.
        let mut installed_id = None;
        let wait_dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < wait_dl {
            if let Some(a) = sup.inner().read().await.last_action.as_ref() {
                if a.id > prev_id && a.kind == expected_kind && a.is_running() {
                    installed_id = Some(a.id);
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let action_id = installed_id.expect("deferred action must install before delay expires");

        // Immediately commit a Stop right after install (BEFORE the 120ms ACK delay finishes!)
        // so `TerminalBroadcaster` receives the `cancelled` terminal while the ACK is still pending.
        {
            let mut inner = sup.inner().write().await;
            assert!(inner.authorized_stop(), "Stop must cancel the newly installed {expected_kind}");
        }
        sup.tick(DT).await;

        // Read WS frames in wire order: ACK for `action_id` MUST arrive strictly BEFORE any
        // terminal observation (`state != "running"`) for `action_id`!
        let mut saw_ack = false;
        let mut saw_cancelled_terminal = false;
        let read_dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < read_dl && !(saw_ack && saw_cancelled_terminal) {
            let remaining = read_dl.saturating_duration_since(Instant::now());
            let frame = tokio::time::timeout(remaining, ws.next())
                .await
                .expect("timed out waiting for ACK + cancelled terminal")
                .expect("ws stream ended")
                .expect("ws error")
                .into_text()
                .unwrap();
            let v: Value = serde_json::from_str(&frame).unwrap();
            if v.get("type").and_then(Value::as_str) == Some("command_ack")
                && v.get("request_id").and_then(Value::as_u64) == Some(req_id)
            {
                assert_eq!(v.get("action_id").and_then(Value::as_u64), Some(action_id));
                assert!(
                    !saw_cancelled_terminal,
                    "M2 violation for {expected_kind}: cancelled terminal overtook command_ack!"
                );
                saw_ack = true;
            }
            if let Some(act) = v.get("action") {
                if act.get("id").and_then(Value::as_u64) == Some(action_id)
                    && act.get("state").and_then(Value::as_str) == Some("cancelled")
                {
                    assert!(
                        saw_ack,
                        "M2 violation for {expected_kind}: cancelled terminal arrived BEFORE command_ack! frame={v}"
                    );
                    assert_eq!(act.get("code").and_then(Value::as_str), Some("STOPPED"));
                    saw_cancelled_terminal = true;
                }
            }
        }
        assert!(saw_ack, "must receive command_ack for {expected_kind}");
        assert!(saw_cancelled_terminal, "must receive cancelled terminal for {expected_kind}");
    }
}

#[test]
fn r2_m3_macro_parent_deadline_stops_output_pre_tx_with_no_advance() {
    let clock = VirtualClock::starting_at(Instant::now());
    let drv = RejectingBusDriver::new(&INSPECT, clock.clone());
    let mut core = SupervisorInner::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    });
    core.lease_enabled = false;
    core.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv.clone())));
    core.commanded_positions = INSPECT.to_vec();
    core.measured_positions = INSPECT.to_vec();
    core.target_positions = INSPECT.to_vec();
    {
        let s = core.current_admission_stamp();
        let a = core.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    for _ in 0..3 {
        core.tick(DT);
    }
    core.register_live_connection(1).unwrap();
    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new_with_clock(
        core,
        BOOT,
        Capacities {
            commands: 8,
            plans: 4,
            replies: 32,
        },
        clock.clone(),
    );
    fx.step();

    // Begin macro with whole-action parent deadline = now + 50ms, and install a long joint child
    // moving J1 from 0.0 -> +0.40 rad (child timeout = 10s >> parent deadline).
    let mut begin = BeginMacroSpec::new(MacroKind::Place, BOOT);
    begin.macro_deadline = Some(fx.now() + Duration::from_millis(50));
    begin.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
    let r_b = b
        .submit(
            ClientId(1),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            Submit::Action(ActionSubmit::BeginMacro(begin)),
        )
        .unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let parent = fx.active().unwrap();
    let _ = drain_all(&mut b);

    let snap = snap_of(&fx);
    let mut target = snap.commanded_positions.clone();
    target[0] += 0.40;
    let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
    let c = ChildSpec {
        ticket: ChildTicket { parent, child_seq: 1 },
        timeout_s: 10.0,
        timeout_code: "CHILD_TIMEOUT",
        contact: None,
    };
    let slot = prepare_child_joint_slot(&snap.commanded_positions, &target, &max_vel, 0.2, None, c).unwrap();
    submit_child(&mut b, ClientId(1), 2, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot, true).unwrap();
    // Tick 1 (t = 20ms, before 50ms parent deadline): installs child and advances one step.
    assert!(fx.step().command.unwrap().1.is_ok());
    let _ = drain_all(&mut b);
    // Tick 2 (t = 40ms, still before 50ms parent deadline): advances cleanly while alive.
    let rep_mid = fx.step();
    assert!(rep_mid.ended.is_none());
    let q_before_expiry = fx.core.commanded_positions.clone();
    let written_before_expiry = drv.last_written.lock().unwrap().clone();
    assert!(
        q_before_expiry[0] > INSPECT[0] + 1e-4,
        "child must be actively advancing before parent deadline: q0={} inspect={}",
        q_before_expiry[0],
        INSPECT[0]
    );
    assert!(
        (written_before_expiry[0] - q_before_expiry[0]).abs() < 1e-5,
        "pre-expiry bus write matches commanded pose"
    );

    // Tick 3 (t = 60ms > 50ms parent deadline): pre-output parent expiry MUST fire BEFORE
    // trajectory stepping and BEFORE bus TX -> ZERO reference advance, stationary hold written,
    // and macro terminates with PLACE_TIMEOUT in THAT tick!
    let rep_exp = fx.step();
    assert_eq!(
        rep_exp.ended,
        Some((parent, Outcome::Failed("PLACE_TIMEOUT"))),
        "expired parent must end with PLACE_TIMEOUT once hold write is accepted"
    );
    assert!(
        (fx.core.commanded_positions[0] - q_before_expiry[0]).abs() < 1e-6,
        "M3 violation: reference advanced after parent deadline! before={} after={}",
        q_before_expiry[0],
        fx.core.commanded_positions[0]
    );
    assert!(
        fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6),
        "M3 violation: velocities must be zeroed on expired parent tick: {:?}",
        fx.core.commanded_velocities
    );
    let written_on_expiry = drv.last_written.lock().unwrap().clone();
    assert!(
        (written_on_expiry[0] - q_before_expiry[0]).abs() < 1e-6,
        "M3 violation: driver transmitted a post-deadline motion target! before={} written={}",
        q_before_expiry[0],
        written_on_expiry[0]
    );
    let reps = drain_all(&mut b);
    assert!(reps.iter().any(|(_, r)| matches!(
        r,
        Reply::Terminal { request, action, outcome: Outcome::Failed("PLACE_TIMEOUT"), .. }
            if *request == r_b && *action == parent
    )));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r2_m6_multiview_measurement_published_before_terminal_on_both_owners() {
    // M6: on BOTH Legacy and Executor owners, `record_action_measurement(id, value)` before
    // terminal commit MUST populate `obs.action.measurement` (and `obs.previous_action.measurement`
    // if a subsequent action starts) BEFORE the terminal is delivered via `TerminalBroadcaster`,
    // `subscribe_telemetry`, `sup.observation()`, and `sup.get_observation()`.
    let payload = json!({
        "method": "motion_stereo",
        "table_z_m": 0.01,
        "objects": [{"label": "mug", "height_m": 0.085, "views_used": 3}]
    });

    // --- 1) Legacy owner ---
    {
        let cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(DT);
        }
        sup.start_dispatcher().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<armd::ArmObservation>::new()));
        let delivered_sink = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            delivered_sink.lock().unwrap().push(obs.clone());
            true
        });

        let id = {
            let mut inner = sup.inner().write().await;
            let ep = inner.safety.epoch_id();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::begin_orchestrated_action(a, "measure_multiview", None, ep).unwrap()
        };
        sup.record_action_measurement(id, payload.clone()).await;
        {
            let mut inner = sup.inner().write().await;
            inner.finish_macro(id, "done", None, None, None, None);
        }
        sup.tick(DT).await;

        let deadline = Instant::now() + Duration::from_secs(3);
        let term = loop {
            if let Some(obs) = delivered
                .lock()
                .unwrap()
                .iter()
                .find(|o| o.action.as_ref().is_some_and(|a| a.id == id))
                .cloned()
            {
                break obs;
            }
            assert!(Instant::now() < deadline, "legacy terminal not delivered");
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            term.action.as_ref().and_then(|a| a.measurement.as_ref()),
            Some(&payload),
            "legacy terminal must carry action.measurement"
        );
        let obs = sup.observation().await.unwrap();
        assert_eq!(obs.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(&payload));
        let obs2 = sup.get_observation().await;
        assert_eq!(obs2.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(&payload));
    }

    // --- 2) Executor owner ---
    {
        let cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.set_lease_enabled(false).await;
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(DT);
        }
        sup.start_dispatcher().unwrap();
        let handle = sup
            .start_executor(armd::rt_core::executor::ExecutorConfig::for_tests(
                Duration::from_millis(20),
            ))
            .await
            .unwrap();
        sup.register_ws_connection(88).await.unwrap();
        sup.register_ws_connection(89).await.unwrap();
        let sink_fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let sink_fired_c = Arc::clone(&sink_fired);
        handle.attach_external_reply_sink(Arc::new(move |_cid, reply| {
            if matches!(reply, Reply::Terminal { outcome: Outcome::Completed, .. }) {
                sink_fired_c.store(true, std::sync::atomic::Ordering::Release);
            }
            armd::rt_core::broker::DeliveryDecision::Delivered
        }));

        let delivered = Arc::new(Mutex::new(Vec::<armd::ArmObservation>::new()));
        let delivered_sink = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            delivered_sink.lock().unwrap().push(obs.clone());
            true
        });
        let mut tel_rx = sup.subscribe_telemetry().unwrap();

        let wait_pub = Instant::now() + Duration::from_secs(3);
        while sup.hub().published_seq() == 0 {
            assert!(Instant::now() < wait_pub);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 88;
        let snap = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let boot = handle.boot();
        let mut spec = BeginMacroSpec::new(MacroKind::MeasureMultiview, boot);
        spec.admission_request_id = adm.request_id;
        let (id, _) = armd::macro_action::begin_macro_executor(
            &sup,
            &snap.stamp,
            spec,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
        )
        .await
        .unwrap();

        // Publish sidecar measurement BEFORE finishing the macro (exact measure_multiview.rs:2637 order).
        sup.record_action_measurement(id, payload.clone()).await;

        // Even before terminal commit, `sup.observation()` reflects the recorded measurement on the running action.
        let running_obs = sup.observation().await.unwrap();
        assert_eq!(
            running_obs.action.as_ref().and_then(|a| a.measurement.as_ref()),
            Some(&payload),
            "executor observation must reflect action.measurement immediately after record_action_measurement"
        );

        let ctx = armd::macro_action::MacroCtx {
            sup: sup.clone(),
            id,
            stamp: adm,
            deadline: Instant::now() + Duration::from_secs(30),
            phase_timeout_s: 30.0,
            timeout_code: "MEASURE_TIMEOUT",
            timeout_message: String::new(),
        };
        ctx.finish(
            armd::rt_core::requests::actions::FinishState::Done,
            None,
            None,
            None,
            None,
            HeldUpdate::Unchanged,
        )
        .await;

        // Wait for TerminalBroadcaster to deliver the terminal observation for `id`.
        let deadline = Instant::now() + Duration::from_secs(5);
        let term = loop {
            if let Some(obs) = delivered
                .lock()
                .unwrap()
                .iter()
                .find(|o| o.action.as_ref().is_some_and(|a| a.id == id))
                .cloned()
            {
                break obs;
            }
            assert!(Instant::now() < deadline, "executor terminal not delivered");
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            term.action.as_ref().map(|a| (a.kind.as_str(), a.state.as_str())),
            Some(("measure_multiview", "done"))
        );
        assert_eq!(
            term.action.as_ref().and_then(|a| a.measurement.as_ref()),
            Some(&payload),
            "M6: executor terminal observation MUST carry action.measurement before delivery"
        );

        // `sup.observation()`, `sup.get_observation()`, and telemetry stream must ALL carry it.
        let obs = sup.observation().await.unwrap();
        assert_eq!(obs.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(&payload));
        let obs2 = sup.get_observation().await;
        assert_eq!(obs2.action.as_ref().and_then(|a| a.measurement.as_ref()), Some(&payload));
        let mut saw_tel_measurement = false;
        while let Ok(o) = tel_rx.try_recv() {
            if o.action.as_ref().is_some_and(|a| a.id == id && !a.is_running() && a.measurement.as_ref() == Some(&payload)) {
                saw_tel_measurement = true;
            }
        }
        assert!(saw_tel_measurement, "telemetry stream must also carry action.measurement on executor");

        // Starting a subsequent action pushes `id` to `previous_action`; measurement must remain attached!
        let mut adm2 = sup.capture_admission_stamp();
        adm2.connection_id = 89;
        let snap2 = ActionSnapshot::capture(&sup, adm2, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let mut spec2 = BeginMacroSpec::new(MacroKind::Grasp, boot);
        spec2.admission_request_id = adm2.request_id;
        let (id2, _) = armd::macro_action::begin_macro_executor(
            &sup,
            &snap2.stamp,
            spec2,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
        )
        .await
        .unwrap();
        assert_ne!(id2, id);
        let wait_prev = Instant::now() + Duration::from_secs(3);
        let obs_after = loop {
            let o = sup.observation().await.unwrap();
            if o.action.as_ref().is_some_and(|a| a.id == id2) {
                break o;
            }
            assert!(Instant::now() < wait_prev);
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            obs_after.previous_action.as_ref().and_then(|a| a.measurement.as_ref()),
            Some(&payload),
            "previous_action.measurement must remain attached on executor after next action starts"
        );

        handle.stop_and_join();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r3_m4_replay_pose_change_during_planning_refused_never_installs_unswept_curve() {
    // Round 3 M4: if commanded_positions changes while replay planning runs off-thread,
    // BOTH Legacy and Executor `MacroCtx::install_replay_child` MUST compare against the
    // prepared approach's original start (`plan.input.start` / `plan.prepared_approach.start`)
    // and refuse with `PLAN_STALE` — NEVER substituting a newer snapshot's start or rebuilding
    // an unswept curve at commit. After re-planning from the new pose, the freshly fitted &
    // swept approach commits and completes cleanly. Also proves unswept approaches are refused
    // when workspace policy is active.
    let tmp = tempfile::tempdir().unwrap();
    let mut frame0 = INSPECT;
    frame0[0] += 0.14;
    let mut frames = Vec::new();
    for k in 0..=20 {
        let f = k as f32 / 20.0;
        let mut q = frame0.to_vec();
        q[0] -= 0.05 * f;
        frames.push(armd::recording::RecordFrame {
            timestamp: 10.0 + k as f64 * 0.02,
            observation_state: q.clone(),
            action: q,
        });
    }
    armd::recording::write_frames_to_parquet(&tmp.path().join("ep_r3_m4.parquet"), &frames)
        .unwrap();

    // --- 1) Legacy owner: pose drifts during off-thread planning -> PLAN_STALE ---
    {
        let cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        assert!(cfg.workspace.is_some());
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.set_lease_enabled(false).await;
        let (parent_id, ws_rev, cal_rev, epoch, adm) = {
            let mut inner = sup.inner().write().await;
            inner.episodes = armd::recording::EpisodeStore::new(Some(tmp.path().to_path_buf()));
            inner.commanded_positions = INSPECT.to_vec();
            inner.measured_positions = INSPECT.to_vec();
            inner.target_positions = INSPECT.to_vec();
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            inner.tick(DT);
            let env = armd::rt_core::requests::actions::ActionPrepEnv::from_inner(&inner);
            sup.hub().update_prep_env(Arc::new(env));
            let ep = inner.safety.epoch_id();
            let ws_rev = inner.workspace_revision;
            let cal_rev = inner.calibration_revision;
            let adm = inner.current_admission_stamp();
            let a2 = inner.check_admission(adm).unwrap();
            let id = SupervisorInner::begin_sequence_action(a2, None, ep).unwrap();
            (id, ws_rev, cal_rev, ep, adm)
        };
        let ctx = armd::macro_action::MacroCtx {
            sup: sup.clone(),
            id: parent_id,
            stamp: adm,
            deadline: Instant::now() + Duration::from_secs(30),
            phase_timeout_s: 20.0,
            timeout_code: "SEQUENCE_TIMEOUT",
            timeout_message: String::new(),
        };
        // Plan replay off-thread at INSPECT (`Q_old`).
        let snap_old = ctx.snapshot(ctx.deadline).await.unwrap();
        let req_r = armd::replay::ReplayRequest {
            episode: "ep_r3_m4".into(),
            speed: 1.0,
            gripper: false,
            tag: None,
            allow_stale: false,
        };
        let input_old = snap_old.replay_input(req_r.clone(), epoch).unwrap();
        let plan_old = armd::replay::plan(input_old).unwrap();
        assert!(
            plan_old.prepared_approach.as_ref().is_some_and(|a| a.workspace_swept),
            "off-thread replay plan must record workspace_swept = true"
        );
        // Simulate commanded pose changing to `Q_new` while planning ran!
        {
            let mut inner = sup.inner().write().await;
            inner.commanded_positions[0] += 0.08;
            inner.measured_positions[0] += 0.08;
            inner.target_positions[0] += 0.08;
            inner.tick(DT);
        }
        let err = ctx
            .install_replay_child("replay", plan_old, Some(ws_rev), Some(cal_rev))
            .await
            .expect_err("Legacy install_replay_child MUST refuse PLAN_STALE when pose changed during planning");
        assert!(
            matches!(err, armd::macro_action::Stop::Fail(ref code, _) if code == "PLAN_STALE"),
            "expected PLAN_STALE on legacy owner, got {err:?}"
        );
        assert!(
            sup.inner().read().await.active_trajectory.is_none(),
            "no unswept curve may be installed on legacy owner"
        );
    }

    // --- 2) Executor owner: pose moves via child motion during off-thread planning -> PLAN_STALE ---
    {
        let cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        assert!(cfg.workspace.is_some());
        let sup = Arc::new(ArmSupervisor::new(cfg));
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
            inner.tick(DT);
            let env = armd::rt_core::requests::actions::ActionPrepEnv::from_inner(&inner);
            sup.hub().update_prep_env(Arc::new(env));
        }
        let handle = sup
            .start_executor(armd::rt_core::executor::ExecutorConfig::for_tests(
                Duration::from_millis(5),
            ))
            .await
            .unwrap();
        sup.register_ws_connection(91).await.unwrap();
        handle.attach_external_reply_sink(Arc::new(|_cid, _reply| {
            armd::rt_core::broker::DeliveryDecision::Delivered
        }));
        let wait_pub = Instant::now() + Duration::from_secs(3);
        while sup.hub().published_seq() == 0 {
            assert!(Instant::now() < wait_pub);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let mut adm = sup.capture_admission_stamp();
        adm.connection_id = 91;
        let snap0 = ActionSnapshot::capture(&sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE)
            .await
            .unwrap();
        let ws_rev = snap0.owner.authority.config_revision;
        let cal_rev = snap0.owner.calibration_revision;
        let epoch = snap0.owner.authority.epoch;
        let mut begin = BeginMacroSpec::new(MacroKind::Sequence, handle.boot());
        begin.admission_request_id = adm.request_id;
        let (parent_id, _) = armd::macro_action::begin_macro_executor(
            &sup,
            &snap0.stamp,
            begin,
            Instant::now() + DEFAULT_ACTION_DEADLINE,
        )
        .await
        .unwrap();
        let ctx = armd::macro_action::MacroCtx {
            sup: sup.clone(),
            id: parent_id,
            stamp: adm,
            deadline: Instant::now() + Duration::from_secs(30),
            phase_timeout_s: 20.0,
            timeout_code: "SEQUENCE_TIMEOUT",
            timeout_message: String::new(),
        };

        // Plan replay off-thread at INSPECT (`Q_old`).
        let snap_old = ctx.snapshot(ctx.deadline).await.unwrap();
        let req_r = armd::replay::ReplayRequest {
            episode: "ep_r3_m4".into(),
            speed: 1.0,
            gripper: false,
            tag: None,
            allow_stale: false,
        };
        let input_old = snap_old.replay_input(req_r.clone(), epoch).unwrap();
        let plan_old = armd::replay::plan(input_old).unwrap();

        // While `plan_old` was prepared for `Q_old`, move the arm to `Q_new` via a real child!
        let mut q_new = snap_old.commanded_positions.clone();
        q_new[0] += 0.08;
        let limits = snap_old.env.planning_limits(Some(0.8));
        ctx.install_joint_child("drift", q_new.clone(), limits, true, Some(ws_rev), Some(cal_rev))
            .await
            .expect("drift child must complete");
        let q_at_new = sup.get_observation().await.positions;
        assert!((q_at_new[0] - q_new[0]).abs() < 2e-3);

        // Now call `install_replay_child` with the stale `plan_old` -> MUST refuse `PLAN_STALE`
        // (even though `install_replay_child` captures a fresh snapshot at `Q_new` internally!).
        let err = ctx
            .install_replay_child("replay", plan_old, Some(ws_rev), Some(cal_rev))
            .await
            .expect_err("Executor install_replay_child MUST refuse PLAN_STALE when pose moved during planning");
        assert!(
            matches!(err, armd::macro_action::Stop::Fail(ref code, _) if code == "PLAN_STALE"),
            "expected PLAN_STALE on executor owner, got {err:?}"
        );
        // Arm must NOT have moved along any unswept curve!
        let q_after_refusal = sup.get_observation().await.positions;
        assert!(
            (q_after_refusal[0] - q_new[0]).abs() < 2e-3,
            "refused stale replay must not move arm from Q_new"
        );

        // Re-plan off-thread from fresh `Q_new` -> newly fitted & swept curve commits & completes!
        let snap_fresh = ctx.snapshot(ctx.deadline).await.unwrap();
        let input_fresh = snap_fresh.replay_input(req_r, epoch).unwrap();
        let plan_fresh = armd::replay::plan(input_fresh).unwrap();
        assert!(plan_fresh.prepared_approach.as_ref().is_some_and(|a| a.workspace_swept));
        ctx.install_replay_child("replay", plan_fresh, Some(ws_rev), Some(cal_rev))
            .await
            .expect("re-planned & re-swept replay from Q_new must succeed");

        ctx.finish(
            armd::rt_core::requests::actions::FinishState::Done,
            None,
            None,
            None,
            None,
            HeldUpdate::Unchanged,
        )
        .await;
        handle.stop_and_join();
    }

    // --- 3) Direct RT check: unswept approach with active workspace -> PLAN_STALE ---
    {
        let (mut fx, mut b, _clock, _stop) = fixture();
        assert!(fx.core.workspace_policy().is_some());
        let start = fx.core.commanded_positions.clone();
        let mut first = start.clone();
        first[0] += 0.05;
        let mut slot = PlanSlot::default();
        for k in 0..=10 {
            let mut q = first.clone();
            q[0] += 0.01 * (k as f32 / 10.0);
            slot.path.push(&q, k as f64).unwrap();
        }
        let unswept = armd::replay::PreparedReplayApproach::from_endpoints(
            &start, &first, 0.25, None, false,
        )
        .unwrap();
        slot.replay_approach = Some(unswept);
        let tool_digest =
            armd::rt_core::ops::tool_digest_hash(&fx.core.config.active_tool.config_digest);
        slot.primitive = armd::rt_core::ops::Primitive::Action(
            armd::rt_core::requests::actions::StagedAction::Replay {
                child: None,
                dof: start.len() as u8,
                start: unswept.start,
                episode: armd::rt_core::ops::EpisodeRef::new("ep_unswept").unwrap(),
                speed: 1.0,
                gripper: false,
                sample_dt: 0.02,
                duration_s: 0.2,
                power_scale: None,
                approach_duration_s: 0.25,
                approach_power_scale: None,
                tool_digest,
            },
        );
        let req = submit_slot_prepared(
            &mut b,
            ClientId(1),
            1,
            auth(&fx),
            fx.now() + DEFAULT_ACTION_DEADLINE,
            PreparedActionKind::Replay,
            BOOT,
            slot,
            true,
            None,
            None,
        )
        .unwrap();
        let rep = fx.step();
        assert_eq!(
            rep.command,
            Some((req, Err("PLAN_STALE"))),
            "unswept PreparedReplayApproach must be refused PLAN_STALE when workspace policy is active"
        );
        assert!(fx.core.active_trajectory.is_none());
    }
}

#[derive(Clone)]
struct RecordingHoldDriver {
    q: Arc<Mutex<Vec<f32>>>,
    written: Arc<Mutex<Vec<(Vec<f32>, Vec<f32>)>>>,
    read_sleep_ms: Arc<std::sync::atomic::AtomicU64>,
    /// When `Some((limit, step))`, once `set_tx_release_deadline(Some(cap))` installs a cutoff
    /// `<= limit`, the next `gripper_pos_force_info()` inside `mit.rs:823`'s final driver lock
    /// advances `clock` by `step` (simulating a deadline falling inside the TX critical section).
    bump_when_cutoff_le: Arc<Mutex<Option<(Instant, Duration)>>>,
    armed_cutoff: Arc<Mutex<Option<Instant>>>,
    clock: Option<VirtualClock>,
}

impl RecordingHoldDriver {
    fn new(init_q: &[f32], clock: Option<VirtualClock>) -> Self {
        Self {
            q: Arc::new(Mutex::new(init_q.to_vec())),
            written: Arc::new(Mutex::new(Vec::new())),
            read_sleep_ms: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            bump_when_cutoff_le: Arc::new(Mutex::new(None)),
            armed_cutoff: Arc::new(Mutex::new(None)),
            clock,
        }
    }

    fn now(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, VirtualClock::now)
    }

    fn clear_writes(&self) {
        self.written.lock().unwrap().clear();
    }

    fn last_write(&self) -> (Vec<f32>, Vec<f32>) {
        self.written
            .lock()
            .unwrap()
            .last()
            .cloned()
            .expect("driver must have recorded at least one write")
    }
}

impl armd::driver::ArmDriver for RecordingHoldDriver {
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
    fn supports_rt_executor(&self) -> Result<(), &'static str> { Ok(()) }
    fn clock_now(&self) -> Instant { self.now() }
    fn enable_state(&self) -> armd::driver::EnableState { armd::driver::EnableState::Enabled }
    fn is_feedback_valid(&self) -> bool { true }
    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        self.q.lock().unwrap().len() == dof
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        vec![Some(self.now()); self.q.lock().unwrap().len()]
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        vec![10; self.q.lock().unwrap().len()]
    }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
        let ms = self.read_sleep_ms.load(std::sync::atomic::Ordering::SeqCst);
        if ms > 0 {
            std::thread::sleep(Duration::from_millis(ms));
        }
        let q = self.q.lock().unwrap();
        Ok(q.iter()
            .map(|&p| armd::model::JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            })
            .collect())
    }
    fn write_joint_targets(&mut self, targets: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
        let pos: Vec<f32> = targets.iter().map(|t| t.position).collect();
        let vel: Vec<f32> = targets.iter().map(|t| t.velocity.unwrap_or(0.0)).collect();
        self.written.lock().unwrap().push((pos, vel));
        Ok(())
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[armd::driver::JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, armd::driver::ArmDriverError> {
        let pos: Vec<f32> = targets.iter().map(|t| t.position).collect();
        let vel: Vec<f32> = targets.iter().map(|t| t.velocity.unwrap_or(0.0)).collect();
        self.written.lock().unwrap().push((pos, vel));
        Ok(armd::driver::TxBatchOutcome::Complete)
    }
    fn set_torque(&mut self, _enable: bool) -> Result<(), armd::driver::ArmDriverError> { Ok(()) }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        *self.armed_cutoff.lock().unwrap() = deadline;
    }
    fn gripper_pos_force_info(&self) -> Option<armd::driver::GripperPosForceInfo> {
        if let Some(cap) = *self.armed_cutoff.lock().unwrap() {
            let mut guard = self.bump_when_cutoff_le.lock().unwrap();
            if let Some((limit, step)) = *guard {
                if cap <= limit {
                    *guard = None;
                    if let Some(ref c) = self.clock {
                        c.advance(step);
                    }
                }
            }
        }
        None
    }
}

fn driven_inner(drv: RecordingHoldDriver) -> SupervisorInner {
    let mut inner = SupervisorInner::new(ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    });
    inner.lease_enabled = false;
    inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv)));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
    }
    for _ in 0..3 {
        inner.tick(DT);
    }
    inner.register_live_connection(1).unwrap();
    inner
}

#[test]
fn r3_m3_deadline_between_before_output_and_check_sends_no_target_both_owners() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use armd::rt_core::ops::Checkpoint;

    // Helper: construct a fresh Executor ReleaseFixture with a running macro parent + joint child
    // that has already advanced 2 ticks (moving actively along J1 toward +0.40 rad).
    let setup_executor_moving = |parent_offset_ms: Option<u64>, child_timeout_s: f64| {
        let clock = VirtualClock::starting_at(Instant::now());
        let drv = RecordingHoldDriver::new(&INSPECT, Some(clock.clone()));
        let inner = driven_inner(drv.clone());
        let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 4, replies: 32 },
            clock.clone(),
        );
        fx.step();
        let mut begin = BeginMacroSpec::new(MacroKind::Place, BOOT);
        let parent_dl = parent_offset_ms.map(|ms| fx.now() + Duration::from_millis(ms));
        begin.macro_deadline = parent_dl;
        begin.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
        let r_b = b
            .submit(
                ClientId(1),
                1,
                auth(&fx),
                fx.now() + DEFAULT_ACTION_DEADLINE,
                Submit::Action(ActionSubmit::BeginMacro(begin)),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        let parent = fx.active().unwrap();
        let _ = drain_all(&mut b);

        let snap = snap_of(&fx);
        let mut target = snap.commanded_positions.clone();
        target[0] += 0.40;
        let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
        let c = ChildSpec {
            ticket: ChildTicket { parent, child_seq: 1 },
            timeout_s: child_timeout_s,
            timeout_code: "CHILD_TIMEOUT",
            contact: None,
        };
        let slot = prepare_child_joint_slot(&snap.commanded_positions, &target, &max_vel, 0.2, None, c).unwrap();
        submit_child(&mut b, ClientId(1), 2, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot, true).unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        let _ = drain_all(&mut b);
        let rep_mid = fx.step();
        assert!(rep_mid.ended.is_none());
        let q_before = fx.core.commanded_positions.clone();
        assert!(q_before[0] > INSPECT[0] + 1e-4, "child must be moving before expiry test");
        assert!(fx.core.commanded_velocities[0].abs() > 1e-4, "velocity must be non-zero before expiry");
        drv.clear_writes();
        (fx, b, clock, drv, parent, r_b, q_before)
    };

    // === 1a) Executor: PARENT deadline falls strictly BETWEEN BeforeOutput and pre-step Check #1 ===
    {
        // Begin at t0 + 20ms; after begin(+20ms), install(+20ms), mid(+20ms), clock is at t0 + 80ms.
        // Set parent deadline to +65ms from begin (= t0 + 85ms): still alive at BeforeOutput (t0 + 80ms),
        // expired at Check #1 (t0 + 88ms > t0 + 85ms), while still well inside release_deadline (t0 + 100ms).
        let (mut fx, mut b, clock, drv, parent, r_b, q_before) = setup_executor_moving(Some(65), 10.0);
        let base_now = clock.now();
        let parent_dl = fx.core.macro_run.as_ref().unwrap().deadline.unwrap();
        assert!(base_now < parent_dl, "BeforeOutput sample must still be strictly before parent deadline");
        let after_deadline = parent_dl + Duration::from_millis(3);

        let saw_before_output = Arc::new(AtomicBool::new(false));
        let post_bo_reads = Arc::new(AtomicUsize::new(0));
        {
            let flag = Arc::clone(&saw_before_output);
            fx.on_checkpoint(move |cp| {
                if cp == Checkpoint::BeforeOutput {
                    flag.store(true, Ordering::SeqCst);
                }
            });
        }
        let rep = fx.step_with_clock(|| {
            if !saw_before_output.load(Ordering::SeqCst) {
                return base_now;
            }
            let idx = post_bo_reads.fetch_add(1, Ordering::SeqCst);
            if idx == 0 {
                // `BeforeOutput` checkpoint's own sample (`out_ctx.now`): BEFORE deadline!
                base_now
            } else {
                // Every fresh clock read inside `advance_output_with_clock` (including Check #1): AFTER deadline!
                after_deadline
            }
        });
        assert!(post_bo_reads.load(Ordering::SeqCst) >= 3, "must have taken fresh reads after BeforeOutput");
        assert_eq!(rep.ended, Some((parent, Outcome::Failed("PLACE_TIMEOUT"))));
        assert!(
            (fx.core.commanded_positions[0] - q_before[0]).abs() < 1e-6,
            "Executor Check #1 must not advance reference after parent deadline: before={} after={}",
            q_before[0],
            fx.core.commanded_positions[0]
        );
        assert!(fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        let (w_pos, w_vel) = drv.last_write();
        assert!(
            (w_pos[0] - q_before[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6),
            "Executor Check #1 must send stationary hold only, got pos={} vel={:?}",
            w_pos[0],
            w_vel
        );
        let reps = drain_all(&mut b);
        assert!(reps.iter().any(|(_, r)| matches!(
            r,
            Reply::Terminal { request, action, outcome: Outcome::Failed("PLACE_TIMEOUT"), .. }
                if *request == r_b && *action == parent
        )));
    }

    // === 1b) Executor: PARENT deadline falls inside final TX lock (generic ArmDriver, mit.rs:850) ===
    {
        let (mut fx, mut b, clock, drv, parent, r_b, q_before) = setup_executor_moving(Some(65), 10.0);
        let base_now = clock.now();
        let parent_dl = fx.core.macro_run.as_ref().unwrap().deadline.unwrap();
        assert!(base_now < parent_dl);
        let advance_delta = (parent_dl + Duration::from_millis(3)).saturating_duration_since(base_now);
        // Advance the shared VirtualClock strictly inside `mit.rs:823`'s final driver lock once
        // `install_effective_tx_cutoff` has armed `parent_dl` -> `mit.rs:850` TX choke point MUST
        // refuse with 0 writes on this tick, and next tick holds + emits PLACE_TIMEOUT once!
        *drv.bump_when_cutoff_le.lock().unwrap() = Some((parent_dl, advance_delta + Duration::from_millis(1)));
        let rep = fx.step();
        assert!(
            drv.written.lock().unwrap().is_empty(),
            "TX choke point must refuse all writes after parent deadline elapsed before TX"
        );
        assert!(fx.core.safety.state().is_operational(), "deadline refusal must not latch a fault");
        assert!(fx.core.active_trajectory.is_none());
        assert!(fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        assert!(rep.ended.is_none(), "RT+driver defers MacroFinish until next tick's hold write");
        let q_held = fx.core.commanded_positions.clone();
        assert!(q_held[0] > q_before[0], "Step 3 ran before TX refusal; next tick must hold it");

        // Next tick transmits the stationary hold (no further advance) and delivers PLACE_TIMEOUT once.
        let rep_hold = fx.step();
        assert_eq!(rep_hold.ended, Some((parent, Outcome::Failed("PLACE_TIMEOUT"))));
        assert!((fx.core.commanded_positions[0] - q_held[0]).abs() < 1e-6);
        let (w_pos, w_vel) = drv.last_write();
        assert!(
            (w_pos[0] - q_held[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6),
            "next tick after refused TX must send stationary hold, got pos={} vel={:?}",
            w_pos[0],
            w_vel
        );
        // Subsequent ticks emit no duplicate terminal.
        assert!(fx.step().ended.is_none());
        let reps = drain_all(&mut b);
        let term_count = reps
            .iter()
            .filter(|(_, r)| matches!(
                r,
                Reply::Terminal { request, action, outcome: Outcome::Failed("PLACE_TIMEOUT"), .. }
                    if *request == r_b && *action == parent
            ))
            .count();
        assert_eq!(term_count, 1, "PLACE_TIMEOUT terminal must be delivered exactly once");
    }

    // === 1c) Executor: CHILD deadline falls before pre-step AND inside TX critical section ===
    for expire_at_tx in [false, true] {
        // Parent deadline far away (+10s), child timeout = 0.045s (45ms from install):
        // after install(+20ms) + mid(+20ms) = 40ms elapsed -> 5ms left at BeforeOutput!
        let (mut fx, _b, clock, drv, parent, _r_b, q_before) = setup_executor_moving(Some(10_000), 0.045);
        let base_now = clock.now();
        let child_dl = fx.core.macro_run.as_ref().unwrap().child.deadline.unwrap();
        assert!(base_now < child_dl, "child must still be before deadline at BeforeOutput");
        let after_child_dl = child_dl + Duration::from_millis(3);

        let rep = if !expire_at_tx {
            let saw_before_output = Arc::new(AtomicBool::new(false));
            let post_bo_reads = Arc::new(AtomicUsize::new(0));
            {
                let flag = Arc::clone(&saw_before_output);
                fx.on_checkpoint(move |cp| {
                    if cp == Checkpoint::BeforeOutput {
                        flag.store(true, Ordering::SeqCst);
                    }
                });
            }
            fx.step_with_clock(|| {
                if !saw_before_output.load(Ordering::SeqCst) {
                    return base_now;
                }
                let idx = post_bo_reads.fetch_add(1, Ordering::SeqCst);
                if idx == 0 { base_now } else { after_child_dl }
            })
        } else {
            let delta = after_child_dl.saturating_duration_since(base_now);
            *drv.bump_when_cutoff_le.lock().unwrap() = Some((child_dl, delta + Duration::from_millis(1)));
            fx.step()
        };
        assert!(rep.ended.is_none(), "child deadline expiry must leave parent running");
        assert_eq!(fx.active(), Some(parent));
        assert!(fx.core.safety.state().is_operational());
        assert!(matches!(
            fx.core.macro_run.as_ref().map(|m| m.sub),
            Some(SubState::Failed { code, .. }) if code.as_str() == "CHILD_TIMEOUT"
        ));
        assert!(fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        if !expire_at_tx {
            assert!((fx.core.commanded_positions[0] - q_before[0]).abs() < 1e-6);
            let (w_pos, w_vel) = drv.last_write();
            assert!((w_pos[0] - q_before[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6));
        } else {
            assert!(
                drv.written.lock().unwrap().is_empty(),
                "child deadline refused TX must send 0 writes on that tick"
            );
            let q_held = fx.core.commanded_positions.clone();
            assert!(fx.step().ended.is_none());
            let (w_pos, w_vel) = drv.last_write();
            assert!((w_pos[0] - q_held[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6));
        }
    }

    // Helper: construct a fresh Legacy SupervisorInner with a running macro parent + joint child.
    let setup_legacy_moving = |clock: Option<VirtualClock>, parent_deadline: Option<Instant>, child_deadline: Option<Instant>| {
        let drv = RecordingHoldDriver::new(&INSPECT, clock);
        let mut inner = driven_inner(drv.clone());
        let ep = inner.safety.epoch_id();
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        let id = SupervisorInner::begin_orchestrated_action(a, "place", None, ep).unwrap();
        if let Some(m) = inner.macro_run.as_mut() {
            m.deadline = parent_deadline;
            m.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
        }
        let mut target = inner.commanded_positions.clone();
        target[0] += 0.40;
        let limits = inner.config.limits.clone();
        let s2 = inner.current_admission_stamp();
        let a2 = inner.check_admission(s2).unwrap();
        SupervisorInner::install_macro_quintic(a2, id, target, &limits).unwrap();
        if let Some(cdl) = child_deadline {
            inner.arm_macro_child(id, cdl, "CHILD_TIMEOUT", "child timed out".into());
        }
        inner.tick(DT);
        inner.tick(DT);
        let q_before = inner.commanded_positions.clone();
        assert!(q_before[0] > INSPECT[0] + 1e-4, "legacy child must be moving before expiry");
        assert!(inner.commanded_velocities[0].abs() > 1e-4, "legacy velocity must be non-zero");
        drv.clear_writes();
        (inner, drv, id, q_before)
    };

    // === 2a) Legacy: PARENT deadline before pre-step AND inside TX critical section ===
    for expire_at_tx in [false, true] {
        let clock = VirtualClock::starting_at(Instant::now());
        let parent_dl = clock.now() + Duration::from_secs(60);
        let (mut inner, drv, id, q_before) = setup_legacy_moving(Some(clock.clone()), Some(parent_dl), None);
        if !expire_at_tx {
            let after_dl = parent_dl + Duration::from_millis(5);
            let mut clock_fn = || after_dl;
            inner.tick_control_legacy_with_clock(DT, &mut clock_fn);
            assert!((inner.commanded_positions[0] - q_before[0]).abs() < 1e-6);
            assert!(inner.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
            let (w_pos, w_vel) = drv.last_write();
            assert!((w_pos[0] - q_before[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6));
        } else {
            // Advance clock to 5ms before deadline, then advance past deadline inside `mit.rs:823`
            // final driver lock -> TX choke point refuses with 0 writes!
            clock.advance(Duration::from_secs(60) - Duration::from_millis(5));
            *drv.bump_when_cutoff_le.lock().unwrap() = Some((parent_dl, Duration::from_millis(10)));
            let mut clock_fn = || clock.now();
            inner.tick_control_legacy_with_clock(DT, &mut clock_fn);
            assert!(
                drv.written.lock().unwrap().is_empty(),
                "Legacy TX choke point must refuse all writes after parent deadline"
            );
            assert!(inner.safety.state().is_operational());
            assert!(inner.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
            let q_held = inner.commanded_positions.clone();
            // Next tick sends stationary hold.
            inner.tick_control_legacy_with_clock(DT, &mut clock_fn);
            let (w_pos, w_vel) = drv.last_write();
            assert!((w_pos[0] - q_held[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6));
        }
        let act = inner.last_action.as_ref().expect("finished action record");
        assert_eq!(act.id, id);
        assert_eq!(act.state, "failed");
        assert_eq!(act.code.as_deref(), Some("PLACE_TIMEOUT"));
    }

    // === 2b) Legacy: CHILD deadline inside TX critical section (parent alive, next tick holds) ===
    {
        let clock = VirtualClock::starting_at(Instant::now());
        let parent_dl = clock.now() + Duration::from_secs(120);
        let child_dl = clock.now() + Duration::from_secs(60);
        let (mut inner, drv, id, _q_before) =
            setup_legacy_moving(Some(clock.clone()), Some(parent_dl), Some(child_dl));
        clock.advance(Duration::from_secs(60) - Duration::from_millis(5));
        *drv.bump_when_cutoff_le.lock().unwrap() = Some((child_dl, Duration::from_millis(10)));
        let mut clock_fn = || clock.now();
        inner.tick_control_legacy_with_clock(DT, &mut clock_fn);
        assert!(drv.written.lock().unwrap().is_empty(), "0 writes on refused tick");
        assert!(inner.safety.state().is_operational());
        assert!(inner.macro_alive(id), "child timeout must keep legacy parent alive");
        assert!(matches!(
            inner.macro_run.as_ref().map(|m| m.sub),
            Some(SubState::Failed { code, .. }) if code.as_str() == "CHILD_TIMEOUT"
        ));
        assert!(inner.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        let q_held = inner.commanded_positions.clone();
        inner.tick_control_legacy_with_clock(DT, &mut clock_fn);
        let (w_pos, w_vel) = drv.last_write();
        assert!((w_pos[0] - q_held[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6));
    }

    // === 2c) Legacy: real wall-clock `inner.tick(DT)` where deadline passes during RX read ===
    {
        let (mut inner, drv, id, q_before) = setup_legacy_moving(None, None, None);
        // Set parent deadline 10ms in the future (still alive when `tick()` starts!), and stall
        // driver RX read for 30ms so the fresh `Instant::now()` before Step 3 catches expiry.
        inner.macro_run.as_mut().unwrap().deadline = Some(Instant::now() + Duration::from_millis(10));
        drv.read_sleep_ms.store(30, Ordering::SeqCst);
        let obs = inner.tick(DT);
        assert!(
            (inner.commanded_positions[0] - q_before[0]).abs() < 1e-6,
            "Legacy real-clock tick must not advance after mid-tick parent deadline"
        );
        assert!(inner.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        let (w_pos, w_vel) = drv.last_write();
        assert!(
            (w_pos[0] - q_before[0]).abs() < 1e-6 && w_vel.iter().all(|v| v.abs() < 1e-6),
            "Legacy real-clock tick must transmit stationary hold only"
        );
        assert_eq!(obs.action.as_ref().map(|a| (a.id, a.state.as_str(), a.code.as_deref())), Some((id, "failed", Some("PLACE_TIMEOUT"))));
    }
}

/// Wrapper around a real `DamiaoArmDriver<FakeBus>` that can advance its shared `MockClock`
/// strictly INSIDE the final driver lock (`mit.rs:823` `gripper_pos_force_info`), simulating a
/// deadline that falls inside the TX critical section right before `write_joint_targets_bounded`.
struct LockCriticalSectionDriver {
    inner: armd::driver::DamiaoArmDriver<mit_support::FakeBus>,
    clock: armd::transport::MockClock,
    armed_cutoff: Option<Instant>,
    bump_when_cutoff_le: Arc<Mutex<Option<(Instant, Duration)>>>,
    bumped_inside_lock: Arc<std::sync::atomic::AtomicUsize>,
}

impl LockCriticalSectionDriver {
    fn new(
        inner: armd::driver::DamiaoArmDriver<mit_support::FakeBus>,
        clock: armd::transport::MockClock,
        bump_when_cutoff_le: Arc<Mutex<Option<(Instant, Duration)>>>,
        bumped_inside_lock: Arc<std::sync::atomic::AtomicUsize>,
    ) -> Self {
        Self {
            inner,
            clock,
            armed_cutoff: None,
            bump_when_cutoff_le,
            bumped_inside_lock,
        }
    }
}

impl armd::driver::ArmDriver for LockCriticalSectionDriver {
    fn capabilities(&self) -> armd::driver::DriverCapabilities { self.inner.capabilities() }
    fn supports_rt_executor(&self) -> Result<(), &'static str> { self.inner.supports_rt_executor() }
    fn clock_now(&self) -> Instant { self.inner.clock_now() }
    fn enable_state(&self) -> armd::driver::EnableState { self.inner.enable_state() }
    fn is_feedback_valid(&self) -> bool { self.inner.is_feedback_valid() }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.inner.feedback_all_within(dof, max_age)
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> { self.inner.feedback_instants() }
    fn feedback_seqs(&self) -> Vec<u64> { self.inner.feedback_seqs() }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) { self.inner.feedback_ages_into(out) }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) { self.inner.feedback_age_ms_into(out) }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_instants_into(out) }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_receive_instants_into(out) }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) { self.inner.feedback_seqs_into(out) }
    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) { self.inner.feedback_queue_bounds_into(out) }
    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) { self.inner.take_recovery_writes_into(out) }
    fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
        self.inner.read_joint_states()
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<armd::model::JointState>) -> Result<(), armd::driver::ArmDriverError> {
        self.inner.read_joint_states_into(out)
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<armd::model::JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), armd::driver::ArmDriverError> {
        self.inner.read_joint_states_budgeted(out, deadline, max_rx_frames)
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<armd::model::JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)
    }
    fn write_joint_targets(&mut self, targets: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
        self.inner.write_joint_targets(targets)
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[armd::driver::JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, armd::driver::ArmDriverError> {
        self.inner.write_joint_targets_bounded(targets)
    }
    fn write_estop_hold_bounded(
        &mut self,
        targets: &[armd::driver::JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, armd::driver::ArmDriverError> {
        self.inner.write_estop_hold_bounded(targets)
    }
    fn discard_pending_tx(&mut self) { armd::driver::ArmDriver::discard_pending_tx(&mut self.inner); }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.armed_cutoff = deadline;
        self.inner.set_tx_release_deadline(deadline);
    }
    fn tx_frame_max_age(&self) -> Duration { self.inner.tx_frame_max_age() }
    fn last_tx_expired_age(&self) -> Option<Duration> { self.inner.last_tx_expired_age() }
    fn oldest_pending_tx_age(&self) -> Option<Duration> { self.inner.oldest_pending_tx_age() }
    fn pending_motion_tx_len(&self) -> usize { self.inner.pending_motion_tx_len() }
    fn pending_hold_tx_len(&self) -> usize { self.inner.pending_hold_tx_len() }
    fn last_target_tx_sent_mask(&self) -> u64 { self.inner.last_target_tx_sent_mask() }
    fn last_hold_tx_sent_any(&self) -> bool { self.inner.last_hold_tx_sent_any() }
    fn set_torque(&mut self, enable: bool) -> Result<(), armd::driver::ArmDriverError> {
        self.inner.set_torque(enable)
    }
    fn gripper_pos_force_info(&self) -> Option<armd::driver::GripperPosForceInfo> {
        if let Some(cap) = self.armed_cutoff {
            let mut guard = self.bump_when_cutoff_le.lock().unwrap();
            if let Some((limit, step)) = *guard {
                if cap <= limit {
                    *guard = None;
                    self.clock.advance(step);
                    self.bumped_inside_lock.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                }
            }
        }
        self.inner.gripper_pos_force_info()
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn r4_m3_single_effective_cutoff_blocks_tx_critical_section_on_both_owners() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use armd::driver::{ArmDriver, DamiaoArmDriver, TxBudget};
    use armd::protocol::damiao::DamiaoLimits;
    use armd::transport::MockClock;
    use mit_support::FakeBus;

    let dm_limits = || -> Vec<DamiaoLimits> {
        ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect()
    };

    let build_damiao_supervisor = |clock: &MockClock,
                                   bump_lock: Arc<Mutex<Option<(Instant, Duration)>>>,
                                   bumped_count: Arc<AtomicUsize>| {
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            for (m, &q0) in s.motors.iter_mut().zip(INSPECT.iter()) {
                m.q = q0;
            }
            s.immediate_replies = false;
            s.follow_commands = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_tx_budget(TxBudget::default());
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let wrapped = LockCriticalSectionDriver::new(drv, clock.clone(), bump_lock, bumped_count);
        let cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(wrapped));
        inner.lease_enabled = false;
        inner.tick(DT);
        let stamp = inner.current_admission_stamp();
        let admitted = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).expect("arm damiao");
        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            inner.tick(DT);
        }
        assert!(inner.safety.state().is_operational());
        inner.register_live_connection(1).unwrap();
        bus.clear_log();
        (inner, bus)
    };

    // === 1) EXECUTOR OWNER (real DamiaoArmDriver<FakeBus> + shared MockClock) ===
    // Test BOTH critical-section expiry points:
    //   (a) inside final driver lock (`mit.rs:823` -> `mit.rs:850`, before frame 0): 0 frames on wire!
    //   (b) mid-batch inside `DamiaoArmDriver::send()` (frame 0 < deadline, `on_tx` advances past
    //       deadline -> frames 1..6 refused before `can.transmit`): 0 frames on wire >= deadline!
    for mid_batch in [false, true] {
        let clock = MockClock::new();
        let bump_lock = Arc::new(Mutex::new(None));
        let bumped_count = Arc::new(AtomicUsize::new(0));
        let (mut inner, bus) =
            build_damiao_supervisor(&clock, Arc::clone(&bump_lock), Arc::clone(&bumped_count));
        inner.rt_executor_owned = true;
        let vclock = VirtualClock::from_mock_clock(clock.clone());
        let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 4, replies: 32 },
            vclock,
        );
        bus.with(|s| s.push_all_feedback());
        fx.step();

        // Begin macro with parent deadline = now + 65ms (after begin +20ms, install +20ms, mid +20ms
        // = 60ms elapsed -> 5ms remaining at the start of the expiry tick!).
        let mut begin = BeginMacroSpec::new(MacroKind::Place, BOOT);
        let parent_dl = fx.now() + Duration::from_millis(65);
        begin.macro_deadline = Some(parent_dl);
        begin.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
        let r_b = b
            .submit(
                ClientId(1),
                1,
                auth(&fx),
                fx.now() + DEFAULT_ACTION_DEADLINE,
                Submit::Action(ActionSubmit::BeginMacro(begin)),
            )
            .unwrap();
        bus.with(|s| s.push_all_feedback());
        assert!(fx.step().command.unwrap().1.is_ok());
        let parent = fx.active().unwrap();
        let _ = drain_all(&mut b);

        let snap = snap_of(&fx);
        let mut target = snap.commanded_positions.clone();
        target[0] += 0.40;
        let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
        let c = ChildSpec {
            ticket: ChildTicket { parent, child_seq: 1 },
            timeout_s: 10.0,
            timeout_code: "CHILD_TIMEOUT",
            contact: None,
        };
        let slot = prepare_child_joint_slot(&snap.commanded_positions, &target, &max_vel, 0.2, None, c).unwrap();
        submit_child(&mut b, ClientId(1), 2, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot, true).unwrap();
        bus.with(|s| s.push_all_feedback());
        assert!(fx.step().command.unwrap().1.is_ok());
        let _ = drain_all(&mut b);

        bus.with(|s| s.push_all_feedback());
        assert!(fx.step().ended.is_none());
        assert!(fx.now() < parent_dl, "must still be strictly before parent_dl");
        assert!(fx.core.commanded_velocities[0].abs() > 1e-4);

        bus.clear_log();
        bus.with(|s| s.push_all_feedback());
        let tx_hook_fired = Arc::new(AtomicBool::new(false));
        if !mid_batch {
            // Advance MockClock past `parent_dl` strictly INSIDE the final driver lock (`mit.rs:823`).
            *bump_lock.lock().unwrap() = Some((parent_dl, Duration::from_millis(8)));
        } else {
            // Frame 0 hits `can.transmit` BEFORE `parent_dl`, then `on_tx` advances `MockClock`
            // past `parent_dl` -> `DamiaoArmDriver::send()` MUST refuse frames 1..6 BEFORE wire TX!
            let clk = clock.clone();
            let fired = Arc::clone(&tx_hook_fired);
            bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    if !fired.swap(true, Ordering::SeqCst) {
                        clk.advance(Duration::from_millis(8));
                    }
                }));
            });
        }

        let rep_exp = fx.step();
        let (tx_frames, tx_times) = bus.with(|s| (s.tx_log.clone(), s.tx_times.clone()));
        assert_eq!(tx_frames.len(), tx_times.len());
        for (idx, t_opt) in tx_times.iter().enumerate() {
            let t = t_opt.expect("FakeBus with MockClock records tx timestamp");
            assert!(
                t < parent_dl,
                "Executor (mid_batch={mid_batch}) transmitted frame #{idx} AT/AFTER parent_dl! t={t:?} dl={parent_dl:?}"
            );
        }
        if !mid_batch {
            assert_eq!(bumped_count.load(Ordering::SeqCst), 1, "clock must have bumped inside final driver lock");
            assert!(tx_frames.is_empty(), "0 frames may hit wire when deadline falls inside final driver lock");
        } else {
            assert!(tx_hook_fired.load(Ordering::SeqCst), "mid-batch on_tx hook must have fired");
            assert_eq!(
                tx_frames.len(),
                1,
                "only pre-deadline frame 0 may hit wire; frames 1..6 must be refused by DamiaoArmDriver::send()"
            );
            bus.with(|s| s.on_tx = None);
        }
        assert!(fx.core.safety.state().is_operational(), "deadline refusal must not latch CAN_TX_STALLED or fault");
        assert!(fx.core.active_trajectory.is_none(), "expired macro must quiesce child immediately");
        assert!(fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        assert!(rep_exp.ended.is_none(), "RT+driver waits for next tick's hold write before terminal");

        // Next tick: stationary hold transmits cleanly (full 7-joint batch, no further advance)
        // and emits `PLACE_TIMEOUT` terminal EXACTLY ONCE!
        let q_hold = fx.core.commanded_positions.clone();
        bus.clear_log();
        bus.with(|s| s.push_all_feedback());
        let rep_hold = fx.step();
        assert_eq!(rep_hold.ended, Some((parent, Outcome::Failed("PLACE_TIMEOUT"))));
        assert!((fx.core.commanded_positions[0] - q_hold[0]).abs() < 1e-6);
        assert!(fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        assert_eq!(bus.tx_log().len(), 7, "hold tick must complete full 7-joint stationary hold batch");

        // Additional ticks: stationary hold, ZERO duplicate terminals!
        for _ in 0..2 {
            bus.with(|s| s.push_all_feedback());
            assert!(fx.step().ended.is_none());
        }
        let reps = drain_all(&mut b);
        let term_count = reps
            .iter()
            .filter(|(_, r)| matches!(
                r,
                Reply::Terminal { request, action, outcome: Outcome::Failed("PLACE_TIMEOUT"), .. }
                    if *request == r_b && *action == parent
            ))
            .count();
        assert_eq!(term_count, 1, "Executor PLACE_TIMEOUT terminal must be delivered exactly once");
    }

    // === 2) EXECUTOR CHILD DEADLINE inside `DamiaoArmDriver::send()` (parent alive) ===
    {
        let clock = MockClock::new();
        let bump_lock = Arc::new(Mutex::new(None));
        let bumped_count = Arc::new(AtomicUsize::new(0));
        let (mut inner, bus) =
            build_damiao_supervisor(&clock, Arc::clone(&bump_lock), Arc::clone(&bumped_count));
        inner.rt_executor_owned = true;
        let vclock = VirtualClock::from_mock_clock(clock.clone());
        let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 4, replies: 32 },
            vclock,
        );
        bus.with(|s| s.push_all_feedback());
        fx.step();
        let mut begin = BeginMacroSpec::new(MacroKind::Place, BOOT);
        begin.macro_deadline = Some(fx.now() + Duration::from_secs(10));
        begin.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
        b.submit(ClientId(1), 1, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, Submit::Action(ActionSubmit::BeginMacro(begin))).unwrap();
        bus.with(|s| s.push_all_feedback());
        assert!(fx.step().command.unwrap().1.is_ok());
        let parent = fx.active().unwrap();
        let _ = drain_all(&mut b);

        let snap = snap_of(&fx);
        let mut target = snap.commanded_positions.clone();
        target[0] += 0.40;
        let max_vel: Vec<f32> = snap.env.planning_limits(None).iter().map(|l| l.max_velocity).collect();
        let c = ChildSpec {
            ticket: ChildTicket { parent, child_seq: 1 },
            timeout_s: 0.045,
            timeout_code: "CHILD_TIMEOUT",
            contact: None,
        };
        let slot = prepare_child_joint_slot(&snap.commanded_positions, &target, &max_vel, 0.2, None, c).unwrap();
        submit_child(&mut b, ClientId(1), 2, auth(&fx), fx.now() + DEFAULT_ACTION_DEADLINE, BOOT, slot, true).unwrap();
        bus.with(|s| s.push_all_feedback());
        assert!(fx.step().command.unwrap().1.is_ok());
        let _ = drain_all(&mut b);
        bus.with(|s| s.push_all_feedback());
        assert!(fx.step().ended.is_none());
        let child_dl = fx.core.macro_run.as_ref().unwrap().child.deadline.unwrap();
        assert!(fx.now() < child_dl);

        bus.clear_log();
        bus.with(|s| s.push_all_feedback());
        let clk = clock.clone();
        let fired = Arc::new(AtomicBool::new(false));
        bus.with(|s| {
            s.on_tx = Some(Arc::new(move || {
                if !fired.swap(true, Ordering::SeqCst) {
                    clk.advance(Duration::from_millis(8));
                }
            }));
        });
        assert!(fx.step().ended.is_none());
        bus.with(|s| s.on_tx = None);
        let (tx_frames, tx_times) = bus.with(|s| (s.tx_log.clone(), s.tx_times.clone()));
        assert_eq!(tx_frames.len(), 1, "only frame 0 before child_dl may hit wire");
        assert!(tx_times[0].unwrap() < child_dl);
        assert!(fx.core.safety.state().is_operational());
        assert_eq!(fx.active(), Some(parent), "child timeout must leave parent running");
        assert!(matches!(
            fx.core.macro_run.as_ref().map(|m| m.sub),
            Some(SubState::Failed { code, .. }) if code.as_str() == "CHILD_TIMEOUT"
        ));
        assert!(fx.core.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
    }

    // === 3) LEGACY OWNER (real ArmSupervisor + DamiaoArmDriver<FakeBus> + TerminalBroadcaster) ===
    // Verify BOTH inside final driver lock (0 frames) AND mid-batch inside `DamiaoArmDriver::send()`
    // (0 frames >= deadline), AND assert `TerminalBroadcaster` delivers `PLACE_TIMEOUT` EXACTLY ONCE!
    for mid_batch in [false, true] {
        let clock = MockClock::new();
        let bump_lock = Arc::new(Mutex::new(None));
        let bumped_count = Arc::new(AtomicUsize::new(0));
        let limits = dm_limits();
        let bus = FakeBus::with_clock(&limits, clock.clone());
        bus.with(|s| {
            for (m, &q0) in s.motors.iter_mut().zip(INSPECT.iter()) {
                m.q = q0;
            }
            s.immediate_replies = false;
            s.follow_commands = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits)
            .unwrap()
            .with_tx_budget(TxBudget::default());
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let wrapped = LockCriticalSectionDriver::new(
            drv,
            clock.clone(),
            Arc::clone(&bump_lock),
            Arc::clone(&bumped_count),
        );
        let cfg = ArmConfig {
            collision: None,
            lease_timeout_ms: 600_000,
            // 10 Hz -> 98ms release cap so a 50ms action deadline is strictly tighter than the
            // release cap (never colliding with line 4912's RX cap) while having 49ms margin
            // against CI CPU jitter before reaching the TX critical section.
            control_frequency_hz: 10,
            ..ArmConfig::rebot_dm()
        };
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(wrapped)));
        sup.set_lease_enabled(false).await;
        sup.start_dispatcher().unwrap();
        let delivered = Arc::new(Mutex::new(Vec::<armd::ArmObservation>::new()));
        let delivered_sink = Arc::clone(&delivered);
        let _sub = sup.test_subscribe_action_terminals(move |obs| {
            delivered_sink.lock().unwrap().push(obs.clone());
            true
        });

        let (id, parent_dl) = {
            let mut inner = sup.inner().write().await;
            inner.tick(DT);
            let s = inner.current_admission_stamp();
            let a = inner.check_admission(s).unwrap();
            SupervisorInner::enable_torque(a).unwrap();
            for _ in 0..3 {
                clock.advance(Duration::from_millis(20));
                bus.with(|st| st.push_all_feedback());
                inner.tick(DT);
            }
            let ep = inner.safety.epoch_id();
            let s2 = inner.current_admission_stamp();
            let a2 = inner.check_admission(s2).unwrap();
            let id = SupervisorInner::begin_orchestrated_action(a2, "place", None, ep).unwrap();
            let mut target = inner.commanded_positions.clone();
            target[0] += 0.40;
            let lims = inner.config.limits.clone();
            let s3 = inner.current_admission_stamp();
            let a3 = inner.check_admission(s3).unwrap();
            SupervisorInner::install_macro_quintic(a3, id, target, &lims).unwrap();
            for _ in 0..2 {
                clock.advance(Duration::from_millis(20));
                bus.with(|st| st.push_all_feedback());
                inner.tick(DT);
            }
            // Arm parent deadline 25ms ahead (< 45ms frame_max_age and < 98ms release cap, >> 1ms tick time).
            let dl_owner = Instant::now() + Duration::from_millis(25);
            let dl_drv = clock.now() + Duration::from_millis(25);
            if let Some(m) = inner.macro_run.as_mut() {
                m.deadline = Some(dl_owner);
                m.timeout_code = armd::safety::BoundedText::new("PLACE_TIMEOUT");
            }
            (id, dl_drv)
        };

        bus.clear_log();
        bus.with(|s| s.push_all_feedback());
        let tx_hook_fired = Arc::new(AtomicBool::new(false));
        if !mid_batch {
            // Advance driver `MockClock` by +30ms (> 25ms deadline, < 45ms frame_max_age) strictly INSIDE `mit.rs:823` lock!
            *bump_lock.lock().unwrap() = Some((parent_dl + Duration::from_millis(5), Duration::from_millis(30)));
        } else {
            // Frame 0 hits `can.transmit` BEFORE deadline; `on_tx` advances `MockClock` by +30ms
            // (> 25ms deadline, < 45ms frame_max_age) -> `DamiaoArmDriver::send()` MUST refuse frames 1..6 BEFORE wire TX!
            let clk = clock.clone();
            let fired = Arc::clone(&tx_hook_fired);
            bus.with(|s| {
                s.on_tx = Some(Arc::new(move || {
                    if !fired.swap(true, Ordering::SeqCst) {
                        clk.advance(Duration::from_millis(30));
                    }
                }));
            });
        }

        // Run production Legacy `sup.tick(DT)` (uses real `Instant::now()` + `TerminalBroadcaster`!).
        sup.tick(DT).await;
        bus.with(|s| s.on_tx = None);

        let (tx_frames, tx_times) = bus.with(|s| (s.tx_log.clone(), s.tx_times.clone()));
        for (idx, t_opt) in tx_times.iter().enumerate() {
            let t = t_opt.expect("FakeBus timestamp");
            assert!(
                t < parent_dl,
                "Legacy (mid_batch={mid_batch}) transmitted frame #{idx} AT/AFTER parent_dl! t={t:?} dl={parent_dl:?}"
            );
        }
        if !mid_batch {
            assert_eq!(bumped_count.load(Ordering::SeqCst), 1);
            assert!(tx_frames.is_empty(), "Legacy must put 0 frames on wire when deadline falls in driver lock");
        } else {
            assert!(tx_hook_fired.load(Ordering::SeqCst));
            assert_eq!(tx_frames.len(), 1, "Legacy mid-batch must refuse frames 1..6 inside DamiaoArmDriver::send()");
        }

        // Verify next ticks hold cleanly and deliver NO duplicate terminal!
        let q_hold = sup.inner().read().await.commanded_positions.clone();
        assert!(
            sup.inner().read().await.safety.state().is_operational(),
            "Legacy (mid_batch={mid_batch}) must stay operational, got {:?}",
            sup.inner().read().await.safety.state()
        );
        assert!(sup.inner().read().await.active_trajectory.is_none());
        assert!(sup.inner().read().await.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        for _ in 0..2 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            sup.tick(DT).await;
            let inner = sup.inner().read().await;
            assert!((inner.commanded_positions[0] - q_hold[0]).abs() < 1e-6);
            assert!(inner.commanded_velocities.iter().all(|v| v.abs() < 1e-6));
        }

        // Wait for dispatcher to flush terminal and assert EXACTLY ONE `PLACE_TIMEOUT` terminal!
        let wait_dl = Instant::now() + Duration::from_secs(3);
        while Instant::now() < wait_dl {
            if delivered.lock().unwrap().iter().any(|o| o.action.as_ref().is_some_and(|a| a.id == id)) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        tokio::time::sleep(Duration::from_millis(30)).await;
        let matching: Vec<_> = delivered
            .lock()
            .unwrap()
            .iter()
            .filter(|o| o.action.as_ref().is_some_and(|a| a.id == id))
            .cloned()
            .collect();
        assert_eq!(matching.len(), 1, "Legacy TerminalBroadcaster must deliver PLACE_TIMEOUT exactly once");
        assert_eq!(
            matching[0].action.as_ref().map(|a| (a.state.as_str(), a.code.as_deref())),
            Some(("failed", Some("PLACE_TIMEOUT")))
        );
    }
}
