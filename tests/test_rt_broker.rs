//! Broker contracts against the supervisor's production adapter, stepped one release at a
//! time (`armd::rt_core::ops::ReleaseFixture`, mock follower, tiny capacities): exact
//! replies, retirement counts, and the active motion left unchanged by refused admissions
//! and refused commits.

use std::time::{Duration, Instant};

use armd::rt_core::broker::{
    AdmitError, Authority, BootId, Broker, Capacities, ClientId, ImmediateOp, Outcome, Reply, RequestId, Submit,
};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, Submitted, TargetAuthority, TargetFrame, TargetWriter, MAX_DOF};
use armd::rt_core::ops::{Checkpoint, CoreOps, PlanSlot, Primitive, ReleaseFixture, StubCore};
use armd::rt_core::urgent::StopRequests;
use armd::{ArmConfig, SupervisorInner};

const BOOT: BootId = BootId(42);

fn supervisor() -> SupervisorInner {
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.lease_enabled = false;
    inner.tick(0.02);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
    inner
}

type Fx = (ReleaseFixture<SupervisorInner>, Broker<PlanSlot>, TargetWriter, StopRequests);

fn fixture(caps: Capacities) -> Fx { ReleaseFixture::new(supervisor(), BOOT, caps) }

fn later() -> Instant { Instant::now() + Duration::from_secs(10) }

fn auth(fx: &ReleaseFixture<SupervisorInner>) -> Authority { fx.core.authority(&fx.ctx(fx.now())) }

use armd::rt_core::requests::motion::{self as req_motion, MotionPlanSnapshot, MotionRequest, PlanAuthority, PreparedMotionSlot};

enum InstallArg {
    Prim(Primitive),
    Jog(PreparedMotionSlot),
}
impl From<Primitive> for InstallArg { fn from(p: Primitive) -> Self { Self::Prim(p) } }
impl From<PreparedMotionSlot> for InstallArg { fn from(s: PreparedMotionSlot) -> Self { Self::Jog(s) } }

fn prepare_jog_from(fx: &ReleaseFixture<SupervisorInner>, start: &[f32], q: &[f32], duration_s: f64) -> PreparedMotionSlot {
    let mut snap = MotionPlanSnapshot::from_supervisor(&fx.core);
    snap.commanded_positions = start.to_vec();
    let a = fx.core.authority(&fx.ctx(fx.now()));
    snap.authority = PlanAuthority {
        boot: fx.boot(),
        epoch: a.epoch,
        lease_generation: a.lease_generation,
        stop_generation: a.stop_generation,
        deadline: fx.now().max(Instant::now()) + Duration::from_secs(600),
    };
    req_motion::prepare_motion(&snap, &MotionRequest::jog(q.to_vec(), Some(duration_s), None))
        .expect("prepare_motion")
        .slot
}

fn quintic(fx: &ReleaseFixture<SupervisorInner>, d0: f32, duration_s: f64) -> PreparedMotionSlot {
    let start = fx.core.commanded_positions.clone();
    let mut q = start.clone();
    q[0] += d0;
    prepare_jog_from(fx, &start, &q, duration_s)
}

fn submit_install(b: &mut Broker<PlanSlot>, seq: u64, a: Authority, deadline: Instant, arg: impl Into<InstallArg>) -> Result<RequestId, AdmitError> {
    match arg.into() {
        InstallArg::Prim(p) => b.submit(ClientId(1), seq, a, deadline, Submit::Install(&move |slot: &mut PlanSlot| slot.set(p))),
        InstallArg::Jog(j) => b.submit(ClientId(1), seq, a, deadline, Submit::Install(&move |slot: &mut PlanSlot| req_motion::stage_jog(slot, &j))),
    }
}

fn replies(b: &mut Broker<PlanSlot>) -> Vec<Reply> {
    let mut v = Vec::new();
    b.poll(|_, r| { v.push(r); true });
    v
}

/// The running action and whether a trajectory is active.
fn target_of(fx: &ReleaseFixture<SupervisorInner>) -> (Option<u64>, bool) {
    (fx.core.last_action.as_ref().filter(|a| a.is_running()).map(|a| a.id), fx.core.active_trajectory.is_some())
}

#[test]
fn an_install_is_acked_then_completes_and_retires_its_slot() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 2, plans: 1, replies: 2 });
    let p = quintic(&fx, 0.05, 0.2);
    let r = submit_install(&mut b, 1, auth(&fx), later(), p).unwrap();
    let rep = fx.step();
    let (req, Ok(commit_seq)) = rep.command.unwrap() else { panic!("{rep:?}") };
    assert_eq!((req, commit_seq), (r, 1));
    let action = fx.active().expect("active action");
    assert!(fx.core.active_trajectory.is_some());
    let mut ended = None;
    for _ in 0..100 {
        if let Some(e) = fx.step().ended { ended = Some(e); break; }
    }
    assert_eq!(ended, Some((action, Outcome::Completed)));
    assert_eq!(replies(&mut b), vec![
        Reply::Ack { request: r, commit_seq: 1, action: Some(action) },
        Reply::Terminal { request: r, action, outcome: Outcome::Completed, children: 0 },
    ]);
    let s = b.stats();
    assert_eq!((s.admitted, s.acked, s.terminals, s.retired, b.reserved()), (1, 1, 1, 1, 0));
    assert_eq!(fx.reply_overflow(), 0);
}

#[test]
fn saturated_admission_and_a_stalled_consumer_leave_the_active_motion_unchanged() {
    let ((mut fx, mut b, _tw, _stop), _) = leased_with(600_000, Capacities { commands: 2, plans: 1, replies: 4 });
    let a = auth(&fx);
    submit_install(&mut b, 1, a, later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    let active = fx.active().unwrap();
    let before = (target_of(&fx), fx.core.commanded_positions.clone());
    // the only plan slot is held by the active action
    assert_eq!(submit_install(&mut b, 2, a, later(), quintic(&fx, -0.2, 2.0)), Err(AdmitError::PlansFull));
    // stalled consumer: no release runs; immediate commands fill the ring, then reply capacity
    b.submit(ClientId(2), 1, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
    b.submit(ClientId(2), 2, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
    assert_eq!(b.submit(ClientId(2), 3, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)), Err(AdmitError::CommandsFull));
    assert_eq!(b.stats().refused_admission, 2);
    assert_eq!((target_of(&fx), fx.core.commanded_positions.clone()), before, "refused admission touched the core");
    assert_eq!(fx.active(), Some(active));
    // the consumer resumes: one command per release, the motion keeps running
    let r1 = fx.step();
    let r2 = fx.step();
    assert!(matches!(r1.command, Some((RequestId(2), Ok(2)))), "{r1:?}");
    assert!(matches!(r2.command, Some((RequestId(3), Ok(3)))), "{r2:?}");
    assert_eq!(fx.active(), Some(active));
}

#[test]
fn an_authorized_stop_invalidates_queued_and_active_work_regardless_of_capacity() {
    let (mut fx, mut b, _tw, stop) = fixture(Capacities { commands: 2, plans: 2, replies: 4 });
    let a = auth(&fx);
    let r1 = submit_install(&mut b, 1, a, later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    let action = fx.active().unwrap();
    let mut got = replies(&mut b);
    let r2 = submit_install(&mut b, 2, a, later(), quintic(&fx, -0.2, 2.0)).unwrap();
    b.submit(ClientId(1), 3, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
    // the ring is full; the stop does not go through it
    assert_eq!(b.submit(ClientId(1), 4, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)), Err(AdmitError::CommandsFull));
    stop.request();
    let rep = fx.step();
    assert!(rep.stopped);
    assert_eq!(rep.command, Some((r2, Err("STOPPED"))));
    assert_eq!(rep.ended, Some((action, Outcome::Stopped)));
    assert!(fx.core.active_trajectory.is_none(), "stop ended the motion");
    assert_eq!(fx.step().command.map(|c| c.1), Some(Err("STOPPED")));
    got.extend(replies(&mut b));
    assert_eq!(got[0], Reply::Ack { request: r1, commit_seq: 1, action: Some(action) });
    assert!(got.contains(&Reply::Refused { request: r2, code: "STOPPED" }));
    assert!(got.contains(&Reply::Terminal { request: r1, action, outcome: Outcome::Stopped, children: 0 }));
    assert_eq!((b.reserved(), b.outstanding(), b.stats().retired), (0, 0, 2));
}

#[test]
fn stale_authority_and_expired_deadlines_are_refused_at_commit() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    submit_install(&mut b, 1, a, later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    let active = fx.active().unwrap();
    let before = fx.core.last_action.as_ref().map(|x| x.id);
    // lease generation changed (expiry / takeover) before commit
    submit_install(&mut b, 2, Authority { lease_generation: a.lease_generation + 1, ..a }, later(), quintic(&fx, -0.1, 1.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("LEASE_LOST"));
    // epoch moved
    submit_install(&mut b, 3, Authority { epoch: a.epoch + 1, ..a }, later(), quintic(&fx, -0.1, 1.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("EPOCH_MISMATCH"));
    // the configuration changed since the plan
    submit_install(&mut b, 4, Authority { config_revision: a.config_revision + 1, ..a }, later(), quintic(&fx, -0.1, 1.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    // the original deadline passed while queued
    submit_install(&mut b, 5, a, fx.now(), quintic(&fx, -0.1, 1.0)).unwrap();
    fx.clock().advance(Duration::from_millis(1));
    assert_eq!(fx.step().command.unwrap().1, Err("DEADLINE_EXPIRED"));
    assert_eq!(fx.active(), Some(active));
    assert_eq!(fx.core.last_action.as_ref().map(|x| x.id), before, "a refused commit replaced the action");
    replies(&mut b);
    assert_eq!(b.stats().refused_by_rt, 4);
    assert_eq!(b.stats().retired, 4, "refused installs retire their slots");
}

#[test]
fn plan_slots_are_reused_across_many_actions() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 1, plans: 1, replies: 2 });
    for k in 0..20u64 {
        let d = if k % 2 == 0 { 0.01 } else { -0.01 };
        let r = submit_install(&mut b, k, auth(&fx), later(), quintic(&fx, d, 0.06)).unwrap();
        let rep = fx.step();
        assert_eq!(rep.command.map(|c| c.0), Some(r));
        assert!(rep.command.unwrap().1.is_ok(), "{rep:?}");
        let mut n = 0;
        while fx.active().is_some() && n < 50 { fx.step(); n += 1; }
        assert!(fx.active().is_none());
        replies(&mut b);
    }
    let s = b.stats();
    assert_eq!((s.admitted, s.acked, s.terminals, s.retired, b.reserved()), (20, 20, 20, 20, 0));
}

#[test]
fn targets_are_rechecked_at_consumption() {
    let (mut fx, _b, mut tw, _stop) = fixture(Capacities::default());
    let a = auth(&fx);
    let q = fx.core.commanded_positions.clone();
    let now = fx.now();
    let frame = |seq: u64, authority: Authority, dq: f32| {
        let mut arr = [0.0; MAX_DOF];
        arr[..q.len()].copy_from_slice(&q);
        arr[0] += dq;
        TargetFrame { arm: ArmId::LOCAL, boot: BOOT, stream: StreamId(1), seq, authority, policy_generation: 0, input_at: now, vision_at: None, vision_frame_id: None, deadline: later(), dof: q.len() as u8, q: arr, replaceable: true }
    };
    // no stream authorized yet: refused
    assert_eq!(tw.submit(frame(1, a, 0.001)), Submitted::Accepted);
    assert_eq!(fx.step().target, Some((1, Err("STREAM_NOT_AUTHORIZED"))));
    fx.core.core.authorize_stream(TargetAuthority { stream: Some(StreamId(1)), max_input_age: Some(Duration::from_millis(100)), ..TargetAuthority::default() });
    let (f1, f2) = (TargetFrame { seq: 10, ..frame(1, a, 0.001) }, TargetFrame { seq: 11, ..frame(2, a, 0.002) });
    assert_eq!(tw.submit(f1), Submitted::Accepted);
    assert_eq!(tw.submit(f2), Submitted::Coalesced { replaced: 10 });
    let rep = fx.step();
    assert_eq!(rep.target, Some((11, Ok(()))));
    // a frame whose input aged past the bound while buffered
    tw.submit(TargetFrame { seq: 12, ..frame(2, a, 0.002) });
    fx.clock().advance(Duration::from_millis(200));
    assert_eq!(fx.step().target, Some((12, Err("INPUT_STALE"))));
    fx.core.core.authorize_stream(TargetAuthority { stream: Some(StreamId(1)), ..TargetAuthority::default() });
    // the stream was replaced: frames of the old one are refused
    tw.submit(TargetFrame { stream: StreamId(7), ..frame(1, a, 0.001) });
    assert_eq!(fx.step().target, Some((1, Err("STREAM_NOT_AUTHORIZED"))));
    let frame = |seq: u64, authority: Authority, dq: f32| TargetFrame { seq: seq + 100, ..frame(seq, authority, dq) };
    assert_eq!(tw.submit(frame(1, a, 0.001)), Submitted::Accepted);
    assert_eq!(tw.submit(frame(2, a, 0.002)), Submitted::Coalesced { replaced: 101 });
    let rep = fx.step();
    assert_eq!(rep.target, Some((102, Ok(()))));
    assert!(fx.step().target.is_none(), "consumed once");
    tw.submit(frame(3, Authority { epoch: a.epoch + 1, ..a }, 0.003));
    assert_eq!(fx.step().target, Some((103, Err("EPOCH_MISMATCH"))));
}

#[test]
fn a_stub_core_never_acknowledges_success() {
    let (mut fx, mut b, mut tw, _stop) = ReleaseFixture::new(StubCore::default(), BOOT, Capacities { commands: 4, plans: 2, replies: 8 });
    let a = Authority::default();
    let r1 = b.submit(ClientId(1), 1, a, later(), Submit::Install(&|p: &mut PlanSlot| p.set(Primitive::joint_quintic(&[0.0; 7], &[0.1; 7], 1.0).unwrap()))).unwrap();
    let r2 = b.submit(ClientId(1), 2, a, later(), Submit::Immediate(ImmediateOp::TorqueOn)).unwrap();
    tw.submit(TargetFrame { arm: ArmId::LOCAL, boot: BOOT, stream: StreamId(9), seq: 1, authority: a, policy_generation: 0, input_at: Instant::now(), vision_at: None, vision_frame_id: None, deadline: later(), dof: 1, q: [0.0; MAX_DOF], replaceable: true });
    for _ in 0..3 { fx.step(); }
    assert_eq!(replies(&mut b), vec![Reply::Refused { request: r1, code: "NOT_WIRED" }, Reply::Refused { request: r2, code: "NOT_WIRED" }]);
    assert_eq!((fx.core.commits_seen, fx.core.targets_seen, fx.core.releases), (2, 1, 3));
    assert_eq!(b.stats().acked, 0);
}

#[test]
fn a_planning_snapshot_is_stamped_with_the_publication_and_authority() {
    let (mut fx, _b, _tw, stop) = fixture(Capacities::default());
    fx.step();
    stop.request();
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    assert_eq!(snap.published_seq, fx.core.core.published_seq());
    assert_eq!(snap.authority.stop_generation, 1);
    assert_eq!(snap.measured.len(), fx.core.config.dof);
    assert!(!snap.motion_active);
    assert!(fx.core.poll_hardware(&armd::rt_core::ops::ServiceBudget::for_release(fx.now(), fx.now())).driverless);
}

/// A supervisor that enforces leases, with `client` holding one (`lease_timeout_ms`).
fn leased(lease_timeout_ms: u64) -> (Fx, u64) { leased_with(lease_timeout_ms, Capacities { commands: 4, plans: 2, replies: 16 }) }

fn leased_with(lease_timeout_ms: u64, caps: Capacities) -> (Fx, u64) {
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms, ..ArmConfig::rebot_dm() });
    inner.lease_enabled = true;
    inner.tick(0.02);
    let generation = inner.acquire_lease("client").expect("lease");
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
    (ReleaseFixture::new(inner, BOOT, caps), generation)
}

#[test]
fn a_queued_command_is_refused_once_its_lease_expired_before_the_watchdog_ran() {
    let ((mut fx, mut b, _tw, _stop), generation) = leased(1_000);
    let a = auth(&fx);
    assert_eq!(a.lease_generation, generation);
    submit_install(&mut b, 1, a, later(), quintic(&fx, 0.05, 0.5)).unwrap();
    // the lease expires while the command waits; the stored lease is still in place
    fx.core.active_lease.as_mut().unwrap().acquired_at -= Duration::from_millis(1_001);
    let before = target_of(&fx);
    assert_eq!(fx.step().command.unwrap().1, Err("LEASE_LOST"));
    assert_eq!(target_of(&fx), before, "a refused commit touched the motion");
    assert_eq!(fx.core.active_lease.as_ref().map(|l| l.generation), Some(generation), "the commit did not clean up");
    // without a lease generation, nothing commits while leases are enforced
    submit_install(&mut b, 2, armd::rt_core::broker::Authority { lease_generation: 0, ..a }, later(), quintic(&fx, 0.05, 0.5)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("NO_LEASE"));
}

#[test]
fn a_heartbeat_renews_its_valid_lease_and_cannot_resurrect_an_expired_one() {
    let ((mut fx, mut b, _tw, _stop), generation) = leased(1_000);
    let a = auth(&fx);
    // nearly expired: the heartbeat renews it at the commit instant
    fx.core.active_lease.as_mut().unwrap().acquired_at -= Duration::from_millis(900);
    b.submit(ClientId(1), 1, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let held = fx.core.active_lease.as_ref().unwrap().acquired_at.elapsed();
    assert!(held < Duration::from_millis(100), "not renewed: {held:?}");
    // expired while the heartbeat was queued: refused, the lease stays expired
    b.submit(ClientId(1), 2, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
    fx.core.active_lease.as_mut().unwrap().acquired_at -= Duration::from_millis(1_001);
    assert_eq!(fx.step().command.unwrap().1, Err("LEASE_LOST"));
    assert!(!fx.core.is_lease_active(), "a late heartbeat resurrected the lease");
    assert_eq!(fx.core.active_lease.as_ref().map(|l| l.generation), Some(generation));
}

#[test]
fn a_child_commits_only_while_its_parent_is_the_running_action() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    assert_eq!(a.parent_generation, 0, "no running action");
    submit_install(&mut b, 1, a, later(), quintic(&fx, 0.1, 2.0)).unwrap();
    fx.step();
    let parent = fx.active().unwrap();
    let a = auth(&fx);
    assert_eq!(a.parent_generation, parent.0);
    // a child of an action that is no longer running
    let stale = armd::rt_core::broker::Authority { parent_generation: parent.0 + 100, ..a };
    let start = fx.core.commanded_positions.clone();
    let mut q = start.clone();
    q[0] -= 0.05;
    submit_install(&mut b, 2, stale, later(), prepare_jog_from(&fx, &start, &q, 1.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PARENT_REPLACED"));
    assert_eq!(fx.active(), Some(parent));
    // a child of the running parent, prepared from the current commanded position
    let start = fx.core.commanded_positions.clone();
    let mut q = start.clone();
    q[0] -= 0.05;
    submit_install(&mut b, 3, a, later(), prepare_jog_from(&fx, &start, &q, 1.0)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
}

#[test]
fn a_primitive_prepared_from_another_start_is_refused_including_the_gripper() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    let start = fx.core.commanded_positions.clone();
    let gripper = fx.core.config.gripper_index().expect("rebot_dm has a gripper");
    let mut q = start.clone();
    q[0] += 0.05;
    // the gripper moved by 2e-4 since the plan: stale
    let mut moved = start.clone();
    moved[gripper] += 2e-4;
    submit_install(&mut b, 1, a, later(), prepare_jog_from(&fx, &moved, &q, 0.5)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    assert!(fx.active().is_none() && fx.core.active_trajectory.is_none());
    // below the 1e-4 threshold it is the same start
    let mut close = start.clone();
    close[gripper] += 5e-5;
    submit_install(&mut b, 2, a, later(), prepare_jog_from(&fx, &close, &q, 0.5)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
}

#[test]
fn a_hand_built_oversized_primitive_is_refused_without_a_panic() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 2, plans: 1, replies: 4 });
    let a = auth(&fx);
    let bad = Primitive::JointQuintic { dof: MAX_DOF as u8 + 1, start: [0.0; MAX_DOF], q: [0.0; MAX_DOF], duration_s: 1.0 };
    submit_install(&mut b, 1, a, later(), bad).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("INVALID_COMMAND"));
    assert!(fx.active().is_none());
}

#[test]
fn a_terminal_keeps_the_cause_of_a_lease_loss() {
    let ((mut fx, mut b, _tw, _stop), generation) = leased(600_000);
    submit_install(&mut b, 1, auth(&fx), later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    let action = fx.active().unwrap();
    assert!(fx.core.release_lease("client", generation));
    let mut ended = None;
    for _ in 0..10 { if let Some(e) = fx.step().ended { ended = Some(e); break; } }
    assert_eq!(ended, Some((action, Outcome::Cancelled("LEASE_LOST"))));
}

#[test]
fn the_fixture_polls_the_hardware_each_release_within_a_virtual_budget() {
    let (mut fx, _b, _tw, _stop) = ReleaseFixture::new(StubCore::default(), BOOT, Capacities { commands: 4, plans: 2, replies: 8 });
    let base = fx.clock().base();
    // wall time passing does not move the fixture's clock
    std::thread::sleep(Duration::from_millis(15));
    for _ in 0..5 { fx.step(); }
    assert_eq!(fx.core.polls, 5, "one poll per release");
    let budget = fx.core.last_budget.unwrap();
    assert_eq!(budget.now, base + Duration::from_millis(80));
    assert_eq!(budget.deadline, base + Duration::from_millis(85), "5 ms service budget inside the 20 ms release");
    assert_eq!(budget.max_rx_frames, 64);
    assert_eq!(fx.now(), base + Duration::from_millis(100));
}

#[test]
fn a_deadline_that_passes_between_releases_is_refused_in_virtual_time() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    let deadline = fx.now() + Duration::from_millis(10);
    submit_install(&mut b, 1, a, deadline, quintic(&fx, 0.05, 0.5)).unwrap();
    submit_install(&mut b, 2, a, deadline, quintic(&fx, 0.05, 0.5)).unwrap();
    // wall time passing does not expire it; release 2 starts 20 ms later in virtual time
    std::thread::sleep(Duration::from_millis(15));
    assert!(fx.step().command.unwrap().1.is_ok());
    assert_eq!(fx.step().command.unwrap().1, Err("DEADLINE_EXPIRED"));
}

#[test]
fn a_stop_arriving_mid_release_precedes_the_commit_the_target_and_the_output() {
    // before the commit: the queued command admitted under the old generation is refused
    let (mut fx, mut b, _tw, stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    submit_install(&mut b, 1, a, later(), quintic(&fx, 0.1, 1.0)).unwrap();
    let s = stop.clone();
    fx.on_checkpoint(move |c| if c == Checkpoint::BeforeCommit { s.request(); });
    let rep = fx.step();
    assert!(rep.stopped);
    assert_eq!(rep.command.unwrap().1, Err("STOPPED"));
    assert!(fx.active().is_none() && fx.core.active_trajectory.is_none());
    assert!(!stop.pending(), "applied in the same release");

    // before the target: the buffered target admitted under the old generation is refused
    let (mut fx, _b, mut tw, stop) = fixture(Capacities::default());
    let a = auth(&fx);
    let q = fx.core.commanded_positions.clone();
    let mut arr = [0.0; MAX_DOF];
    arr[..q.len()].copy_from_slice(&q);
    fx.core.core.authorize_stream(TargetAuthority { stream: Some(StreamId(1)), ..TargetAuthority::default() });
    tw.submit(TargetFrame { arm: ArmId::LOCAL, boot: BOOT, stream: StreamId(1), seq: 1, authority: a, policy_generation: 0, input_at: fx.now(), vision_at: None, vision_frame_id: None, deadline: later(), dof: q.len() as u8, q: arr, replaceable: true });
    let s = stop.clone();
    fx.on_checkpoint(move |c| if c == Checkpoint::BeforeTarget { s.request(); });
    let rep = fx.step();
    assert!(rep.stopped);
    assert_eq!(rep.target, Some((1, Err("STOPPED"))));

    // before the output: the running motion is stopped before the tick moves it again
    let (mut fx, mut b, _tw, stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    submit_install(&mut b, 1, auth(&fx), later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    fx.step();
    let action = fx.active().unwrap();
    let s = stop.clone();
    fx.on_checkpoint(move |c| if c == Checkpoint::BeforeOutput { s.request(); });
    let rep = fx.step();
    assert!(rep.stopped);
    assert_eq!(rep.ended, Some((action, Outcome::Stopped)), "stopped before this release's output");
    assert!(fx.core.active_trajectory.is_none());
}

#[test]
fn a_deadline_expiring_after_the_release_started_is_refused_at_the_commit() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    let deadline = fx.now() + Duration::from_millis(1);
    submit_install(&mut b, 1, a, deadline, quintic(&fx, 0.05, 0.5)).unwrap();
    let clock = fx.clock().clone();
    // the release starts before the deadline; time passes it before the commit
    fx.on_checkpoint(move |c| if c == Checkpoint::BeforeCommit { clock.advance(Duration::from_millis(2)); });
    let before = target_of(&fx);
    assert_eq!(fx.step().command.unwrap().1, Err("DEADLINE_EXPIRED"));
    assert_eq!(target_of(&fx), before);
}

#[test]
fn a_config_reservation_needs_current_authority_and_inhibits_motion_until_it_ends() {
    use armd::rt_core::broker::{RequestId, Stamp};
    use armd::rt_core::config_iface::{Activated, CandidateId, ConfigAuthority, ConfigDomain, ConfigProposal, Released, Reserved};
    let (mut fx, mut b, _tw, stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    let revision = fx.core.revision(ConfigDomain::Keepout);
    let stamp = Stamp { arm: ArmId::LOCAL, boot: BOOT, request: RequestId(900), client: ClientId(5), client_seq: 1, authority: a, deadline: later() };
    let p = ConfigProposal { domain: ConfigDomain::Keepout, expected_revision: revision, candidate: CandidateId(1), stamp };
    // a stop since the proposal was prepared: the stale authority cannot inhibit motion
    stop.request();
    fx.step();
    let ctx = fx.ctx(fx.now());
    assert!(matches!(fx.core.reserve(&p, &ctx), Reserved::Refused { code: "STOPPED", .. }));
    assert!(!fx.core.core.motion_inhibited_by_config());
    // prepared under the current authority: granted, and motion is inhibited
    let p = ConfigProposal { stamp: Stamp { authority: auth(&fx), ..stamp }, ..p };
    let Reserved::Granted(r) = fx.core.reserve(&p, &ctx) else { panic!("not granted") };
    assert_eq!(r.stamp().request, RequestId(900));
    assert!(fx.core.planning_snapshot(&ctx).config_inhibited);
    submit_install(&mut b, 1, auth(&fx), later(), quintic(&fx, 0.05, 0.5)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("CONFIG_BUSY"));
    // the legacy admission paths refuse it too: a jog (quintic) and a Cartesian plan input
    let code = |e: armd::ArmError| armd::api::rest::ApiError::from(e).code;
    let mut q = fx.core.commanded_positions.clone();
    q[0] += 0.05;
    assert_eq!({ let __s = fx.core.current_admission_stamp(); fx.core.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::jog_action(__a, q.clone(), Some(0.5), None, "jog")) }.map(drop).map_err(code), Err(Some("CONFIG_BUSY".into())));
    assert_eq!(fx.core.plan_input_tcp().map(drop).map_err(code), Err(Some("CONFIG_BUSY".into())));
    assert!(fx.core.active_trajectory.is_none());
    // nothing was applied for this candidate: activation hands the reservation back, released
    let Activated::Refused { code: "NOT_APPLIED", reservation } = fx.core.activate(r) else { panic!() };
    assert_eq!(fx.core.release(reservation), Released::Cleared);
    assert!(!fx.core.core.motion_inhibited_by_config());
    assert!(fx.core.plan_input_tcp().is_ok());
    submit_install(&mut b, 2, auth(&fx), later(), quintic(&fx, 0.05, 0.5)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
}

#[test]
fn a_torque_transition_is_acked_then_its_terminal_follows_from_the_poll() {
    use armd::rt_core::broker::TransitionOp;
    let ((mut fx, mut b, _tw, _stop), _) = leased(600_000);
    let r = b.submit(ClientId(1), 1, auth(&fx), later(), Submit::Transition(TransitionOp::TorqueOff)).unwrap();
    assert_eq!(b.reserved(), 2, "ACK and terminal reserved");
    let rep = fx.step();
    assert!(rep.command.unwrap().1.is_ok());
    let t = fx.transition().expect("in flight");
    assert!(!fx.core.is_torqued);
    let rep = fx.step();
    assert_eq!(rep.transition_ended, Some((t, Outcome::Completed)));
    assert!(fx.transition().is_none());
    let got = replies(&mut b);
    assert_eq!(got[1], Reply::Terminal { request: r, action: t, outcome: Outcome::Completed, children: 0 });
    assert_eq!((b.reserved(), b.outstanding()), (0, 0));
}

#[test]
fn macro_children_name_their_parent_and_a_macro_terminal_carries_its_child_count() {
    use armd::rt_core::ops::{ChildSpec, ChildTicket};
    use armd::rt_core::MacroTerminal;
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    submit_install(&mut b, 1, auth(&fx), later(), quintic(&fx, 0.1, 2.0)).unwrap();
    fx.step();
    let parent = fx.active().unwrap();
    let a = auth(&fx);
    let start = { let mut s = [0.0; MAX_DOF]; let c = &fx.core.commanded_positions; s[..c.len()].copy_from_slice(c); s };
    let child = |p: u64| ChildSpec { ticket: ChildTicket { parent: armd::rt_core::broker::ActionId(p), child_seq: 1 }, timeout_s: 2.0, timeout_code: "GRIPPER_TIMEOUT", contact: None };
    let grip = |p: u64| Primitive::Gripper { child: Some(child(p)), dof: 7, start, fraction: 0.0, speed: 1.0, grasp: true };
    // a ticket for another parent than the stamp's
    submit_install(&mut b, 2, a, later(), grip(parent.0 + 1)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PARENT_REPLACED"));
    // well-formed, but the parent is a plain joint move, not a macro
    submit_install(&mut b, 3, a, later(), grip(parent.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PARENT_REPLACED"));
    assert_eq!(fx.active(), Some(parent));
    // the macro's terminal record (outcome and children) outlives its action record
    fx.core.core.record_macro_terminal(MacroTerminal { action: parent, children: 3, outcome: Outcome::Failed("NOT_HOLDING") });
    let rep = fx.step();
    assert_eq!(rep.ended, Some((parent, Outcome::Failed("NOT_HOLDING"))));
    let got = replies(&mut b);
    assert!(got.iter().any(|r| matches!(r, Reply::Terminal { action, children: 3, outcome: Outcome::Failed("NOT_HOLDING"), .. } if *action == parent)), "{got:?}");
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    assert!(snap.feedback_queue_bounds.is_empty(), "mock follower: no queue bounds");
}

#[test]
fn a_buffered_frame_is_refused_once_its_lease_was_released_and_the_stream_hears_why() {
    use armd::rt_core::mailbox::{Refusal, RefusalDetail, TargetOutcome};
    let ((mut fx, _b, mut tw, _stop), generation) = leased(600_000);
    fx.core.core.authorize_stream(TargetAuthority { stream: Some(StreamId(1)), ..TargetAuthority::default() });
    let a = auth(&fx);
    let q = fx.core.commanded_positions.clone();
    let mut arr = [0.0; MAX_DOF];
    arr[..q.len()].copy_from_slice(&q);
    let frame = TargetFrame { arm: ArmId::LOCAL, boot: BOOT, stream: StreamId(1), seq: 1, authority: a, policy_generation: 0, input_at: fx.now(),
        vision_at: None, vision_frame_id: Some(77), deadline: later(), dof: q.len() as u8, q: arr, replaceable: true };
    assert_eq!(tw.submit(frame), Submitted::Accepted);
    // the WS closes: its lease is released while the frame is buffered
    fx.core.release_lease("client", generation);
    let rep = fx.step();
    assert_eq!(rep.target, Some((1, Err("LEASE_LOST"))));
    assert_eq!(rep.target_refusal, Some(Refusal { code: "LEASE_LOST", detail: RefusalDetail::LeaseLoss { reason: "expired" } }));
    let mut got = Vec::new();
    tw.outcomes(|o| got.push(o));
    assert_eq!(got, vec![TargetOutcome { stream: StreamId(1), seq: 1, result: Err(rep.target_refusal.unwrap()) }]);
}

// ---------------------------------------------------------------------------------------
// Track C: commit rechecks, stop and leases

use armd::model::{JointLimit, RestingPosePolicy};

const PARK_LEASE_MS: u64 = 1_000;

/// A 2-joint mock arm with a resting pose, torqued, `client-a` holding a lease.
fn parking_arm() -> (Fx, u64) {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0), JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0, 0.0];
    config.lease_timeout_ms = PARK_LEASE_MS;
    config.resting_pose = Some(RestingPosePolicy { positions: None, position_tolerances: vec![0.08; 2], max_total_deviation: 0.16,
        max_velocity: 0.015, joint_max_velocities: vec![], support_torque_limits: vec![(0, 1.0)], support_load_limits: vec![], feedback_max_age_ms: 100 });
    let mut inner = SupervisorInner::new(config);
    inner.tick(0.02);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
    let generation = inner.acquire_lease("client-a").expect("lease");
    (ReleaseFixture::new(inner, BOOT, Capacities { commands: 4, plans: 2, replies: 8 }), generation)
}

/// The holder's heartbeat at the fixture's (virtual) now; the legacy watchdog reads wall time.
fn renew(fx: &mut ReleaseFixture<SupervisorInner>) {
    let now = fx.now().max(Instant::now());
    fx.core.active_lease.as_mut().expect("lease").acquired_at = now;
    fx.core.last_heartbeat = Instant::now();
}

/// The lease expired and the watchdog's heartbeat window passed (no sleeping).
fn expire(fx: &mut ReleaseFixture<SupervisorInner>) {
    let past = Duration::from_millis(PARK_LEASE_MS + 50);
    fx.core.active_lease.as_mut().expect("lease").acquired_at = Instant::now().min(fx.now()) - past;
    fx.core.last_heartbeat = Instant::now() - past;
}

fn to(fx: &ReleaseFixture<SupervisorInner>, q: [f32; 2], duration_s: f64) -> PreparedMotionSlot {
    let start = fx.core.commanded_positions.clone();
    prepare_jog_from(fx, &start, &q, duration_s)
}

/// Runs the active action to its end, renewing the lease.
fn run_out(fx: &mut ReleaseFixture<SupervisorInner>) {
    for _ in 0..300 { renew(fx); if fx.step().ended.is_some() { return; } }
    panic!("the action did not end");
}

#[test]
fn a_lease_expiring_before_the_commit_leaves_the_watchdog_park_as_the_active_trajectory() {
    let ((mut fx, mut b, _tw, _stop), generation) = parking_arm();
    renew(&mut fx);
    let a = auth(&fx);
    assert_eq!(a.lease_generation, generation);
    let r1 = submit_install(&mut b, 1, a, later(), to(&fx, [0.6, 0.3], 0.2)).unwrap();
    run_out(&mut fx);
    assert!(fx.core.active_trajectory.is_none());
    // queued under the lease, committed after it expired: refused, then the watchdog parks
    let r2 = submit_install(&mut b, 2, a, later(), to(&fx, [-0.4, 0.0], 0.5)).unwrap();
    expire(&mut fx);
    let rep = fx.step();
    assert_eq!(rep.command, Some((r2, Err("LEASE_LOST"))));
    assert!(fx.core.pending_normal_unload && fx.core.watchdog_park.is_some(), "watchdog park");
    assert_eq!((fx.core.target_positions.clone(), fx.active()), (vec![0.0, 0.0], None), "the park, not the install");
    let got = replies(&mut b);
    assert!(matches!(got[0], Reply::Ack { request, .. } if request == r1), "{got:?}");
    assert_eq!(got.last(), Some(&Reply::Refused { request: r2, code: "LEASE_LOST" }));
    assert_eq!((b.reserved(), b.outstanding(), b.stats().retired), (0, 0, 2));
}

#[test]
fn a_stop_during_the_watchdog_park_keeps_the_park_until_it_parks() {
    let ((mut fx, mut b, _tw, _stop), _) = parking_arm();
    renew(&mut fx);
    submit_install(&mut b, 1, auth(&fx), later(), to(&fx, [0.6, 0.3], 0.2)).unwrap();
    run_out(&mut fx);
    // a stop of client motion ends it
    renew(&mut fx);
    submit_install(&mut b, 2, auth(&fx), later(), to(&fx, [0.2, 0.3], 1.0)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert!(fx.core.authorized_stop(), "client motion stops");
    assert!(fx.core.active_trajectory.is_none());
    // the lease expires: the watchdog parks; a stop (the expired owner's) keeps the park
    expire(&mut fx);
    fx.step();
    assert!(fx.core.pending_normal_unload && fx.core.watchdog_park.is_some());
    assert!(!fx.core.authorized_stop(), "the park is not the client's motion");
    assert!(fx.core.pending_normal_unload && fx.core.active_trajectory.is_some());
    for _ in 0..300 { fx.step(); if !fx.core.is_torqued { break; } }
    assert_eq!(*fx.core.safety.state(), armd::SafetyState::Idle, "parked, no watchdog fault");
    assert!(!fx.core.is_torqued && fx.core.watchdog_park.is_none());
    assert!(fx.core.commanded_positions.iter().all(|q| q.abs() < 1e-3), "{:?}", fx.core.commanded_positions);
}

#[test]
fn an_urgent_stop_through_the_release_keeps_the_watchdog_park() {
    let ((mut fx, mut b, _tw, stop), _) = parking_arm();
    renew(&mut fx);
    submit_install(&mut b, 1, auth(&fx), later(), to(&fx, [0.6, 0.3], 0.2)).unwrap();
    run_out(&mut fx);
    expire(&mut fx);
    fx.step();
    assert!(fx.core.pending_normal_unload && fx.core.watchdog_park.is_some(), "watchdog park");
    // CoreOps::apply_stop at the start of the release: the park is not the client's motion
    stop.request();
    assert!(fx.step().stopped);
    assert!(fx.core.pending_normal_unload && fx.core.active_trajectory.is_some(), "the park continues");
    for _ in 0..300 { fx.step(); if !fx.core.is_torqued { break; } }
    assert_eq!(*fx.core.safety.state(), armd::SafetyState::Idle, "parked, no watchdog fault");
    assert!(!fx.core.is_torqued && fx.core.watchdog_park.is_none());
}

#[test]
fn saturated_ingress_then_stop_refuses_every_older_command_and_a_fresh_install_commits() {
    let (mut fx, mut b, _tw, stop) = fixture(Capacities { commands: 3, plans: 3, replies: 12 });
    let a = auth(&fx);
    let r1 = submit_install(&mut b, 1, a, later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    let action = fx.active().unwrap();
    let r2 = submit_install(&mut b, 2, a, later(), quintic(&fx, -0.2, 2.0)).unwrap();
    let r3 = submit_install(&mut b, 3, a, later(), quintic(&fx, 0.1, 2.0)).unwrap();
    let r4 = b.submit(ClientId(1), 4, a, later(), Submit::Immediate(ImmediateOp::Heartbeat)).unwrap();
    assert_eq!(submit_install(&mut b, 5, a, later(), quintic(&fx, 0.3, 2.0)), Err(AdmitError::CommandsFull));
    stop.request();
    let rep = fx.step();
    assert!(rep.stopped);
    assert_eq!((rep.command, rep.ended), (Some((r2, Err("STOPPED"))), Some((action, Outcome::Stopped))));
    for r in [r3, r4] {
        assert_eq!(fx.step().command, Some((r, Err("STOPPED"))));
        assert!(fx.core.active_trajectory.is_none(), "nothing older than the stop runs");
    }
    // a command admitted after the stop commits
    let r6 = submit_install(&mut b, 6, auth(&fx), later(), quintic(&fx, 0.05, 0.5)).unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(2))) if r == r6), "{rep:?}");
    assert!(fx.core.active_trajectory.is_some());
    let got = replies(&mut b);
    assert_eq!(got[0], Reply::Ack { request: r1, commit_seq: 1, action: Some(action) });
    assert_eq!(&got[1..], &[
        Reply::Refused { request: r2, code: "STOPPED" },
        Reply::Terminal { request: r1, action, outcome: Outcome::Stopped, children: 0 },
        Reply::Refused { request: r3, code: "STOPPED" },
        Reply::Refused { request: r4, code: "STOPPED" },
        Reply::Ack { request: r6, commit_seq: 2, action: fx.active() },
    ]);
    assert_eq!((b.stats().retired, b.reserved()), (3, 1), "r6's terminal stays reserved");
}

#[test]
fn a_motion_between_prepare_and_install_refuses_the_install_and_keeps_the_running_trajectory() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 8 });
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    let a = snap.authority;
    let r1 = submit_install(&mut b, 1, a, later(), quintic(&fx, 0.2, 1.0)).unwrap();
    fx.step();
    let first = fx.active().unwrap();
    for _ in 0..10 { fx.step(); }
    // prepared from the snapshot, queued now: r1 moved the commanded joints since
    let mut q2 = snap.commanded.clone();
    q2[1] += 0.1;
    let r2 = submit_install(&mut b, 2, a, later(), prepare_jog_from(&fx, &snap.commanded, &q2, 1.0)).unwrap();
    let before = (target_of(&fx), fx.core.target_positions.clone());
    let rep = fx.step();
    assert_eq!(rep.command, Some((r2, Err("PLAN_STALE"))));
    assert_eq!(fx.active(), Some(first));
    assert_eq!((target_of(&fx), fx.core.target_positions.clone()), before, "the running trajectory is left");
    // prepared from the current commanded state, it replaces the running joint move
    let r3 = submit_install(&mut b, 3, auth(&fx), later(), quintic(&fx, -0.05, 1.0)).unwrap();
    assert!(matches!(fx.step().command, Some((r, Ok(2))) if r == r3));
    let got = replies(&mut b);
    assert!(got.contains(&Reply::Refused { request: r2, code: "PLAN_STALE" }));
    assert!(got.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Superseded, .. } if *request == r1)), "{got:?}");
}

#[test]
fn invalid_feedback_refuses_the_commit_and_leaves_the_motion() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let a = auth(&fx);
    submit_install(&mut b, 1, a, later(), quintic(&fx, 0.2, 2.0)).unwrap();
    fx.step();
    let active = fx.active().unwrap();
    let before = target_of(&fx);
    fx.core.feedback_numeric_valid = false;
    submit_install(&mut b, 2, a, later(), quintic(&fx, -0.1, 1.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("FEEDBACK_STALE"));
    assert_eq!((fx.active(), target_of(&fx)), (Some(active), before));
    fx.core.feedback_numeric_valid = true;
    submit_install(&mut b, 3, a, later(), quintic(&fx, -0.1, 1.0)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
}

#[test]
fn a_worker_refuses_a_joint_move_at_admission_while_a_config_hold_inhibits_motion() {
    use armd::rt_core::broker::{RequestId, Stamp};
    use armd::rt_core::config_iface::{CandidateId, ConfigAuthority, ConfigDomain, ConfigProposal, Reserved};
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let ctx = fx.ctx(fx.now());
    let snap = fx.core.planning_snapshot(&ctx);
    let mut q = snap.commanded.clone();
    q[0] += 0.05;
    // free: prepared from the snapshot's commanded joints, admitted and committed
    assert!(armd::motion::prepare_joint_move(&snap, &q, 0.5).is_ok());
    let slot = prepare_jog_from(&fx, &snap.commanded, &q, 0.5);
    submit_install(&mut b, 1, snap.authority, later(), slot).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert_eq!(armd::motion::prepare_joint_move(&snap, &q[..2], 0.5), Err("INVALID_COMMAND"));
    // a configuration worker holds a motion domain: the next move is refused before admission
    let a = auth(&fx);
    let stamp = Stamp { arm: ArmId::LOCAL, boot: BOOT, request: RequestId(900), client: ClientId(5), client_seq: 1, authority: a, deadline: later() };
    let proposal = ConfigProposal { domain: ConfigDomain::Keepout, expected_revision: fx.core.revision(ConfigDomain::Keepout), candidate: CandidateId(1), stamp };
    let ctx = fx.ctx(fx.now());
    let Reserved::Granted(r) = fx.core.reserve(&proposal, &ctx) else { panic!("not granted") };
    let snap = fx.core.planning_snapshot(&ctx);
    assert_eq!(armd::motion::prepare_joint_move(&snap, &snap.commanded.clone(), 0.5), Err("CONFIG_BUSY"));
    assert_eq!(b.stats().admitted, 1, "nothing queued");
    fx.core.release(r);
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    assert!(armd::motion::prepare_joint_move(&snap, &snap.commanded.clone(), 0.5).is_ok());
}

#[test]
fn the_target_port_is_reachable_from_the_supervisor_module() {
    use armd::supervisor::targets::{policy_stream, servo_stream, SERVO_STREAM_BIT};
    assert_ne!(servo_stream(7), policy_stream(7));
    assert_eq!(servo_stream(7).0 & SERVO_STREAM_BIT, SERVO_STREAM_BIT);
    assert_eq!(policy_stream(7).0 & SERVO_STREAM_BIT, 0);
}

#[test]
fn child_primitives_of_a_running_macro_commit_through_the_release() {
    use armd::rt_core::ops::{ChildSpec, ChildTicket};
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 4, plans: 2, replies: 16 });
    let epoch = fx.core.safety.epoch_id();
    let id = { let __s = fx.core.current_admission_stamp(); fx.core.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_orchestrated_action(__a, "sequence", None, epoch)) }.unwrap();
    let parent = armd::rt_core::broker::ActionId(id);
    let a = auth(&fx);
    assert_eq!(a.parent_generation, id);
    let start = |fx: &ReleaseFixture<SupervisorInner>| { let mut s = [0.0; MAX_DOF]; let c = &fx.core.commanded_positions; s[..c.len()].copy_from_slice(c); s };
    let spec = |seq| Some(ChildSpec { ticket: ChildTicket { parent, child_seq: seq }, timeout_s: 10.0, timeout_code: "SEQUENCE_TIMEOUT", contact: None });
    let dof = fx.core.commanded_positions.len() as u8;
    let mut q = start(&fx);
    q[0] += 0.05;
    let joint = Primitive::JointQuinticLimited { child: spec(1), dof, start: start(&fx), q, duration_s: 0.5, max_velocity: [0.0; MAX_DOF], settle_s: 0.0 };
    let r1 = submit_install(&mut b, 1, a, later(), joint).unwrap();
    let rep = fx.step();
    assert!(matches!(rep.command, Some((r, Ok(_))) if r == r1), "{rep:?}");
    assert_eq!((fx.active(), fx.core.macro_children(id)), (Some(parent), Some(1)));
    assert!(fx.core.active_trajectory.is_some());
    // the same ticket again (not the next child) is stale; nothing replaces the running child
    submit_install(&mut b, 2, a, later(), joint).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    let child_done = |fx: &ReleaseFixture<SupervisorInner>| matches!(fx.core.macro_run.as_ref().map(|m| &m.sub), Some(armd::macro_action::SubState::Done(_)));
    for _ in 0..300 { fx.step(); if child_done(&fx) { break; } }
    assert!(child_done(&fx));
    assert!((fx.core.commanded_positions[0] - q[0]).abs() < 1e-3);
    // a gripper child, then the macro ends: one terminal with both children
    let grip = Primitive::Gripper { child: spec(2), dof, start: start(&fx), fraction: 0.5, speed: 1.0, grasp: false };
    submit_install(&mut b, 3, auth(&fx), later(), grip).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    assert_eq!(fx.core.macro_children(id), Some(2));
    for _ in 0..300 { fx.step(); if child_done(&fx) { break; } }
    assert!(child_done(&fx));
    fx.core.finish_macro(id, "done", None, None, None, None);
    assert_eq!(fx.step().ended, Some((parent, Outcome::Completed)));
    let got = replies(&mut b);
    assert!(got.iter().any(|r| matches!(r, Reply::Terminal { action, children: 2, outcome: Outcome::Completed, .. } if *action == parent)), "{got:?}");
    assert_eq!((b.reserved(), b.outstanding()), (0, 0));
}

#[test]
fn default_eight_plan_slots_exhaust_and_recover_and_stale_boot_is_refused() {
    use armd::rt_core::broker::PLAN_SLOTS;

    let (mut fx, mut b, mut tw, _stop) = ReleaseFixture::new_fresh(supervisor(), Capacities::default());
    assert_eq!(b.plan_capacity(), PLAN_SLOTS);
    assert_eq!(b.boot(), fx.boot());
    assert!(!fx.boot().is_zero());

    let a = auth(&fx);
    let p = quintic(&fx, 0.05, 0.5);
    for seq in 1..=(PLAN_SLOTS as u64) {
        submit_install(&mut b, seq, a, later(), p).expect("slot free");
    }
    assert_eq!(submit_install(&mut b, 9, a, later(), p), Err(AdmitError::PlansFull));
    assert_eq!(b.stats().refused_admission, 1);

    // Committing install #1 then superseding it with #2 retires slot #1 so #9 can be admitted.
    assert!(fx.step().command.unwrap().1.is_ok());
    // Install #2 was prepared from the same start as #1; after #1 advanced slightly it may
    // be refused PLAN_STALE, which also retires its slot immediately.
    let _ = fx.step();
    replies(&mut b);
    assert!(b.stats().retired >= 1);
    assert!(submit_install(&mut b, 9, auth(&fx), later(), quintic(&fx, -0.02, 0.2)).is_ok());

    // Target frames carrying a stale boot id or zero boot id are refused with WRONG_TARGET.
    fx.core.core.authorize_stream(TargetAuthority { stream: Some(StreamId(1)), ..TargetAuthority::default() });
    let q = fx.core.commanded_positions.clone();
    let mut arr = [0.0; MAX_DOF];
    arr[..q.len()].copy_from_slice(&q);
    for (seq, bad_boot) in [(1u64, BootId::fresh()), (2u64, BootId(0))] {
        tw.submit(TargetFrame {
            arm: ArmId::LOCAL,
            boot: bad_boot,
            stream: StreamId(1),
            seq,
            authority: auth(&fx),
            policy_generation: 0,
            input_at: fx.now(),
            vision_at: None,
            vision_frame_id: None,
            deadline: later(),
            dof: q.len() as u8,
            q: arr,
            replaceable: true,
        });
        assert_eq!(fx.step().target, Some((seq, Err("WRONG_TARGET"))));
    }
}
