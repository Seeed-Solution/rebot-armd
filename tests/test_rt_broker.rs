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
use armd::rt_core::ops::{Checkpoint, CoreOps, Primitive, ReleaseFixture, StubCore};
use armd::rt_core::urgent::StopRequests;
use armd::{ArmConfig, SupervisorInner};

const BOOT: BootId = BootId(42);

fn supervisor() -> SupervisorInner {
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.lease_enabled = false;
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    inner
}

type Fx = (ReleaseFixture<SupervisorInner>, Broker<Primitive>, TargetWriter, StopRequests);

fn fixture(caps: Capacities) -> Fx { ReleaseFixture::new(supervisor(), BOOT, caps) }

fn later() -> Instant { Instant::now() + Duration::from_secs(10) }

fn auth(fx: &ReleaseFixture<SupervisorInner>) -> Authority { fx.core.authority(&fx.ctx(fx.now())) }

fn quintic(fx: &ReleaseFixture<SupervisorInner>, d0: f32, duration_s: f64) -> Primitive {
    let start = fx.core.commanded_positions.clone();
    let mut q = start.clone();
    q[0] += d0;
    Primitive::joint_quintic(&start, &q, duration_s).unwrap()
}

fn submit_install(b: &mut Broker<Primitive>, seq: u64, a: Authority, deadline: Instant, p: Primitive) -> Result<RequestId, AdmitError> {
    b.submit(ClientId(1), seq, a, deadline, Submit::Install(&move |slot: &mut Primitive| *slot = p))
}

fn replies(b: &mut Broker<Primitive>) -> Vec<Reply> {
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
    let r1 = b.submit(ClientId(1), 1, a, later(), Submit::Install(&|p: &mut Primitive| *p = Primitive::joint_quintic(&[0.0; 7], &[0.1; 7], 1.0).unwrap())).unwrap();
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
    inner.set_torque(true).expect("arm");
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
    submit_install(&mut b, 2, stale, later(), Primitive::joint_quintic(&start, &q, 1.0).unwrap()).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PARENT_REPLACED"));
    assert_eq!(fx.active(), Some(parent));
    // a child of the running parent, prepared from the current commanded position
    let start = fx.core.commanded_positions.clone();
    let mut q = start.clone();
    q[0] -= 0.05;
    submit_install(&mut b, 3, a, later(), Primitive::joint_quintic(&start, &q, 1.0).unwrap()).unwrap();
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
    submit_install(&mut b, 1, a, later(), Primitive::joint_quintic(&moved, &q, 0.5).unwrap()).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    assert!(fx.active().is_none() && fx.core.active_trajectory.is_none());
    // below the 1e-4 threshold it is the same start
    let mut close = start.clone();
    close[gripper] += 5e-5;
    submit_install(&mut b, 2, a, later(), Primitive::joint_quintic(&close, &q, 0.5).unwrap()).unwrap();
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
    // the foundation adapter does not activate: the reservation comes back and is released
    let Activated::Refused { code: "NOT_WIRED", reservation } = fx.core.activate(r) else { panic!() };
    assert_eq!(fx.core.release(reservation), Released::Cleared);
    assert!(!fx.core.core.motion_inhibited_by_config());
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
    // well-formed, not wired in the foundation adapter
    submit_install(&mut b, 3, a, later(), grip(parent.0)).unwrap();
    assert_eq!(fx.step().command.unwrap().1, Err("UNSUPPORTED"));
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
