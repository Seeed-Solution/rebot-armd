//! Broker contracts against the supervisor's production adapter, stepped one release at a
//! time (`armd::rt_core::ops::ReleaseFixture`, mock follower, tiny capacities): exact
//! replies, retirement counts, and the active motion left unchanged by refused admissions
//! and refused commits.

use std::time::{Duration, Instant};

use armd::rt_core::broker::{
    AdmitError, Authority, BootId, Broker, Capacities, ClientId, ImmediateOp, Outcome, Reply, RequestId, Submit,
};
use armd::rt_core::mailbox::{StreamId, Submitted, TargetFrame, TargetWriter, MAX_DOF};
use armd::rt_core::ops::{CoreOps, Primitive, ReleaseFixture, StubCore};
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

fn auth(fx: &ReleaseFixture<SupervisorInner>) -> Authority { fx.core.authority(fx.ctx(Instant::now()).stop_generation) }

fn quintic(fx: &ReleaseFixture<SupervisorInner>, d0: f32, duration_s: f64) -> Primitive {
    let mut q = fx.core.commanded_positions.clone();
    q[0] += d0;
    Primitive::joint_quintic(&q, duration_s)
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
        Reply::Terminal { request: r, action, outcome: Outcome::Completed },
    ]);
    let s = b.stats();
    assert_eq!((s.admitted, s.acked, s.terminals, s.retired, b.reserved()), (1, 1, 1, 1, 0));
    assert_eq!(fx.reply_overflow(), 0);
}

#[test]
fn saturated_admission_and_a_stalled_consumer_leave_the_active_motion_unchanged() {
    let (mut fx, mut b, _tw, _stop) = fixture(Capacities { commands: 2, plans: 1, replies: 4 });
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
    assert!(got.contains(&Reply::Terminal { request: r1, action, outcome: Outcome::Stopped }));
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
    submit_install(&mut b, 5, a, Instant::now(), quintic(&fx, -0.1, 1.0)).unwrap();
    std::thread::sleep(Duration::from_millis(2));
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
    let frame = |seq: u64, authority: Authority, dq: f32| {
        let mut arr = [0.0; MAX_DOF];
        arr[..q.len()].copy_from_slice(&q);
        arr[0] += dq;
        TargetFrame { stream: StreamId(1), seq, authority, policy_generation: 0, input_at: Instant::now(), vision_at: None, dof: q.len() as u8, q: arr, replaceable: true }
    };
    assert_eq!(tw.submit(frame(1, a, 0.001)), Submitted::Accepted);
    assert_eq!(tw.submit(frame(2, a, 0.002)), Submitted::Coalesced { replaced: 1 });
    let rep = fx.step();
    assert_eq!(rep.target, Some((2, Ok(()))));
    assert!(fx.step().target.is_none(), "consumed once");
    tw.submit(frame(3, Authority { epoch: a.epoch + 1, ..a }, 0.003));
    assert_eq!(fx.step().target, Some((3, Err("EPOCH_MISMATCH"))));
}

#[test]
fn a_stub_core_never_acknowledges_success() {
    let (mut fx, mut b, mut tw, _stop) = ReleaseFixture::new(StubCore::default(), BOOT, Capacities { commands: 4, plans: 2, replies: 8 });
    let a = Authority::default();
    let r1 = b.submit(ClientId(1), 1, a, later(), Submit::Install(&|p: &mut Primitive| *p = Primitive::joint_quintic(&[0.1; 7], 1.0))).unwrap();
    let r2 = b.submit(ClientId(1), 2, a, later(), Submit::Immediate(ImmediateOp::TorqueOn)).unwrap();
    tw.submit(TargetFrame { stream: StreamId(9), seq: 1, authority: a, policy_generation: 0, input_at: Instant::now(), vision_at: None, dof: 1, q: [0.0; MAX_DOF], replaceable: true });
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
    let snap = fx.core.planning_snapshot(stop.current());
    assert_eq!(snap.published_seq, fx.core.core.published_seq());
    assert_eq!(snap.authority.stop_generation, 1);
    assert_eq!(snap.measured.len(), fx.core.config.dof);
    assert!(!snap.motion_active);
    assert!(fx.core.poll_hardware().driverless);
}
