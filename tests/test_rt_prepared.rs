//! Prepared plan slots (armd 1.6 RT core, requests C2 / E3): the worker plans a Cartesian
//! path (IK) or reads and resamples an episode, and stages the joint samples in the plan
//! slot's preallocated buffer; the RT commit installs them without IK or file I/O. Production
//! adapter (`SupervisorInner`, mock follower) stepped one release at a time.

mod common;

use std::time::{Duration, Instant};

use armd::motion::{plan_cartesian, prepare_cartesian_path, resolve_goal, GoalSpec, JointPath, Speed};
use armd::rt_core::broker::{ActionId, Authority, BootId, Broker, Capacities, ClientId, Outcome, Reply, RequestId, Submit};
use armd::rt_core::mailbox::MAX_DOF;
use armd::rt_core::ops::{
    tool_digest_hash, ChildSpec, ChildTicket, CoreOps, EpisodeRef, PathKind, PlanSlot, PreparedPath, Primitive, ReleaseFixture, MAX_PATH_SAMPLES,
};
use armd::{ArmConfig, SupervisorInner};
use common::INSPECT;

const BOOT: BootId = BootId(5);

type Fx = ReleaseFixture<SupervisorInner>;

fn fixture() -> (Fx, Broker<PlanSlot>) {
    let mut inner = common::sim_at(ArmConfig { collision: None, ..ArmConfig::rebot_dm() }, &INSPECT);
    for _ in 0..5 { inner.tick(0.02); }
    let (fx, b, _tw, _stop) = ReleaseFixture::new(inner, BOOT, Capacities { commands: 4, plans: 2, replies: 16 });
    (fx, b)
}

fn auth(fx: &Fx) -> Authority { fx.core.authority(&fx.ctx(fx.now())) }

fn later() -> Instant { Instant::now() + Duration::from_secs(30) }

fn submit(b: &mut Broker<PlanSlot>, seq: u64, a: Authority, prepared: &PlanSlot) -> RequestId {
    let prepared = prepared.clone();
    b.submit(ClientId(1), seq, a, later(), Submit::Install(&move |slot: &mut PlanSlot| slot.copy_from(&prepared).unwrap())).unwrap()
}

/// A 3 cm move along x from the commanded TCP pose, planned on the worker side.
fn planned_path(fx: &Fx) -> JointPath {
    let input = fx.core.plan_input().unwrap();
    let p0 = armd::motion::start_pose(&input).p;
    let goal = resolve_goal(&input, &GoalSpec::Absolute { p: [p0[0] + 0.03, p0[1], p0[2]], r: None });
    plan_cartesian(&input, goal, Speed::default()).unwrap()
}

fn run_until_ended(fx: &mut Fx, max: usize) -> (ActionId, Outcome) {
    for _ in 0..max { if let Some(e) = fx.step().ended { return e; } }
    panic!("the action did not end: {:?}", fx.core.last_action);
}

#[test]
fn a_prepared_cartesian_path_commits_without_planning_and_runs_to_its_end() {
    use armd::motion::Reference;
    let (mut fx, mut b) = fixture();
    let path = planned_path(&fx);
    let last = path.last();
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    let slot = prepare_cartesian_path(&snap, &path, None).unwrap();
    assert_eq!(slot.path.len(), path.q.len());
    let r = submit(&mut b, 1, auth(&fx), &slot);
    let rep = fx.step();
    assert!(matches!(rep.command, Some((req, Ok(_))) if req == r), "{rep:?}");
    let borrows = matches!(
        fx.core.active_trajectory.as_ref().map(|a| &a.reference),
        Some(Reference::Path(p)) if p.borrows_plan_slot()
    );
    assert!(borrows, "prepared Cartesian path must borrow directly from the pinned plan slot");
    b.poll(|_, _| true);
    assert_eq!(b.stats().retired, 0, "pinned plan slot must not be reclaimed while Cartesian path runs");
    let action = fx.active().unwrap();
    assert_eq!(fx.core.last_action.as_ref().map(|a| a.kind.as_str()), Some("move_to"));
    assert_eq!(run_until_ended(&mut fx, 400), (action, Outcome::Completed));
    for (c, q) in fx.core.commanded_positions.iter().zip(last) { assert!((c - q).abs() < 1e-3, "{:?} vs {last:?}", fx.core.commanded_positions); }
    let got = { let mut v = Vec::new(); b.poll(|_, r| { v.push(r); true }); v };
    assert_eq!(b.stats().retired, 1, "pinned plan slot reclaimed after Cartesian path terminal");
    assert!(matches!(got.last(), Some(Reply::Terminal { outcome: Outcome::Completed, .. })), "{got:?}");
}

#[test]
fn a_prepared_path_whose_start_moved_or_samples_do_not_fit_is_refused() {
    let (mut fx, mut b) = fixture();
    let path = planned_path(&fx);
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    let slot = prepare_cartesian_path(&snap, &path, None).unwrap();
    // the commanded gripper moved by 1e-3 since the preparation: stale, nothing installed
    fx.core.commanded_positions[6] += 1e-3;
    submit(&mut b, 1, auth(&fx), &slot);
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
    assert!(fx.core.active_trajectory.is_none());
    // a path planned from another start is refused before admission
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    assert_eq!(prepare_cartesian_path(&snap, &path, None).unwrap_err(), "PLAN_STALE");
    // the buffer never grows: a full one refuses the next sample
    let mut buf = PreparedPath::with_capacity(2);
    buf.push(&[0.0; 7], 0.0).unwrap();
    buf.push(&[0.0; 7], 1.0).unwrap();
    assert_eq!(buf.push(&[0.0; 7], 1.0), Err("PLAN_TOO_LARGE"));
    assert_eq!(PlanSlot::default().path.capacity(), MAX_PATH_SAMPLES);
    let mut small = PlanSlot::with_path_capacity(1);
    assert_eq!(small.copy_from(&slot), Err("PLAN_TOO_LARGE"));
    assert_eq!(small.primitive, Primitive::None, "nothing staged");
}

/// A prepared replay: samples every 20 ms moving J1 by 0.1 rad over 1 s (as `prepare_replay`
/// stages a resampled episode); the commit plans only the approach.
fn replay_slot(fx: &Fx, tool_digest: u64) -> PlanSlot {
    let start = fx.core.commanded_positions.clone();
    let mut slot = PlanSlot::default();
    for k in 0..=50 {
        let mut q = start.clone();
        q[0] += 0.1 * k as f32 / 50.0;
        slot.path.push(&q, k as f64).unwrap();
    }
    let mut s = [0.0; MAX_DOF];
    s[..start.len()].copy_from_slice(&start);
    slot.replay_approach = Some(
        armd::replay::PreparedReplayApproach::from_endpoints(&start, &start, 0.2, None, true)
            .unwrap(),
    );
    slot.primitive = Primitive::Prepared {
        child: None, dof: start.len() as u8, start: s,
        kind: PathKind::Replay {
            episode: EpisodeRef::new("ep1").unwrap(),
            speed: 1.0,
            gripper: false,
            sample_dt: 0.02,
            approach_duration_s: 0.2,
            approach_power_scale: None,
            tool_digest,
        },
        duration_s: 1.0, power_scale: None,
    };
    slot
}

#[test]
fn a_prepared_replay_commits_from_the_slot_and_checks_the_tool_it_was_prepared_for() {
    let (mut fx, mut b) = fixture();
    let tool = tool_digest_hash(&fx.core.config.active_tool.config_digest);
    // another tool than the active one: refused, nothing installed
    submit(&mut b, 1, auth(&fx), &replay_slot(&fx, tool ^ 1));
    assert_eq!(fx.step().command.unwrap().1, Err("TOOL_MISMATCH"));
    assert!(fx.core.active_trajectory.is_none());
    let target = fx.core.commanded_positions[0] + 0.1;
    submit(&mut b, 2, auth(&fx), &replay_slot(&fx, tool));
    assert!(fx.step().command.unwrap().1.is_ok());
    let action = fx.active().unwrap();
    assert_eq!(fx.core.last_action.as_ref().map(|a| (a.kind.as_str(), a.episode.as_deref())), Some(("replay", Some("ep1"))));
    assert_eq!(run_until_ended(&mut fx, 400), (action, Outcome::Completed));
    assert!((fx.core.commanded_positions[0] - target).abs() < 2e-3, "{:?}", fx.core.commanded_positions);
}

#[test]
fn a_prepared_path_child_of_a_running_macro() {
    let (mut fx, mut b) = fixture();
    let epoch = fx.core.safety.epoch_id();
    let id = { let __s = fx.core.current_admission_stamp(); fx.core.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| armd::SupervisorInner::begin_orchestrated_action(__a, "sequence", None, epoch)) }.unwrap();
    let path = planned_path(&fx);
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    assert_eq!(snap.authority.parent_generation, id);
    let spec = ChildSpec { ticket: ChildTicket { parent: ActionId(id), child_seq: 1 }, timeout_s: 10.0, timeout_code: "SEQUENCE_TIMEOUT", contact: None };
    let slot = prepare_cartesian_path(&snap, &path, Some(spec)).unwrap();
    submit(&mut b, 1, snap.authority, &slot);
    let rep = fx.step();
    assert!(rep.command.unwrap().1.is_ok(), "{rep:?}");
    assert_eq!((fx.active(), fx.core.macro_children(id)), (Some(ActionId(id)), Some(1)));
    // the same child again is stale
    submit(&mut b, 2, auth(&fx), &slot);
    assert_eq!(fx.step().command.unwrap().1, Err("PLAN_STALE"));
}

#[test]
fn a_prepared_replay_borrows_its_plan_slot_until_terminal_retirement() {
    use armd::motion::Reference;
    let (mut fx, mut b) = fixture();
    let tool = tool_digest_hash(&fx.core.config.active_tool.config_digest);
    submit(&mut b, 1, auth(&fx), &replay_slot(&fx, tool));
    assert!(fx.step().command.unwrap().1.is_ok());
    let borrows = matches!(
        fx.core.active_trajectory.as_ref().map(|a| &a.reference),
        Some(Reference::Replay(r)) if r.borrows_plan_slot() && r.episode_str() == "ep1" && r.sample_count() == 51
    );
    assert!(borrows, "prepared replay must hold a pinned PlanSlotRef rather than copying samples");
    b.poll(|_, _| true);
    assert_eq!(b.stats().retired, 0, "pinned plan slot must not be reclaimed while replay runs");
    let action = fx.active().unwrap();
    assert_eq!(run_until_ended(&mut fx, 400), (action, Outcome::Completed));
    let mut got = Vec::new();
    b.poll(|_, r| { got.push(r); true });
    assert_eq!(b.stats().retired, 1, "pinned plan slot reclaimed after terminal retirement");
    assert!(got.iter().any(|r| matches!(r, Reply::Terminal { outcome: Outcome::Completed, .. })));
}

#[test]
fn a_prepared_path_or_replay_with_stale_boot_or_overtaken_stop_is_refused() {
    use armd::rt_core::broker::{CommandKind, Descriptor, Stamp};
    use armd::rt_core::ids::ArmId;
    use armd::rt_core::ops::CommitCtx;

    let mut inner = common::sim_at(ArmConfig { collision: None, ..ArmConfig::rebot_dm() }, &INSPECT);
    for _ in 0..5 { inner.tick(0.02); }
    let live_boot = BootId::fresh();
    let stale_boot = BootId::fresh();
    let (mut fx, mut b, _tw, stop) = ReleaseFixture::new(inner, live_boot, Capacities { commands: 4, plans: 2, replies: 16 });
    let path = planned_path(&fx);
    let snap = fx.core.planning_snapshot(&fx.ctx(fx.now()));
    let cart_slot = prepare_cartesian_path(&snap, &path, None).unwrap();
    let tool = tool_digest_hash(&fx.core.config.active_tool.config_digest);
    let rep_slot = replay_slot(&fx, tool);

    // Overtaken stop refuses queued prepared Cartesian path and prepared Replay.
    let pre_stop_auth = auth(&fx);
    submit(&mut b, 1, pre_stop_auth, &cart_slot);
    submit(&mut b, 2, pre_stop_auth, &rep_slot);
    stop.request();
    assert_eq!(fx.step().command.unwrap().1, Err("STOPPED"));
    assert_eq!(fx.step().command.unwrap().1, Err("STOPPED"));
    assert!(fx.core.active_trajectory.is_none());

    // Stale boot id and zero boot id are refused fail-closed on commit_prepared_slot.
    let (mut pw, mut pr) = armd::rt_core::broker::plan_slots(2, PlanSlot::default);
    let hc = pw.stage(|s| s.copy_from(&cart_slot).unwrap()).unwrap();
    let hr = pw.stage(|s| s.copy_from(&rep_slot).unwrap()).unwrap();
    let ctx = CommitCtx { arm: ArmId::LOCAL, boot: live_boot, now: fx.now(), stop_generation: stop.current() };
    let cur_auth = auth(&fx);
    let make_desc = |boot: BootId, plan| Descriptor {
        stamp: Stamp {
            arm: ArmId::LOCAL,
            boot,
            request: RequestId(99),
            client: ClientId(1),
            client_seq: 1,
            authority: cur_auth,
            deadline: later(),
        },
        kind: CommandKind::Install { plan },
    };
    assert_eq!(fx.core.commit_prepared_slot(&make_desc(stale_boot, hc), pr.pin(hc).unwrap(), &ctx), Err("WRONG_TARGET"));
    assert_eq!(fx.core.commit_prepared_slot(&make_desc(BootId(0), hc), pr.pin(hc).unwrap(), &ctx), Err("WRONG_TARGET"));
    assert_eq!(fx.core.commit_prepared_slot(&make_desc(stale_boot, hr), pr.pin(hr).unwrap(), &ctx), Err("WRONG_TARGET"));
    assert_eq!(fx.core.commit_prepared_slot(&make_desc(BootId(0), hr), pr.pin(hr).unwrap(), &ctx), Err("WRONG_TARGET"));
    assert!(fx.core.active_trajectory.is_none());
    assert!(pr.retire(hc) && pr.retire(hr));
    assert_eq!(pw.reclaim(), 2, "refused commits must not leak plan slot pins");
}
