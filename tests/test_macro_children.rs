//! Child primitives of macro actions (armd 1.6 RT core, track E): child identity and count,
//! cancellation between preparation and install, late results after the parent was replaced,
//! the RT deadline of a child and the contact guard evaluated by the control tick.
mod common;

use std::sync::Mutex;
use std::time::{Duration, Instant};

use armd::macro_action::{ActionId, ChildSpec, ChildTicket, ContactGuard, SubState};
use armd::rt_core::broker::Outcome;
use armd::rt_core::mailbox::MAX_DOF;
use armd::rt_core::ops::Primitive;
use armd::model::ArmConfig;
use armd::rt_core::broker::BootId;
use armd::rt_core::ids::ArmId;
use armd::rt_core::ops::{CommitCtx, CoreOps};
use armd::SupervisorInner;
use common::{TestArm, INSPECT};

const DT: f64 = 0.02;

fn j2_guard(nm: f32, stop_code: Option<&'static str>) -> ContactGuard {
    let mut max_residual_nm = [0.0; MAX_DOF];
    max_residual_nm[1] = nm;
    ContactGuard { max_residual_nm, stop_code }
}

fn driven() -> (SupervisorInner, TestArm) {
    let arm = TestArm::new();
    let mut inner = SupervisorInner::new(ArmConfig { collision: None, lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| inner.enable_torque(__a)) }.unwrap();
    for _ in 0..5 { inner.tick(DT); }
    (inner, arm)
}

fn inner() -> SupervisorInner {
    let mut inner = common::sim_at(ArmConfig { collision: None, ..ArmConfig::rebot_dm() }, &INSPECT);
    for _ in 0..5 { inner.tick(DT); }
    inner
}

fn begin(inner: &mut SupervisorInner) -> u64 {
    let epoch = inner.safety.epoch_id();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.begin_orchestrated_action(__a, "sequence", None, epoch)) } }.unwrap()
}

/// A J1 joint-space child `dq` away from the commanded state.
fn install(inner: &mut SupervisorInner, id: u64, dq: f32) -> Result<f64, armd::ArmError> {
    let mut target = inner.commanded_positions.clone();
    target[0] += dq;
    let limits = inner.planning_limits();
    { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.install_macro_quintic(__a, id, target, &limits)) } }
}

fn run_until_done(inner: &mut SupervisorInner, t: ChildTicket) {
    for _ in 0..500 {
        inner.tick(DT);
        if matches!(inner.macro_child_state(t), Some(SubState::Done(_))) { return; }
    }
    panic!("child {t:?} did not finish: {:?}", inner.macro_child_state(t));
}

#[test]
fn children_are_counted_and_a_finished_childs_ticket_does_not_read_the_next_one() {
    let mut inner = inner();
    let id = begin(&mut inner);
    assert_eq!(inner.macro_children(id), Some(0));
    install(&mut inner, id, 0.05).unwrap();
    let first = inner.arm_macro_child(id, Instant::now() + Duration::from_secs(30), "SEQUENCE_TIMEOUT", "x".into()).unwrap();
    assert_eq!(first, ChildTicket { parent: ActionId(id), child_seq: 1 });
    run_until_done(&mut inner, first);
    install(&mut inner, id, -0.05).unwrap();
    let second = inner.arm_macro_child(id, Instant::now() + Duration::from_secs(30), "SEQUENCE_TIMEOUT", "x".into()).unwrap();
    assert_eq!(second.child_seq, 2);
    assert_eq!(inner.macro_children(id), Some(2), "two physical children");
    // the first child's result is not the second's
    assert_eq!(inner.macro_child_state(first), None);
    assert_eq!(inner.macro_child_state(second), Some(SubState::Running));
    run_until_done(&mut inner, second);
}

#[test]
fn stop_between_preparation_and_install_installs_nothing() {
    let mut inner = inner();
    let id = begin(&mut inner);
    // preparation: the worker plans from a snapshot
    let ctx = CommitCtx { arm: ArmId::LOCAL, boot: BootId(1), now: Instant::now(), stop_generation: 0 };
    let snap = inner.planning_snapshot(&ctx);
    assert_eq!(snap.authority.parent_generation, id, "the running macro is the parent its children commit under");
    let mut target = snap.commanded.clone();
    target[0] += 0.1;
    let before = inner.commanded_positions.clone();
    // the stop arrives before the install
    inner.request_macro_stop();
    let limits = inner.planning_limits();
    assert!({ let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.install_macro_quintic(__a, id, target, &limits)) } }.is_err(), "install after the stop is refused");
    assert!(inner.active_trajectory.is_none(), "no physical child");
    assert_eq!(inner.macro_children(id), None, "the run record is gone");
    let a = inner.last_action.clone().unwrap();
    assert_eq!((a.state.as_str(), a.code.as_deref()), ("cancelled", Some("STOPPED")));
    for _ in 0..25 { inner.tick(DT); }
    assert_eq!(inner.commanded_positions, before, "the arm did not move");
}

#[test]
fn a_replaced_parent_reads_its_child_as_cancelled_and_cannot_install() {
    let mut inner = inner();
    let a = begin(&mut inner);
    install(&mut inner, a, 0.05).unwrap();
    let ta = inner.arm_macro_child(a, Instant::now() + Duration::from_secs(30), "SEQUENCE_TIMEOUT", "x".into()).unwrap();
    run_until_done(&mut inner, ta);
    // a new macro replaces A while A's worker is between children
    let b = begin(&mut inner);
    assert_ne!(a, b);
    assert_eq!(inner.macro_child_state(ta), None, "A's late result is not read");
    assert!(install(&mut inner, a, 0.05).is_err(), "A cannot install under B");
    assert!(inner.arm_macro_child(a, Instant::now(), "X", "x".into()).is_none());
    // A's late finish does not touch B
    inner.finish_macro(a, "failed", Some("GRASP_TIMEOUT"), None, None, None);
    assert_eq!(inner.macro_children(b), Some(0));
    let rec = inner.last_action.clone().unwrap();
    assert_eq!((rec.id, rec.state.as_str()), (b, "running"));
    assert!(inner.active_trajectory.is_none());
}

#[test]
fn the_tick_fails_an_overdue_child_with_the_timeout_code() {
    let mut inner = inner();
    let id = begin(&mut inner);
    install(&mut inner, id, 0.6).unwrap();
    let t = inner.arm_macro_child(id, Instant::now() + Duration::from_millis(30), "SEQUENCE_TIMEOUT", "Phase pose did not finish in time".into()).unwrap();
    std::thread::sleep(Duration::from_millis(40));
    inner.tick(DT);
    match inner.macro_child_state(t) {
        Some(SubState::Failed { code, message }) => {
            assert_eq!(code, "SEQUENCE_TIMEOUT");
            assert_eq!(message, "Phase pose did not finish in time");
        }
        other => panic!("expected the timeout failure: {other:?}"),
    }
    assert!(inner.active_trajectory.is_none(), "the motion stopped");
    // the parent is not ended as STOPPED: its worker ends it with the timeout code
    for _ in 0..5 { inner.tick(DT); }
    assert!(inner.last_action.as_ref().unwrap().is_running());
    inner.finish_macro(id, "failed", Some("SEQUENCE_TIMEOUT"), Some("Phase pose did not finish in time".into()), None, None);
    assert_eq!(inner.last_action.as_ref().unwrap().code.as_deref(), Some("SEQUENCE_TIMEOUT"));
    assert_eq!(inner.macro_children(id), None);
}

#[test]
fn contact_during_a_guarded_descent_ends_the_child_in_the_tick() {
    let (mut inner, arm) = driven();
    let id = begin(&mut inner);
    let guard = j2_guard(2.0, None);
    inner.guard_next_macro_child(id, Some(guard));
    install(&mut inner, id, 0.5).unwrap();
    let t = inner.arm_macro_child(id, Instant::now() + Duration::from_secs(30), "GRASP_TIMEOUT", "x".into()).unwrap();
    for _ in 0..10 { inner.tick(DT); }
    assert_eq!(inner.macro_child_state(t), Some(SubState::Running));
    let w = inner.macro_child_watch(t).unwrap();
    assert!(w.peak_residual_nm.is_some_and(|p| p[0] < 0.05) && w.contact_joint.is_none(), "{w:?}");
    // contact on J2
    arm.set_residual(1, 3.0);
    inner.tick(DT);
    let at_stop = inner.commanded_positions.clone();
    assert!(matches!(inner.macro_child_state(t), Some(SubState::Done(_))), "{:?}", inner.macro_child_state(t));
    assert!(inner.active_trajectory.is_none(), "the child motion ended");
    let w = inner.macro_child_watch(t).unwrap();
    assert_eq!(w.contact_joint, Some(1), "{w:?}");
    assert!(w.contact_tcp_z_m.is_some_and(f64::is_finite), "{w:?}");
    assert!(w.peak_residual_nm.is_some_and(|p| (p[0] - 3.0).abs() < 0.05), "{w:?}");
    for (c, m) in at_stop.iter().zip(&inner.measured_positions).take(6) { assert!((c - m).abs() < 1e-6, "held at the measured position"); }
    for _ in 0..10 { inner.tick(DT); }
    assert_eq!(&inner.commanded_positions[..6], &at_stop[..6], "no further motion");
    assert_eq!(inner.macro_children(id), Some(1));
    // the guard belonged to that child only
    install(&mut inner, id, -0.1).unwrap();
    let t2 = inner.macro_run.as_ref().unwrap().ticket().unwrap();
    assert!(inner.macro_run.as_ref().unwrap().child.guard.is_none());
    assert_eq!(inner.macro_child_watch(t2).unwrap(), Default::default());
}

/// A `JointQuinticLimited` child for `parent` / `seq`, J1 `dq` away from the commanded state.
fn joint_child(inner: &SupervisorInner, parent: u64, seq: u32, dq: f32, timeout_s: f64, contact: Option<ContactGuard>) -> Primitive {
    let dof = inner.commanded_positions.len();
    let mut start = [0.0; MAX_DOF];
    start[..dof].copy_from_slice(&inner.commanded_positions);
    let mut q = start;
    q[0] += dq;
    Primitive::JointQuinticLimited {
        child: Some(ChildSpec { ticket: ChildTicket { parent: ActionId(parent), child_seq: seq }, timeout_s, timeout_code: "SEQUENCE_TIMEOUT", contact }),
        dof: dof as u8, start, q, duration_s: 1.0, max_velocity: [0.0; MAX_DOF], settle_s: 0.0,
    }
}

#[test]
fn child_primitive_commit_checks_parent_sequence_start_and_busy() {
    let mut inner = inner();
    let id = begin(&mut inner);
    let now = Instant::now();
    // not the next child, a stale start, a parent that is not running: nothing installed
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &joint_child(&inner, id, 2, 0.05, 30.0, None), now) }, Err("PLAN_STALE"));
    let mut stale = joint_child(&inner, id, 1, 0.05, 30.0, None);
    if let Primitive::JointQuinticLimited { start, .. } = &mut stale { start[6] += 1e-3; }
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &stale, now) }, Err("PLAN_STALE"), "the gripper counts");
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &joint_child(&inner, id + 7, 1, 0.05, 30.0, None), now) }, Err("PARENT_REPLACED"));
    assert!(inner.active_trajectory.is_none());
    assert_eq!(inner.macro_children(id), Some(0));
    // the next child installs; a second while it runs is busy
    let target = inner.commanded_positions[0] + 0.05;
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &joint_child(&inner, id, 1, 0.05, 30.0, None), now) }, Ok(ActionId(id)));
    assert_eq!(inner.macro_children(id), Some(1));
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &joint_child(&inner, id, 2, 0.05, 30.0, None), now) }, Err("MOTION_BUSY"));
    let t = inner.macro_run.as_ref().unwrap().ticket().unwrap();
    run_until_done(&mut inner, t);
    assert!((inner.commanded_positions[0] - target).abs() < 1e-3);
    // a gripper child to half open
    let dof = inner.commanded_positions.len();
    let mut start = [0.0; MAX_DOF];
    start[..dof].copy_from_slice(&inner.commanded_positions);
    let gripper = Primitive::Gripper {
        child: Some(ChildSpec { ticket: ChildTicket { parent: ActionId(id), child_seq: 2 }, timeout_s: 10.0, timeout_code: "GRIPPER_TIMEOUT", contact: None }),
        dof: dof as u8, start, fraction: 0.5, speed: 1.0, grasp: false,
    };
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &gripper, Instant::now()) }, Ok(ActionId(id)));
    let t = inner.macro_run.as_ref().unwrap().ticket().unwrap();
    assert_eq!(t.child_seq, 2);
    run_until_done(&mut inner, t);
    assert_eq!(inner.macro_children(id), Some(2));
}

#[test]
fn a_child_primitive_timeout_and_the_macro_terminal_record() {
    let mut inner = inner();
    let id = begin(&mut inner);
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &joint_child(&inner, id, 1, 0.6, 0.0, None), Instant::now()) }, Ok(ActionId(id)));
    let t = inner.macro_run.as_ref().unwrap().ticket().unwrap();
    std::thread::sleep(Duration::from_millis(2));
    inner.tick(DT);
    assert!(matches!(inner.macro_child_state(t), Some(SubState::Failed { ref code, .. }) if code == "SEQUENCE_TIMEOUT"), "{:?}", inner.macro_child_state(t));
    assert!(inner.active_trajectory.is_none());
    inner.finish_macro(id, "failed", Some("SEQUENCE_TIMEOUT"), None, None, None);
    // the record outlives the action record and a newer action
    let rec = inner.core.last_macro().unwrap();
    assert_eq!((rec.action, rec.children, rec.outcome), (ActionId(id), 1, Outcome::Failed("SEQUENCE_TIMEOUT")));
    assert_eq!(CoreOps::action_outcome(&inner, ActionId(id)), Some(Outcome::Failed("SEQUENCE_TIMEOUT")));
    assert_eq!(CoreOps::action_children(&inner, ActionId(id)), 1);
    // a stop of a running macro: Stopped with its child count
    let id2 = begin(&mut inner);
    install(&mut inner, id2, 0.05).unwrap();
    inner.request_macro_stop();
    let rec = inner.core.last_macro().unwrap();
    assert_eq!((rec.action, rec.children, rec.outcome), (ActionId(id2), 1, Outcome::Stopped));
}

#[test]
fn a_contact_guard_with_a_stop_code_fails_the_child() {
    let (mut inner, arm) = driven();
    let id = begin(&mut inner);
    assert_eq!({ let a = inner.check_admission(inner.current_admission_stamp()).unwrap(); inner.commit_macro_child(a, &joint_child(&inner, id, 1, 0.5, 30.0, Some(j2_guard(2.0, Some("COLLISION")))), Instant::now()) }, Ok(ActionId(id)));
    let t = inner.macro_run.as_ref().unwrap().ticket().unwrap();
    for _ in 0..5 { inner.tick(DT); }
    arm.set_residual(1, 3.0);
    inner.tick(DT);
    match inner.macro_child_state(t) {
        Some(SubState::Failed { code, message }) => assert_eq!((code.as_str(), message.as_str()), ("COLLISION", "contact on J2")),
        other => panic!("{other:?}"),
    }
    assert!(inner.active_trajectory.is_none());
    assert_eq!(inner.macro_child_watch(t).unwrap().contact_joint, Some(1));
}

#[test]
fn a_newer_action_records_the_superseded_macros_terminal() {
    let mut inner = inner();
    let id = begin(&mut inner);
    install(&mut inner, id, 0.05).unwrap();
    // a jog starts a new action while the macro's child runs
    let mut q = inner.commanded_positions.clone();
    q[0] -= 0.05;
    inner.stop_motion();
    let (_, jog, _) = { let s = inner.current_admission_stamp(); { let __s = s; inner.check_admission(__s).map_err(armd::ArmError::from).and_then(|__a| inner.jog_action(__a, q, Some(0.5), None, "joint_move")) } }.unwrap();
    assert!(inner.macro_run.is_none() && jog != id);
    let rec = inner.core.last_macro().unwrap();
    assert_eq!((rec.action, rec.children, rec.outcome), (ActionId(id), 1, Outcome::Superseded));
    assert_eq!(CoreOps::action_outcome(&inner, ActionId(id)), Some(Outcome::Superseded));
    assert_eq!(CoreOps::action_children(&inner, ActionId(id)), 1);
}
