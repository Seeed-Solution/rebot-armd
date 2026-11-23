//! Child primitives of macro actions (armd 1.6 RT core, track E): child identity and count,
//! cancellation between preparation and install, late results after the parent was replaced,
//! the RT deadline of a child and the contact guard evaluated by the control tick.
mod common;

use std::sync::Mutex;
use std::time::{Duration, Instant};

use armd::macro_action::{ChildTicket, ContactGuard, SubState};
use armd::model::ArmConfig;
use armd::rt_core::broker::BootId;
use armd::rt_core::ids::ArmId;
use armd::rt_core::ops::{CommitCtx, CoreOps};
use armd::SupervisorInner;
use common::{TestArm, INSPECT};

const DT: f64 = 0.02;

fn inner() -> SupervisorInner {
    let mut inner = common::sim_at(ArmConfig { collision: None, ..ArmConfig::rebot_dm() }, &INSPECT);
    for _ in 0..5 { inner.tick(DT); }
    inner
}

fn begin(inner: &mut SupervisorInner) -> u64 {
    let epoch = inner.safety.epoch_id();
    inner.begin_orchestrated_action("sequence", None, epoch).unwrap()
}

/// A J1 joint-space child `dq` away from the commanded state.
fn install(inner: &mut SupervisorInner, id: u64, dq: f32) -> Result<f64, armd::ArmError> {
    let mut target = inner.commanded_positions.clone();
    target[0] += dq;
    let limits = inner.planning_limits();
    inner.install_macro_quintic(id, target, &limits)
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
    assert_eq!(first, ChildTicket { parent: id, seq: 1 });
    run_until_done(&mut inner, first);
    install(&mut inner, id, -0.05).unwrap();
    let second = inner.arm_macro_child(id, Instant::now() + Duration::from_secs(30), "SEQUENCE_TIMEOUT", "x".into()).unwrap();
    assert_eq!(second.seq, 2);
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
    assert!(inner.install_macro_quintic(id, target, &limits).is_err(), "install after the stop is refused");
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
    let arm = TestArm::new();
    let mut inner = SupervisorInner::new(ArmConfig { collision: None, lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.lease_enabled = false;
    inner.driver = Some(Mutex::new(Box::new(arm.clone())));
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    inner.set_torque(true).unwrap();
    for _ in 0..5 { inner.tick(DT); }
    let id = begin(&mut inner);
    let guard = ContactGuard { stop_nm: Some(2.0), table_z_m: 0.0 };
    inner.guard_next_macro_child(id, Some(guard));
    install(&mut inner, id, 0.5).unwrap();
    let t = inner.arm_macro_child(id, Instant::now() + Duration::from_secs(30), "GRASP_TIMEOUT", "x".into()).unwrap();
    for _ in 0..10 { inner.tick(DT); }
    assert_eq!(inner.macro_child_state(t), Some(SubState::Running));
    let w = inner.macro_child_watch(t).unwrap();
    assert!(w.peak_residual_nm.is_some_and(|p| p[0] < 0.05) && w.contact_height_m.is_none(), "{w:?}");
    // contact on J2
    arm.set_residual(1, 3.0);
    inner.tick(DT);
    let at_stop = inner.commanded_positions.clone();
    assert!(matches!(inner.macro_child_state(t), Some(SubState::Done(_))), "{:?}", inner.macro_child_state(t));
    assert!(inner.active_trajectory.is_none(), "the child motion ended");
    let w = inner.macro_child_watch(t).unwrap();
    assert!(w.contact_height_m.is_some(), "{w:?}");
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
