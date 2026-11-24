//! Torque transitions through the RT contract on the production adapter (armd 1.6 RT core,
//! integration of track A): `commit_transition` starts the Damiao handshake
//! (`ArmDriver::begin_torque`), `poll_hardware` serves it within the release's budget
//! (`poll_torque`, then the supervisor's disable confirmation), and the outcome is applied to
//! the supervisor state when it ends. Real `DamiaoArmDriver` on the fake CAN bus, driver time
//! on a `MockClock` advanced one release at a time.

mod mit_support;

use std::time::Duration;

use armd::driver::{ArmDriver, DamiaoArmDriver, GripperPosForce};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{Authority, BootId, Broker, Capacities, ClientId, Outcome, Reply, Submit, TransitionOp};
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture, ReleaseReport, VirtualClock};
use armd::transport::MockClock;
use armd::{ArmConfig, SupervisorInner};
use mit_support::FakeBus;

const BOOT: BootId = BootId(11);

fn limits() -> Vec<DamiaoLimits> { ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect() }

struct Rig {
    fx: ReleaseFixture<SupervisorInner>,
    b: Broker<PlanSlot>,
    bus: FakeBus,
    #[allow(dead_code)]
    clock: MockClock,
    seq: u64,
}

fn rig() -> Rig { rig_with(|d| d) }

/// J7 in Damiao position-force mode with a current ceiling (track A's gripper drive).
fn pos_force(d: DamiaoArmDriver<FakeBus>) -> DamiaoArmDriver<FakeBus> {
    d.with_gripper_pos_force(GripperPosForce { joint: 6, current_raw: 220, close_speed_rad_s: 0.30, closed_rad: 0.0, open_rad: -5.7,
        calibrated: false, ceiling_torque_nm: None, torque_reference: None }).unwrap()
}

fn rig_with(drive: impl FnOnce(DamiaoArmDriver<FakeBus>) -> DamiaoArmDriver<FakeBus>) -> Rig {
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits(), clock.clone());
    let d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(limits()).unwrap();
    let mut d = drive(d);
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() }).with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    let (fx, b, _tw, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities { commands: 4, plans: 1, replies: 8 },
        VirtualClock::from_mock_clock(clock.clone()),
    );
    Rig { fx, b, bus, clock, seq: 0 }
}

impl Rig {
    fn auth(&self) -> Authority { self.fx.core.authority(&self.fx.ctx(self.fx.now())) }

    fn submit(&mut self, op: TransitionOp) {
        self.seq += 1;
        let a = self.auth();
        self.b.submit(ClientId(1), self.seq, a, self.fx.now() + Duration::from_secs(5), Submit::Transition(op)).unwrap();
    }

    /// One release; executor and driver share `self.clock` so sub-release wakes advance driver time too.
    fn step(&mut self) -> ReleaseReport {
        self.fx.step()
    }

    /// Releases until the transition ends: (release count, outcome, reports).
    fn until_ended(&mut self, max: usize) -> (usize, Outcome, Vec<ReleaseReport>) {
        let mut reports = Vec::new();
        for k in 1..=max {
            let r = self.step();
            reports.push(r);
            if let Some((_, o)) = r.transition_ended { return (k, o, reports); }
        }
        panic!("transition did not end in {max} releases: {:?}", reports.last());
    }

    fn replies(&mut self) -> Vec<Reply> {
        let mut v = Vec::new();
        self.b.poll(|_, r| { v.push(r); true });
        v
    }

    fn enable_frames(&self) -> usize { self.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_ENABLE).count() }
}

#[test]
fn an_enable_is_withheld_until_the_driver_confirms_it_then_applied_to_the_supervisor() {
    let mut r = rig();
    r.bus.with(|s| s.motors[2].torque_reply_delay = Duration::from_millis(100));
    r.submit(TransitionOp::TorqueOn);
    let first = r.step();
    assert!(first.command.unwrap().1.is_ok(), "{first:?}");
    assert!(r.fx.transition().is_some(), "in flight");
    assert!(!r.fx.core.is_torqued, "not torqued before the confirmation: {first:?} pending {} drv {:?} {}", r.fx.core.torque_transition_pending(), r.fx.core.safety.state(), r.enable_frames());
    assert_eq!(r.enable_frames(), 7, "the enable frames went out at the commit");
    let (k, outcome, reports) = r.until_ended(20);
    assert_eq!(outcome, Outcome::Completed);
    assert!(k >= 2, "confirmed only after the delayed reply ({k} releases)");
    assert!(reports[..k - 1].iter().all(|p| matches!(p.poll.torque, Some(Ok(_)))), "pending meanwhile");
    // the supervisor state of a confirmed enable
    let inner = &r.fx.core;
    assert!(inner.is_torqued && !inner.pending_rearm && !inner.disable_pending && !inner.torque_transition_pending());
    assert!(inner.safety.state().is_operational(), "{:?}", inner.safety.state());
    let got = r.replies();
    assert!(matches!(got.last(), Some(Reply::Terminal { outcome: Outcome::Completed, .. })), "{got:?}");
}

#[test]
fn an_enable_a_motor_never_confirms_fails_with_the_fault_and_torque_stays_off() {
    // 1. Silent motor (> 100 ms stale at 150 ms enable timeout): no automatic rollback,
    //    enters FailedHold, keeps holding healthy joints, and latches non-recoverable
    //    `ENABLE_FAILED` (alongside `BUS_LOSS` because motor 4 is actually silent while torqued).
    let mut r = rig();
    r.bus.with(|s| s.motors[3].silent = true);
    r.submit(TransitionOp::TorqueOn);
    assert!(r.step().command.unwrap().1.is_ok());
    let (_, outcome, _) = r.until_ended(30);
    assert_eq!(outcome, Outcome::Failed("ENABLE_FAILED"));
    let inner = &r.fx.core;
    assert!(!inner.torque_transition_pending());
    assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
    assert!(inner.is_torqued && !inner.disable_pending, "failed enable must hold without rollback ({}, {})", inner.is_torqued, inner.disable_pending);
    assert!(matches!(inner.safety.state(), armd::SafetyState::Fault(_)), "{:?}", inner.safety.state());
    assert!(inner.safety.has_latched_fault_code(armd::FaultCode::EnableFailed));
    assert!(!inner.safety.recoverable());
    assert_eq!(
        r.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "failed enable must send ZERO automatic rollback disable frames"
    );
    // a second enable while the fault latches is refused at the commit, before any frame
    r.bus.with(|s| s.motors[3].silent = false);
    let before = r.enable_frames();
    r.submit(TransitionOp::TorqueOn);
    assert!(r.step().command.unwrap().1.is_err());
    assert_eq!(r.enable_frames(), before);

    // 2. Raised pose with fresh feedback and non-confirming motor: NO automatic rollback
    //    (0 disables), enters FailedHold, and latches non-recoverable `ENABLE_FAILED` with
    //    per-joint status.
    let mut r_raised = rig();
    r_raised.bus.with(|s| {
        s.follow_commands = false;
        s.motors[1].q = 0.40;
        s.motors[3].drop_torque_frames = 10;
        s.push_all_feedback();
    });
    r_raised.step();
    r_raised.bus.clear_log();
    r_raised.submit(TransitionOp::TorqueOn);
    assert!(r_raised.step().command.unwrap().1.is_ok());
    let mut raised_outcome = None;
    for _ in 0..30 {
        r_raised.bus.with(|s| s.push_all_feedback());
        let rep = r_raised.step();
        if let Some((_, o)) = rep.transition_ended {
            raised_outcome = Some(o);
            break;
        }
    }
    assert_eq!(raised_outcome, Some(Outcome::Failed("ENABLE_FAILED")));
    assert_eq!(r_raised.fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
    assert!(r_raised.fx.core.is_torqued && !r_raised.fx.core.disable_pending);
    assert_eq!(
        r_raised.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "raised failed enable must send ZERO automatic rollback disable frames"
    );
    let raised_fault = r_raised.fx.core.safety.fault().expect("ENABLE_FAILED fault expected");
    assert_eq!(raised_fault.code, armd::FaultCode::EnableFailed);
    assert!(!r_raised.fx.core.safety.recoverable(), "ENABLE_FAILED must be non-recoverable");
    assert!(raised_fault.message.contains("J4:unk/queued"), "missing initial per-joint status: {}", raised_fault.message);
    r_raised.bus.with(|s| s.push_all_feedback());
    r_raised.step();
    let live_fault = r_raised.fx.core.current_observation().fault.expect("live observation fault expected");
    assert!(live_fault.message.contains("J4:unk/held"), "missing live post-hold per-joint status: {}", live_fault.message);

    // 3. Rest pose with non-confirming motor: STILL NO automatic rollback (0 disables), enters
    //    FailedHold + non-recoverable `ENABLE_FAILED`; a later explicit TorqueOff at rest
    //    passes the rest gate and disables torque cleanly.
    let mut r2 = rig();
    r2.bus.with(|s| s.motors[3].drop_torque_frames = 10);
    r2.bus.clear_log();
    r2.submit(TransitionOp::TorqueOn);
    assert!(r2.step().command.unwrap().1.is_ok());
    let mut rest_outcome = None;
    for _ in 0..30 {
        r2.bus.with(|s| s.push_all_feedback());
        let rep = r2.step();
        if let Some((_, o)) = rep.transition_ended {
            rest_outcome = Some(o);
            break;
        }
    }
    assert_eq!(rest_outcome, Some(Outcome::Failed("ENABLE_FAILED")));
    assert_eq!(
        r2.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "failed enable at rest must NOT automatically roll back"
    );
    let inner2 = &r2.fx.core;
    assert!(!inner2.torque_transition_pending());
    assert_eq!(inner2.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
    assert!(inner2.is_torqued && !inner2.disable_pending);
    assert_eq!(inner2.safety.fault().map(|f| f.code), Some(armd::FaultCode::EnableFailed));
    assert!(!inner2.safety.recoverable());

    // Explicit TorqueOff at the rest pose succeeds through the rest gate (K=3 rest-pose ticks)!
    r2.bus.with(|s| s.motors[3].drop_torque_frames = 0);
    for _ in 0..3 {
        r2.bus.with(|s| s.push_all_feedback());
        r2.step();
    }
    r2.submit(TransitionOp::TorqueOff);
    assert!(r2.step().command.unwrap().1.is_ok());
    let (_, off_outcome, _) = r2.until_ended(30);
    assert_eq!(off_outcome, Outcome::Completed);
    assert!(!r2.fx.core.is_torqued);
    assert_eq!(r2.fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::Disarmed);
}

#[test]
fn a_disable_waits_for_fresh_disabled_feedback_and_holds_no_output_meanwhile() {
    let mut r = rig();
    r.submit(TransitionOp::TorqueOn);
    r.step();
    assert_eq!(r.until_ended(20).1, Outcome::Completed);
    r.replies();
    r.bus.with(|s| s.motors[1].torque_reply_delay = Duration::from_millis(100));
    r.submit(TransitionOp::TorqueOff);
    let first = r.step();
    assert!(first.command.unwrap().1.is_ok(), "{first:?}");
    assert!(r.fx.core.disable_pending && r.fx.core.is_torqued, "pending until confirmed: {first:?} {} {} {}", r.fx.core.disable_pending, r.fx.core.is_torqued, r.fx.core.torque_transition_pending());
    r.bus.clear_log();
    let (_, outcome, reports) = r.until_ended(20);
    assert_eq!(outcome, Outcome::Completed);
    // no target frame while the transition owned the bus (only disable retries / polls)
    let targets = r.bus.tx_log().iter().filter(|f| (1..=7).contains(&f.id) && f.data != damiao::CMD_DISABLE).count();
    assert_eq!(targets, 0, "{:?}", r.bus.tx_log());
    let inner = &r.fx.core;
    assert!(!inner.is_torqued && !inner.disable_pending && !inner.pending_rearm && !inner.torque_transition_pending());
    assert_eq!(*inner.safety.state(), armd::SafetyState::Idle);
    assert!(reports.iter().map(|p| p.wakes).sum::<u32>() > 0, "the confirmation was served between releases");
}



/// The torque-related frames of a log: register writes / reads (0x7FF) and enable / disable.
fn handshake_frames(bus: &FakeBus) -> Vec<(u32, [u8; 8])> {
    bus.tx_log().iter().filter(|f| f.id == 0x7FF || f.data == damiao::CMD_ENABLE || f.data == damiao::CMD_DISABLE).map(|f| (f.id, f.data)).collect()
}

#[test]
fn the_rt_enable_sends_the_j7_position_force_handshake_like_the_blocking_enable() {
    let is_write = |f: &(u32, [u8; 8])| f.0 == 0x7FF && f.1[2] == damiao::REG_WRITE && f.1[3] == damiao::RID_CTRL_MODE;
    let is_read = |f: &(u32, [u8; 8])| f.0 == 0x7FF && f.1[2] == damiao::REG_READ && f.1[3] == damiao::RID_CTRL_MODE;
    // the blocking path (legacy set_torque) as the reference
    let mut blocking = rig_with(pos_force);
    blocking.bus.clear_log();
    { let __s = blocking.fx.core.current_admission_stamp(); blocking.fx.core.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("blocking enable");
    let reference = handshake_frames(&blocking.bus);
    // the RT transition path
    let mut r = rig_with(pos_force);
    r.bus.clear_log();
    r.submit(TransitionOp::TorqueOn);
    assert!(r.step().command.unwrap().1.is_ok());
    assert_eq!(r.until_ended(20).1, Outcome::Completed);
    let frames = handshake_frames(&r.bus);
    assert_eq!(frames, reference, "same handshake frames, same order");
    let write = frames.iter().position(is_write).expect("mode-4 write");
    let read = frames.iter().position(is_read).expect("mode readback");
    let enable = frames.iter().position(|f| f.1 == damiao::CMD_ENABLE).expect("enable frame");
    assert!(write < read && read < enable, "write {write} < read {read} < enable {enable}: {frames:?}");
    assert_eq!(frames[write].1[4..8], damiao::CTRL_MODE_POS_FORCE.to_le_bytes());
    assert_eq!(r.bus.with(|s| s.motors[6].ctrl_mode()), damiao::CTRL_MODE_POS_FORCE);
    // the cap is visible through the supervisor's (joint-zero wrapped) driver
    let cap = r.fx.core.driver.as_ref().unwrap().lock().unwrap().motor_current_cap().expect("current cap");
    assert!(cap.active && cap.current_raw == Some(220), "{cap:?}");
    assert!(r.fx.core.is_torqued);
}
