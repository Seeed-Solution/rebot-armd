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
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture, ReleaseReport};
use armd::transport::MockClock;
use armd::{ArmConfig, SupervisorInner};
use mit_support::FakeBus;

const BOOT: BootId = BootId(11);
const RELEASE: Duration = Duration::from_millis(20);

fn limits() -> Vec<DamiaoLimits> { ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect() }

struct Rig {
    fx: ReleaseFixture<SupervisorInner>,
    b: Broker<PlanSlot>,
    bus: FakeBus,
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
    let (fx, b, _tw, _stop) = ReleaseFixture::new(inner, BOOT, Capacities { commands: 4, plans: 1, replies: 8 });
    Rig { fx, b, bus, clock, seq: 0 }
}

impl Rig {
    fn auth(&self) -> Authority { self.fx.core.authority(&self.fx.ctx(self.fx.now())) }

    fn submit(&mut self, op: TransitionOp) {
        self.seq += 1;
        let a = self.auth();
        self.b.submit(ClientId(1), self.seq, a, self.fx.now() + Duration::from_secs(5), Submit::Transition(op)).unwrap();
    }

    /// One release; the driver's clock moves with the executor's.
    fn step(&mut self) -> ReleaseReport {
        let r = self.fx.step();
        self.clock.advance(RELEASE);
        r
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
    let mut r = rig();
    r.bus.with(|s| s.motors[3].silent = true);
    r.submit(TransitionOp::TorqueOn);
    assert!(r.step().command.unwrap().1.is_ok());
    let (_, outcome, _) = r.until_ended(30);
    assert_eq!(outcome, Outcome::Failed("ENABLE_FAILED"));
    let inner = &r.fx.core;
    assert!(!inner.torque_transition_pending());
    assert!(!inner.is_torqued || inner.disable_pending, "torque off or its disable pending ({}, {})", inner.is_torqued, inner.disable_pending);
    assert!(matches!(inner.safety.state(), armd::SafetyState::Fault(_)), "{:?}", inner.safety.state());
    // a second enable while the fault latches is refused at the commit, before any frame
    r.bus.with(|s| s.motors[3].silent = false);
    let before = r.enable_frames();
    r.submit(TransitionOp::TorqueOn);
    assert!(r.step().command.unwrap().1.is_err());
    assert_eq!(r.enable_frames(), before);
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
