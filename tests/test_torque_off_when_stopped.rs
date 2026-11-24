//! Torque-off while stopped (E-stop / Fault): Rule A′ integration tests on both owners
//! (legacy SupervisorInner / ArmSupervisor / WS / REST and RT executor ReleaseFixture).

mod common;
mod mit_support;

use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, DamiaoArmDriver, EnableState};
use armd::model::{ArmConfig, DEFAULT_REST_POSE_TOLERANCE_RAD};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    Authority, BootId, Broker, Capacities, ClientId, ImmediateOp, Outcome, Reply, Submit,
    TransitionOp,
};
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture};
use armd::safety::{FaultCode, FaultReason, SafetyError, SafetyState, StopPolicy};
use armd::supervisor::{ArmError, StoppedTorqueOffRefusal};
use armd::transport::MockClock;
use armd::{ArmSupervisor, SupervisorInner};
use mit_support::FakeBus;
use serde_json::json;

const BOOT: BootId = BootId(42);
const RELEASE: Duration = Duration::from_millis(20);

fn limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn make_driver_rig() -> (SupervisorInner, FakeBus, MockClock) {
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits(), clock.clone());
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits())
        .unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(d));
    inner.lease_enabled = false;
    inner.tick(0.02);
    (inner, bus, clock)
}

struct ExecRig {
    fx: ReleaseFixture<SupervisorInner>,
    b: Broker<PlanSlot>,
    bus: FakeBus,
    clock: MockClock,
    seq: u64,
}

impl ExecRig {
    fn new() -> Self {
        let (inner, bus, clock) = make_driver_rig();
        let (fx, b, _tw, _stop) = ReleaseFixture::new(
            inner,
            BOOT,
            Capacities {
                commands: 8,
                plans: 1,
                replies: 16,
            },
        );
        Self {
            fx,
            b,
            bus,
            clock,
            seq: 0,
        }
    }

    fn auth(&self) -> Authority {
        self.fx.core.authority(&self.fx.ctx(self.fx.now()))
    }

    fn submit_transition(&mut self, op: TransitionOp) {
        self.seq += 1;
        let a = self.auth();
        self.b
            .submit(
                ClientId(1),
                self.seq,
                a,
                self.fx.now() + Duration::from_secs(5),
                Submit::Transition(op),
            )
            .unwrap();
    }

    fn step(&mut self) -> armd::rt_core::ops::ReleaseReport {
        let r = self.fx.step();
        self.clock.advance(RELEASE);
        r
    }

    fn step_with_feedback(&mut self) -> armd::rt_core::ops::ReleaseReport {
        self.bus.with(|s| s.push_all_feedback());
        self.step()
    }

    fn run_transition_to_end(&mut self, max: usize) -> Outcome {
        for _ in 0..max {
            let r = self.step_with_feedback();
            if let Some((_, o)) = r.transition_ended {
                return o;
            }
        }
        panic!("transition did not finish in {max} releases");
    }

    fn replies(&mut self) -> Vec<Reply> {
        let mut out = Vec::new();
        self.b.poll(|_, r| {
            out.push(r);
            true
        });
        out
    }

    fn arm_now(&mut self) {
        self.bus.with(|s| s.push_all_feedback());
        self.submit_transition(TransitionOp::TorqueOn);
        assert!(self.step().command.unwrap().1.is_ok());
        assert_eq!(self.run_transition_to_end(20), Outcome::Completed);
        self.replies();
        assert!(self.fx.core.is_torqued);
    }
}

#[test]
fn rest_pose_tolerance_rad_profile_field_defaults_customizes_and_validates() {
    assert!((ArmConfig::default().rest_pose_tolerance_rad - 0.05).abs() < 1e-6);
    assert!((ArmConfig::rebot_dm().rest_pose_tolerance_rad - DEFAULT_REST_POSE_TOLERANCE_RAD).abs() < 1e-6);

    // Missing field in JSON deserializes to default 0.05 rad.
    let mut json = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    json.as_object_mut().unwrap().remove("rest_pose_tolerance_rad");
    let parsed: ArmConfig = serde_json::from_value(json).unwrap();
    assert!((parsed.rest_pose_tolerance_rad - 0.05).abs() < 1e-6);

    // Custom value round-trips and validates.
    let mut custom = ArmConfig::rebot_dm();
    custom.rest_pose_tolerance_rad = 0.12;
    custom.validate().expect("positive finite tolerance must validate");
    let schema = armd::model::ObservationSchema::from_config(&custom);
    assert_eq!(schema.rest_pose_tolerance_rad, Some(0.12));

    // Invalid values are rejected by ArmConfig::validate().
    for bad in [0.0, -0.05, f32::NAN, f32::INFINITY] {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.rest_pose_tolerance_rad = bad;
        assert!(
            cfg.validate().is_err(),
            "expected validate() to reject rest_pose_tolerance_rad={bad}"
        );
    }

    // Custom tolerance and custom resting_pose.positions are enforced by the decision function.
    let mut cfg = ArmConfig::rebot_dm();
    cfg.rest_pose_tolerance_rad = 0.10;
    let mut rest_pos = vec![0.0; 7];
    rest_pos[1] = -0.50;
    cfg.resting_pose.as_mut().unwrap().positions = Some(rest_pos);
    let mut inner = SupervisorInner::new(cfg);
    inner.set_torque(true).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("test".into()));

    // At 0.0 (home, 0.50 rad away from custom rest -0.50) -> refused naming joint_2 with rest -0.50.
    assert!(matches!(
        inner.set_torque(false),
        Err(SafetyError::TorqueOffWouldDrop {
            ref joint,
            position: Some(p),
            rest: Some(r),
            ..
        }) if joint.as_deref() == Some("joint_2") && p.abs() < 1e-4 && (r + 0.50).abs() < 1e-4
    ));

    // At -0.42 (0.08 rad from -0.50, within custom 0.10 rad tolerance) -> allowed, latch kept.
    inner.measured_positions[1] = -0.42;
    assert!(matches!(inner.set_torque(false), Ok(SafetyState::Fault(_))));
    assert!(!inner.is_torqued);
}

#[test]
fn legacy_fault_at_rest_allows_disable_keeps_latch_and_ignores_gripper() {
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();

    // Gripper wide open (-3.0 rad), all arm joints within 0.05 rad of rest (0.0).
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        for j in 0..6 {
            s.motors[j].q = 0.03;
        }
        s.motors[6].q = -3.0;
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!((inner.measured_positions[6] + 3.0).abs() < 0.01, "gripper must be measured open");
    assert!((inner.measured_positions[1] - 0.03).abs() < 0.01, "J2 must be measured at 0.03 rad");

    inner.safety.trigger_fault_coded(
        FaultCode::DriverWriteFailed,
        FaultReason::Other("simulated fault".into()),
    );
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert!(inner.is_torqued);
    let epoch_before = inner.safety.epoch_id();
    let kinds_before = inner.safety.latched_fault_kinds();
    let fault_before = inner.safety.fault();

    let state = inner.set_torque(false).expect("disable at rest in Fault must succeed");
    assert!(matches!(state, SafetyState::Fault(_)), "Fault latch must stay active");
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert_eq!(inner.safety.epoch_id(), epoch_before, "torque-off must not advance epoch");
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before, "torque-off must not mutate fault record");
    assert_eq!(inner.safety.fault(), fault_before, "torque-off must not mutate fault detail");
    assert!(!inner.is_torqued, "is_torqued must become false");
    assert!(!inner.disable_pending, "disable must be confirmed");
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)), "all motors disabled on wire");

    // Later exact-epoch clear_fault follows the normal rules and transitions to Idle with torque off.
    clock.advance(Duration::from_millis(10));
    bus.with(|s| s.push_all_feedback());
    let cleared = inner
        .clear_fault_for_epoch(Some(epoch_before))
        .expect("exact-epoch clear_fault after torque-off must succeed");
    assert_eq!(cleared, SafetyState::Idle);
    assert!(!inner.is_torqued);
}

#[test]
fn executor_fault_at_rest_allows_disable_keeps_latch_and_ignores_gripper() {
    let mut r = ExecRig::new();
    r.arm_now();

    // Gripper wide open (-3.0 rad), arm joints within 0.05 rad of rest.
    r.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        for j in 0..6 {
            s.motors[j].q = -0.04;
        }
        s.motors[6].q = -3.0;
        s.push_all_feedback();
    });
    r.step();
    assert!((r.fx.core.measured_positions[6] + 3.0).abs() < 0.01, "gripper must be measured open");
    assert!((r.fx.core.measured_positions[1] + 0.04).abs() < 0.01, "J2 must be measured at -0.04 rad");

    r.fx.core.safety.trigger_fault_coded(
        FaultCode::DriverWriteFailed,
        FaultReason::Other("simulated executor fault".into()),
    );
    assert!(matches!(r.fx.core.safety.state(), SafetyState::Fault(_)));
    assert!(r.fx.core.is_torqued);
    let epoch_before = r.fx.core.safety.epoch_id();
    let kinds_before = r.fx.core.safety.latched_fault_kinds();
    let fault_before = r.fx.core.safety.fault();

    r.bus.with(|s| s.push_all_feedback());
    r.submit_transition(TransitionOp::TorqueOff);
    let first = r.step();
    assert!(first.command.unwrap().1.is_ok(), "commit must accept at-rest disable in Fault: {first:?}");
    let outcome = r.run_transition_to_end(20);
    assert_eq!(outcome, Outcome::Completed);

    assert!(matches!(r.fx.core.safety.state(), SafetyState::Fault(_)), "Fault latch must stay active");
    assert_eq!(r.fx.core.safety.epoch_id(), epoch_before, "torque-off must not advance epoch");
    assert_eq!(r.fx.core.safety.latched_fault_kinds(), kinds_before, "torque-off must not mutate fault record");
    assert_eq!(r.fx.core.safety.fault(), fault_before, "torque-off must not mutate fault detail");
    assert!(!r.fx.core.is_torqued);
    assert!(!r.fx.core.disable_pending);
    assert!(r.bus.with(|s| s.motors.iter().all(|m| !m.enabled)));

    // Later clear_fault_executor follows normal rules.
    r.step_with_feedback();
    let cleared = r
        .fx
        .core
        .clear_fault_executor(Instant::now() + Duration::from_secs(1))
        .expect("clear_fault_executor must succeed");
    assert_eq!(cleared, SafetyState::Idle);
    assert!(!r.fx.core.is_torqued);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_raised_in_fault_refuses_with_torque_off_would_drop_and_no_force_override() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 60_000;
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let (mut ws, generation, _, sup, addr) = common::torqued_rig(sup, &[0.0, 0.35, -0.80, 0.0, 0.0, 0.0, 0.0]).await;

    // Trigger Fault while raised (J2 at 0.35, J3 at -0.80 -> first offending joint is joint_2).
    {
        let mut inner = sup.inner().write().await;
        inner.safety.trigger_fault_coded(
            FaultCode::DriverWriteFailed,
            FaultReason::Other("raised fault".into()),
        );
    }
    let epoch = sup.get_observation().await.epoch_id;

    // 1. WS /api/v1/torque {enable: false} -> TORQUE_OFF_WOULD_DROP naming joint_2
    let rej = common::rpc(&mut ws, generation, epoch, "/api/v1/torque", json!({"enable": false})).await;
    assert_eq!(rej["type"], "command_reject", "{rej}");
    assert_eq!(rej["status"], 409, "{rej}");
    assert_eq!(rej["code"], "TORQUE_OFF_WOULD_DROP", "{rej}");
    assert_eq!(rej["joint"], "joint_2", "{rej}");
    let msg = rej["message"].as_str().unwrap_or("");
    assert!(msg.contains("joint_2"), "message must name first offending joint: {msg}");
    assert!(msg.contains("0.350"), "message must include position: {msg}");
    assert!(msg.contains("0.000"), "message must include rest value: {msg}");
    assert!(msg.to_lowercase().contains("would drop"), "message must warn arm would drop: {msg}");
    assert!(sup.get_observation().await.is_torqued);

    // 2. Unknown `force: true` field on /api/v1/torque is STILL refused (no override).
    let rej_force = common::rpc(
        &mut ws,
        generation,
        epoch,
        "/api/v1/torque",
        json!({"enable": false, "force": true}),
    )
    .await;
    assert_eq!(rej_force["type"], "command_reject", "{rej_force}");
    assert_eq!(rej_force["code"], "TORQUE_OFF_WOULD_DROP", "{rej_force}");
    assert!(sup.get_observation().await.is_torqued);

    // 3. WS /api/v1/arm alias with `force: true` is ALSO refused via the same check.
    let rej_arm = common::rpc(
        &mut ws,
        generation,
        epoch,
        "/api/v1/arm",
        json!({"enable": false, "force": true}),
    )
    .await;
    assert_eq!(rej_arm["type"], "command_reject", "{rej_arm}");
    assert_eq!(rej_arm["code"], "TORQUE_OFF_WOULD_DROP", "{rej_arm}");
    assert!(sup.get_observation().await.is_torqued);

    // 4. REST /api/v1/torque stays lease-gated (never bypasses WS lease or rule 2).
    let rest_resp = reqwest::Client::new()
        .post(format!("http://{addr}/api/v1/torque"))
        .json(&json!({"enable": false, "force": true}))
        .send()
        .await
        .unwrap();
    assert_eq!(rest_resp.status(), reqwest::StatusCode::CONFLICT);
    let rest_body: serde_json::Value = rest_resp.json().await.unwrap();
    assert_eq!(rest_body["code"], "LEASE_ACTIVE", "{rest_body}");
    assert!(sup.get_observation().await.is_torqued);

    // 5. SDK-facing ArmSupervisor::set_torque(false) returns SafetyError::TorqueOffWouldDrop.
    let sdk_err = sup.set_torque(false).await.unwrap_err();
    match sdk_err {
        ArmError::Safety(SafetyError::TorqueOffWouldDrop {
            joint,
            position,
            rest,
            reason,
            message,
        }) => {
            assert_eq!(joint.as_deref(), Some("joint_2"));
            assert_eq!(reason.as_deref(), Some("outside_rest_pose"));
            assert!((position.unwrap() - 0.35).abs() < 1e-4);
            assert!(rest.unwrap().abs() < 1e-4);
            assert!(message.to_lowercase().contains("would drop"));
        }
        other => panic!("expected TorqueOffWouldDrop, got {other:?}"),
    }
    assert!(sup.get_observation().await.is_torqued);
}

#[test]
fn executor_raised_in_fault_refuses_with_torque_off_would_drop_both_transition_and_immediate() {
    // 1. Driver-backed executor: TransitionOp::TorqueOff
    let mut r = ExecRig::new();
    r.arm_now();
    r.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[2].q = -0.60; // J3 raised
        s.push_all_feedback();
    });
    r.step();
    assert!(
        (r.fx.core.measured_positions[2] + 0.60).abs() < 0.01,
        "J3 must be measured at raised pose before Fault, got {:?}",
        r.fx.core.measured_positions
    );
    r.fx.core.safety.trigger_fault_coded(
        FaultCode::DriverWriteFailed,
        FaultReason::Other("raised fault".into()),
    );
    r.bus.clear_log();

    r.bus.with(|s| s.push_all_feedback());
    r.submit_transition(TransitionOp::TorqueOff);
    let rep = r.step();
    assert_eq!(rep.command.unwrap().1, Err("TORQUE_OFF_WOULD_DROP"));
    assert!(
        matches!(
            r.fx.core.last_stopped_torque_off_refusal(),
            Some(StoppedTorqueOffRefusal::JointOutsideRest { joint_index: 2, .. })
        ),
        "executor refusal must preserve bounded first-joint detail: {:?}",
        r.fx.core.last_stopped_torque_off_refusal()
    );
    assert!(r.fx.core.is_torqued);
    assert!(!r.fx.core.disable_pending);
    assert_eq!(
        r.bus
            .tx_log()
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE)
            .count(),
        0,
        "no disable frame may be sent when refused"
    );
    let replies = r.replies();
    assert!(
        matches!(replies.last(), Some(Reply::Refused { code: "TORQUE_OFF_WOULD_DROP", .. })),
        "{replies:?}"
    );

    // 2. Driverless executor: ImmediateOp::TorqueOff
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = false;
    inner.set_torque(true).unwrap();
    inner.measured_positions[1] = 0.25;
    inner.safety.trigger_fault_coded(
        FaultCode::WatchdogTimeout,
        FaultReason::WatchdogTimeout,
    );
    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
        inner,
        BOOT,
        Capacities {
            commands: 4,
            plans: 1,
            replies: 8,
        },
    );
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(
        ClientId(1),
        1,
        auth,
        fx.now() + Duration::from_secs(5),
        Submit::Immediate(ImmediateOp::TorqueOff),
    )
    .unwrap();
    let rep = fx.step();
    assert_eq!(rep.command.unwrap().1, Err("TORQUE_OFF_WOULD_DROP"));
    assert!(fx.core.is_torqued);

    // Move to rest -> ImmediateOp::TorqueOff succeeds and keeps Fault latch.
    fx.core.measured_positions = vec![0.0; 7];
    let auth2 = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(
        ClientId(1),
        2,
        auth2,
        fx.now() + Duration::from_secs(5),
        Submit::Immediate(ImmediateOp::TorqueOff),
    )
    .unwrap();
    let rep2 = fx.step();
    assert!(rep2.command.unwrap().1.is_ok(), "{rep2:?}");
    assert!(!fx.core.is_torqued);
    assert!(matches!(fx.core.safety.state(), SafetyState::Fault(_)));
}

#[test]
fn stale_and_missing_feedback_refused_as_not_at_rest_on_both_owners() {
    // 1. Legacy owner with real DamiaoArmDriver + MockClock
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();
    // Arm is physically at rest (0.0), enter Fault.
    inner.safety.trigger_fault_coded(
        FaultCode::BusLoss,
        FaultReason::BusLoss,
    );
    // Advance driver clock beyond 100 ms without new feedback.
    clock.advance(Duration::from_millis(101));
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback)
    );
    let err = inner.set_torque(false).unwrap_err();
    assert!(matches!(err, SafetyError::TorqueOffWouldDrop { joint: None, .. }), "{err:?}");
    assert!(inner.is_torqued);

    // Once fresh feedback arrives after an observed-empty queue boundary (<= 100 ms),
    // disable succeeds and keeps Fault latch.
    inner.tick(0.02); // observe empty RX queue at current time
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    assert!(matches!(inner.set_torque(false), Ok(SafetyState::Fault(_))));
    assert!(!inner.is_torqued);

    // 2. Executor owner with stale feedback (> 100 ms)
    let mut r = ExecRig::new();
    r.arm_now();
    r.fx.core.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    r.bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
        s.delayed.clear();
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    // Advance > 100 ms with no replies.
    r.clock.advance(Duration::from_millis(120));
    r.submit_transition(TransitionOp::TorqueOff);
    let rep = r.step();
    assert!(!r.fx.core.hardware_feedback_fresh(Duration::from_millis(100)));
    assert_eq!(rep.command.unwrap().1, Err("TORQUE_OFF_WOULD_DROP"));
    assert_eq!(
        r.fx.core.last_stopped_torque_off_refusal(),
        Some(StoppedTorqueOffRefusal::StaleOrMissingFeedback)
    );
    assert!(r.fx.core.is_torqued);
}

#[test]
fn estop_same_rules_and_disable_exempt_from_stop_gate_on_both_owners() {
    // 1. Legacy owner in E-Stop
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();

    // Raised -> trigger E-stop -> refused with TORQUE_OFF_WOULD_DROP
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[0].q = 0.20;
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!((inner.measured_positions[0] - 0.20).abs() < 0.01);
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert!(inner.e_stop_latched.load(Ordering::SeqCst));
    let epoch_before = inner.safety.epoch_id();
    let kinds_before = inner.safety.latched_fault_kinds();
    let fault_before = inner.safety.fault();
    assert!(matches!(
        inner.set_torque(false),
        Err(SafetyError::TorqueOffWouldDrop { ref joint, .. }) if joint.as_deref() == Some("joint_1")
    ));
    assert!(inner.is_torqued);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(inner.safety.fault(), fault_before);

    // At rest + fresh feedback in pure E-Stop:
    // First inject a disable TX failure while in EStop: sticky disable stays pending and
    // SafetyState::EStop, fault record, and epoch must NOT be mutated.
    bus.with(|s| {
        s.motors[0].q = 0.0;
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    bus.with(|s| s.tx_error = true);
    assert!(inner.set_torque(false).is_err());
    assert_eq!(*inner.safety.state(), SafetyState::EStop, "failed disable must not overwrite E-Stop latch");
    assert_eq!(inner.safety.epoch_id(), epoch_before, "failed disable must not advance epoch");
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before, "failed disable must not mutate fault record");
    assert_eq!(inner.safety.fault(), fault_before, "failed disable must not mutate fault detail");
    assert!(inner.disable_pending, "sticky disable_pending must stay set on failure");

    // At rest -> stale feedback -> refused without mutating EStop latch, fault record, or epoch.
    bus.with(|s| s.tx_error = false);
    clock.advance(Duration::from_millis(150));
    assert!(matches!(
        inner.set_torque(false),
        Err(SafetyError::TorqueOffWouldDrop { joint: None, .. })
    ));
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(inner.safety.fault(), fault_before);

    // Refresh driver RX queue bound without triggering a stale hold-output transmit, then confirm
    // TxKind::Disable bypasses stop gate and keeps the exact EStop latch and epoch.
    {
        let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
        let _ = drv.read_joint_states(); // observe empty RX queue after 150 ms stall
    }
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    let st = inner.set_torque(false).expect("disable at rest in E-Stop must succeed");
    assert_eq!(st, SafetyState::EStop);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before, "confirmed disable must not advance epoch");
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before, "confirmed disable must not mutate fault record");
    assert_eq!(inner.safety.fault(), fault_before, "confirmed disable must not mutate fault detail");
    assert!(inner.e_stop_latched.load(Ordering::SeqCst), "e_stop_latched must remain set");
    assert!(!inner.is_torqued);
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));

    // Exact-epoch clear still works after stopped torque-off (confirming epoch did not advance).
    inner.e_stop_latched.store(false, Ordering::SeqCst);
    bus.with(|s| s.push_all_feedback());
    assert_eq!(
        inner.clear_fault_for_epoch(Some(epoch_before)).expect("exact-epoch clear after E-Stop disable must succeed"),
        SafetyState::Idle
    );

    // 2. Executor owner in E-Stop
    let mut r = ExecRig::new();
    r.arm_now();
    r.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[1].q = 0.30;
        s.push_all_feedback();
    });
    r.step();
    assert!(
        (r.fx.core.measured_positions[1] - 0.30).abs() < 0.01,
        "J2 must be measured at raised pose before E-Stop, got {:?}",
        r.fx.core.measured_positions
    );
    r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
    assert_eq!(*r.fx.core.safety.state(), SafetyState::EStop);
    assert!(r.fx.core.e_stop_latched.load(Ordering::SeqCst));
    let exec_epoch_before = r.fx.core.safety.epoch_id();
    let exec_kinds_before = r.fx.core.safety.latched_fault_kinds();
    let exec_fault_before = r.fx.core.safety.fault();

    // Raised in E-Stop -> refused
    r.bus.with(|s| s.push_all_feedback());
    r.submit_transition(TransitionOp::TorqueOff);
    assert_eq!(r.step().command.unwrap().1, Err("TORQUE_OFF_WOULD_DROP"));
    assert!(r.fx.core.is_torqued);
    assert_eq!(r.fx.core.safety.epoch_id(), exec_epoch_before);
    assert_eq!(r.fx.core.safety.latched_fault_kinds(), exec_kinds_before);
    assert_eq!(r.fx.core.safety.fault(), exec_fault_before);

    // Move to rest with fresh feedback -> allowed; EStop latch, fault record, and epoch kept.
    r.bus.with(|s| {
        s.motors[1].q = 0.0;
        s.push_all_feedback();
    });
    r.step();
    r.bus.with(|s| s.push_all_feedback());
    r.submit_transition(TransitionOp::TorqueOff);
    assert!(r.step().command.unwrap().1.is_ok());
    assert_eq!(r.run_transition_to_end(20), Outcome::Completed);
    assert_eq!(*r.fx.core.safety.state(), SafetyState::EStop);
    assert_eq!(r.fx.core.safety.epoch_id(), exec_epoch_before);
    assert_eq!(r.fx.core.safety.latched_fault_kinds(), exec_kinds_before);
    assert_eq!(r.fx.core.safety.fault(), exec_fault_before);
    assert!(r.fx.core.e_stop_latched.load(Ordering::SeqCst));
    assert!(!r.fx.core.is_torqued);
    assert!(r.bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

#[test]
fn relatching_deferral_fault_loop_allows_disable_at_rest_on_both_owners() {
    // Simulate the 2026-11-24 TX deferral loop: each transmit takes 2 ms on the driver clock,
    // so a 7-motor batch exceeds DAMIAO_TX_MAX_DURATION (5 ms) and defers every tick.
    // TxKind::Disable is exempt from the TX budget and stop gate, so disable at rest must
    // still succeed on both owners even when faults re-latch every tick.

    // 1. Legacy owner
    let (mut inner, bus, _clock) = make_driver_rig();
    inner.set_torque(true).unwrap();
    bus.with(|s| s.tx_step = Duration::from_millis(2));

    // Run ticks at rest: write_joint_targets defers -> latches DriverWriteFailed; subsequent
    // ticks keep failing write_estop_hold / re-latching fault.
    for _ in 0..3 {
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        // Simulate a fault that re-latches on every tick.
        inner.safety.trigger_fault_coded(
            FaultCode::DriverWriteFailed,
            FaultReason::Other("TX deferral re-latch".into()),
        );
    }
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert!(inner.is_torqued);

    // Disable at rest succeeds despite the deferral cost (TxKind::Disable exempt from TX budget).
    bus.with(|s| s.push_all_feedback());
    let state = inner.set_torque(false).expect("disable at rest must beat deferral fault loop");
    assert!(matches!(state, SafetyState::Fault(_)));
    assert!(!inner.is_torqued);
    assert!(!inner.disable_pending);
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));

    // Verify hold output stops after torque is off (no more target frames on subsequent ticks).
    bus.clear_log();
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    let post_targets = bus
        .tx_log()
        .iter()
        .filter(|f| (1..=7).contains(&f.id) && f.data != damiao::CMD_DISABLE)
        .count();
    assert_eq!(post_targets, 0, "no hold frames after torque-off");
    assert_eq!(
        inner.driver.as_ref().unwrap().lock().unwrap().enable_state(),
        EnableState::Disabled
    );

    // 2. Executor owner
    let mut r = ExecRig::new();
    r.arm_now();
    r.bus.with(|s| s.tx_step = Duration::from_millis(2));
    // Re-latch fault on every tick before disabling, and keep re-latching while disable confirms.
    for _ in 0..3 {
        r.step_with_feedback();
        r.fx.core.safety.trigger_fault_coded(
            FaultCode::DriverWriteFailed,
            FaultReason::Other("executor TX deferral re-latch".into()),
        );
    }
    assert!(matches!(r.fx.core.safety.state(), SafetyState::Fault(_)));
    assert!(r.fx.core.is_torqued);

    // Delay one motor's disable reply so confirmation spans multiple releases while faults re-latch.
    r.bus.with(|s| {
        s.motors[2].torque_reply_delay = Duration::from_millis(60);
        s.push_all_feedback();
    });
    r.submit_transition(TransitionOp::TorqueOff);
    let first = r.step();
    assert!(first.command.unwrap().1.is_ok(), "{first:?}");
    let mut ended = None;
    for _ in 0..25 {
        // Re-latch fault on every release while disable is confirming!
        r.fx.core.safety.trigger_fault_coded(
            FaultCode::DriverWriteFailed,
            FaultReason::Other("executor TX deferral re-latch mid-confirm".into()),
        );
        let rep = r.step_with_feedback();
        if let Some((_, o)) = rep.transition_ended {
            ended = Some(o);
            break;
        }
    }
    assert_eq!(ended, Some(Outcome::Completed));
    assert!(matches!(r.fx.core.safety.state(), SafetyState::Fault(_)));
    assert!(!r.fx.core.is_torqued);
    assert!(!r.fx.core.disable_pending);
    assert!(r.bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

#[test]
fn cockpit_html_exposes_stopped_torque_off_inline_message_without_override() {
    let html = include_str!("../src/api/index.html");
    assert!(html.contains("id=\"btn-torque-off\""), "stopped torque-off button missing");
    assert!(html.contains("id=\"torque-off-msg\""), "inline TORQUE_OFF_WOULD_DROP container missing");
    assert!(
        html.contains("Torque stays on: {joint} is at {pos} rad (rest {rest}); the arm would drop. Clear the fault and park, or cut power."),
        "exact required inline refusal template missing"
    );
    assert!(html.contains("TORQUE_OFF_WOULD_DROP"), "TORQUE_OFF_WOULD_DROP handler missing");
    // Verify no `/api/v1/torque` call passes a `force` override and no confirm-anyway button exists.
    for line in html.lines().filter(|l| l.contains("/api/v1/torque")) {
        assert!(!line.contains("force"), "torque call must not pass force override: {line}");
    }
    assert!(!html.contains("confirm-anyway"), "cockpit must not expose a confirm-anyway override");
    assert!(html.contains("no_resting_pose"), "cockpit must localize uncalibrated resting_pose refusal");
    assert!(html.contains("stale_feedback"), "cockpit must localize stale_feedback refusal");
}

#[test]
fn hardware_without_calibrated_resting_pose_refuses_stopped_torque_off_on_both_owners() {
    // Hardware with resting_pose: None must NOT trust home_positions for stopped torque-off.
    let make_uncalibrated = || {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        bus.with(|s| s.push_all_feedback());
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        let mut cfg = ArmConfig::rebot_dm();
        cfg.lease_timeout_ms = 600_000;
        cfg.resting_pose = None;
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(d));
        inner.lease_enabled = false;
        inner.tick(0.02);
        (inner, bus, clock)
    };

    // 1. Legacy owner
    let (mut inner, _bus, _clock) = make_uncalibrated();
    inner.set_torque(true).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::NoRestingPoseConfigured)
    );
    assert!(matches!(
        inner.set_torque(false),
        Err(SafetyError::TorqueOffWouldDrop {
            joint: None,
            ref reason,
            ..
        }) if reason.as_deref() == Some("no_resting_pose")
    ));
    assert!(inner.is_torqued);

    // 2. Executor owner
    let (inner, bus, clock) = make_uncalibrated();
    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
        inner,
        BOOT,
        Capacities { commands: 4, plans: 1, replies: 8 },
    );
    bus.with(|s| s.push_all_feedback());
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(ClientId(1), 1, auth, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    for _ in 0..20 {
        bus.with(|s| s.push_all_feedback());
        let rep = fx.step();
        clock.advance(RELEASE);
        if rep.transition_ended.is_some() { break; }
    }
    assert!(fx.core.is_torqued);
    fx.core.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));
    bus.with(|s| s.push_all_feedback());
    let auth2 = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(ClientId(1), 2, auth2, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOff)).unwrap();
    let rep = fx.step();
    assert_eq!(rep.command.unwrap().1, Err("TORQUE_OFF_WOULD_DROP"));
    assert_eq!(
        fx.core.last_stopped_torque_off_refusal(),
        Some(StoppedTorqueOffRefusal::NoRestingPoseConfigured)
    );
    assert!(fx.core.is_torqued);
}

#[test]
fn silent_gripper_does_not_block_arm_joint_rest_admission_and_driverless_missing_feedback_refuses() {
    // 1. Silent gripper (J7 stale > 150 ms) while J1..J6 are fresh (<= 100 ms) at rest:
    // rest-pose admission must PASS because gripper is excluded from rule 1.
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[6].silent = true;
    });
    clock.advance(Duration::from_millis(150));
    inner.tick(0.02); // observe empty RX queue at current time
    bus.with(|s| {
        for j in 0..6 {
            s.push_feedback(j);
        }
    });
    inner.tick(0.02);
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Ok(()),
        "silent gripper must not block rest-pose admission when J1..J6 are fresh and at rest"
    );

    // If an arm joint (J3) goes stale (> 100 ms), admission MUST refuse.
    bus.with(|s| s.motors[2].silent = true);
    clock.advance(Duration::from_millis(120));
    bus.with(|s| {
        for j in [0, 1, 3, 4, 5] {
            s.push_feedback(j);
        }
    });
    inner.tick(0.02);
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback)
    );

    // 2. Driverless invalid / missing / stale synthetic feedback refuses.
    let mut mock = SupervisorInner::new(ArmConfig::rebot_dm());
    mock.set_torque(true).unwrap();
    mock.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));

    mock.feedback_numeric_valid = false;
    assert_eq!(mock.may_disable_torque_while_stopped(), Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback));
    mock.feedback_numeric_valid = true;

    mock.measured_positions[0] = f32::NAN;
    assert_eq!(mock.may_disable_torque_while_stopped(), Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback));
    mock.measured_positions[0] = 0.0;

    mock.last_can_feedback_instant = Some(Instant::now() - Duration::from_millis(200));
    assert_eq!(mock.may_disable_torque_while_stopped(), Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback));
}

#[test]
fn queued_old_can_frames_exceeding_queue_bound_are_rejected_as_stale() {
    // Even if old at-rest frames are dequeued at `now` (decode age = 0 ms), the conservative
    // receive-queue bound (`rx_empty_instant` > 100 ms ago) must reject them as stale.
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
    });
    // Advance clock > 100 ms with no empty-queue drain, then enqueue old frames that get decoded now.
    clock.advance(Duration::from_millis(150));
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
        "frames decoded after a >100 ms gap without a fresh empty-queue boundary must be rejected"
    );
    // After the previous tick drained the queue to WouldBlock, rx_empty_instant is now fresh.
    // A new frame arriving now has a fresh queue bound (<= 100 ms) and passes admission.
    clock.advance(Duration::from_millis(20));
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    assert_eq!(inner.may_disable_torque_while_stopped(), Ok(()));
}
