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
        s.rx.clear();
        s.pending.clear();
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

    // 2. Driverless invalid / missing / stale synthetic feedback refuses; invalid gripper ignored.
    let mut mock = SupervisorInner::new(ArmConfig::rebot_dm());
    mock.set_torque(true).unwrap();
    mock.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));

    mock.measured_positions[6] = f32::NAN;
    assert_eq!(
        mock.may_disable_torque_while_stopped(),
        Ok(()),
        "invalid gripper position must not block J1..J6 rest-pose admission in driverless mode"
    );
    mock.measured_positions[6] = 0.0;

    mock.feedback_numeric_valid = false;
    assert_eq!(mock.may_disable_torque_while_stopped(), Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback));
    mock.feedback_numeric_valid = true;

    mock.measured_positions[0] = f32::NAN;
    assert_eq!(mock.may_disable_torque_while_stopped(), Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback));
    mock.measured_positions[0] = 0.0;

    mock.last_can_feedback_instant = Some(Instant::now() - Duration::from_millis(200));
    assert_eq!(mock.may_disable_torque_while_stopped(), Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback));

    // 3. Real driver with missing timestamps (before any feedback arrives) MUST refuse as not at rest,
    // even though SupervisorInner::new initializes feedback_numeric_valid=true and measured_positions=rest.
    let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let bus_empty = FakeBus::new(&limits);
    let drv_empty = DamiaoArmDriver::new(bus_empty, vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let mut inner_empty = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_empty));
    inner_empty.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    assert!(inner_empty.feedback_numeric_valid);
    assert_eq!(
        inner_empty.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
        "missing timestamps on a real driver must count as not at rest"
    );

    // 4. Real driver where gripper reports NaN (making whole-arm feedback_numeric_valid false)
    // while J1..J6 are fresh and valid at rest MUST still pass; NaN on J2 MUST refuse.
    struct GripperInvalidDriver {
        states: Vec<armd::model::JointState>,
        instants: Vec<Option<Instant>>,
    }
    impl ArmDriver for GripperInvalidDriver {
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
        fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
            Ok(self.states.clone())
        }
        fn write_joint_targets(&mut self, _: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), armd::driver::ArmDriverError> {
            Ok(())
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            self.instants.iter().map(|t| t.map(|i| i.elapsed())).collect()
        }
        fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
            let caps = self.capabilities();
            for (slot, &i) in out.iter_mut().zip(joints.iter()) {
                let fresh = self.instants.get(i).copied().flatten().is_some_and(|t| t.elapsed() <= max_age);
                let Some(s) = self.states.get(i) else { return false; };
                if !fresh
                    || !s.position.is_finite()
                    || (caps.velocity_feedback && !s.velocity.is_finite())
                    || (caps.torque_feedback && !s.torque.is_finite())
                    || (caps.temperature_feedback && !s.temperature.is_finite())
                {
                    return false;
                }
                *slot = s.position;
            }
            true
        }
    }

    let now = Instant::now();
    let mut bad_grip_states = vec![armd::model::JointState::default(); 7];
    bad_grip_states[6].position = f32::NAN;
    bad_grip_states[6].velocity = f32::NAN;
    let drv_bg = GripperInvalidDriver {
        states: bad_grip_states,
        instants: vec![Some(now); 7],
    };
    let mut inner_bg = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_bg));
    inner_bg.tick(0.02);
    assert!(!inner_bg.feedback_numeric_valid, "whole-arm validity is false when gripper has NaN");
    inner_bg.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));
    assert_eq!(
        inner_bg.may_disable_torque_while_stopped(),
        Ok(()),
        "invalid gripper feedback must not block J1..J6 rest-pose admission"
    );

    let mut bad_j2_states = vec![armd::model::JointState::default(); 7];
    bad_j2_states[1].velocity = f32::NAN;
    let drv_bj2 = GripperInvalidDriver {
        states: bad_j2_states,
        instants: vec![Some(now); 7],
    };
    let mut inner_bj2 = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv_bj2));
    inner_bj2.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));
    assert_eq!(
        inner_bj2.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
        "NaN velocity on arm joint J2 must refuse rest-pose admission"
    );
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

#[test]
fn fresh_driver_timestamps_never_vouch_for_stale_measured_positions_and_zero_offset_applies() {
    // 1. Reproduce the clear_fault / silent-gripper split: supervisor.measured_positions is stale
    // at rest (0.0), while the driver's fresh J1..J6 feedback (read during clear_fault) shows J2
    // raised to 0.45 rad. The rest check MUST use the fresh driver positions and refuse.
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    assert!((inner.measured_positions[1] - 0.0).abs() < 1e-3);

    // Silence the gripper and advance past 250 ms so clear_fault's whole-arm fb_fresh is false.
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[6].silent = true;
    });
    clock.advance(Duration::from_millis(300));
    // Observe empty RX queue so the next J1..J6 frames get a fresh queue bound.
    {
        let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
        let _ = drv.read_joint_states();
    }
    // Lift J2 to 0.45 rad and push fresh J1..J6 feedback only; call clear_fault (fails because
    // gripper is stale, so supervisor.measured_positions stays stale at 0.0 while driver has fresh
    // timestamps and cached_states[1].position = 0.45).
    bus.with(|s| {
        s.motors[1].q = 0.45;
        for j in 0..6 {
            s.push_feedback(j);
        }
    });
    assert!(inner.clear_fault().is_err());
    assert!(
        inner.measured_positions[1].abs() < 1e-3,
        "test precondition: supervisor.measured_positions[1] must still be stale at 0.0, got {}",
        inner.measured_positions[1]
    );

    // Rest check MUST refuse naming joint_2 with the fresh ~0.45 rad driver position.
    let err = inner.set_torque(false).unwrap_err();
    match err {
        SafetyError::TorqueOffWouldDrop {
            joint: Some(ref j),
            position: Some(p),
            ..
        } => {
            assert_eq!(j, "joint_2");
            assert!(
                (p - 0.45).abs() < 0.02,
                "expected fresh driver position ~0.45 rad, got {p}"
            );
        }
        other => panic!("expected TorqueOffWouldDrop naming joint_2 at ~0.45 rad, got {other:?}"),
    }
    assert!(inner.is_torqued);

    // Move J2 back to rest with fresh J1..J6 feedback (gripper STILL silent, feedback_numeric_valid
    // forced false): admission MUST succeed using fresh J1..J6 driver positions.
    inner.feedback_numeric_valid = false;
    bus.with(|s| {
        s.motors[1].q = 0.0;
        for j in 0..6 {
            s.push_feedback(j);
        }
    });
    {
        let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
        let _ = drv.read_joint_states();
    }
    assert_eq!(inner.may_disable_torque_while_stopped(), Ok(()));

    // 2. ZeroOffsetDriver must apply joint-zero offsets in fresh_joint_positions_into.
    let (mut inner_z, bus_z, _clock_z) = make_driver_rig();
    let mut offsets = vec![0.0; 7];
    offsets[1] = 0.40;
    inner_z.set_joint_zero_offsets(&offsets).unwrap();
    inner_z.set_torque(true).unwrap();
    inner_z.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));
    // Motor raw q = 0.0 with offset 0.40 => logical q = -0.40 (outside rest).
    bus_z.with(|s| {
        s.follow_commands = false;
        s.motors[1].q = 0.0;
        s.push_all_feedback();
    });
    inner_z.tick(0.02);
    assert!(matches!(
        inner_z.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::JointOutsideRest { joint_index: 1, position, .. })
            if (position + 0.40).abs() < 0.02
    ));
    // Motor raw q = 0.40 with offset 0.40 => logical q = 0.0 (at rest).
    bus_z.with(|s| {
        s.motors[1].q = 0.40;
        s.push_all_feedback();
    });
    inner_z.tick(0.02);
    assert_eq!(inner_z.may_disable_torque_while_stopped(), Ok(()));
}

#[test]
fn thermal_trip_and_shutdown_disable_at_raised_pose_without_consulting_rest_pose_decision() {
    // 1. ThermalTrip protective cut: raised pose AND resting_pose = None (so the rest-pose
    // decision would refuse with NoRestingPoseConfigured) -> thermal trip STILL disables all
    // motors and does NOT consult `eval_stopped_torque_off` (`last_stopped_torque_off_refusal`
    // stays None).
    let (mut inner_t, bus_t, _clock_t) = make_driver_rig();
    inner_t.config.resting_pose = None;
    inner_t.set_torque(true).unwrap();
    bus_t.with(|s| {
        s.follow_commands = false;
        s.motors[1].q = 0.45;
        s.motors[0].temperature_c = 90;
        s.push_all_feedback();
    });
    inner_t.tick(0.02);
    assert!(
        matches!(inner_t.safety.state(), SafetyState::Fault(_)),
        "overheating motor must latch ThermalTrip"
    );
    assert_eq!(inner_t.safety.fault_code(), Some(FaultCode::ThermalTrip));
    assert!(!inner_t.is_torqued, "ThermalTrip must disable motors at a raised pose");
    assert!(
        bus_t.with(|s| s.motors.iter().all(|m| !m.enabled)),
        "all motors must be disabled on wire after ThermalTrip at a raised pose"
    );
    assert_eq!(
        inner_t.last_stopped_torque_off_refusal(),
        None,
        "ThermalTrip must NOT consult eval_stopped_torque_off"
    );

    // 2. Signal path (`request_shutdown` for normal SIGTERM / Ctrl-C) with a known-enabled arm
    //    at a raised pose: refuses shutdown (returns false) on both first and repeated signals,
    //    latches EStop, keeps process in control and motors enabled (does NOT drop the arm), and
    //    does NOT consult `eval_stopped_torque_off`.
    let (mut inner_s, bus_s, clock_s) = make_driver_rig();
    inner_s.config.resting_pose = None;
    inner_s.set_torque(true).unwrap();
    bus_s.with(|s| {
        s.follow_commands = false;
        s.motors[1].q = 0.45;
        s.push_all_feedback();
    });
    inner_s.tick(0.02);
    assert!(inner_s.is_torqued);

    bus_s.with(|s| s.tx_log.clear());
    assert!(!inner_s.request_shutdown(), "first SIGTERM/Ctrl-C on known-enabled arm must refuse shutdown");
    assert!(!inner_s.request_shutdown(), "repeated SIGTERM/Ctrl-C on known-enabled arm must still refuse shutdown");
    assert_eq!(*inner_s.safety.state(), SafetyState::EStop);
    assert!(inner_s.is_torqued, "normal signal must keep known-enabled raised arm holding");
    assert!(
        bus_s.with(|s| s.motors.iter().all(|m| m.enabled)),
        "normal signal must not disable known-enabled motors at a raised pose"
    );
    assert!(
        bus_s.with(|s| s.tx_log.iter().all(|f| f.data != armd::protocol::damiao::CMD_DISABLE)),
        "no disable frame sent on normal signal with known-enabled arm"
    );
    assert_eq!(inner_s.last_stopped_torque_off_refusal(), None);

    // 3. Repeated signal teardown when enable state is Unknown and disable_pending is active:
    //    signal path issues the protective disable at the raised pose without consulting Rule A'.
    inner_s.disable_pending = true;
    clock_s.advance(Duration::from_millis(300)); // feedback stale -> driver enable_state == Unknown
    assert!(!inner_s.request_shutdown(), "first signal while Unknown records shutdown_refused_unknown");
    bus_s.with(|s| s.push_all_feedback()); // bus recovers for the repeated-signal teardown disable
    assert!(
        inner_s.request_shutdown(),
        "repeated shutdown signal while state was Unknown must complete protective disable at raised pose"
    );
    assert!(!inner_s.is_torqued);
    assert!(bus_s.with(|s| s.motors.iter().all(|m| !m.enabled)));
    assert_eq!(inner_s.last_stopped_torque_off_refusal(), None);

    // 4. Actual forced process exit / panic path (`attempt_panic_disable`) at a raised pose:
    //    disables motors unconditionally without consulting `eval_stopped_torque_off`.
    let (mut inner_p, bus_p, _clock_p) = make_driver_rig();
    inner_p.config.resting_pose = None;
    inner_p.set_torque(true).unwrap();
    bus_p.with(|s| {
        s.follow_commands = false;
        s.motors[1].q = 0.45;
        s.push_all_feedback();
    });
    inner_p.tick(0.02);
    let outcome = inner_p.attempt_panic_disable(Duration::from_secs(1));
    assert_eq!(outcome, armd::supervisor::PanicDisableOutcome::Confirmed);
    assert!(!inner_p.is_torqued, "panic/forced teardown must cut torque at a raised pose");
    assert!(
        bus_p.with(|s| s.motors.iter().all(|m| !m.enabled)),
        "all motors must be disabled on wire after panic/forced teardown at a raised pose"
    );
    assert!(inner_p.request_shutdown(), "shutdown_ready must be true once disabled");
    assert_eq!(
        inner_p.last_stopped_torque_off_refusal(),
        None,
        "panic/forced teardown must NOT consult eval_stopped_torque_off"
    );
}

#[test]
fn stopped_disable_retries_are_bounded_to_1s_and_new_request_requires_new_rest_check() {
    let (mut inner, bus, clock) = make_driver_rig();
    inner.set_torque(true).unwrap();
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    let epoch_before = inner.safety.epoch_id();

    // Authorize a stopped disable at rest whose TX fails -> sets disable_pending with DisableRequested.
    bus.with(|s| {
        s.follow_commands = false;
        s.tx_error = true;
        s.push_all_feedback();
    });
    assert!(inner.set_torque(false).is_err());
    assert!(inner.disable_pending);
    assert!(matches!(
        inner.active_disable_kind(),
        Some(armd::driver::DisableKind::DisableRequested { .. })
    ));

    // Within the 1 s lifetime, periodic retry (>= 250 ms since last attempt) still tries to complete it.
    // Refresh the empty-queue RX bound before crossing 100 ms so feedback stays provably fresh (<= 50 ms).
    clock.advance(Duration::from_millis(200));
    inner.last_disable_attempt = Some(Instant::now());
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    clock.advance(Duration::from_millis(50));
    inner.last_disable_attempt = Some(Instant::now() - Duration::from_millis(300));
    bus.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!(inner.disable_pending, "still pending while TX fails within 1 s lifetime");

    // Once the single 1 s lifetime elapses (checked inside `send`), even with fresh feedback at rest
    // (fresh receive-queue bound <= 25 ms), no disable frames are sent, disable_pending clears,
    // is_torqued and EStop latch + epoch stay intact, and a new request needs a fresh rest check.
    clock.advance(Duration::from_millis(740));
    inner.last_disable_attempt = Some(Instant::now());
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02); // refreshes empty-queue RX bound at t+990 ms
    clock.advance(Duration::from_millis(25)); // now t+1015 ms (> 1 s), feedback & queue bound <= 25 ms
    inner.last_disable_attempt = Some(Instant::now() - Duration::from_millis(300));
    bus.with(|s| {
        s.tx_error = false;
        s.tx_log.clear();
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!(!inner.disable_pending, "stopped disable request must end after 1 s lifetime in send");
    assert!(inner.active_disable_kind().is_none());
    assert!(inner.is_torqued);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    assert!(
        bus.with(|s| s.tx_log.iter().all(|f| f.data != armd::protocol::damiao::CMD_DISABLE)),
        "no disable frame may be sent once the 1 s request lifetime has elapsed"
    );

    // Raising the arm and issuing a new disable request must run a fresh rest check and refuse.
    bus.with(|s| {
        s.motors[1].q = 0.35;
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!(matches!(
        inner.set_torque(false),
        Err(SafetyError::TorqueOffWouldDrop { joint: Some(ref j), .. }) if j == "joint_2"
    ));
}

#[test]
fn feetech_fault_reset_routes_through_rest_pose_decision_on_both_owners() {
    use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
    use armd::model::{JointLimit, JointState, RestingPosePolicy};
    use std::sync::atomic::AtomicUsize;

    struct ResetDisablesArmDriver {
        pos: f32,
        reset_calls: Arc<AtomicUsize>,
    }
    impl ArmDriver for ResetDisablesArmDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
        fn enable_state(&self) -> EnableState { EnableState::Enabled }
        fn is_feedback_valid(&self) -> bool { true }
        fn has_any_joint_enabled(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(5))] }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            out.clear();
            out.push(Some(Duration::from_millis(5)));
        }
        fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
            dof == 1 && max_age >= Duration::from_millis(5)
        }
        fn fresh_joint_positions_into(&self, joints: &[usize], max_age: Duration, out: &mut [f32]) -> bool {
            if joints != [0] || out.is_empty() || max_age < Duration::from_millis(5) {
                return false;
            }
            out[0] = self.pos;
            true
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![JointState { position: self.pos, velocity: 0.0, torque: 0.0, temperature: 30.0 }])
        }
        fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
        fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        fn clear_fault_disables_joints(&self) -> Vec<usize> { vec![0] }
        fn clear_fault_disables_arm_joint(&self, arm_dof: usize) -> bool { arm_dof > 0 }
        fn has_hardware_fault(&self) -> bool { true }
        fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
            Err(ArmDriverError::HardwareFault("overload".into()))
        }
        fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
            self.reset_calls.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
        fn clear_fault_budgeted(&mut self, _deadline: Instant) -> Result<(), ArmDriverError> {
            self.reset_calls.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
    }

    let make_cfg = || {
        let mut cfg = ArmConfig::default();
        cfg.dof = 1;
        cfg.joint_names = vec!["joint_1".into()];
        cfg.limits = vec![JointLimit::new("joint_1", -3.0, 3.0, 2.0, 5.0)];
        cfg.home_positions = vec![0.0];
        cfg.rest_pose_tolerance_rad = 0.05;
        // Resting-support policy tolerance (0.08 rad) is wider than rest_pose_tolerance_rad (0.05 rad),
        // so at 0.07 rad the resting-support check passes while Rule A' MUST refuse with TORQUE_OFF_WOULD_DROP.
        cfg.resting_pose = Some(RestingPosePolicy {
            positions: Some(vec![0.0]),
            position_tolerances: vec![0.08],
            max_total_deviation: 0.16,
            max_velocity: 0.05,
            joint_max_velocities: vec![],
            support_torque_limits: vec![],
            support_load_limits: vec![],
            feedback_max_age_ms: 100,
        });
        cfg
    };

    // 1. Legacy clear_fault at 0.07 rad -> refused by Rule A' with TORQUE_OFF_WOULD_DROP, 0 resets sent.
    let reset_legacy = Arc::new(AtomicUsize::new(0));
    let mut inner_l = SupervisorInner::new(make_cfg())
        .with_driver(Box::new(ResetDisablesArmDriver { pos: 0.07, reset_calls: Arc::clone(&reset_legacy) }));
    inner_l.is_torqued = true;
    inner_l.measured_positions = vec![0.07];
    inner_l.measured_velocities = vec![0.0];
    inner_l.temperatures = vec![30.0];
    inner_l.safety.trigger_fault_coded(FaultCode::HardwareFault, FaultReason::Other("overload".into()));
    let err_l = inner_l.clear_fault().unwrap_err();
    assert!(
        matches!(err_l, SafetyError::TorqueOffWouldDrop { joint: Some(ref j), .. } if j == "joint_1"),
        "expected TORQUE_OFF_WOULD_DROP on legacy Feetech reset at 0.07 rad, got {err_l:?}"
    );
    assert_eq!(reset_legacy.load(Ordering::SeqCst), 0, "no hardware reset/torque-off may be sent");

    // 2. RT executor clear_fault_executor at 0.07 rad -> refused by Rule A' with TORQUE_OFF_WOULD_DROP, 0 resets sent.
    let reset_exec = Arc::new(AtomicUsize::new(0));
    let mut inner_e = SupervisorInner::new(make_cfg())
        .with_driver(Box::new(ResetDisablesArmDriver { pos: 0.07, reset_calls: Arc::clone(&reset_exec) }));
    inner_e.is_torqued = true;
    inner_e.measured_positions = vec![0.07];
    inner_e.measured_velocities = vec![0.0];
    inner_e.temperatures = vec![30.0];
    inner_e.safety.trigger_fault_coded(FaultCode::HardwareFault, FaultReason::Other("overload".into()));
    let err_e = inner_e.clear_fault_executor(Instant::now() + Duration::from_secs(1)).unwrap_err();
    assert!(
        matches!(err_e, SafetyError::TorqueOffWouldDrop { joint: Some(ref j), .. } if j == "joint_1"),
        "expected TORQUE_OFF_WOULD_DROP on executor Feetech reset at 0.07 rad, got {err_e:?}"
    );
    assert_eq!(reset_exec.load(Ordering::SeqCst), 0, "no hardware reset/torque-off may be sent");

    // 3. At 0.02 rad (within 0.05 rad rest tolerance) -> Rule A' allows the reset on both owners.
    let reset_ok = Arc::new(AtomicUsize::new(0));
    let mut inner_ok = SupervisorInner::new(make_cfg())
        .with_driver(Box::new(ResetDisablesArmDriver { pos: 0.02, reset_calls: Arc::clone(&reset_ok) }));
    inner_ok.is_torqued = true;
    inner_ok.measured_positions = vec![0.02];
    inner_ok.measured_velocities = vec![0.0];
    inner_ok.temperatures = vec![30.0];
    inner_ok.safety.trigger_fault_coded(FaultCode::HardwareFault, FaultReason::Other("overload".into()));
    let _ = inner_ok.clear_fault(); // sends reset, asks for fresh post-reset feedback
    assert_eq!(reset_ok.load(Ordering::SeqCst), 1, "reset must be sent when within rest_pose_tolerance_rad");
}

#[cfg(unix)]
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_sigterm_to_armd_subprocess_refuses_while_raised_and_exits_once_disabled() {
    use std::io::{BufRead, BufReader};
    use std::net::{SocketAddr, UdpSocket};
    use std::process::{Child, Command, Stdio};
    use std::sync::atomic::AtomicBool;
    use std::sync::Mutex;

    struct ChildGuard(Option<Child>);
    impl Drop for ChildGuard {
        fn drop(&mut self) {
            if let Some(mut c) = self.0.take() {
                let _ = c.kill();
                let _ = c.wait();
            }
        }
    }

    struct SimPeerState {
        enabled: bool,
        q: [f32; 7],
        ack: u64,
        disable_cmds: Vec<u64>,
        enable_cmds: Vec<u64>,
        client: Option<SocketAddr>,
    }

    async fn run_sigterm_for_owner(rt_owner: &str) {
        let tmp = tempfile::tempdir().unwrap();

        let udp = UdpSocket::bind("127.0.0.1:0").unwrap();
        let sim_addr = udp.local_addr().unwrap();
        udp.set_read_timeout(Some(Duration::from_millis(10))).unwrap();

        let peer_state = Arc::new(Mutex::new(SimPeerState {
            enabled: false,
            q: [0.0; 7],
            ack: 0,
            disable_cmds: Vec::new(),
            enable_cmds: Vec::new(),
            client: None,
        }));
        let peer_stop = Arc::new(AtomicBool::new(false));
        let peer_thread = {
            let state = Arc::clone(&peer_state);
            let stop = Arc::clone(&peer_stop);
            std::thread::spawn(move || {
                let mut buf = [0u8; 4096];
                let mut seq = 0u64;
                let mut last_tx = Instant::now() - Duration::from_secs(1);
                while !stop.load(Ordering::Relaxed) {
                    if let Ok((n, src)) = udp.recv_from(&mut buf) {
                        if let Ok(cmd) = serde_json::from_slice::<armd::sim_driver::SimCommand>(&buf[..n]) {
                            let mut s = state.lock().unwrap();
                            s.client = Some(src);
                            if cmd.kind == "cmd" {
                                s.ack = s.ack.max(cmd.seq);
                                match cmd.enable {
                                    Some(true) => {
                                        s.enabled = true;
                                        s.enable_cmds.push(cmd.seq);
                                    }
                                    Some(false) => {
                                        s.enabled = false;
                                        s.disable_cmds.push(cmd.seq);
                                    }
                                    None => {}
                                }
                                seq += 1;
                                let reply = armd::sim_driver::SimState {
                                    v: 1,
                                    kind: "state".into(),
                                    session: 1,
                                    seq,
                                    ack: s.ack,
                                    enabled: s.enabled,
                                    q: s.q.to_vec(),
                                    dq: vec![0.0; 7],
                                    tau: vec![0.0; 7],
                                    temp: vec![30.0; 7],
                                    loads: None,
                                    t: 0.0,
                                };
                                let bytes = serde_json::to_vec(&reply).unwrap();
                                let _ = udp.send_to(&bytes, src);
                                last_tx = Instant::now();
                            }
                        }
                    }
                    if last_tx.elapsed() >= Duration::from_millis(10) {
                        let s = state.lock().unwrap();
                        if let Some(dst) = s.client {
                            seq += 1;
                            let reply = armd::sim_driver::SimState {
                                v: 1,
                                kind: "state".into(),
                                session: 1,
                                seq,
                                ack: s.ack,
                                enabled: s.enabled,
                                q: s.q.to_vec(),
                                dq: vec![0.0; 7],
                                tau: vec![0.0; 7],
                                temp: vec![30.0; 7],
                                loads: None,
                                t: 0.0,
                            };
                            drop(s);
                            let bytes = serde_json::to_vec(&reply).unwrap();
                            let _ = udp.send_to(&bytes, dst);
                            last_tx = Instant::now();
                        }
                    }
                }
            })
        };

        let mut child = Command::new(env!("CARGO_BIN_EXE_armd"))
            .args([
                "--follower",
                "sim",
                "--follower-port",
                &format!("udp://{sim_addr}"),
                "--port",
                "0",
                "--log-dir",
                "off",
                "--blackbox-dir",
                "off",
                "--episodes-dir",
                "off",
                "--no-operator-auth",
            ])
            .env("ARMD_RT_OWNER", rt_owner)
            .env("ARMD_CALIBRATION", tmp.path().join("calibration.json"))
            .env("ARMD_GRASP_SETUP", tmp.path().join("grasp_setup.json"))
            .env("ARMD_KEEPOUT", tmp.path().join("keepout.json"))
            .env("ARMD_POSES", tmp.path().join("poses.json"))
            .env("ARMD_SEQUENCES", tmp.path().join("sequences.json"))
            .env("ARMD_COLLISION", tmp.path().join("collision.json"))
            .env("ARMD_TOOL_COMMISSIONING", tmp.path().join("tool_comm.json"))
            .env("ARMD_GRIPPER_CALIB", tmp.path().join("gripper_calib.json"))
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .expect("spawn armd binary");

        let stdout = child.stdout.take().expect("piped stdout");
        let (port_tx, port_rx) = std::sync::mpsc::channel::<u16>();
        let stdout_drain = std::thread::spawn(move || {
            let reader = BufReader::new(stdout);
            let mut sent = false;
            for line in reader.lines().map_while(Result::ok) {
                if !sent {
                    if let Some(idx) = line.find("listening on http://") {
                        let rest = &line[idx + "listening on http://".len()..];
                        if let Some(colon) = rest.rfind(':') {
                            let digits: String = rest[colon + 1..]
                                .chars()
                                .take_while(|c| c.is_ascii_digit())
                                .collect();
                            if let Ok(port) = digits.parse::<u16>() {
                                let _ = port_tx.send(port);
                                sent = true;
                            }
                        }
                    }
                }
            }
        });

        let mut guard = ChildGuard(Some(child));
        let pid = guard.0.as_ref().unwrap().id() as libc::pid_t;

        let http_port = port_rx
            .recv_timeout(Duration::from_secs(10))
            .expect("armd did not advertise listening port");
        assert!(
            ![18765u16, 18001, 18990].contains(&http_port),
            "ephemeral test port collided with forbidden port {http_port}"
        );
        let http_addr: SocketAddr = format!("127.0.0.1:{http_port}").parse().unwrap();
        let client = reqwest::Client::new();
        let obs_url = format!("http://{http_addr}/api/v1/observation");

        // Wait until armd has fresh feedback from the UDP sim peer.
        let mut ready = false;
        for _ in 0..100 {
            if let Ok(resp) = client.get(&obs_url).send().await {
                if let Ok(v) = resp.json::<serde_json::Value>().await {
                    if v["feedback_valid"] == true && v["enable_state"] == "Disabled" {
                        ready = true;
                        break;
                    }
                }
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        assert!(ready, "armd ({rt_owner}) never became feedback_valid/Disabled");

        // Acquire WS lease and enable torque at rest.
        let (mut ws, generation) = common::controller(http_addr).await;
        let obs0: serde_json::Value = client.get(&obs_url).send().await.unwrap().json().await.unwrap();
        let epoch0 = obs0["epoch_id"].as_u64().unwrap();
        let ack = common::rpc(&mut ws, generation, epoch0, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack["type"], "command_ack", "enable torque failed ({rt_owner}): {ack}");

        // Move arm to a raised pose on the sim peer and wait until armd observes it.
        peer_state.lock().unwrap().q = [0.0, 0.45, -0.35, 0.0, 0.0, 0.0, 0.0];
        let mut raised_observed = false;
        for _ in 0..100 {
            let obs: serde_json::Value = client.get(&obs_url).send().await.unwrap().json().await.unwrap();
            if obs["is_torqued"] == true
                && obs["enable_state"] == "Enabled"
                && obs["positions"][1].as_f64().is_some_and(|p| p > 0.40)
            {
                raised_observed = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(raised_observed, "armd ({rt_owner}) did not observe raised torqued arm");

        // Clear recorded disable commands, then deliver a REAL OS SIGTERM to the armd PID.
        peer_state.lock().unwrap().disable_cmds.clear();
        // SAFETY: `pid` is the live child process owned by `guard`.
        let rc = unsafe { libc::kill(pid, libc::SIGTERM) };
        assert_eq!(rc, 0, "libc::kill(SIGTERM) failed");

        // Wait until armd processes SIGTERM and latches EStop.
        let mut estop_observed = false;
        for _ in 0..100 {
            let obs: serde_json::Value = client.get(&obs_url).send().await.unwrap().json().await.unwrap();
            if obs["safety_state"] == "EStop" {
                estop_observed = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(estop_observed, "armd ({rt_owner}) did not latch EStop upon real SIGTERM");

        // Give the control loop time to run multiple ticks after SIGTERM: process MUST stay alive,
        // keep holding torque, and transmit ZERO disable commands to the sim peer.
        tokio::time::sleep(Duration::from_millis(250)).await;
        assert!(
            guard.0.as_mut().unwrap().try_wait().unwrap().is_none(),
            "armd ({rt_owner}) must NOT exit on SIGTERM while arm is known-enabled and raised"
        );
        {
            let s = peer_state.lock().unwrap();
            assert!(
                s.disable_cmds.is_empty(),
                "real SIGTERM on raised arm ({rt_owner}) must send ZERO disable commands, got {:?}",
                s.disable_cmds
            );
            assert!(s.enabled, "sim motors must remain enabled after refused SIGTERM ({rt_owner})");
        }
        let obs_after: serde_json::Value = client.get(&obs_url).send().await.unwrap().json().await.unwrap();
        assert_eq!(obs_after["safety_state"], "EStop");
        assert_eq!(obs_after["is_torqued"], true);

        // Requested torque-off via WS while raised is STILL refused with TORQUE_OFF_WOULD_DROP
        // and sends ZERO disable commands.
        let epoch_estop = obs_after["epoch_id"].as_u64().unwrap();
        let rej = common::rpc(&mut ws, generation, epoch_estop, "/api/v1/torque", json!({"enable": false})).await;
        assert_eq!(rej["type"], "command_reject", "{rej}");
        assert_eq!(rej["code"], "TORQUE_OFF_WOULD_DROP", "{rej}");
        assert!(peer_state.lock().unwrap().disable_cmds.is_empty());

        // Bring the arm back to the resting pose, disable torque cleanly, then send a second
        // real SIGTERM and confirm armd exits cleanly.
        peer_state.lock().unwrap().q = [0.0; 7];
        let mut rest_observed = false;
        for _ in 0..100 {
            let obs: serde_json::Value = client.get(&obs_url).send().await.unwrap().json().await.unwrap();
            if obs["positions"][1].as_f64().is_some_and(|p| p.abs() < 0.01) {
                rest_observed = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(rest_observed, "armd ({rt_owner}) did not observe arm returning to rest");

        // Refresh WS heartbeat before torque-off at rest so lease stays fresh even under CI load.
        common::send(
            &mut ws,
            json!({"type": "heartbeat", "generation": generation, "epoch_id": epoch_estop}),
        )
        .await;
        let off_ack = common::rpc(&mut ws, generation, epoch_estop, "/api/v1/torque", json!({"enable": false})).await;
        assert_eq!(off_ack["type"], "command_ack", "torque-off at rest failed ({rt_owner}): {off_ack}");
        assert!(
            !peer_state.lock().unwrap().disable_cmds.is_empty(),
            "torque-off at rest must send disable command"
        );

        let mut disabled_observed = false;
        for _ in 0..100 {
            let obs: serde_json::Value = client.get(&obs_url).send().await.unwrap().json().await.unwrap();
            if obs["is_torqued"] == false && obs["enable_state"] == "Disabled" {
                disabled_observed = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(disabled_observed, "armd ({rt_owner}) did not confirm Disabled at rest");

        // Second real SIGTERM now that torque is confirmed off: armd exits cleanly.
        // SAFETY: `pid` is the live child process owned by `guard`.
        let rc2 = unsafe { libc::kill(pid, libc::SIGTERM) };
        assert_eq!(rc2, 0, "second libc::kill(SIGTERM) failed");

        let mut exit_status = None;
        for _ in 0..200 {
            if let Some(st) = guard.0.as_mut().unwrap().try_wait().unwrap() {
                exit_status = Some(st);
                break;
            }
            tokio::time::sleep(Duration::from_millis(25)).await;
        }
        let status = exit_status.expect("armd did not exit within 5 s after SIGTERM while Disabled");
        assert!(
            status.success(),
            "armd ({rt_owner}) must exit cleanly after SIGTERM while Disabled, got {status:?}"
        );
        guard.0 = None;
        peer_stop.store(true, Ordering::Relaxed);
        let _ = peer_thread.join();
        let _ = stdout_drain.join();
    }

    run_sigterm_for_owner("legacy").await;
}

/// Comprehensive choke-point matrix over every disable entry point and retry path
/// (legacy handshake resend, RT poll retry, periodic retry, pre-stop-started request,
/// rollback with Unknown prior state, Feetech reset/disable, SimArmDriver, and protective
/// reasons) × {AtRest, Raised, Stale}, asserting directly on the bus / transport trace.
#[test]
fn chokepoint_matrix_all_disable_paths_by_pose_asserts_bus_trace() {
    use armd::driver::{
        DisableKind, DisableProtectiveReason, FeetechArmDriver, StoppedDisableGate,
    };
    use armd::protocol::feetech::{calculate_checksum, INST_WRITE, REG_TORQUE_ENABLE};
    use armd::sim_driver::{SimArmDriver, SimCommand, SimState};
    use armd::transport::MockTransport;
    use std::net::{SocketAddr, UdpSocket};
    use std::sync::atomic::AtomicBool;
    use std::sync::Mutex;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum PoseCase {
        AtRest,
        Raised,
        Stale,
    }

    const ALL_POSES: [PoseCase; 3] = [PoseCase::AtRest, PoseCase::Raised, PoseCase::Stale];

    fn count_damiao_disables(bus: &FakeBus) -> usize {
        bus.tx_log()
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE)
            .count()
    }

    fn count_feetech_torque_off_writes(tx: &[u8]) -> usize {
        // Feetech WRITE packet to REG_TORQUE_ENABLE (0x28) with value 0:
        // [0xFF, 0xFF, id, 0x04, INST_WRITE (0x03), REG_TORQUE_ENABLE (0x28), 0x00, cs]
        tx.windows(8)
            .filter(|w| {
                w[0] == 0xFF
                    && w[1] == 0xFF
                    && w[3] == 0x04
                    && w[4] == INST_WRITE
                    && w[5] == REG_TORQUE_ENABLE
                    && w[6] == 0x00
            })
            .count()
    }

    // -------------------------------------------------------------------------
    // 1. Legacy handshake resend (`DamiaoArmDriver::poll_disable`) × {AtRest, Raised, Stale}
    // -------------------------------------------------------------------------
    for pose in ALL_POSES {
        let (mut inner, bus, clock) = make_driver_rig();
        inner.set_torque(true).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        // Start a DisableRequested handshake at rest while motor 2 stays silent (never replies
        // state-0), so DamiaoTorque::Disable stays pending waiting for the 25 ms retry.
        bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        inner.tick(0.02);
        bus.with(|s| {
            s.motors[1].torque_reply_delay = Duration::from_secs(10);
            s.tx_log.clear();
        });
        let started_at = {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let t0 = drv.clock_now();
            let p = drv
                .begin_disable_torque(DisableKind::DisableRequested {
                    started_at: t0,
                    rollback: false,
                })
                .expect("initial disable at rest must start");
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
            t0
        };
        assert_eq!(count_damiao_disables(&bus), 7, "initial send emits 7 disable frames");

        // Clear bus trace before the 25 ms handshake resend, then set pose case.
        // Restore motor 2 `enabled = true` to simulate a lost disable frame on the wire so the
        // motor is still physically energized (0x1) and `poll_torque` must issue a retry.
        bus.with(|s| {
            s.tx_log.clear();
            s.rx.clear();
            s.pending.clear();
            s.motors[1].enabled = true;
        });
        match pose {
            PoseCase::AtRest => {
                clock.advance(Duration::from_millis(30));
                bus.with(|s| s.push_all_feedback());
            }
            PoseCase::Raised => {
                clock.advance(Duration::from_millis(30));
                bus.with(|s| {
                    s.motors[1].q = 0.40;
                    s.push_all_feedback();
                });
            }
            PoseCase::Stale => {
                // Past 100 ms feedback bound, still within 150 ms confirm window -> hits 25 ms retry.
                clock.advance(Duration::from_millis(110));
            }
        }
        let poll_res = {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let dl = drv.clock_now() + Duration::from_millis(5);
            drv.poll_torque(dl)
        };
        let disables = count_damiao_disables(&bus);
        match pose {
            PoseCase::AtRest => {
                assert!(poll_res.is_ok(), "legacy handshake resend at rest failed: {poll_res:?}");
                assert!(disables > 0, "legacy handshake resend at rest must transmit retry disable");
            }
            PoseCase::Raised | PoseCase::Stale => {
                assert!(
                    matches!(poll_res, Err(armd::driver::ArmDriverError::TorqueOffWouldDrop)),
                    "legacy handshake resend ({pose:?}) must refuse with TorqueOffWouldDrop, got {poll_res:?}"
                );
                assert_eq!(
                    disables, 0,
                    "legacy handshake resend ({pose:?}) must transmit ZERO disable frames"
                );
            }
        }
        let _ = started_at;
    }

    // Also verify legacy handshake resend past 1 s total request lifetime sends 0 frames even at rest.
    {
        let (mut inner, bus, clock) = make_driver_rig();
        inner.set_torque(true).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        inner.tick(0.02);
        let t_req = clock.now();
        // Start the handshake at t+985 ms (still < 1 s), then poll retry at t+1015 ms (> 1 s).
        clock.advance(Duration::from_millis(950));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        clock.advance(Duration::from_millis(35));
        bus.with(|s| {
            s.motors[1].torque_reply_delay = Duration::from_secs(10);
            s.push_all_feedback();
        });
        inner.tick(0.02);
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let p = drv
                .begin_disable_torque(DisableKind::DisableRequested {
                    started_at: t_req,
                    rollback: false,
                })
                .expect("disable started within 1 s must start");
            assert!(matches!(p, armd::driver::TorqueProgress::Pending { .. }));
        }
        bus.with(|s| {
            s.tx_log.clear();
            s.rx.clear();
            s.pending.clear();
            s.motors[1].enabled = true;
        });
        clock.advance(Duration::from_millis(30)); // now t_req + 1015 ms, feedback is fresh (30 ms)!
        bus.with(|s| s.push_all_feedback());
        let res = {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let dl = drv.clock_now() + Duration::from_millis(5);
            drv.poll_torque(dl)
        };
        assert!(
            matches!(res, Err(armd::driver::ArmDriverError::DisableRequestExpired)),
            "legacy handshake resend past 1 s must return DisableRequestExpired, got {res:?}"
        );
        assert_eq!(count_damiao_disables(&bus), 0, "0 disable frames after 1 s lifetime");
    }

    // -------------------------------------------------------------------------
    // 2. RT poll retry (`service_torque_transition`) × {AtRest, Raised, Stale}
    // -------------------------------------------------------------------------
    for pose in ALL_POSES {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            s.push_all_feedback();
        });
        r.step();
        r.bus.with(|s| {
            s.motors[2].torque_reply_delay = Duration::from_secs(10);
            s.push_all_feedback();
        });
        r.submit_transition(TransitionOp::TorqueOff);
        let first = r.step();
        assert!(first.command.unwrap().1.is_ok(), "{first:?}");
        assert!(r.fx.core.torque_transition_pending());

        r.bus.with(|s| {
            s.tx_log.clear();
            s.rx.clear();
            s.pending.clear();
            s.motors[2].enabled = true;
        });
        match pose {
            PoseCase::AtRest => {
                r.clock.advance(Duration::from_millis(30));
                r.bus.with(|s| s.push_all_feedback());
            }
            PoseCase::Raised => {
                r.clock.advance(Duration::from_millis(30));
                r.bus.with(|s| {
                    s.motors[1].q = 0.40;
                    s.push_all_feedback();
                });
            }
            PoseCase::Stale => {
                r.clock.advance(Duration::from_millis(110));
            }
        }
        let rep = r.step();
        let disables = count_damiao_disables(&r.bus);
        match pose {
            PoseCase::AtRest => {
                assert!(disables > 0, "RT poll retry at rest must transmit disable retry frame");
            }
            PoseCase::Raised | PoseCase::Stale => {
                assert_eq!(
                    rep.transition_ended.map(|(_, o)| o),
                    Some(Outcome::Failed("TORQUE_OFF_WOULD_DROP")),
                    "RT poll retry ({pose:?}) must fail with TORQUE_OFF_WOULD_DROP"
                );
                assert!(!r.fx.core.disable_pending);
                assert_eq!(
                    disables, 0,
                    "RT poll retry ({pose:?}) must transmit ZERO disable frames"
                );
            }
        }
    }

    // -------------------------------------------------------------------------
    // 3. Periodic retry & request-started-before-stop × {AtRest, Raised, Stale}
    // -------------------------------------------------------------------------
    for started_before_stop in [false, true] {
        for pose in ALL_POSES {
            let (mut inner, bus, clock) = make_driver_rig();
            inner.set_torque(true).unwrap();
            bus.with(|s| {
                s.follow_commands = false;
                s.rx.clear();
                s.pending.clear();
                s.push_all_feedback();
            });
            inner.tick(0.02);

            if !started_before_stop {
                inner.apply_stop_policy(StopPolicy::EStopLatch);
            }
            bus.with(|s| {
                s.tx_error = true;
                s.rx.clear();
                s.pending.clear();
                s.push_all_feedback();
            });
            assert!(inner.set_torque(false).is_err());
            assert!(inner.disable_pending);

            if started_before_stop {
                inner.apply_stop_policy(StopPolicy::EStopLatch);
            }
            bus.with(|s| {
                s.tx_error = false;
                s.tx_log.clear();
                s.rx.clear();
                s.pending.clear();
            });
            inner.last_disable_attempt = Some(Instant::now() - Duration::from_millis(300));
            match pose {
                PoseCase::AtRest => {
                    clock.advance(Duration::from_millis(30));
                    bus.with(|s| s.push_all_feedback());
                }
                PoseCase::Raised => {
                    clock.advance(Duration::from_millis(30));
                    bus.with(|s| {
                        s.motors[1].q = 0.40;
                        s.push_all_feedback();
                    });
                }
                PoseCase::Stale => {
                    clock.advance(Duration::from_millis(110));
                }
            }
            inner.tick(0.02);
            let disables = count_damiao_disables(&bus);
            match pose {
                PoseCase::AtRest => {
                    assert!(
                        disables > 0,
                        "periodic retry (pre_stop={started_before_stop}) at rest must transmit disable"
                    );
                    assert!(!inner.disable_pending);
                }
                PoseCase::Raised | PoseCase::Stale => {
                    assert_eq!(
                        disables, 0,
                        "periodic retry (pre_stop={started_before_stop}, {pose:?}) must send ZERO disable frames"
                    );
                    assert!(!inner.disable_pending);
                    assert!(inner.is_torqued);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // 4. Enable rollback (BOTH previously-disabled AND Unknown prior state) × {AtRest, Raised, Stale}
    //    Round 6 (Decision 1): EVERY enable rollback goes through `DisableRequested { rollback: true }`
    //    with NO already-off bypass — allowed ONLY at rest from fresh feedback; refused raised/stale
    //    with 0 disables, torque held, and recoverable `ENABLE_ROLLBACK_HELD` fault.
    // -------------------------------------------------------------------------
    for prior_all_off in [false, true] {
        for pose in ALL_POSES {
            let clock = MockClock::new();
            let bus = FakeBus::with_clock(&limits(), clock.clone());
            let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(limits())
                .unwrap();
            // Establish initial empty-queue bound.
            d.read_joint_states().unwrap();
            d.check_mapping_ranges();
            let mut inner = SupervisorInner::new(ArmConfig {
                lease_timeout_ms: 600_000,
                ..ArmConfig::rebot_dm()
            })
            .with_driver(Box::new(d));
            inner.lease_enabled = false;

            // Set arm pose and prior enable state:
            // - prior_all_off = true: all 7 motors freshly report state 0 -> EnableState::Disabled.
            // - prior_all_off = false: J1..J6 freshly report status 0x8 and J7 is silent -> Unknown.
            // Both MUST follow the exact same `DisableRequested { rollback: true }` rule!
            let target_q = if pose == PoseCase::Raised { 0.40 } else { 0.0 };
            bus.with(|s| {
                s.follow_commands = false;
                s.rx.clear();
                s.pending.clear();
                for m in &mut s.motors {
                    m.enabled = false;
                    m.state_override = if prior_all_off { None } else { Some(0x8) };
                    m.q = target_q;
                }
                if prior_all_off {
                    s.push_all_feedback();
                } else {
                    for j in 0..6 {
                        s.push_feedback(j);
                    }
                }
            });
            {
                let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
                drv.read_joint_states().unwrap();
                let st = drv.enable_state();
                if prior_all_off {
                    assert_eq!(st, EnableState::Disabled);
                } else {
                    assert_eq!(st, EnableState::Unknown);
                }
            }

            // Fail the enable handshake on motor 4 (CAN ID 4) so `enable_failed` triggers rollback.
            if pose == PoseCase::Stale {
                bus.with(|s| {
                    s.rx.clear();
                    s.pending.clear();
                    s.fail_tx_ids = vec![4];
                    s.tx_lost = true;
                    s.tx_step = Duration::from_millis(40);
                });
            } else {
                bus.with(|s| {
                    s.fail_tx_ids = vec![4];
                });
            }
            bus.clear_log();
            let err = inner.set_torque(true).unwrap_err();
            assert!(
                matches!(err, SafetyError::ClearFaultRejected(_)),
                "expected enable failure, got {err:?}"
            );
            assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
            let disables = count_damiao_disables(&bus);
            match pose {
                PoseCase::AtRest => {
                    assert!(
                        disables > 0,
                        "enable rollback at rest (prior_all_off={prior_all_off}) must send disable frames"
                    );
                    assert_eq!(
                        inner.safety.fault().map(|f| f.code),
                        Some(FaultCode::EnableFailed),
                        "allowed at-rest rollback latches EnableFailed"
                    );
                }
                PoseCase::Raised | PoseCase::Stale => {
                    assert_eq!(
                        disables, 0,
                        "enable rollback (prior_all_off={prior_all_off}, {pose:?}) must transmit ZERO disable frames"
                    );
                    assert!(inner.is_torqued, "refused rollback must keep torque on");
                    assert!(!inner.disable_pending, "refused rollback must clear disable_pending");
                    assert_eq!(
                        inner.safety.fault().map(|f| f.code),
                        Some(FaultCode::EnableRollbackHeld),
                        "refused rollback (prior_all_off={prior_all_off}, {pose:?}) must latch EnableRollbackHeld"
                    );
                    assert!(
                        inner.safety.recoverable(),
                        "EnableRollbackHeld must be recoverable through arm.recover()"
                    );
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // 5. Feetech reset (`clear_fault`) & `disable_torque` × {AtRest, Raised, Stale}
    // -------------------------------------------------------------------------
    for pose in ALL_POSES {
        let clock = MockClock::new();
        let mock = MockTransport::new_preserving().with_clock(clock.clone());
        let servo_ids: Vec<u8> = (1..=6).collect();
        let mut drv = FeetechArmDriver::new(mock, servo_ids.clone())
            .with_calibrated(true)
            .with_tx_echo(Some(false))
            .with_gripper_index(Some(5));

        let safety_stopped = Arc::new(AtomicBool::new(true));
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        for i in 0..5 {
            arm_joints[i] = i;
        }
        drv.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::clone(&safety_stopped),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 5,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });

        // Push fresh feedback with servo 2 reporting overload error (0x20) at target pose:
        // 2048 steps = 0.0 rad (rest), 2400 steps ≈ +0.54 rad (raised).
        let pos_steps: u16 = if pose == PoseCase::Raised { 2400 } else { 2048 };
        let pos_le = pos_steps.to_le_bytes();
        let mut rx_fb = Vec::new();
        for &id in &servo_ids {
            let err_byte = if id == 2 { 0x20 } else { 0x00 };
            let params = [pos_le[0], pos_le[1], 0, 0, 0, 0, 70, 35];
            let cs = calculate_checksum(id, 10, err_byte, &params);
            rx_fb.extend_from_slice(&[0xFF, 0xFF, id, 10, err_byte]);
            rx_fb.extend_from_slice(&params);
            rx_fb.push(cs);
        }
        // Feetech read_joint_states loops over both 3-servo chunks in one call.
        drv.transport_mut().provide_rx(&rx_fb);
        drv.read_joint_states().unwrap();
        let _ = drv.transport_mut().take_tx();

        if pose == PoseCase::Stale {
            clock.advance(Duration::from_millis(120));
        }

        // 5a. Feetech `clear_fault()` hardware reset (routes through DisableRequested choke point).
        let mut rx_rb = Vec::new();
        for &id in &servo_ids {
            let cs = calculate_checksum(id, 3, 0x00, &[0]);
            rx_rb.extend_from_slice(&[0xFF, 0xFF, id, 3, 0x00, 0, cs]);
        }
        drv.transport_mut().provide_rx(&rx_rb);
        let cf_res = drv.clear_fault();
        let cf_tx = drv.transport_mut().take_tx();
        let cf_disables = count_feetech_torque_off_writes(&cf_tx);
        match pose {
            PoseCase::AtRest => {
                assert!(cf_res.is_ok(), "Feetech clear_fault at rest failed: {cf_res:?}");
                assert_eq!(cf_disables, 1, "Feetech clear_fault at rest disables faulted servo 2");
            }
            PoseCase::Raised | PoseCase::Stale => {
                assert!(
                    matches!(cf_res, Err(armd::driver::ArmDriverError::TorqueOffWouldDrop)),
                    "Feetech clear_fault ({pose:?}) must refuse with TorqueOffWouldDrop, got {cf_res:?}"
                );
                assert_eq!(
                    cf_disables, 0,
                    "Feetech clear_fault ({pose:?}) must write ZERO TORQUE_ENABLE=0 packets"
                );
            }
        }

        // 5b. Feetech `disable_torque(DisableRequested)` vs `DisableProtective`.
        drv.transport_mut().provide_rx(&rx_rb);
        let req_res = drv.disable_torque(DisableKind::DisableRequested {
            started_at: drv.clock_now(),
            rollback: false,
        });
        let req_tx = drv.transport_mut().take_tx();
        let req_disables = count_feetech_torque_off_writes(&req_tx);
        match pose {
            PoseCase::AtRest => {
                assert!(req_res.is_ok(), "Feetech DisableRequested at rest failed: {req_res:?}");
                assert_eq!(req_disables, 6);
            }
            PoseCase::Raised | PoseCase::Stale => {
                assert!(
                    matches!(req_res, Err(armd::driver::ArmDriverError::TorqueOffWouldDrop)),
                    "Feetech DisableRequested ({pose:?}) must refuse, got {req_res:?}"
                );
                assert_eq!(req_disables, 0, "Feetech DisableRequested ({pose:?}) must send 0 packets");
            }
        }

        // Protective reasons (`Thermal`, `ProcessExit`) bypass Rule A′ on Feetech in all poses.
        for reason in [
            DisableProtectiveReason::Thermal,
            DisableProtectiveReason::ProcessExit,
        ] {
            drv.transport_mut().provide_rx(&rx_rb);
            drv.disable_torque(DisableKind::DisableProtective { reason })
                .unwrap();
            let prot_tx = drv.transport_mut().take_tx();
            assert_eq!(
                count_feetech_torque_off_writes(&prot_tx),
                6,
                "Feetech DisableProtective({reason:?}) ({pose:?}) must send disable to all servos"
            );
        }
    }

    // -------------------------------------------------------------------------
    // 6. SimArmDriver (`DisableRequested` vs `DisableProtective`) × {AtRest, Raised, Stale}
    // -------------------------------------------------------------------------
    for pose in ALL_POSES {
        let udp = UdpSocket::bind("127.0.0.1:0").unwrap();
        let peer_addr: SocketAddr = udp.local_addr().unwrap();
        udp.set_read_timeout(Some(Duration::from_millis(5))).unwrap();

        let target_q = if pose == PoseCase::Raised {
            vec![0.0, 0.40, 0.0, 0.0, 0.0, 0.0, 0.0]
        } else {
            vec![0.0; 7]
        };
        let disable_cmds = Arc::new(Mutex::new(Vec::<u64>::new()));
        let send_state = Arc::new(AtomicBool::new(true));
        let stop_peer = Arc::new(AtomicBool::new(false));
        let peer_thread = {
            let disable_cmds = Arc::clone(&disable_cmds);
            let send_state = Arc::clone(&send_state);
            let stop_peer = Arc::clone(&stop_peer);
            let target_q = target_q.clone();
            std::thread::spawn(move || {
                let mut buf = [0u8; 4096];
                let mut seq = 0u64;
                let mut ack = 0u64;
                let mut enabled = true;
                let mut client: Option<SocketAddr> = None;
                let mut last_tx = Instant::now() - Duration::from_secs(1);
                while !stop_peer.load(Ordering::Relaxed) {
                    if let Ok((n, src)) = udp.recv_from(&mut buf) {
                        client = Some(src);
                        if let Ok(cmd) = serde_json::from_slice::<SimCommand>(&buf[..n]) {
                            if cmd.kind == "cmd" {
                                ack = ack.max(cmd.seq);
                                if cmd.enable == Some(false) {
                                    disable_cmds.lock().unwrap().push(cmd.seq);
                                    enabled = false;
                                } else if cmd.enable == Some(true) {
                                    enabled = true;
                                }
                            }
                        }
                    }
                    if send_state.load(Ordering::Relaxed) && last_tx.elapsed() >= Duration::from_millis(5) {
                        if let Some(dst) = client {
                            seq += 1;
                            let reply = SimState {
                                v: 1,
                                kind: "state".into(),
                                session: 1,
                                seq,
                                ack,
                                enabled,
                                q: target_q.clone(),
                                dq: vec![0.0; 7],
                                tau: vec![0.0; 7],
                                temp: vec![30.0; 7],
                                loads: None,
                                t: 0.0,
                            };
                            let bytes = serde_json::to_vec(&reply).unwrap();
                            let _ = udp.send_to(&bytes, dst);
                            last_tx = Instant::now();
                        }
                    }
                }
            })
        };

        let mut sim = SimArmDriver::connect(&peer_addr.to_string(), 7).unwrap();
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        for i in 0..6 {
            arm_joints[i] = i;
        }
        sim.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 6,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });

        // Wait for fresh feedback packet from peer.
        let mut got_fb = false;
        for _ in 0..100 {
            let _ = sim.read_joint_states();
            if sim.is_feedback_valid() {
                got_fb = true;
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        assert!(got_fb, "sim peer did not deliver initial feedback");

        if pose == PoseCase::Stale {
            send_state.store(false, Ordering::Relaxed);
            std::thread::sleep(Duration::from_millis(120));
        }
        disable_cmds.lock().unwrap().clear();

        let req_res = sim.begin_disable_torque(DisableKind::DisableRequested {
            started_at: Instant::now(),
            rollback: false,
        });
        std::thread::sleep(Duration::from_millis(25));
        let req_count = disable_cmds.lock().unwrap().len();
        match pose {
            PoseCase::AtRest => {
                assert!(req_res.is_ok(), "Sim DisableRequested at rest failed: {req_res:?}");
                assert_eq!(req_count, 1, "Sim DisableRequested at rest must transmit enable=false");
            }
            PoseCase::Raised | PoseCase::Stale => {
                assert!(
                    matches!(req_res, Err(armd::driver::ArmDriverError::TorqueOffWouldDrop)),
                    "Sim DisableRequested ({pose:?}) must refuse with TorqueOffWouldDrop, got {req_res:?}"
                );
                assert_eq!(
                    req_count, 0,
                    "Sim DisableRequested ({pose:?}) must transmit ZERO UDP disable commands"
                );
            }
        }

        // Protective reasons (`Thermal`, `ProcessExit`) bypass Rule A′ on SimArmDriver across all poses.
        for reason in [
            DisableProtectiveReason::Thermal,
            DisableProtectiveReason::ProcessExit,
        ] {
            disable_cmds.lock().unwrap().clear();
            sim.begin_disable_torque(DisableKind::DisableProtective { reason })
                .unwrap();
            let mut seen = false;
            for _ in 0..40 {
                if !disable_cmds.lock().unwrap().is_empty() {
                    seen = true;
                    break;
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            assert!(
                seen,
                "Sim DisableProtective({reason:?}) ({pose:?}) must transmit UDP disable command"
            );
        }

        stop_peer.store(true, Ordering::Relaxed);
        let _ = peer_thread.join();
    }

    // -------------------------------------------------------------------------
    // 7. Protective reasons (`Thermal`, `ProcessExit`) on Damiao × {AtRest, Raised, Stale}
    // -------------------------------------------------------------------------
    for reason in [
        DisableProtectiveReason::Thermal,
        DisableProtectiveReason::ProcessExit,
    ] {
        for pose in ALL_POSES {
            let (mut inner, bus, clock) = make_driver_rig();
            inner.set_torque(true).unwrap();
            inner.apply_stop_policy(StopPolicy::EStopLatch);
            bus.with(|s| {
                s.follow_commands = false;
                s.rx.clear();
                s.pending.clear();
                if pose == PoseCase::Raised {
                    s.motors[1].q = 0.50;
                }
                s.push_all_feedback();
            });
            inner.tick(0.02);
            if pose == PoseCase::Stale {
                clock.advance(Duration::from_millis(120));
            }
            bus.clear_log();
            {
                let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
                drv.begin_disable_torque(DisableKind::DisableProtective { reason })
                    .unwrap();
            }
            assert_eq!(
                count_damiao_disables(&bus),
                7,
                "Damiao DisableProtective({reason:?}) ({pose:?}) must transmit all 7 disable frames"
            );
        }
    }
}

/// Round 5 item 2: partial enable at a raised pose followed by an enable-finish error (both
/// `read_res.is_err()` and `!is_feedback_valid() / enable_state != Enabled` branches while
/// safety is still `Idle`) MUST gate rollback via `DisableRequested { rollback: true }`:
/// 0 disable frames transmitted to the enabled motors, motors remain holding, and a recoverable
/// `EnableFailed` fault explains it.
#[test]
fn partial_enable_at_raised_pose_then_enable_error_refuses_rollback_holds_and_latches_recoverable_fault() {
    use armd::driver::{ArmDriverError, DisableKind, JointTarget, StoppedDisableGate};
    use armd::model::JointState;
    use std::sync::atomic::AtomicBool;

    struct EnableFinishFailWrapper {
        inner: DamiaoArmDriver<FakeBus>,
        bus: FakeBus,
        fail_read_err: bool,
        post_enable_armed: bool,
        in_post_enable: bool,
    }

    impl ArmDriver for EnableFinishFailWrapper {
        fn capabilities(&self) -> armd::driver::DriverCapabilities {
            self.inner.capabilities()
        }
        fn enable_state(&self) -> EnableState {
            self.inner.enable_state()
        }
        fn all_motors_confirmed_off_after_enable_writes(&self) -> bool {
            self.inner.all_motors_confirmed_off_after_enable_writes()
        }
        fn has_any_joint_enabled(&self) -> bool {
            self.inner.has_any_joint_enabled()
        }
        fn has_known_partial_enable(&self) -> bool {
            self.inner.has_known_partial_enable()
        }
        fn clock_now(&self) -> Instant {
            self.inner.clock_now()
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            self.inner.feedback_ages()
        }
        fn feedback_instants(&self) -> Vec<Option<Instant>> {
            self.inner.feedback_instants()
        }
        fn feedback_seqs(&self) -> Vec<u64> {
            self.inner.feedback_seqs()
        }
        fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
            self.inner.feedback_queue_bounds()
        }
        fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
            self.inner.feedback_receive_instants()
        }
        fn is_feedback_valid(&self) -> bool {
            self.inner.is_feedback_valid()
        }
        fn fresh_joint_positions_into(
            &self,
            joints: &[usize],
            max_age: Duration,
            out: &mut [f32],
        ) -> bool {
            self.inner.fresh_joint_positions_into(joints, max_age, out)
        }
        fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
            self.inner.read_cached_joint_states_into(out)
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            if self.in_post_enable && self.fail_read_err {
                return Err(ArmDriverError::HardwareFault(
                    "injected post-enable read failure".into(),
                ));
            }
            self.inner.read_joint_states()
        }
        fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            self.inner.write_joint_targets(targets)
        }
        fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
            if enabled && self.post_enable_armed {
                // Partially enable J1..J6 via the real Damiao handshake (J7 stays silent) so
                // motors 1..6 are physically enabled (`0x1`) at the raised pose and
                // `last_enable_write_seq[0..6]` is recorded, then return `Ok(())` so
                // `SupervisorInner::set_torque(true)` enters `enable_finish_inner` while
                // `safety.state()` is STILL `Idle`!
                let _ = self.inner.begin_torque(true)?;
                self.bus.with(|s| {
                    for j in 0..6 {
                        s.motors[j].state_override = None;
                        s.motors[j].enabled = true;
                        s.push_feedback(j);
                    }
                });
                let dl = self.inner.clock_now() + Duration::from_millis(5);
                let _ = self.inner.poll_torque(dl);
                self.inner.abort_enable();
                self.in_post_enable = true;
                return Ok(());
            }
            self.inner.set_torque(enabled)
        }
        fn disable_torque(&mut self, kind: DisableKind) -> Result<(), ArmDriverError> {
            self.inner.disable_torque(kind)
        }
        fn set_stopped_disable_gate(&mut self, gate: StoppedDisableGate) {
            self.inner.set_stopped_disable_gate(gate);
        }
        fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) {
            self.inner.set_abort_flag(flag);
        }
    }

    for fail_read_err in [true, false] {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        // Start with Unknown prior state at raised pose q = 0.40 (J1..J6 status 0x8, J7 silent).
        bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            s.motors[6].silent = true;
            for j in 0..6 {
                s.motors[j].enabled = false;
                s.motors[j].state_override = Some(0x8);
                s.motors[j].q = 0.40;
                s.push_feedback(j);
            }
        });
        dm.read_joint_states().unwrap();
        assert_eq!(dm.enable_state(), EnableState::Unknown);

        let wrapper = EnableFinishFailWrapper {
            inner: dm,
            bus: bus.clone(),
            fail_read_err,
            post_enable_armed: true,
            in_post_enable: false,
        };
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(wrapper));
        inner.lease_enabled = false;
        assert_eq!(*inner.safety.state(), SafetyState::Idle);

        bus.clear_log();
        let err = inner.set_torque(true).unwrap_err();
        assert!(
            matches!(err, SafetyError::FeedbackUnavailable),
            "fail_read_err={fail_read_err}: expected FeedbackUnavailable, got {err:?}"
        );

        // 1. ZERO disable frames transmitted to any enabled motor (J1..J6)!
        let disable_frames_to_enabled: Vec<u32> = bus
            .tx_log()
            .iter()
            .filter(|f| f.data == damiao::CMD_DISABLE && (1..=6).contains(&f.id))
            .map(|f| f.id)
            .collect();
        assert!(
            disable_frames_to_enabled.is_empty(),
            "fail_read_err={fail_read_err}: rollback while Idle at raised pose MUST NOT send disable frames to enabled motors, got {disable_frames_to_enabled:?}"
        );

        // 2. Enabled motors remain physically enabled & holding at raised pose!
        assert!(bus.with(|s| (0..6).all(|j| s.motors[j].enabled)));
        assert!(inner.is_torqued);
        assert!(!inner.disable_pending);
        for j in 0..6 {
            assert!(
                (inner.commanded_positions[j] - 0.40).abs() < 0.02,
                "fail_read_err={fail_read_err}: holding commanded_positions[{j}] must stay at raised pose ~0.40, got {}",
                inner.commanded_positions[j]
            );
        }

        // 3. A recoverable fault (`EnableRollbackHeld`, `recoverable() == true`) explains why
        //    rollback was refused and allows recovery through `arm.recover()`!
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(_)),
            "fail_read_err={fail_read_err}: must latch recoverable Fault, got {:?}",
            inner.safety.state()
        );
        assert_eq!(
            inner.safety.fault().map(|f| f.code),
            Some(FaultCode::EnableRollbackHeld),
            "fault code must be EnableRollbackHeld"
        );
        assert!(
            inner.safety.recoverable(),
            "EnableRollbackHeld must be recoverable through arm.recover()"
        );
        assert_eq!(
            inner.safety.fault().map(|f| f.recoverable),
            Some(true),
            "observation fault.recoverable must be true"
        );

        // 4. Holding targets are sent to all motors on tick() while held, and recovery via
        //    `clear_fault()` succeeds AT THE RAISED POSE (keeping motors holding, partial enable
        //    known) once all 7 motors respond with fresh feedback!
        if !fail_read_err {
            bus.with(|s| {
                s.motors[6].silent = false;
                s.rx.clear();
                s.pending.clear();
                for j in 0..7 {
                    s.motors[j].state_override = None;
                }
                s.push_all_feedback();
            });
            bus.clear_log();
            clock.advance(Duration::from_millis(10));
            inner.tick(0.02);
            let hold_frames: Vec<u32> = bus
                .tx_log()
                .iter()
                .filter(|f| f.id >= 0x101 && f.id <= 0x107)
                .map(|f| f.id)
                .collect();
            assert_eq!(
                hold_frames.len(),
                7,
                "tick() while in EnableRollbackHeld must send hold targets to all 7 motors, got {hold_frames:?}"
            );
            assert!(bus.with(|s| (0..6).all(|j| s.motors[j].enabled)));

            // `clear_fault()` (arm.recover() step 1) succeeds while raised without dropping torque!
            clock.advance(Duration::from_millis(10));
            bus.with(|s| s.push_all_feedback());
            let cleared = inner
                .clear_fault()
                .expect("EnableRollbackHeld must be clearable via arm.recover() while raised");
            assert_eq!(cleared, SafetyState::Idle);
            assert!(inner.is_torqued, "arm stays holding after clear_fault at raised pose");
        }
    }
}

/// Round 6 (Decision 1): enforce at source level that ONLY thermal trip and process-exit/panic
/// paths construct `DisableProtective`, and ZERO already-off bypass mechanisms remain in `src/`.
#[test]
fn only_thermal_and_process_exit_construct_disable_protective_and_no_already_off_bypasses_exist() {
    use std::path::{Path, PathBuf};

    fn collect_rs(dir: &Path, out: &mut Vec<PathBuf>) {
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                collect_rs(&path, out);
            } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }

    /// Returns `(1-based line_no, line)` for production lines in `src/`, skipping only
    /// `#[cfg(test)]`-attributed items (tracked by brace depth) so production code following
    /// an inline `#[cfg(test)]` helper is still inspected.
    fn prod_lines(src: &str) -> Vec<(usize, &str)> {
        let mut out = Vec::new();
        let mut pending_cfg_test = false;
        let mut skip_depth: usize = 0;
        let mut in_skip = false;
        for (idx, line) in src.lines().enumerate() {
            let trimmed = line.trim();
            if in_skip {
                for ch in line.chars() {
                    if ch == '{' {
                        skip_depth += 1;
                    } else if ch == '}' {
                        skip_depth = skip_depth.saturating_sub(1);
                    }
                }
                if skip_depth == 0 {
                    in_skip = false;
                }
                continue;
            }
            if trimmed == "#[cfg(test)]" {
                pending_cfg_test = true;
                continue;
            }
            if pending_cfg_test {
                if trimmed.starts_with('#') || trimmed.starts_with("//") || trimmed.is_empty() {
                    continue;
                }
                let opens = line.chars().filter(|&c| c == '{').count();
                let closes = line.chars().filter(|&c| c == '}').count();
                if opens > closes {
                    skip_depth = opens - closes;
                    in_skip = true;
                }
                // Single-line item or `mod tests;` statement ends immediately.
                pending_cfg_test = false;
                continue;
            }
            out.push((idx + 1, line));
        }
        out
    }

    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let src_dir = manifest_dir.join("src");
    let mut files = Vec::new();
    collect_rs(&src_dir, &mut files);
    files.sort();

    let forbidden = [
        "EnableRollbackProven",
        "prior_proven_off",
        "motor_freshly_physically_off",
        "all_motors_watchdog_expired_or_disabled",
    ];
    let mut protective_constructions: Vec<(String, usize, String)> = Vec::new();

    for path in &files {
        let rel = path
            .strip_prefix(manifest_dir)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/");
        let full = std::fs::read_to_string(path).unwrap();
        let prod = prod_lines(&full);

        for needle in forbidden {
            assert!(
                !full.contains(needle),
                "forbidden already-off symbol `{needle}` found in {rel}"
            );
        }

        for (pos, &(line_no, line)) in prod.iter().enumerate() {
            let trimmed = line.trim();
            if trimmed.starts_with("//") {
                continue;
            }
            if trimmed.contains("DisableProtectiveReason::Thermal")
                || trimmed.contains("DisableProtectiveReason::ProcessExit")
            {
                protective_constructions.push((rel.clone(), line_no, trimmed.to_string()));
            }
            if trimmed.contains("DisableProtective {")
                && !trimmed.starts_with("DisableProtective {")
                && !trimmed.contains("=>")
                && !trimmed.contains("matches!(")
                && !trimmed.contains("DisableProtective { .. }")
            {
                // Pure pass-through mappings from `DisableKind::DisableProtective { reason }`
                // into driver-internal enums (`TxKind` / `SendGate`) are allowed ONLY in
                // `src/driver.rs` and `src/sim_driver.rs`.
                let is_allowed_passthrough = (rel == "src/driver.rs"
                    && trimmed == "TxKind::DisableProtective { reason }")
                    || (rel == "src/sim_driver.rs"
                        && trimmed == "SendGate::DisableProtective { reason }");
                if !is_allowed_passthrough && !trimmed.contains("DisableProtectiveReason::") {
                    // Multiline construction; the `reason:` line below carries the variant.
                    let next = prod.get(pos + 1).map(|(_, l)| l.trim()).unwrap_or("");
                    assert!(
                        next.contains("DisableProtectiveReason::Thermal")
                            || next.contains("DisableProtectiveReason::ProcessExit"),
                        "unexpected DisableProtective construction at {rel}:{line_no}: `{trimmed}` / `{next}`"
                    );
                }
            }
        }
    }

    // Exact exhaustive allowlist of every production `DisableProtective` construction in `src/`:
    // 1. Thermal trip (`src/supervisor/motion.rs`)
    // 2. Process exit / panic teardown (`src/driver.rs`, `src/supervisor.rs`, `src/supervisor/driver_transitions.rs` × 2)
    let mut actual: Vec<(String, &'static str)> = protective_constructions
        .iter()
        .map(|(rel, _, line)| {
            let kind = if line.contains("DisableProtectiveReason::Thermal") {
                "Thermal"
            } else {
                "ProcessExit"
            };
            (rel.clone(), kind)
        })
        .collect();
    actual.sort();

    let mut expected: Vec<(String, &'static str)> = vec![
        ("src/driver.rs".to_string(), "ProcessExit"),
        ("src/supervisor.rs".to_string(), "ProcessExit"),
        ("src/supervisor/driver_transitions.rs".to_string(), "ProcessExit"),
        ("src/supervisor/driver_transitions.rs".to_string(), "ProcessExit"),
        ("src/supervisor/motion.rs".to_string(), "Thermal"),
    ];
    expected.sort();

    assert_eq!(
        actual, expected,
        "ONLY thermal trip and process-exit/panic paths may construct DisableProtective; found: {protective_constructions:#?}"
    );
}

/// Round 6 (Decision 2): `enable_state()` and `SupervisorInner::sync_torque_state_after_stop()`
/// share ONE helper and NEVER conclude "torque off, stop holding" from feedback received before
/// ANY motor's latest enable write (including deferred and retried enable writes).
#[test]
fn stopped_reconciliation_never_trusts_pre_write_off_feedback_after_deferred_or_retried_enable() {
    use armd::driver::{TorqueProgress, TxBudget};

    // Case A: Deferred enable write (TX budget = 1 frame/tick) moves motor 2's boundary.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        // Limit TX to 1 frame per tick so motors 2..7 are deferred past tick 1.
        dm.set_tx_budget(TxBudget {
            max_frames: 1,
            max_duration: Duration::from_millis(10),
            ..TxBudget::default()
        });
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.40;
                m.enabled = false;
                m.drop_torque_frames = 10; // keep motors reporting 0x0 on enable frames
            }
            s.push_all_feedback();
        });
        dm.read_joint_states().unwrap();

        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm));
        inner.lease_enabled = false;

        // Tick 1: begin_torque(true) sends CMD_ENABLE to motor 1 only; motors 2..7 deferred.
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert!(matches!(
                drv.begin_torque(true).unwrap(),
                TorqueProgress::Pending { .. }
            ));
        }

        // All 7 motors report fresh 0x0 BEFORE motor 2's deferred CMD_ENABLE transmits!
        clock.advance(Duration::from_millis(5));
        bus.with(|s| s.push_all_feedback());

        // Tick 2: poll_torque drains those 0x0 replies, then sends deferred CMD_ENABLE to motor 2.
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let dl = clock.now() + Duration::from_millis(5);
            assert!(matches!(
                drv.poll_torque(dl).unwrap(),
                TorqueProgress::Pending { .. }
            ));
            // Abort enable + drop budget back to normal so hold targets can transmit cleanly.
            drv.abort_enable();
        }
        // Simulate E-stop with torque on (arm holding at raised pose) right after motor 2's deferred write.
        inner.is_torqued = true;
        inner.disable_pending = false;
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        // `sync_torque_state_after_stop()` runs inside `tick()`: motor 2 only has PRE-write 0x0,
        // so enable_state MUST be Unknown and `is_torqued` MUST stay true (arm keeps holding!).
        {
            let drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert_eq!(drv.enable_state(), EnableState::Unknown);
            assert!(!drv.all_motors_confirmed_off_after_enable_writes());
        }
        bus.clear_log();
        inner.tick(0.02);
        assert!(
            inner.is_torqued,
            "sync_torque_state_after_stop MUST NOT drop holding from pre-deferred-write 0x0"
        );
        assert!(
            bus.tx_log().iter().any(|f| f.id >= 0x101 && f.id <= 0x107),
            "arm must keep transmitting hold targets while motor 2's enable write is unconfirmed"
        );

        // Now push a fresh post-write 0x0 reply from motor 2 (all other motors already had post-write 0x0):
        // reconciliation now sees EVERY motor off strictly after its last enable write!
        clock.advance(Duration::from_millis(5));
        bus.with(|s| {
            s.motors[1].enabled = false;
            s.push_feedback(1);
        });
        inner.tick(0.02);
        {
            let drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert!(drv.all_motors_confirmed_off_after_enable_writes());
            assert_eq!(drv.enable_state(), EnableState::Disabled);
        }
        assert!(
            !inner.is_torqued,
            "once every motor reports 0x0 strictly after its last enable write, reconciliation sets is_torqued=false"
        );
    }

    // Case B: Retried enable write (`Service::Retry` at +25 ms) moves write boundary for all unconfirmed motors.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.40;
                m.enabled = false;
                m.drop_torque_frames = 10;
            }
            s.push_all_feedback();
        });
        dm.read_joint_states().unwrap();

        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm));
        inner.lease_enabled = false;

        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert!(matches!(
                drv.begin_torque(true).unwrap(),
                TorqueProgress::Pending { .. }
            ));
        }
        // At +20 ms, all 7 motors reply 0x0 (after initial write, BEFORE +25 ms retry).
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let dl = clock.now() + Duration::from_millis(5);
            assert!(matches!(drv.poll_torque(dl).unwrap(), TorqueProgress::Pending { .. }));
        }
        // At +30 ms, poll_torque retries CMD_ENABLE to all 7 motors (updating every motor's boundary!).
        clock.advance(Duration::from_millis(10));
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let dl = clock.now() + Duration::from_millis(5);
            assert!(matches!(drv.poll_torque(dl).unwrap(), TorqueProgress::Pending { .. }));
            drv.abort_enable();
        }
        inner.is_torqued = true;
        inner.disable_pending = false;
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        // Even if motors 1..6 emit fresh post-retry 0x0 while motor 7 still has ONLY pre-retry 0x0:
        // enable_state MUST stay Unknown and `sync_torque_state_after_stop()` MUST keep holding!
        clock.advance(Duration::from_millis(5));
        bus.with(|s| {
            for j in 0..6 {
                s.push_feedback(j);
            }
        });
        inner.tick(0.02);
        {
            let drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert_eq!(drv.enable_state(), EnableState::Unknown);
            assert!(!drv.all_motors_confirmed_off_after_enable_writes());
        }
        assert!(
            inner.is_torqued,
            "pre-retry 0x0 on motor 7 must prevent stopped reconciliation from dropping holding"
        );

        // Once motor 7 ALSO emits post-retry 0x0 -> reconciliation sets is_torqued = false!
        clock.advance(Duration::from_millis(5));
        bus.with(|s| s.push_feedback(6));
        inner.tick(0.02);
        {
            let drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert!(drv.all_motors_confirmed_off_after_enable_writes());
            assert_eq!(drv.enable_state(), EnableState::Disabled);
        }
        assert!(!inner.is_torqued);
    }
}
