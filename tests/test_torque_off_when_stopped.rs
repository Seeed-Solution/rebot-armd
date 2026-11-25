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

fn enable_torque_admitted(inner: &mut SupervisorInner) -> Result<SafetyState, SafetyError> {
    let stamp = inner.current_admission_stamp();
    let admitted = inner.check_admission(stamp)?;
    SupervisorInner::enable_torque(admitted)
}

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
    inner.register_live_connection(1).unwrap();
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
    enable_torque_admitted(&mut inner).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("test".into()));

    // At 0.0 (home, 0.50 rad away from custom rest -0.50) -> refused naming joint_2 with rest -0.50.
    assert!(matches!(
        inner.disable_torque(),
        Err(SafetyError::TorqueOffWouldDrop {
            ref joint,
            position: Some(p),
            rest: Some(r),
            ..
        }) if joint.as_deref() == Some("joint_2") && p.abs() < 1e-4 && (r + 0.50).abs() < 1e-4
    ));

    // At -0.42 (0.08 rad from -0.50, within custom 0.10 rad tolerance) -> allowed, latch kept.
    inner.measured_positions[1] = -0.42;
    assert!(matches!(inner.disable_torque(), Ok(SafetyState::Fault(_))));
    assert!(!inner.is_torqued);
}

#[test]
fn legacy_fault_at_rest_allows_disable_keeps_latch_and_ignores_gripper() {
    let (mut inner, bus, clock) = make_driver_rig();
    enable_torque_admitted(&mut inner).unwrap();

    // Gripper wide open (-3.0 rad), all arm joints within 0.05 rad of rest (0.0).
    // S2: supply K=3 consecutive causal samples spanning >= 2 ticks (>= 20 ms) at rest.
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        for j in 0..6 {
            s.motors[j].q = 0.03;
        }
        s.motors[6].q = -3.0;
    });
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    assert!((inner.measured_positions[6] + 3.0).abs() < 0.01, "gripper must be measured open");
    assert!((inner.measured_positions[1] - 0.03).abs() < 0.01, "J2 must be measured at 0.03 rad");

    inner.safety.trigger_fault_coded(
        FaultCode::DriverWriteFailed,
        FaultReason::Other("simulated fault".into()),
    );
    // Clear post-motion settle (> 20 ms) and establish K=3 stopped rest samples at 50 Hz (20 ms spacing).
    clock.advance(Duration::from_millis(25));
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(20));
        }
        bus.with(|s| {
            s.rx.clear();
            s.push_all_feedback();
        });
        inner.tick(0.02);
    }
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert!(inner.is_torqued);
    let epoch_before = inner.safety.epoch_id();
    let kinds_before = inner.safety.latched_fault_kinds();
    let fault_before = inner.safety.fault();

    let state = inner.disable_torque().expect("disable at rest in Fault must succeed");
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

    // Gripper wide open (-3.0 rad), arm joints within 0.05 rad of rest (S2: K=3 across >= 2 ticks).
    r.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        for j in 0..6 {
            s.motors[j].q = -0.04;
        }
        s.motors[6].q = -3.0;
    });
    for _ in 0..3 {
        r.step_with_feedback();
    }
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
    let sdk_err = sup.disable_torque().await.unwrap_err();
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
    inner.register_live_connection(1).unwrap();
    enable_torque_admitted(&mut inner).unwrap();
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
    enable_torque_admitted(&mut inner).unwrap();
    // Arm is physically at rest (0.0), enter Fault.
    inner.safety.trigger_fault_coded(
        FaultCode::BusLoss,
        FaultReason::BusLoss,
    );
    // Silence motors and advance driver clock beyond 100 ms: M3 defers and probes within the 1 s
    // lifetime, then expires with TORQUE_OFF_WOULD_DROP and 0 CMD_DISABLE frames when motors stay silent.
    bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    clock.advance(Duration::from_millis(101));
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback)
    );
    bus.clear_log();
    // R12 B1: Legacy disable waiting for K=3 defers per-release without blocking under the driver
    // mutex; step legacy releases across the 1 s lifetime -> expires with 0 CMD_DISABLE frames
    // while `is_torqued` stays true and holds continue.
    let res = inner.disable_torque();
    assert!(res.is_ok(), "deferred legacy disable must not block/fail immediately: {res:?}");
    assert!(inner.disable_pending);
    assert_eq!(inner.torque_intent, armd::TorqueIntent::On);
    for _ in 0..55 {
        if !inner.disable_pending {
            break;
        }
        clock.advance(Duration::from_millis(20));
        inner.tick(0.02);
    }
    assert!(!inner.disable_pending, "silent legacy disable must expire after 1 s lifetime");
    assert_eq!(
        bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0
    );
    assert!(inner.is_torqued);

    // Once motors reply again and K=3 fresh causal samples arrive after an observed-empty queue
    // boundary (<= 100 ms, spanning >= 20 ms per S2), disable succeeds and keeps Fault latch.
    bus.with(|s| {
        for m in &mut s.motors {
            m.silent = false;
        }
    });
    inner.tick(0.02); // observe empty RX queue at current time
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    assert!(matches!(inner.disable_torque(), Ok(SafetyState::Fault(_))));
    assert!(!inner.is_torqued);

    // 2. Executor owner with stale/silent feedback (> 100 ms): M3 defers into AwaitRestGate probe
    //    loop and expires at 1 s with Outcome::Failed("TORQUE_OFF_WOULD_DROP") and 0 CMD_DISABLE frames.
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
    r.bus.clear_log();
    r.submit_transition(TransitionOp::TorqueOff);
    let rep = r.step();
    assert!(!r.fx.core.hardware_feedback_fresh(Duration::from_millis(100)));
    assert!(
        matches!(rep.command, Some((_, Ok(_)))),
        "M3: drained stale bus must defer into probe loop, got {:?}",
        rep.command
    );
    let mut expired = false;
    for _ in 0..60 {
        let rep_step = r.step();
        if let Some((_, outcome)) = rep_step.transition_ended {
            assert_eq!(outcome, Outcome::Failed("TORQUE_OFF_WOULD_DROP"));
            expired = true;
            break;
        }
    }
    assert!(expired, "M3: silent bus must expire at 1 s lifetime");
    assert_eq!(
        r.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0
    );
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
    enable_torque_admitted(&mut inner).unwrap();

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
        inner.disable_torque(),
        Err(SafetyError::TorqueOffWouldDrop { ref joint, .. }) if joint.as_deref() == Some("joint_1")
    ));
    assert!(inner.is_torqued);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before);
    assert_eq!(inner.safety.fault(), fault_before);

    // At rest + fresh feedback in pure E-Stop (R10: clear post-motion settle > 20 ms, then K=3 spaced rest samples):
    // First inject a disable TX failure while in EStop: sticky disable stays pending and
    // SafetyState::EStop, fault record, and epoch must NOT be mutated.
    bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
        s.motors[0].q = 0.0;
    });
    clock.advance(Duration::from_millis(25));
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    bus.with(|s| s.tx_error = true);
    assert!(inner.disable_torque().is_err());
    assert_eq!(*inner.safety.state(), SafetyState::EStop, "failed disable must not overwrite E-Stop latch");
    assert_eq!(inner.safety.epoch_id(), epoch_before, "failed disable must not advance epoch");
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before, "failed disable must not mutate fault record");
    assert_eq!(inner.safety.fault(), fault_before, "failed disable must not mutate fault detail");
    assert!(inner.disable_pending, "sticky disable_pending must stay set on failure");

    // First verify confirmed disable at rest in pure E-Stop keeps exact EStop latch, fault
    // record, and epoch intact (`TxKind::DisableRequested` bypasses the stop gate).
    bus.with(|s| {
        s.tx_error = false;
        s.rx.clear();
        s.pending.clear();
    });
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(20));
        }
        bus.with(|s| {
            s.rx.clear();
            s.push_all_feedback();
        });
        inner.tick(0.02);
    }
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    let st = inner.disable_torque().expect("disable at rest in E-Stop must succeed");
    assert_eq!(st, SafetyState::EStop);
    assert_eq!(*inner.safety.state(), SafetyState::EStop);
    assert_eq!(inner.safety.epoch_id(), epoch_before, "confirmed disable must not advance epoch");
    assert_eq!(inner.safety.latched_fault_kinds(), kinds_before, "confirmed disable must not mutate fault record");
    assert_eq!(inner.safety.fault(), fault_before, "confirmed disable must not mutate fault detail");
    assert!(!inner.is_torqued);

    // Next verify silent/stale deferred disable in E-Stop expires cleanly at 1 s without
    // transmitting CMD_DISABLE or dropping the E-Stop latch (R12 B1).
    let (mut inner_s, bus_s, clock_s) = make_driver_rig();
    enable_torque_admitted(&mut inner_s).unwrap();
    inner_s.apply_stop_policy(StopPolicy::EStopLatch);
    bus_s.with(|s| {
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    clock_s.advance(Duration::from_millis(150));
    bus_s.clear_log();
    let _ = inner_s.disable_torque();
    for _ in 0..55 {
        if !inner_s.disable_pending {
            break;
        }
        clock_s.advance(Duration::from_millis(20));
        inner_s.tick(0.02);
    }
    assert!(!inner_s.disable_pending, "silent EStop disable must expire at 1 s");
    assert!(inner_s.is_torqued);
    assert!(
        inner_s.safety.has_latched_fault_kind(armd::safety::FaultKind::Estop),
        "E-Stop latch must remain recorded across silent disable expiry"
    );
    assert_eq!(spec_count_damiao_disables(&bus_s), 0, "silent expiry must send 0 CMD_DISABLE frames");
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

    // Move to rest with K=3 fresh feedback samples -> allowed; EStop latch, fault record, and epoch kept.
    r.bus.with(|s| {
        s.motors[1].q = 0.0;
    });
    for _ in 0..3 {
        r.step_with_feedback();
    }
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
    let (mut inner, bus, clock) = make_driver_rig();
    enable_torque_admitted(&mut inner).unwrap();
    bus.with(|s| s.tx_step = Duration::from_millis(2));

    // Run ticks at rest (spanning >= 20 ms for S2 K=3 rest window): write_joint_targets defers ->
    // latches DriverWriteFailed; subsequent ticks keep failing write_estop_hold / re-latching fault.
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
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
    let state = inner.disable_torque().expect("disable at rest must beat deferral fault loop");
    assert!(matches!(state, SafetyState::Fault(_)));
    for _ in 0..5 {
        if !inner.disable_pending {
            break;
        }
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        inner.safety.trigger_fault_coded(
            FaultCode::DriverWriteFailed,
            FaultReason::Other("TX deferral re-latch".into()),
        );
    }
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
    enable_torque_admitted(&mut inner).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::Other, FaultReason::Other("fault".into()));
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::NoRestingPoseConfigured)
    );
    assert!(matches!(
        inner.disable_torque(),
        Err(SafetyError::TorqueOffWouldDrop {
            joint: None,
            ref reason,
            ..
        }) if reason.as_deref() == Some("no_resting_pose")
    ));
    assert!(inner.is_torqued);

    // 2. Executor owner
    let (mut inner, bus, clock) = make_uncalibrated();
    inner.register_live_connection(1).unwrap();
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
    // 1. M2: Damiao rest gate covers EVERY motor that receives CMD_DISABLE, J7 included.
    //    Silent J7 (> 150 ms) MUST block Damiao rest-pose admission until J7 provides K=3
    //    fresh valid samples, at which point ANY finite jaw position (no pose tolerance) passes.
    let (mut inner, bus, clock) = make_driver_rig();
    enable_torque_admitted(&mut inner).unwrap();
    inner.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[6].silent = true;
    });
    clock.advance(Duration::from_millis(150));
    inner.tick(0.02); // observe empty RX queue at current time
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| {
            for j in 0..6 {
                s.push_feedback(j);
            }
        });
        inner.tick(0.02);
    }
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
        "M2: silent J7 (receives CMD_DISABLE) must block Damiao rest-pose gate until K=3 fresh J7 samples arrive"
    );

    // Unmute J7 at an arbitrary non-zero jaw position (1.75 rad, well outside arm tolerance):
    // K=3 fresh J7 samples must confirm the gate because J7 rest is any finite jaw position.
    bus.with(|s| {
        s.motors[6].silent = false;
        s.motors[6].q = 1.75;
    });
    for _ in 0..3 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Ok(()),
        "M2: J7 at any finite jaw position with K=3 fresh samples must pass Damiao rest-pose gate"
    );

    // If an arm joint (J3) goes stale (> 100 ms), admission MUST refuse.
    bus.with(|s| s.motors[2].silent = true);
    clock.advance(Duration::from_millis(120));
    bus.with(|s| {
        for j in [0, 1, 3, 4, 5, 6] {
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
    enable_torque_admitted(&mut mock).unwrap();
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
    enable_torque_admitted(&mut inner).unwrap();
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
    // K=3 new causal rest frames arriving across >= 2 ticks (>= 20 ms) pass admission (S2).
    for _ in 0..3 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    assert_eq!(inner.may_disable_torque_while_stopped(), Ok(()));
}

#[test]
fn fresh_driver_timestamps_never_vouch_for_stale_measured_positions_and_zero_offset_applies() {
    // 1. Reproduce the clear_fault / silent-gripper split: supervisor.measured_positions is stale
    // at rest (0.0), while the driver's fresh J1..J6 feedback (read during clear_fault) shows J2
    // raised to 0.45 rad. The rest check MUST use the fresh driver positions and refuse.
    let (mut inner, bus, clock) = make_driver_rig();
    enable_torque_admitted(&mut inner).unwrap();
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
    let err = inner.disable_torque().unwrap_err();
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

    // Move J2 back to rest with K=3 fresh J1..J6 feedback samples while J7 is STILL silent:
    // M2 requires K=3 fresh samples on every motor receiving CMD_DISABLE (J7 included).
    inner.feedback_numeric_valid = false;
    bus.with(|s| {
        s.motors[1].q = 0.0;
    });
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| {
            for j in 0..6 {
                s.push_feedback(j);
            }
        });
        let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
        drv.begin_control_release(100 + step as u64);
        let _ = drv.read_joint_states();
    }
    assert_eq!(
        inner.may_disable_torque_while_stopped(),
        Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
        "M2: silent J7 must keep Damiao rest gate unconfirmed"
    );
    // Unmute J7 at arbitrary jaw angle (-2.0 rad) with K=3 fresh samples while
    // `inner.feedback_numeric_valid` remains false -> admission succeeds from fresh driver samples.
    bus.with(|s| {
        s.motors[6].silent = false;
        s.motors[6].q = -2.0;
    });
    for step in 0..3 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| s.push_all_feedback());
        let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
        drv.begin_control_release(110 + step as u64);
        let _ = drv.read_joint_states();
    }
    assert_eq!(inner.may_disable_torque_while_stopped(), Ok(()));

    // 2. ZeroOffsetDriver must apply joint-zero offsets in fresh_joint_positions_into.
    let (mut inner_z, bus_z, clock_z) = make_driver_rig();
    let mut offsets = vec![0.0; 7];
    offsets[1] = 0.40;
    inner_z.set_joint_zero_offsets(&offsets).unwrap();
    enable_torque_admitted(&mut inner_z).unwrap();
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
    // Motor raw q = 0.40 with offset 0.40 => logical q = 0.0 (at rest, K=3 samples spanning >= 20 ms).
    bus_z.with(|s| {
        s.rx.clear();
        s.pending.clear();
        s.motors[1].q = 0.40;
    });
    for step in 0..3 {
        if step > 0 {
            clock_z.advance(Duration::from_millis(10));
        }
        bus_z.with(|s| s.push_all_feedback());
        inner_z.tick(0.02);
    }
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
    enable_torque_admitted(&mut inner_t).unwrap();
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
    enable_torque_admitted(&mut inner_s).unwrap();
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
    enable_torque_admitted(&mut inner_p).unwrap();
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
    enable_torque_admitted(&mut inner).unwrap();
    inner.apply_stop_policy(StopPolicy::EStopLatch);
    let epoch_before = inner.safety.epoch_id();

    // Authorize a stopped disable at rest (S2: K=3 spaced rest samples) whose TX fails ->
    // sets disable_pending with DisableRequested.
    bus.with(|s| {
        s.follow_commands = false;
    });
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    bus.with(|s| {
        s.tx_error = true;
        s.push_all_feedback();
    });
    assert!(inner.disable_torque().is_err());
    assert!(inner.disable_pending);
    assert!(matches!(
        inner.active_disable_kind(),
        Some(armd::driver::DisableKind::DisableRequested { .. })
    ));

    // Within the 1 s lifetime, periodic retry (>= 250 ms since last attempt) still tries to complete it.
    // Refresh the empty-queue RX bound + K=3 rest window before crossing 100 ms so feedback stays fresh.
    clock.advance(Duration::from_millis(200));
    inner.last_disable_attempt = Some(Instant::now());
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    clock.advance(Duration::from_millis(30));
    inner.last_disable_attempt = Some(Instant::now() - Duration::from_millis(300));
    bus.with(|s| {
        s.tx_log.clear();
        s.push_all_feedback();
    });
    inner.tick(0.02);
    assert!(inner.disable_pending, "still pending while TX fails within 1 s lifetime");

    // Once the single 1 s lifetime elapses (checked inside `send`), even with fresh K=3 rest feedback,
    // no disable frames are sent, disable_pending clears, is_torqued and EStop latch + epoch stay intact.
    clock.advance(Duration::from_millis(720));
    inner.last_disable_attempt = Some(Instant::now());
    for step in 0..3 {
        if step > 0 {
            clock.advance(Duration::from_millis(10));
        }
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    clock.advance(Duration::from_millis(25)); // now > 1 s, feedback & rest window fresh
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
        inner.disable_torque(),
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
        enable_torque_admitted(&mut inner).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        // Start a DisableRequested handshake at rest (S2: K=3 spaced rest samples) while motor 2
        // stays silent (never replies state-0), so DamiaoTorque::Disable waits for the 25 ms retry.
        bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
        });
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
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
            PoseCase::Raised => {
                assert!(
                    matches!(poll_res, Err(armd::driver::ArmDriverError::TorqueOffWouldDrop)),
                    "legacy handshake resend (Raised) must refuse immediately with TorqueOffWouldDrop, got {poll_res:?}"
                );
                assert_eq!(
                    disables, 0,
                    "legacy handshake resend (Raised) must transmit ZERO disable frames"
                );
            }
            PoseCase::Stale => {
                // M3: Stale evidence defers within 1 s with ZERO disable frames and refuses on 1 s expiry.
                assert!(
                    matches!(poll_res, Ok(armd::driver::TorqueProgress::Pending { .. })),
                    "legacy handshake resend (Stale) must defer within 1 s lifetime, got {poll_res:?}"
                );
                assert_eq!(
                    disables, 0,
                    "legacy handshake resend (Stale) must transmit ZERO disable frames"
                );
                clock.advance(Duration::from_secs(1));
                let exp_res = {
                    let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
                    let dl = drv.clock_now() + Duration::from_millis(5);
                    drv.poll_torque(dl)
                };
                assert!(
                    matches!(exp_res, Err(armd::driver::ArmDriverError::DisableRequestExpired)),
                    "legacy handshake resend (Stale) must refuse on 1 s expiry, got {exp_res:?}"
                );
                assert_eq!(
                    count_damiao_disables(&bus),
                    0,
                    "legacy handshake resend (Stale) past 1 s expiry must transmit ZERO disable frames"
                );
            }
        }
        let _ = started_at;
    }

    // Also verify legacy handshake resend past 1 s total request lifetime sends 0 frames even at rest.
    {
        let (mut inner, bus, clock) = make_driver_rig();
        enable_torque_admitted(&mut inner).unwrap();
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
        for step in 0..2 {
            clock.advance(Duration::from_millis(if step == 0 { 15 } else { 20 }));
            if step == 1 {
                bus.with(|s| s.motors[1].torque_reply_delay = Duration::from_secs(10));
            }
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let p = drv
                .begin_disable_torque(DisableKind::DisableRequested {
                    started_at: t_req,
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
        });
        // 1 release clears the 20 ms post-motion settle after `arm_now`, then 3 releases fill K=3.
        for _ in 0..4 {
            r.step_with_feedback();
        }
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
            PoseCase::Raised => {
                assert_eq!(
                    rep.transition_ended.map(|(_, o)| o),
                    Some(Outcome::Failed("TORQUE_OFF_WOULD_DROP")),
                    "RT poll retry (Raised) must fail immediately with TORQUE_OFF_WOULD_DROP"
                );
                assert!(!r.fx.core.disable_pending);
                assert_eq!(
                    disables, 0,
                    "RT poll retry (Raised) must transmit ZERO disable frames"
                );
            }
            PoseCase::Stale => {
                // M3: Stale evidence defers within 1 s with ZERO disable frames and expires at 1 s.
                assert_eq!(
                    rep.transition_ended, None,
                    "RT poll retry (Stale) must defer within 1 s lifetime"
                );
                assert_eq!(
                    disables, 0,
                    "RT poll retry (Stale) must transmit ZERO disable frames"
                );
                r.clock.advance(Duration::from_secs(1));
                let rep_exp = r.step();
                assert_eq!(
                    rep_exp.transition_ended.map(|(_, o)| o),
                    Some(Outcome::Failed("TORQUE_OFF_WOULD_DROP")),
                    "RT poll retry (Stale) must fail at 1 s expiry with TORQUE_OFF_WOULD_DROP"
                );
                assert!(!r.fx.core.disable_pending);
                assert_eq!(
                    count_damiao_disables(&r.bus),
                    0,
                    "RT poll retry (Stale) past 1 s expiry must transmit ZERO disable frames"
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
            enable_torque_admitted(&mut inner).unwrap();
            bus.with(|s| {
                s.follow_commands = false;
                s.rx.clear();
                s.pending.clear();
            });
            if !started_before_stop {
                inner.apply_stop_policy(StopPolicy::EStopLatch);
                // Clear post-motion settle (> 20 ms) and build K=3 rest samples while stopped.
                clock.advance(Duration::from_millis(25));
                for step in 0..3 {
                    if step > 0 {
                        clock.advance(Duration::from_millis(10));
                    }
                    bus.with(|s| s.push_all_feedback());
                    inner.tick(0.02);
                }
            } else {
                bus.with(|s| s.push_all_feedback());
                inner.tick(0.02);
                // Under B5, operational `disable_torque()` also requires K=3 post-motion rest
                // samples before attempting `CMD_DISABLE`. Establish K=3 directly on the driver
                // after the > 20 ms post-motion settle so the subsequent `tx_error` fails the
                // `CMD_DISABLE` transmission itself (leaving `disable_pending == true`).
                clock.advance(Duration::from_millis(25));
                {
                    let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
                    let _ = drv.read_joint_states();
                    for step in 0..3 {
                        if step > 0 {
                            clock.advance(Duration::from_millis(12));
                        }
                        bus.with(|s| s.push_all_feedback());
                        drv.begin_control_release(100 + step as u64);
                        let _ = drv.read_joint_states();
                    }
                }
            }
            bus.with(|s| {
                s.tx_error = true;
                s.rx.clear();
                s.pending.clear();
                s.push_all_feedback();
            });
            assert!(inner.disable_torque().is_err());
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
                    if started_before_stop {
                        // Establish post-motion K=3 rest window after entering E-stop before retry fires.
                        clock.advance(Duration::from_millis(25));
                        for step in 0..3 {
                            if step > 0 {
                                clock.advance(Duration::from_millis(10));
                            }
                            bus.with(|s| s.push_all_feedback());
                            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
                            drv.begin_control_release(200 + step as u64);
                            let _ = drv.read_joint_states();
                        }
                    } else {
                        clock.advance(Duration::from_millis(30));
                        bus.with(|s| s.push_all_feedback());
                    }
                }
                PoseCase::Raised => {
                    clock.advance(Duration::from_millis(30));
                    bus.with(|s| {
                        s.motors[1].q = 0.40;
                        s.push_all_feedback();
                    });
                }
                PoseCase::Stale => {
                    bus.with(|s| {
                        for m in &mut s.motors {
                            m.silent = true;
                        }
                    });
                    clock.advance(Duration::from_millis(110));
                }
            }
            inner.tick(0.02);
            if pose == PoseCase::Stale {
                // R12 B1: Stale periodic retry defers per-release without blocking under mutex;
                // step through the 1 s request lifetime so it expires with 0 CMD_DISABLE frames.
                for _ in 0..55 {
                    if !inner.disable_pending {
                        break;
                    }
                    clock.advance(Duration::from_millis(20));
                    inner.tick(0.02);
                }
            }
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
    // 4. Failed enable (BOTH previously-disabled AND Unknown prior state) × {AtRest, Raised, Stale}
    //    §2 / §4 / §5: Failed enable NEVER rolls back automatically at any pose (0 disable frames),
    //    enters `FailedHold` with measured-position holds, and latches non-recoverable `ENABLE_FAILED`.
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
            // - prior_all_off = true: verified disable transaction -> EnableState::Disabled.
            // - prior_all_off = false: J1..J6 freshly report status 0x8 and J7 is silent -> Unknown.
            // Both MUST enter `FailedHold` + `ENABLE_FAILED` with ZERO automatic rollback!
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
                if prior_all_off {
                    assert_eq!(drv.enable_state(), EnableState::Disabled);
                } else {
                    assert_eq!(drv.enable_state(), EnableState::Unknown);
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
            let err = enable_torque_admitted(&mut inner).unwrap_err();
            assert!(
                matches!(err, SafetyError::ClearFaultRejected(_)),
                "expected enable failure, got {err:?}"
            );
            assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
            let disables = count_damiao_disables(&bus);
            // No automatic rollback at ANY pose (including AtRest): enters FailedHold + non-recoverable ENABLE_FAILED.
            assert_eq!(
                disables, 0,
                "failed enable (prior_all_off={prior_all_off}, {pose:?}) must transmit ZERO automatic rollback disable frames"
            );
            assert_eq!(
                inner.enable_lifecycle(),
                armd::supervisor::EnableLifecycle::FailedHold,
                "failed enable (prior_all_off={prior_all_off}, {pose:?}) must enter FailedHold"
            );
            assert!(inner.is_torqued, "failed enable after writes must keep holding");
            assert!(!inner.disable_pending, "failed enable must not leave disable_pending");
            assert_eq!(
                inner.safety.fault().map(|f| f.code),
                Some(FaultCode::EnableFailed),
                "failed enable (prior_all_off={prior_all_off}, {pose:?}) must latch EnableFailed"
            );
            assert!(
                !inner.safety.recoverable(),
                "EnableFailed must be NON-recoverable (operator inspection, explicit clear, re-enable)"
            );
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
        // S2: provide K=3 causal position reads spanning >= 20 ms for rest-pose window.
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            drv.transport_mut().provide_rx(&rx_fb);
            drv.read_joint_states().unwrap();
        }
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
        for _ in 0..3 {
            drv.transport_mut().provide_rx(&rx_rb);
        }
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
        for _ in 0..3 {
            drv.transport_mut().provide_rx(&rx_rb);
        }
        let req_res = drv.disable_torque(DisableKind::DisableRequested {
            started_at: drv.clock_now(),
        });
        let req_tx = drv.transport_mut().take_tx();
        let req_disables = count_feetech_torque_off_writes(&req_tx);
        match pose {
            PoseCase::AtRest => {
                assert!(req_res.is_ok(), "Feetech DisableRequested at rest failed: {req_res:?}");
                assert_eq!(req_disables, 6);
            }
            PoseCase::Raised => {
                assert!(
                    matches!(req_res, Err(armd::driver::ArmDriverError::TorqueOffWouldDrop)),
                    "Feetech DisableRequested (Raised) must refuse immediately, got {req_res:?}"
                );
                assert_eq!(req_disables, 0, "Feetech DisableRequested (Raised) must send 0 packets");
            }
            PoseCase::Stale => {
                assert!(
                    matches!(req_res, Err(armd::driver::ArmDriverError::DisableRequestExpired)),
                    "Feetech DisableRequested (Stale) must defer/probe and expire at 1 s, got {req_res:?}"
                );
                assert_eq!(req_disables, 0, "Feetech DisableRequested (Stale) must send 0 packets");
            }
        }

        // Protective reasons (`Thermal`, `ProcessExit`) bypass Rule A′ on Feetech in all poses.
        for reason in [
            DisableProtectiveReason::Thermal,
            DisableProtectiveReason::ProcessExit,
        ] {
            for _ in 0..3 {
                drv.transport_mut().provide_rx(&rx_rb);
            }
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
            enable_torque_admitted(&mut inner).unwrap();
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

/// Partial enable at a raised pose followed by a post-enable validation failure (both
/// `read_res.is_err()` and `!is_feedback_valid() / enable_state != Enabled` branches while
/// safety is still `Idle`) MUST route through the unified reducer's `FailedHold` transition:
/// 0 disable frames transmitted, motors remain holding at measured poses, and non-recoverable
/// `ENABLE_FAILED` is latched.
#[test]
fn partial_enable_at_raised_pose_then_validation_failure_enters_failed_hold_and_non_recoverable_enable_failed() {
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
        fn has_driver_fresh_position_api(&self) -> bool {
            self.inner.has_driver_fresh_position_api()
        }
        fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
            self.inner.fresh_measured_position_for(joint, max_age)
        }
        fn joint_hold_mode_verified(&self, joint: usize) -> bool {
            self.inner.joint_hold_mode_verified(joint)
        }
        fn drop_pending_holds_outside_mask(&mut self, hold_mask: u64) {
            self.inner.drop_pending_holds_outside_mask(hold_mask);
        }
        fn write_measured_hold_masked(
            &mut self,
            hold_mask: u64,
            targets: &[JointTarget],
        ) -> Result<armd::driver::HoldPassReport, ArmDriverError> {
            self.inner.write_measured_hold_masked(hold_mask, targets)
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
                // motors 1..6 are physically enabled (`0x1`) at the raised pose with causal
                // per-motor evidence recorded, then return `Ok(())` so the unified enable reducer
                // enters `step_enable_validate` while `safety.state()` is STILL `Idle`.
                let _ = self.inner.begin_torque(true)?;
                // Advance past the 20 ms E1/S1 causal settle window and deliver K=3 consecutive
                // post-settle state-1 samples spanning >= 2 ticks (>= 20 ms, S2) so J1..J6
                // establish causal `OnConfirmed` evidence before `step_enable_validate`.
                for step in 0..3 {
                    self.bus.with(|s| {
                        if let Some(c) = &s.clock {
                            c.advance(if step == 0 {
                                Duration::from_millis(22)
                            } else {
                                Duration::from_millis(10)
                            });
                        }
                        for j in 0..6 {
                            s.motors[j].state_override = None;
                            s.motors[j].enabled = true;
                            s.push_feedback(j);
                        }
                    });
                    let dl = self.inner.clock_now() + Duration::from_millis(5);
                    let _ = self.inner.poll_torque(dl);
                }
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
        // Advance past 100 ms so silent J7's startup feedback is genuinely stale before enable.
        clock.advance(Duration::from_millis(120));
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
        let err = enable_torque_admitted(&mut inner).unwrap_err();
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

        // 3. Non-recoverable `ENABLE_FAILED` with per-joint torque and hold status explains the
        //    failure and enters FailedHold.
        assert_eq!(
            inner.enable_lifecycle(),
            armd::supervisor::EnableLifecycle::FailedHold,
            "fail_read_err={fail_read_err}: post-enable validation failure must enter FailedHold"
        );
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(_)),
            "fail_read_err={fail_read_err}: must latch Fault, got {:?}",
            inner.safety.state()
        );
        let fault = inner.safety.fault().expect("ENABLE_FAILED fault expected");
        assert_eq!(
            fault.code,
            FaultCode::EnableFailed,
            "fault code must be EnableFailed"
        );
        assert!(
            !inner.safety.recoverable(),
            "ENABLE_FAILED must be non-recoverable"
        );
        assert!(
            !fault.recoverable,
            "observation fault.recoverable must be false"
        );
        assert!(
            fault.message.contains("J1:queued") && fault.message.contains("J7:unheld(no_fresh_fb)"),
            "ENABLE_FAILED message must include per-joint hold status: {}",
            fault.message
        );

        // 4. Holding targets are sent to all motors with fresh feedback on tick() while in
        //    FailedHold, and explicit `clear_fault()` succeeds AT THE RAISED POSE (keeping motors
        //    holding) once all 7 motors respond with fresh feedback!
        if !fail_read_err {
            bus.with(|s| {
                s.motors[6].silent = false;
                s.rx.clear();
                s.pending.clear();
                for j in 0..7 {
                    s.motors[j].state_override = None;
                }
            });
            bus.clear_log();
            clock.advance(Duration::from_millis(25));
            bus.with(|s| s.push_all_feedback());
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
                "tick() while in FailedHold must send hold targets to all 7 OnConfirmed/Unknown motors with fresh feedback, got {hold_frames:?}"
            );
            assert!(bus.with(|s| (0..6).all(|j| s.motors[j].enabled)));

            // Explicit `clear_fault()` succeeds while raised without dropping torque!
            clock.advance(Duration::from_millis(10));
            bus.with(|s| s.push_all_feedback());
            let cleared = inner
                .clear_fault()
                .expect("ENABLE_FAILED must be clearable via explicit clear_fault while raised");
            assert_eq!(cleared, SafetyState::Idle);
            assert!(inner.is_torqued, "arm stays holding after clear_fault at raised pose");
            assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
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
        "EnableRollbackHeld",
        "ENABLE_ROLLBACK_HELD",
        "sync_torque_state_after_stop",
        "all_motors_confirmed_off_after_enable_writes",
        "apply_refused_enable_rollback",
        "enable_failed_budgeted",
        "last_enable_write_seq",
        "OffConfirmed",
        "OnConfirmed",
        "MotorTorqueEvidence",
        "DisableConfirm",
        "DAMIAO_DISABLE_CONFIRM",
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

/// Round 6 (Major 3): `RobStrideArmDriver::clear_fault()` sends type-4 (`TYPE_STOP`) CAN frames
/// with `data[0] = 1` (documented under motor-stop; hardware behaviour unverified). Fail closed:
/// supervisor `clear_fault()` refuses away from rest with `TORQUE_OFF_WOULD_DROP` (0 type-4 frames
/// transmitted) and allows it at fresh rest pose.
#[test]
fn robstride_clear_fault_routes_through_rest_pose_gate_and_refuses_raised() {
    use armd::driver::RobStrideArmDriver;
    use armd::model::RestingPosePolicy;
    use armd::protocol::robstride::{self, RobStrideLimits};
    use armd::transport::{CanFrame, CanTransport, DriverClock, MockCanTransport, TransportError};
    use std::sync::Mutex;

    #[derive(Clone)]
    struct SharedCan(Arc<Mutex<MockCanTransport>>);
    impl SharedCan {
        fn new(clock: MockClock) -> Self {
            Self(Arc::new(Mutex::new(MockCanTransport::new().with_clock(clock))))
        }
        fn push_rx(&self, f: CanFrame) {
            self.0.lock().unwrap().provide_rx_frame(f);
        }
        fn take_tx(&self) -> Vec<CanFrame> {
            self.0.lock().unwrap().take_tx_frames()
        }
    }
    impl CanTransport for SharedCan {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            self.0.lock().unwrap().clock()
        }
        fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
            self.0.lock().unwrap().transmit(frame)
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.0.lock().unwrap().receive()
        }
        fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
            self.0.lock().unwrap().receive_timestamped()
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            self.0.lock().unwrap().flush()
        }
    }

    let rs_limits = RobStrideLimits::default();
    let make_fb_frame = |motor_id: u8, pos_rad: f32, fault_flags: u8| {
        let id = robstride::build_feedback_extended_id(fault_flags, motor_id, 0xFD);
        let p_raw = ((pos_rad - rs_limits.p_min) / (rs_limits.p_max - rs_limits.p_min) * 65535.0)
            .clamp(0.0, 65535.0) as u16;
        let v_raw = 32768u16;
        let t_raw = 32768u16;
        let temp_raw = 300u16; // 30.0 C
        let data = [
            (p_raw >> 8) as u8,
            p_raw as u8,
            (v_raw >> 8) as u8,
            v_raw as u8,
            (t_raw >> 8) as u8,
            t_raw as u8,
            (temp_raw >> 8) as u8,
            temp_raw as u8,
        ];
        CanFrame::new_extended(id, &data).unwrap()
    };

    let mut cfg = ArmConfig::default();
    cfg.dof = 6;
    cfg.joint_names = (1..=6).map(|i| format!("joint_{i}")).collect();
    cfg.limits.truncate(6);
    cfg.home_positions = vec![0.0; 6];
    // Wider resting_pose position tolerance (0.10) than rest_pose_tolerance_rad (0.05) so a
    // 0.08 rad offset specifically exercises the Rule A′ TORQUE_OFF_WOULD_DROP refusal, while
    // 0.40 rad is refused by resting-support check (both transmit 0 type-4 frames).
    cfg.resting_pose = Some(RestingPosePolicy {
        positions: Some(vec![0.0; 6]),
        position_tolerances: vec![0.10; 6],
        max_total_deviation: 0.30,
        max_velocity: 0.1,
        joint_max_velocities: Vec::new(),
        feedback_max_age_ms: 100,
        support_torque_limits: Vec::new(),
        support_load_limits: Vec::new(),
    });
    cfg.rest_pose_tolerance_rad = 0.05;
    cfg.lease_timeout_ms = 600_000;

    let clock = MockClock::new();
    let can = SharedCan::new(clock.clone());
    let mut drv = RobStrideArmDriver::new(can.clone(), vec![1, 2, 3, 4, 5, 6]);
    // Establish initial empty-queue receive bound.
    drv.read_joint_states().unwrap();

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.is_torqued = true;

    // 1. Non-rest pose (J2 = 0.08 rad > 0.05 rad tolerance) with hardware fault on motor 2 ->
    //    clear_fault refuses with TORQUE_OFF_WOULD_DROP and transmits ZERO type-4 frames!
    for id in 1..=6u8 {
        let q = if id == 2 { 0.08 } else { 0.0 };
        let flags = if id == 2 { 0x04 } else { 0x00 };
        can.push_rx(make_fb_frame(id, q, flags));
    }
    inner.tick(0.02);
    assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
    let epoch_before = inner.safety.epoch_id();
    let _ = can.take_tx();

    // Provide another fresh feedback batch for `clear_fault()`'s initial `read_joint_states`.
    for id in 1..=6u8 {
        let q = if id == 2 { 0.08 } else { 0.0 };
        let flags = if id == 2 { 0x04 } else { 0x00 };
        can.push_rx(make_fb_frame(id, q, flags));
    }
    let err = inner.clear_fault().unwrap_err();
    assert!(
        matches!(
            err,
            SafetyError::TorqueOffWouldDrop {
                joint: Some(ref j),
                ..
            } if j == "joint_2"
        ),
        "RobStride clear_fault away from rest must refuse with TorqueOffWouldDrop, got {err:?}"
    );
    let sent_raised = can.take_tx();
    assert!(
        sent_raised.is_empty(),
        "RobStride clear_fault away from rest must transmit 0 type-4 frames, got {sent_raised:?}"
    );
    assert_eq!(inner.safety.epoch_id(), epoch_before);
    assert!(inner.is_torqued);

    // 2. At rest pose (all 6 joints 0.0 rad) with K=3 fresh feedback samples spanning >= 20 ms (S2)
    //    -> clear_fault transmits type-4 clear-fault frames to all 6 motors!
    for _ in 0..3 {
        clock.advance(Duration::from_millis(10));
        for id in 1..=6u8 {
            let flags = if id == 2 { 0x04 } else { 0x00 };
            can.push_rx(make_fb_frame(id, 0.0, flags));
        }
        inner.tick(0.02);
    }
    let _ = can.take_tx();
    for id in 1..=6u8 {
        let flags = if id == 2 { 0x04 } else { 0x00 };
        can.push_rx(make_fb_frame(id, 0.0, flags));
    }
    let res_at_rest = inner.clear_fault();
    assert!(
        matches!(res_at_rest, Err(SafetyError::ClearFaultRejected(_))),
        "first clear_fault sends hardware reset and awaits post-reset clean feedback, got {res_at_rest:?}"
    );
    let sent_at_rest = can.take_tx();
    assert_eq!(
        sent_at_rest.len(),
        6,
        "RobStride clear_fault at fresh rest pose must transmit 6 type-4 clear-fault frames, got {}",
        sent_at_rest.len()
    );
    for (idx, frame) in sent_at_rest.iter().enumerate() {
        let expected = robstride::build_clear_fault_frame((idx + 1) as u8, 0xFD);
        assert_eq!(*frame, expected);
    }
}

// ============================================================================
// Torque-evidence + FailedHold reducer spec tests (7 required cases)
// ============================================================================

/// Helper: count Damiao `CMD_DISABLE` frames in the bus TX log.
fn spec_count_damiao_disables(bus: &FakeBus) -> usize {
    bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count()
}

/// Helper: count Feetech `REG_TORQUE_ENABLE = 0` write packets in raw serial TX bytes.
fn spec_count_feetech_torque_off_writes(tx: &[u8]) -> usize {
    let mut count = 0;
    let mut i = 0;
    while i + 8 <= tx.len() {
        if tx[i] == 0xFF
            && tx[i + 1] == 0xFF
            && tx[i + 3] == 4
            && tx[i + 4] == 0x03
            && tx[i + 5] == 0x28
            && tx[i + 6] == 0
        {
            count += 1;
            i += 8;
        } else {
            i += 1;
        }
    }
    count
}

/// Helper: decode the f32 target position from a Damiao POS_VEL frame (`0x101..=0x107`).
fn decode_damiao_pos_vel_pos(frame: &armd::transport::CanFrame) -> f32 {
    f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]])
}

/// Helper: extract every `(servo_id, goal_steps)` entry from Feetech `INST_SYNC_WRITE` packets
/// targeting `REG_GOAL_POSITION` (`0x2A`) inside raw serial TX bytes.
fn parse_feetech_sync_write_goal_steps(tx: &[u8]) -> Vec<Vec<(u8, u16)>> {
    let mut packets = Vec::new();
    let mut i = 0;
    while i + 7 <= tx.len() {
        if tx[i] == 0xFF && tx[i + 1] == 0xFF && tx[i + 2] == 0xFE {
            let len = usize::from(tx[i + 3]);
            let pkt_end = i + 4 + len;
            if pkt_end <= tx.len() && tx[i + 4] == 0x83 && tx[i + 5] == 0x2A && tx[i + 6] == 2 {
                let mut entries = Vec::new();
                let mut p = i + 7;
                while p + 3 < pkt_end {
                    let id = tx[p];
                    let steps = u16::from_le_bytes([tx[p + 1], tx[p + 2]]);
                    entries.push((id, steps));
                    p += 3;
                }
                packets.push(entries);
                i = pkt_end;
                continue;
            }
        }
        i += 1;
    }
    packets
}

/// Helper: build a 6-DOF Feetech ArmConfig with resting pose at 0.0 rad (2048 steps).
fn feetech_6dof_config() -> ArmConfig {
    let mut cfg = ArmConfig::default();
    cfg.dof = 6;
    cfg.joint_names = (1..=6).map(|i| format!("joint_{i}")).collect();
    cfg.limits = (1..=6)
        .map(|i| armd::model::JointLimit::new(format!("joint_{i}"), -3.0, 3.0, 2.0, 5.0))
        .collect();
    cfg.home_positions = vec![0.0; 6];
    cfg.resting_pose = Some(armd::model::RestingPosePolicy {
        positions: Some(vec![0.0; 6]),
        position_tolerances: vec![0.05; 6],
        max_total_deviation: 0.25,
        max_velocity: 0.1,
        joint_max_velocities: Vec::new(),
        feedback_max_age_ms: 100,
        support_torque_limits: Vec::new(),
        support_load_limits: Vec::new(),
    });
    cfg.rest_pose_tolerance_rad = 0.05;
    cfg.lease_timeout_ms = 600_000;
    cfg
}

/// Helper: build raw Feetech SYNC_READ feedback responses for 6 servos at `steps_per_servo`.
fn feetech_feedback_bytes(servo_ids: &[u8], steps_per_servo: &[u16]) -> Vec<u8> {
    let mut rx = Vec::new();
    for (&id, &steps) in servo_ids.iter().zip(steps_per_servo.iter()) {
        let pos = steps.to_le_bytes();
        let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
        let cs = armd::protocol::feetech::calculate_checksum(id, 10, 0x00, &params);
        rx.extend_from_slice(&[0xFF, 0xFF, id, 10, 0x00]);
        rx.extend_from_slice(&params);
        rx.push(cs);
    }
    rx
}

/// Helper: build a single Feetech 1-byte register readback response (`id`, `val`).
fn feetech_reg_u8_resp(id: u8, val: u8) -> Vec<u8> {
    let cs = armd::protocol::feetech::calculate_checksum(id, 3, 0x00, &[val]);
    vec![0xFF, 0xFF, id, 3, 0x00, val, cs]
}

/// Spec test 1 (B1): A queued or unsolicited `0x0` Damiao frame decoded after a deferred or
/// retried enable NEVER makes an attempted motor `OffConfirmed`, and the arm keeps holding at
/// its measured positions while skipping motors whose enable write never reached the transport.
/// Spec test 2 (B2): Feetech failed enable after the arm was moved holds EVERY joint at its
/// MEASURED position, NEVER at old cached `commanded_positions` (regardless of reported enable state).
#[test]
fn spec_b2_feetech_failed_enable_holds_at_measured_positions_never_old_cached_commands() {
    use armd::driver::{FeetechArmDriver, JointHoldStatus};
    use armd::transport::MockTransport;

    #[derive(Clone)]
    struct SharedSerial(Arc<std::sync::Mutex<MockTransport>>);
    impl armd::transport::Transport for SharedSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().write(buf)
        }
        fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().write_all(buf)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().read(buf)
        }
        fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().flush()
        }
        fn clear_input_buffer(&mut self) {
            self.0.lock().unwrap().clear_input_buffer();
        }
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            armd::transport::Transport::clock(&*self.0.lock().unwrap())
        }
    }

    let clock = MockClock::new();
    let servo_ids: Vec<u8> = (1..=6).collect();
    let calib = armd::protocol::feetech::FeetechJointCalib::default();
    let old_cmd_steps: [u16; 6] = [2048, 2600, 1700, 2300, 1800, 2048];
    let moved_steps: [u16; 6] = [2150, 2450, 1820, 2250, 1920, 2120];

    // M-11: destructive `clear_input_buffer` (NOT `new_preserving`); stage replies strictly on
    // exact protocol requests so readback ordering is genuinely exercised.
    let serial_inner = Arc::new(std::sync::Mutex::new(
        MockTransport::new().with_clock(clock.clone()),
    ));
    let sync_pos_req_1 =
        armd::protocol::feetech::build_sync_read(&servo_ids[..3], armd::protocol::feetech::REG_PRESENT_POSITION_L, 8)
            .unwrap();
    let sync_pos_req_2 =
        armd::protocol::feetech::build_sync_read(&servo_ids[3..], armd::protocol::feetech::REG_PRESENT_POSITION_L, 8)
            .unwrap();
    {
        let mut ser = serial_inner.lock().unwrap();
        ser.stage_reply(&sync_pos_req_1, &feetech_feedback_bytes(&servo_ids[..3], &moved_steps[..3]));
        ser.stage_reply(&sync_pos_req_2, &feetech_feedback_bytes(&servo_ids[3..], &moved_steps[3..]));
    }
    let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial_inner)), servo_ids.clone())
        .with_calibrated(true)
        .with_tx_echo(Some(false))
        .with_gripper_index(Some(5));
    // First read at pose C (moved by hand) while supervisor commanded_positions holds old pose B.
    drv.read_joint_states().unwrap();
    let _ = serial_inner.lock().unwrap().take_tx();

    let mut inner = SupervisorInner::new(feetech_6dof_config()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    for j in 0..6 {
        inner.commanded_positions[j] = calib.steps_to_rad(i32::from(old_cmd_steps[j]));
        inner.target_positions[j] = inner.commanded_positions[j];
    }

    // Attempt set_torque(true): stage exact per-servo REG_TORQUE_ENABLE readback replies:
    // - Servos 1..3 read back 1, Servo 4 returns unexpected 0x05, Servos 5..6 read back 0.
    // Under R10, holds are the default for EVERY joint regardless of reported enable state.
    {
        let mut ser = serial_inner.lock().unwrap();
        for (id, val) in [(1u8, 1u8), (2, 1), (3, 1), (4, 5), (5, 0), (6, 0)] {
            let req = armd::protocol::feetech::build_read(
                id,
                armd::protocol::feetech::REG_TORQUE_ENABLE,
                1,
            );
            ser.stage_reply(&req, &feetech_reg_u8_resp(id, val));
        }
    }

    let err = enable_torque_admitted(&mut inner).unwrap_err();
    assert!(matches!(err, SafetyError::ClearFaultRejected(_)));
    let enable_tx = serial_inner.lock().unwrap().take_tx();
    assert_eq!(
        spec_count_feetech_torque_off_writes(&enable_tx),
        0,
        "Feetech failed enable must send ZERO automatic rollback disable packets"
    );
    assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
    assert_eq!(inner.safety.fault().map(|f| f.code), Some(FaultCode::EnableFailed));
    assert!(!inner.safety.recoverable());
    // M-07: Before the first hold pass transmits, every joint with fresh position is `Queued`.
    assert_eq!(inner.joint_hold_status(), &[JointHoldStatus::Queued; 6]);

    // Corrupt `commanded_positions` again with old pose B to prove hold servicing NEVER reads it.
    for j in 0..6 {
        inner.commanded_positions[j] = calib.steps_to_rad(i32::from(old_cmd_steps[j]));
    }
    clock.advance(Duration::from_millis(10));
    let _ = serial_inner.lock().unwrap().take_tx();
    inner.tick(0.02);

    assert_eq!(inner.joint_hold_status(), &[JointHoldStatus::Held; 6]);
    let hold_tx = serial_inner.lock().unwrap().take_tx();
    let sync_writes = parse_feetech_sync_write_goal_steps(&hold_tx);
    assert_eq!(sync_writes.len(), 1, "expected exactly 1 SYNC_WRITE hold packet, got {sync_writes:?}");
    let entries = &sync_writes[0];
    assert_eq!(
        entries.iter().map(|&(id, _)| id).collect::<Vec<_>>(),
        vec![1, 2, 3, 4, 5, 6],
        "holds are default: every joint with fresh position receives a measured hold"
    );
    for (idx, &(id, steps)) in entries.iter().enumerate() {
        assert!(
            steps.abs_diff(moved_steps[idx]) <= 1,
            "servo {id} must hold MEASURED position {} steps, got {} (old cached command was {})",
            moved_steps[idx],
            steps,
            old_cmd_steps[idx]
        );
        assert!(
            steps.abs_diff(old_cmd_steps[idx]) > 20,
            "servo {id} hold must NOT equal old cached command {}",
            old_cmd_steps[idx]
        );
    }
}

/// Spec test 3 (B3): A Damiao `POS_FORCE` gripper whose mode verification was cleared NEVER
/// vetoes the arm joints' measured-position holds: J1..J6 receive hold frames at their exact
/// measured positions while J7 receives 0 frames and is reported `UnheldNoVerifiedMode`.
#[test]
fn spec_b3_damiao_pos_force_gripper_cleared_mode_verification_does_not_veto_arm_holds() {
    use armd::driver::{GripperPosForce, JointHoldStatus};

    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits(), clock.clone());
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits())
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
            joint: 6,
            current_raw: 1000,
            close_speed_rad_s: 1.0,
            closed_rad: -2.0,
            open_rad: 0.0,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        })
        .unwrap();
    let pose = [0.15_f32, 0.35, -0.25, 0.10, -0.20, 0.05, -0.80];
    bus.with(|s| {
        s.follow_commands = false;
        s.immediate_replies = true;
        for (j, m) in s.motors.iter_mut().enumerate() {
            m.q = pose[j];
            m.enabled = false;
        }
        s.push_all_feedback();
    });
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();

    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(dm));
    inner.lease_enabled = false;
    inner.register_live_connection(1).unwrap();
    inner.tick(0.02);

    let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
        inner,
        BOOT,
        Capacities { commands: 8, plans: 1, replies: 16 },
    );

    // 1. L2: J7 completed causal POS_FORCE register write/readback and pre-enable hold, then
    //    enable confirmation times out on J7 (drop enable frames on J7 while J1..J6 confirm
    //    OnConfirmed). Per L2, a failed enable MUST NOT clear J7's causally verified hold mode
    //    flag (`verified_modes[6]`), so J7 remains verified and held alongside J1..J6.
    bus.with(|s| {
        s.motors[6].drop_torque_frames = 10;
        s.push_all_feedback();
    });
    bus.clear_log();
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    b.submit(ClientId(1), 1, auth, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn)).unwrap();
    assert!(fx.step().command.unwrap().1.is_ok());
    let mut outcome = None;
    for _ in 0..30 {
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        let rep = fx.step();
        if let Some((_, o)) = rep.transition_ended {
            outcome = Some(o);
            break;
        }
    }
    assert_eq!(outcome, Some(Outcome::Failed("ENABLE_FAILED")));
    assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
    assert_eq!(spec_count_damiao_disables(&bus), 0, "no automatic rollback on failed enable");
    assert_eq!(
        fx.core.joint_hold_status()[6],
        JointHoldStatus::Held,
        "L2: failed enable must NOT clear J7's causally verified POS_FORCE hold mode"
    );

    // 2. B3: When J7 genuinely has NO causal mode verification (e.g. J7 silent on RID_CTRL_MODE
    //    while J1..J6 are causally verified and energized in stopped hold), unverified J7 is
    //    reported UnheldNoVerifiedMode, emits ZERO hold frames, and NEVER vetoes J1..J6 arm holds!
    let clock2 = MockClock::new();
    let bus2 = FakeBus::with_clock(&limits(), clock2.clone());
    bus2.with(|s| {
        s.follow_commands = false;
        s.immediate_replies = true;
        s.motors[6].regs.remove(&damiao::RID_CTRL_MODE);
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = pose[i];
            m.enabled = i < 6;
        }
    });
    let mut dm2 = DamiaoArmDriver::new(bus2.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits())
        .unwrap()
        .with_gripper_pos_force(GripperPosForce {
            joint: 6,
            current_raw: 1000,
            close_speed_rad_s: 1.0,
            closed_rad: -2.0,
            open_rad: 0.0,
            calibrated: false,
            ceiling_torque_nm: None,
            torque_reference: None,
        })
        .unwrap();
    dm2.check_mapping_ranges();
    for step in 0..3 {
        if step > 0 {
            clock2.advance(Duration::from_millis(10));
        }
        bus2.with(|s| s.push_all_feedback());
        dm2.read_joint_states().unwrap();
    }
    let mut inner2 = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(dm2));
    inner2.lease_enabled = false;
    inner2.is_torqued = true;
    inner2.apply_stop_policy(StopPolicy::EStopLatch);
    let (mut fx2, _b2, _tw2, _stop2) = ReleaseFixture::new(
        inner2,
        BOOT,
        Capacities { commands: 8, plans: 1, replies: 16 },
    );

    clock2.advance(Duration::from_millis(10));
    bus2.with(|s| s.push_all_feedback());
    bus2.clear_log();
    fx2.step();

    assert_eq!(fx2.core.joint_hold_status()[0..6], [JointHoldStatus::Held; 6]);
    assert_eq!(fx2.core.joint_hold_status()[6], JointHoldStatus::UnheldNoVerifiedMode);

    let tx = bus2.tx_log();
    let arm_holds: Vec<_> = tx.iter().filter(|f| f.id >= 0x101 && f.id <= 0x106).collect();
    assert_eq!(
        arm_holds.len(),
        6,
        "unverified POS_FORCE gripper must NOT veto arm joints J1..J6 holds; got {arm_holds:?}"
    );
    for (j, f) in arm_holds.iter().enumerate() {
        assert_eq!(f.id, 0x101 + j as u32);
        assert!(
            (decode_damiao_pos_vel_pos(f) - pose[j]).abs() < 2e-3,
            "J{} hold mismatch: got {} expected {}",
            j + 1,
            decode_damiao_pos_vel_pos(f),
            pose[j]
        );
    }
    let j7_frames: Vec<_> = tx
        .iter()
        .filter(|f| f.id == 0x307 || f.id == 0x107 || f.id == 0x007)
        .collect();
    assert!(
        j7_frames.is_empty(),
        "unverified POS_FORCE J7 must transmit ZERO hold frames, got {j7_frames:?}"
    );
    assert_eq!(fx2.core.joint_hold_status()[6], JointHoldStatus::UnheldNoVerifiedMode);
}

/// Spec test 4 (M1/M2): Both a delayed RT enable failure (Damiao via ReleaseFixture) and a
/// Feetech-internal enable failure end in the exact same `EnableLifecycle::FailedHold` +
/// non-recoverable `FaultCode::EnableFailed` with per-joint torque and hold status.
#[test]
fn spec_m1_m2_delayed_rt_failure_and_feetech_internal_failure_both_enter_failed_hold_and_enable_failed() {
    use armd::driver::{FeetechArmDriver, JointHoldStatus};
    use armd::transport::MockTransport;

    // (M1) Delayed RT failure on Damiao (motor 4 drops enable frames until 150 ms confirm timeout).
    {
        let mut r = ExecRig::new();
        r.bus.with(|s| {
            s.follow_commands = false;
            s.motors[1].q = 0.35;
            s.motors[3].drop_torque_frames = 10;
            s.push_all_feedback();
        });
        r.step();
        r.bus.clear_log();
        r.submit_transition(TransitionOp::TorqueOn);
        assert!(r.step().command.unwrap().1.is_ok());
        let mut outcome = None;
        for _ in 0..30 {
            r.clock.advance(Duration::from_millis(20));
            r.bus.with(|s| s.push_all_feedback());
            let rep = r.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(outcome, Some(Outcome::Failed("ENABLE_FAILED")));
        assert_eq!(spec_count_damiao_disables(&r.bus), 0);
        assert_eq!(r.fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert!(r.fx.core.is_torqued && !r.fx.core.disable_pending);
        let fault = r.fx.core.safety.fault().expect("M1 fault expected");
        assert_eq!(fault.code, FaultCode::EnableFailed);
        assert!(!r.fx.core.safety.recoverable());
        assert!(!fault.recoverable);
        assert!(
            fault.message.contains("J1:queued") && fault.message.contains("J4:queued"),
            "M1 fault must contain per-joint hold status: {}",
            fault.message
        );
    }

    // (M2) Feetech-internal failure (servo 3 fails readback while servos 1..2 on and 4..6 off),
    // using destructive `clear_input_buffer` and exact request-matched readback replies (M-11).
    {
        let clock = MockClock::new();
        let mut mock = MockTransport::new().with_clock(clock.clone());
        let servo_ids: Vec<u8> = (1..=6).collect();
        let steps = [2048u16, 2300, 1950, 2048, 2048, 2048];
        let sync_pos_req_1 =
            armd::protocol::feetech::build_sync_read(&servo_ids[..3], armd::protocol::feetech::REG_PRESENT_POSITION_L, 8)
                .unwrap();
        let sync_pos_req_2 =
            armd::protocol::feetech::build_sync_read(&servo_ids[3..], armd::protocol::feetech::REG_PRESENT_POSITION_L, 8)
                .unwrap();
        mock.stage_reply(&sync_pos_req_1, &feetech_feedback_bytes(&servo_ids[..3], &steps[..3]));
        mock.stage_reply(&sync_pos_req_2, &feetech_feedback_bytes(&servo_ids[3..], &steps[3..]));
        for (id, val) in [(1u8, 1u8), (2, 1), (3, 9), (4, 0), (5, 0), (6, 0)] {
            let req = armd::protocol::feetech::build_read(
                id,
                armd::protocol::feetech::REG_TORQUE_ENABLE,
                1,
            );
            mock.stage_reply(&req, &feetech_reg_u8_resp(id, val));
        }
        let mut drv = FeetechArmDriver::new(mock, servo_ids.clone())
            .with_calibrated(true)
            .with_tx_echo(Some(false))
            .with_gripper_index(Some(5));
        drv.read_joint_states().unwrap();
        let _ = drv.transport_mut().take_tx();

        let mut inner = SupervisorInner::new(feetech_6dof_config()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        let err = enable_torque_admitted(&mut inner).unwrap_err();
        assert!(matches!(err, SafetyError::ClearFaultRejected(_)));
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert!(inner.is_torqued && !inner.disable_pending);
        let fault = inner.safety.fault().expect("M2 fault expected");
        assert_eq!(fault.code, FaultCode::EnableFailed);
        assert!(!inner.safety.recoverable());
        assert!(!fault.recoverable);
        assert!(
            fault.message.contains("J1:queued")
                && fault.message.contains("J3:queued")
                && fault.message.contains("J4:queued"),
            "M2 fault must contain per-joint hold status: {}",
            fault.message
        );
        assert_eq!(inner.joint_hold_status(), &[JointHoldStatus::Queued; 6]);
        // Service one tick -> all 6 servos transmit & flush -> status transitions Queued -> Held.
        clock.advance(Duration::from_millis(10));
        inner.tick(0.02);
        assert_eq!(inner.joint_hold_status(), &[JointHoldStatus::Held; 6]);
    }
}

/// Spec test 5: A joint without fresh feedback gets NO hold frame, drops any queued hold for
/// that joint, is reported unheld (`UnheldNoFreshFeedback`), keeps other joints fixed at their
/// captured measured positions while fresh, and recaptures when fresh feedback returns.
#[test]
fn spec_5_joint_without_fresh_feedback_gets_no_hold_frame_drops_queued_and_recaptures() {
    use armd::driver::{JointHoldStatus, TxBudget};

    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits(), clock.clone());
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits())
        .unwrap();
    let initial_pose = [0.20_f32, 0.45, -0.35, 0.15, 0.10, -0.05, 0.0];
    bus.with(|s| {
        s.follow_commands = false;
        s.immediate_replies = true;
        for (j, m) in s.motors.iter_mut().enumerate() {
            m.q = initial_pose[j];
            m.enabled = false;
        }
        s.motors[3].drop_torque_frames = 10;
        s.push_all_feedback();
    });
    dm.read_joint_states().unwrap();
    dm.check_mapping_ranges();
    // Cap TX to 2 frames/tick so tick 1 in FailedHold queues J3..J7 holds in the hold queue,
    // allowing us to verify that stale J3 drops its queued hold while J1..J2/J4..J7 keep sending.
    dm.set_tx_budget(TxBudget {
        max_frames: 2,
        max_duration: Duration::from_millis(10),
        ..TxBudget::default()
    });

    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(dm));
    inner.lease_enabled = false;

    // Enter FailedHold with all 7 joints fresh at `initial_pose`.
    assert!(enable_torque_admitted(&mut inner).is_err());
    assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);

    // Tick 1: J1 & J2 transmit, J3..J7 holds are queued in the hold TX queue.
    clock.advance(Duration::from_millis(10));
    bus.with(|s| s.push_all_feedback());
    bus.clear_log();
    inner.tick(0.02);
    {
        let drv = inner.driver.as_ref().unwrap().lock().unwrap();
        assert!(drv.pending_tx_len_total() > 0, "J3..J7 holds should be queued in hold queue");
    }

    // M-12: Specifically queue J3's hold frame at t=85 ms (age=85 ms <= 100 ms, TX age=0 ms < 45 ms),
    // then advance 20 ms to t=105 ms so J3 crosses the 100 ms freshness boundary while its queued
    // hold is only 20 ms old (cannot expire by the 45 ms TX frame age limit!). Assert J3's queued
    // hold is dropped and emits ZERO 0x103 frames on that VERY FIRST stale pass while peers send.
    bus.with(|s| {
        s.motors[2].silent = true;
        s.motors[0].q = initial_pose[0] + 0.03; // drifts while still fresh -> captured hold MUST stay fixed!
    });
    // Step 1: advance to t=+45 ms (J3 age = 45 ms <= 100 ms), tick so J3..J4 transmit and J1..J2/J5..J7 rotate.
    // Keep advancing until J3 is specifically NOT transmitted on a tick right before 100 ms (t=+85 ms),
    // meaning J3 is guaranteed freshly queued in `pending_hold_tx` with TX age = 0 ms at t=+85 ms.
    clock.advance(Duration::from_millis(35)); // J3 age = 35 ms
    bus.with(|s| {
        for j in [0, 1, 3, 4, 5, 6] {
            s.push_feedback(j);
        }
    });
    inner.tick(0.02);
    clock.advance(Duration::from_millis(35)); // J3 age = 70 ms
    bus.with(|s| {
        for j in [0, 1, 3, 4, 5, 6] {
            s.push_feedback(j);
        }
    });
    inner.tick(0.02);
    clock.advance(Duration::from_millis(15)); // J3 age = 85 ms <= 100 ms
    bus.with(|s| {
        for j in [0, 1, 3, 4, 5, 6] {
            s.push_feedback(j);
        }
    });
    bus.clear_log();
    inner.tick(0.02);
    // At t=+85 ms, J3 is either still queued (`Queued`, TX age 0..15 ms) or just transmitted; if it
    // just transmitted on that tick, run one more 5 ms tick (J3 age = 90 ms) so J3 is freshly
    // re-queued at the back of the 2-frame/tick queue with TX age = 0 ms!
    if inner.joint_hold_status()[2] == JointHoldStatus::Held {
        clock.advance(Duration::from_millis(5)); // J3 age = 90 ms <= 100 ms
        bus.with(|s| {
            for j in [0, 1, 3, 4, 5, 6] {
                s.push_feedback(j);
            }
        });
        inner.tick(0.02);
    }
    assert_eq!(
        inner.joint_hold_status()[2],
        JointHoldStatus::Queued,
        "J3 hold must be specifically queued right before crossing the 100 ms freshness boundary"
    );
    let pending_before_stale = inner.driver.as_ref().unwrap().lock().unwrap().pending_tx_len_total();
    assert!(pending_before_stale > 0);

    // Cross the 100 ms freshness boundary by advancing 20 ms (J3 age = 105..110 ms > 100 ms, while
    // J3's queued hold TX age is only 20 ms < 45 ms). Under R11 B2, J3 MUST NOT drop its already-
    // captured hold while torque_intent == On: it keeps holding at `initial_pose[2]` and is
    // reported `JointHoldStatus::HeldStaleTarget` ("hold target stale") while peers keep holding!
    clock.advance(Duration::from_millis(20));
    bus.with(|s| {
        for j in [0, 1, 3, 4, 5, 6] {
            s.push_feedback(j);
        }
    });
    bus.clear_log();
    inner.tick(0.02);

    assert_eq!(
        inner.joint_hold_status()[2],
        JointHoldStatus::HeldStaleTarget,
        "B2: stale J3 with captured hold must be reported HeldStaleTarget on the first stale pass"
    );
    assert_eq!(inner.joint_hold_status()[2].short_tag(), "hold target stale");
    let first_stale_tx = bus.tx_log();
    assert!(
        first_stale_tx.iter().any(|f| f.id >= 0x101 && f.id <= 0x107),
        "eligible joints must still transmit on the first stale pass: {first_stale_tx:?}"
    );

    // Run 4 more 10 ms ticks so every joint (including J1 and stale J3) rotates through the
    // 2-frame/tick budget while J3 stays silent, proving J1 keeps `initial_pose[0]` AND stale J3
    // keeps holding its LAST captured target `initial_pose[2]`!
    for _ in 0..4 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| {
            for j in [0, 1, 3, 4, 5, 6] {
                s.push_feedback(j);
            }
        });
        inner.tick(0.02);
    }

    assert_eq!(inner.joint_hold_status()[2], JointHoldStatus::HeldStaleTarget);
    for j in [0, 1, 3, 4, 5, 6] {
        assert!(matches!(inner.joint_hold_status()[j], JointHoldStatus::Held | JointHoldStatus::Queued));
    }
    let tx_stale = bus.tx_log();
    let j3_stale_frame = tx_stale
        .iter()
        .find(|f| f.id == 0x103)
        .expect("B2: stale J3 must keep transmitting its last captured hold target");
    assert!(
        (decode_damiao_pos_vel_pos(j3_stale_frame) - initial_pose[2]).abs() < 2e-3,
        "B2: stale J3 must keep holding its last captured target {}, got {}",
        initial_pose[2],
        decode_damiao_pos_vel_pos(j3_stale_frame)
    );
    let j1_frame = tx_stale.iter().find(|f| f.id == 0x101).expect("J1 hold frame expected");
    assert!(
        (decode_damiao_pos_vel_pos(j1_frame) - initial_pose[0]).abs() < 2e-3,
        "J1 captured hold target must stay FIXED at {} while fresh, got {}",
        initial_pose[0],
        decode_damiao_pos_vel_pos(j1_frame)
    );

    // Now J3 recovers with fresh feedback at a new measured position (-0.50 rad):
    // J3 MUST recapture -0.50 rad and resume holding across the 2-frame/tick budget!
    let new_j3 = -0.50_f32;
    bus.with(|s| {
        s.motors[2].silent = false;
        s.motors[2].q = new_j3;
    });
    bus.clear_log();
    for _ in 0..4 {
        clock.advance(Duration::from_millis(10));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }

    assert!(matches!(inner.joint_hold_status()[2], JointHoldStatus::Held | JointHoldStatus::Queued));
    let j3_frame = bus
        .tx_log()
        .into_iter()
        .find(|f| f.id == 0x103)
        .expect("recaptured J3 must transmit hold frame 0x103");
    assert!(
        (decode_damiao_pos_vel_pos(&j3_frame) - new_j3).abs() < 2e-3,
        "recaptured J3 must hold new measured position {new_j3}, got {}",
        decode_damiao_pos_vel_pos(&j3_frame)
    );
}

/// Spec test 6: RobStride with no device enable evidence reports `Unknown` (and `Unconfirmed` on
/// `begin_torque` / `begin_disable_torque`), NEVER `Confirmed`.
#[test]
fn spec_6_robstride_without_device_evidence_reports_unknown_not_confirmed() {
    use armd::driver::{DisableKind, RobStrideArmDriver, TorqueOutcome, TorqueProgress};
    use armd::transport::{CanFrame, MockCanTransport};

    let mut drv = RobStrideArmDriver::new(MockCanTransport::new(), vec![1, 2, 3, 4, 5, 6]);
    assert!(!drv.capabilities().enable_feedback);
    assert_eq!(drv.enable_state(), EnableState::Unknown);

    // Enable send succeeds on the bus, but with no device evidence MUST return Unconfirmed + Unknown.
    let en = drv.begin_torque(true).unwrap();
    assert_eq!(
        en,
        TorqueProgress::Done(TorqueOutcome::Unconfirmed),
        "RobStride begin_torque(true) must report Unconfirmed, never Confirmed"
    );
    assert_eq!(drv.enable_state(), EnableState::Unknown);
    assert!(!drv.is_enabled());

    // Configure rest-pose gate + K=3 spaced rest samples so operational DisableRequested passes B5.
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..6 {
        arm_joints[i] = i;
    }
    drv.set_stopped_disable_gate(armd::driver::StoppedDisableGate {
        safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(false)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: 0.05,
    });
    // Wait past post-motion settle (> 20 ms after enable) and drain an initial empty RX boundary
    // before collecting K=3 spaced rest samples; RobStride still reports Unknown enable telemetry.
    std::thread::sleep(Duration::from_millis(25));
    drv.read_joint_states().unwrap();
    for step in 0..3 {
        if step > 0 {
            std::thread::sleep(Duration::from_millis(12));
        }
        for id in 1..=6u8 {
            let ext_id = armd::protocol::robstride::build_feedback_extended_id(0, id, 0xFD);
            drv.can_mut()
                .provide_rx_frame(CanFrame::new_extended(ext_id, &[128, 0, 128, 0, 128, 0, 0, 250]).unwrap());
        }
        drv.read_joint_states().unwrap();
    }
    assert_eq!(drv.enable_state(), EnableState::Unknown);

    // Disable send also reports Unconfirmed + Unknown, never Confirmed.
    let dis = drv
        .begin_disable_torque(DisableKind::DisableRequested {
            started_at: drv.clock_now(),
        })
        .unwrap();
    assert_eq!(
        dis,
        TorqueProgress::Done(TorqueOutcome::Unconfirmed),
        "RobStride begin_disable_torque must report Unconfirmed, never Confirmed"
    );
    assert_eq!(drv.enable_state(), EnableState::Unknown);
}

/// Spec test 7: A failed enable AT THE REST POSE also does NOT roll back automatically (0
/// disable frames on both Damiao and Feetech), enters `FailedHold` + non-recoverable
/// `ENABLE_FAILED`, and a later explicit disable succeeds through the rest gate.
#[test]
fn spec_7_failed_enable_at_rest_pose_does_not_roll_back_automatically() {
    use armd::driver::FeetechArmDriver;
    use armd::transport::MockTransport;

    // Part A: Damiao failed enable at exact rest pose (0.0 rad on all joints, fresh feedback).
    {
        let (mut inner, bus, clock) = make_driver_rig();
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.0;
                m.enabled = false;
            }
            s.motors[3].drop_torque_frames = 10;
            s.push_all_feedback();
        });
        inner.tick(0.02);
        bus.clear_log();

        let err = enable_torque_admitted(&mut inner).unwrap_err();
        assert!(matches!(err, SafetyError::ClearFaultRejected(_)));
        assert_eq!(
            spec_count_damiao_disables(&bus),
            0,
            "failed Damiao enable at the rest pose must NOT roll back automatically"
        );
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert!(inner.is_torqued && !inner.disable_pending);
        assert_eq!(inner.safety.fault().map(|f| f.code), Some(FaultCode::EnableFailed));
        assert!(!inner.safety.recoverable());

        // Later explicit disable at the rest pose passes the rest gate and disables torque cleanly
        // once K=3 fresh (<= 100 ms) rest-pose feedback samples spanning >= 20 ms are read (S2).
        bus.with(|s| {
            s.motors[3].drop_torque_frames = 0;
        });
        for _ in 0..3 {
            clock.advance(Duration::from_millis(10));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
        bus.clear_log();
        let state = inner.disable_torque().expect("explicit disable at rest must succeed");
        assert!(matches!(state, SafetyState::Fault(_)), "explicit disable preserves ENABLE_FAILED latch");
        assert!(spec_count_damiao_disables(&bus) >= 7, "explicit disable sends disable frames");
        assert!(!inner.is_torqued);
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::Disarmed);
    }

    // Part B: Feetech failed enable at exact rest pose (2048 steps = 0.0 rad, fresh feedback),
    // retaining a shared transport recorder with destructive `clear_input_buffer` to prove ZERO
    // `TORQUE_ENABLE=0` wire packets during both failure and subsequent service ticks (M-11, M-13).
    {
        #[derive(Clone)]
        struct RecSerial(Arc<std::sync::Mutex<MockTransport>>);
        impl armd::transport::Transport for RecSerial {
            fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
                self.0.lock().unwrap().write(buf)
            }
            fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
                self.0.lock().unwrap().write_all(buf)
            }
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
                self.0.lock().unwrap().read(buf)
            }
            fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
                self.0.lock().unwrap().flush()
            }
            fn clear_input_buffer(&mut self) {
                self.0.lock().unwrap().clear_input_buffer();
            }
            fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
                armd::transport::Transport::clock(&*self.0.lock().unwrap())
            }
        }

        let clock = MockClock::new();
        let servo_ids: Vec<u8> = (1..=6).collect();
        let serial = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let sync_pos_req_1 = armd::protocol::feetech::build_sync_read(
            &servo_ids[..3],
            armd::protocol::feetech::REG_PRESENT_POSITION_L,
            8,
        )
        .unwrap();
        let sync_pos_req_2 = armd::protocol::feetech::build_sync_read(
            &servo_ids[3..],
            armd::protocol::feetech::REG_PRESENT_POSITION_L,
            8,
        )
        .unwrap();
        {
            let mut ser = serial.lock().unwrap();
            ser.stage_reply(&sync_pos_req_1, &feetech_feedback_bytes(&servo_ids[..3], &[2048u16; 3]));
            ser.stage_reply(&sync_pos_req_2, &feetech_feedback_bytes(&servo_ids[3..], &[2048u16; 3]));
            for id in 1..=6u8 {
                let req = armd::protocol::feetech::build_read(
                    id,
                    armd::protocol::feetech::REG_TORQUE_ENABLE,
                    1,
                );
                ser.stage_reply(&req, &feetech_reg_u8_resp(id, u8::from(id == 1)));
            }
        }
        let mut drv = FeetechArmDriver::new(RecSerial(Arc::clone(&serial)), servo_ids.clone())
            .with_calibrated(true)
            .with_tx_echo(Some(false))
            .with_gripper_index(Some(5));
        drv.read_joint_states().unwrap();
        let _ = serial.lock().unwrap().take_tx();

        let mut inner = SupervisorInner::new(feetech_6dof_config()).with_driver(Box::new(drv));
        inner.lease_enabled = false;

        let err = enable_torque_admitted(&mut inner).unwrap_err();
        assert!(matches!(err, SafetyError::ClearFaultRejected(_)));
        let fail_tx = serial.lock().unwrap().take_tx();
        assert_eq!(
            spec_count_feetech_torque_off_writes(&fail_tx),
            0,
            "Feetech failed enable at rest pose must send ZERO TORQUE_ENABLE=0 wire packets"
        );
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert!(inner.is_torqued && !inner.disable_pending);
        assert_eq!(inner.safety.fault().map(|f| f.code), Some(FaultCode::EnableFailed));
        assert!(!inner.safety.recoverable());
        assert_eq!(inner.torque_intent, armd::TorqueIntent::On);
        // Service subsequent FailedHold ticks -> STILL zero TORQUE_ENABLE=0 packets on the wire.
        for _ in 0..3 {
            clock.advance(Duration::from_millis(10));
            inner.tick(0.02);
        }
        let hold_tx = serial.lock().unwrap().take_tx();
        assert_eq!(
            spec_count_feetech_torque_off_writes(&hold_tx),
            0,
            "Feetech FailedHold service ticks at rest must send ZERO TORQUE_ENABLE=0 wire packets"
        );
    }
}

/// Review Blocker B-04 regression: Mid-batch E-stop after motor 1's enable write on BOTH
/// FashionStar and RobStride MUST immediately abort remaining enable writes (0 later enable
/// packets), route through the shared failure transition to `FailedHold` + `ENABLE_FAILED`,
/// and preserve `SafetyState::EStop`.
#[test]
fn review_b04_fashionstar_and_robstride_mid_batch_estop_aborts_with_zero_later_enables() {
    use armd::driver::{FashionStarArmDriver, RobStrideArmDriver};
    use armd::rt_core::urgent::EstopState;
    use armd::transport::{CanFrame, CanTransport, MockTransport, Transport};

    // Part A: FashionStar mid-batch E-stop after servo 1 enable write -> 0 enable writes to servos 2..6.
    {
        struct EstopAfterFirstWriteSerial {
            inner: MockTransport,
            enable_writes: Arc<std::sync::Mutex<Vec<Vec<u8>>>>,
            servo1_enable: Vec<u8>,
            estop_state: Arc<EstopState>,
            tripped: bool,
        }
        impl Transport for EstopAfterFirstWriteSerial {
            fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
                let is_enable = (1..=6u8).any(|id| buf == armd::protocol::fashionstar::build_set_torque(id, true).as_slice());
                if is_enable {
                    self.enable_writes.lock().unwrap().push(buf.to_vec());
                }
                let res = self.inner.write(buf);
                // Trip E-stop right after servo 1's torque-enable packet is written.
                if !self.tripped && buf == self.servo1_enable.as_slice() {
                    self.tripped = true;
                    let _ = self.estop_state.request_and_latch();
                }
                res
            }
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
                self.inner.read(buf)
            }
            fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
                self.inner.flush()
            }
            fn clear_input_buffer(&mut self) {
                self.inner.clear_input_buffer();
            }
        }

        let enable_writes = Arc::new(std::sync::Mutex::new(Vec::new()));
        let mut inner = SupervisorInner::new(ArmConfig {
            dof: 6,
            lease_timeout_ms: 600_000,
            ..ArmConfig::default()
        });
        inner.lease_enabled = false;
        let serial = EstopAfterFirstWriteSerial {
            inner: MockTransport::new(),
            enable_writes: Arc::clone(&enable_writes),
            servo1_enable: armd::protocol::fashionstar::build_set_torque(1, true),
            estop_state: inner.estop_state.clone(),
            tripped: false,
        };
        let drv = FashionStarArmDriver::new(serial, vec![1, 2, 3, 4, 5, 6]);
        inner = inner.with_driver(Box::new(drv));
        enable_writes.lock().unwrap().clear();

        let err = enable_torque_admitted(&mut inner).unwrap_err();
        assert!(matches!(err, SafetyError::EStopActive), "expected EStopActive, got {err:?}");
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert!(inner.e_stop_latched.load(Ordering::SeqCst), "E-stop latch flag must be preserved");
        assert!(
            inner.safety.has_latched_fault_kind(armd::safety::FaultKind::Estop),
            "E-stop fault kind must be preserved"
        );
        assert!(inner.safety.has_latched_fault_code(FaultCode::EnableFailed));
        let sent = enable_writes.lock().unwrap().clone();
        assert_eq!(
            sent.len(),
            1,
            "mid-batch E-stop after FashionStar servo 1 must send 0 further enable packets: {sent:?}"
        );
    }

    // Part B: RobStride mid-batch E-stop after motor 1 enable frame -> 0 enable frames to motors 2..6.
    {
        struct EstopAfterFirstEnableCan {
            tx: Arc<std::sync::Mutex<Vec<CanFrame>>>,
            estop_state: Arc<EstopState>,
            tripped: bool,
        }
        impl CanTransport for EstopAfterFirstEnableCan {
            fn transmit(&mut self, frame: &CanFrame) -> Result<(), armd::transport::TransportError> {
                self.tx.lock().unwrap().push(*frame);
                let comm_type = (frame.id >> 24) & 0x1F;
                if !self.tripped && comm_type == 3 {
                    self.tripped = true;
                    let _ = self.estop_state.request_and_latch();
                }
                Ok(())
            }
            fn receive(&mut self) -> Result<CanFrame, armd::transport::TransportError> {
                Err(armd::transport::TransportError::WouldBlock)
            }
            fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
                Ok(())
            }
        }

        let tx = Arc::new(std::sync::Mutex::new(Vec::new()));
        let mut inner = SupervisorInner::new(ArmConfig {
            dof: 6,
            lease_timeout_ms: 600_000,
            ..ArmConfig::default()
        });
        inner.lease_enabled = false;
        let can = EstopAfterFirstEnableCan {
            tx: Arc::clone(&tx),
            estop_state: inner.estop_state.clone(),
            tripped: false,
        };
        let drv = RobStrideArmDriver::new(can, vec![1, 2, 3, 4, 5, 6]);
        inner = inner.with_driver(Box::new(drv));
        tx.lock().unwrap().clear();

        let err = enable_torque_admitted(&mut inner).unwrap_err();
        assert!(matches!(err, SafetyError::EStopActive), "expected EStopActive, got {err:?}");
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert!(inner.e_stop_latched.load(Ordering::SeqCst), "E-stop latch flag must be preserved");
        assert!(
            inner.safety.has_latched_fault_kind(armd::safety::FaultKind::Estop),
            "E-stop fault kind must be preserved"
        );
        assert!(inner.safety.has_latched_fault_code(FaultCode::EnableFailed));
        let sent_enables: Vec<_> = tx
            .lock()
            .unwrap()
            .iter()
            .filter(|f| ((f.id >> 24) & 0x1F) == 3)
            .copied()
            .collect();
        assert_eq!(
            sent_enables.len(),
            1,
            "mid-batch E-stop after RobStride motor 1 must send 0 further enable frames: {sent_enables:?}"
        );
    }
}

/// Review Majors M-01, M-06, M-07, M-08, M-09, M-11 regression:
/// - M-01: Unverified Damiao arm joint (`joint_modes[i] == None`) gets 0 hold frames & `UnheldNoVerifiedMode`.
/// - M-06: `StopPolicy::ThermalTrip` interrupts in-flight enable cleanly via reducer (preserving ThermalTrip + EnableFailed).
/// - M-07: Live observation `ENABLE_FAILED` updates without epoch bump and survives subsequent `HardwareFault`.
/// - M-08 / M-11: Feetech late 1-byte reply after timed-out read stays `Unknown` until `PING` barrier; duplicate 1-byte reply -> `Unknown`.
/// - M-09: Feedback continuity gap > 100 ms with resumed feedback before next tick recaptures the new measured position.
#[test]
fn review_m01_m06_m07_m08_m09_unverified_mode_thermal_late_reply_and_gap_recapture() {
    use armd::driver::{DamiaoArmDriver, EnableState, FeetechArmDriver, JointHoldStatus};
    use armd::rt_core::broker::{BootId, Capacities, ClientId, Submit, TransitionOp};
    use armd::rt_core::ops::{CoreOps, ReleaseFixture};
    use armd::transport::MockTransport;

    // M-01 + M-07 + M-09 on Damiao FailedHold:
    {
        let (mut inner, bus, clock) = make_driver_rig();
        let pose = [0.10f32, 0.30, -0.20, 0.25, -0.15, 0.12, 0.0];
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = pose[i];
                m.enabled = false;
            }
            // Motor 7 drops enable so enable fails into FailedHold; motors 1..6 enable.
            s.motors[6].drop_torque_frames = 10;
            s.push_all_feedback();
        });
        inner.tick(0.02);

        // M-01: When an arm joint's mode is unverified (`joint_modes[2] == None` after a failed
        // mode write on J3), `joint_hold_mode_verified(2)` MUST be false and
        // `write_measured_hold_masked` MUST emit ZERO frames (`0x103` or `0x003`) for J3 while
        // still holding verified peers!
        {
            let m01_clock = MockClock::new();
            let m01_bus = FakeBus::with_clock(&limits(), m01_clock.clone());
            m01_bus.with(|s| {
                s.immediate_replies = true;
                for (i, m) in s.motors.iter_mut().enumerate() {
                    m.q = pose[i];
                }
                s.push_all_feedback();
            });
            let mut m01_drv = DamiaoArmDriver::new(m01_bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
                .with_joint_limits(limits())
                .unwrap();
            // Simulate J3 in unexpected CTRL_MODE = MIT (1) during range check -> leaves
            // `joint_modes[2] == None` while peers remain verified `PosVel` (exact M-01 trigger!).
            m01_bus.with(|s| {
                s.motors[2].regs.insert(
                    armd::protocol::damiao::RID_CTRL_MODE,
                    armd::protocol::damiao::CTRL_MODE_MIT.to_le_bytes(),
                );
            });
            m01_drv.check_mapping_ranges();
            assert!(
                !m01_drv.joint_hold_mode_verified(2),
                "M-01: arm joint with joint_modes[2] == None must NOT be hold-mode verified"
            );
            m01_bus.with(|s| s.push_all_feedback());
            m01_drv.read_joint_states().unwrap();
            let hold_t: Vec<_> = pose.iter().map(|&p| armd::driver::JointTarget::position(p)).collect();
            m01_bus.clear_log();
            let rep = m01_drv.write_measured_hold_masked(0b0111_1111, &hold_t).unwrap();
            assert_eq!(rep.joint_status[2], JointHoldStatus::UnheldNoVerifiedMode);
            assert_eq!(rep.joint_status[1], JointHoldStatus::Held);
            let j3_tx: Vec<_> = m01_bus
                .tx_log()
                .into_iter()
                .filter(|f| f.id == 0x103 || f.id == 0x003)
                .collect();
            assert!(
                j3_tx.is_empty(),
                "M-01: unverified J3 must receive ZERO POS_VEL or MIT hold frames: {j3_tx:?}"
            );
        }

        // Trigger FailedHold via blocking set_torque(true).
        let err = enable_torque_admitted(&mut inner).unwrap_err();
        assert!(matches!(err, SafetyError::ClearFaultRejected(_)));
        assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        // Enable timed out after 150 ms (> 100 ms hold freshness) so initial status is unheld(no_fresh_fb).
        assert_eq!(inner.joint_hold_status()[1], JointHoldStatus::UnheldNoFreshFeedback);
        let epoch_at_latch = inner.safety.epoch_id();

        // M-07: Tick transmits hold at pose[1] = 0.30 -> live observation fault updates to `held`
        // with ZERO epoch bump!
        clock.advance(Duration::from_millis(10));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        assert_eq!(inner.joint_hold_status()[1], JointHoldStatus::Held);
        assert_eq!(inner.safety.epoch_id(), epoch_at_latch, "live hold status update must not bump epoch");
        let obs_fault = inner.current_observation().fault.expect("live ENABLE_FAILED expected");
        assert_eq!(obs_fault.code, FaultCode::EnableFailed);
        assert!(
            obs_fault.message.contains("J2:held"),
            "live ENABLE_FAILED must update from queued to held: {}",
            obs_fault.message
        );
        let j2_before_gap: Vec<f32> = bus
            .tx_log()
            .into_iter()
            .filter(|f| f.id == 0x102)
            .map(|f| decode_damiao_pos_vel_pos(&f))
            .collect();
        assert!(!j2_before_gap.is_empty());
        assert!((j2_before_gap[0] - 0.30).abs() < 2e-3, "initial hold must be 0.30: {j2_before_gap:?}");

        // M-09: Unobserved > 100 ms gap (120 ms) with NO intermediate tick, J2 moves to 0.55 and
        // feedback resumes: under R11 B2, the first tick (whose empty-queue bound is still stale)
        // keeps holding J2's last captured target 0.30 (`HeldStaleTarget`, never dropping holds
        // while torque_intent == On), then fresh post-empty feedback at 0.55 recaptures 0.55!
        clock.advance(Duration::from_millis(120));
        bus.with(|s| {
            s.motors[1].q = 0.55;
            s.push_all_feedback();
        });
        bus.clear_log();
        inner.tick(0.02);
        assert_eq!(inner.joint_hold_status()[1], JointHoldStatus::HeldStaleTarget);
        let j2_stale_bound: Vec<f32> = bus
            .tx_log()
            .into_iter()
            .filter(|f| f.id == 0x102)
            .map(|f| decode_damiao_pos_vel_pos(&f))
            .collect();
        assert!(
            !j2_stale_bound.is_empty() && (j2_stale_bound[0] - 0.30).abs() < 2e-3,
            "B2: stale empty-queue bound after 120 ms gap must keep last captured 0.30 hold: {j2_stale_bound:?}"
        );
        clock.advance(Duration::from_millis(5));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        let j2_after_gap: Vec<f32> = bus
            .tx_log()
            .into_iter()
            .filter(|f| f.id == 0x102)
            .map(|f| decode_damiao_pos_vel_pos(&f))
            .collect();
        assert!(!j2_after_gap.is_empty(), "J2 must transmit recaptured hold after continuity break");
        assert!(
            (j2_after_gap[0] - 0.55).abs() < 2e-3,
            "M-09: >100 ms gap with resumed feedback must recapture 0.55, not old 0.30: {j2_after_gap:?}"
        );

        // M-07: Subsequent HardwareFault latching into FaultKind::Other must NOT clobber EnableFailed detail!
        inner.safety.latch(
            armd::safety::FaultKind::Other,
            armd::safety::FaultDetail::formatted(FaultCode::HardwareFault, |b| b.push_str("later hw fault")),
        );
        assert!(inner.safety.has_latched_fault_code(FaultCode::EnableFailed));
        let post_hw_fault = inner.current_observation().fault.expect("ENABLE_FAILED must remain preserved");
        assert_eq!(post_hw_fault.code, FaultCode::EnableFailed);
    }

    // M-06: ThermalTrip during an in-flight RT enable cancels cleanly through the reducer,
    // preserving ThermalTrip + EnableFailed and running protective disable.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.35;
                m.enabled = false;
                m.drop_torque_frames = 10; // keep TorqueOn pending
            }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.rt_executor_owned = true;
        inner.tick(0.02);

        let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
            inner,
            BootId(88),
            Capacities { commands: 8, plans: 1, replies: 16 },
        );
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert!(fx.core.torque_transition_pending(), "enable must be in flight before thermal trip");

        bus.clear_log();
        fx.core.apply_stop_policy(StopPolicy::ThermalTrip);
        assert!(
            fx.core.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip),
            "ThermalTrip latch must be preserved"
        );
        assert!(
            fx.core.safety.has_latched_fault_code(FaultCode::EnableFailed),
            "M-06: ThermalTrip interrupting in-flight enable must record ENABLE_FAILED via reducer"
        );
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "M-06: ThermalTrip protective disable frames must still be transmitted"
        );
    }

    // M-08 + M-11: Feetech late 1-byte reply after timed-out read MUST NOT promote to OffConfirmed/OnConfirmed
    // until a 0-byte PING barrier clears the FIFO; duplicate back-to-back 1-byte reply -> Unknown.
    {
        let clock = MockClock::new();
        let mock = MockTransport::new().with_clock(clock.clone());
        let mut drv = FeetechArmDriver::new(mock, vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));

        // 1. Timeout on REG_TORQUE_ENABLE read arms `unmatched_1b_read[0] = true`.
        let st1 = drv.refresh_enable_state().unwrap();
        assert_eq!(st1, EnableState::Unknown);

        // 2. Next refresh has ONLY a late 1-byte `0` reply queued (no 0-byte PING reply):
        //    PING barrier MUST time out and reject the late 1-byte reply -> stays Unknown!
        let ping_req = armd::protocol::feetech::build_ping(1);
        let read_req = armd::protocol::feetech::build_read(1, armd::protocol::feetech::REG_TORQUE_ENABLE, 1);
        let late_off = feetech_reg_u8_resp(1, 0);
        drv.transport_mut().stage_reply(&ping_req, &late_off);
        drv.transport_mut().stage_reply(&read_req, &late_off);
        let st2 = drv.refresh_enable_state().unwrap();
        assert_eq!(st2, EnableState::Unknown, "late 1-byte reply without PING barrier must stay Unknown");

        // 3. Now stage late 1-byte `0` followed by 0-byte PING reply on PING, plus genuine `1` on
        //    REG_TORQUE_ENABLE: PING barrier drains the late `0`, then readback reports Enabled!
        let ping_cs = armd::protocol::feetech::calculate_checksum(1, 2, 0x00, &[]);
        let ping_ok = [0xFF, 0xFF, 1, 2, 0x00, ping_cs];
        let mut ping_with_late = late_off.clone();
        ping_with_late.extend_from_slice(&ping_ok);
        let mock2 = MockTransport::new().with_clock(clock.clone());
        let mut drv2 = FeetechArmDriver::new(mock2, vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));
        let _ = drv2.refresh_enable_state().unwrap(); // arm unmatched_1b_read[0]
        drv2.transport_mut().stage_reply(&ping_req, &ping_with_late);
        drv2.transport_mut().stage_reply(&read_req, &feetech_reg_u8_resp(1, 1));
        let st3 = drv2.refresh_enable_state().unwrap();
        assert_eq!(st3, EnableState::Enabled, "after PING FIFO barrier, genuine 1 reports Enabled");
    }
}

/// Rule E2: The rest-pose disable gate requires a completed RX drain.
/// A cached resting pose MUST NEVER authorize a disable while newer frames (e.g. a queued
/// raised-pose frame behind an exhausted drain) may still be queued.
#[test]
fn e2_rest_pose_disable_gate_refuses_queued_raised_pose_behind_exhausted_drain() {
    use armd::driver::{
        ArmDriverError, DisableKind, DisableProtectiveReason, RobStrideArmDriver, StoppedDisableGate,
        TorqueProgress,
    };
    use armd::protocol::robstride;
    use armd::transport::{CanFrame, MockCanTransport};
    use std::sync::atomic::AtomicBool;

    let dm_limits = limits();
    let make_dm_fb = |id: u16, state: u8, q: f32| -> CanFrame {
        let l = &dm_limits[usize::from(id - 1)];
        let p = damiao::float_to_uint(q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        CanFrame::new_standard(
            0x10 + id,
            &[
                (state << 4) | (id as u8 & 0x0f),
                (p >> 8) as u8,
                p as u8,
                (v >> 4) as u8,
                (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
                t as u8,
                30,
                30,
            ],
        )
        .unwrap()
    };
    let junk_frame = CanFrame::new_standard(0x500, &[0u8; 8]).unwrap();

    // 1. Damiao: cached resting pose (0.0) + queued raised-pose frame (0.45) behind an exhausted drain.
    {
        let (mut inner, bus, clock) = make_driver_rig();
        enable_torque_admitted(&mut inner).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        // Fresh resting pose (0.0 rad) with completed drain and K=3 spaced causal samples (S2).
        bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(10));
            }
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
        assert_eq!(inner.may_disable_torque_while_stopped(), Ok(()));

        // Enqueue 70 unrelated bus frames followed by a REAL raised-pose frame for J1 (q = 0.45 rad).
        // First: a budgeted Step-2 read of 64 frames exhausts BEFORE reaching the raised-pose frame!
        bus.with(|s| {
            s.tx_log.clear();
            for _ in 0..70 {
                s.rx.push_back(junk_frame);
            }
            s.rx.push_back(make_dm_fb(1, 1, 0.45));
        });
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let mut out = Vec::new();
            let dl = drv.clock_now() + Duration::from_millis(10);
            let (frames, more) = drv.read_joint_states_budgeted(&mut out, dl, 64).unwrap();
            assert_eq!(frames, 64);
            assert!(more, "drain must report exhaustion");
            // L1: Zero-RX-budget disable behind exhausted drain holds the request pending
            // (`AwaitRestGate`) inside the 1 s lifetime with ZERO CMD_DISABLE frames!
            let started = drv.clock_now();
            let p = drv
                .begin_disable_torque_budgeted(
                    DisableKind::DisableRequested {
                        started_at: started,
                    },
                    0,
                )
                .expect("L1: exhausted drain must keep stopped disable pending, not fail");
            assert!(
                matches!(p, TorqueProgress::Pending { .. }),
                "expected Pending awaiting rest-pose drain, got {p:?}"
            );
        }
        assert_eq!(
            bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            0,
            "zero-RX-budget pending behind exhausted drain must send ZERO CMD_DISABLE frames"
        );
        // Supervisor gate (`may_disable_torque_while_stopped`) must still refuse while not drained!
        assert_eq!(
            inner.may_disable_torque_while_stopped(),
            Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
            "cached rest pose must NEVER pass supervisor gate while RX drain is incomplete"
        );

        // Second: bounded pre-disable drain (`max_rx_frames = 4`) also exhausts BEFORE reaching
        // the queued raised-pose frame -> STILL stays Pending with ZERO CMD_DISABLE frames!
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let started = drv.clock_now();
            let p = drv
                .begin_disable_torque_budgeted(
                    DisableKind::DisableRequested {
                        started_at: started,
                    },
                    4,
                )
                .expect("L1: exhausted pre-disable drain must keep stopped disable pending");
            assert!(
                matches!(p, TorqueProgress::Pending { .. }),
                "expected Pending behind exhausted pre-disable drain, got {p:?}"
            );
        }
        assert_eq!(
            bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            0,
            "exhausted pre-disable drain must send ZERO CMD_DISABLE frames"
        );

        // Next tick finishes draining the backlog (`mark_rx_empty` clears untrusted backlog
        // timestamps -> M3 defers in AwaitRestGate with 0 CMD_DISABLE frames). Then fresh
        // post-boundary feedback at the raised pose (0.45 rad) refuses immediately as TorqueOffWouldDrop!
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let _ = drv.read_joint_states().unwrap();
            let started = drv.clock_now();
            let p = drv
                .begin_disable_torque_budgeted(
                    DisableKind::DisableRequested {
                        started_at: started,
                    },
                    0,
                )
                .expect("M3: drained stale/missing backlog must defer into AwaitRestGate");
            assert!(matches!(p, TorqueProgress::Pending { .. }));
        }
        assert_eq!(
            bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            0
        );
        assert_eq!(
            inner.may_disable_torque_while_stopped(),
            Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback)
        );
        bus.with(|s| {
            s.motors[0].q = 0.45;
            s.push_all_feedback();
        });
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let dl = drv.clock_now() + Duration::from_millis(5);
            let err = drv.poll_torque_budgeted(dl, 64).unwrap_err();
            assert!(
                matches!(err, ArmDriverError::TorqueOffWouldDrop),
                "M3: fresh raised pose arriving during AwaitRestGate must refuse immediately, got {err:?}"
            );
        }
        assert!(matches!(
            inner.may_disable_torque_while_stopped(),
            Err(StoppedTorqueOffRefusal::JointOutsideRest { joint_index: 0, .. })
        ));

        // Third: contrast case — if the queued traffic behind an exhausted drain turns out to be
        // harmless (arm genuinely remains at rest 0.0), retrying on the next tick after the drain
        // completes and fresh post-boundary feedback arrives (within the 1 s lifetime) SUCCEEDS!
        bus.with(|s| {
            s.rx.clear();
            s.pending.clear();
            s.tx_log.clear();
            s.motors[0].q = 0.0;
            s.push_all_feedback();
        });
        inner.tick(0.02);
        let req_started = clock.now();
        bus.with(|s| {
            for _ in 0..10 {
                s.rx.push_back(junk_frame);
            }
            s.rx.push_back(make_dm_fb(1, 1, 0.0));
        });
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            // Tick 1: 4-frame drain exhausts -> Pending (`AwaitRestGate`, 0 CMD_DISABLE frames).
            let p1 = drv
                .begin_disable_torque_budgeted(
                    DisableKind::DisableRequested {
                        started_at: req_started,
                    },
                    4,
                )
                .expect("L1: exhausted drain must keep disable pending inside 1 s lifetime");
            assert!(matches!(p1, TorqueProgress::Pending { .. }));
            assert_eq!(bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(), 0);

            // Tick 2+ (within 1 s lifetime): polling finishes drain and collects K=3 fresh
            // post-boundary rest samples spanning >= 20 ms -> transmits all 7 CMD_DISABLE frames!
            let mut sent_disables = false;
            for step in 0..8u64 {
                clock.advance(Duration::from_millis(12));
                drv.begin_control_release(100 + step);
                let dl = drv.clock_now() + Duration::from_millis(5);
                let _ = drv
                    .poll_torque_budgeted(dl, 64)
                    .expect("L1 retry poll at rest within 1 s lifetime must not fail");
                if bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count() >= 7 {
                    sent_disables = true;
                    break;
                }
            }
            assert!(sent_disables, "L1 poll must transmit all 7 CMD_DISABLE frames after drain + K=3 rest window");
        }
        assert_eq!(
            bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            7
        );

        // Fourth: protective disable (`DisableProtective::Thermal`) still bypasses even behind an
        // exhausted drain.
        bus.with(|s| {
            s.tx_log.clear();
            for _ in 0..20 {
                s.rx.push_back(junk_frame);
            }
        });
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let p = drv
                .begin_disable_torque_budgeted(
                    DisableKind::DisableProtective {
                        reason: DisableProtectiveReason::Thermal,
                    },
                    4,
                )
                .expect("protective thermal disable must not be blocked by exhausted drain");
            assert!(matches!(p, TorqueProgress::Pending { .. }));
        }
        assert_eq!(
            bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            7
        );
    }

    // 2. RobStride: queued raised-pose frame behind an exhausted drain MUST ALSO refuse!
    {
        let clock = MockClock::new();
        let can = MockCanTransport::new().with_clock(clock.clone());
        let mut rs = RobStrideArmDriver::new(can, vec![1]);
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        rs.set_stopped_disable_gate(StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 1,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        });
        let fb_id = robstride::build_feedback_extended_id(0x00, 1, 0xFD);
        // 0x8000 = 0.0 rad (rest); 0xA000 = raised (~3.14 rad)
        let fb_rest = CanFrame::new_extended(fb_id, &[0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 250]).unwrap();
        let fb_raised = CanFrame::new_extended(fb_id, &[0xA0, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 250]).unwrap();

        // Establish empty-queue bound + fresh resting pose.
        let _ = rs.read_joint_states().unwrap();
        rs.can_mut().provide_rx_frame(fb_rest);
        let _ = rs.read_joint_states().unwrap();
        let _ = rs.can_mut().take_tx_frames();

        // Queue 10 junk frames + raised-pose frame behind them; 4-frame budgeted disable MUST refuse!
        for _ in 0..10 {
            rs.can_mut().provide_rx_frame(junk_frame);
        }
        rs.can_mut().provide_rx_frame(fb_raised);
        let err = rs
            .begin_disable_torque_budgeted(
                DisableKind::DisableRequested {
                    started_at: clock.now(),
                },
                4,
            )
            .unwrap_err();
        assert!(
            matches!(err, ArmDriverError::TorqueOffWouldDrop),
            "RobStride exhausted drain must refuse disable: {err:?}"
        );
        assert!(
            rs.can_mut().take_tx_frames().is_empty(),
            "RobStride exhausted drain refusal must emit 0 stop frames"
        );
    }
}

/// Rule E3: Feetech request/reply correlation.
/// - Before every request, pending RX bytes are discarded.
/// - Replies are accepted ONLY if received within that request's timeout window, after write.
/// - A timed-out request poisons the line: discard until idle before the next request.
/// - Each reply is stamped with ITS OWN receive time, never poll start.
#[test]
fn e3_feetech_delayed_torque_read_rejected_and_delayed_position_never_fresh() {
    use armd::driver::FeetechArmDriver;
    use armd::protocol::feetech;
    use armd::transport::{DriverClock, Transport, TransportError};
    use std::collections::VecDeque;
    use std::sync::Mutex;

    struct TimedReply {
        deliver_at: Instant,
        bytes: Vec<u8>,
    }

    struct ScriptedSerialState {
        clock: MockClock,
        rx: VecDeque<u8>,
        scheduled: Vec<TimedReply>,
        tx_log: Vec<Vec<u8>>,
        /// Optional hook invoked on write to schedule replies relative to write time.
        on_write: Option<Box<dyn FnMut(&[u8], Instant, &mut Vec<TimedReply>) + Send>>,
        /// Step added to clock on empty read (default 1 ms).
        idle_step: Duration,
    }

    #[derive(Clone)]
    struct ScriptedSerial(Arc<Mutex<ScriptedSerialState>>);

    impl ScriptedSerial {
        fn new(clock: MockClock) -> Self {
            Self(Arc::new(Mutex::new(ScriptedSerialState {
                clock,
                rx: VecDeque::new(),
                scheduled: Vec::new(),
                tx_log: Vec::new(),
                on_write: None,
                idle_step: Duration::from_millis(1),
            })))
        }
        fn with<R>(&self, f: impl FnOnce(&mut ScriptedSerialState) -> R) -> R {
            f(&mut self.0.lock().unwrap())
        }
    }

    impl Transport for ScriptedSerial {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(Arc::new(self.0.lock().unwrap().clock.clone()))
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            let mut s = self.0.lock().unwrap();
            let now = s.clock.now();
            let mut ready = Vec::new();
            let mut pending = Vec::new();
            for item in s.scheduled.drain(..) {
                if item.deliver_at <= now {
                    ready.push(item);
                } else {
                    pending.push(item);
                }
            }
            s.scheduled = pending;
            for item in ready {
                s.rx.extend(item.bytes);
            }
            let avail = s.rx.len().min(buf.len());
            if avail == 0 {
                let step = s.idle_step;
                s.clock.advance(step);
                return Ok(0);
            }
            for b in buf.iter_mut().take(avail) {
                *b = s.rx.pop_front().unwrap();
            }
            Ok(avail)
        }
        fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
            let mut s = self.0.lock().unwrap();
            s.tx_log.push(buf.to_vec());
            let now = s.clock.now();
            let mut scheduled = std::mem::take(&mut s.scheduled);
            if let Some(ref mut cb) = s.on_write {
                cb(buf, now, &mut scheduled);
            }
            s.scheduled = scheduled;
            Ok(buf.len())
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        fn clear_input_buffer(&mut self) {
            self.0.lock().unwrap().rx.clear();
        }
    }

    // 1. Delayed torque-read reply arriving after timeout / during next request is REJECTED:
    //    timed-out request poisons the line, forcing discard-until-idle before the next request.
    {
        let clock = MockClock::new();
        let serial = ScriptedSerial::new(clock.clone());
        let late_zero = feetech_reg_u8_resp(1, 0);
        let req_count = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let req_count_cb = req_count.clone();
        let late_zero_cb = late_zero.clone();
        serial.with(|s| {
            s.on_write = Some(Box::new(move |buf, now, sched| {
                // Only fire on REG_TORQUE_ENABLE read requests.
                if buf == feetech::build_read(1, feetech::REG_TORQUE_ENABLE, 1).as_slice() {
                    let n = req_count_cb.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                    if n == 0 {
                        // Request 1 reply arrives at +27 ms (2 ms PAST the 25 ms read timeout),
                        // right in the middle of Request 2's pre-request window!
                        sched.push(TimedReply {
                            deliver_at: now + Duration::from_millis(27),
                            bytes: late_zero_cb.clone(),
                        });
                    }
                    // Request 2 itself receives NO genuine reply from the servo.
                }
            }));
        });
        let mut drv = FeetechArmDriver::new(serial.clone(), vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));

        // Request 1: times out at 25 ms -> poisons the line!
        let err1 = drv
            .read_register_u8_direct(1, feetech::REG_TORQUE_ENABLE)
            .unwrap_err();
        assert!(
            matches!(err1, armd::driver::ArmDriverError::Timeout(1)),
            "first torque read must time out: {err1:?}"
        );

        // Request 2 immediately after: MUST discard Request 1's late 0x0 reply (arriving at +27 ms)
        // during its poisoned-line discard-until-idle pass BEFORE writing Request 2, and because
        // Request 2 gets no genuine reply of its own, MUST fail (NEVER accept 0x0 as OffConfirmed)!
        let err2 = drv
            .read_register_u8_direct(1, feetech::REG_TORQUE_ENABLE)
            .unwrap_err();
        assert!(
            matches!(err2, armd::driver::ArmDriverError::Timeout(1)),
            "delayed torque-read reply from request 1 MUST be discarded/rejected, not returned to request 2: {err2:?}"
        );
        assert_eq!(drv.enable_state(), EnableState::Unknown);

        // Also verify pre-queued stale RX bytes already in the buffer before a request are discarded.
        serial.with(|s| {
            s.rx.extend(late_zero.iter().copied());
        });
        let err3 = drv
            .read_register_u8_direct(1, feetech::REG_TORQUE_ENABLE)
            .unwrap_err();
        assert!(
            matches!(err3, armd::driver::ArmDriverError::Timeout(1)),
            "pending pre-request RX bytes must be discarded before transmitting: {err3:?}"
        );
    }

    // 2. Delayed position reply (arriving past the 4 ms CHUNK_TIMEOUT) is NEVER accepted or
    //    stamped as fresh!
    {
        let clock = MockClock::new();
        let serial = ScriptedSerial::new(clock.clone());
        let pos_bytes = feetech_feedback_bytes(&[1], &[2048]);
        serial.with(|s| {
            s.idle_step = Duration::from_micros(500);
            let pos_bytes = pos_bytes.clone();
            s.on_write = Some(Box::new(move |buf, now, sched| {
                if buf
                    == feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8)
                        .unwrap()
                        .as_slice()
                {
                    // Reply arrives at +4.2 ms (PAST the 4 ms CHUNK_TIMEOUT, inside the old
                    // 500 us post-timeout drain window that previously stamped it fresh!).
                    sched.push(TimedReply {
                        deliver_at: now + Duration::from_micros(4200),
                        bytes: pos_bytes.clone(),
                    });
                }
            }));
        });
        let mut drv = FeetechArmDriver::new(serial.clone(), vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));

        let _ = drv.read_joint_states().unwrap();
        assert!(
            drv.feedback_receive_instants()[0].is_none(),
            "delayed position reply past CHUNK_TIMEOUT must NEVER be accepted or stamped fresh"
        );
        let mut pos_out = [0.0f32; 1];
        assert!(
            !drv.fresh_joint_positions_into(&[0], Duration::from_millis(100), &mut pos_out),
            "delayed position reply must never produce fresh rest-pose evidence"
        );
    }

    // 3. Per-reply receive timestamps (NEVER poll start): in one SYNC_READ poll for servos [1, 2],
    //    servo 1 arrives at req_written + 1 ms and servo 2 arrives at req_written + 3 ms.
    //    Each servo MUST be stamped with ITS OWN receive instant!
    {
        let clock = MockClock::new();
        let serial = ScriptedSerial::new(clock.clone());
        let fb1 = feetech_feedback_bytes(&[1], &[2048]);
        let fb2 = feetech_feedback_bytes(&[2], &[2100]);
        let written_at = Arc::new(Mutex::new(None::<Instant>));
        let written_at_cb = written_at.clone();
        serial.with(|s| {
            s.idle_step = Duration::from_millis(1);
            s.on_write = Some(Box::new(move |buf, now, sched| {
                if buf
                    == feetech::build_sync_read(&[1, 2], feetech::REG_PRESENT_POSITION_L, 8)
                        .unwrap()
                        .as_slice()
                {
                    *written_at_cb.lock().unwrap() = Some(now);
                    sched.push(TimedReply {
                        deliver_at: now + Duration::from_millis(1),
                        bytes: fb1.clone(),
                    });
                    sched.push(TimedReply {
                        deliver_at: now + Duration::from_millis(3),
                        bytes: fb2.clone(),
                    });
                }
            }));
        });
        let mut drv = FeetechArmDriver::new(serial.clone(), vec![1, 2])
            .with_calibrated(true)
            .with_tx_echo(Some(false));

        let poll_start = clock.now();
        let _ = drv.read_joint_states().unwrap();
        let req_ts = written_at.lock().unwrap().expect("sync read written");
        let rx_instants = drv.feedback_receive_instants();
        let t1 = rx_instants[0].expect("servo 1 timestamp");
        let t2 = rx_instants[1].expect("servo 2 timestamp");
        assert!(t1 > poll_start, "servo 1 must not be stamped with poll_start");
        assert_eq!(
            t1.saturating_duration_since(req_ts),
            Duration::from_millis(1),
            "servo 1 must be stamped with its own receive instant (+1 ms)"
        );
        assert_eq!(
            t2.saturating_duration_since(req_ts),
            Duration::from_millis(3),
            "servo 2 must be stamped with its own receive instant (+3 ms)"
        );
        assert!(t2 > t1, "distinct reply arrivals must carry distinct receive timestamps");
    }
}

/// Rule E4: Damiao mode evidence.
/// A Damiao hold is sent ONLY in a VERIFIED mode, from a CTRL_MODE readback after this daemon
/// start. If the mode is not verified, the joint is unheld (`UnheldNoVerifiedMode`, 0 frames),
/// never assumed `PosVel`.
#[test]
fn e4_damiao_unverified_mode_unheld_never_assumed_posvel() {
    use armd::driver::{DamiaoControlMode, JointHoldStatus, JointTarget};

    // 1. Fresh DamiaoArmDriver after daemon start (no CTRL_MODE readback yet, even with
    //    `with_control_mode(PosVel)`): every joint MUST be unverified, reported
    //    `UnheldNoVerifiedMode`, and transmit ZERO hold frames!
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&limits(), clock.clone());
    bus.with(|s| {
        s.follow_commands = false;
        for m in &mut s.motors {
            m.q = 0.25;
            m.enabled = false;
        }
        s.push_all_feedback();
    });
    let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits())
        .unwrap()
        .with_control_mode(DamiaoControlMode::PosVel);
    // Establish empty-queue receive bound first, then deliver fresh position feedback.
    dm.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    dm.read_joint_states().unwrap();
    for j in 0..7 {
        assert!(
            !dm.joint_hold_mode_verified(j),
            "fresh driver without CTRL_MODE readback must NOT claim verified hold mode on J{}",
            j + 1
        );
    }
    let targets: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.25)).collect();
    bus.clear_log();
    let report = dm.write_measured_hold_masked(0b111_1111, &targets).unwrap();
    assert!(!report.sent_any, "unverified driver must transmit ZERO hold frames");
    assert_eq!(
        report.joint_status[..7],
        [JointHoldStatus::UnheldNoVerifiedMode; 7],
        "unverified joints must be reported UnheldNoVerifiedMode, never assumed PosVel"
    );
    assert!(
        bus.tx_log().iter().all(|f| !(0x101..=0x107).contains(&f.id) && !(1..=7).contains(&f.id)),
        "unverified mode must emit ZERO POS_VEL or MIT hold frames: {:?}",
        bus.tx_log()
    );

    // 2. Startup `check_mapping_ranges()` with J3 silent (no CTRL_MODE readback on J3) and
    //    J1, J2, J4..J7 answering CTRL_MODE=2: J3 MUST remain unheld (`UnheldNoVerifiedMode`,
    //    0 frames to J3) while answering peers ARE held!
    bus.with(|s| {
        s.motors[2].silent = true;
    });
    let _ = dm.check_mapping_ranges();
    assert!(!dm.joint_hold_mode_verified(2), "silent J3 must remain unverified");
    for j in [0, 1, 3, 4, 5, 6] {
        assert!(
            dm.joint_hold_mode_verified(j),
            "J{} answered CTRL_MODE=2 readback and must be verified",
            j + 1
        );
    }
    bus.with(|s| {
        s.motors[2].silent = false;
        s.push_all_feedback();
    });
    dm.read_joint_states().unwrap();
    bus.clear_log();
    let report2 = dm.write_measured_hold_masked(0b111_1111, &targets).unwrap();
    assert!(report2.sent_any);
    assert_eq!(
        report2.joint_status[2],
        JointHoldStatus::UnheldNoVerifiedMode,
        "silent J3 must stay UnheldNoVerifiedMode even after receiving fresh position feedback"
    );
    for j in [0, 1, 3, 4, 5, 6] {
        assert_eq!(report2.joint_status[j], JointHoldStatus::Held);
    }
    let hold_ids: Vec<u32> = bus
        .tx_log()
        .iter()
        .filter(|f| (0x101..=0x107).contains(&f.id) || (1..=7).contains(&f.id))
        .map(|f| f.id)
        .collect();
    assert!(
        !hold_ids.contains(&0x103) && !hold_ids.contains(&3),
        "unverified J3 must receive ZERO hold frames, got {hold_ids:?}"
    );
    assert_eq!(hold_ids.len(), 6, "only the 6 verified peers may receive hold frames: {hold_ids:?}");

    // 3. Supervisor `FailedHold` propagates `UnheldNoVerifiedMode` into `joint_hold_status()` and
    //    `ENABLE_FAILED` fault message when J3 provides fresh feedback but never answers
    //    `RID_CTRL_MODE`; once J3 subsequently answers causal `CTRL_MODE` readback, J3 becomes held!
    bus.with(|s| {
        s.motors[2].regs.remove(&damiao::RID_CTRL_MODE);
        s.motors[2].drop_torque_frames = 10; // fail enable so supervisor enters FailedHold
        s.push_all_feedback();
    });
    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    })
    .with_driver(Box::new(dm));
    inner.lease_enabled = false;
    assert!(enable_torque_admitted(&mut inner).is_err());
    assert_eq!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
    assert_eq!(
        inner.joint_hold_status()[2],
        JointHoldStatus::UnheldNoVerifiedMode,
        "supervisor must report unverified J3 as UnheldNoVerifiedMode"
    );
    let fault_msg = inner.safety.fault().expect("ENABLE_FAILED").message;
    assert!(
        fault_msg.contains("J3:unheld(no_mode)"),
        "ENABLE_FAILED fault message must report J3 unheld(no_mode): {fault_msg}"
    );
    // Tick while J3 mode is still unverified -> J3 receives ZERO hold frames while peers hold.
    bus.with(|s| s.push_all_feedback());
    bus.clear_log();
    inner.tick(0.02);
    assert_eq!(inner.joint_hold_status()[2], JointHoldStatus::UnheldNoVerifiedMode);
    assert!(
        !bus.tx_log().iter().any(|f| f.id == 0x103 || f.id == 3),
        "FailedHold tick must emit ZERO hold frames for unverified J3"
    );
}

/// Rule T1: Thermal checks are active during `Enabling`.
/// Verifies:
/// 1. RT `TransitionOp::TorqueOn` in `Enabling` (pending motor reply) receiving an over-temperature
///    frame trips `ThermalTrip`, delivers `Outcome::Failed("THERMAL_TRIP")` to the broker, never
///    arms, and transmits protective `CMD_DISABLE` even at a raised pose.
/// 2. RT `TransitionOp::TorqueOn` whose confirming feedback frame carries an over-temperature
///    sample trips inside `step_enable_validate` BEFORE arming (never returns `Completed` or arms).
/// 3. Blocking `enable_torque()` trips `ThermalTrip` while `Enabling` when an over-temperature
///    sample arrives, never arms, and transmits protective `CMD_DISABLE`.
#[test]
fn t1_thermal_checks_active_during_enabling_rt_and_legacy() {
    use armd::rt_core::broker::{Outcome, Reply};
    use armd::rt_core::ops::VirtualClock;

    // 1. RT `TransitionOp::TorqueOn` still Pending in `Enabling` when J1 reports 92 °C (> 70 °C):
    //    thermal protection MUST trip immediately, fail the transition with `THERMAL_TRIP`,
    //    never arm, and send protective `CMD_DISABLE` at a raised pose.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.40; // raised pose (not at rest)
                m.enabled = false;
                m.temperature_c = 35;
                m.drop_torque_frames = 10; // keep TorqueOn pending
            }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            max_temperature: 70.0,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.rt_executor_owned = true;
        inner.tick(0.02);

        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BootId(101),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(clock.clone()),
        );
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        broker
            .submit(
                ClientId(1),
                1,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::Enabling);
        assert!(fx.core.torque_transition_pending());
        assert!(!fx.core.is_torqued);

        // Inject over-temperature on J1 while enable is still Pending in Enabling.
        bus.clear_log();
        bus.with(|s| {
            s.motors[0].temperature_c = 92;
            s.push_all_feedback();
        });
        let rep = fx.step();
        assert!(
            fx.core.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip),
            "T1: over-temp during Pending Enabling must latch ThermalTrip"
        );
        assert_eq!(fx.core.safety.fault_code(), Some(FaultCode::ThermalTrip));
        assert!(
            !matches!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::Armed),
            "T1: thermal trip during Enabling must never arm"
        );
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "T1: thermal trip during Enabling must transmit protective CMD_DISABLE at raised pose"
        );
        assert_eq!(
            rep.transition_ended.map(|(_, o)| o),
            Some(Outcome::Failed("THERMAL_TRIP")),
            "T1: broker transition must end with THERMAL_TRIP"
        );
        let mut terminals = Vec::new();
        broker.poll(|_, r| {
            if let Reply::Terminal { outcome, .. } = r {
                terminals.push(outcome);
            }
            true
        });
        assert_eq!(
            terminals,
            vec![Outcome::Failed("THERMAL_TRIP")],
            "T1: broker must receive exactly one THERMAL_TRIP terminal"
        );
    }

    // 2. RT `TransitionOp::TorqueOn` where the confirming feedback frame itself carries 95 °C:
    //    `step_enable_validate` MUST trip ThermalTrip BEFORE arming (never returns Completed).
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.40;
                m.enabled = false;
                m.temperature_c = 35;
            }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            max_temperature: 70.0,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.rt_executor_owned = true;
        inner.tick(0.02);

        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BootId(102),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(clock.clone()),
        );
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        // Delay J3's enable confirmation reply by 25 ms AND set its temperature to 95 °C before
        // `CMD_ENABLE` is transmitted (without pushing feedback yet), so release 1 stays in
        // `Enabling` and release 2 receives the delayed confirming frame carrying 95 °C.
        bus.with(|s| {
            s.motors[2].torque_reply_delay = Duration::from_millis(25);
            s.motors[2].temperature_c = 95;
        });
        broker
            .submit(
                ClientId(1),
                2,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());

        bus.clear_log();
        let mut terminal_outcomes = Vec::new();
        for _ in 0..4 {
            let _ = fx.step();
            broker.poll(|_, r| {
                if let Reply::Terminal { outcome, .. } = r {
                    terminal_outcomes.push(outcome);
                }
                true
            });
        }
        assert!(
            fx.core.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip),
            "T1: confirming enable frame with 95 °C must latch ThermalTrip before arming"
        );
        assert!(
            !matches!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::Armed),
            "T1: arm must NEVER reach Armed when confirming frame is over-temperature"
        );
        assert_eq!(
            terminal_outcomes,
            vec![Outcome::Failed("THERMAL_TRIP")],
            "T1: confirming over-temp frame must deliver THERMAL_TRIP, never Completed"
        );
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "T1: protective CMD_DISABLE must be sent when confirming frame trips thermal"
        );
    }

    // 3. Blocking `enable_torque()` when over-temperature arrives during `Enabling`:
    //    must fail (never arm), latch `ThermalTrip`, and transmit protective `CMD_DISABLE`.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.40;
                m.enabled = false;
                m.temperature_c = 35;
            }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            max_temperature: 70.0,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.tick(0.02);

        // Over-temperature arrives on the motor replies during blocking enable_torque().
        bus.clear_log();
        bus.with(|s| {
            s.motors[1].temperature_c = 88;
        });
        let res = enable_torque_admitted(&mut inner);
        assert!(res.is_err(), "T1: blocking enable_torque must fail on over-temperature during Enabling");
        assert!(
            inner.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip),
            "T1: blocking enable_torque must latch ThermalTrip"
        );
        assert!(!inner.is_torqued, "T1: blocking enable_torque must confirm protective disable");
        assert!(
            !matches!(inner.enable_lifecycle(), armd::supervisor::EnableLifecycle::Armed),
            "T1: blocking enable_torque must never arm on over-temperature"
        );
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "T1: blocking enable_torque thermal trip must transmit protective CMD_DISABLE"
        );
    }
}

/// Rule T2: A thermal trip must not discard a pending `Disable`.
/// Verifies:
/// 1. Broker submits `TransitionOp::TorqueOff` (in-flight `TorqueTransition::Disable`); `< 250 ms`
///    later, over-temperature arrives on Step 2 (even while the arm drifts to a raised pose where
///    `DisableRequested` retries would refuse): `ThermalTrip` latches, the 250 ms rate limit is
///    bypassed, `DisableProtective` transmits immediately, `TorqueTransition::Disable` completes,
///    and the broker receives `Reply::Terminal` **EXACTLY ONCE** (never dropped / never duplicated).
/// 2. Thermal trip during a pending broker `Disable` when protective transmit fails with a bus
///    error delivers **EXACTLY ONE** `Reply::Terminal` (`Outcome::Failed`) to the broker while
///    keeping `disable_pending` + `DisableProtective` armed for background recovery once the bus
///    recovers.
#[test]
fn t2_thermal_trip_upgrades_pending_disable_with_single_broker_terminal() {
    use armd::rt_core::broker::{Outcome, Reply};
    use armd::rt_core::ops::VirtualClock;

    // 1. In-flight broker `TransitionOp::TorqueOff` interrupted < 250 ms later by over-temperature
    //    at a raised pose: MUST NOT discard the pending Disable, MUST bypass the 250 ms throttle
    //    to transmit `DisableProtective`, MUST confirm disable, and MUST deliver exactly 1 terminal.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let rest = ArmConfig::rebot_dm().resting_positions().to_vec();
        bus.with(|s| {
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.enabled = true;
                m.temperature_c = 35;
            }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            max_temperature: 70.0,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.rt_executor_owned = true;
        // Arm normally first.
        inner.tick(0.02);
        assert!(enable_torque_admitted(&mut inner).is_ok());
        assert!(inner.is_torqued);

        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BootId(201),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(clock.clone()),
        );
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        let req_id = broker
            .submit(
                ClientId(1),
                1,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert!(fx.core.torque_transition_pending(), "T2: Disable must be in flight");
        assert!(fx.transition().is_some(), "T2: ReleaseEngine must hold installed Disable");

        // Advance 1 step so `AwaitRestGate` is still collecting K=3 (pending); J2 hits 94 °C
        // (> 70 °C) while `Disable` is pending, upgrading the pending broker transition to
        // `DisableProtective { Thermal }`. Then the arm drifts to a raised pose (`0.50 rad`),
        // proving `DisableProtective` completes at a raised pose and delivers exactly 1 terminal.
        bus.clear_log();
        bus.with(|s| {
            s.motors[1].temperature_c = 94;
            s.push_all_feedback();
        });
        let _ = fx.step();
        bus.with(|s| {
            for m in &mut s.motors {
                m.q = 0.50; // raised pose (rest-pose gate would refuse DisableRequested)
                m.drop_torque_frames = 0;
            }
            s.push_all_feedback();
        });

        let mut terminals = Vec::new();
        broker.poll(|_, r| {
            if let Reply::Terminal { request, outcome, .. } = r {
                terminals.push((request, outcome));
            }
            true
        });
        for _ in 0..8 {
            let _ = fx.step();
            broker.poll(|_, r| {
                if let Reply::Terminal { request, outcome, .. } = r {
                    terminals.push((request, outcome));
                }
                true
            });
        }
        assert!(
            fx.core.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip),
            "T2: over-temperature during pending Disable must latch ThermalTrip"
        );
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "T2: thermal upgrade must bypass 250 ms throttle and transmit DisableProtective at raised pose"
        );
        assert!(!fx.core.is_torqued, "T2: DisableProtective must confirm torque off");
        assert!(!fx.core.disable_pending, "T2: disable_pending must clear after confirmation");
        assert!(fx.transition().is_none(), "T2: ReleaseEngine must not retain orphaned transition");
        assert_eq!(
            terminals,
            vec![(req_id, Outcome::Completed)],
            "T2: broker request for pending Disable must receive terminal EXACTLY ONCE"
        );
    }

    // 2. Thermal trip during a pending broker `Disable` while CAN TX returns errors:
    //    broker receives EXACTLY ONE terminal (`Outcome::Failed`), `ThermalTrip` latches, and
    //    `disable_pending` + `DisableProtective` remain armed so disable confirms once TX recovers.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let rest = ArmConfig::rebot_dm().resting_positions().to_vec();
        bus.with(|s| {
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = rest[i];
                m.enabled = true;
                m.temperature_c = 35;
            }
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            max_temperature: 70.0,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.rt_executor_owned = true;
        inner.tick(0.02);
        assert!(enable_torque_admitted(&mut inner).is_ok());

        bus.with(|s| {
            for m in &mut s.motors {
                m.drop_torque_frames = 10;
            }
        });

        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BootId(202),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(clock.clone()),
        );
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        let req_id = broker
            .submit(
                ClientId(1),
                2,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert!(fx.core.torque_transition_pending());

        // Inject bus TX error + over-temperature on Step 2.
        bus.with(|s| {
            s.tx_error = true;
            s.motors[0].temperature_c = 95;
            s.push_all_feedback();
        });
        let mut terminals = Vec::new();
        for _ in 0..3 {
            let _ = fx.step();
            broker.poll(|_, r| {
                if let Reply::Terminal { request, outcome, .. } = r {
                    terminals.push((request, outcome));
                }
                true
            });
        }
        assert!(fx.core.safety.has_latched_fault_kind(armd::safety::FaultKind::ThermalTrip));
        assert!(fx.core.disable_pending, "T2: protective disable must stay pending after TX error");
        assert_eq!(
            terminals.len(),
            1,
            "T2: broker must receive EXACTLY ONE terminal on thermal TX error, got {terminals:?}"
        );
        assert_eq!(terminals[0].0, req_id);
        assert!(matches!(terminals[0].1, Outcome::Failed(_)));

        // Restore CAN bus and advance past the 250 ms background retry cadence (wall-clock retry
        // check on `last_disable_attempt`): clear `last_disable_attempt` to trigger background retry
        // deterministically without sleeping, then step past the 20 ms settle window.
        bus.clear_log();
        bus.with(|s| {
            s.tx_error = false;
            for m in &mut s.motors {
                m.drop_torque_frames = 0;
                m.q = 0.50; // raised pose
            }
        });
        fx.core.last_disable_attempt = None;
        for _ in 0..9 {
            let _ = fx.step();
            broker.poll(|_, r| {
                if let Reply::Terminal { request, outcome, .. } = r {
                    terminals.push((request, outcome));
                }
                true
            });
        }
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "T2: background thermal DisableProtective retry must transmit at raised pose"
        );
        assert!(!fx.core.is_torqued && !fx.core.disable_pending, "T2: background protective disable must confirm");
        assert_eq!(
            terminals.len(),
            1,
            "T2: background thermal completion must NEVER emit a second broker terminal"
        );
    }
}

/// R10 Test 1: Holds are the default — after stop, fault, E-stop, or FailedHold, every joint
/// continues receiving measured-position hold frames on every tick even when motor feedback
/// claims every motor is off (`state=0` / `EnableState::Disabled`). Reported enable state is
/// telemetry only and never stops holds or makes `shutdown_ready()` true.
#[test]
fn r10_holds_continue_after_stop_and_fault_when_feedback_claims_all_motors_off() {
    use armd::driver::{FeetechArmDriver, JointHoldStatus};
    use armd::transport::MockTransport;
    use armd::TorqueIntent;

    let pose = [0.20_f32, 0.35, -0.25, 0.15, -0.10, 0.12, 0.0];

    // Part A: Damiao legacy owner — Fault + E-stop with all 7 motors reporting state=0 (Disabled telemetry).
    {
        let (mut inner, bus, clock) = make_driver_rig();
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = pose[i];
                m.enabled = false;
            }
            s.push_all_feedback();
        });
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();
        assert_eq!(inner.torque_intent, TorqueIntent::On);

        // Fault the supervisor and force EVERY motor's telemetry to claim state=0 (Disabled).
        inner.safety.latch(
            armd::safety::FaultKind::WatchdogTimeout,
            armd::safety::FaultDetail::WatchdogTimeout,
        );
        bus.with(|s| {
            for m in &mut s.motors {
                m.enabled = false;
            }
        });

        for tick_idx in 0..5 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            bus.clear_log();
            inner.tick(0.02);

            let obs = inner.current_observation();
            assert_eq!(
                obs.enable_state,
                EnableState::Disabled,
                "telemetry must honestly report motor state=0 on tick {tick_idx}"
            );
            assert_eq!(inner.torque_intent, TorqueIntent::On);
            assert!(inner.is_torqued, "holds must stay active despite state=0 telemetry");
            assert!(
                !inner.shutdown_ready(),
                "shutdown_ready must remain false until an explicit disable is accepted"
            );
            let hold_ids: Vec<u32> = bus
                .tx_log()
                .iter()
                .filter(|f| (0x101..=0x107).contains(&f.id))
                .map(|f| f.id)
                .collect();
            assert_eq!(
                hold_ids.len(),
                7,
                "tick {tick_idx}: EVERY joint must receive a hold frame even with state=0 telemetry, got {hold_ids:?}"
            );
            assert_eq!(inner.joint_hold_status(), &[JointHoldStatus::Held; 7]);
        }

        // Clearing the recoverable fault keeps torque_intent=On and holds active (never drops torque).
        inner.clear_fault().unwrap();
        assert_eq!(inner.torque_intent, TorqueIntent::On);
        assert!(inner.is_torqued);
        assert!(!inner.shutdown_ready());
    }

    // Part B: Damiao RT executor owner — E-stop with all 7 motors reporting state=0 keeps holding every tick.
    {
        let mut r = ExecRig::new();
        r.bus.with(|s| {
            s.follow_commands = false;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = pose[i];
            }
            s.push_all_feedback();
        });
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            for m in &mut s.motors {
                m.enabled = false;
            }
        });
        for tick_idx in 0..4 {
            r.bus.clear_log();
            r.step_with_feedback();
            let obs = r.fx.core.current_observation();
            assert_eq!(obs.enable_state, EnableState::Disabled);
            assert_eq!(r.fx.core.torque_intent, TorqueIntent::On);
            assert!(r.fx.core.is_torqued);
            let hold_ids: Vec<u32> = r
                .bus
                .tx_log()
                .iter()
                .filter(|f| (0x101..=0x107).contains(&f.id))
                .map(|f| f.id)
                .collect();
            assert_eq!(
                hold_ids.len(),
                7,
                "RT tick {tick_idx}: EVERY joint must receive a hold frame despite state=0 telemetry"
            );
        }
    }

    // Part C: Feetech — after fault with all 6 servos reporting REG_TORQUE_ENABLE=0, holds continue for ALL 6 servos.
    {
        #[derive(Clone)]
        struct SharedSerial(Arc<std::sync::Mutex<MockTransport>>);
        impl armd::transport::Transport for SharedSerial {
            fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
                self.0.lock().unwrap().write(buf)
            }
            fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
                self.0.lock().unwrap().write_all(buf)
            }
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
                self.0.lock().unwrap().read(buf)
            }
            fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
                self.0.lock().unwrap().flush()
            }
            fn clear_input_buffer(&mut self) {
                self.0.lock().unwrap().clear_input_buffer();
            }
            fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
                armd::transport::Transport::clock(&*self.0.lock().unwrap())
            }
        }

        let clock = MockClock::new();
        let servo_ids: Vec<u8> = (1..=6).collect();
        let raised_steps: [u16; 6] = [2250, 2400, 1900, 2150, 2000, 2100];
        let serial = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let req1 = armd::protocol::feetech::build_sync_read(
            &servo_ids[..3],
            armd::protocol::feetech::REG_PRESENT_POSITION_L,
            8,
        )
        .unwrap();
        let req2 = armd::protocol::feetech::build_sync_read(
            &servo_ids[3..],
            armd::protocol::feetech::REG_PRESENT_POSITION_L,
            8,
        )
        .unwrap();
        {
            let mut ser = serial.lock().unwrap();
            ser.stage_reply(&req1, &feetech_feedback_bytes(&servo_ids[..3], &raised_steps[..3]));
            ser.stage_reply(&req2, &feetech_feedback_bytes(&servo_ids[3..], &raised_steps[3..]));
            for id in 1..=6u8 {
                let r_en = armd::protocol::feetech::build_read(
                    id,
                    armd::protocol::feetech::REG_TORQUE_ENABLE,
                    1,
                );
                ser.stage_reply(&r_en, &feetech_reg_u8_resp(id, 1));
            }
        }
        let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial)), servo_ids.clone())
            .with_calibrated(true)
            .with_tx_echo(Some(false))
            .with_gripper_index(Some(5));
        drv.read_joint_states().unwrap();

        let mut inner = SupervisorInner::new(feetech_6dof_config()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        enable_torque_admitted(&mut inner).unwrap();
        assert!(inner.is_torqued);

        // Now flip Feetech REG_TORQUE_ENABLE telemetry to 0 (Disabled) while raised and E-stop.
        {
            let mut ser = serial.lock().unwrap();
            ser.clear();
            ser.clear_write_responses();
            ser.stage_reply(&req1, &feetech_feedback_bytes(&servo_ids[..3], &raised_steps[..3]));
            ser.stage_reply(&req2, &feetech_feedback_bytes(&servo_ids[3..], &raised_steps[3..]));
            for id in 1..=6u8 {
                let r_en = armd::protocol::feetech::build_read(
                    id,
                    armd::protocol::feetech::REG_TORQUE_ENABLE,
                    1,
                );
                ser.stage_reply(&r_en, &feetech_reg_u8_resp(id, 0));
            }
        }
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert_eq!(drv.refresh_enable_state().unwrap(), EnableState::Disabled);
        }
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        let _ = serial.lock().unwrap().take_tx();

        clock.advance(Duration::from_millis(20));
        inner.tick(0.02);
        let tx = serial.lock().unwrap().take_tx();
        let sync_writes = parse_feetech_sync_write_goal_steps(&tx);
        assert_eq!(sync_writes.len(), 1, "Feetech must emit measured hold SYNC_WRITE despite Disabled telemetry");
        assert_eq!(
            sync_writes[0].iter().map(|&(id, _)| id).collect::<Vec<_>>(),
            vec![1, 2, 3, 4, 5, 6],
            "ALL 6 Feetech servos must be held despite Disabled telemetry"
        );
        assert_eq!(inner.torque_intent, TorqueIntent::On);
        assert!(inner.is_torqued);
    }
}

/// R10 Test 2: Disable at rest is accepted, transmits bounded repeats (`DAMIAO_DISABLE_REPEATS`),
/// records `torque_intent = Off`, and stops holds; disable at a raised pose is refused with
/// `TORQUE_OFF_WOULD_DROP` (`0` disable frames) and holds continue.
#[test]
fn r10_disable_at_rest_accepted_stops_holds_while_raised_refused_and_holds_continue() {
    use armd::driver::DAMIAO_DISABLE_REPEATS;
    use armd::TorqueIntent;

    // Part A: Legacy owner (raised refused + holds continue; rest accepted + bounded repeats + holds stop).
    {
        let (mut inner, bus, clock) = make_driver_rig();
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = if i == 1 { 0.45 } else { 0.0 };
            }
            s.push_all_feedback();
        });
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }

        // Raised pose: disable refused with TorqueOffWouldDrop, 0 disables, holds continue.
        bus.clear_log();
        let err = inner.disable_torque().unwrap_err();
        assert!(matches!(err, SafetyError::TorqueOffWouldDrop { .. }), "expected TorqueOffWouldDrop, got {err:?}");
        assert_eq!(spec_count_damiao_disables(&bus), 0);
        assert_eq!(inner.torque_intent, TorqueIntent::On);
        assert!(inner.is_torqued);

        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        assert_eq!(
            bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
            7,
            "holds must continue after refused raised disable"
        );

        // Move to rest pose and collect K=3 releases spanning >= 20 ms.
        bus.with(|s| {
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
        // Even if motors never reply to CMD_DISABLE, bounded repeats finish and stop holds by decision!
        bus.with(|s| {
            s.immediate_replies = false;
            for m in &mut s.motors {
                m.drop_torque_frames = 20;
            }
        });
        bus.clear_log();
        let st = inner.disable_torque().expect("disable at rest must succeed");
        assert!(matches!(st, SafetyState::EStop));
        assert_eq!(
            spec_count_damiao_disables(&bus),
            7 * usize::from(DAMIAO_DISABLE_REPEATS),
            "accepted disable must transmit bounded DAMIAO_DISABLE_REPEATS per motor"
        );
        assert_eq!(inner.torque_intent, TorqueIntent::Off);
        assert!(!inner.is_torqued);
        assert!(inner.shutdown_ready());

        // Subsequent ticks emit ZERO hold frames!
        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            bus.clear_log();
            inner.tick(0.02);
            assert_eq!(
                bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
                0,
                "accepted disable must stop all hold frames on subsequent ticks"
            );
        }
    }

    // Part B: RT executor owner (raised refused + holds continue; rest accepted + holds stop; re-enable -> On).
    {
        let mut r = ExecRig::new();
        r.bus.with(|s| {
            s.follow_commands = false;
            s.motors[1].q = 0.45;
            s.push_all_feedback();
        });
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        for _ in 0..3 {
            r.step_with_feedback();
        }
        r.bus.clear_log();
        r.submit_transition(TransitionOp::TorqueOff);
        let first = r.step();
        let err = first.command.expect("command reply").1.unwrap_err();
        assert_eq!(err, "TORQUE_OFF_WOULD_DROP");
        assert_eq!(spec_count_damiao_disables(&r.bus), 0);
        assert_eq!(r.fx.core.torque_intent, TorqueIntent::On);
        assert!(r.fx.core.is_torqued);
        r.bus.clear_log();
        r.step_with_feedback();
        assert_eq!(
            r.bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
            7,
            "RT holds must continue after refused raised disable"
        );

        // Move to rest pose -> K=3 releases -> TorqueOff accepted -> holds stop!
        r.bus.with(|s| {
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        for _ in 0..3 {
            r.step_with_feedback();
        }
        r.bus.clear_log();
        r.submit_transition(TransitionOp::TorqueOff);
        let mut outcome = r.step().transition_ended.map(|(_, o)| o);
        for _ in 0..10 {
            if outcome.is_some() {
                break;
            }
            if let Some((_, o)) = r.step_with_feedback().transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(outcome, Some(Outcome::Completed));
        assert_eq!(spec_count_damiao_disables(&r.bus), 7 * usize::from(DAMIAO_DISABLE_REPEATS));
        assert_eq!(r.fx.core.torque_intent, TorqueIntent::Off);
        assert!(!r.fx.core.is_torqued);

        r.bus.clear_log();
        r.step_with_feedback();
        assert_eq!(
            r.bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
            0,
            "RT holds must stop after accepted TorqueOff"
        );
    }
}

/// R10 Test 3: `CLOCK_REALTIME` step (> 1 ms) between motion/enable write and decode discards the
/// sample, restarts the rest-pose window, and re-anchors the boundary (nothing accepted until a
/// fresh stable post-settle K=3 window completes). Also verifies S1 pre-write kernel timestamp
/// rejection and 20 ms settle window on both Damiao and RobStride.
#[test]
fn r10_realtime_clock_step_between_write_and_decode_restarts_rest_window() {
    use armd::driver::{
        ArmDriver, DamiaoArmDriver, JointTarget, RestPoseGateStatus, RobStrideArmDriver,
        StoppedDisableGate,
    };
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::protocol::robstride::{self, RobStrideLimits};
    use armd::transport::{
        CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
    };
    use std::collections::VecDeque;

    struct KernelTsBus {
        clock: MockClock,
        base_real_ns: i128,
        realtime_step_ns: i128,
        base_mono: std::time::Instant,
        rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    }

    impl KernelTsBus {
        fn new(clock: MockClock) -> Self {
            let base_mono = clock.now();
            Self {
                clock,
                base_real_ns: 1_700_000_000_000_000_000,
                realtime_step_ns: 0,
                base_mono,
                rx: VecDeque::new(),
            }
        }
        fn mono_elapsed_ns(&self) -> i128 {
            self.clock.now().saturating_duration_since(self.base_mono).as_nanos() as i128
        }
        fn current_kernel_ns(&self) -> i128 {
            self.base_real_ns + self.mono_elapsed_ns() + self.realtime_step_ns
        }
        fn current_offset_ns(&self) -> i128 {
            self.base_real_ns + self.realtime_step_ns
        }
        fn step_realtime_ns(&mut self, delta_ns: i128) {
            self.realtime_step_ns += delta_ns;
        }
        fn push_kernel_ts(&mut self, frame: CanFrame, kernel_ns: i128) {
            self.rx.push_back((frame, Some(CanRxTime::KernelRealtimeNs(kernel_ns))));
        }
        fn push_untimestamped(&mut self, frame: CanFrame) {
            self.rx.push_back((frame, None));
        }
    }

    impl CanTransport for KernelTsBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(Arc::new(self.clock.clone()))
        }
        fn rx_clock_now(&self) -> Option<CanRxTime> {
            Some(CanRxTime::KernelRealtimeNs(self.current_kernel_ns()))
        }
        fn rx_clock_sample(&self) -> Option<RxClockSample> {
            Some(RxClockSample {
                rx_now: CanRxTime::KernelRealtimeNs(self.current_kernel_ns()),
                realtime_minus_monotonic_ns: Some(self.current_offset_ns()),
            })
        }
        fn transmit(&mut self, _frame: &CanFrame) -> Result<(), TransportError> {
            Ok(())
        }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.rx.pop_front().map(|(f, _)| f).ok_or(TransportError::WouldBlock)
        }
        fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
            self.rx.pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        fn supports_bounded_flush(&self) -> bool { true }
        fn flush_is_noop(&self) -> bool { true }
        fn supports_nonblocking_rt(&self) -> bool { true }
    }

    let make_gate = || -> StoppedDisableGate {
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        StoppedDisableGate {
            safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 1,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        }
    };

    let dm_limits = DamiaoLimits::default();
    let dm_fb = |id: u16, state: u8, pos: f32| -> CanFrame {
        let p = damiao::float_to_uint(pos, dm_limits.p_min, dm_limits.p_max, 16);
        let v = damiao::float_to_uint(0.0, dm_limits.v_min, dm_limits.v_max, 12);
        let t = damiao::float_to_uint(0.0, dm_limits.t_min, dm_limits.t_max, 12);
        CanFrame::new_standard(
            0x10 + id,
            &[
                ((state & 0x0f) << 4) | (id as u8 & 0x0f),
                (p >> 8) as u8,
                p as u8,
                (v >> 4) as u8,
                (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
                t as u8,
                30,
                30,
            ],
        )
        .unwrap()
    };

    // Part A: Damiao — pre-write rejection, settle window, +2 ms and -2 ms CLOCK_REALTIME steps restart window.
    {
        let clock = MockClock::new();
        let bus = KernelTsBus::new(clock.clone());
        let mut drv = DamiaoArmDriver::new(bus, vec![1]).with_limits(dm_limits);
        drv.set_stopped_disable_gate(make_gate());

        // Motion write stamps `rest_pose_boundary` with `write_done` + `anchor_offset_ns`.
        let t_pre_write = drv.can_mut().current_kernel_ns();
        clock.advance(Duration::from_millis(2));
        drv.write_joint_targets(&[JointTarget::position(0.0)]).unwrap();
        let t_write_done = drv.can_mut().current_kernel_ns();

        // Advance 25 ms: pre-write, exact +20 ms (strict >), and untimestamped frames are all rejected.
        let mut fresh_pos = [0.0f32; 1];
        clock.advance(Duration::from_millis(25));
        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), t_pre_write);
        let _ = drv.read_joint_states();
        assert!(
            !drv.fresh_joint_positions_into(&[0], Duration::from_millis(100), &mut fresh_pos),
            "pre-write kernel ts must be rejected from fresh pose / rest gate"
        );
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), t_write_done + 20_000_000);
        let _ = drv.read_joint_states();
        assert!(
            !drv.fresh_joint_positions_into(&[0], Duration::from_millis(100), &mut fresh_pos),
            "exact write_done + 20 ms must be rejected (strict >)"
        );
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        drv.can_mut().push_untimestamped(dm_fb(1, 1, 0.0));
        let _ = drv.read_joint_states();
        assert!(
            !drv.fresh_joint_positions_into(&[0], Duration::from_millis(100), &mut fresh_pos),
            "untimestamped frame must be rejected"
        );
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Accumulate 2 valid post-settle rest samples across 2 releases (window count = 2, not yet confirmed).
        let ts1 = drv.can_mut().current_kernel_ns();
        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts1);
        drv.read_joint_states().unwrap();
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        clock.advance(Duration::from_millis(20));
        let ts2 = drv.can_mut().current_kernel_ns();
        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts2);
        drv.read_joint_states().unwrap();
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Now step CLOCK_REALTIME by +2 ms (> 1 ms tolerance) before decoding release 3!
        clock.advance(Duration::from_millis(20));
        drv.can_mut().step_realtime_ns(2_000_000);
        let ts3_stepped = drv.can_mut().current_kernel_ns();
        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts3_stepped);
        drv.read_joint_states().unwrap();
        assert_ne!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "CLOCK_REALTIME +2 ms step between write and decode MUST discard sample and restart window"
        );

        // Immediately after re-anchor (+5 ms inside new 20 ms settle window), frames are still rejected.
        clock.advance(Duration::from_millis(5));
        let ts_in_settle = drv.can_mut().current_kernel_ns();
        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts_in_settle);
        drv.read_joint_states().unwrap();
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Wait past new 20 ms settle window, deliver 2 samples, then test negative -2 ms step too!
        clock.advance(Duration::from_millis(20));
        for _ in 0..2 {
            let ts = drv.can_mut().current_kernel_ns();
            drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts);
            drv.read_joint_states().unwrap();
            clock.advance(Duration::from_millis(15));
        }
        drv.can_mut().step_realtime_ns(-2_000_000);
        let ts_neg_step = drv.can_mut().current_kernel_ns();
        drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts_neg_step);
        drv.read_joint_states().unwrap();
        assert_ne!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "CLOCK_REALTIME -2 ms step MUST discard sample and restart window"
        );

        // After settling > 20 ms with stable offset and 3 fresh rest releases spanning >= 20 ms -> Confirmed!
        clock.advance(Duration::from_millis(25));
        for _ in 0..3 {
            let ts = drv.can_mut().current_kernel_ns();
            drv.can_mut().push_kernel_ts(dm_fb(1, 1, 0.0), ts);
            drv.read_joint_states().unwrap();
            clock.advance(Duration::from_millis(15));
        }
        assert_eq!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "stable post-settle K=3 releases after re-anchor must confirm rest pose"
        );
    }

    // Part B: RobStride — pre-write rejection, 20 ms settle window, and +2 ms CLOCK_REALTIME step restarts window.
    {
        let rs_limits = RobStrideLimits::default();
        let rs_fb = |motor_id: u8, pos_rad: f32| -> CanFrame {
            let id = robstride::build_feedback_extended_id(0, motor_id, 0xFD);
            let p_raw = ((pos_rad - rs_limits.p_min) / (rs_limits.p_max - rs_limits.p_min) * 65535.0)
                .clamp(0.0, 65535.0) as u16;
            CanFrame::new_extended(
                id,
                &[(p_raw >> 8) as u8, p_raw as u8, 0x80, 0, 0x80, 0, 0x01, 0x2C],
            )
            .unwrap()
        };

        let clock = MockClock::new();
        let bus = KernelTsBus::new(clock.clone());
        let mut drv = RobStrideArmDriver::new(bus, vec![1]);
        drv.set_stopped_disable_gate(make_gate());

        let t_pre_write = drv.can_mut().current_kernel_ns();
        clock.advance(Duration::from_millis(2));
        drv.write_joint_targets(&[JointTarget::position(0.0)])
            .expect("RobStride motion write stamps rest_pose_boundary");

        // Pre-write and untimestamped frames rejected even 25 ms later.
        let mut fresh_pos = [0.0f32; 1];
        clock.advance(Duration::from_millis(25));
        drv.can_mut().push_kernel_ts(rs_fb(1, 0.0), t_pre_write);
        let mut out = Vec::new();
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
        assert!(
            !drv.fresh_joint_positions_into(&[0], Duration::from_millis(100), &mut fresh_pos),
            "RobStride pre-write frame must be rejected from fresh pose / rest gate"
        );
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        drv.can_mut().push_untimestamped(rs_fb(1, 0.0));
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
        assert!(
            !drv.fresh_joint_positions_into(&[0], Duration::from_millis(100), &mut fresh_pos),
            "RobStride untimestamped frame must be rejected from fresh pose / rest gate"
        );
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Accumulate 2 valid post-settle rest samples.
        for _ in 0..2 {
            let ts = drv.can_mut().current_kernel_ns();
            drv.can_mut().push_kernel_ts(rs_fb(1, 0.0), ts);
            let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
            clock.advance(Duration::from_millis(20));
        }
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Step CLOCK_REALTIME by +2 ms -> 3rd sample discarded and window restarted!
        drv.can_mut().step_realtime_ns(2_000_000);
        let ts3 = drv.can_mut().current_kernel_ns();
        drv.can_mut().push_kernel_ts(rs_fb(1, 0.0), ts3);
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
        assert_ne!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "RobStride: CLOCK_REALTIME step > 1 ms must discard sample and restart window"
        );

        // After settling > 20 ms with stable offset + 3 releases -> Confirmed!
        clock.advance(Duration::from_millis(25));
        for _ in 0..3 {
            let ts = drv.can_mut().current_kernel_ns();
            drv.can_mut().push_kernel_ts(rs_fb(1, 0.0), ts);
            let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
            clock.advance(Duration::from_millis(15));
        }
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);
    }
}

/// R10 Test 4: Future-dated RX timestamps (`rx_ts > now`) are REJECTED on both `KernelRealtimeNs`
/// and `Monotonic` clock domains (NEVER clamped to `now`), and `KernelRealtimeNs` without a
/// same-clock `CLOCK_REALTIME - CLOCK_MONOTONIC` offset fails closed.
#[test]
fn r10_future_dated_timestamps_rejected_never_clamped() {
    use armd::driver::{
        ArmDriver, DamiaoArmDriver, RestPoseGateStatus, RobStrideArmDriver, StoppedDisableGate,
    };
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::protocol::robstride::{self, RobStrideLimits};
    use armd::transport::{
        CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, TransportError,
    };
    use std::collections::VecDeque;

    struct CustomTsBus {
        clock: MockClock,
        base_real_ns: i128,
        base_mono: std::time::Instant,
        provide_offset: bool,
        rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    }

    impl CustomTsBus {
        fn new(clock: MockClock, provide_offset: bool) -> Self {
            let base_mono = clock.now();
            Self {
                clock,
                base_real_ns: 1_700_000_000_000_000_000,
                base_mono,
                provide_offset,
                rx: VecDeque::new(),
            }
        }
        fn current_kernel_ns(&self) -> i128 {
            let elapsed = self.clock.now().saturating_duration_since(self.base_mono).as_nanos() as i128;
            self.base_real_ns + elapsed
        }
        fn push(&mut self, frame: CanFrame, ts: Option<CanRxTime>) {
            self.rx.push_back((frame, ts));
        }
    }

    impl CanTransport for CustomTsBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(Arc::new(self.clock.clone()))
        }
        fn rx_clock_now(&self) -> Option<CanRxTime> {
            Some(CanRxTime::KernelRealtimeNs(self.current_kernel_ns()))
        }
        fn rx_clock_sample(&self) -> Option<RxClockSample> {
            if self.provide_offset {
                Some(RxClockSample {
                    rx_now: CanRxTime::KernelRealtimeNs(self.current_kernel_ns()),
                    realtime_minus_monotonic_ns: Some(self.base_real_ns),
                })
            } else {
                None
            }
        }
        fn transmit(&mut self, _frame: &CanFrame) -> Result<(), TransportError> { Ok(()) }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.rx.pop_front().map(|(f, _)| f).ok_or(TransportError::WouldBlock)
        }
        fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
            self.rx.pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
        fn supports_bounded_flush(&self) -> bool { true }
        fn flush_is_noop(&self) -> bool { true }
        fn supports_nonblocking_rt(&self) -> bool { true }
    }

    let make_gate = || -> StoppedDisableGate {
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        StoppedDisableGate {
            safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 1,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        }
    };

    let dm_limits = DamiaoLimits::default();
    let dm_fb = |pos: f32| -> CanFrame {
        let p = damiao::float_to_uint(pos, dm_limits.p_min, dm_limits.p_max, 16);
        let v = damiao::float_to_uint(0.0, dm_limits.v_min, dm_limits.v_max, 12);
        let t = damiao::float_to_uint(0.0, dm_limits.t_min, dm_limits.t_max, 12);
        CanFrame::new_standard(
            0x11,
            &[0x11, (p >> 8) as u8, p as u8, (v >> 4) as u8, (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8, t as u8, 30, 30],
        )
        .unwrap()
    };

    // 1. Damiao: future KernelRealtimeNs (+5 ms) & future Monotonic (+5 ms) across 3 releases NEVER accepted.
    {
        let clock = MockClock::new();
        let bus = CustomTsBus::new(clock.clone(), true);
        let mut drv = DamiaoArmDriver::new(bus, vec![1]).with_limits(dm_limits);
        drv.set_stopped_disable_gate(make_gate());
        drv.read_joint_states().unwrap();

        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            let future_k = drv.can_mut().current_kernel_ns() + 5_000_000;
            let future_m = clock.now() + Duration::from_millis(5);
            drv.can_mut().push(dm_fb(0.0), Some(CanRxTime::KernelRealtimeNs(future_k)));
            drv.can_mut().push(dm_fb(0.0), Some(CanRxTime::Monotonic(future_m)));
            drv.read_joint_states().unwrap();
            assert!(
                drv.feedback_ages()[0].is_none(),
                "Damiao future-dated timestamps MUST be rejected, never clamped to now"
            );
            assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);
        }

        // Valid non-future frames across 3 releases DO confirm.
        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            let valid_k = drv.can_mut().current_kernel_ns();
            drv.can_mut().push(dm_fb(0.0), Some(CanRxTime::KernelRealtimeNs(valid_k)));
            drv.read_joint_states().unwrap();
        }
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);
    }

    // 2. Damiao: KernelRealtimeNs without offset sample fails closed (rejected).
    {
        let clock = MockClock::new();
        let bus = CustomTsBus::new(clock.clone(), false);
        let mut drv = DamiaoArmDriver::new(bus, vec![1]).with_limits(dm_limits);
        drv.read_joint_states().unwrap();
        let ts = drv.can_mut().current_kernel_ns();
        drv.can_mut().push(dm_fb(0.0), Some(CanRxTime::KernelRealtimeNs(ts)));
        drv.read_joint_states().unwrap();
        assert!(
            drv.feedback_ages()[0].is_none(),
            "KernelRealtimeNs without rx_clock_sample offset must fail closed"
        );
    }

    // 3. RobStride: future KernelRealtimeNs (+5 ms) & future Monotonic (+5 ms) rejected, never clamped.
    {
        let rs_limits = RobStrideLimits::default();
        let rs_fb = |pos_rad: f32| -> CanFrame {
            let id = robstride::build_feedback_extended_id(0, 1, 0xFD);
            let p_raw = ((pos_rad - rs_limits.p_min) / (rs_limits.p_max - rs_limits.p_min) * 65535.0)
                .clamp(0.0, 65535.0) as u16;
            CanFrame::new_extended(id, &[(p_raw >> 8) as u8, p_raw as u8, 0x80, 0, 0x80, 0, 1, 0x2C]).unwrap()
        };

        let clock = MockClock::new();
        let bus = CustomTsBus::new(clock.clone(), true);
        let mut drv = RobStrideArmDriver::new(bus, vec![1]);
        drv.set_stopped_disable_gate(make_gate());
        let mut out = Vec::new();
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);

        for _ in 0..3 {
            clock.advance(Duration::from_millis(20));
            let future_k = drv.can_mut().current_kernel_ns() + 5_000_000;
            let future_m = clock.now() + Duration::from_millis(5);
            drv.can_mut().push(rs_fb(0.0), Some(CanRxTime::KernelRealtimeNs(future_k)));
            drv.can_mut().push(rs_fb(0.0), Some(CanRxTime::Monotonic(future_m)));
            let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
            assert!(
                drv.feedback_ages()[0].is_none(),
                "RobStride future-dated timestamps MUST be rejected, never clamped to now"
            );
            assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);
        }
    }
}

/// R10 Test 5: Two or more queued samples drained inside ONE control release count as only 1 tick
/// (NOT 2 ticks), even when their RX timestamps span >= 20 ms. Also verifies K=3 distinct control
/// releases + disagreeing-sample reset across Damiao, RobStride, Feetech, and FashionStar.
#[test]
fn r10_two_queued_samples_in_one_release_do_not_count_as_two_ticks() {
    use armd::driver::{
        ArmDriver, DamiaoArmDriver, FashionStarArmDriver, FeetechArmDriver, RestPoseGateStatus,
        RobStrideArmDriver, StoppedDisableGate,
    };
    use armd::protocol::damiao::{self, DamiaoLimits};
    use armd::protocol::fashionstar;
    use armd::protocol::feetech;
    use armd::protocol::robstride::{self, RobStrideLimits};
    use armd::transport::{
        CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, MockTransport, TransportError,
    };
    use std::collections::VecDeque;

    struct QueuedMonoBus {
        clock: MockClock,
        rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    }
    impl QueuedMonoBus {
        fn new(clock: MockClock) -> Self {
            Self { clock, rx: VecDeque::new() }
        }
        fn push_at(&mut self, frame: CanFrame, ts: Instant) {
            self.rx.push_back((frame, Some(CanRxTime::Monotonic(ts))));
        }
    }
    impl CanTransport for QueuedMonoBus {
        fn clock(&self) -> Option<Arc<dyn DriverClock>> {
            Some(Arc::new(self.clock.clone()))
        }
        fn transmit(&mut self, _frame: &CanFrame) -> Result<(), TransportError> { Ok(()) }
        fn receive(&mut self) -> Result<CanFrame, TransportError> {
            self.rx.pop_front().map(|(f, _)| f).ok_or(TransportError::WouldBlock)
        }
        fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
            self.rx.pop_front().ok_or(TransportError::WouldBlock)
        }
        fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
        fn supports_bounded_flush(&self) -> bool { true }
        fn flush_is_noop(&self) -> bool { true }
        fn supports_nonblocking_rt(&self) -> bool { true }
    }

    let make_gate = || -> StoppedDisableGate {
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        StoppedDisableGate {
            safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 1,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        }
    };

    // 1. Damiao with owner-managed `begin_control_release`:
    //    - Release 1 drains TWO queued frames spanning 10 ms (plus even a second read in the same release!)
    //    - Release 2 at +25 ms adds 1 frame -> total 3 frames spanning 25 ms, ONLY 2 releases -> NOT confirmed!
    //    - Release 3 at +40 ms adds 1 frame -> 3 distinct releases spanning >= 20 ms -> Confirmed!
    {
        let dm_limits = DamiaoLimits::default();
        let dm_fb = |pos: f32| -> CanFrame {
            let p = damiao::float_to_uint(pos, dm_limits.p_min, dm_limits.p_max, 16);
            let v = damiao::float_to_uint(0.0, dm_limits.v_min, dm_limits.v_max, 12);
            let t = damiao::float_to_uint(0.0, dm_limits.t_min, dm_limits.t_max, 12);
            CanFrame::new_standard(
                0x11,
                &[0x11, (p >> 8) as u8, p as u8, (v >> 4) as u8, (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8, t as u8, 30, 30],
            )
            .unwrap()
        };

        let clock = MockClock::new();
        let mut drv = DamiaoArmDriver::new(QueuedMonoBus::new(clock.clone()), vec![1]).with_limits(dm_limits);
        drv.set_stopped_disable_gate(make_gate());
        drv.begin_control_release(1);
        drv.read_joint_states().unwrap(); // establish empty RX boundary

        // Release 2: TWO queued frames separated by 15 ms, PLUS even a 3rd frame spanning 30 ms, all in release 2!
        let t0 = clock.now();
        clock.advance(Duration::from_millis(30));
        drv.begin_control_release(2);
        drv.can_mut().push_at(dm_fb(0.0), t0);
        drv.can_mut().push_at(dm_fb(0.0), t0 + Duration::from_millis(15));
        drv.can_mut().push_at(dm_fb(0.0), t0 + Duration::from_millis(30));
        drv.read_joint_states().unwrap();
        assert_ne!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "3 queued frames spanning 30 ms inside ONE release must count as 1 tick, never Confirmed"
        );

        // Release 3 (+20 ms): 1 more frame -> total 4 frames spanning 50 ms across ONLY 2 releases -> STILL NOT Confirmed!
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(3);
        drv.can_mut().push_at(dm_fb(0.0), clock.now());
        drv.read_joint_states().unwrap();
        assert_ne!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "queued samples in release 2 + 1 sample in release 3 = only 2 distinct releases, MUST NOT confirm"
        );

        // Release 4 (+20 ms): 3rd distinct release -> Confirmed!
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(4);
        drv.can_mut().push_at(dm_fb(0.0), clock.now());
        drv.read_joint_states().unwrap();
        assert_eq!(
            drv.rest_pose_gate_status(),
            RestPoseGateStatus::Confirmed,
            "3 distinct control releases spanning >= 20 ms must confirm rest pose"
        );

        // Disagreeing non-rest sample resets the window immediately.
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(5);
        drv.can_mut().push_at(dm_fb(0.40), clock.now());
        drv.read_joint_states().unwrap();
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::RefusedNonRest);
    }

    // 2. RT Owner (`ReleaseFixture::step()`) stamps `begin_control_release` per release:
    //    2 queued rest frames in release 1 + 1 frame in release 2 -> still NOT confirmed until release 3!
    {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        // Expire old window and observe empty boundary.
        r.clock.advance(Duration::from_millis(120));
        r.step();

        // Release 1: queue TWO full bursts of rest feedback frames in the same control release.
        r.bus.with(|s| {
            s.push_all_feedback();
            s.push_all_feedback();
        });
        r.clock.advance(Duration::from_millis(20));
        r.step();
        assert_eq!(
            r.fx.core.may_disable_torque_while_stopped(),
            Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
            "2 queued frames in release 1 must count as only 1 tick"
        );

        // Release 2 (+25 ms): 1 more frame -> STILL only 2 distinct releases!
        r.clock.advance(Duration::from_millis(25));
        r.bus.with(|s| s.push_all_feedback());
        r.step();
        assert_eq!(
            r.fx.core.may_disable_torque_while_stopped(),
            Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback),
            "only 2 distinct control releases must STILL refuse rest gate"
        );

        // Release 3 (+20 ms): 3rd distinct release -> confirmed!
        r.clock.advance(Duration::from_millis(20));
        r.bus.with(|s| s.push_all_feedback());
        r.step();
        assert_eq!(r.fx.core.may_disable_torque_while_stopped(), Ok(()));
    }

    // 3. RobStride with `begin_control_release`: 2 queued frames in 1 release != 2 ticks; 3 releases confirms.
    {
        let rs_limits = RobStrideLimits::default();
        let rs_fb = |pos_rad: f32| -> CanFrame {
            let id = robstride::build_feedback_extended_id(0, 1, 0xFD);
            let p_raw = ((pos_rad - rs_limits.p_min) / (rs_limits.p_max - rs_limits.p_min) * 65535.0)
                .clamp(0.0, 65535.0) as u16;
            CanFrame::new_extended(id, &[(p_raw >> 8) as u8, p_raw as u8, 0x80, 0, 0x80, 0, 1, 0x2C]).unwrap()
        };
        let clock = MockClock::new();
        let mut drv = RobStrideArmDriver::new(QueuedMonoBus::new(clock.clone()), vec![1]);
        drv.set_stopped_disable_gate(make_gate());
        let mut out = Vec::new();
        drv.begin_control_release(1);
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);

        let t0 = clock.now();
        clock.advance(Duration::from_millis(25));
        drv.begin_control_release(2);
        drv.can_mut().push_at(rs_fb(0.0), t0);
        drv.can_mut().push_at(rs_fb(0.0), t0 + Duration::from_millis(25));
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(3);
        drv.can_mut().push_at(rs_fb(0.0), clock.now());
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(4);
        drv.can_mut().push_at(rs_fb(0.0), clock.now());
        let _ = drv.read_joint_states_budgeted(&mut out, clock.now() + Duration::from_millis(5), 64);
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);
    }

    // 4. Feetech & FashionStar: K=3 distinct releases spanning >= 20 ms required for rest gate.
    {
        let ft_pos_reply = |id: u8, steps: u16| -> Vec<u8> {
            let p = steps.to_le_bytes();
            let params = [p[0], p[1], 0, 0, 0, 0, 70, 35];
            let cs = feetech::calculate_checksum(id, 10, 0, &params);
            let mut v = vec![0xFF, 0xFF, id, 10, 0];
            v.extend_from_slice(&params);
            v.push(cs);
            v
        };
        let clock = MockClock::new();
        let mock = MockTransport::new_preserving().with_clock(clock.clone());
        let mut drv = FeetechArmDriver::new(mock, vec![1]).with_calibrated(true);
        drv.set_stopped_disable_gate(make_gate());

        // Two reads inside the same control release count as 1 tick!
        drv.begin_control_release(1);
        drv.transport_mut().provide_rx(&ft_pos_reply(1, 2048));
        drv.read_joint_states().unwrap();
        clock.advance(Duration::from_millis(25));
        drv.transport_mut().provide_rx(&ft_pos_reply(1, 2048));
        drv.read_joint_states().unwrap();
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Release 2 -> still 2 ticks!
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(2);
        drv.transport_mut().provide_rx(&ft_pos_reply(1, 2048));
        drv.read_joint_states().unwrap();
        assert_ne!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // Release 3 -> Confirmed!
        clock.advance(Duration::from_millis(20));
        drv.begin_control_release(3);
        drv.transport_mut().provide_rx(&ft_pos_reply(1, 2048));
        drv.read_joint_states().unwrap();
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        // FashionStar K=3 releases.
        let fs_angle_reply = |id: u8, tenth_deg: i16| -> Vec<u8> {
            let [lo, hi] = tenth_deg.to_le_bytes();
            let mut pkt = vec![0x05, 0x1C, fashionstar::CMD_QUERY_SERVO_ANGLE, 3, id, lo, hi];
            let cs: u8 = pkt.iter().fold(0u8, |a, &b| a.wrapping_add(b));
            pkt.push(cs);
            pkt
        };
        let clock_fs = MockClock::new();
        let mock_fs = MockTransport::new_preserving().with_clock(clock_fs.clone());
        let mut drv_fs = FashionStarArmDriver::new(mock_fs, vec![0]);
        drv_fs.set_stopped_disable_gate(make_gate());

        drv_fs.begin_control_release(1);
        drv_fs.transport_mut().provide_rx(&fs_angle_reply(0, 0));
        drv_fs.read_joint_states().unwrap();
        assert_ne!(drv_fs.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        clock_fs.advance(Duration::from_millis(20));
        drv_fs.begin_control_release(2);
        drv_fs.transport_mut().provide_rx(&fs_angle_reply(0, 0));
        drv_fs.read_joint_states().unwrap();
        assert_ne!(drv_fs.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);

        clock_fs.advance(Duration::from_millis(20));
        drv_fs.begin_control_release(3);
        drv_fs.transport_mut().provide_rx(&fs_angle_reply(0, 0));
        drv_fs.read_joint_states().unwrap();
        assert_eq!(drv_fs.rest_pose_gate_status(), RestPoseGateStatus::Confirmed);
    }
}

/// R10 Test 6: Feetech defers an incomplete rest-pose window (or poisoned serial line) within the
/// 1 s request lifetime, transmitting 0 `REG_TORQUE_ENABLE=0` packets until K=3 releases confirm,
/// while immediately refusing real non-rest poses and expiring cleanly past 1 s. Tested both
/// standalone and through the RT supervisor `ReleaseFixture`.
#[test]
fn r10_feetech_defers_incomplete_rest_window_within_1s_lifetime() {
    use armd::driver::{
        ArmDriver, ArmDriverError, DisableKind, FeetechArmDriver, RestPoseGateStatus,
        StoppedDisableGate, TorqueOutcome, TorqueProgress,
    };
    use armd::protocol::feetech;
    use armd::rt_core::broker::{BootId, Capacities, ClientId, Outcome, Submit, TransitionOp};
    use armd::rt_core::ops::{CoreOps, ReleaseFixture};
    use armd::transport::MockTransport;
    use armd::TorqueIntent;

    #[derive(Clone)]
    struct SharedSerial(Arc<std::sync::Mutex<MockTransport>>);
    impl armd::transport::Transport for SharedSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().write(buf)
        }
        fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().write_all(buf)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().read(buf)
        }
        fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().flush()
        }
        fn clear_input_buffer(&mut self) {
            self.0.lock().unwrap().clear_input_buffer();
        }
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            armd::transport::Transport::clock(&*self.0.lock().unwrap())
        }
    }

    let make_gate = |dof: usize| -> StoppedDisableGate {
        let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
        for (i, slot) in arm_joints.iter_mut().take(dof).enumerate() {
            *slot = i;
        }
        StoppedDisableGate {
            safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: dof,
            rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
            rest_pose_tolerance_rad: 0.05,
        }
    };

    // Part A: Standalone Feetech defers with 1 rest sample, stays Pending on release 2, completes on release 3.
    {
        let clock = MockClock::new();
        let serial = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let pos_req = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
        let en_req = feetech::build_read(1, feetech::REG_TORQUE_ENABLE, 1);
        {
            let mut ser = serial.lock().unwrap();
            ser.stage_reply(&pos_req, &feetech_feedback_bytes(&[1], &[2048]));
            ser.stage_reply(&en_req, &feetech_reg_u8_resp(1, 0));
        }
        let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial)), vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));
        drv.set_stopped_disable_gate(make_gate(1));
        assert!(drv.supports_budgeted_disable_await());

        // Only 1 rest sample -> window incomplete (`PendingDrainOrWindow`).
        drv.read_joint_states().unwrap();
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::PendingDrainOrWindow);
        let _ = serial.lock().unwrap().take_tx();

        let started_at = clock.now();
        let p0 = drv
            .begin_disable_torque(DisableKind::DisableRequested { started_at })
            .expect("incomplete rest window must defer, not fail");
        assert!(matches!(p0, TorqueProgress::Pending { .. }));
        assert!(drv.torque_transition_pending());
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 0);

        // Release 2 (+20 ms): still Pending, 0 disable packets.
        clock.advance(Duration::from_millis(20));
        let p1 = drv.poll_torque(clock.now() + Duration::from_millis(10)).unwrap();
        assert!(matches!(p1, TorqueProgress::Pending { .. }));
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 0);

        // Release 3 (+20 ms): K=3 spanning 40 ms confirmed inside 1 s -> Done + TORQUE_ENABLE=0 written!
        clock.advance(Duration::from_millis(20));
        let p2 = drv.poll_torque(clock.now() + Duration::from_millis(10)).unwrap();
        assert_eq!(p2, TorqueProgress::Done(TorqueOutcome::Confirmed));
        assert!(!drv.torque_transition_pending());
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 1);
    }

    // Part B: Poisoned line (timeout -> 20 ms quiet required) defers cleanly and completes after recovery.
    {
        let clock = MockClock::new();
        let serial = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let pos_req = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
        let en_req = feetech::build_read(1, feetech::REG_TORQUE_ENABLE, 1);
        let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial)), vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));
        drv.set_stopped_disable_gate(make_gate(1));

        // Empty RX -> chunk timeout poisons the line.
        drv.read_joint_states().unwrap();
        assert_eq!(drv.rest_pose_gate_status(), RestPoseGateStatus::PendingDrainOrWindow);

        let started_at = clock.now();
        let p0 = drv
            .begin_disable_torque(DisableKind::DisableRequested { started_at })
            .expect("poisoned line must defer within 1 s lifetime");
        assert!(matches!(p0, TorqueProgress::Pending { .. }));

        // At +10 ms (still inside 20 ms poison quiet window): even with replies staged, stays Pending with 0 disables.
        {
            let mut ser = serial.lock().unwrap();
            ser.stage_reply(&pos_req, &feetech_feedback_bytes(&[1], &[2048]));
            ser.stage_reply(&en_req, &feetech_reg_u8_resp(1, 0));
        }
        clock.advance(Duration::from_millis(10));
        let p_poison = drv.poll_torque(clock.now() + Duration::from_millis(10)).unwrap();
        assert!(matches!(p_poison, TorqueProgress::Pending { .. }));
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 0);

        // After 25 ms quiet + 3 releases inside 1 s -> clears poison, completes K=3, and writes disable!
        clock.advance(Duration::from_millis(25));
        let mut done = false;
        for _ in 0..5 {
            match drv.poll_torque(clock.now() + Duration::from_millis(10)).unwrap() {
                TorqueProgress::Done(TorqueOutcome::Confirmed) => {
                    done = true;
                    break;
                }
                TorqueProgress::Done(TorqueOutcome::Unconfirmed) => {
                    panic!("Feetech rest-gated disable must complete Confirmed, not Unconfirmed");
                }
                TorqueProgress::Pending { .. } => clock.advance(Duration::from_millis(20)),
            }
        }
        assert!(done, "poisoned Feetech line must recover and complete disable within 1 s");
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 1);
    }

    // Part C & D: Raised pose refuses immediately (`TorqueOffWouldDrop`); 1 s expiry returns `DisableRequestExpired`.
    {
        let clock = MockClock::new();
        let serial = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let pos_req = feetech::build_sync_read(&[1], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
        serial.lock().unwrap().stage_reply(&pos_req, &feetech_feedback_bytes(&[1], &[2600]));
        let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial)), vec![1])
            .with_calibrated(true)
            .with_tx_echo(Some(false));
        drv.set_stopped_disable_gate(make_gate(1));
        drv.read_joint_states().unwrap();
        let _ = serial.lock().unwrap().take_tx();
        let err = drv
            .begin_disable_torque(DisableKind::DisableRequested {
                started_at: clock.now(),
            })
            .unwrap_err();
        assert!(matches!(err, ArmDriverError::TorqueOffWouldDrop), "raised pose must refuse immediately: {err:?}");
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 0);

        // Now switch to rest, start defer with 1 sample, then advance past 1 s -> DisableRequestExpired.
        {
            let mut ser = serial.lock().unwrap();
            ser.clear();
            ser.clear_write_responses();
            ser.stage_reply(&pos_req, &feetech_feedback_bytes(&[1], &[2048]));
        }
        drv.read_joint_states().unwrap();
        let started_at = clock.now();
        let p0 = drv
            .begin_disable_torque(DisableKind::DisableRequested { started_at })
            .unwrap();
        assert!(matches!(p0, TorqueProgress::Pending { .. }));
        clock.advance(Duration::from_millis(1050));
        let _ = serial.lock().unwrap().take_tx();
        let exp_err = drv.poll_torque(clock.now() + Duration::from_millis(10)).unwrap_err();
        assert!(
            matches!(exp_err, ArmDriverError::DisableRequestExpired),
            "incomplete Feetech rest window past 1 s must expire: {exp_err:?}"
        );
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 0);
    }

    // Part E: RT supervisor (`ReleaseFixture`) with Feetech defers incomplete rest window and completes inside 1 s.
    {
        let clock = MockClock::new();
        let servo_ids: Vec<u8> = (1..=6).collect();
        let serial = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let req1 = feetech::build_sync_read(&servo_ids[..3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
        let req2 = feetech::build_sync_read(&servo_ids[3..], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
        {
            let mut ser = serial.lock().unwrap();
            ser.stage_reply(&req1, &feetech_feedback_bytes(&servo_ids[..3], &[2048; 3]));
            ser.stage_reply(&req2, &feetech_feedback_bytes(&servo_ids[3..], &[2048; 3]));
            for id in 1..=6u8 {
                let r_en = feetech::build_read(id, feetech::REG_TORQUE_ENABLE, 1);
                ser.stage_reply(&r_en, &feetech_reg_u8_resp(id, 0));
            }
        }
        let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial)), servo_ids.clone())
            .with_calibrated(true)
            .with_tx_echo(Some(false))
            .with_gripper_index(Some(5));
        drv.read_joint_states().unwrap(); // only 1 rest sample so far!

        let mut inner = SupervisorInner::new(feetech_6dof_config()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.rt_executor_owned = true;
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        let (mut fx, mut b, _tw, _stop) = ReleaseFixture::new(
            inner,
            BootId(91),
            Capacities { commands: 8, plans: 1, replies: 16 },
        );
        let _ = serial.lock().unwrap().take_tx();
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();

        // Release 1 (still < 3 rest releases): accepted as Pending with 0 disable packets!
        clock.advance(Duration::from_millis(20));
        let r1 = fx.step();
        assert!(r1.command.as_ref().is_some_and(|(_, res)| res.is_ok()));
        assert!(fx.core.torque_transition_pending(), "RT Feetech TorqueOff must defer while K=3 completes");
        assert_eq!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()), 0);

        // Subsequent releases inside 1 s complete K=3 and disable torque cleanly!
        let mut outcome = r1.transition_ended.map(|(_, o)| o);
        for _ in 0..8 {
            if outcome.is_some() {
                break;
            }
            clock.advance(Duration::from_millis(20));
            if let Some((_, o)) = fx.step().transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(outcome, Some(Outcome::Completed));
        assert_eq!(fx.core.torque_intent, TorqueIntent::Off);
        assert!(!fx.core.is_torqued);
        assert!(spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()) >= 6);

        // Next release emits 0 hold SYNC_WRITE packets.
        let _ = serial.lock().unwrap().take_tx();
        clock.advance(Duration::from_millis(20));
        fx.step();
        assert!(
            parse_feetech_sync_write_goal_steps(&serial.lock().unwrap().take_tx()).is_empty(),
            "Feetech holds must stop after accepted deferred TorqueOff"
        );
    }
}

/// R10 Test 7: `try_lock` contention on stopped-hold TX escalates after `N = 25` consecutive
/// skips (0.5 s at 50 Hz) with a counted warning + non-recoverable `HardwareFault`, latches once
/// per streak, and resets on a successful uncontended hold pass.
#[test]
fn r10_try_lock_contention_escalates_after_25_consecutive_skips() {
    use armd::diagnostics::tick::{
        reset_stopped_hold_contention_escalations, reset_stopped_hold_lock_contended,
        stopped_hold_contention_escalations_count, stopped_hold_lock_contended_count,
    };
    use armd::rt_core::ops::ServiceBudget;

    assert_eq!(armd::STOPPED_HOLD_CONTENTION_ESCALATE_SKIPS, 25);

    let mut r = ExecRig::new();
    r.bus.with(|s| {
        s.follow_commands = false;
        s.motors[1].q = 0.35;
        s.push_all_feedback();
    });
    r.arm_now();
    // Use recoverable WatchdogTimeout so we can assert escalation upgrades to non-recoverable HardwareFault.
    r.fx.core.safety.latch(
        armd::safety::FaultKind::WatchdogTimeout,
        armd::safety::FaultDetail::WatchdogTimeout,
    );
    assert!(r.fx.core.safety.recoverable());

    // Populate owner snapshot with fresh verified joints so hold_mask != 0.
    r.bus.with(|s| s.push_all_feedback());
    let b0 = ServiceBudget::for_release(r.fx.now(), r.fx.now() + Duration::from_millis(20));
    let _ = r.fx.core.poll_hardware(&b0);
    assert!(r.fx.core.service_stopped_measured_hold_complete(0.02, false));
    assert_ne!(r.fx.core.owner_hold_mask(), 0);

    reset_stopped_hold_lock_contended();
    reset_stopped_hold_contention_escalations();

    // Streak 1: 24 skips -> no escalation; 25th skip -> escalates once; 26th skip -> does not double-count.
    r.fx.core.with_contended_driver_for_test(|core| {
        for skip in 1..=24u32 {
            assert!(!core.service_stopped_measured_hold_complete(0.02, false));
            assert_eq!(stopped_hold_lock_contended_count(), u64::from(skip));
            assert_eq!(
                stopped_hold_contention_escalations_count(),
                0,
                "must NOT escalate before N=25 (at skip {skip})"
            );
            assert!(core.safety.recoverable(), "must stay recoverable before N=25 (at skip {skip})");
        }

        // 25th consecutive skip: escalates with counted warning + non-recoverable HardwareFault!
        assert!(!core.service_stopped_measured_hold_complete(0.02, false));
        assert_eq!(stopped_hold_lock_contended_count(), 25);
        assert_eq!(stopped_hold_contention_escalations_count(), 1);
        assert!(
            core.safety.has_latched_fault_code(FaultCode::HardwareFault),
            "25th consecutive contended skip must latch HardwareFault"
        );
        assert!(
            !core.safety.recoverable(),
            "25th consecutive contended skip fault must be non-recoverable"
        );

        // 26th skip in the same unbroken streak does not re-increment the escalation counter.
        assert!(!core.service_stopped_measured_hold_complete(0.02, false));
        assert_eq!(stopped_hold_lock_contended_count(), 26);
        assert_eq!(stopped_hold_contention_escalations_count(), 1);
    });

    // One successful uncontended hold pass resets the streak counter!
    r.clock.advance(Duration::from_millis(20));
    r.bus.with(|s| s.push_all_feedback());
    let b1 = ServiceBudget::for_release(r.fx.now(), r.fx.now() + Duration::from_millis(20));
    let _ = r.fx.core.poll_hardware(&b1);
    assert!(r.fx.core.service_stopped_measured_hold_complete(0.02, false));

    // Streak 2: 24 more skips stay at 1 escalation; 25th skip increments to 2.
    r.fx.core.with_contended_driver_for_test(|core| {
        for _ in 1..=24 {
            assert!(!core.service_stopped_measured_hold_complete(0.02, false));
        }
        assert_eq!(stopped_hold_contention_escalations_count(), 1);
        assert!(!core.service_stopped_measured_hold_complete(0.02, false));
        assert_eq!(stopped_hold_contention_escalations_count(), 2);
    });
}

/// L1: An exhausted RX drain or incomplete K=3 rest-pose window keeps a stopped disable request
/// pending and retries on subsequent RT releases inside the 1 s lifetime. Refuses ONLY on a real
/// non-rest pose or when the 1 s lifetime expires.
#[test]
fn l1_exhausted_drain_retries_and_completes_disable_within_1s_lifetime() {
    use armd::transport::CanFrame;
    let junk_frame = CanFrame::new_standard(0x500, &[0u8; 8]).unwrap();

    // 1. 7-motor RT executor bus at rest with an 80-frame burst exhausting Step 2's 64-frame drain:
    //    `TransitionOp::TorqueOff` stays pending with 0 CMD_DISABLE frames during the exhausted
    //    release, drains the burst, collects K=3 post-drain rest samples across >= 2 ticks, and
    //    completes disable (`Outcome::Completed`) well within the 1 s lifetime.
    {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        // Clear the 20 ms post-motion settle (plus 1-tick delayed reply on FakeBus) and fill K=3.
        for _ in 0..5 {
            r.step_with_feedback();
        }
        assert_eq!(r.fx.core.may_disable_torque_while_stopped(), Ok(()));

        // Inject an 80-frame burst so Step 2's 64-frame RX budget exhausts (`more = true`).
        r.bus.with(|s| {
            s.tx_log.clear();
            for _ in 0..80 {
                s.rx.push_back(junk_frame);
            }
        });
        let t0 = r.clock.now();
        r.submit_transition(TransitionOp::TorqueOff);
        let first = r.step();
        assert!(
            first.command.as_ref().is_some_and(|(_, res)| res.is_ok()),
            "L1: TorqueOff behind exhausted drain must be accepted as pending, got {first:?}"
        );
        assert!(
            r.fx.core.torque_transition_pending(),
            "L1: TorqueOff must stay pending while drain + K-window completes"
        );
        assert_eq!(
            spec_count_damiao_disables(&r.bus),
            0,
            "L1: ZERO CMD_DISABLE frames allowed while RX drain is still incomplete"
        );

        // Advance releases inside 1 s: drain finishes, K=3 rest window + K=3 OffConfirmed complete.
        let mut outcome = first.transition_ended.map(|(_, o)| o);
        for _ in 0..25 {
            if outcome.is_some() {
                break;
            }
            let rep = r.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        let elapsed = r.clock.now().saturating_duration_since(t0);
        assert_eq!(
            outcome,
            Some(Outcome::Completed),
            "L1: 7-motor disable after exhausted burst drain must complete within 1 s (elapsed={elapsed:?})"
        );
        assert!(
            elapsed < Duration::from_secs(1),
            "L1: disable must finish strictly inside the 1 s lifetime, took {elapsed:?}"
        );
        assert!(!r.fx.core.is_torqued);
        assert!(matches!(r.fx.core.safety.state(), SafetyState::EStop));
    }

    // 2. Contrast: incomplete K-window (only 1 fresh rest sample so far) ALSO stays pending and
    //    completes within 1 s once K=3 samples arrive.
    {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        // Advance > 100 ms so prior arm-now window resets, observe empty queue, then deliver only 1 sample.
        r.clock.advance(Duration::from_millis(120));
        r.step(); // empty RX boundary
        r.step_with_feedback(); // 1 rest sample only -> K-window incomplete!
        assert_eq!(
            r.fx.core.may_disable_torque_while_stopped(),
            Err(StoppedTorqueOffRefusal::StaleOrMissingFeedback)
        );
        r.bus.with(|s| s.tx_log.clear());
        let t0 = r.clock.now();
        r.submit_transition(TransitionOp::TorqueOff);
        let first = r.step();
        assert!(
            first.command.as_ref().is_some_and(|(_, res)| res.is_ok()),
            "L1: incomplete K-window must keep TorqueOff pending, got {first:?}"
        );
        assert_eq!(spec_count_damiao_disables(&r.bus), 0, "L1: 0 CMD_DISABLE before K=3 completes");
        let mut outcome = first.transition_ended.map(|(_, o)| o);
        for _ in 0..25 {
            if outcome.is_some() {
                break;
            }
            let rep = r.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(outcome, Some(Outcome::Completed));
        assert!(r.clock.now().saturating_duration_since(t0) < Duration::from_secs(1));
    }

    // 3. Contrast: burst hiding a REAL raised arm (J1 = 0.45 rad) MUST refuse with
    //    TORQUE_OFF_WOULD_DROP and transmit ZERO CMD_DISABLE frames!
    {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        for _ in 0..3 {
            r.step_with_feedback();
        }
        // Arm actually moved to 0.45 rad while an 80-frame burst fills RX.
        r.bus.with(|s| {
            s.tx_log.clear();
            s.motors[0].q = 0.45;
            for _ in 0..80 {
                s.rx.push_back(junk_frame);
            }
        });
        r.submit_transition(TransitionOp::TorqueOff);
        let first = r.step();
        assert!(first.command.as_ref().is_some_and(|(_, res)| res.is_ok()));
        let mut outcome = first.transition_ended.map(|(_, o)| o);
        for _ in 0..10 {
            if outcome.is_some() {
                break;
            }
            let rep = r.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(
            outcome,
            Some(Outcome::Failed("TORQUE_OFF_WOULD_DROP")),
            "L1: burst hiding real raised pose must refuse once drained"
        );
        assert_eq!(spec_count_damiao_disables(&r.bus), 0, "L1: ZERO CMD_DISABLE frames on raised arm");
        assert!(r.fx.core.is_torqued);
    }

    // 4. Contrast: continuous bus flood exhausting every drain until the 1 s lifetime expires
    //    refuses cleanly with TORQUE_OFF_WOULD_DROP and ZERO CMD_DISABLE frames.
    {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        r.bus.with(|s| {
            s.follow_commands = false;
            s.rx.clear();
            s.pending.clear();
            for m in &mut s.motors {
                m.q = 0.0;
            }
        });
        for _ in 0..3 {
            r.step_with_feedback();
        }
        r.bus.with(|s| {
            s.tx_log.clear();
            for _ in 0..80 {
                s.rx.push_back(junk_frame);
            }
        });
        r.submit_transition(TransitionOp::TorqueOff);
        let first = r.step();
        assert!(first.command.as_ref().is_some_and(|(_, res)| res.is_ok()));
        let mut outcome = first.transition_ended.map(|(_, o)| o);
        for _ in 0..60 {
            if outcome.is_some() {
                break;
            }
            r.bus.with(|s| {
                for _ in 0..80 {
                    s.rx.push_back(junk_frame);
                }
            });
            let rep = r.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(
            outcome,
            Some(Outcome::Failed("TORQUE_OFF_WOULD_DROP")),
            "L1: permanently flooded bus must refuse when 1 s lifetime expires"
        );
        assert_eq!(spec_count_damiao_disables(&r.bus), 0, "L1: ZERO CMD_DISABLE frames on 1 s expiry");
        assert!(r.fx.core.is_torqued);
    }
}

/// L2 (part 1): Enable requires causal `verified_modes` for every joint it energizes (not just
/// assumed `joint_modes`). When `PMAX`/`VMAX`/`TMAX` range replies arrive while `RID_CTRL_MODE`
/// is missing, enable transmits ZERO `CMD_ENABLE` frames; and a failed enable NEVER clears a
/// `verified_modes` flag established by a causal readback after this daemon started.
#[test]
fn l2_missing_ctrl_mode_with_range_replies_never_energizes() {
    use armd::driver::RangeCheckStatus;

    // 1. PMAX/VMAX/TMAX reply OK on all 7 motors while J3 drops RID_CTRL_MODE:
    //    `check_mapping_ranges()` reports Passed, yet J3 `verified_modes` stays None, and
    //    BOTH legacy and RT enable transmit ZERO `CMD_ENABLE` frames to any motor.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            s.motors[2].regs.remove(&damiao::RID_CTRL_MODE);
            for m in &mut s.motors {
                m.enabled = false;
            }
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        let rc = dm.check_mapping_ranges().clone();
        assert_eq!(
            rc.status,
            RangeCheckStatus::Passed,
            "precondition: PMAX/VMAX/TMAX range replies all arrived and passed"
        );
        assert!(dm.joint_hold_mode_verified(0), "J1 causal CTRL_MODE readback must verify");
        assert!(!dm.joint_hold_mode_verified(2), "J3 missing CTRL_MODE must stay unverified");

        // Legacy blocking `set_torque(true)` must fail with ZERO CMD_ENABLE frames.
        bus.clear_log();
        assert!(dm.set_torque(true).is_err(), "L2: enable with missing CTRL_MODE must fail");
        let enables: Vec<_> = bus
            .tx_log()
            .into_iter()
            .filter(|f| f.data == damiao::CMD_ENABLE)
            .collect();
        assert!(
            enables.is_empty(),
            "L2: missing CTRL_MODE with passing ranges must send ZERO CMD_ENABLE frames, got {enables:?}"
        );
        // Failed enable must NOT clear J1..J2/J4..J7 causally verified flags!
        for j in [0usize, 1, 3, 4, 5, 6] {
            assert!(
                dm.joint_hold_mode_verified(j),
                "L2: failed enable must NOT clear causally verified J{}",
                j + 1
            );
        }

        // RT executor `TransitionOp::TorqueOn` must also fail with ZERO CMD_ENABLE frames.
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 1, replies: 16 },
        );
        bus.clear_log();
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        broker
            .submit(
                ClientId(1),
                1,
                auth,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        let mut outcome = None;
        for _ in 0..10 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            let rep = fx.step();
            if let Some((_, o)) = rep.transition_ended {
                outcome = Some(o);
                break;
            }
        }
        assert_eq!(outcome, Some(Outcome::Failed("ENABLE_FAILED")));
        let rt_enables: Vec<_> = bus
            .tx_log()
            .into_iter()
            .filter(|f| f.data == damiao::CMD_ENABLE)
            .collect();
        assert!(
            rt_enables.is_empty(),
            "L2: RT enable with missing CTRL_MODE must send ZERO CMD_ENABLE frames, got {rt_enables:?}"
        );
    }

    // 2. Failed enable on a joint that WAS causally verified after daemon start (e.g. J7
    //    drops CMD_ENABLE or times out a later readback) MUST NOT clear its verified flag.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        for j in 0..7 {
            assert!(dm.joint_hold_mode_verified(j), "J{} causally verified at startup", j + 1);
        }
        // Fail enable on J7 (drop enable frames) -> every joint's verified flag stays true!
        bus.with(|s| s.motors[6].drop_torque_frames = 10);
        assert!(dm.set_torque(true).is_err());
        for j in 0..7 {
            assert!(
                dm.joint_hold_mode_verified(j),
                "L2: failed enable must preserve causal verified_modes on J{}",
                j + 1
            );
        }
    }
}

/// L2 (part 2): An energized joint whose control mode is unverified receives a `RID_CTRL_MODE`
/// readback request every tick. If still unverified after 200 ms, latches a non-recoverable fault
/// naming the joint (`J3`) and reports it unheld (`UnheldNoVerifiedMode`). If it answers before
/// 200 ms, it becomes verified and held with no fault.
#[test]
fn l2_energized_unverified_mode_readback_200ms_bound_latches_non_recoverable_fault() {
    use armd::driver::JointHoldStatus;

    let is_j3_ctrl_mode_read = |f: &armd::transport::CanFrame| -> bool {
        f.id == 0x7FF
            && f.data[0] == 3
            && f.data[1] == 0
            && f.data[2] == damiao::REG_READ
            && f.data[3] == damiao::RID_CTRL_MODE
    };

    // 1. Silent RID_CTRL_MODE on energized J3: every tick requests readback; < 200 ms no fault;
    //    at >= 200 ms latches non-recoverable fault naming J3 and keeps J3 unheld.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = if i < 6 { 0.25 } else { 0.0 };
            }
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm));
        inner.lease_enabled = false;
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();

        // Fail a mode switch on energized J3 while J3 drops RID_CTRL_MODE -> J3 remains
        // physically energized (`OnConfirmed`) with `verified_modes[2] == None`!
        bus.with(|s| {
            s.motors[2].regs.remove(&damiao::RID_CTRL_MODE);
            s.fail_tx_ids.push(0x7FF); // fail mode write to invalidate J3 mode verification
        });
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let hold = vec![armd::driver::JointTarget::position(0.25); 7];
            let modes = [
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
            ];
            let _ = drv.set_joint_modes_confirmed(&modes, &hold);
            assert!(!drv.joint_hold_mode_verified(2), "precondition: J3 mode unverified while energized");
        }
        bus.with(|s| {
            s.fail_tx_ids.clear();
            s.motors[2].regs.remove(&damiao::RID_CTRL_MODE);
        });
        // Enter recoverable WatchdogTimeout so stopped-hold is active (J1..J2/J4..J7 held,
        // unverified J3 must stay UnheldNoVerifiedMode with 0 hold frames and receive a
        // RID_CTRL_MODE readback request every tick until 200 ms latches a non-recoverable fault).
        inner.safety.latch(
            armd::safety::FaultKind::WatchdogTimeout,
            armd::safety::FaultDetail::WatchdogTimeout,
        );
        assert!(inner.safety.recoverable(), "precondition: WatchdogTimeout is recoverable");

        // Tick 0..9 (0 ms .. 180 ms): every tick sends RID_CTRL_MODE readback to J3, J3 is
        // unheld (`UnheldNoVerifiedMode`), and no non-recoverable fault latches yet.
        for tick_idx in 0..10 {
            if tick_idx > 0 {
                clock.advance(Duration::from_millis(20));
            }
            bus.with(|s| s.push_all_feedback());
            bus.clear_log();
            inner.tick(0.02);
            let reads = bus.tx_log().iter().filter(|f| is_j3_ctrl_mode_read(f)).count();
            assert!(
                reads >= 1,
                "L2: tick {tick_idx} ({} ms) must send RID_CTRL_MODE readback to unverified energized J3",
                tick_idx * 20
            );
            assert_eq!(
                inner.joint_hold_status()[2],
                JointHoldStatus::UnheldNoVerifiedMode,
                "L2: unverified J3 must be reported unheld at tick {tick_idx}"
            );
            assert!(
                !bus.tx_log().iter().any(|f| f.id == 0x103 || f.id == 3),
                "L2: unverified J3 must receive ZERO hold frames at tick {tick_idx}"
            );
            assert!(
                inner.safety.recoverable(),
                "L2: non-recoverable fault must NOT latch before 200 ms (tick {tick_idx}, {} ms): {:?}",
                tick_idx * 20,
                inner.safety.fault()
            );
        }

        // Tick 10 (200 ms elapsed): non-recoverable fault naming J3 MUST latch and J3 stays unheld!
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        let fault = inner
            .safety
            .fault()
            .expect("L2: 200 ms unverified energized joint MUST latch a fault");
        assert_eq!(fault.code, FaultCode::HardwareFault, "expected HardwareFault, got {fault:?}");
        assert!(
            !fault.recoverable && !inner.safety.recoverable(),
            "L2: 200 ms unverified-mode fault MUST be non-recoverable: {fault:?}"
        );
        assert!(
            fault.message.as_str().contains("J3"),
            "L2: 200 ms unverified-mode fault must name J3, got: {}",
            fault.message
        );
        assert_eq!(
            inner.joint_hold_status()[2],
            JointHoldStatus::UnheldNoVerifiedMode,
            "L2: timed-out unverified J3 must remain reported unheld"
        );
        assert!(
            !bus.tx_log().iter().any(|f| f.id == 0x103 || f.id == 3),
            "L2: timed-out unverified J3 must still receive ZERO hold frames"
        );
    }

    // 2. Contrast: if J3 answers the per-tick RID_CTRL_MODE readback before 200 ms (at 60 ms),
    //    it becomes verified causally, transitions from UnheldNoVerifiedMode to Held, and no
    //    non-recoverable 200 ms fault ever latches.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = if i < 6 { 0.25 } else { 0.0 };
            }
            s.push_all_feedback();
        });
        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        })
        .with_driver(Box::new(dm));
        inner.lease_enabled = false;
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();

        bus.with(|s| {
            s.motors[2].regs.remove(&damiao::RID_CTRL_MODE);
            s.fail_tx_ids.push(0x7FF);
        });
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let hold = vec![armd::driver::JointTarget::position(0.25); 7];
            let modes = [
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Mit,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
                armd::driver::JointMode::Position,
            ];
            let _ = drv.set_joint_modes_confirmed(&modes, &hold);
            assert!(!drv.joint_hold_mode_verified(2));
        }
        bus.with(|s| {
            s.fail_tx_ids.clear();
            s.motors[2].regs.remove(&damiao::RID_CTRL_MODE);
        });
        inner.safety.latch(
            armd::safety::FaultKind::WatchdogTimeout,
            armd::safety::FaultDetail::WatchdogTimeout,
        );
        assert!(inner.safety.recoverable());

        // 3 ticks (0, 20, 40 ms): unanswered -> still unverified, recoverable.
        for tick_idx in 0..3 {
            if tick_idx > 0 {
                clock.advance(Duration::from_millis(20));
            }
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
            assert_eq!(inner.joint_hold_status()[2], JointHoldStatus::UnheldNoVerifiedMode);
            assert!(inner.safety.recoverable());
        }
        // At 60 ms, restore J3's CTRL_MODE register so the per-tick readback succeeds!
        bus.with(|s| s.motors[2].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL));
        for _ in 0..2 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
        assert!(
            inner.driver.as_ref().unwrap().lock().unwrap().joint_hold_mode_verified(2),
            "L2: causal on-tick readback reply before 200 ms must verify J3"
        );
        assert_eq!(
            inner.joint_hold_status()[2],
            JointHoldStatus::Held,
            "L2: verified J3 must transition from UnheldNoVerifiedMode to Held"
        );
        // Advance past 200 ms -> NO non-recoverable fault ever latches because J3 is now verified!
        for _ in 0..10 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
        }
        assert!(
            inner.safety.recoverable(),
            "L2: no non-recoverable fault after J3 verified before 200 ms: {:?}",
            inner.safety.fault()
        );
    }
}

/// R11 B1: A pending disable (`AwaitRestGate` while collecting the K=3 rest window) must NOT
/// suppress stationary holds. Holds continue on every tick until the rest gate passes and the
/// first disable frame is dispatched (`torque_intent = Off`), after which holds stop immediately.
#[test]
fn r11_b1_pending_disable_continues_holds_until_first_disable_frame_accepted() {
    use armd::TorqueIntent;

    let mut r = ExecRig::new();
    r.bus.with(|s| {
        s.follow_commands = false;
        s.immediate_replies = false;
        for m in &mut s.motors {
            m.q = 0.0;
            m.v = 0.0;
        }
        s.push_all_feedback();
    });
    r.arm_now();
    assert!(r.fx.core.is_torqued);
    assert_eq!(r.fx.core.torque_intent, armd::TorqueIntent::On);

    // Latch E-stop first, then momentarily report off-rest to reset the K=3 window, then return
    // to rest (clearing stale delayed replies from the off-rest pose) with ONLY 1 fresh rest
    // sample (K=1 < 3 -> `PendingDrainOrWindow`).
    r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
    r.bus.with(|s| {
        s.motors[0].q = 0.30;
    });
    r.step_with_feedback();
    r.bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
        s.motors[0].q = 0.0;
    });
    r.step_with_feedback(); // K=1 (< 3)
    // Clear delayed hold-replies from the K=1 tick so the TorqueOff submission tick stays at K=1.
    r.bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
    });

    // Submit TorqueOff while K=1 (< 3): enters `AwaitRestGate` (Pending, 0 CMD_DISABLE frames).
    // Holds MUST continue on this tick!
    let gate_before = r.fx.core.driver.as_ref().unwrap().lock().unwrap().rest_pose_gate_status();
    let eval_before = r.fx.core.may_disable_torque_while_stopped();
    r.bus.clear_log();
    r.submit_transition(TransitionOp::TorqueOff);
    let r_submit = r.step();
    let cmd_res = r_submit.command.unwrap().1;
    assert!(
        cmd_res.is_ok(),
        "TorqueOff must defer into AwaitRestGate, got {cmd_res:?} (gate_before={gate_before:?}, eval_before={eval_before:?})"
    );
    assert!(r_submit.transition_ended.is_none(), "K<3 must stay pending in AwaitRestGate");
    assert!(r.fx.core.disable_pending);
    assert_eq!(r.fx.core.torque_intent, TorqueIntent::On);
    assert_eq!(
        spec_count_damiao_disables(&r.bus),
        0,
        "0 CMD_DISABLE frames while awaiting K=3 rest window"
    );
    let holds_tick1 = r.bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count();
    assert_eq!(
        holds_tick1, 7,
        "B1: all 7 joints must keep receiving hold frames while disable is pending in AwaitRestGate"
    );

    // Tick with K=2 (< 3): still AwaitRestGate, 0 CMD_DISABLE, holds MUST continue!
    r.bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
    });
    r.bus.clear_log();
    let r_k2 = r.step_with_feedback();
    assert!(r_k2.transition_ended.is_none(), "K=2 must still stay pending in AwaitRestGate");
    assert_eq!(r.fx.core.torque_intent, TorqueIntent::On);
    assert_eq!(spec_count_damiao_disables(&r.bus), 0);
    let holds_tick2 = r.bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count();
    assert_eq!(holds_tick2, 7, "B1: all 7 joints must continue receiving hold frames on K=2 tick");

    // Tick with K=3: rest gate confirms and first CMD_DISABLE batch dispatches!
    // Once accepted (`torque_intent == Off`), ZERO hold frames may be emitted on this or later ticks.
    r.bus.with(|s| {
        s.rx.clear();
        s.pending.clear();
    });
    r.bus.clear_log();
    let r_k3 = r.step_with_feedback();
    let disables_k3 = spec_count_damiao_disables(&r.bus);
    assert!(
        disables_k3 >= 7,
        "K=3 confirmed must dispatch first CMD_DISABLE batch (got {disables_k3})"
    );
    let (drv_dispatched, drv_pending) = {
        let d = r.fx.core.driver.as_ref().unwrap().lock().unwrap();
        (d.disable_frames_dispatched(), d.torque_transition_pending())
    };
    assert_eq!(
        r.fx.core.torque_intent,
        TorqueIntent::Off,
        "B1: torque_intent must flip to Off as soon as first disable frame is dispatched (disables_k3={disables_k3}, drv_dispatched={drv_dispatched}, drv_pending={drv_pending}, sup_trans={}, r_k3={r_k3:?}, tx_log={:?})",
        r.fx.core.torque_transition_pending(),
        r.bus.tx_log()
    );
    let holds_after_accept =
        r.bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count();
    assert_eq!(
        holds_after_accept, 0,
        "B1: holds must stop immediately once the first disable frame is dispatched"
    );

    // Step until bounded repeats complete; zero holds throughout and clean Done.
    let mut completed = false;
    for _ in 0..10 {
        r.bus.clear_log();
        let step = r.step_with_feedback();
        let any_hold = r.bus.tx_log().iter().any(|f| (0x101..=0x107).contains(&f.id));
        assert!(!any_hold, "B1: no holds during disable repeat frames");
        if step.transition_ended.map(|(_, o)| o) == Some(Outcome::Completed) {
            completed = true;
            break;
        }
    }
    assert!(completed, "B1: disable transition must complete cleanly after repeats");
    assert!(!r.fx.core.disable_pending);
    assert_eq!(r.fx.core.torque_intent, TorqueIntent::Off);
}

/// R11 B2: Holds must NEVER be dropped while `torque_intent == On`:
/// 1. J3 loses feedback -> J1, J2, J4–J7 are still held, and J3 keeps holding its LAST captured
///    hold target and is reported `HeldStaleTarget` ("hold target stale").
/// 2. When MIT ownership is `MitOwner::Lost`, `service_stopped_measured_hold` does NOT return
///    early: MIT joints are excluded while ALL non-MIT position joints (J1–J3, J5–J7) stay held!
#[test]
fn r11_b2_holds_never_dropped_on_stale_feedback_or_mit_owner_lost() {
    use armd::driver::JointHoldStatus;
    use armd::mit::MitOwner;
    use armd::TorqueIntent;

    // Part 1: J3 loses feedback -> J1, J2, J4–J7 remain held; J3 keeps its last captured target
    // and reports `HeldStaleTarget` ("hold target stale").
    {
        let (mut inner, bus, clock) = make_driver_rig();
        let captured_pose = [0.12_f32, -0.25, 0.42, -0.18, 0.20, -0.10, 0.05];
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = captured_pose[i];
            }
            s.push_all_feedback();
        });
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);

        // Capture holds on all 7 joints while fresh.
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        assert_eq!(inner.torque_intent, TorqueIntent::On);
        assert_eq!(inner.joint_hold_status(), &[JointHoldStatus::Held; 7]);

        // Mute ONLY J3 and advance past 100 ms (140 ms, below 500 ms BUS_LOSS) while feeding J1, J2, J4–J7.
        bus.with(|s| {
            s.motors[2].silent = true;
        });
        for _ in 0..7 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| {
                for j in [0, 1, 3, 4, 5, 6] {
                    s.push_feedback(j);
                }
            });
            bus.clear_log();
            inner.tick(0.02);
        }

        // J3 must report `HeldStaleTarget` ("hold target stale") AND keep sending its last target 0.42!
        assert_eq!(inner.joint_hold_status()[2], JointHoldStatus::HeldStaleTarget);
        assert_eq!(inner.joint_hold_status()[2].short_tag(), "hold target stale");
        for j in [0, 1, 3, 4, 5, 6] {
            assert_eq!(
                inner.joint_hold_status()[j],
                JointHoldStatus::Held,
                "B2: joint J{} must remain Held when J3 goes stale",
                j + 1
            );
        }
        let tx = bus.tx_log();
        for (j, &expected_q) in captured_pose.iter().enumerate() {
            let can_id = 0x101u32 + j as u32;
            let f = tx
                .iter()
                .find(|f| f.id == can_id)
                .unwrap_or_else(|| panic!("B2: expected hold frame 0x{can_id:03X}, log={tx:?}"));
            let sent_q = decode_damiao_pos_vel_pos(f);
            assert!(
                (sent_q - expected_q).abs() < 2e-3,
                "B2: J{} must hold captured target {expected_q}, got {sent_q}",
                j + 1
            );
        }
    }

    // Part 2: MIT ownership Lost while J4 is configured as MIT with verified `PosVel` mode ->
    // R12 B2: MIT Lost MUST NOT mask out J4; J4 falls back to verified `PosVel` hold (`0x104`)
    // with ZERO MIT frames (`0x004`), and all other position joints stay held.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        let mut jt = armd::tracking::JointTrackingConfig::rebot_dm();
        jt.enabled = false;
        jt.joints[3].strategy = armd::tracking::TrackingStrategy::Mit; // J4 is MIT
        jt.joints[3].kp = 10.0;
        jt.joints[3].kd = 0.5;
        jt.joints[3].effort_limit_nm = 3.0;
        jt.joints[3].gravity_scale = 0.0;
        jt.joints[3].hold_kp = Some(10.0);
        jt.joints[3].hold_kd = Some(0.5);
        cfg.joint_tracking = Some(jt);

        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        let hold_pose = [0.10_f32, 0.20, -0.15, 0.30, -0.20, 0.05, 0.0];
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = hold_pose[i];
            }
            s.push_all_feedback();
        });
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(dm));
        inner.lease_enabled = false;
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();
        inner.config.joint_tracking.as_mut().unwrap().enabled = true;

        // Simulate MIT ownership loss + armed driver MIT fence while stopped; J4 is verified `PosVel`.
        inner.mit.owner = MitOwner::Lost;
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        {
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            drv.set_mit_deadline_fence(Some(Duration::from_millis(50)), Duration::ZERO);
        }

        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);

        let tx = bus.tx_log();
        // J4 MUST NOT receive any MIT frame (`0x004`), and MUST fall back to verified `PosVel` (`0x104`).
        assert!(
            tx.iter().all(|f| f.id != 0x004),
            "R12 B2: MIT-lost joint J4 must not receive MIT frames (0x004): {tx:?}"
        );
        // Every joint including J4 (0x101..=0x107) MUST be held in verified `PosVel` mode!
        for j in 0..7usize {
            let can_id = 0x101u32 + j as u32;
            let f = tx
                .iter()
                .find(|f| f.id == can_id)
                .unwrap_or_else(|| panic!("R12 B2: joint J{} (0x{can_id:03X}) must be held in PosVel under MitOwner::Lost: {tx:?}", j + 1));
            let sent_q = decode_damiao_pos_vel_pos(f);
            assert!(
                (sent_q - hold_pose[j]).abs() < 2e-3,
                "R12 B2: J{} must hold {}, got {sent_q}",
                j + 1,
                hold_pose[j]
            );
            assert_eq!(inner.joint_hold_status()[j], JointHoldStatus::Held);
        }
    }
}

/// R11 B3: Watchdog `NormalUnload` must NOT bypass the K=3 rest-pose gate after a park settle:
/// 1. If the park settles where the static policy allows it (e.g. wider policy tolerance) or the
///    driver's rest-pose gate refuses (joint outside `rest_pose_tolerance_rad`, or K<3 / non-rest),
///    the post-settle disable is REFUSED with 0 `CMD_DISABLE` frames, `torque_intent` stays `On`,
///    holds continue, and `WatchdogTimeout` latches.
/// 2. When the park genuinely reaches rest and completes K=3, the rest gate accepts the disable,
///    transmits `CMD_DISABLE`, records `torque_intent = Off` by decision, and enters `Idle`.
#[test]
fn r11_b3_watchdog_normal_unload_enforces_rest_pose_gate_and_holds_on_refusal() {
    use armd::TorqueIntent;

    // Part 1: Rest gate refuses when park settles at 0.07 rad (inside wider 0.08 rad static
    // `resting_pose.position_tolerances`, OUTSIDE strict 0.05 rad `rest_pose_tolerance_rad`!).
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.0;
                m.v = 0.0;
            }
            s.motors[0].q = 0.07; // > 0.05 rad rest_pose_tolerance_rad, < 0.10 rad static policy!
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        let mut cfg = ArmConfig {
            lease_timeout_ms: 50,
            rest_pose_tolerance_rad: 0.05,
            ..ArmConfig::rebot_dm()
        };
        if let Some(ref mut rp) = cfg.resting_pose {
            rp.position_tolerances = vec![0.10; 7];
            rp.max_total_deviation = 0.50;
        }
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(d));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();

        // Trigger watchdog NormalUnload park.
        inner.lease_enabled = true;
        inner.last_heartbeat = Instant::now() - Duration::from_millis(200);
        bus.clear_log();
        // Tick through park trajectory + 800 ms unload settle (> 20 ms causal settle window so
        // driver pose feedback stays provably fresh and evaluates the exact 0.07 rad pose).
        for _ in 0..200 {
            clock.advance(Duration::from_millis(21));
            bus.with(|s| {
                s.motors[0].q = 0.07;
                s.motors[0].v = 0.0;
                s.push_all_feedback();
            });
            inner.tick(0.02);
            if matches!(inner.safety.state(), SafetyState::Fault(FaultReason::WatchdogTimeout)) {
                break;
            }
        }
        assert_eq!(
            spec_count_damiao_disables(&bus),
            0,
            "B3: watchdog NormalUnload must NOT bypass rest-pose gate when J1 is at 0.07 > 0.05 rad"
        );
        assert_eq!(
            inner.torque_intent,
            TorqueIntent::On,
            "B3: torque_intent must remain On when rest gate refuses post-settle unload"
        );
        assert!(inner.is_torqued, "B3: arm must remain torqued and holding");
        assert!(
            matches!(inner.safety.state(), SafetyState::Fault(FaultReason::WatchdogTimeout)),
            "B3: refused watchdog unload must report failure by latching WatchdogTimeout, got {:?}",
            inner.safety.state()
        );
        assert!(
            matches!(
                inner.last_stopped_torque_off_refusal(),
                Some(StoppedTorqueOffRefusal::JointOutsideRest { joint_index: 0, .. })
            ),
            "B3: refusal reason must record JointOutsideRest for J1, got {:?}",
            inner.last_stopped_torque_off_refusal()
        );
        // Subsequent tick keeps holding all 7 joints!
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        assert_eq!(
            bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
            7,
            "B3: arm must keep holding after watchdog NormalUnload rest-gate refusal"
        );
    }

    // Part 2: Park genuinely reaches rest (0.0 rad) and passes K=3 rest-pose gate -> accepted,
    // CMD_DISABLE sent, `torque_intent = Off` recorded by decision, and enters Idle!
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        bus.with(|s| {
            s.follow_commands = true;
            s.immediate_replies = true;
            for m in &mut s.motors {
                m.q = 0.0;
                m.v = 0.0;
            }
            s.motors[0].q = 0.25;
            s.push_all_feedback();
        });
        d.read_joint_states().unwrap();
        d.check_mapping_ranges();
        let cfg = ArmConfig {
            lease_timeout_ms: 50,
            ..ArmConfig::rebot_dm()
        };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(d));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();
        inner.lease_enabled = true;
        inner.last_heartbeat = Instant::now() - Duration::from_millis(200);
        bus.clear_log();
        for _ in 0..200 {
            clock.advance(Duration::from_millis(21));
            bus.with(|s| s.push_all_feedback());
            inner.tick(0.02);
            if *inner.safety.state() == SafetyState::Idle && !inner.is_torqued {
                break;
            }
        }
        assert_eq!(*inner.safety.state(), SafetyState::Idle);
        assert!(!inner.is_torqued);
        assert_eq!(inner.torque_intent, TorqueIntent::Off);
        assert!(
            spec_count_damiao_disables(&bus) >= 7,
            "B3: watchdog NormalUnload at rest must transmit CMD_DISABLE after passing K=3 gate"
        );
    }
}

/// R11 B4: Feetech `write_torque_off` must recheck stopped state, 1 s lifetime, and K=3 rest
/// evidence IMMEDIATELY before transmitting `TORQUE_ENABLE = 0`, strictly AFTER any RX
/// clear/drain inside the same function.
/// 1. An E-stop latching during RX drain while the arm lacks rest evidence -> refuses with
///    `TorqueOffWouldDrop` and writes ZERO `TORQUE_ENABLE = 0` packets.
/// 2. Request lifetime expiring during RX drain while stopped at rest -> refuses with
///    `DisableRequestExpired` and writes ZERO `TORQUE_ENABLE = 0` packets.
#[test]
fn r11_b4_feetech_disable_rechecks_stopped_state_and_rest_evidence_after_drain() {
    use armd::driver::{ArmDriverError, DisableKind, FeetechArmDriver, StoppedDisableGate};
    use armd::transport::MockTransport;

    #[derive(Clone)]
    struct DrainHookSerial {
        inner: Arc<std::sync::Mutex<MockTransport>>,
        on_clear: Arc<std::sync::Mutex<Option<Box<dyn FnMut() + Send>>>>,
    }
    impl armd::transport::Transport for DrainHookSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
            self.inner.lock().unwrap().write(buf)
        }
        fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
            self.inner.lock().unwrap().write_all(buf)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
            self.inner.lock().unwrap().read(buf)
        }
        fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
            self.inner.lock().unwrap().flush()
        }
        fn clear_input_buffer(&mut self) {
            self.inner.lock().unwrap().clear_input_buffer();
            if let Some(ref mut hook) = *self.on_clear.lock().unwrap() {
                hook();
            }
        }
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            armd::transport::Transport::clock(&*self.inner.lock().unwrap())
        }
    }

    let servo_ids: Vec<u8> = (1..=6).collect();
    let cfg = feetech_6dof_config();
    let sync_pos_req_1 = armd::protocol::feetech::build_sync_read(
        &servo_ids[..3],
        armd::protocol::feetech::REG_PRESENT_POSITION_L,
        8,
    )
    .unwrap();
    let sync_pos_req_2 = armd::protocol::feetech::build_sync_read(
        &servo_ids[3..],
        armd::protocol::feetech::REG_PRESENT_POSITION_L,
        8,
    )
    .unwrap();

    let make_gate = |stopped: bool| StoppedDisableGate {
        safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(stopped)),
        has_resting_pose: true,
        arm_joints: {
            let mut a = [0usize; armd::rt_core::mailbox::MAX_DOF];
            for i in 0..5 {
                a[i] = i;
            }
            a
        },
        arm_joints_len: 5,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: cfg.rest_pose_tolerance_rad,
    };

    // Case 1: Operational arm at raised pose (no rest evidence); E-stop latches DURING the RX
    // clear/drain inside `write_torque_off` -> post-drain recheck MUST refuse with 0 writes!
    {
        let clock = MockClock::new();
        let serial_inner = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let on_clear: Arc<std::sync::Mutex<Option<Box<dyn FnMut() + Send>>>> =
            Arc::new(std::sync::Mutex::new(None));
        let estop_state = Arc::new(armd::rt_core::urgent::EstopState::new());
        let raised_steps: [u16; 6] = [2600, 2048, 2048, 2048, 2048, 2048];
        {
            let mut ser = serial_inner.lock().unwrap();
            ser.stage_reply(
                &sync_pos_req_1,
                &feetech_feedback_bytes(&servo_ids[..3], &raised_steps[..3]),
            );
            ser.stage_reply(
                &sync_pos_req_2,
                &feetech_feedback_bytes(&servo_ids[3..], &raised_steps[3..]),
            );
        }
        let mut drv = FeetechArmDriver::new(
            DrainHookSerial {
                inner: Arc::clone(&serial_inner),
                on_clear: Arc::clone(&on_clear),
            },
            servo_ids.clone(),
        )
        .with_calibrated(true)
        .with_tx_echo(Some(false))
        .with_gripper_index(Some(5));
        drv.set_estop_state(Arc::clone(&estop_state));
        drv.set_stopped_disable_gate(make_gate(false));
        drv.read_joint_states().unwrap();
        let _ = serial_inner.lock().unwrap().take_tx();

        // Latch E-stop during `clear_input_buffer()` (between pre-drain check and `write_all`).
        let estop_hook = Arc::clone(&estop_state);
        *on_clear.lock().unwrap() = Some(Box::new(move || {
            let _ = estop_hook.request_and_latch();
        }));

        let started_at = drv.clock_now();
        let res = drv.disable_torque(DisableKind::DisableRequested {
            started_at,
        });
        assert!(
            matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
            "B4: E-stop latching during RX drain without rest evidence must refuse with TorqueOffWouldDrop, got {res:?}"
        );
        let tx = serial_inner.lock().unwrap().take_tx();
        assert_eq!(
            spec_count_feetech_torque_off_writes(&tx),
            0,
            "B4: ZERO TORQUE_ENABLE=0 writes permitted when E-stop latches between drain and write"
        );
    }

    // Case 2: Stopped arm at rest with K=3 evidence; 1 s lifetime expires DURING RX drain ->
    // post-drain recheck MUST refuse with `DisableRequestExpired` and write ZERO packets!
    {
        let clock = MockClock::new();
        let serial_inner = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let on_clear: Arc<std::sync::Mutex<Option<Box<dyn FnMut() + Send>>>> =
            Arc::new(std::sync::Mutex::new(None));
        let rest_steps: [u16; 6] = [2048; 6];
        let mut drv = FeetechArmDriver::new(
            DrainHookSerial {
                inner: Arc::clone(&serial_inner),
                on_clear: Arc::clone(&on_clear),
            },
            servo_ids.clone(),
        )
        .with_calibrated(true)
        .with_tx_echo(Some(false))
        .with_gripper_index(Some(5));
        drv.set_stopped_disable_gate(make_gate(true));
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(12));
            }
            {
                let mut ser = serial_inner.lock().unwrap();
                ser.stage_reply(
                    &sync_pos_req_1,
                    &feetech_feedback_bytes(&servo_ids[..3], &rest_steps[..3]),
                );
                ser.stage_reply(
                    &sync_pos_req_2,
                    &feetech_feedback_bytes(&servo_ids[3..], &rest_steps[3..]),
                );
            }
            drv.read_joint_states().unwrap();
        }
        let _ = serial_inner.lock().unwrap().take_tx();

        // Advance clock past 1 s request lifetime during `clear_input_buffer()`.
        let clock_hook = clock.clone();
        *on_clear.lock().unwrap() = Some(Box::new(move || {
            clock_hook.advance(Duration::from_millis(1100));
        }));

        let started_at = drv.clock_now();
        let res = drv.disable_torque(DisableKind::DisableRequested {
            started_at,
        });
        assert!(
            matches!(res, Err(ArmDriverError::DisableRequestExpired)),
            "B4: 1 s request expiry during RX drain must refuse with DisableRequestExpired, got {res:?}"
        );
        let tx = serial_inner.lock().unwrap().take_tx();
        assert_eq!(
            spec_count_feetech_torque_off_writes(&tx),
            0,
            "B4: ZERO TORQUE_ENABLE=0 writes permitted when 1 s lifetime expires between drain and write"
        );
    }
}

/// R11 M1: Reported `enable_state()` is telemetry only when holding/stopped (never latches
/// `UnexpectedTorqueLoss` or drops holds), while during active motion `UnexpectedTorqueLoss`
/// stays a fault AND the arm keeps sending holds (`torque_intent == On`).
#[test]
fn r11_m1_reported_enable_state_is_telemetry_when_holding_and_keeps_holds_on_motion_fault() {
    use armd::TorqueIntent;

    let (mut inner, bus, clock) = make_driver_rig();
    bus.with(|s| {
        s.follow_commands = false;
        s.immediate_replies = true;
        for m in &mut s.motors {
            m.q = 0.0;
            m.v = 0.0;
        }
        s.push_all_feedback();
    });
    inner.tick(0.02);
    enable_torque_admitted(&mut inner).unwrap();
    assert_eq!(*inner.safety.state(), SafetyState::Armed);
    assert_eq!(inner.torque_intent, TorqueIntent::On);

    // Part 1: Stationary / holding (no active motion) — motor feedback reports disabled (status=0).
    // Must NOT latch `UnexpectedTorqueLoss` and must keep sending control/hold frames while
    // exposing `EnableState::Disabled` in telemetry!
    bus.with(|s| {
        for m in &mut s.motors {
            m.enabled = false;
        }
    });
    for _ in 0..3 {
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        assert_eq!(
            *inner.safety.state(),
            SafetyState::Armed,
            "M1: reported enable_state == Disabled while holding must NOT latch UnexpectedTorqueLoss"
        );
        assert_eq!(inner.torque_intent, TorqueIntent::On);
        assert_eq!(
            inner.current_observation().enable_state,
            EnableState::Disabled,
            "M1: reported enable_state must still be published in telemetry"
        );
        assert_eq!(
            bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
            7,
            "M1: holding frames must continue when feedback claims disabled"
        );
    }

    // Restore reported enabled=true, start active motion (jog trajectory), then inject status=0:
    // During active motion, `UnexpectedTorqueLoss` MUST latch as a fault AND the arm MUST keep
    // sending stopped holds (`torque_intent == On`)!
    bus.with(|s| {
        for m in &mut s.motors {
            m.enabled = true;
        }
        s.push_all_feedback();
    });
    clock.advance(Duration::from_millis(20));
    inner.tick(0.02);

    let stamp = inner.current_admission_stamp();
    let admitted = inner.check_admission(stamp).unwrap();
    SupervisorInner::jog(admitted, vec![0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Some(1.0), None)
        .unwrap();
    assert!(inner.active_trajectory.is_some());

    bus.with(|s| {
        for m in &mut s.motors {
            m.enabled = false;
        }
    });
    clock.advance(Duration::from_millis(20));
    bus.with(|s| s.push_all_feedback());
    bus.clear_log();
    inner.tick(0.02);

    assert!(
        inner.safety.state().is_stopped(),
        "M1: reported torque loss during active motion must latch UnexpectedTorqueLoss fault"
    );
    assert_eq!(
        inner.safety.fault().map(|f| f.message.to_string()),
        Some("Unexpected motor torque loss detected while operational".to_string()),
        "M1: fault message must be UnexpectedTorqueLoss"
    );
    assert!(inner.active_trajectory.is_none(), "active motion must stop on UnexpectedTorqueLoss");
    assert_eq!(inner.torque_intent, TorqueIntent::On, "M1: torque_intent stays On so holds continue");
    assert_eq!(
        bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
        7,
        "M1: arm must keep sending stationary holds on the UnexpectedTorqueLoss tick"
    );

    // Subsequent tick while faulted also keeps sending holds on all 7 joints!
    clock.advance(Duration::from_millis(20));
    bus.with(|s| s.push_all_feedback());
    bus.clear_log();
    inner.tick(0.02);
    assert_eq!(
        bus.tx_log().iter().filter(|f| (0x101..=0x107).contains(&f.id)).count(),
        7,
        "M1: arm must continue sending holds after UnexpectedTorqueLoss"
    );
}

#[test]
fn r11_m2_damiao_rest_gate_covers_every_cmd_disable_motor_including_j7() {
    let mut r = ExecRig::new();
    r.arm_now();
    r.fx.core.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);

    // Silence J7 while J1..J6 accumulate K=3 fresh rest samples across 3 distinct releases.
    r.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.delayed.clear();
        s.motors[6].silent = true;
    });
    // Wait > 100 ms so J7's prior feedback expires.
    r.clock.advance(Duration::from_millis(120));
    r.step(); // observe empty RX queue boundary
    for _ in 0..3 {
        r.clock.advance(Duration::from_millis(10));
        r.bus.with(|s| {
            for j in 0..6 {
                s.push_feedback(j);
            }
        });
        r.step();
    }

    // With J7 stale, Damiao rest gate MUST NOT be Confirmed: M3 defers into AwaitRestGate with
    // 0 CMD_DISABLE frames while probing J7 across releases!
    r.bus.clear_log();
    r.submit_transition(TransitionOp::TorqueOff);
    let rep0 = r.step();
    assert!(
        matches!(rep0.command, Some((_, Ok(_)))),
        "M2+M3: TorqueOff with silent J7 defers in AwaitRestGate, got {:?}",
        rep0.command
    );
    assert!(rep0.transition_ended.is_none());
    for _ in 0..3 {
        let rep = r.step();
        assert!(
            rep.transition_ended.is_none(),
            "M2: TorqueOff must remain pending while J7 is silent"
        );
    }
    assert_eq!(
        r.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "M2: silent J7 must prevent CMD_DISABLE transmission even when J1..J6 are confirmed at rest"
    );
    assert!(
        r.bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[0] == 7 && f.data[2] == 0xCC),
        "M2: poll_await_rest_gate must emit 0xCC probe for J7 (motor 7)"
    );
    assert!(r.fx.core.is_torqued);
    assert_eq!(r.fx.core.torque_intent, armd::TorqueIntent::On);

    // Unmute J7 at an arbitrary non-zero jaw position (-2.5 rad, far outside arm rest tolerance):
    // 1st J7 sample is NOT enough (gate stays PendingDrainOrWindow, 0 CMD_DISABLE); once probes
    // collect K=3 fresh J7 samples across 3 releases, the SAME pending TorqueOff completes!
    r.bus.with(|s| {
        s.motors[6].silent = false;
        s.motors[6].q = -2.5;
    });
    let rep1 = r.step();
    assert!(
        rep1.transition_ended.is_none(),
        "M2: single J7 sample must not satisfy K=3 rest gate yet"
    );
    assert_eq!(
        r.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "M2: 0 CMD_DISABLE frames before J7 reaches K=3"
    );

    let mut completed = false;
    for _ in 0..10 {
        let rep = r.step();
        if let Some((_, outcome)) = rep.transition_ended {
            assert_eq!(
                outcome,
                Outcome::Completed,
                "M2: TorqueOff must complete once J7 reaches K=3 at -2.5 rad: {outcome:?}"
            );
            completed = true;
            break;
        }
    }
    assert!(completed, "M2: TorqueOff must complete after J7 probes collect K=3 samples");
    assert!(
        r.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count() >= 7,
        "M2: all 7 motors receive CMD_DISABLE once J7 reaches K=3"
    );
    assert!(!r.fx.core.is_torqued);
    assert_eq!(r.fx.core.torque_intent, armd::TorqueIntent::Off);
}

#[test]
fn r11_m3_drained_bus_stale_or_missing_defers_and_probes_within_1s_lifetime() {
    // Part A: Drained bus with stale (> 100 ms) feedback while stopped & motors alive at rest ->
    // TorqueOff MUST defer (Pending, 0 CMD_DISABLE), run 0xCC probe loop, and succeed within 1 s!
    let mut r = ExecRig::new();
    r.arm_now();
    r.fx.core.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    r.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.delayed.clear();
    });
    // Advance > 100 ms without pushing new feedback (bus is drained and stale).
    r.clock.advance(Duration::from_millis(150));
    {
        let drv = r.fx.core.driver.as_ref().unwrap().lock().unwrap();
        assert_eq!(
            drv.rest_pose_gate_status(),
            armd::driver::RestPoseGateStatus::StaleOrMissing,
            "precondition: drained bus with >100ms feedback must report StaleOrMissing"
        );
    }

    r.bus.clear_log();
    r.submit_transition(TransitionOp::TorqueOff);
    let rep0 = r.step();
    assert!(
        matches!(rep0.command, Some((_, Ok(_)))),
        "M3: drained stale bus must defer TorqueOff instead of refusing immediately, got {:?}",
        rep0.command
    );
    assert!(rep0.transition_ended.is_none(), "M3: must wait in AwaitRestGate probe loop");
    assert_eq!(
        r.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "M3: 0 CMD_DISABLE frames before probe loop gathers K=3 fresh rest samples"
    );

    // Subsequent releases run the 0xCC probe loop, collect K=3 fresh rest replies, and disable!
    let mut done = false;
    for _ in 0..10 {
        let rep = r.step();
        if let Some((_, outcome)) = rep.transition_ended {
            assert_eq!(outcome, Outcome::Completed, "M3: probe loop must confirm rest and disable: {outcome:?}");
            done = true;
            break;
        }
    }
    assert!(done, "M3: deferred stale-bus disable must complete via probe loop within 1 s");
    assert!(
        r.bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
        "M3: must transmit read-only 0xCC status probes while awaiting stale/missing evidence"
    );
    assert!(!r.fx.core.is_torqued);
    assert_eq!(r.fx.core.torque_intent, armd::TorqueIntent::Off);

    // Part B: Drained bus with silent motors -> defers and probes, then expires at 1 s with
    // TORQUE_OFF_WOULD_DROP and 0 CMD_DISABLE frames.
    let mut r_silent = ExecRig::new();
    r_silent.arm_now();
    r_silent.fx.core.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    r_silent.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.delayed.clear();
        for m in &mut s.motors {
            m.silent = true;
        }
    });
    r_silent.clock.advance(Duration::from_millis(150));
    r_silent.bus.clear_log();
    r_silent.submit_transition(TransitionOp::TorqueOff);
    let rep_s0 = r_silent.step();
    assert!(matches!(rep_s0.command, Some((_, Ok(_)))), "M3: silent bus must defer first");
    assert!(rep_s0.transition_ended.is_none());

    // Advance through probes until the 1 s lifetime expires (50 Hz = 50 releases = 1.0 s).
    let mut expired = false;
    for _ in 0..60 {
        let rep = r_silent.step();
        if let Some((_, outcome)) = rep.transition_ended {
            assert!(
                matches!(outcome, Outcome::Failed("TORQUE_OFF_WOULD_DROP")),
                "M3: silent bus must expire at 1 s with TORQUE_OFF_WOULD_DROP, got {outcome:?}"
            );
            expired = true;
            break;
        }
    }
    assert!(expired, "M3: silent bus disable must expire at 1 s lifetime");
    assert!(
        r_silent.bus.tx_log().iter().any(|f| f.id == 0x7FF && f.data[2] == 0xCC),
        "M3: silent bus must still have run 0xCC probes before expiry"
    );
    assert_eq!(
        r_silent.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "M3: silent bus expiry must emit 0 CMD_DISABLE frames"
    );
    assert!(r_silent.fx.core.is_torqued);
    assert_eq!(r_silent.fx.core.torque_intent, armd::TorqueIntent::On);

    // Part C: Real non-rest pose refuses IMMEDIATELY (no 1 s wait, 0 CMD_DISABLE frames).
    let mut r_raised = ExecRig::new();
    r_raised.arm_now();
    r_raised.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[1].q = 0.45;
        s.push_all_feedback();
    });
    r_raised.step();
    r_raised.fx.core.safety.trigger_fault_coded(FaultCode::BusLoss, FaultReason::BusLoss);
    r_raised.bus.clear_log();
    r_raised.submit_transition(TransitionOp::TorqueOff);
    let rep_r = r_raised.step();
    assert_eq!(
        rep_r.command.map(|(_, r)| r),
        Some(Err("TORQUE_OFF_WOULD_DROP")),
        "M3: real non-rest pose must still refuse IMMEDIATELY without deferring"
    );
    assert_eq!(
        r_raised.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0
    );
}

/// Round 12 Decision / B5: ALL `DisableRequested` go through the single K=3 rest-pose gate in
/// EVERY safety state (operational `Armed` / `Idle` as well as stopped `EStop` / `Fault`).
/// The ONLY ungated disable is `DisableProtective` (`Thermal` / `ProcessExit`).
#[test]
fn r12_b5_operational_torque_off_enforces_rest_gate_on_both_owners_and_drivers() {
    use armd::driver::{DisableKind, DisableProtectiveReason, TorqueProgress};
    use armd::rt_core::broker::{Outcome, TransitionOp};

    // 1. Operational RT owner at raised pose -> refused with 0 CMD_DISABLE frames.
    let mut rt_raised = ExecRig::new();
    rt_raised.arm_now();
    rt_raised.bus.with(|s| {
        s.follow_commands = false;
        s.rx.clear();
        s.pending.clear();
        s.motors[1].q = 0.45;
        s.push_all_feedback();
    });
    rt_raised.step();
    assert!(rt_raised.fx.core.safety.state().is_operational());
    rt_raised.bus.clear_log();
    rt_raised.submit_transition(TransitionOp::TorqueOff);
    let rep = rt_raised.step();
    assert!(
        matches!(rep.command, Some((_, Err("ARM_NOT_AT_RESTING_POSE" | "TORQUE_OFF_WOULD_DROP")))),
        "B5: operational RT TorqueOff at raised pose must refuse, got {rep:?}"
    );
    assert_eq!(
        rt_raised.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "B5: operational RT TorqueOff at raised pose must transmit 0 CMD_DISABLE frames"
    );
    assert_eq!(rt_raised.fx.core.torque_intent, armd::TorqueIntent::On);

    // 2. Operational RT owner at rest with < K=3 samples -> defers (0 CMD_DISABLE on submit),
    //    then completes once K=3 rest samples arrive.
    let mut rt_rest = ExecRig::new();
    rt_rest.arm_now();
    assert!(rt_rest.fx.core.safety.state().is_operational());
    rt_rest.bus.clear_log();
    rt_rest.submit_transition(TransitionOp::TorqueOff);
    let rep0 = rt_rest.step();
    assert!(
        matches!(rep0.command, Some((_, Ok(_)))),
        "B5: operational RT TorqueOff at rest with incomplete K=3 must defer, got {rep0:?}"
    );
    assert_eq!(
        rt_rest.bus.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "B5: operational RT TorqueOff must emit 0 CMD_DISABLE frames before K=3 rest confirmation"
    );
    let mut completed = false;
    for _ in 0..20 {
        let rep = rt_rest.step();
        if let Some((_, Outcome::Completed)) = rep.transition_ended {
            completed = true;
            break;
        }
    }
    assert!(completed, "B5: operational RT TorqueOff at rest must complete after K=3");
    assert_eq!(rt_rest.fx.core.torque_intent, armd::TorqueIntent::Off);
    assert!(!rt_rest.fx.core.is_torqued);

    // 3. Direct driver choke point while safety_stopped == false:
    //    DisableRequested at raised pose -> TorqueOffWouldDrop (0 TX);
    //    DisableProtective at raised pose -> succeeds (ungated).
    let (mut inner_leg, bus_leg, clock_leg) = make_driver_rig();
    enable_torque_admitted(&mut inner_leg).unwrap();
    assert!(inner_leg.safety.state().is_operational());
    bus_leg.with(|s| {
        s.follow_commands = false;
        s.motors[2].q = 0.50;
        s.push_all_feedback();
    });
    clock_leg.advance(Duration::from_millis(20));
    inner_leg.tick(0.02);
    bus_leg.clear_log();
    // Legacy operational disable_torque at raised pose refuses with 0 CMD_DISABLE.
    assert!(inner_leg.disable_torque().is_err());
    assert_eq!(
        bus_leg.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
        0,
        "B5: legacy operational disable_torque at raised pose must send 0 CMD_DISABLE"
    );
    // Direct driver begin_disable_torque(DisableRequested) while operational + raised also refuses (0 TX).
    {
        let mut drv = inner_leg.driver.as_ref().unwrap().lock().unwrap();
        let started_at = drv.clock_now();
        let err = drv
            .begin_disable_torque(DisableKind::DisableRequested { started_at })
            .unwrap_err();
        assert!(
            matches!(err, armd::driver::ArmDriverError::TorqueOffWouldDrop),
            "B5: direct operational DisableRequested at raised pose must refuse with TorqueOffWouldDrop, got {err:?}"
        );
        assert_eq!(
            bus_leg.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count(),
            0
        );
        // DisableProtective remains ungated at raised pose.
        let prog = drv
            .begin_disable_torque(DisableKind::DisableProtective {
                reason: DisableProtectiveReason::Thermal,
            })
            .expect("B5: DisableProtective must remain ungated at raised pose");
        assert!(matches!(prog, TorqueProgress::Pending { .. } | TorqueProgress::Done(_)));
        assert!(
            bus_leg.tx_log().iter().filter(|f| f.data == damiao::CMD_DISABLE).count() >= 7,
            "B5: DisableProtective must transmit CMD_DISABLE to all motors even at raised pose"
        );
    }
}

/// Round 12 B1: Feetech (legacy) disable waiting for K=3 must NOT hold the driver mutex while
/// collecting evidence or sleeping, and must NOT starve measured-position holds between releases.
#[test]
fn r12_b1_feetech_legacy_pending_disable_continues_holds_without_starvation() {
    use armd::driver::{ArmDriver, FeetechArmDriver};
    use armd::protocol::feetech;
    use armd::transport::MockTransport;
    use armd::TorqueIntent;

    #[derive(Clone)]
    struct SharedSerial(Arc<std::sync::Mutex<MockTransport>>);
    impl armd::transport::Transport for SharedSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().write(buf)
        }
        fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().write_all(buf)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
            self.0.lock().unwrap().read(buf)
        }
        fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
            self.0.lock().unwrap().flush()
        }
        fn clear_input_buffer(&mut self) {
            self.0.lock().unwrap().clear_input_buffer();
        }
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            armd::transport::Transport::clock(&*self.0.lock().unwrap())
        }
    }

    let clock = MockClock::new();
    let serial = Arc::new(std::sync::Mutex::new(
        MockTransport::new().with_clock(clock.clone()),
    ));
    let servo_ids = vec![1u8, 2, 3, 4, 5, 6];
    let req1 = feetech::build_sync_read(&servo_ids[..3], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    let req2 = feetech::build_sync_read(&servo_ids[3..], feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    {
        let mut ser = serial.lock().unwrap();
        ser.stage_reply(&req1, &feetech_feedback_bytes(&servo_ids[..3], &[2048; 3]));
        ser.stage_reply(&req2, &feetech_feedback_bytes(&servo_ids[3..], &[2048; 3]));
        for id in 1..=6u8 {
            let r_en = feetech::build_read(id, feetech::REG_TORQUE_ENABLE, 1);
            ser.stage_reply(&r_en, &feetech_reg_u8_resp(id, 0));
        }
    }
    let mut drv = FeetechArmDriver::new(SharedSerial(Arc::clone(&serial)), servo_ids.clone())
        .with_calibrated(true)
        .with_tx_echo(Some(false))
        .with_gripper_index(Some(5));
    drv.read_joint_states().unwrap(); // only 1 sample so far (K < 3)

    // Legacy owner (`rt_executor_owned == false`, as Feetech always runs on legacy).
    let mut inner = SupervisorInner::new(feetech_6dof_config()).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    assert!(!inner.rt_executor_owned);
    inner.apply_stop_policy(StopPolicy::EStopLatch);

    // Trigger disable while K=3 is still incomplete: MUST return immediately without sleeping
    // or advancing the mock clock, leaving `disable_pending == true`, `torque_intent == On`,
    // and 0 `TORQUE_ENABLE = 0` writes.
    let _ = serial.lock().unwrap().take_tx();
    let t_before = clock.now();
    let st = inner
        .disable_torque()
        .expect("B1: legacy Feetech disable awaiting K=3 must defer cleanly");
    assert_eq!(st, SafetyState::EStop);
    assert_eq!(
        clock.now(),
        t_before,
        "B1: legacy Feetech disable_torque must NOT sleep while awaiting K=3"
    );
    assert!(inner.disable_pending, "B1: disable must remain pending until K=3 completes");
    assert_eq!(inner.torque_intent, TorqueIntent::On);
    assert_eq!(
        spec_count_feetech_torque_off_writes(&serial.lock().unwrap().take_tx()),
        0,
        "B1: zero TORQUE_ENABLE=0 writes before K=3"
    );

    // Driver mutex MUST be free immediately after `disable_torque()` returned.
    assert!(
        inner.driver.as_ref().unwrap().try_lock().is_ok(),
        "B1: driver mutex must be released immediately after begin_disable_torque"
    );

    // Release 1 (+20 ms, sample 1 of 3 after gate installed): disable stays pending AND holds ARE transmitted!
    let _ = serial.lock().unwrap().take_tx();
    clock.advance(Duration::from_millis(20));
    inner.tick(0.02);
    let tx_r1 = serial.lock().unwrap().take_tx();
    assert!(
        !parse_feetech_sync_write_goal_steps(&tx_r1).is_empty(),
        "B1: legacy Feetech holds MUST continue on pending release 1 (not starved)"
    );
    assert_eq!(
        spec_count_feetech_torque_off_writes(&tx_r1),
        0,
        "B1: zero disable writes while K=1 < 3"
    );
    assert!(inner.disable_pending);
    assert_eq!(inner.torque_intent, TorqueIntent::On);

    // Release 2 (+20 ms, sample 2 of 3): disable STILL pending AND holds CONTINUE!
    clock.advance(Duration::from_millis(20));
    inner.tick(0.02);
    let tx_r2 = serial.lock().unwrap().take_tx();
    assert!(
        !parse_feetech_sync_write_goal_steps(&tx_r2).is_empty(),
        "B1: legacy Feetech holds MUST continue on pending release 2 (not starved)"
    );
    assert_eq!(
        spec_count_feetech_torque_off_writes(&tx_r2),
        0,
        "B1: zero disable writes while K=2 < 3"
    );
    assert!(inner.disable_pending);
    assert_eq!(inner.torque_intent, TorqueIntent::On);

    // Release 3 (+20 ms, sample 3 of 3 spanning 40 ms >= 20 ms): K=3 confirmed -> TORQUE_ENABLE=0 written!
    clock.advance(Duration::from_millis(20));
    inner.tick(0.02);
    let tx_r3 = serial.lock().unwrap().take_tx();
    assert_eq!(
        spec_count_feetech_torque_off_writes(&tx_r3),
        6,
        "B1: all 6 Feetech servos must receive TORQUE_ENABLE=0 once K=3 confirms"
    );
    assert!(!inner.disable_pending);
    assert_eq!(inner.torque_intent, TorqueIntent::Off);
    assert!(!inner.is_torqued);

    // Release 4 (+20 ms): holds MUST stop after accepted disable.
    clock.advance(Duration::from_millis(20));
    inner.tick(0.02);
    let tx_r4 = serial.lock().unwrap().take_tx();
    assert!(
        parse_feetech_sync_write_goal_steps(&tx_r4).is_empty(),
        "B1: Feetech holds must stop after accepted disable"
    );
}

/// R12 B2: `MitOwner::Lost` must NEVER mask out held joints:
/// 1. When J4 (configured as MIT) has verified `PosVel` mode, it falls back to `0x104`
///    `PosVel` hold at its captured position (0 `0x004` MIT frames).
/// 2. When J4 is still in `Mit` mode (no verified non-MIT hold mode), it stays unheld ONLY
///    under L2's 200 ms `RID_CTRL_MODE` readback bound and latches a non-recoverable
///    `HardwareFault` at 200 ms while all other joints stay held.
/// 3. If J4 reports `PosVel` (`CTRL_MODE = 2`) on L2's `RID_CTRL_MODE` readback before 200 ms,
///    it immediately resumes verified `0x104` `PosVel` holds at its preserved captured target
///    without faulting.
#[test]
fn r12_b2_mit_lost_falls_back_to_verified_posvel_hold_or_faults_under_l2_bound() {
    use armd::driver::{JointHoldStatus, JointMode, JointTarget};
    use armd::mit::MitOwner;

    let make_rig = |j4_in_mit_mode: bool| {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&limits(), clock.clone());
        let mut cfg = ArmConfig {
            lease_timeout_ms: 600_000,
            ..ArmConfig::rebot_dm()
        };
        let mut jt = armd::tracking::JointTrackingConfig::rebot_dm();
        jt.enabled = false;
        jt.joints[3].strategy = armd::tracking::TrackingStrategy::Mit;
        jt.joints[3].kp = 10.0;
        jt.joints[3].kd = 0.5;
        jt.joints[3].effort_limit_nm = 3.0;
        jt.joints[3].gravity_scale = 0.0;
        jt.joints[3].hold_kp = Some(10.0);
        jt.joints[3].hold_kd = Some(0.5);
        cfg.joint_tracking = Some(jt);

        let mut dm = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(limits())
            .unwrap();
        let hold_pose = [0.10_f32, 0.20, -0.15, 0.30, -0.20, 0.05, 0.0];
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.q = hold_pose[i];
            }
            s.push_all_feedback();
        });
        dm.read_joint_states().unwrap();
        dm.check_mapping_ranges();
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(dm));
        inner.lease_enabled = false;
        inner.tick(0.02);
        enable_torque_admitted(&mut inner).unwrap();
        if j4_in_mit_mode {
            // Switch J4 into MIT mode with verified mode evidence.
            let mut drv = inner.driver.as_ref().unwrap().lock().unwrap();
            let modes = [
                JointMode::Position,
                JointMode::Position,
                JointMode::Position,
                JointMode::Mit,
                JointMode::Position,
                JointMode::Position,
                JointMode::Position,
            ];
            let targets: Vec<JointTarget> = hold_pose
                .iter()
                .map(|&p| JointTarget {
                    position: p,
                    velocity: Some(0.0),
                    kp: Some(10.0),
                    kd: Some(0.5),
                    torque: Some(0.0),
                })
                .collect();
            drv.set_joint_modes(&modes, &targets).unwrap();
        }
        // Capture initial stopped hold targets before MIT loss so J4 has a captured target.
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        (inner, bus, clock, hold_pose)
    };

    // Case 1: J4 verified in `PosVel` + `MitOwner::Lost` -> falls back directly to `0x104`.
    {
        let (mut inner, bus, clock, hold_pose) = make_rig(false);
        inner.mit.owner = MitOwner::Lost;
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        let tx = bus.tx_log();
        assert!(tx.iter().all(|f| f.id != 0x004), "Case 1: zero MIT frames on J4: {tx:?}");
        let j4_frame = tx
            .iter()
            .find(|f| f.id == 0x104)
            .expect("Case 1: J4 must hold in verified PosVel (0x104) under MitOwner::Lost");
        assert!((decode_damiao_pos_vel_pos(j4_frame) - hold_pose[3]).abs() < 2e-3);
        assert_eq!(inner.joint_hold_status()[3], JointHoldStatus::Held);
    }

    // Case 2: J4 still in `Mit` mode + `MitOwner::Lost` -> unheld under L2 200 ms bound -> HardwareFault.
    {
        let (mut inner, bus, clock, hold_pose) = make_rig(true);
        inner.mit.owner = MitOwner::Lost;
        // Tick 1 under MIT loss: J4 stays unheld (0x004 & 0x104 absent), L2 probes RID_CTRL_MODE.
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);
        let tx1 = bus.tx_log();
        assert!(
            tx1.iter().all(|f| f.id != 0x004 && f.id != 0x104),
            "Case 2: J4 in Mit mode under MitOwner::Lost must stay unheld until non-MIT mode verified: {tx1:?}"
        );
        assert!(
            tx1.iter().any(|f| f.id == 0x7FF && f.data[..4] == [4, 0, 0x33, 10]),
            "Case 2: L2 must probe RID_CTRL_MODE for unverified J4: {tx1:?}"
        );
        assert_eq!(inner.joint_hold_status()[3], JointHoldStatus::UnheldNoVerifiedMode);
        assert!(
            !inner.safety.has_latched_fault_code(FaultCode::HardwareFault),
            "Case 2: L2 must not fault before 200 ms bound"
        );
        // Advance past 200 ms with J4 still reporting CTRL_MODE=1 (Mit): L2 latches non-recoverable HardwareFault.
        for _ in 0..10 {
            clock.advance(Duration::from_millis(25));
            bus.with(|s| s.push_all_feedback());
            bus.clear_log();
            inner.tick(0.02);
            let tx = bus.tx_log();
            assert!(tx.iter().all(|f| f.id != 0x004 && f.id != 0x104));
            for j in [0usize, 1, 2, 4, 5, 6] {
                let id = 0x101u32 + j as u32;
                let f = tx.iter().find(|f| f.id == id).expect("other joints remain held");
                assert!((decode_damiao_pos_vel_pos(f) - hold_pose[j]).abs() < 2e-3);
            }
        }
        assert!(
            inner.safety.has_latched_fault_code(FaultCode::HardwareFault),
            "Case 2: L2 must raise HardwareFault at 200 ms when J4 has no verified non-MIT hold mode"
        );
        let fault = inner.safety.fault().expect("fault latched");
        assert!(!fault.recoverable, "Case 2: L2 unverified-mode fault must be non-recoverable");
    }

    // Case 3: J4 in `Mit` mode + `MitOwner::Lost`, then J4 reports `CTRL_MODE=2 (PosVel)` before 200 ms ->
    // falls back immediately to verified `0x104` hold at its preserved captured target without L2 fault!
    {
        let (mut inner, bus, clock, hold_pose) = make_rig(true);
        inner.mit.owner = MitOwner::Lost;
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
        assert_eq!(inner.joint_hold_status()[3], JointHoldStatus::UnheldNoVerifiedMode);

        // Motor switches to PosVel (CTRL_MODE=2); next L2 RID_CTRL_MODE readback verifies it.
        bus.with(|s| {
            s.motors[3].set_reg_u32(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL);
            s.push_all_feedback();
        });
        clock.advance(Duration::from_millis(20));
        inner.tick(0.02); // sends RID_CTRL_MODE readback & queues reply (or receives it next tick)
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        bus.clear_log();
        inner.tick(0.02);

        let tx = bus.tx_log();
        assert!(tx.iter().all(|f| f.id != 0x004), "Case 3: zero MIT frames on J4: {tx:?}");
        let j4_frame = tx
            .iter()
            .find(|f| f.id == 0x104)
            .expect("Case 3: once PosVel is verified via L2 readback, J4 must resume 0x104 hold");
        assert!(
            (decode_damiao_pos_vel_pos(j4_frame) - hold_pose[3]).abs() < 2e-3,
            "Case 3: J4 must hold its preserved captured target {}",
            hold_pose[3]
        );
        assert_eq!(inner.joint_hold_status()[3], JointHoldStatus::Held);
        assert!(
            !inner.safety.has_latched_fault_code(FaultCode::HardwareFault),
            "Case 3: no L2 fault when PosVel verifies within 200 ms"
        );
    }
}

/// R12 B3: Partial disable dispatch (expiry right after J1):
/// 1. Once J1 receives `CMD_DISABLE` for an accepted request and the 1 s lifetime expires before
///    J2, the disable is committed immediately: `torque_intent == TorqueIntent::Off`, per-motor
///    dispatch progress (`disable_dispatched_mask == 0b0000001`) is recorded, and NO hold frame
///    is ever sent to J1 (neither through supervisor ticks nor direct `write_measured_hold_masked`).
/// 2. Subsequent releases retry the remaining undispatched motors (J2..J7) to completion on BOTH
///    RT executor (`service_background_disable`) and legacy (`service_legacy_pending_disable`)
///    owners without ever reverting `torque_intent` to `On`.
#[test]
fn r12_b3_partial_disable_commits_intent_off_retries_remaining_and_skips_holds_on_disabled_motors() {
    use armd::driver::{JointHoldStatus, JointTarget};
    use armd::transport::CanFrame;
    use armd::TorqueIntent;
    use std::sync::atomic::{AtomicBool, Ordering as AtomicOrdering};

    // Part 1: RT executor owner — mid-batch expiry right after J1.
    {
        let mut r = ExecRig::new();
        r.arm_now();
        r.fx.core.apply_stop_policy(StopPolicy::EStopLatch);
        for _ in 0..4 {
            r.bus.with(|s| {
                s.rx.clear();
                s.push_all_feedback();
            });
            r.step();
        }
        let tripped = Arc::new(AtomicBool::new(false));
        let tripped_cb = tripped.clone();
        let clock_cb = r.clock.clone();
        r.bus.with(|s| {
            // Advance clock past the 1 s request lifetime immediately when J1 transmits CMD_DISABLE.
            // `on_disable` fires right before J1's reply is processed; use `on_disable_state` to
            // advance AFTER J1's transmit succeeded (wait: `on_disable` fires during J1's `transmit`
            // AFTER `send()` already passed J1's pre-TX 1 s check, so J1 succeeds and J2 hits expiry!).
            s.on_disable = Some(Arc::new(move || {
                if !tripped_cb.swap(true, AtomicOrdering::SeqCst) {
                    clock_cb.advance(Duration::from_millis(1_050));
                }
            }));
        });
        r.bus.clear_log();
        r.submit_transition(TransitionOp::TorqueOff);
        let step1 = r.step();
        assert!(
            matches!(step1.transition_ended, Some((_, Outcome::Failed(_)))),
            "R12 B3 (RT): mid-batch expiry after J1 must report transition failure on initial pass: {step1:?}"
        );
        assert!(tripped.load(AtomicOrdering::SeqCst), "expiry hook must have fired after J1");
        assert_eq!(
            r.fx.core.torque_intent,
            TorqueIntent::Off,
            "R12 B3 (RT): partial dispatch after J1 must commit torque_intent=Off"
        );
        assert!(
            r.fx.core.disable_pending,
            "R12 B3 (RT): disable_pending must stay true to retry remaining motors"
        );
        {
            let drv = r.fx.core.driver.as_ref().unwrap().lock().unwrap();
            assert!(drv.disable_frames_dispatched());
            assert_eq!(
                drv.disable_dispatched_mask(),
                0b0000_0001,
                "R12 B3 (RT): only J1 (bit 0) should be recorded dispatched after expiry"
            );
        }
        let tx_initial = r.bus.tx_log();
        assert!(tx_initial.iter().any(|f| f.id == 1 && f.data == damiao::CMD_DISABLE));
        assert!(
            (2u32..=7).all(|id| !tx_initial.iter().any(|f| f.id == id && f.data == damiao::CMD_DISABLE)),
            "R12 B3 (RT): J2..J7 must NOT have been dispatched after 1s expiry: {tx_initial:?}"
        );
        // Direct driver hold call must skip already-disabled J1.
        {
            r.bus.clear_log();
            let mut drv = r.fx.core.driver.as_ref().unwrap().lock().unwrap();
            let targets = vec![
                JointTarget {
                    position: 0.0,
                    velocity: Some(0.0),
                    kp: None,
                    kd: None,
                    torque: Some(0.0),
                };
                7
            ];
            let report = drv.write_measured_hold_masked(0b111_1111, &targets).unwrap();
            assert_eq!(report.joint_status[0], JointHoldStatus::SkippedOff);
            assert!(r.bus.tx_log().iter().all(|f| f.id != 0x101 && f.id != 0x001));
        }
        // Subsequent RT releases retry J2..J7 to completion with zero holds on J1.
        let mut all_retry_tx: Vec<CanFrame> = Vec::new();
        for _ in 0..16 {
            r.bus.with(|s| {
                s.rx.clear();
                s.push_all_feedback();
            });
            r.bus.clear_log();
            r.step();
            let step_tx = r.bus.tx_log();
            assert!(
                step_tx.iter().all(|f| f.id != 0x101),
                "R12 B3 (RT): zero hold frames allowed on already-disabled J1 during retry: {step_tx:?}"
            );
            all_retry_tx.extend(step_tx);
            assert_eq!(r.fx.core.torque_intent, TorqueIntent::Off);
            if !r.fx.core.disable_pending && !r.fx.core.is_torqued {
                break;
            }
        }
        assert!(
            !r.fx.core.disable_pending && !r.fx.core.is_torqued,
            "R12 B3 (RT): remaining motors J2..J7 must complete disable through retry path"
        );
        for id in 2u32..=7 {
            assert!(
                all_retry_tx.iter().any(|f| f.id == id && f.data == damiao::CMD_DISABLE),
                "R12 B3 (RT): remaining motor J{id} must receive CMD_DISABLE on retry: {all_retry_tx:?}"
            );
        }
        let drv = r.fx.core.driver.as_ref().unwrap().lock().unwrap();
        assert_eq!(drv.disable_dispatched_mask(), 0);
    }

    // Part 2: Legacy owner — mid-batch expiry right after J1.
    {
        let (mut inner, bus, clock) = make_driver_rig();
        bus.with(|s| {
            s.follow_commands = false;
            s.immediate_replies = true;
            for m in s.motors.iter_mut() {
                m.q = 0.0;
            }
        });
        enable_torque_admitted(&mut inner).unwrap();
        inner.apply_stop_policy(StopPolicy::EStopLatch);
        for _ in 0..4 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| {
                s.rx.clear();
                s.push_all_feedback();
            });
            inner.tick(0.02);
        }
        let tripped = Arc::new(AtomicBool::new(false));
        let tripped_cb = tripped.clone();
        let clock_cb = clock.clone();
        bus.with(|s| {
            s.on_disable = Some(Arc::new(move || {
                if !tripped_cb.swap(true, AtomicOrdering::SeqCst) {
                    clock_cb.advance(Duration::from_millis(1_050));
                }
            }));
        });
        bus.clear_log();
        let res = inner.disable_torque();
        assert!(
            res.is_err(),
            "R12 B3 (legacy): mid-batch expiry after J1 must report error on initial pass"
        );
        assert!(tripped.load(AtomicOrdering::SeqCst));
        assert_eq!(inner.torque_intent, TorqueIntent::Off);
        assert!(inner.disable_pending);
        {
            let drv = inner.driver.as_ref().unwrap().lock().unwrap();
            assert!(drv.disable_frames_dispatched());
            assert_eq!(drv.disable_dispatched_mask(), 0b0000_0001);
        }
        let mut all_retry_tx: Vec<CanFrame> = Vec::new();
        for _ in 0..16 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| {
                s.rx.clear();
                s.push_all_feedback();
            });
            bus.clear_log();
            inner.tick(0.02);
            let step_tx = bus.tx_log();
            assert!(
                step_tx.iter().all(|f| f.id != 0x101),
                "R12 B3 (legacy): zero hold frames allowed on already-disabled J1 during retry: {step_tx:?}"
            );
            all_retry_tx.extend(step_tx);
            assert_eq!(inner.torque_intent, TorqueIntent::Off);
            if !inner.disable_pending && !inner.is_torqued {
                break;
            }
        }
        assert!(
            !inner.disable_pending && !inner.is_torqued,
            "R12 B3 (legacy): remaining motors J2..J7 must complete disable through retry path"
        );
        for id in 2u32..=7 {
            assert!(
                all_retry_tx.iter().any(|f| f.id == id && f.data == damiao::CMD_DISABLE),
                "R12 B3 (legacy): remaining motor J{id} must receive CMD_DISABLE on retry: {all_retry_tx:?}"
            );
        }
        let drv = inner.driver.as_ref().unwrap().lock().unwrap();
        assert_eq!(drv.disable_dispatched_mask(), 0);
    }
}

/// R12 B4: Thermal retry on the RT owner under driver mutex contention:
/// 1. First thermal trip fires while `self.driver` is contended (`WouldBlock`):
///    latches `ThermalTrip`, arms bounded per-release `DisableProtective { Thermal }` retry,
///    and sends 0 `CMD_DISABLE` frames on the contended attempt.
/// 2. On the VERY NEXT uncontended release (even within < 25 ms, EVERY release), the protective
///    thermal cut retries and dispatches `CMD_DISABLE` to all 7 motors even at a raised pose
///    (and the existing `ThermalTrip` latch does NOT suppress redispatch or bump epoch again).
/// 3. Even if `ThermalTrip` was already latched while `is_torqued == true` with no retry armed,
///    `check_thermal_protection` redispatches `DisableProtective { Thermal }` on the next release.
#[test]
fn r12_b4_rt_thermal_cut_retries_every_release_after_first_contention() {
    use armd::safety::FaultKind;
    use armd::TorqueIntent;

    // Part 1: Contended first RT thermal cut arms bounded retry and dispatches on the very next
    // uncontended release (at raised pose, with < 25 ms spacing).
    {
        let mut r = ExecRig::new();
        r.fx.core.rt_executor_owned = true;
        r.arm_now();
        // Move arm to raised pose (away from rest) and set J2 over max_temperature.
        let overtemp = r.fx.core.config.max_temperature + 10.0;
        let overtemp_u8 = overtemp.ceil() as u8;
        r.bus.with(|s| {
            s.follow_commands = false;
            for m in s.motors.iter_mut() {
                m.q = 0.60;
            }
            s.motors[1].temperature_c = 35;
            s.push_all_feedback();
        });
        r.step();
        assert!(!r.fx.core.safety.has_latched_fault_kind(FaultKind::ThermalTrip));

        // Now raise temperature above limit and contend the driver mutex during the first thermal cut!
        r.fx.core.temperatures[1] = overtemp;
        r.bus.clear_log();
        r.fx.core.with_contended_driver_for_test(|core| {
            core.apply_stop_policy(StopPolicy::ThermalTrip);
        });
        assert!(
            r.fx.core.safety.has_latched_fault_kind(FaultKind::ThermalTrip),
            "B4: ThermalTrip must latch on the first overtemp attempt"
        );
        let epoch_after_first = r.fx.core.safety.epoch_id();
        assert!(
            r.fx.core.disable_pending,
            "B4: disable_pending must stay true after contended thermal cut"
        );
        assert_eq!(
            r.bus
                .tx_log()
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count(),
            0,
            "B4: contended attempt cannot transmit CMD_DISABLE yet"
        );

        // Next release is only 10 ms later (< 25 ms normal retry interval) and STILL contended:
        // must keep retry armed without blocking or losing retry state.
        r.clock.advance(Duration::from_millis(10));
        r.bus.with(|s| {
            s.motors[1].temperature_c = overtemp_u8;
            s.push_all_feedback();
        });
        r.bus.clear_log();
        let t_wall = Instant::now();
        {
            let driver_ref = r.fx.core.driver.as_ref().unwrap();
            let addr =
                (driver_ref as *const std::sync::Mutex<Box<dyn ArmDriver>>) as usize;
            let (locked_tx, locked_rx) = std::sync::mpsc::sync_channel::<()>(1);
            let (release_tx, release_rx) = std::sync::mpsc::sync_channel::<()>(1);
            let holder = std::thread::spawn(move || {
                let mutex =
                    unsafe { &*(addr as *const std::sync::Mutex<Box<dyn ArmDriver>>) };
                let _guard = mutex.lock().unwrap();
                let _ = locked_tx.send(());
                let _ = release_rx.recv_timeout(Duration::from_secs(5));
            });
            locked_rx.recv_timeout(Duration::from_secs(2)).unwrap();
            let _ = r.fx.step();
            let _ = release_tx.send(());
            let _ = holder.join();
        }
        assert!(
            t_wall.elapsed() < Duration::from_millis(500),
            "B4: contended RT step must be strictly non-blocking (elapsed={:?})",
            t_wall.elapsed()
        );
        assert_eq!(
            r.bus
                .tx_log()
                .iter()
                .filter(|f| f.data == damiao::CMD_DISABLE)
                .count(),
            0
        );

        // Next release is uncontended (only 10 ms later, proving EVERY-release retry):
        // protective thermal cut MUST dispatch CMD_DISABLE to ALL 7 motors at raised pose!
        r.bus.with(|s| {
            s.motors[1].temperature_c = overtemp_u8;
            s.push_all_feedback();
        });
        r.bus.clear_log();
        let _ = r.fx.step();
        let tx_uncontended = r.bus.tx_log();
        for id in 1u32..=7 {
            assert!(
                tx_uncontended.iter().any(|f| f.id == id && f.data == damiao::CMD_DISABLE),
                "B4: uncontended release immediately after contention must dispatch CMD_DISABLE to J{id}: {tx_uncontended:?}"
            );
        }
        assert_eq!(
            r.fx.core.torque_intent,
            TorqueIntent::Off,
            "B4: protective thermal dispatch commits torque_intent=Off"
        );
        assert!(
            r.fx.core.safety.has_latched_fault_kind(FaultKind::ThermalTrip),
            "B4: ThermalTrip remains latched after protective dispatch"
        );
        let _ = epoch_after_first;

        // Complete Damiao repeat passes to full confirmation.
        for _ in 0..8 {
            r.step_with_feedback();
            if !r.fx.core.disable_pending && !r.fx.core.is_torqued {
                break;
            }
        }
        assert!(!r.fx.core.is_torqued && !r.fx.core.disable_pending);
    }

    // Part 2: An existing `ThermalTrip` latch MUST NOT suppress thermal redispatch when
    // torque is still on and no protective retry is armed (supervisor.rs:4341).
    {
        let mut r = ExecRig::new();
        r.fx.core.rt_executor_owned = true;
        r.arm_now();
        let overtemp = r.fx.core.config.max_temperature + 12.0;
        let overtemp_u8 = overtemp.ceil() as u8;
        r.bus.with(|s| {
            s.follow_commands = false;
            for m in s.motors.iter_mut() {
                m.q = 0.55;
            }
            s.motors[1].temperature_c = 35;
            s.push_all_feedback();
        });
        r.step();

        // Pre-latch ThermalTrip WITHOUT having dispatched the protective disable yet.
        r.fx.core.safety.latch(
            FaultKind::ThermalTrip,
            armd::safety::FaultDetail::ThermalTrip {
                joint_idx: 1,
                temperature: overtemp,
                limit: r.fx.core.config.max_temperature,
            },
        );
        assert!(r.fx.core.is_torqued);
        assert_eq!(r.fx.core.torque_intent, TorqueIntent::On);

        // Feed overtemp sample and run one RT release: existing ThermalTrip latch MUST NOT
        // suppress redispatch of DisableProtective { Thermal }!
        r.bus.with(|s| {
            s.motors[1].temperature_c = overtemp_u8;
            s.push_all_feedback();
        });
        r.bus.clear_log();
        r.step();
        let tx = r.bus.tx_log();
        for id in 1u32..=7 {
            assert!(
                tx.iter().any(|f| f.id == id && f.data == damiao::CMD_DISABLE),
                "B4 Part 2: existing ThermalTrip latch must not suppress redispatch to J{id}: {tx:?}"
            );
        }
        assert_eq!(r.fx.core.torque_intent, TorqueIntent::Off);
    }
}

/// R12 M1: Feetech recheck after RX clear/drain:
/// 1. Any position reply already queued in RX BEFORE the clear is decoded and fed into
///    `rest_pose_window` first (never discarded): a queued raised position reply causes immediate
///    `TorqueOffWouldDrop` with 0 `TORQUE_ENABLE=0` writes even when prior K=3 was at rest.
/// 2. If the rest-pose window ages out (> 100 ms) during the RX drain while still inside the 1 s
///    request lifetime, `begin_disable_torque` DEFERS (`Ok(TorqueProgress::Pending)`) instead of
///    returning `TorqueOffWouldDrop`, and completes once fresh K=3 rest replies arrive within 1 s.
#[test]
fn r12_m1_feetech_disable_decodes_queued_rx_before_clear_and_defers_if_window_aged() {
    use armd::driver::{ArmDriverError, DisableKind, FeetechArmDriver, StoppedDisableGate, TorqueProgress};
    use armd::transport::MockTransport;

    #[derive(Clone)]
    struct DrainHookSerial {
        inner: Arc<std::sync::Mutex<MockTransport>>,
        on_clear: Arc<std::sync::Mutex<Option<Box<dyn FnMut() + Send>>>>,
    }
    impl armd::transport::Transport for DrainHookSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, armd::transport::TransportError> {
            self.inner.lock().unwrap().write(buf)
        }
        fn write_all(&mut self, buf: &[u8]) -> Result<(), armd::transport::TransportError> {
            self.inner.lock().unwrap().write_all(buf)
        }
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, armd::transport::TransportError> {
            self.inner.lock().unwrap().read(buf)
        }
        fn flush(&mut self) -> Result<(), armd::transport::TransportError> {
            self.inner.lock().unwrap().flush()
        }
        fn clear_input_buffer(&mut self) {
            self.inner.lock().unwrap().clear_input_buffer();
            if let Some(ref mut hook) = *self.on_clear.lock().unwrap() {
                hook();
            }
        }
        fn clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
            armd::transport::Transport::clock(&*self.inner.lock().unwrap())
        }
    }

    let servo_ids: Vec<u8> = (1..=6).collect();
    let cfg = feetech_6dof_config();
    let sync_pos_req_1 = armd::protocol::feetech::build_sync_read(
        &servo_ids[..3],
        armd::protocol::feetech::REG_PRESENT_POSITION_L,
        8,
    )
    .unwrap();
    let sync_pos_req_2 = armd::protocol::feetech::build_sync_read(
        &servo_ids[3..],
        armd::protocol::feetech::REG_PRESENT_POSITION_L,
        8,
    )
    .unwrap();
    let make_gate = || StoppedDisableGate {
        safety_stopped: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        has_resting_pose: true,
        arm_joints: {
            let mut a = [0usize; armd::rt_core::mailbox::MAX_DOF];
            for i in 0..5 {
                a[i] = i;
            }
            a
        },
        arm_joints_len: 5,
        rest_positions: [0.0; armd::rt_core::mailbox::MAX_DOF],
        rest_pose_tolerance_rad: cfg.rest_pose_tolerance_rad,
    };
    let rest_steps: [u16; 6] = [2048; 6];

    // Part 1: Queued raised position reply sitting in RX before disable MUST be decoded into
    // `rest_pose_window` (never discarded by clear), refusing with `TorqueOffWouldDrop` and 0 writes.
    {
        let clock = MockClock::new();
        let serial_inner = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let on_clear: Arc<std::sync::Mutex<Option<Box<dyn FnMut() + Send>>>> =
            Arc::new(std::sync::Mutex::new(None));
        let mut drv = FeetechArmDriver::new(
            DrainHookSerial {
                inner: Arc::clone(&serial_inner),
                on_clear: Arc::clone(&on_clear),
            },
            servo_ids.clone(),
        )
        .with_calibrated(true)
        .with_tx_echo(Some(false))
        .with_gripper_index(Some(5));
        drv.set_stopped_disable_gate(make_gate());

        // Establish K=3 confirmed at rest.
        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(12));
            }
            {
                let mut ser = serial_inner.lock().unwrap();
                ser.stage_reply(&sync_pos_req_1, &feetech_feedback_bytes(&servo_ids[..3], &rest_steps[..3]));
                ser.stage_reply(&sync_pos_req_2, &feetech_feedback_bytes(&servo_ids[3..], &rest_steps[3..]));
            }
            drv.read_joint_states().unwrap();
        }
        // Now queue a fresh RAISED position reply for servo 1 (2600 steps >> rest 2048) directly in RX!
        clock.advance(Duration::from_millis(5));
        {
            let mut ser = serial_inner.lock().unwrap();
            let _ = ser.take_tx();
            ser.provide_rx(&feetech_feedback_bytes(&[1], &[2600]));
        }
        let started_at = drv.clock_now();
        let res = drv.begin_disable_torque(DisableKind::DisableRequested { started_at });
        assert!(
            matches!(res, Err(ArmDriverError::TorqueOffWouldDrop)),
            "M1 Part 1: queued raised position reply must be decoded before clear and refuse with TorqueOffWouldDrop, got {res:?}"
        );
        let tx = serial_inner.lock().unwrap().take_tx();
        assert_eq!(
            spec_count_feetech_torque_off_writes(&tx),
            0,
            "M1 Part 1: ZERO TORQUE_ENABLE=0 writes when queued raised reply decoded"
        );
    }

    // Part 2: Rest-pose window ages out (> 100 ms) DURING the RX drain while still inside the 1 s
    // request lifetime -> MUST DEFER (`Ok(TorqueProgress::Pending)`) instead of `TorqueOffWouldDrop`,
    // and succeed once fresh K=3 rest replies arrive within 1 s!
    {
        let clock = MockClock::new();
        let serial_inner = Arc::new(std::sync::Mutex::new(
            MockTransport::new().with_clock(clock.clone()),
        ));
        let on_clear: Arc<std::sync::Mutex<Option<Box<dyn FnMut() + Send>>>> =
            Arc::new(std::sync::Mutex::new(None));
        let mut drv = FeetechArmDriver::new(
            DrainHookSerial {
                inner: Arc::clone(&serial_inner),
                on_clear: Arc::clone(&on_clear),
            },
            servo_ids.clone(),
        )
        .with_calibrated(true)
        .with_tx_echo(Some(false))
        .with_gripper_index(Some(5));
        drv.set_stopped_disable_gate(make_gate());

        for step in 0..3 {
            if step > 0 {
                clock.advance(Duration::from_millis(12));
            }
            {
                let mut ser = serial_inner.lock().unwrap();
                ser.stage_reply(&sync_pos_req_1, &feetech_feedback_bytes(&servo_ids[..3], &rest_steps[..3]));
                ser.stage_reply(&sync_pos_req_2, &feetech_feedback_bytes(&servo_ids[3..], &rest_steps[3..]));
            }
            drv.read_joint_states().unwrap();
        }
        let _ = serial_inner.lock().unwrap().take_tx();

        // Advance clock by 120 ms (> 100 ms feedback freshness, < 1000 ms request lifetime) ONCE
        // during `clear_input_buffer()` so the window ages out mid-drain!
        let clock_hook = clock.clone();
        let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let fired_hook = Arc::clone(&fired);
        *on_clear.lock().unwrap() = Some(Box::new(move || {
            if !fired_hook.swap(true, Ordering::SeqCst) {
                clock_hook.advance(Duration::from_millis(120));
            }
        }));

        let started_at = drv.clock_now();
        let res = drv.begin_disable_torque(DisableKind::DisableRequested { started_at });
        assert!(
            matches!(res, Ok(TorqueProgress::Pending { .. })),
            "M1 Part 2: window aging out during drain within 1 s lifetime MUST defer (Pending), NOT TorqueOffWouldDrop, got {res:?}"
        );
        assert!(fired.load(Ordering::SeqCst), "M1 Part 2: drain hook must have fired");
        assert!(drv.torque_transition_pending(), "M1 Part 2: pending_disable must stay armed");
        assert_eq!(
            spec_count_feetech_torque_off_writes(&serial_inner.lock().unwrap().take_tx()),
            0,
            "M1 Part 2: zero writes while deferred"
        );

        // Now deliver 3 fresh spaced rest polls within the remaining 1 s lifetime -> completes!
        let mut done = false;
        for _ in 0..4 {
            clock.advance(Duration::from_millis(15));
            {
                let mut ser = serial_inner.lock().unwrap();
                ser.stage_reply(&sync_pos_req_1, &feetech_feedback_bytes(&servo_ids[..3], &rest_steps[..3]));
                ser.stage_reply(&sync_pos_req_2, &feetech_feedback_bytes(&servo_ids[3..], &rest_steps[3..]));
            }
            let deadline = drv.clock_now() + Duration::from_millis(8);
            match drv.poll_torque(deadline).unwrap() {
                TorqueProgress::Done(_) => {
                    done = true;
                    break;
                }
                TorqueProgress::Pending { .. } => {}
            }
        }
        assert!(done, "M1 Part 2: deferred Feetech disable must complete once K=3 re-confirms within 1 s");
        let tx = serial_inner.lock().unwrap().take_tx();
        assert_eq!(
            spec_count_feetech_torque_off_writes(&tx),
            6,
            "M1 Part 2: all 6 servos must receive TORQUE_ENABLE=0 once K=3 confirms"
        );
    }
}
