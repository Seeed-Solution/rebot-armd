//! Track D: servo, teleop and policy targets through the latest-target mailbox, stepped one
//! release at a time against the supervisor's production adapter (`ReleaseFixture`, mock
//! follower). Each test asserts the target the core actually commands (`servo_target`,
//! `target_positions`) and the protocol reply.

use std::time::Duration;

use serde_json::{json, Value};

use super::targets::*;
use crate::model::ArmConfig;
use crate::rt_core::broker::{Authority, BootId, Broker, Capacities};
use crate::rt_core::ids::ArmId;
use crate::rt_core::mailbox::{Refusal, RefusalDetail, TargetFrame, TargetOutcome, MAX_DOF};
use crate::rt_core::ops::{PlanSlot, ReleaseFixture};
use crate::rt_core::urgent::StopRequests;
use crate::supervisor::{ArmError, SupervisorInner};

const CONN: &str = "conn-track-d";
const BOOT: BootId = BootId(7);

struct Rig {
    fx: ReleaseFixture<SupervisorInner>,
    _broker: Broker<PlanSlot>,
    port: TargetPort,
    stop: StopRequests,
    stream: ServoStream,
    gen: u64,
}

fn rig() -> Rig {
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    let gen = inner.acquire_lease(CONN).expect("lease");
    // the connection got the lease: its servo stream is the one consumed
    inner.authorize_servo_stream(gen);
    let (fx, broker, tw, stop) = ReleaseFixture::new(inner, BOOT, Capacities::default());
    let mut stream = ServoStream::new();
    stream.lease_granted(gen);
    Rig { fx, _broker: broker, port: TargetPort::new(tw, ArmId::LOCAL, BOOT), stop, stream, gen }
}

impl Rig {
    fn epoch(&self) -> u64 { self.fx.core.safety.epoch_id() }

    fn authority(&self) -> Authority {
        Authority { lease_generation: self.gen, epoch: self.epoch(), stop_generation: self.stop.current(), ..Authority::default() }
    }

    /// Commanded pose with joint 1 offset by `dq`.
    fn pose(&self, dq: f32) -> Vec<f32> {
        let mut q = self.fx.core.commanded_positions.clone();
        q[0] += dq;
        q
    }

    fn servo(&mut self, seq: Option<u64>, dq: f32) -> Option<Value> {
        let q = self.pose(dq);
        let msg = ServoMessage { sequence_id: seq, positions: &q, authority: self.authority(), received_at: self.fx.now(), calibration_busy: false };
        self.port.submit_servo(&mut self.stream, &msg)
    }

    fn reply_ctx(&self) -> ReplyCtx {
        ReplyCtx { epoch_id: self.epoch(), safety_state: json!(self.fx.core.safety.state()), lease_generation: self.gen }
    }

    /// One release: the consumed target's refusal and the WS reply built from the outcome
    /// the stream received through the outcome ring.
    fn step_reply(&mut self) -> (Option<Refusal>, Option<Value>) {
        let rep = self.fx.step();
        let mut outs = Vec::new();
        self.port.outcomes(|o| outs.push(o));
        // earlier plain `step`s left their outcomes too: this release's is the last
        assert_eq!(outs.is_empty(), rep.target.is_none(), "an outcome per consumed frame");
        if let (Some(o), Some((seq, _))) = (outs.last(), rep.target) {
            assert_eq!((o.seq, o.result.err()), (seq, rep.target_refusal));
        }
        let reply = rep.target_refusal.as_ref().and_then(|r| servo_outcome_reply(r, &self.reply_ctx()));
        (rep.target_refusal, reply)
    }

    /// A frame of `stream` with this rig's identity and authority.
    fn raw_frame(&self, stream: crate::rt_core::mailbox::StreamId, authority: Authority, dq: f32) -> TargetFrame {
        let mut q = [0.0; MAX_DOF];
        let pose = self.pose(dq);
        q[..pose.len()].copy_from_slice(&pose);
        TargetFrame { arm: ArmId::LOCAL, boot: BOOT, stream, seq: 1, authority, policy_generation: 0, input_at: self.fx.now(),
            vision_at: None, vision_frame_id: None, deadline: self.fx.now() + Duration::from_secs(1), dof: pose.len() as u8, q, replaceable: true }
    }

    fn consume(&mut self, f: &TargetFrame) -> Result<(), &'static str> {
        let ctx = self.fx.ctx(self.fx.now());
        self.fx.core.consume_target_frame(f, &ctx).map_err(|r| r.code)
    }

    /// The servo target joint 1 (None: no target).
    fn target0(&self) -> Option<f32> { self.fx.core.servo_target.as_ref().map(|t| t[0]) }
}

fn close(a: Option<f32>, b: f32) -> bool { a.is_some_and(|a| (a - b).abs() < 1e-6) }

#[test]
fn a_servo_burst_coalesces_to_the_newest_frame_and_is_applied_once() {
    let mut r = rig();
    let q0 = r.fx.core.commanded_positions[0];
    for i in 1..=5u64 {
        assert_eq!(r.servo(Some(i), 0.001 * i as f32), None, "a queued servo frame gets no reply");
    }
    assert_eq!(r.port.coalesced(), 4);
    let rep = r.fx.step();
    assert_eq!(rep.target, Some((5, Ok(()))), "only the newest frame is consumed");
    assert!(close(r.target0(), q0 + 0.005), "commanded target {:?}", r.target0());
    assert!(r.fx.step().target.is_none(), "consumed once");
}

#[test]
fn a_delayed_servo_frame_applies_within_its_ttl_and_is_refused_after() {
    let mut r = rig();
    let q0 = r.fx.core.commanded_positions[0];
    // consumed 150 ms after it was received: applied unchanged
    assert_eq!(r.servo(Some(1), 0.002), None);
    r.fx.clock().advance(Duration::from_millis(150));
    assert_eq!(r.fx.step().target, Some((1, Ok(()))));
    assert!(close(r.target0(), q0 + 0.002));
    // consumed after the servo motion-frame timeout: refused, the previous target stays
    assert_eq!(r.servo(Some(2), 0.004), None);
    r.fx.clock().advance(SERVO_TARGET_TTL + Duration::from_millis(1));
    let (refusal, reply) = r.step_reply();
    assert_eq!(refusal.map(|f| f.code), Some("DEADLINE_EXPIRED"));
    assert_eq!(reply.unwrap()["code"], json!("DEADLINE_EXPIRED"));
    assert!(close(r.target0(), q0 + 0.002));
}

#[test]
fn frames_of_a_stream_that_is_not_authorized_are_refused_and_the_stream_hears_lease_lost() {
    let mut r = rig();
    r.fx.core.core.revoke_stream();
    assert_eq!(r.servo(Some(1), 0.002), None);
    let (refusal, reply) = r.step_reply();
    assert_eq!(refusal.map(|f| f.code), Some("STREAM_NOT_AUTHORIZED"));
    assert_eq!(reply, Some(json!({"type":"lease_lost", "generation":r.gen})));
    assert_eq!(r.target0(), None);
    r.fx.core.authorize_servo_stream(r.gen);
    assert_eq!(r.servo(Some(2), 0.002), None);
    assert_eq!(r.step_reply(), (None, None));
    assert!(r.target0().is_some());
}

#[test]
fn duplicate_and_far_sequences_are_refused_and_a_wrapped_one_is_kept() {
    let mut r = rig();
    let q0 = r.fx.core.commanded_positions[0];
    assert_eq!(r.servo(Some(u64::MAX - 1), 0.001), None);
    assert_eq!(r.servo(Some(u64::MAX - 1), 0.009), Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"})), "duplicate");
    assert_eq!(r.servo(Some(u64::MAX), 0.002), None);
    assert_eq!(r.servo(Some(0), 0.003), None, "wrapped by one");
    assert_eq!(r.servo(Some(2000), 0.009), Some(json!({"type":"servo_reject", "reason":"invalid_sequence_id"})), "jump > 1000");
    assert_eq!(r.servo(None, 0.009), Some(json!({"type":"servo_reject", "reason":"missing_sequence_id"})));
    assert_eq!(r.stream.last_sequence(), Some(0));
    let rep = r.fx.step();
    assert_eq!(rep.target.map(|(_, res)| res), Some(Ok(())));
    assert!(close(r.target0(), q0 + 0.003), "the wrapped frame is the one applied: {:?}", r.target0());
    // a new lease restarts the sequence
    r.stream.lease_granted(r.gen + 1);
    assert_eq!(r.stream.last_sequence(), None);
}

#[test]
fn a_disconnect_with_a_buffered_target_commands_nothing() {
    // the connection closes: its lease is released (and the legacy close stops motion)
    let mut r = rig();
    let before = r.fx.core.target_positions.clone();
    assert_eq!(r.servo(Some(1), 0.004), None);
    assert!(r.fx.core.release_lease(CONN, r.gen));
    r.port.close_servo(r.gen);
    let (refusal, reply) = r.step_reply();
    assert_eq!(refusal, Some(Refusal { code: "LEASE_LOST", detail: RefusalDetail::LeaseLoss { reason: "expired" } }));
    assert_eq!((r.target0(), &r.fx.core.target_positions), (None, &before));
    assert_eq!(reply, Some(json!({"type":"lease_lost", "generation":r.gen, "reason":"expired"})));

    // the same with the stop the close requests: refused as stopped, no reply
    let mut r = rig();
    assert_eq!(r.servo(Some(1), 0.004), None);
    r.fx.core.release_lease(CONN, r.gen);
    r.stop.request();
    let rep = r.fx.step();
    assert!(rep.stopped);
    assert_eq!(rep.target, Some((1, Err("STOPPED"))));
    assert_eq!(r.target0(), None);
    assert_eq!(servo_outcome_reply(&rep.target_refusal.unwrap(), &r.reply_ctx()), None);
    let mut outs = Vec::new();
    r.port.outcomes(|o: TargetOutcome| outs.push(o));
    assert_eq!(outs.len(), 1, "the stream hears the refusal");
}

#[test]
fn a_stop_with_a_buffered_target_leaves_the_arm_holding() {
    let mut r = rig();
    assert_eq!(r.servo(Some(1), 0.002), None);
    r.fx.step();
    assert!(r.target0().is_some());
    assert_eq!(r.servo(Some(2), 0.004), None);
    r.stop.request();
    let rep = r.fx.step();
    assert!(rep.stopped, "the stop is applied before the target");
    assert_eq!(rep.target, Some((2, Err("STOPPED"))));
    assert_eq!(r.target0(), None, "stop cleared the servo target and the buffered one was not applied");
    assert_eq!(r.fx.core.target_positions, r.fx.core.commanded_positions);
    // frames received after the stop carry its generation and apply again
    assert_eq!(r.servo(Some(3), 0.001), None);
    assert_eq!(r.fx.step().target, Some((3, Ok(()))));
}

#[test]
fn epoch_lease_expiry_stream_and_calibration_are_rechecked_at_consumption() {
    let mut r = rig();
    // epoch moved on after the frame was received
    assert_eq!(r.servo(Some(1), 0.002), None);
    r.fx.core.safety.trigger_fault(crate::safety::FaultReason::WatchdogTimeout);
    let (refusal, reply) = r.step_reply();
    assert_eq!(refusal.map(|f| f.code), Some("EPOCH_MISMATCH"));
    assert_eq!(reply, Some(json!({"type":"epoch_sync", "epoch_id":r.epoch(), "safety_state":r.fx.core.safety.state()})));
    assert_eq!(r.target0(), None);

    // calibration started after the frame was received
    let mut r = rig();
    assert_eq!(r.servo(Some(2), 0.002), None);
    r.fx.core.calibration_busy = true;
    let (refusal, reply) = r.step_reply();
    assert_eq!(refusal.map(|f| f.code), Some("CALIBRATION_BUSY"));
    assert_eq!(reply, Some(json!({"type":"servo_reject", "reason":"calibration_busy"})));
    r.fx.core.calibration_busy = false;

    // the lease expired while the frame waited (a queued frame does not renew it)
    assert_eq!(r.servo(Some(3), 0.002), None);
    r.fx.core.lease_timeout = Duration::ZERO;
    assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("LEASE_LOST")));
    assert_eq!(r.target0(), None);

    // a frame under the lease but on another stream; another arm or boot
    let mut r = rig();
    let forged = r.raw_frame(servo_stream(r.gen + 1), r.authority(), 0.001);
    assert_eq!(r.consume(&forged), Err("STREAM_NOT_AUTHORIZED"));
    let own = r.raw_frame(servo_stream(r.gen), r.authority(), 0.001);
    assert_eq!(r.consume(&TargetFrame { boot: BootId(8), ..own }), Err("WRONG_TARGET"));
    // a frame without a lease is refused while leases are enforced, also when nobody holds one
    let lease_less = TargetFrame { authority: Authority { lease_generation: 0, ..r.authority() }, ..own };
    assert_eq!(r.consume(&lease_less), Err("LEASE_LOST"));
    r.fx.core.release_lease(CONN, r.gen);
    let lease_less = TargetFrame { authority: Authority { lease_generation: 0, ..r.authority() }, ..own };
    r.fx.core.core.authorize_stream(crate::rt_core::mailbox::TargetAuthority { stream: Some(own.stream), ..Default::default() });
    assert_eq!(r.consume(&lease_less), Err("NO_LEASE"));
    assert_eq!(r.target0(), None);
}

#[test]
fn a_refused_servo_target_gets_the_legacy_reply_fields() {
    let mut r = rig();
    let mut twin = rig();
    // joint 1 far outside its soft limit
    assert_eq!(r.servo(Some(1), 100.0), None);
    let (refusal, reply) = r.step_reply();
    let twin_stamp = twin.fx.core.current_admission_stamp();
    let legacy = crate::api::rest::ApiError::from(ArmError::from(twin.fx.core.servo(twin_stamp, twin.pose(100.0), twin.epoch()).unwrap_err()));
    let reply = reply.unwrap();
    assert_eq!(refusal, Some(Refusal { code: "SOFT_LIMIT_EXCEEDED", detail: RefusalDetail::Limit { joint: Some(0) } }));
    assert_eq!(
        (&reply["type"], &reply["status"], &reply["code"], &reply["error"], &reply["safety_state"], &reply["epoch_id"]),
        (&json!("servo_reject"), &json!(legacy.status.as_u16()), &json!(legacy.code), &json!(legacy.error), &json!(legacy.safety_state), &json!(r.epoch())),
    );
    assert_eq!(r.target0(), None);
    // the same for every error code the legacy servo path maps with fixed fields
    for e in [crate::safety::SafetyError::EStopActive, crate::safety::SafetyError::MotionBusy, crate::safety::SafetyError::HandoverSettling,
        crate::safety::SafetyError::InvalidValue, crate::safety::SafetyError::FaultActive(crate::safety::FaultReason::WatchdogTimeout)] {
        let legacy = crate::api::rest::ApiError::from(ArmError::from(e.clone()));
        let code = legacy.code.clone().unwrap();
        assert_eq!(safety_code(&e), code.as_str());
        let reply = servo_reject(&code, "", 1);
        assert_eq!((&reply["status"], &reply["error"], &reply["safety_state"]),
            (&json!(legacy.status.as_u16()), &json!(legacy.error), &json!(legacy.safety_state)), "{code}");
    }
}

#[cfg(feature = "policy-runtime")]
mod policy {
    use super::*;
    use crate::rt_core::mailbox::Submitted;
    use crate::policy_runtime::{PolicyMode, PolicyRunStart};

    fn begin(r: &mut Rig, mode: PolicyMode) -> u64 {
        let stamp = r.fx.core.current_admission_stamp();
        r.fx.core.begin_policy_run(stamp, PolicyRunStart {
            epoch_id: r.epoch(), connection_id: CONN.into(), lease_generation: r.gen, policy_id: "p".into(),
            artifact_digest: "d".into(), backend: "mlp".into(), mode, sim_only: false, max_first_step_rad: None,
            max_step_rad: None, tag: None, i_am_at_the_arm: false, verified_release_gate: None,
        }).expect("policy run")
    }

    /// Submits a target whose joint inputs were sampled `input_ago` and camera sample `vision_ago` before now.
    fn submit(r: &mut Rig, action_id: u64, dq: f32, input_ago: Duration, vision_ago: Option<Duration>) -> Submitted {
        let now = r.fx.now();
        let (input_at, vision_at) = (now - input_ago, vision_ago.map(|d| now - d));
        let stamp = r.fx.core.policy_target_stamp(action_id, r.stop.current()).expect("stamp");
        let mut q = r.fx.core.measured_positions.clone();
        q[0] += dq;
        r.port.submit_policy(&PolicyTarget { action_id, stamp, targets: &q, input_at, vision_at, vision_frame_id: vision_ago.map(|_| 41) })
    }

    fn run_code(r: &Rig) -> Option<String> { r.fx.core.last_action.as_ref().and_then(|a| a.code.as_deref().map(str::to_owned)) }

    #[test]
    fn a_live_target_is_applied_and_policy_frames_do_not_coalesce() {
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Live);
        let m0 = r.fx.core.measured_positions[0];
        assert_eq!(submit(&mut r, id, 0.01, Duration::ZERO, None), Submitted::Accepted);
        assert_eq!(submit(&mut r, id, 0.02, Duration::ZERO, None), Submitted::Busy, "an unconsumed policy step is not replaced");
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Ok(())));
        assert!(close(r.target0(), m0 + 0.01));
        let run = r.fx.core.policy_run.as_ref().unwrap();
        assert_eq!((run.steps, run.last_live_target.as_ref().map(|t| t[0])), (1, Some(r.target0().unwrap())));
        // the camera frame id travels with the target
        assert_eq!(submit(&mut r, id, 0.012, Duration::ZERO, Some(Duration::ZERO)), Submitted::Accepted);
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Ok(())));
        assert_eq!(r.fx.core.policy_run.as_ref().unwrap().vision_frame_id, Some(41));
    }

    #[test]
    fn a_delayed_policy_target_from_stale_joint_inputs_ends_the_run() {
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Live);
        let max = r.fx.core.policy_run.as_ref().unwrap().max_input_age_ms;
        // the inputs were fresh when submitted, the frame waited past the bound
        submit(&mut r, id, 0.01, Duration::ZERO, None);
        r.fx.clock().advance(Duration::from_millis(max + 50));
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("INPUT_STALE")));
        assert!(r.fx.core.policy_run.is_none());
        assert_eq!(run_code(&r).as_deref(), Some("STALE_INPUT"));
        assert_eq!(r.target0(), None);
    }

    #[test]
    fn a_revoked_vision_generation_refuses_the_buffered_target_and_brakes() {
        use crate::policy_runtime::manifest::VideoEmbeddingSpec;
        use crate::policy_runtime::vision::VideoEmbeddingHub;
        use crate::policy_runtime::VisionStalePolicy;
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Live);
        // one applied step, so a held run has something to brake from
        submit(&mut r, id, 0.01, Duration::ZERO, None);
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Ok(())));
        let hub = VideoEmbeddingHub::new();
        let spec = VideoEmbeddingSpec { embedding_dim: 4, history_len: 1, max_staleness_ms: 200, max_window_span_ms: 400,
            fps_staleness_budget_ms: None, encoder_artifact_sha256: "e".into(), preprocess_config_sha256: "p".into(),
            embedding_norm: None, on_stale: VisionStalePolicy::Hold };
        hub.bind_policy(id, spec, 20.0);
        hub.bind_target_authorization(id, r.fx.core.policy_run.as_ref().unwrap().authorization.clone());
        assert_eq!(submit(&mut r, id, 0.02, Duration::ZERO, Some(Duration::ZERO)), Submitted::Accepted);
        hub.bump_invalidation_generation(); // the vision model / producer changed after the target was computed
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("POLICY_REVOKED")));
        let run = r.fx.core.policy_run.as_ref().expect("held, not ended");
        assert_eq!((run.steps, run.last_live_target.is_none()), (1, true));
        assert_eq!(r.target0(), None, "the held run brakes");
        // a target stamped after the revocation applies (first-step check against the measured pose)
        assert_eq!(submit(&mut r, id, 0.02, Duration::ZERO, Some(Duration::ZERO)), Submitted::Accepted);
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Ok(())));
        // unbinding revokes too
        assert_eq!(submit(&mut r, id, 0.02, Duration::ZERO, None), Submitted::Accepted);
        hub.unbind_policy(id);
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("POLICY_REVOKED")));
    }

    #[test]
    fn a_camera_sample_that_aged_past_the_bound_holds_or_ends_the_run() {
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Live);
        r.fx.core.set_policy_max_vision_age(id, Some(50), true);
        submit(&mut r, id, 0.01, Duration::ZERO, Some(Duration::from_millis(80)));
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("VISION_STALE")));
        assert!(r.fx.core.policy_run.is_some(), "on_stale hold keeps the run");
        assert_eq!(r.target0(), None);
        r.fx.core.set_policy_max_vision_age(id, Some(50), false);
        submit(&mut r, id, 0.01, Duration::ZERO, Some(Duration::from_millis(80)));
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("VISION_STALE")));
        assert!(r.fx.core.policy_run.is_none(), "on_stale stop ends the run");
        assert_eq!(run_code(&r).as_deref(), Some("VISION_STALE"));
    }

    #[test]
    fn a_shadow_target_has_no_output_authority() {
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Shadow);
        let before = r.fx.core.target_positions.clone();
        submit(&mut r, id, 0.01, Duration::ZERO, None);
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Ok(())));
        assert_eq!((r.target0(), &r.fx.core.target_positions), (None, &before));
        let run = r.fx.core.policy_run.as_ref().unwrap();
        assert_eq!((run.steps, run.last_shadow_target.is_some(), run.last_live_target.is_none()), (1, true, true));
    }

    #[test]
    fn a_stop_ends_the_run_and_refuses_its_buffered_target() {
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Live);
        submit(&mut r, id, 0.01, Duration::ZERO, None);
        r.stop.request();
        let rep = r.fx.step();
        assert!(rep.stopped);
        assert_eq!(rep.target.map(|(_, res)| res), Some(Err("STOPPED")));
        assert!(r.fx.core.policy_run.is_none());
        assert_eq!(r.target0(), None);
    }

    #[test]
    fn a_target_of_an_ended_run_or_a_servo_frame_during_a_run_is_refused() {
        let mut r = rig();
        let id = begin(&mut r, PolicyMode::Live);
        submit(&mut r, id, 0.01, Duration::ZERO, None);
        assert!(r.fx.core.stop_policy_run(Some(id), "STOPPED"));
        assert_eq!(r.fx.step().target.map(|(_, res)| res), Some(Err("STREAM_NOT_AUTHORIZED")), "the lease holder's servo stream is back");
        assert_eq!(r.fx.core.core.target_authority().stream, Some(servo_stream(r.gen)));
        let id2 = begin(&mut r, PolicyMode::Live);
        assert_ne!(id, id2);
        // a WS servo frame while a run is active: the legacy MOTION_BUSY
        assert_eq!(r.servo(Some(1), 0.001), None);
        let (refusal, reply) = r.step_reply();
        assert_eq!(refusal.map(|f| f.code), Some("MOTION_BUSY"));
        assert_eq!(reply.unwrap()["code"], json!("MOTION_BUSY"));
    }
}
