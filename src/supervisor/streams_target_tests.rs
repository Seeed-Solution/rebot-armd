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
    let stamp = inner.current_admission_stamp();
    let admitted = inner.check_admission(stamp).unwrap();
    SupervisorInner::enable_torque(admitted).expect("arm");
    let gen = inner.acquire_lease(CONN).expect("lease");
    // the connection got the lease: its servo stream is the one consumed
    let st = inner.current_admission_stamp();
    let adm = inner.check_admission(st).unwrap();
    SupervisorInner::authorize_servo_stream(adm, gen).expect("authorize servo stream");
    let (mut fx, broker, tw, stop) = ReleaseFixture::new(inner, BOOT, Capacities::default());
    fx.core.attach_stop_requests(stop.clone());
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
    let st = r.fx.core.current_admission_stamp();
    let adm = r.fx.core.check_admission(st).unwrap();
    SupervisorInner::authorize_servo_stream(adm, r.gen).expect("authorize");
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

    // the same with the stop the close requests: refused as stopped, matching legacy servo_reject
    let mut r = rig();
    assert_eq!(r.servo(Some(1), 0.004), None);
    r.fx.core.release_lease(CONN, r.gen);
    r.stop.request();
    let rep = r.fx.step();
    assert!(rep.stopped);
    assert_eq!(rep.target, Some((1, Err("STOPPED"))));
    assert_eq!(r.target0(), None);
    let stop_reply = servo_outcome_reply(&rep.target_refusal.unwrap(), &r.reply_ctx())
        .expect("STOPPED streaming frame must emit servo_reject with reason=stopped");
    assert_eq!(stop_reply["type"], "servo_reject");
    assert_eq!(stop_reply["status"], 409);
    assert_eq!(stop_reply["code"], "STOPPED");
    assert_eq!(stop_reply["error"], "STOPPED");
    assert_eq!(stop_reply["reason"], "stopped");
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
    let twin_q = twin.pose(100.0);
    let twin_ep = twin.epoch();
    let twin_stamp = twin.fx.core.current_admission_stamp();
    let twin_adm = twin.fx.core.check_admission(twin_stamp).unwrap();
    let legacy = crate::api::rest::ApiError::from(ArmError::from(SupervisorInner::servo(twin_adm, twin_q, twin_ep).unwrap_err()));
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
        let ep = r.epoch();
        let gen = r.gen;
        let stamp = r.fx.core.current_admission_stamp();
        let admitted = r.fx.core.check_admission(stamp).unwrap();
        SupervisorInner::begin_policy_run(admitted, PolicyRunStart {
            epoch_id: ep, connection_id: CONN.into(), lease_generation: gen, policy_id: "p".into(),
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

#[test]
fn close_servo_rejects_late_submissions_and_reclaims_stream_slots() {
    let mut r = rig();
    let q = r.pose(0.005);
    let auth1 = r.authority();
    let msg1 = ServoMessage {
        sequence_id: Some(1),
        positions: &q,
        authority: auth1,
        received_at: r.fx.now(),
        calibration_busy: false,
    };
    assert_eq!(r.port.submit_servo(&mut r.stream, &msg1), None);
    assert_eq!(r.fx.step().target, Some((1, Ok(()))));

    // Close generation 1: a late submission from generation 1 is rejected immediately without
    // advancing `ServoStream::last_sequence` or enqueuing into the mailbox.
    r.port.close_servo(r.gen);
    assert!(r.port.is_servo_closed(r.gen));
    let seq_before = r.stream.last_sequence();
    let late = ServoMessage {
        sequence_id: Some(2),
        positions: &q,
        authority: auth1,
        received_at: r.fx.now(),
        calibration_busy: false,
    };
    let reject = r.port.submit_servo(&mut r.stream, &late).expect("late submission must be refused");
    assert_eq!(reject["type"], json!("lease_lost"));
    assert_eq!(reject["generation"], json!(r.gen));
    assert_eq!(r.stream.last_sequence(), seq_before, "rejected late frame must not advance sequence");
    assert!(r.fx.step().target.is_none(), "rejected late frame must not enter mailbox");

    // Reclaim stream slots across 16 sequential lease generations (> MAX_STREAMS = 8).
    for g in 2..=17u64 {
        let mut s = ServoStream::new();
        s.lease_granted(g);
        let auth = Authority {
            lease_generation: g,
            epoch: r.epoch(),
            stop_generation: r.stop.current(),
            ..Authority::default()
        };
        let msg = ServoMessage {
            sequence_id: Some(1),
            positions: &q,
            authority: auth,
            received_at: r.fx.now(),
            calibration_busy: false,
        };
        assert_eq!(
            r.port.submit_servo(&mut s, &msg),
            None,
            "generation {g} must not hit STREAMS_FULL after closing previous generation"
        );
        r.port.close_servo(g);
        assert!(r.port.is_servo_closed(g));
    }
}

#[test]
fn lease_commit_authorizes_servo_stream_expiry_revokes_before_target_and_clear_fault_restores() {
    use crate::rt_core::broker::ClientId;
    use crate::rt_core::requests::lease::{
        install_lease_acquire, install_lease_preempt, ConnectionHandle, LeaseAcquire,
        LeaseInstallCtx, LeaseOutcome, LeasePreempt,
    };
    use crate::auth::OperatorGrant;

    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 200,
        ..ArmConfig::rebot_dm()
    });
    inner.tick(0.02);
    let stamp = inner.current_admission_stamp();
    let admitted = inner.check_admission(stamp).unwrap();
    SupervisorInner::enable_torque(admitted).expect("arm");
    inner.register_live_connection(1).unwrap();
    inner.register_live_connection(2).unwrap();

    let (mut fx, _broker, tw, stop) = ReleaseFixture::new(inner, BOOT, Capacities::default());
    let mut port = TargetPort::new(tw, ArmId::LOCAL, BOOT);

    // 1. `install_lease_acquire` automatically authorizes `servo_stream(g1)` (no manual call).
    let conn1 = ConnectionHandle::try_new(1, "connection:1").unwrap();
    let ctx1 = LeaseInstallCtx {
        expected_arm: ArmId::LOCAL,
        expected_boot: BOOT,
        stamp_arm: ArmId::LOCAL,
        stamp_boot: BOOT,
        now: fx.now(),
        request_id: crate::supervisor::next_admission_request_id(),
        client: ClientId(1),
        deadline: fx.now() + Duration::from_secs(1),
        enforce_single_use: true,
    };
    let g1 = match install_lease_acquire(
        &mut fx.core,
        &LeaseAcquire {
            connection: conn1,
            label: crate::model::HolderLabel::new("c1"),
            has_label: true,
        },
        &ctx1,
    )
    .unwrap()
    {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };
    assert_eq!(fx.core.core.target_authority().stream, Some(servo_stream(g1)));

    // 2. Buffer a target, advance virtual clock past `lease_timeout` (200 ms), then step:
    //    expiry MUST revoke `core.target_authority().stream` BEFORE Step 4 target consumption!
    let mut q = fx.core.commanded_positions.clone();
    let q0 = q[0];
    q[0] += 0.25;
    let mut s1 = ServoStream::new();
    s1.lease_granted(g1);
    let auth1 = Authority {
        lease_generation: g1,
        epoch: fx.core.safety.epoch_id(),
        stop_generation: stop.current(),
        ..Authority::default()
    };
    let msg1 = ServoMessage {
        sequence_id: Some(1),
        positions: &q,
        authority: auth1,
        received_at: fx.now(),
        calibration_busy: false,
    };
    assert_eq!(port.submit_servo(&mut s1, &msg1), None);
    // Advance past lease timeout (250 ms > 200 ms lease_timeout), keeping deadline > now by using a
    // raw frame or advancing 205ms with lease_timeout=150ms. Here lease_timeout=200ms and TTL=200ms,
    // so submit right at +100ms then advance +110ms (age=110ms < 200ms TTL, lease elapsed=210ms >= 200ms)!
    fx.clock().advance(Duration::from_millis(100));
    let msg2 = ServoMessage {
        sequence_id: Some(2),
        positions: &q,
        authority: auth1,
        received_at: fx.now(),
        calibration_busy: false,
    };
    assert_eq!(port.submit_servo(&mut s1, &msg2), None);
    fx.clock().advance(Duration::from_millis(110));
    let rep = fx.step();
    assert_eq!(
        rep.target.map(|(_, res)| res),
        Some(Err("LEASE_LOST")),
        "expired lease must refuse buffered target with LEASE_LOST before consumption"
    );
    assert_eq!(fx.core.core.target_authority().stream, None, "expired lease must revoke stream at owner");
    assert_eq!(fx.core.servo_target, None, "expired target must produce zero servo output");
    assert!((fx.core.commanded_positions[0] - q0).abs() < 1e-6);

    // 3. Preemption commits new generation g2 and automatically authorizes `servo_stream(g2)`.
    let conn2 = ConnectionHandle::try_new(2, "connection:2").unwrap();
    let ctx2 = LeaseInstallCtx {
        expected_arm: ArmId::LOCAL,
        expected_boot: BOOT,
        stamp_arm: ArmId::LOCAL,
        stamp_boot: BOOT,
        now: fx.now(),
        request_id: crate::supervisor::next_admission_request_id(),
        client: ClientId(2),
        deadline: fx.now() + Duration::from_secs(1),
        enforce_single_use: true,
    };
    let g2 = match install_lease_preempt(
        &mut fx.core,
        &LeasePreempt {
            connection: conn2,
            label: crate::model::HolderLabel::new("c2"),
            operator: OperatorGrant::unenforced(),
        },
        &ctx2,
    )
    .unwrap()
    {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };
    assert!(g2 > g1);
    assert_eq!(fx.core.core.target_authority().stream, Some(servo_stream(g2)));

    // 4. Acquiring a lease while E-stop is latched leaves the servo stream revoked until `clear_fault`
    //    succeeds, at which point `clear_fault` restores `servo_stream(g3)`.
    fx.core.estop_state.request_and_latch();
    let _ = fx.core.consume_pending_estop();
    fx.core.clear_active_lease_in_place();
    fx.core.core.revoke_stream();
    let ctx3 = LeaseInstallCtx {
        expected_arm: ArmId::LOCAL,
        expected_boot: BOOT,
        stamp_arm: ArmId::LOCAL,
        stamp_boot: BOOT,
        now: fx.now(),
        request_id: crate::supervisor::next_admission_request_id(),
        client: ClientId(1),
        deadline: fx.now() + Duration::from_secs(1),
        enforce_single_use: true,
    };
    let g3 = match install_lease_acquire(
        &mut fx.core,
        &LeaseAcquire {
            connection: conn1,
            label: crate::model::HolderLabel::new("c1-estop"),
            has_label: true,
        },
        &ctx3,
    )
    .unwrap()
    {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted under E-stop, got {other:?}"),
    };
    assert_eq!(
        fx.core.core.target_authority().stream,
        None,
        "servo stream must remain unauthorized while E-stop is latched"
    );
    fx.core.clear_fault().expect("clear E-stop");
    assert_eq!(
        fx.core.core.target_authority().stream,
        Some(servo_stream(g3)),
        "clear_fault must authorize the committed lease's servo stream"
    );
}

#[test]
fn expired_lease_during_pending_torque_enable_preserves_step2_timing_and_revokes_target() {
    use crate::rt_core::broker::ClientId;
    use crate::rt_core::requests::lease::{
        install_lease_acquire, ConnectionHandle, LeaseAcquire, LeaseInstallCtx, LeaseOutcome,
    };
    use crate::supervisor::driver_transitions::{EnableAuthority, TorqueTransition};

    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 200,
        ..ArmConfig::rebot_dm()
    });
    inner.tick(0.02);
    inner.register_live_connection(1).unwrap();
    let (mut fx, _broker, tw, stop) = ReleaseFixture::new(inner, BOOT, Capacities::default());
    let mut port = TargetPort::new(tw, ArmId::LOCAL, BOOT);

    let conn1 = ConnectionHandle::try_new(1, "connection:1").unwrap();
    let ctx1 = LeaseInstallCtx {
        expected_arm: ArmId::LOCAL,
        expected_boot: BOOT,
        stamp_arm: ArmId::LOCAL,
        stamp_boot: BOOT,
        now: fx.now(),
        request_id: crate::supervisor::next_admission_request_id(),
        client: ClientId(1),
        deadline: fx.now() + Duration::from_secs(1),
        enforce_single_use: true,
    };
    let g1 = match install_lease_acquire(
        &mut fx.core,
        &LeaseAcquire {
            connection: conn1,
            label: crate::model::HolderLabel::new("c1"),
            has_label: true,
        },
        &ctx1,
    )
    .unwrap()
    {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };
    assert_eq!(fx.core.core.target_authority().stream, Some(servo_stream(g1)));

    // Simulate an in-flight TorqueTransition::Enable owned by g1.
    fx.core.torque_transition = Some(TorqueTransition::Enable {
        origin: crate::supervisor::driver_transitions::EnableOrigin::BrokerTransition,
        was_torqued: false,
        prev_gripper_cmd: None,
        authority: EnableAuthority {
            lease_generation: g1,
            owner_conn_id: 1,
            admitted_epoch: fx.core.safety.epoch_id(),
        },
    });

    // Submit a target while the lease is live, then advance past lease_timeout (200 ms).
    let mut q = fx.core.commanded_positions.clone();
    let q0 = q[0];
    q[0] += 0.25;
    let mut s1 = ServoStream::new();
    s1.lease_granted(g1);
    let auth1 = Authority {
        lease_generation: g1,
        epoch: fx.core.safety.epoch_id(),
        stop_generation: stop.current(),
        ..Authority::default()
    };
    fx.clock().advance(Duration::from_millis(100));
    assert_eq!(
        port.submit_servo(
            &mut s1,
            &ServoMessage {
                sequence_id: Some(1),
                positions: &q,
                authority: auth1,
                received_at: fx.now(),
                calibration_busy: false,
            }
        ),
        None
    );
    fx.clock().advance(Duration::from_millis(110));

    // Step 1 sync must revoke target authority and queue close_servo(g1) WITHOUT prematurely
    // aborting the in-flight TorqueTransition::Enable before Step 2 `service_torque_transition`.
    assert!(fx.core.sync_nonblocking_disconnect_fence(fx.now()));
    assert_eq!(
        fx.core.core.target_authority().stream,
        None,
        "Step 1 expiry fence must revoke servo stream before target intake"
    );
    assert!(
        matches!(fx.core.torque_transition, Some(TorqueTransition::Enable { .. })),
        "Step 1 expiry fence must not prematurely abort TorqueTransition::Enable before Step 2"
    );

    // Running the full release lets Step 2 abort TorqueTransition::Enable cleanly and Step 4
    // refuse the buffered target with LEASE_LOST (zero motion output).
    let rep = fx.step();
    assert!(
        fx.core.torque_transition.is_none(),
        "Step 2 service_torque_transition must abort the expired Enable transition"
    );
    assert_eq!(rep.target.map(|(_, r)| r), Some(Err("LEASE_LOST")));
    assert_eq!(fx.core.servo_target, None);
    assert!((fx.core.commanded_positions[0] - q0).abs() < 1e-6);
}

#[test]
fn servo_j7_admitted_stamp_updates_only_when_frame_moves_j7() {
    let mut inner = SupervisorInner::new(ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    });
    inner.tick(0.02);
    let s0 = inner.current_admission_stamp();
    let a0 = inner.check_admission(s0).unwrap();
    SupervisorInner::enable_torque(a0).expect("arm");
    let g1 = inner.acquire_lease("conn-j7").expect("lease");
    let ep = inner.safety.epoch_id();

    // Clear stamp after arming so we can observe exact stamping behavior on servo frames.
    inner.j7_admitted_stamp = None;
    let base = inner.commanded_positions.clone();

    // Frame moving only J1 (J7 unchanged) must NOT stamp `j7_admitted_stamp`.
    let mut q_j1_only = base.clone();
    q_j1_only[0] += 0.05;
    let st1 = crate::supervisor::AdmissionStamp::for_rt_internal(0, ep, g1);
    let ad1 = inner.check_admission(st1).unwrap();
    SupervisorInner::servo_slice_rt(ad1, &q_j1_only, ep).expect("j1-only frame accepted");
    assert_eq!(
        inner.j7_admitted_stamp, None,
        "servo frame that does not move J7 must not advance j7_admitted_stamp"
    );

    // Frame moving J7 must stamp `j7_admitted_stamp` from its admission.
    let mut q_j7 = q_j1_only.clone();
    let g_idx = inner.config.gripper_index().expect("gripper joint");
    let g_lim = &inner.config.limits[g_idx];
    let mid = 0.5 * (g_lim.min_position + g_lim.max_position);
    q_j7[g_idx] = if (mid - q_j7[g_idx]).abs() > 1e-2 {
        mid
    } else {
        g_lim.min_position + 0.25 * (g_lim.max_position - g_lim.min_position)
    };
    let st2 = crate::supervisor::AdmissionStamp::for_rt_internal(0, ep, g1);
    let ad2 = inner.check_admission(st2).unwrap();
    SupervisorInner::servo_slice_rt(ad2, &q_j7, ep).expect("j7-moving frame accepted");
    assert_eq!(
        inner.j7_admitted_stamp,
        Some(crate::driver::PosForceIntentStamp {
            stop_gen: 0,
            estop_epoch: ep,
            lease_gen: g1,
        }),
        "servo frame moving J7 must stamp j7_admitted_stamp from its admission"
    );
}

#[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
mod pr_y_policy_unit_tests {
    use super::*;
    use crate::model::HolderLabel;
    use crate::motion::PolicyStats;
    use crate::policy_runtime::PolicyMode;
    use crate::rt_core::broker::{ClientId, Reply, Submit};
    use crate::rt_core::ops::Checkpoint;
    use crate::rt_core::requests::lease::ConnectionHandle;
    use crate::rt_core::requests::policy::{
        BeginPolicySpec, EndPolicySpec, PolicyCancelSlots, PolicyCommand, ReservePolicyLoadSpec,
        POLICY_CANCEL_SLOTS,
    };

    fn begin_rt_policy_via_broker(
        r: &mut Rig,
        conn_id: u64,
        token: u64,
        cancel_slot: u8,
        mode: PolicyMode,
        max_first_step_rad: f32,
        max_step_rad: Option<f32>,
        max_input_age_ms: u64,
        max_vision_age_ms: Option<u64>,
        vision_stale_hold: bool,
        gate_max_speed_scale: Option<f64>,
    ) -> u64 {
        r.fx.core.rt_executor_owned = true;
        r.fx.core.policy_run_spare = Some(crate::policy_runtime::PolicyRun::preallocated_rt_spare(
            r.fx.core.rt_policy_auth.clone(),
        ));
        let conn = ConnectionHandle::try_new(conn_id, CONN).unwrap();
        let auth = r.authority();
        let now = r.fx.now();
        let reserve = ReservePolicyLoadSpec {
            load_token: token,
            cancel_slot,
            admission_request_id: token * 100 + 1,
            prepared_boot: BOOT,
            connection: conn,
            mode,
            sim_only: false,
            i_am_at_the_arm: true,
            operator: crate::auth::OperatorGrant::unenforced(),
            reservation_expires_at: now + Duration::from_secs(10),
        };
        r._broker
            .submit(
                ClientId(conn_id),
                token * 10 + 1,
                auth,
                now + Duration::from_secs(5),
                Submit::Policy(PolicyCommand::ReserveLoad(reserve)),
            )
            .expect("admit reserve");
        r.fx.step();
        let mut replies = Vec::new();
        r._broker.poll(|_, rep| {
            replies.push(rep);
            true
        });
        assert!(
            replies.iter().any(|rep| matches!(rep, Reply::Ack { .. })),
            "ReserveLoad must ACK: {replies:?}"
        );

        let now2 = r.fx.now();
        let begin = BeginPolicySpec {
            load_token: token,
            cancel_slot,
            admission_request_id: token * 100 + 1,
            prepared_boot: BOOT,
            connection: conn,
            operator: crate::auth::OperatorGrant::unenforced(),
            calibration_revision: r.fx.core.calibration_revision,
            policy_id: HolderLabel::new("test_p"),
            artifact_digest: HolderLabel::new("digest_a"),
            backend: HolderLabel::new("mlp"),
            mode,
            sim_only: false,
            i_am_at_the_arm: true,
            max_first_step_rad,
            max_step_rad,
            max_input_age_ms,
            max_vision_age_ms,
            vision_stale_hold,
            period_us: 50_000,
            deadline: None,
            has_verified_release_gate: gate_max_speed_scale.is_some(),
            gate_policy_sha256: HolderLabel::new("digest_a"),
            gate_calibration_digest: HolderLabel::new(r.fx.core.active_calibration_digest_str()),
            gate_max_speed_scale: gate_max_speed_scale.unwrap_or(1.0),
            gate_is_production_release: false,
            production_qualified_verified: false,
        };
        r._broker
            .submit(
                ClientId(conn_id),
                token * 10 + 2,
                auth,
                now2 + Duration::from_secs(5),
                Submit::Policy(PolicyCommand::BeginPolicy(begin)),
            )
            .expect("admit begin");
        r.fx.step();
        replies.clear();
        r._broker.poll(|_, rep| {
            replies.push(rep);
            true
        });
        match replies.last() {
            Some(Reply::Ack {
                action: Some(aid), ..
            }) => aid.0,
            other => panic!("BeginPolicy must ACK with action_id: {other:?}"),
        }
    }

    #[test]
    fn policy_target_refusal_precedence_and_outcome_state_consistency() {
        let mut r = rig();
        let aid = begin_rt_policy_via_broker(
            &mut r,
            1,
            1,
            0,
            PolicyMode::Live,
            0.25,
            Some(0.10),
            100,
            Some(120),
            true,
            None,
        );
        let stream = policy_stream(aid);
        let auth = r.authority();
        let gen_ok = r.fx.core.policy_run.as_ref().unwrap().authorization.current();
        let now = r.fx.now();

        // 1. Invalid DOF precedes everything and does NOT terminate run.
        let mut bad_dof = r.raw_frame(stream, auth, 0.01);
        bad_dof.dof = 0;
        bad_dof.policy_generation = gen_ok;
        bad_dof.replaceable = false;
        assert_eq!(r.consume(&bad_dof), Err("INVALID_COMMAND"));
        assert!(r.fx.core.policy_run.is_some());

        // 2. Wrong boot precedes authority/stream and does NOT terminate run.
        let mut wrong_boot = r.raw_frame(stream, auth, 0.01);
        wrong_boot.boot = BootId(999);
        wrong_boot.policy_generation = gen_ok;
        wrong_boot.replaceable = false;
        assert_eq!(r.consume(&wrong_boot), Err("WRONG_TARGET"));
        assert!(r.fx.core.policy_run.is_some());

        // 3. Stream mismatch does NOT terminate run.
        let mut wrong_stream = r.raw_frame(policy_stream(aid + 99), auth, 0.01);
        wrong_stream.policy_generation = gen_ok;
        wrong_stream.replaceable = false;
        assert_eq!(r.consume(&wrong_stream), Err("STREAM_NOT_AUTHORIZED"));
        assert!(r.fx.core.policy_run.is_some());

        // 4. Revoked policy_generation precedes input staleness & deadline, brakes in place, keeps run alive.
        let mut revoked_and_stale = r.raw_frame(stream, auth, 0.01);
        revoked_and_stale.policy_generation = gen_ok + 7;
        revoked_and_stale.input_at = now - Duration::from_millis(500);
        revoked_and_stale.deadline = now - Duration::from_millis(100);
        revoked_and_stale.replaceable = false;
        assert_eq!(r.consume(&revoked_and_stale), Err("POLICY_REVOKED"));
        assert!(r.fx.core.policy_run.is_some(), "revocation holds rather than terminates");
        assert!(r.fx.core.servo_target.is_none());

        // 5. Missing vision_at when vision required -> VISION_STALE (hold mode keeps run alive).
        let gen_ok = r.fx.core.rt_policy_auth.current();
        let mut missing_vis = r.raw_frame(stream, auth, 0.01);
        missing_vis.policy_generation = gen_ok;
        missing_vis.vision_at = None;
        missing_vis.replaceable = false;
        assert_eq!(r.consume(&missing_vis), Err("VISION_STALE"));
        assert!(r.fx.core.policy_run.is_some());

        // 6. Future vision_at (> now + 50 ms) -> VISION_STALE (hold mode).
        let gen_ok = r.fx.core.rt_policy_auth.current();
        let mut future_vis = r.raw_frame(stream, auth, 0.01);
        future_vis.policy_generation = gen_ok;
        future_vis.vision_at = Some(now + Duration::from_millis(100));
        future_vis.replaceable = false;
        assert_eq!(r.consume(&future_vis), Err("VISION_STALE"));
        assert!(r.fx.core.policy_run.is_some());

        // 7. Stale input_at (> 100 ms) ALSO past deadline (> 200 ms) -> INPUT_STALE precedes deadline
        //    and terminates the run with matching `stop_reason == "STALE_INPUT"`.
        let gen_ok = r.fx.core.rt_policy_auth.current();
        let mut stale_and_past_dl = r.raw_frame(stream, auth, 0.01);
        stale_and_past_dl.policy_generation = gen_ok;
        stale_and_past_dl.input_at = now - Duration::from_millis(250);
        stale_and_past_dl.vision_at = Some(now);
        stale_and_past_dl.deadline = now - Duration::from_millis(50);
        stale_and_past_dl.replaceable = false;
        assert_eq!(r.consume(&stale_and_past_dl), Err("INPUT_STALE"));
        assert!(r.fx.core.policy_run.is_none(), "INPUT_STALE must terminate run");
        assert_eq!(
            r.fx.core
                .last_policy_obs
                .as_ref()
                .and_then(|o| o.stop_reason.as_deref()),
            Some("STALE_INPUT")
        );

        // 8. Future input_at (> now + 50 ms) on a fresh run -> INPUT_STALE & terminates as STALE_INPUT.
        let aid2 = begin_rt_policy_via_broker(
            &mut r,
            1,
            2,
            1,
            PolicyMode::Live,
            0.25,
            Some(0.10),
            100,
            None,
            false,
            None,
        );
        let gen2 = r.fx.core.policy_run.as_ref().unwrap().authorization.current();
        let now2 = r.fx.now();
        let mut future_in = r.raw_frame(policy_stream(aid2), r.authority(), 0.01);
        future_in.policy_generation = gen2;
        future_in.input_at = now2 + Duration::from_millis(100);
        future_in.replaceable = false;
        assert_eq!(r.consume(&future_in), Err("INPUT_STALE"));
        assert!(r.fx.core.policy_run.is_none());
        assert_eq!(
            r.fx.core
                .last_policy_obs
                .as_ref()
                .and_then(|o| o.stop_reason.as_deref()),
            Some("STALE_INPUT")
        );
    }

    #[test]
    fn policy_step5_revocation_without_new_frame_and_between_step4_step5_brakes_immediately() {
        let mut r = rig();
        let aid = begin_rt_policy_via_broker(
            &mut r,
            1,
            1,
            0,
            PolicyMode::Live,
            0.25,
            Some(0.25),
            200,
            None,
            false,
            None,
        );
        let q0 = r.fx.core.commanded_positions[0];
        let auth_handle = r.fx.core.rt_policy_auth.clone();
        let gen0 = auth_handle.current();
        let tgt = r.pose(0.05);

        // Case A: Revoke strictly BETWEEN Step 4 (accepted frame) and Step 5 (`BeforeOutput`):
        // Step 5 must catch `live_target_auth_gen != current()`, brake immediately, and move 0 rad.
        let auth_hook = auth_handle.clone();
        let triggered = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let trig_c = triggered.clone();
        r.fx.on_checkpoint(move |cp| {
            if cp == Checkpoint::BeforeOutput
                && !trig_c.swap(true, std::sync::atomic::Ordering::AcqRel)
            {
                auth_hook.revoke();
            }
        });
        assert_eq!(
            r.port.submit_policy(&PolicyTarget {
                action_id: aid,
                stamp: PolicyStamp {
                    authority: r.authority(),
                    authorization: gen0,
                    max_input_age: Duration::from_millis(200),
                },
                targets: &tgt,
                input_at: r.fx.now(),
                vision_at: None,
                vision_frame_id: None,
            }),
            crate::rt_core::mailbox::Submitted::Accepted
        );
        let rep = r.fx.step();
        assert_eq!(rep.target, Some((1, Ok(()))));
        assert!(r.fx.core.policy_run.is_some(), "revocation holds run active");
        assert!(r.fx.core.servo_target.is_none(), "Step 5 must brake in place");
        assert!(
            (r.fx.core.commanded_positions[0] - q0).abs() < 1e-6,
            "zero policy-driven movement after revocation between Step 4 and Step 5"
        );

        // Case B: Accept a fresh live target at the new generation, let 1 tick move, then revoke
        // across a release with NO new target frame -> next release brakes immediately at Step 5.
        let gen1 = auth_handle.current();
        assert_eq!(
            r.port.submit_policy(&PolicyTarget {
                action_id: aid,
                stamp: PolicyStamp {
                    authority: r.authority(),
                    authorization: gen1,
                    max_input_age: Duration::from_millis(200),
                },
                targets: &tgt,
                input_at: r.fx.now(),
                vision_at: None,
                vision_frame_id: None,
            }),
            crate::rt_core::mailbox::Submitted::Accepted
        );
        let rep2 = r.fx.step();
        assert_eq!(rep2.target, Some((2, Ok(()))));
        assert!(r.fx.core.servo_target.is_some());
        let q_after_step = r.fx.core.commanded_positions[0];
        assert!(q_after_step > q0, "fresh target must move arm");

        // Revoke with NO new frame submitted:
        auth_handle.revoke();
        let rep3 = r.fx.step();
        assert!(rep3.target.is_none());
        assert!(
            r.fx.core.servo_target.is_none(),
            "Step 5 must clear active policy servo_target even with no new mailbox frame"
        );
        assert!(r.fx.core.policy_run.is_some());
    }

    #[test]
    fn policy_first_step_jump_limit_subsequent_step_limit_and_release_gate_speed_scale() {
        // 1. First-step JUMP_LIMIT
        let mut r = rig();
        let mut unscaled_lims = Vec::new();
        r.fx.core.servo_limits_into(&mut unscaled_lims);
        let base_servo_v0 = unscaled_lims[0].max_velocity;
        let aid = begin_rt_policy_via_broker(
            &mut r,
            1,
            1,
            0,
            PolicyMode::Live,
            0.08,
            Some(0.04),
            200,
            None,
            false,
            Some(0.25),
        );
        let mut lims = Vec::new();
        r.fx.core.servo_limits_into(&mut lims);
        assert!(
            (lims[0].max_velocity - base_servo_v0 * 0.25).abs() < 1e-5,
            "RtGateBinding max_speed_scale=0.25 must scale RT servo velocity limits (got {} vs {})",
            lims[0].max_velocity,
            base_servo_v0 * 0.25
        );

        let gen = r.fx.core.rt_policy_auth.current();
        let q_jump = r.pose(0.15); // > 0.08 first-step limit
        let mut f_jump = r.raw_frame(policy_stream(aid), r.authority(), 0.15);
        f_jump.policy_generation = gen;
        f_jump.replaceable = false;
        f_jump.q[..q_jump.len()].copy_from_slice(&q_jump);
        assert_eq!(r.consume(&f_jump), Err("JUMP_LIMIT"));
        assert!(r.fx.core.policy_run.is_none());
        assert_eq!(
            r.fx.core
                .last_policy_obs
                .as_ref()
                .and_then(|o| o.stop_reason.as_deref()),
            Some("JUMP_LIMIT")
        );

        // 2. Subsequent-step STEP_LIMIT
        let aid2 = begin_rt_policy_via_broker(
            &mut r,
            1,
            2,
            1,
            PolicyMode::Live,
            0.08,
            Some(0.03),
            200,
            None,
            false,
            None,
        );
        let gen2 = r.fx.core.rt_policy_auth.current();
        let mut f_ok = r.raw_frame(policy_stream(aid2), r.authority(), 0.05);
        f_ok.policy_generation = gen2;
        f_ok.replaceable = false;
        assert_eq!(r.consume(&f_ok), Ok(()));
        assert_eq!(r.fx.core.policy_run.as_ref().unwrap().steps, 1);

        let mut f_step_big = r.raw_frame(policy_stream(aid2), r.authority(), 0.12); // +0.07 from 0.05 > 0.03
        f_step_big.policy_generation = gen2;
        f_step_big.seq = 2;
        f_step_big.replaceable = false;
        assert_eq!(r.consume(&f_step_big), Err("STEP_LIMIT"));
        assert!(r.fx.core.policy_run.is_none());
        assert_eq!(
            r.fx.core
                .last_policy_obs
                .as_ref()
                .and_then(|o| o.stop_reason.as_deref()),
            Some("STEP_LIMIT")
        );
    }

    #[test]
    fn policy_cancel_slots_exclusive_lifetime_refusal_isolation_and_stop_ordering() {
        // Part A: Exclusive-lifetime reclamation invariant on 64 slots
        let slots = std::sync::Arc::new(PolicyCancelSlots::new());
        let mut guards = Vec::new();
        for _ in 0..POLICY_CANCEL_SLOTS {
            guards.push(slots.claim_slot().expect("claim within 64"));
        }
        assert_eq!(slots.claim_slot().err(), Some("MOTION_BUSY"));

        // Retire slot 0 with an in-flight admitted request (req_id = 42) and active run token.
        let mut g0 = guards.remove(0);
        let tok0 = g0.token();
        let slot0 = g0.slot();
        g0.record_admitted_req(42);
        slots.set_active_run_token(tok0);
        g0.disarm();
        drop(g0);

        // Still blocked because `last_completed_policy_req_id < 42`.
        assert_eq!(slots.claim_slot().err(), Some("MOTION_BUSY"));

        // Advance completed watermark to 42, STILL blocked because `active_run_token == tok0`.
        slots.mark_policy_req_completed(42);
        assert_eq!(slots.claim_slot().err(), Some("MOTION_BUSY"));

        // Clear active_run_token -> now slot 0 reclaims cleanly!
        slots.set_active_run_token(0);
        let g_new = slots.claim_slot().expect("slot 0 reclaimed cleanly");
        assert_eq!(g_new.slot(), slot0);
        assert_ne!(g_new.token(), tok0);
        drop(g_new);
        drop(guards);

        // Part B: Refused higher-token ReserveLoad does NOT cancel active lower-token run;
        // delayed EndPolicy::{Hold,Finish,Fail} from old run A rejected after new run B starts.
        let mut r = rig();
        let aid_a = begin_rt_policy_via_broker(
            &mut r,
            1,
            10,
            0,
            PolicyMode::Live,
            0.25,
            Some(0.10),
            200,
            None,
            false,
            None,
        );
        let conn = ConnectionHandle::try_new(1, CONN).unwrap();
        let now = r.fx.now();
        r._broker
            .submit(
                ClientId(1),
                901,
                r.authority(),
                now + Duration::from_secs(5),
                Submit::Policy(PolicyCommand::ReserveLoad(ReservePolicyLoadSpec {
                    load_token: 99,
                    cancel_slot: 1,
                    admission_request_id: 9901,
                    prepared_boot: BOOT,
                    connection: conn,
                    mode: PolicyMode::Live,
                    sim_only: false,
                    i_am_at_the_arm: true,
                    operator: crate::auth::OperatorGrant::unenforced(),
                    reservation_expires_at: now + Duration::from_secs(5),
                })),
            )
            .expect("submit overlapping reserve");
        r.fx.step();
        let mut reps = Vec::new();
        r._broker.poll(|_, rep| {
            reps.push(rep);
            true
        });
        assert!(matches!(reps.last(), Some(Reply::Refused { code: "MOTION_BUSY", .. })));
        assert_eq!(
            r.fx.core.policy_run.as_ref().map(|p| (p.action_id, p.load_token)),
            Some((aid_a, 10)),
            "refused higher-token reserve must not cancel active lower-token run"
        );

        // Stop run A and start run B on same lease.
        r._broker
            .submit(
                ClientId(1),
                902,
                r.authority(),
                r.fx.now() + Duration::from_secs(5),
                Submit::Policy(PolicyCommand::EndPolicy(EndPolicySpec::Stop {
                    connection: conn,
                    lease_generation: r.gen,
                })),
            )
            .expect("stop A");
        r.fx.step();
        reps.clear();
        r._broker.poll(|_, rep| {
            reps.push(rep);
            true
        });
        assert!(r.fx.core.policy_run.is_none());

        let aid_b = begin_rt_policy_via_broker(
            &mut r,
            1,
            20,
            2,
            PolicyMode::Live,
            0.25,
            Some(0.10),
            200,
            None,
            false,
            None,
        );
        assert_ne!(aid_a, aid_b);

        // Delayed Hold / Finish / Fail for old run A must ALL be refused `POLICY_NOT_ACTIVE`.
        for (i, spec) in [
            EndPolicySpec::Hold {
                action_id: aid_a,
                load_token: 10,
                lease_generation: r.gen,
            },
            EndPolicySpec::Finish {
                action_id: aid_a,
                load_token: 10,
                lease_generation: r.gen,
                reason: HolderLabel::new("MAX_SECONDS"),
                final_steps: 5,
                final_stats: PolicyStats::default(),
            },
            EndPolicySpec::Fail {
                action_id: aid_a,
                load_token: 10,
                lease_generation: r.gen,
                reason: HolderLabel::new("INFERENCE_ERROR"),
                message: HolderLabel::new("stale"),
                final_steps: 5,
                final_stats: PolicyStats::default(),
            },
        ]
        .into_iter()
        .enumerate()
        {
            r._broker
                .submit(
                    ClientId(1),
                    910 + i as u64,
                    r.authority(),
                    r.fx.now() + Duration::from_secs(5),
                    Submit::Policy(PolicyCommand::EndPolicy(spec)),
                )
                .expect("submit delayed end");
            r.fx.step();
            reps.clear();
            r._broker.poll(|_, rep| {
            reps.push(rep);
            true
        });
            assert!(
                matches!(
                    reps.last(),
                    Some(Reply::Refused {
                        code: "POLICY_NOT_ACTIVE",
                        ..
                    })
                ),
                "delayed EndPolicy for run A must be refused POLICY_NOT_ACTIVE: {reps:?}"
            );
            assert_eq!(
                r.fx.core.policy_run.as_ref().map(|p| p.action_id),
                Some(aid_b),
                "run B must remain active"
            );
        }

        // Part C: `EndPolicy::Stop` interleaved with concurrent general Stop on unrelated active trajectory.
        // First end run B, start a plain trajectory, then race StopRequests::request() right before commit!
        r._broker
            .submit(
                ClientId(1),
                950,
                r.authority(),
                r.fx.now() + Duration::from_secs(5),
                Submit::Policy(PolicyCommand::EndPolicy(EndPolicySpec::Stop {
                    connection: conn,
                    lease_generation: r.gen,
                })),
            )
            .expect("stop B");
        r.fx.step();
        r._broker.poll(|_, _| true);
        assert!(r.fx.core.policy_run.is_none());

        let jog_goal = r.pose(0.2);
        let ep = r.epoch();
        let stamp = r.fx.core.current_admission_stamp();
        let adm = r.fx.core.check_admission(stamp).unwrap();
        let _ = SupervisorInner::jog_action(adm, jog_goal, Some(1.0), Some(ep), "jog")
            .expect("start active jog");
        assert!(r.fx.core.active_trajectory.is_some());

        // Attach shared `StopRequests` mirror and trigger an unapplied concurrent general Stop
        // right before `commit_policy(EndPolicy::Stop)` directly to exercise the exact race window!
        let stops = r.stop.clone();
        r.fx.core.stop_requests = Some(stops.clone());
        let auth_now = r.authority();
        let _gen_before = stops.request(); // unapplied general stop (`prev != stops.applied()`)
        let fake_desc = crate::rt_core::broker::Descriptor {
            stamp: crate::rt_core::broker::Stamp {
                arm: ArmId::LOCAL,
                boot: BOOT,
                client: ClientId(1),
                client_seq: 999,
                request: crate::rt_core::broker::RequestId(999),
                authority: auth_now,
                deadline: r.fx.now() + Duration::from_secs(2),
            },
            kind: crate::rt_core::broker::CommandKind::Policy(PolicyCommand::EndPolicy(
                EndPolicySpec::Stop {
                    connection: conn,
                    lease_generation: r.gen,
                },
            )),
        };
        let commit_ctx = r.fx.ctx(r.fx.now());
        let res = crate::rt_core::requests::policy::commit_policy(
            &mut r.fx.core,
            &fake_desc,
            PolicyCommand::EndPolicy(EndPolicySpec::Stop {
                connection: conn,
                lease_generation: r.gen,
            }),
            &commit_ctx,
        );
        assert!(res.is_ok(), "EndPolicy::Stop must commit: {res:?}");
        assert!(
            r.fx.core.active_trajectory.is_none(),
            "concurrent general Stop interleaved before EndPolicy::Stop must stop unrelated active trajectory"
        );
    }

    #[test]
    fn policy_review_regression_pre_gripper_revocation_and_force_diag_off_rt() {
        let mut r = rig();
        let g_idx = r.fx.core.config.gripper_index().expect("gripper joint");
        let close_dir = r
            .fx
            .core
            .config
            .gripper_policy
            .as_ref()
            .map_or(1.0_f32, |p| if p.closed_rad >= p.open_rad { 1.0 } else { -1.0 });
        let aid = begin_rt_policy_via_broker(
            &mut r,
            1,
            1,
            0,
            PolicyMode::Live,
            0.25,
            Some(0.25),
            200,
            None,
            false,
            Some(0.25),
        );
        // Step 1: accept a live policy target moving J7 toward closing so `gripper_stream_target`
        // is owned by the live policy run.
        let gen0 = r.fx.core.rt_policy_auth.current();
        let mut q = r.fx.core.commanded_positions.clone();
        let g_before = q[g_idx];
        q[g_idx] = (g_before + close_dir * 0.12).clamp(
            r.fx.core.config.limits[g_idx].min_position,
            r.fx.core.config.limits[g_idx].max_position,
        );
        let mut frame = r.raw_frame(policy_stream(aid), r.authority(), 0.0);
        frame.q[..q.len()].copy_from_slice(&q);
        frame.policy_generation = gen0;
        frame.replaceable = false;
        assert_eq!(r.consume(&frame), Ok(()));
        assert!(r.fx.core.gripper_stream_target.is_some());

        // Revoke authorization BEFORE the next tick (with NO new target frame):
        // Step 0.0b MUST run BEFORE 0.1 Gripper stall protection, clearing `gripper_stream_target`
        // and `servo_target` so neither CapRamp nor servo advances J7.
        r.fx.core.rt_policy_auth.revoke();
        let g_pre_step = r.fx.core.commanded_positions[g_idx];
        let _ = r.fx.step();
        assert_eq!(
            r.fx.core.gripper_stream_target, None,
            "pre-gripper revocation must clear policy-owned gripper_stream_target"
        );
        assert_eq!(r.fx.core.servo_target, None);
        assert!(
            (r.fx.core.commanded_positions[g_idx] - g_pre_step).abs() < 1e-6,
            "revoked policy must not advance J7 on tick"
        );

        // Blocker 4: on `rt_executor_owned == true`, gripper force-cap trip path
        // records into lock-free `GripperForceDiagSlot` instead of calling `tracing::warn!`.
        if let Some(cap_nm) = r
            .fx
            .core
            .config
            .gripper_policy
            .as_ref()
            .and_then(|p| p.max_close_torque_nm)
        {
            let (_, cap_before, _) = crate::rt_core::ops::gripper_force_diag().counts();
            r.fx.core.gripper_stalled = false;
            r.fx.core.gripper_force_capped = false;
            r.fx.core.gripper_over_cap_prev = false;
            r.fx.core.gripper_stream_target = Some(q[g_idx]);
            r.fx.core.gripper_stream_lease = Some(r.gen);
            r.fx.core.commanded_positions[g_idx] = g_before + close_dir * 0.05;
            r.fx.core.measured_positions[g_idx] = g_before;
            r.fx.core.torques[g_idx] = cap_nm + 1.0;
            let _ = r.fx.step();
            let (_, cap_after, _) = crate::rt_core::ops::gripper_force_diag().counts();
            assert!(
                cap_after > cap_before,
                "RT gripper force diag must record cap trip off-RT ({cap_before} -> {cap_after})"
            );
            crate::rt_core::ops::gripper_force_diag().drain_and_log_once();
        }
    }

    #[test]
    fn policy_review_regression_lease_expired_tx_fail_closed_and_j7_speed_scale() {
        use crate::driver::PosForceIntentKind;
        use crate::supervisor::TickOutput;

        let mut r = rig();
        let g_idx = r.fx.core.config.gripper_index().expect("gripper joint");
        let prof_j7_v = r.fx.core.config.limits[g_idx].max_velocity;

        let aid = begin_rt_policy_via_broker(
            &mut r,
            1,
            1,
            0,
            PolicyMode::Live,
            0.25,
            Some(0.25),
            200,
            None,
            false,
            Some(0.25),
        );
        // Major 5: even when `retained_motion_limits` holds unscaled profile limits, a live gated
        // policy with `RtGateBinding { max_speed_scale: 0.25 }` clamps J7 velocity ceiling to 0.25x.
        r.fx.core.tracking.retained_motion_limits = Some(r.fx.core.config.limits.clone());
        let ceil = r
            .fx
            .core
            .active_joint_velocity_ceiling(g_idx)
            .expect("j7 velocity ceiling");
        assert!(
            (ceil - prof_j7_v * 0.25).abs() < 1e-5,
            "J7 ceiling must obey RtGateBinding max_speed_scale=0.25 even with retained_motion_limits (got {ceil} vs {})",
            prof_j7_v * 0.25
        );

        // Submit a valid target so live policy is actively driving output.
        let gen0 = r.fx.core.rt_policy_auth.current();
        let mut frame = r.raw_frame(policy_stream(aid), r.authority(), 0.02);
        frame.policy_generation = gen0;
        frame.replaceable = false;
        assert_eq!(r.consume(&frame), Ok(()));

        // Blocker 3: expire the active lease between top-of-tick fence and TX staging; calling
        // `write_output_slice` directly MUST fail closed (`TickOutput::None` + J7 Hold intent),
        // never downgrading `output_lease_gen` to lease-exempt `0`.
        r.fx.core.lease_timeout = Duration::ZERO;
        if let Some(ref mut lease) = r.fx.core.active_lease {
            lease.acquired_at = std::time::Instant::now() - Duration::from_secs(5);
        }
        r.fx.core.j7_intent = None;
        let mut targets: Vec<crate::driver::JointTarget> = r
            .fx
            .core
            .commanded_positions
            .iter()
            .map(|&p| crate::driver::JointTarget::position(p))
            .collect();
        let out = r.fx.core.write_output_slice(&mut targets, 0.01, None);
        assert!(
            matches!(out, TickOutput::None),
            "expired live policy lease must fail closed in write_output_slice, got {out:?}"
        );
        assert!(
            matches!(
                r.fx.core.j7_intent.map(|i| i.kind),
                Some(PosForceIntentKind::Hold)
            ),
            "expired live policy lease must set J7 PosForce Hold intent, got {:?}",
            r.fx.core.j7_intent
        );
    }

    #[test]
    fn policy_review_regression_coalesced_bookkeeping_validity_bounds_and_rt_deadline() {
        use crate::rt_core::mailbox::{PolicyValidityBounds, Submitted};

        let mut r = rig();
        let aid = begin_rt_policy_via_broker(
            &mut r,
            1,
            1,
            0,
            PolicyMode::Live,
            0.25,
            Some(0.25),
            200,
            Some(200),
            true,
            None,
        );
        // Major 1: submit a replaceable servo frame (unconsumed in mailbox), then submit a policy
        // frame that coalesces it -> `TargetPort::submit_policy_with_validity` returns `Coalesced`
        // AND still tracks `aid` in `open_policy_actions` (not leaked/untracked).
        assert_eq!(r.servo(Some(1), 0.005), None);
        let q = r.pose(0.01);
        let gen0 = r.fx.core.rt_policy_auth.current();
        let sub = r.port.submit_policy_with_validity(
            &PolicyTarget {
                action_id: aid,
                stamp: PolicyStamp {
                    authority: r.authority(),
                    authorization: gen0,
                    max_input_age: Duration::from_millis(200),
                },
                targets: &q,
                input_at: r.fx.now(),
                vision_at: Some(r.fx.now()),
                vision_frame_id: Some(7),
            },
            PolicyValidityBounds {
                vision_window_expires_at: Some(r.fx.now() - Duration::from_millis(1)),
                handshake_expires_at: Some(r.fx.now() + Duration::from_secs(5)),
            },
        );
        assert!(
            matches!(sub, Submitted::Coalesced { .. }),
            "policy frame replacing unconsumed servo frame must return Coalesced: {sub:?}"
        );
        assert!(!r.port.is_policy_closed(aid));

        // Major 2 (part a): elapsed `vision_window_expires_at` -> `VISION_STALE` (hold mode keeps run).
        let rep = r.fx.step();
        assert_eq!(rep.target.map(|(_, res)| res), Some(Err("VISION_STALE")));
        assert!(r.fx.core.policy_run.is_some());
        assert_eq!(r.target0(), None);

        // Major 2 (part b): elapsed `handshake_expires_at` -> `HANDSHAKE_EXPIRED` & terminates run!
        let gen1 = r.fx.core.rt_policy_auth.current();
        let sub2 = r.port.submit_policy_with_validity(
            &PolicyTarget {
                action_id: aid,
                stamp: PolicyStamp {
                    authority: r.authority(),
                    authorization: gen1,
                    max_input_age: Duration::from_millis(200),
                },
                targets: &q,
                input_at: r.fx.now(),
                vision_at: Some(r.fx.now()),
                vision_frame_id: Some(8),
            },
            PolicyValidityBounds {
                vision_window_expires_at: Some(r.fx.now() + Duration::from_secs(5)),
                handshake_expires_at: Some(r.fx.now() - Duration::from_millis(1)),
            },
        );
        assert_eq!(sub2, Submitted::Accepted);
        let rep2 = r.fx.step();
        assert_eq!(rep2.target.map(|(_, res)| res), Some(Err("HANDSHAKE_EXPIRED")));
        assert!(r.fx.core.policy_run.is_none());
        assert_eq!(
            r.fx.core
                .last_policy_obs
                .as_ref()
                .and_then(|o| o.stop_reason.as_deref()),
            Some("HANDSHAKE_EXPIRED")
        );

        // Major 2 (part c): RT `PolicyRun::deadline` (`max_seconds`) enforced directly on `armd-control`
        // even with NO new target frame!
        let aid2 = begin_rt_policy_via_broker(
            &mut r,
            1,
            2,
            1,
            PolicyMode::Live,
            0.25,
            Some(0.25),
            200,
            None,
            false,
            None,
        );
        let dl = r.fx.now() + Duration::from_millis(30);
        r.fx.core.policy_run.as_mut().unwrap().deadline = Some(dl);
        let gen2 = r.fx.core.rt_policy_auth.current();
        let mut f_ok = r.raw_frame(policy_stream(aid2), r.authority(), 0.02);
        f_ok.policy_generation = gen2;
        f_ok.replaceable = false;
        assert_eq!(r.consume(&f_ok), Ok(()));
        assert!(r.fx.core.servo_target.is_some());

        // Advance past deadline with NO new frame -> next release terminates run with `MAX_SECONDS`!
        r.fx.clock().advance(Duration::from_millis(40));
        let _ = r.fx.step();
        assert!(
            r.fx.core.policy_run.is_none(),
            "elapsed PolicyRun::deadline must terminate run on RT tick"
        );
        assert_eq!(r.fx.core.servo_target, None);
        assert_eq!(
            r.fx.core
                .last_policy_obs
                .as_ref()
                .and_then(|o| o.stop_reason.as_deref()),
            Some("MAX_SECONDS")
        );
    }

    #[test]
    fn policy_review_regression_fenced_embedding_binding_and_action_scoped_auth() {
        use crate::policy_runtime::manifest::VideoEmbeddingSpec;
        use crate::policy_runtime::vision::VideoEmbeddingHub;
        use crate::policy_runtime::{PolicyAuthorization, VisionStalePolicy};

        // Action-scoped `PolicyAuthorization`: old run 100 cannot revoke or unbind active run 200.
        let auth = PolicyAuthorization::default();
        let g100 = auth.bind_action(100);
        assert_ne!(g100, 0);
        let g200 = auth.bind_action(200);
        assert_ne!(g200, g100);
        assert!(
            !auth.revoke_for_action(100),
            "stale action 100 must not revoke active action 200"
        );
        assert_eq!(auth.current(), g200);
        auth.unbind_action(100);
        assert_eq!(auth.current(), g200, "stale unbind_action(100) must leave action 200 intact");
        assert!(
            auth.revoke_for_action(200),
            "active action 200 must revoke its own generation"
        );
        let g200_rev = auth.current();
        assert_ne!(g200_rev, g200);
        assert_eq!(g200_rev >> 32, g200 >> 32, "action tag preserved on self-revoke");

        // Monotonically fenced `VideoEmbeddingHub::bind_policy_fenced`: late older run 100 cannot
        // overwrite newer run 200 or perturb its authorization, while unbinding 200 DOES revoke 200.
        let hub = VideoEmbeddingHub::new();
        let spec = VideoEmbeddingSpec {
            embedding_dim: 4,
            history_len: 1,
            max_staleness_ms: 200,
            max_window_span_ms: 400,
            fps_staleness_budget_ms: None,
            encoder_artifact_sha256: "a".repeat(64),
            preprocess_config_sha256: "b".repeat(64),
            embedding_norm: None,
            on_stale: VisionStalePolicy::Hold,
        };
        assert!(hub.bind_policy_fenced(200, spec.clone(), 20.0, Some(auth.clone())));
        let g_after_bind200 = auth.current();
        assert!(
            !hub.bind_policy_fenced(100, spec, 20.0, Some(auth.clone())),
            "late older runner 100 must be refused by bind_policy_fenced"
        );
        assert_eq!(
            auth.current(),
            g_after_bind200,
            "refused stale bind_policy_fenced(100) must not revoke active run 200"
        );
        hub.unbind_policy(200);
        assert_ne!(
            auth.current(),
            g_after_bind200,
            "unbinding active run 200 must revoke run 200's scoped authorization"
        );
    }

    #[test]
    fn policy_review_regression_qualification_pending_fence_and_immutable_outcomes() {
        use crate::policy_runtime::manifest::StrictPolicyContract;
        use crate::policy_runtime::qualification::QualificationRecord;
        use crate::policy_runtime::release_gate::{ActiveDeploymentContext, ReleaseGate};
        use crate::rt_core::requests::policy::{
            PolicyQualificationCoordinator, PolicyTerminalOutcome,
        };

        let dir = tempfile::tempdir().unwrap();
        let ledger = dir.path().join("qual.jsonl");
        let sha = "c".repeat(64);
        let cal = "d".repeat(64);
        let cfg = ArmConfig::rebot_dm();
        let pj = json!({
            "format": "rebot-onnx-policy/v1",
            "name": "qual_unit",
            "unit_mode": "rad",
            "action_dim": 7,
            "action_space": "joint_positions",
            "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
            "model_file": "model.rmlp"
        });
        let contract = StrictPolicyContract::parse(&serde_json::to_vec(&pj).unwrap()).unwrap();
        let ctx = ActiveDeploymentContext {
            scene_id: "scene_1".into(),
            tool_id: "tool_1".into(),
            calibration_digest: cal.clone(),
        };
        let make_gate = |tier: &str| {
            let raw = json!({
                "schema_version": "rebot-release-gate/v1",
                "authorization_tier": tier,
                "policy_sha256": sha,
                "scene_id": "scene_1",
                "tool_id": "tool_1",
                "profile_id": "rebot_dm",
                "calibration_digest": cal,
                "runner_config": {
                    "fps": 20,
                    "max_speed_scale": 0.25,
                    "unit_mode": "rad",
                    "action_space": "joint_positions"
                },
                "sim_eval": {
                    "seeds_total": 10,
                    "seeds_passed": 10,
                    "min_success_rate": 0.8,
                    "collisions": 0,
                    "keepout_violations": 0
                },
                "prerequisites": {
                    "calibration_verified": true,
                    "power_cut_witnessed": true,
                    "shadow_run_passed": true
                }
            });
            ReleaseGate::parse_and_verify(&raw, &sha, &cfg, &contract, 20.0, &ctx)
                .expect("verify gate")
        };

        let sup_gate = make_gate("supervised_first_run");
        let prod_gate = make_gate("production_release");

        let (mut tx, coord) = PolicyQualificationCoordinator::channel();
        // Seed ledger with `MIN_SUPERVISED_PASSES` prior passing supervised records.
        for aid in 1..=(crate::policy_runtime::qualification::MIN_SUPERVISED_PASSES as u64) {
            let rec_pass = QualificationRecord::for_run(&sup_gate, aid, 5, "MAX_SECONDS", true);
            coord
                .append_and_release_pending(&ledger, &rec_pass, None)
                .expect("append pass");
        }
        assert!(
            coord.check_production_qualified_locked(&ledger, &prod_gate).is_ok(),
            "prior passes qualify production release when no run is pending"
        );

        // Register a pending supervised run: MUST block `ProductionRelease` fail-closed!
        let mut guard = coord.register_pending(11, &sup_gate);
        let err = coord
            .check_production_qualified_locked(&ledger, &prod_gate)
            .expect_err("pending supervised run must block production release");
        assert_eq!(
            crate::api::rest::ApiError::from(err).code.as_deref(),
            Some("QUALIFICATION_REQUIRED")
        );

        // Immutable owner terminal outcome ring records exact non-pass reason (`STOPPED`/`cancelled`).
        tx.push(PolicyTerminalOutcome {
            action_id: 11,
            stop_reason: HolderLabel::new("STOPPED"),
            action_state: HolderLabel::new("cancelled"),
            steps: 3,
            stats: PolicyStats::default(),
        });
        assert!(coord.take_terminal_outcome(999).is_none());
        let out = coord
            .take_terminal_outcome(11)
            .expect("immutable outcome for action 11");
        assert_eq!(out.stop_reason.as_str(), "STOPPED");
        assert_eq!(out.action_state.as_str(), "cancelled");
        assert_eq!(out.steps, 3);

        // Atomically append the failed record and release the pending guard; latest record is now
        // non-passing (`passed == false`), so `check_production_qualified_locked` STILL refuses!
        let rec_fail = QualificationRecord::for_run(&sup_gate, 11, out.steps, out.stop_reason.as_str(), false);
        coord
            .append_and_release_pending(&ledger, &rec_fail, Some(&mut guard))
            .expect("append fail");
        assert!(!coord.has_pending_for(&prod_gate));
        let err2 = coord
            .check_production_qualified_locked(&ledger, &prod_gate)
            .expect_err("failed latest supervised run must revoke production qualification");
        assert_eq!(
            crate::api::rest::ApiError::from(err2).code.as_deref(),
            Some("QUALIFICATION_REQUIRED")
        );
    }
}
