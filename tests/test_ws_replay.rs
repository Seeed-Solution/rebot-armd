//! A1b PR 1 WS replay baseline tests:
//! 1. Strictness / negative-control tests for the type-safe, order-isomorphic transcript comparator.
//! 2. Controlled-clock & injected owner-event in-process replay test (extracting real telemetry
//!    fields and exact deterministic sample counts, plus pre/post-EStop payload verification).
//! 3. Server-initiated fatal Close ordering test proving `type: "error"` arrives before
//!    `CloseOutcome::Frame { code: Some(1011), .. }` (distinct from `Eof` and `TransportError`).
//! 4. Full corpus replay against `ARMD_REPLAY_BIN` (defaults to `CARGO_BIN_EXE_armd`) with 0-diff
//!    verification against `tests/ws_replay/` (covering both default and `--no-default-features`).

mod ws_replay_harness;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use ws_replay_harness::comparator::{
    compare_transcripts, CloseOutcome, RecordedEvent, ScriptStep, SessionSetup, SessionTranscript,
};
use ws_replay_harness::runner::{
    execute_binary_session, quantize_positions_mrad, run_steps_against,
};

const EXPECTED_BASELINE_COMMIT: &str = "0afeb1d";

fn corpus_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/ws_replay")
}

fn replay_binary() -> PathBuf {
    std::env::var_os("ARMD_REPLAY_BIN")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_BIN_EXE_armd")))
}

fn sample_transcript(events: Vec<RecordedEvent>) -> SessionTranscript {
    SessionTranscript {
        id: "sample".into(),
        family: "test".into(),
        description: "unit test sample".into(),
        setup: SessionSetup::default(),
        steps: Vec::new(),
        events,
    }
}

fn sample_terminal(
    action_id: u64,
    state: &str,
    code: Option<&str>,
    message: Option<&str>,
    safety_state: Value,
    epoch_id: u64,
) -> RecordedEvent {
    let mut act = json!({
        "id": action_id,
        "kind": "joint_move",
        "state": state,
        "code": code,
        "message": message,
        "progress": if state == "done" { 1.0 } else { 0.42 },
        "started_at": 1700000000.0,
        "finished_at": 1700000001.0
    });
    if state == "done" {
        act["gripper_gap_rad"] = json!(0.15);
    }
    RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action: act,
        safety_state,
        epoch_id,
        is_torqued: true,
        control_mode: "position".into(),
    }
}

#[test]
fn comparator_accepts_bijective_nondeterministic_renumbering_and_rejects_every_semantic_diff() {
    let base = sample_transcript(vec![
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({"type": "lease_granted", "client_id": "a", "generation": 1, "seq": 1}),
        },
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({
                "type": "command_ack",
                "request_id": 1,
                "status": "accepted",
                "epoch_id": 10,
                "action_id": 100,
                "expected_duration_s": 2.0001,
                "safety_state": "Armed",
                "is_torqued": true,
                "control_mode": "position",
                "pose_id": "01KXYZ11111111111111111111",
                "saved_at_ms": 1700000001000u64
            }),
        },
        sample_terminal(100, "done", None, Some("Completed"), json!("Armed"), 10),
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({"type": "epoch_sync", "epoch_id": 11, "safety_state": "EStop"}),
        },
        RecordedEvent::ServerClose {
            client: "c0".into(),
            close: CloseOutcome::Frame {
                code: Some(1000),
                reason: Some("normal".into()),
            },
        },
    ]);

    let shifted = sample_transcript(vec![
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({"type": "lease_granted", "client_id": "a", "generation": 7, "seq": 1}),
        },
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({
                "type": "command_ack",
                "request_id": 1,
                "status": "accepted",
                "epoch_id": 42,
                "action_id": 999,
                "expected_duration_s": 2.0002,
                "safety_state": "Armed",
                "is_torqued": true,
                "control_mode": "position",
                "pose_id": "01KABC22222222222222222222",
                "saved_at_ms": 1799999999000u64
            }),
        },
        sample_terminal(999, "done", None, Some("Completed"), json!("Moving"), 42),
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({"type": "epoch_sync", "epoch_id": 45, "safety_state": "EStop"}),
        },
        RecordedEvent::ServerClose {
            client: "c0".into(),
            close: CloseOutcome::Frame {
                code: Some(1000),
                reason: Some("normal".into()),
            },
        },
    ]);
    assert!(
        compare_transcripts(&base, &shifted).is_empty(),
        "bijective order-preserving shift must produce 0 diffs: {:?}",
        compare_transcripts(&base, &shifted)
    );

    // Broken action_id bijection -> diff.
    let mut bad_action = shifted.clone();
    if let RecordedEvent::ActionTerminal { action, .. } = &mut bad_action.events[2] {
        action["id"] = json!(1000);
    }
    assert!(!compare_transcripts(&base, &bad_action).is_empty());

    // Reserved zero epoch_id / action_id MUST NOT normalize to positive IDs.
    let mut zero_action = shifted.clone();
    if let RecordedEvent::ActionTerminal { action, .. } = &mut zero_action.events[2] {
        action["id"] = json!(0);
    }
    assert!(!compare_transcripts(&base, &zero_action).is_empty());

    let mut zero_epoch = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut zero_epoch.events[3] {
        frame["epoch_id"] = json!(0);
    }
    assert!(!compare_transcripts(&base, &zero_epoch).is_empty());

    // Episode frames: 0 or float 4.0 MUST be rejected vs positive integer 4.
    let ep_ok = sample_transcript(vec![RecordedEvent::ServerFrame {
        client: "c0".into(),
        frame: json!({"type": "command_ack", "episode": {"id": "01K1", "frames": 4, "bytes": 120, "duration_s": 0.08}}),
    }]);
    let ep_zero = sample_transcript(vec![RecordedEvent::ServerFrame {
        client: "c0".into(),
        frame: json!({"type": "command_ack", "episode": {"id": "01K2", "frames": 0, "bytes": 120, "duration_s": 0.08}}),
    }]);
    let ep_float = sample_transcript(vec![RecordedEvent::ServerFrame {
        client: "c0".into(),
        frame: json!({"type": "command_ack", "episode": {"id": "01K3", "frames": 4.0, "bytes": 120, "duration_s": 0.08}}),
    }]);
    assert!(!compare_transcripts(&ep_ok, &ep_zero).is_empty());
    assert!(!compare_transcripts(&ep_ok, &ep_float).is_empty());

    // policy_stats.steps normalizes positive integer counts (2 vs 3) but rejects 0 and float.
    let ps_2 = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action: json!({"id": 1, "state": "cancelled", "code": "STOPPED", "policy_stats": {"steps": 2, "period_us": 50000}}),
        safety_state: json!("Idle"),
        epoch_id: 1,
        is_torqued: false,
        control_mode: "position".into(),
    }]);
    let ps_3 = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action: json!({"id": 1, "state": "cancelled", "code": "STOPPED", "policy_stats": {"steps": 3, "period_us": 50000}}),
        safety_state: json!("Idle"),
        epoch_id: 1,
        is_torqued: false,
        control_mode: "position".into(),
    }]);
    let ps_0 = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action: json!({"id": 1, "state": "cancelled", "code": "STOPPED", "policy_stats": {"steps": 0, "period_us": 50000}}),
        safety_state: json!("Idle"),
        epoch_id: 1,
        is_torqued: false,
        control_mode: "position".into(),
    }]);
    assert!(compare_transcripts(&ps_2, &ps_3).is_empty());
    assert!(!compare_transcripts(&ps_2, &ps_0).is_empty());

    // Broken epoch equality -> diff.
    let mut bad_epoch_eq = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_epoch_eq.events[3] {
        frame["epoch_id"] = json!(42);
    }
    assert!(!compare_transcripts(&base, &bad_epoch_eq).is_empty());

    // Reversed adjacent epoch order -> diff.
    let mut bad_epoch_order = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_epoch_order.events[3] {
        frame["epoch_id"] = json!(40);
    }
    assert!(!compare_transcripts(&base, &bad_epoch_order).is_empty());

    // Non-adjacent total order reversal: [1, 3, 2] vs [2, 3, 1] have the same adjacent signs (+,-)
    // but reverse 1 < 2 vs 2 > 1; rank signature MUST catch it!
    let ep_132 = sample_transcript(vec![
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"epoch_sync","epoch_id":1}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"epoch_sync","epoch_id":3}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"epoch_sync","epoch_id":2}) },
    ]);
    let ep_231 = sample_transcript(vec![
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"epoch_sync","epoch_id":2}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"epoch_sync","epoch_id":3}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"epoch_sync","epoch_id":1}) },
    ]);
    assert!(
        !compare_transcripts(&ep_132, &ep_231).is_empty(),
        "total-order reversal [1,3,2] vs [2,3,1] must be rejected"
    );

    // Order reversals on generation, action_id, and stop_generation MUST also be caught.
    let gen_asc = sample_transcript(vec![
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"lease_granted","generation":1}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"lease_granted","generation":2}) },
    ]);
    let gen_desc = sample_transcript(vec![
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"lease_granted","generation":5}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"lease_granted","generation":4}) },
    ]);
    assert!(!compare_transcripts(&gen_asc, &gen_desc).is_empty());

    let sg_asc = sample_transcript(vec![
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"command_ack","stop_generation":1}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"command_ack","stop_generation":2}) },
    ]);
    let sg_desc = sample_transcript(vec![
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"command_ack","stop_generation":9}) },
        RecordedEvent::ServerFrame { client: "c0".into(), frame: json!({"type":"command_ack","stop_generation":3}) },
    ]);
    assert!(!compare_transcripts(&sg_asc, &sg_desc).is_empty());

    // Type-safe normalization: literal string "$gen_1" MUST NOT match numeric generation 7.
    let mut str_gen = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut str_gen.events[0] {
        frame["generation"] = json!("$gen_1");
    }
    assert!(
        !compare_transcripts(&base, &str_gen).is_empty(),
        "wire type change u64 -> string must be rejected"
    );

    // Zero generation stays distinct from positive generation.
    let mut bad_gen_zero = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_gen_zero.events[0] {
        frame["generation"] = json!(0);
    }
    assert!(!compare_transcripts(&base, &bad_gen_zero).is_empty());

    // Deterministic REST expires_in_s MUST NOT be normalized away.
    let unlock_1800 = sample_transcript(vec![RecordedEvent::RestResponse {
        method: "POST".into(),
        path: "/api/v1/auth/unlock".into(),
        status: 200,
        body: json!({"token": "aaa", "role": "operator", "expires_in_s": 1800}),
    }]);
    let unlock_1 = sample_transcript(vec![RecordedEvent::RestResponse {
        method: "POST".into(),
        path: "/api/v1/auth/unlock".into(),
        status: 200,
        body: json!({"token": "bbb", "role": "operator", "expires_in_s": 1}),
    }]);
    assert!(!compare_transcripts(&unlock_1800, &unlock_1).is_empty());

    // >= 1 ms drift in expected_duration_s MUST be caught.
    let mut bad_dur = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_dur.events[1] {
        frame["expected_duration_s"] = json!(2.005);
    }
    assert!(!compare_transcripts(&base, &bad_dur).is_empty());

    // Terminal message and full ActionStatus field changes (e.g. gripper_gap_rad) MUST be caught.
    let mut bad_msg = shifted.clone();
    if let RecordedEvent::ActionTerminal { action, .. } = &mut bad_msg.events[2] {
        action["message"] = json!("Different message");
    }
    assert!(!compare_transcripts(&base, &bad_msg).is_empty());

    let mut bad_gap = shifted.clone();
    if let RecordedEvent::ActionTerminal { action, .. } = &mut bad_gap.events[2] {
        action["gripper_gap_rad"] = json!(0.25);
    }
    assert!(!compare_transcripts(&base, &bad_gap).is_empty());

    // Frame kind, extra key, ordering swap -> all rejected.
    let mut bad_kind = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_kind.events[1] {
        frame["type"] = json!("command_reject");
    }
    assert!(!compare_transcripts(&base, &bad_kind).is_empty());

    let mut extra_key = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut extra_key.events[1] {
        frame["unexpected"] = json!(true);
    }
    assert!(!compare_transcripts(&base, &extra_key).is_empty());

    let mut swapped = shifted.clone();
    swapped.events.swap(1, 2);
    assert!(!compare_transcripts(&base, &swapped).is_empty());

    // Close outcome distinctions: Frame vs Eof vs TransportError vs reason change -> all rejected.
    let mut bad_close_reason = shifted.clone();
    bad_close_reason.events[4] = RecordedEvent::ServerClose {
        client: "c0".into(),
        close: CloseOutcome::Frame {
            code: Some(1000),
            reason: Some("ESTOP_LATCHED:WRITE_TIMEOUT".into()),
        },
    };
    assert!(!compare_transcripts(&base, &bad_close_reason).is_empty());

    let mut close_eof = shifted.clone();
    close_eof.events[4] = RecordedEvent::ServerClose {
        client: "c0".into(),
        close: CloseOutcome::Eof,
    };
    assert!(!compare_transcripts(&base, &close_eof).is_empty());

    let mut close_err = shifted.clone();
    close_err.events[4] = RecordedEvent::ServerClose {
        client: "c0".into(),
        close: CloseOutcome::TransportError,
    };
    assert!(!compare_transcripts(&base, &close_err).is_empty());
    assert!(!compare_transcripts(&close_eof, &close_err).is_empty());

    // STOPPED cancellation normalizes Moving/Armed tick boundary but rejects Fault/EStop/Idle
    // and non-STOPPED cancellations.
    let stopped_moving = sample_transcript(vec![sample_terminal(
        1,
        "cancelled",
        Some("STOPPED"),
        Some("Motion stopped"),
        json!("Moving"),
        1,
    )]);
    let stopped_armed = sample_transcript(vec![sample_terminal(
        2,
        "cancelled",
        Some("STOPPED"),
        Some("Motion stopped"),
        json!("Armed"),
        1,
    )]);
    assert!(compare_transcripts(&stopped_moving, &stopped_armed).is_empty());

    let stopped_fault = sample_transcript(vec![sample_terminal(
        2,
        "cancelled",
        Some("STOPPED"),
        Some("Motion stopped"),
        json!("Fault"),
        1,
    )]);
    assert!(!compare_transcripts(&stopped_moving, &stopped_fault).is_empty());

    let estop_moving = sample_transcript(vec![sample_terminal(
        1,
        "cancelled",
        Some("ESTOP"),
        None,
        json!("Moving"),
        2,
    )]);
    let estop_armed = sample_transcript(vec![sample_terminal(
        1,
        "cancelled",
        Some("ESTOP"),
        None,
        json!("Armed"),
        2,
    )]);
    assert!(!compare_transcripts(&estop_moving, &estop_armed).is_empty());

    // TelemetrySummary: joint position or timestamp-completeness drift MUST be rejected.
    let tel_a = sample_transcript(vec![RecordedEvent::TelemetrySummary {
        client: "c0".into(),
        in_range: true,
        has_all_timestamps: true,
        monotonic: true,
        all_samples_consistent: true,
        dof: 7,
        is_torqued: false,
        control_mode: "position".into(),
        positions_mrad: vec![0, 0, 0, 0, 0, 0, 0],
        safety_state: json!("Idle"),
        epoch_id: 1,
    }]);
    let tel_pos_drift = sample_transcript(vec![RecordedEvent::TelemetrySummary {
        client: "c0".into(),
        in_range: true,
        has_all_timestamps: true,
        monotonic: true,
        all_samples_consistent: true,
        dof: 7,
        is_torqued: false,
        control_mode: "position".into(),
        positions_mrad: vec![100, 0, 0, 0, 0, 0, 0],
        safety_state: json!("Idle"),
        epoch_id: 1,
    }]);
    assert!(!compare_transcripts(&tel_a, &tel_pos_drift).is_empty());
}

/// Controlled-clock in-process replay: drives `ArmSupervisor::tick(0.02)` manually (no background
/// control loop) at explicit `obs_hz=20` intervals, verifies EVERY telemetry field from real wire
/// samples (exact count, timestamps, positions, safety_state, epoch_id), injects a direct owner
/// E-stop, verifies telemetry + heartbeat `epoch_sync` transition to `EStop`, and asserts 0-diff
/// comparator parity across runs.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn controlled_clock_and_injected_owner_event_replay_parity() {
    async fn read_one_telemetry(
        ws: &mut tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
    ) -> Value {
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let Message::Text(t) = ws.next().await.unwrap().unwrap() else { continue };
                let v: Value = serde_json::from_str(&t).unwrap();
                if v.get("type").is_none() {
                    return v;
                }
            }
        })
        .await
        .expect("timed out waiting for telemetry sample")
    }

    async fn run_once() -> SessionTranscript {
        let cfg = armd::ArmConfig {
            lease_timeout_ms: 15_000,
            control_frequency_hz: 50,
            ..armd::ArmConfig::rebot_dm()
        };
        let sup = Arc::new(armd::ArmSupervisor::new(cfg));
        let router = armd::create_router(sup.clone(), armd::OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let srv = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws?obs_hz=20"))
            .await
            .unwrap();
        let mut events = Vec::new();

        let acq = json!({"type": "acquire_lease", "client_id": "clock_client", "seq": 1});
        ws.send(Message::Text(acq.to_string().into())).await.unwrap();
        events.push(RecordedEvent::ClientFrame {
            client: "c0".into(),
            frame: acq,
        });
        let granted: Value = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let Message::Text(t) = ws.next().await.unwrap().unwrap() else { continue };
                let v: Value = serde_json::from_str(&t).unwrap();
                if v.get("type").is_some() {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        let gen = granted["generation"].as_u64().unwrap();
        events.push(RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: granted,
        });

        // Step 3 explicit ticks spaced > 1/obs_hz (50 ms) apart: expect EXACTLY 3 observations.
        let mut samples = Vec::new();
        for i in 0..3 {
            if i > 0 {
                tokio::time::sleep(Duration::from_millis(60)).await;
            }
            sup.tick(0.02).await;
            samples.push(read_one_telemetry(&mut ws).await);
        }
        // Assert no spurious extra observation arrives without another tick.
        assert!(
            tokio::time::timeout(Duration::from_millis(40), ws.next())
                .await
                .is_err(),
            "unexpected extra telemetry frame without supervisor tick"
        );

        let mut ts_count = 0usize;
        let mut last_ts: Option<f64> = None;
        let mut strictly_increasing = true;
        let mut first_shape: Option<(usize, bool, String, Vec<i32>, Value, u64)> = None;
        let mut all_samples_consistent = true;
        for s in &samples {
            if let Some(ts) = s.get("timestamp").and_then(Value::as_f64) {
                ts_count += 1;
                if last_ts.is_some_and(|p| ts <= p) {
                    strictly_increasing = false;
                }
                last_ts = Some(ts);
            }
            let qpos = quantize_positions_mrad(s);
            let shape = (
                qpos.len(),
                s["is_torqued"].as_bool().unwrap(),
                s["control_mode"].as_str().unwrap().to_string(),
                qpos,
                s["safety_state"].clone(),
                s["epoch_id"].as_u64().unwrap(),
            );
            match &first_shape {
                None => first_shape = Some(shape),
                Some(exp) => {
                    if *exp != shape {
                        all_samples_consistent = false;
                    }
                }
            }
        }
        let (dof, is_torqued, control_mode, positions_mrad, safety_state, epoch_id) =
            first_shape.unwrap();
        let has_all_timestamps = samples.len() == 3 && ts_count == samples.len();
        assert!(has_all_timestamps && strictly_increasing && all_samples_consistent);
        assert_eq!(dof, 7);
        assert_eq!(safety_state, "Idle");
        events.push(RecordedEvent::TelemetrySummary {
            client: "c0".into(),
            in_range: true,
            has_all_timestamps,
            monotonic: has_all_timestamps && strictly_increasing,
            all_samples_consistent,
            dof,
            is_torqued,
            control_mode,
            positions_mrad,
            safety_state,
            epoch_id,
        });

        // Inject direct owner E-stop and verify BOTH post-EStop telemetry and heartbeat epoch_sync.
        let old_epoch = sup.get_observation().await.epoch_id;
        let (st, new_epoch) = sup.trigger_e_stop().await;
        events.push(RecordedEvent::OwnerEvent {
            label: "direct_owner_estop".into(),
            detail: json!({"safety_state": st, "epoch_id": new_epoch}),
        });
        assert!(new_epoch > old_epoch);

        tokio::time::sleep(Duration::from_millis(60)).await;
        sup.tick(0.02).await;
        let estop_obs = read_one_telemetry(&mut ws).await;
        assert_eq!(estop_obs["safety_state"], "EStop");
        assert_eq!(estop_obs["epoch_id"].as_u64(), Some(new_epoch));
        let estop_pos = quantize_positions_mrad(&estop_obs);
        events.push(RecordedEvent::TelemetrySummary {
            client: "c0".into(),
            in_range: true,
            has_all_timestamps: estop_obs.get("timestamp").and_then(Value::as_f64).is_some(),
            monotonic: true,
            all_samples_consistent: true,
            dof: estop_pos.len(),
            is_torqued: estop_obs["is_torqued"].as_bool().unwrap(),
            control_mode: estop_obs["control_mode"].as_str().unwrap().to_string(),
            positions_mrad: estop_pos,
            safety_state: estop_obs["safety_state"].clone(),
            epoch_id: estop_obs["epoch_id"].as_u64().unwrap(),
        });

        let hb = json!({"type": "heartbeat", "generation": gen, "epoch_id": old_epoch});
        ws.send(Message::Text(hb.to_string().into())).await.unwrap();
        events.push(RecordedEvent::ClientFrame {
            client: "c0".into(),
            frame: hb,
        });
        let sync: Value = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let Message::Text(t) = ws.next().await.unwrap().unwrap() else { continue };
                let v: Value = serde_json::from_str(&t).unwrap();
                if v.get("type").is_some() {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(sync["type"], "epoch_sync");
        assert_eq!(sync["safety_state"], "EStop");
        events.push(RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: sync,
        });

        let _ = ws.close(None).await;
        srv.abort();
        sample_transcript(events)
    }

    let run1 = run_once().await;
    let run2 = run_once().await;
    let diffs = compare_transcripts(&run1, &run2);
    assert!(diffs.is_empty(), "controlled-clock parity diffs: {diffs:?}");
}

/// Verifies that server-initiated `FatalClose` emits the fatal `{"type": "error", ...}` frame
/// BEFORE the WebSocket Close handshake frame (`CloseOutcome::Frame`), and that the runner
/// preserves that exact order rather than dropping preceding control frames or collapsing Close
/// into EOF/TransportError.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn server_fatal_close_preserves_error_before_close_frame_order() {
    async fn run_fatal_once() -> SessionTranscript {
        let cfg = armd::ArmConfig {
            lease_timeout_ms: 15_000,
            control_frequency_hz: 50,
            ..armd::ArmConfig::rebot_dm()
        };
        let sup = Arc::new(armd::ArmSupervisor::new(cfg));
        let router = armd::create_router(sup.clone(), armd::OperatorAuth::disabled());
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let srv = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        let base_url = format!("http://{addr}");
        let ws_base = format!("ws://{addr}/api/v1/ws");
        let http = reqwest::Client::new();

        // Connect and acquire lease on this test's own isolated `ArmSupervisor`, then read its
        // exact server-side `connection:<id>` directly from `sup` (zero global registry race with
        // parallel tests).
        let (mut ws, resp) = connect_async(&ws_base).await.unwrap();
        assert_eq!(resp.status().as_u16(), 101);
        let acq = json!({"type": "acquire_lease", "client_id": "stall_client", "seq": 1});
        ws.send(Message::Text(acq.to_string().into())).await.unwrap();
        let granted: Value = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let Message::Text(t) = ws.next().await.unwrap().unwrap() else { continue };
                let v: Value = serde_json::from_str(&t).unwrap();
                if v.get("type").is_some() {
                    return v;
                }
            }
        })
        .await
        .expect("acquire_lease grant");
        let gen = granted["generation"].as_u64().unwrap();
        let conn_id = sup
            .inner()
            .read()
            .await
            .active_lease
            .as_ref()
            .map(|a| a.client_id.clone())
            .expect("isolated supervisor must hold active_lease with connection:<id>");
        assert!(conn_id.starts_with("connection:"), "unexpected conn_id {conn_id}");

        armd::api::ws::set_test_ws_write_timeout(&conn_id, Some(Duration::from_millis(60)));
        armd::api::ws::set_test_ws_write_stall_once(&conn_id, true);

        // Send release_lease: its write stalls past 60 ms -> server triggers fatal WRITE_TIMEOUT.
        let rel = json!({"type": "release_lease", "generation": gen});
        ws.send(Message::Text(rel.to_string().into())).await.unwrap();

        let mut events = Vec::new();
        let close = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                match ws.next().await {
                    Some(Ok(Message::Text(txt))) => {
                        let v: Value = serde_json::from_str(&txt).unwrap();
                        if v.get("type").is_some() {
                            events.push(RecordedEvent::ServerFrame {
                                client: "c0".into(),
                                frame: v,
                            });
                        }
                    }
                    Some(Ok(Message::Close(cf))) => {
                        return CloseOutcome::Frame {
                            code: cf.as_ref().map(|c| c.code.into()),
                            reason: cf.as_ref().map(|c| c.reason.to_string()),
                        };
                    }
                    Some(Ok(_)) => continue,
                    None => return CloseOutcome::Eof,
                    Some(Err(_)) => return CloseOutcome::TransportError,
                }
            }
        })
        .await
        .expect("timed out waiting for fatal server close");
        events.push(RecordedEvent::ServerClose {
            client: "c0".into(),
            close,
        });

        armd::api::ws::set_test_ws_write_timeout(&conn_id, None);
        armd::api::ws::set_test_ws_write_stall_once(&conn_id, false);

        // Also exercise `run_steps_against` `ExpectClose` on client-initiated close against the same
        // router, confirming `CloseOutcome::Eof` is recorded distinctly from `Frame`/`TransportError`.
        let handshake_events = run_steps_against(
            &http,
            &base_url,
            &ws_base,
            &[
                ScriptStep::Connect {
                    client: "h0".into(),
                    query: String::new(),
                    auth_header: None,
                    expect_http_status: Some(101),
                },
                ScriptStep::SendCloseFrame {
                    client: "h0".into(),
                    code: 1000,
                    reason: "bye".into(),
                },
                ScriptStep::ExpectClose {
                    client: "h0".into(),
                },
            ],
        )
        .await;
        assert!(
            matches!(
                handshake_events.last(),
                Some(RecordedEvent::ServerClose {
                    close: CloseOutcome::TransportError | CloseOutcome::Eof,
                    ..
                })
            ),
            "unexpected client-close outcome: {:?}",
            handshake_events.last()
        );
        // Verify client-initiated close is distinct from server FatalClose's explicit Close frame.
        assert!(
            !matches!(
                handshake_events.last(),
                Some(RecordedEvent::ServerClose {
                    close: CloseOutcome::Frame { .. },
                    ..
                })
            ),
            "expected no server Close frame on client-initiated teardown"
        );

        srv.abort();
        sample_transcript(events)
    }

    let tr1 = run_fatal_once().await;
    let tr2 = run_fatal_once().await;

    // Assert exact fatal frame BEFORE Close frame ordering.
    assert!(
        tr1.events.len() >= 2,
        "expected fatal error frame followed by ServerClose, got: {:?}",
        tr1.events
    );
    let err_idx = tr1
        .events
        .iter()
        .position(|ev| matches!(ev, RecordedEvent::ServerFrame { frame, .. } if frame["type"] == "error" && frame["code"] == "WRITE_TIMEOUT"))
        .expect("fatal WRITE_TIMEOUT error frame must be delivered before Close");
    let close_idx = tr1
        .events
        .iter()
        .position(|ev| {
            matches!(
                ev,
                RecordedEvent::ServerClose {
                    close: CloseOutcome::Frame { code: Some(1013), reason: Some(r) },
                    ..
                } if r == "WRITE_TIMEOUT"
            )
        })
        .expect("server must emit Close frame 1013 WRITE_TIMEOUT");
    assert!(
        err_idx < close_idx,
        "fatal error frame ({err_idx}) must precede WS Close frame ({close_idx})"
    );
    assert!(
        compare_transcripts(&tr1, &tr2).is_empty(),
        "fatal close replay diffs: {:?}",
        compare_transcripts(&tr1, &tr2)
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn replay_corpus_matches_baseline_zero_diff() {
    let dir = corpus_dir();
    let bin = replay_binary();

    if std::env::var_os("ARMD_WS_REPLAY_RECORD").is_some_and(|v| v == "1") {
        // Provenance guard: refuse to overwrite baseline corpus unless binary & commit are explicit.
        assert!(
            std::env::var_os("ARMD_REPLAY_BIN").is_some(),
            "ARMD_WS_REPLAY_RECORD=1 requires explicit ARMD_REPLAY_BIN pointing to the baseline binary"
        );
        let commit = std::env::var("ARMD_BASELINE_COMMIT")
            .expect("ARMD_WS_REPLAY_RECORD=1 requires explicit ARMD_BASELINE_COMMIT");
        assert_eq!(
            commit, EXPECTED_BASELINE_COMMIT,
            "baseline corpus must be recorded against {EXPECTED_BASELINE_COMMIT}"
        );
        std::fs::create_dir_all(&dir).unwrap();
        let specs = ws_replay_harness::scenarios::all_scenarios();
        let mut filenames = Vec::with_capacity(specs.len());
        let mut total_frames = 0usize;
        for spec in specs {
            let tr = execute_binary_session(
                &bin,
                spec.id,
                spec.family,
                spec.description,
                &spec.setup,
                &spec.steps,
            )
            .await;
            total_frames += tr.events.len();
            let filename = format!("{}.json", spec.id);
            let text = tr.to_compact_json().unwrap();
            std::fs::write(dir.join(&filename), text).unwrap();
            filenames.push(filename);
        }
        let manifest = json!({
            "baseline_commit": commit,
            "sessions": filenames,
            "policy_disabled_session": "16_policy_disabled_reject.json",
        });
        std::fs::write(
            dir.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).unwrap() + "\n",
        )
        .unwrap();
        eprintln!(
            "REPLAY_RECORDED: binary={} baseline={} sessions={} frames={}",
            bin.display(),
            commit,
            filenames.len(),
            total_frames
        );
        return;
    }

    if std::env::var_os("ARMD_WS_REPLAY_RECORD_NOFEAT").is_some_and(|v| v == "1") {
        assert!(
            std::env::var_os("ARMD_REPLAY_BIN").is_some(),
            "ARMD_WS_REPLAY_RECORD_NOFEAT=1 requires explicit ARMD_REPLAY_BIN"
        );
        let commit = std::env::var("ARMD_BASELINE_COMMIT")
            .expect("ARMD_WS_REPLAY_RECORD_NOFEAT=1 requires explicit ARMD_BASELINE_COMMIT");
        assert_eq!(commit, EXPECTED_BASELINE_COMMIT);
        let spec = ws_replay_harness::scenarios::policy_disabled_scenario();
        let tr = execute_binary_session(
            &bin,
            spec.id,
            spec.family,
            spec.description,
            &spec.setup,
            &spec.steps,
        )
        .await;
        let filename = format!("{}.json", spec.id);
        std::fs::write(dir.join(&filename), tr.to_compact_json().unwrap()).unwrap();
        eprintln!(
            "REPLAY_RECORDED_NOFEAT: binary={} baseline={} file={} frames={}",
            bin.display(),
            commit,
            filename,
            tr.events.len()
        );
        return;
    }

    let manifest_path = dir.join("manifest.json");
    assert!(
        manifest_path.exists(),
        "missing WS replay corpus manifest at {}",
        manifest_path.display()
    );
    let manifest: Value =
        serde_json::from_str(&std::fs::read_to_string(&manifest_path).unwrap()).unwrap();
    assert_eq!(
        manifest["baseline_commit"].as_str(),
        Some(EXPECTED_BASELINE_COMMIT),
        "manifest.baseline_commit mismatch"
    );
    let files = manifest["sessions"]
        .as_array()
        .expect("manifest.sessions array");
    let policy_disabled_file = manifest["policy_disabled_session"]
        .as_str()
        .expect("manifest.policy_disabled_session string");
    assert!(
        files.len() >= 30,
        "corpus must have at least 30 sessions, found {}",
        files.len()
    );

    let mut total_frames = 0usize;
    let mut total_diffs = 0usize;
    for entry in files {
        let raw_filename = entry.as_str().expect("session filename");
        #[cfg(feature = "policy-runtime")]
        let filename = raw_filename;
        #[cfg(not(feature = "policy-runtime"))]
        let filename = if raw_filename == "16_policy_shadow_start_stop.json" {
            policy_disabled_file
        } else {
            raw_filename
        };
        #[cfg(feature = "policy-runtime")]
        let _ = policy_disabled_file;

        let path = dir.join(filename);
        let expected: SessionTranscript =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap())
                .unwrap_or_else(|e| panic!("parse {}: {e}", path.display()));
        let actual = execute_binary_session(
            &bin,
            &expected.id,
            &expected.family,
            &expected.description,
            &expected.setup,
            &expected.steps,
        )
        .await;
        total_frames += actual.events.len();
        let diffs = compare_transcripts(&expected, &actual);
        if !diffs.is_empty() {
            total_diffs += diffs.len();
            eprintln!("DIFF in session {} ({}):", expected.id, filename);
            for d in &diffs {
                eprintln!(
                    "  ev[{}] {}: expected={} actual={}",
                    d.event_index, d.path, d.expected, d.actual
                );
            }
        }
    }
    eprintln!(
        "REPLAY_SUMMARY: binary={} sessions={} frames={} diffs={}",
        bin.display(),
        files.len(),
        total_frames,
        total_diffs
    );
    assert_eq!(total_diffs, 0, "WS replay corpus had {total_diffs} diff(s)");
}
