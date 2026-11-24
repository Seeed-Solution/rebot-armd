//! A1b PR 1 WS replay baseline tests:
//! 1. Strictness / negative-control tests for the equivalence-preserving transcript comparator.
//! 2. Controlled-clock & injected owner-event in-process replay test (deterministic `tick(dt)`,
//!    exact `?obs_hz=N` telemetry pacing, direct owner E-stop injection).
//! 3. Full corpus replay against `ARMD_REPLAY_BIN` (defaults to `CARGO_BIN_EXE_armd`) with 0-diff
//!    verification against `tests/ws_replay/`.

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
    compare_transcripts, CloseInfo, RecordedEvent, SessionSetup, SessionTranscript,
};
use ws_replay_harness::runner::execute_binary_session;

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
                "safety_state": "Armed",
                "is_torqued": true,
                "control_mode": "position",
                "pose_id": "01KXYZ11111111111111111111",
                "saved_at_ms": 1700000001000u64
            }),
        },
        RecordedEvent::ActionTerminal {
            client: "c0".into(),
            action_id: 100,
            kind: "joint_move".into(),
            state: "done".into(),
            code: None,
            safety_state: json!("Armed"),
            epoch_id: 10,
            is_torqued: true,
            control_mode: "position".into(),
        },
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({"type": "epoch_sync", "epoch_id": 11, "safety_state": "EStop"}),
        },
        RecordedEvent::ServerClose {
            client: "c0".into(),
            close: CloseInfo {
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
                "safety_state": "Armed",
                "is_torqued": true,
                "control_mode": "position",
                "pose_id": "01KABC22222222222222222222",
                "saved_at_ms": 1799999999000u64
            }),
        },
        RecordedEvent::ActionTerminal {
            client: "c0".into(),
            action_id: 999,
            kind: "joint_move".into(),
            state: "done".into(),
            code: None,
            safety_state: json!("Armed"),
            epoch_id: 42,
            is_torqued: true,
            control_mode: "position".into(),
        },
        RecordedEvent::ServerFrame {
            client: "c0".into(),
            frame: json!({"type": "epoch_sync", "epoch_id": 45, "safety_state": "EStop"}),
        },
        RecordedEvent::ServerClose {
            client: "c0".into(),
            close: CloseInfo {
                code: Some(1000),
                reason: Some("normal".into()),
            },
        },
    ]);
    assert!(
        compare_transcripts(&base, &shifted).is_empty(),
        "bijective shift must produce 0 diffs: {:?}",
        compare_transcripts(&base, &shifted)
    );

    let mut bad_action = shifted.clone();
    if let RecordedEvent::ActionTerminal { action_id, .. } = &mut bad_action.events[2] {
        *action_id = 1000;
    }
    assert!(!compare_transcripts(&base, &bad_action).is_empty());

    let mut bad_epoch_eq = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_epoch_eq.events[3] {
        frame["epoch_id"] = json!(42);
    }
    assert!(!compare_transcripts(&base, &bad_epoch_eq).is_empty());

    let mut bad_epoch_order = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_epoch_order.events[3] {
        frame["epoch_id"] = json!(40);
    }
    assert!(!compare_transcripts(&base, &bad_epoch_order).is_empty());

    let mut bad_gen_zero = shifted.clone();
    if let RecordedEvent::ServerFrame { frame, .. } = &mut bad_gen_zero.events[0] {
        frame["generation"] = json!(0);
    }
    assert!(!compare_transcripts(&base, &bad_gen_zero).is_empty());

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

    let mut bad_close = shifted.clone();
    if let RecordedEvent::ServerClose { close, .. } = &mut bad_close.events[4] {
        close.reason = Some("ESTOP_LATCHED:WRITE_TIMEOUT".into());
    }
    assert!(!compare_transcripts(&base, &bad_close).is_empty());

    // STOPPED cancellation normalizes Moving/Armed tick boundary but rejects Fault/EStop/Idle
    // and non-STOPPED cancellations.
    let stopped_moving = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action_id: 1,
        kind: "joint_move".into(),
        state: "cancelled".into(),
        code: Some("STOPPED".into()),
        safety_state: json!("Moving"),
        epoch_id: 1,
        is_torqued: true,
        control_mode: "position".into(),
    }]);
    let stopped_armed = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action_id: 2,
        kind: "joint_move".into(),
        state: "cancelled".into(),
        code: Some("STOPPED".into()),
        safety_state: json!("Armed"),
        epoch_id: 1,
        is_torqued: true,
        control_mode: "position".into(),
    }]);
    assert!(compare_transcripts(&stopped_moving, &stopped_armed).is_empty());

    let stopped_fault = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action_id: 2,
        kind: "joint_move".into(),
        state: "cancelled".into(),
        code: Some("STOPPED".into()),
        safety_state: json!("Fault"),
        epoch_id: 1,
        is_torqued: true,
        control_mode: "position".into(),
    }]);
    assert!(!compare_transcripts(&stopped_moving, &stopped_fault).is_empty());

    let estop_moving = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action_id: 1,
        kind: "joint_move".into(),
        state: "cancelled".into(),
        code: Some("ESTOP".into()),
        safety_state: json!("Moving"),
        epoch_id: 2,
        is_torqued: true,
        control_mode: "position".into(),
    }]);
    let estop_armed = sample_transcript(vec![RecordedEvent::ActionTerminal {
        client: "c0".into(),
        action_id: 1,
        kind: "joint_move".into(),
        state: "cancelled".into(),
        code: Some("ESTOP".into()),
        safety_state: json!("Armed"),
        epoch_id: 2,
        is_torqued: true,
        control_mode: "position".into(),
    }]);
    assert!(!compare_transcripts(&estop_moving, &estop_armed).is_empty());
}

/// Controlled-clock in-process replay: drives `ArmSupervisor::tick(0.02)` manually (no wall-clock
/// control loop) to verify exact `?obs_hz=5` decimation, injected owner E-stop, and transcript
/// comparator parity across two identical runs.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn controlled_clock_and_injected_owner_event_replay_parity() {
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

        let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws?obs_hz=5"))
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

        for _ in 0..25 {
            sup.tick(0.02).await;
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        let mut obs_count = 0usize;
        let mut last_ts = None;
        let mut monotonic = true;
        let mut last_epoch = 0u64;
        while let Ok(Some(Ok(Message::Text(t)))) =
            tokio::time::timeout(Duration::from_millis(25), ws.next()).await
        {
            let v: Value = serde_json::from_str(&t).unwrap();
            if v.get("type").is_none() {
                obs_count += 1;
                if let Some(ts) = v.get("timestamp").and_then(Value::as_f64) {
                    if last_ts.is_some_and(|p| ts < p) {
                        monotonic = false;
                    }
                    last_ts = Some(ts);
                }
                last_epoch = v["epoch_id"].as_u64().unwrap_or(0);
            }
        }
        assert!(
            (1..=4).contains(&obs_count),
            "5 Hz decimation over 0.5 s virtual time yielded {obs_count} obs"
        );
        events.push(RecordedEvent::TelemetrySummary {
            client: "c0".into(),
            in_range: true,
            monotonic,
            dof: 7,
            safety_state: json!("Idle"),
            epoch_id: last_epoch,
        });

        let old_epoch = sup.get_observation().await.epoch_id;
        let (st, new_epoch) = sup.trigger_e_stop().await;
        events.push(RecordedEvent::OwnerEvent {
            label: "direct_owner_estop".into(),
            detail: json!({"safety_state": st, "epoch_id": new_epoch}),
        });
        assert!(new_epoch > old_epoch);

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

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn replay_corpus_matches_baseline_zero_diff() {
    let dir = corpus_dir();
    let bin = replay_binary();
    if std::env::var_os("ARMD_WS_REPLAY_RECORD").is_some_and(|v| v == "1") {
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
            "baseline_commit": "0afeb1d",
            "sessions": filenames,
        });
        std::fs::write(
            dir.join("manifest.json"),
            serde_json::to_string_pretty(&manifest).unwrap() + "\n",
        )
        .unwrap();
        eprintln!(
            "REPLAY_RECORDED: binary={} sessions={} frames={}",
            bin.display(),
            filenames.len(),
            total_frames
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
    let files = manifest["sessions"]
        .as_array()
        .expect("manifest.sessions array");
    assert!(
        files.len() >= 30,
        "corpus must have at least 30 sessions, found {}",
        files.len()
    );
    let mut total_frames = 0usize;
    let mut total_diffs = 0usize;
    for entry in files {
        let filename = entry.as_str().expect("session filename");
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
