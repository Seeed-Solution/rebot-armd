//! A1b PR 1 WS replay baseline tests: equivalence-preserving transcript comparator unit tests.

mod ws_replay_harness;

use serde_json::json;

use ws_replay_harness::comparator::{
    compare_transcripts, CloseInfo, RecordedEvent, SessionSetup, SessionTranscript,
};

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
}
