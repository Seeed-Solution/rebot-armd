use tempfile::tempdir;
use armd::{
    read_frames_from_parquet, write_frames_to_parquet, EpisodeRecorder, EpisodeState, RecordFrame,
};

#[test]
fn test_parquet_writer_and_reader_roundtrip() {
    let dir = tempdir().unwrap();
    let file_path = dir.path().join("test_episode.parquet");

    let original_frames = vec![
        RecordFrame {
            timestamp: 0.0,
            observation_state: vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            action: vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        },
        RecordFrame {
            timestamp: 0.02,
            observation_state: vec![0.11, 0.21, 0.31, 0.41, 0.51, 0.61],
            action: vec![0.15, 0.25, 0.35, 0.45, 0.55, 0.65],
        },
        RecordFrame {
            timestamp: 0.04,
            observation_state: vec![0.12, 0.22, 0.32, 0.42, 0.52, 0.62],
            action: vec![0.20, 0.30, 0.40, 0.50, 0.60, 0.70],
        },
    ];

    // Write to Parquet
    write_frames_to_parquet(&file_path, &original_frames).unwrap();
    assert!(file_path.exists());

    // Read back and verify
    let read_frames = read_frames_from_parquet(&file_path).unwrap();
    assert_eq!(read_frames.len(), 3);

    for (orig, read) in original_frames.iter().zip(read_frames.iter()) {
        assert!((orig.timestamp - read.timestamp).abs() < 1e-6);
        assert_eq!(orig.observation_state.len(), read.observation_state.len());
        assert_eq!(orig.action.len(), read.action.len());

        for (v1, v2) in orig.observation_state.iter().zip(read.observation_state.iter()) {
            assert!((v1 - v2).abs() < 1e-5);
        }
        for (a1, a2) in orig.action.iter().zip(read.action.iter()) {
            assert!((a1 - a2).abs() < 1e-5);
        }
    }
}

#[test]
fn test_episode_recorder_state_machine_commit() {
    let dir = tempdir().unwrap();
    let file_path = dir.path().join("committed_episode.parquet");

    let mut recorder = EpisodeRecorder::new();
    assert_eq!(*recorder.state(), EpisodeState::Idle);

    // Cannot save or discard while Idle
    assert!(recorder.save().is_err());
    assert!(recorder.discard().is_err());

    // Start recording
    recorder
        .start(Some("ep_001".into()), Some(file_path.clone()))
        .unwrap();
    assert_eq!(*recorder.state(), EpisodeState::Recording);
    assert_eq!(recorder.episode_id(), "ep_001");
    assert_eq!(recorder.frame_count(), 0);

    // Append frames
    recorder.append_frame(RecordFrame {
        timestamp: 1.0,
        observation_state: vec![0.0, 0.1],
        action: vec![0.05, 0.15],
    });
    recorder.append_frame(RecordFrame {
        timestamp: 1.02,
        observation_state: vec![0.01, 0.11],
        action: vec![0.06, 0.16],
    });
    assert_eq!(recorder.frame_count(), 2);

    // Save
    let saved_path = recorder.save().unwrap();
    assert_eq!(saved_path, file_path);
    assert_eq!(*recorder.state(), EpisodeState::Committed);
    assert!(file_path.exists());

    // Read back committed parquet
    let read_back = read_frames_from_parquet(&file_path).unwrap();
    assert_eq!(read_back.len(), 2);
}

#[test]
fn test_episode_recorder_discard() {
    let dir = tempdir().unwrap();
    let file_path = dir.path().join("discarded_episode.parquet");

    let mut recorder = EpisodeRecorder::new();
    recorder
        .start(Some("ep_discard".into()), Some(file_path.clone()))
        .unwrap();

    recorder.append_frame(RecordFrame {
        timestamp: 0.1,
        observation_state: vec![1.0],
        action: vec![2.0],
    });

    recorder.discard().unwrap();
    assert_eq!(*recorder.state(), EpisodeState::Discarded);
    assert_eq!(recorder.frame_count(), 0);
    assert!(!file_path.exists());
}
