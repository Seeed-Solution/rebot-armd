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

// ---------------------------------------------------------------------------
// Saving must not hold the supervisor lock across the disk write: the control
// tick takes the same lock, so a slow write would stall control cycles.
// ---------------------------------------------------------------------------

fn recording_supervisor() -> std::sync::Arc<armd::ArmSupervisor> {
    let config = armd::ArmConfig {
        dof: 2,
        joint_names: vec!["joint_1".to_string(), "joint_2".to_string()],
        limits: vec![
            armd::JointLimit::new("joint_1", -2.0, 2.0, 2.0, 5.0),
            armd::JointLimit::new("joint_2", -2.0, 2.0, 2.0, 5.0),
        ],
        home_positions: vec![0.0, 0.0],
        ..Default::default()
    };
    std::sync::Arc::new(armd::ArmSupervisor::new(config))
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_slow_save_does_not_block_control_tick() {
    use std::time::{Duration, Instant};

    let dir = tempdir().unwrap();
    let path = dir.path().join("slow.parquet");
    let supervisor = recording_supervisor();
    supervisor.start_recording(Some("slow".into()), Some(path.clone())).await.unwrap();
    for _ in 0..5 {
        supervisor.tick(0.02).await;
    }

    let saver = supervisor.clone();
    let save = tokio::spawn(async move {
        saver
            .save_recording_with(|p, frames| {
                std::thread::sleep(Duration::from_millis(200));
                write_frames_to_parquet(p, frames)
            })
            .await
    });

    // Let the save reach the writer, then tick while it sleeps.
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert_eq!(supervisor.get_recording_state().await.0, EpisodeState::Finalizing);
    let mut worst = Duration::ZERO;
    for _ in 0..5 {
        let t = Instant::now();
        supervisor.tick(0.02).await;
        worst = worst.max(t.elapsed());
    }
    assert!(!save.is_finished(), "writer should still be sleeping while ticks run");
    assert!(worst < Duration::from_millis(50), "tick blocked by save for {worst:?}");

    let saved = save.await.unwrap().unwrap();
    assert_eq!(saved, path);
    let (state, count) = supervisor.get_recording_state().await;
    assert_eq!(state, EpisodeState::Committed);
    // Ticks during Finalizing are not recorded, matching the old locked save.
    assert_eq!(count, 5);
    assert_eq!(read_frames_from_parquet(&path).unwrap().len(), 5);
}

#[tokio::test]
async fn test_failed_save_leaves_finalizing() {
    let dir = tempdir().unwrap();
    let supervisor = recording_supervisor();
    supervisor
        .start_recording(Some("fail".into()), Some(dir.path().join("fail.parquet")))
        .await
        .unwrap();
    supervisor.tick(0.02).await;

    let err = supervisor
        .save_recording_with(|_, _| Err(armd::recording::ParquetRecordingError::EmptyFrames))
        .await;
    assert!(err.is_err());
    assert_eq!(supervisor.get_recording_state().await.0, EpisodeState::Discarded);
    // A new recording can start after the failure.
    supervisor.start_recording(Some("next".into()), None).await.unwrap();
}

#[tokio::test]
async fn test_panicking_writer_does_not_wedge_recorder() {
    let supervisor = recording_supervisor();
    supervisor.start_recording(Some("panic".into()), None).await.unwrap();
    supervisor.tick(0.02).await;

    let err = supervisor.save_recording_with(|_, _| panic!("writer bug")).await;
    assert!(err.is_err());
    assert_eq!(supervisor.get_recording_state().await.0, EpisodeState::Discarded);
}
