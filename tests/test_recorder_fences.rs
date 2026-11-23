//! Recorder start/stop fences (armd 1.6 RT core, track E): an episode holds exactly the ticks
//! between its start and stop fence, and samples the tick dropped on a full recording ring
//! (a stalled consumer up to the stop) are counted in the sidecar.
use std::sync::Arc;

use armd::rt_core::record_ring::RECORD_RING_CAPACITY;
use armd::teach::{RecorderOwner, Stopped, Stopper};
use armd::{ArmConfig, ArmSupervisor};

const DT: f64 = 0.02;

async fn supervisor(dir: &std::path::Path) -> Arc<ArmSupervisor> {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() }));
    sup.set_episodes_dir(Some(dir.to_path_buf())).await;
    sup
}

async fn save(sup: &ArmSupervisor) -> Box<armd::recording::store::EpisodeSidecar> {
    match sup.stop_recording_as(Stopper::Owner(RecorderOwner::LegacyRest), true, Some("ok"), None).await.unwrap() {
        Stopped::Saved { sidecar, .. } => sidecar,
        Stopped::Discarded => panic!("discarded"),
    }
}

#[tokio::test]
async fn an_episode_holds_exactly_the_ticks_between_its_fences() {
    let tmp = tempfile::tempdir().unwrap();
    let sup = supervisor(tmp.path()).await;
    for _ in 0..15 { sup.tick(DT).await; }
    sup.start_recording(None, None).await.unwrap();
    for _ in 0..37 { sup.tick(DT).await; }
    let sidecar = save(&sup).await;
    assert_eq!((sidecar.frames, sidecar.frames_dropped), (37, None), "{sidecar:?}");
    // no session after the stop fence: later ticks go nowhere
    for _ in 0..10 { sup.tick(DT).await; }
    assert!(!sup.inner().read().await.core.recording_active());
    // the next session starts empty
    sup.start_recording(None, None).await.unwrap();
    for _ in 0..3 { sup.tick(DT).await; }
    let second = save(&sup).await;
    assert_eq!((second.frames, second.frames_dropped), (3, None));
}

#[tokio::test]
async fn overflow_up_to_the_stop_is_counted_and_kept_in_the_sidecar() {
    let tmp = tempfile::tempdir().unwrap();
    let sup = supervisor(tmp.path()).await;
    sup.start_recording(None, None).await.unwrap();
    let ring = sup.inner().read().await.core.record_ring().unwrap().clone();
    let extra = 44;
    {
        // the consumer stalls for the whole session: the ring fills, the tick drops and counts
        let _stalled = ring.consumer_guard();
        for _ in 0..RECORD_RING_CAPACITY + extra { sup.tick(DT).await; }
        let inner = sup.inner().read().await;
        assert_eq!((inner.core.record_pushed(), inner.core.record_dropped()), (RECORD_RING_CAPACITY, extra as u64));
    }
    let sidecar = save(&sup).await;
    assert_eq!(sidecar.frames, RECORD_RING_CAPACITY, "{sidecar:?}");
    assert_eq!(sidecar.frames_dropped, Some(extra as u64), "the episode is marked incomplete");
    assert_eq!(sidecar.frames as u64 + sidecar.frames_dropped.unwrap(), (RECORD_RING_CAPACITY + extra) as u64, "every tick between the fences");
    // the listed episode keeps the count
    let listed = sup.episode_store().await.list_episodes(None, None, None, &[]);
    assert!(listed.iter().any(|e| e.sidecar.id == sidecar.id && e.sidecar.frames_dropped == Some(extra as u64)), "{listed:?}");
}
