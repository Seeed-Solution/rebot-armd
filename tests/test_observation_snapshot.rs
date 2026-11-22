//! `ArmSupervisor::get_observation` serves the control tick's snapshot without the
//! supervisor lock, and only while that snapshot is the current state.

use std::sync::Arc;
use std::time::Duration;

use armd::{ArmConfig, ArmSupervisor};

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn observation_is_served_while_a_writer_waits_behind_a_reader() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let tick_obs = sup.tick(0.02).await;
    // A slow reader holds the lock and a writer (the next control tick) queues behind it:
    // tokio's RwLock then blocks new readers, as on the device.
    let reader = sup.inner().read().await;
    let writer = {
        let sup = sup.clone();
        tokio::spawn(async move { sup.inner().write().await.temperatures = vec![41.0; 7]; })
    };
    tokio::time::sleep(Duration::from_millis(5)).await;
    let obs = tokio::time::timeout(Duration::from_millis(30), sup.get_observation())
        .await
        .expect("observation must not wait for the supervisor lock");
    assert_eq!(obs.timestamp, tick_obs.timestamp);
    drop(reader);
    writer.await.unwrap();
    let snap = sup.tick_stats();
    assert_eq!((snap.observation_cache.hits, snap.observation_cache.misses), (1, 0));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_write_after_the_tick_is_visible_at_once() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    sup.tick(0.02).await;
    sup.set_temperatures(vec![55.0; 7]).await;
    assert_eq!(sup.get_observation().await.temperatures, vec![55.0; 7]);
    sup.tick(0.02).await;
    assert_eq!(sup.get_observation().await.temperatures, vec![55.0; 7]);
    let c = sup.tick_stats().observation_cache;
    assert_eq!((c.hits, c.misses), (1, 1));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_snapshot_older_than_two_periods_is_recomputed() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    let tick_obs = sup.tick(0.02).await;
    // 50 Hz: two periods are 40 ms.
    tokio::time::sleep(Duration::from_millis(60)).await;
    let obs = sup.get_observation().await;
    assert!(obs.timestamp >= tick_obs.timestamp + 0.05, "{} vs {}", obs.timestamp, tick_obs.timestamp);
    assert_eq!(sup.tick_stats().observation_cache.misses, 1);
}
