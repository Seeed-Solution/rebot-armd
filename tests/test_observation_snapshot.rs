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

/// Shadow compare: the published sample equals `current_observation()` computed under the
/// lock at the same state, field by field (time-derived fields aside), across states.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn published_sample_matches_the_locked_observation_field_by_field() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    let compare = |label: &str, published: armd::ArmObservation, locked: armd::ArmObservation| {
        let mut a = serde_json::to_value(&published).unwrap();
        let mut b = serde_json::to_value(&locked).unwrap();
        for v in [&mut a, &mut b] {
            let m = v.as_object_mut().unwrap();
            m.remove("timestamp");
        }
        let (a, b) = (a.as_object().unwrap(), b.as_object().unwrap());
        let keys: std::collections::BTreeSet<_> = a.keys().chain(b.keys()).collect();
        for k in keys {
            assert_eq!(a.get(k), b.get(k), "{label}: field {k}");
        }
    };
    let check = |label: &'static str| {
        let sup = &sup;
        async move {
            sup.tick(0.02).await;
            let published = sup.get_observation().await;
            let locked = sup.inner().read().await.current_observation();
            (label, published, locked)
        }
    };
    let (l, p, c) = check("idle").await;
    compare(l, p, c);
    sup.inner().write().await.lease_enabled = false;
    sup.set_torque(true).await.unwrap();
    let (l, p, c) = check("torqued").await;
    compare(l, p, c);
    let mut target = sup.get_config().await.home_positions.clone();
    target[0] += 0.2;
    sup.jog(sup.capture_admission_stamp(), target, Some(1.0), None).await.unwrap();
    for i in 0..5 {
        let (l, p, c) = check(if i == 0 { "jog started" } else { "jog running" }).await;
        assert!(p.action.is_some());
        compare(l, p, c);
    }
    sup.set_temperatures(vec![44.0; 7]).await;
    let (l, p, c) = check("after a write").await;
    assert_eq!(p.temperatures, vec![44.0; 7]);
    compare(l, p, c);
    let c = sup.tick_stats().observation_cache;
    assert_eq!(c.misses, 0, "{c:?}");
}

/// Expiry: the previous-action window closes without a write; a sample published inside it
/// is not served after it closes (the observation is computed again).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn previous_action_window_expires_like_the_locked_observation() {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    sup.tick(0.02).await;
    {
        let mut inner = sup.inner().write().await;
        let now = inner.current_observation().timestamp;
        let a = inner.last_action.clone().unwrap_or_else(|| serde_json::from_value(serde_json::json!({
            "id": 1, "kind": "home", "state": "done", "progress": 1.0, "started_at": 0.0})).unwrap());
        inner.previous_action = Some((a, now + 0.015));
    }
    let obs = sup.tick(0.02).await;
    assert!(obs.previous_action.is_some());
    assert!(sup.get_observation().await.previous_action.is_some());
    tokio::time::sleep(Duration::from_millis(25)).await;
    let later = sup.get_observation().await;
    assert!(later.previous_action.is_none());
    assert_eq!(later.previous_action, sup.inner().read().await.current_observation().previous_action);
}
