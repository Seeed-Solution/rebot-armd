//! Per-connection observation rate (`/api/v1/ws?obs_hz=N`).

mod common;

use std::time::{Duration, Instant};
use futures_util::StreamExt;
use tokio_tungstenite::connect_async;

use armd::ArmConfig;
use common::{ws_reply, TestSocket};

/// Observations received within `window`.
async fn count_observations(ws: &mut TestSocket, window: Duration) -> usize {
    let end = Instant::now() + window;
    let mut n = 0;
    while let Ok(Some(Ok(msg))) = tokio::time::timeout_at(end.into(), ws.next()).await {
        let v: serde_json::Value = serde_json::from_str(&msg.into_text().unwrap()).unwrap();
        if v.get("positions").is_some() { n += 1; }
    }
    n
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn obs_hz_decimates_observations_but_not_control_replies() {
    let (addr, supervisor) = common::spawn_server(ArmConfig::rebot_dm()).await;
    let (mut full, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let (mut slow, _) = connect_async(format!("ws://{addr}/api/v1/ws?obs_hz=5")).await.unwrap();
    let ticker = tokio::spawn(async move {
        // 50 Hz control rate for 1.2 s.
        for _ in 0..60 {
            supervisor.tick(0.02).await;
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    });
    let (n_full, n_slow) = tokio::join!(
        count_observations(&mut full, Duration::from_millis(1000)),
        count_observations(&mut slow, Duration::from_millis(1000)),
    );
    ticker.await.unwrap();
    assert!(n_full >= 30, "undecimated connection got {n_full}");
    assert!((3..=7).contains(&n_slow), "obs_hz=5 connection got {n_slow} in 1 s");

    // Lease replies are not decimated.
    let reply = ws_reply(&mut slow, serde_json::json!({"type":"acquire_lease", "client_id":"slow"}), &["lease_granted", "lease_rejected"]).await;
    assert_eq!(reply["type"], "lease_granted");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn obs_hz_is_clamped_and_validated() {
    let (addr, _supervisor) = common::spawn_server(ArmConfig::rebot_dm()).await;
    // 0 and values above the control rate are clamped, not refused.
    for q in ["0", "1000"] {
        assert!(connect_async(format!("ws://{addr}/api/v1/ws?obs_hz={q}")).await.is_ok(), "obs_hz={q}");
    }
    for q in ["fast", "-1", "2.5"] {
        match connect_async(format!("ws://{addr}/api/v1/ws?obs_hz={q}")).await {
            Err(tokio_tungstenite::tungstenite::Error::Http(r)) => {
                assert_eq!(r.status(), 400);
                let body = String::from_utf8(r.body().clone().unwrap_or_default()).unwrap();
                assert!(body.contains("integer >= 0 (0 and 1 mean 1 Hz"), "{body}");
            }
            other => panic!("obs_hz={q}: {other:?}"),
        }
    }
}
