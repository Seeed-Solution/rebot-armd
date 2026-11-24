//! Stage 3 PR L1 integration & zero-allocation test suite:
//! - Lease lifecycle (`acquire_lease`, `force: true` preemption, `release_lease`, disconnect)
//! - Connection-bound `heartbeat` omission rule, `epoch_sync`, expiry non-resurrection, Stop safety
//! - Per-session lock-free `OperatorGrant` live revocation/expiry and `MAX_OPERATOR_SESSIONS = 64`
//! - `MAX_LIVE_CONNECTIONS = 64` fail-closed cap, slot reclamation, unknown handle refusal
//! - Priority connection lifecycle disconnect under 100% full broker command ring + fencing
//! - Coherent `StateHub` readers (`observation`, `schema`, REST `heartbeat`, WS `?obs_hz=`) with
//!   zero `SupervisorLock` reads while legacy write guard is held, and `503` when executor exits
//! - Zero-alloc / zero-free RT proof under `TimedAlloc` with a live `tracing` subscriber enabled.

mod common;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::api::create_router;
use armd::auth::{OperatorAuth, OperatorGrant, UnlockError, MAX_OPERATOR_SESSIONS};
use armd::rt_core::broker::{
    AdmitError, Authority, Capacities, ClientId, Descriptor, ImmediateOp, Stamp, Submit,
};
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks, RtExecutorHandle};
use armd::rt_core::mailbox::TargetFrame;
use armd::rt_core::ops::{CommitCtx, ReleaseEngine};
use armd::rt_core::requests::lease::{
    apply_conn_lifecycle, commit_lease_descriptor, install_broker_heartbeat, install_lease_acquire,
    install_lease_disconnect, install_lease_preempt, prepare_lease_acquire, prepare_lease_preempt,
    ConnectionHandle, LeaseAcquire, LeaseDisconnect, LeaseInstallCtx, LeaseOutcome, LeaseRelease,
    LeaseRequest, LifecycleCmd,
};
use armd::supervisor::{MAX_LIVE_CONNECTIONS, OWNER_EXECUTOR};
use armd::{ArmConfig, ArmSupervisor, SupervisorInner};
use common::{next_of, send, ws_reply, TestSocket};

fn ensure_tracing_subscriber() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        let _ = tracing_subscriber::registry()
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(std::io::sink)
                    .with_ansi(false),
            )
            .try_init();
    });
}

async fn wait_until(timeout: Duration, mut pred: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if pred() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert!(pred(), "condition not satisfied within {timeout:?}");
}

async fn serve_with_auth(sup: Arc<ArmSupervisor>, auth: OperatorAuth) -> SocketAddr {
    sup.set_operator_auth_enforced(auth.is_enforced()).await;
    let router = create_router(sup, auth);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });
    addr
}

async fn setup_mode(
    cfg: ArmConfig,
    auth: OperatorAuth,
    executor_mode: bool,
) -> (Arc<ArmSupervisor>, SocketAddr, Option<RtExecutorHandle>) {
    let sup = Arc::new(ArmSupervisor::new(cfg));
    let addr = serve_with_auth(sup.clone(), auth).await;
    let handle = if executor_mode {
        let h = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
        Some(h)
    } else {
        None
    };
    (sup, addr, handle)
}

async fn connect_ws(addr: SocketAddr, query: &str) -> TestSocket {
    let url = if query.is_empty() {
        format!("ws://{addr}/api/v1/ws")
    } else {
        format!("ws://{addr}/api/v1/ws?{query}")
    };
    connect_async(url).await.unwrap().0
}

async fn next_non_telemetry_within(ws: &mut TestSocket, dur: Duration) -> Option<Value> {
    let deadline = tokio::time::Instant::now() + dur;
    loop {
        let rem = deadline.saturating_duration_since(tokio::time::Instant::now());
        if rem.is_zero() {
            return None;
        }
        match tokio::time::timeout(rem, ws.next()).await {
            Ok(Some(Ok(Message::Text(text)))) => {
                if let Ok(v) = serde_json::from_str::<Value>(&text) {
                    if v.get("type").is_some() {
                        return Some(v);
                    }
                }
            }
            Ok(Some(Ok(_))) => continue,
            _ => return None,
        }
    }
}

// ============================================================================
// 1. Two WS sockets with identical wire seq + lease generation isolation
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_two_sockets_identical_seq_and_lease_isolation_both_owners() {
    for executor_mode in [false, true] {
        let (sup, addr, handle) = setup_mode(
            ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            },
            OperatorAuth::disabled(),
            executor_mode,
        )
        .await;

        let mut ws1 = connect_ws(addr, "").await;
        let mut ws2 = connect_ws(addr, "").await;

        // Socket 1 acquires with seq=77.
        let r1 = ws_reply(
            &mut ws1,
            json!({"type": "acquire_lease", "client_id": "dup_label", "seq": 77}),
            &["lease_granted", "lease_rejected"],
        )
        .await;
        assert_eq!(r1["type"], "lease_granted", "mode={executor_mode} r1={r1}");
        assert_eq!(r1["seq"], 77);
        assert_eq!(r1["client_id"], "dup_label");
        let g1 = r1["generation"].as_u64().unwrap();
        assert!(g1 >= 1);

        // Socket 2 sends identical seq=77 and label -> controller_busy on ws2, ws1 untouched.
        let r2 = ws_reply(
            &mut ws2,
            json!({"type": "acquire_lease", "client_id": "dup_label", "seq": 77}),
            &["lease_granted", "lease_rejected"],
        )
        .await;
        assert_eq!(r2["type"], "lease_rejected", "mode={executor_mode} r2={r2}");
        assert_eq!(r2["error"], "controller_busy");
        assert_eq!(r2["seq"], 77);

        // Same socket re-acquire keeps exact generation g1.
        let r1_again = ws_reply(
            &mut ws1,
            json!({"type": "acquire_lease", "client_id": "dup_label_v2", "seq": 78}),
            &["lease_granted", "lease_rejected"],
        )
        .await;
        assert_eq!(r1_again["type"], "lease_granted");
        assert_eq!(r1_again["generation"], g1);
        assert_eq!(r1_again["seq"], 78);

        // Socket 2 trying to release ws1's generation fails without affecting ws1.
        let rel_foreign = ws_reply(
            &mut ws2,
            json!({"type": "release_lease", "generation": g1}),
            &["lease_released"],
        )
        .await;
        assert_eq!(rel_foreign["success"], false);

        // Socket 1 releasing wrong generation fails; releasing exact g1 succeeds.
        let rel_wrong = ws_reply(
            &mut ws1,
            json!({"type": "release_lease", "generation": g1 + 99}),
            &["lease_released"],
        )
        .await;
        assert_eq!(rel_wrong["success"], false);

        let rel_ok = ws_reply(
            &mut ws1,
            json!({"type": "release_lease", "generation": g1}),
            &["lease_released"],
        )
        .await;
        assert_eq!(rel_ok["success"], true);

        // Socket 2 can now acquire with the same seq=77 and gets a strictly newer generation.
        let r2_ok = ws_reply(
            &mut ws2,
            json!({"type": "acquire_lease", "client_id": "dup_label", "seq": 77}),
            &["lease_granted", "lease_rejected"],
        )
        .await;
        assert_eq!(r2_ok["type"], "lease_granted", "mode={executor_mode} r2_ok={r2_ok}");
        assert_eq!(r2_ok["seq"], 77);
        assert!(r2_ok["generation"].as_u64().unwrap() > g1);

        if executor_mode {
            assert_eq!(sup.inner().control_thread_attempts(), 0);
        }
        drop(ws1);
        drop(ws2);
        if let Some(h) = handle {
            h.stop_and_join();
        }
    }
}

// ============================================================================
// 2. Heartbeat omission rule, epoch_sync, Stop non-loss, expiry non-resurrection
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_heartbeat_omission_expiry_epoch_sync_and_stop_safety_both_owners() {
    for executor_mode in [false, true] {
        let (sup, addr, handle) = setup_mode(
            ArmConfig {
                lease_timeout_ms: 260,
                ..ArmConfig::rebot_dm()
            },
            OperatorAuth::disabled(),
            executor_mode,
        )
        .await;

        let mut ws = connect_ws(addr, "").await;

        // Before acquiring: heartbeat emits lease_lost with generation=0 and NO "reason" key.
        let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
        let pre_hb = ws_reply(
            &mut ws,
            json!({"type": "heartbeat", "epoch_id": ep}),
            &["lease_lost"],
        )
        .await;
        assert_eq!(pre_hb["generation"], 0, "mode={executor_mode} pre_hb={pre_hb}");
        assert!(
            pre_hb.get("reason").is_none(),
            "generation=0 lease_lost must omit reason: {pre_hb}"
        );

        // Acquire lease.
        let granted = ws_reply(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "hb_ctrl"}),
            &["lease_granted"],
        )
        .await;
        let gen = granted["generation"].as_u64().unwrap();

        // Connection-bound omission rule: omitting `generation` renews the lease past 260ms timeout.
        for _ in 0..4 {
            tokio::time::sleep(Duration::from_millis(90)).await;
            send(&mut ws, json!({"type": "heartbeat", "epoch_id": ep})).await;
        }
        let silent = next_non_telemetry_within(&mut ws, Duration::from_millis(80)).await;
        assert!(
            silent.is_none(),
            "mode={executor_mode}: omitted-generation heartbeat should renew silently, got {silent:?}"
        );

        // Wrong epoch_id emits epoch_sync with current epoch.
        let sync = ws_reply(
            &mut ws,
            json!({"type": "heartbeat", "epoch_id": ep + 999}),
            &["epoch_sync", "lease_lost"],
        )
        .await;
        assert_eq!(sync["type"], "epoch_sync", "mode={executor_mode} sync={sync}");
        assert_eq!(sync["epoch_id"], ep);

        // Mismatched non-zero generation or wrong client_id emits lease_lost with reason.
        let bad_gen = ws_reply(
            &mut ws,
            json!({"type": "heartbeat", "generation": gen + 7, "epoch_id": ep}),
            &["lease_lost"],
        )
        .await;
        assert_eq!(bad_gen["generation"], gen);
        assert!(bad_gen["reason"].is_string(), "{bad_gen}");

        let bad_label = ws_reply(
            &mut ws,
            json!({"type": "heartbeat", "client_id": 123, "epoch_id": ep}),
            &["lease_lost"],
        )
        .await;
        assert_eq!(bad_label["generation"], gen);
        assert!(bad_label["reason"].is_string(), "{bad_label}");

        // Urgent Stop MUST NOT cause a subsequent valid heartbeat to emit spurious lease_lost.
        let stop_ack = ws_reply(
            &mut ws,
            json!({
                "type": "command",
                "request_id": 1,
                "generation": gen,
                "epoch_id": ep,
                "path": "/api/v1/motion/stop",
                "body": {}
            }),
            &["command_ack", "command_reject"],
        )
        .await;
        assert_eq!(stop_ack["type"], "command_ack", "mode={executor_mode} stop={stop_ack}");
        send(&mut ws, json!({"type": "heartbeat", "generation": gen, "epoch_id": ep})).await;
        let post_stop = next_non_telemetry_within(&mut ws, Duration::from_millis(80)).await;
        assert!(
            post_stop.is_none(),
            "mode={executor_mode}: heartbeat after Stop must not emit spurious loss: {post_stop:?}"
        );

        // Let lease expire without heartbeats -> heartbeat emits lease_lost("expired") and cannot resurrect.
        tokio::time::sleep(Duration::from_millis(320)).await;
        let expired1 = ws_reply(
            &mut ws,
            json!({"type": "heartbeat", "generation": gen, "epoch_id": ep}),
            &["lease_lost"],
        )
        .await;
        assert_eq!(expired1["generation"], gen);
        assert_eq!(expired1["reason"], "expired", "mode={executor_mode} expired1={expired1}");

        // Second heartbeat still sees expired (never resurrected).
        let expired2 = ws_reply(
            &mut ws,
            json!({"type": "heartbeat", "generation": gen, "epoch_id": ep}),
            &["lease_lost"],
        )
        .await;
        assert_eq!(expired2["reason"], "expired");

        drop(ws);
        if let Some(h) = handle {
            h.stop_and_join();
        }
    }
}

// ============================================================================
// 3. OperatorGrant per-session isolation, 64-session cap, live revocation & cooldown
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_preemption_operator_grant_isolation_cooldown_and_cap_both_owners() {
    for executor_mode in [false, true] {
        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok_a, _) = auth.unlock("123456").unwrap();
        let (tok_b, _) = auth.unlock("123456").unwrap();

        // Fill remaining slots up to MAX_OPERATOR_SESSIONS (64); 65th must fail-closed.
        let mut extra_tokens = Vec::new();
        for _ in 2..MAX_OPERATOR_SESSIONS {
            extra_tokens.push(auth.unlock("123456").unwrap().0);
        }
        assert_eq!(auth.unlock("123456"), Err(UnlockError::SessionsFull));
        let (sup, addr, handle) = setup_mode(
            ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            },
            auth.clone(),
            executor_mode,
        )
        .await;

        // HTTP unlock also returns 503 OPERATOR_SESSIONS_FULL while full.
        let http = reqwest::Client::new();
        let full_resp = http
            .post(format!("http://{addr}/api/v1/auth/unlock"))
            .json(&json!({"pin": "123456"}))
            .send()
            .await
            .unwrap();
        assert_eq!(full_resp.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
        let full_body: Value = full_resp.json().await.unwrap();
        assert_eq!(full_body["code"], "OPERATOR_SESSIONS_FULL", "{full_body}");

        // Release one extra session -> unlock succeeds again (tok_c).
        auth.lock(extra_tokens.pop().as_deref());
        let (tok_c, _) = auth.unlock("123456").unwrap();

        // Holder acquires lease.
        let mut ws_holder = connect_ws(addr, "").await;
        let g1_reply = ws_reply(
            &mut ws_holder,
            json!({"type": "acquire_lease", "client_id": "holder"}),
            &["lease_granted"],
        )
        .await;
        let g1 = g1_reply["generation"].as_u64().unwrap();

        // Two operator sockets authenticate with independent sessions tok_a and tok_b.
        let mut ws_a = connect_ws(addr, "auth=explicit").await;
        let auth_a = ws_reply(
            &mut ws_a,
            json!({"type": "auth", "token": tok_a, "seq": 1}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_a["role"], "operator");

        let mut ws_b = connect_ws(addr, "auth=explicit").await;
        let auth_b = ws_reply(
            &mut ws_b,
            json!({"type": "auth", "token": tok_b, "seq": 1}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_b["role"], "operator");

        // Lock ONLY tok_a -> ws_a preemption rejected OPERATOR_REQUIRED; ws_b unaffected!
        auth.lock(Some(&tok_a));
        let rej_a = ws_reply(
            &mut ws_a,
            json!({"type": "acquire_lease", "client_id": "op_a", "force": true, "seq": 10}),
            &["lease_rejected", "lease_granted"],
        )
        .await;
        assert_eq!(rej_a["type"], "lease_rejected", "mode={executor_mode} rej_a={rej_a}");
        assert_eq!(rej_a["status"], 403);
        assert_eq!(rej_a["code"], "OPERATOR_REQUIRED");
        assert_eq!(rej_a["error"], "operator_required");
        assert_eq!(rej_a["seq"], 10);

        // ws_b (session tok_b) succeeds in preempting ws_holder.
        let ok_b = ws_reply(
            &mut ws_b,
            json!({"type": "acquire_lease", "client_id": "op_b", "force": true, "seq": 11}),
            &["lease_granted", "lease_rejected"],
        )
        .await;
        assert_eq!(ok_b["type"], "lease_granted", "mode={executor_mode} ok_b={ok_b}");
        assert_eq!(ok_b["seq"], 11);
        let g2 = ok_b["generation"].as_u64().unwrap();
        assert!(g2 > g1);

        // Evicted holder receives lease_lost(reason: "preempted", generation: g1).
        let evicted = next_of(&mut ws_holder, &["lease_lost"]).await;
        assert_eq!(evicted["generation"], g1);
        assert_eq!(evicted["reason"], "preempted");

        // Immediate second preemption from ws_c hits preemption_cooldown.
        let mut ws_c = connect_ws(addr, "auth=explicit").await;
        let _ = ws_reply(
            &mut ws_c,
            json!({"type": "auth", "token": tok_c, "seq": 1}),
            &["auth_ack"],
        )
        .await;
        let cd = ws_reply(
            &mut ws_c,
            json!({"type": "acquire_lease", "client_id": "op_c", "force": true, "seq": 12}),
            &["lease_rejected", "lease_granted"],
        )
        .await;
        assert_eq!(cd["type"], "lease_rejected", "mode={executor_mode} cd={cd}");
        assert_eq!(cd["error"], "preemption_cooldown");
        assert!(cd["remaining_ms"].as_u64().unwrap_or(0) >= 1);
        assert_eq!(cd["seq"], 12);

        drop(ws_holder);
        drop(ws_a);
        drop(ws_b);
        drop(ws_c);
        if let Some(h) = handle {
            h.stop_and_join();
        }
        drop(sup);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_preempt_grant_revoked_and_expired_while_queued_in_broker() {
    let auth = OperatorAuth::enforced("123456", 1800);
    let (tok_q, _) = auth.unlock("123456").unwrap();
    let (tok_live, _) = auth.unlock("123456").unwrap();

    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    }));
    sup.set_operator_auth_enforced(true).await;

    let delay = Arc::new(Mutex::new(None));
    let handle = sup
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(5)),
            PersistTestHooks {
                release_delay_once: Some(delay.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    // Register two connections and acquire lease on conn 101.
    let c1 = ConnectionHandle::from_ws_id(101).unwrap();
    let c2 = ConnectionHandle::from_ws_id(102).unwrap();
    sup.register_ws_connection(101).await.unwrap();
    sup.register_ws_connection(102).await.unwrap();
    let g1 = match sup.acquire_lease_ws(c1, "holder", 1).await.unwrap() {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };

    // Capture live grants BEFORE pausing a release.
    let grant_q = auth.operator_grant(Some(&tok_q));
    let grant_live = auth.operator_grant(Some(&tok_live));
    assert!(grant_q.is_valid_at(Instant::now(), true));
    assert!(grant_live.is_valid_at(Instant::now(), true));

    // Pause one release so `preempt_lease_ws` sits queued in the broker while we revoke `tok_q`.
    *delay.lock().unwrap() = Some(Duration::from_millis(80));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;

    let sup_clone = sup.clone();
    let preempt_task = tokio::spawn(async move {
        sup_clone.preempt_lease_ws(c2, "revoked_op", grant_q, 2).await
    });
    tokio::time::sleep(Duration::from_millis(10)).await;
    // Revoke ONLY tok_q before the paused release finishes and commits Step 3.
    auth.lock(Some(&tok_q));

    let out_q = preempt_task.await.unwrap().unwrap();
    assert_eq!(
        out_q,
        LeaseOutcome::OperatorRequired,
        "revoking session while queued must fail closed at RT commit"
    );
    // Holder lease g1 must remain intact.
    assert!(sup.fresh_snapshot(None).await.unwrap().owner.lease_generation == g1);

    // Concurrent live session `grant_live` is completely unaffected by locking `tok_q`.
    let out_live = sup.preempt_lease_ws(c2, "live_op", grant_live, 3).await.unwrap();
    assert!(
        matches!(
            out_live,
            LeaseOutcome::Granted {
                evicted_connection: Some(ev),
                evicted_generation,
                ..
            } if ev == c1 && evicted_generation == g1
        ),
        "live session grant must still succeed: {out_live:?}"
    );

    // Also verify TTL expiry while queued fails closed at commit.
    let short_auth = OperatorAuth::enforced("123456", 1);
    let (tok_exp, _) = short_auth.unlock("123456").unwrap();
    let grant_exp = short_auth.operator_grant(Some(&tok_exp));
    tokio::time::sleep(Duration::from_millis(1050)).await;
    let out_exp = sup.preempt_lease_ws(c1, "expired_op", grant_exp, 4).await.unwrap();
    assert_eq!(out_exp, LeaseOutcome::OperatorRequired);

    handle.stop_and_join();
}

// ============================================================================
// 4. MAX_LIVE_CONNECTIONS = 64 fail-closed cap, slot reclamation, unknown handle
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_max_live_connections_cap_and_reclamation_both_owners() {
    for executor_mode in [false, true] {
        let (sup, addr, handle) = setup_mode(
            ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            },
            OperatorAuth::disabled(),
            executor_mode,
        )
        .await;

        let mut sockets = Vec::with_capacity(MAX_LIVE_CONNECTIONS);
        for _ in 0..MAX_LIVE_CONNECTIONS {
            sockets.push(connect_ws(addr, "").await);
        }
        // Wait until all 64 WS handlers have completed `register_ws_connection`.
        wait_until(Duration::from_secs(3), || {
            if let Some(ref h) = handle {
                h.live_connection_slots() == MAX_LIVE_CONNECTIONS
            } else {
                sup.inner()
                    .try_read()
                    .is_ok_and(|g| g.live_connection_count() == MAX_LIVE_CONNECTIONS)
            }
        })
        .await;

        // 65th socket must be rejected fail-closed with 503 TOO_MANY_CONNECTIONS + Close(1013).
        let mut ws65 = connect_ws(addr, "").await;
        let err_frame = next_of(&mut ws65, &["error"]).await;
        assert_eq!(err_frame["status"], 503, "mode={executor_mode} err={err_frame}");
        assert_eq!(err_frame["code"], "TOO_MANY_CONNECTIONS");
        let close_msg = tokio::time::timeout(Duration::from_secs(2), ws65.next())
            .await
            .unwrap();
        assert!(
            matches!(close_msg, Some(Ok(Message::Close(Some(ref cf)))) if cf.code == 1013u16.into())
                || close_msg.is_none(),
            "expected Close(1013) on 65th socket, got {close_msg:?}"
        );

        // Close one socket -> slot reclaims -> new socket connects and acquires lease.
        let mut popped = sockets.pop().unwrap();
        let _ = popped.close(None).await;
        drop(popped);
        wait_until(Duration::from_secs(3), || {
            if let Some(ref h) = handle {
                h.live_connection_slots() == MAX_LIVE_CONNECTIONS - 1
            } else {
                sup.inner()
                    .try_read()
                    .is_ok_and(|g| g.live_connection_count() == MAX_LIVE_CONNECTIONS - 1)
            }
        })
        .await;

        let mut ws_new = connect_ws(addr, "").await;
        let granted = ws_reply(
            &mut ws_new,
            json!({"type": "acquire_lease", "client_id": "reclaimed"}),
            &["lease_granted", "lease_rejected"],
        )
        .await;
        assert_eq!(granted["type"], "lease_granted", "mode={executor_mode} granted={granted}");

        // Unknown / unregistered non-zero ConnectionHandle must be refused fail-closed.
        let forged = ConnectionHandle::from_ws_id(999_999).unwrap();
        let err = sup.acquire_lease_ws(forged, "forged", 1).await.unwrap_err();
        assert!(
            matches!(err, armd::ArmError::Rejected { status: 409, ref code, .. } if code == "DISCONNECTED"),
            "mode={executor_mode}: unregistered handle must fail closed DISCONNECTED, got {err:?}"
        );

        drop(ws_new);
        drop(sockets);
        if let Some(h) = handle {
            h.stop_and_join();
        }
    }
}

// ============================================================================
// 5. Full broker command ring disconnect before ACK + target revocation & reclamation
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_full_broker_ring_disconnect_before_ack_and_fencing() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    }));
    let conn_id = 501u64;
    let conn = ConnectionHandle::from_ws_id(conn_id).unwrap();
    // Register connection, acquire lease g1, enable torque, and authorize its servo stream
    // before handing ownership to the RT executor (proving a working authorized stream first).
    sup.register_ws_connection(conn_id).await.unwrap();
    let g1 = match sup.acquire_lease_ws(conn, "full_ring_holder", 1).await.unwrap() {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted, got {other:?}"),
    };
    {
        let mut inner = sup.inner().write().await;
        let s1 = inner.current_admission_stamp();
        let a1 = inner.check_admission(s1).unwrap();
        SupervisorInner::enable_torque(a1).unwrap();
        let s2 = inner.current_admission_stamp();
        let a2 = inner.check_admission(s2).unwrap();
        SupervisorInner::authorize_servo_stream(a2, g1).unwrap();
    }

    let delay = Arc::new(Mutex::new(None));
    let ecfg = ExecutorConfig {
        capacities: Capacities {
            commands: 4,
            plans: 2,
            replies: 8,
        },
        ..ExecutorConfig::for_tests(Duration::from_millis(5))
    };
    let handle = sup
        .start_executor_with_hooks(
            ecfg,
            PersistTestHooks {
                release_delay_once: Some(delay.clone()),
                ..Default::default()
            },
        )
        .await
        .unwrap();
    wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

    let servo_sid = armd::supervisor::targets::servo_stream(g1);
    let snap0 = sup.fresh_snapshot(None).await.unwrap();
    let now0 = Instant::now();
    let mut auth0 = snap0.owner.authority_at(now0);
    auth0.lease_generation = g1;
    let mut q_ok = [0.0f32; 8];
    for (dst, &src) in q_ok.iter_mut().zip(snap0.obs.commanded_positions.iter()).take(7) {
        *dst = src;
    }
    q_ok[0] += 0.01;
    handle.target_writer().lock().unwrap().submit(TargetFrame {
        arm: snap0.owner.arm,
        boot: snap0.owner.boot,
        stream: servo_sid,
        seq: 1,
        authority: auth0,
        policy_generation: 0,
        input_at: now0,
        vision_at: None,
        vision_frame_id: None,
        deadline: now0 + Duration::from_secs(5),
        dof: 7,
        q: q_ok,
        replaceable: true,
    });
    let mut saw_pre_ok = false;
    wait_until(Duration::from_secs(1), || {
        handle.target_writer().lock().unwrap().outcomes(|o| {
            if o.seq == 1 && o.result.is_ok() {
                saw_pre_ok = true;
            }
        });
        saw_pre_ok
    })
    .await;

    // Pause one release so we can fill the broker command ring 4/4 before the RT drains it.
    *delay.lock().unwrap() = Some(Duration::from_millis(90));
    wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;

    let snap = sup.fresh_snapshot(None).await.unwrap();
    let now = Instant::now();
    let mut auth = snap.owner.authority_at(now);
    auth.lease_generation = g1;
    let deadline = now + Duration::from_secs(5);

    // Fill all 4 broker command slots via `handle.submit_immediate_async` (managed by BrokerDispatcher).
    let mut pending_tasks = Vec::new();
    for seq in 1..=4u64 {
        let h = handle.clone();
        pending_tasks.push(tokio::spawn(async move {
            h.submit_immediate_async(ClientId(conn_id), seq, auth, deadline, ImmediateOp::Heartbeat)
                .await
        }));
    }
    wait_until(Duration::from_secs(1), || {
        handle.broker().lock().unwrap().outstanding() == 4
    })
    .await;

    // Confirm ordinary command ring is truly 100% full (`CommandsFull`).
    let overflow = handle.broker().lock().unwrap().submit(
        ClientId(conn_id),
        99,
        auth,
        deadline,
        Submit::Immediate(ImmediateOp::Heartbeat),
    );
    assert_eq!(overflow, Err(AdmitError::CommandsFull));

    // Submit a second TargetFrame (seq=2) on the same authorized stream that MUST be fenced/refused by disconnect.
    q_ok[0] += 0.01;
    let tgt_frame = TargetFrame {
        arm: snap.owner.arm,
        boot: snap.owner.boot,
        stream: servo_sid,
        seq: 2,
        authority: auth,
        policy_generation: 0,
        input_at: now,
        vision_at: None,
        vision_frame_id: None,
        deadline,
        dof: 7,
        q: q_ok,
        replaceable: true,
    };
    handle.target_writer().lock().unwrap().submit(tgt_frame);

    // Disconnect via priority lifecycle channel WHILE broker ring is 100% full!
    let outcome = sup.disconnect_ws_connection(conn).await;
    assert_eq!(
        outcome,
        LeaseOutcome::Disconnected {
            released: true,
            displaced_generation: g1,
        },
        "priority disconnect must succeed even when broker command ring is 100% full"
    );

    // Every pending task wakes cleanly with an error (retired on disconnect, no hang) and broker
    // outstanding() + live slots reclaim to 0.
    for t in pending_tasks {
        let res = tokio::time::timeout(Duration::from_secs(2), t)
            .await
            .unwrap()
            .unwrap();
        assert!(res.is_err(), "disconnected waiter must terminate with Err, got {res:?}");
    }
    wait_until(Duration::from_secs(2), || {
        handle.broker().lock().unwrap().outstanding() == 0 && handle.live_connection_slots() == 0
    })
    .await;

    // TargetFrame seq=2 stamped with displaced generation g1 was refused (stream revoked / lease lost).
    let mut refused_code: Option<&'static str> = None;
    wait_until(Duration::from_secs(1), || {
        handle.target_writer().lock().unwrap().outcomes(|o| {
            if o.seq == 2 {
                if let Err(r) = o.result {
                    refused_code = Some(r.code);
                }
            }
        });
        refused_code.is_some()
    })
    .await;
    assert!(
        matches!(refused_code, Some("LEASE_LOST" | "NO_LEASE" | "STREAM_REVOKED")),
        "expected lease/stream revocation refusal for seq=2, got {refused_code:?}"
    );

    // Published snapshot shows no active lease and fenced generation >= g1.
    let after = sup.fresh_snapshot(None).await.unwrap();
    assert!(!after.owner.is_lease_active_at(Instant::now()));
    assert!(
        after.owner.fenced_lease_generation >= g1,
        "fenced_lease_generation ({}) must be >= g1 ({g1})",
        after.owner.fenced_lease_generation
    );

    // A fresh connection can immediately register and acquire g2 > g1.
    let conn2 = ConnectionHandle::from_ws_id(502).unwrap();
    sup.register_ws_connection(502).await.unwrap();
    let g2 = match sup.acquire_lease_ws(conn2, "successor", 1).await.unwrap() {
        LeaseOutcome::Granted { generation, .. } => generation,
        other => panic!("expected Granted for successor, got {other:?}"),
    };
    assert!(g2 > g1);

    handle.stop_and_join();
}

// ============================================================================
// 6. Coherent StateHub readers with zero SupervisorLock reads + 503 on exit
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_state_hub_readers_coherence_and_zero_supervisor_lock() {
    let (sup, addr, handle) = setup_mode(
        ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        },
        OperatorAuth::disabled(),
        true,
    )
    .await;
    let handle = handle.unwrap();
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

    let http = reqwest::Client::new();

    // Hold the detached legacy write lock exclusively to PROVE zero SupervisorLock reads occur.
    let legacy_write_guard = sup.inner().write().await;

    // 1. GET /api/v1/observation -> 200 OK from StateHub.
    let obs_resp = http.get(format!("http://{addr}/api/v1/observation")).send().await.unwrap();
    assert_eq!(obs_resp.status(), reqwest::StatusCode::OK);
    let obs_json: Value = obs_resp.json().await.unwrap();
    assert!(obs_json["epoch_id"].as_u64().unwrap_or(0) >= 1);

    // 2. GET /api/v1/observation/schema -> 200 OK from StateHub schema cache.
    let schema_resp = http
        .get(format!("http://{addr}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap();
    assert_eq!(schema_resp.status(), reqwest::StatusCode::OK);
    let schema_json: Value = schema_resp.json().await.unwrap();
    assert_eq!(schema_json["dof"], 7, "{schema_json}");

    // 3. POST /api/v1/heartbeat (no active lease) -> 409 LEASE_REQUIRED from StateHub.
    let hb_no_lease = http
        .post(format!("http://{addr}/api/v1/heartbeat"))
        .json(&json!({"client_id": "rest_client", "epoch_id": 1}))
        .send()
        .await
        .unwrap();
    assert_eq!(hb_no_lease.status(), reqwest::StatusCode::CONFLICT);
    let hb_body: Value = hb_no_lease.json().await.unwrap();
    assert_eq!(hb_body["code"], "LEASE_REQUIRED", "{hb_body}");

    // 4. WS upgrade with ?obs_hz=10 -> succeeds using StateHub control_frequency_hz; acquire lease.
    let mut ws = connect_ws(addr, "obs_hz=10").await;
    let granted = ws_reply(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "reader_test"}),
        &["lease_granted"],
    )
    .await;
    assert!(granted["generation"].as_u64().unwrap() >= 1);

    // 5. POST /api/v1/heartbeat (with active lease) -> 409 LEASE_ACTIVE from StateHub.
    let hb_active = http
        .post(format!("http://{addr}/api/v1/heartbeat"))
        .json(&json!({"client_id": "rest_client", "epoch_id": 1}))
        .send()
        .await
        .unwrap();
    assert_eq!(hb_active.status(), reqwest::StatusCode::CONFLICT);
    let hb_active_body: Value = hb_active.json().await.unwrap();
    assert_eq!(hb_active_body["code"], "LEASE_ACTIVE", "{hb_active_body}");

    assert_eq!(sup.inner().control_thread_attempts(), 0);
    drop(ws);
    drop(legacy_write_guard);

    // Stop executor -> all StateHub readers must fail closed with 503 OBSERVATION_UNAVAILABLE.
    handle.stop_and_join();

    let obs_dead = http.get(format!("http://{addr}/api/v1/observation")).send().await.unwrap();
    assert_eq!(obs_dead.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(obs_dead.json::<Value>().await.unwrap()["code"], "OBSERVATION_UNAVAILABLE");

    let schema_dead = http
        .get(format!("http://{addr}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap();
    assert_eq!(schema_dead.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(schema_dead.json::<Value>().await.unwrap()["code"], "OBSERVATION_UNAVAILABLE");

    let hb_dead = http
        .post(format!("http://{addr}/api/v1/heartbeat"))
        .json(&json!({"client_id": "rest_client", "epoch_id": 1}))
        .send()
        .await
        .unwrap();
    assert_eq!(hb_dead.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(hb_dead.json::<Value>().await.unwrap()["code"], "OBSERVATION_UNAVAILABLE");

    // WS upgrade with `?obs_hz=10` fails closed with 503 when executor is stopped.
    let ws_obs_hz_err = connect_async(format!("ws://{addr}/api/v1/ws?obs_hz=10"))
        .await
        .unwrap_err();
    match ws_obs_hz_err {
        tokio_tungstenite::tungstenite::Error::Http(resp) => {
            assert_eq!(resp.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
        }
        other => panic!("expected HTTP 503 on ?obs_hz=10 with stopped executor, got {other:?}"),
    }
}

// ============================================================================
// 7. Zero-alloc / zero-free RT proof with live tracing subscriber enabled
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_alloc_rt_l1_paths_with_tracing_subscriber_enabled() {
    ensure_tracing_subscriber();

    // Positive control: prove TimedAlloc actually measures heap allocations on this thread.
    armd::diagnostics::alloc::time_this_thread();
    let _ = armd::diagnostics::alloc::take();
    let probe: Vec<u8> = vec![0xAB; 256];
    std::hint::black_box(&probe);
    drop(probe);
    let ctrl = armd::diagnostics::alloc::take();
    assert!(ctrl.calls > 0, "positive control failed: TimedAlloc must count allocations");

    // (a) Direct RT-path zero-alloc / zero-free proof across every L1 owner operation.
    {
        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok, _) = auth.unlock("123456").unwrap();
        let grant_ok = auth.operator_grant(Some(&tok));
        let grant_denied = OperatorGrant::DENIED;

        let mut core = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 100,
            ..ArmConfig::rebot_dm()
        });
        core.rt_executor_owned = true;
        core.operator_auth_enforced = true;
        core.prewarm_rt_buffers();
        // Warm all 3 StateHub observation slots before measuring.
        for g in 1..=4 {
            let _ = core.tick_publish(0.005, g, false);
            let _ = core.core.hub().latest();
        }

        let now = Instant::now();
        let deadline = now + Duration::from_secs(5);
        let c1 = ConnectionHandle::from_ws_id(1).unwrap();
        let c2 = ConnectionHandle::from_ws_id(2).unwrap();
        let long_label = "L".repeat(120);
        let snap0 = core.core.hub().latest().unwrap().owner;
        let p_acq1 = prepare_lease_acquire(&snap0, c1, &long_label, 1, now, deadline).unwrap();
        let p_acq2 = prepare_lease_acquire(&snap0, c2, "rival", 2, now, deadline).unwrap();
        let p_pre_denied =
            prepare_lease_preempt(&snap0, c2, "denied", grant_denied, 3, now, deadline).unwrap();
        let p_pre_ok =
            prepare_lease_preempt(&snap0, c2, &long_label, grant_ok, 4, now, deadline).unwrap();
        let p_pre_cd =
            prepare_lease_preempt(&snap0, c1, "cd_check", grant_ok, 5, now, deadline).unwrap();
        let arm = snap0.arm;
        let boot = snap0.boot;

        let mk_ctx = |req_id: u64, client: ClientId, at: Instant| LeaseInstallCtx {
            expected_arm: arm,
            expected_boot: boot,
            stamp_arm: arm,
            stamp_boot: boot,
            now: at,
            request_id: req_id,
            client,
            deadline: at + Duration::from_secs(5),
            enforce_single_use: true,
        };

        // Pre-allocate engine + lifecycle channel BEFORE measuring RT zero-alloc window.
        let (mut eng, _b, _tw, _st) = ReleaseEngine::new(boot, Capacities::default());
        let (_lc_client, lc_intake) =
            armd::rt_core::requests::lease::conn_lifecycle_channel(Arc::new(AtomicBool::new(false)));
        eng.attach_conn_lifecycle(lc_intake);

        let _ = armd::diagnostics::alloc::take();

        // 1. Open both connections.
        assert!(matches!(
            apply_conn_lifecycle(&mut core, LifecycleCmd::Open { seq: 1, conn_id: 1 }, now),
            armd::rt_core::requests::lease::LifecycleReply::Opened { result: Ok(()), .. }
        ));
        assert!(matches!(
            apply_conn_lifecycle(&mut core, LifecycleCmd::Open { seq: 2, conn_id: 2 }, now),
            armd::rt_core::requests::lease::LifecycleReply::Opened { result: Ok(()), .. }
        ));
        assert_eq!(armd::diagnostics::alloc::take().calls, 0, "lifecycle open allocated");

        // 2. Acquire on c1, busy on c2, re-acquire on c1.
        let g1 = match install_lease_acquire(&mut core, &p_acq1.request, &mk_ctx(10, ClientId(1), now)) {
            Ok(LeaseOutcome::Granted { generation, .. }) => generation,
            other => panic!("unexpected: {other:?}"),
        };
        assert!(matches!(
            install_lease_acquire(&mut core, &p_acq2.request, &mk_ctx(11, ClientId(2), now)),
            Ok(LeaseOutcome::Busy { .. })
        ));
        assert!(matches!(
            install_lease_acquire(&mut core, &p_acq1.request, &mk_ctx(12, ClientId(1), now)),
            Ok(LeaseOutcome::Granted { generation, .. }) if generation == g1
        ));
        assert_eq!(armd::diagnostics::alloc::take().calls, 0, "lease acquire allocated");

        // 3. Broker heartbeat renewal + epoch mismatch refusal.
        let mut hb_auth = Authority {
            lease_generation: g1,
            epoch: core.safety.epoch_id(),
            stop_generation: 0,
            config_revision: core.workspace_revision,
            parent_generation: 0,
        };
        let hb_desc = Descriptor {
            stamp: Stamp {
                arm,
                boot,
                request: armd::rt_core::broker::RequestId(1),
                client: ClientId(1),
                client_seq: 1,
                authority: hb_auth,
                deadline: now + Duration::from_secs(5),
            },
            kind: armd::rt_core::broker::CommandKind::Immediate(ImmediateOp::Heartbeat),
        };
        let cctx = CommitCtx {
            arm,
            boot,
            now,
            stop_generation: 0,
        };
        assert_eq!(install_broker_heartbeat(&mut core, &hb_desc, &cctx), Ok(()));
        hb_auth.epoch = 999_999;
        let hb_bad_ep = Descriptor {
            stamp: Stamp {
                authority: hb_auth,
                ..hb_desc.stamp
            },
            kind: armd::rt_core::broker::CommandKind::Immediate(ImmediateOp::Heartbeat),
        };
        assert_eq!(
            install_broker_heartbeat(&mut core, &hb_bad_ep, &cctx),
            Err("EPOCH_MISMATCH")
        );
        assert_eq!(armd::diagnostics::alloc::take().calls, 0, "broker heartbeat allocated");

        // 4. Denied preempt, valid preempt (with 120B label + eviction string recycling), cooldown preempt.
        assert_eq!(
            install_lease_preempt(&mut core, &p_pre_denied.request, &mk_ctx(13, ClientId(2), now)),
            Ok(LeaseOutcome::OperatorRequired)
        );
        let g2 = match install_lease_preempt(&mut core, &p_pre_ok.request, &mk_ctx(14, ClientId(2), now)) {
            Ok(LeaseOutcome::Granted {
                generation,
                evicted_connection: Some(ev),
                evicted_generation,
            }) if ev == c1 && evicted_generation == g1 => generation,
            other => panic!("unexpected preempt: {other:?}"),
        };
        assert!(matches!(
            install_lease_preempt(&mut core, &p_pre_cd.request, &mk_ctx(15, ClientId(1), now)),
            Ok(LeaseOutcome::Cooldown { .. })
        ));
        assert_eq!(armd::diagnostics::alloc::take().calls, 0, "lease preempt allocated");

        // 5. Commit lease via broker descriptor path (Release) + Disconnect via lifecycle.
        let rel_desc = Descriptor {
            stamp: Stamp {
                arm,
                boot,
                request: armd::rt_core::broker::RequestId(2),
                client: ClientId(2),
                client_seq: 1,
                authority: Authority {
                    lease_generation: g2,
                    epoch: core.safety.epoch_id(),
                    stop_generation: 0,
                    config_revision: core.workspace_revision,
                    parent_generation: 0,
                },
                deadline: now + Duration::from_secs(5),
            },
            kind: armd::rt_core::broker::CommandKind::Lease(LeaseRequest::Release(LeaseRelease {
                connection: c2,
                generation: g2,
            })),
        };
        assert!(matches!(
            commit_lease_descriptor(
                &mut core,
                &rel_desc,
                LeaseRequest::Release(LeaseRelease {
                    connection: c2,
                    generation: g2
                }),
                &cctx
            ),
            Ok(LeaseOutcome::Released { released: true, .. })
        ));
        assert_eq!(armd::diagnostics::alloc::take().calls, 0, "lease release allocated");

        // 6. Acquire again on c1, tick control + publish, then disconnect c1 & c2.
        let g3 = match install_lease_acquire(
            &mut core,
            &LeaseAcquire {
                connection: c1,
                label: armd::model::HolderLabel::new("reacquire"),
                has_label: true,
            },
            &mk_ctx(16, ClientId(1), now),
        ) {
            Ok(LeaseOutcome::Granted { generation, .. }) => generation,
            other => panic!("unexpected: {other:?}"),
        };
        let ts = core.tick_control(0.005);
        let _ = core.publish_at(ts, 5, false);
        assert!(matches!(
            install_lease_disconnect(&mut core, &LeaseDisconnect { connection: c1 }, now),
            LeaseOutcome::Disconnected {
                released: true,
                displaced_generation,
            } if displaced_generation == g3
        ));
        assert!(matches!(
            apply_conn_lifecycle(
                &mut core,
                LifecycleCmd::Disconnect {
                    seq: 3,
                    connection: c2
                },
                now
            ),
            armd::rt_core::requests::lease::LifecycleReply::Disconnected { .. }
        ));

        // Unregistered / closed handle refusal also 0 allocs.
        assert_eq!(
            install_lease_acquire(&mut core, &p_acq1.request, &mk_ctx(17, ClientId(1), now)),
            Err("DISCONNECTED")
        );

        // Also test ReleaseEngine service_wake with lifecycle intake (0 allocs).
        let mut rep = armd::rt_core::ops::ReleaseReport::default();
        let _ = eng.service_wake(&mut core, now, now + Duration::from_millis(5), 6, &mut rep);

        let stats = armd::diagnostics::alloc::take();
        assert_eq!(
            stats.calls, 0,
            "RT L1 disconnect + tick + service_wake with tracing subscriber enabled must do 0 allocs/frees: {stats:?}"
        );
    }

    // (b) Live `armd-control` whole-release zero-alloc proof (`alloc_timing = true`) across WS
    // acquire, heartbeat, preemption, release, and disconnect.
    let auth = OperatorAuth::enforced("123456", 1800);
    let (tok_op, _) = auth.unlock("123456").unwrap();
    let sup = Arc::new(ArmSupervisor::new(ArmConfig {
        lease_timeout_ms: 5_000,
        ..ArmConfig::rebot_dm()
    }));
    let addr = serve_with_auth(sup.clone(), auth).await;

    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(5));
    ecfg.alloc_timing = true;
    let handle = sup.start_executor(ecfg).await.unwrap();

    // Warm up all 3 StateHub slots before sampling.
    wait_until(Duration::from_secs(2), || sup.tick_samples().len() >= 4).await;
    let _ = sup.fresh_snapshot(None).await.unwrap();
    wait_until(Duration::from_secs(2), || sup.tick_samples().len() >= 10).await;
    let baseline_samples = sup.tick_samples().len();

    let mut ws1 = connect_ws(addr, "").await;
    let mut ws2 = connect_ws(addr, "auth=explicit").await;
    let _ = ws_reply(
        &mut ws2,
        json!({"type": "auth", "token": tok_op, "seq": 1}),
        &["auth_ack"],
    )
    .await;

    let g1 = ws_reply(
        &mut ws1,
        json!({"type": "acquire_lease", "client_id": "alloc_ctrl_1", "seq": 1}),
        &["lease_granted"],
    )
    .await["generation"]
        .as_u64()
        .unwrap();
    let ep = sup.fresh_snapshot(None).await.unwrap().obs.epoch_id;
    send(&mut ws1, json!({"type": "heartbeat", "generation": g1, "epoch_id": ep})).await;
    tokio::time::sleep(Duration::from_millis(25)).await;

    let g2 = ws_reply(
        &mut ws2,
        json!({"type": "acquire_lease", "client_id": "alloc_ctrl_2", "force": true, "seq": 2}),
        &["lease_granted"],
    )
    .await["generation"]
        .as_u64()
        .unwrap();
    let _ = next_of(&mut ws1, &["lease_lost"]).await;

    let rel = ws_reply(
        &mut ws2,
        json!({"type": "release_lease", "generation": g2}),
        &["lease_released"],
    )
    .await;
    assert_eq!(rel["success"], true);

    let _ = ws1.close(None).await;
    let _ = ws2.close(None).await;
    drop(ws1);
    drop(ws2);
    wait_until(Duration::from_secs(2), || handle.live_connection_slots() == 0).await;

    let end_step = handle.stats().steps.load(Ordering::Relaxed);
    wait_until(Duration::from_secs(2), || {
        handle.stats().steps.load(Ordering::Relaxed) >= end_step + 4
    })
    .await;

    let samples = sup.tick_samples();
    let nonzero: Vec<(usize, u32)> = samples[baseline_samples..]
        .iter()
        .enumerate()
        .filter(|(_, s)| s.usage.lock_alloc_calls > 0)
        .map(|(i, s)| (i, s.usage.lock_alloc_calls))
        .collect();
    assert!(
        nonzero.is_empty(),
        "live executor L1 releases must do 0 allocs/frees (baseline={baseline_samples}, total={}): {nonzero:?}",
        samples.len()
    );
    assert_eq!(sup.inner().control_thread_attempts(), 0);
    handle.stop_and_join();
}

// ============================================================================
// 8. L1 review regressions:
//    - Expired lease stops active servo motion even when servo recently refreshed last_heartbeat
//    - Newly upgraded WS socket latches urgent E-stop immediately while executor is stalled
//    - Transferred active lease cleared on RT does 0 frees; 100 churn cycles on ClosedConnections do 0 allocs
//    - Evictee still receives lease_lost(reason: "preempted") when preemptor disconnects immediately
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn l1_review_regressions_expiry_motion_stalled_estop_churn_transfer_free_and_eviction() {
    ensure_tracing_subscriber();

    // (a) Blocker #1: expired lease retires active servo target even if `last_heartbeat` was just refreshed.
    {
        let mut core = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 80,
            ..ArmConfig::rebot_dm()
        });
        let g = core.acquire_lease("connection:exp-servo").unwrap();
        let stamp = core.current_admission_stamp();
        let admitted = core.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(admitted).unwrap();
        let target = vec![0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        // Sleep almost to expiry, submit valid servo frame (refreshes last_heartbeat without renewing lease).
        std::thread::sleep(Duration::from_millis(60));
        let ep = core.safety.epoch_id();
        let s2 = core.current_admission_stamp();
        let a2 = core.check_admission(s2).unwrap();
        SupervisorInner::servo(a2, target, ep).unwrap();
        assert!(core.servo_target.is_some());
        // Cross lease_timeout (80ms) while last_heartbeat is only ~35ms old (< 80ms watchdog!).
        std::thread::sleep(Duration::from_millis(35));
        core.tick(0.02);
        assert!(
            core.servo_target.is_none(),
            "expired lease must retire active servo target immediately at Step 0.0 fence (gen={g})"
        );
    }

    // (b) Blocker #2: newly upgraded WS socket latches urgent E-stop immediately while executor is stalled.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        }));
        let addr = serve_with_auth(sup.clone(), OperatorAuth::disabled()).await;
        let delay = Arc::new(Mutex::new(None));
        let handle = sup
            .start_executor_with_hooks(
                ExecutorConfig::for_tests(Duration::from_millis(5)),
                PersistTestHooks {
                    release_delay_once: Some(delay.clone()),
                    ..Default::default()
                },
            )
            .await
            .unwrap();
        wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

        // Stall the RT release loop for 400ms so owner-side `Open` cannot complete during this window.
        *delay.lock().unwrap() = Some(Duration::from_millis(400));
        wait_until(Duration::from_secs(1), || delay.lock().unwrap().is_none()).await;

        let t0 = Instant::now();
        let mut ws_fresh = connect_ws(addr, "").await;
        send(&mut ws_fresh, json!({"type": "estop"})).await;
        wait_until(Duration::from_millis(120), || {
            sup.e_stop_latched.load(Ordering::SeqCst)
        })
        .await;
        assert!(
            t0.elapsed() < Duration::from_millis(180),
            "fresh WS socket must latch E-stop without waiting for stalled executor Open: {:?}",
            t0.elapsed()
        );
        drop(ws_fresh);
        handle.stop_and_join();
    }

    // (c) Major #2 + Major #3: transferred active lease cleared on RT does 0 frees, AND 100 churn cycles
    //     with 63 open connections + gaps on ClosedConnections do 0 RT allocs/frees!
    {
        armd::diagnostics::alloc::time_this_thread();
        let mut core = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        });
        // Acquire a lease BEFORE handoff (legacy phase), then prewarm for RT transfer!
        let _g0 = core.acquire_lease("connection:transferred-holder").unwrap();
        core.rt_executor_owned = true;
        core.prewarm_rt_buffers();
        for id in 1..=63u64 {
            core.register_live_connection(id).unwrap();
        }
        let now = Instant::now();
        let _ = armd::diagnostics::alloc::take();

        // Clear the transferred active lease on RT -> MUST do 0 frees / 0 allocs!
        let _ = install_lease_disconnect(
            &mut core,
            &LeaseDisconnect {
                connection: ConnectionHandle::try_new(0, "connection:transferred-holder").unwrap(),
            },
            now,
        );
        assert_eq!(
            armd::diagnostics::alloc::take().calls,
            0,
            "clearing transferred active lease on RT must do 0 allocs/frees"
        );

        // 100 churn cycles of accept 64th (with non-contiguous IDs!) then close 64th -> 0 allocs!
        for k in 0..100u64 {
            let id = 100 + k * 2; // deliberate gaps every cycle
            core.register_live_connection(id).unwrap();
            core.mark_connection_closed(id);
        }
        assert_eq!(
            armd::diagnostics::alloc::take().calls,
            0,
            "100 connection churn cycles with gaps must do 0 RT allocs/frees"
        );
    }

    // (d) Major #8: committed preemption still delivers lease_lost(reason: "preempted") to evictee
    //     even if the preemptor immediately disconnects!
    {
        let (sup, addr, handle) = setup_mode(
            ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            },
            OperatorAuth::disabled(),
            true,
        )
        .await;
        let handle = handle.unwrap();
        let mut ws_holder = connect_ws(addr, "").await;
        let g1 = ws_reply(
            &mut ws_holder,
            json!({"type": "acquire_lease", "client_id": "victim"}),
            &["lease_granted"],
        )
        .await["generation"]
            .as_u64()
            .unwrap();

        let mut ws_preemptor = connect_ws(addr, "").await;
        // Wait for preemptor's slot to finish owner registration before preempting & dropping immediately.
        wait_until(Duration::from_secs(2), || handle.live_connection_slots() == 2).await;
        send(
            &mut ws_preemptor,
            json!({"type": "acquire_lease", "client_id": "hit_and_run", "force": true}),
        )
        .await;
        // Wait briefly for the preempt request to reach the RT executor, then drop preemptor socket!
        tokio::time::sleep(Duration::from_millis(15)).await;
        drop(ws_preemptor);

        let lost = next_of(&mut ws_holder, &["lease_lost"]).await;
        assert_eq!(lost["generation"], g1);
        assert_eq!(lost["reason"], "preempted");

        drop(ws_holder);
        handle.stop_and_join();
        drop(sup);
    }
}
