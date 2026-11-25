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
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

use armd::api::create_router;
use armd::auth::{OperatorAuth, OperatorGrant, UnlockError, MAX_OPERATOR_SESSIONS};
use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueOutcome,
    TorqueProgress as DrvTorqueProgress,
};
use armd::model::JointState;
use armd::rt_core::broker::{
    AdmitError, Authority, BootId, Capacities, ClientId, Descriptor, ImmediateOp, Outcome, Reply,
    Stamp, Submit, TransitionOp,
};
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks, RtExecutorHandle};
use armd::rt_core::mailbox::TargetFrame;
use armd::rt_core::ops::{
    CommitCtx, CoreOps, ReleaseEngine, ReleaseFixture, ServiceBudget, VirtualClock,
};
use armd::rt_core::requests::lease::{
    apply_conn_lifecycle, check_torque_owner_preflight, commit_lease_descriptor,
    conn_lifecycle_channel, install_broker_heartbeat, install_lease_acquire,
    install_lease_clear_fault_rt, install_lease_disconnect, install_lease_preempt,
    prepare_lease_acquire, prepare_lease_preempt, ConnectionHandle, LeaseAcquire,
    LeaseClearFaultOutcome, LeaseDisconnect, LeaseInstallCtx, LeaseOutcome, LeaseRelease,
    LeaseRequest, LifecycleCmd, OwnerAuthPolicy, TorquePreflightError,
};
use armd::supervisor::{MAX_LIVE_CONNECTIONS, OWNER_EXECUTOR};
use armd::transport::{DriverClock, MockClock, TransportError};
use armd::{ArmConfig, ArmSupervisor, SafetyState, SupervisorInner};
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

// ============================================================================
// Milestone L2: TorqueOn / TorqueOff transitions, BackgroundDisableRetry,
//               in-flight enable abort, WS clear_fault, REST lease gates,
//               and L2 zero-alloc RT proof
// ============================================================================

struct L2DriverControl {
    allow_enable_confirm: AtomicBool,
    allow_disable_confirm: AtomicBool,
    fail_enable_poll: AtomicBool,
    /// Remaining calls to `begin_torque_budgeted(false, 0)` that return `Err(WouldBlock)`.
    fail_disable_begin_remaining: AtomicUsize,
    /// When true, `begin_torque_budgeted(false, 0)` returns `Done(Unconfirmed)` instead of confirming.
    disable_always_unconfirmed: AtomicBool,
    /// When true, starting enable leaves `enable_state()` in `Unknown` (partial enable) until a
    /// disable attempt actually succeeds and confirms disabled.
    partial_enable_unknown_until_disable: AtomicBool,
    begin_enable_calls: AtomicUsize,
    begin_disable_calls: AtomicUsize,
    poll_enable_calls: AtomicUsize,
    set_torque_blocking_calls: AtomicUsize,
    abort_enable_calls: AtomicUsize,
    write_targets_calls: AtomicUsize,
}

impl Default for L2DriverControl {
    fn default() -> Self {
        Self {
            allow_enable_confirm: AtomicBool::new(true),
            allow_disable_confirm: AtomicBool::new(true),
            fail_enable_poll: AtomicBool::new(false),
            fail_disable_begin_remaining: AtomicUsize::new(0),
            disable_always_unconfirmed: AtomicBool::new(false),
            partial_enable_unknown_until_disable: AtomicBool::new(false),
            begin_enable_calls: AtomicUsize::new(0),
            begin_disable_calls: AtomicUsize::new(0),
            poll_enable_calls: AtomicUsize::new(0),
            set_torque_blocking_calls: AtomicUsize::new(0),
            abort_enable_calls: AtomicUsize::new(0),
            write_targets_calls: AtomicUsize::new(0),
        }
    }
}

struct L2MockDriver {
    dof: usize,
    positions: Vec<f32>,
    enabled: bool,
    partial_unknown: bool,
    pending: Option<bool>,
    clock: Option<MockClock>,
    ctrl: Arc<L2DriverControl>,
}

impl L2MockDriver {
    fn new(cfg: &ArmConfig, clock: Option<MockClock>, ctrl: Arc<L2DriverControl>) -> Self {
        Self {
            dof: cfg.dof,
            positions: cfg.resting_positions().to_vec(),
            enabled: false,
            partial_unknown: false,
            pending: None,
            clock,
            ctrl,
        }
    }

    fn now(&self) -> Instant {
        self.clock.as_ref().map_or_else(Instant::now, DriverClock::now)
    }
}

impl ArmDriver for L2MockDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            ..DriverCapabilities::default()
        }
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self
            .positions
            .iter()
            .map(|&p| JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            })
            .collect())
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        out.clear();
        for &p in &self.positions {
            out.push(JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            });
        }
        Ok((self.dof as u16, false))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        for &p in &self.positions {
            out.push(JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 30.0,
            });
        }
        Some(())
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.ctrl.write_targets_calls.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.ctrl.set_torque_blocking_calls.fetch_add(1, Ordering::SeqCst);
        self.enabled = enable;
        Ok(())
    }

    fn begin_torque_budgeted(
        &mut self,
        enable: bool,
        _max_rx_frames: u16,
    ) -> Result<DrvTorqueProgress, ArmDriverError> {
        if enable {
            self.ctrl.begin_enable_calls.fetch_add(1, Ordering::SeqCst);
            if self.ctrl.partial_enable_unknown_until_disable.load(Ordering::SeqCst) {
                self.partial_unknown = true;
            }
            self.pending = Some(true);
            return Ok(DrvTorqueProgress::Pending {
                wake: self.now() + Duration::from_millis(2),
            });
        }
        self.ctrl.begin_disable_calls.fetch_add(1, Ordering::SeqCst);
        self.pending = None;
        let rem = self.ctrl.fail_disable_begin_remaining.load(Ordering::SeqCst);
        if rem > 0 {
            let _ = self.ctrl.fail_disable_begin_remaining.compare_exchange(
                rem,
                rem - 1,
                Ordering::SeqCst,
                Ordering::SeqCst,
            );
            return Err(ArmDriverError::Transport(TransportError::WouldBlock));
        }
        if self.ctrl.disable_always_unconfirmed.load(Ordering::SeqCst) {
            return Ok(DrvTorqueProgress::Done(TorqueOutcome::Unconfirmed));
        }
        if self.ctrl.allow_disable_confirm.load(Ordering::SeqCst) {
            self.enabled = false;
            self.partial_unknown = false;
            return Ok(DrvTorqueProgress::Done(TorqueOutcome::Confirmed));
        }
        self.pending = Some(false);
        Ok(DrvTorqueProgress::Pending {
            wake: self.now() + Duration::from_millis(2),
        })
    }

    fn poll_torque_budgeted(
        &mut self,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<DrvTorqueProgress, ArmDriverError> {
        match self.pending {
            Some(true) => {
                self.ctrl.poll_enable_calls.fetch_add(1, Ordering::SeqCst);
                if self.ctrl.fail_enable_poll.load(Ordering::SeqCst) {
                    self.pending = None;
                    return Err(ArmDriverError::Timeout(1));
                }
                if self.ctrl.allow_enable_confirm.load(Ordering::SeqCst) {
                    self.pending = None;
                    self.enabled = true;
                    return Ok(DrvTorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                Ok(DrvTorqueProgress::Pending {
                    wake: self.now() + Duration::from_millis(5),
                })
            }
            Some(false) => {
                if self.ctrl.allow_disable_confirm.load(Ordering::SeqCst) {
                    self.pending = None;
                    self.enabled = false;
                    self.partial_unknown = false;
                    return Ok(DrvTorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                Ok(DrvTorqueProgress::Pending {
                    wake: self.now() + Duration::from_millis(5),
                })
            }
            None => Err(ArmDriverError::Timeout(0)),
        }
    }

    fn abort_enable(&mut self) {
        if self.pending == Some(true) {
            self.pending = None;
            self.ctrl.abort_enable_calls.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn torque_transition_pending(&self) -> bool {
        self.pending.is_some()
    }

    fn clock_now(&self) -> Instant {
        self.now()
    }

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        true
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }

    fn enable_state(&self) -> EnableState {
        if self.partial_unknown {
            EnableState::Unknown
        } else if self.enabled {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }

    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        dof == self.dof && Duration::from_millis(1) <= max_age
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::from_millis(1)); self.dof]
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        for _ in 0..self.dof {
            out.push(Some(Duration::from_millis(1)));
        }
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        for _ in 0..self.dof {
            out.push(Some(1));
        }
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        vec![Some(self.now()); self.dof]
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let t = self.now();
        out.clear();
        for _ in 0..self.dof {
            out.push(Some(t));
        }
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let t = self.now();
        out.clear();
        for _ in 0..self.dof {
            out.push(Some(t));
        }
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        vec![1; self.dof]
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        for _ in 0..self.dof {
            out.push(1);
        }
    }

    fn fresh_joint_positions_into(
        &self,
        joints: &[usize],
        max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        if Duration::from_millis(1) > max_age {
            return false;
        }
        for (dst, &j) in out.iter_mut().zip(joints.iter()) {
            let Some(&p) = self.positions.get(j) else {
                return false;
            };
            *dst = p;
        }
        true
    }
}

// ============================================================================
// 9. L2: WS /api/v1/torque & /api/v1/arm across BOTH owners + delayed hardware confirm
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_torque_arm_and_delayed_hardware_confirmation_both_owners() {
    ensure_tracing_subscriber();

    // (a) Driverless mock: parity across legacy and executor owners.
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

        let mut ws = connect_ws(addr, "").await;
        let g = ws_reply(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "torque_ctrl", "seq": 1}),
            &["lease_granted"],
        )
        .await["generation"]
            .as_u64()
            .unwrap();
        let ep = sup.observation().await.unwrap().epoch_id;

        for (i, (path, enable, want_torqued, want_state)) in [
            ("/api/v1/torque", true, true, "Armed"),
            ("/api/v1/torque", false, false, "Idle"),
            ("/api/v1/arm", true, true, "Armed"),
            ("/api/v1/arm", false, false, "Idle"),
        ]
        .into_iter()
        .enumerate()
        {
            let req_id = (i + 1) as u64;
            let ack = ws_reply(
                &mut ws,
                json!({"type": "command", "request_id": req_id, "generation": g, "epoch_id": ep, "path": path, "body": {"enable": enable}}),
                &["command_ack", "command_reject"],
            )
            .await;
            assert_eq!(ack["type"], "command_ack", "mode={executor_mode} ack={ack}");
            assert_eq!(ack["status"], "accepted");
            assert_eq!(ack["request_id"], req_id);
            assert_eq!(ack["epoch_id"], ep);
            assert_eq!(ack["is_torqued"], want_torqued);
            assert_eq!(ack["safety_state"], want_state);
            assert_eq!(ack["control_mode"], "position");
        }

        // Wrong epoch does NOT consume request_id=10.
        let bad_ep = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 10, "generation": g, "epoch_id": ep + 99, "path": "/api/v1/torque", "body": {"enable": true}}),
            &["command_reject"],
        )
        .await;
        assert_eq!((bad_ep["status"].as_u64(), bad_ep["code"].as_str()), (Some(409), Some("EPOCH_MISMATCH")));

        // Wrong generation does NOT consume request_id=10.
        let bad_gen = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 10, "generation": g + 7, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
            &["command_reject"],
        )
        .await;
        assert_eq!((bad_gen["status"].as_u64(), bad_gen["code"].as_str()), (Some(409), Some("LEASE_LOST")));

        // Same request_id=10 now succeeds with valid epoch & generation.
        let ok10 = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 10, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
            &["command_ack"],
        )
        .await;
        assert_eq!(ok10["is_torqued"], true);

        // Duplicate request_id=10 fails with 400 INVALID_COMMAND (no epoch_id).
        let dup10 = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 10, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": false}}),
            &["command_reject"],
        )
        .await;
        assert_eq!((dup10["status"].as_u64(), dup10["code"].as_str()), (Some(400), Some("INVALID_COMMAND")));
        assert!(dup10.get("epoch_id").is_none(), "duplicate request_id must omit epoch_id: {dup10}");

        // Non-boolean enable and non-object body.
        let bad_en = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 11, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": "nope"}}),
            &["command_reject"],
        )
        .await;
        assert_eq!((bad_en["status"].as_u64(), bad_en["code"].as_str(), bad_en["message"].as_str()), (Some(400), Some("INVALID_COMMAND"), Some("enable must be boolean")));

        let bad_body = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 12, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": 42}),
            &["command_reject"],
        )
        .await;
        assert_eq!((bad_body["status"].as_u64(), bad_body["code"].as_str(), bad_body["message"].as_str()), (Some(400), Some("INVALID_COMMAND"), Some("Command body must be an object")));

        drop(ws);
        if let Some(h) = handle {
            h.stop_and_join();
        }
    }

    // (b) Executor mode with delayed hardware confirmation: WS `command_ack` MUST wait for
    //     confirmed `Outcome::Completed` terminal (NEVER sent on early Step 3 `Ack`).
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        ctrl.allow_disable_confirm.store(false, Ordering::SeqCst);
        let drv = L2MockDriver::new(&cfg, None, ctrl.clone());
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        let addr = serve_with_auth(sup.clone(), OperatorAuth::disabled()).await;
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

        let mut ws = connect_ws(addr, "").await;
        let g = ws_reply(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "hw_ctrl", "seq": 1}),
            &["lease_granted"],
        )
        .await["generation"]
            .as_u64()
            .unwrap();
        let ep = sup.observation().await.unwrap().epoch_id;

        // Send TorqueOn while driver holds confirmation pending.
        send(
            &mut ws,
            json!({"type": "command", "request_id": 1, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
        )
        .await;
        wait_until(Duration::from_secs(2), || {
            ctrl.begin_enable_calls.load(Ordering::SeqCst) >= 1
                && ctrl.poll_enable_calls.load(Ordering::SeqCst) >= 1
        })
        .await;
        assert!(
            next_non_telemetry_within(&mut ws, Duration::from_millis(50))
                .await
                .is_none(),
            "WS command_ack must NOT arrive before hardware confirms TorqueOn"
        );
        assert!(
            !sup.observation().await.unwrap().is_torqued,
            "StateHub must not report is_torqued before hardware confirmation"
        );

        // Release hardware confirmation -> command_ack arrives with is_torqued=true, Armed.
        ctrl.allow_enable_confirm.store(true, Ordering::SeqCst);
        let ack_on = next_of(&mut ws, &["command_ack", "command_reject"]).await;
        assert_eq!(ack_on["type"], "command_ack", "{ack_on}");
        assert_eq!(ack_on["is_torqued"], true);
        assert_eq!(ack_on["safety_state"], "Armed");

        // Send TorqueOff while driver holds disable confirmation pending.
        send(
            &mut ws,
            json!({"type": "command", "request_id": 2, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": false}}),
        )
        .await;
        wait_until(Duration::from_secs(2), || {
            ctrl.begin_disable_calls.load(Ordering::SeqCst) >= 1
        })
        .await;
        assert!(
            next_non_telemetry_within(&mut ws, Duration::from_millis(50))
                .await
                .is_none(),
            "WS command_ack must NOT arrive before hardware confirms TorqueOff"
        );

        ctrl.allow_disable_confirm.store(true, Ordering::SeqCst);
        let ack_off = next_of(&mut ws, &["command_ack", "command_reject"]).await;
        assert_eq!(ack_off["type"], "command_ack", "{ack_off}");
        assert_eq!(ack_off["is_torqued"], false);
        assert_eq!(ack_off["safety_state"], "Idle");

        // Failed enable poll -> command_reject (ENABLE_FAILED) + fault latched + FailedHold (no rollback disable).
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        ctrl.fail_enable_poll.store(true, Ordering::SeqCst);
        let before_dis = ctrl.begin_disable_calls.load(Ordering::SeqCst);
        let rej = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 3, "generation": g, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
            &["command_ack", "command_reject"],
        )
        .await;
        assert_eq!(rej["type"], "command_reject", "{rej}");
        assert_eq!(rej["code"], "ENABLE_FAILED", "{rej}");
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            before_dis,
            "failed enable must enter FailedHold without rollback disable"
        );
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);
        let obs = sup.observation().await.unwrap();
        assert!(obs.is_torqued, "FailedHold must keep is_torqued=true: {obs:?}");
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "{obs:?}");
        assert_eq!(
            obs.fault.as_ref().map(|f| f.code),
            Some(armd::FaultCode::EnableFailed),
            "FailedHold must latch FaultCode::EnableFailed: {obs:?}"
        );

        drop(ws);
        handle.stop_and_join();
    }
}

// ============================================================================
// 10. L2: In-flight TorqueTransition::Enable abort on preempt, disconnect, expiry
// ============================================================================

#[test]
fn in_flight_torque_enable_abort_on_preempt_disconnect_and_expiry() {
    ensure_tracing_subscriber();

    // (a) Preemption during in-flight TorqueOn aborts enable, runs single disable choke point,
    //     and delivers Terminal(Failed("LEASE_LOST")) strictly AFTER Step 6 publish and
    //     strictly BEFORE the replacement LeaseOutcome::Granted!
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.operator_auth_enforced = false;
        inner.prewarm_rt_buffers();
        let c1 = ConnectionHandle::from_ws_id(1).unwrap();
        let c2 = ConnectionHandle::from_ws_id(2).unwrap();
        inner.register_live_connection(1).unwrap();
        inner.register_live_connection(2).unwrap();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(21),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let now = fx.now();
        let dl = now + Duration::from_secs(5);
        let a0 = fx.core.authority(&fx.ctx(now));
        b.submit(
            ClientId(1),
            1,
            a0,
            dl,
            Submit::Lease(LeaseRequest::Acquire(LeaseAcquire {
                connection: c1,
                label: armd::model::HolderLabel::new("holder"),
                has_label: true,
            })),
        )
        .unwrap();
        let _ = fx.step();
        let mut g1 = 0;
        b.poll(|_, r| {
            if let Reply::Lease { outcome: LeaseOutcome::Granted { generation, .. }, .. } = r {
                g1 = generation;
            }
            true
        });
        assert!(g1 >= 1);

        // Start in-flight TorqueOn on c1.
        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        let req_en = b
            .submit(
                ClientId(1),
                2,
                a1,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        let r_en = fx.step();
        assert!(r_en.command.unwrap().1.is_ok());
        assert!(fx.transition().is_some());
        b.poll(|_, _| true); // drain Ack

        // Preempt from c2 while TorqueOn is still pending!
        let a2 = fx.core.authority(&fx.ctx(fx.now()));
        let req_pre = b
            .submit(
                ClientId(2),
                1,
                a2,
                fx.now() + Duration::from_secs(5),
                Submit::Lease(LeaseRequest::Preempt(armd::rt_core::requests::lease::LeasePreempt {
                    connection: c2,
                    label: armd::model::HolderLabel::new("preemptor"),
                    operator: OperatorGrant::unenforced(),
                })),
            )
            .unwrap();
        let r_pre = fx.step();
        assert_eq!(
            r_pre.transition_ended.map(|(_, o)| o),
            Some(Outcome::Failed("LEASE_LOST"))
        );
        assert!(ctrl.abort_enable_calls.load(Ordering::SeqCst) >= 1);
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            0,
            "aborted enable on preempt must enter FailedHold with no rollback disable"
        );
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::On);
        assert!(fx.core.is_torqued);
        assert!(fx.core.safety.has_latched_fault_code(armd::FaultCode::EnableFailed));
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);

        // Exact Step 7 ordering: aborted enable Terminal MUST arrive BEFORE replacement Grant!
        let mut replies = Vec::new();
        b.poll(|_, r| {
            replies.push(r);
            true
        });
        let term_pos = replies
            .iter()
            .position(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Failed("LEASE_LOST"), .. } if *request == req_en))
            .expect("aborted enable terminal must be emitted");
        let grant_pos = replies
            .iter()
            .position(|r| matches!(r, Reply::Lease { request, outcome: LeaseOutcome::Granted { .. }, .. } if *request == req_pre))
            .expect("preempt lease grant must be emitted");
        assert!(
            term_pos < grant_pos,
            "aborted enable Terminal (idx {term_pos}) must precede replacement Grant (idx {grant_pos}): {replies:?}"
        );
        let pub_snap = fx.core.core.hub().latest().unwrap();
        assert!(!pub_snap.owner.torque_transition_pending);
        assert!(pub_snap.owner.lease_generation > g1);
    }

    // (b) Disconnect during in-flight TorqueOn (via priority ConnLifecycleIntake) aborts enable,
    //     calls single disable choke point, and emits Terminal(Failed("DISCONNECTED")).
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.prewarm_rt_buffers();
        let c1 = ConnectionHandle::from_ws_id(1).unwrap();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(22),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let exited = Arc::new(AtomicBool::new(false));
        let (lc_client, lc_intake) = conn_lifecycle_channel(exited);
        fx.attach_conn_lifecycle(lc_intake);

        let lc_ref = Arc::new(lc_client);
        let lc_open = lc_ref.clone();
        let open_th = std::thread::spawn(move || lc_open.open_sync(1));
        for _ in 0..5 {
            let _ = fx.step();
            lc_ref.drain_replies();
            if open_th.is_finished() {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(open_th.join().unwrap(), Ok(Ok(())));

        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Lease(LeaseRequest::Acquire(LeaseAcquire {
                connection: c1,
                label: armd::model::HolderLabel::new("disc_holder"),
                has_label: true,
            })),
        )
        .unwrap();
        let _ = fx.step();
        b.poll(|_, _| true);

        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        let req_en = b
            .submit(
                ClientId(1),
                2,
                a1,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        b.poll(|_, _| true);

        let lc_disc = lc_ref.clone();
        let disc_th = std::thread::spawn(move || lc_disc.disconnect_sync(c1));
        let mut ended_outcome = None;
        for _ in 0..5 {
            let rep = fx.step();
            lc_ref.drain_replies();
            if let Some((_, o)) = rep.transition_ended {
                ended_outcome = Some(o);
            }
            if disc_th.is_finished() {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        let disc_res = disc_th.join().unwrap().unwrap();
        assert!(matches!(disc_res, LeaseOutcome::Disconnected { released: true, .. }));
        assert_eq!(ended_outcome, Some(Outcome::Failed("DISCONNECTED")));
        assert!(ctrl.abort_enable_calls.load(Ordering::SeqCst) >= 1);
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            0,
            "aborted enable on disconnect must enter FailedHold with no rollback disable"
        );
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::On);
        assert!(fx.core.is_torqued);
        assert!(fx.core.safety.has_latched_fault_code(armd::FaultCode::EnableFailed));
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);

        let mut replies = Vec::new();
        b.poll(|_, r| {
            replies.push(r);
            true
        });
        assert!(
            replies.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Failed("DISCONNECTED"), .. } if *request == req_en)),
            "expected DISCONNECTED terminal: {replies:?}"
        );
    }

    // (c) Lease expiry during in-flight TorqueOn aborts BEFORE any retry TX with LEASE_LOST.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 80,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.prewarm_rt_buffers();
        let c1 = ConnectionHandle::from_ws_id(1).unwrap();
        inner.register_live_connection(1).unwrap();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(23),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Lease(LeaseRequest::Acquire(LeaseAcquire {
                connection: c1,
                label: armd::model::HolderLabel::new("exp_holder"),
                has_label: true,
            })),
        )
        .unwrap();
        let _ = fx.step();
        b.poll(|_, _| true);

        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        let req_en = b
            .submit(
                ClientId(1),
                2,
                a1,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        b.poll(|_, _| true);

        let polls_before = ctrl.poll_enable_calls.load(Ordering::SeqCst);
        // Advance virtual clock past lease_timeout (80ms) before the next release!
        mclock.advance(Duration::from_millis(100));
        let rep = fx.step();
        assert_eq!(rep.transition_ended.map(|(_, o)| o), Some(Outcome::Failed("LEASE_LOST")));
        assert_eq!(
            ctrl.poll_enable_calls.load(Ordering::SeqCst),
            polls_before,
            "expired enable must abort BEFORE calling poll_torque_budgeted"
        );
        assert!(ctrl.abort_enable_calls.load(Ordering::SeqCst) >= 1);
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            0,
            "expired enable must enter FailedHold with no rollback disable"
        );
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::On);
        assert!(fx.core.is_torqued);
        assert!(fx.core.safety.has_latched_fault_code(armd::FaultCode::EnableFailed));
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);
        let mut replies = Vec::new();
        b.poll(|_, r| {
            replies.push(r);
            true
        });
        assert!(
            replies.iter().any(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Failed("LEASE_LOST"), .. } if *request == req_en)),
            "expected LEASE_LOST terminal on expiry: {replies:?}"
        );
    }

    // (d) Same-release Step 3 TorqueOn fenced by Disconnect at Step 4 (`BeforeTarget`):
    //     Step 6 MUST publish `torque_transition_pending == false`, and Step 7 MUST emit
    //     `Ack(req_en)` -> `Terminal(req_en, Failed("DISCONNECTED"))` strictly BEFORE lifecycle
    //     Disconnect resolves.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.prewarm_rt_buffers();
        let c1 = ConnectionHandle::from_ws_id(7).unwrap();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(24),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let exited = Arc::new(AtomicBool::new(false));
        let (lc_client, lc_intake) = conn_lifecycle_channel(exited);
        fx.attach_conn_lifecycle(lc_intake);

        let lc_ref = Arc::new(lc_client);
        let lc_open = lc_ref.clone();
        let open_th = std::thread::spawn(move || lc_open.open_sync(7));
        for _ in 0..5 {
            let _ = fx.step();
            lc_ref.drain_replies();
            if open_th.is_finished() {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert_eq!(open_th.join().unwrap(), Ok(Ok(())));

        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(7),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Lease(LeaseRequest::Acquire(LeaseAcquire {
                connection: c1,
                label: armd::model::HolderLabel::new("same_rel_holder"),
                has_label: true,
            })),
        )
        .unwrap();
        let _ = fx.step();
        b.poll(|_, _| true);

        let lc_hook = lc_ref.clone();
        let fired = Arc::new(AtomicBool::new(false));
        let fired_c = fired.clone();
        fx.on_checkpoint(move |cp| {
            if cp == armd::rt_core::ops::Checkpoint::BeforeTarget && !fired_c.swap(true, Ordering::SeqCst) {
                lc_hook.enqueue_disconnect_detached(c1);
            }
        });

        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        let req_en = b
            .submit(
                ClientId(7),
                2,
                a1,
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        let rep = fx.step();
        assert!(fired.load(Ordering::SeqCst), "BeforeTarget hook must fire in the same release");
        assert_eq!(
            rep.transition_ended.map(|(_, o)| o),
            Some(Outcome::Failed("DISCONNECTED")),
            "same-release Disconnect must abort and end the Step-3 TorqueOn before Step 6"
        );
        assert!(ctrl.abort_enable_calls.load(Ordering::SeqCst) >= 1);
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            0,
            "same-release disconnected enable must enter FailedHold with no rollback disable"
        );
        assert_eq!(fx.core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::On);
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);
        let pub_snap = fx.core.core.hub().latest().unwrap();
        assert!(
            !pub_snap.owner.torque_transition_pending,
            "Step 6 published snapshot must report torque_transition_pending == false"
        );
        assert_eq!(pub_snap.owner.lease_generation, 0);

        let mut replies = Vec::new();
        b.poll(|_, r| {
            replies.push(r);
            true
        });
        let ack_pos = replies
            .iter()
            .position(|r| matches!(r, Reply::Ack { request, .. } if *request == req_en))
            .expect("same-release TorqueOn must emit its broker Ack first");
        let term_pos = replies
            .iter()
            .position(|r| matches!(r, Reply::Terminal { request, outcome: Outcome::Failed("DISCONNECTED"), .. } if *request == req_en))
            .expect("same-release TorqueOn must emit its DISCONNECTED Terminal");
        assert!(
            ack_pos < term_pos,
            "Ack (idx {ack_pos}) must precede Terminal (idx {term_pos}): {replies:?}"
        );
    }
}

// ============================================================================
// 11. L2: BackgroundDisableRetry (bounded via single choke point, recovery + fault)
// ============================================================================

#[test]
fn background_disable_retry_bounded_choke_point_recovery_and_fault_exhaustion() {
    ensure_tracing_subscriber();

    // (a) Transient CAN TX failure on explicit TorqueOff recovers on retry 4 via single choke point.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(true, Ordering::SeqCst);
        ctrl.fail_disable_begin_remaining.store(3, Ordering::SeqCst);
        ctrl.allow_disable_confirm.store(true, Ordering::SeqCst);

        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.prewarm_rt_buffers();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(31),
            Capacities { commands: 4, plans: 1, replies: 8 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(fx.step().transition_ended.map(|(_, o)| o), Some(Outcome::Completed));
        assert!(fx.core.is_torqued);

        // Explicit TorqueOff fails with transient transport error -> arms BackgroundDisableRetry.
        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            2,
            a1,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();
        let r_fail = fx.step();
        assert!(r_fail.command.unwrap().1.is_ok());
        assert_eq!(
            r_fail.transition_ended.map(|(_, o)| o),
            Some(Outcome::Failed("DRIVER_WRITE_FAILED"))
        );
        assert!(fx.core.disable_pending);
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), 1);

        // Step releases (20ms each): retries at >=25ms spacing until call 4 succeeds.
        for _ in 0..12 {
            let _ = fx.step();
            if !fx.core.disable_pending {
                break;
            }
        }
        assert!(!fx.core.disable_pending, "background disable must confirm once bus recovers");
        assert!(!fx.core.is_torqued);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::Off);
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), 4);
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);
    }

    // (b) Persistent disable failure on explicit TorqueOff exhausts the exact bounded retry cap
    //     (1 initial + 8 retries = 9), latches specifically `StaticFault::DisableUnconfirmed`
    //     ("Torque disable has not been confirmed by fresh hardware feedback"), and NEVER restarts
    //     across ticks or `request_shutdown`.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(true, Ordering::SeqCst);
        ctrl.fail_disable_begin_remaining.store(100, Ordering::SeqCst);

        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.lease_enabled = false;
        inner.prewarm_rt_buffers();
        inner.register_live_connection(1).unwrap();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(32),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(fx.step().transition_ended.map(|(_, o)| o), Some(Outcome::Completed));
        assert!(fx.core.is_torqued);

        // Explicit TorqueOff fails on call #1 -> arms BackgroundDisableRetry.
        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            2,
            a1,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), 1);
        assert!(fx.core.disable_pending);
        assert!(
            !matches!(
                fx.core.safety.state(),
                SafetyState::Fault(armd::FaultReason::Other(r))
                    if r == "Torque disable has not been confirmed by fresh hardware feedback"
            ),
            "DisableUnconfirmed must NOT be latched before retries exhaust: {:?}",
            fx.core.safety.state()
        );

        // 20 ms release steps with 25 ms retry spacing => each retry fires every 40 ms
        // (2 steps): 8 retries take 8 * 40 ms = 320 ms (16 steps) < 1 s lifetime.
        for _ in 0..20 {
            let _ = fx.step();
        }
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            9,
            "expected exact 1 initial + 8 bounded retries = 9 calls on 20 ms timeline"
        );
        assert!(fx.core.disable_pending, "disable_pending must remain true until confirmed");
        let fault = fx.core.safety.fault().expect("exhaustion must latch fault");
        assert_eq!(fault.code, armd::safety::FaultCode::DriverWriteFailed);
        assert_eq!(
            fault.message.as_str(),
            "Torque disable has not been confirmed by fresh hardware feedback",
            "exhaustion must latch exact StaticFault::DisableUnconfirmed, got: {fault:?}"
        );
        assert!(matches!(fx.core.safety.state(), SafetyState::Fault(_)));
        assert!(fx.core.safety.has_latched_fault_code(armd::safety::FaultCode::DriverWriteFailed));
        assert_eq!(ctrl.set_torque_blocking_calls.load(Ordering::SeqCst), 0);

        // Advance well past both the 250 ms legacy fallback interval (wall + virtual) and the
        // 1 s retry window, and trigger `request_shutdown`: call count MUST stay 9.
        std::thread::sleep(Duration::from_millis(260));
        for _ in 0..55 {
            let _ = fx.step();
        }
        assert!(!fx.core.request_shutdown());
        for _ in 0..10 {
            let _ = fx.step();
        }
        assert_eq!(
            ctrl.begin_disable_calls.load(Ordering::SeqCst),
            9,
            "exhausted BackgroundDisableRetry must NEVER restart across ticks or request_shutdown"
        );
    }

    // (c) Pending driver disable handshake that stays pending across the retry window
    //     (`STOPPED_DISABLE_REQUEST_LIFETIME` = 1 s when stopped) MUST STILL exhaust and latch
    //     `StaticFault::DisableUnconfirmed` rather than bypassing the deadline forever.
    {
        struct StuckPendingDisableDriver {
            dof: usize,
            positions: Vec<f32>,
            enabled: bool,
            clock: MockClock,
            pending_disable: bool,
            disable_calls: Arc<AtomicUsize>,
        }

        impl ArmDriver for StuckPendingDisableDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    enable_feedback: true,
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    ..DriverCapabilities::default()
                }
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(self
                    .positions
                    .iter()
                    .map(|&p| JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 })
                    .collect())
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                _deadline: Instant,
                _max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                out.clear();
                for &p in &self.positions {
                    out.push(JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 });
                }
                Ok((self.dof as u16, false))
            }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
                out.clear();
                for &p in &self.positions {
                    out.push(JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 });
                }
                Some(())
            }
            fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
                self.enabled = enable;
                Ok(())
            }
            fn begin_disable_torque_budgeted(
                &mut self,
                kind: armd::driver::DisableKind,
                _: u16,
            ) -> Result<DrvTorqueProgress, ArmDriverError> {
                let call = self.disable_calls.fetch_add(1, Ordering::SeqCst) + 1;
                if call == 1 {
                    return Err(ArmDriverError::Transport(TransportError::WouldBlock));
                }
                let _ = kind;
                self.pending_disable = true;
                Ok(DrvTorqueProgress::Pending {
                    wake: self.clock.now() + Duration::from_millis(20),
                })
            }
            fn begin_torque_budgeted(&mut self, enable: bool, _: u16) -> Result<DrvTorqueProgress, ArmDriverError> {
                if enable {
                    self.enabled = true;
                    return Ok(DrvTorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                unreachable!("disable routes through begin_disable_torque_budgeted")
            }
            fn poll_torque_budgeted(&mut self, _: Instant, _: u16) -> Result<DrvTorqueProgress, ArmDriverError> {
                if self.pending_disable {
                    // Driver handshake genuinely stays Pending forever; `service_background_disable`
                    // must still terminate on `window_expired` even while `still_pending == true`
                    // and `attempts < BACKGROUND_DISABLE_MAX_RETRIES`.
                    return Ok(DrvTorqueProgress::Pending {
                        wake: self.clock.now() + Duration::from_millis(20),
                    });
                }
                Ok(DrvTorqueProgress::Done(TorqueOutcome::Confirmed))
            }
            fn torque_transition_pending(&self) -> bool { self.pending_disable }
            fn is_enabled(&self) -> bool { self.enabled }
            fn enable_state(&self) -> EnableState {
                if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(1)));
            }
            fn has_driver_fresh_position_api(&self) -> bool {
                true
            }
            fn fresh_joint_positions_into(
                &self,
                joints: &[usize],
                max_age: Duration,
                out: &mut [f32],
            ) -> bool {
                if joints.is_empty() || out.len() < joints.len() || Duration::from_millis(1) > max_age {
                    return false;
                }
                for (dst, &j) in out.iter_mut().zip(joints.iter()) {
                    let Some(&p) = self.positions.get(j) else {
                        return false;
                    };
                    *dst = p;
                }
                true
            }
            fn clock_now(&self) -> Instant { self.clock.now() }
        }

        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let disable_calls = Arc::new(AtomicUsize::new(0));
        let drv = StuckPendingDisableDriver {
            dof: cfg.dof,
            positions: cfg.resting_positions().to_vec(),
            enabled: false,
            clock: mclock.clone(),
            pending_disable: false,
            disable_calls: Arc::clone(&disable_calls),
        };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.lease_enabled = false;
        inner.prewarm_rt_buffers();
        inner.register_live_connection(2).unwrap();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(33),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(2),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert!(fx.core.is_torqued);

        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(2),
            2,
            a1,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(disable_calls.load(Ordering::SeqCst), 1);

        // Advance 60 * 20 ms = 1200 ms > STOPPED_DISABLE_REQUEST_LIFETIME (1 s) while handshake stays Pending:
        // `service_background_disable` must terminate the stopped `DisableRequested` on `window_expired`
        // even while `still_pending == true` and `attempts < BACKGROUND_DISABLE_MAX_RETRIES`.
        for _ in 0..60 {
            let _ = fx.step();
        }
        assert_eq!(disable_calls.load(Ordering::SeqCst), 2);
        assert!(
            !fx.core.disable_pending,
            "expired stopped DisableRequested must clear disable_pending instead of retrying forever"
        );
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::On);
        assert!(matches!(fx.core.safety.state(), SafetyState::Fault(_)));
        assert!(fx.core.safety.has_latched_fault_code(armd::safety::FaultCode::DriverWriteFailed));
        for _ in 0..10 {
            let _ = fx.step();
        }
        assert_eq!(
            disable_calls.load(Ordering::SeqCst),
            2,
            "terminated pending disable must never restart after window expiry"
        );
    }
}

// ============================================================================
// 12. L2: TorqueOff Stop exemption, WS clear_fault OperatorGrant live revocation,
//         and REST /torque, /arm, /safety/clear_fault lease gates in executor mode
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_torque_off_exempt_across_stop_and_clear_fault_operator_grant_both_owners() {
    ensure_tracing_subscriber();

    for executor_mode in [false, true] {
        let auth = OperatorAuth::enforced("246810", 1800);
        let (sup, addr, handle) = setup_mode(
            ArmConfig {
                lease_timeout_ms: 5_000,
                ..ArmConfig::rebot_dm()
            },
            auth,
            executor_mode,
        )
        .await;
        let http = reqwest::Client::new();

        // REST /torque, /arm, /safety/clear_fault return 409 LEASE_REQUIRED with no lease.
        for (path, body) in [
            ("/api/v1/torque", json!({"enable": true})),
            ("/api/v1/arm", json!({"enable": true})),
            ("/api/v1/safety/clear_fault", json!({})),
        ] {
            let resp = http
                .post(format!("http://{addr}{path}"))
                .json(&body)
                .send()
                .await
                .unwrap();
            assert_eq!(resp.status(), reqwest::StatusCode::CONFLICT, "mode={executor_mode} path={path}");
            assert_eq!(resp.json::<Value>().await.unwrap()["code"], "LEASE_REQUIRED");
        }

        let mut ws = connect_ws(addr, "auth=explicit").await;
        let g = ws_reply(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "op_ctrl", "seq": 1}),
            &["lease_granted"],
        )
        .await["generation"]
            .as_u64()
            .unwrap();

        // With active WS lease, REST /torque, /arm, /safety/clear_fault return 409 LEASE_ACTIVE.
        for (path, body) in [
            ("/api/v1/torque", json!({"enable": true})),
            ("/api/v1/arm", json!({"enable": true})),
            ("/api/v1/safety/clear_fault", json!({})),
        ] {
            let resp = http
                .post(format!("http://{addr}{path}"))
                .json(&body)
                .send()
                .await
                .unwrap();
            assert_eq!(resp.status(), reqwest::StatusCode::CONFLICT, "mode={executor_mode} path={path}");
            assert_eq!(resp.json::<Value>().await.unwrap()["code"], "LEASE_ACTIVE");
        }

        // Arm, send urgent Stop, then TorqueOff -> TorqueOff succeeds across Stop!
        let ep0 = sup.observation().await.unwrap().epoch_id;
        let on = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 1, "generation": g, "epoch_id": ep0, "path": "/api/v1/torque", "body": {"enable": true}}),
            &["command_ack"],
        )
        .await;
        assert_eq!(on["is_torqued"], true);

        let stop_ack = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 2, "generation": g, "epoch_id": ep0, "path": "/api/v1/motion/stop", "body": {}}),
            &["command_ack"],
        )
        .await;
        assert_eq!(stop_ack["type"], "command_ack");

        let off = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 3, "generation": g, "epoch_id": ep0, "path": "/api/v1/torque", "body": {"enable": false}}),
            &["command_ack", "command_reject"],
        )
        .await;
        assert_eq!(off["type"], "command_ack", "mode={executor_mode} off={off}");
        assert_eq!(off["is_torqued"], false);
        assert_eq!(off["safety_state"], "Idle");

        // Trigger WS E-stop -> viewer clear_fault refused 403 OPERATOR_REQUIRED.
        let estop_sync = ws_reply(&mut ws, json!({"type": "estop"}), &["epoch_sync"]).await;
        let ep_estop = estop_sync["epoch_id"].as_u64().unwrap();
        assert_eq!(estop_sync["safety_state"], "EStop");

        let denied = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 4, "generation": g, "epoch_id": ep_estop, "path": "/api/v1/safety/clear_fault", "body": {}}),
            &["command_reject"],
        )
        .await;
        assert_eq!(
            (denied["status"].as_u64(), denied["code"].as_str(), denied["message"].as_str()),
            (Some(403), Some("OPERATOR_REQUIRED"), Some("Operator unlock is required for clear_fault (EStop latched)")),
            "mode={executor_mode} denied={denied}"
        );

        // Unlock via REST + WS auth -> invalid mit_recovery rejected 400, valid clear_fault succeeds!
        let unlock_resp: Value = http
            .post(format!("http://{addr}/api/v1/auth/unlock"))
            .json(&json!({"pin": "246810"}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let token = unlock_resp["token"].as_str().unwrap().to_string();
        let auth_ack = ws_reply(
            &mut ws,
            json!({"type": "auth", "token": token, "seq": 1}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(auth_ack["role"], "operator");

        let bad_mit = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 5, "generation": g, "epoch_id": ep_estop, "path": "/api/v1/safety/clear_fault", "body": {"mit_recovery": "wrong"}}),
            &["command_reject"],
        )
        .await;
        assert_eq!((bad_mit["status"].as_u64(), bad_mit["code"].as_str()), (Some(400), Some("INVALID_COMMAND")));

        let cleared = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 6, "generation": g, "epoch_id": ep_estop, "path": "/api/v1/safety/clear_fault", "body": {}}),
            &["command_ack", "command_reject"],
        )
        .await;
        assert_eq!(cleared["type"], "command_ack", "mode={executor_mode} cleared={cleared}");
        assert_eq!(cleared["safety_state"], "Idle");
        assert!(cleared["epoch_id"].as_u64().unwrap() > ep_estop);

        // Second E-stop + REST /api/v1/auth/lock revokes live OperatorGrant -> clear_fault immediately 403!
        let estop2 = ws_reply(&mut ws, json!({"type": "estop"}), &["epoch_sync"]).await;
        let ep_estop2 = estop2["epoch_id"].as_u64().unwrap();
        let lock_resp = http
            .post(format!("http://{addr}/api/v1/auth/lock"))
            .header("authorization", format!("Bearer {token}"))
            .send()
            .await
            .unwrap();
        assert_eq!(lock_resp.status(), reqwest::StatusCode::OK);

        let revoked = ws_reply(
            &mut ws,
            json!({"type": "command", "request_id": 7, "generation": g, "epoch_id": ep_estop2, "path": "/api/v1/safety/clear_fault", "body": {}}),
            &["command_reject"],
        )
        .await;
        assert_eq!(
            (revoked["status"].as_u64(), revoked["code"].as_str()),
            (Some(403), Some("OPERATOR_REQUIRED")),
            "revoked OperatorGrant must fail clear_fault live: mode={executor_mode} {revoked}"
        );

        drop(ws);
        if let Some(h) = handle {
            h.stop_and_join();
            // After executor exit, REST /torque, /arm, /safety/clear_fault return 503 OBSERVATION_UNAVAILABLE.
            for (path, body) in [
                ("/api/v1/torque", json!({"enable": true})),
                ("/api/v1/arm", json!({"enable": true})),
                ("/api/v1/safety/clear_fault", json!({})),
            ] {
                let dead = http
                    .post(format!("http://{addr}{path}"))
                    .json(&body)
                    .send()
                    .await
                    .unwrap();
                assert_eq!(dead.status(), reqwest::StatusCode::SERVICE_UNAVAILABLE);
                assert_eq!(dead.json::<Value>().await.unwrap()["code"], "OBSERVATION_UNAVAILABLE");
            }
        }
    }
}

// ============================================================================
// 13. L2: Zero-alloc / zero-free RT proof across torque transitions, retries,
//         abort, and LeaseClearFault with live tracing subscriber enabled
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn zero_alloc_rt_l2_torque_and_clear_fault_with_tracing_enabled() {
    ensure_tracing_subscriber();

    // Positive control.
    armd::diagnostics::alloc::time_this_thread();
    let _ = armd::diagnostics::alloc::take();
    let probe: Vec<u8> = vec![0xCD; 256];
    std::hint::black_box(&probe);
    drop(probe);
    assert!(armd::diagnostics::alloc::take().calls > 0);

    // (a) Direct RT-path zero-alloc proof across TorqueOn/TorqueOff commit, refusal, failure,
    //     abort, BackgroundDisableRetry exhaustion, and LeaseClearFault (refusal + success).
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok, _) = auth.unlock("123456").unwrap();
        let grant_ok = auth.operator_grant(Some(&tok));
        let grant_denied = OperatorGrant::DENIED;

        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut core = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        core.rt_executor_owned = true;
        core.operator_auth_enforced = true;
        core.prewarm_rt_buffers();
        for g in 1..=4 {
            let _ = core.tick_publish(0.005, g, false);
            let _ = core.core.hub().latest();
        }
        let c1 = ConnectionHandle::from_ws_id(1).unwrap();
        core.register_live_connection(1).unwrap();
        let now = mclock.now();
        let dl = now + Duration::from_secs(5);
        let snap0 = core.core.hub().latest().unwrap().owner;
        let arm = snap0.arm;
        let boot = snap0.boot;
        let p_acq = prepare_lease_acquire(&snap0, c1, "rt_l2_alloc", 1, now, dl).unwrap();
        let g1 = match install_lease_acquire(
            &mut core,
            &p_acq.request,
            &LeaseInstallCtx {
                expected_arm: arm,
                expected_boot: boot,
                stamp_arm: arm,
                stamp_boot: boot,
                now,
                request_id: 1,
                client: ClientId(1),
                deadline: dl,
                enforce_single_use: true,
            },
        ) {
            Ok(LeaseOutcome::Granted { generation, .. }) => generation,
            other => panic!("unexpected: {other:?}"),
        };

        let cctx = CommitCtx {
            arm,
            boot,
            now,
            stop_generation: 0,
        };
        let ws_rev = core.workspace_revision;
        let mk_tr = |req_id: u64, op: TransitionOp, ep: u64| Descriptor {
            stamp: Stamp {
                arm,
                boot,
                request: armd::rt_core::broker::RequestId(req_id),
                client: ClientId(1),
                client_seq: req_id,
                authority: Authority {
                    lease_generation: g1,
                    epoch: ep,
                    stop_generation: 0,
                    config_revision: ws_rev,
                    parent_generation: 0,
                },
                deadline: now + Duration::from_secs(60),
            },
            kind: armd::rt_core::broker::CommandKind::Transition(op),
        };

        let _ = armd::diagnostics::alloc::take();

        // 1. TorqueOff refused ARM_NOT_AT_RESTING_POSE (arm raised off rest pose) -> 0 alloc.
        let ep1 = core.safety.epoch_id();
        let saved_j1 = core.measured_positions[1];
        core.measured_positions[1] = saved_j1 + 0.5;
        let d_off_refuse = mk_tr(10, TransitionOp::TorqueOff, ep1);
        assert_eq!(
            core.commit_transition(&d_off_refuse, TransitionOp::TorqueOff, &cctx),
            Err("ARM_NOT_AT_RESTING_POSE")
        );
        core.measured_positions[1] = saved_j1;

        // 2. TorqueOn start -> poll confirmed -> TorqueOff start -> poll confirmed -> 0 alloc.
        ctrl.allow_enable_confirm.store(true, Ordering::SeqCst);
        let d_on = mk_tr(11, TransitionOp::TorqueOn, ep1);
        assert!(core.commit_transition(&d_on, TransitionOp::TorqueOn, &cctx).is_ok());
        let b1 = ServiceBudget::for_release(mclock.now(), mclock.now() + Duration::from_millis(20));
        let p1 = core.poll_hardware(&b1);
        assert_eq!(p1.torque, Some(Ok(armd::rt_core::ops::TorqueProgress::Done)));
        assert!(core.is_torqued);

        ctrl.allow_disable_confirm.store(false, Ordering::SeqCst);
        let d_off = mk_tr(12, TransitionOp::TorqueOff, ep1);
        assert!(core.commit_transition(&d_off, TransitionOp::TorqueOff, &cctx).is_ok());
        let p2 = core.poll_hardware(&b1);
        assert!(matches!(p2.torque, Some(Ok(armd::rt_core::ops::TorqueProgress::Pending { .. }))));
        ctrl.allow_disable_confirm.store(true, Ordering::SeqCst);
        let p3 = core.poll_hardware(&b1);
        assert_eq!(p3.torque, Some(Ok(armd::rt_core::ops::TorqueProgress::Done)));
        assert!(!core.is_torqued);
        assert_eq!(core.torque_intent(), armd::TorqueIntent::Off);

        // 3a. TorqueOn failure enters FailedHold with no rollback disable -> TorqueOff -> LeaseClearFault -> 0 alloc.
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        ctrl.fail_enable_poll.store(true, Ordering::SeqCst);
        let before_dis = ctrl.begin_disable_calls.load(Ordering::SeqCst);
        let d_on_fail = mk_tr(13, TransitionOp::TorqueOn, ep1);
        assert!(core.commit_transition(&d_on_fail, TransitionOp::TorqueOn, &cctx).is_ok());
        let p_fail = core.poll_hardware(&b1);
        assert_eq!(p_fail.torque, Some(Err("ENABLE_FAILED")));
        assert_eq!(core.enable_lifecycle(), armd::supervisor::EnableLifecycle::FailedHold);
        assert_eq!(core.torque_intent(), armd::TorqueIntent::On);
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), before_dis);

        let ep_fh = core.safety.epoch_id();
        let d_off_fh = mk_tr(14, TransitionOp::TorqueOff, ep_fh);
        assert!(core.commit_transition(&d_off_fh, TransitionOp::TorqueOff, &cctx).is_ok());
        assert_eq!(
            core.poll_hardware(&b1).torque,
            Some(Ok(armd::rt_core::ops::TorqueProgress::Done))
        );
        let cleared_fh = install_lease_clear_fault_rt(
            &mut core,
            c1,
            g1,
            Some(ep_fh),
            true,
            None,
            OwnerAuthPolicy::ClearFault,
            grant_ok,
            mclock.now(),
            mclock.now() + Duration::from_millis(20),
        );
        assert!(matches!(
            cleared_fh,
            LeaseClearFaultOutcome::Cleared { safety_state: SafetyState::Idle, .. }
        ), "{cleared_fh:?}");

        // 3b. Re-arm and test BackgroundDisableRetry exhaustion on transport-failed TorqueOff -> 0 alloc.
        ctrl.fail_enable_poll.store(false, Ordering::SeqCst);
        ctrl.allow_enable_confirm.store(true, Ordering::SeqCst);
        let ep_rearm = core.safety.epoch_id();
        let d_on_ok = mk_tr(15, TransitionOp::TorqueOn, ep_rearm);
        assert!(core.commit_transition(&d_on_ok, TransitionOp::TorqueOn, &cctx).is_ok());
        assert_eq!(
            core.poll_hardware(&b1).torque,
            Some(Ok(armd::rt_core::ops::TorqueProgress::Done))
        );

        ctrl.fail_disable_begin_remaining.store(20, Ordering::SeqCst);
        let d_off_fail = mk_tr(16, TransitionOp::TorqueOff, ep_rearm);
        assert!(core.commit_transition(&d_off_fail, TransitionOp::TorqueOff, &cctx).is_ok());
        assert!(core.disable_pending);
        for _ in 0..10 {
            mclock.advance(Duration::from_millis(30));
            let bn = ServiceBudget::for_release(mclock.now(), mclock.now() + Duration::from_millis(20));
            let _ = core.poll_hardware(&bn);
        }

        // 4. LeaseClearFault & TorquePreflight refusals -> 0 alloc.
        let ep_fault = core.safety.epoch_id();
        let now_f = mclock.now();
        let rd_f = now_f + Duration::from_millis(20);
        assert!(matches!(
            install_lease_clear_fault_rt(&mut core, c1, g1 + 1, Some(ep_fault), true, None, OwnerAuthPolicy::ClearFault, grant_ok, now_f, rd_f),
            LeaseClearFaultOutcome::LeaseLost { .. }
        ));
        assert!(matches!(
            install_lease_clear_fault_rt(&mut core, c1, g1, Some(ep_fault + 1), true, None, OwnerAuthPolicy::ClearFault, grant_ok, now_f, rd_f),
            LeaseClearFaultOutcome::EpochMismatch { consumed_request_id: false, .. }
        ));
        assert!(matches!(
            install_lease_clear_fault_rt(&mut core, c1, g1, Some(ep_fault), false, None, OwnerAuthPolicy::ClearFault, grant_ok, now_f, rd_f),
            LeaseClearFaultOutcome::InvalidRequestId
        ));
        assert!(matches!(
            install_lease_clear_fault_rt(&mut core, c1, g1, Some(ep_fault), true, None, OwnerAuthPolicy::ClearFault, grant_denied, now_f, rd_f),
            LeaseClearFaultOutcome::OperatorRequired { .. }
        ));
        assert!(matches!(
            install_lease_clear_fault_rt(&mut core, c1, g1, Some(ep_fault), true, Some("bad"), OwnerAuthPolicy::ClearFault, grant_ok, now_f, rd_f),
            LeaseClearFaultOutcome::InvalidBody { .. }
        ));
        // ClearFailed because disable_pending is still true -> 0 alloc!
        assert!(matches!(
            install_lease_clear_fault_rt(&mut core, c1, g1, Some(ep_fault), true, None, OwnerAuthPolicy::ClearFault, grant_ok, now_f, rd_f),
            LeaseClearFaultOutcome::ClearFailed { .. }
        ));
        // Live owner torque preflight (malformed & lease-lost & operator policy) -> 0 alloc!
        assert!(matches!(
            check_torque_owner_preflight(&mut core, c1, g1 + 1, Some(ep_fault), true, Some("bad"), OwnerAuthPolicy::None, grant_ok, now_f),
            Err(TorquePreflightError::LeaseLost { .. })
        ));
        assert!(matches!(
            check_torque_owner_preflight(&mut core, c1, g1, Some(ep_fault), true, Some("bad"), OwnerAuthPolicy::None, grant_ok, now_f),
            Err(TorquePreflightError::InvalidBody { .. })
        ));
        assert!(matches!(
            check_torque_owner_preflight(&mut core, c1, g1, Some(ep_fault), true, None, OwnerAuthPolicy::Operator(None), grant_denied, now_f),
            Err(TorquePreflightError::OperatorRequired { .. })
        ));

        // Confirm disable via TorqueOff, then LeaseClearFault succeeds -> 0 alloc!
        ctrl.fail_disable_begin_remaining.store(0, Ordering::SeqCst);
        ctrl.allow_disable_confirm.store(true, Ordering::SeqCst);
        let d_off_ok = mk_tr(17, TransitionOp::TorqueOff, ep_fault);
        assert!(core.commit_transition(&d_off_ok, TransitionOp::TorqueOff, &cctx).is_ok());
        let p_off_done = core.poll_hardware(&b1);
        assert_eq!(p_off_done.torque, Some(Ok(armd::rt_core::ops::TorqueProgress::Done)));
        assert!(!core.disable_pending);
        assert!(!core.is_torqued);
        assert_eq!(core.torque_intent(), armd::TorqueIntent::Off);
        let cleared = install_lease_clear_fault_rt(
            &mut core,
            c1,
            g1,
            Some(ep_fault),
            true,
            None,
            OwnerAuthPolicy::ClearFault,
            grant_ok,
            now_f,
            rd_f,
        );
        assert!(matches!(cleared, LeaseClearFaultOutcome::Cleared { safety_state: SafetyState::Idle, .. }), "{cleared:?}");

        // Refusal on TorqueOn while pending / after fault -> 0 alloc!
        ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
        ctrl.fail_enable_poll.store(false, Ordering::SeqCst);
        let ep_clean = core.safety.epoch_id();
        let cctx_clean = CommitCtx { arm, boot, now: now_f, stop_generation: 0 };
        let d_on2 = mk_tr(18, TransitionOp::TorqueOn, ep_clean);
        assert!(core.commit_transition(&d_on2, TransitionOp::TorqueOn, &cctx_clean).is_ok());
        // Fence displaced lease via disconnect while TorqueOn is pending -> abort_enable_transition_for_fenced_lease -> 0 alloc!
        ctrl.fail_disable_begin_remaining.store(0, Ordering::SeqCst);
        let _ = install_lease_disconnect(&mut core, &LeaseDisconnect { connection: c1 }, now_f);

        let stats = armd::diagnostics::alloc::take();
        assert_eq!(
            stats.calls, 0,
            "RT L2 torque transitions, retries, abort, and LeaseClearFault must do 0 allocs/frees: {stats:?}"
        );
    }

    // (b) Live `armd-control` whole-release zero-alloc proof (`alloc_timing = true`) across
    //     WS /api/v1/torque, /api/v1/arm, estop, and /api/v1/safety/clear_fault.
    {
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

        wait_until(Duration::from_secs(2), || sup.tick_samples().len() >= 4).await;
        let _ = sup.fresh_snapshot(None).await.unwrap();
        wait_until(Duration::from_secs(2), || sup.tick_samples().len() >= 10).await;
        let baseline_samples = sup.tick_samples().len();

        let mut ws = connect_ws(addr, "auth=explicit").await;
        let g = ws_reply(
            &mut ws,
            json!({"type": "acquire_lease", "client_id": "l2_alloc_live", "seq": 1}),
            &["lease_granted"],
        )
        .await["generation"]
            .as_u64()
            .unwrap();
        let ep0 = sup.observation().await.unwrap().epoch_id;

        assert_eq!(
            ws_reply(
                &mut ws,
                json!({"type": "command", "request_id": 1, "generation": g, "epoch_id": ep0, "path": "/api/v1/torque", "body": {"enable": true}}),
                &["command_ack"],
            )
            .await["is_torqued"],
            true
        );
        assert_eq!(
            ws_reply(
                &mut ws,
                json!({"type": "command", "request_id": 2, "generation": g, "epoch_id": ep0, "path": "/api/v1/arm", "body": {"enable": false}}),
                &["command_ack"],
            )
            .await["is_torqued"],
            false
        );
        // Malformed torque body routes via MaintenanceOp::LeaseTorquePreflight -> 0 alloc on RT!
        assert_eq!(
            ws_reply(
                &mut ws,
                json!({"type": "command", "request_id": 3, "generation": g, "epoch_id": ep0, "path": "/api/v1/torque", "body": {"enable": "not_bool"}}),
                &["command_reject"],
            )
            .await["code"],
            "INVALID_COMMAND"
        );

        let ep1 = ws_reply(&mut ws, json!({"type": "estop"}), &["epoch_sync"])
            .await["epoch_id"]
            .as_u64()
            .unwrap();
        assert_eq!(
            ws_reply(
                &mut ws,
                json!({"type": "command", "request_id": 4, "generation": g, "epoch_id": ep1, "path": "/api/v1/safety/clear_fault", "body": {}}),
                &["command_reject"],
            )
            .await["code"],
            "OPERATOR_REQUIRED"
        );
        let _ = ws_reply(
            &mut ws,
            json!({"type": "auth", "token": tok_op, "seq": 1}),
            &["auth_ack"],
        )
        .await;
        assert_eq!(
            ws_reply(
                &mut ws,
                json!({"type": "command", "request_id": 5, "generation": g, "epoch_id": ep1, "path": "/api/v1/safety/clear_fault", "body": {}}),
                &["command_ack"],
            )
            .await["safety_state"],
            "Idle"
        );
        // Post-disconnect live authority preflight: malformed torque on a disconnected handle
        // MUST yield LEASE_LOST (not stale-snapshot INVALID_COMMAND) with 0 alloc on RT!
        let c_disc = ConnectionHandle::from_ws_id(999).unwrap();
        let bad_after_disc = sup
            .torque_ws(
                c_disc,
                g,
                Some(ep1),
                true,
                6,
                Err("enable must be boolean"),
                OwnerAuthPolicy::None,
                armd::auth::OperatorGrant::DENIED,
                sup.capture_admission_stamp(),
            )
            .await
            .unwrap();
        assert!(
            matches!(bad_after_disc, armd::rt_core::requests::lease::TorqueWsOutcome::LeaseLost { .. }),
            "disconnected handle must fail live authority with LeaseLost, got {bad_after_disc:?}"
        );

        let _ = ws.close(None).await;
        drop(ws);
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
            "live executor L2 releases must do 0 allocs/frees: {nonzero:?}"
        );
        assert_eq!(sup.inner().control_thread_attempts(), 0);
        handle.stop_and_join();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn m1_disconnect_fences_lease_first_both_owners_and_transfer_lock_over_500ms() {
    // Part A: Both owners (Legacy & Executor) fence the lease generation immediately on socket
    // close so zero dynamic output from that generation reaches the bus, before task cleanup.
    for executor_mode in [false, true] {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let ctrl = Arc::new(L2DriverControl::default());
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        sup.set_driver(Box::new(L2MockDriver::new(&cfg, None, Arc::clone(&ctrl))))
            .await;
        let auth = OperatorAuth::disabled();
        let addr = serve_with_auth(sup.clone(), auth).await;

        let handle = if executor_mode {
            let h = sup
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
                .unwrap();
            wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;
            Some(h)
        } else {
            sup.control_tick(0.02).await;
            None
        };

        let mut ws1 = connect_ws(addr, "").await;
        let g1 = ws_reply(
            &mut ws1,
            json!({"type": "acquire_lease", "client_id": "m1_holder", "seq": 1}),
            &["lease_granted"],
        )
        .await["generation"]
            .as_u64()
            .unwrap();
        let ep = sup.observation().await.unwrap().epoch_id;

        // Enable torque and verify bus writes happen while lease is active.
        assert_eq!(
            ws_reply(
                &mut ws1,
                json!({"type": "command", "request_id": 1, "generation": g1, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
                &["command_ack"],
            )
            .await["is_torqued"],
            true
        );

        if !executor_mode {
            // Legacy: hold the write lock while closing the socket -> immediate non-blocking fence
            // MUST fence the generation and suppress bus output BEFORE the lock is even released.
            let mut guard = sup.inner().write().await;
            let lease_client_id = guard
                .active_lease
                .as_ref()
                .expect("lease active before close")
                .client_id
                .clone();
            let conn_id: u64 = lease_client_id
                .strip_prefix("connection:")
                .expect("ws connection handle prefix")
                .parse()
                .expect("numeric ws connection id");
            assert_ne!(conn_id, 0);
            // Close the WS socket while the legacy write lock is held.
            let _ = ws1.close(None).await;
            drop(ws1);
            wait_until(Duration::from_secs(2), || {
                sup.is_connection_fenced_closed(conn_id) && sup.has_pending_disconnect_hold()
            })
            .await;
            assert!(
                sup.atomic_fenced_lease_gen() >= g1,
                "legacy socket close must fence lease generation immediately without blocking on write lock"
            );
            assert!(
                sup.has_pending_disconnect_hold(),
                "contended legacy close must queue fence and activate fail-closed hold"
            );
            let writes_before = ctrl.write_targets_calls.load(Ordering::SeqCst);
            guard.tick_control(0.02);
            let writes_after = ctrl.write_targets_calls.load(Ordering::SeqCst);
            assert_eq!(
                writes_after, writes_before,
                "zero bus output from fenced generation may reach driver while fail-closed hold is active"
            );
            assert_eq!(
                guard.current_active_lease_gen(),
                0,
                "fenced lease must not remain valid"
            );
            drop(guard);
            // Once lock is released, pending disconnect drains cleanly and hold clears.
            sup.control_tick(0.02).await;
            wait_until(Duration::from_secs(2), || {
                !sup.has_pending_disconnect_hold()
            })
            .await;
            let snap = sup.fresh_snapshot(None).await.unwrap();
            assert_eq!(snap.owner.lease_generation, 0);
        } else {
            // Executor: start a stalled enable transition (after disabling first) OR verify live
            // disconnect immediately fences `g1`, aborts in-flight enable, and emits zero further
            // enable polls / bus writes for `g1`.
            assert_eq!(
                ws_reply(
                    &mut ws1,
                    json!({"type": "command", "request_id": 2, "generation": g1, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": false}}),
                    &["command_ack"],
                )
                .await["is_torqued"],
                false
            );
            ctrl.allow_enable_confirm.store(false, Ordering::SeqCst);
            send(
                &mut ws1,
                json!({"type": "command", "request_id": 3, "generation": g1, "epoch_id": ep, "path": "/api/v1/torque", "body": {"enable": true}}),
            )
            .await;
            wait_until(Duration::from_secs(2), || {
                ctrl.begin_enable_calls.load(Ordering::SeqCst) >= 2
            })
            .await;
            let _ = ws1.close(None).await;
            drop(ws1);
            wait_until(Duration::from_secs(2), || {
                sup.atomic_fenced_lease_gen() >= g1
                    && handle.as_ref().unwrap().live_connection_slots() == 0
            })
            .await;
            let snap = sup.fresh_snapshot(None).await.unwrap();
            assert_eq!(snap.owner.lease_generation, 0);
            assert!(snap.owner.fenced_lease_generation >= g1);
            assert!(
                ctrl.abort_enable_calls.load(Ordering::SeqCst) >= 1,
                "executor socket close must abort in-flight enable for fenced generation"
            );
        }

        if let Some(h) = handle {
            h.stop_and_join();
        }
    }

    // Part B: Live owner transfer (`OWNER_TRANSFERRING`) with the legacy write lock held for
    // > 500 ms (650 ms): teardown MUST never return unfenced, MUST queue the fence and fail
    // closed (hold), and RT MUST NEVER inherit the stale connection or lease once transfer finishes.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let ctrl = Arc::new(L2DriverControl::default());
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        sup.set_driver(Box::new(L2MockDriver::new(&cfg, None, Arc::clone(&ctrl))))
            .await;
        sup.control_tick(0.02).await;

        let conn = ConnectionHandle::from_ws_id(42).unwrap();
        sup.register_ws_connection(conn.id()).await.unwrap();
        let grant = sup
            .acquire_lease_ws(conn, "m1_transfer_holder", 1)
            .await
            .unwrap();
        let LeaseOutcome::Granted { generation: g1, .. } = grant else {
            panic!("expected lease grant, got {grant:?}");
        };
        // Enable torque under the active lease so bus writes would occur if unfenced.
        {
            let mut inner = sup.inner().write().await;
            let stamp = inner.current_admission_stamp();
            let admitted = inner.check_admission(stamp).unwrap();
            let _ = SupervisorInner::enable_torque(admitted).unwrap();
        }
        sup.control_tick(0.02).await;
        assert!(ctrl.write_targets_calls.load(Ordering::SeqCst) >= 1);

        // Hold the legacy write lock for > 500 ms while starting ownership transfer.
        let mut write_guard = sup.inner().write().await;

        let sup_transfer = Arc::clone(&sup);
        let transfer_task = tokio::spawn(async move {
            sup_transfer
                .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                .await
        });
        wait_until(Duration::from_secs(2), || {
            sup.owner_mode() == armd::supervisor::OWNER_TRANSFERRING
        })
        .await;

        // Run disconnect teardown during `OWNER_TRANSFERRING` while write_guard stays held for 650 ms.
        let sup_disc = Arc::clone(&sup);
        let disc_task = tokio::spawn(async move {
            sup_disc.fence_ws_connection_immediate(conn, g1);
            sup_disc.disconnect_ws_connection(conn).await
        });

        // Wait past the 500 ms settle bound (650 ms total) while STILL holding write_guard.
        tokio::time::sleep(Duration::from_millis(650)).await;
        let _disc_out = tokio::time::timeout(Duration::from_secs(2), disc_task)
            .await
            .expect("disconnect_ws_connection must return bounded even while transfer lock is held > 500 ms")
            .unwrap();

        // Even though disconnect_ws_connection hit its 500 ms bound while the lock was held:
        // it MUST NEVER return unfenced — connection is fenced closed, lease gen fenced,
        // queued to owner, and fail-closed hold blocks bus output!
        assert!(
            sup.is_connection_fenced_closed(conn.id()),
            "teardown during >500ms transfer lock must fence connection closed"
        );
        assert!(
            sup.atomic_fenced_lease_gen() >= g1,
            "teardown during >500ms transfer lock must fence lease generation"
        );
        assert!(
            sup.has_pending_disconnect_hold(),
            "teardown during >500ms transfer lock must queue fence and fail closed (hold)"
        );
        let writes_before = ctrl.write_targets_calls.load(Ordering::SeqCst);
        write_guard.tick_control(0.02);
        let writes_after = ctrl.write_targets_calls.load(Ordering::SeqCst);
        assert_eq!(
            writes_after, writes_before,
            "fail-closed hold must block bus output while queued disconnect fence is pending"
        );

        // Now release write_guard so transfer to RT executor completes.
        drop(write_guard);
        let handle = transfer_task
            .await
            .unwrap()
            .expect("ownership transfer must succeed cleanly after write_guard drops");
        assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

        // Verify RT NEVER inherited the stale connection or lease!
        assert!(
            !sup.has_pending_disconnect_hold(),
            "transfer handoff must drain all queued disconnect fences before spawning RT"
        );
        assert_eq!(
            handle.live_connection_slots(),
            0,
            "RT must never inherit a stale closed connection"
        );
        let snap = sup.fresh_snapshot(None).await.unwrap();
        assert_eq!(
            snap.owner.lease_generation, 0,
            "RT must never inherit a stale lease from a connection closed during transfer"
        );
        assert!(
            snap.owner.lease_owner.is_none(),
            "RT must have no lease owner after transfer drain"
        );
        assert!(
            snap.owner.fenced_lease_generation >= g1,
            "RT must preserve the fenced generation across handoff"
        );
        // Stale connection cannot re-open or renew on RT.
        assert!(
            sup.register_ws_connection(conn.id()).await.is_err(),
            "stale closed connection ID must be rejected by RT"
        );
        // A brand-new connection can cleanly open and acquire a fresh generation > g1 on RT.
        let conn2 = ConnectionHandle::from_ws_id(43).unwrap();
        sup.register_ws_connection(conn2.id()).await.unwrap();
        let g2 = match sup
            .acquire_lease_ws(conn2, "m1_fresh_rt", 1)
            .await
            .unwrap()
        {
            LeaseOutcome::Granted { generation, .. } => generation,
            other => panic!("expected fresh lease on RT, got {other:?}"),
        };
        assert!(g2 > g1);
        let _ = sup.disconnect_ws_connection(conn2).await;
        handle.stop_and_join();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn m2_raw_broker_rejects_unregistered_client_ids_and_binds_client_to_lease_holder() {
    use armd::rt_core::ops::Primitive;
    use armd::rt_core::requests::actions::{ActionSubmit, BeginMacroSpec, MacroKind};

    // (a) Deterministic raw Broker + ReleaseFixture: unregistered non-zero IDs are rejected even
    //     with `lease_enabled = false` (no `require_registered = false` bypass), and when
    //     `lease_enabled = true`, raw broker descriptors MUST belong to a registered connection
    //     AND match the current lease holder (`active_lease_conn_id`), not only `lease_generation`.
    {
        let mut inner = SupervisorInner::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        });
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new(
            inner,
            BootId(61),
            Capacities { commands: 8, plans: 2, replies: 16 },
        );
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        let dl = fx.now() + Duration::from_secs(5);

        // Even with lease_enabled = false, unregistered non-zero ClientId(777) MUST be refused.
        b.submit(ClientId(777), 1, a0, dl, Submit::Immediate(ImmediateOp::TorqueOn))
            .unwrap();
        assert_eq!(fx.step().command.unwrap().1, Err("DISCONNECTED"));
        assert!(!fx.core.is_torqued);

        // Registered ClientId(1) succeeds.
        b.submit(ClientId(1), 1, a0, dl, Submit::Immediate(ImmediateOp::TorqueOn))
            .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert!(fx.core.is_torqued);
    }

    // (b) Live RT executor raw `handle.broker()` attack surface:
    //     c1 (id=10) holds lease g1; c2 (id=11) is a registered live non-holder; id=999 is
    //     unregistered; id=0 is zero sentinel.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        }));
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
            .await
            .unwrap();
        wait_until(Duration::from_secs(2), || sup.hub().latest().is_some()).await;

        let c1 = ConnectionHandle::from_ws_id(10).unwrap();
        let c2 = ConnectionHandle::from_ws_id(11).unwrap();
        sup.register_ws_connection(c1.id()).await.unwrap();
        sup.register_ws_connection(c2.id()).await.unwrap();
        let g1 = match sup.acquire_lease_ws(c1, "m2_holder", 1).await.unwrap() {
            LeaseOutcome::Granted { generation, .. } => generation,
            other => panic!("expected lease grant for c1, got {other:?}"),
        };

        let snap = sup.fresh_snapshot(None).await.unwrap();
        let valid_auth = Authority {
            lease_generation: g1,
            epoch: snap.owner.epoch,
            stop_generation: snap.owner.stop_generation,
            config_revision: snap.owner.config_revision,
            parent_generation: 0,
        };
        let dl = Instant::now() + Duration::from_secs(5);
        let submit_raw = |client: ClientId, seq: u64, what: Submit<'_, armd::rt_core::ops::PlanSlot>| {
            let req = handle
                .broker()
                .lock()
                .unwrap()
                .submit(client, seq, valid_auth, dl, what)
                .unwrap();
            let deadline = Instant::now() + Duration::from_secs(2);
            loop {
                let mut got = None;
                handle.broker().lock().unwrap().poll(|_, r| {
                    if r.request() == req {
                        got = Some(r);
                    }
                    true
                });
                if let Some(r) = got {
                    return r;
                }
                assert!(Instant::now() < deadline, "timed out waiting for raw broker reply");
                std::thread::sleep(Duration::from_millis(2));
            }
        };

        // 1. Unregistered ClientId(999) with valid lease_generation g1 MUST fail DISCONNECTED across
        //    Heartbeat, TorqueOn, TorqueOff, Install, and Action(BeginMacro).
        assert!(matches!(
            submit_raw(ClientId(999), 1, Submit::Immediate(ImmediateOp::Heartbeat)),
            Reply::Refused { code: "DISCONNECTED", .. }
        ));
        assert!(matches!(
            submit_raw(ClientId(999), 2, Submit::Immediate(ImmediateOp::TorqueOn)),
            Reply::Refused { code: "DISCONNECTED", .. }
        ));
        assert!(matches!(
            submit_raw(ClientId(999), 3, Submit::Immediate(ImmediateOp::TorqueOff)),
            Reply::Refused { code: "DISCONNECTED", .. }
        ));
        let q = [0.0f32; 7];
        let mut q2 = q;
        q2[0] = 0.05;
        let prim = Primitive::joint_quintic(&q, &q2, 0.5).unwrap();
        assert!(matches!(
            submit_raw(ClientId(999), 4, Submit::Install(&|s| s.set(prim))),
            Reply::Refused { code: "DISCONNECTED", .. }
        ));
        let macro_spec = BeginMacroSpec {
            kind: MacroKind::Grasp,
            prepared_boot: handle.boot(),
            admission_request_id: 77_001,
            check_start: false,
            dof: 7,
            start: [0.0; armd::rt_core::mailbox::MAX_DOF],
            check_workspace_revision: false,
            calibration_revision: None,
            poses_revision: None,
            sequences_revision: None,
            grasp_setup_revision: None,
            gripper_calib_revision: None,
            power_budget_revision: None,
            macro_deadline: None,
            timeout_code: armd::safety::BoundedText::new("MACRO_TIMEOUT"),
            orientation_levelled: false,
            j7_intent_only: false,
            task: armd::model::HolderLabel::new(""),
            tag: armd::model::HolderLabel::new(""),
        };
        assert!(matches!(
            submit_raw(ClientId(999), 5, Submit::Action(ActionSubmit::BeginMacro(macro_spec))),
            Reply::Refused { code: "DISCONNECTED", .. }
        ));

        // 2. Zero ClientId(0) with valid lease_generation g1 MUST fail DISCONNECTED.
        assert!(matches!(
            submit_raw(ClientId(0), 1, Submit::Immediate(ImmediateOp::TorqueOn)),
            Reply::Refused { code: "DISCONNECTED", .. }
        ));

        // 3. Registered NON-HOLDER ClientId(c2) presenting holder c1's valid generation g1 MUST
        //    fail LEASE_LOST across Heartbeat, TorqueOn, TorqueOff, Install, and Action!
        assert!(matches!(
            submit_raw(ClientId(c2.id()), 1, Submit::Immediate(ImmediateOp::Heartbeat)),
            Reply::Refused { code: "LEASE_LOST", .. }
        ));
        assert!(matches!(
            submit_raw(ClientId(c2.id()), 2, Submit::Immediate(ImmediateOp::TorqueOn)),
            Reply::Refused { code: "LEASE_LOST", .. }
        ));
        assert!(matches!(
            submit_raw(ClientId(c2.id()), 3, Submit::Immediate(ImmediateOp::TorqueOff)),
            Reply::Refused { code: "LEASE_LOST", .. }
        ));
        assert!(matches!(
            submit_raw(ClientId(c2.id()), 4, Submit::Install(&|s| s.set(prim))),
            Reply::Refused { code: "LEASE_LOST", .. }
        ));
        let macro_spec2 = BeginMacroSpec {
            admission_request_id: 77_002,
            ..macro_spec
        };
        assert!(matches!(
            submit_raw(ClientId(c2.id()), 5, Submit::Action(ActionSubmit::BeginMacro(macro_spec2))),
            Reply::Refused { code: "LEASE_LOST", .. }
        ));
        assert!(!sup.fresh_snapshot(None).await.unwrap().owner.is_torqued);

        // 4. Registered HOLDER ClientId(c1) with generation g1 commits cleanly!
        assert!(matches!(
            submit_raw(ClientId(c1.id()), 1, Submit::Immediate(ImmediateOp::Heartbeat)),
            Reply::Ack { .. }
        ));
        assert!(matches!(
            submit_raw(ClientId(c1.id()), 2, Submit::Immediate(ImmediateOp::TorqueOn)),
            Reply::Ack { .. }
        ));
        assert!(sup.fresh_snapshot(None).await.unwrap().owner.is_torqued);

        let _ = sup.disconnect_ws_connection(c1).await;
        let _ = sup.disconnect_ws_connection(c2).await;
        handle.stop_and_join();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn m3_clear_fault_rechecks_authority_before_hardware_reset_and_final_clear() {
    use armd::rt_core::requests::lease::install_lease_clear_fault_legacy;

    struct M3HookDriver {
        dof: usize,
        positions: Vec<f32>,
        hw_fault: Arc<AtomicBool>,
        clear_fault_calls: Arc<AtomicUsize>,
        on_preflight: Arc<Mutex<Option<Box<dyn FnOnce() + Send>>>>,
    }

    impl ArmDriver for M3HookDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                enable_feedback: true,
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                ..DriverCapabilities::default()
            }
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            if let Some(cb) = self.on_preflight.lock().unwrap().take() {
                cb();
            }
            Ok(self
                .positions
                .iter()
                .map(|&p| JointState {
                    position: p,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                })
                .collect())
        }
        fn read_joint_states_budgeted(
            &mut self,
            out: &mut Vec<JointState>,
            _deadline: Instant,
            _max_rx_frames: u16,
        ) -> Result<(u16, bool), ArmDriverError> {
            out.clear();
            for &p in &self.positions {
                out.push(JointState {
                    position: p,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                });
            }
            Ok((self.dof as u16, false))
        }
        fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
            out.clear();
            for &p in &self.positions {
                out.push(JointState {
                    position: p,
                    velocity: 0.0,
                    torque: 0.0,
                    temperature: 30.0,
                });
            }
            Some(())
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn is_enabled(&self) -> bool {
            false
        }
        fn enable_state(&self) -> EnableState {
            EnableState::Disabled
        }
        fn is_feedback_valid(&self) -> bool {
            true
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            vec![Some(Duration::from_millis(1)); self.dof]
        }
        fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
            if let Some(cb) = self.on_preflight.lock().unwrap().take() {
                cb();
            }
            out.clear();
            out.resize(self.dof, Some(Duration::from_millis(1)));
        }
        fn check_hardware_status(&self) -> Result<(), ArmDriverError> {
            if self.hw_fault.load(Ordering::SeqCst) {
                Err(ArmDriverError::HardwareFault("motor fault latched".into()))
            } else {
                Ok(())
            }
        }
        fn has_hardware_fault(&self) -> bool {
            self.hw_fault.load(Ordering::SeqCst)
        }
        fn clear_fault(&mut self) -> Result<(), ArmDriverError> {
            self.clear_fault_calls.fetch_add(1, Ordering::SeqCst);
            self.hw_fault.store(false, Ordering::SeqCst);
            Ok(())
        }
        fn clear_fault_budgeted(&mut self, _deadline: Instant) -> Result<(), ArmDriverError> {
            self.clear_fault_calls.fetch_add(1, Ordering::SeqCst);
            self.hw_fault.store(false, Ordering::SeqCst);
            Ok(())
        }
    }

    let make_case = |hw_fault_init: bool, estop: bool| {
        let cfg = ArmConfig {
            lease_timeout_ms: 5_000,
            ..ArmConfig::rebot_dm()
        };
        let hw_fault = Arc::new(AtomicBool::new(hw_fault_init));
        let clear_fault_calls = Arc::new(AtomicUsize::new(0));
        let on_preflight: Arc<Mutex<Option<Box<dyn FnOnce() + Send>>>> =
            Arc::new(Mutex::new(None));
        let drv = M3HookDriver {
            dof: cfg.dof,
            positions: cfg.resting_positions().to_vec(),
            hw_fault: Arc::clone(&hw_fault),
            clear_fault_calls: Arc::clone(&clear_fault_calls),
            on_preflight: Arc::clone(&on_preflight),
        };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.operator_auth_enforced = true;
        inner.feedback_numeric_valid = true;
        inner.tick(0.02);
        let c1 = ConnectionHandle::from_ws_id(1).unwrap();
        inner.register_live_connection(c1.id()).unwrap();
        let now = Instant::now();
        let ctx = LeaseInstallCtx {
            expected_arm: armd::rt_core::ids::ArmId::LOCAL,
            expected_boot: BootId(71),
            stamp_arm: armd::rt_core::ids::ArmId::LOCAL,
            stamp_boot: BootId(71),
            now,
            request_id: 90_001,
            client: ClientId(c1.id()),
            deadline: now + Duration::from_secs(5),
            enforce_single_use: false,
        };
        let LeaseOutcome::Granted { generation: g1, .. } = install_lease_acquire(
            &mut inner,
            &LeaseAcquire {
                connection: c1,
                label: armd::model::HolderLabel::new("m3_holder"),
                has_label: true,
            },
            &ctx,
        )
        .unwrap() else {
            panic!("expected lease grant");
        };
        if estop {
            inner.safety.trigger_e_stop();
            inner.e_stop_latched.store(true, Ordering::SeqCst);
        } else {
            // Non-recoverable HardwareFault
            inner.safety.latch(
                armd::FaultKind::Other,
                armd::safety::FaultDetail::formatted(armd::FaultCode::HardwareFault, |buf| {
                    buf.push_str("m3 non-recoverable hardware fault");
                }),
            );
        }
        assert!(
            !inner.safety.recoverable(),
            "fault under test must be non-recoverable"
        );
        (inner, c1, g1, hw_fault, clear_fault_calls, on_preflight)
    };

    // 1. Lock operator session DURING driver preflight -> NO hardware reset, NO clear (BOTH owners).
    for use_rt in [false, true] {
        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok, _) = auth.unlock("123456").unwrap();
        let grant = auth.operator_grant(Some(&tok));
        let (mut inner, c1, g1, hw_fault, clear_calls, on_preflight) = make_case(true, false);
        let auth_lock = auth.clone();
        let tok_lock = tok.clone();
        *on_preflight.lock().unwrap() = Some(Box::new(move || {
            auth_lock.lock(Some(&tok_lock));
        }));
        let ep = inner.safety.epoch_id();
        let now = Instant::now();
        let out = if use_rt {
            install_lease_clear_fault_rt(
                &mut inner,
                c1,
                g1,
                Some(ep),
                true,
                None,
                OwnerAuthPolicy::ClearFault,
                grant,
                now,
                now + Duration::from_millis(50),
            )
        } else {
            install_lease_clear_fault_legacy(
                &mut inner,
                c1,
                g1,
                Some(ep),
                true,
                None,
                false,
                OwnerAuthPolicy::ClearFault,
                grant,
                now,
            )
            .unwrap()
        };
        assert!(
            matches!(out, LeaseClearFaultOutcome::OperatorRequired { .. }),
            "locking operator session during preflight must abort before hardware reset (use_rt={use_rt}): {out:?}"
        );
        assert_eq!(
            clear_calls.load(Ordering::SeqCst),
            0,
            "hardware reset MUST NOT be sent once operator session is locked (use_rt={use_rt})"
        );
        assert!(hw_fault.load(Ordering::SeqCst), "hardware fault must stay untouched");
        assert!(
            inner.safety.state().is_stopped(),
            "safety fault must NOT clear when session locked during preflight (use_rt={use_rt})"
        );
    }

    // 2. Lock operator session DURING driver preflight before final clear (no hw reset needed,
    //    E-stop latched) -> NO clear on BOTH owners.
    for use_rt in [false, true] {
        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok, _) = auth.unlock("123456").unwrap();
        let grant = auth.operator_grant(Some(&tok));
        let (mut inner, c1, g1, _hw_fault, clear_calls, on_preflight) = make_case(false, true);
        let auth_lock = auth.clone();
        let tok_lock = tok.clone();
        *on_preflight.lock().unwrap() = Some(Box::new(move || {
            auth_lock.lock(Some(&tok_lock));
        }));
        let ep = inner.safety.epoch_id();
        let now = Instant::now();
        let out = if use_rt {
            install_lease_clear_fault_rt(
                &mut inner,
                c1,
                g1,
                Some(ep),
                true,
                None,
                OwnerAuthPolicy::ClearFault,
                grant,
                now,
                now + Duration::from_millis(50),
            )
        } else {
            install_lease_clear_fault_legacy(
                &mut inner,
                c1,
                g1,
                Some(ep),
                true,
                None,
                false,
                OwnerAuthPolicy::ClearFault,
                grant,
                now,
            )
            .unwrap()
        };
        assert!(
            matches!(out, LeaseClearFaultOutcome::OperatorRequired { .. }),
            "locking operator session during preflight must abort before final clear (use_rt={use_rt}): {out:?}"
        );
        assert_eq!(clear_calls.load(Ordering::SeqCst), 0);
        assert_eq!(
            *inner.safety.state(),
            SafetyState::EStop,
            "E-stop must stay latched when session locked during preflight (use_rt={use_rt})"
        );
    }

    // 3. Non-recoverable fault NEVER clears with `OwnerAuthPolicy::None` (non-explicit clear) or
    //    `OperatorGrant::DENIED`, even if `operator_auth_enforced == false`.
    {
        let (mut inner, c1, g1, _hw_fault, clear_calls, _on_preflight) = make_case(false, false);
        inner.operator_auth_enforced = false;
        let ep = inner.safety.epoch_id();
        let now = Instant::now();
        let out_non_explicit = install_lease_clear_fault_rt(
            &mut inner,
            c1,
            g1,
            Some(ep),
            true,
            None,
            OwnerAuthPolicy::None,
            OperatorGrant::unenforced(),
            now,
            now + Duration::from_millis(50),
        );
        assert!(
            matches!(out_non_explicit, LeaseClearFaultOutcome::OperatorRequired { .. }),
            "non-recoverable fault must never clear without explicit operator clear policy: {out_non_explicit:?}"
        );
        let out_denied = install_lease_clear_fault_rt(
            &mut inner,
            c1,
            g1,
            Some(ep),
            true,
            None,
            OwnerAuthPolicy::ClearFault,
            OperatorGrant::DENIED,
            now,
            now + Duration::from_millis(50),
        );
        assert!(
            matches!(out_denied, LeaseClearFaultOutcome::OperatorRequired { .. }),
            "non-recoverable fault must never clear without a valid session grant: {out_denied:?}"
        );
        assert_eq!(clear_calls.load(Ordering::SeqCst), 0);
        assert!(inner.safety.state().is_stopped());

        // Positive control: explicit operator clear with valid live session succeeds (and triggers
        // hardware reset when needed).
        let auth = OperatorAuth::enforced("123456", 1800);
        let (tok, _) = auth.unlock("123456").unwrap();
        let grant = auth.operator_grant(Some(&tok));
        let (mut inner2, c2, g2, hw_fault2, clear_calls2, _hook2) = make_case(true, false);
        let ep2 = inner2.safety.epoch_id();
        let now2 = Instant::now();
        let reset_step = install_lease_clear_fault_rt(
            &mut inner2,
            c2,
            g2,
            Some(ep2),
            true,
            None,
            OwnerAuthPolicy::ClearFault,
            grant,
            now2,
            now2 + Duration::from_millis(50),
        );
        assert!(
            matches!(reset_step, LeaseClearFaultOutcome::ClearFailed { .. }),
            "first call with hw fault sends reset and requires fresh post-reset confirmation: {reset_step:?}"
        );
        assert_eq!(clear_calls2.load(Ordering::SeqCst), 1, "valid session must reach hardware reset");
        assert!(!hw_fault2.load(Ordering::SeqCst));
        let cleared = install_lease_clear_fault_rt(
            &mut inner2,
            c2,
            g2,
            Some(ep2),
            true,
            None,
            OwnerAuthPolicy::ClearFault,
            grant,
            now2,
            now2 + Duration::from_millis(50),
        );
        assert!(
            matches!(cleared, LeaseClearFaultOutcome::Cleared { safety_state: SafetyState::Idle, .. }),
            "second call after hw reset with valid session clears cleanly: {cleared:?}"
        );
    }
}

#[test]
fn m4_disable_confirm_timeout_retries_in_background_preserving_original_timestamp() {
    ensure_tracing_subscriber();

    // (a) Decision-based disable: `Ok(Done(Unconfirmed))` completes `TorqueOff` immediately with
    //     no `DisableConfirm` phase, while a transport-failed `TorqueOff` arms
    //     `BackgroundDisableRetry` and recovers once the bus clears.
    {
        let cfg = ArmConfig {
            lease_timeout_ms: 60_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let ctrl = Arc::new(L2DriverControl::default());
        ctrl.allow_enable_confirm.store(true, Ordering::SeqCst);
        ctrl.allow_disable_confirm.store(false, Ordering::SeqCst);
        ctrl.disable_always_unconfirmed.store(true, Ordering::SeqCst);

        let drv = L2MockDriver::new(&cfg, Some(mclock.clone()), ctrl.clone());
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.prewarm_rt_buffers();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(41),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        // Enable torque first.
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            1,
            a0,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(fx.step().transition_ended.map(|(_, o)| o), Some(Outcome::Completed));
        assert!(fx.core.is_torqued);

        // `Done(Unconfirmed)` completes immediately (`torque_intent == Off`, no `DisableConfirm`).
        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            2,
            a1,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();
        let r_unconf = fx.step();
        assert!(r_unconf.command.unwrap().1.is_ok());
        assert_eq!(fx.step().transition_ended.map(|(_, o)| o), Some(Outcome::Completed));
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), 1);
        assert!(!fx.core.disable_pending);
        assert!(!fx.core.is_torqued);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::Off);

        // Re-arm and trigger `BackgroundDisableRetry` via transient transport errors on `TorqueOff`.
        let a2 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            3,
            a2,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(fx.step().transition_ended.map(|(_, o)| o), Some(Outcome::Completed));
        assert!(fx.core.is_torqued);

        ctrl.disable_always_unconfirmed.store(false, Ordering::SeqCst);
        ctrl.allow_disable_confirm.store(true, Ordering::SeqCst);
        ctrl.fail_disable_begin_remaining.store(3, Ordering::SeqCst);
        let a3 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(
            ClientId(1),
            4,
            a3,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOff),
        )
        .unwrap();
        let r_err = fx.step();
        assert!(r_err.command.unwrap().1.is_ok());
        assert_eq!(
            r_err.transition_ended.map(|(_, o)| o),
            Some(Outcome::Failed("DRIVER_WRITE_FAILED"))
        );
        assert!(fx.core.disable_pending);
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), 2);

        for _ in 0..12 {
            let _ = fx.step();
            if !fx.core.disable_pending {
                break;
            }
        }
        assert!(!fx.core.disable_pending, "background retry must complete once transport recovers");
        assert!(!fx.core.is_torqued);
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::Off);
        assert_eq!(ctrl.begin_disable_calls.load(Ordering::SeqCst), 5);
    }

    // (b) Original `DisableRequested { started_at }` timestamp preservation: when the initial
    //     pending disable consumes 400 ms before failing and retry #1 consumes another 450 ms,
    //     EVERY retry receives the original `t0` timestamp and the retry window expires at
    //     `t0 + STOPPED_DISABLE_REQUEST_LIFETIME` (1 s) — NOT a fresh window!
    {
        struct SlowPendingDisableDriver {
            dof: usize,
            positions: Vec<f32>,
            enabled: bool,
            clock: MockClock,
            handshake_end: Option<Instant>,
            disable_calls: Arc<AtomicUsize>,
            seen_started_at: Arc<Mutex<Vec<Instant>>>,
        }

        impl ArmDriver for SlowPendingDisableDriver {
            fn capabilities(&self) -> DriverCapabilities {
                DriverCapabilities {
                    enable_feedback: true,
                    velocity_feedback: true,
                    torque_feedback: true,
                    temperature_feedback: true,
                    ..DriverCapabilities::default()
                }
            }
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
                Ok(self
                    .positions
                    .iter()
                    .map(|&p| JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 })
                    .collect())
            }
            fn read_joint_states_budgeted(
                &mut self,
                out: &mut Vec<JointState>,
                _deadline: Instant,
                _max_rx_frames: u16,
            ) -> Result<(u16, bool), ArmDriverError> {
                out.clear();
                for &p in &self.positions {
                    out.push(JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 });
                }
                Ok((self.dof as u16, false))
            }
            fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
                out.clear();
                for &p in &self.positions {
                    out.push(JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 });
                }
                Some(())
            }
            fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
                self.enabled = enable;
                Ok(())
            }
            fn begin_disable_torque_budgeted(
                &mut self,
                kind: armd::driver::DisableKind,
                _: u16,
            ) -> Result<DrvTorqueProgress, ArmDriverError> {
                if let armd::driver::DisableKind::DisableRequested { started_at, .. } = kind {
                    self.seen_started_at.lock().unwrap().push(started_at);
                } else {
                    panic!("expected DisableRequested, got {kind:?}");
                }
                let call = self.disable_calls.fetch_add(1, Ordering::SeqCst) + 1;
                if call == 1 {
                    let end = self.clock.now() + Duration::from_millis(400);
                    self.handshake_end = Some(end);
                    Ok(DrvTorqueProgress::Pending { wake: self.clock.now() + Duration::from_millis(20) })
                } else if call == 2 {
                    let end = self.clock.now() + Duration::from_millis(450);
                    self.handshake_end = Some(end);
                    Ok(DrvTorqueProgress::Pending { wake: self.clock.now() + Duration::from_millis(20) })
                } else {
                    Err(ArmDriverError::Transport(TransportError::WouldBlock))
                }
            }
            fn begin_torque_budgeted(&mut self, enable: bool, _: u16) -> Result<DrvTorqueProgress, ArmDriverError> {
                if enable {
                    self.enabled = true;
                    return Ok(DrvTorqueProgress::Done(TorqueOutcome::Confirmed));
                }
                unreachable!("disable must route through begin_disable_torque_budgeted")
            }
            fn poll_torque_budgeted(&mut self, _: Instant, _: u16) -> Result<DrvTorqueProgress, ArmDriverError> {
                if let Some(end) = self.handshake_end {
                    if self.clock.now() < end {
                        return Ok(DrvTorqueProgress::Pending { wake: self.clock.now() + Duration::from_millis(20) });
                    }
                    self.handshake_end = None;
                    return Err(ArmDriverError::Transport(TransportError::WouldBlock));
                }
                Err(ArmDriverError::Transport(TransportError::WouldBlock))
            }
            fn torque_transition_pending(&self) -> bool { self.handshake_end.is_some() }
            fn is_enabled(&self) -> bool { self.enabled }
            fn enable_state(&self) -> EnableState {
                if self.enabled { EnableState::Enabled } else { EnableState::Disabled }
            }
            fn is_feedback_valid(&self) -> bool { true }
            fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![Some(Duration::from_millis(1)); self.dof] }
            fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
                out.clear();
                out.resize(self.dof, Some(Duration::from_millis(1)));
            }
            fn has_driver_fresh_position_api(&self) -> bool {
                true
            }
            fn fresh_joint_positions_into(
                &self,
                joints: &[usize],
                max_age: Duration,
                out: &mut [f32],
            ) -> bool {
                if joints.is_empty() || out.len() < joints.len() || Duration::from_millis(1) > max_age {
                    return false;
                }
                for (dst, &j) in out.iter_mut().zip(joints.iter()) {
                    let Some(&p) = self.positions.get(j) else {
                        return false;
                    };
                    *dst = p;
                }
                true
            }
            fn clock_now(&self) -> Instant { self.clock.now() }
        }

        let cfg = ArmConfig {
            lease_timeout_ms: 60_000,
            ..ArmConfig::rebot_dm()
        };
        let mclock = MockClock::new();
        let disable_calls = Arc::new(AtomicUsize::new(0));
        let seen_started_at = Arc::new(Mutex::new(Vec::new()));
        let drv = SlowPendingDisableDriver {
            dof: cfg.dof,
            positions: cfg.resting_positions().to_vec(),
            enabled: false,
            clock: mclock.clone(),
            handshake_end: None,
            disable_calls: Arc::clone(&disable_calls),
            seen_started_at: Arc::clone(&seen_started_at),
        };
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.rt_executor_owned = true;
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        inner.prewarm_rt_buffers();

        let (mut fx, mut b, _tw, _st) = ReleaseFixture::new_with_clock(
            inner,
            BootId(42),
            Capacities { commands: 8, plans: 1, replies: 16 },
            VirtualClock::from_mock_clock(mclock.clone()),
        );
        // Enable torque first.
        let a0 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(ClientId(1), 1, a0, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOn)).unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert!(fx.core.is_torqued);

        // Start slow disable at t0: initial handshake runs 400 ms before failing (`< 500 ms`
        // `BACKGROUND_DISABLE_MAX_WINDOW`), arming `BackgroundDisableRetry` with `started_at == t0`.
        let t0 = mclock.now();
        let a1 = fx.core.authority(&fx.ctx(fx.now()));
        b.submit(ClientId(1), 2, a1, fx.now() + Duration::from_secs(5), Submit::Transition(TransitionOp::TorqueOff)).unwrap();
        assert!(fx.step().command.unwrap().1.is_ok());
        assert_eq!(disable_calls.load(Ordering::SeqCst), 1);

        let mut failed_at = None;
        for _ in 0..40 {
            let r = fx.step();
            if r.transition_ended.map(|(_, o)| o) == Some(Outcome::Failed("DRIVER_WRITE_FAILED")) {
                failed_at = Some(mclock.now());
                break;
            }
        }
        let failed_at = failed_at.expect("slow disable must fail after 400 ms");
        let elapsed_at_fail = failed_at.saturating_duration_since(t0);
        assert!(
            elapsed_at_fail >= Duration::from_millis(400) && elapsed_at_fail < Duration::from_millis(460),
            "expected failure around 400ms from t0, got {elapsed_at_fail:?}"
        );

        // Step well past `t0 + STOPPED_DISABLE_REQUEST_LIFETIME` (1 s): call #2 consumes 450 ms
        // (`t0+440ms -> t0+890ms`), leaving only ~100 ms before `t0 + 1 s` -> only 1..=2 extra
        // retries can run (3..=4 total calls, NEVER 9 calls from a reset window!).
        for _ in 0..40 {
            let _ = fx.step();
        }
        let total_calls = disable_calls.load(Ordering::SeqCst);
        assert!(
            (3..=5).contains(&total_calls),
            "retries must honor original DisableRequested timestamp (1s window from t0 => 1 initial + slow retry + <=3 fast retries), got {total_calls} total calls"
        );
        let stamps = seen_started_at.lock().unwrap().clone();
        assert_eq!(stamps.len(), total_calls);
        assert!(
            stamps.iter().all(|&ts| ts == stamps[0]),
            "every retry must preserve original DisableRequested {{ started_at }}: {stamps:?}"
        );
        assert!(
            !fx.core.disable_pending,
            "stopped DisableRequested must terminate when t0 + STOPPED_DISABLE_REQUEST_LIFETIME expires"
        );
        assert_eq!(fx.core.torque_intent(), armd::TorqueIntent::On);
        assert!(matches!(fx.core.safety.state(), SafetyState::Fault(_)));
        assert!(fx.core.safety.has_latched_fault_code(armd::safety::FaultCode::DriverWriteFailed));
        // Advancing even further must produce zero extra calls (exhausted permanently).
        for _ in 0..20 {
            let _ = fx.step();
        }
        assert_eq!(disable_calls.load(Ordering::SeqCst), total_calls);
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn m5_concurrent_estop_latch_never_overwritten_by_prior_clear() {
    ensure_tracing_subscriber();

    // (a) Direct single-AtomicU64 `EstopState` invariants: generation + latch bit live in one
    //     word; clearing an older generation fails CAS without ever touching bit 0.
    {
        let st = armd::rt_core::urgent::EstopState::new();
        assert_eq!(st.snapshot(), (0, false));
        assert_eq!(st.request_and_latch(), 1);
        assert_eq!(st.snapshot(), (1, true));
        assert_eq!(st.request_and_latch(), 2);
        assert_eq!(st.snapshot(), (2, true));
        // Clearing prior gen 1 MUST fail with observed gen 2 and keep latch bit true.
        assert_eq!(st.try_clear_if_generation(1), Err(2));
        assert_eq!(st.snapshot(), (2, true));
        // Clearing current gen 2 succeeds and preserves generation count.
        assert_eq!(st.try_clear_if_generation(2), Ok(()));
        assert_eq!(st.snapshot(), (2, false));
    }

    // (b) Deterministic interleaving across prior E-stop clear vs concurrent `trigger_e_stop_checked`:
    //     Task 1 clears E-stop #1 and pauses inside `before_estop_unlatch_hook` (strictly after
    //     `safety.clear_with_check` committed and strictly before `try_clear_if_generation(1)`).
    //     Task 2 fires a real concurrent `sup.trigger_e_stop_checked()` (which calls
    //     `estop_state.request_and_latch()`, bumping gen 1 -> 2 with bit 0 = 1).
    //     A concurrent poller continuously observes `sup.e_stop_latched.load(SeqCst)` across the
    //     entire clear -> unlatch CAS -> re-apply window and proves bit 0 NEVER drops to `false`.
    {
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
        sup.set_lease_enabled(false).await;
        let (st1, ep1) = sup.trigger_e_stop_checked().await.expect("E-stop #1");
        assert_eq!(st1, SafetyState::EStop);
        assert_eq!(sup.estop_state.snapshot(), (1, true));

        let (hook_entered_tx, hook_entered_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let (allow_unlatch_tx, allow_unlatch_rx) = std::sync::mpsc::sync_channel::<()>(1);
        let allow_unlatch_rx = std::sync::Mutex::new(allow_unlatch_rx);
        let saw_false = Arc::new(AtomicBool::new(false));
        let stop_poller = Arc::new(AtomicBool::new(false));

        // Continuous observer verifying zero transient-false window on `e_stop_latched` once
        // E-stop #2 is requested.
        let estop_for_poller = Arc::clone(&sup.estop_state);
        let saw_false_p = Arc::clone(&saw_false);
        let stop_poller_p = Arc::clone(&stop_poller);
        let poller = std::thread::spawn(move || {
            while !stop_poller_p.load(Ordering::Acquire) {
                let (gen, latched) = estop_for_poller.snapshot();
                if gen >= 2 && !latched {
                    saw_false_p.store(true, Ordering::SeqCst);
                }
                std::hint::spin_loop();
            }
        });

        {
            let mut inner = sup.inner().write().await;
            inner.before_estop_unlatch_hook = Some(Box::new(move || {
                let _ = hook_entered_tx.send(());
                allow_unlatch_rx
                    .lock()
                    .unwrap()
                    .recv_timeout(Duration::from_secs(5))
                    .expect("test harness must release unlatch barrier");
            }));
        }

        let sup_clear = Arc::clone(&sup);
        let clear_task = tokio::spawn(async move {
            sup_clear.clear_fault_for_epoch(Some(ep1)).await
        });

        // Wait deterministically until clear_fault has committed `safety.clear_with_check` and
        // is parked right before `try_clear_if_generation(1)`.
        tokio::task::spawn_blocking(move || {
            hook_entered_rx
                .recv_timeout(Duration::from_secs(5))
                .expect("clear_fault must reach before_estop_unlatch_hook")
        })
        .await
        .unwrap();

        // Now fire concurrent E-stop #2 through the real public API (`begin_e_stop_checked` runs
        // immediately via lock-free `estop_state.request_and_latch()` even while `clear_fault`
        // holds the supervisor write lock!).
        let sup_estop2 = Arc::clone(&sup);
        let estop2_task = tokio::spawn(async move { sup_estop2.trigger_e_stop_checked().await });

        // Wait deterministically until E-stop #2's atomic `request_and_latch()` is visible.
        let wait_deadline = Instant::now() + Duration::from_secs(5);
        while sup.estop_state.requested_count() < 2 {
            assert!(
                Instant::now() < wait_deadline,
                "E-stop #2 did not publish atomic request generation in time"
            );
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        assert_eq!(sup.estop_state.snapshot(), (2, true));

        // Release `clear_fault` to execute `try_clear_if_generation(1)` while E-stop #2 is live.
        allow_unlatch_tx.send(()).unwrap();

        let clear_res = tokio::time::timeout(Duration::from_secs(5), clear_task)
            .await
            .expect("clear_fault must not hang")
            .unwrap()
            .expect("clear_fault returns Ok(EStop) after re-applying concurrent E-stop #2");
        assert_eq!(
            clear_res,
            SafetyState::EStop,
            "clearing prior E-stop #1 must leave concurrent E-stop #2 latched"
        );

        let (st2, ep2) = tokio::time::timeout(Duration::from_secs(5), estop2_task)
            .await
            .expect("trigger_e_stop_checked #2 must resolve cleanly")
            .unwrap()
            .expect("E-stop #2 result");
        assert_eq!(st2, SafetyState::EStop);
        assert!(ep2 > ep1, "E-stop #2 must advance epoch past ep1 ({ep2} <= {ep1})");

        stop_poller.store(true, Ordering::Release);
        poller.join().unwrap();

        assert!(
            !saw_false.load(Ordering::SeqCst),
            "prior E-stop clear transiently overwrote newer E-stop latch to false"
        );
        assert_eq!(sup.estop_state.snapshot(), (2, true));
        assert!(sup.e_stop_latched.load(Ordering::SeqCst));
        {
            let inner = sup.inner().read().await;
            assert_eq!(inner.applied_estop_count, 2);
            assert_eq!(*inner.safety.state(), SafetyState::EStop);
            assert_eq!(inner.safety.epoch_id(), ep2);
        }

        // Positive control: clearing E-stop #2 with the NEW epoch `ep2` and no newer E-stop
        // succeeds and unlatches cleanly.
        let cleared = sup
            .clear_fault_for_epoch(Some(ep2))
            .await
            .expect("clearing E-stop #2 with matching epoch must succeed");
        assert_eq!(cleared, SafetyState::Idle);
        assert_eq!(sup.estop_state.snapshot(), (2, false));
        assert!(!sup.e_stop_latched.load(Ordering::SeqCst));
    }

    // (c) Deterministic interleaving on non-E-stop fault clear: a concurrent E-stop arriving
    //     after `safety.clear_with_check` is never cleared by the non-E-stop fault clear.
    {
        let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
        inner.lease_enabled = false;
        inner.safety.trigger_fault(armd::safety::FaultReason::WatchdogTimeout);
        let fault_ep = inner.safety.epoch_id();
        let estop_ref = Arc::clone(&inner.estop_state);
        inner.before_estop_unlatch_hook = Some(Box::new(move || {
            let gen = estop_ref.request_and_latch();
            assert_eq!(gen, 1);
        }));
        let res = inner
            .clear_fault_for_epoch(Some(fault_ep))
            .expect("post-commit concurrent E-stop returns Ok(EStop)");
        assert_eq!(res, SafetyState::EStop);
        assert_eq!(inner.estop_state.snapshot(), (1, true));
        assert_eq!(inner.applied_estop_count, 1);
        assert!(inner.safety.epoch_id() > fault_ep);
    }
}
