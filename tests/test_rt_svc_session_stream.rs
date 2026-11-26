//! PR0B fault & zero-allocation proof suite:
//! - Off-RT SPSC stream (`rt_core/stream.rs`): drop-newest, loss counters, sequence gaps,
//!   closed-consumer handling, concurrent SPSC stress, zero allocs & zero frees.
//! - Session lifecycle (`rt_core/session.rs` + `rt_core/reply_router.rs`):
//!   1. Cancellation in every phase (Running, AwaitWorker, and C-F1 commissioning phases).
//!   2. Duplicate terminal suppression (RT `SessionDriver` + off-RT P-F1 `ActionReplyRouter`).
//!   3. Stale worker replies (wrong session/seq/gen, post-cancel, post-expiry, revoked, duplicate).
//!   4. Queue saturation (stream ring, RT terminal SPSC handoff retention, P-F1 router per-conn &
//!      global caps, rollback recovery, independent ExclusiveOutput vs Recorder slots).
//!   5. Deadline expiry & effective TX cutoff clamping (session/child deadlines, stopped-hold exemption).
//!   6. SAFE-HOLD same-release fence (energized uncertain session skips Steps 3–5, 0 disables,
//!      `SafeHoldReason::SessionUncertainState`, 0 RT allocs & 0 `SupervisorLock` attempts).

mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::OperatorGrant;
use armd::diagnostics::alloc::{snapshot_counts, time_this_thread};
use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::mit::{
    CommissioningAuthority, CommissioningOpKind, CommissioningOpState, CommissioningStartRequest,
};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    ActionId, AdmitError, Authority, BootId, ClientId, DeliveryDecision, ImmediateOp, Outcome,
    Reply, Submit,
};
use armd::rt_core::executor::{ExecutorConfig, PanicPolicy};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetFrame};
use armd::rt_core::reply_router::{
    ExternalDestinationPolicy, ExternalSubmitError, SessionTerminalEmit, SESSION_REQUEST_ID_BIT,
};
use armd::rt_core::requests::config::{ConfigReply, ConfigRequest};
use armd::rt_core::safe_hold::SafeHoldReason;
use armd::rt_core::session::{
    admit_internal, admit_routed, PhaseStepOutcome, PreparedSession, ReleaseCtx, SessionAdmitError,
    SessionId, SessionKind, SessionSlot, SessionState, SessionStep, SessionTerminalReason,
    SessionTerminalRecord, WorkerReply, WorkerReplyError, WorkerTicket,
};
use armd::rt_core::stream::{self, StreamPushError, StreamSample};
use armd::supervisor::{AdmissionStamp, ArmSupervisor, SupervisorInner};
use mit_support::FakeBus;

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn new_core_with_lease(conn_id: u64) -> (SupervisorInner, u64, u64) {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 30_000;
    let mut inner = SupervisorInner::new(cfg);
    inner.lease_enabled = true;
    let _ = inner.register_live_connection(conn_id);
    let client_name = format!("ws-{conn_id}");
    let gen = inner.acquire_lease(&client_name).unwrap();
    let epoch = inner.safety.epoch_id();
    (inner, gen, epoch)
}

fn make_admit_stamp(conn_id: u64, lease_gen: u64, epoch: u64, stop_gen: u64, req_id: u64) -> AdmissionStamp {
    AdmissionStamp {
        connection_id: conn_id,
        request_id: req_id,
        lease_gen,
        estop_epoch: epoch,
        stop_gen,
    }
}

fn make_release_ctx(now: Instant, epoch: u64, stop_gen: u64) -> ReleaseCtx {
    ReleaseCtx {
        arm: ArmId::LOCAL,
        boot: BootId::process(),
        owner_generation: 1,
        now,
        release_deadline: now + Duration::from_millis(20),
        base_tx_cutoff: now + Duration::from_millis(16),
        epoch,
        stop_generation: stop_gen,
        estop_latched: false,
        urgent_stop_pending: false,
        internal_fault_fenced: false,
        safe_hold_active: false,
    }
}

// ---------------------------------------------------------------------------
// 1. Off-RT SPSC Stream: saturation, loss counters, sequence gaps, zero-alloc
// ---------------------------------------------------------------------------

#[test]
fn stream_queue_saturation_gaps_closed_concurrent_and_zero_alloc() {
    time_this_thread();
    let (mut tx, mut rx) = stream::channel::<u32>(3);
    assert_eq!(tx.capacity(), 3);

    let before = snapshot_counts();
    assert_eq!(tx.try_push(10), Ok(1));
    assert_eq!(tx.try_push(20), Ok(2));
    assert_eq!(tx.try_push(30), Ok(3));
    assert_eq!(tx.try_push(40), Err(StreamPushError::Full));
    assert_eq!(tx.try_push(50), Err(StreamPushError::Full));
    assert_eq!(rx.try_pop(), Some(StreamSample { seq: 1, gap_before: 0, payload: 10 }));
    assert_eq!(tx.try_push(60), Ok(6));
    assert_eq!(rx.try_pop(), Some(StreamSample { seq: 2, gap_before: 0, payload: 20 }));
    assert_eq!(rx.try_pop(), Some(StreamSample { seq: 3, gap_before: 0, payload: 30 }));
    assert_eq!(rx.try_pop(), Some(StreamSample { seq: 6, gap_before: 2, payload: 60 }));
    assert_eq!(rx.try_pop(), None);
    let delta = snapshot_counts().delta_since(before);
    assert_eq!(delta.total(), 0, "stream push/pop must perform 0 allocs and 0 deallocs: {delta:?}");

    let st = rx.stats();
    assert_eq!((st.pushed, st.dropped, st.dropped_full, st.dropped_closed), (4, 2, 2, 0));
    assert_eq!((st.popped, st.sequence_gaps, st.lost_in_gaps), (4, 1, 2));

    drop(rx);
    assert!(tx.is_closed());
    assert_eq!(tx.try_push(70), Err(StreamPushError::Closed));
    assert_eq!(tx.stats().dropped_closed, 1);

    let (mut p, mut c) = stream::channel::<u64>(16);
    let done = Arc::new(AtomicBool::new(false));
    let done_c = Arc::clone(&done);
    let consumer = std::thread::spawn(move || {
        let mut last_seq = 0u64;
        let mut gap_sum = 0u64;
        let mut count = 0u64;
        while !done_c.load(Ordering::Acquire) || !c.is_producer_closed() {
            while let Some(s) = c.try_pop() {
                assert!(s.seq > last_seq, "sequences must be strictly monotonic");
                if last_seq > 0 {
                    assert_eq!(s.gap_before, s.seq - last_seq - 1);
                }
                last_seq = s.seq;
                gap_sum += s.gap_before;
                count += 1;
            }
            std::thread::yield_now();
        }
        while let Some(s) = c.try_pop() {
            assert!(s.seq > last_seq);
            gap_sum += s.gap_before;
            last_seq = s.seq;
            count += 1;
        }
        (count, gap_sum, c.stats())
    });
    for i in 0..10_000u64 {
        let _ = p.try_push(i);
    }
    drop(p);
    done.store(true, Ordering::Release);
    let (popped, gap_sum, stats) = consumer.join().unwrap();
    assert_eq!(popped, stats.popped);
    assert_eq!(stats.pushed + stats.dropped, 10_000);
    assert_eq!(popped, stats.pushed);
    assert_eq!(gap_sum, stats.lost_in_gaps);
    assert!(stats.producer_closed);
}

// ---------------------------------------------------------------------------
// 2. Cancellation in every phase (session phases + live C-F1 commissioning)
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn cancellation_in_every_session_and_cf1_phase() {
    let (mut inner, lease_gen, epoch) = new_core_with_lease(11);
    let now = Instant::now();
    let stop_gen = inner.current_stop_gen();

    // (a) Running phase cancellation + duplicate suppression.
    let admitted = inner.check_admission(make_admit_stamp(11, lease_gen, epoch, stop_gen, 1)).unwrap();
    let sid = admit_internal(admitted, PreparedSession::exclusive(SessionKind::Custom(1), ClientId(11)), now).unwrap();
    assert!(inner.sessions().blocks_ordinary_output(None));
    let out = inner
        .sessions_mut()
        .cancel_slot(SessionSlot::ExclusiveOutput, sid, SessionTerminalReason::Cancelled("STOPPED"), false)
        .unwrap();
    assert_eq!(out.reason, SessionTerminalReason::Cancelled("STOPPED"));
    assert!(inner
        .sessions_mut()
        .cancel_slot(SessionSlot::ExclusiveOutput, sid, SessionTerminalReason::Cancelled("LEASE_LOST"), false)
        .is_none());
    assert_eq!(inner.sessions().stats().duplicate_terminals_suppressed, 1);
    assert!(inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, sid));

    // (b) AwaitWorker phase cancellation invalidates ticket cleanly.
    let admitted2 = inner.check_admission(make_admit_stamp(11, lease_gen, epoch, stop_gen, 2)).unwrap();
    let sid2 = admit_internal(admitted2, PreparedSession::exclusive(SessionKind::Custom(2), ClientId(11)), now).unwrap();
    let rctx = make_release_ctx(now, epoch, stop_gen);
    let step = inner
        .sessions_mut()
        .step_slot(
            SessionSlot::ExclusiveOutput,
            &rctx,
            |_| false,
            |_, _, _| true,
            |_, _| true,
            |_, _| false,
            |_, _, _| PhaseStepOutcome::AwaitWorker,
        )
        .unwrap();
    let SessionStep::AwaitWorker(ticket) = step else { panic!("expected AwaitWorker") };
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, sid2, now, now + Duration::from_millis(20)),
        Err("MOTION_BUSY")
    );
    assert!(inner
        .sessions_mut()
        .cancel_slot(SessionSlot::ExclusiveOutput, sid2, SessionTerminalReason::Cancelled("STOPPED"), false)
        .is_some());
    assert_eq!(
        inner.sessions_mut().deliver_worker_reply(
            WorkerReply { slot: SessionSlot::ExclusiveOutput, ticket, ok: true, code: "", value: 1 },
            now,
            Ok(()),
        ),
        Err(WorkerReplyError::Stale)
    );
    assert!(inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, sid2));

    // (c) Live RT executor C-F1 commissioning reference adapter cancellation across ReadIdentity,
    //     WriteTimeout (GatherRestEvidence/ReadBefore/Write), and MotorZeroDisable.
    let cfg = ArmConfig::rebot_dm();
    let rest_pos = cfg.resting_positions();
    let limits = b601_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = rest_pos.get(i).copied().unwrap_or(0.0);
            m.v = 0.0;
            m.enabled = false;
        }
        s.push_all_feedback();
    });
    let mut raw_drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    let _ = raw_drv.read_joint_states();
    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(raw_drv)));
    let (conn_label, gen, ep) = {
        let mut g = sup.inner().write().await;
        g.lease_enabled = true;
        let _ = g.register_live_connection(11);
        let gen = g.acquire_lease("ws-11").unwrap();
        (armd::model::HolderLabel::new("ws-11"), gen, g.safety.epoch_id())
    };
    let ecfg = ExecutorConfig::for_tests(Duration::from_millis(8));
    let handle = sup.start_executor(ecfg).await.unwrap();
    let dl = Instant::now() + Duration::from_secs(2);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // ReadIdentity dry-run (grant_id = None, txn_id = 0): start -> hold replies so in-flight -> cancel -> clear.
    bus.with(|s| s.motors[0].silent = true);
    let read_req = CommissioningStartRequest {
        kind: CommissioningOpKind::ReadIdentity,
        joints: [0, 1, 2, 3, 4, 5, 0, 0],
        joint_len: 1,
        timeout_counts: 0,
        references: [0.0; 8],
        authority: CommissioningAuthority {
            conn_id: 11,
            lease_holder: Some((conn_label, gen)),
            operator_grant: OperatorGrant::unenforced(),
            expected_epoch: ep,
            expected_stop_gen: 0,
            grant_id: None,
            txn_id: 0,
        },
        txn_deadline: Instant::now() + Duration::from_secs(5),
        prior_disk_mutated: false,
    };
    let op_id = match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartCommissioning(read_req))
        .await
        .unwrap()
    {
        ConfigReply::CommissioningStarted(Ok(id)) => id,
        _ => panic!("StartCommissioning(ReadIdentity) failed"),
    };
    // Concurrent start must be refused as Busy because ExclusiveOutput slot is occupied.
    let busy = sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartCommissioning(read_req))
        .await
        .unwrap();
    assert!(matches!(busy, ConfigReply::CommissioningStarted(Err(_))), "expected Busy");
    tokio::time::sleep(Duration::from_millis(25)).await;
    let cancelled = sup
        .dispatch_config_request_owned_reliable(ConfigRequest::CancelCommissioning { op_id, reason: "STOPPED" })
        .await
        .unwrap();
    match cancelled {
        ConfigReply::CommissioningCancelled(Some(snap)) => {
            assert_eq!(snap.state, CommissioningOpState::Cancelled);
        }
        _ => panic!("unexpected cancel reply"),
    }
    let cleared = sup
        .dispatch_config_request_owned_reliable(ConfigRequest::ClearCommissioning {
            op_id,
            clean_complete: false,
            force_reconcile: false,
        })
        .await
        .unwrap();
    assert!(matches!(cleared, ConfigReply::CommissioningCleared(true)));

    // After clear, ExclusiveOutput slot is free again and a second ReadIdentity admits cleanly.
    bus.with(|s| s.motors[0].silent = false);
    let op_id2 = match sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartCommissioning(read_req))
        .await
        .unwrap()
    {
        ConfigReply::CommissioningStarted(Ok(id)) => id,
        _ => panic!("second StartCommissioning failed after clear"),
    };
    let _ = sup
        .dispatch_config_request_owned_reliable(ConfigRequest::CancelCommissioning { op_id: op_id2, reason: "LEASE_LOST" })
        .await
        .unwrap();
    let _ = sup
        .dispatch_config_request_owned_reliable(ConfigRequest::ClearCommissioning {
            op_id: op_id2,
            clean_complete: false,
            force_reconcile: false,
        })
        .await
        .unwrap();

    handle.stop_and_join();
}

// ---------------------------------------------------------------------------
// 3. Stale worker replies, deadline expiry, TX cutoff & slot independence
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stale_worker_replies_deadline_expiry_and_tx_cutoff_exemption() {
    time_this_thread();
    let (mut inner, lease_gen, epoch) = new_core_with_lease(22);
    let now = Instant::now();
    let stop_gen = inner.current_stop_gen();

    // Admission-time already-expired deadline fails closed.
    let mut expired_prep = PreparedSession::exclusive(SessionKind::Custom(10), ClientId(22));
    expired_prep.deadline = Some(now);
    let adm = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 1)).unwrap();
    assert_eq!(admit_internal(adm, expired_prep, now), Err(SessionAdmitError::DeadlineExpired));

    // Admit ExclusiveOutput with parent deadline 80ms + child deadline 30ms.
    let mut prep = PreparedSession::exclusive(SessionKind::Custom(11), ClientId(22));
    prep.deadline = Some(now + Duration::from_millis(80));
    prep.child_deadline = Some(now + Duration::from_millis(30));
    prep.timeout_code = "SESSION_TIMEOUT";
    prep.child_timeout_code = "CHILD_TIMEOUT";
    let adm2 = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 2)).unwrap();
    let sid = admit_internal(adm2, prep, now).unwrap();

    // ExclusiveOutput deadline feeds active_action_deadline (child 30ms < parent 80ms).
    assert_eq!(inner.active_action_deadline(), Some(now + Duration::from_millis(30)));
    // Independent Recorder admits concurrently while ExclusiveOutput is busy.
    let adm_rec = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 3)).unwrap();
    let rec_id = admit_internal(adm_rec, PreparedSession::recorder(ClientId(22)), now).unwrap();
    assert_ne!(sid, rec_id);

    // Pre-TX gate passes while Running within cutoff, rejects Recorder, expired cutoff, AwaitWorker, and cancelled!
    let base_cutoff = now + Duration::from_millis(50);
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, sid, now + Duration::from_millis(10), base_cutoff),
        Ok(now + Duration::from_millis(30))
    );
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::Recorder, rec_id, now + Duration::from_millis(10), base_cutoff),
        Err("INVALID_COMMAND")
    );
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, sid, now + Duration::from_millis(10), now + Duration::from_millis(5)),
        Err("DEADLINE_EXPIRED")
    );
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, sid, now + Duration::from_millis(35), base_cutoff),
        Err("CHILD_TIMEOUT")
    );

    // Transition to AwaitWorker and verify every stale-worker-reply failure mode (zero mutation).
    let rctx = make_release_ctx(now + Duration::from_millis(5), epoch, stop_gen);
    let step = inner
        .sessions_mut()
        .step_slot(
            SessionSlot::ExclusiveOutput,
            &rctx,
            |_| false,
            |_, _, _| true,
            |_, _| true,
            |_, _| false,
            |_, _, _| PhaseStepOutcome::AwaitWorker,
        )
        .unwrap();
    let SessionStep::AwaitWorker(ticket) = step else { panic!("expected AwaitWorker") };
    // Pre-TX gate rejects AwaitingWorker state.
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, sid, now + Duration::from_millis(10), base_cutoff),
        Err("MOTION_BUSY")
    );

    let mk = |t: WorkerTicket| WorkerReply { slot: SessionSlot::ExclusiveOutput, ticket: t, ok: true, code: "", value: 7 };
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(WorkerTicket { session: SessionId(999), ..ticket }), now, Ok(())), Err(WorkerReplyError::Stale));
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(WorkerTicket { step_seq: ticket.step_seq + 1, ..ticket }), now, Ok(())), Err(WorkerReplyError::Stale));
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(WorkerTicket { generation: ticket.generation + 1, ..ticket }), now, Ok(())), Err(WorkerReplyError::Stale));
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(ticket), now, Err("LEASE_LOST")), Err(WorkerReplyError::Stale));
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(ticket), now + Duration::from_millis(35), Ok(())), Err(WorkerReplyError::Stale));
    assert_eq!(inner.sessions().slot(SessionSlot::ExclusiveOutput).unwrap().state, SessionState::AwaitingWorker(ticket));

    // One-shot acceptance + duplicate rejection + single consumption in step_slot.
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(ticket), now + Duration::from_millis(10), Ok(())), Ok(mk(ticket)));
    assert_eq!(inner.sessions_mut().deliver_worker_reply(mk(ticket), now + Duration::from_millis(11), Ok(())), Err(WorkerReplyError::Stale));
    assert_eq!(inner.sessions().stats().stale_worker_replies_rejected, 6);
    let mut seen_replies = 0usize;
    let _ = inner.sessions_mut().step_slot(
        SessionSlot::ExclusiveOutput,
        &make_release_ctx(now + Duration::from_millis(12), epoch, stop_gen),
        |_| false,
        |_, _, _| true,
        |_, _| true,
        |_, _| false,
        |_, _, r| { if r.is_some() { seen_replies += 1; } PhaseStepOutcome::Pending },
    );
    let _ = inner.sessions_mut().step_slot(
        SessionSlot::ExclusiveOutput,
        &make_release_ctx(now + Duration::from_millis(13), epoch, stop_gen),
        |_| false,
        |_, _, _| true,
        |_, _| true,
        |_, _| false,
        |_, _, r| { if r.is_some() { seen_replies += 1; } PhaseStepOutcome::Pending },
    );
    assert_eq!(seen_replies, 1, "accepted worker reply must be consumed once");

    // Zero-alloc & zero-free proof across RT terminal retention & slot reuse gating.
    let (tx, mut rx) = stream::channel::<SessionTerminalRecord>(1);
    inner.sessions_mut().attach_terminal_producer(tx);
    let exp_ctx = make_release_ctx(now + Duration::from_millis(35), epoch, stop_gen);
    let before = snapshot_counts();
    let exp_step = inner
        .sessions_mut()
        .step_slot(
            SessionSlot::ExclusiveOutput,
            &exp_ctx,
            |_| false,
            |_, _, _| true,
            |_, _| true,
            |_, _| false,
            |_, _, _| PhaseStepOutcome::Pending,
        )
        .unwrap();
    let delta = snapshot_counts().delta_since(before);
    assert_eq!(delta.total(), 0, "RT session step/expiry must perform 0 allocs and 0 deallocs: {delta:?}");
    assert!(matches!(
        exp_step,
        SessionStep::Terminal(o) if o.reason == SessionTerminalReason::DeadlineExpired("CHILD_TIMEOUT")
    ));
    assert!(inner.sessions().slot(SessionSlot::Recorder).unwrap().is_live());
    assert!(inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, sid));

    // Full RT terminal SPSC ring (cap=1) retains pending terminal and blocks slot reuse until drained.
    let router_sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    router_sup.set_lease_enabled(false).await;
    let router_exec = router_sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(8)))
        .await
        .unwrap();
    let _ = router_exec.open_connection_sync(22).unwrap();
    let noop_sink = Arc::new(|_c: ClientId, _r: Reply| DeliveryDecision::Delivered);
    let _tok_22 = router_exec
        .register_connection_reply_sink(ClientId(22), noop_sink, Some(4))
        .unwrap();

    let mut g1 = router_exec
        .reserve_session_terminal(ClientId(22), ActionId(1), ExternalDestinationPolicy::ConnectionOnly)
        .unwrap();
    let r1 = g1.stage_for_admission().unwrap();
    let r1_req = r1.request();
    let p1 = PreparedSession::exclusive(SessionKind::Custom(20), ClientId(22));
    let a1 = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 4)).unwrap();
    let id1 = admit_routed(a1, p1, r1, now).unwrap();
    g1.commit_transferred();
    assert!(inner.sessions_mut().finish_slot(SessionSlot::ExclusiveOutput, id1, SessionTerminalReason::Completed, false).is_some());
    assert!(!inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, id1), "must block reuse before flush");
    assert_eq!(inner.sessions_mut().flush_published_terminals(), 1);
    assert!(inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, id1));

    let mut g2 = router_exec
        .reserve_session_terminal(ClientId(22), ActionId(2), ExternalDestinationPolicy::ConnectionOnly)
        .unwrap();
    let r2 = g2.stage_for_admission().unwrap();
    let r2_req = r2.request();
    let p2 = PreparedSession::exclusive(SessionKind::Custom(21), ClientId(22));
    let a2 = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 5)).unwrap();
    let id2 = admit_routed(a2, p2, r2, now).unwrap();
    g2.commit_transferred();
    assert!(inner.sessions_mut().finish_slot(SessionSlot::ExclusiveOutput, id2, SessionTerminalReason::Completed, false).is_some());
    assert_eq!(inner.sessions_mut().flush_published_terminals(), 0, "full ring retains terminal inline");
    assert!(inner.sessions().has_pending_terminal(SessionSlot::ExclusiveOutput));
    assert!(!inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, id2));
    assert_eq!(rx.try_pop().unwrap().payload.request, r1_req);
    assert_eq!(inner.sessions_mut().flush_published_terminals(), 1);
    assert!(inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, id2));
    assert_eq!(rx.try_pop().unwrap().payload.request, r2_req);
    router_exec.stop_and_join();

    // Verify pre-TX gate rejects explicitly cancelled session even while lease/epoch/stop are live.
    let mut p_cancel = PreparedSession::exclusive(SessionKind::Custom(23), ClientId(22));
    p_cancel.deadline = Some(now + Duration::from_millis(200));
    let a_cancel = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 7)).unwrap();
    let id_cancel = admit_internal(a_cancel, p_cancel, now).unwrap();
    assert!(inner.sessions_mut().cancel_slot(SessionSlot::ExclusiveOutput, id_cancel, SessionTerminalReason::Cancelled("STOPPED"), false).is_some());
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, id_cancel, now, now + Duration::from_millis(20)),
        Err("CANCELLED")
    );
    assert!(inner.sessions_mut().clear_terminal_slot(SessionSlot::ExclusiveOutput, id_cancel));

    // Verify E-stop immediately revokes pre-TX gate and marks safety stopped (which keeps
    // safety/stopped holds exempt from session deadlines in `install_effective_tx_cutoff`).
    let mut p_stop = PreparedSession::exclusive(SessionKind::Custom(22), ClientId(22));
    p_stop.deadline = Some(now + Duration::from_millis(200));
    let a_stop = inner.check_admission(make_admit_stamp(22, lease_gen, epoch, stop_gen, 6)).unwrap();
    let id_stop = admit_internal(a_stop, p_stop, now).unwrap();
    assert!(inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, id_stop, now, now + Duration::from_millis(20)).is_ok());
    let _ = inner.safety.trigger_e_stop();
    assert!(inner.safety.state().is_stopped());
    assert_eq!(
        inner.check_session_pre_tx(SessionSlot::ExclusiveOutput, id_stop, now, now + Duration::from_millis(20)),
        Err("EPOCH_MISMATCH")
    );
}

// ---------------------------------------------------------------------------
// 4. P-F1 router terminal credit saturation, rollback & duplicate suppression
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn pf1_router_terminal_credit_saturation_rollback_and_duplicate_suppression() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;
    let exec = sup.start_executor(ExecutorConfig::for_tests(Duration::from_millis(8))).await.unwrap();

    let delivered: Arc<Mutex<Vec<(ClientId, Reply)>>> = Arc::new(Mutex::new(Vec::new()));
    let d_cb = Arc::clone(&delivered);
    let sink = Arc::new(move |client: ClientId, reply: Reply| {
        d_cb.lock().unwrap().push((client, reply));
        DeliveryDecision::Delivered
    });

    let c1 = ClientId(101);
    let _ = exec.open_connection_sync(101).unwrap();
    let _tok = exec.register_connection_reply_sink(c1, sink, Some(2)).unwrap();

    // Fill per-connection cap (2): 3rd reservation must fail closed with RepliesFull.
    let t1 = exec
        .reserve_session_terminal_direct(c1, ActionId(1), ExternalDestinationPolicy::ConnectionOnly)
        .unwrap();
    assert_ne!(t1.request().0 & SESSION_REQUEST_ID_BIT, 0);
    let t2 = exec
        .reserve_session_terminal_direct(c1, ActionId(2), ExternalDestinationPolicy::ConnectionOnly)
        .unwrap();
    assert_eq!(exec.router_global_reserved(), 2);
    assert_eq!(
        exec.reserve_session_terminal_direct(c1, ActionId(3), ExternalDestinationPolicy::ConnectionOnly),
        Err(ExternalSubmitError::Admit(AdmitError::RepliesFull))
    );

    // Rollback releases credit cleanly so a new reservation succeeds.
    assert!(exec.release_session_terminal(&t2));
    assert!(!exec.release_session_terminal(&t2), "rollback must be idempotent");
    assert_eq!(exec.router_global_reserved(), 1);
    let t3 = exec
        .reserve_session_terminal_direct(c1, ActionId(3), ExternalDestinationPolicy::ConnectionOnly)
        .unwrap();

    // Emit t1 once -> Enqueued & delivered; duplicate emit -> DuplicateSuppressed.
    assert_eq!(exec.emit_session_terminal(&t1, Outcome::Completed, 0), SessionTerminalEmit::Enqueued);
    assert_eq!(
        exec.emit_session_terminal(&t1, Outcome::Failed("DUP"), 0),
        SessionTerminalEmit::DuplicateSuppressed
    );
    assert_eq!(exec.emit_session_terminal(&t3, Outcome::Stopped, 0), SessionTerminalEmit::Enqueued);
    exec.pump_broker_replies();
    assert_eq!(exec.router_global_reserved(), 0, "all P-F1 credits recovered after delivery");
    {
        let items = delivered.lock().unwrap();
        assert_eq!(items.len(), 2, "exactly one terminal per admitted session");
        assert!(matches!(items[0].1, Reply::Terminal { action: ActionId(1), outcome: Outcome::Completed, .. }));
        assert!(matches!(items[1].1, Reply::Terminal { action: ActionId(3), outcome: Outcome::Stopped, .. }));
    }

    exec.stop_and_join();
}

// ---------------------------------------------------------------------------
// 5. Live RtExecutor SAFE-HOLD same-release fence (uncertain energized session)
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn energized_uncertain_session_triggers_same_release_safe_hold_with_zero_alloc_and_zero_disable() {
    let raised = [0.40, -0.30, 0.50, 0.20, -0.20, 0.10, -1.20];
    let limits = b601_limits();
    let bus = FakeBus::new(&limits);
    bus.with(|s| {
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = raised[i];
            m.v = 0.0;
            m.enabled = false;
        }
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap();
    drv.read_joint_states().unwrap();
    bus.with(|s| s.push_all_feedback());
    drv.read_joint_states().unwrap();

    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
    assert!(bus.with(|s| s.motors.iter().all(|m| m.enabled)));

    // Admit an energized ExclusiveOutput session with hw_state_uncertain = true + short deadline,
    // plus an independent Recorder session.
    let (epoch, stop_gen) = {
        let mut inner = sup.inner().write().await;
        let now = Instant::now();
        let ep = inner.safety.epoch_id();
        let sg = inner.current_stop_gen();
        let mut prep = PreparedSession::exclusive(SessionKind::Custom(42), ClientId(33));
        prep.deadline = Some(now + Duration::from_millis(35));
        prep.escalate_safe_hold_on_uncertain = true;
        let adm = inner.check_admission(make_admit_stamp(0, 0, ep, sg, 1001)).unwrap();
        let sid = admit_internal(adm, prep, now).unwrap();
        assert!(inner.sessions_mut().set_hw_state_uncertain(SessionSlot::ExclusiveOutput, sid, true));

        let adm_rec = inner.check_admission(make_admit_stamp(0, 0, ep, sg, 1002)).unwrap();
        let _rec_id = admit_internal(adm_rec, PreparedSession::recorder(ClientId(33)), now).unwrap();
        (ep, sg)
    };
    bus.clear_log();

    let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(8));
    ecfg.alloc_timing = true;
    ecfg.panic_policy = PanicPolicy::StopExecutor;
    let exec = sup.start_executor(ecfg).await.unwrap();

    // Queue an ordinary command AND stage a servo target while the uncertain session is live;
    // neither may reach bus TX across the same-release SAFE-HOLD fence!
    let now_submit = Instant::now();
    let _ = exec.broker().lock().unwrap().submit(
        ClientId(33),
        1,
        Authority {
            lease_generation: 0,
            epoch,
            stop_generation: stop_gen,
            config_revision: 0,
            parent_generation: 0,
        },
        now_submit + Duration::from_secs(2),
        Submit::Immediate(ImmediateOp::TorqueOff),
    );
    let _ = exec.target_writer().lock().unwrap().submit(TargetFrame {
        arm: ArmId::LOCAL,
        boot: exec.boot(),
        stream: StreamId(1),
        seq: 1,
        authority: Authority {
            lease_generation: 0,
            epoch,
            stop_generation: stop_gen,
            config_revision: 0,
            parent_generation: 0,
        },
        policy_generation: 0,
        input_at: now_submit,
        vision_at: None,
        vision_frame_id: None,
        deadline: now_submit + Duration::from_secs(2),
        dof: 7,
        q: [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, -1.2, 0.0],
        replaceable: true,
    });

    let dl = Instant::now() + Duration::from_secs(2);
    while (!exec.is_in_safe_hold() || exec.safe_hold_control().ticks() < 2) && Instant::now() < dl {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let sh = exec.safe_hold_control();
    assert!(exec.is_in_safe_hold(), "energized uncertain session must enter SAFE-HOLD");
    assert_eq!(sh.reason(), SafeHoldReason::SessionUncertainState);
    assert!(sh.hold_tx_count() >= 1, "SAFE-HOLD must actively emit hold frames");
    assert!(sup.internal_fault_fenced(), "same-release SAFE-HOLD must latch internal fault fence");

    // Verify 0 CMD_DISABLE frames and all motors still enabled (torque-off rule!), and that
    // the staged 0.9 rad servo target never reached J1 TX.
    let (all_enabled, disable_count, saw_staged_target) = bus.with(|s| {
        let disables = s.tx_log.iter().filter(|f| f.data == damiao::CMD_DISABLE).count();
        let staged = s.tx_log.iter().any(|f| {
            f.id == 0x101 && {
                let q = f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]);
                (q - 0.9).abs() < 0.05
            }
        });
        (s.motors.iter().all(|m| m.enabled), disables, staged)
    });
    assert!(all_enabled, "all motors must remain enabled in SAFE-HOLD away from rest");
    assert_eq!(disable_count, 0, "SAFE-HOLD transition and queued TorqueOff must emit 0 CMD_DISABLE frames");
    assert!(!saw_staged_target, "staged target must never reach TX across same-release SAFE-HOLD fence");
    // Verify zero RT allocations recorded on armd-control releases before SAFE-HOLD.
    assert_eq!(sup.inner().control_thread_attempts(), 0, "0 SupervisorLock attempts on armd-control");

    let _ = exec.emergency_disable_and_join();
}
