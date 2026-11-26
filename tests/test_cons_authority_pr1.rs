//! PR 1 executor consolidation negative & dual-owner parity tests:
//!
//! 1. Primitive 1 (`AuthorityProof`):
//!    - Stale proof negative tests: pre-existing fault / already-stopped state is NOT proof;
//!      bare `safety.latch(...)` without an owner stop effect is NOT proof; pending E-stop /
//!      pending stop request fences output without counting as proof; lease loss revokes
//!      authority without counting as proof; ownership generation change invalidates prior
//!      baselines.
//!    - Positive proof tests: applied general stop, applied E-stop, and applied qualifying fault
//!      stop (`ThermalTrip`, `BusLossFallback`) after admission return event-specific
//!      `AuthorityProof` tied to the same owner generation.
//!    - Explicit recovery helper (`new_estop_since`): tolerates baseline pre-existing applied
//!      latch while aborting on a new pending or applied E-stop.
//!    - Dual-owner parity (`OWNER_LEGACY` vs `OWNER_EXECUTOR`) for applied-stop vs lease-loss
//!      and pre-existing stopped state.
//!
//! 2. Primitive 2 (`OperatorGrant` / `OperatorRequirement`):
//!    - Revocation between queueing and admission on direct `gravity_comp_command` and
//!      `ModeSwitch` sessions on BOTH owners.
//!    - Fail-closed `OperatorRequirement::NotRequired` refusal on protected `gravity_comp` and
//!      `ModeSwitch` when `operator_auth_enforced == true` on BOTH owners.
//!    - Revocation between staging and activation (`Reserve -> logout -> VerifyAuthority/Activate`
//!      for both volatile pre-persist and durable post-persist paths) on BOTH owners.
//!    - Live mid-session operator grant revocation during `ModeSwitch` on BOTH owners.
//!
//! 3. Primitive 3 (Session admission & terminal credit accounting):
//!    - Missing destination fails closed with `NO_DESTINATION` (both via `start_mode_switch_session`
//!      and direct `ConfigRequest::StartModeSwitch(..., None)` on `OWNER_EXECUTOR`); explicit
//!      internal path (`ClientId(0)` / `admit_internal`) succeeds on BOTH owners.
//!    - Full credit saturation (`RepliesFull` / `REPLIES_FULL`) with exact global & per-connection
//!      credit counts.
//!    - Rejection & RAII rollback on lease/epoch/operator/deadline/busy failures and dropped guard,
//!      plus single-use `stage_for_admission()` enforcement.
//!    - Disconnect credit recovery & closed-client admission refusal (`ConnectionClosed`).
//!    - Session slot reuse AND router connection-slot reuse (stale ticket from closed client A
//!      cannot corrupt client B's credit on the reused slot) with exact credit counts.

mod mit_support;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::auth::{OperatorGrant, OperatorRequirement};
use armd::diagnostics::alloc::{snapshot_counts, time_this_thread, AllocCounts};
use armd::driver::{gripper_drive_from_profile, ArmDriver, DamiaoArmDriver, TxBudget};
use armd::model::{ArmConfig, HolderLabel};
use armd::motor_mode::{
    ControlMode, ModeProgress, ModeSwitchFailure, ModeSwitchPlan, ModeSwitchRecovery,
};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::authority::StopProofEvent;
use armd::rt_core::broker::{ActionId, ClientId, DeliveryDecision, Outcome, Reply};
use armd::rt_core::config_iface::{ConfigDomain, Durability};
use armd::rt_core::executor::{ExecutorConfig, ExecutorStats, PanicPolicy, RtExecutorHandle};
use armd::rt_core::reply_router::{ExternalDestinationPolicy, SessionTerminalEmit};
use armd::rt_core::requests::lease::ConnectionHandle;
use armd::rt_core::requests::config::{
    build_config_proposal, capture_config_admission, gravity_comp_command, ActivateConfig,
    ConfigAuthContext, ConfigCandidate, ConfigError, ConfigReply, ConfigRequest,
    ModeSwitchStartRequest, PersistedConfig, ReserveConfig, ReservePrecheck, VerifyDomainChecks,
};
use armd::rt_core::session::{admit_internal, admit_routed, PreparedSession, SessionKind, SessionSlot};
use armd::rt_core::urgent::{StopRequests, UrgentStop, UrgentStopOutcome};
use armd::safety::{FaultDetail, FaultKind, StopPolicy};
use armd::supervisor::{AdmissionStamp, ArmError, ArmSupervisor, SupervisorInner};
use armd::OperatorAuth;
use mit_support::FakeBus;

const PERIOD_50HZ: Duration = Duration::from_millis(20);
const TIMEOUT_2000_COUNTS: u32 = 2000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum OwnerKind {
    Legacy,
    Executor,
}

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

fn make_damiao_bus_and_driver(
    cfg: &ArmConfig,
    arm_timeout_counts: u32,
) -> (FakeBus, DamiaoArmDriver<FakeBus>) {
    let limits = b601_limits();
    let bus = FakeBus::new(&limits);
    let raised = [0.30, -0.25, 0.40, 0.15, -0.15, 0.10, -0.14];
    bus.with(|s| {
        s.tx_step = Duration::ZERO;
        s.poll_feeds_watchdog = false;
        s.immediate_replies = true;
        for (i, m) in s.motors.iter_mut().enumerate() {
            m.q = raised[i];
            m.v = 0.0;
            m.enabled = false;
            m.set_reg_u32(
                damiao::RID_TIMEOUT,
                if i < 6 { arm_timeout_counts } else { 0 },
            );
            m.set_reg_u32(
                damiao::RID_CTRL_MODE,
                if i == 6 {
                    damiao::CTRL_MODE_POS_FORCE
                } else {
                    damiao::CTRL_MODE_POS_VEL
                },
            );
        }
        s.push_all_feedback();
    });
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(limits)
        .unwrap()
        .with_gripper_drive(gripper_drive_from_profile(cfg).unwrap())
        .unwrap();
    let mut b = TxBudget::default();
    b.max_frames = 16;
    drv.set_tx_budget(b);
    drv.set_rt_mode(true);
    let _ = drv.check_mapping_ranges();
    let _ = drv.read_joint_states();
    (bus, drv)
}

struct DualOwnerRig {
    kind: OwnerKind,
    sup: Arc<ArmSupervisor>,
    bus: Option<FakeBus>,
    exec: Option<RtExecutorHandle>,
    legacy_stop: Option<Arc<AtomicBool>>,
    legacy_thread: Option<std::thread::JoinHandle<()>>,
    lease_gen: u64,
}

impl DualOwnerRig {
    async fn start_mock(kind: OwnerKind, lease_enabled: bool, operator_enforced: bool) -> Self {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.control_frequency_hz = 50;
        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.set_lease_enabled(lease_enabled).await;
        sup.set_operator_auth_enforced(operator_enforced).await;
        let lease_gen = if lease_enabled {
            let lg = sup.acquire_lease("cons-pr1-client").await.unwrap();
            sup.enable_torque(sup.capture_admission_stamp())
                .await
                .unwrap();
            lg
        } else {
            sup.enable_torque(sup.capture_admission_stamp())
                .await
                .unwrap();
            0
        };
        Self::finish_start(kind, sup, None, lease_gen).await
    }

    async fn start_damiao(
        kind: OwnerKind,
        lease_enabled: bool,
        operator_enforced: bool,
        arm_enabled: bool,
    ) -> Self {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.control_frequency_hz = 50;
        let (bus, drv) = make_damiao_bus_and_driver(&cfg, TIMEOUT_2000_COUNTS);
        let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
        sup.set_lease_enabled(lease_enabled).await;
        sup.set_operator_auth_enforced(operator_enforced).await;
        let lease_gen = if lease_enabled {
            sup.acquire_lease("cons-pr1-client").await.unwrap()
        } else {
            0
        };
        if arm_enabled {
            sup.enable_torque(sup.capture_admission_stamp())
                .await
                .unwrap();
            bus.with(|s| {
                s.immediate_replies = false;
                s.push_all_feedback();
            });
        } else {
            bus.with(|s| {
                s.immediate_replies = false;
                s.push_all_feedback();
            });
        }
        Self::finish_start(kind, sup, Some(bus), lease_gen).await
    }

    async fn finish_start(
        kind: OwnerKind,
        sup: Arc<ArmSupervisor>,
        bus: Option<FakeBus>,
        lease_gen: u64,
    ) -> Self {
        match kind {
            OwnerKind::Executor => {
                let mut ecfg = ExecutorConfig::for_tests(PERIOD_50HZ);
                ecfg.alloc_timing = true;
                ecfg.panic_policy = PanicPolicy::StopExecutor;
                let exec = sup.start_executor(ecfg).await.unwrap();
                let dl = Instant::now() + Duration::from_secs(2);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Self {
                    kind,
                    sup,
                    bus,
                    exec: Some(exec),
                    legacy_stop: None,
                    legacy_thread: None,
                    lease_gen,
                }
            }
            OwnerKind::Legacy => {
                let stop = Arc::new(AtomicBool::new(false));
                let stop_t = Arc::clone(&stop);
                let sup_t = Arc::clone(&sup);
                let stats = Arc::new(ExecutorStats::default());
                let rt = tokio::runtime::Handle::current();
                let jh = std::thread::Builder::new()
                    .name("legacy-cons-pr1".into())
                    .spawn(move || {
                        sup_t.run_legacy_control_loop_blocking(
                            PERIOD_50HZ,
                            stop_t,
                            stats,
                            rt,
                            None,
                        );
                    })
                    .unwrap();
                let dl = Instant::now() + Duration::from_secs(2);
                while sup.hub().latest().is_none() {
                    assert!(Instant::now() < dl);
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                Self {
                    kind,
                    sup,
                    bus,
                    exec: None,
                    legacy_stop: Some(stop),
                    legacy_thread: Some(jh),
                    lease_gen,
                }
            }
        }
    }

    fn register_routed_client(
        &self,
        id: u64,
        cap: usize,
    ) -> Option<Arc<Mutex<Vec<(ClientId, Reply)>>>> {
        let exec = self.exec.as_ref()?;
        let _ = exec.open_connection_sync(id);
        let delivered = Arc::new(Mutex::new(Vec::new()));
        let d_cb = Arc::clone(&delivered);
        let sink = Arc::new(move |c: ClientId, r: Reply| {
            d_cb.lock().unwrap().push((c, r));
            DeliveryDecision::Delivered
        });
        let _ = exec
            .register_connection_reply_sink(ClientId(id), sink, Some(cap))
            .unwrap();
        Some(delivered)
    }

    async fn shutdown(mut self, force_emergency_disable: bool) {
        match self.kind {
            OwnerKind::Executor => {
                if let Some(exec) = self.exec.take() {
                    if force_emergency_disable {
                        let _ = exec.emergency_disable_and_join();
                    } else {
                        exec.stop_and_join();
                    }
                }
            }
            OwnerKind::Legacy => {
                if force_emergency_disable {
                    let sh = self.sup.legacy_safe_hold_control();
                    sh.request_process_exit_disable(Instant::now() + Duration::from_secs(2));
                }
                if let Some(stop) = self.legacy_stop.take() {
                    stop.store(true, std::sync::atomic::Ordering::Release);
                }
                if let Some(jh) = self.legacy_thread.take() {
                    let _ = jh.join();
                }
            }
        }
    }
}

fn make_stamp(conn_id: u64, lease_gen: u64, epoch: u64, stop_gen: u64, req_id: u64) -> AdmissionStamp {
    AdmissionStamp {
        connection_id: conn_id,
        lease_gen,
        estop_epoch: epoch,
        stop_gen,
        request_id: req_id,
    }
}

// ============================================================================
// 1. Primitive 1 (`AuthorityProof`): stale proof, pending request, lease loss,
//    owner generation, applied stops, and dual-owner parity
// ============================================================================

#[test]
fn authority_proof_rejects_stale_fault_already_stopped_pending_and_lease_loss_zero_alloc() {
    time_this_thread();
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm());
    inner.lease_enabled = true;
    let lease_gen = inner.acquire_lease("op-client").unwrap();
    let now = Instant::now();

    // 1a. Pre-existing fault / already-stopped state is NOT proof after baseline capture.
    inner.execute_stop(StopPolicy::BusLossFallback);
    assert!(inner.safety.state().is_stopped());
    let base_after_prior_fault = inner.capture_baseline();
    assert_eq!(base_after_prior_fault.owner_generation, inner.owner_generation());
    assert_eq!(base_after_prior_fault.lease_generation, lease_gen);
    assert_eq!(
        inner.stop_applied_since(base_after_prior_fault),
        None,
        "pre-existing fault / already-stopped safety state must NOT qualify as stop proof"
    );

    // 1b. Pre-existing E-stop is NOT proof after baseline capture, and `new_estop_since` tolerates
    //     the baseline pre-existing latch while aborting on a NEW pending/applied E-stop.
    assert!(inner.request_and_consume_estop_for_test());
    assert!(inner.safety.has_latched_fault_kind(FaultKind::Estop));
    let base_after_prior_estop = inner.capture_baseline();
    assert_eq!(
        inner.stop_applied_since(base_after_prior_estop),
        None,
        "already-applied pre-existing E-stop must NOT qualify as new stop proof"
    );
    assert!(
        !inner.new_estop_since(base_after_prior_estop),
        "explicit recovery baseline tolerates pre-existing applied E-stop latch"
    );

    // 1c. Bare `safety.latch(...)` without an owner stop effect is NOT proof.
    let base_before_bare_latch = inner.capture_baseline();
    let _ = inner.safety.latch(FaultKind::WatchdogTimeout, FaultDetail::WatchdogTimeout);
    assert_eq!(
        inner.stop_applied_since(base_before_bare_latch),
        None,
        "bare safety.latch without owner stop effect must NOT qualify as stop proof"
    );

    // 1d. Pending E-stop request fences output (`has_pending_stop_fence == true`), triggers
    //     `new_estop_since == true`, and is NOT applied-stop proof until consumed by the owner.
    let base_before_pending_estop = inner.capture_baseline();
    let _ = inner.estop_state.request_and_latch();
    assert!(inner.has_pending_stop_fence(), "pending E-stop must fence output");
    assert!(
        inner.new_estop_since(base_before_pending_estop),
        "new pending E-stop must abort recovery gate"
    );
    assert_eq!(
        inner.stop_applied_since(base_before_pending_estop),
        None,
        "pending E-stop request must NOT count as applied-stop proof"
    );
    // Once the owner applies it, event-specific EStopApplied proof is returned (0 alloc).
    let before = snapshot_counts();
    assert!(inner.request_and_consume_estop_for_test());
    let proof_estop = inner
        .stop_applied_since(base_before_pending_estop)
        .expect("applied E-stop after admission must produce proof");
    let delta = snapshot_counts().delta_since(before);
    assert_eq!(proof_estop.owner_generation(), base_before_pending_estop.owner_generation);
    assert!(matches!(proof_estop.event(), StopProofEvent::EStopApplied { seq } if seq > base_before_pending_estop.applied_estop_seq));
    assert_eq!(delta.total(), 0, "stop_applied_since check must allocate 0 bytes: {delta:?}");

    // 1e. Pending authorized-stop request (even with StopRequests::mark_applied bypassing owner)
    //     fences output while pending and is NOT proof until `authorized_stop()` runs.
    let stops = StopRequests::new();
    inner.attach_stop_requests(stops.clone());
    let base_before_stop = inner.capture_baseline();
    let req_gen = stops.request();
    assert!(inner.has_pending_stop_fence(), "pending StopRequests must fence output");
    assert_eq!(
        inner.stop_applied_since(base_before_stop),
        None,
        "pending general stop request must NOT count as applied-stop proof"
    );
    stops.mark_applied(req_gen);
    assert_eq!(
        inner.stop_applied_since(base_before_stop),
        None,
        "StopRequests::mark_applied without owner authorized_stop must NOT count as proof"
    );
    assert!(inner.authorized_stop());
    let proof_stop = inner
        .stop_applied_since(base_before_stop)
        .expect("owner authorized_stop after admission must produce GeneralStopApplied proof");
    assert_eq!(proof_stop.owner_generation(), base_before_stop.owner_generation);
    assert!(matches!(
        proof_stop.event(),
        StopProofEvent::GeneralStopApplied { seq } if seq == base_before_stop.applied_general_stop_seq + 1
    ));

    // 1f. Qualifying fault stop applied AFTER baseline (`ThermalTrip` and `BusLossFallback`) IS proof.
    let base_before_thermal = inner.capture_baseline();
    inner.execute_stop(StopPolicy::ThermalTrip);
    let proof_thermal = inner
        .stop_applied_since(base_before_thermal)
        .expect("owner ThermalTrip stop after admission must produce QualifyingFaultStopApplied proof");
    assert!(matches!(
        proof_thermal.event(),
        StopProofEvent::QualifyingFaultStopApplied { epoch } if epoch > base_before_thermal.qualifying_fault_epoch
    ));

    // 1g. Lease loss revokes baseline authority but NEVER qualifies as applied-stop proof.
    let base_before_lease_loss = inner.capture_baseline();
    assert!(inner.is_baseline_lease_live_at(base_before_lease_loss, now));
    inner.release_lease("op-client", lease_gen);
    assert!(
        !inner.is_baseline_lease_live_at(base_before_lease_loss, now),
        "lease loss must revoke baseline authority"
    );
    assert_eq!(
        inner.stop_applied_since(base_before_lease_loss),
        None,
        "lease loss alone must NOT qualify as stop proof"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn authority_proof_invalidated_across_owner_generation_handoff_and_handback() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    sup.set_lease_enabled(false).await;

    // Capture baseline on Legacy (owner_generation == 1).
    let legacy_baseline = {
        let inner = sup.inner().read().await;
        assert_eq!(inner.owner_generation(), 1);
        inner.capture_baseline()
    };

    // Handoff Legacy -> Executor advances owner_generation (1 -> 2).
    let exec = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(8)))
        .await
        .unwrap();
    let dl = Instant::now() + Duration::from_secs(2);
    while sup.hub().latest().is_none() {
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Trigger an applied stop on Executor, then hand back to Legacy (owner_generation 2 -> 3).
    let _ = sup.trigger_e_stop_checked().await.unwrap();
    sup.handback_to_legacy(Duration::from_secs(2))
        .await
        .unwrap();
    drop(exec);

    let inner = sup.inner().read().await;
    assert_eq!(inner.owner_generation(), 3, "Legacy -> Executor -> Legacy must advance owner_generation twice");
    // Even though an E-stop was applied after `legacy_baseline`, owner_generation changed, so
    // `legacy_baseline` MUST NOT produce cross-generation proof!
    assert_eq!(
        inner.stop_applied_since(legacy_baseline),
        None,
        "evidence from a different owner generation must NEVER satisfy an older baseline"
    );
    assert!(
        inner.new_estop_since(legacy_baseline),
        "different owner generation must fail closed on recovery gate"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_parity_applied_stop_vs_lease_loss_mid_mode_switch() {
    let mut results = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        // Case A: Applied ordinary STOP mid-switch -> StoppedRecovered on BOTH owners.
        let h_stop = DualOwnerRig::start_damiao(owner, true, false, true).await;
        let _sink_a = h_stop.register_routed_client(401, 4);
        h_stop
            .bus
            .as_ref()
            .unwrap()
            .with(|s| s.motors[0].mute_feedback_after_mode_ack = true);
        let mut plan = ModeSwitchPlan::switch_axes(
            &[(0, ControlMode::Impedance)],
            ModeSwitchRecovery::FailClosed,
        )
        .unwrap();
        plan.per_axis_timeout = Duration::from_secs(2);
        let sid_stop = h_stop
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h_stop.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(4),
                client: ClientId(401),
            })
            .await
            .unwrap();
        let dl = Instant::now() + Duration::from_secs(2);
        while h_stop
            .bus
            .as_ref()
            .unwrap()
            .with(|s| s.motors[0].ctrl_mode())
            != damiao::CTRL_MODE_MIT
        {
            assert!(Instant::now() < dl, "{owner:?}: J1 must switch forward");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        tokio::time::sleep(Duration::from_millis(60)).await;
        let out = h_stop
            .sup
            .urgent_stop(UrgentStop {
                connection: HolderLabel::new("cons-pr1-client"),
                lease_generation: h_stop.lease_gen,
                epoch: Some(h_stop.sup.estop_epoch()),
                request_id: 1,
                request_id_valid: true,
                body_error: None,
            })
            .await
            .unwrap();
        assert!(matches!(out, UrgentStopOutcome::Applied { .. }), "{owner:?}: {out:?}");
        let dl = Instant::now() + Duration::from_secs(2);
        let stop_prog = loop {
            if let Some(snap) = h_stop.sup.poll_mode_switch_session(sid_stop).await.unwrap() {
                if !snap.running {
                    break snap.progress;
                }
            }
            assert!(Instant::now() < dl, "{owner:?}: switch did not end after STOP");
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            stop_prog,
            ModeProgress::StoppedRecovered {
                reason: ModeSwitchFailure::Stopped
            },
            "{owner:?}: applied STOP after admission must recover cleanly"
        );
        if owner == OwnerKind::Executor {
            assert_eq!(h_stop.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO);
            assert_eq!(h_stop.sup.inner().control_thread_attempts(), 0);
        }
        h_stop
            .bus
            .as_ref()
            .unwrap()
            .with(|s| {
                s.immediate_replies = true;
                s.motors[0].mute_feedback_after_mode_ack = false;
                s.motors[0].mute_feedback = false;
            });
        h_stop.shutdown(true).await;

        // Case B: Lease loss mid-switch (NO applied stop) -> Ambiguous (SAFE-HOLD) on BOTH owners.
        let h_lease = DualOwnerRig::start_damiao(owner, true, false, true).await;
        let _sink_b = h_lease.register_routed_client(402, 4);
        h_lease
            .bus
            .as_ref()
            .unwrap()
            .with(|s| s.motors[0].mute_feedback_after_mode_ack = true);
        let sid_lease = h_lease
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h_lease.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(4),
                client: ClientId(402),
            })
            .await
            .unwrap();
        let dl = Instant::now() + Duration::from_secs(2);
        while h_lease
            .bus
            .as_ref()
            .unwrap()
            .with(|s| s.motors[0].ctrl_mode())
            != damiao::CTRL_MODE_MIT
        {
            assert!(Instant::now() < dl, "{owner:?}: J1 must switch forward");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        tokio::time::sleep(Duration::from_millis(60)).await;
        // Release lease via unified dual-owner path: revokes authority WITHOUT applying a stop.
        if owner == OwnerKind::Legacy {
            h_lease.sup.register_ws_connection(402).await.unwrap();
        }
        let lease_conn = ConnectionHandle::try_new(402, "cons-pr1-client").unwrap();
        let _ = h_lease
            .sup
            .release_lease_ws(lease_conn, h_lease.lease_gen)
            .await
            .unwrap();
        let dl = Instant::now() + Duration::from_secs(2);
        let lease_prog = loop {
            if let Some(snap) = h_lease.sup.poll_mode_switch_session(sid_lease).await.unwrap() {
                if !snap.running {
                    break snap.progress;
                }
            }
            assert!(Instant::now() < dl, "{owner:?}: switch did not end after lease loss");
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            lease_prog,
            ModeProgress::Ambiguous {
                failed_joint: 0,
                reason: ModeSwitchFailure::Revoked
            },
            "{owner:?}: lease loss without applied stop must escalate to Ambiguous"
        );
        h_lease
            .bus
            .as_ref()
            .unwrap()
            .with(|s| {
                s.immediate_replies = true;
                s.motors[0].mute_feedback_after_mode_ack = false;
                s.motors[0].mute_feedback = false;
            });
        h_lease.shutdown(true).await;

        results.push((stop_prog, lease_prog));
    }
    assert_eq!(results[0], results[1], "Legacy and Executor authority-proof parity");
}

// ============================================================================
// 2. Primitive 2 (`OperatorGrant` / `OperatorRequirement`): revocation between
//    queueing & admission, revocation between staging & activation, dual-owner parity
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_gravity_comp_grant_revocation_and_fail_closed_not_required() {
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let auth = OperatorAuth::enforced("1234", 300);
        let h = DualOwnerRig::start_mock(owner, true, true).await;
        let epoch = h.sup.estop_epoch();
        let lease_holder = Some((HolderLabel::new("cons-pr1-client"), h.lease_gen));

        // 2a. Fail-closed: protected gravity_comp Enter rejects `NotRequired` when auth enforced.
        let err_not_req = gravity_comp_command(
            &h.sup,
            capture_config_admission(&h.sup, 1, h.lease_gen, Some(epoch)),
            lease_holder,
            OperatorRequirement::NotRequired,
            true,
            Some(0.8),
            Some(vec![0.2; 6]),
        )
        .await
        .unwrap_err();
        assert!(
            matches!(&err_not_req, ArmError::Rejected { status: 403, code, .. } if code == "OPERATOR_REQUIRED"),
            "{owner:?}: NotRequired on enforced gravity_comp Enter must fail closed, got {err_not_req:?}"
        );

        // 2b. Revocation between queueing (grant capture) and admission:
        //     capture live grant -> logout -> dispatch Enter -> 403 OPERATOR_REQUIRED.
        let (tok_revoked, _) = auth.unlock("1234").unwrap();
        let revoked_grant = auth.operator_grant(Some(&tok_revoked));
        assert!(revoked_grant.is_valid_at(Instant::now(), true));
        auth.lock(Some(&tok_revoked));
        let err_revoked = gravity_comp_command(
            &h.sup,
            capture_config_admission(&h.sup, 1, h.lease_gen, Some(epoch)),
            lease_holder,
            OperatorRequirement::Required(revoked_grant),
            true,
            Some(0.8),
            Some(vec![0.2; 6]),
        )
        .await
        .unwrap_err();
        assert!(
            matches!(&err_revoked, ArmError::Rejected { status: 403, code, .. } if code == "OPERATOR_REQUIRED"),
            "{owner:?}: grant revoked before admission must reject gravity_comp Enter, got {err_revoked:?}"
        );

        // 2c. Live grant succeeds on BOTH owners, and unprivileged Exit succeeds afterwards.
        let (tok_live, _) = auth.unlock("1234").unwrap();
        let live_grant = auth.operator_grant(Some(&tok_live));
        let enter = gravity_comp_command(
            &h.sup,
            capture_config_admission(&h.sup, 1, h.lease_gen, Some(epoch)),
            lease_holder,
            OperatorRequirement::Required(live_grant),
            true,
            Some(0.8),
            Some(vec![0.2; 6]),
        )
        .await
        .unwrap_or_else(|e| panic!("{owner:?}: live grant must admit gravity_comp Enter: {e:?}"));
        assert!(enter.active);
        let exit = gravity_comp_command(
            &h.sup,
            capture_config_admission(&h.sup, 1, h.lease_gen, Some(epoch)),
            lease_holder,
            OperatorRequirement::NotRequired,
            false,
            None,
            None,
        )
        .await
        .unwrap();
        assert!(exit.is_torqued);
        if owner == OwnerKind::Executor {
            assert_eq!(h.exec.as_ref().unwrap().stats().total_allocs(), AllocCounts::ZERO);
            assert_eq!(h.sup.inner().control_thread_attempts(), 0);
        }
        h.shutdown(false).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_mode_switch_grant_revocation_before_admission_and_mid_session() {
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let auth = OperatorAuth::enforced("5678", 300);
        let h = DualOwnerRig::start_damiao(owner, true, true, false).await;
        let _sink = h.register_routed_client(501, 4);
        let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();

        // 2d. Grant revoked between queueing and admission -> OPERATOR_REQUIRED + 0 leaked credits.
        let (tok_a, _) = auth.unlock("5678").unwrap();
        let grant_a = auth.operator_grant(Some(&tok_a));
        auth.lock(Some(&tok_a));
        let err = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(grant_a),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(501),
            })
            .await
            .unwrap_err();
        assert!(
            matches!(err, ConfigError::CalibPrecondition { status: 403, code: "OPERATOR_REQUIRED", .. }),
            "{owner:?}: revoked grant before admission must fail with OPERATOR_REQUIRED, got {err:?}"
        );
        if let Some(exec) = h.exec.as_ref() {
            assert_eq!(exec.router_global_reserved(), 0, "rejected routed start must release credit");
            assert_eq!(exec.router_connection_reserved(ClientId(501)), Some(0));
        }

        // 2e. NotRequired on protected ModeSwitch fails closed when operator auth is enforced.
        let err_nr = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::NotRequired,
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(501),
            })
            .await
            .unwrap_err();
        assert!(
            matches!(err_nr, ConfigError::CalibPrecondition { status: 403, code: "OPERATOR_REQUIRED", .. }),
            "{owner:?}: NotRequired on enforced ModeSwitch must fail closed, got {err_nr:?}"
        );

        // 2f. Grant revoked mid-session while J7 is silent -> session revoked with OPERATOR_REQUIRED.
        h.bus.as_ref().unwrap().with(|s| s.motors[6].silent = true);
        let (tok_b, _) = auth.unlock("5678").unwrap();
        let grant_b = auth.operator_grant(Some(&tok_b));
        let mut slow_plan = plan;
        slow_plan.per_axis_timeout = Duration::from_secs(3);
        let sid = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan: slow_plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(grant_b),
                deadline: Instant::now() + Duration::from_secs(4),
                client: ClientId(501),
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(h.sup.poll_mode_switch_session(sid).await.unwrap().unwrap().running);
        // Revoke operator session while ModeSwitch is live!
        auth.lock(Some(&tok_b));
        let dl = Instant::now() + Duration::from_secs(2);
        let snap = loop {
            if let Some(s) = h.sup.poll_mode_switch_session(sid).await.unwrap() {
                if !s.running {
                    break s;
                }
            }
            assert!(Instant::now() < dl, "{owner:?}: mid-session operator revocation must end ModeSwitch");
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            snap.progress,
            ModeProgress::StoppedRecovered {
                reason: ModeSwitchFailure::Revoked
            },
            "{owner:?}: disarmed ModeSwitch revoked by operator logout should terminate cleanly"
        );
        h.shutdown(false).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_grant_revocation_between_staging_and_activation_blocks_commit() {
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let auth = OperatorAuth::enforced("9999", 300);
        let h = DualOwnerRig::start_damiao(owner, true, true, false).await;

        let (tok, _) = auth.unlock("9999").unwrap();
        let grant = auth.operator_grant(Some(&tok));
        let epoch = h.sup.estop_epoch();
        let pub_cfg = h.sup.published_config().await.unwrap();
        let base_rev = pub_cfg.calibration_revision;
        let ws_rev = pub_cfg.workspace_revision;
        let stamp = capture_config_admission(&h.sup, 0, h.lease_gen, Some(epoch));
        let cfg_auth = ConfigAuthContext {
            admission: stamp,
            lease_holder: Some((HolderLabel::new("cons-pr1-client"), h.lease_gen)),
        };
        let proposal = build_config_proposal(
            &h.sup,
            ConfigDomain::Calibration,
            base_rev,
            stamp,
            h.lease_gen,
            ws_rev,
        );

        // Stage (Reserve) while grant is live -> succeeds.
        let reservation = match h
            .sup
            .dispatch_config_request(ConfigRequest::Reserve(ReserveConfig {
                proposal,
                auth: cfg_auth,
                precheck: ReservePrecheck::MitCommissioning {
                    allow_during_calibration_busy: true,
                    expected_txn_id: None,
                    operator_grant: grant,
                    require_driver: false,
                },
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
            .unwrap()
        {
            ConfigReply::Reserved(Ok(r)) => r,
            ConfigReply::Reserved(Err(e)) => panic!("{owner:?}: staging reserve with live grant failed: {e:?}"),
            _ => panic!("{owner:?}: unexpected reply to Reserve"),
        };

        // Revoke operator session BETWEEN staging (Reserve) and activation (VerifyAuthority + Activate)!
        auth.lock(Some(&tok));

        // VerifyAuthority (pre-rename phase) MUST fail closed with OPERATOR_REQUIRED.
        let verify = h
            .sup
            .dispatch_config_request(ConfigRequest::VerifyAuthority {
                domain: ConfigDomain::Calibration,
                grant: reservation.grant(),
                proposal,
                auth: cfg_auth,
                expected_workspace_revision: None,
                expected_poses_revision: None,
                domain_checks: VerifyDomainChecks {
                    operator_grant: Some(grant),
                    ..VerifyDomainChecks::default()
                },
            })
            .await
            .unwrap();
        assert!(
            matches!(
                verify,
                ConfigReply::AuthorityVerified(Err(ConfigError::CalibPrecondition {
                    status: 403,
                    code: "OPERATOR_REQUIRED",
                    ..
                }))
            ),
            "{owner:?}: VerifyAuthority after logout must reject OPERATOR_REQUIRED"
        );

        // Activate (Volatile / pre-persist) after logout MUST fail closed with OPERATOR_REQUIRED
        // (releasing the hold cleanly) and NOT bump revision.
        let activate_vol = h
            .sup
            .dispatch_config_request(ConfigRequest::Activate(ActivateConfig {
                reservation,
                auth: cfg_auth,
                revoked_off_thread: None,
                candidate: ConfigCandidate::MitQualification {
                    calibration: None,
                    #[cfg(feature = "policy-runtime")]
                    calibration_digest: String::new(),
                    expected_txn_id: None,
                    operator_grant: grant,
                },
                durability: Durability::Volatile,
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
            .unwrap();
        assert!(
            matches!(activate_vol, ConfigReply::Activated(Err(ConfigError::Busy("OPERATOR_REQUIRED")))),
            "{owner:?}: Volatile Activate after logout must reject OPERATOR_REQUIRED"
        );
        let after_vol = h.sup.published_config().await.unwrap();
        assert_eq!(
            after_vol.calibration_revision, base_rev,
            "{owner:?}: revoked volatile activation must NOT mutate calibration_revision"
        );

        // Durable post-staging revocation: Reserve -> Persisted -> logout -> Activate(Durable)
        // MUST enter Reconciling("OPERATOR_REQUIRED") without mutating active revision.
        let (tok2, _) = auth.unlock("9999").unwrap();
        let grant2 = auth.operator_grant(Some(&tok2));
        let stamp2 = capture_config_admission(&h.sup, 0, h.lease_gen, Some(epoch));
        let cfg_auth2 = ConfigAuthContext {
            admission: stamp2,
            lease_holder: Some((HolderLabel::new("cons-pr1-client"), h.lease_gen)),
        };
        let proposal2 = build_config_proposal(
            &h.sup,
            ConfigDomain::Calibration,
            base_rev,
            stamp2,
            h.lease_gen,
            ws_rev,
        );
        let res2 = match h
            .sup
            .dispatch_config_request(ConfigRequest::Reserve(ReserveConfig {
                proposal: proposal2,
                auth: cfg_auth2,
                precheck: ReservePrecheck::MitCommissioning {
                    allow_during_calibration_busy: true,
                    expected_txn_id: None,
                    operator_grant: grant2,
                    require_driver: false,
                },
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
            .unwrap()
        {
            ConfigReply::Reserved(Ok(r)) => r,
            ConfigReply::Reserved(Err(e)) => panic!("{owner:?}: durable staging reserve failed: {e:?}"),
            _ => panic!("{owner:?}: unexpected reply to durable Reserve"),
        };
        let persisted_res = match h
            .sup
            .dispatch_config_request(ConfigRequest::Persisted(PersistedConfig {
                reservation: res2,
                durability: Durability::Durable,
            }))
            .await
            .unwrap()
        {
            ConfigReply::Persisted(Ok(r)) => r,
            ConfigReply::Persisted(Err(e)) => panic!("{owner:?}: Persisted failed: {e:?}"),
            _ => panic!("{owner:?}: unexpected reply to Persisted"),
        };
        auth.lock(Some(&tok2));
        let activate_dur = h
            .sup
            .dispatch_config_request(ConfigRequest::Activate(ActivateConfig {
                reservation: persisted_res,
                auth: cfg_auth2,
                revoked_off_thread: None,
                candidate: ConfigCandidate::MitQualification {
                    calibration: None,
                    #[cfg(feature = "policy-runtime")]
                    calibration_digest: String::new(),
                    expected_txn_id: None,
                    operator_grant: grant2,
                },
                durability: Durability::Durable,
                expected_workspace_revision: None,
                expected_poses_revision: None,
            }))
            .await
            .unwrap();
        assert!(
            matches!(activate_dur, ConfigReply::Activated(Err(ConfigError::Reconciling("OPERATOR_REQUIRED")))),
            "{owner:?}: Durable Activate after logout must enter Reconciling(OPERATOR_REQUIRED)"
        );
        let after_cfg = h.sup.published_config().await.unwrap();
        assert_eq!(
            after_cfg.calibration_revision, base_rev,
            "{owner:?}: revoked durable activation must NOT mutate calibration_revision"
        );
        h.shutdown(false).await;
    }
}

// ============================================================================
// 3. Primitive 3 (Session admission & terminal credit accounting):
//    missing destination, full credit, rejection, disconnect, slot reuse, exact counts
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn routed_session_admission_terminal_credit_invariants_and_slot_reuse() {
    let h = DualOwnerRig::start_damiao(OwnerKind::Executor, true, false, false).await;
    let exec = h.exec.as_ref().unwrap();
    let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();

    // 3a. Missing destination: routed client with NO registered reply sink MUST fail closed
    //     with NO_DESTINATION (never downgraded to Ok(None)), with 0 credits reserved.
    let err_no_dest = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(900),
        })
        .await
        .unwrap_err();
    assert!(
        matches!(
            err_no_dest,
            ConfigError::CalibPrecondition {
                status: 409,
                code: "NO_DESTINATION",
                ..
            }
        ),
        "missing destination must fail closed with NO_DESTINATION, got {err_no_dest:?}"
    );
    assert_eq!(exec.router_global_reserved(), 0);

    // 3b. Direct ConfigRequest::StartModeSwitch with non-zero client and `None` reservation
    //     MUST also fail closed at the owner (defense in depth).
    let direct_no_res = h
        .sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartModeSwitch(
            ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(900),
            },
            None,
        ))
        .await
        .unwrap();
    assert!(
        matches!(
            direct_no_res,
            ConfigReply::ModeSwitchStarted(Err(ConfigError::CalibPrecondition {
                status: 409,
                code: "NO_DESTINATION",
                ..
            }))
        ),
        "direct StartModeSwitch without reservation must fail closed"
    );
    assert_eq!(exec.router_global_reserved(), 0);

    // 3c. Explicit internal admission path (`ClientId(0)` — no external caller) succeeds cleanly
    //     without consuming router credits.
    let snap_internal = h
        .sup
        .run_mode_switch_session(
            ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(0),
            },
            Duration::from_secs(2),
        )
        .await
        .unwrap();
    assert_eq!(snap_internal.progress, ModeProgress::Confirmed);
    assert_eq!(exec.router_global_reserved(), 0);

    // 3d. Full credit saturation (cap = 1) + RAII guard single-use staging + rollback on drop.
    let delivered_a = h.register_routed_client(901, 1).unwrap();
    let mut guard1 = exec
        .reserve_session_terminal(
            ClientId(901),
            ActionId(100),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(901)), Some(1));
    // Staging twice on the same guard returns None (at-most-once reservation transfer).
    let staged = guard1.stage_for_admission();
    assert!(staged.is_some());
    assert!(guard1.stage_for_admission().is_none());
    // While cap=1 is saturated, a routed ModeSwitch start MUST fail closed with REPLIES_FULL.
    let err_full = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(901),
        })
        .await
        .unwrap_err();
    assert!(
        matches!(err_full, ConfigError::Busy("REPLIES_FULL")),
        "saturated credit must fail with REPLIES_FULL, got {err_full:?}"
    );
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(901)), Some(1));
    // Drop un-committed guard -> RAII releases credit cleanly to exact 0.
    drop(staged);
    drop(guard1);
    assert_eq!(exec.router_global_reserved(), 0);
    assert_eq!(exec.router_connection_reserved(ClientId(901)), Some(0));

    // 3e. Rejection at owner (stale lease) -> RAII guard releases credit immediately to 0.
    let mut bad_lease = h.sup.capture_admission_stamp();
    bad_lease.lease_gen = bad_lease.lease_gen.wrapping_add(55);
    let err_rej = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: bad_lease,
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(901),
        })
        .await
        .unwrap_err();
    assert!(matches!(err_rej, ConfigError::LeaseLost), "{err_rej:?}");
    assert_eq!(exec.router_global_reserved(), 0);
    assert_eq!(exec.router_connection_reserved(ClientId(901)), Some(0));

    // 3f. Session slot reuse with exact credit counts & terminal delivery (2 consecutive routed runs).
    for run_idx in 1..=2usize {
        let snap = h
            .sup
            .run_mode_switch_session(
                ModeSwitchStartRequest {
                    plan,
                    admission: h.sup.capture_admission_stamp(),
                    operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                    deadline: Instant::now() + Duration::from_secs(2),
                    client: ClientId(901),
                },
                Duration::from_secs(2),
            )
            .await
            .unwrap();
        assert_eq!(snap.progress, ModeProgress::Confirmed);
        let dl = Instant::now() + Duration::from_secs(2);
        while delivered_a.lock().unwrap().len() < run_idx || exec.router_global_reserved() != 0 {
            let _ = exec.pump_broker_replies();
            assert!(Instant::now() < dl, "run {run_idx}: terminal delivery or credit recovery timed out");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(exec.router_global_reserved(), 0);
        assert_eq!(exec.router_connection_reserved(ClientId(901)), Some(0));
    }
    assert_eq!(delivered_a.lock().unwrap().len(), 2, "exactly 1 terminal per routed session");

    // 3g. Disconnect while routed session holds credit -> credit recovered to 0, closed client
    //     subsequent start fails with ConnectionClosed, AND router connection slot reuse by
    //     client B cannot be corrupted by stale ticket from client A.
    h.bus.as_ref().unwrap().with(|s| s.motors[6].silent = true);
    let mut slow_plan = plan;
    slow_plan.per_axis_timeout = Duration::from_secs(3);
    let sid_disc = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan: slow_plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(4),
            client: ClientId(901),
        })
        .await
        .unwrap();
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(901)), Some(1));
    // Also grab a separate ticket for client 901 before disconnect? Cap=1 so disconnect directly.
    exec.disconnect_client(ClientId(901));
    let dl = Instant::now() + Duration::from_secs(2);
    while exec.router_global_reserved() != 0 {
        assert!(Instant::now() < dl, "disconnect must recover router credit");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(exec.router_connection_reserved(ClientId(901)), None);
    let _ = h.sup.cancel_mode_switch_session(sid_disc, "DISCONNECTED").await;
    let dl = Instant::now() + Duration::from_secs(2);
    while !h.sup.clear_mode_switch_session(sid_disc).await.unwrap_or(false) {
        if h.sup.poll_mode_switch_session(sid_disc).await.ok().flatten().is_none() {
            break;
        }
        assert!(Instant::now() < dl);
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // Closed client 901 MUST fail closed with ConnectionClosed (0 credit leak).
    let err_closed = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(901),
        })
        .await
        .unwrap_err();
    assert!(
        matches!(err_closed, ConfigError::ConnectionClosed),
        "disconnected client must fail with ConnectionClosed, got {err_closed:?}"
    );
    assert_eq!(exec.router_global_reserved(), 0);

    // Router slot reuse: register client 902 (takes slot), take stale ticket, disconnect 902,
    // register client 903 (reuses freed slot with newer slot_gen), reserve for 903, then attempt
    // stale release/emit using 902's ticket -> must NOT touch 903's credit!
    let _sink_902 = h.register_routed_client(902, 2).unwrap();
    let stale_ticket_902 = exec
        .reserve_session_terminal_direct(
            ClientId(902),
            ActionId(201),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    assert_eq!(exec.router_global_reserved(), 1);
    exec.disconnect_client(ClientId(902));
    assert_eq!(exec.router_global_reserved(), 0);

    let _sink_903 = h.register_routed_client(903, 2).unwrap();
    let mut guard_903 = exec
        .reserve_session_terminal(
            ClientId(903),
            ActionId(202),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(903)), Some(1));

    // Stale ticket from 902 MUST fail idempotently without decrementing 903's credit.
    assert!(!exec.release_session_terminal(&stale_ticket_902));
    assert_eq!(
        exec.emit_session_terminal(&stale_ticket_902, Outcome::Completed, 0),
        SessionTerminalEmit::DuplicateSuppressed
    );
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(903)), Some(1));

    // Releasing 903's guard returns exact credit count to 0.
    assert!(guard_903.release_now());
    assert!(!guard_903.release_now());
    assert_eq!(exec.router_global_reserved(), 0);
    assert_eq!(exec.router_connection_reserved(ClientId(903)), Some(0));

    // Direct `admit_routed` vs `admit_internal` client-mismatch negative test (unit boundary).
    let mut core = SupervisorInner::new(ArmConfig::rebot_dm());
    core.lease_enabled = false;
    let mut g_mismatch = exec
        .reserve_session_terminal(
            ClientId(903),
            ActionId(203),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    let res_mismatch = g_mismatch.stage_for_admission().unwrap();
    let adm = core
        .check_admission(make_stamp(0, 0, core.safety.epoch_id(), core.current_stop_gen(), 1))
        .unwrap();
    // PreparedSession says ClientId(999) while reservation belongs to ClientId(903) -> InvalidReservation!
    let prep_wrong_client = PreparedSession::exclusive(SessionKind::Custom(1), ClientId(999));
    assert_eq!(
        admit_routed(adm, prep_wrong_client, res_mismatch, Instant::now()),
        Err(armd::rt_core::session::SessionAdmitError::InvalidReservation)
    );
    assert!(!core.sessions().is_slot_occupied(SessionSlot::ExclusiveOutput));
    drop(g_mismatch);
    assert_eq!(exec.router_global_reserved(), 0);

    // Internal admission works without any reservation and does not touch router credits.
    let adm_int = core
        .check_admission(make_stamp(0, 0, core.safety.epoch_id(), core.current_stop_gen(), 2))
        .unwrap();
    let sid_int = admit_internal(
        adm_int,
        PreparedSession::exclusive(SessionKind::Custom(2), ClientId(0)),
        Instant::now(),
    )
    .unwrap();
    assert!(core.sessions().slot(SessionSlot::ExclusiveOutput).is_some_and(|s| s.id == sid_int));
    assert_eq!(exec.router_global_reserved(), 0);

    // Ownership-transition defense: an executor terminal reservation arriving at a Legacy owner
    // (`!rt_executor_owned`) MUST fail closed with OwnerStopped and release its credit on guard drop.
    let legacy_sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    legacy_sup.set_lease_enabled(false).await;
    let mut g_handback = exec
        .reserve_session_terminal(
            ClientId(903),
            ActionId(204),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    let res_handback = g_handback.stage_for_admission().unwrap();
    let handback_rep = legacy_sup
        .dispatch_config_request(ConfigRequest::StartModeSwitch(
            ModeSwitchStartRequest {
                plan,
                admission: legacy_sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(903),
            },
            Some(res_handback),
        ))
        .await
        .unwrap();
    assert!(matches!(
        handback_rep,
        ConfigReply::ModeSwitchStarted(Err(ConfigError::OwnerStopped))
    ));
    drop(g_handback);
    assert_eq!(exec.router_global_reserved(), 0);

    assert_eq!(exec.stats().total_allocs(), AllocCounts::ZERO);
    assert_eq!(h.sup.inner().control_thread_attempts(), 0);
    h.shutdown(false).await;
}

/// R1 Major 1 regression test: atomic handoff protocol prevents cancellation/admission races
/// both BEFORE owner admission (cancel prevents admission, credit released immediately, 0 RT
/// alloc/free) and AFTER owner admission before `commit_transferred()` (guard drop preserves
/// live session's route & credit until terminal delivery, blocking premature capacity reuse).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn terminal_reservation_cancellation_before_and_after_owner_admission() {
    let h = DualOwnerRig::start_damiao(OwnerKind::Executor, true, false, false).await;
    let exec = h.exec.as_ref().unwrap();
    let delivered = h.register_routed_client(910, 1).unwrap();
    let plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();

    // Case A: Caller cancels AFTER staging/enqueueing and BEFORE the owner processes the request.
    // Dropping the guard transitions PENDING -> CANCELLED and immediately releases the credit.
    // When the already-queued request reaches the owner, admission MUST fail closed (`CANCELLED`),
    // leave the slot idle, perform 0 RT allocs/frees, and keep credits at exact 0.
    let mut guard_pre = exec
        .reserve_session_terminal(
            ClientId(910),
            ActionId(301),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    let staged_pre = guard_pre.stage_for_admission().unwrap();
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(910)), Some(1));
    drop(guard_pre);
    assert_eq!(exec.router_global_reserved(), 0);
    assert_eq!(exec.router_connection_reserved(ClientId(910)), Some(0));

    let pre_rep = h
        .sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartModeSwitch(
            ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(2),
                client: ClientId(910),
            },
            Some(staged_pre),
        ))
        .await
        .unwrap();
    assert!(matches!(
        pre_rep,
        ConfigReply::ModeSwitchStarted(Err(ConfigError::Busy("CANCELLED")))
    ));
    assert_eq!(exec.router_global_reserved(), 0);
    assert_eq!(exec.router_connection_reserved(ClientId(910)), Some(0));
    assert_eq!(delivered.lock().unwrap().len(), 0);

    // Case B: Caller cancels AFTER the owner commits admission (`PENDING -> ADMITTED`) and BEFORE
    // `commit_transferred()` runs on the caller side.
    // Dropping the guard MUST NOT release the credit or delete the live route; cap=1 capacity
    // remains occupied until the live session finishes and delivers its 1 terminal.
    h.bus.as_ref().unwrap().with(|s| s.motors[6].silent = true);
    let mut slow_plan = plan;
    slow_plan.per_axis_timeout = Duration::from_secs(3);

    let mut guard_post = exec
        .reserve_session_terminal(
            ClientId(910),
            ActionId(302),
            ExternalDestinationPolicy::ConnectionOnly,
        )
        .unwrap();
    let staged_post = guard_post.stage_for_admission().unwrap();
    let sid = match h
        .sup
        .dispatch_config_request_owned_reliable(ConfigRequest::StartModeSwitch(
            ModeSwitchStartRequest {
                plan: slow_plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
                deadline: Instant::now() + Duration::from_secs(4),
                client: ClientId(910),
            },
            Some(staged_post),
        ))
        .await
        .unwrap()
    {
        ConfigReply::ModeSwitchStarted(Ok(id)) => id,
        ConfigReply::ModeSwitchStarted(Err(e)) => panic!("expected Ok admission, got {e:?}"),
        _ => panic!("unexpected reply"),
    };
    // Simulate future cancellation before `commit_transferred()`!
    assert!(!guard_post.release_now(), "admitted handoff must block guard release");
    drop(guard_post);
    assert_eq!(exec.router_global_reserved(), 1);
    assert_eq!(exec.router_connection_reserved(ClientId(910)), Some(1));

    // Capacity (cap=1) MUST NOT be reusable while the admitted session is still live.
    let err_full = h
        .sup
        .start_mode_switch_session(ModeSwitchStartRequest {
            plan,
            admission: h.sup.capture_admission_stamp(),
            operator: OperatorRequirement::Required(OperatorGrant::unenforced()),
            deadline: Instant::now() + Duration::from_secs(2),
            client: ClientId(910),
        })
        .await
        .unwrap_err();
    assert!(matches!(err_full, ConfigError::Busy("REPLIES_FULL")), "{err_full:?}");
    assert_eq!(exec.router_global_reserved(), 1);

    // Unmute J7 and deliver its pending CTRL_MODE read reply so the live session completes
    // (`Confirmed`) and delivers its 1 routed terminal cleanly.
    h.bus.as_ref().unwrap().with(|s| {
        s.motors[6].silent = false;
        let [lo, hi] = s.motors[6].id.to_le_bytes();
        let val = damiao::CTRL_MODE_POS_FORCE.to_le_bytes();
        let reply = armd::transport::CanFrame::new_standard(
            s.motors[6].reply_id,
            &[lo, hi, damiao::REG_READ, damiao::RID_CTRL_MODE, val[0], val[1], val[2], val[3]],
        )
        .unwrap();
        s.rx.push_back(reply);
    });
    let dl = Instant::now() + Duration::from_secs(2);
    let snap = loop {
        if let Some(s) = h.sup.poll_mode_switch_session(sid).await.unwrap() {
            if !s.running {
                break s;
            }
        }
        assert!(Instant::now() < dl, "live session did not complete");
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert_eq!(snap.progress, ModeProgress::Confirmed);
    let dl = Instant::now() + Duration::from_secs(2);
    while delivered.lock().unwrap().is_empty() || exec.router_global_reserved() != 0 {
        let _ = exec.pump_broker_replies();
        assert!(Instant::now() < dl, "routed terminal delivery or credit release timed out");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(delivered.lock().unwrap().len(), 1);
    assert_eq!(exec.router_global_reserved(), 0);
    assert_eq!(exec.router_connection_reserved(ClientId(910)), Some(0));
    assert_eq!(exec.stats().total_allocs(), AllocCounts::ZERO);
    assert_eq!(h.sup.inner().control_thread_attempts(), 0);
    h.shutdown(false).await;
}

/// R1 Major 2 regression test: a protected `ModeSwitch` admitted with `OperatorRequirement::NotRequired`
/// while operator auth enforcement is disabled MUST fail closed and revoke immediately on BOTH
/// `Legacy` and `Executor` if operator auth enforcement is enabled while the session is live.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_enabling_operator_auth_mid_not_required_mode_switch_revokes() {
    let mut outcomes = Vec::new();
    for owner in [OwnerKind::Legacy, OwnerKind::Executor] {
        let h = DualOwnerRig::start_damiao(owner, true, false, false).await;
        let _sink = h.register_routed_client(520, 4);
        // Keep J7 silent so the ModeSwitch session stays live after admission.
        h.bus.as_ref().unwrap().with(|s| s.motors[6].silent = true);
        let mut plan = ModeSwitchPlan::confirm_gripper(6, ControlMode::PositionForce).unwrap();
        plan.per_axis_timeout = Duration::from_secs(3);

        // Admit protected ModeSwitch with `NotRequired` while enforcement is disabled -> succeeds.
        let sid = h
            .sup
            .start_mode_switch_session(ModeSwitchStartRequest {
                plan,
                admission: h.sup.capture_admission_stamp(),
                operator: OperatorRequirement::NotRequired,
                deadline: Instant::now() + Duration::from_secs(4),
                client: ClientId(520),
            })
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(
            h.sup.poll_mode_switch_session(sid).await.unwrap().unwrap().running,
            "{owner:?}: session should still be running while enforcement is off"
        );

        // Enable operator auth enforcement while the session is live!
        h.sup.set_operator_auth_enforced(true).await;

        let dl = Instant::now() + Duration::from_secs(2);
        let snap = loop {
            if let Some(s) = h.sup.poll_mode_switch_session(sid).await.unwrap() {
                if !s.running {
                    break s;
                }
            }
            assert!(
                Instant::now() < dl,
                "{owner:?}: enabling operator auth enforcement mid-session MUST revoke NotRequired ModeSwitch"
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        };
        assert_eq!(
            snap.progress,
            ModeProgress::StoppedRecovered {
                reason: ModeSwitchFailure::Revoked
            },
            "{owner:?}: expected revocation on enforcement enable, got {:?}",
            snap.progress
        );
        if let Some(exec) = h.exec.as_ref() {
            let dl = Instant::now() + Duration::from_secs(2);
            while exec.router_global_reserved() != 0 {
                let _ = exec.pump_broker_replies();
                assert!(Instant::now() < dl);
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            assert_eq!(exec.stats().total_allocs(), AllocCounts::ZERO);
            assert_eq!(h.sup.inner().control_thread_attempts(), 0);
        }
        outcomes.push(snap.progress);
        h.shutdown(false).await;
    }
    assert_eq!(outcomes[0], outcomes[1], "Legacy and Executor enforcement-toggle revocation parity");
}
