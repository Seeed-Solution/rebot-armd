//! Regression tests for active read-only `0xCC` shutdown disarm proof across RT executor and
//! legacy owners (dev21 Stage C: cold-boot idle disarmed arm receiving SIGTERM / `stop_and_join`).

mod mit_support;

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, HoldPassReport,
    JointTarget, TorqueProgress,
};
use armd::model::{ArmConfig, JointState};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{ClientId, Outcome, Reply, RequestId, Submit, TransitionOp};
use armd::rt_core::executor::{ExecutorConfig, PanicPolicy, RtExecutorHandle};
use armd::safety::SafetyState;
use armd::supervisor::{ArmSupervisor, PanicDisableOutcome};
use armd::transport::{CanFrame, MockClock};
use mit_support::FakeBus;
use tracing_subscriber::fmt::MakeWriter;

static LOG_BUF: OnceLock<Arc<Mutex<String>>> = OnceLock::new();

#[derive(Clone)]
struct SharedLogWriter(Arc<Mutex<String>>);

struct LockedLogWriter(Arc<Mutex<String>>);

impl Write for LockedLogWriter {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut guard = self.0.lock().unwrap_or_else(|e| e.into_inner());
        guard.push_str(&String::from_utf8_lossy(buf));
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl<'a> MakeWriter<'a> for SharedLogWriter {
    type Writer = LockedLogWriter;
    fn make_writer(&'a self) -> Self::Writer {
        LockedLogWriter(Arc::clone(&self.0))
    }
}

fn log_buf() -> Arc<Mutex<String>> {
    Arc::clone(LOG_BUF.get_or_init(|| {
        let buf = Arc::new(Mutex::new(String::new()));
        let _ = tracing_subscriber::fmt()
            .with_ansi(false)
            .with_max_level(tracing::Level::WARN)
            .with_writer(SharedLogWriter(Arc::clone(&buf)))
            .try_init();
        buf
    }))
}

fn take_logs() -> String {
    let buf = log_buf();
    let mut g = buf.lock().unwrap_or_else(|e| e.into_inner());
    std::mem::take(&mut *g)
}

fn dm_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm()
        .damiao_ranges
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect()
}

struct DropWatchDamiao {
    inner: DamiaoArmDriver<FakeBus>,
    dropped: Arc<AtomicBool>,
}

impl Drop for DropWatchDamiao {
    fn drop(&mut self) {
        self.dropped.store(true, Ordering::Release);
    }
}

impl ArmDriver for DropWatchDamiao {
    fn capabilities(&self) -> DriverCapabilities {
        self.inner.capabilities()
    }
    fn enable_state(&self) -> EnableState {
        self.inner.enable_state()
    }
    fn is_hardware_disarmed_confirmed(&self) -> bool {
        self.inner.is_hardware_disarmed_confirmed()
    }
    fn probe_hardware_disarmed_until(&mut self, deadline: Instant) -> bool {
        self.inner.probe_hardware_disarmed_until(deadline)
    }
    fn has_driver_fresh_position_api(&self) -> bool {
        self.inner.has_driver_fresh_position_api()
    }
    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        self.inner.fresh_measured_position_for(joint, max_age)
    }
    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        self.inner.fresh_measured_sample_for(joint, max_age)
    }
    fn joint_hold_mode_verified(&self, joint: usize) -> bool {
        self.inner.joint_hold_mode_verified(joint)
    }
    fn is_feedback_valid(&self) -> bool {
        self.inner.is_feedback_valid()
    }
    fn is_enabled(&self) -> bool {
        self.inner.is_enabled()
    }
    fn has_any_joint_enabled(&self) -> bool {
        self.inner.has_any_joint_enabled()
    }
    fn has_known_partial_enable(&self) -> bool {
        self.inner.has_known_partial_enable()
    }
    fn torque_transition_pending(&self) -> bool {
        self.inner.torque_transition_pending()
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        self.inner.feedback_ages()
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.inner.feedback_instants()
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        self.inner.feedback_seqs()
    }
    fn feedback_queue_bounds(&self) -> Vec<Option<Instant>> {
        self.inner.feedback_queue_bounds()
    }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.inner.feedback_receive_instants()
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        self.inner.feedback_ages_into(out);
    }
    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        self.inner.feedback_all_within(dof, max_age)
    }
    fn rest_pose_gate_status(&self) -> armd::driver::RestPoseGateStatus {
        self.inner.rest_pose_gate_status()
    }
    fn supports_budgeted_disable_await(&self) -> bool {
        self.inner.supports_budgeted_disable_await()
    }
    fn set_enable_authority(&mut self, deadline: Option<Instant>, expected_lease_gen: Option<u64>) {
        self.inner.set_enable_authority(deadline, expected_lease_gen);
    }
    fn set_disable_authority(&mut self, deadline: Option<Instant>, expected_lease_gen: Option<u64>) {
        self.inner.set_disable_authority(deadline, expected_lease_gen);
    }
    fn driver_clock(&self) -> Option<Arc<dyn armd::transport::DriverClock>> {
        self.inner.driver_clock()
    }
    fn disable_frames_dispatched(&self) -> bool {
        self.inner.disable_frames_dispatched()
    }
    fn disable_dispatched_mask(&self) -> u64 {
        self.inner.disable_dispatched_mask()
    }
    fn enable_frames_dispatched(&self) -> bool {
        self.inner.enable_frames_dispatched()
    }
    fn begin_control_release(&mut self, release_id: u64) {
        self.inner.begin_control_release(release_id);
    }
    fn set_output_lease_gen(&mut self, lease_gen: u64) {
        self.inner.set_output_lease_gen(lease_gen);
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        self.inner.read_joint_states()
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.inner
            .read_joint_states_budgeted(out, deadline, max_rx_frames)
    }
    fn read_joint_states_safe_hold(
        &mut self,
        out: &mut Vec<JointState>,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.inner
            .read_joint_states_safe_hold(out, deadline, max_rx_frames)
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        self.inner.read_cached_joint_states_into(out)
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.inner.write_joint_targets(targets)
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[JointTarget],
    ) -> Result<armd::driver::TxBatchOutcome, ArmDriverError> {
        self.inner.write_joint_targets_bounded(targets)
    }
    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<HoldPassReport, ArmDriverError> {
        self.inner.write_measured_hold_masked(hold_mask, targets)
    }
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.inner.set_torque(enabled)
    }
    fn disable_torque(&mut self, kind: armd::driver::DisableKind) -> Result<(), ArmDriverError> {
        self.inner.disable_torque(kind)
    }
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque(enabled)
    }
    fn begin_disable_torque(
        &mut self,
        kind: armd::driver::DisableKind,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_disable_torque(kind)
    }
    fn finish_dispatched_disable(&mut self) -> Result<(), ArmDriverError> {
        self.inner.finish_dispatched_disable()
    }
    fn poll_torque(&mut self, deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque(deadline)
    }
    fn begin_torque_budgeted(
        &mut self,
        enabled: bool,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_torque_budgeted(enabled, max_rx_frames)
    }
    fn begin_disable_torque_budgeted(
        &mut self,
        kind: armd::driver::DisableKind,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.begin_disable_torque_budgeted(kind, max_rx_frames)
    }
    fn poll_torque_budgeted(
        &mut self,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.inner.poll_torque_budgeted(deadline, max_rx_frames)
    }
    fn panic_disable_step(
        &mut self,
        first: bool,
        deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.inner.panic_disable_step(first, deadline)
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        self.inner.unconfirmed_disable_joints()
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        self.inner.supports_rt_executor()
    }
    fn set_rt_mode(&mut self, rt: bool) {
        self.inner.set_rt_mode(rt);
    }
    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.inner.set_tx_release_deadline(deadline);
    }
    fn set_estop_state(&mut self, state: Arc<armd::rt_core::urgent::EstopState>) {
        self.inner.set_estop_state(state);
    }
    fn set_estop_gate(&mut self, gate: armd::rt_core::urgent::EstopGate) {
        self.inner.set_estop_gate(gate);
    }
    fn set_stopped_disable_gate(&mut self, gate: armd::driver::StoppedDisableGate) {
        self.inner.set_stopped_disable_gate(gate);
    }
    fn prepare_safe_hold(&mut self) {
        self.inner.prepare_safe_hold();
    }
    fn clock_now(&self) -> Instant {
        self.inner.clock_now()
    }
    fn clock_pause(&self, d: Duration) {
        self.inner.clock_pause(d);
    }
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> {
        self.inner.position_limits(joint)
    }
}

fn build_watched_damiao(
    bus: FakeBus,
    seed_feedback: bool,
) -> (Arc<ArmSupervisor>, Arc<AtomicBool>) {
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(dm_limits())
        .unwrap();
    if seed_feedback {
        bus.with(|s| s.push_all_feedback());
        let _ = drv.read_joint_states().unwrap();
    }
    drv.check_mapping_ranges();
    let dropped = Arc::new(AtomicBool::new(false));
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(DropWatchDamiao {
            inner: drv,
            dropped: Arc::clone(&dropped),
        })),
    );
    (sup, dropped)
}

async fn wait_until(timeout: Duration, label: &str, mut cond: impl FnMut() -> bool) {
    let dl = Instant::now() + timeout;
    while Instant::now() < dl {
        if cond() {
            return;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    panic!("timed out waiting for {label}");
}

fn is_refresh_frame(f: &CanFrame) -> bool {
    f.id == 0x7FF && f.dlc == 8 && f.data[2] == 0xCC
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_cold_boot_never_enabled_sigterm_and_stop_and_join_clean_under_1s() {
    let _ = log_buf();

    // (a) Cold boot, never enabled, NO initial feedback pushed -> SIGTERM path
    // (`request_shutdown_until` + `shutdown_and_join_until`) actively polls 0xCC,
    // confirms all 7 axes disarmed, and exits cleanly in < 1 s with NO E-stop latch and NO SAFE-HOLD.
    {
        let bus = FakeBus::new(&dm_limits());
        bus.with(|s| {
            s.immediate_replies = true;
            s.tx_log.clear();
        });
        let (sup, dropped) = build_watched_damiao(bus.clone(), false);
        sup.set_lease_enabled(false).await;
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup.start_executor(cfg).await.expect("start_executor");
        wait_until(Duration::from_secs(1), "initial release", || {
            handle.stats().completed_releases.load(Ordering::Acquire) >= 1
        })
        .await;

        let _ = take_logs();
        let obs_before = sup.get_observation().await;
        assert_eq!(obs_before.safety_state, SafetyState::Idle);
        assert!(!obs_before.is_torqued);
        assert!(obs_before.fault.is_none());

        let t0 = Instant::now();
        let deadline = t0 + armd::supervisor::PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
        assert!(
            sup.request_shutdown_until(deadline).await.unwrap(),
            "cold-boot never-enabled executor must confirm shutdown via active 0xCC disarm proof"
        );
        let obs = sup.get_observation().await;
        assert!(!obs.is_torqued);

        let h_join = handle.clone();
        tokio::task::spawn_blocking(move || h_join.shutdown_and_join_until(deadline))
            .await
            .unwrap()
            .expect("shutdown_and_join_until must succeed cleanly");
        let elapsed = t0.elapsed();
        assert!(
            elapsed < Duration::from_secs(1),
            "clean cold-boot shutdown must finish in < 1 s (took {elapsed:?})"
        );
        assert!(!handle.is_in_safe_hold(), "must NOT enter SAFE-HOLD");
        assert!(handle.is_exited(), "executor thread must have exited");
        assert!(dropped.load(Ordering::Acquire), "confirmed disarmed driver must drop cleanly");
        let shutdown_logs = take_logs();
        assert!(
            !shutdown_logs.contains("Shutdown disarm proof refused")
                && !shutdown_logs.contains("Shutdown disarm proof unconfirmed"),
            "clean cold-boot shutdown must emit no disarm-proof refusal warnings: {shutdown_logs:?}"
        );

        let log = bus.tx_log();
        assert!(
            log.iter().any(is_refresh_frame),
            "expected read-only 0xCC status polls on wire"
        );
        assert!(
            log.iter().all(|f| f.data != damiao::CMD_ENABLE),
            "must NEVER send CMD_ENABLE during shutdown disarm proof"
        );
        assert!(
            log.iter().all(|f| f.data != damiao::CMD_DISABLE),
            "cold-boot disarmed shutdown must not need CMD_DISABLE"
        );
    }

    // (b) Cold boot, never enabled, direct `stop_and_join()` (ordinary teardown) with NO prior
    // feedback -> actively polls 0xCC and exits cleanly without entering SAFE-HOLD.
    {
        let bus = FakeBus::new(&dm_limits());
        bus.with(|s| {
            s.immediate_replies = true;
            s.tx_log.clear();
        });
        let (sup, dropped) = build_watched_damiao(bus.clone(), false);
        sup.set_lease_enabled(false).await;
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup.start_executor(cfg).await.expect("start_executor");

        let t0 = Instant::now();
        let h_stop = handle.clone();
        tokio::task::spawn_blocking(move || h_stop.stop_and_join())
            .await
            .unwrap();
        let elapsed = t0.elapsed();
        assert!(
            elapsed < Duration::from_secs(1),
            "stop_and_join on cold-boot disarmed arm must finish in < 1 s (took {elapsed:?})"
        );
        assert!(
            !sup.e_stop_latched.load(Ordering::Acquire),
            "cold-boot stop_and_join must NOT latch E-stop"
        );
        assert!(!handle.is_in_safe_hold(), "cold-boot stop_and_join must NOT enter SAFE-HOLD");
        assert!(handle.is_exited());
        assert!(dropped.load(Ordering::Acquire));

        let log = bus.tx_log();
        assert!(log.iter().any(is_refresh_frame), "expected 0xCC polls during stop_and_join");
        assert!(log.iter().all(|f| f.data != damiao::CMD_ENABLE));
        assert!(log.iter().all(|f| f.data != damiao::CMD_DISABLE));
    }

    // (c) Cold boot, never enabled, Legacy owner shares the exact same active 0xCC disarm proof.
    {
        let bus = FakeBus::new(&dm_limits());
        bus.with(|s| {
            s.immediate_replies = true;
            s.tx_log.clear();
        });
        let (sup, _dropped) = build_watched_damiao(bus.clone(), false);
        sup.set_lease_enabled(false).await;
        let t0 = Instant::now();
        let deadline = t0 + armd::supervisor::PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
        assert!(
            sup.request_shutdown_until(deadline).await.unwrap(),
            "legacy cold-boot never-enabled arm must confirm shutdown via 0xCC proof"
        );
        assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);
        assert!(sup.inner().write().await.commit_shutdown_until(deadline));
        assert!(t0.elapsed() < Duration::from_secs(1));
        let log = bus.tx_log();
        assert!(log.iter().any(is_refresh_frame));
        assert!(log.iter().all(|f| f.data != damiao::CMD_ENABLE));
        assert!(log.iter().all(|f| f.data != damiao::CMD_DISABLE));
    }
}

fn submit_broker_transition(handle: &RtExecutorHandle, seq: u64, op: TransitionOp) -> RequestId {
    let snap = handle.planning_snapshot().unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut b = handle.broker().lock().unwrap();
    b.submit(ClientId(1), seq, snap.authority, deadline, Submit::Transition(op))
        .unwrap()
}

fn wait_broker_terminal(handle: &RtExecutorHandle, req: RequestId, timeout: Duration) -> Outcome {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let mut found = None;
        handle.broker().lock().unwrap().poll(|_, r| {
            if let Reply::Terminal {
                request, outcome, ..
            } = r
            {
                if request == req {
                    found = Some(outcome);
                }
            }
            true
        });
        if let Some(o) = found {
            return o;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("broker request {req:?} did not reach terminal within {timeout:?}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_enabled_at_rest_then_disabled_sigterm_clean() {
    let _ = log_buf();

    // (a) Live executor: enable at rest -> disable at rest -> SIGTERM shutdown cleanly in < 1 s.
    {
        let bus = FakeBus::new(&dm_limits());
        bus.with(|s| {
            s.immediate_replies = true;
            s.follow_commands = true;
        });
        let (sup, dropped) = build_watched_damiao(bus.clone(), true);
        sup.set_lease_enabled(false).await;
        sup.register_ws_connection(1).await.unwrap();
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup.start_executor(cfg).await.expect("start_executor");
        wait_until(Duration::from_secs(1), "initial executor releases", || {
            handle.stats().completed_releases.load(Ordering::Acquire) >= 2
        })
        .await;

        let req_on = submit_broker_transition(&handle, 1, TransitionOp::TorqueOn);
        assert_eq!(
            wait_broker_terminal(&handle, req_on, Duration::from_secs(2)),
            Outcome::Completed
        );
        assert!(sup.get_observation().await.is_torqued);

        // Let K=3 rest-pose feedback settle at 0.0 rad, then disable torque through the executor.
        wait_until(Duration::from_secs(1), "rest-pose settle releases", || {
            handle.stats().completed_releases.load(Ordering::Acquire) >= 10
        })
        .await;
        let req_off = submit_broker_transition(&handle, 2, TransitionOp::TorqueOff);
        assert_eq!(
            wait_broker_terminal(&handle, req_off, Duration::from_secs(2)),
            Outcome::Completed
        );
        assert!(!sup.get_observation().await.is_torqued);
        assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);

        bus.clear_log();
        let t0 = Instant::now();
        let deadline = t0 + armd::supervisor::PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
        assert!(sup.request_shutdown_until(deadline).await.unwrap());
        let h_join = handle.clone();
        tokio::task::spawn_blocking(move || h_join.shutdown_and_join_until(deadline))
            .await
            .unwrap()
            .expect("shutdown_and_join_until after rest-pose disable must succeed");
        assert!(t0.elapsed() < Duration::from_secs(1));
        assert!(!handle.is_in_safe_hold());
        assert!(handle.is_exited());
        assert!(dropped.load(Ordering::Acquire));
        assert!(bus.tx_log().iter().all(|f| f.data != damiao::CMD_ENABLE));
    }

    // (b) Stale feedback after enable -> disable (VirtualBus / MockClock advanced > 300 ms):
    // `commit_shutdown_until` actively refreshes causal `0xCC` proof and confirms cleanly.
    {
        let clock = MockClock::new();
        let bus = FakeBus::with_clock(&dm_limits(), clock.clone());
        bus.with(|s| {
            s.immediate_replies = true;
            s.push_all_feedback();
        });
        let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
            .with_joint_limits(dm_limits())
            .unwrap();
        let _ = drv.read_joint_states().unwrap();
        drv.check_mapping_ranges();
        let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
        sup.set_lease_enabled(false).await;
        sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();
        for _ in 0..5 {
            clock.advance(Duration::from_millis(15));
            bus.with(|s| s.push_all_feedback());
            sup.tick(0.02).await;
        }
        sup.disable_torque().await.unwrap();
        for _ in 0..10 {
            if sup.shutdown_ready().await {
                break;
            }
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            sup.tick(0.02).await;
        }
        assert!(sup.shutdown_ready().await);

        // Age feedback past the 250 ms validity window so passive proof is missing.
        clock.advance(Duration::from_millis(350));
        bus.clear_log();
        assert!(sup.request_shutdown().await.unwrap());
        let committed = sup
            .inner()
            .write()
            .await
            .commit_shutdown_until(Instant::now() + Duration::from_millis(300));
        assert!(
            committed,
            "commit_shutdown_until must refresh stale post-disable proof via 0xCC and succeed"
        );
        let log = bus.tx_log();
        assert!(log.iter().any(is_refresh_frame));
        assert!(log.iter().all(|f| f.data != damiao::CMD_ENABLE));
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_enabled_status_polls_retain_driver_and_refuse_shutdown() {
    let _ = log_buf();

    // (a) Cold boot, never enabled by armd, BUT motor J2 replies state=1 (physically enabled):
    // - `request_shutdown_until` refuses (returns false), latches EStop, logs J2 enabled,
    //   and sends ZERO CMD_ENABLE and ZERO CMD_DISABLE.
    // - Repeated `request_shutdown_until` STILL refuses (never drops a known-enabled motor).
    // - `stop_and_join` retains the driver in SAFE-HOLD (`!dropped`).
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.immediate_replies = true;
        s.motors[1].enabled = true; // J2 physically enabled before armd boot
        s.tx_log.clear();
    });
    let (sup, dropped) = build_watched_damiao(bus.clone(), false);
    sup.set_lease_enabled(false).await;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    cfg.panic_policy = PanicPolicy::StopExecutor;
    let handle = sup.start_executor(cfg).await.expect("start_executor");
    wait_until(Duration::from_secs(1), "initial release", || {
        handle.stats().completed_releases.load(Ordering::Acquire) >= 1
    })
    .await;

    let _ = take_logs();
    let deadline = Instant::now() + armd::supervisor::PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
    assert!(
        !sup.request_shutdown_until(deadline).await.unwrap(),
        "first SIGTERM must refuse when J2 status poll shows enabled"
    );
    let logs = take_logs();
    assert!(
        logs.contains("J2") && logs.contains("enabled"),
        "expected refusal log naming enabled axis J2, got: {logs:?}"
    );
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::EStop);
    assert!(
        !sup.request_shutdown_until(Instant::now() + Duration::from_millis(400))
            .await
            .unwrap(),
        "repeated SIGTERM must still refuse when J2 is known enabled"
    );
    {
        let log = bus.tx_log();
        assert!(log.iter().all(|f| f.data != damiao::CMD_ENABLE));
        assert!(log.iter().all(|f| f.data != damiao::CMD_DISABLE));
    }

    // Ordinary `stop_and_join` must retain the driver in SAFE-HOLD and NEVER drop it.
    let h_stop = handle.clone();
    tokio::task::spawn_blocking(move || h_stop.stop_and_join())
        .await
        .unwrap();
    assert!(
        handle.is_in_safe_hold(),
        "enabled axis on stop_and_join must retain driver in SAFE-HOLD"
    );
    assert!(
        !dropped.load(Ordering::Acquire),
        "energized driver must NOT be dropped on stop_and_join"
    );

    // Clean up test thread via deliberate ProcessExit so background thread does not outlive test.
    let outcome = sup
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_millis(500))
        .await;
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    wait_until(Duration::from_secs(1), "driver dropped after ProcessExit", || {
        dropped.load(Ordering::Acquire)
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_silent_axis_retained_with_axis_named() {
    let _ = log_buf();

    // Cold boot, never enabled, motor J4 is silent (never answers 0xCC):
    // - `request_shutdown_until` refuses within bounded window, latches EStop, and names J4.
    // - `stop_and_join` retains the driver in SAFE-HOLD (`!dropped`) and names J4.
    let bus = FakeBus::new(&dm_limits());
    bus.with(|s| {
        s.immediate_replies = true;
        s.motors[3].silent = true; // J4 silent
        s.tx_log.clear();
    });
    let (sup, dropped) = build_watched_damiao(bus.clone(), false);
    sup.set_lease_enabled(false).await;
    let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
    cfg.panic_policy = PanicPolicy::StopExecutor;
    let handle = sup.start_executor(cfg).await.expect("start_executor");
    wait_until(Duration::from_secs(1), "initial release", || {
        handle.stats().completed_releases.load(Ordering::Acquire) >= 1
    })
    .await;

    let _ = take_logs();
    let t0 = Instant::now();
    let deadline = t0 + armd::supervisor::PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
    assert!(
        !sup.request_shutdown_until(deadline).await.unwrap(),
        "silent axis J4 must cause shutdown refusal"
    );
    let elapsed = t0.elapsed();
    assert!(
        elapsed < Duration::from_millis(550),
        "silent-axis probe must be bounded (took {elapsed:?})"
    );
    let logs = take_logs();
    assert!(
        logs.contains("J4") && logs.contains("0x4"),
        "expected refusal log naming silent axis J4 (CAN ID 0x4), got: {logs:?}"
    );
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::EStop);
    {
        let log = bus.tx_log();
        assert!(log.iter().any(is_refresh_frame), "expected 0xCC polls");
        assert!(log.iter().all(|f| f.data != damiao::CMD_ENABLE));
        assert!(log.iter().all(|f| f.data != damiao::CMD_DISABLE));
    }

    // Ordinary `stop_and_join` with silent J4 must retain driver in SAFE-HOLD and log J4.
    let _ = take_logs();
    let h_stop = handle.clone();
    tokio::task::spawn_blocking(move || h_stop.stop_and_join())
        .await
        .unwrap();
    assert!(
        handle.is_in_safe_hold(),
        "silent axis on stop_and_join must retain driver in SAFE-HOLD"
    );
    assert!(
        !dropped.load(Ordering::Acquire),
        "driver with silent axis must NOT be dropped on stop_and_join"
    );
    let stop_logs = take_logs();
    assert!(
        stop_logs.contains("J4") && stop_logs.contains("0x4"),
        "expected stop_and_join log naming silent axis J4 (CAN ID 0x4), got: {stop_logs:?}"
    );

    // Unmute J4 and exit cleanly via ProcessExit so the SAFE-HOLD thread terminates.
    bus.with(|s| s.motors[3].silent = false);
    let outcome = sup
        .emergency_process_exit_disable_until(Instant::now() + Duration::from_millis(500))
        .await;
    assert_eq!(outcome, PanicDisableOutcome::Confirmed);
    wait_until(Duration::from_secs(1), "driver dropped after unmute + ProcessExit", || {
        dropped.load(Ordering::Acquire)
    })
    .await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn damiao_fakebus_executor_3_cold_starts_sigterm_zero_faults() {
    let _ = log_buf();

    for cold_start in 1..=3 {
        let bus = FakeBus::new(&dm_limits());
        bus.with(|s| {
            s.immediate_replies = true;
            s.tx_log.clear();
        });
        let (sup, dropped) = build_watched_damiao(bus.clone(), false);
        sup.set_lease_enabled(false).await;
        let mut cfg = ExecutorConfig::for_tests(Duration::from_millis(10));
        cfg.panic_policy = PanicPolicy::StopExecutor;
        let handle = sup
            .start_executor(cfg)
            .await
            .unwrap_or_else(|e| panic!("cold start {cold_start}: start_executor failed: {e}"));
        wait_until(Duration::from_secs(1), "cold start releases", || {
            handle.stats().completed_releases.load(Ordering::Acquire) >= 3
        })
        .await;

        let obs0 = sup.get_observation().await;
        assert_eq!(obs0.safety_state, SafetyState::Idle, "cold start {cold_start}: expected Idle");
        assert!(!obs0.is_torqued, "cold start {cold_start}: expected !is_torqued");
        assert_eq!(sup.inner().control_thread_attempts(), 0, "cold start {cold_start}: 0 lock attempts");

        let _ = take_logs();
        let t0 = Instant::now();
        let deadline = t0 + armd::supervisor::PANIC_DISABLE_TIMEOUT + Duration::from_millis(100);
        assert!(
            sup.request_shutdown_until(deadline).await.unwrap(),
            "cold start {cold_start}: SIGTERM request_shutdown must confirm"
        );
        let h_join = handle.clone();
        tokio::task::spawn_blocking(move || h_join.shutdown_and_join_until(deadline))
            .await
            .unwrap()
            .unwrap_or_else(|e| panic!("cold start {cold_start}: shutdown_and_join failed: {e}"));
        assert!(t0.elapsed() < Duration::from_secs(1), "cold start {cold_start}: < 1 s");
        assert!(!handle.is_in_safe_hold(), "cold start {cold_start}: no SAFE-HOLD");
        assert!(handle.is_exited(), "cold start {cold_start}: exited");
        assert!(dropped.load(Ordering::Acquire), "cold start {cold_start}: driver dropped");
        let logs = take_logs();
        assert!(
            !logs.contains("Shutdown disarm proof refused")
                && !logs.contains("Shutdown disarm proof unconfirmed"),
            "cold start {cold_start}: unexpected disarm warning: {logs:?}"
        );
        assert!(bus.tx_log().iter().all(|f| f.data != damiao::CMD_ENABLE));
    }
}
