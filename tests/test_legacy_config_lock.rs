//! Lock-hold and freshness tests for Legacy `ArmSupervisor::published_config()`.
//!
//! Verifies that `published_config()` never holds `SupervisorLock::read()` across
//! a contended legacy `DriverLock` mutex or snapshot finalization, preserves
//! poisoned-driver fallbacks, and never overwrites a newer snapshot or publishes
//! into the executor's epoch.

use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, GripperPosForceInfo, JointModeExt, JointTarget,
    ModeSwitchExt, MotorCurrentCap, PosForceExt,
};
use armd::model::{ArmConfig, JointState, MotorZeroSupport};
use armd::rt_core::executor::ExecutorConfig;
use armd::rt_core::requests::config::PublishedConfig;
use armd::supervisor::{ArmError, ArmSupervisor, OWNER_EXECUTOR, OWNER_LEGACY};
use armd::supervisor_lock::HoldKind;

/// Minimal mock driver implementing capability helpers exercised by `PublishedConfig`.
struct ConfigLockTestDriver {
    positions: Vec<f32>,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
    motor_current_raw: u16,
    close_speed_rad_s: f32,
}

impl ConfigLockTestDriver {
    fn new(dof: usize, motor_current_raw: u16, close_speed_rad_s: f32) -> Self {
        let now = Instant::now();
        Self {
            positions: vec![0.0; dof],
            rx_at: vec![Some(now); dof],
            seqs: vec![1; dof],
            motor_current_raw,
            close_speed_rad_s,
        }
    }
}

impl ArmDriver for ConfigLockTestDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }

    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::set_current("motor_flash", false, vec![])
    }

    fn is_feedback_valid(&self) -> bool {
        true
    }

    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        self.rx_at.len() == dof
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        out.clear();
        out.extend(std::iter::repeat_n(Some(Duration::ZERO), self.rx_at.len()));
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        out.clear();
        out.extend(std::iter::repeat_n(Some(0), self.rx_at.len()));
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
        out.extend_from_slice(&self.seqs);
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
        out.extend_from_slice(&self.rx_at);
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let now = Instant::now();
        for t in &mut self.rx_at {
            *t = Some(now);
        }
        out.clear();
        for &p in &self.positions {
            out.push(JointState {
                position: p,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
            });
        }
        Ok(())
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.positions.len());
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, _enabled: bool) -> Result<(), ArmDriverError> {
        Ok(())
    }

    armd::ext_accessors!(mode_switch, joint_mode, pos_force);
}

impl ModeSwitchExt for ConfigLockTestDriver {
    fn supports_nonblocking_mode_switch(&self) -> bool {
        true
    }
}

impl JointModeExt for ConfigLockTestDriver {
    fn supports_mit_mode_switch(&self) -> bool {
        true
    }
}

impl PosForceExt for ConfigLockTestDriver {
    fn motor_current_cap(&self) -> Option<MotorCurrentCap> {
        Some(MotorCurrentCap {
            joint: 6,
            mode: "pos_force",
            current_raw: Some(self.motor_current_raw),
            active: true,
            calibrated: false,
            ceiling_torque_nm: Some(0.8),
            torque_reference: Some("bench".into()),
            close_speed_rad_s: Some(self.close_speed_rad_s),
            note: "test motor current cap",
        })
    }

    fn gripper_pos_force_info(&self) -> Option<GripperPosForceInfo> {
        Some(GripperPosForceInfo {
            joint: 6,
            active: true,
            close_speed_rad_s: self.close_speed_rad_s,
            ceiling_torque_estimate_nm: Some(0.8),
        })
    }
}

/// Holds the legacy `DriverLock` mutex on a dedicated OS thread without holding `SupervisorLock`,
/// and guarantees release + thread join on drop (even on test panic) while keeping `_sup` alive.
struct ContendedDriverGuard {
    release_tx: Option<mpsc::SyncSender<()>>,
    holder: Option<std::thread::JoinHandle<()>>,
    _sup: Arc<ArmSupervisor>,
}

impl ContendedDriverGuard {
    async fn acquire(sup: &Arc<ArmSupervisor>) -> Self {
        let addr = {
            let inner = sup.inner().read().await;
            let lock = inner
                .driver
                .as_ref()
                .expect("driver installed")
                .as_legacy_lock()
                .expect("legacy driver lock");
            (lock.raw_mutex() as *const std::sync::Mutex<Box<dyn ArmDriver>>) as usize
        };
        let (locked_tx, locked_rx) = mpsc::sync_channel::<()>(1);
        let (release_tx, release_rx) = mpsc::sync_channel::<()>(1);
        let holder = std::thread::spawn(move || {
            // SAFETY: `_sup` keeps `SupervisorInner` and its legacy `DriverSlot` alive and
            // unmoved for the lifetime of this guard, and `Drop` joins this thread before
            // dropping `_sup`. Tests using this guard never replace or detach `inner.driver`.
            let mutex = unsafe { &*(addr as *const std::sync::Mutex<Box<dyn ArmDriver>>) };
            let _guard = mutex.lock().expect("driver mutex not poisoned");
            let _ = locked_tx.send(());
            let _ = release_rx.recv_timeout(Duration::from_secs(5));
        });
        locked_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("helper thread acquired driver mutex");
        Self {
            release_tx: Some(release_tx),
            holder: Some(holder),
            _sup: Arc::clone(sup),
        }
    }

    fn release(mut self) {
        self.release_inner();
    }

    fn release_inner(&mut self) {
        if let Some(tx) = self.release_tx.take() {
            let _ = tx.send(());
        }
        if let Some(jh) = self.holder.take() {
            let _ = jh.join();
        }
    }
}

impl Drop for ContendedDriverGuard {
    fn drop(&mut self) {
        self.release_inner();
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_published_config_releases_supervisor_lock_while_driver_mutex_is_held() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.profile_id = Some("initial-profile".into());
    let sup = Arc::new(
        ArmSupervisor::new(cfg).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    // Warm the published cache, then force a generation miss.
    let warm = sup.published_config().await.expect("warm published_config");
    assert_eq!(warm.config.profile_id.as_deref(), Some("initial-profile"));
    {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 10;
        inner.config.profile_id = Some("stale-before-contention".into());
    }

    // Hold the legacy driver mutex on a helper OS thread (NOT holding SupervisorLock).
    let contended = ContendedDriverGuard::acquire(&sup).await;

    // Record every individual SupervisorLock guard hold across the contended refresh.
    sup.inner().set_hold_history_enabled(true);

    let reader = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move { sup.published_config().await }
    });

    // Give the reader task time to enter `published_config()`, miss the generation cache,
    // and attempt driver capability capture against the contended driver mutex.
    tokio::time::sleep(Duration::from_millis(30)).await;

    // Bounded handshake: a concurrent supervisor writer MUST acquire `SupervisorLock::write()`
    // while the driver mutex is still held. On base, `published_config()` blocks inside
    // `DriverLock::lock()` while holding `SupervisorLock::read()`, starving this writer.
    let writer_acquired = tokio::time::timeout(Duration::from_millis(200), sup.inner().write())
        .await
        .expect("concurrent supervisor writer must acquire write lock while driver mutex is held");
    {
        let mut inner = writer_acquired;
        inner.workspace_revision = 42;
        inner.config.profile_id = Some("writer-during-contention".into());
        inner.operator_auth_enforced = true;
    }

    // Keep the mutex held briefly longer so the reader cannot finish before release, then release.
    tokio::time::sleep(Duration::from_millis(20)).await;
    assert!(
        !reader.is_finished(),
        "reader must wait for driver mutex release rather than returning fabricated capabilities"
    );
    contended.release();

    let snap = tokio::time::timeout(Duration::from_secs(2), reader)
        .await
        .expect("published_config must complete after driver mutex release")
        .expect("reader task did not panic")
        .expect("published_config returned Ok");

    let holds = sup.inner().take_hold_history();
    sup.inner().set_hold_history_enabled(false);

    // Retry must return the writer's revision and matching payload.
    assert_eq!(snap.workspace_revision, 42);
    assert_eq!(
        snap.config.profile_id.as_deref(),
        Some("writer-during-contention")
    );
    assert!(snap.operator_auth_enforced);
    assert!(snap.has_driver);
    assert!(snap.driver_supports_nonblocking_mode_switch);
    assert!(snap.driver_torque_feedback);
    assert!(snap.driver_temperature_feedback);
    assert!(snap.driver_motor_zero_support.supported);
    assert_eq!(
        snap.driver_motor_current_cap
            .as_ref()
            .and_then(|v| v["current_raw"].as_u64()),
        Some(420),
    );
    assert!(
        (snap.effective_planned_close_speed - 0.25).abs() < 1e-6,
        "expected quantized close speed 0.25, got {}",
        snap.effective_planned_close_speed,
    );

    // Per-guard hold recording: no unpreempted SupervisorLock read hold may span the contention window.
    let read_holds: Vec<_> = holds.iter().filter(|h| h.kind == HoldKind::Read).collect();
    assert!(
        !read_holds.is_empty(),
        "expected at least one recorded SupervisorLock read hold"
    );
    let max_read_hold = read_holds
        .iter()
        .map(|h| h.hold())
        .max()
        .unwrap_or(Duration::ZERO);
    println!(
        "[legacy_config_lock] read_holds={} max_read_hold_us={}",
        read_holds.len(),
        max_read_hold.as_micros(),
    );
    for h in &read_holds {
        if h.involuntary_csw == 0 {
            assert!(
                h.hold() <= Duration::from_millis(5),
                "unpreempted SupervisorLock read hold must stay <= 5 ms (got {:?}, hold={h:?})",
                h.hold(),
            );
        }
    }
}

fn assert_published_config_fields_match(actual: &PublishedConfig, expected: &PublishedConfig) {
    assert_eq!(actual.workspace_revision, expected.workspace_revision);
    assert_eq!(actual.config.profile_id, expected.config.profile_id);
    assert_eq!(actual.operator_auth_enforced, expected.operator_auth_enforced);
    assert_eq!(actual.has_driver, expected.has_driver);
    assert_eq!(
        actual.driver_supports_nonblocking_mode_switch,
        expected.driver_supports_nonblocking_mode_switch
    );
    assert_eq!(actual.driver_torque_feedback, expected.driver_torque_feedback);
    assert_eq!(
        actual.driver_temperature_feedback,
        expected.driver_temperature_feedback
    );
    assert_eq!(
        actual.driver_supports_automatic_unload,
        expected.driver_supports_automatic_unload
    );
    assert_eq!(
        actual.driver_motor_current_cap,
        expected.driver_motor_current_cap
    );
    assert_eq!(
        actual.driver_motor_zero_support,
        expected.driver_motor_zero_support
    );
    assert_eq!(actual.gravity_compensated, expected.gravity_compensated);
    assert_eq!(actual.gravity_comp_supported, expected.gravity_comp_supported);
    assert_eq!(actual.torque_guard_thresholds, expected.torque_guard_thresholds);
    assert!(
        (actual.effective_planned_close_speed - expected.effective_planned_close_speed).abs()
            < 1e-9,
        "effective_planned_close_speed mismatch: actual={} expected={}",
        actual.effective_planned_close_speed,
        expected.effective_planned_close_speed,
    );
    assert_eq!(actual.policy_live_allowed, expected.policy_live_allowed);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_older_paused_capture_cannot_overwrite_newer_snapshot() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.profile_id = Some("rev-0".into());
    let sup = Arc::new(
        ArmSupervisor::new(cfg).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    // Force a miss at rev-10.
    let gen_old = {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 10;
        inner.config.profile_id = Some("rev-10".into());
        sup.inner().generation()
    };

    let (captured_tx, captured_rx) = tokio::sync::oneshot::channel::<u64>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut captured_tx = Some(captured_tx);
    let mut resume_rx = Some(resume_rx);

    let older_reader = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.published_config_with_after_capture_hook(|gen| {
                let tx = captured_tx.take();
                let rx = resume_rx.take();
                async move {
                    if let Some(tx) = tx {
                        let _ = tx.send(gen);
                    }
                    if let Some(rx) = rx {
                        let _ = rx.await;
                    }
                }
            })
            .await
        }
    });

    // Wait until older_reader has completed capture and dropped SupervisorLock::read().
    let captured_gen = tokio::time::timeout(Duration::from_secs(2), captured_rx)
        .await
        .expect("older reader reached after_capture hook")
        .expect("captured_tx not dropped");
    assert_eq!(captured_gen, gen_old);

    // Advance state under a write guard and publish the newer snapshot while older_reader is paused.
    let gen_new = {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 20;
        inner.config.profile_id = Some("rev-20".into());
        inner.operator_auth_enforced = true;
        sup.inner().generation()
    };
    assert!(gen_new > gen_old);

    let newer = sup.published_config().await.expect("newer publish");
    assert_eq!(newer.workspace_revision, 20);
    assert_eq!(newer.config.profile_id.as_deref(), Some("rev-20"));
    let (hub_before, hub_gen_before) = sup.published_config_hub().latest_with_gen();
    assert_eq!(hub_gen_before, gen_new);
    assert_eq!(hub_before.workspace_revision, 20);

    // Release older_reader to finalize and attempt publication of its stale rev-10 capture.
    resume_tx.send(()).expect("resume older reader");
    let older_returned = tokio::time::timeout(Duration::from_secs(2), older_reader)
        .await
        .expect("older reader completed")
        .expect("older reader did not panic")
        .expect("older reader returned Ok");

    // Hub and caller must both preserve the newer rev-20 snapshot.
    let (hub_after, hub_gen_after) = sup.published_config_hub().latest_with_gen();
    assert_eq!(hub_gen_after, gen_new);
    assert_eq!(hub_after.workspace_revision, 20);
    assert_eq!(hub_after.config.profile_id.as_deref(), Some("rev-20"));
    assert!(hub_after.operator_auth_enforced);
    assert_eq!(older_returned.workspace_revision, 20);
    assert_eq!(older_returned.config.profile_id.as_deref(), Some("rev-20"));
    assert!(older_returned.operator_auth_enforced);

    // Subsequent cache-hit read still returns rev-20.
    let final_snap = sup.published_config().await.expect("final read");
    assert_eq!(final_snap.workspace_revision, 20);
    assert_eq!(final_snap.config.profile_id.as_deref(), Some("rev-20"));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_delayed_finalizer_cannot_publish_into_executor_epoch() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.profile_id = Some("legacy-initial".into());
    let sup = Arc::new(
        ArmSupervisor::new(cfg).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    // Force a legacy cache miss at rev-11.
    let legacy_gen = {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 11;
        inner.config.profile_id = Some("legacy-before-handoff".into());
        sup.inner().generation()
    };

    let (captured_tx, captured_rx) = tokio::sync::oneshot::channel::<u64>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut captured_tx = Some(captured_tx);
    let mut resume_rx = Some(resume_rx);

    let delayed_legacy = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.published_config_with_after_capture_hook(|gen| {
                let tx = captured_tx.take();
                let rx = resume_rx.take();
                async move {
                    if let Some(tx) = tx {
                        let _ = tx.send(gen);
                    }
                    if let Some(rx) = rx {
                        let _ = rx.await;
                    }
                }
            })
            .await
        }
    });

    let captured_gen = tokio::time::timeout(Duration::from_secs(2), captured_rx)
        .await
        .expect("delayed legacy reader reached after_capture")
        .expect("captured_tx not dropped");
    assert_eq!(captured_gen, legacy_gen);

    // Transfer ownership to the RT executor while delayed_legacy is paused after capture.
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("start_executor handoff");
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

    // Advance the executor's published config (note: `update` keeps `legacy_lock_gen` unchanged).
    sup.published_config_hub().update(|cfg| {
        cfg.workspace_revision = 99;
        cfg.config.profile_id = Some("executor-epoch".into());
    });

    // Even a hypothetical Legacy publish with `u64::MAX` generation must be rejected once
    // ownership is no longer `OWNER_LEGACY`.
    let mut forged_legacy = (*sup.published_config_hub().latest().expect("executor latest")).clone();
    forged_legacy.workspace_revision = 777;
    forged_legacy.config.profile_id = Some("forged-legacy".into());
    let rejected = sup.published_config_hub().publish_legacy_arc_if_newer(
        Arc::new(forged_legacy),
        u64::MAX,
        || sup.owner_mode() == OWNER_LEGACY,
    );
    assert!(
        rejected.is_none(),
        "publish_legacy_arc_if_newer must refuse publication outside OWNER_LEGACY"
    );

    // Release the delayed legacy finalizer and verify it neither replaces the executor's
    // PublishedConfigHub entry nor returns stale legacy data.
    resume_tx.send(()).expect("resume delayed legacy reader");
    let returned = tokio::time::timeout(Duration::from_secs(2), delayed_legacy)
        .await
        .expect("delayed legacy reader finished")
        .expect("delayed legacy reader did not panic")
        .expect("returns executor snapshot after handoff");

    assert_eq!(returned.workspace_revision, 99);
    assert_eq!(returned.config.profile_id.as_deref(), Some("executor-epoch"));

    let hub_latest = sup
        .published_config_hub()
        .latest()
        .expect("executor hub still available");
    assert_eq!(hub_latest.workspace_revision, 99);
    assert_eq!(
        hub_latest.config.profile_id.as_deref(),
        Some("executor-epoch")
    );

    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .expect("executor thread joined cleanly");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_delayed_finalizer_fails_closed_after_owner_failed() {
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 7;
    }

    let (captured_tx, captured_rx) = tokio::sync::oneshot::channel::<u64>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut captured_tx = Some(captured_tx);
    let mut resume_rx = Some(resume_rx);

    let delayed_legacy = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.published_config_with_after_capture_hook(|gen| {
                let tx = captured_tx.take();
                let rx = resume_rx.take();
                async move {
                    if let Some(tx) = tx {
                        let _ = tx.send(gen);
                    }
                    if let Some(rx) = rx {
                        let _ = rx.await;
                    }
                }
            })
            .await
        }
    });

    tokio::time::timeout(Duration::from_secs(2), captured_rx)
        .await
        .expect("reader captured before failure")
        .expect("captured_tx not dropped");

    sup.force_owner_failed_for_mock_test()
        .await
        .expect("transition to OWNER_FAILED");
    assert!(sup.published_config_hub().latest().is_none());

    resume_tx.send(()).expect("resume delayed reader");
    let res = tokio::time::timeout(Duration::from_secs(2), delayed_legacy)
        .await
        .expect("delayed reader completed")
        .expect("delayed reader did not panic");

    match res {
        Err(ArmError::Rejected { status, code, .. }) => {
            assert_eq!(status, 503);
            assert_eq!(code, "CONFIG_UNAVAILABLE");
        }
        Err(other) => panic!("expected 503 CONFIG_UNAVAILABLE after OWNER_FAILED, got Err({other:?})"),
        Ok(cfg) => panic!(
            "expected 503 CONFIG_UNAVAILABLE after OWNER_FAILED, got Ok(workspace_revision={})",
            cfg.workspace_revision
        ),
    }
    assert!(
        sup.published_config_hub().latest().is_none(),
        "delayed legacy finalizer must not re-enable or overwrite unavailable hub"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_published_config_preserves_poisoned_driver_fallbacks() {
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm())
            .with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    // Poison the legacy driver mutex on a separate thread without leaving SupervisorLock held.
    let addr = {
        let inner = sup.inner().read().await;
        let lock = inner
            .driver
            .as_ref()
            .expect("driver installed")
            .as_legacy_lock()
            .expect("legacy driver lock");
        (lock.raw_mutex() as *const std::sync::Mutex<Box<dyn ArmDriver>>) as usize
    };
    let poison_jh = std::thread::spawn(move || {
        // SAFETY: `sup` stays alive and unmoved until `poison_jh.join()` completes below.
        let mutex = unsafe { &*(addr as *const std::sync::Mutex<Box<dyn ArmDriver>>) };
        let _guard = mutex.lock().expect("acquire mutex before poison");
        panic!("intentional poison for fallback test");
    });
    assert!(poison_jh.join().is_err(), "helper thread must panic to poison mutex");

    // Force a cache miss so published_config() takes the nonblocking capture path.
    {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 55;
    }

    let snap = tokio::time::timeout(Duration::from_secs(1), sup.published_config())
        .await
        .expect("poisoned driver must not spin or hang published_config()")
        .expect("poisoned driver falls back cleanly");

    assert_eq!(snap.workspace_revision, 55);
    assert!(snap.has_driver);
    assert!(!snap.driver_supports_nonblocking_mode_switch);
    assert!(!snap.driver_torque_feedback);
    assert!(!snap.driver_temperature_feedback);
    assert!(snap.driver_motor_current_cap.is_none());
    assert!(!snap.driver_motor_zero_support.supported);
    assert_eq!(
        snap.driver_motor_zero_support.reason.as_deref(),
        Some("driver unavailable")
    );
    assert!(!snap.gravity_comp_supported);

    // Verify exact field-by-field parity with `PublishedConfig::from_inner`.
    let expected = {
        let inner = sup.inner().read().await;
        PublishedConfig::from_inner(&inner)
    };
    assert_published_config_fields_match(&snap, &expected);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_published_config_matches_from_inner_for_live_driver_and_mock() {
    // Live driver parity.
    let sup_hw = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm())
            .with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );
    {
        let mut inner = sup_hw.inner().write().await;
        inner.workspace_revision = 3;
        inner.operator_auth_enforced = true;
    }
    // Regression guard: `published_config()`'s async state machine must stay small so deep
    // tokio worker stacks (e.g. hardware motor_zero / calibration handlers) never overflow.
    {
        let fut = sup_hw.published_config();
        let fut_bytes = std::mem::size_of_val(&fut);
        assert!(
            fut_bytes <= 512,
            "published_config() future must stay <= 512 bytes (got {fut_bytes} bytes)"
        );
    }

    let hw_snap = sup_hw.published_config().await.expect("hw published_config");
    let hw_expected = {
        let inner = sup_hw.inner().read().await;
        PublishedConfig::from_inner(&inner)
    };
    assert_published_config_fields_match(&hw_snap, &hw_expected);

    // Driverless mock follower parity (temperature_feedback == true, automatic_unload == true).
    let sup_mock = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    {
        let mut inner = sup_mock.inner().write().await;
        inner.workspace_revision = 4;
    }
    let mock_snap = sup_mock
        .published_config()
        .await
        .expect("mock published_config");
    assert!(!mock_snap.has_driver);
    assert!(mock_snap.driver_temperature_feedback);
    assert!(mock_snap.driver_supports_automatic_unload);
    assert!(mock_snap.driver_motor_zero_support.supported);
    let mock_expected = {
        let inner = sup_mock.inner().read().await;
        PublishedConfig::from_inner(&inner)
    };
    assert_published_config_fields_match(&mock_snap, &mock_expected);
}
