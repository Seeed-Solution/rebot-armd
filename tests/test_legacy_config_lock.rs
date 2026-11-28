//! Lock-hold, bounded-retry, handoff-race, and freshness tests for Legacy
//! `ArmSupervisor::published_config()` and `ArmSupervisor::set_operator_auth_enforced()`.

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
use armd::sequences::RunPins;
use armd::supervisor::{ArmError, ArmSupervisor, OWNER_EXECUTOR, OWNER_LEGACY};
use armd::supervisor_lock::HoldKind;

/// Minimal configurable mock driver implementing capability helpers exercised by `PublishedConfig`.
struct ConfigLockTestDriver {
    positions: Vec<f32>,
    rx_at: Vec<Option<Instant>>,
    seqs: Vec<u64>,
    motor_current_raw: u16,
    close_speed_rad_s: f32,
    torque_feedback: bool,
    temperature_feedback: bool,
    nonblocking_mode_switch: bool,
    mit_mode_switch: bool,
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
            torque_feedback: true,
            temperature_feedback: true,
            nonblocking_mode_switch: true,
            mit_mode_switch: true,
        }
    }
}

impl ArmDriver for ConfigLockTestDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: self.torque_feedback,
            temperature_feedback: self.temperature_feedback,
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
        self.nonblocking_mode_switch
    }
}

impl JointModeExt for ConfigLockTestDriver {
    fn supports_mit_mode_switch(&self) -> bool {
        self.mit_mode_switch
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

/// Holds the legacy `DriverLock` mutex on a dedicated OS thread without holding `SupervisorLock`.
///
/// The helper thread owns its own `Arc<ArmSupervisor>` declared strictly before locking the
/// driver mutex, so LIFO drop order guarantees the mutex guard is always released before the
/// backing `SupervisorInner` allocation can be dropped — even on panic, timeout, or guard leak.
struct ContendedDriverGuard {
    release_tx: Option<mpsc::SyncSender<()>>,
    holder: Option<std::thread::JoinHandle<()>>,
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
        let sup_owned = Arc::clone(sup);
        let (locked_tx, locked_rx) = mpsc::sync_channel::<()>(1);
        let (release_tx, release_rx) = mpsc::sync_channel::<()>(1);
        let holder = std::thread::spawn(move || {
            // Keep the `Arc<ArmSupervisor>` owned by this thread for its entire lifetime so
            // `_guard` is always dropped strictly before `_sup_keep_alive`. Tests using this
            // guard never replace or detach `inner.driver`.
            let _sup_keep_alive = sup_owned;
            // SAFETY: `_sup_keep_alive` keeps `SupervisorInner` and its legacy `DriverSlot`
            // alive and pinned inside `Arc<SupervisorLock<SupervisorInner>>` until after `_guard`
            // drops when this thread exits.
            let mutex = unsafe { &*(addr as *const std::sync::Mutex<Box<dyn ArmDriver>>) };
            let _guard = mutex.lock().expect("driver mutex not poisoned");
            let _ = locked_tx.send(());
            let _ = release_rx.recv();
        });
        locked_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("helper thread acquired driver mutex");
        Self {
            release_tx: Some(release_tx),
            holder: Some(holder),
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

    let (entered_tx, mut entered_rx) = tokio::sync::mpsc::unbounded_channel::<()>();
    let (busy_tx, mut busy_rx) = tokio::sync::mpsc::unbounded_channel::<()>();

    let reader = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.published_config_with_driver_busy_hooks(
                move || {
                    let _ = entered_tx.send(());
                },
                move || {
                    let _ = busy_tx.send(());
                },
            )
            .await
        }
    });

    // Deterministic handshake (zero sleeps): wait until the reader has entered
    // `SupervisorLock::read()` and attempted the driver borrow.
    tokio::time::timeout(Duration::from_millis(200), entered_rx.recv())
        .await
        .expect("reader must attempt driver borrow")
        .expect("entered channel open");

    // Bounded handshake: a concurrent supervisor writer MUST acquire `SupervisorLock::write()`
    // while the driver mutex is still held. On base (4ea286c0), `published_config()` blocks inside
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

    // Confirm the non-blocking WouldBlock path actually fired and the reader is still retrying
    // after the writer released its guard (zero sleeps).
    tokio::time::timeout(Duration::from_millis(200), busy_rx.recv())
        .await
        .expect("reader must signal WouldBlock retry after dropping SupervisorLock::read()")
        .expect("busy channel open");
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

    // Per-guard hold recording: no unpreempted SupervisorLock read hold may span contention.
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

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_rest_config_fails_closed_within_50ms_budget_and_recovers_after_release() {
    assert_eq!(
        ArmSupervisor::LEGACY_DRIVER_BUSY_RETRY_BUDGET,
        Duration::from_millis(50)
    );

    let mut cfg = ArmConfig::rebot_dm();
    cfg.profile_id = Some("rest-budget-profile".into());
    let sup = Arc::new(
        ArmSupervisor::new(cfg).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    let router = armd::create_router(Arc::clone(&sup), armd::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .expect("bind ephemeral port");
    let addr = listener.local_addr().expect("local addr");
    let server_jh = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });
    let client = reqwest::Client::new();
    let url = format!("http://{addr}/api/v1/safety/describe");

    // Force a cache miss so the REST config handler must capture from SupervisorInner + driver.
    {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 77;
    }

    // Hold the mock driver mutex outside SupervisorLock for 200 ms on a helper thread.
    let contended = ContendedDriverGuard::acquire(&sup).await;
    let hold_started = Instant::now();
    let release_after_200ms = tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(200)).await;
        contended.release();
    });

    let t0 = Instant::now();
    let resp = client.get(&url).send().await.expect("GET safety/describe");
    let elapsed = t0.elapsed();
    let status = resp.status();
    let body: serde_json::Value = resp.json().await.expect("error json body");

    println!(
        "[legacy_config_lock_rest_budget] status={} code={:?} elapsed_ms={}",
        status.as_u16(),
        body.get("code"),
        elapsed.as_millis(),
    );
    assert_eq!(status, reqwest::StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(body["code"], "CONFIG_UNAVAILABLE");
    assert!(
        elapsed >= Duration::from_millis(45),
        "REST config should retry for the bounded ~50 ms window (got {elapsed:?})"
    );
    assert!(
        elapsed < Duration::from_millis(120),
        "REST config must fail closed near 50 ms budget well before 200 ms hold (got {elapsed:?})"
    );
    assert!(
        hold_started.elapsed() < Duration::from_millis(190),
        "driver mutex must still be held when 503 CONFIG_UNAVAILABLE is returned"
    );

    // Wait for the 200 ms driver hold to release, then verify REST config succeeds cleanly.
    release_after_200ms.await.expect("release task completed");
    let ok_resp = client
        .get(&url)
        .send()
        .await
        .expect("GET safety/describe after release");
    assert_eq!(ok_resp.status(), reqwest::StatusCode::OK);
    let ok_body: serde_json::Value = ok_resp.json().await.expect("ok json body");
    assert_eq!(ok_body["hardware"]["gripper_current_cap"]["current_raw"], 420);
    assert_eq!(ok_body["hardware"]["gripper_current_cap"]["mode"], "pos_force");

    server_jh.abort();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_auth_setter_racing_newer_publish_cannot_overwrite() {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.profile_id = Some("auth-race-legacy".into());
    let sup = Arc::new(
        ArmSupervisor::new(cfg).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    let (dropped_tx, dropped_rx) = tokio::sync::oneshot::channel::<u64>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut dropped_tx = Some(dropped_tx);
    let mut resume_rx = Some(resume_rx);

    let setter = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.set_operator_auth_enforced_with_after_drop_hook(true, |gen| {
                let tx = dropped_tx.take();
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
            .await;
        }
    });

    let setter_gen = tokio::time::timeout(Duration::from_secs(2), dropped_rx)
        .await
        .expect("auth setter dropped SupervisorLock::write()")
        .expect("dropped_tx open");

    let newer_gen = {
        let mut inner = sup.inner().write().await;
        inner.operator_auth_enforced = false;
        inner.workspace_revision = 88;
        inner.config.profile_id = Some("newer-after-setter".into());
        sup.inner().generation()
    };
    assert!(newer_gen > setter_gen);
    let newer = sup.published_config().await.expect("newer publish");
    assert_eq!(newer.workspace_revision, 88);
    assert!(!newer.operator_auth_enforced);

    resume_tx.send(()).expect("resume setter");
    tokio::time::timeout(Duration::from_secs(2), setter)
        .await
        .expect("setter completed")
        .expect("setter did not panic");

    let (hub_after, hub_gen) = sup.published_config_hub().latest_with_gen();
    assert_eq!(hub_gen, newer_gen);
    assert_eq!(hub_after.workspace_revision, 88);
    assert!(!hub_after.operator_auth_enforced);
    assert_eq!(
        hub_after.config.profile_id.as_deref(),
        Some("newer-after-setter")
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_auth_setter_racing_handoff_and_owner_failed_cannot_overwrite_or_enable() {
    // Part 1: Auth setter racing handoff to Executor must not overwrite or set available=true.
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm())
            .with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );
    let (dropped_tx, dropped_rx) = tokio::sync::oneshot::channel::<u64>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut dropped_tx = Some(dropped_tx);
    let mut resume_rx = Some(resume_rx);

    let setter = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.set_operator_auth_enforced_with_after_drop_hook(true, |gen| {
                let tx = dropped_tx.take();
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
            .await;
        }
    });

    tokio::time::timeout(Duration::from_secs(2), dropped_rx)
        .await
        .expect("auth setter dropped write guard")
        .expect("dropped_tx open");

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("start_executor handoff");
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    sup.published_config_hub().update(|cfg| {
        cfg.workspace_revision = 99;
        cfg.config.profile_id = Some("executor-after-setter".into());
    });
    sup.published_config_hub().set_available(false);

    resume_tx.send(()).expect("resume setter after handoff");
    tokio::time::timeout(Duration::from_secs(2), setter)
        .await
        .expect("setter completed")
        .expect("setter did not panic");

    assert!(
        sup.published_config_hub().latest().is_none(),
        "delayed legacy auth setter must never set available=true in OWNER_EXECUTOR"
    );
    sup.published_config_hub().set_available(true);
    let hub_after = sup.published_config_hub().latest().expect("executor hub");
    assert_eq!(hub_after.workspace_revision, 99);
    assert_eq!(
        hub_after.config.profile_id.as_deref(),
        Some("executor-after-setter")
    );
    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .expect("executor joined cleanly");

    // Part 2: Auth setter racing OWNER_FAILED must never re-enable available=true.
    let sup_fail = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let (dropped_tx, dropped_rx) = tokio::sync::oneshot::channel::<u64>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut dropped_tx = Some(dropped_tx);
    let mut resume_rx = Some(resume_rx);

    let setter_fail = tokio::spawn({
        let sup = Arc::clone(&sup_fail);
        async move {
            sup.set_operator_auth_enforced_with_after_drop_hook(true, |gen| {
                let tx = dropped_tx.take();
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
            .await;
        }
    });

    tokio::time::timeout(Duration::from_secs(2), dropped_rx)
        .await
        .expect("auth setter dropped write guard")
        .expect("dropped_tx open");
    sup_fail
        .force_owner_failed_for_mock_test()
        .await
        .expect("force OWNER_FAILED");
    assert!(sup_fail.published_config_hub().latest().is_none());

    resume_tx.send(()).expect("resume setter");
    tokio::time::timeout(Duration::from_secs(2), setter_fail)
        .await
        .expect("setter completed")
        .expect("setter did not panic");
    assert!(
        sup_fail.published_config_hub().latest().is_none(),
        "delayed legacy auth setter must not re-enable hub after OWNER_FAILED"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_cache_hit_during_handoff_routes_through_executor_pin_filter() {
    let sup = Arc::new(
        ArmSupervisor::new(ArmConfig::rebot_dm())
            .with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );
    let _ = sup.published_config().await.expect("warm cache");

    let (before_tx, before_rx) = tokio::sync::oneshot::channel::<()>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut before_tx = Some(before_tx);
    let mut resume_rx = Some(resume_rx);

    let reader = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.published_config_with_cache_check_hooks(
                move || {
                    let tx = before_tx.take();
                    let rx = resume_rx.take();
                    async move {
                        if let Some(tx) = tx {
                            let _ = tx.send(());
                        }
                        if let Some(rx) = rx {
                            let _ = rx.await;
                        }
                    }
                },
                || std::future::ready(()),
            )
            .await
        }
    });

    tokio::time::timeout(Duration::from_secs(2), before_rx)
        .await
        .expect("reader paused before cache check")
        .expect("before_tx open");

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("start_executor handoff");
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
    sup.published_config_hub().update(|cfg| {
        cfg.workspace_revision = 123;
        cfg.sequences.pins = Some(RunPins {
            sequence_id: "seq-1".into(),
            action_id: 999,
            pose_ids: vec!["p1".into()],
            episode_ids: vec![],
        });
    });

    resume_tx.send(()).expect("resume reader");
    let snap = tokio::time::timeout(Duration::from_secs(2), reader)
        .await
        .expect("reader completed")
        .expect("reader did not panic")
        .expect("took executor path");

    assert_eq!(snap.workspace_revision, 123);
    assert!(
        snap.sequences.pins.is_none(),
        "cache-hit during handoff must route through executor pin filter, not return raw hub entry"
    );

    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .expect("executor joined");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn legacy_cache_hit_during_exited_executor_or_owner_failed_fails_closed() {
    // Part 1: Handoff + executor exit occurs right after reading a matching Legacy cache entry.
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let _ = sup.published_config().await.expect("warm cache");

    let (after_tx, after_rx) = tokio::sync::oneshot::channel::<()>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut after_tx = Some(after_tx);
    let mut resume_rx = Some(resume_rx);

    let reader = tokio::spawn({
        let sup = Arc::clone(&sup);
        async move {
            sup.published_config_with_cache_check_hooks(
                || std::future::ready(()),
                move || {
                    let tx = after_tx.take();
                    let rx = resume_rx.take();
                    async move {
                        if let Some(tx) = tx {
                            let _ = tx.send(());
                        }
                        if let Some(rx) = rx {
                            let _ = rx.await;
                        }
                    }
                },
            )
            .await
        }
    });

    tokio::time::timeout(Duration::from_secs(2), after_rx)
        .await
        .expect("reader hit cache and paused before owner re-check")
        .expect("after_tx open");

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("start_executor handoff");
    tokio::task::spawn_blocking(move || handle.stop_and_join())
        .await
        .expect("executor stopped and joined");
    assert!(
        sup.executor_handle().is_some_and(|h| h.is_exited()),
        "executor handle must be exited before resuming reader"
    );

    resume_tx.send(()).expect("resume reader");
    let res = tokio::time::timeout(Duration::from_secs(2), reader)
        .await
        .expect("reader completed")
        .expect("reader did not panic");
    assert!(matches!(
        res,
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "CONFIG_UNAVAILABLE"
    ));

    // Part 2: OWNER_FAILED occurs right after reading a matching Legacy cache entry.
    let sup_fail = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()));
    let _ = sup_fail.published_config().await.expect("warm cache");

    let (after_tx, after_rx) = tokio::sync::oneshot::channel::<()>();
    let (resume_tx, resume_rx) = tokio::sync::oneshot::channel::<()>();
    let mut after_tx = Some(after_tx);
    let mut resume_rx = Some(resume_rx);

    let reader_fail = tokio::spawn({
        let sup = Arc::clone(&sup_fail);
        async move {
            sup.published_config_with_cache_check_hooks(
                || std::future::ready(()),
                move || {
                    let tx = after_tx.take();
                    let rx = resume_rx.take();
                    async move {
                        if let Some(tx) = tx {
                            let _ = tx.send(());
                        }
                        if let Some(rx) = rx {
                            let _ = rx.await;
                        }
                    }
                },
            )
            .await
        }
    });

    tokio::time::timeout(Duration::from_secs(2), after_rx)
        .await
        .expect("reader paused after cache read")
        .expect("after_tx open");
    sup_fail
        .force_owner_failed_for_mock_test()
        .await
        .expect("force OWNER_FAILED");

    resume_tx.send(()).expect("resume reader");
    let res_fail = tokio::time::timeout(Duration::from_secs(2), reader_fail)
        .await
        .expect("reader completed")
        .expect("reader did not panic");
    assert!(matches!(
        res_fail,
        Err(ArmError::Rejected { status: 503, ref code, .. }) if code == "CONFIG_UNAVAILABLE"
    ));
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

    let captured_gen = tokio::time::timeout(Duration::from_secs(2), captured_rx)
        .await
        .expect("older reader reached after_capture hook")
        .expect("captured_tx not dropped");
    assert_eq!(captured_gen, gen_old);

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

    resume_tx.send(()).expect("resume older reader");
    let older_returned = tokio::time::timeout(Duration::from_secs(2), older_reader)
        .await
        .expect("older reader completed")
        .expect("older reader did not panic")
        .expect("older reader returned Ok");

    let (hub_after, hub_gen_after) = sup.published_config_hub().latest_with_gen();
    assert_eq!(hub_gen_after, gen_new);
    assert_eq!(hub_after.workspace_revision, 20);
    assert_eq!(hub_after.config.profile_id.as_deref(), Some("rev-20"));
    assert!(hub_after.operator_auth_enforced);
    assert_eq!(older_returned.workspace_revision, 20);
    assert_eq!(older_returned.config.profile_id.as_deref(), Some("rev-20"));
    assert!(older_returned.operator_auth_enforced);

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

    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .expect("start_executor handoff");
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

    sup.published_config_hub().update(|cfg| {
        cfg.workspace_revision = 99;
        cfg.config.profile_id = Some("executor-epoch".into());
    });

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
    let mut cfg = ArmConfig::rebot_dm();
    if let Some(gp) = cfg.gripper_policy.as_mut() {
        gp.close_speed_rad_s = Some(0.187);
    }
    let sup = Arc::new(
        ArmSupervisor::new(cfg).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.25))),
    );

    // Poison the legacy driver mutex on a separate thread that owns an `Arc<ArmSupervisor>`.
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
    let sup_owned = Arc::clone(&sup);
    let poison_jh = std::thread::spawn(move || {
        let _sup_keep_alive = sup_owned;
        // SAFETY: `_sup_keep_alive` keeps `SupervisorInner` and its legacy `DriverSlot` alive
        // until after `_guard` unwinds.
        let mutex = unsafe { &*(addr as *const std::sync::Mutex<Box<dyn ArmDriver>>) };
        let _guard = mutex.lock().expect("acquire mutex before poison");
        panic!("intentional poison for fallback test");
    });
    assert!(
        poison_jh.join().is_err(),
        "helper thread must panic to poison mutex"
    );

    {
        let mut inner = sup.inner().write().await;
        inner.workspace_revision = 55;
        inner.gravity_compensated = true;
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
    // Poisoned driver cannot report pos_force cap, so close speed stays unquantized at policy 0.187.
    assert!(
        (snap.effective_planned_close_speed - f64::from(0.187_f32)).abs() < 1e-6,
        "poisoned driver must fall back to unquantized policy close speed 0.187, got {}",
        snap.effective_planned_close_speed,
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn legacy_published_config_independent_capability_and_quantization_expectations() {
    // 1. Live driver: pos_force cap below policy (0.223 < 0.30) quantizes downward to 0.22 rad/s,
    //    and gravity_comp_supported tracks independent profile/dynamics/driver requirements.
    let mut cfg_hw = ArmConfig::rebot_dm();
    cfg_hw.dynamics_qualified = true;
    if let Some(gp) = cfg_hw.gripper_policy.as_mut() {
        gp.close_speed_rad_s = Some(0.30);
    }
    let expected_tq_thresholds = cfg_hw
        .torque_guard
        .map(|p| p.thresholds(&cfg_hw))
        .unwrap_or_default();
    let sup_hw = Arc::new(
        ArmSupervisor::new(cfg_hw).with_driver(Box::new(ConfigLockTestDriver::new(7, 420, 0.223))),
    );

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

    {
        let mut inner = sup_hw.inner().write().await;
        inner.workspace_revision = 3;
        inner.operator_auth_enforced = true;
        inner.gravity_compensated = true;
    }
    let hw_snap = sup_hw.published_config().await.expect("hw published_config");
    assert_eq!(hw_snap.workspace_revision, 3);
    assert!(hw_snap.operator_auth_enforced);
    assert!(hw_snap.has_driver);
    assert!(hw_snap.driver_supports_nonblocking_mode_switch);
    assert!(hw_snap.driver_torque_feedback);
    assert!(hw_snap.driver_temperature_feedback);
    assert_eq!(hw_snap.torque_guard_thresholds, expected_tq_thresholds);
    assert_eq!(
        hw_snap.driver_motor_zero_support,
        MotorZeroSupport::set_current("motor_flash", false, vec![])
    );
    let cap_json = hw_snap
        .driver_motor_current_cap
        .as_ref()
        .expect("motor_current_cap serialized");
    assert_eq!(cap_json["joint"], 6);
    assert_eq!(cap_json["mode"], "pos_force");
    assert_eq!(cap_json["current_raw"], 420);
    assert_eq!(cap_json["active"], true);
    assert_eq!(cap_json["calibrated"], false);
    assert_eq!(cap_json["torque_reference"], "bench");
    assert_eq!(cap_json["note"], "test motor current cap");
    // Independent expectation: floor(0.223 * 100) / 100 = 0.22.
    assert!(
        (hw_snap.effective_planned_close_speed - 0.22).abs() < 1e-9,
        "expected floor(0.223*100)/100 = 0.22, got {}",
        hw_snap.effective_planned_close_speed,
    );
    // Independent expectation: 7-DOF rebot_dm with gravity_comp + gravity_model +
    // gravity_compensated=true + dynamics_qualified=true + MIT + vel/torque/enable feedback => true.
    let profile_gc_ready = hw_snap.config.gravity_comp.is_some()
        && hw_snap.config.six_plus_gripper()
        && hw_snap.config.gravity_model.is_some();
    assert!(profile_gc_ready, "rebot_dm must provide gravity_comp + gravity_model");
    assert!(hw_snap.gravity_comp_supported);

    // 2. Flip policy close speed to 0.187 (below driver cap 0.223) -> still quantized to 0.18
    //    because pos_force is active; and flip gravity_compensated=false -> gravity_comp_supported=false.
    {
        let mut inner = sup_hw.inner().write().await;
        if let Some(gp) = inner.config.gripper_policy.as_mut() {
            gp.close_speed_rad_s = Some(0.187);
        }
        inner.gravity_compensated = false;
    }
    let hw_snap2 = sup_hw.published_config().await.expect("hw snap 2");
    assert!(
        (hw_snap2.effective_planned_close_speed - 0.18).abs() < 1e-9,
        "expected floor(0.187*100)/100 = 0.18 under active pos_force, got {}",
        hw_snap2.effective_planned_close_speed,
    );
    assert!(!hw_snap2.gravity_comp_supported);

    // 3. Flip dynamics_qualified=false with gravity_compensated=true -> gravity_comp_supported=false.
    {
        let mut inner = sup_hw.inner().write().await;
        inner.gravity_compensated = true;
        inner.config.dynamics_qualified = false;
    }
    let hw_snap3 = sup_hw.published_config().await.expect("hw snap 3");
    assert!(!hw_snap3.gravity_comp_supported);

    // 4. Driver lacking MIT mode switch -> gravity_comp_supported=false even when qualified.
    let mut no_mit_drv = ConfigLockTestDriver::new(7, 420, 0.40);
    no_mit_drv.mit_mode_switch = false;
    let mut cfg_no_mit = ArmConfig::rebot_dm();
    cfg_no_mit.dynamics_qualified = true;
    let sup_no_mit = Arc::new(ArmSupervisor::new(cfg_no_mit).with_driver(Box::new(no_mit_drv)));
    {
        let mut inner = sup_no_mit.inner().write().await;
        inner.gravity_compensated = true;
    }
    let snap_no_mit = sup_no_mit.published_config().await.expect("no_mit snap");
    assert!(!snap_no_mit.gravity_comp_supported);

    // 5. Driver lacking torque feedback -> gravity_comp_supported=false and driver_torque_feedback=false.
    let mut no_tq_drv = ConfigLockTestDriver::new(7, 420, 0.40);
    no_tq_drv.torque_feedback = false;
    let mut cfg_no_tq = ArmConfig::rebot_dm();
    cfg_no_tq.dynamics_qualified = true;
    let sup_no_tq = Arc::new(ArmSupervisor::new(cfg_no_tq).with_driver(Box::new(no_tq_drv)));
    {
        let mut inner = sup_no_tq.inner().write().await;
        inner.gravity_compensated = true;
    }
    let snap_no_tq = sup_no_tq.published_config().await.expect("no_tq snap");
    assert!(!snap_no_tq.driver_torque_feedback);
    assert!(!snap_no_tq.gravity_comp_supported);

    // 6. Driverless mock follower: temperature_feedback=true, automatic_unload=true,
    //    close speed unquantized (0.187), gravity_comp_supported=true when qualified & compensated.
    let mut cfg_mock = ArmConfig::rebot_dm();
    cfg_mock.dynamics_qualified = true;
    if let Some(gp) = cfg_mock.gripper_policy.as_mut() {
        gp.close_speed_rad_s = Some(0.187);
    }
    let sup_mock = Arc::new(ArmSupervisor::new(cfg_mock));
    {
        let mut inner = sup_mock.inner().write().await;
        inner.workspace_revision = 4;
        inner.gravity_compensated = true;
    }
    let mock_snap = sup_mock
        .published_config()
        .await
        .expect("mock published_config");
    assert!(!mock_snap.has_driver);
    assert!(!mock_snap.driver_supports_nonblocking_mode_switch);
    assert!(!mock_snap.driver_torque_feedback);
    assert!(mock_snap.driver_temperature_feedback);
    assert!(mock_snap.driver_supports_automatic_unload);
    assert!(mock_snap.driver_motor_current_cap.is_none());
    assert!(mock_snap.driver_motor_zero_support.supported);
    assert_eq!(
        mock_snap.driver_motor_zero_support.persist.as_deref(),
        Some("process")
    );
    assert!(mock_snap.gravity_comp_supported);
    assert!(
        (mock_snap.effective_planned_close_speed - f64::from(0.187_f32)).abs() < 1e-6,
        "driverless mock must keep unquantized policy close speed 0.187, got {}",
        mock_snap.effective_planned_close_speed,
    );

    // Cross-check PublishedConfig::from_inner agrees with published_config() on mock too.
    let mock_from_inner = {
        let inner = sup_mock.inner().read().await;
        PublishedConfig::from_inner(&inner)
    };
    assert_eq!(
        mock_snap.gravity_comp_supported,
        mock_from_inner.gravity_comp_supported
    );
    assert!(
        (mock_snap.effective_planned_close_speed - mock_from_inner.effective_planned_close_speed)
            .abs()
            < 1e-9
    );
}
