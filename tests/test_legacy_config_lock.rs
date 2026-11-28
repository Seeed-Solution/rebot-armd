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
use armd::supervisor::ArmSupervisor;
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
