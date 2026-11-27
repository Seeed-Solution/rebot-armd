//! Regression tests for S4-R2 item 1 (codex M1):
//! 1. Legacy fault-clear feedback freshness and resting-pose age checks preserve pre-split
//!    `feedback_ages()` semantics (matching base `2626d1fa`).
//! 2. `FeetechArmDriver::feedback_ages_into()` samples the driver clock per joint, matching
//!    `FeetechArmDriver::feedback_ages()`.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, FeetechArmDriver, JointTarget};
use armd::model::{ArmConfig, JointLimit, JointState, RestingPosePolicy};
use armd::protocol::feetech::{self, calculate_checksum};
use armd::safety::{FaultReason, SafetyError, SafetyState};
use armd::supervisor::{ArmError, ArmSupervisor};
use armd::transport::{DriverClock, MockTransport, Transport, TransportError};

struct SplitAgesState {
    primitive_age: Duration,
    into_age: Duration,
}

/// Test driver whose `feedback_ages()` and `feedback_ages_into()` intentionally disagree so
/// legacy fault-clear gates visibly distinguish which read they perform.
struct SplitAgesDriver {
    dof: usize,
    ages: Arc<Mutex<SplitAgesState>>,
    disabling_joints: Vec<usize>,
    torqued: bool,
}

impl SplitAgesDriver {
    fn new(
        dof: usize,
        primitive_age: Duration,
        into_age: Duration,
        disabling_joints: Vec<usize>,
    ) -> (Self, Arc<Mutex<SplitAgesState>>) {
        let ages = Arc::new(Mutex::new(SplitAgesState { primitive_age, into_age }));
        (
            Self { dof, ages: ages.clone(), disabling_joints, torqued: false },
            ages,
        )
    }
}

impl ArmDriver for SplitAgesDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            torque_feedforward: false,
            enable_feedback: true,
            load_feedback: false,
        }
    }

    fn enable_state(&self) -> EnableState {
        if self.torqued { EnableState::Enabled } else { EnableState::Disabled }
    }

    fn is_feedback_valid(&self) -> bool { true }
    fn is_enabled(&self) -> bool { self.torqued }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let age = self.ages.lock().unwrap().primitive_age;
        vec![Some(age); self.dof]
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let age = self.ages.lock().unwrap().into_age;
        out.clear();
        out.resize(self.dof, Some(age));
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![
            JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 25.0 };
            self.dof
        ])
    }

    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.torqued = enabled;
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), ArmDriverError> { Ok(()) }

    fn clear_fault_disables_joints(&self) -> Vec<usize> { self.disabling_joints.clone() }
}

fn two_joint_config(resting_pose: Option<RestingPosePolicy>) -> ArmConfig {
    let mut cfg = ArmConfig::default();
    cfg.dof = 2;
    cfg.joint_names = vec!["j1".into(), "j2".into()];
    cfg.home_positions = vec![0.0, 0.0];
    cfg.limits = vec![
        JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0),
        JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0),
    ];
    cfg.resting_pose = resting_pose;
    cfg
}

#[tokio::test]
async fn legacy_clear_fault_freshness_gate_uses_feedback_ages() {
    let (drv, ages) = SplitAgesDriver::new(
        2,
        Duration::from_millis(300),
        Duration::from_millis(1),
        Vec::new(),
    );
    let sup = ArmSupervisor::new(two_joint_config(None));
    sup.set_lease_enabled(false).await;
    sup.set_driver(Box::new(drv)).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    // Latch a fault whose clear gate requires fresh feedback (<= 250 ms).
    sup.inner().write().await.safety.trigger_fault(FaultReason::BusLoss);

    // feedback_ages() = 300 ms (stale), feedback_ages_into() = 1 ms (fresh): must reject like base.
    let err = sup.clear_fault().await;
    assert!(
        matches!(err, Err(ArmError::Safety(SafetyError::ClearFaultRejected(_)))),
        "legacy freshness gate must reject when feedback_ages() is 300 ms even if feedback_ages_into() is 1 ms; got {err:?}"
    );
    assert!(matches!(sup.get_observation().await.safety_state, SafetyState::Fault(_)));

    // Inverse: feedback_ages() = 1 ms (fresh), feedback_ages_into() = 300 ms (stale): must accept like base.
    {
        let mut g = ages.lock().unwrap();
        g.primitive_age = Duration::from_millis(1);
        g.into_age = Duration::from_millis(300);
    }
    sup.clear_fault().await.expect("legacy freshness gate must accept when feedback_ages() is 1 ms");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);
}

#[tokio::test]
async fn legacy_clear_fault_resting_pose_age_uses_feedback_ages() {
    let policy = RestingPosePolicy {
        positions: None,
        position_tolerances: vec![0.15, 0.15],
        max_total_deviation: 0.30,
        max_velocity: 0.05,
        joint_max_velocities: vec![],
        support_torque_limits: vec![],
        support_load_limits: vec![],
        feedback_max_age_ms: 100,
    };
    let (drv, ages) = SplitAgesDriver::new(
        2,
        Duration::from_millis(150),
        Duration::from_millis(10),
        vec![0],
    );
    let sup = ArmSupervisor::new(two_joint_config(Some(policy)));
    sup.set_lease_enabled(false).await;
    sup.set_driver(Box::new(drv)).await;
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    sup.inner().write().await.safety.trigger_fault(FaultReason::BusLoss);

    // feedback_ages() = 150 ms (passes 250 ms general freshness, fails 100 ms resting-pose check),
    // feedback_ages_into() = 10 ms (passes both): must reject like base.
    let err = sup.clear_fault().await;
    assert!(
        matches!(err, Err(ArmError::Safety(SafetyError::ClearFaultRejected(_)))),
        "legacy resting-pose age check must reject when feedback_ages() is 150 ms > 100 ms; got {err:?}"
    );
    assert!(matches!(sup.get_observation().await.safety_state, SafetyState::Fault(_)));

    // Inverse: feedback_ages() = 10 ms (<= 100 ms), feedback_ages_into() = 150 ms: must accept like base.
    {
        let mut g = ages.lock().unwrap();
        g.primitive_age = Duration::from_millis(10);
        g.into_age = Duration::from_millis(150);
    }
    sup.clear_fault().await.expect("legacy resting-pose age check must accept when feedback_ages() is 10 ms");
    assert_eq!(sup.get_observation().await.safety_state, SafetyState::Idle);
}

/// Deterministic clock that optionally advances by `step_ms` on every `now()` read.
struct SteppingClock {
    origin: Instant,
    offset_ms: AtomicU64,
    step_ms: u64,
    step_on_now: AtomicBool,
}

impl SteppingClock {
    fn new(step_ms: u64) -> Self {
        Self {
            origin: Instant::now(),
            offset_ms: AtomicU64::new(0),
            step_ms,
            step_on_now: AtomicBool::new(false),
        }
    }

    fn set_offset_ms(&self, ms: u64) {
        self.offset_ms.store(ms, Ordering::SeqCst);
    }

    fn set_stepping(&self, on: bool) {
        self.step_on_now.store(on, Ordering::SeqCst);
    }
}

impl DriverClock for SteppingClock {
    fn clock_id(&self) -> u64 { 0x54_52_32 }

    fn now(&self) -> Instant {
        let ms = if self.step_on_now.load(Ordering::SeqCst) {
            self.offset_ms.fetch_add(self.step_ms, Ordering::SeqCst) + self.step_ms
        } else {
            self.offset_ms.load(Ordering::SeqCst)
        };
        self.origin + Duration::from_millis(ms)
    }

    fn sleep(&self, duration: Duration) {
        self.offset_ms.fetch_add(duration.as_millis() as u64, Ordering::SeqCst);
    }

    fn yield_now(&self) {}
}

struct ClockedTransport {
    inner: MockTransport,
    clock: Arc<SteppingClock>,
}

impl Transport for ClockedTransport {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> { Some(self.clock.clone()) }
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> { self.inner.read(buf) }
    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> { self.inner.write(buf) }
    fn flush(&mut self) -> Result<(), TransportError> { self.inner.flush() }
    fn clear_input_buffer(&mut self) { self.inner.clear_input_buffer(); }
    fn clears_input_buffer(&self) -> bool { self.inner.clears_input_buffer() }
}

fn feetech_feedback_reply(id: u8) -> Vec<u8> {
    let pos = 2048u16.to_le_bytes();
    let params = [pos[0], pos[1], 0, 0, 0, 0, 70, 35];
    let cs = calculate_checksum(id, 10, 0x00, &params);
    let mut pkt = vec![0xFF, 0xFF, id, 10, 0x00];
    pkt.extend_from_slice(&params);
    pkt.push(cs);
    pkt
}

#[test]
fn feetech_feedback_ages_into_samples_clock_per_joint_like_feedback_ages() {
    let clock = Arc::new(SteppingClock::new(2));
    let mut mock = MockTransport::new();
    let ids = vec![1u8, 2, 3];
    let sync_read = feetech::build_sync_read(&ids, feetech::REG_PRESENT_POSITION_L, 8).unwrap();
    let mut reply = Vec::new();
    for &id in &ids {
        reply.extend_from_slice(&feetech_feedback_reply(id));
    }
    mock.stage_reply(&sync_read, &reply);

    let mut drv = FeetechArmDriver::new(ClockedTransport { inner: mock, clock: clock.clone() }, ids)
        .with_calibrated(true)
        .with_tx_echo(Some(false));
    // All three joints receive feedback at clock offset 0 ms.
    drv.read_joint_states().unwrap();

    // Start at 247 ms with +2 ms per clock_now() call:
    // joint 0 -> 249 ms (<= 250 ms), joint 1 -> 251 ms (> 250 ms), joint 2 -> 253 ms (> 250 ms).
    clock.set_stepping(true);
    clock.set_offset_ms(247);
    let ages_prim = drv.feedback_ages();

    clock.set_offset_ms(247);
    let mut ages_into = Vec::new();
    drv.feedback_ages_into(&mut ages_into);

    assert_eq!(
        ages_into, ages_prim,
        "Feetech feedback_ages_into() must sample the clock per joint identically to feedback_ages()"
    );
    assert!(ages_into[0].unwrap() <= Duration::from_millis(250));
    assert!(ages_into[1].unwrap() > Duration::from_millis(250));
    assert!(ages_into[2].unwrap() > Duration::from_millis(250));
}
