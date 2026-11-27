//! `--test-fail-executor-startup`: validation of the flag (binary, before any hardware is
//! opened) and the recoverable handback it triggers (`PersistTestHooks::fail_control_spawn`):
//! legacy ownership, advancing ticks, zero torque-off commands.

use std::process::{Command, Output, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::JointState;
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks};
use armd::supervisor::OWNER_LEGACY;
use armd::{ArmConfig, ArmSupervisor};

/// Runs armd with `args`, killing it after 20 s; never binds a fixed port.
fn run_armd(args: &[&str]) -> Output {
    let mut child = Command::new(env!("CARGO_BIN_EXE_armd"))
        .args(["--port", "0", "--log-dir", "off", "--blackbox-dir", "off", "--episodes-dir", "off"])
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(20);
    while child.try_wait().unwrap().is_none() {
        if Instant::now() > deadline {
            let _ = child.kill();
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    child.wait_with_output().unwrap()
}

fn text(o: &Output) -> String {
    format!("{}{}", String::from_utf8_lossy(&o.stdout), String::from_utf8_lossy(&o.stderr))
}

#[test]
fn flag_is_refused_for_followers_other_than_rebot_dm() {
    for follower in ["mock", "sim", "so-arm101"] {
        let o = run_armd(&["--follower", follower, "--test-fail-executor-startup"]);
        assert!(!o.status.success(), "{follower}: {}", text(&o));
        assert!(text(&o).contains("--test-fail-executor-startup is only permitted with '--follower rebot-dm'"), "{follower}: {}", text(&o));
    }
}

#[test]
fn flag_is_refused_when_the_resolved_owner_is_legacy() {
    let o = run_armd(&["--follower", "rebot-dm", "--follower-port", "can_missing0", "--owner", "legacy", "--test-fail-executor-startup"]);
    assert!(!o.status.success(), "{}", text(&o));
    assert!(text(&o).contains("needs the RT executor as owner (resolved owner: legacy)"), "{}", text(&o));
}

#[test]
fn flag_is_off_by_default_and_armed_only_by_its_explicit_name() {
    // Without the flag armd on rebot-dm never prints the armed message (it fails later on the
    // missing CAN interface).
    let o = run_armd(&["--follower", "rebot-dm", "--follower-port", "can_missing0", "--owner", "executor"]);
    assert!(!text(&o).contains("ARMED"), "{}", text(&o));
    // a near-miss spelling is an unknown argument
    let o = run_armd(&["--follower", "mock", "--fail-executor-startup"]);
    assert!(!o.status.success());
    assert!(text(&o).contains("Unknown argument: --fail-executor-startup"), "{}", text(&o));
}

#[test]
fn flag_with_rebot_dm_and_executor_owner_passes_validation_and_logs_error_before_hardware() {
    let o = run_armd(&["--follower", "rebot-dm", "--follower-port", "can_missing0", "--owner", "executor", "--test-fail-executor-startup"]);
    let out = text(&o);
    assert!(!out.contains("is only permitted with") && !out.contains("needs the RT executor as owner"), "{out}");
    assert!(out.contains("--test-fail-executor-startup is ARMED"), "{out}");
}

struct CountingDriver {
    state: Arc<Mutex<EnableState>>,
    enable_sent: Arc<AtomicUsize>,
    disable_sent: Arc<AtomicUsize>,
}

impl ArmDriver for CountingDriver {
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
        *self.state.lock().unwrap()
    }
    fn is_enabled(&self) -> bool {
        *self.state.lock().unwrap() == EnableState::Enabled
    }
    fn is_feedback_valid(&self) -> bool {
        true
    }
    fn has_driver_fresh_position_api(&self) -> bool {
        true
    }
    fn fresh_joint_positions_into(&self, joints: &[usize], _max_age: Duration, out: &mut [f32]) -> bool {
        if out.len() < joints.len() {
            return false;
        }
        out[..joints.len()].fill(0.0);
        true
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(Duration::ZERO); 7]
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState::default(); 7])
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        let counter = if enable { &self.enable_sent } else { &self.disable_sent };
        counter.fetch_add(1, Ordering::SeqCst);
        *self.state.lock().unwrap() = if enable { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }
}

/// The hook the flag sets: the executor start fails recoverably, ownership returns to legacy,
/// the driver is not dropped, ticks advance and no torque-off command is sent.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn fail_control_spawn_hands_back_to_legacy_without_torque_off() {
    let state = Arc::new(Mutex::new(EnableState::Enabled));
    let (enable_sent, disable_sent) = (Arc::new(AtomicUsize::new(0)), Arc::new(AtomicUsize::new(0)));
    let drv = CountingDriver { state: state.clone(), enable_sent: enable_sent.clone(), disable_sent: disable_sent.clone() };
    let sup = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(drv)));
    sup.set_lease_enabled(false).await;
    sup.start_dispatcher().unwrap();
    sup.enable_torque(sup.capture_admission_stamp()).await.unwrap();

    let hooks = PersistTestHooks { fail_control_spawn: true, ..Default::default() };
    let started = sup.start_executor_with_hooks(ExecutorConfig::for_tests(Duration::from_millis(5)), hooks).await;
    assert!(started.is_err(), "the forced spawn failure makes the executor start fail");
    assert_eq!(sup.owner_mode(), OWNER_LEGACY, "ownership is back with the legacy owner");
    assert_eq!(Arc::strong_count(&state), 2, "the driver is returned, not dropped");

    // the legacy owner keeps the arm ticking
    let before = sup.tick_stats().releases_total;
    for _ in 0..5 {
        sup.tick(0.01).await;
    }
    assert!(sup.tick_stats().releases_total >= before, "ticks do not go backwards");
    assert_eq!(disable_sent.load(Ordering::SeqCst), 0, "no torque-off command");
    assert_eq!(*state.lock().unwrap(), EnableState::Enabled);
}
