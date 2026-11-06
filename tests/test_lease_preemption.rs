use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

use armd::api::create_router;
use armd::api::ws::connection_registry;
use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget,
};
use armd::model::{ArmConfig, JointLimit, JointState};
use armd::safety::{FaultReason, SafetyError, SafetyState, StopPolicy};
use armd::supervisor::{ArmError, ArmSupervisor, PreemptError, SupervisorInner};

// ============================================================================
// Helpers and Test Setup
// ============================================================================

fn test_config() -> ArmConfig {
    ArmConfig {
        dof: 2,
        joint_names: vec!["joint_1".to_string(), "joint_2".to_string()],
        limits: vec![
            JointLimit::new("joint_1", -2.0, 2.0, 2.0, 5.0),
            JointLimit::new("joint_2", -2.0, 2.0, 2.0, 5.0),
        ],
        home_positions: vec![0.0, 0.0],
        lease_timeout_ms: 1000,
        ..Default::default()
    }
}

async fn spawn_test_server() -> (SocketAddr, Arc<ArmSupervisor>) {
    let config = test_config();
    let supervisor = Arc::new(ArmSupervisor::new(config));
    let router = create_router(supervisor.clone());

    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();

    tokio::spawn(async move {
        axum::serve(listener, router).await.unwrap();
    });

    (addr, supervisor)
}

type TestSocket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

async fn ws_reply(ws: &mut TestSocket, message: Value, expected: &[&str]) -> Value {
    ws.send(Message::Text(message.to_string().into())).await.unwrap();
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let value: Value = serde_json::from_str(&text).unwrap();
            if expected.iter().any(|kind| value["type"] == *kind) {
                return value;
            }
        }
    })
    .await
    .expect("Timed out waiting for expected WebSocket reply")
}

async fn ws_recv_timeout(ws: &mut TestSocket, timeout: Duration) -> Option<Value> {
    tokio::time::timeout(timeout, async {
        while let Some(Ok(msg)) = ws.next().await {
            if let Ok(text) = msg.into_text() {
                if let Ok(val) = serde_json::from_str::<Value>(&text) {
                    return val;
                }
            }
        }
        panic!("Socket stream closed unexpectedly");
    })
    .await
    .ok()
}

// Minimal dummy driver for mock and poisoning tests
struct DummyDriver {
    capabilities: DriverCapabilities,
    enable_state: EnableState,
    velocities: Vec<f32>,
    feedback_valid: bool,
    feedback_age: Duration,
}

impl DummyDriver {
    fn new_settled(dof: usize) -> Self {
        Self {
            capabilities: DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
            },
            enable_state: EnableState::Enabled,
            velocities: vec![0.0; dof],
            feedback_valid: true,
            feedback_age: Duration::from_millis(10),
        }
    }
}

impl ArmDriver for DummyDriver {
    fn capabilities(&self) -> DriverCapabilities {
        self.capabilities
    }

    fn enable_state(&self) -> EnableState {
        self.enable_state
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![Some(self.feedback_age); self.velocities.len()]
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(self
            .velocities
            .iter()
            .map(|&v| JointState {
                position: 0.0,
                velocity: v,
                torque: 0.1,
                temperature: 25.0,
            })
            .collect())
    }

    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.enable_state = if enabled {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        };
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        self.feedback_valid
    }
}

// ============================================================================
// Unit Tests on SupervisorInner / ArmSupervisor
// ============================================================================

#[tokio::test]
async fn test_preempt_lease_cooldown_enforcement() {
    let mut inner = SupervisorInner::new(test_config());

    // 1. Initial lease for conn:1
    let gen1 = inner.acquire_lease("conn:1").unwrap();
    assert_eq!(gen1, 1);

    // 2. Cross-connection preempt from conn:2 -> succeeds
    let (gen2, evicted) = inner.preempt_lease("conn:2", "client_2").unwrap();
    assert_eq!(gen2, 2);
    assert_eq!(evicted, Some(("conn:1".to_string(), 1)));
    assert_eq!(inner.active_lease.as_ref().unwrap().client_id, "conn:2");

    // 3. Immediately attempt cross-connection preempt from conn:3 -> rejected by cooldown
    let err = inner.preempt_lease("conn:3", "client_3").unwrap_err();
    match err {
        PreemptError::Cooldown(remaining_ms) => {
            assert!(remaining_ms > 0 && remaining_ms <= 3000);
        }
    }

    // 4. Simulate elapsed 3.1s -> cooldown clears and conn:3 succeeds
    inner.last_preemption = Some(Instant::now() - Duration::from_millis(3100));
    let (gen3, evicted3) = inner.preempt_lease("conn:3", "client_3").unwrap();
    assert_eq!(gen3, 3);
    assert_eq!(evicted3, Some(("conn:2".to_string(), 2)));
}

#[tokio::test]
async fn test_preempt_lease_idempotent_same_connection() {
    let mut inner = SupervisorInner::new(test_config());

    // conn:1 preempts
    let (gen1, evicted1) = inner.preempt_lease("conn:1", "client_1").unwrap();
    assert_eq!(gen1, 1);
    assert_eq!(evicted1, None);

    let preemption_instant = inner.last_preemption;

    // conn:1 preempts again before expiration -> idempotent same-connection check
    let (gen1_again, evicted_again) = inner.preempt_lease("conn:1", "client_1").unwrap();
    assert_eq!(gen1_again, 1);
    assert_eq!(evicted_again, None);
    assert_eq!(inner.lease_generation_counter, 1, "Generation counter must not increment on idempotent re-preempt");
    assert_eq!(inner.last_preemption, preemption_instant, "Cooldown timestamp must not update on idempotent re-preempt");
}

#[tokio::test]
async fn test_preempt_lease_four_categories() {
    let config = test_config();

    // Cat 1: Confirmed Disabled
    {
        let mut inner = SupervisorInner::new(config.clone());
        inner.is_torqued = false;
        assert_eq!(*inner.safety.state(), SafetyState::Idle);

        let (gen, _) = inner.preempt_lease("conn:1", "client_1").unwrap();
        assert_eq!(gen, 1);
        assert!(inner.handover_settling_since.is_none(), "Cat 1 must not enter handover gate");
    }

    // Cat 2: Stopped / Fault / EStop
    {
        let mut inner = SupervisorInner::new(config.clone());
        inner.safety.trigger_e_stop();
        assert!(inner.safety.state().is_stopped());

        let (gen, _) = inner.preempt_lease("conn:1", "client_1").unwrap();
        assert_eq!(gen, 1);
        assert!(inner.handover_settling_since.is_none(), "Cat 2 must not enter handover gate");
    }

    // Cat 3: Fully Settled Armed Hold
    {
        let mut inner = SupervisorInner::new(config.clone());
        inner.set_torque(true).unwrap();
        assert_eq!(*inner.safety.state(), SafetyState::Armed);
        assert_eq!(inner.commanded_velocities, vec![0.0, 0.0]);
        assert_eq!(inner.measured_velocities, vec![0.0, 0.0]);

        let (gen, _) = inner.preempt_lease("conn:1", "client_1").unwrap();
        assert_eq!(gen, 1);
        assert!(inner.handover_settling_since.is_none(), "Cat 3 must not enter handover gate");
    }

    // Cat 4: Moving / Braking / Uncertain -> Must enter gate!
    {
        let mut inner = SupervisorInner::new(config.clone());
        inner.set_torque(true).unwrap();
        inner.commanded_velocities = vec![0.5, -0.5];
        inner.measured_velocities = vec![0.5, -0.5];

        let (gen, _) = inner.preempt_lease("conn:1", "client_1").unwrap();
        assert_eq!(gen, 1);
        assert!(inner.handover_settling_since.is_some(), "Cat 4 must enter handover gate!");
    }
}

#[tokio::test]
async fn test_handover_settling_motion_gate_locks_actuation() {
    let mut inner = SupervisorInner::new(test_config());
    inner.set_torque(true).unwrap();
    let epoch = inner.safety.epoch_id();

    // Lock handover gate
    inner.handover_settling_since = Some(Instant::now());

    // 1. set_torque(true) must be rejected
    let torque_err = inner.set_torque(true).unwrap_err();
    assert_eq!(torque_err, SafetyError::HandoverSettling);

    // 2. jog must be rejected
    let jog_err = inner.jog(vec![0.1, 0.1], Some(1.0), Some(epoch)).unwrap_err();
    assert!(matches!(jog_err, ArmError::Safety(SafetyError::HandoverSettling)));

    // 3. servo must be rejected
    let servo_err = inner.servo(vec![0.1, 0.1], epoch).unwrap_err();
    assert_eq!(servo_err, SafetyError::HandoverSettling);

    // 4. NormalUnload must not start motion
    inner.execute_stop(StopPolicy::NormalUnload);
    assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none());
}

#[tokio::test]
async fn test_handover_settling_tick_convergence_to_cat3() {
    let mut inner = SupervisorInner::new(test_config());
    inner.set_torque(true).unwrap();

    // In Cat 4: arm has commanded velocity and gate is active
    inner.commanded_velocities = vec![0.05, 0.05];
    inner.measured_velocities = vec![0.05, 0.05];
    inner.handover_settling_since = Some(Instant::now());

    // Run tick cycles to decelerate
    for _ in 0..20 {
        inner.tick(0.02);
    }

    // Deceleration brought velocities to 0, satisfying Cat 3
    assert_eq!(inner.commanded_velocities, vec![0.0, 0.0]);
    assert_eq!(inner.measured_velocities, vec![0.0, 0.0]);
    assert!(
        inner.handover_settling_since.is_none(),
        "Settling must automatically unlock handover gate once Cat 3 criteria are met"
    );

    // Motion command should now succeed
    let epoch = inner.safety.epoch_id();
    assert!(inner.jog(vec![0.1, 0.1], Some(1.0), Some(epoch)).is_ok());
}

#[tokio::test]
async fn test_handover_settling_timeout_latches_fault() {
    let mut inner = SupervisorInner::new(test_config());
    inner.set_torque(true).unwrap();

    // Moving arm unable to settle within 1500ms
    inner.commanded_velocities = vec![0.5, 0.5];
    inner.measured_velocities = vec![0.5, 0.5];
    inner.handover_settling_since = Some(Instant::now() - Duration::from_millis(1501));

    inner.tick(0.02);

    // Must timeout, clear handover_settling_since, zero commanded velocity, and latch Fault
    assert!(inner.handover_settling_since.is_none());
    assert_eq!(inner.commanded_velocities, vec![0.0, 0.0]);
    assert_eq!(
        *inner.safety.state(),
        SafetyState::Fault(FaultReason::Other("Handover settling timeout".into()))
    );
    // Crucial safety check: torque hold is maintained, power is NOT cut!
    assert!(inner.is_torqued, "Arm must maintain holding torque to prevent falling!");
}

#[tokio::test]
async fn test_fail_closed_driver_lock_poison_protection() {
    let mut inner = SupervisorInner::new(test_config());

    // Create a poisoned driver mutex
    let poisoned_mutex: std::sync::Mutex<Box<dyn ArmDriver>> = {
        let m = Arc::new(std::sync::Mutex::new(
            Box::new(DummyDriver::new_settled(2)) as Box<dyn ArmDriver>,
        ));
        let m_clone = m.clone();
        let _ = std::thread::spawn(move || {
            let _lock = m_clone.lock().unwrap();
            panic!("deliberate panic to poison driver mutex");
        })
        .join();
        Arc::try_unwrap(m).map_err(|_| "unwrap failed").unwrap()
    };

    inner.driver = Some(poisoned_mutex);
    inner.is_torqued = true;

    // 1. Preempt with poisoned driver: ZERO PANIC, Fail-Closed into Cat 4
    let res = inner.preempt_lease("conn:poison", "label");
    assert!(res.is_ok(), "preempt_lease must not panic on poisoned lock");
    assert!(
        inner.handover_settling_since.is_some(),
        "Poisoned driver must fail-closed into Cat 4 gate"
    );

    // 2. Tick with poisoned driver: ZERO PANIC, driver failure safely faults arm
    inner.tick(0.02);
    assert!(
        inner.safety.state().is_stopped(),
        "Poisoned driver must fail-closed into stopped/fault state"
    );
    assert!(
        inner.handover_settling_since.is_none(),
        "Handover settling cleanly terminates when driver faults"
    );
}

// ============================================================================
// WebSocket Integration Tests
// ============================================================================

#[tokio::test]
async fn test_websocket_connection_registry_lifecycle() {
    let (addr, supervisor) = spawn_test_server().await;

    // Connect socket
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    // Acquire lease so supervisor records this connection's ID
    let reply = ws_reply(
        &mut ws,
        json!({"type":"acquire_lease", "client_id":"registry_probe", "seq":1}),
        &["lease_granted"],
    )
    .await;
    assert_eq!(reply["type"], "lease_granted");

    let my_conn_id = supervisor
        .inner()
        .read()
        .await
        .active_lease
        .as_ref()
        .unwrap()
        .client_id
        .clone();

    // Connection must be present in registry
    assert!(
        connection_registry().lock().unwrap().contains_key(&my_conn_id),
        "Active WebSocket connection must be present in connection_registry"
    );

    // Drop socket
    drop(ws);
    tokio::time::sleep(Duration::from_millis(50)).await;

    // Connection must be cleanly removed from registry
    assert!(
        !connection_registry().lock().unwrap().contains_key(&my_conn_id),
        "Closed WebSocket connection must be removed from connection_registry"
    );
}

#[tokio::test]
async fn test_websocket_acquire_lease_seq_echo_back() {
    let (addr, _) = spawn_test_server().await;
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    // 1. acquire_lease with seq -> granted echoes seq
    let granted = ws_reply(
        &mut ws,
        json!({"type":"acquire_lease", "client_id":"client_seq", "seq":1001}),
        &["lease_granted"],
    )
    .await;
    assert_eq!(granted["seq"], 1001);
    assert_eq!(granted["client_id"], "client_seq");

    // 2. Invalid force with seq -> rejected echoes seq
    let rejected = ws_reply(
        &mut ws,
        json!({"type":"acquire_lease", "client_id":"client_seq", "force":"invalid", "seq":1002}),
        &["lease_rejected"],
    )
    .await;
    assert_eq!(rejected["seq"], 1002);
    assert_eq!(rejected["status"], 400);
    assert_eq!(rejected["error"], "invalid_argument");
}

#[tokio::test]
async fn test_websocket_force_parameter_validation() {
    let (addr, _) = spawn_test_server().await;
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    for invalid_val in [json!("true"), json!(1), json!([]), json!({"bad":1})] {
        let reply = ws_reply(
            &mut ws,
            json!({"type":"acquire_lease", "client_id":"tester", "force":invalid_val, "seq":42}),
            &["lease_rejected"],
        )
        .await;
        assert_eq!(reply["status"], 400);
        assert_eq!(reply["error"], "invalid_argument");
        assert_eq!(reply["message"], "force must be a boolean");
        assert_eq!(reply["seq"], 42);
    }

    // force: null and omitted are valid and treated as false
    let reply_null = ws_reply(
        &mut ws,
        json!({"type":"acquire_lease", "client_id":"tester", "force":null, "seq":43}),
        &["lease_granted"],
    )
    .await;
    assert_eq!(reply_null["type"], "lease_granted");
    assert_eq!(reply_null["seq"], 43);
}

#[tokio::test]
async fn test_websocket_preemption_and_eviction_notification() {
    let (addr, _) = spawn_test_server().await;

    // Client 1 connects and acquires lease
    let (mut ws1, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let grant1 = ws_reply(
        &mut ws1,
        json!({"type":"acquire_lease", "client_id":"client_1", "seq":1}),
        &["lease_granted"],
    )
    .await;
    assert_eq!(grant1["generation"], 1);

    // Client 2 connects
    let (mut ws2, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    // Client 2 sends standard acquire (force: false) -> rejected as controller_busy
    let busy = ws_reply(
        &mut ws2,
        json!({"type":"acquire_lease", "client_id":"client_2", "seq":2, "force":false}),
        &["lease_rejected"],
    )
    .await;
    assert_eq!(busy["error"], "controller_busy");
    assert_eq!(busy["seq"], 2);

    // Client 2 sends force: true -> preempts!
    let grant2 = ws_reply(
        &mut ws2,
        json!({"type":"acquire_lease", "client_id":"client_2", "seq":3, "force":true}),
        &["lease_granted"],
    )
    .await;
    assert_eq!(grant2["generation"], 2);
    assert_eq!(grant2["seq"], 3);

    // Client 1 must receive lease_lost with reason preempted and previous generation
    let mut got_lease_lost = false;
    for _ in 0..10 {
        if let Some(msg) = ws_recv_timeout(&mut ws1, Duration::from_millis(200)).await {
            if msg["type"] == "lease_lost" {
                assert_eq!(msg["generation"], 1);
                assert_eq!(msg["reason"], "preempted");
                got_lease_lost = true;
                break;
            }
        }
    }
    assert!(got_lease_lost, "Evicted Client 1 must receive lease_lost notification");
}

#[tokio::test]
async fn test_websocket_preemption_cooldown_error() {
    let (addr, _) = spawn_test_server().await;

    let (mut ws1, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let (mut ws2, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let (mut ws3, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    // Client 1 acquires
    ws_reply(&mut ws1, json!({"type":"acquire_lease", "client_id":"c1"}), &["lease_granted"]).await;

    // Client 2 preempts
    ws_reply(&mut ws2, json!({"type":"acquire_lease", "client_id":"c2", "force":true}), &["lease_granted"]).await;

    // Client 3 immediately attempts preemption -> preemption_cooldown with remaining_ms and seq
    let cd_reply = ws_reply(
        &mut ws3,
        json!({"type":"acquire_lease", "client_id":"c3", "force":true, "seq":301}),
        &["lease_rejected"],
    )
    .await;
    assert_eq!(cd_reply["error"], "preemption_cooldown");
    assert_eq!(cd_reply["seq"], 301);
    let remaining = cd_reply["remaining_ms"].as_u64().unwrap();
    assert!(remaining > 0 && remaining <= 3000);
}

#[tokio::test]
async fn test_websocket_handover_settling_rejection_409() {
    static REQ: AtomicU64 = AtomicU64::new(1);
    let (addr, supervisor) = spawn_test_server().await;

    let (mut ws1, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();
    let (mut ws2, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    // Client 1 acquires, enables torque, starts jogging
    let g1 = ws_reply(&mut ws1, json!({"type":"acquire_lease", "client_id":"c1"}), &["lease_granted"]).await["generation"].as_u64().unwrap();
    ws_reply(&mut ws1, json!({"type":"command", "request_id":REQ.fetch_add(1, Ordering::Relaxed), "generation":g1, "epoch_id":1, "path":"/api/v1/torque", "body":{"enable":true}}), &["command_ack"]).await;

    // Set arm in motion
    supervisor.jog(vec![0.5, -0.5], Some(1.0), Some(1)).await.unwrap();

    // Client 2 preempts while arm is moving -> enters Cat 4 settling gate!
    let g2 = ws_reply(&mut ws2, json!({"type":"acquire_lease", "client_id":"c2", "force":true}), &["lease_granted"]).await["generation"].as_u64().unwrap();

    // Client 2 attempts jog while settling -> rejected with status 409 and HANDOVER_SETTLING
    let reject = ws_reply(
        &mut ws2,
        json!({
            "type": "command",
            "request_id": REQ.fetch_add(1, Ordering::Relaxed),
            "generation": g2,
            "epoch_id": 1,
            "path": "/api/v1/motion/jog",
            "body": {"target_positions": [0.2, 0.2]}
        }),
        &["command_reject"],
    )
    .await;

    assert_eq!(reject["type"], "command_reject");
    assert_eq!(reject["status"], 409);
    assert_eq!(reject["code"], "HANDOVER_SETTLING");
    assert_eq!(reject["error"], "HANDOVER_SETTLING");
    assert!(reject["message"].as_str().unwrap().contains("motion gate locked"));
}

#[tokio::test]
async fn test_websocket_lease_lost_includes_generation() {
    let (addr, _) = spawn_test_server().await;
    let (mut ws, _) = connect_async(format!("ws://{}/api/v1/ws", addr)).await.unwrap();

    // Send command without acquiring lease first (local generation is 0)
    let lost = ws_reply(&mut ws, json!({"type":"heartbeat", "generation":999, "epoch_id":1}), &["lease_lost"]).await;
    assert_eq!(lost["type"], "lease_lost");
    assert_eq!(lost["generation"], 0);
}

#[test]
fn test_handover_settling_preserves_thermal_trip_fault() {
    let mut inner = SupervisorInner::new(test_config());
    let _ = inner.acquire_lease("c1").unwrap();
    inner.set_torque(true).unwrap();

    // Trigger Cat 4 preemption (simulate moving)
    inner.commanded_velocities = vec![0.5; inner.config.dof];
    let _ = inner.preempt_lease("c2", "test").unwrap();
    assert!(inner.handover_settling_since.is_some());

    // While settling, ThermalTrip policy executes (e.g. from thermal protection)
    inner.temperatures = vec![85.0; inner.config.dof];
    inner.apply_stop_policy(StopPolicy::ThermalTrip);
    inner.tick(0.01);
    assert!(inner.handover_settling_since.is_none());
    assert!(matches!(inner.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { joint_idx: 1, temperature: t, .. }) if (t - 85.0).abs() < 1e-3));

    // Even if handover settling timeout expires later, the ThermalTrip fault must not be overwritten
    inner.handover_settling_since = Some(Instant::now() - Duration::from_millis(1501));
    inner.tick(0.01);
    assert!(inner.handover_settling_since.is_none());
    assert!(matches!(inner.safety.state(), SafetyState::Fault(FaultReason::ThermalTrip { joint_idx: 1, temperature: t, .. }) if (t - 85.0).abs() < 1e-3));
}

#[test]
fn test_handover_settling_idle_holding_or_disabled_completes_without_fault() {
    let mut inner = SupervisorInner::new(test_config());
    let _ = inner.acquire_lease("c1").unwrap();

    // 1. Arm is in Idle with torque held (e.g. after clear_fault())
    inner.is_torqued = true;
    assert_eq!(*inner.safety.state(), SafetyState::Idle);

    // Preempting while idle and holding position should complete immediately or settle without timing out to Fault
    let _ = inner.preempt_lease("c2", "test").unwrap();
    assert!(inner.handover_settling_since.is_none(), "Idle holding arm must settle without gate lock");

    // 2. If settling was active and torque is cut to Idle, tick unblocks gate without spurious fault
    inner.commanded_velocities = vec![0.5; inner.config.dof];
    inner.last_preemption = None;
    let _ = inner.preempt_lease("c3", "test").unwrap();
    assert!(inner.handover_settling_since.is_some());

    inner.is_torqued = false;
    inner.tick(0.01);
    assert!(inner.handover_settling_since.is_none(), "Disabled Idle arm must unblock gate cleanly");
    assert_eq!(*inner.safety.state(), SafetyState::Idle);
}

#[test]
fn test_dormant_disabled_arm_with_stale_feedback_preempts_without_handover_fault() {
    struct DormantDriver {
        _motor_status: Vec<u8>,
    }
    impl ArmDriver for DormantDriver {
        fn capabilities(&self) -> DriverCapabilities {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
            }
        }
        fn enable_state(&self) -> EnableState {
            EnableState::Disabled
        }
        fn feedback_ages(&self) -> Vec<Option<Duration>> {
            // Dormant motors transmit zero CAN frames while idle
            vec![Some(Duration::from_secs(30)); 2]
        }
        fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
            Ok(vec![
                JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 25.0 },
                JointState { position: 0.0, velocity: 0.0, torque: 0.0, temperature: 25.0 },
            ])
        }
        fn set_torque(&mut self, _enable: bool) -> Result<(), ArmDriverError> {
            Ok(())
        }
        fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
            Ok(())
        }
    }

    let mut inner = SupervisorInner::new(test_config())
        .with_driver(Box::new(DormantDriver { _motor_status: vec![0, 0] }));

    // Verify initial conditions: unpowered, idle, dormant stale feedback
    assert!(!inner.is_torqued);
    assert_eq!(*inner.safety.state(), SafetyState::Idle);
    assert!(!inner.hardware_feedback_fresh(Duration::from_millis(250)));

    let _ = inner.acquire_lease("c1").unwrap();

    // Preempt lease on dormant idle arm
    let res = inner.preempt_lease("c2", "web-client");
    assert!(res.is_ok());
    assert!(inner.handover_settling_since.is_none(), "Dormant idle arm must not enter handover settling");

    // Multiple control ticks on dormant arm must NEVER latch Handover settling timeout
    for _ in 0..100 {
        inner.tick(0.02);
    }
    assert_eq!(*inner.safety.state(), SafetyState::Idle);
    assert!(!matches!(inner.safety.state(), SafetyState::Fault(_)));
    assert!(inner.shutdown_ready(), "Dormant idle arm must be ready for safe shutdown");
}
