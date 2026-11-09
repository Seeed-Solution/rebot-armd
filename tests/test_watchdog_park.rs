//! Lease watchdog expiry while torqued: park at the resting pose when the arm is
//! healthy, otherwise hold torque and latch Fault(WatchdogTimeout).
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, JointLimit, JointState, RestingPosePolicy};
use armd::safety::FaultReason;
use armd::{ArmSupervisor, SafetyState};

const LEASE_MS: u64 = 50;

fn policy(dof: usize) -> RestingPosePolicy {
    RestingPosePolicy { position_tolerances: vec![0.08; dof], max_total_deviation: 0.08 * dof as f32,
        max_velocity: 0.015, support_torque_limits: vec![(0, 1.0)], feedback_max_age_ms: 100 }
}

fn sim_config(resting_pose: bool) -> ArmConfig {
    let mut config = ArmConfig::default();
    config.dof = 2;
    config.limits = vec![JointLimit::new("j1", -3.0, 3.0, 2.0, 5.0), JointLimit::new("j2", -3.0, 3.0, 2.0, 5.0)];
    config.home_positions = vec![0.0, 0.0];
    config.lease_timeout_ms = LEASE_MS;
    config.resting_pose = resting_pose.then(|| policy(2));
    config
}

/// Simulated arm, torqued and parked away from home with the lease watchdog armed.
async fn sim_torqued_away_from_home(resting_pose: bool) -> ArmSupervisor {
    let supervisor = ArmSupervisor::new(sim_config(resting_pose));
    supervisor.set_torque(true).await.unwrap();
    supervisor.jog(vec![1.0, 0.5], Some(0.3), None).await.unwrap();
    for _ in 0..80 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert!((obs.positions[0] - 1.0).abs() < 1e-3 && obs.is_torqued);
    supervisor
}

async fn expire_lease(supervisor: &ArmSupervisor) -> armd::ArmObservation {
    tokio::time::sleep(Duration::from_millis(LEASE_MS + 30)).await;
    supervisor.tick(0.02).await
}

#[tokio::test]
async fn lease_expiry_while_torqued_parks_then_cuts_torque_to_idle() {
    let supervisor = sim_torqued_away_from_home(true).await;
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped(), "healthy arm must park, not fault: {:?}", obs.safety_state);
    {
        let inner = supervisor.inner().read().await;
        assert!(inner.pending_normal_unload && inner.watchdog_park_deadline.is_some());
        assert_eq!(inner.target_positions, vec![0.0, 0.0]);
    }
    for _ in 0..200 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued, "park must end with torque off");
    assert!(obs.positions.iter().all(|p| p.abs() < 1e-3), "torque cut only at home: {:?}", obs.positions);
    assert!(supervisor.inner().read().await.watchdog_park_deadline.is_none());
}

#[tokio::test]
async fn released_lease_while_torqued_parks_the_same_way() {
    let supervisor = sim_torqued_away_from_home(true).await;
    {
        let mut inner = supervisor.inner().write().await;
        let generation = inner.acquire_lease("client-a").unwrap();
        assert!(inner.release_lease("client-a", generation));
        inner.stop_motion(); // what the WS handler does on release / close
    }
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped());
    assert!(supervisor.inner().read().await.pending_normal_unload);
    for _ in 0..200 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued);
}

#[tokio::test]
async fn new_lease_during_park_takes_over_and_can_move() {
    let supervisor = sim_torqued_away_from_home(true).await;
    expire_lease(&supervisor).await;
    for _ in 0..5 { supervisor.tick(0.02).await; }
    {
        let mut inner = supervisor.inner().write().await;
        assert!(inner.pending_normal_unload);
        inner.acquire_lease("client-b").expect("lease must not be refused during a park");
        assert!(!inner.pending_normal_unload && inner.active_trajectory.is_none());
        assert!(inner.watchdog_park_deadline.is_none());
    }
    for _ in 0..50 {
        supervisor.heartbeat("client-b", supervisor.get_observation().await.epoch_id).await.unwrap();
        supervisor.tick(0.02).await;
    }
    let obs = supervisor.get_observation().await;
    assert!(obs.is_torqued && !obs.safety_state.is_stopped(), "takeover holds torque: {:?}", obs.safety_state);
    supervisor.jog(vec![0.8, 0.4], Some(0.3), None).await.expect("new owner commands are accepted");
}

#[tokio::test]
async fn preempting_lease_during_park_cancels_it() {
    let supervisor = sim_torqued_away_from_home(true).await;
    expire_lease(&supervisor).await;
    supervisor.tick(0.02).await;
    let mut inner = supervisor.inner().write().await;
    assert!(inner.pending_normal_unload);
    inner.preempt_lease("conn-c", "cockpit").expect("preempt must not be refused during a park");
    assert!(!inner.pending_normal_unload && inner.watchdog_park_deadline.is_none());
    assert!(inner.is_torqued && !inner.safety.state().is_stopped());
}

#[tokio::test]
async fn no_resting_pose_keeps_watchdog_fault_with_torque_held() {
    let supervisor = sim_torqued_away_from_home(false).await;
    let obs = expire_lease(&supervisor).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)));
    assert!(obs.is_torqued);
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

#[tokio::test]
async fn already_latched_fault_is_not_parked() {
    let supervisor = sim_torqued_away_from_home(true).await;
    supervisor.inner().write().await.safety.trigger_fault(FaultReason::BusLoss);
    let obs = expire_lease(&supervisor).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::BusLoss)));
    assert!(obs.is_torqued, "torque stays on");
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

#[tokio::test]
async fn expiry_while_not_torqued_does_nothing() {
    let supervisor = ArmSupervisor::new(sim_config(true));
    let obs = expire_lease(&supervisor).await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued);
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

// ---- hardware-like driver with controllable feedback age and fixed position ----

struct Hardware { position: f32, age: Option<Duration>, enabled: EnableState }
struct Driver(Arc<Mutex<Hardware>>);
impl ArmDriver for Driver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities { velocity_feedback: true, torque_feedback: true, temperature_feedback: true,
            torque_feedforward: true, enable_feedback: true }
    }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enabled }
    fn is_feedback_valid(&self) -> bool { self.0.lock().unwrap().age.is_some_and(|a| a <= Duration::from_millis(250)) }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { vec![self.0.lock().unwrap().age] }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        Ok(vec![JointState { position: self.0.lock().unwrap().position, ..JointState::default() }])
    }
    fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, on: bool) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().enabled = if on { EnableState::Enabled } else { EnableState::Disabled };
        Ok(())
    }
}

async fn hw_torqued(position: f32) -> (ArmSupervisor, Arc<Mutex<Hardware>>) {
    let mut config = ArmConfig::default();
    config.dof = 1;
    config.home_positions = vec![0.0];
    config.limits = vec![JointLimit::new("axis", -2.0, 2.0, 2.0, 5.0)];
    config.lease_timeout_ms = LEASE_MS;
    config.resting_pose = Some(policy(1));
    let hw = Arc::new(Mutex::new(Hardware { position, age: Some(Duration::ZERO), enabled: EnableState::Disabled }));
    let supervisor = ArmSupervisor::new(config).with_driver(Box::new(Driver(hw.clone())));
    supervisor.tick(0.02).await;
    supervisor.set_torque(true).await.unwrap();
    (supervisor, hw)
}

#[tokio::test]
async fn stale_feedback_keeps_watchdog_fault_with_torque_held() {
    let (supervisor, hw) = hw_torqued(0.3).await;
    // Older than the resting-pose bound (100 ms), younger than bus loss (500 ms).
    hw.lock().unwrap().age = Some(Duration::from_millis(200));
    let obs = expire_lease(&supervisor).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)), "{:?}", obs.safety_state);
    assert!(obs.is_torqued);
    assert_eq!(hw.lock().unwrap().enabled, EnableState::Enabled);
}

#[tokio::test]
async fn park_that_cannot_reach_rest_falls_back_to_fault_without_cutting_torque() {
    // The joint stays at 0.3 rad (inside the following-error bound): the park never
    // satisfies the resting-pose check, so torque must never be cut.
    let (supervisor, hw) = hw_torqued(0.3).await;
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped(), "{:?}", obs.safety_state);
    assert!(supervisor.inner().read().await.pending_normal_unload);
    for _ in 0..120 { supervisor.tick(0.02).await; }
    assert!(supervisor.get_observation().await.is_torqued, "never torque-off away from the resting pose");
    supervisor.inner().write().await.watchdog_park_deadline = Some(Instant::now());
    let obs = supervisor.tick(0.02).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(FaultReason::WatchdogTimeout)), "{:?}", obs.safety_state);
    assert!(obs.is_torqued);
    assert_eq!(hw.lock().unwrap().enabled, EnableState::Enabled);
    assert!(!supervisor.inner().read().await.pending_normal_unload);
}

// ---- WebSocket close after the lease expired (real-arm incident, armd fce83ff) ----

/// Serve `supervisor` over the real router (no control loop: the test ticks it).
async fn serve(supervisor: Arc<ArmSupervisor>) -> std::net::SocketAddr {
    let router = armd::api::create_router(supervisor);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, router).await.unwrap(); });
    addr
}

async fn ws_lease(addr: std::net::SocketAddr) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    use futures_util::{SinkExt, StreamExt};
    let (mut ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let acquire = serde_json::json!({"type":"acquire_lease", "seq":1, "client_id":"sdk"});
    ws.send(tokio_tungstenite::tungstenite::Message::Text(acquire.to_string().into())).await.unwrap();
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: serde_json::Value = serde_json::from_str(&text).unwrap();
            if v["type"] == "lease_granted" { return; }
        }
    }).await.expect("lease_granted");
    ws
}

/// The lease holder stops heartbeating (transport stall), the watchdog parks, then the
/// client closes its socket. The close must not cancel the park: on the real arm the
/// close-time `release_lease` + `stop_motion` left it holding torque mid-way and
/// latched Fault(WatchdogTimeout) ("Watchdog park interrupted (safety state Moving)").
#[tokio::test]
async fn closing_socket_after_lease_expired_does_not_cancel_watchdog_park() {
    let supervisor = Arc::new(sim_torqued_away_from_home(true).await);
    let addr = serve(supervisor.clone()).await;
    let mut ws = ws_lease(addr).await;
    let obs = expire_lease(&supervisor).await;
    assert!(!obs.safety_state.is_stopped(), "{:?}", obs.safety_state);
    assert!(supervisor.inner().read().await.watchdog_park_deadline.is_some(), "watchdog park started");
    for _ in 0..3 { supervisor.tick(0.02).await; }

    ws.close(None).await.unwrap();
    drop(ws);
    // Let the server observe the close and run its disconnect cleanup.
    tokio::time::sleep(Duration::from_millis(150)).await;
    {
        let inner = supervisor.inner().read().await;
        assert!(inner.pending_normal_unload, "socket close of an expired lease cancelled the watchdog park");
        assert!(inner.watchdog_park_deadline.is_some());
    }
    for _ in 0..200 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle, "park completes");
    assert!(!obs.is_torqued);
}

/// A socket close by a client that still holds a valid lease keeps stopping its motion.
#[tokio::test]
async fn closing_socket_with_valid_lease_still_stops_motion() {
    let supervisor = Arc::new(ArmSupervisor::new({ let mut c = sim_config(true); c.lease_timeout_ms = 5_000; c }));
    supervisor.set_torque(true).await.unwrap();
    let addr = serve(supervisor.clone()).await;
    let ws = ws_lease(addr).await;
    supervisor.jog(vec![1.0, 0.5], Some(2.0), None).await.unwrap();
    supervisor.tick(0.02).await;
    assert!(supervisor.inner().read().await.active_trajectory.is_some());
    drop(ws);
    tokio::time::sleep(Duration::from_millis(150)).await;
    let inner = supervisor.inner().read().await;
    assert!(inner.active_trajectory.is_none(), "disconnect of the lease holder stops its motion");
    assert!(inner.active_lease.is_none());
}

/// Same incident shape: the lease expires mid joint_move action. The action is failed
/// with WATCHDOG, and neither the action bookkeeping nor the later socket close may
/// cancel the park.
#[tokio::test]
async fn lease_expiry_mid_action_then_close_still_parks_to_idle() {
    let supervisor = Arc::new(ArmSupervisor::new(sim_config(true)));
    supervisor.set_torque(true).await.unwrap();
    let addr = serve(supervisor.clone()).await;
    let ws = ws_lease(addr).await;
    {
        let mut inner = supervisor.inner().write().await;
        let epoch = inner.safety.epoch_id();
        inner.jog_action(vec![1.0, 0.5], Some(2.0), Some(epoch), "joint_move").unwrap();
    }
    for _ in 0..5 { supervisor.tick(0.02).await; }
    let obs = expire_lease(&supervisor).await;
    assert_eq!(obs.action.as_ref().map(|a| a.state.as_str()), Some("failed"));
    assert!(supervisor.inner().read().await.watchdog_park_deadline.is_some());
    for _ in 0..3 { supervisor.tick(0.02).await; }
    drop(ws);
    tokio::time::sleep(Duration::from_millis(150)).await;
    assert!(supervisor.inner().read().await.pending_normal_unload, "park cancelled");
    for _ in 0..300 { supervisor.tick(0.05).await; }
    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle, "park completes");
    assert!(!obs.is_torqued);
}

/// Problem-1 candidate check: heartbeats on the WS keep renewing the lease through
/// jog action -> unload (park, torque off) -> torque on -> jog action, i.e. neither an
/// action, a park, nor a torque off/on cycle stops lease renewal.
#[tokio::test]
async fn ws_heartbeats_renew_lease_across_jog_unload_torque_cycles() {
    use futures_util::{SinkExt, StreamExt};
    use tokio_tungstenite::tungstenite::Message;
    let lease_ms = 300u64;
    let supervisor = Arc::new(ArmSupervisor::new({ let mut c = sim_config(true); c.lease_timeout_ms = lease_ms; c }));
    let addr = serve(supervisor.clone()).await;
    let mut ws = ws_lease(addr).await;
    let mut request_id = 0u64;
    let generation = supervisor.inner().read().await.active_lease.as_ref().unwrap().generation;
    let mut command = async |ws: &mut _, path: &str, body: serde_json::Value| {
        let ws: &mut tokio_tungstenite::WebSocketStream<_> = ws;
        request_id += 1;
        let epoch = supervisor.inner().read().await.safety.epoch_id();
        let msg = serde_json::json!({"type":"command", "request_id":request_id, "path":path, "body":body,
            "generation":generation, "epoch_id":epoch, "client_id":"sdk"});
        ws.send(Message::Text(msg.to_string().into())).await.unwrap();
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: serde_json::Value = serde_json::from_str(&text).unwrap();
            if v["type"] == "command_ack" || v["type"] == "command_reject" {
                assert_eq!(v["type"], "command_ack", "{path}: {v}");
                return;
            }
        }
    };
    // Heartbeat + tick for `secs` of wall time (lease timeout is real time).
    async fn run(ws: &mut tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>, supervisor: &ArmSupervisor, generation: u64, secs: f64) {
        use futures_util::SinkExt;
        let end = Instant::now() + Duration::from_secs_f64(secs);
        while Instant::now() < end {
            let epoch = supervisor.inner().read().await.safety.epoch_id();
            let hb = serde_json::json!({"type":"heartbeat", "client_id":"sdk", "generation":generation, "epoch_id":epoch});
            ws.send(tokio_tungstenite::tungstenite::Message::Text(hb.to_string().into())).await.unwrap();
            for _ in 0..3 { supervisor.tick(0.05).await; tokio::time::sleep(Duration::from_millis(16)).await; }
            let inner = supervisor.inner().read().await;
            assert!(inner.watchdog_park_deadline.is_none() && !inner.safety.state().is_stopped(),
                "lease watchdog fired: {:?}", inner.safety.state());
        }
    }
    for _ in 0..2 {
        command(&mut ws, "/api/v1/torque", serde_json::json!({"enable":true})).await;
        command(&mut ws, "/api/v1/motion/jog", serde_json::json!({"target_positions":[1.0, 0.5], "duration_secs":0.5})).await;
        run(&mut ws, &supervisor, generation, 1.0).await;
        command(&mut ws, "/api/v1/motion/unload", serde_json::json!({})).await;
        run(&mut ws, &supervisor, generation, 2.0).await;
        assert!(!supervisor.get_observation().await.is_torqued, "unload ends with torque off");
    }
    command(&mut ws, "/api/v1/torque", serde_json::json!({"enable":true})).await;
    command(&mut ws, "/api/v1/motion/jog", serde_json::json!({"target_positions":[1.0, 0.5], "duration_secs":0.5})).await;
    run(&mut ws, &supervisor, generation, 1.5).await;
    assert!(supervisor.inner().read().await.is_lease_active());
}
