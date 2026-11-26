//! Adapter 3 dual-owner proof suite: mechanical gripper zero check (`/api/v1/action/gripper_zero_check`).
//!
//! Verifies identical WS + REST behavior across BOTH `RtOwner::Legacy` and `RtOwner::Executor`
//! (with `TimedAlloc` proving 0 heap allocations and 0 heap frees on `armd-control`):
//! 1. `store_offset=false` pass/fail threshold, `stop_rad`/`offset_rad`/`pass`, `gripper_zero.state`.
//! 2. `store_offset=true` PR C disk persistence, policy shift, `gripper_calib_revision` bump,
//!    `durability_confirmed`, `GET /api/v1/gripper/calibration` parity, and `prep_env` refresh.
//! 3. `store_offset=true` sanity refusal (`|offset_rad| > 0.5`): `done` + `pass=false`, no write.
//! 4. Conditional operator auth parity on BOTH WS and REST (`store_offset=true` vs `false`,
//!    auth enforced vs disabled) plus mid-motion operator lock revocation.
//! 5. Refusal parity (`NOT_ARMED`, `OBJECT_HELD`, `UNSUPPORTED`, `INVALID_COMMAND`, `LEASE_LOST`,
//!    `EPOCH_MISMATCH`).
//! 6. Motion safety parity: torque cap, seek speed limit, and `GRIPPER_TIMEOUT` (not preempted by
//!    PR0B session `child_deadline`).
//! 7. Executor Stop / E-Stop / Lease-Loss cancellation (holds measured jaw, clears session slot).
//! 8. Executor PR C boundaries: pre-rename revocation, dir-fsync `EIO` -> `PERSIST_FAILED` +
//!    `durability_confirmed=false` + `durability_warning` + background reconciliation retry, and
//!    worker crash after reservation.
//! 9. Stream saturation (`fail_zero_check_stream_push_once`), 20x repeated failure zero-leak proof
//!    (`zero_check_context_count() == 0`), and `PlanSlot` size check.

mod common;

#[global_allocator]
static ALLOC: armd::diagnostics::alloc::TimedAlloc = armd::diagnostics::alloc::TimedAlloc;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures_util::StreamExt;
use serde_json::{json, Value};
use tokio::net::TcpListener;
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::client::IntoClientRequest;
use tokio_tungstenite::tungstenite::http::header::AUTHORIZATION;

use armd::api::create_router_with_auth;
use armd::auth::OperatorAuth;
use armd::diagnostics::alloc::AllocCounts;
use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, TorqueOutcome,
    TorqueProgress,
};
use armd::model::{ArmConfig, JointState};
use armd::rt_core::config_iface::{ConfigDomain, HoldState};
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks, RtOwner};
use armd::rt_core::ops::PlanSlot;
use armd::rt_core::requests::config::{ConfigReply, ConfigRequest};
use armd::supervisor::{HeldObject as SupHeldObject, OWNER_EXECUTOR};
use armd::ArmSupervisor;
use common::{send, ws_reply, TestSocket, INSPECT};

static NEXT_REQ_ID: AtomicU64 = AtomicU64::new(100);

fn next_req_id() -> u64 {
    NEXT_REQ_ID.fetch_add(1, Ordering::Relaxed)
}

async fn connect_ws(addr: SocketAddr, bearer: Option<&str>) -> TestSocket {
    let mut req = format!("ws://{addr}/api/v1/ws").into_client_request().unwrap();
    if let Some(tok) = bearer {
        req.headers_mut().insert(
            AUTHORIZATION,
            format!("Bearer {tok}").parse().unwrap(),
        );
    }
    let (ws, _) = connect_async(req).await.unwrap();
    ws
}

async fn acquire_lease(ws: &mut TestSocket, client_id: &str, force: bool) -> u64 {
    let reply = ws_reply(
        ws,
        json!({"type": "acquire_lease", "client_id": client_id, "force": force, "seq": 1}),
        &["lease_granted"],
    )
    .await;
    reply["generation"].as_u64().unwrap()
}

async fn send_cmd(
    ws: &mut TestSocket,
    generation: u64,
    epoch: u64,
    path: &str,
    body: Value,
) -> Value {
    let req_id = next_req_id();
    ws_reply(
        ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": generation,
            "epoch_id": epoch,
            "path": path,
            "body": body,
        }),
        &["command_ack", "command_reject"],
    )
    .await
}

/// Sends a WS command, verifies `command_ack` arrives strictly BEFORE the action's terminal,
/// and returns `(ack, action_terminal, full_terminal_obs)`.
async fn run_cmd_to_terminal(
    ws: &mut TestSocket,
    generation: u64,
    epoch: u64,
    path: &str,
    body: Value,
    timeout: Duration,
) -> (Value, Value, Value) {
    let req_id = next_req_id();
    send(
        ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": generation,
            "epoch_id": epoch,
            "path": path,
            "body": body,
        }),
    )
    .await;

    let mut ack: Option<Value> = None;
    let mut ack_action_id: Option<u64> = None;
    let mut early_terminals: Vec<u64> = Vec::new();

    let (term_action, term_obs) = tokio::time::timeout(timeout, async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("type").is_some() {
                if v["request_id"].as_u64() == Some(req_id) {
                    assert_eq!(
                        v["type"],
                        json!("command_ack"),
                        "expected command_ack on {path}, got {v}"
                    );
                    let aid = v["action_id"].as_u64().expect("command_ack missing action_id");
                    assert!(
                        !early_terminals.contains(&aid),
                        "terminal for action {aid} arrived before command_ack"
                    );
                    ack_action_id = Some(aid);
                    ack = Some(v);
                }
            } else if let Some(a) = v.get("action").filter(|a| !a.is_null()) {
                if a["state"] != "running" {
                    if let Some(id) = a["id"].as_u64() {
                        if ack_action_id == Some(id) {
                            return (a.clone(), v);
                        }
                        if ack_action_id.is_none() {
                            early_terminals.push(id);
                        }
                    }
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for terminal on {path}"));

    (ack.unwrap(), term_action, term_obs)
}

async fn wait_terminal_for_id(ws: &mut TestSocket, action_id: u64, timeout: Duration) -> Value {
    tokio::time::timeout(timeout, async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("type").is_none() {
                if let Some(a) = v.get("action").filter(|a| !a.is_null()) {
                    if a["id"].as_u64() == Some(action_id) && a["state"] != "running" {
                        return a.clone();
                    }
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for terminal of action {action_id}"))
}

struct DualRig {
    owner: RtOwner,
    sup: Arc<ArmSupervisor>,
    addr: SocketAddr,
    _dir: tempfile::TempDir,
    calib_path: PathBuf,
    ticker: Option<tokio::task::JoinHandle<()>>,
    server: tokio::task::JoinHandle<()>,
}

impl DualRig {
    async fn start(
        owner: RtOwner,
        auth: OperatorAuth,
        configure_cfg: impl FnOnce(&mut ArmConfig),
        configure_inner: impl FnOnce(&mut armd::SupervisorInner),
        hooks: PersistTestHooks,
    ) -> Self {
        Self::start_with_calib(owner, auth, configure_cfg, configure_inner, hooks, None).await
    }

    /// As [`Self::start`], with `initial_calib` written to `gripper_calib.json` before it loads.
    async fn start_with_calib(
        owner: RtOwner,
        auth: OperatorAuth,
        configure_cfg: impl FnOnce(&mut ArmConfig),
        configure_inner: impl FnOnce(&mut armd::SupervisorInner),
        hooks: PersistTestHooks,
        initial_calib: Option<&str>,
    ) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let calib_path = dir.path().join("gripper_calib.json");
        if let Some(text) = initial_calib {
            std::fs::write(&calib_path, text).unwrap();
        }

        let mut cfg = ArmConfig::rebot_dm();
        cfg.collision = None;
        cfg.lease_timeout_ms = 60_000;
        configure_cfg(&mut cfg);

        let sup = Arc::new(ArmSupervisor::new(cfg));
        sup.set_operator_auth_enforced(auth.is_enforced()).await;
        armd::gripper_calib::load_into(&sup, &calib_path).await;
        {
            let mut inner = sup.inner().write().await;
            let mut q = INSPECT;
            q[6] = -0.15;
            inner.measured_positions = q.to_vec();
            inner.commanded_positions = q.to_vec();
            inner.target_positions = q.to_vec();
            configure_inner(&mut inner);
        }

        let ticker = match owner {
            RtOwner::Executor => {
                let mut ecfg = ExecutorConfig::for_tests(Duration::from_millis(5));
                ecfg.alloc_timing = true;
                let _handle = sup
                    .start_executor_with_hooks(ecfg, hooks)
                    .await
                    .expect("start RT executor");
                let dl = Instant::now() + Duration::from_secs(3);
                while sup
                    .hub()
                    .latest()
                    .is_none_or(|s| s.rt.owner.boot.is_zero())
                {
                    assert!(Instant::now() < dl, "executor did not publish initial snapshot");
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
                None
            }
            RtOwner::Legacy => {
                let tick_sup = Arc::clone(&sup);
                Some(tokio::spawn(async move {
                    let mut iv = tokio::time::interval(Duration::from_millis(20));
                    loop {
                        iv.tick().await;
                        tick_sup.tick(0.02).await;
                    }
                }))
            }
        };

        let router = create_router_with_auth(Arc::clone(&sup), auth);
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let server = tokio::spawn(async move {
            let _ = axum::serve(listener, router).await;
        });

        // Wait until at least one control tick has ingested feedback (needed when a driver is attached).
        let fb_dl = Instant::now() + Duration::from_secs(2);
        while !sup.get_observation().await.feedback_valid {
            assert!(Instant::now() < fb_dl, "{owner:?}: feedback never became valid");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }

        Self {
            owner,
            sup,
            addr,
            _dir: dir,
            calib_path,
            ticker,
            server,
        }
    }

    async fn wait_zero_check_ctx_empty(&self, label: &str) {
        if let Some(h) = self.sup.executor_handle() {
            let dl = Instant::now() + Duration::from_secs(2);
            while h.zero_check_context_count() != 0 {
                if Instant::now() >= dl {
                    panic!(
                        "{label}: expected zero_check_context_count == 0, got {}",
                        h.zero_check_context_count()
                    );
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }
    }

    fn assert_zero_rt_allocs(&self, label: &str) {
        if let Some(h) = self.sup.executor_handle() {
            assert_eq!(
                h.stats().total_allocs(),
                AllocCounts::ZERO,
                "{label}: armd-control performed heap operations: {:?}",
                h.stats().total_allocs()
            );
        }
    }

    async fn shutdown(self) {
        if let Some(t) = self.ticker {
            t.abort();
        }
        self.server.abort();
        if self.owner == RtOwner::Executor {
            if let Some(h) = self.sup.executor_handle() {
                h.stop_and_join();
            }
        }
    }
}

/// Controllable RT-capable mock driver for torque cap, stall, timeout, and cancellation tests.
#[derive(Clone)]
struct ZeroCheckDriver {
    state: Arc<Mutex<ZeroCheckDriverState>>,
}

struct ZeroCheckDriverState {
    q: [f32; 7],
    rx_at: [Option<Instant>; 7],
    seqs: [u64; 7],
    torqued: bool,
    /// When `Some(stop_q)`, clamps J7 at `stop_q` and reports `stall_torque_nm` once commanded pushes past it.
    stall_stop_q: Option<f32>,
    stall_torque_nm: f32,
    /// When `true`, J7 slowly moves at 0.01 rad/tick toward open without ever stalling -> triggers `GRIPPER_TIMEOUT`.
    never_stall_creep: bool,
    /// Maximum observed absolute commanded J7 velocity step (rad/s) during seek.
    max_j7_step_rad: f32,
    last_written_j7: Option<f32>,
    /// History of written J7 targets after recording is enabled.
    recorded_j7_targets: Vec<f32>,
    /// Write instants matching `recorded_j7_targets`.
    recorded_j7_at: Vec<Instant>,
    record_targets: bool,
    /// When `Some((age, cached_q))`, holds J7 feedback stale by `age` and reports `cached_q`
    /// without advancing `rx_at[6]`, testing E-stop freshness preservation.
    stale_j7: Option<(Duration, f32)>,
    /// Joint temperature reported for J7 (°C).
    j7_temperature_c: f32,
    /// When `true`, ages all joint feedback timestamps by 2s to trigger `BusLossFallback`.
    bus_loss_stale: bool,
}

impl ZeroCheckDriver {
    fn new(start_j7: f32) -> Self {
        let mut q = INSPECT;
        q[6] = start_j7;
        let now = Instant::now();
        Self {
            state: Arc::new(Mutex::new(ZeroCheckDriverState {
                q,
                rx_at: [Some(now); 7],
                seqs: [1; 7],
                torqued: false,
                stall_stop_q: None,
                stall_torque_nm: 0.45,
                never_stall_creep: false,
                max_j7_step_rad: 0.0,
                last_written_j7: None,
                recorded_j7_targets: Vec::with_capacity(512),
                recorded_j7_at: Vec::with_capacity(512),
                record_targets: false,
                stale_j7: None,
                j7_temperature_c: 30.0,
                bus_loss_stale: false,
            })),
        }
    }
}

impl ArmDriver for ZeroCheckDriver {
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }

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
        if self.state.lock().unwrap().torqued {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }

    fn is_feedback_valid(&self) -> bool {
        true
    }

    fn is_enabled(&self) -> bool {
        self.state.lock().unwrap().torqued
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let s = self.state.lock().unwrap();
        let n = Instant::now();
        s.rx_at.iter().map(|t| t.map(|t| n.saturating_duration_since(t))).collect()
    }

    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        self.state.lock().unwrap().rx_at.to_vec()
    }

    fn feedback_seqs(&self) -> Vec<u64> {
        self.state.lock().unwrap().seqs.to_vec()
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        self.state.lock().unwrap().rx_at.to_vec()
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let s = self.state.lock().unwrap();
        let n = Instant::now();
        out.clear();
        for t in &s.rx_at {
            out.push(t.map(|t| n.saturating_duration_since(t)));
        }
    }

    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        let s = self.state.lock().unwrap();
        let n = Instant::now();
        dof == 7 && s.rx_at.iter().all(|t| t.is_some_and(|t| n.saturating_duration_since(t) <= max_age))
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let s = self.state.lock().unwrap();
        let n = Instant::now();
        out.clear();
        for t in &s.rx_at {
            out.push(t.map(|t| n.saturating_duration_since(t).as_millis() as u64));
        }
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let s = self.state.lock().unwrap();
        out.clear();
        out.extend_from_slice(&s.rx_at);
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        let s = self.state.lock().unwrap();
        out.clear();
        out.extend_from_slice(&s.seqs);
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let s = self.state.lock().unwrap();
        out.clear();
        out.extend_from_slice(&s.rx_at);
    }

    fn fresh_joint_positions_into(
        &self,
        indices: &[usize],
        max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        if out.len() < indices.len() {
            return false;
        }
        let s = self.state.lock().unwrap();
        let now = Instant::now();
        for (slot, &i) in out.iter_mut().zip(indices.iter()) {
            let Some(&p) = s.q.get(i) else { return false };
            let Some(ts) = s.rx_at.get(i).and_then(|t| *t) else {
                return false;
            };
            if now.saturating_duration_since(ts) > max_age {
                return false;
            }
            *slot = p;
        }
        true
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(7);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let mut s = self.state.lock().unwrap();
        let now = Instant::now();
        let j7_torque = if let Some(stop_q) = s.stall_stop_q {
            if s.last_written_j7.is_some_and(|t| t >= stop_q - 1e-4) && (s.q[6] - stop_q).abs() <= 0.02 {
                s.stall_torque_nm
            } else {
                0.02
            }
        } else {
            0.02
        };
        out.clear();
        for i in 0..7 {
            if s.bus_loss_stale {
                s.rx_at[i] = Some(now - Duration::from_secs(2));
            } else if i == 6 {
                if let Some((age, cached_q)) = s.stale_j7 {
                    s.rx_at[6] = Some(now - age);
                    s.q[6] = cached_q;
                } else {
                    s.rx_at[6] = Some(now);
                    s.seqs[6] += 1;
                }
            } else {
                s.rx_at[i] = Some(now);
                s.seqs[i] += 1;
            }
            out.push(JointState {
                position: s.q[i],
                velocity: 0.0,
                torque: if i == 6 { j7_torque } else { 0.0 },
                temperature: if i == 6 { s.j7_temperature_c } else { 30.0 },
            });
        }
        Ok(())
    }

    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _deadline: Instant,
        _max_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((7, false))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        let s = self.state.lock().unwrap();
        let j7_torque = if let Some(stop_q) = s.stall_stop_q {
            if s.last_written_j7.is_some_and(|t| t >= stop_q - 1e-4) && (s.q[6] - stop_q).abs() <= 0.02 {
                s.stall_torque_nm
            } else {
                0.02
            }
        } else {
            0.02
        };
        out.clear();
        for i in 0..7 {
            out.push(JointState {
                position: s.q[i],
                velocity: 0.0,
                torque: if i == 6 { j7_torque } else { 0.0 },
                temperature: 30.0,
            });
        }
        Some(())
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.state.lock().unwrap();
        for (i, t) in targets.iter().enumerate().take(6) {
            s.q[i] = t.position;
        }
        if let Some(t7) = targets.get(6) {
            if let Some(prev) = s.last_written_j7 {
                let step = (t7.position - prev).abs();
                if step > s.max_j7_step_rad {
                    s.max_j7_step_rad = step;
                }
            }
            s.last_written_j7 = Some(t7.position);
            if s.record_targets && s.recorded_j7_targets.len() < s.recorded_j7_targets.capacity() {
                s.recorded_j7_targets.push(t7.position);
                s.recorded_j7_at.push(Instant::now());
            }
            if let Some((_, cached_q)) = s.stale_j7 {
                s.q[6] = cached_q;
            } else if s.never_stall_creep {
                // Oscillate above the stillness-stall threshold without reaching target or
                // making net closing progress -> triggers `GRIPPER_TIMEOUT` right at `timeout_s`.
                s.q[6] = if s.seqs[6].is_multiple_of(2) { -0.08 } else { -0.02 };
            } else if let Some(stop_q) = s.stall_stop_q {
                s.q[6] = t7.position.min(stop_q);
            } else {
                s.q[6] = t7.position;
            }
        }
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.state.lock().unwrap().torqued = enabled;
        Ok(())
    }

    fn begin_torque_budgeted(
        &mut self,
        enabled: bool,
        _max_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.state.lock().unwrap().torqued = enabled;
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }

    fn poll_torque_budgeted(
        &mut self,
        _deadline: Instant,
        _max_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
}

// ---------------------------------------------------------------------------
// 1. Dual-owner WS parity: store_offset=false (pass & fail threshold)
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_pass_and_fail_no_store() {
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        // Case A: stall at +0.05 rad -> pass=true (threshold 0.10)
        let rig = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            PersistTestHooks::default(),
        )
        .await;
        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-pass", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;

        let (ack, term, obs) = run_cmd_to_terminal(
            &mut ws,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": false}),
            Duration::from_secs(6),
        )
        .await;
        assert_eq!(ack["status"], "accepted", "{owner:?}: {ack}");
        assert_eq!(term["kind"], "gripper_zero_check", "{owner:?}: {term}");
        assert_eq!(term["state"], "done", "{owner:?}: {term}");
        assert!(term.get("code").is_none_or(Value::is_null), "{owner:?}: {term}");
        assert!(
            (term["stop_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {term}"
        );
        assert!(
            (term["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {term}"
        );
        assert_eq!(term["pass"], json!(true), "{owner:?}: {term}");
        assert!(
            term.get("durability_confirmed").is_none_or(Value::is_null),
            "{owner:?}: {term}"
        );
        assert_eq!(obs["gripper_zero"]["state"], "checked", "{owner:?}: {obs}");
        assert!(
            (obs["gripper_zero"]["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {obs}"
        );
        assert!(!rig.calib_path.exists(), "{owner:?}: store_offset=false must not write file");
        rig.wait_zero_check_ctx_empty("pass_no_store").await;
        rig.assert_zero_rt_allocs("pass_no_store");
        rig.shutdown().await;

        // Case B: stall at -0.30 rad -> pass=false (threshold 0.10)
        let rig2 = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.measured_positions[6] = -0.50;
                inner.commanded_positions[6] = -0.50;
                inner.target_positions[6] = -0.50;
                inner.set_mock_gripper_stall_offset(Some(-0.30));
            },
            PersistTestHooks::default(),
        )
        .await;
        let mut ws2 = connect_ws(rig2.addr, None).await;
        let gen2 = acquire_lease(&mut ws2, "c-fail", false).await;
        let epoch2 = rig2.sup.get_observation().await.epoch_id;
        let ack_t2 = send_cmd(&mut ws2, gen2, epoch2, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t2["type"], "command_ack", "{owner:?}: {ack_t2}");
        let epoch2 = rig2.sup.get_observation().await.epoch_id;

        let (_, term2, _) = run_cmd_to_terminal(
            &mut ws2,
            gen2,
            epoch2,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": false}),
            Duration::from_secs(6),
        )
        .await;
        assert_eq!(term2["state"], "done", "{owner:?}: {term2}");
        assert!(
            (term2["stop_rad"].as_f64().unwrap() - (-0.30)).abs() < 1e-2,
            "{owner:?}: {term2}"
        );
        assert!(
            (term2["offset_rad"].as_f64().unwrap() - (-0.30)).abs() < 1e-2,
            "{owner:?}: {term2}"
        );
        assert_eq!(term2["pass"], json!(false), "{owner:?}: {term2}");
        assert!(!rig2.calib_path.exists());
        rig2.wait_zero_check_ctx_empty("fail_no_store").await;
        rig2.assert_zero_rt_allocs("fail_no_store");
        rig2.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 2. Dual-owner WS parity: store_offset=true persists, shifts policy, bumps rev, refreshes prep_env
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_store_offset_persists_shifts_and_refreshes_prep_env() {
    let client = reqwest::Client::new();
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        let rig = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            PersistTestHooks::default(),
        )
        .await;

        let cal_before: Value = client
            .get(format!("http://{}/api/v1/gripper/calibration", rig.addr))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let rev_before = cal_before["revision"].as_u64().unwrap();

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-store", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": true}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term["state"], "done", "{owner:?}: {term}");
        assert_eq!(term["pass"], json!(true), "{owner:?}: {term}");
        assert!(
            (term["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {term}"
        );
        assert_eq!(term["durability_confirmed"], json!(true), "{owner:?}: {term}");
        assert!(
            term.get("durability_warning").is_none_or(Value::is_null),
            "{owner:?}: {term}"
        );

        assert!(rig.calib_path.exists(), "{owner:?}: gripper_calib.json must be written");
        assert!(
            !armd::gripper_calib::pending_path_for(&rig.calib_path).exists(),
            "{owner:?}: no candidate may remain after a successful store"
        );
        let on_disk: Value =
            serde_json::from_str(&std::fs::read_to_string(&rig.calib_path).unwrap()).unwrap();
        assert!(
            (on_disk["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {on_disk}"
        );

        // Wait briefly for PublishedConfig / prep_env update on Executor.
        let dl = Instant::now() + Duration::from_secs(2);
        let cal_after = loop {
            let v: Value = client
                .get(format!("http://{}/api/v1/gripper/calibration", rig.addr))
                .send()
                .await
                .unwrap()
                .json()
                .await
                .unwrap();
            if v["revision"].as_u64() == Some(rev_before + 1) {
                break v;
            }
            assert!(
                Instant::now() < dl,
                "{owner:?}: GET /api/v1/gripper/calibration revision did not bump: {v}"
            );
            tokio::time::sleep(Duration::from_millis(10)).await;
        };
        assert!(
            (cal_after["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {cal_after}"
        );
        assert!(
            (cal_after["policy"]["closed_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
            "{owner:?}: {cal_after}"
        );
        assert!(
            (cal_after["policy"]["open_rad"].as_f64().unwrap() - (-5.65)).abs() < 1e-2,
            "{owner:?}: {cal_after}"
        );

        // Second check immediately after persist proves prep_env & session slot are fresh.
        let epoch2 = rig.sup.get_observation().await.epoch_id;
        let (_, term2, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            epoch2,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": false}),
            Duration::from_secs(6),
        )
        .await;
        assert_eq!(term2["state"], "done", "{owner:?}: second check failed: {term2}");

        rig.wait_zero_check_ctx_empty("store_offset").await;
        rig.assert_zero_rt_allocs("store_offset");
        rig.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 3. Dual-owner WS parity: |offset_rad| > 0.5 refuses store without error
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_large_offset_refuses_store() {
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        let rig = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.measured_positions[6] = -0.80;
                inner.commanded_positions[6] = -0.80;
                inner.target_positions[6] = -0.80;
                inner.set_mock_gripper_stall_offset(Some(-0.60));
            },
            PersistTestHooks::default(),
        )
        .await;
        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-large", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": true}),
            Duration::from_secs(6),
        )
        .await;
        assert_eq!(term["state"], "done", "{owner:?}: {term}");
        assert_eq!(term["pass"], json!(false), "{owner:?}: {term}");
        assert!(
            (term["offset_rad"].as_f64().unwrap() - (-0.60)).abs() < 1e-2,
            "{owner:?}: {term}"
        );
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(!rig.calib_path.exists(), "{owner:?}: |offset| > 0.5 must not persist");
        rig.wait_zero_check_ctx_empty("large_offset").await;
        rig.assert_zero_rt_allocs("large_offset");
        rig.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 4. Dual-owner conditional operator auth parity (WS + REST)
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_conditional_operator_auth_ws_and_rest_parity() {
    let client = reqwest::Client::new();
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        let auth = OperatorAuth::enforced("123456", 1800);
        let rig = DualRig::start(
            owner,
            auth,
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.04));
            },
            PersistTestHooks::default(),
        )
        .await;
        let base = format!("http://{}", rig.addr);

        // REST Program with store_offset=true -> 403 OPERATOR_REQUIRED
        let r_prog_store = client
            .post(format!("{base}/api/v1/action/gripper_zero_check"))
            .json(&json!({"store_offset": true}))
            .send()
            .await
            .unwrap();
        assert_eq!(r_prog_store.status(), 403, "{owner:?}");
        let r_prog_store_body: Value = r_prog_store.json().await.unwrap();
        assert_eq!(r_prog_store_body["code"], "OPERATOR_REQUIRED", "{owner:?}: {r_prog_store_body}");

        // REST Program with store_offset=false -> 409 LEASE_REQUIRED (never 503 EXECUTOR_ROUTING_PENDING)
        let r_prog_check = client
            .post(format!("{base}/api/v1/action/gripper_zero_check"))
            .json(&json!({"store_offset": false}))
            .send()
            .await
            .unwrap();
        assert_eq!(r_prog_check.status(), 409, "{owner:?}");
        let r_prog_check_body: Value = r_prog_check.json().await.unwrap();
        assert!(
            matches!(r_prog_check_body["code"].as_str(), Some("LEASE_REQUIRED" | "LEASE_ACTIVE")),
            "{owner:?}: {r_prog_check_body}"
        );

        // Unlock operator session
        let unlock: Value = client
            .post(format!("{base}/api/v1/auth/unlock"))
            .json(&json!({"pin": "123456"}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let token = unlock["token"].as_str().unwrap().to_string();

        // REST Operator with store_offset=true -> 409 LEASE_REQUIRED (passes auth, hits lease gate)
        let r_op_store = client
            .post(format!("{base}/api/v1/action/gripper_zero_check"))
            .header(AUTHORIZATION.as_str(), format!("Bearer {token}"))
            .json(&json!({"store_offset": true}))
            .send()
            .await
            .unwrap();
        assert_eq!(r_op_store.status(), 409, "{owner:?}");

        // WS Program client: denied store_offset=true, allowed store_offset=false
        let mut ws_prog = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws_prog, "prog-cli", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack_t = send_cmd(&mut ws_prog, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;

        let rej = send_cmd(
            &mut ws_prog,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
        )
        .await;
        assert_eq!(rej["type"], "command_reject", "{owner:?}: {rej}");
        assert_eq!(rej["status"], 403, "{owner:?}: {rej}");
        assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{owner:?}: {rej}");

        let (_, term_prog, _) = run_cmd_to_terminal(
            &mut ws_prog,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": false}),
            Duration::from_secs(6),
        )
        .await;
        assert_eq!(term_prog["state"], "done", "{owner:?}: {term_prog}");
        assert!(!rig.calib_path.exists(), "{owner:?}");
        drop(ws_prog);

        // WS Operator client: allowed store_offset=true and persists
        let mut ws_op = connect_ws(rig.addr, Some(&token)).await;
        let gen_op = acquire_lease(&mut ws_op, "op-cli", true).await;
        let epoch_op = rig.sup.get_observation().await.epoch_id;
        let (_, term_op, _) = run_cmd_to_terminal(
            &mut ws_op,
            gen_op,
            epoch_op,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true, "threshold_rad": 0.10}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term_op["state"], "done", "{owner:?}: {term_op}");
        assert_eq!(term_op["durability_confirmed"], json!(true), "{owner:?}: {term_op}");
        assert!(rig.calib_path.exists(), "{owner:?}");

        rig.wait_zero_check_ctx_empty("conditional_auth").await;
        rig.assert_zero_rt_allocs("conditional_auth");
        rig.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 5. Dual-owner refusals parity
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_refusals_parity() {
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        // (a) NOT_ARMED, INVALID_COMMAND, LEASE_LOST, EPOCH_MISMATCH, OBJECT_HELD
        let rig = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            |_| {},
            PersistTestHooks::default(),
        )
        .await;
        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-refuse", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;

        // NOT_ARMED before torque enable
        let r_not_armed = send_cmd(
            &mut ws,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({}),
        )
        .await;
        assert_eq!(r_not_armed["type"], "command_reject", "{owner:?}: {r_not_armed}");
        assert_eq!(r_not_armed["code"], "NOT_ARMED", "{owner:?}: {r_not_armed}");

        // Enable torque
        let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;

        // INVALID_COMMAND: non-positive threshold_rad & non-object body
        for bad_body in [json!({"threshold_rad": 0.0}), json!({"threshold_rad": -0.2}), json!("bad")] {
            let r = send_cmd(&mut ws, gen, epoch, "/api/v1/action/gripper_zero_check", bad_body).await;
            assert_eq!(r["type"], "command_reject", "{owner:?}: {r}");
            assert_eq!(r["code"], "INVALID_COMMAND", "{owner:?}: {r}");
        }

        // LEASE_LOST: wrong generation
        let r_lease = send_cmd(
            &mut ws,
            gen + 99,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({}),
        )
        .await;
        assert_eq!(r_lease["type"], "command_reject", "{owner:?}: {r_lease}");
        assert_eq!(r_lease["code"], "LEASE_LOST", "{owner:?}: {r_lease}");

        // EPOCH_MISMATCH: wrong epoch
        let r_epoch = send_cmd(
            &mut ws,
            gen,
            epoch + 99,
            "/api/v1/action/gripper_zero_check",
            json!({}),
        )
        .await;
        assert_eq!(r_epoch["type"], "command_reject", "{owner:?}: {r_epoch}");
        assert_eq!(r_epoch["code"], "EPOCH_MISMATCH", "{owner:?}: {r_epoch}");

        rig.wait_zero_check_ctx_empty("refusals_a").await;
        rig.assert_zero_rt_allocs("refusals_a");
        rig.shutdown().await;

        // (b) OBJECT_HELD
        let rig_held = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.held = Some(SupHeldObject {
                    label: "cube".into(),
                    spill_safe: false,
                });
            },
            PersistTestHooks::default(),
        )
        .await;
        let mut ws_h = connect_ws(rig_held.addr, None).await;
        let gen_h = acquire_lease(&mut ws_h, "c-held", false).await;
        let ep_h = rig_held.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws_h, gen_h, ep_h, "/api/v1/torque", json!({"enable": true})).await;
        let ep_h = rig_held.sup.get_observation().await.epoch_id;
        let r_held = send_cmd(
            &mut ws_h,
            gen_h,
            ep_h,
            "/api/v1/action/gripper_zero_check",
            json!({}),
        )
        .await;
        assert_eq!(r_held["type"], "command_reject", "{owner:?}: {r_held}");
        assert_eq!(r_held["status"], 409, "{owner:?}: {r_held}");
        assert_eq!(r_held["code"], "OBJECT_HELD", "{owner:?}: {r_held}");
        rig_held.assert_zero_rt_allocs("refusals_held");
        rig_held.shutdown().await;

        // (c) UNSUPPORTED (zero_check_supported == false)
        let rig_unsup = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |cfg| {
                if let Some(p) = cfg.gripper_policy.as_mut() {
                    p.zero_check_supported = false;
                }
            },
            |_| {},
            PersistTestHooks::default(),
        )
        .await;
        let mut ws_u = connect_ws(rig_unsup.addr, None).await;
        let gen_u = acquire_lease(&mut ws_u, "c-unsup", false).await;
        let ep_u = rig_unsup.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws_u, gen_u, ep_u, "/api/v1/torque", json!({"enable": true})).await;
        let ep_u = rig_unsup.sup.get_observation().await.epoch_id;
        let r_unsup = send_cmd(
            &mut ws_u,
            gen_u,
            ep_u,
            "/api/v1/action/gripper_zero_check",
            json!({}),
        )
        .await;
        assert_eq!(r_unsup["type"], "command_reject", "{owner:?}: {r_unsup}");
        assert_eq!(r_unsup["code"], "UNSUPPORTED", "{owner:?}: {r_unsup}");
        rig_unsup.assert_zero_rt_allocs("refusals_unsup");
        rig_unsup.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 6. Dual-owner motion safety: torque cap, speed limit, and GRIPPER_TIMEOUT
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_timeout_and_torque_cap_speed_limit_parity() {
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        // (a) Torque-triggered stall + speed limit verification on RT driver
        let drv = ZeroCheckDriver::new(-0.30);
        drv.state.lock().unwrap().stall_stop_q = Some(0.03);
        let drv_clone = drv.clone();
        let rig = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            move |inner| {
                inner.measured_positions[6] = -0.30;
                inner.commanded_positions[6] = -0.30;
                inner.target_positions[6] = -0.30;
                inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv_clone)));
            },
            PersistTestHooks::default(),
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-drv", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;
        {
            let mut s = drv.state.lock().unwrap();
            s.recorded_j7_targets.clear();
            s.recorded_j7_at.clear();
            s.max_j7_step_rad = 0.0;
            s.record_targets = true;
        }

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": false}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term["state"], "done", "{owner:?}: {term}");
        assert!(
            (term["stop_rad"].as_f64().unwrap() - 0.03).abs() < 0.03,
            "{owner:?}: torque stall stop_rad mismatch: {term}"
        );
        assert_eq!(term["pass"], json!(true), "{owner:?}: {term}");
        // Seek speed is 0.3 rad/s (`start_gripper_zero_check`, wire velocity ceiling). Legacy ticks
        // with a fixed dt of 20 ms: every write steps at most 0.3 * 0.02 = 0.006 rad. The executor
        // runs a 5 ms period with monotonic-clock dt: a write steps at most 0.3 rad/s times the
        // time since the previous write, plus one period of clock skew.
        const SEEK_SPEED_RAD_S: f64 = 0.3;
        let nominal_dt_s = match owner {
            RtOwner::Legacy => 0.02,
            RtOwner::Executor => 0.005,
        };
        {
            let s = drv.state.lock().unwrap();
            let (t, at) = (&s.recorded_j7_targets, &s.recorded_j7_at);
            assert_eq!(t.len(), at.len());
            // Closing segment: up to the first retreat write (stall back-off, not seek motion).
            let end = (1..t.len()).find(|&i| t[i] < t[i - 1]).unwrap_or(t.len());
            assert!(end >= 10, "{owner:?}: too few seek writes recorded ({end})");
            for i in 1..end {
                let dq = f64::from(t[i] - t[i - 1]);
                let step_ceiling = match owner {
                    RtOwner::Legacy => SEEK_SPEED_RAD_S * nominal_dt_s,
                    RtOwner::Executor => {
                        SEEK_SPEED_RAD_S * (at[i].duration_since(at[i - 1]).as_secs_f64() + nominal_dt_s)
                    }
                } + 1e-5;
                assert!(
                    dq <= step_ceiling,
                    "{owner:?}: write {i} seek step {dq:.5} rad exceeds ceiling {step_ceiling:.5} rad"
                );
                // Position advances in step with elapsed time: never ahead of 0.3 rad/s.
                let elapsed = at[i].duration_since(at[0]).as_secs_f64();
                let advance = f64::from(t[i] - t[0]);
                assert!(
                    advance <= SEEK_SPEED_RAD_S * (elapsed + nominal_dt_s) * 1.05 + 1e-4,
                    "{owner:?}: write {i} advanced {advance:.4} rad in {elapsed:.3} s (> 0.3 rad/s)"
                );
            }
            let elapsed = at[end - 1].duration_since(at[0]).as_secs_f64();
            let rate = f64::from(t[end - 1] - t[0]) / elapsed;
            assert!(
                (0.8 * SEEK_SPEED_RAD_S..=1.05 * SEEK_SPEED_RAD_S).contains(&rate),
                "{owner:?}: mean seek rate {rate:.3} rad/s not in step with 0.3 rad/s"
            );
        }
        rig.assert_zero_rt_allocs("torque_cap_speed_limit");
        rig.shutdown().await;

        // (b) GRIPPER_TIMEOUT (short travel from -0.05 -> timeout_s ~ 3.0s; session child_deadline must NOT preempt it)
        let drv_to = ZeroCheckDriver::new(-0.05);
        drv_to.state.lock().unwrap().never_stall_creep = true;
        let drv_to_clone = drv_to.clone();
        let rig_to = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            move |inner| {
                inner.measured_positions[6] = -0.05;
                inner.commanded_positions[6] = -0.05;
                inner.target_positions[6] = -0.05;
                inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv_to_clone)));
            },
            PersistTestHooks::default(),
        )
        .await;
        let mut ws_to = connect_ws(rig_to.addr, None).await;
        let gen_to = acquire_lease(&mut ws_to, "c-to", false).await;
        let ep_to = rig_to.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws_to, gen_to, ep_to, "/api/v1/torque", json!({"enable": true})).await;
        let ep_to = rig_to.sup.get_observation().await.epoch_id;

        let (_, term_to, _) = run_cmd_to_terminal(
            &mut ws_to,
            gen_to,
            ep_to,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": false}),
            Duration::from_secs(12),
        )
        .await;
        assert_eq!(term_to["state"], "failed", "{owner:?}: {term_to}");
        assert_eq!(term_to["code"], "GRIPPER_TIMEOUT", "{owner:?}: {term_to}");
        rig_to.assert_zero_rt_allocs("gripper_timeout");
        rig_to.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 6b. Obstructed seek: current contract (legacy parity) accepts the obstruction stall as zero
// ---------------------------------------------------------------------------

/// Pins the 1.6.1 contract: an obstruction that stalls the seek before the jaw stop ends the
/// check as `done` with the obstruction position as `stop_rad` / `offset_rad` (here inside
/// `threshold_rad`, so `pass=true`). Both owners must agree.
/// See the `FOLLOW-UP:` note at the completion site in `SupervisorInner::finish_action`.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn dual_owner_zero_check_obstructed_seek_stall_accepted_parity() {
    // Obstruction 0.08 rad before the jaw stop: inside `threshold_rad`, so it also passes.
    const OBSTRUCTION_Q: f32 = -0.08;
    let mut results: Vec<(RtOwner, Value)> = Vec::new();
    for owner in [RtOwner::Legacy, RtOwner::Executor] {
        let drv = ZeroCheckDriver::new(-0.30);
        drv.state.lock().unwrap().stall_stop_q = Some(OBSTRUCTION_Q);
        let drv_clone = drv.clone();
        let rig = DualRig::start(
            owner,
            OperatorAuth::disabled(),
            |_| {},
            move |inner| {
                inner.measured_positions[6] = -0.30;
                inner.commanded_positions[6] = -0.30;
                inner.target_positions[6] = -0.30;
                inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv_clone)));
            },
            PersistTestHooks::default(),
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-obstruct", false).await;
        let epoch = rig.sup.get_observation().await.epoch_id;
        let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
        assert_eq!(ack_t["type"], "command_ack", "{owner:?}: {ack_t}");
        let epoch = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            epoch,
            "/api/v1/action/gripper_zero_check",
            json!({"threshold_rad": 0.10, "store_offset": false}),
            Duration::from_secs(8),
        )
        .await;
        // Current contract: the obstruction stall is accepted (not CALIBRATION_OBSTRUCTED).
        assert_eq!(term["state"], "done", "{owner:?}: {term}");
        assert!(term["code"].is_null(), "{owner:?}: {term}");
        assert!(
            (term["stop_rad"].as_f64().unwrap() - f64::from(OBSTRUCTION_Q)).abs() < 0.03,
            "{owner:?}: stop_rad must be the obstruction position: {term}"
        );
        assert!(
            (term["offset_rad"].as_f64().unwrap() - f64::from(OBSTRUCTION_Q)).abs() < 0.03,
            "{owner:?}: {term}"
        );
        assert_eq!(term["pass"], json!(true), "{owner:?}: {term}");
        rig.wait_zero_check_ctx_empty("obstructed_seek").await;
        rig.assert_zero_rt_allocs("obstructed_seek");
        rig.shutdown().await;
        results.push((owner, term));
    }
    let (_, legacy) = &results[0];
    let (_, exec) = &results[1];
    for key in ["state", "code", "pass"] {
        assert_eq!(legacy[key], exec[key], "{key}: legacy {legacy} vs executor {exec}");
    }
    for key in ["stop_rad", "offset_rad"] {
        let (l, e) = (legacy[key].as_f64().unwrap(), exec[key].as_f64().unwrap());
        assert!((l - e).abs() < 0.01, "{key}: legacy {l} vs executor {e}");
    }
}

// ---------------------------------------------------------------------------
// 7. Executor cancellation: Stop, Lease Loss, and E-Stop + session slot reuse
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_stop_estop_and_lease_loss_cancellation_and_slot_reuse() {
    let drv = ZeroCheckDriver::new(-1.20);
    drv.state.lock().unwrap().stall_stop_q = Some(0.0);
    let drv_clone = drv.clone();
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |_| {},
        move |inner| {
            inner.measured_positions[6] = -1.20;
            inner.commanded_positions[6] = -1.20;
            inner.target_positions[6] = -1.20;
            inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv_clone)));
        },
        PersistTestHooks::default(),
    )
    .await;

    let mut ws = connect_ws(rig.addr, None).await;
    let gen = acquire_lease(&mut ws, "c-cancel", false).await;
    let epoch = rig.sup.get_observation().await.epoch_id;
    let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack_t["type"], "command_ack", "{ack_t}");
    let epoch = rig.sup.get_observation().await.epoch_id;

    // (a) Stop mid-motion
    let ack1 = send_cmd(
        &mut ws,
        gen,
        epoch,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
    )
    .await;
    assert_eq!(ack1["type"], "command_ack", "{ack1}");
    let aid1 = ack1["action_id"].as_u64().unwrap();
    tokio::time::sleep(Duration::from_millis(60)).await;
    {
        let mut s = drv.state.lock().unwrap();
        s.recorded_j7_targets.clear();
        s.recorded_j7_at.clear();
        s.record_targets = true;
    }
    let stop_ack = send_cmd(&mut ws, gen, epoch, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");
    let term1 = wait_terminal_for_id(&mut ws, aid1, Duration::from_secs(4)).await;
    assert_eq!(term1["state"], "cancelled", "{term1}");
    assert_eq!(term1["code"], "STOPPED", "{term1}");
    assert!(!rig.calib_path.exists(), "stopped check must not persist");
    {
        let s = drv.state.lock().unwrap();
        // After stop, jaw targets must not continue closing toward 0.0.
        if let Some(&last_t) = s.recorded_j7_targets.last() {
            assert!(
                last_t < -0.20,
                "after stop, gripper must hold measured jaw instead of continuing to 0.0 (last={last_t})"
            );
        }
    }
    rig.wait_zero_check_ctx_empty("after_stop").await;

    // (b) Lease release mid-motion -> session clears so next controller can immediately run
    let epoch2 = rig.sup.get_observation().await.epoch_id;
    let ack2 = send_cmd(
        &mut ws,
        gen,
        epoch2,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
    )
    .await;
    assert_eq!(ack2["type"], "command_ack", "{ack2}");
    let aid2 = ack2["action_id"].as_u64().unwrap();
    tokio::time::sleep(Duration::from_millis(40)).await;
    let rel = ws_reply(
        &mut ws,
        json!({"type": "release_lease", "generation": gen, "seq": 2}),
        &["lease_released"],
    )
    .await;
    assert_eq!(rel["type"], "lease_released", "{rel}");
    let term2 = wait_terminal_for_id(&mut ws, aid2, Duration::from_secs(4)).await;
    assert_eq!(term2["state"], "cancelled", "{term2}");
    assert_eq!(term2["code"], "LEASE_LOST", "{term2}");
    rig.wait_zero_check_ctx_empty("after_lease_loss").await;

    // (c) New controller acquires lease and runs E-stop mid-motion
    let mut ws2 = connect_ws(rig.addr, None).await;
    let gen2 = acquire_lease(&mut ws2, "c-estop", false).await;
    let epoch3 = rig.sup.get_observation().await.epoch_id;
    let ack3 = send_cmd(
        &mut ws2,
        gen2,
        epoch3,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
    )
    .await;
    assert_eq!(ack3["type"], "command_ack", "{ack3}");
    let aid3 = ack3["action_id"].as_u64().unwrap();
    tokio::time::sleep(Duration::from_millis(40)).await;
    let _ = ws_reply(&mut ws2, json!({"type": "e_stop"}), &["epoch_sync"]).await;
    let term3 = wait_terminal_for_id(&mut ws2, aid3, Duration::from_secs(4)).await;
    assert_ne!(term3["state"], "done", "{term3}");
    assert!(!rig.calib_path.exists());
    rig.wait_zero_check_ctx_empty("after_estop").await;

    // Clear fault + re-enable torque + run a short successful check (proves slot reuse!)
    drv.state.lock().unwrap().q[6] = -0.10;
    let ep_es = rig.sup.get_observation().await.epoch_id;
    let clear_ack = send_cmd(&mut ws2, gen2, ep_es, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(clear_ack["type"], "command_ack", "{clear_ack}");
    let ep_cl = rig.sup.get_observation().await.epoch_id;
    let en_ack = send_cmd(&mut ws2, gen2, ep_cl, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(en_ack["type"], "command_ack", "{en_ack}");
    let ep_en = rig.sup.get_observation().await.epoch_id;
    let (_, term4, _) = run_cmd_to_terminal(
        &mut ws2,
        gen2,
        ep_en,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": false}),
        Duration::from_secs(6),
    )
    .await;
    assert_eq!(term4["state"], "done", "{term4}");

    rig.wait_zero_check_ctx_empty("after_reuse").await;
    rig.assert_zero_rt_allocs("stop_estop_lease_reuse");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 8. Executor mid-motion operator lock aborts with OPERATOR_REQUIRED and 0 mutation
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_mid_motion_operator_lock_aborts_without_mutation() {
    let client = reqwest::Client::new();
    let auth = OperatorAuth::enforced("123456", 1800);
    let drv = ZeroCheckDriver::new(-1.0);
    drv.state.lock().unwrap().stall_stop_q = Some(0.02);
    let drv_clone = drv.clone();
    let rig = DualRig::start(
        RtOwner::Executor,
        auth,
        |_| {},
        move |inner| {
            inner.measured_positions[6] = -1.0;
            inner.commanded_positions[6] = -1.0;
            inner.target_positions[6] = -1.0;
            inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(drv_clone)));
        },
        PersistTestHooks::default(),
    )
    .await;
    let base = format!("http://{}", rig.addr);

    let unlock: Value = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let token = unlock["token"].as_str().unwrap().to_string();

    let mut ws = connect_ws(rig.addr, Some(&token)).await;
    let gen = acquire_lease(&mut ws, "c-oplock", false).await;
    let epoch = rig.sup.get_observation().await.epoch_id;
    let ack_t = send_cmd(&mut ws, gen, epoch, "/api/v1/torque", json!({"enable": true})).await;
    assert_eq!(ack_t["type"], "command_ack", "{ack_t}");
    let epoch = rig.sup.get_observation().await.epoch_id;

    let ack = send_cmd(
        &mut ws,
        gen,
        epoch,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
    )
    .await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let aid = ack["action_id"].as_u64().unwrap();

    // Lock operator session while gripper seek is still moving!
    tokio::time::sleep(Duration::from_millis(50)).await;
    let lock_res = client
        .post(format!("{base}/api/v1/auth/lock"))
        .header(AUTHORIZATION.as_str(), format!("Bearer {token}"))
        .send()
        .await
        .unwrap();
    assert_eq!(lock_res.status(), 200);

    let term = wait_terminal_for_id(&mut ws, aid, Duration::from_secs(4)).await;
    assert_eq!(term["code"], "OPERATOR_REQUIRED", "{term}");
    assert!(!rig.calib_path.exists(), "locked operator must not persist");
    rig.wait_zero_check_ctx_empty("mid_motion_op_lock").await;

    // Verify session slot is free: store_offset=false succeeds as Program right away.
    drv.state.lock().unwrap().q[6] = -0.10;
    let ep2 = rig.sup.get_observation().await.epoch_id;
    let (_, term2, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep2,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": false}),
        Duration::from_secs(6),
    )
    .await;
    assert_eq!(term2["state"], "done", "{term2}");

    rig.assert_zero_rt_allocs("mid_motion_op_lock");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 9. Executor PR C boundaries: pre-rename revocation, dir-fsync EIO + reconcile, worker crash
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_prc_revocation_before_rename_dir_sync_retry_and_worker_crash() {
    let client = reqwest::Client::new();

    // (a) Pre-rename operator revocation via after_grant_before_rename hook
    {
        let auth = OperatorAuth::enforced("123456", 1800);
        let auth_hook = auth.clone();
        let tok_slot: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let tok_hook = Arc::clone(&tok_slot);
        let hooks = PersistTestHooks {
            after_grant_before_rename: Some(Arc::new(move || {
                if let Some(tok) = tok_hook.lock().unwrap().as_deref() {
                    auth_hook.lock(Some(tok));
                }
                false
            })),
            ..Default::default()
        };
        let rig = DualRig::start(
            RtOwner::Executor,
            auth,
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            hooks,
        )
        .await;
        let base = format!("http://{}", rig.addr);
        let unlock: Value = client
            .post(format!("{base}/api/v1/auth/unlock"))
            .json(&json!({"pin": "123456"}))
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        let token = unlock["token"].as_str().unwrap().to_string();
        *tok_slot.lock().unwrap() = Some(token.clone());

        let mut ws = connect_ws(rig.addr, Some(&token)).await;
        let gen = acquire_lease(&mut ws, "c-rev", false).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let ep = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term["state"], "failed", "{term}");
        assert_eq!(term["code"], "OPERATOR_REQUIRED", "{term}");
        assert!(!rig.calib_path.exists(), "pre-rename revocation must not rename onto disk");
        rig.wait_zero_check_ctx_empty("pre_rename_revoke").await;
        rig.assert_zero_rt_allocs("pre_rename_revoke");
        rig.shutdown().await;
    }

    // (b) Dir-fsync EIO once -> PERSIST_FAILED + durability_confirmed=false + warning + Reconciling -> retry succeeds
    {
        let sync_calls = Arc::new(AtomicUsize::new(0));
        let sync_calls_h = Arc::clone(&sync_calls);
        let hooks = PersistTestHooks {
            dir_sync_override: Some(Arc::new(move |_| {
                let n = sync_calls_h.fetch_add(1, Ordering::AcqRel);
                if n == 0 {
                    Err(std::io::Error::from_raw_os_error(libc::EIO))
                } else {
                    Ok(())
                }
            })),
            reconcile_retry_interval: Some(Duration::from_millis(20)),
            reconcile_max_attempts: Some(5),
            ..Default::default()
        };
        let rig = DualRig::start(
            RtOwner::Executor,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            hooks,
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-eio", false).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let ep = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term["state"], "failed", "{term}");
        assert_eq!(term["code"], "PERSIST_FAILED", "{term}");
        assert_eq!(term["durability_confirmed"], json!(false), "{term}");
        assert!(
            term["durability_warning"].as_str().is_some_and(|w| !w.is_empty()),
            "expected non-empty durability_warning: {term}"
        );

        // Background reconciliation retries dir-fsync and clears HoldState::Reconciling.
        let dl = Instant::now() + Duration::from_secs(4);
        loop {
            let reply = rig
                .sup
                .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
                .await
                .unwrap();
            if let ConfigReply::HoldInspected { state: None, .. } = reply {
                break;
            }
            assert!(
                Instant::now() < dl,
                "GripperCalibration did not reconcile after dir_sync retry"
            );
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(sync_calls.load(Ordering::Acquire) >= 2);
        rig.wait_zero_check_ctx_empty("dir_sync_retry").await;
        rig.assert_zero_rt_allocs("dir_sync_retry");
        rig.shutdown().await;
    }

    // (c) Worker crash after reservation -> PERSIST_FAILED + HoldState::Reconciling + 0 leak + 0 RT alloc
    {
        let hooks = PersistTestHooks {
            after_grant_before_rename: Some(Arc::new(|| true)),
            ..Default::default()
        };
        let rig = DualRig::start(
            RtOwner::Executor,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            hooks,
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-crash", false).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let ep = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term["state"], "failed", "{term}");
        assert_eq!(term["code"], "PERSIST_FAILED", "{term}");

        let reply = rig
            .sup
            .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
            .await
            .unwrap();
        match reply {
            ConfigReply::HoldInspected {
                state: Some(HoldState::Reconciling),
                ..
            } => {}
            _ => panic!("expected HoldState::Reconciling after worker crash"),
        }
        rig.wait_zero_check_ctx_empty("worker_crash").await;
        rig.assert_zero_rt_allocs("worker_crash");
        rig.shutdown().await;
    }
}

/// Dir-fsync failure settles the action and its session; the background reconciliation must not
/// activate the stored offset once the operator grant of that action is no longer valid, even
/// when the worker sampled the operator session as valid just before queueing `FinishReconcile`.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_reconcile_refuses_activation_after_operator_revoked() {
    let client = reqwest::Client::new();
    let auth = OperatorAuth::enforced("123456", 1800);
    let auth_hook = auth.clone();
    let tok_slot: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let tok_hook = Arc::clone(&tok_slot);
    let revoked = Arc::new(AtomicBool::new(false));
    let revoked_h = Arc::clone(&revoked);
    let sync_calls = Arc::new(AtomicUsize::new(0));
    let sync_calls_h = Arc::clone(&sync_calls);
    let hooks = PersistTestHooks {
        dir_sync_override: Some(Arc::new(move |_| {
            if sync_calls_h.fetch_add(1, Ordering::AcqRel) == 0 {
                Err(std::io::Error::from_raw_os_error(libc::EIO))
            } else {
                Ok(())
            }
        })),
        reconcile_retry_interval: Some(Duration::from_millis(20)),
        reconcile_max_attempts: Some(5),
        // Revoke strictly after the worker sampled the session as valid.
        before_finish_reconcile: Some(Arc::new(move || {
            if let Some(tok) = tok_hook.lock().unwrap().as_deref() {
                auth_hook.lock(Some(tok));
                revoked_h.store(true, Ordering::Release);
            }
        })),
        ..Default::default()
    };
    let rig = DualRig::start(
        RtOwner::Executor,
        auth,
        |_| {},
        |inner| {
            inner.set_mock_gripper_stall_offset(Some(0.05));
        },
        hooks,
    )
    .await;
    let base = format!("http://{}", rig.addr);
    let unlock: Value = client
        .post(format!("{base}/api/v1/auth/unlock"))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let token = unlock["token"].as_str().unwrap().to_string();
    *tok_slot.lock().unwrap() = Some(token.clone());
    let cal_rev = || async {
        let v: Value = client
            .get(format!("{base}/api/v1/gripper/calibration"))
            .bearer_auth(&token)
            .send()
            .await
            .unwrap()
            .json()
            .await
            .unwrap();
        v["revision"].as_u64().unwrap()
    };
    let rev_before = cal_rev().await;

    let mut ws = connect_ws(rig.addr, Some(&token)).await;
    let gen = acquire_lease(&mut ws, "c-rec-rev", false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let (_, term, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(8),
    )
    .await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "PERSIST_FAILED", "{term}");

    let dl = Instant::now() + Duration::from_secs(4);
    while !revoked.load(Ordering::Acquire) {
        assert!(Instant::now() < dl, "reconciliation never reached FinishReconcile");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    // Give the owner time to process FinishReconcile and the worker to publish any activation.
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(
        cal_rev().await,
        rev_before,
        "reconciliation activated the offset after the operator grant was revoked"
    );
    let reply = rig
        .sup
        .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
        .await
        .unwrap();
    assert!(
        matches!(reply, ConfigReply::HoldInspected { state: Some(_), .. }),
        "GripperCalibration must stay held (reconciliation refused)"
    );
    rig.wait_zero_check_ctx_empty("reconcile_revoked").await;
    rig.shutdown().await;
}

const OLD_CALIB: &str = "{\"offset_rad\":0.02}";

/// Policy a fresh supervisor loads at startup from `path` (restart behavior).
async fn policy_after_restart(path: &std::path::Path) -> (f32, f32) {
    let sup = ArmSupervisor::new(ArmConfig::rebot_dm());
    armd::gripper_calib::load_into(&sup, path).await;
    let inner = sup.inner().read().await;
    assert!(!inner.gripper_calib_invalid, "calibration file must load");
    let p = inner.config.gripper_policy.as_ref().expect("gripper policy");
    (p.closed_rad, p.open_rad)
}

/// Executor rig with `OLD_CALIB` on disk, operator auth enforced and an unlocked token.
async fn old_calib_rig(
    hooks: impl FnOnce(OperatorAuth, Arc<Mutex<Option<String>>>) -> PersistTestHooks,
) -> (DualRig, String, (f32, f32)) {
    let auth = OperatorAuth::enforced("123456", 1800);
    let tok_slot: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
    let hooks = hooks(auth.clone(), Arc::clone(&tok_slot));
    let rig = DualRig::start_with_calib(
        RtOwner::Executor,
        auth,
        |_| {},
        |inner| inner.set_mock_gripper_stall_offset(Some(0.05)),
        hooks,
        Some(OLD_CALIB),
    )
    .await;
    let old_policy = policy_after_restart(&rig.calib_path).await;
    let unlock: Value = reqwest::Client::new()
        .post(format!("http://{}/api/v1/auth/unlock", rig.addr))
        .json(&json!({"pin": "123456"}))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let token = unlock["token"].as_str().unwrap().to_string();
    *tok_slot.lock().unwrap() = Some(token.clone());
    (rig, token, old_policy)
}

async fn run_store_check(rig: &DualRig, token: &str, client_id: &str) -> Value {
    let mut ws = connect_ws(rig.addr, Some(token)).await;
    let gen = acquire_lease(&mut ws, client_id, false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let (_, term, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(8),
    )
    .await;
    term
}

/// The owner refuses activation (operator revoked after the candidate is staged): the official
/// file keeps the old contents, the `.pending` candidate is removed, and a restart loads the old
/// calibration.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_refused_activation_leaves_official_file_and_restart_loads_old() {
    let (rig, token, old_policy) = old_calib_rig(|auth, tok| PersistTestHooks {
        after_rename_before_activate: Some(Arc::new(move || {
            if let Some(t) = tok.lock().unwrap().as_deref() {
                auth.lock(Some(t));
            }
            false
        })),
        ..Default::default()
    })
    .await;
    let term = run_store_check(&rig, &token, "c-refuse").await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "OPERATOR_REQUIRED", "{term}");
    assert_eq!(std::fs::read_to_string(&rig.calib_path).unwrap(), OLD_CALIB);
    let pending = armd::gripper_calib::pending_path_for(&rig.calib_path);
    let dl = Instant::now() + Duration::from_secs(2);
    while pending.exists() {
        assert!(Instant::now() < dl, "refused candidate must be unlinked");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(policy_after_restart(&rig.calib_path).await, old_policy);
    rig.wait_zero_check_ctx_empty("refused_activation").await;
    rig.assert_zero_rt_allocs("refused_activation");
    rig.shutdown().await;
}

/// Crash after activation, before the rename: the action fails closed (`PERSIST_FAILED`), the
/// candidate stays at `.pending`, the hold is released, and a restart loads the old calibration
/// and removes the stale candidate.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_crash_after_activation_restarts_with_old_and_removes_pending() {
    let crashed = Arc::new(AtomicBool::new(false));
    let crashed_h = Arc::clone(&crashed);
    let (rig, token, old_policy) = old_calib_rig(move |_, _| PersistTestHooks {
        after_activate_before_publish: Some(Arc::new(move || {
            crashed_h.store(true, Ordering::Release);
            true
        })),
        ..Default::default()
    })
    .await;
    let term = run_store_check(&rig, &token, "c-crash-act").await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "PERSIST_FAILED", "{term}");
    assert_eq!(term["durability_confirmed"], json!(false), "{term}");
    assert!(crashed.load(Ordering::Acquire), "worker must stop between activation and rename");
    let hold = rig
        .sup
        .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
        .await
        .unwrap();
    assert!(
        matches!(hold, ConfigReply::HoldInspected { state: None, .. }),
        "hold must be released after activation"
    );
    let pending = armd::gripper_calib::pending_path_for(&rig.calib_path);
    assert!(pending.exists(), "candidate stays staged after the crash");
    assert_eq!(std::fs::read_to_string(&rig.calib_path).unwrap(), OLD_CALIB);
    assert_eq!(policy_after_restart(&rig.calib_path).await, old_policy);
    assert!(!pending.exists(), "startup must remove the stale candidate");
    rig.shutdown().await;
}

/// Official rename failure after live activation: action fails closed with `PERSIST_FAILED`
/// reporting that the old calibration remains on disk, the hold is released, `.pending` is
/// discarded, and a restart loads the old calibration.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_official_rename_failure_fails_closed_and_restart_loads_old() {
    let (rig, token, old_policy) = old_calib_rig(|_, _| PersistTestHooks {
        fail_publish_rename_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    })
    .await;
    let term = run_store_check(&rig, &token, "c-pub-rename-fail").await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "PERSIST_FAILED", "{term}");
    assert_eq!(term["durability_confirmed"], json!(false), "{term}");
    let msg = term["message"].as_str().unwrap_or("");
    assert!(
        msg.contains("old calibration remains on disk"),
        "expected message to state old calibration remains on disk, got: {msg}"
    );
    let hold = rig
        .sup
        .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
        .await
        .unwrap();
    assert!(
        matches!(hold, ConfigReply::HoldInspected { state: None, .. }),
        "hold must be released after official rename failure"
    );
    assert_eq!(std::fs::read_to_string(&rig.calib_path).unwrap(), OLD_CALIB);
    let pending = armd::gripper_calib::pending_path_for(&rig.calib_path);
    let dl = Instant::now() + Duration::from_secs(2);
    while pending.exists() {
        assert!(Instant::now() < dl, "unrenamed candidate must be discarded");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(policy_after_restart(&rig.calib_path).await, old_policy);
    rig.wait_zero_check_ctx_empty("official_rename_failure").await;
    rig.assert_zero_rt_allocs("official_rename_failure");
    rig.shutdown().await;
}

/// Official parent-directory fsync failure after live activation and rename: action finishes `done`
/// with `durability_confirmed=false`, warning and message stating the calibration is active but
/// persistence is unconfirmed, the hold is released, and a restart loads the new calibration.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_official_dir_sync_failure_reports_unconfirmed_and_restart_loads_new() {
    let (rig, token, old_policy) = old_calib_rig(|_, _| PersistTestHooks {
        fail_publish_dir_sync_once: Some(Arc::new(AtomicBool::new(true))),
        ..Default::default()
    })
    .await;
    let term = run_store_check(&rig, &token, "c-pub-dirsync-fail").await;
    assert_eq!(term["state"], "done", "{term}");
    assert_eq!(term["durability_confirmed"], json!(false), "{term}");
    let warn = term["durability_warning"].as_str().unwrap_or("");
    assert!(
        warn.contains("active")
            && warn.contains("unconfirmed")
            && warn.contains("simulated official dir fsync failure"),
        "expected unconfirmed persistence warning, got: {warn}"
    );
    let msg = term["message"].as_str().unwrap_or("");
    assert!(
        msg.contains("active") && msg.contains("unconfirmed"),
        "expected message stating value is active but persistence is unconfirmed, got: {msg}"
    );
    let hold = rig
        .sup
        .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
        .await
        .unwrap();
    assert!(
        matches!(hold, ConfigReply::HoldInspected { state: None, .. }),
        "hold must be released after official dir-fsync failure"
    );
    assert!(
        !armd::gripper_calib::pending_path_for(&rig.calib_path).exists(),
        ".pending must have been renamed over the official file"
    );
    let on_disk: Value =
        serde_json::from_str(&std::fs::read_to_string(&rig.calib_path).unwrap()).unwrap();
    assert!(
        (on_disk["offset_rad"].as_f64().unwrap() - 0.05).abs() < 1e-2,
        "official file must hold new calibration: {on_disk}"
    );
    let restarted_policy = policy_after_restart(&rig.calib_path).await;
    assert_ne!(restarted_policy, old_policy, "restart must load the new calibration");
    let live_cal: Value = reqwest::Client::new()
        .get(format!("http://{}/api/v1/gripper/calibration", rig.addr))
        .bearer_auth(&token)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let live_closed = live_cal["policy"]["closed_rad"].as_f64().unwrap() as f32;
    let live_open = live_cal["policy"]["open_rad"].as_f64().unwrap() as f32;
    assert!(
        (restarted_policy.0 - live_closed).abs() < 1e-5
            && (restarted_policy.1 - live_open).abs() < 1e-5,
        "restart policy {restarted_policy:?} must match live ({live_closed}, {live_open})"
    );
    rig.wait_zero_check_ctx_empty("official_dir_sync_failure").await;
    rig.assert_zero_rt_allocs("official_dir_sync_failure");
    rig.shutdown().await;
}

/// A stop and a lease change both land after the candidate is staged: the refusal code is the
/// live activation check's (`check_zero_check_authority`), and the official file is unchanged.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_stop_and_lease_change_refusal_uses_live_check_code() {
    let rt = tokio::runtime::Handle::current();
    let addr_slot: Arc<Mutex<Option<(SocketAddr, Arc<ArmSupervisor>)>>> = Arc::new(Mutex::new(None));
    let stop_ack: Arc<Mutex<Option<Value>>> = Arc::new(Mutex::new(None));
    let stop_ack_h = Arc::clone(&stop_ack);
    let addr_h = Arc::clone(&addr_slot);
    let (rig, token, _old) = old_calib_rig(move |_, tok| PersistTestHooks {
        after_rename_before_activate: Some(Arc::new(move || {
            let (addr, sup) = addr_h.lock().unwrap().clone().expect("addr set");
            let t = tok.lock().unwrap().clone();
            let ack_slot = Arc::clone(&stop_ack_h);
            rt.block_on(async move {
                let mut ws2 = connect_ws(addr, t.as_deref()).await;
                let gen2 = acquire_lease(&mut ws2, "c-thief", true).await;
                let ep = sup.get_observation().await.epoch_id;
                let ack = send_cmd(&mut ws2, gen2, ep, "/api/v1/motion/stop", json!({})).await;
                *ack_slot.lock().unwrap() = Some(ack);
                std::mem::forget(ws2);
            });
            false
        })),
        ..Default::default()
    })
    .await;
    *addr_slot.lock().unwrap() = Some((rig.addr, Arc::clone(&rig.sup)));
    let term = run_store_check(&rig, &token, "c-stop-lease").await;
    let ack = stop_ack.lock().unwrap().clone().expect("stop sent");
    assert_eq!(ack["type"], "command_ack", "stop by the new lease holder: {ack}");
    assert_eq!(term["state"], "cancelled", "{term}");
    assert_eq!(term["code"], "STOPPED", "{term}");
    assert_eq!(std::fs::read_to_string(&rig.calib_path).unwrap(), OLD_CALIB);
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 10. Stream saturation, 20x repeated failure zero-leak proof, PlanSlot size
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_stream_saturation_repeated_failures_zero_leak_and_plan_slot_size() {
    // Bounded PlanSlot size invariant (ensure StagedAction::GripperZeroCheck did not bloat PlanSlot).
    let slot_bytes = std::mem::size_of::<PlanSlot>();
    assert!(
        slot_bytes <= 16_384,
        "PlanSlot unexpectedly bloated: {slot_bytes} bytes"
    );

    let fail_push = Arc::new(AtomicBool::new(true));
    let hooks = PersistTestHooks {
        fail_zero_check_stream_push_once: Some(Arc::clone(&fail_push)),
        ..Default::default()
    };
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |_| {},
        |inner| {
            inner.set_mock_gripper_stall_offset(Some(0.05));
        },
        hooks,
    )
    .await;

    let mut ws = connect_ws(rig.addr, None).await;
    let gen = acquire_lease(&mut ws, "c-sat", false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;

    // 1. Injected stream saturation -> PERSIST_FAILED + context reclaimed + slot cleared
    let (_, term_sat, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(8),
    )
    .await;
    assert_eq!(term_sat["state"], "failed", "{term_sat}");
    assert_eq!(term_sat["code"], "PERSIST_FAILED", "{term_sat}");
    assert!(!rig.calib_path.exists());
    rig.wait_zero_check_ctx_empty("stream_saturation").await;

    // 2. 20x consecutive post-staging store_offset=true refusals (exceeding ZERO_CHECK_CTX_CAPACITY=16!)
    //    Hold `config_write_lock` while one store_offset=true check is in PR C persistence so
    //    `ExclusiveOutput` is occupied: every subsequent valid command passes off-RT preparation,
    //    calls `stage_zero_check_context`, submits to the broker, and is refused on RT
    //    (`MOTION_BUSY` / `SESSION_BUSY`), proving `BrokerDispatcher` reclaims every staged context.
    let cw_guard = rig.sup.config_write_lock().lock_owned().await;
    let first_req = next_req_id();
    send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": first_req,
            "generation": gen,
            "epoch_id": ep,
            "path": "/api/v1/action/gripper_zero_check",
            "body": {"store_offset": true},
        }),
    )
    .await;
    let first_ack = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("request_id").and_then(Value::as_u64) == Some(first_req) {
                return v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(first_ack["type"], "command_ack", "{first_ack}");
    let first_action_id = first_ack["action_id"].as_u64().unwrap();

    for i in 0..20u64 {
        let cur_ep = rig.sup.get_observation().await.epoch_id;
        let rej = send_cmd(
            &mut ws,
            gen,
            cur_ep,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
        )
        .await;
        assert_eq!(rej["type"], "command_reject", "iter {i}: {rej}");
        let code = rej["code"].as_str().unwrap_or("");
        assert!(
            code == "MOTION_BUSY" || code == "SESSION_BUSY" || code == "BUSY",
            "iter {i}: expected RT busy refusal after staging, got {rej}"
        );
    }
    drop(cw_guard);
    let term_first = wait_terminal_for_id(&mut ws, first_action_id, Duration::from_secs(8)).await;
    assert_eq!(term_first["state"], "done", "{term_first}");
    assert_eq!(term_first["durability_confirmed"], json!(true), "{term_first}");
    assert!(rig.calib_path.exists());
    rig.wait_zero_check_ctx_empty("post_staging_20x_refusals").await;

    // 3. Subsequent valid store_offset=true succeeds cleanly after saturation + 20 post-staging refusals!
    let ep2 = rig.sup.get_observation().await.epoch_id;
    let (_, term_ok, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep2,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(8),
    )
    .await;
    assert_eq!(term_ok["state"], "done", "{term_ok}");
    assert_eq!(term_ok["durability_confirmed"], json!(true), "{term_ok}");
    assert!(rig.calib_path.exists());

    rig.wait_zero_check_ctx_empty("after_recovery").await;
    rig.assert_zero_rt_allocs("stream_sat_and_20x_refusals");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 11. Blocker 1 regression: E-stop with stale J7 feedback preserves commanded hold
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_estop_preserves_commanded_when_j7_feedback_stale() {
    let driver = ZeroCheckDriver::new(-0.50);
    let driver_for_inner = driver.clone();
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |cfg| {
            // Disable close torque-cap watchdog so RT ingests the 350ms stale J7 sample
            // before E-stop arrives instead of aborting early on FEEDBACK_UNAVAILABLE.
            if let Some(gp) = cfg.gripper_policy.as_mut() {
                gp.max_close_torque_nm = None;
            }
        },
        move |inner| {
            let mut q = INSPECT;
            q[6] = -0.50;
            inner.measured_positions = q.to_vec();
            inner.commanded_positions = q.to_vec();
            inner.target_positions = q.to_vec();
            inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(driver_for_inner)));
        },
        PersistTestHooks::default(),
    )
    .await;

    let mut ws = connect_ws(rig.addr, None).await;
    let gen = acquire_lease(&mut ws, "c-estop-stale", false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;

    let req_id = next_req_id();
    send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen,
            "epoch_id": ep,
            "path": "/api/v1/action/gripper_zero_check",
            "body": {"store_offset": false},
        }),
    )
    .await;
    let ack = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("request_id").and_then(Value::as_u64) == Some(req_id) {
                return v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let action_id = ack["action_id"].as_u64().unwrap();

    // Wait until seek advances J7 commanded target above -0.44 rad.
    let dl = Instant::now() + Duration::from_secs(2);
    loop {
        let last_j7 = driver.state.lock().unwrap().last_written_j7;
        if last_j7.is_some_and(|v| v > -0.44) {
            break;
        }
        assert!(Instant::now() < dl, "J7 seek did not advance: {last_j7:?}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    // Freeze J7 feedback at age 350 ms (> E-stop's 250 ms freshness gate, < 500 ms BUS_LOSS_TIMEOUT)
    // with cached measured_positions[6] = -0.80 rad (far behind commanded ~ -0.42 rad).
    {
        let mut s = driver.state.lock().unwrap();
        s.stale_j7 = Some((Duration::from_millis(350), -0.80));
        s.recorded_j7_targets.clear();
        s.recorded_j7_at.clear();
        s.record_targets = true;
    }
    tokio::time::sleep(Duration::from_millis(25)).await;

    let _ = ws_reply(&mut ws, json!({"type": "e_stop"}), &["epoch_sync"]).await;
    let term = wait_terminal_for_id(&mut ws, action_id, Duration::from_secs(4)).await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "ESTOP_ACTIVE", "{term}");

    // Let several post-E-stop ticks execute and verify no target ever snapped to stale -0.80 rad.
    tokio::time::sleep(Duration::from_millis(60)).await;
    let recorded = driver.state.lock().unwrap().recorded_j7_targets.clone();
    assert!(!recorded.is_empty(), "expected J7 targets recorded across E-stop");
    for &t in &recorded {
        assert!(
            t > -0.60,
            "E-stop overwrote J7 hold target with stale cached feedback (-0.80): recorded={recorded:?}"
        );
    }

    rig.assert_zero_rt_allocs("estop_stale_j7");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 12. Major 1 regression: oversized durability_warning -> 0 RT alloc + full mirror
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_oversized_durability_warning_zero_rt_alloc_and_full_mirror() {
    let calls = Arc::new(AtomicUsize::new(0));
    let calls_hook = Arc::clone(&calls);
    let long_tail = "Z".repeat(420);
    let long_tail_hook = long_tail.clone();
    let hooks = PersistTestHooks {
        dir_sync_override: Some(Arc::new(move |_| {
            let n = calls_hook.fetch_add(1, Ordering::AcqRel);
            if n == 0 {
                Err(std::io::Error::other(format!("dir_sync_eio: {long_tail_hook}")))
            } else {
                Ok(())
            }
        })),
        ..Default::default()
    };
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |_| {},
        |inner| {
            inner.set_mock_gripper_stall_offset(Some(0.05));
        },
        hooks,
    )
    .await;

    let mut ws = connect_ws(rig.addr, None).await;
    let gen = acquire_lease(&mut ws, "c-big-warn", false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;

    let (_, term, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(8),
    )
    .await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "PERSIST_FAILED", "{term}");
    assert_eq!(term["durability_confirmed"], json!(false), "{term}");
    let msg = term["message"].as_str().unwrap_or("");
    assert!(
        msg.contains("Parent directory fsync failed"),
        "expected non-empty dir-sync failure message alongside warning, got: {term}"
    );
    let warn = term["durability_warning"].as_str().unwrap_or("");
    assert!(
        warn.contains(&long_tail),
        "expected full untruncated >420-byte durability_warning in WS mirror, got len={}: {warn}",
        warn.len()
    );

    // Wait for background reconciliation to clear HoldState::Reconciling.
    let dl = Instant::now() + Duration::from_secs(6);
    loop {
        let reply = rig
            .sup
            .dispatch_config_request(ConfigRequest::InspectHold(ConfigDomain::GripperCalibration))
            .await
            .unwrap();
        if let ConfigReply::HoldInspected { state: None, .. } = reply {
            break;
        }
        assert!(Instant::now() < dl, "GripperCalibration did not reconcile");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }

    // Run two follow-up actions to force full rollover of `action` and `prev_action` in ObservationSpares.
    for _ in 0..2 {
        let ep_next = rig.sup.get_observation().await.epoch_id;
        let (_, t2, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            ep_next,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": false}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(t2["state"], "done", "{t2}");
    }

    rig.wait_zero_check_ctx_empty("oversized_warning_rollover").await;
    rig.assert_zero_rt_allocs("oversized_warning_rollover");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 13. Major 2 regression: drop waiter after broker admit + fail-closed after worker exit
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_drop_waiter_preserves_admitted_ctx_and_worker_exit_fails_closed() {
    // Part A: dropping the broker submission oneshot receiver immediately after admission must
    // NOT delete the staged ZeroCheckPersistContext; the admitted action finishes & persists.
    {
        let rig = DualRig::start(
            RtOwner::Executor,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.lease_enabled = false;
                let adm = armd::supervisor::AdmissionStamp {
                    stop_gen: 0,
                    estop_epoch: inner.safety.epoch_id(),
                    lease_gen: 0,
                    connection_id: 0,
                    request_id: 1,
                };
                let admitted = inner.check_admission(adm).unwrap();
                armd::SupervisorInner::enable_torque(admitted).unwrap();
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            PersistTestHooks::default(),
        )
        .await;

        let handle = rig.sup.executor_handle().unwrap();
        handle.attach_external_reply_sink(Arc::new(|_, _| {
            armd::rt_core::broker::DeliveryDecision::Delivered
        }));

        let svc = rig
            .sup
            .read_services(armd::rt_core::publish::ReadRequirements {
                require_within_valid_until: false,
                require_prep_env: true,
                ..armd::rt_core::publish::ReadRequirements::identity()
            })
            .await
            .unwrap();
        let pub_cfg = rig.sup.published_config().await.unwrap();
        let admission = armd::supervisor::AdmissionStamp {
            stop_gen: svc.applied_stop_generation,
            estop_epoch: svc.snapshot.owner.epoch,
            lease_gen: 0,
            connection_id: 0,
            request_id: 7777,
        };
        let deadline = Instant::now() + Duration::from_secs(5);
        let (prepared, persist_ctx) = armd::rt_core::requests::actions::prepare_gripper_zero_check(
            &svc,
            &pub_cfg,
            admission,
            deadline,
            &json!({"store_offset": true}),
            armd::auth::OperatorGrant::unenforced(),
            None,
        )
        .unwrap();
        let ctx_id = handle.stage_zero_check_context(persist_ctx.unwrap()).unwrap();
        let slot = prepared.to_plan_slot(ctx_id).unwrap();
        let write = |dst: &mut PlanSlot| *dst = slot.clone();
        let (_req_id, rx, ack_guard) = handle
            .submit_external_action_with_zero_check_ctx(
                armd::rt_core::broker::ClientId(0),
                0,
                prepared.stamp.broker_authority(),
                deadline,
                armd::rt_core::requests::actions::ActionSubmit::InstallPrepared {
                    write: &write,
                    kind: armd::rt_core::requests::actions::PreparedActionKind::GripperZeroCheck,
                    prepared_boot: prepared.stamp.boot,
                    admission_request_id: prepared.stamp.admission.request_id,
                    check_workspace_revision: false,
                    poses_revision: None,
                    calibration_revision: prepared.stamp.calibration_revision,
                    gripper_calib_revision: prepared.stamp.gripper_calib_revision,
                    power_budget_revision: prepared.stamp.power_budget_revision,
                },
                armd::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOrGlobal,
                false,
                None,
                ctx_id,
            )
            .unwrap();
        // Simulate caller future dropped right after broker admission: drop `rx` and `ack_guard`!
        drop(rx);
        drop(ack_guard);

        // Wait until the admitted action completes and persists cleanly.
        let dl = Instant::now() + Duration::from_secs(8);
        loop {
            handle.pump_broker_replies();
            let obs = rig.sup.get_observation().await;
            if let Some(a) = obs.action.as_ref() {
                if a.kind == "gripper_zero_check" && a.state != "running" {
                    assert_eq!(a.state, "done", "{a:?}");
                    assert_eq!(a.durability_confirmed, Some(true), "{a:?}");
                    break;
                }
            }
            assert!(Instant::now() < dl, "admitted zero_check did not complete after caller drop");
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        assert!(rig.calib_path.exists());
        rig.wait_zero_check_ctx_empty("drop_waiter_admitted").await;
        rig.assert_zero_rt_allocs("drop_waiter_admitted");
        rig.shutdown().await;
    }

    // Part B: once `armd-persist` exits cleanly after a job (`exit_after_first_job`),
    // subsequent `store_offset=true` staging fails closed immediately with `PERSIST_FAILED`.
    {
        let hooks = PersistTestHooks {
            exit_after_first_job: true,
            ..Default::default()
        };
        let rig = DualRig::start(
            RtOwner::Executor,
            OperatorAuth::disabled(),
            |_| {},
            |inner| {
                inner.set_mock_gripper_stall_offset(Some(0.05));
            },
            hooks,
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-exit-closed", false).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let ep = rig.sup.get_observation().await.epoch_id;

        let (_, term, _) = run_cmd_to_terminal(
            &mut ws,
            gen,
            ep,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
            Duration::from_secs(8),
        )
        .await;
        assert_eq!(term["state"], "done", "{term}");
        assert_eq!(term["durability_confirmed"], json!(true), "{term}");
        rig.wait_zero_check_ctx_empty("after_worker_exit").await;

        // Wait briefly for `ExitFlag::drop` on `armd-persist` to mark `ZeroCheckContextStore` closed.
        let handle = rig.sup.executor_handle().unwrap();
        let exit_dl = Instant::now() + Duration::from_secs(2);
        while !handle.persist_worker_exited() {
            assert!(Instant::now() < exit_dl, "armd-persist did not exit after first job");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        let ep2 = rig.sup.get_observation().await.epoch_id;
        let rej = send_cmd(
            &mut ws,
            gen,
            ep2,
            "/api/v1/action/gripper_zero_check",
            json!({"store_offset": true}),
        )
        .await;
        assert_eq!(rej["type"], "command_reject", "{rej}");
        assert_eq!(rej["code"], "PERSIST_FAILED", "{rej}");
        rig.wait_zero_check_ctx_empty("fail_closed_after_worker_exit").await;
        rig.assert_zero_rt_allocs("fail_closed_after_worker_exit");
        rig.shutdown().await;
    }
}

// ---------------------------------------------------------------------------
// 14. Major 3 regression: no-lease disconnect refuses persist via connection_id
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_no_lease_disconnect_refuses_persist_by_connection_id() {
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |_| {},
        |inner| {
            inner.lease_enabled = false;
            let adm = armd::supervisor::AdmissionStamp {
                stop_gen: 0,
                estop_epoch: inner.safety.epoch_id(),
                lease_gen: 0,
                connection_id: 0,
                request_id: 1,
            };
            let admitted = inner.check_admission(adm).unwrap();
            armd::SupervisorInner::enable_torque(admitted).unwrap();
            inner.set_mock_gripper_stall_offset(Some(0.05));
        },
        PersistTestHooks::default(),
    )
    .await;

    let handle = rig.sup.executor_handle().unwrap();
    handle.attach_external_reply_sink(Arc::new(|_, _| {
        armd::rt_core::broker::DeliveryDecision::Delivered
    }));

    // Open connection 42 with NO lease held (`lease_gen = 0`), submit zero_check(store_offset=true)
    // while holding `config_write_lock` so PR C pauses before Reserve, then disconnect conn 42.
    let conn_id = 42u64;
    handle.open_connection_async(conn_id).await.unwrap().unwrap();
    let svc = rig
        .sup
        .read_services(armd::rt_core::publish::ReadRequirements {
            require_within_valid_until: false,
            require_prep_env: true,
            ..armd::rt_core::publish::ReadRequirements::identity()
        })
        .await
        .unwrap();
    let pub_cfg = rig.sup.published_config().await.unwrap();
    let admission = armd::supervisor::AdmissionStamp {
        stop_gen: svc.applied_stop_generation,
        estop_epoch: svc.snapshot.owner.epoch,
        lease_gen: 0,
        connection_id: conn_id,
        request_id: 8888,
    };
    let deadline = Instant::now() + Duration::from_secs(5);
    let (prepared, persist_ctx) = armd::rt_core::requests::actions::prepare_gripper_zero_check(
        &svc,
        &pub_cfg,
        admission,
        deadline,
        &json!({"store_offset": true}),
        armd::auth::OperatorGrant::unenforced(),
        None,
    )
    .unwrap();
    let ctx_id = handle.stage_zero_check_context(persist_ctx.unwrap()).unwrap();
    let slot = prepared.to_plan_slot(ctx_id).unwrap();
    let write = |dst: &mut PlanSlot| *dst = slot.clone();

    let cw_guard = rig.sup.config_write_lock().lock_owned().await;
    let (_req_id, mut rx, _ack_guard) = handle
        .submit_external_action_with_zero_check_ctx(
            armd::rt_core::broker::ClientId(conn_id),
            0,
            prepared.stamp.broker_authority(),
            deadline,
            armd::rt_core::requests::actions::ActionSubmit::InstallPrepared {
                write: &write,
                kind: armd::rt_core::requests::actions::PreparedActionKind::GripperZeroCheck,
                prepared_boot: prepared.stamp.boot,
                admission_request_id: prepared.stamp.admission.request_id,
                check_workspace_revision: false,
                poses_revision: None,
                calibration_revision: prepared.stamp.calibration_revision,
                gripper_calib_revision: prepared.stamp.gripper_calib_revision,
                power_budget_revision: prepared.stamp.power_budget_revision,
            },
            armd::rt_core::reply_router::ExternalDestinationPolicy::ConnectionOrGlobal,
            false,
            None,
            ctx_id,
        )
        .unwrap();
    let ack_dl = Instant::now() + Duration::from_secs(5);
    let action_id = loop {
        handle.pump_broker_replies();
        match rx.try_recv() {
            Ok(Ok(armd::rt_core::broker::Reply::Ack { action: Some(aid), .. })) => break aid.0,
            Ok(other) => panic!("expected Ack for no-lease zero_check, got {other:?}"),
            Err(tokio::sync::oneshot::error::TryRecvError::Closed) => {
                panic!("broker ack channel closed unexpectedly");
            }
            Err(tokio::sync::oneshot::error::TryRecvError::Empty) => {
                assert!(Instant::now() < ack_dl, "timed out waiting for broker ACK");
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
    };

    // Wait deterministically until seek finishes and enters PR C persistence (`stop_rad` set
    // while `state == "running"`, blocked on `cw_guard`).
    let seek_dl = Instant::now() + Duration::from_secs(5);
    loop {
        let obs = rig.sup.get_observation().await;
        if obs
            .action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.stop_rad.is_some() && a.state == "running")
        {
            break;
        }
        assert!(
            Instant::now() < seek_dl,
            "seek did not enter PR C persistence phase: {:?}",
            obs.action
        );
        tokio::time::sleep(Duration::from_millis(15)).await;
    }
    // Disconnect conn 42 (no lease exists -> ONLY `connection_id` in PersistCmd::Reserve catches it!).
    let conn_handle = armd::rt_core::requests::lease::ConnectionHandle::from_ws_id(conn_id).unwrap();
    let _ = handle.disconnect_connection_async(conn_handle).await.unwrap();
    drop(cw_guard);

    let dl = Instant::now() + Duration::from_secs(5);
    loop {
        handle.pump_broker_replies();
        let obs = rig.sup.get_observation().await;
        if let Some(a) = obs.action.as_ref() {
            if a.id == action_id && a.state != "running" {
                assert_eq!(a.state, "cancelled", "{a:?}");
                assert_eq!(a.code.as_deref(), Some("LEASE_LOST"), "{a:?}");
                break;
            }
        }
        assert!(Instant::now() < dl, "disconnected no-lease zero_check did not terminate");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert!(
        !rig.calib_path.exists(),
        "disconnected connection must not persist calibration when no lease is held"
    );
    rig.wait_zero_check_ctx_empty("no_lease_disconnect").await;
    rig.assert_zero_rt_allocs("no_lease_disconnect");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 15. Major 4 regression: config_write_mutex contention past deadline -> DEADLINE_EXPIRED
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_config_write_mutex_deadline_expired() {
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |_| {},
        |inner| {
            inner.set_mock_gripper_stall_offset(Some(0.05));
        },
        PersistTestHooks::default(),
    )
    .await;

    let mut ws = connect_ws(rig.addr, None).await;
    let gen = acquire_lease(&mut ws, "c-dl", false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;

    // Hold `config_write_lock` past `CONFIG_PROPOSAL_BUDGET` (5s): worker must fail promptly with
    // `DEADLINE_EXPIRED` instead of hanging indefinitely, and release `ExclusiveOutput`.
    let cw_guard = rig.sup.config_write_lock().lock_owned().await;
    let (_, term, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(10),
    )
    .await;
    assert_eq!(term["state"], "failed", "{term}");
    assert_eq!(term["code"], "DEADLINE_EXPIRED", "{term}");
    assert!(!rig.calib_path.exists());
    rig.wait_zero_check_ctx_empty("deadline_expired").await;

    // Release mutex and verify SessionSlot::ExclusiveOutput was cleanly cleared.
    drop(cw_guard);
    let ep2 = rig.sup.get_observation().await.epoch_id;
    let (_, term_ok, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep2,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": true}),
        Duration::from_secs(8),
    )
    .await;
    assert_eq!(term_ok["state"], "done", "{term_ok}");
    assert_eq!(term_ok["durability_confirmed"], json!(true), "{term_ok}");
    assert!(rig.calib_path.exists());
    rig.wait_zero_check_ctx_empty("deadline_expired_recovery").await;
    rig.assert_zero_rt_allocs("deadline_expired");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 16. Major 5 regression: stop during PR C persistence -> single terminal, no slot leak
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_stop_during_persist_settles_without_double_terminal_or_slot_leak() {
    let rig = DualRig::start(
        RtOwner::Executor,
        OperatorAuth::disabled(),
        |_| {},
        |inner| {
            inner.set_mock_gripper_stall_offset(Some(0.05));
        },
        PersistTestHooks::default(),
    )
    .await;

    let mut ws = connect_ws(rig.addr, None).await;
    let gen = acquire_lease(&mut ws, "c-stop-prc", false).await;
    let ep = rig.sup.get_observation().await.epoch_id;
    let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
    let ep = rig.sup.get_observation().await.epoch_id;

    let cw_guard = rig.sup.config_write_lock().lock_owned().await;
    let req_id = next_req_id();
    send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": req_id,
            "generation": gen,
            "epoch_id": ep,
            "path": "/api/v1/action/gripper_zero_check",
            "body": {"store_offset": true},
        }),
    )
    .await;
    let ack = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("request_id").and_then(Value::as_u64) == Some(req_id) {
                return v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(ack["type"], "command_ack", "{ack}");
    let action_id = ack["action_id"].as_u64().unwrap();

    // Wait deterministically until seek finishes and enters PR C persistence (`stop_rad` set
    // while `state == "running"`, blocked on `cw_guard`).
    let seek_dl = Instant::now() + Duration::from_secs(5);
    loop {
        let obs = rig.sup.get_observation().await;
        if obs
            .action
            .as_ref()
            .is_some_and(|a| a.id == action_id && a.stop_rad.is_some() && a.state == "running")
        {
            break;
        }
        assert!(
            Instant::now() < seek_dl,
            "seek did not enter PR C persistence phase before stop: {:?}",
            obs.action
        );
        tokio::time::sleep(Duration::from_millis(15)).await;
    }

    // Issue authorized stop while PR C holds the action in `zero_check_persisting`.
    let stop_ack = send_cmd(&mut ws, gen, ep, "/api/v1/motion/stop", json!({})).await;
    assert_eq!(stop_ack["type"], "command_ack", "{stop_ack}");
    tokio::time::sleep(Duration::from_millis(120)).await;
    drop(cw_guard);

    // Collect distinct terminal transitions `(state, code, finished_at)` for `action_id` and
    // assert exactly one transition to cancelled/STOPPED (no double-terminal).
    let mut terminal_keys: Vec<(Value, Value, Value)> = Vec::new();
    let mut first_term: Option<Value> = None;
    let collect_until = Instant::now() + Duration::from_secs(3);
    let mut saw_terminal_at: Option<Instant> = None;
    while Instant::now() < collect_until {
        if let Some(t) = saw_terminal_at {
            if t.elapsed() >= Duration::from_millis(350) {
                break;
            }
        }
        if let Ok(Some(Ok(msg))) = tokio::time::timeout(Duration::from_millis(50), ws.next()).await {
            let text = msg.into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("type").is_none() {
                if let Some(a) = v.get("action").filter(|a| !a.is_null()) {
                    if a["id"].as_u64() == Some(action_id) && a["state"] != "running" {
                        let key = (a["state"].clone(), a["code"].clone(), a["finished_at"].clone());
                        if !terminal_keys.contains(&key) {
                            terminal_keys.push(key);
                        }
                        if first_term.is_none() {
                            first_term = Some(a.clone());
                        }
                        saw_terminal_at.get_or_insert_with(Instant::now);
                    }
                }
            }
        }
    }
    assert_eq!(
        terminal_keys.len(),
        1,
        "expected exactly one terminal transition for stop during PR C, got: {terminal_keys:?}"
    );
    let term = first_term.expect("missing terminal payload");
    assert_eq!(term["state"], "cancelled", "{term}");
    assert_eq!(term["code"], "STOPPED", "{term}");
    assert!(!rig.calib_path.exists());
    rig.wait_zero_check_ctx_empty("stop_during_prc").await;

    // SessionSlot::ExclusiveOutput must be cleanly released for subsequent actions.
    let ep2 = rig.sup.get_observation().await.epoch_id;
    let (_, term_ok, _) = run_cmd_to_terminal(
        &mut ws,
        gen,
        ep2,
        "/api/v1/action/gripper_zero_check",
        json!({"store_offset": false}),
        Duration::from_secs(8),
    )
    .await;
    assert_eq!(term_ok["state"], "done", "{term_ok}");
    rig.assert_zero_rt_allocs("stop_during_prc");
    rig.shutdown().await;
}

// ---------------------------------------------------------------------------
// 17. Major 6 regression: ThermalTrip and BusLossFallback emit failed + exact code
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn executor_zero_check_thermal_trip_and_bus_loss_report_failed_codes() {
    // (a) ThermalTrip -> state="failed", code="THERMAL_TRIP"
    {
        let driver = ZeroCheckDriver::new(-0.50);
        driver.state.lock().unwrap().never_stall_creep = true;
        let driver_for_inner = driver.clone();
        let rig = DualRig::start(
            RtOwner::Executor,
            OperatorAuth::disabled(),
            |_| {},
            move |inner| {
                let mut q = INSPECT;
                q[6] = -0.50;
                inner.measured_positions = q.to_vec();
                inner.commanded_positions = q.to_vec();
                inner.target_positions = q.to_vec();
                inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(driver_for_inner)));
            },
            PersistTestHooks::default(),
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-thermal", false).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let ep = rig.sup.get_observation().await.epoch_id;

        let req_id = next_req_id();
        send(
            &mut ws,
            json!({
                "type": "command",
                "request_id": req_id,
                "generation": gen,
                "epoch_id": ep,
                "path": "/api/v1/action/gripper_zero_check",
                "body": {"store_offset": false},
            }),
        )
        .await;
        let ack = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: Value = serde_json::from_str(&text).unwrap();
                if v.get("request_id").and_then(Value::as_u64) == Some(req_id) {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let action_id = ack["action_id"].as_u64().unwrap();

        tokio::time::sleep(Duration::from_millis(80)).await;
        driver.state.lock().unwrap().j7_temperature_c = 85.0;

        let term = wait_terminal_for_id(&mut ws, action_id, Duration::from_secs(4)).await;
        assert_eq!(term["state"], "failed", "{term}");
        assert_eq!(term["code"], "THERMAL_TRIP", "{term}");
        rig.assert_zero_rt_allocs("thermal_trip");
        rig.shutdown().await;
    }

    // (b) BusLossFallback -> state="failed", code="BUS_LOSS"
    {
        let driver = ZeroCheckDriver::new(-0.50);
        driver.state.lock().unwrap().never_stall_creep = true;
        let driver_for_inner = driver.clone();
        let rig = DualRig::start(
            RtOwner::Executor,
            OperatorAuth::disabled(),
            |_| {},
            move |inner| {
                let mut q = INSPECT;
                q[6] = -0.50;
                inner.measured_positions = q.to_vec();
                inner.commanded_positions = q.to_vec();
                inner.target_positions = q.to_vec();
                inner.driver = Some(armd::supervisor_lock::DriverSlot::legacy(Box::new(driver_for_inner)));
            },
            PersistTestHooks::default(),
        )
        .await;

        let mut ws = connect_ws(rig.addr, None).await;
        let gen = acquire_lease(&mut ws, "c-busloss", false).await;
        let ep = rig.sup.get_observation().await.epoch_id;
        let _ = send_cmd(&mut ws, gen, ep, "/api/v1/torque", json!({"enable": true})).await;
        let ep = rig.sup.get_observation().await.epoch_id;

        let req_id = next_req_id();
        send(
            &mut ws,
            json!({
                "type": "command",
                "request_id": req_id,
                "generation": gen,
                "epoch_id": ep,
                "path": "/api/v1/action/gripper_zero_check",
                "body": {"store_offset": false},
            }),
        )
        .await;
        let ack = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
                let v: Value = serde_json::from_str(&text).unwrap();
                if v.get("request_id").and_then(Value::as_u64) == Some(req_id) {
                    return v;
                }
            }
        })
        .await
        .unwrap();
        assert_eq!(ack["type"], "command_ack", "{ack}");
        let action_id = ack["action_id"].as_u64().unwrap();

        tokio::time::sleep(Duration::from_millis(80)).await;
        driver.state.lock().unwrap().bus_loss_stale = true;

        let term = wait_terminal_for_id(&mut ws, action_id, Duration::from_secs(4)).await;
        assert_eq!(term["state"], "failed", "{term}");
        assert_eq!(term["code"], "BUS_LOSS", "{term}");
        rig.assert_zero_rt_allocs("bus_loss");
        rig.shutdown().await;
    }
}
