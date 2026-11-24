//! `--follower sim`: an [`ArmDriver`] backed by a physics simulator over UDP
//! (`sim/mujoco`, `rebot-sim`). See docs/simulation.md for the wire protocol.
//!
//! The driver is called inside the control tick under the supervisor write lock, so no
//! call blocks on the network: a background thread receives state datagrams into a cache
//! that `read_joint_states` copies. Only `set_torque` waits, bounded like the Damiao
//! enable handshake (80 ms enable, 25 ms disable), for the simulator to confirm.
//!
//! Wire protocol (version 1), one JSON object per UDP datagram:
//! - armd -> sim `{"v":1,"type":"cmd","seq":N,"enable":bool|null,"targets":[...]|null}`;
//!   a target is `{"mode":"pos_vel","q":rad,"vlim":rad/s}` or
//!   `{"mode":"mit","q":rad,"dq":rad/s,"kp":Nm/rad,"kd":Nm·s/rad,"tau":Nm}`.
//!   `enable` is explicit (`true`/`false`) only on enable/disable commands (`targets: null`);
//!   target packets omit `enable` so they preserve the simulator's current torque state and can
//!   never energize a disabled or restarted peer under E-stop.
//! - sim -> armd `{"v":1,"type":"state","session":u64,"seq":N,"ack":N,"enabled":bool,
//!   "q":[..],"dq":[..],"tau":[..],"temp":[..],"t":s}` with one entry per joint.
//!   `ack` is the highest command `seq` applied; `session` changes when the simulator restarts.
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::driver::handshake::Deadline;
use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget, MotorZeroSupport, TorqueOutcome, TorqueProgress};
use crate::model::JointState;

/// Default simulator address for `--follower sim` without `--follower-port`.
pub const DEFAULT_SIM_ADDR: &str = "127.0.0.1:47600";
/// Feedback older than this is not valid (`is_feedback_valid` false, enable state
/// Unknown); the supervisor's own BUS_LOSS bound then applies to the ages.
pub const SIM_FEEDBACK_STALE: Duration = Duration::from_millis(200);
/// Keep-alive interval: the simulator learns armd's address from any datagram.
const KEEPALIVE: Duration = Duration::from_millis(20);
const ENABLE_CONFIRM_TIMEOUT: Duration = Duration::from_millis(80);
const DISABLE_CONFIRM_TIMEOUT: Duration = Duration::from_millis(25);
/// Velocity limit sent with POS_VEL targets, as the Damiao driver's default VLIM.
const POS_VEL_VLIM: f32 = 2.0;
const PROTOCOL_VERSION: u32 = 1;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum SimTarget {
    PosVel { q: f32, vlim: f32 },
    Mit { q: f32, dq: f32, kp: f32, kd: f32, tau: f32 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimCommand {
    pub v: u32,
    #[serde(rename = "type")]
    pub kind: String,
    pub seq: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub enable: Option<bool>,
    pub targets: Option<Vec<SimTarget>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimState {
    pub v: u32,
    #[serde(rename = "type")]
    pub kind: String,
    pub session: u64,
    pub seq: u64,
    pub ack: u64,
    pub enabled: bool,
    pub q: Vec<f32>,
    pub dq: Vec<f32>,
    pub tau: Vec<f32>,
    pub temp: Vec<f32>,
    #[serde(default)]
    pub loads: Option<Vec<f32>>,
    #[serde(default)]
    pub t: f64,
}

/// Parse and validate one state datagram for `dof` joints.
pub fn parse_state(bytes: &[u8], dof: usize) -> Option<SimState> {
    let state: SimState = serde_json::from_slice(bytes).ok()?;
    let arrays = [&state.q, &state.dq, &state.tau, &state.temp];
    let basic_valid = state.v == PROTOCOL_VERSION && state.kind == "state"
        && arrays.iter().all(|a| a.len() == dof && a.iter().all(|v| v.is_finite()));
    if !basic_valid {
        return None;
    }
    if let Some(ref loads) = state.loads {
        if loads.len() != dof || !loads.iter().all(|v| v.is_finite()) {
            return None;
        }
    }
    Some(state)
}

#[derive(Debug, Default)]
struct Shared {
    latest: Option<SimState>,
    received_at: Option<Instant>,
    continuity_epoch: u64,
}

/// Opaque test handle allowing another thread to hold `SimArmDriver`'s internal RX mutex
/// to verify that RT releases never block behind `armd-sim-rx`.
#[doc(hidden)]
pub struct SimRxLockHandle(Arc<Mutex<Shared>>);

#[doc(hidden)]
impl SimRxLockHandle {
    pub fn lock(&self) -> impl Drop + '_ {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Opaque test guard simulating `armd-sim-rx` paused inside its keepalive send window,
/// verifying that `set_rt_mode(true)` sleeps on the condvar rather than busy-yielding.
#[doc(hidden)]
pub struct SimRxKeepaliveGuard(Arc<(Mutex<bool>, Condvar)>);

impl Drop for SimRxKeepaliveGuard {
    fn drop(&mut self) {
        let mut active = self.0 .0.lock().unwrap_or_else(|e| e.into_inner());
        *active = false;
        self.0 .1.notify_all();
    }
}

/// Datagrams not sent because the socket buffer was full (non-blocking send).
static SEND_DROPS: AtomicU64 = AtomicU64::new(0);

/// Send without blocking the control tick (the socket itself stays blocking for the
/// receiver thread, which shares it): a full buffer drops the datagram and counts it, like a
/// lost CAN frame. Other errors are ignored for the same reason.
fn send_nonblocking(socket: &UdpSocket, bytes: &[u8]) {
    use std::os::fd::AsRawFd;
    // SAFETY: valid fd and buffer for the duration of the call.
    let n = unsafe { libc::send(socket.as_raw_fd(), bytes.as_ptr().cast(), bytes.len(), libc::MSG_DONTWAIT) };
    if n < 0 && std::io::Error::last_os_error().kind() == std::io::ErrorKind::WouldBlock {
        SEND_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}

#[derive(Serialize)]
struct SimCommandRef<'a> {
    v: u32,
    #[serde(rename = "type")]
    kind: &'static str,
    seq: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    enable: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    targets: Option<&'a [SimTarget]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    hold_targets: Option<&'a [Option<SimTarget>]>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SendGate {
    /// Target write: inhibited while an unapplied urgent E-stop is pending (an applied E-stop
    /// still holds position with active torque).
    Target,
    /// Enable command: inhibited whenever E-stop is latched or pending.
    Enable,
    /// Requested disable command: subject to the stopped rest-pose rule and 1 s request lifetime
    /// while stopped, OR unconditionally in every safety state when `rollback` is true.
    DisableRequested { started_at: Instant, rollback: bool },
    /// Protective disable command (`Thermal`, `ProcessExit`): never refused.
    DisableProtective { reason: crate::driver::DisableProtectiveReason },
}

pub struct SimArmDriver {
    socket: UdpSocket,
    dof: usize,
    shared: Arc<Mutex<Shared>>,
    rejected: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    rt_mode: Arc<AtomicBool>,
    rx_keepalive_sync: Arc<(Mutex<bool>, Condvar)>,
    rx_keepalives_sent: Arc<AtomicU64>,
    owner_keepalives_sent: u64,
    last_send_ms: Arc<AtomicU64>,
    epoch: Instant,
    thread: Option<std::thread::JoinHandle<()>>,
    seq: u64,
    enable_requested: bool,
    /// Seq of the last enable / disable command; enable state is Unknown until acked.
    enable_seq: u64,
    modes: Vec<JointMode>,
    cached: Vec<JointState>,
    cached_loads: Vec<f32>,
    cached_torques: Vec<f32>,
    /// Owner-local coherent metadata updated from non-blocking `try_lock()` in RT mode.
    cached_ack: u64,
    cached_enabled: bool,
    cached_received_at: Option<Instant>,
    cached_continuity_epoch: u64,
    cached_has_state: bool,
    /// Preallocated outbound target scratch, sparse hold scratch, JSON serialization buffer, and RT poll keepalive (0 alloc).
    cmd_targets: Vec<SimTarget>,
    cmd_hold_targets: Vec<Option<SimTarget>>,
    tx_buf: Vec<u8>,
    keepalive_bytes: Vec<u8>,
    load_only_feedback: bool,
    /// Simulated encoder zero (raw simulator angle of each joint's zero); None = the
    /// simulator's own zero. Set by `set_zero_position`, lost when armd restarts.
    zero_offsets: Option<Vec<f32>>,
    /// Pending torque transition: requested state and the deadline of its acknowledgement.
    torque_hs: Option<(bool, Deadline)>,
    /// Physical E-stop latch inhibiting any new enable command.
    abort_flag: Option<Arc<AtomicBool>>,
    /// Authoritative packed E-stop state (`ArmDriver::set_estop_state`).
    estop_state: Option<Arc<crate::rt_core::urgent::EstopState>>,
    /// Pending-E-stop gate checked immediately before target and enable sends.
    estop_gate: Option<crate::rt_core::urgent::EstopGate>,
    /// Supervisor-injected stopped-disable gate enforced at send time for `DisableRequested`.
    stopped_disable_gate: Option<crate::driver::StoppedDisableGate>,
    /// Optional RT release cutoff deadline enforced right before UDP submission.
    tx_release_deadline: Option<Instant>,
    /// Deterministic test hook invoked inside `send_ref` strictly after JSON serialization and
    /// immediately before the final E-stop send gate and socket transmit.
    before_send_hook: Option<Arc<dyn Fn() + Send + Sync>>,
}

impl SimArmDriver {
    /// Connect to the simulator at `addr` (`udp://host:port` or `host:port`).
    pub fn connect(addr: &str, dof: usize) -> std::io::Result<Self> {
        let peer = resolve(addr)?;
        let bind: SocketAddr = if peer.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" }.parse().expect("literal");
        let socket = UdpSocket::bind(bind)?;
        socket.connect(peer)?;
        socket.set_read_timeout(Some(Duration::from_millis(10)))?;
        let shared = Arc::new(Mutex::new(Shared::default()));
        let rejected = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let rt_mode = Arc::new(AtomicBool::new(false));
        let rx_keepalive_sync = Arc::new((Mutex::new(false), Condvar::new()));
        let rx_keepalives_sent = Arc::new(AtomicU64::new(0));
        let last_send_ms = Arc::new(AtomicU64::new(0));
        let epoch = Instant::now();
        let keepalive_bytes = serde_json::to_vec(&SimCommand {
            v: PROTOCOL_VERSION,
            kind: "poll".into(),
            seq: 0,
            enable: None,
            targets: None,
        })
        .expect("serialisable");
        let thread = {
            let (socket, shared, rejected, stop, rt_mode, rx_keepalive_sync, rx_keepalives_sent, last_send_ms, keepalive) = (
                socket.try_clone()?,
                shared.clone(),
                rejected.clone(),
                stop.clone(),
                rt_mode.clone(),
                rx_keepalive_sync.clone(),
                rx_keepalives_sent.clone(),
                last_send_ms.clone(),
                keepalive_bytes.clone(),
            );
            std::thread::Builder::new().name("armd-sim-rx".into()).spawn(move || {
                receiver_loop(
                    socket,
                    dof,
                    shared,
                    rejected,
                    stop,
                    rt_mode,
                    rx_keepalive_sync,
                    rx_keepalives_sent,
                    last_send_ms,
                    epoch,
                    keepalive,
                )
            })?
        };
        Ok(Self {
            socket, dof, shared, rejected, stop, rt_mode, rx_keepalive_sync, rx_keepalives_sent,
            owner_keepalives_sent: 0, last_send_ms, epoch, thread: Some(thread),
            seq: 0, enable_requested: false, enable_seq: 0,
            modes: vec![JointMode::Position; dof],
            cached: vec![JointState::default(); dof],
            cached_loads: vec![0.0; dof],
            cached_torques: vec![0.0; dof],
            cached_ack: 0,
            cached_enabled: false,
            cached_received_at: None,
            cached_continuity_epoch: 0,
            cached_has_state: false,
            cmd_targets: Vec::with_capacity(dof),
            cmd_hold_targets: Vec::with_capacity(dof),
            tx_buf: Vec::with_capacity(1024),
            keepalive_bytes,
            load_only_feedback: false,
            zero_offsets: None,
            torque_hs: None,
            abort_flag: None,
            estop_state: None,
            estop_gate: None,
            stopped_disable_gate: None,
            tx_release_deadline: None,
            before_send_hook: None,
        })
    }

    pub fn with_load_only_feedback(mut self, enabled: bool) -> Self {
        self.load_only_feedback = enabled;
        self
    }

    /// Installs a deterministic test hook invoked inside `send_ref` right after serialization and
    /// immediately before the final E-stop send gate + `send_nonblocking`.
    #[doc(hidden)]
    pub fn set_before_send_hook(&mut self, hook: Option<Arc<dyn Fn() + Send + Sync>>) {
        self.before_send_hook = hook;
    }

    /// Count of keepalive polls transmitted by the background `armd-sim-rx` thread (must remain
    /// constant after RT ownership transfer via `set_rt_mode(true)`).
    pub fn rx_keepalives_sent(&self) -> u64 { self.rx_keepalives_sent.load(Ordering::Acquire) }

    /// Shared handle to the `armd-sim-rx` keepalive counter for external single-writer audits.
    pub fn rx_keepalives_counter(&self) -> Arc<AtomicU64> { Arc::clone(&self.rx_keepalives_sent) }

    /// Count of keepalive polls transmitted by the RT owner thread.
    pub fn owner_keepalives_sent(&self) -> u64 { self.owner_keepalives_sent }

    /// Test helper: cloneable handle to the internal RX shared mutex so another thread can hold
    /// it while RT releases execute (proving strictly non-blocking RT reads).
    #[doc(hidden)]
    pub fn test_rx_lock_handle(&self) -> SimRxLockHandle {
        SimRxLockHandle(Arc::clone(&self.shared))
    }

    /// Test helper: mark the RX keepalive handshake active until the returned guard drops, so tests
    /// can verify `set_rt_mode(true)` waits cleanly on the condvar without spinning.
    #[doc(hidden)]
    pub fn test_hold_rx_keepalive(&self) -> SimRxKeepaliveGuard {
        let mut active = self.rx_keepalive_sync.0.lock().unwrap_or_else(|e| e.into_inner());
        *active = true;
        drop(active);
        SimRxKeepaliveGuard(Arc::clone(&self.rx_keepalive_sync))
    }

    /// Datagrams dropped as malformed, out of order or for the wrong joint count (lock-free).
    pub fn rejected_datagrams(&self) -> u64 { self.rejected.load(Ordering::Relaxed) }

    /// Commands dropped because the send buffer was full (all sim drivers of the process).
    pub fn dropped_sends() -> u64 { SEND_DROPS.load(Ordering::Relaxed) }

    /// Return measured joint torques for all joints (raw simulator tau).
    pub fn joint_torques(&self) -> Vec<f32> {
        self.cached_torques.clone()
    }

    /// Return normalized load fractions in `[-1.0, 1.0]` when supported (load_only_feedback).
    pub fn joint_loads(&self) -> Option<Vec<f32>> {
        if self.load_only_feedback {
            Some(self.cached_loads.clone())
        } else {
            None
        }
    }

    /// Err([`ArmDriverError::EstopInhibited`]) while an E-stop request is pending and not yet
    /// applied by the release (once applied, E-stop holds position with active torque).
    fn estop_check(&self) -> Result<(), ArmDriverError> {
        if self.estop_gate.as_ref().is_some_and(|g| g.pending()) {
            return Err(ArmDriverError::EstopInhibited);
        }
        Ok(())
    }

    /// Aborts an enable attempt if E-stop is latched or an urgent stop is pending.
    fn enable_estop_inhibited(&mut self) -> Result<(), ArmDriverError> {
        let latched = self.estop_state.as_ref().is_some_and(|s| s.is_latched())
            || self.abort_flag.as_ref().is_some_and(|f| f.load(Ordering::SeqCst));
        let pending = self.estop_gate.as_ref().is_some_and(|g| g.pending());
        if latched || pending {
            if matches!(self.torque_hs, Some((true, _))) {
                self.torque_hs = None;
            }
            self.enable_requested = false;
            return Err(ArmDriverError::EstopInhibited);
        }
        Ok(())
    }

    /// Enforces Rule A′ and the single 1 s request lifetime at send time for `DisableRequested`
    /// while safety state is stopped (Fault or E-stop) OR unconditionally when `rollback` is
    /// true, reading timestamp & positions from the same coherent simulator feedback packet.
    /// Zero heap allocations.
    fn check_stopped_disable_requested(
        &self,
        started_at: Instant,
        rollback: bool,
    ) -> Result<(), ArmDriverError> {
        let stopped = self
            .stopped_disable_gate
            .as_ref()
            .is_some_and(crate::driver::StoppedDisableGate::is_safety_stopped)
            || self.abort_flag.as_ref().is_some_and(|f| f.load(Ordering::SeqCst))
            || self.estop_gate.as_ref().is_some_and(|g| g.pending());
        if !rollback && !stopped {
            return Ok(());
        }
        if Instant::now().saturating_duration_since(started_at)
            >= crate::driver::STOPPED_DISABLE_REQUEST_LIFETIME
        {
            return Err(ArmDriverError::DisableRequestExpired);
        }
        let Some(ref gate) = self.stopped_disable_gate else {
            return Err(ArmDriverError::TorqueOffWouldDrop);
        };
        let arm_joints = gate.arm_joint_indices();
        let mut arm_positions = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        if !self.fresh_joint_positions_into(
            arm_joints,
            crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE,
            &mut arm_positions[..arm_joints.len()],
        ) {
            return Err(ArmDriverError::TorqueOffWouldDrop);
        }
        if !gate.check_positions(&arm_positions[..arm_joints.len()]) {
            return Err(ArmDriverError::TorqueOffWouldDrop);
        }
        Ok(())
    }

    #[inline]
    fn current_stop_stamp(&self) -> (u64, u64) {
        self.estop_gate
            .as_ref()
            .map_or((0, 0), crate::rt_core::urgent::EstopGate::generations)
    }

    fn send_ref(
        &mut self,
        with_targets: bool,
        with_hold_targets: bool,
        enable: Option<bool>,
        gate: SendGate,
        expected_stamp: Option<(u64, u64)>,
    ) -> Result<u64, ArmDriverError> {
        let next_seq = self.seq + 1;
        let cmd = SimCommandRef {
            v: PROTOCOL_VERSION,
            kind: "cmd",
            seq: next_seq,
            enable,
            targets: with_targets.then_some(self.cmd_targets.as_slice()),
            hold_targets: with_hold_targets.then_some(self.cmd_hold_targets.as_slice()),
        };
        self.tx_buf.clear();
        let serialized = serde_json::to_writer(&mut self.tx_buf, &cmd).is_ok();
        if let Some(hook) = self.before_send_hook.as_ref() {
            hook();
        }
        // Final send-time release cutoff, stop-generation stamp, and Rule A′ gate right before
        // the socket send.
        match gate {
            SendGate::Target => {
                if self.tx_release_deadline.is_some_and(|dl| Instant::now() >= dl) {
                    return Err(ArmDriverError::Transport(crate::transport::TransportError::Timeout));
                }
                self.estop_check()?;
                if expected_stamp.is_some_and(|s| self.current_stop_stamp() != s) {
                    return Err(ArmDriverError::EstopInhibited);
                }
            }
            SendGate::Enable => {
                if self.tx_release_deadline.is_some_and(|dl| Instant::now() >= dl) {
                    return Err(ArmDriverError::Transport(crate::transport::TransportError::Timeout));
                }
                self.enable_estop_inhibited()?;
                if expected_stamp.is_some_and(|s| self.current_stop_stamp() != s) {
                    return Err(ArmDriverError::EstopInhibited);
                }
            }
            SendGate::DisableRequested { started_at, rollback } => {
                self.check_stopped_disable_requested(started_at, rollback)?;
            }
            SendGate::DisableProtective { .. } => {}
        }
        self.seq = next_seq;
        // Invalidate torque evidence BEFORE any torque-changing write that could reach the
        // simulator (deferred/refused writes exited above and left `enable_seq` intact).
        if enable.is_some() {
            self.enable_seq = next_seq;
        }
        if serialized {
            // A lost datagram is equivalent to a lost CAN frame: the missing feedback,
            // not the write error, is what the supervisor acts on (BUS_LOSS).
            send_nonblocking(&self.socket, &self.tx_buf);
            self.last_send_ms.store(self.epoch.elapsed().as_millis() as u64, Ordering::Relaxed);
        }
        Ok(self.seq)
    }

    /// In RT mode, the single RT owner transmits keepalive polls on idle periods instead of
    /// `armd-sim-rx`, preserving the single transport-writer invariant.
    fn maybe_send_owner_keepalive(&mut self) {
        if !self.rt_mode.load(Ordering::Relaxed) {
            return;
        }
        let now_ms = self.epoch.elapsed().as_millis() as u64;
        if now_ms.saturating_sub(self.last_send_ms.load(Ordering::Relaxed)) >= KEEPALIVE.as_millis() as u64 {
            send_nonblocking(&self.socket, &self.keepalive_bytes);
            self.last_send_ms.store(now_ms, Ordering::Relaxed);
            self.owner_keepalives_sent += 1;
        }
    }

    /// Refreshes the owner-local coherent snapshot from the receiver cache. In RT mode uses
    /// strictly non-blocking `try_lock()`, retaining the last coherent snapshot if contended.
    fn refresh_from_rx(&mut self) {
        let guard = if self.rt_mode.load(Ordering::Relaxed) {
            self.shared.try_lock().ok()
        } else {
            self.shared.lock().ok()
        };
        if let Some(s) = guard {
            if let Some(ref state) = s.latest {
                self.cached_ack = state.ack;
                self.cached_enabled = state.enabled;
                self.cached_received_at = s.received_at;
                self.cached_continuity_epoch = s.continuity_epoch;
                self.cached_has_state = true;
                self.cached_torques.clear();
                self.cached_torques.extend_from_slice(&state.tau);
                if self.load_only_feedback {
                    self.cached_loads.clear();
                    if let Some(ref loads) = state.loads {
                        if loads.len() == self.dof {
                            self.cached_loads.extend(loads.iter().map(|&l| l.clamp(-1.0, 1.0)));
                        } else {
                            self.cached_loads.extend(state.tau.iter().map(|&t| (t.abs() / 2.94).clamp(0.0, 1.0)));
                        }
                    } else {
                        self.cached_loads.extend(state.tau.iter().map(|&t| (t.abs() / 2.94).clamp(0.0, 1.0)));
                    }
                }
                let off = self.zero_offsets.as_deref();
                for (i, js) in self.cached.iter_mut().enumerate() {
                    *js = JointState {
                        position: off.map_or(state.q[i], |o| state.q[i] - o[i]),
                        velocity: state.dq[i],
                        torque: if self.load_only_feedback { 0.0 } else { state.tau[i] },
                        temperature: state.temp[i],
                    };
                }
            }
        }
    }

    /// Coherent `(Option<(ack, enabled)>, received_at)` without blocking in RT mode (reads
    /// owner-local cached metadata populated by `refresh_from_rx`).
    fn rx_metadata(&self) -> (Option<(u64, bool)>, Option<Instant>) {
        if self.rt_mode.load(Ordering::Relaxed) {
            let st = self.cached_has_state.then_some((self.cached_ack, self.cached_enabled));
            return (st, self.cached_received_at);
        }
        let Ok(s) = self.shared.lock() else { return (None, None); };
        (s.latest.as_ref().map(|st| (st.ack, st.enabled)), s.received_at)
    }

    fn snapshot(&self) -> (Option<SimState>, Option<Duration>) {
        self.shared.lock().map_or((None, None), |s| (s.latest.clone(), s.received_at.map(|t| t.elapsed())))
    }

    /// The simulator applied command `seq` and reports `enabled` in fresh feedback (no allocation,
    /// no blocking lock in RT mode).
    fn acked(&self, seq: u64, enabled: bool) -> bool {
        matches!(
            self.rx_metadata(),
            (Some((ack, en)), Some(rx)) if ack >= seq && en == enabled && rx.elapsed() <= SIM_FEEDBACK_STALE
        )
    }
}

impl Drop for SimArmDriver {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(thread) = self.thread.take() { let _ = thread.join(); }
    }
}

fn resolve(addr: &str) -> std::io::Result<SocketAddr> {
    let host = addr.strip_prefix("udp://").unwrap_or(addr);
    host.to_socket_addrs()?.next()
        .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("cannot resolve {addr}")))
}

#[allow(clippy::too_many_arguments)]
fn receiver_loop(
    socket: UdpSocket,
    dof: usize,
    shared: Arc<Mutex<Shared>>,
    rejected: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    rt_mode: Arc<AtomicBool>,
    rx_keepalive_sync: Arc<(Mutex<bool>, Condvar)>,
    rx_keepalives_sent: Arc<AtomicU64>,
    last_send_ms: Arc<AtomicU64>,
    epoch: Instant,
    keepalive: Vec<u8>,
) {
    let mut buf = [0u8; 4096];
    while !stop.load(Ordering::SeqCst) {
        if !rt_mode.load(Ordering::Acquire) {
            let now_ms = epoch.elapsed().as_millis() as u64;
            if now_ms.saturating_sub(last_send_ms.load(Ordering::Relaxed)) >= KEEPALIVE.as_millis() as u64 {
                {
                    let mut active = rx_keepalive_sync.0.lock().unwrap_or_else(|e| e.into_inner());
                    *active = true;
                }
                if !rt_mode.load(Ordering::SeqCst) {
                    send_nonblocking(&socket, &keepalive);
                    last_send_ms.store(now_ms, Ordering::Relaxed);
                    rx_keepalives_sent.fetch_add(1, Ordering::Relaxed);
                }
                {
                    let mut active = rx_keepalive_sync.0.lock().unwrap_or_else(|e| e.into_inner());
                    *active = false;
                    rx_keepalive_sync.1.notify_all();
                }
            }
        }
        let n = match socket.recv(&mut buf) {
            Ok(n) => n,
            // Timeout, or ECONNREFUSED while no simulator listens: keep polling.
            Err(_) => continue,
        };
        // Parse and drop replaced heap state strictly outside the lock; the lock only guards the swap.
        let parsed = parse_state(&buf[..n], dof);
        let displaced = {
            let Ok(mut s) = shared.lock() else { return };
            match parsed {
                // Same session: drop duplicates and reordered datagrams. A new session
                // (simulator restart) starts a new sequence.
                Some(state) if s.latest.as_ref().is_none_or(|l| l.session != state.session || state.seq > l.seq) => {
                    let now = Instant::now();
                    let gap_broken = s.latest.as_ref().is_some_and(|l| l.session != state.session)
                        || s.received_at.is_none_or(|prev| {
                            now.saturating_duration_since(prev) > crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE
                        });
                    if gap_broken {
                        s.continuity_epoch = s.continuity_epoch.wrapping_add(1);
                    }
                    let prev = s.latest.replace(state);
                    s.received_at = Some(now);
                    prev
                }
                _ => {
                    rejected.fetch_add(1, Ordering::Relaxed);
                    None
                }
            }
        };
        drop(displaced);
    }
}

impl ArmDriver for SimArmDriver {
    fn capabilities(&self) -> DriverCapabilities {
        if self.load_only_feedback {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: false,
                temperature_feedback: true,
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: true,
            }
        } else {
            DriverCapabilities {
                velocity_feedback: true,
                torque_feedback: true,
                temperature_feedback: true,
                // Same as the Damiao driver in its POS_VEL base mode: feed-forward only
                // through the per-joint MIT switch used by gravity compensation.
                torque_feedforward: false,
                enable_feedback: true,
                load_feedback: false,
            }
        }
    }

    fn enable_state(&self) -> EnableState {
        match self.rx_metadata() {
            (Some((ack, enabled)), Some(rx)) if rx.elapsed() <= SIM_FEEDBACK_STALE && ack >= self.enable_seq => {
                if enabled { EnableState::Enabled } else { EnableState::Disabled }
            }
            _ => EnableState::Unknown,
        }
    }

    fn has_driver_fresh_position_api(&self) -> bool { true }

    fn fresh_measured_position_for(&self, joint: usize, max_age: Duration) -> Option<f32> {
        let mut pos = [0.0f32; 1];
        self.fresh_joint_positions_into(&[joint], max_age, &mut pos)
            .then_some(pos[0])
    }

    fn fresh_measured_sample_for(&self, joint: usize, max_age: Duration) -> Option<(f32, u64)> {
        let pos = self.fresh_measured_position_for(joint, max_age)?;
        let epoch = if self.rt_mode.load(Ordering::Relaxed) {
            self.cached_continuity_epoch
        } else {
            self.shared.lock().map_or(self.cached_continuity_epoch, |s| s.continuity_epoch)
        };
        Some((pos, epoch))
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let age = self.rx_metadata().1.map(|t| t.elapsed());
        vec![age; self.dof]
    }

    fn feedback_all_within(&self, dof: usize, max_age: Duration) -> bool {
        if dof != self.dof { return false; }
        self.rx_metadata().1.map(|t| t.elapsed()).is_some_and(|age| age <= max_age)
    }

    fn feedback_joints_within(&self, joints: &[usize], max_age: Duration) -> bool {
        if joints.iter().any(|&i| i >= self.dof) { return false; }
        self.rx_metadata().1.map(|t| t.elapsed()).is_some_and(|age| age <= max_age)
    }

    fn fresh_joint_positions_into(
        &self,
        joints: &[usize],
        max_age: Duration,
        out: &mut [f32],
    ) -> bool {
        if joints.is_empty() || out.len() < joints.len() || joints.iter().any(|&i| i >= self.dof) {
            return false;
        }
        let caps = self.capabilities();
        // Always read BOTH timestamp and joint states from the receiver's latest coherent packet
        // so the send gate never acts on a superseded owner-local pose when `armd-sim-rx` already
        // holds a newer (e.g. raised) pose. In RT mode use strictly non-blocking `try_lock()` and
        // fail closed if contended (zero allocation, zero blocking).
        let s = if self.rt_mode.load(Ordering::Relaxed) {
            match self.shared.try_lock() {
                Ok(g) => g,
                Err(_) => return false,
            }
        } else {
            match self.shared.lock() {
                Ok(g) => g,
                Err(_) => return false,
            }
        };
        if !s.received_at.is_some_and(|t| t.elapsed() <= max_age) {
            return false;
        }
        let Some(ref state) = s.latest else {
            return false;
        };
        let off = self.zero_offsets.as_deref();
        for (slot, &i) in out.iter_mut().zip(joints.iter()) {
            let (Some(&raw_q), Some(&dq), Some(&tau), Some(&temp)) = (
                state.q.get(i),
                state.dq.get(i),
                state.tau.get(i),
                state.temp.get(i),
            ) else {
                return false;
            };
            let js = JointState {
                position: off.map_or(raw_q, |o| raw_q - o[i]),
                velocity: dq,
                torque: if self.load_only_feedback { 0.0 } else { tau },
                temperature: temp,
            };
            if !crate::driver::joint_state_valid(&js, caps) {
                return false;
            }
            if caps.load_feedback {
                let load_ok = match state.loads.as_ref() {
                    Some(loads) if loads.len() == self.dof => loads.get(i).is_some_and(|l| l.is_finite()),
                    _ => tau.is_finite(),
                };
                if !load_ok {
                    return false;
                }
            }
            *slot = js.position;
        }
        true
    }

    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let age = self.rx_metadata().1.map(|t| t.elapsed());
        out.clear();
        out.resize(self.dof, age);
    }

    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let age_ms = self
            .rx_metadata()
            .1
            .map(|t| t.elapsed().as_millis().min(u64::MAX as u128) as u64);
        out.clear();
        out.resize(self.dof, age_ms);
    }

    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let rx = self.rx_metadata().1;
        out.clear();
        out.resize(self.dof, rx);
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        let rx = self.rx_metadata().1;
        vec![rx; self.dof]
    }

    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let rx = self.rx_metadata().1;
        out.clear();
        out.resize(self.dof, rx);
    }

    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        out.clear();
    }

    fn feedback_queue_bounds_into(&self, out: &mut Vec<Option<Instant>>) {
        out.clear();
    }

    fn take_recovery_writes_into(&mut self, out: &mut Vec<usize>) {
        out.clear();
    }

    fn joint_loads_into(&self, out: &mut Vec<f32>) -> bool {
        out.clear();
        if self.load_only_feedback {
            out.extend_from_slice(&self.cached_loads);
            true
        } else {
            false
        }
    }

    fn motor_status_into(&self, out: &mut Vec<u8>) -> bool {
        out.clear();
        match self.enable_state() {
            EnableState::Enabled => {
                out.resize(self.dof, 1);
                true
            }
            EnableState::Disabled => {
                out.resize(self.dof, 0);
                true
            }
            EnableState::Unknown => false,
        }
    }

    fn max_command_velocity(&self, joint: usize) -> Option<f32> {
        (joint < self.dof && self.modes[joint] == JointMode::Position).then_some(POS_VEL_VLIM)
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(self.dof);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }

    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        self.refresh_from_rx();
        out.clear();
        out.extend_from_slice(&self.cached);
        Ok(())
    }

    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _deadline: Instant, _max_rx_frames: u16) -> Result<(u16, bool), ArmDriverError> {
        self.maybe_send_owner_keepalive();
        self.read_joint_states_into(out)?;
        Ok((0, false))
    }

    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        out.clear();
        out.extend_from_slice(&self.cached);
        Some(())
    }

    fn clear_fault_budgeted(&mut self, _deadline: Instant) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn joint_loads(&self) -> Option<Vec<f32>> {
        if self.load_only_feedback {
            Some(self.cached_loads.clone())
        } else {
            None
        }
    }

    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let n = self.dof.min(64);
        let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(mask, targets).map(|_| ())
    }

    fn write_estop_hold_bounded(&mut self, targets: &[JointTarget]) -> Result<crate::driver::TxBatchOutcome, ArmDriverError> {
        let n = self.dof.min(64);
        let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        self.write_measured_hold_masked(mask, targets).map(|r| r.outcome)
    }

    fn write_measured_hold_masked(
        &mut self,
        hold_mask: u64,
        targets: &[JointTarget],
    ) -> Result<crate::driver::HoldPassReport, ArmDriverError> {
        use crate::driver::{HoldPassReport, JointHoldStatus};
        if self.torque_hs.is_some() {
            return Err(ArmDriverError::Protocol("torque transition pending: no hold before its confirmation".into()));
        }
        if targets.len() != self.dof {
            return Err(ArmDriverError::JointCountMismatch { expected: self.dof, actual: targets.len() });
        }
        let dof = self.dof.min(crate::rt_core::mailbox::MAX_DOF);
        let mut report = HoldPassReport::empty(dof);
        let mut held_mask = 0u64;
        let mut stale_mask = 0u64;
        let mut hold_pos = [0.0f32; crate::rt_core::mailbox::MAX_DOF];
        for (i, target) in targets.iter().enumerate().take(dof) {
            let fresh = self.fresh_measured_position_for(i, crate::driver::STOPPED_DISABLE_FEEDBACK_MAX_AGE);
            if (hold_mask & (1u64 << i)) == 0 {
                report.joint_status[i] = if fresh.is_none() {
                    JointHoldStatus::UnheldNoFreshFeedback
                } else {
                    JointHoldStatus::SkippedOff
                };
                continue;
            }
            if target.position.is_finite() {
                hold_pos[i] = target.position;
            } else if let Some(fresh_p) = fresh {
                hold_pos[i] = fresh_p;
            } else {
                report.joint_status[i] = JointHoldStatus::UnheldNoFreshFeedback;
                continue;
            }
            if fresh.is_none() {
                stale_mask |= 1u64 << i;
                report.joint_status[i] = JointHoldStatus::HeldStaleTarget;
            } else {
                report.joint_status[i] = JointHoldStatus::Queued;
            }
            held_mask |= 1u64 << i;
        }
        if held_mask == 0 {
            return Ok(report);
        }
        let start_stamp = self.current_stop_stamp();
        self.estop_check()?;
        let off = self.zero_offsets.as_deref();
        self.cmd_hold_targets.clear();
        self.cmd_hold_targets.extend(targets.iter().zip(&self.modes).enumerate().map(|(i, (t, mode))| {
            if i >= dof || (held_mask & (1u64 << i)) == 0 {
                return None;
            }
            let q_joint = hold_pos[i];
            let q = off.map_or(q_joint, |o| q_joint + o[i]);
            Some(match mode {
                JointMode::Position => SimTarget::PosVel { q, vlim: POS_VEL_VLIM },
                JointMode::Mit => SimTarget::Mit {
                    q,
                    dq: 0.0,
                    kp: t.kp.unwrap_or(0.0),
                    kd: t.kd.unwrap_or(0.0),
                    tau: t.torque.filter(|v| v.is_finite()).unwrap_or(0.0),
                },
            })
        }));
        // Sparse `hold_targets` with `enable: None` leaves excluded actuators untouched and
        // guarantees zero enable or mode-switch side effect.
        self.send_ref(false, true, None, SendGate::Target, Some(start_stamp))?;
        for i in 0..dof {
            if (held_mask & (1u64 << i)) != 0 {
                report.joint_status[i] = if (stale_mask & (1u64 << i)) != 0 {
                    JointHoldStatus::HeldStaleTarget
                } else {
                    JointHoldStatus::Held
                };
            }
        }
        report.sent_any = true;
        Ok(report)
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if self.torque_hs.is_some() {
            return Err(ArmDriverError::Protocol("torque transition pending: no target before its confirmation".into()));
        }
        if targets.len() != self.dof {
            return Err(ArmDriverError::JointCountMismatch { expected: self.dof, actual: targets.len() });
        }
        let start_stamp = self.current_stop_stamp();
        self.estop_check()?;
        let off = self.zero_offsets.as_deref();
        self.cmd_targets.clear();
        self.cmd_targets.extend(targets.iter().zip(&self.modes).enumerate().map(|(i, (t, mode))| match mode {
            JointMode::Position => SimTarget::PosVel { q: off.map_or(t.position, |o| t.position + o[i]), vlim: POS_VEL_VLIM },
            JointMode::Mit => SimTarget::Mit {
                q: off.map_or(t.position, |o| t.position + o[i]),
                dq: t.velocity.unwrap_or(0.0),
                kp: t.kp.unwrap_or(0.0),
                kd: t.kd.unwrap_or(0.0),
                tau: t.torque.filter(|v| v.is_finite()).unwrap_or(0.0),
            },
        }));
        // Target packets omit `enable` so they preserve the peer's torque state; torque changes
        // go exclusively through explicit enable/disable commands (`begin_torque`).
        self.send_ref(true, false, None, SendGate::Target, Some(start_stamp))?;
        Ok(())
    }

    /// Blocking: [`ArmDriver::begin_torque`], then served every 1 ms until it finishes. An
    /// unacknowledged disable returns Ok (the caller checks the enable state).
    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        if enabled {
            let mut progress = self.begin_torque(true)?;
            loop {
                match progress {
                    TorqueProgress::Done(_) => return Ok(()),
                    TorqueProgress::Pending { .. } => {}
                }
                progress = self.poll_torque(Instant::now())?;
                if matches!(progress, TorqueProgress::Pending { .. }) { std::thread::sleep(Duration::from_millis(1)); }
            }
        } else {
            self.disable_torque(crate::driver::DisableKind::DisableRequested {
                started_at: Instant::now(),
                rollback: false,
            })
        }
    }

    fn disable_torque(&mut self, kind: crate::driver::DisableKind) -> Result<(), ArmDriverError> {
        let mut progress = self.begin_disable_torque(kind)?;
        loop {
            match progress {
                TorqueProgress::Done(_) => return Ok(()),
                TorqueProgress::Pending { .. } => {}
            }
            progress = self.poll_torque(Instant::now())?;
            if matches!(progress, TorqueProgress::Pending { .. }) {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    }

    /// The enable / disable command goes out at once; its acknowledgement (fresh state datagram
    /// with `ack >= seq` and the requested `enabled`) is awaited until the deadline. A disable
    /// replaces a pending enable.
    fn begin_torque(&mut self, enabled: bool) -> Result<TorqueProgress, ArmDriverError> {
        if enabled {
            if self.torque_hs.is_some() {
                return Err(ArmDriverError::Protocol("a torque transition is pending".into()));
            }
            let start_stamp = self.current_stop_stamp();
            self.enable_estop_inhibited()?;
            let seq = self.send_ref(false, false, Some(true), SendGate::Enable, Some(start_stamp))?;
            self.enable_requested = true;
            self.enable_seq = seq;
            let now = Instant::now();
            self.torque_hs = Some((true, Deadline::after(now, ENABLE_CONFIRM_TIMEOUT)));
            Ok(TorqueProgress::Pending { wake: now })
        } else {
            self.begin_disable_torque(crate::driver::DisableKind::DisableRequested {
                started_at: Instant::now(),
                rollback: false,
            })
        }
    }

    fn begin_disable_torque(&mut self, kind: crate::driver::DisableKind) -> Result<TorqueProgress, ArmDriverError> {
        let gate = match kind {
            crate::driver::DisableKind::DisableRequested { started_at, rollback } => {
                SendGate::DisableRequested { started_at, rollback }
            }
            crate::driver::DisableKind::DisableProtective { reason } => {
                SendGate::DisableProtective { reason }
            }
        };
        let seq = self.send_ref(false, false, Some(false), gate, None)?;
        self.enable_requested = false;
        self.enable_seq = seq;
        let now = Instant::now();
        self.torque_hs = Some((false, Deadline::after(now, DISABLE_CONFIRM_TIMEOUT)));
        Ok(TorqueProgress::Pending { wake: now })
    }

    fn begin_disable_torque_budgeted(
        &mut self,
        kind: crate::driver::DisableKind,
        _max_rx_frames: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.begin_disable_torque(kind)
    }

    /// No receive pass (the receiver thread fills the cache): `_deadline` bounds nothing.
    fn poll_torque(&mut self, _deadline: Instant) -> Result<TorqueProgress, ArmDriverError> {
        let Some((enabled, deadline)) = self.torque_hs else {
            return Err(ArmDriverError::Protocol("no torque transition pending".into()));
        };
        if enabled {
            self.enable_estop_inhibited()?;
        }
        self.maybe_send_owner_keepalive();
        self.refresh_from_rx();
        if enabled {
            self.enable_estop_inhibited()?;
        }
        // the acknowledgement is checked before the deadline (an ack seen at the deadline counts)
        let outcome = if self.acked(self.enable_seq, enabled) {
            TorqueOutcome::Confirmed
        } else if deadline.passed(Instant::now()) {
            TorqueOutcome::Unconfirmed
        } else {
            return Ok(TorqueProgress::Pending { wake: deadline.at });
        };
        self.torque_hs = None;
        if enabled && outcome == TorqueOutcome::Unconfirmed {
            return Err(ArmDriverError::HardwareFault(format!(
                "Simulator did not confirm enable within {} ms", ENABLE_CONFIRM_TIMEOUT.as_millis())));
        }
        if !enabled { self.modes.fill(JointMode::Position); }
        Ok(TorqueProgress::Done(outcome))
    }

    fn torque_transition_pending(&self) -> bool { self.torque_hs.is_some() }
    fn disable_frames_dispatched(&self) -> bool { matches!(self.torque_hs, Some((false, _))) }

    fn abort_enable(&mut self) {
        if matches!(self.torque_hs, Some((true, _))) {
            self.torque_hs = None;
            self.enable_requested = false;
        }
    }

    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }

    fn set_rt_mode(&mut self, rt: bool) {
        self.rt_mode.store(rt, Ordering::SeqCst);
        if rt {
            // Sleep on the condvar (never busy-yield, avoiding single-CPU SCHED_FIFO priority
            // inversion) until any in-flight `armd-sim-rx` keepalive finishes.
            let mut active = self.rx_keepalive_sync.0.lock().unwrap_or_else(|e| e.into_inner());
            while *active {
                active = self.rx_keepalive_sync.1.wait(active).unwrap_or_else(|e| e.into_inner());
            }
            drop(active);
            self.refresh_from_rx();
        }
    }

    fn is_feedback_valid(&self) -> bool {
        self.rx_metadata()
            .1
            .is_some_and(|rx| rx.elapsed() <= SIM_FEEDBACK_STALE)
    }

    fn is_enabled(&self) -> bool {
        self.enable_requested && self.enable_state() == EnableState::Enabled
    }

    fn motor_status(&self) -> Option<Vec<u8>> {
        match self.enable_state() {
            EnableState::Enabled => Some(vec![1; self.dof]),
            EnableState::Disabled => Some(vec![0; self.dof]),
            EnableState::Unknown => None,
        }
    }

    fn set_torque_flag(&mut self, enabled: bool) { self.enable_requested = enabled; }

    fn set_abort_flag(&mut self, flag: Arc<AtomicBool>) { self.abort_flag = Some(flag); }

    fn set_estop_state(&mut self, state: Arc<crate::rt_core::urgent::EstopState>) { self.estop_state = Some(state); }

    fn set_estop_gate(&mut self, gate: crate::rt_core::urgent::EstopGate) { self.estop_gate = Some(gate); }

    fn set_stopped_disable_gate(&mut self, gate: crate::driver::StoppedDisableGate) {
        self.stopped_disable_gate = Some(gate);
    }

    fn set_tx_release_deadline(&mut self, deadline: Option<Instant>) {
        self.tx_release_deadline = deadline;
    }

    fn supports_mit_mode_switch(&self) -> bool { true }

    /// The simulator switches a joint's control law with the next command, so the
    /// switch is confirmed locally; `hold` is sent at once in the new modes.
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        if modes.len() != self.dof || hold.len() != self.dof {
            return Err(ArmDriverError::JointCountMismatch { expected: self.dof, actual: modes.len().min(hold.len()) });
        }
        self.estop_check()?;
        let prev = self.modes.clone();
        self.modes.copy_from_slice(modes);
        if let Err(e) = self.write_joint_targets(hold) {
            self.modes = prev;
            return Err(e);
        }
        Ok(())
    }

    fn motor_zero_support(&self) -> MotorZeroSupport {
        MotorZeroSupport::set_current("simulator", true, vec!["the new zero is an offset in armd's simulated encoder; it is lost when armd restarts".into()])
    }

    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> {
        if joint >= self.dof { return Err(ArmDriverError::JointCountMismatch { expected: self.dof, actual: joint + 1 }); }
        if self.enable_requested || self.enable_state() != EnableState::Disabled {
            return Err(ArmDriverError::Protocol(format!("joint {}: set-zero needs torque off, confirmed by the simulator", joint + 1)));
        }
        let (Some(state), Some(age)) = self.snapshot() else {
            return Err(ArmDriverError::HardwareFault("no simulator state".into()));
        };
        if age > crate::driver::ZERO_FEEDBACK_MAX_AGE { return Err(ArmDriverError::HardwareFault(format!("simulator state is {age:?} old"))); }
        let off = self.zero_offsets.get_or_insert_with(|| vec![0.0; self.dof]);
        off[joint] = state.q[joint];
        self.cached[joint].position = 0.0;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fake simulator: a UDP socket the driver is connected to.
    struct Peer { socket: UdpSocket, armd: Option<SocketAddr> }

    impl Peer {
        fn new() -> Self {
            let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
            socket.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
            Self { socket, armd: None }
        }
        fn addr(&self) -> String { format!("udp://{}", self.socket.local_addr().unwrap()) }
        /// Next datagram from the driver (learns its address).
        fn recv(&mut self) -> SimCommand {
            let mut buf = [0u8; 4096];
            let (n, from) = self.socket.recv_from(&mut buf).unwrap();
            self.armd = Some(from);
            serde_json::from_slice(&buf[..n]).unwrap()
        }
        fn recv_kind(&mut self, kind: &str) -> SimCommand {
            loop { let c = self.recv(); if c.kind == kind { return c; } }
        }
        fn send_raw(&mut self, bytes: &[u8]) {
            if self.armd.is_none() { self.recv(); }
            self.socket.send_to(bytes, self.armd.unwrap()).unwrap();
        }
        fn send_state(&mut self, session: u64, seq: u64, ack: u64, enabled: bool, q0: f32) {
            let state = SimState { v: 1, kind: "state".into(), session, seq, ack, enabled,
                q: vec![q0, 0.1, 0.2], dq: vec![0.0; 3], tau: vec![0.5; 3], temp: vec![35.0; 3], loads: None, t: 0.0 };
            self.send_raw(&serde_json::to_vec(&state).unwrap());
        }
    }

    fn settle() { std::thread::sleep(Duration::from_millis(30)); }

    #[test]
    fn state_datagram_fills_joint_states_and_ages() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        assert!(!drv.is_feedback_valid());
        assert_eq!(drv.enable_state(), EnableState::Unknown);
        peer.send_state(7, 1, 0, false, 0.3);
        settle();
        let states = drv.read_joint_states().unwrap();
        assert_eq!(states[0].position, 0.3);
        assert_eq!(states[2].temperature, 35.0);
        assert!(drv.is_feedback_valid());
        assert_eq!(drv.enable_state(), EnableState::Disabled);
        assert!(drv.feedback_ages().iter().all(|a| a.is_some_and(|a| a < SIM_FEEDBACK_STALE)));
    }

    #[test]
    fn stale_state_reads_as_lost_feedback() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        peer.send_state(1, 1, 0, true, 0.0);
        settle();
        assert!(drv.is_feedback_valid());
        std::thread::sleep(SIM_FEEDBACK_STALE + Duration::from_millis(30));
        assert!(!drv.is_feedback_valid());
        assert_eq!(drv.enable_state(), EnableState::Unknown);
        assert!(drv.feedback_ages()[0].unwrap() > SIM_FEEDBACK_STALE);
        // the cached values stay readable, as on the Damiao driver
        assert_eq!(drv.read_joint_states().unwrap().len(), 3);
    }

    #[test]
    fn reordered_and_duplicate_datagrams_are_dropped_until_a_new_session() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        peer.send_state(1, 10, 0, false, 1.0);
        peer.send_state(1, 9, 0, false, 2.0);
        peer.send_state(1, 10, 0, false, 3.0);
        settle();
        assert_eq!(drv.read_joint_states().unwrap()[0].position, 1.0);
        assert_eq!(drv.rejected_datagrams(), 2);
        // a restarted simulator (new session) starts from seq 1 again
        peer.send_state(2, 1, 0, false, 4.0);
        settle();
        assert_eq!(drv.read_joint_states().unwrap()[0].position, 4.0);
    }

    #[test]
    fn malformed_datagrams_are_ignored() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        peer.send_state(1, 1, 0, false, 0.5);
        peer.send_raw(b"not json");
        peer.send_raw(br#"{"v":1,"type":"state","session":1,"seq":2,"ack":0,"enabled":false,"q":[9,9],"dq":[0,0],"tau":[0,0],"temp":[0,0]}"#);
        peer.send_raw(br#"{"v":2,"type":"state","session":1,"seq":3,"ack":0,"enabled":false,"q":[9,9,9],"dq":[0,0,0],"tau":[0,0,0],"temp":[0,0,0]}"#);
        settle();
        assert_eq!(drv.read_joint_states().unwrap()[0].position, 0.5);
        assert_eq!(drv.rejected_datagrams(), 3);
        assert!(parse_state(br#"{"v":1,"type":"state","session":1,"seq":1,"ack":0,"enabled":false,"q":[1,2,3],"dq":[0,0,0],"tau":[0,0,0],"temp":[0,0,0]}"#, 3).is_some());
    }

    #[test]
    fn enable_waits_for_acknowledged_state_and_fails_without_simulator() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        let handle = std::thread::spawn(move || {
            let cmd = peer.recv_kind("cmd");
            assert_eq!(cmd.enable, Some(true));
            assert!(cmd.targets.is_none());
            peer.send_state(1, 1, cmd.seq, true, 0.0);
            peer
        });
        drv.set_torque(true).unwrap();
        assert_eq!(drv.enable_state(), EnableState::Enabled);
        assert!(drv.is_enabled());
        let _peer = handle.join().unwrap();

        let silent = Peer::new();
        let mut drv = SimArmDriver::connect(&silent.addr(), 3).unwrap();
        assert!(drv.set_torque(true).is_err());
    }

    #[test]
    fn targets_carry_mode_per_joint() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        drv.write_joint_targets(&[JointTarget::position(0.1), JointTarget::position(0.2), JointTarget::position(0.3)]).unwrap();
        let cmd = peer.recv_kind("cmd");
        assert_eq!(cmd.targets.unwrap()[1], SimTarget::PosVel { q: 0.2, vlim: POS_VEL_VLIM });
        let hold = [JointTarget::position(0.0), JointTarget::mit(0.5, 0.0, 10.0, 1.0, 2.5), JointTarget::position(0.0)];
        drv.set_joint_modes(&[JointMode::Position, JointMode::Mit, JointMode::Position], &hold).unwrap();
        let cmd = peer.recv_kind("cmd");
        assert_eq!(cmd.targets.unwrap()[1], SimTarget::Mit { q: 0.5, dq: 0.0, kp: 10.0, kd: 1.0, tau: 2.5 });
        assert_eq!(drv.max_command_velocity(1), None);
        assert_eq!(drv.max_command_velocity(0), Some(POS_VEL_VLIM));
        assert!(drv.write_joint_targets(&[JointTarget::position(0.0)]).is_err());
    }

    #[test]
    fn set_zero_offsets_the_simulated_encoder() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        peer.send_state(1, 1, 0, false, 0.3);
        settle();
        assert_eq!(drv.read_joint_states().unwrap()[0].position, 0.3);
        assert!(drv.motor_zero_support().supported);
        drv.set_zero_position(0).unwrap();
        // the simulator still reports 0.3 (the arm did not move); armd reads 0
        peer.send_state(1, 2, 0, false, 0.3);
        settle();
        let s = drv.read_joint_states().unwrap();
        assert!(s[0].position.abs() < 1e-6 && s[1].position == 0.1, "{s:?}");
        // targets go out in simulator angles
        drv.write_joint_targets(&[JointTarget::position(0.1), JointTarget::position(0.2), JointTarget::position(0.3)]).unwrap();
        let cmd = peer.recv_kind("cmd");
        assert_eq!(cmd.targets.unwrap()[0], SimTarget::PosVel { q: 0.1 + 0.3, vlim: POS_VEL_VLIM });
        assert!(drv.set_zero_position(3).is_err());
        // refused while torque is requested on
        drv.set_torque_flag(true);
        assert!(drv.set_zero_position(1).is_err());
    }

    #[test]
    fn keepalive_reaches_the_simulator_without_commands() {
        let mut peer = Peer::new();
        let _drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        assert_eq!(peer.recv().kind, "poll");
    }

    #[test]
    fn fresh_joint_positions_reads_timestamp_and_positions_from_same_packet() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        // Packet 1: rest pose (q0 = 0.0), synced into owner-local `self.cached`.
        peer.send_state(1, 1, 0, true, 0.0);
        settle();
        assert_eq!(drv.read_joint_states().unwrap()[0].position, 0.0);

        // Packet 2: raised pose (q0 = 0.45), received on the background thread WITHOUT calling
        // `read_joint_states()` again so `self.cached` still holds 0.0.
        peer.send_state(1, 2, 0, true, 0.45);
        settle();
        assert_eq!(drv.cached[0].position, 0.0, "owner cache must still hold packet 1");

        // Legacy mode MUST read BOTH timestamp and position from packet 2 (0.45), never 0.0.
        let mut out = [0.0f32; 2];
        assert!(drv.fresh_joint_positions_into(&[0, 1], Duration::from_millis(200), &mut out));
        assert!((out[0] - 0.45).abs() < 1e-5, "expected packet 2 position 0.45, got {}", out[0]);

        // RT mode MUST also read BOTH timestamp and position from the receiver's latest packet
        // (packet 2, 0.45), never the superseded owner cache (packet 1, 0.0).
        drv.set_rt_mode(true);
        let mut out_rt = [0.0f32; 2];
        assert!(drv.fresh_joint_positions_into(&[0, 1], Duration::from_millis(200), &mut out_rt));
        assert!(
            (out_rt[0] - 0.45).abs() < 1e-5,
            "RT mode must read receiver's latest packet 2 (0.45), not superseded cache (0.0); got {}",
            out_rt[0]
        );

        // Stale receiver timestamp in RT mode is refused even if owner cache looks fresh.
        drv.cached_received_at = Some(Instant::now());
        drv.shared.lock().unwrap().received_at = Some(Instant::now() - Duration::from_millis(500));
        assert!(
            !drv.fresh_joint_positions_into(&[0, 1], Duration::from_millis(100), &mut out_rt),
            "stale receiver packet timestamp in RT mode must be refused"
        );
    }

    #[test]
    fn rt_sim_send_gate_reads_latest_receiver_packet_and_fails_closed_when_contended() {
        let mut peer = Peer::new();
        let mut drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        let mut arm_joints = [0usize; crate::rt_core::mailbox::MAX_DOF];
        arm_joints[0] = 0;
        arm_joints[1] = 1;
        arm_joints[2] = 2;
        drv.set_stopped_disable_gate(crate::driver::StoppedDisableGate {
            safety_stopped: Arc::new(AtomicBool::new(true)),
            has_resting_pose: true,
            arm_joints,
            arm_joints_len: 3,
            rest_positions: [0.0, 0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            rest_pose_tolerance_rad: 0.05,
        });
        // 1. Packet 1: at rest (`q = [0.0, 0.1, 0.2]`), let peer discover armd's UDP port via
        //    initial poll, then switch into RT mode and sync into owner-local cache.
        peer.send_state(1, 1, 0, true, 0.0);
        settle();
        drv.set_rt_mode(true);
        let mut states = Vec::with_capacity(3);
        drv.read_joint_states_budgeted(&mut states, Instant::now() + Duration::from_millis(5), 64)
            .unwrap();
        assert!((drv.cached[0].position - 0.0).abs() < 1e-5);

        // Drain any initial keepalive polls from peer.
        peer.socket.set_read_timeout(Some(Duration::from_millis(25))).unwrap();
        let mut drain_buf = [0u8; 4096];
        while peer.socket.recv(&mut drain_buf).is_ok() {}

        // 2. Packet 2: newer RAISED pose (`q0 = 0.50`) arrives at receiver WITHOUT calling
        //    `read_joint_states` so `self.cached` still holds the superseded rest pose (`0.0`)!
        peer.send_state(1, 2, 0, true, 0.50);
        settle();
        assert!((drv.cached[0].position - 0.0).abs() < 1e-5, "owner cache still holds superseded rest pose");

        let res_raised = drv.begin_disable_torque(crate::driver::DisableKind::DisableRequested {
            started_at: Instant::now(),
            rollback: false,
        });
        assert!(
            matches!(res_raised, Err(ArmDriverError::TorqueOffWouldDrop)),
            "RT sim send gate must refuse using receiver's newer raised pose, got {res_raised:?}"
        );
        // Verify zero UDP `cmd` datagrams reached the simulator peer.
        while let Ok(n) = peer.socket.recv(&mut drain_buf) {
            if let Ok(cmd) = serde_json::from_slice::<SimCommand>(&drain_buf[..n]) {
                assert_ne!(cmd.kind, "cmd", "unexpected cmd datagram transmitted on raised pose: {cmd:?}");
            }
        }

        // 3. Packet 3 returns to rest (`q0 = 0.0`), but receiver mutex is contended -> RT send
        //    gate must NOT block and must fail closed (`TorqueOffWouldDrop`, 0 cmd datagrams).
        peer.send_state(1, 3, 0, true, 0.0);
        settle();
        let lock = drv.test_rx_lock_handle();
        let _held = lock.lock();
        let t0 = Instant::now();
        let res_contended = drv.begin_disable_torque(crate::driver::DisableKind::DisableRequested {
            started_at: Instant::now(),
            rollback: false,
        });
        let elapsed = t0.elapsed();
        drop(_held);
        assert!(
            matches!(res_contended, Err(ArmDriverError::TorqueOffWouldDrop)),
            "contended RT receiver lock must fail closed, got {res_contended:?}"
        );
        assert!(elapsed < Duration::from_millis(10), "contended RT send gate blocked: {elapsed:?}");

        // 4. Uncontended at rest (packet 3) -> succeeds and transmits enable=false!
        drv.begin_disable_torque(crate::driver::DisableKind::DisableRequested {
            started_at: Instant::now(),
            rollback: false,
        })
        .expect("uncontended fresh rest pose in RT mode must transmit disable");
        peer.socket.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
        let cmd = peer.recv_kind("cmd");
        assert_eq!(cmd.enable, Some(false));
    }
}
