//! `--follower sim`: an [`ArmDriver`] backed by a physics simulator over UDP
//! (`sim/mujoco`, `rebot-sim`). See docs/simulation.md for the wire protocol.
//!
//! The driver is called inside the control tick under the supervisor write lock, so no
//! call blocks on the network: a background thread receives state datagrams into a cache
//! that `read_joint_states` copies. Only `set_torque` waits, bounded like the Damiao
//! enable handshake (80 ms enable, 25 ms disable), for the simulator to confirm.
//!
//! Wire protocol (version 1), one JSON object per UDP datagram:
//! - armd -> sim `{"v":1,"type":"cmd","seq":N,"enable":bool,"targets":[...]|null}`;
//!   a target is `{"mode":"pos_vel","q":rad,"vlim":rad/s}` or
//!   `{"mode":"mit","q":rad,"dq":rad/s,"kp":Nm/rad,"kd":Nm·s/rad,"tau":Nm}`.
//!   `targets: null` (enable / disable / keep-alive) leaves the motor targets unchanged.
//! - sim -> armd `{"v":1,"type":"state","session":u64,"seq":N,"ack":N,"enabled":bool,
//!   "q":[..],"dq":[..],"tau":[..],"temp":[..],"t":s}` with one entry per joint.
//!   `ack` is the highest command `seq` applied; `session` changes when the simulator restarts.
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use crate::driver::{ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointMode, JointTarget};
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
    pub enable: bool,
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
    pub t: f64,
}

/// Parse and validate one state datagram for `dof` joints.
pub fn parse_state(bytes: &[u8], dof: usize) -> Option<SimState> {
    let state: SimState = serde_json::from_slice(bytes).ok()?;
    let arrays = [&state.q, &state.dq, &state.tau, &state.temp];
    (state.v == PROTOCOL_VERSION && state.kind == "state"
        && arrays.iter().all(|a| a.len() == dof && a.iter().all(|v| v.is_finite())))
        .then_some(state)
}

#[derive(Debug, Default)]
struct Shared {
    latest: Option<SimState>,
    received_at: Option<Instant>,
    rejected: u64,
}

pub struct SimArmDriver {
    socket: UdpSocket,
    dof: usize,
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    last_send_ms: Arc<AtomicU64>,
    epoch: Instant,
    thread: Option<std::thread::JoinHandle<()>>,
    seq: u64,
    enable_requested: bool,
    /// Seq of the last enable / disable command; enable state is Unknown until acked.
    enable_seq: u64,
    modes: Vec<JointMode>,
    cached: Vec<JointState>,
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
        let stop = Arc::new(AtomicBool::new(false));
        let last_send_ms = Arc::new(AtomicU64::new(0));
        let epoch = Instant::now();
        let thread = {
            let (socket, shared, stop, last_send_ms) = (socket.try_clone()?, shared.clone(), stop.clone(), last_send_ms.clone());
            std::thread::Builder::new().name("armd-sim-rx".into()).spawn(move || {
                receiver_loop(socket, dof, shared, stop, last_send_ms, epoch)
            })?
        };
        Ok(Self {
            socket, dof, shared, stop, last_send_ms, epoch, thread: Some(thread),
            seq: 0, enable_requested: false, enable_seq: 0,
            modes: vec![JointMode::Position; dof],
            cached: vec![JointState::default(); dof],
        })
    }

    /// Datagrams dropped as malformed, out of order or for the wrong joint count.
    pub fn rejected_datagrams(&self) -> u64 { self.shared.lock().map_or(0, |s| s.rejected) }

    fn send(&mut self, targets: Option<Vec<SimTarget>>) -> u64 {
        self.seq += 1;
        let cmd = SimCommand { v: PROTOCOL_VERSION, kind: "cmd".into(), seq: self.seq, enable: self.enable_requested, targets };
        if let Ok(bytes) = serde_json::to_vec(&cmd) {
            // A lost datagram is equivalent to a lost CAN frame: the missing feedback,
            // not the write error, is what the supervisor acts on (BUS_LOSS).
            let _ = self.socket.send(&bytes);
            self.last_send_ms.store(self.epoch.elapsed().as_millis() as u64, Ordering::Relaxed);
        }
        self.seq
    }

    fn snapshot(&self) -> (Option<SimState>, Option<Duration>) {
        self.shared.lock().map_or((None, None), |s| (s.latest.clone(), s.received_at.map(|t| t.elapsed())))
    }

    fn wait_for_ack(&self, seq: u64, enabled: bool, timeout: Duration) -> bool {
        let start = Instant::now();
        loop {
            if let (Some(state), Some(age)) = self.snapshot() {
                if state.ack >= seq && state.enabled == enabled && age <= SIM_FEEDBACK_STALE { return true; }
            }
            if start.elapsed() >= timeout { return false; }
            std::thread::sleep(Duration::from_millis(1));
        }
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

fn receiver_loop(socket: UdpSocket, dof: usize, shared: Arc<Mutex<Shared>>, stop: Arc<AtomicBool>, last_send_ms: Arc<AtomicU64>, epoch: Instant) {
    let mut buf = [0u8; 4096];
    let keepalive = serde_json::to_vec(&SimCommand { v: PROTOCOL_VERSION, kind: "poll".into(), seq: 0, enable: false, targets: None })
        .expect("serialisable");
    while !stop.load(Ordering::SeqCst) {
        let now_ms = epoch.elapsed().as_millis() as u64;
        if now_ms.saturating_sub(last_send_ms.load(Ordering::Relaxed)) >= KEEPALIVE.as_millis() as u64 {
            let _ = socket.send(&keepalive);
            last_send_ms.store(now_ms, Ordering::Relaxed);
        }
        let n = match socket.recv(&mut buf) {
            Ok(n) => n,
            // Timeout, or ECONNREFUSED while no simulator listens: keep polling.
            Err(_) => continue,
        };
        let Ok(mut s) = shared.lock() else { return };
        match parse_state(&buf[..n], dof) {
            // Same session: drop duplicates and reordered datagrams. A new session
            // (simulator restart) starts a new sequence.
            Some(state) if s.latest.as_ref().is_none_or(|l| l.session != state.session || state.seq > l.seq) => {
                s.latest = Some(state);
                s.received_at = Some(Instant::now());
            }
            _ => s.rejected += 1,
        }
    }
}

impl ArmDriver for SimArmDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            velocity_feedback: true,
            torque_feedback: true,
            temperature_feedback: true,
            // Same as the Damiao driver in its POS_VEL base mode: feed-forward only
            // through the per-joint MIT switch used by gravity compensation.
            torque_feedforward: false,
            enable_feedback: true,
        }
    }

    fn enable_state(&self) -> EnableState {
        match self.snapshot() {
            (Some(state), Some(age)) if age <= SIM_FEEDBACK_STALE && state.ack >= self.enable_seq => {
                if state.enabled { EnableState::Enabled } else { EnableState::Disabled }
            }
            _ => EnableState::Unknown,
        }
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        vec![self.snapshot().1; self.dof]
    }

    fn max_command_velocity(&self, joint: usize) -> Option<f32> {
        (joint < self.dof && self.modes[joint] == JointMode::Position).then_some(POS_VEL_VLIM)
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        if let (Some(state), _) = self.snapshot() {
            for (i, js) in self.cached.iter_mut().enumerate() {
                *js = JointState { position: state.q[i], velocity: state.dq[i], torque: state.tau[i], temperature: state.temp[i] };
            }
        }
        Ok(self.cached.clone())
    }

    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        if targets.len() != self.dof {
            return Err(ArmDriverError::JointCountMismatch { expected: self.dof, actual: targets.len() });
        }
        let wire = targets.iter().zip(&self.modes).map(|(t, mode)| match mode {
            JointMode::Position => SimTarget::PosVel { q: t.position, vlim: POS_VEL_VLIM },
            JointMode::Mit => SimTarget::Mit {
                q: t.position,
                dq: t.velocity.unwrap_or(0.0),
                kp: t.kp.unwrap_or(0.0),
                kd: t.kd.unwrap_or(0.0),
                tau: t.torque.filter(|v| v.is_finite()).unwrap_or(0.0),
            },
        }).collect();
        self.send(Some(wire));
        Ok(())
    }

    fn set_torque(&mut self, enabled: bool) -> Result<(), ArmDriverError> {
        self.enable_requested = enabled;
        self.enable_seq = self.send(None);
        let timeout = if enabled { ENABLE_CONFIRM_TIMEOUT } else { DISABLE_CONFIRM_TIMEOUT };
        let confirmed = self.wait_for_ack(self.enable_seq, enabled, timeout);
        if enabled && !confirmed {
            return Err(ArmDriverError::HardwareFault(format!(
                "Simulator did not confirm enable within {} ms", ENABLE_CONFIRM_TIMEOUT.as_millis())));
        }
        if !enabled { self.modes.fill(JointMode::Position); }
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        self.snapshot().1.is_some_and(|age| age <= SIM_FEEDBACK_STALE)
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

    fn supports_mit_mode_switch(&self) -> bool { true }

    /// The simulator switches a joint's control law with the next command, so the
    /// switch is confirmed locally; `hold` is sent at once in the new modes.
    fn set_joint_modes(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        if modes.len() != self.dof || hold.len() != self.dof {
            return Err(ArmDriverError::JointCountMismatch { expected: self.dof, actual: modes.len().min(hold.len()) });
        }
        self.modes.copy_from_slice(modes);
        self.write_joint_targets(hold)
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
                q: vec![q0, 0.1, 0.2], dq: vec![0.0; 3], tau: vec![0.5; 3], temp: vec![35.0; 3], t: 0.0 };
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
            assert!(cmd.enable && cmd.targets.is_none());
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
    fn keepalive_reaches_the_simulator_without_commands() {
        let mut peer = Peer::new();
        let _drv = SimArmDriver::connect(&peer.addr(), 3).unwrap();
        assert_eq!(peer.recv().kind, "poll");
    }
}
