//! Damiao CAN bus fake for the MIT tracking tests: seven motors with registers (identity,
//! firmware, mode, ranges, TIMEOUT), feedback replies, an optional deterministic clock and
//! destructive failure injection (TX loss / errors / stalls, RX loss, lost hold, failed flush).
//! Register replies arrive on each motor's MST_ID like the motors' feedback.
//!
//! Motor command watchdog model (an assumption for the tests, not a hardware verification):
//! with TIMEOUT > 0 and the motor enabled, the motor keeps its last complete command until
//! `TIMEOUT * 50 µs` after the last command frame it received (MIT, POS_VEL, enable; with
//! `poll_feeds_watchdog` also the 0xCC status poll), then disables itself and reports state
//! 0xD (CAN communication loss).
#![allow(dead_code)]

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, CanTransport, DriverClock, MockClock, TransportError};

/// B601-DM firmware version read on 2026-11-22.
pub const SW_VER: u32 = 942_747_703;

/// Last command a fake motor executes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorCmd {
    PosVel { p: f32, v: f32 },
    Mit { p: f32, v: f32, kp: f32, kd: f32, t: f32 },
}

pub struct FakeMotor {
    pub id: u16,
    pub q: f32,
    pub v: f32,
    pub tau: f32,
    pub enabled: bool,
    /// Feedback state nibble override (fault codes); None = enabled ? 1 : 0.
    pub state_override: Option<u8>,
    pub limits: DamiaoLimits,
    pub regs: HashMap<u8, [u8; 4]>,
    /// CAN ID of the replies (feedback and register); default 0x10 + id (= MST_ID).
    pub reply_id: u16,
    /// No reply of any kind (RX loss for this motor).
    pub silent: bool,
    /// Register writes are acknowledged but not stored.
    pub ignore_reg_writes: bool,
    /// Value the write acknowledgement carries instead of the written one.
    pub ack_value_override: Option<u32>,
    /// Register writes get no acknowledgement.
    pub no_write_ack: bool,
    /// After the next CTRL_MODE acknowledgement the motor goes silent (mode accepted, hold,
    /// post-switch feedback and every later reply lost).
    pub silent_after_mode_ack: bool,
    /// After the next CTRL_MODE acknowledgement the motor sends no feedback until the following
    /// CTRL_MODE write (mode accepted, hold and post-switch feedback lost; a rollback works).
    pub mute_feedback_after_mode_ack: bool,
    pub mute_feedback: bool,
    /// Never any feedback frame (register replies still come).
    pub no_feedback: bool,
    /// Last executed command and when it arrived (fake clock).
    pub last_cmd: Option<MotorCmd>,
    pub last_cmd_at: Option<Instant>,
    /// The motor's own watchdog fired.
    pub watchdog_expired_at: Option<Instant>,
    /// Number of command frames received.
    pub commands: usize,
}

impl FakeMotor {
    pub fn new(id: u16, limits: DamiaoLimits) -> Self {
        let mut regs = HashMap::new();
        regs.insert(damiao::RID_ESC_ID, u32::from(id).to_le_bytes());
        regs.insert(damiao::RID_MST_ID, (0x10 + u32::from(id)).to_le_bytes());
        regs.insert(damiao::RID_SW_VER, SW_VER.to_le_bytes());
        regs.insert(damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL.to_le_bytes());
        regs.insert(damiao::RID_PMAX, limits.p_max.to_le_bytes());
        regs.insert(damiao::RID_VMAX, limits.v_max.to_le_bytes());
        regs.insert(damiao::RID_TMAX, limits.t_max.to_le_bytes());
        regs.insert(damiao::RID_TIMEOUT, 0u32.to_le_bytes());
        Self { id, q: 0.0, v: 0.0, tau: 0.0, enabled: false, state_override: None, limits, regs, reply_id: 0x10 + id,
            silent: false, ignore_reg_writes: false, ack_value_override: None, no_write_ack: false, silent_after_mode_ack: false,
            mute_feedback_after_mode_ack: false, mute_feedback: false, no_feedback: false, last_cmd: None, last_cmd_at: None, watchdog_expired_at: None, commands: 0 }
    }

    pub fn reg_u32(&self, rid: u8) -> u32 { u32::from_le_bytes(self.regs[&rid]) }
    pub fn set_reg_u32(&mut self, rid: u8, v: u32) { self.regs.insert(rid, v.to_le_bytes()); }
    pub fn ctrl_mode(&self) -> u32 { self.reg_u32(damiao::RID_CTRL_MODE) }

    pub fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(self.tau, l.t_min, l.t_max, 12);
        let state = self.state_override.unwrap_or(u8::from(self.enabled));
        let data = [(state << 4) | (self.id as u8 & 0x0f), (p >> 8) as u8, p as u8, (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8, t as u8, 30, 30];
        CanFrame::new_standard(self.reply_id, &data).unwrap()
    }

    fn reg_reply(&self, op: u8, rid: u8, value: [u8; 4]) -> CanFrame {
        let [lo, hi] = self.id.to_le_bytes();
        CanFrame::new_standard(self.reply_id, &[lo, hi, op, rid, value[0], value[1], value[2], value[3]]).unwrap()
    }

    /// Watchdog deadline of the last command, when TIMEOUT is set (documented 50 µs / count).
    pub fn watchdog_deadline(&self) -> Option<Instant> {
        let counts = self.reg_u32(damiao::RID_TIMEOUT);
        (counts > 0 && self.enabled).then_some(())?;
        self.last_cmd_at.map(|t| t + Duration::from_micros(u64::from(counts) * 50))
    }
}

/// Decode an MIT frame with a motor's ranges.
pub fn decode_mit(data: &[u8; 8], l: &DamiaoLimits) -> MotorCmd {
    let p = (u16::from(data[0]) << 8) | u16::from(data[1]);
    let v = (u16::from(data[2]) << 4) | (u16::from(data[3]) >> 4);
    let kp = ((u16::from(data[3]) & 0x0f) << 8) | u16::from(data[4]);
    let kd = (u16::from(data[5]) << 4) | (u16::from(data[6]) >> 4);
    let t = ((u16::from(data[6]) & 0x0f) << 8) | u16::from(data[7]);
    MotorCmd::Mit {
        p: damiao::uint_to_float(p, l.p_min, l.p_max, 16),
        v: damiao::uint_to_float(v, l.v_min, l.v_max, 12),
        kp: damiao::uint_to_float(kp, l.kp_min, l.kp_max, 12),
        kd: damiao::uint_to_float(kd, l.kd_min, l.kd_max, 12),
        t: damiao::uint_to_float(t, l.t_min, l.t_max, 12),
    }
}

pub struct FakeBusState {
    pub motors: Vec<FakeMotor>,
    pub rx: VecDeque<CanFrame>,
    /// Replies to transmitted frames: they reach `rx` only after the receive queue was observed
    /// empty once after the transmit (a reply never precedes its request on the wire).
    pub pending: VecDeque<CanFrame>,
    /// Old frames delivered at once on the next transmit, ahead of any real reply (stale
    /// replies still queued in the transport).
    pub inject_on_next_tx: Vec<CanFrame>,
    /// The same, on the next register write (0x7FF, opcode 0x55).
    pub inject_on_reg_write: Vec<CanFrame>,
    /// Replies enter the receive queue at once (no observed-empty queue between request and reply).
    pub immediate_replies: bool,
    /// Every frame the driver handed to the transport (also the lost and refused ones), with the
    /// fake time.
    pub tx_log: Vec<CanFrame>,
    pub tx_times: Vec<Option<Instant>>,
    pub clock: Option<MockClock>,
    /// TX-only loss: frames are accepted by the transport but never reach a motor.
    pub tx_lost: bool,
    /// Every transmit fails with an error.
    pub tx_error: bool,
    /// Every transmit reports WouldBlock (stalled controller); each attempt advances the clock
    /// by `stall_step`.
    pub tx_stall: bool,
    pub stall_step: Duration,
    /// The next `n` transmits succeed, then every transmit fails (partial batch).
    pub fail_tx_after: Option<usize>,
    /// Transmits to these command CAN IDs fail.
    pub fail_tx_ids: Vec<u32>,
    /// Fake time each dequeued frame / each transmit takes (slow host, queued controller).
    pub rx_step: Duration,
    pub tx_step: Duration,
    /// The next `n` flushes fail.
    pub fail_flushes: usize,
    /// The 0xCC status poll refreshes the motor watchdog (unknown on the device).
    pub poll_feeds_watchdog: bool,
    /// The motor plant follows the commanded position at once.
    pub follow_commands: bool,
}

impl FakeBusState {
    pub fn now(&self) -> Option<Instant> { self.clock.as_ref().map(MockClock::now) }

    pub fn motor_mut(&mut self, id: u16) -> Option<&mut FakeMotor> { self.motors.iter_mut().find(|m| m.id == id) }

    pub fn push_feedback(&mut self, i: usize) {
        if !self.motors[i].silent && !self.motors[i].mute_feedback && !self.motors[i].no_feedback {
            let f = self.motors[i].feedback_frame();
            self.rx.push_back(f);
        }
    }

    pub fn push_all_feedback(&mut self) { for i in 0..self.motors.len() { self.push_feedback(i); } }

    /// Run the motors' own watchdogs up to the current fake time.
    pub fn run_watchdogs(&mut self) {
        let Some(now) = self.now() else { return };
        for m in &mut self.motors {
            if let Some(deadline) = m.watchdog_deadline().filter(|d| now >= *d) {
                m.enabled = false;
                m.state_override = Some(0xD);
                m.watchdog_expired_at = Some(deadline);
            }
        }
    }

    fn command(&mut self, k: usize, cmd: Option<MotorCmd>) {
        let now = self.now();
        let follow = self.follow_commands;
        let m = &mut self.motors[k];
        m.commands += 1;
        if m.enabled {
            if let Some(c) = cmd {
                m.last_cmd = Some(c);
                if follow {
                    m.q = match c { MotorCmd::PosVel { p, .. } | MotorCmd::Mit { p, .. } => p };
                    m.v = 0.0;
                }
            }
            m.last_cmd_at = now;
        }
    }

    fn handle(&mut self, frame: &CanFrame) {
        self.run_watchdogs();
        if frame.id == 0x7FF {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let (op, rid) = (frame.data[2], frame.data[3]);
            let Some(k) = self.motors.iter().position(|m| m.id == id) else { return };
            let now = self.now();
            let poll_feeds = self.poll_feeds_watchdog;
            let m = &mut self.motors[k];
            if m.silent { return; }
            let reply = match op {
                0xCC => {
                    if poll_feeds && m.enabled { m.last_cmd_at = now; }
                    (!m.mute_feedback && !m.no_feedback).then(|| m.feedback_frame())
                }
                damiao::REG_READ => m.regs.get(&rid).map(|v| m.reg_reply(damiao::REG_READ, rid, *v)),
                damiao::REG_WRITE => {
                    let value = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
                    if !m.ignore_reg_writes { m.regs.insert(rid, value); }
                    let ack = m.ack_value_override.map_or(value, u32::to_le_bytes);
                    let r = (!m.no_write_ack).then(|| m.reg_reply(damiao::REG_WRITE, rid, ack));
                    if rid == damiao::RID_CTRL_MODE {
                        m.mute_feedback = std::mem::take(&mut m.mute_feedback_after_mode_ack);
                    }
                    if rid == damiao::RID_CTRL_MODE && m.silent_after_mode_ack {
                        m.silent_after_mode_ack = false;
                        if let Some(r) = r { self.rx.push_back(r); }
                        self.motors[k].silent = true;
                        return;
                    }
                    r
                }
                _ => None,
            };
            if let Some(r) = reply { self.rx.push_back(r); }
            return;
        }
        let cmd = frame.data;
        if let Some(k) = self.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if cmd == damiao::CMD_ENABLE {
                let m = &mut self.motors[k];
                m.enabled = true;
                m.state_override = None;
                self.command(k, None);
            } else if cmd == damiao::CMD_DISABLE {
                self.motors[k].enabled = false;
            } else if cmd == damiao::CMD_CLEAR_FAULTS {
                self.motors[k].state_override = None;
            } else if self.motors[k].ctrl_mode() == damiao::CTRL_MODE_MIT {
                let c = decode_mit(&cmd, &self.motors[k].limits);
                self.command(k, Some(c));
            }
            self.push_feedback(k);
            return;
        }
        if let Some(k) = self.motors.iter().position(|m| u32::from(m.id) + 0x100 == frame.id) {
            if self.motors[k].ctrl_mode() == damiao::CTRL_MODE_POS_VEL {
                let p = f32::from_le_bytes([cmd[0], cmd[1], cmd[2], cmd[3]]);
                let v = f32::from_le_bytes([cmd[4], cmd[5], cmd[6], cmd[7]]);
                self.command(k, Some(MotorCmd::PosVel { p, v }));
            }
            self.push_feedback(k);
        }
    }
}

#[derive(Clone)]
pub struct FakeBus(pub Arc<Mutex<FakeBusState>>);

impl FakeBus {
    pub fn new(limits: &[DamiaoLimits]) -> Self { Self::build(limits, None) }

    /// With a deterministic clock shared with the driver (`CanTransport::clock`).
    pub fn with_clock(limits: &[DamiaoLimits], clock: MockClock) -> Self { Self::build(limits, Some(clock)) }

    fn build(limits: &[DamiaoLimits], clock: Option<MockClock>) -> Self {
        let motors = limits.iter().enumerate().map(|(i, l)| FakeMotor::new(i as u16 + 1, *l)).collect();
        Self(Arc::new(Mutex::new(FakeBusState { motors, rx: VecDeque::new(), pending: VecDeque::new(), inject_on_next_tx: Vec::new(), inject_on_reg_write: Vec::new(), immediate_replies: false, tx_log: Vec::new(), tx_times: Vec::new(), clock,
            tx_lost: false, tx_error: false, tx_stall: false, stall_step: Duration::from_millis(1), fail_tx_after: None,
            fail_tx_ids: Vec::new(), rx_step: Duration::ZERO, tx_step: Duration::ZERO, fail_flushes: 0, poll_feeds_watchdog: false, follow_commands: true })))
    }
    pub fn with<R>(&self, f: impl FnOnce(&mut FakeBusState) -> R) -> R { f(&mut self.0.lock().unwrap()) }
    pub fn tx_log(&self) -> Vec<CanFrame> { self.with(|s| s.tx_log.clone()) }
    pub fn clear_log(&self) { self.with(|s| { s.tx_log.clear(); s.tx_times.clear(); }) }
}

impl CanTransport for FakeBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        self.0.lock().unwrap().clock.clone().map(|c| Arc::new(c) as Arc<dyn DriverClock>)
    }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        if let Some(c) = &s.clock { c.advance(s.tx_step); }
        let t = s.now();
        s.tx_log.push(*frame);
        s.tx_times.push(t);
        if s.tx_stall {
            if let Some(c) = &s.clock { c.advance(s.stall_step); }
            return Err(TransportError::WouldBlock);
        }
        if s.tx_error || s.fail_tx_ids.contains(&frame.id) {
            return Err(TransportError::Can("injected transmit error".into()));
        }
        if let Some(n) = s.fail_tx_after.as_mut() {
            if *n == 0 { return Err(TransportError::Can("injected transmit error (partial batch)".into())); }
            *n -= 1;
        }
        let stale = std::mem::take(&mut s.inject_on_next_tx);
        s.rx.extend(stale);
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let stale = std::mem::take(&mut s.inject_on_reg_write);
            s.rx.extend(stale);
        }
        if !s.tx_lost {
            let before = s.rx.len();
            s.handle(frame);
            if !s.immediate_replies {
                let replies: Vec<CanFrame> = s.rx.drain(before..).collect();
                s.pending.extend(replies);
            }
        }
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        let mut s = self.0.lock().unwrap();
        s.run_watchdogs();
        match s.rx.pop_front() {
            Some(f) => {
                if let Some(c) = &s.clock { c.advance(s.rx_step); }
                Ok(f)
            }
            None => {
                let p = std::mem::take(&mut s.pending);
                s.rx.extend(p);
                Err(TransportError::WouldBlock)
            }
        }
    }
    fn flush(&mut self) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        if s.fail_flushes > 0 { s.fail_flushes -= 1; return Err(TransportError::Can("injected flush error".into())); }
        Ok(())
    }
}

/// No frame on the bus ever used the flash store opcode 0xAA (register access on 0x7FF).
pub fn assert_no_flash_write(log: &[CanFrame]) {
    for f in log {
        assert!(!(f.id == 0x7FF && f.data[2] == 0xAA), "flash store frame on the bus: {f:?}");
    }
}

/// Frames addressed to motor `id` that command it (MIT on `id`, POS_VEL on 0x100 + id, enable).
pub fn command_frames_to(log: &[CanFrame], id: u16) -> Vec<CanFrame> {
    log.iter().filter(|f| f.id == u32::from(id) || f.id == 0x100 + u32::from(id)).copied().collect()
}
