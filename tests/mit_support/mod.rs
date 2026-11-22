//! Damiao CAN bus fake for the MIT tracking tests: seven motors with registers (identity,
//! firmware, mode, ranges, TIMEOUT), feedback replies and failure injection. Register replies
//! arrive on each motor's MST_ID like the motors' feedback. Not a hardware verification.
#![allow(dead_code)]

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, CanTransport, TransportError};

/// B601-DM firmware version read on 2026-11-22.
pub const SW_VER: u32 = 942_747_703;

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
    /// No reply of any kind.
    pub silent: bool,
    /// Register writes are acknowledged but not stored.
    pub ignore_reg_writes: bool,
    /// Value the write acknowledgement carries instead of the written one.
    pub ack_value_override: Option<u32>,
    /// Register writes get no acknowledgement.
    pub no_write_ack: bool,
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
            silent: false, ignore_reg_writes: false, ack_value_override: None, no_write_ack: false }
    }

    pub fn reg_u32(&self, rid: u8) -> u32 { u32::from_le_bytes(self.regs[&rid]) }
    pub fn set_reg_u32(&mut self, rid: u8, v: u32) { self.regs.insert(rid, v.to_le_bytes()); }

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
}

pub struct FakeBusState {
    pub motors: Vec<FakeMotor>,
    pub rx: VecDeque<CanFrame>,
    pub tx_log: Vec<CanFrame>,
}

impl FakeBusState {
    pub fn motor_mut(&mut self, id: u16) -> Option<&mut FakeMotor> { self.motors.iter_mut().find(|m| m.id == id) }

    pub fn push_feedback(&mut self, i: usize) {
        if !self.motors[i].silent {
            let f = self.motors[i].feedback_frame();
            self.rx.push_back(f);
        }
    }

    pub fn push_all_feedback(&mut self) { for i in 0..self.motors.len() { self.push_feedback(i); } }

    fn handle(&mut self, frame: &CanFrame) {
        if frame.id == 0x7FF {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let (op, rid) = (frame.data[2], frame.data[3]);
            let Some(k) = self.motors.iter().position(|m| m.id == id) else { return };
            let m = &mut self.motors[k];
            if m.silent { return; }
            let reply = match op {
                0xCC => Some(m.feedback_frame()),
                damiao::REG_READ => m.regs.get(&rid).map(|v| m.reg_reply(damiao::REG_READ, rid, *v)),
                damiao::REG_WRITE => {
                    let value = [frame.data[4], frame.data[5], frame.data[6], frame.data[7]];
                    if !m.ignore_reg_writes { m.regs.insert(rid, value); }
                    let ack = m.ack_value_override.map_or(value, u32::to_le_bytes);
                    (!m.no_write_ack).then(|| m.reg_reply(damiao::REG_WRITE, rid, ack))
                }
                _ => None,
            };
            if let Some(r) = reply { self.rx.push_back(r); }
            return;
        }
        let cmd = frame.data;
        if let Some(k) = self.motors.iter().position(|m| u32::from(m.id) == frame.id) {
            if cmd == damiao::CMD_ENABLE { self.motors[k].enabled = true; }
            else if cmd == damiao::CMD_DISABLE { self.motors[k].enabled = false; }
            self.push_feedback(k);
            return;
        }
        if let Some(k) = self.motors.iter().position(|m| u32::from(m.id) + 0x100 == frame.id) {
            if self.motors[k].enabled {
                self.motors[k].q = f32::from_le_bytes([cmd[0], cmd[1], cmd[2], cmd[3]]);
            }
            self.push_feedback(k);
        }
    }
}

#[derive(Clone)]
pub struct FakeBus(pub Arc<Mutex<FakeBusState>>);

impl FakeBus {
    pub fn new(limits: &[DamiaoLimits]) -> Self {
        let motors = limits.iter().enumerate().map(|(i, l)| FakeMotor::new(i as u16 + 1, *l)).collect();
        Self(Arc::new(Mutex::new(FakeBusState { motors, rx: VecDeque::new(), tx_log: Vec::new() })))
    }
    pub fn with<R>(&self, f: impl FnOnce(&mut FakeBusState) -> R) -> R { f(&mut self.0.lock().unwrap()) }
    pub fn tx_log(&self) -> Vec<CanFrame> { self.with(|s| s.tx_log.clone()) }
}

impl CanTransport for FakeBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.tx_log.push(*frame);
        s.handle(frame);
        Ok(())
    }
    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.0.lock().unwrap().rx.pop_front().ok_or(TransportError::WouldBlock)
    }
    fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
}

/// No frame on the bus ever used the flash store opcode 0xAA (register access on 0x7FF).
pub fn assert_no_flash_write(log: &[CanFrame]) {
    for f in log {
        assert!(!(f.id == 0x7FF && f.data[2] == 0xAA), "flash store frame on the bus: {f:?}");
    }
}
