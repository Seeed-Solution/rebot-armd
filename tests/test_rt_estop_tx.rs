//! E-stop during the output (design section 4): the urgent request is an atomic generation
//! checked before the tick's output and by the Damiao driver before each frame of a target
//! batch. An E-stop that arrives while the batch is on the bus stops the remaining frames of
//! that batch, and the same tick applies the stop policy (EStop, not a driver-write fault);
//! one that arrives during the receive suppresses the whole output of that tick.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use armd::driver::DamiaoArmDriver;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::safety::SafetyState;
use armd::transport::{CanFrame, CanTransport, TransportError};
use armd::{ArmConfig, SupervisorInner};

/// When the bus raises the E-stop request (as `ArmSupervisor::trigger_e_stop` does).
#[derive(Clone, Copy, PartialEq)]
enum Trigger {
    Off,
    /// After this many target frames of the next batch.
    AfterTargets(usize),
    /// On the next refresh/receive (before the tick's output).
    OnReceive,
}

struct Shared {
    trigger: Trigger,
    targets_sent: usize,
    requested: Option<(Arc<AtomicU64>, Arc<AtomicBool>)>,
}

/// Damiao motors answering every command frame with a feedback frame; counts target frames
/// (POS_VEL 0x100 + id, POS_FORCE 0x300 + id) and raises the E-stop request on its trigger.
struct Bus {
    motors: Vec<(u16, DamiaoLimits, f32, bool)>,
    rx: VecDeque<(CanFrame, Option<std::time::Instant>)>,
    shared: Arc<Mutex<Shared>>,
}

impl Bus {
    fn raise(&self) {
        let s = self.shared.lock().unwrap();
        if let Some((req, latched)) = &s.requested {
            req.fetch_add(1, Ordering::SeqCst);
            latched.store(true, Ordering::SeqCst);
        }
    }

    fn reply(&mut self, k: usize) {
        let (id, l, q, enabled) = self.motors[k];
        let p = damiao::float_to_uint(q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(0.0, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let data = [(u8::from(enabled) << 4) | (id as u8 & 0x0f), (p >> 8) as u8, p as u8, (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8, t as u8, 30, 30];
        self.rx.push_back((CanFrame::new_standard(0x10 + id, &data).unwrap(), Some(std::time::Instant::now())));
    }
}

impl CanTransport for Bus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let id = frame.id as u16;
        let find = |m: &[(u16, DamiaoLimits, f32, bool)], id: u16| m.iter().position(|x| x.0 == id);
        if id == 0x7FF {
            let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(k) = find(&self.motors, mid) { if frame.data[2] == 0xCC { self.reply(k); } }
        } else if let Some(k) = find(&self.motors, id) {
            if frame.data == damiao::CMD_ENABLE { self.motors[k].3 = true; }
            if frame.data == damiao::CMD_DISABLE { self.motors[k].3 = false; }
            self.reply(k);
        } else if let Some(k) = find(&self.motors, id.wrapping_sub(0x100)).or_else(|| find(&self.motors, id.wrapping_sub(0x300))) {
            self.motors[k].2 = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.reply(k);
            let fire = {
                let mut s = self.shared.lock().unwrap();
                s.targets_sent += 1;
                matches!(s.trigger, Trigger::AfterTargets(n) if s.targets_sent == n)
            };
            if fire {
                self.shared.lock().unwrap().trigger = Trigger::Off;
                self.raise();
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        let fire = self.shared.lock().unwrap().trigger == Trigger::OnReceive;
        if fire {
            self.shared.lock().unwrap().trigger = Trigger::Off;
            self.raise();
        }
        self.rx.pop_front().ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
}

fn rig() -> (SupervisorInner, Arc<Mutex<Shared>>) {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits: Vec<DamiaoLimits> = cfg.damiao_ranges.as_ref().unwrap().iter().map(|r| r.limits()).collect();
    let shared = Arc::new(Mutex::new(Shared { trigger: Trigger::Off, targets_sent: 0, requested: None }));
    let bus = Bus {
        motors: limits.iter().enumerate().map(|(i, l)| (i as u16 + 1, *l, 0.0, false)).collect(),
        rx: VecDeque::new(),
        shared: shared.clone(),
    };
    let dof = cfg.dof;
    let driver = DamiaoArmDriver::new(bus, (1..=dof as u16).collect()).with_joint_limits(limits).unwrap();
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(driver));
    shared.lock().unwrap().requested = Some((inner.requested_estop_count.clone(), inner.e_stop_latched.clone()));
    inner.lease_enabled = false;
    inner.tick(0.02);
    inner.set_torque(true).expect("arm");
    for _ in 0..5 { inner.tick(0.02); }
    let mut q = inner.commanded_positions.clone();
    q[0] += 0.5;
    let epoch = inner.safety.epoch_id();
    inner.jog(q, Some(2.0), Some(epoch)).expect("jog");
    for _ in 0..10 { inner.tick(0.02); }
    assert!(inner.active_trajectory.is_some());
    (inner, shared)
}

#[test]
fn estop_during_the_batch_stops_its_remaining_frames_and_applies_in_the_same_tick() {
    let (mut inner, shared) = rig();
    let dof = inner.config.dof;
    let action = inner.last_action.as_ref().unwrap().id;
    {
        let mut s = shared.lock().unwrap();
        s.targets_sent = 0;
        s.trigger = Trigger::AfterTargets(3);
    }
    inner.tick(0.02);
    let sent = shared.lock().unwrap().targets_sent;
    assert_eq!(sent, 3, "frames after the request were sent ({sent} of {dof})");
    assert_eq!(inner.safety.state(), &SafetyState::EStop, "applied in the same tick, not a driver-write fault");
    assert!(inner.active_trajectory.is_none(), "motion stopped");
    let a = inner.last_action.as_ref().unwrap();
    assert_eq!(a.id, action);
    assert_ne!(a.state, "done", "the interrupted motion is not reported done");
    assert_eq!(a.code.as_deref(), Some("ESTOP_ACTIVE"), "{a:?}");
    // the latched E-stop holds: later ticks still send hold frames (no dynamic command)
    let held = inner.commanded_positions.clone();
    for _ in 0..5 { inner.tick(0.02); }
    assert_eq!(inner.commanded_positions, held);
    assert_eq!(inner.safety.state(), &SafetyState::EStop);
}

#[test]
fn estop_during_the_receive_suppresses_the_whole_output_of_the_tick() {
    let (mut inner, shared) = rig();
    {
        let mut s = shared.lock().unwrap();
        s.targets_sent = 0;
        s.trigger = Trigger::OnReceive;
    }
    inner.tick(0.02);
    assert_eq!(shared.lock().unwrap().targets_sent, 0, "no target frame after the request");
    assert_eq!(inner.safety.state(), &SafetyState::EStop);
    assert!(inner.active_trajectory.is_none());
}
