//! Contract tests for the 4 Damiao `ArmDriver` forwarding extractions made in S4-rest
//! to bring `impl ArmDriver for DamiaoArmDriver` under 3000 lines before moving modules:
//! 1. `probe_hardware_disarmed_until` -> `probe_hardware_disarmed_until_impl`
//! 2. `write_estop_hold_bounded` -> `write_estop_hold_bounded_impl`
//! 3. `write_measured_hold_masked` -> `write_measured_hold_masked_impl`
//! 4. `write_joint_targets_bounded` -> `write_joint_targets_bounded_impl`

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, JointTarget, TorqueProgress, TxBatchOutcome,
};
use armd::protocol::damiao::{
    self, DamiaoLimits, CMD_DISABLE, CMD_ENABLE, CTRL_MODE_POS_VEL, POS_VEL_BASE_ID, REG_READ,
    RID_CTRL_MODE,
};
use armd::transport::{CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, TransportError};

#[derive(Clone)]
struct ForwardingTestBus {
    clock: Arc<MockClock>,
    tx: Arc<Mutex<Vec<CanFrame>>>,
    rx: Arc<Mutex<VecDeque<(CanFrame, Option<Instant>)>>>,
    motor_states: Arc<Mutex<[u8; 2]>>,
    motor_pos: Arc<Mutex<[f32; 2]>>,
}

impl ForwardingTestBus {
    fn new() -> (Self, Arc<MockClock>) {
        let clock = Arc::new(MockClock::new());
        let bus = Self {
            clock: Arc::clone(&clock),
            tx: Arc::new(Mutex::new(Vec::new())),
            rx: Arc::new(Mutex::new(VecDeque::new())),
            motor_states: Arc::new(Mutex::new([0, 0])),
            motor_pos: Arc::new(Mutex::new([0.0, 0.0])),
        };
        (bus, clock)
    }

    fn set_states(&self, s0: u8, s1: u8) {
        *self.motor_states.lock().unwrap() = [s0, s1];
    }

    fn clear_tx(&self) {
        self.tx.lock().unwrap().clear();
    }

    fn take_tx(&self) -> Vec<CanFrame> {
        std::mem::take(&mut *self.tx.lock().unwrap())
    }

    fn feedback_frame(motor_id: u8, state: u8, pos: f32) -> CanFrame {
        let lim = DamiaoLimits::default();
        let p_raw = damiao::float_to_uint(pos, lim.p_min, lim.p_max, 16);
        let v_raw = damiao::float_to_uint(0.0, lim.v_min, lim.v_max, 12);
        let t_raw = damiao::float_to_uint(0.0, lim.t_min, lim.t_max, 12);
        let data = [
            ((state & 0x0F) << 4) | (motor_id & 0x0F),
            (p_raw >> 8) as u8,
            (p_raw & 0xFF) as u8,
            (v_raw >> 4) as u8,
            (((v_raw & 0x0F) << 4) | (t_raw >> 8)) as u8,
            (t_raw & 0xFF) as u8,
            25,
            25,
        ];
        CanFrame::new_standard(0x10 + u16::from(motor_id), &data).unwrap()
    }

    fn reg_read_reply(motor_id: u16, rid: u8, value: u32) -> CanFrame {
        let [lo, hi] = motor_id.to_le_bytes();
        let [v0, v1, v2, v3] = value.to_le_bytes();
        CanFrame::new_standard(0x10 + motor_id, &[lo, hi, REG_READ, rid, v0, v1, v2, v3]).unwrap()
    }
}

impl CanTransport for ForwardingTestBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        self.tx.lock().unwrap().push(frame.clone());
        let now = self.clock.now();
        let mut rx = self.rx.lock().unwrap();
        if frame.id == 0x7FF && frame.dlc == 8 {
            let mid = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let op = frame.data[2];
            if op == 0xCC && (mid == 1 || mid == 2) {
                let idx = (mid - 1) as usize;
                let st = self.motor_states.lock().unwrap()[idx];
                let pos = self.motor_pos.lock().unwrap()[idx];
                rx.push_back((Self::feedback_frame(mid as u8, st, pos), Some(now)));
            } else if op == REG_READ && frame.data[3] == RID_CTRL_MODE && (mid == 1 || mid == 2) {
                rx.push_back((Self::reg_read_reply(mid, RID_CTRL_MODE, CTRL_MODE_POS_VEL), Some(now)));
            }
        } else if (frame.id == 1 || frame.id == 2) && frame.data == CMD_ENABLE {
            let idx = (frame.id - 1) as usize;
            self.motor_states.lock().unwrap()[idx] = 1;
            let pos = self.motor_pos.lock().unwrap()[idx];
            rx.push_back((Self::feedback_frame(frame.id as u8, 1, pos), Some(now)));
        } else if (frame.id == 1 || frame.id == 2) && frame.data == CMD_DISABLE {
            let idx = (frame.id - 1) as usize;
            self.motor_states.lock().unwrap()[idx] = 0;
            let pos = self.motor_pos.lock().unwrap()[idx];
            rx.push_back((Self::feedback_frame(frame.id as u8, 0, pos), Some(now)));
        } else if frame.id == u32::from(POS_VEL_BASE_ID + 1) || frame.id == u32::from(POS_VEL_BASE_ID + 2) {
            let mid = (frame.id - u32::from(POS_VEL_BASE_ID)) as u8;
            let idx = (mid - 1) as usize;
            let st = self.motor_states.lock().unwrap()[idx];
            let pos = f32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]]);
            self.motor_pos.lock().unwrap()[idx] = pos;
            rx.push_back((Self::feedback_frame(mid, st, pos), Some(now)));
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        if let Some((f, _)) = self.rx.lock().unwrap().pop_front() {
            Ok(f)
        } else {
            Err(TransportError::Timeout)
        }
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        if let Some((f, ts)) = self.rx.lock().unwrap().pop_front() {
            Ok((f, ts.map(CanRxTime::Monotonic)))
        } else {
            Err(TransportError::Timeout)
        }
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(Arc::clone(&self.clock) as Arc<dyn DriverClock>)
    }

    fn flush_is_noop(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
}

fn enable_two_joint_driver(
    bus: &ForwardingTestBus,
    clock: &Arc<MockClock>,
) -> DamiaoArmDriver<ForwardingTestBus> {
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2]);
    let mut prog = drv.begin_torque_budgeted(true, 64).unwrap();
    for _ in 0..40 {
        if matches!(prog, TorqueProgress::Done(_)) {
            break;
        }
        clock.advance(Duration::from_millis(5));
        prog = drv.poll_torque_budgeted(clock.now() + Duration::from_millis(50), 64).unwrap();
    }
    assert!(matches!(prog, TorqueProgress::Done(_)), "expected enable done, got {prog:?}");
    bus.clear_tx();
    drv
}

#[test]
fn probe_hardware_disarmed_until_forwarding_contract() {
    let (bus, clock) = ForwardingTestBus::new();
    let mut drv = DamiaoArmDriver::new(bus.clone(), vec![1, 2]);

    // 1. Both motors answer state == 0 -> confirmed disarmed using only 0x7FF refresh frames.
    bus.set_states(0, 0);
    let ok = drv.probe_hardware_disarmed_until(clock.now() + Duration::from_millis(200));
    assert!(ok, "expected disarmed proof when all motors answer state == 0");
    let tx = bus.take_tx();
    assert!(!tx.is_empty(), "expected 0xCC refresh frames");
    assert!(tx.iter().all(|f| f.id == 0x7FF && f.data[2] == 0xCC), "only 0xCC refresh frames allowed");

    // 2. One motor answers state == 1 -> fast-fail false.
    clock.advance(Duration::from_millis(350));
    bus.set_states(0, 1);
    let mut drv2 = DamiaoArmDriver::new(bus.clone(), vec![1, 2]);
    let ok_enabled = drv2.probe_hardware_disarmed_until(clock.now() + Duration::from_millis(200));
    assert!(!ok_enabled, "expected false when a motor answers state == 1");

    // 3. Active enabled driver skips probe with 0 TX frames.
    clock.advance(Duration::from_millis(20));
    let mut enabled_drv = enable_two_joint_driver(&bus, &clock);
    let ok_skip = enabled_drv.probe_hardware_disarmed_until(clock.now() + Duration::from_millis(200));
    assert!(!ok_skip, "active enabled driver must skip disarm probe");
    assert!(bus.take_tx().is_empty(), "skipped disarm probe must transmit 0 frames");
}

#[test]
fn write_estop_hold_bounded_forwarding_contract() {
    let (bus, clock) = ForwardingTestBus::new();
    let mut drv = enable_two_joint_driver(&bus, &clock);

    // Negative path: joint count mismatch refuses before sending any frame.
    let err = drv.write_estop_hold_bounded(&[JointTarget::position(0.1)]);
    assert!(matches!(err, Err(ArmDriverError::JointCountMismatch { expected: 2, actual: 1 })));
    assert!(bus.take_tx().is_empty());

    // Positive path: sends PosVel hold frames for both joints at their measured positions (~0.0).
    clock.advance(Duration::from_millis(5));
    let out = drv
        .write_estop_hold_bounded(&[JointTarget::position(0.25), JointTarget::position(-0.5)])
        .expect("estop hold should succeed");
    assert_eq!(out, TxBatchOutcome::Complete);
    let tx = bus.take_tx();
    let ids: Vec<u32> = tx.iter().map(|f| f.id).collect();
    assert_eq!(ids, vec![u32::from(POS_VEL_BASE_ID + 1), u32::from(POS_VEL_BASE_ID + 2)]);
    let p0 = f32::from_le_bytes(tx[0].data[0..4].try_into().unwrap());
    let p1 = f32::from_le_bytes(tx[1].data[0..4].try_into().unwrap());
    assert!(p0.abs() < 1e-3, "expected estop hold near measured pos 0.0, got {p0}");
    assert!(p1.abs() < 1e-3, "expected estop hold near measured pos 0.0, got {p1}");
}

#[test]
fn write_measured_hold_masked_forwarding_contract() {
    let (bus, clock) = ForwardingTestBus::new();
    let mut drv = enable_two_joint_driver(&bus, &clock);

    // Negative path: joint count mismatch refuses with 0 TX frames.
    let err = drv.write_measured_hold_masked(0b01, &[JointTarget::position(0.0)]);
    assert!(matches!(err, Err(ArmDriverError::JointCountMismatch { expected: 2, actual: 1 })));
    assert!(bus.take_tx().is_empty());

    // Positive path: hold_mask = 0b01 sends ONLY joint 0 (0x101) and leaves joint 1 untouched.
    clock.advance(Duration::from_millis(5));
    let report = drv
        .write_measured_hold_masked(0b01, &[JointTarget::position(0.3), JointTarget::position(-0.4)])
        .expect("masked hold should succeed");
    assert!(report.sent_any);
    let tx = bus.take_tx();
    let ids: Vec<u32> = tx.iter().map(|f| f.id).collect();
    assert_eq!(ids, vec![u32::from(POS_VEL_BASE_ID + 1)]);
    assert_eq!(f32::from_le_bytes(tx[0].data[0..4].try_into().unwrap()), 0.3);
}

#[test]
fn write_joint_targets_bounded_forwarding_contract() {
    let (bus, clock) = ForwardingTestBus::new();
    let mut drv = enable_two_joint_driver(&bus, &clock);

    // Negative path: joint count mismatch refuses with 0 TX frames.
    let err = drv.write_joint_targets_bounded(&[JointTarget::position(0.1)]);
    assert!(matches!(err, Err(ArmDriverError::JointCountMismatch { expected: 2, actual: 1 })));
    assert!(bus.take_tx().is_empty());

    // Positive path: sends PosVel target frames with exact float32 LE payloads.
    clock.advance(Duration::from_millis(5));
    let out = drv
        .write_joint_targets_bounded(&[JointTarget::position(0.75), JointTarget::position(-0.25)])
        .expect("bounded target write should succeed");
    assert_eq!(out, TxBatchOutcome::Complete);
    let tx = bus.take_tx();
    let ids: Vec<u32> = tx.iter().map(|f| f.id).collect();
    assert_eq!(ids, vec![u32::from(POS_VEL_BASE_ID + 1), u32::from(POS_VEL_BASE_ID + 2)]);
    assert_eq!(f32::from_le_bytes(tx[0].data[0..4].try_into().unwrap()), 0.75);
    assert_eq!(f32::from_le_bytes(tx[1].data[0..4].try_into().unwrap()), -0.25);
}
