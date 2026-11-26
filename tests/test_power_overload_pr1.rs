//! PR 1 tests: synchronized feedback-loss annotation and pre-enable CTRL_MODE/TIMEOUT
//! verification on startup and after BusLoss, across both legacy and RT executor owners.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{gripper_drive_from_profile, ArmDriver, DamiaoArmDriver, GripperPosForce};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{BootId, Capacities, ClientId, Outcome, Submit, TransitionOp};
use armd::rt_core::ops::{CoreOps, ReleaseFixture, VirtualClock};
use armd::transport::{CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, TransportError};
use armd::{ArmConfig, FaultCode, FaultReason, SafetyState, SupervisorInner};

const DT: f64 = 0.02;
const BOOT: BootId = BootId(0xB601_9001);

fn ms(v: u64) -> Duration {
    Duration::from_millis(v)
}

#[derive(Clone, Copy)]
struct MotorModel {
    id: u16,
    q: f32,
    v: f32,
    tau: f32,
    enabled: bool,
    silent: bool,
    ctrl_mode: u32,
    timeout_counts: u32,
    answer_timeout: bool,
    limits: DamiaoLimits,
}

impl MotorModel {
    fn fb_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(self.tau, l.t_min, l.t_max, 12);
        let state = u8::from(self.enabled);
        let data = [
            ((state & 0x0F) << 4) | (self.id as u8 & 0x0F),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0F) << 4) | ((t >> 8) & 0x0F)) as u8,
            t as u8,
            30,
            30,
        ];
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

struct BusInner {
    clock: Arc<MockClock>,
    motors: [MotorModel; 7],
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    tx_log: Vec<CanFrame>,
}

#[derive(Clone)]
struct FakeBus(Arc<Mutex<BusInner>>);

impl FakeBus {
    fn new(limits: &[DamiaoLimits; 7]) -> (Self, Arc<MockClock>) {
        let clock = Arc::new(MockClock::new());
        let motors = std::array::from_fn(|i| MotorModel {
            id: (i + 1) as u16,
            q: 0.0,
            v: 0.0,
            tau: 0.0,
            enabled: false,
            silent: false,
            ctrl_mode: if i == 6 { damiao::CTRL_MODE_POS_FORCE } else { damiao::CTRL_MODE_POS_VEL },
            timeout_counts: if i == 6 { 0 } else { 2000 },
            answer_timeout: true,
            limits: limits[i],
        });
        (
            Self(Arc::new(Mutex::new(BusInner {
                clock: clock.clone(),
                motors,
                rx: VecDeque::new(),
                tx_log: Vec::new(),
            }))),
            clock,
        )
    }

    fn with<R>(&self, f: impl FnOnce(&mut BusInner) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }

    fn push_all_fb(&self) {
        self.with(|s| {
            let now = s.clock.now();
            for m in &s.motors {
                if !m.silent {
                    s.rx.push_back((m.fb_frame(), Some(CanRxTime::Monotonic(now))));
                }
            }
        });
    }

    fn push_motor_fb(&self, idx: usize) {
        self.with(|s| {
            let now = s.clock.now();
            let m = &s.motors[idx];
            if !m.silent {
                s.rx.push_back((m.fb_frame(), Some(CanRxTime::Monotonic(now))));
            }
        });
    }

    fn push_reg_reply(&self, motor_id: u16, rid: u8, val: u32) {
        self.with(|s| {
            let now = s.clock.now();
            let [lo, hi] = motor_id.to_le_bytes();
            let b = val.to_le_bytes();
            let f = CanFrame::new_standard(
                0x10 + motor_id,
                &[lo, hi, damiao::REG_READ, rid, b[0], b[1], b[2], b[3]],
            )
            .unwrap();
            s.rx.push_back((f, Some(CanRxTime::Monotonic(now))));
        });
    }
}

impl CanTransport for FakeBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone())
    }
    fn supports_nonblocking_rt(&self) -> bool { true }
    fn supports_bounded_flush(&self) -> bool { true }
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.tx_log.push(*frame);
        let now = s.clock.now();

        if frame.id == 0x7FF && frame.data[2] == damiao::REG_READ {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            if let Some(m) = s.motors.iter().find(|m| m.id == id && !m.silent).copied() {
                let val = match rid {
                    damiao::RID_CTRL_MODE => Some(m.ctrl_mode.to_le_bytes()),
                    damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                    damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                    damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                    damiao::RID_TIMEOUT if m.answer_timeout => Some(m.timeout_counts.to_le_bytes()),
                    _ => None,
                };
                if let Some(v) = val {
                    let [lo, hi] = id.to_le_bytes();
                    let rep = CanFrame::new_standard(
                        0x10 + id,
                        &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                    )
                    .unwrap();
                    s.rx.push_back((rep, Some(CanRxTime::Monotonic(now))));
                }
            }
            return Ok(());
        }

        if frame.id == 0x7FF && frame.data[2] == damiao::REG_WRITE {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            let v = u32::from_le_bytes([frame.data[4], frame.data[5], frame.data[6], frame.data[7]]);
            if let Some(m) = s.motors.iter_mut().find(|m| m.id == id && !m.silent) {
                if rid == damiao::RID_CTRL_MODE {
                    m.ctrl_mode = v;
                }
                let rep = CanFrame::new_standard(0x10 + id, &frame.data).unwrap();
                s.rx.push_back((rep, Some(CanRxTime::Monotonic(now))));
            }
            return Ok(());
        }

        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(m) = s.motors.iter().find(|m| m.id == id && !m.silent).copied() {
                s.rx.push_back((m.fb_frame(), Some(CanRxTime::Monotonic(now))));
            }
            return Ok(());
        }

        if frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] {
            let id = (frame.id & 0xFF) as u16;
            if let Some(m) = s.motors.iter_mut().find(|m| m.id == id && !m.silent) {
                m.enabled = true;
                let fb = m.fb_frame();
                s.rx.push_back((fb, Some(CanRxTime::Monotonic(now))));
            }
            return Ok(());
        }

        if frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD] {
            let id = (frame.id & 0xFF) as u16;
            if let Some(m) = s.motors.iter_mut().find(|m| m.id == id && !m.silent) {
                m.enabled = false;
                let fb = m.fb_frame();
                s.rx.push_back((fb, Some(CanRxTime::Monotonic(now))));
            }
            return Ok(());
        }

        // Control frame (0x100 + id or 0x300 + id): reply with feedback if not silent.
        let id = (frame.id & 0xFF) as u16;
        if let Some(m) = s.motors.iter().find(|m| m.id == id && !m.silent).copied() {
            s.rx.push_back((m.fb_frame(), Some(CanRxTime::Monotonic(now))));
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_timestamped().map(|(f, _)| f)
    }

    fn receive_timestamped(&mut self) -> Result<(CanFrame, Option<Instant>), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.rx.pop_front()
            .map(|(f, ts)| {
                let inst = ts.map(|t| match t {
                    CanRxTime::Monotonic(i) => i,
                    _ => unreachable!(),
                });
                (f, inst)
            })
            .ok_or(TransportError::WouldBlock)
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.rx.pop_front().ok_or(TransportError::WouldBlock)
    }

    fn flush(&mut self) -> Result<(), TransportError> { Ok(()) }
}

fn b601_limits() -> [DamiaoLimits; 7] {
    let cfg = ArmConfig::rebot_dm();
    let r = cfg.damiao_ranges.unwrap();
    std::array::from_fn(|i| r[i].limits())
}

fn make_driver(bus: FakeBus, cfg: &ArmConfig) -> DamiaoArmDriver<FakeBus> {
    let limits = b601_limits();
    let drive = gripper_drive_from_profile(cfg).unwrap();
    let mut d = DamiaoArmDriver::new(bus, (1..=7).collect())
        .with_joint_limits(limits.to_vec())
        .unwrap()
        .with_gripper_drive(drive)
        .unwrap();
    if let Some(exp) = cfg.expected_timeout_counts.clone() {
        d = d.with_expected_timeout_counts(exp).unwrap();
    }
    d
}

#[test]
fn expected_timeout_counts_profile_validation_and_kit_defaults() {
    let def = ArmConfig::default();
    assert_eq!(def.expected_timeout_counts, None, "absent by default");

    let kit = ArmConfig::rebot_dm();
    assert_eq!(
        kit.expected_timeout_counts,
        Some(vec![Some(2000), Some(2000), Some(2000), Some(2000), Some(2000), Some(2000), Some(0)])
    );
    kit.clone().validate().unwrap();

    let mut bad_len = kit.clone();
    bad_len.expected_timeout_counts = Some(vec![Some(2000); 6]);
    assert!(bad_len.validate().unwrap_err().contains("expected_timeout_counts"));

    let mut bad_range = kit;
    bad_range.expected_timeout_counts = Some(vec![Some(20_001), Some(2000), Some(2000), Some(2000), Some(2000), Some(2000), Some(0)]);
    assert!(bad_range.validate().unwrap_err().contains("expected_timeout_counts"));
}

#[test]
fn pre_enable_timeout_verification_missing_mismatch_stale_and_valid_j7_zero() {
    let cfg = ArmConfig::rebot_dm();
    let limits = b601_limits();

    // 1. Valid kit values (J1-J6 = 2000, J7 = 0 pos-force) succeed.
    {
        let (bus, clock) = FakeBus::new(&limits);
        let mut drv = make_driver(bus.clone(), &cfg);
        bus.push_all_fb();
        drv.read_joint_states().unwrap();
        clock.advance(ms(5));
        bus.push_all_fb();
        drv.set_torque(true).expect("kit values J1-J6=2000, J7=0 must enable");
        assert!(drv.is_enabled());
    }

    // 2. J1-J6 TIMEOUT = 0 (expected 2000) -> refused with joint, expected, observed; no TIMEOUT write.
    {
        let (bus, clock) = FakeBus::new(&limits);
        bus.with(|s| s.motors[2].timeout_counts = 0); // J3 reset to 0
        let mut drv = make_driver(bus.clone(), &cfg);
        bus.push_all_fb();
        drv.read_joint_states().unwrap();
        clock.advance(ms(5));
        bus.push_all_fb();
        let err = drv.set_torque(true).unwrap_err().to_string();
        assert!(err.contains("joint 3"), "must name joint: {err}");
        assert!(err.contains("2000"), "must name expected: {err}");
        assert!(err.contains("0"), "must name observed: {err}");
        let wrote_timeout = bus.with(|s| {
            s.tx_log
                .iter()
                .any(|f| f.id == 0x7FF && f.data[2] == damiao::REG_WRITE && f.data[3] == damiao::RID_TIMEOUT)
        });
        assert!(!wrote_timeout, "enable must never write TIMEOUT");
    }

    // 3. Missing TIMEOUT reply -> refused.
    {
        let (bus, clock) = FakeBus::new(&limits);
        bus.with(|s| s.motors[0].answer_timeout = false);
        let mut drv = make_driver(bus.clone(), &cfg);
        bus.push_all_fb();
        drv.read_joint_states().unwrap();
        clock.advance(ms(5));
        bus.push_all_fb();
        let err = drv.set_torque(true).unwrap_err().to_string();
        assert!(err.contains("joint 1") && err.contains("TIMEOUT"), "got: {err}");
    }

    // 4. Stale pre-boundary TIMEOUT reply queued before enable must NOT satisfy causal check.
    {
        let (bus, clock) = FakeBus::new(&limits);
        bus.with(|s| s.motors[0].answer_timeout = false);
        let mut drv = make_driver(bus.clone(), &cfg);
        bus.push_all_fb();
        drv.read_joint_states().unwrap();
        // Inject a stale J1 TIMEOUT=2000 reply into RX BEFORE begin_enable drains RX.
        bus.push_reg_reply(1, damiao::RID_TIMEOUT, 2000);
        clock.advance(ms(5));
        bus.push_all_fb();
        let err = drv.set_torque(true).unwrap_err().to_string();
        assert!(err.contains("joint 1") && err.contains("TIMEOUT"), "stale reply must be ignored: {err}");
    }

    // 5. Explicit J7 pos-force zero works when J7=0, refuses if J7=2000.
    {
        let (bus, clock) = FakeBus::new(&limits);
        bus.with(|s| s.motors[6].timeout_counts = 2000);
        let mut drv = make_driver(bus.clone(), &cfg);
        let _ = GripperPosForce {
            joint: 6, current_raw: 460, close_speed_rad_s: 0.3, closed_rad: 0.0, open_rad: -5.7,
            calibrated: false, ceiling_torque_nm: None, torque_reference: None,
        };
        bus.push_all_fb();
        drv.read_joint_states().unwrap();
        clock.advance(ms(5));
        bus.push_all_fb();
        let err = drv.set_torque(true).unwrap_err().to_string();
        assert!(err.contains("joint 7") && err.contains("0") && err.contains("2000"), "got: {err}");
    }
}

#[test]
fn synchronized_loss_annotates_bus_loss_even_if_j7_faults_first_and_requires_post_bus_loss_verify_legacy() {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits = b601_limits();
    let (bus, clock) = FakeBus::new(&limits);
    let drv = make_driver(bus.clone(), &cfg);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    bus.push_all_fb();
    inner.tick(DT);
    clock.advance(ms(20));
    bus.push_all_fb();
    {
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
    }
    assert!(inner.is_torqued);

    // Run loaded ticks (~8.56 A via J4 torque ~ -9.46 N·m) so power estimate is populated.
    bus.with(|s| {
        s.motors[3].tau = -9.46;
        s.motors[1].tau = 8.0;
        s.motors[2].tau = 6.0;
    });
    for _ in 0..20 {
        clock.advance(ms(20));
        bus.push_all_fb();
        inner.tick(DT);
    }
    let last_bus_a = inner.power.bus_a;
    assert!(inner.power.valid && last_bus_a > 5.0, "expected high power estimate, got {last_bus_a}");

    // Synchronized silence across all 7 motors (within 40 ms <= 100 ms window).
    // Stagger J7 slightly earlier so J7FeedbackStale triggers at 250 ms before BusLoss at 500 ms.
    clock.advance(ms(20));
    for j in 0..6 {
        bus.push_motor_fb(j);
    }
    inner.tick(DT);
    bus.with(|s| for m in &mut s.motors { m.silent = true; m.enabled = false; m.timeout_counts = 0; });

    // Advance past J7FeedbackStale (250 ms) — J7 faults first!
    for _ in 0..14 {
        clock.advance(ms(20));
        inner.tick(DT);
    }
    assert!(inner.safety.has_latched_fault_kind(armd::FaultKind::J7FeedbackStale), "J7 must fault first");

    // Advance past BUS_LOSS_TIMEOUT (500 ms) — BusLoss latches and keeps synchronized evidence + power estimate!
    for _ in 0..16 {
        clock.advance(ms(20));
        inner.tick(DT);
    }
    assert!(inner.safety.has_latched_fault_kind(armd::FaultKind::BusLoss));
    assert_eq!(*inner.safety.state(), SafetyState::Fault(FaultReason::BusLoss));
    let fi = inner.safety.fault().expect("fault info");
    assert_eq!(fi.code, FaultCode::BusLoss);
    assert!(!fi.recoverable);
    let msg = fi.message.as_str();
    assert!(
        msg.contains("Suspected motor power loss or shared CAN interruption."),
        "missing synchronized loss annotation: {msg}"
    );
    assert!(
        msg.contains("last power estimate"),
        "preserved power estimate missing: {msg}"
    );

    // Motors recover after reboot with TIMEOUT=0 on J1-J6.
    bus.with(|s| for m in &mut s.motors { m.silent = false; });
    for _ in 0..4 {
        clock.advance(ms(20));
        bus.push_all_fb();
        inner.tick(DT);
    }
    inner.clear_fault().expect("BusLoss clear rules unchanged once feedback is fresh");

    // First enable after BusLoss MUST re-read TIMEOUT and refuse on J1-J6 = 0!
    let err = {
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a))
    }
    .unwrap_err()
    .to_string();
    assert!(
        err.contains("joint 1 TIMEOUT readback mismatch: expected 2000 counts, observed 0 counts"),
        "expected exact J1 TIMEOUT mismatch, got: {err}"
    );
}

#[test]
fn single_axis_silence_does_not_annotate_synchronized_power_loss() {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits = b601_limits();
    let (bus, clock) = FakeBus::new(&limits);
    let drv = make_driver(bus.clone(), &cfg);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    bus.push_all_fb();
    inner.tick(DT);
    clock.advance(ms(20));
    bus.push_all_fb();
    {
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
    }

    // Silence ONLY J3; all other motors keep replying every tick.
    bus.with(|s| s.motors[2].silent = true);
    for _ in 0..30 {
        clock.advance(ms(20));
        bus.push_all_fb();
        inner.tick(DT);
    }
    assert!(inner.safety.has_latched_fault_kind(armd::FaultKind::BusLoss));
    let msg = inner.safety.fault().unwrap().message;
    assert!(
        !msg.as_str().contains("Suspected motor power loss"),
        "single-axis silence must not claim synchronized power loss: {msg}"
    );
}

#[test]
fn synchronized_loss_and_post_bus_loss_timeout_refusal_on_rt_executor() {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits = b601_limits();
    let (bus, clock) = FakeBus::new(&limits);
    let drv = make_driver(bus.clone(), &cfg);

    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.rt_executor_owned = true;
    inner.register_live_connection(1).unwrap();
    bus.push_all_fb();
    inner.tick(DT);

    let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new_with_clock(
        inner,
        BOOT,
        Capacities { commands: 8, plans: 2, replies: 16 },
        VirtualClock::from_mock_clock((*clock).clone()),
    );

    // Enable via RT executor broker transition.
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(
            ClientId(1),
            1,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
    let mut ended = None;
    for _ in 0..30 {
        bus.push_all_fb();
        let rep = fx.step();
        if let Some((_, o)) = rep.transition_ended {
            ended = Some(o);
            break;
        }
    }
    assert_eq!(ended, Some(Outcome::Completed), "initial RT enable failed");
    assert!(fx.core.is_torqued);

    // Load the arm, then synchronized silence across all 7 motors (with reboot TIMEOUT=0).
    bus.with(|s| {
        s.motors[3].tau = -9.46;
        s.motors[1].tau = 8.0;
    });
    for _ in 0..15 {
        bus.push_all_fb();
        let _ = fx.step();
    }
    bus.with(|s| for m in &mut s.motors { m.silent = true; m.enabled = false; m.timeout_counts = 0; });
    for _ in 0..32 {
        let _ = fx.step();
    }
    let fi = fx.core.safety.fault().expect("BusLoss fault on RT executor");
    assert_eq!(fi.code, FaultCode::BusLoss);
    assert!(
        fi.message.as_str().contains("Suspected motor power loss or shared CAN interruption."),
        "RT executor missing synchronized annotation: {}",
        fi.message
    );

    // Motors recover with TIMEOUT=0; clear fault via RT executor, then attempt Enable -> refused.
    bus.with(|s| for m in &mut s.motors { m.silent = false; });
    for _ in 0..4 {
        bus.push_all_fb();
        let _ = fx.step();
    }
    fx.core.clear_fault().expect("RT clear_fault after fresh feedback");

    bus.with(|s| s.tx_log.clear());
    let auth = fx.core.authority(&fx.ctx(fx.now()));
    broker
        .submit(
            ClientId(1),
            2,
            auth,
            fx.now() + Duration::from_secs(5),
            Submit::Transition(TransitionOp::TorqueOn),
        )
        .unwrap();
    let mut rearm_out = None;
    for _ in 0..30 {
        bus.push_all_fb();
        let rep = fx.step();
        if let Some((_, o)) = rep.transition_ended {
            rearm_out = Some(o);
            break;
        }
    }
    assert!(
        matches!(rearm_out, Some(Outcome::Failed(_))),
        "RT re-enable must fail on TIMEOUT=0: {rearm_out:?}"
    );
    let rt_cause = fx.core.safety.fault().map(|f| f.message.as_str().to_string()).unwrap_or_default();
    assert!(
        rt_cause.contains("joint 1 TIMEOUT readback mismatch: expected 2000 counts, observed 0 counts"),
        "RT re-enable must report exact J1 TIMEOUT mismatch, got: {rt_cause}"
    );
    assert!(
        !fx.core.safety.state().is_operational(),
        "arm must not become operational when TIMEOUT=0: {:?}",
        fx.core.safety.state()
    );
    let sent_enable = bus.with(|s| {
        s.tx_log
            .iter()
            .any(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    });
    assert!(!sent_enable, "RT re-enable must send zero CMD_ENABLE frames on TIMEOUT mismatch");
    assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
}

#[test]
fn b1_deferred_tx_stale_reply_and_spoofed_reply_id_rejected() {
    let cfg = ArmConfig::rebot_dm();
    let limits = b601_limits();

    // Case 1 (B1): TX budget defers J2's TIMEOUT REG_READ; a stale J2 TIMEOUT=2000 reply arrives
    // before J2's new REG_READ is transmitted while J2 is rebooted/silent -> MUST refuse enable!
    {
        let (bus, clock) = FakeBus::new(&limits);
        let mut drv = make_driver(bus.clone(), &cfg).with_tx_budget(armd::driver::TxBudget {
            max_frames: 1,
            max_duration: ms(10),
            frame_max_age: ms(500),
        });
        bus.push_all_fb();
        let _ = drv.read_joint_states();
        // Admission invariant rejects max_frames=1 for 7 motors (requires >= 8).
        assert!(matches!(
            drv.begin_torque(true),
            Err(armd::driver::ArmDriverError::WatchdogBudgetInsufficient {
                max_frames: 1,
                required: 8
            })
        ));
        // Enable under valid max_frames=8, then tighten to 1 frame/poll during TIMEOUT readback
        // so J2's TIMEOUT REG_READ is deferred and a stale J2 reply arrives first.
        drv.set_tx_budget(armd::driver::TxBudget {
            max_frames: 8,
            max_duration: ms(10),
            frame_max_age: ms(500),
        });
        bus.with(|s| {
            s.motors[1].answer_timeout = false;
            s.motors[1].timeout_counts = 0;
        });
        let dl = clock.now() + Duration::from_secs(2);
        drv.begin_torque(true).unwrap();
        drv.set_tx_budget(armd::driver::TxBudget {
            max_frames: 1,
            max_duration: ms(10),
            frame_max_age: ms(500),
        });
        let mut injected_stale = false;
        let mut final_err = None;
        for _ in 0..80 {
            clock.advance(ms(5));
            bus.push_all_fb();
            let j1_timeout_sent = bus.with(|s| {
                s.tx_log
                    .iter()
                    .any(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_TIMEOUT)
            });
            let j2_timeout_sent = bus.with(|s| {
                s.tx_log
                    .iter()
                    .any(|f| f.id == 0x7FF && f.data[0] == 2 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_TIMEOUT)
            });
            if j1_timeout_sent && !j2_timeout_sent && !injected_stale {
                bus.push_reg_reply(2, damiao::RID_TIMEOUT, 2000);
                injected_stale = true;
            }
            let _ = drv.read_joint_states();
            match drv.poll_torque(dl) {
                Ok(armd::driver::TorqueProgress::Pending { .. }) => {}
                Ok(armd::driver::TorqueProgress::Done(_)) => panic!("stale pre-TX J2 TIMEOUT reply must NOT authorize enable!"),
                Err(e) => {
                    final_err = Some(e);
                    break;
                }
            }
        }
        assert!(injected_stale, "expected J2 TIMEOUT TX to be deferred behind J1 under 1-frame budget");
        let err = final_err.expect("expected TIMEOUT missing refusal").to_string();
        assert!(
            err.contains("joint 2 TIMEOUT readback: no fresh reply"),
            "expected J2 TIMEOUT missing, got: {err}"
        );
    }

    // Case 2 (B1): Spoofed reply CAN ID (0x16 claiming motor_id=1) rejected as ambiguous!
    {
        let (bus, clock) = FakeBus::new(&limits);
        bus.with(|s| s.motors[0].answer_timeout = false);
        let mut drv = make_driver(bus.clone(), &cfg);
        bus.push_all_fb();
        let _ = drv.read_joint_states();
        let dl = clock.now() + Duration::from_secs(2);
        drv.begin_torque(true).unwrap();
        let mut injected = false;
        let mut err = None;
        for _ in 0..40 {
            clock.advance(ms(5));
            bus.push_all_fb();
            let j1_timeout_sent = bus.with(|s| {
                s.tx_log
                    .iter()
                    .any(|f| f.id == 0x7FF && f.data[0] == 1 && f.data[2] == damiao::REG_READ && f.data[3] == damiao::RID_TIMEOUT)
            });
            if j1_timeout_sent && !injected {
                // Wrong CAN ID 0x16 (motor 6's MST_ID) claiming motor_id=1!
                let b = 2000u32.to_le_bytes();
                let spoof = CanFrame::new_standard(0x16, &[1, 0, damiao::REG_READ, damiao::RID_TIMEOUT, b[0], b[1], b[2], b[3]]).unwrap();
                let now = clock.now();
                bus.with(|s| s.rx.push_back((spoof, Some(CanRxTime::Monotonic(now)))));
                injected = true;
            }
            let _ = drv.read_joint_states();
            match drv.poll_torque(dl) {
                Ok(armd::driver::TorqueProgress::Pending { .. }) => {}
                Ok(armd::driver::TorqueProgress::Done(_)) => panic!("spoofed reply CAN ID must NOT authorize enable"),
                Err(e) => {
                    err = Some(e);
                    break;
                }
            }
        }
        assert!(injected);
        let msg = err.expect("spoofed reply ID must fail").to_string();
        assert!(msg.contains("joint 1 TIMEOUT readback: no fresh reply"), "got: {msg}");
    }
}

#[test]
fn b2_aborted_enable_preserves_startup_readback_requirement() {
    let cfg = ArmConfig::rebot_dm();
    let limits = b601_limits();
    let (bus, clock) = FakeBus::new(&limits);
    let mut drv = make_driver(bus.clone(), &cfg);
    bus.push_all_fb();
    let _ = drv.read_joint_states();
    let dl = clock.now() + Duration::from_secs(2);
    drv.begin_torque(true).unwrap();
    // Advance poll-by-poll until CTRL_MODE and TIMEOUT readbacks have BOTH passed (i.e. J7
    // pre-enable hold 0x307 or CMD_ENABLE is sent), without pushing post-enable state=1 feedback,
    // then abort BEFORE enable confirms!
    let mut reached_post_readback = false;
    for _ in 0..10 {
        clock.advance(ms(5));
        let _ = drv.read_joint_states();
        match drv.poll_torque(dl).unwrap() {
            armd::driver::TorqueProgress::Pending { .. } => {
                if bus.with(|s| {
                    s.tx_log
                        .iter()
                        .any(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
                }) {
                    reached_post_readback = true;
                    break;
                }
            }
            armd::driver::TorqueProgress::Done(_) => panic!("must not confirm without enabled feedback"),
        }
    }
    assert!(reached_post_readback, "expected enable to pass readbacks and send CMD_ENABLE before abort");
    assert!(drv.torque_transition_pending(), "enable must still be pending before abort");
    drv.abort_enable();

    // Now J1 reboots with TIMEOUT=0. The next enable MUST NOT reuse proofs from the aborted attempt!
    bus.with(|s| {
        s.motors[0].timeout_counts = 0;
        s.tx_log.clear();
    });
    let dl2 = clock.now() + Duration::from_secs(2);
    drv.begin_torque(true).unwrap();
    let mut err = None;
    for _ in 0..40 {
        clock.advance(ms(10));
        bus.push_all_fb();
        let _ = drv.read_joint_states();
        match drv.poll_torque(dl2) {
            Ok(armd::driver::TorqueProgress::Pending { .. }) => {}
            Ok(armd::driver::TorqueProgress::Done(_)) => panic!("aborted enable must not leak TIMEOUT proofs!"),
            Err(e) => {
                err = Some(e);
                break;
            }
        }
    }
    let msg = err.expect("retried enable must fail on J1 TIMEOUT=0").to_string();
    assert!(
        msg.contains("joint 1 TIMEOUT readback mismatch: expected 2000 counts, observed 0 counts"),
        "got: {msg}"
    );
}

#[test]
fn m1_timeout_verification_mandatory_when_expected_omitted_or_axis_null() {
    let cfg = ArmConfig::rebot_dm();
    let limits = b601_limits();
    let drive = gripper_drive_from_profile(&cfg).unwrap();
    for expected_opt in [None, Some(vec![None; 7])] {
        let (bus, clock) = FakeBus::new(&limits);
        bus.with(|s| s.motors[1].answer_timeout = false);
        let mut drv = DamiaoArmDriver::new(bus.clone(), (1..=7).collect())
            .with_joint_limits(limits.to_vec())
            .unwrap()
            .with_gripper_drive(drive.clone())
            .unwrap()
            .with_timeout_verification(expected_opt.clone())
            .unwrap();
        bus.push_all_fb();
        let _ = drv.read_joint_states();
        let dl = clock.now() + Duration::from_secs(2);
        drv.begin_torque(true).unwrap();
        let mut err = None;
        for _ in 0..40 {
            clock.advance(ms(10));
            bus.push_all_fb();
            let _ = drv.read_joint_states();
            match drv.poll_torque(dl) {
                Ok(armd::driver::TorqueProgress::Pending { .. }) => {}
                Ok(armd::driver::TorqueProgress::Done(_)) => panic!("missing J2 TIMEOUT reply must fail even for {expected_opt:?}"),
                Err(e) => {
                    err = Some(e);
                    break;
                }
            }
        }
        let msg = err.expect("must fail on silent J2 TIMEOUT").to_string();
        assert!(msg.contains("joint 2 TIMEOUT readback: no fresh reply"), "got: {msg}");
    }
}

#[test]
fn m2_mit_bus_loss_enrichment_preserves_recovery_instructions() {
    let cfg = ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() };
    let limits = b601_limits();
    let (bus, clock) = FakeBus::new(&limits);
    let drv = make_driver(bus.clone(), &cfg);
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    bus.push_all_fb();
    inner.tick(DT);
    {
        let s = inner.current_admission_stamp();
        inner.check_admission(s).and_then(|a| SupervisorInner::enable_torque(a)).unwrap();
    }
    for _ in 0..10 {
        clock.advance(ms(20));
        bus.push_all_fb();
        inner.tick(DT);
    }
    // Simulate early MIT deadline fence BusLoss latch at ~45 ms (before 250 ms synchronized silence).
    inner.safety.latch(
        armd::FaultKind::BusLoss,
        armd::safety::FaultDetail::formatted(FaultCode::BusLoss, |buf| {
            let _ = core::fmt::Write::write_str(
                buf,
                "MIT_LOST: driver deadline fence: feedback stale on J2; if a motor's command timeout expired, clear_fault recovers by turning torque off on every joint",
            );
        }),
    );
    // Now all 7 motors go silent synchronously; after >600 ms the supervisor enriches the same
    // BusLoss incident with the synchronized power-loss annotation without losing MIT instructions!
    bus.with(|s| for m in &mut s.motors { m.silent = true; m.enabled = false; });
    for _ in 0..35 {
        clock.advance(ms(20));
        inner.tick(DT);
    }
    let fi = inner.safety.fault().expect("BusLoss fault");
    assert_eq!(fi.code, FaultCode::BusLoss);
    let msg = fi.message.as_str();
    assert!(msg.contains("MIT_LOST:"), "MIT recovery instructions must be preserved: {msg}");
    assert!(
        msg.contains("clear_fault recovers by turning torque off"),
        "MIT recovery detail truncated: {msg}"
    );
    assert!(
        msg.contains("Suspected motor power loss or shared CAN interruption."),
        "synchronized BusLoss annotation missing after MIT early latch: {msg}"
    );
}
