//! PR 4 tests: bounded fixed-storage torque back-off diagnostics (requested, transmitted,
//! measured, post-trip torque) published off-thread, plus timed fake-bus real-incident
//! reproduction under BOTH Legacy and RT Executor owners.

mod common;

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{
    gripper_drive_from_profile, ArmDriver, DamiaoArmDriver, TorqueProgress,
};
use armd::model::MotionLimitReason;
use armd::power::StaticOverloadSeverity;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{BootId, Capacities, ClientId, Outcome, Submit, TransitionOp};
use armd::rt_core::ops::{CoreOps, ReleaseFixture, VirtualClock};
use armd::torque_guard::{
    Retreat, TorqueBackoffHistory, TorqueBackoffRecord, TorqueHold, TORQUE_BACKOFF_DIAG_CAPACITY,
    TORQUE_HOLD_BACKOFF_RAD,
};
use armd::transport::{CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, TransportError};
use armd::{ArmConfig, FaultCode, SafetyState, SupervisorInner};

const DT: f64 = 0.02;
const BOOT: BootId = BootId(0xB601_9004);

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
            ctrl_mode: if i == 6 {
                damiao::CTRL_MODE_POS_FORCE
            } else {
                damiao::CTRL_MODE_POS_VEL
            },
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

    /// Decode the latest POS_VEL (0x100 + motor_id) wire position sent for `joint_idx`.
    fn last_pos_vel_q(&self, joint_idx: usize) -> Option<f32> {
        let can_id = 0x100 + (joint_idx as u32 + 1);
        self.with(|s| {
            s.tx_log
                .iter()
                .rev()
                .find(|f| f.id == can_id && f.data.len() >= 4)
                .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
        })
    }

    fn enable_cmd_count(&self) -> usize {
        self.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
                .count()
        })
    }
}

impl CanTransport for FakeBus {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone())
    }
    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
    fn supports_bounded_flush(&self) -> bool {
        true
    }
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

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn b601dm_config() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    // Keep torque_guard enabled with default 0.005 rad back-off; disable joint_tracking lead
    // so wire position equals commanded_positions directly for exact wire-level assertion.
    cfg.joint_tracking = None;
    cfg.validate().expect("valid B601-DM config");
    cfg
}

fn b601_limits() -> [DamiaoLimits; 7] {
    let cfg = ArmConfig::rebot_dm();
    let r = cfg.damiao_ranges.unwrap();
    std::array::from_fn(|i| r[i].limits())
}

fn make_driver(cfg: &ArmConfig, bus: FakeBus) -> DamiaoArmDriver<FakeBus> {
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
fn torque_backoff_default_bounds_and_ring_capacity() {
    assert!((TORQUE_HOLD_BACKOFF_RAD - 0.005).abs() < 1e-7);
    // Logged J4 sample: measured -1.612 rad, torque -9.46 N·m (sign = -1) -> nominal target -1.607 rad.
    let hold = TorqueHold {
        joint: 3,
        hold_q: -1.612,
        sign: -1.0,
        torque_nm: -9.46,
        threshold_nm: 6.0,
        since_s: 1.0,
        anchor_q: -1.612,
        push_in: false,
        retreat: Retreat::Unused,
    };
    let backed = hold.backed_off(-1.612);
    assert!((backed - (-1.607)).abs() < 1e-6, "expected -1.607 rad, got {backed}");
    // Bounded by anchor_q on repeated trips near anchor, and never toward the load from q_meas.
    let repeated_near = hold.backed_off(-1.610);
    assert!((repeated_near - (-1.607)).abs() < 1e-6, "back-off must stay bounded by anchor_q: {repeated_near}");
    let already_relieved = hold.backed_off(-1.55);
    assert!((already_relieved - (-1.55)).abs() < 1e-6, "back-off must never push back toward load: {already_relieved}");

    // Fixed-storage ring keeps at most TORQUE_BACKOFF_DIAG_CAPACITY records without allocating.
    let mut hist = TorqueBackoffHistory::default();
    for seq in 1..=(TORQUE_BACKOFF_DIAG_CAPACITY as u64 + 4) {
        hist.push(TorqueBackoffRecord::new(
            seq,
            3,
            seq as f64,
            -1.0,
            -9.46,
            6.0,
            -1.612,
            -1.612,
            -1.607,
            false,
        ));
    }
    assert_eq!(hist.len(), TORQUE_BACKOFF_DIAG_CAPACITY);
    assert_eq!(hist.trips_total, TORQUE_BACKOFF_DIAG_CAPACITY as u64 + 4);
    let diag = hist.to_diagnostics();
    assert_eq!(diag.recent.len(), TORQUE_BACKOFF_DIAG_CAPACITY);
    assert_eq!(diag.recent.first().unwrap().trip_seq, 5);
    assert_eq!(diag.latest.unwrap().trip_seq, TORQUE_BACKOFF_DIAG_CAPACITY as u64 + 4);
}

#[test]
fn fake_bus_real_incident_reproduction_both_owners() {
    let cfg = b601dm_config();
    let lims = b601_limits();

    // =========================================================================
    // Owner 1: Legacy SupervisorInner
    // =========================================================================
    {
        let (bus, clock) = FakeBus::new(&lims);
        bus.with(|s| {
            s.motors[3].q = -1.612;
        });
        let drv = make_driver(&cfg, bus.clone());
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        inner.lease_enabled = false;

        bus.push_all_fb();
        inner.tick(DT);
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(adm).unwrap();
        for _ in 0..25 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
            if inner.is_torqued && inner.safety.state().is_operational() {
                break;
            }
        }
        assert!(inner.is_torqued && inner.safety.state().is_operational());

        // Step 1: ~5.41 A load (J2=13.7, J3=13.5, J4=-3.4 N·m -> ~5.41 A).
        bus.with(|s| {
            s.motors[1].tau = 13.7;
            s.motors[2].tau = 13.5;
            s.motors[3].tau = -3.4;
        });
        for _ in 0..40 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
        }
        let obs1 = inner.current_observation();
        let p1 = obs1.power.as_ref().unwrap();
        assert!((p1.bus_a - 5.41).abs() < 0.25, "step 1 expected ~5.41 A, got {:.2} A", p1.bus_a);
        assert_eq!(p1.severity, StaticOverloadSeverity::None);

        // Step 2 (isolated): J4 spikes to -9.46 N·m at -1.612 rad alone (below 8.5 A static trip)
        // -> J4 torque_guard first trip & 0.005 rad back-off, wire confirmation, post-trip feedback.
        bus.with(|s| {
            s.motors[1].tau = 0.0;
            s.motors[2].tau = 0.0;
            s.motors[3].tau = -9.46;
            s.motors[3].q = -1.612;
            s.tx_log.clear();
        });
        for _ in 0..8 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
        }
        let rec = *inner
            .torque_guard
            .backoff_history
            .latest()
            .expect("J4 torque back-off diagnostic captured on stage 2");
        assert_eq!(rec.joint, 3);
        assert!((rec.measured_at_trip_q - (-1.612)).abs() < 1e-3, "meas={}", rec.measured_at_trip_q);
        assert!((rec.requested_q - (-1.607)).abs() < 1e-3, "req={}", rec.requested_q);
        let wire_j4 = bus.last_pos_vel_q(3).expect("J4 POS_VEL wire frame");
        let tx_q = rec.transmitted_q.expect("transmitted_q recorded");
        assert!((tx_q - rec.requested_q).abs() < 1e-4, "tx={tx_q} req={}", rec.requested_q);
        assert!((wire_j4 - tx_q).abs() < 1e-4, "wire={wire_j4} tx={tx_q}");
        assert!(rec.post_trip_measured_q.is_some(), "post_trip_measured_q missing");
        assert!(rec.post_trip_torque_nm.is_some(), "post_trip_torque_nm missing");
        assert!(rec.post_trip_samples >= 1);

        // Verify off-thread StateHub snapshot carries the exact same back-off record.
        let _ = inner.publish_at(1.0, 1, false);
        let hub_snap = inner.core.hub().latest().expect("published StateHub snapshot");
        assert_eq!(hub_snap.rt.torque_backoff.latest(), Some(&rec));

        // Step 3 (separate stage): Stationary ~8.56+ A (J4=-9.46 N·m + J2 holding gravity) -> static overload Trip.
        bus.with(|s| {
            s.motors[1].tau = 4.5;
            s.motors[2].tau = 0.0;
            s.motors[3].tau = -9.46;
        });
        for _ in 0..17 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
        }

        // Verify Trip alarm + torque limit coexist independently.
        let obs3 = inner.current_observation();
        let p3 = obs3.power.as_ref().unwrap();
        assert_eq!(p3.severity, StaticOverloadSeverity::Trip);
        assert!(p3.static_restricted);
        assert!(p3.bus_a > 7.8, "expected ~8.56 A display estimate, got {:.2} A", p3.bus_a);
        assert_eq!(obs3.motion_limit.as_ref().map(|m| m.reason), Some(MotionLimitReason::Torque));

        // Hold for 1.7 s (85 ticks): verify EVERY transmitted J4 POS_VEL wire frame holds the
        // exact backed-off pose (zero transient retreat) and zero autonomous enable frames.
        let cmd_before_wait = inner.commanded_positions.clone();
        bus.with(|s| s.tx_log.clear());
        for _ in 0..85 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
        }
        assert_eq!(inner.commanded_positions, cmd_before_wait, "no autonomous retreat");
        let j4_wire_frames: Vec<f32> = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x104 && f.dlc == 8)
                .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
                .collect()
        });
        assert!(!j4_wire_frames.is_empty(), "expected J4 wire hold frames during 1.7 s hold");
        for &wq in &j4_wire_frames {
            assert!(
                (wq - cmd_before_wait[3]).abs() < 1e-4,
                "transient J4 wire retreat detected during 1.7 s hold: wire={wq} expected={}",
                cmd_before_wait[3]
            );
        }
        assert_eq!(bus.enable_cmd_count(), 0, "no autonomous enable");

        // Step 4: Synchronized silence 1.7 s later; isolate J1-J6 TIMEOUT=0 first (J7 CTRL_MODE stays valid=3).
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.silent = true;
                m.enabled = false;
                if i < 6 {
                    m.timeout_counts = 0;
                }
            }
            s.tx_log.clear();
        });
        // After ~280 ms (>250 ms J7 fault bound, <600 ms BusLoss), J7 faults first (`J7FeedbackStale`): alarm MUST persist!
        for _ in 0..14 {
            clock.advance(ms(20));
            inner.tick(DT);
        }
        let fi_j7 = inner.safety.fault().expect("J7 fault first");
        assert_eq!(fi_j7.code, FaultCode::J7FeedbackStale);
        assert_eq!(
            inner.current_observation().power.as_ref().unwrap().severity,
            StaticOverloadSeverity::Trip,
            "static overload Trip alarm must persist through J7 fault (legacy)"
        );
        assert_eq!(bus.enable_cmd_count(), 0);

        // Continue silence to >600 ms -> BusLoss latches with synchronized annotation; alarm STILL persists!
        for _ in 0..22 {
            clock.advance(ms(20));
            inner.tick(DT);
        }
        let fi_bl = inner.safety.fault().expect("BusLoss fault");
        assert_eq!(fi_bl.code, FaultCode::BusLoss);
        assert!(
            fi_bl.message.as_str().contains("Suspected motor power loss or shared CAN interruption."),
            "missing synchronized annotation: {}",
            fi_bl.message
        );
        assert_eq!(
            inner.current_observation().power.as_ref().unwrap().severity,
            StaticOverloadSeverity::Trip,
            "static overload Trip alarm must persist through BusLoss (legacy)"
        );

        // Motors come back with isolated J1-J6 TIMEOUT=0 (J7 CTRL_MODE=3 valid).
        bus.with(|s| {
            for m in &mut s.motors {
                m.silent = false;
            }
        });
        for _ in 0..4 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
        }
        inner.clear_fault().expect("clear_fault after fresh feedback");
        bus.with(|s| s.tx_log.clear());
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        let en_err = SupervisorInner::enable_torque(adm).unwrap_err().to_string();
        assert!(
            en_err.contains("joint 1 TIMEOUT readback mismatch: expected 2000 counts, observed 0 counts"),
            "expected isolated J1 TIMEOUT mismatch on legacy, got: {en_err}"
        );
        assert!(!inner.safety.state().is_operational(), "enable must be refused on J1-J6 TIMEOUT=0");
        assert_eq!(bus.enable_cmd_count(), 0, "zero CMD_ENABLE frames may leave wire on TIMEOUT mismatch");
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));

        // Now isolate J7 CTRL_MODE=2 (restore J1-J6 TIMEOUT=2000) -> CTRL_MODE mismatch refusal!
        inner.clear_fault().expect("clear EnableFailed before isolated J7 CTRL_MODE test");
        bus.with(|s| {
            for i in 0..6 {
                s.motors[i].timeout_counts = 2000;
            }
            s.motors[6].ctrl_mode = damiao::CTRL_MODE_POS_VEL;
            s.tx_log.clear();
        });
        let stamp2 = inner.current_admission_stamp();
        let adm2 = inner.check_admission(stamp2).unwrap();
        let cm_err = SupervisorInner::enable_torque(adm2).unwrap_err().to_string();
        assert!(
            cm_err.contains("joint 7 CTRL_MODE readback: reads 2, expected 4"),
            "expected isolated J7 CTRL_MODE mismatch on legacy, got: {cm_err}"
        );
        assert_eq!(bus.enable_cmd_count(), 0, "zero CMD_ENABLE frames on J7 CTRL_MODE mismatch");
    }

    // =========================================================================
    // Owner 2: RT Executor (`ReleaseFixture` + `Broker`)
    // =========================================================================
    {
        let (bus, clock) = FakeBus::new(&lims);
        bus.with(|s| {
            s.motors[3].q = -1.612;
        });
        let drv = make_driver(&cfg, bus.clone());
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
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
        let mut enabled_ok = false;
        for _ in 0..30 {
            bus.push_all_fb();
            let rep = fx.step();
            if matches!(rep.transition_ended, Some((_, Outcome::Completed))) {
                enabled_ok = true;
                break;
            }
        }
        assert!(enabled_ok && fx.core.is_torqued);

        // Step 1: 5.41 A.
        bus.with(|s| {
            s.motors[1].tau = 13.7;
            s.motors[2].tau = 13.5;
            s.motors[3].tau = -3.4;
        });
        for _ in 0..40 {
            bus.push_all_fb();
            let _ = fx.step();
        }
        let p1 = fx.core.current_observation().power.unwrap();
        assert!((p1.bus_a - 5.41).abs() < 0.25, "RT step 1 got {:.2} A", p1.bus_a);

        // Step 2 (isolated): J4 first trip (-9.46 N·m at -1.612 rad) -> back-off & post-trip feedback.
        bus.with(|s| {
            s.motors[1].tau = 0.0;
            s.motors[2].tau = 0.0;
            s.motors[3].tau = -9.46;
            s.motors[3].q = -1.612;
            s.tx_log.clear();
        });
        for _ in 0..8 {
            bus.push_all_fb();
            let _ = fx.step();
        }
        let rec = *fx
            .core
            .torque_guard
            .backoff_history
            .latest()
            .expect("RT J4 back-off diagnostic");
        assert_eq!(rec.joint, 3);
        assert!((rec.requested_q - (-1.607)).abs() < 1e-3);
        let wire_j4 = bus.last_pos_vel_q(3).expect("RT J4 POS_VEL wire frame");
        let tx_q = rec.transmitted_q.expect("RT transmitted_q");
        assert!((wire_j4 - tx_q).abs() < 1e-4);
        assert!(rec.post_trip_measured_q.is_some());
        assert!(rec.post_trip_torque_nm.is_some());
        assert!(rec.post_trip_samples >= 1);
        let hub_snap = fx.core.core.hub().latest().expect("RT published StateHub snapshot");
        assert_eq!(hub_snap.rt.torque_backoff.latest(), Some(&rec));

        // Step 3 (separate stage): Stationary ~8.56+ A on RT -> Trip + bus_a > 7.8 A.
        bus.with(|s| {
            s.motors[1].tau = 4.5;
            s.motors[2].tau = 0.0;
            s.motors[3].tau = -9.46;
        });
        for _ in 0..17 {
            bus.push_all_fb();
            let _ = fx.step();
        }
        let p3_rt = fx.core.current_observation().power.unwrap();
        assert_eq!(p3_rt.severity, StaticOverloadSeverity::Trip);
        assert!(p3_rt.static_restricted);
        assert!(p3_rt.bus_a > 7.8, "RT stage 3 expected >7.8 A, got {:.2} A", p3_rt.bus_a);

        // Hold 1.7 s: assert EVERY J4 wire frame holds exact backed-off pose (zero transient retreat).
        let cmd_before_wait = fx.core.commanded_positions.clone();
        bus.with(|s| s.tx_log.clear());
        for _ in 0..85 {
            bus.push_all_fb();
            let _ = fx.step();
        }
        assert_eq!(fx.core.commanded_positions, cmd_before_wait);
        let rt_j4_wire: Vec<f32> = bus.with(|s| {
            s.tx_log
                .iter()
                .filter(|f| f.id == 0x104 && f.dlc == 8)
                .map(|f| f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]))
                .collect()
        });
        assert!(!rt_j4_wire.is_empty());
        for &wq in &rt_j4_wire {
            assert!(
                (wq - cmd_before_wait[3]).abs() < 1e-4,
                "RT transient J4 wire retreat: wire={wq} expected={}",
                cmd_before_wait[3]
            );
        }
        assert_eq!(bus.enable_cmd_count(), 0);

        // Step 4: Synchronized silence + isolated J1-J6 TIMEOUT=0 (J7 CTRL_MODE=3 valid).
        bus.with(|s| {
            for (i, m) in s.motors.iter_mut().enumerate() {
                m.silent = true;
                m.enabled = false;
                if i < 6 {
                    m.timeout_counts = 0;
                }
            }
            s.tx_log.clear();
        });
        for _ in 0..14 {
            let _ = fx.step();
        }
        assert_eq!(fx.core.safety.fault().unwrap().code, FaultCode::J7FeedbackStale);
        assert_eq!(
            fx.core.current_observation().power.unwrap().severity,
            StaticOverloadSeverity::Trip,
            "RT static overload Trip alarm must persist through J7 fault"
        );
        for _ in 0..22 {
            let _ = fx.step();
        }
        let fi = fx.core.safety.fault().expect("RT BusLoss");
        assert_eq!(fi.code, FaultCode::BusLoss);
        assert!(fi.message.as_str().contains("Suspected motor power loss or shared CAN interruption."));
        assert_eq!(
            fx.core.current_observation().power.unwrap().severity,
            StaticOverloadSeverity::Trip,
            "RT static overload Trip alarm must persist through BusLoss"
        );
        assert_eq!(bus.enable_cmd_count(), 0);

        // Recover with isolated J1-J6 TIMEOUT=0 -> clear fault -> RT enable MUST fail with exact J1 TIMEOUT mismatch.
        bus.with(|s| for m in &mut s.motors { m.silent = false; });
        for _ in 0..4 {
            bus.push_all_fb();
            let _ = fx.step();
        }
        fx.core.clear_fault().expect("RT clear_fault");
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
        assert!(matches!(rearm_out, Some(Outcome::Failed(_))), "expected Failed, got {rearm_out:?}");
        let rt_msg = fx.core.safety.fault().map(|f| f.message.as_str().to_string()).unwrap_or_default();
        assert!(
            rt_msg.contains("joint 1 TIMEOUT readback mismatch: expected 2000 counts, observed 0 counts"),
            "RT must report exact J1 TIMEOUT mismatch, got: {rt_msg}"
        );
        assert!(!fx.core.safety.state().is_operational());
        assert_eq!(bus.enable_cmd_count(), 0);
        assert!(bus.with(|s| s.motors.iter().all(|m| !m.enabled)));
    }
}

#[test]
fn extra_cases_single_axis_silence_stale_replies_and_valid_j7_zero() {
    let cfg = b601dm_config();
    let lims = b601_limits();

    // 1. Stale pre-request register reply ignored + valid J7 TIMEOUT=0 enables cleanly.
    {
        let (bus, clock) = FakeBus::new(&lims);
        // Inject stale bogus reply (TIMEOUT=9999 for J1) BEFORE enable starts.
        bus.push_reg_reply(1, damiao::RID_TIMEOUT, 9999);
        let mut drv = make_driver(&cfg, bus.clone());
        bus.push_all_fb();
        let _ = drv.read_joint_states();
        // Valid kit registers (J1-J6=2000, J7=0) must enable cleanly after fresh causal replies.
        let deadline = clock.now() + Duration::from_secs(2);
        assert!(drv.begin_torque(true).is_ok());
        let mut confirmed = false;
        for _ in 0..30 {
            clock.advance(ms(20));
            bus.push_all_fb();
            let _ = drv.read_joint_states();
            if matches!(drv.poll_torque(deadline).unwrap(), TorqueProgress::Done(_)) {
                confirmed = true;
                break;
            }
        }
        assert!(confirmed, "valid J1-J6=2000 + J7=0 must enable cleanly even after stale pre-request reply");
    }

    // 2. Single-axis silence does NOT emit synchronized power-loss annotation.
    {
        let (bus, clock) = FakeBus::new(&lims);
        let drv = make_driver(&cfg, bus.clone());
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        bus.push_all_fb();
        inner.tick(DT);
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(adm).unwrap();
        for _ in 0..25 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
            if inner.is_torqued && inner.safety.state().is_operational() {
                break;
            }
        }
        // Only J3 goes silent while all other 6 motors keep reporting.
        bus.with(|s| s.motors[2].silent = true);
        for _ in 0..35 {
            clock.advance(ms(20));
            bus.push_all_fb();
            inner.tick(DT);
        }
        let fi = inner.safety.fault().expect("BusLoss on single-axis silence");
        assert_eq!(fi.code, FaultCode::BusLoss);
        assert!(
            !fi.message.as_str().contains("Suspected motor power loss"),
            "single-axis silence must not emit synchronized power-loss annotation: {}",
            fi.message
        );
        assert!(matches!(inner.safety.state(), SafetyState::Fault(_)));
    }
}

#[tokio::test]
async fn diagnostics_routes_publish_torque_backoff_off_thread() {
    let (addr, _sup) = common::spawn_server(ArmConfig::rebot_dm()).await;
    let diag: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let tb = &diag["torque_backoff"];
    assert!((tb["backoff_rad"].as_f64().unwrap() - 0.005).abs() < 1e-6);
    assert_eq!(tb["trips_total"], 0);
    assert!(tb["recent"].is_array());

    let pwr: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/power"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert!((pwr["torque_backoff"]["backoff_rad"].as_f64().unwrap() - 0.005).abs() < 1e-6);
}

#[test]
fn m11_m12_tx_seq_and_post_fb_seq_gating() {
    let mut hist = TorqueBackoffHistory::default();
    let rec = TorqueBackoffRecord::new(1, 3, 1.0, -1.0, -9.46, 6.0, -1.612, -1.612, -1.607, false);
    hist.push(rec);
    // M12: cached feedback BEFORE transmitted_q is recorded must NOT increment post_trip_samples.
    hist.record_post_trip_feedback(3, 10, 1.02, -1.610, -8.5);
    assert_eq!(hist.latest().unwrap().post_trip_samples, 0);

    // M11: wire_tx_seq == 0 (no new wire commit since trip at seq 0) must NOT set transmitted_q.
    hist.record_transmitted(3, 0, -1.607, 10);
    assert!(hist.latest().unwrap().transmitted_q.is_none());

    // Valid new wire commit at seq 1 -> sets transmitted_q and baselines fb_seq = 10.
    hist.record_transmitted(3, 1, -1.607, 10);
    assert!((hist.latest().unwrap().transmitted_q.unwrap() - (-1.607)).abs() < 1e-6);

    // M12: same cached fb_seq=10 must NOT count as a post-trip sample!
    hist.record_post_trip_feedback(3, 10, 1.04, -1.609, -8.1);
    assert_eq!(hist.latest().unwrap().post_trip_samples, 0);

    // New fb_seq=11 counts once; repeating fb_seq=11 does NOT double-count.
    hist.record_post_trip_feedback(3, 11, 1.06, -1.608, -7.8);
    hist.record_post_trip_feedback(3, 11, 1.08, -1.608, -7.8);
    assert_eq!(hist.latest().unwrap().post_trip_samples, 1);

    // Disarm closes capture so a later rearm cannot append into the old record.
    hist.close_open_capture();
    hist.record_post_trip_feedback(3, 12, 1.10, -1.607, -7.5);
    assert_eq!(hist.latest().unwrap().post_trip_samples, 1);
}

#[tokio::test]
async fn m13_rt_executor_diagnostics_power_reads_live_state_hub() {
    let mut cfg = b601dm_config();
    cfg.lease_timeout_ms = 600_000;
    let lims = b601_limits();
    let (bus, _clock) = FakeBus::new(&lims);
    bus.with(|s| {
        s.motors[3].q = -1.612;
        s.motors[3].tau = 0.0;
    });
    let drv = make_driver(&cfg, bus.clone());

    let sup = std::sync::Arc::new(armd::ArmSupervisor::new(cfg.clone()));
    sup.inner().write().await.lease_enabled = false;
    sup.set_driver(Box::new(drv)).await;
    bus.push_all_fb();
    let _ = sup.tick(DT).await;
    // Arm with zero load BEFORE handoff so the detached Legacy shell has trips_total == 0.
    let stamp = sup.capture_admission_stamp();
    sup.enable_torque(stamp).await.expect("initial enable before RT handoff");
    assert_eq!(sup.inner().read().await.torque_guard.backoff_history.trips_total, 0);

    // Hand off ownership to the synchronous RT executor; detached Legacy shell keeps trips_total == 0.
    let exec_cfg = armd::rt_core::executor::ExecutorConfig::for_tests(Duration::from_millis(20));
    let handle = sup.start_executor(exec_cfg).await.expect("start RT executor");
    let app = armd::api::create_router(sup.clone(), armd::auth::OperatorAuth::disabled());
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let server = tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });

    // Now inject J4 torque spike exclusively under RT executor ownership!
    bus.with(|s| {
        s.motors[3].tau = -9.46;
    });
    let feeder = tokio::spawn({
        let bus = bus.clone();
        async move {
            for _ in 0..60 {
                bus.push_all_fb();
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }
    });
    tokio::time::sleep(Duration::from_millis(350)).await;

    let pwr: serde_json::Value = reqwest::get(format!("http://{addr}/api/v1/diagnostics/power"))
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(pwr["enabled"], true, "RT /diagnostics/power: {pwr}");
    let trips = pwr["torque_backoff"]["trips_total"].as_u64().unwrap_or(0);
    assert!(
        trips >= 1,
        "M13: RT /diagnostics/power must read live StateHub torque_backoff (not detached Legacy shell), got: {pwr}"
    );
    assert_eq!(pwr["torque_backoff"]["latest"]["joint"], 3);

    feeder.abort();
    drop(handle);
    server.abort();
}
