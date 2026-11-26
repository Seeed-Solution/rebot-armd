//! Integration tests for clearing carried-over Damiao motor errors latched in motor RAM
//! across an `armd` restart (FakeBus + DamiaoArmDriver, legacy + RT executor + WS/operator auth).

mod common;

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, DamiaoArmDriver, EnableState};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::safety::{ClearFaultError, ClearRefusal};
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, TransportError,
};
use armd::{create_router, ArmConfig, ArmSupervisor, OperatorAuth, SafetyState, SupervisorInner};
use serde_json::json;
use tokio::net::TcpListener;

fn b601_limits() -> Vec<DamiaoLimits> {
    let big = DamiaoLimits::symmetric(12.5, 10.0, 28.0);
    let small = DamiaoLimits::symmetric(12.5, 30.0, 10.0);
    vec![big, big, big, small, small, small, small]
}

#[derive(Clone, Copy)]
struct FakeMotor {
    id: u16,
    q: f32,
    v: f32,
    enabled: bool,
    fault_state: u8,
    persist_fault: bool,
    silent_on_clear: bool,
    limits: DamiaoLimits,
}

impl FakeMotor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let state = if self.fault_state != 0 {
            self.fault_state
        } else if self.enabled {
            1
        } else {
            0
        };
        let p_u = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v_u = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t_u = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let mut data = [0u8; 8];
        data[0] = ((state & 0x0F) << 4) | ((self.id as u8) & 0x0F);
        data[1] = (p_u >> 8) as u8;
        data[2] = (p_u & 0xFF) as u8;
        data[3] = (v_u >> 4) as u8;
        data[4] = (((v_u & 0x0F) as u8) << 4) | (((t_u >> 8) & 0x0F) as u8);
        data[5] = (t_u & 0xFF) as u8;
        data[6] = 30;
        data[7] = 30;
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

struct FakeBusState {
    motors: Vec<FakeMotor>,
    rx: VecDeque<(CanFrame, Option<Instant>)>,
    tx_log: Vec<CanFrame>,
    clock: MockClock,
}

impl FakeBusState {
    fn push_all_feedback(&mut self) {
        let now = self.clock.now();
        for i in 0..self.motors.len() {
            let fb = self.motors[i].feedback_frame();
            self.rx.push_back((fb, Some(now)));
        }
    }
}

#[derive(Clone)]
struct FakeBus {
    state: Arc<Mutex<FakeBusState>>,
}

impl FakeBus {
    fn new(clock: MockClock) -> Self {
        let limits = b601_limits();
        let motors = limits
            .into_iter()
            .enumerate()
            .map(|(i, lim)| FakeMotor {
                id: (i + 1) as u16,
                q: 0.0,
                v: 0.0,
                enabled: false,
                fault_state: 0,
                persist_fault: false,
                silent_on_clear: false,
                limits: lim,
            })
            .collect();
        Self {
            state: Arc::new(Mutex::new(FakeBusState {
                motors,
                rx: VecDeque::new(),
                tx_log: Vec::new(),
                clock,
            })),
        }
    }

    fn with<R>(&self, f: impl FnOnce(&mut FakeBusState) -> R) -> R {
        f(&mut self.state.lock().unwrap())
    }

    fn clear_tx_log(&self) {
        self.with(|s| s.tx_log.clear());
    }

    fn tx_log(&self) -> Vec<CanFrame> {
        self.with(|s| s.tx_log.clone())
    }
}

impl CanTransport for FakeBus {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.state.lock().unwrap();
        let now = s.clock.now();
        s.tx_log.push(*frame);

        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                let fb = s.motors[idx].feedback_frame();
                s.rx.push_back((fb, Some(now)));
            }
            return Ok(());
        }

        if frame.id == 0x7FF && frame.data[2] == damiao::REG_READ {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            if let Some(idx) = s.motors.iter().position(|m| m.id == id) {
                let l = s.motors[idx].limits;
                let val: Option<[u8; 4]> = match rid {
                    damiao::RID_CTRL_MODE => Some(damiao::CTRL_MODE_POS_VEL.to_le_bytes()),
                    damiao::RID_PMAX => Some(l.p_max.to_le_bytes()),
                    damiao::RID_VMAX => Some(l.v_max.to_le_bytes()),
                    damiao::RID_TMAX => Some(l.t_max.to_le_bytes()),
                    _ => None,
                };
                if let Some(v) = val {
                    let [lo, hi] = id.to_le_bytes();
                    let reply = CanFrame::new_standard(
                        0x10 + id,
                        &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                    )
                    .unwrap();
                    s.rx.push_back((reply, Some(now)));
                }
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_CLEAR_FAULTS {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                if !s.motors[idx].persist_fault {
                    s.motors[idx].fault_state = 0;
                }
                if !s.motors[idx].silent_on_clear {
                    let fb = s.motors[idx].feedback_frame();
                    s.rx.push_back((fb, Some(now)));
                }
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_ENABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                s.motors[idx].enabled = true;
                let fb = s.motors[idx].feedback_frame();
                s.rx.push_back((fb, Some(now)));
            }
            return Ok(());
        }

        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_DISABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                s.motors[idx].enabled = false;
                let fb = s.motors[idx].feedback_frame();
                s.rx.push_back((fb, Some(now)));
            }
            return Ok(());
        }

        if let Some(idx) = s
            .motors
            .iter()
            .position(|m| u32::from(m.id) + 0x100 == frame.id || u32::from(m.id) + 0x300 == frame.id)
        {
            let fb = s.motors[idx].feedback_frame();
            s.rx.push_back((fb, Some(now)));
        }

        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_with_rx_time().map(|(f, _)| f)
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        let mut s = self.state.lock().unwrap();
        match s.rx.pop_front() {
            Some((f, t)) => Ok((f, t.map(CanRxTime::Monotonic))),
            None => Err(TransportError::WouldBlock),
        }
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn flush_is_noop(&self) -> bool {
        true
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(Arc::new(self.state.lock().unwrap().clock.clone()))
    }
}

fn make_booted_carryover_supervisor(
    configure: impl FnOnce(&mut FakeBusState),
) -> (SupervisorInner, FakeBus, MockClock) {
    let clock = MockClock::new();
    let bus = FakeBus::new(clock.clone());
    bus.with(configure);
    bus.with(|s| s.push_all_feedback());
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits())
        .unwrap();
    // Initial boot feedback read before attaching driver (reproduces startup feedback with 0xD latched in Damiao RAM).
    driver.read_joint_states().unwrap();
    let mut inner = SupervisorInner::new(ArmConfig::rebot_dm()).with_driver(Box::new(driver));
    inner.lease_enabled = false;
    // One idle tick so observation/feedback mirrors real-arm cold boot (`Idle`, `fault: null`, `enable_state: Unknown`, `motor_errors[0] = 0xD`).
    bus.with(|s| s.push_all_feedback());
    inner.tick(0.02);
    bus.clear_tx_log();
    (inner, bus, clock)
}

#[test]
fn carried_over_0xd_on_boot_clears_and_stays_disabled_legacy_and_rt() {
    for use_rt in [false, true] {
        let (mut inner, bus, _clock) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD; // J1 CAN Communication Loss latched before armd boot
            s.motors[6].q = -2.5; // J7 gripper open — must NOT block J1–J6 rest-pose check
        });

        let obs_before = inner.current_observation();
        assert_eq!(obs_before.safety_state, SafetyState::Idle);
        assert!(obs_before.fault.is_none());
        assert!(!obs_before.is_torqued);
        assert_eq!(obs_before.enable_state, EnableState::Unknown);
        let j1_err = obs_before.motor_errors[0]
            .as_ref()
            .expect("boot observation must show carried-over J1 0xD error");
        assert_eq!(j1_err.code, 0xD);
        assert_eq!(j1_err.name, "CAN Communication Loss");
        assert!(obs_before.motor_errors[1..].iter().all(Option::is_none));

        let cleared = if use_rt {
            inner
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect("RT clear_fault must clear carried-over 0xD on disarmed arm at rest")
        } else {
            inner
                .clear_fault()
                .expect("legacy clear_fault must clear carried-over 0xD on disarmed arm at rest")
        };
        assert_eq!(cleared, SafetyState::Idle);

        let tx = bus.tx_log();
        let clear_frames: Vec<_> = tx
            .iter()
            .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
            .collect();
        assert_eq!(
            clear_frames.len(),
            1,
            "use_rt={use_rt}: CMD_CLEAR_FAULTS must be sent ONLY to affected J1: {tx:?}"
        );
        assert_eq!(clear_frames[0].id, 1, "expected motor ID 1 for J1");
        assert!(
            !tx.iter().any(|f| f.data == damiao::CMD_ENABLE),
            "use_rt={use_rt}: clear_fault must NEVER send CMD_ENABLE: {tx:?}"
        );

        assert!(
            bus.with(|s| s.motors[0].fault_state == 0 && !s.motors[0].enabled),
            "use_rt={use_rt}: J1 error must be cleared and motor must stay disabled"
        );
        assert!(
            bus.with(|s| s.motors.iter().all(|m| !m.enabled)),
            "use_rt={use_rt}: all motors must stay disabled"
        );

        let obs_after = inner.current_observation();
        assert_eq!(obs_after.safety_state, SafetyState::Idle);
        assert!(!obs_after.is_torqued);
        assert_eq!(obs_after.enable_state, EnableState::Disabled);
        assert!(
            obs_after.motor_errors.iter().all(Option::is_none),
            "use_rt={use_rt}: motor_errors must clear immediately after verified clear: {:?}",
            obs_after.motor_errors
        );
    }
}

#[test]
fn carried_over_multiple_affected_and_silent_clear_verifies_via_refresh() {
    let (mut inner, bus, _clock) = make_booted_carryover_supervisor(|s| {
        s.motors[0].fault_state = 0xD;
        s.motors[3].fault_state = 0xA;
        s.motors[3].silent_on_clear = true; // requires 0xCC verification poll
    });

    let cleared = inner
        .clear_fault()
        .expect("clear_fault must clear J1 & J4 and verify J4 via 0xCC");
    assert_eq!(cleared, SafetyState::Idle);

    let tx = bus.tx_log();
    let clear_ids: Vec<u32> = tx
        .iter()
        .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
        .map(|f| f.id)
        .collect();
    assert_eq!(
        clear_ids,
        vec![1, 4],
        "CMD_CLEAR_FAULTS must target ONLY affected motors J1 and J4: {tx:?}"
    );
    assert!(
        tx.iter()
            .any(|f| f.id == 0x7FF && f.data[0] == 4 && f.data[2] == 0xCC),
        "expected read-only 0xCC verification poll for silent motor J4: {tx:?}"
    );
    assert!(!tx.iter().any(|f| f.data == damiao::CMD_ENABLE));
    assert!(inner.current_observation().motor_errors.iter().all(Option::is_none));
    assert_eq!(inner.current_observation().enable_state, EnableState::Disabled);
}

#[test]
fn persisting_motor_error_reports_clear_power_cycle_message() {
    for use_rt in [false, true] {
        let (mut inner, bus, _clock) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
            s.motors[0].persist_fault = true;
        });

        let msg = if use_rt {
            let err = inner
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect_err("persisting error must fail on RT");
            assert_eq!(
                err,
                ClearFaultError::Rejected(ClearRefusal::MotorErrorPersists {
                    joint: 0,
                    code: 0xD,
                    name: Some("CAN Communication Loss"),
                })
            );
            err.to_string()
        } else {
            let err = inner
                .clear_fault()
                .expect_err("persisting error must fail on legacy");
            err.to_string()
        };

        assert!(
            msg.contains("power-cycle the motor") && msg.contains("J1") && msg.contains("0xD"),
            "use_rt={use_rt}: expected clear power-cycle message, got: {msg}"
        );
        let tx = bus.tx_log();
        let clear_frames: Vec<_> = tx
            .iter()
            .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
            .collect();
        assert_eq!(clear_frames.len(), 1);
        assert_eq!(clear_frames[0].id, 1);
        assert!(!tx.iter().any(|f| f.data == damiao::CMD_ENABLE));

        let obs = inner.current_observation();
        assert!(!obs.is_torqued);
        assert_eq!(obs.motor_errors[0].as_ref().map(|e| e.code), Some(0xD));
    }
}

#[test]
fn armed_arm_or_enabled_motor_with_motor_error_is_refused_without_tx() {
    for use_rt in [false, true] {
        // Case A: supervisor is Armed (is_torqued = true) while J1 reports 0xD.
        let (mut inner, bus, _clock) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
        });
        inner.is_torqued = true;
        inner.safety.set_torque(true).unwrap();
        assert_eq!(*inner.safety.state(), SafetyState::Armed);
        bus.clear_tx_log();

        if use_rt {
            let err = inner
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect_err("armed arm must be refused on RT");
            assert_eq!(
                err,
                ClearFaultError::Rejected(ClearRefusal::MotorErrorClearRequiresDisarmed)
            );
        } else {
            let err = inner.clear_fault().expect_err("armed arm must be refused on legacy");
            assert!(
                err.to_string().contains("armed or torque is enabled"),
                "unexpected message: {err}"
            );
        }
        assert!(
            bus.tx_log().is_empty(),
            "use_rt={use_rt}: armed refusal must send 0 CAN frames: {:?}",
            bus.tx_log()
        );

        // Case B: supervisor is Idle (!is_torqued), but J2 hardware feedback reports enabled (state == 1)
        // while J1 reports 0xD (enable_state == Unknown, has_any_joint_enabled == true).
        let (mut inner_hw, bus_hw, _clock_hw) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
            s.motors[1].enabled = true;
        });
        assert_eq!(inner_hw.current_observation().enable_state, EnableState::Unknown);
        assert!(!inner_hw.is_torqued);
        bus_hw.clear_tx_log();

        if use_rt {
            let err = inner_hw
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect_err("hardware-enabled joint must be refused on RT");
            assert_eq!(
                err,
                ClearFaultError::Rejected(ClearRefusal::MotorErrorClearRequiresDisarmed)
            );
        } else {
            let err = inner_hw
                .clear_fault()
                .expect_err("hardware-enabled joint must be refused on legacy");
            assert!(
                err.to_string().contains("armed or torque is enabled"),
                "unexpected message: {err}"
            );
        }
        assert!(
            bus_hw.tx_log().is_empty(),
            "use_rt={use_rt}: hardware-enabled refusal must send 0 CAN frames: {:?}",
            bus_hw.tx_log()
        );
    }
}

#[test]
fn off_rest_pose_and_stale_feedback_refused_without_tx() {
    for use_rt in [false, true] {
        // Off rest pose on J2.
        let (mut inner, bus, _clock) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
            s.motors[1].q = 0.45; // well outside 0.05 rad rest tolerance
        });
        let err_pose = if use_rt {
            let e = inner
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect_err("off-rest pose must refuse on RT");
            assert!(matches!(
                e,
                ClearFaultError::Rejected(ClearRefusal::MotorErrorNotAtRestPose(_))
            ));
            e.to_string()
        } else {
            inner
                .clear_fault()
                .expect_err("off-rest pose must refuse on legacy")
                .to_string()
        };
        assert!(
            err_pose.contains("resting pose"),
            "use_rt={use_rt}: expected resting pose refusal, got: {err_pose}"
        );
        assert!(bus.tx_log().is_empty(), "off-rest refusal must send 0 CAN frames");

        // Stale feedback (> 100 ms).
        let (mut inner_stale, bus_stale, clock_stale) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
        });
        clock_stale.advance(Duration::from_millis(150));
        if use_rt {
            let e = inner_stale
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect_err("stale feedback must refuse on RT");
            assert_eq!(
                e,
                ClearFaultError::Rejected(ClearRefusal::FreshFeedbackRequiredForRestingSupport)
            );
        } else {
            let e = inner_stale
                .clear_fault()
                .expect_err("stale feedback must refuse on legacy");
            assert!(e.to_string().contains("Fresh feedback"), "{e}");
        }
        assert!(
            bus_stale.tx_log().is_empty(),
            "stale feedback refusal must send 0 CAN frames"
        );

        // Queued off-rest J2 frame sitting in CAN RX after an at-rest tick must be drained and
        // caught BEFORE sending CMD_CLEAR_FAULTS (Blocker 1 regression).
        let (mut inner_queued, bus_queued, _clock_queued) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
        });
        bus_queued.with(|s| {
            s.motors[1].q = 0.5;
            let fb = s.motors[1].feedback_frame();
            let now = s.clock.now();
            s.rx.push_back((fb, Some(now)));
        });
        let err_queued = if use_rt {
            let e = inner_queued
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .expect_err("queued off-rest frame must refuse before TX on RT");
            assert!(matches!(
                e,
                ClearFaultError::Rejected(ClearRefusal::MotorErrorNotAtRestPose(_))
            ));
            e.to_string()
        } else {
            inner_queued
                .clear_fault()
                .expect_err("queued off-rest frame must refuse before TX on legacy")
                .to_string()
        };
        assert!(err_queued.contains("resting pose"), "{err_queued}");
        assert!(
            bus_queued.tx_log().is_empty(),
            "queued off-rest refusal must send 0 CAN frames: {:?}",
            bus_queued.tx_log()
        );

        // Stale enabled J7 while J1–J6 stay fresh at rest must be refused (Blocker 2 regression).
        let (mut inner_j7, bus_j7, clock_j7) = make_booted_carryover_supervisor(|s| {
            s.motors[0].fault_state = 0xD;
            s.motors[6].enabled = true;
        });
        clock_j7.advance(Duration::from_millis(300));
        // Push fresh feedback for J1–J6 only (J7 stays stale & enabled).
        bus_j7.with(|s| {
            let now = s.clock.now();
            for i in 0..6 {
                let fb = s.motors[i].feedback_frame();
                s.rx.push_back((fb, Some(now)));
            }
        });
        inner_j7.tick(0.02);
        bus_j7.clear_tx_log();
        let res_j7 = if use_rt {
            inner_j7
                .clear_fault_executor_bounded(Instant::now() + Duration::from_millis(50))
                .map_err(|e| e.to_string())
        } else {
            inner_j7.clear_fault().map_err(|e| e.to_string())
        };
        assert!(
            res_j7.is_err(),
            "use_rt={use_rt}: stale enabled J7 must refuse carryover clear"
        );
        assert!(
            bus_j7.tx_log().is_empty(),
            "use_rt={use_rt}: stale enabled J7 refusal must send 0 CAN frames: {:?}",
            bus_j7.tx_log()
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn ws_clear_fault_enforces_operator_auth_and_clears_carried_over_0xd() {
    let clock = MockClock::new();
    let bus = FakeBus::new(clock.clone());
    bus.with(|s| {
        s.motors[0].fault_state = 0xD;
        s.push_all_feedback();
    });
    let mut driver = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7])
        .with_joint_limits(b601_limits())
        .unwrap();
    driver.read_joint_states().unwrap();

    let auth = OperatorAuth::enforced("246810", 300);
    let supervisor = Arc::new(ArmSupervisor::new(ArmConfig::rebot_dm()).with_driver(Box::new(driver)));
    {
        let mut inner = supervisor.inner().write().await;
        inner.operator_auth_enforced = true;
        bus.with(|s| s.push_all_feedback());
        inner.tick(0.02);
    }
    bus.clear_tx_log();

    let router = create_router(supervisor.clone(), auth.clone());
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let server = tokio::spawn(async move { axum::serve(listener, router).await.unwrap() });

    let (mut ws, gen) = common::controller(addr).await;
    let epoch = supervisor.inner().read().await.safety.epoch_id();

    // 1. Non-operator (program role) is refused with OPERATOR_REQUIRED and sends 0 CAN frames.
    let rej = common::rpc(&mut ws, gen, epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(rej["type"], "command_reject", "{rej}");
    assert_eq!(rej["code"], "OPERATOR_REQUIRED", "{rej}");
    assert!(
        bus.tx_log().is_empty(),
        "non-operator clear_fault must send 0 CAN frames: {:?}",
        bus.tx_log()
    );

    // 2. Unlock operator session via HTTP + authenticate WS connection.
    let client = reqwest::Client::new();
    let unlock_res: serde_json::Value = client
        .post(format!("http://{addr}/api/v1/auth/unlock"))
        .json(&json!({ "pin": "246810" }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let token = unlock_res["token"].as_str().expect("operator token").to_string();
    let auth_ack = common::ws_reply(
        &mut ws,
        json!({ "type": "auth", "seq": 1, "token": token }),
        &["auth_ack"],
    )
    .await;
    assert_eq!(auth_ack["type"], "auth_ack", "{auth_ack}");
    assert_eq!(auth_ack["role"], "operator", "{auth_ack}");

    // 3. Operator calls /api/v1/safety/clear_fault -> clears J1 0xD, stays disarmed & disabled.
    let ack = common::rpc(&mut ws, gen, epoch, "/api/v1/safety/clear_fault", json!({})).await;
    assert_eq!(ack["type"], "command_ack", "{ack}");
    assert_eq!(ack["safety_state"], "Idle", "{ack}");
    assert_eq!(ack["is_torqued"], false, "{ack}");

    let tx = bus.tx_log();
    let clear_frames: Vec<_> = tx
        .iter()
        .filter(|f| f.data == damiao::CMD_CLEAR_FAULTS)
        .collect();
    assert_eq!(clear_frames.len(), 1, "expected 1 CMD_CLEAR_FAULTS to J1: {tx:?}");
    assert_eq!(clear_frames[0].id, 1);
    assert!(!tx.iter().any(|f| f.data == damiao::CMD_ENABLE));

    let obs = supervisor.get_observation().await;
    assert_eq!(obs.safety_state, SafetyState::Idle);
    assert!(!obs.is_torqued);
    assert_eq!(obs.enable_state, EnableState::Disabled);
    assert!(obs.motor_errors.iter().all(Option::is_none));

    server.abort();
}
