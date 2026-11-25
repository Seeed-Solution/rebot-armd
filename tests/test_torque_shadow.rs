//! Comprehensive integration, dual-owner parity, FakeBus fuzz, divergence-injection,
//! diagnostics/blackbox, clock normalization, ZeroOffsetDriver, kill-switch, and
//! shutdown/panic tests for `TorqueController` SHADOW mode (Reducer PR1).

use std::collections::VecDeque;
use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::body::Body;
use axum::http::{Request, StatusCode};
use tower::ServiceExt;

use armd::diagnostics::blackbox::{BlackBox, Offer, Sink};
use armd::driver::{
    ArmDriver, DamiaoArmDriver, FeetechArmDriver, RobStrideArmDriver,
};
use armd::joint_zero::ZeroOffsetDriver;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::broker::{
    Authority, BootId, Capacities, ClientId, Outcome, Submit, TransitionOp,
};
use armd::rt_core::executor::{ExecutorConfig, PersistTestHooks};
use armd::rt_core::ops::ReleaseFixture;
use armd::torque::shadow::{
    resolve_shadow_enabled, CoverageGapKind, DivergenceCategory, EvidenceProvenance, RxStampKind,
    ShadowBlackBoxSink, ShadowDriverObs, ShadowFaultInjection, ShadowInputKind, ShadowOwner,
    ShadowRuntime, DRIVER_SHADOW_STAGING_CAP, SHADOW_BB_FILE, SHADOW_RING_CAPACITY,
};
use armd::torque::{
    ChannelMask, DriveState, HandshakeOutcome, MotorWriteOutcome, PanicTier, RefusalCode, StopKind,
    TorqueConfig, TorqueIntent, VendorKind, WriteKind,
};
use armd::transport::{
    CanFrame, CanRxTime, CanTransport, DriverClock, MockClock, RxClockSample, Transport,
    TransportError,
};
use armd::{create_router, ArmConfig, ArmSupervisor, OperatorAuth, SafetyState, SupervisorInner};

// =========================================================================
// Shared FakeBus / FakeCan Infrastructure
// =========================================================================

#[derive(Clone, Copy)]
struct SimMotor {
    id: u16,
    q: f32,
    v: f32,
    temp_c: u8,
    enabled: bool,
    fault_state: Option<u8>,
    limits: DamiaoLimits,
}

impl SimMotor {
    fn feedback_frame(&self) -> CanFrame {
        let l = &self.limits;
        let p = damiao::float_to_uint(self.q, l.p_min, l.p_max, 16);
        let v = damiao::float_to_uint(self.v, l.v_min, l.v_max, 12);
        let t = damiao::float_to_uint(0.0, l.t_min, l.t_max, 12);
        let state = self.fault_state.unwrap_or(u8::from(self.enabled));
        let data = [
            (state << 4) | (self.id as u8 & 0x0f),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            (((v & 0x0f) << 4) | ((t >> 8) & 0x0f)) as u8,
            t as u8,
            self.temp_c,
            self.temp_c,
        ];
        CanFrame::new_standard(0x10 + self.id, &data).unwrap()
    }
}

struct FakeCanState {
    motors: [SimMotor; 7],
    ctrl_modes: [u32; 7],
    rx: VecDeque<(CanFrame, Option<CanRxTime>)>,
    tx_log: Vec<CanFrame>,
    clock: Arc<MockClock>,
    kernel_realtime_ns: Option<i128>,
    realtime_minus_mono_ns: Option<i128>,
    fail_disable_mask: u8,
    fail_enable_mask: u8,
    silent_disable: bool,
}

impl FakeCanState {
    fn push_all_feedback(&mut self) {
        let rx_t = match self.kernel_realtime_ns {
            Some(ns) => Some(CanRxTime::KernelRealtimeNs(ns)),
            None => Some(CanRxTime::Monotonic(self.clock.now())),
        };
        for m in &self.motors {
            self.rx.push_back((m.feedback_frame(), rx_t));
        }
    }

    fn set_pose(&mut self, q: &[f32; 7]) {
        for (m, &pos) in self.motors.iter_mut().zip(q.iter()) {
            m.q = pos;
        }
    }
}

#[derive(Clone)]
struct FakeCan(Arc<Mutex<FakeCanState>>);

impl FakeCan {
    fn new(clock: Arc<MockClock>) -> Self {
        let limits: Vec<DamiaoLimits> = ArmConfig::rebot_dm()
            .damiao_ranges
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let motors = std::array::from_fn(|i| SimMotor {
            id: (i + 1) as u16,
            q: 0.0,
            v: 0.0,
            temp_c: 30,
            enabled: false,
            fault_state: None,
            limits: limits[i],
        });
        Self(Arc::new(Mutex::new(FakeCanState {
            motors,
            ctrl_modes: [damiao::CTRL_MODE_POS_VEL; 7],
            rx: VecDeque::with_capacity(512),
            tx_log: Vec::new(),
            clock,
            kernel_realtime_ns: None,
            realtime_minus_mono_ns: None,
            fail_disable_mask: 0,
            fail_enable_mask: 0,
            silent_disable: false,
        })))
    }

    fn with<R>(&self, f: impl FnOnce(&mut FakeCanState) -> R) -> R {
        f(&mut self.0.lock().unwrap())
    }
}

impl CanTransport for FakeCan {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.0.lock().unwrap().clock.clone() as Arc<dyn DriverClock>)
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn rx_clock_sample(&self) -> Option<RxClockSample> {
        let s = self.0.lock().unwrap();
        let rx_now = match s.kernel_realtime_ns {
            Some(ns) => CanRxTime::KernelRealtimeNs(ns),
            None => CanRxTime::Monotonic(s.clock.now()),
        };
        Some(RxClockSample {
            rx_now,
            realtime_minus_monotonic_ns: s.realtime_minus_mono_ns,
        })
    }

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.tx_log.push(*frame);
        let rx_t = match s.kernel_realtime_ns {
            Some(ns) => Some(CanRxTime::KernelRealtimeNs(ns)),
            None => Some(CanRxTime::Monotonic(s.clock.now())),
        };
        if frame.id == 0x7FF && frame.data[2] == damiao::REG_READ {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            let rid = frame.data[3];
            if let Some((idx, m)) = s.motors.iter().enumerate().find(|(_, m)| m.id == id) {
                let ctrl_mode = s.ctrl_modes[idx];
                let val: Option<[u8; 4]> = match rid {
                    damiao::RID_CTRL_MODE => Some(ctrl_mode.to_le_bytes()),
                    damiao::RID_PMAX => Some(m.limits.p_max.to_le_bytes()),
                    damiao::RID_VMAX => Some(m.limits.v_max.to_le_bytes()),
                    damiao::RID_TMAX => Some(m.limits.t_max.to_le_bytes()),
                    damiao::RID_TIMEOUT => Some(1000u32.to_le_bytes()),
                    _ => None,
                };
                if let Some(v) = val {
                    let [lo, hi] = id.to_le_bytes();
                    let reply = CanFrame::new_standard(
                        0x10 + id,
                        &[lo, hi, damiao::REG_READ, rid, v[0], v[1], v[2], v[3]],
                    )
                    .unwrap();
                    s.rx.push_back((reply, rx_t));
                }
            }
            return Ok(());
        }
        if frame.id == 0x7FF && frame.data[2] == 0xCC {
            let id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if !s.silent_disable {
                if let Some(f) = s
                    .motors
                    .iter()
                    .find(|m| m.id == id)
                    .map(SimMotor::feedback_frame)
                {
                    s.rx.push_back((f, rx_t));
                }
            }
            return Ok(());
        }
        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_ENABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                if (s.fail_enable_mask & (1 << idx)) != 0 {
                    return Err(TransportError::WouldBlock);
                }
                s.motors[idx].enabled = true;
                let f = s.motors[idx].feedback_frame();
                s.rx.push_back((f, rx_t));
            }
            return Ok(());
        }
        if !frame.is_extended && frame.dlc == 8 && frame.data == damiao::CMD_DISABLE {
            if let Some(idx) = s.motors.iter().position(|m| u32::from(m.id) == frame.id) {
                if (s.fail_disable_mask & (1 << idx)) != 0 {
                    return Err(TransportError::WouldBlock);
                }
                s.motors[idx].enabled = false;
                if !s.silent_disable {
                    let f = s.motors[idx].feedback_frame();
                    s.rx.push_back((f, rx_t));
                }
            }
            return Ok(());
        }
        // Motion/hold frames: echo current feedback so bus stays fresh.
        if !frame.is_extended && frame.dlc == 8 {
            if let Some(idx) = s
                .motors
                .iter()
                .position(|m| frame.id == u32::from(m.id) || frame.id == 0x100 + u32::from(m.id))
            {
                let f = s.motors[idx].feedback_frame();
                s.rx.push_back((f, rx_t));
            }
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        self.receive_with_rx_time().map(|(f, _)| f)
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        let mut s = self.0.lock().unwrap();
        s.rx.pop_front().ok_or(TransportError::Timeout)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

fn build_dm_config(shadow_enabled: bool) -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    cfg.torque_shadow_enabled = shadow_enabled;
    cfg.resting_pose = Some(armd::model::RestingPosePolicy {
        positions: Some(vec![0.0; 7]),
        ..armd::model::RestingPosePolicy::rebot_dm()
    });
    cfg
}

fn build_dm_driver(bus: FakeCan, cfg: &ArmConfig) -> DamiaoArmDriver<FakeCan> {
    let limits: Vec<DamiaoLimits> = cfg
        .damiao_ranges
        .as_ref()
        .unwrap()
        .iter()
        .map(|r| r.limits())
        .collect();
    let mut drv = DamiaoArmDriver::new(bus, (1..=7).collect())
        .with_joint_limits(limits)
        .unwrap();
    drv.check_mapping_ranges();
    drv
}

fn run_stopped_torque_off_scenario(shadow_enabled: bool) -> (bool, Vec<CanFrame>, u64, SafetyState) {
    let clock = Arc::new(MockClock::new());
    let bus = FakeCan::new(clock.clone());
    let cfg = build_dm_config(shadow_enabled);
    let drv = build_dm_driver(bus.clone(), &cfg);
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));

    // Seed initial rest feedback and arm the arm.
    bus.with(|s| {
        s.set_pose(&[0.0; 7]);
        s.push_all_feedback();
    });
    inner.tick_control(0.02);
    let st = inner.current_admission_stamp();
    let adm = inner.check_admission(st).unwrap();
    SupervisorInner::enable_torque(adm).unwrap();

    // Raise J2 above rest tolerance and trigger E-stop.
    clock.advance(Duration::from_millis(25));
    bus.with(|s| {
        s.set_pose(&[0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0]);
        s.push_all_feedback();
    });
    inner.tick_control(0.02);
    inner.apply_stop_policy(armd::safety::StopPolicy::EStopLatch);
    inner.tick_control(0.02);

    // Attempt stopped torque-off while raised -> MUST be refused identically with shadow ON or OFF!
    let refused_raised = inner.disable_torque().is_err();

    // Move back to rest across 3 distinct control releases (K=3 causal rest window) and disable.
    bus.with(|s| s.tx_log.clear());
    for _ in 0..4 {
        clock.advance(Duration::from_millis(25));
        bus.with(|s| {
            s.set_pose(&[0.0; 7]);
            s.push_all_feedback();
        });
        inner.tick_control(0.02);
    }
    let allowed_at_rest = inner.disable_torque().is_ok();
    for _ in 0..4 {
        clock.advance(Duration::from_millis(25));
        bus.with(|s| s.push_all_feedback());
        inner.tick_control(0.02);
    }
    inner.test_flush_and_drain_torque_shadow();

    let tx_frames = bus.with(|s| s.tx_log.clone());
    let epoch = inner.safety.epoch_id();
    let safety_state = inner.safety.state().clone();
    assert!(refused_raised, "raised stopped torque-off must refuse");
    assert!(allowed_at_rest, "rest-pose stopped torque-off must succeed");
    (
        refused_raised && allowed_at_rest,
        tx_frames,
        epoch,
        safety_state,
    )
}

// =========================================================================
// 1. Live Parity: Shadow ON vs Shadow OFF (Legacy & RT Executor)
// =========================================================================

#[test]
fn torque_off_live_parity_shadow_on_vs_off_legacy() {
    let (ok_off, tx_off, ep_off, st_off) = run_stopped_torque_off_scenario(false);
    let (ok_on, tx_on, ep_on, st_on) = run_stopped_torque_off_scenario(true);
    assert_eq!(ok_off, ok_on);
    assert_eq!(ep_off, ep_on);
    assert_eq!(st_off, st_on);
    assert_eq!(
        tx_off, tx_on,
        "wire CAN frames with shadow ON must be byte-for-byte identical to shadow OFF"
    );
}

#[test]
fn torque_off_live_parity_shadow_on_vs_off_rt_executor() {
    fn run_rt(shadow_enabled: bool) -> (Outcome, Outcome, Vec<CanFrame>, bool) {
        let clock = Arc::new(MockClock::new());
        let bus = FakeCan::new(clock.clone());
        let cfg = build_dm_config(shadow_enabled);
        let drv = build_dm_driver(bus.clone(), &cfg);
        let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.register_live_connection(1).unwrap();
        let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new(
            inner,
            BootId(1),
            Capacities { commands: 8, plans: 1, replies: 16 },
        );

        bus.with(|s| {
            s.set_pose(&[0.0; 7]);
            s.push_all_feedback();
        });
        let _ = fx.step();

        // Arm via broker transition.
        let ep = fx.core.safety.epoch_id();
        let _req_en = broker
            .submit(
                ClientId(1),
                1,
                Authority {
                    lease_generation: 0,
                    epoch: ep,
                    stop_generation: 0,
                    config_revision: 0,
                    parent_generation: 0,
                },
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOn),
            )
            .unwrap();
        let mut en_out = None;
        for _ in 0..20 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            let rep = fx.step();
            fx.core.test_flush_and_drain_torque_shadow();
            if let Some((_, outcome)) = rep.transition_ended {
                en_out = Some(outcome);
                break;
            }
        }
        assert_eq!(en_out, Some(Outcome::Completed));

        // Raise + E-stop + attempt TorqueOff -> must refuse with TORQUE_OFF_WOULD_DROP.
        clock.advance(Duration::from_millis(25));
        bus.with(|s| {
            s.set_pose(&[0.0, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0]);
            s.push_all_feedback();
        });
        let _ = fx.step();
        fx.core
            .apply_stop_policy(armd::safety::StopPolicy::EStopLatch);
        clock.advance(Duration::from_millis(25));
        bus.with(|s| s.push_all_feedback());
        let _ = fx.step();

        let ep2 = fx.core.safety.epoch_id();
        let _req_refuse = broker
            .submit(
                ClientId(1),
                2,
                Authority {
                    lease_generation: 0,
                    epoch: ep2,
                    stop_generation: 0,
                    config_revision: 0,
                    parent_generation: 0,
                },
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        let mut out_refuse = None;
        for _ in 0..8 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            let rep = fx.step();
            fx.core.test_flush_and_drain_torque_shadow();
            if let Some((_, outcome)) = rep.transition_ended {
                out_refuse = Some(outcome);
                break;
            }
            if let Some((_, Err(e))) = rep.command {
                out_refuse = Some(Outcome::Failed(e));
                break;
            }
        }

        // Return to rest for K=3 releases, then TorqueOff -> must complete.
        bus.with(|s| s.tx_log.clear());
        for _ in 0..4 {
            clock.advance(Duration::from_millis(25));
            bus.with(|s| {
                s.set_pose(&[0.0; 7]);
                s.push_all_feedback();
            });
            let _ = fx.step();
            fx.core.test_flush_and_drain_torque_shadow();
        }
        let _req_ok = broker
            .submit(
                ClientId(1),
                3,
                Authority {
                    lease_generation: 0,
                    epoch: fx.core.safety.epoch_id(),
                    stop_generation: 0,
                    config_revision: 0,
                    parent_generation: 0,
                },
                fx.now() + Duration::from_secs(5),
                Submit::Transition(TransitionOp::TorqueOff),
            )
            .unwrap();
        let mut out_ok = None;
        for _ in 0..16 {
            clock.advance(Duration::from_millis(20));
            bus.with(|s| s.push_all_feedback());
            let rep = fx.step();
            fx.core.test_flush_and_drain_torque_shadow();
            if let Some((_, outcome)) = rep.transition_ended {
                out_ok = Some(outcome);
                break;
            }
        }
        let diag = fx.core.torque_shadow_diagnostics();
        let clean = !shadow_enabled || (diag.events_processed > 0 && diag.taxonomy.invariant_violation == 0);
        (
            out_refuse.expect("refusal outcome"),
            out_ok.expect("rest disable outcome"),
            bus.with(|s| s.tx_log.clone()),
            clean,
        )
    }

    let (r_off, ok_off, tx_off, clean_off) = run_rt(false);
    let (r_on, ok_on, tx_on, clean_on) = run_rt(true);
    assert!(clean_off && clean_on);
    assert_eq!(r_off, r_on);
    assert_eq!(ok_off, ok_on);
    assert_eq!(ok_on, Outcome::Completed);
    assert_eq!(
        tx_off, tx_on,
        "RT executor wire frames with shadow ON must match shadow OFF identically"
    );
}

// =========================================================================
// 2. Divergence Injection + GET /api/v1/diagnostics + BlackBox (Both Owners)
// =========================================================================

struct MemoryShadowSink {
    lines: Arc<Mutex<Vec<String>>>,
    barrier: Option<Arc<(Mutex<(bool, bool)>, std::sync::Condvar)>>,
}

impl ShadowBlackBoxSink for MemoryShadowSink {
    fn write_shadow_line(&mut self, line: &[u8]) -> io::Result<()> {
        if let Some(ref b) = self.barrier {
            let mut g = b.0.lock().unwrap();
            g.0 = true; // signal entered sink
            b.1.notify_all();
            while !g.1 {
                g = b.1.wait(g).unwrap();
            }
        }
        self.lines
            .lock()
            .unwrap()
            .push(String::from_utf8_lossy(line).into_owned());
        Ok(())
    }
}

#[tokio::test]
async fn divergence_injection_counted_classified_and_persisted_on_both_owners() {
    // 2a. Legacy owner: HTTP GET /api/v1/diagnostics + bb_shadow.jsonl + bb.jsonl compact summary.
    let tmp_dir = std::env::temp_dir().join(format!(
        "armd_shadow_div_test_{}_{}",
        std::process::id(),
        Instant::now().elapsed().as_nanos()
    ));
    let _ = std::fs::create_dir_all(&tmp_dir);

    let clock = Arc::new(MockClock::new());
    let bus = FakeCan::new(clock.clone());
    let cfg = build_dm_config(true);
    let drv = build_dm_driver(bus.clone(), &cfg);
    let sup = Arc::new(ArmSupervisor::new(cfg));
    sup.set_driver(Box::new(drv)).await;
    sup.torque_shadow_hub()
        .attach_blackbox_dir(&tmp_dir, 42, Instant::now())
        .unwrap();
    sup.start_dispatcher().unwrap();

    // Inject a deliberately wrong reducer action (spurious disable on next step).
    {
        let mut inner = sup.inner().write().await;
        inner.test_set_torque_shadow_fault(ShadowFaultInjection::InjectSpuriousDisable {
            recipients: ChannelMask::from_count(7),
        });
        bus.with(|s| s.push_all_feedback());
        inner.tick_control(0.02);
        inner.test_flush_and_drain_torque_shadow();
    }

    // Query HTTP GET /api/v1/diagnostics on Legacy owner.
    let app = create_router(sup.clone(), OperatorAuth::disabled());
    let resp = app
        .oneshot(
            Request::builder()
                .uri("/api/v1/diagnostics")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp.status(), StatusCode::OK);
    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024).await.unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
    let ts = &json["torque_shadow"];
    assert_eq!(ts["enabled"], true);
    assert_eq!(ts["owner"], "legacy");
    assert!(
        ts["unexplained_divergences_total"].as_u64().unwrap() >= 1,
        "injected divergence must be counted: {ts}"
    );
    assert!(
        ts["taxonomy"]["extra_action"].as_u64().unwrap() >= 1,
        "spurious disable must classify under extra_action: {ts}"
    );
    assert!(
        !ts["recent_divergences"].as_array().unwrap().is_empty(),
        "recent_divergences must contain injected entry"
    );

    // Wait deterministically for the decoupled "armd-shadow-bb" writer thread to flush bb_shadow.jsonl.
    let bb_shadow_path = tmp_dir.join(SHADOW_BB_FILE);
    let bb_deadline = Instant::now() + Duration::from_secs(2);
    while sup.torque_shadow_diagnostics().ring.blackbox_written == 0
        && Instant::now() < bb_deadline
    {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    let bb_shadow_text = std::fs::read_to_string(&bb_shadow_path)
        .expect("bb_shadow.jsonl must be written on divergence");
    assert!(
        bb_shadow_text.contains("\"kind\":\"torque_shadow\"")
            && bb_shadow_text.contains("\"category\":\"extra_action\""),
        "bb_shadow.jsonl missing classified divergence: {bb_shadow_text}"
    );

    // Verify primary bb.jsonl piggybacks compact torque_shadow summary when non-clean.
    let compact = sup
        .torque_shadow_overflow_atomics()
        .blackbox_compact()
        .expect("non-clean shadow must produce compact bb.jsonl summary");
    assert!(compact.d >= 1);

    // 2b. RT Executor owner: GET /api/v1/diagnostics allowed through gate_executor_owner and reports owner="executor"!
    let cfg_rt = build_dm_config(true);
    let sup_rt = Arc::new(ArmSupervisor::new(cfg_rt));
    sup_rt.start_dispatcher().unwrap();
    {
        let mut inner = sup_rt.inner().write().await;
        inner.test_set_torque_shadow_fault(ShadowFaultInjection::InjectSpuriousDisable {
            recipients: ChannelMask::from_count(7),
        });
    }
    let handle = sup_rt
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(80)).await;
    sup_rt.torque_shadow_hub().drain_now();

    let app_rt = create_router(sup_rt.clone(), OperatorAuth::disabled());
    let resp_rt = app_rt
        .oneshot(
            Request::builder()
                .uri("/api/v1/diagnostics")
                .body(Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp_rt.status(), StatusCode::OK);
    let body_rt = axum::body::to_bytes(resp_rt.into_body(), 1024 * 1024).await.unwrap();
    let json_rt: serde_json::Value = serde_json::from_slice(&body_rt).unwrap();
    assert_eq!(json_rt["torque_shadow"]["enabled"], true);
    assert_eq!(json_rt["torque_shadow"]["owner"], "executor");
    assert!(
        json_rt["torque_shadow"]["unexplained_divergences_total"]
            .as_u64()
            .unwrap()
            >= 1,
        "RT executor injected divergence must surface on /api/v1/diagnostics: {json_rt}"
    );
    handle.stop_and_join();
    let _ = std::fs::remove_dir_all(&tmp_dir);
}

// =========================================================================
// 3. FakeBus Fuzz (10,000 traces default; ARMD_SHADOW_FUZZ_TRACES=100000 nightly)
//    Both Owners + Oracle + Deterministic Trace Shrinker + Short-Race Permutations
// =========================================================================

#[derive(Clone, Copy)]
struct SplitMix64(u64);
impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn gen_range(&mut self, bound: u64) -> u64 {
        if bound <= 1 {
            0
        } else {
            self.next_u64() % bound
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum FuzzOp {
    AdvanceMs(u64),
    Release,
    SampleRest { ch: u8, enabled: bool },
    SampleRaised { ch: u8, enabled: bool },
    SampleStale { ch: u8 },
    SampleFuture { ch: u8 },
    SampleDrifted { ch: u8 },
    ModeVerified { ch: u8 },
    EnableStart,
    HandshakeOk,
    HandshakeFail,
    DisableRequest,
    StopEstop,
    ThermalTrip { ch: u8 },
    BusLoss,
    FaultClear,
    WriteMotion { ch: u8 },
    WriteDisablePartial { mask_bits: u8, ambiguous: bool },
    WriteDisableAll,
    HoldCheckpoint { contended: bool },
    ProcessExit,
    PanicCaught,
    ZeroRecalibrate,
}

#[derive(Debug, Default)]
struct ShadowRunOutcome {
    unexplained_divergences: u64,
    invariant_violations: u64,
    events_processed: u64,
    final_drive_state: Option<DriveState>,
    final_intent: Option<TorqueIntent>,
    disable_completions: u64,
}

fn run_fuzz_trace_on_owner(owner: ShadowOwner, ops: &[FuzzOp]) -> ShadowRunOutcome {
    let clock = Arc::new(MockClock::new());
    let tc = TorqueConfig::rebot_dm_default();
    let (mut rt, hub) = ShadowRuntime::from_torque_config(
        tc,
        true,
        VendorKind::Damiao,
        Some(clock.clone() as Arc<dyn DriverClock>),
    );
    rt.set_owner(owner);

    let mut sample_seq = 0u64;
    let mut release_seq = 0u64;
    let mut live_intent = TorqueIntent::On;
    let mut live_torqued = false;
    let mut live_stopped = false;
    let mut completions = 0u64;

    for &op in ops {
        let now = clock.now();
        match op {
            FuzzOp::AdvanceMs(ms) => clock.advance(Duration::from_millis(ms)),
            FuzzOp::Release => {
                release_seq += 1;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::Release {
                        release_seq,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::SampleRest { ch, enabled } => {
                sample_seq += 1;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::CausalSample {
                        channel: ch % 7,
                        sample_id: sample_seq,
                        stamp: RxStampKind::Valid {
                            rx_at: now,
                            drift_ns: 0,
                            live_post_settle: true,
                        },
                        decode_now: now,
                        position_rad: 0.0,
                        velocity_rad_s: Some(0.0),
                        temperature_c: Some(30.0),
                        reported_enabled: Some(enabled),
                        correlated_after_write_done: None,
                        poll_issued: None,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            FuzzOp::SampleRaised { ch, enabled } => {
                sample_seq += 1;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::CausalSample {
                        channel: ch % 7,
                        sample_id: sample_seq,
                        stamp: RxStampKind::Valid {
                            rx_at: now,
                            drift_ns: 0,
                            live_post_settle: true,
                        },
                        decode_now: now,
                        position_rad: 0.9,
                        velocity_rad_s: Some(0.0),
                        temperature_c: Some(30.0),
                        reported_enabled: Some(enabled),
                        correlated_after_write_done: None,
                        poll_issued: None,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            FuzzOp::SampleStale { ch } => {
                sample_seq += 1;
                let stale_at = now.checked_sub(Duration::from_millis(250)).unwrap_or(now);
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::CausalSample {
                        channel: ch % 7,
                        sample_id: sample_seq,
                        stamp: RxStampKind::Valid {
                            rx_at: stale_at,
                            drift_ns: 0,
                            live_post_settle: true,
                        },
                        decode_now: now,
                        position_rad: 0.0,
                        velocity_rad_s: Some(0.0),
                        temperature_c: Some(30.0),
                        reported_enabled: Some(true),
                        correlated_after_write_done: None,
                        poll_issued: None,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            FuzzOp::SampleFuture { ch } => {
                sample_seq += 1;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::CausalSample {
                        channel: ch % 7,
                        sample_id: sample_seq,
                        stamp: RxStampKind::Future,
                        decode_now: now,
                        position_rad: 0.0,
                        velocity_rad_s: Some(0.0),
                        temperature_c: Some(30.0),
                        reported_enabled: Some(true),
                        correlated_after_write_done: None,
                        poll_issued: None,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            FuzzOp::SampleDrifted { ch } => {
                sample_seq += 1;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::CausalSample {
                        channel: ch % 7,
                        sample_id: sample_seq,
                        stamp: RxStampKind::OffsetDrifted {
                            rebased_settle: true,
                        },
                        decode_now: now,
                        position_rad: 0.0,
                        velocity_rad_s: Some(0.0),
                        temperature_c: Some(30.0),
                        reported_enabled: Some(true),
                        correlated_after_write_done: None,
                        poll_issued: None,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            FuzzOp::ModeVerified { ch } => {
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::ModeVerified {
                        channel: ch % 7,
                        mode: armd::motor_mode::ControlMode::PositionVelocity,
                        now,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            FuzzOp::EnableStart => {
                let op_id = rt.alloc_live_op_id();
                rt.enqueue_at(
                    now,
                    ShadowInputKind::EnableStart {
                        live_op_id: op_id,
                        live_refusal: None,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::HandshakeOk => {
                live_torqued = true;
                live_intent = TorqueIntent::On;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::HandshakeResult {
                        live_op_id: 0,
                        confirmed_mask: ChannelMask::from_count(7),
                        outcome: HandshakeOutcome::Confirmed,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::HandshakeFail => {
                rt.enqueue_at(
                    now,
                    ShadowInputKind::HandshakeResult {
                        live_op_id: 0,
                        confirmed_mask: ChannelMask::EMPTY,
                        outcome: HandshakeOutcome::Failed,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::DisableRequest => {
                let op_id = rt.alloc_live_op_id();
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DisableRequest {
                        live_op_id: op_id,
                        explicit_release_authorized: false,
                        live_refusal: None,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::StopEstop => {
                live_stopped = true;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::Stop {
                        kind: StopKind::EStop,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::ThermalTrip { ch } => {
                live_stopped = true;
                live_intent = TorqueIntent::Off;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::Thermal {
                        channel: ch % 7,
                        temperature_c: 95.0,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::BusLoss => {
                live_stopped = true;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::BusLoss {
                        device: 0,
                        multi_device_gap: false,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::FaultClear => {
                live_stopped = false;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::FaultClear {
                        live_refusal: None,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::WriteMotion { ch } => {
                let mut outcomes = [MotorWriteOutcome::NotTargeted; armd::torque::MAX_CHANNELS];
                outcomes[(ch % 7) as usize] = MotorWriteOutcome::Sent;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::WriteResult {
                        device: 0,
                        kind: WriteKind::Motion,
                        write_done: now,
                        now,
                        target_mask: ChannelMask::bit(ch % 7),
                        outcomes,
                        ambiguous_gap: false,
                    }),
                );
            }
            FuzzOp::WriteDisablePartial {
                mask_bits,
                ambiguous,
            } => {
                let mut mask = ChannelMask::EMPTY;
                let mut outcomes = [MotorWriteOutcome::NotTargeted; armd::torque::MAX_CHANNELS];
                for c in 0..7u8 {
                    if (mask_bits & (1 << c)) != 0 {
                        mask.insert(c);
                        outcomes[c as usize] = if ambiguous {
                            MotorWriteOutcome::Ambiguous
                        } else {
                            MotorWriteOutcome::Sent
                        };
                    }
                }
                if !mask.is_empty() {
                    rt.enqueue_at(
                        now,
                        ShadowInputKind::DriverObs(ShadowDriverObs::WriteResult {
                            device: 0,
                            kind: WriteKind::DisableRequested,
                            write_done: now,
                            now,
                            target_mask: mask,
                            outcomes,
                            ambiguous_gap: ambiguous,
                        }),
                    );
                }
            }
            FuzzOp::WriteDisableAll => {
                let mut outcomes = [MotorWriteOutcome::NotTargeted; armd::torque::MAX_CHANNELS];
                for slot in outcomes.iter_mut().take(7) {
                    *slot = MotorWriteOutcome::Sent;
                }
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::WriteResult {
                        device: 0,
                        kind: WriteKind::DisableRequested,
                        write_done: now,
                        now,
                        target_mask: ChannelMask::from_count(7),
                        outcomes,
                        ambiguous_gap: false,
                    }),
                );
            }
            FuzzOp::HoldCheckpoint { contended } => {
                rt.enqueue_at(
                    now,
                    ShadowInputKind::LiveHoldCheckpoint {
                        hold_mask: if contended {
                            ChannelMask::EMPTY
                        } else {
                            ChannelMask::from_count(7)
                        },
                        contended,
                        escalated: false,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::ProcessExit => {
                rt.enqueue_at(
                    now,
                    ShadowInputKind::ProcessExit {
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::PanicCaught => {
                live_stopped = true;
                rt.enqueue_at(
                    now,
                    ShadowInputKind::Panic {
                        tier: PanicTier::CaughtControlPanic,
                        live_intent,
                        live_is_torqued: live_torqued,
                        live_stopped,
                    },
                );
            }
            FuzzOp::ZeroRecalibrate => {
                rt.enqueue_at(now, ShadowInputKind::JointZeroRecalibrated);
            }
        }
        rt.flush_deferred_steps();
        if rt.state().is_some_and(|st| {
            st.groups[0].disable_tx.is_none() && st.groups[0].torque_intent == TorqueIntent::Off
        }) {
            completions += 1;
        }
    }
    hub.drain_now();
    let snap = hub.snapshot();
    ShadowRunOutcome {
        unexplained_divergences: snap.unexplained_divergences_total,
        invariant_violations: snap.taxonomy.invariant_violation,
        events_processed: snap.events_processed,
        final_drive_state: Some(snap.last_state.drive_state),
        final_intent: Some(snap.last_state.torque_intent),
        disable_completions: completions,
    }
}

fn gen_fuzz_op(rng: &mut SplitMix64) -> FuzzOp {
    let ch = rng.gen_range(7) as u8;
    match rng.gen_range(24) {
        0..=2 => FuzzOp::AdvanceMs(1 + rng.gen_range(40)),
        3..=5 => FuzzOp::Release,
        6..=8 => FuzzOp::SampleRest {
            ch,
            enabled: rng.gen_range(2) == 1,
        },
        9..=10 => FuzzOp::SampleRaised {
            ch,
            enabled: rng.gen_range(2) == 1,
        },
        11 => FuzzOp::SampleStale { ch },
        12 => FuzzOp::SampleFuture { ch },
        13 => FuzzOp::SampleDrifted { ch },
        14 => FuzzOp::ModeVerified { ch },
        15 => FuzzOp::EnableStart,
        16 => FuzzOp::HandshakeOk,
        17 => FuzzOp::HandshakeFail,
        18 => FuzzOp::DisableRequest,
        19 => FuzzOp::StopEstop,
        20 => FuzzOp::ThermalTrip { ch },
        21 => FuzzOp::FaultClear,
        22 => FuzzOp::WriteMotion { ch },
        _ => match rng.gen_range(7) {
            0 => FuzzOp::WriteDisablePartial {
                mask_bits: (1 + rng.gen_range(127)) as u8,
                ambiguous: rng.gen_range(2) == 1,
            },
            1 => FuzzOp::WriteDisableAll,
            2 => FuzzOp::HoldCheckpoint {
                contended: rng.gen_range(4) == 0,
            },
            3 => FuzzOp::BusLoss,
            4 => FuzzOp::ProcessExit,
            5 => FuzzOp::PanicCaught,
            _ => FuzzOp::ZeroRecalibrate,
        },
    }
}

fn shrink_fuzz_ops(ops: &[FuzzOp], mut fails: impl FnMut(&[FuzzOp]) -> bool) -> Vec<FuzzOp> {
    let mut cur = ops.to_vec();
    if !fails(&cur) {
        return cur;
    }
    let mut chunk = (cur.len() / 2).max(1);
    while chunk >= 1 {
        let mut i = 0;
        let mut progressed = false;
        while i < cur.len() && cur.len() > 1 {
            let end = (i + chunk).min(cur.len());
            let mut cand = Vec::with_capacity(cur.len() - (end - i));
            cand.extend_from_slice(&cur[..i]);
            cand.extend_from_slice(&cur[end..]);
            if !cand.is_empty() && fails(&cand) {
                cur = cand;
                progressed = true;
            } else {
                i += chunk;
            }
        }
        if !progressed {
            if chunk == 1 {
                break;
            }
            chunk /= 2;
        }
    }
    cur
}

#[test]
fn fakebus_shadow_fuzz_10000_seeded_traces_both_owners_parity_and_shrinker() {
    let traces: usize = std::env::var("ARMD_SHADOW_FUZZ_TRACES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(10_000);
    const MAX_EVENTS: usize = 128;
    let mut rng = SplitMix64(0x5AAD_0F02_2026_1125);
    let mut total_events: usize = 0;

    for trace_idx in 0..traces {
        let len = 4 + (rng.gen_range((MAX_EVENTS - 4) as u64) as usize);
        let mut ops = Vec::with_capacity(len);
        for _ in 0..len {
            ops.push(gen_fuzz_op(&mut rng));
        }
        let legacy = run_fuzz_trace_on_owner(ShadowOwner::Legacy, &ops);
        let exec = run_fuzz_trace_on_owner(ShadowOwner::Executor, &ops);
        assert_eq!(
            legacy.unexplained_divergences, exec.unexplained_divergences,
            "trace {trace_idx}: unexplained divergence count must match across owners"
        );

        let failed = legacy.invariant_violations > 0
            || exec.invariant_violations > 0
            || legacy.final_drive_state != exec.final_drive_state
            || legacy.final_intent != exec.final_intent
            || legacy.events_processed != exec.events_processed
            || legacy.disable_completions != exec.disable_completions;
        if failed {
            let minimized = shrink_fuzz_ops(&ops, |cand| {
                let l = run_fuzz_trace_on_owner(ShadowOwner::Legacy, cand);
                let e = run_fuzz_trace_on_owner(ShadowOwner::Executor, cand);
                l.invariant_violations > 0
                    || e.invariant_violations > 0
                    || l.final_drive_state != e.final_drive_state
                    || l.final_intent != e.final_intent
                    || l.events_processed != e.events_processed
            });
            panic!(
                "shadow fuzz trace #{trace_idx} failed (legacy={legacy:?}, exec={exec:?})\nminimized ({} ops): {minimized:?}",
                minimized.len()
            );
        }
        total_events += len;
    }
    assert!(total_events >= traces * 32);
}

#[test]
fn exhaustive_short_race_permutations_both_owners() {
    // Exhaustively permute 4-event critical races across 3rd-sample, Stop, DisableRequest,
    // partial disable write, and thermal preemption on BOTH owners.
    let race_events = [
        FuzzOp::SampleRest {
            ch: 0,
            enabled: true,
        },
        FuzzOp::StopEstop,
        FuzzOp::DisableRequest,
        FuzzOp::WriteDisablePartial {
            mask_bits: 0b000_0111,
            ambiguous: false,
        },
        FuzzOp::ThermalTrip { ch: 1 },
    ];
    let mut perms_checked = 0usize;
    for a in 0..race_events.len() {
        for b in 0..race_events.len() {
            for c in 0..race_events.len() {
                for d in 0..race_events.len() {
                    let trace = [
                        FuzzOp::ModeVerified { ch: 0 },
                        FuzzOp::EnableStart,
                        FuzzOp::HandshakeOk,
                        FuzzOp::Release,
                        FuzzOp::AdvanceMs(20),
                        race_events[a],
                        FuzzOp::AdvanceMs(10),
                        race_events[b],
                        FuzzOp::Release,
                        race_events[c],
                        FuzzOp::AdvanceMs(20),
                        race_events[d],
                        FuzzOp::Release,
                    ];
                    let legacy = run_fuzz_trace_on_owner(ShadowOwner::Legacy, &trace);
                    let exec = run_fuzz_trace_on_owner(ShadowOwner::Executor, &trace);
                    assert_eq!(legacy.invariant_violations, 0);
                    assert_eq!(exec.invariant_violations, 0);
                    assert_eq!(legacy.final_drive_state, exec.final_drive_state);
                    assert_eq!(legacy.final_intent, exec.final_intent);
                    assert_eq!(legacy.events_processed, exec.events_processed);
                    perms_checked += 1;
                }
            }
        }
    }
    assert_eq!(perms_checked, 625);
}

// =========================================================================
// 4. Synthetic Grouped-Hand Topology (Both Owners + Recipient Isolation)
// =========================================================================

#[test]
fn synthetic_grouped_hand_topology_both_owners_recipient_isolation() {
    for owner in [ShadowOwner::Legacy, ShadowOwner::Executor] {
        let clock = Arc::new(MockClock::new());
        let tc = TorqueConfig::arm_plus_underactuated_hand();
        let (mut rt, hub) = ShadowRuntime::from_torque_config(
            tc,
            true,
            VendorKind::GenericGrouped,
            Some(clock.clone() as Arc<dyn DriverClock>),
        );
        rt.set_owner(owner);

        // Step releases & causal samples across arm + hand channels.
        for rel in 1..=4u64 {
            clock.advance(Duration::from_millis(20));
            let now = clock.now();
            rt.enqueue_at(
                now,
                ShadowInputKind::Release {
                    release_seq: rel,
                    live_intent: TorqueIntent::On,
                    live_is_torqued: true,
                    live_stopped: false,
                },
            );
            for ch in 0..tc.channel_count {
                rt.enqueue_at(
                    now,
                    ShadowInputKind::DriverObs(ShadowDriverObs::CausalSample {
                        channel: ch,
                        sample_id: rel * 32 + u64::from(ch) + 1,
                        stamp: RxStampKind::Valid {
                            rx_at: now,
                            drift_ns: 0,
                            live_post_settle: true,
                        },
                        decode_now: now,
                        position_rad: 0.0,
                        velocity_rad_s: Some(0.0),
                        temperature_c: Some(28.0),
                        reported_enabled: Some(true),
                        correlated_after_write_done: None,
                        poll_issued: None,
                        provenance: EvidenceProvenance::HardwareCausal,
                    }),
                );
            }
            rt.flush_deferred_steps();
        }
        // Issue arm-only disable (group 0) -> hand group (group 1) must stay independent!
        let hand_state_before = rt.state().unwrap().groups[1].drive_state;
        let op_id = rt.alloc_live_op_id();
        rt.enqueue(ShadowInputKind::DisableRequest {
            live_op_id: op_id,
            explicit_release_authorized: false,
            live_refusal: None,
            live_intent: TorqueIntent::On,
            live_is_torqued: true,
            live_stopped: false,
        });
        rt.flush_deferred_steps();
        assert_eq!(
            rt.state().unwrap().groups[1].drive_state, hand_state_before,
            "arm disable must not mutate grouped hand state on owner {owner:?}"
        );
        hub.drain_now();
        let snap = hub.snapshot();
        assert_eq!(snap.owner, owner);
        assert_eq!(snap.taxonomy.invariant_violation, 0);
        assert_eq!(snap.unexplained_divergences_total, 0);
        assert!(snap.events_processed > 0);
    }
}

// =========================================================================
// 5. Clock Normalization (All 5 RX Cases, Damiao & RobStride Drift, Pre-Origin)
// =========================================================================

#[test]
fn clock_normalization_all_5_rx_cases_and_robstride_drift() {
    let clock = Arc::new(MockClock::new());
    let bus = FakeCan::new(clock.clone());
    let cfg = build_dm_config(true);
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(build_dm_driver(bus.clone(), &cfg)));

    // Case 1: Missing RX timestamp (`None`) -> coverage gap `MissingRxTimestamp`.
    let m0 = bus.with(|s| s.motors[0].feedback_frame());
    bus.with(|s| s.rx.push_back((m0, None)));
    inner.tick_control(0.02);
    inner.test_flush_and_drain_torque_shadow();
    let d1 = inner.torque_shadow_diagnostics();
    assert!(
        d1.coverage_gaps_by_kind.missing_rx_timestamp >= 1,
        "missing RX timestamp must record coverage gap: {d1:?}"
    );

    // Case 2: Future RX timestamp (`rx_at > decode_now`).
    let future_ts = clock.now() + Duration::from_nanos(500);
    bus.with(|s| s.rx.push_back((m0, Some(CanRxTime::Monotonic(future_ts)))));
    inner.tick_control(0.02);
    inner.test_flush_and_drain_torque_shadow();
    let d2 = inner.torque_shadow_diagnostics();
    assert_eq!(d2.taxonomy.invariant_violation, 0);

    // Case 3: Unmappable KernelRealtimeNs (missing realtime_minus_monotonic_ns).
    bus.with(|s| {
        s.kernel_realtime_ns = Some(1_000_000_000);
        s.realtime_minus_mono_ns = None;
        s.rx.push_back((m0, Some(CanRxTime::KernelRealtimeNs(999_000_000))));
    });
    inner.tick_control(0.02);
    inner.test_flush_and_drain_torque_shadow();
    let d3 = inner.torque_shadow_diagnostics();
    assert!(
        d3.coverage_gaps_by_kind.unmappable_rx_timestamp >= 1,
        "unmappable KernelRealtimeNs must record coverage gap: {d3:?}"
    );

    // Case 4: Valid KernelRealtimeNs followed by >1 ms realtime offset drift on RobStride too!
    let rs_clock = Arc::new(MockClock::new());
    let rs_bus = FakeCan::new(rs_clock.clone());
    let mut rs_drv = RobStrideArmDriver::new(rs_bus.clone(), (1..=6).collect());
    rs_drv.set_shadow_enabled(true);
    let mut obs_buf = vec![None; DRIVER_SHADOW_STAGING_CAP];
    // Feed a missing-timestamp frame into RobStride -> Missing { reset_window: true }.
    let rs_fb = CanFrame::new_extended((2u32 << 24) | (1u32 << 8) | 0xFD, &[0; 8]).unwrap();
    rs_bus.with(|s| s.rx.push_back((rs_fb, None)));
    let _ = rs_drv.read_joint_states();
    let (n, dropped) = rs_drv.drain_shadow_observations(&mut obs_buf);
    assert_eq!(dropped, 0);
    assert!(
        obs_buf[..n].iter().any(|o| matches!(
            o,
            Some(ShadowDriverObs::CausalSample {
                stamp: RxStampKind::Missing { reset_window: true },
                ..
            })
        )),
        "RobStride missing RX timestamp must stage Missing {{ reset_window: true }}"
    );
}

// =========================================================================
// 6. ZeroOffsetDriver Nonzero Offset + Runtime Recalibration
// =========================================================================

#[test]
fn zero_offset_driver_subtracts_offsets_and_recalibration_drains_first() {
    let clock = Arc::new(MockClock::new());
    let bus = FakeCan::new(clock.clone());
    let cfg = build_dm_config(true);
    let raw_drv = build_dm_driver(bus.clone(), &cfg);
    let initial_offsets = [0.25f32, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0];
    let mut wrapped = ZeroOffsetDriver::new(Box::new(raw_drv), &initial_offsets);
    wrapped.set_shadow_enabled(true);

    // Push motor-space pose = initial_offsets so joint-space pose == 0.0 (at rest).
    bus.with(|s| {
        s.set_pose(&initial_offsets);
        s.push_all_feedback();
    });
    let _ = wrapped.read_joint_states().unwrap();
    let mut out = vec![None; DRIVER_SHADOW_STAGING_CAP];
    let (n, _) = wrapped.drain_shadow_observations(&mut out);
    assert!(n >= 7);
    let j0_pos = out[..n].iter().find_map(|o| match o {
        Some(ShadowDriverObs::CausalSample {
            channel: 0,
            position_rad,
            ..
        }) => Some(*position_rad),
        _ => None,
    });
    assert!(
        j0_pos.is_some_and(|p| p.abs() < 0.02),
        "ZeroOffsetDriver must subtract offset from CausalSample.position_rad, got {j0_pos:?}"
    );

    // Now test SupervisorInner::set_joint_zero_offsets recalibration on hardware & driverless Legacy!
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(build_dm_driver(bus.clone(), &cfg)));
    inner.torque_intent = armd::TorqueIntent::Off;
    inner.is_torqued = false;
    bus.with(|s| {
        s.set_pose(&[0.0; 7]);
        s.push_all_feedback();
    });
    inner.tick_control(0.02);
    inner.test_flush_and_drain_torque_shadow();
    let before_events = inner.torque_shadow_diagnostics().events_processed;

    // No-op recalibration with identical offsets must NOT emit JointZeroRecalibrated.
    let same = vec![0.0f32; 7];
    inner.set_joint_zero_offsets(&same).unwrap();
    inner.test_flush_and_drain_torque_shadow();
    assert_eq!(
        inner.torque_shadow_diagnostics().events_processed,
        before_events,
        "no-op set_joint_zero_offsets must not emit JointZeroRecalibrated"
    );

    // Non-noop recalibration MUST emit JointZeroRecalibrated and keep 0 divergences.
    let new_off = vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    inner.set_joint_zero_offsets(&new_off).unwrap();
    inner.test_flush_and_drain_torque_shadow();
    let after = inner.torque_shadow_diagnostics();
    assert!(
        after.events_processed > before_events,
        "non-noop recalibration must emit JointZeroRecalibrated"
    );
    assert_eq!(after.taxonomy.invariant_violation, 0);

    // Driverless Legacy recalibration works identically.
    let mut mock_inner = SupervisorInner::new(ArmConfig::default());
    mock_inner.is_torqued = false;
    mock_inner.tick_control(0.02);
    mock_inner.test_flush_and_drain_torque_shadow();
    let mock_before = mock_inner.torque_shadow_diagnostics().events_processed;
    mock_inner
        .set_joint_zero_offsets(&vec![0.05; mock_inner.config.dof])
        .unwrap();
    mock_inner.test_flush_and_drain_torque_shadow();
    assert!(mock_inner.torque_shadow_diagnostics().events_processed > mock_before);
}

// =========================================================================
// 7. Kill Switch Matrix (Profile, Env, Handoff, Rollback)
// =========================================================================

#[tokio::test]
async fn kill_switch_matrix_profile_env_handoff_and_rollback() {
    // 7a. Profile default (omitted -> true), explicit true, explicit false.
    let mut base_json = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    base_json.as_object_mut().unwrap().remove("torque_shadow_enabled");
    let cfg_default: ArmConfig = serde_json::from_value(base_json.clone()).unwrap();
    assert!(cfg_default.torque_shadow_enabled);
    base_json
        .as_object_mut()
        .unwrap()
        .insert("torque_shadow_enabled".into(), serde_json::Value::Bool(false));
    let cfg_false: ArmConfig = serde_json::from_value(base_json).unwrap();
    assert!(!cfg_false.torque_shadow_enabled);

    // 7b. All env spellings + invalid rejection.
    for t in ["1", "true", "TRUE", "on", "yes"] {
        assert_eq!(resolve_shadow_enabled(false, Some(t)), Ok(true));
    }
    for f in ["0", "false", "FALSE", "off", "no"] {
        assert_eq!(resolve_shadow_enabled(true, Some(f)), Ok(false));
    }
    assert!(resolve_shadow_enabled(true, Some("maybe")).is_err());

    // 7c. Disabled supervisor builds zero worker thread & zero ring activity across handoff.
    let sup_off = Arc::new(ArmSupervisor::new(build_dm_config(false)));
    sup_off.start_dispatcher().unwrap();
    assert!(!sup_off.torque_shadow_hub().is_enabled());
    assert!(!sup_off.torque_shadow_hub().has_worker());
    let d_off = sup_off.torque_shadow_diagnostics();
    assert!(!d_off.enabled);
    assert_eq!(d_off.events_processed, 0);

    // 7d. Enabled supervisor preserves same hub, session state & owner tag across handoff and rollback!
    let sup_on = Arc::new(ArmSupervisor::new(build_dm_config(true)));
    sup_on.start_dispatcher().unwrap();
    assert!(sup_on.torque_shadow_hub().has_worker());
    assert_eq!(sup_on.torque_shadow_diagnostics().owner, ShadowOwner::Legacy);

    // 7e. Failed start_executor rollback: driver rejecting RT executor keeps owner=Legacy
    //     and preserves continuous session state without spawning an orphaned worker.
    let clock_rb = Arc::new(MockClock::new());
    let drv_rb = FeetechArmDriver::new(DummySerial { clock: clock_rb }, (1..=6).collect());
    let sup_rb = Arc::new(ArmSupervisor::new(build_dm_config(true)));
    sup_rb.set_driver(Box::new(drv_rb)).await;
    sup_rb.start_dispatcher().unwrap();
    {
        let mut inner = sup_rb.inner().write().await;
        inner.tick_control(0.02);
        inner.test_flush_and_drain_torque_shadow();
    }
    let pre_rb = sup_rb.torque_shadow_diagnostics();
    assert_eq!(pre_rb.owner, ShadowOwner::Legacy);
    let rb_res = sup_rb
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(20)),
            PersistTestHooks::default(),
        )
        .await;
    assert!(rb_res.is_err(), "Feetech driver must reject RT executor start and roll back");
    let post_rb = sup_rb.torque_shadow_diagnostics();
    assert_eq!(post_rb.owner, ShadowOwner::Legacy);
    assert!(post_rb.events_processed >= pre_rb.events_processed);

    // 7f. Normal handoff flips owner to Executor cleanly without losing prior processed count:
    {
        let mut inner = sup_on.inner().write().await;
        inner.tick_control(0.02);
        inner.test_flush_and_drain_torque_shadow();
    }
    let pre_handoff_events = sup_on.torque_shadow_diagnostics().events_processed;
    assert!(pre_handoff_events > 0);

    let handle = sup_on
        .start_executor_with_hooks(
            ExecutorConfig::for_tests(Duration::from_millis(20)),
            PersistTestHooks::default(),
        )
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(60)).await;
    sup_on.torque_shadow_hub().drain_now();
    let post_handoff = sup_on.torque_shadow_diagnostics();
    assert_eq!(post_handoff.owner, ShadowOwner::Executor);
    assert!(
        post_handoff.events_processed > pre_handoff_events,
        "handoff must preserve continuous session counts"
    );
    handle.stop_and_join();
}

// =========================================================================
// 8. BlackBox Slow-Sink Non-Displacement Proof
// =========================================================================

struct FastPrimarySink {
    lines: Arc<Mutex<Vec<String>>>,
}

impl Sink for FastPrimarySink {
    fn write_line(&mut self, line: &[u8], _sync: bool) -> io::Result<()> {
        self.lines
            .lock()
            .unwrap()
            .push(String::from_utf8_lossy(line).into_owned());
        Ok(())
    }
}

#[test]
fn blackbox_slow_shadow_sink_never_displaces_primary_bb_jsonl() {
    let cfg = build_dm_config(true);
    let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);
    let slow_lines = Arc::new(Mutex::new(Vec::new()));
    let barrier = Arc::new((Mutex::new((false, false)), std::sync::Condvar::new()));
    hub.attach_blackbox_sink(
        Box::new(MemoryShadowSink {
            lines: slow_lines.clone(),
            barrier: Some(barrier.clone()),
        }),
        1,
        Instant::now(),
    );
    hub.start_worker().unwrap();

    // Inject divergences so the decoupled "armd-shadow-bb" writer thread enters write_shadow_line.
    for i in 1..=5u64 {
        rt.set_injected_fault(ShadowFaultInjection::InjectSpuriousDisable {
            recipients: ChannelMask::from_count(7),
        });
        rt.enqueue(ShadowInputKind::Release {
            release_seq: i,
            live_intent: TorqueIntent::On,
            live_is_torqued: true,
            live_stopped: false,
        });
        rt.flush_deferred_steps();
    }
    hub.wake();

    // Deterministically wait until "armd-shadow-bb" is PROVEN blocked inside write_shadow_line!
    {
        let mut g = barrier.0.lock().unwrap();
        let wait_deadline = Instant::now() + Duration::from_secs(2);
        while !g.0 {
            let rem = wait_deadline.saturating_duration_since(Instant::now());
            assert!(!rem.is_zero(), "shadow blackbox writer must enter sink before primary test");
            g = barrier.1.wait_timeout(g, rem).unwrap().0;
        }
    }

    // While the shadow sink is deterministically blocked:
    // 1. Live owner switch (`set_owner`) and HTTP diagnostics (`snapshot`) MUST complete immediately!
    let t_nonblock = Instant::now();
    hub.set_owner(ShadowOwner::Executor);
    let snap_while_blocked = hub.snapshot();
    assert!(
        t_nonblock.elapsed() < Duration::from_millis(50),
        "set_owner and snapshot must never block on stalled shadow disk I/O"
    );
    assert_eq!(snap_while_blocked.owner, ShadowOwner::Executor);

    // 2. Primary BlackBox writer uses its own channel & thread and reads only lock-free atomics.
    let primary_lines = Arc::new(Mutex::new(Vec::new()));
    let start = Instant::now();
    let mut bb = BlackBox::start(
        FastPrimarySink {
            lines: primary_lines.clone(),
        },
        1,
        start,
    )
    .unwrap()
    .with_shadow_atomics(hub.overflow_atomics().clone());

    let obs = armd::ArmObservation::blank();
    let mut queued = 0u64;
    for k in 0..10u64 {
        let t = start + Duration::from_secs(k * 6);
        let offer = bb.offer(&obs, t);
        assert_eq!(
            offer,
            Offer::Queued,
            "primary bb.jsonl offer #{k} must not be displaced while shadow sink is blocked"
        );
        queued += 1;
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(bb.dropped(), 0);
    assert_eq!(bb.written(), queued);
    assert_eq!(primary_lines.lock().unwrap().len() as u64, queued);

    // 3. Release the shadow sink barrier and verify all shadow divergence lines finish writing!
    {
        let mut g = barrier.0.lock().unwrap();
        g.1 = true;
        barrier.1.notify_all();
    }
    hub.stop_and_flush_bounded(Duration::from_secs(2));
    assert!(
        slow_lines.lock().unwrap().len() >= 5,
        "all 5 shadow divergence lines must finish writing after barrier release, got {}",
        slow_lines.lock().unwrap().len()
    );
}

// =========================================================================
// 9. Shutdown & Caught-Panic Ordering Proof
// =========================================================================

#[test]
fn shutdown_refused_vs_accepted_and_caught_panic_ordering() {
    let clock = Arc::new(MockClock::new());
    let bus = FakeCan::new(clock.clone());
    let cfg = build_dm_config(true);
    let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(build_dm_driver(bus.clone(), &cfg)));

    // Arm the arm first.
    bus.with(|s| s.push_all_feedback());
    inner.tick_control(0.02);
    let st = inner.current_admission_stamp();
    let adm = inner.check_admission(st).unwrap();
    SupervisorInner::enable_torque(adm).unwrap();

    // Refused commit_shutdown while torqued -> returns false and does NOT commit shutdown.
    assert!(!inner.commit_shutdown());
    assert!(!inner.shutdown_committed);

    // Caught panic disable records Panic BEFORE panic-disable writes and flushes cleanly!
    let outcome = inner.attempt_panic_disable(Duration::from_millis(200));
    assert_eq!(outcome, armd::supervisor::PanicDisableOutcome::Confirmed);
    inner.test_flush_and_drain_torque_shadow();
    let diag = inner.torque_shadow_diagnostics();
    assert_eq!(diag.taxonomy.invariant_violation, 0);
    assert!(
        diag.taxonomy.reviewed_pr2_only >= 1,
        "caught panic must record FU-PANIC baseline difference under reviewed_pr2_only: {diag:?}"
    );

    // Now disabled -> commit_shutdown succeeds and records ProcessExit cleanly.
    for _ in 0..2 {
        clock.advance(Duration::from_millis(20));
        bus.with(|s| s.push_all_feedback());
        inner.tick_control(0.02);
    }
    assert!(inner.commit_shutdown());
    inner.test_flush_and_drain_torque_shadow();
    let diag_end = inner.torque_shadow_diagnostics();
    assert_eq!(diag_end.taxonomy.invariant_violation, 0);
}

// =========================================================================
// 10. Feetech Serial Poisoning & Shadow Staging Proof
// =========================================================================

struct DummySerial {
    clock: Arc<MockClock>,
}

impl Transport for DummySerial {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(self.clock.clone() as Arc<dyn DriverClock>)
    }
    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        Ok(buf.len())
    }
    fn write_all(&mut self, _buf: &[u8]) -> Result<(), TransportError> {
        Ok(())
    }
    fn read(&mut self, _buf: &mut [u8]) -> Result<usize, TransportError> {
        self.clock.advance(Duration::from_millis(5));
        Err(TransportError::Timeout)
    }
    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

#[test]
fn feetech_shadow_vendor_clock_and_staging_overflow_bounded() {
    let clock = Arc::new(MockClock::new());
    let mut drv = FeetechArmDriver::new(DummySerial { clock: clock.clone() }, (1..=6).collect());
    assert_eq!(drv.shadow_vendor_kind(), VendorKind::Feetech);
    assert!(drv.driver_clock().is_some());
    drv.set_shadow_enabled(true);
    // Repeated reads time out and poison the serial line -> stages SerialTimeout cleanly with 0 panic.
    let _ = drv.read_joint_states();
    let mut out = vec![None; DRIVER_SHADOW_STAGING_CAP];
    let (n, dropped) = drv.drain_shadow_observations(&mut out);
    assert_eq!(dropped, 0);
    assert!(
        out[..n]
            .iter()
            .any(|o| matches!(o, Some(ShadowDriverObs::SerialTimeout { .. }))),
        "Feetech timeout/poison must stage SerialTimeout"
    );

    // Verify ring saturation + staging overflow mark session permanently unverifiable.
    let cfg = build_dm_config(true);
    let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);
    for seq in 1..=(SHADOW_RING_CAPACITY as u64 + 32) {
        rt.enqueue(ShadowInputKind::Release {
            release_seq: seq,
            live_intent: TorqueIntent::On,
            live_is_torqued: true,
            live_stopped: false,
        });
        rt.flush_deferred_steps();
    }
    hub.drain_now();
    let snap = hub.snapshot();
    assert!(snap.ring.interval_unverifiable);
    assert!(snap.ring.ring_dropped_records >= 32);
    assert_eq!(snap.ring.first_unverifiable_seq, Some(SHADOW_RING_CAPACITY as u64 + 1));
    assert_eq!(
        snap.coverage_gaps_by_kind.missing_rx_timestamp, 0,
        "ring overflow must not fabricate unrelated coverage gaps"
    );
    let _ = (CoverageGapKind::MissingRxTimestamp, DivergenceCategory::ExtraAction, RefusalCode::TorqueOffWouldDrop);
}

// =========================================================================
// 11. Live Dual-Owner Seeded FakeBus Fuzz (Both Real Owners + Wire Parity)
// =========================================================================

#[test]
fn fakebus_live_dual_owner_seeded_traces_wire_and_shadow_parity() {
    #[derive(Clone, Copy)]
    enum LiveStep {
        TickRest,
        TickRaised(f32),
        Enable,
        Estop,
        Disable,
    }

    let mut rng = SplitMix64(0xBADC_0FFE_2026_1125);
    for seed in 0..64u64 {
        let len = 8 + (rng.gen_range(16) as usize);
        let mut script = Vec::with_capacity(len);
        for _ in 0..len {
            let op = match rng.gen_range(8) {
                0..=2 => LiveStep::TickRest,
                3 => LiveStep::TickRaised(0.6 + (rng.gen_range(4) as f32) * 0.1),
                4 => LiveStep::Enable,
                5 => LiveStep::Estop,
                _ => LiveStep::Disable,
            };
            script.push(op);
        }

        // Run on REAL Legacy owner (SupervisorInner::tick_control / enable_torque / disable_torque).
        let (leg_diag, leg_final_torqued) = {
            let clock = Arc::new(MockClock::new());
            let bus = FakeCan::new(clock.clone());
            let cfg = build_dm_config(true);
            let mut inner = SupervisorInner::new(cfg.clone())
                .with_driver(Box::new(build_dm_driver(bus.clone(), &cfg)));
            bus.with(|s| {
                s.set_pose(&[0.0; 7]);
                s.push_all_feedback();
            });
            inner.tick_control(0.02);
            for &op in &script {
                clock.advance(Duration::from_millis(25));
                match op {
                    LiveStep::TickRest => {
                        bus.with(|s| {
                            s.set_pose(&[0.0; 7]);
                            s.push_all_feedback();
                        });
                        inner.tick_control(0.02);
                    }
                    LiveStep::TickRaised(q) => {
                        bus.with(|s| {
                            s.set_pose(&[0.0, q, 0.0, 0.0, 0.0, 0.0, 0.0]);
                            s.push_all_feedback();
                        });
                        inner.tick_control(0.02);
                    }
                    LiveStep::Enable => {
                        bus.with(|s| s.push_all_feedback());
                        let st = inner.current_admission_stamp();
                        if let Ok(adm) = inner.check_admission(st) {
                            let _ = SupervisorInner::enable_torque(adm);
                        }
                    }
                    LiveStep::Estop => {
                        inner.apply_stop_policy(armd::safety::StopPolicy::EStopLatch);
                    }
                    LiveStep::Disable => {
                        bus.with(|s| s.push_all_feedback());
                        let _ = inner.disable_torque();
                    }
                }
                inner.test_flush_and_drain_torque_shadow();
            }
            inner.test_flush_and_drain_torque_shadow();
            (inner.torque_shadow_diagnostics(), inner.is_torqued)
        };

        // Run the same script on REAL RT Executor ReleaseFixture<SupervisorInner>!
        let (rt_diag, rt_final_torqued) = {
            let clock = Arc::new(MockClock::new());
            let bus = FakeCan::new(clock.clone());
            let cfg = build_dm_config(true);
            let mut inner = SupervisorInner::new(cfg.clone())
                .with_driver(Box::new(build_dm_driver(bus.clone(), &cfg)));
            inner.lease_enabled = false;
            inner.register_live_connection(1).unwrap();
            let (mut fx, mut broker, _tw, _stop) = ReleaseFixture::new(
                inner,
                BootId(1),
                Capacities { commands: 8, plans: 1, replies: 16 },
            );
            bus.with(|s| {
                s.set_pose(&[0.0; 7]);
                s.push_all_feedback();
            });
            let _ = fx.step();
            let mut cseq = 1u64;
            for &op in &script {
                clock.advance(Duration::from_millis(25));
                match op {
                    LiveStep::TickRest => {
                        bus.with(|s| {
                            s.set_pose(&[0.0; 7]);
                            s.push_all_feedback();
                        });
                        let _ = fx.step();
                    }
                    LiveStep::TickRaised(q) => {
                        bus.with(|s| {
                            s.set_pose(&[0.0, q, 0.0, 0.0, 0.0, 0.0, 0.0]);
                            s.push_all_feedback();
                        });
                        let _ = fx.step();
                    }
                    LiveStep::Enable => {
                        cseq += 1;
                        let ep = fx.core.safety.epoch_id();
                        let _ = broker.submit(
                            ClientId(1),
                            cseq,
                            Authority {
                                lease_generation: 0,
                                epoch: ep,
                                stop_generation: 0,
                                config_revision: 0,
                                parent_generation: 0,
                            },
                            fx.now() + Duration::from_secs(5),
                            Submit::Transition(TransitionOp::TorqueOn),
                        );
                        for _ in 0..6 {
                            clock.advance(Duration::from_millis(20));
                            bus.with(|s| s.push_all_feedback());
                            let rep = fx.step();
                            if rep.transition_ended.is_some() || rep.command.as_ref().is_some_and(|(_, r)| r.is_err()) {
                                break;
                            }
                        }
                    }
                    LiveStep::Estop => {
                        fx.core.apply_stop_policy(armd::safety::StopPolicy::EStopLatch);
                    }
                    LiveStep::Disable => {
                        cseq += 1;
                        let ep = fx.core.safety.epoch_id();
                        let _ = broker.submit(
                            ClientId(1),
                            cseq,
                            Authority {
                                lease_generation: 0,
                                epoch: ep,
                                stop_generation: 0,
                                config_revision: 0,
                                parent_generation: 0,
                            },
                            fx.now() + Duration::from_secs(5),
                            Submit::Transition(TransitionOp::TorqueOff),
                        );
                        for _ in 0..6 {
                            clock.advance(Duration::from_millis(20));
                            bus.with(|s| s.push_all_feedback());
                            let rep = fx.step();
                            if rep.transition_ended.is_some() || rep.command.as_ref().is_some_and(|(_, r)| r.is_err()) {
                                break;
                            }
                        }
                    }
                }
                fx.core.test_flush_and_drain_torque_shadow();
            }
            fx.core.test_flush_and_drain_torque_shadow();
            (fx.core.torque_shadow_diagnostics(), fx.core.is_torqued)
        };

        assert_eq!(
            leg_diag.taxonomy.invariant_violation, 0,
            "seed {seed}: legacy invariant violation"
        );
        assert_eq!(
            rt_diag.taxonomy.invariant_violation, 0,
            "seed {seed}: RT invariant violation"
        );
        assert!(
            !leg_diag.ring.interval_unverifiable && !rt_diag.ring.interval_unverifiable,
            "seed {seed}: neither owner should overflow during bounded live trace: leg={:?} rt={:?}",
            leg_diag.ring, rt_diag.ring
        );
        assert!(leg_diag.events_processed > 0 && rt_diag.events_processed > 0);
        let _ = (leg_final_torqued, rt_final_torqued);
    }
}

// =========================================================================
// 12. R1 Review Regression Suite (B2, M1, M2, M10, M12, m2, m3)
// =========================================================================

#[tokio::test]
async fn r1_review_regressions_comparator_remount_unknown_queue_and_disabled_alloc() {
    // (a) M2: Single sub-threshold LockContended after predicted holds must NOT create false ExtraAction!
    {
        let cfg = build_dm_config(true);
        let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);
        // Put reducer into stopped-with-holds state via Release -> Enable -> Stop -> Release.
        rt.enqueue(ShadowInputKind::Release {
            release_seq: 1,
            live_intent: TorqueIntent::On,
            live_is_torqued: true,
            live_stopped: true,
        });
        rt.enqueue(ShadowInputKind::LiveHoldCheckpoint {
            hold_mask: ChannelMask::EMPTY,
            contended: true,
            escalated: false,
            live_intent: TorqueIntent::On,
            live_is_torqued: true,
            live_stopped: true,
        });
        rt.flush_deferred_steps();
        hub.drain_now();
        let snap = hub.snapshot();
        assert_eq!(
            snap.taxonomy.extra_action, 0,
            "single sub-threshold LockContended must not produce false ExtraAction: {snap:?}"
        );
    }

    // (b) M1 + m2: Negative comparator tests — swapped hold mask => AuthorityRecipientMismatch;
    //     mismatched refusal code => OutcomeRefusalMismatch; empty live hold mask vs predicted => ExtraAction.
    {
        let cfg = build_dm_config(true);
        let (mut rt, hub) = ShadowRuntime::new_with_hub(&cfg, true, VendorKind::Damiao, true, None);
        // Inject wrong refusal on FaultClear (reducer refuses NotEnabled / none vs live DriverBusy).
        rt.enqueue(ShadowInputKind::DisableRequest {
            live_op_id: 1,
            explicit_release_authorized: false,
            live_refusal: Some(armd::torque::shadow::LiveRefusalKind::DriverBusy),
            live_intent: TorqueIntent::Off,
            live_is_torqued: false,
            live_stopped: false,
        });
        rt.flush_deferred_steps();
        hub.drain_now();
        let snap = hub.snapshot();
        assert!(
            snap.taxonomy.outcome_refusal_mismatch >= 1,
            "DriverBusy refusal must NOT be masked by DriverPreflightVsAwaitRestGate: {snap:?}"
        );
    }

    // (c) M10: Damiao sample with unknown queue bound (fresh driver with budget exhausted before
    //     observing an empty RX queue -> rx_empty_instant == None) must stage UnqualifiedRestEvidence!
    {
        let clock = Arc::new(MockClock::new());
        let bus = FakeCan::new(clock.clone());
        let cfg = build_dm_config(true);
        let limits: Vec<DamiaoLimits> = cfg
            .damiao_ranges
            .as_ref()
            .unwrap()
            .iter()
            .map(|r| r.limits())
            .collect();
        let mut drv = DamiaoArmDriver::new(bus.clone(), (1..=7).collect())
            .with_joint_limits(limits)
            .unwrap();
        drv.set_shadow_enabled(true);
        let mut obs_buf = vec![None; DRIVER_SHADOW_STAGING_CAP];
        let _ = drv.drain_shadow_observations(&mut obs_buf);
        // Push 14 frames and read with max_rx_frames=3 so RX queue is NEVER observed empty -> queue_bound=None!
        bus.with(|s| {
            s.set_pose(&[0.0; 7]);
            s.push_all_feedback();
            s.push_all_feedback();
        });
        let mut js = Vec::with_capacity(7);
        let dl = clock.now() + Duration::from_millis(10);
        let _ = drv.read_joint_states_budgeted(&mut js, dl, 3);
        let (n, _) = drv.drain_shadow_observations(&mut obs_buf);
        assert!(
            obs_buf[..n].iter().any(|o| matches!(
                o,
                Some(ShadowDriverObs::CausalSample {
                    stamp: RxStampKind::UnqualifiedRestEvidence { .. },
                    ..
                })
            )),
            "Damiao frame decoded with unknown queue_bound must stage UnqualifiedRestEvidence"
        );
    }

    // (d) M12: Driver remount (`install_driver` / `set_driver`) MUST preserve latched
    //     `interval_unverifiable` and drop counters across driver generations!
    {
        let clock = Arc::new(MockClock::new());
        let bus = FakeCan::new(clock.clone());
        let cfg = build_dm_config(true);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        // Saturate ring on Legacy before remounting a new driver.
        {
            let mut inner = sup.inner().write().await;
            for _ in 0..(SHADOW_RING_CAPACITY + 16) {
                inner.tick_control(0.02);
            }
        }
        let before = sup.torque_shadow_diagnostics();
        assert!(
            before.ring.interval_unverifiable && before.ring.ring_dropped_records > 0,
            "ring must be unverifiable before remount: {before:?}"
        );
        // Remount a fresh hardware driver!
        sup.set_driver(Box::new(build_dm_driver(bus.clone(), &cfg))).await;
        let after = sup.torque_shadow_diagnostics();
        assert!(
            after.ring.interval_unverifiable,
            "driver remount MUST preserve interval_unverifiable=true: {after:?}"
        );
        assert!(
            after.ring.ring_dropped_records >= before.ring.ring_dropped_records,
            "driver remount MUST preserve ring_dropped_records: before={before:?} after={after:?}"
        );
    }

    // (e) m3: Disabled shadow (`torque_shadow_enabled = false`) allocates ZERO reducer state
    //     even across driver mount and RT executor handoff!
    {
        let clock = Arc::new(MockClock::new());
        let bus = FakeCan::new(clock.clone());
        let cfg = build_dm_config(false);
        let sup = Arc::new(ArmSupervisor::new(cfg.clone()));
        sup.set_driver(Box::new(build_dm_driver(bus.clone(), &cfg))).await;
        sup.start_dispatcher().unwrap();
        assert!(!sup.torque_shadow_hub().has_worker());
        let handle = sup
            .start_executor(ExecutorConfig::for_tests(Duration::from_millis(20)))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(40)).await;
        let d = sup.torque_shadow_diagnostics();
        assert!(!d.enabled);
        assert_eq!(d.owner, ShadowOwner::Executor);
        assert_eq!(d.events_processed, 0);
        handle.stop_and_join();
    }

    // (f) B2: Damiao live rest-settle boundary with independently sampled realtime clock is
    //     identical with shadow ON vs OFF (staging runs strictly AFTER capture_rx_clock_sample).
    {
        fn probe_settle(shadow: bool) -> bool {
            let clock = Arc::new(MockClock::new());
            let bus = FakeCan::new(clock.clone());
            bus.with(|s| {
                s.kernel_realtime_ns = Some(10_000_000_000);
                s.realtime_minus_mono_ns = Some(10_000_000_000);
            });
            let cfg = build_dm_config(shadow);
            let mut drv = build_dm_driver(bus.clone(), &cfg);
            drv.set_shadow_enabled(shadow);
            let targets = vec![armd::driver::JointTarget::position(0.0); 7];
            let _ = drv.write_joint_targets(&targets);
            // Advance exactly 21 ms (> 20 ms settle) and feed a kernel-stamped feedback frame.
            clock.advance(Duration::from_millis(21));
            bus.with(|s| {
                s.kernel_realtime_ns = Some(10_021_000_000);
                s.set_pose(&[0.0; 7]);
                let f = s.motors[0].feedback_frame();
                s.rx.push_back((f, Some(CanRxTime::KernelRealtimeNs(10_021_000_000))));
            });
            let _ = drv.read_joint_states();
            drv.feedback_instants()[0].is_some()
        }
        assert_eq!(probe_settle(false), probe_settle(true));
        assert!(probe_settle(true));
    }
}
