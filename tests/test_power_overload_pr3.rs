//! PR 3 tests: Static overload alarm in `power_tick`, stale-feedback persistence/preservation,
//! independent `observation.power.severity` alongside `torque_guard` (`motion_limit.reason == torque`),
//! load-lowering-only restriction across servo, jog, Cartesian, and replay under BOTH owners,
//! and cockpit `#pwr-alarm` banner wiring.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{ArmDriver, ArmDriverError, DriverCapabilities, JointTarget};
use armd::model::{ArmConfig, JointState, MotionLimitReason};
use armd::motion::{self, MotionError, Speed};
use armd::power::{
    Estimate, PowerBudget, PowerMonitor, StaticOverloadSeverity, MAX_JOINTS, STATIC_CLEAR_MS,
    STATIC_HARD_MS, STATIC_TRIP_MS, STATIC_VELOCITY_RAD_S,
};
use armd::recording::EpisodeStore;
use armd::replay::{plan_frames, ReplayInput, ReplayRequest};
use armd::rt_core::broker::{BootId, Capacities, ClientId, Reply, Submit};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetAuthority, TargetFrame, TargetOutcome};
use armd::rt_core::ops::{CoreOps, PlanSlot, ReleaseFixture, VirtualClock};
use armd::rt_core::requests::motion::{
    prepare_motion, stage_jog, MotionConfigSnapshot, MotionPlanSnapshot, MotionRequest,
    PlanAuthority,
};
use armd::safety::SafetyError;
use armd::supervisor::{ArmError, SupervisorInner};
use armd::RecordFrame;

const DT: f64 = 0.02;
const BOOT: BootId = BootId(0xB601_9003);

struct FakeStaticState {
    q: Vec<f32>,
    v: Vec<f32>,
    tau: Vec<f32>,
    rx_at: Option<Instant>,
    torqued: bool,
}

struct FakeStaticDriver {
    state: Arc<Mutex<FakeStaticState>>,
}

impl FakeStaticDriver {
    fn new(q: Vec<f32>, tau: Vec<f32>) -> (Self, Arc<Mutex<FakeStaticState>>) {
        let n = q.len();
        let st = Arc::new(Mutex::new(FakeStaticState {
            q,
            v: vec![0.0; n],
            tau,
            rx_at: Some(Instant::now()),
            torqued: false,
        }));
        (Self { state: Arc::clone(&st) }, st)
    }
}

impl ArmDriver for FakeStaticDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            temperature_feedback: true,
            load_feedback: false,
            torque_feedback: true,
            velocity_feedback: true,
            torque_feedforward: false,
            enable_feedback: false,
        }
    }

    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let st = self.state.lock().unwrap();
        let n = st.q.len();
        Ok((0..n)
            .map(|i| JointState {
                position: st.q[i],
                velocity: st.v[i],
                torque: st.tau[i],
                temperature: 30.0,
            })
            .collect())
    }

    fn write_joint_targets(&mut self, _targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        self.state.lock().unwrap().torqued = enable;
        Ok(())
    }

    fn is_feedback_valid(&self) -> bool {
        self.state.lock().unwrap().rx_at.is_some()
    }

    fn is_enabled(&self) -> bool {
        self.state.lock().unwrap().torqued
    }

    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let st = self.state.lock().unwrap();
        let age = st.rx_at.map(|t| t.elapsed());
        vec![age; st.q.len()]
    }

    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        let st = self.state.lock().unwrap();
        vec![st.rx_at; st.q.len()]
    }
}

fn estimate_of(bus_a: f32) -> Estimate {
    let mut joint_a = [0.0; MAX_JOINTS];
    joint_a[0] = bus_a;
    Estimate { bus_a, regen_w: 0.0, joint_a, n: 7 }
}

fn config_no_pr2_cap() -> ArmConfig {
    let mut cfg = ArmConfig::rebot_dm();
    let b = cfg.power_budget.as_mut().expect("B601-DM power_budget");
    b.static_admission_fraction = None;
    b.static_reserve_a = 0.0;
    cfg.workspace = None;
    cfg.lease_timeout_ms = 600_000;
    cfg.validate().expect("valid config");
    cfg
}

#[test]
fn static_overload_alarm_persistence_hysteresis_and_stale_handling() {
    let budget = PowerBudget::rebot_dm();
    let dt = DT;
    let zeros = [0.0f32; 7];
    let moving = [0.0, STATIC_VELOCITY_RAD_S + 0.01, 0.0, 0.0, 0.0, 0.0, 0.0];

    let hard_a = budget.governor.hard * budget.supply_rated_a + 0.1; // ~6.275 A
    let trip_a = budget.supply_trip_a + 0.76; // ~8.56 A
    let soft_low_a = budget.governor.soft * budget.supply_rated_a - 1.0; // ~4.2 A

    let mut mon = PowerMonitor::default();
    let mut t = 0.0;

    // 1. Moving faster than static_velocity_rad_s does NOT trigger Warning or Trip.
    for _ in 0..20 {
        t += dt;
        mon.update(&budget, &estimate_of(trip_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &moving, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::None);
    assert!(!mon.is_static_restricted());

    // 2. Stationary above hard (below trip): stays None before 200 ms, stale breaks persistence.
    let pre_hard_ticks = (STATIC_HARD_MS / 20) - 2; // 8 ticks = 160 ms
    for _ in 0..pre_hard_ticks {
        t += dt;
        mon.update(&budget, &estimate_of(hard_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::None);
    mon.stale(dt as f32);
    for _ in 0..pre_hard_ticks {
        t += dt;
        mon.update(&budget, &estimate_of(hard_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::None, "stale must break persistence");
    for _ in 0..3 {
        t += dt;
        mon.update(&budget, &estimate_of(hard_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Warning);
    assert!(mon.is_static_restricted());

    // 3. Escalate to Trip after 100 ms stationary >= supply_trip_a.
    let mut ticks_to_trip = 0;
    for _ in 0..25 {
        t += dt;
        mon.update(&budget, &estimate_of(trip_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
        ticks_to_trip += 1;
        if mon.static_severity == StaticOverloadSeverity::Trip {
            break;
        }
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Trip);
    assert!(ticks_to_trip >= (STATIC_TRIP_MS / 20) as usize);

    // 4. Stale samples and full invalidation MUST NOT clear active Trip restriction.
    for _ in 0..30 {
        mon.stale(dt as f32);
    }
    assert!(!mon.valid, "estimate invalidated after STALE_HOLD_S");
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Trip);
    assert!(mon.is_static_restricted());

    // 5. Attempted motion while still above soft keeps Trip active.
    for _ in 0..10 {
        t += dt;
        mon.update(&budget, &estimate_of(hard_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &moving, &moving);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Trip);

    // 6. Below soft for < 300 ms interrupted by stale stays Trip; continuous 300 ms clears.
    let clear_ticks = (STATIC_CLEAR_MS / 20) as usize; // 15 ticks
    mon.invalidate();
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Trip);
    for _ in 0..(clear_ticks - 2) {
        t += dt;
        mon.update(&budget, &estimate_of(soft_low_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Trip);
    mon.stale(dt as f32);
    for _ in 0..(clear_ticks - 2) {
        t += dt;
        mon.update(&budget, &estimate_of(soft_low_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::Trip);
    for _ in 0..3 {
        t += dt;
        mon.update(&budget, &estimate_of(soft_low_a), dt as f32, t);
        mon.step_static_overload(&budget, dt, true, false, &zeros, &zeros);
    }
    assert_eq!(mon.static_severity, StaticOverloadSeverity::None);
    assert!(!mon.is_static_restricted());
}

#[test]
fn dual_owner_static_overload_trip_alarm_independent_of_torque_limit_and_restricts_all_sources() {
    let cfg = config_no_pr2_cap();
    let budget = cfg.power_budget.clone().unwrap();
    // Vertical upright (lower) < folded horizontal (mid) < unfolded forward (worse).
    let mid_q = [0.0f32, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
    let worse_q = [0.0f32, -0.2, 2.0, 0.0, 0.0, 0.0, 0.0];
    let lower_q = [0.0f32, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0];
    let worse_cart_q = [0.0f32, -0.4, 0.0, 0.0, 0.0, 0.0, 0.0];
    let lower_cart_q = [0.0f32, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0];

    // Distribute ~8.7 A across J2, J3, J4 while keeping each joint strictly below its
    // torque_guard threshold (16.8 / 16.8 / 6.0 N·m) so only the static overload alarm trips.
    let trip_torques = vec![0.0f32, 15.5, 15.5, 5.8, 0.0, 0.0, 0.0];

    // --- Owner 1: Legacy SupervisorInner ---
    {
        let (drv, st) = FakeStaticDriver::new(mid_q.to_vec(), trip_torques.clone());
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        let _ = inner.tick(DT);
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(adm).unwrap();
        inner.commanded_positions = mid_q.to_vec();
        inner.measured_positions = mid_q.to_vec();
        inner.target_positions = mid_q.to_vec();

        // Capture a pre-restriction PlanInput & lower-load Cartesian path before alarm triggers.
        let pre_plan_input = inner.plan_input().unwrap();
        assert!(!pre_plan_input.static_restricted);
        let fk_pre_lower = cfg.arm_model.fk(&std::array::from_fn::<f64, 6, _>(|i| f64::from(lower_cart_q[i])));
        let pre_cart_path = motion::plan_cartesian(&pre_plan_input, fk_pre_lower, Speed::default())
            .expect("pre-restriction Cartesian path plans cleanly");

        // Drive stationary 8.6 A for 0.3 s (15 ticks at 50 Hz) -> Trip latches.
        for _ in 0..15 {
            st.lock().unwrap().rx_at = Some(Instant::now());
            let _ = inner.tick(DT);
        }
        let obs = inner.current_observation();
        let p = obs.power.as_ref().expect("observation.power present");
        assert_eq!(p.severity, StaticOverloadSeverity::Trip);
        assert!(p.static_restricted);
        // Alarm only: keep hold, no automatic motion or torque-off.
        assert!(inner.is_torqued);
        assert!(inner.safety.state().is_operational());
        assert!(inner.active_trajectory.is_none());
        assert_eq!(inner.commanded_positions, mid_q);

        // Even while torque_guard sets motion_limit.reason == Torque (higher priority than Power),
        // observation.power.severity MUST remain published independently.
        inner.note_motion_limit(MotionLimitReason::Torque, Some(3));
        let obs2 = inner.current_observation();
        assert_eq!(obs2.motion_limit.as_ref().map(|m| m.reason), Some(MotionLimitReason::Torque));
        assert_eq!(obs2.power.as_ref().map(|p| p.severity), Some(StaticOverloadSeverity::Trip));

        // 1) Servo: worsening rejected with STATIC_LOAD_LIMIT; lowering admitted.
        let ep = inner.safety.epoch_id();
        let s = inner.current_admission_stamp();
        let err = inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::servo_slice(a, &worse_q, ep))
            .unwrap_err();
        assert_eq!(err, SafetyError::StaticLoadLimit);
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::servo_slice(a, &lower_q, ep))
            .expect("load-lowering servo admitted under static restriction");
        inner.stop_motion();
        inner.commanded_positions = mid_q.to_vec();
        inner.commanded_velocities.fill(0.0);

        // 2) Discrete Jog: worsening rejected with STATIC_LOAD_LIMIT; lowering admitted.
        let snap = MotionPlanSnapshot::from_supervisor(&inner);
        assert!(snap.static_restricted);
        let jog_worse = prepare_motion(&snap, &MotionRequest::jog(worse_q.to_vec(), Some(0.5), None));
        assert!(matches!(jog_worse, Err(ArmError::Motion(MotionError::StaticLoadLimit))), "got {jog_worse:?}");
        let jog_lower = prepare_motion(&snap, &MotionRequest::jog(lower_q.to_vec(), Some(0.5), None));
        assert!(jog_lower.is_ok(), "load-lowering jog should plan: {jog_lower:?}");

        // 3) Cartesian path: pre-restriction PlanInput refused as PLAN_STALE; restricted worsening refused as STATIC_LOAD_LIMIT; lowering allowed.
        let s = inner.current_admission_stamp();
        let adm = inner.check_admission(s).unwrap();
        let stale_err = SupervisorInner::install_path(adm, &pre_plan_input, pre_cart_path, "move_to", None).unwrap_err();
        assert!(matches!(stale_err, ArmError::Motion(MotionError::PlanStale(_))), "got {stale_err:?}");
        let cur_input = inner.plan_input().unwrap();
        assert!(cur_input.static_restricted);
        let fk_worse = cfg.arm_model.fk(&std::array::from_fn::<f64, 6, _>(|i| f64::from(worse_cart_q[i])));
        let fk_lower = cfg.arm_model.fk(&std::array::from_fn::<f64, 6, _>(|i| f64::from(lower_cart_q[i])));
        let cart_worse = motion::plan_cartesian(&cur_input, fk_worse, Speed::default());
        assert!(matches!(cart_worse, Err(MotionError::StaticLoadLimit)), "got {cart_worse:?}");
        let cart_lower = motion::plan_cartesian(&cur_input, fk_lower, Speed::default());
        assert!(cart_lower.is_ok(), "load-lowering Cartesian move should plan: {cart_lower:?}");

        // 4) Replay: worsening episode refused with STATIC_LOAD_LIMIT.
        let dir = tempfile::tempdir().unwrap();
        let store = EpisodeStore::new(Some(dir.path().to_path_buf()));
        let frames = vec![
            RecordFrame { timestamp: 0.0, observation_state: mid_q.to_vec(), action: mid_q.to_vec() },
            RecordFrame { timestamp: 0.5, observation_state: worse_cart_q.to_vec(), action: worse_cart_q.to_vec() },
        ];
        let replay_in = ReplayInput {
            request: ReplayRequest { episode: "ep_worse".into(), speed: 1.0, gripper: false, tag: None, allow_stale: false },
            path: dir.path().join("ep.parquet"),
            profile_id: cfg.profile_id.clone(),
            dof: cfg.dof,
            arm_dof: cfg.arm_dof(),
            gripper_idx: cfg.gripper_index(),
            joint_names: cfg.limits.iter().map(|l| l.name.clone()).collect(),
            limits: cfg.limits.clone(),
            speed_limits: cfg.limits.clone(),
            ws: None,
            arm: cfg.arm_model.clone(),
            start: mid_q.to_vec(),
            control_hz: 50,
            epoch: ep,
            workspace_revision: 0,
            power: Some(budget.clone()),
            static_restricted: true,
            store,
            zero_offsets: vec![0.0; 7],
            tcp_offset: cfg.tcp_offset,
            active_tool: cfg.active_tool.clone(),
        };
        let replay_err = plan_frames(replay_in, &frames).unwrap_err();
        match replay_err {
            ArmError::Rejected { status: 422, code, .. } => assert_eq!(code, "STATIC_LOAD_LIMIT"),
            other => panic!("expected 422 STATIC_LOAD_LIMIT on worsening replay, got {other:?}"),
        }
    }

    // --- Owner 2: RT Executor (`ReleaseFixture`) ---
    {
        let (drv, st) = FakeStaticDriver::new(mid_q.to_vec(), trip_torques);
        let mut inner = SupervisorInner::new(cfg.clone()).with_driver(Box::new(drv));
        inner.lease_enabled = false;
        inner.rt_executor_owned = true;
        let _ = inner.tick(DT);
        let stamp = inner.current_admission_stamp();
        let adm = inner.check_admission(stamp).unwrap();
        SupervisorInner::enable_torque(adm).unwrap();
        inner.commanded_positions = mid_q.to_vec();
        inner.measured_positions = mid_q.to_vec();
        inner.target_positions = mid_q.to_vec();
        inner.register_live_connection(1).unwrap();
        let stream = StreamId(1);
        inner.core.authorize_stream(TargetAuthority {
            stream: Some(stream),
            ..Default::default()
        });
        let mcfg = Arc::new(MotionConfigSnapshot::from_supervisor(&inner));
        let ep = inner.safety.epoch_id();

        let (mut fx, mut broker, mut tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 2, replies: 16 },
            VirtualClock::starting_at(Instant::now()),
        );
        // Prepare a worsening jog BEFORE static restriction triggers (allowed since PR 2 cap is None).
        let pre_slot = prepare_motion(
            &MotionPlanSnapshot {
                config: Arc::clone(&mcfg),
                commanded_positions: mid_q.to_vec(),
                commanded_velocities: vec![0.0; 7],
                measured_positions: mid_q.to_vec(),
                torque_hold: None,
                power_hold: None,
                static_restricted: false,
                holding_after_clear: false,
                mit_active: false,
                authority: PlanAuthority {
                    boot: BOOT,
                    epoch: ep,
                    lease_generation: 0,
                    stop_generation: 0,
                    deadline: fx.now() + Duration::from_secs(30),
                },
                workspace_revision: mcfg.workspace_revision,
                motion_config_revision: mcfg.motion_config_revision,
            },
            &MotionRequest::jog(worse_q.to_vec(), Some(0.5), None),
        )
        .expect("pre-restriction jog prepares when PR 2 cap is None")
        .slot;

        // Drive stationary 8.6 A for 0.32 s -> Trip latches on RT owner.
        for _ in 0..16 {
            st.lock().unwrap().rx_at = Some(Instant::now());
            let _ = fx.step();
        }
        let obs = fx.core.current_observation();
        let p = obs.power.as_ref().expect("RT observation.power");
        assert_eq!(p.severity, StaticOverloadSeverity::Trip);
        assert!(p.static_restricted);
        assert!(fx.core.power.is_static_restricted());
        assert!(fx.core.is_torqued);
        assert_eq!(fx.core.commanded_positions, mid_q);

        // 1) RT servo mailbox: worsening rejected with STATIC_LOAD_LIMIT; lowering accepted.
        let auth = fx.core.authority(&fx.ctx(fx.now()));
        let make_frame = |seq: u64, pos: [f32; 7], now: Instant| -> TargetFrame {
            let mut q = [0.0f32; 8];
            q[..7].copy_from_slice(&pos);
            TargetFrame {
                arm: ArmId::LOCAL,
                boot: BOOT,
                stream,
                seq,
                authority: auth,
                policy_generation: 0,
                input_at: now,
                vision_at: None,
                vision_frame_id: None,
                deadline: now + Duration::from_secs(5),
                dof: 7,
                q,
                replaceable: true,
            }
        };
        st.lock().unwrap().rx_at = Some(Instant::now());
        let _ = tw.submit(make_frame(1, worse_q, fx.now()));
        let _ = fx.step();
        let mut outcomes: Vec<TargetOutcome> = Vec::new();
        tw.outcomes(|o| outcomes.push(o));
        assert_eq!(outcomes.len(), 1);
        assert_eq!(outcomes[0].result.unwrap_err().code, "STATIC_LOAD_LIMIT");

        // 2) Submitting pre-restriction worsening jog slot to RT broker -> refused as PLAN_STALE.
        let req_stale = broker
            .submit(
                ClientId(1),
                1,
                auth,
                fx.now() + Duration::from_secs(10),
                Submit::Install(&move |s: &mut PlanSlot| stage_jog(s, &pre_slot)),
            )
            .expect("submit pre-restriction jog");
        st.lock().unwrap().rx_at = Some(Instant::now());
        let _ = fx.step();
        let mut replies = Vec::new();
        broker.poll(|_, r| { replies.push(r); true });
        let stale_reply = replies
            .iter()
            .find(|r| matches!(r, Reply::Refused { request, .. } | Reply::Ack { request, .. } if *request == req_stale))
            .expect("stale reply");
        assert!(
            matches!(stale_reply, Reply::Refused { code: "PLAN_STALE", .. }),
            "pre-restriction slot must be refused as PLAN_STALE, got {stale_reply:?}"
        );

        // 3) Re-preparing under live RT state: worsening fails STATIC_LOAD_LIMIT, lowering commits!
        let live_snap = MotionPlanSnapshot::from_supervisor(&fx.core)
            .with_authority(PlanAuthority {
                boot: BOOT,
                epoch: fx.core.safety.epoch_id(),
                lease_generation: 0,
                stop_generation: auth.stop_generation,
                deadline: fx.now() + Duration::from_secs(30),
            });
        assert!(live_snap.static_restricted);
        let rt_worse = prepare_motion(&live_snap, &MotionRequest::jog(worse_q.to_vec(), Some(0.5), None));
        assert!(matches!(rt_worse, Err(ArmError::Motion(MotionError::StaticLoadLimit))), "got {rt_worse:?}");
        let lower_slot = prepare_motion(&live_snap, &MotionRequest::jog(lower_q.to_vec(), Some(0.5), None))
            .expect("load-lowering RT jog prepares")
            .slot;
        assert!(lower_slot.static_restricted);
        let req_ok = broker
            .submit(
                ClientId(1),
                2,
                auth,
                fx.now() + Duration::from_secs(10),
                Submit::Install(&move |s: &mut PlanSlot| stage_jog(s, &lower_slot)),
            )
            .expect("submit load-lowering jog");
        st.lock().unwrap().rx_at = Some(Instant::now());
        let _ = fx.step();
        let mut replies2 = Vec::new();
        broker.poll(|_, r| { replies2.push(r); true });
        let ok_reply = replies2
            .iter()
            .find(|r| matches!(r, Reply::Refused { request, .. } | Reply::Ack { request, .. } if *request == req_ok))
            .expect("ok reply");
        assert!(
            matches!(ok_reply, Reply::Ack { .. }),
            "load-lowering RT jog should be accepted, got {ok_reply:?}"
        );
    }
}

#[test]
fn cockpit_static_overload_banner_independent_of_motion_limit() {
    let html = include_str!("../src/api/index.html");
    assert!(html.contains(r#"id="pwr-alarm""#), "missing #pwr-alarm banner");
    assert!(html.contains(r#"role="alert""#), "missing alert role");
    assert!(html.contains("'pwr.alarm.warn'"), "missing zh/en warn key");
    assert!(html.contains("'pwr.alarm.trip'"), "missing zh/en trip key");
    assert!(html.contains("'pwr.alarm.sub'"), "missing zh/en sub key");
    assert!(
        html.contains("p.severity === 'trip' || p.severity === 'warning'"),
        "renderPower must read observation.power.severity directly"
    );
}
