//! PR 2 tests: static-load admission (`STATIC_LOAD_LIMIT`), baseline caching, intermediate
//! shaped-increment recheck, discrete jog on both owners, policy stream refusal, and host
//! microbench for the 4-gravity-solve per-release budget.

use std::time::{Duration, Instant};

use armd::chain::ArmModel;
use armd::motion::MotionError;
use armd::power::{
    check_static_admission_values, predict_static_holding_current_a, PowerBudget,
    STATIC_LOAD_LIMIT_MSG,
};
use armd::rt_core::broker::{BootId, Capacities, Outcome};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetFrame, TargetOutcome};
use armd::rt_core::ops::{CoreOps, ReleaseFixture, VirtualClock};
use armd::rt_core::requests::motion::{
    preflight_motion_request, prepare_motion, MotionRequest,
};
use armd::safety::SafetyError;
use armd::{ArmConfig, SupervisorInner};

const DT: f64 = 0.02;
const BOOT: BootId = BootId(0xB601_9002);

fn armed_legacy(mut cfg: ArmConfig, q0: [f32; 7]) -> SupervisorInner {
    cfg.lease_timeout_ms = 600_000;
    cfg.workspace = None;
    let mut inner = SupervisorInner::new(cfg);
    inner.lease_enabled = false;
    inner.commanded_positions = q0.to_vec();
    inner.measured_positions = q0.to_vec();
    inner.target_positions = q0.to_vec();
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::enable_torque(a))
            .unwrap();
    }
    assert!(inner.safety.state().is_operational());
    inner
}

#[test]
fn static_admission_config_defaults_and_validation() {
    let def = PowerBudget::default();
    assert_eq!(def.static_admission_fraction, None);
    assert_eq!(def.static_reserve_a, 0.0);
    assert_eq!(def.static_admission_cap_a(), None);

    let kit = PowerBudget::rebot_dm();
    assert_eq!(kit.static_admission_fraction, Some(0.85));
    assert_eq!(kit.static_reserve_a, 0.0);
    let cap = kit.static_admission_cap_a().unwrap();
    assert!((cap - 5.525).abs() < 1e-4, "expected ~5.53 A, got {cap}");
    kit.validate(7).unwrap();

    // Invalid fraction values
    let mut bad = kit.clone();
    bad.static_admission_fraction = Some(0.0);
    assert!(bad.validate(7).is_err());
    bad.static_admission_fraction = Some(1.3);
    assert!(bad.validate(7).is_err());
    bad.static_admission_fraction = Some(f32::NAN);
    assert!(bad.validate(7).is_err());
    // frac * rated > trip
    bad.supply_trip_a = 6.6;
    bad.static_admission_fraction = Some(1.1);
    assert!(bad.validate(7).is_err());

    // Negative / nonfinite reserve
    let mut bad_res = kit;
    bad_res.static_reserve_a = -0.1;
    assert!(bad_res.validate(7).is_err());
    bad_res.static_reserve_a = f32::INFINITY;
    assert!(bad_res.validate(7).is_err());
}

#[test]
fn nonfinite_pose_or_reserve_is_rejected_explicitly() {
    let mut b = PowerBudget::rebot_dm();
    let arm = ArmModel::REBOT_DEV_ARM;
    assert!(predict_static_holding_current_a(&b, &arm, &[f32::NAN, 0.0, 0.0, 0.0, 0.0, 0.0]).is_err());
    assert!(predict_static_holding_current_a(&b, &arm, &[0.0, f32::INFINITY, 0.0, 0.0, 0.0, 0.0]).is_err());
    b.static_reserve_a = f32::NAN;
    assert!(predict_static_holding_current_a(&b, &arm, &[0.0; 6]).is_err());
    assert!(check_static_admission_values(f32::NAN, 1.0, 5.0, false).is_err());
    assert!(check_static_admission_values(1.0, f32::INFINITY, 5.0, false).is_err());
    assert!(check_static_admission_values(1.0, 1.0, f32::NAN, false).is_err());
}

#[test]
fn directional_admission_dual_owner_servo_and_absent_config_disabled() {
    let arm = ArmModel::REBOT_DEV_ARM;
    // Vertical upright (low gravity torque) -> folded horizontal (mid) -> unfolded horizontal forward (high).
    let q_low = [0.0, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0];
    let q_mid = [0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0];
    let q_stretch = [0.0, -0.2, 2.2, 0.0, 0.0, 0.0, 0.0];

    let kit_budget = PowerBudget::rebot_dm();
    let i_rest = predict_static_holding_current_a(&kit_budget, &arm, &q_low).unwrap();
    let i_mid = predict_static_holding_current_a(&kit_budget, &arm, &q_mid).unwrap();
    let i_stretch = predict_static_holding_current_a(&kit_budget, &arm, &q_stretch).unwrap();
    assert!(
        i_stretch > i_mid && i_mid > i_rest,
        "expected monotonic load: low={i_rest:.3} mid={i_mid:.3} stretch={i_stretch:.3}"
    );
    let q_rest = q_low;

    // 1. Absent static_admission_fraction -> disabled (allows everything).
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.power_budget.as_mut().unwrap().static_admission_fraction = None;
        cfg.power_budget.as_mut().unwrap().static_reserve_a = 10.0;
        let mut inner = armed_legacy(cfg, q_rest);
        let ep = inner.safety.epoch_id();
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::servo_slice(a, &q_stretch, ep))
            .expect("absent static_admission_fraction must keep admission disabled");
    }

    // Set reserve so: rest < cap < mid < stretch (cap = 0.85 * 6.5 = 5.525 A).
    let cap = kit_budget.static_admission_cap_a().unwrap();
    let reserve = cap - 0.5 * (i_rest + i_mid);
    assert!(i_rest + reserve < cap);
    assert!(i_mid + reserve > cap);
    assert!(i_stretch + reserve > i_mid + reserve);

    // 2. Legacy owner: below cap -> worsening above cap rejected; above cap -> worsening rejected, lowering accepted.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.power_budget.as_mut().unwrap().static_reserve_a = reserve;
        let mut inner = armed_legacy(cfg.clone(), q_rest);
        let ep = inner.safety.epoch_id();

        // Crossing above cap from rest -> rejected with STATIC_LOAD_LIMIT & exact message.
        let s = inner.current_admission_stamp();
        let err = inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::servo_slice(a, &q_mid, ep))
            .unwrap_err();
        assert_eq!(err, SafetyError::StaticLoadLimit);
        assert_eq!(err.to_string(), STATIC_LOAD_LIMIT_MSG);
        assert!(inner.servo_target.is_none(), "refused update must not leave a target active");

        // Starting already above cap (q_mid): worsening to q_stretch is refused; lowering to q_rest is accepted.
        let mut inner_hi = armed_legacy(cfg, q_mid);
        let ep_hi = inner_hi.safety.epoch_id();
        let s = inner_hi.current_admission_stamp();
        assert_eq!(
            inner_hi
                .check_admission(s)
                .and_then(|a| SupervisorInner::servo_slice(a, &q_stretch, ep_hi))
                .unwrap_err(),
            SafetyError::StaticLoadLimit
        );
        // Load-lowering command passes.
        let s = inner_hi.current_admission_stamp();
        inner_hi
            .check_admission(s)
            .and_then(|a| SupervisorInner::servo_slice(a, &q_rest, ep_hi))
            .expect("load-lowering target must pass");
    }

    // 3. RT executor owner: identical refusal code and directional behavior.
    {
        let mut cfg = ArmConfig::rebot_dm();
        cfg.power_budget.as_mut().unwrap().static_reserve_a = reserve;
        let mut inner = armed_legacy(cfg, q_mid);
        inner.rt_executor_owned = true;
        inner.register_live_connection(1).unwrap();
        let stream = StreamId(1);
        inner.core.authorize_stream(armd::rt_core::mailbox::TargetAuthority {
            stream: Some(stream),
            ..Default::default()
        });
        let (mut fx, _broker, mut tw, _stop) = ReleaseFixture::new_with_clock(
            inner,
            BOOT,
            Capacities { commands: 8, plans: 2, replies: 16 },
            VirtualClock::starting_at(Instant::now()),
        );
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
        // Worsening target q_stretch -> STATIC_LOAD_LIMIT.
        let _ = tw.submit(make_frame(1, q_stretch, fx.now()));
        let _ = fx.step();
        let mut outcomes: Vec<TargetOutcome> = Vec::new();
        tw.outcomes(|o| outcomes.push(o));
        assert_eq!(outcomes.len(), 1, "expected 1 RT outcome: {outcomes:?}");
        assert_eq!(
            outcomes[0].result.unwrap_err().code,
            "STATIC_LOAD_LIMIT",
            "expected RT STATIC_LOAD_LIMIT refusal"
        );
        assert!(fx.core.servo_target.is_none(), "RT refused update must not leave target active");

        // Load-lowering target q_rest -> Accepted!
        outcomes.clear();
        let _ = tw.submit(make_frame(2, q_rest, fx.now()));
        let _ = fx.step();
        tw.outcomes(|o| outcomes.push(o));
        assert_eq!(outcomes.len(), 1);
        assert!(outcomes[0].result.is_ok(), "RT load-lowering target must be accepted: {:?}", outcomes[0]);
    }
}

#[test]
fn shaped_increment_recheck_catches_higher_load_intermediate_step() {
    let arm = ArmModel::REBOT_DEV_ARM;
    let kit_budget = PowerBudget::rebot_dm();
    let cap = kit_budget.static_admission_cap_a().unwrap();

    // Find two poses where start and end have lower static current than the linear midpoint
    // (swinging J2 through horizontal from one side of vertical to the other).
    // q_start = J2 +0.8, q_end = J2 -0.8 -> midpoint passes through J2 ~ 0 (or J2 -1.0 -> +1.0).
    let q_a = [0.0, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0];
    let q_b = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0];
    let i_a = predict_static_holding_current_a(&kit_budget, &arm, &q_a).unwrap();
    let i_b = predict_static_holding_current_a(&kit_budget, &arm, &q_b).unwrap();
    // Sample intermediate points along J2 to find the true peak.
    let mut peak_i = i_a.max(i_b);
    let mut peak_j2 = q_a[1];
    for step in 1..40 {
        let alpha = step as f32 / 40.0;
        let q = [0.0, q_a[1] + alpha * (q_b[1] - q_a[1]), 0.0, 0.0, 0.0, 0.0, 0.0];
        let cur = predict_static_holding_current_a(&kit_budget, &arm, &q).unwrap();
        if cur > peak_i {
            peak_i = cur;
            peak_j2 = q[1];
        }
    }
    assert!(
        peak_i > i_a.max(i_b) + 0.01,
        "expected intermediate peak along J2 sweep: i_a={i_a:.4}, i_b={i_b:.4}, peak={peak_i:.4} at j2={peak_j2:.3}"
    );

    // Choose reserve so BOTH q_a and q_b are below cap, while peak_i + reserve > cap!
    let reserve = cap - 0.5 * (i_a.max(i_b) + peak_i);
    assert!(i_a + reserve < cap);
    assert!(i_b + reserve < cap);
    assert!(peak_i + reserve > cap);

    let mut cfg = ArmConfig::rebot_dm();
    cfg.power_budget.as_mut().unwrap().static_reserve_a = reserve;
    // Start just before the peak (at q_a) and command q_b (both <= cap): intake accepts q_b,
    // and stepping toward q_b must halt BEFORE crossing above cap on the intermediate increment!
    let mut inner = armed_legacy(cfg, q_a);
    let ep = inner.safety.epoch_id();
    let s = inner.current_admission_stamp();
    inner
        .check_admission(s)
        .and_then(|a| SupervisorInner::servo_slice(a, &q_b, ep))
        .expect("both start and end are below cap, so intake admits q_b");

    let mut halted_before_peak = false;
    for _ in 0..200 {
        inner.tick(DT);
        let cur = predict_static_holding_current_a(inner.power_budget().unwrap(), &arm, &inner.commanded_positions).unwrap();
        assert!(
            cur <= cap + 1e-3,
            "shared tick must never commit an intermediate step above cap: cur={cur:.4} cap={cap:.4}"
        );
        if inner.servo_target.is_none() && (inner.commanded_positions[1] - q_b[1]).abs() > 0.1 {
            halted_before_peak = true;
            break;
        }
    }
    assert!(halted_before_peak, "shared tick recheck must halt the stream before the intermediate peak");
}

#[test]
fn discrete_jog_applies_static_admission_on_both_owners_and_keeps_dynamic_budget() {
    let arm = ArmModel::REBOT_DEV_ARM;
    let kit_budget = PowerBudget::rebot_dm();
    let cap = kit_budget.static_admission_cap_a().unwrap();

    let q_rest = [0.0, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0];
    let q_stretch = [0.0, -0.2, 2.2, 0.0, 0.0, 0.0, 0.0];
    let i_rest = predict_static_holding_current_a(&kit_budget, &arm, &q_rest).unwrap();
    let i_stretch = predict_static_holding_current_a(&kit_budget, &arm, &q_stretch).unwrap();
    assert!(i_stretch > i_rest + 0.5, "expected stretched pose much heavier: rest={i_rest:.3} stretch={i_stretch:.3}");
    let reserve = cap - 0.5 * (i_rest + i_stretch);

    let mut cfg = ArmConfig::rebot_dm();
    cfg.power_budget.as_mut().unwrap().static_reserve_a = reserve;
    // Test even when dynamic planning is disabled: static admission must still refuse jog!
    cfg.power_budget.as_mut().unwrap().planning.enabled = false;

    // 1. Legacy owner jog: worsening refused with STATIC_LOAD_LIMIT.
    let mut inner = armed_legacy(cfg.clone(), q_rest);
    let ep = inner.safety.epoch_id();
    let s = inner.current_admission_stamp();
    let admitted = inner.check_admission(s).unwrap();
    let err = SupervisorInner::jog(admitted, q_stretch.to_vec(), None, Some(ep)).unwrap_err();
    match err {
        armd::ArmError::Motion(MotionError::StaticLoadLimit) => {}
        other => panic!("expected MotionError::StaticLoadLimit on legacy jog, got {other:?}"),
    }

    // 2. RT executor off-thread motion planner (`prepare_motion`): identical refusal!
    let jog_req = MotionRequest::jog(q_stretch.to_vec(), None, None);
    let s = inner.current_admission_stamp();
    let plan_snap = preflight_motion_request(&inner, s, &jog_req, Some(ep)).unwrap();
    let rt_err = prepare_motion(&plan_snap, &jog_req).unwrap_err();
    match rt_err {
        armd::ArmError::Motion(MotionError::StaticLoadLimit) => {}
        other => panic!("expected MotionError::StaticLoadLimit on RT jog, got {other:?}"),
    }

    // 3. Starting stretched: load-lowering jog back to rest is allowed on both owners (and Park is never refused).
    let mut inner_hi = armed_legacy(cfg.clone(), q_stretch);
    let ep_hi = inner_hi.safety.epoch_id();
    let s_hi = inner_hi.current_admission_stamp();
    let jog_back = MotionRequest::jog(q_rest.to_vec(), None, None);
    let plan_hi = preflight_motion_request(&inner_hi, s_hi, &jog_back, Some(ep_hi)).unwrap();
    prepare_motion(&plan_hi, &jog_back).expect("load-lowering jog must pass on RT planner");

    let park_req = MotionRequest::park();
    let s_park = inner_hi.current_admission_stamp();
    let plan_park = preflight_motion_request(&inner_hi, s_park, &park_req, Some(ep_hi)).unwrap();
    prepare_motion(&plan_park, &park_req).expect("park must never be refused by static admission");

    let s = inner_hi.current_admission_stamp();
    let admitted = inner_hi.check_admission(s).unwrap();
    SupervisorInner::jog(admitted, q_rest.to_vec(), None, Some(ep_hi))
        .expect("load-lowering jog must pass on legacy");
}

#[test]
fn host_microbench_four_gravity_solves_per_release() {
    let budget = PowerBudget::rebot_dm();
    let builtin = ArmModel::REBOT_DEV_ARM;
    // Also load the 6-joint URDF chain used on B601-DM.
    let mut v = serde_json::to_value(ArmConfig::rebot_dm()).unwrap();
    let o = v.as_object_mut().unwrap();
    o.insert("kinematics".into(), "Urdf".into());
    o.insert("gravity_model".into(), "Urdf".into());
    o.insert("urdf".into(), concat!(env!("CARGO_MANIFEST_DIR"), "/assets/reBot-DevArm_gripper.urdf").into());
    o.insert("urdf_sha256".into(), armd::gravity::EXPECTED_URDF_SHA256.into());
    o.insert("chain".into(), serde_json::json!({
        "base_link": "base_link",
        "tip_link": "end_link",
        "joints": armd::kinematics::B601DM_MODEL_OFFSETS_RAD.iter().enumerate()
            .map(|(i, off)| serde_json::json!({"urdf_joint": format!("joint{}", i + 1), "offset_rad": off})).collect::<Vec<_>>(),
    }));
    o.remove("model_offsets_rad");
    let mut urdf_cfg: ArmConfig = serde_json::from_value(v).unwrap();
    urdf_cfg.load_arm_model(None).unwrap();
    let urdf_arm = urdf_cfg.arm_model;

    let poses: [[f32; 7]; 4] = [
        [0.0, -0.4, -0.6, -0.3, 0.1, 0.0, 0.0],
        [0.2, -0.8, -0.9, -0.5, -0.2, 0.3, 0.0],
        [-0.3, -1.1, -0.5, -0.7, 0.4, -0.2, 0.0],
        [0.1, -0.6, -1.2, -0.2, -0.1, 0.5, 0.0],
    ];

    for (label, arm) in [("RebotDevArm_builtin", &builtin), ("URDF_6joint_chain", &urdf_arm)] {
        const ITERS: usize = 2000;
        let mut samples_ns = Vec::with_capacity(ITERS);
        let mut sink = 0.0f32;
        // Warm up
        for p in &poses {
            sink += predict_static_holding_current_a(&budget, arm, p).unwrap();
        }
        for iter in 0..ITERS {
            let t0 = Instant::now();
            // 4 gravity solves per release (worst-case uncached: intake ref+cand + increment ref+cand)
            for k in 0..4 {
                let p = &poses[(iter + k) & 3];
                sink += predict_static_holding_current_a(&budget, arm, p).unwrap();
            }
            samples_ns.push(t0.elapsed().as_nanos() as u64);
        }
        samples_ns.sort_unstable();
        let p50_us = samples_ns[ITERS / 2] as f64 / 1000.0;
        let p99_us = samples_ns[(ITERS * 99) / 100] as f64 / 1000.0;
        let max_us = samples_ns[ITERS - 1] as f64 / 1000.0;
        assert!(sink.is_finite());
        eprintln!(
            "MICROBENCH {label} (4 solves/release, N={ITERS}): p50={p50_us:.2} us, p99={p99_us:.2} us, max={max_us:.2} us"
        );
        // Under release optimization on host, 4 gravity solves must stay far below the 200 us
        // (0.2 ms) SG2002 added-p99 target. Debug builds run unoptimized under parallel test load.
        #[cfg(not(debug_assertions))]
        assert!(
            p99_us < 200.0,
            "{label} 4-solve p99 {p99_us:.2} us exceeded 200 us budget"
        );
    }
    let _ = Outcome::Completed;
}

#[test]
fn m5_rebound_above_cap_minor5_model_validation_and_m6_workspace_rollback() {
    let arm = ArmModel::REBOT_DEV_ARM;
    let kit_budget = PowerBudget::rebot_dm();
    let cap = kit_budget.static_admission_cap_a().unwrap();

    // M5: A path 7 A -> 6 A -> 6.5 A (above cap) must be rejected on the upward rebound step,
    // even though 6.5 A < 7 A start baseline!
    let q_hi = [0.0, -0.2, 2.2, 0.0, 0.0, 0.0, 0.0];
    let q_mid = [0.0, -0.5, 1.5, 0.0, 0.0, 0.0, 0.0];
    let q_low = [0.0, -0.9, 0.8, 0.0, 0.0, 0.0, 0.0];
    let i_hi = predict_static_holding_current_a(&kit_budget, &arm, &q_hi).unwrap();
    let i_mid = predict_static_holding_current_a(&kit_budget, &arm, &q_mid).unwrap();
    let i_low = predict_static_holding_current_a(&kit_budget, &arm, &q_low).unwrap();
    assert!(
        i_hi > i_mid + 0.1 && i_mid > i_low + 0.1,
        "expected descending static load: hi={i_hi:.3} mid={i_mid:.3} low={i_low:.3}"
    );
    let mut b_rebound = kit_budget.clone();
    b_rebound.static_reserve_a = cap - i_low + 0.25;
    assert!(i_low + b_rebound.static_reserve_a > cap);
    // Monotone lowering passes:
    armd::power::check_static_load_poses(
        &b_rebound,
        &arm,
        &q_hi,
        [q_mid.as_slice(), q_low.as_slice()],
        false,
    )
    .expect("monotone lowering above cap must pass");
    // Rebound hi -> low -> mid must FAIL (even though mid < hi!):
    let rebound_err = armd::power::check_static_load_poses(
        &b_rebound,
        &arm,
        &q_hi,
        [q_low.as_slice(), q_mid.as_slice()],
        false,
    );
    assert!(
        matches!(rebound_err, Err(MotionError::StaticLoadLimit)),
        "rebound above cap must be refused with StaticLoadLimit, got {rebound_err:?}"
    );

    // Minor 5: ArmConfig::validate rejects dof < arm_model.arm_dof() when static cap is enabled.
    let mut short_cfg = ArmConfig::default();
    short_cfg.dof = 4;
    short_cfg.joint_names = (1..=4).map(|i| format!("joint_{i}")).collect();
    short_cfg.limits.truncate(4);
    short_cfg.home_positions = vec![0.0; 4];
    let mut pb4 = PowerBudget::rebot_dm();
    pb4.joints.truncate(4);
    short_cfg.power_budget = Some(pb4);
    let val_err = short_cfg.validate().unwrap_err();
    assert!(
        val_err.contains("static_admission_fraction"),
        "expected static_admission_fraction dof mismatch error, got: {val_err}"
    );

    // M6: Refused servo step MUST NOT commit workspace escape clearance (`servo_ws_min_depths`).
    let mut ws_cfg = ArmConfig::rebot_dm();
    ws_cfg.power_budget.as_mut().unwrap().static_reserve_a = cap - 0.5 * (i_low + i_hi);
    if let Some(ws) = ws_cfg.workspace.as_mut() {
        ws.floor_z_m = 0.05;
    }
    let mut inner = armed_legacy(ws_cfg, q_low);
    // Ensure start pose has a known workspace state and `servo_ws_min_depths` is initially None.
    assert!(inner.servo_ws_min_depths.is_none());
    let ep = inner.safety.epoch_id();
    let s = inner.current_admission_stamp();
    // Force a servo_target directly that worsens static load above cap so the tick hits
    // `servo_step_workspace_rt` -> `check_static_load_admission` refusal.
    let _ = ep;
    let _ = s;
    inner.servo_target = Some(q_hi.to_vec());
    inner.tick(DT);
    assert!(
        inner.servo_ws_min_depths.is_none(),
        "M6: workspace clearance depths must NOT be committed when static-load admission refuses the step"
    );
}

#[test]
fn r2_m2_non_accumulating_static_load_eps_ceiling() {
    let mut budget = PowerBudget::rebot_dm();
    let arm = ArmConfig::rebot_dm().arm_model;
    let cap = budget.static_admission_cap_a().unwrap();

    // Find a monotone curve where J2 moves from -0.20 (lower load ~1.01 A) toward -0.80 (higher load ~1.71 A).
    let q_start = [0.0, -0.20, 1.5, 0.0, 0.0, 0.0, 0.0];
    let q_end = [0.0, -0.80, 1.5, 0.0, 0.0, 0.0, 0.0];
    let i_start = predict_static_holding_current_a(&budget, &arm, &q_start).unwrap();
    let i_end = predict_static_holding_current_a(&budget, &arm, &q_end).unwrap();
    assert!(i_end > i_start + 0.5, "expected J2 extension to increase load: {i_start:.3} -> {i_end:.3}");

    // Place start already above cap (`cap + 0.30 A`) so only non-worsening motion is legal.
    budget.static_reserve_a = cap - i_start + 0.30;
    let base_a = predict_static_holding_current_a(&budget, &arm, &q_start).unwrap();
    assert!(base_a > cap + 0.25);

    // Build 3,000 tiny upward increments (< 5e-5 A per step, total > 0.15 A creep).
    let steps = 3_000usize;
    let mut poses: Vec<[f32; 7]> = Vec::with_capacity(steps);
    for k in 1..=steps {
        let alpha = k as f32 / steps as f32;
        let mut q = q_start;
        q[1] = q_start[1] + (q_end[1] - q_start[1]) * alpha * 0.25;
        poses.push(q);
    }
    let final_a = predict_static_holding_current_a(&budget, &arm, poses.last().unwrap()).unwrap();
    assert!(
        final_a > base_a + 0.05,
        "creep path must represent real upward drift: base={base_a:.5} final={final_a:.5}"
    );
    // 1. Multi-pose trajectory check must reject cumulative creep!
    let pose_err = armd::power::check_static_load_poses(
        &budget,
        &arm,
        &q_start,
        poses.iter().map(|p| p.as_slice()),
        false,
    );
    assert!(
        matches!(pose_err, Err(MotionError::StaticLoadLimit)),
        "R2-M2: 3000-step +eps creep in check_static_load_poses must be rejected, got {pose_err:?}"
    );

    // 2. Tick-by-tick owner servo step commits must also stop before creeping > STATIC_LOAD_EPS_A!
    let mut cfg = ArmConfig::rebot_dm();
    cfg.power_budget = Some(budget.clone());
    let mut inner = armed_legacy(cfg, q_start);
    let mut max_accepted_a = base_a;
    let mut refused = false;
    for q in &poses {
        // Drive servo_target directly to exercise the tick-by-tick admission commit ceiling.
        inner.servo_target = Some(q.to_vec());
        inner.tick(DT);
        let cur_a = predict_static_holding_current_a(&budget, &arm, &inner.commanded_positions).unwrap();
        if cur_a > max_accepted_a {
            max_accepted_a = cur_a;
        }
        if inner.servo_target.is_none() {
            refused = true;
            break;
        }
    }
    assert!(refused, "R2-M2: tick-by-tick creep must trigger static-load refusal");
    assert!(
        max_accepted_a <= base_a + armd::power::STATIC_LOAD_EPS_A + 1e-4,
        "R2-M2: accepted holding current drifted too far above base: base={base_a:.5}, max={max_accepted_a:.5}"
    );
}

#[test]
fn r2_m1_tracking_position_lead_cannot_worsen_static_load() {
    use armd::tracking::{JointTrackingConfig, JointTrackingJoint, TrackingStrategy};
    let mut cfg = ArmConfig::rebot_dm();
    let arm = cfg.arm_model.clone();
    let cap = cfg.power_budget.as_ref().unwrap().static_admission_cap_a().unwrap();
    let q_low = [0.0, -0.35, 1.4, 0.0, 0.0, 0.0, 0.0];
    let q_higher = [0.0, -0.75, 1.4, 0.0, 0.0, 0.0, 0.0];
    let i_low = predict_static_holding_current_a(cfg.power_budget.as_ref().unwrap(), &arm, &q_low).unwrap();
    let i_hi = predict_static_holding_current_a(cfg.power_budget.as_ref().unwrap(), &arm, &q_higher).unwrap();
    assert!(i_hi > i_low + 0.3);

    // Put start just below cap so even a 0.03 rad position-lead overshoot toward J2=-0.38 exceeds cap!
    cfg.power_budget.as_mut().unwrap().static_reserve_a = cap - i_low - 0.005;
    let mut joints = vec![JointTrackingJoint::default(); cfg.arm_dof()];
    joints[1] = JointTrackingJoint {
        strategy: TrackingStrategy::PositionLead,
        lag_s: 0.03,
        max_lead_rad: 0.03,
        ..Default::default()
    };
    cfg.joint_tracking = Some(JointTrackingConfig {
        enabled: true,
        joints,
    });

    // Attach a recording driver so we verify the ACTUAL wire-emitted target never exceeds cap.
    struct RecordDriver {
        q: Vec<f32>,
        written: std::sync::Arc<std::sync::Mutex<Vec<Vec<f32>>>>,
        now: std::time::Instant,
    }
    impl armd::driver::ArmDriver for RecordDriver {
        fn capabilities(&self) -> armd::driver::DriverCapabilities {
            armd::driver::DriverCapabilities {
                temperature_feedback: true,
                load_feedback: false,
                torque_feedback: true,
                velocity_feedback: true,
                torque_feedforward: false,
                enable_feedback: false,
            }
        }
        fn read_joint_states(&mut self) -> Result<Vec<armd::model::JointState>, armd::driver::ArmDriverError> {
            self.now = std::time::Instant::now();
            Ok(self.q.iter().map(|&p| armd::model::JointState { position: p, velocity: 0.0, torque: 0.0, temperature: 30.0 }).collect())
        }
        fn write_joint_targets(&mut self, targets: &[armd::driver::JointTarget]) -> Result<(), armd::driver::ArmDriverError> {
            self.written.lock().unwrap().push(targets.iter().map(|t| t.position).collect());
            Ok(())
        }
        fn set_torque(&mut self, _: bool) -> Result<(), armd::driver::ArmDriverError> { Ok(()) }
        fn is_feedback_valid(&self) -> bool { true }
        fn is_enabled(&self) -> bool { true }
        fn feedback_ages(&self) -> Vec<Option<std::time::Duration>> { vec![Some(std::time::Duration::ZERO); self.q.len()] }
        fn feedback_receive_instants(&self) -> Vec<Option<std::time::Instant>> { vec![Some(self.now); self.q.len()] }
    }

    let written = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let drv = RecordDriver {
        q: q_low.to_vec(),
        written: written.clone(),
        now: std::time::Instant::now(),
    };
    let budget = cfg.power_budget.clone().unwrap();
    let mut inner = SupervisorInner::new(cfg).with_driver(Box::new(drv));
    inner.lease_enabled = false;
    inner.is_torqued = true;
    inner.feedback_numeric_valid = true;
    inner.measured_positions = q_low.to_vec();
    inner.commanded_positions = q_low.to_vec();
    inner.target_positions = q_low.to_vec();
    inner.measured_velocities = vec![0.0; 7];
    // Negative commanded velocity on J2 triggers PositionLead (-lag_s * |v_cmd| toward higher load).
    inner.commanded_velocities = vec![0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    inner.temperatures = vec![30.0; 7];
    let _ = inner.safety.set_torque(true).unwrap();
    // Seed known tracking history at q_low (within cap).
    inner.tracking.record_issued(q_low.to_vec(), vec![0.0; 7]);
    inner.servo_stream_active = true;

    inner.tick(DT);

    let frames = written.lock().unwrap();
    assert!(!frames.is_empty(), "driver must have received a safe hold frame");
    for (idx, frame) in frames.iter().enumerate() {
        let a = predict_static_holding_current_a(&budget, &arm, frame).unwrap();
        assert!(
            a <= cap + armd::power::STATIC_LOAD_EPS_A,
            "R2-M1: frame #{idx} emitted by joint_tracking exceeded static cap: {a:.4} A > {cap:.4} A (q={frame:?})"
        );
    }
}

#[test]
fn r2_m4_macro_static_load_abort_finishes_last_action_and_core_macro_consistently() {
    let mut cfg = ArmConfig::rebot_dm();
    let arm = cfg.arm_model.clone();
    let cap = cfg.power_budget.as_ref().unwrap().static_admission_cap_a().unwrap();
    let q_low = [0.0, -0.20, 1.4, 0.0, 0.0, 0.0, 0.0];
    let q_hi = [0.0, -0.80, 1.4, 0.0, 0.0, 0.0, 0.0];
    let i_low = predict_static_holding_current_a(cfg.power_budget.as_ref().unwrap(), &arm, &q_low).unwrap();
    let i_hi = predict_static_holding_current_a(cfg.power_budget.as_ref().unwrap(), &arm, &q_hi).unwrap();
    assert!(i_hi > i_low + 0.3, "expected higher static load at q_hi: {i_low:.3} -> {i_hi:.3}");
    cfg.power_budget.as_mut().unwrap().static_reserve_a = cap - 0.5 * (i_low + i_hi);

    let mut inner = armed_legacy(cfg, q_low);
    let plan_input = inner.plan_input().unwrap();
    let stamp = inner.current_admission_stamp();
    let adm = inner.check_admission(stamp).unwrap();
    let macro_id = SupervisorInner::begin_macro(adm, "grasp", &plan_input, None).unwrap();

    // Install a JointPath child directly under `macro_id` that climbs above `cap`.
    let path = armd::motion::JointPath {
        dof: 7,
        arm_dof: 6,
        duration: 0.1,
        s: vec![0.0, 1.0],
        q: vec![q_low, q_hi],
        power_scale: 1.0,
    };
    inner.active_trajectory = Some(armd::motion::ActiveTrajectory::new(
        Some(macro_id),
        armd::motion::Reference::Path(armd::motion::PathTrajectory::new(path)),
    ));

    // Step ticks until the path step exceeds cap and triggers `abort_planned_motion_static_load_rt`.
    for _ in 0..8 {
        inner.tick(DT);
        if inner.active_trajectory.is_none() {
            break;
        }
    }
    assert!(inner.active_trajectory.is_none(), "worsening macro path must be aborted");
    assert!(inner.macro_run.is_none(), "macro_run must be retired on static-load abort");

    // Both `last_action` AND `core.last_macro()` must agree on `"failed"` / `"STATIC_LOAD_LIMIT"`.
    let la = inner.last_action.as_ref().expect("last_action present");
    assert_eq!(la.id, macro_id);
    assert_eq!(la.state.as_str(), "failed", "R2-M4: last_action must be failed, got {la:?}");
    assert_eq!(
        la.code.as_deref(),
        Some("STATIC_LOAD_LIMIT"),
        "R2-M4: last_action.code must be STATIC_LOAD_LIMIT, got {la:?}"
    );
    let mt = inner.core.last_macro().expect("core.last_macro recorded");
    assert_eq!(mt.action.0, macro_id);
    assert_eq!(
        mt.outcome,
        armd::rt_core::broker::Outcome::Failed("STATIC_LOAD_LIMIT"),
        "R2-M4: core.last_macro must agree on Failed(STATIC_LOAD_LIMIT), got {mt:?}"
    );
}
