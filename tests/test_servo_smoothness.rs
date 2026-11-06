use std::time::{Duration, Instant};

use armd::{ArmConfig, ArmObservation, ArmSupervisor, JointLimit, SafetyState, SupervisorInner};

async fn arm(dof: usize, velocity: f32, acceleration: f32) -> ArmSupervisor {
    let mut cfg = ArmConfig::default();
    cfg.dof = dof;
    cfg.home_positions = vec![0.0; dof];
    cfg.limits = (0..dof)
        .map(|i| JointLimit::new(format!("j{i}"), -1.0, 1.0, velocity, acceleration))
        .collect();
    let supervisor = ArmSupervisor::new(cfg);
    supervisor.set_lease_enabled(false).await;
    supervisor.set_torque(true).await.unwrap();
    supervisor.inner().write().await.motion_frame_timeout = Duration::from_secs(60);
    supervisor
}

fn check_step(before: &ArmObservation, after: &ArmObservation, h: f64, a: f32, vmax: f32) {
    assert!(!after.safety_state.is_stopped(), "unexpected fault: {:?}", after.safety_state);
    for i in 0..before.positions.len() {
        let q = f64::from(after.positions[i]);
        let old_q = f64::from(before.positions[i]);
        let v = f64::from(after.velocities[i]);
        let old_v = f64::from(before.velocities[i]);
        // Account only for f32 storage rounding, not a fraction of a control step.
        let position_roundoff = 2.0 * f64::from(f32::EPSILON) * q.abs().max(old_q.abs()) + 1e-12;
        let velocity_roundoff = 2.0 * f64::from(f32::EPSILON) * v.abs().max(old_v.abs()) + 1e-12;
        assert!((q - old_q - v * h).abs() <= position_roundoff,
            "axis {i}: dq={}, v*h={}", q-old_q, v*h);
        assert!((v - old_v).abs() <= f64::from(a) * h + velocity_roundoff,
            "axis {i}: dv={}, allowance={}", v-old_v, f64::from(a)*h);
        assert!(v.abs() <= f64::from(vmax) + velocity_roundoff);
        assert!((-1.0..=1.0).contains(&q), "axis {i}: q={q}");
    }
}

#[tokio::test]
async fn short_target_brakes_without_overshoot_and_reaches_exact_zero_velocity() {
    let supervisor = arm(1, 2.0, 5.0).await;
    supervisor.servo(vec![0.10], 1).await.unwrap();
    let mut previous = supervisor.get_observation().await;
    let mut saw_motion = false;
    for _ in 0..100 {
        let current = supervisor.tick(0.02).await;
        check_step(&previous, &current, 0.02, 5.0, 2.0);
        saw_motion |= current.velocities[0] > 0.0;
        assert!(current.positions[0] <= 0.1);
        previous = current;
    }
    assert!(saw_motion);
    assert!((previous.positions[0] - 0.1).abs() <= 1.0e-5);
    assert_eq!(previous.velocities, vec![0.0]);
    assert!(supervisor.inner().read().await.servo_target.is_none());
}

#[tokio::test]
async fn streaming_small_multi_axis_targets_preserve_derivatives() {
    let supervisor = arm(2, 2.0, 5.0).await;
    let mut previous = supervisor.get_observation().await;
    for k in 1..=50 {
        // Each network sample is smaller than the old 0.04-rad arrival threshold.
        let target = k as f32 * 0.001;
        supervisor.servo(vec![target, -target * 0.5], 1).await.unwrap();
        let current = supervisor.tick(0.02).await;
        check_step(&previous, &current, 0.02, 5.0, 2.0);
        assert!(current.velocities[0] > 0.0);
        assert!(current.velocities[1] < 0.0);
        previous = current;
    }
    assert!(previous.positions[0] > 0.04);
    assert!(previous.positions[1] < -0.02);
}

#[tokio::test]
async fn reversal_moves_in_both_directions_and_settles() {
    let supervisor = arm(1, 2.0, 5.0).await;
    supervisor.servo(vec![0.20], 1).await.unwrap();
    let mut previous = supervisor.get_observation().await;
    for _ in 0..8 {
        let current = supervisor.tick(0.02).await;
        check_step(&previous, &current, 0.02, 5.0, 2.0);
        previous = current;
    }
    assert!(previous.velocities[0] > 0.3);
    // This reverse goal is within the dirty-frame window, so it is exercised.
    supervisor.servo(vec![-0.05], 1).await.unwrap();
    let mut saw_negative = false;
    for _ in 0..150 {
        let current = supervisor.tick(0.02).await;
        check_step(&previous, &current, 0.02, 5.0, 2.0);
        saw_negative |= current.velocities[0] < -0.1;
        previous = current;
    }
    assert!(saw_negative);
    assert!((previous.positions[0] + 0.05).abs() <= 1e-5);
    assert_eq!(previous.velocities[0], 0.0);
}

#[tokio::test]
async fn expired_servo_brakes_at_both_limits_with_alternating_dt() {
    for sign in [-1.0_f32, 1.0] {
        let supervisor = arm(1, 2.0, 5.0).await;
        {
            let mut inner = supervisor.inner().write().await;
            inner.commanded_positions[0] = sign * 0.90;
            inner.measured_positions[0] = sign * 0.90;
            inner.commanded_velocities[0] = sign;
            inner.measured_velocities[0] = sign;
        }
        supervisor.servo(vec![sign * 0.99], 1).await.unwrap();
        {
            let mut inner = supervisor.inner().write().await;
            inner.motion_frame_timeout = Duration::from_millis(200);
            inner.last_motion_frame = Instant::now() - Duration::from_millis(250);
            assert!(inner.servo_target.is_some());
        }
        let mut previous = supervisor.get_observation().await;
        for h in [0.001, 0.06].into_iter().cycle().take(20) {
            let current = supervisor.tick(h).await;
            check_step(&previous, &current, h, 5.0, 2.0);
            assert!(current.velocities[0].abs() <= previous.velocities[0].abs());
            previous = current;
        }
        assert_eq!(previous.velocities[0], 0.0);
        assert!(previous.positions[0].abs() > 0.9);
        assert!(supervisor.inner().read().await.servo_target.is_none());
    }
}

#[tokio::test]
async fn variable_dt_respects_small_acceleration_and_velocity_limits() {
    let supervisor = arm(1, 0.0005, 0.0002).await;
    supervisor.servo(vec![0.001], 1).await.unwrap();
    let mut previous = supervisor.get_observation().await;
    let mut peak_v = 0.0_f32;
    for h in [0.001, 0.01, 0.06, 0.003, 0.02, 0.05].into_iter().cycle().take(600) {
        let current = supervisor.tick(h).await;
        check_step(&previous, &current, h, 0.0002, 0.0005);
        peak_v = peak_v.max(current.velocities[0]);
        previous = current;
    }
    assert!(peak_v > 0.0001, "test must exercise actual motion");
    assert!((previous.positions[0] - 0.001).abs() <= 1e-5);
    assert_eq!(previous.velocities[0], 0.0);
}

#[tokio::test]
async fn tiny_nonzero_velocity_is_not_truncated_during_braking() {
    let supervisor = arm(1, 2.0, 0.001).await;
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_velocities[0] = 0.00008;
        inner.measured_velocities[0] = 0.00008;
    }
    let mut previous = supervisor.get_observation().await;
    for k in 0..10 {
        let current = supervisor.tick(0.02).await;
        check_step(&previous, &current, 0.02, 0.001, 2.0);
        if k == 0 { assert!(current.velocities[0] > 0.00005); }
        previous = current;
    }
    assert_eq!(previous.velocities[0], 0.0);
    assert!(previous.positions[0] > 0.0);
}

#[tokio::test]
async fn infeasible_axis_faults_without_partial_multi_axis_update() {
    for active_servo in [false, true] {
        let supervisor = arm(2, 2.0, 5.0).await;
        {
            let mut inner = supervisor.inner().write().await;
            inner.commanded_positions = vec![0.0, 0.9];
            inner.measured_positions = vec![0.0, 0.9];
            inner.commanded_velocities = vec![0.1, 2.0];
        }
        if active_servo { supervisor.servo(vec![0.1, 0.95], 1).await.unwrap(); }
        let obs = supervisor.tick(0.02).await;
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
        assert_eq!(obs.positions, vec![0.0, 0.9], "no axis may commit a partial step");
        assert_eq!(obs.velocities, vec![0.0, 0.0]);
        assert!(obs.is_torqued);
        let next = supervisor.tick(0.02).await;
        assert_eq!(next.positions, obs.positions);
        assert_eq!(next.velocities, obs.velocities);
        assert!(supervisor.inner().read().await.servo_target.is_none());
    }
}

#[tokio::test]
async fn invalid_acceleration_faults_instead_of_silently_increasing_it() {
    for acceleration in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        let supervisor = arm(1, 2.0, 5.0).await;
        supervisor.servo(vec![0.1], 1).await.unwrap();
        supervisor.inner().write().await.config.limits[0].max_acceleration = acceleration;
        let obs = supervisor.tick(0.02).await;
        assert!(matches!(obs.safety_state, SafetyState::Fault(_)));
        assert_eq!(obs.positions[0], 0.0);
        assert_eq!(obs.velocities[0], 0.0);
    }
}

#[tokio::test]
async fn test_limit_eps_tolerance_exact_accounting() {
    let limit = JointLimit::new("j0", -1.0, 1.0, 2.0, 5.0);
    let min = limit.min_position;
    let h = 0.02_f64;
    let a_max = limit.max_acceleration;
    let q_raw = min - 0.5e-3;
    let v_prev = 0.0_f32;
    let target = min;

    let res = SupervisorInner::bounded_servo_step(q_raw, target, v_prev, h, &limit);
    assert!(res.is_some(), "expected Some((q_next, v_final, _)) within POS_EPS");
    let (q_next, v_final, _) = res.unwrap();

    let lo = f64::from(min);
    let hi = f64::from(limit.max_position);
    let q_projected = f64::from(q_raw).clamp(lo, hi);
    let delta_projection = q_projected - f64::from(q_raw);
    assert!((delta_projection - 0.5e-3).abs() <= 1e-7);

    let v_next = f64::from(v_final);
    let q_next64 = f64::from(q_next);
    let q_raw64 = f64::from(q_raw);

    assert!(
        (q_next64 - q_raw64 - (delta_projection + v_next * h)).abs() <= 1e-7,
        "integral consistency violated"
    );
    assert!(
        (f64::from(v_final) - (q_next64 - q_projected) / h).abs() <= 1e-7,
        "physical differential speed violated"
    );
    assert!(
        (f64::from(v_final) - f64::from(v_prev)).abs() <= f64::from(a_max) * h + 1e-7,
        "acceleration bound violated"
    );
}

#[tokio::test]
async fn test_limit_infeasible_braking_triggers_fault() {
    let limit = JointLimit::new("j0", -1.0, 1.0, 2.0, 5.0);
    let min = limit.min_position;
    let h = 0.02_f64;
    let a_max = limit.max_acceleration;
    let q_raw = min - 0.5e-3;
    let v_prev = -2.0 * a_max * (h as f32);
    let target = min;

    let res = SupervisorInner::bounded_servo_step(q_raw, target, v_prev, h, &limit);
    assert!(res.is_none(), "infeasible braking must strictly return None");

    let supervisor = arm(1, 2.0, 5.0).await;
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions[0] = q_raw;
        inner.measured_positions[0] = q_raw;
        inner.commanded_velocities[0] = v_prev;
        inner.measured_velocities[0] = v_prev;
    }
    supervisor.servo(vec![target], 1).await.unwrap();
    let obs = supervisor.tick(h).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "must trigger fault");
}

#[tokio::test]
async fn test_limit_exceeding_eps_triggers_fault() {
    let limit = JointLimit::new("j0", -1.0, 1.0, 2.0, 5.0);
    let min = limit.min_position;
    let h = 0.02_f64;
    let q_raw = min - 1.5e-3;
    let v_prev = 0.0_f32;
    let target = min;

    let res = SupervisorInner::bounded_servo_step(q_raw, target, v_prev, h, &limit);
    assert!(res.is_none(), "exceeding POS_EPS must strictly return None");

    let supervisor = arm(1, 2.0, 5.0).await;
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions[0] = q_raw;
        inner.measured_positions[0] = q_raw;
    }
    supervisor.servo(vec![target], 1).await.unwrap();
    let obs = supervisor.tick(h).await;
    assert!(matches!(obs.safety_state, SafetyState::Fault(_)), "must trigger fault");
}

#[tokio::test]
async fn test_two_tick_settle_hysteresis_and_exact_resets() {
    let supervisor = arm(1, 2.0, 5.0).await;
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 0);

    // Initial state at target 0.0
    supervisor.servo(vec![0.0], 1).await.unwrap();
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 0);

    // Tick 1: reaches target deadband (|e| <= 1.0e-4 && |v| <= 1.5e-2) -> counter = 1
    let obs1 = supervisor.tick(0.02).await;
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 1);
    assert!(supervisor.inner().read().await.servo_target.is_some());
    assert_eq!(obs1.safety_state, SafetyState::Moving);

    // Tick 2: reaches deadband for 2nd consecutive tick -> counter = 2, triggers settled!
    let obs2 = supervisor.tick(0.02).await;
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 2);
    assert!(supervisor.inner().read().await.servo_target.is_none());
    assert_eq!(obs2.safety_state, SafetyState::Armed);

    // 1. Inject position error exceeding threshold (|v| = 0 but |e| = 1.1e-4) -> counter resets to 0
    {
        let mut inner = supervisor.inner().write().await;
        inner.servo_settled_ticks[0] = 1;
        inner.config.limits[0].max_acceleration = 0.0001; // Tiny a_max so position does not step significantly
        inner.commanded_positions[0] = 0.0;
        inner.commanded_velocities[0] = 0.0;
        inner.servo_target = Some(vec![1.1e-4]);
    }
    supervisor.tick(0.001).await;
    assert_eq!(
        supervisor.inner().read().await.servo_settled_ticks[0], 0,
        "counter must reset to 0 on |e| = 1.1e-4 > 1.0e-4"
    );

    // 2. Inject velocity exceeding threshold (|e| = 0 but |v| = 1.6e-2) -> counter resets to 0
    {
        let mut inner = supervisor.inner().write().await;
        inner.servo_settled_ticks[0] = 1;
        inner.config.limits[0].max_acceleration = 0.001;
        inner.commanded_positions[0] = 0.0;
        inner.commanded_velocities[0] = 0.016; // 1.6e-2 > 1.5e-2
        inner.servo_target = Some(vec![0.0]);
    }
    supervisor.tick(0.001).await;
    assert_eq!(
        supervisor.inner().read().await.servo_settled_ticks[0], 0,
        "counter must reset to 0 on |v| = 1.6e-2 > 1.5e-2"
    );

    // 3. Inject fault case -> counter resets to 0
    {
        let mut inner = supervisor.inner().write().await;
        inner.servo_settled_ticks[0] = 1;
        inner.apply_stop_policy(armd::StopPolicy::EStopLatch);
        assert_eq!(inner.servo_settled_ticks[0], 0, "counter must reset on stop policy / fault");
        inner.clear_fault().unwrap();
        assert_eq!(inner.servo_settled_ticks[0], 0, "counter must reset on clear_fault");
    }
    supervisor.set_torque(true).await.unwrap();

    // 4. Inject timeout case -> counter resets to 0
    {
        let mut inner = supervisor.inner().write().await;
        inner.config.limits[0].max_acceleration = 5.0;
        inner.commanded_positions[0] = 0.0;
        inner.commanded_velocities[0] = 0.0;
        inner.servo_target = Some(vec![0.0]);
        inner.servo_settled_ticks[0] = 1;
        inner.motion_frame_timeout = Duration::from_millis(200);
        inner.last_motion_frame = Instant::now() - Duration::from_millis(250);
    }
    supervisor.tick(0.02).await;
    assert_eq!(
        supervisor.inner().read().await.servo_settled_ticks[0], 0,
        "counter must reset on timeout"
    );
    assert!(supervisor.inner().read().await.servo_target.is_none());

    // 5. When conditions are satisfied again: must strictly accumulate 2 frames again to mark motion finished
    supervisor.inner().write().await.motion_frame_timeout = Duration::from_secs(60);
    let epoch = supervisor.inner().read().await.safety.epoch_id();
    supervisor.servo(vec![0.0], epoch).await.unwrap();
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 0);

    let re_obs1 = supervisor.tick(0.02).await;
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 1);
    assert!(supervisor.inner().read().await.servo_target.is_some());
    assert_eq!(re_obs1.safety_state, SafetyState::Moving);

    let re_obs2 = supervisor.tick(0.02).await;
    assert_eq!(supervisor.inner().read().await.servo_settled_ticks[0], 2);
    assert!(supervisor.inner().read().await.servo_target.is_none());
    assert_eq!(re_obs2.safety_state, SafetyState::Armed);
}

#[tokio::test]
async fn test_settled_residual_velocity_bounded_decay() {
    let a_max = 0.1_f32;
    let h = 0.01_f64;
    let supervisor = arm(1, 2.0, a_max).await;

    // Simulate arm settled at position 0.0 with residual velocity v = 0.010 rad/s
    {
        let mut inner = supervisor.inner().write().await;
        inner.commanded_positions[0] = 0.0;
        inner.measured_positions[0] = 0.0;
        inner.commanded_velocities[0] = 0.010;
        inner.measured_velocities[0] = 0.010;
        inner.servo_target = None; // Settled state
    }

    let mut v_prev = 0.010_f64;
    let mut steps = 0;
    while v_prev > 0.0 {
        let obs = supervisor.tick(h).await;
        let v_curr = f64::from(obs.velocities[0]);
        let dv = (v_prev - v_curr).abs();
        let max_allowed_dv = f64::from(a_max) * h + 1e-7;
        assert!(
            dv <= max_allowed_dv,
            "step {}: dv={} exceeded max allowed decay {}",
            steps, dv, max_allowed_dv
        );
        assert!(
            v_curr < v_prev || v_curr == 0.0,
            "step {}: velocity must strictly decrease towards 0",
            steps
        );
        v_prev = v_curr;
        steps += 1;
        assert!(steps <= 20, "decay must terminate within expected step count");
    }

    assert!(
        steps >= 9,
        "residual velocity was truncated prematurely in only {} steps",
        steps
    );
    assert_eq!(v_prev, 0.0);
}
