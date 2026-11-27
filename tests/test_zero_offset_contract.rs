//! `ZeroOffsetDriver` contract (S4 PR 2, Fable spec §4.6): every method that carries a joint
//! position applies `joint = motor - offset` on reads and `motor = joint + offset` on commands
//! (velocity, torque, kp, kd unchanged), and the four extensions without joint values are the
//! inner driver's own objects.

use std::net::UdpSocket;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, CommissioningExt, DamiaoArmDriver, FashionStarArmDriver,
    FeetechArmDriver, HoldPassReport, JointMode, JointModeExt, JointTarget, ModeSwitchHoldTarget,
    PosForceEmitOutcome, PosForceExt, PosForceIntent, PosForceIntentKind, PosForceIntentStamp,
    PosForceTxStatus, RobStrideArmDriver, StoppedDisableGate, TxBatchOutcome,
};
#[cfg(not(target_arch = "wasm32"))]
use armd::driver::ShadowExt;
use armd::joint_zero::ZeroOffsetDriver;
use armd::mit::CommissioningFeedback;
use armd::model::JointState;
use armd::sim_driver::SimArmDriver;
#[cfg(not(target_arch = "wasm32"))]
use armd::torque::shadow::{EvidenceProvenance, RxStampKind, ShadowDriverObs};
use armd::transport::{MockCanTransport, MockTransport};

const DOF: usize = 7;
const OFF: [f32; DOF] = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
/// Motor-space reading of joint `i`.
fn motor(i: usize) -> f32 { 1.0 + 0.1 * i as f32 }

#[derive(Default)]
struct Seen {
    targets: Vec<(&'static str, Vec<JointTarget>)>,
    fallback: Vec<Option<f32>>,
    mode_holds: Vec<Option<ModeSwitchHoldTarget>>,
    pos_force_target: Option<f32>,
    emitted_pos_force: Vec<(PosForceIntent, PosForceIntentStamp)>,
    gates: Vec<StoppedDisableGate>,
}

/// Returns motor-space values and records what reaches it.
struct Inner(Arc<Mutex<Seen>>);

impl Inner {
    fn states(out: &mut Vec<JointState>) {
        out.clear();
        out.extend((0..DOF).map(|i| JointState { position: motor(i), velocity: 0.25, torque: -0.5, temperature: 30.0 }));
    }
    fn rec(&self, name: &'static str, t: &[JointTarget]) { self.0.lock().unwrap().targets.push((name, t.to_vec())); }
}

impl ArmDriver for Inner {
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut v = Vec::new();
        Self::states(&mut v);
        Ok(v)
    }
    fn read_joint_states_rx_only(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.read_joint_states() }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        Self::states(out);
        Ok(())
    }
    fn read_joint_states_budgeted(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
        Self::states(out);
        Ok((3, false))
    }
    fn read_joint_states_safe_hold(&mut self, out: &mut Vec<JointState>, _: Instant, _: u16) -> Result<(u16, bool), ArmDriverError> {
        Self::states(out);
        Ok((4, true))
    }
    fn read_cached_joint_states_into(&self, out: &mut Vec<JointState>) -> Option<()> {
        Self::states(out);
        Some(())
    }
    fn fresh_measured_position_for(&self, joint: usize, _: Duration) -> Option<f32> { Some(motor(joint)) }
    fn fresh_measured_sample_for(&self, joint: usize, _: Duration) -> Option<(f32, u64)> { Some((motor(joint), 9)) }
    fn fresh_joint_positions_into(&self, joints: &[usize], _: Duration, out: &mut [f32]) -> bool {
        for (o, &j) in out.iter_mut().zip(joints) {
            *o = motor(j);
        }
        true
    }
    fn position_limits(&self, joint: usize) -> Option<(f32, f32)> { Some((motor(joint) - 2.0, motor(joint) + 2.0)) }
    fn last_committed_wire_position(&self, joint: usize) -> Option<(u64, Instant, f32)> { Some((5, Instant::now(), motor(joint))) }
    fn write_joint_targets(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.rec("write_joint_targets", t);
        Ok(())
    }
    fn write_joint_targets_bounded(&mut self, t: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.rec("write_joint_targets_bounded", t);
        Ok(TxBatchOutcome::Complete)
    }
    fn write_estop_hold(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.rec("write_estop_hold", t);
        Ok(())
    }
    fn write_estop_hold_bounded(&mut self, t: &[JointTarget]) -> Result<TxBatchOutcome, ArmDriverError> {
        self.rec("write_estop_hold_bounded", t);
        Ok(TxBatchOutcome::Complete)
    }
    fn write_measured_hold_masked(&mut self, _: u64, t: &[JointTarget]) -> Result<HoldPassReport, ArmDriverError> {
        self.rec("write_measured_hold_masked", t);
        Ok(HoldPassReport::empty(t.len()))
    }
    fn defer_motion_targets_past_cutoff(&mut self, t: &[JointTarget]) { self.rec("defer_motion_targets_past_cutoff", t); }
    fn service_release_watchdog_fallback(&mut self, _: u64, hold: Option<&[Option<f32>]>) {
        self.0.lock().unwrap().fallback = hold.unwrap_or(&[]).to_vec();
    }
    fn set_stopped_disable_gate(&mut self, gate: StoppedDisableGate) {
        self.0.lock().unwrap().gates.push(gate);
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    armd::ext_accessors!(joint_mode, pos_force, commissioning, shadow);
}

impl JointModeExt for Inner {
    fn set_joint_modes(&mut self, _: &[JointMode], h: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.rec("set_joint_modes", h);
        Ok(())
    }
    fn set_joint_modes_in_tick(&mut self, _: &[JointMode], h: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.rec("set_joint_modes_in_tick", h);
        Ok(())
    }
    fn set_joint_modes_confirmed(&mut self, _: &[JointMode], h: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.rec("set_joint_modes_confirmed", h);
        Ok(())
    }
    fn set_mode_switch_hold_targets(&mut self, t: &[Option<ModeSwitchHoldTarget>]) {
        self.0.lock().unwrap().mode_holds = t.to_vec();
    }
}

impl PosForceExt for Inner {
    fn gripper_pos_force_info(&self) -> Option<armd::driver::GripperPosForceInfo> {
        Some(armd::driver::GripperPosForceInfo { joint: 6, active: true, close_speed_rad_s: 0.3, ceiling_torque_estimate_nm: None })
    }
    fn validate_pos_force_intent_target(&self, position: f32, _: Option<f32>, _: Option<u16>) -> Result<(), ArmDriverError> {
        self.0.lock().unwrap().pos_force_target = Some(position);
        Ok(())
    }
    fn emit_pos_force_j7(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        self.0.lock().unwrap().emitted_pos_force.push((intent, current));
        Ok(PosForceEmitOutcome::Sent { joint: 6, freeze: false, pf_status: PosForceTxStatus::Fresh })
    }
    fn pos_force_last_fresh_position(&self) -> Option<f32> { Some(motor(6)) }
}

#[cfg(not(target_arch = "wasm32"))]
impl ShadowExt for Inner {
    fn drain_shadow_observations(&mut self, out: &mut [Option<ShadowDriverObs>]) -> (usize, u64) {
        let now = Instant::now();
        if out.len() >= 2 {
            out[0] = Some(ShadowDriverObs::CausalSample {
                channel: 2,
                sample_id: 42,
                stamp: RxStampKind::Valid { rx_at: now, drift_ns: 0, live_post_settle: true },
                decode_now: now,
                position_rad: motor(2),
                velocity_rad_s: Some(0.25),
                temperature_c: Some(30.0),
                reported_enabled: Some(true),
                correlated_after_write_done: None,
                poll_issued: None,
                provenance: EvidenceProvenance::MockSynthetic,
            });
            out[1] = Some(ShadowDriverObs::SerialTimeout { device: 1, now });
            (2, 7)
        } else {
            (0, 0)
        }
    }
}

impl CommissioningExt for Inner {
    fn commissioning_raw_feedback(&self, joint: usize) -> Option<CommissioningFeedback> {
        Some(CommissioningFeedback {
            raw_position: motor(joint),
            joint_position: motor(joint),
            velocity: 0.25,
            motor_status: 1,
            rx_seq: 2,
            rx_at: Instant::now(),
            queue_bound: None,
        })
    }
}

fn wrapped() -> (Arc<Mutex<Seen>>, Box<dyn ArmDriver>) {
    let seen = Arc::new(Mutex::new(Seen::default()));
    (seen.clone(), Box::new(ZeroOffsetDriver::new(Box::new(Inner(seen)), &OFF)))
}

fn close(a: f32, b: f32) -> bool { (a - b).abs() < 1e-6 }

fn assert_joint_states(name: &str, s: &[JointState]) {
    assert_eq!(s.len(), DOF, "{name}");
    for (i, st) in s.iter().enumerate() {
        assert!(close(st.position, motor(i) - OFF[i]), "{name}: joint {i} position {}", st.position);
        assert_eq!((st.velocity, st.torque), (0.25, -0.5), "{name}: velocity and torque unchanged");
    }
}

#[test]
fn reads_subtract_the_offset() {
    let (_seen, mut d) = wrapped();
    let mut out = Vec::new();
    assert_joint_states("read_joint_states", &d.read_joint_states().unwrap());
    assert_joint_states("read_joint_states_rx_only", &d.read_joint_states_rx_only().unwrap());
    d.read_joint_states_into(&mut out).unwrap();
    assert_joint_states("read_joint_states_into", &out);
    assert_eq!(d.read_joint_states_budgeted(&mut out, Instant::now(), 8).unwrap(), (3, false));
    assert_joint_states("read_joint_states_budgeted", &out);
    assert_eq!(d.read_joint_states_safe_hold(&mut out, Instant::now(), 8).unwrap(), (4, true));
    assert_joint_states("read_joint_states_safe_hold", &out);
    d.read_cached_joint_states_into(&mut out).unwrap();
    assert_joint_states("read_cached_joint_states_into", &out);
    let age = Duration::from_millis(50);
    for j in 0..DOF {
        let want = motor(j) - OFF[j];
        assert!(close(d.fresh_measured_position_for(j, age).unwrap(), want), "fresh_measured_position_for {j}");
        let (p, epoch) = d.fresh_measured_sample_for(j, age).unwrap();
        assert!(close(p, want) && epoch == 9, "fresh_measured_sample_for {j}");
        let (lo, hi) = d.position_limits(j).unwrap();
        assert!(close(lo, want - 2.0) && close(hi, want + 2.0), "position_limits {j}");
        let (seq, _, wire) = d.last_committed_wire_position(j).unwrap();
        assert!(seq == 5 && close(wire, want), "last_committed_wire_position {j}");
    }
    let joints = [0usize, 3, 6];
    let mut pos = [0.0f32; 3];
    assert!(d.fresh_joint_positions_into(&joints, age, &mut pos));
    for (k, &j) in joints.iter().enumerate() {
        assert!(close(pos[k], motor(j) - OFF[j]), "fresh_joint_positions_into {j}");
    }
    assert!(close(d.pos_force().pos_force_last_fresh_position().unwrap(), motor(6) - OFF[6]), "pos_force_last_fresh_position");
    let fb = d.commissioning().commissioning_raw_feedback(2).unwrap();
    assert!(close(fb.raw_position, motor(2)), "commissioning_raw_feedback raw_position untouched");
    assert!(close(fb.joint_position, motor(2) - OFF[2]), "commissioning_raw_feedback joint_position");

    #[cfg(not(target_arch = "wasm32"))]
    {
        let mut obs = [None; 4];
        let (count, dropped) = d.shadow_mut().drain_shadow_observations(&mut obs);
        assert_eq!((count, dropped), (2, 7), "drain_shadow_observations count/dropped");
        match obs[0] {
            Some(ShadowDriverObs::CausalSample {
                channel,
                sample_id,
                position_rad,
                velocity_rad_s,
                temperature_c,
                reported_enabled,
                ..
            }) => {
                assert_eq!((channel, sample_id, reported_enabled), (2, 42, Some(true)));
                assert!(
                    close(position_rad, motor(2) - OFF[2]),
                    "drain_shadow_observations CausalSample.position_rad: got {position_rad}"
                );
                assert_eq!((velocity_rad_s, temperature_c), (Some(0.25), Some(30.0)));
            }
            other => panic!("expected CausalSample in slot 0, got {other:?}"),
        }
        assert!(
            matches!(obs[1], Some(ShadowDriverObs::SerialTimeout { device: 1, .. })),
            "non-CausalSample observation passes through untouched: {:?}",
            obs[1]
        );
        assert!(obs[2].is_none());
    }
}

#[test]
fn commands_add_the_offset_and_keep_velocity_torque_and_gains() {
    let (seen, mut d) = wrapped();
    let t: Vec<JointTarget> = (0..DOF)
        .map(|i| JointTarget { position: 0.05 * i as f32, velocity: Some(0.5), torque: Some(-1.0), kp: Some(20.0), kd: Some(1.5) })
        .collect();
    d.write_joint_targets(&t).unwrap();
    d.write_joint_targets_bounded(&t).unwrap();
    d.write_estop_hold(&t).unwrap();
    d.write_estop_hold_bounded(&t).unwrap();
    d.write_measured_hold_masked(0x7f, &t).unwrap();
    d.defer_motion_targets_past_cutoff(&t);
    let modes = [JointMode::Position; DOF];
    d.joint_mode_mut().set_joint_modes(&modes, &t).unwrap();
    d.joint_mode_mut().set_joint_modes_in_tick(&modes, &t).unwrap();
    d.joint_mode_mut().set_joint_modes_confirmed(&modes, &t).unwrap();
    let names: Vec<&str> = seen.lock().unwrap().targets.iter().map(|(n, _)| *n).collect();
    assert_eq!(names.len(), 9, "{names:?}");
    for (name, sent) in &seen.lock().unwrap().targets {
        for (i, (s, j)) in sent.iter().zip(&t).enumerate() {
            assert!(close(s.position, j.position + OFF[i]), "{name}: joint {i}");
            assert_eq!((s.velocity, s.torque, s.kp, s.kd), (j.velocity, j.torque, j.kp, j.kd), "{name}: joint {i}");
        }
    }
    let hold: Vec<Option<f32>> = (0..DOF).map(|i| (i % 2 == 0).then_some(0.1 * i as f32)).collect();
    d.service_release_watchdog_fallback(0x7f, Some(&hold));
    let mh: Vec<Option<ModeSwitchHoldTarget>> = (0..DOF)
        .map(|i| (i != 3).then_some(ModeSwitchHoldTarget { position: 0.2, velocity: 0.1, kp: 5.0, kd: 0.5, torque: 0.3, is_mit: true }))
        .collect();
    d.joint_mode_mut().set_mode_switch_hold_targets(&mh);
    d.pos_force().validate_pos_force_intent_target(-1.0, None, None).unwrap();
    let intent_stamp = PosForceIntentStamp { stop_gen: 3, estop_epoch: 5, lease_gen: 11 };
    let current_stamp = PosForceIntentStamp { stop_gen: 3, estop_epoch: 5, lease_gen: 11 };
    let target_intent = PosForceIntent::target(-0.85, Some(0.42), Some(321), intent_stamp);
    let hold_stamp = PosForceIntentStamp { stop_gen: 4, estop_epoch: 6, lease_gen: 0 };
    let hold_intent = PosForceIntent::hold(hold_stamp);
    d.pos_force_mut().emit_pos_force_j7(target_intent, current_stamp).unwrap();
    d.pos_force_mut().emit_pos_force_j7(hold_intent, hold_stamp).unwrap();
    let s = seen.lock().unwrap();
    for i in 0..DOF {
        assert_eq!(s.fallback[i].map(|p| close(p, 0.1 * i as f32 + OFF[i])), hold[i].map(|_| true), "fallback {i}");
        match (&s.mode_holds[i], &mh[i]) {
            (Some(got), Some(want)) => {
                assert!(close(got.position, want.position + OFF[i]), "mode switch hold {i}");
                assert_eq!((got.velocity, got.kp, got.kd, got.torque, got.is_mit), (want.velocity, want.kp, want.kd, want.torque, want.is_mit));
            }
            (got, want) => assert_eq!(got.is_none(), want.is_none(), "mode switch hold {i}"),
        }
    }
    assert!(close(s.pos_force_target.unwrap(), -1.0 + OFF[6]), "validate_pos_force_intent_target shifts the gripper joint");
    assert_eq!(s.emitted_pos_force.len(), 2, "emit_pos_force_j7 must record both Target and Hold calls");
    let (got_target, got_current) = s.emitted_pos_force[0];
    assert_eq!(got_target.stamp, intent_stamp, "emit_pos_force_j7 target stamp unchanged");
    assert_eq!(got_current, current_stamp, "emit_pos_force_j7 current stamp unchanged");
    match got_target.kind {
        PosForceIntentKind::Target { position, velocity, current_raw } => {
            assert!(close(position, -0.85 + OFF[6]), "emit_pos_force_j7 motor-space target: got {position}");
            assert_eq!(velocity, Some(0.42), "emit_pos_force_j7 velocity unchanged");
            assert_eq!(current_raw, Some(321), "emit_pos_force_j7 current_raw unchanged");
        }
        other => panic!("expected Target intent, got {other:?}"),
    }
    assert_eq!(s.emitted_pos_force[1], (hold_intent, hold_stamp), "emit_pos_force_j7 Hold passes through unchanged");
}

#[test]
fn stopped_disable_gate_shifts_and_reshifts_on_offset_update() {
    let (seen, mut d) = wrapped();
    let mut arm_joints = [0usize; armd::rt_core::mailbox::MAX_DOF];
    let mut rest_positions = [0.0f32; armd::rt_core::mailbox::MAX_DOF];
    for i in 0..DOF {
        arm_joints[i] = i;
        rest_positions[i] = 0.05 * (i as f32 + 1.0);
    }
    let gate = StoppedDisableGate {
        safety_stopped: Arc::new(AtomicBool::new(true)),
        has_resting_pose: true,
        arm_joints,
        arm_joints_len: 6,
        rest_positions,
        rest_pose_tolerance_rad: 0.08,
    };
    d.set_stopped_disable_gate(gate);

    let new_off: [f32; DOF] = [-0.15, 0.25, -0.35, 0.45, -0.55, 0.65, -0.75];
    d.joint_zero_mut().set_joint_zero_offsets(&new_off);

    let gates = &seen.lock().unwrap().gates;
    assert_eq!(gates.len(), 2, "set_stopped_disable_gate + set_joint_zero_offsets re-shift");
    for (label, g, off) in [("initial", &gates[0], &OFF), ("reshifted", &gates[1], &new_off)] {
        assert!(g.has_resting_pose, "{label}");
        assert_eq!(g.arm_joints_len, 6, "{label}");
        assert_eq!(g.rest_pose_tolerance_rad, 0.08, "{label}");
        for i in 0..DOF {
            assert!(
                close(g.rest_positions[i], rest_positions[i] + off[i]),
                "{label}: joint {i} rest_position {} != {}",
                g.rest_positions[i],
                rest_positions[i] + off[i]
            );
        }
    }
}

#[test]
fn offsets_are_exposed_and_updated_without_reallocating() {
    let (_seen, mut d) = wrapped();
    assert_eq!(d.joint_zero().joint_zero_offsets(), Some(&OFF[..]));
    let ptr = d.joint_zero().joint_zero_offsets().unwrap().as_ptr();
    d.joint_zero_mut().set_joint_zero_offsets(&[0.0; DOF]);
    assert_eq!(d.joint_zero().joint_zero_offsets(), Some(&[0.0; DOF][..]));
    assert_eq!(d.joint_zero().joint_zero_offsets().unwrap().as_ptr(), ptr, "same length: copied in place");
    assert_joint_states_zero(&d.read_joint_states().unwrap());
}

fn assert_joint_states_zero(s: &[JointState]) {
    for (i, st) in s.iter().enumerate() {
        assert_eq!(st.position.to_bits(), motor(i).to_bits(), "offset 0 passes joint {i} through bit for bit");
    }
}

fn addr<T: ?Sized>(r: Option<&T>) -> Option<*const u8> { r.map(|x| x as *const T as *const u8) }

#[test]
fn extensions_without_joint_values_pass_through_to_the_inner_object() {
    // Damiao has all four: the wrapper hands out the inner driver's own object (pointer equality).
    let dm = DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect());
    let inner: Box<dyn ArmDriver> = Box::new(dm);
    let want = [
        addr(inner.tx_ext()),
        addr(inner.mode_switch_ext()),
        addr(inner.mit_ext()),
        addr(inner.bus_recovery_ext()),
    ];
    assert!(want.iter().all(Option::is_some));
    let inner_ptr = &*inner as *const dyn ArmDriver as *const u8;
    assert!(want.iter().all(|p| *p == Some(inner_ptr)), "Damiao returns itself");
    let mut w: Box<dyn ArmDriver> = Box::new(ZeroOffsetDriver::new(inner, &OFF));
    let got = [addr(w.tx_ext()), addr(w.mode_switch_ext()), addr(w.mit_ext()), addr(w.bus_recovery_ext())];
    assert_eq!(got, want, "pass-through accessors return the inner object");
    let got_mut = [
        w.tx_ext_mut().map(|x| x as *mut _ as *const u8),
        w.mode_switch_ext_mut().map(|x| x as *mut _ as *const u8),
        w.mit_ext_mut().map(|x| x as *mut _ as *const u8),
        w.bus_recovery_ext_mut().map(|x| x as *mut _ as *const u8),
    ];
    assert_eq!(got_mut, want);
    // Wrapped extensions are the wrapper itself, present because Damiao has them.
    let w_ptr = &*w as *const dyn ArmDriver as *const u8;
    for (name, p) in [
        ("joint_mode", addr(w.joint_mode_ext())),
        ("pos_force", addr(w.pos_force_ext())),
        ("commissioning", addr(w.commissioning_ext())),
        ("shadow", addr(w.shadow_ext())),
        ("joint_zero", addr(w.joint_zero_ext())),
    ] {
        assert_eq!(p, Some(w_ptr), "{name}: the wrapper applies the offsets");
    }

    // An inner driver without the extensions: the wrapper reports None for all but JointZeroExt.
    let (_seen, mut plain) = {
        struct Plain;
        impl ArmDriver for Plain {
            fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { Ok(Vec::new()) }
            fn write_joint_targets(&mut self, _: &[JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
            fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
        }
        ((), Box::new(ZeroOffsetDriver::new(Box::new(Plain), &OFF)) as Box<dyn ArmDriver>)
    };
    let none = [
        plain.tx_ext().is_none(),
        plain.mode_switch_ext().is_none(),
        plain.mit_ext().is_none(),
        plain.bus_recovery_ext().is_none(),
        plain.joint_mode_ext().is_none(),
        plain.pos_force_ext().is_none(),
        plain.commissioning_ext().is_none(),
        plain.shadow_ext().is_none(),
        plain.tx_ext_mut().is_none(),
        plain.joint_mode_ext_mut().is_none(),
        plain.shadow_ext_mut().is_none(),
    ];
    assert!(none.iter().all(|b| *b), "{none:?}");
    assert!(plain.joint_zero_ext().is_some() && plain.joint_zero_ext_mut().is_some());
}

#[test]
fn wrapped_follower_accessor_matrix() {
    // Fable M1: production owners always hold `ZeroOffsetDriver(follower)`.
    // Every wrapped follower must expose its bare Ext row with `joint_zero = true`.
    const COLS: [&str; 9] = [
        "tx", "mode_switch", "commissioning", "joint_mode", "mit", "pos_force", "bus_recovery", "shadow", "joint_zero",
    ];
    const T: bool = true;
    const F: bool = false;
    fn check(name: &str, d: &mut dyn ArmDriver, want: [bool; 9]) {
        let shared = [
            d.tx_ext().is_some(),
            d.mode_switch_ext().is_some(),
            d.commissioning_ext().is_some(),
            d.joint_mode_ext().is_some(),
            d.mit_ext().is_some(),
            d.pos_force_ext().is_some(),
            d.bus_recovery_ext().is_some(),
            d.shadow_ext().is_some(),
            d.joint_zero_ext().is_some(),
        ];
        let excl = [
            d.tx_ext_mut().is_some(),
            d.mode_switch_ext_mut().is_some(),
            d.commissioning_ext_mut().is_some(),
            d.joint_mode_ext_mut().is_some(),
            d.mit_ext_mut().is_some(),
            d.pos_force_ext_mut().is_some(),
            d.bus_recovery_ext_mut().is_some(),
            d.shadow_ext_mut().is_some(),
            d.joint_zero_ext_mut().is_some(),
        ];
        for i in 0..COLS.len() {
            assert_eq!(shared[i], want[i], "{name}: {}_ext()", COLS[i]);
            assert_eq!(excl[i], want[i], "{name}: {}_ext_mut()", COLS[i]);
        }
    }

    //                                          tx mode comm jmode mit pf  bus shadow jzero
    let damiao = [T, T, T, T, T, T, T, T, T];
    let sim = [F, F, T, T, F, F, F, T, T];
    let legacy = [F, F, F, F, F, F, F, T, T];

    let mut dm = ZeroOffsetDriver::new(Box::new(DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect())), &[0.0; 7]);
    check("ZeroOffset(Damiao)", &mut dm, damiao);
    let mut ft = ZeroOffsetDriver::new(Box::new(FeetechArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6])), &[0.0; 6]);
    check("ZeroOffset(Feetech)", &mut ft, legacy);
    let mut fs = ZeroOffsetDriver::new(Box::new(FashionStarArmDriver::new(MockTransport::new(), vec![1, 2, 3, 4, 5, 6])), &[0.0; 6]);
    check("ZeroOffset(FashionStar)", &mut fs, legacy);
    let mut rs = ZeroOffsetDriver::new(Box::new(RobStrideArmDriver::new(MockCanTransport::new(), (1..=6).collect())), &[0.0; 6]);
    check("ZeroOffset(RobStride)", &mut rs, legacy);
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sm_inner = SimArmDriver::connect(&format!("udp://{}", sock.local_addr().unwrap()), 7).unwrap();
    let mut sm = ZeroOffsetDriver::new(Box::new(sm_inner), &[0.0; 7]);
    check("ZeroOffset(Sim)", &mut sm, sim);
}
