//! Thresholds tuned on readings decoded with the old single Damiao range (VMAX 30,
//! TMAX 18) were converted to real units. The same raw CAN feedback frames must give the
//! same decisions: old decode + old threshold == per-joint decode + new threshold.
//! Each check sweeps raw 12-bit field values across the threshold, through the real
//! decoder; stall and resting pose also run end to end through DamiaoArmDriver and the
//! supervisor.
use std::sync::{Arc, Mutex};
use std::time::Duration;

use armd::driver::{ArmDriver, ArmDriverError, DamiaoArmDriver, DriverCapabilities, EnableState, JointTarget};
use armd::model::{ArmConfig, DamiaoRange, GripperPolicy, JointState, RestingPosePolicy};
use armd::motion::{Settle, SETTLE_VEL};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::supervisor::{GRIPPER_STALL_LATCH_NM, GRIPPER_STALL_RELEASE_NM};
use armd::transport::{CanFrame, MockCanTransport};
use armd::ArmSupervisor;

fn profile_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(DamiaoRange::limits).collect()
}

/// Feedback frame of motor `id` (state 1) at `pos` rad with raw velocity / torque fields.
fn frame(id: u8, pos: f32, v_raw: u16, t_raw: u16) -> CanFrame {
    let p = damiao::float_to_uint(pos, -12.5, 12.5, 16);
    CanFrame::new_standard(0x10 + id as u16, &[
        0x10 | id, (p >> 8) as u8, p as u8, (v_raw >> 4) as u8,
        (((v_raw & 0x0F) << 4) | (t_raw >> 8)) as u8, t_raw as u8, 30, 30,
    ]).unwrap()
}

const ZERO: u16 = 2048; // raw value closest to 0 (decodes to +VMAX/4095 or +TMAX/4095)

fn decode(f: &CanFrame, l: &DamiaoLimits) -> (f32, f32) {
    let fb = damiao::unpack_feedback_frame(f, l).unwrap();
    (fb.velocity, fb.torque)
}

/// (old, new) decode of a raw field on joint `j` (0-based).
fn pair(j: usize, v_raw: u16, t_raw: u16) -> ((f32, f32), (f32, f32)) {
    let f = frame(j as u8 + 1, 0.0, v_raw, t_raw);
    (decode(&f, &DamiaoLimits::default()), decode(&f, &profile_limits()[j]))
}

/// Old rebot-dm values, as tuned on the old readings.
const OLD_STALL_LATCH: f32 = 0.80;
const OLD_STALL_RELEASE: f32 = 0.40;
const OLD_HOLDING_TORQUE: f32 = 0.12;
const OLD_REST_VEL: f32 = 0.015;
const OLD_REST_TORQUE: [(usize, f32); 2] = [(1, 1.5), (2, 4.5)];

#[test]
fn stall_and_holding_torque_decisions_match_over_every_raw_value() {
    let hold_new = GripperPolicy::rebot_dm().holding_torque_nm;
    for t in 0..4096u16 {
        let ((_, old), (_, new)) = pair(6, ZERO, t);
        assert_eq!(old.abs() >= OLD_STALL_LATCH, new.abs() >= GRIPPER_STALL_LATCH_NM, "latch raw {t}: {old} / {new}");
        assert_eq!(old.abs() < OLD_STALL_RELEASE, new.abs() < GRIPPER_STALL_RELEASE_NM, "release raw {t}: {old} / {new}");
        assert_eq!(old.abs() >= OLD_HOLDING_TORQUE, new.abs() >= hold_new, "holding raw {t}: {old} / {new}");
    }
    // J7 velocity range did not change: the |v7| < 0.10 stall condition is unaffected.
    for v in 0..4096u16 {
        let ((old, _), (new, _)) = pair(6, v, ZERO);
        assert_eq!(old, new);
    }
}

#[test]
fn resting_pose_decisions_match_over_every_raw_value() {
    let policy = RestingPosePolicy::rebot_dm();
    for raw in 0..4096u16 {
        for j in 0..7 {
            let ((old, _), (new, _)) = pair(j, raw, ZERO);
            assert_eq!(old.abs() > OLD_REST_VEL, new.abs() > policy.max_velocity_of(j), "J{} v raw {raw}: {old} / {new}", j + 1);
        }
        for (k, &(j, old_limit)) in OLD_REST_TORQUE.iter().enumerate() {
            let ((_, old), (_, new)) = pair(j, ZERO, raw);
            let (nj, new_limit) = policy.support_torque_limits[k];
            assert_eq!(nj, j);
            assert_eq!(old.abs() > old_limit, new.abs() > new_limit, "J{} tau raw {raw}: {old} / {new}", j + 1);
        }
    }
}

/// Settle keeps SETTLE_VEL = 0.05 rad/s in real units on every joint. J4-J6 decode as
/// before, so their decision is unchanged. On J1-J3 the old reading was 3x the real
/// speed: the old check required 0.0167 rad/s real, the new one 0.05 rad/s. The only
/// frames that change decision are those with a real J1-J3 speed in [0.0167, 0.05),
/// which now count as still (looser, documented in docs/control_safety_contract.md).
#[test]
fn settle_decisions_match_except_the_documented_j1_j3_band() {
    let run = |v: &[f32]| {
        let mut s = Settle::default();
        let c = [0.0f32; 6];
        (0..20).find_map(|_| s.step(0.02, &c, &c, v)).unwrap_or(false)
    };
    let mut changed = Vec::new();
    for raw in 0..4096u16 {
        for j in 0..6 {
            let ((old, _), (new, _)) = pair(j, raw, ZERO);
            let mut vo = [0.0f32; 6];
            let mut vn = [0.0f32; 6];
            vo[j] = old;
            vn[j] = new;
            let (a, b) = (run(&vo), run(&vn));
            if a != b {
                assert!(j < 3 && b && !a, "J{} raw {raw}: only J1-J3 may change, and only towards settled", j + 1);
                assert!(new.abs() >= SETTLE_VEL as f32 / 3.0 - 0.003 && new.abs() < SETTLE_VEL as f32, "J{} raw {raw}: {new}", j + 1);
                changed.push((j, raw));
            }
        }
    }
    // 2 x 7 raw values per joint (about 0.0167..0.05 rad/s on each side)
    assert_eq!(changed.len(), 3 * 14, "{changed:?}");
}

// ---------------------------------------------------------------- end to end

/// DamiaoArmDriver shared with the test so feedback frames can be fed between ticks.
struct Shared(Arc<Mutex<DamiaoArmDriver<MockCanTransport>>>);

impl ArmDriver for Shared {
    fn capabilities(&self) -> DriverCapabilities { self.0.lock().unwrap().capabilities() }
    fn enable_state(&self) -> EnableState { self.0.lock().unwrap().enable_state() }
    fn feedback_ages(&self) -> Vec<Option<Duration>> { self.0.lock().unwrap().feedback_ages() }
    fn max_command_velocity(&self, j: usize) -> Option<f32> { self.0.lock().unwrap().max_command_velocity(j) }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> { self.0.lock().unwrap().read_joint_states() }
    fn write_joint_targets(&mut self, t: &[JointTarget]) -> Result<(), ArmDriverError> { self.0.lock().unwrap().write_joint_targets(t) }
    fn set_torque(&mut self, e: bool) -> Result<(), ArmDriverError> { self.0.lock().unwrap().set_torque(e) }
    fn is_feedback_valid(&self) -> bool { self.0.lock().unwrap().is_feedback_valid() }
    fn check_hardware_status(&self) -> Result<(), ArmDriverError> { self.0.lock().unwrap().check_hardware_status() }
    fn is_enabled(&self) -> bool { self.0.lock().unwrap().is_enabled() }
    fn motor_status(&self) -> Option<Vec<u8>> { self.0.lock().unwrap().motor_status() }
    fn set_torque_flag(&mut self, e: bool) { self.0.lock().unwrap().set_torque_flag(e) }
    fn set_pending_rearm(&mut self, p: bool) { self.0.lock().unwrap().set_pending_rearm(p) }
}

struct Rig { sup: ArmSupervisor, drv: Arc<Mutex<DamiaoArmDriver<MockCanTransport>>> }

impl Rig {
    /// rebot-dm arm at home; `old` decodes with the single default range and uses the
    /// old resting-pose thresholds.
    async fn new(old: bool) -> Self {
        let mut config = ArmConfig::rebot_dm();
        let home = config.home_positions.clone();
        let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), (1..=7).collect());
        if old {
            config.damiao_ranges = None;
            let p = config.resting_pose.as_mut().unwrap();
            p.joint_max_velocities.clear();
            p.max_velocity = OLD_REST_VEL;
            p.support_torque_limits = OLD_REST_TORQUE.to_vec();
        } else {
            driver = driver.with_joint_limits(profile_limits()).unwrap();
        }
        for id in 1..=7u8 {
            let f = frame(id, home[id as usize - 1], ZERO, ZERO);
            driver.can_mut().respond_to_transmit(damiao::build_enable_frame(id as u16), f);
            driver.can_mut().provide_rx_frame(f);
        }
        let drv = Arc::new(Mutex::new(driver));
        let sup = ArmSupervisor::new(config);
        sup.set_driver(Box::new(Shared(drv.clone()))).await;
        sup.set_torque(true).await.unwrap();
        Self { sup, drv }
    }

    /// Feed one frame per motor (joint `j` with raw `v`, `t`; others at rest) and tick.
    async fn tick(&self, j: usize, v: u16, t: u16) {
        let home = ArmConfig::rebot_dm().home_positions;
        {
            let mut d = self.drv.lock().unwrap();
            for i in 0..7 {
                let (vi, ti) = if i == j { (v, t) } else { (ZERO, ZERO) };
                d.can_mut().provide_rx_frame(frame(i as u8 + 1, home[i], vi, ti));
            }
        }
        self.sup.tick(0.02).await;
    }
}

#[tokio::test]
async fn stall_end_to_end_matches_the_old_rule() {
    let rig = Rig::new(false).await;
    // around +0.80 / -0.80 (old) = raw 2139 / 1956, and +0.40 / -0.40 = raw 2093 / 2002
    let latch_raws: Vec<u16> = (2128..2150).chain(1945..1968).collect();
    for &t in &latch_raws {
        {
            let mut inner = rig.sup.inner().write().await;
            inner.gripper_stalled = false;
            inner.gripper_high_torque_ticks = 0;
        }
        for _ in 0..6 { rig.tick(6, ZERO, t).await; }
        let ((_, old), _) = pair(6, ZERO, t);
        assert_eq!(rig.sup.is_gripper_stalled().await, old.abs() >= OLD_STALL_LATCH, "latch raw {t}: old reading {old}");
    }
    let release_raws: Vec<u16> = (2082..2104).chain(1991..2013).collect();
    for &t in &release_raws {
        {
            let mut inner = rig.sup.inner().write().await;
            inner.gripper_stalled = true;
            inner.gripper_low_torque_ticks = 0;
        }
        for _ in 0..6 { rig.tick(6, ZERO, t).await; }
        let ((_, old), _) = pair(6, ZERO, t);
        assert_eq!(!rig.sup.is_gripper_stalled().await, old.abs() < OLD_STALL_RELEASE, "release raw {t}: old reading {old}");
    }
}

#[tokio::test]
async fn resting_pose_end_to_end_matches_old_scale() {
    let (old, new) = (Rig::new(true).await, Rig::new(false).await);
    // J1-J3 velocity around the 0.015 rad/s old limit (raw 2047 / 2048 pass, 2046 / 2049 fail)
    // and J2 / J3 torque around 1.5 / 4.5 N·m old readings (raw 2218 / 2560 and mirrored).
    let mut cases: Vec<(usize, u16, u16)> = Vec::new();
    for j in 0..3 { for v in 2040..2057 { cases.push((j, v, ZERO)); } }
    for t in (2205..2232).chain(1864..1891) { cases.push((1, ZERO, t)); }
    for t in (2548..2572).chain(1524..1548) { cases.push((2, ZERO, t)); }
    let mut both = (0, 0);
    for (j, v, t) in cases {
        old.tick(j, v, t).await;
        new.tick(j, v, t).await;
        let (a, b) = (old.sup.is_at_safe_resting_pose().await, new.sup.is_at_safe_resting_pose().await);
        assert_eq!(a, b, "J{} v raw {v} t raw {t}", j + 1);
        if a { both.0 += 1 } else { both.1 += 1 }
    }
    assert!(both.0 > 0 && both.1 > 0, "the sweep covers both decisions: {both:?}");
}
