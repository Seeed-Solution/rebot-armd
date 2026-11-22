//! Joint tracking Phase 2, driver layer: MIT handover with post-switch confirmation, the 45 ms
//! deadline fence, and destructive transport faults, on the real `DamiaoArmDriver` over a
//! fake CAN bus with a deterministic clock. Every transmitted frame is inspected.

mod mit_support;

use std::time::Duration;

use armd::driver::{ArmDriver, DamiaoArmDriver, DamiaoControlMode, JointMode, JointTarget};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, MockClock};
use mit_support::{assert_no_flash_write, command_frames_to, decode_mit, FakeBus, MotorCmd};

const J4: usize = 3;
const FENCE: Duration = Duration::from_millis(45);
const HOLDOFF: Duration = Duration::from_millis(300);

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

/// Enabled driver on a clocked bus; J4 TIMEOUT commissioned to `timeout` counts first.
fn rig(timeout: u32) -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    bus.with(|s| for (i, m) in s.motors.iter_mut().enumerate() { m.q = 0.1 * i as f32; });
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(b601_limits()).unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    if timeout > 0 { d.commission_motor_timeout(&[J4], timeout).unwrap(); }
    d.set_torque(true).unwrap();
    bus.clear_log();
    (d, bus, clock)
}

fn modes_j4(mode: JointMode) -> Vec<JointMode> {
    (0..7).map(|i| if i == J4 { mode } else { JointMode::Position }).collect()
}

fn hold_at(d: &mut DamiaoArmDriver<FakeBus>) -> Vec<JointTarget> {
    let q = d.read_joint_states().unwrap();
    q.iter().map(|s| JointTarget::mit(s.position, 0.0, 12.0, 0.4, 0.5)).collect()
}

/// J4 switched to MIT with the confirmed handover and the fence armed.
fn mit_rig(timeout: u32) -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
    let (mut d, bus, clock) = rig(timeout);
    let hold = hold_at(&mut d);
    d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).expect("confirmed handover");
    assert_eq!(d.joint_control_modes()[J4], Some(DamiaoControlMode::Mit));
    d.set_mit_deadline_fence(Some(FENCE), HOLDOFF);
    bus.clear_log();
    (d, bus, clock)
}

fn targets(d: &mut DamiaoArmDriver<FakeBus>, offset: f32) -> Vec<JointTarget> {
    let q = d.read_joint_states().unwrap();
    q.iter().map(|s| JointTarget::mit(s.position + offset, 0.0, 10.0, 0.3, 0.4)).collect()
}

fn is_command(f: &CanFrame) -> bool { (1..=7).contains(&f.id) || (0x101..=0x107).contains(&f.id) }

#[test]
fn mit_switch_hold_failure() {
    // healthy: the hold's feedback arrives after the mode acknowledgement
    let (_, bus, _) = mit_rig(2000);
    assert_eq!(bus.with(|s| s.motors[J4].ctrl_mode()), damiao::CTRL_MODE_MIT);

    // mode accepted, hold and every later feedback lost: not confirmed, switched back
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| s.motors[J4].mute_feedback_after_mode_ack = true);
    let e = d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).unwrap_err();
    assert!(e.to_string().contains("feedback"), "{e}");
    assert_eq!(d.joint_control_modes()[J4], Some(DamiaoControlMode::PosVel), "rolled back");
    assert_eq!(bus.with(|s| s.motors[J4].ctrl_mode()), damiao::CTRL_MODE_POS_VEL);
    assert!(!d.joint_modes_uncertain());

    // a feedback sample queued before the switch does not confirm it
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| { s.push_feedback(J4); s.motors[J4].mute_feedback_after_mode_ack = true; });
    assert!(d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).is_err(), "pre-switch sample counted");

    // the plain switch (gravity compensation, Phase 1) still accepts the acknowledgement alone
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| s.motors[J4].mute_feedback_after_mode_ack = true);
    d.set_joint_modes(&modes_j4(JointMode::Mit), &hold).expect("pre-Phase-2 switch unchanged");

    // flush lost once: not confirmed, rollback confirmed
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| s.fail_flushes = 1);
    assert!(d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).is_err());
    assert_eq!(d.joint_control_modes()[J4], Some(DamiaoControlMode::PosVel));

    // hold transmit fails (and the rollback's MIT hold too): the mode stays uncertain
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| s.fail_tx_ids = vec![4]);
    assert!(d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).is_err());
    assert!(d.joint_modes_uncertain(), "failed rollback must leave the mode uncertain");

    // everything lost after the acknowledgement: rollback fails, uncertain
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| s.motors[J4].silent_after_mode_ack = true);
    assert!(d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).is_err());
    assert!(d.joint_modes_uncertain());
    assert_no_flash_write(&bus.tx_log());
}

#[test]
fn mit_stale_rx_never_replays() {
    let (mut d, bus, clock) = mit_rig(2000);
    // steady operation: a batch every 20 ms, every motor answers
    for _ in 0..5 {
        clock.advance(Duration::from_millis(20));
        let t = targets(&mut d, 0.0);
        d.write_joint_targets(&t).unwrap();
    }
    // J4 RX lost (motor still receives) after a read of every reply; read and write up to 44 ms
    // after its last feedback
    let _ = d.read_joint_states(); // replies become visible after an observed-empty queue
    let _ = d.read_joint_states();
    bus.with(|s| s.motors[J4].silent = true);
    let t0 = clock.now();
    bus.clear_log();
    clock.advance(Duration::from_millis(22));
    let t = targets(&mut d, 0.0);
    d.write_joint_targets(&t).unwrap();
    clock.advance(Duration::from_millis(22));
    let t = targets(&mut d, 0.0); // 44 ms
    assert!(d.mit_fence_trip().is_none(), "44 ms must not trip");
    d.write_joint_targets(&t).unwrap();
    let before_trip = bus.tx_log();
    assert_eq!(before_trip.iter().filter(|f| f.id == 0x7FF).count(), 0, "no status poll while armed: {before_trip:?}");
    assert_eq!(before_trip.len(), 14, "exactly two batches of seven frames");
    // 45 ms: trips on the read; nothing is sent, now or later
    bus.clear_log();
    clock.advance(Duration::from_millis(1));
    let _ = d.read_joint_states();
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("joint 4")), "{:?}", d.mit_fence_trip());
    assert!({ let t = targets(&mut d, 0.0); d.write_joint_targets(&t) }.is_err());
    for ms in [100u64, 250] {
        let now = clock.now();
        clock.advance(Duration::from_millis(ms) - (now - t0).min(Duration::from_millis(ms)));
        // RX backlog of other motors' samples
        bus.with(|s| for _ in 0..80 { s.push_feedback(0); });
        for _ in 0..4 { let _ = d.read_joint_states(); }
        assert!({ let t = targets(&mut d, 0.0); d.write_joint_targets(&t) }.is_err());
    }
    assert!(bus.tx_log().is_empty(), "frames sent after the trip: {:?}", bus.tx_log());
    // elapsed time alone never resumes polling (review #9): nothing at 500 ms and later
    let now = clock.now();
    clock.advance(Duration::from_millis(500) - (now - t0).min(Duration::from_millis(500)));
    for _ in 0..4 { clock.advance(Duration::from_millis(110)); let _ = d.read_joint_states(); }
    assert!(bus.tx_log().is_empty(), "polls resumed on elapsed time: {:?}", bus.tx_log());
    // after the operator's explicit clearance: only read-only status polls, never a command
    d.acknowledge_mit_fence_trip();
    for _ in 0..4 { clock.advance(Duration::from_millis(110)); let _ = d.read_joint_states(); }
    assert!({ let t = targets(&mut d, 0.0); d.write_joint_targets(&t) }.is_err(), "writes stay refused until a full enable");
    let late = bus.tx_log();
    assert!(!late.is_empty() && late.iter().all(|f| f.id == 0x7FF && f.data[2] == 0xCC), "{late:?}");
    assert!(command_frames_to(&late, 4).is_empty());
    assert_no_flash_write(&bus.tx_log());
}

#[test]
fn mit_fence_bounds_output_gaps_backlog_and_tx_stalls() {
    // a 45 ms gap between complete batches trips before the next transmit
    let (mut d, bus, clock) = mit_rig(2000);
    clock.advance(Duration::from_millis(20));
    let t = targets(&mut d, 0.0);
    d.write_joint_targets(&t).unwrap();
    clock.advance(Duration::from_millis(30));
    bus.with(|s| s.push_all_feedback());
    let t = targets(&mut d, 0.0);
    clock.advance(Duration::from_millis(15));
    bus.clear_log();
    assert!(d.write_joint_targets(&t).is_err());
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("gap")), "{:?}", d.mit_fence_trip());
    assert!(bus.tx_log().is_empty(), "{:?}", bus.tx_log());

    // an RX backlog the bounded read cannot drain trips (queued latency unknown)
    let (mut d, bus, _) = mit_rig(2000);
    bus.with(|s| for _ in 0..200 { s.push_feedback(1); });
    let _ = d.read_joint_states();
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("backlog")), "{:?}", d.mit_fence_trip());
    assert!(bus.tx_log().is_empty());

    // a stalled controller: retries stop at the deadline, no continuation to later joints
    let (mut d, bus, clock) = mit_rig(2000);
    clock.advance(Duration::from_millis(10));
    let t = targets(&mut d, 0.0);
    bus.with(|s| { s.tx_stall = true; s.stall_step = Duration::from_millis(15); });
    bus.clear_log();
    assert!(d.write_joint_targets(&t).is_err());
    let log = bus.tx_log();
    assert!(!log.is_empty() && log.iter().all(|f| f.id == 0x101), "only J1 attempts: {log:?}");
    assert!(log.len() <= 3, "retries past the 45 ms deadline: {}", log.len());
    assert!(d.mit_fence_trip().is_some());
}

#[test]
fn mit_partial_send_reconnect() {
    let (mut d, bus, clock) = mit_rig(2000);
    bus.with(|s| s.follow_commands = false); // measured stays where it is: targets are distinguishable
    clock.advance(Duration::from_millis(20));
    let old = targets(&mut d, 0.05);
    d.write_joint_targets(&old).unwrap();
    clock.advance(Duration::from_millis(20));
    let old = targets(&mut d, 0.05);
    bus.clear_log();
    bus.with(|s| s.fail_tx_after = Some(3));
    assert!(d.write_joint_targets(&old).is_err());
    let log = bus.tx_log();
    assert_eq!(log.iter().map(|f| f.id).collect::<Vec<_>>(), vec![0x101, 0x102, 0x103, 4], "J1-J3 accepted, J4 failed, J5-J7 never sent");
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("after 3 of 7")), "{:?}", d.mit_fence_trip());
    // the group latched: later reads (feedback stale) send nothing, writes are refused
    bus.with(|s| s.fail_tx_after = None);
    bus.clear_log();
    for _ in 0..5 {
        clock.advance(Duration::from_millis(50));
        let _ = d.read_joint_states();
        assert!(d.write_joint_targets(&old).is_err());
    }
    assert!(bus.tx_log().is_empty(), "{:?}", bus.tx_log());
    // reconnect = explicit rearm: modes restored and confirmed enable, no old target on the bus
    clock.advance(HOLDOFF);
    bus.with(|s| s.push_all_feedback());
    let _ = d.read_joint_states();
    bus.clear_log();
    d.set_torque(true).expect("rearm");
    assert!(d.mit_fence_trip().is_none());
    assert_eq!(d.joint_control_modes()[J4], Some(DamiaoControlMode::PosVel));
    let log = bus.tx_log();
    for f in log.iter().filter(|f| is_command(f) && f.data != damiao::CMD_ENABLE) {
        let i = if f.id >= 0x101 { f.id - 0x101 } else { f.id - 1 } as usize;
        let p = if f.id >= 0x101 { f32::from_le_bytes([f.data[0], f.data[1], f.data[2], f.data[3]]) }
            else { match decode_mit(&f.data, &b601_limits()[i]) { MotorCmd::Mit { p, .. } | MotorCmd::PosVel { p, .. } => p } };
        assert!((p - old[i].position).abs() > 0.02, "old target {} replayed on joint {}: {f:?}", old[i].position, i + 1);
    }
    assert_no_flash_write(&bus.tx_log());
}

#[test]
fn mit_watchdog_tx_loss() {
    // TX-only loss with the fence: the motor keeps its last complete command until its own
    // TIMEOUT (2000 counts at the documented 50 us = 100 ms), then drops out
    let (mut d, bus, clock) = mit_rig(2000);
    for _ in 0..3 {
        clock.advance(Duration::from_millis(20));
        let t = targets(&mut d, 0.0);
        d.write_joint_targets(&t).unwrap();
    }
    let (last_cmd, last_at) = bus.with(|s| (s.motors[J4].last_cmd, s.motors[J4].last_cmd_at.unwrap()));
    assert!(matches!(last_cmd, Some(MotorCmd::Mit { .. })));
    bus.with(|s| s.tx_lost = true);
    for _ in 0..12 {
        clock.advance(Duration::from_millis(10));
        let _ = d.read_joint_states();
        let _ = { let t = targets(&mut d, 0.0); d.write_joint_targets(&t) };
        let m = bus.with(|s| (s.motors[J4].enabled, s.motors[J4].last_cmd));
        if clock.now() < last_at + Duration::from_millis(100) {
            assert!(m.0 && m.1 == last_cmd, "the motor must hold its last complete command until expiry");
        }
    }
    let expired = bus.with(|s| s.motors[J4].watchdog_expired_at);
    assert_eq!(expired, Some(last_at + Duration::from_millis(100)), "expiry 100 ms after the last command");
    assert!(d.mit_fence_trip().is_some());

    // RX-only loss: with the fence nothing feeds the motor watchdog; it expires 100 ms after
    // the last supervisor batch
    let (mut d, bus, clock) = mit_rig(2000);
    clock.advance(Duration::from_millis(20));
    let t = targets(&mut d, 0.0);
    d.write_joint_targets(&t).unwrap();
    let _ = d.read_joint_states();
    let _ = d.read_joint_states();
    let lost_at = clock.now();
    bus.with(|s| for m in s.motors.iter_mut() { m.silent = true; });
    for _ in 0..30 {
        clock.advance(Duration::from_millis(10));
        let _ = d.read_joint_states();
        let _ = { let t = targets(&mut d, 0.0); d.write_joint_targets(&t) };
    }
    let (last_at, expired) = bus.with(|s| (s.motors[J4].last_cmd_at.unwrap(), s.motors[J4].watchdog_expired_at));
    assert!(last_at < lost_at + FENCE, "the fence stopped the batches within 45 ms of the last feedback");
    assert_eq!(expired, Some(last_at + Duration::from_millis(100)));

    // the same RX-only loss without the fence (the pre-Phase-2 gravity-compensation path):
    // driver recovery replays the cached MIT frame and keeps feeding the motor watchdog
    let (mut d, bus, clock) = rig(2000);
    let hold = hold_at(&mut d);
    d.set_joint_modes(&modes_j4(JointMode::Mit), &hold).unwrap();
    clock.advance(Duration::from_millis(20));
    let t = targets(&mut d, 0.0);
    d.write_joint_targets(&t).unwrap();
    bus.with(|s| for m in s.motors.iter_mut() { m.silent = true; });
    for _ in 0..30 { clock.advance(Duration::from_millis(10)); let _ = d.read_joint_states(); }
    assert_eq!(bus.with(|s| s.motors[J4].watchdog_expired_at), None, "replays feed the watchdog (why MIT tracking arms the fence)");

    // TIMEOUT 0 (power-up default): no motor-side protection at all; the gate refuses it
    let (mut d, bus, clock) = rig(0);
    let hold = hold_at(&mut d);
    d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).unwrap();
    d.set_mit_deadline_fence(Some(FENCE), HOLDOFF);
    bus.with(|s| s.tx_lost = true);
    for _ in 0..50 { clock.advance(Duration::from_millis(10)); let _ = d.read_joint_states(); }
    assert!(bus.with(|s| s.motors[J4].enabled && s.motors[J4].watchdog_expired_at.is_none()), "TIMEOUT 0 never expires");
    let rb = d.read_motor_identity(&[J4]);
    assert!(rb.is_err() || rb.unwrap()[0].timeout_raw == 0);
    assert_no_flash_write(&bus.tx_log());
}

fn reg_reply(id: u16, op: u8, rid: u8, value: [u8; 4]) -> CanFrame {
    let [lo, hi] = id.to_le_bytes();
    CanFrame::new_standard(0x10 + id, &[lo, hi, op, rid, value[0], value[1], value[2], value[3]]).unwrap()
}

/// Review #2: replies that were already queued when a request went out (stale) never satisfy a
/// register read, a TIMEOUT acknowledgement or a mode-switch confirmation: each transaction first
/// drains the receive queue to an observed-empty boundary, and only frames dequeued after it count.
/// The real replies are withheld; the stale ones are queued before the transaction's boundary.
#[test]
fn mit_stale_queued_replies_never_satisfy_a_transaction() {
    let limits = b601_limits()[J4];
    let stale_identity = |id: u16| -> Vec<CanFrame> {
        let mut v = Vec::new();
        for (rid, val) in [(damiao::RID_ESC_ID, u32::from(id).to_le_bytes()), (damiao::RID_MST_ID, (0x10 + u32::from(id)).to_le_bytes()),
            (damiao::RID_SW_VER, mit_support::SW_VER.to_le_bytes()), (damiao::RID_CTRL_MODE, damiao::CTRL_MODE_POS_VEL.to_le_bytes()),
            (damiao::RID_PMAX, limits.p_max.to_le_bytes()), (damiao::RID_VMAX, limits.v_max.to_le_bytes()),
            (damiao::RID_TMAX, limits.t_max.to_le_bytes()), (damiao::RID_TIMEOUT, 2000u32.to_le_bytes())] {
            v.push(reg_reply(id, damiao::REG_READ, rid, val));
        }
        v
    };
    for immediate in [false, true] {
        // identity readback: J4 silent, a full set of old matching replies queued
        let (mut d, bus, _) = rig(0);
        bus.with(|s| { s.immediate_replies = immediate; s.motors[J4].silent = true; s.rx.extend(stale_identity(4)); });
        assert!(d.read_motor_identity(&[J4]).is_err(), "stale replies satisfied the identity read");

        // TIMEOUT write: an old matching acknowledgement queued, the real one withheld
        let (mut d, bus, _) = rig(0);
        bus.with(|s| {
            s.immediate_replies = immediate;
            s.motors[J4].no_write_ack = true;
            // queued with the identity read of the same call, i.e. before the write's boundary
            s.inject_on_next_tx = vec![reg_reply(4, damiao::REG_WRITE, damiao::RID_TIMEOUT, 2000u32.to_le_bytes())];
        });
        assert!(d.commission_motor_timeout(&[J4], 2000).is_err(), "a stale acknowledgement satisfied the TIMEOUT write");

        // confirmed mode switch: an old CTRL_MODE acknowledgement and an old feedback queued, the
        // motor answers nothing after the switch
        let (mut d, bus, _) = rig(2000);
        let hold = hold_at(&mut d);
        let fb = bus.with(|s| s.motors[J4].feedback_frame());
        bus.with(|s| {
            s.immediate_replies = immediate;
            s.motors[J4].no_write_ack = true;
            s.motors[J4].mute_feedback = true;
            // queued with the MIT range read that precedes the mode write
            s.inject_on_next_tx = vec![reg_reply(4, damiao::REG_WRITE, damiao::RID_CTRL_MODE, damiao::CTRL_MODE_MIT.to_le_bytes()), fb];
        });
        assert!(d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).is_err(), "stale ACK + feedback confirmed the switch");
    }
}

/// Review round 3, #2: a genuine reply that arrives right after its request (before the driver
/// observes an empty queue again) counts: TIMEOUT write, identity read and mode switch succeed
/// with immediate replies.
#[test]
fn mit_immediate_replies_are_accepted() {
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    bus.with(|s| s.immediate_replies = true);
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(b601_limits()).unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    let res = d.commission_motor_timeout(&[J4], 2000).expect("immediate ACK accepted");
    assert_eq!(res[0].readback.timeout_raw, 2000);
    d.set_torque(true).unwrap();
    let hold = hold_at(&mut d);
    d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).expect("immediate ACK + feedback confirm the switch");
}

/// Review #6: the rollback of a failed switch needs fresh feedback after its acknowledgement too
/// (here the motor acknowledges register writes but sends no feedback at all): unconfirmed.
#[test]
fn mit_rollback_requires_post_ack_feedback() {
    let (mut d, bus, _) = rig(2000);
    let hold = hold_at(&mut d);
    bus.with(|s| s.motors[J4].no_feedback = true);
    assert!(d.set_joint_modes_confirmed(&modes_j4(JointMode::Mit), &hold).is_err());
    assert!(d.joint_modes_uncertain(), "rollback confirmed by an acknowledgement alone");
}

/// Review #6: a hand back (confirmed switch while the fence is armed) after the deadline sends
/// nothing: no mode write, no hold, no poll.
#[test]
fn mit_hand_back_respects_the_fence() {
    let (mut d, bus, clock) = mit_rig(2000);
    clock.advance(Duration::from_millis(20));
    let t = targets(&mut d, 0.0);
    d.write_joint_targets(&t).unwrap();
    clock.advance(Duration::from_millis(50)); // past the deadline, nothing read
    bus.clear_log();
    let hold = (0..7).map(|i| JointTarget::mit(t[i].position, 0.0, 12.0, 0.4, 0.5)).collect::<Vec<_>>();
    assert!(d.set_joint_modes_confirmed(&[JointMode::Position; 7], &hold).is_err());
    assert!(bus.tx_log().is_empty(), "sent after the deadline: {:?}", bus.tx_log());
    assert!(d.mit_fence_trip().is_some());
}

/// Review #7: the 5 ms RX budget running out before the queue is drained (slow host, fewer than
/// 64 frames) is a backlog of unknown age: it trips, the dequeued samples do not count as fresh.
#[test]
fn mit_rx_time_budget_exhaustion_is_a_backlog() {
    let (mut d, bus, _) = mit_rig(2000);
    bus.with(|s| { for _ in 0..10 { s.push_feedback(0); } s.rx_step = Duration::from_millis(2); });
    let _ = d.read_joint_states();
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("backlog")), "{:?}", d.mit_fence_trip());
}

/// Review #7: a batch whose last transmit completes past the deadline (slow controller) trips the
/// fence; its completion time never becomes the new reference of the deadline.
#[test]
fn mit_batch_completing_past_the_deadline_trips() {
    let (mut d, bus, clock) = mit_rig(2000);
    clock.advance(Duration::from_millis(10));
    let t = targets(&mut d, 0.0);
    d.write_joint_targets(&t).unwrap();
    let _ = d.read_joint_states();
    clock.advance(Duration::from_millis(38));
    let _ = d.read_joint_states();
    let t = targets(&mut d, 0.0); // fresh feedback; 38 ms after the last batch
    bus.with(|s| s.tx_step = Duration::from_millis(1)); // 7 frames: completes 45 ms after the last batch
    let res = d.write_joint_targets(&t);
    assert!(res.is_err() && d.mit_fence_trip().is_some(), "completion past the deadline accepted: {res:?}");
}
