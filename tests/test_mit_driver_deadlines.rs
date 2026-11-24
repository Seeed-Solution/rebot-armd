//! armd 1.6 RT core, track A: deadline-driven torque handshakes and per-attempt transmit checks
//! on the real `DamiaoArmDriver` over the fake CAN bus with a virtual clock. Every transmitted
//! frame and its fake time is inspected.

mod mit_support;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use armd::driver::{is_estop_inhibited, ArmDriver, DamiaoArmDriver, JointMode, JointTarget, TorqueOutcome, TorqueProgress};
use armd::model::ArmConfig;
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::rt_core::urgent::EstopGate;
use armd::transport::{CanFrame, MockClock};
use mit_support::FakeBus;

const J4: usize = 3;
const FENCE: Duration = Duration::from_millis(45);
const RELEASE: Duration = Duration::from_millis(20);
const RX_PASS: Duration = Duration::from_millis(5);

fn ms(n: u64) -> Duration { Duration::from_millis(n) }

fn b601_limits() -> Vec<DamiaoLimits> {
    ArmConfig::rebot_dm().damiao_ranges.unwrap().iter().map(|r| r.limits()).collect()
}

/// Driver with fresh feedback, torque off, ranges checked (no register traffic in the enable).
fn rig() -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
    let clock = MockClock::new();
    let bus = FakeBus::with_clock(&b601_limits(), clock.clone());
    let mut d = DamiaoArmDriver::new(bus.clone(), vec![1, 2, 3, 4, 5, 6, 7]).with_joint_limits(b601_limits()).unwrap();
    bus.with(|s| s.push_all_feedback());
    d.read_joint_states().unwrap();
    d.check_mapping_ranges();
    bus.clear_log();
    (d, bus, clock)
}

fn enabled_rig() -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
    let (mut d, bus, clock) = rig();
    d.set_torque(true).expect("enable");
    bus.clear_log();
    (d, bus, clock)
}

/// Torque command frames (enable / disable) with their fake time relative to `t0`.
fn torque_frames(bus: &FakeBus, t0: Instant) -> Vec<(u32, [u8; 8], Duration)> {
    bus.with(|s| s.tx_log.iter().zip(&s.tx_times)
        .filter(|(f, _)| f.data == damiao::CMD_ENABLE || f.data == damiao::CMD_DISABLE)
        .map(|(f, t)| (f.id, f.data, t.unwrap().saturating_duration_since(t0)))
        .collect())
}

fn ids(frames: &[(u32, [u8; 8], Duration)]) -> Vec<u32> { frames.iter().map(|f| f.0).collect() }

#[test]
fn missing_enable_confirmation_retries_only_the_silent_joint_and_fails_at_the_deadline() {
    let (mut d, bus, clock) = rig();
    bus.with(|s| s.motors[J4].silent = true);
    let t0 = clock.now();
    let e = d.set_torque(true).unwrap_err();
    assert!(e.to_string().contains("within 150 ms"), "{e}");
    let f = torque_frames(&bus, t0);
    assert_eq!(ids(&f), vec![1, 2, 3, 4, 5, 6, 7, 4, 4], "initial batch, then retries to J4 only");
    assert!(f[7].2 >= ms(25) && f[7].2 < ms(26), "first retry at 25 ms: {:?}", f[7].2);
    assert!(f[8].2 >= ms(75) && f[8].2 < ms(76), "second retry at 75 ms: {:?}", f[8].2);
    let end = clock.now().saturating_duration_since(t0);
    assert!(end >= ms(150) && end < ms(151), "failure reported at {end:?}");
    assert!(!d.is_enabled());
    assert!(!d.torque_transition_pending());
}

#[test]
fn late_and_lost_confirmations() {
    // a reply 30 ms late (B601-DM J3 after a set-zero) confirms inside the window
    let (mut d, bus, clock) = rig();
    bus.with(|s| s.motors[2].torque_reply_delay = ms(30));
    let t0 = clock.now();
    d.set_torque(true).expect("late reply inside the window");
    let took = clock.now().saturating_duration_since(t0);
    assert!(took >= ms(30) && took < ms(31), "{took:?}");
    assert_eq!(ids(&torque_frames(&bus, t0)), vec![1, 2, 3, 4, 5, 6, 7, 3], "the 25 ms retry goes to J3 only");

    // the first enable frame to J2 is lost: the retry at 25 ms confirms it
    let (mut d, bus, clock) = rig();
    bus.with(|s| s.motors[1].drop_torque_frames = 1);
    let t0 = clock.now();
    d.set_torque(true).expect("confirmed by the retry");
    let took = clock.now().saturating_duration_since(t0);
    assert!(took >= ms(25) && took <= ms(26), "{took:?}");

    // a reply arriving exactly at the deadline does not count (window end exclusive) ...
    let (mut d, bus, clock) = rig();
    bus.with(|s| { s.motors[2].torque_reply_delay = ms(150); });
    let t0 = clock.now();
    assert!(d.set_torque(true).is_err(), "a confirmation at 150 ms counted");
    assert!(clock.now().saturating_duration_since(t0) < ms(151));
    // ... one received before it does
    let (mut d, bus, _) = rig();
    bus.with(|s| { s.motors[2].torque_reply_delay = ms(149); });
    d.set_torque(true).expect("a confirmation at 149 ms");
}

#[test]
fn partial_disable_confirmation_is_unconfirmed_not_success() {
    let (mut d, bus, clock) = enabled_rig();
    bus.with(|s| s.motors[4].silent = true);
    let t0 = clock.now();
    let mut p = d.begin_torque(false).unwrap();
    assert!(d.torque_transition_pending());
    let mut served = 0;
    while let TorqueProgress::Pending { wake } = p {
        assert!(wake >= clock.now() || served == 0);
        // serve exactly at the requested wake (or 0.5 ms later when it is now)
        let now = clock.now();
        clock.advance(if wake > now { wake - now } else { Duration::from_micros(500) });
        p = d.poll_torque(clock.now() + RX_PASS).unwrap();
        served += 1;
        assert!(served < 1000);
    }
    assert_eq!(p, TorqueProgress::Done(TorqueOutcome::Unconfirmed));
    let f = torque_frames(&bus, t0);
    assert_eq!(ids(&f), vec![1, 2, 3, 4, 5, 6, 7, 5, 5], "retries to J5 only");
    assert_eq!(f[7].2, ms(25));
    assert_eq!(f[8].2, ms(75));
    assert_eq!(clock.now().saturating_duration_since(t0), ms(150), "served at its deadline");
    assert!(served <= 4, "woken only at the retries and the deadline: {served} services");
}

#[test]
fn success_is_withheld_until_confirmation() {
    let (mut d, bus, clock) = rig();
    bus.with(|s| s.motors[2].torque_reply_delay = ms(10));
    let p = d.begin_torque(true).unwrap();
    assert!(matches!(p, TorqueProgress::Pending { .. }));
    assert!(!d.is_enabled());
    let hold: Vec<JointTarget> = (0..7).map(|_| JointTarget::position(0.0)).collect();
    bus.clear_log();
    assert!(d.write_joint_targets(&hold).is_err(), "a target before the confirmation");
    assert!(bus.tx_log().is_empty());
    assert!(d.begin_torque(true).is_err(), "a second enable while one is pending");
    assert!(matches!(d.poll_torque(clock.now() + RX_PASS).unwrap(), TorqueProgress::Pending { .. }));
    assert!(!d.is_enabled());
    clock.advance(ms(10));
    assert_eq!(d.poll_torque(clock.now() + RX_PASS).unwrap(), TorqueProgress::Done(TorqueOutcome::Confirmed));
    assert!(d.is_enabled());
    d.write_joint_targets(&hold).expect("targets after the confirmation");
    assert!(d.poll_torque(clock.now() + RX_PASS).is_err(), "nothing pending");
}

#[test]
fn disable_replaces_a_pending_enable() {
    let (mut d, bus, clock) = rig();
    bus.with(|s| s.motors[J4].silent = true);
    d.begin_torque(true).unwrap();
    let _ = d.poll_torque(clock.now() + RX_PASS).unwrap();
    bus.with(|s| s.motors[J4].silent = false);
    bus.clear_log();
    let t0 = clock.now();
    let mut p = d.begin_torque(false).unwrap();
    while let TorqueProgress::Pending { .. } = p {
        clock.advance(Duration::from_micros(500));
        p = d.poll_torque(clock.now() + RX_PASS).unwrap();
    }
    assert_eq!(p, TorqueProgress::Done(TorqueOutcome::Confirmed));
    let f = torque_frames(&bus, t0);
    assert!(f.iter().all(|x| x.1 == damiao::CMD_DISABLE), "no enable retry after the disable: {f:?}");
    assert!(!d.is_enabled());
    assert!(!d.torque_transition_pending());
}

/// Service cadence: one service per 20 ms release vs. a wake at the requested instant. A motor
/// whose first enable frame is lost is confirmed by the retry; with releases only the retry and
/// the deadline are served late.
#[test]
fn release_cadence_delays_retries_and_expiry() {
    fn run(serve_at_wake: bool, lose_first: bool, silent: bool) -> (Option<Duration>, Duration, Vec<Duration>) {
        let (mut d, bus, clock) = rig();
        bus.with(|s| { s.motors[1].drop_torque_frames = usize::from(lose_first); s.motors[1].silent = silent; });
        let t0 = clock.now();
        let mut p = d.begin_torque(true).unwrap();
        let mut next_release = t0 + RELEASE;
        let result = loop {
            let wake = match p {
                TorqueProgress::Pending { wake } => wake,
                TorqueProgress::Done(_) => break Ok(()),
            };
            let now = clock.now();
            while next_release <= now { next_release += RELEASE; }
            // the executor serves every release; with `serve_at_wake` also earlier at the wake
            let wake = if wake > now { wake } else { now + Duration::from_micros(500) };
            let at = if serve_at_wake { wake.min(next_release) } else { next_release };
            clock.advance(at - now);
            match d.poll_torque(clock.now() + RX_PASS) { Ok(np) => p = np, Err(e) => break Err(e) }
        };
        let end = clock.now().saturating_duration_since(t0);
        let retries: Vec<Duration> = torque_frames(&bus, t0).iter().skip(7).map(|f| f.2).collect();
        (result.is_ok().then_some(end), end, retries)
    }
    let (ok_w, _, r_w) = run(true, true, false);
    let (ok_r, _, r_r) = run(false, true, false);
    let (_, fail_w, _) = run(true, false, true);
    let (_, fail_r, _) = run(false, false, true);
    eprintln!("lost first frame: wake-driven confirmed at {ok_w:?} (retries {r_w:?}), 20 ms releases at {ok_r:?} (retries {r_r:?})");
    eprintln!("silent motor: failure reported at {fail_w:?} (wake-driven) vs {fail_r:?} (20 ms releases)");
    assert_eq!(r_w.first().copied(), Some(ms(25)));
    assert_eq!(r_r.first().copied(), Some(ms(40)), "first retry waits for the 40 ms release");
    assert!(ok_w.unwrap() < ok_r.unwrap());
    assert_eq!(fail_w, ms(150));
    assert_eq!(fail_r, ms(160), "expiry detected at the first release at or after 150 ms");
}

fn mit_rig() -> (DamiaoArmDriver<FakeBus>, FakeBus, MockClock) {
    let (mut d, bus, clock) = enabled_rig();
    let q = d.read_joint_states().unwrap();
    let hold: Vec<JointTarget> = q.iter().map(|s| JointTarget::mit(s.position, 0.0, 12.0, 0.4, 0.5)).collect();
    let modes: Vec<JointMode> = (0..7).map(|i| if i == J4 { JointMode::Mit } else { JointMode::Position }).collect();
    d.set_joint_modes_confirmed(&modes, &hold).expect("handover");
    d.set_mit_deadline_fence(Some(FENCE), ms(300));
    bus.clear_log();
    (d, bus, clock)
}

fn mit_targets(d: &mut DamiaoArmDriver<FakeBus>) -> Vec<JointTarget> {
    let q = d.read_joint_states().unwrap();
    q.iter().map(|s| JointTarget::mit(s.position, 0.0, 10.0, 0.3, 0.4)).collect()
}

fn is_command(f: &CanFrame) -> bool { (1..=7).contains(&f.id) || (0x101..=0x107).contains(&f.id) }

/// The fence is checked before every transmit attempt: a frame that would start at
/// exactly 45 ms after the last complete batch is not sent, and the group keeps no cached output.
#[test]
fn fence_expires_between_attempts_at_exactly_45_ms() {
    for (before, expect_attempts) in [(Duration::ZERO, 1usize), (Duration::from_nanos(1), 2)] {
        let (mut d, bus, clock) = mit_rig();
        let armed = clock.now();
        clock.advance(ms(20));
        bus.with(|s| s.push_all_feedback());
        let t = mit_targets(&mut d);
        // first frame at 44.9 ms (minus `before`); each transmit advances the clock by 0.1 ms,
        // so the second frame would start at 45.0 ms (minus `before`) with no retry sleep.
        let now = clock.now().saturating_duration_since(armed);
        clock.advance(ms(45) - Duration::from_micros(100) - before - now);
        bus.with(|s| { s.tx_step = Duration::from_micros(100); });
        bus.clear_log();
        let e = d.write_joint_targets(&t).unwrap_err();
        assert!(!is_estop_inhibited(&e));
        let log = bus.tx_log();
        assert_eq!(log.len(), expect_attempts, "attempts with {before:?} margin: {log:?}");
        assert_eq!(log[0].id, 0x101, "first frame is J1: {log:?}");
        if expect_attempts == 2 {
            assert_eq!(log[1].id, 0x102, "second frame is J2: {log:?}");
        }
        assert!(d.mit_fence_trip().is_some_and(|r| r.contains("gap")), "{:?}", d.mit_fence_trip());
        // after the trip: no cached frame is replayed, no write is accepted
        bus.with(|s| { s.tx_step = Duration::ZERO; s.push_all_feedback(); });
        bus.clear_log();
        for _ in 0..5 {
            clock.advance(ms(50));
            let _ = d.read_joint_states();
            assert!(d.write_joint_targets(&t).is_err());
        }
        assert!(!bus.tx_log().iter().any(is_command), "cached output after the trip: {:?}", bus.tx_log());
    }
}

/// An E-stop requested between two attempts of one frame stops the batch at once.
#[test]
fn estop_between_transmit_attempts_sends_nothing_more() {
    let requested = Arc::new(AtomicU64::new(0));
    let applied = Arc::new(AtomicU64::new(0));
    // fenced (MIT tracking): the group trips with the E-stop reason
    let (mut d, bus, clock) = mit_rig();
    d.set_estop_gate(EstopGate::new(requested.clone(), applied.clone()));
    clock.advance(ms(10));
    let t = mit_targets(&mut d);
    let r = requested.clone();
    bus.with(|s| { s.tx_stall = true; s.stall_step = Duration::from_micros(10); s.on_tx = Some(Arc::new(move || { r.fetch_add(1, Ordering::SeqCst); })); });
    bus.clear_log();
    let e = d.write_joint_targets(&t).unwrap_err();
    assert!(is_estop_inhibited(&e), "{e}");
    assert_eq!(bus.tx_log().len(), 1, "{:?}", bus.tx_log());
    assert!(d.mit_fence_trip().is_some_and(|r| r.contains("E-stop")), "{:?}", d.mit_fence_trip());

    // unfenced POS_VEL batch: refused with ESTOP_INHIBITED after the one attempt
    let requested = Arc::new(AtomicU64::new(0));
    let (mut d, bus, _) = enabled_rig();
    d.set_estop_gate(EstopGate::new(requested.clone(), applied));
    let hold: Vec<JointTarget> = d.read_joint_states().unwrap().iter().map(|s| JointTarget::position(s.position)).collect();
    let r = requested.clone();
    bus.with(|s| { s.tx_stall = true; s.on_tx = Some(Arc::new(move || { r.fetch_add(1, Ordering::SeqCst); })); });
    bus.clear_log();
    let e = d.write_joint_targets(&hold).unwrap_err();
    assert!(is_estop_inhibited(&e), "{e}");
    assert_eq!(bus.tx_log().len(), 1, "retries after the E-stop request: {:?}", bus.tx_log());
}

/// Disable during a gravity-comp style MIT session still goes out while an E-stop is pending
/// (torque removal is never inhibited by the gate) and is confirmed.
#[test]
fn disable_is_not_inhibited_by_a_pending_estop() {
    let requested = Arc::new(AtomicU64::new(1));
    let applied = Arc::new(AtomicU64::new(0));
    let (mut d, bus, clock) = enabled_rig();
    d.set_estop_gate(EstopGate::new(requested, applied));
    let t0 = clock.now();
    d.set_torque(false).expect("disable");
    assert_eq!(ids(&torque_frames(&bus, t0)), vec![1, 2, 3, 4, 5, 6, 7]);
    assert!(!d.is_enabled());
}

/// The service deadline caps the receive pass: past it nothing is read, so nothing confirms; the
/// next service within its budget does. With the MIT fence off the cut pass trips nothing.
#[test]
fn service_deadline_caps_the_receive_pass() {
    let (mut d, bus, clock) = rig();
    bus.with(|s| s.rx_step = Duration::from_micros(100));
    let p = d.begin_torque(true).unwrap();
    assert!(matches!(p, TorqueProgress::Pending { .. }));
    // replies are queued; a service whose deadline already passed reads none of them
    let _ = d.poll_torque(clock.now()).unwrap(); // moves pending replies into the queue
    let before = clock.now();
    assert!(matches!(d.poll_torque(before).unwrap(), TorqueProgress::Pending { .. }), "confirmed without a receive pass");
    assert_eq!(clock.now(), before, "frames read past the service deadline");
    assert!(!d.is_enabled());
    // within budget: the replies confirm the enable
    let mut p = d.poll_torque(clock.now() + RX_PASS).unwrap();
    for _ in 0..4 {
        if p == TorqueProgress::Done(TorqueOutcome::Confirmed) { break; }
        clock.advance(Duration::from_micros(500));
        p = d.poll_torque(clock.now() + RX_PASS).unwrap();
    }
    assert_eq!(p, TorqueProgress::Done(TorqueOutcome::Confirmed));
    assert!(d.is_enabled());
}
