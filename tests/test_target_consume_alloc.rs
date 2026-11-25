//! Track D: consuming an accepted servo or policy target from the latest-target mailbox
//! neither allocates nor frees once the servo buffers hold a target (steady streaming). A
//! counting allocator records the calling thread's allocations and deallocations around
//! `CoreOps::consume_target` on the supervisor's production adapter (mock follower).

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

use armd::rt_core::broker::{Authority, BootId};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetAuthority, TargetFrame, MAX_DOF};
use armd::rt_core::ops::{CommitCtx, CoreOps};
use armd::{ArmConfig, SupervisorInner};

thread_local! {
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn count(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) { let _ = c.try_with(|n| n.set(n.get() + 1)); }
}

// SAFETY: forwards to System; the counters are const-initialized thread-locals without
// destructors and do not allocate.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { count(&ALLOCS); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { count(&ALLOCS); unsafe { System.alloc_zeroed(l) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { count(&FREES); unsafe { System.dealloc(p, l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 { count(&ALLOCS); count(&FREES); unsafe { System.realloc(p, l, n) } }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// (allocations, deallocations) of this thread inside `f`.
fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

const BOOT: BootId = BootId(3);
const CONN: &str = "conn-alloc";

fn supervisor() -> (SupervisorInner, u64) {
    let mut inner = SupervisorInner::new(ArmConfig { lease_timeout_ms: 600_000, ..ArmConfig::rebot_dm() });
    inner.tick(0.02);
    { let __s = inner.current_admission_stamp(); inner.check_admission(__s).and_then(|__a| armd::SupervisorInner::enable_torque(__a)) }.expect("arm");
    let gen = inner.acquire_lease(CONN).expect("lease");
    (inner, gen)
}

fn ctx(now: Instant) -> CommitCtx { CommitCtx { arm: ArmId::LOCAL, boot: BOOT, now, stop_generation: 0 } }

fn frame(inner: &SupervisorInner, stream: StreamId, authority: Authority, policy_generation: u64, base: &[f32], dq: f32, seq: u64, now: Instant) -> TargetFrame {
    let mut q = [0.0; MAX_DOF];
    q[..base.len()].copy_from_slice(base);
    q[0] += dq;
    let _ = inner;
    TargetFrame { arm: ArmId::LOCAL, boot: BOOT, stream, seq, authority, policy_generation, input_at: now, vision_at: None,
        vision_frame_id: None, deadline: now + Duration::from_secs(1), dof: base.len() as u8, q, replaceable: policy_generation == 0 }
}

#[test]
fn consuming_servo_targets_does_not_allocate_in_steady_streaming() {
    let (mut inner, gen) = supervisor();
    { let s = inner.current_admission_stamp(); let a = inner.check_admission(s).unwrap(); armd::SupervisorInner::authorize_servo_stream(a, gen).unwrap(); }
    let stream = inner.core.target_authority().stream.expect("authorized");
    let base = inner.commanded_positions.clone();
    let now = Instant::now();
    let authority = Authority { lease_generation: gen, epoch: inner.safety.epoch_id(), config_revision: inner.workspace_revision, ..Authority::default() };
    // warm-up: the first target fills the servo buffers
    for seq in 1..=3 {
        let f = frame(&inner, stream, authority, 0, &base, 0.0005 * seq as f32, seq, now);
        inner.consume_target(&f, &ctx(now)).expect("applied");
    }
    let frames: Vec<TargetFrame> = (4..=20).map(|seq| frame(&inner, stream, authority, 0, &base, 0.0005 * seq as f32, seq, now)).collect();
    let (results, allocs, frees) = counted(|| {
        let mut ok = 0;
        for f in &frames { if inner.consume_target(f, &ctx(now)).is_ok() { ok += 1; } }
        ok
    });
    assert_eq!(results, frames.len(), "every target applied");
    assert_eq!((allocs, frees), (0, 0), "servo consumption touched the allocator");
    assert!((inner.servo_target.as_ref().unwrap()[0] - (base[0] + 0.01)).abs() < 1e-6);
}

#[test]
fn refusing_a_servo_target_for_its_authority_does_not_allocate() {
    let (mut inner, gen) = supervisor();
    inner.core.authorize_stream(TargetAuthority { stream: Some(StreamId(5)), ..TargetAuthority::default() });
    let base = inner.commanded_positions.clone();
    let now = Instant::now();
    let authority = Authority { lease_generation: gen, epoch: inner.safety.epoch_id(), ..Authority::default() };
    let wrong_stream = frame(&inner, StreamId(6), authority, 0, &base, 0.001, 1, now);
    let stale_epoch = frame(&inner, StreamId(5), Authority { epoch: authority.epoch + 1, ..authority }, 0, &base, 0.001, 2, now);
    let (codes, allocs, frees) = counted(|| {
        [inner.consume_target(&wrong_stream, &ctx(now)).unwrap_err().code, inner.consume_target(&stale_epoch, &ctx(now)).unwrap_err().code]
    });
    assert_eq!(codes, ["STREAM_NOT_AUTHORIZED", "EPOCH_MISMATCH"]);
    assert_eq!((allocs, frees), (0, 0));
}

#[test]
fn refusing_a_servo_target_for_soft_limit_or_commissioning_does_not_allocate() {
    let (mut inner, gen) = supervisor();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        armd::SupervisorInner::authorize_servo_stream(a, gen).unwrap();
    }
    let stream = inner.core.target_authority().stream.expect("authorized");
    let base = inner.commanded_positions.clone();
    let now = Instant::now();
    let authority = Authority {
        lease_generation: gen,
        epoch: inner.safety.epoch_id(),
        config_revision: inner.workspace_revision,
        ..Authority::default()
    };
    let oob = frame(&inner, stream, authority, 0, &base, 50.0, 1, now);
    let (oob_err, oob_allocs, oob_frees) = counted(|| inner.consume_target(&oob, &ctx(now)).unwrap_err());
    assert_eq!(oob_err.code, "SOFT_LIMIT_EXCEEDED");
    assert_eq!(
        oob_err.detail,
        armd::rt_core::mailbox::RefusalDetail::Limit { joint: Some(0) }
    );
    assert_eq!(
        (oob_allocs, oob_frees),
        (0, 0),
        "soft-limit refusal on RT servo intake must not allocate"
    );

    inner.config.active_tool.commissioning_required = true;
    let valid = frame(&inner, stream, authority, 0, &base, 0.001, 2, now);
    let (comm_err, comm_allocs, comm_frees) = counted(|| inner.consume_target(&valid, &ctx(now)).unwrap_err());
    assert_eq!(comm_err.code, "COMMISSIONING_REQUIRED");
    assert_eq!(
        (comm_allocs, comm_frees),
        (0, 0),
        "commissioning refusal on RT servo intake must not allocate"
    );
}

#[cfg(feature = "policy-runtime")]
#[test]
fn consuming_live_policy_targets_does_not_allocate_in_steady_streaming() {
    use armd::policy_runtime::{PolicyMode, PolicyRunStart};
    let (mut inner, gen) = supervisor();
    let epoch = inner.safety.epoch_id();
    let action_id = { let s = inner.current_admission_stamp(); let a = inner.check_admission(s).unwrap(); armd::SupervisorInner::begin_policy_run(a, PolicyRunStart {
        epoch_id: epoch, connection_id: CONN.into(), lease_generation: gen, policy_id: "p".into(), artifact_digest: "d".into(),
        backend: "mlp".into(), mode: PolicyMode::Live, sim_only: false, max_first_step_rad: None, max_step_rad: None, tag: None,
        i_am_at_the_arm: false, verified_release_gate: None,
    }) }.expect("run");
    let stamp = inner.policy_target_stamp(action_id, 0).expect("stamp");
    let stream = inner.core.target_authority().stream.expect("policy stream authorized");
    let base = inner.measured_positions.clone();
    let now = Instant::now();
    for seq in 1..=3 {
        let f = frame(&inner, stream, stamp.authority, stamp.authorization, &base, 0.0005 * seq as f32, seq, now);
        inner.consume_target(&f, &ctx(now)).expect("applied");
    }
    let frames: Vec<TargetFrame> = (4..=20).map(|seq| frame(&inner, stream, stamp.authority, stamp.authorization, &base, 0.0005 * seq as f32, seq, now)).collect();
    let (results, allocs, frees) = counted(|| {
        let mut ok = 0;
        for f in &frames { if inner.consume_target(f, &ctx(now)).is_ok() { ok += 1; } }
        ok
    });
    assert_eq!(results, frames.len(), "every target applied");
    assert_eq!((allocs, frees), (0, 0), "policy consumption touched the allocator");
    assert_eq!(inner.policy_run.as_ref().unwrap().steps, 20);
}

/// A stop between two servo targets keeps the target buffer: the stop and the first target
/// after it neither free nor allocate.
#[test]
fn a_stop_and_the_next_servo_target_do_not_touch_the_allocator() {
    let (mut inner, gen) = supervisor();
    { let s = inner.current_admission_stamp(); let a = inner.check_admission(s).unwrap(); armd::SupervisorInner::authorize_servo_stream(a, gen).unwrap(); }
    let stream = inner.core.target_authority().stream.expect("authorized");
    let base = inner.commanded_positions.clone();
    let now = Instant::now();
    let authority = Authority { lease_generation: gen, epoch: inner.safety.epoch_id(), config_revision: inner.workspace_revision, ..Authority::default() };
    for seq in 1..=3 {
        let f = frame(&inner, stream, authority, 0, &base, 0.0005 * seq as f32, seq, now);
        inner.consume_target(&f, &ctx(now)).expect("applied");
    }
    let after = frame(&inner, stream, authority, 0, &base, 0.002, 4, now);
    let (ok, allocs, frees) = counted(|| {
        inner.stop_motion();
        let stopped = inner.servo_target.is_none();
        stopped && inner.consume_target(&after, &ctx(now)).is_ok()
    });
    assert!(ok, "stopped, then the next target applied");
    assert_eq!((allocs, frees), (0, 0), "stop + next target touched the allocator");
    assert!((inner.servo_target.as_ref().unwrap()[0] - (base[0] + 0.002)).abs() < 1e-6);
}

/// The calibration digest a live step compares with the release gate is computed when the
/// calibration is installed: reading it does not serialize, hash or allocate.
#[cfg(feature = "policy-runtime")]
#[test]
fn the_calibration_digest_is_cached_at_install() {
    use armd::calibration::CalibrationFile;
    use armd::policy_runtime::EMPTY_CALIBRATION_DIGEST;
    let (mut inner, _) = supervisor();
    assert_eq!(inner.active_calibration_digest(), EMPTY_CALIBRATION_DIGEST);
    let cal = CalibrationFile { mit_qualification: Some(serde_json::json!({"k": 1})), ..Default::default() };
    let expected = armd::gravity::compute_sha256(&serde_json::to_vec(&cal).unwrap());
    inner.set_calibration(Some(cal));
    let (same, allocs, frees) = counted(|| inner.active_calibration_digest_str().eq_ignore_ascii_case(&expected));
    assert!(same);
    assert_eq!((allocs, frees), (0, 0));
    inner.set_calibration(None);
    assert_eq!(inner.active_calibration_digest(), EMPTY_CALIBRATION_DIGEST);
}
