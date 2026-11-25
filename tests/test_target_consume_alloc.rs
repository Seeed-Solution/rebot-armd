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

/// B2 + M1: a legal workspace with 32 end points + 4 keep-out boxes + 16 link capsules (> 256
/// clearance slots and > 48 geometries) must perform ZERO heap allocations and ZERO frees on
/// `armd-control` across:
/// 1. servo target intake (`consume_target`),
/// 2. servo step (`tick_control`) + Step 6 observation publish (`fill_observation`),
/// 3. workspace refusal on intake (`WORKSPACE_VIOLATION`),
/// 4. in-motion keep-out sweep refusal (`enter_servo_keepout_hold_rt`) + Step 6 publish of
///    `obs.workspace_violation`,
/// 5. stop + clear violation + follow-up servo target & step.
#[test]
fn large_legal_workspace_servo_intake_step_refusal_and_publish_do_not_allocate() {
    use armd::model::{ArmObservation, KeepoutBox, LinkCapsule};

    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };

    // Start from the standard inspection pose (`INSPECT`) and sweep J1 by 0.30 rad (< 0.35 rad
    // dirty-jump guard) across a 4 cm keep-out cube centered at the midpoint (`+0.15 rad`).
    let base_pose = vec![0.0f32, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];
    let mut q_ok = base_pose.clone();
    q_ok[0] += 0.02;
    let mut q_mid = base_pose.clone();
    q_mid[0] += 0.15;
    let mut q_far = base_pose.clone();
    q_far[0] += 0.30;
    let p_mid = cfg.arm_model.fk_f32(&q_mid).expect("fk q_mid").p;

    {
        let ws = cfg.workspace.as_mut().expect("rebot_dm workspace");
        // Fill check_points_end_m to the legal maximum of 32 points inside the gripper envelope.
        let mut idx = 0usize;
        while ws.check_points_end_m.len() < 32 {
            let x = -0.02 - 0.004 * (idx as f32);
            let y = if idx.is_multiple_of(2) { 0.01 } else { -0.01 };
            ws.check_points_end_m.push([x, y, 0.0]);
            idx += 1;
        }
        assert_eq!(ws.check_points_end_m.len(), 32);

        // Fill link_capsules to the legal maximum of 16 capsules (1 + 32 + 1 + 16 = 50 > 48 geoms).
        let mut extra = 0usize;
        while ws.link_capsules.len() < armd::model::MAX_LINK_CAPSULES {
            ws.link_capsules.push(LinkCapsule {
                id: format!("extra_cap_{extra}"),
                frame: "link6".into(),
                a_m: [0.0, 0.0, 0.015],
                b_m: [0.0, 0.0, 0.022],
                radius_m: 0.012,
            });
            extra += 1;
        }
        assert_eq!(ws.link_capsules.len(), armd::model::MAX_LINK_CAPSULES);

        // 4 keep-out boxes: 3 away from the sweep + 1 sitting strictly at `p_mid` so `base_pose`
        // and `q_far` are legal while `q_mid` (and the linear sweep `base_pose -> q_far`) hits it.
        // Total slots = (1 + 32 + 1 + 16) * (7 + 4) + pairs >= 550 > 256 (`MAX_STACK_WORKSPACE_SLOTS`).
        ws.keepout_boxes = vec![
            KeepoutBox {
                name: "kb_far_a".into(),
                min_m: [-0.58, -0.58, 0.62],
                max_m: [-0.45, -0.45, 0.75],
            },
            KeepoutBox {
                name: "kb_far_b".into(),
                min_m: [0.45, -0.58, 0.62],
                max_m: [0.58, -0.45, 0.75],
            },
            KeepoutBox {
                name: "kb_far_c".into(),
                min_m: [-0.58, 0.45, 0.62],
                max_m: [-0.45, 0.58, 0.75],
            },
            KeepoutBox {
                name: "kb_mid_gate".into(),
                min_m: [
                    (p_mid[0] - 0.02) as f32,
                    (p_mid[1] - 0.02) as f32,
                    (p_mid[2] - 0.02) as f32,
                ],
                max_m: [
                    (p_mid[0] + 0.02) as f32,
                    (p_mid[1] + 0.02) as f32,
                    (p_mid[2] + 0.02) as f32,
                ],
            },
        ];
        ws.validate().expect("32 endpoints + 16 capsules + 4 keepout boxes must be legal");
    }

    // Confirm the configuration genuinely overflows the 256-slot inline stack buffer and 48-geom cap.
    {
        let ws = cfg.workspace.as_ref().unwrap();
        let ev = armd::chain::WorkspaceEvaluator::new(&cfg.arm_model, ws, &cfg.camera_mount);
        assert!(
            ev.total_slots > armd::kinematics::MAX_STACK_WORKSPACE_SLOTS,
            "expected > 256 clearance slots, got {}",
            ev.total_slots
        );
        assert!(
            cfg.arm_model
                .workspace_violation_with_tcp(&base_pose, ws, &cfg.camera_mount, cfg.active_tool.tcp_offset_m)
                .worst
                .is_none(),
            "base_pose must be legal"
        );
        assert!(
            cfg.arm_model
                .workspace_violation_with_tcp(&q_far, ws, &cfg.camera_mount, cfg.active_tool.tcp_offset_m)
                .worst
                .is_none(),
            "q_far must be legal"
        );
        assert!(
            cfg.arm_model
                .workspace_violation_with_tcp(&q_mid, ws, &cfg.camera_mount, cfg.active_tool.tcp_offset_m)
                .worst
                .is_some(),
            "q_mid must hit kb_mid_gate"
        );
    }

    let mut inner = SupervisorInner::new(cfg);
    inner.tick(0.02);
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).expect("arm");
    }
    let gen = inner.acquire_lease(CONN).expect("lease");
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::authorize_servo_stream(a, gen).unwrap();
    }
    inner.commanded_positions.clone_from(&base_pose);
    inner.measured_positions.clone_from(&base_pose);
    inner.target_positions.clone_from(&base_pose);
    inner.prewarm_rt_buffers();

    let mut obs = ArmObservation::blank_rt();
    inner.fill_observation(1.0, &mut obs);

    let stream = inner.core.target_authority().stream.expect("authorized");
    let now = Instant::now();
    let authority = Authority {
        lease_generation: gen,
        epoch: inner.safety.epoch_id(),
        config_revision: inner.workspace_revision,
        ..Authority::default()
    };

    let make_frame = |q_vec: &[f32], seq: u64| {
        let mut q = [0.0; MAX_DOF];
        q[..q_vec.len()].copy_from_slice(q_vec);
        TargetFrame {
            arm: ArmId::LOCAL,
            boot: BOOT,
            stream,
            seq,
            authority,
            policy_generation: 0,
            input_at: now,
            vision_at: None,
            vision_frame_id: None,
            deadline: now + Duration::from_secs(1),
            dof: q_vec.len() as u8,
            q,
            replaceable: true,
        }
    };

    let f_ok = make_frame(&q_ok, 1);
    let f_mid = make_frame(&q_mid, 2);

    // 1. Valid servo intake + 2. Valid servo step + Step 6 publish (all >256 slots, zero warmup target!).
    let (intake_ok, allocs1, frees1) = counted(|| {
        let r = inner.consume_target(&f_ok, &ctx(now));
        inner.tick_control(0.02);
        inner.fill_observation(1.02, &mut obs);
        r
    });
    assert!(intake_ok.is_ok(), "valid target on >256-slot workspace must apply: {intake_ok:?}");
    assert_eq!(
        (allocs1, frees1),
        (0, 0),
        "valid servo intake + step + publish on >256-slot workspace must not touch the allocator"
    );
    assert!(
        inner.servo_ws_min_depths.as_ref().is_some_and(|(_, b)| b.is_heap()),
        "expected heap-backed preallocated min_depths SlotBuf for >256-slot workspace"
    );

    // 3. Workspace refusal on intake (`consume_target` -> `WORKSPACE_VIOLATION`): zero alloc / zero free.
    let (intake_err, allocs2, frees2) = counted(|| inner.consume_target(&f_mid, &ctx(now)).unwrap_err());
    assert_eq!(intake_err.code, "WORKSPACE_VIOLATION");
    assert_eq!(
        (allocs2, frees2),
        (0, 0),
        "workspace refusal on RT servo intake must not touch the allocator"
    );

    // 4. In-motion keep-out sweep refusal (`enter_servo_keepout_hold_rt`) + Step 6 publish of
    //    `obs.workspace_violation`: zero alloc / zero free!
    let sweep_frames: Vec<TargetFrame> = (3..303u64).map(|seq| make_frame(&q_far, seq)).collect();
    let f_after_stop = make_frame(&q_ok, 400);
    let (hit_keepout, allocs3, frees3) = counted(|| {
        let mut violated = false;
        for (step, sf) in sweep_frames.iter().enumerate() {
            let _ = inner.consume_target(sf, &ctx(now));
            inner.tick_control(0.02);
            inner.fill_observation(1.04 + 0.02 * (step as f64), &mut obs);
            if inner.workspace_violation.is_some() {
                violated = true;
                break;
            }
        }
        violated
    });
    assert!(
        hit_keepout,
        "sweeping across kb_mid_gate must trigger enter_servo_keepout_hold_rt (cmd={:?}, target={:?}, motion_limit={:?}, blocked={}, safety={:?})",
        inner.commanded_positions,
        inner.servo_target,
        inner.motion_limit,
        inner.servo_blocked_hold,
        inner.safety.state(),
    );
    assert_eq!(
        (allocs3, frees3),
        (0, 0),
        "in-motion keep-out refusal + Step 6 workspace_violation publish must not touch the allocator"
    );
    let published_v = obs
        .workspace_violation
        .as_ref()
        .expect("Step 6 observation must carry workspace_violation");
    assert_eq!(published_v.violation_phase.as_deref(), Some("servo_step"));
    assert!(
        published_v
            .constraint_id
            .as_deref()
            .is_some_and(|c| c.contains("kb_mid_gate")),
        "expected kb_mid_gate in published workspace_violation: {published_v:?}"
    );

    // 5. Stop + clear violation + follow-up servo target & step: zero alloc / zero free!
    let (after_ok, allocs4, frees4) = counted(|| {
        inner.stop_motion();
        inner.clear_workspace_violation_rt();
        inner.fill_observation(3.0, &mut obs);
        let r = inner.consume_target(&f_after_stop, &ctx(now));
        inner.tick_control(0.02);
        inner.fill_observation(3.02, &mut obs);
        r
    });
    assert!(after_ok.is_ok(), "post-stop servo target must apply: {after_ok:?}");
    assert!(obs.workspace_violation.is_none());
    assert_eq!(
        (allocs4, frees4),
        (0, 0),
        "stop + clear workspace_violation + follow-up servo step must not touch the allocator"
    );
}
