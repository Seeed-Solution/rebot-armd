//! Decision Item 5: RT workspace return paths reuse `ws_eval_bufs` and existing scratch with
//! zero heap allocations and zero frees on acceptance, refusal, and fail-closed oversized/undersized
//! inputs on `armd-control`.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::time::{Duration, Instant};

use armd::chain::ArmModel;
use armd::model::{ArmObservation, KeepoutBox};
use armd::rt_core::broker::{Authority, BootId};
use armd::rt_core::ids::ArmId;
use armd::rt_core::mailbox::{StreamId, TargetFrame, MAX_DOF};
use armd::rt_core::ops::{CommitCtx, CoreOps};
use armd::{ArmConfig, SupervisorInner};

thread_local! {
    static WATCH: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<u64> = const { Cell::new(0) };
    static FREES: Cell<u64> = const { Cell::new(0) };
}

struct Counting;

fn count(c: &'static std::thread::LocalKey<Cell<u64>>) {
    if WATCH.try_with(Cell::get).unwrap_or(false) {
        let _ = c.try_with(|n| n.set(n.get() + 1));
    }
}

// SAFETY: forwards to System; thread-local counters are const-initialized and do not allocate.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        count(&ALLOCS);
        unsafe { System.alloc(l) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        count(&ALLOCS);
        unsafe { System.alloc_zeroed(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        count(&FREES);
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        count(&ALLOCS);
        count(&FREES);
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn counted<R>(f: impl FnOnce() -> R) -> (R, u64, u64) {
    ALLOCS.with(|c| c.set(0));
    FREES.with(|c| c.set(0));
    WATCH.with(|w| w.set(true));
    let r = f();
    WATCH.with(|w| w.set(false));
    (r, ALLOCS.with(Cell::get), FREES.with(Cell::get))
}

const BOOT: BootId = BootId(7);
const CONN: &str = "conn-ws-return-alloc";
const INSPECT: [f32; 7] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0];

fn ctx(now: Instant) -> CommitCtx {
    CommitCtx {
        arm: ArmId::LOCAL,
        boot: BOOT,
        now,
        stop_generation: 0,
    }
}

fn make_frame(
    stream: StreamId,
    authority: Authority,
    q_vec: &[f32],
    seq: u64,
    now: Instant,
) -> TargetFrame {
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
}

/// Zero-alloc acceptance and refusal on RT during outside-workspace recovery (including
/// running-minimum anti-ratcheting refusal and hard-slot deepening refusal).
#[test]
fn rt_workspace_return_acceptance_and_refusal_zero_alloc() {
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg.collision = None;
    cfg.power_budget = None;
    let ws = cfg.workspace.as_mut().unwrap();
    ws.max_reach_m = None;
    let check_pts = ws.check_points_end_m.clone();
    let max_x_at = |q: &[f32]| -> f64 {
        let p = ArmModel::REBOT_DEV_ARM.fk_f32(q).unwrap();
        check_pts
            .iter()
            .map(|c| p.transform_point(&c.map(f64::from))[0])
            .fold(p.p[0], f64::max)
    };
    let q_start: [f32; 7] = [0.0, -0.90, -1.15, 0.85, 0.0, 0.0, 0.0];
    let start_max_x = max_x_at(&q_start);
    ws.box_max_m[0] = (start_max_x - 0.010) as f32;

    let mut q_probe = q_start;
    q_probe[1] += 0.01;
    let fwd_sign = if max_x_at(&q_probe) > start_max_x { 1.0f32 } else { -1.0f32 };
    let dx_per_rad = (max_x_at(&{
        let mut q = q_start;
        q[1] += 0.01 * fwd_sign;
        q
    }) - start_max_x)
        / 0.01;
    let mut q_step1 = q_start;
    q_step1[1] += (0.003 / dx_per_rad) as f32 * fwd_sign;
    let mut q_step2 = q_start;
    q_step2[1] += (0.0062 / dx_per_rad) as f32 * fwd_sign;
    let mut q_retreat = q_start;
    q_retreat[1] -= (0.010 / dx_per_rad) as f32 * fwd_sign;

    let mut inner = SupervisorInner::new(cfg);
    inner.rt_executor_owned = true;
    inner.commanded_positions = q_start.to_vec();
    inner.measured_positions = q_start.to_vec();
    inner.target_positions = q_start.to_vec();
    inner.prewarm_rt_buffers();
    inner.tick(0.02);
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::enable_torque(a))
            .unwrap();
    }
    let gen = inner.acquire_lease(CONN).unwrap();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::authorize_servo_stream(a, gen).unwrap();
    }
    let stream = inner.core.target_authority().stream.unwrap();
    let authority = Authority {
        lease_generation: gen,
        epoch: inner.safety.epoch_id(),
        config_revision: inner.workspace_revision,
        ..Authority::default()
    };
    let mut obs = ArmObservation::blank_rt();
    let now = Instant::now();
    // Warm up servo_target buffer once off the counted region.
    let f_warm = make_frame(stream, authority, &q_start, 1, now);
    inner.consume_target(&f_warm, &ctx(now)).expect("warmup");
    inner.tick_control(0.02);
    inner.fill_observation(0.02, &mut obs);

    let f_step1 = make_frame(stream, authority, &q_step1, 2, now);
    let f_step2 = make_frame(stream, authority, &q_step2, 3, now);
    let f_retreat = make_frame(stream, authority, &q_retreat, 4, now);

    // 1. Accepted +3 mm frame + ticks + settle: 0 allocs / 0 frees.
    let (ok1, a1, fr1) = counted(|| {
        let r = inner.consume_target(&f_step1, &ctx(now));
        for k in 0..20 {
            inner.tick_control(0.02);
            inner.fill_observation(0.04 + 0.02 * k as f64, &mut obs);
        }
        r
    });
    assert!(ok1.is_ok(), "step1 must be accepted: {ok1:?}");
    assert_eq!((a1, fr1), (0, 0), "step1 recovery ticks allocated/freed");

    // 2. Refused ratcheting frame (+6.2 mm total from start): 0 allocs / 0 frees.
    let (rej2, a2, fr2) = counted(|| inner.consume_target(&f_step2, &ctx(now)));
    let err2 = rej2.expect_err("ratcheting frame 2 must be refused on RT");
    assert_eq!(err2.code, "WORKSPACE_VIOLATION");
    assert_eq!((a2, fr2), (0, 0), "ratcheting refusal allocated/freed");

    // 3. Accepted retreat frame + tick: 0 allocs / 0 frees.
    let (ok3, a3, fr3) = counted(|| {
        let r = inner.consume_target(&f_retreat, &ctx(now));
        inner.tick_control(0.02);
        inner.fill_observation(1.0, &mut obs);
        r
    });
    assert!(ok3.is_ok(), "retreat frame must be accepted: {ok3:?}");
    assert_eq!((a3, fr3), (0, 0), "retreat acceptance allocated/freed");
}

/// Decision Item 5: undersized scratch / oversized workspace on RT MUST fail closed with 0 allocs / 0 frees.
#[test]
fn rt_oversized_workspace_and_undersized_scratch_fail_closed_zero_alloc() {
    let mut cfg = ArmConfig {
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    cfg.collision = None;
    cfg.power_budget = None;
    let mut inner = SupervisorInner::new(cfg);
    inner.rt_executor_owned = true;
    inner.commanded_positions = INSPECT.to_vec();
    inner.measured_positions = INSPECT.to_vec();
    inner.target_positions = INSPECT.to_vec();
    inner.prewarm_rt_buffers();
    inner.tick(0.02);
    {
        let s = inner.current_admission_stamp();
        inner
            .check_admission(s)
            .and_then(|a| SupervisorInner::enable_torque(a))
            .unwrap();
    }
    let gen = inner.acquire_lease(CONN).unwrap();
    {
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::authorize_servo_stream(a, gen).unwrap();
    }
    let stream = inner.core.target_authority().stream.unwrap();
    let authority = Authority {
        lease_generation: gen,
        epoch: inner.safety.epoch_id(),
        config_revision: inner.workspace_revision,
        ..Authority::default()
    };
    let now = Instant::now();
    let mut obs = ArmObservation::blank_rt();
    // Warm up servo_target buffer first.
    let mut q_ok = INSPECT;
    q_ok[0] += 0.01;
    inner
        .consume_target(&make_frame(stream, authority, &q_ok, 1, now), &ctx(now))
        .expect("warmup");
    inner.tick_control(0.02);
    inner.fill_observation(0.02, &mut obs);

    // Mutate workspace to exceed pre-sized scratch WITHOUT calling prepare_workspace_eval_bufs()!
    // On RT (`rt_executor_owned == true`), both servo intake and servo step MUST fail closed with (0, 0) allocs/frees!
    let ws = inner.config.workspace.as_mut().unwrap();
    for i in 0..8 {
        ws.keepout_boxes.push(KeepoutBox {
            name: format!("extra_{i}"),
            min_m: [1.0, 1.0, 1.0],
            max_m: [1.1, 1.1, 1.1],
        });
    }

    let mut q_next = q_ok;
    q_next[0] += 0.01;
    let f_next = make_frame(stream, authority, &q_next, 2, now);

    // 1. RT servo intake with undersized scratch -> fail closed with WORKSPACE_VIOLATION and (0, 0) allocs/frees!
    let (res_intake, a1, fr1) = counted(|| inner.consume_target(&f_next, &ctx(now)));
    let err = res_intake.expect_err("RT intake with undersized scratch must fail closed");
    assert_eq!(err.code, "WORKSPACE_VIOLATION");
    assert_eq!(
        (a1, fr1),
        (0, 0),
        "RT intake fail-closed on undersized scratch must not allocate or free"
    );

    // 2. RT servo step with undersized scratch -> fail closed into workspace hold with (0, 0) allocs/frees!
    // Re-install a servo_target directly (without allocating) so tick_control runs servo_step_workspace_rt.
    if let Some(t) = inner.servo_target.as_mut() {
        t[0] += 0.05;
    } else {
        inner.servo_target = Some(q_next.to_vec());
    }
    let (_, a2, fr2) = counted(|| {
        inner.tick_control(0.02);
        inner.fill_observation(0.04, &mut obs);
    });
    assert!(
        inner.servo_blocked_hold,
        "RT step with undersized scratch must fail closed into workspace hold"
    );
    assert_eq!(
        (a2, fr2),
        (0, 0),
        "RT step fail-closed on undersized scratch must not allocate or free"
    );
}
