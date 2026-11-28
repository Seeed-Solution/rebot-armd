//! Core supervisor constants, state structs, and calibration/RX support types.

#[allow(unused_imports)]
use super::*;

/// J7 stall latch: |τ7| at or above this (N·m) with the jaw making no progress over the
/// stall window. Tuned on hardware as 0.80 when τ7 was decoded with TMAX 18; the J7 motor's
/// TMAX is 10, so the same physical torque is 0.80 × 10 / 18.
pub const GRIPPER_STALL_LATCH_NM: f32 = 0.444;
/// Minimum measured net position progress (rad) along the commanded direction proving the jaw
/// is advancing under load (~2.5x the 0.0008 rad / 2-LSB stall jitter measured on B601-DM J7,
/// well below the 0.0057 rad median 50 Hz closing tick).
pub const GRIPPER_STALL_PROGRESS_EPS: f32 = 0.002;
/// Hard upper bound (s) on the stall-detection window so a real jam fails within <= 0.5 s after
/// progress stops, independent of the action timeout.
pub const GRIPPER_STALL_WINDOW_MAX_S: f64 = 0.50;
/// J7 stall release: |τ7| below this (N·m) for 6 ticks (formerly 0.40 at TMAX 18;
/// 0.40 × 10 / 18 = 0.2222). Raw torque 2093 decodes to exactly that value and was read
/// as 0.39999962 < 0.40 before, so the bound sits just above it.
pub const GRIPPER_STALL_RELEASE_NM: f32 = 0.2223;
/// A gripper action that ends with its J7 command more than this far from the measured jaw
/// (rad) and no object held has J7 set to the measured jaw on the next tick.
pub const GRIPPER_END_LEAD_RAD: f32 = 0.02;
/// During a `gripper_calibrate` stroke the torque stall latch needs the J7 command at least this
/// far from the measured jaw (rad): a blocked jaw falls behind its command, a free jaw settling
/// onto its target does not (it reads 0.5-0.7 N·m of friction at low speed near open on the
/// B601-DM new finger).
pub const GRIPPER_STALL_MIN_LEAD_RAD: f32 = 0.05;
/// Gripper torque cap: consecutive ticks with |τ| above `gripper_policy.max_close_torque_nm`
/// (fresh feedback, jaw driven closed; or the torque pushing open above it while driven open)
/// that latch the stall regardless of the jaw speed. One: the first sample over the cap latches
/// (at two, one impact sample of a stiff contact passed before the latch).
pub const GRIPPER_FORCE_CAP_TICKS: u32 = 1;
/// After a torque-cap latch under a servo stream or live policy that still closes, the grip
/// ramps back in ([`crate::motion::CapRamp`]: steps sized from the contact stiffness), at most
/// this fast (rad/s) ...
pub const GRIPPER_STREAM_CAP_RAMP_RAD_S: f32 = 0.25;
/// ... until |τ| reaches this fraction of `max_close_torque_nm`, then holds.
pub const GRIPPER_STREAM_CAP_HOLD_FRACTION: f32 = 0.6;
/// The learned free-motion torque may raise the closing stall threshold above
/// GRIPPER_STALL_LATCH_NM by at most this (N·m); opening keeps GRIPPER_STALL_LATCH_NM.
pub const GRIPPER_STALL_BASELINE_RAISE_MAX_NM: f32 = 0.15;
/// Finished gripper moves queued for learning; further ones are dropped until the queue is
/// processed (no allocation on the control tick).
pub const GRIPPER_BASELINE_PENDING_MAX: usize = 4;

/// Records of the strokes of a running `gripper_calibrate` action (id), in stroke order.
#[derive(Debug, Clone)]
pub struct GripperCalibrationRun {
    pub id: u64,
    pub records: Vec<crate::gripper_baseline::MoveRecord>,
}

/// What a gripper action does with the learned free-motion baseline (tests and tools; the API
/// calibrates with the `gripper_calibrate` action).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GripperBaselineCmd {
    /// Ordinary action: only opening moves are learned.
    #[default]
    None,
    /// `"calibrate"`: a calibration stroke with nothing in the jaws; non-grasp closes are learned too.
    Calibrate,
    /// `"reset"`: forget the baseline once the action is admitted, then as `Calibrate`.
    Reset,
}
/// The learned free-motion torque may raise the stream regulation target above
/// `contact_torque_nm` by at most this (N·m); it raises the stream contact detection fully.
pub const GRIPPER_STREAM_TARGET_RAISE_MAX_NM: f32 = 0.10;
/// A torque-cap latch under a servo stream or live policy releases only when the stream's
/// gripper target is at least this much more open than the latch hold (rad). Low torque does
/// not release it there: on a rigid object the hold at the measured jaw reads ~0 N·m, and a
/// release would let the stream squeeze again (a cycle every ~19 ticks in the sim rig).
pub const GRIPPER_FORCE_RELEASE_MARGIN_RAD: f32 = 0.05;
/// Stream contact regulator: the contact ends when the jaw closes this far past the contact
/// point with |τ| below half of `contact_torque_nm` (the object gave way or slipped).
pub const GRIPPER_STREAM_CONTACT_LOST_RAD: f32 = 0.1;

/// Contact regulator state of a servo stream / live policy on the gripper
/// (`gripper_policy.stream_contact_speed_rad_s`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GripperStreamContact {
    /// Measured jaw at the contact (rad).
    pub at_rad: f32,
    /// Most-closed J7 command allowed (rad), applied by `gripper_lead_cap`.
    pub limit_rad: f32,
}

/// Feedback gap that latches BusLoss while torqued. The Damiao motors run with
/// CAN TIMEOUT=0 and keep holding their last POS_VEL target through a gap, so a
/// short interruption is not a hazard by itself. 500 ms tolerates the ~260 ms
/// CAN gaps observed on the reBot rig (2026-11-06, coinciding with 24 V sags);
/// a real power loss drops the arm regardless of this bound.
pub const BUS_LOSS_TIMEOUT: Duration = Duration::from_millis(500);
/// Maximum spread (ms) between previously reporting motors' last valid RX timestamps to
/// classify a BusLoss as a suspected synchronized motor power loss or shared CAN interruption.
pub const POWER_LOSS_SYNC_WINDOW_MS: u64 = 100;
/// A control tick arriving later than this after the previous one (or a non-finite /
/// non-positive step) latches Fault `CONTROL_DEADLINE` while torqued and operational.
/// Same bound as the hardware feedback freshness limit of the drivers.
pub const CONTROL_DEADLINE: Duration = Duration::from_millis(250);
/// After a takeover (`acquire_lease` with `force`) another connection must wait this long
/// before it can take the lease over again.
pub const PREEMPT_COOLDOWN: Duration = Duration::from_secs(3);
/// A ThermalTrip can be cleared only once every joint reports a temperature at or below
/// `max_temperature` minus this margin (°C).
pub const THERMAL_CLEAR_MARGIN_C: f32 = 5.0;
/// Maximum allowed gap between consecutive fresh temperature samples before thermal cooldown
/// dwell restarts from the next fresh sample.
pub const THERMAL_FEEDBACK_FRESHNESS_LIMIT: Duration = Duration::from_millis(250);
/// Outward tolerance (rad) when checking whether measured joint positions are back within
/// configured limits before clearing [`crate::safety::FaultKind::JointLimit`].
pub const JOINT_LIMIT_CLEAR_MARGIN_RAD: f32 = 0.01;
/// Lower bound for the automatic park started by an expired lease (trajectory + settle).
/// The actual budget is `max(WATCHDOG_PARK_TIMEOUT, deceleration + planned duration +
/// UNLOAD_SETTLE_HOLD + WATCHDOG_PARK_MARGIN)`, counted in control-loop time.
pub const WATCHDOG_PARK_TIMEOUT: Duration = Duration::from_secs(10);
/// Tracking lag and slack added to the watchdog park budget.
pub const WATCHDOG_PARK_MARGIN: Duration = Duration::from_secs(3);
/// Extra time granted once when the watchdog park budget runs out with the arm at the
/// resting pose (position, support torque, enable and feedback all pass) and only the
/// stationary check pending. A blocked or unsupported arm gets no grace.
pub const WATCHDOG_PARK_SETTLE_GRACE: Duration = Duration::from_secs(5);
/// Time the arm must stay at the resting pose before an unload cuts torque.
pub const UNLOAD_SETTLE_HOLD: Duration = Duration::from_millis(800);
/// Park with an object in the gripper: at the resting pose J7 opens this far past the contact
/// point (rad, about a third of the B601-DM stroke) to drop it, then closes.
pub const PARK_RELEASE_OPEN_RAD: f32 = 2.0;
/// J7 speed of the park release (rad/s), as macro gripper moves.
pub const PARK_RELEASE_SPEED: f64 = 1.0;

/// Gripper release at the end of a park that carried an object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParkRelease { Opening, Closing }
/// Stationary evidence over the settle hold window, per joint: max - min measured
/// position (rad). 4 mrad in 0.8 s bounds the average speed at 0.005 rad/s.
pub const UNLOAD_STILL_POSITION_SPAN_RAD: f32 = 0.004;
/// Per-sample |velocity| bound during the settle hold, as a multiple of the joint's
/// resting-pose velocity limit (J1-J3: 0.02 rad/s = 4 LSB at VMAX 10). The window mean
/// must stay within the limit itself.
pub const UNLOAD_STILL_VELOCITY_PEAK_FACTOR: f32 = 4.0;
/// The mean velocity over this most recent part of the settle window must also stay within
/// the resting-pose limit, so motion starting late in the window blocks the release
/// before it accumulates the position span.
pub const UNLOAD_STILL_RECENT: Duration = Duration::from_millis(300);
/// A settle pending longer than the hold plus this logs its blocker, at most once per second.
pub(super) const UNLOAD_SETTLE_REPORT_AFTER_S: f64 = 1.0;

/// One stationary-settle sample stored inline without heap vectors (`MAX_MOTION_DOF = 9`),
/// so clearing the settle window on a deferred output gap never allocates or frees on RT.
#[derive(Debug, Clone, Copy)]
pub struct UnloadSettleSample {
    pub t: f64,
    pub dof: usize,
    pub positions: [f32; crate::model::MAX_MOTION_DOF],
    pub velocities: [f32; crate::model::MAX_MOTION_DOF],
}

/// Settle phase of a NormalUnload after its trajectory finished.
#[derive(Debug, Clone, Default)]
pub struct UnloadSettle {
    /// Time in the settle phase (s, control-loop time).
    pub pending_s: f64,
    /// Time from the first to the latest sample of the current run of samples that met
    /// the static resting-pose conditions (s).
    pub held_s: f64,
    /// `pending_s` of the first sample of that run.
    pub held_since_s: Option<f64>,
    /// Preallocated inline sample ring over the last [`UNLOAD_SETTLE_HOLD`] while held.
    pub samples: std::collections::VecDeque<UnloadSettleSample>,
    /// `pending_s` of the last blocker report.
    pub last_report_s: Option<f64>,
}

/// Servo streaming: a lead-capped joint whose measured position moves less than this
/// (rad) over `following_error.persistence_ms` is blocked.
pub const SERVO_BLOCKED_PROGRESS_RAD: f32 = 0.01;

/// Most FK evaluations the keep-out sweep check may add for one servo or braking step;
/// a step that needs more is held like a step into a box.
pub const MAX_KEEPOUT_EVALS_PER_TICK: usize = 64;

/// Most FK evaluations the keep-out sweep check may add for one joint-space trajectory;
/// a path that needs more is WORKSPACE_VIOLATION.
pub const MAX_KEEPOUT_EVALS_PER_PLAN: usize = 4000;

/// `observation.motion_limit` is cleared this long (s) after the limit last applied.
pub const MOTION_LIMIT_CLEAR_S: f64 = 0.3;

/// Park started by the lease watchdog: elapsed control-loop time against its budget.
#[derive(Debug, Clone, PartialEq)]
pub struct WatchdogPark {
    pub elapsed_s: f64,
    pub budget_s: f64,
    /// True once [`WATCHDOG_PARK_SETTLE_GRACE`] was added to the budget.
    pub grace_granted: bool,
}

/// Active WebSocket client lease record
#[derive(Debug, Clone, PartialEq)]
pub struct ActiveLease {
    pub client_id: String,
    pub generation: u64,
    /// The `client_id` label the holder sent with `acquire_lease` (e.g. "rebot-mcp"),
    /// published in the observation as `lease_holder`; empty when none was given.
    pub label: String,
    pub acquired_at: Instant,
    pub last_valid_motion_instant: Option<Instant>,
}

/// An object currently held in the gripper jaws.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeldObject {
    pub label: String,
    pub spill_safe: bool,
}

/// Owner identity bound to a [`DeferredActionDone`] so a later `Complete` never commits a
/// stale or superseded action, macro child, or internal park gripper step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::large_enum_variant)] // inline Copy HolderLabel avoids RT heap allocation
pub(crate) enum DeferredCompletionTarget {
    Action(u64),
    MacroSub {
        action_id: u64,
        ticket: crate::macro_action::ChildTicket,
    },
    MacroFinish {
        action_id: u64,
        state: crate::model::HolderLabel,
        code: Option<crate::model::HolderLabel>,
        message: crate::motion::StepErrorText,
        clear_held: bool,
    },
    ParkRelease(ParkRelease),
    Anonymous,
}

/// Action completion captured during trajectory stepping and published only after
/// that tick's output passes tracking validation and is accepted (`Complete`) by the driver.
pub(crate) struct DeferredActionDone {
    pub(crate) target: DeferredCompletionTarget,
    pub(crate) epoch_id: u64,
    pub(crate) lease_generation: Option<u64>,
    pub(crate) settled: Option<bool>,
    pub(crate) holding: Option<bool>,
    pub(crate) gripper_gap_rad: Option<f32>,
    pub(crate) clear_releasing: bool,
    pub(crate) is_gripper: bool,
}

/// Result of staging a per-release driver TX deadline cap using conservative driver-clock-first,
/// live-owner-clock-second translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DriverTxCapOutcome {
    /// A strictly live deadline (`owner_now < owner_cutoff`) was staged on the driver.
    Installed,
    /// No driver attached (or non-blocking try_lock contended); no deadline staged.
    NotInstalled,
    /// Owner cutoff already expired (`owner_now >= owner_cutoff`) before staging; zero deadline
    /// staged and callers must skip budgeted TX.
    Expired,
}

/// Conservative driver-clock-first, live-owner-clock-second cutoff translation shared by
/// periodic TX, RX recovery, release caps, torque transitions, and budgeted `clear_fault`.
///
/// Samples `drv.clock_now()` FIRST and `now_fn()` SECOND so driver-lock wait or clock latency can
/// never add time back to the cutoff. Returns `Err((drv_now, owner_now))` when already expired
/// (`owner_now >= owner_cutoff`) so callers reject expired cutoffs before staging any driver
/// deadline.
#[inline]
pub(crate) fn translate_driver_cutoff_conservative(
    drv: &dyn ArmDriver,
    owner_cutoff: Instant,
    now_fn: &mut dyn FnMut() -> Instant,
) -> Result<(Instant, Instant, Instant), (Instant, Instant)> {
    let drv_now = drv.clock_now();
    let owner_now = now_fn();
    if owner_now >= owner_cutoff {
        return Err((drv_now, owner_now));
    }
    let remaining = owner_cutoff.saturating_duration_since(owner_now);
    Ok((drv_now, owner_now, drv_now + remaining))
}

/// Single per-tick TX cutoff computed ONCE in the owner:
/// `drv_effective = min(control-release deadline, active action/macro parent & child deadlines)`
/// translated into the driver clock domain and installed at the driver TX choke point.
#[derive(Debug, Clone, Copy)]
pub(crate) struct EffectiveTxCutoff {
    /// Control-release deadline minus publish margin in the driver clock domain (safety holds use
    /// this ceiling so an action deadline never clips a safety/E-stop hold).
    pub(crate) drv_release_cap: Instant,
    /// Active action/macro deadline translated into the driver clock domain (`None` when idle or
    /// already quiesced/finishing).
    pub(crate) drv_action_cutoff: Option<Instant>,
    /// Owner-clock anchor sampled together with `drv_anchor` when translating deadlines.
    pub(crate) owner_anchor: Instant,
    /// Driver-clock anchor sampled together with `owner_anchor`.
    pub(crate) drv_anchor: Instant,
}

/// Zero-allocation classification of a bounded driver output write in one release,
/// explicitly bound to whether the submitted batch was a stopped hold (`is_hold`).
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // inline OutputErrorText avoids RT heap allocation
pub(crate) enum TickOutput {
    None,
    /// Driver refused target/hold because an E-stop or authorized stop is pending/latched.
    /// Zero-allocation typed refusal (avoids formatting or matching `ESTOP_INHIBITED`).
    EstopInhibited { is_hold: bool },
    /// Driver TX choke point dropped stale client-lease motion frames because the lease
    /// generation was fenced by disconnect, release, preemption, or expiry (not a bus fault).
    LeaseFenced,
    /// Driver TX choke point refused target writes because the active action/macro deadline
    /// elapsed before transmit (zero frames placed on the wire after the deadline; never counted
    /// as a CAN bus stall).
    ActionDeadlineRefused {
        drv_now: Instant,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Complete {
        is_hold: bool,
        now: Instant,
        max_age: Duration,
        due_mask: u64,
        sent_mask: u64,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Deferred {
        pending: usize,
        now: Instant,
        max_age: Duration,
        oldest_age: Option<Duration>,
        due_mask: u64,
        is_hold: bool,
        sent_mask: u64,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Expired {
        dropped: usize,
        pending: usize,
        now: Instant,
        max_age: Duration,
        oldest_age: Option<Duration>,
        due_mask: u64,
        sent_mask: u64,
        is_hold: bool,
        pf_status: crate::driver::PosForceTxStatus,
    },
    Error {
        message: crate::mit::OutputErrorText,
        now: Instant,
        max_age: Duration,
        is_hold: bool,
        sent_any: bool,
        due_mask: u64,
        sent_mask: u64,
        pf_status: crate::driver::PosForceTxStatus,
    },
}

/// What [`SupervisorInner::tick_publish`] hands back.
pub struct TickPublished {
    /// A copy of the published observation (asked for with `keep`).
    pub kept: Option<ArmObservation>,
    /// End of the tick body (the tick phase mark), None without phase timing.
    pub body_end: Option<Instant>,
    /// Wall duration (µs) spent in edge-triggered terminal-action capture before publish.
    pub phase_terminal_us: u32,
    /// Wall duration (µs) spent strictly in `StatePublisher::publish_with_rt_meta` (disjoint from
    /// terminal capture, including early supersession/materialization).
    pub publish_only_us: u32,
    /// True if at least one newly terminal action record was pushed during this publish.
    pub captured_any: bool,
    /// Context-switch baseline sampled strictly before `capture_edge_terminals` so `mid_nvcsw`
    /// attributes any voluntary switch during terminal capture, publish, ACK flush, or guard drop.
    pub mid_csw_start: Option<crate::diagnostics::tick::CswMark>,
}

/// Phase of an off-owner deferred park continuation (MIT moving-park handback or bounded
/// `holding_after_clear` rearm).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParkContinuationPhase {
    /// Waiting for `TorqueTransition::Enable { origin: ParkRearm }` confirmation.
    WaitingRearm,
    /// Waiting for MIT tracking to brake to rest and hand back to position mode.
    WaitingMitHandback,
    /// Confirmed rearm or MIT handback complete; off-lock worker may prepare and install.
    ReadyForPlan,
}

/// Exclusive in-flight park continuation owned by a single park action (zero heap allocations).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParkContinuation {
    pub action_id: u64,
    pub phase: ParkContinuationPhase,
    pub epoch: u64,
    pub stop_gen: u64,
    pub lease_gen: Option<u64>,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    /// Original park request deadline enforced on both owners; never extended by continuations.
    pub deadline: Instant,
}

/// Pending off-thread preparation state for a lease-expiry watchdog park (zero heap allocations).
/// While active, the arm holds at its measured position until the off-thread planner installs a
/// prepared `MotionRequest::park()` plan through `rt_core::requests::motion::install_watchdog_park`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WatchdogParkPending {
    pub generation: u64,
    pub attempt: u8,
    pub epoch: u64,
    pub workspace_revision: u64,
    pub motion_config_revision: u64,
    pub waiting_mit_handback: bool,
}

/// Internal mutable state of the robotic arm supervisor
/// Result of a zero-allocation, non-blocking RT resting-pose evaluation (`check_resting_pose_rt`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RtRestingCheck {
    AtRest,
    OffRest,
    LockContended,
}

/// Single-sample snapshot of active J7 `PosForce` freshness and freeze position.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct J7PosForceGate {
    pub(crate) stale: bool,
    pub(crate) freeze_pos: Option<f32>,
}

/// Copy `src` into `dst` (for `ArmObservation::{action, previous_action}`), routing
/// `Option<String>` message transitions (`None <-> Some`) through `spare` so `armd-control`
/// never allocates or drops heap strings on action start, failure, or `previous_action` expiry.
pub(super) fn sync_action_obs(
    dst: &mut Option<ActionStatus>,
    src: Option<&ActionStatus>,
    spare: Option<(&mut String, &mut String)>,
) {
    let swap_opt_str = |src_present: bool, dst_field: &mut Option<String>, sp: &mut String| {
        match (src_present, dst_field.is_some()) {
            (true, false) if sp.capacity() > 0 => {
                *dst_field = Some(std::mem::take(sp));
            }
            (false, true) => {
                if let Some(old) = dst_field.take() {
                    if sp.capacity() == 0 {
                        *sp = old;
                    }
                }
            }
            _ => {}
        }
    };
    let copy_bounded_no_alloc = |dst_str: &mut String, src_str: &str| {
        dst_str.clear();
        if src_str.len() <= dst_str.capacity() {
            dst_str.push_str(src_str);
        } else {
            let mut end = dst_str.capacity();
            while end > 0 && !src_str.is_char_boundary(end) {
                end -= 1;
            }
            dst_str.push_str(&src_str[..end]);
        }
    };
    match (src, dst.as_mut()) {
        (Some(s), Some(d)) => {
            if let Some((sp_msg, sp_warn)) = spare {
                swap_opt_str(s.message.is_some(), &mut d.message, sp_msg);
                swap_opt_str(
                    s.durability_warning.is_some(),
                    &mut d.durability_warning,
                    sp_warn,
                );
                let msg_overflow = s
                    .message
                    .as_deref()
                    .zip(d.message.as_ref())
                    .is_some_and(|(src_m, dst_m)| src_m.len() > dst_m.capacity());
                let warn_overflow = s
                    .durability_warning
                    .as_deref()
                    .zip(d.durability_warning.as_ref())
                    .is_some_and(|(src_w, dst_w)| src_w.len() > dst_w.capacity());
                if msg_overflow || warn_overflow {
                    let mut saved_msg = d.message.take();
                    let mut saved_warn = d.durability_warning.take();
                    let s_msg = s.message.as_deref();
                    let s_warn = s.durability_warning.as_deref();
                    // Clone scalar/bounded fields without letting `clone_from` reallocate `message`
                    // or `durability_warning` on `armd-control`; full text lives in `terminal_capture`.
                    d.clone_from_without_msg_warn(s);
                    if let (Some(dst_m), Some(src_m)) = (saved_msg.as_mut(), s_msg) {
                        copy_bounded_no_alloc(dst_m, src_m);
                    }
                    if let (Some(dst_w), Some(src_w)) = (saved_warn.as_mut(), s_warn) {
                        copy_bounded_no_alloc(dst_w, src_w);
                    }
                    d.message = saved_msg;
                    d.durability_warning = saved_warn;
                    return;
                }
            }
            d.clone_from(s);
        }
        (Some(s), None) => {
            if let Some((sp_msg, sp_warn)) = spare {
                let mut blank = ActionStatus::blank();
                if let Some(src_m) = s.message.as_deref() {
                    if sp_msg.capacity() > 0 {
                        let mut buf = std::mem::take(sp_msg);
                        copy_bounded_no_alloc(&mut buf, src_m);
                        blank.message = Some(buf);
                    }
                }
                if let Some(src_w) = s.durability_warning.as_deref() {
                    if sp_warn.capacity() > 0 {
                        let mut buf = std::mem::take(sp_warn);
                        copy_bounded_no_alloc(&mut buf, src_w);
                        blank.durability_warning = Some(buf);
                    }
                }
                blank.clone_from_without_msg_warn(s);
                *dst = Some(blank);
                return;
            }
            *dst = Some(s.clone());
        }
        (None, Some(d)) => {
            if let Some((sp_msg, sp_warn)) = spare {
                if let Some(old) = d.message.take() {
                    if sp_msg.capacity() == 0 {
                        *sp_msg = old;
                    }
                }
                if let Some(old) = d.durability_warning.take() {
                    if sp_warn.capacity() == 0 {
                        *sp_warn = old;
                    }
                }
            }
            *dst = None;
        }
        (None, None) => {}
    }
}

/// Summary of the release's Step-2 hardware poll passed into Step-5 `tick_control`
/// without allocating or re-reading the bus.
#[derive(Debug, Default)]
pub(crate) struct PendingRxPoll {
    pub active: bool,
    pub invalid_feedback: bool,
    pub driver_torque_lost: bool,
    pub driver_enable_confirmed_ok: bool,
    pub driver_torque_loss_sample_seq: Option<u64>,
    pub bus_feedback_fresh: bool,
    pub recovery_ages: bool,
    pub hw_rx: bool,
    pub reset_stillness_on_read_err: bool,
    pub rx_frames: u16,
    pub budget_exhausted: bool,
}

/// Complete validated gripper calibration snapshot ready for infallible commit after disk rename.
#[derive(Debug, Clone)]
pub struct GripperCalibrationCandidate {
    pub(super) config: ArmConfig,
    pub(super) base_gripper_policy: Option<crate::model::GripperPolicy>,
    pub(super) gripper_calib_offset: Option<f32>,
    pub(super) gripper_calib_jaw_opening: Option<crate::model::JawOpening>,
    pub(super) gripper_calib_invalid: bool,
    pub(super) driver_config_error: Option<String>,
}

/// Pre-captured heap context for `gripper_zero_check(store_offset=true)` persistence,
/// allocated once at action start so `finish_action` on `armd-control` performs zero heap
/// allocations and zero frees.
#[derive(Clone)]
pub struct ZeroCheckPersistContext {
    pub path: PathBuf,
    pub jaw_opening: Option<crate::model::JawOpening>,
    pub stale: Option<crate::calibration::CalibrationStale>,
    pub tool_id: String,
    pub tool_config_digest: String,
    pub active_tool_is_stock: bool,
    pub base_gripper_policy: Option<crate::model::GripperPolicy>,
    pub gripper_limit: Option<crate::model::JointLimit>,
    pub operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    /// Pre-allocated scratch buffer reused on saturation failure without allocating on RT.
    pub sat_msg_buf: String,
}

/// Parameters of a running gripper zero check action.
#[derive(Clone)]
pub struct ZeroCheckParams {
    pub action_id: u64,
    pub store_offset: bool,
    pub threshold_rad: f32,
    pub base_closed_rad: f32,
    pub persist_ctx: Option<Box<ZeroCheckPersistContext>>,
    /// Off-RT staged `ZeroCheckPersistContext` handle for executor-admitted sessions (`0` on legacy
    /// or when `store_offset == false`), avoiding any heap allocation or free on `armd-control`.
    pub persist_ctx_id: u64,
    /// Bound PR0B `SessionId` when driven by `SessionDriver` (`SessionKind::GripperZeroCheck`).
    pub session_id: Option<crate::rt_core::session::SessionId>,
    /// Owning WS connection ID (`0` when submitted outside a tracked connection).
    pub connection_id: u64,
    /// Live lock-free operator grant bound at executor admission (when `store_offset == true`).
    pub operator_grant: Option<crate::auth::OperatorGrant>,
}

/// `Copy` off-RT handoff event emitted over `rt_core::stream` from `armd-control` to `armd-persist`
/// for executor-admitted `gripper_zero_check` sessions (0 RT heap allocs/frees).
#[allow(clippy::large_enum_variant)] // inline Copy event on lock-free SPSC stream; boxing would allocate on RT
#[derive(Debug, Clone, Copy)]
pub enum ZeroCheckStreamEvent {
    Persist {
        action_id: u64,
        offset_rad: f32,
        base_revision: u64,
        connection_id: u64,
        lease_owner: Option<(crate::model::HolderLabel, u64)>,
        estop_epoch: u64,
        stop_gen: u64,
        deadline: Instant,
        persist_ctx_id: u64,
        worker_ticket: Option<crate::rt_core::session::WorkerTicket>,
    },
    Discard {
        persist_ctx_id: u64,
    },
}

/// Deferred atomic persist-before-commit job for `gripper_zero_check(store_offset=true)`.
#[derive(Clone)]
pub struct PendingZeroCheckPersist {
    pub action_id: u64,
    pub offset_rad: f32,
    pub base_revision: u64,
    pub connection_id: u64,
    pub lease_owner: Option<(crate::model::HolderLabel, u64)>,
    pub estop_epoch: u64,
    pub stop_gen: u64,
    pub deadline: Instant,
    pub ctx: Box<ZeroCheckPersistContext>,
}


impl UnloadSettleSample {
    pub fn from_slices(t: f64, q: &[f32], v: &[f32]) -> Self {
        let dof = q.len().min(v.len()).min(crate::model::MAX_MOTION_DOF);
        let mut positions = [f32::NAN; crate::model::MAX_MOTION_DOF];
        let mut velocities = [f32::NAN; crate::model::MAX_MOTION_DOF];
        positions[..dof].copy_from_slice(&q[..dof]);
        velocities[..dof].copy_from_slice(&v[..dof]);
        Self { t, dof, positions, velocities }
    }
}

impl UnloadSettle {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            samples: std::collections::VecDeque::with_capacity(cap),
            ..Self::default()
        }
    }
}

impl TickOutput {
    /// True when the active J7 position-force gripper was stale (> 100 ms) during this pass
    /// (receiving a freeze frame instead of any new target); prevents J7-dependent action
    /// completions from committing on that tick.
    #[inline]
    pub(crate) fn j7_stale(&self) -> bool {
        matches!(
            self,
            Self::ActionDeadlineRefused { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Complete { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Deferred { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Expired { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Error { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
        )
    }
}

impl ZeroCheckPersistContext {
    /// Captures all heap-backed calibration persistence inputs off the RT completion path.
    pub fn from_inner(
        inner: &SupervisorInner,
        operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    ) -> Self {
        let path = inner
            .gripper_calib_path
            .clone()
            .unwrap_or_else(crate::gripper_calib::path_from_env);
        let gripper_limit = inner
            .config
            .gripper_index()
            .and_then(|idx| inner.config.limits.get(idx))
            .cloned();
        Self {
            path,
            jaw_opening: inner.gripper_calib_jaw_opening.clone(),
            stale: inner.gripper_calib_stale.clone(),
            tool_id: inner.config.active_tool.id.clone(),
            tool_config_digest: inner.config.active_tool.config_digest.clone(),
            active_tool_is_stock: inner.config.active_tool.is_stock,
            base_gripper_policy: inner
                .base_gripper_policy
                .clone()
                .or_else(|| inner.config.gripper_policy.clone()),
            gripper_limit,
            operator_auth,
            sat_msg_buf: String::with_capacity(64),
        }
    }

    /// Serializes `GripperCalibFile` off the control thread into a [`crate::persist::PendingWrite`].
    pub fn serialize_write(&self, offset_rad: f32) -> crate::persist::PendingWrite {
        let file = crate::gripper_calib::GripperCalibFile {
            offset_rad: Some(offset_rad),
            jaw_opening: self.jaw_opening.clone(),
            ts: Some(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs()),
            ),
            stale: self.stale.clone(),
            tool_id: Some(self.tool_id.clone()),
            tool_config_digest: Some(self.tool_config_digest.clone()),
        };
        let text = serde_json::to_string_pretty(&file).unwrap_or_default();
        crate::persist::PendingWrite::new(self.path.clone(), text)
    }
}
