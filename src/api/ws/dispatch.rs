//! Unified WebSocket frame & command dispatch table (A1b PR 1, spec §3).
//!
//! One table keyed by normalized [`FrameKind`] and exact command path. Each family module
//! (`lease`, `motion`, `actions`, `config`, `servo`, `policy`, `calibration`) exports its own
//! `ENTRIES` slice so parallel family PRs (L/M/P/C/S/Y) never edit a shared `match`.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64};
#[cfg(feature = "policy-runtime")]
use std::sync::atomic::AtomicUsize;
use std::sync::{Arc, LazyLock, Mutex};

use serde_json::Value;
use tokio::sync::mpsc;

use crate::api::rest::ApiError;
use crate::supervisor::{AdmissionStamp, ArmSupervisor, SupervisorInner};

use super::{
    actions, calibration, config, lease, motion, policy, servo, ConnectionTasks, WsAuth,
};

/// Normalized inbound WebSocket frame kind, preserving exact alias precedence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) enum FrameKind {
    /// `type: "estop"` or `type: "e_stop"`.
    EStop,
    /// `type: "auth"`.
    Auth,
    /// `type: "heartbeat"`.
    Heartbeat,
    /// `type: "acquire_lease"`.
    AcquireLease,
    /// `type: "release_lease"`.
    ReleaseLease,
    /// `type: "command"` (dispatched by exact `path`).
    Command,
    /// `type: "preset"` (takes precedence over implicit `positions` servo frames).
    Preset,
    /// `type: "servo"` or `mode: "cartesian_joint_stream"` (requires explicit `generation`).
    Servo,
    /// Unrecognized `type` carrying a `positions` field (implicit servo frame).
    ImplicitServo,
    /// Any other JSON frame (still subject to lease/epoch checks, ignored when authorized).
    Unknown,
}

/// Primary lookup key in the unified WS dispatch table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) struct DispatchKey {
    pub(super) kind: FrameKind,
    pub(super) path: Option<&'static str>,
}

impl DispatchKey {
    pub(super) const fn frame(kind: FrameKind) -> Self {
        Self { kind, path: None }
    }

    pub(super) const fn command(path: &'static str) -> Self {
        Self {
            kind: FrameKind::Command,
            path: Some(path),
        }
    }
}

/// Ingress lane and queue/cancellation class for a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AdmissionClass {
    /// Lock-free atomic latch in reader transition (`estop` / `e_stop`).
    UrgentEStop,
    /// Urgent non-blocking Stop path with coalescing (`/api/v1/motion/stop`).
    UrgentStop,
    /// Dedicated off-reader auth queue (`auth`).
    Auth,
    /// Dedicated latest-wins heartbeat slot (`heartbeat`).
    Heartbeat,
    /// Ordinary queue; bypasses active `MacroCheck` (`acquire_lease`, `release_lease`).
    OrdinarySession,
    /// Ordinary queue; stop-direction exemption on `enable: false` (`/api/v1/torque`, `/api/v1/arm`).
    OrdinaryTorqueOrStopExempt,
    /// Ordinary queue; standard FIFO serialized command/preset/action/config/calibration.
    Ordinary,
    /// Ordinary queue; streaming servo target (`servo`, implicit servo, `/api/v1/motion/servo`).
    OrdinaryServo,
}

/// Lease validation rule enforced before running a handler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum LeaseRequirement {
    /// No motion lease required (e.g. `estop`, `auth`, `acquire_lease`, depth measure, lease-free teach).
    None,
    /// Active lease required, except `/api/v1/action/grasp` with `body.dry_run == true`.
    RequiredExceptDryRun,
    /// Active lease required, with explicit `generation` field validated before generic lease loss.
    RequiredExplicitGeneration,
    /// Active lease required; legacy clients may omit `generation` (`heartbeat`).
    RequiredAllowZeroGeneration,
    /// Active lease required.
    Required,
}

/// Operator authorization policy for a frame or command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AuthRequirement {
    /// No operator privilege required.
    None,
    /// Always requires an operator session when operator auth is enforced.
    Operator,
    /// Conditional operator privilege evaluated by the existing privilege validator.
    Conditional,
}

/// Outbound reply lane and acknowledgement policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ReplyPolicy {
    /// Urgent E-stop lane (`epoch_sync` or `estop_reject`).
    UrgentEStop,
    /// Urgent Stop lane (`command_ack` or `command_reject`).
    UrgentStop,
    /// Reserved auth lane (`auth_ack`).
    AuthAck,
    /// Silent on success; `lease_lost` or `epoch_sync` on mismatch (`heartbeat`).
    SilentOrLossOrEpochSync,
    /// Immediate ordinary reply (`lease_granted` / `lease_rejected` / `lease_released`).
    LeaseReply,
    /// Immediate `command_ack` or `command_reject`.
    CommandImmediate,
    /// Deferred reply via `ack_tx` or async worker step.
    CommandDeferred,
    /// Immediate `preset_ack` or `preset_reject`.
    PresetReply,
    /// Silent on success; `servo_reject` on error.
    ServoRejectOnly,
}

/// Outcome of a synchronous pre-lock lease-free command check/spawn.
pub(super) enum LeaseFreeOutcome {
    /// Background task spawned onto `worker_tasks`; reply will arrive on `ack_tx`.
    Spawned,
    /// Send this immediate rejection without taking the supervisor lock.
    RejectImmediate(Value),
    /// Read current epoch under supervisor read lock, format `command_reject` for `request_id`, and send.
    RejectWithEpoch { error: ApiError, request_id: Value },
}

/// Context passed to pre-lock lease-free command handlers (`depth_measure`, `teach`).
pub(super) struct LeaseFreeCtx<'a> {
    pub(super) value: Value,
    pub(super) last_ordinary_request: &'a mut Option<u64>,
    pub(super) ws_auth: &'a WsAuth,
    pub(super) recv_supervisor: &'a Arc<ArmSupervisor>,
    pub(super) recv_deps: &'a crate::grasp::GraspDeps,
    pub(super) recv_connection: &'a str,
    pub(super) recv_client_label: &'a Mutex<Option<String>>,
    pub(super) ack_tx: &'a mpsc::Sender<Value>,
    pub(super) worker_tasks: &'a ConnectionTasks,
}

/// Context passed to owner-locked WS handlers (`&mut SupervisorInner` held).
pub(super) struct OwnerDispatchCtx<'a> {
    pub(super) inner: &'a mut SupervisorInner,
    pub(super) stamp: AdmissionStamp,
    pub(super) path: &'a str,
    pub(super) value: &'a Value,
    pub(super) epoch: u64,
    pub(super) local_generation: u64,
    pub(super) recv_conn_id: u64,
    pub(super) recv_connection: &'a str,
    pub(super) recv_generation: &'a AtomicU64,
    pub(super) recv_client_label: &'a Mutex<Option<String>>,
    pub(super) worker_closing: &'a AtomicBool,
    pub(super) ws_auth: &'a WsAuth,
    pub(super) recv_supervisor: &'a Arc<ArmSupervisor>,
    pub(super) recv_deps: &'a crate::grasp::GraspDeps,
    pub(super) recv_calib: &'a crate::calibration::CalibrationDeps,
    #[cfg(feature = "policy-runtime")]
    pub(super) pending_policy_loads: &'a AtomicUsize,
    pub(super) last_sequence: &'a mut Option<u64>,
    pub(super) evicted_notify: &'a mut Option<(String, u64)>,
    pub(super) action_pending: &'a mut actions::ActionPending,
    pub(super) pending_config_write: &'a mut Option<(Value, config::PreparedConfigWrite, u64, u64)>,
    pub(super) pending_calibration: &'a mut Option<(Value, calibration::CalibJob, u64, u64, bool)>,
}

/// Concrete handler binding stored in each [`DispatchEntry`].
#[derive(Clone, Copy)]
pub(super) enum DispatchHandler {
    /// Handled in the reader urgent/dedicated lane (`estop`, `stop`, `auth`).
    ReaderLane,
    /// Pre-lock lease-free command handler (`depth_measure`, `teach::is_lease_free`).
    LeaseFree(fn(LeaseFreeCtx<'_>) -> LeaseFreeOutcome),
    /// Owner-locked handler executed under `supervisor.inner().write()`.
    Owner(fn(&mut OwnerDispatchCtx<'_>) -> Option<Value>),
}

/// One row of the unified WS dispatch table.
#[derive(Clone, Copy)]
pub(super) struct DispatchEntry {
    pub(super) key: DispatchKey,
    pub(super) handler: DispatchHandler,
    pub(super) admission: AdmissionClass,
    pub(super) lease: LeaseRequirement,
    pub(super) auth: AuthRequirement,
    pub(super) reply: ReplyPolicy,
}

/// Core reader-lane entries (`estop`/`e_stop`, `auth`, `/api/v1/motion/stop`).
const CORE_ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::EStop),
        handler: DispatchHandler::ReaderLane,
        admission: AdmissionClass::UrgentEStop,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::UrgentEStop,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Auth),
        handler: DispatchHandler::ReaderLane,
        admission: AdmissionClass::Auth,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::AuthAck,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/stop"),
        handler: DispatchHandler::ReaderLane,
        admission: AdmissionClass::UrgentStop,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::UrgentStop,
    },
];

static DISPATCH_TABLE: LazyLock<HashMap<DispatchKey, &'static DispatchEntry>> = LazyLock::new(|| {
    let slices: &[&[DispatchEntry]] = &[
        CORE_ENTRIES,
        lease::ENTRIES,
        motion::ENTRIES,
        actions::ENTRIES,
        config::ENTRIES,
        servo::ENTRIES,
        policy::ENTRIES,
        calibration::ENTRIES,
    ];
    let mut table = HashMap::new();
    for slice in slices {
        for entry in *slice {
            let prev = table.insert(entry.key, entry);
            assert!(
                prev.is_none(),
                "duplicate WS dispatch table key: {:?}",
                entry.key
            );
        }
    }
    table
});

/// Normalize an inbound WS JSON frame into its canonical [`FrameKind`], preserving exact
/// alias precedence (explicit `type` > `mode: "cartesian_joint_stream"` > implicit `positions`).
pub(super) fn normalize_frame_kind(value: &Value) -> FrameKind {
    match value.get("type").and_then(Value::as_str).unwrap_or("") {
        "estop" | "e_stop" => FrameKind::EStop,
        "auth" => FrameKind::Auth,
        "heartbeat" => FrameKind::Heartbeat,
        "acquire_lease" => FrameKind::AcquireLease,
        "release_lease" => FrameKind::ReleaseLease,
        "command" => FrameKind::Command,
        "preset" => FrameKind::Preset,
        "servo" => FrameKind::Servo,
        _ => {
            if value.get("mode").and_then(Value::as_str) == Some("cartesian_joint_stream") {
                FrameKind::Servo
            } else if value.get("positions").is_some() {
                FrameKind::ImplicitServo
            } else {
                FrameKind::Unknown
            }
        }
    }
}

/// Look up an entry by normalized [`FrameKind`] and exact command path (when `kind == Command`).
pub(super) fn lookup(kind: FrameKind, path: Option<&str>) -> Option<&'static DispatchEntry> {
    let table = &*DISPATCH_TABLE;
    match kind {
        FrameKind::Command => {
            // `COMMAND_TABLE` indexes the command subset of `DISPATCH_TABLE` by `&'static str`
            // so runtime `&str` paths look up directly without allocating.
            COMMAND_TABLE.get(path?).copied()
        }
        other => table.get(&DispatchKey::frame(other)).copied(),
    }
}

static COMMAND_TABLE: LazyLock<HashMap<&'static str, &'static DispatchEntry>> =
    LazyLock::new(|| {
        let mut map = HashMap::new();
        for (key, &entry) in DISPATCH_TABLE.iter() {
            if let DispatchKey {
                kind: FrameKind::Command,
                path: Some(p),
            } = *key
            {
                map.insert(p, entry);
            }
        }
        map
    });

/// Classify an inbound WS JSON frame into `(FrameKind, Option<&'static DispatchEntry>)`.
pub(super) fn classify(value: &Value) -> (FrameKind, Option<&'static DispatchEntry>) {
    let kind = normalize_frame_kind(value);
    let path = if kind == FrameKind::Command {
        value.get("path").and_then(Value::as_str)
    } else {
        None
    };
    (kind, lookup(kind, path))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn dispatch_table_initializes_without_duplicates_and_covers_every_family() {
        let table = &*DISPATCH_TABLE;
        let cmd_table = &*COMMAND_TABLE;
        #[cfg(feature = "policy-runtime")]
        assert_eq!(table.len(), 54, "expected 54 dispatch entries with policy-runtime");
        #[cfg(not(feature = "policy-runtime"))]
        assert_eq!(table.len(), 52, "expected 52 dispatch entries without policy-runtime");
        assert!(cmd_table.len() >= 44);

        // Verify urgent/auth/heartbeat/session admission classes.
        assert_eq!(lookup(FrameKind::EStop, None).unwrap().admission, AdmissionClass::UrgentEStop);
        assert_eq!(lookup(FrameKind::Auth, None).unwrap().admission, AdmissionClass::Auth);
        assert_eq!(lookup(FrameKind::Heartbeat, None).unwrap().admission, AdmissionClass::Heartbeat);
        assert_eq!(lookup(FrameKind::AcquireLease, None).unwrap().admission, AdmissionClass::OrdinarySession);
        assert_eq!(lookup(FrameKind::ReleaseLease, None).unwrap().admission, AdmissionClass::OrdinarySession);
        assert_eq!(
            lookup(FrameKind::Command, Some("/api/v1/motion/stop")).unwrap().admission,
            AdmissionClass::UrgentStop
        );
        for p in ["/api/v1/torque", "/api/v1/arm"] {
            assert_eq!(
                lookup(FrameKind::Command, Some(p)).unwrap().admission,
                AdmissionClass::OrdinaryTorqueOrStopExempt
            );
        }
        // Verify lease-free teach + depth measure entries.
        for p in [
            crate::depth_measure::PATH,
            "/api/v1/record/start",
            "/api/v1/record/stop",
            "/api/v1/episodes/update",
            "/api/v1/episodes/delete",
            "/api/v1/datasets/create",
            "/api/v1/datasets/delete",
        ] {
            let e = lookup(FrameKind::Command, Some(p)).unwrap_or_else(|| panic!("missing {p}"));
            assert_eq!(e.lease, LeaseRequirement::None, "{p}");
            assert!(matches!(e.handler, DispatchHandler::LeaseFree(_)), "{p}");
            if p != crate::depth_measure::PATH {
                assert!(crate::teach::is_lease_free(p), "{p}");
            }
        }
        // Verify grasp dry_run lease exception metadata.
        assert_eq!(
            lookup(FrameKind::Command, Some("/api/v1/action/grasp")).unwrap().lease,
            LeaseRequirement::RequiredExceptDryRun
        );
        // Verify calibration paths all live in calibration::ENTRIES and require Operator auth.
        for p in [
            "/api/v1/calibration/apply",
            "/api/v1/calibration/restore",
            crate::motor_zero::PATH,
            crate::motor_zero::RESTORE_PATH,
            crate::mit::TIMEOUT_PATH,
            crate::mit::QUALIFICATION_PATH,
        ] {
            assert!(calibration::ENTRIES.iter().any(|e| e.key == DispatchKey::command(p)));
            let e = lookup(FrameKind::Command, Some(p)).unwrap();
            assert_eq!(e.auth, AuthRequirement::Operator, "{p}");
        }
    }

    #[test]
    fn alias_precedence_and_implicit_servo_classification_match_legacy() {
        assert_eq!(classify(&json!({"type": "estop"})).0, FrameKind::EStop);
        assert_eq!(classify(&json!({"type": "e_stop"})).0, FrameKind::EStop);
        assert_eq!(classify(&json!({"type": "servo", "positions": [0.0]})).0, FrameKind::Servo);
        assert_eq!(
            classify(&json!({"mode": "cartesian_joint_stream", "positions": [0.0]})).0,
            FrameKind::Servo
        );
        assert_eq!(
            classify(&json!({"positions": [0.0], "sequence_id": 1})).0,
            FrameKind::ImplicitServo
        );
        // Preset carrying `positions` MUST take precedence over implicit servo.
        assert_eq!(
            classify(&json!({"type": "preset", "name": "p", "positions": [0.0]})).0,
            FrameKind::Preset
        );
        // Command carrying `positions` MUST take precedence over implicit servo.
        let (k, e) = classify(&json!({"type": "command", "path": "/api/v1/torque", "positions": [0.0]}));
        assert_eq!(k, FrameKind::Command);
        assert_eq!(e.unwrap().admission, AdmissionClass::OrdinaryTorqueOrStopExempt);
        assert_eq!(classify(&json!({"type": "unknown", "foo": 1})).0, FrameKind::Unknown);
    }
}
