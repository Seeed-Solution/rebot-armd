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
use tokio::sync::{mpsc, watch};

use crate::api::rest::ApiError;
use crate::supervisor::{AdmissionStamp, ArmSupervisor, SupervisorInner};

use super::{
    actions, calibration, config, lease, motion, policy, servo, CancelSignal, ConnectionTasks,
    InFlightStopState, OrdinaryQueue, WsAuth, WsWriterHandle,
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

/// Single-authority operator authorization policy for a frame or command.
///
/// `Operator` is enforced directly by the dispatcher from the table row without any secondary
/// shared path list. `Conditional` / `ConditionalLeaseFree` bind explicit family-owned predicates.
#[derive(Clone, Copy)]
pub(super) enum AuthRequirement {
    /// No operator privilege required.
    None,
    /// Always requires an operator session when operator auth is enforced.
    /// Optional label overrides the operation name in `ApiError::operator_required`; `None` uses `path`.
    Operator(Option<&'static str>),
    /// Conditional operator privilege evaluated under the supervisor write lock by a family predicate.
    Conditional(fn(&WsAuth, &mut SupervisorInner, &str, &Value, u64) -> Option<ApiError>),
    /// Conditional operator privilege evaluated pre-lock for lease-free commands.
    ConditionalLeaseFree(fn(&WsAuth, &str, &Value) -> Option<ApiError>),
    /// Authorization checked inside the handler itself (e.g. `acquire_lease` `force: true` emitting `lease_rejected`).
    InHandler,
}

impl AuthRequirement {
    /// Enforce operator privilege for an owner-locked command row directly from the table entry.
    pub(super) fn check_owner(
        self,
        ws_auth: &WsAuth,
        inner: &mut SupervisorInner,
        path: &str,
        body: &Value,
        epoch: u64,
    ) -> Option<ApiError> {
        match self {
            Self::None | Self::InHandler => None,
            Self::Operator(label) => {
                if ws_auth.is_operator() {
                    None
                } else {
                    Some(ApiError::operator_required(label.unwrap_or(path)))
                }
            }
            Self::Conditional(pred) => pred(ws_auth, inner, path, body, epoch),
            Self::ConditionalLeaseFree(pred) => pred(ws_auth, path, body),
        }
    }

    /// Enforce operator privilege for a pre-lock lease-free command row directly from the table entry.
    pub(super) fn check_lease_free(
        self,
        ws_auth: &WsAuth,
        path: &str,
        body: &Value,
    ) -> Option<ApiError> {
        match self {
            Self::None | Self::InHandler => None,
            Self::Operator(label) => {
                if ws_auth.is_operator() {
                    None
                } else {
                    Some(ApiError::operator_required(label.unwrap_or(path)))
                }
            }
            Self::ConditionalLeaseFree(pred) => pred(ws_auth, path, body),
            Self::Conditional(_) => {
                unreachable!("lease-free row cannot use an owner-locked auth predicate")
            }
        }
    }
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
    /// Immediate synchronous `command_ack` or `command_reject` (never populates deferred async slots).
    CommandImmediate,
    /// May defer reply via async worker/task completion (`ack_tx` or off-lock planning/IO).
    CommandDeferred,
    /// Immediate `preset_ack` or `preset_reject`.
    PresetReply,
    /// Silent on success; `servo_reject` on error.
    ServoRejectOnly,
}

/// Context passed to the bound urgent E-stop reader transition handler.
pub(super) struct UrgentEStopCtx<'a> {
    pub(super) value: &'a Value,
    pub(super) reader_supervisor: &'a Arc<ArmSupervisor>,
    pub(super) reader_cancel_tx: &'a watch::Sender<Option<CancelSignal>>,
    pub(super) reader_writer: &'a WsWriterHandle,
    pub(super) reader_tasks: &'a ConnectionTasks,
}

/// Context passed to the bound urgent Stop reader transition handler.
pub(super) struct UrgentStopCtx<'a> {
    pub(super) value: &'a Value,
    pub(super) reader_supervisor: &'a Arc<ArmSupervisor>,
    pub(super) reader_connection: &'a str,
    pub(super) reader_generation: &'a AtomicU64,
    pub(super) reader_client_label: &'a Mutex<Option<String>>,
    pub(super) reader_last_request: &'a Arc<Mutex<Option<u64>>>,
    pub(super) reader_in_flight_stop: &'a Arc<Mutex<InFlightStopState>>,
    pub(super) reader_ordinary_queue: &'a OrdinaryQueue,
    pub(super) reader_cancel_tx: &'a watch::Sender<Option<CancelSignal>>,
    pub(super) reader_writer: &'a WsWriterHandle,
    pub(super) reader_tasks: &'a ConnectionTasks,
}

/// Context passed to pre-lock lease-free command handlers (`depth_measure`, `teach`).
pub(super) struct LeaseFreeCtx<'a> {
    pub(super) value: Value,
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

/// Concrete executable handler binding stored in each [`DispatchEntry`].
#[derive(Clone, Copy)]
pub(super) enum DispatchHandler {
    /// Lock-free urgent E-stop reader transition (`estop` / `e_stop`). Returns `true` if writer closed.
    UrgentEStop(fn(&mut UrgentEStopCtx<'_>) -> bool),
    /// Urgent non-blocking Stop reader transition (`/api/v1/motion/stop`). Returns `true` if writer closed.
    UrgentStop(fn(&mut UrgentStopCtx<'_>) -> bool),
    /// Dedicated off-reader auth frame processor (`auth`).
    Auth(fn(&WsAuth, &Value) -> Value),
    /// Dedicated latest-wins heartbeat processor (`heartbeat`).
    Heartbeat(
        fn(
            &mut SupervisorInner,
            &Value,
            &str,
            &AtomicU64,
            &Mutex<Option<String>>,
        ) -> Option<Value>,
    ),
    /// Pre-lock lease-free command handler (`depth_measure`, `teach`).
    LeaseFree(fn(LeaseFreeCtx<'_>)),
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

/// Core urgent and auth lane entries (`estop`/`e_stop`, `auth`, `/api/v1/motion/stop`).
const CORE_ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::EStop),
        handler: DispatchHandler::UrgentEStop(super::handle_urgent_estop_frame),
        admission: AdmissionClass::UrgentEStop,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::UrgentEStop,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Auth),
        handler: DispatchHandler::Auth(super::handle_auth_frame),
        admission: AdmissionClass::Auth,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::AuthAck,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/stop"),
        handler: DispatchHandler::UrgentStop(super::handle_urgent_stop_frame),
        admission: AdmissionClass::UrgentStop,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::UrgentStop,
    },
];

fn all_family_slices() -> &'static [&'static [DispatchEntry]] {
    &[
        CORE_ENTRIES,
        lease::ENTRIES,
        motion::ENTRIES,
        actions::ENTRIES,
        config::ENTRIES,
        servo::ENTRIES,
        policy::ENTRIES,
        calibration::ENTRIES,
    ]
}

/// Validate that every table row's `(handler, admission, lease, reply)` metadata is mutually
/// consistent with executable routing at table initialization time.
fn validate_entry(entry: &DispatchEntry) {
    match entry.handler {
        DispatchHandler::UrgentEStop(_) => {
            assert_eq!(entry.admission, AdmissionClass::UrgentEStop, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::None, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::UrgentEStop, "{:?}", entry.key);
        }
        DispatchHandler::UrgentStop(_) => {
            assert_eq!(entry.admission, AdmissionClass::UrgentStop, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::Required, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::UrgentStop, "{:?}", entry.key);
        }
        DispatchHandler::Auth(_) => {
            assert_eq!(entry.admission, AdmissionClass::Auth, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::None, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::AuthAck, "{:?}", entry.key);
        }
        DispatchHandler::Heartbeat(_) => {
            assert_eq!(entry.admission, AdmissionClass::Heartbeat, "{:?}", entry.key);
            assert_eq!(
                entry.lease,
                LeaseRequirement::RequiredAllowZeroGeneration,
                "{:?}",
                entry.key
            );
            assert_eq!(
                entry.reply,
                ReplyPolicy::SilentOrLossOrEpochSync,
                "{:?}",
                entry.key
            );
        }
        DispatchHandler::LeaseFree(_) => {
            assert_eq!(entry.admission, AdmissionClass::Ordinary, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::None, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::CommandDeferred, "{:?}", entry.key);
        }
        DispatchHandler::Owner(_) => {
            assert!(
                matches!(
                    entry.admission,
                    AdmissionClass::OrdinarySession
                        | AdmissionClass::OrdinaryTorqueOrStopExempt
                        | AdmissionClass::Ordinary
                        | AdmissionClass::OrdinaryServo
                ),
                "Owner handler must use Ordinary* admission: {:?}",
                entry.key
            );
            assert!(
                matches!(
                    entry.reply,
                    ReplyPolicy::LeaseReply
                        | ReplyPolicy::CommandImmediate
                        | ReplyPolicy::CommandDeferred
                        | ReplyPolicy::PresetReply
                        | ReplyPolicy::ServoRejectOnly
                ),
                "Owner handler has incompatible reply policy: {:?}",
                entry.key
            );
        }
    }
}

static DISPATCH_TABLE: LazyLock<HashMap<DispatchKey, &'static DispatchEntry>> = LazyLock::new(|| {
    let mut table = HashMap::new();
    for slice in all_family_slices() {
        for entry in *slice {
            validate_entry(entry);
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
    fn dispatch_table_has_no_duplicates_and_binds_real_handlers_per_family() {
        let table = &*DISPATCH_TABLE;
        let expected_total: usize = all_family_slices().iter().map(|s| s.len()).sum();
        assert_eq!(table.len(), expected_total, "duplicate key collapsed in DISPATCH_TABLE");

        // Core urgent & auth handlers are concrete executable variants, not opaque markers.
        let estop = lookup(FrameKind::EStop, None).unwrap();
        assert_eq!(estop.admission, AdmissionClass::UrgentEStop);
        assert!(matches!(estop.handler, DispatchHandler::UrgentEStop(_)));

        let stop = lookup(FrameKind::Command, Some("/api/v1/motion/stop")).unwrap();
        assert_eq!(stop.admission, AdmissionClass::UrgentStop);
        assert!(matches!(stop.handler, DispatchHandler::UrgentStop(_)));

        let auth = lookup(FrameKind::Auth, None).unwrap();
        assert_eq!(auth.admission, AdmissionClass::Auth);
        assert!(matches!(auth.handler, DispatchHandler::Auth(_)));

        let hb = lookup(FrameKind::Heartbeat, None).unwrap();
        assert_eq!(hb.admission, AdmissionClass::Heartbeat);
        assert!(matches!(hb.handler, DispatchHandler::Heartbeat(_)));

        // Lease family invariants.
        for p in ["/api/v1/torque", "/api/v1/arm"] {
            assert_eq!(
                lookup(FrameKind::Command, Some(p)).unwrap().admission,
                AdmissionClass::OrdinaryTorqueOrStopExempt
            );
        }
        assert_eq!(
            lookup(FrameKind::AcquireLease, None).unwrap().admission,
            AdmissionClass::OrdinarySession
        );
        assert_eq!(
            lookup(FrameKind::ReleaseLease, None).unwrap().admission,
            AdmissionClass::OrdinarySession
        );

        // Bidirectional parity between `teach::LEASE_FREE_PATHS` and `actions::ENTRIES`.
        for &p in crate::teach::LEASE_FREE_PATHS {
            let e = lookup(FrameKind::Command, Some(p)).unwrap_or_else(|| panic!("missing teach path {p}"));
            assert_eq!(e.lease, LeaseRequirement::None, "{p}");
            assert!(matches!(e.handler, DispatchHandler::LeaseFree(_)), "{p}");
        }
        for e in actions::ENTRIES {
            if matches!(e.handler, DispatchHandler::LeaseFree(_)) {
                let p = e.key.path.expect("LeaseFree entry must have command path");
                assert!(
                    p == crate::depth_measure::PATH || crate::teach::is_lease_free(p),
                    "unexpected LeaseFree command not in teach::LEASE_FREE_PATHS: {p}"
                );
            }
        }

        // Grasp dry_run lease exception.
        assert_eq!(
            lookup(FrameKind::Command, Some("/api/v1/action/grasp")).unwrap().lease,
            LeaseRequirement::RequiredExceptDryRun
        );

        // Calibration family: all entries enforce Operator auth directly from the table row.
        for e in calibration::ENTRIES {
            assert!(
                matches!(e.auth, AuthRequirement::Operator(_)),
                "calibration row {:?} must require Operator",
                e.key
            );
            assert!(lookup(e.key.kind, e.key.path).is_some());
        }

        // Config family: all entries enforce Operator auth and CommandDeferred reply policy.
        for e in config::ENTRIES {
            assert!(matches!(e.auth, AuthRequirement::Operator(_)));
            assert_eq!(e.reply, ReplyPolicy::CommandDeferred);
            assert!(lookup(e.key.kind, e.key.path).is_some());
        }
    }

    #[test]
    fn operator_auth_requirement_enforces_directly_without_secondary_path_list() {
        let auth = crate::auth::OperatorAuth::enforced("123456", 1800);
        let viewer = WsAuth::new(auth, None, true);
        let sup = crate::supervisor::ArmSupervisor::new(crate::model::ArmConfig::rebot_dm());
        let mut inner = sup.inner().blocking_write();

        // A synthetic command path unknown to ws.rs MUST still be rejected when marked Operator.
        let err = AuthRequirement::Operator(None)
            .check_owner(&viewer, &mut inner, "/api/v1/synthetic_new_family_op", &json!({}), 1)
            .expect("AuthRequirement::Operator must reject non-operator directly");
        assert_eq!(err.code.as_deref(), Some("OPERATOR_REQUIRED"));
        assert!(err.message.contains("/api/v1/synthetic_new_family_op"));

        let lf_err = AuthRequirement::Operator(None)
            .check_lease_free(&viewer, "/api/v1/synthetic_lease_free_op", &json!({}))
            .expect("AuthRequirement::Operator must reject non-operator on lease-free path");
        assert_eq!(lf_err.code.as_deref(), Some("OPERATOR_REQUIRED"));
        assert!(lf_err.message.contains("/api/v1/synthetic_lease_free_op"));

        // Clear-fault pending E-stop interleaving (R2 Major 1): when an E-stop is pending with an
        // outdated epoch, viewer clear_fault must consume it and return EPOCH_MISMATCH (409), not
        // OPERATOR_REQUIRED (403).
        let old_epoch = inner.safety.epoch_id();
        let _ticket = sup.begin_e_stop_checked();
        let clear_entry = lookup(FrameKind::Command, Some("/api/v1/safety/clear_fault")).unwrap();
        let cf_err = clear_entry
            .auth
            .check_owner(&viewer, &mut inner, "/api/v1/safety/clear_fault", &json!({}), old_epoch)
            .expect("pending E-stop must reject clear_fault");
        assert_eq!(cf_err.code.as_deref(), Some("EPOCH_MISMATCH"));
        assert_eq!(cf_err.status, axum::http::StatusCode::CONFLICT);
    }

    #[test]
    #[should_panic]
    fn validate_entry_rejects_mismatched_lease_free_metadata() {
        fn dummy(_ctx: LeaseFreeCtx<'_>) {}
        let bad = DispatchEntry {
            key: DispatchKey::command("/api/v1/bad_lease_free"),
            handler: DispatchHandler::LeaseFree(dummy),
            admission: AdmissionClass::Ordinary,
            lease: LeaseRequirement::Required,
            auth: AuthRequirement::None,
            reply: ReplyPolicy::CommandDeferred,
        };
        validate_entry(&bad);
    }

    #[test]
    fn alias_precedence_and_legacy_mixed_field_cancellation_parity() {
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
        assert_eq!(
            classify(&json!({"type": "preset", "name": "p", "positions": [0.0]})).0,
            FrameKind::Preset
        );
        let (k, e) = classify(&json!({"type": "command", "path": "/api/v1/torque", "positions": [0.0]}));
        assert_eq!(k, FrameKind::Command);
        assert_eq!(e.unwrap().admission, AdmissionClass::OrdinaryTorqueOrStopExempt);
        assert_eq!(classify(&json!({"type": "unknown", "foo": 1})).0, FrameKind::Unknown);

        // Legacy Stop/E-stop cancellation parity (Major 1): mixed-field lease frames carrying
        // `positions` or `mode: "cartesian_joint_stream"` MUST be drained and emit `servo_reject`.
        let q = OrdinaryQueue::new();
        let stamp = AdmissionStamp {
            stop_gen: 0,
            estop_epoch: 1,
            lease_gen: 1,
            connection_id: 1,
            request_id: 1,
        };
        let mixed_release = json!({"type": "release_lease", "positions": null});
        let mixed_acquire = json!({"type": "acquire_lease", "client_id": "a", "mode": "cartesian_joint_stream"});
        let clean_release = json!({"type": "release_lease"});
        assert!(q.try_push(mixed_release.clone(), stamp).is_ok());
        assert!(q.try_push(mixed_acquire.clone(), stamp).is_ok());
        assert!(q.try_push(clean_release.clone(), stamp).is_ok());

        let replies = q.drain_cancellable("STOPPED", Some(7));
        assert_eq!(replies.len(), 2, "both mixed-field lease frames must cancel like baseline");
        for r in &replies {
            assert_eq!(r["type"], "servo_reject");
            assert_eq!(r["reason"], "stopped");
            assert_eq!(r["code"], "STOPPED");
            assert_eq!(r["epoch_id"], 7);
        }
        // Clean release_lease without servo fields remains queued.
        assert_eq!(q.len(), 1);
    }
}
