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
use crate::api::routes::{FaultAvailability, RouteOwnership};
use crate::supervisor::{AdmissionStamp, ArmSupervisor, SupervisorInner};

use super::{
    actions, calibration, config, lease, motion, policy, servo, CancelSignal, ConnectionTasks,
    InFlightStopState, OrdinaryQueue, WsAuth, WsWriterHandle,
};

/// Normalized inbound WebSocket frame kind, preserving exact alias precedence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrameKind {
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

impl FrameKind {
    pub(super) const fn as_static_str(self) -> &'static str {
        match self {
            Self::EStop => "estop",
            Self::Auth => "auth",
            Self::Heartbeat => "heartbeat",
            Self::AcquireLease => "acquire_lease",
            Self::ReleaseLease => "release_lease",
            Self::Command => "command",
            Self::Preset => "preset",
            Self::Servo => "servo",
            Self::ImplicitServo => "implicit_servo",
            Self::Unknown => "unknown",
        }
    }
}

/// Primary lookup key in the unified WS dispatch table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct DispatchKey {
    pub(crate) kind: FrameKind,
    pub(crate) path: Option<&'static str>,
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
pub(crate) enum LeaseRequirement {
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
    /// Owner-side operator authorization policy evaluated live against a lock-free [`OperatorGrant`]
    /// (shared across legacy lock and RT executor with zero heap allocation).
    OwnerGrant(crate::rt_core::requests::lease::OwnerAuthPolicy),
    /// Conditional operator privilege evaluated under the supervisor write lock by a family predicate.
    Conditional(fn(&WsAuth, &mut SupervisorInner, &str, &Value, u64) -> Option<ApiError>),
    /// Conditional operator privilege evaluated pre-lock for lease-free commands.
    ConditionalLeaseFree(fn(&WsAuth, &str, &Value) -> Option<ApiError>),
    /// Authorization checked inside the handler itself (e.g. `acquire_lease` `force: true` emitting `lease_rejected`).
    InHandler,
}

impl AuthRequirement {
    /// Converts a table-declared auth policy into the zero-alloc owner-side policy used by
    /// `DispatchHandler::CommandAsync` handlers across legacy and RT executor owners.
    pub(super) fn owner_auth_policy(self) -> Option<crate::rt_core::requests::lease::OwnerAuthPolicy> {
        use crate::rt_core::requests::lease::OwnerAuthPolicy;
        match self {
            Self::None => Some(OwnerAuthPolicy::None),
            Self::Operator(label) => Some(OwnerAuthPolicy::Operator(label)),
            Self::OwnerGrant(policy) => Some(policy),
            Self::Conditional(_) | Self::ConditionalLeaseFree(_) | Self::InHandler => None,
        }
    }

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
            Self::OwnerGrant(policy) => {
                use crate::rt_core::requests::lease::{evaluate_owner_auth_policy, OwnerAuthError};
                match evaluate_owner_auth_policy(
                    policy,
                    ws_auth.operator_grant(),
                    inner,
                    epoch,
                    std::time::Instant::now(),
                    "/api/v1/safety/clear_fault",
                ) {
                    Ok(()) => None,
                    Err(OwnerAuthError::EpochMismatch { expected, received }) => Some(
                        crate::supervisor::ArmError::Safety(crate::safety::SafetyError::EpochMismatch {
                            expected,
                            received,
                        })
                        .into(),
                    ),
                    Err(OwnerAuthError::OperatorRequired { operation }) => {
                        Some(ApiError::operator_required(operation))
                    }
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
            Self::Conditional(_) | Self::OwnerGrant(_) => {
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
    pub(super) received_at: std::time::Instant,
    pub(super) path: &'a str,
    pub(super) value: &'a Value,
    pub(super) epoch: u64,
    pub(super) local_generation: u64,
    pub(super) ws_auth: &'a WsAuth,
    pub(super) recv_supervisor: &'a Arc<ArmSupervisor>,
    pub(super) recv_deps: &'a crate::grasp::GraspDeps,
    pub(super) recv_calib: &'a crate::calibration::CalibrationDeps,
    pub(super) last_sequence: &'a mut Option<u64>,
    pub(super) action_pending: &'a mut actions::ActionPending,
    pub(super) pending_motion: &'a mut Option<motion::PendingMotion>,
    pub(super) cached_motion_cfg: Option<&'a Arc<crate::rt_core::requests::motion::MotionConfigSnapshot>>,
    pub(super) preparsed_motion: &'a mut Option<motion::PreparsedMotion>,
    pub(super) pending_config_write: &'a mut Option<config::PendingConfigJob>,
    pub(super) pending_calibration: &'a mut Option<(Value, calibration::CalibJob, u64, u64, bool)>,
}

/// Lightweight WS reply payload captured under `SupervisorLock` and formatted into JSON
/// strictly AFTER the lock guard is released (zero JSON serialization under the guard).
#[allow(clippy::large_enum_variant)]
pub(super) enum OwnerReply {
    CommandAck {
        snap: super::AckSnapshot,
        started: super::Started,
    },
    CommandAckJob {
        snap: super::AckSnapshot,
        started: super::Started,
        job_id: String,
    },
    CommandReject {
        epoch: u64,
        error: ApiError,
    },
    PresetReject {
        epoch: u64,
        name: String,
        error: ApiError,
    },
    PresetCalibrationBusy {
        epoch: u64,
    },
    ServoRejectReason(&'static str),
    ServoRejectApi {
        epoch: u64,
        error: ApiError,
    },
    FieldsAck(Value),
    CalibReject(crate::calibration::CalibError),
    LeaseLost {
        epoch: u64,
        local_generation: u64,
        reason: Option<&'static str>,
    },
    EpochMismatch {
        epoch: u64,
        safety_state: crate::safety::SafetyState,
    },
    InvalidRequestId,
    #[cfg(feature = "policy-runtime")]
    PolicyStopAck {
        snap: super::AckSnapshot,
        stopped: bool,
    },
}

impl OwnerReply {
    pub(super) fn into_value(
        self,
        raw_kind: &str,
        frame_kind: FrameKind,
        value: &Value,
        #[cfg(feature = "policy-runtime")] pending_policy_loads: &AtomicUsize,
    ) -> Value {
        match self {
            Self::CommandAck { snap, started } => {
                super::command_ack_from_snap(&snap, &value["request_id"], started)
            }
            Self::CommandAckJob {
                snap,
                started,
                job_id,
            } => {
                let mut ack = super::command_ack_from_snap(&snap, &value["request_id"], started);
                ack["job_id"] = serde_json::json!(job_id);
                ack
            }
            Self::CommandReject { epoch, error } => {
                super::command_reject_at(epoch, &value["request_id"], error)
            }
            Self::PresetReject { epoch, name, error } => {
                let mut reply = super::rejection(error, "preset_reject", epoch);
                reply["name"] = serde_json::json!(name);
                reply
            }
            Self::PresetCalibrationBusy { epoch } => serde_json::json!({
                "type": "preset_reject",
                "name": value.get("name"),
                "status": 409,
                "code": "MOTION_BUSY",
                "error": "MOTION_BUSY",
                "message": "a camera calibration apply or restore is running",
                "epoch_id": epoch,
            }),
            Self::ServoRejectReason(reason) => {
                serde_json::json!({"type": "servo_reject", "reason": reason})
            }
            Self::ServoRejectApi { epoch, error } => {
                let stopped = error.code.as_deref() == Some("STOPPED");
                let mut r = super::rejection(error, "servo_reject", epoch);
                if stopped {
                    r["reason"] = serde_json::json!("stopped");
                }
                r
            }
            Self::FieldsAck(reply) => super::fields_ack(&value["request_id"], reply),
            Self::CalibReject(e) => calibration::calibration_reject(&value["request_id"], &e),
            Self::LeaseLost {
                epoch,
                local_generation,
                reason,
            } => {
                if raw_kind == "command" {
                    serde_json::json!({
                        "type": "command_reject",
                        "request_id": value.get("request_id"),
                        "status": 409,
                        "code": "LEASE_LOST",
                        "message": "This connection does not own an active lease",
                        "error": "LEASE_LOST",
                        "epoch_id": epoch,
                    })
                } else if raw_kind == "release_lease" {
                    serde_json::json!({"type": "lease_released", "success": false})
                } else if local_generation == 0 {
                    serde_json::json!({"type": "lease_lost", "generation": 0})
                } else {
                    serde_json::json!({
                        "type": "lease_lost",
                        "generation": local_generation,
                        "reason": reason.unwrap_or("expired"),
                    })
                }
            }
            Self::EpochMismatch {
                epoch,
                safety_state,
            } => {
                if frame_kind == FrameKind::Command {
                    serde_json::json!({
                        "type": "command_reject",
                        "request_id": value.get("request_id"),
                        "status": 409,
                        "code": "EPOCH_MISMATCH",
                        "message": "Command epoch is no longer current",
                        "error": "EPOCH_MISMATCH",
                        "epoch_id": epoch,
                    })
                } else {
                    serde_json::json!({
                        "type": "epoch_sync",
                        "epoch_id": epoch,
                        "safety_state": safety_state,
                    })
                }
            }
            Self::InvalidRequestId => serde_json::json!({
                "type": "command_reject",
                "request_id": value.get("request_id"),
                "status": 400,
                "code": "INVALID_COMMAND",
                "message": "request_id must be present and strictly increase",
                "error": "INVALID_COMMAND",
            }),
            #[cfg(feature = "policy-runtime")]
            Self::PolicyStopAck { snap, stopped } => {
                let had_pending = pending_policy_loads.load(std::sync::atomic::Ordering::SeqCst) > 0;
                let mut ack = super::command_ack_from_snap(&snap, &value["request_id"], None);
                ack["stopped"] = serde_json::json!(stopped || had_pending);
                ack
            }
        }
    }
}

/// Boxed future returned by the table-bound unified heartbeat handler.
pub(super) type HeartbeatFuture<'a> =
    std::pin::Pin<Box<dyn std::future::Future<Output = Option<Value>> + Send + 'a>>;

/// Arguments passed to unified async session handlers (`acquire_lease`, `release_lease`).
pub(super) struct SessionDispatchCtx<'a> {
    pub(super) supervisor: &'a ArmSupervisor,
    pub(super) value: &'a Value,
    pub(super) conn_handle: crate::rt_core::requests::lease::ConnectionHandle,
    pub(super) recv_generation: &'a AtomicU64,
    pub(super) recv_client_label: &'a Mutex<Option<String>>,
    pub(super) worker_closing: &'a AtomicBool,
    pub(super) ws_auth: &'a WsAuth,
    pub(super) last_sequence: &'a mut Option<u64>,
}

/// Boxed future returned by table-bound unified async session handlers (`acquire_lease`, `release_lease`).
pub(super) type SessionFuture<'a> = std::pin::Pin<
    Box<dyn std::future::Future<Output = (Option<Value>, Option<(String, u64)>)> + Send + 'a>,
>;

/// Arguments passed to unified async command handlers (`/api/v1/torque`, `/api/v1/arm`,
/// `/api/v1/safety/clear_fault`) routed through the active owner (legacy or RT executor).
///
/// Both `local_generation` (from `entry.lease`) and `auth_policy` (from `entry.auth`) are enforced
/// and supplied directly by the dispatcher from the [`DispatchEntry`] row.
pub(super) struct CommandAsyncCtx<'a> {
    pub(super) supervisor: &'a ArmSupervisor,
    pub(super) value: &'a Value,
    pub(super) stamp: AdmissionStamp,
    pub(super) conn_handle: crate::rt_core::requests::lease::ConnectionHandle,
    pub(super) local_generation: u64,
    pub(super) auth_policy: crate::rt_core::requests::lease::OwnerAuthPolicy,
    pub(super) operator: crate::auth::OperatorGrant,
    pub(super) last_ordinary_request: &'a mut Option<u64>,
}

/// Boxed future returned by table-bound unified async command handlers.
pub(super) type CommandAsyncFuture<'a> =
    std::pin::Pin<Box<dyn std::future::Future<Output = Value> + Send + 'a>>;

/// Concrete executable handler binding stored in each [`DispatchEntry`].
#[derive(Clone, Copy)]
pub(super) enum DispatchHandler {
    /// Lock-free urgent E-stop reader transition (`estop` / `e_stop`). Returns `true` if writer closed.
    UrgentEStop(fn(&mut UrgentEStopCtx<'_>) -> bool),
    /// Urgent non-blocking Stop reader transition (`/api/v1/motion/stop`). Returns `true` if writer closed.
    UrgentStop(fn(&mut UrgentStopCtx<'_>) -> bool),
    /// Dedicated off-reader auth frame processor (`auth`).
    Auth(fn(&WsAuth, &Value) -> Value),
    /// Dedicated latest-wins heartbeat processor (`heartbeat`), routed through the active owner.
    Heartbeat(
        for<'a> fn(
            &'a ArmSupervisor,
            &'a Value,
            crate::rt_core::requests::lease::ConnectionHandle,
            &'a AtomicU64,
            &'a Mutex<Option<String>>,
        ) -> HeartbeatFuture<'a>,
    ),
    /// Unified async session handler (`acquire_lease`, `release_lease`), routed through the active owner.
    SessionAsync(for<'a> fn(SessionDispatchCtx<'a>) -> SessionFuture<'a>),
    /// Unified async command handler (`/api/v1/torque`, `/api/v1/arm`, `/api/v1/safety/clear_fault`),
    /// routed through the active owner (legacy or RT executor) with immediate wire reply.
    CommandAsync(for<'a> fn(CommandAsyncCtx<'a>) -> CommandAsyncFuture<'a>),
    /// Pre-lock lease-free command handler (`depth_measure`, `teach`).
    LeaseFree(fn(LeaseFreeCtx<'_>)),
    /// Owner-locked handler executed under `supervisor.inner().write()`.
    Owner(fn(&mut OwnerDispatchCtx<'_>) -> Option<OwnerReply>),
}

/// Direct RT-executor dispatch target for a WS entry (replaces sequential owner probing).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ExecutorWsRoute {
    /// Not routed on the RT executor (`RouteOwnership::LegacyOnly`).
    None,
    /// Dispatched via unified / owner-agnostic lane (`UrgentEStop`, `UrgentStop`, `Auth`,
    /// `Heartbeat`, `SessionAsync`, `CommandAsync`).
    Unified,
    /// Direct dispatch to `servo::handle_executor_ws_servo`.
    Servo,
    /// Direct dispatch to `config::handle_executor_ws_command` (config, calibration, gravity_comp).
    ConfigOrCalib,
    /// Direct dispatch to `motion::handle_executor_ws_motion`.
    Motion,
    /// Direct dispatch to `actions::handle_executor_ws_action`.
    Action,
    /// Direct dispatch to pre-lock lease-free handler (teach recording/episodes/datasets).
    LeaseFree,
    /// Direct dispatch to `policy::handle_executor_ws_policy`.
    #[cfg(feature = "policy-runtime")]
    Policy,
}

/// One row of the unified WS dispatch table.
#[derive(Clone, Copy)]
pub(crate) struct DispatchEntry {
    pub(crate) key: DispatchKey,
    pub(super) handler: DispatchHandler,
    pub(crate) handler_name: &'static str,
    pub(super) admission: AdmissionClass,
    pub(super) lease: LeaseRequirement,
    pub(super) auth: AuthRequirement,
    pub(super) reply: ReplyPolicy,
    pub(crate) ownership: RouteOwnership,
    pub(crate) fault: FaultAvailability,
    pub(super) executor_route: ExecutorWsRoute,
}

impl DispatchEntry {
    /// Projects this WS dispatch row into the unified [`crate::api::routes::RouteSpec`].
    pub(crate) fn route_spec(&self) -> crate::api::routes::RouteSpec {
        use crate::api::routes::{LeasePolicy, OperatorPolicy, RouteSpec};
        let operator = match self.auth {
            AuthRequirement::None => OperatorPolicy::None,
            AuthRequirement::Operator(_) => OperatorPolicy::Required,
            AuthRequirement::OwnerGrant(_) => OperatorPolicy::OwnerGrant,
            AuthRequirement::Conditional(_) | AuthRequirement::ConditionalLeaseFree(_) => {
                OperatorPolicy::Conditional
            }
            AuthRequirement::InHandler => OperatorPolicy::InHandler,
        };
        let lease = match self.lease {
            LeaseRequirement::None => LeasePolicy::None,
            LeaseRequirement::Required => LeasePolicy::Required,
            LeaseRequirement::RequiredExceptDryRun => LeasePolicy::RequiredExceptDryRun,
            LeaseRequirement::RequiredExplicitGeneration => LeasePolicy::RequiredExplicitGeneration,
            LeaseRequirement::RequiredAllowZeroGeneration => {
                LeasePolicy::RequiredAllowZeroGeneration
            }
        };
        RouteSpec {
            handler: self.handler_name,
            ownership: self.ownership,
            operator,
            lease,
            fault: self.fault,
            owner_unavailable: crate::api::routes::OwnerUnavailableAvailability::Fenced,
        }
    }
}

/// Core urgent and auth lane entries (`estop`/`e_stop`, `auth`, `/api/v1/motion/stop`).
const CORE_ENTRIES: &[DispatchEntry] = &[
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::EStop),
        handler: DispatchHandler::UrgentEStop(super::handle_urgent_estop_frame),
        handler_name: "ws::handle_urgent_estop_frame",
        admission: AdmissionClass::UrgentEStop,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::UrgentEStop,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Allowed,
        executor_route: ExecutorWsRoute::Unified,
    },
    DispatchEntry {
        key: DispatchKey::frame(FrameKind::Auth),
        handler: DispatchHandler::Auth(super::handle_auth_frame),
        handler_name: "ws::handle_auth_frame",
        admission: AdmissionClass::Auth,
        lease: LeaseRequirement::None,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::AuthAck,
        ownership: RouteOwnership::OwnerAgnostic,
        fault: FaultAvailability::Allowed,
        executor_route: ExecutorWsRoute::Unified,
    },
    DispatchEntry {
        key: DispatchKey::command("/api/v1/motion/stop"),
        handler: DispatchHandler::UrgentStop(super::handle_urgent_stop_frame),
        handler_name: "ws::handle_urgent_stop_frame",
        admission: AdmissionClass::UrgentStop,
        lease: LeaseRequirement::Required,
        auth: AuthRequirement::None,
        reply: ReplyPolicy::UrgentStop,
        ownership: RouteOwnership::ExecutorRequest,
        fault: FaultAvailability::Allowed,
        executor_route: ExecutorWsRoute::Unified,
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

/// Iterates every registered WS [`DispatchEntry`] across all family tables (used by `api::routes`).
pub(crate) fn all_entries() -> impl Iterator<Item = &'static DispatchEntry> {
    all_family_slices().iter().flat_map(|slice| slice.iter())
}

/// Validate that every table row's `(handler, admission, lease, reply, ownership, executor_route)`
/// metadata is mutually consistent with executable routing at table initialization time.
fn validate_entry(entry: &DispatchEntry) {
    assert!(!entry.handler_name.is_empty(), "{:?}", entry.key);
    match entry.ownership {
        RouteOwnership::LegacyOnly => {
            assert_eq!(entry.executor_route, ExecutorWsRoute::None, "{:?}", entry.key);
        }
        RouteOwnership::OwnerAgnostic | RouteOwnership::ExecutorRequest => {
            assert_ne!(entry.executor_route, ExecutorWsRoute::None, "{:?}", entry.key);
        }
    }
    match entry.handler {
        DispatchHandler::UrgentEStop(_) => {
            assert_eq!(entry.admission, AdmissionClass::UrgentEStop, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::None, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::UrgentEStop, "{:?}", entry.key);
            assert_eq!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
        }
        DispatchHandler::UrgentStop(_) => {
            assert_eq!(entry.admission, AdmissionClass::UrgentStop, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::Required, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::UrgentStop, "{:?}", entry.key);
            assert_eq!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
        }
        DispatchHandler::Auth(_) => {
            assert_eq!(entry.admission, AdmissionClass::Auth, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::None, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::AuthAck, "{:?}", entry.key);
            assert_eq!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
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
            assert_eq!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
        }
        DispatchHandler::SessionAsync(_) => {
            assert_eq!(entry.admission, AdmissionClass::OrdinarySession, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::LeaseReply, "{:?}", entry.key);
            assert_eq!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
        }
        DispatchHandler::CommandAsync(_) => {
            assert!(
                matches!(
                    entry.admission,
                    AdmissionClass::OrdinaryTorqueOrStopExempt | AdmissionClass::Ordinary
                ),
                "CommandAsync handler must use Ordinary/OrdinaryTorqueOrStopExempt admission: {:?}",
                entry.key
            );
            assert!(
                matches!(entry.lease, LeaseRequirement::Required | LeaseRequirement::None),
                "CommandAsync handler must declare Required or None lease policy: {:?}",
                entry.key
            );
            assert!(
                entry.auth.owner_auth_policy().is_some(),
                "CommandAsync handler must declare None, Operator, or OwnerGrant auth policy: {:?}",
                entry.key
            );
            assert_eq!(entry.reply, ReplyPolicy::CommandImmediate, "{:?}", entry.key);
            assert_eq!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
        }
        DispatchHandler::LeaseFree(_) => {
            assert_eq!(entry.admission, AdmissionClass::Ordinary, "{:?}", entry.key);
            assert_eq!(entry.lease, LeaseRequirement::None, "{:?}", entry.key);
            assert_eq!(entry.reply, ReplyPolicy::CommandDeferred, "{:?}", entry.key);
            assert!(
                matches!(
                    entry.executor_route,
                    ExecutorWsRoute::None | ExecutorWsRoute::Unified | ExecutorWsRoute::LeaseFree
                ),
                "LeaseFree handler must use None (LegacyOnly), Unified or LeaseFree (ExecutorRequest) route: {:?}",
                entry.key
            );
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
            assert_ne!(entry.executor_route, ExecutorWsRoute::Unified, "{:?}", entry.key);
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

/// Builds both dispatch tables. Called once when the router is created, before any connection is
/// accepted, so that a connection's reader never waits on the lazy initialisation (run by
/// another connection's task) before it latches an E-stop or hands over a Stop.
pub(super) fn prewarm_tables() {
    LazyLock::force(&COMMAND_TABLE);
}

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
            handler_name: "dummy",
            admission: AdmissionClass::Ordinary,
            lease: LeaseRequirement::Required,
            auth: AuthRequirement::None,
            reply: ReplyPolicy::CommandDeferred,
            ownership: RouteOwnership::LegacyOnly,
            fault: FaultAvailability::Fenced,
            executor_route: ExecutorWsRoute::None,
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
        let now = std::time::Instant::now();
        assert!(q.try_push(mixed_release.clone(), stamp, now).is_ok());
        assert!(q.try_push(mixed_acquire.clone(), stamp, now).is_ok());
        assert!(q.try_push(clean_release.clone(), stamp, now).is_ok());

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
