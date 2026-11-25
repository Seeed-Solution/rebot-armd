//! Lease lifecycle, preemption, heartbeat, and priority connection teardown (PR L / spec A2).
//!
//! Single authority for:
//! - Bounded zero-allocation [`ConnectionHandle`] and [`LeaseRequest`] / [`LeaseOutcome`] types.
//! - Off-thread request preparation (`prepare_*`) working strictly from published
//!   [`OwnerStateSnapshot`]s.
//! - Single owner-side install functions (`install_*`) called identically by both the legacy
//!   owner (under a short write lock) and the synchronous RT executor (`armd-control`).
//! - Lossless 1:1 slot-reserved priority connection lifecycle channel (`ConnLifecycleClient` /
//!   `ConnLifecycleIntake`) capped at [`MAX_LIVE_CONNECTIONS`] so disconnect never drops even
//!   when the ordinary broker command ring is 100% saturated.

use std::collections::{HashMap, VecDeque};
use std::fmt::{self, Write as _};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use rtrb::{Consumer, Producer, RingBuffer};

use crate::auth::OperatorGrant;
use crate::model::HolderLabel;
use crate::rt_core::broker::{Authority, BootId, ClientId, Descriptor};
use crate::rt_core::ids::ArmId;
use crate::rt_core::ops::CommitCtx;
use crate::rt_core::publish::OwnerStateSnapshot;
use crate::safety::SafetyState;
use crate::supervisor::{SupervisorInner, MAX_LIVE_CONNECTIONS, PREEMPT_COOLDOWN};

/// Maximum UTF-8 byte length of a bounded [`ConnectionHandle`] name.
pub const MAX_CONNECTION_NAME_BYTES: usize = 64;

/// Capacity of the priority connection lifecycle SPSC rings: each of the
/// [`MAX_LIVE_CONNECTIONS`] live connection slots holds 1 `Open` credit + 1 `Disconnect` credit,
/// so valid lifecycle operations can never encounter a full ring.
pub const CONN_LIFECYCLE_CAPACITY: usize = MAX_LIVE_CONNECTIONS * 2;

/// Default monotonic deadline window for lease/heartbeat requests.
pub const DEFAULT_LEASE_REQUEST_DEADLINE: Duration = Duration::from_secs(5);

struct StackBuf<const N: usize> {
    buf: [u8; N],
    len: usize,
    overflowed: bool,
}

impl<const N: usize> StackBuf<N> {
    const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
            overflowed: false,
        }
    }
}

impl<const N: usize> fmt::Write for StackBuf<N> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        if self.len.saturating_add(bytes.len()) > N {
            self.overflowed = true;
            return Err(fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

/// Bounded, `Copy`, zero-allocation connection identity carrying both the server-assigned
/// monotonic WS connection ID (`>= 1`, or `0` for non-WS handles) and its bounded UTF-8 name.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConnectionHandle {
    id: u64,
    len: u8,
    bytes: [u8; MAX_CONNECTION_NAME_BYTES],
}

impl ConnectionHandle {
    /// Builds a WS connection handle `"connection:{id}"` without heap allocation.
    /// Fails closed (`None`) when `id == 0`.
    pub fn from_ws_id(id: u64) -> Option<Self> {
        if id == 0 {
            return None;
        }
        let mut out = StackBuf::<MAX_CONNECTION_NAME_BYTES>::new();
        if write!(out, "connection:{id}").is_err() || out.overflowed || out.len == 0 {
            return None;
        }
        Some(Self {
            id,
            len: out.len as u8,
            bytes: out.buf,
        })
    }

    /// Builds a bounded connection handle from `(id, name)` without heap allocation.
    /// Fails closed (`None`) when `name` is empty or exceeds [`MAX_CONNECTION_NAME_BYTES`] bytes.
    pub fn try_new(id: u64, name: &str) -> Option<Self> {
        let raw = name.as_bytes();
        if raw.is_empty() || raw.len() > MAX_CONNECTION_NAME_BYTES {
            return None;
        }
        let mut bytes = [0u8; MAX_CONNECTION_NAME_BYTES];
        bytes[..raw.len()].copy_from_slice(raw);
        Some(Self {
            id,
            len: raw.len() as u8,
            bytes,
        })
    }

    #[inline]
    pub fn id(&self) -> u64 {
        self.id
    }

    #[inline]
    pub fn as_str(&self) -> &str {
        // SAFETY: `bytes[..len]` is always populated from valid UTF-8 `&str` or ASCII digits.
        unsafe { std::str::from_utf8_unchecked(&self.bytes[..usize::from(self.len)]) }
    }

    #[inline]
    pub fn to_holder_label(&self) -> HolderLabel {
        HolderLabel::new(self.as_str())
    }
}

impl fmt::Debug for ConnectionHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ConnectionHandle")
            .field("id", &self.id)
            .field("name", &self.as_str())
            .finish()
    }
}

impl fmt::Display for ConnectionHandle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Acquire the control lease if unheld or expired (or renew if already held by `connection`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeaseAcquire {
    pub connection: ConnectionHandle,
    pub label: HolderLabel,
    pub has_label: bool,
}

/// Force-preempt the control lease with live operator authorization and cooldown enforcement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeasePreempt {
    pub connection: ConnectionHandle,
    pub label: HolderLabel,
    pub operator: OperatorGrant,
}

/// Release the control lease owned by `connection` at `generation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeaseRelease {
    pub connection: ConnectionHandle,
    pub generation: u64,
}

/// Close `connection`, fence its lease generation, stop active motion if valid owner, and release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeaseDisconnect {
    pub connection: ConnectionHandle,
}

/// Lease family broker request variants (`Copy`, zero allocation on RT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeaseRequest {
    Acquire(LeaseAcquire),
    Preempt(LeasePreempt),
    Release(LeaseRelease),
    Disconnect(LeaseDisconnect),
}

impl LeaseRequest {
    #[inline]
    pub fn connection(&self) -> ConnectionHandle {
        match self {
            Self::Acquire(r) => r.connection,
            Self::Preempt(r) => r.connection,
            Self::Release(r) => r.connection,
            Self::Disconnect(r) => r.connection,
        }
    }
}

/// Owner outcome of a [`LeaseRequest`] (`Copy`, zero allocation on RT).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeaseOutcome {
    Granted {
        generation: u64,
        evicted_connection: Option<ConnectionHandle>,
        evicted_generation: u64,
    },
    Busy {
        holder_generation: u64,
    },
    Cooldown {
        remaining_ms: u64,
    },
    OperatorRequired,
    Released {
        released: bool,
        displaced_generation: u64,
    },
    Disconnected {
        released: bool,
        displaced_generation: u64,
    },
}

/// Validation errors returned during off-thread lease preparation (never mutates owner state).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeasePrepareError {
    InvalidClientId,
    InvalidConnection,
    SnapshotUnavailable,
}

/// Prepared `LeaseAcquire` built off-thread from a published [`OwnerStateSnapshot`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedLeaseAcquire {
    pub request: LeaseAcquire,
    pub arm: ArmId,
    pub boot: BootId,
    pub authority: Authority,
    pub client: ClientId,
    pub client_seq: u64,
    pub deadline: Instant,
    pub request_id: u64,
}

/// Prepared `LeasePreempt` built off-thread from a published [`OwnerStateSnapshot`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedLeasePreempt {
    pub request: LeasePreempt,
    pub arm: ArmId,
    pub boot: BootId,
    pub authority: Authority,
    pub client: ClientId,
    pub client_seq: u64,
    pub deadline: Instant,
    pub request_id: u64,
}

/// Prepared `LeaseRelease` built off-thread from a published [`OwnerStateSnapshot`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedLeaseRelease {
    pub request: LeaseRelease,
    pub arm: ArmId,
    pub boot: BootId,
    pub authority: Authority,
    pub client: ClientId,
    pub deadline: Instant,
    pub request_id: u64,
}

/// Prepared `LeaseDisconnect` (never blocked by snapshot unavailability).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedLeaseDisconnect {
    pub request: LeaseDisconnect,
    pub client: ClientId,
}

/// Off-thread preparation for a WS `acquire_lease` (`force == false`) from a published snapshot.
pub fn prepare_lease_acquire(
    snap: &OwnerStateSnapshot,
    connection: ConnectionHandle,
    label: &str,
    client_seq: u64,
    now: Instant,
    deadline: Instant,
) -> Result<PreparedLeaseAcquire, LeasePrepareError> {
    if label.is_empty() || label.len() > 128 {
        return Err(LeasePrepareError::InvalidClientId);
    }
    if snap.boot.is_zero() {
        return Err(LeasePrepareError::SnapshotUnavailable);
    }
    Ok(PreparedLeaseAcquire {
        request: LeaseAcquire {
            connection,
            label: HolderLabel::new(label),
            has_label: true,
        },
        arm: snap.arm,
        boot: snap.boot,
        authority: snap.authority_at(now),
        client: ClientId(connection.id()),
        client_seq,
        deadline,
        request_id: crate::supervisor::next_admission_request_id(),
    })
}

/// Off-thread preparation for a WS `acquire_lease` (`force == true`) from a published snapshot.
pub fn prepare_lease_preempt(
    snap: &OwnerStateSnapshot,
    connection: ConnectionHandle,
    label: &str,
    operator: OperatorGrant,
    client_seq: u64,
    now: Instant,
    deadline: Instant,
) -> Result<PreparedLeasePreempt, LeasePrepareError> {
    if label.is_empty() || label.len() > 128 {
        return Err(LeasePrepareError::InvalidClientId);
    }
    if snap.boot.is_zero() {
        return Err(LeasePrepareError::SnapshotUnavailable);
    }
    Ok(PreparedLeasePreempt {
        request: LeasePreempt {
            connection,
            label: HolderLabel::new(label),
            operator,
        },
        arm: snap.arm,
        boot: snap.boot,
        authority: snap.authority_at(now),
        client: ClientId(connection.id()),
        client_seq,
        deadline,
        request_id: crate::supervisor::next_admission_request_id(),
    })
}

/// Off-thread preparation for a WS `release_lease` from a published snapshot.
pub fn prepare_lease_release(
    snap: &OwnerStateSnapshot,
    connection: ConnectionHandle,
    generation: u64,
    now: Instant,
    deadline: Instant,
) -> Result<PreparedLeaseRelease, LeasePrepareError> {
    if snap.boot.is_zero() {
        return Err(LeasePrepareError::SnapshotUnavailable);
    }
    Ok(PreparedLeaseRelease {
        request: LeaseRelease {
            connection,
            generation,
        },
        arm: snap.arm,
        boot: snap.boot,
        authority: snap.authority_at(now),
        client: ClientId(connection.id()),
        deadline,
        request_id: crate::supervisor::next_admission_request_id(),
    })
}

/// Off-thread preparation for WS disconnect teardown (always succeeds without requiring a snapshot).
pub fn prepare_lease_disconnect(connection: ConnectionHandle) -> PreparedLeaseDisconnect {
    PreparedLeaseDisconnect {
        request: LeaseDisconnect { connection },
        client: ClientId(connection.id()),
    }
}

/// Single owner-side operator authorization policy driven directly by the WS dispatch table
/// (`src/api/ws/dispatch.rs`) and evaluated live on the owner (legacy lock or RT executor) with
/// zero heap allocations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OwnerAuthPolicy {
    /// No operator privilege required.
    None,
    /// Always requires a live operator session when operator auth is enforced.
    Operator(Option<&'static str>),
    /// Clear-fault rule: requires a live operator session when E-stop is latched or safety is in
    /// a non-recoverable stopped/fault state.
    ClearFault,
}

/// Zero-allocation refusal returned by [`evaluate_owner_auth_policy`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OwnerAuthError {
    EpochMismatch { expected: u64, received: u64 },
    OperatorRequired { operation: &'static str },
}

/// Single authoritative owner-side operator authorization evaluator shared by `dispatch.rs`,
/// legacy WS handlers, and RT executor maintenance/preflight paths (zero heap allocations).
pub fn evaluate_owner_auth_policy(
    policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    inner: &mut SupervisorInner,
    epoch: u64,
    now: Instant,
    path_label: &'static str,
) -> Result<(), OwnerAuthError> {
    match policy {
        OwnerAuthPolicy::None => Ok(()),
        OwnerAuthPolicy::Operator(label) => {
            let check_now = now.max(Instant::now());
            if operator.is_valid_at(check_now, inner.operator_auth_enforced) {
                Ok(())
            } else {
                Err(OwnerAuthError::OperatorRequired {
                    operation: label.unwrap_or(path_label),
                })
            }
        }
        OwnerAuthPolicy::ClearFault => {
            if inner.consume_pending_estop() && inner.safety.epoch_id() != epoch {
                return Err(OwnerAuthError::EpochMismatch {
                    expected: inner.safety.epoch_id(),
                    received: epoch,
                });
            }
            let check_now = now.max(Instant::now());
            if operator.is_valid_at(check_now, inner.operator_auth_enforced) {
                return Ok(());
            }
            if inner.consume_pending_estop() && inner.safety.epoch_id() != epoch {
                return Err(OwnerAuthError::EpochMismatch {
                    expected: inner.safety.epoch_id(),
                    received: epoch,
                });
            }
            if inner.e_stop_latched.load(Ordering::SeqCst) {
                return Err(OwnerAuthError::OperatorRequired {
                    operation: "clear_fault (EStop latched)",
                });
            }
            if (inner.safety.state().is_stopped() || inner.safety.has_latched_fault())
                && !inner.safety.recoverable()
            {
                return Err(OwnerAuthError::OperatorRequired {
                    operation: "clear_fault (non-recoverable fault or EStop)",
                });
            }
            Ok(())
        }
    }
}

/// Zero-allocation refusal returned by [`check_torque_owner_preflight`] on both legacy and RT owners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TorquePreflightError {
    LeaseLost { epoch_id: u64 },
    EpochMismatch { epoch_id: u64 },
    InvalidRequestId,
    OperatorRequired { operation: &'static str, epoch_id: u64 },
    MotionBusy { epoch_id: u64 },
    InvalidBody { message: &'static str, epoch_id: u64 },
}

/// Unified off-lock typed preparation for WS `/api/v1/torque` and `/api/v1/arm`, shared by both
/// legacy and RT executor owners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedTorqueWs {
    pub connection: ConnectionHandle,
    pub local_generation: u64,
    pub supplied_epoch: Option<u64>,
    pub request_id_valid: bool,
    pub parsed_request_id: u64,
    pub enable: Option<bool>,
    pub body_error: Option<&'static str>,
    pub auth_policy: OwnerAuthPolicy,
    pub operator: OperatorGrant,
    pub stamp: crate::supervisor::AdmissionStamp,
    pub broker: Option<PreparedTorque>,
}

/// Prepares one typed WS torque command off-lock from a published snapshot for both legacy and
/// executor owners.
#[allow(clippy::too_many_arguments)]
pub fn prepare_torque_ws(
    snap: &OwnerStateSnapshot,
    connection: ConnectionHandle,
    local_generation: u64,
    supplied_epoch: Option<u64>,
    request_id_valid: bool,
    parsed_request_id: u64,
    enable: Result<bool, &'static str>,
    auth_policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    stamp: crate::supervisor::AdmissionStamp,
    now: Instant,
    deadline: Instant,
) -> Result<PreparedTorqueWs, LeasePrepareError> {
    let (enable_opt, body_error) = match enable {
        Ok(v) => (Some(v), None),
        Err(msg) => (None, Some(msg)),
    };
    let broker = match (request_id_valid, enable_opt) {
        (true, Some(en)) => Some(prepare_torque(
            snap,
            connection,
            local_generation,
            supplied_epoch,
            en,
            parsed_request_id,
            stamp,
            now,
            deadline,
        )?),
        _ => None,
    };
    Ok(PreparedTorqueWs {
        connection,
        local_generation,
        supplied_epoch,
        request_id_valid,
        parsed_request_id,
        enable: enable_opt,
        body_error,
        auth_policy,
        operator,
        stamp,
        broker,
    })
}

/// Single live owner-side preflight for WS `/api/v1/torque` and `/api/v1/arm` (zero heap
/// allocations), shared by legacy owner installation and RT executor malformed/auth preflight so
/// live authority (`LiveConnections`, active lease, pending E-stop, epoch, operator policy,
/// `motor_zero_busy`, body validation) always yields identical error precedence across both owners.
#[allow(clippy::too_many_arguments)]
pub fn check_torque_owner_preflight(
    inner: &mut SupervisorInner,
    connection: ConnectionHandle,
    lease_generation: u64,
    supplied_epoch: Option<u64>,
    request_id_valid: bool,
    body_error: Option<&'static str>,
    auth_policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    now: Instant,
) -> Result<u64, TorquePreflightError> {
    let lease_ok = lease_generation != 0
        && inner
            .check_client_connection_live(connection.id())
            .is_ok()
        && inner.validate_lease_at(connection.as_str(), lease_generation, now);
    if !lease_ok {
        return Err(TorquePreflightError::LeaseLost {
            epoch_id: inner.safety.epoch_id(),
        });
    }
    inner.consume_pending_estop();
    let epoch = inner.safety.epoch_id();
    if supplied_epoch != Some(epoch) {
        return Err(TorquePreflightError::EpochMismatch { epoch_id: epoch });
    }
    if !request_id_valid {
        return Err(TorquePreflightError::InvalidRequestId);
    }
    if let Err(auth_err) =
        evaluate_owner_auth_policy(auth_policy, operator, inner, epoch, now, "/api/v1/torque")
    {
        return Err(match auth_err {
            OwnerAuthError::EpochMismatch { expected, .. } => {
                TorquePreflightError::EpochMismatch { epoch_id: expected }
            }
            OwnerAuthError::OperatorRequired { operation } => {
                TorquePreflightError::OperatorRequired {
                    operation,
                    epoch_id: inner.safety.epoch_id(),
                }
            }
        });
    }
    if inner.calibration_busy && inner.motor_zero_busy {
        return Err(TorquePreflightError::MotionBusy { epoch_id: epoch });
    }
    if let Some(message) = body_error {
        return Err(TorquePreflightError::InvalidBody {
            message,
            epoch_id: epoch,
        });
    }
    Ok(epoch)
}

/// Prepared torque transition (`TorqueOn` or `TorqueOff`) built off-thread from a published
/// [`OwnerStateSnapshot`], carrying the admission-captured [`crate::supervisor::AdmissionStamp`]
/// so `TorqueOn` consumes A1's `Admitted<'g>` by value and `TorqueOff` keeps its stop-direction exemption.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PreparedTorque {
    pub enable: bool,
    pub connection: ConnectionHandle,
    pub local_generation: u64,
    pub supplied_epoch: Option<u64>,
    pub arm: ArmId,
    pub boot: BootId,
    pub authority: Authority,
    pub client: ClientId,
    pub client_seq: u64,
    pub deadline: Instant,
    pub stamp: crate::supervisor::AdmissionStamp,
}

/// Off-thread preparation for WS `/api/v1/torque` or `/api/v1/arm` from a published snapshot.
#[allow(clippy::too_many_arguments)]
pub fn prepare_torque(
    snap: &OwnerStateSnapshot,
    connection: ConnectionHandle,
    local_generation: u64,
    supplied_epoch: Option<u64>,
    enable: bool,
    client_seq: u64,
    stamp: crate::supervisor::AdmissionStamp,
    now: Instant,
    deadline: Instant,
) -> Result<PreparedTorque, LeasePrepareError> {
    if snap.boot.is_zero() {
        return Err(LeasePrepareError::SnapshotUnavailable);
    }
    let mut authority = snap.authority_at(now);
    authority.lease_generation = local_generation;
    authority.epoch = supplied_epoch.unwrap_or(u64::MAX);
    // Preserve the admission-captured stop_generation for TorqueOn so an intervening urgent Stop
    // refuses the transition with `STOPPED` (TorqueOff is stop-direction exempt at owner commit).
    if enable {
        authority.stop_generation = stamp.stop_gen;
    }
    Ok(PreparedTorque {
        enable,
        connection,
        local_generation,
        supplied_epoch,
        arm: snap.arm,
        boot: snap.boot,
        authority,
        client: ClientId(connection.id()),
        client_seq,
        deadline,
        stamp,
    })
}

/// Single owner-side RT install function for `TransitionOp::TorqueOn` (consumes A1's `Admitted<'_>`
/// by value; zero heap allocations).
pub fn install_torque_on_transition(
    admitted: crate::supervisor::Admitted<'_>,
) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
    SupervisorInner::begin_enable_torque_transition(admitted)
}

/// Single owner-side RT install function for `TransitionOp::TorqueOff` (exempt from `Admitted`
/// and `stop_generation`; zero heap allocations).
pub fn install_torque_off_transition(
    inner: &mut SupervisorInner,
) -> Result<Result<crate::rt_core::ops::TorqueProgress, &'static str>, &'static str> {
    inner.begin_disable_torque_transition()
}

/// Single owner-side legacy install function for `/api/v1/torque` and `/api/v1/arm` (`enable: true`).
/// Consumes A1's `Admitted<'_>` by value and checks `motor_zero_busy` parity.
pub fn install_torque_on_legacy(
    admitted: crate::supervisor::Admitted<'_>,
) -> Result<SafetyState, crate::safety::SafetyError> {
    let (inner, proof) = admitted.consume()?;
    if inner.calibration_busy && inner.motor_zero_busy {
        return Err(crate::safety::SafetyError::MotionBusy);
    }
    inner.enable_torque_inner(&proof)
}

/// Single owner-side legacy install function for `/api/v1/torque` and `/api/v1/arm` (`enable: false`).
/// Exempt from `Admitted` (stop-direction action) while checking `motor_zero_busy` parity.
pub fn install_torque_off_legacy(
    inner: &mut SupervisorInner,
) -> Result<SafetyState, crate::safety::SafetyError> {
    if inner.calibration_busy && inner.motor_zero_busy {
        return Err(crate::safety::SafetyError::MotionBusy);
    }
    inner.disable_torque()
}

/// Unified outcome of a WS `/api/v1/torque` or `/api/v1/arm` command across legacy and RT executor owners.
#[derive(Debug)]
pub enum TorqueWsOutcome {
    Accepted {
        epoch_id: u64,
        safety_state: SafetyState,
        is_torqued: bool,
        control_mode_gravity_comp: bool,
    },
    LeaseLost {
        epoch_id: u64,
    },
    EpochMismatch {
        epoch_id: u64,
    },
    InvalidRequestId,
    InvalidBody {
        message: &'static str,
        epoch_id: u64,
    },
    Rejected {
        error: crate::api::rest::ApiError,
        epoch_id: u64,
    },
}

impl TorqueWsOutcome {
    /// True when the command passed lease, epoch, and request_id validation and therefore
    /// consumes `last_ordinary_request` on the WS connection.
    #[inline]
    pub fn consumes_request_id(&self) -> bool {
        !matches!(
            self,
            Self::LeaseLost { .. } | Self::EpochMismatch { .. } | Self::InvalidRequestId
        )
    }
}

/// Maps a static RT torque refusal or transition-terminal code into a wire-compatible `ApiError`.
pub fn map_torque_code_to_api_error(
    code: &'static str,
    safety_state_hint: Option<&SafetyState>,
) -> crate::api::rest::ApiError {
    use crate::api::rest::ApiError;
    use crate::safety::SafetyError;
    use crate::supervisor::ArmError;
    use axum::http::StatusCode;
    match code {
        "ARM_NOT_AT_RESTING_POSE" => ApiError {
            status: StatusCode::CONFLICT,
            error: "ArmNotAtRestingPose".to_string(),
            message: "Arm is not at safe resting pose".to_string(),
            code: Some("ARM_NOT_AT_RESTING_POSE".to_string()),
            safety_state: Some("Armed".to_string()),
            details: None,
            workspace_violation: None,
            current_revision: None,
            retry_after_s: None,
        },
        "MOTION_FORBIDDEN_IN_FAULT_STATE" => ApiError {
            status: StatusCode::CONFLICT,
            error: "MotionForbiddenInFaultState".to_string(),
            message: "Cannot disable torque in EStop or Fault. Call clear_fault first.".to_string(),
            code: Some("MOTION_FORBIDDEN_IN_FAULT_STATE".to_string()),
            safety_state: Some("Fault".to_string()),
            details: None,
            workspace_violation: None,
            current_revision: None,
            retry_after_s: None,
        },
        "ESTOP_ACTIVE" => ArmError::Safety(SafetyError::EStopActive).into(),
        "FAULT_ACTIVE" => {
            let message = match safety_state_hint {
                Some(SafetyState::Fault(r)) => SafetyError::FaultActive(r.clone()).to_string(),
                _ => "Fault is active".to_string(),
            };
            ApiError {
                status: StatusCode::CONFLICT,
                error: "FAULT_ACTIVE".to_string(),
                message,
                code: Some("FAULT_ACTIVE".to_string()),
                safety_state: Some("Fault".to_string()),
                details: None,
                workspace_violation: None,
                current_revision: None,
                retry_after_s: None,
            }
        }
        "HANDOVER_SETTLING" => ArmError::Safety(SafetyError::HandoverSettling).into(),
        "MOTION_BUSY" => ArmError::Safety(SafetyError::MotionBusy).into(),
        "STOPPED" => ArmError::Safety(SafetyError::Stopped).into(),
        "LEASE_LOST" | "NO_LEASE" | "DISCONNECTED" => ArmError::Safety(SafetyError::LeaseLost).into(),
        "EPOCH_MISMATCH" => ApiError(
            StatusCode::CONFLICT,
            "Command epoch is no longer current".to_string(),
            "EPOCH_MISMATCH".to_string(),
        ),
        "FEEDBACK_UNAVAILABLE" => ArmError::Safety(SafetyError::FeedbackUnavailable).into(),
        "TRANSITION_BUSY" => ApiError(
            StatusCode::CONFLICT,
            "A torque transition is pending".to_string(),
            "TRANSITION_BUSY".to_string(),
        ),
        "DISABLE_PENDING" => ApiError(
            StatusCode::CONFLICT,
            "Torque disable is still pending".to_string(),
            "DISABLE_PENDING".to_string(),
        ),
        "FEEDBACK_UNCONFIRMED" => ApiError(
            StatusCode::CONFLICT,
            "Torque transition was not confirmed by fresh hardware feedback".to_string(),
            "FEEDBACK_UNCONFIRMED".to_string(),
        ),
        "ENABLE_FAILED" => ApiError(
            StatusCode::CONFLICT,
            "Hardware torque enable failed".to_string(),
            "ENABLE_FAILED".to_string(),
        ),
        "DRIVER_WRITE_FAILED" => ApiError(
            StatusCode::CONFLICT,
            "Torque disable transmission failed".to_string(),
            "DRIVER_WRITE_FAILED".to_string(),
        ),
        "UNSUPPORTED" => {
            ArmError::Unsupported("Torque operation is not supported in current mode".into()).into()
        }
        "STAMP_REUSED" => ArmError::Safety(SafetyError::StampAlreadyConsumed).into(),
        "OWNER_TRANSFERRED" => ArmError::Safety(SafetyError::OwnerTransferred).into(),
        other => ApiError(
            StatusCode::CONFLICT,
            format!("Torque command refused: {other}"),
            other.to_string(),
        ),
    }
}

/// Bounded zero-allocation outcome of owner-routed WS `/api/v1/safety/clear_fault`
/// (`MaintenanceOp::LeaseClearFault` on RT, or legacy short write lock).
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // inline ClearFaultError keeps RT clear_fault zero-alloc
pub enum LeaseClearFaultOutcome {
    Cleared {
        safety_state: SafetyState,
        epoch_id: u64,
        is_torqued: bool,
        control_mode_gravity_comp: bool,
    },
    LeaseLost {
        epoch_id: u64,
    },
    EpochMismatch {
        expected: u64,
        received: u64,
        consumed_request_id: bool,
    },
    InvalidRequestId,
    OperatorRequired {
        operation: &'static str,
        epoch_id: u64,
    },
    MotionBusy {
        epoch_id: u64,
    },
    InvalidBody {
        message: &'static str,
        epoch_id: u64,
    },
    ClearFailed {
        err: crate::safety::ClearFaultError,
        epoch_id: u64,
    },
}

impl LeaseClearFaultOutcome {
    /// True iff lease, initial epoch, and `request_id` checks passed so the WS worker must
    /// advance `last_ordinary_request`.
    #[inline]
    pub fn consumes_request_id(&self) -> bool {
        match self {
            Self::LeaseLost { .. } | Self::InvalidRequestId => false,
            Self::EpochMismatch {
                consumed_request_id,
                ..
            } => *consumed_request_id,
            Self::Cleared { .. }
            | Self::OperatorRequired { .. }
            | Self::MotionBusy { .. }
            | Self::InvalidBody { .. }
            | Self::ClearFailed { .. } => true,
        }
    }
}

/// Live owner-side authority bundle carried through `/api/v1/safety/clear_fault` into the
/// driver preflight, hardware reset (`drv.clear_fault*`), and final `safety.clear_with_check`
/// commit inside the same owner transaction (M3).
#[derive(Debug, Clone, Copy)]
pub struct ClearFaultAuthority {
    pub connection: ConnectionHandle,
    pub lease_generation: u64,
    pub expected_epoch: u64,
    pub auth_policy: OwnerAuthPolicy,
    pub operator: OperatorGrant,
}

/// Zero-allocation refusal returned when live clear_fault authority expires or is revoked before
/// hardware reset or before final safety clear (M3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClearFaultAuthorityError {
    LeaseLost,
    PendingEstop,
    EpochMismatch { expected: u64, received: u64 },
    OperatorRequired { operation: &'static str },
}

/// Single read-only, lock-free, zero-allocation live authority re-check for `/api/v1/safety/clear_fault`
/// (M3): verifies connection liveness, active unfenced lease, epoch consistency, live operator
/// session (`OperatorGrant`), AND forbids clearing a non-recoverable fault/E-stop unless the
/// caller's policy is an explicit operator clear backed by a live valid operator session.
///
/// Safe to invoke while holding the driver mutex right before `drv.clear_fault*` and right before
/// `safety.clear_with_check` (never locks `self.driver`).
pub(crate) fn recheck_clear_fault_authority_readonly(
    inner: &SupervisorInner,
    auth: ClearFaultAuthority,
    now: Instant,
) -> Result<(), ClearFaultAuthorityError> {
    let check_now = now.max(Instant::now());
    let lease_ok = auth.lease_generation != 0
        && inner.check_client_connection_live(auth.connection.id()).is_ok()
        && inner.is_lease_valid_for_conn_at(
            auth.connection.as_str(),
            auth.lease_generation,
            check_now,
        );
    if !lease_ok {
        return Err(ClearFaultAuthorityError::LeaseLost);
    }
    if inner.estop_request_unapplied() {
        return Err(ClearFaultAuthorityError::PendingEstop);
    }
    let epoch = inner.safety.epoch_id();
    if epoch != auth.expected_epoch {
        return Err(ClearFaultAuthorityError::EpochMismatch {
            expected: epoch,
            received: auth.expected_epoch,
        });
    }
    // Never clear a non-recoverable fault (or E-stop) through this path unless it is an explicit
    // operator clear backed by a live valid operator session.
    let non_recoverable = inner.e_stop_latched.load(Ordering::SeqCst)
        || ((inner.safety.state().is_stopped() || inner.safety.has_latched_fault())
            && !inner.safety.recoverable());
    if non_recoverable {
        let explicit_operator_clear = matches!(
            auth.auth_policy,
            OwnerAuthPolicy::ClearFault | OwnerAuthPolicy::Operator(_)
        );
        if !explicit_operator_clear
            || !auth
                .operator
                .is_valid_at(check_now, inner.operator_auth_enforced)
        {
            return Err(ClearFaultAuthorityError::OperatorRequired {
                operation: if inner.e_stop_latched.load(Ordering::SeqCst) {
                    "clear_fault (EStop latched)"
                } else {
                    "clear_fault (non-recoverable fault or EStop)"
                },
            });
        }
    } else if let OwnerAuthPolicy::Operator(label) = auth.auth_policy {
        if !auth
            .operator
            .is_valid_at(check_now, inner.operator_auth_enforced)
        {
            return Err(ClearFaultAuthorityError::OperatorRequired {
                operation: label.unwrap_or("/api/v1/safety/clear_fault"),
            });
        }
    }
    Ok(())
}

fn map_clear_fault_authority_err(
    inner: &mut SupervisorInner,
    auth: ClearFaultAuthority,
    err: ClearFaultAuthorityError,
) -> LeaseClearFaultOutcome {
    match err {
        ClearFaultAuthorityError::LeaseLost => LeaseClearFaultOutcome::LeaseLost {
            epoch_id: inner.safety.epoch_id(),
        },
        ClearFaultAuthorityError::PendingEstop => {
            inner.consume_pending_estop();
            let epoch = inner.safety.epoch_id();
            if epoch != auth.expected_epoch {
                LeaseClearFaultOutcome::EpochMismatch {
                    expected: epoch,
                    received: auth.expected_epoch,
                    consumed_request_id: true,
                }
            } else {
                LeaseClearFaultOutcome::ClearFailed {
                    err: crate::safety::ClearFaultError::EStopActive,
                    epoch_id: epoch,
                }
            }
        }
        ClearFaultAuthorityError::EpochMismatch { expected, received } => {
            LeaseClearFaultOutcome::EpochMismatch {
                expected,
                received,
                consumed_request_id: true,
            }
        }
        ClearFaultAuthorityError::OperatorRequired { operation } => {
            LeaseClearFaultOutcome::OperatorRequired {
                operation,
                epoch_id: inner.safety.epoch_id(),
            }
        }
    }
}

/// Shared zero-allocation owner-side preflight for `/api/v1/safety/clear_fault` enforcing the exact
/// legacy WS precedence:
/// 1. Connection liveness & active lease (`LEASE_LOST`)
/// 2. `consume_pending_estop()` + `epoch_id` match (`EPOCH_MISMATCH`, pre-`request_id`)
/// 3. `request_id` strict monotonicity (`InvalidRequestId`)
/// 4. Pending E-stop + live [`OperatorGrant`] + non-recoverable guard (`OPERATOR_REQUIRED` / post-auth `EPOCH_MISMATCH`)
/// 5. `calibration_busy && motor_zero_busy` (`MOTION_BUSY`)
/// 6. Command body & `mit_recovery` validation (`INVALID_COMMAND`)
#[allow(clippy::too_many_arguments, clippy::result_large_err)] // inline outcome keeps RT clear_fault zero-alloc
fn check_lease_clear_fault_owner_preflight(
    inner: &mut SupervisorInner,
    connection: ConnectionHandle,
    lease_generation: u64,
    supplied_epoch: Option<u64>,
    request_id_valid: bool,
    body_error: Option<&'static str>,
    auth_policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    now: Instant,
) -> Result<ClearFaultAuthority, LeaseClearFaultOutcome> {
    let lease_ok = lease_generation != 0
        && inner
            .check_client_connection_live(connection.id())
            .is_ok()
        && inner.validate_lease_at(connection.as_str(), lease_generation, now);
    if !lease_ok {
        return Err(LeaseClearFaultOutcome::LeaseLost {
            epoch_id: inner.safety.epoch_id(),
        });
    }
    inner.consume_pending_estop();
    let epoch = inner.safety.epoch_id();
    if supplied_epoch != Some(epoch) {
        return Err(LeaseClearFaultOutcome::EpochMismatch {
            expected: epoch,
            received: supplied_epoch.unwrap_or(0),
            consumed_request_id: false,
        });
    }
    if !request_id_valid {
        return Err(LeaseClearFaultOutcome::InvalidRequestId);
    }
    let auth = ClearFaultAuthority {
        connection,
        lease_generation,
        expected_epoch: epoch,
        auth_policy,
        operator,
    };
    if let Err(auth_err) = recheck_clear_fault_authority_readonly(inner, auth, now) {
        return Err(map_clear_fault_authority_err(inner, auth, auth_err));
    }
    if let Err(auth_err) = evaluate_owner_auth_policy(
        auth_policy,
        operator,
        inner,
        epoch,
        now,
        "/api/v1/safety/clear_fault",
    ) {
        return Err(match auth_err {
            OwnerAuthError::EpochMismatch { expected, received } => {
                LeaseClearFaultOutcome::EpochMismatch {
                    expected,
                    received,
                    consumed_request_id: true,
                }
            }
            OwnerAuthError::OperatorRequired { operation } => {
                LeaseClearFaultOutcome::OperatorRequired {
                    operation,
                    epoch_id: inner.safety.epoch_id(),
                }
            }
        });
    }
    if inner.calibration_busy && inner.motor_zero_busy {
        return Err(LeaseClearFaultOutcome::MotionBusy {
            epoch_id: inner.safety.epoch_id(),
        });
    }
    if let Some(message) = body_error {
        return Err(LeaseClearFaultOutcome::InvalidBody {
            message,
            epoch_id: inner.safety.epoch_id(),
        });
    }
    Ok(auth)
}

/// Single RT owner-side install function for `MaintenanceOp::LeaseClearFault` (zero heap allocs).
#[allow(clippy::too_many_arguments)]
pub fn install_lease_clear_fault_rt(
    inner: &mut SupervisorInner,
    connection: ConnectionHandle,
    lease_generation: u64,
    supplied_epoch: Option<u64>,
    request_id_valid: bool,
    body_error: Option<&'static str>,
    auth_policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    now: Instant,
    release_deadline: Instant,
) -> LeaseClearFaultOutcome {
    let mut live_now = || now.max(Instant::now());
    install_lease_clear_fault_rt_with_clock(
        inner,
        connection,
        lease_generation,
        supplied_epoch,
        request_id_valid,
        body_error,
        auth_policy,
        operator,
        now,
        release_deadline,
        &mut live_now,
    )
}

/// Like [`install_lease_clear_fault_rt`], threading the owner's live clock (`now_fn`) into
/// under-guard authority rechecks and TX deadline translation so wait time or driver-clock
/// sampling can never extend TX beyond `release_deadline`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn install_lease_clear_fault_rt_with_clock(
    inner: &mut SupervisorInner,
    connection: ConnectionHandle,
    lease_generation: u64,
    supplied_epoch: Option<u64>,
    request_id_valid: bool,
    body_error: Option<&'static str>,
    auth_policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    now: Instant,
    release_deadline: Instant,
    now_fn: &mut dyn FnMut() -> Instant,
) -> LeaseClearFaultOutcome {
    let auth = match check_lease_clear_fault_owner_preflight(
        inner,
        connection,
        lease_generation,
        supplied_epoch,
        request_id_valid,
        body_error,
        auth_policy,
        operator,
        now,
    ) {
        Ok(a) => a,
        Err(out) => return out,
    };
    match inner.clear_fault_executor_bounded_for_release_with_authority_clock(
        now,
        release_deadline,
        auth,
        now_fn,
    ) {
        Ok(safety_state) => LeaseClearFaultOutcome::Cleared {
            safety_state,
            epoch_id: inner.safety.epoch_id(),
            is_torqued: inner.is_torqued,
            control_mode_gravity_comp: inner.gravity_comp_active(),
        },
        Err(crate::supervisor::ClearFaultWithAuthError::Authority(auth_err)) => {
            map_clear_fault_authority_err(inner, auth, auth_err)
        }
        Err(crate::supervisor::ClearFaultWithAuthError::Clear(
            crate::safety::ClearFaultError::EpochMismatch { expected, received },
        )) => LeaseClearFaultOutcome::EpochMismatch {
            expected,
            received,
            consumed_request_id: true,
        },
        Err(crate::supervisor::ClearFaultWithAuthError::Clear(err)) => {
            LeaseClearFaultOutcome::ClearFailed {
                err,
                epoch_id: inner.safety.epoch_id(),
            }
        }
    }
}

/// Single legacy owner-side install function for WS `/api/v1/safety/clear_fault`, sharing the
/// exact same authority/operator preflight and mid-transaction reset/clear re-checks as
/// [`install_lease_clear_fault_rt`].
#[allow(clippy::too_many_arguments)]
pub fn install_lease_clear_fault_legacy(
    inner: &mut SupervisorInner,
    connection: ConnectionHandle,
    lease_generation: u64,
    supplied_epoch: Option<u64>,
    request_id_valid: bool,
    body_error: Option<&'static str>,
    mit_recovery: bool,
    auth_policy: OwnerAuthPolicy,
    operator: OperatorGrant,
    now: Instant,
) -> Result<LeaseClearFaultOutcome, (crate::supervisor::ArmError, u64)> {
    let auth = match check_lease_clear_fault_owner_preflight(
        inner,
        connection,
        lease_generation,
        supplied_epoch,
        request_id_valid,
        body_error,
        auth_policy,
        operator,
        now,
    ) {
        Ok(a) => a,
        Err(out) => return Ok(out),
    };
    match inner.clear_fault_confirmed_for_epoch_with_authority(mit_recovery, auth) {
        Ok(safety_state) => Ok(LeaseClearFaultOutcome::Cleared {
            safety_state,
            epoch_id: inner.safety.epoch_id(),
            is_torqued: inner.is_torqued,
            control_mode_gravity_comp: inner.gravity_comp_active(),
        }),
        Err(crate::supervisor::ClearFaultLegacyWithAuthError::Authority(auth_err)) => {
            Ok(map_clear_fault_authority_err(inner, auth, auth_err))
        }
        Err(crate::supervisor::ClearFaultLegacyWithAuthError::Arm(err)) => {
            Err((err, inner.safety.epoch_id()))
        }
    }
}

/// Off-thread preparation result for one WS `heartbeat` frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreparedHeartbeat {
    /// Local identity/generation/label checks passed; submit to owner (`ImmediateOp::Heartbeat`).
    Submit {
        connection: ConnectionHandle,
        local_generation: u64,
        supplied_epoch: Option<u64>,
        arm: ArmId,
        boot: BootId,
        authority: Authority,
        client: ClientId,
        deadline: Instant,
    },
    /// Rejected before owner submission (zero local generation, wrong supplied generation, or wrong client label).
    LeaseLost {
        generation: u64,
        reason: Option<&'static str>,
    },
}

/// Single off-thread heartbeat preparation rule shared by legacy and executor WS handlers.
///
/// Preserves the connection-bound omission rule: legacy clients may omit `generation` (`0`),
/// while any non-zero `generation` must match `local_generation` and any supplied `client_id`
/// must match the connection's granted label.
#[allow(clippy::too_many_arguments)]
pub fn prepare_heartbeat(
    snap: &OwnerStateSnapshot,
    connection: ConnectionHandle,
    local_generation: u64,
    supplied_generation: u64,
    local_label: Option<&str>,
    supplied_client_id: Option<Option<&str>>,
    supplied_epoch: Option<u64>,
    now: Instant,
    deadline: Instant,
) -> PreparedHeartbeat {
    if local_generation == 0 {
        return PreparedHeartbeat::LeaseLost {
            generation: 0,
            reason: None,
        };
    }
    let label_matches = supplied_client_id.is_none_or(|id| id == local_label);
    let supplied_generation_ok = supplied_generation == local_generation || supplied_generation == 0;
    if !supplied_generation_ok || !label_matches {
        return PreparedHeartbeat::LeaseLost {
            generation: local_generation,
            reason: Some(snap.lease_loss_reason(connection.as_str(), local_generation)),
        };
    }
    let mut authority = snap.authority_at(now);
    authority.lease_generation = local_generation;
    // Sentinel `u64::MAX` when client omitted `epoch_id` (real epochs start at 1 and never reach MAX),
    // so owner commit reliably detects epoch mismatch and emits `epoch_sync`.
    authority.epoch = supplied_epoch.unwrap_or(u64::MAX);
    PreparedHeartbeat::Submit {
        connection,
        local_generation,
        supplied_epoch,
        arm: snap.arm,
        boot: snap.boot,
        authority,
        client: ClientId(connection.id()),
        deadline,
    }
}

/// Context passed into the single owner-side lease install functions.
#[derive(Debug, Clone, Copy)]
pub struct LeaseInstallCtx {
    /// Expected owner arm identity (`ctx.arm` in RT, `inner.core.id()` in legacy).
    pub expected_arm: ArmId,
    /// Expected owner boot identity (`ctx.boot` in RT, `inner.boot_id` in legacy).
    pub expected_boot: BootId,
    /// Arm identity stamped on the request at preparation/submission.
    pub stamp_arm: ArmId,
    /// Boot identity stamped on the request at preparation/submission.
    pub stamp_boot: BootId,
    pub now: Instant,
    pub request_id: u64,
    pub client: ClientId,
    pub deadline: Instant,
    /// When true, consumes `request_id` in A1's single-use [`ConsumedStamps`] tracker.
    pub enforce_single_use: bool,
}

/// Single owner-side authority check shared by `install_lease_acquire`, `install_lease_preempt`,
/// and `install_lease_release`.
fn check_lease_install_authority(
    inner: &mut SupervisorInner,
    conn: ConnectionHandle,
    ctx: &LeaseInstallCtx,
) -> Result<(), &'static str> {
    if inner.detached {
        return Err("OWNER_TRANSFERRED");
    }
    if ctx.stamp_arm != ctx.expected_arm
        || ctx.stamp_boot != ctx.expected_boot
        || ctx.stamp_boot.is_zero()
        || ctx.expected_boot.is_zero()
    {
        return Err("WRONG_TARGET");
    }
    if ctx.now > ctx.deadline {
        return Err("DEADLINE_EXPIRED");
    }
    if conn.id() != 0 && ctx.client.0 != 0 && conn.id() != ctx.client.0 {
        return Err("DISCONNECTED");
    }
    let effective_id = if conn.id() != 0 {
        conn.id()
    } else {
        ctx.client.0
    };
    inner.check_client_connection_live(effective_id)?;
    if ctx.enforce_single_use {
        inner.consume_single_use_request_id(ctx.request_id)?;
    }
    Ok(())
}

/// Single owner-side install function for `LeaseAcquire` (called by both legacy and RT executor).
pub fn install_lease_acquire(
    inner: &mut SupervisorInner,
    req: &LeaseAcquire,
    ctx: &LeaseInstallCtx,
) -> Result<LeaseOutcome, &'static str> {
    check_lease_install_authority(inner, req.connection, ctx)?;
    let conn = req.connection.as_str();
    let label = if req.has_label { req.label.as_str() } else { "" };

    let fenced_gen = inner.effective_fenced_lease_generation();
    let active_conn_closed = inner.active_lease_conn_id != 0
        && inner.is_connection_closed(inner.active_lease_conn_id);
    if let Some(ref mut active) = inner.active_lease {
        let is_expired = active_conn_closed
            || active.generation <= fenced_gen
            || ctx.now.saturating_duration_since(active.acquired_at) >= inner.lease_timeout;
        if !is_expired {
            if active.client_id == conn {
                if req.has_label {
                    active.label.clear();
                    active.label.push_str(label);
                }
                let gen = active.generation;
                inner.authorize_servo_stream_for_committed_lease(gen);
                return Ok(LeaseOutcome::Granted {
                    generation: gen,
                    evicted_connection: None,
                    evicted_generation: 0,
                });
            }
            return Ok(LeaseOutcome::Busy {
                holder_generation: active.generation,
            });
        }
    }

    inner.cancel_watchdog_park_for_new_lease(conn);
    #[cfg(feature = "policy-runtime")]
    if inner.policy_run.is_some() {
        inner.terminate_policy_run_static(
            "LEASE_LOST",
            "cancelled",
            Some("LEASE_LOST"),
            Some("Control lease acquired by new controller"),
        );
    }
    if let Some(old_gen) = inner.active_lease.as_ref().map(|a| a.generation) {
        inner.fence_displaced_lease_generation(old_gen, true);
    }
    inner.lease_generation_counter += 1;
    let gen = inner.lease_generation_counter;
    inner.set_active_lease_in_place(req.connection.id(), conn, label, gen, ctx.now);
    inner.authorize_servo_stream_for_committed_lease(gen);
    Ok(LeaseOutcome::Granted {
        generation: gen,
        evicted_connection: None,
        evicted_generation: 0,
    })
}

/// Single owner-side install function for `LeasePreempt` (called by both legacy and RT executor).
///
/// Re-verifies `req.operator` live at commit (0 locks, 0 allocs), enforces preemption cooldown,
/// fences the displaced generation BEFORE issuing the new generation, and recycles lease/evicted
/// strings in place (zero RT allocations/frees).
pub fn install_lease_preempt(
    inner: &mut SupervisorInner,
    req: &LeasePreempt,
    ctx: &LeaseInstallCtx,
) -> Result<LeaseOutcome, &'static str> {
    check_lease_install_authority(inner, req.connection, ctx)?;
    if !req.operator.is_valid_at(ctx.now, inner.operator_auth_enforced) {
        return Ok(LeaseOutcome::OperatorRequired);
    }
    let conn = req.connection.as_str();
    let label = req.label.as_str();

    let fenced_gen = inner.effective_fenced_lease_generation();
    let active_conn_closed = inner.active_lease_conn_id != 0
        && inner.is_connection_closed(inner.active_lease_conn_id);
    if let Some(ref active) = inner.active_lease {
        let is_expired = active_conn_closed
            || active.generation <= fenced_gen
            || ctx.now.saturating_duration_since(active.acquired_at) >= inner.lease_timeout;
        if active.client_id == conn && !is_expired {
            let gen = active.generation;
            inner.authorize_servo_stream_for_committed_lease(gen);
            return Ok(LeaseOutcome::Granted {
                generation: gen,
                evicted_connection: None,
                evicted_generation: 0,
            });
        }
        if active.client_id != conn {
            if let Some(last) = inner.last_preemption {
                let elapsed = ctx.now.saturating_duration_since(last);
                if elapsed < PREEMPT_COOLDOWN {
                    let remaining_ms =
                        (PREEMPT_COOLDOWN.saturating_sub(elapsed).as_millis() as u64).max(1);
                    return Ok(LeaseOutcome::Cooldown { remaining_ms });
                }
            }
        }
    }

    // M1: Hardware-disabled readiness is owned strictly by decision state (`torque_intent == Off`),
    // never inferred from feedback-reported `enable_state()`.
    let confirmed_hardware_disabled = !inner.is_torqued
        && inner.torque_intent == crate::supervisor::TorqueIntent::Off
        && !inner.disable_pending
        && inner.safety.state() == &SafetyState::Idle;

    let evicted = inner
        .active_lease
        .as_ref()
        .filter(|a| a.client_id != conn)
        .and_then(|a| {
            ConnectionHandle::try_new(inner.active_lease_conn_id, &a.client_id)
                .map(|h| (h, a.generation))
        });

    let old_gen = inner.active_lease.as_ref().map_or(0, |a| a.generation);
    let needs_stop = !(confirmed_hardware_disabled
        || inner.safety.state().is_stopped()
        || inner.is_fully_settled_armed_hold());

    // Final commit-point live authorization check strictly AFTER all read-only preflight
    // (including driver lock inspection) and IMMEDIATELY BEFORE the first owner mutation.
    let commit_now = ctx.now.max(Instant::now());
    if !req.operator.is_valid_at(commit_now, inner.operator_auth_enforced) {
        return Ok(LeaseOutcome::OperatorRequired);
    }

    inner.cancel_watchdog_park_for_new_lease(conn);
    #[cfg(feature = "policy-runtime")]
    if inner.policy_run.is_some() {
        inner.terminate_policy_run_static(
            "LEASE_LOST",
            "cancelled",
            Some("LEASE_LOST"),
            Some("Control lease preempted"),
        );
    }

    if needs_stop {
        inner.handover_settling_since = Some(commit_now);
    } else {
        inner.handover_settling_since = None;
    }
    if old_gen != 0 {
        inner.fence_displaced_lease_generation(old_gen, needs_stop);
    } else if needs_stop {
        inner.stop_motion();
    }

    inner.lease_generation_counter += 1;
    let gen = inner.lease_generation_counter;
    inner.set_active_lease_in_place(req.connection.id(), conn, label, gen, commit_now);
    inner.authorize_servo_stream_for_committed_lease(gen);
    inner.last_heartbeat = commit_now;
    let (evicted_connection, evicted_generation) = if let Some((ev_handle, ev_gen)) = evicted {
        inner.last_preemption = Some(commit_now);
        inner.set_last_evicted_in_place(ev_handle.as_str(), ev_gen);
        (Some(ev_handle), ev_gen)
    } else {
        (None, 0)
    };
    if !inner.rt_executor_owned {
        tracing::info!(
            "Preempted lease for connection {} (label {}), generation {}, evicted: {:?}",
            conn,
            label,
            gen,
            evicted_connection
        );
    }
    Ok(LeaseOutcome::Granted {
        generation: gen,
        evicted_connection,
        evicted_generation,
    })
}

/// Single owner-side install function for `LeaseRelease` (called by both legacy and RT executor).
pub fn install_lease_release(
    inner: &mut SupervisorInner,
    req: &LeaseRelease,
    ctx: &LeaseInstallCtx,
) -> Result<LeaseOutcome, &'static str> {
    check_lease_install_authority(inner, req.connection, ctx)?;
    let conn = req.connection.as_str();
    let Some(valid) = inner
        .active_lease
        .as_ref()
        .filter(|a| a.client_id == conn && a.generation == req.generation)
        .map(|a| {
            a.generation > inner.fenced_lease_generation
                && ctx.now.saturating_duration_since(a.acquired_at) < inner.lease_timeout
        })
    else {
        return Ok(LeaseOutcome::Released {
            released: false,
            displaced_generation: 0,
        });
    };

    // Terminate any active policy run BEFORE `fence_displaced_lease_generation` -> `stop_motion()`
    // so it records `LEASE_LOST` rather than being masked as `STOPPED`.
    #[cfg(feature = "policy-runtime")]
    if inner.policy_run.is_some() {
        inner.terminate_policy_run_static(
            "LEASE_LOST",
            "cancelled",
            Some("LEASE_LOST"),
            Some("Control lease released"),
        );
    }
    // Fence and stop active motion when valid OR when this release first observes expiry while
    // no autonomous watchdog park is active (preserving any running watchdog park explicitly).
    let stop_client_motion = valid
        || (inner.fenced_lease_generation < req.generation
            && inner.watchdog_park.is_none()
            && !inner.pending_normal_unload);
    inner.fence_displaced_lease_generation(req.generation, stop_client_motion);
    let _ = inner.clear_active_lease_in_place();
    Ok(LeaseOutcome::Released {
        released: valid,
        displaced_generation: req.generation,
    })
}

/// Single owner-side install function for `LeaseDisconnect` (called identically by legacy WS
/// teardown, priority [`ConnLifecycleIntake`], and broker `LeaseRequest::Disconnect`).
///
/// Marks the connection closed and removes it from `live_connections`, fences its lease
/// generation, stops active motion if it held a valid lease (preserving any running watchdog
/// park for an already-expired lease), and releases the lease with zero allocations.
pub fn install_lease_disconnect(
    inner: &mut SupervisorInner,
    req: &LeaseDisconnect,
    now: Instant,
) -> LeaseOutcome {
    if inner.detached {
        return LeaseOutcome::Disconnected {
            released: false,
            displaced_generation: 0,
        };
    }
    if req.connection.id() != 0 {
        inner.mark_connection_closed(req.connection.id());
    }
    let conn = req.connection.as_str();
    let matches_owner = inner.active_lease.as_ref().and_then(|a| {
        let same_conn = a.client_id == conn
            || (req.connection.id() != 0 && inner.active_lease_conn_id == req.connection.id());
        if same_conn {
            let valid = a.generation > inner.fenced_lease_generation
                && now.saturating_duration_since(a.acquired_at) < inner.lease_timeout;
            Some((a.generation, valid))
        } else {
            None
        }
    });

    let Some((gen, valid)) = matches_owner else {
        return LeaseOutcome::Disconnected {
            released: false,
            displaced_generation: 0,
        };
    };

    #[cfg(feature = "policy-runtime")]
    if inner.policy_run.is_some() {
        inner.terminate_policy_run_static(
            "LEASE_LOST",
            "cancelled",
            Some("LEASE_LOST"),
            Some("Control lease released on disconnect"),
        );
    }
    let stop_client_motion = valid
        || (inner.fenced_lease_generation < gen
            && inner.watchdog_park.is_none()
            && !inner.pending_normal_unload);
    inner.fence_displaced_lease_generation(gen, stop_client_motion);
    let _ = inner.clear_active_lease_in_place();
    LeaseOutcome::Disconnected {
        released: valid,
        displaced_generation: gen,
    }
}

/// Single owner-side heartbeat renewal helper shared by broker `ImmediateOp::Heartbeat` and
/// legacy WS heartbeat. Never resurrects an expired or fenced lease and enforces connection
/// identity when the lease is bound to a WS connection.
pub fn renew_lease_heartbeat_at(
    inner: &mut SupervisorInner,
    client_id: u64,
    conn_str: Option<&str>,
    generation: u64,
    now: Instant,
) -> Result<(), &'static str> {
    if inner.detached {
        return Err("OWNER_TRANSFERRED");
    }
    inner.check_client_connection_live(client_id)?;
    if generation == 0 {
        return Err("NO_LEASE");
    }
    let fenced_gen = inner.effective_fenced_lease_generation();
    let active_conn_closed = inner.active_lease_conn_id != 0
        && inner.is_connection_closed(inner.active_lease_conn_id);
    let Some(ref mut active) = inner.active_lease else {
        return Err("LEASE_LOST");
    };
    let not_expired = !active_conn_closed
        && active.generation > fenced_gen
        && now.saturating_duration_since(active.acquired_at) < inner.lease_timeout;
    let owner_matches = conn_str.is_none_or(|s| active.client_id == s)
        && inner.active_lease_conn_id != 0
        && inner.active_lease_conn_id == client_id;
    if !not_expired || active.generation != generation || !owner_matches {
        return Err("LEASE_LOST");
    }
    active.acquired_at = now;
    inner.last_heartbeat = now;
    Ok(())
}

/// Owner-side commit entry point for broker `ImmediateOp::Heartbeat`.
///
/// Checks identity, deadline, `stop_generation` (preserving `test_rt_broker` semantics),
/// connection liveness, lease ownership/validity (BEFORE epoch so expired leases report
/// `LEASE_LOST` rather than `EPOCH_MISMATCH`), epoch, and config revision, then renews.
pub fn install_broker_heartbeat(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    ctx: &CommitCtx,
) -> Result<(), &'static str> {
    if inner.detached {
        return Err("OWNER_TRANSFERRED");
    }
    if d.stamp.arm != ctx.arm || d.stamp.boot != ctx.boot || d.stamp.boot.is_zero() {
        return Err("WRONG_TARGET");
    }
    if ctx.now > d.stamp.deadline {
        return Err("DEADLINE_EXPIRED");
    }
    if d.stamp.authority.stop_generation != ctx.stop_generation {
        return Err("STOPPED");
    }
    inner.check_client_connection_live(d.stamp.client.0)?;
    let generation = d.stamp.authority.lease_generation;
    if generation == 0 {
        return Err("NO_LEASE");
    }
    // Verify active lease ownership & freshness BEFORE epoch check (matching legacy WS precedence).
    let fenced_gen = inner.effective_fenced_lease_generation();
    let active_conn_closed = inner.active_lease_conn_id != 0
        && inner.is_connection_closed(inner.active_lease_conn_id);
    let lease_ok = !active_conn_closed
        && inner.active_lease.as_ref().is_some_and(|a| {
            a.generation == generation
                && a.generation > fenced_gen
                && ctx.now.saturating_duration_since(a.acquired_at) < inner.lease_timeout
                && inner.active_lease_conn_id != 0
                && inner.active_lease_conn_id == d.stamp.client.0
        });
    if !lease_ok {
        return Err("LEASE_LOST");
    }
    if d.stamp.authority.epoch != inner.safety.epoch_id() {
        return Err("EPOCH_MISMATCH");
    }
    if d.stamp.authority.config_revision != inner.workspace_revision {
        return Err("PLAN_STALE");
    }
    renew_lease_heartbeat_at(inner, d.stamp.client.0, None, generation, ctx.now)
}

/// Owner-side commit entry point for broker `CommandKind::Lease(req)`.
pub fn commit_lease_descriptor(
    inner: &mut SupervisorInner,
    d: &Descriptor,
    req: LeaseRequest,
    ctx: &CommitCtx,
) -> Result<LeaseOutcome, &'static str> {
    // Disconnect via broker still checks target identity/boot, never blocked by deadline or closed status.
    if let LeaseRequest::Disconnect(disc) = req {
        if d.stamp.arm != ctx.arm || d.stamp.boot != ctx.boot || d.stamp.boot.is_zero() {
            return Err("WRONG_TARGET");
        }
        if disc.connection.id() != 0 && d.stamp.client.0 != 0 && disc.connection.id() != d.stamp.client.0 {
            return Err("DISCONNECTED");
        }
        return Ok(install_lease_disconnect(inner, &disc, ctx.now));
    }
    let install_ctx = LeaseInstallCtx {
        expected_arm: ctx.arm,
        expected_boot: ctx.boot,
        stamp_arm: d.stamp.arm,
        stamp_boot: d.stamp.boot,
        now: ctx.now,
        request_id: crate::supervisor::AdmissionStamp::from_broker(
            d.stamp.authority.stop_generation,
            d.stamp.authority.epoch,
            d.stamp.authority.lease_generation,
            d.stamp.request.0,
        )
        .request_id,
        client: d.stamp.client,
        deadline: d.stamp.deadline,
        enforce_single_use: true,
    };
    match req {
        LeaseRequest::Acquire(r) => install_lease_acquire(inner, &r, &install_ctx),
        LeaseRequest::Preempt(r) => install_lease_preempt(inner, &r, &install_ctx),
        LeaseRequest::Release(r) => install_lease_release(inner, &r, &install_ctx),
        LeaseRequest::Disconnect(_) => unreachable!(),
    }
}

/// Owner-side heartbeat outcome mirroring `ws/lease.rs`.
#[derive(Debug, Clone, PartialEq)]
pub enum HeartbeatInstallOutcome {
    Renewed,
    LeaseLost {
        generation: u64,
        reason: Option<&'static str>,
    },
    EpochSync {
        epoch: u64,
        safety_state: SafetyState,
    },
}

/// Single legacy owner-side heartbeat install (called under short write lock after `prepare_heartbeat`).
pub fn install_legacy_heartbeat(
    inner: &mut SupervisorInner,
    connection: ConnectionHandle,
    local_generation: u64,
    supplied_epoch: Option<u64>,
    now: Instant,
) -> Result<HeartbeatInstallOutcome, &'static str> {
    if inner.detached {
        return Err("OWNER_TRANSFERRED");
    }
    // Verify connection is not closed and still owns the active unfenced lease at `now`.
    let lease_valid = inner
        .check_client_connection_live(connection.id())
        .is_ok()
        && inner.validate_lease_at(connection.as_str(), local_generation, now);
    if !lease_valid {
        return Ok(HeartbeatInstallOutcome::LeaseLost {
            generation: local_generation,
            reason: Some(inner.lease_loss_reason(connection.as_str(), local_generation)),
        });
    }
    let epoch = inner.safety.epoch_id();
    if supplied_epoch != Some(epoch) {
        return Ok(HeartbeatInstallOutcome::EpochSync {
            epoch,
            safety_state: inner.safety.state().clone(),
        });
    }
    renew_lease_heartbeat_at(
        inner,
        connection.id(),
        Some(connection.as_str()),
        local_generation,
        now,
    )?;
    Ok(HeartbeatInstallOutcome::Renewed)
}

// ---------------------------------------------------------------------------------------
// Priority connection lifecycle channel (Open / Disconnect, 1:1 slot-reserved, lossless)

/// Bounded priority connection lifecycle command (`ConnLifecycleClient` -> RT `ReleaseEngine`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleCmd {
    Open {
        seq: u64,
        conn_id: u64,
    },
    Disconnect {
        seq: u64,
        connection: ConnectionHandle,
    },
}

/// Bounded priority connection lifecycle reply (RT `ReleaseEngine` -> `ConnLifecycleClient`),
/// flushed strictly AFTER Step 6 `StateHub` publication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleReply {
    Opened {
        seq: u64,
        conn_id: u64,
        result: Result<(), &'static str>,
    },
    Disconnected {
        seq: u64,
        conn_id: u64,
        outcome: LeaseOutcome,
    },
    Stopped {
        seq: u64,
        conn_id: u64,
    },
}

impl LifecycleReply {
    #[inline]
    pub fn seq(&self) -> u64 {
        match self {
            Self::Opened { seq, .. }
            | Self::Disconnected { seq, .. }
            | Self::Stopped { seq, .. } => *seq,
        }
    }

    #[inline]
    pub fn conn_id(&self) -> u64 {
        match self {
            Self::Opened { conn_id, .. }
            | Self::Disconnected { conn_id, .. }
            | Self::Stopped { conn_id, .. } => *conn_id,
        }
    }
}

/// Errors from [`ConnLifecycleClient`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnLifecycleError {
    TooManyConnections,
    AlreadyClosed,
    InvalidConnection,
    Stopped,
    /// Caller invoked a synchronous lifecycle wait from inside the active `SerialDrainGate`
    /// callback thread (would deadlock waiting for itself to drain the lifecycle reply).
    ReentrantCallback,
}

/// Applies one [`LifecycleCmd`] on the single owner (0 alloc / 0 free on RT).
pub fn apply_conn_lifecycle(
    inner: &mut SupervisorInner,
    cmd: LifecycleCmd,
    now: Instant,
) -> LifecycleReply {
    match cmd {
        LifecycleCmd::Open { seq, conn_id } => LifecycleReply::Opened {
            seq,
            conn_id,
            result: inner.register_live_connection(conn_id),
        },
        LifecycleCmd::Disconnect { seq, connection } => LifecycleReply::Disconnected {
            seq,
            conn_id: connection.id(),
            outcome: install_lease_disconnect(inner, &LeaseDisconnect { connection }, now),
        },
    }
}

/// RT-side intake for priority connection lifecycle commands (owned exclusively by
/// `ReleaseEngine` on `armd-control`; zero heap allocations and zero frees).
pub struct ConnLifecycleIntake {
    req_rx: Consumer<LifecycleCmd>,
    rep_tx: Producer<LifecycleReply>,
    pending: VecDeque<LifecycleReply>,
    capacity: usize,
}

impl ConnLifecycleIntake {
    /// Drains all currently queued lifecycle commands (at most `capacity`) into the preallocated
    /// `pending` reply deque. Returns `true` if any disconnect released a valid lease (or if any
    /// command mutated owner state requiring publication).
    pub fn drain_requests<F>(&mut self, mut handle: F) -> bool
    where
        F: FnMut(LifecycleCmd) -> LifecycleReply,
    {
        let mut drained = 0usize;
        let mut mutated = false;
        while drained < self.capacity && self.pending.len() < self.capacity {
            let Ok(cmd) = self.req_rx.pop() else {
                break;
            };
            drained += 1;
            let reply = handle(cmd);
            if matches!(
                reply,
                LifecycleReply::Disconnected { .. } | LifecycleReply::Opened { result: Ok(()), .. }
            ) {
                mutated = true;
            }
            self.pending.push_back(reply);
        }
        mutated
    }

    pub fn has_pending_replies(&self) -> bool {
        !self.pending.is_empty()
    }

    /// Flushes completed lifecycle replies into `rep_tx` strictly AFTER `StateHub` publication.
    pub fn flush_replies(&mut self) -> bool {
        let mut pushed = false;
        while !self.rep_tx.is_full() {
            let Some(reply) = self.pending.pop_front() else {
                break;
            };
            let _ = self.rep_tx.push(reply);
            pushed = true;
        }
        pushed
    }

    /// Exit/panic drain on `armd-control`: flushes published replies (if `flush_existing`) and
    /// answers remaining queued requests with `Stopped`.
    pub fn drain_stopped(&mut self, flush_existing: bool) -> bool {
        let mut pushed = false;
        if flush_existing {
            pushed |= self.flush_replies();
        } else {
            while !self.rep_tx.is_full() {
                let Some(reply) = self.pending.pop_front() else {
                    break;
                };
                let _ = self.rep_tx.push(LifecycleReply::Stopped {
                    seq: reply.seq(),
                    conn_id: reply.conn_id(),
                });
                pushed = true;
            }
        }
        while !self.rep_tx.is_full() {
            let Ok(cmd) = self.req_rx.pop() else {
                break;
            };
            let (seq, conn_id) = match cmd {
                LifecycleCmd::Open { seq, conn_id } => (seq, conn_id),
                LifecycleCmd::Disconnect { seq, connection } => (seq, connection.id()),
            };
            let _ = self.rep_tx.push(LifecycleReply::Stopped { seq, conn_id });
            pushed = true;
        }
        pushed
    }
}

enum LifecycleWaiter {
    Sync(mpsc::SyncSender<Result<LifecycleReply, ConnLifecycleError>>),
    Async(tokio::sync::oneshot::Sender<Result<LifecycleReply, ConnLifecycleError>>),
}

impl LifecycleWaiter {
    fn deliver(self, r: Result<LifecycleReply, ConnLifecycleError>) {
        match self {
            Self::Sync(tx) => {
                let _ = tx.send(r);
            }
            Self::Async(tx) => {
                let _ = tx.send(r);
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SlotState {
    Open,
    Closing { seq: u64 },
}

struct ConnLifecycleClientInner {
    req_tx: Producer<LifecycleCmd>,
    rep_rx: Consumer<LifecycleReply>,
    /// Live off-thread connection slots (`len <= MAX_LIVE_CONNECTIONS`). A slot is NOT freed
    /// until its `Disconnected` (or failed `Opened`) reply returns from RT, guaranteeing the
    /// `2 * MAX_LIVE_CONNECTIONS` SPSC ring can never overflow.
    slots: HashMap<u64, SlotState>,
    next_seq: u64,
    closed: bool,
    waiters: HashMap<u64, Vec<LifecycleWaiter>>,
    /// Bounded cache of recently completed disconnect outcomes so an async teardown awaiting
    /// completion after an immediate non-blocking disconnect fence still gets the exact outcome.
    completed_disconnects: VecDeque<(u64, LeaseOutcome)>,
    exited: Arc<AtomicBool>,
}

impl ConnLifecycleClientInner {
    fn pop_all_replies(&mut self) {
        while let Ok(reply) = self.rep_rx.pop() {
            let seq = reply.seq();
            match reply {
                LifecycleReply::Opened {
                    conn_id,
                    result: Err(_),
                    ..
                }
                | LifecycleReply::Stopped { conn_id, .. } => {
                    self.slots.remove(&conn_id);
                }
                LifecycleReply::Disconnected {
                    conn_id, outcome, ..
                } => {
                    self.slots.remove(&conn_id);
                    if self.completed_disconnects.len() >= CONN_LIFECYCLE_CAPACITY {
                        self.completed_disconnects.pop_front();
                    }
                    self.completed_disconnects.push_back((conn_id, outcome));
                }
                LifecycleReply::Opened { result: Ok(()), .. } => {}
            }
            if let Some(waiters) = self.waiters.remove(&seq) {
                let res = match reply {
                    LifecycleReply::Stopped { .. } => Err(ConnLifecycleError::Stopped),
                    other => Ok(other),
                };
                for waiter in waiters {
                    waiter.deliver(res);
                }
            }
        }
    }

    fn drain_replies(&mut self) {
        self.pop_all_replies();
        if self.exited.load(Ordering::Acquire) {
            self.pop_all_replies();
            self.fail_remaining(ConnLifecycleError::Stopped);
        }
    }

    fn fail_remaining(&mut self, err: ConnLifecycleError) {
        let remaining = std::mem::take(&mut self.waiters);
        self.slots.clear();
        for (_, waiters) in remaining {
            for w in waiters {
                w.deliver(Err(err));
            }
        }
    }
}

/// Off-thread priority connection lifecycle client (never touched by `armd-control`).
pub struct ConnLifecycleClient {
    inner: Mutex<ConnLifecycleClientInner>,
}

/// Creates a matched priority connection lifecycle client and RT intake with capacity
/// [`CONN_LIFECYCLE_CAPACITY`] (`2 * MAX_LIVE_CONNECTIONS`).
pub fn conn_lifecycle_channel(exited: Arc<AtomicBool>) -> (ConnLifecycleClient, ConnLifecycleIntake) {
    let (req_tx, req_rx) = RingBuffer::new(CONN_LIFECYCLE_CAPACITY);
    let (rep_tx, rep_rx) = RingBuffer::new(CONN_LIFECYCLE_CAPACITY);
    let client = ConnLifecycleClient {
        inner: Mutex::new(ConnLifecycleClientInner {
            req_tx,
            rep_rx,
            slots: HashMap::with_capacity(MAX_LIVE_CONNECTIONS),
            next_seq: 0,
            closed: false,
            waiters: HashMap::with_capacity(CONN_LIFECYCLE_CAPACITY),
            completed_disconnects: VecDeque::with_capacity(CONN_LIFECYCLE_CAPACITY),
            exited,
        }),
    };
    let intake = ConnLifecycleIntake {
        req_rx,
        rep_tx,
        pending: VecDeque::with_capacity(CONN_LIFECYCLE_CAPACITY),
        capacity: CONN_LIFECYCLE_CAPACITY,
    };
    (client, intake)
}

impl ConnLifecycleClient {
    pub fn drain_replies(&self) {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .drain_replies();
    }

    pub fn close(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
    }

    pub fn finish_stopped(&self) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.closed = true;
        g.pop_all_replies();
        g.fail_remaining(ConnLifecycleError::Stopped);
    }

    /// Number of currently occupied connection slots (including closing slots awaiting owner confirmation).
    pub fn live_slot_count(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .slots
            .len()
    }

    /// True iff `conn_id` currently holds an open (not yet closing) off-thread slot.
    pub fn is_slot_open(&self, conn_id: u64) -> bool {
        self.inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .slots
            .get(&conn_id)
            == Some(&SlotState::Open)
    }

    /// Seeds connections already registered under legacy mode before ownership handoff to the
    /// RT executor so their liveness check and 1:1 disconnect slot carry across handoff.
    pub fn seed_live_connections<I: IntoIterator<Item = u64>>(&self, ids: I) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        for id in ids {
            if id != 0 && g.slots.len() < MAX_LIVE_CONNECTIONS {
                g.slots.insert(id, SlotState::Open);
            }
        }
    }

    fn enqueue_open_inner(
        &self,
        conn_id: u64,
        waiter: LifecycleWaiter,
    ) -> Result<(), ConnLifecycleError> {
        if conn_id == 0 {
            return Err(ConnLifecycleError::InvalidConnection);
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.drain_replies();
        if g.closed || g.exited.load(Ordering::Acquire) {
            return Err(ConnLifecycleError::Stopped);
        }
        if g.slots.contains_key(&conn_id) {
            return Err(ConnLifecycleError::AlreadyClosed);
        }
        if g.slots.len() >= MAX_LIVE_CONNECTIONS {
            return Err(ConnLifecycleError::TooManyConnections);
        }
        g.next_seq += 1;
        let seq = g.next_seq;
        if g.req_tx.push(LifecycleCmd::Open { seq, conn_id }).is_err() {
            return Err(ConnLifecycleError::TooManyConnections);
        }
        g.slots.insert(conn_id, SlotState::Open);
        g.waiters.entry(seq).or_default().push(waiter);
        Ok(())
    }

    fn enqueue_disconnect_inner(
        &self,
        connection: ConnectionHandle,
        waiter: Option<LifecycleWaiter>,
    ) -> Result<(), ConnLifecycleError> {
        let conn_id = connection.id();
        if conn_id == 0 {
            return Err(ConnLifecycleError::InvalidConnection);
        }
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.drain_replies();
        if g.closed || g.exited.load(Ordering::Acquire) {
            return Err(ConnLifecycleError::Stopped);
        }
        // Restrict reserved priority ring capacity strictly to registered open slots so
        // unregistered callers can never consume disconnect credits, and only commit `Closing`
        // AFTER the ring push succeeds. If a non-blocking immediate fence already transitioned
        // this slot to `Closing { seq }`, attach the waiter to the in-flight sequence without
        // pushing a duplicate ring command; if RT already completed it, deliver the recorded outcome.
        match g.slots.get(&conn_id).copied() {
            Some(SlotState::Open) => {}
            Some(SlotState::Closing { seq }) => {
                if let Some(w) = waiter {
                    g.waiters.entry(seq).or_default().push(w);
                }
                return Ok(());
            }
            None => {
                if let Some(outcome) = g
                    .completed_disconnects
                    .iter()
                    .rev()
                    .find(|(id, _)| *id == conn_id)
                    .map(|(_, o)| *o)
                {
                    if let Some(w) = waiter {
                        w.deliver(Ok(LifecycleReply::Disconnected {
                            seq: 0,
                            conn_id,
                            outcome,
                        }));
                    }
                    return Ok(());
                }
                return Err(ConnLifecycleError::AlreadyClosed);
            }
        }
        let seq = g.next_seq + 1;
        if g.req_tx
            .push(LifecycleCmd::Disconnect { seq, connection })
            .is_err()
        {
            return Err(ConnLifecycleError::TooManyConnections);
        }
        g.next_seq = seq;
        if let Some(state) = g.slots.get_mut(&conn_id) {
            *state = SlotState::Closing { seq };
        }
        if let Some(w) = waiter {
            g.waiters.entry(seq).or_default().push(w);
        }
        Ok(())
    }

    pub(crate) fn enqueue_open_oneshot(
        &self,
        conn_id: u64,
        tx: tokio::sync::oneshot::Sender<Result<LifecycleReply, ConnLifecycleError>>,
    ) -> Result<(), ConnLifecycleError> {
        self.enqueue_open_inner(conn_id, LifecycleWaiter::Async(tx))
    }

    pub(crate) fn enqueue_disconnect_oneshot(
        &self,
        connection: ConnectionHandle,
        tx: tokio::sync::oneshot::Sender<Result<LifecycleReply, ConnLifecycleError>>,
    ) -> Result<(), ConnLifecycleError> {
        self.enqueue_disconnect_inner(connection, Some(LifecycleWaiter::Async(tx)))
    }

    pub(crate) fn enqueue_disconnect_sync(
        &self,
        connection: ConnectionHandle,
        tx: mpsc::SyncSender<Result<LifecycleReply, ConnLifecycleError>>,
    ) -> Result<(), ConnLifecycleError> {
        self.enqueue_disconnect_inner(connection, Some(LifecycleWaiter::Sync(tx)))
    }

    /// Non-blocking fire-and-forget disconnect handoff for immediate socket-close fences,
    /// drop safety nets, and checkpoint hooks (never duplicates an in-flight `Closing` slot).
    pub fn enqueue_disconnect_detached(&self, connection: ConnectionHandle) {
        let _ = self.enqueue_disconnect_inner(connection, None);
    }

    pub fn open_sync(&self, conn_id: u64) -> Result<Result<(), &'static str>, ConnLifecycleError> {
        let (tx, rx) = mpsc::sync_channel(1);
        self.enqueue_open_inner(conn_id, LifecycleWaiter::Sync(tx))?;
        match rx.recv() {
            Ok(Ok(LifecycleReply::Opened { result, .. })) => Ok(result),
            Ok(Err(e)) => Err(e),
            _ => Err(ConnLifecycleError::Stopped),
        }
    }

    pub async fn open_async(
        &self,
        conn_id: u64,
    ) -> Result<Result<(), &'static str>, ConnLifecycleError> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.enqueue_open_inner(conn_id, LifecycleWaiter::Async(tx))?;
        match rx.await {
            Ok(Ok(LifecycleReply::Opened { result, .. })) => Ok(result),
            Ok(Err(e)) => Err(e),
            _ => Err(ConnLifecycleError::Stopped),
        }
    }

    pub fn disconnect_sync(
        &self,
        connection: ConnectionHandle,
    ) -> Result<LeaseOutcome, ConnLifecycleError> {
        let (tx, rx) = mpsc::sync_channel(1);
        self.enqueue_disconnect_inner(connection, Some(LifecycleWaiter::Sync(tx)))?;
        match rx.recv() {
            Ok(Ok(LifecycleReply::Disconnected { outcome, .. })) => Ok(outcome),
            Ok(Err(e)) => Err(e),
            _ => Err(ConnLifecycleError::Stopped),
        }
    }

    pub async fn disconnect_async(
        &self,
        connection: ConnectionHandle,
    ) -> Result<LeaseOutcome, ConnLifecycleError> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.enqueue_disconnect_inner(connection, Some(LifecycleWaiter::Async(tx)))?;
        match rx.await {
            Ok(Ok(LifecycleReply::Disconnected { outcome, .. })) => Ok(outcome),
            Ok(Err(e)) => Err(e),
            _ => Err(ConnLifecycleError::Stopped),
        }
    }
}
