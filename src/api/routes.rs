//! Unified REST + WebSocket route capability registry (Executor Services Layer, spec item 1).
//!
//! Single source of truth for:
//! - Route ownership classification (`OwnerAgnostic` | `ExecutorRequest` | `LegacyOnly`)
//! - Operator authorization policy, lease policy, and fault-fence availability
//! - Axum REST router registration (`build_rest_router`) and owner/fault gate (`dispatch_rest_gate`)
//! - Unified `lookup(RouteKey) -> Option<&'static RouteSpec>` covering every REST route and every
//!   WS frame/command (bridged directly from `crate::api::ws::dispatch`), failing closed on unknown keys.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, LazyLock};

use axum::{
    body::Body,
    extract::DefaultBodyLimit,
    handler::Handler,
    http::{Method, Request, StatusCode},
    middleware::{self, Next},
    response::{IntoResponse, Response},
    routing::{delete, get, post, MethodRouter},
    Router,
};

use crate::supervisor::ArmSupervisor;

use super::ws::dispatch::{self as ws_dispatch, FrameKind};
use super::{diagnostics, proxy, rest, sandbox};

/// HTTP method key normalized for the route capability registry (`HEAD` maps to `Get`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RestMethod {
    Get,
    Post,
    Delete,
}

impl RestMethod {
    pub fn from_http(method: &Method) -> Option<Self> {
        match *method {
            Method::GET | Method::HEAD => Some(Self::Get),
            Method::POST => Some(Self::Post),
            Method::DELETE => Some(Self::Delete),
            _ => None,
        }
    }
}

/// Primary lookup key in the unified REST + WS route capability registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RouteKey {
    Rest {
        method: RestMethod,
        template: &'static str,
    },
    Ws {
        kind: FrameKind,
        command: Option<&'static str>,
    },
}

impl RouteKey {
    pub const fn rest(method: RestMethod, template: &'static str) -> Self {
        Self::Rest { method, template }
    }

    pub const fn ws_frame(kind: FrameKind) -> Self {
        Self::Ws { kind, command: None }
    }

    pub const fn ws_command(command: &'static str) -> Self {
        Self::Ws {
            kind: FrameKind::Command,
            command: Some(command),
        }
    }
}

/// Execution ownership requirement of a route or WS frame/command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteOwnership {
    /// Does not depend on `SupervisorInner` / RT owner state (static assets, auth, stateless proxy).
    OwnerAgnostic,
    /// Routed through the active owner (Legacy or RT executor; owner and broker authority revalidated at execution).
    ExecutorRequest,
    /// Requires the legacy supervisor lock; refused with `503 EXECUTOR_ROUTING_PENDING` on `OWNER_EXECUTOR`.
    LegacyOnly,
}

/// Operator authorization policy declared in [`RouteSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatorPolicy {
    None,
    /// Enforced unconditionally before the handler (`require_operator` middleware on REST, dispatcher on WS).
    Required,
    /// Evaluated live against lock-free `OperatorGrant` inside owner execution.
    OwnerGrant,
    /// Evaluated conditionally from request payload / owner state.
    Conditional,
    /// Evaluated inside the handler itself.
    InHandler,
}

/// Motion lease policy declared in [`RouteSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeasePolicy {
    None,
    /// REST control route that enforces no conflicting WS lease (`LEASE_ACTIVE` / `LEASE_REQUIRED`) in handler.
    RestNoActiveLease,
    Required,
    RequiredExceptDryRun,
    RequiredExplicitGeneration,
    RequiredAllowZeroGeneration,
}

/// Whether a route or WS frame/command remains available while `internal_fault_fenced()` is latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultAvailability {
    Allowed,
    Fenced,
}

/// Canonical capability specification stored in the route registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RouteSpec {
    pub handler: &'static str,
    pub ownership: RouteOwnership,
    pub operator: OperatorPolicy,
    pub lease: LeasePolicy,
    pub fault: FaultAvailability,
}

/// Concrete REST route binding registered into Axum from [`REST_ROUTES`].
struct RestRouteEntry {
    method: RestMethod,
    template: &'static str,
    spec: RouteSpec,
    build_method: fn() -> MethodRouter<Arc<ArmSupervisor>>,
}

fn rest_get<H, T>(handler: H, op: OperatorPolicy) -> MethodRouter<Arc<ArmSupervisor>>
where
    H: Handler<T, Arc<ArmSupervisor>> + Clone,
    T: 'static,
{
    match op {
        OperatorPolicy::Required => get(handler.layer(middleware::from_fn(super::require_operator))),
        _ => get(handler),
    }
}

fn rest_post<H, T>(handler: H, op: OperatorPolicy) -> MethodRouter<Arc<ArmSupervisor>>
where
    H: Handler<T, Arc<ArmSupervisor>> + Clone,
    T: 'static,
{
    match op {
        OperatorPolicy::Required => post(handler.layer(middleware::from_fn(super::require_operator))),
        _ => post(handler),
    }
}

fn rest_delete<H, T>(handler: H, op: OperatorPolicy) -> MethodRouter<Arc<ArmSupervisor>>
where
    H: Handler<T, Arc<ArmSupervisor>> + Clone,
    T: 'static,
{
    match op {
        OperatorPolicy::Required => {
            delete(handler.layer(middleware::from_fn(super::require_operator)))
        }
        _ => delete(handler),
    }
}

fn rest_post_limit<H, T>(
    handler: H,
    limit: usize,
    op: OperatorPolicy,
) -> MethodRouter<Arc<ArmSupervisor>>
where
    H: Handler<T, Arc<ArmSupervisor>> + Clone,
    T: 'static,
{
    let limited = handler.layer(DefaultBodyLimit::max(limit));
    match op {
        OperatorPolicy::Required => post(limited.layer(middleware::from_fn(super::require_operator))),
        _ => post(limited),
    }
}

fn rest_post_sandbox_upload<H, T>(handler: H, op: OperatorPolicy) -> MethodRouter<Arc<ArmSupervisor>>
where
    H: Handler<T, Arc<ArmSupervisor>> + Clone,
    T: 'static,
{
    // Exact precedence: require_operator (outer) -> upload_concurrency_middleware -> DefaultBodyLimit -> handler.
    let inner = handler
        .layer(DefaultBodyLimit::max(sandbox::max_upload_body_bytes()))
        .layer(middleware::from_fn(sandbox::upload_concurrency_middleware));
    match op {
        OperatorPolicy::Required => post(inner.layer(middleware::from_fn(super::require_operator))),
        _ => post(inner),
    }
}

macro_rules! rr {
    (GET $path:expr, $h:path, $own:ident, $op:ident, $lease:ident, $fault:ident) => {
        RestRouteEntry {
            method: RestMethod::Get,
            template: $path,
            spec: RouteSpec {
                handler: stringify!($h),
                ownership: RouteOwnership::$own,
                operator: OperatorPolicy::$op,
                lease: LeasePolicy::$lease,
                fault: FaultAvailability::$fault,
            },
            build_method: || rest_get($h, OperatorPolicy::$op),
        }
    };
    (POST $path:expr, $h:path, $own:ident, $op:ident, $lease:ident, $fault:ident) => {
        RestRouteEntry {
            method: RestMethod::Post,
            template: $path,
            spec: RouteSpec {
                handler: stringify!($h),
                ownership: RouteOwnership::$own,
                operator: OperatorPolicy::$op,
                lease: LeasePolicy::$lease,
                fault: FaultAvailability::$fault,
            },
            build_method: || rest_post($h, OperatorPolicy::$op),
        }
    };
    (DELETE $path:expr, $h:path, $own:ident, $op:ident, $lease:ident, $fault:ident) => {
        RestRouteEntry {
            method: RestMethod::Delete,
            template: $path,
            spec: RouteSpec {
                handler: stringify!($h),
                ownership: RouteOwnership::$own,
                operator: OperatorPolicy::$op,
                lease: LeasePolicy::$lease,
                fault: FaultAvailability::$fault,
            },
            build_method: || rest_delete($h, OperatorPolicy::$op),
        }
    };
    (POST_LIMIT $limit:expr, $path:expr, $h:path, $own:ident, $op:ident, $lease:ident, $fault:ident) => {
        RestRouteEntry {
            method: RestMethod::Post,
            template: $path,
            spec: RouteSpec {
                handler: stringify!($h),
                ownership: RouteOwnership::$own,
                operator: OperatorPolicy::$op,
                lease: LeasePolicy::$lease,
                fault: FaultAvailability::$fault,
            },
            build_method: || rest_post_limit($h, $limit, OperatorPolicy::$op),
        }
    };
    (POST_UPLOAD $path:expr, $h:path, $own:ident, $op:ident, $lease:ident, $fault:ident) => {
        RestRouteEntry {
            method: RestMethod::Post,
            template: $path,
            spec: RouteSpec {
                handler: stringify!($h),
                ownership: RouteOwnership::$own,
                operator: OperatorPolicy::$op,
                lease: LeasePolicy::$lease,
                fault: FaultAvailability::$fault,
            },
            build_method: || rest_post_sandbox_upload($h, OperatorPolicy::$op),
        }
    };
}

/// Complete declarative table of every REST route served by `armd`.
static REST_ROUTES: &[RestRouteEntry] = &[
    // Auth
    rr!(GET    "/api/v1/auth/status", rest::auth_status, OwnerAgnostic, None, None, Allowed),
    rr!(POST   "/api/v1/auth/unlock", rest::auth_unlock, OwnerAgnostic, None, None, Allowed),
    rr!(POST   "/api/v1/auth/lock",   rest::auth_lock,   OwnerAgnostic, None, None, Allowed),
    // Calibration
    rr!(GET    "/api/v1/calibration",               rest::get_calibration,                 ExecutorRequest, None,     None, Allowed),
    rr!(GET    "/api/v1/calibration/hover",         crate::calib_hover::get_jobs,          ExecutorRequest, None,     None, Allowed),
    rr!(GET    "/api/v1/calibration/hover/{id}",    crate::calib_hover::get_job,           ExecutorRequest, None,     None, Allowed),
    rr!(GET    "/api/v1/calibration/sessions",      crate::calibration_sessions::get_sessions, OwnerAgnostic, None,   None, Allowed),
    rr!(POST_LIMIT crate::calibration_sessions::MAX_BODY_BYTES,
               "/api/v1/calibration/sessions",      crate::calibration_sessions::post_session, OwnerAgnostic, Required, None, Fenced),
    rr!(GET    "/api/v1/calibration/sessions/{id}", crate::calibration_sessions::get_session,  OwnerAgnostic, None,   None, Allowed),
    rr!(DELETE "/api/v1/calibration/sessions/{id}", crate::calibration_sessions::delete_session, OwnerAgnostic, Required, None, Fenced),
    // Cockpit / static assets
    rr!(GET    "/",                       super::serve_index,        OwnerAgnostic, None, None, Allowed),
    rr!(GET    "/assets/jmuxer.min.js",   super::serve_jmuxer,       OwnerAgnostic, None, None, Allowed),
    rr!(GET    "/assets/calib_wizard.js", super::serve_calib_wizard, OwnerAgnostic, None, None, Allowed),
    rr!(GET    "/assets/three-urdf.min.js", super::assets::serve_three, OwnerAgnostic, None, None, Allowed),
    rr!(GET    "/assets/model/{*path}",   super::assets::serve_model, OwnerAgnostic, None, None, Allowed),
    rr!(GET    "/assets/wasm/{*path}",    super::assets::serve_wasm,  OwnerAgnostic, None, None, Allowed),
    // Sandbox
    rr!(GET    "/api/v1/sandbox/catalog",                   sandbox::get_catalog,       ExecutorRequest, None,     None, Allowed),
    rr!(POST_UPLOAD "/api/v1/sandbox/scenes",               sandbox::upload_scene,      OwnerAgnostic,   Required, None, Fenced),
    rr!(GET    "/api/v1/sandbox/scenes/{id}",               sandbox::get_scene,         OwnerAgnostic,   None,     None, Allowed),
    rr!(GET    "/api/v1/sandbox/scenes/{id}/files/{*path}", sandbox::serve_scene_file,  OwnerAgnostic,   None,     None, Allowed),
    rr!(POST_UPLOAD "/api/v1/sandbox/tools",                sandbox::upload_tool,       OwnerAgnostic,   Required, None, Fenced),
    rr!(GET    "/api/v1/sandbox/tools/{id}",                sandbox::get_tool,          OwnerAgnostic,   None,     None, Allowed),
    rr!(GET    "/api/v1/sandbox/tools/{id}/files/{*path}",  sandbox::serve_tool_file,   OwnerAgnostic,   None,     None, Allowed),
    rr!(POST_UPLOAD "/api/v1/sandbox/policies",             sandbox::upload_policy,     OwnerAgnostic,   Required, None, Fenced),
    rr!(GET    "/api/v1/sandbox/policies/{id}",             sandbox::get_policy,        OwnerAgnostic,   None,     None, Allowed),
    rr!(DELETE "/api/v1/sandbox/policies/{id}",             sandbox::delete_policy,     ExecutorRequest, Required, None, Fenced),
    rr!(GET    "/api/v1/sandbox/policies/{id}/files/{*path}", sandbox::serve_policy_file, OwnerAgnostic, None,     None, Allowed),
    rr!(GET    "/api/v1/sandbox/base-assets/{*path}",       sandbox::serve_base_asset,  OwnerAgnostic,   None,     None, Allowed),
    rr!(POST   "/api/v1/sandbox/activate",                  sandbox::activate,          ExecutorRequest, Required, None, Fenced),
    rr!(POST   "/api/v1/sandbox/recover-split",             sandbox::recover_split,     ExecutorRequest, Required, None, Fenced),
    rr!(GET    "/api/v1/sandbox/bundle",                    sandbox::get_bundle,        ExecutorRequest, None,     None, Allowed),
    rr!(GET    "/api/v1/sandbox/state",                     sandbox::get_state,         OwnerAgnostic,   None,     None, Allowed),
    rr!(POST   "/api/v1/sandbox/reset",                     sandbox::reset,             OwnerAgnostic,   Required, None, Fenced),
    rr!(POST   "/api/v1/sandbox/io",                        sandbox::set_io,            OwnerAgnostic,   Required, None, Fenced),
    rr!(POST   "/api/v1/sandbox/telemetry",                 sandbox::post_telemetry,    OwnerAgnostic,   Required, None, Fenced),
    // Core observation, torque, heartbeat, motion, actions
    rr!(GET    "/api/v1/observation",               rest::get_observation,        ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/observation/schema",        rest::get_observation_schema, ExecutorRequest, None,      None,              Allowed),
    rr!(POST   "/api/v1/torque",                    rest::set_torque,             ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/arm",                       rest::set_torque,             ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/heartbeat",                 rest::heartbeat,              ExecutorRequest, None,      None,              Fenced),
    rr!(POST   "/api/v1/motion/jog",                rest::motion_jog,             ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/motion/home",               rest::motion_home,            ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/motion/unload",             rest::motion_unload,          ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/motion/servo",              rest::motion_servo,           ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/move_to",            rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/move_relative",      rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/gripper",            rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/gripper_zero_check", rest::action_requires_lease,  LegacyOnly,      InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/grasp",              rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/move_to_pose",       rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/place",              rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/action/replay",             rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   crate::measure_multiview::PATH,      rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/mode/gravity_comp",         rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    #[cfg(feature = "policy-runtime")]
    rr!(POST   "/api/v1/action/policy/start",       rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    #[cfg(feature = "policy-runtime")]
    rr!(POST   "/api/v1/action/policy/stop",        rest::action_requires_lease,  ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    // Config, safety, poses, measure, recording, episodes, sequences, diagnostics, WS upgrade
    rr!(GET    "/api/v1/gripper/calibration",       rest::get_gripper_calibration,      ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/tool/commissioning",        rest::get_tool_commissioning,       ExecutorRequest, None,      None,              Allowed),
    rr!(POST   "/api/v1/tool/commissioning/confirm", rest::post_tool_commissioning_confirm, ExecutorRequest, Required, None,           Fenced),
    rr!(GET    "/api/v1/grasp/setup",               rest::get_grasp_setup,              ExecutorRequest, None,      None,              Allowed),
    rr!(POST   crate::depth_measure::PATH,          rest::post_grasp_measure,           ExecutorRequest, Required,  None,              Fenced),
    rr!(GET    "/api/v1/workspace/keepout",         rest::get_keepout,                  ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/safety/collision",          rest::get_collision,                ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/safety/describe",           rest::get_safety_describe,          ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/poses",                     rest::get_poses,                    ExecutorRequest, None,      None,              Allowed),
    rr!(POST   "/api/v1/safety/e_stop",             rest::safety_e_stop,                ExecutorRequest, None,      None,              Allowed),
    rr!(POST   "/api/v1/safety/clear_fault",        rest::safety_clear_fault,           ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(POST   "/api/v1/dataset/record/start",      rest::dataset_record_start,         LegacyOnly,      Required,  None,              Fenced),
    rr!(POST   "/api/v1/dataset/record/save",       rest::dataset_record_save,          LegacyOnly,      Required,  None,              Fenced),
    rr!(POST   "/api/v1/dataset/record/discard",    rest::dataset_record_discard,       LegacyOnly,      Required,  None,              Fenced),
    rr!(GET    "/api/v1/dataset/episodes",          rest::dataset_episodes,             LegacyOnly,      None,      None,              Allowed),
    rr!(DELETE "/api/v1/dataset/episodes/{id}",     rest::dataset_episode_delete,       LegacyOnly,      Required,  None,              Fenced),
    rr!(GET    "/api/v1/episodes",                  rest::get_episodes,                 LegacyOnly,      None,      None,              Allowed),
    rr!(POST_LIMIT super::IMPORT_BODY_LIMIT,
               "/api/v1/episodes/import",           rest::episodes_import,              LegacyOnly,      Required,  None,              Fenced),
    rr!(GET    "/api/v1/episodes/{id}",             rest::get_episode,                  LegacyOnly,      None,      None,              Allowed),
    rr!(GET    "/api/v1/episodes/{id}/data",        rest::get_episode_data,             LegacyOnly,      None,      None,              Allowed),
    rr!(GET    "/api/v1/datasets",                  rest::get_datasets,                 LegacyOnly,      None,      None,              Allowed),
    rr!(GET    "/api/v1/sequences",                 rest::get_sequences,                ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/sequences/{id}",            rest::get_sequence,                 ExecutorRequest, None,      None,              Allowed),
    rr!(POST   "/api/v1/action/run_sequence",       rest::action_requires_lease,        ExecutorRequest, InHandler, RestNoActiveLease, Fenced),
    rr!(GET    "/api/v1/diagnostics",               diagnostics::get_diagnostics,       ExecutorRequest, None,      None,              Allowed),
    rr!(GET    "/api/v1/diagnostics/boots",         diagnostics::get_boots,             OwnerAgnostic,   None,      None,              Allowed),
    rr!(GET    "/api/v1/diagnostics/blackbox",      diagnostics::get_blackbox,          OwnerAgnostic,   None,      None,              Allowed),
    rr!(GET    "/api/v1/diagnostics/tick",          diagnostics::get_tick,              ExecutorRequest, None,      None,              Allowed),
    rr!(POST   "/api/v1/diagnostics/tick/mark",     diagnostics::post_tick_mark,        ExecutorRequest, Required,  None,              Fenced),
    rr!(GET    "/api/v1/diagnostics/power",         diagnostics::get_power,             LegacyOnly,      None,      None,              Allowed),
    rr!(GET    "/api/v1/ws",                        super::ws::ws_handler,              ExecutorRequest, None,      None,              Allowed),
    // Reverse proxy & vision routes
    rr!(GET    "/api/v1/detection/ws",              proxy::proxy_detection_ws,          OwnerAgnostic,   None,      None,              Allowed),
    rr!(GET    "/results",                          proxy::proxy_detection_ws,          OwnerAgnostic,   None,      None,              Allowed),
    rr!(GET    "/api/v1/detection/inference",       proxy::get_detection_inference,     OwnerAgnostic,   None,      None,              Allowed),
    rr!(POST   "/api/v1/detection/inference",       proxy::post_detection_inference,    OwnerAgnostic,   Required,  None,              Fenced),
    rr!(GET    "/api/v1/detection/latest",          proxy::get_detection_latest,        OwnerAgnostic,   None,      None,              Allowed),
    rr!(GET    "/api/v1/camera/snapshot",           proxy::get_camera_snapshot,         OwnerAgnostic,   None,      None,              Allowed),
    rr!(POST   "/api/v1/detection/trigger",         proxy::post_detection_trigger,      OwnerAgnostic,   Required,  None,              Fenced),
    rr!(GET    "/api/v1/detection/mask_focus",      proxy::get_detection_mask_focus,    OwnerAgnostic,   None,      None,              Allowed),
    rr!(POST   "/api/v1/detection/mask_focus",      proxy::post_detection_mask_focus,   OwnerAgnostic,   Required,  None,              Fenced),
    rr!(GET    "/api/v1/vision/mode",               proxy::get_vision_mode,             OwnerAgnostic,   None,      None,              Allowed),
    rr!(POST   "/api/v1/vision/mode",               proxy::post_vision_mode,            OwnerAgnostic,   Required,  None,              Fenced),
    rr!(POST   crate::vision_detector::PATH,        rest::post_vision_detector,         ExecutorRequest, Required,  None,              Fenced),
    rr!(POST   "/api/v1/vision/subscribe",          proxy::post_vision_subscribe,       OwnerAgnostic,   Required,  None,              Fenced),
    rr!(DELETE "/api/v1/vision/subscribe/{id}",     proxy::delete_vision_subscribe,     OwnerAgnostic,   Required,  None,              Fenced),
    rr!(POST   "/api/v1/vision/subscribe/{id}/keepalive", proxy::post_vision_keepalive, OwnerAgnostic,   Required,  None,              Fenced),
    rr!(GET    "/api/v1/vision/subscriptions",      proxy::get_vision_subscriptions,    OwnerAgnostic,   None,      None,              Allowed),
    rr!(GET    "/api/v1/vision/depth",              proxy::get_vision_depth,            OwnerAgnostic,   Required,  None,              Allowed),
    #[cfg(feature = "policy-runtime")]
    rr!(POST   "/api/v1/vision/clock-handshake",    super::post_vision_clock_handshake, ExecutorRequest, Required,  None,              Fenced),
    #[cfg(feature = "policy-runtime")]
    rr!(POST   "/api/v1/vision/embedding",          super::post_vision_embedding,       ExecutorRequest, Required,  None,              Fenced),
];

struct RouteRegistry {
    by_key: HashMap<RouteKey, RouteSpec>,
    rest_get_exact: HashMap<&'static str, RouteSpec>,
    rest_post_exact: HashMap<&'static str, RouteSpec>,
    rest_delete_exact: HashMap<&'static str, RouteSpec>,
    rest_param: Vec<(RestMethod, &'static str, RouteSpec)>,
    ws_commands: HashMap<&'static str, RouteSpec>,
    ws_frames: HashMap<FrameKind, RouteSpec>,
}

static REGISTRY: LazyLock<RouteRegistry> = LazyLock::new(|| {
    let mut by_key = HashMap::new();
    let mut rest_get_exact = HashMap::new();
    let mut rest_post_exact = HashMap::new();
    let mut rest_delete_exact = HashMap::new();
    let mut rest_param = Vec::new();
    let mut ws_commands = HashMap::new();
    let mut ws_frames = HashMap::new();

    for entry in REST_ROUTES {
        assert!(
            entry.template.starts_with('/'),
            "REST route template must start with '/': {}",
            entry.template
        );
        let key = RouteKey::rest(entry.method, entry.template);
        assert!(
            by_key.insert(key, entry.spec).is_none(),
            "duplicate REST route in capability registry: {key:?}"
        );
        if entry.template.contains('{') {
            rest_param.push((entry.method, entry.template, entry.spec));
        } else {
            match entry.method {
                RestMethod::Get => {
                    rest_get_exact.insert(entry.template, entry.spec);
                }
                RestMethod::Post => {
                    rest_post_exact.insert(entry.template, entry.spec);
                }
                RestMethod::Delete => {
                    rest_delete_exact.insert(entry.template, entry.spec);
                }
            }
        }
    }

    for entry in ws_dispatch::all_entries() {
        let spec = entry.route_spec();
        let key = RouteKey::Ws {
            kind: entry.key.kind,
            command: entry.key.path,
        };
        assert!(
            by_key.insert(key, spec).is_none(),
            "duplicate WS route in capability registry: {key:?}"
        );
        match (entry.key.kind, entry.key.path) {
            (FrameKind::Command, Some(cmd)) => {
                ws_commands.insert(cmd, spec);
            }
            (kind, None) => {
                ws_frames.insert(kind, spec);
            }
            other => panic!("invalid WS DispatchKey shape in registry: {other:?}"),
        }
    }

    RouteRegistry {
        by_key,
        rest_get_exact,
        rest_post_exact,
        rest_delete_exact,
        rest_param,
        ws_commands,
        ws_frames,
    }
});

/// Pre-initializes the unified route registry (called once at router construction).
pub fn prewarm_registry() {
    LazyLock::force(&REGISTRY);
}

/// Exact lookup by [`RouteKey`] (REST method + template or WS frame kind + command).
/// Unknown keys return `None` (fail closed).
pub fn lookup(key: RouteKey) -> Option<&'static RouteSpec> {
    REGISTRY.by_key.get(&key)
}

/// Zero-allocation runtime lookup for a WS frame kind and optional command path.
pub fn lookup_ws(kind: FrameKind, command: Option<&str>) -> Option<&'static RouteSpec> {
    match (kind, command) {
        (FrameKind::Command, Some(cmd)) => REGISTRY.ws_commands.get(cmd),
        (FrameKind::Command, None) => None,
        (other, None) => REGISTRY.ws_frames.get(&other),
        (_, Some(_)) => None,
    }
}

/// Zero-allocation runtime lookup matching an HTTP method and concrete request path against
/// [`REST_ROUTES`] templates (static O(1) fast path, then parameterized `{id}` / `{*path}` match).
pub fn resolve_rest(method: &Method, path: &str) -> Option<&'static RouteSpec> {
    let m = RestMethod::from_http(method)?;
    let exact = match m {
        RestMethod::Get => REGISTRY.rest_get_exact.get(path),
        RestMethod::Post => REGISTRY.rest_post_exact.get(path),
        RestMethod::Delete => REGISTRY.rest_delete_exact.get(path),
    };
    if let Some(spec) = exact {
        return Some(spec);
    }
    for (rm, template, spec) in &REGISTRY.rest_param {
        if *rm == m && matches_template(template, path) {
            return Some(spec);
        }
    }
    None
}

/// Matches an Axum 0.8 path template containing `{param}` and/or trailing `{*wildcard}` segments
/// against a concrete request path without allocating.
fn matches_template(template: &str, path: &str) -> bool {
    let mut t_rem = template;
    let mut p_rem = path;
    loop {
        let Some(t_rest) = t_rem.strip_prefix('/') else {
            return false;
        };
        let Some(p_rest) = p_rem.strip_prefix('/') else {
            return false;
        };
        let (t_seg, t_next) = match t_rest.find('/') {
            Some(i) => (&t_rest[..i], Some(&t_rest[i..])),
            None => (t_rest, None),
        };
        if t_seg.starts_with("{*") && t_seg.ends_with('}') && t_next.is_none() {
            // Axum 0.8 `{*path}` catch-all requires at least one byte after the preceding '/'.
            return !p_rest.is_empty();
        }
        let (p_seg, p_next) = match p_rest.find('/') {
            Some(i) => (&p_rest[..i], Some(&p_rest[i..])),
            None => (p_rest, None),
        };
        let seg_ok = if t_seg.starts_with('{') && t_seg.ends_with('}') {
            !p_seg.is_empty()
        } else {
            t_seg == p_seg
        };
        if !seg_ok {
            return false;
        }
        match (t_next, p_next) {
            (None, None) => return true,
            (Some(tn), Some(pn)) => {
                t_rem = tn;
                p_rem = pn;
            }
            _ => return false,
        }
    }
}

/// Generates the Axum REST router directly from [`REST_ROUTES`], merging methods per template
/// while preserving per-method `require_operator`, `DefaultBodyLimit`, and upload concurrency layers.
pub(super) fn build_rest_router() -> Router<Arc<ArmSupervisor>> {
    prewarm_registry();
    let mut by_template: BTreeMap<&'static str, MethodRouter<Arc<ArmSupervisor>>> = BTreeMap::new();
    for entry in REST_ROUTES {
        let mr = (entry.build_method)();
        match by_template.entry(entry.template) {
            std::collections::btree_map::Entry::Vacant(v) => {
                v.insert(mr);
            }
            std::collections::btree_map::Entry::Occupied(mut o) => {
                let existing = std::mem::take(o.get_mut());
                *o.get_mut() = existing.merge(mr);
            }
        }
    }
    let mut router = Router::new();
    for (template, mr) in by_template {
        router = router.route(template, mr);
    }
    router
}

/// Registry-driven REST fault fence and owner gate (replaces the hardcoded `gate_executor_owner`
/// allowlist in `src/api/mod.rs`). Runs as a router-level `.layer(...)` inside `same_origin` so
/// fallback 404/405/`Allow` semantics on `OWNER_LEGACY` and fail-closed `503` semantics on
/// `OWNER_EXECUTOR` / `OWNER_TRANSFERRING` / `OWNER_FAILED` remain exact.
pub(super) async fn dispatch_rest_gate(
    axum::extract::State(supervisor): axum::extract::State<Arc<ArmSupervisor>>,
    request: Request<Body>,
    next: Next,
) -> Response {
    let method = request.method();
    let path = request.uri().path();
    let spec = resolve_rest(method, path);

    if supervisor.internal_fault_fenced() {
        let allowed_under_fault = match spec {
            Some(s) => s.fault == FaultAvailability::Allowed,
            None => matches!(*method, Method::GET | Method::HEAD),
        };
        if !allowed_under_fault {
            return rest::ApiError(
                StatusCode::SERVICE_UNAVAILABLE,
                "Internal fault or SAFE-HOLD active; commands and rearm are fenced".into(),
                "INTERNAL_FAULT".into(),
            )
            .into_response();
        }
    }

    match supervisor.owner_mode() {
        crate::supervisor::OWNER_LEGACY => next.run(request).await,
        crate::supervisor::OWNER_EXECUTOR => {
            let allowed = spec.is_some_and(|s| {
                matches!(
                    s.ownership,
                    RouteOwnership::OwnerAgnostic | RouteOwnership::ExecutorRequest
                )
            });
            if allowed {
                next.run(request).await
            } else {
                rest::ApiError(
                    StatusCode::SERVICE_UNAVAILABLE,
                    format!(
                        "Route {method} {path} is disabled while RT executor owns ArmCore in stage 3 step 1"
                    ),
                    "EXECUTOR_ROUTING_PENDING".into(),
                )
                .into_response()
            }
        }
        _ => rest::ApiError(
            StatusCode::SERVICE_UNAVAILABLE,
            "RT executor ownership is transitioning or unavailable".into(),
            "EXECUTOR_UNAVAILABLE".into(),
        )
        .into_response(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn every_rest_and_ws_route_is_classified_exactly_once_and_unknown_keys_fail_closed() {
        prewarm_registry();

        // 1. Every REST route in REST_ROUTES is classified exactly once.
        let mut seen = HashSet::new();
        for entry in REST_ROUTES {
            let key = RouteKey::rest(entry.method, entry.template);
            assert!(seen.insert(key), "duplicate REST route key: {key:?}");
            let spec = lookup(key).unwrap_or_else(|| panic!("missing REST route in registry: {key:?}"));
            assert_eq!(*spec, entry.spec);
            assert!(!spec.handler.is_empty(), "empty handler name for {key:?}");
            // Verify template resolution agrees with exact key lookup on concrete sample paths.
            let sample_path = entry
                .template
                .replace("{id}", "sample-id")
                .replace("{*path}", "dir/file.bin");
            let http_method = match entry.method {
                RestMethod::Get => Method::GET,
                RestMethod::Post => Method::POST,
                RestMethod::Delete => Method::DELETE,
            };
            let resolved = resolve_rest(&http_method, &sample_path)
                .unwrap_or_else(|| panic!("resolve_rest missed {http_method} {sample_path} ({})", entry.template));
            assert_eq!(*resolved, entry.spec, "resolve_rest mismatch for {}", entry.template);
            if entry.method == RestMethod::Get {
                assert_eq!(
                    resolve_rest(&Method::HEAD, &sample_path),
                    Some(spec),
                    "HEAD must inherit GET spec for {}",
                    entry.template
                );
            }
        }

        // 2. Every WS DispatchEntry is classified exactly once and every known FrameKind is covered.
        let mut ws_count = 0usize;
        for entry in ws_dispatch::all_entries() {
            ws_count += 1;
            let key = RouteKey::Ws {
                kind: entry.key.kind,
                command: entry.key.path,
            };
            assert!(seen.insert(key), "duplicate WS route key: {key:?}");
            let spec = lookup(key).unwrap_or_else(|| panic!("missing WS route in registry: {key:?}"));
            assert_eq!(*spec, entry.route_spec());
            assert_eq!(lookup_ws(entry.key.kind, entry.key.path), Some(spec));
        }
        assert_eq!(REGISTRY.by_key.len(), REST_ROUTES.len() + ws_count);

        for kind in [
            FrameKind::EStop,
            FrameKind::Auth,
            FrameKind::Heartbeat,
            FrameKind::AcquireLease,
            FrameKind::ReleaseLease,
            FrameKind::Preset,
            FrameKind::Servo,
            FrameKind::ImplicitServo,
        ] {
            assert!(
                lookup(RouteKey::ws_frame(kind)).is_some(),
                "FrameKind::{kind:?} must be classified in registry"
            );
        }

        // 3. Unknown keys and unsupported methods fail closed (None).
        assert!(lookup(RouteKey::ws_frame(FrameKind::Unknown)).is_none());
        assert!(lookup_ws(FrameKind::Unknown, None).is_none());
        assert!(lookup_ws(FrameKind::Auth, Some("unknown")).is_none());
        assert!(lookup(RouteKey::ws_command("/api/v1/does_not_exist")).is_none());
        assert!(lookup_ws(FrameKind::Command, Some("/api/v1/does_not_exist")).is_none());
        assert!(lookup(RouteKey::rest(RestMethod::Get, "/api/v1/does_not_exist")).is_none());
        assert!(resolve_rest(&Method::GET, "/api/v1/does_not_exist").is_none());
        assert!(resolve_rest(&Method::OPTIONS, "/api/v1/observation").is_none());
        assert!(resolve_rest(&Method::PUT, "/api/v1/observation").is_none());
        assert!(resolve_rest(&Method::GET, "/assets/model/").is_none());
        assert!(resolve_rest(&Method::GET, "/api/v1/sequences/a/b").is_none());

        // 4. Reference adapter and key ownership invariants.
        assert_eq!(
            lookup(RouteKey::rest(RestMethod::Get, "/api/v1/vision/mode")).unwrap().ownership,
            RouteOwnership::OwnerAgnostic
        );
        assert_eq!(
            lookup(RouteKey::rest(RestMethod::Post, "/api/v1/vision/mode")).unwrap().ownership,
            RouteOwnership::OwnerAgnostic
        );
        assert_eq!(
            lookup(RouteKey::rest(RestMethod::Post, "/api/v1/motion/jog")).unwrap().ownership,
            RouteOwnership::ExecutorRequest
        );
        assert_eq!(
            lookup(RouteKey::rest(RestMethod::Post, crate::depth_measure::PATH)).unwrap().ownership,
            RouteOwnership::ExecutorRequest
        );
        assert_eq!(
            lookup(RouteKey::ws_command(crate::depth_measure::PATH)).unwrap().ownership,
            RouteOwnership::ExecutorRequest
        );
        assert_eq!(
            lookup(RouteKey::rest(RestMethod::Post, "/api/v1/dataset/record/start")).unwrap().ownership,
            RouteOwnership::LegacyOnly
        );
        assert_eq!(
            lookup(RouteKey::ws_command("/api/v1/mode/gravity_comp")).unwrap().ownership,
            RouteOwnership::ExecutorRequest
        );
        assert_eq!(
            lookup(RouteKey::ws_command("/api/v1/record/start")).unwrap().ownership,
            RouteOwnership::LegacyOnly
        );
        assert_eq!(
            lookup(RouteKey::rest(RestMethod::Post, "/api/v1/safety/e_stop")).unwrap().fault,
            FaultAvailability::Allowed
        );
        assert_eq!(
            lookup(RouteKey::ws_frame(FrameKind::ReleaseLease)).unwrap().fault,
            FaultAvailability::Allowed
        );
        assert_eq!(
            lookup(RouteKey::ws_frame(FrameKind::AcquireLease)).unwrap().fault,
            FaultAvailability::Fenced
        );
    }

    #[tokio::test]
    async fn generated_router_serves_vision_mode_get_as_owner_agnostic_on_executor_and_preserves_precedence() {
        use axum::http::header;
        use tower::ServiceExt;

        let sup = Arc::new(ArmSupervisor::new(crate::model::ArmConfig::rebot_dm()));
        let auth = crate::auth::OperatorAuth::enforced("123456", 1800);
        // Unreachable local detector port so proxy returns 502 (proving the handler ran, NOT 503 EXECUTOR_ROUTING_PENDING).
        let detector = proxy::DetectorUpstream::parse("http://127.0.0.1:1").unwrap();
        let router = super::super::create_router_with_detector(sup.clone(), detector, auth);

        // Legacy: OPTIONS /api/v1/observation -> 405 Method Not Allowed (baseline fallback preserved).
        let opt_legacy = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::OPTIONS)
                    .uri("/api/v1/observation")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(opt_legacy.status(), StatusCode::METHOD_NOT_ALLOWED);

        // Legacy: HEAD /api/v1/vision/depth inherits GET's require_operator -> 403 OPERATOR_REQUIRED.
        let depth_head = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::HEAD)
                    .uri("/api/v1/vision/depth")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(depth_head.status(), StatusCode::FORBIDDEN);

        // Legacy: GET /api/v1/calibration/sessions is public (no operator), POST requires operator.
        let sess_get = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::GET)
                    .uri("/api/v1/calibration/sessions")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_ne!(sess_get.status(), StatusCode::FORBIDDEN);
        let sess_post = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/api/v1/calibration/sessions")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from("{}"))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(sess_post.status(), StatusCode::FORBIDDEN);

        // Start RT executor and verify reference adapter GET/HEAD /api/v1/vision/mode vs POST /api/v1/vision/mode.
        let handle = sup
            .start_executor(crate::rt_core::executor::ExecutorConfig::for_tests(
                std::time::Duration::from_millis(20),
            ))
            .await
            .expect("start mock executor");

        for m in [Method::GET, Method::HEAD] {
            let resp = router
                .clone()
                .oneshot(
                    Request::builder()
                        .method(m.clone())
                        .uri("/api/v1/vision/mode")
                        .body(Body::empty())
                        .unwrap(),
                )
                .await
                .unwrap();
            assert_eq!(
                resp.status(),
                StatusCode::BAD_GATEWAY,
                "{m} /api/v1/vision/mode must bypass executor gate as OwnerAgnostic and reach proxy handler"
            );
        }

        // POST /api/v1/vision/mode is OwnerAgnostic (requires operator auth when enforced).
        let post_mode = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/api/v1/vision/mode")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from(r#"{"mode":"aruco"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(post_mode.status(), StatusCode::FORBIDDEN);

        // Unmigrated LegacyOnly route (POST /api/v1/dataset/record/start) still returns 503 EXECUTOR_ROUTING_PENDING.
        let legacy_rec = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/api/v1/dataset/record/start")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from("{}"))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(legacy_rec.status(), StatusCode::SERVICE_UNAVAILABLE);
        let legacy_body = axum::body::to_bytes(legacy_rec.into_body(), 4096).await.unwrap();
        assert!(
            String::from_utf8_lossy(&legacy_body).contains("EXECUTOR_ROUTING_PENDING"),
            "POST /api/v1/dataset/record/start must remain LegacyOnly: {legacy_body:?}"
        );

        // Executor: OPTIONS /api/v1/observation fails closed with 503 EXECUTOR_ROUTING_PENDING.
        let opt_exec = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::OPTIONS)
                    .uri("/api/v1/observation")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(opt_exec.status(), StatusCode::SERVICE_UNAVAILABLE);

        // Cross-origin control POST and WS upgrade are rejected by same_origin (403) before the gate.
        let xorigin_post = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/api/v1/vision/mode")
                    .header(header::HOST, "127.0.0.1:8765")
                    .header(header::ORIGIN, "http://evil.example")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from("{}"))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(xorigin_post.status(), StatusCode::FORBIDDEN);

        let xorigin_ws = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::GET)
                    .uri("/api/v1/ws")
                    .header(header::HOST, "127.0.0.1:8765")
                    .header(header::ORIGIN, "http://evil.example")
                    .header(header::UPGRADE, "websocket")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(xorigin_ws.status(), StatusCode::FORBIDDEN);

        // Internal fault fence: GET /api/v1/vision/mode and POST /api/v1/safety/e_stop stay allowed,
        // while mutating fenced routes return 503 INTERNAL_FAULT.
        sup.internal_fault_lane().latch_control_safe_hold_fence();
        let fault_get_mode = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::GET)
                    .uri("/api/v1/vision/mode")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(fault_get_mode.status(), StatusCode::BAD_GATEWAY);
        let fault_post_mode = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/api/v1/vision/mode")
                    .header(header::CONTENT_TYPE, "application/json")
                    .body(Body::from("{}"))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(fault_post_mode.status(), StatusCode::SERVICE_UNAVAILABLE);
        let fault_body = axum::body::to_bytes(fault_post_mode.into_body(), 4096).await.unwrap();
        assert!(
            String::from_utf8_lossy(&fault_body).contains("INTERNAL_FAULT"),
            "expected INTERNAL_FAULT under fault fence: {fault_body:?}"
        );
        let fault_estop = router
            .clone()
            .oneshot(
                Request::builder()
                    .method(Method::POST)
                    .uri("/api/v1/safety/e_stop")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(fault_estop.status(), StatusCode::OK);

        handle.stop_and_join();

        // Global OWNER_FAILED fence precedes OwnerAgnostic (returns 503 EXECUTOR_UNAVAILABLE).
        let sup_fail = Arc::new(ArmSupervisor::new(crate::model::ArmConfig::rebot_dm()));
        let hooks_fail = crate::rt_core::executor::PersistTestHooks {
            fail_init: Some(false),
            ..Default::default()
        };
        let _ = sup_fail
            .start_executor_with_hooks(
                crate::rt_core::executor::ExecutorConfig::for_tests(std::time::Duration::from_millis(5)),
                hooks_fail,
            )
            .await;
        assert_eq!(sup_fail.owner_mode(), crate::supervisor::OWNER_FAILED);
        let fail_router = super::super::create_router_with_detector(
            sup_fail.clone(),
            proxy::DetectorUpstream::parse("http://127.0.0.1:1").unwrap(),
            crate::auth::OperatorAuth::disabled(),
        );
        let fail_resp = fail_router
            .oneshot(
                Request::builder()
                    .method(Method::GET)
                    .uri("/api/v1/vision/mode")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(fail_resp.status(), StatusCode::SERVICE_UNAVAILABLE);
        let fail_body = axum::body::to_bytes(fail_resp.into_body(), 4096).await.unwrap();
        assert!(
            String::from_utf8_lossy(&fail_body).contains("EXECUTOR_UNAVAILABLE"),
            "OWNER_FAILED must return EXECUTOR_UNAVAILABLE before OwnerAgnostic: {fail_body:?}"
        );
        assert!(
            sup_fail
                .read_services(crate::rt_core::publish::ReadRequirements::identity())
                .await
                .is_err()
        );
    }
}
