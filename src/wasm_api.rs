//! Stateful, filesystem-free JSON facade shared by native tests and the WASM ABI.
use std::cell::RefCell;
use std::collections::VecDeque;

use serde::Deserialize;
use serde_json::{json, Value};

use crate::collision::{CollisionGuard, CollisionPolicy, Sensitivity};
use crate::kinematics::{self as k, Mat3};
use crate::model::{ArmConfig, FollowingErrorPolicy, KeepoutBox};
use crate::motion::{self, GoalFrame, GoalSpec, JointPath, MotionError, PlanInput, Speed};
use crate::torque_guard::TorqueGuard;

const MAX_PATHS: usize = 32;

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Request {
    Version,
    InitProfile {
        robot: Option<String>,
        config_json: Option<Value>,
        urdf_xml: Option<String>,
    },
    SetKeepout {
        boxes: Vec<KeepoutBox>,
    },
    SetToolEnvelope {
        #[serde(default)]
        tcp_offset: Option<[f64; 3]>,
        #[serde(default)]
        extra_check_points_end_m: Option<Vec<[f32; 3]>>,
        #[serde(default)]
        permanent_tool_check_points_end_m: Option<Vec<[f32; 3]>>,
        #[serde(default)]
        transient_payload_check_points_end_m: Option<Vec<[f32; 3]>>,
    },
    FkPose {
        q_motor: [f64; 6],
        include_tool_tcp: Option<bool>,
    },
    PlanCartesian {
        q0_motor: [f64; 6],
        j7: f32,
        goal: Goal,
        speed_mps: Option<f64>,
        angular_rps: Option<f64>,
        speed_scale: Option<f64>,
        auto_yaw: Option<bool>,
        include_tool_tcp: Option<bool>,
    },
    SamplePath {
        path_id: u64,
        t_s: f64,
    },
    CheckWorkspace {
        q_motor: [f32; 7],
    },
    ResetGuards,
    StepGuards {
        dt_s: f64,
        torqued: bool,
        q_cmd: Vec<f32>,
        q_meas: Vec<f32>,
        torques_nm: Vec<f32>,
        collision_enabled: bool,
        collision_sensitivity: Option<Sensitivity>,
    },
    ComposeMjcf {
        base_xml: Option<String>,
        scene_world_xml: Option<String>,
        tool_manifest: Option<Box<crate::mjcf_compose::ToolManifest>>,
        tool_xml: Option<String>,
        tool_preset: Option<String>,
    },
    ResolveToolConfig {
        manifest: Box<crate::tool::ToolPackConfig>,
        #[serde(default)]
        robot_id: Option<String>,
    },
}

#[derive(Deserialize)]
struct Goal {
    xyz: [f64; 3],
    rpy: Option<[f64; 3]>,
    mat: Option<Mat3>,
}

impl Goal {
    fn orientation(&self) -> Result<Option<Mat3>, MotionError> {
        finite(&self.xyz, "goal.xyz")?;
        if self.rpy.is_some() && self.mat.is_some() {
            return invalid("goal accepts either rpy or mat, not both");
        }
        if let Some(rpy) = self.rpy {
            finite(&rpy, "goal.rpy")?;
            return Ok(Some(k::rpy_to_r(rpy[0], rpy[1], rpy[2])));
        }
        if let Some(mat) = self.mat {
            for row in &mat {
                finite(row, "goal.mat")?;
            }
            // A finite but non-orthonormal matrix is not a rotation and cannot be
            // passed to the planner's rotation logarithm.
            let gram = k::mul_t(&mat, &mat);
            if (0..3)
                .any(|i| (0..3).any(|j| (gram[i][j] - if i == j { 1.0 } else { 0.0 }).abs() > 1e-6))
                || k::dot(&mat[0], &k::cross(&mat[1], &mat[2])) < 1.0 - 1e-6
            {
                return invalid("goal.mat must be a proper orthonormal rotation matrix");
            }
            return Ok(Some(mat));
        }
        Ok(None)
    }
}

struct StoredPath {
    path: JointPath,
    generation: u64,
    workspace_revision: u64,
}

/// Deterministic aggregate tracking-error timer. Like the native supervisor,
/// any out-of-bound arm joint keeps the timer running; the gripper is excluded.
/// The native supervisor's wall-clock adapter is outside the Stage 1 whitelist.
#[derive(Default)]
pub struct FollowingErrorMonitor {
    elapsed_s: f64,
    fault_joint: Option<usize>,
}

impl FollowingErrorMonitor {
    pub fn step(
        &mut self,
        dt_s: f64,
        torqued: bool,
        q_cmd: &[f32],
        q_meas: &[f32],
        gripper_index: Option<usize>,
        policy: &FollowingErrorPolicy,
    ) -> Option<usize> {
        if self.fault_joint.is_some() {
            return self.fault_joint;
        }
        let outside = q_cmd
            .iter()
            .zip(q_meas)
            .enumerate()
            .find(|(i, (cmd, meas))| {
                Some(*i) != gripper_index
                    && (!meas.is_finite() || (**cmd - **meas).abs() > policy.max_error_rad)
            });
        if let Some((joint, _)) = outside.filter(|_| torqued) {
            self.elapsed_s += dt_s;
            if self.elapsed_s + 1e-12 >= policy.persistence_ms as f64 / 1000.0 {
                self.fault_joint = Some(joint);
            }
        } else {
            self.elapsed_s = 0.0;
        }
        self.fault_joint
    }
}

struct WasmEngine {
    config: ArmConfig,
    profile_keepouts: Vec<KeepoutBox>,
    generation: u64,
    workspace_revision: u64,
    next_path_id: u64,
    paths: VecDeque<(u64, StoredPath)>,
    simulation_s: f64,
    following_error: FollowingErrorMonitor,
    collision: CollisionGuard,
    collision_fault: bool,
    torque_guard: TorqueGuard,
}

impl Default for WasmEngine {
    fn default() -> Self {
        let mut config = ArmConfig::rebot_dm();
        let _ = config.validate();
        Self {
            profile_keepouts: config
                .workspace
                .as_ref()
                .map(|w| w.keepout_boxes.clone())
                .unwrap_or_default(),
            collision: CollisionGuard::from_policy(&config.collision.unwrap_or_default()),
            config,
            generation: 0,
            workspace_revision: 0,
            next_path_id: 1,
            paths: VecDeque::new(),
            simulation_s: 0.0,
            following_error: FollowingErrorMonitor::default(),
            collision_fault: false,
            torque_guard: TorqueGuard::default(),
        }
    }
}

thread_local! { static ENGINE: RefCell<WasmEngine> = RefCell::new(WasmEngine::default()); }

fn invalid<T>(message: impl Into<String>) -> Result<T, MotionError> {
    Err(MotionError::InvalidCommand(message.into()))
}

fn finite(values: &[f64], name: &str) -> Result<(), MotionError> {
    if values.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        invalid(format!("{name} must contain finite numbers"))
    }
}

fn finite_f32(values: &[f32], name: &str) -> Result<(), MotionError> {
    if values.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        invalid(format!("{name} must contain finite f32 numbers"))
    }
}

fn error_json(code: &str, message: impl std::fmt::Display) -> Value {
    json!({"ok": false, "code": code, "message": message.to_string()})
}

fn motion_error_json(error: &MotionError) -> Value {
    let mut v = error_json(error.code(), error);
    if let MotionError::WorkspaceViolation { detail, .. } = error {
        if let Some(c) = &detail.constraint_id { v["constraint_id"] = json!(c); }
        if let Some(g) = &detail.geometry_id { v["geometry_id"] = json!(g); }
        if let Some(p) = &detail.violation_phase { v["violation_phase"] = json!(p); }
        if let Some(r) = &detail.reason { v["reason"] = json!(r); }
    }
    v
}

/// Dispatch one request using thread-local simulation state. No filesystem or network I/O.
pub fn dispatch_json(req_json: &str) -> String {
    std::panic::catch_unwind(|| {
        let request = match serde_json::from_str::<Request>(req_json) {
            Ok(request) => request,
            Err(error) => return error_json("INVALID_COMMAND", error),
        };
        ENGINE.with(|engine| match engine.borrow_mut().dispatch(request) {
            Ok(value) => value,
            Err(error) => motion_error_json(&error),
        })
    })
    .unwrap_or_else(|_| error_json("INTERNAL", "core request panicked"))
    .to_string()
}

impl WasmEngine {
    fn reset_guards(&mut self) {
        self.simulation_s = 0.0;
        self.following_error = FollowingErrorMonitor::default();
        self.collision = CollisionGuard::from_policy(&self.config.collision.unwrap_or_default());
        self.collision_fault = false;
        self.torque_guard = TorqueGuard::default();
    }

    fn invalidate_paths(&mut self) {
        self.workspace_revision += 1;
        self.paths.clear();
    }

    fn dispatch(&mut self, request: Request) -> Result<Value, MotionError> {
        match request {
            Request::Version => Ok(
                json!({"ok": true, "armd_version": env!("CARGO_PKG_VERSION"), "core_version": 1}),
            ),
            Request::InitProfile {
                robot,
                config_json,
                urdf_xml,
            } => {
                let robot = robot.as_deref().unwrap_or("b601dm");
                if !matches!(robot, "b601dm" | "so-arm101") {
                    return invalid("unknown robot profile");
                }
                let custom_config = config_json.is_some();
                let mut config: ArmConfig = match config_json {
                    Some(value) => serde_json::from_value(value)
                        .map_err(|e| MotionError::InvalidCommand(e.to_string()))?,
                    None if robot == "so-arm101" => {
                        serde_json::from_str(include_str!("../profiles/so_arm101.json"))
                            .map_err(|e| MotionError::InvalidCommand(e.to_string()))?
                    }
                    None => ArmConfig::rebot_dm(),
                };
                let urdf = urdf_xml.as_deref().or_else(|| {
                    if robot == "so-arm101" && !custom_config {
                        Some(include_str!("../assets/so101_new_calib.urdf"))
                    } else {
                        None
                    }
                });
                config
                    .init_arm_model_in_memory(urdf)
                    .map_err(MotionError::InvalidCommand)?;
                config.validate().map_err(MotionError::InvalidCommand)?;
                if config.dof > 7 || config.arm_dof() > 6 {
                    return invalid(
                        "JSON core supports at most six arm joints and seven motor joints",
                    );
                }
                self.profile_keepouts = config
                    .workspace
                    .as_ref()
                    .map(|w| w.keepout_boxes.clone())
                    .unwrap_or_default();
                self.config = config;
                self.generation += 1;
                self.invalidate_paths();
                self.reset_guards();
                let c = &self.config;
                let offsets: Vec<f64> = c
                    .arm_model
                    .joint_map()
                    .iter()
                    .map(|j| j.offset_rad)
                    .collect();
                Ok(json!({"ok": true, "profile": {
                    "dof": c.dof, "arm_dof": c.arm_dof(), "model_offsets_rad": offsets,
                    "limits": c.limits, "workspace": c.workspace, "camera_mount": c.camera_mount,
                    "speed_cap": c.speed_cap,
                    "action_speed_max": {"linear_mps": motion::MAX_LINEAR_MPS, "angular_rps": motion::MAX_ANGULAR_RPS},
                    "following_error": c.following_error, "collision": c.collision, "torque_guard": c.torque_guard
                }}))
            }
            Request::SetKeepout { boxes } => {
                if self.config.kinematics.is_none() || self.config.workspace.is_none() {
                    return Err(MotionError::Unsupported(
                        "keep-out boxes need kinematics and a workspace".into(),
                    ));
                }
                KeepoutBox::validate_list(&boxes).map_err(MotionError::InvalidCommand)?;
                let effective: Vec<_> = self
                    .profile_keepouts
                    .iter()
                    .chain(&boxes)
                    .cloned()
                    .collect();
                KeepoutBox::validate_list(&effective).map_err(MotionError::InvalidCommand)?;
                self.config
                    .workspace
                    .as_mut()
                    .expect("workspace checked")
                    .keepout_boxes = effective.clone();
                self.invalidate_paths();
                Ok(
                    json!({"ok": true, "workspace_revision": self.workspace_revision, "effective_boxes": effective}),
                )
            }
            Request::SetToolEnvelope {
                tcp_offset,
                extra_check_points_end_m,
                permanent_tool_check_points_end_m,
                transient_payload_check_points_end_m,
            } => {
                if let Some(off) = &tcp_offset {
                    finite(off, "tcp_offset")?;
                }
                for (name, pts_opt) in [
                    ("extra_check_points_end_m", &extra_check_points_end_m),
                    ("permanent_tool_check_points_end_m", &permanent_tool_check_points_end_m),
                    ("transient_payload_check_points_end_m", &transient_payload_check_points_end_m),
                ] {
                    if let Some(pts) = pts_opt {
                        for p in pts {
                            finite_f32(p, name)?;
                        }
                    }
                }
                if self.config.kinematics.is_none() || self.config.workspace.is_none() {
                    return Err(MotionError::Unsupported(
                        "tool envelope needs kinematics and a workspace".into(),
                    ));
                }
                let mut candidate = self.config.clone();
                if let Some(off) = tcp_offset {
                    candidate.tcp_offset = off;
                }
                if let Some(perm) = permanent_tool_check_points_end_m {
                    candidate.permanent_tool_check_points_end_m = perm;
                }
                if let Some(payload) = transient_payload_check_points_end_m {
                    candidate.transient_payload_check_points_end_m = payload;
                } else if let Some(extra) = extra_check_points_end_m {
                    candidate.transient_payload_check_points_end_m = extra;
                }
                candidate
                    .rebuild_effective_check_points()
                    .map_err(MotionError::InvalidCommand)?;
                if candidate.tcp_offset != [0.0; 3] || !candidate.permanent_tool_check_points_end_m.is_empty() {
                    let robot_id = candidate
                        .profile_id
                        .as_deref()
                        .and_then(crate::tool::normalize_robot_id)
                        .unwrap_or("unknown");
                    let (max_w, grasp_h) = crate::tool::baseline_grasp_limits(&candidate);
                    let edge_to_low = candidate.calibration_tool.as_ref().and_then(|c| c.edge_to_low_m);
                    candidate.active_tool = crate::tool::ActiveToolInfo {
                        schema_version: 1,
                        id: "custom_envelope".into(),
                        is_stock: false,
                        kind: "gripper".into(),
                        tcp_offset_m: candidate.tcp_offset,
                        config_digest: crate::tool::compute_tool_config_digest(
                            robot_id,
                            "custom_envelope",
                            "gripper",
                            candidate.tcp_offset,
                            &candidate.permanent_tool_check_points_end_m,
                            candidate.physical_contact_end_m,
                            candidate.contact_opening_m,
                            None,
                            candidate.gripper_policy.as_ref(),
                            max_w,
                            grasp_h,
                            edge_to_low,
                        ),
                        pack_digest: None,
                        commissioning_required: false,
                    };
                } else {
                    candidate.active_tool = crate::tool::stock_active_tool_info(&candidate);
                }
                candidate.validate().map_err(MotionError::InvalidCommand)?;
                if self.config.tcp_offset != candidate.tcp_offset
                    || self.config.permanent_tool_check_points_end_m != candidate.permanent_tool_check_points_end_m
                    || self.config.transient_payload_check_points_end_m != candidate.transient_payload_check_points_end_m
                    || self.config.workspace.as_ref().map(|w| &w.check_points_end_m)
                        != candidate.workspace.as_ref().map(|w| &w.check_points_end_m)
                {
                    self.config = candidate;
                    self.invalidate_paths();
                }
                Ok(json!({"ok": true, "workspace_revision": self.workspace_revision}))
            }
            Request::FkPose {
                q_motor,
                include_tool_tcp,
            } => {
                finite(&q_motor, "q_motor")?;
                let end = self.config.arm_model.fk(&q_motor);
                let pose = if include_tool_tcp.unwrap_or(false) {
                    self.config.end_to_tcp(&end)
                } else {
                    end
                };
                Ok(
                    json!({"ok": true, "xyz": pose.p, "rpy": k::r_to_rpy(&pose.r), "rot_mat": pose.r}),
                )
            }
            Request::PlanCartesian {
                q0_motor,
                j7,
                goal,
                speed_mps,
                angular_rps,
                speed_scale,
                auto_yaw,
                include_tool_tcp,
            } => {
                finite(&q0_motor, "q0_motor")?;
                finite_f32(&[j7], "j7")?;
                if self.config.kinematics.is_none() || self.config.dof != self.config.arm_dof() + 1
                {
                    return Err(MotionError::Unsupported(
                        "Cartesian planning needs an arm and a gripper".into(),
                    ));
                }
                let scale = speed_scale.unwrap_or(1.0);
                if !scale.is_finite() || scale <= 0.0 {
                    return invalid("speed_scale must be positive and finite");
                }
                let goal_frame = if include_tool_tcp.unwrap_or(false) {
                    GoalFrame::Tcp
                } else {
                    GoalFrame::EndLink
                };
                let input = PlanInput {
                    q0: q0_motor,
                    j7,
                    limits: self.planning_limits(),
                    speed_cap: self.config.speed_cap,
                    ws: self.config.workspace.clone(),
                    cam: self.config.camera_mount.clone(),
                    arm: self.config.arm_model.clone(),
                    epoch: self.generation,
                    workspace_revision: self.workspace_revision,
                    power: self.config.power_budget.clone(),
                    tcp_offset: self.config.tcp_offset,
                    goal_frame,
                };
                let r = goal.orientation()?;
                let spec = match auto_yaw {
                    Some(auto_yaw) => GoalSpec::AbsoluteEx {
                        p: goal.xyz,
                        r,
                        auto_yaw,
                    },
                    None => GoalSpec::Absolute { p: goal.xyz, r },
                };
                let (target, opts) = motion::resolve_goal_opts(&input, &spec);
                let path = motion::plan_cartesian_opts(
                    &input,
                    target,
                    Speed {
                        linear: speed_mps.unwrap_or(motion::DEFAULT_LINEAR_MPS) * scale,
                        angular: angular_rps.unwrap_or(motion::DEFAULT_ANGULAR_RPS),
                    },
                    opts,
                )?;
                let id = self.next_path_id;
                self.next_path_id += 1;
                let response = json!({"ok": true, "path_id": id, "generation": self.generation,
                    "workspace_revision": self.workspace_revision, "duration_s": path.duration,
                    "power_scale": path.power_scale, "samples": path.q.len(), "last_q": path.last()});
                self.paths.push_back((
                    id,
                    StoredPath {
                        path,
                        generation: self.generation,
                        workspace_revision: self.workspace_revision,
                    },
                ));
                if self.paths.len() > MAX_PATHS {
                    self.paths.pop_front();
                }
                Ok(response)
            }
            Request::SamplePath { path_id, t_s } => {
                if !t_s.is_finite() || t_s < 0.0 {
                    return invalid("t_s must be finite and nonnegative");
                }
                let Some((_, stored)) = self.paths.iter().find(|(id, _)| *id == path_id) else {
                    return Ok(error_json("PLAN_STALE", "path was invalidated or evicted"));
                };
                if stored.generation != self.generation
                    || stored.workspace_revision != self.workspace_revision
                {
                    return Ok(error_json("PLAN_STALE", "profile or workspace changed"));
                }
                Ok(
                    json!({"ok": true, "q_motor": stored.path.position_at_time(t_s),
                    "done": t_s >= stored.path.duration, "progress": (t_s / stored.path.duration).clamp(0.0, 1.0)}),
                )
            }
            Request::CheckWorkspace { q_motor } => {
                finite_f32(&q_motor, "q_motor")?;
                let Some(ws) = self
                    .config
                    .workspace
                    .as_ref()
                    .filter(|_| self.config.kinematics.is_some())
                else {
                    return Err(MotionError::Unsupported(
                        "workspace check is disabled".into(),
                    ));
                };
                let violation = self.config.arm_model.workspace_violation_with_tcp(
                    &q_motor,
                    ws,
                    &self.config.camera_mount,
                    self.config.tcp_offset,
                );
                Ok(json!({"ok": true, "violation_total": violation.total,
                    "violated": violation.total > k::WORKSPACE_EPS,
                    "worst": violation.worst.map(|w| json!({"point": w.point, "limit": w.label(ws),
                        "value": w.value, "bound": w.bound, "keepout": w.keepout}))}))
            }
            Request::ResetGuards => {
                self.reset_guards();
                Ok(json!({"ok": true}))
            }
            Request::StepGuards {
                dt_s,
                torqued,
                q_cmd,
                q_meas,
                torques_nm,
                collision_enabled,
                collision_sensitivity,
            } => self.step_guards(
                dt_s,
                torqued,
                &q_cmd,
                &q_meas,
                &torques_nm,
                collision_enabled,
                collision_sensitivity,
            ),
            Request::ComposeMjcf {
                base_xml,
                scene_world_xml,
                tool_manifest,
                tool_xml,
                tool_preset,
            } => {
                let preset = match tool_preset.as_deref() {
                    Some(id) => {
                        Some(crate::mjcf_compose::builtin_tool_preset(id).ok_or_else(|| {
                            MotionError::InvalidCommand(format!("unknown tool preset '{id}'"))
                        })?)
                    }
                    None => None,
                };
                let manifest = tool_manifest
                    .as_deref()
                    .or_else(|| preset.as_ref().map(|p| &p.0));
                let xml = tool_xml.as_deref().or_else(|| preset.as_ref().map(|p| p.1));
                let xml = crate::mjcf_compose::compose_mjcf(
                    base_xml
                        .as_deref()
                        .unwrap_or(crate::mjcf_compose::FALLBACK_BASE_XML),
                    scene_world_xml.as_deref(),
                    manifest,
                    xml,
                )
                .map_err(MotionError::InvalidCommand)?;
                Ok(json!({"ok": true, "xml": xml}))
            }
            Request::ResolveToolConfig { manifest, robot_id } => {
                let summary = crate::tool::resolve_tool_config_summary(
                    &manifest,
                    robot_id.as_deref(),
                )
                .map_err(MotionError::InvalidCommand)?;
                serde_json::to_value(summary)
                    .map_err(|e| MotionError::InvalidCommand(e.to_string()))
            }
        }
    }

    fn step_guards(
        &mut self,
        dt_s: f64,
        torqued: bool,
        q_cmd: &[f32],
        q_meas: &[f32],
        torques: &[f32],
        collision_enabled: bool,
        sensitivity: Option<Sensitivity>,
    ) -> Result<Value, MotionError> {
        if !dt_s.is_finite() || dt_s <= 0.0 || !(self.simulation_s + dt_s).is_finite() {
            return invalid("dt_s must be positive and finite");
        }
        for (name, values) in [
            ("q_cmd", q_cmd),
            ("q_meas", q_meas),
            ("torques_nm", torques),
        ] {
            if values.len() != self.config.dof {
                return invalid(format!("{name} needs {} values", self.config.dof));
            }
            finite_f32(values, name)?;
        }
        let available = self.config.gravity_model.is_some() && self.config.arm_dof() == 6;
        if collision_enabled && !available {
            return Err(MotionError::Unsupported(
                "collision guard needs a six-joint arm with a gravity model".into(),
            ));
        }
        let residual = if collision_enabled {
            let gravity = self
                .config
                .arm_model
                .gravity_torques(q_meas)
                .map_err(|e| MotionError::InvalidCommand(e.to_string()))?;
            Some(std::array::from_fn(|j| torques[j] - gravity[j]))
        } else {
            None
        };
        self.simulation_s += dt_s;
        let following = self.following_error.step(
            dt_s,
            torqued && !self.collision_fault,
            q_cmd,
            q_meas,
            self.config.gripper_index(),
            &self.config.following_error,
        );
        let operational = torqued && following.is_none() && !self.collision_fault;
        let policy = self.config.collision.unwrap_or_default();
        self.collision.enabled = collision_enabled;
        if let Some(sensitivity) = sensitivity {
            self.collision.sensitivity = sensitivity;
        }
        let supported = self.supported_joints(q_meas, &policy);
        let collision = self.collision.step(
            residual.as_ref(),
            dt_s,
            self.simulation_s,
            &policy,
            operational,
            &supported,
        );
        self.collision_fault |= collision.escalated;
        if let Some(policy) = self.config.torque_guard {
            let thresholds = policy.thresholds(&self.config);
            self.torque_guard.step(
                torques,
                &thresholds,
                dt_s,
                self.simulation_s,
                &policy,
                operational && !self.collision_fault,
                operational && !self.collision_fault,
                q_meas,
                q_cmd,
                Some(q_cmd),
            );
            if let Some(hold) = self.torque_guard.hold.as_mut() {
                let limit = &self.config.limits[hold.joint];
                hold.hold_q = hold.hold_q.clamp(limit.min_position, limit.max_position);
            }
        }
        let fault = if following.is_some() {
            Some("FOLLOWING_ERROR")
        } else if self.collision_fault {
            Some("COLLISION")
        } else {
            None
        };
        Ok(json!({"ok": true, "following_error": following.is_some(),
            "following_error_joint": following, "collision": self.collision_fault || self.collision.hold.is_some(),
            "collision_fault": self.collision_fault, "collision_hold": self.collision.hold.is_some(),
            "torque_hold": self.torque_guard.hold.is_some(), "fault": fault,
            "code": fault, "simulation_s": self.simulation_s}))
    }

    // Native plan_input applies these profile caps before entering the shared planner.
    fn planning_limits(&self) -> Vec<crate::model::JointLimit> {
        let mut limits = self.config.limits.clone();
        let scale = self
            .config
            .speed_cap
            .and_then(|cap| cap.joint_velocity_scale)
            .unwrap_or(1.0);
        for (joint, limit) in limits.iter_mut().enumerate() {
            if Some(joint) != self.config.gripper_index() {
                if let Some(cap) = self.config.planned_max_velocity {
                    limit.max_velocity = limit.max_velocity.min(cap);
                }
                limit.max_velocity *= scale;
            }
        }
        limits
    }

    fn supported_joints(&self, q: &[f32], policy: &CollisionPolicy) -> [bool; 6] {
        let mut supported = [false; 6];
        let Some(rest) = self.config.resting_pose.as_ref() else {
            return supported;
        };
        let n = self.config.resting_arm_joints();
        let home = self.config.resting_positions();
        if n > 0
            && (0..n).all(
                |j| match (q.get(j), home.get(j), rest.position_tolerances.get(j)) {
                    (Some(q), Some(home), Some(tolerance)) => (q - home).abs() <= *tolerance,
                    _ => false,
                },
            )
        {
            if let Some(exempt) = policy.rest_exempt_joints {
                supported = exempt;
            } else {
                for &(j, _) in &rest.support_torque_limits {
                    if j < 6 {
                        supported[j] = true;
                    }
                }
            }
        }
        supported
    }
}

/// Allocate an exact byte slice. Ownership passes to the caller until `armd_dealloc`.
#[cfg(target_arch = "wasm32")]
#[no_mangle]
pub extern "C" fn armd_alloc(len: usize) -> *mut u8 {
    Box::into_raw(vec![0u8; len].into_boxed_slice()).cast::<u8>()
}

/// # Safety
/// `ptr` and `len` must identify a live allocation returned by `armd_alloc`,
/// and must be freed exactly once. A null pointer is ignored.
#[cfg(target_arch = "wasm32")]
#[no_mangle]
pub unsafe extern "C" fn armd_dealloc(ptr: *mut u8, len: usize) {
    if !ptr.is_null() {
        drop(unsafe { Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)) });
    }
}

/// # Safety
/// `ptr` must identify `len` readable bytes in WASM memory for this call.
/// A null pointer is accepted only for an empty request.
#[cfg(target_arch = "wasm32")]
#[no_mangle]
pub unsafe extern "C" fn armd_call_json(ptr: *const u8, len: usize) -> u64 {
    let response = std::panic::catch_unwind(|| {
        if ptr.is_null() && len != 0 {
            return error_json("INVALID_COMMAND", "null request pointer").to_string();
        }
        let bytes = if len == 0 {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(ptr, len) }
        };
        match std::str::from_utf8(bytes) {
            Ok(request) => dispatch_json(request),
            Err(error) => error_json("INVALID_COMMAND", error).to_string(),
        }
    })
    .unwrap_or_else(|_| error_json("INTERNAL", "core request panicked").to_string());
    let bytes = response.into_bytes().into_boxed_slice();
    let len = bytes.len();
    let ptr = Box::into_raw(bytes).cast::<u8>();
    ((ptr as u32 as u64) << 32) | len as u32 as u64
}

/// # Safety
/// `ptr` and `len` must be the result pair from `armd_call_json`, freed exactly once.
#[cfg(target_arch = "wasm32")]
#[no_mangle]
pub unsafe extern "C" fn armd_free_result(ptr: *mut u8, len: usize) {
    if !ptr.is_null() {
        drop(unsafe { Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];

    fn call(request: Value) -> Value {
        serde_json::from_str(&dispatch_json(&request.to_string())).unwrap()
    }

    fn ok(request: Value) -> Value {
        let result = call(request);
        assert_eq!(result["ok"], true, "{result}");
        result
    }

    fn init() -> Value {
        ok(json!({"op":"init_profile", "robot":"b601dm"}))
    }

    fn init_config(mut config: ArmConfig) -> Value {
        config.validate().unwrap();
        ENGINE.with(|e| {
            let mut eng = e.borrow_mut();
            eng.profile_keepouts = config
                .workspace
                .as_ref()
                .map(|w| w.keepout_boxes.clone())
                .unwrap_or_default();
            eng.config = config;
            eng.generation += 1;
            eng.invalidate_paths();
            eng.reset_guards();
        });
        json!({"ok": true})
    }

    fn xyz(value: &Value) -> [f64; 3] {
        serde_json::from_value(value["xyz"].clone()).unwrap()
    }

    fn pose(tcp: bool) -> Value {
        ok(json!({"op":"fk_pose", "q_motor":INSPECT, "include_tool_tcp":tcp}))
    }

    fn plan_to(target: [f64; 3], tcp: bool) -> Value {
        call(json!({"op":"plan_cartesian", "q0_motor":INSPECT, "j7":0.0,
            "goal":{"xyz":target,"rpy":null,"mat":null}, "include_tool_tcp":tcp}))
    }

    fn assert_code(result: Value, code: &str) {
        assert_eq!(result["ok"], false, "{result}");
        assert_eq!(result["code"], code, "{result}");
    }

    #[test]
    fn version_and_default_j6_profile_contract() {
        let version = ok(json!({"op":"version"}));
        assert_eq!(version["armd_version"], env!("CARGO_PKG_VERSION"));
        assert_eq!(version["core_version"], 1);
        let result = init();
        assert_eq!(result["profile"]["dof"], 7);
        assert_eq!(result["profile"]["arm_dof"], 6);
        assert_eq!(result["profile"]["action_speed_max"]["linear_mps"], 0.15);
        let offset = result["profile"]["model_offsets_rad"][5].as_f64().unwrap();
        assert_eq!(offset, 0.0, "official convention: URDF J6 = joint J6");
        let result = ok(json!({"op":"init_profile","robot":null}));
        assert_eq!(result["profile"]["model_offsets_rad"][5], offset);
    }

    #[test]
    fn so101_initializes_from_embedded_profile_and_urdf() {
        let result = ok(json!({"op":"init_profile","robot":"so-arm101"}));
        assert_eq!(result["profile"]["dof"], 6);
        assert_eq!(result["profile"]["arm_dof"], 5);
        let position = ok(json!({"op":"fk_pose","q_motor":[0.0,-0.2,0.5,0.5,0.0,0.0]}));
        assert!(xyz(&position).iter().all(|x| x.is_finite()));
        // Index 5, rather than index 6, is this profile's gripper.
        for _ in 0..20 {
            let result = ok(json!({"op":"step_guards","dt_s":0.1,"torqued":true,
                "q_cmd":[0.0,0.0,0.0,0.0,0.0,10.0],"q_meas":[0.0,0.0,0.0,0.0,0.0,0.0],
                "torques_nm":[0.0,0.0,0.0,0.0,0.0,0.0],"collision_enabled":false}));
            assert_eq!(result["following_error"], false);
        }
    }

    #[test]
    fn fk_plan_sample_roundtrip_preserves_motor_offset_and_tool_tcp() {
        init();
        let flange = pose(false);
        let offset = [0.03, 0.02, 0.01];
        let envelope =
            ok(json!({"op":"set_tool_envelope","tcp_offset":offset,"extra_check_points_end_m":[]}));
        let same =
            ok(json!({"op":"set_tool_envelope","tcp_offset":offset,"extra_check_points_end_m":[]}));
        assert_eq!(envelope["workspace_revision"], same["workspace_revision"]);
        let tcp = pose(true);
        let rotation: Mat3 = serde_json::from_value(flange["rot_mat"].clone()).unwrap();
        let expected = k::add(&xyz(&flange), &k::mat_vec(&rotation, &offset));
        assert!(k::norm(&k::sub(&xyz(&tcp), &expected)) < 1e-12);
        let mut target = xyz(&tcp);
        target[0] += 0.02;
        let plan = ok(json!({"op":"plan_cartesian","q0_motor":INSPECT,"j7":0.0,
            "goal":{"xyz":target,"mat":tcp["rot_mat"]},"include_tool_tcp":true,"auto_yaw":true}));
        let start = ok(json!({"op":"sample_path","path_id":plan["path_id"],"t_s":0.0}));
        assert_eq!(start["done"], false);
        let start_q: [f32; 7] = serde_json::from_value(start["q_motor"].clone()).unwrap();
        for j in 0..6 {
            assert!((f64::from(start_q[j]) - INSPECT[j]).abs() < 1e-6);
        }
        let duration = plan["duration_s"].as_f64().unwrap();
        let middle = ok(json!({"op":"sample_path","path_id":plan["path_id"],"t_s":duration/2.0}));
        assert!((middle["progress"].as_f64().unwrap() - 0.5).abs() < 1e-12);
        let last = ok(json!({"op":"sample_path","path_id":plan["path_id"],"t_s":duration}));
        assert_eq!(last["done"], true);
        assert_eq!(last["progress"], 1.0);
        let q: [f32; 7] = serde_json::from_value(last["q_motor"].clone()).unwrap();
        let q6: [f64; 6] = std::array::from_fn(|j| f64::from(q[j]));
        let reached = ok(json!({"op":"fk_pose","q_motor":q6,"include_tool_tcp":true}));
        assert!(
            k::norm(&k::sub(&xyz(&reached), &target)) < 1e-3,
            "{reached} target={target:?}"
        );
        let reached_flange = ok(json!({"op":"fk_pose","q_motor":q6}));
        assert!(k::norm(&k::sub(&xyz(&reached_flange), &target)) > 0.03);
    }

    #[test]
    fn carried_pcb_corner_enters_divider_wall() {
        init();
        let start = pose(false);
        let mut target = xyz(&start);
        target[0] += 0.03;
        let r: Mat3 = serde_json::from_value(start["rot_mat"].clone()).unwrap();
        let corner = [0.0f32, 0.20, 0.0];
        let center = k::add(&target, &k::mat_vec(&r, &corner.map(f64::from)));
        let min = center.map(|v| (v - 0.004) as f32);
        let max = center.map(|v| (v + 0.004) as f32);
        ok(json!({"op":"set_keepout","boxes":[{"name":"divider_wall","min_m":min,"max_m":max}]}));
        let bare = plan_to(target, false);
        assert_eq!(bare["ok"], true, "bare gripper must clear the wall: {bare}");
        ok(
            json!({"op":"set_tool_envelope","tcp_offset":[0.0,0.0,0.0],"extra_check_points_end_m":[corner]}),
        );
        let carrying = plan_to(target, false);
        assert_code(carrying.clone(), "WORKSPACE_VIOLATION");
        assert!(carrying["message"]
            .as_str()
            .unwrap()
            .contains("divider_wall"));
        ok(
            json!({"op":"set_tool_envelope","tcp_offset":[0.0,0.0,0.0],"extra_check_points_end_m":[]}),
        );
        assert_eq!(plan_to(target, false)["ok"], true);
    }

    #[test]
    fn planning_reports_unreachable_floor_keepout_and_speed_errors() {
        init();
        let start = xyz(&pose(false));
        let unreachable = [start[0] + 0.5, start[1], start[2]];
        assert_code(plan_to(unreachable, false), "IK_UNREACHABLE");
        let floor = plan_to([start[0], start[1], -0.02], false);
        assert_code(floor.clone(), "WORKSPACE_VIOLATION");
        assert!(floor["message"].as_str().unwrap().contains("floor"));
        assert_code(
            call(json!({"op":"plan_cartesian","q0_motor":INSPECT,"j7":0.0,
            "goal":{"xyz":start},"speed_mps":0.151})),
            "INVALID_COMMAND",
        );
        assert_code(
            call(json!({"op":"plan_cartesian","q0_motor":INSPECT,"j7":0.0,
            "goal":{"xyz":start},"speed_mps":0.1,"speed_scale":2.0})),
            "INVALID_COMMAND",
        );
        let center = [start[0] + 0.025, start[1], start[2]];
        ok(json!({"op":"set_keepout","boxes":[{"name":"divider_wall",
            "min_m":center.map(|x| x-0.004),"max_m":center.map(|x| x+0.004)}]}));
        let wall = plan_to([start[0] + 0.05, start[1], start[2]], false);
        assert_code(wall.clone(), "WORKSPACE_VIOLATION");
        assert!(wall["message"].as_str().unwrap().contains("divider_wall"));
    }

    #[test]
    fn keepout_union_revision_and_stale_paths() {
        let mut config = ArmConfig::rebot_dm();
        config
            .workspace
            .as_mut()
            .unwrap()
            .keepout_boxes
            .push(KeepoutBox {
                name: "profile_box".into(),
                min_m: [-0.59, -0.59, 0.01],
                max_m: [-0.58, -0.58, 0.02],
            });
        init_config(config);
        let target = xyz(&pose(false));
        let plan = plan_to(target, false);
        assert_eq!(plan["ok"], true, "{plan}");
        let added = ok(json!({"op":"set_keepout","boxes":[{"name":"runtime_box",
            "min_m":[-0.57,-0.57,0.01],"max_m":[-0.56,-0.56,0.02]}]}));
        assert!(
            added["workspace_revision"].as_u64().unwrap()
                > plan["workspace_revision"].as_u64().unwrap()
        );
        assert_eq!(added["effective_boxes"].as_array().unwrap().len(), 2);
        let cleared = ok(json!({"op":"set_keepout","boxes":[]}));
        assert_eq!(cleared["effective_boxes"].as_array().unwrap().len(), 1);
        assert_eq!(cleared["effective_boxes"][0]["name"], "profile_box");
        assert_code(
            call(json!({"op":"sample_path","path_id":plan["path_id"],"t_s":0.0})),
            "PLAN_STALE",
        );
        let fresh = plan_to(target, false);
        assert_eq!(fresh["ok"], true, "{fresh}");
        init();
        assert_code(
            call(json!({"op":"sample_path","path_id":fresh["path_id"],"t_s":0.0})),
            "PLAN_STALE",
        );
    }

    #[test]
    fn guard_simulated_persistence_latches_and_excludes_gripper() {
        let profile = init();
        let dt = profile["profile"]["following_error"]["persistence_ms"]
            .as_f64()
            .unwrap()
            / 1000.0
            / 5.0;
        let mut q_cmd = [0.0f32; 7];
        q_cmd[6] = 10.0;
        let request = |q: [f32; 7], torqued| {
            json!({"op":"step_guards","dt_s":dt,"torqued":torqued,
            "q_cmd":q,"q_meas":[0.0,0.0,0.0,0.0,0.0,0.0,0.0],
            "torques_nm":[0.0,0.0,0.0,0.0,0.0,0.0,0.0],"collision_enabled":false})
        };
        for _ in 0..10 {
            assert_eq!(ok(request(q_cmd, true))["following_error"], false);
        }
        q_cmd[0] = 1.0;
        for _ in 0..10 {
            assert_eq!(ok(request(q_cmd, false))["following_error"], false);
        }
        for _ in 0..4 {
            assert_eq!(ok(request(q_cmd, true))["following_error"], false);
        }
        let fault = ok(request(q_cmd, true));
        assert_eq!(fault["following_error"], true);
        assert_eq!(fault["code"], "FOLLOWING_ERROR");
        assert_eq!(
            ok(request([0.0; 7], true))["following_error"],
            true,
            "fault stays latched"
        );
        ok(json!({"op":"reset_guards"}));
        assert_eq!(ok(request([0.0; 7], true))["following_error"], false);
    }

    #[test]
    fn collision_uses_gravity_residual_resume_and_escalation() {
        let mut config = ArmConfig::rebot_dm();
        let policy = CollisionPolicy {
            enabled: true,
            thresholds_nm: [1.0; 6],
            persist_ticks: 2,
            resume_after_s: 0.2,
            max_auto_resumes: 1,
            ..Default::default()
        };
        config.collision = Some(policy);
        config.torque_guard = None;
        let measured: [f32; 7] =
            std::array::from_fn(|j| if j < 6 { INSPECT[j] as f32 } else { 0.0 });
        let gravity = config.arm_model.gravity_torques(&measured).unwrap();
        let quiet: [f32; 7] = std::array::from_fn(|j| gravity[j]);
        let mut contact = quiet;
        contact[0] += 2.0;
        init_config(config);
        let step = |tau: [f32; 7]| {
            json!({"op":"step_guards","dt_s":0.1,"torqued":true,
            "q_cmd":measured,"q_meas":measured,"torques_nm":tau,"collision_enabled":true})
        };
        for _ in 0..5 {
            assert_eq!(ok(step(quiet))["collision"], false);
        }
        assert_eq!(ok(step(contact))["collision"], false);
        assert_eq!(ok(step(contact))["collision_hold"], true);
        assert_eq!(ok(step(quiet))["collision_hold"], true);
        assert_eq!(ok(step(quiet))["collision_hold"], false);
        ok(step(contact));
        let fault = ok(step(contact));
        assert_eq!(fault["collision_fault"], true);
        assert_eq!(fault["code"], "COLLISION");
        assert_eq!(ok(step(quiet))["collision_fault"], true);
    }

    #[test]
    fn compose_pcb_scene_and_suction_without_io() {
        let result = ok(
            json!({"op":"compose_mjcf", "base_xml":crate::mjcf_compose::FALLBACK_BASE_XML,
            "scene_world_xml":crate::mjcf_compose::PRESET_PCB_SUCTION_XML,"tool_preset":"suction"}),
        );
        let xml = result["xml"].as_str().unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        assert!(doc
            .descendants()
            .any(|n| n.attribute("name") == Some("divider_wall")));
        assert!(doc
            .descendants()
            .any(|n| n.attribute("name") == Some("m7_suction")));
        assert!(!doc
            .descendants()
            .any(|n| n.attribute("name") == Some("finger_left")));
        let expected_tool = crate::mjcf_compose::builtin_tool_preset("suction").unwrap();
        let direct = crate::mjcf_compose::compose_mjcf(
            crate::mjcf_compose::FALLBACK_BASE_XML,
            Some(crate::mjcf_compose::PRESET_PCB_SUCTION_XML),
            Some(&expected_tool.0),
            Some(expected_tool.1),
        )
        .unwrap();
        assert_eq!(xml, direct);
    }

    #[test]
    fn invalid_requests_do_not_mutate_state_or_silently_coerce_values() {
        init();
        let original = pose(false);
        assert_code(
            call(json!({"op":"init_profile","robot":"unknown"})),
            "INVALID_COMMAND",
        );
        assert_eq!(pose(false), original);
        assert_code(
            call(json!({"op":"fk_pose","q_motor":["bad",0,0,0,0,0]})),
            "INVALID_COMMAND",
        );
        assert_code(
            call(json!({"op":"fk_pose","q_motor":INSPECT,"include_tool_tcp":"yes"})),
            "INVALID_COMMAND",
        );
        assert_code(
            call(json!({"op":"compose_mjcf","tool_preset":"missing"})),
            "INVALID_COMMAND",
        );
        assert_code(
            call(json!({"op":"compose_mjcf","tool_manifest":{"invalid":true}})),
            "INVALID_COMMAND",
        );
        assert_code(
            call(json!({"op":"set_tool_envelope","tcp_offset":[0.2,0.0,0.0],
            "extra_check_points_end_m":(0..30).map(|i| [0.0,0.3,i as f32 * 0.01]).collect::<Vec<_>>()})),
            "INVALID_COMMAND",
        );
        assert_eq!(pose(true), original, "invalid envelope must not update TCP");
        let target = xyz(&original);
        assert_code(
            call(json!({"op":"plan_cartesian","q0_motor":INSPECT,"j7":0.0,
            "goal":{"xyz":target,"mat":[[1,0,0],[0,2,0],[0,0,1]]}})),
            "INVALID_COMMAND",
        );
        let plan = plan_to(target, false);
        assert_code(
            call(json!({"op":"sample_path","path_id":plan["path_id"],"t_s":-1})),
            "INVALID_COMMAND",
        );
    }
    #[test]
    fn custom_envelope_baseline_and_failed_keepout_are_atomic() {
        let mut config = ArmConfig::rebot_dm();
        let base = vec![[0.01, 0.02, 0.03]];
        config.workspace.as_mut().unwrap().check_points_end_m = base.clone();
        config.workspace.as_mut().unwrap().keepout_boxes = (0..16)
            .map(|i| KeepoutBox {
                name: format!("profile_{i}"),
                min_m: [-0.59, -0.59, 0.01],
                max_m: [-0.58, -0.58, 0.02],
            })
            .collect();
        init_config(config);
        let before = ENGINE.with(|e| e.borrow().workspace_revision);
        assert_code(
            call(json!({"op":"set_keepout","boxes":[{"name":"one_too_many",
            "min_m":[-0.57,-0.57,0.01],"max_m":[-0.56,-0.56,0.02]}]})),
            "INVALID_COMMAND",
        );
        ENGINE.with(|e| {
            let engine = e.borrow();
            assert_eq!(engine.workspace_revision, before);
            assert_eq!(
                engine
                    .config
                    .workspace
                    .as_ref()
                    .unwrap()
                    .keepout_boxes
                    .len(),
                16
            );
        });
        ok(json!({"op":"set_tool_envelope","tcp_offset":[0.03,0.0,0.0],
            "extra_check_points_end_m":[[0.02,0.03,0.04]]}));
        ENGINE.with(|e| {
            let engine = e.borrow();
            let points = &engine.config.workspace.as_ref().unwrap().check_points_end_m;
            assert_eq!(points, &vec![base[0], [0.03, 0.0, 0.0], [0.02, 0.03, 0.04]]);
        });
        ok(
            json!({"op":"set_tool_envelope","tcp_offset":[0.0,0.0,0.0],"extra_check_points_end_m":[]}),
        );
        ENGINE.with(|e| {
            assert_eq!(
                e.borrow()
                    .config
                    .workspace
                    .as_ref()
                    .unwrap()
                    .check_points_end_m,
                base
            )
        });
    }

    #[test]
    fn bounded_path_cache_evicts_old_ids_and_tool_changes_invalidate() {
        init();
        let target = xyz(&pose(false));
        let oldest = plan_to(target, false);
        assert_eq!(oldest["ok"], true);
        for _ in 0..MAX_PATHS {
            assert_eq!(plan_to(target, false)["ok"], true);
        }
        ENGINE.with(|e| assert_eq!(e.borrow().paths.len(), MAX_PATHS));
        assert_code(
            call(json!({"op":"sample_path","path_id":oldest["path_id"],"t_s":0.0})),
            "PLAN_STALE",
        );
        let newest = plan_to(target, false);
        ok(
            json!({"op":"set_tool_envelope","tcp_offset":[0.01,0.0,0.0],"extra_check_points_end_m":[]}),
        );
        assert_code(
            call(json!({"op":"sample_path","path_id":newest["path_id"],"t_s":0.0})),
            "PLAN_STALE",
        );
    }

    #[test]
    fn torque_hold_uses_measured_release_and_simulated_persistence() {
        let mut config = ArmConfig::rebot_dm();
        config.torque_guard = Some(crate::torque_guard::TorqueGuardPolicy {
            enabled: true,
            ratio: 0.85,
            persist_ms: 100,
            joints: [true; 6],
        });
        let threshold = config.torque_guard.unwrap().thresholds(&config)[0].unwrap();
        init_config(config);
        let step = |q: f32, torque: f32, torqued: bool| {
            json!({"op":"step_guards","dt_s":0.05,"torqued":torqued,
            "q_cmd":[q,0.0,0.0,0.0,0.0,0.0,0.0],"q_meas":[q,0.0,0.0,0.0,0.0,0.0,0.0],
            "torques_nm":[torque,0.0,0.0,0.0,0.0,0.0,0.0],"collision_enabled":false})
        };
        assert_eq!(ok(step(0.0, threshold, true))["torque_hold"], false);
        assert_eq!(ok(step(0.0, threshold, true))["torque_hold"], true);
        assert_eq!(ok(step(0.0, 0.0, true))["torque_hold"], true);
        assert_eq!(ok(step(0.1, 0.0, true))["torque_hold"], false);
        assert_eq!(ok(step(0.1, threshold, false))["torque_hold"], false);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn native_plan_input_parity_includes_profile_speed_and_power_caps() {
        let mut config = ArmConfig::rebot_dm();
        config.planned_max_velocity = Some(0.2);
        config.speed_cap = Some(crate::model::SpeedCap {
            linear_mps: Some(0.02),
            angular_rps: Some(0.2),
            joint_velocity_scale: Some(0.5),
        });
        init_config(config.clone());
        let mut native = crate::supervisor::SupervisorInner::new(config);
        native.lease_enabled = false;
        native.set_torque(true).unwrap();
        let positions: Vec<f32> = INSPECT.into_iter().map(|v| v as f32).chain([0.0]).collect();
        native.commanded_positions = positions.clone();
        native.measured_positions = positions.clone();
        native.target_positions = positions;
        let input = native.plan_input().unwrap();
        let mut goal = input.arm.fk(&input.q0);
        goal.p[0] += 0.02;
        let path = motion::plan_cartesian_ex(&input, goal, Speed::default(), false).unwrap();
        // Use the same f32 observation promoted to f64 as the native snapshot.
        let response = ok(
            json!({"op":"plan_cartesian","q0_motor":input.q0,"j7":input.j7,
            "goal":{"xyz":goal.p,"mat":goal.r}}),
        );
        assert!((response["duration_s"].as_f64().unwrap() - path.duration).abs() < 1e-9);
        assert_eq!(response["power_scale"], path.power_scale);
        let last: [f32; 7] = serde_json::from_value(response["last_q"].clone()).unwrap();
        assert_eq!(last, path.last());
    }

    #[test]
    fn four_layer_envelope_and_tcp_only_floor_and_payload_lifecycle() {
        // Start with permanent tool envelope points + nonzero tcp_offset via apply_tool_overlay.
        let base_cfg = ArmConfig::rebot_dm();
        let base_count = base_cfg.workspace.as_ref().unwrap().check_points_end_m.len();
        let pack: crate::tool::ToolPackConfig = serde_json::from_value(json!({
            "id": "perm_tool",
            "kind": "gripper",
            "robots": ["b601dm"],
            "tcp_offset": [0.05, 0.0, 0.0],
            "armd": {
                "extra_check_points_end_m": [[0.02, 0.04, 0.01]]
            }
        }))
        .unwrap();
        let config = crate::tool::apply_tool_overlay(&base_cfg, &pack).unwrap();
        assert_eq!(
            config.workspace.as_ref().unwrap().check_points_end_m.len(),
            base_count + 2
        );
        init_config(config);

        // Attach a transient payload, then detach it (`extra_check_points_end_m: []`);
        // permanent tool check points AND tcp_offset must remain intact!
        let payload_pt = [0.0, 0.15, 0.0];
        let attached = ok(json!({
            "op": "set_tool_envelope",
            "extra_check_points_end_m": [payload_pt]
        }));
        assert_eq!(attached["ok"], true);
        ENGINE.with(|e| {
            let eng = e.borrow();
            assert_eq!(eng.config.tcp_offset, [0.05, 0.0, 0.0]);
            assert_eq!(
                eng.config.permanent_tool_check_points_end_m,
                vec![[0.02, 0.04, 0.01]]
            );
            assert_eq!(eng.config.transient_payload_check_points_end_m, vec![payload_pt]);
            assert_eq!(
                eng.config.workspace.as_ref().unwrap().check_points_end_m.len(),
                base_count + 3
            );
        });
        let detached = ok(json!({
            "op": "set_tool_envelope",
            "extra_check_points_end_m": []
        }));
        assert_eq!(detached["ok"], true);
        ENGINE.with(|e| {
            let eng = e.borrow();
            assert_eq!(eng.config.tcp_offset, [0.05, 0.0, 0.0]);
            assert_eq!(
                eng.config.permanent_tool_check_points_end_m,
                vec![[0.02, 0.04, 0.01]]
            );
            assert!(eng.config.transient_payload_check_points_end_m.is_empty());
            assert_eq!(
                eng.config.workspace.as_ref().unwrap().check_points_end_m.len(),
                base_count + 2
            );
        });

        // TCP-offset-only floor rejection: bare base check_points clear floor_z, while
        // tcp_offset extending downward breaks floor_z even with extra_check_points_end_m = [].
        let mut bare_cfg = ArmConfig::rebot_dm();
        // Keep only origin in base check points so ONLY tcp_offset extends downward.
        bare_cfg.workspace.as_mut().unwrap().check_points_end_m = vec![[0.0, 0.0, 0.0]];
        bare_cfg.base_check_points_end_m = vec![[0.0, 0.0, 0.0]];
        let end_pose = bare_cfg.arm_model.fk(&INSPECT);
        // Set floor_z just 1 cm below end_link, while tcp_offset puts TCP 3 cm below end_link.
        let r = end_pose.r;
        let down_in_end = k::mat_vec(&k::transpose(&r), &[0.0, 0.0, -0.04]);
        bare_cfg.workspace.as_mut().unwrap().floor_z_m = (end_pose.p[2] - 0.01) as f32;
        bare_cfg.tcp_offset = down_in_end;
        bare_cfg.rebuild_effective_check_points().unwrap();
        let robot_id = bare_cfg
            .profile_id
            .as_deref()
            .and_then(crate::tool::normalize_robot_id)
            .unwrap_or("unknown");
        let (max_w, grasp_h) = crate::tool::baseline_grasp_limits(&bare_cfg);
        bare_cfg.active_tool = crate::tool::ActiveToolInfo {
            schema_version: 1,
            id: "tcp_only_test".into(),
            is_stock: false,
            kind: "gripper".into(),
            tcp_offset_m: bare_cfg.tcp_offset,
            config_digest: crate::tool::compute_tool_config_digest(
                robot_id,
                "tcp_only_test",
                "gripper",
                bare_cfg.tcp_offset,
                &bare_cfg.permanent_tool_check_points_end_m,
                bare_cfg.physical_contact_end_m,
                bare_cfg.contact_opening_m,
                None,
                bare_cfg.gripper_policy.as_ref(),
                max_w,
                grasp_h,
                None,
            ),
            pack_digest: None,
            commissioning_required: false,
        };
        init_config(bare_cfg);

        let tcp_start = pose(true);
        let mut target = xyz(&tcp_start);
        target[0] += 0.01;
        let rej = plan_to(target, true);
        assert_code(rej.clone(), "WORKSPACE_VIOLATION");
        assert!(rej["message"].as_str().unwrap().contains("floor"), "{rej}");
    }

    #[test]
    fn resolve_tool_config_returns_canonical_digest_and_rejects_omitted_tcp_offset() {
        let res = ok(json!({
            "op": "resolve_tool_config",
            "robot_id": "b601dm",
            "manifest": {
                "id": "soft_finray_gripper",
                "kind": "gripper",
                "robots": ["b601dm"],
                "tcp_offset": [0.0288, 0.0, 0.0],
                "armd": {
                    "extra_check_points_end_m": [
                        [0.032,  0.082,  0.022],
                        [0.032,  0.082, -0.022],
                        [0.032, -0.082,  0.022],
                        [0.032, -0.082, -0.022],
                        [0.032,  0.000,  0.022],
                        [0.032,  0.000, -0.022],
                        [0.000,  0.082,  0.022],
                        [0.000,  0.082, -0.022],
                        [0.000, -0.082,  0.022],
                        [0.000, -0.082, -0.022]
                    ],
                    "physical_contact_end_m": [0.0288, 0.0, 0.0],
                    "contact_opening_m": 0.0036,
                    "dynamics": { "mass_delta_kg": 0.035, "com_moment_delta_nm": 0.018 },
                    "gripper_policy": {
                        "jaw_opening_nominal": [[-5.7, 0.0886], [0.0, 0.0036]],
                        "holding_gap_rad": 0.30,
                        "holding_torque_nm": 0.0667,
                        "contact_torque_nm": 0.6,
                        "squeeze_rad": 0.15,
                        "max_close_torque_nm": 1.0
                    },
                    "grasp": { "max_grasp_width_m": 0.075, "grasp_height_m": 0.015, "edge_to_low_m": null }
                }
            }
        }));
        assert_eq!(res["id"], "soft_finray_gripper");
        assert_eq!(res["robot_id"], "b601dm");
        assert_eq!(res["is_stock"], false);
        assert_eq!(res["kind"], "gripper");
        assert_eq!(res["tcp_offset_m"], json!([0.0288, 0.0, 0.0]));
        assert_eq!(
            res["config_digest"],
            "v1:377d453f74a2016a65bd15dfd2a471fc3c6afe9d3cf9c8f0c494fea024d14360"
        );
        assert!(res["canonical_json"].as_str().unwrap().contains("\"tool_id\":\"soft_finray_gripper\""));

        let stock_so101 = ok(json!({
            "op": "resolve_tool_config",
            "robot_id": "so101",
            "manifest": {"id": "default", "kind": "gripper"}
        }));
        assert_eq!(stock_so101["is_stock"], true);
        assert_eq!(
            stock_so101["config_digest"],
            "v1:4a99263dd305f5bf7e7f0f6a96dc2802b6be9b4255e26b88fcbd53f3112c0ef0"
        );

        let missing_tcp = call(json!({
            "op": "resolve_tool_config",
            "robot_id": "b601dm",
            "manifest": {"id": "no_tcp", "kind": "gripper", "robots": ["b601dm"]}
        }));
        assert_code(missing_tcp.clone(), "INVALID_COMMAND");
        assert!(missing_tcp["message"].as_str().unwrap().contains("tcp_offset"));

        // R2 M3: ComposeMjcf with non-stock manifest + explicit tcp_offset + omitted/whitespace tool_xml
        // still updates base <site name="tcp"> and fails closed when <site name="tcp"> is missing.
        for tool_xml_field in [json!(null), json!(""), json!("   ")] {
            let mut req = json!({
                "op": "compose_mjcf",
                "tool_manifest": {
                    "id": "offset_only_tool",
                    "kind": "gripper",
                    "robots": ["b601dm"],
                    "tcp_offset": [0.025, 0.0, 0.0]
                }
            });
            if !tool_xml_field.is_null() {
                req["tool_xml"] = tool_xml_field;
            }
            let out = ok(req);
            let xml = out["xml"].as_str().unwrap();
            let doc = roxmltree::Document::parse(xml).unwrap();
            let tcp_site = doc
                .descendants()
                .find(|n| n.is_element() && n.tag_name().name() == "site" && n.attribute("name") == Some("tcp"))
                .unwrap();
            assert_eq!(tcp_site.attribute("pos"), Some("0.025 0 0"));
        }
        let no_tcp_err = call(json!({
            "op": "compose_mjcf",
            "base_xml": "<mujoco><worldbody><body name=\"end_link\"/></worldbody></mujoco>",
            "tool_manifest": {
                "id": "offset_only_tool",
                "kind": "gripper",
                "robots": ["b601dm"],
                "tcp_offset": [0.025, 0.0, 0.0]
            },
            "tool_xml": ""
        }));
        assert_code(no_tcp_err, "INVALID_COMMAND");
    }
}
