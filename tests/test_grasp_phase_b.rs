//! Comprehensive dual-owner (`OWNER_LEGACY` and `OWNER_EXECUTOR`) Phase B adaptive grasp test suite.
//!
//! Covers every requirement of `grasp-phase-b-spec.md` except the real-arm hardware test:
//! 1. Oblique geometry end-to-end (legacy `IK_UNREACHABLE` -> adaptive oblique succeeds -> `place`
//!    consumes `HeldObject`); asserts `yaw_outward` is never applied and J7 moves ONLY in gripper
//!    children (0 J7 drift during arm children).
//! 2. Swept-joint `pregrasp` fallback (`PregraspMode::SweptJoint`) with `lock_j7_hold: true`
//!    preserving exact IK branch and locking J7 on both owners.
//! 3. Mid-transition keep-out, thin keep-out piercing carried-object face (all 8 corners outside)
//!    during `separation` and `lift`, and dry-run/live refusal parity with checked-only hints.
//! 4. Contact vs empty (`NOT_HOLDING` -> closed-jaw withdrawal, NEVER opening jaws at contact),
//!    early contact outside `contact_window_m` (`COLLISION` -> open-jaw withdrawal), and post-lift
//!    drop (`DROPPED` -> NO automatic regrip retry) on both owners.
//! 5. Lease-free `OWNER_EXECUTOR` dry-run parity (0 `SupervisorLock`) + legacy dry-run untouched +
//!    budget exhaustion / lift torque / tight parent deadline parity.
//! 6. Stale/cached frames (`TARGET_LOST`), `model_generation` change (`DETECTION_UNAVAILABLE`),
//!    shifted / rotated / ambiguous duplicate targets (`TARGET_DISPLACED`), stale start pose
//!    (`PLAN_STALE`), and concurrent revision mutations (workspace, grasp_setup, calibration,
//!    including final-poll-to-`FinishMacro(Done, HeldUpdate::Set)` race).
//! 7. Boundary Stop / E-Stop / Fault / LeaseLoss / Timeout (`GRASP_TIMEOUT`) matrix on BOTH owners:
//!    asserts EXACTLY ONE terminal and ZERO motion TX after effective TX cutoff.
#![allow(clippy::too_many_arguments)]

mod common;

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use armd::driver::{
    ArmDriver, ArmDriverError, DriverCapabilities, EnableState, JointTarget, MotorCurrentCap,
    PosForceEmitOutcome, PosForceIntent, PosForceIntentKind, PosForceIntentStamp, PosForceTxStatus,
    TorqueOutcome, TorqueProgress, TxBatchOutcome,
};
use armd::grasp::{
    self, BoxFuture, CamModel, Detection, DetectionFrame, DetectionSource, GraspDeps, GraspSetup,
    ModeInfo, ProviderModels, ResidentModels,
};
use armd::grasp_select::{self, PregraspMode, Strategy};
use armd::kinematics::{self, Pose, Vec3};
use armd::macro_action::{ActionId, ChildSpec, ChildTicket};
use armd::model::{CameraMount, JointState, KeepoutBox};
use armd::place;
use armd::rt_core::broker::{ClientId, DeliveryDecision, Outcome, Reply, Submit};
use armd::rt_core::config_iface::{ConfigDomain, ConfigProposal};
use armd::rt_core::executor::{ExecutorConfig, RtExecutorHandle};
use armd::rt_core::ops::{PlanSlot, Primitive};
use armd::rt_core::requests::actions::{
    install_begin_macro_admitted, install_macro_child, prepare_child_cartesian_slot_with_options,
    ActionSnapshot, ActionStamp, ActionSubmit, FinishMacroSpec, FinishState, HeldUpdate,
    MacroChildPayload, OwnerCheckCtx, PreparedApproachLeg, ResolvedCartesianLegs, StagedAction,
    StagedQuinticLeg, DEFAULT_ACTION_DEADLINE,
};
use armd::rt_core::requests::config::{
    run_config_commit, ConfigAuthContext, ConfigCandidate, ConfigCommitPlan, ConfigWriteOp,
    ReservePrecheck,
};
use armd::rt_core::requests::lease::{
    install_lease_acquire, ConnectionHandle, LeaseAcquire, LeaseInstallCtx, LeaseRelease,
    LeaseRequest,
};
use armd::rt_core::urgent::UrgentStop;
use armd::supervisor::OWNER_EXECUTOR;
use armd::{ArmConfig, ArmError, ArmSupervisor, SupervisorInner};
use serde_json::{json, Value};

const INSPECT: [f64; 6] = [0.0, -1.167, -1.565, 1.435, 0.0, 0.0];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Owner {
    Legacy,
    Executor,
}

impl Owner {
    const BOTH: [Self; 2] = [Self::Legacy, Self::Executor];
}

// ---- Simulated Driver with TX Recording, Object Contact, Early Obstruction & Fault Injection ----

#[derive(Debug, Clone)]
struct ContactInject {
    pregrasp_tcp: Vec3,
    grasp_tcp: Vec3,
    trigger_from_pregrasp_m: f64,
    min_rem_to_grasp_m: f64,
    residual_nm: f32,
    armed: bool,
    fired: bool,
}

struct DriverSim {
    enabled: bool,
    pos: [f32; 7],
    vel: [f32; 7],
    tau: [f32; 7],
    open_rad: f32,
    closed_rad: f32,
    /// When `Some(j7)`, jaws stall at `j7` with holding torque when closing past it.
    object_j7: Option<f32>,
    /// When `Some(z)`, object slips out once TCP z exceeds `z` (for `DROPPED` test).
    drop_after_z: Option<f64>,
    dropped: bool,
    /// Optional early obstruction during insertion (for `COLLISION` test).
    contact: Option<ContactInject>,
    /// When true, `read_joint_state` returns a driver fault.
    inject_fault: bool,
    /// Realistic per-read feedback sequence and timestamp.
    fb_seq: u64,
    fb_instant: Instant,
    /// When true, expose active `PosForce` J7 mode and move J7 ONLY via `emit_pos_force_j7`.
    pos_force_active: bool,
    external_pf: bool,
    /// Every emitted J7 PosForce intent: `(timestamp, kind, is_stale, measured_pos_before_step)`.
    pf_intent_log: Vec<(Instant, PosForceIntentKind, bool, [f32; 7])>,
    /// Every TX write `(timestamp, enabled, is_estop_hold, target_positions)`.
    tx_log: Vec<(Instant, bool, bool, [f32; 7])>,
    /// Counts distinct close transitions (from open-side toward closed-side) to prove no regrip.
    close_transitions: usize,
    last_j7_cmd: f32,
    in_close_stroke: bool,
    arm_model: armd::chain::ArmModel,
    tcp_offset: [f64; 3],
}

struct PhaseBDriver {
    sim: Arc<Mutex<DriverSim>>,
}

impl PhaseBDriver {
    fn new(
        cfg: &ArmConfig,
        q0: &[f32; 7],
        object_j7: Option<f32>,
        pos_force_active: bool,
    ) -> (Self, Arc<Mutex<DriverSim>>) {
        let gp = cfg.gripper_policy.as_ref().expect("gripper policy");
        let sim = Arc::new(Mutex::new(DriverSim {
            enabled: false,
            pos: *q0,
            vel: [0.0; 7],
            tau: [0.0; 7],
            open_rad: gp.open_rad,
            closed_rad: gp.closed_rad,
            object_j7,
            drop_after_z: None,
            dropped: false,
            contact: None,
            inject_fault: false,
            fb_seq: 1,
            fb_instant: Instant::now(),
            pos_force_active,
            external_pf: false,
            pf_intent_log: Vec::new(),
            tx_log: Vec::new(),
            close_transitions: 0,
            last_j7_cmd: q0[6],
            in_close_stroke: false,
            arm_model: cfg.arm_model.clone(),
            tcp_offset: cfg.active_tool.tcp_offset_m,
        }));
        (Self { sim: Arc::clone(&sim) }, sim)
    }

    fn apply_j7_target(s: &mut DriverSim, desired_j7: f32, max_step: Option<f32>) {
        let close_dir = if s.closed_rad >= s.open_rad { 1.0f32 } else { -1.0f32 };
        let next_cmd = match max_step {
            Some(step) if step.is_finite() && step > 0.0 => {
                let d = desired_j7 - s.pos[6];
                if d.abs() <= step {
                    desired_j7
                } else {
                    s.pos[6] + step.copysign(d)
                }
            }
            _ => desired_j7,
        };
        let dj7 = (next_cmd - s.last_j7_cmd) * close_dir;
        if dj7 > 0.005 {
            if !s.in_close_stroke {
                s.close_transitions += 1;
                s.in_close_stroke = true;
            }
        } else if dj7 < -0.005 {
            s.in_close_stroke = false;
        }
        s.last_j7_cmd = next_cmd;
        let eff_obj = if s.dropped { None } else { s.object_j7 };
        if let Some(obj_j7) = eff_obj {
            let past_obj = (next_cmd - obj_j7) * close_dir >= 0.0;
            if past_obj {
                s.pos[6] = obj_j7;
                s.vel[6] = 0.0;
                s.tau[6] = 1.5 * close_dir;
                return;
            }
        }
        s.vel[6] = (next_cmd - s.pos[6]) / 0.02;
        s.pos[6] = next_cmd;
        s.tau[6] = 0.0;
    }
}

impl ArmDriver for PhaseBDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            torque_feedback: true,
            ..Default::default()
        }
    }
    fn supports_rt_executor(&self) -> Result<(), &'static str> {
        Ok(())
    }
    fn is_feedback_valid(&self) -> bool {
        !self.sim.lock().unwrap().inject_fault
    }
    fn feedback_ages(&self) -> Vec<Option<Duration>> {
        let now = Instant::now();
        let ts = self.sim.lock().unwrap().fb_instant;
        vec![Some(now.saturating_duration_since(ts)); 7]
    }
    fn feedback_instants(&self) -> Vec<Option<Instant>> {
        vec![Some(self.sim.lock().unwrap().fb_instant); 7]
    }
    fn feedback_seqs(&self) -> Vec<u64> {
        vec![self.sim.lock().unwrap().fb_seq; 7]
    }
    fn feedback_receive_instants(&self) -> Vec<Option<Instant>> {
        vec![Some(self.sim.lock().unwrap().fb_instant); 7]
    }
    fn feedback_ages_into(&self, out: &mut Vec<Option<Duration>>) {
        let now = Instant::now();
        let ts = self.sim.lock().unwrap().fb_instant;
        let age = now.saturating_duration_since(ts);
        out.clear();
        out.extend(std::iter::repeat_n(Some(age), 7));
    }
    fn feedback_all_within(&self, dof: usize, _max_age: Duration) -> bool {
        dof == 7 && !self.sim.lock().unwrap().inject_fault
    }
    fn feedback_age_ms_into(&self, out: &mut Vec<Option<u64>>) {
        let now = Instant::now();
        let ts = self.sim.lock().unwrap().fb_instant;
        let ms = now.saturating_duration_since(ts).as_millis() as u64;
        out.clear();
        out.extend(std::iter::repeat_n(Some(ms), 7));
    }
    fn feedback_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let ts = self.sim.lock().unwrap().fb_instant;
        out.clear();
        out.extend(std::iter::repeat_n(Some(ts), 7));
    }
    fn feedback_seqs_into(&self, out: &mut Vec<u64>) {
        let seq = self.sim.lock().unwrap().fb_seq;
        out.clear();
        out.extend(std::iter::repeat_n(seq, 7));
    }
    fn feedback_receive_instants_into(&self, out: &mut Vec<Option<Instant>>) {
        let ts = self.sim.lock().unwrap().fb_instant;
        out.clear();
        out.extend(std::iter::repeat_n(Some(ts), 7));
    }
    fn motor_current_cap(&self) -> Option<MotorCurrentCap> {
        let s = self.sim.lock().unwrap();
        s.pos_force_active.then_some(MotorCurrentCap {
            joint: 6,
            mode: "pos_force",
            current_raw: Some(2500),
            active: true,
            calibrated: true,
            ceiling_torque_nm: Some(1.5),
            torque_reference: None,
            close_speed_rad_s: Some(2.0),
            note: "phase_b_sim_pos_force",
        })
    }
    fn set_external_pos_force_emitter(&mut self, enabled: bool) {
        self.sim.lock().unwrap().external_pf = enabled;
    }
    fn pos_force_feedback_fresh(&self) -> bool {
        !self.sim.lock().unwrap().inject_fault
    }
    fn pos_force_last_fresh_position(&self) -> Option<f32> {
        Some(self.sim.lock().unwrap().pos[6])
    }
    fn emit_pos_force_j7(
        &mut self,
        intent: PosForceIntent,
        current: PosForceIntentStamp,
    ) -> Result<PosForceEmitOutcome, ArmDriverError> {
        let mut s = self.sim.lock().unwrap();
        if !s.pos_force_active {
            return Ok(PosForceEmitOutcome::NotApplicable);
        }
        let stale = intent.stamp.is_stale_against(current);
        let snap_pos = s.pos;
        s.pf_intent_log.push((Instant::now(), intent.kind, stale, snap_pos));
        if s.enabled && !s.inject_fault {
            if let PosForceIntentKind::Target { position, velocity, .. } = intent.kind {
                if !stale {
                    let step = velocity.filter(|v| v.is_finite() && *v > 0.0).map(|v| v * 0.02);
                    Self::apply_j7_target(&mut s, position, step);
                }
            } else {
                // Hold / Stop keeps current measured J7 stationary (maintaining stall torque if holding).
                s.vel[6] = 0.0;
                let close_dir = if s.closed_rad >= s.open_rad { 1.0f32 } else { -1.0f32 };
                let eff_obj = if s.dropped { None } else { s.object_j7 };
                if let Some(obj_j7) = eff_obj {
                    if (s.pos[6] - obj_j7).abs() < 0.05 {
                        s.tau[6] = 1.5 * close_dir;
                    }
                } else {
                    s.tau[6] = 0.0;
                }
            }
        }
        Ok(PosForceEmitOutcome::Sent {
            joint: 6,
            freeze: false,
            pf_status: PosForceTxStatus::Fresh,
        })
    }
    fn enable_state(&self) -> EnableState {
        if self.sim.lock().unwrap().enabled {
            EnableState::Enabled
        } else {
            EnableState::Disabled
        }
    }
    fn read_joint_states(&mut self) -> Result<Vec<JointState>, ArmDriverError> {
        let mut out = Vec::with_capacity(7);
        self.read_joint_states_into(&mut out)?;
        Ok(out)
    }
    fn read_joint_states_into(&mut self, out: &mut Vec<JointState>) -> Result<(), ArmDriverError> {
        let mut s = self.sim.lock().unwrap();
        if s.inject_fault {
            return Err(ArmDriverError::HardwareFault("injected Phase B driver fault".into()));
        }
        s.fb_seq = s.fb_seq.saturating_add(1);
        s.fb_instant = Instant::now();
        out.clear();
        for i in 0..7 {
            out.push(JointState {
                position: s.pos[i],
                velocity: s.vel[i],
                torque: s.tau[i],
                temperature: 30.0,
            });
        }
        Ok(())
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<JointState>,
        _dl: Instant,
        _max: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.read_joint_states_into(out)?;
        Ok((7, false))
    }
    fn write_joint_targets(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.sim.lock().unwrap();
        if s.inject_fault {
            return Err(ArmDriverError::HardwareFault("injected Phase B driver TX fault".into()));
        }
        let mut cmd = s.pos;
        for (i, t) in targets.iter().take(7).enumerate() {
            cmd[i] = t.position;
        }
        let now = Instant::now();
        let enabled = s.enabled;
        s.tx_log.push((now, enabled, false, cmd));
        if !enabled {
            return Ok(());
        }

        // When PosForce is active with external emitter, J7 moves ONLY via `emit_pos_force_j7`!
        let pos_force_external = s.pos_force_active && s.external_pf;
        if !pos_force_external {
            let close_dir = if s.closed_rad >= s.open_rad { 1.0f32 } else { -1.0f32 };
            let dj7 = (cmd[6] - s.last_j7_cmd) * close_dir;
            if dj7 > 0.005 {
                if !s.in_close_stroke {
                    s.close_transitions += 1;
                    s.in_close_stroke = true;
                }
            } else if dj7 < -0.005 {
                s.in_close_stroke = false;
            }
            s.last_j7_cmd = cmd[6];
        }

        // Arm joints 0..6: check optional early contact obstruction (only armed AFTER arriving
        // at pregrasp with open jaws, so pregrasp transition itself is never obstructed!).
        let mut obstructed = false;
        let cur_end = s.arm_model.fk_f32(&s.pos);
        let cur_tcp = cur_end.map(|e| armd::motion::end_to_tcp(&e, &s.tcp_offset));
        let cmd_tcp = s
            .arm_model
            .fk_f32(&cmd)
            .map(|e| armd::motion::end_to_tcp(&e, &s.tcp_offset));
        let jaws_open = (s.pos[6] - s.open_rad).abs() < 0.6;
        if let (Some(ref mut ci), Some(tcp)) = (s.contact.as_mut(), cur_tcp) {
            let d_pre = kinematics::norm(&kinematics::sub(&tcp.p, &ci.pregrasp_tcp));
            let d_grasp = kinematics::norm(&kinematics::sub(&tcp.p, &ci.grasp_tcp));
            if !ci.armed && jaws_open && d_pre < 0.008 {
                ci.armed = true;
            }
            if ci.armed && !ci.fired && d_pre >= ci.trigger_from_pregrasp_m && d_grasp >= ci.min_rem_to_grasp_m {
                ci.fired = true;
                obstructed = true;
            } else if ci.fired {
                // Once fired, check if command is still pushing deeper toward grasp_tcp;
                // once moving back toward pregrasp_tcp, clear obstruction so withdrawal runs!
                if let Some(ct) = cmd_tcp {
                    let cmd_d_grasp = kinematics::norm(&kinematics::sub(&ct.p, &ci.grasp_tcp));
                    if cmd_d_grasp <= d_grasp + 0.002 {
                        obstructed = true;
                    }
                }
            }
        }

        // Gravity torques for realistic J2/J3 residual (gravity + obstruction residual).
        let grav = s.arm_model.gravity_torques(&s.pos).unwrap_or([0.0; 8]);
        for i in 0..6 {
            if obstructed {
                s.vel[i] = 0.0;
                s.tau[i] = grav[i];
            } else {
                s.vel[i] = (cmd[i] - s.pos[i]) / 0.02;
                s.pos[i] = cmd[i];
                s.tau[i] = grav[i];
            }
        }
        if obstructed {
            let res = s.contact.as_ref().map_or(0.0, |c| c.residual_nm);
            s.tau[1] = grav[1] + res;
        }

        // Check post-lift drop trigger (only after the gripper has actually closed at least once).
        if s.close_transitions >= 1 {
            if let (Some(dz), Some(end)) = (s.drop_after_z, s.arm_model.fk_f32(&s.pos)) {
                let tcp = armd::motion::end_to_tcp(&end, &s.tcp_offset);
                if tcp.p[2] >= dz {
                    s.dropped = true;
                }
            }
        }

        // Gripper joint 6: stall on object when closing unless dropped (only when non-PosForce).
        if !pos_force_external {
            let close_dir = if s.closed_rad >= s.open_rad { 1.0f32 } else { -1.0f32 };
            let eff_obj = if s.dropped { None } else { s.object_j7 };
            if let Some(obj_j7) = eff_obj {
                let past_obj = (cmd[6] - obj_j7) * close_dir >= 0.0;
                if past_obj {
                    s.pos[6] = obj_j7;
                    s.vel[6] = 0.0;
                    s.tau[6] = 1.5 * close_dir;
                } else {
                    s.vel[6] = (cmd[6] - s.pos[6]) / 0.02;
                    s.pos[6] = cmd[6];
                    s.tau[6] = 0.0;
                }
            } else {
                s.vel[6] = (cmd[6] - s.pos[6]) / 0.02;
                s.pos[6] = cmd[6];
                s.tau[6] = 0.0;
            }
        }
        Ok(())
    }
    fn write_joint_targets_bounded(
        &mut self,
        targets: &[JointTarget],
    ) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_joint_targets(targets)?;
        Ok(TxBatchOutcome::Complete)
    }
    fn write_estop_hold(&mut self, targets: &[JointTarget]) -> Result<(), ArmDriverError> {
        let mut s = self.sim.lock().unwrap();
        let mut cmd = s.pos;
        for (i, t) in targets.iter().take(7).enumerate() {
            cmd[i] = t.position;
        }
        let enabled = s.enabled;
        s.tx_log.push((Instant::now(), enabled, true, cmd));
        Ok(())
    }
    fn write_estop_hold_bounded(
        &mut self,
        targets: &[JointTarget],
    ) -> Result<TxBatchOutcome, ArmDriverError> {
        self.write_estop_hold(targets)?;
        Ok(TxBatchOutcome::Complete)
    }
    fn set_torque(&mut self, enable: bool) -> Result<(), ArmDriverError> {
        let mut s = self.sim.lock().unwrap();
        s.enabled = enable;
        Ok(())
    }
    fn begin_torque_budgeted(
        &mut self,
        enable: bool,
        _max: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        let mut s = self.sim.lock().unwrap();
        s.enabled = enable;
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn poll_torque_budgeted(
        &mut self,
        _dl: Instant,
        _max: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        Ok(TorqueProgress::Done(TorqueOutcome::Confirmed))
    }
    fn abort_enable(&mut self) {}
    fn torque_transition_pending(&self) -> bool {
        false
    }
}

// ---- Simulated Vision Source for Phase B Revalidation ----

struct VisionState {
    poly: Vec<[f64; 2]>,
    height_m: f64,
    table_z: f64,
    label: String,
    shift_xy: [f64; 2],
    rotate_deg: f64,
    visible: bool,
    duplicate_near: bool,
    stale_frames: bool,
    repeat_frame_id: Option<u64>,
    omit_policy_captured_at: bool,
    per_request: bool,
    mode_generation: Option<u64>,
    frame_generation: Option<u64>,
    preflight_delay: Option<Duration>,
    on_frame_hook: Option<Arc<dyn Fn(u64) + Send + Sync>>,
}

struct PhaseBVision {
    sup: Arc<ArmSupervisor>,
    cam: CamModel,
    arm_model: armd::chain::ArmModel,
    next_id: AtomicU64,
    state: Mutex<VisionState>,
}

impl PhaseBVision {
    fn new(
        sup: Arc<ArmSupervisor>,
        cam: CamModel,
        arm_model: armd::chain::ArmModel,
        poly: Vec<[f64; 2]>,
        height_m: f64,
        table_z: f64,
    ) -> Arc<Self> {
        Arc::new(Self {
            sup,
            cam,
            arm_model,
            next_id: AtomicU64::new(10),
            state: Mutex::new(VisionState {
                poly,
                height_m,
                table_z,
                label: "banana".into(),
                shift_xy: [0.0, 0.0],
                rotate_deg: 0.0,
                visible: true,
                duplicate_near: false,
                stale_frames: false,
                repeat_frame_id: None,
                omit_policy_captured_at: false,
                per_request: true,
                mode_generation: Some(1),
                frame_generation: Some(1),
                preflight_delay: None,
                on_frame_hook: None,
            }),
        })
    }

    fn transformed_poly(st: &VisionState) -> Vec<[f64; 2]> {
        let c = armd::grasp_select::geometry::centroid(&st.poly);
        let (sn, cs) = st.rotate_deg.to_radians().sin_cos();
        st.poly
            .iter()
            .map(|p| {
                let dx = p[0] - c[0];
                let dy = p[1] - c[1];
                [
                    c[0] + dx * cs - dy * sn + st.shift_xy[0],
                    c[1] + dx * sn + dy * cs + st.shift_xy[1],
                ]
            })
            .collect()
    }

    async fn build_frame(&self) -> DetectionFrame {
        let q = if self.sup.owner_mode() == OWNER_EXECUTOR {
            self.sup
                .hub()
                .latest()
                .map(|s| s.rt.positions.to_vec())
                .unwrap_or_else(|| vec![0.0; 7])
        } else {
            self.sup.inner().read().await.measured_positions.clone()
        };
        let pose = self.arm_model.fk(&kinematics::q6(&q));
        let (
            frame_id,
            captured_at,
            policy_captured_at,
            model_generation,
            detections,
            hook,
        ) = {
            let st = self.state.lock().unwrap();
            let fid = st
                .repeat_frame_id
                .unwrap_or_else(|| self.next_id.fetch_add(1, Ordering::SeqCst));
            let now = Instant::now();
            let cap = if st.stale_frames {
                now - Duration::from_secs(30)
            } else {
                now
            };
            let pol_cap = if st.omit_policy_captured_at { None } else { Some(cap) };
            let mut dets = Vec::new();
            if st.visible {
                let poly = Self::transformed_poly(&st);
                let mut b = [1.0f64, 1.0, 0.0, 0.0];
                let mut ok = true;
                for z in [st.table_z, st.table_z + st.height_m] {
                    for v in &poly {
                        if let Some(p) = self.cam.project(&pose, &[v[0], v[1], z]) {
                            let n = self.cam.to_norm(p);
                            b = [b[0].min(n[0]), b[1].min(n[1]), b[2].max(n[0]), b[3].max(n[1])];
                        } else {
                            ok = false;
                        }
                    }
                }
                if ok && b[2] > b[0] && b[3] > b[1] {
                    let contour: Option<Vec<[f64; 2]>> = poly
                        .iter()
                        .map(|v| {
                            self.cam
                                .project(&pose, &[v[0], v[1], st.table_z + st.height_m * 0.5])
                                .map(|p| self.cam.to_norm(p))
                        })
                        .collect();
                    let det = Detection {
                        label: st.label.clone(),
                        score: 0.92,
                        raw_score_valid: true,
                        box_norm: [
                            b[0].clamp(0.0, 1.0),
                            b[1].clamp(0.0, 1.0),
                            b[2].clamp(0.0, 1.0),
                            b[3].clamp(0.0, 1.0),
                        ],
                        angle_deg: None,
                        angle_conf: None,
                        grasp_width_px: None,
                        mask_focus: false,
                        disp_fg: None,
                        disp_bg: None,
                        depth_conf: None,
                        length_px: None,
                        jaw_clear: None,
                        depth_angle_deg: None,
                        depth_angle_conf: None,
                        depth_width_px: None,
                        contour_norm: contour.map(Ok),
                    };
                    dets.push(det.clone());
                    if st.duplicate_near {
                        let mut dup = det;
                        dup.score = 0.90;
                        dup.box_norm = [
                            (dup.box_norm[0] + 0.015).clamp(0.0, 1.0),
                            (dup.box_norm[1] + 0.010).clamp(0.0, 1.0),
                            (dup.box_norm[2] + 0.015).clamp(0.0, 1.0),
                            (dup.box_norm[3] + 0.010).clamp(0.0, 1.0),
                        ];
                        dets.push(dup);
                    }
                }
            }
            (fid, cap, pol_cap, st.frame_generation, dets, st.on_frame_hook.clone())
        };
        if let Some(h) = hook {
            h(frame_id);
        }
        DetectionFrame {
            frame_id,
            captured_at,
            policy_captured_at,
            image_size_px: self.cam.size,
            detections,
            depth: None,
            model_generation,
        }
    }
}

impl DetectionSource for PhaseBVision {
    fn inference_enabled(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async { Ok(true) })
    }
    fn set_inference(&self, _enabled: bool) -> BoxFuture<'_, Result<(), String>> {
        Box::pin(async { Ok(()) })
    }
    fn next_frame(
        &self,
        _after: Option<u64>,
        _wait_ms: u64,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move {
            tokio::time::sleep(Duration::from_millis(8)).await;
            Ok(Some(self.build_frame().await))
        })
    }
    fn mode_info(&self) -> BoxFuture<'_, Result<ModeInfo, String>> {
        Box::pin(async move {
            let (per_req, gen, delay) = {
                let st = self.state.lock().unwrap();
                (st.per_request, st.mode_generation, st.preflight_delay)
            };
            if let Some(d) = delay {
                tokio::time::sleep(d).await;
            }
            Ok(ModeInfo {
                reported: true,
                per_request_pipelines: per_req,
                yolo_available: true,
                aruco_available: false,
                depth_available: false,
                gray_available: false,
                gray_format: Ok(None),
                upstream_loopback: true,
                depth_metric: false,
                model_kind: Some("segment".into()),
                vision: Ok(None),
                models: gen.map(|g| ProviderModels {
                    generation: g,
                    resident: ResidentModels {
                        detector: Some("yolo-seg".into()),
                        depth: None,
                    },
                    swap_state: "idle".into(),
                    swap_error: None,
                    catalogue: Default::default(),
                    ion: None,
                }),
            })
        })
    }
    fn per_request_pipelines(&self) -> BoxFuture<'_, Result<bool, String>> {
        Box::pin(async move { Ok(self.state.lock().unwrap().per_request) })
    }
    fn trigger_with_focus(
        &self,
        _pipelines: &'static [&'static str],
        _wait_ms: u64,
        _focus: Option<&grasp::MaskFocus>,
    ) -> BoxFuture<'_, Result<Option<DetectionFrame>, String>> {
        Box::pin(async move { Ok(Some(self.build_frame().await)) })
    }
}

// ---- Scene & Dual-Owner Rig Helpers ----

fn banana(mid: [f64; 2], rc: f64, half_deg: f64, w: f64, turn_deg: f64) -> Vec<[f64; 2]> {
    let n = 12;
    let (h, t) = (half_deg.to_radians(), turn_deg.to_radians());
    let mut pts = Vec::new();
    for k in 0..=n {
        let a = -h + 2.0 * h * k as f64 / n as f64;
        pts.push([(rc + w / 2.0) * a.cos(), (rc + w / 2.0) * a.sin()]);
    }
    for k in (0..=n).rev() {
        let a = -h + 2.0 * h * k as f64 / n as f64;
        pts.push([(rc - w / 2.0) * a.cos(), (rc - w / 2.0) * a.sin()]);
    }
    pts.iter()
        .map(|p| {
            let x = p[0] - rc;
            let (xr, yr) = (x * t.cos() - p[1] * t.sin(), x * t.sin() + p[1] * t.cos());
            [mid[0] + xr, mid[1] + yr]
        })
        .collect()
}

fn fast_mounted_config() -> (ArmConfig, GraspSetup) {
    let mut c = ArmConfig {
        collision: None,
        lease_timeout_ms: 600_000,
        ..ArmConfig::rebot_dm()
    };
    c.camera_mount = CameraMount {
        rot_deg: 90.0,
        tilt_deg: -38.0,
        offset_m: Some([-0.0731, 0.0005, 0.0660]),
        ..CameraMount::default()
    };
    if let Some(gp) = c.gripper_policy.as_mut() {
        gp.close_speed_rad_s = Some(2.0);
    }
    if let Some(dg) = c.damiao_gripper.as_mut() {
        dg.close_speed_rad_s = 2.0;
    }
    let s = GraspSetup {
        fx_px: 824.0,
        fy_px: 824.0,
        camera_tilt_deg: -38.0,
        approach_speed_mps: 0.14,
        descend_speed_mps: 0.12,
        settle_delay_s: 0.02,
        post_lift_settle_ms: 40,
        lost_timeout_s: 0.35,
        lost_frames: 3,
        recheck_tol_px: 35.0,
        phase_timeout_s: 30.0,
        timeout_s: 60.0,
        ..GraspSetup::default()
    };
    (c, s)
}

fn solve_camera_down(config: &ArmConfig, setup: &GraspSetup, lens: Vec3) -> [f64; 6] {
    let arm = &config.arm_model;
    let cam = CamModel::assumed(arm, setup, &config.camera_mount);
    let az = lens[1].atan2(lens[0]);
    let r0 = kinematics::mat_mul(
        &kinematics::axis_r(&[0.0, 0.0, 1.0], az),
        &grasp::look_down(&arm.fk(&INSPECT).r),
    );
    let r = grasp::camera_down(&r0, &cam.rec);
    let off = kinematics::mat_vec(&r, &cam.offset);
    let goal = Pose::new(r, kinematics::sub(&lens, &off));
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let mut q = INSPECT;
    q[0] = az;
    for _ in 0..200 {
        let sol = arm.solve_dls(&goal, &q, &lo, &hi, kinematics::PAR.w);
        q = sol.q;
        if sol.ok {
            return q;
        }
    }
    panic!("camera-down start at {lens:?} unreachable");
}

fn adaptive_request(cam: &CamModel, start: &Pose, table_z: f64, poly: &[[f64; 2]], h: f64) -> Value {
    let mut b = [1.0f64, 1.0, 0.0, 0.0];
    for z in [table_z, table_z + h] {
        for v in poly {
            let px = cam.to_norm(cam.project(start, &[v[0], v[1], z]).expect("in view"));
            b = [b[0].min(px[0]), b[1].min(px[1]), b[2].max(px[0]), b[3].max(px[1])];
        }
    }
    let contour: Vec<[f64; 2]> = poly
        .iter()
        .map(|v| cam.to_norm(cam.project(start, &[v[0], v[1], table_z + h / 2.0]).unwrap()))
        .collect();
    json!({
        "label": "banana",
        "box_norm": b,
        "frame_id": 7,
        "strategy": "adaptive",
        "evidence": {
            "contour": contour,
            "height_m": [h * 0.9, h],
            "payload_kg": 0.12
        }
    })
}

struct Rig {
    owner: Owner,
    sup: Arc<ArmSupervisor>,
    sim: Arc<Mutex<DriverSim>>,
    vision: Arc<PhaseBVision>,
    cam: CamModel,
    start_end: Pose,
    setup: GraspSetup,
    handle: Option<RtExecutorHandle>,
    sink_replies: Arc<Mutex<Vec<(ClientId, Reply)>>>,
    ticker_stop: Arc<AtomicBool>,
    conn_id: u64,
}

impl Rig {
    async fn start(
        owner: Owner,
        config: ArmConfig,
        setup: GraspSetup,
        q_arm: [f64; 6],
        poly: Vec<[f64; 2]>,
        height_m: f64,
        object_j7: Option<f32>,
        lease_enabled: bool,
    ) -> Self {
        Self::start_with_pos_force(owner, config, setup, q_arm, poly, height_m, object_j7, lease_enabled, false).await
    }

    async fn start_with_pos_force(
        owner: Owner,
        config: ArmConfig,
        setup: GraspSetup,
        q_arm: [f64; 6],
        poly: Vec<[f64; 2]>,
        height_m: f64,
        object_j7: Option<f32>,
        lease_enabled: bool,
        pos_force_active: bool,
    ) -> Self {
        let gp = config.gripper_policy.as_ref().expect("gripper policy");
        let start_j7 = gp.closed_rad;
        let q7: [f32; 7] = std::array::from_fn(|i| if i < 6 { q_arm[i] as f32 } else { start_j7 });
        let (drv, sim) = PhaseBDriver::new(&config, &q7, object_j7, pos_force_active);
        let cam = CamModel::assumed(&config.arm_model, &setup, &config.camera_mount);
        let start_end = config.arm_model.fk(&q_arm);
        let arm_model = config.arm_model.clone();
        let sup = Arc::new(ArmSupervisor::new(config).with_driver(Box::new(drv)));
        sup.set_lease_enabled(lease_enabled).await;
        {
            let mut inner = sup.inner().write().await;
            inner.set_grasp_setup(setup.clone());
            inner.commanded_positions = q7.to_vec();
            inner.measured_positions = q7.to_vec();
            inner.target_positions = q7.to_vec();
            if owner == Owner::Legacy {
                let _ = inner.register_live_connection(41);
                if lease_enabled {
                    let ch = ConnectionHandle::from_ws_id(41).unwrap();
                    let now = Instant::now();
                    install_lease_acquire(
                        &mut inner,
                        &LeaseAcquire {
                            connection: ch,
                            label: armd::model::HolderLabel::new("connection:41"),
                            has_label: true,
                        },
                        &LeaseInstallCtx {
                            expected_arm: armd::rt_core::ids::ArmId::LOCAL,
                            expected_boot: armd::rt_core::broker::BootId(1),
                            stamp_arm: armd::rt_core::ids::ArmId::LOCAL,
                            stamp_boot: armd::rt_core::broker::BootId(1),
                            now,
                            request_id: 1,
                            client: ClientId(41),
                            deadline: now + Duration::from_secs(5),
                            enforce_single_use: false,
                        },
                    )
                    .unwrap();
                }
            }
            // Temporarily enable torque before optional executor handover (with lease bypassed
            // on executor until LeaseRequest::Acquire runs post-handover).
            let saved_lease = inner.lease_enabled;
            if owner == Owner::Executor {
                inner.lease_enabled = false;
            }
            let stamp = inner.current_admission_stamp();
            let adm = inner.check_admission(stamp).unwrap();
            SupervisorInner::enable_torque(adm).unwrap();
            inner.lease_enabled = saved_lease;
            for _ in 0..3 {
                inner.tick(0.02);
            }
        }
        let vision = PhaseBVision::new(sup.clone(), cam.clone(), arm_model, poly, height_m, setup.table_z_m);
        let sink_replies = Arc::new(Mutex::new(Vec::<(ClientId, Reply)>::new()));
        let ticker_stop = Arc::new(AtomicBool::new(false));
        let handle = match owner {
            Owner::Legacy => {
                sup.start_dispatcher().unwrap();
                let sup_t = sup.clone();
                let stop_t = ticker_stop.clone();
                tokio::spawn(async move {
                    while !stop_t.load(Ordering::Acquire) {
                        sup_t.tick(0.02).await;
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                });
                None
            }
            Owner::Executor => {
                let h = sup
                    .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
                    .await
                    .unwrap();
                sup.register_ws_connection(41).await.unwrap();
                assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);
                assert!(
                    sup.inner().write().await.acquire_lease("probe").is_err(),
                    "legacy shell must be detached under OWNER_EXECUTOR"
                );
                let out = Arc::clone(&sink_replies);
                h.attach_external_reply_sink(Arc::new(move |c, r| {
                    out.lock().unwrap().push((c, r));
                    DeliveryDecision::Delivered
                }));
                for _ in 0..100 {
                    if sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                if lease_enabled {
                    let auth = sup.hub().latest().unwrap().rt.owner.authority;
                    let ch = ConnectionHandle::from_ws_id(41).unwrap();
                    let _ = h
                        .broker()
                        .lock()
                        .unwrap()
                        .submit(
                            ClientId(41),
                            1,
                            auth,
                            Instant::now() + Duration::from_secs(5),
                            Submit::Lease(LeaseRequest::Acquire(LeaseAcquire {
                                connection: ch,
                                label: armd::model::HolderLabel::new("connection:41"),
                                has_label: true,
                            })),
                        )
                        .unwrap();
                    for _ in 0..100 {
                        h.pump_broker_replies();
                        if sup.hub().latest().is_some_and(|s| s.rt.owner.authority.lease_generation > 0) {
                            break;
                        }
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                    assert!(
                        sup.hub().latest().unwrap().rt.owner.authority.lease_generation > 0,
                        "executor lease must be acquired"
                    );
                }
                Some(h)
            }
        };
        Self {
            owner,
            sup,
            sim,
            vision,
            cam,
            start_end,
            setup,
            handle,
            sink_replies,
            ticker_stop,
            conn_id: 41,
        }
    }

    async fn admission_stamp(&self) -> armd::supervisor::AdmissionStamp {
        let mut s = if self.owner == Owner::Executor {
            self.sup.capture_admission_stamp()
        } else {
            self.sup.inner().read().await.current_admission_stamp()
        };
        s.connection_id = self.conn_id;
        s
    }

    async fn plan_and_stamp_adaptive(
        &self,
        body: &Value,
    ) -> Result<(grasp::PlannedGrasp, armd::supervisor::AdmissionStamp), ArmError> {
        let adm = self.admission_stamp().await;
        let pending = match self.owner {
            Owner::Legacy => {
                let inner = self.sup.inner().read().await;
                grasp::prepare(&inner, body, inner.safety.epoch_id())?
            }
            Owner::Executor => {
                let snap = ActionSnapshot::capture(
                    &self.sup,
                    adm,
                    Instant::now() + DEFAULT_ACTION_DEADLINE,
                )
                .await?;
                grasp::prepare_from_snapshot(&snap, body, snap.owner.authority.epoch)?
            }
        };
        let mut planned = tokio::task::spawn_blocking(move || grasp::plan(pending))
            .await
            .unwrap()?;
        grasp::preflight(self.vision.as_ref(), &mut planned).await?;
        Ok((planned, adm))
    }

    async fn plan_adaptive(&self, body: &Value) -> Result<grasp::PlannedGrasp, ArmError> {
        Ok(self.plan_and_stamp_adaptive(body).await?.0)
    }

    async fn start_adaptive_with_stamp(
        &self,
        planned: grasp::PlannedGrasp,
        adm: armd::supervisor::AdmissionStamp,
    ) -> Result<(u64, f64), ArmError> {
        match self.owner {
            Owner::Legacy => {
                let mut inner = self.sup.inner().write().await;
                grasp::begin_with_stamp(
                    &mut inner,
                    adm,
                    &self.sup,
                    self.vision.clone(),
                    planned,
                )
            }
            Owner::Executor => {
                grasp::begin_executor(&self.sup, adm, self.vision.clone(), planned).await
            }
        }
    }

    async fn start_adaptive(&self, planned: grasp::PlannedGrasp) -> Result<(u64, f64), ArmError> {
        let adm = self.admission_stamp().await;
        self.start_adaptive_with_stamp(planned, adm).await
    }

    async fn wait_action_end(&self, action_id: u64, timeout: Duration) -> (String, Option<String>, Option<bool>) {
        let dl = Instant::now() + timeout;
        match self.owner {
            Owner::Legacy => {
                while Instant::now() < dl {
                    {
                        let inner = self.sup.inner().read().await;
                        if let Some(a) = inner.last_action.as_ref().filter(|a| a.id == action_id) {
                            if a.state != "running" {
                                return (
                                    a.state.as_str().to_string(),
                                    a.code.as_ref().map(|c| c.as_str().to_string()),
                                    a.holding,
                                );
                            }
                        }
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
                panic!("legacy action {action_id} did not finish within {timeout:?}");
            }
            Owner::Executor => {
                let h = self.handle.as_ref().unwrap();
                while Instant::now() < dl {
                    h.pump_broker_replies();
                    let found = self.sink_replies.lock().unwrap().iter().find_map(|(_, r)| match r {
                        Reply::Terminal { action, outcome, .. } if *action == ActionId(action_id) => {
                            Some(*outcome)
                        }
                        _ => None,
                    });
                    if let Some(outcome) = found {
                        let (state, code) = match outcome {
                            Outcome::Completed => ("done".to_string(), None),
                            Outcome::Cancelled(_) | Outcome::Stopped | Outcome::Superseded => {
                                ("cancelled".to_string(), None)
                            }
                            Outcome::Failed(code) => ("failed".to_string(), Some(code.to_string())),
                        };
                        let holding = self
                            .sup
                            .hub()
                            .latest()
                            .map(|s| s.rt.owner.held.is_some());
                        return (state, code, holding);
                    }
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
                let latest = self.sup.hub().latest();
                panic!(
                    "executor action {action_id} did not finish within {timeout:?}; macro={:?} safety={:?}",
                    latest.as_ref().map(|s| s.rt.owner.macro_state),
                    latest.as_ref().map(|s| &s.obs.safety_state),
                );
            }
        }
    }

    fn executor_terminal_count(&self, action_id: u64) -> usize {
        if let Some(h) = self.handle.as_ref() {
            h.pump_broker_replies();
        }
        self.sink_replies
            .lock()
            .unwrap()
            .iter()
            .filter(|(_, r)| matches!(r, Reply::Terminal { action, .. } if *action == ActionId(action_id)))
            .count()
    }

    async fn held_label(&self) -> Option<String> {
        match self.owner {
            Owner::Legacy => self.sup.inner().read().await.held.as_ref().map(|h| h.label.clone()),
            Owner::Executor => self
                .sup
                .hub()
                .latest()
                .and_then(|s| s.rt.owner.held.map(|h| h.label.as_str().to_string())),
        }
    }

    async fn measured_positions(&self) -> [f32; 7] {
        let v = match self.owner {
            Owner::Legacy => self.sup.inner().read().await.measured_positions.clone(),
            Owner::Executor => self.sup.hub().latest().unwrap().rt.positions.to_vec(),
        };
        std::array::from_fn(|i| v[i])
    }

    async fn bump_grasp_setup_revision(&self) {
        let mut new_setup = self.setup.clone();
        new_setup.lift_height_m += 0.002;
        match self.owner {
            Owner::Legacy => {
                let mut inner = self.sup.inner().write().await;
                inner.set_grasp_setup(new_setup);
                inner.grasp_setup_revision += 1;
            }
            Owner::Executor => {
                let env = self.sup.hub().prep_env().unwrap();
                let latest = self.sup.hub().latest().unwrap();
                let auth = ConfigAuthContext {
                    admission: self.admission_stamp().await,
                    lease_holder: None,
                };
                let proposal = ConfigProposal {
                    domain: ConfigDomain::GraspSetup,
                    expected_revision: env.grasp_setup_revision,
                    candidate: armd::rt_core::config_iface::CandidateId(1),
                    stamp: armd::rt_core::broker::Stamp {
                        arm: latest.rt.owner.arm,
                        boot: latest.rt.owner.boot,
                        request: armd::rt_core::broker::RequestId(1),
                        client: ClientId(self.conn_id),
                        client_seq: 1,
                        authority: latest.rt.owner.authority,
                        deadline: Instant::now() + Duration::from_secs(5),
                    },
                };
                let plan = ConfigCommitPlan {
                    proposal,
                    auth,
                    precheck: ReservePrecheck::GraspSetup {
                        new_camera_tilt_deg: new_setup.camera_tilt_deg as f32,
                        allow_during_calibration_busy: false,
                    },
                    candidate: ConfigCandidate::GraspSetup {
                        setup: new_setup.clone(),
                        ws_eval_bufs: None,
                    },
                    write: ConfigWriteOp::Volatile,
                    expected_workspace_revision: None,
                    expected_poses_revision: None,
                };
                run_config_commit(
                    &self.sup,
                    plan,
                    || Ok(()),
                    move |pub_cfg, act| {
                        pub_cfg.config.grasp = Some(new_setup);
                        pub_cfg.grasp_setup_revision = act.revision;
                    },
                )
                .await
                .expect("executor grasp_setup commit");
            }
        }
    }

    fn shutdown(mut self) {
        self.ticker_stop.store(true, Ordering::Release);
        if let Some(h) = self.handle.take() {
            h.stop_and_join();
        }
    }
}

// ============================================================================
// 1. Oblique Geometry End-to-End + J7 Intent Invariance + Place Handoff
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_oblique_grasp_and_place_both_owners() {
    for owner in Owner::BOTH {
        let (cfg, setup) = fast_mounted_config();
        let gp = cfg.gripper_policy.clone().unwrap();
        let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);
        let q0 = solve_camera_down(&cfg, &setup, [0.42, 0.0, 0.26]);
        let poly = banana([0.42, 0.0], 0.12, 30.0, 0.034, 90.0);
        let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;

        let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
        let mut dry_req = req.clone();
        dry_req["dry_run"] = json!(true);
        let dry = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &dry_req).await.fields();
        assert_eq!(dry["reachable"], json!(true), "{owner:?} dry: {dry:?}");
        assert_eq!(dry["legacy"]["reachable"], json!(false), "far banana must fail legacy IK");
        assert_eq!(dry["legacy"]["code"], json!("IK_UNREACHABLE"));
        assert_eq!(dry["strategy"], json!("adaptive"));
        assert_eq!(dry["outcome"], json!("selected"));
        let tilt = dry["selected_candidate"]["tilt_deg"].as_f64().unwrap();
        assert!(tilt > 5.0, "{owner:?}: expected oblique tilt > 5 deg, got {tilt}");
        assert!(dry["expected_duration_s"].as_f64().unwrap() > 0.0);

        let planned = rig.plan_adaptive(&req).await.expect("adaptive plan");
        let route = planned.adaptive.as_ref().unwrap().route.clone();
        assert!(route.tilt_deg > 5.0, "live route must select oblique pose");
        assert!(
            (route.budget.nominal_total_s - dry["expected_duration_s"].as_f64().unwrap()).abs() < 1e-3,
            "dry-run / live duration parity"
        );

        let tx_start = rig.sim.lock().unwrap().tx_log.len();
        let (id, _dur) = rig.start_adaptive(planned).await.expect("begin adaptive");
        let (state, code, holding) = rig.wait_action_end(id, Duration::from_secs(20)).await;
        assert_eq!((state.as_str(), code.as_deref(), holding), ("done", None, Some(true)), "{owner:?}");
        assert_eq!(rig.held_label().await.as_deref(), Some("banana"), "{owner:?}");
        if owner == Owner::Executor {
            assert_eq!(rig.executor_terminal_count(id), 1, "exactly one terminal");
        }

        // Verify J7 never moves simultaneously with arm joints (arm-only children lock J7;
        // gripper-only children move J7 while arm holds still).
        let log = rig.sim.lock().unwrap().tx_log[tx_start..].to_vec();
        let mut arm_only_ticks = 0usize;
        let mut j7_only_ticks = 0usize;
        for w in log.windows(2) {
            if !w[0].1 || !w[1].1 {
                continue;
            }
            let arm_d = (0..6).map(|i| (w[1].3[i] - w[0].3[i]).abs()).fold(0.0f32, f32::max);
            let j7_d = (w[1].3[6] - w[0].3[6]).abs();
            if arm_d > 5e-4 {
                arm_only_ticks += 1;
                assert!(
                    j7_d < 1e-4,
                    "{owner:?}: J7 moved ({j7_d}) during arm motion (arm_d={arm_d}) — J7 must be intent-only"
                );
            } else if j7_d > 5e-4 {
                j7_only_ticks += 1;
            }
        }
        assert!(arm_only_ticks > 10 && j7_only_ticks > 5, "{owner:?}: arm={arm_only_ticks}, j7={j7_only_ticks}");

        // Verify lifted TCP orientation preserved the oblique candidate orientation (never yaw_outward / look-down).
        let q_end = rig.measured_positions().await;
        let end_pose = cfg.arm_model.fk_f32(&q_end).unwrap();
        let tcp_end = armd::motion::end_to_tcp(&end_pose, &cfg.active_tool.tcp_offset_m);
        let rot_err_deg = kinematics::angle_between(&tcp_end.r, &route.plan.lift.r).to_degrees();
        assert!(rot_err_deg < 3.0, "{owner:?}: oblique orientation drifted by {rot_err_deg:.2} deg");

        // Follow-up `place` consumes `HeldObject` and clears `held` on BOTH owners!
        let place_body = json!({
            "position": [0.28, 0.08, setup.table_z_m + 0.04],
            "approach_height_m": 0.05,
            "release": true,
            "retreat": true
        });
        let place_id = match owner {
            Owner::Legacy => {
                let pending_p = {
                    let inner = rig.sup.inner().read().await;
                    place::prepare(&inner, &place_body, inner.safety.epoch_id()).unwrap()
                };
                let planned_p = place::plan(pending_p).unwrap();
                let adm = rig.admission_stamp().await;
                let mut inner = rig.sup.inner().write().await;
                place::begin_with_stamp(&mut inner, adm, &rig.sup, planned_p).unwrap().0
            }
            Owner::Executor => {
                let adm = rig.admission_stamp().await;
                let snap = ActionSnapshot::capture(&rig.sup, adm, Instant::now() + DEFAULT_ACTION_DEADLINE).await.unwrap();
                let pending_p = place::prepare_from_snapshot(&snap, &place_body, snap.owner.authority.epoch).unwrap();
                let planned_p = place::plan(pending_p).unwrap();
                place::begin_executor(&rig.sup, adm, planned_p).await.unwrap().0
            }
        };
        let (p_state, p_code, _) = rig.wait_action_end(place_id, Duration::from_secs(20)).await;
        assert_eq!((p_state.as_str(), p_code.as_deref()), ("done", None), "{owner:?}: place failed");
        assert_eq!(rig.held_label().await, None, "{owner:?}: place must clear HeldObject");

        rig.shutdown();
    }
}

// ============================================================================
// 2. Swept-Joint Pregrasp Fallback + J7 Lock on Both Owners
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_swept_joint_pregrasp_fallback_locks_j7_both_owners() {
    // First compute a deterministic scene where straight Cartesian `start -> pregrasp` hits a
    // keep-out box placed on the Cartesian chord midpoint while swept-joint arcs cleanly around it.
    let (base_cfg, setup) = fast_mounted_config();
    let cam = CamModel::assumed(&base_cfg.arm_model, &setup, &base_cfg.camera_mount);
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);

    // Solve camera-down above [0.30, 0.0, 0.30] on the wound-J6 branch (`q0[5] ≈ -5.93 rad`,
    // near the lower J6 limit `-6.2 rad`, identical camera pose & view): straight-line Cartesian
    // `q0 -> pregrasp` hits the J6 limit / branch flip, so `select_adaptive_route` MUST select
    // `PregraspMode::SweptJoint` (`plan_swept_joint_path` with `lock_j7_hold: true`).
    let fk_ins = base_cfg.arm_model.fk(&INSPECT);
    let lens = [0.30, 0.0, 0.30];
    let r0 = grasp::look_down(&fk_ins.r);
    let r = grasp::camera_down(&r0, &cam.rec);
    let off = kinematics::mat_vec(&r, &cam.offset);
    let goal = Pose::new(r, kinematics::sub(&lens, &off));
    let lo = [-3.1, -2.05, -2.6, -3.1, -2.05, -6.2];
    let hi = [3.1, 2.05, 2.6, 3.1, 2.05, 6.2];
    let mut q0 = [0.0, -1.5, -1.5, 1.7, 1.5, -3.1];
    let mut solved = false;
    for _ in 0..300 {
        let sol = base_cfg.arm_model.solve_dls(&goal, &q0, &lo, &hi, kinematics::PAR.w);
        q0 = sol.q;
        if sol.ok {
            solved = true;
            break;
        }
    }
    assert!(solved, "wound-J6 camera-down start must solve cleanly");
    let start_end = base_cfg.arm_model.fk(&q0);
    let req = adaptive_request(&cam, &start_end, setup.table_z_m, &poly, 0.035);

    let gp = base_cfg.gripper_policy.clone().unwrap();
    let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);

    for owner in Owner::BOTH {
        let rig = Rig::start(owner, base_cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
        let mut dry_req = req.clone();
        dry_req["dry_run"] = json!(true);
        let dry = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &dry_req).await.fields();
        assert_eq!(dry["reachable"], json!(true), "{owner:?}: {dry:?}");
        assert_eq!(dry["pregrasp_mode"], json!("swept_joint"), "{owner:?}: {dry:?}");

        let planned = rig.plan_adaptive(&req).await.expect("swept-joint adaptive plan");
        assert_eq!(planned.adaptive.as_ref().unwrap().route.pregrasp_mode, PregraspMode::SweptJoint);

        let tx_start = rig.sim.lock().unwrap().tx_log.len();
        let (id, _) = rig.start_adaptive(planned).await.expect("begin swept-joint");
        // Unwinding ~6 rad on J6 + pre_open + insert + close + lift takes ~22 s wall-clock on
        // real-time Executor, so allow a 45 s test wait budget.
        let (state, code, holding) = rig.wait_action_end(id, Duration::from_secs(45)).await;
        assert_eq!((state.as_str(), code.as_deref(), holding), ("done", None, Some(true)), "{owner:?}");

        // Verify J7 stayed identically locked during swept-joint pregrasp and all arm children.
        let log = rig.sim.lock().unwrap().tx_log[tx_start..].to_vec();
        for w in log.windows(2) {
            if !w[0].1 || !w[1].1 {
                continue;
            }
            let arm_d = (0..6).map(|i| (w[1].3[i] - w[0].3[i]).abs()).fold(0.0f32, f32::max);
            let j7_d = (w[1].3[6] - w[0].3[6]).abs();
            if arm_d > 5e-4 {
                assert!(j7_d < 1e-4, "{owner:?}: J7 drifted ({j7_d}) during swept-joint arm motion");
            }
        }
        rig.shutdown();
    }
}

// ============================================================================
// 3. Keep-Outs: Thin Box Piercing Carried-Object Face (All 8 Corners Outside)
//    During Separation & Lift + Mid-Transition Keep-Out + Checked-Only Hints
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_keepout_carried_object_sat_and_checked_only_hints() {
    let (base_cfg, setup) = fast_mounted_config();
    let q0 = solve_camera_down(&base_cfg, &setup, [0.30, 0.0, 0.26]);
    let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
    let cam = CamModel::assumed(&base_cfg.arm_model, &setup, &base_cfg.camera_mount);
    let start_end = base_cfg.arm_model.fk(&q0);
    let req = adaptive_request(&cam, &start_end, setup.table_z_m, &poly, 0.035);

    // 3a) Thin keep-out piercing the carried object's middle during SEPARATION (+0.03 m above table)
    //     with all 8 object corners outside the thin rod!
    {
        let mut cfg = base_cfg.clone();
        // Object spans roughly x in [0.27, 0.33], y in [-0.06, 0.06], z in [0.002, 0.037] -> during
        // separation (+0.03 m), object passes through z = 0.055 at (0.30, 0.0) while all corners are
        // well outside this thin 6 mm x 6 mm rod!
        cfg.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
            name: "thin_sep_rod".into(),
            min_m: [0.297, -0.003, 0.050],
            max_m: [0.303, 0.003, 0.062],
        });
        let rig = Rig::start(Owner::Legacy, cfg, setup.clone(), q0, poly.clone(), 0.035, Some(-1.5), false).await;
        let mut dry_req = req.clone();
        dry_req["dry_run"] = json!(true);
        let dry = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &dry_req).await.fields();
        assert_eq!(dry["reachable"], json!(false), "thin rod piercing carried object during separation must be rejected: {dry:?}");
        assert!(dry.get("failing_phase").and_then(Value::as_str).is_some(), "{dry:?}");
        // Live plan must refuse with exact parity (failing_phase + code + checked-only hint).
        let err = rig.plan_adaptive(&req).await.err().expect("live plan must also refuse");
        match err {
            ArmError::Detailed { code, details, .. } => {
                assert_eq!(Value::String(code), dry["code"]);
                assert_eq!(details["failing_phase"], dry["failing_phase"]);
                // Because legacy checked reachable in this scene, hint is present ONLY because legacy was checked!
                assert_eq!(details.get("hint"), dry.get("hint"));
            }
            other => panic!("expected ArmError::Detailed, got {other:?}"),
        }
        rig.shutdown();
    }

    // 3b) Thin keep-out piercing carried object during LIFT (z = 0.11 m, above separation but below lift_height_m=0.12)
    {
        let mut cfg = base_cfg.clone();
        cfg.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
            name: "thin_lift_rod".into(),
            min_m: [0.297, -0.003, 0.102],
            max_m: [0.303, 0.003, 0.114],
        });
        let rig = Rig::start(Owner::Legacy, cfg, setup.clone(), q0, poly.clone(), 0.035, Some(-1.5), false).await;
        let mut dry_req = req.clone();
        dry_req["dry_run"] = json!(true);
        let dry = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &dry_req).await.fields();
        assert_eq!(dry["reachable"], json!(false), "thin rod piercing carried object during lift must be rejected: {dry:?}");
        assert!(rig.plan_adaptive(&req).await.is_err());
        rig.shutdown();
    }

    // 3c) No checked alternative -> `hint` MUST be omitted/null (never guess unchecked hints!)
    {
        let mut cfg = base_cfg.clone();
        // Huge keepout blocking EVERYTHING (both legacy and adaptive).
        cfg.workspace.as_mut().unwrap().keepout_boxes.push(KeepoutBox {
            name: "total_wall".into(),
            min_m: [0.20, -0.20, 0.01],
            max_m: [0.45, 0.20, 0.22],
        });
        let rig = Rig::start(Owner::Legacy, cfg, setup.clone(), q0, poly.clone(), 0.035, Some(-1.5), false).await;
        let mut dry_req = req.clone();
        dry_req["dry_run"] = json!(true);
        let dry = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &dry_req).await.fields();
        assert_eq!(dry["reachable"], json!(false));
        assert_eq!(dry["legacy"]["reachable"], json!(false));
        assert!(dry.get("hint").is_none() || dry["hint"].is_null(), "must NOT emit unchecked hint: {dry:?}");
        let err = rig.plan_adaptive(&req).await.err().expect("must refuse");
        if let ArmError::Detailed { details, .. } = err {
            assert!(details.get("hint").is_none(), "live error must NOT emit unchecked hint: {details:?}");
        } else {
            panic!("expected Detailed error, got {err:?}");
        }
        rig.shutdown();
    }
}

// ============================================================================
// 4. Contact vs Empty (Closed-Jaw Withdrawal) vs Early Obstruction vs Drop
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_empty_early_contact_and_drop_no_retry_both_owners() {
    for owner in Owner::BOTH {
        let (cfg, mut setup) = fast_mounted_config();
        let gp = cfg.gripper_policy.clone().unwrap();
        let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);
        let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.26]);
        let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);

        // 4a) Empty grasp -> NOT_HOLDING + closed-jaw withdrawal (NEVER open jaws at contact!)
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, None, false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            let pre_p = planned.adaptive.as_ref().unwrap().route.plan.pregrasp.p;
            let tx_start = rig.sim.lock().unwrap().tx_log.len();
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, holding) = rig.wait_action_end(id, Duration::from_secs(20)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("NOT_HOLDING")), "{owner:?}");
            assert_ne!(holding, Some(true));
            assert_eq!(rig.held_label().await, None);
            // Assert arm withdrew back to pregrasp AND J7 stayed closed (NEVER opened after close)!
            let q_end = rig.measured_positions().await;
            let tcp_end = armd::motion::end_to_tcp(&cfg.arm_model.fk_f32(&q_end).unwrap(), &cfg.active_tool.tcp_offset_m);
            let d_pre = kinematics::norm(&kinematics::sub(&tcp_end.p, &pre_p));
            assert!(d_pre < 0.015, "{owner:?}: empty grasp must withdraw to pregrasp (dist={d_pre:.4})");
            assert!(
                (q_end[6] - gp.closed_rad).abs() < 0.20,
                "{owner:?}: jaws must remain CLOSED during empty withdrawal (got j7={}, closed={})",
                q_end[6],
                gp.closed_rad
            );
            // Check TX log after pre_open reached open_rad and then close reached closed_rad:
            // J7 NEVER moved back toward open_rad during withdrawal!
            let log = rig.sim.lock().unwrap().tx_log[tx_start..].to_vec();
            let first_open = log
                .iter()
                .position(|(_, en, _, q)| *en && (q[6] - gp.open_rad).abs() < 0.30)
                .expect("pre_open must open jaws");
            let first_closed_after_open = log[first_open..]
                .iter()
                .position(|(_, en, _, q)| *en && (q[6] - gp.closed_rad).abs() < 0.15)
                .map(|idx| first_open + idx)
                .expect("close must reach closed_rad after pre_open");
            for (_, en, _, q) in &log[first_closed_after_open..] {
                if *en {
                    assert!(
                        (q[6] - gp.closed_rad).abs() < 0.20,
                        "{owner:?}: jaws opened at contact during NOT_HOLDING recovery! j7={}",
                        q[6]
                    );
                }
            }
            assert_eq!(rig.sim.lock().unwrap().close_transitions, 1, "{owner:?}: NO automatic retry");
            rig.shutdown();
        }

        // 4b) Early unexpected contact outside contact_window_m -> COLLISION + open-jaw withdrawal
        {
            setup.descend_contact_stop_nm = Some(1.5);
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            let route = planned.adaptive.as_ref().unwrap().route.clone();
            rig.sim.lock().unwrap().contact = Some(ContactInject {
                pregrasp_tcp: route.plan.pregrasp.p,
                grasp_tcp: route.plan.grasp.p,
                trigger_from_pregrasp_m: 0.012,
                min_rem_to_grasp_m: route.contact_window_m + 0.010,
                residual_nm: 3.5,
                armed: false,
                fired: false,
            });
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(20)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("COLLISION")), "{owner:?}");
            assert_eq!(rig.held_label().await, None);
            // Jaws never closed, and arm withdrew open-jaw back to pregrasp.
            assert_eq!(rig.sim.lock().unwrap().close_transitions, 0, "{owner:?}: must not close after early collision");
            let q_end = rig.measured_positions().await;
            let tcp_end = armd::motion::end_to_tcp(&cfg.arm_model.fk_f32(&q_end).unwrap(), &cfg.active_tool.tcp_offset_m);
            let d_pre = kinematics::norm(&kinematics::sub(&tcp_end.p, &route.plan.pregrasp.p));
            assert!(d_pre < 0.020, "{owner:?}: early collision must withdraw back to pregrasp (dist={d_pre:.4})");
            rig.shutdown();
            setup.descend_contact_stop_nm = None;
        }

        // 4c) Post-lift drop -> DROPPED with ZERO automatic regrip retry!
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            let drop_z = planned.adaptive.as_ref().unwrap().route.plan.separation.p[2] + 0.01;
            rig.sim.lock().unwrap().drop_after_z = Some(drop_z);
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, holding) = rig.wait_action_end(id, Duration::from_secs(20)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("DROPPED")), "{owner:?}");
            assert_ne!(holding, Some(true));
            assert_eq!(rig.held_label().await, None);
            assert_eq!(
                rig.sim.lock().unwrap().close_transitions,
                1,
                "{owner:?}: adaptive DROPPED must NOT perform automatic regrip retry"
            );
            rig.shutdown();
        }
    }
}

// ============================================================================
// 5. Lease-Free Executor Dry-Run Parity + Budget / Payload Refusal Parity
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_dry_run_parity_and_lease_free_executor() {
    let (cfg, setup) = fast_mounted_config();
    let q0 = solve_camera_down(&cfg, &setup, [0.42, 0.0, 0.26]);
    let poly = banana([0.42, 0.0], 0.12, 30.0, 0.034, 90.0);

    // Enable leases on OWNER_EXECUTOR and run dry_run_task WITHOUT holding a lease!
    let rig = Rig::start(Owner::Executor, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(-1.5), true).await;
    // Release lease so NO client holds the lease while dry_run_task executes.
    let mut dry_req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
    dry_req["dry_run"] = json!(true);

    // 5a) Lease-free Executor adaptive dry-run succeeds and matches live plan!
    let dry = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &dry_req).await.fields();
    assert_eq!(dry["reachable"], json!(true), "{dry:?}");
    assert_eq!(dry["strategy"], json!("adaptive"));
    assert_eq!(dry["outcome"], json!("selected"));

    // 5b) Tight parent timeout_s (while every individual child fits within phase_timeout_s) ->
    //     GRASP_TIMEOUT in failing_phase "budget" (parity!)
    let nom_total = dry["expected_duration_s"].as_f64().unwrap();
    let tight_timeout = (nom_total * 0.75).max(5.0);
    let mut tight_budget = dry_req.clone();
    tight_budget["overrides"] = json!({
        "phase_timeout_s": tight_timeout,
        "timeout_s": tight_timeout
    });
    let dry_to = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &tight_budget).await.fields();
    assert_eq!(dry_to["reachable"], json!(false), "{dry_to:?}");
    assert_eq!(dry_to["code"], json!("GRASP_TIMEOUT"), "{dry_to:?}");
    assert_eq!(dry_to["failing_phase"], json!("budget"), "{dry_to:?}");

    let mut live_tight = tight_budget.clone();
    live_tight.as_object_mut().unwrap().remove("dry_run");
    let err = rig.plan_adaptive(&live_tight).await.err().expect("live must match dry-run budget refusal");
    match err {
        ArmError::Detailed { code, details, .. } => {
            assert_eq!(code, "GRASP_TIMEOUT");
            assert_eq!(details["failing_phase"], json!("budget"));
        }
        other => panic!("expected Detailed GRASP_TIMEOUT, got {other:?}"),
    }

    // 5c) Excess payload_kg (valid range [0, 5] kg, exceeds derated J2 holding torque at 0.42 m) -> LIFT_TORQUE parity
    let mut heavy = dry_req.clone();
    heavy["evidence"]["payload_kg"] = json!(4.5);
    let dry_heavy = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &heavy).await.fields();
    assert_eq!(dry_heavy["reachable"], json!(false), "{dry_heavy:?}");
    assert_eq!(dry_heavy["code"], json!("LIFT_TORQUE"), "{dry_heavy:?}");
    assert_eq!(dry_heavy["failing_phase"], json!("lift"), "{dry_heavy:?}");

    // 5d) Legacy dry-run on OWNER_EXECUTOR stays completely untouched (zero adaptive keys!)
    let mut legacy_dry = dry_req.clone();
    legacy_dry.as_object_mut().unwrap().remove("strategy");
    let leg = grasp::dry_run_task(&rig.sup, rig.vision.as_ref(), &legacy_dry).await.fields();
    assert!(!leg.contains_key("strategy") && !leg.contains_key("failing_phase") && !leg.contains_key("selected_candidate"), "{leg:?}");

    rig.shutdown();
}

// ============================================================================
// 6. Stale Target / Generation Change / Displacement / Rotation / Duplicate /
//    Concurrent Revisions + FinishMacro Revision Race on Both Owners
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_revalidation_and_revision_fencing_both_owners() {
    for owner in Owner::BOTH {
        let (cfg, setup) = fast_mounted_config();
        let gp = cfg.gripper_policy.clone().unwrap();
        let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);
        let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.26]);
        let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);

        // 6a) Stale / cached frames -> TARGET_LOST before any arm motion
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.vision.state.lock().unwrap().stale_frames = true;
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("TARGET_LOST")), "{owner:?}: stale frame");
            rig.shutdown();
        }

        // 6b) Repeated request frame_id (frame_id <= 7) -> TARGET_LOST
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.vision.state.lock().unwrap().repeat_frame_id = Some(7);
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("TARGET_LOST")), "{owner:?}: duplicate frame_id");
            rig.shutdown();
        }

        // 6c) model_generation change between preflight and revalidation -> DETECTION_UNAVAILABLE
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.vision.state.lock().unwrap().frame_generation = Some(2);
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("DETECTION_UNAVAILABLE")), "{owner:?}: model_gen swap");
            rig.shutdown();
        }

        // 6d) Shifted target -> TARGET_DISPLACED
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.vision.state.lock().unwrap().shift_xy = [0.035, 0.0];
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("TARGET_DISPLACED")), "{owner:?}: shifted");
            rig.shutdown();
        }

        // 6e) Rotated elongated target (45 deg) -> TARGET_DISPLACED
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.vision.state.lock().unwrap().rotate_deg = 45.0;
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("TARGET_DISPLACED")), "{owner:?}: rotated");
            rig.shutdown();
        }

        // 6f) Ambiguous duplicate same-label detection -> TARGET_DISPLACED
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.vision.state.lock().unwrap().duplicate_near = true;
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("TARGET_DISPLACED")), "{owner:?}: duplicate");
            rig.shutdown();
        }

        // 6g) Concurrent grasp_setup_revision change BEFORE BeginMacro -> PLAN_STALE refusal!
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            rig.bump_grasp_setup_revision().await;
            let err = rig.start_adaptive(planned).await.expect_err("stale revision must refuse BeginMacro");
            let is_plan_stale = matches!(
                err,
                ArmError::Motion(armd::motion::MotionError::PlanStale(_))
            ) || matches!(
                err,
                ArmError::Rejected { ref code, .. } if code == "PLAN_STALE"
            );
            assert!(is_plan_stale, "{owner:?}: expected PLAN_STALE, got {err:?}");
            rig.shutdown();
        }

        // 6h) Concurrent grasp_setup_revision change MID-GRASP -> PLAN_STALE, held NOT set!
        {
            let mut slow_setup = setup.clone();
            slow_setup.post_lift_settle_ms = 250;
            let rig = Rig::start(owner, cfg.clone(), slow_setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, slow_setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            let (id, _) = rig.start_adaptive(planned).await.unwrap();
            tokio::time::sleep(Duration::from_millis(120)).await;
            rig.bump_grasp_setup_revision().await;
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(15)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("PLAN_STALE")), "{owner:?}: mid-grasp revision bump");
            assert_eq!(rig.held_label().await, None, "{owner:?}: held must NOT be committed on revision bump");
            if owner == Owner::Executor {
                assert_eq!(rig.executor_terminal_count(id), 1);
            }
            rig.shutdown();
        }

        // 6i) Direct FinishMacro(Done, HeldUpdate::Set) revision race on BOTH owners:
        //     mutating grasp_setup_revision right before `finish_fenced` MUST refuse Done without
        //     setting `held`, allowing fallback `finish(Failed)` to end the parent ONCE!
        {
            let rig = Rig::start(owner, cfg.clone(), setup.clone(), q0, poly.clone(), 0.035, Some(obj_j7), false).await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let planned = rig.plan_adaptive(&req).await.unwrap();
            let rev = planned.adaptive.as_ref().unwrap().revisions;
            let adm = rig.admission_stamp().await;
            let dl = Instant::now() + Duration::from_secs(30);
            let id = match owner {
                Owner::Legacy => {
                    let mut inner = rig.sup.inner().write().await;
                    let admitted = inner.check_admission(adm).unwrap();
                    SupervisorInner::begin_macro(admitted, "grasp", &planned.input, None).unwrap()
                }
                Owner::Executor => {
                    let h = rig.handle.as_ref().unwrap();
                    let snap = ActionSnapshot::capture(&rig.sup, adm, dl).await.unwrap();
                    let mut spec = grasp::begin_macro_spec(&planned, h.boot(), 7, Some(6), dl).unwrap();
                    spec.admission_request_id = adm.request_id;
                    let mut stamp = snap.stamp;
                    stamp.grasp_setup_revision = Some(rev.grasp_setup_revision);
                    stamp.gripper_calib_revision = Some(rev.gripper_calib_revision);
                    stamp.calibration_revision = Some(rev.calibration_revision);
                    stamp.power_budget_revision = Some(rev.power_budget_revision);
                    armd::macro_action::begin_macro_executor(&rig.sup, &stamp, spec, dl)
                        .await
                        .unwrap()
                        .0
                }
            };
            // Now bump grasp_setup_revision BEFORE calling finish_fenced(Done, HeldUpdate::Set)!
            rig.bump_grasp_setup_revision().await;
            let ctx = armd::macro_action::MacroCtx {
                sup: rig.sup.clone(),
                id,
                stamp: adm,
                deadline: dl,
                phase_timeout_s: 30.0,
                timeout_code: "GRASP_TIMEOUT",
                timeout_message: "timeout".into(),
            };
            let res = ctx
                .finish_fenced(
                    FinishState::Done,
                    None,
                    None,
                    Some(true),
                    Some(0.4),
                    HeldUpdate::Set {
                        label: armd::model::HolderLabel::new("banana"),
                        spill_safe: false,
                    },
                    Some(rev),
                )
                .await;
            assert_eq!(res, Err("PLAN_STALE"), "{owner:?}: fenced FinishMacro(Done) must refuse PLAN_STALE");
            assert_eq!(rig.held_label().await, None, "{owner:?}: refused FinishMacro(Done) must NOT commit held");
            if owner == Owner::Executor {
                assert_eq!(rig.executor_terminal_count(id), 0, "no terminal emitted on refused FinishMacro");
            }
            // Fallback failed finish terminates parent exactly once with HeldUpdate::Clear.
            ctx.finish(
                FinishState::Failed,
                Some("PLAN_STALE"),
                Some("revision raced".into()),
                Some(false),
                None,
                HeldUpdate::Clear,
            )
            .await;
            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(5)).await;
            assert_eq!((state.as_str(), code.as_deref()), ("failed", Some("PLAN_STALE")), "{owner:?}");
            assert_eq!(rig.held_label().await, None);
            if owner == Owner::Executor {
                assert_eq!(rig.executor_terminal_count(id), 1, "exactly one terminal after fallback finish");
            }
            let _ = FinishMacroSpec::done(ActionId(id), armd::rt_core::broker::BootId(0));
            rig.shutdown();
        }
    }
}

// ============================================================================
// 7. Boundary Stop / E-Stop / Fault / LeaseLoss / Timeout Matrix (Both Owners)
//    Single Terminal + Zero Motion TX After Effective Cutoff
// ============================================================================

#[derive(Debug, Clone, Copy)]
enum BoundaryEvent {
    Stop,
    EStop,
    Fault,
    LeaseLoss,
    Timeout,
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_boundary_stop_estop_fault_lease_timeout_matrix_both_owners() {
    let events = [
        BoundaryEvent::Stop,
        BoundaryEvent::EStop,
        BoundaryEvent::Fault,
        BoundaryEvent::LeaseLoss,
        BoundaryEvent::Timeout,
    ];
    for owner in Owner::BOTH {
        for ev in events {
            let (cfg, mut setup) = fast_mounted_config();
            // Slow arm slightly so the active child is reliably in flight when we inject the event.
            setup.approach_speed_mps = 0.04;
            setup.descend_speed_mps = 0.04;
            setup.settle_delay_s = 0.01;
            let gp = cfg.gripper_policy.clone().unwrap();
            let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);
            let q0 = solve_camera_down(&cfg, &setup, [0.30, 0.0, 0.26]);
            let poly = banana([0.30, 0.0], 0.12, 30.0, 0.034, 90.0);
            let need_lease = matches!(ev, BoundaryEvent::LeaseLoss | BoundaryEvent::Stop);

            let rig = Rig::start_with_pos_force(
                owner,
                cfg.clone(),
                setup.clone(),
                q0,
                poly.clone(),
                0.035,
                Some(obj_j7),
                need_lease,
                true,
            )
            .await;
            let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
            let (planned, adm) = rig.plan_and_stamp_adaptive(&req).await.unwrap();

            // Subscribe to lossless Legacy action terminals BEFORE BeginMacro so we assert exact
            // terminal count == 1 on BOTH Legacy and Executor (not merely liveness).
            let legacy_terminals = Arc::new(Mutex::new(Vec::<u64>::new()));
            let _term_sub = if owner == Owner::Legacy {
                let log_t = Arc::clone(&legacy_terminals);
                Some(rig.sup.test_subscribe_action_terminals(move |obs| {
                    if let Some(a) = obs.action.as_ref() {
                        if a.state != "running" {
                            log_t.lock().unwrap().push(a.id);
                        }
                    }
                    true
                }))
            } else {
                None
            };

            let id = match ev {
                BoundaryEvent::Timeout => {
                    // Use production `BeginMacroSpec` + short deadline on BOTH Legacy and Executor
                    // (900 ms lets `pregrasp` begin moving well past 0.005 rad while firing
                    // `GRASP_TIMEOUT` mid-child on the 0.04 m/s transition).
                    let short_dl = Instant::now() + Duration::from_millis(900);
                    match owner {
                        Owner::Legacy => {
                            let mut inner = rig.sup.inner().write().await;
                            let ws_rev = planned.adaptive.as_ref().unwrap().revisions.workspace_revision;
                            let boot = OwnerCheckCtx::for_legacy(&inner).boot;
                            let mut spec =
                                grasp::begin_macro_spec(&planned, boot, 7, Some(6), short_dl)
                                    .unwrap();
                            spec.admission_request_id = adm.request_id;
                            let admitted = inner.check_admission(adm).unwrap();
                            let aid = install_begin_macro_admitted(
                                admitted,
                                &spec,
                                Some(ws_rev),
                                None,
                                None,
                            )
                            .unwrap();
                            drop(inner);
                            grasp::spawn_with_deadline(
                                rig.sup.clone(),
                                rig.vision.clone(),
                                aid,
                                planned,
                                adm,
                                short_dl,
                            );
                            aid
                        }
                        Owner::Executor => {
                            let h = rig.handle.as_ref().unwrap();
                            let snap = ActionSnapshot::capture(
                                &rig.sup,
                                adm,
                                Instant::now() + DEFAULT_ACTION_DEADLINE,
                            )
                            .await
                            .unwrap();
                            let mut spec =
                                grasp::begin_macro_spec(&planned, h.boot(), 7, Some(6), short_dl)
                                    .unwrap();
                            spec.admission_request_id = adm.request_id;
                            let rev = planned.adaptive.as_ref().unwrap().revisions;
                            let mut stamp = snap.stamp;
                            stamp.grasp_setup_revision = Some(rev.grasp_setup_revision);
                            stamp.gripper_calib_revision = Some(rev.gripper_calib_revision);
                            stamp.calibration_revision = Some(rev.calibration_revision);
                            stamp.power_budget_revision = Some(rev.power_budget_revision);
                            let (aid, _) = armd::macro_action::begin_macro_executor(
                                &rig.sup,
                                &stamp,
                                spec,
                                Instant::now() + DEFAULT_ACTION_DEADLINE,
                            )
                            .await
                            .unwrap();
                            grasp::spawn_with_deadline(
                                rig.sup.clone(),
                                rig.vision.clone(),
                                aid,
                                planned,
                                adm,
                                short_dl,
                            );
                            aid
                        }
                    }
                }
                _ => rig.start_adaptive_with_stamp(planned, adm).await.unwrap().0,
            };

            // Wait deterministically until the first arm child (`pregrasp`) has actually moved!
            let move_dl = Instant::now() + Duration::from_secs(2);
            let mut moved = 0.0f32;
            while Instant::now() < move_dl {
                let q = rig.measured_positions().await;
                moved = (0..6)
                    .map(|i| (q[i] - q0[i] as f32).abs())
                    .fold(0.0f32, f32::max);
                if moved > 0.005 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            assert!(
                moved > 0.005,
                "{owner:?} / {ev:?}: arm child never began moving before boundary injection (moved={moved})"
            );

            // Inject boundary event.
            match ev {
                BoundaryEvent::Stop => {
                    match owner {
                        Owner::Legacy => {
                            let mut inner = rig.sup.inner().write().await;
                            inner.stop_motion();
                        }
                        Owner::Executor => {
                            let auth = rig.sup.hub().latest().unwrap().rt.owner.authority;
                            let out = rig
                                .sup
                                .urgent_stop(UrgentStop {
                                    connection: armd::model::HolderLabel::new("connection:41"),
                                    lease_generation: auth.lease_generation,
                                    epoch: Some(auth.epoch),
                                    request_id: 900,
                                    request_id_valid: true,
                                    body_error: None,
                                })
                                .await
                                .expect("executor urgent_stop");
                            assert!(
                                matches!(out, armd::rt_core::urgent::UrgentStopOutcome::Applied { .. }),
                                "executor urgent_stop must apply, got {out:?}"
                            );
                        }
                    }
                }
                BoundaryEvent::EStop => {
                    rig.sup.trigger_e_stop_checked().await.unwrap();
                }
                BoundaryEvent::Fault => {
                    rig.sim.lock().unwrap().inject_fault = true;
                }
                BoundaryEvent::LeaseLoss => {
                    match owner {
                        Owner::Legacy => {
                            let mut inner = rig.sup.inner().write().await;
                            let _ = inner.release_lease_for_connection("connection:41");
                        }
                        Owner::Executor => {
                            let h = rig.handle.as_ref().unwrap();
                            let auth = rig.sup.hub().latest().unwrap().rt.owner.authority;
                            let ch = ConnectionHandle::from_ws_id(rig.conn_id).unwrap();
                            h.broker()
                                .lock()
                                .unwrap()
                                .submit(
                                    ClientId(rig.conn_id),
                                    901,
                                    auth,
                                    Instant::now() + Duration::from_secs(2),
                                    Submit::Lease(LeaseRequest::Release(LeaseRelease {
                                        connection: ch,
                                        generation: auth.lease_generation,
                                    })),
                                )
                                .unwrap();
                        }
                    }
                }
                BoundaryEvent::Timeout => {
                    // Let the short deadline expire naturally.
                }
            }

            let (state, code, _) = rig.wait_action_end(id, Duration::from_secs(10)).await;
            assert_ne!(state, "done", "{owner:?} / {ev:?}: must not complete normally");
            if matches!(ev, BoundaryEvent::Timeout) {
                assert_eq!(
                    code.as_deref(),
                    Some("GRASP_TIMEOUT"),
                    "{owner:?}: expected GRASP_TIMEOUT, got ({state}, {code:?})"
                );
            }
            assert_eq!(rig.held_label().await, None, "{owner:?} / {ev:?}: held must stay None");

            // Record cutoff IMMEDIATELY at terminal observation (NO 25 ms skip!) and observe 80 ms
            // to assert ZERO motion steps AND ZERO PosForce Target intents after cutoff.
            let (cutoff_idx, pf_cutoff_idx) = {
                let s = rig.sim.lock().unwrap();
                (s.tx_log.len(), s.pf_intent_log.len())
            };
            tokio::time::sleep(Duration::from_millis(80)).await;
            let (post_tx, post_pf) = {
                let s = rig.sim.lock().unwrap();
                (
                    s.tx_log[cutoff_idx..].to_vec(),
                    s.pf_intent_log[pf_cutoff_idx..].to_vec(),
                )
            };
            assert!(
                post_pf
                    .iter()
                    .all(|(_, k, stale, _)| !matches!(k, PosForceIntentKind::Target { .. }) || *stale),
                "{owner:?} / {ev:?}: active PosForce Target emitted after cutoff! {post_pf:?}"
            );
            match ev {
                BoundaryEvent::EStop => {
                    assert!(
                        post_tx.iter().all(|(_, en, is_hold, _)| !*en || *is_hold),
                        "{owner:?} / {ev:?}: non-hold motion TX continued after E-stop cutoff ({} entries)",
                        post_tx.len()
                    );
                    for w in post_tx.windows(2) {
                        let step = (0..7)
                            .map(|i| (w[1].3[i] - w[0].3[i]).abs())
                            .fold(0.0f32, f32::max);
                        assert!(step < 1e-4, "{owner:?} / EStop: position stepped ({step}) after cutoff");
                    }
                }
                BoundaryEvent::Fault => {
                    assert!(
                        post_tx.iter().all(|(_, en, _, _)| !*en),
                        "{owner:?} / {ev:?}: enabled TX continued after Fault cutoff ({} entries)",
                        post_tx.len()
                    );
                }
                BoundaryEvent::Stop | BoundaryEvent::LeaseLoss | BoundaryEvent::Timeout => {
                    for w in post_tx.windows(2) {
                        if w[0].1 && w[1].1 {
                            let step = (0..7)
                                .map(|i| (w[1].3[i] - w[0].3[i]).abs())
                                .fold(0.0f32, f32::max);
                            assert!(
                                step < 1e-4,
                                "{owner:?} / {ev:?}: motion TX step {step} after effective cutoff!"
                            );
                        }
                    }
                }
            }

            if owner == Owner::Executor {
                assert_eq!(
                    rig.executor_terminal_count(id),
                    1,
                    "{owner:?} / {ev:?}: parent macro MUST end exactly once"
                );
            } else {
                let count = legacy_terminals
                    .lock()
                    .unwrap()
                    .iter()
                    .filter(|&&aid| aid == id)
                    .count();
                assert_eq!(
                    count, 1,
                    "Legacy / {ev:?}: parent macro MUST publish exactly one lossless terminal (got {count})"
                );
                assert!(
                    !rig.sup.inner().read().await.macro_alive(id),
                    "legacy macro must not stay alive after {ev:?}"
                );
            }
            let _ = AtomicUsize::new(0);
            rig.shutdown();
        }
    }
}

// ============================================================================
// 8. PosForce Active J7 Intent-Only Ownership + `lock_j7_hold` + Approach Refusal
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_pos_force_j7_intent_only_and_approach_lock_refusal_both_owners() {
    for owner in Owner::BOTH {
        let (cfg, setup) = fast_mounted_config();
        let gp = cfg.gripper_policy.clone().unwrap();
        let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);
        let q0 = solve_camera_down(&cfg, &setup, [0.42, 0.0, 0.26]);
        let poly = banana([0.42, 0.0], 0.12, 30.0, 0.034, 90.0);

        let rig = Rig::start_with_pos_force(
            owner,
            cfg.clone(),
            setup.clone(),
            q0,
            poly.clone(),
            0.035,
            Some(obj_j7),
            false,
            true,
        )
        .await;
        let req = adaptive_request(&rig.cam, &rig.start_end, setup.table_z_m, &poly, 0.035);
        let (planned, adm) = rig.plan_and_stamp_adaptive(&req).await.expect("adaptive plan");

        let (tx_start, pf_start) = {
            let s = rig.sim.lock().unwrap();
            (s.tx_log.len(), s.pf_intent_log.len())
        };
        let (id, _) = rig.start_adaptive_with_stamp(planned, adm).await.expect("begin adaptive");
        let (state, code, holding) = rig.wait_action_end(id, Duration::from_secs(20)).await;
        assert_eq!(
            (state.as_str(), code.as_deref(), holding),
            ("done", None, Some(true)),
            "{owner:?}: PosForce adaptive grasp must succeed with holding=true"
        );
        assert_eq!(rig.held_label().await.as_deref(), Some("banana"), "{owner:?}");

        let pf_log = rig.sim.lock().unwrap().pf_intent_log[pf_start..].to_vec();
        let tx_log = rig.sim.lock().unwrap().tx_log[tx_start..].to_vec();
        let target_count = pf_log
            .iter()
            .filter(|(_, k, stale, _)| !*stale && matches!(k, PosForceIntentKind::Target { .. }))
            .count();
        let hold_count = pf_log
            .iter()
            .filter(|(_, k, _, _)| matches!(k, PosForceIntentKind::Hold | PosForceIntentKind::Stop))
            .count();
        assert!(
            target_count >= 5 && hold_count >= 10,
            "{owner:?}: expected both Target ({target_count}) and Hold ({hold_count}) PosForce intents"
        );
        // On EVERY tick where the arm joints stepped (> 5e-4 rad), the PosForce J7 emitter MUST
        // have emitted Hold/Stop (NEVER an active Target!), proving J7 moves ONLY inside GripperMotion!
        for w in pf_log.windows(2) {
            let arm_d = (0..6)
                .map(|i| (w[1].3[i] - w[0].3[i]).abs())
                .fold(0.0f32, f32::max);
            if arm_d > 5e-4 {
                assert!(
                    !matches!(w[1].1, PosForceIntentKind::Target { .. }) || w[1].2,
                    "{owner:?}: PosForce emitted active Target while arm moved (arm_d={arm_d})!"
                );
            }
        }
        let _ = tx_log;

        // Also assert `lock_j7_hold: true` + `approach_leg` / `approach_then` is refused with
        // `INVALID_COMMAND` on BOTH owners!
        let q_cur = rig.measured_positions().await;
        let q_cur6 = [
            f64::from(q_cur[0]),
            f64::from(q_cur[1]),
            f64::from(q_cur[2]),
            f64::from(q_cur[3]),
            f64::from(q_cur[4]),
            f64::from(q_cur[5]),
        ];
        let mut q_goal6 = q_cur6;
        q_goal6[0] += 0.04;
        let mut q_goal7 = q_cur;
        q_goal7[0] = q_goal6[0] as f32;

        match owner {
            Owner::Legacy => {
                let inp = {
                    let inner = rig.sup.inner().read().await;
                    inner.plan_input_in(armd::motion::GoalFrame::Tcp).unwrap()
                };
                let swept = armd::grasp_select::feasibility::plan_swept_joint_path(
                    &inp,
                    q_goal6,
                    &[[0.0, 0.0, 0.0]],
                    -1.0,
                )
                .unwrap();
                let mut inner = rig.sup.inner().write().await;
                let stamp_l = inner.current_admission_stamp();
                let admitted = inner.check_admission(stamp_l).unwrap();
                let pid = SupervisorInner::begin_macro(admitted, "grasp", &inp, None).unwrap();
                let owner_ctx = OwnerCheckCtx::for_legacy(&inner);
                let req_stamp = ActionStamp {
                    arm: owner_ctx.arm,
                    boot: owner_ctx.boot,
                    admission: stamp_l.for_macro_child(),
                    config_revision: inner.workspace_revision,
                    calibration_revision: None,
                    poses_revision: None,
                    sequences_revision: None,
                    grasp_setup_revision: None,
                    gripper_calib_revision: None,
                    power_budget_revision: None,
                    parent_generation: pid,
                    deadline: Instant::now() + DEFAULT_ACTION_DEADLINE,
                };
                let dummy_seg = armd::trajectory::MultiJointQuinticTrajectory::new(
                    &q_cur,
                    &q_goal7,
                    &[0.0; 7],
                    &[0.0; 7],
                    0.5,
                )
                .unwrap();
                let mut target8 = [0.0f32; 8];
                target8[..7].copy_from_slice(&q_goal7);
                let payload = MacroChildPayload::LegacyCartesian {
                    spec: ChildSpec {
                        ticket: ChildTicket { parent: ActionId(pid), child_seq: 1 },
                        timeout_s: 10.0,
                        timeout_code: "GRASP_TIMEOUT",
                        contact: None,
                    },
                    start: q_cur.to_vec(),
                    input: inp,
                    resolved: ResolvedCartesianLegs {
                        path: swept,
                        approach_leg: Some(PreparedApproachLeg {
                            seg: dummy_seg,
                            target: q_goal7.to_vec(),
                            power_scale: None,
                            staged: StagedQuinticLeg {
                                target: target8,
                                duration_s: 0.5,
                                power_scale: None,
                            },
                        }),
                        outcome: None,
                        expected_duration_s: 1.0,
                    },
                    lock_j7_hold: true,
                };
                let res = install_macro_child(&mut inner, &req_stamp, &owner_ctx, false, payload);
                assert!(
                    matches!(res, Err(ArmError::Rejected { ref code, .. }) if code == "INVALID_COMMAND")
                        || matches!(res, Err(ArmError::Motion(armd::motion::MotionError::InvalidCommand(_)))),
                    "Legacy must refuse lock_j7_hold=true with approach_leg=Some: {res:?}"
                );
                inner.finish_macro(pid, "cancelled", None, None, None, None);
            }
            Owner::Executor => {
                let h = rig.handle.as_ref().unwrap();
                let boot = h.boot();
                let adm_e = rig.admission_stamp().await;
                let snap = ActionSnapshot::capture(
                    &rig.sup,
                    adm_e,
                    Instant::now() + DEFAULT_ACTION_DEADLINE,
                )
                .await
                .unwrap();
                let inp = snap.plan_input_in(armd::motion::GoalFrame::Tcp).unwrap();
                let swept = armd::grasp_select::feasibility::plan_swept_joint_path(
                    &inp,
                    q_goal6,
                    &[[0.0, 0.0, 0.0]],
                    -1.0,
                )
                .unwrap();
                let begin = armd::rt_core::requests::actions::BeginMacroSpec::new(
                    armd::rt_core::requests::actions::MacroKind::Grasp,
                    boot,
                );
                let (pid, _) = armd::macro_action::begin_macro_executor(
                    &rig.sup,
                    &snap.stamp,
                    begin,
                    Instant::now() + DEFAULT_ACTION_DEADLINE,
                )
                .await
                .unwrap();
                let snap2 = ActionSnapshot::capture(
                    &rig.sup,
                    adm_e,
                    Instant::now() + DEFAULT_ACTION_DEADLINE,
                )
                .await
                .unwrap();
                let mut slot = prepare_child_cartesian_slot_with_options(
                    &snap2.commanded_positions,
                    &swept,
                    None,
                    None,
                    true,
                    ChildSpec {
                        ticket: ChildTicket { parent: ActionId(pid), child_seq: 1 },
                        timeout_s: 10.0,
                        timeout_code: "GRASP_TIMEOUT",
                        contact: None,
                    },
                )
                .unwrap();
                // Inject a forbidden `approach_then` into the `lock_j7_hold: true` slot and submit!
                if let Primitive::Action(StagedAction::Cartesian {
                    ref mut approach_then, ..
                }) = slot.primitive
                {
                    let mut target = [0.0f32; 8];
                    target[..7].copy_from_slice(&q_goal7);
                    *approach_then = Some(StagedQuinticLeg {
                        target,
                        duration_s: 0.5,
                        power_scale: None,
                    });
                }
                let mut auth = snap2.owner.authority;
                auth.parent_generation = pid;
                let w = |dst: &mut PlanSlot| *dst = slot.clone();
                let req_id = h
                    .broker()
                    .lock()
                    .unwrap()
                    .submit(
                        ClientId(rig.conn_id),
                        777,
                        auth,
                        Instant::now() + DEFAULT_ACTION_DEADLINE,
                        Submit::Action(ActionSubmit::InstallChild {
                            write: &w,
                            prepared_boot: boot,
                            check_workspace_revision: false,
                            calibration_revision: None,
                            grasp_setup_revision: None,
                            gripper_calib_revision: None,
                            power_budget_revision: None,
                        }),
                    )
                    .unwrap();
                let dl = Instant::now() + Duration::from_secs(2);
                let mut refused_code: Option<&'static str> = None;
                while Instant::now() < dl {
                    h.pump_broker_replies();
                    if let Some(c) = rig.sink_replies.lock().unwrap().iter().find_map(|(_, r)| match r {
                        Reply::Refused { request, code } if *request == req_id => Some(*code),
                        _ => None,
                    }) {
                        refused_code = Some(c);
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
                assert_eq!(
                    refused_code,
                    Some("INVALID_COMMAND"),
                    "Executor must refuse lock_j7_hold=true with approach_then=Some"
                );
            }
        }
        rig.shutdown();
    }
}

// ============================================================================
// 9. Production Axum HTTP + WebSocket E2E on `OWNER_EXECUTOR`
//    (No Pre-Attached Reply Sink, Lease-Free Dry-Run, Live Adaptive WS, Lease-Loss During Plan)
// ============================================================================

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn phase_b_executor_ws_end_to_end_live_dry_run_and_lease_loss() {
    use futures_util::StreamExt;
    use tokio::net::TcpListener;
    use tokio_tungstenite::connect_async;

    let (cfg, setup) = fast_mounted_config();
    let gp = cfg.gripper_policy.clone().unwrap();
    let obj_j7 = gp.closed_rad + 0.35 * (gp.open_rad - gp.closed_rad);
    let q0 = solve_camera_down(&cfg, &setup, [0.42, 0.0, 0.26]);
    let poly = banana([0.42, 0.0], 0.12, 30.0, 0.034, 90.0);
    let q7: [f32; 7] = std::array::from_fn(|i| if i < 6 { q0[i] as f32 } else { gp.closed_rad });
    let (drv, _sim) = PhaseBDriver::new(&cfg, &q7, Some(obj_j7), true);
    let cam = CamModel::assumed(&cfg.arm_model, &setup, &cfg.camera_mount);
    let start_end = cfg.arm_model.fk(&q0);
    let arm_model = cfg.arm_model.clone();

    let sup = Arc::new(ArmSupervisor::new(cfg).with_driver(Box::new(drv)));
    sup.set_lease_enabled(true).await;
    {
        let mut inner = sup.inner().write().await;
        inner.set_grasp_setup(setup.clone());
        inner.commanded_positions = q7.to_vec();
        inner.measured_positions = q7.to_vec();
        inner.target_positions = q7.to_vec();
        let saved = inner.lease_enabled;
        inner.lease_enabled = false;
        let s = inner.current_admission_stamp();
        let a = inner.check_admission(s).unwrap();
        SupervisorInner::enable_torque(a).unwrap();
        inner.lease_enabled = saved;
        for _ in 0..3 {
            inner.tick(0.02);
        }
    }
    // Start RT Executor WITHOUT attaching any external reply sink: production WS routing MUST
    // attach the WS broker reply sink on demand!
    let handle = sup
        .start_executor(ExecutorConfig::for_tests(Duration::from_millis(5)))
        .await
        .unwrap();
    for _ in 0..100 {
        if sup.hub().latest().is_some_and(|s| !s.rt.owner.boot.is_zero()) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(sup.owner_mode(), OWNER_EXECUTOR);

    let vision = PhaseBVision::new(sup.clone(), cam.clone(), arm_model, poly.clone(), 0.035, setup.table_z_m);
    let grasp_deps = GraspDeps {
        source: vision.clone(),
        setup_path: None,
    };
    let detector = armd::api::proxy::DetectorUpstream::from_env();
    let diagnostics = armd::diagnostics::DiagnosticsState::global();
    let calibration = armd::calibration::CalibrationDeps::from_env(None, detector.clone());
    let router = armd::api::create_router_calibration_with_auth(
        sup.clone(),
        detector,
        grasp_deps,
        diagnostics,
        calibration,
        armd::OperatorAuth::disabled(),
    );
    // Ephemeral OS port (127.0.0.1:0 — never 18765/18001/18990).
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let srv = tokio::spawn(async move {
        let _ = axum::serve(listener, router).await;
    });

    let http = reqwest::Client::new();
    let req_body = adaptive_request(&cam, &start_end, setup.table_z_m, &poly, 0.035);

    // 9a) Schema advertises both `grasp_select` and `grasp_select_exec` on 6-DOF Executor!
    let schema: Value = http
        .get(format!("http://{addr}/api/v1/observation/schema"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(schema["capabilities"]["grasp_select"], json!(true), "{schema}");
    assert_eq!(schema["capabilities"]["grasp_select_exec"], json!(true), "{schema}");

    // 9b) Lease-free REST dry_run succeeds with 200 OK while NO client holds the lease!
    let mut dry_body = req_body.clone();
    dry_body["dry_run"] = json!(true);
    let rest_dry = http
        .post(format!("http://{addr}/api/v1/action/grasp"))
        .json(&dry_body)
        .send()
        .await
        .unwrap();
    assert_eq!(rest_dry.status(), reqwest::StatusCode::OK);
    let rest_dry_json: Value = rest_dry.json().await.unwrap();
    assert_eq!(rest_dry_json["reachable"], json!(true), "{rest_dry_json}");
    assert_eq!(rest_dry_json["strategy"], json!("adaptive"));
    assert_eq!(rest_dry_json["outcome"], json!("selected"));

    // 9c) REST live grasp (without dry_run: true) refuses with 409 LEASE_REQUIRED.
    let rest_live = http
        .post(format!("http://{addr}/api/v1/action/grasp"))
        .json(&req_body)
        .send()
        .await
        .unwrap();
    assert_eq!(rest_live.status(), reqwest::StatusCode::CONFLICT);
    let rest_live_json: Value = rest_live.json().await.unwrap();
    assert_eq!(rest_live_json["code"], json!("LEASE_REQUIRED"), "{rest_live_json}");

    // 9d) Open WS: lease-free WS dry_run succeeds BEFORE acquiring a lease!
    let (mut ws, _) = connect_async(format!("ws://{addr}/api/v1/ws")).await.unwrap();
    let epoch0 = sup.hub().latest().unwrap().obs.epoch_id;
    let ws_dry = common::ws_reply(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 1,
            "generation": 0,
            "epoch_id": epoch0,
            "path": "/api/v1/action/grasp",
            "body": dry_body
        }),
        &["command_ack", "command_reject"],
    )
    .await;
    assert_eq!(ws_dry["type"], json!("command_ack"), "WS lease-free dry_run failed: {ws_dry}");
    assert_eq!(ws_dry["reachable"], json!(true), "{ws_dry}");
    assert_eq!(ws_dry["strategy"], json!("adaptive"));

    // 9e) Acquire WS lease -> test lease loss during planning FIRST (while arm is still at `q0`):
    //     delay preflight 180 ms, release lease mid-flight -> MUST reject with `LEASE_LOST` and
    //     NOT start a macro!
    let granted = common::ws_reply(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "phase_b_ws_e2e"}),
        &["lease_granted", "lease_rejected"],
    )
    .await;
    assert_eq!(granted["type"], json!("lease_granted"), "{granted}");
    let gen0 = granted["generation"].as_u64().unwrap();
    let epoch = sup.hub().latest().unwrap().obs.epoch_id;

    vision.state.lock().unwrap().preflight_delay = Some(Duration::from_millis(180));
    common::send(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 2,
            "generation": gen0,
            "epoch_id": epoch,
            "path": "/api/v1/action/grasp",
            "body": req_body
        }),
    )
    .await;
    tokio::time::sleep(Duration::from_millis(40)).await;
    common::send(&mut ws, json!({"type": "release_lease", "generation": gen0})).await;
    let reject = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if matches!(v["type"].as_str(), Some("command_ack" | "command_reject"))
                && v["request_id"].as_u64() == Some(2)
            {
                return v;
            }
        }
    })
    .await
    .expect("timed out waiting for lease-loss rejection");
    assert_eq!(reject["type"], json!("command_reject"), "{reject}");
    assert_eq!(reject["code"], json!("LEASE_LOST"), "{reject}");
    assert!(
        !sup.hub().latest().unwrap().rt.owner.macro_state.alive,
        "lease loss during planning must NOT start a macro"
    );

    // 9f) Re-acquire WS lease -> live adaptive grasp over WS -> command_ack -> lossless terminal!
    vision.state.lock().unwrap().preflight_delay = None;
    let granted2 = common::ws_reply(
        &mut ws,
        json!({"type": "acquire_lease", "client_id": "phase_b_ws_e2e"}),
        &["lease_granted", "lease_rejected"],
    )
    .await;
    assert_eq!(granted2["type"], json!("lease_granted"), "{granted2}");
    let gen = granted2["generation"].as_u64().unwrap();
    let epoch2 = sup.hub().latest().unwrap().obs.epoch_id;

    let live_ack = common::ws_reply(
        &mut ws,
        json!({
            "type": "command",
            "request_id": 3,
            "generation": gen,
            "epoch_id": epoch2,
            "path": "/api/v1/action/grasp",
            "body": req_body
        }),
        &["command_ack", "command_reject"],
    )
    .await;
    assert_eq!(
        live_ack["type"],
        json!("command_ack"),
        "Executor WS live adaptive grasp must be accepted (not 503 EXECUTOR_ROUTING_PENDING): {live_ack}"
    );
    let action_id = live_ack["action_id"].as_u64().expect("action_id on ack");
    assert!(live_ack["expected_duration_s"].as_f64().unwrap_or(0.0) > 0.0);

    // Wait for lossless terminal observation on the WebSocket stream with holding == true!
    let terminal = tokio::time::timeout(Duration::from_secs(20), async {
        loop {
            let text = ws.next().await.unwrap().unwrap().into_text().unwrap();
            let v: Value = serde_json::from_str(&text).unwrap();
            if v.get("type").is_none() {
                if let Some(a) = v.get("action").filter(|a| !a.is_null()) {
                    if a["id"].as_u64() == Some(action_id) && a["state"] != "running" {
                        return a.clone();
                    }
                }
            }
        }
    })
    .await
    .expect("timed out waiting for lossless WS adaptive grasp terminal");
    assert_eq!(terminal["state"], json!("done"), "{terminal}");
    assert_eq!(terminal["holding"], json!(true), "{terminal}");
    assert_eq!(
        sup.hub().latest().unwrap().rt.owner.held.map(|h| h.label.as_str().to_string()),
        Some("banana".to_string())
    );

    srv.abort();
    handle.stop_and_join();
}

// ============================================================================
// 10. Verify Companion SDK Doc Request Example Through Actual `armd` Parsers
// ============================================================================

#[test]
fn phase_b_sdk_doc_request_example_parses_cleanly() {
    let doc = include_str!("../docs/grasp_phase_b_sdk.md");
    let start = doc.find("```json\n").expect("docs/grasp_phase_b_sdk.md must contain a ```json example");
    let rest = &doc[start + "```json\n".len()..];
    let end = rest.find("\n```").expect("closing ``` fence");
    let raw_json = &rest[..end];
    let body: Value = serde_json::from_str(raw_json).expect("valid JSON request example");

    let req = grasp::parse_grasp(&body, &GraspSetup::default()).expect("parse_grasp must accept doc example");
    assert_eq!(req.label, "banana");
    assert_eq!(req.frame_id, Some(7));
    assert_eq!(grasp_select::parse_strategy(&body).unwrap(), Strategy::Adaptive);
    let ev = grasp_select::snapshot::parse_evidence(&body).expect("parse_evidence must accept doc example");
    assert_eq!(ev.contour_norm.as_ref().map(Vec::len), Some(4));
    assert_eq!(ev.height_m, Some([0.032, 0.038]));
    assert_eq!(ev.payload_kg, Some(0.15));
    let prefs = grasp_select::snapshot::parse_preferences(&body).expect("parse_preferences must accept doc example");
    assert_eq!(prefs.approach_side, grasp_select::snapshot::Side::Any);
    assert_eq!(prefs.preferred_max_tilt_deg, Some(45.0));
    assert_eq!(prefs.hard_max_tilt_deg, Some(70.0));
    assert_eq!(grasp_select::snapshot::parse_budget(&body).unwrap(), 500);
}
