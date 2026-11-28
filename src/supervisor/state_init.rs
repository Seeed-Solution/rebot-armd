//! `SupervisorInner` construction, detached shell, disconnect fence, and in-place lease helpers.

#[allow(unused_imports)]
use super::*;

impl SupervisorInner {
    pub fn new(config: ArmConfig) -> Self {
        Self::new_inner(config, true, true)
    }

    pub(super) fn new_inner(mut config: ArmConfig, allow_shadow: bool, strict_planning: bool) -> Self {
        if strict_planning {
            config.validate().expect("Invalid ArmConfig: invariant violation");
        } else {
            config
                .validate_runtime_narrowed()
                .expect("Invalid ArmConfig: invariant violation");
        }
        let dof = config.dof;
        let collision_policy = config.collision.unwrap_or_default();
        // power planning: build the dynamics chain now (the built-in model parses the embedded
        // URDF once), not on the first planned motion
        if config.power_budget.as_ref().is_some_and(|b| b.enabled && b.planning.enabled) { let _ = config.arm_model.dynamics_chain(); }
        let positions = config.home_positions.clone();
        let target_positions = positions.clone();
        let lease_timeout = Duration::from_millis(config.lease_timeout_ms);
        let motion_frame_timeout = Duration::from_millis(200);
        let now = Instant::now();

        let candidate_urdf_paths = [
            "assets/reBot-DevArm_gripper.urdf",
            "/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf",
            "/usr/share/rebot-teleop/reBot-DevArm_gripper.urdf",
        ];
        let mut gravity_compensated = false;
        if let Some(chain) = config.arm_model.chain() {
            // Profile URDF: read and (with urdf_sha256) verified by ArmConfig::load_arm_model.
            tracing::info!("Arm model from URDF '{}' ({} -> {}, {} joints, {:.3} kg moving)",
                chain.robot, chain.base_link, chain.tip_link, chain.dof(), chain.moving_mass());
            gravity_compensated = chain.moving_mass() > 0.0;
        }
        for path in candidate_urdf_paths.iter().filter(|_| config.arm_model.chain().is_none()) {
            if std::path::Path::new(path).exists() {
                match crate::gravity::verify_urdf_sha256(path) {
                    Ok(()) => {
                        tracing::info!("URDF model verified ({}); output still requires an explicit model profile and compatible driver.", path);
                        gravity_compensated = true;
                        break;
                    }
                    Err(e) => {
                        tracing::warn!("URDF verification failed for {} ({:?})", path, e);
                    }
                }
            }
        }
        if !gravity_compensated {
            tracing::warn!("Running in degraded mode without gravity compensation.");
        }

        let base_gripper_policy = config.gripper_policy.clone();
        let profile_camera_mount = config.camera_mount.clone();
        let profile_floor_z_m = config.workspace.as_ref().map(|w| w.floor_z_m);
        let profile_grasp = config.grasp.clone();
        let mock_gripper_stall_offset = std::env::var("ARMD_MOCK_GRIPPER_STALL_OFFSET").ok().and_then(|s| s.parse::<f32>().ok());
        let unload_settle_cap = ((UNLOAD_SETTLE_HOLD.as_secs_f64() * f64::from(config.control_frequency_hz.max(1)))
            .ceil() as usize
            + 16)
            .max(64);

        let estop_state = Arc::new(crate::rt_core::urgent::EstopState::new());
        let e_stop_latched = crate::rt_core::urgent::EstopLatchView::new(estop_state.clone());
        let requested_estop_count = crate::rt_core::urgent::EstopCounterView::new(estop_state.clone());
        #[cfg(not(target_arch = "wasm32"))]
        let torque_shadow = if allow_shadow {
            crate::torque::shadow::ShadowRuntime::new_with_hub(
                &config,
                config.torque_shadow_enabled,
                crate::torque::VendorKind::Damiao,
                false,
                None,
            )
            .0
        } else {
            crate::torque::shadow::ShadowRuntime::disabled(&config)
        };

        let mut scratch = crate::rt_core::scratch::TickScratch::with_dof(dof);
        crate::rt_core::scratch::copy_limits(&mut scratch.step_limits, &config.limits);
        crate::rt_core::scratch::copy_limits(&mut scratch.fallback_limits, &config.limits);

        let mut inner = Self {
            config,
            safety: SafetyManager::new(),
            held: None,
            releasing: false,
            measured_positions: positions.clone(),
            measured_velocities: vec![0.0; dof],
            commanded_positions: positions,
            commanded_velocities: vec![0.0; dof],
            last_commanded_velocities: vec![0.0; dof],
            commanded_accelerations: vec![0.0; dof],
            following_error_since: None,
            disable_pending: false,
            torque_transition: None,
            legacy_torque_seq: 0,
            legacy_torque_slots: std::array::from_fn(|_| Arc::new(driver_transitions::LegacyTorqueSlot::new())),
            driver_has_load_feedback: false,
            nonblocking_driver_release: false,
            legacy_torque_settled_this_tick: false,
            detached_torque_pending_atomic: Arc::new(AtomicBool::new(false)),
            detached_cancel_restore_intent: None,
            last_tick_torque_ws_direction: None,
            enable_lifecycle: driver_transitions::EnableLifecycle::Disarmed,
            torque_intent: driver_transitions::TorqueIntent::Off,
            process_exit_authorized: false,
            control_release_seq: 0,
            control_release_seq_atomic: Arc::new(AtomicU64::new(0)),
            #[cfg(not(target_arch = "wasm32"))]
            torque_shadow,
            #[cfg(not(target_arch = "wasm32"))]
            shadow_enable_op_id: 0,
            measured_hold: driver_transitions::MeasuredHoldTracker::new(dof),
            owner_driver_snapshot: driver_transitions::OwnerDriverSnapshot::new(dof),
            background_disable_retry: None,
            last_disable_attempt: None,
            active_disable_kind: None,
            active_disable_client_park: None,
            last_stopped_torque_off_refusal: None,
            last_reported_torque_loss_warn: None,
            reported_torque_loss_count: 0,
            reported_torque_loss_last_release: 0,
            reported_torque_loss_last_sample_seq: None,
            reported_torque_loss_from_motion: false,
            disable_lock_contended_skips: 0,
            disable_lock_contended_escalated: false,
            output_suspended: false,
            feedback_numeric_valid: true,
            sim_time_s: 0.0,
            last_still_since_s: None,
            still_anchor_positions: Vec::with_capacity(dof),
            still_window_ever_moved: false,
            table_still_first_rx_instants: Vec::with_capacity(dof),
            table_still_last_rx_instants: Vec::with_capacity(dof),
            table_still_anchor_positions: Vec::with_capacity(dof),
            torques: vec![0.0; dof],
            loads: vec![0.0; dof],
            temperatures: vec![25.0; dof],
            target_positions,
            servo_target: None,
            servo_target_spare: Vec::with_capacity(crate::rt_core::mailbox::MAX_DOF.max(dof)),
            servo_stream_active: false,
            servo_settled_ticks: vec![0; dof],
            servo_lead_stall: vec![None; dof],
            motor_errors: Vec::new(),
            motion_limit: None,
            workspace_violation: None,
            workspace_violation_spare: Some(crate::kinematics::WorkspaceViolationDetail::preallocated_rt()),
            motion_limit_seen_s: 0.0,
            servo_blocked_hold: false,
            servo_ws_min_depths: None,
            gripper_hold_vel_ceiling: None,
            is_torqued: false,
            pending_rearm: false,
            active_trajectory: None,
            last_action: None,
            previous_action: None,
            next_action_id: 0,
            action_lease_generation: None,
            macro_run: None,
            #[cfg(feature = "policy-runtime")]
            policy_run: None,
            #[cfg(feature = "policy-runtime")]
            policy_loading: false,
            driver_feedback_confirmed_cached: false,
            #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
            policy_load_reservation: None,
            #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
            policy_cancel_slots: Arc::new(crate::rt_core::requests::policy::PolicyCancelSlots::new()),
            #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
            policy_terminal_tx: None,
            #[cfg(feature = "policy-runtime")]
            last_policy_obs: None,
            #[cfg(feature = "policy-runtime")]
            rt_policy_auth: crate::policy_runtime::PolicyAuthorization::default(),
            #[cfg(feature = "policy-runtime")]
            policy_run_spare: None,
            #[cfg(feature = "policy-runtime")]
            policy_live_vec_spare: Vec::with_capacity(crate::rt_core::mailbox::MAX_DOF),
            #[cfg(feature = "policy-runtime")]
            policy_shadow_vec_spare: Vec::with_capacity(crate::rt_core::mailbox::MAX_DOF),
            #[cfg(feature = "policy-runtime")]
            last_policy_obs_spare: Some(crate::model::PolicyObservation::preallocated_rt()),
            #[cfg(feature = "policy-runtime")]
            last_policy_obs_vec_spare: Vec::with_capacity(crate::rt_core::mailbox::MAX_DOF),
            #[cfg(feature = "policy-runtime")]
            pending_policy_close_max: 0,
            #[cfg(feature = "policy-runtime")]
            atomic_diag_sealed_policy_action_id: Arc::new(AtomicU64::new(0)),
            #[cfg(feature = "policy-runtime")]
            atomic_closed_policy_action_id: Arc::new(AtomicU64::new(0)),
            #[cfg(feature = "policy-runtime")]
            policy_ledger_path: crate::policy_runtime::qualification::ledger_path_from_env(),
            quintic_settle: None,
            before_install_hook: None,
            last_heartbeat: now,
            last_motion_frame: now,
            motion_frame_timeout,
            lease_timeout,
            lease_enabled: true,
            active_lease: None,
            active_lease_conn_id: 0,
            fenced_lease_generation: 0,
            pending_servo_closes: [0; 32],
            pending_servo_close_len: 0,
            lease_generation_counter: 0,
            boot_id: crate::rt_core::broker::BootId::process(),
            schema_revision: 0,
            cell: None,
            gripper_stalled: false,
            gripper_high_torque_ticks: 0,
            gripper_low_torque_ticks: 0,
            gripper_stall_window: crate::motion::GripperProgressWindow::default(),
            gripper_stall_sign: 0.0,
            gripper_prev_hw_overload: false,
            gripper_force_capped: false,
            gripper_force_ticks: 0,
            gripper_force_trips: 0,
            gripper_motion_hold: None,
            gripper_stream_target: None,
            gripper_stream_contact: None,
            gripper_stream_contact_detector: crate::motion::GripperContactDetector::default(),
            gripper_stream_lease: None,
            gripper_ref_prev: false,
            gripper_ref_done: false,
            gripper_ref_holding: false,
            gripper_cap_ramp: None,
            gripper_over_cap_prev: false,
            park_gripper_contact: false,
            park_release: None,
            pending_normal_unload: false,
            unload_settle: None,
            watchdog_park: None,
            watchdog_park_blocked_reason: None,
            park_rejection: None,
            workspace_revision: 0,
            calibration: None,
            #[cfg(feature = "policy-runtime")]
            calibration_digest: crate::policy_runtime::EMPTY_CALIBRATION_DIGEST.to_string(),
            mit_gate_error: None,
            #[cfg(not(target_arch = "wasm32"))]
            mit_gate_proof: None,
            mit: Default::default(),
            profile_floor_z_m,
            floor_source: "profile",
            hover_jobs: Vec::new(),
            calibration_busy: false,
            motor_zero_busy: false,
            calibration_txn_id: None,
            #[cfg(not(target_arch = "wasm32"))]
            commissioning: crate::mit::CommissioningMachine::default(),
            #[cfg(not(target_arch = "wasm32"))]
            sessions: crate::rt_core::session::SessionDriver::new(),
            #[cfg(not(target_arch = "wasm32"))]
            rt_recorder: crate::rt_core::recorder::RtRecorder::new(),
            #[cfg(not(target_arch = "wasm32"))]
            mode_switch: ModeSwitchRunner::new(),
            #[cfg(not(target_arch = "wasm32"))]
            mode_switch_ran_release_seq: 0,
            authority: crate::rt_core::authority::OwnerAuthorityState::new(),
            #[cfg(not(target_arch = "wasm32"))]
            last_legacy_release_budget: None,
            pending_conservative_tx_cap: false,
            conservative_step_rx_delta: 0,
            conservative_step_tx_delta: 0,
            calibration_quarantined: false,
            cached_driver_pos_limits: [None; crate::rt_core::mailbox::MAX_DOF],
            calibration_load_error: None,
            joint_zero_offsets: vec![0.0; dof],
            zero_convention: Default::default(),
            pending_legacy_j6_log: None,
            zero_check_pending: true,
            zero_check_prev_torqued: false,
            profile_camera_mount,
            profile_grasp,
            unload_planned_duration: None,
            recorder: EpisodeRecorder::new(),
            calibration_revision: 0,
            sequences: Default::default(),
            recording: None,
            max_episode_s: crate::recording::store::MAX_EPISODE_S,
            episodes: crate::recording::EpisodeStore::new(Some(PathBuf::from(crate::recording::store::FALLBACK_EPISODES_DIR))),
            start_time: now,
            driver: None,
            driver_lock_attempts: Arc::new(AtomicU64::new(0)),
            gravity_compensated,
            gravity_torques: Vec::new(),
            gravity_scale: 0.5,
            gravity_torques_ramped: vec![0.0; dof],
            last_can_feedback_instant: None,
            last_valid_motor_rx: [None; crate::model::MAX_MOTION_DOF],
            pending_driver_bus_loss_notify: false,
            core: crate::rt_core::ArmCore::new(dof),
            joint_feedback_queue_bounds: Vec::with_capacity(dof),
            scratch,
            ws_eval_bufs: Default::default(),
            bus_loss_instant: None,
            estop_state,
            e_stop_latched,
            requested_estop_count,
            applied_estop_count: 0,
            before_estop_unlatch_hook: None,
            estop_applied: Arc::new(AtomicU64::new(0)),
            stop_gen: Arc::new(AtomicU64::new(0)),
            lease_gen_mirror: Arc::new(AtomicU64::new(0)),
            shared_closed_bitset: Arc::new(crate::rt_core::requests::actions::ClosedBitset::new()),
            atomic_fenced_lease_gen: Arc::new(AtomicU64::new(0)),
            pending_disconnect_hold: Arc::new(AtomicU64::new(0)),
            last_preemption: None,
            last_evicted: None,
            handover_settling_since: None,
            gravity_comp: None,
            retired_gravity_comp: std::collections::VecDeque::with_capacity(8),
            retired_gravity_comp_policies: std::collections::VecDeque::with_capacity(8),
            #[cfg(not(target_arch = "wasm32"))]
            pending_gravity_enter: None,
            gravity_comp_last_exit: None,
            keepout: crate::keepout::Keepout::default(),
            poses: crate::poses::PoseStore::default(),
            residual_torques: None,
            collision: crate::collision::CollisionGuard::from_policy(&collision_policy),
            torque_guard: crate::torque_guard::TorqueGuard::default(),
            tracking: crate::tracking::TrackingRuntime::with_capacity(dof),
            power: crate::power::PowerMonitor::default(),
            static_load_cache: crate::power::StaticLoadBaselineCache::default(),
            static_load_tracking_cache: crate::power::StaticLoadBaselineCache::default(),
            follower: None,
            base_gripper_policy,
            gripper_calib_offset: None,
            gripper_calib_jaw_opening: None,
            gripper_calib_path: None,
            gripper_baseline: None,
            gripper_baseline_path: None,
            gripper_baseline_dirty: false,
            gripper_baseline_pending: Vec::with_capacity(GRIPPER_BASELINE_PENDING_MAX),
            gripper_baseline_generation: 0,
            gripper_calibration: None,
            gripper_last_contact: None,
            gripper_zero_checked: false,
            gripper_calib_invalid: false,
            gripper_calib_stale: None,
            last_zero_check_offset: None,
            mock_gripper_stall_offset,
            mock_gripper_stall_torque: false,
            mock_gripper_object: None,
            driver_config_error: None,
            retired_driver_config_error: None,
            config_reconcile_fault: None,
            thermal_cooldown_duration: Duration::ZERO,
            thermal_cooldown: ThermalCooldown::default(),
            zero_check_params: None,
            gripper_calib_revision: 1,
            grasp_setup_revision: 1,
            grasp_setup_needs_camera_sync: false,
            defer_zero_check_persist: false,
            prepared_quintics_used: 0,
            tick_phase_mark: None,
            tick_phases_us: [0; 5],
            zero_check_persisting: None,
            zero_check_persisting_grant: None,
            pending_zero_check_persist: None,
            zero_check_stream_tx: None,
            pending_zero_check_stream_persist: None,
            zero_check_stream_pushed: false,
            retired_zero_check_ctx_ids: [0; 16],
            retired_zero_check_ctx_id_len: 0,
            #[cfg(feature = "policy-runtime")]
            pending_policy_qual_persist: None,
            retired_zero_check_ctx: std::collections::VecDeque::with_capacity(4),
            operator_auth_enforced: false,
            tool_commissioning_path: None,
            tool_commissioning_confirmed_at: None,
            tool_commissioning_revision: 1,
            pending_rx_poll: PendingRxPoll::default(),
            active_tx_cutoff: None,
            stop_requests: None,
            detached: false,
            persist_capacity_available: true,
            rt_executor_owned: false,
            owner_reason: crate::model::OwnerReason::Default,
            shutdown_committed: false,
            shutdown_refused_unknown: false,
            internal_fault_lane: Arc::new(crate::panic_coord::InternalFaultLane::new()),
            internal_fault_park_active: false,
            pending_estop_acks: std::collections::VecDeque::with_capacity(
                crate::rt_core::executor::ESTOP_ACK_CAPACITY,
            ),
            estop_ack_overflow_gen: 0,
            pending_retired_stops: std::collections::VecDeque::with_capacity(
                crate::rt_core::urgent::URGENT_STOP_CAPACITY,
            ),
            closed_connections: ClosedConnections::new(),
            live_connections: LiveConnections::new(),
            lease_client_spare: String::with_capacity(64),
            lease_label_spare: String::with_capacity(128),
            evicted_conn_spare: String::with_capacity(64),
            consumed_stamps: ConsumedStamps::new(),
            consumed_broker_stamps: ConsumedStamps::new(),
            consumed_internal_stamps: ConsumedStamps::new(),
            terminals: None,
            defer_terminal_capture: false,
            terminal_capture_tx: None,
            terminal_capture_channel: None,
            staged_terminals: terminal_capture::StagedTerminalBuf::default(),
            terminal_edge_tracker: terminal_capture::TerminalEdgeTracker::default(),
            last_phase_terminal_us: 0,
            last_publish_only_us: 0,
            first_terminal_csw_mark: None,
            last_mid_csw_start: None,
            capture_driver_cache: terminal_capture::CaptureDriverCache::default(),
            template_obs_for_drain: None,
            terminal_capture_test_hook: None,
            safety_log_ring_dropped: 0,
            before_publish_hook: None,
            tx_joint_no_progress_ticks: vec![0; dof],
            tx_joint_no_progress_since: vec![None; dof],
            hold_tx_no_progress_ticks: vec![0; dof.saturating_mul(2)],
            hold_tx_no_progress_since: vec![None; dof.saturating_mul(2)],
            hold_tx_stalled_latched: false,
            last_hold_tx_stall_log: None,
            last_hold_tx_error_log: None,
            last_hold_tx_error_delivered: None,
            hold_tx_error_suppressed: 0,
            pending_hold_tx_error_retry: None,
            j7_stale_since: None,
            driver_clock: None,
            pending_deferred_done: None,
            gravity_comp_handback_ready: false,
            action_msg_spares: [
                Some(String::with_capacity(256)),
                Some(String::with_capacity(256)),
                Some(String::with_capacity(256)),
                Some(String::with_capacity(256)),
            ],
            retired_trajectories: std::collections::VecDeque::with_capacity(8),
            retired_macro_runs: std::collections::VecDeque::with_capacity(8),
            #[cfg(feature = "policy-runtime")]
            retired_policy_runs: std::collections::VecDeque::with_capacity(8),
            unload_settle_spare: Some(UnloadSettle::with_capacity(unload_settle_cap)),
            park_release_spare: None,
            motion_config_revision: 1,
            motion_generation: 0,
            last_published_motion_fp: 0,
            driver_caps: DriverCapabilities::default(),
            last_poll_enable_state: EnableState::Unknown,
            last_poll_partial_enable: false,
            last_poll_oldest_age: None,
            last_poll_feedback_valid: false,
            park_continuation: None,
            park_continuation_notify: Arc::new(tokio::sync::Notify::new()),
            motion_plan_deadline: crate::rt_core::requests::motion::DEFAULT_MOTION_PLAN_DEADLINE,
            last_clock_now: None,
            watchdog_park_pending: None,
            watchdog_park_seq: 0,
            defer_watchdog_park_plan: false,
            defer_legacy_commissioning: false,
            j7_intent: None,
            j7_admitted_stamp: None,
            rt_j7_pf_gate: None,
            post_stop_check_hook: None,
            gripper_motion_spare: None,
            move_samples_spares: Vec::new(),
            driver_has_torque_feedback: true,
            driver_gripper_close_speed_limit: None,
            driver_gripper_ceiling_torque_estimate: None,
            held_label_spare: String::new(),
            power_budget_revision: 1,
            rt_closed_connections: None,
        };
        #[cfg(feature = "policy-runtime")]
        {
            inner.policy_run_spare = Some(crate::policy_runtime::PolicyRun::preallocated_rt_spare(
                inner.rt_policy_auth.clone(),
            ));
        }
        inner.prepare_workspace_eval_bufs();
        inner.prepare_park_release_spare();
        inner.refresh_schema_cache();
        inner
    }

    /// Minimal empty shell left inside `ArmSupervisor::inner` once ownership transfers to
    /// the synchronous RT executor; all legacy lock paths reject it when `detached == true`.
    pub(crate) fn detached_shell(
        config: &ArmConfig,
        estop_state: Arc<crate::rt_core::urgent::EstopState>,
        stop_gen: Arc<AtomicU64>,
        lease_gen_mirror: Arc<AtomicU64>,
        shared_closed_bitset: Arc<crate::rt_core::requests::actions::ClosedBitset>,
        atomic_fenced_lease_gen: Arc<AtomicU64>,
        pending_disconnect_hold: Arc<AtomicU64>,
    ) -> Self {
        let mut shell = Self::new_inner(config.clone(), false, false);
        shell.e_stop_latched = crate::rt_core::urgent::EstopLatchView::new(estop_state.clone());
        shell.requested_estop_count = crate::rt_core::urgent::EstopCounterView::new(estop_state.clone());
        shell.estop_state = estop_state;
        shell.stop_gen = stop_gen;
        shell.lease_gen_mirror = lease_gen_mirror;
        shell.shared_closed_bitset = shared_closed_bitset;
        shell.atomic_fenced_lease_gen = atomic_fenced_lease_gen;
        shell.pending_disconnect_hold = pending_disconnect_hold;
        shell.detached = true;
        shell
    }

    /// Current owner-serialized stop generation.
    /// Single authoritative computation of the effective stop generation combining owner-local
    /// `stop_gen` with the lock-free `StopRequests` handle when attached.
    #[inline]
    pub(crate) fn compute_current_stop_gen(
        stop_gen: &AtomicU64,
        stop_requests: Option<&crate::rt_core::urgent::StopRequests>,
    ) -> u64 {
        let local = stop_gen.load(Ordering::SeqCst);
        match stop_requests {
            Some(stops) => local.max(stops.current()),
            None => local,
        }
    }

    pub fn current_stop_gen(&self) -> u64 {
        Self::compute_current_stop_gen(&self.stop_gen, self.stop_requests.as_ref())
    }

    /// Effective highest fenced lease generation, combining owner-local and lock-free immediate
    /// socket-close fences (`SeqCst`).
    #[inline]
    pub(crate) fn effective_fenced_lease_generation(&self) -> u64 {
        self.fenced_lease_generation
            .max(self.atomic_fenced_lease_gen.load(Ordering::SeqCst))
    }

    /// Lock-free check evaluated immediately before emitting actuator targets: true when an
    /// un-drained disconnect hold is active, an immediate socket-close generation fence has not
    /// yet been synced, or the active lease's connection is already closed and unfenced.
    #[inline]
    pub(crate) fn disconnect_fence_blocks_output(&self) -> bool {
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            return true;
        }
        let atomic_fenced = self.atomic_fenced_lease_gen.load(Ordering::SeqCst);
        if atomic_fenced > self.fenced_lease_generation {
            return true;
        }
        if self.active_lease_conn_id != 0
            && self.is_connection_closed(self.active_lease_conn_id)
            && self
                .active_lease
                .as_ref()
                .is_some_and(|a| a.generation > self.fenced_lease_generation)
        {
            return true;
        }
        false
    }

    /// Lock-free, zero-allocation owner synchronization of immediate socket-close fences
    /// (`atomic_fenced_lease_gen`, `shared_closed_bitset`, `pending_disconnect_hold`) and lease
    /// expiry at `now`. Runs at the top of every control tick, RT Step 1, and every RT checkpoint
    /// (`BeforeCommit`, `BeforeTarget`, `BeforeOutput`) BEFORE any command commit, target intake,
    /// or bus output.
    pub(crate) fn sync_nonblocking_disconnect_fence(&mut self, now: Instant) -> bool {
        let mut fenced_any = false;
        let atomic_fenced = self.atomic_fenced_lease_gen.load(Ordering::SeqCst);
        if atomic_fenced > self.fenced_lease_generation {
            let stop_client_motion =
                self.watchdog_park.is_none() && !self.pending_normal_unload;
            self.fence_displaced_lease_generation(atomic_fenced, stop_client_motion);
            fenced_any = true;
        }
        let closed_active = self.active_lease.as_ref().and_then(|a| {
            let conn_closed = self.active_lease_conn_id != 0
                && self.is_connection_closed(self.active_lease_conn_id);
            let gen_fenced = a.generation <= self.fenced_lease_generation;
            (conn_closed || gen_fenced).then_some((a.generation, conn_closed))
        });
        if let Some((gen, conn_closed)) = closed_active {
            let stop_client_motion =
                self.watchdog_park.is_none() && !self.pending_normal_unload;
            self.fence_displaced_lease_generation(gen, stop_client_motion);
            if conn_closed {
                let cid = self.active_lease_conn_id;
                if cid != 0 {
                    self.mark_connection_closed(cid);
                }
                let _ = self.clear_active_lease_in_place();
            }
            fenced_any = true;
        }
        // Fence an expired lease generation BEFORE target consumption, gripper, motion, or
        // watchdog output runs (preserves `active_lease = Some(expired)` for expiry telemetry,
        // retires expired client motion, and preserves any autonomous watchdog park explicitly).
        if let Some(expired_gen) = self
            .active_lease
            .as_ref()
            .filter(|a| {
                a.generation > self.fenced_lease_generation
                    && now.saturating_duration_since(a.acquired_at) >= self.lease_timeout
            })
            .map(|a| a.generation)
        {
            let enable_owned_by_expired = matches!(
                self.torque_transition,
                Some(driver_transitions::TorqueTransition::Enable(reducer))
                    if reducer
                        .authority
                        .is_some_and(|a| a.lease_generation == expired_gen)
            );
            if enable_owned_by_expired {
                // Preserve Step 2 `service_torque_transition` timing for an in-flight enable
                // transition owned by the expiring lease (do not take `driver.lock()` or start
                // rollback disable prematurely in Step 1). Still revoke target stream authority
                // and queue off-thread `close_servo(expired_gen)` before Step 4 target intake.
                self.core.revoke_stream();
                self.record_pending_servo_close(expired_gen);
                if self.gripper_stream_lease == Some(expired_gen) {
                    self.end_gripper_stream();
                }
                if self.watchdog_park.is_none() && !self.pending_normal_unload {
                    self.stop_motion();
                }
            } else {
                let stop_client_motion =
                    self.watchdog_park.is_none() && !self.pending_normal_unload;
                self.fence_displaced_lease_generation(expired_gen, stop_client_motion);
            }
            fenced_any = true;
        }
        if self.pending_disconnect_hold.load(Ordering::SeqCst) > 0 {
            if self.watchdog_park.is_none() && !self.pending_normal_unload {
                self.stop_motion();
                self.core.revoke_stream();
            }
            fenced_any = true;
        }
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        {
            self.prune_stale_policy_load(now);
            if self.policy_run.as_ref().is_some_and(|r| {
                r.rt_owned && self.policy_cancel_slots.is_cancelled(r.cancel_slot, r.load_token)
            }) {
                self.terminate_policy_run_static(
                    "STOPPED",
                    "cancelled",
                    Some("STOPPED"),
                    Some("Policy launch or runner cancelled"),
                );
                fenced_any = true;
            }
        }
        fenced_any
    }

    /// Prunes any expired, cancelled, disconnected, lease-fenced, epoch/stop-diverged, or
    /// faulted/E-stopped executor policy load reservation without allocating (zero RT allocs).
    #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
    pub(crate) fn prune_stale_policy_load(&mut self, now: Instant) {
        let Some(res) = self.policy_load_reservation else {
            return;
        };
        let expired = now > res.expires_at;
        let cancelled = self.policy_cancel_slots.is_cancelled(res.cancel_slot, res.token);
        let conn_closed = res.conn_id != 0 && self.is_connection_closed(res.conn_id);
        let lease_invalid =
            self.lease_valid_at(now).map_or(0, |l| l.generation) != res.lease_generation;
        let epoch_changed = self.safety.epoch_id() != res.epoch_id;
        let stop_changed = self.current_stop_gen() != res.stop_generation;
        let estop_or_fault = self.estop_pending()
            || matches!(self.safety.state(), SafetyState::EStop | SafetyState::Fault(_));
        if expired
            || cancelled
            || conn_closed
            || lease_invalid
            || epoch_changed
            || stop_changed
            || estop_or_fault
        {
            self.policy_load_reservation = None;
            self.policy_loading = false;
            self.policy_cancel_slots.set_active_load_token(0);
            self.reconcile_target_stream_authority(now);
        }
    }

    /// Purges any connection ID already closed in the lock-free `shared_closed_bitset` from
    /// `live_connections` (and records it in `closed_connections`) with zero heap allocations.
    /// Called before RT ownership handoff so RT never inherits a stale connection or lease.
    pub(crate) fn purge_closed_live_connections(&mut self) {
        let mut closed_ids = [0u64; MAX_LIVE_CONNECTIONS];
        let mut n = 0usize;
        for &id in &self.live_connections.ids[..self.live_connections.len] {
            if id != 0 && self.is_connection_closed(id) {
                closed_ids[n] = id;
                n += 1;
            }
        }
        for &id in &closed_ids[..n] {
            self.mark_connection_closed(id);
        }
        self.sync_nonblocking_disconnect_fence(Instant::now());
    }

    /// Current valid (unexpired at `now`, unfenced, open-connection) active lease generation, or `0`.
    pub(crate) fn active_lease_gen_at(&self, now: Instant) -> u64 {
        let fenced = self.effective_fenced_lease_generation();
        if self.active_lease_conn_id != 0 && self.is_connection_closed(self.active_lease_conn_id) {
            return 0;
        }
        self.active_lease
            .as_ref()
            .filter(|a| {
                a.generation > fenced
                    && now.saturating_duration_since(a.acquired_at) < self.lease_timeout
            })
            .map_or(0, |a| a.generation)
    }

    /// Current valid (unexpired, unfenced, open-connection) active lease generation, or `0` when no valid lease is held.
    pub fn current_active_lease_gen(&self) -> u64 {
        self.active_lease_gen_at(Instant::now())
    }

    /// Capture an [`AdmissionStamp`] from the current owner state (for tests and in-process
    /// request entry before mutating/stopping; never called inside motion-start functions).
    pub fn current_admission_stamp(&self) -> AdmissionStamp {
        AdmissionStamp {
            stop_gen: self.current_stop_gen(),
            estop_epoch: self.safety.epoch_id(),
            lease_gen: self.current_active_lease_gen(),
            connection_id: 0,
            request_id: next_admission_request_id(),
        }
    }

    /// Registers a live WS connection ID (`>= 1`) under the owner lock (fail-closed at
    /// [`MAX_LIVE_CONNECTIONS`]). Rejects IDs already fenced closed in the lock-free bitset.
    pub fn register_live_connection(&mut self, connection_id: u64) -> Result<(), &'static str> {
        if connection_id != 0 && self.is_connection_closed(connection_id) {
            self.closed_connections
                .mark_closed(connection_id, &self.live_connections);
            return Err("DISCONNECTED");
        }
        self.live_connections
            .register(connection_id, &self.closed_connections)
    }

    /// Marks WS connection `connection_id` (`>= 1`) closed under the owner lock and removes it
    /// from the live connection table so no resuming async install, queued broker command, or
    /// stale [`Admitted`] token from that connection can commit or start motion.
    pub fn mark_connection_closed(&mut self, connection_id: u64) {
        self.live_connections.remove(connection_id);
        self.closed_connections
            .mark_closed(connection_id, &self.live_connections);
        self.shared_closed_bitset.mark_closed(connection_id);
        if let Some(rt) = self.rt_closed_connections.as_ref() {
            rt.mark_closed(connection_id);
        }
    }

    /// True when driven by the RT `ReleaseEngine` (live `start_executor` or `ReleaseFixture`).
    pub(crate) fn is_rt_owner(&self) -> bool {
        self.rt_executor_owned || self.rt_closed_connections.is_some()
    }

    /// Attaches the broker's lock-free closed-connection bitset read on `armd-control`.
    pub(crate) fn attach_closed_bitset(
        &mut self,
        bitset: Arc<crate::rt_core::requests::actions::ClosedBitset>,
    ) {
        self.rt_closed_connections = Some(bitset);
        self.prewarm_rt_gripper_spares();
    }

    /// Prewarms the RT `move_samples_spares` pool and `gripper_motion_spare` box off-thread.
    pub(crate) fn prewarm_rt_gripper_spares(&mut self) {
        let target_spares = GRIPPER_BASELINE_PENDING_MAX + 2;
        if self.move_samples_spares.capacity() < target_spares {
            self.move_samples_spares
                .reserve(target_spares - self.move_samples_spares.len());
        }
        while self.move_samples_spares.len() < target_spares {
            self.move_samples_spares
                .push(crate::gripper_baseline::MoveSamples::default());
        }
        if self.gripper_motion_spare.is_none() {
            if let (Some(g_idx), Some(policy)) = (self.config.gripper_index(), self.config.gripper_policy.as_ref()) {
                let lim = &self.config.limits[g_idx];
                let limit_zero = crate::model::JointLimit {
                    name: String::new(),
                    min_position: lim.min_position,
                    max_position: lim.max_position,
                    max_velocity: lim.max_velocity,
                    max_acceleration: lim.max_acceleration,
                    planning_min_position: lim.planning_min_position,
                    planning_max_position: lim.planning_max_position,
                };
                let req = crate::motion::GripperRequest {
                    command: crate::motion::GripperCommand::Open,
                    fraction: None,
                    grasp: false,
                    speed: 0.5,
                    timeout_s: None,
                    tag: None,
                };
                if let Ok(m) = crate::motion::GripperMotion::try_with_close_speed_limit(
                    req,
                    policy.clone_without_jaw_opening(),
                    limit_zero,
                    self.commanded_positions.get(g_idx).copied().unwrap_or(0.0),
                    g_idx,
                    None,
                ) {
                    self.gripper_motion_spare = Some(Box::new(m));
                }
            }
        }
    }

    /// Returns true if WS connection `connection_id` (`>= 1`) has been marked closed under
    /// the legacy owner lock, the lock-free shared disconnect bitset, or the RT broker bitset.
    pub fn is_connection_closed(&self, connection_id: u64) -> bool {
        self.closed_connections.is_closed(connection_id)
            || self.shared_closed_bitset.is_closed(connection_id)
            || self
                .rt_closed_connections
                .as_ref()
                .is_some_and(|rt| rt.is_closed(connection_id))
    }

    /// Returns true if `connection_id` (`>= 1`) is currently registered and open.
    pub fn is_connection_live(&self, connection_id: u64) -> bool {
        !self.is_connection_closed(connection_id)
            && self.live_connections.contains(connection_id)
    }

    /// Number of currently open registered WS connections (`<= MAX_LIVE_CONNECTIONS`).
    pub fn live_connection_count(&self) -> usize {
        self.live_connections.len()
    }

    /// Currently registered live WS connection IDs (used to seed `ConnLifecycleClient` on
    /// legacy -> executor ownership transfer).
    pub fn live_connection_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.live_connections.ids[..self.live_connections.len].iter().copied()
    }

    /// Single owner-side connection liveness check (M2):
    /// - `client_id == 0`: refused (`DISCONNECTED`) whenever leases are enforced; allowed only for
    ///   non-WS internal/REST paths when `lease_enabled == false`.
    /// - `client_id != 0`: ALWAYS requires a registered, open (non-closed) connection in
    ///   `live_connections` (fail-closed `DISCONNECTED`).
    pub(crate) fn check_client_connection_live(&self, client_id: u64) -> Result<(), &'static str> {
        if client_id == 0 {
            if self.lease_enabled {
                return Err("DISCONNECTED");
            }
            return Ok(());
        }
        if self.is_connection_closed(client_id) || !self.live_connections.contains(client_id) {
            return Err("DISCONNECTED");
        }
        Ok(())
    }

    /// Single owner-side binding of a broker/action descriptor's `ClientId` to the current active
    /// lease holder (`active_lease_conn_id`), not only `lease_generation` (M2).
    pub(crate) fn check_descriptor_lease_owner(
        &self,
        client_id: u64,
        lease_generation: u64,
    ) -> Result<(), &'static str> {
        if self.lease_enabled {
            if lease_generation == 0 {
                return Err("NO_LEASE");
            }
            if self.active_lease_conn_id == 0 || self.active_lease_conn_id != client_id {
                return Err("LEASE_LOST");
            }
        } else if lease_generation != 0
            && self.active_lease_conn_id != 0
            && self.active_lease_conn_id != client_id
        {
            return Err("LEASE_LOST");
        }
        Ok(())
    }

    /// Single-use request ID consumption reusing A1's [`ConsumedStamps`] (zero second tracker).
    pub(crate) fn consume_single_use_request_id(&mut self, request_id: u64) -> Result<(), &'static str> {
        if self.is_stamp_consumed(request_id) {
            return Err("STAMP_REUSED");
        }
        self.record_consumed_stamp(request_id);
        Ok(())
    }

    /// Installs or updates `active_lease` in place reusing preallocated string spares (0 alloc / 0 free on RT).
    pub(crate) fn set_active_lease_in_place(
        &mut self,
        conn_id: u64,
        conn_str: &str,
        label: &str,
        generation: u64,
        now: Instant,
    ) {
        if let Some(ref mut active) = self.active_lease {
            active.client_id.clear();
            active.client_id.push_str(conn_str);
            active.label.clear();
            active.label.push_str(label);
            active.generation = generation;
            active.acquired_at = now;
            active.last_valid_motion_instant = None;
        } else {
            let mut client_buf = std::mem::take(&mut self.lease_client_spare);
            client_buf.clear();
            client_buf.push_str(conn_str);
            let mut label_buf = std::mem::take(&mut self.lease_label_spare);
            label_buf.clear();
            label_buf.push_str(label);
            self.active_lease = Some(ActiveLease {
                client_id: client_buf,
                generation,
                label: label_buf,
                acquired_at: now,
                last_valid_motion_instant: None,
            });
        }
        self.active_lease_conn_id = conn_id;
        self.sync_lease_gen_mirror();
    }

    /// Clears `active_lease`, recycling its heap buffers back into spares (0 alloc / 0 free on RT).
    ///
    /// Invariant (enforced by [`Self::prewarm_observation_slots`] and [`Self::set_active_lease_in_place`]):
    /// when `active_lease` is `Some`, `lease_client_spare` and `lease_label_spare` are 0-capacity
    /// `String::new()`, so replacing them performs zero deallocations on `armd-control`.
    pub(crate) fn clear_active_lease_in_place(&mut self) -> Option<u64> {
        let old = self.active_lease.take()?;
        let gen = old.generation;
        if self.lease_client_spare.capacity() == 0
            || (!self.rt_executor_owned
                && old.client_id.capacity() > self.lease_client_spare.capacity())
        {
            let mut buf = old.client_id;
            buf.clear();
            self.lease_client_spare = buf;
        }
        if self.lease_label_spare.capacity() == 0
            || (!self.rt_executor_owned
                && old.label.capacity() > self.lease_label_spare.capacity())
        {
            let mut buf = old.label;
            buf.clear();
            self.lease_label_spare = buf;
        }
        self.active_lease_conn_id = 0;
        self.sync_lease_gen_mirror();
        Some(gen)
    }

    /// Records the most recently evicted `(conn_str, generation)` in place using `evicted_conn_spare`
    /// (0 alloc / 0 free on RT).
    pub(crate) fn set_last_evicted_in_place(&mut self, conn_str: &str, generation: u64) {
        if let Some((ref mut buf, ref mut gen)) = self.last_evicted {
            buf.clear();
            buf.push_str(conn_str);
            *gen = generation;
        } else {
            let mut buf = std::mem::take(&mut self.evicted_conn_spare);
            buf.clear();
            buf.push_str(conn_str);
            self.last_evicted = Some((buf, generation));
        }
    }

    pub(super) fn record_pending_servo_close(&mut self, displaced_gen: u64) {
        if displaced_gen == 0 {
            return;
        }
        let slice = &self.pending_servo_closes[..self.pending_servo_close_len];
        if slice.contains(&displaced_gen) {
            return;
        }
        if self.pending_servo_close_len < self.pending_servo_closes.len() {
            self.pending_servo_closes[self.pending_servo_close_len] = displaced_gen;
            self.pending_servo_close_len += 1;
        } else {
            let last = self.pending_servo_closes.len() - 1;
            self.pending_servo_closes[last] = self.pending_servo_closes[last].max(displaced_gen);
        }
    }

    /// Drains all displaced lease generations recorded since the last drain (zero allocs).
    pub(crate) fn drain_pending_servo_closes(&mut self, mut each: impl FnMut(u64)) {
        let n = self.pending_servo_close_len;
        self.pending_servo_close_len = 0;
        for i in 0..n {
            let gen = self.pending_servo_closes[i];
            if gen != 0 {
                each(gen);
            }
        }
    }

    /// Single owner-side entry point fencing a displaced lease generation (release, expiry,
    /// preemption, or disconnect) BEFORE another owner's output can run.
    pub(crate) fn fence_displaced_lease_generation(
        &mut self,
        displaced_gen: u64,
        stop_active_motion: bool,
    ) {
        if displaced_gen == 0 {
            return;
        }
        self.fenced_lease_generation = self.fenced_lease_generation.max(displaced_gen);
        self.atomic_fenced_lease_gen
            .fetch_max(self.fenced_lease_generation, Ordering::SeqCst);
        self.core.revoke_stream();
        self.record_pending_servo_close(displaced_gen);
        if self.gripper_stream_lease == Some(displaced_gen) {
            self.end_gripper_stream();
        }
        if self.action_lease_generation == Some(displaced_gen) {
            self.pending_deferred_done = None;
        }
        self.sync_lease_gen_mirror();
        #[cfg(all(not(target_arch = "wasm32"), feature = "policy-runtime"))]
        if self
            .policy_load_reservation
            .is_some_and(|r| r.lease_generation <= displaced_gen)
        {
            self.policy_load_reservation = None;
            self.policy_loading = false;
            self.policy_cancel_slots.set_active_load_token(0);
        }
        #[cfg(feature = "policy-runtime")]
        if self
            .policy_run
            .as_ref()
            .is_some_and(|r| r.lease_generation <= displaced_gen)
        {
            self.terminate_policy_run_static(
                "LEASE_LOST",
                "cancelled",
                Some("LEASE_LOST"),
                Some("Control lease lost or expired"),
            );
        }
        if stop_active_motion {
            self.stop_motion();
        }
        self.abort_enable_transition_for_fenced_lease(displaced_gen);
    }

    /// Aborts any in-flight `TorqueTransition::Enable` owned by `displaced_gen`, initiating
    /// bounded background disable cleanup via the driver's single disable choke point when the
    /// arm was not previously torqued.
    pub(crate) fn abort_enable_transition_for_fenced_lease(&mut self, displaced_gen: u64) {
        self.abort_enable_transition_for_fenced_lease_impl(displaced_gen);
    }
}
