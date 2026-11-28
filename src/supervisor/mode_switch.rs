//! Mode-switch runner state machine and `ArmSupervisor` session helpers.

#[allow(unused_imports)]
use super::*;

/// Semantic purpose attached to the active PR0C `ModeSwitch` session on [`SupervisorInner`]
/// (`Copy`, 0 heap alloc). Every terminal transition — normal completion, gate revocation,
/// or external revocation — flows through a single RT completion handler.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum ModeSwitchPurpose {
    /// Direct PR0C `StartModeSwitch` caller (reference gripper confirm, standalone tests).
    Standalone,
    /// Hardware gravity-comp entry (`POS_VEL -> MIT` on J1–J6); payload lives in
    /// `SupervisorInner::pending_gravity_enter`.
    GravityCompEnter,
    /// Stop-direction hardware gravity-comp exit (`MIT -> POS_VEL` on J1–J6).
    GravityCompExit {
        hold: [f32; 6],
    },
    /// MIT tracking activation (`POS_VEL -> MIT` on configured MIT joints).
    MitActivate {
        mask: u64,
        hold_positions: [f32; crate::rt_core::mailbox::MAX_DOF],
        ff: [f32; crate::rt_core::mailbox::MAX_DOF],
    },
    /// Stop-direction MIT tracking handback (`MIT -> POS_VEL` on active MIT joints).
    MitHandBack {
        reason: MitHandBackReason,
        mask: u64,
        hold_positions: [f32; crate::rt_core::mailbox::MAX_DOF],
        ff: [f32; crate::rt_core::mailbox::MAX_DOF],
    },
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MitHandBackReason {
    Park,
    AuthorizationRevoked,
    GravityCompEntry,
    Rearm,
}

/// Preallocated, zero-allocation inline holder for one PR0C `ModeSwitch` session inside
/// [`SupervisorInner`].
#[cfg(not(target_arch = "wasm32"))]
pub(crate) struct ModeSwitchRunner {
    active_session: Option<crate::rt_core::session::SessionId>,
    purpose: ModeSwitchPurpose,
    machine: Option<crate::motor_mode::ModeSwitch>,
    event_tx: Option<crate::rt_core::stream::StreamProducer<crate::motor_mode::ModeSwitchEvent>>,
    retained_events: [Option<crate::motor_mode::ModeSwitchEvent>; 16],
    retained_len: usize,
    recent_terminals: [Option<crate::motor_mode::ModeSwitchSnapshot>; 4],
    recent_terminal_cursor: usize,
}

/// Allocates a collision-free `ActionId` for a router-bound `ModeSwitch` session terminal
/// (session-request namespace bit set, never equal to a broker `ActionId`).
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn next_mode_switch_terminal_action_id() -> crate::rt_core::broker::ActionId {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    let n = NEXT.fetch_add(1, Ordering::Relaxed);
    crate::rt_core::broker::ActionId(crate::rt_core::reply_router::SESSION_REQUEST_ID_BIT | n)
}


#[cfg(not(target_arch = "wasm32"))]
impl ModeSwitchRunner {
    pub(crate) fn new() -> Self {
        Self {
            active_session: None,
            purpose: ModeSwitchPurpose::Standalone,
            machine: None,
            event_tx: None,
            retained_events: [None; 16],
            retained_len: 0,
            recent_terminals: [None; 4],
            recent_terminal_cursor: 0,
        }
    }

    pub(crate) fn attach_event_channel(
        &mut self,
        capacity: usize,
    ) -> crate::rt_core::stream::StreamConsumer<crate::motor_mode::ModeSwitchEvent> {
        let (tx, rx) = crate::rt_core::stream::channel(capacity.max(1));
        self.event_tx = Some(tx);
        rx
    }

    #[inline]
    pub(crate) fn is_running(&self) -> bool {
        self.machine
            .as_ref()
            .is_some_and(crate::motor_mode::ModeSwitch::is_running)
    }

    pub(crate) fn record_terminal(&mut self, snap: crate::motor_mode::ModeSwitchSnapshot) {
        for slot in &mut self.recent_terminals {
            if slot.is_some_and(|s| s.session_id == snap.session_id) {
                *slot = Some(snap);
                return;
            }
        }
        let idx = self.recent_terminal_cursor % self.recent_terminals.len();
        self.recent_terminals[idx] = Some(snap);
        self.recent_terminal_cursor = self.recent_terminal_cursor.wrapping_add(1);
    }

    pub(crate) fn find_terminal(
        &self,
        session_id: u64,
    ) -> Option<crate::motor_mode::ModeSwitchSnapshot> {
        self.recent_terminals
            .iter()
            .flatten()
            .copied()
            .find(|s| s.session_id == session_id)
    }

    pub(crate) fn latest_terminal(&self) -> Option<crate::motor_mode::ModeSwitchSnapshot> {
        if self.recent_terminal_cursor == 0 {
            return self.recent_terminals[0];
        }
        let idx = self.recent_terminal_cursor.wrapping_sub(1) % self.recent_terminals.len();
        self.recent_terminals[idx]
    }

    pub(crate) fn previous_terminal(&self) -> Option<crate::motor_mode::ModeSwitchSnapshot> {
        if self.recent_terminal_cursor < 2 {
            return None;
        }
        let idx = self.recent_terminal_cursor.wrapping_sub(2) % self.recent_terminals.len();
        self.recent_terminals[idx]
    }

    #[inline]
    pub(crate) fn is_idle(&self) -> bool {
        self.active_session.is_none() && self.recent_terminal_cursor == 0
    }

    pub(crate) fn install_session(
        &mut self,
        sid: crate::rt_core::session::SessionId,
        purpose: ModeSwitchPurpose,
        machine: crate::motor_mode::ModeSwitch,
    ) {
        self.active_session = Some(sid);
        self.purpose = purpose;
        self.machine = Some(machine);
    }

    pub(crate) fn retire_if_session(&mut self, sid: crate::rt_core::session::SessionId) {
        if self.active_session == Some(sid) {
            self.active_session = None;
            self.purpose = ModeSwitchPurpose::Standalone;
            self.machine = None;
        }
    }

    fn stage_events_from_machine(&mut self, sid: crate::rt_core::session::SessionId) {
        let Some(m) = self.machine.as_mut() else { return };
        let retained = &mut self.retained_events;
        let len = &mut self.retained_len;
        m.drain_events(sid.0, |ev| {
            if *len < retained.len() {
                retained[*len] = Some(ev);
                *len += 1;
            }
        });
    }

    /// Flushes staged observational events strictly AFTER publication into the preallocated
    /// `rt_core/stream` SPSC ring (drop-newest on saturation; never blocks RT or terminals).
    pub(crate) fn flush_events(&mut self) -> usize {
        let n = self.retained_len;
        if n == 0 {
            return 0;
        }
        let mut pushed = 0usize;
        if let Some(tx) = self.event_tx.as_mut() {
            for slot in &mut self.retained_events[..n] {
                if let Some(ev) = slot.take() {
                    if tx.try_push(ev).is_ok() {
                        pushed += 1;
                    }
                }
            }
        } else {
            for slot in &mut self.retained_events[..n] {
                *slot = None;
            }
        }
        self.retained_len = 0;
        pushed
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl SupervisorInner {
    /// Attaches a preallocated off-RT [`crate::rt_core::stream`] consumer for `ModeSwitchEvent`s.
    pub fn attach_mode_switch_event_stream(
        &mut self,
        capacity: usize,
    ) -> crate::rt_core::stream::StreamConsumer<crate::motor_mode::ModeSwitchEvent> {
        self.mode_switch.attach_event_channel(capacity)
    }

    #[inline]
    pub(crate) fn current_mode_switch_snapshot(
        &self,
    ) -> Option<crate::motor_mode::ModeSwitchSnapshot> {
        let Some(sid) = self.mode_switch.active_session else {
            return self.mode_switch.latest_terminal();
        };
        if let Some(machine) = self.mode_switch.machine.as_ref() {
            let drv_uncertain = self.driver.as_ref().map_or(0, |slot| {
                let mut m = 0u32;
                let _ = slot.try_with_ref(|d| m = d.mode_switch().mode_write_uncertain_mask());
                m
            });
            return Some(machine.snapshot(sid.0, drv_uncertain));
        }
        self.mode_switch.latest_terminal()
    }

    /// True when both bounded gravity-comp retirement queues have headroom for one more
    /// `GravityCompRun` and/or `PendingGravityCompEnter` policy without exceeding capacity.
    pub(crate) fn has_gravity_comp_reclaim_capacity(&self) -> bool {
        #[cfg(not(target_arch = "wasm32"))]
        let pending_enter = usize::from(self.pending_gravity_enter.is_some());
        #[cfg(target_arch = "wasm32")]
        let pending_enter = 0usize;
        let runs_needed = self.retired_gravity_comp.len()
            + usize::from(self.gravity_comp.is_some())
            + pending_enter;
        let policies_needed = self.retired_gravity_comp_policies.len() + pending_enter;
        runs_needed < self.retired_gravity_comp.capacity()
            && policies_needed < self.retired_gravity_comp_policies.capacity()
    }

    /// Read-only inspection helper returning `(retired_runs_len, retired_policies_len)`.
    pub fn retired_gravity_comp_queue_lens(&self) -> (usize, usize) {
        (
            self.retired_gravity_comp.len(),
            self.retired_gravity_comp_policies.len(),
        )
    }

    /// Queues a retired `GravityCompRun` for off-thread drop without allocating or freeing on RT.
    pub(crate) fn retire_gravity_comp_run(&mut self, run: crate::gravity_comp::GravityCompRun) {
        if self.retired_gravity_comp.len() < self.retired_gravity_comp.capacity() {
            self.retired_gravity_comp.push_back(run);
        } else {
            debug_assert!(
                !self.rt_executor_owned,
                "retired_gravity_comp capacity exhausted on RT executor despite admission reservation"
            );
            drop(run);
        }
    }

    pub(crate) fn pop_retired_gravity_comp(&mut self) -> Option<crate::gravity_comp::GravityCompRun> {
        self.retired_gravity_comp.pop_front()
    }

    /// Queues a retired `GravityCompPolicy` for off-thread drop without allocating or freeing on RT.
    pub(crate) fn retire_gravity_comp_policy(&mut self, policy: crate::model::GravityCompPolicy) {
        if self.retired_gravity_comp_policies.len() < self.retired_gravity_comp_policies.capacity() {
            self.retired_gravity_comp_policies.push_back(policy);
        } else {
            debug_assert!(
                !self.rt_executor_owned,
                "retired_gravity_comp_policies capacity exhausted on RT executor despite admission reservation"
            );
            drop(policy);
        }
    }

    pub(crate) fn pop_retired_gravity_comp_policy(&mut self) -> Option<crate::model::GravityCompPolicy> {
        self.retired_gravity_comp_policies.pop_front()
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[inline]
    pub(crate) fn driver_supports_nonblocking_mode_switch(&self) -> bool {
        self.driver
            .as_ref()
            .is_some_and(|d| d.try_with_ref(|drv| drv.mode_switch().supports_nonblocking_mode_switch()).unwrap_or(false))
    }

    /// True when any exclusive PR0B session (e.g. `ModeSwitch` or `Commissioning`) owns output
    /// OR a session has requested SAFE-HOLD (mutual exclusion against legacy gravity-comp & MIT).
    #[inline]
    pub(crate) fn exclusive_session_owns_output(&self) -> bool {
        self.sessions.is_safe_hold_requested()
            || self.mode_switch.is_running()
            || self
                .sessions
                .slot(crate::rt_core::session::SessionSlot::ExclusiveOutput)
                .is_some_and(|s| s.is_live())
    }

    #[inline]
    pub(crate) fn mode_switch_owns_bus(&self) -> bool {
        let Some(slot) = self
            .sessions
            .slot(crate::rt_core::session::SessionSlot::ExclusiveOutput)
        else {
            return false;
        };
        if !self.mode_switch.is_running()
            || slot.kind != crate::rt_core::session::SessionKind::ModeSwitch
            || !slot.is_live()
        {
            return false;
        }
        if !self.safety.state().is_stopped() {
            return true;
        }
        // Owner-internal stop-direction MIT -> POS_VEL handback in non-E-stop Fault may own
        // the bus exclusively ONLY when no E-stop is pending or latched (never bypassing
        // Damiao's TxKind::ModeWrite E-stop gate).
        matches!(
            slot.authority.mode,
            crate::rt_core::session::SessionAuthorityMode::PosVelHandback {
                allow_in_fault: true,
                ..
            }
        ) && !self.estop_pending()
            && !self.urgent_stop_unapplied()
            && !matches!(self.safety.state(), crate::safety::SafetyState::EStop)
    }

    #[allow(clippy::result_large_err)]
    pub(crate) fn start_mode_switch_rt(
        &mut self,
        req: crate::rt_core::requests::config::ModeSwitchStartRequest,
        mut terminal: Option<crate::rt_core::reply_router::SessionTerminalReservation>,
        retired: &mut crate::rt_core::requests::config::ConfigRetiredPayloads,
        owner_now: Instant,
    ) -> Result<u64, crate::rt_core::requests::config::ConfigError> {
        use crate::rt_core::requests::config::ConfigError;
        use crate::rt_core::session::{
            admit_internal, admit_routed_retiring, PreparedSession, SessionAdmitError, SessionKind,
        };
        let retire_terminal =
            |term: &mut Option<crate::rt_core::reply_router::SessionTerminalReservation>,
             ret: &mut crate::rt_core::requests::config::ConfigRetiredPayloads| {
                if let Some(res) = term.take() {
                    let (_, handoff) = res.into_parts();
                    ret.session_terminal_handoff = Some(handoff);
                }
            };
        if self.rt_executor_owned && req.client.0 != 0 && terminal.is_none() {
            return Err(ConfigError::CalibPrecondition {
                status: 409,
                code: "NO_DESTINATION",
                message: "routed mode-switch session requires a reserved terminal destination",
            });
        }
        if !self.rt_executor_owned && terminal.is_some() {
            retire_terminal(&mut terminal, retired);
            return Err(ConfigError::OwnerStopped);
        }
        if self.mode_switch.active_session.is_some() {
            retire_terminal(&mut terminal, retired);
            return Err(ConfigError::Busy("a mode-switch session is already active"));
        }
        if req.plan.validate().is_err() {
            retire_terminal(&mut terminal, retired);
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "invalid mode-switch plan",
            });
        }
        let period =
            Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        if req.plan.max_watchdog_gap
            < period.saturating_add(crate::motor_mode::DEFAULT_HOLD_DELIVERY_MARGIN)
        {
            retire_terminal(&mut terminal, retired);
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "WATCHDOG_GAP_INFEASIBLE",
                message: "mode-switch watchdog gap is shorter than one control period plus the hold delivery margin",
            });
        }
        if let Some(slot) = self.driver.as_ref() {
            let mut budget_ok = Ok(());
            let _ = slot.try_with_ref(|drv| {
                budget_ok = drv.tx().check_watchdog_tx_budget_for_energized();
            });
            if budget_ok.is_err() {
                retire_terminal(&mut terminal, retired);
                return Err(ConfigError::CalibPrecondition {
                    status: 400,
                    code: "WATCHDOG_BUDGET_INSUFFICIENT",
                    message: "tx_budget.max_frames is smaller than energized arm axes + J7 + 1",
                });
            }
        }
        let cur_epoch = self.safety.epoch_id();
        let admitted = match self.check_admission(req.admission) {
            Ok(a) => a,
            Err(e) => {
                retire_terminal(&mut terminal, retired);
                return Err(match e {
                    SafetyError::StampAlreadyConsumed => ConfigError::StampConsumed,
                    SafetyError::LeaseLost => ConfigError::LeaseLost,
                    SafetyError::Stopped => ConfigError::Stopped,
                    SafetyError::EpochMismatch { .. } => ConfigError::EpochMismatch {
                        current_epoch: cur_epoch,
                    },
                    SafetyError::EStopActive => ConfigError::CalibPrecondition {
                        status: 409,
                        code: "ESTOP_ACTIVE",
                        message: "e-stop active",
                    },
                    _ => ConfigError::CalibPrecondition {
                        status: 409,
                        code: "REJECTED",
                        message: "mode-switch admission rejected",
                    },
                });
            }
        };
        let mut prepared = PreparedSession::exclusive(SessionKind::ModeSwitch, req.client);
        prepared.deadline = Some(req.deadline);
        prepared.timeout_code = "TIMEOUT";
        prepared.operator = req.operator;
        prepared.revoke_on_stop = true;
        prepared.escalate_safe_hold_on_uncertain = true;
        let admit_res = match terminal.take() {
            Some(reservation) => admit_routed_retiring(
                admitted,
                prepared,
                reservation,
                &mut retired.session_terminal_handoff,
                owner_now,
            ),
            None => admit_internal(admitted, prepared, owner_now),
        };
        let sid = admit_res.map_err(|err| match err {
            SessionAdmitError::SlotBusy | SessionAdmitError::OutputBusy(_) => {
                ConfigError::Busy(err.code())
            }
            SessionAdmitError::DeadlineExpired => ConfigError::CalibPrecondition {
                status: 408,
                code: "DEADLINE_EXPIRED",
                message: "mode-switch deadline already expired",
            },
            SessionAdmitError::InvalidSlotKind => ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "invalid mode-switch session slot",
            },
            SessionAdmitError::InvalidReservation => ConfigError::CalibPrecondition {
                status: 409,
                code: "NO_DESTINATION",
                message: "invalid or missing mode-switch terminal reservation",
            },
            SessionAdmitError::ReservationCancelled => ConfigError::Busy("CANCELLED"),
            SessionAdmitError::AuthorityRejected("CONNECTION_CLOSED") => {
                ConfigError::ConnectionClosed
            }
            SessionAdmitError::AuthorityRejected("LEASE_LOST") => ConfigError::LeaseLost,
            SessionAdmitError::AuthorityRejected("EPOCH_MISMATCH") => ConfigError::EpochMismatch {
                current_epoch: self.safety.epoch_id(),
            },
            SessionAdmitError::AuthorityRejected("STOPPED") => ConfigError::Stopped,
            SessionAdmitError::AuthorityRejected("OPERATOR_REQUIRED") => {
                ConfigError::CalibPrecondition {
                    status: 403,
                    code: "OPERATOR_REQUIRED",
                    message: "operator authorization required",
                }
            }
            SessionAdmitError::AuthorityRejected(code) => ConfigError::CalibPrecondition {
                status: 409,
                code,
                message: "mode-switch authority rejected",
            },
        })?;
        // R2 Major 3: Assert `mode_switch_active_mask` immediately at admission (BEFORE the
        // opening release's Step 2 RX runs) so optional Step 2 refresh/recovery cannot spend
        // release TX slots ahead of ModeSwitch.
        let mut plan_mask = 0u32;
        for target in req.plan.axis_slice() {
            if usize::from(target.joint) < crate::motor_mode::MAX_MODE_SWITCH_AXES {
                plan_mask |= 1u32 << target.joint;
            }
        }
        if let Some(slot) = self.driver.as_mut() {
            let _ = slot.try_with_mut(|drv| drv.mode_switch_mut().set_mode_switch_active_mask(plan_mask));
        }
        self.install_mode_switch_session(
            sid,
            ModeSwitchPurpose::Standalone,
            crate::motor_mode::ModeSwitch::new(req.plan),
        );
        Ok(sid.0)
    }

    /// Single admission point installing a `ModeSwitch` session into `self.mode_switch` (0 alloc).
    pub(crate) fn install_mode_switch_session(
        &mut self,
        sid: crate::rt_core::session::SessionId,
        purpose: ModeSwitchPurpose,
        machine: crate::motor_mode::ModeSwitch,
    ) {
        self.mode_switch.install_session(sid, purpose, machine);
    }

    pub(crate) fn poll_mode_switch_rt(
        &self,
        session_id: u64,
    ) -> Option<crate::motor_mode::ModeSwitchSnapshot> {
        let drv_uncertain = self.driver.as_ref().map_or(0, |slot| {
            let mut m = 0u32;
            let _ = slot.try_with_ref(|d| m = d.mode_switch().mode_write_uncertain_mask());
            m
        });
        if let (Some(sid), Some(machine)) =
            (self.mode_switch.active_session, self.mode_switch.machine.as_ref())
        {
            if sid.0 == session_id {
                return Some(machine.snapshot(sid.0, drv_uncertain));
            }
        }
        self.mode_switch.find_terminal(session_id)
    }

    /// Owner proof that a stop effect (E-stop, general stop, or qualifying fault stop) was
    /// APPLIED by this same owner generation strictly after the active `ModeSwitch` session was
    /// admitted. Pre-existing faults/already-stopped states, pending stop requests, policy-only
    /// stops, and lease loss never qualify.
    #[inline]
    pub(crate) fn mode_switch_stop_proof(
        &self,
    ) -> Option<crate::rt_core::authority::AuthorityProof> {
        let sid = self.mode_switch.active_session?;
        let active = self
            .sessions
            .slot(crate::rt_core::session::SessionSlot::ExclusiveOutput)
            .filter(|s| s.id == sid)?;
        self.stop_applied_since(active.authority.baseline)
    }

    #[inline]
    pub(crate) fn mode_switch_stop_applied(&self) -> bool {
        self.mode_switch_stop_proof().is_some()
    }

    /// Single RT convergence point invoked ONCE whenever a `ModeSwitch` session reaches a
    /// terminal state (normal completion, in-step gate revocation, or external revocation).
    /// Applies purpose-specific state transitions (0 heap alloc, 0 RT dealloc).
    pub(crate) fn apply_mode_switch_terminal_rt(
        &mut self,
        sid: crate::rt_core::session::SessionId,
        purpose: ModeSwitchPurpose,
        progress: crate::motor_mode::ModeProgress,
        uncertain: bool,
    ) {
        if let Some(slot) = self.driver.as_mut() {
            let _ = slot.try_with_mut(|drv| drv.mode_switch_mut().set_mode_switch_active_mask(0));
        }
        match purpose {
            ModeSwitchPurpose::Standalone => {}
            ModeSwitchPurpose::GravityCompEnter => {
                self.on_gravity_comp_enter_mode_switch_terminal_rt(sid.0, progress, uncertain);
            }
            ModeSwitchPurpose::GravityCompExit { hold } => {
                self.on_gravity_comp_exit_mode_switch_terminal_rt(hold, progress, uncertain);
            }
            ModeSwitchPurpose::MitActivate { mask, hold_positions, ff } => {
                self.on_mit_activate_mode_switch_terminal_rt(mask, &hold_positions, &ff, progress, uncertain);
            }
            ModeSwitchPurpose::MitHandBack { reason, mask, hold_positions, ff } => {
                self.on_mit_handback_mode_switch_terminal_rt(reason, mask, &hold_positions, &ff, progress, uncertain);
            }
        }
    }

    /// Reconciles active `ModeSwitch` state and driver `mode_write_uncertain_mask` /
    /// `mode_switch_active_mask` on ANY revocation path (stop, E-stop, disconnect, expiry,
    /// cancel, fault, SAFE-HOLD, handback), syncing `hw_state_uncertain` into `SessionDriver`.
    pub(crate) fn reconcile_mode_switch_on_revocation(&mut self, reason_code: &'static str) -> bool {
        let Some(sid) = self.mode_switch.active_session else {
            return false;
        };
        let Some(mut machine) = self.mode_switch.machine.take() else {
            return false;
        };
        let purpose = self.mode_switch.purpose;
        let reason = crate::motor_mode::ModeSwitchFailure::from_revoke_code(reason_code);
        let mut drv_uncertain = 0u32;
        let stop_applied = self.mode_switch_stop_applied();
        let progress = if let Some(slot) = self.driver.as_mut() {
            let mut p = None;
            let _ = slot.try_with_mut(|drv| {
                p = Some(machine.reconcile_external_revocation(Some(drv), reason, stop_applied));
                drv_uncertain = drv.mode_switch().mode_write_uncertain_mask();
            });
            p.unwrap_or_else(|| machine.reconcile_external_revocation(None, reason, stop_applied))
        } else {
            machine.reconcile_external_revocation(None, reason, stop_applied)
        };
        let uncertain = progress.is_ambiguous() || drv_uncertain != 0;
        let snap = machine.snapshot(sid.0, drv_uncertain);
        self.mode_switch.machine = Some(machine);
        self.mode_switch.purpose = ModeSwitchPurpose::Standalone;
        self.mode_switch.stage_events_from_machine(sid);
        self.mode_switch.record_terminal(snap);
        self.apply_mode_switch_terminal_rt(sid, purpose, progress, uncertain);
        let _ = self.sessions.set_hw_state_uncertain(
            crate::rt_core::session::SessionSlot::ExclusiveOutput,
            sid,
            uncertain,
        );
        uncertain
    }

    pub(crate) fn cancel_mode_switch_rt(
        &mut self,
        session_id: u64,
        reason_code: &'static str,
    ) -> Option<crate::motor_mode::ModeSwitchSnapshot> {
        use crate::rt_core::session::{SessionSlot, SessionTerminalReason};
        let sid = self.mode_switch.active_session?;
        if sid.0 != session_id {
            return None;
        }
        let escalate = self
            .sessions
            .slot(SessionSlot::ExclusiveOutput)
            .is_some_and(|s| s.id == sid && s.escalate_safe_hold_on_uncertain);
        let uncertain = self.reconcile_mode_switch_on_revocation(reason_code);
        if let Some(outcome) = self.sessions.cancel_slot(
            SessionSlot::ExclusiveOutput,
            sid,
            SessionTerminalReason::Cancelled(reason_code),
            uncertain,
        ) {
            self.maybe_escalate_session_safe_hold(outcome, escalate);
        }
        self.poll_mode_switch_rt(session_id)
    }

    /// Records an admission-bound one-shot MIT qualification proof on RT (0 alloc).
    /// Verifies live epoch, stop_gen, calibration_revision, motion_config_revision, zero offsets
    /// digest, and `sched::mit_refusal()` before storing; rejects stale proofs fail-closed.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn record_mit_gate_proof_rt(
        &mut self,
        proof: crate::rt_core::requests::config::MitGateProof,
        now: Instant,
    ) -> bool {
        if crate::rt_core::sched::mit_refusal().is_some()
            || self.estop_pending()
            || self.safety.state().is_stopped()
            || self.safety.epoch_id() != proof.estop_epoch
            || self.current_stop_gen() != proof.stop_gen
            || self.calibration_revision != proof.calibration_revision
            || self.motion_config_revision != proof.motion_config_revision
            || crate::mit::zero_offsets_hash(&self.joint_zero_offsets) != proof.zero_offsets_hash
            || now.saturating_duration_since(proof.verified_at) > Duration::from_secs(5)
        {
            self.mit_gate_proof = None;
            return false;
        }
        self.mit.qualified_holdoff = proof.holdoff;
        self.mit_gate_proof = Some(proof);
        true
    }

    pub(crate) fn clear_mode_switch_rt(&mut self, session_id: u64) -> bool {
        use crate::rt_core::session::SessionSlot;
        let Some(sid) = self.mode_switch.active_session else {
            return self.mode_switch.find_terminal(session_id).is_some();
        };
        if sid.0 != session_id {
            return false;
        }
        if self.mode_switch.is_running() {
            let _ = self.cancel_mode_switch_rt(session_id, "CANCELLED");
        }
        if !self.sessions.clear_terminal_slot(SessionSlot::ExclusiveOutput, sid) {
            return false;
        }
        self.mode_switch.active_session = None;
        self.mode_switch.purpose = ModeSwitchPurpose::Standalone;
        self.mode_switch.machine = None;
        true
    }

    pub(crate) fn service_mode_switch_release_with_clock(
        &mut self,
        ctx: &crate::rt_core::ops::CommitCtx,
        release_deadline: Instant,
        owner_now_fn: &mut dyn FnMut() -> Instant,
    ) {
        let mut ledger = crate::rt_core::budget::ReleaseBudget::new(
            self.control_release_seq,
            ctx.now,
            release_deadline,
            crate::rt_core::ops::RX_FRAME_BUDGET,
            self.release_tx_budget_limit(),
        );
        self.service_mode_switch_release_with_budget(ctx, &mut ledger, owner_now_fn);
    }

    pub(crate) fn service_mode_switch_release_with_budget(
        &mut self,
        ctx: &crate::rt_core::ops::CommitCtx,
        release_budget: &mut crate::rt_core::budget::ReleaseBudget,
        owner_now_fn: &mut dyn FnMut() -> Instant,
    ) {
        use crate::motor_mode::{ModeProgress, ModeSwitchFailure, ReleaseBudget};
        use crate::rt_core::session::{
            PhaseStepOutcome, SessionKind, SessionSlot, SessionStep, SessionTerminalReason,
        };
        let Some(active) = self.sessions.slot(SessionSlot::ExclusiveOutput).copied() else {
            return;
        };
        if !active.is_live() || active.kind != SessionKind::ModeSwitch {
            return;
        }
        let sid = active.id;
        let now = owner_now_fn();
        let release_deadline = release_budget.deadline();
        let total_span = release_deadline.saturating_duration_since(now);
        let margin = self.tx_release_publish_margin().min(total_span / 2);
        let base_cutoff = release_deadline.checked_sub(margin).unwrap_or(now);
        let rctx = self.build_session_release_ctx(
            &crate::rt_core::ops::CommitCtx { now, ..*ctx },
            release_deadline,
            base_cutoff,
        );
        let conn_closed = active.authority.connection_id != 0
            && self.is_connection_closed(active.authority.connection_id);
        let lease_ok = !active.authority.require_lease
            || self.is_session_lease_valid_at(
                active.authority.connection_id,
                active.authority.lease_generation,
                now,
            );
        let op_ok = self
            .check_operator_requirement(active.authority.operator, now)
            .is_ok();

        // Pre-phase live authority check via SessionDriver.
        if let Err(reason) = self.sessions.check_live_authority(
            SessionSlot::ExclusiveOutput,
            sid,
            &rctx,
            |_| conn_closed,
            |_, _, _| lease_ok,
            |_, _| op_ok,
        ) {
            let purpose = self.mode_switch.purpose;
            let uncertain = self.reconcile_mode_switch_on_revocation(reason.code());
            if let Some(outcome) =
                self.sessions
                    .finish_slot(SessionSlot::ExclusiveOutput, sid, reason, uncertain)
            {
                self.maybe_escalate_session_safe_hold(outcome, active.escalate_safe_hold_on_uncertain);
                if matches!(active.terminal, crate::rt_core::session::SessionTerminalBinding::Internal)
                    && !matches!(purpose, ModeSwitchPurpose::Standalone)
                {
                    let _ = self.sessions.clear_terminal_slot(SessionSlot::ExclusiveOutput, sid);
                    self.mode_switch.retire_if_session(sid);
                }
            }
            return;
        }

        let effective_cutoff = self
            .sessions
            .effective_tx_cutoff(SessionSlot::ExclusiveOutput, base_cutoff)
            .min(release_budget.deadline());
        let Some(mut machine) = self.mode_switch.machine.take() else {
            if let Some(outcome) = self.sessions.finish_slot(
                SessionSlot::ExclusiveOutput,
                sid,
                SessionTerminalReason::Failed("INTERNAL"),
                false,
            ) {
                self.maybe_escalate_session_safe_hold(outcome, active.escalate_safe_hold_on_uncertain);
            }
            return;
        };

        let purpose = self.mode_switch.purpose;
        let period = Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz.max(1)));
        let max_tx_frames = self
            .config
            .damiao_tx_budget()
            .max_frames
            .min(u8::MAX as usize) as u8;
        let before_drv = self.sync_driver_budget_before(release_budget);
        let (ledger_rx_rem, ledger_tx_rem, max_tx_frames) = if before_drv.is_some() {
            (
                release_budget.rx_remaining(),
                release_budget.tx_remaining(),
                max_tx_frames,
            )
        } else {
            (0, 0, 0)
        };
        let ledger_seq = release_budget.release_seq();

        // Temporarily take driver out (restored unconditionally even on unwind) so the live
        // pre-TX gate closure can borrow `&*self` right before `can.transmit` (0 alloc).
        let mut taken_drv = self.driver.take();
        self.mode_switch_ran_release_seq = self.control_release_seq;
        let mut drv_uncertain = 0u32;
        let mut ms_rx_used: u16 = 0;
        let mut ms_rx_exhausted = false;
        let mut ms_tx_used: u16 = 0;
        let mut mit_fence_tripped = false;
        let mut handback_committed_tx: Option<Instant> = None;
        let step_progress = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let Some(slot) = taken_drv.as_mut() else {
                return ModeProgress::FailedClean {
                    failed_joint: 0,
                    reason: ModeSwitchFailure::UnsupportedMode,
                };
            };
            let mut out = ModeProgress::Pending { wake: None };
            let locked = slot.try_with_mut(|drv| {
                // Check driver MIT deadline fence BEFORE stepping activation/handback so a
                // prior fence trip never lets activation promote to Active.
                if matches!(
                    purpose,
                    ModeSwitchPurpose::MitActivate { .. } | ModeSwitchPurpose::MitHandBack { .. }
                ) && drv.mit().mit_fence_trip_bounded().is_some()
                {
                    mit_fence_tripped = true;
                    drv_uncertain = drv.mode_switch().mode_write_uncertain_mask();
                    out = machine.reconcile_external_revocation(
                        Some(drv),
                        ModeSwitchFailure::WatchdogBudgetBreached,
                        false,
                    );
                    return;
                }
                let Ok((drv_now, _, drv_cutoff)) =
                    translate_driver_cutoff_conservative(drv, effective_cutoff, &mut *owner_now_fn)
                else {
                    out = ModeProgress::Pending { wake: None };
                    drv_uncertain = drv.mode_switch().mode_write_uncertain_mask();
                    return;
                };
                let mut drv_ledger = crate::rt_core::budget::ReleaseBudget::new(
                    ledger_seq,
                    drv_now,
                    drv_cutoff,
                    ledger_rx_rem,
                    ledger_tx_rem,
                );
                let budget = ReleaseBudget::from_ledger(
                    &drv_ledger,
                    drv_now,
                    drv_cutoff,
                    period,
                    max_tx_frames,
                );
                let mut gate = || -> Result<(), &'static str> {
                    let gate_now = owner_now_fn();
                    self.check_session_pre_tx(
                        SessionSlot::ExclusiveOutput,
                        sid,
                        gate_now,
                        base_cutoff,
                    )
                    .map(|_| ())
                };
                let stop_proof = || self.mode_switch_stop_applied();
                drv.set_tx_release_deadline(Some(drv_cutoff));
                out = machine.step_with_gate_and_stop_proof_ledger(
                    drv,
                    &mut gate,
                    &stop_proof,
                    budget,
                    &mut drv_ledger,
                );
                drv.set_tx_release_deadline(None);
                ms_rx_used = drv_ledger.rx_used();
                ms_rx_exhausted = drv_ledger.rx_exhausted();
                ms_tx_used = drv_ledger.tx_used();
                drv_uncertain = drv.mode_switch().mode_write_uncertain_mask();
                if matches!(
                    purpose,
                    ModeSwitchPurpose::MitActivate { .. } | ModeSwitchPurpose::MitHandBack { .. }
                ) {
                    mit_fence_tripped = drv.mit().mit_fence_trip_bounded().is_some();
                    if let ModeSwitchPurpose::MitHandBack { mask, .. } = purpose {
                        if !mit_fence_tripped && !out.is_terminal() {
                            // Supply genuine oldest committed control-frame TX timestamp across
                            // the handback axes so `mit_supervise()` observes real committed holds
                            // rather than declaring ownership lost during a healthy multi-tick handback.
                            handback_committed_tx = drv.mit().mit_min_last_ctrl_tx(mask);
                        }
                    }
                }
            });
            if locked.is_err() {
                ModeProgress::Pending { wake: None }
            } else {
                out
            }
        }));
        self.driver = taken_drv;
        if step_progress.is_err() {
            if let Some(d) = self.driver.as_mut() {
                let _ = d.try_with_mut(|drv| drv.set_tx_release_deadline(None));
            }
        }
        self.charge_driver_budget_after(
            release_budget,
            before_drv,
            ms_rx_used,
            ms_rx_exhausted,
            ms_tx_used,
        );

        let mut progress = match step_progress {
            Ok(p) => p,
            Err(payload) => {
                self.mode_switch.machine = Some(machine);
                std::panic::resume_unwind(payload);
            }
        };

        if let Some(ts) = handback_committed_tx {
            if self.mit.owner == crate::mit::MitOwner::Active {
                self.mit.last_output = Some(match self.mit.last_output {
                    Some(prev) => prev.max(ts),
                    None => ts,
                });
            }
        }

        // If the MIT deadline fence tripped during activation/handback, never allow Confirmed.
        if mit_fence_tripped && !progress.is_ambiguous() {
            progress = ModeProgress::Ambiguous {
                failed_joint: 0,
                reason: ModeSwitchFailure::WatchdogBudgetBreached,
            };
        }

        let uncertain_now = progress.is_ambiguous() || drv_uncertain != 0;
        let _ = self
            .sessions
            .set_hw_state_uncertain(SessionSlot::ExclusiveOutput, sid, uncertain_now);

        let phase_out = match progress {
            ModeProgress::Pending { .. } => PhaseStepOutcome::Pending,
            ModeProgress::Confirmed => PhaseStepOutcome::Terminal {
                reason: SessionTerminalReason::Completed,
                uncertain_hw_state: false,
            },
            ModeProgress::RolledBack { reason, .. } => PhaseStepOutcome::Terminal {
                reason: SessionTerminalReason::Failed(reason.code()),
                uncertain_hw_state: false,
            },
            ModeProgress::StoppedRecovered { reason } => PhaseStepOutcome::Terminal {
                reason: SessionTerminalReason::Cancelled(reason.code()),
                uncertain_hw_state: false,
            },
            ModeProgress::FailedClean { reason, .. } => PhaseStepOutcome::Terminal {
                reason: SessionTerminalReason::Failed(reason.code()),
                uncertain_hw_state: false,
            },
            ModeProgress::Ambiguous { reason, .. } => PhaseStepOutcome::Terminal {
                reason: SessionTerminalReason::SafeHoldUncertain(reason.code()),
                uncertain_hw_state: true,
            },
        };

        let snap = machine.snapshot(sid.0, drv_uncertain);
        self.mode_switch.machine = Some(machine);
        self.mode_switch.stage_events_from_machine(sid);
        if progress.is_terminal() {
            self.mode_switch.purpose = ModeSwitchPurpose::Standalone;
            self.mode_switch.record_terminal(snap);
            self.apply_mode_switch_terminal_rt(sid, purpose, progress, uncertain_now);
        }

        if let Some(SessionStep::Terminal(outcome)) = self.sessions.step_slot(
            SessionSlot::ExclusiveOutput,
            &rctx,
            |_| false,
            |_, _, _| true,
            |_, _| true,
            |_, _| uncertain_now,
            |_, _, _| phase_out,
        ) {
            self.maybe_escalate_session_safe_hold(outcome, active.escalate_safe_hold_on_uncertain);
            // Owner-internal sessions (no external request/router terminal waiting to call
            // ClearModeSwitch) retire their terminal slot immediately so subsequent ModeSwitch
            // sessions (e.g. MitHandBack, GravityCompEnter/Exit) are never blocked with SlotBusy.
            if matches!(active.terminal, crate::rt_core::session::SessionTerminalBinding::Internal)
                && !matches!(purpose, ModeSwitchPurpose::Standalone)
            {
                let _ = self.sessions.clear_terminal_slot(SessionSlot::ExclusiveOutput, sid);
                self.mode_switch.retire_if_session(sid);
            }
        }
    }
}

impl ArmSupervisor {
    // ---- PR0C: Dual-owner ModeSwitch session orchestrator (PR0A/PR0B shared layer) ----

    /// Admits a PR0C `ModeSwitch` session on the active owner (`OWNER_LEGACY` or `OWNER_EXECUTOR`)
    /// after verifying live service coherence via [`Self::read_services`].
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn start_mode_switch_session(
        &self,
        req: crate::rt_core::requests::config::ModeSwitchStartRequest,
    ) -> Result<u64, crate::rt_core::requests::config::ConfigError> {
        use crate::rt_core::requests::config::{ConfigError, ConfigReply, ConfigRequest};
        let _svc = self
            .read_services(crate::rt_core::publish::ReadRequirements::identity())
            .await
            .map_err(|_| ConfigError::OwnerStopped)?;
        // Routed requests on OWNER_EXECUTOR (`req.client.0 != 0`) MUST obtain a typed P-F1
        // terminal reservation via the shared RAII guard BEFORE enqueue; missing destination or
        // disconnected clients fail closed (never silently downgraded to `None`).
        let handle = self.executor_handle();
        let mut terminal_guard = match handle.as_ref() {
            Some(h) if req.client.0 != 0 => {
                use crate::rt_core::reply_router::{ExternalDestinationPolicy, ExternalSubmitError};
                match h.reserve_session_terminal(
                    req.client,
                    next_mode_switch_terminal_action_id(),
                    ExternalDestinationPolicy::ConnectionOnly,
                ) {
                    Ok(guard) => Some(guard),
                    Err(ExternalSubmitError::Admit(crate::rt_core::broker::AdmitError::RepliesFull)) => {
                        return Err(ConfigError::Busy("REPLIES_FULL"));
                    }
                    Err(ExternalSubmitError::Admit(crate::rt_core::broker::AdmitError::Disconnected)) => {
                        return Err(ConfigError::ConnectionClosed);
                    }
                    Err(ExternalSubmitError::NoDestination) => {
                        return Err(ConfigError::CalibPrecondition {
                            status: 409,
                            code: "NO_DESTINATION",
                            message: "routed mode-switch session has no registered reply destination",
                        });
                    }
                    Err(ExternalSubmitError::Stopped) => return Err(ConfigError::OwnerStopped),
                    Err(ExternalSubmitError::NotAnExternalAction) => {
                        return Err(ConfigError::CalibPrecondition {
                            status: 400,
                            code: "INVALID_COMMAND",
                            message: "session terminal reservation requires an external routed action",
                        });
                    }
                    Err(ExternalSubmitError::Admit(_)) => {
                        return Err(ConfigError::Busy("ADMIT_REJECTED"));
                    }
                }
            }
            _ => None,
        };
        let reservation = terminal_guard
            .as_mut()
            .and_then(|g| g.stage_for_admission());
        let res = match self
            .dispatch_config_request_owned_reliable(ConfigRequest::StartModeSwitch(
                req,
                reservation,
            ))
            .await
        {
            Ok(ConfigReply::ModeSwitchStarted(res)) => res,
            Ok(_) => Err(ConfigError::OwnerStopped),
            Err(e) => Err(e),
        };
        if res.is_ok() {
            if let Some(g) = terminal_guard.as_mut() {
                g.commit_transferred();
            }
        }
        res
    }

    /// Polls the authoritative [`crate::motor_mode::ModeSwitchSnapshot`] for `session_id`
    /// directly via [`Self::read_services`] (no config exchange, no write lock, no command label
    /// mutation, and unaffected by off-RT event stream saturation). Checks both the active slot
    /// and the bounded recent-terminal ring, and falls back to the latest published `StateHub`
    /// snapshot if the owner just transitioned to SAFE-HOLD.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn poll_mode_switch_session(
        &self,
        session_id: u64,
    ) -> Result<Option<crate::motor_mode::ModeSwitchSnapshot>, crate::rt_core::requests::config::ConfigError>
    {
        let lookup = |meta: &crate::rt_core::publish::PublishedOwnerMeta| {
            meta.mode_switch
                .filter(|s| s.session_id == session_id)
                .or_else(|| {
                    meta.prev_mode_switch_terminal
                        .filter(|s| s.session_id == session_id)
                })
                .map(crate::motor_mode::ModeSwitchSnapshot::from)
        };
        if let Ok(svc) = self
            .read_services(crate::rt_core::publish::ReadRequirements::identity())
            .await
        {
            return Ok(lookup(&svc.snapshot.rt.owner));
        }
        if let Some(snap) = self.hub.latest().and_then(|latest| lookup(&latest.rt.owner)) {
            return Ok(Some(snap));
        }
        Err(crate::rt_core::requests::config::ConfigError::OwnerStopped)
    }

    /// Cancels an active `ModeSwitch` session matching `session_id` (stale IDs return `Ok(None)`).
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn cancel_mode_switch_session(
        &self,
        session_id: u64,
        reason: &'static str,
    ) -> Result<Option<crate::motor_mode::ModeSwitchSnapshot>, crate::rt_core::requests::config::ConfigError>
    {
        use crate::rt_core::requests::config::{ConfigError, ConfigReply, ConfigRequest};
        match self
            .dispatch_config_request_owned_reliable(ConfigRequest::CancelModeSwitch {
                session_id,
                reason,
            })
            .await?
        {
            ConfigReply::ModeSwitchCancelled(snap) => Ok(snap),
            _ => Err(ConfigError::OwnerStopped),
        }
    }

    /// Clears a terminal `ModeSwitch` slot matching `session_id` (returns `false` for stale IDs
    /// or while a router-bound terminal is still pending flush).
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn clear_mode_switch_session(
        &self,
        session_id: u64,
    ) -> Result<bool, crate::rt_core::requests::config::ConfigError> {
        use crate::rt_core::requests::config::{ConfigError, ConfigReply, ConfigRequest};
        match self
            .dispatch_config_request_owned_reliable(ConfigRequest::ClearModeSwitch { session_id })
            .await?
        {
            ConfigReply::ModeSwitchCleared(cleared) => Ok(cleared),
            _ => Err(ConfigError::OwnerStopped),
        }
    }

    /// Waits via `read_services` (`poll_mode_switch_session`) until `session_id` reaches terminal
    /// or `wait_deadline` elapses (cancelling on timeout), clearing the slot when safe.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) async fn await_mode_switch_session(
        &self,
        session_id: u64,
        wait_deadline: Instant,
    ) -> Result<Option<crate::motor_mode::ModeSwitchSnapshot>, crate::rt_core::requests::config::ConfigError>
    {
        loop {
            match self.poll_mode_switch_session(session_id).await {
                Ok(Some(snap)) if !snap.running => {
                    if !snap.progress.is_ambiguous() && !self.internal_fault_lane.is_fenced() {
                        let _ = self.clear_mode_switch_session(session_id).await;
                    }
                    return Ok(Some(snap));
                }
                Ok(Some(_)) | Ok(None) => {}
                Err(e) => return Err(e),
            }
            if Instant::now() >= wait_deadline {
                let mut snap = self.cancel_mode_switch_session(session_id, "TIMEOUT").await?;
                if snap.is_none() {
                    snap = self
                        .poll_mode_switch_session(session_id)
                        .await?
                        .filter(|s| !s.running);
                }
                if !snap.as_ref().is_some_and(|s| s.progress.is_ambiguous())
                    && !self.internal_fault_lane.is_fenced()
                {
                    let _ = self.clear_mode_switch_session(session_id).await;
                }
                return Ok(snap);
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    /// Runs a `ModeSwitch` session to completion on the active owner (`OWNER_LEGACY` or
    /// `OWNER_EXECUTOR`), polling via [`Self::read_services`] until terminal and clearing the slot
    /// before returning the final authoritative snapshot.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn run_mode_switch_session(
        &self,
        req: crate::rt_core::requests::config::ModeSwitchStartRequest,
        poll_timeout: Duration,
    ) -> Result<crate::motor_mode::ModeSwitchSnapshot, crate::rt_core::requests::config::ConfigError>
    {
        use crate::rt_core::requests::config::ConfigError;
        // R2 Major 8: validate deadline arithmetic BEFORE admitting the session!
        let Some(wait_deadline) = Instant::now().checked_add(poll_timeout) else {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "mode-switch poll timeout overflows monotonic clock",
            });
        };
        let session_id = self.start_mode_switch_session(req).await?;
        self.await_mode_switch_session(session_id, wait_deadline)
            .await?
            .ok_or(ConfigError::OwnerStopped)
    }

    /// Reference adapter (PR0C): runs causal J7 gripper mode confirmation (`PositionVelocity` or
    /// `PositionForce`) as a PR0B `SessionKind::ModeSwitch` session on either owner.
    #[cfg(not(target_arch = "wasm32"))]
    pub async fn confirm_gripper_mode_session(
        &self,
        joint: usize,
        mode: crate::motor_mode::ControlMode,
        admission: AdmissionStamp,
        operator_grant: crate::auth::OperatorGrant,
        timeout: Duration,
    ) -> Result<crate::motor_mode::ModeSwitchSnapshot, crate::rt_core::requests::config::ConfigError>
    {
        use crate::rt_core::requests::config::ConfigError;
        let plan = crate::motor_mode::ModeSwitchPlan::confirm_gripper(joint, mode).map_err(|_| {
            ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "invalid gripper joint index or control mode",
            }
        })?;
        // R3 Minor 1: validate against the configured gripper joint via `read_services`.
        let svc = self
            .read_services(crate::rt_core::publish::ReadRequirements::identity())
            .await
            .map_err(|_| ConfigError::OwnerStopped)?;
        if svc.identity.dof <= svc.identity.arm_dof || joint != svc.identity.arm_dof {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "joint is not the configured gripper axis",
            });
        }
        // R2 Major 8: checked deadline & grace arithmetic BEFORE admission!
        let now = Instant::now();
        let Some(deadline) = now.checked_add(timeout) else {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "gripper mode confirmation timeout overflows clock",
            });
        };
        let Some(poll_timeout) = timeout.checked_add(Duration::from_millis(250)) else {
            return Err(ConfigError::CalibPrecondition {
                status: 400,
                code: "INVALID_COMMAND",
                message: "gripper mode confirmation poll timeout overflows clock",
            });
        };
        let req = crate::rt_core::requests::config::ModeSwitchStartRequest {
            plan,
            admission,
            operator: crate::auth::OperatorRequirement::Required(operator_grant),
            deadline,
            client: crate::rt_core::broker::ClientId(admission.connection_id),
        };
        self.run_mode_switch_session(req, poll_timeout).await
    }
}
