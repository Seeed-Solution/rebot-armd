//! Damiao mode switching, restore, and verified hold-frame builders.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::protocol::damiao;
use crate::transport::{CanFrame, CanTransport};

impl<C: CanTransport> DamiaoArmDriver<C> {
    /// Shared mode-switch implementation used by both off-tick [`ArmDriver::set_joint_modes`]
    /// (which opens a fresh TX pass) and in-tick [`ArmDriver::set_joint_modes_in_tick`] (which
    /// shares the active release's TX pass).
    pub(in crate::driver::can_servo) fn set_joint_modes_impl(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        let start_stamp = self.current_stop_stamp();
        let n = self.motor_ids.len();
        if modes.len() != n || hold.len() != n {
            return Err(ArmDriverError::JointCountMismatch { expected: n, actual: modes.len().min(hold.len()) });
        }
        if !self.supports_mit_mode_switch() {
            return Err(ArmDriverError::Protocol("joint mode switching needs the POS_VEL base mode".into()));
        }
        let target: Vec<DamiaoControlMode> = modes.iter().enumerate().map(|(i, m)| match m {
            JointMode::Position => self.base_mode(i),
            JointMode::Mit => DamiaoControlMode::Mit,
        }).collect();
        let changing: Vec<usize> = (0..n).filter(|&i| self.joint_modes[i] != Some(target[i])).collect();
        if changing.is_empty() { return Ok(()); }
        let is_base_restore = changing.iter().all(|&i| target[i] == self.base_mode(i));
        // CTRL_MODE writes (forward and rollback alike) are refused while a stop/E-stop is pending OR latched.
        self.strict_stop_check(Some(start_stamp))?;
        let forward_policy = if !is_base_restore {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let effective_hold = hold.to_vec();
        self.validate_pos_force_holds(&changing, &target, &effective_hold, false)?;
        let to_mit: Vec<usize> = changing.iter().copied()
            .filter(|&i| target[i] == DamiaoControlMode::Mit && !self.mit_limits_verified[i]).collect();
        if !to_mit.is_empty() { self.verify_mit_limits(&to_mit, start_stamp)?; }
        let previous = self.joint_modes.clone();
        let failed = self.write_modes(&changing, &target, &effective_hold, false, forward_policy)?;
        if failed.is_empty() {
            tracing::info!("Damiao control mode switched for joints {:?}", changing.iter().map(|i| i + 1).collect::<Vec<_>>());
            return Ok(());
        }
        // Not confirmed: switch every changed joint back to its previous mode, holding it
        // with both frame kinds meanwhile so it holds whichever mode it is actually in.
        for &i in &changing { self.joint_modes[i] = None; }
        let back: Vec<DamiaoControlMode> = (0..n).map(|i| previous[i].unwrap_or(self.base_mode(i))).collect();
        let back_is_base = changing.iter().all(|&i| back[i] == self.base_mode(i));
        // Rollback CTRL_MODE writes are also ModeWrite: refused while pending, latched, or after a stop.
        self.strict_stop_check(Some(start_stamp))?;
        let rollback_policy = if !back_is_base {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let rollback_hold = hold.to_vec();
        let unrecovered = self.write_modes(&changing, &back, &rollback_hold, true, rollback_policy)?;
        Err(ArmDriverError::HardwareFault(format!(
            "Control-mode switch not confirmed for joints {:?}; switched back{}",
            failed.iter().map(|i| i + 1).collect::<Vec<_>>(),
            if unrecovered.is_empty() { String::new() } else {
                format!(", joints {:?} still unconfirmed", unrecovered.iter().map(|i| i + 1).collect::<Vec<_>>())
            })))
    }

    #[inline]
    pub(in crate::driver::can_servo) fn is_axis_mode_uncertain(&self, i: usize) -> bool {
        i < 32 && (self.mode_write_uncertain_mask & (1u32 << i)) != 0
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[inline]
    pub(in crate::driver::can_servo) fn stage_damiao_mode_verified(&mut self, idx: usize, mode: DamiaoControlMode) {
        if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
            let now = self.now();
            self.shadow_staging
                .push(crate::torque::shadow::ShadowDriverObs::ModeVerified {
                    channel: idx as u8,
                    mode: damiao_mode_to_shadow_control_mode(mode),
                    now,
                    provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                });
        }
    }

    /// Confirmed control mode of each joint (None = unconfirmed).
    pub fn joint_control_modes(&self) -> &[Option<DamiaoControlMode>] {
        &self.joint_modes
    }

    pub(in crate::driver::can_servo) fn mode_value(mode: DamiaoControlMode) -> u32 {
        match mode {
            DamiaoControlMode::Mit => damiao::CTRL_MODE_MIT,
            DamiaoControlMode::PosVel => damiao::CTRL_MODE_POS_VEL,
            DamiaoControlMode::PosForce => damiao::CTRL_MODE_POS_FORCE,
        }
    }

    pub(in crate::driver::can_servo) fn mode_from_u32(val: u32) -> Option<DamiaoControlMode> {
        match val {
            damiao::CTRL_MODE_MIT => Some(DamiaoControlMode::Mit),
            damiao::CTRL_MODE_POS_VEL => Some(DamiaoControlMode::PosVel),
            damiao::CTRL_MODE_POS_FORCE => Some(DamiaoControlMode::PosForce),
            _ => None,
        }
    }

    /// Stationary hold target at joint `i`'s fresh (`<= 100 ms`) measured position for
    /// base-mode rollback while a stop or E-stop is active; returns `None` without fresh
    /// feedback so callers fail closed without sending a hold.
    pub(in crate::driver::can_servo) fn fresh_rollback_hold_target_at(&self, i: usize, now: Instant) -> Option<JointTarget> {
        let pos = self.fresh_measured_position_at(i, POS_FORCE_FEEDBACK_MAX_AGE, now)?;
        if let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) {
            Some(JointTarget {
                position: pos,
                velocity: Some(pf.close_speed_rad_s),
                ..JointTarget::position(pos)
            })
        } else {
            Some(JointTarget::position(pos))
        }
    }

    pub(in crate::driver::can_servo) fn fresh_rollback_hold_target(&self, i: usize) -> Option<JointTarget> {
        self.fresh_rollback_hold_target_at(i, self.now())
    }

    pub(in crate::driver::can_servo) fn rollback_requires_fresh(&self, policy: ModeWritePolicy) -> bool {
        match policy {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold } => {
                force_fresh_hold
                    || self.is_stop_pending()
                    || self.is_stop_latched()
                    || self.current_stop_stamp() != start_stamp
            }
            ModeWritePolicy::EnableRestore | ModeWritePolicy::Strict(_) => false,
        }
    }

    pub(in crate::driver::can_servo) fn policy_check(&mut self, policy: ModeWritePolicy) -> Result<(), ArmDriverError> {
        match policy {
            ModeWritePolicy::EnableRestore => self.enable_estop_inhibited(),
            ModeWritePolicy::Strict(stamp) => self.strict_stop_check(Some(stamp)),
            ModeWritePolicy::BaseRollback { start_stamp, .. } => self.pending_stop_check(Some(start_stamp)),
        }
    }

    pub(in crate::driver::can_servo) fn mode_hold_slot_count(mode: DamiaoControlMode, dual_hold: bool) -> usize {
        match mode {
            DamiaoControlMode::PosForce => 1,
            _ if dual_hold => 2,
            DamiaoControlMode::Mit | DamiaoControlMode::PosVel => 1,
        }
    }

    pub(in crate::driver::can_servo) fn build_mode_hold_frame(
        &self,
        i: usize,
        mode: DamiaoControlMode,
        hold: &JointTarget,
        dual_hold: bool,
        slot: usize,
    ) -> Option<CanFrame> {
        self.mode_frames(i, mode, hold, dual_hold)
            .and_then(|fs| fs.get(slot).copied().flatten())
    }

    /// Hold frames of non-PosForce mode slots (`PosVel` / `Mit`) on the stack without allocation.
    /// `PosForce` hold slots are validated here and emitted exclusively through
    /// [`Self::emit_pos_force_frame`].
    pub(in crate::driver::can_servo) fn mode_frames(&self, i: usize, mode: DamiaoControlMode, hold: &JointTarget, dual: bool) -> Option<[Option<CanFrame>; 2]> {
        match mode {
            DamiaoControlMode::PosForce => {
                self.pos_force_target_valid(i, hold).then_some([None, None])
            }
            _ if dual => {
                let pos = if self.base_mode(i) == DamiaoControlMode::PosForce {
                    if !self.pos_force_target_valid(i, hold) {
                        return None;
                    }
                    None
                } else {
                    Some(self.pos_vel_frame(i, hold.position))
                };
                Some([pos, Some(self.mit_frame(i, hold))])
            }
            DamiaoControlMode::Mit => Some([Some(self.mit_frame(i, hold)), None]),
            DamiaoControlMode::PosVel => Some([Some(self.pos_vel_frame(i, hold.position)), None]),
        }
    }

    /// Write CTRL_MODE for `joints` (each followed at once by `hold` in the new mode)
    /// and wait for the write acknowledgements. Returns the joints that were not confirmed,
    /// or `Err(EstopInhibited)` when stopped by `policy`.
    ///
    /// With `strict_mode_handover` (joint tracking enabled) a joint is confirmed only when
    /// the mode write, every hold frame transmit, the bus flush and the matching register
    /// reply all succeed. Without it the matching reply alone confirms the joint, as before
    /// joint tracking (e4f72b6): hold transmit and flush errors are ignored.
    pub(in crate::driver::can_servo) fn write_modes(
        &mut self,
        joints: &[usize],
        modes: &[DamiaoControlMode],
        hold: &[JointTarget],
        dual_hold: bool,
        policy: ModeWritePolicy,
    ) -> Result<Vec<usize>, ArmDriverError> {
        let w = match self.write_modes_send(joints, modes, hold, dual_hold, policy) {
            Ok(w) => w,
            Err(e) if is_estop_inhibited(&e) => return Err(e),
            Err(_) => return Ok(joints.to_vec()),
        };
        if w.sent_len == 0 {
            let (failed, len) = self.write_modes_finish(w);
            return Ok(failed[..len].iter().map(|&j| usize::from(j)).collect());
        }
        let mut changing_mask = 0u32;
        for &raw_i in &w.sent[..usize::from(w.sent_len)] {
            let i = usize::from(raw_i);
            if i < MAX_MODE_JOINTS {
                changing_mask |= 1u32 << i;
            }
        }
        loop {
            let _ = self.receive_feedback();
            if self.motor_energized_mask != 0 {
                let _ = self.service_energized_watchdog_holds(
                    WATCHDOG_HOLD_PROACTIVE_INTERVAL,
                    None,
                    changing_mask,
                    false,
                );
                // Keep in-flight changing joints refreshed in their target mode so delayed ACKs
                // never leave a motor unserved or falsely credited with a stale-mode frame.
                let now = self.now();
                let mut sent_changing = false;
                for &raw_i in &w.sent[..usize::from(w.sent_len)] {
                    let i = usize::from(raw_i);
                    if i >= MAX_MODE_JOINTS
                        || (self.motor_energized_mask & (1u32 << i)) == 0
                        || self.active_pos_force_joint() == Some(i)
                    {
                        continue;
                    }
                    let age = self.motor_last_ctrl_tx[i]
                        .map_or(Duration::MAX, |t| now.saturating_duration_since(t));
                    if age < WATCHDOG_HOLD_PROACTIVE_INTERVAL {
                        continue;
                    }
                    let target_mode = w.modes[i];
                    let target = &hold[i];
                    let res = self.send_plain(TxKind::MeasuredHold(i), None, None, false, |d| {
                        match target_mode {
                            DamiaoControlMode::PosVel => Some(d.pos_vel_frame(i, target.position)),
                            DamiaoControlMode::Mit => Some(d.mit_frame(i, target)),
                            DamiaoControlMode::PosForce => None,
                        }
                    });
                    if res.is_ok() {
                        sent_changing = true;
                    }
                }
                if sent_changing {
                    let _ = self.flush_can_and_commit();
                }
            }
            if self.mode_write_replied(&w) || w.deadline.passed(self.now()) {
                let (failed, len) = self.write_modes_finish(w);
                return Ok(failed[..len].iter().map(|&j| usize::from(j)).collect());
            }
            self.pause(Duration::from_micros(500));
        }
    }

    /// First half of `write_modes`: the mode writes and holds; the replies are awaited until the
    /// returned deadline (REGISTER_REPLY_TIMEOUT after the flush).
    /// Enforces `policy` via [`Self::stop_gate_check`]: forward switches and enable restores
    /// refuse every frame while a stop/E-stop is pending or latched; base-mode rollbacks while
    /// stopped only transmit with fresh (`<= 100 ms`) measured feedback, overriding holds to
    /// the fresh measured position.
    pub(in crate::driver::can_servo) fn write_modes_send(
        &mut self,
        joints: &[usize],
        modes: &[DamiaoControlMode],
        hold: &[JointTarget],
        dual_hold: bool,
        policy: ModeWritePolicy,
    ) -> Result<ModeWrite, ArmDriverError> {
        fn restoring_pos_force<C: CanTransport>(d: &DamiaoArmDriver<C>, _i: usize, mode: DamiaoControlMode) -> bool {
            mode == DamiaoControlMode::PosForce && !d.is_enabled
        }
        if modes.len() > MAX_MODE_JOINTS || joints.len() > MAX_MODE_JOINTS {
            return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
        }
        self.policy_check(policy)?;
        let strict = self.strict_mode_handover;
        self.register_replies.clear();
        self.register_reply_log.clear();
        let mut mode_arr = [DamiaoControlMode::PosVel; MAX_MODE_JOINTS];
        mode_arr[..modes.len()].copy_from_slice(modes);

        // Torque-off enable restore defers any TX-budget-limited CTRL_MODE or hold frames inline
        // in `ModeWrite` so `poll_restore` retries them across ticks until `deadline`.
        if matches!(policy, ModeWritePolicy::EnableRestore) {
            let mut deferred = [0u8; MAX_MODE_JOINTS];
            for (k, &j) in joints.iter().enumerate() {
                deferred[k] = j as u8;
            }
            let mut hold_pos = [0.0f32; MAX_MODE_JOINTS];
            for (k, t) in hold.iter().enumerate().take(MAX_MODE_JOINTS) {
                hold_pos[k] = t.position;
            }
            let mut w = ModeWrite {
                modes: mode_arr,
                sent: [0u8; MAX_MODE_JOINTS],
                sent_len: 0,
                unsent: [0u8; MAX_MODE_JOINTS],
                unsent_len: 0,
                hold_failed: [0u8; MAX_MODE_JOINTS],
                hold_failed_len: 0,
                deferred,
                deferred_stage: [0u8; MAX_MODE_JOINTS],
                deferred_len: joints.len() as u8,
                hold_pos,
                dual_hold,
                flush_ok: true,
                deadline: Deadline::after(self.now(), REGISTER_REPLY_TIMEOUT),
            };
            self.drain_restore_deferred(&mut w)?;
            return Ok(w);
        }

        let expected_stamp = match policy {
            ModeWritePolicy::EnableRestore => self.enable_stop_stamp,
            ModeWritePolicy::Strict(stamp) | ModeWritePolicy::BaseRollback { start_stamp: stamp, .. } => Some(stamp),
        };
        let is_rollback = matches!(policy, ModeWritePolicy::BaseRollback { .. });
        let mut unsent = [0u8; MAX_MODE_JOINTS];
        let mut unsent_len = 0usize;
        let mut hold_failed = [0u8; MAX_MODE_JOINTS];
        let mut hold_failed_len = 0usize;
        let mut any_tx_sent = false;
        for &i in joints {
            self.policy_check(policy)?;
            // Validate hold availability before writing CTRL_MODE so an enabled position-force
            // joint never switches mode without a valid capped hold frame, and a base-mode
            // rollback while stopped never writes CTRL_MODE without fresh measured feedback.
            let restoring_pf = restoring_pos_force(self, i, modes[i]);
            let need_fresh_rollback = !restoring_pf && self.rollback_requires_fresh(policy);
            let hold_slots = if restoring_pf {
                if self.pos_force_write_refused(&[i]).is_some() {
                    unsent[unsent_len] = i as u8;
                    unsent_len += 1;
                    continue;
                }
                0
            } else {
                let effective_hold = if need_fresh_rollback {
                    let Some(fresh_target) = self.fresh_rollback_hold_target(i) else {
                        unsent[unsent_len] = i as u8;
                        unsent_len += 1;
                        continue;
                    };
                    fresh_target
                } else {
                    hold[i].clone()
                };
                if self.mode_frames(i, modes[i], &effective_hold, dual_hold).is_none() {
                    unsent[unsent_len] = i as u8;
                    unsent_len += 1;
                    continue;
                }
                Self::mode_hold_slot_count(modes[i], dual_hold)
            };
            let id = self.motor_ids[i];
            let is_pf_joint = self.base_mode(i) == DamiaoControlMode::PosForce;
            let needs_pf_hold = !restoring_pf
                && (modes[i] == DamiaoControlMode::PosForce
                    || (dual_hold && is_pf_joint));
            let needed = 1 + hold_slots;
            let _ = self.ensure_tx_pass();
            if !self.can_start_tick_tx_n(needed) {
                unsent[unsent_len] = i as u8;
                unsent_len += 1;
                continue;
            }
            let ctrl_kind = TxKind::ModeWrite;
            let hold_kind = if is_rollback { TxKind::RollbackHold(i) } else { TxKind::Hold };
            match self.send_with_retry(ctrl_kind, expected_stamp, None, false, |d| {
                if restoring_pf {
                    if d.pos_force_write_refused(&[i]).is_some() {
                        return None;
                    }
                } else {
                    let rb_fresh = d.rollback_requires_fresh(policy);
                    if rb_fresh && d.fresh_rollback_hold_target(i).is_none() {
                        return None;
                    }
                    if needs_pf_hold {
                        let pf_hold = if rb_fresh {
                            d.fresh_rollback_hold_target(i)?
                        } else {
                            hold[i].clone()
                        };
                        if !d.pos_force_target_valid(i, &pf_hold) {
                            return None;
                        }
                    }
                }
                Some(damiao::build_set_control_mode_frame(id, Self::mode_value(modes[i])))
            }) {
                Ok(_) => {
                    any_tx_sent = true;
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    if any_tx_sent {
                        for &j in joints { self.joint_modes[j] = None; }
                    }
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(_) => {
                    unsent[unsent_len] = i as u8;
                    unsent_len += 1;
                    continue;
                }
            }
            // The hold frame directly follows the mode write on the bus, rebuilt fresh at send time
            // inside `send` (never resending a cached pre-stop hold frame across attempts or retries).
            let mut all_holds_ok = true;
            for slot in 0..hold_slots {
                let hold_res = if self.is_pos_force_mode_hold_slot(i, modes[i], dual_hold, slot) {
                    let intent = if self.rollback_requires_fresh(policy) {
                        PosForceIntentKind::Hold
                    } else {
                        PosForceIntentKind::Target {
                            position: hold[i].position,
                            velocity: hold[i].velocity,
                            current_raw: None,
                        }
                    };
                    pos_force_emitter::emit_mode_switch_hold(self, i, is_rollback, expected_stamp, intent, false)
                } else {
                    self.send_with_retry(hold_kind, expected_stamp, None, false, |d| {
                        let eff_hold = if d.rollback_requires_fresh(policy) {
                            d.fresh_rollback_hold_target(i)?
                        } else {
                            hold[i].clone()
                        };
                        d.build_mode_hold_frame(i, modes[i], &eff_hold, dual_hold, slot)
                    })
                };
                match hold_res {
                    Ok(sent_frame) => {
                        let mit_params = if self.rollback_requires_fresh(policy) {
                            (self.default_kp, self.default_kd, 0.0)
                        } else {
                            self.resolved_mit_params(&hold[i])
                        };
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                    }
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(_) => {
                        all_holds_ok = false;
                    }
                }
            }
            if !all_holds_ok
                && (strict
                    || self.base_mode(i) == DamiaoControlMode::PosForce
                    || self.rollback_requires_fresh(policy))
            {
                hold_failed[hold_failed_len] = i as u8;
                hold_failed_len += 1;
                if self.base_mode(i) == DamiaoControlMode::PosForce || self.rollback_requires_fresh(policy) {
                    self.joint_modes[i] = None;
                    self.last_target_frames[i] = None;
                    self.last_mit_params[i] = None;
                }
            }
        }
        self.policy_check(policy)?;
        let flush_ok = if any_tx_sent {
            self.flush_can_and_commit().is_ok() || !strict
        } else {
            true
        };
        let mut sent = [0u8; MAX_MODE_JOINTS];
        let mut sent_len = 0usize;
        for &i in joints {
            let idx = i as u8;
            if !unsent[..unsent_len].contains(&idx) {
                sent[sent_len] = idx;
                sent_len += 1;
            }
        }
        Ok(ModeWrite {
            modes: mode_arr,
            sent,
            sent_len: sent_len as u8,
            unsent,
            unsent_len: unsent_len as u8,
            hold_failed,
            hold_failed_len: hold_failed_len as u8,
            deferred: [0u8; MAX_MODE_JOINTS],
            deferred_stage: [0u8; MAX_MODE_JOINTS],
            deferred_len: 0,
            hold_pos: [0.0; MAX_MODE_JOINTS],
            dual_hold,
            flush_ok,
            deadline: Deadline::after(self.now(), REGISTER_REPLY_TIMEOUT),
        })
    }

    pub(in crate::driver::can_servo) fn push_mode_joint_unique(buf: &mut [u8; MAX_MODE_JOINTS], len: &mut u8, val: u8) {
        let n = usize::from(*len);
        if !buf[..n].contains(&val) && n < MAX_MODE_JOINTS {
            buf[n] = val;
            *len = (n + 1) as u8;
        }
    }

    pub(in crate::driver::can_servo) fn pop_deferred_front(w: &mut ModeWrite) {
        let n = usize::from(w.deferred_len);
        if n == 0 { return; }
        for k in 1..n {
            w.deferred[k - 1] = w.deferred[k];
            w.deferred_stage[k - 1] = w.deferred_stage[k];
        }
        w.deferred_len = (n - 1) as u8;
    }

    /// Transmit as many queued torque-off enable-restore frames (`CTRL_MODE` writes and holds)
    /// as `w.deadline` and the active per-tick TX budget allow, routing every attempt and retry
    /// through [`Self::send`] with `Some(w.deadline)` and deferring remaining frames inline in
    /// `w` for subsequent `poll_restore` ticks (zero heap allocation).
    pub(in crate::driver) fn drain_restore_deferred(&mut self, w: &mut ModeWrite) -> Result<(), ArmDriverError> {
        let strict = self.strict_mode_handover;
        let expected_stamp = self.enable_stop_stamp;
        let deadline = w.deadline;
        let mut sent_any = false;
        while w.deferred_len > 0 {
            let raw_i = w.deferred[0];
            let i = usize::from(raw_i);
            let stage = w.deferred_stage[0];
            let mode = w.modes[i];
            let restoring_pf = mode == DamiaoControlMode::PosForce && !self.is_enabled;
            let id = self.motor_ids[i];
            if stage == 0 {
                match self.send_with_retry(TxKind::Handshake, expected_stamp, Some(deadline), false, |d| {
                    if restoring_pf {
                        if d.pos_force_write_refused(&[i]).is_some() {
                            return None;
                        }
                    } else {
                        let hold = JointTarget::position(w.hold_pos[i]);
                        d.mode_frames(i, mode, &hold, w.dual_hold)?;
                    }
                    Some(damiao::build_set_control_mode_frame(id, Self::mode_value(mode)))
                }) {
                    Ok(_) => {
                        sent_any = true;
                        Self::push_mode_joint_unique(&mut w.sent, &mut w.sent_len, raw_i);
                        if restoring_pf {
                            Self::pop_deferred_front(w);
                        } else {
                            w.deferred_stage[0] = 1;
                        }
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.abort_enable_for_estop();
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                    Err(_) => {
                        Self::push_mode_joint_unique(&mut w.unsent, &mut w.unsent_len, raw_i);
                        Self::pop_deferred_front(w);
                    }
                }
            } else {
                let slot = usize::from(stage - 1);
                if slot >= Self::mode_hold_slot_count(mode, w.dual_hold) {
                    Self::pop_deferred_front(w);
                    continue;
                }
                match self.send_with_retry(TxKind::Handshake, expected_stamp, Some(deadline), false, |d| {
                    let hold = JointTarget::position(w.hold_pos[i]);
                    d.build_mode_hold_frame(i, mode, &hold, w.dual_hold, slot)
                }) {
                    Ok(sent_frame) => {
                        sent_any = true;
                        let mit_params = (self.default_kp, self.default_kd, 0.0);
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                        if stage == 1 && w.dual_hold && mode != DamiaoControlMode::PosForce {
                            w.deferred_stage[0] = 2;
                        } else {
                            Self::pop_deferred_front(w);
                        }
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.abort_enable_for_estop();
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                    Err(_) => {
                        if strict {
                            Self::push_mode_joint_unique(&mut w.hold_failed, &mut w.hold_failed_len, raw_i);
                        }
                        Self::pop_deferred_front(w);
                    }
                }
            }
        }
        if sent_any {
            self.enable_estop_inhibited()?;
            w.flush_ok &= self.flush_can_and_commit().is_ok() || !strict;
        }
        Ok(())
    }

    pub(in crate::driver::can_servo) fn mode_write_reply(&self, i: usize) -> Option<damiao::RegisterReply> {
        let id = self.motor_ids[i];
        if self.base_mode(i) == DamiaoControlMode::PosForce {
            let expected_id = self.expected_reply_id(i);
            self.register_reply_log.iter().rev()
                .find(|(r, fid, ..)| *fid == expected_id && r.motor_id == id && r.write && r.rid == damiao::RID_CTRL_MODE)
                .map(|(r, ..)| *r)
        } else {
            self.register_replies.iter().rev().copied().find(|r| r.motor_id == id && r.write && r.rid == damiao::RID_CTRL_MODE)
        }
    }

    /// Every sent mode write has a register reply.
    pub(in crate::driver::can_servo) fn mode_write_replied(&self, w: &ModeWrite) -> bool {
        w.sent[..usize::from(w.sent_len)].iter().all(|&i| self.mode_write_reply(usize::from(i)).is_some())
    }

    /// Second half of `write_modes`: confirm the joints whose reply carries the written mode.
    /// Returns `(failed_joints, len)` inline without heap allocation.
    pub(in crate::driver::can_servo) fn write_modes_finish(&mut self, w: ModeWrite) -> ([u8; MAX_MODE_JOINTS], usize) {
        let strict = self.strict_mode_handover;
        let mut failed = w.unsent;
        let mut failed_len = usize::from(w.unsent_len);
        let hold_failed = &w.hold_failed[..usize::from(w.hold_failed_len)];
        for &raw_i in &w.sent[..usize::from(w.sent_len)] {
            let i = usize::from(raw_i);
            if w.flush_ok
                && !hold_failed.contains(&raw_i)
                && self.mode_write_reply(i).is_some_and(|r| r.as_u32() == Self::mode_value(w.modes[i]))
            {
                self.joint_modes[i] = Some(w.modes[i]);
                self.verified_modes[i] = Some(w.modes[i]);
                if i < 32 {
                    self.mode_write_uncertain_mask &= !(1u32 << i);
                }
                #[cfg(not(target_arch = "wasm32"))]
                self.stage_damiao_mode_verified(i, w.modes[i]);
            } else {
                self.verified_modes[i] = None;
                if strict || self.base_mode(i) == DamiaoControlMode::PosForce {
                    self.joint_modes[i] = None;
                }
                if failed_len < MAX_MODE_JOINTS {
                    failed[failed_len] = raw_i;
                    failed_len += 1;
                }
            }
        }
        (failed, failed_len)
    }

    /// MIT-path mode write: the CTRL_MODE writes and holds of `joints`, then a matching
    /// acknowledgement and fresh feedback per joint, each received after the writes went out (enqueue
    /// bound after the send mark) and the feedback after the acknowledgement in RX order; a
    /// read-only status poll asks once for missing feedback. Every hold transmit and the flush must
    /// succeed. Returns the joints not confirmed (their mode becomes unknown) and why.
    pub(in crate::driver::can_servo) fn write_modes_confirmed(
        &mut self,
        joints: &[usize],
        modes: &[DamiaoControlMode],
        hold: &[JointTarget],
        dual_hold: bool,
        policy: ModeWritePolicy,
    ) -> Result<(Vec<usize>, &'static str), ArmDriverError> {
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        let total_joints = self.motor_ids.len().min(u8::MAX as usize) as u8;
        let expected_stamp = match policy {
            ModeWritePolicy::EnableRestore => self.enable_stop_stamp,
            ModeWritePolicy::Strict(stamp) | ModeWritePolicy::BaseRollback { start_stamp: stamp, .. } => Some(stamp),
        };
        let is_rollback = matches!(policy, ModeWritePolicy::BaseRollback { .. });
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
            }
            for &i in joints { self.joint_modes[i] = None; }
            return Err(e);
        }
        let Ok(sent) = self.rx_boundary() else {
            for &i in joints { self.joint_modes[i] = None; }
            return Ok((joints.to_vec(), "receive boundary (queue not drained)"));
        };
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopBeforeBatch { total_joints });
            }
            for &i in joints { self.joint_modes[i] = None; }
            return Err(e);
        }
        self.register_replies.clear();
        self.register_reply_log.clear();
        let mut failed: Vec<usize> = Vec::new();
        let mut hold_started: Vec<(usize, Instant)> = Vec::new();
        let mut any_tx_sent = false;
        for (completed, &i) in joints.iter().enumerate() {
            if let Err(e) = self.policy_check(policy) {
                if fenced {
                    self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                        completed_joints: completed as u8,
                        total_joints,
                    });
                }
                for &j in joints { self.joint_modes[j] = None; }
                return Err(e);
            }
            let effective_hold = if self.rollback_requires_fresh(policy) {
                let Some(fresh_target) = self.fresh_rollback_hold_target(i) else {
                    failed.push(i);
                    continue;
                };
                fresh_target
            } else {
                hold[i].clone()
            };
            if self.mode_frames(i, modes[i], &effective_hold, dual_hold).is_none() {
                failed.push(i);
                continue;
            }
            let id = self.motor_ids[i];
            let is_pf_joint = self.base_mode(i) == DamiaoControlMode::PosForce;
            let needs_pf_hold = modes[i] == DamiaoControlMode::PosForce
                || (dual_hold && is_pf_joint);
            let hold_slots = Self::mode_hold_slot_count(modes[i], dual_hold);
            let needed = 1 + hold_slots;
            let _ = self.ensure_tx_pass();
            if !self.can_start_tick_tx_n(needed) {
                failed.push(i);
                continue;
            }
            let ctrl_kind = TxKind::ModeWrite;
            let hold_kind = if is_rollback { TxKind::RollbackHold(i) } else { TxKind::Hold };
            match self.send_with_retry(ctrl_kind, expected_stamp, None, fenced, |d| {
                let rb_fresh = d.rollback_requires_fresh(policy);
                if rb_fresh && d.fresh_rollback_hold_target(i).is_none() {
                    return None;
                }
                if needs_pf_hold {
                    let pf_hold = if rb_fresh {
                        d.fresh_rollback_hold_target(i)?
                    } else {
                        hold[i].clone()
                    };
                    if !d.pos_force_target_valid(i, &pf_hold) {
                        return None;
                    }
                }
                Some(damiao::build_set_control_mode_frame(id, Self::mode_value(modes[i])))
            }) {
                Ok(_) => {
                    any_tx_sent = true;
                }
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    if fenced {
                        self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                            completed_joints: completed as u8,
                            total_joints,
                        });
                    }
                    for &j in joints { self.joint_modes[j] = None; }
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(_) => {
                    failed.push(i);
                    continue;
                }
            }
            hold_started.push((i, self.now()));
            for slot in 0..hold_slots {
                let hold_res = if self.is_pos_force_mode_hold_slot(i, modes[i], dual_hold, slot) {
                    let intent = if self.rollback_requires_fresh(policy) {
                        PosForceIntentKind::Hold
                    } else {
                        PosForceIntentKind::Target {
                            position: hold[i].position,
                            velocity: hold[i].velocity,
                            current_raw: None,
                        }
                    };
                    pos_force_emitter::emit_mode_switch_hold(self, i, is_rollback, expected_stamp, intent, fenced)
                } else {
                    self.send_with_retry(hold_kind, expected_stamp, None, fenced, |d| {
                        let eff_hold = if d.rollback_requires_fresh(policy) {
                            d.fresh_rollback_hold_target(i)?
                        } else {
                            hold[i].clone()
                        };
                        d.build_mode_hold_frame(i, modes[i], &eff_hold, dual_hold, slot)
                    })
                };
                match hold_res {
                    Ok(sent_frame) => {
                        let mit_params = if self.rollback_requires_fresh(policy) {
                            (self.default_kp, self.default_kd, 0.0)
                        } else {
                            self.resolved_mit_params(&hold[i])
                        };
                        self.record_sent_target(i, sent_frame, Some(mit_params));
                    }
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        if fenced {
                            self.trip_fence(MitFenceTripReason::EstopDuringBatch {
                                completed_joints: completed as u8,
                                total_joints,
                            });
                        }
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(_) => {
                        if !failed.contains(&i) { failed.push(i); }
                        if self.base_mode(i) == DamiaoControlMode::PosForce || self.rollback_requires_fresh(policy) {
                            self.joint_modes[i] = None;
                            self.last_target_frames[i] = None;
                            self.last_mit_params[i] = None;
                        }
                    }
                }
            }
        }
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopBeforeFlush { total_joints });
            }
            for &j in joints { self.joint_modes[j] = None; }
            return Err(e);
        }
        let mut why = "mode write, hold or flush";
        if any_tx_sent && self.flush_can_and_commit().is_err() { failed = joints.to_vec(); }
        if let Err(e) = self.policy_check(policy) {
            if fenced {
                self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
            }
            for &j in joints { self.joint_modes[j] = None; }
            return Err(e);
        }
        if self.mit_fence.is_some() || self.mit_fence_trip.is_some() {
            // holds completing past the deadline trip it: the switch fails, nothing more is sent
            if self.fence_check().is_err() {
                for &i in joints { self.joint_modes[i] = None; }
                return Ok((joints.to_vec(), "deadline fence (holds completed past it)"));
            }
            if failed.is_empty() {
                // each switched joint's hold is its complete command, dated by its first frame
                for (i, t) in hold_started { self.mit_fence_last_cmd[i] = Some(t); }
            }
        }
        let pending: Vec<usize> = joints.iter().copied().filter(|i| !failed.contains(i)).collect();
        let expected: Vec<(usize, bool, u8)> = pending.iter().map(|&i| (i, true, damiao::RID_CTRL_MODE)).collect();
        if !expected.is_empty() {
            self.await_replies_after(&expected, sent);
        }
        let ack_seq: Vec<Option<u64>> = pending.iter().map(|&i| {
            let (id, want) = (self.motor_ids[i], Self::mode_value(modes[i]));
            let expected_id = self.expected_reply_id(i);
            let check_id = self.base_mode(i) == DamiaoControlMode::PosForce;
            self.register_reply_log.iter().rev()
                .find(|(r, fid, _, seq, _)| (!check_id || *fid == expected_id) && r.motor_id == id && r.write && r.rid == damiao::RID_CTRL_MODE && *seq > sent && r.as_u32() == want)
                .map(|(_, _, _, seq, _)| *seq)
        }).collect();
        if ack_seq.iter().any(Option::is_none) { why = "matching acknowledgement received after the mode write"; }
        let unconfirmed = |d: &Self| -> Vec<usize> {
            pending.iter().zip(&ack_seq)
                .filter(|(&i, ack)| !ack.is_some_and(|a| d.last_feedback_seq[i] > a))
                .map(|(&i, _)| i).collect()
        };
        let confirm_deadline = Deadline::after(self.now(), REGISTER_REPLY_TIMEOUT);
        let mut polled = false;
        // a joint without a matching acknowledgement fails at once (nothing to wait for), so the
        // rollback starts within the deadline
        let acks_complete = ack_seq.iter().all(Option::is_some);
        let mut changing_confirm_mask = 0u32;
        for &i in &pending {
            if i < MAX_MODE_JOINTS {
                changing_confirm_mask |= 1u32 << i;
            }
        }
        loop {
            let missing = unconfirmed(self);
            if missing.is_empty() || !acks_complete || confirm_deadline.passed(self.now()) { break; }
            if self.motor_energized_mask != 0 && self.mit_fence.is_none() {
                let _ = self.service_energized_watchdog_holds(
                    WATCHDOG_HOLD_PROACTIVE_INTERVAL,
                    Some(confirm_deadline),
                    changing_confirm_mask,
                    false,
                );
            }
            if !polled && ack_seq.iter().all(Option::is_some) {
                if let Err(e) = self.policy_check(policy) {
                    if fenced {
                        self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                    }
                    for &j in joints { self.joint_modes[j] = None; }
                    return Err(e);
                }
                let mut refresh_sent = false;
                for &i in &missing {
                    if !is_rollback && self.is_stop_latched() {
                        if fenced {
                            self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                        }
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    match self.send_with_retry(
                        TxKind::Refresh,
                        expected_stamp,
                        Some(confirm_deadline),
                        fenced,
                        |d| Some(damiao::build_refresh_frame(d.motor_ids[i])),
                    ) {
                        Ok(_) => {
                            refresh_sent = true;
                        }
                        Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                            if fenced {
                                self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                            }
                            for &j in joints { self.joint_modes[j] = None; }
                            return Err(ArmDriverError::EstopInhibited);
                        }
                        Err(_) => break,
                    }
                }
                if refresh_sent {
                    if let Err(e) = self.policy_check(policy) {
                        if fenced {
                            self.trip_fence(MitFenceTripReason::EstopAfterFlush { total_joints });
                        }
                        for &j in joints { self.joint_modes[j] = None; }
                        return Err(e);
                    }
                    let _ = self.flush_can_and_commit();
                }
                polled = true;
            }
            self.pause(Duration::from_micros(500));
            let _ = self.read_joint_states();
        }
        let missing = unconfirmed(self);
        if !missing.is_empty() && why == "mode write, hold or flush" { why = "fresh feedback after the mode acknowledgement"; }
        failed.extend(missing);
        for &i in joints {
            self.joint_modes[i] = if failed.contains(&i) { None } else { Some(modes[i]) };
            self.verified_modes[i] = self.joint_modes[i];
            #[cfg(not(target_arch = "wasm32"))]
            if let Some(m) = self.verified_modes[i] {
                if self.shadow_staging.is_enabled() && i < crate::torque::MAX_CHANNELS {
                    let now = self.now();
                    self.shadow_staging
                        .push(crate::torque::shadow::ShadowDriverObs::ModeVerified {
                            channel: i as u8,
                            mode: damiao_mode_to_shadow_control_mode(m),
                            now,
                            provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                        });
                }
            }
        }
        Ok((failed, why))
    }

    /// `set_joint_modes_confirmed`: the strict switch, then fresh feedback from every switched
    /// motor received after its mode acknowledgement (RX order, so samples queued before the
    /// switch never count; a read-only status poll asks for it once). Anything missing switches
    /// every changed joint back with dual holds; a rollback that is not confirmed leaves the
    /// joint uncertain (`joint_modes_uncertain`).
    pub(in crate::driver::can_servo) fn switch_joint_modes_confirmed(&mut self, modes: &[JointMode], hold: &[JointTarget]) -> Result<(), ArmDriverError> {
        self.discard_all_pending_tx();
        self.begin_tx_tick();
        let start_stamp = self.current_stop_stamp();
        let n = self.motor_ids.len();
        if modes.len() != n || hold.len() != n {
            return Err(ArmDriverError::JointCountMismatch { expected: n, actual: modes.len().min(hold.len()) });
        }
        if self.control_mode != DamiaoControlMode::PosVel {
            return Err(ArmDriverError::Protocol("joint mode switching needs the POS_VEL base mode".into()));
        }
        // armed fence (hand back): the deadline holds for the switch too; tripped: refused
        self.fence_check().map_err(|e| ArmDriverError::HardwareFault(format!("{e}; rearm first")))?;
        let target: Vec<DamiaoControlMode> = modes.iter().enumerate().map(|(i, m)| match m {
            JointMode::Position => self.base_mode(i),
            JointMode::Mit => DamiaoControlMode::Mit,
        }).collect();
        let changing: Vec<usize> = (0..n).filter(|&i| self.joint_modes[i] != Some(target[i])).collect();
        if changing.is_empty() { return Ok(()); }
        let fenced = self.mit_fence.is_some() || self.mit_fence_trip.is_some();
        let is_base_restore = changing.iter().all(|&i| target[i] == self.base_mode(i));
        // CTRL_MODE writes (forward and rollback alike) are refused while a stop/E-stop is pending OR latched.
        self.strict_stop_check(Some(start_stamp))?;
        let forward_policy = if !is_base_restore || fenced {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let effective_hold = hold.to_vec();
        self.validate_pos_force_holds(&changing, &target, &effective_hold, true)?;
        let to_mit: Vec<usize> = changing.iter().copied()
            .filter(|&i| target[i] == DamiaoControlMode::Mit && !self.mit_limits_verified[i]).collect();
        for &i in &to_mit {
            self.strict_stop_check(Some(start_stamp))?;
            // bounded reads: PMAX / VMAX / TMAX replies received after the request
            let v = self.read_registers_fresh(i, &[damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX], start_stamp, false)?;
            let got = [f32::from_le_bytes(v[0].0), f32::from_le_bytes(v[1].0), f32::from_le_bytes(v[2].0)];
            let want = [self.limits[i].p_max, self.limits[i].v_max, self.limits[i].t_max];
            if !Self::ranges_match(&got, &want) {
                return Err(ArmDriverError::HardwareFault(format!("Joint {} reports PMAX/VMAX/TMAX {got:?}; the driver maps MIT frames with {want:?}", i + 1)));
            }
            self.mit_limits_verified[i] = true;
        }
        let previous = self.joint_modes.clone();
        let (failed, why) = self.write_modes_confirmed(&changing, &target, &effective_hold, false, forward_policy)?;
        // the confirmation wait counts against the fence too: a trip there fails the switch
        let fence_ok = !(self.mit_fence.is_some() || self.mit_fence_trip.is_some()) || self.fence_check().is_ok();
        if failed.is_empty() && fence_ok {
            tracing::info!("Damiao control mode switched and confirmed for joints {:?}", changing.iter().map(|i| i + 1).collect::<Vec<_>>());
            return Ok(());
        }
        let failed = if failed.is_empty() { changing.clone() } else { failed };
        for &i in &changing { self.joint_modes[i] = None; }
        let back: Vec<DamiaoControlMode> = (0..n).map(|i| previous[i].unwrap_or(self.base_mode(i))).collect();
        let back_is_base = changing.iter().all(|&i| back[i] == self.base_mode(i));
        // Rollback CTRL_MODE writes are also ModeWrite: refused while pending, latched, or after a stop.
        self.strict_stop_check(Some(start_stamp))?;
        let rollback_policy = if !back_is_base || fenced {
            ModeWritePolicy::Strict(start_stamp)
        } else {
            ModeWritePolicy::BaseRollback { start_stamp, force_fresh_hold: false }
        };
        let rollback_hold = hold.to_vec();
        // the rollback is confirmed like the forward switch (acknowledgement + fresh feedback)
        let (unrecovered, _) = self.write_modes_confirmed(&changing, &back, &rollback_hold, true, rollback_policy)?;
        Err(ArmDriverError::HardwareFault(format!(
            "Control-mode switch not confirmed for joints {:?} (no {why}); switched back{}",
            failed.iter().map(|i| i + 1).collect::<Vec<_>>(),
            if unrecovered.is_empty() { String::new() } else {
                format!(", joints {:?} still unconfirmed", unrecovered.iter().map(|i| i + 1).collect::<Vec<_>>())
            })))
    }

    /// Verified hold mode for joint `i` (`None` when mode has not been verified by a causal
    /// `CTRL_MODE` readback or confirmed mode-register write after this daemon start, or was
    /// cleared by a causal mismatch / failed mode-changing write). A failed enable NEVER clears
    /// a causally verified hold mode flag (L2). Never assumes `PosVel` (E4).
    pub(in crate::driver::can_servo) fn verified_hold_mode(&self, i: usize) -> Option<DamiaoControlMode> {
        if i >= self.motor_ids.len() {
            return None;
        }
        if self.is_axis_mode_uncertain(i) {
            return None;
        }
        let verified = self.verified_modes.get(i).copied().flatten()?;
        let active = self.joint_modes.get(i).copied().flatten()?;
        if verified != active {
            return None;
        }
        if verified == DamiaoControlMode::Mit && !self.is_mit_hold_eligible(i) {
            return None;
        }
        Some(verified)
    }

    /// Build the single stationary hold frame matching non-PosForce joint `i`'s verified hold mode.
    /// Active `PosForce` joints are NEVER built here — they are emitted exclusively by the single
    /// J7 emitter (`pos_force_emitter`). Never emits dual frames for an unverified `None` joint
    /// and has zero enable or mode-switch side effect.
    pub(in crate::driver::can_servo) fn build_verified_hold_frame(&self, i: usize, target: &JointTarget) -> Option<CanFrame> {
        if !target.position.is_finite() {
            return None;
        }
        match self.verified_hold_mode(i)? {
            DamiaoControlMode::PosVel => Some(self.pos_vel_frame(i, target.position)),
            DamiaoControlMode::Mit => Some(self.mit_frame(i, target)),
            DamiaoControlMode::PosForce => None,
        }
    }

    /// True when joint `i` has a verified hold mode that accepts stationary hold frames without
    /// enable or mode-switch side effects. Every joint (arm or gripper) requires verified mode
    /// evidence.
    pub(in crate::driver::can_servo) fn joint_hold_mode_verified_inner(&self, i: usize) -> bool {
        self.verified_hold_mode(i).is_some()
    }

}
