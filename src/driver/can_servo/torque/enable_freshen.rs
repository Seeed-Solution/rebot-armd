//! Damiao pre-enable feedback freshening, range check, and 0xCC refresh helpers.

#[allow(unused_imports)]
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
#[allow(unused_imports)]
use std::sync::Arc;
#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::driver::can_servo::*;
#[allow(unused_imports)]
use crate::model::JointState;
#[allow(unused_imports)]
use crate::protocol::damiao::{self, DamiaoLimits};
#[allow(unused_imports)]
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};

impl<C: CanTransport> DamiaoArmDriver<C> {
    /// Start enable's initial bounded `0xCC` FRESHEN phase: polls every joint with read-only
    /// `0xCC` status frames (allowed for J7 under `NoOutputUntilFresh`) and waits for a fresh
    /// causal post-boundary reply from every joint before CTRL_MODE restore/readback, pre-enable
    /// hold, and `CMD_ENABLE`. Zero locks, zero heap allocations.
    pub(in crate::driver::can_servo) fn start_enable_freshen(
        &mut self,
        target_mask: u32,
        drained_now: bool,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        if self.pos_force.is_some() {
            self.pos_force_verified = false;
        }
        self.refresh_capture = None;
        let start = self.now();
        let mut dl_at = start + ENABLE_FRESHEN_TIMEOUT;
        if let Some(auth_dl) = self.enable_tx_deadline {
            dl_at = dl_at.min(auth_dl);
        }
        let deadline = Deadline { at: dl_at };
        let mut f = EnableFreshen {
            target_mask,
            sent_mask: 0,
            confirmed_mask: 0,
            started: start,
            base_rx_seq: self.rx_seq,
            tx_boundary_delta: [0; MAX_MODE_JOINTS],
            tx_offset_ns: [EnableFreshen::UNSET_NS; MAX_MODE_JOINTS],
            next_retry_at: None,
            deadline,
        };
        if drained_now && !deadline.passed(self.now()) && self.remaining_rx_frame_limit() > 0 {
            let _ = self.send_freshen_polls(&mut f, target_mask)?;
        }
        let wake = self.freshen_pending_wake(&f);
        self.torque_hs = Some(DamiaoTorque::Freshen(f));
        Ok(TorqueProgress::Pending { wake })
    }

    pub(in crate::driver::can_servo) fn freshen_pending_wake(&self, f: &EnableFreshen) -> Instant {
        let now = self.now();
        let unsent = f.target_mask & !f.sent_mask;
        if unsent != 0 && self.tx_release_deadline.is_none_or(|cap| now < cap) {
            now.min(f.deadline.at)
        } else {
            f.deadline.at
        }
    }

    /// Sticky post-boundary J7 unsafe veto (fault, motion, or state != 0) cannot be masked by
    /// a later benign frame in the same RX pass or swallowed by a replacement-poll drain.
    pub(in crate::driver::can_servo) fn check_freshen_j7_unsafe_capture(&mut self) -> Result<(), ArmDriverError> {
        let Some(pf_j) = self.pos_force.as_ref().map(|p| p.joint) else {
            return Ok(());
        };
        let bad = self
            .refresh_capture
            .as_ref()
            .filter(|c| c.joint == pf_j)
            .and_then(|c| c.unsafe_sample);
        if let Some(sample) = bad {
            self.refresh_capture = None;
            self.enable_failed();
            let still = sample.velocity.is_finite() && sample.velocity.abs() <= 0.05;
            let off = !self.is_enabled && sample.motor_status == 0;
            return Err(ArmDriverError::EnableFreshenUnsafeGripper {
                joint: pf_j,
                still,
                off,
                status: sample.motor_status,
            });
        }
        Ok(())
    }

    /// Establish a tight observed-empty RX boundary and transmit read-only `0xCC` status polls
    /// for joints in `send_mask` within the per-tick TX budget, stop gate, and enable authority.
    pub(in crate::driver::can_servo) fn send_freshen_polls(
        &mut self,
        f: &mut EnableFreshen,
        send_mask: u32,
    ) -> Result<bool, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        if send_mask == 0 || f.deadline.passed(self.now()) {
            return Ok(false);
        }
        let drained = self.drain_rx_for_boundary(f.deadline);
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        self.check_freshen_j7_unsafe_capture()?;
        if !drained {
            return Ok(false);
        }
        if f.deadline.passed(self.now()) {
            return Ok(false);
        }
        let pf_joint = self.pos_force.as_ref().map(|p| p.joint);
        let mut sent_any = false;
        for j in handshake::mask_joints(send_mask) {
            self.enable_estop_inhibited()?;
            self.check_enable_tx_authority()?;
            if f.deadline.passed(self.now()) {
                break;
            }
            let Some(&id) = self.motor_ids.get(j) else {
                continue;
            };
            match self.send_plain(
                TxKind::Handshake,
                self.enable_stop_stamp,
                Some(f.deadline),
                false,
                |_| Some(damiao::build_refresh_frame(id)),
            ) {
                Ok(_) => {
                    if (f.sent_mask & (1u32 << j)) == 0 {
                        let (boundary, tx_at) =
                            self.last_tx_mark.unwrap_or((self.rx_seq, self.now()));
                        f.record_tx(j, boundary, tx_at);
                        if pf_joint == Some(j) && j < MAX_MODE_JOINTS {
                            self.refresh_capture = Some(RefreshCapture {
                                joint: j,
                                boundary: f.tx_boundary(j),
                                tx_at: f.tx_at(j).unwrap_or(tx_at),
                                deadline: f.deadline,
                                allow_enabled_state: false,
                                first: None,
                                unsafe_sample: None,
                            });
                        }
                    }
                    sent_any = true;
                }
                Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                    self.abort_enable_for_estop();
                    return Err(ArmDriverError::EstopInhibited);
                }
                Err(r) => {
                    self.torque_hs = None;
                    self.enable_failed();
                    return Err(Self::refusal_to_driver_err(r));
                }
            }
        }
        if sent_any {
            self.enable_estop_inhibited()?;
            self.check_enable_tx_authority()?;
            if let Err(e) = self.flush_can_and_commit() {
                self.enable_failed();
                return Err(e.into());
            }
            if (f.target_mask & !f.sent_mask) == 0 {
                f.next_retry_at = Some(self.now() + ENABLE_FRESHEN_RETRY_INTERVAL);
            }
        }
        Ok(sent_any)
    }

    /// Advance enable's initial bounded `0xCC` FRESHEN phase by one poll step.
    /// Accepts completion ONLY after every target joint has delivered a causal post-boundary
    /// feedback reply observed through an empty RX queue and remains well within freshness bounds.
    pub(in crate::driver::can_servo) fn poll_freshen(&mut self, mut f: EnableFreshen) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        let expired_before_read = f.deadline.passed(self.now());
        if f.sent_mask != 0 || !expired_before_read {
            let _ = self.receive_feedback();
            self.enable_estop_inhibited()?;
            self.check_enable_tx_authority()?;
        }

        let pf_joint = self.pos_force.as_ref().map(|p| p.joint);
        self.check_freshen_j7_unsafe_capture()?;

        if self.last_read_drained {
            for j in handshake::mask_joints(f.target_mask & f.sent_mask) {
                let Some(tx_at) = f.tx_at(j) else {
                    continue;
                };
                let tx_boundary = f.tx_boundary(j);
                if pf_joint == Some(j) {
                    let cap_first = self
                        .refresh_capture
                        .as_ref()
                        .filter(|c| c.joint == j && c.boundary == tx_boundary)
                        .and_then(|c| c.first);
                    if let Some(sample) = cap_first {
                        let timely = sample.seq > tx_boundary
                            && sample.rx_at >= tx_at
                            && !f.deadline.passed(sample.rx_at)
                            && sample.position.is_finite()
                            && self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
                            && sample
                                .queue_bound
                                .is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
                        if timely {
                            let st = self.motor_status[j];
                            if damiao_fault_name(st).is_some() {
                                let motor_id = self.motor_ids[j];
                                self.refresh_capture = None;
                                self.enable_failed();
                                return Err(ArmDriverError::MotorHardwareFault {
                                    joint: j,
                                    motor_id,
                                    status: st,
                                });
                            }
                            let still = self.cached_states[j].velocity.is_finite()
                                && self.cached_states[j].velocity.abs() <= 0.05;
                            let off = !self.is_enabled && st == 0;
                            if !(still && off) {
                                self.refresh_capture = None;
                                self.enable_failed();
                                return Err(ArmDriverError::EnableFreshenUnsafeGripper {
                                    joint: j,
                                    still,
                                    off,
                                    status: st,
                                });
                            }
                            if self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_some()
                            {
                                f.confirmed_mask |= 1u32 << j;
                            }
                        }
                    }
                    continue;
                }
                let causal = self.last_feedback_seq[j] > tx_boundary
                    && self
                        .last_feedback_instant[j]
                        .is_some_and(|rx_at| rx_at >= tx_at && !f.deadline.passed(rx_at));
                if causal
                    && self.fresh_measured_position(j, ENABLE_FRESHEN_EXIT_ARM_MAX_AGE).is_some()
                {
                    // Arm-joint hardware fault bits (e.g. 0xD LostCommunication left after an
                    // idle/MIT watchdog timeout) are checked after Restore + CMD_ENABLE in
                    // `poll_enable`, not before mode restore.
                    f.confirmed_mask |= 1u32 << j;
                }
            }

            if (f.confirmed_mask & f.target_mask) == f.target_mask {
                let mut re_stale = 0u32;
                for j in handshake::mask_joints(f.target_mask) {
                    let max_age = if pf_joint == Some(j) {
                        ENABLE_FRESHEN_EXIT_PF_MAX_AGE
                    } else {
                        ENABLE_FRESHEN_EXIT_ARM_MAX_AGE
                    };
                    if self.fresh_measured_position(j, max_age).is_none() {
                        re_stale |= 1u32 << j;
                    }
                }
                if re_stale == 0 {
                    self.refresh_capture = None;
                    return self.proceed_after_freshen(true);
                }
                f.confirmed_mask &= !re_stale;
                f.sent_mask &= !re_stale;
                for j in handshake::mask_joints(re_stale) {
                    f.tx_offset_ns[j] = EnableFreshen::UNSET_NS;
                }
                // Keep `self.refresh_capture` armed across the replacement drain so any fault,
                // motion, or state != 0 J7 frame arriving before the replacement 0xCC still vetoes.
            }
        }

        if expired_before_read || f.deadline.passed(self.now()) {
            let missing = f.target_mask & !f.confirmed_mask;
            let joint = if missing != 0 {
                missing.trailing_zeros() as usize
            } else {
                0
            };
            self.refresh_capture = None;
            self.enable_failed();
            return Err(ArmDriverError::EnableFreshenTimeout { joint });
        }

        if self.motor_energized_mask != 0 {
            self.service_urgent_enable_watchdog_holds(0, true)?;
        }

        let unsent = f.target_mask & !f.sent_mask;
        if unsent != 0 {
            let _ = self.send_freshen_polls(&mut f, unsent)?;
        } else if f.next_retry_at.is_some_and(|t| self.now() >= t) {
            let unconfirmed = f.target_mask & !f.confirmed_mask;
            if unconfirmed != 0 {
                let _ = self.send_freshen_polls(&mut f, unconfirmed)?;
            }
        }

        let wake = self.freshen_pending_wake(&f);
        self.torque_hs = Some(DamiaoTorque::Freshen(f));
        Ok(TorqueProgress::Pending { wake })
    }

    pub(in crate::driver::can_servo) fn proceed_after_freshen(&mut self, drained_now: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;

        // CTRL_MODE lives in motor RAM: a joint left in MIT by an interrupted gravity
        // compensation session must be back in the base mode before it is enabled.
        let dof = self.motor_ids.len();
        if dof > MAX_MODE_JOINTS {
            self.enable_failed();
            return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
        }
        let mut stray = [0usize; MAX_MODE_JOINTS];
        let mut stray_u8 = [0u8; MAX_MODE_JOINTS];
        let mut stray_len = 0usize;
        for i in 0..dof {
            if self.joint_modes[i] != Some(self.base_mode(i)) {
                stray[stray_len] = i;
                stray_u8[stray_len] = i as u8;
                stray_len += 1;
            }
        }
        let stray_slice = &stray[..stray_len];
        if let Some(j) = self.pos_force.as_ref().map(|p| p.joint).filter(|j| stray_slice.contains(j)) {
            if self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_none() {
                let refresh = match self.start_feedback_refresh(j, drained_now) {
                    Ok(r) => r,
                    Err(e) if is_estop_inhibited(&e) => return Err(e),
                    Err(e) => {
                        self.enable_failed();
                        return Err(e);
                    }
                };
                let wake = self.now().min(refresh.deadline.at);
                self.torque_hs = Some(DamiaoTorque::RefreshBeforeRestore {
                    refresh,
                    stray: stray_u8,
                    stray_len: stray_len as u8,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
        }
        if let Some(e) = self.pos_force_write_refused(stray_slice) {
            self.enable_failed();
            return Err(e);
        }
        self.start_restore_or_readback(stray_slice, false)
    }

    pub(in crate::driver::can_servo) fn start_restore_or_readback(&mut self, stray: &[usize], pf_refreshed: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        if !stray.is_empty() {
            let dof = self.motor_ids.len();
            if dof > MAX_MODE_JOINTS {
                self.enable_failed();
                return Err(ArmDriverError::Protocol("joint count exceeds MAX_MODE_JOINTS".into()));
            }
            let mut base = [DamiaoControlMode::PosVel; MAX_MODE_JOINTS];
            base[..dof].copy_from_slice(&self.base_modes[..dof]);
            let mut hold: [JointTarget; MAX_MODE_JOINTS] = std::array::from_fn(|_| JointTarget::position(0.0));
            for i in 0..dof {
                hold[i] = JointTarget::position(self.cached_states[i].position);
            }
            // after MIT tracking (fence armed, tripped or used since the last confirmed enable)
            // the base mode is restored with POS_VEL frames only: no MIT frame from cached
            // positions before the mode is confirmed (a new handover is the only way back)
            let dual = !self.mit_tracking_used;
            let w = self.write_modes_send(stray, &base[..dof], &hold[..dof], dual, ModeWritePolicy::EnableRestore)?;
            let wake = self.now();
            self.torque_hs = Some(DamiaoTorque::Restore { write: w, pf_refreshed });
            return Ok(TorqueProgress::Pending { wake });
        }
        self.readback_or_enable()
    }

    /// Budgeted legacy mapping-range retry for joints that were `RangeCheckStatus::Unverified` at
    /// startup: sends read-only CTRL_MODE / PMAX / VMAX / TMAX requests after an observed-empty
    /// RX boundary, validates causal replies, updates `self.range_check` in place (zero heap
    /// allocation), refuses with `RangeCheckMismatch` on any mismatch, and proceeds into enable
    /// once every unverified joint has been checked.
    pub(in crate::driver::can_servo) fn poll_range_check(&mut self, mut rc: RangeCheckPhase) -> Result<TorqueProgress, ArmDriverError> {
        const RIDS: [u8; 4] = [
            damiao::RID_CTRL_MODE,
            damiao::RID_PMAX,
            damiao::RID_VMAX,
            damiao::RID_TMAX,
        ];
        self.enable_estop_inhibited()?;
        let _ = self.receive_feedback();
        self.enable_estop_inhibited()?;

        let j = match rc.active_joint {
            Some(j) => usize::from(j),
            None => {
                let Some(next) = handshake::mask_joints(rc.pending_mask).next() else {
                    return self.continue_enable_after_drain(self.last_read_drained);
                };
                rc.active_joint = Some(next as u8);
                rc.boundary = None;
                rc.sent_rids = 0;
                rc.deadline = Deadline::after(self.now(), DAMIAO_RX_PASS);
                next
            }
        };

        if rc.boundary.is_none() {
            if self.last_read_drained {
                rc.boundary = Some(self.rx_empty_seq);
                self.register_reply_log.clear();
                rc.deadline = Deadline::after(self.now(), TICKED_RANGE_CHECK_REPLY_TIMEOUT);
            } else if rc.deadline.passed(self.now()) {
                self.awaiting_rx_boundary = true;
                self.enable_failed();
                return Err(ArmDriverError::RxDrainTimeout);
            } else {
                let wake = rc.deadline.at;
                self.torque_hs = Some(DamiaoTorque::RangeCheck(rc));
                return Ok(TorqueProgress::Pending { wake });
            }
        }

        let boundary = rc.boundary.expect("set above");
        let id = self.motor_ids[j];
        let base_mode = self.base_mode(j);
        let base = Self::mode_value(base_mode);
        let expected_id = self.expected_reply_id(j);

        if rc.sent_rids != 0x0F && !rc.deadline.passed(self.now()) {
            let mut sent_any = false;
            for (k, &rid) in RIDS.iter().enumerate() {
                if (rc.sent_rids & (1u8 << k)) != 0 {
                    continue;
                }
                match self.send_plain(TxKind::Handshake, self.enable_stop_stamp, Some(rc.deadline), false, |_| {
                    Some(damiao::build_read_register_frame(id, rid))
                }) {
                    Ok(_) => {
                        rc.sent_rids |= 1u8 << k;
                        sent_any = true;
                    }
                    Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.abort_enable_for_estop();
                        return Err(ArmDriverError::EstopInhibited);
                    }
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => break,
                    Err(TxRefusal::Transport(e)) => {
                        self.enable_failed();
                        return Err(ArmDriverError::Transport(e));
                    }
                    Err(r_err) => {
                        self.enable_failed();
                        return Err(Self::refusal_to_driver_err(r_err));
                    }
                }
            }
            if sent_any {
                self.enable_estop_inhibited()?;
                if let Err(e) = self.flush_can_and_commit() {
                    self.enable_failed();
                    return Err(ArmDriverError::Transport(e));
                }
                if self.remaining_rx_frame_limit() > 0 {
                    let _ = self.receive_feedback();
                    self.enable_estop_inhibited()?;
                }
            }
        }

        let find_reply = |rid: u8, require_expected_id: bool| {
            self.register_reply_log
                .iter()
                .rev()
                .find(|(r, fid, _, seq, _)| {
                    r.motor_id == id
                        && !r.write
                        && r.rid == rid
                        && *seq > boundary
                        && (!require_expected_id || *fid == expected_id)
                })
                .map(|(r, ..)| *r)
        };
        let r_mode = find_reply(damiao::RID_CTRL_MODE, base_mode == DamiaoControlMode::PosForce);
        let r_mode_any = find_reply(damiao::RID_CTRL_MODE, false);
        let r_p = find_reply(damiao::RID_PMAX, false);
        let r_v = find_reply(damiao::RID_VMAX, false);
        let r_t = find_reply(damiao::RID_TMAX, false);

        let all_replied = r_mode_any.is_some() && r_p.is_some() && r_v.is_some() && r_t.is_some();
        if !all_replied && !rc.deadline.passed(self.now()) {
            let wake = if rc.sent_rids != 0x0F { self.now() } else { rc.deadline.at };
            self.torque_hs = Some(DamiaoTorque::RangeCheck(rc));
            return Ok(TorqueProgress::Pending { wake });
        }

        let l = self.limits[j];
        let configured = [l.p_max, l.v_max, l.t_max];
        let ctrl_mode = r_mode_any.map(|r| r.as_u32());
        let (status, reported) = Self::evaluate_range_reading(
            r_p.map(|r| r.as_f32()),
            r_v.map(|r| r.as_f32()),
            r_t.map(|r| r.as_f32()),
            &configured,
        );
        self.mit_limits_verified[j] = status == RangeCheckStatus::Passed;
        let verified_ctrl = r_mode.map(|r| r.as_u32());
        let verified_mode = verified_ctrl.and_then(Self::mode_from_u32);
        if verified_ctrl == Some(base) && verified_mode == Some(base_mode) {
            if base_mode == DamiaoControlMode::PosForce {
                self.joint_modes[j] = Some(DamiaoControlMode::PosForce);
            }
            self.verified_modes[j] = Some(base_mode);
        } else if verified_ctrl.or(ctrl_mode).is_some() {
            self.joint_modes[j] = None;
            self.verified_modes[j] = None;
        } else {
            self.verified_modes[j] = None;
        }
        if let Some(ref mut check) = self.range_check {
            if let Some(entry) = check.joints.get_mut(j) {
                entry.status = status;
                entry.ctrl_mode = ctrl_mode;
                entry.reported = reported;
            }
        }

        rc.pending_mask &= !(1u32 << j);
        rc.active_joint = None;
        rc.boundary = None;
        rc.sent_rids = 0;

        if rc.pending_mask != 0 {
            let now = self.now();
            rc.deadline = Deadline::after(now, DAMIAO_RX_PASS);
            self.torque_hs = Some(DamiaoTorque::RangeCheck(rc));
            return Ok(TorqueProgress::Pending { wake: now });
        }

        let final_status = if let Some(ref mut check) = self.range_check {
            let st = if check.joints.iter().any(|jj| jj.status == RangeCheckStatus::Mismatch) {
                RangeCheckStatus::Mismatch
            } else if check.joints.iter().any(|jj| jj.status == RangeCheckStatus::Unverified) {
                RangeCheckStatus::Unverified
            } else {
                RangeCheckStatus::Passed
            };
            check.status = st;
            st
        } else {
            RangeCheckStatus::Passed
        };
        if final_status == RangeCheckStatus::Mismatch {
            self.enable_failed();
            return Err(ArmDriverError::RangeCheckMismatch);
        }
        self.continue_enable_after_drain(self.last_read_drained)
    }

    /// Budgeted enable pre-drain: waits under `poll_torque_budgeted`'s budget until a receive
    /// pass observes an empty RX queue, then continues into mode restore / enable TX.
    pub(in crate::driver::can_servo) fn poll_drain_before_enable(&mut self, deadline: Deadline) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let _ = self.receive_feedback();
        self.enable_estop_inhibited()?;
        if self.last_read_drained {
            return self.continue_enable_after_drain(true);
        }
        if deadline.passed(self.now()) {
            self.awaiting_rx_boundary = true;
            self.enable_failed();
            return Err(ArmDriverError::RxDrainTimeout);
        }
        let wake = deadline.at;
        self.torque_hs = Some(DamiaoTorque::DrainBeforeEnable(deadline));
        Ok(TorqueProgress::Pending { wake })
    }

    /// Establish a tight pre-transmit RX boundary, transmit `0xCC` for `joint` within the
    /// per-tick TX budget, and arm `refresh_capture`. Returns `Ok(None)` if RX cannot be drained
    /// or the TX budget is exhausted/backpressured before `deadline`.
    pub(in crate::driver::can_servo) fn send_refresh_with_boundary_opts(
        &mut self,
        joint: usize,
        deadline: Deadline,
        allow_enabled_state: bool,
    ) -> Result<Option<u64>, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        if !self.refresh_scheduler.slot_available() {
            return Ok(None);
        }
        if !self.drain_rx_for_boundary(deadline) {
            self.enable_estop_inhibited()?;
            self.check_enable_tx_authority()?;
            return Ok(None);
        }
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        if deadline.passed(self.now()) {
            return Ok(None);
        }
        // Snapshot the receive sequence and transmit instant right before transmitting `0xCC`:
        // only replies with `seq > boundary` AND receive time `rx_at >= tx_at` are accepted.
        let boundary = self.rx_seq;
        let tx_at = self.now();
        let id = self.motor_ids[joint];
        match self.send_plain(TxKind::Handshake, self.enable_stop_stamp, Some(deadline), false, |_| {
            Some(damiao::build_refresh_frame(id))
        }) {
            Ok(_) => {}
            Err(TxRefusal::DeadlinePassed | TxRefusal::Deferred) => {
                self.refresh_scheduler.record_deferred();
                return Ok(None);
            }
            Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                self.abort_enable_for_estop();
                return Err(ArmDriverError::EstopInhibited);
            }
            Err(r @ (TxRefusal::LeaseFenced | TxRefusal::EnableAuthorityExpired)) => {
                self.torque_hs = None;
                self.enable_failed();
                return Err(Self::refusal_to_driver_err(r));
            }
            Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                self.refresh_scheduler.record_deferred();
                return Ok(None);
            }
            Err(r) => return Err(Self::refusal_to_driver_err(r)),
        }
        self.enable_estop_inhibited()?;
        self.check_enable_tx_authority()?;
        self.flush_can_and_commit()?;
        self.refresh_scheduler.mark_slot_used();
        self.refresh_capture = Some(RefreshCapture {
            joint,
            boundary,
            tx_at,
            deadline,
            allow_enabled_state,
            first: None,
            unsafe_sample: None,
        });
        Ok(Some(boundary))
    }

    /// Start a bounded 0xCC feedback refresh for `joint`. Captures the fixed `REGISTER_REPLY_TIMEOUT`
    /// (50 ms) deadline before any drain or transmit so slow transport I/O never extends the window.
    /// Sends immediately only when `drained_now` is true; otherwise leaves `boundary: None` so
    /// `poll_feedback_refresh` establishes a fresh boundary first and never admits pre-request frames.
    pub(in crate::driver::can_servo) fn start_feedback_refresh(&mut self, joint: usize, drained_now: bool) -> Result<FeedbackRefresh, ArmDriverError> {
        self.enable_estop_inhibited()?;
        self.refresh_capture = None;
        let start = self.now();
        let deadline = Deadline::after(start, REGISTER_REPLY_TIMEOUT);
        let boundary = if drained_now && !deadline.passed(self.now()) {
            self.send_refresh_with_boundary_opts(joint, deadline, false)?
        } else {
            None
        };
        Ok(FeedbackRefresh { joint, boundary, deadline })
    }

    /// Serve a bounded 0xCC feedback refresh: accepts only when a post-boundary J7 reply
    /// arrived strictly before `r.deadline` (with `rx_at >= tx_at`, within
    /// `POS_FORCE_FEEDBACK_MAX_AGE`) AND the RX queue reached an observed empty-queue boundary
    /// (`last_read_drained`), so newer queued motion or fault replies beyond the RX budget can
    /// never be skipped. Any post-boundary unsafe J7 sample (motion or non-zero status/fault)
    /// vetoes immediately. Even at the expiry wake, runs one bounded RX drain before deciding;
    /// if the queue still has not drained at the deadline, fails closed (`TimedOut`).
    pub(in crate::driver::can_servo) fn poll_feedback_refresh(&mut self, r: FeedbackRefresh) -> Result<RefreshStep, ArmDriverError> {
        self.poll_feedback_refresh_opts(r, false)
    }

    pub(in crate::driver::can_servo) fn poll_feedback_refresh_opts(
        &mut self,
        mut r: FeedbackRefresh,
        allow_enabled_state: bool,
    ) -> Result<RefreshStep, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let expired_before_read = r.deadline.passed(self.now());
        match r.boundary {
            None => {
                if !expired_before_read {
                    r.boundary =
                        self.send_refresh_with_boundary_opts(r.joint, r.deadline, allow_enabled_state)?;
                }
            }
            Some(b) => {
                let _ = self.receive_feedback();
                self.enable_estop_inhibited()?;
                let cap_state = self
                    .refresh_capture
                    .as_ref()
                    .filter(|c| c.joint == r.joint && c.boundary == b)
                    .map(|c| (c.tx_at, c.first, c.unsafe_sample));
                if let Some((tx_at, first, unsafe_sample)) = cap_state {
                    if let Some(bad) = unsafe_sample {
                        self.refresh_capture = None;
                        return Ok(RefreshStep::Ready(bad));
                    }
                    if self.last_read_drained {
                        if let Some(sample) = first {
                            let fresh_rx = self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE;
                            let fresh_bound = sample
                                .queue_bound
                                .is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
                            if sample.seq > b
                                && sample.rx_at >= tx_at
                                && !r.deadline.passed(sample.rx_at)
                                && fresh_rx
                                && fresh_bound
                                && sample.position.is_finite()
                            {
                                self.refresh_capture = None;
                                // Do NOT roll back live cached_states/motor_status: return Ready(sample)
                                // and let callers enforce safety checks on BOTH `sample` and live
                                // telemetry (now up to date through the empty-queue boundary).
                                return Ok(RefreshStep::Ready(sample));
                            }
                        }
                    }
                }
            }
        }
        if r.deadline.passed(self.now()) {
            self.refresh_capture = None;
            return Ok(RefreshStep::TimedOut);
        }
        Ok(RefreshStep::Pending(r))
    }

    pub(in crate::driver::can_servo) fn poll_refresh_before_restore(
        &mut self,
        refresh: FeedbackRefresh,
        stray_u8: [u8; MAX_MODE_JOINTS],
        stray_len: u8,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let joint = refresh.joint;
        let len = usize::from(stray_len).min(MAX_MODE_JOINTS);
        let mut stray = [0usize; MAX_MODE_JOINTS];
        for i in 0..len {
            stray[i] = usize::from(stray_u8[i]);
        }
        let stray_slice = &stray[..len];
        match self.poll_feedback_refresh(refresh) {
            Ok(RefreshStep::Ready(sample)) => {
                let sample_fresh = sample.position.is_finite()
                    && self.since(sample.rx_at) <= POS_FORCE_FEEDBACK_MAX_AGE
                    && sample.queue_bound.is_some_and(|qb| self.since(qb) <= POS_FORCE_FEEDBACK_MAX_AGE);
                if let Some(e) = self.pos_force_sample_refused(
                    joint,
                    "position-force mode not written",
                    sample_fresh,
                    sample.velocity,
                    sample.motor_status,
                ) {
                    self.enable_failed();
                    return Err(e);
                }
                if let Some(e) = self.pos_force_write_refused(stray_slice) {
                    self.enable_failed();
                    return Err(e);
                }
                self.start_restore_or_readback(stray_slice, true)
            }
            Ok(RefreshStep::Pending(r)) => {
                let wake = r.deadline.at;
                self.torque_hs = Some(DamiaoTorque::RefreshBeforeRestore {
                    refresh: r,
                    stray: stray_u8,
                    stray_len,
                });
                Ok(TorqueProgress::Pending { wake })
            }
            Ok(RefreshStep::TimedOut) => {
                let e = self.pos_force_write_refused(stray_slice).unwrap_or_else(|| ArmDriverError::HardwareFault(format!(
                    "joint {} position-force mode not written: no timely post-refresh feedback within {} ms; capped operation unavailable",
                    joint + 1,
                    REGISTER_REPLY_TIMEOUT.as_millis()
                )));
                self.enable_failed();
                Err(e)
            }
            Err(e) if is_estop_inhibited(&e) => Err(e),
            Err(e) => {
                self.enable_failed();
                Err(e)
            }
        }
    }

    /// Serve the base-mode restore: replies recorded strictly before the deadline complete ->
    /// readback / enable; at or after the deadline, no new RX frames are accepted and already-recorded
    /// timely replies are still evaluated before timing out missing joints. If the position-force
    /// gripper's feedback expired mid-restore (after earlier stray joints transmitted) and its mode
    /// was not yet written, transition once into `RefreshBeforeRestore` for that joint.
    pub(in crate::driver::can_servo) fn poll_restore(&mut self, mut w: ModeWrite, pf_refreshed: bool) -> Result<TorqueProgress, ArmDriverError> {
        self.enable_estop_inhibited()?;
        let expired_before_read = w.deadline.passed(self.now());
        if !expired_before_read {
            let _ = self.receive_feedback();
            self.enable_estop_inhibited()?;
            if self.motor_energized_mask != 0 && !w.deadline.passed(self.now()) {
                let mut changing_mask = 0u32;
                for &raw_i in &w.sent[..usize::from(w.sent_len)] {
                    let i = usize::from(raw_i);
                    if i < MAX_MODE_JOINTS {
                        changing_mask |= 1u32 << i;
                    }
                }
                self.service_urgent_enable_watchdog_holds(changing_mask, w.deferred_len > 0)?;
            }
            if w.deferred_len > 0 && !w.deadline.passed(self.now()) {
                self.drain_restore_deferred(&mut w)?;
            }
        }
        let finished = if w.deadline.passed(self.now()) {
            self.register_reply_log.retain(|(_, _, rx_at, _, _)| !w.deadline.passed(*rx_at));
            self.register_replies.clear();
            for (rep, ..) in &self.register_reply_log {
                if self.register_replies.len() < 64 { self.register_replies.push(*rep); }
            }
            for k in 0..usize::from(w.deferred_len) {
                let raw_i = w.deferred[k];
                if w.deferred_stage[k] == 0 {
                    Self::push_mode_joint_unique(&mut w.unsent, &mut w.unsent_len, raw_i);
                } else {
                    Self::push_mode_joint_unique(&mut w.hold_failed, &mut w.hold_failed_len, raw_i);
                }
            }
            w.deferred_len = 0;
            true
        } else {
            w.deferred_len == 0 && self.mode_write_replied(&w)
        };
        let pf_unsent = self.pos_force.as_ref().map(|p| p.joint)
            .filter(|&j| w.unsent[..usize::from(w.unsent_len)].contains(&(j as u8)));
        if !finished {
            let wake = if w.deferred_len > 0 { self.now() } else { w.deadline.at };
            self.torque_hs = Some(DamiaoTorque::Restore { write: w, pf_refreshed });
            return Ok(TorqueProgress::Pending { wake });
        }
        let (failed, failed_len) = self.write_modes_finish(w);
        if failed_len > 0 {
            if !pf_refreshed
                && pf_unsent.is_some_and(|j| failed_len == 1 && usize::from(failed[0]) == j
                    && self.fresh_measured_position(j, POS_FORCE_FEEDBACK_MAX_AGE).is_none())
            {
                let j = pf_unsent.unwrap();
                let drained_now = !expired_before_read && self.last_read_drained;
                let refresh = match self.start_feedback_refresh(j, drained_now) {
                    Ok(r) => r,
                    Err(e) if is_estop_inhibited(&e) => return Err(e),
                    Err(e) => {
                        self.enable_failed();
                        return Err(e);
                    }
                };
                let mut stray = [0u8; MAX_MODE_JOINTS];
                stray[0] = j as u8;
                let wake = self.now().min(refresh.deadline.at);
                self.torque_hs = Some(DamiaoTorque::RefreshBeforeRestore {
                    refresh,
                    stray,
                    stray_len: 1,
                });
                return Ok(TorqueProgress::Pending { wake });
            }
            self.enable_failed();
            return Err(ArmDriverError::HardwareFault(format!(
                "Control mode of joints {:?} could not be restored before enable",
                failed[..failed_len].iter().map(|&i| usize::from(i) + 1).collect::<Vec<_>>())));
        }
        self.readback_or_enable()
    }

}
