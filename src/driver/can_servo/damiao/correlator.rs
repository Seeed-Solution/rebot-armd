//! Damiao CAN RX frame processing, register reply capture, and MIT probe correlation.

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
    /// The bounded receive of `read_joint_states` (at most 64 frames or 5 ms) with its
    /// recovery holds and stale-motor refresh; updates `cached_states`.
    /// The receive pass started at `t0` is over: 5 ms, or the caller's service deadline
    /// (`poll_torque`), whichever comes first.
    pub(in crate::driver::can_servo) fn rx_over(&self, t0: Instant) -> bool {
        let now = self.now();
        now.saturating_duration_since(t0) >= DAMIAO_RX_PASS || self.rx_deadline.is_some_and(|d| now >= d)
    }

    /// Decode one received CAN frame into the driver's feedback cache and, when
    /// `record_register_replies` is true, into the register reply buffers.
    pub(in crate::driver::can_servo) fn process_rx_frame(&mut self, frame: &CanFrame, record_register_replies: bool, rx_time: Option<CanRxTime>) {
        self.rx_seq += 1;
        if self.awaiting_rx_boundary {
            return;
        }
        // Ignore transmit commands: on SocketCAN with local echo enabled,
        // host transmit frames for PosVel are 0x100 + tx_id, gripper echo 0x300 + tx_id,
        // or special 0x7FF, or command frames.
        if frame.id == 0x7FF
            || self
                .motor_ids
                .iter()
                .any(|&tx_id| {
                    frame.id == (0x100 + tx_id as u32)
                        || frame.id == (u32::from(damiao::POS_FORCE_BASE_ID) + tx_id as u32)
                })
            || frame.data == damiao::CMD_ENABLE
            || frame.data == damiao::CMD_DISABLE
            || frame.data == damiao::CMD_CLEAR_FAULTS
            || frame.data == damiao::CMD_SET_ZERO
        {
            return;
        }
        // Local echo of an MIT command (same ID as some feedback paths).
        if self.last_target_frames.iter().flatten().any(|f| f.id == frame.id && f.data == frame.data) {
            return;
        }
        if let Some(reply) = damiao::parse_register_reply(frame)
            .filter(|r| self.motor_ids.contains(&r.motor_id))
        {
            // A register reply is not feedback: byte 0 would decode as state 0.
            // Host dequeue time never counts; register replies require a kernel/transport RX timestamp.
            let Some(rx_t) = rx_time else {
                return;
            };
            let decode_now = self.now();
            let Ok((rx_at, decode_offset_ns)) = can_rx_time_to_monotonic(&self.can, rx_t, decode_now) else {
                return;
            };
            let motor_idx = self.motor_ids.iter().position(|&id| id == reply.motor_id);
            // L2: Accept causal post-boundary on-tick `RID_CTRL_MODE` readback replies even during
            // ordinary `read_joint_states` (`record_register_replies == false`).
            if !reply.write && reply.rid == damiao::RID_CTRL_MODE {
                if let Some(idx) = motor_idx {
                    let owned_by_mode_switch =
                        idx < 32 && (self.mode_switch_active_mask & (1u32 << idx)) != 0;
                    // R2 Blocker 1: joints in `mode_write_uncertain_mask` MUST NOT clear eagerly
                    // on the first raw reply before RX closure; they are resolved strictly AFTER
                    // observed empty RX queue closure via the timed causal register matcher!
                    let uncertain_mode = self.is_axis_mode_uncertain(idx);
                    let expected_id = self.expected_reply_id(idx);
                    let id_ok = !owned_by_mode_switch
                        && !uncertain_mode
                        && (self.base_mode(idx) != DamiaoControlMode::PosForce
                            || frame.id == expected_id);
                    if id_ok
                        && self
                            .mode_readback_boundary
                            .get(idx)
                            .copied()
                            .flatten()
                            .is_some_and(|b| self.rx_seq > b)
                    {
                        if let Some(mode) = Self::mode_from_u32(reply.as_u32()) {
                            let prev = self.joint_modes[idx];
                            if prev != Some(mode) {
                                self.pending_tx.discard_joint(idx);
                                self.pending_hold_tx.discard_joint(idx);
                            }
                            self.verified_modes[idx] = Some(mode);
                            self.joint_modes[idx] = Some(mode);
                            self.mode_readback_boundary[idx] = None;
                            if self
                                .pos_force
                                .as_ref()
                                .is_some_and(|pf| pf.joint == idx)
                                && mode != DamiaoControlMode::PosForce
                            {
                                self.pos_force_verified = false;
                            }
                            #[cfg(not(target_arch = "wasm32"))]
                            if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS
                            {
                                self.shadow_staging.push(
                                    crate::torque::shadow::ShadowDriverObs::ModeVerified {
                                        channel: idx as u8,
                                        mode: damiao_mode_to_shadow_control_mode(mode),
                                        now: decode_now,
                                        provenance:
                                            crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                    },
                                );
                            }
                        }
                    }
                }
            }
            if record_register_replies {
                if self.register_replies.len() < 64 { self.register_replies.push(reply); }
                if self.register_reply_log.len() < 64 { self.register_reply_log.push((reply, frame.id, rx_at, self.rx_seq, self.rx_empty_seq)); }
                if self.comm_reg_reply_len < self.comm_reg_replies.len() {
                    self.comm_reg_replies[self.comm_reg_reply_len] = Some(crate::mit::CommissioningRegReply {
                        motor_id: reply.motor_id,
                        write: reply.write,
                        rid: reply.rid,
                        reply_id: frame.id,
                        value: reply.value,
                        rx_at,
                        rx_seq: self.rx_seq,
                        rx_empty_seq: self.rx_empty_seq,
                        decode_offset_ns,
                    });
                    self.comm_reg_reply_len += 1;
                } else {
                    self.comm_reg_reply_overflow = true;
                }
            }
            return;
        }
        if !frame.is_extended && !frame.is_rtr && frame.dlc == 8 {
            // Byte 0 names the motor; bind per-motor feedback CAN IDs (own ID and 0x10 + own ID)
            // to that specific joint so another motor's MST_ID (e.g. 0x16) cannot spoof it, while
            // preserving shared-master addressing (0x00 and master_id).
            let fid = damiao::feedback_motor_id(frame);
            let Some(idx) = self.motor_ids.iter().position(|&id| id as u8 == fid) else {
                return;
            };
            let motor_id = u32::from(self.motor_ids[idx]);
            let is_valid_feedback_id = match self.base_mode(idx) {
                DamiaoControlMode::PosForce => frame.id == self.expected_reply_id(idx),
                _ => frame.id == 0
                    || frame.id == self.master_id
                    || frame.id == motor_id
                    || frame.id == (0x10 + motor_id),
            };
            if !is_valid_feedback_id {
                return;
            }
            if let Ok(feedback) = damiao::unpack_feedback_frame(frame, &self.limits[idx]) {
                if !self.awaiting_rx_boundary
                    && feedback.state_or_error != 0
                    && idx < MAX_MODE_JOINTS
                {
                    self.last_adverse_feedback_seq[idx] = self.rx_seq;
                }
                // During an active shutdown disarm probe, record ANY decoded adverse (enabled or
                // faulted) frame BEFORE RX timestamp filtering so untimestamped/unmappable adverse
                // replies still refuse disarm proof, while positive state==0 proof continues to
                // require a valid kernel/transport RX timestamp.
                if self.disarm_probe_active
                    && !self.awaiting_rx_boundary
                    && feedback.state_or_error != 0
                    && idx < MAX_MODE_JOINTS
                    && (self.disarm_probe_adverse_mask & (1u32 << idx)) == 0
                {
                    self.disarm_probe_adverse_mask |= 1u32 << idx;
                    self.disarm_probe_adverse_state[idx] = feedback.state_or_error;
                }
                // Host dequeue time never counts: require a kernel/transport receive timestamp,
                // and reject future-dated timestamps completely (never clamp).
                let Some(rx_t) = rx_time else {
                    #[cfg(not(target_arch = "wasm32"))]
                    if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                        let decode_now = self.now();
                        let sample_id = self.shadow_staging.next_sample_id();
                        self.shadow_staging.push(
                            crate::torque::shadow::ShadowDriverObs::CausalSample {
                                channel: idx as u8,
                                sample_id,
                                stamp: crate::torque::shadow::RxStampKind::Missing {
                                    reset_window: false,
                                },
                                decode_now,
                                position_rad: feedback.position,
                                velocity_rad_s: Some(feedback.velocity),
                                temperature_c: Some(feedback.t_mos.max(feedback.t_rotor)),
                                reported_enabled: Some(feedback.state_or_error == 1),
                                correlated_after_write_done: None,
                                poll_issued: None,
                                provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                            },
                        );
                    }
                    return;
                };
                let decode_now = self.now();
                let (rx_at, decode_offset_ns) = match can_rx_time_to_monotonic(&self.can, rx_t, decode_now) {
                    Ok(mapped) => mapped,
                    Err(reject) => {
                        self.rest_pose_window[idx].reset();
                        if self.mit_expiry_recovery_active && idx < MAX_MODE_JOINTS {
                            self.mit_expiry_axes[idx].observe_untimestamped_feedback();
                        }
                        #[cfg(not(target_arch = "wasm32"))]
                        if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                            let sample_id = self.shadow_staging.next_sample_id();
                            self.shadow_staging.push(
                                crate::torque::shadow::ShadowDriverObs::CausalSample {
                                    channel: idx as u8,
                                    sample_id,
                                    stamp: can_rx_reject_to_shadow_stamp(reject),
                                    decode_now,
                                    position_rad: feedback.position,
                                    velocity_rad_s: Some(feedback.velocity),
                                    temperature_c: Some(feedback.t_mos.max(feedback.t_rotor)),
                                    reported_enabled: Some(feedback.state_or_error == 1),
                                    correlated_after_write_done: None,
                                    poll_issued: None,
                                    provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                                },
                            );
                        }
                        return;
                    }
                };
                // Per-motor rest-pose causal boundary check (after last motion command):
                // - OffsetDrifted (> 1 ms CLOCK_REALTIME step): discard completely and restart window.
                // - InSettle (<= write_done + 20 ms): reset rest window & do NOT refresh pose_feedback_instant.
                // - Satisfied: eligible for pose_feedback_instant and K=3 rest-pose window.
                let boundary_check = match self.rest_pose_boundary[idx] {
                    None => RestBoundaryCheck::Satisfied,
                    Some(boundary) => {
                        check_rest_pose_boundary(boundary, rx_t, decode_now, decode_offset_ns)
                    }
                };
                if boundary_check == RestBoundaryCheck::OffsetDrifted {
                    self.rest_pose_window[idx].reset();
                    if self.mit_expiry_recovery_active && idx < MAX_MODE_JOINTS {
                        self.mit_expiry_axes[idx].observe_untimestamped_feedback();
                    }
                    let (cur_rx, cur_off) = self.capture_rx_clock_sample(decode_now);
                    self.rest_pose_boundary[idx] = Some(RestPoseBoundary {
                        write_done: decode_now,
                        write_done_rx_clock: Some(cur_rx),
                        write_done_offset_ns: cur_off,
                    });
                    #[cfg(not(target_arch = "wasm32"))]
                    if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                        let sample_id = self.shadow_staging.next_sample_id();
                        self.shadow_staging.push(
                            crate::torque::shadow::ShadowDriverObs::CausalSample {
                                channel: idx as u8,
                                sample_id,
                                stamp: crate::torque::shadow::RxStampKind::OffsetDrifted {
                                    rebased_settle: true,
                                },
                                decode_now,
                                position_rad: feedback.position,
                                velocity_rad_s: Some(feedback.velocity),
                                temperature_c: Some(feedback.t_mos.max(feedback.t_rotor)),
                                reported_enabled: Some(feedback.state_or_error == 1),
                                correlated_after_write_done: None,
                                poll_issued: None,
                                provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                            },
                        );
                    }
                    return;
                }
                let queue_bound = self.rx_empty_instant;
                let seq = self.rx_seq;
                let prev_rx = self.last_feedback_instant[idx];
                let prev_qb = self.feedback_queue_bound[idx];
                if prev_rx.is_none_or(|p| rx_at.saturating_duration_since(p) > STOPPED_DISABLE_FEEDBACK_MAX_AGE)
                    || prev_qb.is_none_or(|qb| rx_at.saturating_duration_since(qb) > STOPPED_DISABLE_FEEDBACK_MAX_AGE)
                    || queue_bound.is_none()
                    || !feedback.position.is_finite()
                {
                    self.feedback_continuity_epoch[idx] =
                        self.feedback_continuity_epoch[idx].wrapping_add(1);
                }
                self.cached_states[idx] = JointState {
                    position: feedback.position,
                    velocity: feedback.velocity,
                    torque: feedback.torque,
                    temperature: feedback.t_mos.max(feedback.t_rotor),
                };
                if feedback.position.is_finite() && idx < MAX_MODE_JOINTS {
                    self.motor_keepalive_hold_pos[idx] = Some(feedback.position);
                }
                self.last_feedback_instant[idx] = Some(rx_at);
                if let Some(slot) = self.last_feedback_decode_offset_ns.get_mut(idx) {
                    *slot = decode_offset_ns;
                }
                self.feedback_seq[idx] = self.feedback_seq[idx].wrapping_add(1);
                if self.mit_expiry_recovery_active && idx < MAX_MODE_JOINTS {
                    self.mit_expiry_axes[idx].observe_feedback(rx_at);
                }
                self.feedback_queue_bound[idx] = queue_bound;
                self.last_feedback_seq[idx] = seq;
                self.motor_status[idx] = feedback.state_or_error;
                if !self.awaiting_rx_boundary && idx < MAX_MODE_JOINTS {
                    if feedback.state_or_error == 1 {
                        self.motor_energized_mask |= 1u32 << idx;
                        if self.motor_last_ctrl_tx[idx].is_none() {
                            self.motor_last_ctrl_tx[idx] = Some(rx_at);
                        }
                    } else if damiao_fault_name(feedback.state_or_error).is_some() {
                        // Damiao hardware faults (>= 0x8, including 0xD) self-disable the motor.
                        self.motor_energized_mask &= !(1u32 << idx);
                        self.motor_last_ctrl_tx[idx] = None;
                    }
                }
                if self.pos_force.as_ref().is_some_and(|pf| pf.joint == idx) {
                    self.pf_last_fresh_feedback = if feedback.position.is_finite() {
                        queue_bound.map(|qb| (rx_at, qb))
                    } else {
                        None
                    };
                    // Use the EXACT SAME freshness predicate as live checks at `decode_now`:
                    // BOTH `decode_now - rx_at <= 100 ms` AND `decode_now - queue_bound <= 100 ms`
                    // AND finite position must hold, so a frame that sat > 100 ms in the RX queue
                    // (even with an earlier `rx_timestamp` close to `queue_bound`) can NEVER
                    // overwrite `pf_last_fresh_position`.
                    if self.pos_force_feedback_valid(idx, decode_now) {
                        self.pf_last_fresh_position = Some(feedback.position);
                        self.pf_fresh_gen = self.pf_fresh_gen.wrapping_add(1);
                    }
                }
                if !self.awaiting_rx_boundary && seq > self.motor_enable_seq[idx] {
                    self.motor_post_enable_feedback[idx] = true;
                    if feedback.state_or_error != 1 {
                        self.reported_torque_loss_sample_seq =
                            self.reported_torque_loss_sample_seq.wrapping_add(1).max(1);
                    }
                }
                if !self.awaiting_rx_boundary {
                    if feedback.state_or_error == 0 {
                        // Keep probe samples provisional until `probe_hardware_disarmed_until`
                        // observes an empty post-poll RX boundary with zero adverse frames.
                        let after_enable = match self.motor_last_enable_tx[idx] {
                            Some((en_seq, en_at)) => seq > en_seq && rx_at >= en_at,
                            None => true,
                        };
                        let mit_recovery_active = self.mit_expiry_recovery_active
                            || self.mit_expiry_budgeted_boundary.is_some();
                        if after_enable && !self.disarm_probe_active && !mit_recovery_active {
                            self.motor_disarmed_after_enable[idx] = true;
                            if idx < MAX_MODE_JOINTS && !self.is_enabling {
                                self.motor_energized_mask &= !(1u32 << idx);
                                self.motor_last_ctrl_tx[idx] = None;
                            }
                        }
                        if let Some((pd_seq, pd_at)) = self.panic_disable_tx_boundary[idx] {
                            if seq > pd_seq && rx_at >= pd_at {
                                self.panic_disable_confirmed[idx] = true;
                            }
                        }
                    } else {
                        self.motor_disarmed_after_enable[idx] = false;
                        self.panic_disable_confirmed[idx] = false;
                    }
                }
                let mut window_offset_drifted = false;
                match boundary_check {
                    RestBoundaryCheck::OffsetDrifted => unreachable!(),
                    RestBoundaryCheck::InSettle => {
                        self.rest_pose_window[idx].reset();
                    }
                    RestBoundaryCheck::Satisfied => {
                        self.pose_feedback_instant[idx] = Some(rx_at);
                        if let Some(ref gate) = self.stopped_disable_gate {
                            // Rest pose is judged from fresh causal feedback on arm joints (J1–J6)
                            // ONLY; J7 (gripper) never gates rest-pose disable.
                            if gate.is_arm_joint(idx) {
                                if !self.awaiting_rx_boundary
                                    && queue_bound.is_some()
                                    && joint_state_valid(&self.cached_states[idx], self.capabilities())
                                    && gate.joint_at_rest(idx, feedback.position)
                                {
                                    let release_id = self.control_release_id;
                                    if self.rest_pose_window[idx].record_agree(
                                        rx_at,
                                        release_id,
                                        STOPPED_DISABLE_FEEDBACK_MAX_AGE,
                                        decode_offset_ns,
                                    ) == WindowRecord::OffsetDrifted
                                    {
                                        window_offset_drifted = true;
                                        let (cur_rx, cur_off) = self.capture_rx_clock_sample(decode_now);
                                        self.rest_pose_boundary[idx] = Some(RestPoseBoundary {
                                            write_done: decode_now,
                                            write_done_rx_clock: Some(cur_rx),
                                            write_done_offset_ns: cur_off,
                                        });
                                    }
                                } else {
                                    self.rest_pose_window[idx].record_disagree();
                                }
                            }
                        }
                    }
                }
                #[cfg(not(target_arch = "wasm32"))]
                if self.shadow_staging.is_enabled() && idx < crate::torque::MAX_CHANNELS {
                    let sample_id = self.shadow_staging.next_sample_id();
                    let rest_qualified = !self.awaiting_rx_boundary
                        && queue_bound.is_some()
                        && joint_state_valid(&self.cached_states[idx], self.capabilities());
                    let stamp = if window_offset_drifted {
                        crate::torque::shadow::RxStampKind::OffsetDrifted {
                            rebased_settle: true,
                        }
                    } else {
                        let drift_ns = self
                            .rest_pose_boundary[idx]
                            .and_then(|b| b.write_done_offset_ns)
                            .zip(decode_offset_ns)
                            .map_or(0, |(w, d)| (d - w).unsigned_abs().min(u128::from(u64::MAX)) as u64);
                        if rest_qualified {
                            crate::torque::shadow::RxStampKind::Valid {
                                rx_at,
                                drift_ns,
                                live_post_settle: boundary_check == RestBoundaryCheck::Satisfied,
                            }
                        } else {
                            crate::torque::shadow::RxStampKind::UnqualifiedRestEvidence {
                                rx_at,
                                drift_ns,
                            }
                        }
                    };
                    self.shadow_staging
                        .push(crate::torque::shadow::ShadowDriverObs::CausalSample {
                            channel: idx as u8,
                            sample_id,
                            stamp,
                            decode_now,
                            position_rad: feedback.position,
                            velocity_rad_s: Some(feedback.velocity),
                            temperature_c: Some(feedback.t_mos.max(feedback.t_rotor)),
                            reported_enabled: Some(feedback.state_or_error == 1),
                            correlated_after_write_done: None,
                            poll_issued: None,
                            provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                        });
                }
                if let Some(cap) = self.refresh_capture.as_mut() {
                    if cap.joint == idx && seq > cap.boundary && rx_at >= cap.tx_at {
                        let sample = RefreshSample {
                            position: feedback.position,
                            velocity: feedback.velocity,
                            motor_status: feedback.state_or_error,
                            rx_at,
                            queue_bound,
                            seq,
                        };
                        let status_unsafe = if cap.allow_enabled_state {
                            feedback.state_or_error != 0 && feedback.state_or_error != 1
                        } else {
                            feedback.state_or_error != 0
                        };
                        let vel_unsafe = if cap.allow_enabled_state && feedback.state_or_error == 1 {
                            !feedback.velocity.is_finite()
                        } else {
                            !(feedback.velocity.is_finite() && feedback.velocity.abs() <= 0.05)
                        };
                        let unsafe_motion_or_state = vel_unsafe || status_unsafe;
                        if unsafe_motion_or_state && cap.unsafe_sample.is_none() {
                            cap.unsafe_sample = Some(sample);
                        }
                        if cap.first.is_none()
                            && !cap.deadline.passed(rx_at)
                            && feedback.position.is_finite()
                            && queue_bound.is_some()
                        {
                            cap.first = Some(sample);
                        }
                    }
                }
            }
        }
    }

    pub(in crate::driver::can_servo) fn mark_rx_empty(&mut self, attempt: Instant) {
        self.rx_empty_instant = Some(attempt);
        self.last_read_drained = true;
        self.rx_seq += 1;
        self.rx_empty_seq = self.rx_seq;
        // Note (E1): An empty host RX queue is NOT a causal torque boundary; `motor_*_tx`
        // boundaries are satisfied solely by host monotonic receive time >= write_done + SETTLE.
        if self.awaiting_rx_boundary {
            self.awaiting_rx_boundary = false;
            self.last_feedback_instant.fill(None);
            self.pose_feedback_instant.fill(None);
            self.rest_pose_window.fill(CausalWindow::default());
            self.pf_last_fresh_feedback = None;
            let boundary_now = self.now();
            self.last_enable_command = Some(boundary_now);
            self.last_enable_command_rx_seq = Some(self.rx_seq);
        }
    }

    pub(in crate::driver::can_servo) fn receive_feedback(&mut self) -> Result<(), ArmDriverError> {
        // Every hardware receive pass opens a fresh per-tick TX budget window unless we are inside
        // `poll_torque_inner` or already opened the single TX pass for the current managed control
        // release, so transition polls, RX recovery/refresh, and Step 5 output share one budget.
        if !self.in_poll_torque {
            self.ensure_release_tx_tick();
        }
        let max_frames = self.remaining_rx_frame_limit();
        let t0 = self.now();
        let mut frames_processed = 0;
        let mut rx_drained = false;
        let mut hit_err = false;
        self.last_read_drained = false;
        while frames_processed < max_frames && !self.rx_over(t0) {
            // Taken before the receive: an empty result means the queue was empty after it.
            let attempt = self.now();
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, true, rx_time);
                    if frames_processed >= max_frames || self.rx_over(t0) {
                        break;
                    }
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(_) => {
                    hit_err = true;
                    break;
                }
            }
        }
        self.account_rx_pass(frames_processed, !rx_drained && !hit_err);

        // R2 Blocker 1: Resolve SAFE-HOLD / stopped recovery `REG_READ(RID_CTRL_MODE)` probes for
        // mode-uncertain joints strictly AFTER observed empty RX queue closure using the timed
        // causal register matcher (`commissioning_find_register_reply`), rejecting pre-TX stamps,
        // clock-offset drift, duplicates, overflow, wrong reply IDs, and unverified MIT ranges.
        if rx_drained && self.mode_write_uncertain_mask != 0 && self.mode_switch_active_mask == 0 {
            let n = self.motor_ids.len().min(MAX_MODE_JOINTS);
            let now = self.now();
            for j in 0..n {
                if !self.is_axis_mode_uncertain(j) {
                    continue;
                }
                let Some((after_seq, tx_at, _retry_at, probe_deadline, tx_offset_ns)) =
                    self.uncertain_readback_probe[j]
                else {
                    continue;
                };
                // R3 Major 2: expire probe BEFORE matching so delayed replies outside the
                // original `REGISTER_REPLY_TIMEOUT` window can never clear uncertainty.
                if now >= probe_deadline {
                    self.uncertain_readback_probe[j] = None;
                    continue;
                }
                match self.mode_switch_find_ctrl_mode_reply(
                    j,
                    false,
                    after_seq,
                    tx_at,
                    probe_deadline,
                    tx_offset_ns,
                    true,
                ) {
                    crate::motor_mode::ModeRegMatch::Matched {
                        mode: Some(ctrl_mode),
                        ..
                    } => {
                        let dm = control_mode_to_damiao(ctrl_mode);
                        let eligible = match dm {
                            Some(DamiaoControlMode::PosVel) => true,
                            Some(DamiaoControlMode::Mit) => {
                                self.mit_limits_verified.get(j).copied().unwrap_or(false)
                                    && !self.mit_ownership_lost
                                    && self.mit_fence_trip.is_none()
                            }
                            // PositionForce requires full stationary causal feedback proof; never
                            // auto-enable PosForce from a bare recovery readback.
                            Some(DamiaoControlMode::PosForce) | None => false,
                        };
                        if eligible {
                            if let Some(dm) = dm {
                                self.pending_tx.discard_joint(j);
                                self.pending_hold_tx.discard_joint(j);
                                self.verified_modes[j] = Some(dm);
                                self.joint_modes[j] = Some(dm);
                                self.mode_readback_boundary[j] = None;
                                if j < 32 {
                                    self.mode_write_uncertain_mask &= !(1u32 << j);
                                }
                                if self.pos_force.as_ref().is_some_and(|pf| pf.joint == j) {
                                    self.pos_force_verified = false;
                                }
                                self.uncertain_readback_probe[j] = None;
                                #[cfg(not(target_arch = "wasm32"))]
                                self.stage_damiao_mode_verified(j, dm);
                            }
                        } else {
                            self.uncertain_readback_probe[j] = None;
                        }
                    }
                    crate::motor_mode::ModeRegMatch::Matched { mode: None, .. }
                    | crate::motor_mode::ModeRegMatch::Ambiguous => {
                        // Conflicting/delayed/duplicate reply -> keep uncertainty latched!
                        self.uncertain_readback_probe[j] = None;
                    }
                    crate::motor_mode::ModeRegMatch::Pending => {}
                }
            }
        }

        // Evaluate feedback age only after receive() explicitly reports an empty
        // queue. If the bounded budget expires first, defer recovery to the next
        // tick so a frame beyond the budget cannot be mistaken for a bus timeout.
        // MIT tracking fence: a stale joint trips it; no recovery frame of any kind is sent
        // (no cached MIT replay, no invented POS_VEL hold) while it is armed or tripped.
        if self.mit_fence.is_some() {
            if rx_drained {
                let _ = self.fence_check();
            } else {
                // the bounded read (64 frames or 5 ms) could not reach the newest sample: the
                // queued latency is unknown and nothing dequeued counts as fresh
                self.trip_fence(MitFenceTripReason::RxBacklog {
                    frames: frames_processed.min(u16::MAX as usize) as u16,
                });
            }
        }
        if self.strict_stop_check(None).is_err() {
            self.discard_pending_tx_stopped();
        }
        let rx_stop_stamp = self.current_stop_stamp();
        let stopped_pending = self.pending_tx.drop_stopped(rx_stop_stamp);
        if stopped_pending > 0 {
            crate::diagnostics::tick::record_tx_dropped_stopped(stopped_pending as u32);
        }
        if self.pending_stop_check(None).is_err() {
            self.discard_pending_hold_tx();
        } else {
            let _ = self.drop_stopped_hold_tx(rx_stop_stamp);
        }
        // Unified owner-local `0xCC` refresh scheduler: reserves at most ONE `0xCC` slot per tick
        // BEFORE autonomous recovery and ordinary TX across `FailedHold`, `Idle`-torqued, and
        // stopped states — serving every joint lacking recent replies, including J7 under
        // `NoOutputUntilFresh`. (During enable transitions `is_enabling == true`, `poll_enable`
        // and causal `RefreshCapture` handshakes drive the same `RefreshScheduler`.)
        let enabled_pf_joint = if self.is_enabled { self.active_pos_force_joint() } else { None };
        if let Some(pf_j) = enabled_pf_joint {
            if rx_drained && !self.awaiting_rx_boundary && !self.is_enabling {
                if self.pos_force_feedback_valid(pf_j, self.now()) {
                    self.pf_refresh_pending = false;
                    self.pf_refresh_deferred_ticks = 0;
                } else {
                    self.pf_refresh_pending = true;
                }
            }
        }
        // Blocker 1 & Major 1: while a `ModeSwitch` transaction owns the bus (`mode_switch_active_mask != 0`),
        // Step 2 RX MUST NOT consume release TX slots on optional refresh/recovery frames or emit
        // control frames before `ModeSwitch::step` runs watchdog-first holds.
        if !self.is_enabling && self.mode_switch_active_mask == 0 {
            match self.service_refresh_scheduler(rx_stop_stamp, None, true, u64::MAX) {
                Ok(_) | Err(ArmDriverError::EstopInhibited) => {}
                Err(e) => return Err(e),
            }
        }
        let managed_recovery_cap = self.in_active_control_release();
        // When a managed release already runs Step 5 normal output every tick without joint
        // tracking (`!self.strict_mode_handover`), or is actively running Step 5 stopped/measured
        // holds (`self.last_output_was_hold`), Step 5 will immediately emit a fresh control/hold
        // frame to every eligible joint in this release (and end-of-release watchdog fallback
        // covers any deferred/absent motor). Skip redundant Step-2 recovery holds here so no motor
        // receives duplicate control frames in the same release.
        let skip_redundant_step2_recovery = managed_recovery_cap
            && self.output_ran_since_rx
            && (!self.strict_mode_handover || self.last_output_was_hold);
        if rx_drained
            && self.is_enabled
            && !self.is_enabling
            && self.mode_switch_active_mask == 0
            && !self.is_stopped_for_disable()
            && !matches!(
                self.torque_hs,
                Some(DamiaoTorque::AwaitRestGate { .. } | DamiaoTorque::Disable { .. })
            )
            && self.recovery_tx_allowed()
            && self.is_feedback_valid()
            && !(self.output_ran_since_rx && self.has_class1_transition_pending())
            && !skip_redundant_step2_recovery
            && (!managed_recovery_cap || self.has_optional_release_budget())
        {
            self.discard_staged_ctrl_tx();
            let mut any_recovery_sent = false;
            let mut recovery_flush_aborted = false;
            let n = self.motor_ids.len();
            let start = if managed_recovery_cap && n > 0 {
                self.recovery_cursor % n
            } else {
                0
            };
            for offset in 0..n {
                if managed_recovery_cap
                    && (self.release_recovery_tx_count >= DAMIAO_MAX_RECOVERY_PER_RELEASE
                        || !self.has_optional_release_budget())
                {
                    break;
                }
                let i = (start + offset) % n;
                if managed_recovery_cap && i < MAX_MODE_JOINTS && (self.release_ctrl_tx_mask & (1u32 << i)) != 0 {
                    continue;
                }
                if i < 64 && (self.disable_dispatched_mask & (1u64 << i)) != 0 {
                    continue;
                }
                // Blocker 1: NEVER transmit a recovery control frame for a joint whose mode is uncertain!
                if self.is_axis_mode_uncertain(i) {
                    continue;
                }
                if self.strict_stop_check(Some(rx_stop_stamp)).is_err() {
                    self.discard_pending_tx_stopped();
                    break;
                }
                let stale = self
                    .last_feedback_instant
                    .get(i)
                    .and_then(|&opt| opt)
                    .is_none_or(|t| self.since(t) >= Duration::from_millis(45));
                if !stale {
                    continue;
                }

                // Recovery never replays a queued motion target. When owned by `SupervisorInner`
                // (`external_pos_force_emitter == true`), `PosForce` J7 frames are emitted solely
                // by the tick epilogue (never duplicated here); standalone driver callers still
                // route through the ONE `emit_pos_force_frame` helper (`allow_freeze = false`).
                let Some(resolved_mode) = self.joint_modes.get(i).copied().flatten() else {
                    // R2 Blocker 3: NEVER invent `self.control_mode` for a joint with no confirmed mode!
                    continue;
                };
                if resolved_mode == DamiaoControlMode::Mit && !self.is_mit_hold_eligible(i) {
                    continue;
                }
                let send_res = if resolved_mode == DamiaoControlMode::PosForce {
                    if self.external_pos_force_emitter {
                        continue;
                    }
                    let Some(pf) = self.pos_force.as_ref().filter(|pf| pf.joint == i) else {
                        continue;
                    };
                    let Some(pos) = self.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE) else {
                        continue;
                    };
                    let id = self.motor_ids[i];
                    let last_raw = self.last_target_frames[i]
                        .filter(|f| f.id == u32::from(damiao::POS_FORCE_BASE_ID + id))
                        .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
                        .filter(|&r| r >= 1);
                    let recovery_vel = last_raw
                        .map(crate::model::DamiaoGripperDrive::wire_speed_f32)
                        .map_or(pf.close_speed_rad_s, |v| v.min(pf.close_speed_rad_s));
                    pos_force_emitter::emit_standalone_recovery_hold(
                        self,
                        Some(rx_stop_stamp),
                        pos,
                        recovery_vel,
                    )
                } else {
                    self.send_plain(TxKind::Recovery, Some(rx_stop_stamp), None, false, |d| {
                        let id = d.motor_ids[i];
                        let p = d.cached_states.get(i).map_or(0.0, |s| s.position);
                        match resolved_mode {
                            DamiaoControlMode::PosForce => None,
                            DamiaoControlMode::Mit if d.control_mode == DamiaoControlMode::PosVel => {
                                // A joint switched to MIT by set_joint_modes (gravity compensation):
                                // rebuild a fresh stationary MIT hold from current fresh measured state
                                // (same <= 100 ms + observed-empty receive bound as J7 PosForce) and
                                // the last MIT gains/torque instead of replaying cached bytes.
                                let pos = d.fresh_measured_position(i, POS_FORCE_FEEDBACK_MAX_AGE)?;
                                let (kp, kd, torque) = d.last_mit_params.get(i).copied().flatten()?;
                                Some(damiao::build_mit_frame(
                                    id,
                                    pos,
                                    0.0,
                                    kp,
                                    kd,
                                    torque,
                                    &d.limits[i],
                                ))
                            }
                            DamiaoControlMode::Mit => Some(damiao::build_mit_frame(
                                id,
                                p,
                                0.0,
                                d.default_kp,
                                d.default_kd,
                                0.0,
                                &d.limits[i],
                            )),
                            DamiaoControlMode::PosVel => Some(damiao::build_pos_vel_frame(
                                0x100,
                                id,
                                p,
                                d.default_vlim.abs().clamp(0.1, 4.0),
                            )),
                        }
                    })
                };
                let recovery_sent = match send_res {
                    Ok(sent_frame) => {
                        self.last_target_frames[i] = Some(sent_frame);
                        true
                    }
                    Err(TxRefusal::Deferred) => break,
                    Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged) => {
                        self.discard_pending_tx_stopped();
                        recovery_flush_aborted = true;
                        break;
                    }
                    Err(TxRefusal::BuildRefused | TxRefusal::PosForceFeedbackExpired(_)) => continue,
                    Err(TxRefusal::Transport(e)) if e.is_tx_backpressure() => {
                        if self.strict_stop_check(Some(rx_stop_stamp)).is_err() {
                            self.discard_pending_tx_stopped();
                            recovery_flush_aborted = true;
                        }
                        break;
                    }
                    Err(TxRefusal::Transport(_)) => {
                        self.mark_tx_pass_backpressured();
                        recovery_flush_aborted = true;
                        break;
                    }
                    Err(_) => false,
                };
                if recovery_sent {
                    any_recovery_sent = true;
                    self.release_recovery_tx_count = self.release_recovery_tx_count.saturating_add(1);
                    if n > 0 {
                        self.recovery_cursor = (i + 1) % n;
                    }
                    self.discard_pending_tx_stopped();
                    if !self.recovery_written_joints.contains(&i) {
                        self.recovery_written_joints.push(i);
                    }
                }
                if self.strict_stop_check(Some(rx_stop_stamp)).is_err() {
                    self.discard_pending_tx_stopped();
                    recovery_flush_aborted = true;
                    break;
                }
            }
            if any_recovery_sent && !self.can.flush_is_noop() {
                if recovery_flush_aborted
                    || self.flush_cutoff_exceeded()
                    || self.strict_stop_check(Some(rx_stop_stamp)).is_err()
                {
                    self.discard_staged_ctrl_tx();
                } else {
                    let _ = self.flush_can_and_commit();
                }
            } else if recovery_flush_aborted {
                self.discard_staged_ctrl_tx();
            }
        }
        self.output_ran_since_rx = false;
        crate::diagnostics::tick::record_tx_deferred(0, self.total_pending_tx_len());
        Ok(())
    }
}
