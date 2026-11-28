//! Damiao CAN register read/write, mapping range checks, and `CommissioningExt`.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::driver::can_servo::*;
use crate::protocol::damiao;
use crate::transport::{CanTransport, TransportError, MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS};

impl<C: CanTransport> DamiaoArmDriver<C> {
    /// Result of the startup check of the motors' mapping ranges against the configured ones.
    pub fn range_check(&self) -> Option<DriverRangeCheck> { self.range_check.clone() }

    /// Enable mandatory startup and post-BusLoss causal `CTRL_MODE` and `TIMEOUT` readback
    /// verification across all motors. When `expected` is `Some`, each `Some(counts)` entry is
    /// also matched against the observed raw `RID_TIMEOUT` counts; when `expected` is `None` (or
    /// an individual axis entry is `None`), fresh causal replies are still required while exact
    /// count comparison is skipped for that axis.
    pub fn with_timeout_verification(
        mut self,
        expected: Option<Vec<Option<u32>>>,
    ) -> Result<Self, ArmDriverError> {
        let list = match expected {
            Some(list) => {
                if list.len() != self.motor_ids.len() {
                    return Err(ArmDriverError::JointCountMismatch {
                        expected: self.motor_ids.len(),
                        actual: list.len(),
                    });
                }
                list
            }
            None => vec![None; self.motor_ids.len()],
        };
        self.expected_timeout_counts = list;
        self.timeout_check_enabled = true;
        self.timeout_verified.fill(false);
        self.startup_register_check_pending = true;
        Ok(self)
    }

    /// Opt-in expected Damiao `RID_TIMEOUT` raw counts per axis (`None` skips value matching
    /// for that axis while still requiring a fresh causal reply on startup and after BusLoss).
    pub fn with_expected_timeout_counts(self, expected: Vec<Option<u32>>) -> Result<Self, ArmDriverError> {
        self.with_timeout_verification(Some(expected))
    }

    /// Read CTRL_MODE, PMAX, VMAX and TMAX (registers 10, 21-23) from every motor and
    /// compare the ranges with the configured ones. Register reads do not change the
    /// enable state or the targets, so this is safe with the motors disabled; it sends
    /// no enable frame. A mismatch makes `set_torque(true)` refuse to arm. A motor that
    /// does not answer leaves the check `Unverified`: the configured ranges are used
    /// and gravity compensation stays refused. A joint whose CTRL_MODE is not the base
    /// mode is marked unconfirmed, so the enable handshake writes the base mode first.
    pub fn check_mapping_ranges(&mut self) -> &DriverRangeCheck {
        const RIDS: [u8; 4] = [damiao::RID_CTRL_MODE, damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX];
        let start = self.now();
        let mut joints = Vec::with_capacity(self.motor_ids.len());
        for i in 0..self.motor_ids.len() {
            let id = self.motor_ids[i];
            let base_mode = self.base_mode(i);
            let base = Self::mode_value(base_mode);
            let pre_seq = self.rx_boundary().ok();
            self.register_replies.clear();
            self.register_reply_log.clear();
            let start_stamp = self.current_stop_stamp();
            let sent = RIDS.iter().all(|&rid| {
                self.send_with_retry(TxKind::Refresh, Some(start_stamp), None, false, |_| {
                    Some(damiao::build_read_register_frame(id, rid))
                })
                .is_ok()
            });
            let _ = self.flush_can_and_commit();
            let expected: Vec<(usize, bool, u8)> = RIDS.iter().map(|&rid| (i, false, rid)).collect();
            let replies = if sent { self.await_register_replies_for(&expected, RANGE_CHECK_REPLY_TIMEOUT) } else { vec![None; RIDS.len()] };
            let l = self.limits[i];
            let configured = [l.p_max, l.v_max, l.t_max];
            let ctrl_mode = replies[0].map(|r| r.as_u32());
            let (status, reported) = Self::evaluate_range_reading(
                replies[1].map(|r| r.as_f32()),
                replies[2].map(|r| r.as_f32()),
                replies[3].map(|r| r.as_f32()),
                &configured,
            );
            self.mit_limits_verified[i] = status == RangeCheckStatus::Passed;
            // E4: Require a causal post-boundary `RID_CTRL_MODE` readback after this daemon start
            // to mark joint `i`'s hold mode verified; silent or mismatched motors stay unverified.
            let expected_id = self.expected_reply_id(i);
            let verified_ctrl = pre_seq.and_then(|pre| {
                self.register_reply_log
                    .iter()
                    .rev()
                    .find(|(r, fid, _, seq, _)| {
                        r.motor_id == id
                            && !r.write
                            && r.rid == damiao::RID_CTRL_MODE
                            && *seq > pre
                            && (base_mode != DamiaoControlMode::PosForce || *fid == expected_id)
                    })
                    .map(|(r, ..)| r.as_u32())
            });
            let verified_mode = verified_ctrl.and_then(Self::mode_from_u32);
            if verified_ctrl == Some(base) && verified_mode == Some(base_mode) {
                if base_mode == DamiaoControlMode::PosForce {
                    self.joint_modes[i] = Some(DamiaoControlMode::PosForce);
                }
                self.verified_modes[i] = Some(base_mode);
                #[cfg(not(target_arch = "wasm32"))]
                if self.shadow_staging.is_enabled() && i < crate::torque::MAX_CHANNELS {
                    let now = self.now();
                    self.shadow_staging
                        .push(crate::torque::shadow::ShadowDriverObs::ModeVerified {
                            channel: i as u8,
                            mode: damiao_mode_to_shadow_control_mode(base_mode),
                            now,
                            provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                        });
                }
            } else if let Some(m) = verified_ctrl.or(ctrl_mode) {
                if m != base {
                    tracing::warn!(
                        "Damiao joint {} (CAN ID 0x{:X}) is in CTRL_MODE {}; the base mode {} is restored before enable",
                        i + 1, id, m, base
                    );
                }
                self.joint_modes[i] = None;
                self.verified_modes[i] = None;
            } else {
                // Silent motor: never assume PosVel for holds until a causal CTRL_MODE readback succeeds.
                self.verified_modes[i] = None;
            }
            joints.push(JointRangeReading { joint: i + 1, motor_id: id, status, ctrl_mode, reported, configured });
        }
        let fmt = |v: &[f32; 3]| format!("{:.3}/{:.3}/{:.3}", v[0], v[1], v[2]);
        let mismatched: Vec<String> = joints.iter().filter(|j| j.status == RangeCheckStatus::Mismatch)
            .map(|j| format!("J{} reports PMAX/VMAX/TMAX {} (configured {})", j.joint, fmt(&j.reported.unwrap_or_default()), fmt(&j.configured))).collect();
        let silent: Vec<usize> = joints.iter().filter(|j| j.status == RangeCheckStatus::Unverified).map(|j| j.joint).collect();
        let duration_ms = self.since(start).as_millis() as u64;
        let (status, message) = if !mismatched.is_empty() {
            let m = format!("Damiao mapping range mismatch: {}; arming refused until damiao_ranges matches the motors", mismatched.join("; "));
            tracing::error!("{m}");
            (RangeCheckStatus::Mismatch, m)
        } else if !silent.is_empty() {
            let m = format!("Damiao joints {silent:?} did not answer the range read; using the configured ranges unverified");
            tracing::warn!("{m}");
            (RangeCheckStatus::Unverified, m)
        } else {
            let m = format!("Damiao mapping ranges of {} motors match the configuration", joints.len());
            tracing::info!("{m} ({duration_ms} ms)");
            (RangeCheckStatus::Passed, m)
        };
        self.range_check.insert(DriverRangeCheck { status, message, joints, duration_ms })
    }

    pub(in crate::driver::can_servo) fn range_value_matches(got: f32, want: f32) -> bool {
        got.is_finite() && (got - want).abs() <= 0.01 + want.abs() * 1e-3
    }

    /// Evaluates startup and ticked mapping-range replies with a single shared rule:
    /// any received out-of-tolerance or non-finite register value is immediately a `Mismatch`
    /// even when another register reply was missing; only a complete in-tolerance triple is
    /// `Passed`; partial/missing replies whose received values all match remain `Unverified`.
    pub(in crate::driver::can_servo) fn evaluate_range_reading(
        p: Option<f32>,
        v: Option<f32>,
        t: Option<f32>,
        configured: &[f32; 3],
    ) -> (RangeCheckStatus, Option<[f32; 3]>) {
        let any_mismatch = p.is_some_and(|g| !Self::range_value_matches(g, configured[0]))
            || v.is_some_and(|g| !Self::range_value_matches(g, configured[1]))
            || t.is_some_and(|g| !Self::range_value_matches(g, configured[2]));
        let reported = match (p, v, t) {
            (Some(p), Some(v), Some(t)) => Some([p, v, t]),
            _ => None,
        };
        let status = if any_mismatch {
            RangeCheckStatus::Mismatch
        } else if reported.is_some() {
            RangeCheckStatus::Passed
        } else {
            RangeCheckStatus::Unverified
        };
        (status, reported)
    }

    #[allow(dead_code)]
    pub(in crate::driver::can_servo) fn ranges_match(got: &[f32; 3], want: &[f32; 3]) -> bool {
        got.iter().zip(want).all(|(&g, &w)| Self::range_value_matches(g, w))
    }

    /// Poll feedback until every `(joint, write, rid)` has a reply or the timeout passes.
    pub(in crate::driver::can_servo) fn await_register_replies(&mut self, expected: &[(usize, bool, u8)]) -> Vec<Option<damiao::RegisterReply>> {
        self.await_register_replies_for(expected, REGISTER_REPLY_TIMEOUT)
    }

    pub(in crate::driver::can_servo) fn await_register_replies_for(&mut self, expected: &[(usize, bool, u8)], timeout: Duration) -> Vec<Option<damiao::RegisterReply>> {
        let start = self.now();
        loop {
            let _ = self.read_joint_states();
            if self.motor_energized_mask != 0 {
                let _ = self.service_energized_watchdog_holds(
                    WATCHDOG_HOLD_PROACTIVE_INTERVAL,
                    None,
                    0,
                    false,
                );
            }
            let found: Vec<Option<damiao::RegisterReply>> = expected.iter().map(|&(i, write, rid)| {
                let id = self.motor_ids[i];
                self.register_replies.iter().rev().copied()
                    .find(|r| r.motor_id == id && r.write == write && r.rid == rid)
            }).collect();
            if found.iter().all(Option::is_some) || self.since(start) >= timeout {
                return found;
            }
            self.pause(Duration::from_micros(500));
        }
    }

    /// Read PMAX / VMAX / TMAX of `joints` and require each to equal that joint's range:
    /// an MIT frame encoded with the wrong TMAX would scale every torque command.
    pub(in crate::driver::can_servo) fn verify_mit_limits(&mut self, joints: &[usize], expected_stamp: StopStamp) -> Result<(), ArmDriverError> {
        const RIDS: [u8; 3] = [damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX];
        for &i in joints {
            self.strict_stop_check(Some(expected_stamp))?;
            self.register_replies.clear();
            for rid in RIDS {
                self.send_with_retry(TxKind::Refresh, Some(expected_stamp), None, false, |d| {
                    Some(damiao::build_read_register_frame(d.motor_ids[i], rid))
                })
                .map_err(Self::refusal_to_driver_err)?;
            }
            self.strict_stop_check(Some(expected_stamp))?;
            self.flush_can_and_commit()?;
            let expected: Vec<(usize, bool, u8)> = RIDS.map(|rid| (i, false, rid)).to_vec();
            let replies = self.await_register_replies(&expected);
            let want = [self.limits[i].p_max, self.limits[i].v_max, self.limits[i].t_max];
            let got: Vec<Option<f32>> = (0..3).map(|r| replies[r].map(|reply| reply.as_f32())).collect();
            if got.iter().any(Option::is_none) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) did not answer the PMAX/VMAX/TMAX register read", i + 1, self.motor_ids[i])));
            }
            let got: Vec<f32> = got.into_iter().flatten().collect();
            if !Self::ranges_match(&[got[0], got[1], got[2]], &want) {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Joint {} (CAN ID 0x{:X}) reports PMAX {:.3} VMAX {:.3} TMAX {:.3}; the driver maps MIT frames with {:.3} / {:.3} / {:.3}",
                    i + 1, self.motor_ids[i], got[0], got[1], got[2], want[0], want[1], want[2])));
            }
            self.mit_limits_verified[i] = true;
        }
        Ok(())
    }

    /// Pure RX-only, register-recording queue drain until empty (or bounded frame/time limit).
    /// Never runs `service_refresh_scheduler` or transmits any `0xCC` / recovery frame.
    pub(in crate::driver::can_servo) fn drain_rx_only_record_registers(&mut self) -> Result<bool, ArmDriverError> {
        let max_frames = self.remaining_rx_frame_limit();
        let t0 = self.now();
        let mut frames_processed = 0usize;
        let mut rx_drained = false;
        let mut hit_err: Option<TransportError> = None;
        self.last_read_drained = false;
        while frames_processed < max_frames && !self.rx_over(t0) {
            let attempt = self.now();
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames_processed += 1;
                    self.process_rx_frame(&frame, true, rx_time);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    rx_drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(e) => {
                    hit_err = Some(e);
                    break;
                }
            }
        }
        self.account_rx_pass(frames_processed, !rx_drained && hit_err.is_none());
        if let Some(e) = hit_err {
            return Err(e.into());
        }
        Ok(rx_drained)
    }

    /// Bounded reply wait (MIT transactions): poll until every `(joint, write, rid)` has a reply
    /// dequeued after the boundary `pre`, or REGISTER_REPLY_TIMEOUT passes.
    pub(in crate::driver::can_servo) fn await_replies_after(&mut self, expected: &[(usize, bool, u8)], sent: u64) {
        let mut exclude_mask = 0u32;
        for &(i, write, rid) in expected {
            if write && rid == damiao::RID_CTRL_MODE && i < MAX_MODE_JOINTS {
                exclude_mask |= 1u32 << i;
            }
        }
        let start = self.now();
        loop {
            let _ = self.read_joint_states();
            if self.motor_energized_mask != 0 && self.mit_fence.is_none() {
                let _ = self.service_energized_watchdog_holds(
                    WATCHDOG_HOLD_PROACTIVE_INTERVAL,
                    None,
                    exclude_mask,
                    false,
                );
            }
            let all = expected.iter().all(|&(i, write, rid)| {
                let id = self.motor_ids[i];
                self.register_reply_log.iter().any(|(r, _, _, seq, _)| r.motor_id == id && r.write == write && r.rid == rid && *seq > sent)
            });
            if all || self.since(start) >= REGISTER_REPLY_TIMEOUT { return; }
            self.pause(Duration::from_micros(500));
        }
    }

    /// Read u32 / f32 registers `rids` of joint `i`: requests sent after the log is cleared,
    /// replies matched by motor ID, read opcode and RID, newer than the request. Returns
    /// (value, reply CAN ID) per RID.
    pub(in crate::driver::can_servo) fn read_registers_fresh(
        &mut self,
        i: usize,
        rids: &[u8],
        expected_stamp: StopStamp,
        allow_latched: bool,
    ) -> Result<Vec<([u8; 4], u32)>, ArmDriverError> {
        if allow_latched {
            self.pending_stop_check(Some(expected_stamp))?;
        } else {
            self.strict_stop_check(Some(expected_stamp))?;
        }
        let id = self.motor_ids[i];
        let sent = self.rx_boundary()?;
        if allow_latched {
            self.pending_stop_check(Some(expected_stamp))?;
        } else {
            self.strict_stop_check(Some(expected_stamp))?;
        }
        self.register_replies.clear();
        self.register_reply_log.clear();
        for &rid in rids {
            if !allow_latched {
                self.strict_stop_check(Some(expected_stamp))?;
            }
            self.send_with_retry(TxKind::Refresh, Some(expected_stamp), None, false, |_| {
                Some(damiao::build_read_register_frame(id, rid))
            })
            .map_err(Self::refusal_to_driver_err)?;
        }
        if allow_latched {
            self.pending_stop_check(Some(expected_stamp))?;
        } else {
            self.strict_stop_check(Some(expected_stamp))?;
        }
        self.flush_can_and_commit()?;
        let expected: Vec<(usize, bool, u8)> = rids.iter().map(|&rid| (i, false, rid)).collect();
        self.await_replies_after(&expected, sent);
        rids.iter().map(|&rid| {
            self.register_reply_log.iter().rev()
                .find(|(r, _, _, seq, _)| r.motor_id == id && !r.write && r.rid == rid && *seq > sent)
                .map(|(r, fid, _, _, _)| (r.value, *fid))
                .ok_or_else(|| ArmDriverError::HardwareFault(format!("Joint {} (CAN ID 0x{id:X}) did not answer the read of register {rid}", i + 1)))
        }).collect()
    }

    pub(in crate::driver::can_servo) fn identity_of(&mut self, i: usize) -> Result<crate::mit::MotorIdentity, ArmDriverError> {
        const RIDS: [u8; 8] = [damiao::RID_ESC_ID, damiao::RID_MST_ID, damiao::RID_SW_VER, damiao::RID_CTRL_MODE,
            damiao::RID_PMAX, damiao::RID_VMAX, damiao::RID_TMAX, damiao::RID_TIMEOUT];
        let start_stamp = self.current_stop_stamp();
        let v = self.read_registers_fresh(i, &RIDS, start_stamp, true)?;
        let reply_id = v[0].1;
        if let Some((_, other)) = v.iter().find(|(_, fid)| *fid != reply_id) {
            return Err(ArmDriverError::HardwareFault(format!("Joint {}: register replies arrived on CAN IDs 0x{reply_id:X} and 0x{other:X}", i + 1)));
        }
        let u = |k: usize| u32::from_le_bytes(v[k].0);
        let f = |k: usize| f32::from_le_bytes(v[k].0);
        let timeout_raw = u(7);
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() && i < crate::torque::MAX_CHANNELS {
            let now = self.now();
            self.shadow_staging
                .push(crate::torque::shadow::ShadowDriverObs::TimeoutReadback {
                    channel: i as u8,
                    raw_reg: timeout_raw,
                    now,
                });
        }
        Ok(crate::mit::MotorIdentity { joint: i + 1, motor_id: self.motor_ids[i], reply_id, esc_id: u(0), mst_id: u(1), sw_ver: u(2),
            ctrl_mode: u(3), pmax: f(4), vmax: f(5), tmax: f(6), timeout_raw })
    }

    /// One joint of `commission_motor_timeout`; `written` is set once the RAM write was sent.
    pub(in crate::driver::can_servo) fn commission_one(&mut self, i: usize, counts: u32, written: &mut bool) -> Result<crate::mit::TimeoutCommissioning, ArmDriverError> {
        {
            let start_stamp = self.current_stop_stamp();
            self.strict_stop_check(Some(start_stamp))?;
            let id = self.motor_ids[i];
            let before = self.identity_of(i)?;
            if before.reply_id != before.mst_id {
                return Err(ArmDriverError::HardwareFault(format!("joint {}: register replies on 0x{:X} but MST_ID {}", i + 1, before.reply_id, before.mst_id)));
            }
            // RAM write (0x55), checked acknowledgement
            let sent = self.rx_boundary()?;
            self.strict_stop_check(Some(start_stamp))?;
            self.register_replies.clear();
            self.register_reply_log.clear();
            // uncertain from the moment the transport accepted the write (a write that never left,
            // WouldBlock until the retries ran out, changed nothing)
            self.send_with_retry(TxKind::ModeWrite, Some(start_stamp), None, false, |_| {
                Some(damiao::build_write_register_u32_frame(id, damiao::RID_TIMEOUT, counts))
            })
            .map_err(Self::refusal_to_driver_err)?;
            *written = true;
            self.strict_stop_check(Some(start_stamp))?;
            self.flush_can_and_commit()?;
            self.await_replies_after(&[(i, true, damiao::RID_TIMEOUT)], sent);
            let ack = self.register_reply_log.iter().rev()
                .find(|(r, _, _, seq, _)| r.motor_id == id && r.write && r.rid == damiao::RID_TIMEOUT && *seq > sent).copied();
            let Some((ack, ack_id, _, _, _)) = ack else {
                return Err(ArmDriverError::HardwareFault(format!("joint {} (CAN ID 0x{id:X}) did not acknowledge the TIMEOUT write", i + 1)));
            };
            if ack_id != before.mst_id || ack.as_u32() != counts {
                return Err(ArmDriverError::HardwareFault(format!("joint {}: TIMEOUT write acknowledged with {} on 0x{ack_id:X} (wrote {counts}, MST_ID {})",
                    i + 1, ack.as_u32(), before.mst_id)));
            }
            // separate read: fresh motor ID, reply ID, opcode, RID and value
            let readback = self.identity_of(i)?;
            let same = readback.motor_id == before.motor_id && readback.reply_id == before.reply_id && readback.esc_id == before.esc_id
                && readback.mst_id == before.mst_id && readback.sw_ver == before.sw_ver;
            if !same || readback.timeout_raw != counts {
                return Err(ArmDriverError::HardwareFault(format!("joint {}: TIMEOUT readback {} (wrote {counts}) or identity changed", i + 1, readback.timeout_raw)));
            }
            tracing::warn!("Damiao joint {} (CAN ID 0x{id:X}): TIMEOUT {} -> {counts} counts (RAM)", i + 1, before.timeout_raw);
            Ok(crate::mit::TimeoutCommissioning { before, written_raw: counts, ack_raw: ack.as_u32(), readback,
                unit_us: damiao::TIMEOUT_UNIT_US_DOCUMENTED, unit_verified: false })
        }
    }

    /// Valid reply CAN arbitration ID for a register readback on joint `j`:
    /// PosForce axes and profiles with an explicit `configured_master_id` require exact
    /// `expected_reply_id(j)`; ordinary axes without an explicit `configured_master_id` accept
    /// their own `expected_reply_id(j)` (`0x10 + motor_id`), own `motor_id`, or shared master ID
    /// (`0` / `self.master_id`), rejecting another motor's `MST_ID`.
    #[inline]
    pub(in crate::driver::can_servo) fn valid_register_reply_id(&self, j: usize, reply_id: u32) -> bool {
        let expected = self.expected_reply_id(j);
        if self.base_mode(j) == DamiaoControlMode::PosForce || self.configured_master_id.is_some() {
            reply_id == expected
        } else {
            let motor_id = u32::from(self.motor_ids[j]);
            reply_id == expected || reply_id == motor_id || reply_id == self.master_id || reply_id == 0
        }
    }

}

impl<C: CanTransport> CommissioningExt for DamiaoArmDriver<C> {
    fn read_motor_identity(&mut self, joints: &[usize]) -> Result<Vec<crate::mit::MotorIdentity>, ArmDriverError> {
        let n = self.motor_ids.len();
        if let Some(&j) = joints.iter().find(|&&j| j >= n) { return Err(ArmDriverError::JointCountMismatch { expected: n, actual: j + 1 }); }
        joints.iter().map(|&i| self.identity_of(i)).collect()
    }

    fn commission_motor_timeout(&mut self, joints: &[usize], counts: u32) -> Result<Vec<crate::mit::TimeoutCommissioning>, crate::mit::TimeoutBatchError> {
        let none_attempted = |e: ArmDriverError| crate::mit::TimeoutBatchError { not_attempted: joints.to_vec(), ..e.into() };
        let n = self.motor_ids.len();
        if let Some(&j) = joints.iter().find(|&&j| j >= n) { return Err(none_attempted(ArmDriverError::JointCountMismatch { expected: n, actual: j + 1 })); }
        if counts == 0 || counts > crate::mit::MAX_TIMEOUT_COUNTS {
            return Err(none_attempted(ArmDriverError::Protocol(format!("TIMEOUT counts must be in 1..={}", crate::mit::MAX_TIMEOUT_COUNTS))));
        }
        if self.is_enabled || self.is_enabling {
            return Err(none_attempted(ArmDriverError::Protocol("the TIMEOUT commissioning needs torque off".into())));
        }
        for &i in joints {
            let fresh = !self.awaiting_rx_boundary
                && self.last_feedback_instant[i].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
            if !fresh || self.motor_status[i] != 0 {
                return Err(none_attempted(ArmDriverError::HardwareFault(format!("joint {} (CAN ID 0x{:X}) is not confirmed disabled (state {})",
                    i + 1, self.motor_ids[i], self.motor_status[i]))));
            }
        }
        let mut out = Vec::with_capacity(joints.len());
        for (k, &i) in joints.iter().enumerate() {
            let mut written = false;
            let res = self.commission_one(i, counts, &mut written);
            match res {
                Ok(c) => out.push(c),
                Err(error) => return Err(crate::mit::TimeoutBatchError { applied: out, uncertain: if written { vec![i] } else { Vec::new() },
                    not_attempted: joints[k + 1..].to_vec(), error }),
            }
        }
        Ok(out)
    }

    fn set_zero_position(&mut self, joint: usize) -> Result<(), ArmDriverError> {
        let start_stamp = self.current_stop_stamp();
        self.strict_stop_check(Some(start_stamp))?;
        let n = self.motor_ids.len();
        if joint >= n { return Err(ArmDriverError::JointCountMismatch { expected: n, actual: joint + 1 }); }
        let id = self.motor_ids[joint];
        if self.is_enabled || self.is_enabling {
            return Err(ArmDriverError::Protocol(format!("joint {} (CAN ID 0x{id:X}): set-zero needs torque off", joint + 1)));
        }
        let fresh = !self.awaiting_rx_boundary
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return Err(ArmDriverError::HardwareFault(format!(
                "joint {} (CAN ID 0x{id:X}) is not confirmed disabled (state {}, feedback age {:?})",
                joint + 1, self.motor_status[joint], self.last_feedback_instant[joint].map(|t| self.since(t)))));
        }
        self.send_with_retry(TxKind::ModeWrite, Some(start_stamp), None, false, |_| {
            Some(damiao::build_set_zero_frame(id))
        })
        .map_err(Self::refusal_to_driver_err)?;
        self.strict_stop_check(Some(start_stamp))?;
        self.flush_can_and_commit()?;
        tracing::warn!("Damiao joint {} (CAN ID 0x{id:X}): set-zero sent at reading {:.4} rad", joint + 1, self.cached_states[joint].position);
        Ok(())
    }

    fn supports_nonblocking_commissioning(&self) -> bool {
        self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush()
    }

    fn supports_nonblocking_motor_zero(&self) -> bool {
        self.can.supports_nonblocking_rt() && self.can.supports_bounded_flush()
    }

    fn commissioning_motor_id(&self, joint: usize) -> Option<u16> {
        self.motor_ids.get(joint).copied()
    }

    fn commissioning_drain_rx_step(
        &mut self,
        max_frames: u16,
        deadline: Instant,
    ) -> Result<(u16, bool), ArmDriverError> {
        if !self.can.supports_nonblocking_rt() {
            return Err(ArmDriverError::Protocol("non-blocking CAN RX unsupported".into()));
        }
        let limit = usize::from(self.clamp_release_rx_limit(max_frames));
        let mut frames = 0u16;
        let mut drained = false;
        self.last_read_drained = false;
        while usize::from(frames) < limit && self.now() < deadline {
            let attempt = self.now();
            match self.receive_with_rx_time_timed() {
                Ok((frame, rx_time)) => {
                    frames = frames.saturating_add(1);
                    self.process_rx_frame(&frame, true, rx_time);
                }
                Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    drained = true;
                    self.mark_rx_empty(attempt);
                    break;
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok((frames, drained))
    }

    fn commissioning_clear_register_log(&mut self) {
        self.comm_reg_reply_len = 0;
        self.comm_reg_reply_overflow = false;
    }

    fn commissioning_try_send_register_read(
        &mut self,
        joint: usize,
        rid: u8,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        let start_stamp = self.current_stop_stamp();
        match self.send_with_commissioning_gate(
            TxKind::Refresh,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_read_register_frame(id, rid)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::mit::CommissioningTxStatus::Refused("register read transmit failed"),
        }
    }

    fn commissioning_try_send_rest_probe(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        if self.is_enabled || self.is_enabling {
            return crate::mit::CommissioningTxStatus::Refused(
                "torque must be off for TIMEOUT commissioning rest-pose probe",
            );
        }
        let start_stamp = self.current_stop_stamp();
        match self.send_with_commissioning_gate(
            TxKind::Refresh,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_refresh_frame(id)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => {
                crate::mit::CommissioningTxStatus::Refused("rest-pose probe transmit failed")
            }
        }
    }

    fn commissioning_try_send_timeout_write(
        &mut self,
        joint: usize,
        counts: u32,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        if counts == 0 || counts > crate::mit::MAX_TIMEOUT_COUNTS {
            return crate::mit::CommissioningTxStatus::Refused("TIMEOUT counts out of range");
        }
        if self.is_enabled || self.is_enabling {
            return crate::mit::CommissioningTxStatus::Refused("torque must be off for TIMEOUT commissioning");
        }
        let fresh = !self.awaiting_rx_boundary
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return crate::mit::CommissioningTxStatus::Refused("joint is not confirmed disabled");
        }
        let start_stamp = self.current_stop_stamp();
        // RAM write opcode 0x55 only (NEVER flash store 0xAA).
        match self.send_with_commissioning_gate(
            TxKind::ModeWrite,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_write_register_u32_frame(id, damiao::RID_TIMEOUT, counts)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::mit::CommissioningTxStatus::Refused("TIMEOUT write transmit failed"),
        }
    }

    fn commissioning_find_register_reply(
        &self,
        joint: usize,
        write: bool,
        rid: u8,
        after_tx_seq: u64,
        min_rx_at: Instant,
        max_rx_at: Option<Instant>,
        tx_offset_ns: Option<i128>,
    ) -> crate::mit::CommissioningRegMatch {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningRegMatch::Pending;
        };
        if self.comm_reg_reply_overflow {
            return crate::mit::CommissioningRegMatch::Ambiguous;
        }
        let mut matched: Option<crate::mit::CommissioningRegReply> = None;
        for slot in &self.comm_reg_replies[..self.comm_reg_reply_len] {
            let Some(rep) = *slot else { continue };
            if rep.motor_id != id || rep.write != write || rep.rid != rid {
                continue;
            }
            if rep.rx_seq <= after_tx_seq {
                continue;
            }
            // B1: Any reply for `(motor_id, op, rid)` dequeued AFTER the empty-queue TX boundary
            // whose timestamp precedes `min_rx_at` or whose realtime-to-monotonic clock offset
            // drifted proves delayed/ambiguous in-flight bus traffic -> fail closed!
            if rep.rx_at < min_rx_at {
                return crate::mit::CommissioningRegMatch::Ambiguous;
            }
            let offset_ok = match (tx_offset_ns, rep.decode_offset_ns) {
                (Some(w_off), Some(d_off)) => {
                    (d_off - w_off).abs() <= MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS
                }
                (None, None) => true,
                _ => false,
            };
            if !offset_ok {
                return crate::mit::CommissioningRegMatch::Ambiguous;
            }
            // Multiple replies for the same `(motor_id, op, rid)` in one request window ->
            // cannot distinguish delayed stale traffic from the current request -> fail closed!
            if matched.is_some() {
                return crate::mit::CommissioningRegMatch::Ambiguous;
            }
            // Blocker 4: Never accept a late reply whose arrival timestamp reached/passed the
            // request expiry deadline (`max_rx_at`).
            if max_rx_at.is_some_and(|dl| rep.rx_at >= dl) {
                continue;
            }
            matched = Some(rep);
        }
        match matched {
            Some(rep) => crate::mit::CommissioningRegMatch::Matched(rep),
            None => crate::mit::CommissioningRegMatch::Pending,
        }
    }

    fn commissioning_disable_step(
        &mut self,
        started_at: Instant,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningDisableStep {
        let n = self.motor_ids.len().min(64);
        let all_mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
        }
        let start_stamp = self.current_stop_stamp();
        let tx_kind = TxKind::DisableRequested { started_at };
        let mut sent_any = false;
        let mut wait_evidence = false;
        let mut last_tx_seq = self.rx_seq;
        let mut last_tx_started = self.now();
        let mut last_tx_offset = None;
        let mut early_ret: Option<crate::mit::CommissioningDisableStep> = None;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_mask = crate::torque::ChannelMask::EMPTY;
        #[cfg(not(target_arch = "wasm32"))]
        let mut deferred_refusal_outcomes =
            [crate::torque::MotorWriteOutcome::NotTargeted; crate::torque::MAX_CHANNELS];
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = true;
            self.shadow_disable_pass_sent_mask = self.disable_dispatched_mask;
        }
        for idx in 0..n {
            if (self.disable_dispatched_mask & (1u64 << idx)) != 0 {
                continue;
            }
            if self.now() >= deadline {
                break;
            }
            let id = self.motor_ids[idx];
            match self.send_with_commissioning_gate(
                tx_kind,
                Some(start_stamp),
                Some(Deadline { at: deadline }),
                pre_tx_gate,
                |_| Some(damiao::build_disable_frame(id)),
            ) {
                Ok((_, tx_seq, tx_started, tx_offset)) => {
                    sent_any = true;
                    last_tx_seq = tx_seq;
                    last_tx_started = tx_started;
                    last_tx_offset = tx_offset;
                    self.disable_dispatched_mask |= 1u64 << idx;
                }
                Err(Ok(code)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    if sent_any && !self.can.flush_is_noop() && !self.flush_cutoff_exceeded() {
                        let _ = self.flush_can_and_commit();
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Revoked(code));
                    break;
                }
                Err(Err(TxRefusal::DisableWaitEvidence)) => {
                    wait_evidence = true;
                }
                Err(Err(TxRefusal::TorqueOffWouldDrop)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Refused(
                        "ARM_NOT_AT_RESTING_POSE",
                    ));
                    break;
                }
                Err(Err(TxRefusal::DisableRequestExpired)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Refused(
                        "disable request expired",
                    ));
                    break;
                }
                Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::Deferred;
                    }
                    break;
                }
                Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => break,
                Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Revoked("STOPPED"));
                    break;
                }
                Err(Err(r)) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    if !matches!(r, TxRefusal::Transport(_)) && idx < crate::torque::MAX_CHANNELS {
                        deferred_refusal_mask.insert(idx as u8);
                        deferred_refusal_outcomes[idx] =
                            crate::torque::MotorWriteOutcome::RejectedStale;
                    }
                    early_ret = Some(crate::mit::CommissioningDisableStep::Refused(
                        "disable transmit failed",
                    ));
                    break;
                }
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.defer_disable_shadow = false;
            if self.shadow_staging.is_enabled() && !deferred_refusal_mask.is_empty() {
                PendingDisableShadowBatch::merge_batch(
                    &mut self.pending_disable_shadow,
                    PendingDisableShadowBatch {
                        kind: crate::torque::WriteKind::DisableRequested,
                        target_mask: deferred_refusal_mask,
                        outcomes: deferred_refusal_outcomes,
                        ambiguous_gap: false,
                        write_done: None,
                    },
                );
            }
        }
        if let Some(step) = early_ret {
            #[cfg(not(target_arch = "wasm32"))]
            self.flush_pending_disable_shadow_now();
            return step;
        }
        if sent_any && !self.can.flush_is_noop() {
            if self.flush_cutoff_exceeded() || self.now() >= deadline {
                #[cfg(not(target_arch = "wasm32"))]
                self.flush_pending_disable_shadow_now();
                return crate::mit::CommissioningDisableStep::InProgress {
                    dispatched_mask: self.disable_dispatched_mask,
                };
            }
            if self.flush_can_and_commit().is_err() {
                #[cfg(not(target_arch = "wasm32"))]
                self.flush_pending_disable_shadow_now();
                return crate::mit::CommissioningDisableStep::Refused("disable flush failed");
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.flush_pending_disable_shadow_now();
        if (self.disable_dispatched_mask & all_mask) == all_mask {
            self.disable_dispatched_mask = 0;
            crate::mit::CommissioningDisableStep::AllDispatched {
                boundary_seq: last_tx_seq,
                tx_started: last_tx_started,
                tx_offset_ns: last_tx_offset,
            }
        } else if wait_evidence && !sent_any {
            crate::mit::CommissioningDisableStep::WaitEvidence
        } else {
            crate::mit::CommissioningDisableStep::InProgress {
                dispatched_mask: self.disable_dispatched_mask,
            }
        }
    }

    fn commissioning_try_send_set_zero(
        &mut self,
        joint: usize,
        deadline: Instant,
        pre_tx_gate: &mut dyn FnMut() -> Result<(), &'static str>,
    ) -> crate::mit::CommissioningTxStatus {
        let Some(&id) = self.motor_ids.get(joint) else {
            return crate::mit::CommissioningTxStatus::Refused("joint out of range");
        };
        if self.is_enabled || self.is_enabling {
            return crate::mit::CommissioningTxStatus::Refused("set-zero needs torque off");
        }
        let fresh = !self.awaiting_rx_boundary
            && self.last_feedback_instant[joint].is_some_and(|t| self.since(t) <= ZERO_FEEDBACK_MAX_AGE);
        if !fresh || self.motor_status[joint] != 0 {
            return crate::mit::CommissioningTxStatus::Refused("joint is not confirmed disabled");
        }
        let start_stamp = self.current_stop_stamp();
        match self.send_with_commissioning_gate(
            TxKind::ModeWrite,
            Some(start_stamp),
            Some(Deadline { at: deadline }),
            pre_tx_gate,
            |_| Some(damiao::build_set_zero_frame(id)),
        ) {
            Ok((_, tx_seq, tx_started, tx_offset_ns)) => {
                if !self.can.flush_is_noop() {
                    if self.flush_cutoff_exceeded() || self.now() >= deadline {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                    if self.flush_can_and_commit().is_err() {
                        return crate::mit::CommissioningTxStatus::SentFlushFailed {
                            tx_seq,
                            tx_started,
                            tx_offset_ns,
                        };
                    }
                }
                crate::mit::CommissioningTxStatus::Sent {
                    tx_seq,
                    tx_started,
                    tx_offset_ns,
                }
            }
            Err(Ok(code)) => crate::mit::CommissioningTxStatus::Revoked(code),
            Err(Err(TxRefusal::Deferred | TxRefusal::DeadlinePassed)) => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::Transport(e))) if e.is_tx_backpressure() => {
                crate::mit::CommissioningTxStatus::Backpressure
            }
            Err(Err(TxRefusal::StopInhibited | TxRefusal::StopStampChanged)) => {
                crate::mit::CommissioningTxStatus::Revoked("STOPPED")
            }
            Err(Err(_)) => crate::mit::CommissioningTxStatus::Refused("set-zero transmit failed"),
        }
    }

    fn commissioning_raw_feedback(&self, joint: usize) -> Option<crate::mit::CommissioningFeedback> {
        let s = self.cached_states.get(joint)?;
        let rx_at = self.last_feedback_instant.get(joint).copied().flatten()?;
        let rx_seq = self.last_feedback_seq.get(joint).copied()?;
        let motor_status = self.motor_status.get(joint).copied()?;
        let queue_bound = self.feedback_queue_bound.get(joint).copied().flatten();
        Some(crate::mit::CommissioningFeedback {
            raw_position: s.position,
            joint_position: s.position,
            velocity: s.velocity,
            motor_status,
            rx_seq,
            rx_at,
            queue_bound,
        })
    }
}

/// Time allowed for register replies (mode switch, range readback).
pub(in crate::driver) const REGISTER_REPLY_TIMEOUT: Duration = Duration::from_millis(50);
