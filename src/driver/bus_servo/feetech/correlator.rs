//! Feetech STS3215 half-duplex RX framing, echo drain, line-poisoning, and reply correlation.

#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::protocol::feetech;
#[allow(unused_imports)]
use crate::transport::{Transport, TransportError};

impl<T: Transport> FeetechArmDriver<T> {
    #[inline]
    pub(in crate::driver::bus_servo) fn poison_line(&mut self) {
        let now = self.clock_now();
        self.line_poisoned = true;
        self.line_poison_quiet_since = Some(now);
        #[cfg(not(target_arch = "wasm32"))]
        if self.shadow_staging.is_enabled() {
            self.shadow_staging
                .push(crate::torque::shadow::ShadowDriverObs::SerialTimeout { device: 0, now });
        }
    }

    #[inline]
    #[cfg(not(target_arch = "wasm32"))]
    pub(in crate::driver::bus_servo) fn stage_feetech_causal_sample(
        &mut self,
        joint_idx: usize,
        pkt_rx_at: Instant,
        decode_now: Instant,
        req_written_at: Option<Instant>,
        after_motion: bool,
    ) {
        if self.shadow_staging.is_enabled() && joint_idx < crate::torque::MAX_CHANNELS {
            let sample_id = self.shadow_staging.next_sample_id();
            self.shadow_staging
                .push(crate::torque::shadow::ShadowDriverObs::CausalSample {
                    channel: joint_idx as u8,
                    sample_id,
                    stamp: crate::torque::shadow::RxStampKind::Valid {
                        rx_at: pkt_rx_at,
                        drift_ns: 0,
                        live_post_settle: after_motion,
                    },
                    decode_now,
                    position_rad: self.cached_states[joint_idx].position,
                    velocity_rad_s: Some(self.cached_states[joint_idx].velocity),
                    temperature_c: Some(self.cached_states[joint_idx].temperature),
                    reported_enabled: self.reported_enable[joint_idx].map(|v| v == 1),
                    correlated_after_write_done: if after_motion {
                        self.last_motion_write_at[joint_idx]
                    } else {
                        None
                    },
                    poll_issued: req_written_at,
                    provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                });
        }
    }

    pub(in crate::driver::bus_servo) fn clear_rx(&mut self) {
        self.rx_buffer.clear();
        self.rx_timestamps.clear();
    }

    #[inline]
    pub(in crate::driver::bus_servo) fn push_rx(&mut self, bytes: &[u8], rx_at: Instant) {
        self.rx_buffer.extend_from_slice(bytes);
        self.rx_timestamps.resize(self.rx_buffer.len(), rx_at);
    }

    #[inline]
    pub(in crate::driver::bus_servo) fn drain_rx_prefix(&mut self, n: usize) {
        let count = n.min(self.rx_buffer.len());
        self.rx_buffer.drain(..count);
        let ts_count = count.min(self.rx_timestamps.len());
        self.rx_timestamps.drain(..ts_count);
    }

    /// Continuous quiet period (no bytes) required to declare a poisoned half-duplex serial line idle (S2).
    const LINE_IDLE_QUIET: Duration = Duration::from_millis(20);
    /// Upper bound for draining a poisoned line before failing closed (S2).
    const LINE_IDLE_MAX_WAIT: Duration = Duration::from_millis(80);

    /// Non-blocking poison-drain check for bounded `read_joint_states` polls: drains any pending
    /// bytes (resetting the 20 ms silence timer on any byte), clearing `line_poisoned` ONLY once
    /// the line has remained continuously silent for [`Self::LINE_IDLE_QUIET`] (20 ms).
    pub(in crate::driver::bus_servo) fn poll_poisoned_line_idle(&mut self) -> bool {
        if !self.line_poisoned {
            return true;
        }
        self.clear_rx();
        let mut discard_buf = [0u8; 64];
        let mut saw_bytes = false;
        for _ in 0..8 {
            match self.transport.read(&mut discard_buf) {
                Ok(n) if n > 0 => {
                    saw_bytes = true;
                    self.line_poison_quiet_since = Some(self.clock_now());
                }
                _ => break,
            }
        }
        if saw_bytes {
            return false;
        }
        let now = self.clock_now();
        let quiet_since = *self.line_poison_quiet_since.get_or_insert(now);
        if self.elapsed_since(quiet_since) >= Self::LINE_IDLE_QUIET {
            self.line_poisoned = false;
            self.line_poison_quiet_since = None;
            self.transport.clear_input_buffer();
            self.clear_rx();
            true
        } else {
            false
        }
    }

    /// Drains and discards all RX bytes until the serial line remains continuously quiet for
    /// [`Self::LINE_IDLE_QUIET`] (20 ms), clearing `line_poisoned`. Fails closed if late traffic
    /// never subsides within [`Self::LINE_IDLE_MAX_WAIT`].
    pub(in crate::driver::bus_servo) fn discard_until_idle(&mut self) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.transport.clear_input_buffer();
        self.clear_rx();
        let start = self.clock_now();
        let mut quiet_since = self.line_poison_quiet_since.unwrap_or(start);
        let mut discard_buf = [0u8; 64];
        loop {
            let before_read = self.clock_now();
            match self.transport.read(&mut discard_buf) {
                Ok(n) if n > 0 => {
                    let now = self.clock_now();
                    quiet_since = now;
                    self.line_poison_quiet_since = Some(now);
                    if self.elapsed_since(start) >= Self::LINE_IDLE_MAX_WAIT {
                        return Err(ArmDriverError::ProtocolStatic("FEETECH_LINE_BUSY"));
                    }
                }
                Ok(_) | Err(TransportError::Timeout) | Err(TransportError::WouldBlock) => {
                    if self.elapsed_since(quiet_since) >= Self::LINE_IDLE_QUIET {
                        self.line_poisoned = false;
                        self.line_poison_quiet_since = None;
                        self.transport.clear_input_buffer();
                        self.clear_rx();
                        return Ok(());
                    }
                    if self.clock_now() == before_read {
                        self.clock_sleep(Duration::from_millis(1));
                    }
                    if self.elapsed_since(start) >= Self::LINE_IDLE_MAX_WAIT {
                        return Err(ArmDriverError::ProtocolStatic("FEETECH_LINE_IDLE_TIMEOUT"));
                    }
                    self.clock_yield();
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// Rule E3 pre-request boundary: discard all pending RX bytes before every request; if a
    /// prior request timed out (`line_poisoned`), actively discard until the serial line is idle.
    pub(in crate::driver::bus_servo) fn discard_pending_rx_for_request(&mut self) -> Result<(), ArmDriverError> {
        self.transport.clear_input_buffer();
        self.clear_rx();
        if self.line_poisoned {
            self.discard_until_idle()?;
        }
        Ok(())
    }

    /// Zero-allocation position reply applicator shared by RX polling and M1 pre-clear draining.
    /// Feeds decoded position into `cached_states`, `last_feedback_instant`, and `rest_pose_window`
    /// without discarding queued replies. Returns `true` when a valid 8-byte position reply for a
    /// known servo was applied.
    pub(in crate::driver::bus_servo) fn apply_decoded_position_reply(
        &mut self,
        pkt_id: u8,
        pkt_error: u8,
        params: &[u8; 8],
        pkt_rx_at: Instant,
        req_written_at: Option<Instant>,
    ) -> bool {
        if pkt_id == feetech::BROADCAST_ID {
            return false;
        }
        let Some(joint_idx) = self.servo_ids.iter().position(|&id| id == pkt_id) else {
            return false;
        };
        let decode_now = self.clock_now();
        if pkt_rx_at > decode_now {
            self.rest_pose_window[joint_idx].reset();
            #[cfg(not(target_arch = "wasm32"))]
            if !self.in_disable_drain
                && self.shadow_staging.is_enabled()
                && joint_idx < crate::torque::MAX_CHANNELS
            {
                let sample_id = self.shadow_staging.next_sample_id();
                self.shadow_staging
                    .push(crate::torque::shadow::ShadowDriverObs::CausalSample {
                        channel: joint_idx as u8,
                        sample_id,
                        stamp: crate::torque::shadow::RxStampKind::Future,
                        decode_now,
                        position_rad: self.cached_states[joint_idx].position,
                        velocity_rad_s: None,
                        temperature_c: None,
                        reported_enabled: self.reported_enable[joint_idx].map(|v| v == 1),
                        correlated_after_write_done: self.last_motion_write_at[joint_idx],
                        poll_issued: req_written_at,
                        provenance: crate::torque::shadow::EvidenceProvenance::HardwareCausal,
                    });
            }
            return false;
        }
        let default_calib = feetech::FeetechJointCalib::default();
        let pos_raw = u16::from_le_bytes([params[0], params[1]]);
        let speed_raw = u16::from_le_bytes([params[2], params[3]]);
        let load_raw = u16::from_le_bytes([params[4], params[5]]);
        let temp_raw = params[7];

        let calib = self.config.joints.get(joint_idx).unwrap_or(&default_calib);
        let raw_vel = feetech::raw_speed_to_rad_s(speed_raw);
        let velocity = if calib.drive_mode_invert { -raw_vel } else { raw_vel };
        let raw_load = feetech::raw_load_to_fraction(load_raw);
        let load = if calib.drive_mode_invert { -raw_load } else { raw_load };
        let pos_steps = feetech::decode_present_position(pos_raw);
        self.cached_raw_steps[joint_idx] = pos_steps;
        self.cached_states[joint_idx] = JointState {
            position: calib.steps_to_rad(pos_steps),
            velocity,
            torque: 0.0,
            temperature: feetech::raw_to_temperature(temp_raw),
        };
        self.cached_loads[joint_idx] = load;
        self.motor_status[joint_idx] = pkt_error;
        if self.last_feedback_instant[joint_idx].is_none_or(|prev| {
            pkt_rx_at.saturating_duration_since(prev) > STOPPED_DISABLE_FEEDBACK_MAX_AGE
        }) || !self.cached_states[joint_idx].position.is_finite()
        {
            self.feedback_continuity_epoch[joint_idx] =
                self.feedback_continuity_epoch[joint_idx].wrapping_add(1);
        }
        self.last_feedback_instant[joint_idx] = Some(pkt_rx_at);
        self.feedback_seq[joint_idx] = self.feedback_seq[joint_idx].wrapping_add(1);
        // Feed causal post-motion position sample into per-joint K=3 rest-pose window.
        let after_motion = self.last_motion_write_at[joint_idx].is_none_or(|w| {
            pkt_rx_at >= w && req_written_at.is_none_or(|req_at| req_at >= w)
        });
        #[cfg(not(target_arch = "wasm32"))]
        if !self.in_disable_drain {
            self.stage_feetech_causal_sample(joint_idx, pkt_rx_at, decode_now, req_written_at, after_motion);
        }
        if let Some(ref gate) = self.stopped_disable_gate {
            if gate.is_arm_joint(joint_idx) {
                let pos = self.cached_states[joint_idx].position;
                if after_motion
                    && joint_state_valid(&self.cached_states[joint_idx], self.capabilities())
                    && gate.joint_at_rest(joint_idx, pos)
                {
                    let release_id = self.control_release_id;
                    let _ = self.rest_pose_window[joint_idx].record_agree(
                        pkt_rx_at,
                        release_id,
                        STOPPED_DISABLE_FEEDBACK_MAX_AGE,
                        None,
                    );
                } else {
                    self.rest_pose_window[joint_idx].record_disagree();
                }
            }
        }
        true
    }

    /// Zero-allocation decoder draining any complete 8-byte position replies currently queued in
    /// `self.rx_buffer` into `cached_states` and `rest_pose_window` (R12 M1/M3), bounded by
    /// `rx_budget`. Returns the number of decoded packets.
    pub(in crate::driver::bus_servo) fn decode_queued_position_replies_in_rx_budgeted(
        &mut self,
        fallback_time: Instant,
        rx_budget: &mut u16,
    ) -> u16 {
        let mut decoded: u16 = 0;
        while !self.rx_buffer.is_empty() && *rx_budget > 0 {
            match feetech::parse_response_ref(&self.rx_buffer) {
                Ok((pkt, consumed)) => {
                    let pkt_id = pkt.id;
                    let pkt_error = pkt.error;
                    let params8: Option<[u8; 8]> = if pkt.params.len() == 8 {
                        let mut a = [0u8; 8];
                        a.copy_from_slice(pkt.params);
                        Some(a)
                    } else {
                        None
                    };
                    let pkt_rx_at = self
                        .rx_timestamps
                        .get(consumed.saturating_sub(1))
                        .copied()
                        .unwrap_or(fallback_time);
                    self.drain_rx_prefix(consumed);
                    *rx_budget = rx_budget.saturating_sub(1);
                    decoded = decoded.saturating_add(1);
                    if let Some(p8) = params8 {
                        let _ = self.apply_decoded_position_reply(
                            pkt_id,
                            pkt_error,
                            &p8,
                            pkt_rx_at,
                            None,
                        );
                    }
                }
                Err(feetech::FeetechRefParseError::Incomplete) => break,
                Err(_) => {
                    self.drain_rx_prefix(1);
                }
            }
        }
        decoded
    }

    pub(in crate::driver::bus_servo) fn drain_tx_echo(&mut self, tx: &[u8]) -> Result<bool, ArmDriverError> {
        self.drain_tx_echo_until(tx, None)
    }

    pub(in crate::driver::bus_servo) fn drain_tx_echo_until(
        &mut self,
        tx: &[u8],
        deadline: Option<Instant>,
    ) -> Result<bool, ArmDriverError> {
        self.assert_clock_matches_transport();
        if tx.is_empty() || self.tx_echo == Some(false) {
            return Ok(false);
        }

        let mut temp = [0u8; 1];
        let start = self.clock_now();
        const DRAIN_TIMEOUT: Duration = Duration::from_millis(5);
        let drain_deadline = match deadline {
            Some(dl) => (start + DRAIN_TIMEOUT).min(dl),
            None => start + DRAIN_TIMEOUT,
        };
        if start >= drain_deadline {
            return Ok(false);
        }

        let resync_rx = |rx: &mut Vec<u8>, ts: &mut Vec<Instant>| {
            let mut i = 0;
            while i < rx.len() {
                if rx[i] != 0xFF {
                    let next = rx[i..].iter().position(|&b| b == 0xFF).map(|p| i + p).unwrap_or(rx.len());
                    i = next;
                    if i >= rx.len() {
                        break;
                    }
                }
                if tx.starts_with(&[0xFF, 0xFF])
                    && tx.get(2) != Some(&0xFF)
                    && rx[i..].len() >= 3
                    && rx[i..i + 3] == [0xFF, 0xFF, 0xFF]
                {
                    i += 1;
                    continue;
                }
                let slice = &rx[i..];
                let is_echo = slice.windows(tx.len()).any(|w| w == tx) || tx.starts_with(slice);
                if !is_echo {
                    match feetech::parse_response_ref(slice) {
                        Err(feetech::FeetechRefParseError::Incomplete) | Ok(_) => break,
                        Err(_) => {
                            i += 1;
                            continue;
                        }
                    }
                }
                break;
            }
            if i > 0 {
                let count = i.min(rx.len());
                rx.drain(..count);
                let ts_count = count.min(ts.len());
                ts.drain(..ts_count);
            }
        };

        let auto_detecting = self.tx_echo.is_none() || (self.tx_echo == Some(true) && self.tx_echo_auto);

        while !self.rx_buffer.windows(tx.len()).any(|w| w == tx)
            && self.clock_now() < drain_deadline
            && self.rx_buffer.len() < tx.len() + 64
        {
            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    let rx_at = self.clock_now();
                    self.push_rx(&temp[..n], rx_at);
                }
                _ => {
                    self.clock_yield();
                }
            }

            resync_rx(&mut self.rx_buffer, &mut self.rx_timestamps);

            if let Ok((pkt, consumed)) = feetech::parse_response_ref(&self.rx_buffer) {
                if &self.rx_buffer[..consumed] != tx
                    && pkt.id != feetech::BROADCAST_ID
                    && !self.rx_buffer.windows(tx.len()).any(|w| w == tx)
                {
                    if auto_detecting {
                        tracing::warn!("Auto-detected tx_echo on /dev/ttySC* received valid non-echo response; disabling tx_echo");
                        self.tx_echo = Some(false);
                        self.tx_echo_auto = false;
                    }
                    break;
                }
            }
        }

        resync_rx(&mut self.rx_buffer, &mut self.rx_timestamps);

        let mut drained = false;
        if let Some(pos) = self.rx_buffer.windows(tx.len()).position(|w| w == tx) {
            self.drain_rx_prefix(pos + tx.len());
            drained = true;
            if self.tx_echo.is_none() || self.tx_echo_auto {
                self.tx_echo = Some(true);
                self.tx_echo_auto = false;
            }
        } else if auto_detecting {
            let has_partial_echo = (2..tx.len()).any(|k| self.rx_buffer.ends_with(&tx[..k]))
                || (self.rx_buffer.len() < 2 && tx.starts_with(&self.rx_buffer));
            let has_valid_non_echo = matches!(
                feetech::parse_response_ref(&self.rx_buffer),
                Ok((pkt, c)) if &self.rx_buffer[..c] != tx && pkt.id != feetech::BROADCAST_ID
            );
            if self.tx_echo == Some(true) && self.tx_echo_auto && !has_partial_echo {
                tracing::warn!("Auto-detected tx_echo=true on /dev/ttySC* but no echo received within timeout; disabling tx_echo");
                self.tx_echo = Some(false);
                self.tx_echo_auto = false;
            } else if self.tx_echo.is_none() && has_valid_non_echo {
                self.tx_echo = Some(false);
            }
        }

        if self.tx_echo == Some(true) && !self.tx_echo_auto {
            if drained {
                self.consecutive_missed_echoes = 0;
            } else {
                self.consecutive_missed_echoes = self.consecutive_missed_echoes.saturating_add(1);
                if self.consecutive_missed_echoes >= 10 && !self.warned_missed_echoes {
                    tracing::warn!(
                        "Feetech driver has missed {} consecutive TX echoes with explicit tx_echo=true",
                        self.consecutive_missed_echoes
                    );
                    self.warned_missed_echoes = true;
                }
            }
        }

        Ok(drained)
    }

    pub(in crate::driver::bus_servo) fn decode_position_frames_from(
        &mut self,
        rx: &mut Vec<u8>,
        rx_ts: Option<&mut Vec<Instant>>,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        fallback_time: Instant,
        window: Option<(Instant, Duration)>,
    ) {
        let default_calib = feetech::FeetechJointCalib::default();
        let mut rx_ts = rx_ts;
        while !rx.is_empty() {
            match feetech::parse_response(rx) {
                Ok((pkt, consumed)) => {
                    let pkt_rx_at = rx_ts
                        .as_deref()
                        .and_then(|ts| ts.get(consumed.saturating_sub(1)).copied())
                        .unwrap_or(fallback_time);
                    rx.drain(..consumed);
                    if let Some(ts) = rx_ts.as_deref_mut() {
                        let c = consumed.min(ts.len());
                        ts.drain(..c);
                    }
                    if pkt.id == feetech::BROADCAST_ID {
                        continue;
                    }
                    let decode_now = self.clock_now();
                    if pkt_rx_at > decode_now {
                        if let Some(joint_idx) = self.servo_ids.iter().position(|&id| id == pkt.id) {
                            self.rest_pose_window[joint_idx].reset();
                        }
                        continue;
                    }
                    if let Some((req_written_at, timeout)) = window {
                        if pkt_rx_at < req_written_at
                            || pkt_rx_at.saturating_duration_since(req_written_at) >= timeout
                        {
                            continue;
                        }
                    }
                    if expected_ids.contains(&pkt.id) && !replied_ids.contains(&pkt.id) {
                        if let Some(joint_idx) = self.servo_ids.iter().position(|&id| id == pkt.id) {
                            if pkt.params.len() == 8 {
                                let pos_raw = u16::from_le_bytes([pkt.params[0], pkt.params[1]]);
                                let speed_raw = u16::from_le_bytes([pkt.params[2], pkt.params[3]]);
                                let load_raw = u16::from_le_bytes([pkt.params[4], pkt.params[5]]);
                                let temp_raw = pkt.params[7];

                                let calib = self.config.joints.get(joint_idx).unwrap_or(&default_calib);
                                let raw_vel = feetech::raw_speed_to_rad_s(speed_raw);
                                let velocity = if calib.drive_mode_invert { -raw_vel } else { raw_vel };
                                let raw_load = feetech::raw_load_to_fraction(load_raw);
                                let load = if calib.drive_mode_invert { -raw_load } else { raw_load };
                                let pos_steps = feetech::decode_present_position(pos_raw);
                                self.cached_raw_steps[joint_idx] = pos_steps;
                                self.cached_states[joint_idx] = JointState {
                                    position: calib.steps_to_rad(pos_steps),
                                    velocity,
                                    torque: 0.0,
                                    temperature: feetech::raw_to_temperature(temp_raw),
                                };
                                self.cached_loads[joint_idx] = load;
                                self.motor_status[joint_idx] = pkt.error;
                                if self.last_feedback_instant[joint_idx].is_none_or(|prev| {
                                    pkt_rx_at.saturating_duration_since(prev) > STOPPED_DISABLE_FEEDBACK_MAX_AGE
                                }) || !self.cached_states[joint_idx].position.is_finite()
                                {
                                    self.feedback_continuity_epoch[joint_idx] =
                                        self.feedback_continuity_epoch[joint_idx].wrapping_add(1);
                                }
                                self.last_feedback_instant[joint_idx] = Some(pkt_rx_at);
                                self.feedback_seq[joint_idx] = self.feedback_seq[joint_idx].wrapping_add(1);
                                // Feed causal post-motion position sample into per-joint K=3 rest-pose window.
                                let after_motion = self.last_motion_write_at[joint_idx].is_none_or(|w| {
                                    pkt_rx_at >= w
                                        && window.is_some_and(|(req_at, _)| req_at >= w)
                                });
                                #[cfg(not(target_arch = "wasm32"))]
                                if self.shadow_staging.is_enabled() {
                                    let decode_now = self.clock_now();
                                    let req_at = window.map(|(r, _)| r);
                                    self.stage_feetech_causal_sample(
                                        joint_idx,
                                        pkt_rx_at,
                                        decode_now,
                                        req_at,
                                        after_motion,
                                    );
                                }
                                if let Some(ref gate) = self.stopped_disable_gate {
                                    if gate.is_arm_joint(joint_idx) {
                                        let pos = self.cached_states[joint_idx].position;
                                        if after_motion
                                            && joint_state_valid(&self.cached_states[joint_idx], self.capabilities())
                                            && gate.joint_at_rest(joint_idx, pos)
                                        {
                                            let release_id = self.control_release_id;
                                            let _ = self.rest_pose_window[joint_idx].record_agree(
                                                pkt_rx_at,
                                                release_id,
                                                STOPPED_DISABLE_FEEDBACK_MAX_AGE,
                                                None,
                                            );
                                        } else {
                                            self.rest_pose_window[joint_idx].record_disagree();
                                        }
                                    }
                                }

                                replied_ids.insert(pkt.id);
                            }
                        }
                    }
                }
                Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                    break;
                }
                Err(_) => {
                    rx.remove(0);
                    if let Some(ts) = rx_ts.as_deref_mut() {
                        if !ts.is_empty() {
                            ts.remove(0);
                        }
                    }
                }
            }
        }
    }

    pub(in crate::driver::bus_servo) fn decode_position_frames_windowed(
        &mut self,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        req_written_at: Instant,
        timeout: Duration,
    ) {
        let mut rx = std::mem::take(&mut self.rx_buffer);
        let mut ts = std::mem::take(&mut self.rx_timestamps);
        self.decode_position_frames_from(
            &mut rx,
            Some(&mut ts),
            expected_ids,
            replied_ids,
            req_written_at,
            Some((req_written_at, timeout)),
        );
        self.rx_buffer = rx;
        self.rx_timestamps = ts;
    }

    pub(in crate::driver::bus_servo) fn decode_partial_position_frames(
        &mut self,
        expected_ids: &std::collections::HashSet<u8>,
        replied_ids: &mut std::collections::HashSet<u8>,
        inc_ts: Instant,
        ttl: Duration,
    ) {
        let mut rx = std::mem::take(&mut self.partial_pos_rx);
        let mut ts = std::mem::take(&mut self.partial_pos_rx_timestamps);
        ts.resize(rx.len(), inc_ts);
        self.decode_position_frames_from(
            &mut rx,
            Some(&mut ts),
            expected_ids,
            replied_ids,
            inc_ts,
            Some((inc_ts, ttl)),
        );
        self.partial_pos_rx = rx;
        self.partial_pos_rx_timestamps = ts;
    }
}
