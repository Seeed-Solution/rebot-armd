//! Feetech STS3215 direct register access and cold servo commissioning.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::protocol::feetech;
use crate::transport::{Transport, TransportError};

impl<T: Transport> FeetechArmDriver<T> {
    /// Zero-allocation 1-byte register readback honouring `deadline` and `rx_budget` (R12 M3).
    pub(in crate::driver::bus_servo) fn read_register_u8_budgeted(
        &mut self,
        id: u8,
        reg: u8,
        deadline: Option<Instant>,
        rx_budget: &mut u16,
    ) -> Result<(u8, Instant), ArmDriverError> {
        self.assert_clock_matches_transport();
        if *rx_budget == 0 || deadline.is_some_and(|dl| self.clock_now() >= dl) {
            return Err(ArmDriverError::Timeout(id));
        }
        self.invalidate_partial_pos_rx();
        if deadline.is_some() {
            if !self.poll_poisoned_line_idle() {
                return Err(ArmDriverError::ProtocolStatic("FEETECH_LINE_BUSY"));
            }
            self.transport.clear_input_buffer();
            self.clear_rx();
        } else {
            self.discard_pending_rx_for_request()?;
        }
        let req = feetech::build_read_array(id, reg, 1);
        self.transport.write_all(&req)?;
        self.transport.flush()?;
        let req_written_at = self.clock_now();
        let echo_drained = self.drain_tx_echo_until(&req, deadline)?;

        let mut temp = [0u8; 1];
        const READ_TIMEOUT: Duration = Duration::from_millis(25);
        let read_deadline = match deadline {
            Some(dl) => (req_written_at + READ_TIMEOUT).min(dl),
            None => req_written_at + READ_TIMEOUT,
        };
        let require_echo = !echo_drained && self.tx_echo == Some(true);
        let mut can_skip_echo = !echo_drained && self.tx_echo != Some(false);
        let mut echo_seen = echo_drained;

        loop {
            while !self.rx_buffer.is_empty() && *rx_budget > 0 {
                match feetech::parse_response_ref(&self.rx_buffer) {
                    Ok((pkt, consumed)) => {
                        let pkt_id = pkt.id;
                        let pkt_param0 = (pkt.params.len() == 1).then(|| pkt.params[0]);
                        let is_req_echo = can_skip_echo
                            && consumed >= req.len()
                            && self.rx_buffer[consumed - req.len()..consumed] == req[..];
                        let pkt_rx_at = self
                            .rx_timestamps
                            .get(consumed - 1)
                            .copied()
                            .unwrap_or_else(|| self.clock_now());
                        self.drain_rx_prefix(consumed);
                        if is_req_echo {
                            can_skip_echo = false;
                            echo_seen = true;
                            self.tx_echo = Some(true);
                            self.tx_echo_auto = false;
                            continue;
                        }
                        if require_echo && !echo_seen {
                            continue;
                        }
                        if pkt_id == feetech::BROADCAST_ID {
                            continue;
                        }
                        *rx_budget = rx_budget.saturating_sub(1);
                        let decode_now = self.clock_now();
                        if pkt_rx_at < req_written_at
                            || pkt_rx_at > decode_now
                            || pkt_rx_at >= read_deadline
                        {
                            continue;
                        }
                        if pkt_id == id {
                            if let Some(val) = pkt_param0 {
                                if self.tx_echo.is_none() {
                                    self.tx_echo = Some(false);
                                }
                                let mut scan = self.rx_buffer.as_slice();
                                while !scan.is_empty() {
                                    match feetech::parse_response_ref(scan) {
                                        Ok((extra, c)) => {
                                            scan = &scan[c..];
                                            if extra.id == id && extra.params.len() == 1 {
                                                self.clear_rx();
                                                self.poison_line();
                                                if let Some(idx) = self.servo_ids.iter().position(|&sid| sid == id) {
                                                    self.unmatched_1b_read[idx] = true;
                                                }
                                                return Err(ArmDriverError::AmbiguousDuplicateReply(id));
                                            }
                                        }
                                        Err(feetech::FeetechRefParseError::Incomplete) => break,
                                        Err(_) => scan = &scan[1..],
                                    }
                                }
                                return Ok((val, pkt_rx_at));
                            }
                        }
                    }
                    Err(feetech::FeetechRefParseError::Incomplete) => break,
                    Err(_) => {
                        self.drain_rx_prefix(1);
                    }
                }
            }

            if *rx_budget == 0 || self.clock_now() >= read_deadline {
                break;
            }

            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    let rx_at = self.clock_now();
                    if rx_at < req_written_at || rx_at >= read_deadline {
                        break;
                    }
                    self.push_rx(&temp[..n], rx_at);
                }
                _ => {
                    self.clock_yield();
                }
            }
        }

        // E3/S2: Timed-out request poisons the serial line until drained idle (20 ms) before the next request.
        self.poison_line();
        self.clear_rx();
        if let Some(idx) = self.servo_ids.iter().position(|&sid| sid == id) {
            self.unmatched_1b_read[idx] = true;
        }
        Err(ArmDriverError::Timeout(id))
    }

    pub fn read_register_u8_direct(&mut self, id: u8, reg: u8) -> Result<u8, ArmDriverError> {
        self.read_register_direct_timestamped(id, reg, 1).map(|(p, _)| p[0])
    }

    pub fn read_register_u16_direct(&mut self, id: u8, reg: u8) -> Result<u16, ArmDriverError> {
        self.read_register_direct_timestamped(id, reg, 2)
            .map(|(p, _)| u16::from_le_bytes([p[0], p[1]]))
    }

    pub(in crate::driver::bus_servo) fn read_register_direct_timestamped(
        &mut self,
        id: u8,
        reg: u8,
        len: u8,
    ) -> Result<(Vec<u8>, Instant), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        self.discard_pending_rx_for_request()?;
        let req = feetech::build_read(id, reg, len);
        self.transport.write_all(&req)?;
        self.transport.flush()?;
        let req_written_at = self.clock_now();
        let echo_drained = self.drain_tx_echo(&req)?;

        let mut temp = [0u8; 1];
        const READ_TIMEOUT: Duration = Duration::from_millis(25);
        let require_echo = !echo_drained && self.tx_echo == Some(true);
        let mut can_skip_echo = !echo_drained && self.tx_echo != Some(false);
        let mut echo_seen = echo_drained;

        loop {
            while !self.rx_buffer.is_empty() {
                match feetech::parse_response(&self.rx_buffer) {
                    Ok((pkt, consumed)) => {
                        let is_req_echo = can_skip_echo
                            && consumed >= req.len()
                            && self.rx_buffer[consumed - req.len()..consumed] == req[..];
                        let pkt_rx_at = self
                            .rx_timestamps
                            .get(consumed - 1)
                            .copied()
                            .unwrap_or_else(|| self.clock_now());
                        self.drain_rx_prefix(consumed);
                        if is_req_echo {
                            can_skip_echo = false;
                            echo_seen = true;
                            self.tx_echo = Some(true);
                            self.tx_echo_auto = false;
                            continue;
                        }
                        if require_echo && !echo_seen {
                            // Explicit TX-echo bus: any packet received before our request echo
                            // arrived prior to this request and cannot satisfy it.
                            continue;
                        }
                        if pkt.id == feetech::BROADCAST_ID {
                            continue;
                        }
                        let decode_now = self.clock_now();
                        if pkt_rx_at < req_written_at
                            || pkt_rx_at > decode_now
                            || pkt_rx_at.saturating_duration_since(req_written_at) >= READ_TIMEOUT
                        {
                            continue;
                        }
                        if pkt.id == id && pkt.params.len() == len as usize {
                            if self.tx_echo.is_none() {
                                self.tx_echo = Some(false);
                            }
                            if len == 1 {
                                // Scan any already-buffered bytes in `self.rx_buffer` so a late
                                // 1-byte reply queued back-to-back with the genuine reply is
                                // rejected as ambiguous without consuming future transport replies.
                                let mut scan = self.rx_buffer.as_slice();
                                while !scan.is_empty() {
                                    match feetech::parse_response(scan) {
                                        Ok((extra, c)) => {
                                            scan = &scan[c..];
                                            if extra.id == id && extra.params.len() == 1 {
                                                self.clear_rx();
                                                self.poison_line();
                                                if let Some(idx) = self.servo_ids.iter().position(|&sid| sid == id) {
                                                    self.unmatched_1b_read[idx] = true;
                                                }
                                                return Err(ArmDriverError::AmbiguousDuplicateReply(id));
                                            }
                                        }
                                        Err(feetech::FeetechError::IncompleteFrame { .. }) => break,
                                        Err(_) => scan = &scan[1..],
                                    }
                                }
                            }
                            return Ok((pkt.params, pkt_rx_at));
                        }
                    }
                    Err(feetech::FeetechError::IncompleteFrame { .. }) => {
                        break;
                    }
                    Err(_) => {
                        self.drain_rx_prefix(1);
                    }
                }
            }

            if self.elapsed_since(req_written_at) >= READ_TIMEOUT {
                break;
            }

            match self.transport.read(&mut temp) {
                Ok(n) if n > 0 => {
                    let rx_at = self.clock_now();
                    if rx_at < req_written_at
                        || rx_at.saturating_duration_since(req_written_at) >= READ_TIMEOUT
                    {
                        break;
                    }
                    self.push_rx(&temp[..n], rx_at);
                }
                _ => {
                    self.clock_yield();
                }
            }
        }

        // E3/S2: Timed-out request poisons the serial line until drained idle (20 ms) before the next request.
        self.poison_line();
        self.clear_rx();
        if len == 1 {
            if let Some(idx) = self.servo_ids.iter().position(|&sid| sid == id) {
                self.unmatched_1b_read[idx] = true;
            }
        }
        Err(ArmDriverError::Timeout(id))
    }

    pub(in crate::driver::bus_servo) fn write_register_u8_direct(&mut self, id: u8, reg: u8, val: u8) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        self.write_register_u8_cleanup(id, reg, val)
    }

    pub(in crate::driver::bus_servo) fn write_register_u8_cleanup(&mut self, id: u8, reg: u8, val: u8) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        self.discard_pending_rx_for_request()?;
        let pkt = feetech::build_write_u8(id, reg, val);
        self.transport.write_all(&pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&pkt)?;
        Ok(())
    }

    pub(in crate::driver::bus_servo) fn write_register_u16_direct(&mut self, id: u8, reg: u8, val: u16) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        self.invalidate_partial_pos_rx();
        self.discard_pending_rx_for_request()?;
        let pkt = feetech::build_write_u16(id, reg, val);
        self.transport.write_all(&pkt)?;
        self.transport.flush()?;
        self.drain_tx_echo(&pkt)?;
        Ok(())
    }

    /// Configure EEPROM and RAM registers for all servos based on `FeetechConfig`.
    pub fn configure_servos(&mut self) -> Result<(), ArmDriverError> {
        self.assert_clock_matches_transport();
        self.invalidate_partial_pos_rx();
        if self.is_aborted() {
            return Err(ArmDriverError::HardwareFault("Aborted by emergency stop".into()));
        }
        let mut all_disabled = true;
        let mut all_enabled = true;

        for joint_idx in 0..self.servo_ids.len() {
            let id = self.servo_ids[joint_idx];
            let calib = self.config.joints.get(joint_idx).cloned().unwrap_or(feetech::FeetechJointCalib {
                id,
                ..Default::default()
            });

            // 1. Verify model
            let model = self.read_register_u16_direct(id, feetech::REG_MODEL_L)?;
            if model != feetech::EXPECTED_MODEL_STS3215 {
                return Err(ArmDriverError::Protocol(format!(
                    "Servo ID {} reported model {}, expected STS3215 ({})",
                    id, model, feetech::EXPECTED_MODEL_STS3215
                )));
            }

            // 2. Read torque enable status
            let (torque_en_params, torque_en_rx_at) =
                self.read_register_direct_timestamped(id, feetech::REG_TORQUE_ENABLE, 1)?;
            let torque_en = torque_en_params[0];
            match torque_en {
                0 => {
                    if joint_idx < self.reported_enable.len() {
                        self.reported_enable[joint_idx] = Some(0);
                        self.last_enable_confirmed[joint_idx] = Some(torque_en_rx_at);
                        self.missed_enable_polls[joint_idx] = 0;
                    }
                    all_enabled = false;
                }
                1 => {
                    if joint_idx < self.reported_enable.len() {
                        self.reported_enable[joint_idx] = Some(1);
                        self.last_enable_confirmed[joint_idx] = Some(torque_en_rx_at);
                        self.missed_enable_polls[joint_idx] = 0;
                    }
                    all_disabled = false;
                }
                _ => {
                    if joint_idx < self.reported_enable.len() {
                        self.reported_enable[joint_idx] = None;
                        self.last_enable_confirmed[joint_idx] = None;
                    }
                    all_enabled = false;
                    all_disabled = false;
                }
            }

            // 3. Read current registers
            let cur_return_delay = self.read_register_u8_direct(id, feetech::REG_RETURN_DELAY)?;
            let cur_status_level = self.read_register_u8_direct(id, feetech::REG_STATUS_RETURN_LEVEL)?;
            let cur_min_angle = self.read_register_u16_direct(id, feetech::REG_MIN_ANGLE_LIMIT_L)?;
            let cur_max_angle = self.read_register_u16_direct(id, feetech::REG_MAX_ANGLE_LIMIT_L)?;
            let cur_p_coef = self.read_register_u8_direct(id, feetech::REG_P_COEF)?;
            let cur_d_coef = self.read_register_u8_direct(id, feetech::REG_D_COEF)?;
            let cur_i_coef = self.read_register_u8_direct(id, feetech::REG_I_COEF)?;
            let cur_max_torque = self.read_register_u16_direct(id, feetech::REG_MAX_TORQUE_L)?;
            let cur_prot_current = self.read_register_u16_direct(id, feetech::REG_PROTECTION_CURRENT_L)?;
            let cur_overload_torque = self.read_register_u8_direct(id, feetech::REG_OVERLOAD_TORQUE)?;
            let cur_operating_mode = self.read_register_u8_direct(id, feetech::REG_OPERATING_MODE)?;

            // Effective angle limits validation and storage
            if joint_idx < self.hw_angle_limits.len() {
                self.hw_angle_limits[joint_idx] = Some((cur_min_angle, cur_max_angle));
            }
            let eff_min = calib.range_min_steps.unwrap_or(cur_min_angle);
            let eff_max = calib.range_max_steps.unwrap_or(cur_max_angle);
            if eff_min >= eff_max || eff_max > 4095 {
                return Err(ArmDriverError::HardwareFault(format!(
                    "Servo ID {} has invalid angle limits min={} max={}",
                    id, eff_min, eff_max
                )));
            }
            if joint_idx < self.effective_ranges.len() {
                self.effective_ranges[joint_idx] = (eff_min, eff_max);
            }

            // Expected homing offset verification
            if let Some(expected) = calib.expected_homing_offset_steps {
                let raw_offset = self.read_register_u16_direct(id, feetech::REG_HOMING_OFFSET_L)?;
                let decoded = feetech::decode_sign_magnitude(raw_offset, 0x0800);
                if decoded != expected {
                    return Err(ArmDriverError::HardwareFault(format!(
                        "Servo ID {} homing offset mismatch: expected {}, read {}",
                        id, expected, decoded
                    )));
                }
            }

            // Targets
            let target_return_delay = self.config.return_delay_raw;
            let target_status_level = 0u8;
            let target_p_coef = self.config.p_coef;
            let target_d_coef = self.config.d_coef;
            let target_i_coef = self.config.i_coef;
            let target_max_torque = calib.max_torque_raw.unwrap_or(self.config.max_torque_raw);
            let target_prot_current = calib.protection_current_raw.unwrap_or(self.config.protection_current_raw);
            let target_overload_torque = calib.overload_torque_raw.or(self.config.overload_torque_raw);
            let target_operating_mode = 0u8;

            let differs = cur_return_delay != target_return_delay
                || cur_status_level != target_status_level
                || calib.range_min_steps.is_some_and(|m| cur_min_angle != m)
                || calib.range_max_steps.is_some_and(|m| cur_max_angle != m)
                || cur_p_coef != target_p_coef
                || cur_d_coef != target_d_coef
                || cur_i_coef != target_i_coef
                || cur_max_torque != target_max_torque
                || cur_prot_current != target_prot_current
                || target_overload_torque.is_some_and(|ot| cur_overload_torque != ot)
                || cur_operating_mode != target_operating_mode;

            if differs {
                if torque_en != 0 {
                    return Err(ArmDriverError::HardwareFault(
                        "cannot configure EEPROM while torque is enabled".to_string(),
                    ));
                }

                let mut changed_registers: Vec<(u8, u8, u16)> = Vec::new();

                let write_res = (|| -> Result<(), ArmDriverError> {
                    // Unlock EEPROM inside closure so REG_LOCK=1 always runs
                    self.write_register_u8_direct(id, feetech::REG_LOCK, 0)?;
                    self.clock_sleep(Duration::from_millis(2));

                    if cur_return_delay != target_return_delay {
                        self.write_register_u8_direct(id, feetech::REG_RETURN_DELAY, target_return_delay)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_RETURN_DELAY, 1, target_return_delay as u16));
                    }
                    if cur_status_level != target_status_level {
                        self.write_register_u8_direct(id, feetech::REG_STATUS_RETURN_LEVEL, target_status_level)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_STATUS_RETURN_LEVEL, 1, target_status_level as u16));
                    }
                    if let Some(target_min) = calib.range_min_steps {
                        if cur_min_angle != target_min {
                            self.write_register_u16_direct(id, feetech::REG_MIN_ANGLE_LIMIT_L, target_min)?;
                            self.clock_sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_MIN_ANGLE_LIMIT_L, 2, target_min));
                        }
                    }
                    if let Some(target_max) = calib.range_max_steps {
                        if cur_max_angle != target_max {
                            self.write_register_u16_direct(id, feetech::REG_MAX_ANGLE_LIMIT_L, target_max)?;
                            self.clock_sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_MAX_ANGLE_LIMIT_L, 2, target_max));
                        }
                    }
                    if cur_p_coef != target_p_coef {
                        self.write_register_u8_direct(id, feetech::REG_P_COEF, target_p_coef)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_P_COEF, 1, target_p_coef as u16));
                    }
                    if cur_d_coef != target_d_coef {
                        self.write_register_u8_direct(id, feetech::REG_D_COEF, target_d_coef)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_D_COEF, 1, target_d_coef as u16));
                    }
                    if cur_i_coef != target_i_coef {
                        self.write_register_u8_direct(id, feetech::REG_I_COEF, target_i_coef)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_I_COEF, 1, target_i_coef as u16));
                    }
                    if cur_max_torque != target_max_torque {
                        self.write_register_u16_direct(id, feetech::REG_MAX_TORQUE_L, target_max_torque)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_MAX_TORQUE_L, 2, target_max_torque));
                    }
                    if cur_prot_current != target_prot_current {
                        self.write_register_u16_direct(id, feetech::REG_PROTECTION_CURRENT_L, target_prot_current)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_PROTECTION_CURRENT_L, 2, target_prot_current));
                    }
                    if let Some(target_overload) = target_overload_torque {
                        if cur_overload_torque != target_overload {
                            self.write_register_u8_direct(id, feetech::REG_OVERLOAD_TORQUE, target_overload)?;
                            self.clock_sleep(Duration::from_millis(2));
                            changed_registers.push((feetech::REG_OVERLOAD_TORQUE, 1, target_overload as u16));
                        }
                    }
                    if cur_operating_mode != target_operating_mode {
                        self.write_register_u8_direct(id, feetech::REG_OPERATING_MODE, target_operating_mode)?;
                        self.clock_sleep(Duration::from_millis(2));
                        changed_registers.push((feetech::REG_OPERATING_MODE, 1, target_operating_mode as u16));
                    }

                    // Verify readback of all changed registers before lock
                    for &(reg, size, expected_val) in &changed_registers {
                        if size == 1 {
                            let read_val = self.read_register_u8_direct(id, reg)?;
                            if (read_val as u16) != expected_val {
                                return Err(ArmDriverError::HardwareFault(format!(
                                    "EEPROM readback verify failed for servo ID {} reg 0x{:02X}: expected {}, got {}",
                                    id, reg, expected_val, read_val
                                )));
                            }
                        } else {
                            let read_val = self.read_register_u16_direct(id, reg)?;
                            if read_val != expected_val {
                                return Err(ArmDriverError::HardwareFault(format!(
                                    "EEPROM readback verify failed for servo ID {} reg 0x{:02X}: expected {}, got {}",
                                    id, reg, expected_val, read_val
                                )));
                            }
                        }
                    }

                    Ok(())
                })();

                // ALWAYS lock EEPROM before returning
                let lock_res = self.write_register_u8_cleanup(id, feetech::REG_LOCK, 1);
                if let Err(e) = write_res {
                    if let Err(le) = lock_res {
                        tracing::error!("Failed to relock EEPROM for servo {id} during error recovery: {le}");
                    }
                    return Err(e);
                }
                lock_res?;
                if cur_max_torque != target_max_torque {
                    self.write_register_u16_direct(id, feetech::REG_TORQUE_LIMIT_L, target_max_torque)?;
                }
            }
        }

        let _ = (all_disabled, all_enabled);

        Ok(())
    }
}
