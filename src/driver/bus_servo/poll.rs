//! Feetech STS3215 budgeted and blocking joint-state polling loops.

use std::time::{Duration, Instant};

use crate::driver::*;
use crate::protocol::feetech;
use crate::transport::Transport;

impl<T: Transport> FeetechArmDriver<T> {
    /// Zero-allocation, deadline- and RX-budget-honouring Feetech position poll pass used by
    /// `poll_torque` and `poll_torque_budgeted` (R12 M3).
    /// Returns `(frames_decoded, budget_exhausted)` without allocating or running `did_enable_poll`.
    pub(super) fn read_joint_states_budgeted_pass(
        &mut self,
        deadline: Instant,
        max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        self.assert_clock_matches_transport();
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok((0, false));
        }
        if max_rx_frames == 0 {
            return Ok((0, true));
        }
        let start_time = self.clock_now();
        if start_time >= deadline {
            return Ok((0, false));
        }

        const MAX_SYNC_READ_SERVOS: usize = 3;
        const MAX_READ_DURATION: Duration = Duration::from_millis(8);
        const CHUNK_TIMEOUT: Duration = Duration::from_millis(4);
        const INCOMPLETE_FRAME_TTL: Duration = Duration::from_millis(25);
        const REASSEMBLY_TIMEOUT: Duration = Duration::from_millis(2);
        let read_deadline = (start_time + MAX_READ_DURATION).min(deadline);

        self.read_poll_count += 1;
        let mut frames_decoded: u16 = 0;
        let mut budget_exhausted = false;

        // Decode frames from `self.rx_buffer` into `replied_mask` (zero alloc, bounded by `max_rx_frames`).
        let decode_from_rx = |drv: &mut Self,
                              chunk: &[u8],
                              replied_mask: &mut u8,
                              window_start: Instant,
                              window_deadline: Instant,
                              frames: &mut u16| {
            while !drv.rx_buffer.is_empty() && *frames < max_rx_frames {
                match feetech::parse_response_ref(&drv.rx_buffer) {
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
                        let pkt_rx_at = drv
                            .rx_timestamps
                            .get(consumed.saturating_sub(1))
                            .copied()
                            .unwrap_or(window_start);
                        drv.drain_rx_prefix(consumed);
                        if pkt_id == feetech::BROADCAST_ID {
                            continue;
                        }
                        let decode_now = drv.clock_now();
                        if pkt_rx_at < window_start
                            || pkt_rx_at > decode_now
                            || pkt_rx_at >= window_deadline
                        {
                            continue;
                        }
                        if let Some(pos_in_chunk) = chunk.iter().position(|&id| id == pkt_id) {
                            if let Some(p8) = params8 {
                                if drv.apply_decoded_position_reply(
                                    pkt_id,
                                    pkt_error,
                                    &p8,
                                    pkt_rx_at,
                                    Some(window_start),
                                ) {
                                    *replied_mask |= 1u8 << pos_in_chunk;
                                    *frames = frames.saturating_add(1);
                                }
                            }
                        }
                    }
                    Err(feetech::FeetechRefParseError::Incomplete) => break,
                    Err(_) => {
                        drv.drain_rx_prefix(1);
                    }
                }
            }
        };

        // Handle an in-flight cross-tick partial frame without allocating.
        if let Some(inc_idx) = self.incomplete_chunk_idx.take() {
            let inc_ts = self.incomplete_frame_started_at.take().unwrap_or(start_time);
            let inc_start = inc_idx * MAX_SYNC_READ_SERVOS;
            if self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL || inc_start >= dof {
                self.invalidate_partial_pos_rx();
                self.poison_line();
            } else {
                let inc_end = (inc_start + MAX_SYNC_READ_SERVOS).min(dof);
                let inc_len = inc_end - inc_start;
                let mut inc_chunk = [0u8; MAX_SYNC_READ_SERVOS];
                inc_chunk[..inc_len].copy_from_slice(&self.servo_ids[inc_start..inc_end]);
                let inc_all_mask: u8 = (1u8 << inc_len) - 1;
                let mut replied_mask: u8 = 0;

                // Move partial bytes into `rx_buffer` in place (zero alloc).
                self.clear_rx();
                let plen = self.partial_pos_rx.len();
                let tlen = self.partial_pos_rx_timestamps.len();
                for i in 0..plen {
                    let b = self.partial_pos_rx[i];
                    let t = if i < tlen { self.partial_pos_rx_timestamps[i] } else { inc_ts };
                    self.rx_buffer.push(b);
                    self.rx_timestamps.push(t);
                }
                self.partial_pos_rx.clear();
                self.partial_pos_rx_timestamps.clear();

                let inc_ttl_deadline = inc_ts + INCOMPLETE_FRAME_TTL;
                let reasm_deadline = (start_time + REASSEMBLY_TIMEOUT)
                    .min(read_deadline)
                    .min(inc_ttl_deadline);
                decode_from_rx(
                    self,
                    &inc_chunk[..inc_len],
                    &mut replied_mask,
                    inc_ts,
                    inc_ttl_deadline,
                    &mut frames_decoded,
                );
                let mut drain_buf = [0u8; 32];
                while replied_mask != inc_all_mask
                    && frames_decoded < max_rx_frames
                    && self.clock_now() < reasm_deadline
                {
                    match self.transport.read(&mut drain_buf) {
                        Ok(n) if n > 0 => {
                            let byte_rx_at = self.clock_now();
                            if byte_rx_at >= inc_ttl_deadline {
                                break;
                            }
                            self.push_rx(&drain_buf[..n], byte_rx_at);
                            decode_from_rx(
                                self,
                                &inc_chunk[..inc_len],
                                &mut replied_mask,
                                inc_ts,
                                inc_ttl_deadline,
                                &mut frames_decoded,
                            );
                        }
                        _ => {
                            if self.clock_now() >= reasm_deadline {
                                break;
                            }
                            self.clock_yield();
                        }
                    }
                }
                if replied_mask != inc_all_mask {
                    if frames_decoded >= max_rx_frames {
                        budget_exhausted = true;
                    } else if self.clock_now() >= inc_ttl_deadline {
                        self.poison_line();
                    } else if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(
                        &self.rx_buffer,
                        &inc_chunk[..inc_len],
                        10,
                    ) {
                        let tail_offset = self.rx_buffer.len() - tail.len();
                        let tail_len = tail.len().min(64);
                        self.partial_pos_rx.clear();
                        self.partial_pos_rx_timestamps.clear();
                        for i in 0..tail_len {
                            self.partial_pos_rx.push(self.rx_buffer[tail_offset + i]);
                            let ts = self
                                .rx_timestamps
                                .get(tail_offset + i)
                                .copied()
                                .unwrap_or(inc_ts);
                            self.partial_pos_rx_timestamps.push(ts);
                        }
                        self.incomplete_chunk_idx = Some(inc_idx);
                        self.incomplete_frame_started_at = Some(inc_ts);
                    } else {
                        self.poison_line();
                    }
                }
                self.clear_rx();
            }
        }

        if self.incomplete_chunk_idx.is_some() {
            self.last_read_release_id = self.control_release_id;
            return Ok((frames_decoded, budget_exhausted));
        }

        let num_chunks = dof.div_ceil(MAX_SYNC_READ_SERVOS);
        let start_chunk = self.poll_cursor % num_chunks;
        let mut temp_chunk = [0u8; 1];
        let mut sync_read_buf = [0u8; 16];

        'chunk_loop: for offset in 0..num_chunks {
            let chunk_idx = (start_chunk + offset) % num_chunks;
            if frames_decoded >= max_rx_frames {
                budget_exhausted = true;
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }
            if self.clock_now() >= read_deadline {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }
            if !self.poll_poisoned_line_idle() {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }
            self.transport.clear_input_buffer();
            self.clear_rx();
            if self.clock_now() >= read_deadline {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }

            let c_start = chunk_idx * MAX_SYNC_READ_SERVOS;
            let c_end = (c_start + MAX_SYNC_READ_SERVOS).min(dof);
            let c_len = c_end - c_start;
            let mut chunk_ids = [0u8; MAX_SYNC_READ_SERVOS];
            chunk_ids[..c_len].copy_from_slice(&self.servo_ids[c_start..c_end]);
            let all_mask: u8 = (1u8 << c_len) - 1;

            let pkt_len = feetech::write_sync_read_slice(
                &chunk_ids[..c_len],
                feetech::REG_PRESENT_POSITION_L,
                8,
                &mut sync_read_buf,
            )
            .map_err(ArmDriverError::ProtocolStatic)?;

            self.transport.write_all(&sync_read_buf[..pkt_len])?;
            self.transport.flush()?;
            let req_written_at = self.clock_now();
            self.drain_tx_echo_until(&sync_read_buf[..pkt_len], Some(read_deadline))?;

            let chunk_deadline = (req_written_at + CHUNK_TIMEOUT).min(read_deadline);
            let mut replied_mask: u8 = 0;

            while replied_mask != all_mask
                && frames_decoded < max_rx_frames
                && self.clock_now() < chunk_deadline
            {
                decode_from_rx(
                    self,
                    &chunk_ids[..c_len],
                    &mut replied_mask,
                    req_written_at,
                    chunk_deadline,
                    &mut frames_decoded,
                );
                if replied_mask == all_mask || frames_decoded >= max_rx_frames {
                    break;
                }
                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        let byte_rx_at = self.clock_now();
                        if byte_rx_at < req_written_at || byte_rx_at >= chunk_deadline {
                            break;
                        }
                        self.push_rx(&temp_chunk[..n], byte_rx_at);
                    }
                    _ => {}
                }
                decode_from_rx(
                    self,
                    &chunk_ids[..c_len],
                    &mut replied_mask,
                    req_written_at,
                    chunk_deadline,
                    &mut frames_decoded,
                );
                if replied_mask != all_mask
                    && frames_decoded < max_rx_frames
                    && self.clock_now() < chunk_deadline
                {
                    self.clock_yield();
                }
            }

            if replied_mask != all_mask {
                if frames_decoded >= max_rx_frames {
                    budget_exhausted = true;
                    self.clear_rx();
                    self.poll_cursor = (chunk_idx + 1) % num_chunks;
                    break 'chunk_loop;
                }
                self.missed_poll_replies += 1;
                if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(
                    &self.rx_buffer,
                    &chunk_ids[..c_len],
                    10,
                ) {
                    let tail_offset = self.rx_buffer.len() - tail.len();
                    let tail_started_at = self
                        .rx_timestamps
                        .get(tail_offset)
                        .copied()
                        .unwrap_or(req_written_at);
                    if tail_started_at >= req_written_at && tail_started_at < chunk_deadline {
                        let tail_len = tail.len().min(64);
                        self.partial_pos_rx.clear();
                        self.partial_pos_rx_timestamps.clear();
                        for i in 0..tail_len {
                            self.partial_pos_rx.push(self.rx_buffer[tail_offset + i]);
                            let ts = self
                                .rx_timestamps
                                .get(tail_offset + i)
                                .copied()
                                .unwrap_or(tail_started_at);
                            self.partial_pos_rx_timestamps.push(ts);
                        }
                        self.incomplete_chunk_idx = Some(chunk_idx);
                        self.incomplete_frame_started_at = Some(tail_started_at);
                        self.clear_rx();
                        self.poll_cursor = (chunk_idx + 1) % num_chunks;
                        break 'chunk_loop;
                    }
                }
                self.poison_line();
            }
            self.incomplete_chunk_idx = None;
            self.incomplete_frame_started_at = None;
            self.partial_pos_rx.clear();
            self.partial_pos_rx_timestamps.clear();
            self.clear_rx();
            self.poll_cursor = (chunk_idx + 1) % num_chunks;
        }

        self.last_read_release_id = self.control_release_id;
        Ok((frames_decoded, budget_exhausted))
    }

    pub(super) fn read_joint_states_inner(&mut self, advance_unmanaged_release: bool) -> Result<Vec<JointState>, ArmDriverError> {
        self.assert_clock_matches_transport();
        let dof = self.servo_ids.len();
        if dof == 0 {
            return Ok(Vec::new());
        }

        const MAX_SYNC_READ_SERVOS: usize = 3;
        const MAX_READ_DURATION: Duration = Duration::from_millis(8);
        self.read_poll_count += 1;
        if advance_unmanaged_release && !self.control_release_managed {
            self.control_release_id = self.control_release_id.wrapping_add(1).max(1);
        }
        let start_time = self.clock_now();
        let mut temp_chunk = [0u8; 1];

        const INCOMPLETE_FRAME_TTL: Duration = Duration::from_millis(25);
        if let Some(inc_idx) = self.incomplete_chunk_idx.take() {
            let inc_ts = self.incomplete_frame_started_at.take().unwrap_or_else(|| self.clock_now());
            if self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL {
                self.invalidate_partial_pos_rx();
                self.poison_line();
            } else if let Some(inc_chunk) = self.servo_ids.chunks(MAX_SYNC_READ_SERVOS).nth(inc_idx).map(|c| c.to_vec()) {
                let expected_ids: std::collections::HashSet<u8> = inc_chunk.iter().copied().collect();
                let mut replied_ids = std::collections::HashSet::new();
                self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts, INCOMPLETE_FRAME_TTL);

                let reassembly_start = self.clock_now();
                const REASSEMBLY_TIMEOUT: Duration = Duration::from_millis(2);
                let mut drain_buf = [0u8; 32];

                while replied_ids.len() < inc_chunk.len()
                    && self.elapsed_since(reassembly_start) < REASSEMBLY_TIMEOUT
                    && self.elapsed_since(start_time) < MAX_READ_DURATION
                    && self.elapsed_since(inc_ts) < INCOMPLETE_FRAME_TTL
                {
                    match self.transport.read(&mut drain_buf) {
                        Ok(n) if n > 0 => {
                            let byte_rx_at = self.clock_now();
                            if byte_rx_at.saturating_duration_since(inc_ts) >= INCOMPLETE_FRAME_TTL {
                                break;
                            }
                            self.partial_pos_rx.extend_from_slice(&drain_buf[..n]);
                            self.partial_pos_rx_timestamps.resize(self.partial_pos_rx.len(), byte_rx_at);
                            self.decode_partial_position_frames(&expected_ids, &mut replied_ids, inc_ts, INCOMPLETE_FRAME_TTL);
                        }
                        _ => {
                            if self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL {
                                break;
                            }
                            self.clock_yield();
                        }
                    }
                }

                let all_replied = replied_ids.len() == inc_chunk.len();
                let inc_timed_out = self.elapsed_since(inc_ts) >= INCOMPLETE_FRAME_TTL;

                if !all_replied && !inc_timed_out {
                    if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(&self.partial_pos_rx, &inc_chunk, 10) {
                        let tail_offset = self.partial_pos_rx.len() - tail.len();
                        let tail_bytes = tail.to_vec();
                        let tail_ts = if self.partial_pos_rx_timestamps.len() == self.partial_pos_rx.len() {
                            self.partial_pos_rx_timestamps[tail_offset..].to_vec()
                        } else {
                            vec![inc_ts; tail_bytes.len()]
                        };
                        self.partial_pos_rx = tail_bytes;
                        self.partial_pos_rx_timestamps = tail_ts;
                        self.incomplete_chunk_idx = Some(inc_idx);
                        self.incomplete_frame_started_at = Some(inc_ts);
                    } else {
                        self.invalidate_partial_pos_rx();
                        self.poison_line();
                    }
                } else {
                    if !all_replied && inc_timed_out {
                        self.poison_line();
                    }
                    self.invalidate_partial_pos_rx();
                }
            } else {
                self.invalidate_partial_pos_rx();
            }
        }

        let did_enable_poll = self.incomplete_chunk_idx.is_none()
            && (self.reported_enable.contains(&Some(1)) || self.torque_enable_commanded)
            && (self.read_poll_count.is_multiple_of(10) || self.last_enable_poll.is_none_or(|t| self.elapsed_since(t) >= Duration::from_millis(200)));
        if did_enable_poll {
            self.last_enable_poll = Some(self.clock_now());
            if !self.poll_poisoned_line_idle() {
                self.reported_enable.fill(None);
            } else if let Ok(sync_read) = feetech::build_sync_read(&self.servo_ids, feetech::REG_TORQUE_ENABLE, 1) {
                self.transport.clear_input_buffer();
                self.clear_rx();
                let _ = self.transport.write_all(&sync_read);
                let _ = self.transport.flush();
                let req_written_at = self.clock_now();
                let _ = self.drain_tx_echo(&sync_read);

                let mut temp_en = [0u8; 1];
                const ENABLE_POLL_TIMEOUT: Duration = Duration::from_millis(3);
                let mut replies: std::collections::HashMap<u8, Option<(u8, Instant)>> = std::collections::HashMap::new();
                let mut had_duplicate = false;

                while replies.len() < self.servo_ids.len()
                    && self.elapsed_since(req_written_at) < ENABLE_POLL_TIMEOUT
                    && self.elapsed_since(start_time) < Duration::from_millis(10)
                {
                    if let Ok(n) = self.transport.read(&mut temp_en) {
                        if n > 0 {
                            let rx_at = self.clock_now();
                            if rx_at < req_written_at
                                || rx_at.saturating_duration_since(req_written_at) >= ENABLE_POLL_TIMEOUT
                                || self.elapsed_since(start_time) >= Duration::from_millis(10)
                            {
                                break;
                            }
                            self.push_rx(&temp_en[..n], rx_at);
                        }
                    }
                    while !self.rx_buffer.is_empty() {
                        match feetech::parse_response(&self.rx_buffer) {
                            Ok((pkt, consumed)) => {
                                let pkt_rx_at = self
                                    .rx_timestamps
                                    .get(consumed - 1)
                                    .copied()
                                    .unwrap_or(req_written_at);
                                self.drain_rx_prefix(consumed);
                                let decode_now = self.clock_now();
                                if pkt_rx_at < req_written_at
                                    || pkt_rx_at > decode_now
                                    || pkt_rx_at.saturating_duration_since(req_written_at) >= ENABLE_POLL_TIMEOUT
                                {
                                    continue;
                                }
                                if pkt.id != feetech::BROADCAST_ID && self.servo_ids.contains(&pkt.id) && pkt.params.len() == 1 {
                                    replies
                                        .entry(pkt.id)
                                        .and_modify(|slot| {
                                            *slot = None;
                                            had_duplicate = true;
                                        })
                                        .or_insert(Some((pkt.params[0], pkt_rx_at)));
                                }
                            }
                            Err(feetech::FeetechError::IncompleteFrame { .. }) => break,
                            Err(_) => {
                                self.drain_rx_prefix(1);
                            }
                        }
                    }
                    if replies.len() < self.servo_ids.len() {
                        self.clock_yield();
                    }
                }

                if replies.len() < self.servo_ids.len() || had_duplicate {
                    self.poison_line();
                }
                self.clear_rx();

                let now = self.clock_now();
                let servo_ids_len = self.servo_ids.len();
                for idx in 0..servo_ids_len {
                    let id = self.servo_ids[idx];
                    if self.unmatched_1b_read.get(idx).copied().unwrap_or(false) {
                        self.reported_enable[idx] = None;
                        self.last_enable_confirmed[idx] = None;
                        continue;
                    }
                    match replies.get(&id).copied() {
                        Some(Some((v @ (0 | 1), reply_rx_at))) => {
                            self.record_reported_enable(idx, v, reply_rx_at);
                        }
                        Some(_) => {
                            self.reported_enable[idx] = None;
                            self.last_enable_confirmed[idx] = None;
                        }
                        None => {
                            self.missed_enable_polls[idx] = self.missed_enable_polls[idx].saturating_add(1);
                            if self.missed_enable_polls[idx] >= 3
                                || self.last_enable_confirmed[idx].is_none_or(|t| now.saturating_duration_since(t) > Duration::from_millis(600))
                            {
                                self.reported_enable[idx] = None;
                                self.last_enable_confirmed[idx] = None;
                            }
                        }
                    }
                }
            }
        }

        let chunks: Vec<Vec<u8>> = self.servo_ids.chunks(MAX_SYNC_READ_SERVOS).map(|c| c.to_vec()).collect();
        let num_chunks = chunks.len();
        let start_chunk = self.poll_cursor % num_chunks;

        if self.incomplete_chunk_idx.is_some() {
            return Ok(self.cached_states.clone());
        }

        'chunk_loop: for offset in 0..num_chunks {
            let chunk_idx = (start_chunk + offset) % num_chunks;
            let chunk = &chunks[chunk_idx];

            if self.elapsed_since(start_time) >= MAX_READ_DURATION {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }

            if !self.poll_poisoned_line_idle() {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }
            self.transport.clear_input_buffer();
            self.clear_rx();
            if self.elapsed_since(start_time) >= MAX_READ_DURATION {
                self.poll_cursor = chunk_idx;
                break 'chunk_loop;
            }
            let sync_read = feetech::build_sync_read(chunk, feetech::REG_PRESENT_POSITION_L, 8)
                .map_err(|e| ArmDriverError::Protocol(e.to_string()))?;

            self.transport.write_all(&sync_read)?;
            self.transport.flush()?;
            let req_written_at = self.clock_now();
            self.drain_tx_echo(&sync_read)?;

            let expected_ids: std::collections::HashSet<u8> = chunk.iter().copied().collect();
            let mut replied_ids = std::collections::HashSet::new();
            const CHUNK_TIMEOUT: Duration = Duration::from_millis(4);

            while replied_ids.len() < chunk.len()
                && self.elapsed_since(req_written_at) < CHUNK_TIMEOUT
                && self.elapsed_since(start_time) < MAX_READ_DURATION
            {
                self.decode_position_frames_windowed(&expected_ids, &mut replied_ids, req_written_at, CHUNK_TIMEOUT);
                if replied_ids.len() >= chunk.len() {
                    break;
                }

                match self.transport.read(&mut temp_chunk) {
                    Ok(n) if n > 0 => {
                        let byte_rx_at = self.clock_now();
                        if byte_rx_at < req_written_at
                            || byte_rx_at.saturating_duration_since(req_written_at) >= CHUNK_TIMEOUT
                            || byte_rx_at.saturating_duration_since(start_time) >= MAX_READ_DURATION
                        {
                            break;
                        }
                        self.push_rx(&temp_chunk[..n], byte_rx_at);
                    }
                    _ => {}
                }

                self.decode_position_frames_windowed(&expected_ids, &mut replied_ids, req_written_at, CHUNK_TIMEOUT);

                if replied_ids.len() < chunk.len() {
                    self.clock_yield();
                }
            }

            if replied_ids.len() < chunk.len() {
                self.missed_poll_replies += 1;
                if let Some(tail) = feetech::trailing_partial_frame_for_sync_read(&self.rx_buffer, chunk, 10) {
                    let tail_offset = self.rx_buffer.len() - tail.len();
                    let tail_started_at = self
                        .rx_timestamps
                        .get(tail_offset)
                        .copied()
                        .unwrap_or(req_written_at);
                    if tail_started_at >= req_written_at
                        && tail_started_at.saturating_duration_since(req_written_at) < CHUNK_TIMEOUT
                    {
                        let tail_bytes = tail.to_vec();
                        let tail_ts = if self.rx_timestamps.len() == self.rx_buffer.len() {
                            self.rx_timestamps[tail_offset..].to_vec()
                        } else {
                            vec![tail_started_at; tail_bytes.len()]
                        };
                        self.partial_pos_rx = tail_bytes;
                        self.partial_pos_rx_timestamps = tail_ts;
                        self.incomplete_chunk_idx = Some(chunk_idx);
                        self.incomplete_frame_started_at = Some(tail_started_at);
                        self.clear_rx();
                        self.poll_cursor = (chunk_idx + 1) % num_chunks;
                        break 'chunk_loop;
                    }
                }
                self.poison_line();
            }
            self.incomplete_chunk_idx = None;
            self.incomplete_frame_started_at = None;
            self.partial_pos_rx.clear();
            self.partial_pos_rx_timestamps.clear();
            self.clear_rx();
            self.poll_cursor = (chunk_idx + 1) % num_chunks;
        }

        if self.rx_buffer.len() > 512 {
            let trim = self.rx_buffer.len() - 256;
            self.rx_buffer.drain(..trim);
        }

        self.last_read_release_id = self.control_release_id;
        Ok(self.cached_states.clone())
    }
}
