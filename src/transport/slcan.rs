use crate::transport::{CanFrame, CanTransport, Transport, TransportError};

/// Lawicel SLCAN protocol encoder and decoder
pub struct SlcanCodec;

impl SlcanCodec {
    /// Format a CAN frame into SLCAN ASCII format terminated by `\r`
    pub fn encode_frame(frame: &CanFrame) -> String {
        let dlc = (frame.dlc as usize).min(8);
        let mut out = String::with_capacity(32);

        if frame.is_rtr {
            if frame.is_extended {
                out.push('R');
                out.push_str(&format!("{:08X}", frame.id & 0x1FFFFFFF));
            } else {
                out.push('r');
                out.push_str(&format!("{:03X}", frame.id & 0x7FF));
            }
            out.push_str(&format!("{:X}", dlc));
        } else if frame.is_extended {
            out.push('T');
            out.push_str(&format!("{:08X}", frame.id & 0x1FFFFFFF));
            out.push_str(&format!("{:X}", dlc));
            for b in &frame.data[..dlc] {
                out.push_str(&format!("{:02X}", b));
            }
        } else {
            out.push('t');
            out.push_str(&format!("{:03X}", frame.id & 0x7FF));
            out.push_str(&format!("{:X}", dlc));
            for b in &frame.data[..dlc] {
                out.push_str(&format!("{:02X}", b));
            }
        }

        out.push('\r');
        out
    }

    /// Decode an SLCAN ASCII string into a `CanFrame`
    pub fn decode_frame(raw_line: &str) -> Result<CanFrame, TransportError> {
        let trimmed_line = raw_line.trim_matches(|c| c == '\r' || c == '\n');
        if trimmed_line.is_empty() {
            return Err(TransportError::Protocol("Empty SLCAN frame".to_string()));
        }

        // Validate that if raw_line contained line terminators, there are no invalid trailing or embedded characters
        if raw_line.contains('\r') || raw_line.contains('\n') {
            let without_endings = raw_line.trim_end_matches(|c| c == '\r' || c == '\n');
            if without_endings != trimmed_line {
                return Err(TransportError::Protocol(format!(
                    "Malformed trailing characters in SLCAN frame: {:?}",
                    raw_line
                )));
            }
        }

        let line = trimmed_line;
        let bytes = line.as_bytes();
        let cmd = bytes[0] as char;

        match cmd {
            // Standard data frame: t<id:3><dlc:1><data:2*dlc>
            't' => {
                if line.len() < 5 {
                    return Err(TransportError::FrameTooShort {
                        expected: 5,
                        actual: line.len(),
                    });
                }
                let id = u16::from_str_radix(&line[1..4], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid standard CAN ID in SLCAN: {}", e))
                })?;
                let dlc = u8::from_str_radix(&line[4..5], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid DLC in SLCAN: {}", e))
                })?;
                if dlc > 8 {
                    return Err(TransportError::Can(format!(
                        "SLCAN DLC exceeds 8: {}",
                        dlc
                    )));
                }
                let expected_len = 5 + (dlc as usize) * 2;
                if line.len() < expected_len {
                    return Err(TransportError::FrameTooShort {
                        expected: expected_len,
                        actual: line.len(),
                    });
                }
                if line.len() > expected_len {
                    return Err(TransportError::Protocol(format!(
                        "Extra trailing characters in SLCAN frame: expected len {}, got {}",
                        expected_len,
                        line.len()
                    )));
                }

                let mut data = [0u8; 8];
                for i in 0..(dlc as usize) {
                    let byte_str = &line[5 + i * 2..7 + i * 2];
                    data[i] = u8::from_str_radix(byte_str, 16).map_err(|e| {
                        TransportError::Protocol(format!(
                            "Invalid data hex '{}' in SLCAN: {}",
                            byte_str, e
                        ))
                    })?;
                }

                Ok(CanFrame {
                    id: id as u32,
                    is_extended: false,
                    is_rtr: false,
                    dlc,
                    data,
                })
            }

            // Extended data frame: T<id:8><dlc:1><data:2*dlc>
            'T' => {
                if line.len() < 10 {
                    return Err(TransportError::FrameTooShort {
                        expected: 10,
                        actual: line.len(),
                    });
                }
                let id = u32::from_str_radix(&line[1..9], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid extended CAN ID in SLCAN: {}", e))
                })?;
                let dlc = u8::from_str_radix(&line[9..10], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid DLC in SLCAN: {}", e))
                })?;
                if dlc > 8 {
                    return Err(TransportError::Can(format!(
                        "SLCAN DLC exceeds 8: {}",
                        dlc
                    )));
                }
                let expected_len = 10 + (dlc as usize) * 2;
                if line.len() < expected_len {
                    return Err(TransportError::FrameTooShort {
                        expected: expected_len,
                        actual: line.len(),
                    });
                }
                if line.len() > expected_len {
                    return Err(TransportError::Protocol(format!(
                        "Extra trailing characters in SLCAN frame: expected len {}, got {}",
                        expected_len,
                        line.len()
                    )));
                }

                let mut data = [0u8; 8];
                for i in 0..(dlc as usize) {
                    let byte_str = &line[10 + i * 2..12 + i * 2];
                    data[i] = u8::from_str_radix(byte_str, 16).map_err(|e| {
                        TransportError::Protocol(format!(
                            "Invalid data hex '{}' in SLCAN: {}",
                            byte_str, e
                        ))
                    })?;
                }

                Ok(CanFrame {
                    id,
                    is_extended: true,
                    is_rtr: false,
                    dlc,
                    data,
                })
            }

            // Standard RTR frame: r<id:3><dlc:1>
            'r' => {
                if line.len() < 5 {
                    return Err(TransportError::FrameTooShort {
                        expected: 5,
                        actual: line.len(),
                    });
                }
                if line.len() > 5 {
                    return Err(TransportError::Protocol(format!(
                        "Extra trailing characters in SLCAN RTR frame: expected len 5, got {}",
                        line.len()
                    )));
                }
                let id = u16::from_str_radix(&line[1..4], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid standard CAN ID in SLCAN RTR: {}", e))
                })?;
                let dlc = u8::from_str_radix(&line[4..5], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid DLC in SLCAN RTR: {}", e))
                })?;
                if dlc > 8 {
                    return Err(TransportError::Can(format!(
                        "SLCAN DLC exceeds 8: {}",
                        dlc
                    )));
                }
                Ok(CanFrame::new_rtr_standard(id, dlc))
            }

            // Extended RTR frame: R<id:8><dlc:1>
            'R' => {
                if line.len() < 10 {
                    return Err(TransportError::FrameTooShort {
                        expected: 10,
                        actual: line.len(),
                    });
                }
                if line.len() > 10 {
                    return Err(TransportError::Protocol(format!(
                        "Extra trailing characters in SLCAN RTR frame: expected len 10, got {}",
                        line.len()
                    )));
                }
                let id = u32::from_str_radix(&line[1..9], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid extended CAN ID in SLCAN RTR: {}", e))
                })?;
                let dlc = u8::from_str_radix(&line[9..10], 16).map_err(|e| {
                    TransportError::Protocol(format!("Invalid DLC in SLCAN RTR: {}", e))
                })?;
                if dlc > 8 {
                    return Err(TransportError::Can(format!(
                        "SLCAN DLC exceeds 8: {}",
                        dlc
                    )));
                }
                Ok(CanFrame::new_rtr_extended(id, dlc))
            }

            _ => Err(TransportError::Protocol(format!(
                "Unknown or non-frame SLCAN command '{}'",
                cmd
            ))),
        }
    }
}

/// SLCAN transport wrapper adapting any byte stream `Transport` to `CanTransport`
pub struct SlcanTransport<T: Transport> {
    inner: T,
    rx_buffer: Vec<u8>,
}

impl<T: Transport> SlcanTransport<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            rx_buffer: Vec::with_capacity(128),
        }
    }

    /// Access the underlying transport
    pub fn inner_mut(&mut self) -> &mut T {
        &mut self.inner
    }

    /// Open CAN channel command ('O\r')
    pub fn open_channel(&mut self) -> Result<(), TransportError> {
        self.inner.write_all(b"O\r")?;
        self.inner.flush()
    }

    /// Close CAN channel command ('C\r')
    pub fn close_channel(&mut self) -> Result<(), TransportError> {
        self.inner.write_all(b"C\r")?;
        self.inner.flush()
    }

    /// Set bitrate command ('S0\r' to 'S8\r')
    /// Standard speeds: S4=125k, S5=250k, S6=500k, S8=1M
    pub fn set_bitrate(&mut self, rate_code: u8) -> Result<(), TransportError> {
        if rate_code > 8 {
            return Err(TransportError::Protocol(format!(
                "Invalid SLCAN bitrate code {} (must be 0-8)",
                rate_code
            )));
        }
        let cmd = format!("S{}\r", rate_code);
        self.inner.write_all(cmd.as_bytes())?;
        self.inner.flush()
    }
}

impl<T: Transport> CanTransport for SlcanTransport<T> {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let ascii = SlcanCodec::encode_frame(frame);
        self.inner.write_all(ascii.as_bytes())?;
        self.inner.flush()
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        let mut temp_buf = [0u8; 64];

        loop {
            // Check if there is already a full frame terminated with \r or \n in rx_buffer
            if let Some(pos) = self.rx_buffer.iter().position(|&b| b == b'\r' || b == b'\n') {
                let line_bytes: Vec<u8> = self.rx_buffer.drain(..=pos).collect();
                if let Ok(line) = std::str::from_utf8(&line_bytes) {
                    let trimmed = line.trim();
                    if !trimmed.is_empty() {
                        return SlcanCodec::decode_frame(trimmed);
                    }
                }
                continue;
            }

            // Read more data from inner transport
            let n = self.inner.read(&mut temp_buf)?;
            if n == 0 {
                return Err(TransportError::Timeout);
            }
            self.rx_buffer.extend_from_slice(&temp_buf[..n]);
        }
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        self.inner.flush()
    }
}
