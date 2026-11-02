#[cfg(target_os = "linux")]
use socketcan::{
    CanDataFrame, CanFrame as SkCanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket,
    StandardId,
};
use crate::transport::{CanFrame, CanTransport, TransportError};

/// Linux native SocketCAN hardware transport
#[cfg(target_os = "linux")]
pub struct SocketCanTransport {
    socket: CanSocket,
    iface: String,
}

#[cfg(target_os = "linux")]
impl SocketCanTransport {
    /// Open a SocketCAN interface, e.g. "can0", "vcan0"
    pub fn open(iface: &str) -> Result<Self, TransportError> {
        let socket = CanSocket::open(iface).map_err(|e| {
            TransportError::Can(format!("Failed to open SocketCAN interface '{}': {}", iface, e))
        })?;
        // Set short 1ms read timeout to avoid stalling 50Hz/100Hz control loop
        let _ = socket.set_read_timeout(Some(std::time::Duration::from_millis(1)));

        Ok(Self {
            socket,
            iface: iface.to_string(),
        })
    }

    pub fn iface(&self) -> &str {
        &self.iface
    }

    pub fn set_timeout(&mut self, timeout: Option<std::time::Duration>) -> Result<(), TransportError> {
        self.socket
            .set_read_timeout(timeout)
            .map_err(|e| TransportError::Can(format!("Failed to set read timeout: {}", e)))
    }

    pub fn with_timeout(self, timeout: std::time::Duration) -> Result<Self, TransportError> {
        self.socket
            .set_read_timeout(Some(timeout))
            .map_err(|e| TransportError::Can(format!("Failed to set read timeout: {}", e)))?;
        Ok(self)
    }
}

#[cfg(target_os = "linux")]
impl CanTransport for SocketCanTransport {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let can_id = if frame.is_extended {
            Id::Extended(
                ExtendedId::new(frame.id & 0x1FFFFFFF)
                    .ok_or_else(|| TransportError::Can("Invalid extended ID".to_string()))?,
            )
        } else {
            Id::Standard(
                StandardId::new((frame.id & 0x7FF) as u16)
                    .ok_or_else(|| TransportError::Can("Invalid standard ID".to_string()))?,
            )
        };

        let sk_frame = CanDataFrame::new(can_id, frame.data_slice()).ok_or_else(|| {
            TransportError::Can("Failed to construct socketcan CanDataFrame".to_string())
        })?;

        self.socket
            .write_frame(&sk_frame)
            .map_err(|e| TransportError::Can(format!("SocketCAN write failed: {}", e)))?;

        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        let sk_frame = match self.socket.read_frame() {
            Ok(frame) => frame,
            Err(e) => {
                if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut {
                    return Err(TransportError::Timeout);
                }
                return Err(TransportError::Can(format!("SocketCAN read failed: {}", e)));
            }
        };

        match sk_frame {
            SkCanFrame::Data(df) => {
                let id = match df.id() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(eid) => eid.as_raw(),
                };
                let is_extended = df.is_extended();
                let data_slice = df.data();
                let dlc = data_slice.len() as u8;
                let mut data = [0u8; 8];
                data[..dlc as usize].copy_from_slice(data_slice);

                Ok(CanFrame {
                    id,
                    is_extended,
                    is_rtr: false,
                    dlc,
                    data,
                })
            }
            SkCanFrame::Remote(rf) => {
                let id = match rf.id() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(eid) => eid.as_raw(),
                };
                let is_extended = rf.is_extended();
                let dlc = rf.dlc() as u8;
                Ok(CanFrame {
                    id,
                    is_extended,
                    is_rtr: true,
                    dlc,
                    data: [0u8; 8],
                })
            }
            SkCanFrame::Error(ef) => {
                Err(TransportError::Can(format!("CAN bus error frame: {:?}", ef)))
            }
        }
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}
