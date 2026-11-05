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
    _iface: String,
}

#[cfg(target_os = "linux")]
impl SocketCanTransport {
    /// Open a SocketCAN interface, e.g. "can0", "vcan0"
    pub fn open(iface: &str) -> Result<Self, TransportError> {
        let socket = CanSocket::open(iface).map_err(|e| {
            TransportError::Can(format!("Failed to open SocketCAN interface '{}': {}", iface, e))
        })?;
        // Set non-blocking mode to ensure the real-time control loop never blocks on recv
        socket.set_nonblocking(true).map_err(|e| {
            TransportError::Can(format!("Failed to set SocketCAN non-blocking mode: {}", e))
        })?;
        let _ = socket.set_read_timeout(Some(std::time::Duration::from_millis(1)));

        #[cfg(target_os = "linux")]
        {
            use std::os::unix::io::AsRawFd;
            let fd = socket.as_raw_fd();
            unsafe {
                let zero: libc::c_int = 0;
                // Linux kernel CAN RAW socket options:
                // SOL_CAN_RAW = 101
                // CAN_RAW_FILTER = 1
                // CAN_RAW_ERR_FILTER = 2
                // CAN_RAW_LOOPBACK = 3
                // CAN_RAW_RECV_OWN_MSGS = 4
                let res1 = libc::setsockopt(
                    fd,
                    101, /* SOL_CAN_RAW */
                    4,   /* CAN_RAW_RECV_OWN_MSGS */
                    &zero as *const _ as *const libc::c_void,
                    std::mem::size_of_val(&zero) as libc::socklen_t,
                );
                if res1 < 0 {
                    let err = std::io::Error::last_os_error();
                    return Err(TransportError::Can(format!(
                        "Failed to set CAN_RAW_RECV_OWN_MSGS: {}",
                        err
                    )));
                }

                let res2 = libc::setsockopt(
                    fd,
                    101, /* SOL_CAN_RAW */
                    3,   /* CAN_RAW_LOOPBACK */
                    &zero as *const _ as *const libc::c_void,
                    std::mem::size_of_val(&zero) as libc::socklen_t,
                );
                if res2 < 0 {
                    let err = std::io::Error::last_os_error();
                    return Err(TransportError::Can(format!(
                        "Failed to set CAN_RAW_LOOPBACK: {}",
                        err
                    )));
                }
            }
        }

        Ok(Self {
            socket,
            _iface: iface.to_string(),
        })
    }
}

#[cfg(not(target_os = "linux"))]
pub struct SocketCanTransport;

#[cfg(not(target_os = "linux"))]
impl SocketCanTransport {
    pub fn open(_iface: &str) -> Result<Self, TransportError> {
        Err(TransportError::Can(
            "SocketCAN is only supported on Linux targets".to_string(),
        ))
    }
}

#[cfg(target_os = "linux")]
impl CanTransport for SocketCanTransport {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        let sk_frame = if frame.is_extended {
            let id = ExtendedId::new(frame.id)
                .ok_or_else(|| TransportError::Can(format!("Invalid extended CAN ID: {:#x}", frame.id)))?;
            CanDataFrame::new(id, &frame.data[..frame.dlc as usize])
                .ok_or_else(|| TransportError::Can("Failed to create extended CanDataFrame".to_string()))?
        } else {
            let id = StandardId::new(frame.id as u16)
                .ok_or_else(|| TransportError::Can(format!("Invalid standard CAN ID: {:#x}", frame.id)))?;
            CanDataFrame::new(id, &frame.data[..frame.dlc as usize])
                .ok_or_else(|| TransportError::Can("Failed to create standard CanDataFrame".to_string()))?
        };

        self.socket
            .write_frame(&SkCanFrame::Data(sk_frame))
            .map_err(|e| TransportError::Can(format!("SocketCAN send failed: {}", e)))?;

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

#[cfg(not(target_os = "linux"))]
impl CanTransport for SocketCanTransport {
    fn transmit(&mut self, _frame: &CanFrame) -> Result<(), TransportError> {
        Err(TransportError::Can(
            "SocketCAN is only supported on Linux targets".to_string(),
        ))
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        Err(TransportError::Can(
            "SocketCAN is only supported on Linux targets".to_string(),
        ))
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}
