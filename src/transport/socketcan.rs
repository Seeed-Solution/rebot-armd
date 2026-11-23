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
                .ok_or(TransportError::InvalidCanFrame { reason: "extended ID out of range", id: frame.id })?;
            CanDataFrame::new(id, frame.data_slice())
                .ok_or(TransportError::InvalidCanFrame { reason: "extended data frame rejected", id: frame.id })?
        } else {
            let id = StandardId::new(frame.id as u16)
                .ok_or(TransportError::InvalidCanFrame { reason: "standard ID out of range", id: frame.id })?;
            CanDataFrame::new(id, frame.data_slice())
                .ok_or(TransportError::InvalidCanFrame { reason: "standard data frame rejected", id: frame.id })?
        };

        self.socket
            .write_frame(&SkCanFrame::Data(sk_frame))
            .map_err(map_socketcan_tx_error)?;

        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        let sk_frame = match self.socket.read_frame() {
            Ok(frame) => frame,
            Err(e) => {
                if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut {
                    return Err(TransportError::Timeout);
                }
                return Err(TransportError::SocketCanOs {
                    op: "read",
                    errno: e.raw_os_error(),
                    kind: e.kind(),
                });
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
                let id = match ef.id() {
                    Id::Standard(sid) => sid.as_raw() as u32,
                    Id::Extended(eid) => eid.as_raw(),
                };
                Err(TransportError::CanBusErrorFrame { id })
            }
        }
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
}

/// Map a Linux SocketCAN transmit `io::Error` without heap allocation on any outcome
/// (`EAGAIN`/`EWOULDBLOCK`, `ENOBUFS`, or hard OS errors such as `ENETDOWN`).
#[cfg(target_os = "linux")]
pub fn map_socketcan_tx_error(e: std::io::Error) -> TransportError {
    if e.kind() == std::io::ErrorKind::WouldBlock {
        TransportError::WouldBlock
    } else if e.raw_os_error() == Some(libc::ENOBUFS) {
        TransportError::NoBufferSpace
    } else {
        TransportError::SocketCanOs {
            op: "send",
            errno: e.raw_os_error(),
            kind: e.kind(),
        }
    }
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;

    #[test]
    fn socketcan_tx_error_classifies_would_block_and_enobufs() {
        let wb = map_socketcan_tx_error(std::io::Error::from(std::io::ErrorKind::WouldBlock));
        assert!(matches!(wb, TransportError::WouldBlock));
        assert!(wb.is_tx_backpressure());

        let nb = map_socketcan_tx_error(std::io::Error::from_raw_os_error(libc::ENOBUFS));
        assert!(matches!(nb, TransportError::NoBufferSpace));
        assert!(nb.is_tx_backpressure());

        let netdown = map_socketcan_tx_error(std::io::Error::from_raw_os_error(libc::ENETDOWN));
        assert!(matches!(
            netdown,
            TransportError::SocketCanOs { op: "send", errno: Some(libc::ENETDOWN), .. }
        ));
        assert!(!netdown.is_tx_backpressure());
        assert!(netdown.to_string().contains("SocketCAN send failed"), "{netdown}");

        let other = map_socketcan_tx_error(std::io::Error::from(std::io::ErrorKind::BrokenPipe));
        assert!(matches!(
            other,
            TransportError::SocketCanOs { op: "send", kind: std::io::ErrorKind::BrokenPipe, .. }
        ));
        assert!(!other.is_tx_backpressure());
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

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
}
