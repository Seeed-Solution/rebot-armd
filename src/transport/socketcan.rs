#[cfg(target_os = "linux")]
use socketcan::{
    CanDataFrame, CanFrame as SkCanFrame, CanSocket, EmbeddedFrame, ExtendedId, Socket,
    StandardId,
};
use crate::transport::{CanFrame, CanRxTime, CanTransport, TransportError};

/// Linux kernel software RX timestamping flags (`linux/net_tstamp.h`).
#[cfg(target_os = "linux")]
const SOF_TIMESTAMPING_RX_SOFTWARE: libc::c_int = 1 << 3;
#[cfg(target_os = "linux")]
const SOF_TIMESTAMPING_SOFTWARE: libc::c_int = 1 << 4;

/// Linux CAN ID flag bits (`linux/can.h`).
#[cfg(target_os = "linux")]
const CAN_EFF_FLAG: u32 = 0x8000_0000;
#[cfg(target_os = "linux")]
const CAN_RTR_FLAG: u32 = 0x4000_0000;
#[cfg(target_os = "linux")]
const CAN_ERR_FLAG: u32 = 0x2000_0000;
#[cfg(target_os = "linux")]
const CAN_SFF_MASK: u32 = 0x0000_07FF;
#[cfg(target_os = "linux")]
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;

/// Aligned stack buffer for SocketCAN `recvmsg` ancillary control messages (zero heap allocation).
#[cfg(target_os = "linux")]
#[repr(C, align(8))]
struct CmsgBuf([u8; 128]);

/// Read Linux `CLOCK_REALTIME` in nanoseconds since the Unix epoch — the exact kernel clock
/// domain used by `SO_TIMESTAMPING` (software RX) and `SO_TIMESTAMPNS` (`ktime_get_real_ts64`).
#[cfg(target_os = "linux")]
fn read_kernel_realtime_ns() -> Option<i128> {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    let rc = unsafe { libc::clock_gettime(libc::CLOCK_REALTIME, &mut ts) };
    if rc != 0 {
        return None;
    }
    timespec_to_ns(ts)
}

#[cfg(target_os = "linux")]
#[inline]
fn timespec_to_ns(ts: libc::timespec) -> Option<i128> {
    if ts.tv_sec < 0 || ts.tv_nsec < 0 || ts.tv_nsec >= 1_000_000_000 {
        return None;
    }
    if ts.tv_sec == 0 && ts.tv_nsec == 0 {
        return None;
    }
    Some(i128::from(ts.tv_sec) * 1_000_000_000 + i128::from(ts.tv_nsec))
}

/// Convert a kernel `CLOCK_REALTIME` receive timestamp to a monotonic `Instant` anchored at
/// `(mono_now, real_now_ns)`. Returns `None` if `real_now_ns` is unavailable or `rx_ns` lies
/// unreasonably ahead of `real_now_ns`.
#[cfg(target_os = "linux")]
fn kernel_realtime_ns_to_monotonic(
    rx_ns: i128,
    mono_now: std::time::Instant,
    real_now_ns: Option<i128>,
) -> Option<std::time::Instant> {
    let now_ns = real_now_ns?;
    if now_ns >= rx_ns {
        let age_ns = (now_ns - rx_ns).min(i128::from(u64::MAX)) as u64;
        mono_now.checked_sub(std::time::Duration::from_nanos(age_ns))
    } else {
        // Minor skew or backward wall-clock step: clamp to `mono_now` rather than fabricating a
        // future instant; causal torque checks compare raw `KernelRealtimeNs` directly.
        Some(mono_now)
    }
}

/// Parse kernel RX timestamp (nanoseconds since Unix epoch) from a `recvmsg` control message chain.
/// Supports `SCM_TIMESTAMPING` (software RX `ts[0]`) and `SCM_TIMESTAMPNS` (`ts`).
/// Fails closed (`None`) on `MSG_CTRUNC`, invalid `timespec`, or missing cmsg.
#[cfg(target_os = "linux")]
unsafe fn parse_cmsg_rx_timestamp_ns(msg: &libc::msghdr) -> Option<i128> {
    if (msg.msg_flags & libc::MSG_CTRUNC) != 0 {
        return None;
    }
    let ts_size = std::mem::size_of::<libc::timespec>();
    let min_tsing_len = libc::CMSG_LEN((3 * ts_size) as u32) as usize;
    let min_tsns_len = libc::CMSG_LEN(ts_size as u32) as usize;

    let mut cmsg = libc::CMSG_FIRSTHDR(msg);
    let mut fallback_tsns: Option<i128> = None;
    while !cmsg.is_null() {
        let hdr = &*cmsg;
        let cmsg_len = hdr.cmsg_len as usize;
        if hdr.cmsg_level == libc::SOL_SOCKET {
            if hdr.cmsg_type == libc::SO_TIMESTAMPING && cmsg_len >= min_tsing_len {
                let data_ptr = libc::CMSG_DATA(cmsg) as *const [libc::timespec; 3];
                let tss = std::ptr::read_unaligned(data_ptr);
                // Software RX timestamp is slot [0] (`SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE`).
                if let Some(ns) = timespec_to_ns(tss[0]) {
                    return Some(ns);
                }
            } else if hdr.cmsg_type == libc::SO_TIMESTAMPNS && cmsg_len >= min_tsns_len {
                let data_ptr = libc::CMSG_DATA(cmsg) as *const libc::timespec;
                let ts = std::ptr::read_unaligned(data_ptr);
                if let Some(ns) = timespec_to_ns(ts) {
                    fallback_tsns = Some(ns);
                }
            }
        }
        cmsg = libc::CMSG_NXTHDR(msg, cmsg);
    }
    fallback_tsns
}

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

                // S1: Enable kernel software RX timestamps (`SO_TIMESTAMPING`, fallback `SO_TIMESTAMPNS`)
                // so every received frame carries its kernel arrival time rather than host dequeue time.
                let ts_flags: libc::c_int =
                    SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
                let res_tsing = libc::setsockopt(
                    fd,
                    libc::SOL_SOCKET,
                    libc::SO_TIMESTAMPING,
                    &ts_flags as *const _ as *const libc::c_void,
                    std::mem::size_of_val(&ts_flags) as libc::socklen_t,
                );
                if res_tsing < 0 {
                    let one: libc::c_int = 1;
                    let res_tsns = libc::setsockopt(
                        fd,
                        libc::SOL_SOCKET,
                        libc::SO_TIMESTAMPNS,
                        &one as *const _ as *const libc::c_void,
                        std::mem::size_of_val(&one) as libc::socklen_t,
                    );
                    if res_tsns < 0 {
                        let err = std::io::Error::last_os_error();
                        return Err(TransportError::Can(format!(
                            "Failed to enable SocketCAN kernel RX timestamps (SO_TIMESTAMPING/SO_TIMESTAMPNS): {}",
                            err
                        )));
                    }
                }
            }
        }

        Ok(Self {
            socket,
            _iface: iface.to_string(),
        })
    }

    /// Zero-allocation non-blocking `recvmsg` reading one SocketCAN frame plus its kernel
    /// software RX timestamp (nanoseconds since Unix epoch on `CLOCK_REALTIME`).
    fn receive_raw_with_kernel_ns(&mut self) -> Result<(CanFrame, Option<i128>), TransportError> {
        use std::os::unix::io::AsRawFd;
        let fd = self.socket.as_raw_fd();
        let mut raw_frame: libc::can_frame = unsafe { std::mem::zeroed() };
        let mut cmsg_buf = CmsgBuf([0u8; 128]);
        let mut iov = libc::iovec {
            iov_base: (&mut raw_frame as *mut libc::can_frame).cast::<libc::c_void>(),
            iov_len: std::mem::size_of::<libc::can_frame>(),
        };

        let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
        msg.msg_iov = &mut iov;
        msg.msg_iovlen = 1 as _;
        msg.msg_control = cmsg_buf.0.as_mut_ptr().cast::<libc::c_void>();
        msg.msg_controllen = std::mem::size_of_val(&cmsg_buf.0) as _;

        let n = loop {
            let rc = unsafe { libc::recvmsg(fd, &mut msg, libc::MSG_DONTWAIT) };
            if rc < 0 {
                let err = std::io::Error::last_os_error();
                if err.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                if err.kind() == std::io::ErrorKind::WouldBlock
                    || err.kind() == std::io::ErrorKind::TimedOut
                {
                    return Err(TransportError::Timeout);
                }
                return Err(TransportError::SocketCanOs {
                    op: "read",
                    errno: err.raw_os_error(),
                    kind: err.kind(),
                });
            }
            break rc;
        };

        if (n as usize) != std::mem::size_of::<libc::can_frame>() {
            return Err(TransportError::SocketCanOs {
                op: "read",
                errno: None,
                kind: std::io::ErrorKind::InvalidData,
            });
        }

        let raw_id = raw_frame.can_id;
        if (raw_id & CAN_ERR_FLAG) != 0 {
            return Err(TransportError::CanBusErrorFrame {
                id: raw_id & CAN_EFF_MASK,
            });
        }

        let is_extended = (raw_id & CAN_EFF_FLAG) != 0;
        let is_rtr = (raw_id & CAN_RTR_FLAG) != 0;
        let id = if is_extended {
            raw_id & CAN_EFF_MASK
        } else {
            raw_id & CAN_SFF_MASK
        };
        let dlc = raw_frame.can_dlc.min(8);
        let mut data = [0u8; 8];
        if !is_rtr {
            data[..usize::from(dlc)].copy_from_slice(&raw_frame.data[..usize::from(dlc)]);
        }

        let kernel_ns = unsafe { parse_cmsg_rx_timestamp_ns(&msg) };
        Ok((
            CanFrame {
                id,
                is_extended,
                is_rtr,
                dlc,
                data,
            },
            kernel_ns,
        ))
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
    fn rx_clock_now(&self) -> Option<CanRxTime> {
        read_kernel_realtime_ns().map(CanRxTime::KernelRealtimeNs)
    }

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
        self.receive_raw_with_kernel_ns().map(|(f, _)| f)
    }

    fn receive_timestamped(
        &mut self,
    ) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        let (frame, kernel_ns) = self.receive_raw_with_kernel_ns()?;
        let ts = kernel_ns.and_then(|rx_ns| {
            kernel_realtime_ns_to_monotonic(
                rx_ns,
                std::time::Instant::now(),
                read_kernel_realtime_ns(),
            )
        });
        Ok((frame, ts))
    }

    fn receive_with_rx_time(&mut self) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        let (frame, kernel_ns) = self.receive_raw_with_kernel_ns()?;
        Ok((frame, kernel_ns.map(CanRxTime::KernelRealtimeNs)))
    }

    /// No-op on SocketCAN: `transmit()` only enqueues to the kernel socket, so `flush()` does
    /// not wait for wire completion and is NOT a causal RX boundary.
    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn flush_is_noop(&self) -> bool {
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

    #[test]
    fn socketcan_parse_cmsg_extracts_so_timestamping_and_so_timestampns() {
        let mut buf = CmsgBuf([0u8; 128]);
        let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
        msg.msg_control = buf.0.as_mut_ptr().cast::<libc::c_void>();
        msg.msg_controllen = std::mem::size_of_val(&buf.0) as _;

        // 1. SCM_TIMESTAMPING software RX in slot [0].
        unsafe {
            let cmsg = libc::CMSG_FIRSTHDR(&msg);
            assert!(!cmsg.is_null());
            let ts_arr = [
                libc::timespec { tv_sec: 1_700_000_010, tv_nsec: 123_456_789 },
                libc::timespec { tv_sec: 0, tv_nsec: 0 },
                libc::timespec { tv_sec: 0, tv_nsec: 0 },
            ];
            (*cmsg).cmsg_level = libc::SOL_SOCKET;
            (*cmsg).cmsg_type = libc::SO_TIMESTAMPING;
            (*cmsg).cmsg_len = libc::CMSG_LEN((std::mem::size_of_val(&ts_arr)) as u32) as _;
            std::ptr::write_unaligned(libc::CMSG_DATA(cmsg).cast::<[libc::timespec; 3]>(), ts_arr);
            msg.msg_controllen = libc::CMSG_SPACE((std::mem::size_of_val(&ts_arr)) as u32) as _;

            let parsed = parse_cmsg_rx_timestamp_ns(&msg);
            assert_eq!(parsed, Some(1_700_000_010_i128 * 1_000_000_000 + 123_456_789));

            // Truncated control buffer must fail closed (None).
            msg.msg_flags = libc::MSG_CTRUNC;
            assert_eq!(parse_cmsg_rx_timestamp_ns(&msg), None);
        }

        // 2. SCM_TIMESTAMPNS fallback.
        let mut buf2 = CmsgBuf([0u8; 128]);
        let mut msg2: libc::msghdr = unsafe { std::mem::zeroed() };
        msg2.msg_control = buf2.0.as_mut_ptr().cast::<libc::c_void>();
        msg2.msg_controllen = std::mem::size_of_val(&buf2.0) as _;
        unsafe {
            let cmsg = libc::CMSG_FIRSTHDR(&msg2);
            let ts = libc::timespec { tv_sec: 1_700_000_020, tv_nsec: 987_654_321 };
            (*cmsg).cmsg_level = libc::SOL_SOCKET;
            (*cmsg).cmsg_type = libc::SO_TIMESTAMPNS;
            (*cmsg).cmsg_len = libc::CMSG_LEN(std::mem::size_of_val(&ts) as u32) as _;
            std::ptr::write_unaligned(libc::CMSG_DATA(cmsg).cast::<libc::timespec>(), ts);
            msg2.msg_controllen = libc::CMSG_SPACE(std::mem::size_of_val(&ts) as u32) as _;

            let parsed = parse_cmsg_rx_timestamp_ns(&msg2);
            assert_eq!(parsed, Some(1_700_000_020_i128 * 1_000_000_000 + 987_654_321));
        }

        // 3. Empty control buffer -> None (never host dequeue time).
        let msg3: libc::msghdr = unsafe { std::mem::zeroed() };
        assert_eq!(unsafe { parse_cmsg_rx_timestamp_ns(&msg3) }, None);

        // 4. Mapping kernel realtime RX timestamp back to monotonic Instant preserves 25 ms age.
        let mono_now = std::time::Instant::now();
        let real_now_ns: i128 = 1_700_000_100_000_000_000;
        let rx_25ms_ago_ns = real_now_ns - 25_000_000;
        let mapped = kernel_realtime_ns_to_monotonic(rx_25ms_ago_ns, mono_now, Some(real_now_ns))
            .expect("25ms age maps to monotonic instant");
        assert_eq!(mono_now.duration_since(mapped), std::time::Duration::from_millis(25));
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

    /// No-op on SocketCAN: `transmit()` only enqueues to the kernel socket, so `flush()` does
    /// not wait for wire completion and is NOT a causal RX boundary.
    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn flush_is_noop(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }
}
