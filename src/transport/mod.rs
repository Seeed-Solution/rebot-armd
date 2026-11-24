use std::fmt;
use thiserror::Error;

pub mod mock;
pub mod serial;
pub mod slcan;

#[cfg(target_os = "linux")]
pub mod socketcan;

pub use mock::{MockCanTransport, MockClock, MockTransport};
pub use serial::{SerialTransport, SerialTransportConfig};
pub use slcan::{SlcanCodec, SlcanTransport};

#[cfg(target_os = "linux")]
pub use socketcan::SocketCanTransport;

/// Errors that can occur in the transport layer
#[derive(Debug, Error)]
pub enum TransportError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serial port error: {0}")]
    Serial(String),

    #[error("CAN error: {0}")]
    Can(String),

    #[error("Timeout error")]
    Timeout,

    #[error("Operation would block")]
    WouldBlock,

    #[error("Transmit queue full (ENOBUFS)")]
    NoBufferSpace,

    #[error("SocketCAN {op} failed: {}", format_os_error(*errno, *kind))]
    SocketCanOs {
        op: &'static str,
        errno: Option<i32>,
        kind: std::io::ErrorKind,
    },

    #[error("Invalid CAN frame ({reason}, id 0x{id:X})")]
    InvalidCanFrame {
        reason: &'static str,
        id: u32,
    },

    #[error("CAN bus error frame (id 0x{id:X})")]
    CanBusErrorFrame {
        id: u32,
    },

    #[error("Device not open")]
    NotOpen,

    #[error("Protocol error: {0}")]
    Protocol(String),

    #[error("Buffer overflow: {0}")]
    BufferOverflow(String),

    #[error("Checksum mismatch: expected 0x{expected:02X}, calculated 0x{actual:02X}")]
    ChecksumMismatch { expected: u8, actual: u8 },

    #[error("Frame too short: expected at least {expected} bytes, got {actual}")]
    FrameTooShort { expected: usize, actual: usize },
}

impl TransportError {
    /// Non-blocking TX queue backpressure (`EAGAIN`/`EWOULDBLOCK` or `ENOBUFS`): transient
    /// queue-full condition rather than a hard transport failure.
    #[inline]
    pub fn is_tx_backpressure(&self) -> bool {
        matches!(self, Self::WouldBlock | Self::NoBufferSpace)
    }
}

pub(crate) struct OsErrorDisplay {
    errno: Option<i32>,
    kind: std::io::ErrorKind,
}

#[cfg(unix)]
fn write_raw_os_error(code: i32, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let mut buf = [0u8; 128];
    // SAFETY: `buf` is a valid writable stack buffer of length `buf.len()`.
    // Rust's `libc::strerror_r` exposes the XSI `c_int` signature on glibc, musl, and macOS.
    let rc = unsafe { libc::strerror_r(code, buf.as_mut_ptr() as *mut libc::c_char, buf.len()) };
    if rc == 0 {
        let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        if let Ok(msg) = std::str::from_utf8(&buf[..len]) {
            if !msg.is_empty() {
                return write!(f, "{msg} (os error {code})");
            }
        }
    }
    write!(f, "os error {code}")
}

impl fmt::Display for OsErrorDisplay {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(code) = self.errno {
            #[cfg(unix)]
            {
                write_raw_os_error(code, f)
            }
            #[cfg(not(unix))]
            {
                write!(f, "{}", std::io::Error::from_raw_os_error(code))
            }
        } else {
            write!(f, "{:?}", self.kind)
        }
    }
}

pub(crate) fn format_os_error(errno: Option<i32>, kind: std::io::ErrorKind) -> OsErrorDisplay {
    OsErrorDisplay { errno, kind }
}

/// Standard 8-byte CAN frame definition
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub is_extended: bool,
    pub is_rtr: bool,
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CanFrame {
    /// Create a standard 11-bit ID data frame
    pub fn new_standard(id: u16, data_slice: &[u8]) -> Result<Self, TransportError> {
        if id > 0x7FF {
            return Err(TransportError::Can(format!(
                "Standard CAN ID out of range (0x{:X} > 0x7FF)",
                id
            )));
        }
        if data_slice.len() > 8 {
            return Err(TransportError::Can(format!(
                "CAN payload length exceeds 8 bytes: {}",
                data_slice.len()
            )));
        }
        let mut data = [0u8; 8];
        data[..data_slice.len()].copy_from_slice(data_slice);
        Ok(Self {
            id: id as u32,
            is_extended: false,
            is_rtr: false,
            dlc: data_slice.len() as u8,
            data,
        })
    }

    /// Create an extended 29-bit ID data frame
    pub fn new_extended(id: u32, data_slice: &[u8]) -> Result<Self, TransportError> {
        if id > 0x1FFFFFFF {
            return Err(TransportError::Can(format!(
                "Extended CAN ID out of range (0x{:X} > 0x1FFFFFFF)",
                id
            )));
        }
        if data_slice.len() > 8 {
            return Err(TransportError::Can(format!(
                "CAN payload length exceeds 8 bytes: {}",
                data_slice.len()
            )));
        }
        let mut data = [0u8; 8];
        data[..data_slice.len()].copy_from_slice(data_slice);
        Ok(Self {
            id,
            is_extended: true,
            is_rtr: false,
            dlc: data_slice.len() as u8,
            data,
        })
    }

    /// Create a standard Remote Transmission Request (RTR) frame
    pub fn new_rtr_standard(id: u16, dlc: u8) -> Self {
        Self {
            id: (id & 0x7FF) as u32,
            is_extended: false,
            is_rtr: true,
            dlc: dlc.min(8),
            data: [0u8; 8],
        }
    }

    /// Create an extended RTR frame
    pub fn new_rtr_extended(id: u32, dlc: u8) -> Self {
        Self {
            id: id & 0x1FFFFFFF,
            is_extended: true,
            is_rtr: true,
            dlc: dlc.min(8),
            data: [0u8; 8],
        }
    }

    /// Returns the active data slice based on DLC
    pub fn data_slice(&self) -> &[u8] {
        let len = (self.dlc as usize).min(8);
        &self.data[..len]
    }
}

impl fmt::Debug for CanFrame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "CanFrame {{ id: 0x{:X}, ext: {}, rtr: {}, dlc: {}, data: {:02X?} }}",
            self.id,
            self.is_extended,
            self.is_rtr,
            self.dlc,
            self.data_slice()
        )
    }
}

/// Monotonic clock and pacing seam for deterministic transport testing.
///
/// Production hardware transports return `None` from [`Transport::clock`],
/// so drivers use `Instant::now()`, `std::thread::sleep()`, and
/// `std::thread::yield_now()` directly with zero behavioral change.
pub trait DriverClock: Send + Sync {
    /// Unique identifier for this clock domain (used to detect transport replacement mismatches).
    fn clock_id(&self) -> u64;
    /// Current monotonic timestamp.
    fn now(&self) -> std::time::Instant;
    /// Sleep / advance by `duration`.
    fn sleep(&self, duration: std::time::Duration);
    /// Yield while polling for bytes.
    fn yield_now(&self);
}

/// Synchronous byte-stream hardware transport trait
pub trait Transport: Send {
    /// Optional deterministic clock supplied by mock/test transports.
    /// Production transports return `None` to use the real system clock.
    fn clock(&self) -> Option<std::sync::Arc<dyn DriverClock>> {
        None
    }

    /// Read incoming bytes into buffer
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write outgoing bytes from buffer
    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush pending writes to the underlying hardware
    fn flush(&mut self) -> Result<(), TransportError>;

    /// Discard unread bytes from the hardware input buffer if supported
    fn clear_input_buffer(&mut self) {}

    /// Write entire buffer until finished
    fn write_all(&mut self, mut buf: &[u8]) -> Result<(), TransportError> {
        while !buf.is_empty() {
            let n = self.write(buf)?;
            if n == 0 {
                return Err(TransportError::Io(std::io::Error::new(
                    std::io::ErrorKind::WriteZero,
                    "failed to write entire buffer to transport",
                )));
            }
            buf = &buf[n..];
        }
        Ok(())
    }

    /// Read exact number of bytes to fill buffer
    fn read_exact(&mut self, mut buf: &mut [u8]) -> Result<(), TransportError> {
        while !buf.is_empty() {
            let n = self.read(buf)?;
            if n == 0 {
                return Err(TransportError::Io(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "transport reached unexpected EOF or timeout while reading",
                )));
            }
            buf = &mut buf[n..];
        }
        Ok(())
    }
}

/// Asynchronous byte-stream hardware transport trait
#[allow(async_fn_in_trait)]
pub trait AsyncTransport: Send {
    async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;
    async fn write_async(&mut self, buf: &[u8]) -> Result<usize, TransportError>;
    async fn flush_async(&mut self) -> Result<(), TransportError>;

    async fn write_all_async(&mut self, mut buf: &[u8]) -> Result<(), TransportError> {
        while !buf.is_empty() {
            let n = self.write_async(buf).await?;
            if n == 0 {
                return Err(TransportError::Io(std::io::Error::new(
                    std::io::ErrorKind::WriteZero,
                    "failed to write entire buffer async",
                )));
            }
            buf = &buf[n..];
        }
        Ok(())
    }

    async fn read_exact_async(&mut self, mut buf: &mut [u8]) -> Result<(), TransportError> {
        while !buf.is_empty() {
            let n = self.read_async(buf).await?;
            if n == 0 {
                return Err(TransportError::Io(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "unexpected EOF while reading async",
                )));
            }
            buf = &mut buf[n..];
        }
        Ok(())
    }
}

/// Maximum permitted drift in `CLOCK_REALTIME - CLOCK_MONOTONIC` between `write_done`
/// (or window start) and decode before rest-pose evidence is discarded and restarted (1 ms).
pub const MAX_REALTIME_MONOTONIC_OFFSET_DRIFT_NS: i128 = 1_000_000;

/// Coherent sample of the transport's RX clock domain alongside the optional
/// `CLOCK_REALTIME - CLOCK_MONOTONIC` offset (in nanoseconds) when `rx_now` is
/// [`CanRxTime::KernelRealtimeNs`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxClockSample {
    pub rx_now: CanRxTime,
    pub realtime_minus_monotonic_ns: Option<i128>,
}

/// Kernel or transport receive-clock timestamp carried alongside a CAN frame.
///
/// Causal ordering after a torque-changing write compares `rx_time` against the
/// `write_done` timestamp captured on the **same** clock domain (`CanTransport::rx_clock_now()`).
/// Missing timestamps (`None`) or cross-domain comparisons always fail closed — host dequeue time
/// never substitutes for a kernel/transport receive timestamp.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanRxTime {
    /// Linux kernel software RX timestamp in nanoseconds since the Unix epoch (`CLOCK_REALTIME`),
    /// delivered via `SO_TIMESTAMPING` (`SCM_TIMESTAMPING` `ts[0]`) or `SO_TIMESTAMPNS`
    /// (`SCM_TIMESTAMPNS`).
    KernelRealtimeNs(i128),
    /// Deterministic transport/mock receive instant on the same clock domain as
    /// [`CanTransport::clock`] (or `std::time::Instant::now()`).
    Monotonic(std::time::Instant),
}

impl CanRxTime {
    /// True iff `self` is strictly later than `write_done + settle` on the same clock domain.
    pub fn is_after_settle(self, write_done: CanRxTime, settle: std::time::Duration) -> bool {
        match (self, write_done) {
            (CanRxTime::KernelRealtimeNs(rx_ns), CanRxTime::KernelRealtimeNs(tx_ns)) => {
                let settle_ns = settle.as_nanos() as i128;
                rx_ns > tx_ns.saturating_add(settle_ns)
            }
            (CanRxTime::Monotonic(rx), CanRxTime::Monotonic(tx)) => {
                rx.checked_duration_since(tx).is_some_and(|d| d > settle)
            }
            _ => false,
        }
    }

    /// Non-negative duration from `earlier` to `self` on the same clock domain (`None` across
    /// domains or if `self < earlier`).
    pub fn checked_duration_since(self, earlier: CanRxTime) -> Option<std::time::Duration> {
        match (self, earlier) {
            (CanRxTime::KernelRealtimeNs(later_ns), CanRxTime::KernelRealtimeNs(earlier_ns)) => {
                let diff = later_ns.checked_sub(earlier_ns)?;
                if diff < 0 {
                    return None;
                }
                let clamped = diff.min(i128::from(u64::MAX)) as u64;
                Some(std::time::Duration::from_nanos(clamped))
            }
            (CanRxTime::Monotonic(later), CanRxTime::Monotonic(earlier)) => {
                later.checked_duration_since(earlier)
            }
            _ => None,
        }
    }
}

/// CAN bus frame transport trait
pub trait CanTransport: Send {
    /// Optional deterministic clock supplied by test transports ([`DriverClock`]).
    /// Production transports return `None`: drivers use the system clock unchanged.
    fn clock(&self) -> Option<std::sync::Arc<dyn DriverClock>> {
        None
    }

    /// Current time on the external kernel receive-clock domain used to stamp RX frames in
    /// [`Self::receive_with_rx_time`].
    ///
    /// On Linux [`SocketCanTransport`], this reads `CLOCK_REALTIME` (matching kernel
    /// `SO_TIMESTAMPING` software RX / `SO_TIMESTAMPNS`). Transports whose RX timestamps live on
    /// the driver's monotonic clock domain (`DriverClock` / `Instant::now()`) return `None` so
    /// drivers reuse their cached clock with zero allocation.
    fn rx_clock_now(&self) -> Option<CanRxTime> {
        None
    }

    /// Coherent sample of the transport's RX clock together with `CLOCK_REALTIME - CLOCK_MONOTONIC`
    /// (in nanoseconds) when [`Self::rx_clock_now`] uses `KernelRealtimeNs`.
    ///
    /// Transports delivering [`CanRxTime::KernelRealtimeNs`] must override this (e.g.
    /// [`SocketCanTransport`]) so rest-pose decode can detect wall-clock steps `> 1 ms`.
    fn rx_clock_sample(&self) -> Option<RxClockSample> {
        self.rx_clock_now().map(|rx_now| RxClockSample {
            rx_now,
            realtime_minus_monotonic_ns: None,
        })
    }

    /// Transmit a CAN frame
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError>;

    /// Receive a CAN frame
    fn receive(&mut self) -> Result<CanFrame, TransportError>;

    /// Receive a CAN frame together with its optional transport-measured monotonic receive time
    /// (on the same clock domain as [`Self::clock`], or `Instant::now()` when `clock()` is `None`).
    /// Transports that do not capture arrival timestamps inherit the default (`None`); host
    /// dequeue time never counts as causal evidence for torque state, mode, or rest pose.
    fn receive_timestamped(
        &mut self,
    ) -> Result<(CanFrame, Option<std::time::Instant>), TransportError> {
        self.receive().map(|f| (f, None))
    }

    /// Receive a CAN frame together with its kernel or transport receive timestamp
    /// ([`CanRxTime`]) on the same clock domain as [`Self::rx_clock_now`].
    ///
    /// Linux [`SocketCanTransport`] overrides this to deliver the raw kernel receive timestamp
    /// from `SO_TIMESTAMPING` / `SO_TIMESTAMPNS` without converting through host dequeue time.
    /// Transports that return `None` provide no causal RX timestamp; host dequeue time never counts.
    fn receive_with_rx_time(
        &mut self,
    ) -> Result<(CanFrame, Option<CanRxTime>), TransportError> {
        self.receive_timestamped()
            .map(|(f, ts)| (f, ts.map(CanRxTime::Monotonic)))
    }

    /// Flush pending CAN frames
    fn flush(&mut self) -> Result<(), TransportError>;

    /// True when [`Self::flush`] is non-blocking / time-bounded (e.g. SocketCAN or in-memory
    /// test transports). Serial transports (`SlcanTransport`) default to `false` so bounded
    /// RT maintenance fails closed rather than stalling the executor thread.
    fn supports_bounded_flush(&self) -> bool {
        false
    }

    /// True when [`Self::flush`] performs no transport I/O and cannot block or consume TX budget
    /// (e.g. Linux SocketCAN, where each `transmit()` submits directly to the kernel socket).
    /// Transports with a real flush default to `false` so the driver enforces the tick TX
    /// start-cutoff before starting `flush()`.
    fn flush_is_noop(&self) -> bool {
        false
    }

    /// True when [`Self::receive`], [`Self::transmit`], and [`Self::flush`] are all non-blocking
    /// and safe to drive directly from the synchronous RT executor (`armd-control`).
    /// Blocking serial transports (`SlcanTransport`) default to `false`.
    fn supports_nonblocking_rt(&self) -> bool {
        false
    }
}
