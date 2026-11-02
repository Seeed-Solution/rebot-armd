use std::fmt;
use thiserror::Error;

pub mod mock;
pub mod serial;
pub mod slcan;

#[cfg(target_os = "linux")]
pub mod socketcan;

pub use mock::{MockCanTransport, MockTransport};
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

/// Synchronous byte-stream hardware transport trait
pub trait Transport: Send {
    /// Read incoming bytes into buffer
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write outgoing bytes from buffer
    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush pending writes to the underlying hardware
    fn flush(&mut self) -> Result<(), TransportError>;

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

/// CAN bus frame transport trait
pub trait CanTransport: Send {
    /// Transmit a CAN frame
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError>;

    /// Receive a CAN frame
    fn receive(&mut self) -> Result<CanFrame, TransportError>;

    /// Flush pending CAN frames
    fn flush(&mut self) -> Result<(), TransportError>;
}
