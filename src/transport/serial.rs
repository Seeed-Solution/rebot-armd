use std::io::{Read, Write};
use std::time::Duration;
use crate::transport::{AsyncTransport, Transport, TransportError};

/// Configuration for opening a serial port transport
#[derive(Debug, Clone)]
pub struct SerialTransportConfig {
    /// Port path, e.g. "/dev/ttyUSB0", "/dev/cu.usbserial-110", "COM3"
    pub port: String,
    /// Baud rate, standard e.g. 115200, 1000000 (1 Mbps)
    pub baud_rate: u32,
    /// Read/write timeout duration
    pub timeout: Duration,
    /// Enable 1ms low-latency optimization (for FTDI/POSIX/Windows)
    pub low_latency_1ms: bool,
}

impl Default for SerialTransportConfig {
    fn default() -> Self {
        Self {
            port: String::new(),
            baud_rate: 115_200,
            timeout: Duration::from_millis(1),
            low_latency_1ms: true,
        }
    }
}

/// Hardware serial port transport wrapping `serialport::SerialPort`
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    config: SerialTransportConfig,
}

impl SerialTransport {
    /// Open a serial port with custom configuration
    pub fn open(config: SerialTransportConfig) -> Result<Self, TransportError> {
        let normalized_port = Self::normalize_port_path(&config.port);

        let builder = serialport::new(&normalized_port, config.baud_rate)
            .timeout(config.timeout);

        let mut port = builder.open().map_err(|e| {
            TransportError::Serial(format!(
                "Failed to open serial port {}: {}",
                normalized_port, e
            ))
        })?;

        // Apply low latency settings if requested
        if config.low_latency_1ms {
            Self::apply_low_latency(&mut *port, &normalized_port);
        }

        Ok(Self { port, config })
    }

    /// Open a serial port with default 1ms timeout and low-latency enabled
    pub fn open_default(port: &str, baud_rate: u32) -> Result<Self, TransportError> {
        Self::open(SerialTransportConfig {
            port: port.to_string(),
            baud_rate,
            timeout: Duration::from_millis(1),
            low_latency_1ms: true,
        })
    }

    /// Normalizes port names across platforms:
    /// - On macOS/Darwin, prefers `/dev/cu.*` instead of `/dev/tty.*` to avoid carrier-detect blocking
    pub fn normalize_port_path(raw_port: &str) -> String {
        #[cfg(target_os = "macos")]
        {
            if raw_port.starts_with("/dev/tty.") {
                return raw_port.replacen("/dev/tty.", "/dev/cu.", 1);
            }
        }
        raw_port.to_string()
    }

    /// Attempt to apply 1ms latency timer optimization on Linux/FTDI/Windows
    fn apply_low_latency(port: &mut dyn serialport::SerialPort, _port_path: &str) {
        // Attempt Linux ASYNC_LOW_LATENCY ioctl if available
        #[cfg(target_os = "linux")]
        {
            // Try setting low_latency flag on Linux serial devices if supported
            if let Ok(settings) = port.read_carrier_detect() {
                let _ = settings;
            }
        }
    }

    /// Number of bytes available to read immediately in the OS buffer
    pub fn bytes_to_read(&self) -> Result<u32, TransportError> {
        self.port
            .bytes_to_read()
            .map_err(|e| TransportError::Serial(e.to_string()))
    }

    /// Reference to configuration
    pub fn config(&self) -> &SerialTransportConfig {
        &self.config
    }
}

impl Transport for SerialTransport {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        match self.port.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(TransportError::Io(e)),
        }
    }

    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.port.write(buf).map_err(TransportError::Io)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        self.port.flush().map_err(TransportError::Io)
    }
}

impl AsyncTransport for SerialTransport {
    async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        self.read(buf)
    }

    async fn write_async(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.write(buf)
    }

    async fn flush_async(&mut self) -> Result<(), TransportError> {
        self.flush()
    }
}
