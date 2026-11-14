use std::io::{Read, Write};
use std::time::Duration;
use serde::{Deserialize, Serialize};
use crate::transport::{AsyncTransport, Transport, TransportError};

#[cfg(target_os = "linux")]
#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
struct SerialRs485 {
    flags: u32,
    delay_rts_before_send: u32,
    delay_rts_after_send: u32,
    padding: [u32; 5],
}

#[cfg(target_os = "linux")]
const TIOCGRS485: libc::c_ulong = 0x542E;
#[cfg(target_os = "linux")]
const TIOCSRS485: libc::c_ulong = 0x542F;
#[cfg(target_os = "linux")]
const SER_RS485_ENABLED: u32 = 1 << 0;
#[cfg(target_os = "linux")]
const SER_RS485_RTS_ON_SEND: u32 = 1 << 1;
#[cfg(target_os = "linux")]
const SER_RS485_RTS_AFTER_SEND: u32 = 1 << 2;

/// Configuration for opening a serial port transport
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerialTransportConfig {
    /// Port path, e.g. "/dev/ttyUSB0", "/dev/cu.usbserial-110", "COM3"
    pub port: String,
    /// Baud rate, standard e.g. 115200, 1000000 (1 Mbps)
    pub baud_rate: u32,
    /// Read/write timeout duration
    pub timeout: Duration,
    /// Enable 1ms low-latency optimization (for FTDI/POSIX/Windows)
    pub low_latency_1ms: bool,
    /// RS-485 RTS-on-send control (None = auto: enabled iff port starts with "/dev/ttySC")
    #[serde(default)]
    pub rs485_rts_on_send: Option<bool>,
}

impl SerialTransportConfig {
    pub fn is_tty_sc(&self) -> bool {
        self.port.starts_with("/dev/ttySC")
    }

    pub fn resolve_rs485_rts_on_send(&self) -> bool {
        self.rs485_rts_on_send.unwrap_or_else(|| self.is_tty_sc())
    }
}

impl Default for SerialTransportConfig {
    fn default() -> Self {
        Self {
            port: String::new(),
            baud_rate: 115_200,
            timeout: Duration::from_millis(1),
            low_latency_1ms: true,
            rs485_rts_on_send: None,
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
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(config.timeout);

        #[cfg(target_os = "linux")]
        let (mut port, fd) = {
            use std::os::unix::io::AsRawFd;
            let native_port = builder.open_native().map_err(|e| {
                TransportError::Serial(format!(
                    "Failed to open serial port {}: {}",
                    normalized_port, e
                ))
            })?;
            let fd = native_port.as_raw_fd();
            let boxed: Box<dyn serialport::SerialPort> = Box::new(native_port);
            (boxed, fd)
        };

        #[cfg(not(target_os = "linux"))]
        let mut port = builder.open().map_err(|e| {
            TransportError::Serial(format!(
                "Failed to open serial port {}: {}",
                normalized_port, e
            ))
        })?;

        let _ = port.clear_break();

        #[cfg(target_os = "linux")]
        {
            let rs485_needed = config.resolve_rs485_rts_on_send();
            if rs485_needed {
                let mut rs485 = SerialRs485::default();
                if unsafe { libc::ioctl(fd, TIOCGRS485, &mut rs485) } < 0 {
                    rs485 = SerialRs485::default();
                }
                rs485.flags = (rs485.flags & !SER_RS485_RTS_AFTER_SEND) | SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
                rs485.delay_rts_before_send = 0;
                rs485.delay_rts_after_send = 0;
                let rc = unsafe { libc::ioctl(fd, TIOCSRS485, &rs485) };
                if rc < 0 {
                    let err = std::io::Error::last_os_error();
                    return Err(TransportError::Serial(format!(
                        "Failed to configure RS-485 (TIOCSRS485) on {}: {}",
                        normalized_port, err
                    )));
                }
            }

            if config.low_latency_1ms {
                Self::apply_low_latency_fd(fd, &mut *port, &normalized_port);
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            if config.low_latency_1ms {
                Self::apply_low_latency(&mut *port, &normalized_port);
            }
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
            rs485_rts_on_send: None,
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

    #[cfg(target_os = "linux")]
    fn apply_low_latency_fd(fd: std::os::unix::io::RawFd, port: &mut dyn serialport::SerialPort, _port_path: &str) {
        #[repr(C)]
        struct SerialStruct {
            line_type: libc::c_int,
            line: libc::c_int,
            port: libc::c_uint,
            irq: libc::c_int,
            flags: libc::c_int,
            xmit_fifo_size: libc::c_int,
            custom_divisor: libc::c_int,
            baud_base: libc::c_int,
            close_delay: libc::c_ushort,
            io_type: libc::c_char,
            reserved_char: [libc::c_char; 1],
            hub6: libc::c_int,
            closing_wait: libc::c_ushort,
            closing_wait2: libc::c_ushort,
            iomem_base: *mut libc::c_uchar,
            iomem_reg_shift: libc::c_ushort,
            port_high: libc::c_int,
            iomap_base: libc::c_ulong,
        }

        const TIOCGSERIAL: libc::c_ulong = 0x541E;
        const TIOCSSERIAL: libc::c_ulong = 0x541F;
        const ASYNC_LOW_LATENCY: libc::c_int = 0x2000;

        let mut ss: SerialStruct = unsafe { std::mem::zeroed() };
        let rc = unsafe { libc::ioctl(fd, TIOCGSERIAL, &mut ss) };
        if rc == 0 {
            ss.flags |= ASYNC_LOW_LATENCY;
            let set_rc = unsafe { libc::ioctl(fd, TIOCSSERIAL, &ss) };
            if set_rc != 0 {
                let err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                if err != libc::ENOTTY && err != libc::EINVAL && err != libc::EOPNOTSUPP {
                    // benign no-op
                }
            }
        } else {
            let err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
            if err != libc::ENOTTY && err != libc::EINVAL && err != libc::EOPNOTSUPP {
                // benign no-op
            }
        }

        if let Ok(settings) = port.read_carrier_detect() {
            let _ = settings;
        }
    }

    #[cfg(not(target_os = "linux"))]
    fn apply_low_latency(port: &mut dyn serialport::SerialPort, _port_path: &str) {
        if let Ok(settings) = port.read_carrier_detect() {
            let _ = settings;
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

    /// Check if port name is a SC16IS752 port (/dev/ttySC*)
    pub fn is_tty_sc(&self) -> bool {
        self.config.is_tty_sc()
    }

    /// Port name
    pub fn port_name(&self) -> &str {
        &self.config.port
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
