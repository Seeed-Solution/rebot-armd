use std::collections::VecDeque;
use crate::transport::{AsyncTransport, CanFrame, CanTransport, Transport, TransportError};

/// Mock byte stream transport for unit testing and loopback verification
#[derive(Debug, Clone, Default)]
pub struct MockTransport {
    rx_buffer: VecDeque<u8>,
    tx_buffer: VecDeque<u8>,
    loopback: bool,
    chunk_size_limit: Option<usize>,
    error_on_next_read: Option<String>,
    error_on_next_write: Option<String>,
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_rx_data(bytes: &[u8]) -> Self {
        let mut transport = Self::new();
        transport.provide_rx(bytes);
        transport
    }

    pub fn provide_rx(&mut self, bytes: &[u8]) {
        self.rx_buffer.extend(bytes);
    }

    pub fn take_tx(&mut self) -> Vec<u8> {
        self.tx_buffer.drain(..).collect()
    }

    pub fn peek_tx(&self) -> Vec<u8> {
        self.tx_buffer.iter().copied().collect()
    }

    pub fn set_loopback(&mut self, loopback: bool) {
        self.loopback = loopback;
    }

    pub fn set_chunk_size_limit(&mut self, limit: Option<usize>) {
        self.chunk_size_limit = limit;
    }

    pub fn set_error_on_next_read(&mut self, err: Option<String>) {
        self.error_on_next_read = err;
    }

    pub fn set_error_on_next_write(&mut self, err: Option<String>) {
        self.error_on_next_write = err;
    }

    pub fn rx_len(&self) -> usize {
        self.rx_buffer.len()
    }

    pub fn tx_len(&self) -> usize {
        self.tx_buffer.len()
    }

    pub fn clear(&mut self) {
        self.rx_buffer.clear();
        self.tx_buffer.clear();
    }
}

impl Transport for MockTransport {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        if let Some(err_msg) = self.error_on_next_read.take() {
            return Err(TransportError::Io(std::io::Error::new(
                std::io::ErrorKind::Other,
                err_msg,
            )));
        }

        if buf.is_empty() {
            return Ok(0);
        }

        let max_read = if let Some(limit) = self.chunk_size_limit {
            buf.len().min(limit)
        } else {
            buf.len()
        };

        let available = self.rx_buffer.len().min(max_read);
        if available == 0 {
            return Ok(0);
        }

        for byte in buf.iter_mut().take(available) {
            *byte = self.rx_buffer.pop_front().unwrap();
        }

        Ok(available)
    }

    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        if let Some(err_msg) = self.error_on_next_write.take() {
            return Err(TransportError::Io(std::io::Error::new(
                std::io::ErrorKind::Other,
                err_msg,
            )));
        }

        self.tx_buffer.extend(buf);
        if self.loopback {
            self.rx_buffer.extend(buf);
        }

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

impl AsyncTransport for MockTransport {
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

/// Mock CAN bus frame transport for testing CAN controllers and protocols
#[derive(Debug, Clone, Default)]
pub struct MockCanTransport {
    rx_frames: VecDeque<CanFrame>,
    tx_frames: VecDeque<CanFrame>,
    loopback: bool,
    error_on_next_tx: Option<String>,
    error_on_next_rx: Option<String>,
}

impl MockCanTransport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn provide_rx_frame(&mut self, frame: CanFrame) {
        self.rx_frames.push_back(frame);
    }

    pub fn provide_rx_frames(&mut self, frames: &[CanFrame]) {
        for f in frames {
            self.rx_frames.push_back(*f);
        }
    }

    pub fn take_tx_frames(&mut self) -> Vec<CanFrame> {
        self.tx_frames.drain(..).collect()
    }

    pub fn peek_tx_frames(&self) -> Vec<CanFrame> {
        self.tx_frames.iter().copied().collect()
    }

    pub fn set_loopback(&mut self, loopback: bool) {
        self.loopback = loopback;
    }

    pub fn set_error_on_next_tx(&mut self, err: Option<String>) {
        self.error_on_next_tx = err;
    }

    pub fn set_error_on_next_rx(&mut self, err: Option<String>) {
        self.error_on_next_rx = err;
    }

    pub fn rx_len(&self) -> usize {
        self.rx_frames.len()
    }

    pub fn tx_len(&self) -> usize {
        self.tx_frames.len()
    }

    pub fn clear(&mut self) {
        self.rx_frames.clear();
        self.tx_frames.clear();
    }
}

impl CanTransport for MockCanTransport {
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), TransportError> {
        if let Some(err) = self.error_on_next_tx.take() {
            return Err(TransportError::Can(err));
        }

        self.tx_frames.push_back(*frame);
        if self.loopback {
            self.rx_frames.push_back(*frame);
        }
        Ok(())
    }

    fn receive(&mut self) -> Result<CanFrame, TransportError> {
        if let Some(err) = self.error_on_next_rx.take() {
            return Err(TransportError::Can(err));
        }

        self.rx_frames
            .pop_front()
            .ok_or(TransportError::Timeout)
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}
