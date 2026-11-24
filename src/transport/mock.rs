use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::transport::{AsyncTransport, CanFrame, CanTransport, DriverClock, Transport, TransportError};

static NEXT_MOCK_CLOCK_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug)]
struct MockClockInner {
    id: u64,
    base: Instant,
    elapsed_ns: AtomicU64,
    idle_read_step_ns: AtomicU64,
}

/// Deterministic virtual clock for mock transports.
#[derive(Clone, Debug)]
pub struct MockClock {
    inner: Arc<MockClockInner>,
}

impl Default for MockClock {
    fn default() -> Self {
        Self::new()
    }
}

impl MockClock {
    pub fn new() -> Self {
        Self::starting_at(Instant::now())
    }

    pub fn starting_at(base: Instant) -> Self {
        Self {
            inner: Arc::new(MockClockInner {
                id: NEXT_MOCK_CLOCK_ID.fetch_add(1, Ordering::Relaxed),
                base,
                elapsed_ns: AtomicU64::new(0),
                idle_read_step_ns: AtomicU64::new(1_000_000),
            }),
        }
    }

    pub fn base(&self) -> Instant {
        self.inner.base
    }

    pub fn elapsed(&self) -> Duration {
        Duration::from_nanos(self.inner.elapsed_ns.load(Ordering::Acquire))
    }

    pub fn now(&self) -> Instant {
        self.inner
            .base
            .checked_add(self.elapsed())
            .expect("MockClock virtual time exceeded representable Instant")
    }

    pub fn advance(&self, duration: Duration) {
        let delta_ns = u64::try_from(duration.as_nanos())
            .expect("MockClock advance duration exceeds representable u64 nanoseconds");
        self.inner
            .elapsed_ns
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |cur| {
                let next = cur.checked_add(delta_ns)?;
                self.inner.base.checked_add(Duration::from_nanos(next))?;
                Some(next)
            })
            .expect("MockClock virtual time exhausted / overflowed representable Instant");
    }

    pub fn set_idle_read_step(&self, step: Duration) {
        assert!(
            !step.is_zero(),
            "MockClock idle_read_step must be non-zero to guarantee polling termination"
        );
        let step_ns = u64::try_from(step.as_nanos())
            .expect("MockClock idle_read_step exceeds u64 nanoseconds");
        assert!(
            self.inner
                .base
                .checked_add(Duration::from_nanos(step_ns))
                .is_some(),
            "MockClock idle_read_step exceeds representable Instant"
        );
        self.inner.idle_read_step_ns.store(step_ns, Ordering::Release);
    }

    pub fn idle_read_step(&self) -> Duration {
        Duration::from_nanos(self.inner.idle_read_step_ns.load(Ordering::Acquire))
    }

    pub(crate) fn advance_idle_read(&self) {
        self.advance(self.idle_read_step());
    }
}

impl DriverClock for MockClock {
    fn clock_id(&self) -> u64 {
        self.inner.id
    }

    fn now(&self) -> Instant {
        MockClock::now(self)
    }

    fn sleep(&self, duration: Duration) {
        self.advance(duration);
    }

    fn yield_now(&self) {}
}

/// Mock byte stream transport for unit testing and loopback verification
#[derive(Debug, Clone, Default)]
pub struct MockTransport {
    rx_buffer: VecDeque<u8>,
    tx_buffer: VecDeque<u8>,
    loopback: bool,
    chunk_size_limit: Option<usize>,
    error_on_next_read: Option<String>,
    error_on_next_write: Option<String>,
    write_responses: Vec<(Vec<u8>, Vec<u8>)>,
    staged_reads: VecDeque<Vec<u8>>,
    clear_input_count: usize,
    preserve_on_clear: bool,
    clock: MockClock,
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn clock(&self) -> MockClock {
        self.clock.clone()
    }

    pub fn with_clock(mut self, clock: MockClock) -> Self {
        self.clock = clock;
        self
    }

    pub fn advance_clock(&self, duration: Duration) {
        self.clock.advance(duration);
    }

    pub fn set_idle_read_step(&self, step: Duration) {
        self.clock.set_idle_read_step(step);
    }

    pub fn new_preserving() -> Self {
        let mut transport = Self::new();
        transport.preserve_on_clear = true;
        transport
    }

    pub fn with_rx_data(bytes: &[u8]) -> Self {
        let mut transport = Self::new();
        transport.preserve_on_clear = true;
        transport.provide_rx(bytes);
        transport
    }

    /// Opt-in response generated only after the matching write, for handshake tests.
    pub fn respond_to_write(&mut self, request: &[u8], response: &[u8]) {
        assert!(!request.is_empty());
        self.write_responses.push((request.to_vec(), response.to_vec()));
    }

    pub fn stage_reply(&mut self, request: &[u8], response: &[u8]) {
        self.respond_to_write(request, response);
    }

    pub fn clear_write_responses(&mut self) {
        self.write_responses.clear();
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

    /// Opt-in queue of staged RX byte chunks returned in sequence across consecutive reads.
    pub fn queue_staged_read(&mut self, chunk: &[u8]) {
        self.staged_reads.push_back(chunk.to_vec());
    }

    pub fn clear(&mut self) {
        self.rx_buffer.clear();
        self.tx_buffer.clear();
        self.staged_reads.clear();
    }

    pub fn clear_input_count(&self) -> usize {
        self.clear_input_count
    }

    pub fn set_preserve_on_clear(&mut self, preserve: bool) {
        self.preserve_on_clear = preserve;
    }
}

impl Transport for MockTransport {
    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        Some(Arc::new(self.clock.clone()))
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        if let Some(err_msg) = self.error_on_next_read.take() {
            if !buf.is_empty() {
                self.clock.advance_idle_read();
            }
            return Err(TransportError::Io(std::io::Error::other(
                err_msg,
            )));
        }

        if buf.is_empty() {
            return Ok(0);
        }

        if let Some(mut chunk) = self.staged_reads.pop_front() {
            let max_read = if let Some(limit) = self.chunk_size_limit {
                buf.len().min(limit)
            } else {
                buf.len()
            };
            let n = chunk.len().min(max_read);
            buf[..n].copy_from_slice(&chunk[..n]);
            if n < chunk.len() {
                chunk.drain(..n);
                self.staged_reads.push_front(chunk);
            }
            if n == 0 {
                self.clock.advance_idle_read();
            }
            return Ok(n);
        }

        let max_read = if let Some(limit) = self.chunk_size_limit {
            buf.len().min(limit)
        } else {
            buf.len()
        };

        let available = self.rx_buffer.len().min(max_read);
        if available == 0 {
            self.clock.advance_idle_read();
            return Ok(0);
        }

        for byte in buf.iter_mut().take(available) {
            *byte = self.rx_buffer.pop_front().unwrap();
        }

        Ok(available)
    }

    fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        if let Some(err_msg) = self.error_on_next_write.take() {
            return Err(TransportError::Io(std::io::Error::other(
                err_msg,
            )));
        }

        self.tx_buffer.extend(buf);
        if self.loopback {
            self.rx_buffer.extend(buf);
        }

        for (request, response) in &self.write_responses {
            if buf.windows(request.len()).any(|window| window == request) {
                self.rx_buffer.extend(response);
            }
        }

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn clear_input_buffer(&mut self) {
        self.clear_input_count += 1;
        if !self.preserve_on_clear {
            self.rx_buffer.clear();
            self.staged_reads.clear();
        }
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
    transmit_responses: Vec<(CanFrame, CanFrame)>,
    clock: Option<MockClock>,
}

impl MockCanTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Attach a deterministic virtual clock so drivers using this mock transport
    /// are completely independent of host wall-clock scheduling.
    pub fn with_clock(mut self, clock: MockClock) -> Self {
        self.clock = Some(clock);
        self
    }

    pub fn mock_clock(&self) -> Option<&MockClock> {
        self.clock.as_ref()
    }

    /// Opt-in response generated after a matching transmit, never before it.
    pub fn respond_to_transmit(&mut self, request: CanFrame, response: CanFrame) {
        self.transmit_responses.push((request, response));
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

        for (request, response) in &self.transmit_responses {
            if frame == request {
                self.rx_frames.push_back(*response);
            }
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

    fn supports_bounded_flush(&self) -> bool {
        true
    }

    fn flush_is_noop(&self) -> bool {
        true
    }

    fn supports_nonblocking_rt(&self) -> bool {
        true
    }

    fn clock(&self) -> Option<Arc<dyn DriverClock>> {
        self.clock.clone().map(|c| Arc::new(c) as Arc<dyn DriverClock>)
    }
}
