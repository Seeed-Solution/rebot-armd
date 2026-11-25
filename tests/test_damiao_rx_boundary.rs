//! Receive-queue boundary of Damiao feedback: a reply that was queued before an instant must not
//! report a queue bound at or after that instant, even when the bounded per-tick drain decodes
//! it later. The motors are simulated by MockCanTransport (not a hardware verification).
use std::time::Instant;

use armd::driver::{ArmDriver, DamiaoArmDriver};
use armd::protocol::damiao::{self, DamiaoLimits};
use armd::transport::{CanFrame, MockCanTransport};

fn feedback_frame(id: u8, position: f32) -> CanFrame {
    let limits = DamiaoLimits::default();
    let p = damiao::float_to_uint(position, limits.p_min, limits.p_max, 16);
    let mut data = [0u8; 8];
    data[0] = (1 << 4) | (id & 0x0f);
    data[1] = (p >> 8) as u8;
    data[2] = p as u8;
    CanFrame::new_standard(0x10 + id as u16, &data).unwrap()
}

#[test]
fn backlog_decoded_after_fence_keeps_pre_fence_queue_bound() {
    let mut driver = DamiaoArmDriver::new(MockCanTransport::new(), vec![1, 2]);
    // Tick 0: an empty queue is observed (boundary before the backlog).
    driver.read_joint_states().unwrap();
    // A backlog larger than one tick's drain budget (64 frames), all queued before the fence;
    // the last two replies of joint 1 sit at its end.
    let mut backlog: Vec<CanFrame> = (0..70).map(|_| feedback_frame(2, 0.1)).collect();
    backlog.push(feedback_frame(1, 0.2));
    backlog.push(feedback_frame(1, 0.2));
    driver.can_mut().provide_rx_frames(&backlog);
    let fence = Instant::now();
    let seq0 = driver.feedback_seqs()[0];

    // Tick 1 decodes 64 frames, tick 2 the rest and both joint-1 replies, then sees the queue empty.
    driver.read_joint_states().unwrap();
    driver.read_joint_states().unwrap();
    assert_eq!(driver.feedback_seqs()[0].wrapping_sub(seq0), 2, "both old joint-1 replies were decoded");
    assert!(
        driver.feedback_instants()[0].is_some_and(|t| t <= fence),
        "S1: transport arrival stamps precede the fence even when decoded afterwards"
    );
    let bounds = driver.feedback_queue_bounds();
    assert_eq!(bounds.len(), 2);
    assert!(bounds[0].is_none_or(|b| b < fence), "a pre-fence reply reports a post-fence queue bound: {bounds:?}");
    assert!(bounds[1].is_none_or(|b| b < fence), "{bounds:?}");

    // A reply enqueued after tick 2 observed the empty queue is bounded after the fence.
    driver.can_mut().provide_rx_frame(feedback_frame(1, 0.2));
    driver.read_joint_states().unwrap();
    let bounds = driver.feedback_queue_bounds();
    assert!(bounds[0].is_some_and(|b| b >= fence), "{bounds:?}");
    assert!(bounds[1].is_none_or(|b| b < fence), "joint 2 got no fresh reply: {bounds:?}");
}
