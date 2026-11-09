#ifndef _ARUCO_CAPTURE_TIME_H_
#define _ARUCO_CAPTURE_TIME_H_

#include <cstdint>

namespace aruco {

/**
 * Unix-epoch milliseconds at which a frame was captured.
 *
 * af::VideoFrame::ts_ms is CLOCK_MONOTONIC milliseconds truncated to 32 bits,
 * stamped when the frame is grabbed from the camera. The age of the frame is
 * the wrap-safe difference between the current monotonic time and that stamp;
 * subtract it from the current wall-clock time.
 */
inline uint64_t capture_unix_ms(uint64_t now_unix_ms, uint32_t now_mono_ms, uint32_t frame_ts_ms) {
    const uint32_t age_ms = now_mono_ms - frame_ts_ms;  // unsigned wrap is intended
    return age_ms > now_unix_ms ? now_unix_ms : now_unix_ms - age_ms;
}

}  // namespace aruco

#endif  // _ARUCO_CAPTURE_TIME_H_
