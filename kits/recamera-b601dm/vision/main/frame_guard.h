// rebot-vision: exception containment for the per-frame callback. Portable (no SDK).
#ifndef REBOT_VISION_FRAME_GUARD_H_
#define REBOT_VISION_FRAME_GUARD_H_

#include <exception>
#include <new>

namespace rv {

/**
 * Run `body()`; an exception it throws (std::bad_alloc from a copy of the instance id or from
 * publication preparation, or anything else) does not leave the frame callback. Returns true when
 * `body` completed. On an exception, `on_failure(what)` runs inside the handler (`what` is valid
 * only during that call); an exception from `on_failure` itself is swallowed.
 */
template <class Body, class OnFailure>
bool contain_frame(Body&& body, OnFailure&& on_failure) noexcept {
    const char* what = "unknown exception";
    try {
        body();
        return true;
    } catch (const std::bad_alloc&) {
        what = "allocation failed";
        try { on_failure(what); } catch (...) {}
    } catch (const std::exception& e) {
        try { on_failure(e.what()); } catch (...) {}
    } catch (...) {
        try { on_failure(what); } catch (...) {}
    }
    return false;
}

}  // namespace rv

#endif  // REBOT_VISION_FRAME_GUARD_H_
