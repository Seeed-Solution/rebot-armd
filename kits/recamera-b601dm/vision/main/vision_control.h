// Modified from sscma-example-sg200x solutions/aruco-detector/main/inference_control.h
// (commit 9af8118, Apache-2.0): vision modes (off / yolo / aruco / both), one shared
// inference rate, and one result envelope with detections and markers.
#ifndef REBOT_VISION_CONTROL_H_
#define REBOT_VISION_CONTROL_H_

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

namespace rv {

/** Perception provider API version, reported as "api_version" in every :8090 body. */
constexpr int kApiVersion = 1;

enum class Mode : uint8_t { Off, Yolo, Aruco, Both };

struct Pipelines {
    bool yolo = false;
    bool aruco = false;
    bool any() const noexcept { return yolo || aruco; }
    bool operator==(const Pipelines& o) const noexcept { return yolo == o.yolo && aruco == o.aruco; }
};

Pipelines pipelines_of(Mode m) noexcept;
const char* mode_name(Mode m) noexcept;
bool parse_mode(const std::string& s, Mode& out) noexcept;

/** One processed frame. JSON fragments are built by the frame thread (result_json.h). */
struct VisionResult {
    uint32_t frame_id = 0;  ///< 0: no result produced yet
    uint64_t ts_ms = 0;     ///< frame capture time, Unix epoch ms
    float inference_ms = 0;
    Mode mode = Mode::Off;  ///< base mode when the frame ran
    bool ran_yolo = false;
    bool ran_aruco = false;
    float yolo_ms = 0;
    float aruco_ms = 0;
    int width = 640;   ///< visible image size that detections / markers refer to
    int height = 360;
    std::string detections_json = "[]";  ///< normalised to width x height
    int count = 0;                       ///< accepted markers (after filtering)
    int rejected = 0;                    ///< markers dropped by the false-positive filter
    bool undistorted = false;
    std::string markers_json = "[]";  ///< pixels in width x height
};

class VisionControl {
public:
    static constexpr int kIdleFps = 1;
    static constexpr int kMaxWaitMs = 5000;
    static constexpr int kMinFps = 1;
    static constexpr int kMaxFps = 15;

    /** Called with the processing state and inference channel rate whenever either changes. */
    using OnChange = std::function<void(bool processing, int fps)>;

    struct Options {
        int fps = 5;
        Mode mode = Mode::Yolo;
        bool enabled = false;
        bool yolo_available = true;
        std::string model_name;
        std::string model_path;
        std::string family = "aruco4x4_50";
    };

    enum class Status { Ok, BadFps, YoloUnavailable };

    class TriggerScope {
    public:
        explicit TriggerScope(VisionControl& c) : c_(&c) { c_->trigger_begin(); }
        ~TriggerScope() { release(); }
        TriggerScope(const TriggerScope&) = delete;
        TriggerScope& operator=(const TriggerScope&) = delete;
        void release() {
            if (c_) {
                c_->trigger_end();
                c_ = nullptr;
            }
        }

    private:
        VisionControl* c_;
    };

    VisionControl(const Options& opt, OnChange on_change);

    Mode mode() const;
    Mode last_active_mode() const;
    bool continuous_enabled() const;
    /** Pipelines run on frames (continuous, or a pending trigger). */
    bool processing() const;
    bool enabled() const { return processing(); }
    int active_fps() const;
    int current_fps() const;
    bool yolo_available() const noexcept { return yolo_available_; }
    const std::string& model_name() const noexcept { return model_name_; }
    const std::string& model_path() const noexcept { return model_path_; }
    const std::string& family() const noexcept { return family_; }

    /** Frame thread: pipelines for the next frame (empty when standing by). */
    Pipelines frame_pipelines() const;

    /**
     * Change any of mode / continuous enable / rate in one step. Mode off forces enabled
     * false; enabling while off selects last_active_mode(). Nothing changes on an error.
     */
    Status apply(const Mode* mode, const bool* enabled, const int* fps);
    Status set_mode(Mode m) { return apply(&m, nullptr, nullptr); }
    bool set_enabled(bool on);

    void trigger_begin();
    void trigger_end();
    TriggerScope trigger_scope() { return TriggerScope(*this); }

    void publish(const VisionResult& r);
    /** As aruco-detector: returns true when no result exists or the wait was not satisfied. */
    bool latest(bool has_after, uint32_t after, int wait_ms, VisionResult& out);
    void shutdown();

private:
    bool processing_locked() const noexcept {
        return (continuous_ && mode_ != Mode::Off) || triggers_ > 0;
    }
    int fps_locked() const noexcept { return processing_locked() ? fps_ : kIdleFps; }

    const bool yolo_available_;
    const std::string model_name_;
    const std::string model_path_;
    const std::string family_;
    OnChange on_change_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    Mode mode_ = Mode::Off;
    Mode last_active_ = Mode::Yolo;
    bool continuous_ = false;
    int fps_ = 5;
    size_t triggers_ = 0;
    VisionResult latest_;
    bool stopping_ = false;
};

struct HttpReply {
    int status = 200;
    std::string body;
};

/**
 * Route one request:
 *   GET  /api/inference                 POST /api/inference {"enabled": bool}
 *   GET  /api/mode                      POST /api/mode {"mode"?, "enabled"?, "fps"?}
 *   GET  /api/results/latest[?after=<frame_id>&wait_ms=<0..5000>]
 *   POST /api/trigger[?wait_ms=<0..5000>]
 */
HttpReply handle_control_request(VisionControl& ctl, const std::string& method, const std::string& target,
                                 const std::string& body, uint64_t now_ms = 0);

}  // namespace rv

#endif  // REBOT_VISION_CONTROL_H_
