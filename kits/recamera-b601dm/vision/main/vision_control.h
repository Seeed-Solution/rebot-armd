// Modified from sscma-example-sg200x solutions/aruco-detector/main/inference_control.h
// (commit 9af8118, Apache-2.0): vision modes (off / yolo / aruco / both), per-request
// pipelines, subscriptions for continuous inference, and one result envelope with detections
// and markers.
#ifndef REBOT_VISION_CONTROL_H_
#define REBOT_VISION_CONTROL_H_

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace rv {

/** Perception provider API version, reported as "api_version" in every :8090 body. */
constexpr int kApiVersion = 2;

enum class Mode : uint8_t { Off, Yolo, Aruco, Both };

struct Pipelines {
    bool yolo = false;
    bool aruco = false;
    bool any() const noexcept { return yolo || aruco; }
    bool operator==(const Pipelines& o) const noexcept { return yolo == o.yolo && aruco == o.aruco; }
    Pipelines operator|(const Pipelines& o) const noexcept { return {yolo || o.yolo, aruco || o.aruco}; }
    /** Every pipeline of `o` is in this set. */
    bool covers(const Pipelines& o) const noexcept { return (yolo || !o.yolo) && (aruco || !o.aruco); }
};

/**
 * A caller's continuous inference: these pipelines at `fps` (clamped to the configured rate)
 * until `expires_ms` (mono_ms()); refreshed by a keep-alive or by subscribing again.
 */
struct Subscription {
    std::string id;
    std::string client;
    Pipelines pipelines;
    int fps = 0;  ///< requested rate; runs at min(fps, configured rate)
    int ttl_ms = 3000;
    uint64_t expires_ms = 0;
};

/** What runs continuously: the union of the active subscriptions (and the default one). */
struct Continuous {
    Pipelines pipelines;
    int fps = 0;  ///< max requested rate, clamped to the configured rate; 0 when nothing runs
    int subscriptions = 0;  ///< active subscriptions, the default one not counted
    bool active() const noexcept { return pipelines.any(); }
};

Pipelines pipelines_of(Mode m) noexcept;
const char* mode_name(Mode m) noexcept;
bool parse_mode(const std::string& s, Mode& out) noexcept;

/** Monotonic milliseconds (steady_clock), the clock of frame and trigger times. */
uint64_t mono_ms() noexcept;

/**
 * Picks `fps` frames per second out of a faster frame stream by due time. A frame is taken
 * when it is at most half a period early; the next due time then advances by one period,
 * so the long-run rate is exactly `fps` whatever the input rate and its jitter (as long as
 * the input is at least that fast), and a late frame does not cost a whole period. After
 * falling a full period behind it restarts from the current frame.
 */
class RateGate {
public:
    void reset() noexcept { started_ = false; }
    bool accept(uint64_t t_ms, int fps) noexcept;

private:
    bool started_ = false;
    double next_due_ = 0;
};

/** Events per second over the last `kWindowMs`. */
class RateMeter {
public:
    static constexpr uint64_t kWindowMs = 2000;
    void add(uint64_t t_ms) noexcept;
    double rate(uint64_t now_ms) const noexcept;

private:
    static constexpr size_t kCap = 64;
    uint64_t t_[kCap] = {};
    size_t n_ = 0;
    size_t head_ = 0;
};

/** One processed frame. JSON fragments are built by the frame thread (result_json.h). */
struct VisionResult {
    uint32_t frame_id = 0;  ///< 0: no result produced yet
    uint64_t ts_ms = 0;     ///< frame capture time, Unix epoch ms
    uint64_t frame_ms = 0;  ///< mono_ms() the frame is known to be newer than (frame_plan())
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
    std::string aruco_stages_json;    ///< pipelines.aruco.stages object (ms per stage); empty: omitted
};

class VisionControl {
public:
    static constexpr int kIdleFps = 1;
    static constexpr int kMaxWaitMs = 5000;
    static constexpr int kMinFps = 1;
    static constexpr int kMaxFps = 15;
    /** Camera gate rate that passes every frame of a channel at up to kMaxFps (see camera_gate_fps). */
    static constexpr int kOpenGateFps = 30;
    static constexpr int kMaxSubscriptions = 16;
    static constexpr int kDefaultTtlMs = 3000;
    static constexpr int kMinTtlMs = 500;
    static constexpr int kMaxTtlMs = 60000;

    /**
     * Rate for the camera callback's frame gate (sscma-micro ma_camera_sg200x.cpp
     * vpssCallback, set through Camera kFps). The gate passes a frame once 90 % of 1/rate has
     * passed since the last frame it passed, measured on arrival. On the 15 fps inference
     * channel (66.7 ms) a gate at the target rate halves it when arrivals jitter (5 fps needs
     * 180 ms, three frames are 200 ms) and cannot give 8..14 fps at all (one frame is too
     * early, two give 7.5 fps). So the gate only bounds the CPU copies, RateGate picks the
     * frames: idle 1 fps; a pending trigger opens it fully (next frame); up to 7 fps it passes
     * at least every other frame (>= 7.5 fps), above that every frame.
     */
    static int camera_gate_fps(bool processing, bool trigger_pending, int fps) noexcept;

    /** Called with the processing state and the camera gate rate whenever either changes. */
    using OnChange = std::function<void(bool processing, int camera_fps)>;

    struct Options {
        int fps = 5;
        Mode mode = Mode::Yolo;
        bool enabled = false;
        bool yolo_available = true;
        std::string model_name;
        std::string model_path;
        std::string family = "aruco4x4_50";
    };

    enum class Status { Ok, BadFps, YoloUnavailable, Full, NotFound };

    class TriggerScope {
    public:
        TriggerScope(VisionControl& c, uint64_t request_ms, Pipelines p) : c_(&c), req_(request_ms) {
            c_->trigger_begin(req_, p);
        }
        ~TriggerScope() { release(); }
        TriggerScope(const TriggerScope&) = delete;
        TriggerScope& operator=(const TriggerScope&) = delete;
        void release() {
            if (c_) {
                c_->trigger_end(req_);
                c_ = nullptr;
            }
        }

    private:
        VisionControl* c_;
        uint64_t req_;
    };

    VisionControl(const Options& opt, OnChange on_change);

    Mode mode() const;
    Mode last_active_mode() const;
    /** The default subscription (the mode API's `enabled`): never expires. */
    bool continuous_enabled() const;
    /** Union of the active subscriptions and the default one. */
    Continuous continuous() const;
    /** Pipelines run on frames (continuous, or a pending trigger). */
    bool processing() const;
    bool enabled() const { return processing(); }
    /** Configured rate: the default subscription's rate and the cap of every subscription. */
    int active_fps() const;
    int current_fps() const;
    bool yolo_available() const noexcept { return yolo_available_; }
    const std::string& model_name() const noexcept { return model_name_; }
    const std::string& model_path() const noexcept { return model_path_; }
    const std::string& family() const noexcept { return family_; }

    /** Camera gate rate requested last (camera_gate_fps for the current state). */
    int camera_fps() const;

    /**
     * Frame thread, once per frame: the pipelines to run on it (empty: skip). frame_ms is a
     * mono_ms() the frame is known to be newer than. Drops subscriptions expired by frame_ms,
     * runs the continuous pipelines at their rate (RateGate) and adds the pipelines of every
     * trigger requested at or before frame_ms. triggered (optional): the pipelines of those
     * triggers alone.
     */
    Pipelines frame_plan(uint64_t frame_ms, Pipelines* triggered = nullptr);

    /** Frames offered to frame_plan() and results published, per second (last 2 s). */
    double input_fps(uint64_t now_ms) const;
    double result_fps(uint64_t now_ms) const;

    /**
     * Change any of mode / continuous enable / rate in one step. Mode off forces enabled
     * false; enabling while off selects last_active_mode(). Nothing changes on an error.
     */
    Status apply(const Mode* mode, const bool* enabled, const int* fps);
    Status set_mode(Mode m) { return apply(&m, nullptr, nullptr); }
    bool set_enabled(bool on);

    /**
     * Add or refresh a subscription. The one with the same `id`, else the same `client`, is
     * updated in place (same id); otherwise a new one is created. fps 0 = the configured rate.
     */
    Status subscribe(const std::string& client, const std::string& id, Pipelines p, int fps, int ttl_ms,
                     uint64_t now_ms, Subscription& out);
    Status keepalive(const std::string& id, uint64_t now_ms, Subscription& out);
    bool unsubscribe(const std::string& id);
    /** Active subscriptions (expired ones are dropped first). */
    std::vector<Subscription> subscriptions(uint64_t now_ms);
    /** Drop subscriptions expired at now_ms. */
    void expire(uint64_t now_ms);

    /** A trigger without pipelines: the default subscription's, else the last active mode's. */
    Pipelines default_trigger_pipelines() const;
    void trigger_begin(uint64_t request_ms, Pipelines p);
    void trigger_end(uint64_t request_ms);
    TriggerScope trigger_scope(uint64_t request_ms, Pipelines p) { return TriggerScope(*this, request_ms, p); }
    TriggerScope trigger_scope(uint64_t request_ms) { return TriggerScope(*this, request_ms, default_trigger_pipelines()); }

    void publish(const VisionResult& r);
    /** As aruco-detector: returns true when no result exists or the wait was not satisfied. */
    bool latest(bool has_after, uint32_t after, int wait_ms, VisionResult& out);
    /**
     * Wait for a result newer than `after` from a frame newer than request_ms that ran at least
     * `need` (a trigger's result). Returns true on timeout.
     */
    bool wait_fresh(uint32_t after, uint64_t request_ms, Pipelines need, int wait_ms, VisionResult& out);
    void shutdown();

private:
    struct Trigger {
        uint64_t request_ms;
        Pipelines pipelines;
    };
    Continuous continuous_locked() const noexcept;
    bool processing_locked() const noexcept { return continuous_locked().active() || !triggers_.empty(); }
    int fps_locked() const noexcept;
    int gate_locked() const noexcept;
    bool expire_locked(uint64_t now_ms);
    /**
     * Report a processing / gate change to on_change_, and restart the rate gate when the
     * continuous work starts or changes rate (called with mu_ held).
     */
    void notify_locked();

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
    std::vector<Trigger> triggers_;  ///< pending triggers
    std::vector<Subscription> subs_;
    uint64_t next_sub_id_ = 1;
    bool reported_proc_ = false;
    int reported_gate_ = 0;
    bool gate_active_ = false;  ///< continuous work the rate gate was started for
    int gate_fps_ = 0;
    RateGate gate_;
    RateMeter input_meter_;
    RateMeter result_meter_;
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
 *   POST /api/trigger[?wait_ms=<0..5000>] {"pipelines"?: ["yolo"|"aruco", ...], "wait_ms"?}
 *   POST /api/subscribe {"client", "pipelines", "fps"?, "ttl_ms"?, "id"?}
 *   POST /api/subscribe/<id>/keepalive  DELETE /api/subscribe/<id>  GET /api/subscriptions
 */
HttpReply handle_control_request(VisionControl& ctl, const std::string& method, const std::string& target,
                                 const std::string& body, uint64_t now_ms = 0);

}  // namespace rv

#endif  // REBOT_VISION_CONTROL_H_
