// Modified from sscma-example-sg200x solutions/aruco-detector/main/inference_control.h
// (commit 9af8118, Apache-2.0): vision modes (off / yolo / aruco / both), per-request
// pipelines, subscriptions for continuous inference, and one result envelope with detections
// and markers.
#ifndef REBOT_VISION_CONTROL_H_
#define REBOT_VISION_CONTROL_H_

#include <condition_variable>
#include <cstdint>
#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "depth_map.h"
#include "gray_frame.h"
#include "intrinsics.h"
#include "mask_geom.h"
#include "model_catalogue.h"

namespace rv {

/** Perception provider API version, reported as "api_version" in every :8090 body. */
constexpr int kApiVersion = 2;

enum class Mode : uint8_t { Off, Yolo, Aruco, Both };

struct Pipelines {
    bool yolo = false;
    bool aruco = false;
    bool depth = false;
    bool gray = false;
    bool any() const noexcept { return yolo || aruco || depth || gray; }
    bool operator==(const Pipelines& o) const noexcept {
        return yolo == o.yolo && aruco == o.aruco && depth == o.depth && gray == o.gray;
    }
    Pipelines operator|(const Pipelines& o) const noexcept {
        return {yolo || o.yolo, aruco || o.aruco, depth || o.depth, gray || o.gray};
    }
    /** Every pipeline of `o` is in this set. */
    bool covers(const Pipelines& o) const noexcept {
        return (yolo || !o.yolo) && (aruco || !o.aruco) && (depth || !o.depth) && (gray || !o.gray);
    }
};

/**
 * A caller's continuous inference: these pipelines at `fps` (clamped to the configured rate)
 * until `expires_ms` (mono_ms()); refreshed by a keep-alive or by subscribing again.
 * Without pipelines it is a camera hold: no inference, the camera gate stays open (every frame
 * passes), so a trigger is served by the next frame instead of waiting for the idle 1 fps gate.
 */
struct Subscription {
    std::string id;
    std::string client;
    Pipelines pipelines;
    int fps = 0;  ///< requested rate; runs at min(fps, configured rate)
    int ttl_ms = 3000;
    uint64_t expires_ms = 0;
    bool hold() const noexcept { return !pipelines.any(); }
};

/** What runs continuously: the union of the active subscriptions (and the default one). */
struct Continuous {
    Pipelines pipelines;
    int fps = 0;  ///< max requested rate, clamped to the configured rate; 0 when nothing runs
    int subscriptions = 0;  ///< active subscriptions, the default one and camera holds not counted
    int holds = 0;          ///< active camera holds (subscriptions without pipelines)
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
    std::string instance_id;  ///< process/instance identity owning `frame_id`
    uint32_t frame_id = 0;    ///< 0: no result produced yet
    uint64_t ts_ms = 0;       ///< frame capture time, Unix epoch ms
    uint64_t frame_ms = 0;    ///< mono_ms() the frame is known to be newer than (frame_plan())
    float inference_ms = 0;
    Mode mode = Mode::Off;  ///< base mode when the frame ran
    bool ran_yolo = false;
    bool ran_aruco = false;
    bool ran_depth = false;
    bool ran_gray = false;
    float yolo_ms = 0;
    float aruco_ms = 0;
    float depth_ms = 0;
    float gray_ms = 0;
    float mask_ms = -1.0f;
    int width = 640;   ///< visible image size that detections / markers refer to
    int height = 360;
    std::string detections_json = "[]";  ///< normalised to width x height
    int count = 0;                       ///< accepted markers (after filtering)
    int rejected = 0;                    ///< markers dropped by the false-positive filter
    bool undistorted = false;
    std::string markers_json = "[]";  ///< pixels in width x height
    std::string aruco_stages_json;    ///< pipelines.aruco.stages object (ms per stage); empty: omitted
    std::string depth_json;           ///< top-level depth object; empty: omitted
    /** The depth grid (disparity) of this frame; only set while a depth map request waits. */
    std::shared_ptr<const std::vector<float>> depth_disp;
    int depth_grid_w = 0;
    int depth_grid_h = 0;
    // Frame thread timeline (mono_ms()), reported in a trigger's "timing" object.
    uint64_t wait_ms = 0;     ///< the frame thread started waiting for this frame
    uint64_t grab_ms = 0;     ///< retrieveFrame() returned it
    uint64_t done_ms = 0;     ///< publish() (pipelines done)
    uint32_t offered = 0;     ///< frame_plan() calls up to and including this frame
    uint64_t model_generation = 0;  ///< ModelSet generation the frame ran with
};

/** What is loaded: one snapshot, copied under VisionControl's lock. */
struct ModelSet {
    bool yolo_available = false;
    std::string model_name;  ///< catalogue name of the YOLO slot (empty: slot empty)
    std::string model_path;
    std::string model_kind;  ///< detect / segment / obb; empty with the slot empty (model.kind null)
    bool mask = false;
    bool depth_available = false;
    std::string depth_model_name;  ///< catalogue name of the depth slot (empty: slot empty)
    std::string depth_model_path;
    bool depth_metric = false;
    uint64_t generation = 0;  ///< +1 per executed swap
};

/** GET /api/calibration/intrinsics: the accepted intrinsics and where they are stored. */
struct IntrinsicsState {
    Intrinsics in;
    uint64_t revision = 1;           ///< 1 at process start, +1 per accepted PUT (not persisted)
    bool overridden_by_cli = false;  ///< --fx..--k3 or --marker-size given: PUT refused (CLI_OVERRIDE)
    std::string config_path;         ///< the file a PUT writes (the active --config)
};

/** State of the model swap (POST /api/models). */
enum class SwapState { Idle, Pending, Running, Failed };

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
     * at least every other frame (>= 7.5 fps), above that every frame. A camera hold opens it
     * like a pending trigger (gate_locked()).
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
        std::string model_kind = "detect";  ///< ignored without a YOLO model (yolo_available false)
        bool mask = false;
        int mask_max_boxes = 3;
        std::string family = "aruco4x4_50";
        bool depth_available = false;
        std::string depth_model_name;
        std::string depth_model_path;
        bool depth_metric = false;
        Catalogue catalogue;  ///< models POST /api/models can load, and the ION budget
        Intrinsics intrinsics;           ///< what the ArUco stage started with
        bool intrinsics_cli_override = false;
        std::string config_path;         ///< active vision.json (PUT /api/calibration/intrinsics writes it)
        bool gray_available = true;      ///< fixed-format 640x360 gray frames supported by this config
        int gray_width = kGrayWidth;
        int gray_height = kGrayHeight;
        int gray_crop[4] = {0, 140, kGrayWidth, kGrayHeight};
        int frame_size[2] = {640, 640};
        std::string instance_id;         ///< optional override for tests; empty = auto-generated
        uint32_t initial_frame_id = 0;   ///< optional starting frame counter for rollover tests
        int max_gray_sends = 2;          ///< maximum concurrent GET /api/frame/gray responses
    };

    enum class IntrinsicsStatus { Ok, CliOverride, RevisionMismatch, PersistFailed };
    static constexpr int kIntrinsicsApplyWaitMs = 2000;

    enum class Status { Ok, BadFps, YoloUnavailable, DepthUnavailable, Full, NotFound, SwapBusy, IonBudget, NotOwner, InstanceChanged };

    class TriggerScope {
    public:
        TriggerScope(VisionControl& c, uint64_t request_ms, Pipelines p, const MaskFocus* focus = nullptr)
            : c_(&c), handle_(c_->trigger_begin(request_ms, p, focus)) {}
        ~TriggerScope() { release(); }
        TriggerScope(const TriggerScope&) = delete;
        TriggerScope& operator=(const TriggerScope&) = delete;
        void release() {
            if (c_) {
                c_->trigger_end(handle_);
                c_ = nullptr;
            }
        }

    private:
        VisionControl* c_;
        uint64_t handle_ = 0;
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
    /** The loaded models (copy). */
    ModelSet models() const;
    bool yolo_available() const { return models().yolo_available; }
    int mask_max_boxes() const noexcept { return mask_max_boxes_; }
    const std::string& family() const noexcept { return family_; }
    bool depth_available() const { return models().depth_available; }
    bool gray_available() const noexcept { return gray_available_; }
    int gray_width() const noexcept { return gray_width_; }
    int gray_height() const noexcept { return gray_height_; }
    const int* gray_crop() const noexcept { return gray_crop_; }
    const int* frame_size() const noexcept { return frame_size_; }
    std::string instance_id() const;
    /** Allocate the next frame_id on the frame thread, rotating instance_id before u32 wrap. */
    uint32_t next_frame_id(std::string& instance_out);
    const Catalogue& catalogue() const noexcept { return catalogue_; }

    /**
     * POST /api/models: store `req` as the single pending swap when the resident set with it
     * applied fits the ION budget. Ok: `generation` = the generation the swap will publish.
     * SwapBusy: a swap is pending or running. IonBudget: `budget` has need / limit.
     */
    Status request_swap(const SwapRequest& req, uint64_t& generation, BudgetCheck& budget);
    /**
     * Frame thread: take the pending swap (state running), re-validated against the resident
     * set. False when none is pending, or (finished as failed, ION_BUDGET) it no longer fits.
     */
    bool take_swap(SwapRequest& out);
    /** Frame thread: publish the set after a swap (generation +1); `error` empty = idle, else failed. */
    uint64_t finish_swap(ModelSet now, const std::string& error);
    /**
     * End a Running swap that threw: clears the slots the caller unloaded, marks the swap Failed
     * ("SWAP_FAILED") and returns the new generation. Does not allocate.
     */
    uint64_t fail_swap(bool detector_cleared, bool depth_cleared) noexcept;
    /** Swap state, its error and the generation it publishes (pending / running) or published. */
    SwapState swap_state(std::string* error = nullptr, uint64_t* generation = nullptr) const;

    /** The accepted intrinsics (copy). */
    IntrinsicsState intrinsics() const;
    /**
     * PUT /api/calibration/intrinsics after validation. Serialised with other PUTs: refuses
     * (CliOverride) when command-line intrinsics are in force, (RevisionMismatch) when
     * `expected_revision` is not the current revision; else writes the config file
     * (persist_intrinsics; PersistFailed with `error`, nothing changes) and then accepts `in`
     * as the new state with revision + 1, pending for the frame thread. `revision` = the
     * current revision (mismatch) or the new one (Ok).
     */
    IntrinsicsStatus put_intrinsics(const Intrinsics& in, uint64_t expected_revision, uint64_t& revision,
                                    std::string& error);
    /** Frame thread: the accepted intrinsics when they are not applied yet (false otherwise). */
    bool take_intrinsics(Intrinsics& out, uint64_t& revision);
    /** Frame thread: the ArUco stage now runs with `revision`. */
    void finish_intrinsics(uint64_t revision);
    /** Wait until the frame thread applied `revision` or later; false on timeout or shutdown. */
    bool wait_intrinsics_applied(uint64_t revision, int wait_ms);

    /** Camera gate rate requested last (camera_gate_fps for the current state). */
    int camera_fps() const;

    /**
     * Frame thread, once per frame: the pipelines to run on it (empty: skip). frame_ms is a
     * mono_ms() the frame is known to be newer than. Drops subscriptions expired by frame_ms,
     * runs the continuous pipelines at their rate (RateGate) and adds the pipelines of every
     * trigger requested at or before frame_ms. triggered (optional): the pipelines of those
     * triggers alone. focus_out (optional): active mask focus for this frame.
     */
    Pipelines frame_plan(uint64_t frame_ms, Pipelines* triggered = nullptr, MaskFocus* focus_out = nullptr);
    /** frame_plan() calls so far (frames offered to the pipelines). */
    uint32_t offered() const;

    /** Frames offered to frame_plan() and results published, per second (last 2 s). */
    double input_fps(uint64_t now_ms) const;
    double result_fps(uint64_t now_ms) const;

    /** Persistent mask focus methods. */
    void set_mask_focus(const MaskFocus& f);
    void clear_mask_focus();
    bool get_mask_focus(uint64_t now_ms, MaskFocus& out) const;

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
    /**
     * Remove subscription `id` only when it belongs to `client` (NotOwner otherwise) and, when
     * `instance` is not empty, this process is that instance (InstanceChanged otherwise: after a
     * restart ids start again at s1 and may name another client's subscription). Empty `client`
     * skips the owner check. NotFound when unknown or expired.
     */
    Status unsubscribe_owned(const std::string& id, const std::string& client, const std::string& instance);
    /** Active subscriptions (expired ones are dropped first). */
    std::vector<Subscription> subscriptions(uint64_t now_ms);
    /** Drop subscriptions expired at now_ms. */
    void expire(uint64_t now_ms);

    /** A trigger without pipelines: the default subscription's, else the last active mode's. */
    Pipelines default_trigger_pipelines() const;
    uint64_t trigger_begin(uint64_t request_ms, Pipelines p, const MaskFocus* focus = nullptr);
    void trigger_end(uint64_t handle);
    TriggerScope trigger_scope(uint64_t request_ms, Pipelines p, const MaskFocus* focus = nullptr) {
        return TriggerScope(*this, request_ms, p, focus);
    }
    /** GET /api/depth: one map request at a time, at most one per second. */
    DepthMapGate& depth_map_gate() noexcept { return depth_map_gate_; }
    /** Map requests waiting for a frame: while > 0 the frame thread attaches the depth grid. */
    void depth_map_wait(bool on) noexcept { depth_map_wanted_.fetch_add(on ? 1 : -1); }
    bool depth_map_wanted() const noexcept { return depth_map_wanted_.load() > 0; }

    TriggerScope trigger_scope(uint64_t request_ms) {
        return TriggerScope(*this, request_ms, default_trigger_pipelines(), nullptr);
    }

    /**
     * Atomically publish a processed frame result and its optional gray frame record (frozen as
     * immutable once retained in `gray_ring_`, with `stored_mono_ms` stamped after insertion under
     * `mu_`). Any evicted gray frame is destroyed outside `mu_`. If `r.ran_gray` is true without a
     * valid matching `gray` record, `ran_gray` is cleared and nothing is installed in the gray ring.
     */
    void publish(const VisionResult& r, std::shared_ptr<GrayFrame> gray = nullptr);
    /** Atomically capture the current `(instance_id, frame_id)` baseline under `mu_`. */
    void current_baseline(std::string& instance_out, uint32_t& frame_id_out) const;
    /** Look up a retained gray frame by `(instance_id, frame_id)` (immediate, no wait). */
    GrayRing::Lookup get_gray(const std::string& instance_id, uint32_t frame_id,
                              std::shared_ptr<const GrayFrame>& out) const;
    /** Acquire a concurrent gray-frame send slot; returns nullptr when at capacity (503 GRAY_BUSY). */
    std::shared_ptr<void> try_acquire_gray_send();
    int active_gray_sends() const noexcept { return active_gray_sends_->load(std::memory_order_acquire); }
    int max_gray_sends() const noexcept { return max_gray_sends_; }
    /** As aruco-detector: returns true when no result exists or the wait was not satisfied. */
    bool latest(bool has_after, uint32_t after, int wait_ms, VisionResult& out);
    /**
     * Wait for a result newer than `(after_instance, after)` from a frame newer than request_ms
     * that ran at least `need` (a trigger's result). Passing the captured `(after_instance, after)`
     * pair prevents a rollover race between baseline capture and wait entry. When
     * `require_depth_map` is true, only results carrying a valid `depth_disp` grid satisfy the wait.
     * Returns true on timeout.
     */
    bool wait_fresh(const std::string& after_instance, uint32_t after, uint64_t request_ms, Pipelines need,
                    int wait_ms, VisionResult& out, bool require_depth_map = false);
    bool wait_fresh(uint32_t after, uint64_t request_ms, Pipelines need, int wait_ms, VisionResult& out,
                    bool require_depth_map = false);
    void shutdown();

private:
    struct Trigger {
        uint64_t handle = 0;
        uint64_t request_ms = 0;
        Pipelines pipelines;
        bool has_focus = false;
        MaskFocus focus;
    };
    static constexpr size_t kRecentResultsCap = 8;
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

    const int mask_max_boxes_;
    const std::string family_;
    const Catalogue catalogue_;
    OnChange on_change_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    Mode mode_ = Mode::Off;
    Mode last_active_ = Mode::Yolo;
    bool continuous_ = false;
    int fps_ = 5;
    MaskFocus persistent_focus_;
    std::vector<Trigger> triggers_;  ///< pending triggers
    uint64_t next_trigger_handle_ = 1;
    std::vector<Subscription> subs_;
    uint64_t next_sub_id_ = 1;
    bool reported_proc_ = false;
    int reported_gate_ = 0;
    bool gate_active_ = false;  ///< continuous work the rate gate was started for
    int gate_fps_ = 0;
    RateGate gate_;
    RateMeter input_meter_;
    RateMeter result_meter_;
    uint32_t offered_ = 0;
    const bool gray_available_;
    const int gray_width_;
    const int gray_height_;
    int gray_crop_[4] = {0, 140, kGrayWidth, kGrayHeight};
    int frame_size_[2] = {640, 640};
    std::string instance_id_;
    uint32_t frame_seq_ = 0;
    GrayRing gray_ring_{kGrayRingCapacity};
    VisionResult latest_;
    VisionResult latest_depth_map_;
    std::vector<VisionResult> recent_results_;
    size_t recent_head_ = 0;
    size_t recent_count_ = 0;
    bool stopping_ = false;
    ModelSet models_;
    SwapState swap_ = SwapState::Idle;
    SwapRequest swap_req_;
    std::string swap_error_;
    // intrinsics: the write lock serialises PUTs (file write outside mu_); the rest under mu_
    std::mutex intrinsics_write_mu_;
    const bool intrinsics_cli_;
    const std::string config_path_;
    Intrinsics intrinsics_;
    uint64_t intrinsics_rev_ = 1;
    uint64_t intrinsics_applied_ = 1;
    DepthMapGate depth_map_gate_{1000};
    std::atomic<int> depth_map_wanted_{0};
    const int max_gray_sends_;
    std::shared_ptr<std::atomic<int>> active_gray_sends_;
};

/** When the HTTP server accepted the connection and finished reading the request (mono_ms()). */
struct RequestTimes {
    uint64_t accept_ms = 0;
    uint64_t read_ms = 0;
};

struct HttpReply {
    HttpReply() = default;
    HttpReply(int s, std::string b) : status(s), body(std::move(b)) {}
    int status = 200;
    std::string body;
    std::string content_type = "application/json";
    std::vector<std::pair<std::string, std::string>> headers;  ///< extra response headers
    /** Optional immutable binary segment appended directly after `body` without copying. */
    std::shared_ptr<const std::vector<uint8_t>> binary_segment;
    /** Optional RAII guard (e.g., concurrent gray send slot) held until reply transmission finishes. */
    std::shared_ptr<void> send_guard;

    size_t content_length() const noexcept {
        return body.size() + (binary_segment ? binary_segment->size() : 0u);
    }
    std::string full_body() const {
        if (!binary_segment || binary_segment->empty()) return body;
        std::string out;
        out.reserve(body.size() + binary_segment->size());
        out.append(body);
        out.append(reinterpret_cast<const char*>(binary_segment->data()), binary_segment->size());
        return out;
    }
};

/** Who sent the request. Loopback = 127.0.0.0/8, ::1 or ::ffff:127.0.0.0/104. */
struct RequestPeer {
    bool loopback = false;
};

/**
 * Route one request:
 *   GET  /api/inference                 POST /api/inference {"enabled": bool}
 *   GET  /api/mode                      POST /api/mode {"mode"?, "enabled"?, "fps"?}
 *   GET  /api/results/latest[?after=<frame_id>&wait_ms=<0..5000>]
 *   POST /api/trigger[?wait_ms=<0..5000>] {"pipelines"?: ["yolo"|"aruco"|"depth"|"gray", ...], "wait_ms"?}
 *        ("gray" accepted from loopback peers only; 403 otherwise)
 *   GET  /api/frame/gray?instance_id=<id>&frame_id=<u32>  (loopback peer only; 403 otherwise)
 *        exact immediate lookup; 200 application/octet-stream (RGF1), 400 invalid/duplicate/overflow
 *        params, 404 FRAME_NOT_STORED, 410 FRAME_EXPIRED, 503 GRAY_BUSY + Retry-After
 *   POST /api/subscribe {"client", "pipelines", "fps"?, "ttl_ms"?, "id"?}  ("gray" refused: 400;
 *        "pipelines": [] is a camera hold: no inference, the camera gate open until it expires or
 *        is deleted; GET /api/mode "camera_hold": true)
 *   POST /api/subscribe/<id>/keepalive  DELETE /api/subscribe/<id>  GET /api/subscriptions
 *   POST /api/models {"detector"?: name|null, "depth"?: name|null}  (202 {"generation"})
 *   GET  /api/depth[?wait_ms=<0..5000>]  one depth inference on a fresh frame; 200
 *        application/octet-stream (depth_map_body), 409 depth unavailable / not metric /
 *        busy, 429 + Retry-After (more than 1 per s), 504 no frame in time
 *   GET  /api/calibration/intrinsics    PUT /api/calibration/intrinsics (loopback peer only)
 *        {fx, fy, cx, cy, k1, k2, p1, p2, k3, width, height, marker_size?, expected_revision}
 *        200 the GET body once applied, 202 {"revision"} when not applied within 2 s,
 *        400 {"error":"INVALID_FIELD","field"}, 403, 409 REVISION_MISMATCH / CLI_OVERRIDE,
 *        500 PERSIST_FAILED
 */
HttpReply handle_control_request(VisionControl& ctl, const std::string& method, const std::string& target,
                                 const std::string& body, uint64_t now_ms = 0, const RequestTimes* times = nullptr,
                                 RequestPeer peer = {});

}  // namespace rv

#endif  // REBOT_VISION_CONTROL_H_
