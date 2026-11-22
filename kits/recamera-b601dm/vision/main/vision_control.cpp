// Modified from sscma-example-sg200x solutions/aruco-detector/main/inference_control.cpp
// (commit 9af8118, Apache-2.0): modes, per-request pipelines, subscriptions, combined result
// envelope.
#include "vision_control.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "json.hpp"

namespace rv {

Pipelines pipelines_of(Mode m) noexcept {
    switch (m) {
        case Mode::Yolo: return {true, false};
        case Mode::Aruco: return {false, true};
        case Mode::Both: return {true, true};
        default: return {};
    }
}

const char* mode_name(Mode m) noexcept {
    switch (m) {
        case Mode::Yolo: return "yolo";
        case Mode::Aruco: return "aruco";
        case Mode::Both: return "both";
        default: return "off";
    }
}

bool parse_mode(const std::string& s, Mode& out) noexcept {
    if (s == "off") out = Mode::Off;
    else if (s == "yolo") out = Mode::Yolo;
    else if (s == "aruco") out = Mode::Aruco;
    else if (s == "both") out = Mode::Both;
    else return false;
    return true;
}

uint64_t mono_ms() noexcept {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

bool RateGate::accept(uint64_t t_ms, int fps) noexcept {
    const double period = 1000.0 / (fps > 0 ? fps : 1);
    const double t = static_cast<double>(t_ms);
    if (!started_) {
        started_ = true;
        next_due_ = t + period;
        return true;
    }
    if (t + period / 2 < next_due_) return false;
    next_due_ = (t - next_due_ >= period) ? t + period : next_due_ + period;
    return true;
}

void RateMeter::add(uint64_t t_ms) noexcept {
    t_[(head_ + n_) % kCap] = t_ms;
    if (n_ < kCap) ++n_;
    else head_ = (head_ + 1) % kCap;
}

double RateMeter::rate(uint64_t now_ms) const noexcept {
    size_t k = 0;
    for (size_t i = 0; i < n_; ++i) {
        const uint64_t t = t_[(head_ + i) % kCap];
        if (t <= now_ms && now_ms - t < kWindowMs) ++k;
    }
    return static_cast<double>(k) * 1000.0 / kWindowMs;
}

static uint64_t get_unix_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
}

static int clamp_fps(int f) {
    return f < VisionControl::kMinFps ? VisionControl::kMinFps : (f > VisionControl::kMaxFps ? VisionControl::kMaxFps : f);
}

static ModelSet model_set_of(const VisionControl::Options& o) {
    ModelSet m;
    m.yolo_available = o.yolo_available;
    m.model_name = o.model_name;
    m.model_path = o.model_path;
    m.model_kind = o.yolo_available ? o.model_kind : "";
    m.mask = o.mask;
    m.depth_available = o.depth_available;
    m.depth_model_name = o.depth_model_name;
    m.depth_model_path = o.depth_model_path;
    m.depth_metric = o.depth_metric;
    return m;
}

VisionControl::VisionControl(const Options& opt, OnChange on_change)
    : mask_max_boxes_(opt.mask_max_boxes),
      family_(opt.family),
      catalogue_(opt.catalogue),
      on_change_(std::move(on_change)),
      fps_(clamp_fps(opt.fps)),
      gray_available_(opt.gray_available && opt.gray_width == kGrayWidth && opt.gray_height == kGrayHeight),
      gray_width_(opt.gray_width),
      gray_height_(opt.gray_height),
      instance_id_(opt.instance_id.empty() ? generate_instance_id() : opt.instance_id),
      frame_seq_(opt.initial_frame_id),
      recent_results_(kRecentResultsCap),
      models_(model_set_of(opt)),
      intrinsics_cli_(opt.intrinsics_cli_override),
      config_path_(opt.config_path),
      intrinsics_(opt.intrinsics),
      max_gray_sends_(opt.max_gray_sends > 0 ? opt.max_gray_sends : 1),
      active_gray_sends_(std::make_shared<std::atomic<int>>(0)) {
    for (int i = 0; i < 4; ++i) gray_crop_[i] = opt.gray_crop[i];
    for (int i = 0; i < 2; ++i) frame_size_[i] = opt.frame_size[i];
    Mode m = opt.mode;
    // Without a model, YOLO modes fall back to what still works.
    if (!opt.yolo_available) m = (m == Mode::Both || m == Mode::Aruco) ? Mode::Aruco : Mode::Off;
    mode_ = m;
    last_active_ = m != Mode::Off ? m : (opt.yolo_available ? Mode::Yolo : Mode::Aruco);
    continuous_ = opt.enabled && m != Mode::Off;
    latest_.mode = mode_;
    latest_.instance_id = instance_id_;
    reported_proc_ = processing_locked();
    reported_gate_ = gate_locked();
    gate_active_ = continuous_locked().active();
    gate_fps_ = continuous_locked().fps;
    if (on_change_) on_change_(reported_proc_, reported_gate_);
}

int VisionControl::camera_gate_fps(bool processing, bool trigger_pending, int fps) noexcept {
    if (!processing) return kIdleFps;
    if (trigger_pending) return kOpenGateFps;
    return fps <= 7 ? 2 * fps : kOpenGateFps;
}

Continuous VisionControl::continuous_locked() const noexcept {
    Continuous c;
    if (continuous_ && mode_ != Mode::Off) {
        c.pipelines = pipelines_of(mode_);
        c.fps = fps_;
    }
    for (const Subscription& s : subs_) {
        c.pipelines = c.pipelines | s.pipelines;
        c.fps = std::max(c.fps, s.fps);
        ++c.subscriptions;
    }
    c.fps = c.active() ? std::min(c.fps, fps_) : 0;
    return c;
}

int VisionControl::fps_locked() const noexcept {
    const Continuous c = continuous_locked();
    if (c.active()) return c.fps;
    return triggers_.empty() ? kIdleFps : fps_;
}

int VisionControl::gate_locked() const noexcept {
    const Continuous c = continuous_locked();
    // a pending swap opens the gate like a trigger: the frame thread applies it on the next frame
    const bool wake = !triggers_.empty() || swap_ == SwapState::Pending;
    return camera_gate_fps(c.active() || wake, wake, c.active() ? c.fps : fps_);
}

bool VisionControl::expire_locked(uint64_t now_ms) {
    const size_t n = subs_.size();
    subs_.erase(std::remove_if(subs_.begin(), subs_.end(), [&](const Subscription& s) { return s.expires_ms <= now_ms; }),
                subs_.end());
    return subs_.size() != n;
}

void VisionControl::notify_locked() {
    // Continuous work starts, stops or changes rate: the next frame is processed.
    const Continuous c = continuous_locked();
    if (c.active() != gate_active_ || c.fps != gate_fps_) {
        gate_.reset();
        gate_active_ = c.active();
        gate_fps_ = c.fps;
    }
    const bool proc = processing_locked();
    const int gate = gate_locked();
    if (proc == reported_proc_ && gate == reported_gate_) return;
    reported_proc_ = proc;
    reported_gate_ = gate;
    if (on_change_) on_change_(proc, gate);
}

int VisionControl::camera_fps() const {
    std::lock_guard<std::mutex> lk(mu_);
    return reported_gate_;
}

Mode VisionControl::mode() const {
    std::lock_guard<std::mutex> lk(mu_);
    return mode_;
}

Mode VisionControl::last_active_mode() const {
    std::lock_guard<std::mutex> lk(mu_);
    return last_active_;
}

bool VisionControl::continuous_enabled() const {
    std::lock_guard<std::mutex> lk(mu_);
    return continuous_;
}

Continuous VisionControl::continuous() const {
    std::lock_guard<std::mutex> lk(mu_);
    return continuous_locked();
}

bool VisionControl::processing() const {
    std::lock_guard<std::mutex> lk(mu_);
    return processing_locked();
}

int VisionControl::active_fps() const {
    std::lock_guard<std::mutex> lk(mu_);
    return fps_;
}

int VisionControl::current_fps() const {
    std::lock_guard<std::mutex> lk(mu_);
    return fps_locked();
}

Pipelines VisionControl::frame_plan(uint64_t frame_ms, Pipelines* triggered, MaskFocus* focus_out) {
    bool expired = false;
    Pipelines p;
    MaskFocus active_focus;
    bool has_active_focus = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        input_meter_.add(frame_ms);
        ++offered_;
        if (expire_locked(frame_ms)) {
            expired = true;
            notify_locked();
        }
        const Continuous c = continuous_locked();
        if (c.active() && gate_.accept(frame_ms, c.fps)) p = c.pipelines;
        Pipelines tp;
        for (Trigger& t : triggers_) {
            if (t.request_ms <= frame_ms) {
                tp = tp | t.pipelines;
                if (!has_active_focus && t.has_focus) {
                    active_focus = t.focus;
                    has_active_focus = true;
                    t.has_focus = false;
                }
            }
        }
        p = p | tp;
        if (triggered) *triggered = tp;

        if (!has_active_focus) {
            if (persistent_focus_.active) {
                if (frame_ms < persistent_focus_.expires_ms) {
                    active_focus = persistent_focus_;
                    has_active_focus = true;
                } else {
                    persistent_focus_.active = false;
                }
            }
        }
        if (focus_out) {
            if (has_active_focus) {
                *focus_out = active_focus;
            } else {
                *focus_out = MaskFocus{};
            }
        }
    }
    if (expired) cv_.notify_all();
    return p;
}

uint32_t VisionControl::offered() const {
    std::lock_guard<std::mutex> lk(mu_);
    return offered_;
}

double VisionControl::input_fps(uint64_t now_ms) const {
    std::lock_guard<std::mutex> lk(mu_);
    return input_meter_.rate(now_ms);
}

double VisionControl::result_fps(uint64_t now_ms) const {
    std::lock_guard<std::mutex> lk(mu_);
    return result_meter_.rate(now_ms);
}

VisionControl::Status VisionControl::apply(const Mode* mode, const bool* enabled, const int* fps) {
    if (fps && (*fps < kMinFps || *fps > kMaxFps)) return Status::BadFps;
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (mode && pipelines_of(*mode).yolo && !models_.yolo_available) return Status::YoloUnavailable;
        if (fps) fps_ = *fps;
        if (mode) {
            mode_ = *mode;
            if (*mode == Mode::Off) continuous_ = false;
            else last_active_ = *mode;
        }
        if (enabled && !(mode && *mode == Mode::Off)) {  // "off" wins over enabled:true
            if (*enabled && mode_ == Mode::Off) mode_ = last_active_;
            continuous_ = *enabled;
        }
        notify_locked();
    }
    cv_.notify_all();
    return Status::Ok;
}

bool VisionControl::set_enabled(bool on) {
    const bool was = continuous_enabled();
    apply(nullptr, &on, nullptr);
    return was != on;
}

VisionControl::Status VisionControl::subscribe(const std::string& client, const std::string& id, Pipelines p,
                                               int fps, int ttl_ms, uint64_t now_ms, Subscription& out) {
    if (fps != 0 && (fps < kMinFps || fps > kMaxFps)) return Status::BadFps;
    ttl_ms = std::max(kMinTtlMs, std::min(kMaxTtlMs, ttl_ms));
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (p.yolo && !models_.yolo_available) return Status::YoloUnavailable;
        if (p.depth && !models_.depth_available) return Status::DepthUnavailable;
        expire_locked(now_ms);
        auto it = subs_.end();
        if (!id.empty()) {
            it = std::find_if(subs_.begin(), subs_.end(), [&](const Subscription& s) { return s.id == id; });
        }
        if (it == subs_.end()) {
            it = std::find_if(subs_.begin(), subs_.end(), [&](const Subscription& s) { return s.client == client; });
        }
        if (it == subs_.end()) {
            if (static_cast<int>(subs_.size()) >= kMaxSubscriptions) {
                notify_locked();
                return Status::Full;
            }
            Subscription s;
            s.id = "s" + std::to_string(next_sub_id_++);
            subs_.push_back(s);
            it = subs_.end() - 1;
        }
        it->client = client;
        it->pipelines = p;
        it->fps = fps == 0 ? fps_ : fps;
        it->ttl_ms = ttl_ms;
        it->expires_ms = now_ms + static_cast<uint64_t>(ttl_ms);
        out = *it;
        notify_locked();
    }
    cv_.notify_all();
    return Status::Ok;
}

VisionControl::Status VisionControl::keepalive(const std::string& id, uint64_t now_ms, Subscription& out) {
    std::lock_guard<std::mutex> lk(mu_);
    if (expire_locked(now_ms)) notify_locked();
    for (Subscription& s : subs_) {
        if (s.id == id) {
            s.expires_ms = now_ms + static_cast<uint64_t>(s.ttl_ms);
            out = s;
            return Status::Ok;
        }
    }
    return Status::NotFound;
}

bool VisionControl::unsubscribe(const std::string& id) {
    bool found = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        const size_t n = subs_.size();
        subs_.erase(std::remove_if(subs_.begin(), subs_.end(), [&](const Subscription& s) { return s.id == id; }),
                    subs_.end());
        found = subs_.size() != n;
        notify_locked();
    }
    cv_.notify_all();
    return found;
}

std::vector<Subscription> VisionControl::subscriptions(uint64_t now_ms) {
    expire(now_ms);
    std::lock_guard<std::mutex> lk(mu_);
    return subs_;
}

void VisionControl::expire(uint64_t now_ms) {
    bool changed = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        changed = expire_locked(now_ms);
        if (changed) notify_locked();
    }
    if (changed) cv_.notify_all();
}

void VisionControl::set_mask_focus(const MaskFocus& f) {
    std::lock_guard<std::mutex> lk(mu_);
    persistent_focus_ = f;
}

void VisionControl::clear_mask_focus() {
    std::lock_guard<std::mutex> lk(mu_);
    persistent_focus_.active = false;
}

bool VisionControl::get_mask_focus(uint64_t now_ms, MaskFocus& out) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (persistent_focus_.active) {
        if (now_ms < persistent_focus_.expires_ms) {
            out = persistent_focus_;
            return true;
        }
    }
    return false;
}

Pipelines VisionControl::default_trigger_pipelines() const {
    std::lock_guard<std::mutex> lk(mu_);
    return pipelines_of(mode_ != Mode::Off ? mode_ : last_active_);
}

std::string VisionControl::instance_id() const {
    std::lock_guard<std::mutex> lk(mu_);
    return instance_id_;
}

uint32_t VisionControl::next_frame_id(std::string& instance_out) {
    std::vector<std::shared_ptr<const GrayFrame>> displaced;
    VisionResult displaced_depth;
    uint32_t id = 0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        const bool rollover = frame_seq_ == UINT32_MAX;
        // 1. Everything that can allocate, before any member changes: a failure leaves identity,
        //    sequence and retained records as they were, and the next call rolls over again.
        std::string next_inst;
        std::string latest_inst;
        if (rollover) {
            next_inst = generate_instance_id();
            if (next_inst == instance_id_) next_inst = generate_instance_id();
            latest_inst = next_inst;
            displaced.reserve(gray_ring_.capacity());  // GrayRing::clear() then does not allocate
        }
        std::string out = rollover ? next_inst : instance_id_;
        // 2. Commit with non-throwing operations.
        if (rollover) {
            gray_ring_.clear(displaced);
            // Keep the pre-sized history slots (publish() indexes them); empty them and restart
            // the ring so no index refers to a previous-generation entry.
            for (VisionResult& slot : recent_results_) slot = VisionResult{};
            recent_head_ = 0;
            recent_count_ = 0;
            // The retained depth map belongs to the previous generation.
            displaced_depth = std::move(latest_depth_map_);
            latest_depth_map_ = VisionResult{};
            latest_.frame_id = 0;
            latest_.instance_id = std::move(latest_inst);
            instance_id_ = std::move(next_inst);
            frame_seq_ = 0;
        }
        id = ++frame_seq_;
        instance_out = std::move(out);
    }
    displaced.clear();
    displaced_depth.depth_disp.reset();
    return id;
}

uint64_t VisionControl::trigger_begin(uint64_t request_ms, Pipelines p, const MaskFocus* focus) {
    uint64_t handle = 0;
    {
        std::lock_guard<std::mutex> lk(mu_);
        Trigger t;
        handle = next_trigger_handle_++;
        t.handle = handle;
        t.request_ms = request_ms;
        t.pipelines = p;
        if (focus && focus->active) {
            t.has_focus = true;
            t.focus = *focus;
        } else {
            t.has_focus = false;
        }
        triggers_.push_back(t);
        notify_locked();
    }
    cv_.notify_all();
    return handle;
}

void VisionControl::trigger_end(uint64_t handle) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        for (auto it = triggers_.begin(); it != triggers_.end(); ++it) {
            if (it->handle == handle) {
                triggers_.erase(it);
                break;
            }
        }
        notify_locked();
    }
    cv_.notify_all();
}

static bool has_valid_depth_map(const VisionResult& r) noexcept {
    return r.depth_disp && r.depth_grid_w > 0 && r.depth_grid_h > 0 &&
           r.depth_disp->size() == static_cast<size_t>(r.depth_grid_w) * static_cast<size_t>(r.depth_grid_h);
}

void VisionControl::publish(const VisionResult& r, std::shared_ptr<GrayFrame> gray) {
    std::shared_ptr<const GrayFrame> displaced_gray;
    VisionResult displaced_latest;
    VisionResult displaced_depth;
    VisionResult displaced_hist;
    {
        std::lock_guard<std::mutex> lk(mu_);
        // 1. Prepare all potentially allocating state BEFORE mutating any shared member so an
        //    allocation failure cannot leave gray_ring_, latest_, or recent_results_ out of sync.
        VisionResult stored = r;
        if (stored.instance_id.empty()) stored.instance_id = instance_id_;
        bool gray_ok = false;
        if (stored.ran_gray) {
            gray_ok = gray && gray->valid() && gray->frame_id == stored.frame_id &&
                      gray->instance_id == stored.instance_id;
            if (!gray_ok) {
                stored.ran_gray = false;
                stored.gray_ms = 0;
                gray.reset();
            }
        } else {
            gray.reset();
        }
        stored.done_ms = mono_ms();

        VisionResult hist = stored;
        hist.depth_disp.reset();
        const bool save_depth = has_valid_depth_map(stored);
        VisionResult depth_copy;
        if (save_depth) depth_copy = stored;

        // 2. Commit atomically using strictly non-throwing operations into pre-allocated storage.
        if (gray_ok) {
            GrayFrame* raw_gray = gray.get();
            displaced_gray = gray_ring_.put(std::move(gray));
            // Stamp stored_mono_ms after ring insertion while mu_ is still held (no reader can
            // observe the record until mu_ unlocks, after which it is only exposed as const).
            raw_gray->stored_mono_ms = mono_ms();
        }
        displaced_latest = std::move(latest_);
        latest_ = std::move(stored);
        if (save_depth) {
            displaced_depth = std::move(latest_depth_map_);
            latest_depth_map_ = std::move(depth_copy);
        }
        size_t hist_idx = 0;
        if (recent_count_ >= kRecentResultsCap) {
            hist_idx = recent_head_;
            recent_head_ = (recent_head_ + 1) % kRecentResultsCap;
        } else {
            hist_idx = (recent_head_ + recent_count_) % kRecentResultsCap;
            ++recent_count_;
        }
        displaced_hist = std::move(recent_results_[hist_idx]);
        recent_results_[hist_idx] = std::move(hist);
        result_meter_.add(latest_.frame_ms);
    }
    cv_.notify_all();
    // Displaced (or rejected) buffers are destroyed here, outside mu_.
    displaced_gray.reset();
    displaced_depth.depth_disp.reset();
    displaced_latest.depth_disp.reset();
    gray.reset();
}

void VisionControl::current_baseline(std::string& instance_out, uint32_t& frame_id_out) const {
    std::lock_guard<std::mutex> lk(mu_);
    instance_out = (latest_.frame_id != 0 && !latest_.instance_id.empty()) ? latest_.instance_id : instance_id_;
    frame_id_out = latest_.frame_id;
}

GrayRing::Lookup VisionControl::get_gray(const std::string& instance_id, uint32_t frame_id,
                                         std::shared_ptr<const GrayFrame>& out) const {
    std::lock_guard<std::mutex> lk(mu_);
    return gray_ring_.find(instance_id, frame_id, instance_id_, out);
}

std::shared_ptr<void> VisionControl::try_acquire_gray_send() {
    std::atomic<int>* counter = active_gray_sends_.get();
    int cur = counter->load(std::memory_order_acquire);
    while (cur < max_gray_sends_) {
        if (counter->compare_exchange_weak(cur, cur + 1, std::memory_order_acq_rel, std::memory_order_acquire)) {
            std::shared_ptr<std::atomic<int>> keep = active_gray_sends_;
            return std::shared_ptr<void>(counter, [keep](void*) {
                keep->fetch_sub(1, std::memory_order_acq_rel);
            });
        }
    }
    return nullptr;
}

bool VisionControl::wait_fresh(const std::string& after_instance, uint32_t after, uint64_t request_ms, Pipelines need,
                               int wait_ms, VisionResult& out, bool require_depth_map) {
    if (wait_ms < 0) wait_ms = 0;
    if (wait_ms > kMaxWaitMs) wait_ms = kMaxWaitMs;
    std::unique_lock<std::mutex> lk(mu_);
    const std::string start_instance = after_instance.empty() ? instance_id_ : after_instance;
    // A frame planned before this trigger was registered can be newer than request_ms but
    // lack its pipelines; the next one has them. Scanning recent_results_ newest-to-oldest
    // also preserves the qualifying result if a later non-matching frame updates latest_.
    // Map waiters require the actual depth_disp payload, preserved in latest_depth_map_ across
    // intervening non-depth publications.
    auto matches = [&](const VisionResult& r) {
        if (r.frame_id == 0) return false;
        // Only the baseline's generation and the current one exist for this waiter; anything
        // else (a retained entry of an older generation) is not newer than the baseline.
        if (r.instance_id != start_instance && r.instance_id != instance_id_) return false;
        const bool newer = (r.instance_id == start_instance) ? (r.frame_id > after) : true;
        if (!newer || r.frame_ms < request_ms) return false;
        if (!Pipelines{r.ran_yolo, r.ran_aruco, r.ran_depth, r.ran_gray}.covers(need)) return false;
        if (require_depth_map && !has_valid_depth_map(r)) return false;
        return true;
    };
    auto find_match = [&]() -> const VisionResult* {
        if (require_depth_map) {
            if (matches(latest_)) return &latest_;
            if (matches(latest_depth_map_)) return &latest_depth_map_;
            return nullptr;
        }
        for (size_t i = recent_count_; i-- > 0;) {
            const VisionResult& entry = recent_results_[(recent_head_ + i) % kRecentResultsCap];
            if (matches(entry)) {
                if (entry.frame_id == latest_.frame_id && entry.instance_id == latest_.instance_id) return &latest_;
                return &entry;
            }
        }
        return matches(latest_) ? &latest_ : nullptr;
    };
    cv_.wait_for(lk, std::chrono::milliseconds(wait_ms), [&] { return find_match() != nullptr || stopping_; });
    if (const VisionResult* hit = find_match()) {
        out = *hit;
        return false;
    }
    out = latest_;
    return true;
}

bool VisionControl::wait_fresh(uint32_t after, uint64_t request_ms, Pipelines need, int wait_ms, VisionResult& out,
                               bool require_depth_map) {
    return wait_fresh(std::string{}, after, request_ms, need, wait_ms, out, require_depth_map);
}

bool VisionControl::latest(bool has_after, uint32_t after, int wait_ms, VisionResult& out) {
    if (wait_ms < 0) wait_ms = 0;
    if (wait_ms > kMaxWaitMs) wait_ms = kMaxWaitMs;
    std::unique_lock<std::mutex> lk(mu_);
    auto satisfied = [&] { return !has_after || latest_.frame_id > after; };
    if (has_after && wait_ms > 0) {
        cv_.wait_for(lk, std::chrono::milliseconds(wait_ms),
                     [&] { return satisfied() || stopping_ || !processing_locked(); });
    }
    out = latest_;
    return latest_.frame_id == 0 || !satisfied();
}

ModelSet VisionControl::models() const {
    std::lock_guard<std::mutex> lk(mu_);
    return models_;
}

VisionControl::Status VisionControl::request_swap(const SwapRequest& req, uint64_t& generation, BudgetCheck& budget) {
    std::lock_guard<std::mutex> lk(mu_);
    if (swap_ == SwapState::Pending || swap_ == SwapState::Running) return Status::SwapBusy;
    budget = check_budget(catalogue_, after_swap(req.detector, models_.model_name),
                          after_swap(req.depth, models_.depth_model_name));
    if (!budget.ok) return Status::IonBudget;
    SwapRequest stored = req;  // may throw: copied before the swap is marked Pending
    swap_req_ = std::move(stored);
    swap_error_.clear();
    swap_ = SwapState::Pending;
    generation = models_.generation + 1;
    notify_locked();
    return Status::Ok;
}

bool VisionControl::take_swap(SwapRequest& out) {
    std::lock_guard<std::mutex> lk(mu_);
    if (swap_ != SwapState::Pending) return false;
    // the resident set can differ from the request's time only by a slot a failed load left empty
    if (!check_budget(catalogue_, after_swap(swap_req_.detector, models_.model_name),
                      after_swap(swap_req_.depth, models_.depth_model_name)).ok) {
        swap_ = SwapState::Failed;
        swap_error_ = "ION_BUDGET";
        ++models_.generation;
        notify_locked();
        return false;
    }
    SwapRequest taken = swap_req_;  // may throw: a failed copy leaves the swap Pending
    out = std::move(taken);
    swap_ = SwapState::Running;
    notify_locked();
    return true;
}

uint64_t VisionControl::finish_swap(ModelSet now, const std::string& error) {
    std::lock_guard<std::mutex> lk(mu_);
    now.generation = models_.generation + 1;
    models_ = std::move(now);
    swap_ = error.empty() ? SwapState::Idle : SwapState::Failed;
    swap_error_ = error;
    return models_.generation;
}

SwapState VisionControl::swap_state(std::string* error, uint64_t* generation) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (error) *error = swap_error_;
    if (generation) *generation = models_.generation + (swap_ == SwapState::Pending || swap_ == SwapState::Running ? 1 : 0);
    return swap_;
}

IntrinsicsState VisionControl::intrinsics() const {
    std::lock_guard<std::mutex> lk(mu_);
    IntrinsicsState s;
    s.in = intrinsics_;
    s.revision = intrinsics_rev_;
    s.overridden_by_cli = intrinsics_cli_;
    s.config_path = config_path_;
    return s;
}

VisionControl::IntrinsicsStatus VisionControl::put_intrinsics(const Intrinsics& in, uint64_t expected_revision,
                                                              uint64_t& revision, std::string& error) {
    std::lock_guard<std::mutex> write_lock(intrinsics_write_mu_);  // only PUTs change the revision
    {
        std::lock_guard<std::mutex> lk(mu_);
        revision = intrinsics_rev_;
    }
    if (intrinsics_cli_) return IntrinsicsStatus::CliOverride;
    if (expected_revision != revision) return IntrinsicsStatus::RevisionMismatch;
    if (!persist_intrinsics(config_path_, in, error)) return IntrinsicsStatus::PersistFailed;
    {
        std::lock_guard<std::mutex> lk(mu_);
        intrinsics_ = in;
        revision = ++intrinsics_rev_;
    }
    return IntrinsicsStatus::Ok;
}

bool VisionControl::take_intrinsics(Intrinsics& out, uint64_t& revision) {
    std::lock_guard<std::mutex> lk(mu_);
    if (intrinsics_applied_ >= intrinsics_rev_) return false;
    out = intrinsics_;
    revision = intrinsics_rev_;
    return true;
}

void VisionControl::finish_intrinsics(uint64_t revision) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (revision > intrinsics_applied_) intrinsics_applied_ = revision;
    }
    cv_.notify_all();
}

bool VisionControl::wait_intrinsics_applied(uint64_t revision, int wait_ms) {
    std::unique_lock<std::mutex> lk(mu_);
    cv_.wait_for(lk, std::chrono::milliseconds(wait_ms),
                 [&] { return stopping_ || intrinsics_applied_ >= revision; });
    return intrinsics_applied_ >= revision;
}

void VisionControl::shutdown() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stopping_ = true;
        if (swap_ == SwapState::Pending) swap_ = SwapState::Idle;  // a pending swap is dropped
    }
    cv_.notify_all();
}

// ---------------------------------------------------------------------------
// HTTP routes

static std::string error_body(const std::string& msg) {
    return nlohmann::json{{"api_version", kApiVersion}, {"error", msg}}.dump();
}

static void put_rates(std::ostringstream& s, const VisionControl& ctl) {
    s << ",\"fps\":" << ctl.current_fps() << ",\"active_fps\":" << ctl.active_fps()
      << ",\"idle_fps\":" << VisionControl::kIdleFps;
}

static std::string inference_body(const VisionControl& ctl) {
    std::ostringstream s;
    s << "{\"api_version\":" << kApiVersion << ",\"enabled\":" << (ctl.continuous_enabled() ? "true" : "false");
    put_rates(s, ctl);
    s << ",\"mode\":\"" << mode_name(ctl.mode()) << "\"}";
    return s.str();
}

static nlohmann::json pipeline_list(Pipelines p) {
    nlohmann::json a = nlohmann::json::array();
    if (p.yolo) a.push_back("yolo");
    if (p.aruco) a.push_back("aruco");
    if (p.depth) a.push_back("depth");
    if (p.gray) a.push_back("gray");
    return a;
}

static nlohmann::json continuous_json(const Continuous& c) {
    return {{"active", c.active()}, {"pipelines", pipeline_list(c.pipelines)}, {"fps", c.fps},
            {"subscriptions", c.subscriptions}};
}

static nlohmann::json subscription_json(const Subscription& s, uint64_t now_ms, int cap) {
    const long long left = static_cast<long long>(s.expires_ms) - static_cast<long long>(now_ms);
    return {{"id", s.id},       {"client", s.client},
            {"pipelines", pipeline_list(s.pipelines)},
            {"fps", std::min(s.fps, cap)},
            {"requested_fps", s.fps},
            {"ttl_ms", s.ttl_ms}, {"expires_in_ms", left < 0 ? 0 : left}};
}

static nlohmann::json mask_focus_json(const MaskFocus& f, uint64_t now_ms) {
    if (!f.active || f.expires_ms <= now_ms) {
        return nullptr;
    }
    const long long left = static_cast<long long>(f.expires_ms) - static_cast<long long>(now_ms);
    nlohmann::json j = {
        {"box_norm", {f.box_norm[0], f.box_norm[1], f.box_norm[2], f.box_norm[3]}},
        {"ttl_ms", f.ttl_ms},
        {"expires_in_ms", left < 0 ? 0 : left},
        {"remaining_ttl_ms", left < 0 ? 0 : left}
    };
    if (!f.label.empty()) {
        j["label"] = f.label;
    }
    return j;
}

static const char* swap_state_name(SwapState s) {
    switch (s) {
        case SwapState::Pending: return "pending";
        case SwapState::Running: return "running";
        case SwapState::Failed: return "failed";
        default: return "idle";
    }
}

static nlohmann::json name_or_null(const std::string& s) {
    return s.empty() ? nlohmann::json(nullptr) : nlohmann::json(s);
}

/** `models` (generation, swap, resident, catalogue) and `ion` of the mode body. */
static void put_models(nlohmann::json& j, const VisionControl& ctl, const ModelSet& ms) {
    std::string error;
    uint64_t swap_generation = 0;
    const SwapState st = ctl.swap_state(&error, &swap_generation);
    nlohmann::json swap = {{"state", swap_state_name(st)}, {"generation", swap_generation}};
    if (!error.empty()) swap["error"] = error;
    const Catalogue& cat = ctl.catalogue();
    nlohmann::json list = nlohmann::json::array();
    for (const ModelEntry& m : cat.models) {
        nlohmann::json e = {{"name", m.name}, {"kind", m.kind}, {"path", m.path}, {"ion_mb", m.ion_mb},
                            {"resident", m.name == ms.model_name || m.name == ms.depth_model_name}};
        if (m.kind == "depth") e["metric"] = depth_metric_of(m);
        list.push_back(e);
    }
    j["models"] = {{"generation", ms.generation}, {"swap", swap},
                   {"resident", {{"detector", name_or_null(ms.model_name)}, {"depth", name_or_null(ms.depth_model_name)}}},
                   {"catalogue", list}};
    const BudgetCheck used = check_budget(cat, ms.model_name, ms.depth_model_name);
    j["ion"] = {{"base_mb", cat.ion.base_mb}, {"used_mb", used.need_mb - cat.ion.reserve_mb},
                {"total_mb", cat.ion.total_mb}, {"reserve_mb", cat.ion.reserve_mb}};
}

/** GET /api/calibration/intrinsics body without api_version (also /api/mode "intrinsics"). */
static nlohmann::json intrinsics_json(const IntrinsicsState& s) {
    const aruco::CameraModel& c = s.in.cam;
    return {{"fx", c.fx}, {"fy", c.fy}, {"cx", c.cx}, {"cy", c.cy}, {"k1", c.k1}, {"k2", c.k2},
            {"p1", c.p1}, {"p2", c.p2}, {"k3", c.k3}, {"width", kIntrinsicsWidth}, {"height", kIntrinsicsHeight},
            {"marker_size", s.in.marker_size}, {"revision", s.revision},
            {"overridden_by_cli", s.overridden_by_cli}, {"config_path", s.config_path}};
}

static std::string intrinsics_body(const VisionControl& ctl) {
    nlohmann::json j = intrinsics_json(ctl.intrinsics());
    j["api_version"] = kApiVersion;
    return j.dump();
}

static HttpReply put_intrinsics(VisionControl& ctl, const std::string& body, RequestPeer peer) {
    if (!peer.loopback) {
        return {403, nlohmann::json{{"api_version", kApiVersion}, {"error", "FORBIDDEN"},
                                    {"message", "PUT /api/calibration/intrinsics is accepted from a loopback peer only"}}
                         .dump()};
    }
    Intrinsics in;
    uint64_t expected = 0;
    std::string field, message;
    if (!parse_intrinsics_put(body, ctl.intrinsics().in, in, expected, field, message)) {
        nlohmann::json e = {{"api_version", kApiVersion}, {"error", "INVALID_FIELD"}, {"message", message}};
        e["field"] = field.empty() ? nlohmann::json(nullptr) : nlohmann::json(field);
        return {400, e.dump()};
    }
    uint64_t revision = 0;
    std::string error;
    switch (ctl.put_intrinsics(in, expected, revision, error)) {
        case VisionControl::IntrinsicsStatus::CliOverride:
            return {409, nlohmann::json{{"api_version", kApiVersion}, {"error", "CLI_OVERRIDE"}, {"revision", revision},
                                        {"message", "intrinsics are set on the command line; a restart would drop a "
                                                    "stored calibration"}}
                             .dump()};
        case VisionControl::IntrinsicsStatus::RevisionMismatch:
            return {409, nlohmann::json{{"api_version", kApiVersion}, {"error", "REVISION_MISMATCH"},
                                        {"revision", revision}, {"expected_revision", expected}}
                             .dump()};
        case VisionControl::IntrinsicsStatus::PersistFailed:
            return {500, nlohmann::json{{"api_version", kApiVersion}, {"error", "PERSIST_FAILED"}, {"message", error}}
                             .dump()};
        case VisionControl::IntrinsicsStatus::Ok: break;
    }
    if (!ctl.wait_intrinsics_applied(revision, VisionControl::kIntrinsicsApplyWaitMs)) {
        return {202, nlohmann::json{{"api_version", kApiVersion}, {"revision", revision}}.dump()};
    }
    return {200, intrinsics_body(ctl)};
}

static std::string status_json(const VisionControl& ctl, uint64_t now_ms = 0, RequestPeer peer = {}) {
    const uint64_t now = now_ms ? now_ms : mono_ms();
    const Mode m = ctl.mode();
    const Pipelines p = pipelines_of(m);
    const ModelSet ms = ctl.models();
    const bool gray_avail = peer.loopback && ctl.gray_available();
    const int* crop = ctl.gray_crop();
    const int* fsize = ctl.frame_size();
    MaskFocus mf;
    nlohmann::json j = {
        {"api_version", kApiVersion},
        {"instance_id", ctl.instance_id()},
        {"mode", mode_name(m)},
        {"modes", {"off", "yolo", "aruco", "both"}},
        {"pipelines", {{"yolo", p.yolo}, {"aruco", p.aruco}, {"depth", p.depth}}},
        {"last_active_mode", mode_name(ctl.last_active_mode())},
        {"enabled", ctl.continuous_enabled()},
        {"processing", ctl.processing()},
        {"fps", ctl.current_fps()},
        {"active_fps", ctl.active_fps()},
        {"idle_fps", VisionControl::kIdleFps},
        {"fps_range", {VisionControl::kMinFps, VisionControl::kMaxFps}},
        {"camera_fps", ctl.camera_fps()},
        {"measured", {{"input_fps", ctl.input_fps(now)}, {"result_fps", ctl.result_fps(now)}}},
        {"trigger", true},
        {"per_request_pipelines", true},
        {"subscriptions", true},
        {"max_fps", ctl.active_fps()},
        {"continuous", continuous_json(ctl.continuous())},
        {"available", {{"yolo", ms.yolo_available}, {"aruco", true}, {"depth", ms.depth_available}, {"gray", gray_avail}}},
        {"gray_frames",
         gray_avail
             ? nlohmann::json{{"encoding", "RGF1"},
                              {"format", "gray8"},
                              {"width", ctl.gray_width()},
                              {"height", ctl.gray_height()},
                              {"stride", ctl.gray_width()},
                              {"crop", {crop[0], crop[1], crop[2], crop[3]}},
                              {"frame_size", {fsize[0], fsize[1]}},
                              {"luma", "(77*R+150*G+29*B)>>8"},
                              {"luma_range", "full_0_255"},
                              {"ring", kGrayRingCapacity},
                              {"exposure_qualified", false}}
             : nlohmann::json(nullptr)},
        {"model", {{"name", ms.model_name}, {"path", ms.model_path}, {"kind", name_or_null(ms.model_kind)}, {"mask", ms.mask}, {"mask_max_boxes", ctl.mask_max_boxes()}}},
        {"depth_model", ms.depth_available ? nlohmann::json{{"name", ms.depth_model_name}, {"path", ms.depth_model_path}, {"metric", ms.depth_metric}} : nullptr},
        {"family", ctl.family()},
        {"mask_focus", ctl.get_mask_focus(now, mf) ? mask_focus_json(mf, now) : nullptr},
    };
    put_models(j, ctl, ms);
    j["intrinsics"] = intrinsics_json(ctl.intrinsics());
    return j.dump();
}

static std::string mode_body(const VisionControl& ctl, uint64_t now_ms = 0, RequestPeer peer = {}) {
    return status_json(ctl, now_ms, peer);
}

static bool query_param(const std::string& query, const std::string& key, std::string& value) {
    size_t pos = 0;
    while (pos <= query.size()) {
        size_t end = query.find('&', pos);
        if (end == std::string::npos) end = query.size();
        const std::string item = query.substr(pos, end - pos);
        const size_t eq = item.find('=');
        if (item.substr(0, eq) == key) {
            value = eq == std::string::npos ? "" : item.substr(eq + 1);
            return true;
        }
        pos = end + 1;
    }
    return false;
}

static bool parse_u64(const std::string& s, unsigned long long& v) {
    if (s.empty() || s.size() > 19) return false;
    for (char c : s) {
        if (c < '0' || c > '9') return false;
    }
    v = std::strtoull(s.c_str(), nullptr, 10);
    return true;
}

static bool wait_ms_param(const std::string& query, int& wait_ms) {
    std::string v;
    unsigned long long n = 0;
    if (!query_param(query, "wait_ms", v)) return true;
    if (!parse_u64(v, n)) return false;
    wait_ms = n > static_cast<unsigned long long>(VisionControl::kMaxWaitMs) ? VisionControl::kMaxWaitMs
                                                                              : static_cast<int>(n);
    return true;
}

/** Envelope members shared by latest and trigger, after the leading members. */
static void put_result(std::ostringstream& s, const VisionResult& r, const std::string& family, uint64_t now_ms) {
    const uint64_t now = now_ms == 0 ? get_unix_ms() : now_ms;
    const long long age = static_cast<long long>(now) - static_cast<long long>(r.ts_ms);
    s << std::fixed << ",\"instance_id\":" << nlohmann::json(r.instance_id).dump()
      << ",\"mode\":\"" << mode_name(r.mode) << "\",\"frame_id\":" << r.frame_id
      << ",\"ts_ms\":" << r.ts_ms << ",\"age_ms\":" << (age < 0 ? 0 : age) << ",\"inference_time_ms\":"
      << std::setprecision(1) << r.inference_ms << ",\"width\":" << r.width << ",\"height\":" << r.height
      << ",\"pipelines\":{\"yolo\":{\"ran\":" << (r.ran_yolo ? "true" : "false") << ",\"ms\":" << r.yolo_ms;
    if (r.mask_ms >= 0.0f) {
        s << ",\"mask_ms\":" << r.mask_ms;
    }
    s << "},\"aruco\":{\"ran\":" << (r.ran_aruco ? "true" : "false") << ",\"ms\":" << r.aruco_ms;
    if (!r.aruco_stages_json.empty()) s << ",\"stages\":" << r.aruco_stages_json;
    s << "},\"depth\":{\"ran\":" << (r.ran_depth ? "true" : "false") << ",\"ms\":" << r.depth_ms
      << "},\"gray\":{\"ran\":" << (r.ran_gray ? "true" : "false") << ",\"ms\":" << r.gray_ms << "}}"
      << ",\"detections\":" << r.detections_json << ",\"count\":" << r.count << ",\"family\":\"" << family
      << "\",\"undistorted\":" << (r.undistorted ? "true" : "false") << ",\"rejected\":" << r.rejected
      << ",\"markers\":" << r.markers_json << ",\"model_generation\":" << r.model_generation;
    if (r.ran_depth && !r.depth_json.empty()) {
        s << ",\"depth\":" << r.depth_json;
    }
}

static HttpReply latest_reply(VisionControl& ctl, const std::string& query, uint64_t now_ms) {
    std::string v;
    unsigned long long n = 0;
    bool has_after = false;
    uint32_t after = 0;
    int wait_ms = 0;
    if (query_param(query, "after", v)) {
        if (!parse_u64(v, n) || n > 0xFFFFFFFFull) return {400, error_body("after must be a frame_id (u32)")};
        has_after = true;
        after = static_cast<uint32_t>(n);
    }
    if (!wait_ms_param(query, wait_ms)) return {400, error_body("wait_ms must be a non-negative integer")};

    VisionResult r;
    const bool timeout = ctl.latest(has_after, after, wait_ms, r);
    const char* enabled = ctl.enabled() ? "true" : "false";
    std::ostringstream s;
    s << "{\"api_version\":" << kApiVersion << ",\"enabled\":" << enabled;
    if (r.frame_id == 0) {
        s << ",\"mode\":\"" << mode_name(ctl.mode())
          << "\",\"frame_id\":0,\"count\":0,\"detections\":[],\"markers\":[],\"timeout\":true}";
        return {200, s.str()};
    }
    put_result(s, r, ctl.family(), now_ms);
    if (timeout) s << ",\"timeout\":true";
    s << "}";
    return {200, s.str()};
}

static HttpReply yolo_unavailable_reply() {
    nlohmann::json j = {{"api_version", kApiVersion}, {"error", "yolo unavailable"},
                        {"detail", "the YOLO model did not load; aruco and off still work"}};
    return {409, j.dump()};
}

static HttpReply depth_unavailable_reply() {
    nlohmann::json j = {{"api_version", kApiVersion}, {"error", "depth unavailable"},
                        {"detail", "the depth model did not load"}};
    return {409, j.dump()};
}

/** Body as a JSON object; an empty or blank body is {}. False when it is not an object. */
static bool body_object(const std::string& body, nlohmann::json& out) {
    if (body.find_first_not_of(" \t\r\n") == std::string::npos) {
        out = nlohmann::json::object();
        return true;
    }
    out = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
    return !out.is_discarded() && out.is_object();
}

/** "pipelines": a non-empty list of "yolo" / "aruco" / "depth" (and "gray" when `allow_gray`). */
static bool parse_pipelines(const nlohmann::json& v, Pipelines& out, bool allow_gray = false) {
    if (!v.is_array() || v.empty()) return false;
    Pipelines p;
    for (const auto& x : v) {
        if (!x.is_string()) return false;
        const std::string s = x.get<std::string>();
        if (s == "yolo") p.yolo = true;
        else if (s == "aruco") p.aruco = true;
        else if (s == "depth") p.depth = true;
        else if (allow_gray && s == "gray") p.gray = true;
        else return false;
    }
    out = p;
    return true;
}

static const char* kPipelinesError = "\"pipelines\" must be a non-empty list of \"yolo\", \"aruco\", \"depth\"";
static const char* kTriggerPipelinesError =
    "\"pipelines\" must be a non-empty list of \"yolo\", \"aruco\", \"depth\", \"gray\"";

static bool parse_mask_focus(const nlohmann::json& v, MaskFocus& out) {
    if (!v.is_object()) return false;
    out = MaskFocus{};
    if (auto it = v.find("label"); it != v.end()) {
        if (!it->is_string()) return false;
        out.label = it->get<std::string>();
    }
    auto it_box = v.find("box_norm");
    if (it_box == v.end() || !it_box->is_array() || it_box->size() != 4) return false;
    for (size_t i = 0; i < 4; ++i) {
        if (!(*it_box)[i].is_number()) return false;
        out.box_norm[i] = (*it_box)[i].get<float>();
    }
    if (auto it_ttl = v.find("ttl_ms"); it_ttl != v.end()) {
        if (!it_ttl->is_number_integer()) return false;
        out.ttl_ms = it_ttl->get<int>();
    } else {
        out.ttl_ms = 2000;
    }
    out.active = true;
    return true;
}

static HttpReply post_mask_focus(VisionControl& ctl, const std::string& body, uint64_t now_ms) {
    nlohmann::json j;
    if (!body_object(body, j)) return {400, error_body("body must be a JSON object")};
    if (j.empty()) {
        ctl.clear_mask_focus();
        nlohmann::json res = {{"api_version", kApiVersion}, {"mask_focus", nullptr}};
        return {200, res.dump()};
    }
    if (j.contains("ttl_ms") && j["ttl_ms"].is_number_integer() && j["ttl_ms"].get<int>() == 0) {
        ctl.clear_mask_focus();
        nlohmann::json res = {{"api_version", kApiVersion}, {"mask_focus", nullptr}};
        return {200, res.dump()};
    }
    MaskFocus f;
    if (!parse_mask_focus(j, f)) {
        return {400, error_body("invalid mask_focus")};
    }
    int ttl = f.ttl_ms;
    if (ttl < 100) ttl = 100;
    if (ttl > 10000) ttl = 10000;
    f.ttl_ms = ttl;
    f.expires_ms = now_ms + static_cast<uint64_t>(ttl);
    ctl.set_mask_focus(f);
    nlohmann::json res = {{"api_version", kApiVersion}, {"mask_focus", mask_focus_json(f, now_ms)}};
    return {200, res.dump()};
}

static HttpReply trigger_reply(VisionControl& ctl, const std::string& query, const std::string& body,
                               uint64_t now_ms, const RequestTimes* times, RequestPeer peer) {
    int wait_ms = 2500;  // aruco-detector default for the single shot
    if (!wait_ms_param(query, wait_ms)) return {400, error_body("wait_ms must be a non-negative integer")};
    nlohmann::json j;
    if (!body_object(body, j)) return {400, error_body("body must be a JSON object")};
    if (auto it = j.find("wait_ms"); it != j.end()) {
        if (!it->is_number_unsigned()) return {400, error_body("wait_ms must be a non-negative integer")};
        wait_ms = static_cast<int>(std::min<uint64_t>(it->get<uint64_t>(), VisionControl::kMaxWaitMs));
    }
    Pipelines want = ctl.default_trigger_pipelines();
    if (auto it = j.find("pipelines"); it != j.end()) {
        if (!parse_pipelines(*it, want, /*allow_gray=*/true)) return {400, error_body(kTriggerPipelinesError)};
        if (want.gray && !peer.loopback) {
            return {403, nlohmann::json{{"api_version", kApiVersion},
                                        {"error", "FORBIDDEN"},
                                        {"detail", "gray pipeline is accepted from a loopback peer only"}}
                             .dump()};
        }
        if (want.gray && !ctl.gray_available()) {
            return {409, nlohmann::json{{"api_version", kApiVersion},
                                        {"error", "gray unavailable"},
                                        {"detail", "gray frames are not supported for the configured frame geometry"}}
                             .dump()};
        }
        if (want.yolo && !ctl.yolo_available()) return yolo_unavailable_reply();
        if (want.depth && !ctl.depth_available()) return depth_unavailable_reply();
    }
    bool has_focus = false;
    MaskFocus focus;
    if (auto it = j.find("mask_focus"); it != j.end()) {
        if (!it->is_null()) {
            if (!parse_mask_focus(*it, focus)) return {400, error_body("invalid mask_focus")};
            has_focus = true;
        }
    }

    std::string baseline_instance;
    uint32_t baseline = 0;
    ctl.current_baseline(baseline_instance, baseline);

    // Served by the first frame newer than this request, with the requested pipelines added
    // to that frame; the camera gate opens meanwhile.
    const uint64_t request_ms = mono_ms();
    const uint32_t offered0 = ctl.offered();
    VisionResult r;
    bool timeout = false;
    uint64_t gate_ms = 0;
    uint64_t wake_ms = 0;
    {
        auto scope = ctl.trigger_scope(request_ms, want, has_focus ? &focus : nullptr);
        gate_ms = mono_ms();
        timeout = ctl.wait_fresh(baseline_instance, baseline, request_ms, want, wait_ms, r);
        wake_ms = mono_ms();
    }
    std::ostringstream s;
    s << "{\"api_version\":" << kApiVersion;
    const bool same_inst = (r.instance_id == baseline_instance);
    if (timeout || r.frame_id == 0 || (same_inst && r.frame_id <= baseline)) {
        s << ",\"success\":false,\"timeout\":true,\"count\":0,\"detections\":[],\"markers\":[]}";
        return {200, s.str()};
    }
    s << ",\"success\":true,\"requested\":" << pipeline_list(want).dump();
    put_result(s, r, ctl.family(), now_ms);
    // Where the time went (mono_ms(); "skipped": frames offered after the request that did not
    // serve it, e.g. grabbed before it).
    const uint64_t end_ms = mono_ms();
    s << ",\"timing\":{\"accept_ms\":" << (times ? times->accept_ms : 0)
      << ",\"read_ms\":" << (times ? times->read_ms : 0) << ",\"request_ms\":" << request_ms
      << ",\"gate_ms\":" << gate_ms << ",\"wait_ms\":" << r.wait_ms << ",\"grab_ms\":" << r.grab_ms
      << ",\"frame_ms\":" << r.frame_ms << ",\"done_ms\":" << r.done_ms << ",\"wake_ms\":" << wake_ms
      << ",\"reply_ms\":" << end_ms << ",\"skipped\":" << (r.offered > offered0 ? r.offered - offered0 - 1 : 0)
      << "}";
    s << "}";
    return {200, s.str()};
}

static nlohmann::json depth_error_json(const char* code, const std::string& detail) {
    return {{"api_version", kApiVersion}, {"error", code}, {"detail", detail}};
}

static HttpReply depth_error(int status, const char* code, const std::string& detail) {
    return {status, depth_error_json(code, detail).dump()};
}

/**
 * GET /api/depth: one depth inference on the first frame after the request (a depth trigger;
 * the frame thread runs the depth stage, so it never runs twice at once), returned as the
 * millimetre map of depth_map_body(). One request at a time, at most one per second.
 */
static HttpReply depth_map_reply(VisionControl& ctl, const std::string& query, uint64_t now) {
    int wait_ms = 2500;
    if (!wait_ms_param(query, wait_ms)) return {400, error_body("wait_ms must be a non-negative integer")};
    const ModelSet ms = ctl.models();
    if (!ms.depth_available) return depth_unavailable_reply();
    if (!ms.depth_metric)
        return depth_error(409, "depth not metric", "the loaded depth model gives relative depth; no map in mm");
    uint64_t retry_ms = 0;
    switch (ctl.depth_map_gate().begin(now, retry_ms)) {
        case DepthMapGate::Admit::Busy:
            return depth_error(409, "depth busy", "another depth map request is running");
        case DepthMapGate::Admit::RateLimited: {
            nlohmann::json j = depth_error_json("depth rate limited", "at most one depth map per second");
            j["retry_after_ms"] = retry_ms;
            HttpReply r{429, j.dump()};
            r.headers.push_back({"Retry-After", std::to_string((retry_ms + 999) / 1000)});
            return r;
        }
        case DepthMapGate::Admit::Ok: break;
    }
    struct GateEnd {
        VisionControl& c;
        ~GateEnd() { c.depth_map_gate().end(); }
    } gate_end{ctl};

    const Pipelines want{false, false, true};
    std::string baseline_instance;
    uint32_t baseline = 0;
    ctl.current_baseline(baseline_instance, baseline);
    ctl.depth_map_wait(true);
    const uint64_t request_ms = mono_ms();
    VisionResult r;
    bool timeout = false;
    {
        auto scope = ctl.trigger_scope(request_ms, want, nullptr);
        timeout = ctl.wait_fresh(baseline_instance, baseline, request_ms, want, wait_ms, r, /*require_depth_map=*/true);
    }
    ctl.depth_map_wait(false);
    const bool same_inst = (r.instance_id == baseline_instance);
    if (timeout || r.frame_id == 0 || (same_inst && r.frame_id <= baseline)) return depth_error(504, "depth timeout", "no depth frame within wait_ms");
    if (!r.depth_disp || r.depth_grid_w <= 0 || r.depth_grid_h <= 0 ||
        r.depth_disp->size() != static_cast<size_t>(r.depth_grid_w) * static_cast<size_t>(r.depth_grid_h))
        return depth_error(503, "depth failed", "the depth stage gave no map for this frame");

    const IntrinsicsState in = ctl.intrinsics();
    DepthMapMeta meta;
    meta.width = r.depth_grid_w;
    meta.height = r.depth_grid_h;
    meta.image_w = r.width;
    meta.image_h = r.height;
    meta.frame_id = r.frame_id;
    meta.ts_ms = r.ts_ms;
    meta.inference_ms = r.depth_ms;
    meta.model = ms.depth_model_name;
    meta.fx = in.in.cam.fx;
    meta.scale_m = depth_scale_m(in.in.cam.fx, kIntrinsicsWidth);
    std::vector<uint16_t> mm;
    const DepthMapStats st = encode_depth_mm(r.depth_disp->data(), meta.width, meta.height, meta.scale_m, mm);
    HttpReply out{200, depth_map_body(meta, st, mm)};
    out.content_type = "application/octet-stream";
    return out;
}

static nlohmann::json gray_error_json(const char* code, const std::string& detail) {
    return {{"api_version", kApiVersion}, {"error", code}, {"detail", detail}};
}

static HttpReply gray_error(int status, const char* code, const std::string& detail) {
    return {status, gray_error_json(code, detail).dump()};
}

static bool valid_instance_token(const std::string& s) {
    if (s.empty() || s.size() > 64) return false;
    for (char c : s) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '-' || c == '_';
        if (!ok) return false;
    }
    return true;
}

static bool parse_gray_query(const std::string& query, std::string& instance_id, uint32_t& frame_id,
                             std::string& detail) {
    if (query.empty()) {
        detail = "query requires instance_id and frame_id";
        return false;
    }
    bool seen_inst = false;
    bool seen_fid = false;
    size_t pos = 0;
    while (pos <= query.size()) {
        size_t end = query.find('&', pos);
        if (end == std::string::npos) end = query.size();
        const std::string item = query.substr(pos, end - pos);
        if (item.empty()) {
            detail = "empty query parameter";
            return false;
        }
        const size_t eq = item.find('=');
        if (eq == std::string::npos || eq == 0) {
            detail = "malformed query parameter";
            return false;
        }
        const std::string k = item.substr(0, eq);
        const std::string v = item.substr(eq + 1);
        if (k == "instance_id") {
            if (seen_inst) {
                detail = "duplicate instance_id parameter";
                return false;
            }
            if (!valid_instance_token(v)) {
                detail = "instance_id must be 1..64 [A-Za-z0-9_-] characters";
                return false;
            }
            instance_id = v;
            seen_inst = true;
        } else if (k == "frame_id") {
            if (seen_fid) {
                detail = "duplicate frame_id parameter";
                return false;
            }
            unsigned long long n = 0;
            if (!parse_u64(v, n) || n > 0xFFFFFFFFull) {
                detail = "frame_id must be an unsigned 32-bit integer";
                return false;
            }
            frame_id = static_cast<uint32_t>(n);
            seen_fid = true;
        } else {
            detail = "unknown query parameter \"" + k + "\"";
            return false;
        }
        if (end == query.size()) break;
        pos = end + 1;
    }
    if (!seen_inst || !seen_fid) {
        detail = "both instance_id and frame_id are required";
        return false;
    }
    return true;
}

static HttpReply gray_frame_reply(VisionControl& ctl, const std::string& query, RequestPeer peer) {
    if (!peer.loopback) {
        return gray_error(403, "FORBIDDEN", "GET /api/frame/gray is accepted from a loopback peer only");
    }
    std::string inst;
    uint32_t fid = 0;
    std::string detail;
    if (!parse_gray_query(query, inst, fid, detail)) {
        return gray_error(400, "INVALID_PARAM", detail);
    }
    std::shared_ptr<void> guard = ctl.try_acquire_gray_send();
    if (!guard) {
        HttpReply busy = gray_error(503, "GRAY_BUSY", "too many concurrent gray frame transfers");
        busy.headers.push_back({"Retry-After", "1"});
        return busy;
    }
    std::shared_ptr<const GrayFrame> rec;
    switch (ctl.get_gray(inst, fid, rec)) {
        case GrayRing::Lookup::Expired:
            return gray_error(410, "FRAME_EXPIRED",
                              "gray frame is from an older instance or has been evicted from the ring");
        case GrayRing::Lookup::NotStored:
            return gray_error(404, "FRAME_NOT_STORED", "no gray frame stored for that frame_id");
        case GrayRing::Lookup::Found: {
            HttpReply out;
            out.status = 200;
            out.content_type = "application/octet-stream";
            out.body = gray_frame_header(*rec);
            out.binary_segment = rec->pixels;
            out.send_guard = std::move(guard);
            return out;
        }
    }
    return gray_error(404, "FRAME_NOT_STORED", "no gray frame stored for that frame_id");
}

static HttpReply status_reply(VisionControl::Status st, const std::string& ok_body) {
    switch (st) {
        case VisionControl::Status::BadFps:
            return {400, error_body("fps must be an integer in 1..15")};
        case VisionControl::Status::YoloUnavailable: return yolo_unavailable_reply();
        case VisionControl::Status::DepthUnavailable: return depth_unavailable_reply();
        case VisionControl::Status::Full: return {503, error_body("too many subscriptions")};
        case VisionControl::Status::NotFound: return {404, error_body("no such subscription")};
        case VisionControl::Status::SwapBusy: return {409, error_body("SWAP_BUSY")};
        default: return {200, ok_body};
    }
}

static HttpReply post_mode(VisionControl& ctl, const std::string& body, RequestPeer peer) {
    const nlohmann::json j = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded() || !j.is_object()) return {400, error_body("body must be a JSON object")};
    Mode mode;
    bool enabled = false;
    int fps = 0;
    const Mode* pm = nullptr;
    const bool* pe = nullptr;
    const int* pf = nullptr;
    if (auto it = j.find("mode"); it != j.end()) {
        if (!it->is_string() || !parse_mode(it->get<std::string>(), mode))
            return {400, error_body("\"mode\" must be one of off, yolo, aruco, both")};
        pm = &mode;
    }
    if (auto it = j.find("enabled"); it != j.end()) {
        if (!it->is_boolean()) return {400, error_body("\"enabled\" must be true or false")};
        enabled = it->get<bool>();
        pe = &enabled;
    }
    if (auto it = j.find("fps"); it != j.end()) {
        if (!it->is_number_integer()) return {400, error_body("fps must be an integer in 1..15")};
        fps = it->get<int>();
        pf = &fps;
    }
    if (!pm && !pe && !pf) return {400, error_body("body needs \"mode\", \"enabled\" or \"fps\"")};
    const VisionControl::Status st = ctl.apply(pm, pe, pf);
    return status_reply(st, st == VisionControl::Status::Ok ? mode_body(ctl, 0, peer) : "");
}

static HttpReply post_models(VisionControl& ctl, const std::string& body) {
    SwapRequest req;
    std::string err;
    if (!parse_swap_request(body, ctl.catalogue(), req, err)) return {400, error_body(err)};
    uint64_t generation = 0;
    BudgetCheck budget;
    const VisionControl::Status st = ctl.request_swap(req, generation, budget);
    if (st == VisionControl::Status::IonBudget) {
        const nlohmann::json e = {{"api_version", kApiVersion}, {"error", "ION_BUDGET"},
                                  {"need_mb", budget.need_mb}, {"limit_mb", budget.limit_mb}};
        return {409, e.dump()};
    }
    if (st != VisionControl::Status::Ok) return status_reply(st, "");
    return {202, nlohmann::json{{"api_version", kApiVersion}, {"generation", generation}}.dump()};
}

static std::string subscription_body(VisionControl& ctl, const Subscription& s, uint64_t mono) {
    nlohmann::json j = subscription_json(s, mono, ctl.active_fps());
    j["api_version"] = kApiVersion;
    j["continuous"] = continuous_json(ctl.continuous());
    return j.dump();
}

static HttpReply post_subscribe(VisionControl& ctl, const std::string& body, uint64_t mono) {
    nlohmann::json j;
    if (!body_object(body, j)) return {400, error_body("body must be a JSON object")};
    const auto c = j.find("client");
    if (c == j.end() || !c->is_string() || c->get<std::string>().empty() || c->get<std::string>().size() > 64)
        return {400, error_body("\"client\" must be a string of 1..64 characters")};
    Pipelines p;
    const auto pl = j.find("pipelines");
    if (pl == j.end() || !parse_pipelines(*pl, p)) return {400, error_body(kPipelinesError)};
    int fps = 0;
    if (auto it = j.find("fps"); it != j.end()) {
        if (!it->is_number_integer()) return {400, error_body("fps must be an integer in 1..15")};
        fps = it->get<int>();
        if (fps == 0) return {400, error_body("fps must be an integer in 1..15")};
    }
    int ttl = VisionControl::kDefaultTtlMs;
    if (auto it = j.find("ttl_ms"); it != j.end()) {
        if (!it->is_number_unsigned()) return {400, error_body("ttl_ms must be a positive integer")};
        ttl = static_cast<int>(std::min<uint64_t>(it->get<uint64_t>(), VisionControl::kMaxTtlMs));
    }
    std::string id;
    if (auto it = j.find("id"); it != j.end()) {
        if (!it->is_string()) return {400, error_body("\"id\" must be a string")};
        id = it->get<std::string>();
    }
    Subscription s;
    const VisionControl::Status st = ctl.subscribe(c->get<std::string>(), id, p, fps, ttl, mono, s);
    return status_reply(st, st == VisionControl::Status::Ok ? subscription_body(ctl, s, mono) : "");
}

static std::string subscriptions_body(VisionControl& ctl, uint64_t mono) {
    nlohmann::json list = nlohmann::json::array();
    for (const Subscription& s : ctl.subscriptions(mono)) list.push_back(subscription_json(s, mono, ctl.active_fps()));
    const Mode m = ctl.mode();
    nlohmann::json j = {
        {"api_version", kApiVersion},
        {"subscriptions", list},
        {"default", {{"mode", mode_name(m)}, {"enabled", ctl.continuous_enabled()}, {"fps", ctl.active_fps()}}},
        {"continuous", continuous_json(ctl.continuous())},
        {"max_fps", ctl.active_fps()},
        {"max_subscriptions", VisionControl::kMaxSubscriptions},
    };
    return j.dump();
}

HttpReply handle_control_request(VisionControl& ctl, const std::string& method, const std::string& target,
                                 const std::string& body, uint64_t now_ms, const RequestTimes* times,
                                 RequestPeer peer) {
    const size_t q = target.find('?');
    const std::string path = target.substr(0, q);
    const std::string query = q == std::string::npos ? "" : target.substr(q + 1);
    const uint64_t now = now_ms ? now_ms : mono_ms();

    if (path == "/api/inference") {
        if (method == "GET") return {200, inference_body(ctl)};
        if (method == "POST") {
            const nlohmann::json j = nlohmann::json::parse(body, nullptr, /*allow_exceptions=*/false);
            if (j.is_discarded() || !j.is_object()) return {400, error_body("body must be a JSON object")};
            const auto it = j.find("enabled");
            if (it == j.end() || !it->is_boolean()) return {400, error_body("\"enabled\" must be true or false")};
            ctl.set_enabled(it->get<bool>());
            return {200, inference_body(ctl)};
        }
    } else if (path == "/api/mode" || path == "/api/status") {
        if (method == "GET") return {200, status_json(ctl, now, peer)};
        if (method == "POST" && path == "/api/mode") return post_mode(ctl, body, peer);
    } else if (path == "/api/mask_focus") {
        if (method == "GET") {
            MaskFocus f;
            bool active = ctl.get_mask_focus(now, f);
            nlohmann::json res = {{"api_version", kApiVersion}, {"mask_focus", active ? mask_focus_json(f, now) : nullptr}};
            return {200, res.dump()};
        }
        if (method == "POST") return post_mask_focus(ctl, body, now);
    } else if (path == "/api/results/latest" && method == "GET") {
        return latest_reply(ctl, query, now_ms);
    } else if (path == "/api/trigger" && method == "POST") {
        return trigger_reply(ctl, query, body, now_ms, times, peer);
    } else if (path == "/api/frame/gray" && method == "GET") {
        return gray_frame_reply(ctl, query, peer);
    } else if (path == "/api/depth" && method == "GET") {
        return depth_map_reply(ctl, query, now);
    } else if (path == "/api/models" && method == "POST") {
        return post_models(ctl, body);
    } else if (path == "/api/calibration/intrinsics") {
        if (method == "GET") return {200, intrinsics_body(ctl)};
        if (method == "PUT") return put_intrinsics(ctl, body, peer);
    } else if (path == "/api/subscriptions" && method == "GET") {
        return {200, subscriptions_body(ctl, mono_ms())};
    } else if (path == "/api/subscribe" && method == "POST") {
        return post_subscribe(ctl, body, mono_ms());
    } else if (path.rfind("/api/subscribe/", 0) == 0) {
        std::string id = path.substr(std::strlen("/api/subscribe/"));
        const std::string ka = "/keepalive";
        const bool keep = id.size() > ka.size() && id.compare(id.size() - ka.size(), ka.size(), ka) == 0;
        if (keep) id.resize(id.size() - ka.size());
        if (id.empty() || id.find('/') != std::string::npos) return {404, error_body("not found")};
        const uint64_t mono = mono_ms();
        if (keep && method == "POST") {
            Subscription s;
            const VisionControl::Status st = ctl.keepalive(id, mono, s);
            return status_reply(st, st == VisionControl::Status::Ok ? subscription_body(ctl, s, mono) : "");
        }
        if (!keep && method == "DELETE") {
            if (!ctl.unsubscribe(id)) return {404, error_body("no such subscription")};
            nlohmann::json j = {{"api_version", kApiVersion}, {"deleted", id},
                                {"continuous", continuous_json(ctl.continuous())}};
            return {200, j.dump()};
        }
    }
    return {404, error_body("not found")};
}

}  // namespace rv
