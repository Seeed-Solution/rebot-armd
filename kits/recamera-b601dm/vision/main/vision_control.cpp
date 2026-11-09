// Modified from sscma-example-sg200x solutions/aruco-detector/main/inference_control.cpp
// (commit 9af8118, Apache-2.0): modes, one shared rate, combined result envelope.
#include "vision_control.h"

#include <chrono>
#include <cstdlib>
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

static uint64_t get_unix_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
}

static int clamp_fps(int f) {
    return f < VisionControl::kMinFps ? VisionControl::kMinFps : (f > VisionControl::kMaxFps ? VisionControl::kMaxFps : f);
}

VisionControl::VisionControl(const Options& opt, OnChange on_change)
    : yolo_available_(opt.yolo_available),
      model_name_(opt.model_name),
      model_path_(opt.model_path),
      family_(opt.family),
      on_change_(std::move(on_change)),
      fps_(clamp_fps(opt.fps)) {
    Mode m = opt.mode;
    // Without a model, YOLO modes fall back to what still works.
    if (!yolo_available_) m = (m == Mode::Both || m == Mode::Aruco) ? Mode::Aruco : Mode::Off;
    mode_ = m;
    last_active_ = m != Mode::Off ? m : (yolo_available_ ? Mode::Yolo : Mode::Aruco);
    continuous_ = opt.enabled && m != Mode::Off;
    latest_.mode = mode_;
    if (on_change_) on_change_(processing_locked(), fps_locked());
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

Pipelines VisionControl::frame_pipelines() const {
    std::lock_guard<std::mutex> lk(mu_);
    if (continuous_ && mode_ != Mode::Off) return pipelines_of(mode_);
    if (triggers_ > 0) return pipelines_of(mode_ != Mode::Off ? mode_ : last_active_);
    return {};
}

VisionControl::Status VisionControl::apply(const Mode* mode, const bool* enabled, const int* fps) {
    if (fps && (*fps < kMinFps || *fps > kMaxFps)) return Status::BadFps;
    if (mode && pipelines_of(*mode).yolo && !yolo_available_) return Status::YoloUnavailable;
    {
        std::lock_guard<std::mutex> lk(mu_);
        const bool old_proc = processing_locked();
        const int old_fps = fps_locked();
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
        const bool new_proc = processing_locked();
        const int new_fps = fps_locked();
        if ((old_proc != new_proc || old_fps != new_fps) && on_change_) on_change_(new_proc, new_fps);
    }
    cv_.notify_all();
    return Status::Ok;
}

bool VisionControl::set_enabled(bool on) {
    const bool was = continuous_enabled();
    apply(nullptr, &on, nullptr);
    return was != on;
}

void VisionControl::trigger_begin() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        const bool old_proc = processing_locked();
        ++triggers_;
        if (!old_proc && on_change_) on_change_(true, fps_);
    }
    cv_.notify_all();
}

void VisionControl::trigger_end() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (triggers_ == 0) return;
        --triggers_;
        if (!processing_locked() && on_change_) on_change_(false, kIdleFps);
    }
    cv_.notify_all();
}

void VisionControl::publish(const VisionResult& r) {
    {
        std::lock_guard<std::mutex> lk(mu_);
        latest_ = r;
    }
    cv_.notify_all();
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

void VisionControl::shutdown() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stopping_ = true;
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

static std::string mode_body(const VisionControl& ctl) {
    const Mode m = ctl.mode();
    const Pipelines p = pipelines_of(m);
    nlohmann::json j = {
        {"api_version", kApiVersion},
        {"mode", mode_name(m)},
        {"modes", {"off", "yolo", "aruco", "both"}},
        {"pipelines", {{"yolo", p.yolo}, {"aruco", p.aruco}}},
        {"last_active_mode", mode_name(ctl.last_active_mode())},
        {"enabled", ctl.continuous_enabled()},
        {"processing", ctl.processing()},
        {"fps", ctl.current_fps()},
        {"active_fps", ctl.active_fps()},
        {"idle_fps", VisionControl::kIdleFps},
        {"fps_range", {VisionControl::kMinFps, VisionControl::kMaxFps}},
        {"trigger", true},
        {"available", {{"yolo", ctl.yolo_available()}, {"aruco", true}}},
        {"model", {{"name", ctl.model_name()}, {"path", ctl.model_path()}}},
        {"family", ctl.family()},
    };
    return j.dump();
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
    s << std::fixed << ",\"mode\":\"" << mode_name(r.mode) << "\",\"frame_id\":" << r.frame_id
      << ",\"ts_ms\":" << r.ts_ms << ",\"age_ms\":" << (age < 0 ? 0 : age) << ",\"inference_time_ms\":"
      << std::setprecision(1) << r.inference_ms << ",\"width\":" << r.width << ",\"height\":" << r.height
      << ",\"pipelines\":{\"yolo\":{\"ran\":" << (r.ran_yolo ? "true" : "false") << ",\"ms\":" << r.yolo_ms
      << "},\"aruco\":{\"ran\":" << (r.ran_aruco ? "true" : "false") << ",\"ms\":" << r.aruco_ms << "}}"
      << ",\"detections\":" << r.detections_json << ",\"count\":" << r.count << ",\"family\":\"" << family
      << "\",\"undistorted\":" << (r.undistorted ? "true" : "false") << ",\"rejected\":" << r.rejected
      << ",\"markers\":" << r.markers_json;
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

static HttpReply trigger_reply(VisionControl& ctl, const std::string& query, uint64_t now_ms) {
    int wait_ms = 2500;  // aruco-detector default for the single shot
    if (!wait_ms_param(query, wait_ms)) return {400, error_body("wait_ms must be a non-negative integer")};

    VisionResult current;
    ctl.latest(false, 0, 0, current);
    const uint32_t baseline = current.frame_id;

    VisionResult r;
    bool timeout = false;
    {
        auto scope = ctl.trigger_scope();
        timeout = ctl.latest(true, baseline, wait_ms, r);
    }
    std::ostringstream s;
    s << "{\"api_version\":" << kApiVersion;
    if (timeout || r.frame_id <= baseline) {
        s << ",\"success\":false,\"timeout\":true,\"count\":0,\"detections\":[],\"markers\":[]}";
        return {200, s.str()};
    }
    s << ",\"success\":true";
    put_result(s, r, ctl.family(), now_ms);
    s << "}";
    return {200, s.str()};
}

static HttpReply status_reply(VisionControl::Status st, const std::string& ok_body) {
    switch (st) {
        case VisionControl::Status::BadFps:
            return {400, error_body("fps must be an integer in 1..15")};
        case VisionControl::Status::YoloUnavailable: {
            nlohmann::json j = {{"api_version", kApiVersion}, {"error", "yolo unavailable"},
                                {"detail", "the YOLO model did not load; aruco and off still work"}};
            return {409, j.dump()};
        }
        default: return {200, ok_body};
    }
}

static HttpReply post_mode(VisionControl& ctl, const std::string& body) {
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
    return status_reply(st, st == VisionControl::Status::Ok ? mode_body(ctl) : "");
}

HttpReply handle_control_request(VisionControl& ctl, const std::string& method, const std::string& target,
                                 const std::string& body, uint64_t now_ms) {
    const size_t q = target.find('?');
    const std::string path = target.substr(0, q);
    const std::string query = q == std::string::npos ? "" : target.substr(q + 1);

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
    } else if (path == "/api/mode") {
        if (method == "GET") return {200, mode_body(ctl)};
        if (method == "POST") return post_mode(ctl, body);
    } else if (path == "/api/results/latest" && method == "GET") {
        return latest_reply(ctl, query, now_ms);
    } else if (path == "/api/trigger" && method == "POST") {
        return trigger_reply(ctl, query, now_ms);
    }
    return {404, error_body("not found")};
}

}  // namespace rv
