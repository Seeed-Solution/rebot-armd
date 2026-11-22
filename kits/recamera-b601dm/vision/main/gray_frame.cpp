#include "gray_frame.h"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>
#include <random>

#include "json.hpp"

namespace rv {

namespace {
constexpr int kGrayApiVersion = 2;
}

bool rgb888_to_gray(const uint8_t* rgb, int w, int h, int stride, uint8_t* out) noexcept {
    if (rgb == nullptr || out == nullptr || w <= 0 || h <= 0 || stride <= 0) return false;
    const size_t min_stride = static_cast<size_t>(w) * 3u;
    if (static_cast<size_t>(stride) < min_stride) return false;
    for (int y = 0; y < h; ++y) {
        const uint8_t* src = rgb + static_cast<size_t>(y) * static_cast<size_t>(stride);
        uint8_t* dst = out + static_cast<size_t>(y) * static_cast<size_t>(w);
        for (int x = 0; x < w; ++x) {
            const uint32_t r = src[3u * static_cast<size_t>(x)];
            const uint32_t g = src[3u * static_cast<size_t>(x) + 1u];
            const uint32_t b = src[3u * static_cast<size_t>(x) + 2u];
            dst[x] = static_cast<uint8_t>((r * 77u + g * 150u + b * 29u) >> 8);
        }
    }
    return true;
}

bool extract_gray_crop(const uint8_t* frame_rgb, int frame_w, int frame_h, int frame_stride,
                       int crop_x, int crop_y, int crop_w, int crop_h, std::vector<uint8_t>& out) {
    if (frame_rgb == nullptr || frame_w <= 0 || frame_h <= 0 || frame_stride <= 0) return false;
    if (static_cast<size_t>(frame_stride) < static_cast<size_t>(frame_w) * 3u) return false;
    if (crop_x < 0 || crop_y < 0 || crop_w <= 0 || crop_h <= 0) return false;
    if (static_cast<int64_t>(crop_x) + static_cast<int64_t>(crop_w) > frame_w) return false;
    if (static_cast<int64_t>(crop_y) + static_cast<int64_t>(crop_h) > frame_h) return false;

    const size_t total = static_cast<size_t>(crop_w) * static_cast<size_t>(crop_h);
    std::vector<uint8_t> tmp;
    try {
        tmp.resize(total);
    } catch (const std::bad_alloc&) {
        return false;
    }

    const uint8_t* start = frame_rgb + static_cast<size_t>(crop_y) * static_cast<size_t>(frame_stride) +
                           static_cast<size_t>(crop_x) * 3u;
    if (!rgb888_to_gray(start, crop_w, crop_h, frame_stride, tmp.data())) return false;
    out = std::move(tmp);
    return true;
}

bool GrayFrame::valid() const noexcept {
    if (instance_id.empty() || frame_id == 0) return false;
    if (width <= 0 || height <= 0 || stride < width) return false;
    if (!pixels) return false;
    const size_t expected = static_cast<size_t>(stride) * static_cast<size_t>(height);
    return pixels->size() == expected;
}

std::string generate_instance_id() {
    static std::atomic<uint64_t> seq{0};
    const uint64_t n = seq.fetch_add(1, std::memory_order_relaxed) + 1;
    const uint64_t now = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const uint64_t pid = static_cast<uint64_t>(::getpid());
    uint64_t rnd = 0;
    try {
        std::random_device rd;
        rnd = (static_cast<uint64_t>(rd()) << 32) ^ static_cast<uint64_t>(rd());
    } catch (...) {
        rnd = now * 0x9e3779b97f4a7c15ull;
    }
    uint64_t mix = rnd ^ (now + 0x9e3779b97f4a7c15ull) ^ (pid << 16) ^ (n * 0xbf58476d1ce4e5b9ull);
    mix ^= mix >> 30;
    mix *= 0xbf58476d1ce4e5b9ull;
    mix ^= mix >> 27;
    mix *= 0x94d049bb133111ebull;
    mix ^= mix >> 31;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "rv-%016llx", static_cast<unsigned long long>(mix));
    return std::string(buf);
}

std::string gray_frame_header(const GrayFrame& f) {
    const double convert_ms = std::isfinite(f.convert_ms) ? std::round(static_cast<double>(f.convert_ms) * 100.0) / 100.0 : 0.0;
    const nlohmann::json j = {
        {"api_version", kGrayApiVersion},
        {"encoding", "RGF1"},
        {"format", "gray8"},
        {"instance_id", f.instance_id},
        {"frame_id", f.frame_id},
        {"width", f.width},
        {"height", f.height},
        {"stride", f.stride},
        {"frame_size", {f.frame_w, f.frame_h}},
        {"frame_format", "RGB888"},
        {"crop", {f.crop_x, f.crop_y, f.crop_w, f.crop_h}},
        {"channel", f.channel},
        {"luma", "(77*R+150*G+29*B)>>8"},
        {"luma_range", "full_0_255"},
        {"ts_ms", f.ts_ms},
        {"frame_mono_ms", f.frame_mono_ms},
        {"wait_mono_ms", f.wait_mono_ms},
        {"grab_mono_ms", f.grab_mono_ms},
        {"stored_mono_ms", f.stored_mono_ms},
        {"convert_ms", convert_ms},
        {"model_generation", f.model_generation},
    };
    std::string js = j.dump();
    const size_t head = sizeof(kGrayFrameMagic) + 4u;
    while ((head + js.size()) % kGrayFrameAlign != 0) js.push_back(' ');

    std::string out;
    out.reserve(head + js.size());
    out.append(kGrayFrameMagic, sizeof(kGrayFrameMagic));
    const uint32_t n = static_cast<uint32_t>(js.size());
    for (int b = 0; b < 4; ++b) out.push_back(static_cast<char>((n >> (8 * b)) & 0xFFu));
    out += js;
    return out;
}

std::string gray_frame_body(const GrayFrame& f) {
    std::string out = gray_frame_header(f);
    if (f.pixels && !f.pixels->empty()) {
        out.reserve(out.size() + f.pixels->size());
        out.append(reinterpret_cast<const char*>(f.pixels->data()), f.pixels->size());
    }
    return out;
}

bool parse_gray_frame_body(const std::string& body, std::string& json_out, std::vector<uint8_t>& pixels_out) {
    const size_t head = sizeof(kGrayFrameMagic) + 4u;
    if (body.size() < head || std::memcmp(body.data(), kGrayFrameMagic, sizeof(kGrayFrameMagic)) != 0) return false;
    uint32_t n = 0;
    for (int b = 0; b < 4; ++b) {
        n |= static_cast<uint32_t>(static_cast<uint8_t>(body[4 + b])) << (8 * b);
    }
    if (n == 0 || n > kGrayMaxHeaderBytes || (head + n) % kGrayFrameAlign != 0) return false;
    if (body.size() < head + static_cast<size_t>(n)) return false;

    const std::string js = body.substr(head, n);
    const nlohmann::json j = nlohmann::json::parse(js, nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded() || !j.is_object()) return false;
    if (!j.contains("width") || !j.contains("height") || !j.contains("stride")) return false;
    if (!j["width"].is_number_unsigned() || !j["height"].is_number_unsigned() || !j["stride"].is_number_unsigned()) {
        return false;
    }
    const uint64_t w = j["width"].get<uint64_t>();
    const uint64_t h = j["height"].get<uint64_t>();
    const uint64_t stride = j["stride"].get<uint64_t>();
    if (w == 0 || h == 0 || stride < w || w > 65536 || h > 65536 || stride > 65536) return false;

    const uint64_t payload = stride * h;
    const size_t off = head + static_cast<size_t>(n);
    if (body.size() - off != payload) return false;

    json_out = js;
    pixels_out.assign(reinterpret_cast<const uint8_t*>(body.data() + off),
                      reinterpret_cast<const uint8_t*>(body.data() + body.size()));
    return true;
}

std::shared_ptr<const GrayFrame> GrayRing::put(std::shared_ptr<const GrayFrame> frame) {
    std::shared_ptr<const GrayFrame> displaced;
    if (!frame || !frame->valid()) return displaced;
    if (ring_.size() >= capacity_) {
        displaced = std::move(ring_.front());
        ring_.pop_front();
        if (displaced) {
            if (!has_evicted_ || displaced->frame_id > highest_evicted_id_) {
                highest_evicted_id_ = displaced->frame_id;
            }
            has_evicted_ = true;
        }
    }
    ring_.push_back(std::move(frame));
    return displaced;
}

GrayRing::Lookup GrayRing::find(const std::string& instance_id, uint32_t frame_id,
                                const std::string& current_instance_id,
                                std::shared_ptr<const GrayFrame>& out) const {
    out.reset();
    if (instance_id.empty() || instance_id != current_instance_id) return Lookup::Expired;
    for (auto it = ring_.rbegin(); it != ring_.rend(); ++it) {
        if (*it && (*it)->frame_id == frame_id && (*it)->instance_id == instance_id) {
            out = *it;
            return Lookup::Found;
        }
    }
    if (has_evicted_ && frame_id <= highest_evicted_id_) return Lookup::Expired;
    return Lookup::NotStored;
}

void GrayRing::clear(std::vector<std::shared_ptr<const GrayFrame>>& displaced) {
    displaced.reserve(displaced.size() + ring_.size());
    for (auto& e : ring_) {
        if (e) displaced.push_back(std::move(e));
    }
    ring_.clear();
    has_evicted_ = false;
    highest_evicted_id_ = 0;
}

}  // namespace rv
