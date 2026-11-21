#include "depth_map.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "json.hpp"

namespace rv {

double depth_scale_m(double fx, double width) noexcept {
    if (!(fx > 0) || !(width > 0)) return 1.0;
    return (fx / width) * 640.0 / 600.0;
}

DepthMapStats encode_depth_mm(const float* disp, int w, int h, double scale, std::vector<uint16_t>& out) {
    DepthMapStats st;
    const size_t n = (w > 0 && h > 0) ? static_cast<size_t>(w) * static_cast<size_t>(h) : 0;
    out.assign(n, 0);
    if (disp == nullptr || n == 0) return st;
    std::vector<uint16_t> valid;
    valid.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const float d = disp[i];
        if (!std::isfinite(d) || d <= 0.0f) continue;
        const double z = scale / static_cast<double>(d);
        if (!(z >= kDepthMapMinM) || !(z <= kDepthMapMaxM)) continue;
        const long mm = std::lround(z * 1000.0);
        const uint16_t v = static_cast<uint16_t>(std::min<long>(std::max<long>(mm, 1), 65535));
        out[i] = v;
        valid.push_back(v);
    }
    st.valid = static_cast<uint32_t>(valid.size());
    if (valid.empty()) return st;
    auto at = [&](double q) {
        const size_t k = std::min(valid.size() - 1, static_cast<size_t>(q * static_cast<double>(valid.size() - 1) + 0.5));
        std::nth_element(valid.begin(), valid.begin() + static_cast<long>(k), valid.end());
        return valid[k];
    };
    st.p2_mm = at(0.02);
    st.p50_mm = at(0.50);
    st.p98_mm = at(0.98);
    st.min_mm = *std::min_element(valid.begin(), valid.end());
    st.max_mm = *std::max_element(valid.begin(), valid.end());
    return st;
}

std::string depth_map_body(const DepthMapMeta& m, const DepthMapStats& st, const std::vector<uint16_t>& mm) {
    const nlohmann::json j = {
        {"format", "u16le_mm"},
        {"width", m.width},
        {"height", m.height},
        {"units", "mm"},
        {"invalid", 0},
        {"frame_id", m.frame_id},
        {"ts_ms", m.ts_ms},
        {"inference_ms", std::round(m.inference_ms * 10.0f) / 10.0f},
        {"model", m.model},
        {"scale", {{"depth_m", "scale_m / disparity"}, {"scale_m", m.scale_m}, {"fx", m.fx}, {"fx_width", m.image_w}}},
        // the grid is the whole visible image resized (bilinear, pixel centres aligned):
        // cell (i, j) centre at ((i + 0.5) * image_w / width, (j + 0.5) * image_h / height)
        {"mapping", {{"image_w", m.image_w}, {"image_h", m.image_h}, {"fit", "stretch"},
                     {"x0", 0}, {"y0", 0}, {"x1", m.image_w}, {"y1", m.image_h}}},
        {"stats", {{"valid", st.valid}, {"min_mm", st.min_mm}, {"p2_mm", st.p2_mm}, {"p50_mm", st.p50_mm},
                   {"p98_mm", st.p98_mm}, {"max_mm", st.max_mm}}},
    };
    std::string js = j.dump();
    const size_t head = sizeof(kDepthMapMagic) + 4;
    while ((head + js.size()) % kDepthMapAlign != 0) js.push_back(' ');
    std::string out;
    out.reserve(head + js.size() + mm.size() * 2);
    out.append(kDepthMapMagic, sizeof(kDepthMapMagic));
    const uint32_t n = static_cast<uint32_t>(js.size());
    for (int b = 0; b < 4; ++b) out.push_back(static_cast<char>((n >> (8 * b)) & 0xFF));
    out += js;
    for (uint16_t v : mm) {
        out.push_back(static_cast<char>(v & 0xFF));
        out.push_back(static_cast<char>(v >> 8));
    }
    return out;
}

bool parse_depth_map_body(const std::string& body, std::string& json, std::vector<uint16_t>& mm) {
    const size_t head = sizeof(kDepthMapMagic) + 4;
    if (body.size() < head || std::memcmp(body.data(), kDepthMapMagic, sizeof(kDepthMapMagic)) != 0) return false;
    uint32_t n = 0;
    for (int b = 0; b < 4; ++b) n |= static_cast<uint32_t>(static_cast<uint8_t>(body[4 + b])) << (8 * b);
    if (body.size() < head + n || (body.size() - head - n) % 2 != 0) return false;
    json = body.substr(head, n);
    const size_t off = head + n;
    mm.resize((body.size() - off) / 2);
    for (size_t i = 0; i < mm.size(); ++i) {
        mm[i] = static_cast<uint16_t>(static_cast<uint8_t>(body[off + 2 * i]) |
                                      (static_cast<uint16_t>(static_cast<uint8_t>(body[off + 2 * i + 1])) << 8));
    }
    return true;
}

DepthMapGate::Admit DepthMapGate::begin(uint64_t now_ms, uint64_t& retry_after_ms) {
    std::lock_guard<std::mutex> lk(mu_);
    retry_after_ms = 0;
    if (running_) return Admit::Busy;
    if (started_ && now_ms < last_ms_ + min_interval_ms_) {
        retry_after_ms = last_ms_ + min_interval_ms_ - now_ms;
        return Admit::RateLimited;
    }
    running_ = true;
    started_ = true;
    last_ms_ = now_ms;
    return Admit::Ok;
}

void DepthMapGate::end() {
    std::lock_guard<std::mutex> lk(mu_);
    running_ = false;
}

}  // namespace rv
