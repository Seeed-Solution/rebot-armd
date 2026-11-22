#include "result_json.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace rv {

ContentRect content_rect(int inference_w, int inference_h, int display_w, int display_h) noexcept {
    ContentRect r{0, 0, static_cast<float>(inference_w), static_cast<float>(inference_h)};
    if (inference_w <= 0 || inference_h <= 0 || display_w <= 0 || display_h <= 0) return r;
    const long long d = static_cast<long long>(display_w) * inference_h - static_cast<long long>(display_h) * inference_w;
    if (d > 0) {  // display wider: bars top and bottom
        r.h = static_cast<float>(inference_w) * display_h / display_w;
        r.y = (inference_h - r.h) * 0.5f;
    } else if (d < 0) {  // display taller: bars left and right
        r.w = static_cast<float>(inference_h) * display_w / display_h;
        r.x = (inference_w - r.w) * 0.5f;
    }
    return r;
}

float jaw_close_deg(const DetBox& d) noexcept {
    return std::isfinite(d.angle_deg) ? d.angle_deg + 90.0f : NAN;
}

void apply_box_depth(DetBox& d, const BoxDepthResult& r) noexcept {
    if (!r.valid) return;
    d.disp_fg = r.disp_fg;
    d.disp_bg = r.disp_bg;
    d.depth_conf = r.depth_conf;
    if (!std::isfinite(d.length_px)) d.length_px = r.length_px;
    d.jaw_clear = r.jaw_clear;
    d.depth_angle_deg = r.angle_deg;
    d.depth_angle_conf = r.angle_conf;
    d.depth_width_px = r.grasp_width_px;
}

static void json_escape(std::ostringstream& s, const char* str) {
    for (const char* c = str ? str : ""; *c; ++c) {
        if (*c == '"' || *c == '\\') s << '\\' << *c;
        else if (static_cast<unsigned char>(*c) >= 0x20) s << *c;
    }
}

std::string detections_json(const std::vector<DetBox>& dets, const ContentRect& rect, int inference_w,
                            int inference_h) {
    std::ostringstream s;
    s << std::fixed << '[';
    const float rw = rect.w > 0 ? rect.w : 1.0f;
    const float rh = rect.h > 0 ? rect.h : 1.0f;
    for (size_t i = 0; i < dets.size(); ++i) {
        const DetBox& d = dets[i];
        if (i) s << ',';
        const float x = (d.x * inference_w - rect.x) / rw;
        const float y = (d.y * inference_h - rect.y) / rh;
        const float w = d.w * inference_w / rw;
        const float h = d.h * inference_h / rh;
        s << "{\"class_id\":" << d.class_id << ",\"format\":\"xywh_center\",\"class_name\":\"";
        json_escape(s, d.class_name);
        s << "\",\"confidence\":" << std::setprecision(3) << d.confidence << ",\"x\":" << std::setprecision(4) << x
          << ",\"y\":" << y << ",\"w\":" << w << ",\"h\":" << h;
        if (d.has_obb) {
            bool all_finite = true;
            for (size_t k = 0; k < 4; ++k) {
                if (!std::isfinite(d.obb_corners[k][0]) || !std::isfinite(d.obb_corners[k][1])) {
                    all_finite = false;
                    break;
                }
            }
            if (all_finite) {
                s << ",\"obb_corners\":[";
                for (size_t k = 0; k < 4; ++k) {
                    if (k) s << ',';
                    const float cu = (d.obb_corners[k][0] * inference_w - rect.x) / rw;
                    const float cv = (d.obb_corners[k][1] * inference_h - rect.y) / rh;
                    s << '[' << std::setprecision(4) << cu << ',' << cv << ']';
                }
                s << ']';
            }
        }
        if (std::isfinite(d.angle_deg)) s << ",\"angle_deg\":" << std::setprecision(2) << d.angle_deg;
        if (std::isfinite(d.angle_conf)) s << ",\"angle_conf\":" << std::setprecision(3) << d.angle_conf;
        if (std::isfinite(d.grasp_width_px)) s << ",\"grasp_width_px\":" << std::setprecision(2) << d.grasp_width_px;
        if (d.mask_focus) s << ",\"mask_focus\":true";
        if (std::isfinite(d.disp_fg)) s << ",\"disp_fg\":" << std::setprecision(4) << d.disp_fg;
        if (std::isfinite(d.disp_bg)) s << ",\"disp_bg\":" << std::setprecision(4) << d.disp_bg;
        if (std::isfinite(d.depth_conf)) s << ",\"depth_conf\":" << std::setprecision(3) << d.depth_conf;
        if (std::isfinite(d.length_px)) s << ",\"length_px\":" << std::setprecision(2) << d.length_px;
        if (d.jaw_clear >= 0) s << ",\"jaw_clear\":" << (d.jaw_clear ? "true" : "false");
        if (std::isfinite(d.depth_angle_deg)) s << ",\"depth_angle_deg\":" << std::setprecision(2) << d.depth_angle_deg;
        if (std::isfinite(d.depth_angle_conf))
            s << ",\"depth_angle_conf\":" << std::setprecision(3) << d.depth_angle_conf;
        if (std::isfinite(d.depth_width_px)) s << ",\"depth_width_px\":" << std::setprecision(2) << d.depth_width_px;
        if (d.contour.size() >= 3) {
            s << ",\"contour\":[" << std::setprecision(4);
            const size_t n = std::min(d.contour.size(), static_cast<size_t>(kMaxContourVertices));
            for (size_t k = 0; k < n; ++k) {
                if (k) s << ',';
                const float cu = (d.contour[k][0] * inference_w - rect.x) / rw;
                const float cv = (d.contour[k][1] * inference_h - rect.y) / rh;
                s << '[' << std::min(1.0f, std::max(0.0f, std::isfinite(cu) ? cu : 0.0f)) << ','
                  << std::min(1.0f, std::max(0.0f, std::isfinite(cv) ? cv : 0.0f)) << ']';
            }
            s << ']';
        }
        s << '}';
    }
    s << ']';
    return s.str();
}

std::string markers_json(const std::vector<MarkerOut>& markers, double sx, double sy) {
    std::string out = "[";
    char item[480];
    for (size_t i = 0; i < markers.size(); ++i) {
        const MarkerOut& m = markers[i];
        if (i) out.push_back(',');
        snprintf(item, sizeof(item),
                 "{\"id\":%d,\"margin\":%.1f,\"center\":[%.1f,%.1f],\"corners\":[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]],"
                 "\"pose\":{\"t\":[%.4f,%.4f,%.4f],\"r_deg\":[%.2f,%.2f,%.2f],\"error\":%.4f}}",
                 m.id, m.margin, m.c[0] * sx, m.c[1] * sy, m.p[0][0] * sx, m.p[0][1] * sy, m.p[1][0] * sx,
                 m.p[1][1] * sy, m.p[2][0] * sx, m.p[2][1] * sy, m.p[3][0] * sx, m.p[3][1] * sy, m.t[0], m.t[1],
                 m.t[2], m.r_deg[0], m.r_deg[1], m.r_deg[2], m.error);
        out.append(item);
    }
    out.push_back(']');
    return out;
}

std::string debug_extra(Mode mode, const std::string& display_markers_json) {
    std::string s = "\"mode\":\"";
    s += mode_name(mode);
    s += "\",\"markers\":";
    s += display_markers_json.empty() ? "[]" : display_markers_json;
    return s;
}

}  // namespace rv
