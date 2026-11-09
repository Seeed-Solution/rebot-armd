#include "result_json.h"

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
          << ",\"y\":" << y << ",\"w\":" << w << ",\"h\":" << h << '}';
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
