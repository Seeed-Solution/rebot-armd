#include "marker_filter.h"

#include <algorithm>
#include <cmath>

namespace aruco {

bool MarkerFilter::id_allowed(int id) const noexcept {
    return ids.empty() || std::find(ids.begin(), ids.end(), id) != ids.end();
}

double min_quad_side(const double p[4][2]) noexcept {
    double m = std::hypot(p[1][0] - p[0][0], p[1][1] - p[0][1]);
    for (int i = 1; i < 4; ++i) {
        const int j = (i + 1) & 3;
        m = std::min(m, std::hypot(p[j][0] - p[i][0], p[j][1] - p[i][1]));
    }
    return m;
}

FilterVerdict classify_marker(const MarkerFilter& f, int id, float decision_margin, const double p[4][2]) noexcept {
    if (decision_margin < f.min_margin) return FilterVerdict::LowMargin;
    if (min_quad_side(p) < static_cast<double>(f.min_side_px)) return FilterVerdict::SmallSide;
    if (!f.id_allowed(id)) return FilterVerdict::IdNotAllowed;
    return FilterVerdict::Accept;
}

bool parse_id_list(const std::string& text, std::vector<int>& out, std::string* err) {
    out.clear();
    auto fail = [&](const std::string& msg) {
        out.clear();
        if (err) *err = msg;
        return false;
    };
    auto is_ws = [](char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };

    size_t b = 0, e = text.size();
    while (b < e && is_ws(text[b])) ++b;
    while (e > b && is_ws(text[e - 1])) --e;
    if (b == e) return true;

    size_t pos = b;
    while (pos <= e) {
        size_t comma = text.find(',', pos);
        if (comma == std::string::npos || comma > e) comma = e;
        size_t tb = pos, te = comma;
        while (tb < te && is_ws(text[tb])) ++tb;
        while (te > tb && is_ws(text[te - 1])) --te;
        if (tb == te) return fail("empty entry in id list");
        long v = 0;
        for (size_t i = tb; i < te; ++i) {
            if (text[i] < '0' || text[i] > '9') return fail("invalid id '" + text.substr(tb, te - tb) + "'");
            v = v * 10 + (text[i] - '0');
            if (v > 65535) return fail("id out of range '" + text.substr(tb, te - tb) + "'");
        }
        out.push_back(static_cast<int>(v));
        if (comma == e) break;
        pos = comma + 1;
    }
    return true;
}

std::string format_id_list(const std::vector<int>& ids) {
    if (ids.empty()) return "all";
    std::string s;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) s += ',';
        s += std::to_string(ids[i]);
    }
    return s;
}

}  // namespace aruco
