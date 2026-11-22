#include "mask_geom.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace rv {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

void extract_points_from_mask(const MaskView& mask, std::vector<double>& xs, std::vector<double>& ys) {
    if (mask.m == nullptr || mask.w <= 0 || mask.h <= 0) {
        return;
    }
    xs.reserve(static_cast<size_t>(mask.w * mask.h) / 4);
    ys.reserve(static_cast<size_t>(mask.w * mask.h) / 4);
    for (int y = 0; y < mask.h; ++y) {
        const uint8_t* row = mask.m + static_cast<size_t>(y) * mask.stride;
        for (int x = 0; x < mask.w; ++x) {
            if (row[x] != 0) {
                xs.push_back(static_cast<double>(x));
                ys.push_back(static_cast<double>(y));
            }
        }
    }
}

}  // namespace

Moments mask_moments(const double* xs, const double* ys, size_t n, double scale_x, double scale_y) {
    if (n == 0 || xs == nullptr || ys == nullptr) {
        return {0.0, 0.0};
    }
    if (scale_x == 0.0) scale_x = 1.0;
    if (scale_y == 0.0) scale_y = 1.0;

    double sum_px = 0.0;
    double sum_py = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sum_px += xs[i] * scale_x;
        sum_py += ys[i] * scale_y;
    }
    const double mean_px = sum_px / static_cast<double>(n);
    const double mean_py = sum_py / static_cast<double>(n);

    double sum_cxx = 0.0;
    double sum_cyy = 0.0;
    double sum_cxy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dx = xs[i] * scale_x - mean_px;
        const double dy = ys[i] * scale_y - mean_py;
        sum_cxx += dx * dx;
        sum_cyy += dy * dy;
        sum_cxy += dx * dy;
    }
    const double cxx = sum_cxx / static_cast<double>(n);
    const double cyy = sum_cyy / static_cast<double>(n);
    const double cxy = sum_cxy / static_cast<double>(n);

    double angle = (0.5 * std::atan2(2.0 * cxy, cxx - cyy)) * kRadToDeg;
    if (angle >= 90.0 - 1e-9) {
        angle -= 180.0;
    }
    const double half = std::hypot((cxx - cyy) / 2.0, cxy);
    const double major = (cxx + cyy) / 2.0 + half;
    const double minor = (cxx + cyy) / 2.0 - half;
    double conf = 0.0;
    if (major > 1e-9) {
        conf = 1.0 - std::max(minor, 0.0) / major;
    }
    conf = std::clamp(conf, 0.0, 1.0);

    double rounded_angle = std::nearbyint(angle * 100.0) / 100.0;
    if (rounded_angle >= 90.0 - 1e-9) {
        rounded_angle -= 180.0;
    }
    if (std::abs(rounded_angle) < 1e-12) {
        rounded_angle = 0.0;
    }
    double rounded_conf = std::nearbyint(conf * 1000.0) / 1000.0;
    rounded_conf = std::clamp(rounded_conf, 0.0, 1.0);
    if (std::abs(rounded_conf) < 1e-12) {
        rounded_conf = 0.0;
    }

    return {rounded_angle, rounded_conf};
}

Moments mask_moments(const MaskView& mask) {
    if (mask.m == nullptr || mask.w <= 0 || mask.h <= 0) {
        return {0.0, 0.0};
    }
    std::vector<double> xs;
    std::vector<double> ys;
    extract_points_from_mask(mask, xs, ys);
    const double sx = (mask.scale_x != 0.0f) ? static_cast<double>(mask.scale_x) : 1.0;
    const double sy = (mask.scale_y != 0.0f) ? static_cast<double>(mask.scale_y) : 1.0;
    return mask_moments(xs.data(), ys.data(), xs.size(), sx, sy);
}

CrossSection mask_cross_section(const double* xs, const double* ys, size_t n, int step_deg, double band_px,
                               double scale_x, double scale_y) {
    if (n == 0 || xs == nullptr || ys == nullptr) {
        return {0.0, 0.0, 0.0, false};
    }
    if (step_deg <= 0) step_deg = 5;
    if (band_px <= 0.0) band_px = 8.0;
    if (scale_x == 0.0) scale_x = 1.0;
    if (scale_y == 0.0) scale_y = 1.0;

    std::vector<double> px(n);
    std::vector<double> py(n);
    double sum_px = 0.0;
    double sum_py = 0.0;
    for (size_t i = 0; i < n; ++i) {
        px[i] = xs[i] * scale_x;
        py[i] = ys[i] * scale_y;
        sum_px += px[i];
        sum_py += py[i];
    }
    double gx = sum_px / static_cast<double>(n);
    double gy = sum_py / static_cast<double>(n);

    const int qx = static_cast<int>(std::nearbyint(gx / scale_x));
    const int qy = static_cast<int>(std::nearbyint(gy / scale_y));

    bool found = false;
    for (size_t i = 0; i < n; ++i) {
        if (static_cast<int>(std::nearbyint(xs[i])) == qx &&
            static_cast<int>(std::nearbyint(ys[i])) == qy) {
            found = true;
            break;
        }
    }
    if (!found) {
        double min_dist_sq = std::numeric_limits<double>::infinity();
        size_t best_idx = 0;
        for (size_t i = 0; i < n; ++i) {
            const double dx = px[i] - gx;
            const double dy = py[i] - gy;
            const double dsq = dx * dx + dy * dy;
            if (dsq < min_dist_sq) {
                min_dist_sq = dsq;
                best_idx = i;
            }
        }
        gx = px[best_idx];
        gy = py[best_idx];
    }

    const double half_band = band_px / 2.0;

    std::vector<int> thetas;
    std::vector<double> widths;
    thetas.reserve(180 / step_deg + 1);
    widths.reserve(180 / step_deg + 1);

    for (int theta = 0; theta < 180; theta += step_deg) {
        thetas.push_back(theta);
        const double rad = static_cast<double>(theta) * kDegToRad;
        const double cos_t = std::cos(rad);
        const double sin_t = std::sin(rad);

        double proj_min = std::numeric_limits<double>::infinity();
        double proj_max = -std::numeric_limits<double>::infinity();
        bool any_in_band = false;

        for (size_t i = 0; i < n; ++i) {
            const double dx = px[i] - gx;
            const double dy = py[i] - gy;
            const double n_dot = -sin_t * dx + cos_t * dy;
            if (std::abs(n_dot) <= half_band) {
                const double proj = cos_t * dx + sin_t * dy;
                if (proj < proj_min) proj_min = proj;
                if (proj > proj_max) proj_max = proj;
                any_in_band = true;
            }
        }
        if (any_in_band) {
            // + one sample's extent along the closing direction: a mask sample is a cell of
            // scale_x x scale_y px
            widths.push_back(proj_max - proj_min + std::abs(cos_t) * scale_x + std::abs(sin_t) * scale_y);
        } else {
            widths.push_back(0.0);
        }
    }

    if (thetas.empty() || widths.empty()) {
        return {0.0, 0.0, 0.0, false};
    }

    double w_min = widths[0];
    double w_max = widths[0];
    for (double w : widths) {
        if (w < w_min) w_min = w;
        if (w > w_max) w_max = w;
    }

    const double thresh = std::max(w_min + 1.0, w_min * 1.05);
    const Moments moments = mask_moments(xs, ys, n, scale_x, scale_y);
    const double pca_short = moments.angle_deg + 90.0;

    int best_cand = thetas[0];
    double best_ang_dist = std::numeric_limits<double>::infinity();
    double best_width = std::numeric_limits<double>::infinity();
    bool have_best = false;

    for (size_t k = 0; k < thetas.size(); ++k) {
        if (widths[k] <= thresh) {
            const int theta = thetas[k];
            // ang_dist = abs((theta - pca_short + 90.0) % 180.0 - 90.0)
            const double v = static_cast<double>(theta) - pca_short + 90.0;
            double rem = std::fmod(v, 180.0);
            if (rem < 0.0) rem += 180.0;
            const double raw_ang_dist = std::abs(rem - 90.0);
            const double ang_dist = std::nearbyint(raw_ang_dist * 1e5) / 1e5;
            const double width = std::nearbyint(widths[k] * 1e5) / 1e5;

            bool is_better = false;
            if (!have_best) {
                is_better = true;
            } else if (ang_dist < best_ang_dist) {
                is_better = true;
            } else if (ang_dist == best_ang_dist && width < best_width) {
                is_better = true;
            }

            if (is_better) {
                have_best = true;
                best_cand = theta;
                best_ang_dist = ang_dist;
                best_width = width;
            }
        }
    }

    const int theta_star = best_cand;
    double angle_deg = static_cast<double>(theta_star) - 90.0;
    if (angle_deg >= 90.0 - 1e-9) {
        angle_deg = -90.0;
    }

    double grasp_width_px = 0.0;
    for (size_t k = 0; k < thetas.size(); ++k) {
        if (thetas[k] == theta_star) {
            grasp_width_px = widths[k];
            break;
        }
    }

    double angle_conf = 0.0;
    if (w_max >= 12.0) {
        angle_conf = std::clamp(1.0 - w_min / w_max, 0.0, 1.0);
    }

    double rounded_angle = std::nearbyint(angle_deg * 100.0) / 100.0;
    if (rounded_angle >= 90.0 - 1e-9) {
        rounded_angle = -90.0;
    }
    if (std::abs(rounded_angle) < 1e-12) {
        rounded_angle = 0.0;
    }
    double rounded_conf = std::nearbyint(angle_conf * 1000.0) / 1000.0;
    rounded_conf = std::clamp(rounded_conf, 0.0, 1.0);
    if (std::abs(rounded_conf) < 1e-12) {
        rounded_conf = 0.0;
    }
    double rounded_width = std::nearbyint(grasp_width_px * 100.0) / 100.0;

    return {rounded_angle, rounded_conf, rounded_width, true};
}

CrossSection mask_cross_section(const MaskView& mask, int step_deg, double band_px) {
    if (mask.m == nullptr || mask.w <= 0 || mask.h <= 0) {
        return {0.0, 0.0, 0.0, false};
    }
    std::vector<double> xs;
    std::vector<double> ys;
    extract_points_from_mask(mask, xs, ys);
    const double sx = (mask.scale_x != 0.0f) ? static_cast<double>(mask.scale_x) : 1.0;
    const double sy = (mask.scale_y != 0.0f) ? static_cast<double>(mask.scale_y) : 1.0;
    return mask_cross_section(xs.data(), ys.data(), xs.size(), step_deg, band_px, sx, sy);
}

void decode_crop_mask(
    const float* proto, int proto_w, int proto_h, int num_proto_channels,
    const float* coef,
    int x0, int y0, int crop_w, int crop_h,
    uint8_t* out_mask,
    float* scratch_acc
) {
    if (crop_w <= 0 || crop_h <= 0 || out_mask == nullptr) {
        return;
    }
    const size_t total_px = static_cast<size_t>(crop_w) * static_cast<size_t>(crop_h);
    if (proto == nullptr || coef == nullptr || num_proto_channels <= 0 ||
        proto_w <= 0 || proto_h <= 0) {
        std::memset(out_mask, 0, total_px);
        return;
    }

    std::vector<float> local_acc;
    float* acc = scratch_acc;
    if (acc == nullptr) {
        local_acc.resize(total_px);
        acc = local_acc.data();
    }
    std::fill_n(acc, total_px, 0.0f);

    const size_t channel_stride = static_cast<size_t>(proto_w) * static_cast<size_t>(proto_h);
    const bool fully_inside = (x0 >= 0 && y0 >= 0 && (x0 + crop_w) <= proto_w && (y0 + crop_h) <= proto_h);

    if (fully_inside) {
        for (int k = 0; k < num_proto_channels; ++k) {
            const float c_k = coef[k];
            const float* chan_ptr = proto + static_cast<size_t>(k) * channel_stride;
            for (int r = 0; r < crop_h; ++r) {
                const float* proto_row = chan_ptr + static_cast<size_t>(y0 + r) * proto_w + x0;
                float* acc_row = acc + static_cast<size_t>(r) * crop_w;
                for (int c = 0; c < crop_w; ++c) {
                    acc_row[c] += c_k * proto_row[c];
                }
            }
        }
    } else {
        for (int k = 0; k < num_proto_channels; ++k) {
            const float c_k = coef[k];
            const float* chan_ptr = proto + static_cast<size_t>(k) * channel_stride;
            for (int r = 0; r < crop_h; ++r) {
                const int y = y0 + r;
                if (y < 0 || y >= proto_h) {
                    continue;
                }
                const int x_start = std::max(0, x0);
                const int x_end = std::min(proto_w, x0 + crop_w);
                if (x_start >= x_end) {
                    continue;
                }
                const int c_offset = x_start - x0;
                const int num_cols = x_end - x_start;
                const float* proto_row = chan_ptr + static_cast<size_t>(y) * proto_w + x_start;
                float* acc_ptr = acc + static_cast<size_t>(r) * crop_w + c_offset;
                for (int c = 0; c < num_cols; ++c) {
                    acc_ptr[c] += c_k * proto_row[c];
                }
            }
        }
    }

    for (size_t i = 0; i < total_px; ++i) {
        out_mask[i] = (acc[i] > 0.0f) ? 1 : 0;
    }
}

float box_iou(float ax0, float ay0, float ax1, float ay1,
              float bx0, float by0, float bx1, float by1) {
    const float ix0 = std::max(ax0, bx0);
    const float iy0 = std::max(ay0, by0);
    const float ix1 = std::min(ax1, bx1);
    const float iy1 = std::min(ay1, by1);
    const float iw = std::max(0.0f, ix1 - ix0);
    const float ih = std::max(0.0f, iy1 - iy0);
    const float inter = iw * ih;
    const float area_a = std::max(0.0f, ax1 - ax0) * std::max(0.0f, ay1 - ay0);
    const float area_b = std::max(0.0f, bx1 - bx0) * std::max(0.0f, by1 - by0);
    const float u = area_a + area_b - inter;
    return u > 0.0f ? (inter / u) : 0.0f;
}

int select_focus_box(const MaskFocus& focus,
                     const std::vector<FocusCandidate>& candidates,
                     float content_x, float content_y,
                     float content_w, float content_h,
                     float iw, float ih) {
    if (!focus.active) return -1;
    const float rw = content_w > 0.0f ? content_w : (iw > 0.0f ? iw : 640.0f);
    const float rh = content_h > 0.0f ? content_h : (ih > 0.0f ? ih : 640.0f);
    const float cur_iw = iw > 0.0f ? iw : 640.0f;
    const float cur_ih = ih > 0.0f ? ih : 640.0f;

    int best_idx = -1;
    float best_iou = 0.09999f;  // IoU must be >= 0.1
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        if (!focus.label.empty() && focus.label != c.label) {
            continue;
        }
        const float bx0_inf = c.x - c.w * 0.5f;
        const float by0_inf = c.y - c.h * 0.5f;
        const float bx1_inf = c.x + c.w * 0.5f;
        const float by1_inf = c.y + c.h * 0.5f;
        const float bx0 = (bx0_inf * cur_iw - content_x) / rw;
        const float by0 = (by0_inf * cur_ih - content_y) / rh;
        const float bx1 = (bx1_inf * cur_iw - content_x) / rw;
        const float by1 = (by1_inf * cur_ih - content_y) / rh;
        const float iou = box_iou(focus.box_norm[0], focus.box_norm[1],
                                  focus.box_norm[2], focus.box_norm[3],
                                  bx0, by0, bx1, by1);
        if (iou > best_iou) {
            best_iou = iou;
            best_idx = static_cast<int>(i);
        }
    }
    return best_idx;
}

namespace {

double seg_dist(const std::array<float, 2>& p, const std::array<float, 2>& a, const std::array<float, 2>& b) {
    const double dx = b[0] - a[0], dy = b[1] - a[1];
    const double l2 = dx * dx + dy * dy;
    double t = l2 < 1e-18 ? 0.0 : ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2;
    t = std::max(0.0, std::min(1.0, t));
    const double ex = a[0] + t * dx - p[0], ey = a[1] + t * dy - p[1];
    return std::sqrt(ex * ex + ey * ey);
}

// Douglas-Peucker on chain[i..j] (iterative, bounded stack)
void dp_mark(const Contour& c, size_t i, size_t j, double eps, std::vector<char>& keep) {
    std::vector<std::pair<size_t, size_t>> st;
    st.emplace_back(i, j);
    while (!st.empty()) {
        auto [a, b] = st.back();
        st.pop_back();
        if (b <= a + 1) continue;
        double best = -1;
        size_t bi = a;
        for (size_t k = a + 1; k < b; ++k) {
            const double d = seg_dist(c[k], c[a], c[b]);
            if (d > best) {
                best = d;
                bi = k;
            }
        }
        if (best > eps) {
            keep[bi] = 1;
            st.emplace_back(a, bi);
            st.emplace_back(bi, b);
        }
    }
}

Contour simplify_closed(const Contour& poly, double eps, int max_vertices) {
    if (poly.size() <= 3 || max_vertices < 3) return poly;
    size_t far = 1;
    double fd = -1;
    for (size_t k = 1; k < poly.size(); ++k) {
        const double dx = poly[k][0] - poly[0][0], dy = poly[k][1] - poly[0][1];
        if (dx * dx + dy * dy > fd) {
            fd = dx * dx + dy * dy;
            far = k;
        }
    }
    // the chain far .. n-1, 0 as its own array
    Contour wrap(poly.begin() + static_cast<long>(far), poly.end());
    wrap.push_back(poly[0]);
    double e = std::max(eps, 1e-6);
    for (;;) {
        std::vector<char> keep(poly.size(), 0), keep2(wrap.size(), 0);
        keep[0] = keep[far] = 1;
        dp_mark(poly, 0, far, e, keep);
        dp_mark(wrap, 0, wrap.size() - 1, e, keep2);
        for (size_t k = 0; k + 1 < wrap.size(); ++k)
            if (keep2[k]) keep[far + k] = 1;
        Contour out;
        for (size_t k = 0; k < poly.size(); ++k)
            if (keep[k]) out.push_back(poly[k]);
        if (static_cast<int>(out.size()) <= max_vertices) return out;
        e *= 2;
    }
}

}  // namespace

Contour mask_exterior_contour(const uint8_t* m, int w, int h, int stride, int max_vertices, float eps_cells) {
    Contour out;
    if (m == nullptr || w <= 0 || h <= 0 || stride < w) return out;
    // label the 4-connected components; keep the largest
    std::vector<int> label(static_cast<size_t>(w) * h, 0);
    std::vector<int> stack;
    int best = 0, best_n = 0, next = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t s = static_cast<size_t>(y) * w + x;
            if (!m[static_cast<size_t>(y) * stride + x] || label[s]) continue;
            const int id = ++next;
            int n = 0;
            label[s] = id;
            stack.push_back(static_cast<int>(s));
            while (!stack.empty()) {
                const int k = stack.back();
                stack.pop_back();
                ++n;
                const int kx = k % w, ky = k / w;
                const int nb[4][2] = {{kx - 1, ky}, {kx + 1, ky}, {kx, ky - 1}, {kx, ky + 1}};
                for (const auto& q : nb) {
                    if (q[0] < 0 || q[1] < 0 || q[0] >= w || q[1] >= h) continue;
                    const size_t qi = static_cast<size_t>(q[1]) * w + q[0];
                    if (m[static_cast<size_t>(q[1]) * stride + q[0]] && !label[qi]) {
                        label[qi] = id;
                        stack.push_back(static_cast<int>(qi));
                    }
                }
            }
            if (n > best_n) {
                best_n = n;
                best = id;
            }
        }
    }
    if (best == 0) return out;
    auto at = [&](int x, int y) { return x >= 0 && y >= 0 && x < w && y < h && label[static_cast<size_t>(y) * w + x] == best; };
    int sx = 0, sy = 0;
    for (size_t k = 0; k < label.size(); ++k) {
        if (label[k] == best) {
            sx = static_cast<int>(k % w);
            sy = static_cast<int>(k / w);
            break;
        }
    }
    // crack following, the component on the right: 0 right, 1 down, 2 left, 3 up (y down)
    static const int D[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    int x = sx, y = sy, d = 0;
    Contour raw;
    const size_t limit = 4 * (static_cast<size_t>(w) + 1) * (h + 1) + 8;
    for (size_t it = 0; it < limit; ++it) {
        bool fl, fr;
        switch (d) {
            case 0: fl = at(x, y - 1); fr = at(x, y); break;
            case 1: fl = at(x, y); fr = at(x - 1, y); break;
            case 2: fl = at(x - 1, y); fr = at(x - 1, y - 1); break;
            default: fl = at(x - 1, y - 1); fr = at(x, y - 1); break;
        }
        const int nd = !fr ? (d + 1) % 4 : (fl ? (d + 3) % 4 : d);
        if (nd != d) raw.push_back({static_cast<float>(x), static_cast<float>(y)});
        d = nd;
        if (!fr || fl) {
            if (x == sx && y == sy && d == 0 && !raw.empty()) break;
            continue;
        }
        x += D[d][0];
        y += D[d][1];
        if (x == sx && y == sy && d == 0) break;
    }
    // start at the top-left corner of the first cell; drop a repeated closing vertex
    for (size_t k = 0; k < raw.size(); ++k) {
        if (raw[k][0] == static_cast<float>(sx) && raw[k][1] == static_cast<float>(sy)) {
            std::rotate(raw.begin(), raw.begin() + static_cast<long>(k), raw.end());
            break;
        }
    }
    raw.erase(std::unique(raw.begin(), raw.end()), raw.end());
    if (raw.size() > 1 && raw.front() == raw.back()) raw.pop_back();
    return simplify_closed(raw, eps_cells, max_vertices);
}

void contour_to_frame_norm(Contour& c, const CellToFrame& t) {
    const float s = t.scale > 0 ? t.scale : 1.0f;
    const float fw = t.frame_w > 0 ? t.frame_w : 1.0f, fh = t.frame_h > 0 ? t.frame_h : 1.0f;
    for (auto& p : c) {
        const float ix = (static_cast<float>(t.crop_x) + p[0]) * t.cell_w;
        const float iy = (static_cast<float>(t.crop_y) + p[1]) * t.cell_h;
        p[0] = ((ix - t.pad_x) / s + t.src_x) / fw;
        p[1] = ((iy - t.pad_y) / s + t.src_y) / fh;
    }
}

}  // namespace rv
