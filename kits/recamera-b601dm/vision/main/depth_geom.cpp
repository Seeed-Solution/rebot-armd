#include "depth_geom.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rv {

namespace {

inline float calc_percentile(std::vector<float>& vals, float p) {
    if (vals.empty()) return 0.0f;
    if (vals.size() == 1) return vals[0];
    p = std::clamp(p, 0.0f, 1.0f);
    float rank = p * static_cast<float>(vals.size() - 1);
    size_t low = static_cast<size_t>(rank);
    size_t high = std::min(low + 1, vals.size() - 1);
    float frac = rank - static_cast<float>(low);
    std::nth_element(vals.begin(), vals.begin() + low, vals.end());
    float v_low = vals[low];
    std::nth_element(vals.begin() + low, vals.begin() + high, vals.end());
    float v_high = vals[high];
    return v_low + frac * (v_high - v_low);
}

inline float calc_median(std::vector<float>& vals) {
    return calc_percentile(vals, 0.5f);
}

inline float calc_mad(const std::vector<float>& residuals) {
    if (residuals.empty()) return 0.0f;
    std::vector<float> res_copy = residuals;
    float med = calc_median(res_copy);
    std::vector<float> abs_dev(residuals.size());
    for (size_t i = 0; i < residuals.size(); ++i) {
        abs_dev[i] = std::abs(residuals[i] - med);
    }
    return calc_median(abs_dev);
}

struct PlaneFit {
    float p0 = 0.0f;
    float pu = 0.0f;
    float pv = 0.0f;
};

PlaneFit fit_plane_irls(const std::vector<float>& dus,
                        const std::vector<float>& dvs,
                        const std::vector<float>& ds) {
    const size_t n = ds.size();
    if (n == 0) return {0.0f, 0.0f, 0.0f};

    PlaneFit fit;
    std::vector<float> ds_copy = ds;
    fit.p0 = calc_median(ds_copy);
    fit.pu = 0.0f;
    fit.pv = 0.0f;

    std::vector<float> w(n, 1.0f);

    for (int iter = 0; iter < 3; ++iter) {
        double m00 = 0.0, m01 = 0.0, m02 = 0.0;
        double m11 = 0.0, m12 = 0.0, m22 = 0.0;
        double b0 = 0.0, b1 = 0.0, b2 = 0.0;

        for (size_t i = 0; i < n; ++i) {
            const double wi = w[i];
            const double u = dus[i];
            const double v = dvs[i];
            const double d = ds[i];

            m00 += wi;
            m01 += wi * u;
            m02 += wi * v;
            m11 += wi * u * u;
            m12 += wi * u * v;
            m22 += wi * v * v;

            b0 += wi * d;
            b1 += wi * u * d;
            b2 += wi * v * d;
        }

        const double adj00 = m11 * m22 - m12 * m12;
        const double adj01 = m02 * m12 - m01 * m22;
        const double adj02 = m01 * m12 - m02 * m11;
        const double adj11 = m00 * m22 - m02 * m02;
        const double adj12 = m01 * m02 - m00 * m12;
        const double adj22 = m00 * m11 - m01 * m01;

        const double det = m00 * adj00 + m01 * adj01 + m02 * adj02;
        if (std::abs(det) < 1e-9) {
            fit.pu = 0.0f;
            fit.pv = 0.0f;
            fit.p0 = calc_median(ds_copy);
            break;
        }

        fit.p0 = static_cast<float>((adj00 * b0 + adj01 * b1 + adj02 * b2) / det);
        fit.pu = static_cast<float>((adj01 * b0 + adj11 * b1 + adj12 * b2) / det);
        fit.pv = static_cast<float>((adj02 * b0 + adj12 * b1 + adj22 * b2) / det);

        if (iter < 2) {
            std::vector<float> r(n);
            for (size_t i = 0; i < n; ++i) {
                r[i] = ds[i] - (fit.p0 + fit.pu * dus[i] + fit.pv * dvs[i]);
            }
            float s = std::max(1.4826f * calc_mad(r), 1e-4f);
            float k = 1.345f * s;
            for (size_t i = 0; i < n; ++i) {
                float abs_r = std::abs(r[i]);
                w[i] = (abs_r <= k) ? 1.0f : (k / abs_r);
            }
        }
    }
    return fit;
}

struct FgPoint {
    int gx;
    int gy;
    float u;
    float v;
    float disp;
};

}  // namespace

void resize_rgb888_bilinear(const uint8_t* src, int src_w, int src_h, int src_stride,
                             uint8_t* dst, int dst_w, int dst_h, int dst_stride) {
    if (src == nullptr || dst == nullptr || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
        return;
    }
    const int effective_dst_stride = (dst_stride > 0) ? dst_stride : (dst_w * 3);

    std::vector<int> x0_tab(dst_w);
    std::vector<int> x1_tab(dst_w);
    std::vector<int> wx_tab(dst_w);
    const float sx = static_cast<float>(src_w) / static_cast<float>(dst_w);
    for (int x = 0; x < dst_w; ++x) {
        float fx = (x + 0.5f) * sx - 0.5f;
        if (fx < 0.0f) fx = 0.0f;
        if (fx > static_cast<float>(src_w - 1)) fx = static_cast<float>(src_w - 1);
        int x0 = static_cast<int>(fx);
        int x1 = std::min(x0 + 1, src_w - 1);
        x0_tab[x] = x0 * 3;
        x1_tab[x] = x1 * 3;
        wx_tab[x] = static_cast<int>(std::round((fx - static_cast<float>(x0)) * 256.0f));
    }

    std::vector<int> y0_tab(dst_h);
    std::vector<int> y1_tab(dst_h);
    std::vector<int> wy_tab(dst_h);
    const float sy = static_cast<float>(src_h) / static_cast<float>(dst_h);
    for (int y = 0; y < dst_h; ++y) {
        float fy = (y + 0.5f) * sy - 0.5f;
        if (fy < 0.0f) fy = 0.0f;
        if (fy > static_cast<float>(src_h - 1)) fy = static_cast<float>(src_h - 1);
        int y0 = static_cast<int>(fy);
        int y1 = std::min(y0 + 1, src_h - 1);
        y0_tab[y] = y0;
        y1_tab[y] = y1;
        wy_tab[y] = static_cast<int>(std::round((fy - static_cast<float>(y0)) * 256.0f));
    }

    for (int y = 0; y < dst_h; ++y) {
        const int y0 = y0_tab[y];
        const int y1 = y1_tab[y];
        const int wy = wy_tab[y];
        const int w0 = 256 - wy;
        const int w1 = wy;

        const uint8_t* r0 = src + static_cast<size_t>(y0) * src_stride;
        const uint8_t* r1 = src + static_cast<size_t>(y1) * src_stride;
        uint8_t* out_row = dst + static_cast<size_t>(y) * effective_dst_stride;

        for (int x = 0; x < dst_w; ++x) {
            const int x0 = x0_tab[x];
            const int x1 = x1_tab[x];
            const int wx = wx_tab[x];

            const int w00 = (256 - wx) * w0;
            const int w01 = wx * w0;
            const int w10 = (256 - wx) * w1;
            const int w11 = wx * w1;

            for (int c = 0; c < 3; ++c) {
                const int val = (r0[x0 + c] * w00 + r0[x1 + c] * w01 +
                                 r1[x0 + c] * w10 + r1[x1 + c] * w11 + 32768) >> 16;
                out_row[x * 3 + c] = static_cast<uint8_t>(val);
            }
        }
    }
}

BoxDepthResult analyze_box_depth(const float* disp_map, int grid_w, int grid_h,
                                 const DepthBox& target_box,
                                 const std::vector<DepthBox>& other_boxes,
                                 float content_w, float content_h) {
    BoxDepthResult res;
    if (disp_map == nullptr || grid_w <= 0 || grid_h <= 0) return res;

    const float cell_w_px = content_w / static_cast<float>(grid_w);
    const float cell_h_px = content_h / static_cast<float>(grid_h);

    const float x0 = std::min(target_box.x0, target_box.x1);
    const float x1 = std::max(target_box.x0, target_box.x1);
    const float y0 = std::min(target_box.y0, target_box.y1);
    const float y1 = std::max(target_box.y0, target_box.y1);
    const float bw = x1 - x0;
    const float bh = y1 - y0;
    const float uc = 0.5f * (x0 + x1);
    const float vc = 0.5f * (y0 + y1);

    // Expand box by max(25%, 4 grid cells) for border ring
    const float mx = std::max(0.25f * bw, 4.0f * cell_w_px);
    const float my = std::max(0.25f * bh, 4.0f * cell_h_px);
    const float ring_x0 = x0 - mx;
    const float ring_x1 = x1 + mx;
    const float ring_y0 = y0 - my;
    const float ring_y1 = y1 + my;

    auto in_box = [](float u, float v, float bx0, float by0, float bx1, float by1) {
        return u >= bx0 && u <= bx1 && v >= by0 && v <= by1;
    };

    auto in_other_box = [&](float u, float v) {
        for (const auto& ob : other_boxes) {
            float ox0 = std::min(ob.x0, ob.x1);
            float ox1 = std::max(ob.x0, ob.x1);
            float oy0 = std::min(ob.y0, ob.y1);
            float oy1 = std::max(ob.y0, ob.y1);
            if (in_box(u, v, ox0, oy0, ox1, oy1)) return true;
        }
        return false;
    };

    std::vector<float> ring_dus;
    std::vector<float> ring_dvs;
    std::vector<float> ring_ds;
    int quad_counts[4] = {0, 0, 0, 0};

    // Find grid bounding box for ring search
    int gx_min = std::clamp(static_cast<int>(std::floor(ring_x0 / cell_w_px)), 0, grid_w - 1);
    int gx_max = std::clamp(static_cast<int>(std::ceil(ring_x1 / cell_w_px)), 0, grid_w - 1);
    int gy_min = std::clamp(static_cast<int>(std::floor(ring_y0 / cell_h_px)), 0, grid_h - 1);
    int gy_max = std::clamp(static_cast<int>(std::ceil(ring_y1 / cell_h_px)), 0, grid_h - 1);

    for (int gy = gy_min; gy <= gy_max; ++gy) {
        const float v_px = (gy + 0.5f) * cell_h_px;
        for (int gx = gx_min; gx <= gx_max; ++gx) {
            const float u_px = (gx + 0.5f) * cell_w_px;

            if (!in_box(u_px, v_px, ring_x0, ring_y0, ring_x1, ring_y1)) continue;
            if (in_box(u_px, v_px, x0, y0, x1, y1)) continue;
            if (in_other_box(u_px, v_px)) continue;

            const float du = u_px - uc;
            const float dv = v_px - vc;
            const float d = disp_map[gy * grid_w + gx];
            if (!std::isfinite(d)) continue;

            ring_dus.push_back(du);
            ring_dvs.push_back(dv);
            ring_ds.push_back(d);

            int q = 0;
            if (du >= 0.0f && dv >= 0.0f) q = 0;
            else if (du < 0.0f && dv >= 0.0f) q = 1;
            else if (du < 0.0f && dv < 0.0f) q = 2;
            else q = 3;
            quad_counts[q]++;
        }
    }

    if (ring_ds.empty()) return res;

    int num_quadrants = 0;
    for (int q = 0; q < 4; ++q) {
        if (quad_counts[q] > 0) num_quadrants++;
    }
    const float ring_cov = static_cast<float>(num_quadrants) / 4.0f;

    PlaneFit fit;
    if (ring_ds.size() >= 12 && num_quadrants >= 2) {
        fit = fit_plane_irls(ring_dus, ring_dvs, ring_ds);
    } else {
        std::vector<float> ring_copy = ring_ds;
        fit.p0 = calc_median(ring_copy);
        fit.pu = 0.0f;
        fit.pv = 0.0f;
    }

    // mad_bg = max(1.4826f * MAD(ring residuals), 1e-4f)
    std::vector<float> ring_residuals(ring_ds.size());
    for (size_t i = 0; i < ring_ds.size(); ++i) {
        ring_residuals[i] = ring_ds[i] - (fit.p0 + fit.pu * ring_dus[i] + fit.pv * ring_dvs[i]);
    }
    const float mad_bg = std::max(1.4826f * calc_mad(ring_residuals), 1e-4f);

    // Box cells
    int bgx_min = std::clamp(static_cast<int>(std::floor(x0 / cell_w_px)), 0, grid_w - 1);
    int bgx_max = std::clamp(static_cast<int>(std::ceil(x1 / cell_w_px)), 0, grid_w - 1);
    int bgy_min = std::clamp(static_cast<int>(std::floor(y0 / cell_h_px)), 0, grid_h - 1);
    int bgy_max = std::clamp(static_cast<int>(std::ceil(y1 / cell_h_px)), 0, grid_h - 1);

    std::vector<float> box_deltas;
    std::vector<FgPoint> box_pts;
    for (int gy = bgy_min; gy <= bgy_max; ++gy) {
        const float v_px = (gy + 0.5f) * cell_h_px;
        for (int gx = bgx_min; gx <= bgx_max; ++gx) {
            const float u_px = (gx + 0.5f) * cell_w_px;
            if (!in_box(u_px, v_px, x0, y0, x1, y1)) continue;

            const float d_hat = disp_map[gy * grid_w + gx];
            if (!std::isfinite(d_hat)) continue;
            const float disp_bg_hat = fit.p0 + fit.pu * (u_px - uc) + fit.pv * (v_px - vc);
            const float delta_d = d_hat - disp_bg_hat;

            box_deltas.push_back(delta_d);
            box_pts.push_back({gx, gy, u_px, v_px, d_hat});
        }
    }

    if (box_deltas.empty()) return res;

    std::vector<float> deltas_copy = box_deltas;
    const float delta_d_P80 = calc_percentile(deltas_copy, 0.80f);
    const float delta_d_P75 = calc_percentile(deltas_copy, 0.75f);
    const float fg_thresh = std::max(3.0f * mad_bg, 0.25f * delta_d_P80);

    std::vector<FgPoint> fg_pts;
    std::vector<float> fg_disp;
    for (size_t i = 0; i < box_pts.size(); ++i) {
        if (box_deltas[i] > fg_thresh) {
            fg_pts.push_back(box_pts[i]);
            fg_disp.push_back(box_pts[i].disp);
        }
    }

    const int fg_count = static_cast<int>(fg_pts.size());
    res.fg_count = fg_count;

    if (fg_count < 6 || !(delta_d_P75 > 2.5f * mad_bg)) {
        return res;
    }

    res.valid = true;
    res.disp_fg = calc_percentile(fg_disp, 0.75f);
    res.disp_bg = std::max(0.0f, fit.p0);

    const float snr = std::max(0.0f, (res.disp_fg - res.disp_bg) / mad_bg);
    const float snr_term = std::clamp(snr / (snr + 6.0f), 0.0f, 1.0f);
    const float count_term = std::min(1.0f, static_cast<float>(fg_count) / 16.0f);
    res.depth_conf = snr_term * count_term * ring_cov;

    // 2D Covariance PCA
    double sum_u = 0.0, sum_v = 0.0;
    for (const auto& pt : fg_pts) {
        sum_u += pt.u;
        sum_v += pt.v;
    }
    const double mean_u = sum_u / fg_count;
    const double mean_v = sum_v / fg_count;

    double sum_cxx = 0.0, sum_cyy = 0.0, sum_cxy = 0.0;
    for (const auto& pt : fg_pts) {
        const double du = pt.u - mean_u;
        const double dv = pt.v - mean_v;
        sum_cxx += du * du;
        sum_cyy += dv * dv;
        sum_cxy += du * dv;
    }
    const double cxx = sum_cxx / fg_count;
    const double cyy = sum_cyy / fg_count;
    const double cxy = sum_cxy / fg_count;

    const double half = std::hypot((cxx - cyy) * 0.5, cxy);
    const double eval_max = (cxx + cyy) * 0.5 + half;
    const double eval_min = std::max(0.0, (cxx + cyy) * 0.5 - half);

    double angle = (0.5 * std::atan2(2.0 * cxy, cxx - cyy)) * (180.0 / M_PI);
    while (angle >= 90.0 - 1e-5) angle -= 180.0;
    while (angle < -90.0) angle += 180.0;
    res.angle_deg = static_cast<float>(angle);

    const double eval_max_safe = std::max(eval_max, 1e-6);
    const double angle_conf = std::clamp(1.0 - eval_min / eval_max_safe, 0.0, 1.0) * res.depth_conf;
    res.angle_conf = static_cast<float>(angle_conf);

    const double theta = angle * (M_PI / 180.0);
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);

    std::vector<float> proj_maj(fg_count);
    std::vector<float> proj_min(fg_count);
    for (int i = 0; i < fg_count; ++i) {
        const double du = fg_pts[i].u - mean_u;
        const double dv = fg_pts[i].v - mean_v;
        proj_maj[i] = static_cast<float>(du * cos_t + dv * sin_t);
        proj_min[i] = static_cast<float>(-du * sin_t + dv * cos_t);
    }

    std::vector<float> p_maj_copy = proj_maj;
    std::vector<float> p_min_copy = proj_min;
    const float p5_maj = calc_percentile(p_maj_copy, 0.05f);
    const float p95_maj = calc_percentile(p_maj_copy, 0.95f);
    const float p5_min = calc_percentile(p_min_copy, 0.05f);
    const float p95_min = calc_percentile(p_min_copy, 0.95f);

    res.length_px = (p95_maj - p5_maj) + cell_w_px;
    res.grasp_width_px = (p95_min - p5_min) + cell_w_px;

    // Jaw clear test
    const float jaw_pad_px = 18.0f;
    const float major_span = res.length_px;
    const float maj_bound = 0.20f * major_span;
    const float conflict_thresh = 0.5f * (res.disp_fg - res.disp_bg);

    std::vector<bool> is_self_fg(static_cast<size_t>(grid_w * grid_h), false);
    for (const auto& pt : fg_pts) {
        is_self_fg[static_cast<size_t>(pt.gy * grid_w + pt.gx)] = true;
    }

    int total_jaw_cells = 0;
    int conflict_cells = 0;

    for (int gy = 0; gy < grid_h; ++gy) {
        const float v_px = (gy + 0.5f) * cell_h_px;
        const double dv = v_px - mean_v;
        for (int gx = 0; gx < grid_w; ++gx) {
            const float u_px = (gx + 0.5f) * cell_w_px;
            const double du = u_px - mean_u;

            const double p_maj = du * cos_t + dv * sin_t;
            if (p_maj < -maj_bound || p_maj > maj_bound) continue;

            const double p_min = -du * sin_t + dv * cos_t;
            const bool in_jaw = (p_min > p95_min && p_min <= p95_min + jaw_pad_px) ||
                                (p_min < p5_min && p_min >= p5_min - jaw_pad_px);
            if (!in_jaw) continue;

            ++total_jaw_cells;
            const size_t idx = static_cast<size_t>(gy * grid_w + gx);
            if (!is_self_fg[idx]) {
                const float d_hat = disp_map[idx];
                if (!std::isfinite(d_hat)) continue;
                const float disp_bg_hat = fit.p0 + fit.pu * static_cast<float>(u_px - uc) +
                                          fit.pv * static_cast<float>(v_px - vc);
                if (d_hat - disp_bg_hat > conflict_thresh) {
                    ++conflict_cells;
                }
            }
        }
    }

    const float conflict_ratio = (total_jaw_cells > 0)
                                     ? (static_cast<float>(conflict_cells) / static_cast<float>(total_jaw_cells))
                                     : 0.0f;
    res.jaw_clear = (conflict_ratio < 0.15f) ? 1 : 0;

    return res;
}

BoxDepthResult analyze_box_depth(const float* disp_map, int grid_w, int grid_h,
                                 size_t target_idx,
                                 const std::vector<DepthBox>& all_boxes,
                                 float content_w, float content_h) {
    if (target_idx >= all_boxes.size()) return BoxDepthResult{};
    std::vector<DepthBox> other_boxes;
    other_boxes.reserve(all_boxes.size() > 0 ? all_boxes.size() - 1 : 0);
    for (size_t i = 0; i < all_boxes.size(); ++i) {
        if (i != target_idx) other_boxes.push_back(all_boxes[i]);
    }
    return analyze_box_depth(disp_map, grid_w, grid_h, all_boxes[target_idx], other_boxes, content_w, content_h);
}

std::vector<TableSample> sample_table_disparity(const float* disp_map, int grid_w, int grid_h,
                                               const std::vector<DepthBox>& boxes,
                                               float content_w, float content_h) {
    std::vector<TableSample> samples;
    if (disp_map == nullptr || grid_w <= 0 || grid_h <= 0) return samples;

    const float cell_w_px = content_w / static_cast<float>(grid_w);
    const float cell_h_px = content_h / static_cast<float>(grid_h);

    std::vector<DepthBox> exp_boxes;
    exp_boxes.reserve(boxes.size());
    for (const auto& b : boxes) {
        float x0 = std::min(b.x0, b.x1);
        float x1 = std::max(b.x0, b.x1);
        float y0 = std::min(b.y0, b.y1);
        float y1 = std::max(b.y0, b.y1);
        float bw = x1 - x0;
        float bh = y1 - y0;
        exp_boxes.push_back({
            x0 - 0.15f * bw,
            y0 - 0.15f * bh,
            x1 + 0.15f * bw,
            y1 + 0.15f * bh
        });
    }

    auto in_any_exp_box = [&](float u, float v) {
        for (const auto& eb : exp_boxes) {
            if (u >= eb.x0 && u <= eb.x1 && v >= eb.y0 && v <= eb.y1) return true;
        }
        return false;
    };

    for (int iy = 0; iy < 6; ++iy) {
        float fy = 0.08f + (0.92f - 0.08f) * (static_cast<float>(iy) / 5.0f);
        int cgy = static_cast<int>(std::round(fy * static_cast<float>(grid_h - 1)));

        for (int ix = 0; ix < 8; ++ix) {
            float fx = 0.08f + (0.92f - 0.08f) * (static_cast<float>(ix) / 7.0f);
            int cgx = static_cast<int>(std::round(fx * static_cast<float>(grid_w - 1)));

            std::vector<float> patch_disps;
            patch_disps.reserve(25);
            double sum_u = 0.0, sum_v = 0.0;

            for (int dy = -2; dy <= 2; ++dy) {
                int gy = cgy + dy;
                if (gy < 0 || gy >= grid_h) continue;
                float v_px = (gy + 0.5f) * cell_h_px;

                for (int dx = -2; dx <= 2; ++dx) {
                    int gx = cgx + dx;
                    if (gx < 0 || gx >= grid_w) continue;
                    float u_px = (gx + 0.5f) * cell_w_px;

                    if (in_any_exp_box(u_px, v_px)) continue;

                    const float d = disp_map[gy * grid_w + gx];
                    if (!std::isfinite(d)) continue;

                    patch_disps.push_back(d);
                    sum_u += u_px;
                    sum_v += v_px;
                }
            }

            if (patch_disps.size() < 9) continue;

            std::vector<float> p_copy = patch_disps;
            float med = calc_median(p_copy);
            float mad_val = calc_mad(patch_disps);
            if (!std::isfinite(med) || !std::isfinite(mad_val)) continue;
            float mad_thresh = 0.05f * std::max(med, 1e-3f);

            if (mad_val > mad_thresh) continue;

            float avg_u = static_cast<float>(sum_u / patch_disps.size());
            float avg_v = static_cast<float>(sum_v / patch_disps.size());
            samples.push_back({avg_u, avg_v, med});
        }
    }

    return samples;
}

}  // namespace rv
