#include "depth_geom.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

using namespace rv;

static int g_fail = 0;
#define CHECK(c)                                                           \
    do {                                                                   \
        if (!(c)) {                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);       \
            ++g_fail;                                                      \
        }                                                                  \
    } while (0)

static void test_bilinear_resize() {
    // Test 1: Constant color
    const int src_w = 640, src_h = 360;
    const int dst_w = 384, dst_h = 224;
    std::vector<uint8_t> src(src_w * src_h * 3, 0);
    for (size_t i = 0; i < static_cast<size_t>(src_w * src_h); ++i) {
        src[i * 3 + 0] = 120;
        src[i * 3 + 1] = 200;
        src[i * 3 + 2] = 50;
    }
    std::vector<uint8_t> dst(dst_w * dst_h * 3, 0);
    resize_rgb888_bilinear(src.data(), src_w, src_h, src_w * 3, dst.data(), dst_w, dst_h);

    bool all_match = true;
    for (size_t i = 0; i < static_cast<size_t>(dst_w * dst_h); ++i) {
        if (dst[i * 3 + 0] != 120 || dst[i * 3 + 1] != 200 || dst[i * 3 + 2] != 50) {
            all_match = false;
            break;
        }
    }
    CHECK(all_match);

    // Test 2: Linear ramp with sub-pixel verification
    std::vector<uint8_t> ramp_src(src_w * src_h * 3);
    for (int y = 0; y < src_h; ++y) {
        for (int x = 0; x < src_w; ++x) {
            uint8_t v = static_cast<uint8_t>((x * 255) / (src_w - 1));
            ramp_src[(y * src_w + x) * 3 + 0] = v;
            ramp_src[(y * src_w + x) * 3 + 1] = v;
            ramp_src[(y * src_w + x) * 3 + 2] = v;
        }
    }
    std::vector<uint8_t> ramp_dst(dst_w * dst_h * 3, 0);
    resize_rgb888_bilinear(ramp_src.data(), src_w, src_h, src_w * 3, ramp_dst.data(), dst_w, dst_h);

    float max_err = 0.0f;
    for (int y = 0; y < dst_h; ++y) {
        for (int x = 0; x < dst_w; ++x) {
            float fx = (x + 0.5f) * (static_cast<float>(src_w) / static_cast<float>(dst_w)) - 0.5f;
            float expected = (fx * 255.0f) / static_cast<float>(src_w - 1);
            float actual = static_cast<float>(ramp_dst[(y * dst_w + x) * 3]);
            float err = std::abs(actual - expected);
            if (err > max_err) max_err = err;
        }
    }
    // Fixed-point rounding error should be <= 1.0 intensity step
    CHECK(max_err <= 1.5f);

    // Test 3: Stride padding
    const int padded_stride = src_w * 3 + 64;
    std::vector<uint8_t> padded_src(padded_stride * src_h, 0);
    for (int y = 0; y < src_h; ++y) {
        std::memcpy(padded_src.data() + y * padded_stride, src.data() + y * src_w * 3, src_w * 3);
    }
    std::vector<uint8_t> padded_dst(dst_w * dst_h * 3, 0);
    resize_rgb888_bilinear(padded_src.data(), src_w, src_h, padded_stride, padded_dst.data(), dst_w, dst_h);
    CHECK(std::memcmp(dst.data(), padded_dst.data(), dst.size()) == 0);
}

static void test_tilted_plane_recovery() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;
    std::vector<float> disp_map(grid_w * grid_h);

    // Ground truth plane: disp(u, v) = 0.60 + 0.0004*(u - 320) - 0.0003*(v - 180)
    const float true_p0 = 0.60f;
    const float true_pu = 0.0004f;
    const float true_pv = -0.0003f;
    const float uc = 320.0f;
    const float vc = 180.0f;

    for (int gy = 0; gy < grid_h; ++gy) {
        float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
        for (int gx = 0; gx < grid_w; ++gx) {
            float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
            disp_map[gy * grid_w + gx] = true_p0 + true_pu * (u_px - uc) + true_pv * (v_px - vc);
        }
    }

    // Add a foreground object in the center box
    DepthBox target{280.0f, 150.0f, 360.0f, 210.0f};
    for (int gy = 0; gy < grid_h; ++gy) {
        float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
        for (int gx = 0; gx < grid_w; ++gx) {
            float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
            if (u_px >= target.x0 && u_px <= target.x1 && v_px >= target.y0 && v_px <= target.y1) {
                disp_map[gy * grid_w + gx] += 0.25f;  // foreground protrusion
            }
        }
    }

    auto res = analyze_box_depth(disp_map.data(), grid_w, grid_h, target, {}, content_w, content_h);
    CHECK(res.valid);
    // Background p0 recovery error < 1%
    float p0_err = std::abs(res.disp_bg - true_p0) / true_p0;
    CHECK(p0_err < 0.01f);
    CHECK(res.disp_fg > res.disp_bg + 0.20f);
    CHECK(res.depth_conf > 0.5f);
}

// Oblique view: the table disparity changes steeply with v and the object fills only the upper part
// of its box. The exported pair must describe one location (protrusion 0.03), not the raw object
// disparity against the plane at the box centre (0.03 + slope * offset = ~0.17).
static void test_fg_bg_matched_location() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;
    const float p0 = 0.60f, pv = -0.004f, vc = 180.0f;
    std::vector<float> disp_map(grid_w * grid_h);
    DepthBox target{260.0f, 120.0f, 380.0f, 240.0f};
    float sum_v = 0.0f;
    int n_obj = 0;
    for (int gy = 0; gy < grid_h; ++gy) {
        const float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
        for (int gx = 0; gx < grid_w; ++gx) {
            const float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
            float d = p0 + pv * (v_px - vc);
            if (u_px >= target.x0 && u_px <= target.x1 && v_px >= 120.0f && v_px <= 170.0f) {
                d += 0.03f;
                sum_v += v_px;
                ++n_obj;
            }
            disp_map[gy * grid_w + gx] = d;
        }
    }
    auto res = analyze_box_depth(disp_map.data(), grid_w, grid_h, target, {}, content_w, content_h);
    CHECK(res.valid);
    const float protrusion = res.disp_fg - res.disp_bg;
    CHECK(std::abs(protrusion - 0.03f) < 0.003f);
    const float bg_at_obj = p0 + pv * (sum_v / static_cast<float>(n_obj) - vc);
    CHECK(std::abs(res.disp_bg - bg_at_obj) < 0.01f);
}

static void test_pca_aspect_correction() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;
    std::vector<float> disp_map(grid_w * grid_h, 0.40f);

    // Create an elongated bar oriented at exactly 45 degrees in 640x360 pixel coordinates
    // Center at (320, 180), length 80 px along 45 deg, width 24 px
    DepthBox target{280.0f, 140.0f, 360.0f, 220.0f};
    const float uc = 320.0f, vc = 180.0f;

    for (int gy = 0; gy < grid_h; ++gy) {
        float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
        for (int gx = 0; gx < grid_w; ++gx) {
            float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
            if (u_px >= target.x0 && u_px <= target.x1 && v_px >= target.y0 && v_px <= target.y1) {
                float du = u_px - uc;
                float dv = v_px - vc;
                // Projections along 45 deg and orthogonal (-45 deg)
                float proj_axis = (du + dv) * 0.70710678f;
                float proj_orth = (-du + dv) * 0.70710678f;
                if (std::abs(proj_axis) <= 38.0f && std::abs(proj_orth) <= 12.0f) {
                    disp_map[gy * grid_w + gx] = 0.80f;  // foreground bar
                }
            }
        }
    }

    auto res = analyze_box_depth(disp_map.data(), grid_w, grid_h, target, {}, content_w, content_h);
    CHECK(res.valid);
    // Recovered angle must be 45 degrees within 1 degree
    CHECK(std::abs(res.angle_deg - 45.0f) < 1.0f);
    CHECK(res.angle_conf > 0.5f);
    // Length along major axis should be close to ~76 px
    CHECK(res.length_px > 50.0f && res.length_px < 90.0f);
    // Grasp width along minor axis should be around ~24 px + cell_size
    CHECK(res.grasp_width_px > 15.0f && res.grasp_width_px < 35.0f);
}

static void test_jaw_clear() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;

    // Base scene: horizontal bar (angle 0) at center (320, 180)
    DepthBox target{270.0f, 165.0f, 370.0f, 195.0f};

    auto make_map = [&]() {
        std::vector<float> dmap(grid_w * grid_h, 0.40f);
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
                if (u_px >= 280.0f && u_px <= 360.0f && v_px >= 172.0f && v_px <= 188.0f) {
                    dmap[gy * grid_w + gx] = 0.85f;  // target object
                }
            }
        }
        return dmap;
    };

    // Case A: Clear
    {
        auto map_clear = make_map();
        auto res = analyze_box_depth(map_clear.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(res.valid);
        CHECK(res.jaw_clear == 1);
    }

    // Case B: Obstacle flanking the minor axis (outside [minor_min, minor_max] by 5 px)
    {
        auto map_blocked = make_map();
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
                // Flanking obstacle blocking the upper jaw footprint: v in [189..205], u in [300..340]
                if (u_px >= 300.0f && u_px <= 340.0f && v_px >= 189.0f && v_px <= 205.0f) {
                    map_blocked[gy * grid_w + gx] = 0.80f;  // obstacle in jaw footprint
                }
            }
        }
        auto res = analyze_box_depth(map_blocked.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(res.valid);
        CHECK(res.jaw_clear == 0);
    }

    auto get_jaw_band_cells = [&]() {
        const float cell_w_px = content_w / static_cast<float>(grid_w);
        const float cell_h_px = content_h / static_cast<float>(grid_h);
        const float mean_u = 320.0f;
        const float mean_v = 180.0f;
        std::vector<float> proj_maj, proj_min;
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * cell_h_px;
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * cell_w_px;
                if (u_px >= 280.0f && u_px <= 360.0f && v_px >= 172.0f && v_px <= 188.0f) {
                    proj_maj.push_back(u_px - mean_u);
                    proj_min.push_back(v_px - mean_v);
                }
            }
        }
        std::sort(proj_maj.begin(), proj_maj.end());
        std::sort(proj_min.begin(), proj_min.end());
        auto calc_p = [](const std::vector<float>& vals, float p) {
            float rank = p * static_cast<float>(vals.size() - 1);
            size_t low = static_cast<size_t>(rank);
            size_t high = std::min(low + 1, vals.size() - 1);
            float frac = rank - static_cast<float>(low);
            return vals[low] + frac * (vals[high] - vals[low]);
        };
        const float p5_maj = calc_p(proj_maj, 0.05f);
        const float p95_maj = calc_p(proj_maj, 0.95f);
        const float p5_min = calc_p(proj_min, 0.05f);
        const float p95_min = calc_p(proj_min, 0.95f);
        const float major_span = (p95_maj - p5_maj) + cell_w_px;
        const float maj_bound = 0.20f * major_span;
        const float jaw_pad_px = 18.0f;

        std::vector<size_t> jaw_cells;
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * cell_h_px;
            float dv = v_px - mean_v;
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * cell_w_px;
                float du = u_px - mean_u;
                if (du < -maj_bound || du > maj_bound) continue;
                bool in_jaw = (dv > p95_min && dv <= p95_min + jaw_pad_px) ||
                              (dv < p5_min && dv >= p5_min - jaw_pad_px);
                bool in_fg = (u_px >= 280.0f && u_px <= 360.0f && v_px >= 172.0f && v_px <= 188.0f);
                if (in_jaw && !in_fg) {
                    jaw_cells.push_back(static_cast<size_t>(gy * grid_w + gx));
                }
            }
        }
        return jaw_cells;
    };

    // Case C: All cells in lateral jaw clearance bands are NAN while foreground + border ring are finite
    {
        auto map_nan_jaw = make_map();
        auto jaw_cells = get_jaw_band_cells();
        CHECK(!jaw_cells.empty());
        for (size_t idx : jaw_cells) {
            map_nan_jaw[idx] = NAN;
        }
        auto res = analyze_box_depth(map_nan_jaw.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(res.valid);
        CHECK(res.jaw_clear == -1);
    }

    // Case D: 60% of jaw band cells are finite with 25% of those finite cells conflicting
    // (which would have been diluted to 0.15 < 0.15 if NaNs inflated denominator) -> jaw_clear == 0
    {
        auto map_partial_nan = make_map();
        auto jaw_cells = get_jaw_band_cells();
        CHECK(!jaw_cells.empty());
        const size_t n = jaw_cells.size();
        const size_t nan_count = static_cast<size_t>(0.40f * static_cast<float>(n));
        const size_t finite_count = n - nan_count;
        const size_t conflict_count = static_cast<size_t>(0.25f * static_cast<float>(finite_count));

        for (size_t i = 0; i < nan_count; ++i) {
            map_partial_nan[jaw_cells[i]] = NAN;
        }
        // Place conflict cells at the end of jaw_cells (upper jaw band outside target box)
        for (size_t i = 0; i < conflict_count; ++i) {
            map_partial_nan[jaw_cells[n - 1 - i]] = 0.80f;  // obstacle in jaw footprint
        }
        auto res = analyze_box_depth(map_partial_nan.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(res.valid);
        CHECK(res.jaw_clear == 0);
    }
}

// jaw_clear strips follow the given closing direction, not the depth PCA minor axis.
static void test_jaw_clear_direction() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;
    // Horizontal bar (PCA angle 0, minor axis vertical); the box is the bar itself.
    DepthBox target{280.0f, 172.0f, 360.0f, 188.0f};
    auto make_map = [&](bool obstacle_at_end, bool obstacle_above) {
        std::vector<float> dmap(grid_w * grid_h, 0.40f);
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
                if (u_px >= 280.0f && u_px <= 360.0f && v_px >= 172.0f && v_px <= 188.0f) {
                    dmap[gy * grid_w + gx] = 0.85f;  // target object
                }
                // beside the bar's right end, i.e. in the strip of jaws closing along u
                if (obstacle_at_end && u_px >= 362.0f && u_px <= 374.0f && v_px >= 176.0f && v_px <= 184.0f) {
                    dmap[gy * grid_w + gx] = 0.80f;
                }
                // above the bar's middle, i.e. in the strip of jaws closing along v (PCA minor axis)
                if (obstacle_above && u_px >= 300.0f && u_px <= 340.0f && v_px >= 189.0f && v_px <= 205.0f) {
                    dmap[gy * grid_w + gx] = 0.80f;
                }
            }
        }
        return dmap;
    };

    // Obstacle at the bar's end: clear for the PCA minor axis and for closing at 90 deg,
    // blocked when the jaws close along the bar (0 deg, e.g. a mask angle of -90 deg).
    {
        auto m = make_map(true, false);
        auto pca = analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(pca.valid);
        CHECK(std::abs(pca.angle_deg) < 1.0f);
        CHECK(pca.jaw_clear == 1);
        auto across = analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h, 90.0f);
        CHECK(across.jaw_clear == 1);
        auto along = analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h, 0.0f);
        CHECK(along.valid);
        CHECK(along.jaw_clear == 0);
        // the closing direction only moves the jaw strips; angle, width and length are the PCA values
        CHECK(along.angle_deg == pca.angle_deg);
        CHECK(along.grasp_width_px == pca.grasp_width_px);
        CHECK(along.length_px == pca.length_px);
        CHECK(along.depth_conf == pca.depth_conf);
    }
    // Obstacle above the bar's middle: blocked for the PCA minor axis and for 90 deg (and -90),
    // clear when the jaws close along the bar.
    {
        auto m = make_map(false, true);
        auto pca = analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(pca.jaw_clear == 0);
        CHECK(analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h, 90.0f).jaw_clear == 0);
        CHECK(analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h, -90.0f).jaw_clear == 0);
        CHECK(analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h, 0.0f).jaw_clear == 1);
        CHECK(analyze_box_depth(m.data(), grid_w, grid_h, target, {}, content_w, content_h, 180.0f).jaw_clear == 1);
    }
    // Index overload passes the direction through.
    {
        auto m = make_map(true, false);
        std::vector<DepthBox> all{target};
        CHECK(analyze_box_depth(m.data(), grid_w, grid_h, size_t{0}, all, content_w, content_h, 0.0f).jaw_clear == 0);
        CHECK(analyze_box_depth(m.data(), grid_w, grid_h, size_t{0}, all, content_w, content_h).jaw_clear == 1);
    }
}

static void test_sample_table_disparity() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;
    std::vector<float> disp_map(grid_w * grid_h, 0.50f);

    // Box at center covering [250..390] x [130..230]
    DepthBox b1{250.0f, 130.0f, 390.0f, 230.0f};
    std::vector<DepthBox> boxes = {b1};

    auto samples = sample_table_disparity(disp_map.data(), grid_w, grid_h, boxes, content_w, content_h);

    CHECK(!samples.empty());
    CHECK(samples.size() <= 48);

    // Verify all samples have disparity ~ 0.50 and are outside b1 + 15%
    const float bw = b1.x1 - b1.x0;
    const float bh = b1.y1 - b1.y0;
    const float eb_x0 = b1.x0 - 0.15f * bw;
    const float eb_x1 = b1.x1 + 0.15f * bw;
    const float eb_y0 = b1.y0 - 0.15f * bh;
    const float eb_y1 = b1.y1 + 0.15f * bh;

    for (const auto& s : samples) {
        CHECK(std::abs(s.disp - 0.50f) < 0.001f);
        bool inside = (s.u_px >= eb_x0 && s.u_px <= eb_x1 && s.v_px >= eb_y0 && s.v_px <= eb_y1);
        CHECK(!inside);
    }
}

static void test_non_finite_handling() {
    const int grid_w = 384, grid_h = 224;
    const float content_w = 640.0f, content_h = 360.0f;

    // 1. Inject NAN and INFINITY into disp_map for sample_table_disparity
    {
        std::vector<float> disp_map(grid_w * grid_h, 0.50f);
        for (int i = 0; i < grid_w * grid_h; i += 7) {
            disp_map[i] = (i % 2 == 0) ? NAN : INFINITY;
        }
        for (int i = 3; i < grid_w * grid_h; i += 11) {
            disp_map[i] = -INFINITY;
        }

        auto samples = sample_table_disparity(disp_map.data(), grid_w, grid_h, {}, content_w, content_h);
        CHECK(!samples.empty());
        for (const auto& s : samples) {
            CHECK(std::isfinite(s.u_px));
            CHECK(std::isfinite(s.v_px));
            CHECK(std::isfinite(s.disp));
        }
    }

    // 2. Verify an all-NaN ring around target returns !res.valid cleanly
    {
        std::vector<float> disp_map(grid_w * grid_h, NAN);
        DepthBox target{280.0f, 150.0f, 360.0f, 210.0f};
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
                if (u_px >= target.x0 && u_px <= target.x1 && v_px >= target.y0 && v_px <= target.y1) {
                    disp_map[gy * grid_w + gx] = 0.80f;
                }
            }
        }
        auto res = analyze_box_depth(disp_map.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(!res.valid);
    }

    // 3. Verify isolated NAN pixels inside target are skipped while res.valid remains true with finite metrics
    {
        std::vector<float> disp_map(grid_w * grid_h, 0.50f);
        DepthBox target{280.0f, 150.0f, 360.0f, 210.0f};
        int pt = 0;
        for (int gy = 0; gy < grid_h; ++gy) {
            float v_px = (gy + 0.5f) * (content_h / static_cast<float>(grid_h));
            for (int gx = 0; gx < grid_w; ++gx) {
                float u_px = (gx + 0.5f) * (content_w / static_cast<float>(grid_w));
                if (u_px >= target.x0 && u_px <= target.x1 && v_px >= target.y0 && v_px <= target.y1) {
                    ++pt;
                    if (pt % 10 == 0) {
                        disp_map[gy * grid_w + gx] = NAN;
                    } else {
                        disp_map[gy * grid_w + gx] = 0.80f;
                    }
                }
            }
        }
        auto res = analyze_box_depth(disp_map.data(), grid_w, grid_h, target, {}, content_w, content_h);
        CHECK(res.valid);
        CHECK(std::isfinite(res.disp_fg));
        CHECK(std::isfinite(res.disp_bg));
        CHECK(std::isfinite(res.depth_conf));
        CHECK(std::isfinite(res.angle_deg));
        CHECK(std::isfinite(res.angle_conf));
        CHECK(std::isfinite(res.length_px));
        CHECK(std::isfinite(res.grasp_width_px));
    }
}

int main() {
    test_bilinear_resize();
    test_tilted_plane_recovery();
    test_fg_bg_matched_location();
    test_pca_aspect_correction();
    test_jaw_clear();
    test_jaw_clear_direction();
    test_sample_table_disparity();
    test_non_finite_handling();

    if (g_fail == 0) {
        std::printf("ALL %s tests passed!\n", __FILE__);
        return 0;
    }
    std::printf("%d test(s) FAILED in %s\n", g_fail, __FILE__);
    return 1;
}
