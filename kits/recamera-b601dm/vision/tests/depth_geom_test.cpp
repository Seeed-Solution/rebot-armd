#include "depth_geom.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

int main() {
    test_bilinear_resize();
    test_tilted_plane_recovery();
    test_pca_aspect_correction();
    test_jaw_clear();
    test_sample_table_disparity();

    if (g_fail == 0) {
        std::printf("ALL %s tests passed!\n", __FILE__);
        return 0;
    }
    std::printf("%d test(s) FAILED in %s\n", g_fail, __FILE__);
    return 1;
}
