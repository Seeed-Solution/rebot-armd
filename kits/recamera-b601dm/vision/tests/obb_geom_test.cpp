#include <cmath>
#include <cstdio>
#include <vector>

#include "json.hpp"
#include "obb_geom.h"

using namespace rv;
using json = nlohmann::json;

static int g_fail = 0;
#define CHECK(c)                                                             \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);         \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

static void test_prob_iou() {
    ObbBox b1;
    b1.cx = 100.0f;
    b1.cy = 100.0f;
    b1.w = 40.0f;
    b1.h = 20.0f;
    b1.angle_rad = 0.0f;
    b1.score = 0.9f;
    b1.class_id = 0;

    // Identical box -> IoU == 1.0f
    float iou_same = prob_iou(b1, b1);
    CHECK(std::abs(iou_same - 1.0f) < 1e-4f);

    // Symmetric
    ObbBox b2 = b1;
    b2.cx = 110.0f;
    b2.cy = 105.0f;
    b2.angle_rad = 0.3f;
    float iou_12 = prob_iou(b1, b2);
    float iou_21 = prob_iou(b2, b1);
    CHECK(std::abs(iou_12 - iou_21) < 1e-5f);
    CHECK(iou_12 > 0.3f && iou_12 < 0.95f);

    // Completely disjoint box -> IoU very close to 0
    ObbBox b3 = b1;
    b3.cx = 500.0f;
    b3.cy = 500.0f;
    float iou_disjoint = prob_iou(b1, b3);
    CHECK(iou_disjoint < 1e-3f);
}

static void test_obb_nms() {
    std::vector<ObbBox> boxes;
    ObbBox b1;
    b1.cx = 200.0f; b1.cy = 200.0f; b1.w = 50.0f; b1.h = 25.0f;
    b1.angle_rad = 0.1f; b1.score = 0.95f; b1.class_id = 0;

    // Duplicate box with slightly lower score
    ObbBox b2 = b1;
    b2.cx = 202.0f; b2.cy = 201.0f; b2.score = 0.85f;

    // Box of a different class at the same location
    ObbBox b3 = b1;
    b3.class_id = 1; b3.score = 0.90f;

    // Distant box
    ObbBox b4;
    b4.cx = 50.0f; b4.cy = 50.0f; b4.w = 30.0f; b4.h = 30.0f;
    b4.angle_rad = 0.0f; b4.score = 0.80f; b4.class_id = 0;

    boxes = {b2, b1, b3, b4};
    auto kept_class_aware = obb_nms(boxes, 0.45f, true);
    // b1 suppresses b2 (same class 0). b3 (class 1) is kept. b4 (distant) is kept. Total = 3.
    CHECK(kept_class_aware.size() == 3);
    CHECK(kept_class_aware[0].score == 0.95f);

    boxes = {b2, b1, b3, b4};
    auto kept_class_agnostic = obb_nms(boxes, 0.45f, false);
    // b1 suppresses both b2 and b3. Total = 2 (b1 and b4).
    CHECK(kept_class_agnostic.size() == 2);
    CHECK(kept_class_agnostic[0].score == 0.95f);
    CHECK(kept_class_agnostic[1].score == 0.80f);
}

static void test_obb_to_geom() {
    // 1. Horizontal box: w = 60 >= h = 20, angle = 0 -> long side angle 0 deg
    ObbBox b1;
    b1.cx = 320.0f; b1.cy = 320.0f; b1.w = 60.0f; b1.h = 20.0f;
    b1.angle_rad = 0.0f; b1.score = 0.88f; b1.class_id = 3;

    ObbGeom g1 = obb_to_geom(b1, 640.0f, 640.0f);
    CHECK(std::abs(g1.grasp_width_px - 20.0f) < 1e-3f);
    CHECK(std::abs(g1.length_px - 60.0f) < 1e-3f);
    CHECK(std::abs(g1.angle_deg - 0.0f) < 1e-2f);
    CHECK(std::abs(g1.angle_conf - (1.0f - 20.0f / 60.0f)) < 1e-3f);
    CHECK(std::abs(g1.x - 0.5f) < 1e-3f && std::abs(g1.y - 0.5f) < 1e-3f);

    // 2. Vertical box: w = 20 < h = 60, angle = 0 rad -> long side angle = 0 + pi/2 = 90 deg -> wrapped to -90 deg
    ObbBox b2 = b1;
    b2.w = 20.0f; b2.h = 60.0f; b2.angle_rad = 0.0f;
    ObbGeom g2 = obb_to_geom(b2, 640.0f, 640.0f);
    CHECK(std::abs(g2.angle_deg - (-90.0f)) < 1e-2f);
    CHECK(std::abs(g2.grasp_width_px - 20.0f) < 1e-3f);
    CHECK(std::abs(g2.length_px - 60.0f) < 1e-3f);

    // 3. 45 deg box
    ObbBox b3 = b1;
    b3.w = 40.0f; b3.h = 20.0f; b3.angle_rad = static_cast<float>(M_PI / 4.0); // +45 deg
    ObbGeom g3 = obb_to_geom(b3, 640.0f, 640.0f);
    CHECK(std::abs(g3.angle_deg - 45.0f) < 1e-2f);

    // Perimeter order check: check side vectors of corners 0->1->2->3->0
    const auto& c = g3.obb_corners;
    // Edge 0->1 should be along w (length 40/640)
    float e01_x = (c[1][0] - c[0][0]) * 640.0f;
    float e01_y = (c[1][1] - c[0][1]) * 640.0f;
    float len01 = std::hypot(e01_x, e01_y);
    CHECK(std::abs(len01 - 40.0f) < 1e-2f);

    // Edge 1->2 should be along h (length 20/640)
    float e12_x = (c[2][0] - c[1][0]) * 640.0f;
    float e12_y = (c[2][1] - c[1][1]) * 640.0f;
    float len12 = std::hypot(e12_x, e12_y);
    CHECK(std::abs(len12 - 20.0f) < 1e-2f);

    // Orthogonal: e01 . e12 == 0
    float dot = e01_x * e12_x + e01_y * e12_y;
    CHECK(std::abs(dot) < 1e-3f);

    // 4. Content rect unmapping
    // Suppose content rect is 640x360 centered in 640x640 (content_y = 140, content_h = 360)
    ObbGeom g_content = obb_to_geom(b1, 640.0f, 640.0f, 0.0f, 140.0f, 640.0f, 360.0f);
    // Center at y = 320 px inside 140..500 -> (320 - 140) / 360 = 180 / 360 = 0.5f
    CHECK(std::abs(g_content.x - 0.5f) < 1e-3f);
    CHECK(std::abs(g_content.y - 0.5f) < 1e-3f);
    CHECK(g_content.obb_corners[0][0] >= 0.0f && g_content.obb_corners[0][0] <= 1.0f);
}

static void test_decode_layout_a() {
    // 1 box, 2 classes (num_channels = 5 + 2 = 7), 1 candidate
    // channels: cx, cy, w, h, cls0, cls1, angle
    const int num_channels = 7;
    const int num_boxes = 2;
    // Box 0: score 0.85 on cls1
    // Box 1: score 0.10 on cls0
    std::vector<float> data(num_channels * num_boxes, 0.0f);
    // Channel 0: cx
    data[0 * num_boxes + 0] = 150.0f;
    data[0 * num_boxes + 1] = 50.0f;
    // Channel 1: cy
    data[1 * num_boxes + 0] = 200.0f;
    data[1 * num_boxes + 1] = 60.0f;
    // Channel 2: w
    data[2 * num_boxes + 0] = 30.0f;
    data[2 * num_boxes + 1] = 10.0f;
    // Channel 3: h
    data[3 * num_boxes + 0] = 40.0f;
    data[3 * num_boxes + 1] = 15.0f;
    // Channel 4: cls0
    data[4 * num_boxes + 0] = 0.2f;
    data[4 * num_boxes + 1] = 0.1f;
    // Channel 5: cls1
    data[5 * num_boxes + 0] = 0.85f;
    data[5 * num_boxes + 1] = 0.05f;
    // Channel 6: angle
    data[6 * num_boxes + 0] = 0.5f;
    data[6 * num_boxes + 1] = -0.2f;

    auto res = decode_obb_layout_a(data.data(), num_channels, num_boxes, 0.25f, true);
    CHECK(res.size() == 1);
    CHECK(res[0].class_id == 1);
    CHECK(std::abs(res[0].score - 0.85f) < 1e-5f);
    CHECK(std::abs(res[0].cx - 150.0f) < 1e-5f);
    CHECK(std::abs(res[0].cy - 200.0f) < 1e-5f);
    CHECK(std::abs(res[0].w - 30.0f) < 1e-5f);
    CHECK(std::abs(res[0].h - 40.0f) < 1e-5f);
    CHECK(std::abs(res[0].angle_rad - 0.5f) < 1e-5f);
}

static void test_decode_layout_b() {
    // 9 tensors: 3 scales. Let's make scale 0 (stride 8, grid 80x80) have 1 detection.
    // Scales: box@0, cls@1, ang@2 (interleaved)
    // Box tensor: [1, 64, 80, 80] (NCHW)
    // Cls tensor: [1, 1, 80, 80]
    // Ang tensor: [1, 1, 80, 80]
    std::vector<float> box0(64 * 80 * 80, 0.0f);
    std::vector<float> cls0(1 * 80 * 80, -10.0f); // all low
    std::vector<float> ang0(1 * 80 * 80, 0.0f);

    // Place a high score at cell (10, 20): gy = 10, gx = 20
    int cell_idx = 10 * 80 + 20;
    cls0[cell_idx] = 5.0f; // sigmoid(5) ~ 0.993
    ang0[cell_idx] = 0.0f; // sigmoid(0) = 0.5 -> (0.5 - 0.25) * pi = 0.25 * pi

    // DFL bins: set bin 4 to high value for all 4 coordinates [lt_x, lt_y, rb_x, rb_y]
    // Each coord b in 0..3 has 16 bins
    for (int b = 0; b < 4; ++b) {
        int ch = b * 16 + 4; // bin 4
        box0[ch * (80 * 80) + cell_idx] = 10.0f;
    }

    std::vector<float> dummy(64 * 40 * 40, 0.0f);
    std::vector<float> dummy_cls(1 * 40 * 40, -10.0f);
    std::vector<float> dummy_ang(1 * 40 * 40, 0.0f);

    std::array<ObbTensorView, 9> views{};
    // Scale 0 (80x80)
    views[0].data = box0.data(); views[0].dims = {1, 64, 80, 80};
    views[1].data = cls0.data(); views[1].dims = {1, 1, 80, 80};
    views[2].data = ang0.data(); views[2].dims = {1, 1, 80, 80};

    // Scale 1 (40x40)
    views[3].data = dummy.data(); views[3].dims = {1, 64, 40, 40};
    views[4].data = dummy_cls.data(); views[4].dims = {1, 1, 40, 40};
    views[5].data = dummy_ang.data(); views[5].dims = {1, 1, 40, 40};

    // Scale 2 (20x20)
    std::vector<float> dummy20(64 * 20 * 20, 0.0f);
    std::vector<float> dummy20_cls(1 * 20 * 20, -10.0f);
    std::vector<float> dummy20_ang(1 * 20 * 20, 0.0f);
    views[6].data = dummy20.data(); views[6].dims = {1, 64, 20, 20};
    views[7].data = dummy20_cls.data(); views[7].dims = {1, 1, 20, 20};
    views[8].data = dummy20_ang.data(); views[8].dims = {1, 1, 20, 20};

    auto res = decode_obb_layout_b(views.data(), 9, 640.0f, 640.0f, 0.5f);
    CHECK(res.size() == 1);
    CHECK(res[0].score > 0.9f);
    // Stride is 640 / 80 = 8.0f
    // DFL bin 4 * stride = 4 * 8 = 32.0f for all coordinates
    // lt_x = 32, rb_x = 32 -> w = 64
    // lt_y = 32, rb_y = 32 -> h = 64
    // xf = 0, yf = 0 -> dx = 0, dy = 0
    // cx = (20 + 0.5) * 8 = 164.0f
    // cy = (10 + 0.5) * 8 = 84.0f
    CHECK(std::abs(res[0].w - 64.0f) < 0.1f);
    CHECK(std::abs(res[0].h - 64.0f) < 0.1f);
    CHECK(std::abs(res[0].cx - 164.0f) < 0.1f);
    CHECK(std::abs(res[0].cy - 84.0f) < 0.1f);
    CHECK(std::abs(res[0].angle_rad - 0.25f * 3.14159265f) < 1e-3f);
}

static void test_decode_layout_a_edge_cases() {
    const int num_channels = 7; // 4 + 2 classes + 1 angle
    const int num_boxes = 5;
    std::vector<float> data(num_channels * num_boxes, 0.0f);

    auto set_box = [&](int i, float cx, float cy, float w, float h, float s0, float s1, float a) {
        data[0 * num_boxes + i] = cx;
        data[1 * num_boxes + i] = cy;
        data[2 * num_boxes + i] = w;
        data[3 * num_boxes + i] = h;
        data[4 * num_boxes + i] = s0;
        data[5 * num_boxes + i] = s1;
        data[6 * num_boxes + i] = a;
    };

    // Box 0: w = 0 (zero width) -> drop
    set_box(0, 100.0f, 100.0f, 0.0f, 20.0f, 0.1f, 0.9f, 0.0f);
    // Box 1: h = -10 (negative height) -> drop
    set_box(1, 100.0f, 100.0f, 20.0f, -10.0f, 0.1f, 0.9f, 0.0f);
    // Box 2: cx = NaN -> drop
    set_box(2, std::numeric_limits<float>::quiet_NaN(), 100.0f, 20.0f, 20.0f, 0.1f, 0.9f, 0.0f);
    // Box 3: angle = Inf -> drop
    set_box(3, 100.0f, 100.0f, 20.0f, 20.0f, 0.1f, 0.9f, std::numeric_limits<float>::infinity());
    // Box 4: valid box
    set_box(4, 200.0f, 200.0f, 50.0f, 30.0f, 0.1f, 0.88f, 0.45f);

    auto res = decode_obb_layout_a(data.data(), num_channels, num_boxes, 0.5f, true);
    CHECK(res.size() == 1);
    CHECK(res[0].class_id == 1);
    CHECK(std::abs(res[0].cx - 200.0f) < 1e-4f);
    CHECK(std::abs(res[0].cy - 200.0f) < 1e-4f);
    CHECK(std::abs(res[0].w - 50.0f) < 1e-4f);
    CHECK(std::abs(res[0].h - 30.0f) < 1e-4f);
    CHECK(std::abs(res[0].angle_rad - 0.45f) < 1e-4f);
}

static void test_decode_layout_b_int8() {
    // Test INT8 quantization with non-trivial scale and non-zero zero_point
    const float scale = 0.05f;
    const int32_t zp = 10;
    auto q = [scale, zp](float v) -> int8_t {
        double val = std::round(static_cast<double>(v) / scale) + zp;
        if (val < -128.0) val = -128.0;
        if (val > 127.0) val = 127.0;
        return static_cast<int8_t>(val);
    };

    // Stride 8 (80x80)
    std::vector<int8_t> box0(64 * 80 * 80, q(-10.0f));
    std::vector<int8_t> cls0(1 * 80 * 80, q(-10.0f));
    std::vector<int8_t> ang0(1 * 80 * 80, q(0.0f));

    const int cell = 15 * 80 + 25; // gy = 15, gx = 25
    cls0[cell] = q(4.0f); // sigmoid(4) ~ 0.982
    ang0[cell] = q(0.0f); // sigmoid(0) = 0.5 -> (0.5 - 0.25) * pi = 0.25 * pi

    // DFL bin 3 for all 4 coordinates
    for (int b = 0; b < 4; ++b) {
        int ch = b * 16 + 3;
        box0[ch * (80 * 80) + cell] = q(5.0f);
    }

    std::vector<int8_t> dummy40_box(64 * 40 * 40, q(-10.0f));
    std::vector<int8_t> dummy40_cls(1 * 40 * 40, q(-10.0f));
    std::vector<int8_t> dummy40_ang(1 * 40 * 40, q(0.0f));

    std::vector<int8_t> dummy20_box(64 * 20 * 20, q(-10.0f));
    std::vector<int8_t> dummy20_cls(1 * 20 * 20, q(-10.0f));
    std::vector<int8_t> dummy20_ang(1 * 20 * 20, q(0.0f));

    std::array<ObbTensorView, 9> views{};
    auto set_view = [&](size_t idx, const int8_t* ptr, std::array<int32_t, 4> dims) {
        views[idx].data = ptr;
        views[idx].is_int8 = true;
        views[idx].scale = scale;
        views[idx].zero_point = zp;
        views[idx].dims = dims;
        views[idx].num_dims = 4;
    };

    set_view(0, box0.data(), {1, 64, 80, 80});
    set_view(1, cls0.data(), {1, 1, 80, 80});
    set_view(2, ang0.data(), {1, 1, 80, 80});

    set_view(3, dummy40_box.data(), {1, 64, 40, 40});
    set_view(4, dummy40_cls.data(), {1, 1, 40, 40});
    set_view(5, dummy40_ang.data(), {1, 1, 40, 40});

    set_view(6, dummy20_box.data(), {1, 64, 20, 20});
    set_view(7, dummy20_cls.data(), {1, 1, 20, 20});
    set_view(8, dummy20_ang.data(), {1, 1, 20, 20});

    auto res = decode_obb_layout_b(views.data(), 9, 640.0f, 640.0f, 0.5f);
    CHECK(res.size() == 1);
    CHECK(res[0].score > 0.90f);
    // Stride is 8.0f. DFL bin 3 -> dist = 3 * 8 = 24.0f
    // w = 48.0f, h = 48.0f
    CHECK(std::abs(res[0].w - 48.0f) < 0.2f);
    CHECK(std::abs(res[0].h - 48.0f) < 0.2f);
    CHECK(std::abs(res[0].cx - (25.5f * 8.0f)) < 0.2f);
    CHECK(std::abs(res[0].cy - (15.5f * 8.0f)) < 0.2f);
    CHECK(std::abs(res[0].angle_rad - 0.25f * 3.14159265f) < 0.05f);
}

static void test_decode_layout_b_grouped_and_nhwc() {
    // Test Grouped order: box at 0, 1, 2; cls at 3, 4, 5; ang at 6, 7, 8
    // Test NHWC layout: [1, H, W, C]
    // Test nc == 64 (class count exactly equals 64, matching box channel count!)
    // Test asymmetric lt != rb verifying dist2rbox rotated (cx, cy) offset
    const int nc = 64;

    // Scale 0: 80x80
    std::vector<float> box0(80 * 80 * 64, 0.0f);
    std::vector<float> cls0(80 * 80 * nc, -10.0f);
    std::vector<float> ang0(80 * 80 * 1, 0.0f);

    const int gy = 5, gx = 5;
    const size_t cell_offset = (static_cast<size_t>(gy) * 80 + gx);

    // Target class: 42
    cls0[cell_offset * nc + 42] = 5.0f; // sigmoid(5) ~ 0.993
    // Angle raw = 0.0 -> a = (0.5 - 0.25) * pi = 0.25 * pi (45 deg)
    ang0[cell_offset * 1 + 0] = 0.0f;

    // Asymmetric DFL:
    // Stride 8:
    // lt_x = 1 * 8 = 8.0f (bin 1)
    // lt_y = 2 * 8 = 16.0f (bin 2)
    // rb_x = 3 * 8 = 24.0f (bin 3)
    // rb_y = 4 * 8 = 32.0f (bin 4)
    box0[cell_offset * 64 + (0 * 16 + 1)] = 10.0f; // lt_x = bin 1
    box0[cell_offset * 64 + (1 * 16 + 2)] = 10.0f; // lt_y = bin 2
    box0[cell_offset * 64 + (2 * 16 + 3)] = 10.0f; // rb_x = bin 3
    box0[cell_offset * 64 + (3 * 16 + 4)] = 10.0f; // rb_y = bin 4

    std::vector<float> dummy40_box(40 * 40 * 64, 0.0f);
    std::vector<float> dummy40_cls(40 * 40 * nc, -10.0f);
    std::vector<float> dummy40_ang(40 * 40 * 1, 0.0f);

    std::vector<float> dummy20_box(20 * 20 * 64, 0.0f);
    std::vector<float> dummy20_cls(20 * 20 * nc, -10.0f);
    std::vector<float> dummy20_ang(20 * 20 * 1, 0.0f);

    std::array<ObbTensorView, 9> views{};
    auto set_nhwc = [&](size_t idx, const float* ptr, int h, int w, int c) {
        views[idx].data = ptr;
        views[idx].is_int8 = false;
        views[idx].dims = {1, h, w, c};
        views[idx].num_dims = 4;
    };

    // Grouped: box at 0, 1, 2
    set_nhwc(0, box0.data(), 80, 80, 64);
    set_nhwc(1, dummy40_box.data(), 40, 40, 64);
    set_nhwc(2, dummy20_box.data(), 20, 20, 64);

    // Grouped: cls at 3, 4, 5
    set_nhwc(3, cls0.data(), 80, 80, nc);
    set_nhwc(4, dummy40_cls.data(), 40, 40, nc);
    set_nhwc(5, dummy20_cls.data(), 20, 20, nc);

    // Grouped: ang at 6, 7, 8
    set_nhwc(6, ang0.data(), 80, 80, 1);
    set_nhwc(7, dummy40_ang.data(), 40, 40, 1);
    set_nhwc(8, dummy20_ang.data(), 20, 20, 1);

    auto res = decode_obb_layout_b(views.data(), 9, 640.0f, 640.0f, 0.5f);
    CHECK(res.size() == 1);
    CHECK(res[0].class_id == 42);
    CHECK(res[0].score > 0.9f);

    // Geometry check for asymmetric lt != rb:
    // lt_x = 8.0, lt_y = 16.0, rb_x = 24.0, rb_y = 32.0
    // w = lt_x + rb_x = 32.0f
    // h = lt_y + rb_y = 48.0f
    // xf = (24.0 - 8.0) * 0.5 = 8.0f
    // yf = (32.0 - 16.0) * 0.5 = 8.0f
    // a = 0.25 * pi (cos a = sin a = sqrt(2)/2)
    // dx = xf * cos(a) - yf * sin(a) = 0.0f
    // dy = xf * sin(a) + yf * cos(a) = 16.0 * (sqrt(2)/2) = 8.0 * sqrt(2) ~ 11.3137f
    // base_cx = (5 + 0.5) * 8 = 44.0f -> cx = 44.0f
    // base_cy = (5 + 0.5) * 8 = 44.0f -> cy = 44.0f + 11.3137f = 55.3137f
    CHECK(std::abs(res[0].w - 32.0f) < 0.2f);
    CHECK(std::abs(res[0].h - 48.0f) < 0.2f);
    CHECK(std::abs(res[0].cx - 44.0f) < 0.2f);
    CHECK(std::abs(res[0].cy - 55.3137f) < 0.2f);
}

static void test_shape_validators() {
    // 1. Layout A validator tests
    ObbTensorSpec in_spec;
    in_spec.dtype = ObbDataType::FP32;
    in_spec.num_dims = 4;
    in_spec.dims = {1, 3, 640, 640};

    // Valid Layout A: [1, 20, 8400]
    ObbTensorSpec out_valid;
    out_valid.dtype = ObbDataType::FP32;
    out_valid.num_dims = 3;
    out_valid.dims = {1, 20, 8400};

    CHECK(validate_obb_layout_a(in_spec, out_valid, true, "yolo11n-obb.cvimodel"));
    // Reject if allow_auto_1out is false
    CHECK(!validate_obb_layout_a(in_spec, out_valid, false, "yolo11n-obb.cvimodel"));
    // Reject if filename contains "pose"
    CHECK(!validate_obb_layout_a(in_spec, out_valid, true, "yolo11n-pose.cvimodel"));
    CHECK(!validate_obb_layout_a(in_spec, out_valid, true, "/path/to/YOLO_POSE_MODEL.cvimodel"));
    // Reject if data type is INT8
    ObbTensorSpec out_int8 = out_valid;
    out_int8.dtype = ObbDataType::INT8;
    CHECK(!validate_obb_layout_a(in_spec, out_int8, true, "yolo11n-obb.cvimodel"));
    // Reject YoloV5 output shape [1, 25200, 85] (25200 != 8400)
    ObbTensorSpec out_yolov5;
    out_yolov5.dtype = ObbDataType::FP32;
    out_yolov5.num_dims = 3;
    out_yolov5.dims = {1, 85, 25200};
    CHECK(!validate_obb_layout_a(in_spec, out_yolov5, true, "model.cvimodel"));

    // 2. Layout B validator / resolver tests
    auto make_spec = [](int c, int h, int w, bool nchw, ObbDataType dt) -> ObbTensorSpec {
        ObbTensorSpec s;
        s.dtype = dt;
        s.num_dims = 4;
        if (nchw) s.dims = {1, c, h, w};
        else s.dims = {1, h, w, c};
        return s;
    };

    // Valid 9-output Interleaved NCHW with nc = 64
    std::array<ObbTensorSpec, 9> specs_valid_interleaved{};
    specs_valid_interleaved[0] = make_spec(64, 80, 80, true, ObbDataType::INT8);
    specs_valid_interleaved[1] = make_spec(64, 80, 80, true, ObbDataType::INT8);
    specs_valid_interleaved[2] = make_spec(1,  80, 80, true, ObbDataType::INT8);

    specs_valid_interleaved[3] = make_spec(64, 40, 40, true, ObbDataType::INT8);
    specs_valid_interleaved[4] = make_spec(64, 40, 40, true, ObbDataType::INT8);
    specs_valid_interleaved[5] = make_spec(1,  40, 40, true, ObbDataType::INT8);

    specs_valid_interleaved[6] = make_spec(64, 20, 20, true, ObbDataType::INT8);
    specs_valid_interleaved[7] = make_spec(64, 20, 20, true, ObbDataType::INT8);
    specs_valid_interleaved[8] = make_spec(1,  20, 20, true, ObbDataType::INT8);

    auto info_inter = resolve_obb_layout_b(specs_valid_interleaved.data(), 9);
    CHECK(info_inter.valid);
    CHECK(info_inter.is_interleaved);
    CHECK(info_inter.is_nchw);
    CHECK(info_inter.num_classes == 64);

    // Reject 9-output Pose model (ang_c == 51)
    auto specs_pose = specs_valid_interleaved;
    specs_pose[2] = make_spec(51, 80, 80, true, ObbDataType::INT8);
    specs_pose[5] = make_spec(51, 40, 40, true, ObbDataType::INT8);
    specs_pose[8] = make_spec(51, 20, 20, true, ObbDataType::INT8);
    CHECK(!resolve_obb_layout_b(specs_pose.data(), 9).valid);

    // Reject non-matching (H, W) between box and cls
    auto specs_mismatch = specs_valid_interleaved;
    specs_mismatch[1] = make_spec(64, 70, 70, true, ObbDataType::INT8); // 70 != 80
    CHECK(!resolve_obb_layout_b(specs_mismatch.data(), 9).valid);

    // Reject invalid box_c != 64
    auto specs_bad_box = specs_valid_interleaved;
    specs_bad_box[0] = make_spec(32, 80, 80, true, ObbDataType::INT8);
    CHECK(!resolve_obb_layout_b(specs_bad_box.data(), 9).valid);

    // Valid 9-output Grouped NHWC with nc = 1
    std::array<ObbTensorSpec, 9> specs_valid_grouped_nhwc{};
    specs_valid_grouped_nhwc[0] = make_spec(64, 80, 80, false, ObbDataType::FP32);
    specs_valid_grouped_nhwc[1] = make_spec(64, 40, 40, false, ObbDataType::FP32);
    specs_valid_grouped_nhwc[2] = make_spec(64, 20, 20, false, ObbDataType::FP32);

    specs_valid_grouped_nhwc[3] = make_spec(1, 80, 80, false, ObbDataType::FP32);
    specs_valid_grouped_nhwc[4] = make_spec(1, 40, 40, false, ObbDataType::FP32);
    specs_valid_grouped_nhwc[5] = make_spec(1, 20, 20, false, ObbDataType::FP32);

    specs_valid_grouped_nhwc[6] = make_spec(1, 80, 80, false, ObbDataType::FP32);
    specs_valid_grouped_nhwc[7] = make_spec(1, 40, 40, false, ObbDataType::FP32);
    specs_valid_grouped_nhwc[8] = make_spec(1, 20, 20, false, ObbDataType::FP32);

    auto info_grp = resolve_obb_layout_b(specs_valid_grouped_nhwc.data(), 9);
    CHECK(info_grp.valid);
    CHECK(!info_grp.is_interleaved);
    CHECK(!info_grp.is_nchw);
    CHECK(info_grp.num_classes == 1);

    // 512x512 input where scale 0 grid is 64x64
    std::array<ObbTensorSpec, 9> specs_512{};
    specs_512[0] = make_spec(64, 64, 64, true, ObbDataType::FP32);
    specs_512[1] = make_spec(10, 64, 64, true, ObbDataType::FP32);
    specs_512[2] = make_spec(1,  64, 64, true, ObbDataType::FP32);

    specs_512[3] = make_spec(64, 32, 32, true, ObbDataType::FP32);
    specs_512[4] = make_spec(10, 32, 32, true, ObbDataType::FP32);
    specs_512[5] = make_spec(1,  32, 32, true, ObbDataType::FP32);

    specs_512[6] = make_spec(64, 16, 16, true, ObbDataType::FP32);
    specs_512[7] = make_spec(10, 16, 16, true, ObbDataType::FP32);
    specs_512[8] = make_spec(1,  16, 16, true, ObbDataType::FP32);

    auto info_512 = resolve_obb_layout_b(specs_512.data(), 9);
    CHECK(info_512.valid);
    CHECK(info_512.is_nchw);
    CHECK(info_512.is_interleaved);
    CHECK(info_512.num_classes == 10);
}

static void test_decode_layout_a_boxes_first() {
    // Input spec [1, 3, 640, 640] -> expected ibox_len = 8400
    ObbTensorSpec in_spec;
    in_spec.dtype = ObbDataType::FP32;
    in_spec.num_dims = 4;
    in_spec.dims = {1, 3, 640, 640};

    // Output spec in boxes-first format: [1, 8400, 7]
    ObbTensorSpec out_spec;
    out_spec.dtype = ObbDataType::FP32;
    out_spec.num_dims = 3;
    out_spec.dims = {1, 8400, 7};

    auto info = resolve_obb_layout_a(in_spec, out_spec, true, "yolo11n-obb.cvimodel");
    CHECK(info.valid);
    CHECK(!info.channels_first);
    CHECK(info.num_channels == 7);
    CHECK(info.num_boxes == 8400);

    // Also test 2D boxes-first: [8400, 7]
    ObbTensorSpec out_spec2d = out_spec;
    out_spec2d.num_dims = 2;
    out_spec2d.dims = {8400, 7, 0, 0};
    auto info2d = resolve_obb_layout_a(in_spec, out_spec2d, true, "yolo11n-obb.cvimodel");
    CHECK(info2d.valid);
    CHECK(!info2d.channels_first);
    CHECK(info2d.num_channels == 7);
    CHECK(info2d.num_boxes == 8400);

    // Construct a buffer for decoding with channels_first = false
    std::vector<float> data(8400 * 7, 0.0f);
    // Put a detection at box index 42
    const int b = 42;
    data[b * 7 + 0] = 120.0f; // cx
    data[b * 7 + 1] = 150.0f; // cy
    data[b * 7 + 2] = 40.0f;  // w
    data[b * 7 + 3] = 25.0f;  // h
    data[b * 7 + 4] = 0.1f;   // cls0
    data[b * 7 + 5] = 0.92f;  // cls1
    data[b * 7 + 6] = 0.35f;  // angle

    auto res = decode_obb_layout_a(data.data(), info.num_channels, info.num_boxes, 0.5f, info.channels_first);
    CHECK(res.size() == 1);
    CHECK(res[0].class_id == 1);
    CHECK(std::abs(res[0].score - 0.92f) < 1e-4f);
    CHECK(std::abs(res[0].cx - 120.0f) < 1e-4f);
    CHECK(std::abs(res[0].cy - 150.0f) < 1e-4f);
    CHECK(std::abs(res[0].w - 40.0f) < 1e-4f);
    CHECK(std::abs(res[0].h - 25.0f) < 1e-4f);
    CHECK(std::abs(res[0].angle_rad - 0.35f) < 1e-4f);
}

static void test_decode_layout_b_null_data() {
    std::array<ObbTensorView, 9> views{};
    auto res = decode_obb_layout_b(views.data(), 9, 640.0f, 640.0f, 0.5f);
    CHECK(res.empty());
}

int main() {
    test_prob_iou();
    test_obb_nms();
    test_obb_to_geom();
    test_decode_layout_a();
    test_decode_layout_a_edge_cases();
    test_decode_layout_a_boxes_first();
    test_decode_layout_b();
    test_decode_layout_b_int8();
    test_decode_layout_b_grouped_and_nhwc();
    test_decode_layout_b_null_data();
    test_shape_validators();

    if (g_fail == 0) {
        std::printf("All obb_geom tests passed!\n");
        return 0;
    } else {
        std::printf("%d obb_geom tests failed!\n", g_fail);
        return 1;
    }
}
