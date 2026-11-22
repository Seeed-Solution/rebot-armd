// Host tests of the 10-output YOLO11-seg decoder (main/seg_heads.*).
//
//   seg_heads_test [TENSOR_DIR]
//
// TENSOR_DIR: the device output tensors of yolo11n_seg_384x640_cv181x_int8_fuse.cvimodel on
// real_0 (real_0_dev_{box,cls,mask}_stride_{8,16,32}.bin, real_0_dev_proto.bin; FP32 NCHW, 5.5 MB,
// not in git). CMake passes SEG384_TENSOR_DIR (-D or env); without it the real-tensor checks are
// skipped and the synthetic ones still run.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "mask_geom.h"
#include "obb_geom.h"
#include "seg_heads.h"

using namespace rv;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

static ObbTensorSpec spec(int c, int h, int w, ObbDataType t = ObbDataType::FP32) {
    ObbTensorSpec s;
    s.dtype = t;
    s.num_dims = 4;
    s.dims = {1, c, h, w};
    return s;
}

// output order of the cvimodel (model_tool --info, outputs 129..138)
static std::vector<ObbTensorSpec> seg384_specs() {
    return {spec(80, 12, 20), spec(32, 12, 20), spec(64, 12, 20), spec(80, 24, 40), spec(32, 24, 40),
            spec(64, 24, 40), spec(80, 48, 80), spec(32, 96, 160), spec(32, 48, 80), spec(64, 48, 80)};
}

static void test_layout() {
    const auto s = seg384_specs();
    const SegHeadsInfo info = resolve_seg_heads(s.data(), s.size(), 640, 384);
    CHECK(info.valid);
    CHECK(info.num_classes == 80);
    CHECK(info.proto == 7 && info.proto_w == 160 && info.proto_h == 96);
    CHECK(info.box[0] == 9 && info.cls[0] == 6 && info.coef[0] == 8);
    CHECK(info.box[1] == 5 && info.cls[1] == 3 && info.coef[1] == 4);
    CHECK(info.box[2] == 2 && info.cls[2] == 0 && info.coef[2] == 1);
    CHECK(info.stride[0] == 8 && info.grid_w[0] == 80 && info.grid_h[0] == 48);
    CHECK(info.stride[2] == 32 && info.grid_w[2] == 20 && info.grid_h[2] == 12);

    // 640x640 variant, other order
    std::vector<ObbTensorSpec> sq = {spec(32, 160, 160), spec(64, 80, 80), spec(3, 80, 80), spec(32, 80, 80),
                                     spec(64, 40, 40), spec(3, 40, 40), spec(32, 40, 40), spec(64, 20, 20),
                                     spec(3, 20, 20), spec(32, 20, 20)};
    const SegHeadsInfo sqi = resolve_seg_heads(sq.data(), sq.size(), 640, 640);
    CHECK(sqi.valid && sqi.num_classes == 3 && sqi.proto == 0);

    // the other layouts are not taken as segment heads
    auto bad = s;
    bad[0].dtype = ObbDataType::INT8;
    CHECK(!resolve_seg_heads(bad.data(), bad.size(), 640, 384).valid);
    CHECK(!resolve_seg_heads(s.data(), 9, 640, 384).valid);
    CHECK(!resolve_seg_heads(s.data(), s.size(), 640, 640).valid);  // grids do not match the input
    bad = s;
    bad[0] = spec(32, 12, 20);  // 32 classes: class and coefficient tensors are ambiguous
    CHECK(!resolve_seg_heads(bad.data(), bad.size(), 640, 384).valid);
    // official 2-output seg [1,116,8400,1] + [1,32,160,160]
    const std::vector<ObbTensorSpec> off = {spec(116, 8400, 1), spec(32, 160, 160)};
    CHECK(!resolve_seg_heads(off.data(), off.size(), 640, 640).valid);
    // OBB Layout B (9 outputs: box 64, cls 15, angle 1 per scale) stays OBB; the seg heads are not OBB
    std::vector<ObbTensorSpec> obb;
    for (int g : {80, 40, 20}) {
        obb.push_back(spec(64, g, g));
        obb.push_back(spec(15, g, g));
        obb.push_back(spec(1, g, g));
    }
    CHECK(resolve_obb_layout_b(obb.data(), obb.size()).valid);
    CHECK(!resolve_seg_heads(obb.data(), obb.size(), 640, 640).valid);
    CHECK(!resolve_obb_layout_b(s.data(), s.size()).valid);
    CHECK(!resolve_obb_layout_b(s.data(), 9).valid);
}

static void test_dfl_logit() {
    float bins[16] = {};
    bins[5] = 50.0f;
    CHECK(std::fabs(dfl16(bins, 1) - 5.0f) < 1e-4f);
    float flat[16] = {};
    CHECK(std::fabs(dfl16(flat, 1) - 7.5f) < 1e-4f);
    CHECK(std::fabs(seg_logit(0.5f)) < 1e-6f);
    CHECK(std::fabs(seg_logit(0.25f) - std::log(1.0f / 3.0f)) < 1e-6f);
}

static bool same(const std::vector<SegCandidate>& a, const std::vector<SegCandidate>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].class_id != b[i].class_id || a[i].score != b[i].score || a[i].x0 != b[i].x0 || a[i].y0 != b[i].y0 ||
            a[i].x1 != b[i].x1 || a[i].y1 != b[i].y1 || a[i].coef != b[i].coef) {
            return false;
        }
    }
    return true;
}

struct Tensors {
    SegHeadsInfo info;
    std::vector<std::vector<float>> data;  // by output index
    std::vector<const float*> ptrs;
};

static Tensors synthetic(unsigned seed) {
    Tensors t;
    const auto s = seg384_specs();
    t.info = resolve_seg_heads(s.data(), s.size(), 640, 384);
    std::mt19937 rng(seed);
    std::normal_distribution<float> n(-6.0f, 2.0f);  // class logits mostly low, some cells above 0.25
    std::normal_distribution<float> u(0.0f, 2.0f);
    t.data.resize(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        t.data[i].resize(static_cast<size_t>(s[i].dims[1]) * s[i].dims[2] * s[i].dims[3]);
        const bool cls = s[i].dims[1] == 80;
        for (float& v : t.data[i]) v = cls ? n(rng) : u(rng);
    }
    for (auto& d : t.data) t.ptrs.push_back(d.data());
    return t;
}

static void test_prefilter_synthetic() {
    for (unsigned seed : {1u, 2u, 3u}) {
        const Tensors t = synthetic(seed);
        for (float conf : {0.1f, 0.25f, 0.5f}) {
            const auto a = decode_seg_heads(t.info, t.ptrs.data(), conf, true);
            const auto b = decode_seg_heads(t.info, t.ptrs.data(), conf, false);
            CHECK(!a.empty());
            CHECK(same(a, b));
            CHECK(same(seg_nms(a, 0.45f), seg_nms(b, 0.45f)));
        }
    }
}

static void test_letterbox() {
    // the 640x640 camera frame: 16:9 content 640x360 at y=140 -> rows 12..371 of 640x384
    const LetterboxGeom g = letterbox_geom(0, 140, 640, 360, 640, 384);
    CHECK(g.scale == 1.0f && g.pad_x == 0 && g.pad_y == 12 && g.dst_w == 640 && g.dst_h == 360);
    // Ultralytics LetterBox for a 1920x1080 image: 640x360, 12 rows top and bottom
    const LetterboxGeom g2 = letterbox_geom(0, 0, 1920, 1080, 640, 384);
    CHECK(std::fabs(g2.scale - 1.0f / 3.0f) < 1e-6f && g2.pad_y == 12 && g2.dst_h == 360);

    const int fw = 640, fh = 640, stride = fw * 3 + 8;
    std::vector<uint8_t> frame(static_cast<size_t>(stride) * fh);
    for (int y = 0; y < fh; ++y)
        for (int x = 0; x < fw; ++x)
            for (int c = 0; c < 3; ++c) frame[static_cast<size_t>(y) * stride + x * 3 + c] = static_cast<uint8_t>(y + x + c);
    std::vector<uint8_t> in(640 * 384 * 3, 0);
    letterbox_rgb(frame.data(), fw, fh, stride, g, in.data(), 640, 384);
    CHECK(in[0] == 114 && in[(11 * 640 + 639) * 3 + 2] == 114 && in[(372 * 640) * 3] == 114 &&
          in[(383 * 640 + 639) * 3 + 2] == 114);
    bool rows_ok = true;
    for (int y = 12; y < 372; ++y)
        rows_ok = rows_ok && std::memcmp(&in[static_cast<size_t>(y) * 640 * 3],
                                         &frame[static_cast<size_t>(y - 12 + 140) * stride], 640 * 3) == 0;
    CHECK(rows_ok);
    // nearest-neighbour path (content 1280x720 of a larger frame, scale 0.5)
    const int bw = 1280, bh = 720, bs = bw * 3;
    std::vector<uint8_t> big(static_cast<size_t>(bs) * bh);
    for (int y = 0; y < bh; ++y)
        for (int x = 0; x < bw; ++x) big[static_cast<size_t>(y) * bs + x * 3] = static_cast<uint8_t>(x / 2);
    const LetterboxGeom g3 = letterbox_geom(0, 0, bw, bh, 640, 384);
    letterbox_rgb(big.data(), bw, bh, bs, g3, in.data(), 640, 384);
    CHECK(g3.pad_y == 12 && in[(100 * 640 + 300) * 3] == static_cast<uint8_t>(300));

    // box mapping: input pixels -> frame-normalised, clipped to the content band
    const auto s = seg384_specs();
    const SegHeadsInfo info = resolve_seg_heads(s.data(), s.size(), 640, 384);
    SegCandidate c;
    c.x0 = 10;
    c.y0 = 2;  // above the content band (row 12): clipped
    c.x1 = 170;
    c.y1 = 112;
    const SegBoxOut o = seg_box_to_frame(c, info, g, 640, 640);
    CHECK(std::fabs(o.x - 90.0f / 640) < 1e-6f && std::fabs(o.w - 160.0f / 640) < 1e-6f);
    CHECK(std::fabs(o.y - (140 + 50.0f) / 640) < 1e-6f && std::fabs(o.h - 100.0f / 640) < 1e-6f);
    CHECK(o.crop_x == 2 && o.crop_y == 3 && o.crop_w == 41 && o.crop_h == 25);
}

static bool load(const std::string& path, std::vector<float>& out, size_t count) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.resize(count);
    f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(count * sizeof(float)));
    return static_cast<size_t>(f.gcount()) == count * sizeof(float);
}

static float iou(const SegCandidate& a, const float* b) {
    const float iw = std::fmin(a.x1, b[2]) - std::fmax(a.x0, b[0]);
    const float ih = std::fmin(a.y1, b[3]) - std::fmax(a.y0, b[1]);
    if (iw <= 0 || ih <= 0) return 0;
    const float i = iw * ih;
    return i / ((a.x1 - a.x0) * (a.y1 - a.y0) + (b[2] - b[0]) * (b[3] - b[1]) - i);
}

static void test_real(const std::string& dir) {
    Tensors t;
    const auto s = seg384_specs();
    t.info = resolve_seg_heads(s.data(), s.size(), 640, 384);
    const char* names[10] = {"cls_stride_32", "mask_stride_32", "box_stride_32", "cls_stride_16", "mask_stride_16",
                             "box_stride_16", "cls_stride_8",  "proto",          "mask_stride_8", "box_stride_8"};
    t.data.resize(10);
    for (size_t i = 0; i < 10; ++i) {
        const size_t n = static_cast<size_t>(s[i].dims[1]) * s[i].dims[2] * s[i].dims[3];
        if (!load(dir + "/real_0_dev_" + names[i] + ".bin", t.data[i], n)) {
            std::printf("FAIL cannot read %s/real_0_dev_%s.bin\n", dir.c_str(), names[i]);
            ++g_fail;
            return;
        }
        t.ptrs.push_back(t.data[i].data());
    }

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<float> scratch;
    std::vector<SegCandidate> pre;
    for (int r = 0; r < 20; ++r) pre = decode_seg_heads(t.info, t.ptrs.data(), 0.25f, true, &scratch);
    const auto t1 = std::chrono::steady_clock::now();
    std::vector<SegCandidate> full;
    for (int r = 0; r < 20; ++r) full = decode_seg_heads(t.info, t.ptrs.data(), 0.25f, false);
    const auto t2 = std::chrono::steady_clock::now();
    std::printf("real_0: %zu candidates >= 0.25; host decode prefilter %.3f ms, full %.3f ms (mean of 20)\n",
                pre.size(), std::chrono::duration<double, std::milli>(t1 - t0).count() / 20,
                std::chrono::duration<double, std::milli>(t2 - t1).count() / 20);
    CHECK(same(pre, full));
    for (float conf : {0.1f, 0.4f, 0.5f}) {
        CHECK(same(decode_seg_heads(t.info, t.ptrs.data(), conf, true),
                   decode_seg_heads(t.info, t.ptrs.data(), conf, false)));
    }

    const auto kept = seg_nms(pre, 0.45f);
    const LetterboxGeom g = letterbox_geom(0, 140, 640, 360, 640, 384);
    for (const auto& k : kept) {
        const SegBoxOut o = seg_box_to_frame(k, t.info, g, 640, 640);
        std::vector<uint8_t> m(static_cast<size_t>(o.crop_w) * o.crop_h);
        decode_crop_mask(t.data[t.info.proto].data(), 160, 96, 32, k.coef.data(), o.crop_x, o.crop_y, o.crop_w,
                         o.crop_h, m.data());
        int px = 0;
        for (uint8_t v : m) px += v;
        const auto cs = mask_cross_section(MaskView{m.data(), o.crop_w, o.crop_h, o.crop_w, 4.0f, 4.0f});
        std::printf("  class %d score %.4f input box [%.1f %.1f %.1f %.1f] frame-norm c(%.4f %.4f) wh(%.4f %.4f) "
                    "mask %d/%d px angle %.2f conf %.3f width %.1f\n",
                    k.class_id, k.score, k.x0, k.y0, k.x1, k.y1, o.x, o.y, o.w, o.h, px, o.crop_w * o.crop_h,
                    cs.angle_deg, cs.conf, cs.width_px);
    }
    // the objects of the official 640x640 model on this image: cup and keyboard
    const float cup[4] = {10.5f, 119.8f, 167.8f, 285.4f};
    const float keyboard[4] = {330.9f, 203.3f, 638.8f, 369.3f};
    // the clear jar (image x 640..1040, y 516..900 of 1920x1080); a model output: the FP32 ONNX
    // export gives cup 0.43 on the same cell (stride 16, row 15, col 18), see REPORT
    const float jar[4] = {213.6f, 184.0f, 347.0f, 312.2f};
    int n_cup = 0, n_kb = 0, n_jar = 0;
    for (const auto& k : kept) {
        if (k.class_id == 41 && iou(k, cup) > 0.95f && k.score > 0.75f) ++n_cup;
        if (k.class_id == 66 && iou(k, keyboard) > 0.95f && k.score > 0.5f) ++n_kb;
        if (k.class_id == 41 && iou(k, jar) > 0.95f) ++n_jar;
    }
    CHECK(kept.size() == 3);
    CHECK(n_cup == 1 && n_kb == 1 && n_jar == 1);
    CHECK(kept.size() == 3 && kept[0].class_id == 41 && kept[0].score > kept[1].score);
    // at conf 0.45 the jar (0.40) is gone and the official objects stay
    const auto k45 = seg_nms(decode_seg_heads(t.info, t.ptrs.data(), 0.45f, true), 0.45f);
    CHECK(k45.size() == 2 && k45[0].class_id == 41 && k45[1].class_id == 66);
}

// seg384: a box's mask crop maps through seg_cells_to_frame like its box through seg_box_to_frame
static void test_seg384_contour_mapping() {
    const LetterboxGeom g = letterbox_geom(0, 140, 640, 360, 640, 384);
    SegHeadsInfo info;
    info.in_w = 640;
    info.in_h = 384;
    info.proto_w = 160;
    info.proto_h = 96;
    SegCandidate c;
    c.x0 = 100;
    c.y0 = 52;
    c.x1 = 200;
    c.y1 = 112;
    const SegBoxOut b = seg_box_to_frame(c, info, g, 640, 640);
    Contour corners = {{0, 0}, {static_cast<float>(b.crop_w), 0}, {static_cast<float>(b.crop_w), static_cast<float>(b.crop_h)}};
    contour_to_frame_norm(corners, seg_cells_to_frame(info, g, b, 640, 640));
    // crop cells are whole cells around the box: x 100..200 -> cells 25..50, y 52..112 -> 13..28
    CHECK(std::abs(corners[0][0] - 100.0f / 640) < 1e-6 && std::abs(corners[0][1] - (52.0f - 12 + 140) / 640) < 1e-6);
    CHECK(std::abs(corners[2][0] - 200.0f / 640) < 1e-6 && std::abs(corners[2][1] - (112.0f - 12 + 140) / 640) < 1e-6);
    // the box centre (frame-normalised) lies inside the mapped crop
    CHECK(b.y > corners[0][1] && b.y < corners[2][1]);
}

int main(int argc, char** argv) {
    test_seg384_contour_mapping();
    test_layout();
    test_dfl_logit();
    test_prefilter_synthetic();
    test_letterbox();
    if (argc > 1 && argv[1][0] != '\0') {
        test_real(argv[1]);
    } else {
        std::printf("SKIP real-tensor checks (no SEG384_TENSOR_DIR)\n");
    }
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("seg_heads_test: all checks passed\n");
    return 0;
}
