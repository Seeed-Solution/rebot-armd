#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "mask_geom.h"

using namespace rv;

static int g_fail = 0;
#define CHECK(c)                                                             \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);         \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

#define CHECK_MSG(c, msg)                                                   \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::printf("FAIL %s:%d: %s (%s)\n", __FILE__, __LINE__, #c,     \
                        (msg));                                              \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

static void naive_full_decode(
    const float* proto, int proto_w, int proto_h, int num_proto_channels,
    const float* coef,
    uint8_t* out_full_mask
) {
    const size_t channel_stride = static_cast<size_t>(proto_w) * static_cast<size_t>(proto_h);
    for (int y = 0; y < proto_h; ++y) {
        for (int x = 0; x < proto_w; ++x) {
            float sum = 0.0f;
            for (int k = 0; k < num_proto_channels; ++k) {
                sum += coef[k] * proto[static_cast<size_t>(k) * channel_stride + static_cast<size_t>(y) * proto_w + x];
            }
            out_full_mask[y * proto_w + x] = (sum > 0.0f) ? 1 : 0;
        }
    }
}

static void test_random_crops() {
    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> dist(-3.0f, 3.0f);

    struct Config {
        int w;
        int h;
        int channels;
    };
    const Config configs[] = {
        {160, 160, 32},
        {80, 60, 16},
        {40, 40, 4},
        {25, 25, 1},
    };

    for (const auto& cfg : configs) {
        std::vector<float> proto(static_cast<size_t>(cfg.channels * cfg.w * cfg.h));
        for (float& v : proto) v = dist(rng);

        std::vector<float> coef(cfg.channels);
        for (float& v : coef) v = dist(rng);

        std::vector<uint8_t> full_mask(cfg.w * cfg.h);
        naive_full_decode(proto.data(), cfg.w, cfg.h, cfg.channels, coef.data(), full_mask.data());

        struct Crop { int x0, y0, cw, ch; const char* desc; };
        std::vector<Crop> crops = {
            {0, 0, cfg.w, cfg.h, "full"},
            {cfg.w / 4, cfg.h / 4, cfg.w / 2, cfg.h / 2, "center"},
            {0, 0, std::min(15, cfg.w), std::min(12, cfg.h), "top_left"},
            {cfg.w - std::min(20, cfg.w), cfg.h - std::min(18, cfg.h), std::min(20, cfg.w), std::min(18, cfg.h), "bottom_right"},
            {cfg.w / 2, cfg.h / 2, 1, 1, "1x1"},
            {std::min(5, cfg.w - 1), std::min(7, cfg.h - 1), std::min(13, cfg.w - 5), std::min(11, cfg.h - 7), "odd_crop"},
        };

        // Add 5 random crops
        for (int i = 0; i < 5; ++i) {
            int cw = 1 + (rng() % (cfg.w));
            int ch = 1 + (rng() % (cfg.h));
            int x0 = rng() % (cfg.w - cw + 1);
            int y0 = rng() % (cfg.h - ch + 1);
            crops.push_back({x0, y0, cw, ch, "random"});
        }

        for (const auto& crop : crops) {
            std::vector<uint8_t> crop_mask(crop.cw * crop.ch, 0xAA);
            decode_crop_mask(proto.data(), cfg.w, cfg.h, cfg.channels, coef.data(),
                             crop.x0, crop.y0, crop.cw, crop.ch,
                             crop_mask.data(), nullptr);

            for (int r = 0; r < crop.ch; ++r) {
                for (int c = 0; c < crop.cw; ++c) {
                    uint8_t expected = full_mask[(crop.y0 + r) * cfg.w + (crop.x0 + c)];
                    uint8_t actual = crop_mask[r * crop.cw + c];
                    CHECK_MSG(actual == expected, crop.desc);
                }
            }

            // Test with scratch buffer
            std::vector<float> scratch(crop.cw * crop.ch);
            std::vector<uint8_t> crop_mask_scratch(crop.cw * crop.ch, 0x55);
            decode_crop_mask(proto.data(), cfg.w, cfg.h, cfg.channels, coef.data(),
                             crop.x0, crop.y0, crop.cw, crop.ch,
                             crop_mask_scratch.data(), scratch.data());

            CHECK(crop_mask == crop_mask_scratch);
        }
    }
}

static void test_edge_cases() {
    float proto[16] = {0};
    float coef[1] = {1.0f};
    uint8_t out[16] = {0};

    // Zero crop dimensions
    decode_crop_mask(proto, 4, 4, 1, coef, 0, 0, 0, 0, out);
    decode_crop_mask(proto, 4, 4, 1, coef, 0, 0, -1, 4, out);

    // Null proto or coef
    decode_crop_mask(nullptr, 4, 4, 1, coef, 0, 0, 4, 4, out);
    for (int i = 0; i < 16; ++i) CHECK(out[i] == 0);

    decode_crop_mask(proto, 4, 4, 1, nullptr, 0, 0, 4, 4, out);
    for (int i = 0; i < 16; ++i) CHECK(out[i] == 0);

    decode_crop_mask(proto, 4, 4, 0, coef, 0, 0, 4, 4, out);
    for (int i = 0; i < 16; ++i) CHECK(out[i] == 0);

    // All positive vs all negative
    float pos_proto[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    float pos_coef[1] = {2.0f};
    decode_crop_mask(pos_proto, 2, 2, 1, pos_coef, 0, 0, 2, 2, out);
    for (int i = 0; i < 4; ++i) CHECK(out[i] == 1);

    float neg_coef[1] = {-2.0f};
    decode_crop_mask(pos_proto, 2, 2, 1, neg_coef, 0, 0, 2, 2, out);
    for (int i = 0; i < 4; ++i) CHECK(out[i] == 0);
}

static FocusCandidate make_candidate(float cx_cont, float cy_cont, float w_cont, float h_cont,
                                      float score, int class_id, const std::string& label) {
    float cx_px = cx_cont * 640.0f;
    float cy_px = 140.0f + cy_cont * 360.0f;
    float w_px = w_cont * 640.0f;
    float h_px = h_cont * 360.0f;
    FocusCandidate c;
    c.x = cx_px / 640.0f;
    c.y = cy_px / 640.0f;
    c.w = w_px / 640.0f;
    c.h = h_px / 640.0f;
    c.score = score;
    c.class_id = class_id;
    c.label = label;
    return c;
}

static void test_mask_focus_selection() {
    // 1. Focus selects the 4th-ranked box
    std::vector<FocusCandidate> candidates = {
        make_candidate(0.10f, 0.10f, 0.10f, 0.10f, 0.95f, 0, "obj"),
        make_candidate(0.30f, 0.10f, 0.10f, 0.10f, 0.90f, 0, "obj"),
        make_candidate(0.50f, 0.10f, 0.10f, 0.10f, 0.85f, 0, "obj"),
        make_candidate(0.70f, 0.10f, 0.10f, 0.10f, 0.80f, 0, "obj"),  // 4th ranked
        make_candidate(0.10f, 0.50f, 0.10f, 0.10f, 0.75f, 0, "obj"),
    };

    MaskFocus focus;
    focus.active = true;
    // Box 3 content bounds: [0.65, 0.05, 0.75, 0.15]
    focus.box_norm[0] = 0.65f;
    focus.box_norm[1] = 0.05f;
    focus.box_norm[2] = 0.75f;
    focus.box_norm[3] = 0.15f;

    int sel = select_focus_box(focus, candidates, 0, 140, 640, 360, 640, 640);
    CHECK_MSG(sel == 3, "Focus must select the 4th-ranked box");

    // 2. Label filter
    std::vector<FocusCandidate> multi_label = {
        make_candidate(0.30f, 0.30f, 0.20f, 0.20f, 0.95f, 0, "cup"),
        make_candidate(0.32f, 0.32f, 0.20f, 0.20f, 0.80f, 1, "bottle"),
    };
    MaskFocus bottle_focus;
    bottle_focus.active = true;
    bottle_focus.label = "bottle";
    bottle_focus.box_norm[0] = 0.20f;
    bottle_focus.box_norm[1] = 0.20f;
    bottle_focus.box_norm[2] = 0.40f;
    bottle_focus.box_norm[3] = 0.40f;

    int sel_bottle = select_focus_box(bottle_focus, multi_label, 0, 140, 640, 360, 640, 640);
    CHECK_MSG(sel_bottle == 1, "Focus with label 'bottle' must ignore 'cup' even with higher IoU");

    MaskFocus any_focus = bottle_focus;
    any_focus.label = "";  // empty matches any label
    int sel_any = select_focus_box(any_focus, multi_label, 0, 140, 640, 360, 640, 640);
    CHECK_MSG(sel_any == 0, "Focus with empty label must match highest IoU candidate");

    // 3. IoU threshold (< 0.1 rejected, >= 0.1 accepted)
    std::vector<FocusCandidate> single_cand = {
        make_candidate(0.10f, 0.10f, 0.20f, 0.20f, 0.90f, 0, "obj"),  // bounds: [0, 0, 0.2, 0.2]
    };
    MaskFocus low_iou_focus;
    low_iou_focus.active = true;
    // Overlap [0.18, 0.18, 0.20, 0.20] has area 0.0004, union ~0.0796 -> IoU ~0.005 < 0.1
    low_iou_focus.box_norm[0] = 0.18f;
    low_iou_focus.box_norm[1] = 0.18f;
    low_iou_focus.box_norm[2] = 0.38f;
    low_iou_focus.box_norm[3] = 0.38f;

    int sel_low = select_focus_box(low_iou_focus, single_cand, 0, 140, 640, 360, 640, 640);
    CHECK_MSG(sel_low == -1, "IoU < 0.1 must be rejected");

    MaskFocus high_iou_focus;
    high_iou_focus.active = true;
    // Overlap [0.10, 0.10, 0.20, 0.20] has area 0.01, union 0.07 -> IoU ~0.14 >= 0.1
    high_iou_focus.box_norm[0] = 0.10f;
    high_iou_focus.box_norm[1] = 0.10f;
    high_iou_focus.box_norm[2] = 0.30f;
    high_iou_focus.box_norm[3] = 0.30f;

    int sel_high = select_focus_box(high_iou_focus, single_cand, 0, 140, 640, 360, 640, 640);
    CHECK_MSG(sel_high == 0, "IoU >= 0.1 must be accepted");
}

int main() {
    std::printf("Running seg_decode tests...\n");
    test_random_crops();
    test_edge_cases();
    test_mask_focus_selection();

    if (g_fail == 0) {
        std::printf("All seg_decode tests passed!\n");
        return 0;
    } else {
        std::printf("%d seg_decode tests failed!\n", g_fail);
        return 1;
    }
}
