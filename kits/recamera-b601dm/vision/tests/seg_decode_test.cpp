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

int main() {
    std::printf("Running seg_decode tests...\n");
    test_random_crops();
    test_edge_cases();

    if (g_fail == 0) {
        std::printf("All seg_decode tests passed!\n");
        return 0;
    } else {
        std::printf("%d seg_decode tests failed!\n", g_fail);
        return 1;
    }
}
