// Host tests for gray_frame.{h,cpp} and VisionControl atomic gray publish / trigger handles / rollover.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "gray_frame.h"
#include "json.hpp"
#include "vision_control.h"

using namespace rv;
using json = nlohmann::json;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

static std::shared_ptr<GrayFrame> make_gray(const std::string& inst, uint32_t id, int w = kGrayWidth, int h = kGrayHeight,
                                            uint8_t fill = 0x5A) {
    auto f = std::make_shared<GrayFrame>();
    f->instance_id = inst;
    f->frame_id = id;
    f->width = w;
    f->height = h;
    f->stride = w;
    f->frame_w = 640;
    f->frame_h = 640;
    f->crop_x = 0;
    f->crop_y = 140;
    f->crop_w = w;
    f->crop_h = h;
    f->channel = 0;
    f->ts_ms = 1763791234000ull + id;
    f->frame_mono_ms = 10000 + id;
    f->wait_mono_ms = 9990 + id;
    f->grab_mono_ms = 10002 + id;
    f->stored_mono_ms = 10007 + id;
    f->convert_ms = 4.25f;
    f->model_generation = 2;
    auto px = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(w) * static_cast<size_t>(h), fill);
    f->pixels = std::move(px);
    return f;
}

static void test_rgb_primaries_and_stride() {
    // 5 pixels in one row with padded stride (15 bytes payload + 9 bytes garbage = 24)
    const int w = 5;
    const int h = 2;
    const int stride = w * 3 + 9;
    std::vector<uint8_t> rgb(static_cast<size_t>(h) * static_cast<size_t>(stride), 0xEE);
    auto set_px = [&](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        uint8_t* p = rgb.data() + static_cast<size_t>(y) * stride + static_cast<size_t>(x) * 3u;
        p[0] = r;
        p[1] = g;
        p[2] = b;
    };
    // Row 0: black, white, pure R, pure G, pure B
    set_px(0, 0, 0, 0, 0);
    set_px(1, 0, 255, 255, 255);
    set_px(2, 0, 255, 0, 0);
    set_px(3, 0, 0, 255, 0);
    set_px(4, 0, 0, 0, 255);
    // Row 1: mid-tones
    set_px(0, 1, 100, 150, 200);
    set_px(1, 1, 1, 1, 1);
    set_px(2, 1, 254, 254, 254);
    set_px(3, 1, 128, 64, 32);
    set_px(4, 1, 17, 233, 91);

    std::vector<uint8_t> out(static_cast<size_t>(w * h), 0);
    CHECK(rgb888_to_gray(rgb.data(), w, h, stride, out.data()));
    CHECK(out[0] == 0);
    CHECK(out[1] == 255);
    CHECK(out[2] == static_cast<uint8_t>((255u * 77u) >> 8));
    CHECK(out[3] == static_cast<uint8_t>((255u * 150u) >> 8));
    CHECK(out[4] == static_cast<uint8_t>((255u * 29u) >> 8));
    CHECK(out[5] == static_cast<uint8_t>((100u * 77u + 150u * 150u + 200u * 29u) >> 8));
    CHECK(out[6] == 1);
    CHECK(out[7] == 254);
    CHECK(out[8] == static_cast<uint8_t>((128u * 77u + 64u * 150u + 32u * 29u) >> 8));
    CHECK(out[9] == static_cast<uint8_t>((17u * 77u + 233u * 150u + 91u * 29u) >> 8));

    // Random RGB with padded stride matches reference ArUco formula bit-for-bit
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 255);
    const int rw = 37, rh = 19, rstride = rw * 3 + 11;
    std::vector<uint8_t> rbuf(static_cast<size_t>(rh) * rstride);
    for (auto& b : rbuf) b = static_cast<uint8_t>(dist(rng));
    std::vector<uint8_t> rout(static_cast<size_t>(rw * rh), 0);
    CHECK(rgb888_to_gray(rbuf.data(), rw, rh, rstride, rout.data()));
    for (int y = 0; y < rh; ++y) {
        for (int x = 0; x < rw; ++x) {
            const uint8_t* p = rbuf.data() + static_cast<size_t>(y) * rstride + static_cast<size_t>(x) * 3u;
            const uint8_t expect = static_cast<uint8_t>((uint32_t(p[0]) * 77u + uint32_t(p[1]) * 150u + uint32_t(p[2]) * 29u) >> 8);
            CHECK(rout[static_cast<size_t>(y) * rw + x] == expect);
        }
    }

    // Invalid inputs rejected
    CHECK(!rgb888_to_gray(nullptr, w, h, stride, out.data()));
    CHECK(!rgb888_to_gray(rgb.data(), w, h, stride, nullptr));
    CHECK(!rgb888_to_gray(rgb.data(), 0, h, stride, out.data()));
    CHECK(!rgb888_to_gray(rgb.data(), w, -1, stride, out.data()));
    CHECK(!rgb888_to_gray(rgb.data(), w, h, w * 3 - 1, out.data()));
}

static void test_crop_edges_mutation_and_failure() {
    // 640x640 letterbox with padded stride; black bars outside rows 140..499, distinct edge rows
    const int fw = 640, fh = 640, fstride = fw * 3 + 16;
    std::vector<uint8_t> frame(static_cast<size_t>(fh) * fstride, 0);
    for (int y = 140; y < 500; ++y) {
        uint8_t* row = frame.data() + static_cast<size_t>(y) * fstride;
        for (int x = 0; x < fw; ++x) {
            uint8_t v = 120;
            if (y == 140) v = 200;       // top visible edge
            else if (y == 499) v = 220;  // bottom visible edge
            if (x == 0) v = 180;
            if (x == 639) v = 240;
            row[3 * x] = v;
            row[3 * x + 1] = v;
            row[3 * x + 2] = v;
        }
    }
    // Paint sentinel values on rows 139 and 500 (must NOT appear in crop)
    std::memset(frame.data() + 139u * fstride, 11, fw * 3);
    std::memset(frame.data() + 500u * fstride, 22, fw * 3);

    std::vector<uint8_t> gray;
    CHECK(extract_gray_crop(frame.data(), fw, fh, fstride, 0, 140, 640, 360, gray));
    CHECK(gray.size() == 640u * 360u);
    CHECK(gray[0] == 180);
    CHECK(gray[1] == 200);
    CHECK(gray[639] == 240);
    CHECK(gray[359u * 640u + 0] == 180);
    CHECK(gray[359u * 640u + 1] == 220);
    CHECK(gray[359u * 640u + 639] == 240);
    CHECK(gray[100u * 640u + 100] == 120);

    // Mutate source RGB after conversion: owned gray pixels must be unaffected
    std::fill(frame.begin(), frame.end(), 0);
    CHECK(gray[1] == 200 && gray[359u * 640u + 1] == 220 && gray[100u * 640u + 100] == 120);

    // Invalid crop bounds leave output untouched
    std::vector<uint8_t> keep = {1, 2, 3};
    CHECK(!extract_gray_crop(frame.data(), fw, fh, fstride, -1, 140, 640, 360, keep));
    CHECK(!extract_gray_crop(frame.data(), fw, fh, fstride, 0, 300, 640, 360, keep));
    CHECK(!extract_gray_crop(frame.data(), fw, fh, fw * 3 - 3, 0, 140, 640, 360, keep));
    CHECK(keep == std::vector<uint8_t>({1, 2, 3}));
}

static void test_rgf1_roundtrip_and_alignment() {
    auto f = make_gray("rv-0123456789abcdef", 42, kGrayWidth, kGrayHeight, 0);
    auto px = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(kGrayWidth * kGrayHeight));
    for (size_t i = 0; i < px->size(); ++i) (*px)[i] = static_cast<uint8_t>((i * 31u + 7u) & 0xFFu);
    f->pixels = px;

    const std::string hdr = gray_frame_header(*f);
    CHECK(hdr.size() >= 8);
    CHECK(hdr.compare(0, 4, "RGF1") == 0);
    uint32_t n = 0;
    for (int b = 0; b < 4; ++b) n |= static_cast<uint32_t>(static_cast<uint8_t>(hdr[4 + b])) << (8 * b);
    CHECK(hdr.size() == 8u + n);
    CHECK((8u + n) % kGrayFrameAlign == 0);

    const std::string body = gray_frame_body(*f);
    CHECK(body.size() == hdr.size() + px->size());

    std::string js;
    std::vector<uint8_t> back;
    CHECK(parse_gray_frame_body(body, js, back));
    CHECK(back == *px);
    const json j = json::parse(js);
    CHECK(j["api_version"] == 2 && j["encoding"] == "RGF1" && j["format"] == "gray8");
    CHECK(j["instance_id"] == "rv-0123456789abcdef" && j["frame_id"] == 42);
    CHECK(j["width"] == 640 && j["height"] == 360 && j["stride"] == 640);
    CHECK(j["crop"] == json({0, 140, 640, 360}) && j["frame_size"] == json({640, 640}));
    CHECK(j["luma"] == "(77*R+150*G+29*B)>>8" && j["luma_range"] == "full_0_255");

    // Malformed bodies rejected
    std::string bad_magic = body;
    bad_magic[0] = 'X';
    CHECK(!parse_gray_frame_body(bad_magic, js, back));
    CHECK(!parse_gray_frame_body(body.substr(0, body.size() - 1), js, back));
    CHECK(!parse_gray_frame_body(body + "X", js, back));
    CHECK(!parse_gray_frame_body(body.substr(0, 6), js, back));
}

static void test_ring_sparse_eviction_and_instance() {
    GrayRing ring(4);
    const std::string inst = "rv-inst1";
    std::shared_ptr<const GrayFrame> out;

    // Empty ring: missing IDs are NotStored (404), wrong instance is Expired (410)
    CHECK(ring.find(inst, 1, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find("rv-other", 1, inst, out) == GrayRing::Lookup::Expired);

    // Sparse inserts: 10, 20, 30, 40 (no eviction yet)
    CHECK(ring.put(make_gray(inst, 10, 640, 360, 10)) == nullptr);
    CHECK(ring.put(make_gray(inst, 20, 640, 360, 20)) == nullptr);
    CHECK(ring.put(make_gray(inst, 30, 640, 360, 30)) == nullptr);
    CHECK(ring.put(make_gray(inst, 40, 640, 360, 40)) == nullptr);
    CHECK(!ring.has_evicted() && ring.size() == 4);
    // Low missing ID before first eviction -> NotStored (404)
    CHECK(ring.find(inst, 5, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 15, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 50, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 10, inst, out) == GrayRing::Lookup::Found && out->pixels->at(0) == 10);
    // Mismatched instance even for a present ID -> Expired (410)
    CHECK(ring.find("rv-old", 10, inst, out) == GrayRing::Lookup::Expired && !out);

    // Insert 50 -> displaces 10
    auto disp = ring.put(make_gray(inst, 50, 640, 360, 50));
    CHECK(disp != nullptr && disp->frame_id == 10);
    CHECK(ring.has_evicted() && ring.highest_evicted_id() == 10);
    // <= 10 -> Expired; 15 (between 10 and 20) -> NotStored; 20..50 -> Found; 60 -> NotStored
    CHECK(ring.find(inst, 10, inst, out) == GrayRing::Lookup::Expired);
    CHECK(ring.find(inst, 5, inst, out) == GrayRing::Lookup::Expired);
    CHECK(ring.find(inst, 15, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 20, inst, out) == GrayRing::Lookup::Found && out->pixels->at(0) == 20);
    CHECK(ring.find(inst, 50, inst, out) == GrayRing::Lookup::Found && out->pixels->at(0) == 50);
    CHECK(ring.find(inst, 60, inst, out) == GrayRing::Lookup::NotStored);
}

static void test_atomic_publish_failed_gray_and_rollover() {
    VisionControl::Options o;
    o.instance_id = "rv-fixed";
    o.initial_frame_id = std::numeric_limits<uint32_t>::max() - 1;
    VisionControl ctl(o, nullptr);
    CHECK(ctl.instance_id() == "rv-fixed");

    // Failed gray (ran_gray=true with null record) must NOT publish ran_gray or partial ring entry
    VisionResult bad;
    std::string inst0;
    bad.frame_id = ctl.next_frame_id(inst0);
    bad.instance_id = inst0;
    bad.frame_ms = 1000;
    bad.ran_yolo = true;
    bad.ran_gray = true;
    CHECK(bad.frame_id == std::numeric_limits<uint32_t>::max() && inst0 == "rv-fixed");
    ctl.publish(bad, nullptr);

    VisionResult got;
    CHECK(!ctl.latest(false, 0, 0, got));
    CHECK(!got.ran_gray);
    std::shared_ptr<const GrayFrame> gf;
    CHECK(ctl.get_gray(inst0, bad.frame_id, gf) == GrayRing::Lookup::NotStored);

    // Valid atomic publish on UINT32_MAX
    auto valid_gray = make_gray(inst0, bad.frame_id, 640, 360, 77);
    ctl.publish(bad, valid_gray);
    CHECK(!ctl.latest(false, 0, 0, got) && got.ran_gray && got.instance_id == inst0);
    CHECK(ctl.get_gray(inst0, bad.frame_id, gf) == GrayRing::Lookup::Found && gf->pixels->at(0) == 77);

    // Rollover: next_frame_id rotates instance_id and resets frame_id to 1, expiring old instance
    std::string inst1;
    const uint32_t wrapped_id = ctl.next_frame_id(inst1);
    CHECK(wrapped_id == 1 && !inst1.empty() && inst1 != inst0);
    CHECK(ctl.instance_id() == inst1);
    CHECK(ctl.get_gray(inst0, bad.frame_id, gf) == GrayRing::Lookup::Expired);
}

static void test_same_ms_trigger_handles_and_waiter_interleaving() {
    VisionControl::Options o;
    o.instance_id = "rv-trig";
    VisionControl ctl(o, nullptr);

    // Two simultaneous triggers sharing the exact same request_ms with different pipelines
    const uint64_t t_req = 50000;
    {
        auto s1 = ctl.trigger_scope(t_req, Pipelines{true, false, false, false});
        auto s2 = ctl.trigger_scope(t_req, Pipelines{false, true, false, true});
        Pipelines p = ctl.frame_plan(t_req);
        CHECK(p.yolo && p.aruco && p.gray && !p.depth);

        // Ending s1 must NOT cancel s2 even though request_ms is identical!
        s1.release();
        p = ctl.frame_plan(t_req + 1);
        CHECK(!p.yolo && p.aruco && p.gray);
    }
    // Both released -> back to standby
    CHECK(!ctl.frame_plan(t_req + 2).any());

    // Waiter interleaving: frame 1 publishes [yolo, gray]; immediately frame 2 publishes [yolo] only
    // before the waiter calls/resumes wait_fresh -> waiter must still get frame 1's identity & gray!
    VisionResult r1;
    r1.frame_id = ctl.next_frame_id(r1.instance_id);
    r1.frame_ms = t_req + 10;
    r1.ran_yolo = true;
    r1.ran_gray = true;
    auto g1 = make_gray(r1.instance_id, r1.frame_id, 640, 360, 0xAB);
    ctl.publish(r1, g1);

    VisionResult r2;
    r2.frame_id = ctl.next_frame_id(r2.instance_id);
    r2.frame_ms = t_req + 20;
    r2.ran_yolo = true;
    r2.ran_gray = false;
    ctl.publish(r2, nullptr);

    VisionResult matched;
    const bool timed_out = ctl.wait_fresh(0, t_req + 5, Pipelines{true, false, false, true}, 100, matched);
    CHECK(!timed_out);
    CHECK(matched.frame_id == r1.frame_id && matched.instance_id == r1.instance_id && matched.ran_gray);
    std::shared_ptr<const GrayFrame> fetched;
    CHECK(ctl.get_gray(matched.instance_id, matched.frame_id, fetched) == GrayRing::Lookup::Found);
    CHECK(fetched && fetched->pixels->at(0) == 0xAB);
}

int main() {
    test_rgb_primaries_and_stride();
    test_crop_edges_mutation_and_failure();
    test_rgf1_roundtrip_and_alignment();
    test_ring_sparse_eviction_and_instance();
    test_atomic_publish_failed_gray_and_rollover();
    test_same_ms_trigger_handles_and_waiter_interleaving();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("gray_frame: all checks passed\n");
    return 0;
}
