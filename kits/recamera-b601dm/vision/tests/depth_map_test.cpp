// GET /api/depth: millimetre encoding, body layout and grid mapping, gate, and the handler
// against a simulated frame thread.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "depth_map.h"
#include "vision_control.h"
#include "json.hpp"

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

static constexpr int kW = 384;
static constexpr int kH = 224;

/** Disparity of a synthetic scene: depth 0.30 m at the left edge to 0.60 m at the right. */
static std::vector<float> gradient_disp() {
    std::vector<float> d(static_cast<size_t>(kW * kH));
    for (int j = 0; j < kH; ++j)
        for (int i = 0; i < kW; ++i) d[static_cast<size_t>(j * kW + i)] = 1.0f / (0.30f + 0.30f * i / (kW - 1));
    return d;
}

static void test_scale() {
    CHECK(std::fabs(depth_scale_m(600.0, 640.0) - 1.0) < 1e-12);
    CHECK(std::fabs(depth_scale_m(660.0, 640.0) - 1.1) < 1e-12);
    CHECK(depth_scale_m(0.0, 640.0) == 1.0);
}

static void test_encode() {
    const float nan = NAN;
    const float disp[8] = {2.0f, 0.0f, -1.0f, nan, 100.0f, 0.01f, 1.0f, 4.0f};
    std::vector<uint16_t> mm;
    const DepthMapStats st = encode_depth_mm(disp, 4, 2, 1.0, mm);
    CHECK(mm.size() == 8);
    CHECK(mm[0] == 500);   // 1 / 2 m
    CHECK(mm[1] == 0 && mm[2] == 0 && mm[3] == 0);  // no disparity
    CHECK(mm[4] == 0);     // 10 mm: below the 20 mm floor
    CHECK(mm[5] == 0);     // 100 m: beyond 65 m
    CHECK(mm[6] == 1000 && mm[7] == 250);
    CHECK(st.valid == 3 && st.min_mm == 250 && st.max_mm == 1000 && st.p50_mm == 500);
    // scale 1.1 (fx 660): 1.1 / 2 = 550 mm
    encode_depth_mm(disp, 1, 1, 1.1, mm);
    CHECK(mm.size() == 1 && mm[0] == 550);
    // nothing valid
    const DepthMapStats none = encode_depth_mm(disp + 1, 3, 1, 1.0, mm);
    CHECK(none.valid == 0 && none.p2_mm == 0 && none.p98_mm == 0);

    // gradient: percentiles of 300..600 mm columns
    const std::vector<float> g = gradient_disp();
    const DepthMapStats gs = encode_depth_mm(g.data(), kW, kH, 1.0, mm);
    CHECK(gs.valid == static_cast<uint32_t>(kW * kH));
    CHECK(gs.min_mm == 300 && gs.max_mm == 600);
    CHECK(std::abs(static_cast<int>(gs.p2_mm) - 306) <= 1);
    CHECK(std::abs(static_cast<int>(gs.p98_mm) - 594) <= 1);
    CHECK(mm[0] == 300 && mm[kW - 1] == 600 && mm[static_cast<size_t>(100 * kW + 191)] == 450);
}

static void test_body() {
    const std::vector<float> g = gradient_disp();
    std::vector<uint16_t> mm;
    const DepthMapStats st = encode_depth_mm(g.data(), kW, kH, 1.0, mm);
    DepthMapMeta m;
    m.frame_id = 42;
    m.ts_ms = 1763700000123ull;
    m.inference_ms = 173.26f;
    m.model = "zipdepth_metric_384x224_bf16_rgb";
    const std::string body = depth_map_body(m, st, mm);
    CHECK(body.compare(0, 4, "RDM1") == 0);
    uint32_t n = 0;
    for (int b = 0; b < 4; ++b) n |= static_cast<uint32_t>(static_cast<uint8_t>(body[4 + b])) << (8 * b);
    CHECK((8 + n) % kDepthMapAlign == 0);
    CHECK(body.size() == 8 + n + static_cast<size_t>(kW * kH) * 2);
    // first value little-endian: 300 = 0x012C
    CHECK(static_cast<uint8_t>(body[8 + n]) == 0x2C && static_cast<uint8_t>(body[8 + n + 1]) == 0x01);

    std::string js;
    std::vector<uint16_t> back;
    CHECK(parse_depth_map_body(body, js, back));
    CHECK(back == mm);
    const json j = json::parse(js);
    CHECK(j["format"] == "u16le_mm" && j["units"] == "mm" && j["invalid"] == 0);
    CHECK(j["width"] == kW && j["height"] == kH);
    CHECK(j["frame_id"] == 42 && j["ts_ms"] == 1763700000123ull);
    CHECK(std::fabs(j["inference_ms"].get<double>() - 173.3) < 1e-3);
    CHECK(j["model"] == "zipdepth_metric_384x224_bf16_rgb");
    CHECK(j["mapping"]["image_w"] == 640 && j["mapping"]["image_h"] == 360 && j["mapping"]["fit"] == "stretch");
    CHECK(j["mapping"]["x0"] == 0 && j["mapping"]["x1"] == 640 && j["mapping"]["y1"] == 360);
    CHECK(j["stats"]["valid"] == kW * kH && j["stats"]["p50_mm"] == st.p50_mm);
    CHECK(j["scale"]["scale_m"] == 1.0 && j["scale"]["fx"] == 600.0);

    // mapping: image pixel (u, v) -> cell (floor(u * W / 640), floor(v * H / 360)); u = 320 is
    // column 192, depth 0.30 + 0.30 * 192 / 383
    const int col = static_cast<int>(320.0 * kW / 640.0);
    const int row = static_cast<int>(180.0 * kH / 360.0);
    CHECK(col == 192 && row == 112);
    CHECK(back[static_cast<size_t>(row * kW + col)] == static_cast<uint16_t>(std::lround(1000.0 * (0.30 + 0.30 * 192 / 383.0))));

    std::string bad = body;
    bad[0] = 'X';
    CHECK(!parse_depth_map_body(bad, js, back));
    CHECK(!parse_depth_map_body(body.substr(0, 6), js, back));
}

static void test_gate() {
    DepthMapGate g(1000);
    uint64_t retry = 0;
    CHECK(g.begin(10000, retry) == DepthMapGate::Admit::Ok);
    CHECK(g.begin(10100, retry) == DepthMapGate::Admit::Busy);
    g.end();
    CHECK(g.begin(10300, retry) == DepthMapGate::Admit::RateLimited && retry == 700);
    CHECK(g.begin(10999, retry) == DepthMapGate::Admit::RateLimited && retry == 1);
    CHECK(g.begin(11000, retry) == DepthMapGate::Admit::Ok && retry == 0);
    g.end();
}

static VisionControl::Options opts(bool depth, bool metric) {
    VisionControl::Options o;
    o.mode = Mode::Yolo;
    o.yolo_available = true;
    o.model_name = "yolo11n";
    o.depth_available = depth;
    o.depth_model_name = depth ? "zipdepth_metric_384x224_bf16_rgb" : "";
    o.depth_metric = metric;
    return o;
}

/** Frame thread: runs frame_plan() every 20 ms and attaches the grid while a map request waits. */
class FakeCamera {
public:
    explicit FakeCamera(VisionControl& c) : c_(c), disp_(gradient_disp()) { th_ = std::thread([this] { loop(); }); }
    ~FakeCamera() {
        stop_ = true;
        th_.join();
    }
    std::atomic<int> depth_runs{0};

private:
    void loop() {
        uint32_t id = 0;
        while (!stop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            const uint64_t frame_ms = mono_ms();
            const Pipelines p = c_.frame_plan(frame_ms);
            if (!p.any()) continue;
            VisionResult r;
            r.frame_id = ++id;
            r.frame_ms = frame_ms;
            r.ts_ms = 1763700000000ull + id;
            r.ran_depth = p.depth;
            r.depth_ms = 173.0f;
            if (p.depth) {
                ++depth_runs;
                if (c_.depth_map_wanted()) {
                    r.depth_grid_w = kW;
                    r.depth_grid_h = kH;
                    r.depth_disp = std::make_shared<const std::vector<float>>(disp_);
                }
            }
            c_.publish(r);
        }
    }
    VisionControl& c_;
    std::vector<float> disp_;
    std::atomic<bool> stop_{false};
    std::thread th_;
};

static bool has_header(const HttpReply& r, const std::string& k, const std::string& v) {
    for (const auto& h : r.headers)
        if (h.first == k && h.second == v) return true;
    return false;
}

static void test_handler() {
    {
        VisionControl c(opts(false, false), nullptr);
        const HttpReply r = handle_control_request(c, "GET", "/api/depth", "");
        CHECK(r.status == 409 && json::parse(r.body)["error"] == "depth unavailable");
        CHECK(r.content_type == "application/json");
    }
    {
        VisionControl c(opts(true, false), nullptr);
        const HttpReply r = handle_control_request(c, "GET", "/api/depth", "");
        CHECK(r.status == 409 && json::parse(r.body)["error"] == "depth not metric");
    }
    {
        VisionControl c(opts(true, true), nullptr);
        // no frame thread: times out, and the gate is released
        const HttpReply t = handle_control_request(c, "GET", "/api/depth?wait_ms=50", "", 5000);
        CHECK(t.status == 504 && json::parse(t.body)["error"] == "depth timeout");
        CHECK(!c.depth_map_wanted());
        const HttpReply rl = handle_control_request(c, "GET", "/api/depth", "", 5400);
        CHECK(rl.status == 429 && has_header(rl, "Retry-After", "1"));
        CHECK(json::parse(rl.body)["retry_after_ms"] == 600);
        CHECK(handle_control_request(c, "POST", "/api/depth", "{}").status == 404);
    }
    {
        VisionControl c(opts(true, true), nullptr);
        FakeCamera cam(c);
        const HttpReply r = handle_control_request(c, "GET", "/api/depth", "", 20000);
        CHECK(r.status == 200 && r.content_type == "application/octet-stream");
        std::string js;
        std::vector<uint16_t> mm;
        CHECK(parse_depth_map_body(r.body, js, mm));
        CHECK(mm.size() == static_cast<size_t>(kW * kH) && mm[0] == 300 && mm[kW - 1] == 600);
        const json j = json::parse(js);
        CHECK(j["frame_id"].get<uint32_t>() >= 1 && j["inference_ms"] == 173.0);
        CHECK(j["stats"]["min_mm"] == 300 && j["stats"]["max_mm"] == 600);
        CHECK(!c.depth_map_wanted());
        CHECK(cam.depth_runs.load() == 1);  // one depth inference, no continuous depth
        // 1 per second
        const HttpReply again = handle_control_request(c, "GET", "/api/depth", "", 20500);
        CHECK(again.status == 429 && has_header(again, "Retry-After", "1"));
        const HttpReply later = handle_control_request(c, "GET", "/api/depth", "", 21000);
        CHECK(later.status == 200);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK(cam.depth_runs.load() == 2);
    }
}

int main() {
    test_scale();
    test_encode();
    test_body();
    test_gate();
    test_handler();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("depth_map_test: all checks passed\n");
    return 0;
}
