// Modified from sscma-example-sg200x solutions/aruco-detector/tests/inference_control_test.cpp
// (commit 9af8118, Apache-2.0): modes, subscriptions, per-request pipelines, combined envelope.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "http_control.h"
#include "result_json.h"
#include "vision_control.h"
#include "json.hpp"

using namespace rv;
using json = nlohmann::json;
using clk = std::chrono::steady_clock;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

static json req(VisionControl& c, const std::string& m, const std::string& t, const std::string& b = "",
                int* status = nullptr, uint64_t now_ms = 1000100) {
    HttpReply r = handle_control_request(c, m, t, b, now_ms);
    if (status) *status = r.status;
    return json::parse(r.body);
}

struct Calls {
    std::vector<std::pair<bool, int>> v;
    VisionControl::OnChange fn() {
        return [this](bool on, int fps) { v.push_back({on, fps}); };
    }
};

static VisionControl::Options opts(Mode m, bool enabled = false, int fps = 10, bool yolo = true, bool depth = false) {
    VisionControl::Options o;
    o.mode = m;
    o.enabled = enabled;
    o.fps = fps;
    o.yolo_available = yolo;
    o.model_name = "yolo11n";
    o.model_path = "/userdata/local/models/yolo11n.cvimodel";
    o.depth_available = depth;
    o.depth_model_name = depth ? "zipdepth_base_384x224_bf16_rgb" : "";
    o.depth_model_path = depth ? "/userdata/local/models/zipdepth_base_384x224_bf16_rgb.cvimodel" : "";
    o.depth_metric = false;
    return o;
}

/** Simulated frame thread: runs frame_plan() at `period`, like on_frame. */
class FakeCamera {
public:
    FakeCamera(VisionControl& c, int period_ms) : c_(c), period_(period_ms) {
        th_ = std::thread([this] { loop(); });
    }
    ~FakeCamera() {
        stop_ = true;
        th_.join();
    }
    std::atomic<int> processed{0};

private:
    void loop() {
        uint32_t id = 0;
        while (!stop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(period_));
            const uint64_t frame_ms = mono_ms();
            const Pipelines p = c_.frame_plan(frame_ms);
            if (!p.any()) continue;
            VisionResult r;
            r.frame_id = ++id;
            r.frame_ms = frame_ms;
            r.ts_ms = 1000000 + id;
            r.mode = c_.mode();
            r.ran_yolo = p.yolo;
            r.ran_aruco = p.aruco;
            r.ran_depth = p.depth;
            r.detections_json = p.yolo ? R"([{"class_id":39,"class_name":"bottle","confidence":0.7,"x":0.5,"y":0.5,"w":0.1,"h":0.2}])" : "[]";
            r.markers_json = p.aruco ? R"([{"id":3,"center":[320.0,180.0]}])" : "[]";
            r.count = p.aruco ? 1 : 0;
            if (p.depth) {
                r.depth_json = R"({"grid_h":224,"grid_w":384,"metric_model":false,"table_samples":[[320.0,180.0,0.5]]})";
            }
            c_.publish(r);
            ++processed;
        }
    }
    VisionControl& c_;
    int period_;
    std::atomic<bool> stop_{false};
    std::thread th_;
};

static void test_modes() {
    Calls calls;
    VisionControl c(opts(Mode::Yolo), calls.fn());
    CHECK(calls.v.size() == 1 && calls.v[0] == std::make_pair(false, 1));
    CHECK(c.mode() == Mode::Yolo && !c.continuous_enabled() && !c.processing());
    CHECK(!c.frame_plan(mono_ms()).any());

    json m = req(c, "GET", "/api/mode");
    CHECK(m["api_version"] == kApiVersion);
    CHECK(m["mode"] == "yolo" && m["enabled"] == false && m["fps"] == 1 && m["active_fps"] == 10);
    CHECK(m["available"]["yolo"] == true && m["available"]["aruco"] == true && m["trigger"] == true);
    CHECK(m["available"]["depth"] == false && m["depth_model"].is_null());
    CHECK(m["modes"].size() == 4);

    // mode + enabled in one call; rate follows
    m = req(c, "POST", "/api/mode", R"({"mode":"both","enabled":true})");
    CHECK(m["mode"] == "both" && m["enabled"] == true && m["fps"] == 10);
    CHECK(c.frame_plan(mono_ms()) == (Pipelines{true, true}));
    CHECK(calls.v.back() == std::make_pair(true, VisionControl::kOpenGateFps));  // 10 fps: gate fully open
    CHECK(m["camera_fps"] == VisionControl::kOpenGateFps && m["measured"]["input_fps"].is_number());

    // switching between active modes does not touch the camera rate
    const size_t n = calls.v.size();
    CHECK(c.set_mode(Mode::Aruco) == VisionControl::Status::Ok);
    CHECK(calls.v.size() == n);
    CHECK(c.frame_plan(mono_ms() + 1000) == (Pipelines{false, true}));

    // off forces enabled false, idle rate
    m = req(c, "POST", "/api/mode", R"({"mode":"off"})");
    CHECK(m["mode"] == "off" && m["enabled"] == false && m["fps"] == 1 && m["last_active_mode"] == "aruco");
    CHECK(calls.v.back() == std::make_pair(false, 1));
    // "off" wins over enabled:true in the same body
    m = req(c, "POST", "/api/mode", R"({"mode":"off","enabled":true})");
    CHECK(m["mode"] == "off" && m["enabled"] == false);

    // legacy enable while off restores the last active mode
    json inf = req(c, "POST", "/api/inference", R"({"enabled":true})");
    CHECK(inf["enabled"] == true && inf["mode"] == "aruco" && inf["api_version"] == kApiVersion);
    CHECK(c.mode() == Mode::Aruco);

    // shared rate is runtime configurable
    m = req(c, "POST", "/api/mode", R"({"fps":5})");
    CHECK(m["fps"] == 5 && m["active_fps"] == 5);
    CHECK(calls.v.back() == std::make_pair(true, 10));  // gate at twice the rate

    int st = 0;
    json e = req(c, "POST", "/api/mode", R"({"mode":"video"})", &st);
    CHECK(st == 400 && e.contains("error"));
    req(c, "POST", "/api/mode", R"({"fps":16})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/mode", R"({"fps":"5"})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/mode", R"({})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/mode", "nope", &st);
    CHECK(st == 400);
    req(c, "DELETE", "/api/mode", "", &st);
    CHECK(st == 404);
    CHECK(c.mode() == Mode::Aruco && c.active_fps() == 5);  // errors change nothing
}

static void test_yolo_unavailable() {
    VisionControl c(opts(Mode::Both, true, 10, /*yolo=*/false), nullptr);
    CHECK(c.mode() == Mode::Aruco);  // both -> aruco without a model
    int st = 0;
    json e = req(c, "POST", "/api/mode", R"({"mode":"yolo"})", &st);
    CHECK(st == 409 && e["error"] == "yolo unavailable");
    req(c, "POST", "/api/mode", R"({"mode":"both","enabled":false})", &st);
    CHECK(st == 409 && c.continuous_enabled());  // rejected as a whole
    json m = req(c, "GET", "/api/mode");
    CHECK(m["available"]["yolo"] == false);

    VisionControl c2(opts(Mode::Yolo, true, 10, false), nullptr);
    CHECK(c2.mode() == Mode::Off && !c2.continuous_enabled() && c2.last_active_mode() == Mode::Aruco);
}

static void test_trigger_each_mode() {
    for (Mode m : {Mode::Yolo, Mode::Aruco, Mode::Both, Mode::Off}) {
        Calls calls;
        VisionControl c(opts(m, false, 10), calls.fn());
        FakeCamera cam(c, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        CHECK(cam.processed == 0);  // standby: nothing runs
        json r = req(c, "POST", "/api/trigger?wait_ms=1000");
        CHECK(r["api_version"] == kApiVersion);
        CHECK(r["success"] == true);
        CHECK(r["frame_id"].get<int>() >= 1);
        const Pipelines want = pipelines_of(m == Mode::Off ? Mode::Yolo : m);  // off runs last_active (yolo)
        CHECK(r["pipelines"]["yolo"]["ran"] == want.yolo);
        CHECK(r["pipelines"]["aruco"]["ran"] == want.aruco);
        CHECK(r["detections"].is_array() && r["markers"].is_array());
        CHECK(r["detections"].empty() != want.yolo);
        CHECK(r["markers"].empty() != want.aruco);
        CHECK(r["count"] == (want.aruco ? 1 : 0));
        CHECK(r["width"] == 640 && r["height"] == 360);
        CHECK(r["mode"] == mode_name(m));
        // timeline: request, gate, serving frame, publish, wake-up, reply (mono_ms)
        const json& tm = r["timing"];
        CHECK(tm.is_object() && tm["accept_ms"] == 0 && tm["read_ms"] == 0);  // no HTTP server here
        CHECK(tm["request_ms"] <= tm["gate_ms"] && tm["gate_ms"] <= tm["wake_ms"] && tm["wake_ms"] <= tm["reply_ms"]);
        CHECK(tm["request_ms"] <= tm["frame_ms"] && tm["frame_ms"] <= tm["done_ms"] && tm["done_ms"] <= tm["wake_ms"]);
        CHECK(tm["skipped"].get<int>() >= 0);
        // back to standby, continuous state untouched
        CHECK(!c.processing() && !c.continuous_enabled() && c.mode() == m);
        CHECK(c.current_fps() == 1);
        CHECK(calls.v.back() == std::make_pair(false, 1));
        // the frame that served the trigger (or one planned just before trigger_end) can still
        // be counting; after that nothing runs
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        const int after = cam.processed;
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        CHECK(cam.processed == after);
    }
}

static void test_trigger_timeout_and_continuous() {
    VisionControl c(opts(Mode::Yolo), nullptr);
    // no frame thread: nothing arrives
    const auto t0 = clk::now();
    json r = req(c, "POST", "/api/trigger?wait_ms=150");
    CHECK(r["success"] == false && r["timeout"] == true && r["count"] == 0);
    CHECK(r["detections"].empty() && r["markers"].empty());
    CHECK(std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - t0).count() >= 140);
    CHECK(!c.processing());

    // trigger during continuous inference returns the next frame and keeps it running
    c.set_enabled(true);
    FakeCamera cam(c, 20);
    json first = req(c, "GET", "/api/results/latest?after=0&wait_ms=1000");
    CHECK(first["frame_id"].get<int>() >= 1);
    r = req(c, "POST", "/api/trigger");
    CHECK(r["success"] == true && r["frame_id"].get<int>() > first["frame_id"].get<int>());
    CHECK(c.continuous_enabled() && c.processing());

    int st = 0;
    req(c, "POST", "/api/trigger?wait_ms=x", "", &st);
    CHECK(st == 400);
}


static void test_rate_gate() {
    // Exact long-run rate out of a 15 fps stream with +-12 ms arrival jitter, for every rate.
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jit(-12.0, 12.0);
    for (int fps = 1; fps <= 15; ++fps) {
        RateGate g;
        int n = 0;
        for (int i = 0; i < 15 * 60; ++i) {  // 60 s
            const double t = 1000.0 + i * (1000.0 / 15) + jit(rng);
            n += g.accept(static_cast<uint64_t>(t), fps) ? 1 : 0;
        }
        CHECK(std::abs(n - fps * 60) <= 1);
    }
    // Input slower than the rate: every frame passes.
    RateGate g;
    int n = 0;
    for (int i = 0; i < 100; ++i) n += g.accept(1000 + i * 200, 10) ? 1 : 0;
    CHECK(n == 100);
    // reset: the next frame passes at once.
    CHECK(g.accept(30000, 1));
    CHECK(!g.accept(30100, 1));
    g.reset();
    CHECK(g.accept(30150, 1));

    RateMeter m;
    for (int i = 0; i < 40; ++i) m.add(10000 + i * 100);  // 10 / s for 4 s
    CHECK(std::abs(m.rate(14000) - 10.0) < 0.6);
    CHECK(m.rate(20000) == 0.0);
}

/**
 * Model of the frame path: VPSS CH0 delivering `hw_fps` (arrivals with jitter), the camera
 * callback's gate (passes a frame once 90 % of 1/gate has passed since the last one it
 * passed), then optionally RateGate. Returns frames processed per second.
 */
static double model_rate(int hw_fps, int gate_fps, int rate_gate_fps, double jitter_ms, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jit(-jitter_ms, jitter_ms);
    RateGate rg;
    double last = -1e9;
    int n = 0;
    const int secs = 60;
    for (int i = 0; i < hw_fps * secs; ++i) {
        const double t = 1000.0 + i * (1000.0 / hw_fps) + jit(rng);
        if (t - last < 0.9 * (1000 / gate_fps)) continue;  // integer period as in ma_camera_sg200x.cpp
        last = t;
        if (rate_gate_fps > 0 && !rg.accept(static_cast<uint64_t>(t), rate_gate_fps)) continue;
        ++n;
    }
    return static_cast<double>(n) / secs;
}

static void test_rate_model() {
    // 1.2.0: CH0 VPSS rate fixed at the start rate (vision.json fps 5), gate = fps, no RateGate.
    // 1.2.1: CH0 VPSS at 15, gate = camera_gate_fps(), RateGate at fps.
    for (double jitter : {10.0, 30.0}) {
        std::printf("rate model (CH0 arrivals +-%.0f ms jitter): fps -> 1.2.0 / 1.2.1 results/s\n", jitter);
        for (int fps : {1, 2, 3, 5, 8, 10, 12, 15}) {
            const double before = model_rate(5, fps, 0, jitter, 11);
            const double after = model_rate(VisionControl::kMaxFps, VisionControl::camera_gate_fps(true, false, fps),
                                            fps, jitter, 11);
            std::printf("  %2d -> %5.2f / %5.2f\n", fps, before, after);
            // at +-30 ms (arrivals can be < 1 gate period apart) rates near the channel rate lose frames
            if (jitter <= 10.0 || fps <= 10) CHECK(std::abs(after - fps) <= 0.05 * fps + 0.02);
        }
    }
    // gate rates
    CHECK(VisionControl::camera_gate_fps(false, false, 10) == VisionControl::kIdleFps);
    CHECK(VisionControl::camera_gate_fps(true, true, 2) == VisionControl::kOpenGateFps);
    CHECK(VisionControl::camera_gate_fps(true, false, 5) == 10);
    CHECK(VisionControl::camera_gate_fps(true, false, 8) == VisionControl::kOpenGateFps);
}

static void test_trigger_gate_and_freshness() {
    // Standby -> trigger opens the camera gate fully, then back to idle.
    Calls calls;
    VisionControl c(opts(Mode::Aruco, false, 5), calls.fn());
    CHECK(c.camera_fps() == VisionControl::kIdleFps);
    {
        const uint64_t req = 5000;
        auto scope = c.trigger_scope(req);
        CHECK(c.processing() && c.camera_fps() == VisionControl::kOpenGateFps);
        CHECK(calls.v.back() == std::make_pair(true, VisionControl::kOpenGateFps));
        CHECK(!c.frame_plan(4999).any());                      // frame from before the request
        CHECK(c.frame_plan(5000) == (Pipelines{false, true}));  // first frame not older than it
    }
    CHECK(!c.processing() && c.camera_fps() == VisionControl::kIdleFps);
    CHECK(calls.v.back() == std::make_pair(false, VisionControl::kIdleFps));

    // wait_fresh ignores a newer result from a frame older than the request.
    VisionResult old;
    old.frame_id = 1;
    old.frame_ms = 100;
    c.publish(old);
    VisionResult out;
    CHECK(c.wait_fresh(0, 200, Pipelines{}, 30, out));  // timeout
    std::thread th([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        VisionResult r;
        r.frame_id = 2;
        r.frame_ms = 250;
        c.publish(r);
    });
    CHECK(!c.wait_fresh(0, 200, Pipelines{}, 2000, out) && out.frame_id == 2);
    th.join();

    // Continuous at 1 fps: the rate gate would skip the next frame, a trigger runs it anyway.
    VisionControl k(opts(Mode::Yolo, true, 1), nullptr);
    CHECK(k.frame_plan(10000).any());
    CHECK(!k.frame_plan(10066).any());
    {
        auto scope = k.trigger_scope(10100);
        CHECK(!k.frame_plan(10099).any());
        CHECK(k.frame_plan(10133) == (Pipelines{true, false}));
    }
    CHECK(!k.frame_plan(10200).any());
}

static void test_trigger_latency() {
    // Frame thread at 15 fps (camera at kMaxFps): a trigger from standby is served by the next
    // frame, not after an idle period.
    for (Mode m : {Mode::Yolo, Mode::Aruco, Mode::Both}) {
        VisionControl c(opts(m, false, 5), nullptr);
        FakeCamera cam(c, 1000 / VisionControl::kMaxFps);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto t0 = clk::now();
        json r = req(c, "POST", "/api/trigger?wait_ms=2000");
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - t0).count();
        CHECK(r["success"] == true);
        CHECK(ms < 200);
        CHECK(c.camera_fps() == VisionControl::kIdleFps && !c.processing());
    }
}

static void test_latest() {
    VisionControl c(opts(Mode::Aruco, true), nullptr);
    json r = req(c, "GET", "/api/results/latest");
    CHECK(r["frame_id"] == 0 && r["timeout"] == true && r["detections"].empty() && r["markers"].empty());
    CHECK(r["mode"] == "aruco");

    VisionResult v;
    v.frame_id = 7;
    v.ts_ms = 1000000;
    v.mode = Mode::Both;
    v.ran_yolo = v.ran_aruco = true;
    v.mask_ms = 4.2f;
    v.detections_json = "[]";
    v.markers_json = R"([{"id":1}])";
    v.count = 1;
    c.publish(v);
    r = req(c, "GET", "/api/results/latest");
    CHECK(r["frame_id"] == 7 && r["age_ms"] == 100 && !r.contains("timeout"));
    CHECK(r["mode"] == "both" && r["family"] == "aruco4x4_50" && r["count"] == 1);
    CHECK(std::abs(r["pipelines"]["yolo"]["mask_ms"].get<double>() - 4.2) < 1e-3);

    v.mask_ms = -1.0f;
    c.publish(v);
    r = req(c, "GET", "/api/results/latest");
    CHECK(!r["pipelines"]["yolo"].contains("mask_ms"));
    r = req(c, "GET", "/api/results/latest?after=7&wait_ms=50");
    CHECK(r["timeout"] == true);

    // a waiting long-poll wakes when processing stops
    std::thread off([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        c.set_mode(Mode::Off);
    });
    const auto t0 = clk::now();
    r = req(c, "GET", "/api/results/latest?after=7&wait_ms=3000");
    off.join();
    CHECK(std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - t0).count() < 1500);
    CHECK(r["timeout"] == true && r["enabled"] == false);

    int st = 0;
    req(c, "GET", "/api/results/latest?after=-1", "", &st);
    CHECK(st == 400);
}

static void test_result_json() {
    ContentRect r = content_rect(640, 640, 1280, 720);
    CHECK(r.x == 0 && r.y == 140 && r.w == 640 && r.h == 360);
    r = content_rect(640, 360, 1280, 720);
    CHECK(r.x == 0 && r.y == 0 && r.w == 640 && r.h == 360);
    r = content_rect(640, 640, 640, 640);
    CHECK(r.y == 0 && r.h == 640);

    // a box centred on the content band centre stays centred; height scales by 640/360
    std::vector<DetBox> d = {{0.5f, 0.5f, 0.1f, 0.18f, 0.71f, 39, "bottle"}};
    json j = json::parse(detections_json(d, content_rect(640, 640, 1280, 720), 640, 640));
    CHECK(j.size() == 1 && j[0]["class_name"] == "bottle" && j[0]["class_id"] == 39 && j[0]["format"] == "xywh_center");
    CHECK(std::abs(j[0]["x"].get<double>() - 0.5) < 1e-3 && std::abs(j[0]["y"].get<double>() - 0.5) < 1e-3);
    CHECK(std::abs(j[0]["h"].get<double>() - 0.32) < 1e-3 && std::abs(j[0]["w"].get<double>() - 0.1) < 1e-3);
    // top of the content band (model y = 140/640) maps to 0
    d[0].y = 140.0f / 640.0f;
    j = json::parse(detections_json(d, content_rect(640, 640, 1280, 720), 640, 640));
    CHECK(std::abs(j[0]["y"].get<double>()) < 1e-3);

    // finite angle and grasp width
    d[0].angle_deg = 45.25f;
    d[0].angle_conf = 0.952f;
    d[0].grasp_width_px = 32.50f;
    j = json::parse(detections_json(d, content_rect(640, 640, 1280, 720), 640, 640));
    CHECK(std::abs(j[0]["angle_deg"].get<double>() - 45.25) < 1e-2);
    CHECK(std::abs(j[0]["angle_conf"].get<double>() - 0.952) < 1e-3);
    CHECK(std::abs(j[0]["grasp_width_px"].get<double>() - 32.50) < 1e-2);

    // NAN angle and grasp width should be omitted from JSON
    d[0].angle_deg = NAN;
    d[0].angle_conf = NAN;
    d[0].grasp_width_px = NAN;
    j = json::parse(detections_json(d, content_rect(640, 640, 1280, 720), 640, 640));
    CHECK(!j[0].contains("angle_deg"));
    CHECK(!j[0].contains("angle_conf"));
    CHECK(!j[0].contains("grasp_width_px"));

    MarkerOut m;
    m.id = 3;
    m.c[0] = 320;
    m.c[1] = 180;
    m.p[2][0] = 10;
    m.t[2] = 0.45;
    json mk = json::parse(markers_json({m}));
    CHECK(mk[0]["id"] == 3 && mk[0]["center"][0] == 320.0 && mk[0]["pose"]["t"][2] == 0.45);
    mk = json::parse(markers_json({m}, 2.0, 2.0));
    CHECK(mk[0]["center"][1] == 360.0 && mk[0]["corners"][2][0] == 20.0 && mk[0]["pose"]["t"][2] == 0.45);
    CHECK(json::parse(markers_json({})).empty());

    json extra = json::parse("{" + debug_extra(Mode::Both, markers_json({m}, 2, 2)) + "}");
    CHECK(extra["mode"] == "both" && extra["markers"].size() == 1);
    extra = json::parse("{" + debug_extra(Mode::Yolo, "") + "}");
    CHECK(extra["markers"].empty());
}

static std::string sub_body(const std::string& client, const std::string& pipes, int fps = 0, int ttl = 0) {
    std::string b = "{\"client\":\"" + client + "\",\"pipelines\":" + pipes;
    if (fps) b += ",\"fps\":" + std::to_string(fps);
    if (ttl) b += ",\"ttl_ms\":" + std::to_string(ttl);
    return b + "}";
}

static void test_standby() {
    Calls calls;
    VisionControl c(opts(Mode::Yolo, false, 5), calls.fn());
    CHECK(!c.processing() && c.camera_fps() == VisionControl::kIdleFps && c.current_fps() == VisionControl::kIdleFps);
    CHECK(!c.continuous().active() && c.continuous().fps == 0);
    for (uint64_t t = 1000; t < 3000; t += 67) CHECK(!c.frame_plan(t).any());
    json l = req(c, "GET", "/api/subscriptions");
    CHECK(l["api_version"] == 2 && l["subscriptions"].empty() && l["continuous"]["active"] == false);
    CHECK(l["default"]["mode"] == "yolo" && l["default"]["enabled"] == false && l["max_fps"] == 5);
    json m = req(c, "GET", "/api/mode");
    CHECK(m["per_request_pipelines"] == true && m["subscriptions"] == true && m["continuous"]["active"] == false);
    CHECK(m["max_fps"] == 5 && m["processing"] == false && m["fps"] == 1);
    CHECK(calls.v.size() == 1 && calls.v[0] == std::make_pair(false, 1));
}

static void test_subscription_union() {
    Calls calls;
    VisionControl c(opts(Mode::Yolo, false, 5), calls.fn());
    int st = 0;
    json a = req(c, "POST", "/api/subscribe", sub_body("overlay", R"(["yolo"])", 2), &st);
    CHECK(st == 200 && a["api_version"] == 2 && a["id"].is_string() && a["pipelines"] == json({"yolo"}) && a["fps"] == 2);
    CHECK(a["ttl_ms"] == VisionControl::kDefaultTtlMs && a["continuous"]["pipelines"] == json({"yolo"}));
    CHECK(c.processing() && calls.v.back() == std::make_pair(true, 4));  // gate at twice the rate
    json b = req(c, "POST", "/api/subscribe", sub_body("calib", R"(["aruco"])", 4), &st);
    CHECK(st == 200 && b["id"] != a["id"]);
    Continuous k = c.continuous();
    CHECK(k.pipelines == (Pipelines{true, true}) && k.fps == 4 && k.subscriptions == 2);
    CHECK(b["continuous"]["pipelines"] == json({"yolo", "aruco"}) && b["continuous"]["fps"] == 4);
    CHECK(c.frame_plan(mono_ms()) == (Pipelines{true, true}));
    // a faster request is clamped to the configured rate (the cap)
    json f = req(c, "POST", "/api/subscribe", sub_body("fast", R"(["yolo"])", 12), &st);
    CHECK(st == 200 && f["fps"] == 5 && f["requested_fps"] == 12 && c.continuous().fps == 5);
    CHECK(calls.v.back() == std::make_pair(true, 10));
    json l = req(c, "GET", "/api/subscriptions");
    CHECK(l["subscriptions"].size() == 3 && l["continuous"]["subscriptions"] == 3);
    // re-posting with the same client updates in place (same id)
    json a2 = req(c, "POST", "/api/subscribe", sub_body("overlay", R"(["yolo","aruco"])", 3), &st);
    CHECK(st == 200 && a2["id"] == a["id"] && a2["pipelines"] == json({"yolo", "aruco"}));
    CHECK(req(c, "GET", "/api/subscriptions")["subscriptions"].size() == 3);
    // delete: the union shrinks, the rate follows
    json d = req(c, "DELETE", "/api/subscribe/" + f["id"].get<std::string>(), "", &st);
    CHECK(st == 200 && d["deleted"] == f["id"] && d["continuous"]["fps"] == 4);
    req(c, "DELETE", "/api/subscribe/" + f["id"].get<std::string>(), "", &st);
    CHECK(st == 404);
    req(c, "DELETE", "/api/subscribe/" + a["id"].get<std::string>(), "", &st);
    req(c, "DELETE", "/api/subscribe/" + b["id"].get<std::string>(), "", &st);
    CHECK(st == 200 && !c.processing() && calls.v.back() == std::make_pair(false, 1));
    // the configured rate is the cap: lowering it slows every subscription
    req(c, "POST", "/api/subscribe", sub_body("overlay", R"(["aruco"])", 5), &st);
    req(c, "POST", "/api/mode", R"({"fps":2})", &st);
    CHECK(st == 200 && c.continuous().fps == 2);

    // errors
    req(c, "POST", "/api/subscribe", R"({"pipelines":["yolo"]})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/subscribe", R"({"client":"x","pipelines":{}})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/subscribe", R"({"client":"x","pipelines":"yolo"})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/subscribe", R"({"client":"x","pipelines":["video"]})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/subscribe", R"({"client":"x","pipelines":["yolo"],"fps":16})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/subscribe", "nope", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/subscribe/zz/keepalive", "{}", &st);
    CHECK(st == 404);
    req(c, "GET", "/api/subscribe/zz", "", &st);
    CHECK(st == 404);
    for (int i = 0; i < VisionControl::kMaxSubscriptions - 1; ++i)
        req(c, "POST", "/api/subscribe", sub_body("c" + std::to_string(i), R"(["aruco"])"), &st);
    CHECK(st == 200);
    req(c, "POST", "/api/subscribe", sub_body("one-too-many", R"(["aruco"])"), &st);
    CHECK(st == 503);

    VisionControl n(opts(Mode::Aruco, false, 5, /*yolo=*/false), nullptr);
    json e = req(n, "POST", "/api/subscribe", sub_body("x", R"(["aruco","yolo"])"), &st);
    CHECK(st == 409 && e["error"] == "yolo unavailable" && !n.processing());
    // no YOLO model: no kind (Options.model_kind is ignored)
    e = req(n, "GET", "/api/mode");
    CHECK(e["model"]["kind"].is_null() && e["available"]["yolo"] == false);
}

static void test_subscription_expiry() {
    Calls calls;
    VisionControl c(opts(Mode::Yolo, false, 5), calls.fn());
    Subscription s;
    CHECK(c.subscribe("page", "", Pipelines{false, true}, 0, 3000, 1000, s) == VisionControl::Status::Ok);
    CHECK(s.fps == 5 && s.expires_ms == 4000);
    CHECK(c.frame_plan(1000) == (Pipelines{false, true}));
    CHECK(c.keepalive(s.id, 3500, s) == VisionControl::Status::Ok && s.expires_ms == 6500);
    CHECK(c.processing());
    c.frame_plan(6400);
    CHECK(c.processing() && c.subscriptions(6400).size() == 1);
    // no keep-alive: the next frame after expiry drops it and the camera goes back to idle
    CHECK(!c.frame_plan(6600).any());
    CHECK(!c.processing() && c.camera_fps() == VisionControl::kIdleFps && calls.v.back() == std::make_pair(false, 1));
    CHECK(c.keepalive(s.id, 6700, s) == VisionControl::Status::NotFound);
    // re-posting after expiry creates a new subscription
    Subscription s2;
    CHECK(c.subscribe("page", s.id, Pipelines{false, true}, 0, 3000, 7000, s2) == VisionControl::Status::Ok);
    CHECK(s2.id != s.id && c.processing());
    c.expire(10001);
    CHECK(!c.processing());
    // ttl is clamped
    CHECK(c.subscribe("short", "", Pipelines{true, false}, 0, 1, 20000, s) == VisionControl::Status::Ok);
    CHECK(s.ttl_ms == VisionControl::kMinTtlMs && s.expires_ms == 20000 + VisionControl::kMinTtlMs);

    // over HTTP, real clock: a 500 ms subscription without keep-alive expires; one kept alive stays
    VisionControl h(opts(Mode::Yolo, false, 5), nullptr);
    int st = 0;
    json a = req(h, "POST", "/api/subscribe", sub_body("gone", R"(["yolo"])", 0, 500), &st);
    json b = req(h, "POST", "/api/subscribe", sub_body("kept", R"(["aruco"])", 0, 500), &st);
    for (int i = 0; i < 4; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        json k = req(h, "POST", "/api/subscribe/" + b["id"].get<std::string>() + "/keepalive", "{}", &st);
        CHECK(st == 200 && k["expires_in_ms"].get<int>() > 400);
    }
    json l = req(h, "GET", "/api/subscriptions");
    CHECK(l["subscriptions"].size() == 1 && l["subscriptions"][0]["id"] == b["id"]);
    CHECK(l["continuous"]["pipelines"] == json({"aruco"}));
    req(h, "POST", "/api/subscribe/" + a["id"].get<std::string>() + "/keepalive", "{}", &st);
    CHECK(st == 404);
}

static void test_camera_hold() {
    // "pipelines": [] = camera hold: no inference, the camera gate open like a pending trigger
    Calls calls;
    VisionControl c(opts(Mode::Yolo, false, 5), calls.fn());
    int st = 0;
    json m = req(c, "GET", "/api/mode");
    CHECK(m["camera_hold"] == true && m["continuous"]["holds"] == 0);
    json h = req(c, "POST", "/api/subscribe", R"({"client":"armd-mview","pipelines":[],"ttl_ms":5000})", &st);
    CHECK(st == 200 && h["hold"] == true && h["pipelines"] == json::array() && h["ttl_ms"] == 5000);
    CHECK(h["continuous"]["active"] == false && h["continuous"]["holds"] == 1 && h["continuous"]["subscriptions"] == 0);
    CHECK(!c.processing() && c.current_fps() == VisionControl::kIdleFps);
    CHECK(c.camera_fps() == VisionControl::kOpenGateFps && calls.v.back() == std::make_pair(false, VisionControl::kOpenGateFps));
    for (uint64_t t = mono_ms(); t < mono_ms() + 1000; t += 67) CHECK(!c.frame_plan(t).any());  // frames pass, nothing runs
    // a trigger during the hold runs on the first frame not older than the request
    {
        const uint64_t req_ms = mono_ms() + 10;
        auto scope = c.trigger_scope(req_ms, Pipelines{true, false});
        CHECK(c.processing() && c.camera_fps() == VisionControl::kOpenGateFps);
        CHECK(!c.frame_plan(req_ms - 1).any() && c.frame_plan(req_ms) == (Pipelines{true, false}));
    }
    CHECK(!c.processing() && c.camera_fps() == VisionControl::kOpenGateFps);
    // a slow continuous subscription next to the hold: the gate stays open (not 2 x 2 fps)
    json s = req(c, "POST", "/api/subscribe", sub_body("overlay", R"(["aruco"])", 2), &st);
    CHECK(st == 200 && c.processing() && c.camera_fps() == VisionControl::kOpenGateFps);
    CHECK(c.continuous().fps == 2 && c.continuous().subscriptions == 1 && c.continuous().holds == 1);
    json l = req(c, "GET", "/api/subscriptions");
    CHECK(l["subscriptions"].size() == 2 && l["continuous"]["holds"] == 1 && l["continuous"]["pipelines"] == json({"aruco"}));
    // deleting the hold restores the state before it: the subscription's gate, then idle
    req(c, "DELETE", "/api/subscribe/" + h["id"].get<std::string>(), "", &st);
    CHECK(st == 200 && c.camera_fps() == 4 && calls.v.back() == std::make_pair(true, 4));
    req(c, "DELETE", "/api/subscribe/" + s["id"].get<std::string>(), "", &st);
    CHECK(st == 200 && c.camera_fps() == VisionControl::kIdleFps && calls.v.back() == std::make_pair(false, 1));
    // a hold that is not kept alive expires: the gate goes back to idle; a keep-alive extends it
    Subscription k;
    CHECK(c.subscribe("armd-mview", "", Pipelines{}, 0, 1000, 1000, k) == VisionControl::Status::Ok && k.hold());
    CHECK(c.camera_fps() == VisionControl::kOpenGateFps);
    CHECK(c.keepalive(k.id, 1800, k) == VisionControl::Status::Ok && k.expires_ms == 2800);
    c.expire(2700);
    CHECK(c.camera_fps() == VisionControl::kOpenGateFps);
    c.expire(2800);
    CHECK(c.camera_fps() == VisionControl::kIdleFps && !c.processing() && calls.v.back() == std::make_pair(false, 1));
    // a hold does not change what a trigger without pipelines runs
    CHECK(c.default_trigger_pipelines() == (Pipelines{true, false}));
}

static void test_trigger_pipelines() {
    // standby in base mode yolo: each requested set runs exactly, on a fresh frame
    VisionControl c(opts(Mode::Yolo, false, 5), nullptr);
    FakeCamera cam(c, 1000 / VisionControl::kMaxFps);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    CHECK(cam.processed == 0);
    struct Case { const char* body; bool yolo, aruco; };
    for (const Case& k : {Case{R"({"pipelines":["aruco"]})", false, true}, Case{R"({"pipelines":["yolo"]})", true, false},
                          Case{R"({"pipelines":["yolo","aruco"]})", true, true}, Case{"", true, false},
                          Case{R"({"pipelines":["aruco"],"wait_ms":2000})", false, true}}) {
        json r = req(c, "POST", "/api/trigger?wait_ms=2000", k.body);
        CHECK(r["success"] == true && r["api_version"] == 2);
        CHECK(r["pipelines"]["yolo"]["ran"] == k.yolo && r["pipelines"]["aruco"]["ran"] == k.aruco);
        CHECK(r["detections"].empty() != k.yolo && r["markers"].empty() != k.aruco);
        CHECK(r["requested"].size() == static_cast<size_t>(k.yolo + k.aruco));
        CHECK(!c.processing() && c.camera_fps() == VisionControl::kIdleFps);
    }
    int st = 0;
    req(c, "POST", "/api/trigger", R"({"pipelines":"yolo"})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/trigger", R"({"pipelines":["x"]})", &st);
    CHECK(st == 400);
    req(c, "POST", "/api/trigger", "[1]", &st);
    CHECK(st == 400);

    // a trigger while a subscription runs: served by the next frame with its pipelines added
    Subscription s;
    CHECK(c.subscribe("overlay", "", Pipelines{false, true}, 2, 3000, mono_ms(), s) == VisionControl::Status::Ok);
    json first = req(c, "GET", "/api/results/latest?after=0&wait_ms=1000");
    const auto t0 = clk::now();
    json r = req(c, "POST", "/api/trigger?wait_ms=2000", R"({"pipelines":["yolo"]})");
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clk::now() - t0).count();
    CHECK(r["success"] == true && r["pipelines"]["yolo"]["ran"] == true && r["frame_id"] > first["frame_id"]);
    CHECK(ms < 300);  // next frame, not the subscription's 2 fps slot
    CHECK(c.processing() && c.continuous().pipelines == (Pipelines{false, true}));
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    json later = req(c, "GET", "/api/results/latest");
    CHECK(later["pipelines"]["aruco"]["ran"] == true);
    c.unsubscribe(s.id);

    // a result lacking the requested pipelines does not answer the trigger
    VisionControl w(opts(Mode::Yolo, false, 5), nullptr);
    VisionResult only_yolo;
    only_yolo.frame_id = 1;
    only_yolo.frame_ms = 500;
    only_yolo.ran_yolo = true;
    w.publish(only_yolo);
    VisionResult out;
    CHECK(w.wait_fresh(0, 400, Pipelines{false, true}, 30, out));
    CHECK(!w.wait_fresh(0, 400, Pipelines{true, false}, 30, out));

    // without a model: yolo triggers answer 409, aruco ones work
    VisionControl n(opts(Mode::Aruco, false, 5, false), nullptr);
    json e = req(n, "POST", "/api/trigger", R"({"pipelines":["yolo"]})", &st);
    CHECK(st == 409 && e["error"] == "yolo unavailable");
}

static void test_mode_endpoint_compat() {
    // POST /api/mode drives the default subscription; it never expires and joins the union
    VisionControl c(opts(Mode::Yolo, false, 5), nullptr);
    int st = 0;
    json m = req(c, "POST", "/api/mode", R"({"mode":"yolo","enabled":true})", &st);
    CHECK(st == 200 && m["enabled"] == true && m["continuous"]["pipelines"] == json({"yolo"}) && m["continuous"]["subscriptions"] == 0);
    c.expire(mono_ms() + 3600 * 1000);
    CHECK(c.continuous().pipelines == (Pipelines{true, false}));
    Subscription s;
    CHECK(c.subscribe("calib", "", Pipelines{false, true}, 0, 3000, mono_ms(), s) == VisionControl::Status::Ok);
    json l = req(c, "GET", "/api/subscriptions");
    CHECK(l["default"]["enabled"] == true && l["subscriptions"].size() == 1);
    CHECK(l["continuous"]["pipelines"] == json({"yolo", "aruco"}) && l["continuous"]["fps"] == 5);
    // mode off stops only the default subscription
    req(c, "POST", "/api/mode", R"({"mode":"off"})", &st);
    CHECK(c.continuous().pipelines == (Pipelines{false, true}) && c.processing());
    m = req(c, "GET", "/api/mode");
    CHECK(m["mode"] == "off" && m["enabled"] == false && m["processing"] == true);
    // legacy /api/inference toggles the default subscription
    json inf = req(c, "POST", "/api/inference", R"({"enabled":true})");
    CHECK(inf["enabled"] == true && c.continuous().pipelines == (Pipelines{true, true}));
    c.unsubscribe(s.id);
    req(c, "POST", "/api/inference", R"({"enabled":false})");
    CHECK(!c.processing());
}

static std::string http_roundtrip(int port, const std::string& raw) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a;
    std::memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(static_cast<uint16_t>(port));
    if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) {
        ::close(fd);
        return "";
    }
    ::send(fd, raw.data(), raw.size(), 0);
    std::string out;
    char buf[1024];
    while (true) {
        ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        out.append(buf, static_cast<size_t>(n));
    }
    ::close(fd);
    return out;
}

static void test_http_server() {
    const int port = 18091;
    VisionControl c(opts(Mode::Yolo), nullptr);
    HttpControlServer server(c);
    CHECK(server.start(port));
    std::string resp = http_roundtrip(port, "GET /api/mode HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
    CHECK(resp.find("200 OK") != std::string::npos && resp.find("\"mode\":\"yolo\"") != std::string::npos);
    const std::string body = R"({"mode":"aruco"})";
    resp = http_roundtrip(port, "POST /api/mode HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: " +
                                    std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
    CHECK(resp.find("200 OK") != std::string::npos && c.mode() == Mode::Aruco);
    resp = http_roundtrip(port, "POST /api/mode HTTP/1.1\r\nHost: localhost\r\nContent-Type: text/plain\r\nContent-Length: " +
                                    std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
    CHECK(resp.find("415") != std::string::npos);
    const std::string sb = R"({"client":"t","pipelines":["aruco"]})";
    resp = http_roundtrip(port, "POST /api/subscribe HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: " +
                                    std::to_string(sb.size()) + "\r\nConnection: close\r\n\r\n" + sb);
    CHECK(resp.find("200 OK") != std::string::npos && c.processing());
    const std::string id = json::parse(resp.substr(resp.find("\r\n\r\n") + 4))["id"];
    resp = http_roundtrip(port, "DELETE /api/subscribe/" + id + " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
    CHECK(resp.find("200 OK") != std::string::npos && !c.processing());
    // PUT needs JSON like POST (415); other methods are 405
    resp = http_roundtrip(port, "PUT /api/mode HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
    CHECK(resp.find("415") != std::string::npos);
    resp = http_roundtrip(port, "PATCH /api/mode HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
    CHECK(resp.find("405") != std::string::npos);
    server.stop();
}

static void test_status_endpoint() {
    // 1. Detect options: model_kind = "detect", mask = false
    {
        VisionControl::Options o = opts(Mode::Yolo);
        o.model_kind = "detect";
        o.mask = false;
        VisionControl c(o, nullptr);
        int st = 0;
        json j = req(c, "GET", "/api/status", "", &st);
        CHECK(st == 200);
        CHECK(j.contains("model"));
        CHECK(j["model"]["name"] == "yolo11n");
        CHECK(j["model"]["path"] == "/userdata/local/models/yolo11n.cvimodel");
        CHECK(j["model"]["kind"] == "detect");
        CHECK(j["model"]["mask"] == false);
        CHECK(j["model"]["mask_max_boxes"] == 3);
        CHECK(j["mask_focus"].is_null());
    }
    // 2. Segment options: model_kind = "segment", mask = true
    {
        VisionControl::Options o = opts(Mode::Yolo);
        o.model_name = "yolo11n-seg";
        o.model_path = "/userdata/local/models/yolo11n-seg.cvimodel";
        o.model_kind = "segment";
        o.mask = true;
        o.mask_max_boxes = 5;
        VisionControl c(o, nullptr);
        int st = 0;
        json j = req(c, "GET", "/api/status", "", &st);
        CHECK(st == 200);
        CHECK(j.contains("model"));
        CHECK(j["model"]["name"] == "yolo11n-seg");
        CHECK(j["model"]["path"] == "/userdata/local/models/yolo11n-seg.cvimodel");
        CHECK(j["model"]["kind"] == "segment");
        CHECK(j["model"]["mask"] == true);
        CHECK(j["model"]["mask_max_boxes"] == 5);
    }
    // 3. Segment options with mask = false
    {
        VisionControl::Options o = opts(Mode::Yolo);
        o.model_kind = "segment";
        o.mask = false;
        VisionControl c(o, nullptr);
        int st = 0;
        json j = req(c, "GET", "/api/status", "", &st);
        CHECK(st == 200);
        CHECK(j["model"]["kind"] == "segment");
        CHECK(j["model"]["mask"] == false);
    }
}

static void test_mask_focus_control() {
    VisionControl::Options o = opts(Mode::Yolo);
    VisionControl c(o, nullptr);
    int st = 0;

    // 1. Initial status has mask_focus = null
    json j0 = req(c, "GET", "/api/status", "", &st, 1000000);
    CHECK(st == 200 && j0["mask_focus"].is_null());

    // 2. Persistent focus POST sets focus with TTL
    json set_body = {
        {"label", "target_box"},
        {"box_norm", {0.1f, 0.2f, 0.3f, 0.4f}},
        {"ttl_ms", 500}
    };
    json post_res = req(c, "POST", "/api/mask_focus", set_body.dump(), &st, 1000000);
    CHECK(st == 200);
    CHECK(!post_res["mask_focus"].is_null());
    CHECK(post_res["mask_focus"]["label"] == "target_box");
    CHECK(post_res["mask_focus"]["ttl_ms"] == 500);
    CHECK(post_res["mask_focus"]["remaining_ttl_ms"] == 500);

    // 3. Status during active TTL shows focus and remaining TTL
    json j_mid = req(c, "GET", "/api/status", "", &st, 1000200);
    CHECK(st == 200);
    CHECK(!j_mid["mask_focus"].is_null());
    CHECK(j_mid["mask_focus"]["remaining_ttl_ms"] == 300);

    // GET /api/mask_focus also shows focus
    json get_mf = req(c, "GET", "/api/mask_focus", "", &st, 1000200);
    CHECK(st == 200);
    CHECK(!get_mf["mask_focus"].is_null());
    CHECK(get_mf["mask_focus"]["label"] == "target_box");

    // 4. TTL expiry: after 500 ms (at t = 1000501), focus is expired
    json j_exp = req(c, "GET", "/api/status", "", &st, 1000600);
    CHECK(st == 200);
    CHECK(j_exp["mask_focus"].is_null());

    json get_exp = req(c, "GET", "/api/mask_focus", "", &st, 1000600);
    CHECK(st == 200);
    CHECK(get_exp["mask_focus"].is_null());

    // 5. POST /api/mask_focus {} clears focus immediately
    req(c, "POST", "/api/mask_focus", set_body.dump(), &st, 2000000);
    json j_active = req(c, "GET", "/api/status", "", &st, 2000100);
    CHECK(!j_active["mask_focus"].is_null());

    json clear_res = req(c, "POST", "/api/mask_focus", "{}", &st, 2000150);
    CHECK(st == 200);
    CHECK(clear_res["mask_focus"].is_null());
    json j_cleared = req(c, "GET", "/api/status", "", &st, 2000200);
    CHECK(j_cleared["mask_focus"].is_null());

    // 6. POST /api/mask_focus {"ttl_ms": 0} also clears focus
    req(c, "POST", "/api/mask_focus", set_body.dump(), &st, 3000000);
    json clear_res2 = req(c, "POST", "/api/mask_focus", "{\"ttl_ms\": 0}", &st, 3000100);
    CHECK(st == 200);
    CHECK(clear_res2["mask_focus"].is_null());
    json j_cleared2 = req(c, "GET", "/api/status", "", &st, 3000200);
    CHECK(j_cleared2["mask_focus"].is_null());

    // 7. Trigger-only focus does not persist
    MaskFocus trig_focus;
    trig_focus.active = true;
    trig_focus.label = "trig_target";
    trig_focus.box_norm[0] = 0.2f;
    trig_focus.box_norm[1] = 0.2f;
    trig_focus.box_norm[2] = 0.4f;
    trig_focus.box_norm[3] = 0.4f;

    c.trigger_begin(4000100, Pipelines{true, false}, &trig_focus);

    // Frame that serves the trigger
    Pipelines triggered;
    MaskFocus frame1_focus;
    Pipelines p1 = c.frame_plan(4000150, &triggered, &frame1_focus);
    CHECK(p1.yolo);
    CHECK(frame1_focus.active == true);
    CHECK(frame1_focus.label == "trig_target");

    // Next frame: trigger focus does NOT persist
    MaskFocus frame2_focus;
    Pipelines p2 = c.frame_plan(4000200, &triggered, &frame2_focus);
    (void)p2;
    CHECK(frame2_focus.active == false);

    // Persistent focus was never set
    MaskFocus status_focus;
    CHECK(c.get_mask_focus(4000250, status_focus) == false);
    json j_after_trig = req(c, "GET", "/api/status", "", &st, 4000250);
    CHECK(j_after_trig["mask_focus"].is_null());

    c.trigger_end(4000100);
}

static void test_depth_unavailable() {
    VisionControl c(opts(Mode::Off, false, 10, true, /*depth=*/false), nullptr);
    int st = 0;
    json e = req(c, "POST", "/api/trigger", R"({"pipelines":["depth"]})", &st);
    CHECK(st == 409 && e["error"] == "depth unavailable" && e["detail"] == "the depth model did not load");

    req(c, "POST", "/api/subscribe", R"({"client":"sub1","pipelines":["depth"]})", &st);
    CHECK(st == 409);

    json m = req(c, "GET", "/api/mode");
    CHECK(m["available"]["depth"] == false);
    CHECK(m["pipelines"]["depth"] == false);
    CHECK(m["depth_model"].is_null());
}

static void test_depth_trigger_and_status() {
    VisionControl c(opts(Mode::Off, false, 10, true, /*depth=*/true), nullptr);
    json m = req(c, "GET", "/api/mode");
    CHECK(m["available"]["depth"] == true);
    CHECK(!m["depth_model"].is_null());
    CHECK(m["depth_model"]["name"] == "zipdepth_base_384x224_bf16_rgb");
    CHECK(m["depth_model"]["metric"] == false);

    FakeCamera cam(c, 20);
    int st = 0;
    json r = req(c, "POST", "/api/trigger?wait_ms=2000", R"({"pipelines":["depth"]})", &st);
    CHECK(st == 200 && r["success"] == true);
    CHECK(r["requested"] == json({"depth"}));
    CHECK(r["pipelines"]["depth"]["ran"] == true);
    CHECK(r.contains("depth"));
    CHECK(r["depth"]["grid_w"] == 384);
    CHECK(r["depth"]["grid_h"] == 224);
    CHECK(r["depth"]["metric_model"] == false);
    CHECK(r["depth"]["table_samples"].is_array());
    CHECK(r["depth"]["table_samples"].size() == 1);
    CHECK(r["depth"]["table_samples"][0][0] == 320.0);
    CHECK(r["depth"]["table_samples"][0][2] == 0.5);
}

static void test_detections_json_depth() {
    DetBox d;
    d.x = 0.5f; d.y = 0.5f; d.w = 0.2f; d.h = 0.2f;
    d.confidence = 0.85f;
    d.class_id = 1;
    d.class_name = "cube";
    d.disp_fg = 0.75f;
    d.disp_bg = 0.40f;
    d.depth_conf = 0.92f;
    d.length_px = 54.3f;
    d.jaw_clear = 1;

    ContentRect cr{0, 0, 640, 360};
    std::string s = detections_json({d}, cr, 640, 360);
    json j = json::parse(s);
    CHECK(j.is_array() && j.size() == 1);
    CHECK(std::abs(j[0]["disp_fg"].get<float>() - 0.75f) < 1e-3);
    CHECK(std::abs(j[0]["disp_bg"].get<float>() - 0.40f) < 1e-3);
    CHECK(std::abs(j[0]["depth_conf"].get<float>() - 0.92f) < 1e-3);
    CHECK(std::abs(j[0]["length_px"].get<float>() - 54.3f) < 1e-1);
    CHECK(j[0]["jaw_clear"] == true);

    d.jaw_clear = 0;
    s = detections_json({d}, cr, 640, 360);
    j = json::parse(s);
    CHECK(j[0]["jaw_clear"] == false);

    d.jaw_clear = -1;
    s = detections_json({d}, cr, 640, 360);
    j = json::parse(s);
    CHECK(!j[0].contains("jaw_clear"));
}

// A plain detection model with depth: the depth PCA angle/width are reported as depth_* fields,
// angle_deg / angle_conf / grasp_width_px stay absent. A seg model keeps its mask values.
static void test_detections_json_depth_angle_split() {
    const int grid_w = 384, grid_h = 224;
    const float cw = 640.0f, ch = 360.0f;
    std::vector<float> dmap(grid_w * grid_h, 0.40f);
    for (int gy = 0; gy < grid_h; ++gy) {
        const float v = (gy + 0.5f) * (ch / grid_h);
        for (int gx = 0; gx < grid_w; ++gx) {
            const float u = (gx + 0.5f) * (cw / grid_w);
            if (u >= 280.0f && u <= 360.0f && v >= 172.0f && v <= 188.0f) dmap[gy * grid_w + gx] = 0.85f;
        }
    }
    DetBox d;
    d.x = 320.0f / 640.0f; d.y = 180.0f / 360.0f; d.w = 100.0f / 640.0f; d.h = 30.0f / 360.0f;
    d.confidence = 0.8f;
    d.class_name = "block";
    CHECK(std::isnan(jaw_close_deg(d)));
    DepthBox box{270.0f, 165.0f, 370.0f, 195.0f};
    BoxDepthResult r = analyze_box_depth(dmap.data(), grid_w, grid_h, box, {}, cw, ch, jaw_close_deg(d));
    CHECK(r.valid);
    apply_box_depth(d, r);
    ContentRect cr{0, 0, 640, 360};
    json j = json::parse(detections_json({d}, cr, 640, 360));
    CHECK(!j[0].contains("angle_deg"));
    CHECK(!j[0].contains("angle_conf"));
    CHECK(!j[0].contains("grasp_width_px"));
    CHECK(j[0].contains("depth_angle_deg") && std::abs(j[0]["depth_angle_deg"].get<float>()) < 1.0f);
    CHECK(j[0].contains("depth_angle_conf") && j[0]["depth_angle_conf"].get<float>() > 0.0f);
    CHECK(j[0].contains("depth_width_px") && std::abs(j[0]["depth_width_px"].get<float>() - r.grasp_width_px) < 0.01f);
    CHECK(j[0].contains("depth_conf") && j[0].contains("length_px") && j[0].contains("jaw_clear"));

    // seg model: mask angle 30 deg, width 20 px -> unchanged; jaws close at 120 deg
    DetBox s = d;
    s.angle_deg = 30.0f; s.angle_conf = 0.9f; s.grasp_width_px = 20.0f;
    s.depth_angle_deg = s.depth_angle_conf = s.depth_width_px = NAN;
    CHECK(std::abs(jaw_close_deg(s) - 120.0f) < 1e-4f);
    apply_box_depth(s, r);
    j = json::parse(detections_json({s}, cr, 640, 360));
    CHECK(std::abs(j[0]["angle_deg"].get<float>() - 30.0f) < 1e-3f);
    CHECK(std::abs(j[0]["grasp_width_px"].get<float>() - 20.0f) < 1e-3f);
    CHECK(j[0].contains("depth_angle_deg"));

    // OBB model: angle_deg = -25.0f, angle_conf = 0.85f, grasp_width_px = 24.0f, length_px = 68.0f, has_obb = true, finite obb_corners
    DetBox obb;
    obb.x = 0.5f; obb.y = 0.5f; obb.w = 0.1f; obb.h = 0.2f;
    obb.confidence = 0.9f;
    obb.class_name = "block";
    obb.angle_deg = -25.0f;
    obb.angle_conf = 0.85f;
    obb.grasp_width_px = 24.0f;
    obb.length_px = 68.0f;
    obb.has_obb = true;
    obb.obb_corners = {{{0.45f, 0.40f}, {0.55f, 0.40f}, {0.55f, 0.60f}, {0.45f, 0.60f}}};
    CHECK(std::abs(jaw_close_deg(obb) - 65.0f) < 1e-4f);
    apply_box_depth(obb, r);
    CHECK(std::abs(obb.angle_deg - (-25.0f)) < 1e-4f);
    CHECK(std::abs(obb.angle_conf - 0.85f) < 1e-4f);
    CHECK(std::abs(obb.grasp_width_px - 24.0f) < 1e-4f);
    CHECK(std::abs(obb.length_px - 68.0f) < 1e-4f);
    CHECK(obb.has_obb);
    CHECK(obb.obb_corners[0][0] == 0.45f && obb.obb_corners[0][1] == 0.40f);
    CHECK(obb.obb_corners[1][0] == 0.55f && obb.obb_corners[1][1] == 0.40f);
    CHECK(obb.obb_corners[2][0] == 0.55f && obb.obb_corners[2][1] == 0.60f);
    CHECK(obb.obb_corners[3][0] == 0.45f && obb.obb_corners[3][1] == 0.60f);
    CHECK(std::isfinite(obb.depth_angle_deg));
    CHECK(std::isfinite(obb.depth_angle_conf));
    CHECK(std::isfinite(obb.depth_width_px));
    CHECK(obb.jaw_clear >= 0);
    j = json::parse(detections_json({obb}, cr, 640, 360));
    CHECK(j[0].contains("obb_corners"));
    CHECK(std::abs(j[0]["angle_deg"].get<float>() - (-25.0f)) < 1e-3f);
    CHECK(std::abs(j[0]["grasp_width_px"].get<float>() - 24.0f) < 1e-3f);
    CHECK(std::abs(j[0]["length_px"].get<float>() - 68.0f) < 1e-1f);
    CHECK(j[0].contains("depth_angle_deg"));
    CHECK(j[0].contains("depth_angle_conf"));
    CHECK(j[0].contains("depth_width_px"));
    CHECK(j[0].contains("jaw_clear"));

    // invalid depth result: nothing written
    DetBox e;
    apply_box_depth(e, BoxDepthResult{});
    j = json::parse(detections_json({e}, cr, 640, 360));
    CHECK(!j[0].contains("depth_angle_deg") && !j[0].contains("depth_conf"));
}

static void test_detections_json_obb() {
    DetBox d;
    d.x = 0.5f; d.y = 0.5f; d.w = 0.1f; d.h = 0.2f;
    d.confidence = 0.90f;
    d.class_id = 39;
    d.class_name = "bottle";
    d.has_obb = true;
    d.obb_corners = {{
        {0.45f, 0.40f},
        {0.55f, 0.40f},
        {0.55f, 0.60f},
        {0.45f, 0.60f}
    }};
    d.angle_deg = -15.5f;
    d.angle_conf = 0.82f;
    d.grasp_width_px = 30.0f;
    d.length_px = 75.0f;

    ContentRect cr{0, 0, 640, 360};
    std::string s = detections_json({d}, cr, 640, 360);
    json j = json::parse(s);
    CHECK(j.is_array() && j.size() == 1);
    CHECK(j[0].contains("obb_corners"));
    CHECK(!j[0].contains("corners"));
    CHECK(!j[0].contains("polygon"));
    CHECK(j[0]["obb_corners"].size() == 4);
    CHECK(std::abs(j[0]["obb_corners"][0][0].get<float>() - 0.45f) < 1e-3);
    CHECK(std::abs(j[0]["obb_corners"][0][1].get<float>() - 0.40f) < 1e-3);
    CHECK(std::abs(j[0]["length_px"].get<float>() - 75.0f) < 1e-1);

    d.has_obb = false;
    s = detections_json({d}, cr, 640, 360);
    j = json::parse(s);
    CHECK(!j[0].contains("obb_corners"));
}

static void test_vision_control_obb_kind() {
    VisionControl::Options o = opts(Mode::Yolo, false, 10, true, false);
    o.model_kind = "obb";
    VisionControl c(o, nullptr);
    json r = req(c, "GET", "/api/mode");
    CHECK(r["model"]["kind"] == "obb");
}

static void test_model_swap() {
    VisionControl::Options o = opts(Mode::Yolo, false, 10, true, true);
    o.model_name = "det";
    o.depth_model_name = "depth";
    o.depth_metric = true;
    std::vector<std::string> w;
    parse_catalogue(std::string(R"({"models": {
        "det": {"path": "/m/det.cvimodel", "kind": "detect", "ion_mb": 8.5},
        "seg": {"path": "/m/seg.cvimodel", "kind": "segment", "ion_mb": 19.5},
        "depth": {"path": "/m/depth.cvimodel", "kind": "depth", "ion_mb": 20.5}}})"),
                    o.catalogue, w);
    Calls calls;
    VisionControl c(o, calls.fn());
    json m = req(c, "GET", "/api/mode");
    CHECK(m["models"]["generation"] == 0 && m["models"]["swap"]["state"] == "idle" && m["models"]["swap"]["generation"] == 0);
    CHECK(m["models"]["resident"] == json({{"detector", "det"}, {"depth", "depth"}}));
    // catalogue in name order (vision.json object keys)
    CHECK(m["models"]["catalogue"].size() == 3 && m["models"]["catalogue"][1] ==
          json({{"name", "det"}, {"kind", "detect"}, {"path", "/m/det.cvimodel"}, {"ion_mb", 8.5}, {"resident", true}}));
    CHECK(m["models"]["catalogue"][0]["resident"] == true && m["models"]["catalogue"][2]["resident"] == false);
    // depth entries report their metric flag (here from the file name: relative)
    CHECK(m["models"]["catalogue"][0]["metric"] == false && !m["models"]["catalogue"][1].contains("metric"));
    CHECK(m["ion"]["used_mb"] == 50.5 && m["ion"]["total_mb"] == 62.9 && m["ion"]["reserve_mb"] == 8.0);

    // seg next to the resident depth model: over the budget, nothing changes
    int st = 0;
    json e = req(c, "POST", "/api/models", R"({"detector":"seg"})", &st);
    CHECK(st == 409 && e["error"] == "ION_BUDGET" && e["need_mb"] == 69.5 && e["limit_mb"] == 62.9);
    e = req(c, "POST", "/api/models", R"({"detector":"nope"})", &st);
    CHECK(st == 400 && e["error"] == "no model named \"nope\"");
    CHECK(c.swap_state() == SwapState::Idle);

    // accepted at once (202), a second request is busy; the pending swap opens the camera gate
    const size_t n_calls = calls.v.size();
    e = req(c, "POST", "/api/models", R"({"detector":"seg","depth":null})", &st);
    CHECK(st == 202 && e["generation"] == 1);
    CHECK(calls.v.size() == n_calls + 1 && calls.v.back().second == VisionControl::kOpenGateFps);
    e = req(c, "POST", "/api/models", R"({"depth":"depth"})", &st);
    CHECK(st == 409 && e["error"] == "SWAP_BUSY");
    m = req(c, "GET", "/api/mode");
    CHECK(m["models"]["swap"]["state"] == "pending" && m["models"]["swap"]["generation"] == 1 && m["models"]["generation"] == 0);

    // the frame thread takes it, loads, publishes
    SwapRequest s;
    CHECK(c.take_swap(s) && s.detector.name == "seg" && s.depth.change && s.depth.name.empty());
    CHECK(c.swap_state() == SwapState::Running && !c.take_swap(s));
    CHECK(calls.v.back().second == VisionControl::kIdleFps);  // gate back to idle once taken
    ModelSet now = c.models();
    now.model_name = "seg";
    now.model_kind = "segment";
    now.mask = true;
    now.depth_available = false;
    now.depth_model_name.clear();
    CHECK(c.finish_swap(now, "") == 1);
    m = req(c, "GET", "/api/mode");
    CHECK(m["models"]["generation"] == 1 && m["models"]["swap"]["state"] == "idle" && !m["models"]["swap"].contains("error"));
    CHECK(m["model"]["kind"] == "segment" && m["available"]["depth"] == false && m["depth_model"].is_null());
    CHECK(m["models"]["resident"] == json({{"detector", "seg"}, {"depth", nullptr}}));
    req(c, "POST", "/api/trigger", R"({"pipelines":["depth"]})", &st);
    CHECK(st == 409);

    // a failed load: that slot stays empty, the generation still advances, the state says why
    e = req(c, "POST", "/api/models", R"({"detector":"det","depth":"depth"})", &st);
    CHECK(st == 202 && e["generation"] == 2);
    CHECK(c.take_swap(s));
    now = c.models();
    now.model_name = "det";
    now.model_kind = "detect";
    CHECK(c.finish_swap(now, "LOAD_FAILED:depth") == 2);
    m = req(c, "GET", "/api/mode");
    CHECK(m["models"]["swap"] == json({{"state", "failed"}, {"generation", 2}, {"error", "LOAD_FAILED:depth"}}));

    // detector unloaded: yolo is unavailable to triggers and modes, results carry the generation
    e = req(c, "POST", "/api/models", R"({"detector":null})", &st);
    CHECK(st == 202 && c.take_swap(s));
    now = c.models();
    now.yolo_available = false;
    now.model_name.clear();
    now.model_kind.clear();  // main.cpp set_detector(nullptr)
    c.finish_swap(now, "");
    CHECK(!c.yolo_available());
    m = req(c, "GET", "/api/mode");
    CHECK(m["model"]["kind"].is_null() && m["available"]["yolo"] == false && m["models"]["resident"]["detector"].is_null());
    req(c, "POST", "/api/trigger", R"({"pipelines":["yolo"]})", &st);
    CHECK(st == 409);
    req(c, "POST", "/api/mode", R"({"mode":"both"})", &st);
    CHECK(st == 409);
    VisionResult r;
    r.frame_id = 1;
    r.model_generation = 3;
    c.publish(r);
    json l = req(c, "GET", "/api/results/latest");
    CHECK(l["model_generation"] == 3);

    // shutdown drops a pending swap
    req(c, "POST", "/api/models", R"({"detector":"det"})", &st);
    CHECK(st == 202 && c.swap_state() == SwapState::Pending);
    c.shutdown();
    CHECK(c.swap_state() == SwapState::Idle && !c.take_swap(s));
}

int main() {
    test_model_swap();
    test_modes();
    test_status_endpoint();
    test_yolo_unavailable();
    test_depth_unavailable();
    test_depth_trigger_and_status();
    test_detections_json_depth();
    test_detections_json_depth_angle_split();
    test_detections_json_obb();
    test_vision_control_obb_kind();
    test_trigger_each_mode();
    test_trigger_timeout_and_continuous();
    test_rate_gate();
    test_rate_model();
    test_trigger_gate_and_freshness();
    test_trigger_latency();
    test_latest();
    test_result_json();
    test_standby();
    test_subscription_union();
    test_subscription_expiry();
    test_camera_hold();
    test_trigger_pipelines();
    test_mode_endpoint_compat();
    test_mask_focus_control();
    test_http_server();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("vision_control: all checks passed\n");
    return 0;
}
