// Modified from sscma-example-sg200x solutions/aruco-detector/tests/inference_control_test.cpp
// (commit 9af8118, Apache-2.0): modes, shared rate, trigger in every mode, combined envelope.
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
                int* status = nullptr) {
    HttpReply r = handle_control_request(c, m, t, b, 1000100);
    if (status) *status = r.status;
    return json::parse(r.body);
}

struct Calls {
    std::vector<std::pair<bool, int>> v;
    VisionControl::OnChange fn() {
        return [this](bool on, int fps) { v.push_back({on, fps}); };
    }
};

static VisionControl::Options opts(Mode m, bool enabled = false, int fps = 10, bool yolo = true) {
    VisionControl::Options o;
    o.mode = m;
    o.enabled = enabled;
    o.fps = fps;
    o.yolo_available = yolo;
    o.model_name = "yolo11n";
    o.model_path = "/userdata/local/models/yolo11n.cvimodel";
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
            r.detections_json = p.yolo ? R"([{"class_id":39,"class_name":"bottle","confidence":0.7,"x":0.5,"y":0.5,"w":0.1,"h":0.2}])" : "[]";
            r.markers_json = p.aruco ? R"([{"id":3,"center":[320.0,180.0]}])" : "[]";
            r.count = p.aruco ? 1 : 0;
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
        // back to standby, continuous state untouched
        CHECK(!c.processing() && !c.continuous_enabled() && c.mode() == m);
        CHECK(c.current_fps() == 1);
        CHECK(calls.v.back() == std::make_pair(false, 1));
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
    CHECK(c.wait_fresh(0, 200, 30, out));  // timeout
    std::thread th([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        VisionResult r;
        r.frame_id = 2;
        r.frame_ms = 250;
        c.publish(r);
    });
    CHECK(!c.wait_fresh(0, 200, 2000, out) && out.frame_id == 2);
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
    v.detections_json = "[]";
    v.markers_json = R"([{"id":1}])";
    v.count = 1;
    c.publish(v);
    r = req(c, "GET", "/api/results/latest");
    CHECK(r["frame_id"] == 7 && r["age_ms"] == 100 && !r.contains("timeout"));
    CHECK(r["mode"] == "both" && r["family"] == "aruco4x4_50" && r["count"] == 1);
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
    CHECK(j.size() == 1 && j[0]["class_name"] == "bottle" && j[0]["class_id"] == 39);
    CHECK(std::abs(j[0]["x"].get<double>() - 0.5) < 1e-3 && std::abs(j[0]["y"].get<double>() - 0.5) < 1e-3);
    CHECK(std::abs(j[0]["h"].get<double>() - 0.32) < 1e-3 && std::abs(j[0]["w"].get<double>() - 0.1) < 1e-3);
    // top of the content band (model y = 140/640) maps to 0
    d[0].y = 140.0f / 640.0f;
    j = json::parse(detections_json(d, content_rect(640, 640, 1280, 720), 640, 640));
    CHECK(std::abs(j[0]["y"].get<double>()) < 1e-3);

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
    server.stop();
}

int main() {
    test_modes();
    test_yolo_unavailable();
    test_trigger_each_mode();
    test_trigger_timeout_and_continuous();
    test_rate_gate();
    test_rate_model();
    test_trigger_gate_and_freshness();
    test_trigger_latency();
    test_latest();
    test_result_json();
    test_http_server();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("vision_control: all checks passed\n");
    return 0;
}
