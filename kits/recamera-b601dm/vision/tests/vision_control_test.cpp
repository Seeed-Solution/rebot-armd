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

/** Simulated frame thread: runs frame_pipelines() at `period`, like on_frame. */
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
            const Pipelines p = c_.frame_pipelines();
            if (!p.any()) continue;
            VisionResult r;
            r.frame_id = ++id;
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
    CHECK(!c.frame_pipelines().any());

    json m = req(c, "GET", "/api/mode");
    CHECK(m["api_version"] == kApiVersion);
    CHECK(m["mode"] == "yolo" && m["enabled"] == false && m["fps"] == 1 && m["active_fps"] == 10);
    CHECK(m["available"]["yolo"] == true && m["available"]["aruco"] == true && m["trigger"] == true);
    CHECK(m["modes"].size() == 4);

    // mode + enabled in one call; rate follows
    m = req(c, "POST", "/api/mode", R"({"mode":"both","enabled":true})");
    CHECK(m["mode"] == "both" && m["enabled"] == true && m["fps"] == 10);
    CHECK(c.frame_pipelines() == (Pipelines{true, true}));
    CHECK(calls.v.back() == std::make_pair(true, 10));

    // switching between active modes does not touch the camera rate
    const size_t n = calls.v.size();
    CHECK(c.set_mode(Mode::Aruco) == VisionControl::Status::Ok);
    CHECK(calls.v.size() == n);
    CHECK(c.frame_pipelines() == (Pipelines{false, true}));

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
    CHECK(calls.v.back() == std::make_pair(true, 5));

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
