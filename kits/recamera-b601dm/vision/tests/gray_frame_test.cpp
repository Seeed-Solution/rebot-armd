// Host tests for gray_frame.{h,cpp}, VisionControl gray contract, and HttpControlServer sockets.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "gray_frame.h"
#include "http_control.h"
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

static GrayFrame make_golden_frame() {
    GrayFrame f;
    f.instance_id = "rv-golden-0001";
    f.frame_id = 7;
    f.width = kGrayWidth;
    f.height = kGrayHeight;
    f.stride = kGrayWidth;
    f.frame_w = 640;
    f.frame_h = 640;
    f.crop_x = 0;
    f.crop_y = 140;
    f.crop_w = kGrayWidth;
    f.crop_h = kGrayHeight;
    f.channel = 0;
    f.ts_ms = 1763791234567ull;
    f.frame_mono_ms = 123450;
    f.wait_mono_ms = 123401;
    f.grab_mono_ms = 123452;
    f.stored_mono_ms = 123459;
    f.convert_ms = 4.9f;
    f.model_generation = 3;
    auto px = std::make_shared<std::vector<uint8_t>>(static_cast<size_t>(kGrayWidth) * static_cast<size_t>(kGrayHeight));
    for (int y = 0; y < kGrayHeight; ++y) {
        for (int x = 0; x < kGrayWidth; ++x) {
            (*px)[static_cast<size_t>(y) * kGrayWidth + static_cast<size_t>(x)] =
                static_cast<uint8_t>((x * 3 + y * 5 + 17) & 0xFF);
        }
    }
    f.pixels = std::move(px);
    return f;
}

static void test_rgb_primaries_and_stride() {
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
    set_px(0, 0, 0, 0, 0);
    set_px(1, 0, 255, 255, 255);
    set_px(2, 0, 255, 0, 0);
    set_px(3, 0, 0, 255, 0);
    set_px(4, 0, 0, 0, 255);
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

    CHECK(!rgb888_to_gray(nullptr, w, h, stride, out.data()));
    CHECK(!rgb888_to_gray(rgb.data(), w, h, stride, nullptr));
    CHECK(!rgb888_to_gray(rgb.data(), 0, h, stride, out.data()));
    CHECK(!rgb888_to_gray(rgb.data(), w, -1, stride, out.data()));
    CHECK(!rgb888_to_gray(rgb.data(), w, h, w * 3 - 1, out.data()));
}

static void test_crop_edges_mutation_and_failure() {
    const int fw = 640, fh = 640, fstride = fw * 3 + 16;
    std::vector<uint8_t> frame(static_cast<size_t>(fh) * fstride, 0);
    for (int y = 140; y < 500; ++y) {
        uint8_t* row = frame.data() + static_cast<size_t>(y) * fstride;
        for (int x = 0; x < fw; ++x) {
            uint8_t v = 120;
            if (y == 140) v = 200;
            else if (y == 499) v = 220;
            if (x == 0) v = 180;
            if (x == 639) v = 240;
            row[3 * x] = v;
            row[3 * x + 1] = v;
            row[3 * x + 2] = v;
        }
    }
    std::memset(frame.data() + 139u * fstride, 11, fw * 3);
    std::memset(frame.data() + 500u * fstride, 22, fw * 3);

    std::vector<uint8_t> gray;
    CHECK(extract_gray_crop(frame.data(), fw, fh, fstride, 0, 140, 640, 360, gray));
    CHECK(gray.size() == 640u * 360u);
    CHECK(gray[0] == 180 && gray[1] == 200 && gray[639] == 240);
    CHECK(gray[359u * 640u + 0] == 180 && gray[359u * 640u + 1] == 220 && gray[359u * 640u + 639] == 240);
    CHECK(gray[100u * 640u + 100] == 120);

    std::fill(frame.begin(), frame.end(), 0);
    CHECK(gray[1] == 200 && gray[359u * 640u + 1] == 220 && gray[100u * 640u + 100] == 120);

    std::vector<uint8_t> keep = {1, 2, 3};
    CHECK(!extract_gray_crop(frame.data(), fw, fh, fstride, -1, 140, 640, 360, keep));
    CHECK(!extract_gray_crop(frame.data(), fw, fh, fstride, 0, 300, 640, 360, keep));
    CHECK(!extract_gray_crop(frame.data(), fw, fh, fw * 3 - 3, 0, 140, 640, 360, keep));
    CHECK(keep == std::vector<uint8_t>({1, 2, 3}));
}

static void test_rgf1_roundtrip_and_golden() {
    const GrayFrame f = make_golden_frame();
    const std::string hdr = gray_frame_header(f);
    CHECK(hdr.size() >= 8 && hdr.compare(0, 4, "RGF1") == 0);
    uint32_t n = 0;
    for (int b = 0; b < 4; ++b) n |= static_cast<uint32_t>(static_cast<uint8_t>(hdr[4 + b])) << (8 * b);
    CHECK(hdr.size() == 8u + n && (8u + n) % kGrayFrameAlign == 0);

    const std::string body = gray_frame_body(f);
    CHECK(body.size() == hdr.size() + f.pixels->size());

    std::string js;
    std::vector<uint8_t> back;
    CHECK(parse_gray_frame_body(body, js, back));
    CHECK(back == *f.pixels);
    const json j = json::parse(js);
    CHECK(j["api_version"] == 2 && j["encoding"] == "RGF1" && j["format"] == "gray8");
    CHECK(j["instance_id"] == "rv-golden-0001" && j["frame_id"] == 7);
    CHECK(j["width"] == 640 && j["height"] == 360 && j["stride"] == 640);
    CHECK(j["crop"] == json({0, 140, 640, 360}) && j["frame_size"] == json({640, 640}));
    CHECK(j["luma"] == "(77*R+150*G+29*B)>>8" && j["luma_range"] == "full_0_255");

    std::string bad_magic = body;
    bad_magic[0] = 'X';
    CHECK(!parse_gray_frame_body(bad_magic, js, back));
    CHECK(!parse_gray_frame_body(body.substr(0, body.size() - 1), js, back));
    CHECK(!parse_gray_frame_body(body + "X", js, back));
    CHECK(!parse_gray_frame_body(body.substr(0, 6), js, back));

#ifdef GOLDEN_RGF1_PATH
    std::ifstream ifs(GOLDEN_RGF1_PATH, std::ios::binary);
    CHECK(ifs.good());
    if (ifs.good()) {
        std::string disk((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        CHECK(disk == body);
    }
#endif
}

static void test_ring_sparse_eviction_and_instance() {
    GrayRing ring(4);
    const std::string inst = "rv-inst1";
    std::shared_ptr<const GrayFrame> out;

    CHECK(ring.find(inst, 1, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find("rv-other", 1, inst, out) == GrayRing::Lookup::Expired);

    CHECK(ring.put(make_gray(inst, 10, 640, 360, 10)) == nullptr);
    CHECK(ring.put(make_gray(inst, 20, 640, 360, 20)) == nullptr);
    CHECK(ring.put(make_gray(inst, 30, 640, 360, 30)) == nullptr);
    CHECK(ring.put(make_gray(inst, 40, 640, 360, 40)) == nullptr);
    CHECK(!ring.has_evicted() && ring.size() == 4);
    CHECK(ring.find(inst, 5, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 15, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 50, inst, out) == GrayRing::Lookup::NotStored);
    CHECK(ring.find(inst, 10, inst, out) == GrayRing::Lookup::Found && out->pixels->at(0) == 10);
    CHECK(ring.find("rv-old", 10, inst, out) == GrayRing::Lookup::Expired && !out);

    auto disp = ring.put(make_gray(inst, 50, 640, 360, 50));
    CHECK(disp != nullptr && disp->frame_id == 10);
    CHECK(ring.has_evicted() && ring.highest_evicted_id() == 10);
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

    auto valid_gray = make_gray(inst0, bad.frame_id, 640, 360, 77);
    ctl.publish(bad, valid_gray);
    CHECK(!ctl.latest(false, 0, 0, got) && got.ran_gray && got.instance_id == inst0);
    CHECK(ctl.get_gray(inst0, bad.frame_id, gf) == GrayRing::Lookup::Found && gf->pixels->at(0) == 77);

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

    const uint64_t t_req = 50000;
    {
        auto s1 = ctl.trigger_scope(t_req, Pipelines{true, false, false, false});
        auto s2 = ctl.trigger_scope(t_req, Pipelines{false, true, false, true});
        Pipelines p = ctl.frame_plan(t_req);
        CHECK(p.yolo && p.aruco && p.gray && !p.depth);

        s1.release();
        p = ctl.frame_plan(t_req + 1);
        CHECK(!p.yolo && p.aruco && p.gray);
    }
    CHECK(!ctl.frame_plan(t_req + 2).any());

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

static void test_http_contract_peer_errors_and_subscriptions() {
    VisionControl::Options o;
    o.instance_id = "rv-http1";
    o.max_gray_sends = 2;
    VisionControl ctl(o, nullptr);
    const RequestPeer lo{true};
    const RequestPeer remote{false};

    // Peer-aware /api/mode
    json m_lo = json::parse(handle_control_request(ctl, "GET", "/api/mode", "", 1000, nullptr, lo).body);
    CHECK(m_lo["available"]["gray"] == true);
    CHECK(m_lo["gray_frames"]["encoding"] == "RGF1" && m_lo["gray_frames"]["format"] == "gray8");
    CHECK(m_lo["gray_frames"]["width"] == 640 && m_lo["gray_frames"]["height"] == 360 && m_lo["gray_frames"]["stride"] == 640);
    CHECK(m_lo["gray_frames"]["ring"] == 4 && m_lo["gray_frames"]["exposure_qualified"] == false);

    json m_rem = json::parse(handle_control_request(ctl, "GET", "/api/mode", "", 1000, nullptr, remote).body);
    CHECK(m_rem["available"]["gray"] == false && m_rem["gray_frames"].is_null());

    // Unsupported geometry disables gray even for loopback
    VisionControl::Options o_nogray = o;
    o_nogray.gray_available = false;
    VisionControl ctl_ng(o_nogray, nullptr);
    json m_ng = json::parse(handle_control_request(ctl_ng, "GET", "/api/mode", "", 1000, nullptr, lo).body);
    CHECK(m_ng["available"]["gray"] == false && m_ng["gray_frames"].is_null());
    HttpReply r_ng = handle_control_request(ctl_ng, "POST", "/api/trigger", R"({"pipelines":["gray"]})", 1000, nullptr, lo);
    CHECK(r_ng.status == 409);

    // Subscriptions reject "gray"
    HttpReply sub1 = handle_control_request(ctl, "POST", "/api/subscribe", R"({"client":"c1","pipelines":["gray"]})", 1000, nullptr, lo);
    HttpReply sub2 = handle_control_request(ctl, "POST", "/api/subscribe", R"({"client":"c1","pipelines":["yolo","gray"]})", 1000, nullptr, lo);
    CHECK(sub1.status == 400 && sub2.status == 400);

    // Remote peer receives 403 on gray trigger & gray GET
    HttpReply rem_trig = handle_control_request(ctl, "POST", "/api/trigger", R"({"pipelines":["yolo","gray"]})", 1000, nullptr, remote);
    CHECK(rem_trig.status == 403 && json::parse(rem_trig.body)["error"] == "FORBIDDEN");
    HttpReply rem_get = handle_control_request(ctl, "GET", "/api/frame/gray?instance_id=rv-http1&frame_id=1", "", 1000, nullptr, remote);
    CHECK(rem_get.status == 403 && json::parse(rem_get.body)["error"] == "FORBIDDEN");

    // Bad query parameters -> 400
    for (const char* q : {
             "/api/frame/gray",
             "/api/frame/gray?",
             "/api/frame/gray?frame_id=1",
             "/api/frame/gray?instance_id=rv-http1",
             "/api/frame/gray?instance_id=rv-http1&frame_id=1&frame_id=1",
             "/api/frame/gray?instance_id=rv-http1&instance_id=rv-http1&frame_id=1",
             "/api/frame/gray?instance_id=rv-http1&frame_id=1&wait_ms=100",
             "/api/frame/gray?instance_id=rv-http1&frame_id=4294967296",
             "/api/frame/gray?instance_id=rv-http1&frame_id=-1",
             "/api/frame/gray?instance_id=&frame_id=1",
             "/api/frame/gray?instance_id=bad/id&frame_id=1",
             "/api/frame/gray?instance_id=rv-http1&frame_id=abc",
         }) {
        HttpReply r = handle_control_request(ctl, "GET", q, "", 1000, nullptr, lo);
        CHECK(r.status == 400);
    }

    // 404 before eviction, 200 for retained, 410 after eviction & on wrong instance
    for (uint32_t i = 1; i <= 5; ++i) {
        VisionResult vr;
        vr.frame_id = ctl.next_frame_id(vr.instance_id);
        vr.frame_ms = 2000 + i;
        vr.ran_yolo = true;
        vr.ran_gray = true;
        ctl.publish(vr, make_gray(vr.instance_id, vr.frame_id, 640, 360, static_cast<uint8_t>(i * 10)));
    }
    HttpReply r410 = handle_control_request(ctl, "GET", "/api/frame/gray?instance_id=rv-http1&frame_id=1", "", 2010, nullptr, lo);
    CHECK(r410.status == 410 && json::parse(r410.body)["error"] == "FRAME_EXPIRED");
    HttpReply r410_inst = handle_control_request(ctl, "GET", "/api/frame/gray?instance_id=rv-other&frame_id=5", "", 2010, nullptr, lo);
    CHECK(r410_inst.status == 410 && json::parse(r410_inst.body)["error"] == "FRAME_EXPIRED");
    HttpReply r404 = handle_control_request(ctl, "GET", "/api/frame/gray?instance_id=rv-http1&frame_id=99", "", 2010, nullptr, lo);
    CHECK(r404.status == 404 && json::parse(r404.body)["error"] == "FRAME_NOT_STORED");
    HttpReply r200 = handle_control_request(ctl, "GET", "/api/frame/gray?instance_id=rv-http1&frame_id=5", "", 2010, nullptr, lo);
    CHECK(r200.status == 200 && r200.content_type == "application/octet-stream");
    CHECK(r200.binary_segment && r200.binary_segment->size() == 640u * 360u && r200.binary_segment->at(0) == 50);
    CHECK(ctl.active_gray_sends() == 1);
    r200 = HttpReply{};
    CHECK(ctl.active_gray_sends() == 0);
}

static int connect_loopback(int port, int rcvbuf = 0) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    if (rcvbuf > 0) {
        ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    }
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(static_cast<uint16_t>(port));
    if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

static std::string http_roundtrip(int port, const std::string& raw, size_t chunk_bytes = 65536, int pause_ms = 0) {
    int fd = connect_loopback(port);
    if (fd < 0) return "";
    ::send(fd, raw.data(), raw.size(), MSG_NOSIGNAL);
    std::string out;
    std::vector<char> buf(chunk_bytes);
    for (;;) {
        const ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
        if (n <= 0) break;
        out.append(buf.data(), static_cast<size_t>(n));
        if (pause_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(pause_ms));
    }
    ::close(fd);
    return out;
}

static void test_http_server_sockets() {
    VisionControl::Options o;
    o.instance_id = "rv-sock";
    o.max_gray_sends = 1;
    VisionControl ctl(o, nullptr);

    // Simulated frame thread responding to triggers (including gray and old-client yolo-only triggers)
    std::atomic<bool> stop_cam{false};
    std::thread cam([&] {
        while (!stop_cam.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            const uint64_t fms = mono_ms();
            const Pipelines p = ctl.frame_plan(fms);
            if (!p.any()) continue;
            VisionResult vr;
            vr.frame_id = ctl.next_frame_id(vr.instance_id);
            vr.frame_ms = fms;
            vr.ts_ms = 1763791234000ull + vr.frame_id;
            vr.ran_yolo = p.yolo;
            vr.ran_aruco = p.aruco;
            vr.ran_depth = p.depth;
            std::shared_ptr<const GrayFrame> gf;
            if (p.gray) {
                gf = make_gray(vr.instance_id, vr.frame_id, 640, 360, static_cast<uint8_t>((vr.frame_id * 13) & 0xFF));
                vr.ran_gray = true;
                vr.gray_ms = 4.2f;
            }
            ctl.publish(vr, std::move(gf));
        }
    });

    HttpControlServer server(ctl);
    server.set_send_deadline_ms(300);
    CHECK(server.start(0));
    const int port = server.bound_port();
    CHECK(port > 0);

    // 1. Old-client trigger (no gray) works over TCP
    {
        const std::string body = R"({"pipelines":["yolo"],"wait_ms":1000})";
        const std::string resp = http_roundtrip(
            port, "POST /api/trigger HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: " +
                      std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
        CHECK(resp.find("HTTP/1.1 200 OK") == 0);
        const size_t sep = resp.find("\r\n\r\n");
        CHECK(sep != std::string::npos);
        json j = json::parse(resp.substr(sep + 4));
        CHECK(j["success"] == true && j["pipelines"]["yolo"]["ran"] == true && j["pipelines"]["gray"]["ran"] == false);
        CHECK(j["instance_id"] == "rv-sock");
    }

    // 2. Gray trigger + complete read + partial-chunk read over TCP
    uint32_t gray_fid = 0;
    {
        const std::string body = R"({"pipelines":["yolo","gray"],"wait_ms":1000})";
        const std::string resp = http_roundtrip(
            port, "POST /api/trigger HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: " +
                      std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
        CHECK(resp.find("HTTP/1.1 200 OK") == 0);
        const size_t sep = resp.find("\r\n\r\n");
        json j = json::parse(resp.substr(sep + 4));
        CHECK(j["success"] == true && j["pipelines"]["gray"]["ran"] == true && j["instance_id"] == "rv-sock");
        gray_fid = j["frame_id"].get<uint32_t>();
    }
    const std::string get_req = "GET /api/frame/gray?instance_id=rv-sock&frame_id=" + std::to_string(gray_fid) +
                                " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n";
    {
        const std::string resp = http_roundtrip(port, get_req);
        CHECK(resp.find("HTTP/1.1 200 OK") == 0);
        CHECK(resp.find("Content-Type: application/octet-stream\r\n") != std::string::npos);
        CHECK(resp.find("Cache-Control: no-store\r\n") != std::string::npos);
        const size_t sep = resp.find("\r\n\r\n");
        std::string js;
        std::vector<uint8_t> px;
        CHECK(parse_gray_frame_body(resp.substr(sep + 4), js, px));
        CHECK(px.size() == 640u * 360u && px[0] == static_cast<uint8_t>((gray_fid * 13) & 0xFF));
    }
    // Partial chunk reads (4 KB chunks with 1 ms pause)
    {
        const std::string resp = http_roundtrip(port, get_req, 4096, 1);
        const size_t sep = resp.find("\r\n\r\n");
        std::string js;
        std::vector<uint8_t> px;
        CHECK(sep != std::string::npos && parse_gray_frame_body(resp.substr(sep + 4), js, px));
        CHECK(px.size() == 640u * 360u);
    }

    // 3. 410 Gone reason phrase over TCP
    {
        const std::string resp = http_roundtrip(
            port, "GET /api/frame/gray?instance_id=rv-stale&frame_id=1 HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
        CHECK(resp.find("HTTP/1.1 410 Gone\r\n") == 0);
        CHECK(resp.find("Cache-Control: no-store\r\n") != std::string::npos);
    }

    // 4. Admission limit (503 GRAY_BUSY + Retry-After) while /api/mode stays responsive
    {
        auto held = ctl.try_acquire_gray_send();
        CHECK(held != nullptr && ctl.active_gray_sends() == 1);
        const std::string busy = http_roundtrip(port, get_req);
        CHECK(busy.find("HTTP/1.1 503 Service Unavailable\r\n") == 0);
        CHECK(busy.find("Retry-After: 1\r\n") != std::string::npos);
        CHECK(busy.find("GRAY_BUSY") != std::string::npos);
        const std::string mode_ok = http_roundtrip(
            port, "GET /api/mode HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
        CHECK(mode_ok.find("HTTP/1.1 200 OK\r\n") == 0);
        held.reset();
        CHECK(ctl.active_gray_sends() == 0);
    }

    // 5. Client disconnect mid-transfer cleans up promptly
    {
        int fd = connect_loopback(port, 4096);
        CHECK(fd >= 0);
        ::send(fd, get_req.data(), get_req.size(), MSG_NOSIGNAL);
        char tiny[64];
        (void)::recv(fd, tiny, sizeof(tiny), 0);
        ::close(fd);
        // Wait for server handler to finish & release send slot
        for (int i = 0; i < 50 && ctl.active_gray_sends() != 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        CHECK(ctl.active_gray_sends() == 0);
    }

    // 6. Send deadline with a stalled reader (large synthetic frame so TCP send buffer fills)
    {
        auto big = std::make_shared<GrayFrame>();
        big->instance_id = "rv-sock";
        big->frame_id = ctl.next_frame_id(big->instance_id);
        big->width = 4096;
        big->height = 1024;
        big->stride = 4096;
        big->pixels = std::make_shared<std::vector<uint8_t>>(4096u * 1024u, 0x33);
        VisionResult vr;
        vr.instance_id = big->instance_id;
        vr.frame_id = big->frame_id;
        vr.frame_ms = mono_ms();
        vr.ran_gray = true;
        ctl.publish(vr, big);

        int stalled = connect_loopback(port, 4096);
        CHECK(stalled >= 0);
        const std::string big_req = "GET /api/frame/gray?instance_id=rv-sock&frame_id=" +
                                    std::to_string(big->frame_id) +
                                    " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n";
        const auto t0 = std::chrono::steady_clock::now();
        ::send(stalled, big_req.data(), big_req.size(), MSG_NOSIGNAL);

        // Wait until slot is taken or server finishes
        bool saw_active = false;
        for (int i = 0; i < 30; ++i) {
            if (ctl.active_gray_sends() > 0) {
                saw_active = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        // Control plane (/api/mode) stays responsive even while a stalled gray transfer is in flight
        const std::string mode_resp = http_roundtrip(
            port, "GET /api/mode HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n");
        CHECK(mode_resp.find("HTTP/1.1 200 OK\r\n") == 0);

        // Wait for deadline (300 ms) to abort the stalled transfer and release the slot
        for (int i = 0; i < 100 && ctl.active_gray_sends() != 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        CHECK(ctl.active_gray_sends() == 0);
        if (saw_active) {
            CHECK(elapsed_ms < 1500);
        }
        ::close(stalled);
    }

    // 7. Server shutdown completes promptly even with a stalled connection open
    {
        int idle_client = connect_loopback(port);
        CHECK(idle_client >= 0);
        const auto t0 = std::chrono::steady_clock::now();
        server.stop();
        const auto stop_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        CHECK(stop_ms < 1000);
        ::close(idle_client);
    }

    stop_cam.store(true);
    cam.join();
}

static void test_depth_map_waiter_intervening_publication() {
    VisionControl::Options opt;
    opt.depth_available = true;
    opt.depth_metric = true;
    opt.depth_model_name = "depth_anything_v2_metric";
    opt.instance_id = "rv-depth";
    VisionControl ctl(opt, nullptr);

    // 1. A depth-only result lacking depth_disp followed by a depth result WITH depth_disp,
    //    immediately overwritten in latest_ by an intervening YOLO-only frame before the waiter
    //    finishes reading: /api/depth must still return 200 with the valid depth map (not 503).
    HttpReply reply;
    std::thread waiter([&] {
        reply = handle_control_request(ctl, "GET", "/api/depth?wait_ms=1000", "", 5000, nullptr, RequestPeer{true});
    });

    // Wait until /api/depth registers its trigger
    for (int i = 0; i < 100 && !ctl.depth_map_wanted(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(ctl.depth_map_wanted());
    const uint64_t t_now = mono_ms() + 1;

    // Intervening depth result without depth_disp (must be skipped by map waiter, not return 503)
    {
        VisionResult no_map;
        no_map.frame_id = ctl.next_frame_id(no_map.instance_id);
        no_map.frame_ms = t_now;
        no_map.ran_depth = true;
        no_map.width = 640;
        no_map.height = 360;
        ctl.publish(no_map);
    }

    // Valid depth map frame immediately followed by a non-depth publication overwriting latest_
    uint32_t expected_depth_id = 0;
    {
        VisionResult with_map;
        with_map.frame_id = ctl.next_frame_id(with_map.instance_id);
        expected_depth_id = with_map.frame_id;
        with_map.frame_ms = t_now + 1;
        with_map.ran_depth = true;
        with_map.depth_ms = 12.0f;
        with_map.width = 640;
        with_map.height = 360;
        with_map.depth_grid_w = 4;
        with_map.depth_grid_h = 4;
        with_map.depth_disp = std::make_shared<std::vector<float>>(16, 0.5f);
        ctl.publish(with_map);

        VisionResult intervening;
        intervening.frame_id = ctl.next_frame_id(intervening.instance_id);
        intervening.frame_ms = t_now + 2;
        intervening.ran_yolo = true;
        intervening.width = 640;
        intervening.height = 360;
        ctl.publish(intervening);
    }

    waiter.join();
    CHECK(reply.status == 200);
    CHECK(reply.content_type == "application/octet-stream");
    std::string djs;
    std::vector<uint16_t> dmm;
    CHECK(parse_depth_map_body(reply.body, djs, dmm));
    CHECK(dmm.size() == 16);
    CHECK(djs.find("\"frame_id\":" + std::to_string(expected_depth_id)) != std::string::npos);

    // 2. Direct wait_fresh(require_depth_map=true) after an intervening non-depth publication
    //    already overwrote latest_ still selects latest_depth_map_ with its intact payload.
    VisionResult direct;
    const bool timed_out = ctl.wait_fresh(0, t_now, Pipelines{false, false, true, false}, 100, direct,
                                          /*require_depth_map=*/true);
    CHECK(!timed_out);
    CHECK(direct.frame_id == expected_depth_id);
    CHECK(direct.depth_disp != nullptr && direct.depth_disp->size() == 16);
}

static void test_gray_allocation_failure_after_conversion_and_noexcept_ring() {
    static_assert(noexcept(std::declval<GrayRing&>().put(std::shared_ptr<const GrayFrame>{})),
                  "GrayRing::put must be noexcept");

    // 1. Allocation failure AFTER RGB888 -> gray8 conversion succeeds (bad_alloc from record
    //    allocator): make_gray_frame must catch it and return nullptr without leaving partial state.
    const std::vector<uint8_t> rgb(640u * 640u * 3u, 0x55);
    GrayFrameMeta meta;
    meta.instance_id = "rv-alloc";
    meta.frame_id = 1;
    meta.ts_ms = 1763791234000;
    meta.frame_mono_ms = 100;
    meta.wait_mono_ms = 90;
    meta.grab_mono_ms = 101;
    meta.stored_mono_ms = 105;

    bool alloc_called = false;
    auto failing_alloc = [&]() -> std::shared_ptr<GrayFrame> {
        alloc_called = true;
        throw std::bad_alloc();
    };
    auto failed_rec = make_gray_frame(rgb.data(), 640, 640, 640 * 3, 0, 140, 640, 360, meta, failing_alloc);
    CHECK(alloc_called);
    CHECK(failed_rec == nullptr);

    // Also test null return from record allocator
    auto null_rec = make_gray_frame(rgb.data(), 640, 640, 640 * 3, 0, 140, 640, 360, meta,
                                    []() -> std::shared_ptr<GrayFrame> { return nullptr; });
    CHECK(null_rec == nullptr);

    // 2. Normal path succeeds and publishes atomically; failed record leaves ring & ran_gray clean.
    VisionControl::Options opt;
    opt.instance_id = "rv-alloc";
    VisionControl ctl(opt, nullptr);
    VisionResult vr;
    vr.instance_id = "rv-alloc";
    vr.frame_id = ctl.next_frame_id(vr.instance_id);
    vr.frame_ms = mono_ms();
    vr.ran_gray = (failed_rec != nullptr);
    ctl.publish(vr, failed_rec);
    VisionResult got;
    ctl.latest(false, 0, 0, got);
    CHECK(!got.ran_gray);
    std::shared_ptr<const GrayFrame> found;
    CHECK(ctl.get_gray("rv-alloc", vr.frame_id, found) == GrayRing::Lookup::NotStored);

    // 3. Wraparound of pre-allocated recent_results_ ring (> kRecentResultsCap) preserves newest matches.
    for (int i = 0; i < 12; ++i) {
        meta.frame_id = ctl.next_frame_id(meta.instance_id);
        auto ok_rec = make_gray_frame(rgb.data(), 640, 640, 640 * 3, 0, 140, 640, 360, meta);
        CHECK(ok_rec != nullptr && ok_rec->valid());
        VisionResult r;
        r.instance_id = meta.instance_id;
        r.frame_id = meta.frame_id;
        r.frame_ms = mono_ms() + 10 + static_cast<uint64_t>(i);
        r.ran_gray = (i == 9);  // only one recent gray frame before 2 non-gray frames
        r.ran_yolo = true;
        ctl.publish(r, r.ran_gray ? ok_rec : nullptr);
    }
    VisionResult matched;
    CHECK(!ctl.wait_fresh(0, mono_ms(), Pipelines{false, false, false, true}, 50, matched));
    CHECK(matched.ran_gray && matched.frame_id == 11);
}

int main(int argc, char** argv) {
    if (argc == 3 && std::strcmp(argv[1], "--write-golden") == 0) {
        const std::string body = gray_frame_body(make_golden_frame());
        std::ofstream ofs(argv[2], std::ios::binary | std::ios::trunc);
        if (!ofs.good()) return 2;
        ofs.write(body.data(), static_cast<std::streamsize>(body.size()));
        ofs.close();
        std::printf("wrote golden RGF1 (%zu bytes) -> %s\n", body.size(), argv[2]);
        return 0;
    }
    test_rgb_primaries_and_stride();
    test_crop_edges_mutation_and_failure();
    test_rgf1_roundtrip_and_golden();
    test_ring_sparse_eviction_and_instance();
    test_atomic_publish_failed_gray_and_rollover();
    test_same_ms_trigger_handles_and_waiter_interleaving();
    test_http_contract_peer_errors_and_subscriptions();
    test_http_server_sockets();
    test_depth_map_waiter_intervening_publication();
    test_gray_allocation_failure_after_conversion_and_noexcept_ring();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("gray_frame: all checks passed\n");
    return 0;
}
