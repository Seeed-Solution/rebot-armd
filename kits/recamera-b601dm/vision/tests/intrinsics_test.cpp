// GET/PUT /api/calibration/intrinsics: validation, vision.json merge and atomic write,
// loopback check, CLI override, revision handling and the frame-thread apply.
#include <arpa/inet.h>
#include <dirent.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "http_control.h"
#include "intrinsics.h"
#include "json.hpp"
#include "vision_control.h"

using namespace rv;
using json = nlohmann::json;
using ojson = nlohmann::ordered_json;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

static std::string g_dir;

static std::string read_file(const std::string& p) {
    std::string s;
    bool missing = false;
    read_text_file(p, s, missing);
    return s;
}

static void write_file(const std::string& p, const std::string& s) {
    FILE* f = std::fopen(p.c_str(), "wb");
    std::fwrite(s.data(), 1, s.size(), f);
    std::fclose(f);
}

static std::vector<std::string> dir_entries(const std::string& d) {
    std::vector<std::string> v;
    DIR* dp = ::opendir(d.c_str());
    while (dirent* e = dp ? ::readdir(dp) : nullptr) {
        if (std::strcmp(e->d_name, ".") && std::strcmp(e->d_name, "..")) v.push_back(e->d_name);
    }
    if (dp) ::closedir(dp);
    return v;
}

static json good_body(uint64_t rev = 1) {
    return {{"fx", 830.5}, {"fy", 829.25}, {"cx", 321.0}, {"cy", 182.5}, {"k1", -0.12}, {"k2", 0.05},
            {"p1", 0.001}, {"p2", -0.002}, {"k3", 0.0},  {"width", 640},  {"height", 360},
            {"marker_size", 0.05}, {"expected_revision", rev}};
}

static void test_validation_table() {
    Intrinsics cur;
    cur.marker_size = 0.042;
    struct Case {
        const char* name;
        json body;  // null: delete `key`
        const char* key;
        const char* field;  // expected error field; nullptr = accepted
    };
    auto with = [](const char* k, json v) {
        json b = good_body();
        b[k] = v;
        return b;
    };
    auto without = [](const char* k) {
        json b = good_body();
        b.erase(k);
        return b;
    };
    const std::vector<Case> cases = {
        {"good", good_body(), "", nullptr},
        {"fx low edge", with("fx", 100), "", nullptr},
        {"fx high edge", with("fx", 5000), "", nullptr},
        {"fx below", with("fx", 99.9), "", "fx"},
        {"fx above", with("fx", 5000.1), "", "fx"},
        {"fx string", with("fx", "830"), "", "fx"},
        {"fx missing", without("fx"), "", "fx"},
        {"fy below", with("fy", 50), "", "fy"},
        {"cx zero", with("cx", 0), "", "cx"},
        {"cx width", with("cx", 640), "", "cx"},
        {"cx negative", with("cx", -1), "", "cx"},
        {"cx inside", with("cx", 639.9), "", nullptr},
        {"cy height", with("cy", 360), "", "cy"},
        {"cy zero", with("cy", 0.0), "", "cy"},
        {"k1 edge", with("k1", -10), "", nullptr},
        {"k1 above", with("k1", 10.5), "", "k1"},
        {"k2 below", with("k2", -11), "", "k2"},
        {"k3 above", with("k3", 12), "", "k3"},
        {"p1 edge", with("p1", 1), "", nullptr},
        {"p1 above", with("p1", 1.01), "", "p1"},
        {"p2 below", with("p2", -1.5), "", "p2"},
        {"p2 missing", without("p2"), "", "p2"},
        {"width 1280", with("width", 1280), "", "width"},
        {"width missing", without("width"), "", "width"},
        {"height 720", with("height", 720), "", "height"},
        {"height float ok", with("height", 360.0), "", nullptr},
        {"marker zero", with("marker_size", 0), "", "marker_size"},
        {"marker above", with("marker_size", 1.5), "", "marker_size"},
        {"marker one", with("marker_size", 1), "", nullptr},
        {"marker null", with("marker_size", nullptr), "", "marker_size"},
        {"marker absent", without("marker_size"), "", nullptr},
        {"rev missing", without("expected_revision"), "", "expected_revision"},
        {"rev negative", with("expected_revision", -1), "", "expected_revision"},
        {"rev float", with("expected_revision", 1.5), "", "expected_revision"},
        {"unknown key", with("fxx", 1), "", "fxx"},
        {"read-only keys ignored", with("revision", 7), "", nullptr},
        {"config_path ignored", with("config_path", "/x"), "", nullptr},
    };
    for (const Case& c : cases) {
        Intrinsics out;
        uint64_t rev = 0;
        std::string field, msg;
        const bool ok = parse_intrinsics_put(c.body.dump(), cur, out, rev, field, msg);
        if (c.field == nullptr) {
            if (!ok) std::printf("case '%s': unexpected error %s: %s\n", c.name, field.c_str(), msg.c_str());
            CHECK(ok);
        } else {
            if (ok || field != c.field) std::printf("case '%s': ok=%d field='%s' (want '%s')\n", c.name, ok, field.c_str(), c.field);
            CHECK(!ok && field == c.field);
            CHECK(msg.find(c.field) != std::string::npos);
        }
    }
    // values and the marker_size default
    Intrinsics out;
    uint64_t rev = 0;
    std::string field, msg;
    json b = good_body(5);
    b.erase("marker_size");
    CHECK(parse_intrinsics_put(b.dump(), cur, out, rev, field, msg));
    CHECK(rev == 5 && out.cam.fx == 830.5 && out.cam.cy == 182.5 && out.cam.k1 == -0.12 && out.cam.p2 == -0.002);
    CHECK(out.marker_size == 0.042);
    // an overflowing literal is rejected by the JSON parser (no inf reaches the checks); non-object bodies
    std::string big = good_body().dump();
    big.replace(big.find("830.5"), 5, "1e999");
    CHECK(!parse_intrinsics_put(big, cur, out, rev, field, msg) && field.empty());
    CHECK(!parse_intrinsics_put("[1,2]", cur, out, rev, field, msg) && field.empty());
    CHECK(!parse_intrinsics_put("{bad", cur, out, rev, field, msg) && field.empty());
}

static Intrinsics sample() {
    Intrinsics in;
    in.cam.fx = 824.125;
    in.cam.fy = 823.5;
    in.cam.cx = 319.75;
    in.cam.cy = 181.0;
    in.cam.k1 = -0.1;
    in.cam.k2 = 0.02;
    in.cam.p1 = 0.0;
    in.cam.p2 = 0.0;
    in.cam.k3 = 0.0;
    in.marker_size = 0.0485;
    return in;
}

static void test_merge() {
    const Intrinsics in = sample();
    // the same content in two formattings; unknown and nested keys must survive with their values
    const std::string pretty = R"({
  "mode": "yolo",
  "fps": 5,
  "fx": 600,
  "conf_threshold": 0.25,
  "ids": "0,1,2,3,4,5,6,7,8,9,10,11",
  "future_key": {"nested": [1, 2.5, "x", null, true]},
  "models": [{"name": "yolo11n", "path": "/userdata/rebot-teleop/models/yolo11n.cvimodel", "ion_mb": 9.5}],
  "big": 18446744073709551615,
  "neg": -3
})";
    std::string compact = ojson::parse(pretty).dump();
    for (const std::string& src : {pretty, compact}) {
        std::string out, err;
        CHECK(merge_intrinsics_config(src, in, out, err));
        const ojson before = ojson::parse(src);
        const ojson after = ojson::parse(out);
        for (auto it = before.begin(); it != before.end(); ++it) {
            if (it.key() == "fx") continue;
            CHECK(after.contains(it.key()) && after[it.key()] == it.value());
        }
        CHECK(after["big"].is_number_unsigned() && after["big"].get<uint64_t>() == 18446744073709551615ULL);
        CHECK(after["fps"].is_number_integer());
        CHECK(after["fx"] == 824.125 && after["fy"] == 823.5 && after["cx"] == 319.75 && after["cy"] == 181.0);
        CHECK(after["k1"] == -0.1 && after["k2"] == 0.02 && after["p1"] == 0.0 && after["p2"] == 0.0 && after["k3"] == 0.0);
        CHECK(after["marker_size"] == 0.0485);
        // key order of the existing file kept; new keys appended
        std::vector<std::string> keys;
        for (auto it = after.begin(); it != after.end(); ++it) keys.push_back(it.key());
        CHECK(keys.size() == before.size() + 9);  // fy cx cy k1 k2 p1 p2 k3 marker_size added
        CHECK(keys[0] == "mode" && keys[1] == "fps" && keys[2] == "fx" && keys[3] == "conf_threshold");
        CHECK(keys[8] == "neg" && keys[9] == "fy");
        // idempotent: merging again changes nothing
        std::string again;
        CHECK(merge_intrinsics_config(out, in, again, err) && again == out);
    }
    std::string out, err;
    CHECK(merge_intrinsics_config("", in, out, err) && ojson::parse(out).size() == 10);
    CHECK(merge_intrinsics_config("  \n", in, out, err));
    out = "unchanged";
    CHECK(!merge_intrinsics_config("[1]", in, out, err) && out == "unchanged" && !err.empty());
    CHECK(!merge_intrinsics_config("{\"a\":", in, out, err) && out == "unchanged");
}

static void test_atomic_write() {
    const std::string d = g_dir + "/atomic";
    ::mkdir(d.c_str(), 0755);
    const std::string p = d + "/vision.json";
    std::string err;
    CHECK(write_file_atomic(p, "{\"a\":1}\n", err));
    CHECK(read_file(p) == "{\"a\":1}\n");
    CHECK(write_file_atomic(p, "{\"a\":2}\n", err));
    CHECK(read_file(p) == "{\"a\":2}\n");
    const std::vector<std::string> e = dir_entries(d);
    CHECK(e.size() == 1 && e[0] == "vision.json");  // no temporary file left
    struct stat st{};
    CHECK(::stat(p.c_str(), &st) == 0 && (st.st_mode & 0777) == 0644);
    // missing directory: error, nothing created
    CHECK(!write_file_atomic(g_dir + "/no/such/dir/vision.json", "{}", err) && !err.empty());
    // persist into a missing file creates it; a non-object file is left alone
    const std::string q = d + "/new.json";
    CHECK(persist_intrinsics(q, sample(), err));
    CHECK(json::parse(read_file(q))["fx"] == 824.125);
    write_file(q, "not json");
    CHECK(!persist_intrinsics(q, sample(), err) && read_file(q) == "not json");
}

static void test_loopback() {
    auto v4 = [](const char* ip) {
        sockaddr_in a{};
        a.sin_family = AF_INET;
        inet_pton(AF_INET, ip, &a.sin_addr);
        return is_loopback_address(reinterpret_cast<sockaddr*>(&a), sizeof(a));
    };
    auto v6 = [](const char* ip) {
        sockaddr_in6 a{};
        a.sin6_family = AF_INET6;
        inet_pton(AF_INET6, ip, &a.sin6_addr);
        return is_loopback_address(reinterpret_cast<sockaddr*>(&a), sizeof(a));
    };
    CHECK(v4("127.0.0.1"));
    CHECK(v4("127.8.9.10"));
    CHECK(!v4("192.168.42.1"));
    CHECK(!v4("10.0.0.127"));
    CHECK(!v4("0.0.0.0"));
    CHECK(!v4("128.0.0.1"));
    CHECK(v6("::1"));
    CHECK(v6("::ffff:127.0.0.1"));
    CHECK(!v6("::ffff:192.168.42.1"));
    CHECK(!v6("fe80::1"));
    CHECK(!v6("::"));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    CHECK(!is_loopback_address(reinterpret_cast<sockaddr*>(&a), 4));  // truncated
    CHECK(!is_loopback_address(nullptr, 0));
    sockaddr u{};
    u.sa_family = AF_UNIX;
    CHECK(!is_loopback_address(&u, sizeof(u)));
    // PUT goes through the same JSON check as POST
    CHECK(precheck_request("PUT", "application/json") == 0);
    CHECK(precheck_request("PUT", "Application/JSON; charset=utf-8") == 0);
    CHECK(precheck_request("PUT", "text/plain") == 415);
    CHECK(precheck_request("PUT", "") == 415);
    CHECK(precheck_request("OPTIONS", "") == 405);
    CHECK(precheck_request("PATCH", "application/json") == 405);
}

static VisionControl::Options opts(const std::string& config, bool cli = false) {
    VisionControl::Options o;
    o.mode = Mode::Aruco;
    o.yolo_available = false;
    o.intrinsics.cam.fx = 600;
    o.intrinsics.cam.fy = 600;
    o.intrinsics.cam.cx = 320;
    o.intrinsics.cam.cy = 180;
    o.intrinsics.marker_size = 0.05;
    o.intrinsics_cli_override = cli;
    o.config_path = config;
    return o;
}

static HttpReply put(VisionControl& c, const json& body, bool loopback = true) {
    RequestPeer peer;
    peer.loopback = loopback;
    return handle_control_request(c, "PUT", "/api/calibration/intrinsics", body.dump(), 0, nullptr, peer);
}

/** Stand-in for the frame thread: applies pending intrinsics every 20 ms. */
struct FrameThread {
    explicit FrameThread(VisionControl& c) : c_(c), t_([this] {
        while (!stop_) {
            Intrinsics in;
            uint64_t rev = 0;
            if (c_.take_intrinsics(in, rev)) {
                applied_fx = in.cam.fx;
                c_.finish_intrinsics(rev);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }) {}
    ~FrameThread() {
        stop_ = true;
        t_.join();
    }
    VisionControl& c_;
    std::atomic<bool> stop_{false};
    std::atomic<double> applied_fx{0};
    std::thread t_;
};

static void test_endpoint() {
    const std::string cfg = g_dir + "/vision.json";
    write_file(cfg, "{\"mode\":\"aruco\",\"fx\":600,\"ids\":\"0,1\",\"x_unknown\":{\"keep\":[1,2]}}");
    VisionControl c(opts(cfg), nullptr);

    HttpReply r = handle_control_request(c, "GET", "/api/calibration/intrinsics", "");
    CHECK(r.status == 200);
    json g = json::parse(r.body);
    CHECK(g["fx"] == 600.0 && g["cx"] == 320.0 && g["width"] == 640 && g["height"] == 360);
    CHECK(g["marker_size"] == 0.05 && g["revision"] == 1 && g["overridden_by_cli"] == false && g["config_path"] == cfg);
    for (const char* k : {"fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "width", "height", "marker_size",
                          "revision", "overridden_by_cli", "config_path"})
        CHECK(g.contains(k));
    // /api/mode carries the same object
    json m = json::parse(handle_control_request(c, "GET", "/api/mode", "").body);
    json gi = g;
    gi.erase("api_version");
    CHECK(m["intrinsics"] == gi);

    // non-loopback peer: 403, nothing written
    const std::string before = read_file(cfg);
    r = put(c, good_body(1), false);
    CHECK(r.status == 403 && json::parse(r.body)["error"] == "FORBIDDEN");
    CHECK(read_file(cfg) == before && c.intrinsics().revision == 1);

    // validation error: 400 with the field
    json bad = good_body(1);
    bad["cy"] = 400;
    r = put(c, bad);
    CHECK(r.status == 400);
    json e = json::parse(r.body);
    CHECK(e["error"] == "INVALID_FIELD" && e["field"] == "cy");
    r = put(c, json::array());
    CHECK(r.status == 400 && json::parse(r.body)["field"].is_null());

    // stale revision: 409
    r = put(c, good_body(0));
    CHECK(r.status == 409);
    e = json::parse(r.body);
    CHECK(e["error"] == "REVISION_MISMATCH" && e["revision"] == 1 && e["expected_revision"] == 0);
    CHECK(read_file(cfg) == before);

    {
        FrameThread ft(c);
        r = put(c, good_body(1));
        CHECK(r.status == 200);
        json n = json::parse(r.body);
        CHECK(n["revision"] == 2 && n["fx"] == 830.5 && n["k1"] == -0.12 && n["config_path"] == cfg);
        CHECK(ft.applied_fx == 830.5);
        // persisted: new values, other keys kept
        json f = json::parse(read_file(cfg));
        CHECK(f["fx"] == 830.5 && f["fy"] == 829.25 && f["p2"] == -0.002 && f["marker_size"] == 0.05);
        CHECK(f["mode"] == "aruco" && f["ids"] == "0,1" && f["x_unknown"] == json({{"keep", {1, 2}}}));
        // the GET body equals the PUT reply
        CHECK(json::parse(handle_control_request(c, "GET", "/api/calibration/intrinsics", "").body) == n);
        // the old revision is stale now
        r = put(c, good_body(1));
        CHECK(r.status == 409 && json::parse(r.body)["revision"] == 2);
        json b3 = good_body(2);
        b3["fx"] = 900;
        b3.erase("marker_size");
        r = put(c, b3);
        CHECK(r.status == 200 && json::parse(r.body)["revision"] == 3 && json::parse(r.body)["marker_size"] == 0.05);
        CHECK(ft.applied_fx == 900.0);
    }

    // no frame within 2 s: 202 with the revision (persisted and accepted, applied later)
    r = put(c, good_body(3));
    CHECK(r.status == 202);
    e = json::parse(r.body);
    CHECK(e["revision"] == 4 && !e.contains("fx"));
    CHECK(c.intrinsics().revision == 4 && json::parse(read_file(cfg))["fx"] == 830.5);
    Intrinsics in;
    uint64_t rev = 0;
    CHECK(c.take_intrinsics(in, rev) && rev == 4 && in.cam.fx == 830.5);
    c.finish_intrinsics(rev);
    CHECK(!c.take_intrinsics(in, rev));

    // persist failure: 500, revision unchanged, nothing applied
    VisionControl bad_dir(opts(g_dir + "/missing/vision.json"), nullptr);
    r = put(bad_dir, good_body(1));
    CHECK(r.status == 500 && json::parse(r.body)["error"] == "PERSIST_FAILED");
    CHECK(bad_dir.intrinsics().revision == 1 && bad_dir.intrinsics().in.cam.fx == 600);
    CHECK(!bad_dir.take_intrinsics(in, rev));
    // a config file that is not a JSON object is not overwritten
    const std::string broken = g_dir + "/broken.json";
    write_file(broken, "[\"not an object\"]");
    VisionControl bc(opts(broken), nullptr);
    r = put(bc, good_body(1));
    CHECK(r.status == 500 && read_file(broken) == "[\"not an object\"]");
}

static void test_cli_override() {
    const std::string cfg = g_dir + "/cli.json";
    write_file(cfg, "{\"fx\":700}");
    VisionControl c(opts(cfg, true), nullptr);
    json g = json::parse(handle_control_request(c, "GET", "/api/calibration/intrinsics", "").body);
    CHECK(g["overridden_by_cli"] == true);
    CHECK(json::parse(handle_control_request(c, "GET", "/api/mode", "").body)["intrinsics"]["overridden_by_cli"] == true);
    HttpReply r = put(c, good_body(1));
    CHECK(r.status == 409 && json::parse(r.body)["error"] == "CLI_OVERRIDE");
    CHECK(read_file(cfg) == "{\"fx\":700}" && c.intrinsics().revision == 1);
}

static std::string http_roundtrip(int port, const std::string& raw) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
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
    for (;;) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        out.append(buf, static_cast<size_t>(n));
    }
    ::close(fd);
    return out;
}

static void test_http_server_loopback() {
    const std::string cfg = g_dir + "/server.json";
    write_file(cfg, "{}");
    VisionControl c(opts(cfg), nullptr);
    FrameThread ft(c);
    HttpControlServer server(c);
    const int port = 18093;
    CHECK(server.start(port));
    const std::string body = good_body(1).dump();
    std::string resp = http_roundtrip(port, "PUT /api/calibration/intrinsics HTTP/1.1\r\nHost: localhost\r\n"
                                            "Content-Type: application/json\r\nContent-Length: " +
                                                std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
    CHECK(resp.find("200 OK") != std::string::npos && resp.find("\"revision\":2") != std::string::npos);
    resp = http_roundtrip(port, "PUT /api/calibration/intrinsics HTTP/1.1\r\nHost: localhost\r\n"
                                "Content-Type: text/plain\r\nContent-Length: " +
                                    std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body);
    CHECK(resp.find("415") != std::string::npos);
    server.stop();
}

int main() {
    char tmpl[] = "/tmp/rv-intrinsics-XXXXXX";
    if (!::mkdtemp(tmpl)) return 2;
    g_dir = tmpl;
    test_validation_table();
    test_merge();
    test_atomic_write();
    test_loopback();
    test_endpoint();
    test_cli_override();
    test_http_server_loopback();
    std::system(("rm -rf '" + g_dir + "'").c_str());
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("intrinsics: all checks passed\n");
    return 0;
}
