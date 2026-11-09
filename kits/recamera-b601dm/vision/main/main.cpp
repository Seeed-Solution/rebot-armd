// rebot-vision: the reBot reCamera kit's camera process. One af::App owns the camera; YOLO
// (TPU) and ArUco (CPU) run on the same 640x640 inference frame by mode (off / yolo / aruco /
// both), switched at runtime over HTTP :8090; H.264 over WebSocket :8001 as yolo-detector.
//
// Merged and modified from sscma-example-sg200x (commit 9af8118, Apache-2.0)
// solutions/yolo-detector/main/main.cpp (camera/stream setup, AE fixed frame rate, detector)
// and solutions/aruco-detector/main/main.cpp (CLI, marker pipeline, capture time). Dropped:
// MQTT / Home Assistant, RTSP, ONVIF, person tracking, zones/lines, privacy blur.
#include <getopt.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <af/app.h>
#include <af/ma_image.h>
#include <cvi_ae.h>
#include <cvi_isp.h>
#include <sscma.h>

#include "aruco_stage.h"
#include "capture_time.h"
#include "detector.h"
#include "http_control.h"
#include "json.hpp"
#include "result_json.h"
#include "version.h"
#include "vision_control.h"

#define TAG "rebot-vision"

namespace {

struct Config {
    std::string config_path = "/userdata/rebot-teleop/vision.json";
    std::string model_path;  // empty: first existing default
    float conf_threshold = 0.25f;
    rv::Mode mode = rv::Mode::Yolo;
    bool inference_enabled = false;
    int fps = 5;  // one inference rate for every mode

    int inference_w = 640;
    int inference_h = 640;
    int stream_w = 1280;
    int stream_h = 720;
    int stream_fps = 15;
    int stream_bitrate_kbps = 1000;
    int debug_port = 8001;
    int control_port = 8090;

    rv::ArucoConfig aruco;
    int aruco_stats_every = 200;  ///< log p50/p95 of the ArUco stages every N ArUco frames (0 = off)
    bool verbose = false;
} g_cfg;

const char* kDefaultModels[] = {
    "/userdata/rebot-teleop/models/yolo11n.cvimodel",
    "/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel",
    "/userdata/local/models/yolo11n_detection_cv181x_int8.cvimodel",
    "/userdata/local/models/yolo11n_cv181x_int8.cvimodel",
};

/** Collects ArUco frame timings and logs p50/p95 per stage every n frames. */
class ArucoStats {
public:
    void add(double total_ms, const rv::ArucoTiming& t, int n) {
        if (n <= 0) return;
        const double v[kN] = {total_ms,       t.gray_ms,   t.decimate_ms, t.threshold_ms, t.unionfind_ms,
                              t.clusters_ms,  t.quad_fit_ms, t.refine_ms,  t.decode_ms,    t.detect_other_ms,
                              t.filter_ms,    t.pose_ms,   static_cast<double>(t.quads)};
        for (int i = 0; i < kN; ++i) col_[i].push_back(v[i]);
        if (static_cast<int>(col_[0].size()) < n) return;
        static const char* names[kN] = {"total", "gray", "decimate", "threshold", "unionfind", "clusters", "quad_fit",
                                        "refine", "decode", "detect_other", "filter", "pose", "quads"};
        std::string line;
        char b[64];
        for (int i = 0; i < kN; ++i) {
            std::vector<double>& c = col_[i];
            std::sort(c.begin(), c.end());
            std::snprintf(b, sizeof(b), " %s=%.1f/%.1f", names[i], c[c.size() / 2], c[(c.size() * 95) / 100]);
            line += b;
            c.clear();
        }
        MA_LOGI(TAG, "ArUco p50/p95 ms over %d frames:%s", n, line.c_str());
    }

private:
    static constexpr int kN = 13;
    std::vector<double> col_[kN];
};

bool file_exists(const std::string& p) {
    return !p.empty() && access(p.c_str(), R_OK) == 0;
}

std::string base_name(const std::string& p) {
    std::string b = p.substr(p.find_last_of('/') + 1);
    const size_t dot = b.find('.');
    return dot == std::string::npos ? b : b.substr(0, dot);
}

void cam_defaults() {
    g_cfg.aruco.cam.fx = 600.0;
    g_cfg.aruco.cam.fy = 600.0;
    g_cfg.aruco.cam.cx = 320.0;
    g_cfg.aruco.cam.cy = 180.0;
}

/** vision.json: same keys as the long options (with '_' for '-'). Missing file = defaults. */
void load_config_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) return;
    const nlohmann::json j = nlohmann::json::parse(f, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        MA_LOGW(TAG, "Ignoring %s: not a JSON object", path.c_str());
        return;
    }
    auto num = [&](const char* k, auto& dst) {
        auto it = j.find(k);
        if (it != j.end() && it->is_number()) dst = it->get<std::decay_t<decltype(dst)>>();
    };
    auto str = [&](const char* k, std::string& dst) {
        auto it = j.find(k);
        if (it != j.end() && it->is_string()) dst = it->get<std::string>();
    };
    std::string mode;
    str("mode", mode);
    if (!mode.empty() && !rv::parse_mode(mode, g_cfg.mode)) MA_LOGW(TAG, "%s: bad mode '%s'", path.c_str(), mode.c_str());
    if (auto it = j.find("inference"); it != j.end() && it->is_boolean()) g_cfg.inference_enabled = it->get<bool>();
    num("fps", g_cfg.fps);
    str("model", g_cfg.model_path);
    num("conf_threshold", g_cfg.conf_threshold);
    num("stream_fps", g_cfg.stream_fps);
    num("stream_bitrate", g_cfg.stream_bitrate_kbps);
    str("family", g_cfg.aruco.family);
    num("marker_size", g_cfg.aruco.marker_size);
    num("decimate", g_cfg.aruco.quad_decimate);
    num("fx", g_cfg.aruco.cam.fx);
    num("fy", g_cfg.aruco.cam.fy);
    num("cx", g_cfg.aruco.cam.cx);
    num("cy", g_cfg.aruco.cam.cy);
    num("k1", g_cfg.aruco.cam.k1);
    num("k2", g_cfg.aruco.cam.k2);
    num("p1", g_cfg.aruco.cam.p1);
    num("p2", g_cfg.aruco.cam.p2);
    num("k3", g_cfg.aruco.cam.k3);
    num("min_margin", g_cfg.aruco.filter.min_margin);
    num("min_side", g_cfg.aruco.filter.min_side_px);
    num("aruco_stats_every", g_cfg.aruco_stats_every);
    if (auto it = j.find("ids"); it != j.end() && it->is_string()) {
        std::string err;
        if (!aruco::parse_id_list(it->get<std::string>(), g_cfg.aruco.filter.ids, &err))
            MA_LOGW(TAG, "%s: bad ids: %s", path.c_str(), err.c_str());
    }
}

void print_usage(const char* prog) {
    printf("rebot-vision %s: YOLO detection and ArUco markers for the reBot reCamera kit\n\n", PROJECT_VERSION);
    printf("Usage: %s [options]   (options override %s)\n", prog, g_cfg.config_path.c_str());
    printf("  --config PATH            JSON config file (keys as the long options, '_' for '-')\n");
    printf("  --mode off|yolo|aruco|both  Base mode at start [default: yolo]\n");
    printf("  --inference on|off       Default subscription (the mode API) at start [default: off = standby]\n");
    printf("  --fps N                  Inference rate of the default subscription and cap of every\n");
    printf("                           subscription, 1..15 [default: 5]\n");
    printf("  -m, --model PATH         YOLO .cvimodel [default: first of /userdata/rebot-teleop/models/yolo11n.cvimodel,\n");
    printf("                           /usr/share/supervisor/models/..., /userdata/local/models/...]\n");
    printf("  -c, --conf-threshold F   YOLO confidence threshold [default: 0.25]\n");
    printf("  -f, --family NAME        Marker family (aruco4x4_50|100|250, aruco5x5_50, aruco6x6_250, tag36h11, tag16h5)\n");
    printf("  -s, --marker-size M      Marker side in meters [default: 0.05]\n");
    printf("  --fx --fy --cx --cy V    Intrinsics at 640x360 [default: 600 600 320 180]\n");
    printf("  --k1 --k2 --p1 --p2 --k3 V  Lens distortion (OpenCV order) [default: 0]\n");
    printf("  --decimate F             Quad decimation [default: 2.0]\n");
    printf("  --min-margin V           Drop markers with decision_margin below V (0 = off) [default: 30]\n");
    printf("  --min-side PX            Drop markers whose shortest edge is below PX (0 = off) [default: 12]\n");
    printf("  --ids LIST               Marker id allowlist, e.g. 3,7,12 [default: all]\n");
    printf("  --stream-fps N           H.264 stream rate, 1..30 [default: 15]\n");
    printf("  --stream-bitrate KBPS    H.264 CBR target [default: 1000]\n");
    printf("  --debug-port PORT        WebSocket video/results port [default: 8001]\n");
    printf("  --control-port PORT      HTTP control port [default: 8090]\n");
    printf("  -v, --verbose            Verbose logging\n");
    printf("  -h, --help               This help\n");
}

bool parse_double(const char* s, double& out, bool non_negative = false) {
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s || *end != '\0' || !std::isfinite(v) || (non_negative && v < 0)) return false;
    out = v;
    return true;
}

bool parse_args(int argc, char** argv) {
    enum {
        kConfig = 1000, kMode, kInference, kFps, kFx, kFy, kCx, kCy, kK1, kK2, kP1, kP2, kK3, kDecimate,
        kMinMargin, kMinSide, kIds, kStreamFps, kStreamBitrate, kDebugPort, kControlPort
    };
    static struct option long_opts[] = {
        {"config", required_argument, nullptr, kConfig},
        {"mode", required_argument, nullptr, kMode},
        {"inference", required_argument, nullptr, kInference},
        {"fps", required_argument, nullptr, kFps},
        {"model", required_argument, nullptr, 'm'},
        {"conf-threshold", required_argument, nullptr, 'c'},
        {"family", required_argument, nullptr, 'f'},
        {"marker-size", required_argument, nullptr, 's'},
        {"fx", required_argument, nullptr, kFx},
        {"fy", required_argument, nullptr, kFy},
        {"cx", required_argument, nullptr, kCx},
        {"cy", required_argument, nullptr, kCy},
        {"k1", required_argument, nullptr, kK1},
        {"k2", required_argument, nullptr, kK2},
        {"p1", required_argument, nullptr, kP1},
        {"p2", required_argument, nullptr, kP2},
        {"k3", required_argument, nullptr, kK3},
        {"decimate", required_argument, nullptr, kDecimate},
        {"min-margin", required_argument, nullptr, kMinMargin},
        {"min-side", required_argument, nullptr, kMinSide},
        {"ids", required_argument, nullptr, kIds},
        {"stream-fps", required_argument, nullptr, kStreamFps},
        {"stream-bitrate", required_argument, nullptr, kStreamBitrate},
        {"debug-port", required_argument, nullptr, kDebugPort},
        {"control-port", required_argument, nullptr, kControlPort},
        {"verbose", no_argument, nullptr, 'v'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0},
    };
    // --config first, so the remaining options override the file
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0) g_cfg.config_path = argv[i + 1];
    }
    load_config_file(g_cfg.config_path);

    int opt;
    double d = 0;
    while ((opt = getopt_long(argc, argv, "m:c:f:s:vh", long_opts, nullptr)) != -1) {
        switch (opt) {
            case kConfig: break;
            case kMode:
                if (!rv::parse_mode(optarg, g_cfg.mode)) {
                    fprintf(stderr, "Invalid --mode '%s' (off|yolo|aruco|both)\n", optarg);
                    return false;
                }
                break;
            case kInference:
                if (std::strcmp(optarg, "on") && std::strcmp(optarg, "off")) {
                    fprintf(stderr, "Invalid --inference '%s' (on|off)\n", optarg);
                    return false;
                }
                g_cfg.inference_enabled = std::strcmp(optarg, "on") == 0;
                break;
            case kFps: g_cfg.fps = std::atoi(optarg); break;
            case 'm': g_cfg.model_path = optarg; break;
            case 'c': g_cfg.conf_threshold = static_cast<float>(std::atof(optarg)); break;
            case 'f': g_cfg.aruco.family = optarg; break;
            case 's': g_cfg.aruco.marker_size = std::atof(optarg); break;
            case kFx: g_cfg.aruco.cam.fx = std::atof(optarg); break;
            case kFy: g_cfg.aruco.cam.fy = std::atof(optarg); break;
            case kCx: g_cfg.aruco.cam.cx = std::atof(optarg); break;
            case kCy: g_cfg.aruco.cam.cy = std::atof(optarg); break;
            case kK1: g_cfg.aruco.cam.k1 = std::atof(optarg); break;
            case kK2: g_cfg.aruco.cam.k2 = std::atof(optarg); break;
            case kP1: g_cfg.aruco.cam.p1 = std::atof(optarg); break;
            case kP2: g_cfg.aruco.cam.p2 = std::atof(optarg); break;
            case kK3: g_cfg.aruco.cam.k3 = std::atof(optarg); break;
            case kDecimate: g_cfg.aruco.quad_decimate = std::atof(optarg); break;
            case kMinMargin:
            case kMinSide:
                if (!parse_double(optarg, d, true)) {
                    fprintf(stderr, "Invalid value for --%s: '%s' (expected number >= 0)\n",
                            opt == kMinMargin ? "min-margin" : "min-side", optarg);
                    return false;
                }
                (opt == kMinMargin ? g_cfg.aruco.filter.min_margin : g_cfg.aruco.filter.min_side_px) =
                    static_cast<float>(d);
                break;
            case kIds: {
                std::string err;
                if (!aruco::parse_id_list(optarg, g_cfg.aruco.filter.ids, &err)) {
                    fprintf(stderr, "Invalid --ids '%s': %s\n", optarg, err.c_str());
                    return false;
                }
                break;
            }
            case kStreamFps: g_cfg.stream_fps = std::atoi(optarg); break;
            case kStreamBitrate: g_cfg.stream_bitrate_kbps = std::atoi(optarg); break;
            case kDebugPort: g_cfg.debug_port = std::atoi(optarg); break;
            case kControlPort: g_cfg.control_port = std::atoi(optarg); break;
            case 'v': g_cfg.verbose = g_cfg.aruco.verbose = true; break;
            case 'h': print_usage(argv[0]); exit(0);
            default: print_usage(argv[0]); return false;
        }
    }
    g_cfg.fps = std::max(rv::VisionControl::kMinFps, std::min(rv::VisionControl::kMaxFps, g_cfg.fps));
    g_cfg.stream_fps = std::max(1, std::min(30, g_cfg.stream_fps));
    if (!rv::aruco_family_supported(g_cfg.aruco.family)) {
        fprintf(stderr, "Unsupported marker family '%s'\n", g_cfg.aruco.family.c_str());
        return false;
    }
    return true;
}

// AE defaults to slow-shutter, which drops the sensor to ~10 fps indoors. Teleop needs a
// steady frame rate, so keep it fixed and let AE raise gain (from yolo-detector).
void set_ae_fixed_frame_rate() {
    ISP_EXPOSURE_ATTR_S exp_attr;
    std::memset(&exp_attr, 0, sizeof(exp_attr));
    if (CVI_ISP_GetExposureAttr(0, &exp_attr) == CVI_SUCCESS) {
        exp_attr.stAuto.enAEMode = AE_MODE_FIX_FRAME_RATE;
        if (CVI_ISP_SetExposureAttr(0, &exp_attr) == CVI_SUCCESS) MA_LOGI(TAG, "AE mode set to fixed frame rate");
        else MA_LOGW(TAG, "Failed to set AE fixed frame rate");
    }
}

uint64_t capture_unix_ms(uint32_t frame_mono_ms) {
    timespec mono{};
    clock_gettime(CLOCK_MONOTONIC, &mono);
    const uint32_t now_mono = static_cast<uint32_t>(static_cast<uint64_t>(mono.tv_sec) * 1000u +
                                                    static_cast<uint64_t>(mono.tv_nsec) / 1000000u);
    const uint64_t now_unix = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                        std::chrono::system_clock::now().time_since_epoch())
                                                        .count());
    return aruco::capture_unix_ms(now_unix, now_mono, frame_mono_ms);
}

float ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

/** The camera singleton that af::App's CameraHub drives (CH0 is its selected channel). */
ma::Camera* device_camera() {
    ma::Device* dev = ma::Device::getInstance();
    if (dev == nullptr) return nullptr;
    for (auto* sensor : dev->getSensors()) {
        if (sensor->getType() == ma::Sensor::Type::kCamera) return static_cast<ma::Camera*>(sensor);
    }
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    cam_defaults();
    if (!parse_args(argc, argv)) return 1;
    MA_LOGI(TAG, "rebot-vision %s (perception API v%d)", PROJECT_VERSION, rv::kApiVersion);

    // YOLO: the model stays loaded in every mode, so switching to yolo/both needs no load.
    std::string model = g_cfg.model_path;
    if (model.empty()) {
        for (const char* p : kDefaultModels) {
            if (file_exists(p)) {
                model = p;
                break;
            }
        }
    }
    yolo::Detector detector;
    bool yolo_ok = false;
    if (!file_exists(model)) {
        MA_LOGW(TAG, "No YOLO model found (%s); yolo/both unavailable, aruco works",
                model.empty() ? "default paths" : model.c_str());
    } else if (detector.init(model)) {
        detector.setThreshold(g_cfg.conf_threshold);
        yolo_ok = true;
    } else {
        MA_LOGE(TAG, "YOLO model %s failed to load; yolo/both unavailable", model.c_str());
    }

    rv::ArucoStage aruco_stage;
    if (!aruco_stage.init(g_cfg.aruco)) return 1;
    MA_LOGI(TAG, "ArUco %s, marker %.3f m, fx=%.1f fy=%.1f cx=%.1f cy=%.1f, undistort %s, filter margin>=%.1f side>=%.1fpx ids=%s",
            g_cfg.aruco.family.c_str(), g_cfg.aruco.marker_size, g_cfg.aruco.cam.fx, g_cfg.aruco.cam.fy,
            g_cfg.aruco.cam.cx, g_cfg.aruco.cam.cy, aruco_stage.undistorted() ? "on" : "off",
            g_cfg.aruco.filter.min_margin, g_cfg.aruco.filter.min_side_px,
            aruco::format_id_list(g_cfg.aruco.filter.ids).c_str());

    af::App app("rebot-vision");
    af::CameraOptions cam;
    cam.width = static_cast<uint16_t>(g_cfg.inference_w);
    cam.height = static_cast<uint16_t>(g_cfg.inference_h);
    // CH0's hardware frame-rate control (VPSS) is fixed at this start rate; a later
    // set_camera_fps() only moves the camera callback's software gate, which cannot pass more
    // frames than VPSS delivers. Start at the highest inference rate so every rate up to it
    // is reachable at runtime; VisionControl sets the gate (idle 1 fps) and RateGate picks
    // the frames.
    cam.fps = static_cast<uint8_t>(rv::VisionControl::kMaxFps);
    cam.physical = false;
    cam.idle_fps = rv::VisionControl::kIdleFps;
    af::StreamOptions stream;
    stream.width = static_cast<uint16_t>(g_cfg.stream_w);
    stream.height = static_cast<uint16_t>(g_cfg.stream_h);
    stream.fps = static_cast<uint8_t>(g_cfg.stream_fps);
    stream.bitrate_kbps = static_cast<uint32_t>(g_cfg.stream_bitrate_kbps);
    app.with_camera(cam).with_stream_mode(af::StreamMode::WsOnly, stream, g_cfg.debug_port);

    rv::VisionControl::Options vo;
    vo.fps = g_cfg.fps;
    vo.mode = g_cfg.mode;
    vo.enabled = g_cfg.inference_enabled;
    vo.yolo_available = yolo_ok;
    vo.model_name = yolo_ok ? base_name(model) : "";
    vo.model_path = yolo_ok ? model : "";
    vo.family = g_cfg.aruco.family;
    ma::Camera* camera = device_camera();
    rv::VisionControl ctl(vo, [&app, camera](bool on, int gate_fps) {
        // set_camera_fps() is applied by the frame thread at its next loop, but that thread
        // is blocked in retrieveFrame() until the old gate passes a frame (up to 1 s in
        // standby). Writing the gate rate directly lets the next CH0 frame through, so a
        // trigger or an enable is served within one frame. Both writes go to the same int
        // (CameraSG200X channel fps), in the same order under VisionControl's lock.
        app.set_camera_fps(static_cast<uint8_t>(gate_fps));
        if (camera != nullptr) {
            ma::Camera::CtrlValue v{};
            v.i32 = gate_fps;
            camera->commandCtrl(ma::Camera::CtrlType::kFps, ma::Camera::CtrlMode::kWrite, v);
        }
        MA_LOGI(TAG, "Processing %s (camera gate %d fps)", on ? "ON" : "OFF", gate_fps);
    });
    MA_LOGI(TAG, "Start: mode %s, inference %s, fps %d", rv::mode_name(ctl.mode()),
            ctl.continuous_enabled() ? "on" : "standby", ctl.active_fps());

    rv::HttpControlServer http(ctl);
    if (!http.start(g_cfg.control_port)) MA_LOGE(TAG, "HTTP control unavailable on port %d", g_cfg.control_port);

    // The ArUco crop and the detection normalisation use the visible part of the
    // letterboxed inference frame (same geometry as the debug /results overlay).
    const rv::ContentRect content = rv::content_rect(g_cfg.inference_w, g_cfg.inference_h, g_cfg.stream_w, g_cfg.stream_h);
    const af::Rect crop{static_cast<int32_t>(std::lround(content.x)), static_cast<int32_t>(std::lround(content.y)),
                        static_cast<int32_t>(std::lround(content.w)), static_cast<int32_t>(std::lround(content.h))};
    const double disp_sx = content.w > 0 ? g_cfg.stream_w / content.w : 1.0;
    const double disp_sy = content.h > 0 ? g_cfg.stream_h / content.h : 1.0;
    MA_LOGI(TAG, "Content rect in %dx%d: x=%d y=%d w=%d h=%d", g_cfg.inference_w, g_cfg.inference_h, crop.x, crop.y,
            crop.w, crop.h);

    // Reused across frames.
    std::vector<af::Box> boxes;
    std::vector<rv::DetBox> dets;
    std::vector<rv::MarkerOut> markers;
    ArucoStats aruco_stats;
    std::string extra;
    uint32_t frame_id = 0;
    // Frame age for triggers and the rate gate. retrieveFrame() returns the newest queued
    // frame: if the grab waited, the frame arrived during the wait (now); if one was already
    // queued, it arrived after the previous grab.
    uint64_t before_grab_ms = 0;
    uint64_t last_grab_ms = 0;

    app.on_started(set_ae_fixed_frame_rate).on_before_grab([&] { before_grab_ms = rv::mono_ms(); });
    app.on_frame([&](const af::VideoFrame& frame, af::Result& r) {
        const uint64_t grab_ms = rv::mono_ms();
        const uint64_t frame_ms = grab_ms > before_grab_ms ? grab_ms : last_grab_ms;
        last_grab_ms = grab_ms;
        const rv::Pipelines p = ctl.frame_plan(frame_ms);
        if (!p.any() || !frame.valid() || !frame.cpu_accessible()) {
            r.publish = false;  // standby: no /results; the stream and snapshot keep running
            return;
        }
        const auto t_frame = std::chrono::steady_clock::now();
        rv::VisionResult vr;
        vr.frame_id = ++frame_id;
        vr.frame_ms = frame_ms;
        vr.ts_ms = capture_unix_ms(frame.ts_ms);
        vr.mode = ctl.mode();
        vr.width = crop.w;
        vr.height = crop.h;
        boxes.clear();
        dets.clear();
        markers.clear();

        if (p.yolo) {
            const auto t0 = std::chrono::steady_clock::now();
            ma_img_t img = af::to_ma_img(frame);
            for (const auto& d : detector.detect(&img)) {
                dets.push_back({d.x, d.y, d.w, d.h, d.confidence, d.class_id, yolo::Detector::getClassName(d.class_id)});
                af::Box b;  // centre pixels in the inference frame; the debug sink maps them to the display
                b.x = d.x * frame.width;
                b.y = d.y * frame.height;
                b.w = d.w * frame.width;
                b.h = d.h * frame.height;
                b.score = d.confidence;
                b.cls = d.class_id;
                b.label = yolo::Detector::getClassName(d.class_id);
                boxes.push_back(b);
            }
            vr.ran_yolo = true;
            vr.yolo_ms = ms_since(t0);
            vr.detections_json = rv::detections_json(dets, content, frame.width, frame.height);
        }
        std::string display_markers;
        if (p.aruco) {
            const auto t0 = std::chrono::steady_clock::now();
            const af::VideoFrame sub = frame.sub_frame(crop);
            if (sub.valid()) {
                vr.rejected = aruco_stage.run(sub.data(), sub.width, sub.height, static_cast<int>(sub.stride()), markers);
            }
            vr.ran_aruco = true;
            vr.aruco_ms = ms_since(t0);
            vr.aruco_stages_json = rv::aruco_stages_json(aruco_stage.last_timing());
            aruco_stats.add(vr.aruco_ms, aruco_stage.last_timing(), g_cfg.aruco_stats_every);
            vr.count = static_cast<int>(markers.size());
            vr.undistorted = aruco_stage.undistorted();
            vr.markers_json = rv::markers_json(markers);
            display_markers = rv::markers_json(markers, disp_sx, disp_sy);
        }
        vr.inference_ms = ms_since(t_frame);
        ctl.publish(vr);

        r.width = frame.width;
        r.height = frame.height;
        r.inference_ms = vr.inference_ms;
        r.set_boxes(boxes.data(), boxes.size());
        extra = rv::debug_extra(vr.mode, display_markers);
        r.debug_extra = extra;

        if (g_cfg.verbose) {
            MA_LOGI(TAG, "Frame %u %s: %zu detections (%.0f ms), %zu markers (%.0f ms)", vr.frame_id,
                    rv::mode_name(vr.mode), dets.size(), vr.yolo_ms, markers.size(), vr.aruco_ms);
        }
    });

    const int rc = app.run();
    http.stop();
    MA_LOGI(TAG, "rebot-vision terminated");
    return rc;
}
