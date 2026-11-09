// Modified from sscma-example-sg200x solutions/aruco-detector/main/main.cpp (commit 9af8118,
// Apache-2.0): family table, gray conversion, detection, filter and pose moved into a stage;
// rows are read with a stride so the stage runs on a zero-copy crop of the 640x640 frame.
#include "aruco_stage.h"
#include "tag_pose.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/timeprofile.h"
#include "aruco/tagAruco4x4_100.h"
#include "aruco/tagAruco4x4_250.h"
#include "aruco/tagAruco4x4_50.h"
#include "aruco/tagAruco5x5_50.h"
#include "aruco/tagAruco6x6_250.h"
#include "tag16h5.h"
#include "tag36h11.h"

#if __has_include(<sscma.h>)
#include <sscma.h>
#else  // host build (tests/aruco_bench): no SDK logging
#include <cstdio>
#define MA_LOGE(tag, ...) (std::fprintf(stderr, "E %s: ", tag), std::fprintf(stderr, __VA_ARGS__), std::fputc('\n', stderr))
#define MA_LOGW MA_LOGE
#define MA_LOGI MA_LOGE
#endif

#define TAG "rebot-vision-aruco"

namespace rv {

static apriltag_family_t* create_family(const std::string& name) {
    if (name == "aruco4x4_50") return tagAruco4x4_50_create();
    if (name == "aruco4x4_100") return tagAruco4x4_100_create();
    if (name == "aruco4x4_250") return tagAruco4x4_250_create();
    if (name == "aruco5x5_50") return tagAruco5x5_50_create();
    if (name == "aruco6x6_250") return tagAruco6x6_250_create();
    if (name == "tag36h11") return tag36h11_create();
    if (name == "tag16h5") return tag16h5_create();
    return nullptr;
}

static void destroy_family(const std::string& name, apriltag_family_t* tf) {
    if (!tf) return;
    if (name == "aruco4x4_50") tagAruco4x4_50_destroy(tf);
    else if (name == "aruco4x4_100") tagAruco4x4_100_destroy(tf);
    else if (name == "aruco4x4_250") tagAruco4x4_250_destroy(tf);
    else if (name == "aruco5x5_50") tagAruco5x5_50_destroy(tf);
    else if (name == "aruco6x6_250") tagAruco6x6_250_destroy(tf);
    else if (name == "tag36h11") tag36h11_destroy(tf);
    else if (name == "tag16h5") tag16h5_destroy(tf);
}

bool aruco_family_supported(const std::string& name) {
    static const char* k[] = {"aruco4x4_50", "aruco4x4_100", "aruco4x4_250", "aruco5x5_50",
                              "aruco6x6_250", "tag36h11", "tag16h5"};
    return std::find(std::begin(k), std::end(k), name) != std::end(k);
}

static void rot_to_euler(const matd_t* R, double& roll, double& pitch, double& yaw) {
    const double r20 = MATD_EL(R, 2, 0), r21 = MATD_EL(R, 2, 1), r22 = MATD_EL(R, 2, 2);
    const double r00 = MATD_EL(R, 0, 0), r10 = MATD_EL(R, 1, 0);
    pitch = std::asin(-std::clamp(r20, -1.0, 1.0));
    if (std::abs(r20) < 0.999999) {
        roll = std::atan2(r21, r22);
        yaw = std::atan2(r10, r00);
    } else {
        roll = std::atan2(-MATD_EL(R, 1, 2), MATD_EL(R, 1, 1));
        yaw = 0.0;
    }
}

ArucoStage::~ArucoStage() {
    if (td_) apriltag_detector_destroy(td_);
    destroy_family(cfg_.family, tf_);
}

bool ArucoStage::init(const ArucoConfig& cfg) {
    cfg_ = cfg;
    undistort_ = cfg.cam.has_distortion();
    tf_ = create_family(cfg.family);
    if (!tf_) {
        MA_LOGE(TAG, "Unsupported marker family: %s", cfg.family.c_str());
        return false;
    }
    td_ = apriltag_detector_create();
    if (!td_) {
        MA_LOGE(TAG, "Failed to create apriltag detector");
        return false;
    }
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = static_cast<float>(cfg.quad_decimate);
    td_->quad_sigma = static_cast<float>(cfg.quad_sigma);
    td_->refine_edges = cfg.refine_edges ? 1 : 0;
    td_->decode_sharpening = cfg.decode_sharpening;
    td_->qtp.min_white_black_diff = cfg.min_white_black_diff;
    td_->qtp.min_cluster_pixels = cfg.min_cluster_pixels;
    td_->qtp.max_nmaxima = cfg.max_nmaxima;
    td_->qtp.max_line_fit_mse = cfg.max_line_fit_mse;
    td_->nthreads = 1;
    return true;
}

int ArucoStage::run(const uint8_t* rgb, int width, int height, int stride, std::vector<MarkerOut>& out) {
    out.clear();
    timing_ = ArucoTiming{};
    if (!td_ || !rgb || width <= 0 || height <= 0) return 0;
    using clock = std::chrono::steady_clock;
    auto ms = [](clock::time_point a, clock::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    const auto t0 = clock::now();
    const size_t total = static_cast<size_t>(width) * height;
    if (gray_.size() < total) gray_.resize(total);

    // RGB888 -> 8-bit gray, integer fixpoint (as aruco-detector), row by row for the crop stride
    uint8_t* gray = gray_.data();
    for (int y = 0; y < height; ++y) {
        const uint8_t* src = rgb + static_cast<size_t>(y) * stride;
        uint8_t* dst = gray + static_cast<size_t>(y) * width;
        for (int x = 0; x < width; ++x) {
            const uint32_t r = src[3 * x], g = src[3 * x + 1], b = src[3 * x + 2];
            dst[x] = static_cast<uint8_t>((r * 77 + g * 150 + b * 29) >> 8);
        }
    }
    image_u8_t im = {width, height, width, gray};

    const auto t1 = clock::now();
    zarray_t* detections = apriltag_detector_detect(td_, &im);
    const auto t2 = clock::now();
    split_detect(timing_);
    timing_.quads = static_cast<int>(td_->nquads);
    double pose_ms = 0;
    const int n = zarray_size(detections);
    int rejected = 0;
    out.reserve(n);
    for (int i = 0; i < n; ++i) {
        apriltag_detection_t* det = nullptr;
        zarray_get(detections, i, &det);
        if (!det) continue;

        const aruco::FilterVerdict verdict =
            aruco::classify_marker(cfg_.filter, det->id, det->decision_margin, det->p);
        if (verdict != aruco::FilterVerdict::Accept) {
            ++rejected;
            if (cfg_.verbose) {
                MA_LOGI(TAG, "Rejected id=%d margin=%.1f side=%.1fpx (%s)", det->id, det->decision_margin,
                        aruco::min_quad_side(det->p),
                        verdict == aruco::FilterVerdict::LowMargin   ? "margin"
                        : verdict == aruco::FilterVerdict::SmallSide ? "side"
                                                                     : "id");
            }
            continue;
        }

        // Pose is solved on ideal (undistorted) pixel coordinates; JSON keeps raw corners.
        apriltag_detection_info_t info;
        apriltag_detection_t det_u;
        matd_t* H_u = nullptr;
        info.det = det;
        if (undistort_) {
            det_u = *det;
            for (int p = 0; p < 4; ++p) cfg_.cam.undistort(det->p[p][0], det->p[p][1], det_u.p[p][0], det_u.p[p][1]);
            cfg_.cam.undistort(det->c[0], det->c[1], det_u.c[0], det_u.c[1]);
            double h[9];
            if (!aruco::homography_from_corners(det_u.p, h)) {
                ++rejected;
                if (cfg_.verbose) MA_LOGW(TAG, "Rejected id=%d: homography from undistorted corners failed", det->id);
                continue;
            }
            H_u = matd_create_data(3, 3, h);
            det_u.H = H_u;
            info.det = &det_u;
        }
        info.tagsize = cfg_.marker_size;
        info.fx = cfg_.cam.fx;
        info.fy = cfg_.cam.fy;
        info.cx = cfg_.cam.cx;
        info.cy = cfg_.cam.cy;

        const auto tp0 = clock::now();
        apriltag_pose_t pose;
        const double err = cfg_.apriltag_pose ? estimate_tag_pose(&info, &pose) : tag_pose_estimate(&info, &pose);
        double roll = 0, pitch = 0, yaw = 0;
        rot_to_euler(pose.R, roll, pitch, yaw);
        pose_ms += ms(tp0, clock::now());

        MarkerOut m;
        m.id = det->id;
        m.margin = det->decision_margin;
        m.c[0] = det->c[0];
        m.c[1] = det->c[1];
        for (int p = 0; p < 4; ++p) {
            m.p[p][0] = det->p[p][0];
            m.p[p][1] = det->p[p][1];
        }
        for (int k = 0; k < 3; ++k) m.t[k] = MATD_EL(pose.t, k, 0);
        m.r_deg[0] = roll * 180.0 / M_PI;
        m.r_deg[1] = pitch * 180.0 / M_PI;
        m.r_deg[2] = yaw * 180.0 / M_PI;
        m.error = err;
        out.push_back(m);

        matd_destroy(pose.R);
        matd_destroy(pose.t);
        if (H_u) matd_destroy(H_u);
    }
    apriltag_detections_destroy(detections);
    const auto t3 = clock::now();
    timing_.gray_ms = ms(t0, t1);
    timing_.detect_ms = ms(t1, t2);
    timing_.pose_ms = pose_ms;
    timing_.filter_ms = std::max(0.0, ms(t2, t3) - pose_ms);
    return rejected;
}

// Stamps of the detect call that just ran (apriltag's time profile): each stamp's time is
// the time since the previous one.
void ArucoStage::split_detect(ArucoTiming& t) const {
    const timeprofile_t* tp = td_->tp;
    int64_t last = tp->utime;
    for (int i = 0; i < zarray_size(tp->stamps); ++i) {
        struct timeprofile_entry* e = nullptr;
        zarray_get_volatile(tp->stamps, i, &e);
        const double d = static_cast<double>(e->utime - last) / 1000.0;
        last = e->utime;
        const char* n = e->name;
        if (!std::strcmp(n, "decimate")) t.decimate_ms += d;
        else if (!std::strcmp(n, "threshold")) t.threshold_ms += d;
        else if (!std::strcmp(n, "unionfind")) t.unionfind_ms += d;
        else if (!std::strcmp(n, "make clusters")) t.clusters_ms += d;
        else if (!std::strcmp(n, "fit quads to clusters")) t.quad_fit_ms += d;
        else if (!std::strcmp(n, "refine")) t.refine_ms += d;
        else if (!std::strcmp(n, "decode")) t.decode_ms += d;
        else t.detect_other_ms += d;
    }
}

std::string aruco_stages_json(const ArucoTiming& t) {
    char b[384];
    std::snprintf(b, sizeof(b),
                  "{\"gray\":%.1f,\"decimate\":%.1f,\"threshold\":%.1f,\"unionfind\":%.1f,\"clusters\":%.1f,"
                  "\"quad_fit\":%.1f,\"refine\":%.1f,\"decode\":%.1f,\"detect_other\":%.1f,\"filter\":%.1f,"
                  "\"pose\":%.1f,\"quads\":%d}",
                  t.gray_ms, t.decimate_ms, t.threshold_ms, t.unionfind_ms, t.clusters_ms, t.quad_fit_ms,
                  t.refine_ms, t.decode_ms, t.detect_other_ms, t.filter_ms, t.pose_ms, t.quads);
    return b;
}

}  // namespace rv
