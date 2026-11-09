// Modified from sscma-example-sg200x solutions/aruco-detector/main/main.cpp (commit 9af8118,
// Apache-2.0): the per-frame marker detection, filter and pose, as a stage of rebot-vision.
#ifndef REBOT_VISION_ARUCO_STAGE_H_
#define REBOT_VISION_ARUCO_STAGE_H_

#include <cstdint>
#include <string>
#include <vector>

#include "camera_model.h"
#include "marker_filter.h"
#include "result_json.h"

struct apriltag_detector;
struct apriltag_family;

namespace rv {

struct ArucoConfig {
    std::string family = "aruco4x4_50";  // aruco4x4_50|100|250, aruco5x5_50, aruco6x6_250, tag36h11, tag16h5
    double quad_decimate = 2.0;
    double quad_sigma = 0.0;
    bool refine_edges = true;
    // Quad detector tuning. apriltag defaults (aruco-detector 0.1.3) except the contrast
    // threshold: 5 lets sensor noise in flat areas through the threshold step, and every
    // noise blob costs union-find, clustering and quad fitting. 20 keeps recall and false
    // positives at the 0.1.3 level on the host benchmark (tests/aruco_bench.cpp).
    double decode_sharpening = 0.25;
    int min_white_black_diff = 20;  ///< min contrast of a black/white component pair (0..255)
    int min_cluster_pixels = 24;   ///< min boundary pixels of a candidate quad (decimated image)
    int max_nmaxima = 10;
    float max_line_fit_mse = 10.0f;
    /// Pose with apriltag's estimate_tag_pose() instead of tag_pose_estimate() (same result,
    /// ~26k heap calls per marker); for the host benchmark's baseline.
    bool apriltag_pose = false;
    double marker_size = 0.05;  // m
    aruco::CameraModel cam;     // intrinsics at the content image size (640x360)
    aruco::MarkerFilter filter;
    bool verbose = false;
};

bool aruco_family_supported(const std::string& name);

/** Split of the last run() in milliseconds (CPU wall time on the frame thread). */
struct ArucoTiming {
    double gray_ms = 0;    ///< RGB888 -> 8-bit gray
    double detect_ms = 0;  ///< apriltag quad detection and decoding
    double pose_ms = 0;    ///< filter, undistortion and pose of the accepted markers
};

class ArucoStage {
public:
    ArucoStage() = default;
    ~ArucoStage();
    ArucoStage(const ArucoStage&) = delete;
    ArucoStage& operator=(const ArucoStage&) = delete;

    bool init(const ArucoConfig& cfg);
    bool undistorted() const noexcept { return undistort_; }

    /**
     * Detect markers on a packed RGB888 image (stride in bytes, e.g. a sub_frame of CH0).
     * Fills accepted markers; returns the number rejected by the filter.
     */
    int run(const uint8_t* rgb, int width, int height, int stride, std::vector<MarkerOut>& out);
    const ArucoTiming& last_timing() const noexcept { return timing_; }

private:
    ArucoConfig cfg_;
    bool undistort_ = false;
    apriltag_family* tf_ = nullptr;
    apriltag_detector* td_ = nullptr;
    std::vector<uint8_t> gray_;
    ArucoTiming timing_;
};

}  // namespace rv

#endif  // REBOT_VISION_ARUCO_STAGE_H_
