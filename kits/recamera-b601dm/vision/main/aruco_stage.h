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
    double marker_size = 0.05;  // m
    aruco::CameraModel cam;     // intrinsics at the content image size (640x360)
    aruco::MarkerFilter filter;
    bool verbose = false;
};

bool aruco_family_supported(const std::string& name);

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

private:
    ArucoConfig cfg_;
    bool undistort_ = false;
    apriltag_family* tf_ = nullptr;
    apriltag_detector* td_ = nullptr;
    std::vector<uint8_t> gray_;
};

}  // namespace rv

#endif  // REBOT_VISION_ARUCO_STAGE_H_
