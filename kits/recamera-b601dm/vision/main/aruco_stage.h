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
    /// Gradient clusters: test the neighbour colours before the union-find lookups (same
    /// result, fewer random memory reads; apriltag_patched/).
    bool lazy_clusters = true;
    /// Keep apriltag's union-find, cluster hash map and cluster point arrays from frame to
    /// frame and build the clusters in one pass (apriltag_patched/).
    bool reuse_buffers = true;
    /// Drop candidate quads before edge refinement and decoding when their shortest side is
    /// below filter.min_side_px - kSizeGateSlackPx (refinement moves corners by about a pixel;
    /// off when min_side_px <= kSizeGateSlackPx).
    bool size_gate = true;
    static constexpr double kSizeGateSlackPx = 2.0;
    /// Adaptive decimation (0 = off): use this quad_decimate while the previous frame accepted
    /// markers and all of them had a shortest side >= adaptive_min_side_px. Back to
    /// quad_decimate for adaptive_hold_frames after a frame accepts fewer markers than the one
    /// before, and every adaptive_full_every-th frame (to pick up new small markers).
    double adaptive_decimate = 0.0;
    double adaptive_min_side_px = 40.0;
    int adaptive_hold_frames = 10;
    int adaptive_full_every = 10;
    /// ROI tracking (subscriptions only; run(..., full_scan = true) always scans the whole
    /// image): detect in the box around the last frame's accepted markers, padded by
    /// roi_pad times the largest marker side plus roi_pad_px. The whole image is scanned when
    /// the last frame accepted no marker, after a frame that accepted fewer markers than the
    /// one before, and every roi_full_every-th frame.
    bool roi_tracking = false;
    double roi_pad = 1.0;
    double roi_pad_px = 16.0;
    int roi_full_every = 10;
    /// Pose with apriltag's estimate_tag_pose() instead of tag_pose_estimate() (same result,
    /// ~26k heap calls per marker); for the host benchmark's baseline.
    bool apriltag_pose = false;
    double marker_size = 0.05;  // m
    aruco::CameraModel cam;     // intrinsics at the content image size (640x360)
    aruco::MarkerFilter filter;
    bool verbose = false;
};

bool aruco_family_supported(const std::string& name);

/**
 * Split of the last run() in milliseconds (CPU wall time on the frame thread). The detect
 * stages come from apriltag's time profile (td->tp, stamps in apriltag_detector_detect and
 * apriltag_quad_thresh); detect_ms is the whole apriltag_detector_detect() call.
 */
struct ArucoTiming {
    double gray_ms = 0;       ///< RGB888 -> 8-bit gray
    double decimate_ms = 0;   ///< quad_decimate downsampling
    double threshold_ms = 0;  ///< tile min/max and adaptive threshold
    double unionfind_ms = 0;  ///< connected components
    double clusters_ms = 0;   ///< boundary clusters between black and white components
    double quad_fit_ms = 0;   ///< line fits and quads per cluster
    double refine_ms = 0;     ///< edge refinement on the full-resolution image
    double decode_ms = 0;     ///< homography and bit decoding per quad
    double detect_other_ms = 0;  ///< blur/sharpen, freeing the clusters, reconcile, cleanup
    double detect_ms = 0;     ///< apriltag_detector_detect() in total
    double filter_ms = 0;     ///< false-positive filter and copying the accepted markers
    double pose_ms = 0;       ///< undistortion and pose of the accepted markers
    int quads = 0;            ///< candidate quads (before the size gate)
    double quad_decimate = 0; ///< quad_decimate used for this frame
    int roi_w = 0, roi_h = 0; ///< detected region (the whole image without ROI tracking)
};

/** {"gray":..,"decimate":..,...}: the stages of t in ms, one decimal. */
std::string aruco_stages_json(const ArucoTiming& t);

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
     * Fills accepted markers; returns the number rejected by the filter. full_scan: ignore
     * ROI tracking for this frame (a trigger's frame).
     */
    int run(const uint8_t* rgb, int width, int height, int stride, std::vector<MarkerOut>& out,
            bool full_scan = false);
    const ArucoTiming& last_timing() const noexcept { return timing_; }

private:
    void split_detect(ArucoTiming& t) const;

    ArucoConfig cfg_;
    bool undistort_ = false;
    apriltag_family* tf_ = nullptr;
    apriltag_detector* td_ = nullptr;
    std::vector<uint8_t> gray_;
    ArucoTiming timing_;
    // adaptive decimation state (previous frame)
    int last_accepted_ = 0;
    double last_min_side_ = 0;
    int hold_ = 0;
    int since_full_ = 0;
    // ROI tracking state: box of the last frame's accepted markers (pixels), largest side
    double box_[4] = {0, 0, 0, 0};
    double box_side_ = 0;
    int roi_since_full_ = 0;
    bool roi_rescan_ = false;
};

}  // namespace rv

#endif  // REBOT_VISION_ARUCO_STAGE_H_
