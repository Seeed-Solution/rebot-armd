// rebot-vision: portable result building (no SDK). Geometry of the letterboxed inference
// frame, YOLO detections and ArUco markers as JSON, and the debug /results extra members.
#ifndef REBOT_VISION_RESULT_JSON_H_
#define REBOT_VISION_RESULT_JSON_H_

#include <cmath>
#include <string>
#include <vector>

#include "depth_geom.h"
#include "vision_control.h"

namespace rv {

/** Sensor content inside the inference frame, in inference pixels. */
struct ContentRect {
    float x = 0, y = 0, w = 0, h = 0;
};

/**
 * Where the display-aspect sensor image sits inside the inference frame (VPSS keeps the
 * aspect and centres it). Same formula as debug_stream_letterbox_to_display()
 * (components/debug_stream/src/debug_stream.cpp). 640x640 with 1280x720 -> {0,140,640,360}.
 */
ContentRect content_rect(int inference_w, int inference_h, int display_w, int display_h) noexcept;

/** One YOLO box, centre format, normalised to the model input (as sscma-micro reports it). */
struct DetBox {
    float x = 0, y = 0, w = 0, h = 0;
    float confidence = 0;
    int class_id = 0;
    const char* class_name = "unknown";
    float angle_deg = NAN;
    float angle_conf = NAN;
    float grasp_width_px = NAN;
    bool mask_focus = false;
    float disp_fg = NAN;
    float disp_bg = NAN;
    float depth_conf = NAN;
    float length_px = NAN;
    int jaw_clear = -1;
    float depth_angle_deg = NAN;   ///< depth PCA long-side angle; never copied into angle_deg
    float depth_angle_conf = NAN;
    float depth_width_px = NAN;    ///< depth PCA cross-section width
};

/**
 * Jaw closing direction for analyze_box_depth: the mask angle + 90 deg when the detection has a
 * finite angle_deg (segmentation model), else NAN (the depth PCA minor axis).
 */
float jaw_close_deg(const DetBox& d) noexcept;

/**
 * Copies a valid depth result into the detection. The depth PCA angle and width go to
 * depth_angle_deg / depth_angle_conf / depth_width_px only; angle_deg / angle_conf /
 * grasp_width_px stay what the detector reported (absent for a plain detection model).
 */
void apply_box_depth(DetBox& d, const BoxDepthResult& r) noexcept;

/**
 * Detections JSON array (the yolo-detector field set) normalised to the content rect,
 * i.e. the visible image, not the letterboxed model input.
 */
std::string detections_json(const std::vector<DetBox>& dets, const ContentRect& rect, int inference_w,
                            int inference_h);

/** One accepted marker; pixel coordinates in the content image (e.g. 640x360). */
struct MarkerOut {
    int id = 0;
    double margin = 0;
    double c[2] = {0, 0};
    double p[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    double t[3] = {0, 0, 0};
    double r_deg[3] = {0, 0, 0};
    double error = 0;
};

/** Markers JSON array (aruco-detector field set); pixel coordinates multiplied by scale. */
std::string markers_json(const std::vector<MarkerOut>& markers, double scale_x = 1.0, double scale_y = 1.0);

/** Extra members for the debug /results envelope: "mode":"...","markers":[...] (display pixels). */
std::string debug_extra(Mode mode, const std::string& display_markers_json);

}  // namespace rv

#endif  // REBOT_VISION_RESULT_JSON_H_
