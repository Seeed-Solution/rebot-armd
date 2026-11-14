#ifndef _REBOT_VISION_OBB_GEOM_H_
#define _REBOT_VISION_OBB_GEOM_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rv {

struct ObbBox {
    float cx = 0.0f;        // Center X in model input pixels
    float cy = 0.0f;        // Center Y in model input pixels
    float w = 0.0f;         // Width in model input pixels (along angle_rad)
    float h = 0.0f;         // Height in model input pixels (along angle_rad + pi/2)
    float angle_rad = 0.0f; // Angle in radians [-pi/4, 3pi/4)
    float score = 0.0f;     // Confidence score [0, 1]
    int class_id = 0;       // Class ID
};

struct ObbGeom {
    float x = 0.0f;         // AABB center X [0, 1]
    float y = 0.0f;         // AABB center Y [0, 1]
    float w = 0.0f;         // AABB width [0, 1]
    float h = 0.0f;         // AABB height [0, 1]
    float score = 0.0f;
    int class_id = 0;
    float angle_deg = 0.0f; // Long-side angle in [-90, 90) deg
    float angle_conf = 0.0f;// Confidence in [0, 1]
    float grasp_width_px = 0.0f; // min(w_px, h_px)
    float length_px = 0.0f;      // max(w_px, h_px)
    std::array<std::array<float, 2>, 4> obb_corners{}; // 4 corners in perimeter order
};

struct ObbTensorView {
    const void* data = nullptr;
    bool is_int8 = false;       // false: FP32, true: INT8
    float scale = 1.0f;
    int32_t zero_point = 0;
    std::array<int32_t, 4> dims{0, 0, 0, 0}; // shape (N, C, H, W or N, H, W, C)
    int num_dims = 4;
};

enum class ObbDataType {
    Unknown = 0,
    FP32 = 1,
    INT8 = 2,
};

struct ObbTensorSpec {
    ObbDataType dtype = ObbDataType::Unknown;
    int num_dims = 0;
    std::array<int32_t, 4> dims{0, 0, 0, 0};
};

struct ObbLayoutAInfo {
    bool valid = false;
    bool channels_first = true;
    int num_channels = 0;
    int num_boxes = 0;
};

struct ObbLayoutBInfo {
    bool valid = false;
    bool is_nchw = true;
    bool is_interleaved = true;
    int num_classes = 0;
    ObbDataType dtype = ObbDataType::Unknown;
    std::array<size_t, 3> box_indices{};
    std::array<size_t, 3> cls_indices{};
    std::array<size_t, 3> ang_indices{};
    std::array<int, 3> grid_h{};
    std::array<int, 3> grid_w{};
};

// Resolve & validate Ultralytics 1-output Layout A model shape & data type.
// Validates batch=1, in_h, in_w >= 32 (% 32 == 0), in_c in {1, 3},
// candidate count == expected_ibox_len = (h>>5)*(w>>5) + (h>>4)*(w>>4) + (h>>3)*(w>>3),
// num_channels >= 6 (5 + nc), FP32 data type, rejects "pose" in model path,
// enforces allow_auto_1out, and resolves channels_first vs boxes_first.
ObbLayoutAInfo resolve_obb_layout_a(const ObbTensorSpec& input_spec,
                                    const ObbTensorSpec& output_spec,
                                    bool allow_auto_1out,
                                    const char* model_path = nullptr);

inline bool validate_obb_layout_a(const ObbTensorSpec& input_spec,
                                  const ObbTensorSpec& output_spec,
                                  bool allow_auto_1out,
                                  const char* model_path = nullptr) {
    return resolve_obb_layout_a(input_spec, output_spec, allow_auto_1out, model_path).valid;
}

// Resolve & validate 9-output Layout B structure across 3 scales.
// Resolves is_nchw and interleaved vs grouped by matching spatial (H, W) per scale.
// Resolves is_nchw from box scales where grid != 64 (box_c == 64).
// Requires identical (H, W) across (box_s, cls_s, ang_s) at each scale,
// box_c == 64, ang_c == 1, cls_c >= 1 across all 3 scales, and INT8 or FP32 data type.
ObbLayoutBInfo resolve_obb_layout_b(const ObbTensorSpec* specs, size_t num_specs);

// Convert ObbBox to ObbGeom.
// If content_w > 0 && content_h > 0, maps AABB (clamped to [0, 1]) and corners (un-clamped)
// into visible content rect [content_x, content_y, content_w, content_h].
// Otherwise normalizes to [0, 1] of model input [img_w, img_h].
ObbGeom obb_to_geom(const ObbBox& box, float img_w, float img_h,
                    float content_x = 0.0f, float content_y = 0.0f,
                    float content_w = 0.0f, float content_h = 0.0f);

// Closed-form Gaussian ProbIoU between two rotated bounding boxes
float prob_iou(const ObbBox& b1, const ObbBox& b2);

// Greedy rotated Non-Maximum Suppression (NMS)
std::vector<ObbBox> obb_nms(std::vector<ObbBox>& boxes, float iou_threshold, bool class_aware = true);

// Decode Layout A: Ultralytics single-output FP32 [1, 5 + nc, N]
// Channels per candidate: cx, cy, w, h, cls[0..nc-1], angle_rad
std::vector<ObbBox> decode_obb_layout_a(const float* data, int num_channels, int num_boxes,
                                        float score_threshold, bool channels_first = true);

// Decode Layout B: Decoupled 9-output (INT8 or FP32 across 3 scales)
// tensors: array of 9 tensor views
// img_w, img_h: model input dimensions
std::vector<ObbBox> decode_obb_layout_b(const ObbTensorView* tensors, size_t num_tensors,
                                        float img_w, float img_h, float score_threshold);

}  // namespace rv

#endif  // _REBOT_VISION_OBB_GEOM_H_
