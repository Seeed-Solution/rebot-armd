#ifndef REBOT_VISION_DEPTH_GEOM_H_
#define REBOT_VISION_DEPTH_GEOM_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rv {

/**
 * 16-bit fixed-point bilinear resize from an RGB888 image (src_w x src_h, src_stride)
 * to (dst_w x dst_h, dst_stride). When dst_stride <= 0, dst_w * 3 is used.
 * Avoids floating-point math in the inner loop.
 */
void resize_rgb888_bilinear(const uint8_t* src, int src_w, int src_h, int src_stride,
                             uint8_t* dst, int dst_w, int dst_h, int dst_stride = 0);

/** Axis-aligned bounding box in content image pixels (e.g. 640x360). */
struct DepthBox {
    float x0 = 0.0f;
    float y0 = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
};

/** Disparity and grasp geometry result for one bounding box. */
struct BoxDepthResult {
    bool valid = false;
    float disp_fg = NAN;
    float disp_bg = NAN;
    float depth_conf = NAN;
    float angle_deg = NAN;
    float angle_conf = NAN;
    float grasp_width_px = NAN;
    float length_px = NAN;
    int jaw_clear = -1;  ///< -1: not evaluated; 0: false (conflict); 1: true (clear)
                         ///< evaluated across the jaw closing direction (see jaw_close_deg)
    int fg_count = 0;
};

/**
 * Analyzes disparity inside target_box using the 384x224 dense disparity map.
 * Maps (gx, gy) to (u_px, v_px) = ((gx+0.5f)*(content_w/384.0f), (gy+0.5f)*(content_h/224.0f))
 * to prevent aspect distortion during PCA and width calculation.
 *
 * @param disp_map Pointer to dense disparity map (grid_w x grid_h floats).
 * @param grid_w Width of disparity map (e.g. 384).
 * @param grid_h Height of disparity map (e.g. 224).
 * @param target_box Target detection box in content pixels.
 * @param other_boxes Other detection boxes to exclude from the border ring.
 * @param content_w Visible content width in pixels (default: 640.0f).
 * @param content_h Visible content height in pixels (default: 360.0f).
 * @param jaw_close_deg Direction the jaws close along, degrees in content pixels (atan2(dv, du)).
 *        The jaw_clear strips lie beyond the foreground's extent along this direction and span
 *        +-20% of its extent across it. NAN (default): the depth PCA minor axis.
 */
BoxDepthResult analyze_box_depth(const float* disp_map, int grid_w, int grid_h,
                                 const DepthBox& target_box,
                                 const std::vector<DepthBox>& other_boxes,
                                 float content_w = 640.0f, float content_h = 360.0f,
                                 float jaw_close_deg = NAN);

/** Overload accepting all boxes and the target box index. */
BoxDepthResult analyze_box_depth(const float* disp_map, int grid_w, int grid_h,
                                 size_t target_idx,
                                 const std::vector<DepthBox>& all_boxes,
                                 float content_w = 640.0f, float content_h = 360.0f,
                                 float jaw_close_deg = NAN);

/** A sparse patch sample of the background table plane. */
struct TableSample {
    float u_px = 0.0f;
    float v_px = 0.0f;
    float disp = 0.0f;
};

/**
 * Samples up to 8x6 = 48 patches across [8%..92%] of the content frame (5x5 grid cells each),
 * excluding cells inside any YOLO box expanded by +15% margin and rejecting patches
 * where internal MAD > 0.05f * max(median, 1e-3f).
 */
std::vector<TableSample> sample_table_disparity(const float* disp_map, int grid_w, int grid_h,
                                               const std::vector<DepthBox>& boxes,
                                               float content_w = 640.0f, float content_h = 360.0f);

}  // namespace rv

#endif  // REBOT_VISION_DEPTH_GEOM_H_
