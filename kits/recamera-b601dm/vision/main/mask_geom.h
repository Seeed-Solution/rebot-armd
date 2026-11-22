#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rv {

struct MaskFocus {
    bool active = false;
    std::string label;  // optional; empty matches any label
    float box_norm[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // [x0, y0, x1, y1] in content image
    int ttl_ms = 2000;
    uint64_t expires_ms = 0;
};

float box_iou(float ax0, float ay0, float ax1, float ay1,
              float bx0, float by0, float bx1, float by1);

struct FocusCandidate {
    float x = 0;  // center x [0, 1] in inference frame
    float y = 0;  // center y [0, 1] in inference frame
    float w = 0;  // width [0, 1] in inference frame
    float h = 0;  // height [0, 1] in inference frame
    float score = 0;
    int class_id = 0;
    std::string label;
};

int select_focus_box(const MaskFocus& focus,
                     const std::vector<FocusCandidate>& candidates,
                     float content_x = 0.0f, float content_y = 0.0f,
                     float content_w = 640.0f, float content_h = 640.0f,
                     float iw = 640.0f, float ih = 640.0f);

struct MaskView {
    const uint8_t* m;
    int w;
    int h;
    int stride;
    float scale_x;
    float scale_y;
};

struct Moments {
    double angle_deg;
    double conf;
};
Moments mask_moments(const MaskView& mask);
Moments mask_moments(const double* xs, const double* ys, size_t n, double scale_x = 1.0, double scale_y = 1.0);

struct CrossSection {
    double angle_deg;
    double conf;
    double width_px;
    bool ok;
};
CrossSection mask_cross_section(const MaskView& mask, int step_deg = 5, double band_px = 8.0);
CrossSection mask_cross_section(const double* xs, const double* ys, size_t n, int step_deg = 5, double band_px = 8.0,
                               double scale_x = 1.0, double scale_y = 1.0);

void decode_crop_mask(
    const float* proto, int proto_w, int proto_h, int num_proto_channels,
    const float* coef,
    int x0, int y0, int crop_w, int crop_h,
    uint8_t* out_mask,
    float* scratch_acc = nullptr
);

/** Most vertices of a detection contour (the perception API's bound; armd refuses more). */
constexpr int kMaxContourVertices = 128;

using Contour = std::vector<std::array<float, 2>>;

/**
 * Exterior contour of the largest 4-connected component of a binary mask (`w` x `h`, `stride`
 * bytes per row, non-zero = set), traced along the cell edges: vertices at cell corners in cell
 * units of the mask (its top-left corner at 0, 0), clockwise in image coordinates. Simplified by
 * one Douglas-Peucker pass at `eps_cells` (an outline over 2048 vertices is first subsampled to
 * 2048, bounding the work), then subsampled to at most `max_vertices`; at least 3 vertices for any
 * non-empty mask. Empty for an empty mask.
 */
Contour mask_exterior_contour(const uint8_t* m, int w, int h, int stride, int max_vertices = kMaxContourVertices,
                              float eps_cells = 0.7f);

/**
 * Where mask cells land in the detection's normalised frame: cell (u, v) of a crop starting at
 * prototype cell (crop_x, crop_y) is model-input pixel ((crop_x + u) * cell_w, (crop_y + v) * cell_h),
 * frame pixel (input - pad) / scale + src, then divided by the frame size (the space of Detection
 * x, y). The 640-square model: cell 4, pad 0, scale 1, src 0, frame 640x640 (VPSS letterboxed the
 * content already); the 10-output seg model: its LetterboxGeom (seg384: pad_y 12).
 */
struct CellToFrame {
    float cell_w = 4, cell_h = 4;
    int crop_x = 0, crop_y = 0;
    float pad_x = 0, pad_y = 0;
    float scale = 1;
    float src_x = 0, src_y = 0;
    float frame_w = 640, frame_h = 640;
};
void contour_to_frame_norm(Contour& c, const CellToFrame& t);

}  // namespace rv
