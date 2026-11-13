#pragma once

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

}  // namespace rv
