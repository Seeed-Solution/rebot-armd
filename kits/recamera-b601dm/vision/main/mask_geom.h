#pragma once

#include <cstddef>
#include <cstdint>

namespace rv {

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
