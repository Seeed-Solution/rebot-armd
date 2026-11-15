#include "obb_geom.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace rv {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kHalfPi = 1.57079632679489661923f;

inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

inline float inverse_sigmoid(float x) {
    float d = 1.0f - x;
    if (d < 1e-7f) d = 1e-7f;
    if (x < 1e-7f) x = 1e-7f;
    return std::log(x / d);
}

inline float dequantize(int8_t v, float scale, int32_t zero_point) {
    return static_cast<float>(static_cast<int32_t>(v) - zero_point) * scale;
}

}  // namespace

ObbGeom obb_to_geom(const ObbBox& box, float img_w, float img_h,
                    float content_x, float content_y,
                    float content_w, float content_h) {
    ObbGeom geom;
    geom.score = box.score;
    geom.class_id = box.class_id;

    // Ultralytics OBB orientation:
    // w is along angle_rad (cos a, sin a), h is along angle_rad + pi/2 (-sin a, cos a).
    const float a = box.angle_rad;
    const float cos_a = std::cos(a);
    const float sin_a = std::sin(a);
    const float vx = 0.5f * box.w * cos_a;
    const float vy = 0.5f * box.w * sin_a;
    const float ux = -0.5f * box.h * sin_a;
    const float uy = 0.5f * box.h * cos_a;

    // 4 corners in perimeter order
    const std::array<std::array<float, 2>, 4> pts_px = {{
        {box.cx + vx + ux, box.cy + vy + uy},
        {box.cx - vx + ux, box.cy - vy + uy},
        {box.cx - vx - ux, box.cy - vy - uy},
        {box.cx + vx - ux, box.cy + vy - uy}
    }};

    const float grasp_w = std::min(box.w, box.h);
    const float length_p = std::max(box.w, box.h);
    geom.grasp_width_px = std::nearbyint(grasp_w * 100.0f) / 100.0f;
    geom.length_px = std::nearbyint(length_p * 100.0f) / 100.0f;

    // Long side angle: if w >= h use angle_rad, else angle_rad + pi/2
    const float long_side_rad = (box.w >= box.h) ? a : (a + kHalfPi);
    float angle_deg = long_side_rad * (180.0f / kPi);

    // Wrap to [-90, 90) matching mask_geom.cpp
    angle_deg = std::fmod(angle_deg + 90.0f, 180.0f);
    if (angle_deg < 0.0f) angle_deg += 180.0f;
    angle_deg -= 90.0f;

    float rounded_angle = std::nearbyint(angle_deg * 100.0f) / 100.0f;
    if (rounded_angle >= 90.0f - 1e-6f) {
        rounded_angle = -90.0f;
    }
    if (std::abs(rounded_angle) < 1e-6f) {
        rounded_angle = 0.0f;
    }
    geom.angle_deg = rounded_angle;

    // angle_conf: (length_px >= 12.0f && length_px > 0.0f) ? clamp(1.0f - grasp_width_px / length_px, 0.0f, 1.0f) : 0.0f
    float conf = 0.0f;
    if (geom.length_px >= 12.0f && geom.length_px > 0.0f) {
        conf = std::clamp(1.0f - (geom.grasp_width_px / geom.length_px), 0.0f, 1.0f);
    }
    float rounded_conf = std::nearbyint(conf * 1000.0f) / 1000.0f;
    rounded_conf = std::clamp(rounded_conf, 0.0f, 1.0f);
    if (std::abs(rounded_conf) < 1e-6f) {
        rounded_conf = 0.0f;
    }
    geom.angle_conf = rounded_conf;

    const bool use_content = (content_w > 0.0f && content_h > 0.0f);
    const float ref_x = use_content ? content_x : 0.0f;
    const float ref_y = use_content ? content_y : 0.0f;
    const float ref_w = use_content ? content_w : (img_w > 0.0f ? img_w : 1.0f);
    const float ref_h = use_content ? content_h : (img_h > 0.0f ? img_h : 1.0f);

    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();

    for (int i = 0; i < 4; ++i) {
        // Corners unmapped into content space without clamping so polygon doesn't shear
        const float u = (pts_px[i][0] - ref_x) / ref_w;
        const float v = (pts_px[i][1] - ref_y) / ref_h;
        geom.obb_corners[i][0] = u;
        geom.obb_corners[i][1] = v;
        min_x = std::min(min_x, u);
        max_x = std::max(max_x, u);
        min_y = std::min(min_y, v);
        max_y = std::max(max_y, v);
    }

    // AABB is clamped to [0, 1]
    const float c_min_x = std::clamp(min_x, 0.0f, 1.0f);
    const float c_max_x = std::clamp(max_x, 0.0f, 1.0f);
    const float c_min_y = std::clamp(min_y, 0.0f, 1.0f);
    const float c_max_y = std::clamp(max_y, 0.0f, 1.0f);

    geom.x = (c_min_x + c_max_x) * 0.5f;
    geom.y = (c_min_y + c_max_y) * 0.5f;
    geom.w = c_max_x - c_min_x;
    geom.h = c_max_y - c_min_y;

    return geom;
}

namespace {

/** Covariance terms of a box's Gaussian (ProbIoU), computed once per box. */
struct ObbCov {
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
};

ObbCov obb_cov(const ObbBox& box) {
    const float a = (box.w * box.w) / 12.0f;
    const float b_var = (box.h * box.h) / 12.0f;
    const float cos_t = std::cos(box.angle_rad);
    const float sin_t = std::sin(box.angle_rad);
    const float cos_sq = cos_t * cos_t;
    const float sin_sq = sin_t * sin_t;
    ObbCov s;
    s.a = a * cos_sq + b_var * sin_sq;
    s.b = a * sin_sq + b_var * cos_sq;
    s.c = (a - b_var) * cos_t * sin_t;
    return s;
}

float prob_iou_cov(const ObbBox& b1, const ObbCov& s1, const ObbBox& b2, const ObbCov& s2) {
    constexpr float eps = 1e-7f;

    const float sa = s1.a + s2.a;
    const float sb = s1.b + s2.b;
    const float sc = s1.c + s2.c;
    const float denom = sa * sb - sc * sc + eps;

    const float dx = b1.cx - b2.cx;
    const float dy = b1.cy - b2.cy;

    const float t1 = ((sa * dy * dy + sb * dx * dx) / denom) * 0.25f;
    const float t2 = ((sc * (b2.cx - b1.cx) * (b1.cy - b2.cy)) / denom) * 0.5f;

    const float det1 = std::max(0.0f, s1.a * s1.b - s1.c * s1.c);
    const float det2 = std::max(0.0f, s2.a * s2.b - s2.c * s2.c);
    const float ratio = denom / (4.0f * std::sqrt(det1 * det2) + eps);
    const float t3 = 0.5f * std::log(std::max(1e-7f, ratio));

    const float bd = std::clamp(t1 + t2 + t3, 0.0f, 100.0f);
    const float hd = std::sqrt(std::max(0.0f, 1.0f - std::exp(-bd)));
    return std::clamp(1.0f - hd, 0.0f, 1.0f);
}

}  // namespace

float prob_iou(const ObbBox& b1, const ObbBox& b2) {
    return prob_iou_cov(b1, obb_cov(b1), b2, obb_cov(b2));
}

std::vector<ObbBox> obb_nms(std::vector<ObbBox>& boxes, float iou_threshold, bool class_aware,
                            size_t max_nms, size_t max_det) {
    std::vector<ObbBox> out;
    if (boxes.empty() || max_det == 0) return out;

    std::sort(boxes.begin(), boxes.end(), [](const ObbBox& a, const ObbBox& b) {
        return a.score > b.score;
    });
    if (boxes.size() > max_nms) boxes.resize(max_nms);  // the top max_nms by score

    std::vector<ObbCov> cov(boxes.size());
    for (size_t i = 0; i < boxes.size(); ++i) cov[i] = obb_cov(boxes[i]);

    std::vector<bool> suppressed(boxes.size(), false);
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (suppressed[i]) continue;
        const auto& b1 = boxes[i];
        out.push_back(b1);
        if (out.size() >= max_det) break;

        for (size_t j = i + 1; j < boxes.size(); ++j) {
            if (suppressed[j]) continue;
            const auto& b2 = boxes[j];
            if (class_aware && b1.class_id != b2.class_id) continue;

            const float iou = prob_iou_cov(b1, cov[i], b2, cov[j]);
            if (iou > iou_threshold) {
                suppressed[j] = true;
            }
        }
    }

    return out;
}

std::vector<ObbBox> decode_obb_layout_a(const float* data, int num_channels, int num_boxes,
                                        float score_threshold, bool channels_first) {
    std::vector<ObbBox> boxes;
    if (!data || num_channels < 6 || num_boxes <= 0) return boxes;

    const int num_classes = num_channels - 5;
    boxes.reserve(static_cast<size_t>(num_boxes) / 4);

    for (int i = 0; i < num_boxes; ++i) {
        // Find best class score among channels 4 .. 4 + num_classes - 1
        float max_cls = -1.0f;
        int best_cls = -1;
        for (int c = 0; c < num_classes; ++c) {
            const int ch = 4 + c;
            const size_t idx = channels_first
                ? (static_cast<size_t>(ch) * num_boxes + i)
                : (static_cast<size_t>(i) * num_channels + ch);
            const float score = data[idx];
            if (score > max_cls) {
                max_cls = score;
                best_cls = c;
            }
        }

        if (max_cls < score_threshold) continue;

        auto get_val = [&](int ch) -> float {
            const size_t idx = channels_first
                ? (static_cast<size_t>(ch) * num_boxes + i)
                : (static_cast<size_t>(i) * num_channels + ch);
            return data[idx];
        };

        ObbBox b;
        b.cx = get_val(0);
        b.cy = get_val(1);
        b.w = get_val(2);
        b.h = get_val(3);
        b.score = max_cls;
        b.class_id = best_cls;
        b.angle_rad = get_val(4 + num_classes);

        if (!(b.w > 0.0f && b.h > 0.0f) ||
            !std::isfinite(b.cx) || !std::isfinite(b.cy) || !std::isfinite(b.angle_rad) ||
            !std::isfinite(b.w) || !std::isfinite(b.h) || !std::isfinite(b.score)) {
            continue;
        }
        boxes.push_back(b);
    }

    return boxes;
}

ObbLayoutAInfo resolve_obb_layout_a(const ObbTensorSpec& input_spec,
                                    const ObbTensorSpec& output_spec,
                                    bool allow_auto_1out,
                                    const char* model_path) {
    ObbLayoutAInfo info;
    info.valid = false;
    if (!allow_auto_1out) return info;

    if (model_path) {
        std::string p(model_path);
        std::transform(p.begin(), p.end(), p.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (p.find("pose") != std::string::npos) {
            return info;
        }
    }

    if (output_spec.dtype != ObbDataType::FP32) {
        return info;
    }

    if (input_spec.num_dims != 4 || input_spec.dims[0] != 1) {
        return info;
    }

    int in_c = 0, in_h = 0, in_w = 0;
    if (input_spec.dims[1] == 3 || input_spec.dims[1] == 1) {
        in_c = input_spec.dims[1];
        in_h = input_spec.dims[2];
        in_w = input_spec.dims[3];
    } else if (input_spec.dims[3] == 3 || input_spec.dims[3] == 1) {
        in_h = input_spec.dims[1];
        in_w = input_spec.dims[2];
        in_c = input_spec.dims[3];
    } else {
        return info;
    }

    if (in_c != 3 && in_c != 1) return info;
    if (in_h < 32 || (in_h % 32 != 0)) return info;
    if (in_w < 32 || (in_w % 32 != 0)) return info;

    const int expected_ibox_len = (in_h >> 5) * (in_w >> 5) +
                                  (in_h >> 4) * (in_w >> 4) +
                                  (in_h >> 3) * (in_w >> 3);

    if (output_spec.num_dims == 3) {
        if (output_spec.dims[0] != 1) return info;
        if (output_spec.dims[1] >= 6 && output_spec.dims[2] == expected_ibox_len) {
            info.channels_first = true;
            info.num_channels = output_spec.dims[1];
            info.num_boxes = output_spec.dims[2];
            info.valid = true;
            return info;
        } else if (output_spec.dims[2] >= 6 && output_spec.dims[1] == expected_ibox_len) {
            info.channels_first = false;
            info.num_channels = output_spec.dims[2];
            info.num_boxes = output_spec.dims[1];
            info.valid = true;
            return info;
        }
    } else if (output_spec.num_dims == 2) {
        if (output_spec.dims[0] >= 6 && output_spec.dims[1] == expected_ibox_len) {
            info.channels_first = true;
            info.num_channels = output_spec.dims[0];
            info.num_boxes = output_spec.dims[1];
            info.valid = true;
            return info;
        } else if (output_spec.dims[1] >= 6 && output_spec.dims[0] == expected_ibox_len) {
            info.channels_first = false;
            info.num_channels = output_spec.dims[1];
            info.num_boxes = output_spec.dims[0];
            info.valid = true;
            return info;
        }
    }

    return info;
}

ObbLayoutBInfo resolve_obb_layout_b(const ObbTensorSpec* specs, size_t num_specs) {
    ObbLayoutBInfo info;
    info.valid = false;
    if (!specs || num_specs != 9) return info;

    const ObbDataType common_dtype = specs[0].dtype;
    if (common_dtype != ObbDataType::FP32 && common_dtype != ObbDataType::INT8) {
        return info;
    }

    for (size_t i = 0; i < 9; ++i) {
        if (specs[i].num_dims != 4) return info;
        if (specs[i].dims[0] != 1) return info;
        if (specs[i].dtype != common_dtype) return info;
    }

    struct CandidateLayout {
        bool is_interleaved;
        std::array<size_t, 3> box;
        std::array<size_t, 3> cls;
        std::array<size_t, 3> ang;
    };

    const std::array<CandidateLayout, 2> candidates = {{
        {true,  {0, 3, 6}, {1, 4, 7}, {2, 5, 8}},
        {false, {0, 1, 2}, {3, 4, 5}, {6, 7, 8}}
    }};

    for (const auto& cand : candidates) {
        int nchw_votes = 0;
        int nhwc_votes = 0;
        bool has_ambiguous_scale = false;

        for (int s = 0; s < 3; ++s) {
            const auto& b = specs[cand.box[s]];
            const bool c_at_1 = (b.dims[1] == 64);
            const bool c_at_3 = (b.dims[3] == 64);
            if (!c_at_1 && !c_at_3) {
                nchw_votes = nhwc_votes = -1;
                break;
            }
            if (c_at_1 && c_at_3) {
                has_ambiguous_scale = true;
            } else if (c_at_1) {
                nchw_votes++;
            } else {
                nhwc_votes++;
            }
        }

        if (nchw_votes < 0) continue;
        if (nchw_votes > 0 && nhwc_votes > 0) continue;

        bool is_nchw = true;
        if (nhwc_votes > 0) {
            is_nchw = false;
        } else if (nchw_votes > 0) {
            is_nchw = true;
        } else if (has_ambiguous_scale) {
            is_nchw = true;
        } else {
            continue;
        }

        std::array<int, 3> grid_h{};
        std::array<int, 3> grid_w{};
        int common_nc = -1;
        bool valid_cand = true;

        for (int s = 0; s < 3; ++s) {
            const auto& t_box = specs[cand.box[s]];
            const auto& t_cls = specs[cand.cls[s]];
            const auto& t_ang = specs[cand.ang[s]];

            const int box_c = is_nchw ? t_box.dims[1] : t_box.dims[3];
            const int b_h   = is_nchw ? t_box.dims[2] : t_box.dims[1];
            const int b_w   = is_nchw ? t_box.dims[3] : t_box.dims[2];

            if (box_c != 64 || b_h <= 0 || b_w <= 0) {
                valid_cand = false;
                break;
            }

            const int cls_c = is_nchw ? t_cls.dims[1] : t_cls.dims[3];
            const int c_h   = is_nchw ? t_cls.dims[2] : t_cls.dims[1];
            const int c_w   = is_nchw ? t_cls.dims[3] : t_cls.dims[2];

            if (c_h != b_h || c_w != b_w || cls_c < 1) {
                valid_cand = false;
                break;
            }

            if (s == 0) {
                common_nc = cls_c;
            } else if (cls_c != common_nc) {
                valid_cand = false;
                break;
            }

            const int ang_c = is_nchw ? t_ang.dims[1] : t_ang.dims[3];
            const int a_h   = is_nchw ? t_ang.dims[2] : t_ang.dims[1];
            const int a_w   = is_nchw ? t_ang.dims[3] : t_ang.dims[2];

            if (a_h != b_h || a_w != b_w || ang_c != 1) {
                valid_cand = false;
                break;
            }

            grid_h[s] = b_h;
            grid_w[s] = b_w;
        }

        if (!valid_cand) continue;
        if (grid_h[0] == grid_h[1] && grid_w[0] == grid_w[1]) continue;
        if (grid_h[1] == grid_h[2] && grid_w[1] == grid_w[2]) continue;

        info.valid = true;
        info.is_nchw = is_nchw;
        info.is_interleaved = cand.is_interleaved;
        info.num_classes = common_nc;
        info.dtype = common_dtype;
        info.box_indices = cand.box;
        info.cls_indices = cand.cls;
        info.ang_indices = cand.ang;
        info.grid_h = grid_h;
        info.grid_w = grid_w;
        return info;
    }

    return info;
}

std::vector<ObbBox> decode_obb_layout_b(const ObbTensorView* tensors, size_t num_tensors,
                                        float img_w, float img_h, float score_threshold) {
    std::vector<ObbBox> boxes;
    if (!tensors || num_tensors < 9 || img_w <= 0.0f || img_h <= 0.0f) return boxes;
    for (size_t i = 0; i < 9; ++i) {
        if (!tensors[i].data) return boxes;
    }

    std::array<ObbTensorSpec, 9> specs;
    for (size_t i = 0; i < 9; ++i) {
        specs[i].dtype = tensors[i].is_int8 ? ObbDataType::INT8 : ObbDataType::FP32;
        specs[i].num_dims = tensors[i].num_dims;
        specs[i].dims = tensors[i].dims;
    }

    const auto info = resolve_obb_layout_b(specs.data(), 9);
    if (!info.valid) return boxes;

    const bool is_nchw = info.is_nchw;
    const int num_classes = info.num_classes;
    const float score_thresh_non_sig = inverse_sigmoid(score_threshold);

    size_t max_grid_l = 0;
    for (int s = 0; s < 3; ++s) {
        if (info.grid_h[s] > 0 && info.grid_w[s] > 0) {
            const size_t l = static_cast<size_t>(info.grid_h[s]) * static_cast<size_t>(info.grid_w[s]);
            if (l > max_grid_l) max_grid_l = l;
        }
    }

    std::vector<int8_t> cell_max_s8;
    std::vector<float> cell_max_f32;
    if (is_nchw && max_grid_l > 0 && num_classes > 1) {
        if (info.dtype == ObbDataType::INT8) {
            cell_max_s8.resize(max_grid_l);
        } else if (info.dtype == ObbDataType::FP32) {
            cell_max_f32.resize(max_grid_l);
        }
    }

    for (int s = 0; s < 3; ++s) {
        const auto& t_box = tensors[info.box_indices[s]];
        const auto& t_cls = tensors[info.cls_indices[s]];
        const auto& t_ang = tensors[info.ang_indices[s]];
        const int grid_h = info.grid_h[s];
        const int grid_w = info.grid_w[s];
        if (grid_h <= 0 || grid_w <= 0) continue;

        const size_t grid_l = static_cast<size_t>(grid_h) * static_cast<size_t>(grid_w);
        const float stride = img_w / static_cast<float>(grid_w);

        auto emit_cell = [&](int gy, int gx, size_t cell_idx, int best_cls, float score) {
            auto read_box = [&](int ch) -> float {
                const size_t idx = is_nchw
                    ? (static_cast<size_t>(ch) * grid_l + cell_idx)
                    : (cell_idx * 64 + static_cast<size_t>(ch));
                if (t_box.is_int8) {
                    const auto* p = static_cast<const int8_t*>(t_box.data);
                    return dequantize(p[idx], t_box.scale, t_box.zero_point);
                } else {
                    const auto* p = static_cast<const float*>(t_box.data);
                    return p[idx];
                }
            };

            auto read_ang = [&]() -> float {
                // ang_c == 1, so offset is cell_idx for both NCHW and NHWC
                if (t_ang.is_int8) {
                    const auto* p = static_cast<const int8_t*>(t_ang.data);
                    return dequantize(p[cell_idx], t_ang.scale, t_ang.zero_point);
                } else {
                    const auto* p = static_cast<const float*>(t_ang.data);
                    return p[cell_idx];
                }
            };

            // Box DFL softmax across 4 coordinates (16 bins each)
            float dist[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (int b = 0; b < 4; ++b) {
                float bin_vals[16];
                float max_bin = -1e9f;
                for (int k = 0; k < 16; ++k) {
                    const int ch = b * 16 + k;
                    const float val = read_box(ch);
                    bin_vals[k] = val;
                    if (val > max_bin) max_bin = val;
                }

                float exp_sum = 0.0f;
                float exp_v[16];
                for (int k = 0; k < 16; ++k) {
                    exp_v[k] = std::exp(bin_vals[k] - max_bin);
                    exp_sum += exp_v[k];
                }

                float acc = 0.0f;
                if (exp_sum > 1e-9f) {
                    for (int k = 0; k < 16; ++k) {
                        acc += (exp_v[k] / exp_sum) * static_cast<float>(k);
                    }
                }
                dist[b] = acc * stride;
            }

            // Angle decoding: a = (sigmoid(raw_ang) - 0.25f) * pi
            const float raw_ang = read_ang();
            const float a = (sigmoid(raw_ang) - 0.25f) * kPi;

            // dist2rbox
            const float lt_x = dist[0];
            const float lt_y = dist[1];
            const float rb_x = dist[2];
            const float rb_y = dist[3];

            const float xf = (rb_x - lt_x) * 0.5f;
            const float yf = (rb_y - lt_y) * 0.5f;
            const float cos_a = std::cos(a);
            const float sin_a = std::sin(a);
            const float dx = xf * cos_a - yf * sin_a;
            const float dy = xf * sin_a + yf * cos_a;

            ObbBox obb;
            obb.cx = (static_cast<float>(gx) + 0.5f) * stride + dx;
            obb.cy = (static_cast<float>(gy) + 0.5f) * stride + dy;
            obb.w = lt_x + rb_x;
            obb.h = lt_y + rb_y;
            obb.angle_rad = a;
            obb.score = score;
            obb.class_id = best_cls;

            if (!(obb.w > 0.0f && obb.h > 0.0f) ||
                !std::isfinite(obb.cx) || !std::isfinite(obb.cy) || !std::isfinite(obb.angle_rad) ||
                !std::isfinite(obb.w) || !std::isfinite(obb.h) || !std::isfinite(obb.score)) {
                return;
            }
            boxes.push_back(obb);
        };

        const bool is_valid_int8 = t_cls.is_int8 && t_cls.scale > 1e-30f && std::isfinite(t_cls.scale) &&
                                   std::abs(static_cast<double>(t_cls.zero_point)) < 1e7;

        if (is_nchw && is_valid_int8) {
            // Path A: Fast Path NCHW INT8
            int q_min = 128;
            for (int q = -128; q <= 127; ++q) {
                const float dq = dequantize(static_cast<int8_t>(q), t_cls.scale, t_cls.zero_point);
                if (dq > -1e9f && dq >= score_thresh_non_sig && sigmoid(dq) >= score_threshold) {
                    q_min = q;
                    break;
                }
            }

            if (q_min > 127) {
                continue;
            }

            const auto* cls_ptr = static_cast<const int8_t*>(t_cls.data);
            if (num_classes == 1) {
                for (size_t n = 0; n < grid_l; ++n) {
                    const int8_t val_q = cls_ptr[n];
                    if (static_cast<int>(val_q) < q_min) continue;
                    const float score = sigmoid(dequantize(val_q, t_cls.scale, t_cls.zero_point));
                    if (score < score_threshold) continue;
                    const int gy = static_cast<int>(n / static_cast<size_t>(grid_w));
                    const int gx = static_cast<int>(n % static_cast<size_t>(grid_w));
                    emit_cell(gy, gx, n, 0, score);
                }
            } else {
                int8_t* dst = cell_max_s8.data();
                std::memcpy(dst, cls_ptr, grid_l);
                for (int c = 1; c < num_classes; ++c) {
                    const int8_t* plane = cls_ptr + static_cast<size_t>(c) * grid_l;
                    for (size_t n = 0; n < grid_l; ++n) {
                        const int8_t v = plane[n];
                        if (v > dst[n]) {
                            dst[n] = v;
                        }
                    }
                }

                for (size_t n = 0; n < grid_l; ++n) {
                    const int8_t max_q = dst[n];
                    if (static_cast<int>(max_q) < q_min) continue;
                    int best_cls = 0;
                    for (int c = 0; c < num_classes; ++c) {
                        if (cls_ptr[static_cast<size_t>(c) * grid_l + n] == max_q) {
                            best_cls = c;
                            break;
                        }
                    }
                    const float score = sigmoid(dequantize(max_q, t_cls.scale, t_cls.zero_point));
                    if (score < score_threshold) continue;
                    const int gy = static_cast<int>(n / static_cast<size_t>(grid_w));
                    const int gx = static_cast<int>(n % static_cast<size_t>(grid_w));
                    emit_cell(gy, gx, n, best_cls, score);
                }
            }
        } else if (is_nchw && !t_cls.is_int8) {
            // Path B: Fast Path NCHW FP32
            const auto* cls_ptr = static_cast<const float*>(t_cls.data);
            if (num_classes == 1) {
                for (size_t n = 0; n < grid_l; ++n) {
                    const float max_cls = cls_ptr[n];
                    if (max_cls < score_thresh_non_sig) continue;
                    const float score = sigmoid(max_cls);
                    if (score < score_threshold) continue;
                    const int gy = static_cast<int>(n / static_cast<size_t>(grid_w));
                    const int gx = static_cast<int>(n % static_cast<size_t>(grid_w));
                    emit_cell(gy, gx, n, 0, score);
                }
            } else {
                float* dst = cell_max_f32.data();
                std::fill_n(dst, grid_l, -1e9f);
                for (int c = 0; c < num_classes; ++c) {
                    const float* plane = cls_ptr + static_cast<size_t>(c) * grid_l;
                    for (size_t n = 0; n < grid_l; ++n) {
                        const float v = plane[n];
                        if (v > dst[n]) {
                            dst[n] = v;
                        }
                    }
                }

                for (size_t n = 0; n < grid_l; ++n) {
                    const float max_cls = dst[n];
                    if (max_cls < score_thresh_non_sig) continue;
                    const float score = sigmoid(max_cls);
                    if (score < score_threshold) continue;
                    int best_cls = 0;
                    for (int c = 0; c < num_classes; ++c) {
                        if (cls_ptr[static_cast<size_t>(c) * grid_l + n] == max_cls) {
                            best_cls = c;
                            break;
                        }
                    }
                    const int gy = static_cast<int>(n / static_cast<size_t>(grid_w));
                    const int gx = static_cast<int>(n % static_cast<size_t>(grid_w));
                    emit_cell(gy, gx, n, best_cls, score);
                }
            }
        } else {
            // Fallback Path: Generic NHWC or edge cases
            auto read_cls_elem = [&](int c, int gy, int gx) -> float {
                size_t idx = 0;
                if (is_nchw) {
                    idx = static_cast<size_t>(c) * grid_l + static_cast<size_t>(gy) * static_cast<size_t>(grid_w) + static_cast<size_t>(gx);
                } else {
                    idx = (static_cast<size_t>(gy) * static_cast<size_t>(grid_w) + static_cast<size_t>(gx)) * static_cast<size_t>(num_classes) + static_cast<size_t>(c);
                }
                if (t_cls.is_int8) {
                    const auto* p = static_cast<const int8_t*>(t_cls.data);
                    return dequantize(p[idx], t_cls.scale, t_cls.zero_point);
                } else {
                    const auto* p = static_cast<const float*>(t_cls.data);
                    return p[idx];
                }
            };

            for (int gy = 0; gy < grid_h; ++gy) {
                for (int gx = 0; gx < grid_w; ++gx) {
                    const size_t cell_idx = static_cast<size_t>(gy) * static_cast<size_t>(grid_w) + static_cast<size_t>(gx);
                    float max_cls_dequant = -1e9f;
                    int best_cls = -1;
                    for (int c = 0; c < num_classes; ++c) {
                        const float val = read_cls_elem(c, gy, gx);
                        if (val > max_cls_dequant) {
                            max_cls_dequant = val;
                            best_cls = c;
                        }
                    }

                    if (max_cls_dequant < score_thresh_non_sig) continue;
                    const float score = sigmoid(max_cls_dequant);
                    if (score < score_threshold) continue;

                    emit_cell(gy, gx, cell_idx, best_cls, score);
                }
            }
        }
    }

    return boxes;
}

}  // namespace rv
