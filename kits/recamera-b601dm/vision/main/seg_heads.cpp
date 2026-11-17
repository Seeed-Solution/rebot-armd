#include "seg_heads.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rv {

namespace {

float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

float iou(const SegCandidate& a, const SegCandidate& b) {
    const float iw = std::min(a.x1, b.x1) - std::max(a.x0, b.x0);
    const float ih = std::min(a.y1, b.y1) - std::max(a.y0, b.y0);
    if (iw <= 0 || ih <= 0) return 0;
    const float inter = iw * ih;
    const float ua = (a.x1 - a.x0) * (a.y1 - a.y0) + (b.x1 - b.x0) * (b.y1 - b.y0) - inter;
    return ua > 0 ? inter / ua : 0;
}

}  // namespace

SegHeadsInfo resolve_seg_heads(const ObbTensorSpec* outputs, size_t num_outputs, int in_w, int in_h) {
    SegHeadsInfo info;
    if (outputs == nullptr || num_outputs != kSegHeadsOutputs || in_w < 32 || in_h < 32 || in_w % 32 || in_h % 32) {
        return info;
    }
    for (size_t i = 0; i < num_outputs; ++i) {
        const ObbTensorSpec& s = outputs[i];
        if (s.dtype != ObbDataType::FP32 || s.num_dims != 4 || s.dims[0] != 1 || s.dims[1] <= 0) return info;
    }
    info.in_w = in_w;
    info.in_h = in_h;

    // the prototypes: input / 4, 32 channels, the only tensor at that grid
    int proto_count = 0;
    for (size_t i = 0; i < num_outputs; ++i) {
        const ObbTensorSpec& s = outputs[i];
        if (s.dims[2] == in_h / 4 && s.dims[3] == in_w / 4) {
            ++proto_count;
            info.proto = i;
        }
    }
    if (proto_count != 1 || outputs[info.proto].dims[1] != kSegCoef) return info;
    info.proto_h = in_h / 4;
    info.proto_w = in_w / 4;

    const int strides[3] = {8, 16, 32};
    for (int k = 0; k < 3; ++k) {
        const int gh = in_h / strides[k];
        const int gw = in_w / strides[k];
        int n = 0, n_box = 0, n_coef = 0, n_cls = 0;
        for (size_t i = 0; i < num_outputs; ++i) {
            const ObbTensorSpec& s = outputs[i];
            if (s.dims[2] != gh || s.dims[3] != gw) continue;
            ++n;
            if (s.dims[1] == 64) {
                info.box[k] = i;
                ++n_box;
            } else if (s.dims[1] == kSegCoef) {
                info.coef[k] = i;
                ++n_coef;
            } else {
                info.cls[k] = i;
                ++n_cls;
                if (k == 0) info.num_classes = s.dims[1];
                else if (s.dims[1] != info.num_classes) return SegHeadsInfo{};
            }
        }
        if (n != 3 || n_box != 1 || n_coef != 1 || n_cls != 1) return SegHeadsInfo{};
        info.grid_h[k] = gh;
        info.grid_w[k] = gw;
        info.stride[k] = strides[k];
    }
    info.valid = true;
    return info;
}

float dfl16(const float* bins, size_t bin_stride) {
    float mx = bins[0];
    for (int k = 1; k < 16; ++k) mx = std::max(mx, bins[k * bin_stride]);
    float sum = 0, acc = 0;
    for (int k = 0; k < 16; ++k) {
        const float e = std::exp(bins[k * bin_stride] - mx);
        sum += e;
        acc += e * static_cast<float>(k);
    }
    return acc / sum;
}

float seg_logit(float threshold) {
    const float t = std::min(std::max(threshold, 1e-6f), 1.0f - 1e-6f);
    return std::log(t / (1.0f - t));
}

std::vector<SegCandidate> decode_seg_heads(const SegHeadsInfo& info, const float* const* outputs, float conf,
                                           bool prefilter, std::vector<float>* scratch) {
    std::vector<SegCandidate> out;
    if (!info.valid || outputs == nullptr) return out;
    const float logit_thr = seg_logit(conf);
    std::vector<float> local;
    std::vector<float>& best = scratch ? *scratch : local;
    std::vector<int> best_cls;

    for (int k = 0; k < 3; ++k) {
        const int gw = info.grid_w[k];
        const size_t hw = static_cast<size_t>(info.grid_h[k]) * gw;
        const float* box = outputs[info.box[k]];
        const float* cls = outputs[info.cls[k]];
        const float* coef = outputs[info.coef[k]];
        if (!box || !cls || !coef) return {};
        const float stride = static_cast<float>(info.stride[k]);

        auto make = [&](size_t idx, float score, int c) {
            SegCandidate cand;
            const float gx = static_cast<float>(idx % gw) + 0.5f;
            const float gy = static_cast<float>(idx / gw) + 0.5f;
            cand.x0 = (gx - dfl16(box + idx, hw)) * stride;
            cand.y0 = (gy - dfl16(box + 16 * hw + idx, hw)) * stride;
            cand.x1 = (gx + dfl16(box + 32 * hw + idx, hw)) * stride;
            cand.y1 = (gy + dfl16(box + 48 * hw + idx, hw)) * stride;
            cand.score = score;
            cand.class_id = c;
            for (int j = 0; j < kSegCoef; ++j) cand.coef[j] = coef[j * hw + idx];
            return cand;
        };

        if (prefilter) {
            // plane by plane: sequential reads of each class plane (the outputs are large, the
            // per-cell walk across 80 planes misses the cache on every read)
            best.assign(cls, cls + hw);
            best_cls.assign(hw, 0);
            for (int c = 1; c < info.num_classes; ++c) {
                const float* plane = cls + static_cast<size_t>(c) * hw;
                for (size_t i = 0; i < hw; ++i) {
                    if (plane[i] > best[i]) {
                        best[i] = plane[i];
                        best_cls[i] = c;
                    }
                }
            }
            for (size_t i = 0; i < hw; ++i) {
                if (best[i] < logit_thr) continue;
                const float score = sigmoid(best[i]);
                if (score < conf) continue;
                out.push_back(make(i, score, best_cls[i]));
            }
        } else {
            for (size_t i = 0; i < hw; ++i) {
                float score = -1;
                int c_best = 0;
                for (int c = 0; c < info.num_classes; ++c) {
                    const float s = sigmoid(cls[static_cast<size_t>(c) * hw + i]);
                    if (s > score) {
                        score = s;
                        c_best = c;
                    }
                }
                // every cell decoded in full, then thresholded
                const SegCandidate cand = make(i, score, c_best);
                if (score >= conf) out.push_back(cand);
            }
        }
    }
    return out;
}

std::vector<SegCandidate> seg_nms(std::vector<SegCandidate> cands, float iou_thr, size_t max_nms, size_t max_det) {
    std::stable_sort(cands.begin(), cands.end(),
                     [](const SegCandidate& a, const SegCandidate& b) { return a.score > b.score; });
    if (cands.size() > max_nms) cands.resize(max_nms);
    std::vector<SegCandidate> kept;
    std::vector<char> gone(cands.size(), 0);
    for (size_t i = 0; i < cands.size() && kept.size() < max_det; ++i) {
        if (gone[i]) continue;
        kept.push_back(cands[i]);
        for (size_t j = i + 1; j < cands.size(); ++j) {
            if (!gone[j] && cands[j].class_id == cands[i].class_id && iou(cands[i], cands[j]) > iou_thr) gone[j] = 1;
        }
    }
    return kept;
}

LetterboxGeom letterbox_geom(float content_x, float content_y, float content_w, float content_h, int in_w,
                             int in_h) {
    LetterboxGeom g;
    g.src_x = content_x;
    g.src_y = content_y;
    g.src_w = content_w > 0 ? content_w : static_cast<float>(in_w);
    g.src_h = content_h > 0 ? content_h : static_cast<float>(in_h);
    g.scale = std::min(static_cast<float>(in_w) / g.src_w, static_cast<float>(in_h) / g.src_h);
    g.dst_w = std::min(in_w, static_cast<int>(std::lround(g.src_w * g.scale)));
    g.dst_h = std::min(in_h, static_cast<int>(std::lround(g.src_h * g.scale)));
    g.pad_x = static_cast<int>(std::lround((in_w - g.dst_w) * 0.5 - 0.1));
    g.pad_y = static_cast<int>(std::lround((in_h - g.dst_h) * 0.5 - 0.1));
    return g;
}

void letterbox_rgb(const uint8_t* frame, int frame_w, int frame_h, int stride, const LetterboxGeom& g,
                   uint8_t* input, int in_w, int in_h, uint8_t pad_value) {
    const size_t row_bytes = static_cast<size_t>(in_w) * 3;
    const int sx0 = static_cast<int>(std::lround(g.src_x));
    const int sy0 = static_cast<int>(std::lround(g.src_y));
    for (int y = 0; y < in_h; ++y) {
        uint8_t* dst = input + y * row_bytes;
        const int cy = y - g.pad_y;
        if (cy < 0 || cy >= g.dst_h) {
            std::memset(dst, pad_value, row_bytes);
            continue;
        }
        const int sy = std::min(frame_h - 1, sy0 + static_cast<int>(cy / g.scale));
        const uint8_t* src = frame + static_cast<size_t>(sy) * stride;
        std::memset(dst, pad_value, static_cast<size_t>(g.pad_x) * 3);
        std::memset(dst + static_cast<size_t>(g.pad_x + g.dst_w) * 3, pad_value,
                    static_cast<size_t>(in_w - g.pad_x - g.dst_w) * 3);
        if (g.scale == 1.0f && sx0 + g.dst_w <= frame_w) {
            std::memcpy(dst + static_cast<size_t>(g.pad_x) * 3, src + static_cast<size_t>(sx0) * 3,
                        static_cast<size_t>(g.dst_w) * 3);
            continue;
        }
        for (int x = 0; x < g.dst_w; ++x) {
            const int sx = std::min(frame_w - 1, sx0 + static_cast<int>(x / g.scale));
            std::memcpy(dst + static_cast<size_t>(g.pad_x + x) * 3, src + static_cast<size_t>(sx) * 3, 3);
        }
    }
}

SegBoxOut seg_box_to_frame(const SegCandidate& c, const SegHeadsInfo& info, const LetterboxGeom& g, float frame_w,
                           float frame_h) {
    SegBoxOut o;
    const float lx = static_cast<float>(g.pad_x), ly = static_cast<float>(g.pad_y);
    const float hx = lx + static_cast<float>(g.dst_w), hy = ly + static_cast<float>(g.dst_h);
    const float x0 = std::min(std::max(c.x0, lx), hx), x1 = std::min(std::max(c.x1, lx), hx);
    const float y0 = std::min(std::max(c.y0, ly), hy), y1 = std::min(std::max(c.y1, ly), hy);
    const float fw = frame_w > 0 ? frame_w : 1.0f, fh = frame_h > 0 ? frame_h : 1.0f;
    const float s = g.scale > 0 ? g.scale : 1.0f;
    const float fx0 = (x0 - lx) / s + g.src_x, fx1 = (x1 - lx) / s + g.src_x;
    const float fy0 = (y0 - ly) / s + g.src_y, fy1 = (y1 - ly) / s + g.src_y;
    o.x = (fx0 + fx1) * 0.5f / fw;
    o.y = (fy0 + fy1) * 0.5f / fh;
    o.w = (fx1 - fx0) / fw;
    o.h = (fy1 - fy0) / fh;
    if (info.in_w > 0 && info.in_h > 0) {
        const float px = static_cast<float>(info.proto_w) / info.in_w, py = static_cast<float>(info.proto_h) / info.in_h;
        const int cx0 = std::max(0, static_cast<int>(std::floor(x0 * px)));
        const int cy0 = std::max(0, static_cast<int>(std::floor(y0 * py)));
        const int cx1 = std::min(info.proto_w, static_cast<int>(std::ceil(x1 * px)));
        const int cy1 = std::min(info.proto_h, static_cast<int>(std::ceil(y1 * py)));
        o.crop_x = cx0;
        o.crop_y = cy0;
        o.crop_w = std::max(0, cx1 - cx0);
        o.crop_h = std::max(0, cy1 - cy0);
    }
    return o;
}

}  // namespace rv
