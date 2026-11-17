#include "seg_heads_model.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "mask_geom.h"

namespace yolo {

namespace {

std::array<rv::ObbTensorSpec, rv::kSegHeadsOutputs> output_specs(Engine* engine) {
    std::array<rv::ObbTensorSpec, rv::kSegHeadsOutputs> specs{};
    for (size_t i = 0; i < specs.size(); ++i) {
        const ma_shape_t s = engine->getOutputShape(i);
        const ma_tensor_t t = engine->getOutput(i);
        specs[i].dtype = t.type == MA_TENSOR_TYPE_F32 ? rv::ObbDataType::FP32 : rv::ObbDataType::Unknown;
        specs[i].num_dims = static_cast<int>(s.size);
        for (size_t d = 0; d < 4 && d < s.size; ++d) specs[i].dims[d] = s.dims[d];
    }
    return specs;
}

rv::SegHeadsInfo resolve(Engine* engine) {
    if (!engine || engine->getInputSize() != 1 || engine->getOutputSize() != rv::kSegHeadsOutputs) return {};
    const ma_shape_t in = engine->getInputShape(0);
    if (in.size != 4 || in.dims[3] != 3) return {};  // uint8 NHWC RGB input (fused preprocess)
    const auto specs = output_specs(engine);
    return rv::resolve_seg_heads(specs.data(), specs.size(), in.dims[2], in.dims[1]);
}

}  // namespace

SegHeadsModel::SegHeadsModel(Engine* engine)
    : ma::model::Detector(engine, "yolo11_seg_heads", MA_MODEL_TYPE_UNDEFINED), info_(resolve(engine)) {}

bool SegHeadsModel::isValid(Engine* engine) { return resolve(engine).valid; }

void SegHeadsModel::setFrame(const rv::MaskFocus* focus, float cx, float cy, float cw, float ch) {
    if (focus && focus->active) {
        mask_focus_ = *focus;
    } else {
        mask_focus_.active = false;
    }
    content_x_ = cx;
    content_y_ = cy;
    content_w_ = cw;
    content_h_ = ch;
}

ma_err_t SegHeadsModel::preprocess() {
    if (input_img_ == nullptr) return MA_OK;
    const auto t_pre = std::chrono::steady_clock::now();
    if (!info_.valid || input_img_->physical || input_img_->format != MA_PIXEL_FORMAT_RGB888 ||
        input_img_->width == 0 || input_img_->height == 0 || input_.data.u8 == nullptr) {
        return MA_ENOTSUP;
    }
    frame_w_ = static_cast<float>(input_img_->width);
    frame_h_ = static_cast<float>(input_img_->height);
    const bool whole = content_w_ <= 0 || content_h_ <= 0;
    geom_ = rv::letterbox_geom(whole ? 0 : content_x_, whole ? 0 : content_y_, whole ? frame_w_ : content_w_,
                               whole ? frame_h_ : content_h_, info_.in_w, info_.in_h);
    const int stride = static_cast<int>(input_img_->size / input_img_->height);
    rv::letterbox_rgb(input_img_->data, input_img_->width, input_img_->height, stride, geom_, input_.data.u8,
                      info_.in_w, info_.in_h);
    if (input_.type == MA_TENSOR_TYPE_S8) {
        for (int i = 0; i < input_.size; i++) input_.data.u8[i] -= 128;
    }
    last_pre_ms_ = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t_pre).count();
    return MA_OK;
}

ma_err_t SegHeadsModel::postprocess() {
    const auto t_post = std::chrono::steady_clock::now();
    const ma_err_t err = decode();
    last_post_ms_ = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t_post).count();
    return err;
}

ma_err_t SegHeadsModel::decode() {
    detections_.clear();
    results_.clear();
    last_mask_ms_ = mask_enabled_ ? 0.0f : -1.0f;
    if (!info_.valid) return MA_EINVAL;

    std::array<const float*, rv::kSegHeadsOutputs> outs{};
    for (size_t i = 0; i < outs.size(); ++i) {
        const ma_tensor_t t = p_engine_->getOutput(i);
        if (t.type != MA_TENSOR_TYPE_F32 || !t.data.f32) return MA_EINVAL;
        outs[i] = t.data.f32;
    }
    const float nms = static_cast<float>(threshold_nms_ > 0 ? threshold_nms_ : 0.45);
    const auto kept = rv::seg_nms(
        rv::decode_seg_heads(info_, outs.data(), static_cast<float>(threshold_score_), true, &scratch_), nms);
    if (kept.empty()) {
        mask_focus_.active = false;
        return MA_OK;
    }

    std::vector<rv::SegBoxOut> boxes;
    boxes.reserve(kept.size());
    for (const auto& c : kept) boxes.push_back(rv::seg_box_to_frame(c, info_, geom_, frame_w_, frame_h_));

    // decode order: the focused box first, then by score (as SegModel)
    std::vector<size_t> order(kept.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    int focus_idx = -1;
    if (mask_focus_.active) {
        std::vector<rv::FocusCandidate> cands;
        cands.reserve(kept.size());
        for (size_t i = 0; i < kept.size(); ++i) {
            cands.push_back({boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h, kept[i].score, kept[i].class_id,
                             class_names_ ? class_names_->name(kept[i].class_id)
                                          : rv::ClassNames{}.name(kept[i].class_id)});
        }
        focus_idx = rv::select_focus_box(mask_focus_, cands, content_x_, content_y_, content_w_, content_h_,
                                         frame_w_, frame_h_);
        if (focus_idx > 0) {
            order.erase(order.begin() + focus_idx);
            order.insert(order.begin(), static_cast<size_t>(focus_idx));
        }
    }

    const float* proto = outs[info_.proto];
    int cell_budget = info_.proto_w * info_.proto_h;
    int processed = 0;
    // prototype cell -> frame pixels (4 input px per cell, divided by the letterbox scale)
    const float sx = static_cast<float>(info_.in_w) / info_.proto_w / geom_.scale;
    const float sy = static_cast<float>(info_.in_h) / info_.proto_h / geom_.scale;
    float mask_ms = 0;

    for (size_t i : order) {
        const auto& c = kept[i];
        const auto& b = boxes[i];
        const bool focused = focus_idx >= 0 && static_cast<size_t>(focus_idx) == i;
        Detection det;
        det.x = b.x;
        det.y = b.y;
        det.w = b.w;
        det.h = b.h;
        det.confidence = c.score;
        det.class_id = c.class_id;
        det.id = 0;
        const int cells = b.crop_w * b.crop_h;
        if (mask_enabled_ && processed < mask_max_boxes_ && cells > 0 && (focused || cells <= cell_budget)) {
            const auto t0 = std::chrono::steady_clock::now();
            if (static_cast<size_t>(cells) > mask_buf_.size()) {
                mask_buf_.resize(cells);
                acc_buf_.resize(cells);
            }
            rv::decode_crop_mask(proto, info_.proto_w, info_.proto_h, rv::kSegCoef, c.coef.data(), b.crop_x, b.crop_y,
                                 b.crop_w, b.crop_h, mask_buf_.data(), acc_buf_.data());
            const auto cs = rv::mask_cross_section(rv::MaskView{mask_buf_.data(), b.crop_w, b.crop_h, b.crop_w, sx, sy});
            if (cs.ok) {
                det.angle_deg = static_cast<float>(cs.angle_deg);
                det.angle_conf = static_cast<float>(cs.conf);
                det.grasp_width_px = static_cast<float>(cs.width_px);
            }
            det.mask_focus = focused;
            mask_ms += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
            cell_budget -= cells;
            ++processed;
        }
        detections_.push_back(det);
    }
    std::stable_sort(detections_.begin(), detections_.end(),
                     [](const Detection& a, const Detection& b) { return a.confidence > b.confidence; });
    for (const auto& d : detections_) {
        ma_bbox_t bbox;
        bbox.x = d.x;
        bbox.y = d.y;
        bbox.w = d.w;
        bbox.h = d.h;
        bbox.score = d.confidence;
        bbox.target = d.class_id;
        results_.push_front(bbox);
    }
    mask_focus_.active = false;
    if (mask_enabled_) last_mask_ms_ = mask_ms;
    return MA_OK;
}

}  // namespace yolo
