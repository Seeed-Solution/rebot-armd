#include "seg_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <forward_list>
#include <vector>

#include "mask_geom.h"

namespace yolo {

SegModel::SegModel(Engine* engine)
    : ma::model::Segmentor(engine, "yolo11_seg", MA_MODEL_TYPE_YOLO11_SEG) {
    MA_ASSERT(engine != nullptr);

    bboxes_ = engine->getOutput(0);
    protos_ = engine->getOutput(1);

    num_class_  = bboxes_.shape.dims[1] - 36;
    num_record_ = bboxes_.shape.dims[2];

    const int proto_w = protos_.shape.dims[3] > 0 ? protos_.shape.dims[3] : 160;
    const int proto_h = protos_.shape.dims[2] > 0 ? protos_.shape.dims[2] : 160;
    mask_buf_.resize(proto_w * proto_h);
    acc_buf_.resize(proto_w * proto_h);
}

ma_err_t SegModel::postprocess() {
    detections_.clear();
    last_mask_ms_ = -1.0f;

    if (bboxes_.type != MA_TENSOR_TYPE_F32) {
        return MA_ENOTSUP;
    }

    std::forward_list<ma_bbox_ext_t> multi_level_bboxes;
    auto* data = bboxes_.data.f32;

    const float* scores = data + num_record_ * 4;
    cell_max_f32_.assign(num_record_, -INFINITY);
    float* cell_max = cell_max_f32_.data();
    for (int c = 0; c < num_class_; c++) {
        const float* plane = scores + num_record_ * c;
        for (decltype(num_record_) n = 0; n < num_record_; n++) {
            cell_max[n] = plane[n] > cell_max[n] ? plane[n] : cell_max[n];
        }
    }

    const float iw = img_.width > 0 ? static_cast<float>(img_.width) : 640.0f;
    const float ih = img_.height > 0 ? static_cast<float>(img_.height) : 640.0f;

    for (decltype(num_record_) i = 0; i < num_record_; ++i) {
        if (cell_max[i] < threshold_score_)
            continue;

        int target = num_class_ - 1;
        while (target > 0 && scores[num_record_ * target + i] != cell_max[i]) {
            target--;
        }
        float max = scores[num_record_ * target + i];

        float x = data[i];
        float y = data[i + num_record_];
        float w = data[i + num_record_ * 2];
        float h = data[i + num_record_ * 3];

        ma_bbox_ext_t bbox;
        bbox.level  = 0;
        bbox.index  = i;
        bbox.x      = x / iw;
        bbox.y      = y / ih;
        bbox.w      = w / iw;
        bbox.h      = h / ih;
        bbox.score  = max;
        bbox.target = target;

        multi_level_bboxes.emplace_front(std::move(bbox));
    }

    ma::utils::nms(multi_level_bboxes, threshold_nms_, threshold_score_, false, true);

    if (multi_level_bboxes.empty()) {
        if (mask_enabled_) {
            last_mask_ms_ = 0.0f;
        }
        return MA_OK;
    }

    std::vector<ma_bbox_ext_t> boxes(multi_level_bboxes.begin(), multi_level_bboxes.end());
    std::sort(boxes.begin(), boxes.end(), [](const ma_bbox_ext_t& a, const ma_bbox_ext_t& b) {
        return a.score > b.score;
    });

    const int proto_w = protos_.shape.dims[3] > 0 ? protos_.shape.dims[3] : 160;
    const int proto_h = protos_.shape.dims[2] > 0 ? protos_.shape.dims[2] : 160;
    int cell_budget = proto_w * proto_h;
    int boxes_processed = 0;

    auto clamp = [](int v, int lo, int hi) {
        return std::max(lo, std::min(v, hi));
    };

    float total_mask_time = 0.0f;

    for (const auto& bbox : boxes) {
        Detection det;
        det.x = bbox.x;
        det.y = bbox.y;
        det.w = bbox.w;
        det.h = bbox.h;
        det.confidence = bbox.score;
        det.class_id = bbox.target;
        det.id = 0;
        det.angle_deg = NAN;
        det.angle_conf = NAN;
        det.grasp_width_px = NAN;

        int x0 = clamp(static_cast<int>(std::floor((bbox.x - bbox.w * 0.5f) * proto_w)), 0, proto_w);
        int y0 = clamp(static_cast<int>(std::floor((bbox.y - bbox.h * 0.5f) * proto_h)), 0, proto_h);
        int x1 = clamp(static_cast<int>(std::ceil((bbox.x + bbox.w * 0.5f) * proto_w)), 0, proto_w);
        int y1 = clamp(static_cast<int>(std::ceil((bbox.y + bbox.h * 0.5f) * proto_h)), 0, proto_h);

        int crop_w = x1 - x0;
        int crop_h = y1 - y0;

        float coef[32];
        for (int j = 0; j < 32; ++j) {
            coef[j] = bboxes_.data.f32[bbox.index + num_record_ * (4 + num_class_ + j)];
        }

        if (mask_enabled_ && boxes_processed < mask_max_boxes_ &&
            crop_w > 0 && crop_h > 0 && (crop_w * crop_h <= cell_budget)) {
            auto t0 = std::chrono::steady_clock::now();

            if (static_cast<size_t>(crop_w * crop_h) > mask_buf_.size()) {
                mask_buf_.resize(crop_w * crop_h);
                acc_buf_.resize(crop_w * crop_h);
            }

            rv::decode_crop_mask(protos_.data.f32, proto_w, proto_h, 32, coef,
                                 x0, y0, crop_w, crop_h,
                                 mask_buf_.data(), acc_buf_.data());

            auto cs = rv::mask_cross_section(
                rv::MaskView{mask_buf_.data(), crop_w, crop_h, crop_w, scale_x_, scale_y_});

            if (cs.ok) {
                det.angle_deg = static_cast<float>(cs.angle_deg);
                det.angle_conf = static_cast<float>(cs.conf);
                det.grasp_width_px = static_cast<float>(cs.width_px);
            }

            auto t1 = std::chrono::steady_clock::now();
            total_mask_time += std::chrono::duration<float, std::milli>(t1 - t0).count();

            cell_budget -= crop_w * crop_h;
            boxes_processed++;
        }

        detections_.push_back(det);
    }

    if (mask_enabled_) {
        last_mask_ms_ = total_mask_time;
    }

    return MA_OK;
}

}  // namespace yolo
