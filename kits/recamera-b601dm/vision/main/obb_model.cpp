#include "obb_model.h"

#include <algorithm>
#include <cctype>
#include <string>

namespace yolo {

namespace {

rv::ObbDataType obb_dtype(ma_tensor_type_t type) {
    return (type == MA_TENSOR_TYPE_S8) ? rv::ObbDataType::INT8 :
           (type == MA_TENSOR_TYPE_F32) ? rv::ObbDataType::FP32 : rv::ObbDataType::Unknown;
}

rv::ObbTensorSpec obb_spec(const ma_shape_t& shape, rv::ObbDataType dtype) {
    rv::ObbTensorSpec spec;
    spec.dtype = dtype;
    spec.num_dims = static_cast<int>(shape.size);
    for (size_t d = 0; d < 4 && d < shape.size; ++d) {
        spec.dims[d] = shape.dims[d];
    }
    return spec;
}

}  // namespace

ObbModel::ObbModel(Engine* engine)
    : Detector(engine, "yolo-obb", MA_MODEL_TYPE_UNDEFINED) {
    if (engine) {
        is_layout_a_ = (engine->getOutputSize() == 1);
        const auto& in_shape = engine->getInputShape(0);
        if (in_shape.size == 4) {
            if (in_shape.dims[1] == 3 || in_shape.dims[1] == 1) {
                img_.height = in_shape.dims[2];
                img_.width = in_shape.dims[3];
            } else {
                img_.height = in_shape.dims[1];
                img_.width = in_shape.dims[2];
            }
        }
    }
}

bool ObbModel::isValid(Engine* engine, bool allow_auto_1out, const char* model_path) {
    if (!engine) return false;
    const size_t in_count = engine->getInputSize();
    const size_t out_count = engine->getOutputSize();
    if (in_count != 1) return false;

    if (model_path) {
        std::string p(model_path);
        std::transform(p.begin(), p.end(), p.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (p.find("pose") != std::string::npos) {
            return false;
        }
    }

    if (out_count == 1) {
        const auto in_shape = engine->getInputShape(0);
        const auto out_shape = engine->getOutputShape(0);
        const auto in_tensor = engine->getInput(0);
        const auto out_tensor = engine->getOutput(0);

        const rv::ObbTensorSpec in_spec = obb_spec(in_shape, obb_dtype(in_tensor.type));
        const rv::ObbTensorSpec out_spec = obb_spec(out_shape, obb_dtype(out_tensor.type));
        return rv::validate_obb_layout_a(in_spec, out_spec, allow_auto_1out, model_path);
    }

    if (out_count == 9) {
        std::array<rv::ObbTensorSpec, 9> specs;
        for (size_t i = 0; i < 9; ++i) {
            const auto s = engine->getOutputShape(i);
            const auto t = engine->getOutput(i);
            specs[i] = obb_spec(s, obb_dtype(t.type));
        }

        const auto info = rv::resolve_obb_layout_b(specs.data(), 9);
        return info.valid;
    }

    return false;
}

ma_err_t ObbModel::postprocess() {
    detections_.clear();
    results_.clear();

    const float score_threshold = static_cast<float>(threshold_score_);
    const float nms_threshold = static_cast<float>(threshold_nms_ > 0 ? threshold_nms_ : 0.45);

    const auto in_shape = p_engine_->getInputShape(0);
    float img_w = 640.0f;
    float img_h = 640.0f;
    if (in_shape.size == 4) {
        if (in_shape.dims[1] == 3 || in_shape.dims[1] == 1) {
            img_h = static_cast<float>(in_shape.dims[2]);
            img_w = static_cast<float>(in_shape.dims[3]);
        } else {
            img_h = static_cast<float>(in_shape.dims[1]);
            img_w = static_cast<float>(in_shape.dims[2]);
        }
    }

    std::vector<rv::ObbBox> raw_boxes;
    if (is_layout_a_) {
        const ma_tensor_t t = p_engine_->getOutput(0);
        if (t.type != MA_TENSOR_TYPE_F32 || !t.data.f32) {
            return MA_EINVAL;
        }
        const rv::ObbTensorSpec in_spec = obb_spec(in_shape, rv::ObbDataType::FP32);
        const rv::ObbTensorSpec out_spec = obb_spec(t.shape, rv::ObbDataType::FP32);
        const auto info = rv::resolve_obb_layout_a(in_spec, out_spec, true, nullptr);
        if (!info.valid) {
            return MA_EINVAL;
        }

        raw_boxes = rv::decode_obb_layout_a(t.data.f32, info.num_channels, info.num_boxes,
                                            score_threshold, info.channels_first);
    } else {
        std::array<rv::ObbTensorView, 9> views;
        for (size_t i = 0; i < 9; ++i) {
            const ma_tensor_t t = p_engine_->getOutput(i);
            if (!t.data.data) {
                return MA_EINVAL;
            }
            views[i].data = t.data.data;
            views[i].is_int8 = (t.type == MA_TENSOR_TYPE_S8);
            views[i].scale = t.quant_param.scale;
            views[i].zero_point = t.quant_param.zero_point;
            views[i].num_dims = static_cast<int>(t.shape.size);
            for (size_t d = 0; d < 4 && d < t.shape.size; ++d) {
                views[i].dims[d] = t.shape.dims[d];
            }
        }
        raw_boxes = rv::decode_obb_layout_b(views.data(), 9, img_w, img_h, score_threshold);
    }

    auto nms_boxes = rv::obb_nms(raw_boxes, nms_threshold, true);

    detections_.reserve(nms_boxes.size());
    for (const auto& b : nms_boxes) {
        rv::ObbGeom geom = rv::obb_to_geom(b, img_w, img_h);

        Detection det;
        det.x = geom.x;
        det.y = geom.y;
        det.w = geom.w;
        det.h = geom.h;
        det.confidence = geom.score;
        det.class_id = geom.class_id;
        det.id = 0;
        det.angle_deg = geom.angle_deg;
        det.angle_conf = geom.angle_conf;
        det.grasp_width_px = geom.grasp_width_px;
        det.length_px = geom.length_px;
        det.has_obb = true;
        det.obb_corners = geom.obb_corners;
        det.mask_focus = false;

        detections_.push_back(det);

        ma_bbox_t bbox;
        bbox.x = det.x;
        bbox.y = det.y;
        bbox.w = det.w;
        bbox.h = det.h;
        bbox.score = det.confidence;
        bbox.target = det.class_id;
        results_.push_front(bbox);
    }

    return MA_OK;
}

}  // namespace yolo
