#include "detector.h"
#include "seg_model.h"
#include "seg_heads_model.h"
#include "obb_model.h"

#include <algorithm>
#include <utility>

#define TAG "Detector"

namespace yolo {

Detector::Detector()
    : engine_(nullptr),
      model_(nullptr),
      seg_model_(nullptr),
      seg_heads_model_(nullptr),
      obb_model_(nullptr),
      kind_(Kind::Detect),
      threshold_(0.25f),
      input_width_(640),
      input_height_(640),
      initialized_(false),
      detection_id_counter_(0) {}

// Explicit order: the model wrappers hold the engine's tensors, so they go first, then the
// engine (its TPU buffers and the cvimodel registration). A swap destroys the whole object.
Detector::~Detector() {
    if (model_) {
        ma::ModelFactory::remove(model_);
        model_ = nullptr;
    }
    obb_model_.reset();
    seg_heads_model_.reset();
    seg_model_.reset();
    engine_.reset();
}

bool Detector::init(const std::string& model_path, ModelKindHint hint) {
    MA_LOGI(TAG, "Initializing detector with model: %s (hint: %d)", model_path.c_str(), static_cast<int>(hint));

    engine_ = std::make_unique<ma::engine::EngineCVI>();
    ma_err_t ret = engine_->init();
    if (ret != MA_OK) {
        MA_LOGE(TAG, "Failed to initialize CVI engine");
        return false;
    }

    ret = engine_->load(model_path.c_str());
    if (ret != MA_OK) {
        MA_LOGE(TAG, "Failed to load model: %s", model_path.c_str());
        return false;
    }

    // Log tensor info
    int num_inputs = engine_->getInputSize();
    int num_outputs = engine_->getOutputSize();
    MA_LOGI(TAG, "Model loaded: %d inputs, %d outputs", num_inputs, num_outputs);

    // YOLO11-seg cut before the head decode: 10 outputs (3 x box / class / mask coefficients + prototypes)
    if ((hint == ModelKindHint::Segment || hint == ModelKindHint::Auto) && num_outputs == rv::kSegHeadsOutputs &&
        SegHeadsModel::isValid(engine_.get())) {
        seg_heads_model_ = std::make_unique<SegHeadsModel>(engine_.get());
        kind_ = Kind::Segment;
        num_classes_ = seg_heads_model_->numClasses();
        const ma_img_t* model_input = static_cast<const ma_img_t*>(seg_heads_model_->getInput());
        if (model_input) {
            input_width_ = model_input->width;
            input_height_ = model_input->height;
        }
        MA_LOGI(TAG, "Segmentor initialized (type: %s, 10 outputs, input: %dx%d)", seg_heads_model_->getName(),
                input_width_, input_height_);
        initialized_ = true;
        return true;
    }

    if (hint == ModelKindHint::Segment ||
        (hint == ModelKindHint::Auto && num_outputs == 2 && ma::model::Yolo11Seg::isValid(engine_.get()))) {
        if (num_outputs == 2 && ma::model::Yolo11Seg::isValid(engine_.get())) {
            seg_model_ = std::make_unique<SegModel>(engine_.get());
            kind_ = Kind::Segment;
            num_classes_ = seg_model_->numClasses();
            const ma_img_t* model_input = static_cast<const ma_img_t*>(seg_model_->getInput());
            if (model_input) {
                input_width_ = model_input->width;
                input_height_ = model_input->height;
            }
            MA_LOGI(TAG, "Segmentor initialized (type: %s, input: %dx%d)",
                    seg_model_->getName(), input_width_, input_height_);
            initialized_ = true;
            return true;
        } else if (hint == ModelKindHint::Segment) {
            MA_LOGE(TAG, "Requested segment model kind, but model is not a valid segmentor");
            return false;
        }
    }

    if (hint == ModelKindHint::Obb || hint == ModelKindHint::Auto) {
        bool path_has_obb = false;
        std::string lower_path = model_path;
        std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (lower_path.find("obb") != std::string::npos) {
            path_has_obb = true;
        }

        const bool allow_1out = (hint == ModelKindHint::Obb) ? true : path_has_obb;
        if (ObbModel::isValid(engine_.get(), allow_1out, model_path.c_str())) {
            obb_model_ = std::make_unique<ObbModel>(engine_.get());
            kind_ = Kind::Obb;
            num_classes_ = obb_model_->numClasses();
            const ma_img_t* model_input = static_cast<const ma_img_t*>(obb_model_->getInput());
            if (model_input) {
                input_width_ = model_input->width;
                input_height_ = model_input->height;
            }
            MA_LOGI(TAG, "OBB Detector initialized (type: %s, input: %dx%d)",
                    obb_model_->getName(), input_width_, input_height_);
            initialized_ = true;
            return true;
        } else if (hint == ModelKindHint::Obb) {
            MA_LOGE(TAG, "Requested OBB model kind, but model is not a valid OBB model");
            return false;
        }
    }

    kind_ = Kind::Detect;
    // Use ModelFactory to auto-detect model type (YOLO11, YOLO26, YOLOv8, etc.)
    ma::Model* m = ma::ModelFactory::create(engine_.get());
    if (m == nullptr) {
        MA_LOGE(TAG, "ModelFactory::create failed - model format not recognized");
        return false;
    }

    if (m->getOutputType() != MA_OUTPUT_TYPE_BBOX) {
        MA_LOGE(TAG, "Model output type is not BBOX, got: %d", m->getOutputType());
        ma::ModelFactory::remove(m);
        return false;
    }

    model_ = static_cast<ma::model::Detector*>(m);
    std::vector<std::vector<int>> shapes;
    for (size_t i = 0; i < static_cast<size_t>(engine_->getOutputSize()); ++i) {
        const ma_shape_t s = engine_->getOutputShape(i);
        shapes.emplace_back(s.dims, s.dims + s.size);
    }
    num_classes_ = rv::detect_num_classes(shapes);

    // Get input dimensions from model
    const ma_img_t* model_input = static_cast<const ma_img_t*>(model_->getInput());
    if (model_input) {
        input_width_ = model_input->width;
        input_height_ = model_input->height;
    }

    MA_LOGI(TAG, "Detector initialized (type: %s, input: %dx%d)",
            m->getName(), input_width_, input_height_);
    initialized_ = true;
    return true;
}

void Detector::setThreshold(float threshold) {
    threshold_ = std::max(0.0f, std::min(1.0f, threshold));
    if (model_) {
        model_->setConfig(MA_MODEL_CFG_OPT_THRESHOLD, threshold_);
    }
    if (seg_model_) {
        seg_model_->setConfig(MA_MODEL_CFG_OPT_THRESHOLD, static_cast<double>(threshold_));
    }
    if (seg_heads_model_) {
        seg_heads_model_->setConfig(MA_MODEL_CFG_OPT_THRESHOLD, static_cast<double>(threshold_));
    }
    if (obb_model_) {
        obb_model_->setConfig(MA_MODEL_CFG_OPT_THRESHOLD, static_cast<double>(threshold_));
    }
}

std::vector<Detection> Detector::detect(ma_img_t* img, const rv::MaskFocus* focus,
                                         float content_x, float content_y,
                                         float content_w, float content_h) {
    std::vector<Detection> results;

    if (!initialized_ || img == nullptr) {
        return results;
    }

    if (kind_ == Kind::Segment && seg_heads_model_) {
        seg_heads_model_->setFrame(focus, content_x, content_y, content_w, content_h);
        ma_err_t ret = seg_heads_model_->run(img);
        if (ret != MA_OK) {
            MA_LOGE(TAG, "Segmentation run failed: %d", ret);
            return results;
        }
        results = seg_heads_model_->getDetections();
        for (auto& det : results) {
            det.id = detection_id_counter_++;
        }
        return results;
    }

    if (kind_ == Kind::Segment) {
        if (!seg_model_) {
            return results;
        }
        seg_model_->setMaskFocus(focus, content_x, content_y, content_w, content_h);
        ma_err_t ret = seg_model_->run(img);
        if (ret != MA_OK) {
            MA_LOGE(TAG, "Segmentation run failed: %d", ret);
            return results;
        }
        results = seg_model_->getDetections();
        for (auto& det : results) {
            det.id = detection_id_counter_++;
        }
        return results;
    }

    if (kind_ == Kind::Obb) {
        if (!obb_model_) {
            return results;
        }
        ma_err_t ret = obb_model_->run(img);
        if (ret != MA_OK) {
            MA_LOGE(TAG, "OBB run failed: %d", ret);
            return results;
        }
        results = obb_model_->getDetections();
        for (auto& det : results) {
            det.id = detection_id_counter_++;
        }
        return results;
    }

    if (model_ == nullptr) {
        return results;
    }

    ma_err_t ret = model_->run(img);
    if (ret != MA_OK) {
        MA_LOGE(TAG, "Detection failed: %d", ret);
        return results;
    }

    auto bboxes = model_->getResults();
    for (const auto& bbox : bboxes) {
        if (bbox.score >= threshold_) {
            Detection det;
            det.x = bbox.x;
            det.y = bbox.y;
            det.w = bbox.w;
            det.h = bbox.h;
            det.confidence = bbox.score;
            det.class_id = bbox.target;
            det.id = detection_id_counter_++;
            results.push_back(det);
        }
    }

    return results;
}

float Detector::mask_ms() const {
    if (kind_ == Kind::Segment && seg_model_) {
        return seg_model_->last_mask_ms();
    }
    if (kind_ == Kind::Segment && seg_heads_model_) {
        return seg_heads_model_->last_mask_ms();
    }
    return -1.0f;
}

void Detector::stage_ms(float& pre, float& infer, float& post) const {
    pre = infer = post = -1.0f;
    const ma::Model* m = seg_heads_model_ ? static_cast<const ma::Model*>(seg_heads_model_.get())
                         : seg_model_     ? static_cast<const ma::Model*>(seg_model_.get())
                         : obb_model_     ? static_cast<const ma::Model*>(obb_model_.get())
                                          : static_cast<const ma::Model*>(model_);
    if (m == nullptr) return;
    const ma_perf_t p = m->getPerf();
    pre = static_cast<float>(p.preprocess);
    infer = static_cast<float>(p.inference);
    post = static_cast<float>(p.postprocess);
    if (seg_heads_model_) {
        pre = seg_heads_model_->last_pre_ms();
        post = seg_heads_model_->last_post_ms();
    }
}

void Detector::setMaskEnabled(bool enabled) {
    if (seg_model_) {
        seg_model_->setMaskEnabled(enabled);
    }
    if (seg_heads_model_) {
        seg_heads_model_->setMaskEnabled(enabled);
    }
}

void Detector::setMaskMaxBoxes(int max_boxes) {
    if (seg_model_) {
        seg_model_->setMaskMaxBoxes(max_boxes);
    }
    if (seg_heads_model_) {
        seg_heads_model_->setMaskMaxBoxes(max_boxes);
    }
}

void Detector::setMaskFocus(const rv::MaskFocus* focus, float content_x, float content_y,
                            float content_w, float content_h) {
    if (seg_model_) {
        seg_model_->setMaskFocus(focus, content_x, content_y, content_w, content_h);
    }
    if (seg_heads_model_) {
        seg_heads_model_->setFrame(focus, content_x, content_y, content_w, content_h);
    }
}

void Detector::setScale(float sx, float sy) {
    if (seg_model_) {
        seg_model_->setScale(sx, sy);
    }
}

void Detector::setClassNames(rv::ClassNames names) {
    class_names_ = std::move(names);
    if (seg_model_) {
        seg_model_->setClassNames(&class_names_);
    }
    if (seg_heads_model_) {
        seg_heads_model_->setClassNames(&class_names_);
    }
}

}  // namespace yolo
