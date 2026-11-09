#include "detector.h"

#include <algorithm>

#define TAG "Detector"

namespace yolo {

const char* COCO_CLASSES[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};

Detector::Detector()
    : engine_(nullptr),
      model_(nullptr),
      threshold_(0.25f),
      input_width_(640),
      input_height_(640),
      initialized_(false),
      detection_id_counter_(0) {}

Detector::~Detector() {
    if (model_) {
        ma::ModelFactory::remove(model_);
        model_ = nullptr;
    }
}

bool Detector::init(const std::string& model_path) {
    MA_LOGI(TAG, "Initializing detector with model: %s", model_path.c_str());

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
}

std::vector<Detection> Detector::detect(ma_img_t* img) {
    std::vector<Detection> results;

    if (!initialized_ || model_ == nullptr || img == nullptr) {
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

const char* Detector::getClassName(int class_id) {
    if (class_id >= 0 && class_id < 80) {
        return COCO_CLASSES[class_id];
    }
    return "unknown";
}

}  // namespace yolo
