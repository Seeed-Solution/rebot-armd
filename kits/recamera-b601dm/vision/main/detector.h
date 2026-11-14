#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <sscma.h>
#include "mask_geom.h"

namespace yolo {

enum Kind { Detect, Segment, Obb };
enum class ModelKindHint { Auto, Detect, Segment, Obb };

struct Detection {
    float x;            // Normalized [0-1]
    float y;
    float w;
    float h;
    float confidence;
    int class_id;
    int id;
    float angle_deg = NAN;
    float angle_conf = NAN;
    float grasp_width_px = NAN;
    bool mask_focus = false;
    float length_px = NAN;
    bool has_obb = false;
    std::array<std::array<float, 2>, 4> obb_corners{};
};

class SegModel;
class ObbModel;

class Detector {
public:
    Detector();
    ~Detector();

    bool init(const std::string& model_path, ModelKindHint hint = ModelKindHint::Auto);
    void setThreshold(float threshold);

    std::vector<Detection> detect(ma_img_t* img, const rv::MaskFocus* focus = nullptr,
                                  float content_x = 0, float content_y = 0,
                                  float content_w = 0, float content_h = 0);

    Kind getKind() const { return kind_; }
    float mask_ms() const;

    void setMaskEnabled(bool enabled);
    void setMaskMaxBoxes(int max_boxes);
    void setMaskFocus(const rv::MaskFocus* focus, float content_x = 0, float content_y = 0,
                      float content_w = 0, float content_h = 0);
    void setScale(float sx, float sy);

    int getInputWidth() const { return input_width_; }
    int getInputHeight() const { return input_height_; }
    bool isInitialized() const { return initialized_; }

    static const char* getClassName(int class_id);

private:
    std::unique_ptr<ma::engine::EngineCVI> engine_;
    ma::model::Detector* model_;
    std::unique_ptr<SegModel> seg_model_;
    std::unique_ptr<ObbModel> obb_model_;
    Kind kind_;
    float threshold_;
    int input_width_;
    int input_height_;
    bool initialized_;
    int detection_id_counter_;
};

// COCO 80 class names
extern const char* COCO_CLASSES[80];

}  // namespace yolo

#endif  // _DETECTOR_H_
