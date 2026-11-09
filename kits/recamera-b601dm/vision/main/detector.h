#ifndef _DETECTOR_H_
#define _DETECTOR_H_

#include <vector>
#include <string>
#include <memory>

#include <sscma.h>

namespace yolo {

struct Detection {
    float x;            // Normalized [0-1]
    float y;
    float w;
    float h;
    float confidence;
    int class_id;
    int id;
};

class Detector {
public:
    Detector();
    ~Detector();

    bool init(const std::string& model_path);
    void setThreshold(float threshold);

    std::vector<Detection> detect(ma_img_t* img);

    int getInputWidth() const { return input_width_; }
    int getInputHeight() const { return input_height_; }
    bool isInitialized() const { return initialized_; }

    static const char* getClassName(int class_id);

private:
    std::unique_ptr<ma::engine::EngineCVI> engine_;
    ma::model::Detector* model_;
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
