#ifndef _SEG_MODEL_H_
#define _SEG_MODEL_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <sscma.h>
#include "detector.h"

namespace yolo {

using Engine = ma::engine::Engine;

class SegModel : public ma::model::Segmentor {
public:
    explicit SegModel(Engine* engine);
    ~SegModel() override = default;

    const std::vector<Detection>& getDetections() const { return detections_; }
    float last_mask_ms() const { return last_mask_ms_; }
    void setMaskEnabled(bool enabled) { mask_enabled_ = enabled; }
    void setMaskMaxBoxes(int max_boxes) { mask_max_boxes_ = max_boxes; }
    void setMaskFocus(const rv::MaskFocus* focus, float cx, float cy, float cw, float ch) {
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
    void setScale(float sx, float sy) { scale_x_ = sx; scale_y_ = sy; }
    int numClasses() const { return num_class_; }
    /** Names for the mask-focus label match (owned by the Detector; null: "class_<id>"). */
    void setClassNames(const rv::ClassNames* names) { class_names_ = names; }

protected:
    ma_err_t postprocess() override;

private:
    ma_tensor_t bboxes_{};
    ma_tensor_t protos_{};
    int32_t num_record_{0};
    int32_t num_class_{0};
    std::vector<float> cell_max_f32_;

    std::vector<Detection> detections_;
    bool mask_enabled_{false};
    int mask_max_boxes_{3};
    rv::MaskFocus mask_focus_;
    float content_x_{0.0f};
    float content_y_{0.0f};
    float content_w_{640.0f};
    float content_h_{640.0f};
    float scale_x_{1.0f};
    float scale_y_{1.0f};
    float last_mask_ms_{-1.0f};
    const rv::ClassNames* class_names_{nullptr};

    std::vector<uint8_t> mask_buf_;
    std::vector<float> acc_buf_;
};

}  // namespace yolo

#endif  // _SEG_MODEL_H_
