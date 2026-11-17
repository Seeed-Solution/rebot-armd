#ifndef _SEG_HEADS_MODEL_H_
#define _SEG_HEADS_MODEL_H_

#include <cstdint>
#include <vector>

#include <sscma.h>
#include "detector.h"
#include "seg_heads.h"

namespace yolo {

using Engine = ma::engine::Engine;

/**
 * YOLO11-seg cut before the head decode (10 outputs, seg_heads.h), e.g. the 384x640 model for
 * the 16:9 stream. The model input is filled from the visible content of the frame (letterbox,
 * no stretch); boxes are reported normalised to the frame like the other models, and the masks
 * give the same fields as SegModel (angle_deg, angle_conf, grasp_width_px, mask_focus).
 */
class SegHeadsModel final : public ma::model::Detector {
public:
    explicit SegHeadsModel(Engine* engine);
    ~SegHeadsModel() override = default;

    static bool isValid(Engine* engine);

    const std::vector<Detection>& getDetections() const { return detections_; }
    float last_mask_ms() const { return last_mask_ms_; }
    /** Last frame: input fill (letterbox) and postprocess (decode + NMS + masks), ms. */
    float last_pre_ms() const { return last_pre_ms_; }
    float last_post_ms() const { return last_post_ms_; }
    void setMaskEnabled(bool enabled) { mask_enabled_ = enabled; }
    void setMaskMaxBoxes(int max_boxes) { mask_max_boxes_ = max_boxes; }
    /** Content rect of the frame (pixels; w or h <= 0: the whole frame) and the optional mask focus. */
    void setFrame(const rv::MaskFocus* focus, float cx, float cy, float cw, float ch);
    int numClasses() const { return info_.num_classes; }
    void setClassNames(const rv::ClassNames* names) { class_names_ = names; }

protected:
    ma_err_t preprocess() override;
    ma_err_t postprocess() override;

private:
    ma_err_t decode();

    rv::SegHeadsInfo info_;
    rv::LetterboxGeom geom_;
    float frame_w_ = 0;
    float frame_h_ = 0;
    float content_x_ = 0, content_y_ = 0, content_w_ = 0, content_h_ = 0;
    std::vector<Detection> detections_;
    bool mask_enabled_ = false;
    int mask_max_boxes_ = 3;
    rv::MaskFocus mask_focus_;
    float last_mask_ms_ = -1.0f;
    float last_pre_ms_ = -1.0f;
    float last_post_ms_ = -1.0f;
    const rv::ClassNames* class_names_ = nullptr;
    std::vector<float> scratch_;
    std::vector<uint8_t> mask_buf_;
    std::vector<float> acc_buf_;
};

}  // namespace yolo

#endif  // _SEG_HEADS_MODEL_H_
