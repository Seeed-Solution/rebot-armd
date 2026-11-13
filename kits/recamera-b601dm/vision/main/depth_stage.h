#ifndef REBOT_VISION_DEPTH_STAGE_H_
#define REBOT_VISION_DEPTH_STAGE_H_

#include <memory>
#include <string>
#include <vector>

#include <sscma.h>

#include "depth_geom.h"

namespace rv {

/**
 * CVI TPU runner for monocular depth estimation (ZipDepth).
 * Feeds a 384x224 input tensor directly from the 640x360 sub frame and produces
 * a dense 384x224 disparity map.
 */
class DepthStage {
public:
    DepthStage();
    ~DepthStage();

    DepthStage(const DepthStage&) = delete;
    DepthStage& operator=(const DepthStage&) = delete;

    /**
     * Initializes the engine and loads the cvimodel. Checks input/output tensor shapes and types.
     */
    bool init(const std::string& model_path);

    /**
     * Preprocesses the 640x360 RGB888 sub frame directly into the TPU input tensor and runs inference.
     */
    bool run(const uint8_t* src_rgb, int src_w, int src_h, int src_stride);

    const float* depth_map() const noexcept { return depth_.data(); }
    int grid_w() const noexcept { return grid_w_; }
    int grid_h() const noexcept { return grid_h_; }
    float last_inference_ms() const noexcept { return last_inference_ms_; }
    bool initialized() const noexcept { return initialized_; }

private:
    std::unique_ptr<ma::engine::EngineCVI> engine_;
    ma_tensor_t input_{};
    ma_tensor_t output_{};
    std::vector<float> depth_;
    int grid_w_ = 384;
    int grid_h_ = 224;
    bool is_nhwc_ = true;
    size_t input_row_bytes_ = 0;
    float last_inference_ms_ = 0.0f;
    bool initialized_ = false;
};

}  // namespace rv

#endif  // REBOT_VISION_DEPTH_STAGE_H_
