#include "depth_stage.h"

#include <chrono>
#include <cstring>

#define TAG "DepthStage"

namespace rv {

DepthStage::DepthStage() = default;
// The engine holds the TPU buffers behind input_ / output_ (no wrapper objects); a swap
// destroys the whole object.
DepthStage::~DepthStage() { engine_.reset(); }

bool DepthStage::init(const std::string& model_path) {
    MA_LOGI(TAG, "Loading depth model: %s", model_path.c_str());

    engine_ = std::make_unique<ma::engine::EngineCVI>();
    if (engine_->init() != MA_OK) {
        MA_LOGE(TAG, "EngineCVI init failed");
        return false;
    }
    if (engine_->load(model_path.c_str()) != MA_OK) {
        MA_LOGE(TAG, "Failed to load depth model: %s", model_path.c_str());
        return false;
    }

    const int32_t n_in = engine_->getInputSize();
    const int32_t n_out = engine_->getOutputSize();
    if (n_in != 1 || n_out != 1) {
        MA_LOGE(TAG, "Expected 1 in / 1 out, got %d in / %d out", n_in, n_out);
        return false;
    }

    input_ = engine_->getInput(0);
    output_ = engine_->getOutput(0);

    const ma_shape_t in_shape = input_.shape;
    const ma_shape_t out_shape = output_.shape;

    if (in_shape.size != 4 || in_shape.dims[0] != 1) {
        MA_LOGE(TAG, "Unsupported input shape dim_size=%u, batch=%d", in_shape.size, in_shape.dims[0]);
        return false;
    }

    if (in_shape.dims[3] == 3) {
        is_nhwc_ = true;
        grid_h_ = in_shape.dims[1];
        grid_w_ = in_shape.dims[2];
    } else if (in_shape.dims[1] == 3) {
        is_nhwc_ = false;
        grid_h_ = in_shape.dims[2];
        grid_w_ = in_shape.dims[3];
    } else {
        MA_LOGE(TAG, "Unsupported input shape (dims neither [1,H,W,3] nor [1,3,H,W])");
        return false;
    }

    if (grid_w_ != 384 || grid_h_ != 224) {
        MA_LOGE(TAG, "Expected 384x224 grid, got %dx%d", grid_w_, grid_h_);
        return false;
    }

    int od[2] = {0, 0};
    bool out_ok = false;
    if (out_shape.size == 4 && out_shape.dims[0] == 1 && out_shape.dims[1] == 1) {
        od[0] = out_shape.dims[2];
        od[1] = out_shape.dims[3];
        out_ok = true;
    } else if (out_shape.size == 3 && out_shape.dims[0] == 1) {
        od[0] = out_shape.dims[1];
        od[1] = out_shape.dims[2];
        out_ok = true;
    } else if (out_shape.size == 2) {
        od[0] = out_shape.dims[0];
        od[1] = out_shape.dims[1];
        out_ok = true;
    }

    if (!out_ok || od[0] != 224 || od[1] != 384) {
        MA_LOGE(TAG, "Unsupported output shape (expected 224x384 depth map)");
        return false;
    }

    if (is_nhwc_ && input_.type != MA_TENSOR_TYPE_U8) {
        MA_LOGE(TAG, "NHWC input only supported for UINT8 tensor type (got %d)", input_.type);
        return false;
    }
    if (!is_nhwc_ && input_.type != MA_TENSOR_TYPE_F32) {
        MA_LOGE(TAG, "NCHW input only supported for FP32 tensor type (got %d)", input_.type);
        return false;
    }
    if (output_.type != MA_TENSOR_TYPE_F32 && output_.type != MA_TENSOR_TYPE_BF16) {
        MA_LOGE(TAG, "Unsupported output dtype %d (f32 or bf16 expected)", output_.type);
        return false;
    }
    if (input_.data.data == nullptr || output_.data.data == nullptr) {
        MA_LOGE(TAG, "Model tensors carry no buffer");
        return false;
    }

    input_row_bytes_ = (is_nhwc_ && input_.type == MA_TENSOR_TYPE_U8 &&
                        (input_.size % static_cast<size_t>(grid_h_) == 0))
                           ? (input_.size / static_cast<size_t>(grid_h_))
                           : static_cast<size_t>(grid_w_) * 3;

    depth_.assign(static_cast<size_t>(grid_w_ * grid_h_), 0.0f);
    initialized_ = true;
    MA_LOGI(TAG, "DepthStage ready (in %s %dx%d, row_bytes=%zu, out %dx%d)",
            is_nhwc_ ? "NHWC" : "NCHW", grid_w_, grid_h_, input_row_bytes_, od[1], od[0]);
    return true;
}

bool DepthStage::run(const uint8_t* src_rgb, int src_w, int src_h, int src_stride) {
    if (!initialized_ || src_rgb == nullptr || src_w <= 0 || src_h <= 0) return false;

    using clk = std::chrono::steady_clock;
    const auto t0 = clk::now();

    if (is_nhwc_ && input_.type == MA_TENSOR_TYPE_U8) {
        uint8_t* dst = input_.data.u8;
        resize_rgb888_bilinear(src_rgb, src_w, src_h, src_stride, dst, grid_w_, grid_h_,
                               static_cast<int>(input_row_bytes_));
    } else {
        std::vector<uint8_t> tmp(static_cast<size_t>(grid_w_ * grid_h_ * 3));
        resize_rgb888_bilinear(src_rgb, src_w, src_h, src_stride, tmp.data(), grid_w_, grid_h_, grid_w_ * 3);
        const size_t plane = static_cast<size_t>(grid_w_ * grid_h_);
        float* dst_r = input_.data.f32;
        float* dst_g = dst_r + plane;
        float* dst_b = dst_g + plane;
        constexpr float kScale = 1.0f / 255.0f;
        for (size_t i = 0; i < plane; ++i) {
            dst_r[i] = static_cast<float>(tmp[i * 3 + 0]) * kScale;
            dst_g[i] = static_cast<float>(tmp[i * 3 + 1]) * kScale;
            dst_b[i] = static_cast<float>(tmp[i * 3 + 2]) * kScale;
        }
    }

    if (engine_->run() != MA_OK) {
        MA_LOGE(TAG, "Depth TPU forward pass failed");
        return false;
    }

    output_ = engine_->getOutput(0);
    if (output_.data.data == nullptr) return false;

    const size_t n = depth_.size();
    if (output_.type == MA_TENSOR_TYPE_F32) {
        std::memcpy(depth_.data(), output_.data.f32, n * sizeof(float));
    } else if (output_.type == MA_TENSOR_TYPE_BF16) {
        const uint16_t* bf = static_cast<const uint16_t*>(output_.data.data);
        for (size_t i = 0; i < n; ++i) {
            const uint32_t u32 = static_cast<uint32_t>(bf[i]) << 16;
            float val;
            std::memcpy(&val, &u32, sizeof(float));
            depth_[i] = val;
        }
    }

    const auto t1 = clk::now();
    last_inference_ms_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
    return true;
}

}  // namespace rv
