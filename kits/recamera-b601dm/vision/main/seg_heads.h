// Decoder for a YOLO11-seg model cut before its detection-head decode (10 outputs): per
// stride 8 / 16 / 32 a box tensor [1, 64, H, W] (DFL 4 x 16 bins), a class tensor [1, nc, H, W]
// (logits, no sigmoid) and a mask-coefficient tensor [1, 32, H, W]; plus the prototypes
// [1, 32, in_h / 4, in_w / 4]. All FP32, NCHW. Portable: no SDK, host-tested
// (tests/seg_heads_test.cpp). The model wrapper is seg_heads_model.*.
#ifndef REBOT_VISION_SEG_HEADS_H_
#define REBOT_VISION_SEG_HEADS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "obb_geom.h"  // ObbTensorSpec

namespace rv {

constexpr int kSegHeadsOutputs = 10;
constexpr int kSegCoef = 32;  ///< mask coefficients per box (= prototype channels)

struct SegHeadsInfo {
    bool valid = false;
    int in_w = 0;
    int in_h = 0;
    int num_classes = 0;
    std::array<size_t, 3> box{};   ///< output index per scale (stride 8, 16, 32)
    std::array<size_t, 3> cls{};
    std::array<size_t, 3> coef{};
    std::array<int, 3> grid_h{};
    std::array<int, 3> grid_w{};
    std::array<int, 3> stride{};
    size_t proto = 0;
    int proto_h = 0;
    int proto_w = 0;
};

/**
 * The 10-output layout from the output shapes (any order): tensors grouped by (H, W); the group
 * with one tensor is the prototypes (32 channels, grid = input / 4), the three groups of three are
 * the scales (input / 8, / 16, / 32), each with one 64-channel box tensor, one 32-channel
 * coefficient tensor and the class tensor (the remaining one; nc must differ from 32 and 64).
 * Invalid for any other count, rank, data type or shape.
 */
SegHeadsInfo resolve_seg_heads(const ObbTensorSpec* outputs, size_t num_outputs, int in_w, int in_h);

/** One box in model-input pixels, with its mask coefficients. */
struct SegCandidate {
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    float score = 0;
    int class_id = 0;
    std::array<float, kSegCoef> coef{};
};

/** Expected value of one DFL distribution (16 bins, softmax), in grid cells. */
float dfl16(const float* bins, size_t bin_stride);

/** Logit of a probability threshold: sigmoid(x) >= t  <=>  x >= seg_logit(t) (t in (0, 1)). */
float seg_logit(float threshold);

/**
 * Candidates with score >= `conf` (score = sigmoid of the best class logit), boxes in input pixels.
 * `prefilter` (the device path): the per-cell best logit is taken plane by plane and compared
 * with seg_logit(conf); the DFL and the coefficients are read only for cells that pass.
 * Without it every cell is decoded in full (sigmoid of every class, DFL of every cell): the
 * reference the prefilter is tested against. `outputs` are the 10 output pointers by index.
 */
std::vector<SegCandidate> decode_seg_heads(const SegHeadsInfo& info, const float* const* outputs, float conf,
                                           bool prefilter = true, std::vector<float>* scratch = nullptr);

/**
 * Class-aware greedy NMS (suppress IoU > `iou`), best score first. At most `max_nms` candidates
 * (top by score) enter and at most `max_det` are returned.
 */
std::vector<SegCandidate> seg_nms(std::vector<SegCandidate> cands, float iou, size_t max_nms = 3000,
                                  size_t max_det = 300);

/**
 * Where the visible content of the camera frame lands in the model input: input pixel =
 * (frame pixel - src) * scale + pad. Same rule as Ultralytics LetterBox (auto=False): the content
 * is scaled to fit and centred, padding split with the extra row/column at the bottom/right.
 */
struct LetterboxGeom {
    float src_x = 0, src_y = 0;  ///< content origin in the frame
    float src_w = 0, src_h = 0;  ///< content size in the frame
    float scale = 1;
    int pad_x = 0, pad_y = 0;    ///< content origin in the input
    int dst_w = 0, dst_h = 0;    ///< content size in the input
};
LetterboxGeom letterbox_geom(float content_x, float content_y, float content_w, float content_h, int in_w,
                             int in_h);

/**
 * Fills an RGB888 input (in_w x in_h, packed) from the content of an RGB888 frame (`stride` bytes per
 * row): the content rows at scale 1 are copied, other scales use nearest neighbour; the padding is
 * `pad_value` (114, as the calibration images).
 */
void letterbox_rgb(const uint8_t* frame, int frame_w, int frame_h, int stride, const LetterboxGeom& g,
                   uint8_t* input, int in_w, int in_h, uint8_t pad_value = 114);

/**
 * A kept box for the result: clipped to the content in the input, mapped to the frame and
 * normalised to it (centre x, y and w, h in [0, 1] of frame_w x frame_h, as the other models),
 * with its crop in prototype cells for the mask decode.
 */
struct SegBoxOut {
    float x = 0, y = 0, w = 0, h = 0;
    int crop_x = 0, crop_y = 0, crop_w = 0, crop_h = 0;
};
SegBoxOut seg_box_to_frame(const SegCandidate& c, const SegHeadsInfo& info, const LetterboxGeom& g, float frame_w,
                           float frame_h);

}  // namespace rv

#endif  // REBOT_VISION_SEG_HEADS_H_
