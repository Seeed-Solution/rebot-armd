// rebot-vision: on-demand depth map (GET /api/depth). Portable (no SDK): the millimetre
// encoding of the ZipDepth output, the binary reply body and the one-at-a-time / 1 per s gate.
#ifndef REBOT_VISION_DEPTH_MAP_H_
#define REBOT_VISION_DEPTH_MAP_H_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace rv {

/** Body magic: "RDM1", then a u32 LE JSON length, the JSON (space-padded) and the u16 LE map. */
constexpr char kDepthMapMagic[4] = {'R', 'D', 'M', '1'};
/** The u16 map starts at a multiple of this offset in the body (Uint16Array / DataView friendly). */
constexpr size_t kDepthMapAlign = 8;
/** Encoded depths outside this range are written as 0 (no depth). */
constexpr float kDepthMapMinM = 0.02f;
constexpr float kDepthMapMaxM = 65.0f;

/**
 * Metric scale of the ZipDepth output: depth_m = scale / disp. The model's disparity is the
 * inverse depth for a focal of 600 px at 640 px width (armd grasp.rs a_prior), so with the
 * camera's fx at `width` px: scale = (fx / width) * 640 / 600.
 */
double depth_scale_m(double fx, double width) noexcept;

/** Robust stats of an encoded map (valid pixels only); all 0 when none is valid. */
struct DepthMapStats {
    uint32_t valid = 0;
    uint16_t min_mm = 0, p2_mm = 0, p50_mm = 0, p98_mm = 0, max_mm = 0;
};

/**
 * depth_m = scale / disp per cell, rounded to mm. A non-finite or non-positive disparity, or a
 * depth outside [kDepthMapMinM, kDepthMapMaxM], gives 0. `out` gets w * h values, row-major.
 */
DepthMapStats encode_depth_mm(const float* disp, int w, int h, double scale, std::vector<uint16_t>& out);

/** Where the grid lies on the visible image, and what the values mean. */
struct DepthMapMeta {
    int width = 384;  ///< grid
    int height = 224;
    int image_w = 640;  ///< visible image the grid was resized from (detections refer to it)
    int image_h = 360;
    uint32_t frame_id = 0;
    uint64_t ts_ms = 0;  ///< frame capture time, Unix ms
    float inference_ms = 0;
    std::string model;
    double scale_m = 1.0;  ///< depth_scale_m used
    double fx = 600.0;     ///< the fx (at image_w) the scale came from
};

/**
 * Reply body: magic, u32 LE n, n bytes of JSON (`meta` + `stats`, padded with spaces so the map
 * starts at a multiple of kDepthMapAlign), then w * h u16 LE millimetres (0 = no depth).
 * The JSON states the mapping: grid cell (i, j) covers image x in [i, i+1) * image_w / width,
 * y in [j, j+1) * image_h / height (the whole image, stretched; no crop or letterbox).
 */
std::string depth_map_body(const DepthMapMeta& meta, const DepthMapStats& stats, const std::vector<uint16_t>& mm);

/** Parse a body back (tests and tools). False when it is malformed. */
bool parse_depth_map_body(const std::string& body, std::string& json, std::vector<uint16_t>& mm);

/**
 * At most one map request at a time, and at most one accepted per `min_interval_ms`.
 * begin() returns Ok (call end() afterwards), Busy (one is running) or RateLimited with
 * retry_after_ms until the next one is accepted.
 */
class DepthMapGate {
public:
    enum class Admit { Ok, Busy, RateLimited };
    explicit DepthMapGate(uint64_t min_interval_ms = 1000) : min_interval_ms_(min_interval_ms) {}
    Admit begin(uint64_t now_ms, uint64_t& retry_after_ms);
    void end();

private:
    std::mutex mu_;
    const uint64_t min_interval_ms_;
    bool running_ = false;
    bool started_ = false;
    uint64_t last_ms_ = 0;
};

}  // namespace rv

#endif  // REBOT_VISION_DEPTH_MAP_H_
