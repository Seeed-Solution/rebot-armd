// rebot-vision: stride-aware RGB888 -> gray8 conversion, immutable gray frame records,
// fixed-size retention ring, and RGF1 binary wire encoding. Portable (no SDK).
#ifndef REBOT_VISION_GRAY_FRAME_H_
#define REBOT_VISION_GRAY_FRAME_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rv {

constexpr int kGrayWidth = 640;
constexpr int kGrayHeight = 360;
constexpr size_t kGrayRingCapacity = 4;

/** Wire magic: "RGF1", u32 LE JSON length N, N bytes of space-padded UTF-8 JSON, then pixels. */
constexpr char kGrayFrameMagic[4] = {'R', 'G', 'F', '1'};
/** Pixel payload starts at a multiple of 8 bytes from the start of the body (8 + N). */
constexpr size_t kGrayFrameAlign = 8;
/** Maximum allowed RGF1 JSON header length (bytes). */
constexpr size_t kGrayMaxHeaderBytes = 16384;

/**
 * Full-range integer luma on packed RGB888 bytes: (77*R + 150*G + 29*B) >> 8.
 * `stride` is bytes per source row (>= w * 3). Writes tightly packed `w * h` bytes to `out`.
 * Returns false if pointers are null, dimensions are non-positive, or stride < w * 3.
 */
bool rgb888_to_gray(const uint8_t* rgb, int w, int h, int stride, uint8_t* out) noexcept;

/**
 * Validate source geometry and crop bounds, allocate `crop_w * crop_h` bytes into `out`, and
 * convert the cropped RGB888 region to gray8. Leaves `out` unchanged on failure.
 */
bool extract_gray_crop(const uint8_t* frame_rgb, int frame_w, int frame_h, int frame_stride,
                       int crop_x, int crop_y, int crop_w, int crop_h, std::vector<uint8_t>& out);

/** Immutable owned gray frame record stored in GrayRing and pinned during socket transmission. */
struct GrayFrame {
    std::string instance_id;
    uint32_t frame_id = 0;
    int width = 0;
    int height = 0;
    int stride = 0;
    int frame_w = 0;
    int frame_h = 0;
    int crop_x = 0;
    int crop_y = 0;
    int crop_w = 0;
    int crop_h = 0;
    int channel = 0;
    uint64_t ts_ms = 0;            ///< grab-derived Unix ms diagnostic
    uint64_t frame_mono_ms = 0;    ///< frame_plan() monotonic ms
    uint64_t wait_mono_ms = 0;     ///< before_grab monotonic ms
    uint64_t grab_mono_ms = 0;     ///< retrieveFrame() return monotonic ms
    uint64_t stored_mono_ms = 0;   ///< conversion complete monotonic ms
    float convert_ms = 0.0f;       ///< RGB888 -> gray8 conversion duration (ms)
    uint64_t model_generation = 0;
    std::shared_ptr<const std::vector<uint8_t>> pixels;

    bool valid() const noexcept;
};

/** Metadata copied into a newly built GrayFrame by make_gray_frame(). */
struct GrayFrameMeta {
    std::string instance_id;
    uint32_t frame_id = 0;
    int channel = 0;
    uint64_t ts_ms = 0;
    uint64_t frame_mono_ms = 0;
    uint64_t wait_mono_ms = 0;
    uint64_t grab_mono_ms = 0;
    uint64_t stored_mono_ms = 0;
    float convert_ms = 0.0f;
    uint64_t model_generation = 0;
};

/**
 * Validate source geometry, convert the RGB888 crop into owned gray8 pixels, and allocate the
 * complete GrayFrame record. Any allocation failure (before or after conversion) is caught and
 * returns nullptr without throwing or leaving partial state.
 */
std::shared_ptr<GrayFrame> make_gray_frame(
    const uint8_t* frame_rgb, int frame_w, int frame_h, int frame_stride,
    int crop_x, int crop_y, int crop_w, int crop_h, const GrayFrameMeta& meta,
    const std::function<std::shared_ptr<GrayFrame>()>& alloc_record = nullptr) noexcept;

/** Generate an opaque process/instance identifier unique per vision startup or rollover. */
std::string generate_instance_id();

/**
 * Build the RGF1 prefix: "RGF1" + u32 LE N + N bytes of UTF-8 JSON padded with trailing ASCII
 * spaces so (8 + N) % kGrayFrameAlign == 0. Suitable for segmented send with `f.pixels`.
 */
std::string gray_frame_header(const GrayFrame& f);

/** Build the complete RGF1 body (header + pixels). Used by tests and fixture generation. */
std::string gray_frame_body(const GrayFrame& f);

/** Parse a complete RGF1 body back into its JSON header string and pixel bytes (tests/tools). */
bool parse_gray_frame_body(const std::string& body, std::string& json_out, std::vector<uint8_t>& pixels_out);

/**
 * Fixed-capacity FIFO retention ring for immutable GrayFrame records (lazy allocation).
 * Not internally locked; caller (VisionControl) guards access with its mutex and destroys any
 * returned displaced shared_ptr outside the lock.
 */
class GrayRing {
public:
    enum class Lookup { Found, NotStored, Expired };

    explicit GrayRing(size_t capacity = kGrayRingCapacity)
        : capacity_(capacity > 0 ? capacity : 1), slots_(capacity_) {}

    /** Insert `frame` without allocating. Returns the displaced oldest entry if full, else nullptr. */
    std::shared_ptr<const GrayFrame> put(std::shared_ptr<const GrayFrame> frame) noexcept;

    /**
     * Resolve `(instance_id, frame_id)` against `current_instance_id`:
     * - mismatched instance -> Expired (410 FRAME_EXPIRED)
     * - present in ring -> Found (200)
     * - at or below highest evicted gray frame_id -> Expired (410 FRAME_EXPIRED)
     * - otherwise (missing / future / before first eviction) -> NotStored (404 FRAME_NOT_STORED)
     */
    Lookup find(const std::string& instance_id, uint32_t frame_id, const std::string& current_instance_id,
                std::shared_ptr<const GrayFrame>& out) const;

    /** Remove all retained records into `displaced` (caller frees outside lock) and reset eviction state. */
    void clear(std::vector<std::shared_ptr<const GrayFrame>>& displaced);

    size_t size() const noexcept { return count_; }
    size_t capacity() const noexcept { return capacity_; }
    bool has_evicted() const noexcept { return has_evicted_; }
    uint32_t highest_evicted_id() const noexcept { return highest_evicted_id_; }

private:
    size_t capacity_;
    std::vector<std::shared_ptr<const GrayFrame>> slots_;
    size_t head_ = 0;
    size_t count_ = 0;
    bool has_evicted_ = false;
    uint32_t highest_evicted_id_ = 0;
};

}  // namespace rv

#endif  // REBOT_VISION_GRAY_FRAME_H_
