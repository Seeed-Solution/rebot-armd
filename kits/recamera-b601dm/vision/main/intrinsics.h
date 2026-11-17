// Camera intrinsics of the ArUco stage as a runtime setting: validation of
// PUT /api/calibration/intrinsics, the merge into vision.json and its atomic write.
// Portable (no SDK); host tests in tests/intrinsics_test.cpp.
#ifndef REBOT_VISION_INTRINSICS_H_
#define REBOT_VISION_INTRINSICS_H_

#include <cstdint>
#include <string>

#include "camera_model.h"

namespace rv {

/** The only image size intrinsics are accepted at: the content image ArUco corners refer to. */
constexpr int kIntrinsicsWidth = 640;
constexpr int kIntrinsicsHeight = 360;

/** Intrinsics at kIntrinsicsWidth x kIntrinsicsHeight and the marker side (m). */
struct Intrinsics {
    aruco::CameraModel cam;
    double marker_size = 0.05;
};

/**
 * A PUT /api/calibration/intrinsics body, parsed and validated.
 * Required: fx, fy, cx, cy, k1, k2, p1, p2, k3, width (640), height (360), expected_revision.
 * Optional: marker_size (absent: `current.marker_size`). Ignored (read-only fields of the GET
 * body): revision, overridden_by_cli, config_path, api_version. Any other key is an error.
 * Rules: finite numbers; fx, fy in [100, 5000]; 0 < cx < width; 0 < cy < height; |k1|, |k2|,
 * |k3| <= 10; |p1|, |p2| <= 1; marker_size in (0, 1]; expected_revision an unsigned integer.
 * On error: false, `field` = the offending key ("" for a body that is not a JSON object),
 * `message` = a sentence naming it.
 */
bool parse_intrinsics_put(const std::string& body, const Intrinsics& current, Intrinsics& out,
                          uint64_t& expected_revision, std::string& field, std::string& message);

/**
 * vision.json text with fx..k3 and marker_size set from `in`; every other key keeps its value
 * and its position (key order of `existing`). `existing` empty (no file) = an empty object.
 * False (out untouched, `error` set) when `existing` is not a JSON object: the file is not
 * overwritten then.
 */
bool merge_intrinsics_config(const std::string& existing, const Intrinsics& in, std::string& out, std::string& error);

/** Whole file as text; false when it cannot be opened (`missing` true for ENOENT). */
bool read_text_file(const std::string& path, std::string& out, bool& missing);

/**
 * Replace `path` atomically: write `<path>.tmp.<pid>` in the same directory, fsync it, rename
 * it over `path`, fsync the directory. The old file stays on any failure (the temporary file
 * is removed). The new file gets mode 0644.
 */
bool write_file_atomic(const std::string& path, const std::string& text, std::string& error);

/**
 * Read `path`, merge `in` into it (merge_intrinsics_config) and write it back
 * (write_file_atomic). A missing file is created.
 */
bool persist_intrinsics(const std::string& path, const Intrinsics& in, std::string& error);

}  // namespace rv

#endif  // REBOT_VISION_INTRINSICS_H_
