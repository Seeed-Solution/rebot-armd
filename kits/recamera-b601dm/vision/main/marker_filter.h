#ifndef _ARUCO_MARKER_FILTER_H_
#define _ARUCO_MARKER_FILTER_H_

#include <string>
#include <vector>

namespace aruco {

/** False-positive rejection thresholds applied before pose estimation. */
struct MarkerFilter {
    float min_margin = 30.0f;    ///< drop if decision_margin < min_margin (0 = off)
    float min_side_px = 12.0f;   ///< drop if shortest quad edge < min_side_px (0 = off)
    std::vector<int> ids;        ///< allowlist; empty = all ids accepted

    bool id_allowed(int id) const noexcept;
};

enum class FilterVerdict { Accept, LowMargin, SmallSide, IdNotAllowed };

/** Shortest of the 4 edges of the quad (raw pixel coordinates). */
double min_quad_side(const double p[4][2]) noexcept;

/** Checks margin, then side, then id allowlist. */
FilterVerdict classify_marker(const MarkerFilter& f, int id, float decision_margin, const double p[4][2]) noexcept;

/**
 * Parse a comma-separated list of non-negative integers ("3,7, 12"). Empty or
 * whitespace-only input yields an empty list (= all ids). Empty tokens, signs,
 * non-digits, or values above 65535 are malformed: returns false and sets *err.
 */
bool parse_id_list(const std::string& text, std::vector<int>& out, std::string* err = nullptr);

/** "all" or "3,7,12" for logging. */
std::string format_id_list(const std::vector<int>& ids);

}  // namespace aruco

#endif
