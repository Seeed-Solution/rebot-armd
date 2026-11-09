// Marker pose without per-step heap allocation.
//
// estimate_tag_pose() in apriltag (apriltag_pose.c) runs orthogonal iteration (Lu et al.
// 2000) for 50 steps, twice per marker, with matd_t temporaries: about 26 000 malloc/free
// calls per marker. tag_pose_estimate() runs the same algorithm, steps and solution choice
// with fixed-size 3x3 arithmetic; only the one-off initial guess (homography) and the
// second-minimum search (fix_pose_ambiguities) still go through apriltag.
#ifndef REBOT_VISION_TAG_POSE_H_
#define REBOT_VISION_TAG_POSE_H_

#include "apriltag_pose.h"

namespace rv {

/**
 * Orthogonal iteration on 4 tag corners. v: normalised image rays (x, y, 1); p: object
 * points with z = 0. R (row-major 3x3) is the initial guess and the result; t is the result.
 * Returns the object-space error of the last step. Same arithmetic as apriltag's
 * orthogonal_iteration(), with R taken as the rotation nearest to M3 (det +1).
 */
double tag_pose_orthogonal_iteration(const double v[4][3], const double p[4][3], double R[9], double t[3],
                                     int n_steps);

/** Drop-in for estimate_tag_pose(): returns the error; pose.R / pose.t are matd_t the caller frees. */
double tag_pose_estimate(apriltag_detection_info_t* info, apriltag_pose_t* pose, int n_steps = 50);

}  // namespace rv

#endif  // REBOT_VISION_TAG_POSE_H_
