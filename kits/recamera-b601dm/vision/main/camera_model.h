#ifndef _ARUCO_CAMERA_MODEL_H_
#define _ARUCO_CAMERA_MODEL_H_

namespace aruco {

/**
 * Pinhole intrinsics + OpenCV Brown-Conrady distortion (k1, k2, p1, p2, k3),
 * expressed at the inference channel resolution (--camera-width/--camera-height).
 */
struct CameraModel {
    double fx = 600.0, fy = 600.0, cx = 320.0, cy = 180.0;
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0, k3 = 0.0;

    bool has_distortion() const noexcept {
        return k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0;
    }

    /** Ideal (undistorted) pixel -> distorted pixel, OpenCV projectPoints model. */
    void distort(double ux, double uy, double& dx, double& dy) const noexcept;

    /**
     * Distorted pixel -> ideal pixel. Iterative fixed-point inversion as in
     * cv::undistortPoints (P = K). Stops early once the update is below 1e-9
     * normalized units; max_iters bounds the cost.
     */
    void undistort(double dx, double dy, double& ux, double& uy, int max_iters = 10) const noexcept;
};

/**
 * Homography mapping apriltag tag coordinates to image pixels, with the same
 * corner convention apriltag uses for det->p: p[0]=(-1,+1), p[1]=(+1,+1),
 * p[2]=(+1,-1), p[3]=(-1,-1). Output H is row-major 3x3 with H[8]=1.
 * Returns false if the corners are degenerate.
 */
bool homography_from_corners(const double p[4][2], double H[9]) noexcept;

}  // namespace aruco

#endif
