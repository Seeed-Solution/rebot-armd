#include "camera_model.h"

#include <cmath>

namespace aruco {

void CameraModel::distort(double ux, double uy, double& dx, double& dy) const noexcept {
    const double x = (ux - cx) / fx;
    const double y = (uy - cy) / fy;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
    const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    dx = xd * fx + cx;
    dy = yd * fy + cy;
}

void CameraModel::undistort(double dx, double dy, double& ux, double& uy, int max_iters) const noexcept {
    const double x0 = (dx - cx) / fx;
    const double y0 = (dy - cy) / fy;
    double x = x0, y = y0;
    for (int i = 0; i < max_iters; ++i) {
        const double r2 = x * x + y * y;
        const double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
        if (radial <= 0.0) break;  // outside the model's valid domain; keep last estimate
        const double delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        const double nx = (x0 - delta_x) / radial;
        const double ny = (y0 - delta_y) / radial;
        const double step = std::fabs(nx - x) + std::fabs(ny - y);
        x = nx;
        y = ny;
        if (step < 1e-9) break;
    }
    ux = x * fx + cx;
    uy = y * fy + cy;
}

bool homography_from_corners(const double p[4][2], double H[9]) noexcept {
    static const double tag[4][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
    // Solve A h = b for the 8 unknowns (h22 = 1), Gaussian elimination with partial pivoting.
    double A[8][9];
    for (int i = 0; i < 4; ++i) {
        const double x = tag[i][0], y = tag[i][1], u = p[i][0], v = p[i][1];
        double* r0 = A[2 * i];
        double* r1 = A[2 * i + 1];
        r0[0] = x; r0[1] = y; r0[2] = 1; r0[3] = 0; r0[4] = 0; r0[5] = 0; r0[6] = -x * u; r0[7] = -y * u; r0[8] = u;
        r1[0] = 0; r1[1] = 0; r1[2] = 0; r1[3] = x; r1[4] = y; r1[5] = 1; r1[6] = -x * v; r1[7] = -y * v; r1[8] = v;
    }
    for (int col = 0; col < 8; ++col) {
        int piv = col;
        for (int r = col + 1; r < 8; ++r)
            if (std::fabs(A[r][col]) > std::fabs(A[piv][col])) piv = r;
        if (std::fabs(A[piv][col]) < 1e-10) return false;
        if (piv != col)
            for (int k = 0; k < 9; ++k) { const double t = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = t; }
        for (int r = col + 1; r < 8; ++r) {
            const double f = A[r][col] / A[col][col];
            for (int k = col; k < 9; ++k) A[r][k] -= f * A[col][k];
        }
    }
    double h[8];
    for (int col = 7; col >= 0; --col) {
        double sum = 0;
        for (int k = col + 1; k < 8; ++k) sum += A[col][k] * h[k];
        h[col] = (A[col][8] - sum) / A[col][col];
    }
    for (int i = 0; i < 8; ++i) H[i] = h[i];
    H[8] = 1.0;
    return true;
}

}  // namespace aruco
