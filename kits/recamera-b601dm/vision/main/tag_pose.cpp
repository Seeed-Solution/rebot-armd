// Port of orthogonal_iteration() and estimate_tag_pose() from apriltag apriltag_pose.c
// (sscma-example-sg200x components/apriltag, BSD-2-Clause, Copyright (C) 2013-2016 The
// Regents of The University of Michigan; licence text in THIRD_PARTY_NOTICES.md), with
// fixed-size arithmetic instead of matd_t temporaries.
#include "tag_pose.h"

#include <cmath>

extern "C" {
// Not declared in apriltag_pose.h, defined in apriltag_pose.c.
matd_t* fix_pose_ambiguities(matd_t** v, matd_t** p, matd_t* t, matd_t* R, int n_points);
}

namespace rv {
namespace {

using M3 = double[9];  // row-major

void mat_vec(const double* m, const double* x, double* y) {
    for (int i = 0; i < 3; ++i) y[i] = m[3 * i] * x[0] + m[3 * i + 1] * x[1] + m[3 * i + 2] * x[2];
}

double dot(const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

void cross(const double* a, const double* b, double* c) {
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

bool inverse(const double* m, double* o) {
    const double d = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
                     m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (d == 0) return false;
    o[0] = (m[4] * m[8] - m[5] * m[7]) / d;
    o[1] = (m[2] * m[7] - m[1] * m[8]) / d;
    o[2] = (m[1] * m[5] - m[2] * m[4]) / d;
    o[3] = (m[5] * m[6] - m[3] * m[8]) / d;
    o[4] = (m[0] * m[8] - m[2] * m[6]) / d;
    o[5] = (m[2] * m[3] - m[0] * m[5]) / d;
    o[6] = (m[3] * m[7] - m[4] * m[6]) / d;
    o[7] = (m[1] * m[6] - m[0] * m[7]) / d;
    o[8] = (m[0] * m[4] - m[1] * m[3]) / d;
    return true;
}

/** Eigen-decomposition of a symmetric 3x3 (cyclic Jacobi). Columns of V, values descending. */
void sym_eigen(const double* a_in, double w[3], double V[9]) {
    double a[9];
    for (int i = 0; i < 9; ++i) a[i] = a_in[i];
    for (int i = 0; i < 9; ++i) V[i] = (i % 4 == 0) ? 1 : 0;
    for (int sweep = 0; sweep < 32; ++sweep) {
        const double off = a[1] * a[1] + a[2] * a[2] + a[5] * a[5];
        if (off < 1e-300) break;
        for (int pq = 0; pq < 3; ++pq) {
            const int p = pq == 2 ? 1 : 0, q = pq == 0 ? 1 : 2;
            const double apq = a[3 * p + q];
            if (std::fabs(apq) < 1e-300) continue;
            const double theta = (a[3 * q + q] - a[3 * p + p]) / (2 * apq);
            const double tn = (theta >= 0 ? 1 : -1) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
            const double c = 1 / std::sqrt(tn * tn + 1), s = tn * c;
            for (int k = 0; k < 3; ++k) {  // A <- A J
                const double akp = a[3 * k + p], akq = a[3 * k + q];
                a[3 * k + p] = c * akp - s * akq;
                a[3 * k + q] = s * akp + c * akq;
            }
            for (int k = 0; k < 3; ++k) {  // A <- J' A
                const double apk = a[3 * p + k], aqk = a[3 * q + k];
                a[3 * p + k] = c * apk - s * aqk;
                a[3 * q + k] = s * apk + c * aqk;
            }
            for (int k = 0; k < 3; ++k) {  // V <- V J
                const double vkp = V[3 * k + p], vkq = V[3 * k + q];
                V[3 * k + p] = c * vkp - s * vkq;
                V[3 * k + q] = s * vkp + c * vkq;
            }
        }
    }
    for (int i = 0; i < 3; ++i) w[i] = a[4 * i];
    for (int i = 0; i < 2; ++i) {  // sort descending
        for (int j = i + 1; j < 3; ++j) {
            if (w[j] > w[i]) {
                const double tw = w[i];
                w[i] = w[j];
                w[j] = tw;
                for (int k = 0; k < 3; ++k) {
                    const double tv = V[3 * k + i];
                    V[3 * k + i] = V[3 * k + j];
                    V[3 * k + j] = tv;
                }
            }
        }
    }
}

/**
 * Rotation nearest to m (det +1): U diag(1, 1, det(UV')) V'. apriltag takes U V' and
 * negates its last column when det < 0; for coplanar object points (m's last column is 0,
 * so v3 = +-z) both give the same matrix.
 */
void nearest_rotation(const double* m, double* R) {
    double mtm[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) mtm[3 * i + j] = m[i] * m[j] + m[3 + i] * m[3 + j] + m[6 + i] * m[6 + j];
    double w[3], V[9];
    sym_eigen(mtm, w, V);
    double v[3][3], u[3][3];
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) v[i][k] = V[3 * k + i];
    for (int i = 0; i < 2; ++i) {
        mat_vec(m, v[i], u[i]);
        const double n = std::sqrt(dot(u[i], u[i]));
        for (int k = 0; k < 3; ++k) u[i][k] = n > 0 ? u[i][k] / n : 0;
    }
    // Re-orthogonalise u1 against u0 (sigma1 can be small), then complete both bases
    // right-handed: R = u0 v0' + u1 v1' + (u0 x u1)(v0 x v1)'.
    const double d01 = dot(u[0], u[1]);
    for (int k = 0; k < 3; ++k) u[1][k] -= d01 * u[0][k];
    const double n1 = std::sqrt(dot(u[1], u[1]));
    for (int k = 0; k < 3; ++k) u[1][k] = n1 > 0 ? u[1][k] / n1 : 0;
    cross(u[0], u[1], u[2]);
    cross(v[0], v[1], v[2]);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R[3 * i + j] = u[0][i] * v[0][j] + u[1][i] * v[1][j] + u[2][i] * v[2][j];
}

}  // namespace

double tag_pose_orthogonal_iteration(const double v[4][3], const double p[4][3], double R[9], double t[3],
                                     int n_steps) {
    constexpr int n = 4;
    double p_mean[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) p_mean[k] += p[i][k] / n;
    double p_res[n][3];
    for (int i = 0; i < n; ++i)
        for (int k = 0; k < 3; ++k) p_res[i][k] = p[i][k] - p_mean[k];

    // F_i = v v' / v'v (projection onto the line of sight); M1_inv = (I - mean F)^-1.
    double F[n][9];
    M3 avg_F = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < n; ++i) {
        const double vv = dot(v[i], v[i]);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                F[i][3 * r + c] = v[i][r] * v[i][c] / vv;
                avg_F[3 * r + c] += F[i][3 * r + c] / n;
            }
    }
    M3 M1, M1_inv;
    for (int k = 0; k < 9; ++k) M1[k] = (k % 4 == 0 ? 1.0 : 0.0) - avg_F[k];
    if (!inverse(M1, M1_inv)) return HUGE_VAL;

    double error = HUGE_VAL;
    for (int step = 0; step < n_steps; ++step) {
        // Translation: t = M1_inv * mean((F_j - I) R p_j).
        double M2[3] = {0, 0, 0};
        double Rp[n][3];
        for (int j = 0; j < n; ++j) {
            mat_vec(R, p[j], Rp[j]);
            double FRp[3];
            mat_vec(F[j], Rp[j], FRp);
            for (int k = 0; k < 3; ++k) M2[k] += (FRp[k] - Rp[j][k]) / n;
        }
        mat_vec(M1_inv, M2, t);

        // Rotation: nearest rotation to sum (q_j - mean q) p_res_j', q_j = F_j (R p_j + t).
        double q[n][3], q_mean[3] = {0, 0, 0};
        for (int j = 0; j < n; ++j) {
            const double x[3] = {Rp[j][0] + t[0], Rp[j][1] + t[1], Rp[j][2] + t[2]};
            mat_vec(F[j], x, q[j]);
            for (int k = 0; k < 3; ++k) q_mean[k] += q[j][k] / n;
        }
        M3 M3m = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (int j = 0; j < n; ++j)
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) M3m[3 * r + c] += (q[j][r] - q_mean[r]) * p_res[j][c];
        nearest_rotation(M3m, R);

        // Object-space error: sum |(I - F_j)(R p_j + t)|^2.
        error = 0;
        for (int j = 0; j < n; ++j) {
            double x[3], Fx[3];
            mat_vec(R, p[j], x);
            for (int k = 0; k < 3; ++k) x[k] += t[k];
            mat_vec(F[j], x, Fx);
            for (int k = 0; k < 3; ++k) error += (x[k] - Fx[k]) * (x[k] - Fx[k]);
        }
    }
    return error;
}

double tag_pose_estimate(apriltag_detection_info_t* info, apriltag_pose_t* pose, int n_steps) {
    const double s = info->tagsize / 2.0;
    const double p[4][3] = {{-s, s, 0}, {s, s, 0}, {s, -s, 0}, {-s, -s, 0}};
    double v[4][3];
    for (int i = 0; i < 4; ++i) {
        v[i][0] = (info->det->p[i][0] - info->cx) / info->fx;
        v[i][1] = (info->det->p[i][1] - info->cy) / info->fy;
        v[i][2] = 1;
    }

    // Solution 1 from the homography, refined.
    apriltag_pose_t s1;
    estimate_pose_for_tag_homography(info, &s1);
    const double err1 = tag_pose_orthogonal_iteration(v, p, s1.R->data, s1.t->data, n_steps);

    // Solution 2: the other local minimum, if any (apriltag, once per marker).
    matd_t* vm[4];
    matd_t* pm[4];
    for (int i = 0; i < 4; ++i) {
        vm[i] = matd_create_data(3, 1, v[i]);
        pm[i] = matd_create_data(3, 1, p[i]);
    }
    apriltag_pose_t s2;
    s2.R = fix_pose_ambiguities(vm, pm, s1.t, s1.R, 4);
    s2.t = nullptr;
    for (int i = 0; i < 4; ++i) {
        matd_destroy(vm[i]);
        matd_destroy(pm[i]);
    }
    double err2 = HUGE_VAL;
    if (s2.R) {
        s2.t = matd_create(3, 1);
        err2 = tag_pose_orthogonal_iteration(v, p, s2.R->data, s2.t->data, n_steps);
    }

    if (err1 <= err2) {
        *pose = s1;
        if (s2.R) {
            matd_destroy(s2.R);
            matd_destroy(s2.t);
        }
        return err1;
    }
    *pose = s2;
    matd_destroy(s1.R);
    matd_destroy(s1.t);
    return err2;
}

}  // namespace rv
