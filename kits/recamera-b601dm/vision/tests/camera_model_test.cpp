#include <cmath>
#include <cstdio>

#include "camera_model.h"

using aruco::CameraModel;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

// Grid over the full 640x360 inference frame (corners included): ideal -> distort -> undistort.
static double max_roundtrip_err(const CameraModel& m, int iters) {
    double worst = 0.0;
    for (int yi = 0; yi <= 18; ++yi) {
        for (int xi = 0; xi <= 32; ++xi) {
            const double ux = xi * 20.0, uy = yi * 20.0;
            double dx, dy, rx, ry;
            m.distort(ux, uy, dx, dy);
            m.undistort(dx, dy, rx, ry, iters);
            worst = std::fmax(worst, std::hypot(rx - ux, ry - uy));
        }
    }
    return worst;
}

static void test_roundtrip() {
    struct Case { const char* name; double k1, k2, p1, p2, k3; };
    const Case cases[] = {
        {"barrel k1=-0.3", -0.3, 0.0, 0.0, 0.0, 0.0},
        {"barrel k1=-0.3 k2=0.1", -0.3, 0.1, 0.0, 0.0, 0.0},
        {"full k1=-0.3 k2=0.1 p1=1e-3 p2=-5e-4 k3=-0.02", -0.3, 0.1, 1e-3, -5e-4, -0.02},
        {"pincushion k1=0.15", 0.15, 0.0, 0.0, 0.0, 0.0},
    };
    for (const auto& c : cases) {
        CameraModel m;
        m.fx = 600; m.fy = 600; m.cx = 320; m.cy = 180;
        m.k1 = c.k1; m.k2 = c.k2; m.p1 = c.p1; m.p2 = c.p2; m.k3 = c.k3;
        CHECK(m.has_distortion());
        const double e10 = max_roundtrip_err(m, 10);
        std::printf("  roundtrip %-48s max_err(10 it)=%.2e px\n", c.name, e10);
        CHECK(e10 < 0.01);
    }
}

static void test_identity_when_zero() {
    CameraModel m;
    CHECK(!m.has_distortion());
    double ux, uy;
    m.undistort(12.5, 345.25, ux, uy);
    CHECK(std::fabs(ux - 12.5) < 1e-12 && std::fabs(uy - 345.25) < 1e-12);
}

static void project(const double H[9], double x, double y, double& u, double& v) {
    const double w = H[6] * x + H[7] * y + H[8];
    u = (H[0] * x + H[1] * y + H[2]) / w;
    v = (H[3] * x + H[4] * y + H[5]) / w;
}

static void test_homography() {
    // A perspective quad; H must reproduce apriltag's corner convention exactly.
    const double p[4][2] = {{100, 220}, {260, 210}, {250, 90}, {120, 70}};
    const double tag[4][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
    double H[9];
    CHECK(aruco::homography_from_corners(p, H));
    double worst = 0;
    for (int i = 0; i < 4; ++i) {
        double u, v;
        project(H, tag[i][0], tag[i][1], u, v);
        worst = std::fmax(worst, std::hypot(u - p[i][0], v - p[i][1]));
    }
    std::printf("  homography corner reprojection max_err=%.2e px\n", worst);
    CHECK(worst < 1e-6);

    const double degenerate[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    CHECK(!aruco::homography_from_corners(degenerate, H));
}

int main() {
    test_roundtrip();
    test_identity_when_zero();
    test_homography();
    if (g_fail == 0) {
        std::printf("ALL CAMERA MODEL TESTS PASSED\n");
        return 0;
    }
    std::printf("%d TESTS FAILED\n", g_fail);
    return 1;
}
