// Host benchmark for rv::ArucoStage: per-frame time and detection recall on synthetic
// 640x360 scenes with aruco4x4_50 markers (sizes 14..150 px, in-plane rotation, tilt up
// to 60 degrees, uneven lighting, low contrast, blur, sensor noise, a cluttered
// background). The scene sits in the 640x640 RGB888 letterboxed frame at y = 140 and is
// passed with the frame's stride, as main.cpp passes the zero-copy crop.
//
//   aruco_bench [--scenes N] [--reps N] [--check] [--variant baseline|tuned|both]
//               [--wb N] [--cluster N] [--decimate F] [--sigma F] [--sharpen F] [--nmax N]
//               [--refine 0|1] [--no-filter] [--sweep 1,1.5,2,...]
//
// --sweep: one table row per quad_decimate value (rv defaults otherwise), see run_sweep();
// with --check it fails when the default decimate loses more than 3 points of recall for
// markers of 40 px and larger at normal contrast against the first value of the list, or
// more than 0.15 px of corner accuracy.
// "baseline" = the aruco-detector 0.1.3 detector settings; "tuned" = rv::ArucoConfig
// defaults (with rv::tag_pose_estimate; baseline uses apriltag's estimate_tag_pose).
// --check (ctest) fails when tuned recall is below baseline recall, tuned has more false
// positives, its mean corner error (vs the rendered corners) is > 0.02 px worse, or a
// marker both report with the same corners gets a pose that differs by > 1 mm or > 1 deg.
// Times are host times: compare variants, not absolute device ms.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "apriltag.h"
#include "aruco/tagAruco4x4_50.h"
#include "aruco_stage.h"

// Heap calls made inside ArucoStage::run (apriltag allocates its images, union-find,
// cluster hash and per-cluster arrays every frame). Counted through the linker's --wrap
// (CMakeLists.txt); the device's musl allocator is much slower per call than glibc's, so
// this is reported next to the host time.
extern "C" {
void* __real_malloc(size_t);
void* __real_calloc(size_t, size_t);
void* __real_realloc(void*, size_t);
static unsigned long g_allocs = 0;
void* __wrap_malloc(size_t n) { ++g_allocs; return __real_malloc(n); }
void* __wrap_calloc(size_t a, size_t b) { ++g_allocs; return __real_calloc(a, b); }
void* __wrap_realloc(void* p, size_t n) { ++g_allocs; return __real_realloc(p, n); }
}

namespace {

constexpr int kW = 640, kH = 360, kFrameW = 640, kFrameH = 640, kBandY = 140;

struct Truth {
    int id;
    double cx, cy;
    double side;          // shortest edge of the black border in pixels
    double corners[4][2];  // black-border corners in the image
    double contrast = 0;   // rendered white minus black level (0..255)
    double tilt_deg = 0;   // larger of the two out-of-plane tilts
};

struct Scene {
    std::vector<uint8_t> rgb;  // kFrameW x kFrameH x 3
    std::vector<Truth> truth;
};

// 4-point homography: maps (u,v) in src to dst. h is row-major 3x3 with h[8] = 1.
bool homography4(const double src[4][2], const double dst[4][2], double h[9]) {
    double a[8][9] = {};
    for (int i = 0; i < 4; ++i) {
        const double u = src[i][0], v = src[i][1], x = dst[i][0], y = dst[i][1];
        double* r0 = a[2 * i];
        double* r1 = a[2 * i + 1];
        r0[0] = u; r0[1] = v; r0[2] = 1; r0[6] = -u * x; r0[7] = -v * x; r0[8] = x;
        r1[3] = u; r1[4] = v; r1[5] = 1; r1[6] = -u * y; r1[7] = -v * y; r1[8] = y;
    }
    for (int c = 0; c < 8; ++c) {
        int piv = c;
        for (int r = c + 1; r < 8; ++r)
            if (std::fabs(a[r][c]) > std::fabs(a[piv][c])) piv = r;
        if (std::fabs(a[piv][c]) < 1e-12) return false;
        for (int k = 0; k < 9; ++k) std::swap(a[c][k], a[piv][k]);
        for (int r = 0; r < 8; ++r) {
            if (r == c) continue;
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < 9; ++k) a[r][k] -= f * a[c][k];
        }
    }
    for (int i = 0; i < 8; ++i) h[i] = a[i][8] / a[i][i];
    h[8] = 1;
    return true;
}

bool invert3(const double m[9], double o[9]) {
    const double d = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
                     m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (std::fabs(d) < 1e-15) return false;
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

void apply(const double h[9], double u, double v, double& x, double& y) {
    const double w = h[6] * u + h[7] * v + h[8];
    x = (h[0] * u + h[1] * v + h[2]) / w;
    y = (h[3] * u + h[4] * v + h[5]) / w;
}

class SceneGen {
public:
    explicit SceneGen(unsigned seed) : rng_(seed) { fam_ = tagAruco4x4_50_create(); }
    ~SceneGen() { tagAruco4x4_50_destroy(fam_); }

    Scene make() {
        std::vector<float> g(kW * kH);
        background(g);
        Scene s;
        const int n = 1 + static_cast<int>(uni(0, 4.999));
        std::vector<std::array<double, 4>> boxes;
        for (int k = 0; k < n; ++k) {
            for (int attempt = 0; attempt < 40; ++attempt) {
                if (place_marker(g, boxes, s.truth)) break;
            }
        }
        blur(g);
        // Sensor noise: low in good light, higher when AE raises the gain.
        std::normal_distribution<float> noise(0.f, static_cast<float>(uni(1.5, 6.0)));
        s.rgb.assign(static_cast<size_t>(kFrameW) * kFrameH * 3, 0);
        const float tint[3] = {static_cast<float>(uni(0.9, 1.05)), 1.f, static_cast<float>(uni(0.85, 1.05))};
        for (int y = 0; y < kH; ++y) {
            uint8_t* row = s.rgb.data() + static_cast<size_t>(y + kBandY) * kFrameW * 3;
            for (int x = 0; x < kW; ++x) {
                for (int c = 0; c < 3; ++c) {
                    const float v = g[y * kW + x] * tint[c] + noise(rng_);
                    row[3 * x + c] = static_cast<uint8_t>(std::clamp(v, 0.f, 255.f));
                }
            }
        }
        return s;
    }

private:
    double uni(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng_); }

    void background(std::vector<float>& g) {
        // Smooth lighting: bilinear value noise at two scales plus a gradient.
        auto grid = [&](int cell, float amp, std::vector<float>& acc) {
            const int gw = kW / cell + 2, gh = kH / cell + 2;
            std::vector<float> v(gw * gh);
            for (auto& e : v) e = static_cast<float>(uni(-amp, amp));
            for (int y = 0; y < kH; ++y) {
                for (int x = 0; x < kW; ++x) {
                    const float fx = static_cast<float>(x) / cell, fy = static_cast<float>(y) / cell;
                    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy);
                    const float tx = fx - ix, ty = fy - iy;
                    const float a = v[iy * gw + ix], b = v[iy * gw + ix + 1];
                    const float c = v[(iy + 1) * gw + ix], d = v[(iy + 1) * gw + ix + 1];
                    acc[y * kW + x] += (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
                }
            }
        };
        const float base = static_cast<float>(uni(90, 170));
        const float gx = static_cast<float>(uni(-0.1, 0.1)), gy = static_cast<float>(uni(-0.15, 0.15));
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) g[y * kW + x] = base + gx * (x - kW / 2) + gy * (y - kH / 2);
        grid(80, 35.f, g);
        grid(12, 10.f, g);
        // Clutter: rectangles, ellipses, lines and small high-contrast grids (text, keys).
        const int shapes = static_cast<int>(uni(40, 90));
        for (int i = 0; i < shapes; ++i) {
            const double kind = uni(0, 1);
            const double cx = uni(0, kW), cy = uni(0, kH), ang = uni(0, M_PI);
            const float val = static_cast<float>(uni(10, 245));
            const double ca = std::cos(ang), sa = std::sin(ang);
            if (kind < 0.35) {  // rotated rectangle
                const double hw = uni(3, 70), hh = uni(3, 50);
                fill(g, cx, cy, hw + hh, [&](double dx, double dy) {
                    const double u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
                    return std::fabs(u) < hw && std::fabs(v) < hh;
                }, val);
            } else if (kind < 0.55) {  // ellipse
                const double rx = uni(3, 50), ry = uni(3, 40);
                fill(g, cx, cy, std::max(rx, ry), [&](double dx, double dy) {
                    const double u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
                    return (u * u) / (rx * rx) + (v * v) / (ry * ry) < 1;
                }, val);
            } else if (kind < 0.8) {  // line
                const double len = uni(20, 300), wid = uni(0.6, 2.5);
                fill(g, cx, cy, len / 2 + wid, [&](double dx, double dy) {
                    const double u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
                    return std::fabs(u) < len / 2 && std::fabs(v) < wid;
                }, val);
            } else {  // grid of small dark squares on a light patch
                const double cell = uni(3, 9);
                const int nx = static_cast<int>(uni(2, 8)), ny = static_cast<int>(uni(1, 4));
                const float light = static_cast<float>(uni(150, 240)), dark = static_cast<float>(uni(10, 80));
                const double hw = nx * cell, hh = ny * cell;
                fill(g, cx, cy, hw + hh, [&](double dx, double dy) {
                    const double u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
                    return std::fabs(u) < hw && std::fabs(v) < hh;
                }, light);
                std::bernoulli_distribution on(0.5);
                for (int a = 0; a < 2 * nx; ++a) {
                    for (int b = 0; b < 2 * ny; ++b) {
                        if (!on(rng_)) continue;
                        const double u0 = -hw + (a + 0.5) * cell, v0 = -hh + (b + 0.5) * cell;
                        const double px = cx + u0 * ca - v0 * sa, py = cy + u0 * sa + v0 * ca;
                        fill(g, px, py, cell, [&](double dx, double dy) {
                            const double u = dx * ca + dy * sa, v = -dx * sa + dy * ca;
                            return std::fabs(u) < cell * 0.35 && std::fabs(v) < cell * 0.35;
                        }, dark);
                    }
                }
            }
        }
    }

    template <typename F>
    void fill(std::vector<float>& g, double cx, double cy, double r, F inside, float val) {
        const int x0 = std::max(0, static_cast<int>(cx - r - 1)), x1 = std::min(kW - 1, static_cast<int>(cx + r + 1));
        const int y0 = std::max(0, static_cast<int>(cy - r - 1)), y1 = std::min(kH - 1, static_cast<int>(cy + r + 1));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                if (inside(x + 0.5 - cx, y + 0.5 - cy)) g[y * kW + x] = val;
    }

    bool place_marker(std::vector<float>& g, std::vector<std::array<double, 4>>& boxes, std::vector<Truth>& truth) {
        const int id = static_cast<int>(uni(0, 49.999));
        const double side = std::exp(uni(std::log(14.0), std::log(150.0)));  // black border, image px
        const double tilt_x = uni(-1, 1) * (uni(0, 1) < 0.5 ? 0.35 : 1.05);  // up to ~60 degrees
        const double tilt_y = uni(-1, 1) * (uni(0, 1) < 0.5 ? 0.35 : 1.05);
        const double rot = uni(0, 2 * M_PI);
        // 3D square (black border spans -0.5..0.5) rotated and projected with f = 600.
        const double f = 600.0, z = f / side;
        const double cx0 = uni(side, kW - side), cy0 = uni(side * 0.8, kH - side * 0.8);
        double tagc[4][2], img[4][2];
        const double tw = fam_->total_width;  // 8 cells incl. 1 white quiet cell per side
        const double cu[4] = {-0.5, 0.5, 0.5, -0.5}, cv[4] = {-0.5, -0.5, 0.5, 0.5};
        const double cr = std::cos(rot), sr = std::sin(rot);
        const double cxr = std::cos(tilt_x), sxr = std::sin(tilt_x), cyr = std::cos(tilt_y), syr = std::sin(tilt_y);
        // quiet zone included: scale the 3D square to 8/6 of the border
        const double q = tw / (tw - 2);
        for (int i = 0; i < 4; ++i) {
            double X = cu[i] * q, Y = cv[i] * q, Z = 0;
            double x1 = X * cr - Y * sr, y1 = X * sr + Y * cr;                // Rz
            double y2 = y1 * cxr - Z * sxr, z2 = y1 * sxr + Z * cxr;            // Rx
            double x3 = x1 * cyr + z2 * syr, z3 = -x1 * syr + z2 * cyr;         // Ry
            img[i][0] = cx0 + f * x3 / (z + z3);
            img[i][1] = cy0 + f * y2 / (z + z3);
            tagc[i][0] = (cu[i] + 0.5) * tw;
            tagc[i][1] = (cv[i] + 0.5) * tw;
        }
        double h[9], hi[9];
        if (!homography4(tagc, img, h) || !invert3(h, hi)) return false;
        double bx0 = 1e9, by0 = 1e9, bx1 = -1e9, by1 = -1e9;
        for (auto& p : img) {
            bx0 = std::min(bx0, p[0]); by0 = std::min(by0, p[1]);
            bx1 = std::max(bx1, p[0]); by1 = std::max(by1, p[1]);
        }
        if (bx0 < 2 || by0 < 2 || bx1 > kW - 3 || by1 > kH - 3) return false;
        for (auto& b : boxes)
            if (!(bx1 + 4 < b[0] || bx0 - 4 > b[2] || by1 + 4 < b[1] || by0 - 4 > b[3])) return false;
        // Black-border corners in the image for the truth side.
        double bc[4][2];
        const double b0 = 1, b1 = tw - 1;
        const double bu[4] = {b0, b1, b1, b0}, bv[4] = {b0, b0, b1, b1};
        for (int i = 0; i < 4; ++i) apply(h, bu[i], bv[i], bc[i][0], bc[i][1]);
        double min_side = 1e9;
        for (int i = 0; i < 4; ++i)
            min_side = std::min(min_side, std::hypot(bc[(i + 1) % 4][0] - bc[i][0], bc[(i + 1) % 4][1] - bc[i][1]));
        if (min_side < 14) return false;

        image_u8_t* tag = apriltag_to_image(fam_, static_cast<uint32_t>(id));
        // Lighting: normal, dim, and low contrast.
        const double light = uni(0, 1) < 0.25 ? uni(0.35, 0.6) : uni(0.75, 1.0);
        double lo = uni(10, 45), hi_v = uni(170, 240);
        if (uni(0, 1) < 0.2) hi_v = lo + uni(55, 90);  // low contrast print / glare
        lo *= light;
        hi_v *= light;
        for (int y = std::max(0, static_cast<int>(by0)); y <= std::min(kH - 1, static_cast<int>(by1) + 1); ++y) {
            for (int x = std::max(0, static_cast<int>(bx0)); x <= std::min(kW - 1, static_cast<int>(bx1) + 1); ++x) {
                double acc = 0;
                int hits = 0;
                for (int sy = 0; sy < 3; ++sy) {
                    for (int sx = 0; sx < 3; ++sx) {
                        double u, v;
                        apply(hi, x + (sx + 0.5) / 3.0, y + (sy + 0.5) / 3.0, u, v);
                        if (u < 0 || v < 0 || u >= tw || v >= tw) continue;
                        const int iu = static_cast<int>(u), iv = static_cast<int>(v);
                        acc += tag->buf[iv * tag->stride + iu] ? hi_v : lo;
                        ++hits;
                    }
                }
                if (hits) g[y * kW + x] = static_cast<float>((acc + (9 - hits) * g[y * kW + x]) / 9.0);
            }
        }
        image_u8_destroy(tag);
        boxes.push_back({bx0, by0, bx1, by1});
        double tcx, tcy;
        apply(h, tw / 2, tw / 2, tcx, tcy);
        Truth tr{id, tcx, tcy, min_side, {}, hi_v - lo,
                 std::max(std::fabs(tilt_x), std::fabs(tilt_y)) * 180.0 / M_PI};
        for (int i = 0; i < 4; ++i) {
            tr.corners[i][0] = bc[i][0];
            tr.corners[i][1] = bc[i][1];
        }
        truth.push_back(tr);
        return true;
    }

    void blur(std::vector<float>& g) {  // Gaussian sigma ~0.8, separable [1 4 6 4 1]/16 at 0.5 weight
        std::vector<float> t(g.size());
        const float k[5] = {0.03f, 0.22f, 0.5f, 0.22f, 0.03f};
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) {
                float s = 0;
                for (int i = -2; i <= 2; ++i) s += k[i + 2] * g[y * kW + std::clamp(x + i, 0, kW - 1)];
                t[y * kW + x] = s;
            }
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) {
                float s = 0;
                for (int i = -2; i <= 2; ++i) s += k[i + 2] * t[std::clamp(y + i, 0, kH - 1) * kW + x];
                g[y * kW + x] = s;
            }
    }

    std::mt19937 rng_;
    apriltag_family_t* fam_;
};

struct Stats {
    double mean_ms = 0, median_ms = 0, p90_ms = 0;
    double gray_ms = 0, detect_ms = 0, pose_ms = 0;  // means
    double allocs = 0;                                // heap calls per frame
    double corner_err_sum = 0, corner_err_max = 0;    // px, matched markers vs truth
    int truth = 0, found = 0, fp = 0;
    int bucket_truth[4] = {}, bucket_found[4] = {};
};

int bucket(double side) { return side < 20 ? 0 : side < 40 ? 1 : side < 80 ? 2 : 3; }

Stats run_variant(const rv::ArucoConfig& cfg, const std::vector<Scene>& scenes, int reps,
                  std::vector<std::vector<rv::MarkerOut>>& outs, std::vector<std::vector<int>>* match = nullptr) {
    rv::ArucoStage stage;
    Stats st;
    if (!stage.init(cfg)) {
        std::fprintf(stderr, "init failed\n");
        std::exit(2);
    }
    std::vector<double> times;
    std::vector<rv::MarkerOut> out;
    for (const Scene& s : scenes) {
        const uint8_t* band = s.rgb.data() + static_cast<size_t>(kBandY) * kFrameW * 3;
        for (int r = 0; r < reps; ++r) {
            const unsigned long a0 = g_allocs;
            const auto t0 = std::chrono::steady_clock::now();
            stage.run(band, kW, kH, kFrameW * 3, out);
            st.allocs += static_cast<double>(g_allocs - a0);
            times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
            st.gray_ms += stage.last_timing().gray_ms;
            st.detect_ms += stage.last_timing().detect_ms;
            st.pose_ms += stage.last_timing().pose_ms;
        }
        outs.push_back(out);
        std::vector<bool> used(out.size(), false);
        if (match) match->emplace_back(s.truth.size(), -1);
        for (size_t ti = 0; ti < s.truth.size(); ++ti) {
            const Truth& t = s.truth[ti];
            ++st.truth;
            ++st.bucket_truth[bucket(t.side)];
            for (size_t i = 0; i < out.size(); ++i) {
                if (used[i] || out[i].id != t.id) continue;
                if (std::hypot(out[i].c[0] - t.cx, out[i].c[1] - t.cy) > std::max(3.0, 0.25 * t.side)) continue;
                used[i] = true;
                if (match) match->back()[ti] = static_cast<int>(i);
                ++st.found;
                // mean corner distance to the truth, best of the 4 cyclic orders and both windings
                double best = 1e9;
                for (int dir = 0; dir < 2; ++dir) {
                    for (int r = 0; r < 4; ++r) {
                        double e = 0;
                        for (int k = 0; k < 4; ++k) {
                            const int j = dir == 0 ? (k + r) % 4 : (r - k + 4) % 4;
                            e += std::hypot(out[i].p[k][0] - t.corners[j][0], out[i].p[k][1] - t.corners[j][1]) / 4;
                        }
                        best = std::min(best, e);
                    }
                }
                st.corner_err_sum += best;
                st.corner_err_max = std::max(st.corner_err_max, best);
                ++st.bucket_found[bucket(t.side)];
                break;
            }
        }
        for (bool u : used) st.fp += u ? 0 : 1;
    }
    std::sort(times.begin(), times.end());
    double sum = 0;
    for (double t : times) sum += t;
    st.mean_ms = sum / times.size();
    st.gray_ms /= times.size();
    st.detect_ms /= times.size();
    st.pose_ms /= times.size();
    st.allocs /= times.size();
    st.median_ms = times[times.size() / 2];
    st.p90_ms = times[times.size() * 9 / 10];
    return st;
}

void print(const char* name, const rv::ArucoConfig& c, const Stats& s) {
    std::printf("%-8s decimate=%.1f wb=%d cluster=%d nmax=%d sharpen=%.2f refine=%d | "
                "ms/frame mean %.3f median %.3f p90 %.3f (gray %.3f detect %.3f pose %.3f), heap calls %.0f | corner error mean %.3f max %.2f px | recall %d/%d = %.1f%% "
                "(<20px %d/%d, 20-40 %d/%d, 40-80 %d/%d, >=80 %d/%d) | false positives %d\n",
                name, c.quad_decimate, c.min_white_black_diff, c.min_cluster_pixels, c.max_nmaxima,
                c.decode_sharpening, c.refine_edges ? 1 : 0, s.mean_ms, s.median_ms, s.p90_ms, s.gray_ms, s.detect_ms, s.pose_ms, s.allocs, s.corner_err_sum / std::max(1, s.found), s.corner_err_max, s.found, s.truth,
                100.0 * s.found / std::max(1, s.truth), s.bucket_found[0], s.bucket_truth[0], s.bucket_found[1],
                s.bucket_truth[1], s.bucket_found[2], s.bucket_truth[2], s.bucket_found[3], s.bucket_truth[3], s.fp);
}

// --sweep: one row per quad_decimate (rv defaults otherwise). Recall by marker side (shortest
// black-border edge in px; at fx 600 a marker of S m is 600 * S / z px at z m, e.g. 5 cm at
// 40 cm = 75 px, 2 cm at 40 cm = 30 px), the smallest side found, the smallest side bucket
// from which every larger bucket keeps >= 95 % recall, corner error vs the rendered corners,
// and the pose difference vs the first value of the list for markers both find.
constexpr int kSweepBuckets = 7;
constexpr double kLowContrast = 90;  // the generator's low-contrast prints and dim light
const double kSweepEdges[kSweepBuckets + 1] = {14, 20, 25, 30, 40, 60, 80, 1e9};
int sweep_bucket(double side) {
    int b = 0;
    while (b + 1 < kSweepBuckets && side >= kSweepEdges[b + 1]) ++b;
    return b;
}

struct SweepRow {
    double decimate = 0;
    Stats st;
    int truth[kSweepBuckets] = {}, found[kSweepBuckets] = {};
    // markers of 40 px and larger by rendered contrast (white - black >= kLowContrast or below)
    int big_truth[2] = {}, big_found[2] = {};
    double min_found_side = 0;
    std::vector<std::vector<rv::MarkerOut>> outs;
    std::vector<std::vector<int>> match;
};

double rot_diff_deg(const rv::MarkerOut& a, const rv::MarkerOut& b) {
    double dr = 0;
    for (int k = 0; k < 3; ++k) {
        const double d = std::fabs(a.r_deg[k] - b.r_deg[k]);
        dr = std::max(dr, std::min(d, 360 - d));
    }
    return dr;
}

double recall_from(const SweepRow& r, int first_bucket) {
    int t = 0, f = 0;
    for (int b = first_bucket; b < kSweepBuckets; ++b) t += r.truth[b], f += r.found[b];
    return t ? static_cast<double>(f) / t : 1.0;
}

int run_sweep(const std::vector<double>& decimates, const rv::ArucoConfig& base, const std::vector<Scene>& scenes,
              int reps, bool check) {
    std::vector<SweepRow> rows;
    for (double d : decimates) {
        rv::ArucoConfig c = base;
        c.quad_decimate = d;
        SweepRow r;
        r.decimate = d;
        r.st = run_variant(c, scenes, reps, r.outs, &r.match);
        r.min_found_side = 1e9;
        for (size_t s = 0; s < scenes.size(); ++s) {
            for (size_t ti = 0; ti < scenes[s].truth.size(); ++ti) {
                const double side = scenes[s].truth[ti].side;
                const int b = sweep_bucket(side);
                const bool big = side >= 40, low = scenes[s].truth[ti].contrast < kLowContrast;
                ++r.truth[b];
                if (big) ++r.big_truth[low];
                if (r.match[s][ti] >= 0) {
                    ++r.found[b];
                    if (big) ++r.big_found[low];
                    r.min_found_side = std::min(r.min_found_side, side);
                }
            }
        }
        rows.push_back(std::move(r));
    }
    std::printf("\n| decimate | ms/frame mean | median | detect ms | recall | 14-20 px | 20-25 | 25-30 | 30-40 | 40-60 "
                "| 60-80 | >=80 | >=40 px contrast >=90 | >=40 px contrast <90 | smallest found | >=95%% from | false pos. | corner err px | vs %.1f: |dt| mm median / p95 / max "
                "| rot deg median / p95 / max | rot > 5 deg |\n",
                rows[0].decimate);
    std::printf("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
    for (const SweepRow& r : rows) {
        // pose difference vs the first row for truths both find
        std::vector<double> dts, drs;
        for (size_t s = 0; s < scenes.size(); ++s) {
            for (size_t ti = 0; ti < scenes[s].truth.size(); ++ti) {
                const int a = rows[0].match[s][ti], b = r.match[s][ti];
                if (a < 0 || b < 0) continue;
                const rv::MarkerOut& ma = rows[0].outs[s][a];
                const rv::MarkerOut& mb = r.outs[s][b];
                dts.push_back(1000 * std::sqrt(std::pow(ma.t[0] - mb.t[0], 2) + std::pow(ma.t[1] - mb.t[1], 2) +
                                               std::pow(ma.t[2] - mb.t[2], 2)));
                drs.push_back(rot_diff_deg(ma, mb));
            }
        }
        std::sort(dts.begin(), dts.end());
        std::sort(drs.begin(), drs.end());
        auto med = [](const std::vector<double>& v) { return v.empty() ? 0.0 : v[v.size() / 2]; };
        auto mx = [](const std::vector<double>& v) { return v.empty() ? 0.0 : v.back(); };
        auto p95 = [](const std::vector<double>& v) { return v.empty() ? 0.0 : v[v.size() * 95 / 100]; };
        int from = kSweepBuckets;
        while (from > 0 && recall_from(r, from - 1) >= 0.95 &&
               (r.truth[from - 1] == 0 || static_cast<double>(r.found[from - 1]) / r.truth[from - 1] >= 0.95))
            --from;
        std::printf("| %.1f | %.2f | %.2f | %.2f | %.1f%% |", r.decimate, r.st.mean_ms, r.st.median_ms, r.st.detect_ms,
                    100.0 * r.st.found / std::max(1, r.st.truth));
        for (int b = 0; b < kSweepBuckets; ++b)
            std::printf(" %d/%d |", r.found[b], r.truth[b]);
        for (int low = 0; low < 2; ++low)
            std::printf(" %.1f%% |", 100.0 * r.big_found[low] / std::max(1, r.big_truth[low]));
        const size_t flips = static_cast<size_t>(drs.end() - std::upper_bound(drs.begin(), drs.end(), 5.0));
        std::printf(" %.1f px | %s | %d | %.3f | %.2f / %.2f / %.0f | %.2f / %.2f / %.0f | %zu/%zu |\n", r.min_found_side,
                    from == kSweepBuckets ? "none" : (std::to_string(static_cast<int>(kSweepEdges[from])) + " px").c_str(),
                    r.st.fp, r.st.corner_err_sum / std::max(1, r.st.found), med(dts), p95(dts), mx(dts), med(drs),
                    p95(drs), mx(drs), flips, drs.size());
    }
    if (!check) return 0;
    // The default decimate (rv::ArucoConfig) must keep, against the first value of the list,
    // the recall of markers of 40 px and larger at normal contrast within 3 points, and the
    // corner error (the pose input; corners are refined at full resolution) within 0.15 px.
    const rv::ArucoConfig defaults;
    const SweepRow* def = nullptr;
    for (const SweepRow& r : rows)
        if (std::fabs(r.decimate - defaults.quad_decimate) < 1e-9) def = &r;
    if (!def) {
        std::printf("FAIL: the sweep does not include the default decimate %.1f\n", defaults.quad_decimate);
        return 1;
    }
    auto big_recall = [](const SweepRow& r) { return static_cast<double>(r.big_found[0]) / std::max(1, r.big_truth[0]); };
    const double ref = big_recall(rows[0]), got = big_recall(*def);
    const double ref_err = rows[0].st.corner_err_sum / std::max(1, rows[0].st.found);
    const double def_err = def->st.corner_err_sum / std::max(1, def->st.found);
    if (got < ref - 0.03 || def_err > ref_err + 0.15) {
        std::printf("FAIL: decimate %.1f: recall >= 40 px at contrast >= %.0f %.1f%% (decimate %.1f: %.1f%%), "
                    "corner error %.3f (%.3f) px\n",
                    def->decimate, kLowContrast, 100 * got, rows[0].decimate, 100 * ref, def_err, ref_err);
        return 1;
    }
    std::printf("OK: default decimate %.1f: recall >= 40 px at contrast >= %.0f %.1f%% (decimate %.1f: %.1f%%), "
                "corner error %.3f (%.3f) px\n",
                def->decimate, kLowContrast, 100 * got, rows[0].decimate, 100 * ref, def_err, ref_err);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    int scenes_n = 120, reps = 3;
    bool check = false;
    std::string variant = "both";
    rv::ArucoConfig tuned;  // rv defaults
    bool custom = false;
    bool no_filter = false;  // diagnostics: count raw detections (margin/side filter off)
    std::vector<double> sweep;  // --sweep 1,1.5,2: one table row per quad_decimate
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s needs a value\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--scenes") scenes_n = std::atoi(next());
        else if (a == "--reps") reps = std::atoi(next());
        else if (a == "--check") check = true;
        else if (a == "--variant") variant = next();
        else if (a == "--wb") tuned.min_white_black_diff = std::atoi(next()), custom = true;
        else if (a == "--cluster") tuned.min_cluster_pixels = std::atoi(next()), custom = true;
        else if (a == "--decimate") tuned.quad_decimate = std::atof(next()), custom = true;
        else if (a == "--sharpen") tuned.decode_sharpening = std::atof(next()), custom = true;
        else if (a == "--nmax") tuned.max_nmaxima = std::atoi(next()), custom = true;
        else if (a == "--sigma") tuned.quad_sigma = std::atof(next()), custom = true;
        else if (a == "--refine") tuned.refine_edges = std::atoi(next()) != 0, custom = true;
        else if (a == "--no-filter") no_filter = true;
        else if (a == "--sweep") {
            std::string list = next();
            for (size_t pos = 0; pos <= list.size();) {
                const size_t comma = std::min(list.find(',', pos), list.size());
                const double d = std::atof(list.substr(pos, comma - pos).c_str());
                if (d < 1.0) {
                    std::fprintf(stderr, "--sweep: decimate values must be >= 1\n");
                    return 2;
                }
                sweep.push_back(d);
                pos = comma + 1;
            }
        }
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }

    // aruco-detector 0.1.3: decimate 2, sigma 0, refine_edges, 1 thread, apriltag defaults.
    rv::ArucoConfig baseline;
    baseline.quad_decimate = 2.0;
    baseline.quad_sigma = 0.0;
    baseline.refine_edges = true;
    baseline.decode_sharpening = 0.25;
    baseline.min_white_black_diff = 5;
    baseline.min_cluster_pixels = 24;
    baseline.max_nmaxima = 10;
    baseline.max_line_fit_mse = 10.0f;
    baseline.apriltag_pose = true;
    baseline.lazy_clusters = false;
    baseline.reuse_buffers = false;
    baseline.size_gate = false;

    if (no_filter) {
        baseline.filter.min_margin = baseline.filter.min_side_px = 0;
        tuned.filter.min_margin = tuned.filter.min_side_px = 0;
    }

    SceneGen gen(20261109u);
    std::vector<Scene> scenes;
    scenes.reserve(scenes_n);
    for (int i = 0; i < scenes_n; ++i) scenes.push_back(gen.make());
    std::printf("%d scenes 640x360 (stride %d), %d reps each, filter margin>=%.0f side>=%.0fpx\n", scenes_n,
                kFrameW * 3, reps, baseline.filter.min_margin, baseline.filter.min_side_px);

    if (!sweep.empty()) return run_sweep(sweep, tuned, scenes, reps, check);

    Stats b, t;
    std::vector<std::vector<rv::MarkerOut>> bo, to;
    if (variant == "baseline" || variant == "both") {
        b = run_variant(baseline, scenes, reps, bo);
        print("baseline", baseline, b);
    }
    if (variant == "tuned" || variant == "both") {
        t = run_variant(tuned, scenes, reps, to);
        print(custom ? "custom" : "tuned", tuned, t);
    }
    double max_dt_mm = 0, max_dr_deg = 0, max_dc_px = 0;
    int same = 0, pose_diff = 0, corner_diff = 0;
    if (variant == "both") {
        std::printf("speed-up (mean): %.2fx\n", b.mean_ms / std::max(1e-9, t.mean_ms));
        // Markers both variants report: corner and pose differences.
        for (size_t s = 0; s < bo.size() && s < to.size(); ++s) {
            for (const auto& mb : bo[s]) {
                for (const auto& mt : to[s]) {
                    if (mb.id != mt.id || std::hypot(mb.c[0] - mt.c[0], mb.c[1] - mt.c[1]) > 2) continue;
                    double dc = 0;
                    for (int k = 0; k < 4; ++k)
                        dc = std::max(dc, std::hypot(mb.p[k][0] - mt.p[k][0], mb.p[k][1] - mt.p[k][1]));
                    max_dc_px = std::max(max_dc_px, dc);
                    if (dc > 1e-9) {  // the detector found other corners: not a pose-solver comparison
                        ++corner_diff;
                        continue;
                    }
                    ++same;
                    const double dt_mm = 1000 * std::sqrt(std::pow(mb.t[0] - mt.t[0], 2) + std::pow(mb.t[1] - mt.t[1], 2) +
                                                          std::pow(mb.t[2] - mt.t[2], 2));
                    double dr = 0;
                    for (int k = 0; k < 3; ++k) {
                        double d = std::fabs(mb.r_deg[k] - mt.r_deg[k]);
                        dr = std::max(dr, std::min(d, 360 - d));
                    }
                    if (dt_mm > 1.0 || dr > 1.0) ++pose_diff;
                    max_dt_mm = std::max(max_dt_mm, dt_mm);
                    max_dr_deg = std::max(max_dr_deg, dr);
                }
            }
        }
        std::printf("markers found by both: %d with other corners (max %.4f px), %d with the same corners: "
                    "max |dt| %.4f mm, max euler angle difference %.4f deg (marker 0.05 m); poses differing by > 1 mm or > 1 deg: %d\n",
                    corner_diff, max_dc_px, same, max_dt_mm, max_dr_deg, pose_diff);
    }
    if (check && variant == "both") {
        const double b_err = b.corner_err_sum / std::max(1, b.found), t_err = t.corner_err_sum / std::max(1, t.found);
        if (t.found < b.found || t.fp > b.fp || pose_diff > 0 || t_err > b_err + 0.02) {
            std::printf("FAIL: tuned recall %d (baseline %d), false positives %d (baseline %d), "
                        "poses differing %d, corner error %.3f (baseline %.3f) px\n", t.found, b.found, t.fp, b.fp,
                        pose_diff, t_err, b_err);
            return 1;
        }
        std::printf("OK: tuned recall >= baseline, false positives <= baseline, corner error within 0.02 px, "
                    "same poses from the same corners\n");
    }
    return 0;
}
