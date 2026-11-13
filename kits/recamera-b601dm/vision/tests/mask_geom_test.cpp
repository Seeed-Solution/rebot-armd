#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "json.hpp"
#include "mask_geom.h"

using json = nlohmann::json;
using namespace rv;

static int g_fail = 0;
#define CHECK(c)                                                             \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);         \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

#define CHECK_MSG(c, msg)                                                   \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::printf("FAIL %s:%d: %s (%s)\n", __FILE__, __LINE__, #c,     \
                        (msg));                                              \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

static std::string find_vectors_path() {
#ifdef TEST_DATA_DIR
    std::string p = std::string(TEST_DATA_DIR) + "/cross_section_vectors.json";
    std::ifstream f(p);
    if (f.good()) return p;
#endif
    const char* candidates[] = {
        "kits/recamera-b601dm/vision/tests/data/cross_section_vectors.json",
        "data/cross_section_vectors.json",
        "../kits/recamera-b601dm/vision/tests/data/cross_section_vectors.json",
        "../../kits/recamera-b601dm/vision/tests/data/cross_section_vectors.json",
        "../../../kits/recamera-b601dm/vision/tests/data/cross_section_vectors.json",
    };
    for (const char* c : candidates) {
        std::ifstream fc(c);
        if (fc.good()) return c;
    }
    return "";
}

struct TestCase {
    std::string name;
    double scale_x{1.0};
    double scale_y{1.0};
    bool has_expected_angle{false};
    double expected_angle_deg{0.0};
    bool has_expected_width{false};
    double expected_width_px{0.0};
    bool has_expected_conf{false};
    double expected_conf{0.0};
    double angle_tol{0.5};
    double width_tol{1.5};
    double conf_min{0.0};
    double conf_max{1.0};
    bool has_conf_min{false};
    bool has_conf_max{false};
    bool snap_centroid{false};
    std::vector<double> xs;
    std::vector<double> ys;
};

static std::map<std::string, TestCase> load_vectors() {
    std::string path = find_vectors_path();
    CHECK_MSG(!path.empty(), "Could not locate cross_section_vectors.json");
    std::map<std::string, TestCase> cases;
    if (path.empty()) return cases;

    std::ifstream f(path);
    CHECK_MSG(f.is_open(), "Could not open cross_section_vectors.json");
    if (!f.is_open()) return cases;

    json j;
    f >> j;
    for (const auto& item : j) {
        TestCase tc;
        tc.name = item["name"].get<std::string>();
        tc.scale_x = item.value("scale_x", 1.0);
        tc.scale_y = item.value("scale_y", 1.0);
        if (item.contains("expected_angle_deg") && !item["expected_angle_deg"].is_null()) {
            tc.has_expected_angle = true;
            tc.expected_angle_deg = item["expected_angle_deg"].get<double>();
        }
        if (item.contains("expected_width_px") && !item["expected_width_px"].is_null()) {
            tc.has_expected_width = true;
            tc.expected_width_px = item["expected_width_px"].get<double>();
        }
        if (item.contains("expected_conf") && !item["expected_conf"].is_null()) {
            tc.has_expected_conf = true;
            tc.expected_conf = item["expected_conf"].get<double>();
        }
        tc.angle_tol = item.value("angle_tol", 0.5);
        tc.width_tol = item.value("width_tol", 1.5);
        if (item.contains("conf_min")) {
            tc.has_conf_min = true;
            tc.conf_min = item["conf_min"].get<double>();
        }
        if (item.contains("conf_max")) {
            tc.has_conf_max = true;
            tc.conf_max = item["conf_max"].get<double>();
        }
        tc.snap_centroid = item.value("snap_centroid", false);

        for (const auto& pt : item["points"]) {
            tc.xs.push_back(pt[0].get<double>());
            tc.ys.push_back(pt[1].get<double>());
        }
        cases[tc.name] = tc;
    }
    return cases;
}

static double angle_err(double a, double b) {
    double diff = std::fmod(a - b + 90.0, 180.0);
    if (diff < 0.0) diff += 180.0;
    return std::abs(diff - 90.0);
}

static void test_all_cases_execute_and_produce_valid_outputs(const std::map<std::string, TestCase>& cases) {
    for (const auto& kv : cases) {
        const auto& c = kv.second;
        Moments mo = mask_moments(c.xs.data(), c.ys.data(), c.xs.size(), c.scale_x, c.scale_y);
        CHECK_MSG(mo.angle_deg >= -90.0 && mo.angle_deg < 90.0, c.name.c_str());
        CHECK_MSG(mo.conf >= 0.0 && mo.conf <= 1.0, c.name.c_str());

        CrossSection cs = mask_cross_section(c.xs.data(), c.ys.data(), c.xs.size(), 5, 8.0, c.scale_x, c.scale_y);
        CHECK_MSG(cs.ok, c.name.c_str());
        CHECK_MSG(cs.angle_deg >= -90.0 && cs.angle_deg < 90.0, c.name.c_str());
        CHECK_MSG(cs.conf >= 0.0 && cs.conf <= 1.0, c.name.c_str());
        CHECK_MSG(cs.width_px >= 0.0, c.name.c_str());

        if (c.has_expected_angle) {
            double err = angle_err(cs.angle_deg, c.expected_angle_deg);
            CHECK_MSG(err <= c.angle_tol, (c.name + " angle mismatch").c_str());
        }
        if (c.has_expected_width) {
            CHECK_MSG(std::abs(cs.width_px - c.expected_width_px) <= c.width_tol,
                      (c.name + " width mismatch").c_str());
        }
        if (c.has_conf_min) {
            CHECK_MSG(cs.conf >= c.conf_min, (c.name + " conf < conf_min").c_str());
        }
        if (c.has_conf_max) {
            CHECK_MSG(cs.conf <= c.conf_max, (c.name + " conf > conf_max").c_str());
        }
        if (c.has_expected_conf && !c.has_conf_min && !c.has_conf_max) {
            CHECK_MSG(std::abs(cs.conf - c.expected_conf) <= 0.01,
                      (c.name + " conf mismatch").c_str());
        }
    }
}

static void test_rect_angles_and_widths(const std::map<std::string, TestCase>& cases) {
    struct RectExpectation {
        const char* name;
        double expected_angle;
    };
    const RectExpectation rects[] = {
        {"rect_0", 0.0},
        {"rect_30", 30.0},
        {"rect_75", 75.0},
    };
    for (const auto& r : rects) {
        auto it = cases.find(r.name);
        CHECK_MSG(it != cases.end(), r.name);
        if (it == cases.end()) continue;
        const auto& c = it->second;
        CrossSection cs = mask_cross_section(c.xs.data(), c.ys.data(), c.xs.size(), 5, 8.0, c.scale_x, c.scale_y);

        double err = angle_err(cs.angle_deg, r.expected_angle);
        CHECK_MSG(err <= 0.5, (std::string(r.name) + " angle error > 0.5").c_str());
        CHECK_MSG(std::abs(cs.width_px - 20.0) <= 1.5, (std::string(r.name) + " width outside 20+-1.5").c_str());
        CHECK_MSG(cs.conf >= 0.45 && cs.conf <= 0.65, (std::string(r.name) + " conf outside [0.45, 0.65]").c_str());
    }
}

static void test_stretched_rect_matches_unstretched(const std::map<std::string, TestCase>& cases) {
    auto it0 = cases.find("rect_0");
    auto it_str = cases.find("rect_stretched");
    CHECK(it0 != cases.end() && it_str != cases.end());
    if (it0 == cases.end() || it_str == cases.end()) return;

    const auto& c0 = it0->second;
    const auto& cs_str = it_str->second;

    CrossSection res0 = mask_cross_section(c0.xs.data(), c0.ys.data(), c0.xs.size(), 5, 8.0, c0.scale_x, c0.scale_y);
    CrossSection res_str = mask_cross_section(cs_str.xs.data(), cs_str.ys.data(), cs_str.xs.size(), 5, 8.0, cs_str.scale_x, cs_str.scale_y);

    CHECK(res0.ok && res_str.ok);
    // a resampled mask differs from the original only by discretisation
    auto ang_diff = [](double a, double b) { return std::abs(std::fmod(a - b + 270.0, 180.0) - 90.0); };
    CHECK(ang_diff(res_str.angle_deg, res0.angle_deg) <= 0.5);
    CHECK(std::abs(res_str.conf - res0.conf) <= 0.03);
    CHECK(std::abs(res_str.width_px - res0.width_px) <= 1.0);

    Moments mo0 = mask_moments(c0.xs.data(), c0.ys.data(), c0.xs.size(), c0.scale_x, c0.scale_y);
    Moments mo_str = mask_moments(cs_str.xs.data(), cs_str.ys.data(), cs_str.xs.size(), cs_str.scale_x, cs_str.scale_y);
    CHECK(ang_diff(mo_str.angle_deg, mo0.angle_deg) <= 0.5);
    CHECK(std::abs(mo_str.conf - mo0.conf) <= 0.03);
}

static void test_disc_confidence_low(const std::map<std::string, TestCase>& cases) {
    auto it = cases.find("disc");
    CHECK(it != cases.end());
    if (it == cases.end()) return;
    const auto& c = it->second;

    CrossSection cs = mask_cross_section(c.xs.data(), c.ys.data(), c.xs.size(), 5, 8.0, c.scale_x, c.scale_y);
    CHECK(cs.conf < 0.1);
    CHECK(std::abs(cs.width_px - 40.0) <= 2.0);
}

static void test_square_geometry(const std::map<std::string, TestCase>& cases) {
    auto it = cases.find("square");
    CHECK(it != cases.end());
    if (it == cases.end()) return;
    const auto& c = it->second;

    CrossSection cs = mask_cross_section(c.xs.data(), c.ys.data(), c.xs.size(), 5, 8.0, c.scale_x, c.scale_y);
    CHECK(std::abs(cs.width_px - 40.0) <= 1.0);
    CHECK(cs.conf < 0.35);
}

static void test_ring_and_l_shape_snap_centroid(const std::map<std::string, TestCase>& cases) {
    const char* names[] = {"ring", "l_shape"};
    for (const char* name : names) {
        auto it = cases.find(name);
        CHECK(it != cases.end());
        if (it == cases.end()) continue;
        const auto& c = it->second;

        // Verify un-snapped mean centroid is outside mask
        double sum_px = 0.0, sum_py = 0.0;
        for (size_t i = 0; i < c.xs.size(); ++i) {
            sum_px += c.xs[i] * c.scale_x;
            sum_py += c.ys[i] * c.scale_y;
        }
        double gx_mean = sum_px / static_cast<double>(c.xs.size());
        double gy_mean = sum_py / static_cast<double>(c.xs.size());
        int qx = static_cast<int>(std::nearbyint(gx_mean / c.scale_x));
        int qy = static_cast<int>(std::nearbyint(gy_mean / c.scale_y));

        bool found = false;
        for (size_t i = 0; i < c.xs.size(); ++i) {
            if (static_cast<int>(std::nearbyint(c.xs[i])) == qx &&
                static_cast<int>(std::nearbyint(c.ys[i])) == qy) {
                found = true;
                break;
            }
        }
        CHECK_MSG(!found, (std::string(name) + " expected centroid outside mask").c_str());

        CrossSection cs = mask_cross_section(c.xs.data(), c.ys.data(), c.xs.size(), 5, 8.0, c.scale_x, c.scale_y);
        CHECK(cs.ok);
        CHECK(cs.angle_deg >= -90.0 && cs.angle_deg < 90.0);
        CHECK(cs.conf >= 0.0 && cs.conf <= 1.0);
        CHECK(cs.width_px > 0.0);
    }
}

static void test_mask_view_equivalence(const std::map<std::string, TestCase>& cases) {
    const char* test_names[] = {"rect_0", "rect_30", "square", "disc"};
    for (const char* name : test_names) {
        auto it = cases.find(name);
        CHECK(it != cases.end());
        if (it == cases.end()) continue;
        const auto& c = it->second;

        // Find bounding box to create a compact MaskView bitmap
        double min_x = c.xs[0], max_x = c.xs[0];
        double min_y = c.ys[0], max_y = c.ys[0];
        for (size_t i = 1; i < c.xs.size(); ++i) {
            if (c.xs[i] < min_x) min_x = c.xs[i];
            if (c.xs[i] > max_x) max_x = c.xs[i];
            if (c.ys[i] < min_y) min_y = c.ys[i];
            if (c.ys[i] > max_y) max_y = c.ys[i];
        }
        const int offset_x = -static_cast<int>(std::floor(min_x));
        const int offset_y = -static_cast<int>(std::floor(min_y));
        const int w = static_cast<int>(std::ceil(max_x)) + offset_x + 1;
        const int h = static_cast<int>(std::ceil(max_y)) + offset_y + 1;

        std::vector<uint8_t> bitmap(w * h, 0);
        for (size_t i = 0; i < c.xs.size(); ++i) {
            int bx = static_cast<int>(std::nearbyint(c.xs[i])) + offset_x;
            int by = static_cast<int>(std::nearbyint(c.ys[i])) + offset_y;
            if (bx >= 0 && bx < w && by >= 0 && by < h) {
                bitmap[by * w + bx] = 1;
            }
        }

        MaskView view;
        view.m = bitmap.data();
        view.w = w;
        view.h = h;
        view.stride = w;
        view.scale_x = static_cast<float>(c.scale_x);
        view.scale_y = static_cast<float>(c.scale_y);

        CrossSection cs_pts = mask_cross_section(c.xs.data(), c.ys.data(), c.xs.size(), 5, 8.0, c.scale_x, c.scale_y);
        CrossSection cs_view = mask_cross_section(view, 5, 8.0);
        CHECK_MSG(cs_view.ok == cs_pts.ok, name);
        CHECK_MSG(cs_view.angle_deg == cs_pts.angle_deg, name);
        CHECK_MSG(cs_view.conf == cs_pts.conf, name);
        CHECK_MSG(cs_view.width_px == cs_pts.width_px, name);

        Moments mo_pts = mask_moments(c.xs.data(), c.ys.data(), c.xs.size(), c.scale_x, c.scale_y);
        Moments mo_view = mask_moments(view);
        CHECK_MSG(mo_view.angle_deg == mo_pts.angle_deg, name);
        CHECK_MSG(mo_view.conf == mo_pts.conf, name);
    }
}

static void test_empty_and_null() {
    Moments mo0 = mask_moments(nullptr, nullptr, 0);
    CHECK(mo0.angle_deg == 0.0 && mo0.conf == 0.0);

    CrossSection cs0 = mask_cross_section(nullptr, nullptr, 0);
    CHECK(!cs0.ok && cs0.angle_deg == 0.0 && cs0.conf == 0.0 && cs0.width_px == 0.0);

    MaskView null_view{nullptr, 0, 0, 0, 1.0f, 1.0f};
    Moments mo_null = mask_moments(null_view);
    CHECK(mo_null.angle_deg == 0.0 && mo_null.conf == 0.0);

    CrossSection cs_null = mask_cross_section(null_view);
    CHECK(!cs_null.ok && cs_null.angle_deg == 0.0 && cs_null.conf == 0.0 && cs_null.width_px == 0.0);
}

int main() {
    std::printf("Running mask_geom tests...\n");
    auto cases = load_vectors();
    CHECK_MSG(!cases.empty(), "No test vector cases loaded");

    test_all_cases_execute_and_produce_valid_outputs(cases);
    test_rect_angles_and_widths(cases);
    test_stretched_rect_matches_unstretched(cases);
    test_disc_confidence_low(cases);
    test_square_geometry(cases);
    test_ring_and_l_shape_snap_centroid(cases);
    test_mask_view_equivalence(cases);
    test_empty_and_null();

    if (g_fail == 0) {
        std::printf("All mask_geom tests passed!\n");
        return 0;
    } else {
        std::printf("%d mask_geom tests failed!\n", g_fail);
        return 1;
    }
}
