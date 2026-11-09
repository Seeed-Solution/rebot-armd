#include <cstdio>
#include <string>
#include <vector>

#include "marker_filter.h"

using namespace aruco;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

static const double kSquare40[4][2] = {{0, 40}, {40, 40}, {40, 0}, {0, 0}};

static void test_side() {
    CHECK(std::abs(min_quad_side(kSquare40) - 40.0) < 1e-9);
    const double thin[4][2] = {{0, 0}, {100, 0}, {100, 5}, {0, 5}};
    CHECK(std::abs(min_quad_side(thin) - 5.0) < 1e-9);
    // closing edge p[3]->p[0] must be considered
    const double closing[4][2] = {{0, 0}, {50, 0}, {50, 50}, {0, 3}};
    CHECK(std::abs(min_quad_side(closing) - 3.0) < 1e-9);
}

static void test_classify() {
    MarkerFilter f;
    f.min_margin = 30.0f;
    f.min_side_px = 12.0f;
    CHECK(classify_marker(f, 1, 55.0f, kSquare40) == FilterVerdict::Accept);
    CHECK(classify_marker(f, 1, 30.0f, kSquare40) == FilterVerdict::Accept);  // boundary: not below
    CHECK(classify_marker(f, 1, 29.9f, kSquare40) == FilterVerdict::LowMargin);

    const double small[4][2] = {{0, 11}, {11, 11}, {11, 0}, {0, 0}};
    CHECK(classify_marker(f, 1, 90.0f, small) == FilterVerdict::SmallSide);
    const double edge12[4][2] = {{0, 12}, {12, 12}, {12, 0}, {0, 0}};
    CHECK(classify_marker(f, 1, 90.0f, edge12) == FilterVerdict::Accept);

    f.ids = {3, 7};
    CHECK(classify_marker(f, 7, 90.0f, kSquare40) == FilterVerdict::Accept);
    CHECK(classify_marker(f, 4, 90.0f, kSquare40) == FilterVerdict::IdNotAllowed);

    // disabled thresholds accept everything
    MarkerFilter off;
    off.min_margin = 0.0f;
    off.min_side_px = 0.0f;
    CHECK(classify_marker(off, 9, 0.0f, small) == FilterVerdict::Accept);
}

static void test_parse() {
    std::vector<int> v;
    std::string err;
    CHECK(parse_id_list("", v, &err) && v.empty());
    CHECK(parse_id_list("   ", v, &err) && v.empty());
    CHECK(parse_id_list("5", v, &err) && v == std::vector<int>({5}));
    CHECK(parse_id_list("3,7, 12 ", v, &err) && v == std::vector<int>({3, 7, 12}));
    CHECK(parse_id_list(" 0 ,\t49", v, &err) && v == std::vector<int>({0, 49}));
    CHECK(format_id_list({}) == "all");
    CHECK(format_id_list({3, 7}) == "3,7");

    const char* bad[] = {"a", "1,a", "1,,2", ",1", "1,", "-1", "+1", "1.5", "1 2", "70000", "99999999999999999999", ","};
    for (const char* b : bad) {
        err.clear();
        v = {42};
        const bool ok = parse_id_list(b, v, &err);
        if (ok || !v.empty() || err.empty()) std::printf("  unexpected result for '%s'\n", b);
        CHECK(!ok);
        CHECK(v.empty());
        CHECK(!err.empty());
    }
    CHECK(parse_id_list("1,2", v, nullptr));
    CHECK(!parse_id_list("x", v, nullptr));
}

int main() {
    test_side();
    test_classify();
    test_parse();
    if (g_fail == 0) {
        std::printf("ALL MARKER FILTER TESTS PASSED\n");
        return 0;
    }
    std::printf("%d TESTS FAILED\n", g_fail);
    return 1;
}
