// Model catalogue parsing, the ION budget rule and swap request validation.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "model_catalogue.h"

using namespace rv;

static int g_fail = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++g_fail;                                                \
        }                                                            \
    } while (0)

/** The four measured models (STEP3B-MEASURE: detect 8.04, seg 18.71, obb ~7.6, depth 19.98 MB + ~5 %). */
static Catalogue measured() {
    Catalogue c;
    std::vector<std::string> w;
    parse_catalogue(std::string(R"({"models": {
        "det": {"path": "/m/yolo11n_detection_cv181x_int8.cvimodel", "kind": "detect", "ion_mb": 8.5},
        "seg": {"path": "/m/yolo11n_segment_cv181x_int8.cvimodel", "kind": "segment", "ion_mb": 19.5},
        "obb": {"path": "/m/yolo11n_obb_cv181x_int8_fuse_qout.cvimodel", "kind": "obb", "ion_mb": 8.0},
        "depth": {"path": "/m/zipdepth_metric_384x224_bf16_rgb.cvimodel", "kind": "depth", "ion_mb": 20.5}}})"),
                     c, w);
    CHECK(w.empty() && c.models.size() == 4);
    return c;
}

static void test_budget_table() {
    const Catalogue c = measured();
    struct Row { const char* det; const char* depth; bool ok; double need; };
    const Row rows[] = {
        {"det", "depth", true, 58.5}, {"seg", "depth", false, 69.5}, {"obb", "depth", true, 58.0},
        {"seg", "", true, 49.0},      {"det", "", true, 38.0},       {"", "", true, 29.5},
        {"", "depth", true, 50.0},
    };
    for (const Row& r : rows) {
        const BudgetCheck b = check_budget(c, r.det, r.depth);
        CHECK(b.ok == r.ok);
        CHECK(std::fabs(b.need_mb - r.need) < 1e-9);
        CHECK(b.limit_mb == 62.9);
    }
    // unknown names count 0 (the catalogue lists every resident model)
    CHECK(std::fabs(check_budget(c, "nope", "").need_mb - 29.5) < 1e-9);
    // a smaller reserve admits seg + depth
    Catalogue tight = c;
    tight.ion.reserve_mb = 1.0;
    CHECK(check_budget(tight, "seg", "depth").ok);
}

static void test_catalogue_parsing() {
    Catalogue c;
    std::vector<std::string> w;
    parse_catalogue(std::string(R"({"model": "/m/x.cvimodel"})"), c, w);  // legacy keys only
    CHECK(c.models.empty() && w.empty());
    CHECK(c.ion.base_mb == 21.5 && c.ion.total_mb == 62.9 && c.ion.reserve_mb == 8.0);

    Catalogue d;
    parse_catalogue(std::string(R"({"ion_base_mb": 20, "ion_total_mb": 60, "ion_reserve_mb": 4, "models": {
        "ok": {"path": "/a", "kind": "obb", "ion_mb": 0},
        "no_ion": {"path": "/b", "kind": "detect"},
        "neg": {"path": "/b", "kind": "detect", "ion_mb": -1},
        "str_ion": {"path": "/b", "kind": "detect", "ion_mb": "8"},
        "bad kind": {"path": "/b", "kind": "detect", "ion_mb": 1},
        "kind": {"path": "/b", "kind": "pose", "ion_mb": 1},
        "nopath": {"kind": "depth", "ion_mb": 1},
        "notobj": 3}})"),
                     d, w);
    CHECK(d.models.size() == 1 && d.models[0].name == "ok" && d.models[0].kind == "obb");
    CHECK(w.size() == 7);
    CHECK(d.ion.base_mb == 20 && d.ion.total_mb == 60 && d.ion.reserve_mb == 4);
    CHECK(d.find("ok") != nullptr && d.find("no_ion") == nullptr);

    w.clear();
    Catalogue e;
    parse_catalogue(std::string(R"({"ion_total_mb": -3, "models": []})"), e, w);
    CHECK(w.size() == 2 && e.ion.total_mb == 62.9);

    CHECK(legacy_ion_mb("detect") == 8.5 && legacy_ion_mb("segment") == 19.5 && legacy_ion_mb("obb") == 8.0 &&
          legacy_ion_mb("depth") == 20.5 && legacy_ion_mb("other") == 0);
    CHECK(valid_model_name("yolo11n_detection_cv181x_int8") && valid_model_name("a.b-c"));
    CHECK(!valid_model_name("") && !valid_model_name("a b") && !valid_model_name(std::string(65, 'a')));
}

static void test_swap_request() {
    const Catalogue c = measured();
    SwapRequest r;
    std::string err;
    CHECK(parse_swap_request(std::string(R"({"detector": "seg"})"), c, r, err));
    CHECK(r.detector.change && r.detector.name == "seg" && !r.depth.change);
    CHECK(after_swap(r.detector, "det") == "seg" && after_swap(r.depth, "depth") == "depth");
    CHECK(parse_swap_request(std::string(R"({"detector": "obb", "depth": null})"), c, r, err));
    CHECK(r.depth.change && r.depth.name.empty() && after_swap(r.depth, "depth").empty());

    struct Bad { const char* body; const char* err; };
    const Bad bad[] = {
        {R"({})", "body needs \"detector\" or \"depth\""},
        {R"([])", "body must be a JSON object"},
        {R"({"detector": "x"})", "no model named \"x\""},
        {R"({"detector": "depth"})", "model \"depth\" (kind depth) cannot go in the detector slot"},
        {R"({"depth": "seg"})", "model \"seg\" (kind segment) cannot go in the depth slot"},
        {R"({"detector": 3})", "\"detector\" must be a model name or null"},
        {R"({"model": "seg"})", "unknown key \"model\" (detector, depth)"},
    };
    for (const Bad& b : bad) {
        err.clear();
        CHECK(!parse_swap_request(std::string(b.body), c, r, err));
        CHECK(err == b.err);
        if (err != b.err) std::printf("  got: %s\n", err.c_str());
    }
}

int main() {
    test_budget_table();
    test_catalogue_parsing();
    test_swap_request();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("model_catalogue: all checks passed\n");
    return 0;
}
