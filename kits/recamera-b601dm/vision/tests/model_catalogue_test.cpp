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

/** Metric depth per entry: the entry's flag, then --depth-metric for the startup model only, then the name. */
static void test_depth_metric() {
    Catalogue c;
    std::vector<std::string> w;
    parse_catalogue(std::string(R"({"models": {
        "base": {"path": "/m/zipdepth_base_384x224_bf16_rgb.cvimodel", "kind": "depth", "ion_mb": 20.5},
        "metric": {"path": "/m/zipdepth_metric_384x224_bf16_rgb.cvimodel", "kind": "depth", "ion_mb": 20.5},
        "flagged": {"path": "/m/my_depth.cvimodel", "kind": "depth", "ion_mb": 20.5, "metric": true},
        "unflagged": {"path": "/m/zipdepth_metric_v2.cvimodel", "kind": "depth", "ion_mb": 20.5, "metric": false},
        "str": {"path": "/m/d.cvimodel", "kind": "depth", "ion_mb": 20.5, "metric": "yes"},
        "det": {"path": "/m/det.cvimodel", "kind": "detect", "ion_mb": 8.5, "metric": true}}})"),
                     c, w);
    CHECK(c.models.size() == 4 && w.size() == 2);
    CHECK(!c.find("base")->metric && !depth_metric_of(*c.find("base")));
    CHECK(!c.find("metric")->metric && depth_metric_of(*c.find("metric")));
    CHECK(depth_metric_of(*c.find("flagged")) && !depth_metric_of(*c.find("unflagged")));
    CHECK(file_stem("/a.b/zipdepth_metric_384x224.bf16.cvimodel") == "zipdepth_metric_384x224");

    // --depth-metric on for a startup model in the catalogue without a flag: stored on that entry
    // only; a relative model swapped in later still reports metric false
    Catalogue on = c;
    CHECK(register_startup(on, "/m/my_depth.cvimodel", "depth", false) == "flagged");
    CHECK(depth_metric_of(*on.find("flagged")));  // the entry's flag wins over --depth-metric off
    on = c;
    CHECK(register_startup(on, "/m/zipdepth_metric_384x224_bf16_rgb.cvimodel", "depth", false) == "metric");
    CHECK(!depth_metric_of(*on.find("metric")) && !depth_metric_of(*on.find("base")));
    on = c;
    CHECK(register_startup(on, "/m/zipdepth_base_384x224_bf16_rgb.cvimodel", "depth", true) == "base");
    CHECK(depth_metric_of(*on.find("base")) && !on.find("metric")->metric);
    // auto (unset): the file name
    on = c;
    register_startup(on, "/m/zipdepth_base_384x224_bf16_rgb.cvimodel", "depth", std::nullopt);
    CHECK(!depth_metric_of(*on.find("base")));

    // a startup model outside the catalogue: a legacy entry with the flag, named from the file stem
    Catalogue legacy;
    CHECK(register_startup(legacy, "/x/zipdepth_base_384x224_bf16_rgb.cvimodel", "depth", true) ==
          "zipdepth_base_384x224_bf16_rgb");
    CHECK(legacy.models.size() == 1 && legacy.models[0].ion_mb == 20.5 && depth_metric_of(legacy.models[0]));
    CHECK(register_startup(legacy, "/y/zipdepth_base_384x224_bf16_rgb.cvimodel", "depth") ==
          "zipdepth_base_384x224_bf16_rgb-startup");
    CHECK(!depth_metric_of(legacy.models[1]));
    // YOLO startup models carry no metric flag
    CHECK(register_startup(legacy, "/x/det.cvimodel", "detect", true) == "det" && !legacy.find("det")->metric);
}

static void test_default_models() {
    // only files that exist; the first existing path of a model; names are file stems
    const std::vector<std::string> present = {
        "/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel",
        "/userdata/rebot-teleop/models/yolo11n_seg_384x640_cv181x_int8_fuse.cvimodel",
        "/userdata/local/models/yolo11n_obb_cv181x_int8_fuse_qout.cvimodel",
        "/userdata/local/models/zipdepth_metric_384x224_bf16_rgb.cvimodel",
        "/userdata/rebot-teleop/models/zipdepth_metric_384x224_bf16_rgb.cvimodel"};
    auto exists = [&](const std::string& p) {
        for (const std::string& q : present) if (q == p) return true;
        return false;
    };
    Catalogue c;
    add_default_models(c, exists);
    CHECK(c.models.size() == 4);
    CHECK(c.find("yolo11n_segment_cv181x_int8") == nullptr);
    const ModelEntry* det = c.find("yolo11n_detection_cv181x_int8");
    CHECK(det && det->kind == "detect" && det->ion_mb == 8.5 && det->classes.empty());
    const ModelEntry* s384 = c.find("yolo11n_seg_384x640_cv181x_int8_fuse");
    CHECK(s384 && s384->kind == "segment" && s384->ion_mb == 13.1 && s384->path == present[1]);
    const ModelEntry* obb = c.find("yolo11n_obb_cv181x_int8_fuse_qout");
    CHECK(obb && obb->kind == "obb" && obb->classes.size() == 15 && obb->classes[0] == "plane");
    const ModelEntry* depth = c.find("zipdepth_metric_384x224_bf16_rgb");
    CHECK(depth && depth->path == present[3] && depth->metric && *depth->metric && depth->ion_mb == 20.0);
    // seg384 + depth fits the default budget (21.5 + 13.1 + 20.0 + 8 = 62.6 <= 62.9)
    CHECK(check_budget(c, s384->name, depth->name).ok);
    // a startup model at a catalogued path keeps the entry's name
    CHECK(register_startup(c, present[0], "detect") == det->name && c.models.size() == 4);
    // vision.json `models` (even an empty object) turns the built-in list off; no key keeps it on
    Catalogue a, b;
    std::vector<std::string> w;
    parse_catalogue(R"({"models": {}})", a, w);
    parse_catalogue(R"({"fps": 5})", b, w);
    CHECK(a.configured && !b.configured);
}

static void test_first_loadable() {
    const std::vector<std::string> c = {"seg384", "missing", "detect", "obb"};
    std::vector<std::string> tried, failed;
    auto exists = [](const std::string& p) { return p != "missing"; };
    // the first existing candidate is corrupt: the next one that loads wins
    int k = first_loadable(c, exists, [&](const std::string& p) { tried.push_back(p); return p != "seg384"; }, &failed);
    CHECK(k == 2);
    CHECK((tried == std::vector<std::string>{"seg384", "detect"}));
    CHECK((failed == std::vector<std::string>{"seg384"}));
    // the first loads: nothing else tried
    tried.clear();
    CHECK(first_loadable(c, exists, [&](const std::string& p) { tried.push_back(p); return true; }) == 0 && tried.size() == 1);
    // none loads / none exists
    failed.clear();
    CHECK(first_loadable(c, exists, [](const std::string&) { return false; }, &failed) == -1 && failed.size() == 3);
    CHECK(first_loadable(c, [](const std::string&) { return false; }, [](const std::string&) { return true; }) == -1);
}

int main() {
    test_first_loadable();
    test_default_models();
    test_budget_table();
    test_catalogue_parsing();
    test_swap_request();
    test_depth_metric();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("model_catalogue: all checks passed\n");
    return 0;
}
