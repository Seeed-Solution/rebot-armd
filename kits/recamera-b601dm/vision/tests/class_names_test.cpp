// Class names per model: catalogue `classes`, sidecar `.names`, COCO for 80-class detect /
// segment models, else "class_<id>"; and the class count of a detection head from its shapes.
#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "class_names.h"
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

static void test_parse_and_paths() {
    CHECK(parse_names_text("plane\r\nship\n\n  storage tank \nlast") ==
          std::vector<std::string>({"plane", "ship", "storage tank", "last"}));
    CHECK(parse_names_text("").empty() && parse_names_text("\n \n").empty());
    CHECK(names_sidecar_path("/m/yolo11n_obb_cv181x_int8_fuse_qout.cvimodel") == "/m/yolo11n_obb_cv181x_int8_fuse_qout.names");
    CHECK(names_sidecar_path("/m.d/model") == "/m.d/model.names");
    CHECK(names_sidecar_path("/m/.hidden") == "/m/.hidden.names");
}

static void test_fallbacks() {
    const std::string none = "/nonexistent-dir/model.cvimodel";
    // detect with 80 classes: COCO
    ClassNames det = resolve_class_names({}, none, "detect", 80);
    CHECK(det.coco && det.source == "coco" && det.name(0) == "person" && det.name(41) == "cup");
    CHECK(det.name(80) == "class_80" && det.name(-1) == "class_-1");
    // segment (yolo11n-seg, COCO): COCO as well
    CHECK(resolve_class_names({}, none, "segment", 80).name(39) == "bottle");
    // a 15-class DOTA OBB model: class 0 is not "person"
    ClassNames obb = resolve_class_names({}, none, "obb", 15);
    CHECK(!obb.coco && obb.source == "class_<id>" && obb.name(0) == "class_0" && obb.name(14) == "class_14");
    // an 80-class OBB model, or a detect model of another or unknown class count: ids
    CHECK(resolve_class_names({}, none, "obb", 80).name(0) == "class_0");
    CHECK(resolve_class_names({}, none, "detect", 3).name(0) == "class_0");
    CHECK(resolve_class_names({}, none, "detect", 0).name(0) == "class_0");
}

static void test_catalogue_and_sidecar() {
    char dir[] = "/tmp/rv_class_names_XXXXXX";
    CHECK(mkdtemp(dir) != nullptr);
    const std::string model = std::string(dir) + "/dota_obb.cvimodel";
    {
        std::ofstream f(std::string(dir) + "/dota_obb.names");
        f << "plane\nship\nstorage tank\n";
    }
    // the sidecar file
    ClassNames side = resolve_class_names({}, model, "obb", 3);
    CHECK(side.source == std::string(dir) + "/dota_obb.names" && side.warning.empty());
    CHECK(side.name(0) == "plane" && side.name(2) == "storage tank" && side.name(3) == "class_3");
    // a count that differs from the model: used, with a warning
    ClassNames off = resolve_class_names({}, model, "obb", 15);
    CHECK(off.name(1) == "ship" && off.warning == "3 class names for a model with 15 classes" && off.name(5) == "class_5");
    // the catalogue's classes win over the sidecar; a sidecar also replaces COCO for a detect model
    ClassNames cat = resolve_class_names({"bolt", "nut"}, model, "obb", 2);
    CHECK(cat.source == "catalogue" && cat.name(0) == "bolt" && cat.name(1) == "nut");
    CHECK(resolve_class_names({}, model, "detect", 80).name(0) == "plane");
    std::remove((std::string(dir) + "/dota_obb.names").c_str());
    rmdir(dir);
}

static void test_detect_num_classes() {
    // YOLO11 detect (6 outputs, NCHW): box [1,64,H,W] + cls [1,80,H,W]
    CHECK(detect_num_classes({{1, 64, 80, 80}, {1, 80, 80, 80}, {1, 64, 40, 40}, {1, 80, 40, 40}, {1, 64, 20, 20}, {1, 80, 20, 20}}) == 80);
    // YOLO26 (box [1,4,H,W]) with 15 classes
    CHECK(detect_num_classes({{1, 4, 80, 80}, {1, 15, 80, 80}, {1, 4, 40, 40}, {1, 15, 40, 40}, {1, 4, 20, 20}, {1, 15, 20, 20}}) == 15);
    // YOLOv8 [1,84,8400], YOLOv5 [1,25200,85]: 80; another single output: unknown
    CHECK(detect_num_classes({{1, 84, 8400}}) == 80 && detect_num_classes({{1, 25200, 85}}) == 80);
    CHECK(detect_num_classes({{1, 8, 8400}}) == 0);
    // disagreeing heads or other ranks: unknown
    CHECK(detect_num_classes({{1, 64, 80, 80}, {1, 80, 80, 80}, {1, 30, 40, 40}}) == 0);
    CHECK(detect_num_classes({}) == 0 && detect_num_classes({{1, 80}, {1, 80}}) == 0);
}

static void test_catalogue_classes() {
    Catalogue c;
    std::vector<std::string> w;
    parse_catalogue(std::string(R"({"models": {
        "dota": {"path": "/m/obb.cvimodel", "kind": "obb", "ion_mb": 8.0, "classes": ["plane", "ship"]},
        "det": {"path": "/m/det.cvimodel", "kind": "detect", "ion_mb": 8.5},
        "empty": {"path": "/m/e.cvimodel", "kind": "detect", "ion_mb": 8.5, "classes": []},
        "num": {"path": "/m/n.cvimodel", "kind": "detect", "ion_mb": 8.5, "classes": ["a", 3]},
        "blank": {"path": "/m/b.cvimodel", "kind": "segment", "ion_mb": 8.5, "classes": ["a", ""]},
        "depth": {"path": "/m/d.cvimodel", "kind": "depth", "ion_mb": 20.5, "classes": ["x"]}}})"),
                    c, w);
    CHECK(c.models.size() == 2 && w.size() == 4);
    CHECK(c.find("dota")->classes == std::vector<std::string>({"plane", "ship"}) && c.find("det")->classes.empty());
}

int main() {
    test_parse_and_paths();
    test_fallbacks();
    test_catalogue_and_sidecar();
    test_detect_num_classes();
    test_catalogue_classes();
    if (g_fail) {
        std::printf("%d check(s) failed\n", g_fail);
        return 1;
    }
    std::printf("class_names: all checks passed\n");
    return 0;
}
