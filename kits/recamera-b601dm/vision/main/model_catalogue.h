// Model catalogue, ION budget and model swap requests (POST /api/models). Portable: no SDK,
// host-tested (tests/model_catalogue_test.cpp). JSON comes in as text: main.cpp sees the
// sscma-micro copy of nlohmann/json, a different type from components/json used here.
#ifndef REBOT_VISION_MODEL_CATALOGUE_H_
#define REBOT_VISION_MODEL_CATALOGUE_H_

#include <optional>
#include <string>
#include <vector>

namespace rv {

/** One loadable model: a YOLO model (kind detect / segment / obb) or a depth model (kind depth). */
struct ModelEntry {
    std::string name;
    std::string path;
    std::string kind;
    double ion_mb = 0;  ///< ION the model takes when loaded (measured for that file)
    /// Depth models: metric (true) or relative (false) depth; unset = the file name decides
    /// (see depth_metric_of). vision.json `"metric"`, or --depth-metric for the startup model.
    std::optional<bool> metric;
};

/**
 * Static ION budget: base (camera, VPSS, VENC) + the resident models + reserve <= total. Valid only
 * from a clean ION baseline (no other ION user on the unit); it is not a fail-safe allocator.
 */
struct IonBudget {
    double base_mb = 21.5;
    double total_mb = 62.9;
    double reserve_mb = 8.0;
};

struct Catalogue {
    std::vector<ModelEntry> models;
    IonBudget ion;
    const ModelEntry* find(const std::string& name) const;
};

/** ION of a legacy `model` / `depth_model` entry by kind (measured + ~5 %); 0 for an unknown kind. */
double legacy_ion_mb(const std::string& kind);

/** Kind of a YOLO slot entry (detect / segment / obb). */
bool is_yolo_kind(const std::string& kind);

/** File stem: the file name up to its first '.'. */
std::string file_stem(const std::string& path);

/**
 * Whether a depth entry is a metric model: the entry's `metric` flag, else "metric" in the file
 * stem (zipdepth_metric_* metric, zipdepth_base_* relative).
 */
bool depth_metric_of(const ModelEntry& m);

/**
 * Catalogue name of a startup model (`model` / `depth_model`, else a default file): the entry
 * with this path, else a new legacy entry (file stem, `ion_mb` from the kind table). `metric`
 * (--depth-metric on|off; unset for auto) applies to this startup depth model only: it is
 * stored on the entry unless the entry states its own `metric`.
 */
std::string register_startup(Catalogue& cat, const std::string& path, const std::string& kind,
                             std::optional<bool> metric = std::nullopt);

/** Catalogue names: 1..64 of letters, digits, '_', '-', '.'. */
bool valid_model_name(const std::string& name);

/**
 * vision.json (`config`, a JSON object) `models` ({"<name>": {"path", "kind", "ion_mb", "metric"?}})
 * and `ion_base_mb` / `ion_total_mb` / `ion_reserve_mb` into `out`. Invalid entries are skipped
 * with a message in `warnings` (ion_mb must be a finite number >= 0; `metric`, optional, a
 * boolean on a depth entry).
 */
void parse_catalogue(const std::string& config, Catalogue& out, std::vector<std::string>& warnings);

struct BudgetCheck {
    bool ok = true;
    double need_mb = 0;   ///< base + resident models + reserve
    double limit_mb = 0;  ///< total
};

/** Budget of a resident set (names of the catalogue; empty = slot empty, unknown names count 0). */
BudgetCheck check_budget(const Catalogue& cat, const std::string& detector, const std::string& depth);

/** One slot of a swap request: `change` false = keep; `name` empty = unload. */
struct SlotChange {
    bool change = false;
    std::string name;
};

struct SwapRequest {
    SlotChange detector;
    SlotChange depth;
    bool any() const noexcept { return detector.change || depth.change; }
};

/**
 * `{"detector": "<name>"|null, "depth": "<name>"|null}` (absent = keep). False with `err` when the
 * body is not such an object, a name is not in the catalogue or has the wrong kind for its slot.
 */
bool parse_swap_request(const std::string& body, const Catalogue& cat, SwapRequest& out, std::string& err);

/** The resident name of a slot after the request: the requested one, else `current`. */
inline const std::string& after_swap(const SlotChange& s, const std::string& current) {
    return s.change ? s.name : current;
}

}  // namespace rv

#endif  // REBOT_VISION_MODEL_CATALOGUE_H_
