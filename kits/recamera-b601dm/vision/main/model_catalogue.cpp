#include "model_catalogue.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "json.hpp"

namespace rv {

const ModelEntry* Catalogue::find(const std::string& name) const {
    for (const ModelEntry& m : models) {
        if (m.name == name) return &m;
    }
    return nullptr;
}

double legacy_ion_mb(const std::string& kind) {
    if (kind == "detect") return 8.5;
    if (kind == "segment") return 19.5;
    if (kind == "obb") return 8.0;
    if (kind == "depth") return 20.5;
    return 0;
}

bool is_yolo_kind(const std::string& kind) {
    return kind == "detect" || kind == "segment" || kind == "obb";
}

std::string file_stem(const std::string& path) {
    const std::string b = path.substr(path.find_last_of('/') + 1);
    return b.substr(0, b.find('.'));
}

bool depth_metric_of(const ModelEntry& m) {
    if (m.metric) return *m.metric;
    return file_stem(m.path).find("metric") != std::string::npos;
}

std::string register_startup(Catalogue& cat, const std::string& path, const std::string& kind,
                             std::optional<bool> metric) {
    for (ModelEntry& m : cat.models) {
        if (m.path != path) continue;
        if (kind == "depth" && !m.metric) m.metric = metric;
        return m.name;
    }
    std::string name = file_stem(path);
    if (cat.find(name) != nullptr) name += "-startup";
    ModelEntry e;
    e.name = name;
    e.path = path;
    e.kind = kind;
    e.ion_mb = legacy_ion_mb(kind);
    if (kind == "depth") e.metric = metric;
    cat.models.push_back(e);
    return name;
}

bool valid_model_name(const std::string& name) {
    if (name.empty() || name.size() > 64) return false;
    for (char c : name) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
                        c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

namespace {

std::string trim_cr(std::string s) {
    if (!s.empty() && s.back() == '\r') s.pop_back();
    return s;
}

std::string kind_of_role(const std::string& role) {
    if (role == "detector-detect") return "detect";
    if (role == "detector-segment" || role == "detector-segment-fallback") return "segment";
    if (role == "detector-obb") return "obb";
    if (role == "depth") return "depth";
    return "";
}

std::vector<std::string> paths_for_entry(const ManifestEntry& e) {
    if (e.ship == "os") {
        return {
            "/usr/share/supervisor/models/" + e.file,
            "/userdata/local/models/" + e.file,
            "/usr/share/rebot-teleop/models/" + e.file,
        };
    }
    return {
        "/userdata/local/models/" + e.file,
        "/userdata/rebot-teleop/models/" + e.file,
        "/usr/share/rebot-teleop/models/" + e.file,
    };
}

void append_role_paths(std::vector<std::string>& out, const std::vector<ManifestEntry>& manifest, const char* role) {
    for (const ManifestEntry& e : manifest) {
        if (e.role != role) continue;
        for (const std::string& p : paths_for_entry(e)) out.push_back(p);
    }
}

}  // namespace

std::vector<ManifestEntry> parse_manifest(const std::string& tsv_text) {
    std::vector<ManifestEntry> out;
    std::istringstream ss(tsv_text);
    std::string line;
    while (std::getline(ss, line)) {
        line = trim_cr(line);
        size_t first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') continue;
        std::vector<std::string> cols;
        size_t pos = 0;
        while (true) {
            const size_t tab = line.find('\t', pos);
            if (tab == std::string::npos) {
                cols.push_back(line.substr(pos));
                break;
            }
            cols.push_back(line.substr(pos, tab - pos));
            pos = tab + 1;
        }
        if (cols.size() < 7 || cols[0].empty() || cols[1].empty()) continue;
        ManifestEntry e;
        e.role = cols[0];
        e.file = cols[1];
        e.md5 = cols[2];
        e.size = std::strtoull(cols[3].c_str(), nullptr, 10);
        e.ion_mb = std::strtod(cols[4].c_str(), nullptr);
        if (!std::isfinite(e.ion_mb) || e.ion_mb < 0) e.ion_mb = 0;
        e.url = cols[5];
        e.ship = cols[6];
        out.push_back(std::move(e));
    }
    return out;
}

std::vector<ManifestEntry> builtin_manifest() {
    return {
        {"detector-segment", "yolo11n_seg_384x640_cv181x_int8_fuse.cvimodel", "5ad01b51727c5259abc1448f2bd0cec6",
         3333080, 13.1,
         "https://sensecraft-statics.seeed.cc/solution-app/rebot/models/yolo11n_seg_384x640_cv181x_int8_fuse-5ad01b51.cvimodel",
         "bundled"},
        {"detector-segment-fallback", "yolo11n_segment_cv181x_int8.cvimodel", "b3312af49a5d59a2533c973695a2267a",
         4043296, 19.5, "-", "os"},
        {"detector-detect", "yolo11n_detection_cv181x_int8.cvimodel", "7819bd8d3958f5184fb20cd8b0490525",
         3113208, 8.5, "-", "os"},
        {"depth", "zipdepth_metric_384x224_bf16_rgb.cvimodel", "d0a6ee396c12680188a1cb8d56cd4a94",
         12918008, 20.0,
         "https://sensecraft-statics.seeed.cc/solution-app/rebot/models/zipdepth_metric_384x224_bf16_rgb-d0a6ee39.cvimodel",
         "bundled"},
        {"detector-obb", "yolo11n_obb_cv181x_int8_fuse_qout.cvimodel", "d9607b7039dda5981fb62b957f678bdf",
         3187696, 8.0,
         "https://sensecraft-statics.seeed.cc/solution-app/rebot/models/yolo11n_obb_cv181x_int8_fuse-d9607b70.cvimodel",
         "optional"},
    };
}

std::vector<ManifestEntry> load_manifest_or_builtin(const std::string& path) {
    if (!path.empty()) {
        std::ifstream f(path);
        if (f) {
            std::ostringstream buf;
            buf << f.rdbuf();
            std::vector<ManifestEntry> parsed = parse_manifest(buf.str());
            if (!parsed.empty()) return parsed;
        }
    }
    return builtin_manifest();
}

std::vector<std::string> default_detector_candidates(const std::vector<ManifestEntry>& manifest) {
    std::vector<std::string> out;
    // seg384 first: resident segmentation gives masks (grasp angle, measure_multiview contours)
    // without a model swap and coexists with depth inside the 62.9 MB ION carveout.
    append_role_paths(out, manifest, "detector-segment");
    out.push_back("/userdata/rebot-teleop/models/yolo11n.cvimodel");
    append_role_paths(out, manifest, "detector-detect");
    out.push_back("/userdata/local/models/yolo11n_cv181x_int8.cvimodel");
    out.push_back("/userdata/rebot-teleop/models/yolo11n-obb.cvimodel");
    append_role_paths(out, manifest, "detector-obb");
    out.push_back("/userdata/local/models/yolo11n_obb_cv181x_int8.cvimodel");
    out.push_back("/userdata/local/models/yolo26n_obb_cv181x_int8.cvimodel");
    append_role_paths(out, manifest, "detector-segment-fallback");
    return out;
}

std::vector<std::string> default_depth_candidates(const std::vector<ManifestEntry>& manifest) {
    std::vector<std::string> out;
    append_role_paths(out, manifest, "depth");
    out.push_back("/userdata/rebot-teleop/models/zipdepth_base_384x224_bf16_rgb.cvimodel");
    out.push_back("/userdata/local/models/zipdepth_base_384x224_bf16_rgb.cvimodel");
    return out;
}

void add_default_models(Catalogue& cat, const std::function<bool(const std::string&)>& exists,
                        const std::vector<ManifestEntry>& manifest) {
    static const std::vector<std::string> kDota = {
        "plane", "ship", "storage tank", "baseball diamond", "tennis court", "basketball court",
        "ground track field", "harbor", "bridge", "large vehicle", "small vehicle", "helicopter",
        "roundabout", "soccer ball field", "swimming pool"};
    for (const ManifestEntry& m : manifest) {
        const std::string kind = kind_of_role(m.role);
        if (kind.empty()) continue;
        for (const std::string& p : paths_for_entry(m)) {
            if (!exists(p)) continue;
            ModelEntry e;
            e.name = file_stem(p);
            e.path = p;
            e.kind = kind;
            e.ion_mb = m.ion_mb;
            if (e.kind == "depth") e.metric = true;
            if (e.kind == "obb") e.classes = kDota;
            if (cat.find(e.name) == nullptr) cat.models.push_back(e);
            break;
        }
    }
}

void add_default_models(Catalogue& cat, const std::function<bool(const std::string&)>& exists) {
    add_default_models(cat, exists, load_manifest_or_builtin());
}

void parse_catalogue(const std::string& text, Catalogue& out, std::vector<std::string>& warnings) {
    const nlohmann::json config = nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (!config.is_object()) return;
    auto mb = [&](const char* key, double& dst) {
        auto it = config.find(key);
        if (it == config.end()) return;
        if (it->is_number() && std::isfinite(it->get<double>()) && it->get<double>() >= 0) dst = it->get<double>();
        else warnings.push_back(std::string(key) + " must be a number >= 0");
    };
    mb("ion_base_mb", out.ion.base_mb);
    mb("ion_total_mb", out.ion.total_mb);
    mb("ion_reserve_mb", out.ion.reserve_mb);
    auto models = config.find("models");
    if (models == config.end()) return;
    out.configured = true;
    if (!models->is_object()) {
        warnings.push_back("models must be an object of name -> {path, kind, ion_mb}");
        return;
    }
    for (auto it = models->begin(); it != models->end(); ++it) {
        const std::string& name = it.key();
        const nlohmann::json& v = it.value();
        auto bad = [&](const char* why) { warnings.push_back("models." + name + ": " + why); };
        if (!valid_model_name(name)) { bad("name must be 1..64 of letters, digits, '_', '-', '.'"); continue; }
        if (!v.is_object()) { bad("must be an object"); continue; }
        auto path = v.find("path");
        auto kind = v.find("kind");
        auto ion = v.find("ion_mb");
        if (path == v.end() || !path->is_string() || path->get<std::string>().empty()) { bad("path must be a string"); continue; }
        if (kind == v.end() || !kind->is_string() ||
            !(is_yolo_kind(kind->get<std::string>()) || kind->get<std::string>() == "depth")) {
            bad("kind must be detect, segment, obb or depth");
            continue;
        }
        if (ion == v.end() || !ion->is_number() || !std::isfinite(ion->get<double>()) || ion->get<double>() < 0) {
            bad("ion_mb must be a number >= 0 (measured for this model file)");
            continue;
        }
        ModelEntry e;
        e.name = name;
        e.path = path->get<std::string>();
        e.kind = kind->get<std::string>();
        e.ion_mb = ion->get<double>();
        if (auto metric = v.find("metric"); metric != v.end()) {
            if (!metric->is_boolean() || e.kind != "depth") {
                bad("metric must be true or false, on a depth entry");
                continue;
            }
            e.metric = metric->get<bool>();
        }
        if (auto classes = v.find("classes"); classes != v.end()) {
            bool ok = is_yolo_kind(e.kind) && classes->is_array() && !classes->empty() && classes->size() <= 1000;
            for (const nlohmann::json& c : ok ? *classes : nlohmann::json::array()) {
                if (!c.is_string() || c.get<std::string>().empty()) ok = false;
                else e.classes.push_back(c.get<std::string>());
            }
            if (!ok) {
                bad("classes must be an array of 1..1000 non-empty strings, on a detect, segment or obb entry");
                continue;
            }
        }
        out.models.push_back(e);
    }
}

BudgetCheck check_budget(const Catalogue& cat, const std::string& detector, const std::string& depth) {
    double need = cat.ion.base_mb + cat.ion.reserve_mb;
    for (const std::string* n : {&detector, &depth}) {
        if (const ModelEntry* m = n->empty() ? nullptr : cat.find(*n)) need += m->ion_mb;
    }
    return {need <= cat.ion.total_mb + 1e-9, need, cat.ion.total_mb};
}

bool parse_swap_request(const std::string& text, const Catalogue& cat, SwapRequest& out, std::string& err) {
    const nlohmann::json body = nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (!body.is_object()) {
        err = "body must be a JSON object";
        return false;
    }
    out = SwapRequest{};
    for (auto it = body.begin(); it != body.end(); ++it) {
        const bool detector = it.key() == "detector";
        if (!detector && it.key() != "depth") {
            err = "unknown key \"" + it.key() + "\" (detector, depth)";
            return false;
        }
        SlotChange& slot = detector ? out.detector : out.depth;
        slot.change = true;
        if (it->is_null()) continue;
        if (!it->is_string()) {
            err = "\"" + it.key() + "\" must be a model name or null";
            return false;
        }
        slot.name = it->get<std::string>();
        const ModelEntry* m = cat.find(slot.name);
        if (m == nullptr) {
            err = "no model named \"" + slot.name + "\"";
            return false;
        }
        if (detector != is_yolo_kind(m->kind)) {
            err = "model \"" + slot.name + "\" (kind " + m->kind + ") cannot go in the " + it.key() + " slot";
            return false;
        }
    }
    if (!out.any()) {
        err = "body needs \"detector\" or \"depth\"";
        return false;
    }
    return true;
}

int first_loadable(const std::vector<std::string>& candidates, const std::function<bool(const std::string&)>& exists,
                   const std::function<bool(const std::string&)>& load, std::vector<std::string>* failed) {
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (!exists(candidates[i])) continue;
        if (load(candidates[i])) return static_cast<int>(i);
        if (failed) failed->push_back(candidates[i]);
    }
    return -1;
}

}  // namespace rv
