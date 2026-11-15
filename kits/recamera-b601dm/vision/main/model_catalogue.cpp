#include "model_catalogue.h"

#include <cmath>

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

}  // namespace rv
