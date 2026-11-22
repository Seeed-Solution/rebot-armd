// rebot-vision: the model swap of the frame thread (POST /api/models) as one transaction.
// Portable (no SDK): the loaders are passed in, so host tests can inject failures.
#ifndef REBOT_VISION_MODEL_SWAP_H_
#define REBOT_VISION_MODEL_SWAP_H_

#include <string>
#include <utility>

#include "model_catalogue.h"
#include "vision_control.h"

namespace rv {

/** Metadata of one slot, copied from its catalogue entry before the model is installed. */
struct SlotMeta {
    std::string name;
    std::string path;
    std::string kind;  ///< reserved: the kind assigned after the load does not allocate
    bool metric = false;
};

/** Longest kind name a detector reports (detect / segment / obb). */
constexpr size_t kSlotKindCapacity = 16;

inline SlotMeta prepare_slot(const ModelEntry& m) {
    SlotMeta s;
    s.name = m.name;
    s.path = m.path;
    s.kind.reserve(kSlotKindCapacity);
    s.metric = depth_metric_of(m);
    return s;
}

inline void clear_detector_meta(ModelSet& s) noexcept {
    s.yolo_available = false;
    s.model_name.clear();
    s.model_path.clear();
    s.model_kind.clear();
    s.mask = false;
}

inline void clear_depth_meta(ModelSet& s) noexcept {
    s.depth_available = false;
    s.depth_model_name.clear();
    s.depth_model_path.clear();
    s.depth_metric = false;
}

/**
 * Run the taken swap `s` (after VisionControl::take_swap) and end it with finish_swap or, on any
 * exception, fail_swap; it never leaves the swap Running. Order: prepare every allocating copy
 * (slot metadata, the error text), unload the changed slots, load, then describe each loaded slot
 * with non-throwing moves. On an exception the changed slots are unloaded again, so no model
 * holds ION without its name in the resident set (the ION budget of later swaps counts it).
 *
 * `load_detector(entry, kind, mask)` installs the detector and returns true (setting `kind` to a
 * static name of at most kSlotKindCapacity characters and `mask`), false on a load failure;
 * `load_depth(entry)` likewise. `unload_detector()` / `unload_depth()` must not throw.
 * `resident` is the frame thread's resident set. `failed_out` gets the failed slots ("detector",
 * "depth") or "exception" when the swap ended by an exception. Returns the new model generation.
 */
template <class LoadDetector, class LoadDepth, class UnloadDetector, class UnloadDepth>
uint64_t run_model_swap(VisionControl& ctl, const SwapRequest& s, ModelSet& resident, LoadDetector&& load_detector,
                        LoadDepth&& load_depth, UnloadDetector&& unload_detector, UnloadDepth&& unload_depth,
                        std::string* failed_out = nullptr) noexcept {
    bool det_touched = false;
    bool dep_touched = false;
    try {
        const ModelEntry* dm = s.detector.change ? ctl.catalogue().find(s.detector.name) : nullptr;
        const ModelEntry* pm = s.depth.change ? ctl.catalogue().find(s.depth.name) : nullptr;
        SlotMeta det_meta;
        SlotMeta dep_meta;
        if (dm) det_meta = prepare_slot(*dm);
        if (pm) dep_meta = prepare_slot(*pm);
        std::string failed;
        failed.reserve(32);
        if (s.detector.change) {
            det_touched = true;
            unload_detector();
            clear_detector_meta(resident);
        }
        if (s.depth.change) {
            dep_touched = true;
            unload_depth();
            clear_depth_meta(resident);
        }
        if (dm) {
            const char* kind = "";
            bool mask = false;
            if (load_detector(*dm, kind, mask)) {
                det_meta.kind.assign(kind);  // fits the reserved capacity
                resident.yolo_available = true;
                resident.model_name = std::move(det_meta.name);
                resident.model_path = std::move(det_meta.path);
                resident.model_kind = std::move(det_meta.kind);
                resident.mask = mask;
            } else {
                failed.append("detector");
            }
        }
        if (pm) {
            if (load_depth(*pm)) {
                resident.depth_available = true;
                resident.depth_model_name = std::move(dep_meta.name);
                resident.depth_model_path = std::move(dep_meta.path);
                resident.depth_metric = dep_meta.metric;
            } else {
                failed.append(failed.empty() ? "depth" : ",depth");
            }
        }
        ModelSet commit = resident;
        std::string error = failed.empty() ? std::string() : "LOAD_FAILED:" + failed;
        if (failed_out) *failed_out = failed;
        return ctl.finish_swap(std::move(commit), error);
    } catch (...) {
        if (failed_out) {
            try {
                failed_out->assign("exception");  // fits a default string's inline capacity
            } catch (...) {
            }
        }
        if (det_touched) {
            unload_detector();
            clear_detector_meta(resident);
        }
        if (dep_touched) {
            unload_depth();
            clear_depth_meta(resident);
        }
        return ctl.fail_swap(det_touched, dep_touched);
    }
}

}  // namespace rv

#endif  // REBOT_VISION_MODEL_SWAP_H_
