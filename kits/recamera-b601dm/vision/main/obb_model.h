#ifndef _OBB_MODEL_H_
#define _OBB_MODEL_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <sscma.h>
#include "detector.h"
#include "obb_geom.h"

namespace yolo {

using Engine = ma::engine::Engine;

class ObbModel final : public ma::model::Detector {
public:
    explicit ObbModel(Engine* engine);
    ~ObbModel() override = default;

    static bool isValid(Engine* engine, bool allow_auto_1out = true, const char* model_path = nullptr);

    const std::vector<Detection>& getDetections() const { return detections_; }
    /** Classes of the model (Layout A: channels - 5; Layout B: the class head's channels). */
    int numClasses() const { return num_classes_; }

protected:
    ma_err_t postprocess() override;

private:
    std::vector<Detection> detections_;
    bool is_layout_a_ = false;
    int num_classes_ = 0;
};

}  // namespace yolo

#endif  // _OBB_MODEL_H_
