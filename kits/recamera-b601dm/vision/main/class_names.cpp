#include "class_names.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace rv {

const char* const kCocoClasses[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
};

std::string ClassNames::name(int id) const {
    if (id >= 0 && static_cast<size_t>(id) < names.size()) return names[static_cast<size_t>(id)];
    if (coco && id >= 0 && id < 80) return kCocoClasses[id];
    return "class_" + std::to_string(id);
}

std::vector<std::string> parse_names_text(const std::string& text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        const size_t b = line.find_first_not_of(" \t\r");
        if (b == std::string::npos) continue;
        const size_t e = line.find_last_not_of(" \t\r");
        out.push_back(line.substr(b, e - b + 1));
    }
    return out;
}

std::string names_sidecar_path(const std::string& model_path) {
    const size_t slash = model_path.find_last_of('/');
    const size_t dot = model_path.find_last_of('.');
    const bool has_ext = dot != std::string::npos && (slash == std::string::npos || dot > slash + 1);
    return (has_ext ? model_path.substr(0, dot) : model_path) + ".names";
}

ClassNames resolve_class_names(const std::vector<std::string>& catalogue_classes, const std::string& model_path,
                               const std::string& kind, int num_classes) {
    ClassNames c;
    if (!catalogue_classes.empty()) {
        c.names = catalogue_classes;
        c.source = "catalogue";
    } else {
        const std::string sidecar = names_sidecar_path(model_path);
        std::ifstream f(sidecar);
        if (f) {
            std::ostringstream text;
            text << f.rdbuf();
            c.names = parse_names_text(text.str());
            if (!c.names.empty()) c.source = sidecar;
        }
    }
    if (!c.names.empty()) {
        if (num_classes > 0 && static_cast<size_t>(num_classes) != c.names.size()) {
            c.warning = std::to_string(c.names.size()) + " class names for a model with " +
                        std::to_string(num_classes) + " classes";
        }
        return c;
    }
    c.coco = (kind == "detect" || kind == "segment") && num_classes == 80;
    c.source = c.coco ? "coco" : "class_<id>";
    return c;
}

int detect_num_classes(const std::vector<std::vector<int>>& output_shapes) {
    if (output_shapes.size() == 1) {
        const std::vector<int>& s = output_shapes[0];
        if (s.size() != 3) return 0;
        const int side = std::min(s[1], s[2]);
        return (side == 84 || side == 85) ? 80 : 0;
    }
    int nc = 0;
    for (const std::vector<int>& s : output_shapes) {
        if (s.size() != 4) return 0;
        const int ch = s[1];
        if (ch == 64 || ch == 4 || ch == 1) continue;
        if (nc != 0 && ch != nc) return 0;
        nc = ch;
    }
    return nc;
}

}  // namespace rv
