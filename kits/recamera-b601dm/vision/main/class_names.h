// Class names of the loaded YOLO model (detection `class_name`, debug overlay labels, the
// mask-focus label match). Portable: no SDK, host-tested (tests/class_names_test.cpp).
#ifndef REBOT_VISION_CLASS_NAMES_H_
#define REBOT_VISION_CLASS_NAMES_H_

#include <string>
#include <vector>

namespace rv {

/** The 80 COCO class names (yolo11n detection / segmentation). */
extern const char* const kCocoClasses[80];

/** Class id -> name for one loaded model. */
struct ClassNames {
    std::vector<std::string> names;  ///< from the catalogue entry or the sidecar file (empty: none)
    bool coco = false;               ///< no names given and an 80-class detect / segment model
    std::string source;              ///< "catalogue", the sidecar path, "coco" or "class_<id>" (for the log)
    std::string warning;             ///< e.g. a names count that differs from the model's classes

    /** names[id], else COCO when `coco`, else "class_<id>". */
    std::string name(int id) const;
};

/** One name per line (Darknet .names / sscma labels.txt); '\r' and surrounding blanks trimmed, blank lines skipped. */
std::vector<std::string> parse_names_text(const std::string& text);

/** The sidecar names file of a model: its path with the last extension replaced by ".names". */
std::string names_sidecar_path(const std::string& model_path);

/**
 * Names of a YOLO model of `kind` (detect / segment / obb) with `num_classes` classes (0 =
 * unknown): the catalogue entry's `classes`, else the sidecar file (names_sidecar_path), else
 * COCO for a detect or segment model with 80 classes, else "class_<id>".
 */
ClassNames resolve_class_names(const std::vector<std::string>& catalogue_classes, const std::string& model_path,
                               const std::string& kind, int num_classes);

/**
 * Classes of a plain detection model from its output shapes (NCHW, as sscma reports them; 0 =
 * unknown). Split heads (YOLO11 / YOLO26: box [1,64|4,H,W] + cls [1,nc,H,W] per scale): the
 * channel count that is not 64, 4 or 1, when all such outputs agree. One output [1,a,b]
 * (YOLOv8 4 + nc, YOLOv5 5 + nc): 80 when its short side is 84 or 85, else unknown.
 */
int detect_num_classes(const std::vector<std::vector<int>>& output_shapes);

}  // namespace rv

#endif  // REBOT_VISION_CLASS_NAMES_H_
