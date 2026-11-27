mod frames;
mod gray;
mod http;
mod tracking;

pub use frames::{
    parse_contour, DepthFrameInfo, Detection, DetectionFrame, GrayFormatDescriptor, MaskFocus,
    ModeInfo, ProviderModels, ResidentModels, VisionMode, MAX_CONTOUR_VERTICES,
};
pub(super) use frames::refusal_message;

pub use gray::{
    parse_gray_frame, parse_gray_frame_with_format, GrayFrame, GrayFrameError, GrayTriggerResult,
    GRAY_FRAME_ALIGN, GRAY_FRAME_MAGIC, GRAY_MAX_HEADER_BYTES, GRAY_V1_HEIGHT, GRAY_V1_LUMA,
    GRAY_V1_LUMA_RANGE, GRAY_V1_STRIDE, GRAY_V1_WIDTH,
};

pub use http::{
    grasp_mode, rescale_frame_to_cam, CameraHoldTicket, DetectionSource, HttpDetectionSource,
    GRASP_PIPELINES,
};
pub(crate) use http::TRIGGER_WAIT_MS;

pub use tracking::{box_centre, box_changed_px, iou, parse_detections, parse_markers, Tracker};
#[doc(hidden)]
pub use tracking::median;
pub(super) use tracking::f;
