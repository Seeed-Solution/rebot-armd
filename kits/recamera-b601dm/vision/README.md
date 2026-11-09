# vision/ (phase 2, placeholder)

This directory will hold the kit's perception process (`rebot-vision`, C++ on the reCamera),
which serves YOLO detections and ArUco markers through the perception provider API that
armd proxies under `/api/v1/detection/*`. Design: [docs/dev_flow_spec_vision_runtime.md](../../../docs/dev_flow_spec_vision_runtime.md).

Today the cockpit and armd use the camera's own detector service (`yolo-detector`, default
`http://127.0.0.1:8090`, set with `ARMD_DETECTOR_URL`), which is not part of this kit; its
API as armd uses it is in [docs/detection_api.md](../../../docs/detection_api.md).
