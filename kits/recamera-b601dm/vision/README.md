# rebot-vision

The reCamera kit's camera process (C++, SG2002). One process owns the camera and serves:

- H.264 video over WebSocket on `:8001` (`/`; each binary message is one access unit followed by
  an 8-byte little-endian Unix-ms timestamp), the overlay stream `/results` and `/snapshot.jpg`;
- the perception provider API on `:8090` (`/api/inference`, `/api/results/latest`,
  `/api/trigger`, `/api/mode`), which armd proxies under `/api/v1/detection/*` and
  `/api/v1/vision/mode`. API and result shapes: [docs/detection_api.md](../../../docs/detection_api.md).

Modes `off` / `yolo` / `aruco` / `both`, switched at runtime without restarting the process. YOLO
runs on the TPU on the 640x640 inference frame; ArUco runs on the CPU on a zero-copy crop of the
visible 640x360 band of the same frame. All modes share one inference rate (`fps`, 1..15). With
continuous inference off the inference channel stands by at 1 fps and `POST /api/trigger` runs the
current mode once.

Design: [docs/dev_flow_spec_vision_runtime.md](../../../docs/dev_flow_spec_vision_runtime.md).
Differences from the spec are listed at the end of this file.

## Sources

| Path | Origin (sscma-example-sg200x `9af8118`, Apache-2.0) |
|---|---|
| `main/main.cpp` | merged from `yolo-detector/main/main.cpp` and `aruco-detector/main/main.cpp` |
| `main/vision_control.*` | `aruco-detector/main/inference_control.*`, with modes and the combined result |
| `main/aruco_stage.*` | `aruco-detector/main/main.cpp` frame handler |
| `main/http_control.*` | `aruco-detector/main/http_control.*` |
| `main/detector.*` | `yolo-detector/main/detector.*`, unchanged |
| `main/marker_filter.*`, `camera_model.*`, `capture_time.h` | `aruco-detector/main/`, unchanged |
| `main/result_json.*` | new (letterbox geometry, detections / markers JSON) |

Licence: these files stay under Apache-2.0 (`LICENSE-APACHE`); see `THIRD_PARTY_NOTICES.md`.

Not taken from the source apps: MQTT and Home Assistant discovery, RTSP (`:8554`), ONVIF and its
metadata, person tracking with dwell and zone / line counting, privacy blur, the console-managed
`config.json` (zones, lines). `ha_mqtt`, `rtsp_server` and `onvif_service` are still linked because
`app_framework` requires them, but nothing starts them.

## Build

`scripts/build_vision.sh` cross-compiles against an `sscma-example-sg200x` checkout used as an SDK
(`SSCMA_ROOT`, at the commit in `sscma.lock`; CMake stops on another commit unless
`REBOT_VISION_ALLOW_UNPINNED=1`), the reCamera OS SDK (`SG200X_SDK_PATH`) and the musl toolchain
from sophgo host-tools. With a running `sophgo/tpuc_dev` container that mounts those under
`/recamera`:

```sh
RECAMERA_DOCKER=tpuc_recamera scripts/build_vision.sh        # -> target/vision-riscv/rebot-vision
RECAMERA_DOCKER=tpuc_recamera RECAMERA_TOOLCHAIN=... scripts/build_deb.sh --flavor recamera
```

`RECAMERA_ROOT`, `RECAMERA_GCC`, `SG200X_SDK_PATH`, `SSCMA_ROOT` override the paths inside the
container (defaults under `/recamera`). `REBOT_VISION_BIN=<binary>` makes `build_deb.sh` use a
prebuilt binary instead.

Host tests (no SDK; nlohmann-json from `SSCMA_ROOT/components/json` or the system package):

```sh
cmake -S kits/recamera-b601dm/vision/tests -B target/vision-tests
cmake --build target/vision-tests && ctest --test-dir target/vision-tests
```

## Run

The init script `/etc/init.d/K92rebot-teleop` starts it after armd through
`/usr/share/rebot-teleop/rebot-vision-run` (nice 10, restart with backoff), after stopping a running
camera app (yolo-detector, aruco-detector, ...) with that app's own init script. Log:
`/var/log/rebot-vision.log`.

Configuration: `/userdata/rebot-teleop/vision.json` (seeded from
`/usr/share/rebot-teleop/vision.json.default`; keys as the long options with `_`, e.g. `fps`,
`mode`, `inference`, `model`, `family`, `marker_size`, `fx`..`k3`, `min_margin`, `min_side`,
`ids`). Command-line options override the file; `rebot-vision --help` lists them.

YOLO model: not in the repository or the deb. The first existing file of `vision.json` `model`,
`/userdata/rebot-teleop/models/yolo11n.cvimodel`,
`/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_detection_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_cv181x_int8.cvimodel` is loaded. Without a model `yolo` and `both`
answer 409 and ArUco works.

## Differences from the spec

- No holds (`/api/mode/hold`): the grasp and the calibration drawer set the mode and restore the
  previous one afterwards (user decision).
- No `/api/model` runtime model switch; the model is chosen in `vision.json` and loaded at start.
- The base mode is not written back to `vision.json`; after a restart the mode comes from
  `vision.json` (`mode`, default `yolo`) in standby.
- No `vision` group in the console `config_schema` (the console restarts the whole app, armd
  included, on a config change); mode and rate are set in the cockpit or over the API.
- Default rate 5 fps until the device measurements in the spec (section 8.2) pick one.
