# rebot-vision

The reCamera kit's camera process (C++, SG2002). One process owns the camera and serves:

- H.264 video over WebSocket on `:8001` (`/`; each binary message is one access unit followed by
  an 8-byte little-endian Unix-ms timestamp), the overlay stream `/results` and `/snapshot.jpg`;
- the perception provider API v2 on `:8090` (`/api/inference`, `/api/results/latest`,
  `/api/trigger`, `/api/mode`, `/api/subscribe`, `/api/subscriptions`), which armd proxies under
  `/api/v1/detection/*` and `/api/v1/vision/*`. API and result shapes:
  [docs/detection_api.md](../../../docs/detection_api.md).

Vision runs on demand per caller. YOLO runs on the TPU on the 640x640 inference frame; ArUco runs
on the CPU on a zero-copy crop of the visible 640x360 band of the same frame.

- Standby: with no subscription nothing is inferred; the inference channel's camera gate is at
  1 fps, the H.264 stream and the snapshot keep running.
- `POST /api/trigger {"pipelines": ["yolo"] | ["aruco"] | ["yolo", "aruco"]}` runs those pipelines on
  the first frame after the request. Without `pipelines` it runs the default subscription's mode
  (the last active mode when that is `off`), as in API v1.
- `POST /api/subscribe {"client", "pipelines", "fps"?, "ttl_ms"?}` starts continuous inference for one
  caller until it is deleted or not kept alive for `ttl_ms` (default 3000). What runs is the union of
  the active subscriptions' pipelines at the highest requested rate, capped at the configured `fps`
  (`vision.json`, default 5, 1..15).
- `/api/mode` (modes `off` / `yolo` / `aruco` / `both`) sets the default subscription, which never
  expires; it joins the union like any other subscription.

## Frame rate and triggers

- The inference channel (CH0) runs at 15 fps in VPSS hardware in every state. Its hardware
  frame-rate control is set once at start-up (sscma-example `components/sophgo/video/video.c`
  `setGrpChn`); a runtime rate change only reaches the camera callback's software gate
  (`ma_camera_sg200x.cpp` `vpssCallback`). 1.2.0 started CH0 at `vision.json` `fps` (5), so
  `fps` 10 and 15 still gave about 5 results/s.
- The camera gate bounds the CPU copies of CH0 frames: 1 fps in standby, twice the continuous rate
  up to 7 fps,
  every frame above that, every frame while a trigger waits (`VisionControl::camera_gate_fps`).
  The gate passes a frame once 90 % of its period has passed since the last one, so it is not
  used to set the rate: `RateGate` picks the continuous rate's frames by due time, which holds the
  rate under arrival jitter. It restarts when continuous work starts or its rate changes.
- Expired subscriptions are dropped by the frame thread (on the next CH0 frame, at most about 1 s
  late in standby) and by every subscription request.
- Results per second = `min(fps, 15, 1000 / pipeline ms)`. `GET /api/mode` reports
  `measured.input_fps` (CH0 frames reaching rebot-vision) and `measured.result_fps`.
- A trigger opens the gate at once and its pipelines are added to that frame's continuous ones (the gate rate is written to the camera directly, because the
  frame thread is blocked in `retrieveFrame` until the old gate passes a frame) and is served by the
  first frame that arrives after the request: one CH0 frame (up to 67 ms) plus the pipelines.
- With the forked `cv181x_vpss.ko` (online frame-rate control) the ISP runs at the fastest
  channel's rate, `max(stream_fps, 15)`; with `stream_fps` 15 (default) that is unchanged.
- Default `fps` 5 (`vision.json.default` and the code). Measured on the device with 1.2.1
  (whole-chip CPU, results/s from `measured.result_fps`):

  | mode @ fps | whole-chip CPU | results/s |
  |---|---|---|
  | both @ 5 / 10 / 15 | 58 / 75 / 75 % | 4.7 / 7.2 / 7.2 |
  | yolo @ 15 | 40 % | 14.3 |
  | aruco @ 10 | 82 % | 8.7 |
  | aruco @ 5 | 40 % | 5 |

  `both` stops gaining above about 7 results/s: the ArUco pipeline (`pipelines.aruco.ms` 60-75 ms
  at decimate 2) and YOLO share the frame thread.

## ArUco cost

- Pose: `main/tag_pose.cpp` runs apriltag's orthogonal iteration (2 x 50 steps per marker) with
  fixed-size arithmetic. apriltag's `estimate_tag_pose` allocates matrix temporaries in every
  step, about 26 000 heap calls per marker. Same corners give the same pose (host benchmark).
- Detector: `min_white_black_diff` 20 instead of apriltag's 5, so sensor noise in flat areas is
  not segmented; other settings as aruco-detector 0.1.3 (decimate 2, quad sigma 0, refine edges,
  1 thread). `vision.json` `decimate` (default 2.0) sets apriltag's `quad_decimate`: quad
  detection runs on the image shrunk by that factor, and with refine edges the corners are then
  refined on the full-resolution image. On the device quad detection dominates the 60-75 ms.
- Decimate sweep, host, 1000 synthetic scenes x 3 runs (`aruco_bench --scenes 1000 --reps 3
  --sweep 1,1.5,2,3,4`). Side = shortest black-border edge in px of the 640x360 band; at fx 600 a
  marker of S m at distance z m is 600 x S / z px (5 cm at 20-40 cm: 75-150 px; 2-3 cm at 20-40
  cm: 30-90 px). Low contrast = white minus black below 90 levels (dim light, glare, faint print).
  Pose columns compare with decimate 1.0 on markers both find.

  | decimate | host ms/frame | recall all | 14-20 px | 20-30 px | 30-40 px | >= 40 px, contrast >= 90 | >= 40 px, contrast < 90 | corner error px | pose vs 1.0, median | rotation > 5 deg vs 1.0 |
  |---|---|---|---|---|---|---|---|---|---|---|
  | 1.0 | 7.63 | 78.7 % | 70 % | 73 % | 75 % | 99.1 % | 65.1 % | 0.106 | - | - |
  | 1.5 | 3.34 | 73.3 % | 68 % | 72 % | 73 % | 98.4 % | 36.8 % | 0.144 | 0.17 mm, 0.07 deg | 6.7 % |
  | 2.0 | 3.05 | 68.9 % | 61 % | 70 % | 71 % | 97.4 % | 21.4 % | 0.194 | 0.19 mm, 0.09 deg | 8.7 % |
  | 3.0 | 1.84 | 56.9 % | 25 % | 60 % | 70 % | 96.6 % | 10.2 % | 0.286 | 0.26 mm, 0.13 deg | 11.8 % |
  | 4.0 | 1.31 | 44.3 % | 2 % | 34 % | 63 % | 96.5 % | 5.7 % | 0.317 | 0.21 mm, 0.14 deg | 11.9 % |

  Every setting above 1.0 loses recall on markers of 40 px and larger, mostly low-contrast ones;
  no decimate keeps the 1.0 recall there. 2.0 stays the default: 3.0 and 4.0 cut host time by 40
  and 57 % but lose low-contrast markers (10 and 6 % found) and markers below 25 px; 1.5 finds more
  low-contrast markers (37 %) at about the same host time as 2.0, its device time is not measured.
  "Rotation > 5 deg" counts markers whose pose takes the other solution of the planar pose
  ambiguity (tilted or small markers) when the corners move by about 0.1 px.
- Host benchmark (`tests/aruco_bench.cpp`, built when `SSCMA_ROOT` is set; ctest `aruco_bench`
  checks recall, false positives, corner error and pose against the 0.1.3 settings, ctest
  `aruco_decimate` checks the default decimate against 1.0: recall of markers >= 40 px at normal
  contrast within 3 points, corner error within 0.15 px):

```sh
SSCMA_ROOT=<sscma-example-sg200x> cmake -S kits/recamera-b601dm/vision/tests -B target/vision-tests
cmake --build target/vision-tests && target/vision-tests/aruco_bench --scenes 400 --reps 3
```

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
`mode`, `inference`, `model`, `family`, `marker_size`, `decimate`, `fx`..`k3`, `min_margin`,
`min_side`, `ids`). Command-line options override the file; `rebot-vision --help` lists them.

YOLO model: not in the repository or the deb. The first existing file of `vision.json` `model`,
`/userdata/rebot-teleop/models/yolo11n.cvimodel`,
`/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_detection_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_cv181x_int8.cvimodel` is loaded. Without a model `yolo` and `both`
answer 409 and ArUco works.

## Differences from the spec

- No holds (`/api/mode/hold`). From 1.3.0 callers no longer switch and restore the mode: the grasp
  triggers `["yolo"]` per centring step, the cockpit overlay and the calibration drawer hold their
  own subscriptions (user decision; with one global mode the caller that restored last won).
- No `/api/model` runtime model switch; the model is chosen in `vision.json` and loaded at start.
- The base mode is not written back to `vision.json`; after a restart the mode comes from
  `vision.json` (`mode`, default `yolo`) in standby.
- No `vision` group in the console `config_schema` (the console restarts the whole app, armd
  included, on a config change); the rate cap is set over the API (`POST /api/mode {"fps"}`) or in
  `vision.json`.
- Default rate 5 fps, from the device measurements under "Frame rate and triggers" (both @ 5:
  58 % whole-chip CPU; @ 10 or 15: 75 % for 7.2 results/s).
