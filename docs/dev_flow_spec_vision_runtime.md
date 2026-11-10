# Spec: vision runtime in rebot-armd (`rebot-vision`)

Status: implemented in rebot-teleop 1.2.0 with the deviations listed in `kits/recamera-b601dm/vision/README.md` (no holds, no `/api/model`; grasp and calibration set and restore the mode). Design history: from 1.3.0 the perception API is version 2 (per-request pipelines, subscriptions; grasp and calibration no longer set the mode, see [detection_api.md](detection_api.md)), and `sscma-example-sg200x` `9af8118` is on the fork's public main (the branch `phase3-hw` named below was local). Device measurements (section 8.2) are open. Date: 2026-11-09; layout updated the same day for the core/kit
split ([docs/architecture.md](architecture.md)): the vision code is part of the official reCamera kit and
lives in `kits/recamera-b601dm/vision/`, not in a top-level `kits/recamera-b601dm/vision/`.
Inputs read: this repo at `e799c2a`, `sscma-example-sg200x` at `9af8118` (local branch `phase3-hw`),
`rebot-companion` at `4e4d5d9`, `sensecraft-solutions` branch `feat/rebot-agent-arm` (`7a2ff5c`).
Items marked **(needs verification)** are inferences that have not been checked on a device or in a build.

## 0. Problem and decision

The reCamera console (supervisor App Manager) runs one gallery app at a time:

- `solutions/supervisor/main/include/api_app.h:17-21`: "Exactly one application may own the camera at a
  time. Switching is: stop current (10s, TERM only) -> sleep 2 -> start target (15s)".
- `solutions/supervisor/main/src/api_app.cpp:870-906`: `switch` reads `active_script` from `state.json`,
  runs `sh_stop(active_script)`, then `sh_start(target_script)`.

armd (`rebot-teleop`) and the camera app (`yolo-detector` or `aruco-detector`) are two gallery apps, so
activating the camera app stops armd. The SenseCraft solution works around this with a reboot
(`sensecraft-solutions` `solutions/rebot_agent_arm/guide.md:94`, `devices/camera_app.yaml` header comment):
`S92rebot-teleop` starts armd at boot and `app_restore` starts the camera app.

Decision (user): move the vision code into this repo and ship two runtimes.

| Runtime | Contents | Target |
|---|---|---|
| **lite** | `armd` only | any Linux host (x86_64, aarch64, riscv64 musl) |
| **reCamera** | `armd` + `rebot-vision` (one process owning the camera) + model config | reCamera SG2002, one gallery app `rebot-teleop` |

`rebot-vision` runs YOLO (TPU) and ArUco/AprilTag (CPU) with modes `off | yolo | aruco | both`, switched at
runtime over HTTP. In `both`, every processed frame runs YOLO and ArUco and one result carries both.
YOLO and ArUco share **one inference frame rate** for all modes (no separate ArUco cadence).

---

## 1. Source layout

### 1.1 Options

**A. Vendor everything** (app code + `components/app_framework`, `sscma-micro`, `sophgo`, `debug_stream`,
`rtsp_server`, `ha_mqtt`, `onvif_service`, `apriltag`, `json`, `libwebsockets`).
- Size: `sscma-micro` 33 MB, `app_framework` 5.4 MB, `apriltag` 1.1 MB, `json` 0.9 MB, others < 0.5 MB each
  (`du -sh` on the local checkout). About 42 MB of third-party C/C++ in an MIT Rust repo.
- `app_framework` cannot be taken alone: `components/app_framework/CMakeLists.txt:16-22` has
  `REQUIREDS sscma-micro sophgo` and `PRIVATE_REQUIREDS rtsp_server debug_stream ha_mqtt onvif_service pthread`.
  Dropping MQTT in the app does not remove these build dependencies.
- Camera fixes land in these components (6 VENC VB blocks + FRC, `CVI_SYS_MmapCache` copy in
  `components/sscma-micro/porting/sophgo/sg200x/ma_camera_sg200x.cpp`). A vendored copy has to be re-synced
  by hand for every fix.

**B. Vendor only the app-level code; use a pinned `sscma-example-sg200x` checkout as an external SDK**
(like `RECAMERA_TOOLCHAIN` for armd today, `scripts/build_deb.sh:13-16`).
- The build system already works this way: a solution's `CMakeLists.txt` sets `ROOT_DIR`, includes
  `${ROOT_DIR}/cmake/toolchain-riscv64-linux-musl-x86_64.cmake` and `${ROOT_DIR}/cmake/project.cmake`
  (`solutions/aruco-detector/CMakeLists.txt:8-12`); `project.cmake:44-66` scans `${ROOT_DIR}/components/*`
  and includes `${PROJECT_DIR}/main/CMakeLists.txt`. Pointing `ROOT_DIR` at an external checkout instead
  of `../../` is a one-line change.
- The SoC SDK (`SG200X_SDK_PATH`, 1.1 GB: MPI, TPU runtime, sysroot, `project.cmake:7-35`) is external in
  both options; no option removes it.

### 1.2 Recommendation: B

```
kits/recamera-b601dm/vision/
  CMakeLists.txt          # project(rebot-vision VERSION x.y.z); ROOT_DIR = $ENV{SSCMA_ROOT}; checks pin
  sscma.lock              # repo URL + full commit hash of sscma-example-sg200x, required libwebsockets build
  main/
    CMakeLists.txt        # component_register(main ... PRIVATE_REQUIREDS app_framework apriltag sscma-micro
                          #   sophgo debug_stream json)  (rtsp_server/ha_mqtt/onvif_service come via app_framework)
    main.cpp              # new: one App, mode state machine, per-frame dispatch (sections 2.2-2.4)
    vision_control.{h,cpp}# new: modes, holds, inference toggle, trigger, latest (portable, host-tested)
    http_control.{h,cpp}  # from aruco-detector/main/http_control.cpp (327 lines), routes from vision_control
    yolo_stage.{h,cpp}    # from yolo-detector/main/detector.{h,cpp} (133 lines) + on_frame body, main.cpp:349-438
    aruco_stage.{h,cpp}   # from aruco-detector/main/main.cpp:394-597 (gray conversion, detect, filter, pose)
    marker_filter.{h,cpp} # copied unchanged from aruco-detector/main/
    camera_model.{h,cpp}  # copied unchanged
    capture_time.h        # copied unchanged
    result_json.{h,cpp}   # new: one result envelope for all modes (section 2.3)
  tests/                  # host-only CMake (no SDK), like aruco-detector/tests and app_framework/tests
  rootfs/                 # nothing; packaging stays in kits/recamera-b601dm/packaging
scripts/build_vision.sh   # docker build of kits/recamera-b601dm/vision/ (section 4.4)
```

Not copied: `person_tracker.*` (343 lines, dwell/line counting), `mqtt_payload.*`, `app_config.*` (zones/lines
from the console schema), ONVIF meta, privacy blur (`yolo-detector/main/main.cpp:55-64, 100-118, 303-322,
382-429, 466-492`). Reasons in section 2.6.

Estimated copied/new C++: 1.5-2.0 k lines (needs verification after the merge).

Pin rules:
- `sscma.lock` must name a commit that exists on a public remote. Since 2026-11-10 the pinned commit
  `9af8118` is the head of `main` on the fork `github.com/suharvest/sscma-example-sg200x` (fast-forwarded
  from `phase3-hw`). The fork is about 305 commits ahead of `Seeed-Studio/sscma-example-sg200x`; the parts
  rebot-vision depends on (video/VPSS VB blocks, camera copy, websocket transport) are not upstream yet.
- `kits/recamera-b601dm/vision/CMakeLists.txt` fails configure when `git -C $SSCMA_ROOT rev-parse HEAD` differs from the lock,
  unless `-DREBOT_VISION_ALLOW_UNPINNED=ON`.
- `components/libwebsockets/fetch_and_build.sh` must be run once per SSCMA checkout
  (`sscma-example-sg200x/README.md:64-80`); `build_vision.sh` runs it when the artefact is missing.

---

## 2. `rebot-vision` runtime design

### 2.1 What the two apps do today

| | yolo-detector | aruco-detector 0.1.3 |
|---|---|---|
| Inference channel (CH0, RGB888, CPU copy) | 640x640 @ 8 fps (`main.cpp:43-45`) | 640x360 @ 25 fps (`main.cpp:49-51`) |
| Stream channel (CH2 H.264) | 1280x720 @ 15 fps, ws only (`main.cpp:46-52, 69`) | 1280x720 @ 30 fps, ws + RTSP (`main.cpp:54-57`) |
| Work per frame | TPU `Detector::detect` (~70 ms `inference_time_ms` in `yolo-detector.md:76`), "SG2002 sustains ~4-5 inferences/s" (`main.cpp:45`) | AprilTag on 640x360 gray, decimate 2, 1 thread (`main.cpp:340-344`), "~4 ms" (`aruco-detector.md:25`, not measured by us) |
| HTTP :8090 | `/api/inference`, `/api/results/latest` (`inference_control.cpp:142-157`); **no `/api/trigger`** | same + `POST /api/trigger` (`inference_control.cpp:213-277`) |
| Result list | `detections` (`class_id, class_name, confidence, x, y, w, h` normalised, `mqtt_payload.cpp:14-37`) | `markers` (`id, margin, center, corners, pose{t, r_deg, error}` in pixels, `main.cpp:548-563`) |
| Idle | CH0 1 fps (`InferenceControl::kIdleFps`), stream 15 fps | CH0 1 fps, **stream and ISP stay at 30 fps** |

Both use one `af::App` with one inference channel; `af::App` has no second inference channel
(`components/app_framework/include/af/app.h:58-71, 193`). CH0 geometry is fixed at start; only its rate
changes at runtime (`app.h:265-271`, `set_camera_fps`).

### 2.2 One camera geometry for all modes

CH0 is configured **640x640 RGB888** in every mode. VPSS uses `ASPECT_RATIO_AUTO`
(`components/sophgo/video/src/video_paramparse.c:99-101`), and `debug_stream_letterbox_to_display`
(`components/debug_stream/src/debug_stream.cpp:608-630`) is written for a letterboxed CH0: a 16:9 sensor image
sits in a 640x360 band at y = 140..500 of the 640x640 frame. **(needs verification:** dump one CH0 frame and
check the bars.)

- YOLO runs on the full 640x640 frame (the model input).
- ArUco runs on `frame.sub_frame({0, 140, 640, 360})` (zero-copy crop, `af/frame.h:14-15, 119-125`). The
  pixels and intrinsics are the same as aruco-detector's 640x360 channel today, so `fx fy cx cy k1..k3`,
  `marker_size`, `decimate`, the filter and `CameraModel` carry over unchanged. The crop rect is computed from
  CH0 and stream geometry with the same formula as `debug_stream.cpp:615-630`, not hard-coded.
- Consequence: switching modes never reconfigures VPSS, so no camera restart.

Cost: ArUco-only mode copies 640x640x3 = 1.2 MB per frame instead of 0.69 MB. With the cached copy
(`CVI_SYS_MmapCache`), yolo-detector's copy thread measured 2.6 % CPU at 8 fps (2026-11-07). At the
common rate chosen in 2.4 this is expected to be 2-5 % **(needs verification)**.

### 2.3 Modes, API and result shape

#### State

```
mode        off | yolo | aruco | both     persisted base selection (vision.json)
holds       [{holder, pipelines:[yolo|aruco], expires_at}]   max 8, TTL 1..60 s
effective   = pipelines(mode) ∪ pipelines(all unexpired holds)
enabled     continuous inference toggle (existing /api/inference)
fps         one inference rate for every mode (2.4)
```

Rules:
- Processing runs when `effective` is non-empty and (`enabled` or a trigger or a hold is active).
  A hold implies processing for its duration, like grasp's "turn inference on for the duration"
  (`src/grasp.rs:921-933`).
- `POST /api/mode {"mode":"off"}` also sets `enabled=false`. `POST /api/inference {"enabled":true}` while
  `effective` is empty sets `mode` to `last_active_mode` (default `yolo`). This keeps the cockpit Detect button
  and older clients working (`src/api/index.html:2881-2904`).
- CH0 runs at `fps` while processing, at `kIdleFps` (1) otherwise, as both apps do today.
- Mode/hold changes take effect at the next frame boundary in the frame thread (atomic snapshot read at the
  top of `on_frame`). No model load, no VPSS change. Expected switch time: one frame interval at the current
  CH0 rate, plus the 1 fps -> `fps` ramp when leaving idle: <= 1.1 s from idle, <= 1/`fps` s between active
  modes **(needs verification, measured in 8.2)**.

#### On-demand trigger in every mode (requirement)

User requirement: YOLO, ArUco and `both` all support on-demand operation, not only continuous streaming.

- Standby: with `enabled=false` and no hold, no pipeline runs and CH0 stays at the idle rate
  (`kIdleFps`, 1 fps), as aruco-detector does today (`/api/inference` reports `idle_fps`).
- `POST /api/trigger?wait_ms=<0..5000>` runs the active pipelines once (`effective`, or
  `last_active_mode` when `effective` is empty) — `yolo`, `aruco` or both — on the first frame captured after
  the call, returns that one result and goes back to standby. It does not change `enabled`, `mode` or holds,
  so it does not interfere with clients that turned continuous inference on.
- Compatible with today's aruco-detector trigger as documented in [docs/detection_api.md](detection_api.md):
  `wait_ms` default 2500, max 5000; success body `{"success":true,"frame_id","ts_ms","age_ms",
  "inference_time_ms","count","markers":[...]}` (plus `detections` and the other envelope keys below);
  no fresh frame within `wait_ms` -> `{"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}`;
  armd forwards it as `POST /api/v1/detection/trigger` with its `wait_ms` + 2 s upstream timeout. Clients
  that fall back to enable -> `latest` -> disable on a 404 keep working because the route now always exists.
- In mode `off` a trigger runs `last_active_mode` (default `yolo`), so a trigger always has a pipeline.
- Tests (8.1): trigger in each of `yolo`, `aruco`, `both` from standby returns one result on a newer frame,
  leaves `enabled` false, and returns to the idle rate; trigger while continuous inference runs returns the next
  frame's result without stopping it.

#### HTTP :8090 (loopback and LAN, same as today)

| Route | Method | Change |
|---|---|---|
| `/api/inference` | GET/POST | unchanged shape `{"enabled","fps","active_fps","idle_fps"}`, plus `"mode"` |
| `/api/results/latest` | GET | `?after=&wait_ms=` unchanged; body below |
| `/api/trigger` | POST | now in every mode; runs `effective` (or `last_active_mode` if empty) once on a frame newer than the call; `wait_ms` default 2500, max 5000 (as `aruco-detector/main/inference_control.cpp:213-255`) |
| `/api/mode` | GET | `{"mode","effective":[...],"holds":[{holder,pipelines,ttl_ms}],"fps","available":{"yolo":bool,"aruco":true},"model":{"name","path"}}` |
| `/api/mode` | POST | `{"mode":"off"|"yolo"|"aruco"|"both"}` -> same body as GET. Bad value -> 400 `{"error"}` |
| `/api/mode/hold` | POST | `{"holder":str<=64,"pipelines":["aruco"],"ttl_ms":1000..60000}` create or renew; returns GET body |
| `/api/mode/hold` | DELETE | `?holder=` release; unknown holder is a no-op 200 |
| `/api/model` | GET/POST | list / select a YOLO model (2.5) |

`available.yolo=false` when the model failed to load; then `yolo`/`both` requests return 409
`{"error":"yolo unavailable","detail":...}` and ArUco still works.

#### Result envelope (`/api/results/latest`, `/api/trigger`, MQTT not shipped)

```json
{"enabled":true, "mode":"both", "frame_id":42, "ts_ms":1762680000123, "age_ms":35,
 "inference_time_ms":81.2, "width":640, "height":360,
 "pipelines":{"yolo":{"ran":true,"ms":74.0},"aruco":{"ran":true,"ms":6.1}},
 "detections":[{"class_id":39,"class_name":"bottle","confidence":0.71,"x":0.42,"y":0.55,"w":0.08,"h":0.21}],
 "count":1, "family":"aruco4x4_50", "undistorted":false, "rejected":0,
 "markers":[{"id":3,"margin":61.0,"center":[...],"corners":[[...]...],"pose":{"t":[...],"r_deg":[...],"error":0.0}}]}
```

- `detections` and `markers` are always present (empty arrays when the pipeline did not run), so a consumer
  that reads one key works in every mode:
  - grasp reads only `detections` (`src/grasp.rs:311-326`);
  - rebot-companion `camera_client.py:112,120` reads only `markers`;
  - the cockpit and rebot-companion `parse_detections` merge both (`index.html:2636-2640`,
    `rebot_companion/snapshot.py:550-566`).
- `count` keeps aruco-detector's meaning (accepted markers); trigger keeps `success` and the
  `{"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}` timeout body.
- **Coordinate frame (behaviour change):** `detections` x/y/w/h are normalised to the visible image
  (the 640x360 content band), not to the 640x640 letterboxed model input; `markers` pixels are in the 640x360
  band; `width`/`height` give that size. yolo-detector today normalises to the model input
  (`detector.cpp:112-113`, no letterbox correction found), while grasp assumes a 640x360 image with intrinsics
  at that size (`src/grasp.rs:102, 429, 474`). No client compensates for letterboxing (grep of grasp, cockpit,
  rebot-companion). If the letterbox in 2.2 is confirmed, today's grasp targets are vertically off by up to
  140 px and this change fixes it; check before and after on the device **(needs verification)**.

#### Debug WS `/results` on :8001

Built by `DebugWsSink` (`components/app_framework/src/app.cpp:68-97`): `boxes` in display (1280x720) pixels
after letterbox mapping, plus `Result::debug_extra`.
- YOLO boxes go into `Result::boxes` as today.
- Markers do **not** go into `boxes` (aruco-detector puts them there with label `ID:n`, `main.cpp:527-538`);
  they go into `debug_extra` as `"markers":[...]` with corners/center mapped to display pixels, and
  `"mode":"..."`. Reason: the cockpit merges `markers` with boxes (`index.html:2636-2640`), so putting markers
  in both lists draws each marker twice, and the marker list carries the pose the calibration live line needs.

#### H.264 WS :8001

Unchanged: one Annex-B access unit per binary message followed by an 8-byte little-endian Unix-ms timestamp
(`debug_stream.cpp:115`; the cockpit reads it at `index.html:5362-5363`). Stream defaults: 1280x720, 15 fps,
1000 kbps CBR, ws only, no RTSP (yolo-detector `main.cpp:46-52, 69`).

### 2.4 One inference rate, and how to choose its default

`--fps N` / `vision.json "fps"`, 1..15, one value for all modes. In `both`, each processed frame runs YOLO
then ArUco in the frame thread and publishes one result. There is no separate ArUco rate.

The TPU path limits the effective rate: yolo-detector reaches ~4-5 inferences/s at CH0 8 fps
(`yolo-detector/main/main.cpp:45`); frames that arrive while the handler runs are dropped by the framework.
In `yolo`/`both` the effective rate is `min(fps, throughput)`; in `aruco` it is `fps`.

Default selection procedure (on a device, arm enabled and torqued, armd at 50 Hz, cockpit streaming video,
one client long-polling `/api/results/latest`):

1. Measure total CPU (`top -d 1` over 60 s, idle %), per-thread CPU (`top -H`), RSS, effective result rate
   (frame_id delta / s), and armd loop timing (observation `timestamp` jitter, any `WATCHDOG` or following-error
   warnings in `/var/log/rebot-teleop.log`) for:
   - mode `both` at fps 5, 10, 15;
   - mode `yolo` and `aruco` at the chosen fps;
   - `off`, inference enabled=false.
2. Budget: total CPU <= 70 % (30 % headroom for armd bursts, Wi-Fi AP, the console), armd loop jitter not
   worse than with vision off. These limits are proposals for the user to confirm.
3. Default = the highest of {15, 10, 5} for which `both` meets the budget. If none does, lower the common rate
   further (4, 3) rather than adding a separate ArUco rate. Record the table in this file and in the README.

Expected numbers (estimates, all **needs verification**): armd 7 % (measured); ISP + VENC + WS at 15 fps
about 9 % (yolo-detector measurement "15 fps ~9 %", `main.cpp:48`); CH0 copy 2-5 %; YOLO pre/post-processing
per frame 10-20 ms CPU (the TPU part does not use CPU if the runtime blocks in an ioctl); ArUco 4-15 ms per
frame at 640x360 decimate 2. At an effective 5 Hz in `both`: about 25-35 % total. ArUco-only at 15 fps:
about 25-40 % total.

Idle ISP rate: aruco-detector kept the stream and therefore the ISP at 30 fps while idle (`main.cpp:54-57`),
about 7.6 % CPU more than needed. `rebot-vision` keeps the stream at 15 fps. Measurement item: CPU with mode
`off` at stream 15 fps vs 10 fps vs the 30 fps of aruco-detector, with the forked `cv181x_vpss.ko` that makes
the ISP follow the fastest channel's FRC (memory note 2026-11-08). On a device with the stock module the ISP
runs at the sensor rate regardless; record which module is installed with each measurement.

Process priority: the init script starts `rebot-vision` with `nice -n 10` so armd's control loop wins CPU
contention; frames drop instead **(needs verification** that VENC callbacks under nice do not stall the stream).

### 2.5 YOLO models

- Shipped: one detection model, `yolo11n` COCO-80. Default path order:
  1. `/userdata/rebot-teleop/models/<name>.cvimodel` (user-provided or downloaded);
  2. `/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel`, the preset the console package
     ships (`solutions/supervisor/rootfs/usr/share/supervisor/scripts/main.sh:16-17, 133-137`)
     **(needs verification** that current console packages still ship it);
  3. `/userdata/local/models/yolo11n_detection_cv181x_int8.cvimodel` (yolo-detector's default,
     `K92yolo-detector` `DEFAULT_MODEL`).
  No `.cvimodel` file is committed to this repo or put in the deb (licence, section 7; root space, 4.3).
- Optional: `yolo26n` (`/userdata/local/models/yolo26n_cv181x_int8.cvimodel` in yolo-detector's manifest).
  Pose/segment models are not supported (the grasp needs boxes).
- Switching: `POST /api/model {"name":"yolo26n"}` loads the new model in-process while the frame thread is
  paused (mode stays; results report `available.yolo=false` during the load), then persists the choice in
  `/userdata/rebot-teleop/vision.json`. Expected 1-3 s **(needs verification)**. If re-initialising the
  sscma-micro engine in-process proves unreliable, the fallback is to restart only `rebot-vision`
  (camera bring-up 3-5 s, first-frame time observed on this camera), never armd.
- The gallery manifest keeps `"models": []`. The console's `setModel` writes `<app>.model` and restarts the
  **whole active app** (`api_app.cpp:1005-1070`, `restart_after_change`), which would stop armd. With an
  empty list the console answers "app has no switchable models" (`api_app.cpp:1032-1037`).
- The TPU model stays loaded in every mode, so `aruco`/`off` -> `yolo` needs no load.

### 2.6 yolo-detector features: keep or drop

| Feature | Decision | Reason |
|---|---|---|
| HTTP :8090 inference/latest/trigger | keep | cockpit, grasp, rebot-companion |
| H.264 WS :8001, `/results` | keep | cockpit video and overlay |
| RTSP :8554 | drop (flag `--rtsp` off by default) | teleop uses WS; RTSP + go2rtc cost ~17 % CPU (memory note 2026-11-07). The manifest's `rtsp_url` is removed |
| MQTT results / HA discovery | drop | no consumer in armd, cockpit or rebot-companion; saves the broker connection. `app_framework` still links `ha_mqtt`/mosquitto (1.1), unused at runtime |
| Person tracker, dwell, zone/line counting | drop | security-camera features; tracking for grasp is done in armd (`src/grasp.rs:417-445`) |
| ONVIF service and metadata | drop | needs RTSP; not used |
| Privacy blur (RGN) | drop | not used by the arm |
| Console `config_schema` zones/confidence | replace | confidence threshold and marker settings move to `vision.json` and `/api/mode`-adjacent settings; the gallery `config_schema` gains a "Vision" group (4.2) |

### 2.7 Process, memory and supervision

- One process `rebot-vision`, binary `/usr/bin/rebot-vision`, log `/var/log/rebot-vision.log`,
  pidfile `/var/run/rebot-vision.pid`, config `/userdata/rebot-teleop/vision.json` (camera intrinsics,
  marker family/size, filter, fps, model, last mode). CLI flags as in the two apps
  (`aruco-detector/main/main.cpp:140-276`), read from the config file by the init script.
- RAM: 180 MB device, armd 3.7 MB RSS. `rebot-vision` RSS is expected 15-30 MB plus ION for the model and VB
  pools outside RSS **(needs verification**: `/proc/<pid>/status` VmRSS, `/proc/cvitek/ion`,
  `/proc/cvitek/vb` per mode). ION carveout limit about 40 MB (recamera-app skill); one yolo11n model is 3.1 MB.
- If `rebot-vision` exits, armd keeps running and the arm is unaffected; armd's proxy returns 502
  (`src/api/proxy.rs:354-357`) and grasp ends with `DETECTION_UNAVAILABLE`. The init script wraps
  `rebot-vision` in a respawn loop with backoff (1, 2, 4 ... 30 s; gives up after 10 starts in 5 min);
  the loop's own pid goes into the pidfile.

---

## 3. armd side

### 3.1 Proxy and state

- New routes in `src/api/mod.rs` next to `:74-81`:
  - `GET/POST /api/v1/vision/mode` -> upstream `/api/mode` (use `forward`, `src/api/proxy.rs:310-335`,
    timeout margin as `get_detection_inference`, `:376-382`);
  - `POST/DELETE /api/v1/vision/mode/hold` -> `/api/mode/hold`;
  - `GET/POST /api/v1/vision/model` -> `/api/model` (POST timeout 10 s).
  - Same rules as the detection routes (`docs/detection_api.md`): pass-through status/body, 502 when
    unreachable, same-origin check on POST.
- Rename the upstream env var conceptually to the vision service but keep `ARMD_DETECTOR_URL`
  (`src/api/proxy.rs:144-195`) as the only name; no alias needed.
- Vision state cache: an `Arc<RwLock<VisionStatus>>` in the API layer, updated from every proxied mode
  response and by a 2 s background poll of `GET /api/mode` (only while a WS client is connected). Published:
  - observation: `vision: {"mode": str|null, "effective": [..], "available": bool, "age_ms": n}` merged at
    serialization in `rest::get_observation` (`src/api/rest.rs:218-223`) and the WS telemetry path; `null`
    fields when the service never answered (lite runtime);
  - schema: extend the existing `capabilities.vision` object (today `{"detection_proxy": true}`,
    `Capabilities` in `src/model.rs`) with `"modes": ["off","yolo","aruco","both"]`,
    `"endpoint": "/api/v1/vision/mode"` and `"trigger": true`, and bump `CAPABILITIES_VERSION` to 2
    (`rest::get_observation_schema`, `src/api/rest.rs:225-230`).
  The supervisor (`src/supervisor.rs:2550`) is not touched: vision state is not arm state and must not take
  the supervisor lock.
- Lite build: the routes exist and return 502 like the detection routes when no service listens.

### 3.2 Grasp

`src/grasp.rs:917-933` turns inference on for the duration. Change `DetectionSource`
(`src/grasp.rs:283-289`) to add `hold(pipelines, ttl) / release()`, and in `run`:

- before the orchestrator: `hold(["yolo"], ttl = 10 s)`, renewed every 3 s by the orchestrator loop;
  release in every outcome (replaces the enable/restore pair). The base mode is untouched, so a user in
  `aruco` gets `aruco ∪ yolo` = both pipelines for the grasp and `aruco` afterwards.
- hold rejected with 409 `yolo unavailable` -> `DETECTION_UNAVAILABLE` (existing code).
- Fallback for an upstream without `/api/mode/hold` (404, e.g. an old yolo-detector on a lite host): keep the
  current enable/restore logic.
- Optional, later: use `markers` from the same frame for the place location (tray marker) or table frame. Not
  in this change; the result shape already carries them per frame, so no API change is needed later.
- `parse_detections` stays as is; it reads `detections` only.

### 3.3 Cockpit (`src/api/index.html`)

- Vision mode selector in the video HUD next to Detect (`index.html:223` style block, toggle logic at
  `:2881-2904`): segmented `Off | YOLO | ArUco | Both`, reads/writes `/api/v1/vision/mode`, shows "N/A" when
  the route fails (as Detect does). Detect stays as the on/off toggle; selecting a mode other than Off also
  enables inference.
- Hand-eye calibration drawer (`calOpen`, `index.html:3720-3732`): on open, `POST /api/v1/vision/mode/hold`
  `{"holder":"cockpit-calib-<tabid>","pipelines":["aruco"],"ttl_ms":15000}`, renewed every 5 s while open;
  `DELETE` on close and on `pagehide`. Because a hold is a union, YOLO keeps running if it was on, and closing
  restores the previous state without the page remembering it. The TTL covers a closed tab.
- Calibration capture keeps using `/api/v1/detection/trigger` (`index.html:2953-3030`); markers are read from
  `markers` as today.
- Overlay: `parseDetections` already merges `markers` and boxes (`:2603-2640`); with the 2.3 change markers
  appear once. Add a test case with a `both` payload to `scripts/calib_selftest.mjs` or a new self-test.
- Grasp HUD hint (`gset.hint`, `index.html:1546`) changes from "Turn on Detect" to "Select YOLO or Both".

### 3.4 rebot-companion

- `snapshot.py:630-690` (armd detector proxy client): add `get_vision_mode()`, `set_vision_mode(mode)`,
  `hold(pipelines, ttl)` / `release()` on `/api/v1/vision/*`.
- `mcp_server.py`:
  - new tool `set_vision_mode(mode: off|yolo|aruco|both)` returning the `/api/mode` body;
  - `detect` (`mcp_server.py:475-488, 556`): new optional `pipelines` argument (default `["yolo"]`); it takes a
    short hold for the call so it works regardless of the user's base mode, and returns both
    `detections` and `markers`;
  - `get_state` includes `observation.vision`.
  - Instructions text (`build_instructions`, `:106`) mentions that the camera has modes and that `detect`
    selects what it needs.
- `camera_client.py:72` talks to `:8090` directly (`/api/trigger`, `markers`); it keeps working because the
  envelope keeps `markers`. Change its docstring only.
- Tests: `tests/test_mcp_server.py`, `tests/test_detections.py` gain a `both` payload.

---

## 4. Build and packaging

### 4.1 Build script

`scripts/build_deb.sh --flavor lite|recamera [VERSION]` exists since the kits split (default `recamera`).
Today `lite` builds `dist/armd-lite_<ver>_riscv64.tar.gz` (riscv64 musl only: `armd`, `LICENSE`,
`THIRD_PARTY_NOTICES.md`) and `recamera` builds the rebot-teleop deb from `kits/recamera-b601dm/packaging/`.
The table below is the target state; the extra lite targets and files are not implemented.

| Flavor | Steps | Output |
|---|---|---|
| `lite` | `cargo build --release --target <triple>` for each of `x86_64-unknown-linux-musl`, `aarch64-unknown-linux-musl`, `riscv64gc-unknown-linux-musl`; tarball with `armd`, `index.html` is embedded, URDF, README excerpt, a sample systemd unit | `dist/armd-<ver>-<triple>.tar.gz` (no deb, no gallery manifest) |
| `recamera` | current armd steps (`build_deb.sh:63-134`) + `scripts/build_vision.sh` (or `REBOT_VISION_BIN=<path>` to use a prebuilt binary) + staging of `rebot-vision`, init script, config default | `dist/rebot-teleop_<ver>_riscv64.deb` |

The lite flavour builds no deb for the reCamera: on a reCamera, running armd without the bundled vision
reintroduces the App Manager conflict, so the reCamera package is always the `recamera` flavour.

### 4.2 Package

- Name `rebot-teleop` (package name = app id = init script suffix, required by the console,
  `api_app.cpp:1392-1395`). Version **1.2.0**.
- `Depends:` empty (opkg). `Description` drops "object detection overlay" wording only if needed; it already
  lists detection.
- Files:
  - `/usr/bin/armd`, `/usr/local/bin/armd` symlink (as today);
  - `/usr/bin/rebot-vision` (~3-4 MB, yolo-detector binary is 3.1 MB, aruco-detector 0.86 MB);
  - `/etc/init.d/K92rebot-teleop` (starts both, 4.5);
  - `/usr/share/rebot-teleop/vision.json.default`, manifests and docs as today;
  - no model files.
- `postinst`:
  - copy manifest/docs to `/userdata/local/apps/` (as today, `debian/postinst:4-7`);
  - create `/userdata/rebot-teleop/{models,}` and seed `vision.json` from the default if missing;
  - `S92rebot-teleop`: create only when the console is absent (`[ ! -x /usr/share/supervisor/scripts/main.sh ]`);
    when the console is present, remove an existing `S92rebot-teleop` symlink (only if it is a symlink to our
    K92). Reason: with the console, `app_restore` starts the active app at boot (`main.sh:1424-1446`); an S92
    link plus a different active camera app starts two camera holders at boot. This is the same migration
    yolo-detector made (`solutions/yolo-detector/control/postinst`, "Gallery apps must NOT start at boot");
  - remove the S92 creation from `build_deb.sh:92` and `debian/postinst:11-12`.
  - postinst does not write `state.json` and does not activate the app; activation is the console's
    `appMgr/switch` (SenseCraft step, section 6, or the user in the console).
- Gallery manifest `rebot-teleop.json`:
  - `"requires": ["can"]` kept; `"models": []` kept (2.5);
  - remove `rtsp_url`; add `"debug_ws": {"port": 8001, "video_path": "/", "results_path": "/results"}` and
    `"stream_mode": "ws"` as yolo-detector's manifest has, so the console preview works
    **(needs verification** that the console reads these keys for native apps);
  - `name`/`description`: state that vision (YOLO detection, ArUco markers) is included and that no other
    camera app is needed; fix the "serial control for SO-ARM 101" wording to CAN as part of the same edit;
  - `config_schema`: new group `vision` with `default_mode` (select off/yolo/aruco/both), `fps` (number 1..15),
    `marker_family` (select), `marker_size_m` (number). Note: the console's `setConfig` restarts the active app
    (`restart_after_change`), i.e. armd too. Values that change at runtime (mode, model) are only offered in the
    cockpit, not in the schema.

### 4.3 Install paths and root space

- Root partition is about 92 % full (about 40 MB free). The console's `main.sh:1452-1462` comment says
  `/etc`, `/usr`, `/usr/local` are overlays with the upper dir on `/userdata`. If so, files that opkg installs
  under `/usr` consume `/userdata`, not the root partition **(needs verification:** `mount | grep overlay`,
  `df -h / /usr /userdata` before and after installing 1.2.0).
- Binaries stay in `/usr/bin` (about 8 MB total with armd 4.4 MB). Models, config, logs that grow
  (datasets already in `/userdata/datasets`) live under `/userdata`. If the overlay check fails (upper on the
  root partition), move `rebot-vision` to `/userdata/rebot-teleop/bin/` and keep only a symlink in `/usr/bin`.
- postinst refuses to continue (exit 1, message) if the free space on the filesystem holding `/usr/bin` is
  below 16 MB after unpacking.

### 4.4 C++ build: local and CI

Local (documented in README "Cross-compile" and `scripts/build_vision.sh`):

```sh
docker run --rm -v $HOME/project/recamera:/recamera -v $PWD:/src sophgo/tpuc_dev:v3.1 bash -c '
  export PATH=/recamera/host-tools/gcc/riscv64-linux-musl-x86_64/bin:$PATH
  export SG200X_SDK_PATH=/recamera/sg2002_recamera_emmc
  export SSCMA_ROOT=/recamera/sscma-example-sg200x      # at the commit in kits/recamera-b601dm/vision/sscma.lock
  cmake -S /src/vision -B /src/target/vision-riscv -DCMAKE_BUILD_TYPE=Release
  cmake --build /src/target/vision-riscv -j$(nproc)'
```

The existing `tpuc_recamera` container mounts `/home/harvest/project/recamera` at `/recamera`
(`docker inspect`); `build_vision.sh` accepts `DOCKER_CONTAINER=tpuc_recamera` to `docker exec` into it, or
runs a fresh `sophgo/tpuc_dev:v3.1` (public on Docker Hub, 7.8 GB) container.
Inputs for outside contributors: `sophgo/host-tools` (`gcc/riscv64-linux-musl-x86_64` only), the prebuilt
reCamera OS SDK (`reCamera_OS_SDK_x.x.x.tar.gz` from the reCamera-OS releases,
`sscma-example-sg200x/README.md:40-50`), and `sscma-example-sg200x` at the lock commit.

CI (`.github/workflows/ci.yml`):
- always: host build and run of `kits/recamera-b601dm/vision/tests` (plain g++/cmake, no SDK), like the cockpit self-tests job;
- `workflow_dispatch` only: a `build-recamera` job that downloads the SDK tarball and host-tools subset,
  caches them, and builds `rebot-vision` + the deb as an artifact. Enabled only after the SDK download URL and
  size are confirmed and the lock commit is public **(needs verification)**.
- Release: built locally with `build_deb.sh --flavor recamera`, then uploaded to the CDN path used by the
  catalog (`.../recamera_ecosystem/packages/rebot-teleop_1.2.0_riscv64.deb`) with its sha256.

### 4.5 Init script `K92rebot-teleop`

Based on the current script (`kits/recamera-b601dm/packaging/rootfs/etc/init.d/K92rebot-teleop`).

`start`:
1. If armd is running from our pidfile and `rebot-vision` is running from its pidfile: print "already running",
   **return 0** (today it returns 1, `:152-160`; `app_restore` at boot after `S92`, or a console `switch` after a
   boot start, then reports a failure).
2. If armd is not running: existing checks (`:163-166`), Wi-Fi, CAN, start armd (`:168-215`).
3. Camera: `stop_camera_holders` (section 5), then check that `:8001` and `:8090` are free
   (`netstat -ltn`); if not, log the holder and leave vision stopped, return 0 with a warning in the log
   (armd and the arm are usable without vision).
4. Start the `rebot-vision` respawn loop with `nice -n 10`, `LD_LIBRARY_PATH` as yolo-detector
   (`K92yolo-detector`: `/mnt/system/lib:/mnt/system/usr/lib:...`).
5. Liveness check of both (as `:197-211`). Whole start must finish within the console's 15 s
   (`main.sh:1367-1383`).

`stop`: TERM the vision loop and process first (pidfile identity check like `:255-262`), then the existing
armd stop (`:235-290`, up to 8 s). Vision stop takes < 1 s expected, so the total stays inside the console's 10 s
TERM-only stop (`main.sh:1385-1401`) **(needs verification** with the arm torqued: armd's own stop can take the
full 8 s).

`status`: running when armd runs (vision state printed but not part of the exit code; the arm is the app).

---

## 5. Migration on devices that have yolo-detector / aruco-detector

Rule: stop only the process that holds the camera; never disable, uninstall or rewrite other apps' scripts;
do not edit `state.json`.

- Console path (normal): the SenseCraft step or the user switches to `rebot-teleop`. `appMgr/switch` stops the
  active app (the camera app) and waits 2 s (`api_app.cpp:899-906`), then runs our `start`.
- Non-console path (boot via S92 on a device without the console, manual `K92rebot-teleop start`):
  `stop_camera_holders` iterates a fixed list of camera apps:
  `yolo-detector aruco-detector face-analysis face-recognition detection-blur ppocr-reader retail-vision
  qrcode-reader trigger-inspection-demo microduck-pose-qa fall-detection` (union of the lists in
  `K92yolo-detector:73-87` and `K92aruco-detector:56-69`). For each `/etc/init.d/[SK]??<app>` that exists **and**
  whose pidfile `/var/run/<app>.pid` points to a live process, run `<script> stop` and log
  `stopped camera holder <app>`. Scripts of apps that are not running are not invoked. Then `sleep 2`
  (VPSS release, as the console does). `sscma-node` / Node-RED: not stopped; if Node-RED mode is active the
  console refuses app operations (`api_app.cpp` Node-RED gate), and our start logs the busy port instead.
- The old `S92rebot-teleop` link is removed by postinst when the console is present (4.2).
- Fresh device without `state.json`: the console's boot fallback starts `DEFAULT_APP_ID=yolo-detector`
  (`main.sh:1229-1232, 1427-1441`) if installed. Activation through `appMgr/switch` writes `state.json`, so this
  only matters until the first activation.
- After migration, activating yolo-detector or aruco-detector in the console stops `rebot-teleop` (armd and
  vision). This is console behaviour and is documented in the package README, not worked around.

---

## 6. SenseCraft solution `rebot_agent_arm`

Current steps (`guide.md` on `feat/rebot-agent-arm`): 1 install rebot-teleop, 2 optional MCP, 3 optional
camera app + reboot, 4 verify.

New steps:

| # | Id | Type | Change |
|---|---|---|---|
| 1 | `install_teleop` | `recamera_console_app` | installs and activates `rebot-teleop` 1.2.0. Deployment Complete text: cockpit at `:8765`, video and YOLO/ArUco included. Troubleshooting adds "Video shows N/A: another camera app was started after this step; activate reBot Arm Teleop again in the console" |
| 2 | `install_mcp` | `script` (optional) | unchanged except the tool count (15 with `set_vision_mode`) and the `CAMERA_UNAVAILABLE` row ("vision service not running; check `/var/log/rebot-vision.log`") |
| 3 | `open_cockpit` | `web_dashboard` (verify) | unchanged steps; calibration text says the drawer switches the camera to ArUco by itself |

Removed: Step 3 `install_camera_app` and `devices/camera_app.yaml`, the reboot workaround (`guide.md:94-100`),
and the comment in `recamera_ecosystem/devices/apps/rebot_teleop.yaml:16-17` about switch stopping armd.
Both `guide.md` and `guide_zh.md` change; the preset intro (`guide.md:3`) drops "and a camera app".

Catalog (`recamera_ecosystem/devices/apps/rebot_teleop.yaml:33-44`): deb path
`rebot-teleop_1.2.0_riscv64.deb`, new sha256, description lists YOLO + ArUco. `solution.yaml`
`output_interfaces` :8001 entry description: "Camera H.264 stream from the reBot vision service"; add
`/api/v1/vision/mode` on :8765.

`aruco-detector` catalog entry (`recamera_ecosystem/devices/apps/aruco_detector.yaml`): keep. It is a
standalone gallery app for marker tracking without an arm (MQTT, RTSP, filters), and it stays useful to
users who do not own the arm. It is no longer referenced by `rebot_agent_arm`.

---

## 7. Licensing

This repo is MIT (`LICENSE`). Components that end up in the `rebot-vision` binary or source:

| Component | Licence | How used | Notice needed |
|---|---|---|---|
| sscma-example-sg200x app code (aruco-detector, yolo-detector main sources copied into `kits/recamera-b601dm/vision/main`) | Apache-2.0 (`sscma-example-sg200x/LICENSE`) | copied, modified | keep Apache-2.0 licence text for those files (`kits/recamera-b601dm/vision/LICENSE-APACHE` or in notices), mark modified files (Apache 4(b)), keep any copyright headers |
| `app_framework`, `debug_stream`, `rtsp_server`, `ha_mqtt`, `onvif_service`, `sophgo` components | Apache-2.0 (repo licence; no per-component LICENSE files found) | external, statically compiled into the binary | Apache-2.0 text + copyright line in `THIRD_PARTY_NOTICES.md` (binary distribution) |
| `sscma-micro` | Apache-2.0 (`components/sscma-micro/sscma-micro/LICENSE`) | external, compiled in | same |
| `sscma-micro/3rdparty/eigen` (header-only, included by `sscma-micro/CMakeLists.txt:6`) | MPL-2.0 (most files) | headers compiled in | MPL-2.0 notice and where the source is available (upstream Eigen, unmodified) |
| `sscma-micro/3rdparty/JPEGENC` | Apache-2.0, Copyright 2020 BitBank Software | compiled in if linked **(needs verification** with the link map) | Apache notice |
| `sscma-micro/3rdparty/json`, `components/json` (nlohmann) | MIT | compiled in | MIT text |
| `components/apriltag` incl. `aruco/` dictionaries | BSD-2-Clause, Copyright 2013-2016 The Regents of the University of Michigan (`apriltag.c:1-28`) | compiled in | BSD-2 text (binary redistribution clause 2) |
| `libwebsockets` (private build, `components/libwebsockets/fetch_and_build.sh`) | MIT | linked (static or shared, **needs verification**) | MIT text; if shared and shipped, ship the .so's notice |
| mosquitto (via `ha_mqtt`) | EPL-2.0 / EDL-1.0 dual | dynamically linked against the device's copy, not shipped | none for us if not shipped; state in notices that it is a runtime dependency |
| Sophgo MPI / cviruntime / TPU SDK libs (`/mnt/system/lib`) | vendor SDK licence **(needs verification)** | dynamically linked against the device's copies, not shipped | none if not shipped; do not copy these libs into the deb |
| YOLO11n / YOLO26n weights (Ultralytics) | AGPL-3.0 for Ultralytics models, unless an enterprise licence applies **(needs verification** for the Seeed-converted cvimodels) | data file loaded at runtime from the device, not shipped | no file in the repo or deb; README states the model licence and that the user's device copy is used |

Conflicts with MIT: Apache-2.0, BSD-2, MIT and MPL-2.0 (file-level, unmodified) are compatible with an MIT
repository as long as the notices are kept; the repo stays MIT and the `kits/recamera-b601dm/vision/` Apache files keep their
licence (the existing notice file already handles a non-MIT file the same way, the CERN-OHL-W URDF).
The only open conflict is the YOLO weights (AGPL-3.0); avoiding redistribution keeps them out of this repo.
Legal review is needed before any model file is bundled.

`THIRD_PARTY_NOTICES.md` gets a section "rebot-vision (reCamera runtime)" listing the rows above with licence
texts or links; the deb installs it as `/usr/share/doc/rebot-teleop/THIRD_PARTY_NOTICES.md`.

---

## 8. Test plan

### 8.1 Host-side unit tests

C++ (`kits/recamera-b601dm/vision/tests`, no SDK; pattern of `aruco-detector/tests/*.cpp` and `yolo-detector/tests`):
- `vision_control`: mode transitions; `off` forces `enabled=false`; enable with empty `effective` restores
  `last_active_mode`; holds (union, renew, TTL expiry, max 8, release unknown holder); trigger with empty
  effective; long-poll wake-ups on mode change and shutdown (port of `inference_control_test.cpp` from both apps).
- HTTP routing: every route, 400/404/409 bodies, query parsing (`after`, `wait_ms` clamp).
- `result_json`: envelope for each mode; `detections`/`markers` always present; letterbox content-rect mapping
  (inference -> content -> display) for 640x640 / 1280x720, and for 640x360 (no bars).
- `marker_filter`, `camera_model`, `capture_time`: existing tests copied.

Rust:
- `src/api/proxy.rs` tests for the new routes (parse/forward as `:421-464`).
- grasp: `tests/test_grasp` simulated `DetectionSource` with `hold/release`, release on every outcome,
  404 fallback to enable/restore, 409 -> `DETECTION_UNAVAILABLE`.
- observation `vision` fields present and `null` without a service; schema `capabilities.vision.modes`
  present (version 2).

JS: cockpit self-test for a `both` payload (no duplicate marker boxes) and for the calibration hold
open/renew/close sequence (fake fetch).

Python (rebot-companion): `set_vision_mode`, `detect(pipelines=...)`, `both` payload parsing.

### 8.2 Real-device checklist (reCamera with B4 CAN board, arm attached)

Record for each run: OS version, console version, `cv181x_vpss.ko` md5 (stock or forked), `state.json`
`active_app`, `ps` before and after (the device may be shared with other sessions).

1. Install 1.2.0 over 1.1.0 with yolo-detector active: `df -h / /usr /userdata` before/after; overlay check;
   S92 link removed; yolo-detector stopped by the console switch, still installed (`opkg list-installed`).
2. Same with no console activation (manual `K92rebot-teleop start`) and aruco-detector running: only the running
   holder is stopped; log line present; other apps' scripts untouched (md5 of `/etc/init.d/*` before/after).
3. Reboot: only rebot-teleop processes hold the camera; `app_restore` start returns 0.
4. App Manager: `switch` to rebot-teleop returns success; console shows it running; no other app stopped
   except the previous active one; `setModel` returns "app has no switchable models".
5. Mode switch time: `POST /api/mode` then poll `latest` until `mode` and pipelines match, 20 runs per
   transition (off->yolo, yolo->aruco, aruco->both, both->off, idle->both). Target <= 1.1 s from idle,
   <= 1 frame between active modes.
6. CPU/RAM per mode (2.4 procedure): both at 5/10/15 fps with the arm torqued; yolo, aruco, off; RSS,
   `/proc/cvitek/ion`, `/proc/cvitek/vb`; armd loop jitter. Pick the default fps.
7. Idle ISP: CPU in `off` with stream at 15 vs 10 fps, and aruco-detector 0.1.3 idle (30 fps) as reference.
8. Cockpit overlay: YOLO boxes and ArUco markers aligned with objects in `yolo`, `aruco`, `both`; screenshots
   at desktop and 375x812.
9. Letterbox: a box drawn around a known object at the image bottom; compare `detections` y with the visible
   position, before (1.1.0 + yolo-detector) and after.
10. Hand-eye calibration end to end (eye-in-hand, 9 poses): drawer open switches effective to include aruco,
    close restores; solve RMSE comparable to the aruco-detector run (same intrinsics in `vision.json`).
11. Grasp with YOLO from base mode `aruco` and `off`: hold visible in `GET /api/mode` during the grasp,
    released after success, failure and cancel; base mode unchanged.
12. Model switch yolo11n -> yolo26n via `/api/model`: time, no armd restart, arm stays enabled.
13. Kill `rebot-vision`: arm unaffected, respawn within backoff, cockpit recovers video.
14. Console stop with the arm torqued: completes within 10 s or armd refuses as today; vision stopped.

### 8.3 Commit split

rebot-armd (one subject per commit):
1. `docs: add vision runtime spec` (this file).
2. `feat(vision): import aruco/yolo app sources from sscma-example-sg200x` (unchanged copies + notices,
   `sscma.lock`).
3. `feat(vision): vision_control with modes and holds` + host tests.
4. `feat(vision): single-camera pipeline with yolo/aruco/both stages and unified result` + tests.
5. `build(vision): build_vision.sh and CMake against pinned SSCMA_ROOT`.
6. `feat(api): proxy /api/v1/vision/* and publish vision state in observation/schema`.
7. `feat(grasp): hold yolo for the grasp instead of toggling inference`.
8. `feat(cockpit): vision mode selector and aruco hold in the calibration drawer`.
9. `feat(packaging): rebot-teleop 1.2.0 with rebot-vision, init script, manifest`.
10. `build: build_deb.sh --flavor lite|recamera` (done: riscv64 lite tarball; other targets open).
11. `ci: host tests for kits/recamera-b601dm/vision/`.
12. `docs: README, detection_api.md, THIRD_PARTY_NOTICES.md for the reCamera runtime`.

rebot-companion: `feat(mcp): set_vision_mode tool and detect pipelines` (separate repo, separate PR).
sensecraft-solutions: `feat(rebot_agent_arm): drop camera app step for rebot-teleop 1.2.0` and
`chore(recamera_ecosystem): rebot-teleop 1.2.0 package`.

---

## 9. Open questions

1. Resolved 2026-11-10: the pinned commit is published on the fork's `main`. Open: which of the fork's
   changes to upstream to `Seeed-Studio/sscma-example-sg200x`.
2. CPU budget numbers (70 % total, armd jitter) and the default fps follow from 8.2 item 6.
3. Is the CH0 640x640 frame letterboxed (2.2)? If not, the ArUco crop and the coordinate change in 2.3 change.
4. Can the sscma-micro engine reload a model in-process (2.5), or does model switching restart `rebot-vision`?
5. YOLO weight licence for the Seeed-converted cvimodels, and whether the console still ships the preset model.
6. Is `/usr` an overlay on `/userdata` on current OS images (4.3)?
7. Should a `lite` riscv64 deb exist for reCamera users without the camera module? This spec says no.
