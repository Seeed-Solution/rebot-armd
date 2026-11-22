# rebot-vision

The reCamera kit's camera process (C++, SG2002). One process owns the camera and serves:

- H.264 video over WebSocket on `:8001` (`/`; each binary message is one access unit followed by
  an 8-byte little-endian Unix-ms timestamp), the overlay stream `/results` and `/snapshot.jpg`;
- the perception provider API v2 on `:8090` (`/api/inference`, `/api/results/latest`,
  `/api/trigger`, `/api/mode`, `/api/subscribe`, `/api/subscriptions`, `/api/depth`,
  `/api/frame/gray`), which armd proxies under `/api/v1/detection/*` and `/api/v1/vision/*`
  (except loopback-only `/api/frame/gray` and `/api/calibration/intrinsics`). API and result shapes:
  [docs/detection_api.md](../../../docs/detection_api.md).
- `GET /api/frame/gray?instance_id=<id>&frame_id=<u32>` (loopback-only): exact retrieval of a
  640x360 8-bit grayscale frame produced by a loopback `POST /api/trigger` naming `"gray"`
  (`main/gray_frame.{h,cpp}`). Converted from the validated CH0 RGB888 visible band
  (`(77*R + 150*G + 29*B) >> 8`, full-range 0..255) into an owned buffer before YOLO/ArUco/depth
  run, published atomically with the trigger result into a 4-entry FIFO ring, and sent as a
  segmented `RGF1` binary payload (`application/octet-stream`, `Cache-Control: no-store`, bounded
  concurrent transfers with an absolute send deadline). Remote peers see `available.gray: false` and
  receive `403 FORBIDDEN`. Subscriptions reject `"gray"` (`400`).
- `GET /api/depth`: one depth inference on the next frame (a depth trigger, so it shares the frame
  thread's single depth stage), returned as a 384x224 u16 millimetre map with a JSON header
  (`main/depth_map.{h,cpp}`). One request at a time (409 `depth busy`), at most one per second
  (429 with `Retry-After`); no continuous depth mode. The values are the model's estimate, not a
  measurement: on the B601-DM unit (2026-11-21) the median absolute error against the calibrated
  table plane was 89 mm at 0.32-0.45 m (see docs/detection_api.md "Depth map").

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
  `"pipelines": []` is a camera hold (`GET /api/mode` `"camera_hold": true`): no inference, the camera
  gate stays open so a trigger is served by the next frame; it expires like any subscription.
  Subscription bodies carry `instance_id` (this process). `DELETE /api/subscribe/<id>?client=<c>&instance_id=<i>`
  deletes only the subscription of client `c` made by instance `i`: 409 `NOT_OWNER` for another client's,
  409 `INSTANCE_CHANGED` after a restart (ids start again at `s1`), 404 when unknown. Without the query it
  deletes by id (cockpit overlay).
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
- Measured on the device with 1.3.1 (perception API 2, arm torque off, two runs of 15 s
  windows, on-device node script against `127.0.0.1:8090`):

  | state | whole-chip CPU | rebot-vision CPU | results/s |
  |---|---|---|---|
  | standby (no subscription) | 20-26 % | 6 % | 0 |
  | subscription yolo @ 5 | 34 % | 16 % | 5.1-5.2 |
  | subscription aruco @ 5 | 53 % | 33 % | 4.9-5.0 |
  | subscriptions yolo @ 5 + aruco @ 5 (union) | 54-55 % | 39-40 % | 5.3-5.4 |

  The standby whole-chip figure moves between runs with other system services; the rebot-vision
  share does not. Subscription expiry: 10.01-10.02 s after the last keep-alive with `ttl_ms` 10000.

  Trigger from standby, end to end (5 per set, ms):

  | pipelines | end to end | pipeline ms |
  |---|---|---|
  | `["yolo"]` | 101-607 | yolo 45-48 (one 177) |
  | `["aruco"]` | 108-170 | aruco 45-71 |
  | `["yolo","aruco"]` | 124-221 | yolo 45-48, aruco 43-59 |

  Open issues seen in these runs, investigated afterwards (1.3.1 and 1.3.0 binaries swapped on
  the device and interleaved, 4 rounds x 10 triggers per set each; then a build with the trigger
  `timing` object, 5 rounds of 15 per set; arm torque off; all on-device node scripts, 1.2-1.6 s between
  triggers):
  - YOLO trigger tail (286-607 ms): not a 1.3.1 regression and mostly not rebot-vision time.
    The first trigger a node process sends takes 523-611 ms with both binaries (8 of 8 runs),
    the rest are alike:

    | binary | first trigger of the script | other YOLO triggers, ms p50 / p95 / max | > 250 ms | ArUco triggers, ms p50 / p95 / max |
    |---|---|---|---|---|
    | 1.3.0 | 542, 530, 548, 531 | 149 / 240 / 500 (n 36) | 2 | 148 / 260 / 790 (n 40) |
    | 1.3.1 | 611, 556, 523, 545 | 135 / 211 / 408 (n 36) | 1 | 128 / 194 / 441 (n 40) |

    The `timing` timestamps put the first trigger's extra 400 ms in the client: 204-206 ms from
    the client's request to the server's `accept`, 101-126 ms from `accept` to the end of the
    request, 123-139 ms from the reply to the client having it; gate to frame 36-65 ms and
    YOLO 48 ms as in every other trigger. With three `GET /api/mode` requests first, the
    first trigger took 181 and 123 ms. The 1.3.0 figures of 96-137 ms came from a client
    that was already warm. The triggers above 250 ms that remain (8 of 147 with `timing`)
    are a pipeline preempted on the single core (frame to publish 141-597 ms; YOLO is 47 ms
    otherwise) or a client delay (client to `accept` 56-119 ms); none waited past a frame
    (`skipped` 0 in all 150). Gate to frame: p50 43-56 ms, max 107 ms (one CH0 frame is
    67 ms).
  - ArUco slower in a trigger: only the `clusters` stage grows. Same scene (7-8 candidate quads;
    the 37 ms above was a scene with 4), 50 frames per case, ms p50:

    | case | ArUco ms p50 / p95 | gray | threshold | unionfind | clusters | quad fit | refine |
    |---|---|---|---|---|---|---|---|
    | subscription @ 1 (frames 1 s apart, gate 2 fps) | 56.4 / 191.6 | 4.8 | 2.9 | 5.9 | 14.8 | 21.0 | 3.6 |
    | subscription @ 2 | 53.3 / 59.8 | 4.9 | 2.8 | 6.0 | 14.3 | 20.2 | 3.6 |
    | subscription @ 5 | 55.9 / 73.2 | 4.8 | 2.8 | 6.2 | 14.4 | 20.9 | 4.0 |
    | subscription @ 10 (gate 30 fps, as during a trigger) | 55.5 / 59.7 | 4.9 | 2.8 | 6.2 | 14.5 | 20.9 | 3.6 |
    | trigger from standby | 61.2 / 204.1 | 4.9 | 2.8 | 5.9 | 19.4 | 21.2 | 4.0 |
    | trigger from standby, client on another host (30) | 63.4 / 67.0 | 4.9 | 2.9 | 5.9 | 20.2 | 22.8 | 4.0 |

    Ruled out: an idle gap before the frame (subscription @ 1 has a 1 s gap and no growth), the
    open camera gate's copies (subscription @ 10 runs with the same gate), the measuring client
    on the device (same with the client on another host), and the trigger skipping the patched
    path (`ArucoStage::run` takes the same detector path; `full_scan` only turns ROI tracking
    off, which is off by default). The cause is not found: something specific to the trigger
    frame costs about 5 ms in the gradient-cluster pass. It is not the standby state: with an
    aruco subscription @ 1 running and 30 triggers in between (same scene, median 6 quads), the
    trigger frames gave clusters p50 19.3 ms (p90 21.9) against 14.0 ms (p90 18.4) for the 56
    subscription frames. The per-frame total was the same in that run (p50 51.7 ms trigger,
    53.3 ms subscription), so the cost is small; the cause is still open.
  - A shell loop polling the API with `curl` every 100 ms on the device raises ArUco wall time
    to about 170 ms (one core); measure with process CPU ticks or a low-overhead script, and
    warm a node client up before timing its first request.

  `POST /api/trigger` returns a `timing` object (`mono_ms`, CLOCK_MONOTONIC, the clock of
  node's `process.hrtime`): `accept_ms` (connection accepted), `read_ms` (request read),
  `request_ms` (trigger registered), `gate_ms` (camera gate opened), `wait_ms` (frame thread
  started waiting for the serving frame), `grab_ms` (frame retrieved), `frame_ms`, `done_ms`
  (pipelines done, result published), `wake_ms` (request thread woke), `reply_ms` (reply
  built) and `skipped` (frames planned after the request that did not serve it).

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
- Stage split on the device (`pipelines.aruco.stages`; `vision.json` `aruco_stats_every` logs
  p50/p95 per stage). reCamera, aruco subscription at 5 fps, 200 frames per run, indoor desk in
  dim light with no marker in view (4 candidate quads per frame); ms p50 / p95. Before: the
  1.3.0 detector code with stage timing (220 frames). After: the 1.3.1 defaults, mean of two
  runs.

  | stage | before | after |
  |---|---|---|
  | gray | 4.8 / 16.6 | 4.8 / 26.1 |
  | decimate | 1.3 / 4.6 | 1.3 / 1.9 |
  | threshold | 2.7 / 6.0 | 2.8 / 6.1 |
  | unionfind | 6.3 / 8.4 | 5.3 / 9.5 |
  | clusters | 23.9 / 27.7 | 13.2 / 14.6 |
  | quad fit | 5.5 / 9.0 | 7.2 / 12.9 |
  | refine | 2.4 / 5.0 | 1.2 / 3.1 |
  | decode | 0.2 / 0.5 | 0.2 / 0.4 |
  | other (reconcile, frees) | 1.1 / 1.5 | 0.1 / 0.2 |
  | filter, pose (no marker) | 0.0 / 0.1 | 0.0 / 0.0 |
  | total | 50.5 / 69.9 | 37.1 / 67.7 |

  The p95 values include preemption: rebot-vision runs at nice 10 next to armd on one core.
- Per option, same scene, two interleaved runs of 200 frames each; each row adds its option to
  the rows above (`vision.json` keys; ArUco ms p50, whole-chip CPU and rebot-vision CPU at
  aruco@5; host = `aruco_bench`, 500 synthetic scenes; device bench = `aruco_bench`
  cross-compiled for the device, 60 synthetic scenes):

  | `vision.json` | default | ArUco ms p50 | whole-chip CPU | vision CPU | host / device bench |
  |---|---|---|---|---|---|
  | all below off (1.3.0 code paths) | - | 51.2 / 55.0 | 59.4 / 59.2 % | 30.1 / 30.6 % | - |
  | `lazy_clusters`: neighbour colours before union-find lookups | on | 51.9 / 53.7 (clusters 25.3 -> 23.8) | 59.5 / 60.6 % | 28.8 / 29.8 % | same markers and corners |
  | `reuse_buffers`: union-find, cluster map and point arrays kept between frames, one cluster pass | on | 38.9 / 39.3 | 53.1 / 55.3 % | 23.4 / 23.6 % | same markers and corners; heap calls 1147 -> 197 per device frame |
  | `size_gate`: quads below `min_side` - 2 px dropped before refine | on | 36.7 / 37.5 | 51.8 / 53.3 % | 23.0 / 23.2 % | recall same in every size bucket, corner error same, false positives 30 -> 27; device bench 76.1 -> 64.9 ms |
  | `adaptive_decimate` 3 (markers >= `adaptive_min_side` 40 px) | 0 (off) | 37.7 / 38.4 (no marker: not used) | 51.9 / 53.9 % | 22.8 / 23.6 % | device bench 73.9 -> 64.2 ms, recall 68.0 -> 67.4 % |
  | `roi_tracking` (subscriptions; triggers scan the whole image) | off | as above | | | device bench steady state 71.5 -> 54.9 ms, same recall; with a new scene every 3 frames recall 68.0 -> 63.0 % |

  `adaptive_decimate` and `roi_tracking` act only while markers are in view and are off by
  default: a new marker that is small (adaptive) or outside the tracked box (ROI) is found on the
  next full scan, up to 10 frames later (2 s at 5 fps). Decimate 3 on every frame (device bench
  64.9 -> 44.8 ms) loses markers below 20 px (23/41 -> 11/41) and low-contrast markers (20.8 ->
  8.3 %), hence only the adaptive form. `refine_edges` stays on (corner accuracy for hand-eye
  calibration).
- Host benchmark (`tests/aruco_bench.cpp`, built when `SSCMA_ROOT` is set; ctest `aruco_bench`
  checks recall, false positives, corner error and pose against the 0.1.3 settings, ctest
  `aruco_decimate` checks the default decimate against 1.0: recall of markers >= 40 px at normal
  contrast within 3 points, corner error within 0.15 px; ctest `aruco_same` checks that
  `lazy_clusters` and `reuse_buffers` report the same markers and corners as the upstream code;
  `--rows` compares variants, `--pgm` times gray frames saved from the device):

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
| `main/detector.*` | `yolo-detector/main/detector.*`, updated for SegModel and ObbModel auto-detection |
| `main/mask_geom.*` | new (PCA moments, minimum cross section, crop mask decode) |
| `main/seg_model.*` | new (SegModel : ma::model::Segmentor, anchor indexing, budget-capped mask decode) |
| `main/obb_geom.*` | new (letterbox-safe rotated quad geometry, ProbIoU rotated NMS) |
| `main/obb_model.*` | new (`ObbModel : ma::model::Detector`, Layout A FP32 & Layout B 9-output INT8/FP32 OBB decoder) |
| `main/marker_filter.*`, `camera_model.*`, `capture_time.h` | `aruco-detector/main/`, unchanged |
| `main/result_json.*` | new (letterbox geometry, detections / markers JSON) |
| `apriltag_patched/` | `components/apriltag` `apriltag.c`, `apriltag_quad_thresh.c` (BSD-2-Clause), modified: stage time stamps, cluster step options, size gate (see each file's header); `apriltag.cmake` builds them in place of the SDK's files |

Licence: these files stay under Apache-2.0 (`LICENSE-APACHE`); see `THIRD_PARTY_NOTICES.md`.

Not taken from the source apps: MQTT and Home Assistant discovery, RTSP (`:8554`), ONVIF and its
metadata, person tracking with dwell and zone / line counting, privacy blur, the console-managed
`config.json` (zones, lines). `ha_mqtt`, `rtsp_server` and `onvif_service` are still linked because
`app_framework` requires them, but nothing starts them.

## Build

`scripts/build_vision.sh` cross-compiles against an `sscma-example-sg200x` checkout used as an SDK
(`SSCMA_ROOT`, at the commit in `sscma.lock`; CMake stops on another commit unless
`REBOT_VISION_ALLOW_UNPINNED=1`), the reCamera OS SDK (`SG200X_SDK_PATH`) and the musl toolchain
from sophgo host-tools, inside a running `sophgo/tpuc_dev` container that mounts those under
`/recamera`.

If you only need the deb, skip this: download `rebot-vision-<version>-riscv64` and `SHA256SUMS`
from the [GitHub releases](https://github.com/Seeed-Solution/rebot-armd/releases) and pass the
binary with `REBOT_VISION_BIN=<binary> scripts/build_deb.sh --flavor recamera` (see the root
README, "Cross-compile for reCamera").

From source, all inputs public (run from a work directory next to the `rebot-armd` checkout):

```sh
# 1. Inputs
git clone --depth 1 https://github.com/sophgo/host-tools              # 9.3 GB checkout
curl -LO https://github.com/Seeed-Studio/reCamera-OS/releases/download/0.2.4/sg2002_reCamera_0.2.4_emmc_sdk.tar.gz  # 349 MB
echo "1c0b5fe3d43c24f3e56eb5ba89bc73e93a07d688282500969d5a2603efcd94f3  sg2002_reCamera_0.2.4_emmc_sdk.tar.gz" | sha256sum -c
tar -xzf sg2002_reCamera_0.2.4_emmc_sdk.tar.gz         # -> sg2002_recamera_emmc/ (1.2 GB)
git clone https://github.com/suharvest/sscma-example-sg200x
git -C sscma-example-sg200x checkout \
   $(sed -n 's/^commit=//p' rebot-armd/kits/recamera-b601dm/vision/sscma.lock)
git -C sscma-example-sg200x submodule update --init

# 2. Build container (sophgo/tpuc_dev is amd64 only, 7.8 GB; on an arm64 host it runs
#    through an x86_64 qemu-user binfmt handler, e.g. Debian/Ubuntu qemu-user-static)
docker run -d --name rebot_vision_build --platform linux/amd64 \
  -v $PWD/host-tools:/recamera/host-tools \
  -v $PWD/sg2002_recamera_emmc:/recamera/sg2002_recamera_emmc \
  -v $PWD/sscma-example-sg200x:/recamera/sscma-example-sg200x \
  sophgo/tpuc_dev:v3.1 sleep infinity

# 3. libwebsockets, once per sscma-example-sg200x clone (-> components/libwebsockets/prebuilt)
docker exec rebot_vision_build bash -c '
  export SG200X_SDK_PATH=/recamera/sg2002_recamera_emmc
  export PATH=/recamera/host-tools/gcc/riscv64-linux-musl-x86_64/bin:$PATH
  /recamera/sscma-example-sg200x/components/libwebsockets/fetch_and_build.sh'

# 4. rebot-vision, then the deb (from the rebot-armd checkout)
cd rebot-armd
RECAMERA_DOCKER=rebot_vision_build scripts/build_vision.sh   # -> target/vision-riscv/rebot-vision
RECAMERA_DOCKER=rebot_vision_build RECAMERA_TOOLCHAIN=$PWD/../host-tools/gcc/riscv64-linux-musl-x86_64 \
  scripts/build_deb.sh --flavor recamera                     # -> dist/rebot-teleop_<version>_riscv64.deb
docker rm -f rebot_vision_build
```

Notes:

- The CMake configure step clones cJSON and Eigen from GitHub (SSCMA-Micro fetch scripts), so the
  container needs network access on the first build.
- The container runs as root, so `components/libwebsockets/prebuilt` and the files the fetch
  scripts write in the `sscma-example-sg200x` checkout are owned by root on the host.
- Tested with reCamera OS SDK 0.2.4 (`sg2002_reCamera_0.2.4_emmc_sdk.tar.gz`, the release marked
  Latest) and `sophgo/tpuc_dev:v3.1` on an arm64 host: the binary was byte-identical to the v1.3.4
  release asset `rebot-vision-1.3.4-riscv64`. The `reCamera_OS_SDK_x.x.x.tar.gz` name in the
  sscma-example-sg200x README does not match the current release assets, which are named
  `sg2002_reCamera_<version>_emmc_sdk.tar.gz`. The release's `sg2002_recamera_emmc_md5sum.txt` does
  not list the SDK tarball; the SHA-256 above is the asset digest GitHub shows on the release page.

`RECAMERA_ROOT`, `RECAMERA_GCC`, `SG200X_SDK_PATH`, `SSCMA_ROOT` override the paths inside the
container (defaults: `/recamera`, `/recamera/host-tools/gcc/riscv64-linux-musl-x86_64/bin`,
`/recamera/sg2002_recamera_emmc`, `/recamera/sscma-example-sg200x`). `REBOT_VISION_BIN=<binary>`
makes `build_deb.sh` use a prebuilt binary instead.

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
File-only ArUco keys (see "ArUco cost"): `lazy_clusters`, `reuse_buffers`, `size_gate` (default
true), `adaptive_decimate` (default 0 = off), `adaptive_min_side` (40), `roi_tracking` (false),
`aruco_stats_every` (200, 0 = off).

Intrinsics (`fx`..`k3`, `marker_size`) can also be written at runtime with
`PUT /api/calibration/intrinsics` from a loopback peer (armd's calibration apply): the active
config file is rewritten atomically with the other keys kept, and the ArUco stage switches at the
next frame. `GET /api/calibration/intrinsics` and `/api/mode` `intrinsics` report the values and a
`revision` (1 at process start, not persisted). Intrinsics given on the command line refuse the PUT
(409 `CLI_OVERRIDE`). See [detection_api.md](../../../docs/detection_api.md#camera-intrinsics--相机内参).

YOLO model: not in the repository or the deb. The first existing file of `vision.json` `model`,
`/userdata/rebot-teleop/models/yolo11n.cvimodel`,
`/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_detection_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_cv181x_int8.cvimodel`,
`/userdata/rebot-teleop/models/yolo11n-obb.cvimodel`,
`/userdata/local/models/yolo11n_obb_cv181x_int8_fuse_qout.cvimodel`,
`/userdata/local/models/yolo11n_obb_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo26n_obb_cv181x_int8.cvimodel`,
`/usr/share/supervisor/models/yolo11n_segment_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_segment_cv181x_int8.cvimodel`,
`/userdata/local/models/yolo11n_seg_384x640_cv181x_int8_fuse.cvimodel` is loaded. Without a model `yolo` and `both`
answer 409 and ArUco works.

Model downloads: the official `yolo11n` detect and segment models ship with reCamera OS in
`/usr/share/supervisor/models`. The other models rebot-vision uses are listed in
[`models/models.tsv`](models/models.tsv) (file name, download URL, md5, license).
`scripts/fetch_models.sh [dir] [kind ...]` downloads them and checks the md5; the default directory
`/userdata/local/models` is one rebot-vision searches. On a PC: `scripts/fetch_models.sh ./models`, then copy
the files to `/userdata/local/models/` on the device.

Switching between the models: with no `models` key in `vision.json` (the default file has none), rebot-vision builds
its model catalogue at startup from the model files present on the unit: the reCamera OS detect and segment models,
and the `models.tsv` models in `/userdata/local/models/` (or `/userdata/rebot-teleop/models/`). Each entry is named
by its file stem, for example `yolo11n_obb_cv181x_int8_fuse_qout`, carries the measured `ion_mb`, and the OBB entry
carries the DOTAv1 class names. After downloading models, restart rebot-teleop with the arm idle and torque off
(`/etc/init.d/K92rebot-teleop restart`, which also restarts armd) so the catalogue picks them up. The startup YOLO model is still the first existing default path above (the OS detect
model on a stock unit). Switch with `POST /api/models` on `:8090`, for example
`{"detector": "yolo11n_seg_384x640_cv181x_int8_fuse", "depth": "zipdepth_metric_384x224_bf16_rgb"}`, or a grasp
task's `models`, or set `vision.json` `"model"` to the file path. A `models` object in `vision.json` replaces the
built-in list ([detection_api.md](../../../docs/detection_api.md#model-switching--模型切换)). The ZipDepth metric model is distilled from
Depth-Anything-V2-Metric-Indoor-Large, whose weights are licensed CC-BY-NC-4.0; check that license before
commercial use.

Model options: command-line `--model PATH`, `--model-kind auto|detect|segment|obb` (default: auto), `--mask on|off` (default: off; decode instance segmentation masks when using a segmentation model), `--mask-max-boxes N` (1..20, default 3; max boxes for mask decoding), and `vision.json` keys `"model"`, `"model_kind": "auto"|"detect"|"segment"|"obb"`, `"mask": true|false`, and `"mask_max_boxes": 1..20`.
When a 2-output YOLO11-seg model is loaded (e.g. `yolo11n_segment_cv181x_int8.cvimodel`), `rebot-vision` auto-detects `model.kind: "segment"`. When `--mask on` (or `vision.json` `"mask": true`), it decodes cropped instance masks for up to `mask_max_boxes` detections within a 160x160 cell budget and computes mask orientation, confidence, and grasp width.
A YOLO11-seg model cut before its head decode (10 `fp32` NCHW outputs: per stride 8 / 16 / 32 a box tensor `[1, 64, H, W]` (DFL 4 x 16), a class-logit tensor `[1, nc, H, W]` and a mask-coefficient tensor `[1, 32, H, W]`, plus the prototypes `[1, 32, in_h/4, in_w/4]`; e.g. `yolo11n_seg_384x640_cv181x_int8_fuse.cvimodel`, input `uint8 [1, 384, 640, 3]`) is also `model.kind: "segment"` (auto-detected from the output shapes, or `--model-kind segment`). The decoder is `main/seg_heads.*` (host test `tests/seg_heads_test.cpp`), the model wrapper `main/seg_heads_model.*`. The model input is filled from the visible 640x360 content of the 640x640 frame: rows copied to rows 12..371 of the 640x384 input, padding 114 (Ultralytics LetterBox, as the calibration images); detections are normalised like the other models, and the mask fields (`angle_deg`, `angle_conf`, `grasp_width_px`, `mask_focus`) come from the same crop decode and cross-section. Decode order: per scale the best class logit per cell (class planes read in order) is compared with logit(`conf_threshold`), the DFL box and the 32 coefficients are read only for cells that pass, then class-aware NMS (IoU 0.45), then masks for at most `mask_max_boxes` boxes. On the saved device outputs of `real_0` the prefiltered decode gives the same 25 candidates as the decode of every cell (host: 0.14 ms vs 1.72 ms). On the device (2026-11-17, 30 triggers per run, desk scene): yolo pipeline p50 67-74 ms vs 107 ms for the 640x640 model; decode after inference (excluding masks) p50 7.7-9.9 ms, p95 24.7-60.0 ms; mask stage p50 24.1 ms (19.4 ms for the 640x640 model). ION: 34.31 MB in total with this model alone (39.90 MB with the 640x640 model), 54.29 MB with ZipDepth metric; catalogue `ion_mb` 13.1 (ZipDepth 20.0). Without a catalogue entry the startup value is the `segment` default 19.5 MB. Raw data: docs/detection_api.md catalogue note. At `conf_threshold` 0.25 this model also reports a clear jar in the test scene as `cup` 0.40 that the 640x640 model does not report; the FP32 ONNX export gives the same box at 0.43 (a model output, not a decode error).
When an oriented bounding box (OBB) model is loaded, `rebot-vision` auto-detects (or forces via `--model-kind obb`) `model.kind: "obb"`. It supports both Layout A (Ultralytics 1-output FP32 `[1, 5 + nc, N]`) and Layout B (decoupled 9-output INT8/FP32 across 3 scale strides, interleaved or grouped, NHWC or NCHW). OBB models decode rotated boxes directly via ProbIoU NMS (the top 3000 candidates by score enter the NMS and at most 300 boxes are returned: `kObbMaxNms` / `kObbMaxDet` in `main/obb_geom.h`, Ultralytics `max_nms` 30000 / `max_det` 300 scaled for the SG2002 CPU) and provide `angle_deg`, `angle_conf`, `grasp_width_px`, `length_px`, and 4 normalised quad corners in `"obb_corners"`. Class names per model: `vision.json` catalogue `"classes"`, else a `<model stem>.names` file next to the `.cvimodel` (one name per line, as sscma's `labels.txt`), else COCO for 80-class detect / segment models, else `class_<id>`; OBB models never use COCO names (see [detection_api.md](../../../docs/detection_api.md#model-switching--模型切换)).
Segmentation timing from the standalone bench: Forward 71.2 ms, crop mask decode measured before the cache fix 15.6 ms (bench numbers; on-device `rebot-vision` measurements for detection, OBB, and segmentation (`--mask on`) are recorded in `## YOLO model comparison (SG2002)` below).

## YOLO model comparison (SG2002)

### Models and memory footprint

Measurements from `model_tool --info` and live `rebot-vision` v1.3.10 on SG2002:

- `detect` (`yolo11n_detection_cv181x_int8.cvimodel`, 80 COCO classes): file `3,113,208 B` (`2.97 MB`), ION `7.69 MB`, input `images_raw` (`uint8 [1, 640, 640, 3]`), 6 `int8` outputs (`cv2.{0,1,2}` `[1,64,H,W]` + `cv3.{0,1,2}` `[1,80,H,W]` at `80x80`, `40x40`, `20x20`). Live `rebot-vision`: standby `VmRSS 11,420 kB` (`VmPeak 32,012 kB`), `15 fps` `VmRSS 13,848 kB` (`VmPeak 35,684 kB`).
- `obb (fp32 out)` (`yolo11n_obb_cv181x_int8_fuse.cvimodel`, 15 DOTAv1 classes): file `3,201,512 B` (`3.05 MB`), ION `9.17 MB`, input `images_raw` (`uint8 [1, 640, 640, 3]`), 9 `fp32` outputs (`cv2.{0,1,2}` `[1,64,H,W]` + `cv3.{0,1,2}` `[1,15,H,W]` + `cv4.{0,1,2}` `[1,1,H,W]` at `80x80`, `40x40`, `20x20`). Live `rebot-vision`: standby `VmRSS 11,548 kB` (`VmPeak 33,604 kB`), `15 fps` `VmRSS 16,060 kB` (`VmPeak 36,340 kB`).
- `obb (int8 quant_output)` (`yolo11n_obb_cv181x_int8_fuse_qout.cvimodel`, 15 DOTAv1 classes): file `3,187,696 B` (`3.04 MB`), ION `7.24 MB`, input `images_raw` (`uint8 [1, 640, 640, 3]`), 9 `int8` outputs (`cv2.{0,1,2}` `[1,64,H,W]` + `cv3.{0,1,2}` `[1,15,H,W]` + `cv4.{0,1,2}` `[1,1,H,W]` at `80x80`, `40x40`, `20x20`, totaling `672,000 B` (`656.25 KiB`) vs `2,688,000 B` (`2.56 MiB`) for `fp32`). Live `rebot-vision`: standby `VmRSS 11,376 kB` (`VmPeak 31,620 kB`), `15 fps` `VmRSS 12,588 kB` (`VmPeak 32,832 kB`).
- Unfused OBB note (`yolo11n_obb_cv181x_int8.cvimodel`): file `3,194,880 B` (`3.05 MB`), ION `12.68 MB`, input `images` (`fp32 [1, 3, 640, 640]`), 9 `fp32` outputs; not compatible with `ma::model::Detector` (feeds `uint8`), and standalone `CVI_NN_Forward` measured `151.59 ms` vs `37.96 ms` fused.
- `segment` (`yolo11n_segment_cv181x_int8.cvimodel`, 80 COCO classes): file `4,043,296 B` (`3.86 MB`), ION `17.91 MB`, input `images_raw` (`uint8 [1, 640, 640, 3]`), 2 `fp32` outputs (`output0_Concat_f32 [1, 116, 8400, 1]`, `output1_Mul_f32 [1, 32, 160, 160]`). Live `rebot-vision` (`--mask on`): standby `VmRSS 11,556 kB` (`VmPeak 42,736 kB`), `15 fps` `VmRSS 17,620 kB` (`VmPeak 48,792 kB`).

### Live rebot-vision v1.3.10 HTTP API v2 benchmark

Measured on a physical SG2002 reCamera. Test conditions: `rebot-vision` started with `nice -n 10 --depth-model off`, external client over USB-RNDIS, triggers N=30 at 1.2 s spacing, continuous streaming measured over a 12 s window + 20 polled frames.

Scene and class-count asymmetry: indoor desk scene yielded 0 detections for DOTAv1 15-class OBB and COCO 80-class detection, and 0..2 detections (mean 0.3 on triggers, 0.2 at 5 fps, 0.35 at 15 fps) for segmentation with `--mask on`.

#### Standby (10 s)

| Model | Chip CPU | Process CPU |
|---|---|---|
| `obb (fp32 out)` | 15.74% | 9.47% |
| `obb (int8 qout)` | 15.77% | 9.69% |
| `detect` | 15.64% | 9.36% |
| `segment (--mask on)` | 15.91% | 9.55% |

#### Triggers (N=30, skipped = 0)

| Metric | `obb (fp32 out)` | `obb (int8 qout)` | `detect` | `segment (--mask on)` |
|---|---|---|---|---|
| `yolo_ms` mean (p50, p95, max) | 42.79 (42.7, 43.4, 43.7) | 38.81 (38.8, 38.9, 39.8) | 46.46 (46.4, 47.0, 47.4) | 93.83 (86.1, 119.3, 120.4) |
| `mask_ms` mean (p50, p95, max) | - | - | - | 7.72 (0.0, 33.2, 34.3) |
| Server `grab-gate` mean / p50 (ms) | 38.03 / 38 | 31.57 / 32 | 39.83 / 39 | 70.53 / 71 |
| Server `done-frame` mean / p50 (ms) | 42.9 / 43 | 38.90 / 39 | 46.6 / 47 | 93.83 / 86 |
| Server `reply-accept` mean / p50 (ms) | 82.1 / 82 | 71.27 / 71 | 88.33 / 87 | 166.3 / 159 |
| Client HTTP RTT mean (p50, p95) (ms) | 92.81 (93.00, 94.45) | 78.58 (78.91, 80.30) | 96.66 (96.39, 98.99) | 176.61 (169.31, 202.96) |

#### Continuous streaming

| Workload | Chip CPU | Process CPU | VmRSS (VmPeak) | Input / Result FPS | `yolo_ms` mean (p50, p95) | `mask_ms` mean (p50, p95) |
|---|---|---|---|---|---|---|
| `yolo @ 5 fps` `obb (fp32 out)` | 22.47% | 16.21% | 12,448 kB (33,932 kB) | 7.5 / 5.0 | 42.67 (42.7, 42.8) | - |
| `yolo @ 5 fps` `obb (int8 qout)` | 20.67% | 14.73% | 11,384 kB (31,808 kB) | 7.5 / 5.0 | 38.73 (38.7, 38.8) | - |
| `yolo @ 5 fps` `detect` | 24.85% | 18.42% | 11,436 kB (33,240 kB) | 7.5 / 5.0 | 46.34 (46.3, 46.4) | - |
| `yolo @ 5 fps` `segment (--mask on)` | 31.68% | 24.77% | 11,600 kB (46,824 kB) | 7.5 / 5.0 | 92.20 (85.6, 119.3) | 6.63 (0.0, 33.4) |
| `yolo @ 15 fps` `obb (fp32 out)` | 33.45% | 26.45% | 16,060 kB (36,340 kB) | 14.5 / 14.0 | 43.08 (43.0, 44.0) | - |
| `yolo @ 15 fps` `obb (int8 qout)` | 28.80% | 21.97% | 12,588 kB (32,832 kB) | 15.0 / 14.5 | 38.85 (38.8, 39.8) | - |
| `yolo @ 15 fps` `detect` | 40.09% | 33.16% | 13,848 kB (35,684 kB) | 15.0 / 14.5 | 48.78 (46.8, 87.7) | - |
| `yolo @ 15 fps` `segment (--mask on)` | 45.81% | 38.36% | 17,620 kB (48,792 kB) | 9.5 / 8.0 | 112.93 (92.6, 337.3) | 18.65 (0.0, 160.5) |

At requested 15 fps, segmentation with `--mask on` saturated below the target rate (input 9.5 fps, result 8.0 fps).

### OBB stage breakdown on SG2002

Measured with `obb_stage_bench` (standalone binary, 5 warmup + 50 runs on 640x640 DOTA input):
- Baseline (`int8_fuse`, 9 `fp32` outputs, cell-major decoder): `CVI_NN_RegisterModel` `20.29 ms`, `VmPeak 13,892 kB`, `VmRSS 3,368 kB`, `VmHWM 3,372 kB`.
- Optimized FP32 (`int8_fuse`, 9 `fp32` outputs, NCHW plane-major decoder): `CVI_NN_RegisterModel` `20.46 ms`, `VmPeak 13,932 kB`, `VmRSS 3,380 kB`, `VmHWM 3,384 kB`.
- Optimized INT8 (`int8_fuse_qout`, 9 `int8` outputs, NCHW plane-major `int8` decoder with `q_min` threshold pre-filter): `CVI_NN_RegisterModel` `20.11 ms`, `VmPeak 11,920 kB`, `VmRSS 3,356 kB`, `VmHWM 3,360 kB`.

#### Nominal config: `conf=0.25, nms_iou=0.45` (Candidates Raw Decoded: 35, After NMS: 4)

| Stage | Baseline (`int8_fuse`, FP32 out, cell-major) | Optimized FP32 (`int8_fuse`, plane-major) | Optimized INT8 (`int8_fuse_qout`, plane-major + `q_min`) |
|---|---|---|---|
| `CVI_NN_Forward` (TPU) | mean 36.306, p50 36.280, p95 36.479, max 36.526 ms | mean 36.284, p50 36.278, p95 36.338, max 36.436 ms | mean 35.167, p50 35.164, p95 35.202, max 35.214 ms |
| `rv::decode_obb_layout_b` (CPU) | mean 4.645, p50 4.626, p95 4.721, max 4.872 ms | mean 2.830, p50 2.831, p95 2.893, max 2.925 ms | mean 1.727, p50 1.705, p95 1.764, max 1.801 ms |
| `rv::obb_nms` (CPU ProbIoU NMS) | mean 0.094, p50 0.094, p95 0.096, max 0.137 ms | mean 0.094, p50 0.092, p95 0.094, max 0.157 ms | mean 0.092, p50 0.091, p95 0.093, max 0.145 ms |
| `rv::obb_to_geom + detections_json` | mean 0.695, p50 0.680, p95 0.750, max 0.790 ms | mean 0.676, p50 0.665, p95 0.735, max 0.859 ms | mean 0.692, p50 0.681, p95 0.750, max 0.767 ms |
| Total pipeline | mean 41.741, p50 41.720, p95 41.943, max 42.200 ms (23.96 FPS) | mean 39.883, p50 39.870, p95 39.966, max 40.056 ms (25.07 FPS) | mean 37.678, p50 37.691, p95 37.735, max 37.782 ms (26.54 FPS) |

#### Stress config: `conf=0.05, nms_iou=0.45` (Candidates Raw Decoded: 50, After NMS: 4)

| Stage | Baseline (`int8_fuse`, FP32 out, cell-major) | Optimized FP32 (`int8_fuse`, plane-major) | Optimized INT8 (`int8_fuse_qout`, plane-major + `q_min`) |
|---|---|---|---|
| `CVI_NN_Forward` (TPU) | mean 36.326, p50 36.281, p95 36.348, max 38.327 ms | mean 36.290, p50 36.280, p95 36.385, max 36.413 ms | mean 35.171, p50 35.166, p95 35.211, max 35.255 ms |
| `rv::decode_obb_layout_b` (CPU) | mean 4.817, p50 4.802, p95 4.892, max 4.922 ms | mean 2.962, p50 2.999, p95 3.049, max 3.107 ms | mean 1.881, p50 1.870, p95 1.966, max 2.064 ms |
| `rv::obb_nms` (CPU ProbIoU NMS) | mean 0.119, p50 0.115, p95 0.162, max 0.163 ms | mean 0.114, p50 0.114, p95 0.115, max 0.165 ms | mean 0.112, p50 0.112, p95 0.113, max 0.165 ms |
| `rv::obb_to_geom + detections_json` | mean 0.671, p50 0.680, p95 0.747, max 0.749 ms | mean 0.664, p50 0.662, p95 0.732, max 0.739 ms | mean 0.674, p50 0.678, p95 0.749, max 0.763 ms |
| Total pipeline | mean 41.933, p50 41.910, p95 42.031, max 43.940 ms (23.85 FPS) | mean 40.030, p50 40.067, p95 40.160, max 40.213 ms (24.98 FPS) | mean 37.838, p50 37.872, p95 37.960, max 38.016 ms (26.43 FPS) |

OBB CPU utilization and latency are lower than `detect` in this test because DOTAv1 has 15 classes compared to 80 COCO classes, and `decode_obb_layout_b` skips DFL softmax unless class score exceeds the inverse-sigmoid threshold. Switching from strided cell-major to NCHW plane-major sequential class scanning reduced FP32 CPU decode latency from 4.645 ms to 2.830 ms (conf=0.25). Compiling with `--quant_output` emits 9 INT8 outputs directly, eliminating 9 trailing `tpu.Cast` ops, saving 1.93 MB ION memory (7.24 MB vs 9.17 MB) and 1.14 ms TPU execution time (35.167 ms vs 36.306 ms); applying exact INT8 score threshold pre-filtering (`q_min`) further reduced CPU decode latency from 2.830 ms to 1.727 ms.

### CV181X OBB export requirements

- CV181X TPU-MLIR `TopToTpuPass` lacks lowering support for `top.Cos` and `top.Sin` emitted by Ultralytics rotated box decoding, so export truncates the ONNX head to the 9 raw conv outputs (`cv2.{0,1,2}`, `cv3.{0,1,2}`, `cv4.{0,1,2}`) before DFL and rotated decode.
- Compile with `--fuse_preprocess --customization_format RGB_PACKED` so the cvimodel expects `UINT8 [1, 640, 640, 3]` (`ma::model::Detector` feeds `UINT8`), reducing standalone `CVI_NN_Forward` from `151.59 ms` / `12.68 MB` ION (unfused FP32 NCHW) to `37.96 ms` / `9.17 MB` ION in the standalone `cviruntime` bench (and `36.306 ms` / `9.17 MB` ION in `obb_stage_bench`).
- Compile with `--quant_output` to emit 9 `INT8` outputs directly with zero extra quantization (`CosineSim > 0.999997` and raw `int8` diff `0.000000` vs intermediate pre-cast outputs of `int8_fuse`, matching `ObbModel` / `decode_obb_layout_b` `MA_TENSOR_TYPE_S8` support), while keeping input `UINT8 [1, 640, 640, 3]` via `--fuse_preprocess --customization_format RGB_PACKED` rather than signed `INT8` (`--quant_input`) which triggers `u8[i] -= 128` CPU subtraction inside `ma::model::Detector::preprocess`.

#### Reproducible TPU-MLIR v1.7 build recipe

Configuration: `UINT8 [1, 640, 640, 3]` `RGB_PACKED` fused input + 9 `INT8` direct outputs, `7.24 MB` ION footprint.

1. Export ONNX (downgrade ONNX IR version to `<= 8` if required by `tpu_mlir==1.7`):
```sh
yolo export model=yolo11n-obb.pt format=onnx opset=17 simplify=True imgsz=640
```

2. Transform ONNX to MLIR with fused preprocessing and truncated head (raw conv outputs before DFL and rotated decode; head index `23` for YOLO11-OBB, `22` for YOLOv8-OBB):
```sh
model_transform.py \
    --model_name yolo11n_obb \
    --model_def yolo11n-obb.onnx \
    --input_shapes [[1,3,640,640]] \
    --mean 0.0,0.0,0.0 \
    --scale 0.0039216,0.0039216,0.0039216 \
    --keep_aspect_ratio \
    --pixel_format rgb \
    --output_names /model.23/cv2.0/cv2.0.2/Conv_output_0,/model.23/cv3.0/cv3.0.2/Conv_output_0,/model.23/cv4.0/cv4.0.2/Conv_output_0,/model.23/cv2.1/cv2.1.2/Conv_output_0,/model.23/cv3.1/cv3.1.2/Conv_output_0,/model.23/cv4.1/cv4.1.2/Conv_output_0,/model.23/cv2.2/cv2.2.2/Conv_output_0,/model.23/cv3.2/cv3.2.2/Conv_output_0,/model.23/cv4.2/cv4.2.2/Conv_output_0 \
    --mlir yolo11n_obb.mlir
```

3. Calibrate INT8 quantization:
```sh
run_calibration.py yolo11n_obb.mlir \
    --dataset <calib_dir> \
    --input_num 100 \
    -o yolo11n_obb_cali_table
```

4. Deploy cvimodel with fused `RGB_PACKED` input and direct `INT8` quantized output (do not pass `--quant_input`, so input remains `UINT8` to avoid CPU `u8 - 128` subtraction):
```sh
model_deploy.py \
    --mlir yolo11n_obb.mlir \
    --quantize INT8 \
    --chip cv181x \
    --calibration_table yolo11n_obb_cali_table \
    --fuse_preprocess \
    --customization_format RGB_PACKED \
    --quant_output \
    --model yolo11n_obb_cv181x_int8_fuse_qout.cvimodel
```

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
- Default rate 5 fps, from the device measurements under "Frame rate and triggers" (1.2.1 both @ 5:
  58 % whole-chip CPU; @ 10 or 15: 75 % for 7.2 results/s; 1.3.1 yolo + aruco @ 5: 55 %).
