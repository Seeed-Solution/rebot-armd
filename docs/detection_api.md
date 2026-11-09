# Detection API / 识别接口

armd proxies the camera detector's on-demand inference API under its own origin (port 8765), so the page and external agents use one port. The upstream defaults to `http://127.0.0.1:8090`; set `ARMD_DETECTOR_URL` to change it.

armd 在自身端口（8765）下代理摄像头检测程序的按需推理接口，页面和外部 agent 只需访问一个端口。上游默认 `http://127.0.0.1:8090`，可用环境变量 `ARMD_DETECTOR_URL` 修改。

On the reCamera kit the provider is `rebot-vision`. From `rebot-teleop` 1.3.0 (API version 2) vision runs on demand per caller: a trigger names the pipelines it needs, continuous inference comes from subscriptions that expire without keep-alive, and with no subscription nothing is inferred. `rebot-teleop` 1.2 (API version 1) had one global vision mode (`off`, `yolo`, `aruco`, `both`) that callers switched and restored. Older providers (`yolo-detector`, `aruco-detector`) still work with the detection routes; `/api/v1/vision/*` answers 404 for them.

reCamera 套件的上游是 `rebot-vision`。从 `rebot-teleop` 1.3.0（接口版本 2）起视觉按调用方按需运行：单次触发指定所需流水线，持续推理通过订阅进行，订阅不续期即过期；没有订阅时不做推理。`rebot-teleop` 1.2（接口版本 1）只有一个全局视觉模式（`off`、`yolo`、`aruco`、`both`），由调用方切换再恢复。旧版检测程序（`yolo-detector`、`aruco-detector`）仍可使用检测路由，`/api/v1/vision/*` 对它们返回 404。

## API version / 接口版本

Every `rebot-vision` response body has `"api_version"`: 2 from 1.3.0, 1 in 1.2. A body without the field comes from a version 0 provider (no `/api/mode`; `latest` carries only `detections` or only `markers`). armd reports the provider's version as `capabilities.vision.api_version` in `GET /api/v1/observation/schema`, and `capabilities.vision.per_request_pipelines` / `capabilities.vision.subscriptions` (true from version 2).

`rebot-vision` 的每个响应体都有 `"api_version"`：1.3.0 起为 2，1.2 为 1。没有该字段的响应来自版本 0 的服务（没有 `/api/mode`；`latest` 只含 `detections` 或只含 `markers`）。armd 在 `GET /api/v1/observation/schema` 的 `capabilities.vision.api_version` 中报告该版本，并报告 `capabilities.vision.per_request_pipelines` / `capabilities.vision.subscriptions`（版本 2 起为 true）。

## Routes / 路由

| Route | Method | Upstream | Response |
|---|---|---|---|
| `/api/v1/detection/inference` | `GET` | `GET /api/inference` | `{"enabled":bool,"fps":n,"active_fps":n,"idle_fps":1}` |
| `/api/v1/detection/inference` | `POST` | `POST /api/inference` | Body `{"enabled":true\|false}`; same shape as GET. A bad body returns 400 `{"error":...}` |
| `/api/v1/detection/latest` | `GET` | `GET /api/results/latest` (query string passed through) | `{"enabled":bool,"frame_id":u32,"ts_ms":n,"age_ms":n,"inference_time_ms":n,"detections":[...],"timeout"?:true}` |
| `/api/v1/detection/trigger` | `POST` | `POST /api/trigger` (query string and JSON body passed through) | Body (version 2) `{"pipelines"?: ["yolo"] \| ["aruco"] \| ["yolo","aruco"], "wait_ms"?: 0..5000}`. `{"success":true,"requested":[...],"frame_id":u32,...}` (result envelope below) or `{"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}`. Bad `pipelines` → 400, `yolo` without a model → 409 |
| `/api/v1/vision/mode` | `GET` | `GET /api/mode` | mode body below |
| `/api/v1/vision/mode` | `POST` | `POST /api/mode` | Body `{"mode"?: "off"\|"yolo"\|"aruco"\|"both", "enabled"?: bool, "fps"?: 1..15}` (at least one key); same body as GET. Bad value → 400, YOLO mode without a model → 409 `{"error":"yolo unavailable"}`. Version 2: the default subscription, `fps` the cap of every subscription |
| `/api/v1/vision/subscribe` | `POST` | `POST /api/subscribe` | Body `{"client": "1..64 chars", "pipelines": [...], "fps"?: 1..15, "ttl_ms"?: 500..60000, "id"?}` → subscription body (version 2). 400 bad body, 409 `yolo` without a model, 503 more than 16 subscriptions |
| `/api/v1/vision/subscribe/{id}/keepalive` | `POST` | `POST /api/subscribe/{id}/keepalive` | subscription body; 404 when unknown or expired. armd answers 400 for an id that is not `[A-Za-z0-9_-]{1,32}` |
| `/api/v1/vision/subscribe/{id}` | `DELETE` | `DELETE /api/subscribe/{id}` | `{"deleted":id,"continuous":{...}}`; 404 when unknown |
| `/api/v1/vision/subscriptions` | `GET` | `GET /api/subscriptions` | list body below |

- Status code and JSON body are passed through unchanged. 状态码和 JSON 原样透传。
- `latest?after=<frame_id>&wait_ms=<0..5000>` long-polls until a frame newer than `after` exists or `wait_ms` elapses (`"timeout":true`). The upstream caps `wait_ms` at 5000. armd waits `wait_ms` (capped at 5000) + 2 s for the upstream. 长轮询：等到比 `after` 新的帧或超时；armd 的上游超时为 `wait_ms`（上限 5000）+ 2 秒。
- `trigger?wait_ms=<0..5000>` (default 2500; a body `wait_ms` wins) runs one inference on a frame newer than the call and returns it, without changing any continuous inference, so it does not interfere with other clients. Version 2 runs the body's `pipelines` (without `pipelines`: the default subscription's mode, or the last active mode when that is `off`, as in version 1). No fresh frame within `wait_ms` → `"success":false,"timeout":true`. armd waits `wait_ms` (default 2500, capped at 5000) + 2 s. Detector builds without this route return 404, which armd passes through; use the enable → `latest` → disable sequence below for those. 单次触发：对调用之后的新帧推理一次并返回，不改变持续推理开关，不影响其他已开启推理的客户端；`wait_ms` 内无新帧返回 `success:false`。armd 上游超时为 `wait_ms`（默认 2500，上限 5000）+ 2 秒。旧版检测程序无此路由时返回 404（原样透传），此时改用下方 开启 → `latest` → 关闭 流程。
- Detector unreachable or timed out: `502 {"error":"detector unreachable"}`. 检测程序不可达或超时返回 502。
- No authentication, like the other `/api/v1` routes. `POST` and `DELETE` are subject to the same browser same-origin check (cross-origin `Origin` → 403). Use only on a trusted network. 与其他 `/api/v1` 路由一致，无鉴权；`POST`、`DELETE` 同样做浏览器同源校验，仅限可信网络。
- Directly on `:8090`, `POST` needs `Content-Type: application/json` (415 otherwise), keep-alive included (send `{}`); armd adds it. 直接访问 `:8090` 时 `POST` 必须带 `Content-Type: application/json`（否则 415），续期也一样（发送 `{}`）；经 armd 时由 armd 添加。

The page's **Detect** button (video HUD, top-left in expert mode) turns the page's detection overlay on and off. With a version 2 provider the overlay holds the page's own subscription (pipelines from the vision selector, kept alive every second with `ttl_ms` 3000, deleted when the overlay is turned off, while the page is hidden and when it closes); with older providers it toggles `/api/v1/detection/inference`. It shows "N/A" and is disabled when the route fails (for example an older detector build). Turning detection off clears the overlay.

The single-shot button calls `/api/v1/detection/trigger` with the selector's pipelines; calibration capture sends `{"pipelines":["aruco"]}`. On a 404 the page remembers that the detector lacks the route and uses enable → `latest` → disable instead, disabling inference afterwards only if the page itself enabled it.

页面视频区的「识别」按钮开关本页的检测叠加层。上游为版本 2 时，叠加层持有本页自己的订阅（流水线来自视觉模式选择，每秒续期，`ttl_ms` 3000；关闭叠加层、页面隐藏或关闭时删除）；旧版上游则切换 `/api/v1/detection/inference`。接口不可用时显示「不可用」并禁用。关闭识别会清空检测框。

「单次识别」按钮以所选流水线调用 `/api/v1/detection/trigger`；标定采样发送 `{"pipelines":["aruco"]}`。返回 404 时页面记住检测程序不支持该路由，改用 开启 → `latest` → 关闭，且只在推理是页面自己开启时才关闭。

## Subscriptions (API version 2) / 订阅

- Standby: with no active subscription (and the default one off) nothing is inferred. The inference channel's camera gate is at `idle_fps` (1); the H.264 stream and `/snapshot.jpg` keep running. 无订阅时不做推理，推理通道取帧 1 fps 待机，视频流与快照照常。
- A subscription is one caller's continuous inference: `pipelines` at `fps` (default and cap: the configured rate, `vision.json` `fps`, default 5). It ends with `DELETE`, or `ttl_ms` (default 3000, 500..60000) after the last subscribe or keep-alive. One subscription per `client`: subscribing again with the same `client` (or the returned `id`) updates it in place and refreshes it. 订阅即某个调用方的持续推理；`DELETE` 结束，或在最后一次订阅/续期后 `ttl_ms` 内未续期则过期。同一 `client` 再次订阅会原地更新并续期。
- What runs: the union of the active subscriptions' pipelines (and the default subscription's), at the highest requested rate clamped to the configured rate. `continuous` in the bodies reports it. Expired subscriptions are dropped on the next camera frame (within about 1 s in standby) and by every subscription request. 实际运行的是所有有效订阅流水线的并集，帧率取请求的最大值并受配置帧率限制。
- Triggers keep working while subscriptions run: the requested pipelines are added to the next frame, which is then also a continuous result. `pipelines.*.ran` tells what ran on a frame (a trigger result can show more than it asked for). 订阅运行期间单次触发照常：所请求流水线加到下一帧上。
- Results of continuous runs go to `latest` and the `/results` stream as before.
- `/api/v1/vision/mode` (and `/api/v1/detection/inference`) is the default subscription: it never expires and joins the union; `{"mode":"off"}` ends only it. 模式接口即默认订阅，永不过期，参与并集。

`POST /api/v1/vision/subscribe {"client":"agent-1","pipelines":["aruco"],"fps":10,"ttl_ms":3000}`:

```json
{"api_version":2,"id":"s2","client":"agent-1","pipelines":["aruco"],"fps":5,"requested_fps":10,"ttl_ms":3000,"expires_in_ms":3000,
 "continuous":{"active":true,"pipelines":["yolo","aruco"],"fps":5,"subscriptions":2}}
```

`GET /api/v1/vision/subscriptions`:

```json
{"api_version":2,"max_fps":5,"max_subscriptions":16,
 "default":{"mode":"yolo","enabled":false,"fps":5},
 "continuous":{"active":true,"pipelines":["yolo","aruco"],"fps":5,"subscriptions":2},
 "subscriptions":[
   {"id":"s1","client":"cockpit-overlay-ab12","pipelines":["yolo"],"fps":3,"requested_fps":3,"ttl_ms":3000,"expires_in_ms":2982},
   {"id":"s2","client":"agent-1","pipelines":["aruco"],"fps":5,"requested_fps":10,"ttl_ms":3000,"expires_in_ms":2991}]}
```

## Vision modes and the default subscription / 视觉模式与默认订阅

- `mode`: which pipelines run on a processed frame. `yolo` = YOLO on the TPU, `aruco` = ArUco markers on the CPU, `both` = both on the same frame (one result carries both lists), `off` = none. 每帧运行的流水线；`both` 在同一帧上同时运行两者，一个结果同时包含两个列表。
- `enabled`: the default subscription (continuous inference in version 1). With `enabled:false` and no other subscription the camera's inference channel stands by at `idle_fps` (1) and nothing runs until a trigger. `{"mode":"off"}` also sets `enabled:false`; `POST /api/v1/detection/inference {"enabled":true}` while the mode is `off` selects `last_active_mode` (default `yolo`). 持续推理开关；关闭时推理通道以 1 fps 待机，只有单次触发才运行。`off` 同时关闭推理；在 `off` 时开启推理会恢复上次的模式。
- `fps`: the configured rate (`active_fps` = `max_fps`, 1..15, default 5 from `vision.json`): the default subscription's rate and, in version 2, the cap of every subscription; results per second while continuous inference runs, as long as the pipelines keep up (`measured.result_fps`). `fps` in the response is the current rate (`idle_fps` in standby). `camera_fps` is the camera frame gate behind it (1 in standby, 30 during a trigger), `measured.input_fps` / `measured.result_fps` the frames reaching rebot-vision and the results published per second over the last 2 s. 所有模式共用一个推理帧率，即持续推理时每秒结果数（流水线跟得上时）；`measured` 为最近 2 秒实测的输入帧率与结果率。
- Trigger: `POST /api/v1/detection/trigger` runs the requested `pipelines` (version 2; without them the current mode, in `off` the `last_active_mode`) once on the first frame that arrives after the call, returns that result and leaves `mode`, `enabled` and the subscriptions unchanged. From standby the camera gate opens for the trigger, so the wait is one camera frame (15 fps) plus the pipelines, then the idle rate returns. 单次触发在每种模式下都可用，不改变模式和推理开关；待机时触发会临时放开取帧，等待约一帧（15 fps）加处理时间，之后恢复待机帧率。
- A mode change applies from the next processed frame; the YOLO model stays loaded, so no reload. 模式切换在下一帧生效，模型常驻，无需重新加载。

`GET /api/v1/vision/mode`:

`per_request_pipelines`, `subscriptions`, `max_fps` and `continuous` are new in version 2.

```json
{"api_version":2,"mode":"both","modes":["off","yolo","aruco","both"],
 "pipelines":{"yolo":true,"aruco":true},"last_active_mode":"both","enabled":true,"processing":true,
 "fps":5,"active_fps":5,"idle_fps":1,"fps_range":[1,15],"camera_fps":10,
 "measured":{"input_fps":7.5,"result_fps":5.0},"trigger":true,
 "per_request_pipelines":true,"subscriptions":true,"max_fps":5,
 "continuous":{"active":true,"pipelines":["yolo","aruco"],"fps":5,"subscriptions":0},
 "available":{"yolo":true,"aruco":true},
 "model":{"name":"yolo11n_detection_cv181x_int8","path":"/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel"},
 "family":"aruco4x4_50"}
```

Result envelope of `latest` and `trigger` (versions 1 and 2; `requested` on a trigger and `format` on detections are new in version 2). `detections` and `markers` are always present (empty when that pipeline did not run), so a client reading one list works in every mode. `width`/`height` give the visible image size the coordinates refer to: `detections` are normalised to it (centre `x`, `y`, `w`, `h`), `markers` are pixels in it. `pipelines` tells what ran on this frame. `count` is the number of accepted markers (as aruco-detector); `rejected` the markers dropped by the false-positive filter.

`latest` / `trigger` 的结果（版本 1）：`detections` 与 `markers` 始终存在（对应流水线未运行时为空）。`width`/`height` 为坐标所对应的可见画面尺寸：`detections` 按其归一化（中心点），`markers` 为其像素坐标。

```json
{"api_version":2,"success":true,"requested":["yolo","aruco"],"mode":"both","frame_id":42,"ts_ms":1762680000123,"age_ms":35,
 "inference_time_ms":81.2,"width":640,"height":360,
 "pipelines":{"yolo":{"ran":true,"ms":74.0},"aruco":{"ran":true,"ms":6.1}},
 "detections":[{"class_id":39,"format":"xywh_center","class_name":"bottle","confidence":0.710,"x":0.4200,"y":0.5500,"w":0.0800,"h":0.2100}],
 "count":1,"family":"aruco4x4_50","undistorted":false,"rejected":0,
 "markers":[{"id":3,"margin":61.0,"center":[182.0,212.0],"corners":[[150.0,244.0],[214.0,244.0],[214.0,180.0],[150.0,180.0]],
   "pose":{"t":[-0.0921,0.0213,0.4502],"r_deg":[178.20,1.50,3.00],"error":0.0004}}]}
```

From `rebot-teleop` 1.3.1, `pipelines.aruco` also has `"stages"`: ms per ArUco stage on that frame (`gray`, `decimate`, `threshold`, `unionfind`, `clusters`, `quad_fit`, `refine`, `decode`, `detect_other`, `filter`, `pose`), the candidate quad count `quads`, the `quad_decimate` used and the scanned region `roi` `[w,h]`. 自 `rebot-teleop` 1.3.1 起 `pipelines.aruco` 另含 `"stages"`：该帧 ArUco 各阶段耗时（ms）、候选四边形数 `quads`、所用 `quad_decimate` 与检测区域 `roi` `[w,h]`。

`latest` has `"enabled"` in place of `"success"`, and `"timeout":true` when no newer frame arrived within `wait_ms`. The trigger timeout body is `{"api_version":2,"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}`.

Coordinate change against yolo-detector: the inference frame is 640x640 with the 16:9 camera image letterboxed into the band y = 140..500. yolo-detector normalised boxes to the whole 640x640 frame; `rebot-vision` normalises them to the visible 640x360 band, the image size the grasp and the cockpit assume. Markers are detected on that band (same pixels and intrinsics as aruco-detector's 640x360 channel). The debug stream `/results` (video overlay) is unchanged for boxes (display pixels) and adds `"mode"` and `"markers"` in display pixels.

与 yolo-detector 的坐标差异：推理帧为 640x640，16:9 画面以信箱方式位于 y = 140..500。yolo-detector 按整个 640x640 归一化；`rebot-vision` 按可见的 640x360 区域归一化（抓取和驾驶舱假设的尺寸）。`/results` 调试流中的框不变（显示像素），新增 `"mode"` 和显示像素的 `"markers"`。

The cockpit's vision selector (video HUD, next to Detect) picks the pipelines of the page's overlay subscription with a version 2 provider (YOLO / ArUco / Both turn the overlay on, Off turns it off) and never sets the global mode; with a version 1 provider it reads and writes `/api/v1/vision/mode` as before. It is hidden when the route answers 404. The hand-eye calibration drawer holds an ArUco subscription for its live preview while it is open and the page is visible, and captures each pose with one `["aruco"]` trigger. The grasp action triggers `["yolo"]` once per centring step and changes no mode. With a version 1 provider the drawer and the grasp switch the mode and restore it afterwards.

驾驶舱视频区「识别」旁的模式选择在版本 2 上游下选择本页叠加层订阅的流水线（YOLO / ArUco / 两者 开启叠加层，「关」关闭），不修改全局模式；版本 1 上游下仍读取并设置 `/api/v1/vision/mode`。路由返回 404 时隐藏。手眼标定抽屉打开且页面可见期间持有一个 ArUco 订阅用于实时预览，每次采样用一次 `["aruco"]` 单次触发。抓取动作每次对准用一次 `["yolo"]` 单次触发，不修改模式。版本 1 上游下标定抽屉和抓取仍切换模式并在结束后恢复。

## Agent loop example / Agent 调用示例

```sh
ARMD=http://<device-ip>:8765

# Single shot with the pipelines this call needs (API version 2); changes nothing for other callers
curl -s -X POST -H 'Content-Type: application/json' -d '{"pipelines":["aruco"]}' \
  "$ARMD/api/v1/detection/trigger?wait_ms=3000" | jq '{success, frame_id, pipelines, markers}'

# Continuous: subscribe, keep alive within ttl_ms, read latest, delete when done
ID=$(curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"client":"my-agent","pipelines":["yolo"],"fps":2,"ttl_ms":5000}' $ARMD/api/v1/vision/subscribe | jq -r .id)
curl -s "$ARMD/api/v1/detection/latest?after=0&wait_ms=3000" | jq '.detections'
curl -s -X POST $ARMD/api/v1/vision/subscribe/$ID/keepalive > /dev/null
curl -s -X DELETE $ARMD/api/v1/vision/subscribe/$ID
```

API version 1 providers, or detectors without `/api/trigger`:

```sh
ARMD=http://<device-ip>:8765

# 1. Enable inference
curl -s -X POST -H 'Content-Type: application/json' -d '{"enabled":true}' $ARMD/api/v1/detection/inference

# 2. Wait for the first result (up to 3 s)
R=$(curl -s "$ARMD/api/v1/detection/latest?after=0&wait_ms=3000")
echo "$R" | jq '{frame_id, age_ms, timeout, detections}'

# 3. Read detections; pass the last frame_id as `after` to wait for the next frame
F=$(echo "$R" | jq '.frame_id')
curl -s "$ARMD/api/v1/detection/latest?after=$F&wait_ms=3000" | jq '.detections'

# 4. Disable inference when done
curl -s -X POST -H 'Content-Type: application/json' -d '{"enabled":false}' $ARMD/api/v1/detection/inference
```
