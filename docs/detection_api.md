# Detection API / 识别接口

armd proxies the camera detector's on-demand inference API under its own origin (port 8765), so the page and external agents use one port. The upstream defaults to `http://127.0.0.1:8090`; set `ARMD_DETECTOR_URL` to change it.

armd 在自身端口（8765）下代理摄像头检测程序的按需推理接口，页面和外部 agent 只需访问一个端口。上游默认 `http://127.0.0.1:8090`，可用环境变量 `ARMD_DETECTOR_URL` 修改。

On the reCamera kit (`rebot-teleop` 1.2.0) the provider is `rebot-vision`, which adds vision modes (`off`, `yolo`, `aruco`, `both`). Older providers (`yolo-detector`, `aruco-detector`) still work with the detection routes; `/api/v1/vision/mode` answers 404 for them.

reCamera 套件（`rebot-teleop` 1.2.0）的上游是 `rebot-vision`，增加视觉模式（`off`、`yolo`、`aruco`、`both`）。旧版检测程序（`yolo-detector`、`aruco-detector`）仍可使用检测路由，`/api/v1/vision/mode` 对它们返回 404。

## API version / 接口版本

Every `rebot-vision` response body has `"api_version": 1`. A body without the field comes from a version 0 provider (no `/api/mode`; `latest` carries only `detections` or only `markers`). armd reports the provider's version as `capabilities.vision.api_version` in `GET /api/v1/observation/schema`.

`rebot-vision` 的每个响应体都有 `"api_version": 1`。没有该字段的响应来自版本 0 的服务（没有 `/api/mode`；`latest` 只含 `detections` 或只含 `markers`）。armd 在 `GET /api/v1/observation/schema` 的 `capabilities.vision.api_version` 中报告该版本。

## Routes / 路由

| Route | Method | Upstream | Response |
|---|---|---|---|
| `/api/v1/detection/inference` | `GET` | `GET /api/inference` | `{"enabled":bool,"fps":n,"active_fps":n,"idle_fps":1}` |
| `/api/v1/detection/inference` | `POST` | `POST /api/inference` | Body `{"enabled":true\|false}`; same shape as GET. A bad body returns 400 `{"error":...}` |
| `/api/v1/detection/latest` | `GET` | `GET /api/results/latest` (query string passed through) | `{"enabled":bool,"frame_id":u32,"ts_ms":n,"age_ms":n,"inference_time_ms":n,"detections":[...],"timeout"?:true}` |
| `/api/v1/detection/trigger` | `POST` | `POST /api/trigger` (query string passed through) | `{"success":true,"frame_id":u32,"ts_ms":n,"age_ms":n,"inference_time_ms":n,"count":n,"markers":[...]}` or `{"success":false,"timeout":true,"count":0,"markers":[]}` |
| `/api/v1/vision/mode` | `GET` | `GET /api/mode` | mode body below (version 1) |
| `/api/v1/vision/mode` | `POST` | `POST /api/mode` | Body `{"mode"?: "off"\|"yolo"\|"aruco"\|"both", "enabled"?: bool, "fps"?: 1..15}` (at least one key); same body as GET. Bad value → 400, YOLO mode without a model → 409 `{"error":"yolo unavailable"}` |

- Status code and JSON body are passed through unchanged. 状态码和 JSON 原样透传。
- `latest?after=<frame_id>&wait_ms=<0..5000>` long-polls until a frame newer than `after` exists or `wait_ms` elapses (`"timeout":true`). The upstream caps `wait_ms` at 5000. armd waits `wait_ms` (capped at 5000) + 2 s for the upstream. 长轮询：等到比 `after` 新的帧或超时；armd 的上游超时为 `wait_ms`（上限 5000）+ 2 秒。
- `trigger?wait_ms=<0..5000>` (default 2500) runs one inference on a frame newer than the call and returns it, without changing the continuous `enabled` state, so it does not interfere with other clients that turned inference on. No fresh frame within `wait_ms` → `"success":false,"timeout":true`. armd waits `wait_ms` (default 2500, capped at 5000) + 2 s. Detector builds without this route return 404, which armd passes through; use the enable → `latest` → disable sequence below for those. 单次触发：对调用之后的新帧推理一次并返回，不改变持续推理开关，不影响其他已开启推理的客户端；`wait_ms` 内无新帧返回 `success:false`。armd 上游超时为 `wait_ms`（默认 2500，上限 5000）+ 2 秒。旧版检测程序无此路由时返回 404（原样透传），此时改用下方 开启 → `latest` → 关闭 流程。
- Detector unreachable or timed out: `502 {"error":"detector unreachable"}`. 检测程序不可达或超时返回 502。
- No authentication, like the other `/api/v1` routes. `POST` is subject to the same browser same-origin check (cross-origin `Origin` → 403). Use only on a trusted network. 与其他 `/api/v1` 路由一致，无鉴权；`POST` 同样做浏览器同源校验，仅限可信网络。

The page's **Detect** button (video HUD, top-left in expert mode) reads and toggles `/api/v1/detection/inference`. It shows "N/A" and is disabled when the route fails (for example an older detector build). Turning detection off clears the overlay.

The single-shot button and calibration capture call `/api/v1/detection/trigger`. On a 404 the page remembers that the detector lacks the route and uses enable → `latest` → disable instead, disabling inference afterwards only if the page itself enabled it.

页面视频区的「识别」按钮读取并切换该接口；接口不可用时显示「不可用」并禁用。关闭识别会清空检测框。

「单次识别」按钮和标定采样调用 `/api/v1/detection/trigger`；返回 404 时页面记住检测程序不支持该路由，改用 开启 → `latest` → 关闭，且只在推理是页面自己开启时才关闭。

## Vision modes (rebot-vision, API version 1) / 视觉模式

- `mode`: which pipelines run on a processed frame. `yolo` = YOLO on the TPU, `aruco` = ArUco markers on the CPU, `both` = both on the same frame (one result carries both lists), `off` = none. 每帧运行的流水线；`both` 在同一帧上同时运行两者，一个结果同时包含两个列表。
- `enabled`: continuous inference. With `enabled:false` the camera's inference channel stands by at `idle_fps` (1) and nothing runs until a trigger. `{"mode":"off"}` also sets `enabled:false`; `POST /api/v1/detection/inference {"enabled":true}` while the mode is `off` selects `last_active_mode` (default `yolo`). 持续推理开关；关闭时推理通道以 1 fps 待机，只有单次触发才运行。`off` 同时关闭推理；在 `off` 时开启推理会恢复上次的模式。
- `fps`: one inference rate for every mode (`active_fps`, 1..15, default 5 from `vision.json`): results per second while enabled, as long as the pipelines keep up (`measured.result_fps`). `fps` in the response is the current rate (`idle_fps` in standby). `camera_fps` is the camera frame gate behind it (1 in standby, 30 during a trigger), `measured.input_fps` / `measured.result_fps` the frames reaching rebot-vision and the results published per second over the last 2 s. 所有模式共用一个推理帧率，即持续推理时每秒结果数（流水线跟得上时）；`measured` 为最近 2 秒实测的输入帧率与结果率。
- Trigger: `POST /api/v1/detection/trigger` runs the current mode once on the first frame that arrives after the call (in `off` it runs `last_active_mode`), returns that result and leaves `mode` and `enabled` unchanged. From standby the camera gate opens for the trigger, so the wait is one camera frame (15 fps) plus the pipelines, then the idle rate returns. 单次触发在每种模式下都可用，不改变模式和推理开关；待机时触发会临时放开取帧，等待约一帧（15 fps）加处理时间，之后恢复待机帧率。
- A mode change applies from the next processed frame; the YOLO model stays loaded, so no reload. 模式切换在下一帧生效，模型常驻，无需重新加载。

`GET /api/v1/vision/mode`:

```json
{"api_version":1,"mode":"both","modes":["off","yolo","aruco","both"],
 "pipelines":{"yolo":true,"aruco":true},"last_active_mode":"both","enabled":true,"processing":true,
 "fps":5,"active_fps":5,"idle_fps":1,"fps_range":[1,15],"camera_fps":10,
 "measured":{"input_fps":7.5,"result_fps":5.0},"trigger":true,
 "available":{"yolo":true,"aruco":true},
 "model":{"name":"yolo11n_detection_cv181x_int8","path":"/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel"},
 "family":"aruco4x4_50"}
```

Result envelope of `latest` and `trigger` (version 1). `detections` and `markers` are always present (empty when that pipeline did not run), so a client reading one list works in every mode. `width`/`height` give the visible image size the coordinates refer to: `detections` are normalised to it (centre `x`, `y`, `w`, `h`), `markers` are pixels in it. `pipelines` tells what ran on this frame. `count` is the number of accepted markers (as aruco-detector); `rejected` the markers dropped by the false-positive filter.

`latest` / `trigger` 的结果（版本 1）：`detections` 与 `markers` 始终存在（对应流水线未运行时为空）。`width`/`height` 为坐标所对应的可见画面尺寸：`detections` 按其归一化（中心点），`markers` 为其像素坐标。

```json
{"api_version":1,"success":true,"mode":"both","frame_id":42,"ts_ms":1762680000123,"age_ms":35,
 "inference_time_ms":81.2,"width":640,"height":360,
 "pipelines":{"yolo":{"ran":true,"ms":74.0},"aruco":{"ran":true,"ms":6.1}},
 "detections":[{"class_id":39,"class_name":"bottle","confidence":0.710,"x":0.4200,"y":0.5500,"w":0.0800,"h":0.2100}],
 "count":1,"family":"aruco4x4_50","undistorted":false,"rejected":0,
 "markers":[{"id":3,"margin":61.0,"center":[182.0,212.0],"corners":[[150.0,244.0],[214.0,244.0],[214.0,180.0],[150.0,180.0]],
   "pose":{"t":[-0.0921,0.0213,0.4502],"r_deg":[178.20,1.50,3.00],"error":0.0004}}]}
```

`latest` has `"enabled"` in place of `"success"`, and `"timeout":true` when no newer frame arrived within `wait_ms`. The trigger timeout body is `{"api_version":1,"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}`.

Coordinate change against yolo-detector: the inference frame is 640x640 with the 16:9 camera image letterboxed into the band y = 140..500. yolo-detector normalised boxes to the whole 640x640 frame; `rebot-vision` normalises them to the visible 640x360 band, the image size the grasp and the cockpit assume. Markers are detected on that band (same pixels and intrinsics as aruco-detector's 640x360 channel). The debug stream `/results` (video overlay) is unchanged for boxes (display pixels) and adds `"mode"` and `"markers"` in display pixels.

与 yolo-detector 的坐标差异：推理帧为 640x640，16:9 画面以信箱方式位于 y = 140..500。yolo-detector 按整个 640x640 归一化；`rebot-vision` 按可见的 640x360 区域归一化（抓取和驾驶舱假设的尺寸）。`/results` 调试流中的框不变（显示像素），新增 `"mode"` 和显示像素的 `"markers"`。

The cockpit's vision selector (video HUD, next to Detect) reads and writes `/api/v1/vision/mode`; choosing a mode other than Off also turns continuous inference on. It is hidden when the route answers 404. The hand-eye calibration drawer switches to `aruco` (`both` when YOLO was on) with inference on while it is open and restores the previous mode when it closes. The grasp action switches to `yolo` (`both` when ArUco was on) for its duration and restores the previous mode afterwards.

驾驶舱视频区「识别」旁的模式选择读取并设置 `/api/v1/vision/mode`，选择非「关」的模式会开启持续识别；路由返回 404 时隐藏。手眼标定抽屉打开期间切换到 `aruco`（原来开着 YOLO 时为 `both`），关闭时恢复。抓取动作执行期间切换到 `yolo`（原来开着 ArUco 时为 `both`），结束后恢复。

## Agent loop example / Agent 调用示例

```sh
ARMD=http://<device-ip>:8765

# rebot-vision: choose what runs; trigger stays available in standby
curl -s -X POST -H 'Content-Type: application/json' -d '{"mode":"aruco"}' $ARMD/api/v1/vision/mode | jq '{mode, enabled}'

# Single shot (detector builds with /api/trigger); leaves the inference toggle unchanged
curl -s -X POST "$ARMD/api/v1/detection/trigger?wait_ms=3000" | jq '{success, frame_id, count, markers}'
```

Continuous use, or detectors without `/api/trigger`:

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
