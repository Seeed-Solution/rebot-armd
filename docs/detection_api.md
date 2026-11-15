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
| `/api/v1/detection/trigger` | `POST` | `POST /api/trigger` (query string and JSON body passed through) | Body (version 2) `{"pipelines"?: ["yolo"] \| ["aruco"] \| ["depth"] \| ["yolo","depth"] \| ..., "wait_ms"?: 0..5000, "mask_focus"?: {"label"?: string, "box_norm": [x0,y0,x1,y1]}}`. `{"success":true,"requested":[...],"frame_id":u32,...}` (result envelope below) or `{"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}`. Bad `pipelines` → 400, `yolo`/`depth` without a model → 409 |
| `/api/v1/detection/mask_focus` | `GET` | `GET /api/mask_focus` | `{"api_version":2,"mask_focus":{...}\|null}` |
| `/api/v1/detection/mask_focus` | `POST` | `POST /api/mask_focus` | Body `{"label"?: string, "box_norm": [x0,y0,x1,y1], "ttl_ms"?: 100..10000}`; empty `{}` or `ttl_ms: 0` clears it. Returns same shape as GET |
| `/api/v1/vision/mode` | `GET` | `GET /api/mode` | mode body below |
| `/api/v1/vision/mode` | `POST` | `POST /api/mode` | Body `{"mode"?: "off"\|"yolo"\|"aruco"\|"both", "enabled"?: bool, "fps"?: 1..15}` (at least one key); same body as GET. Bad value → 400, YOLO mode without a model → 409 `{"error":"yolo unavailable"}`. Version 2: the default subscription, `fps` the cap of every subscription |
| `/api/v1/vision/subscribe` | `POST` | `POST /api/subscribe` | Body `{"client": "1..64 chars", "pipelines": [...], "fps"?: 1..15, "ttl_ms"?: 500..60000, "id"?}` → subscription body (version 2). 400 bad body, 409 `yolo` without a model, 503 more than 16 subscriptions |
| `/api/v1/vision/subscribe/{id}/keepalive` | `POST` | `POST /api/subscribe/{id}/keepalive` | subscription body; 404 when unknown or expired. armd answers 400 for an id that is not `[A-Za-z0-9_-]{1,32}` |
| `/api/v1/vision/subscribe/{id}` | `DELETE` | `DELETE /api/subscribe/{id}` | `{"deleted":id,"continuous":{...}}`; 404 when unknown |
| `/api/v1/vision/subscriptions` | `GET` | `GET /api/subscriptions` | list body below |

- Status code and JSON body are passed through unchanged. 状态码和 JSON 原样透传。
- `mask_focus` (API version 2): sets persistent focus on the grasp target object. `POST /api/v1/detection/mask_focus` takes `{"label"?: string, "box_norm": [x0, y0, x1, y1], "ttl_ms"?: 100..10000}` (default 2000 ms). Empty `{}` or `ttl_ms: 0` clears it. `GET /api/v1/detection/mask_focus` returns the active focus and its remaining TTL. When active, segmentation models decode the matching target box first (IoU >= 0.1) before taking other boxes by score. Detections decoded via mask focus carry `"mask_focus": true`. `POST /api/v1/detection/trigger` also accepts an optional `"mask_focus"` field in its body, applying to that trigger's frame only without persisting.
- 掩码焦点（接口版本 2）：设置抓取目标焦点。`POST /api/v1/detection/mask_focus` 接收 `{"label"?: string, "box_norm": [x0, y0, x1, y1], "ttl_ms"?: 100..10000}`（默认 2000 毫秒）；空对象 `{}` 或 `ttl_ms: 0` 清空焦点。`GET /api/v1/detection/mask_focus` 查询当前焦点及剩余 TTL。生效时分割模型优先解码与焦点匹配的目标框（IoU >= 0.1）。经焦点解码的检测结果包含 `"mask_focus": true`。`POST /api/v1/detection/trigger` 亦可在请求体中传入 `"mask_focus"`，仅作用于该次单次触发的帧，不产生持续焦点。
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

`per_request_pipelines`, `subscriptions`, `max_fps` and `continuous` are new in version 2. `available.depth` and `depth_model` (`null` or `{"name": string, "path": string, "metric": bool}`) report monocular depth availability (`metric`: the catalogue entry's `"metric"`, else `--depth-metric on|off` for the startup depth model, else `"metric"` in the model file name; see [Model switching](#model-switching--模型切换)). `model.kind` is `detect`, `segment` or `obb`, and `null` when no YOLO model is loaded (`available.yolo` false; 1.3.10 and earlier reported `detect`). `model.mask_max_boxes` (1..20, default 3) and `mask_focus` (active focus or `null`) report mask focus and budget configuration.

```json
{"api_version":2,"mode":"both","modes":["off","yolo","aruco","both"],
 "pipelines":{"yolo":true,"aruco":true},"last_active_mode":"both","enabled":true,"processing":true,
 "fps":5,"active_fps":5,"idle_fps":1,"fps_range":[1,15],"camera_fps":10,
 "measured":{"input_fps":7.5,"result_fps":5.0},"trigger":true,
 "per_request_pipelines":true,"subscriptions":true,"max_fps":5,
 "continuous":{"active":true,"pipelines":["yolo","aruco"],"fps":5,"subscriptions":0},
 "available":{"yolo":true,"aruco":true,"depth":true},
 "model":{"name":"yolo11n_detection_cv181x_int8","path":"/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel","kind":"detect","mask":false,"mask_max_boxes":3},
 "depth_model":{"name":"zipdepth_metric_384x224_bf16_rgb","path":"/userdata/local/models/zipdepth_metric_384x224_bf16_rgb.cvimodel","metric":true},
 "family":"aruco4x4_50",
 "mask_focus":{"box_norm":[0.35,0.40,0.50,0.65],"ttl_ms":2000,"expires_in_ms":1850,"remaining_ttl_ms":1850,"label":"bottle"}}
```

`models` and `ion` (builds with model switching): `models.generation` (+1 per executed swap), `models.swap` (`state` `idle` / `pending` / `running` / `failed`, the `generation` it publishes or published, `error` when failed), `models.resident` (catalogue names of the loaded YOLO and depth models, `null` for an empty slot) and `models.catalogue` (`name`, `kind`, `path`, `ion_mb`, `resident`, and `metric` on depth entries); `ion` is the ION budget (`base_mb`, `used_mb` = base + resident models, `total_mb`, `reserve_mb`). See [Model switching](#model-switching--模型切换).

`models` 与 `ion`（支持模型切换的构建）：`models.generation`（每执行一次切换 +1）、`models.swap`（`state` 为 `idle` / `pending` / `running` / `failed`，其发布或已发布的 `generation`，失败时的 `error`）、`models.resident`（已加载的 YOLO 与深度模型的目录名称，空槽位为 `null`）与 `models.catalogue`（`name`、`kind`、`path`、`ion_mb`、`resident`，深度项另有 `metric`）；`ion` 为 ION 预算（`base_mb`、`used_mb` = 基础 + 常驻模型、`total_mb`、`reserve_mb`）。见[模型切换](#model-switching--模型切换)。

```json
"models":{"generation":1,"swap":{"state":"idle","generation":1},
 "resident":{"detector":"det","depth":"depth"},
 "catalogue":[{"name":"depth","kind":"depth","path":"/userdata/local/models/zipdepth_metric_384x224_bf16_rgb.cvimodel","ion_mb":20.5,"resident":true,"metric":true},
              {"name":"det","kind":"detect","path":"/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel","ion_mb":8.5,"resident":true},
              {"name":"obb","kind":"obb","path":"/userdata/local/models/yolo11n_obb_cv181x_int8_fuse_qout.cvimodel","ion_mb":8.0,"resident":false},
              {"name":"seg","kind":"segment","path":"/usr/share/supervisor/models/yolo11n_segment_cv181x_int8.cvimodel","ion_mb":19.5,"resident":false}]},
"ion":{"base_mb":21.5,"used_mb":50.5,"total_mb":62.9,"reserve_mb":8.0}
```

Result envelope of `latest` and `trigger` (versions 1 and 2; `requested` on a trigger and `format` on detections are new in version 2). `detections` and `markers` are always present (empty when that pipeline did not run), so a client reading one list works in every mode. `width`/`height` give the visible image size the coordinates refer to: `detections` are normalised to it (centre `x`, `y`, `w`, `h`), `markers` are pixels in it. Segmentation models (`model.kind: "segment"`) with mask decoding enabled (`model.mask: true`, flag `--mask on`, `vision.json` `"mask"`) provide `angle_deg` (object long side, [-90, 90) deg), `angle_conf` ([0, 1]), and `grasp_width_px` (cross-section grasp width in visible image pixels) on finite detections, and report `mask_ms` in `pipelines.yolo`. Oriented bounding box models (`model.kind: "obb"`, configured via `--model-kind auto|detect|segment|obb` or `vision.json` `"model_kind"`) decode oriented boxes and provide `angle_deg`, `angle_conf`, `grasp_width_px`, `length_px`, and `"obb_corners"` (4 normalised corners in perimeter order `[[u0,v0],[u1,v1],[u2,v2],[u3,v3]]` mapped to the visible image) on finite detections. Supported OBB layouts include Layout A (Ultralytics 1-output FP32 `[1, 5 + nc, N]`) and Layout B (decoupled 9-output INT8/FP32 across 3 scale strides, interleaved or grouped, NHWC or NCHW). When a detection is decoded via mask focus, it carries `"mask_focus": true`. When the `"depth"` pipeline runs, each detection carries `disp_fg` (foreground disparity), `disp_bg` (table background disparity around object), `depth_conf` ([0, 1] disparity confidence), `length_px` (object major axis length in pixels), `jaw_clear` (gripper jaw collision clearance flag), and the depth PCA geometry `depth_angle_deg` (long side, [-90, 90) deg), `depth_angle_conf` ([0, 1]) and `depth_width_px` (cross-section width in visible image pixels); each is omitted when not finite. The depth values are never written into `angle_deg` / `angle_conf` / `grasp_width_px`: with a plain detection model those stay absent while depth runs. `jaw_clear` checks strips beside the object along the jaw closing direction: the detector `angle_deg` + 90 deg when the detection has a mask or OBB angle, else the depth PCA minor axis. A top-level `"depth"` object provides `metric_model` (bool, same as `depth_model.metric`), `grid_w`/`grid_h` (grid resolution, e.g. 384x224), and `table_samples` (`[[u_px, v_px, disp], ...]`). When a standard detection model is loaded and depth is not running, optional segmentation, OBB, and depth fields are omitted, maintaining byte-identical output. `pipelines` tells what ran on this frame. `count` is the number of accepted markers (as aruco-detector); `rejected` the markers dropped by the false-positive filter.

`latest` / `trigger` 的结果（版本 1 / 2）：`detections` 与 `markers` 始终存在（对应流水线未运行时为空）。`width`/`height` 为坐标所对应的可见画面尺寸：`detections` 按其归一化（中心点），`markers` 为其像素坐标。分割模型（`model.kind: "segment"`）在开启掩码解码（`model.mask: true`，参数 `--mask on` 或 `vision.json` `"mask"`）时，有限检测结果另提供 `angle_deg`（目标长边角度，[-90, 90) 度）、`angle_conf`（[0, 1]）与 `grasp_width_px`（可见画面像素下的横截面抓取宽度），并在 `pipelines.yolo` 中输出掩码耗时 `mask_ms`。旋转框检测模型（`model.kind: "obb"`，可通过 `--model-kind auto|detect|segment|obb` 或 `vision.json` `"model_kind"` 配置）解码旋转框并在检测结果中直接输出 `angle_deg`、`angle_conf`、`grasp_width_px`、`length_px` 与 `"obb_corners"`（映射至可见画面的 4 个顺次归一化角点坐标 `[[u0,v0],[u1,v1],[u2,v2],[u3,v3]]`）。支持的 OBB 架构包括 Layout A（Ultralytics 单输出 FP32 `[1, 5 + nc, N]`）与 Layout B（解耦 9 输出 INT8/FP32，覆盖 3 个特征尺度，支持交错/分组及 NHWC/NCHW 格式）。经掩码焦点解码的目标检测框带有 `"mask_focus": true`。当运行 `"depth"` 深度流水线时，每项检测包含 `disp_fg`（前景视差）、`disp_bg`（背景桌面视差）、`depth_conf`（深度置信度）、`length_px`（目标长轴像素长度）、`jaw_clear`（夹爪净空无碰撞标志），以及深度 PCA 几何 `depth_angle_deg`（长边角度，[-90, 90) 度）、`depth_angle_conf`（[0, 1]）与 `depth_width_px`（可见画面像素下的横截面宽度），非有限值时省略。深度结果不会写入 `angle_deg` / `angle_conf` / `grasp_width_px`：普通检测模型运行深度时这三个字段仍不出现。`jaw_clear` 检查物体两侧沿夹爪闭合方向的条带：检测带掩码或旋转框角度时为 `angle_deg` + 90 度，否则为深度 PCA 短轴方向；顶层 `"depth"` 对象包含 `metric_model`（是否度量模型，同 `depth_model.metric`）、`grid_w`/`grid_h`（网格分辨率）与 `table_samples`（桌面采样点 `[[u_px, v_px, disp], ...]`）。使用普通检测模型且未运行深度时，上述字段均省略以保持输出字节级一致。

```json
{"api_version":2,"success":true,"requested":["yolo","depth"],"mode":"both","frame_id":42,"ts_ms":1762680000123,"age_ms":35,
 "inference_time_ms":95.4,"width":640,"height":360,
 "pipelines":{"yolo":{"ran":true,"ms":74.0,"mask_ms":15.6},"depth":{"ran":true,"ms":14.2},"aruco":{"ran":false}},
 "detections":[{"class_id":39,"format":"xywh_center","class_name":"bottle","confidence":0.710,"x":0.4200,"y":0.5500,"w":0.0800,"h":0.2100,"angle_deg":15.00,"angle_conf":0.850,"grasp_width_px":28.50,"length_px":65.0,"disp_fg":1.82,"disp_bg":0.85,"depth_conf":0.92,"jaw_clear":true,"depth_angle_deg":14.20,"depth_angle_conf":0.610,"depth_width_px":30.10,"mask_focus":true}],
 "depth":{"metric_model":false,"grid_w":384,"grid_h":224,"table_samples":[[40.0,20.0,0.85],[320.0,180.0,0.81],[600.0,340.0,0.76]]},
 "count":1,"family":"aruco4x4_50","undistorted":false,"rejected":0,
 "markers":[{"id":3,"margin":61.0,"center":[182.0,212.0],"corners":[[150.0,244.0],[214.0,244.0],[214.0,180.0],[150.0,180.0]],
   "pose":{"t":[-0.0921,0.0213,0.4502],"r_deg":[178.20,1.50,3.00],"error":0.0004}}]}
```

From `rebot-teleop` 1.3.1, `pipelines.aruco` also has `"stages"`: ms per ArUco stage on that frame (`gray`, `decimate`, `threshold`, `unionfind`, `clusters`, `quad_fit`, `refine`, `decode`, `detect_other`, `filter`, `pose`), the candidate quad count `quads`, the `quad_decimate` used and the scanned region `roi` `[w,h]`. 自 `rebot-teleop` 1.3.1 起 `pipelines.aruco` 另含 `"stages"`：该帧 ArUco 各阶段耗时（ms）、候选四边形数 `quads`、所用 `quad_decimate` 与检测区域 `roi` `[w,h]`。

A trigger reply from builds after `rebot-teleop` 1.3.1 also has `"timing"`: CLOCK_MONOTONIC ms (node `process.hrtime`) of `accept_ms`, `read_ms`, `request_ms`, `gate_ms`, `wait_ms`, `grab_ms`, `frame_ms`, `done_ms`, `wake_ms`, `reply_ms`, and `skipped` (frames that did not serve the trigger); fields in `kits/recamera-b601dm/vision/README.md`. 1.3.1 之后的构建中，触发结果另含 `"timing"`：连接接受、请求读完、登记、开闸、取帧、发布、唤醒、回复各时刻（CLOCK_MONOTONIC 毫秒）及未用于该触发的帧数 `skipped`。

Builds with model switching add `"model_generation"` to every result: the `models.generation` the frame ran with. 支持模型切换的构建在每个结果中加入 `"model_generation"`：该帧运行时的 `models.generation`。

`latest` has `"enabled"` in place of `"success"`, and `"timeout":true` when no newer frame arrived within `wait_ms`. The trigger timeout body is `{"api_version":2,"success":false,"timeout":true,"count":0,"detections":[],"markers":[]}`.

Coordinate change against yolo-detector: the inference frame is 640x640 with the 16:9 camera image letterboxed into the band y = 140..500. yolo-detector normalised boxes to the whole 640x640 frame; `rebot-vision` normalises them to the visible 640x360 band, the image size the grasp and the cockpit assume. Markers are detected on that band (same pixels and intrinsics as aruco-detector's 640x360 channel). The debug stream `/results` (video overlay) is unchanged for boxes (display pixels) and adds `"mode"` and `"markers"` in display pixels.

与 yolo-detector 的坐标差异：推理帧为 640x640，16:9 画面以信箱方式位于 y = 140..500。yolo-detector 按整个 640x640 归一化；`rebot-vision` 按可见的 640x360 区域归一化（抓取和驾驶舱假设的尺寸）。`/results` 调试流中的框不变（显示像素），新增 `"mode"` 和显示像素的 `"markers"`。

The cockpit's vision selector (video HUD, next to Detect) picks the pipelines of the page's overlay subscription with a version 2 provider (YOLO / ArUco / Both turn the overlay on, Off turns it off) and never sets the global mode; with a version 1 provider it reads and writes `/api/v1/vision/mode` as before. It is hidden when the route answers 404. The hand-eye calibration drawer holds an ArUco subscription for its live preview while it is open and the page is visible, and captures each pose with one `["aruco"]` trigger. The grasp action triggers `["yolo"]` once per centring step and changes no mode. With a version 1 provider the drawer and the grasp switch the mode and restore it afterwards.

驾驶舱视频区「识别」旁的模式选择在版本 2 上游下选择本页叠加层订阅的流水线（YOLO / ArUco / 两者 开启叠加层，「关」关闭），不修改全局模式；版本 1 上游下仍读取并设置 `/api/v1/vision/mode`。路由返回 404 时隐藏。手眼标定抽屉打开且页面可见期间持有一个 ArUco 订阅用于实时预览，每次采样用一次 `["aruco"]` 单次触发。抓取动作每次对准用一次 `["yolo"]` 单次触发，不修改模式。版本 1 上游下标定抽屉和抓取仍切换模式并在结束后恢复。

## Model switching / 模型切换

rebot-vision loads one YOLO model (the `detector` slot: kind `detect`, `segment` or `obb`) and at most one depth model (the `depth` slot) at a time and can swap them in-process on request. Measured on the SG2002 test unit (2026-11-14, rebot-vision 1.3.10): all ION is allocated at startup (peak during triggers = idle); detect 29.2 MB, detect + depth 49.2 MB, segment 39.9 MB, segment + depth 59.9 MB (3.0 MB free of the 62.9 MB carveout), obb + depth 48.7 MB. With a 3.9 MB buffer left by another process, segment + depth made the VENC fail to allocate and the process could not exit until a reboot; a model registration without free ION aborts the process. So every load is preceded by a static budget check:

rebot-vision 同一时刻加载一个 YOLO 模型（`detector` 槽位：类型 `detect`、`segment` 或 `obb`）和至多一个深度模型（`depth` 槽位），可按请求在进程内切换。SG2002 测试机实测（2026-11-14，rebot-vision 1.3.10）：ION 全部在启动时分配（触发期间峰值 = 空闲值）；detect 29.2 MB，detect + depth 49.2 MB，segment 39.9 MB，segment + depth 59.9 MB（62.9 MB 预留区剩 3.0 MB），obb + depth 48.7 MB。其他进程残留 3.9 MB 缓冲时，segment + depth 使 VENC 分配失败，进程直到重启都无法退出；无空闲 ION 时注册模型会使进程中止。因此每次加载前做静态预算检查：

`ion_base_mb` + sum(`ion_mb` of the resident set) + `ion_reserve_mb` <= `ion_total_mb` (defaults 21.5, 8, 62.9 MB in `vision.json`). The budget holds only from a clean ION baseline (no other ION user on the unit); it is not a fail-safe allocator. No source of free ION other than debugfs was found, so the check uses the catalogue's numbers, not a reading. With the measured ion_mb (+ ~5 %: detect 8.5, segment 19.5, obb 8.0, depth 20.5): detect + depth 58.5, obb + depth 58.0, segment alone 49.0 fit; segment + depth 69.5 > 62.9 is refused.

`ion_base_mb` + 常驻模型的 `ion_mb` 之和 + `ion_reserve_mb` <= `ion_total_mb`（`vision.json` 默认 21.5、8、62.9 MB）。该预算只在 ION 基线干净（本机无其他 ION 使用者）时成立，不是可靠的分配器。除 debugfs 外未找到读取空闲 ION 的途径，因此检查使用目录中的数值而非实时读数。按实测 ion_mb（约 +5 %：detect 8.5、segment 19.5、obb 8.0、depth 20.5）：detect + depth 58.5、obb + depth 58.0、单独 segment 49.0 可加载；segment + depth 69.5 > 62.9 被拒绝。

Catalogue in `vision.json` (optional) / `vision.json` 中的目录（可选）:

```json
{"models": {
   "det":   {"path": "/usr/share/supervisor/models/yolo11n_detection_cv181x_int8.cvimodel", "kind": "detect",  "ion_mb": 8.5},
   "seg":   {"path": "/usr/share/supervisor/models/yolo11n_segment_cv181x_int8.cvimodel",   "kind": "segment", "ion_mb": 19.5},
   "obb":   {"path": "/userdata/local/models/yolo11n_obb_cv181x_int8_fuse_qout.cvimodel",   "kind": "obb",     "ion_mb": 8.0,
             "classes": ["plane", "ship", "storage tank", "baseball diamond", "tennis court", "basketball court",
                         "ground track field", "harbor", "bridge", "large vehicle", "small vehicle", "helicopter",
                         "roundabout", "soccer ball field", "swimming pool"]},
   "depth": {"path": "/userdata/local/models/zipdepth_metric_384x224_bf16_rgb.cvimodel",    "kind": "depth",   "ion_mb": 20.5, "metric": true}},
 "ion_base_mb": 21.5, "ion_total_mb": 62.9, "ion_reserve_mb": 8}
```

- Every catalogue entry states `ion_mb` (finite, >= 0, measured for that model file); entries without it, with another kind or a name that is not 1..64 of letters, digits, `_`, `-`, `.` are skipped with a warning. 每个目录项必须给出 `ion_mb`（有限值，>= 0，按该模型文件实测）；缺少该值、类型不符或名称不是 1..64 个字母、数字、`_`、`-`、`.` 的项被跳过并记录警告。
- The startup models stay today's (`model` / `depth_model`, else the first existing default file). A startup model whose path is in the catalogue uses that entry; otherwise it is added as an entry named from its file stem with `ion_mb` from the kind table above (the YOLO kind as loaded). 启动模型不变（`model` / `depth_model`，否则第一个存在的默认文件）。路径在目录中的启动模型使用该目录项；否则以文件名主干为名加入目录，`ion_mb` 取上表中对应类型的值（YOLO 类型以实际加载结果为准）。
- Depth entries may state `"metric": true|false` (metric or relative depth). The metric flag of a loaded depth model is resolved in this order: the entry's `metric`; then `--depth-metric on|off` (`vision.json` `"depth_metric"`), only for the startup depth model it was given for; then `"metric"` in the file stem (`zipdepth_metric_*` metric, `zipdepth_base_*` relative). A relative model swapped in by `POST /api/models` therefore reports `metric: false` even when the process was started with `--depth-metric on`. `metric` on a non-depth entry or a non-boolean value skips the entry with a warning. `models.catalogue` reports the resolved `metric` of every depth entry. 深度目录项可给出 `"metric": true|false`（度量或相对深度）。已加载深度模型的度量标志按以下顺序确定：目录项的 `metric`；其次 `--depth-metric on|off`（`vision.json` `"depth_metric"`），只作用于它所指定的启动深度模型；最后看文件名主干是否含 `"metric"`（`zipdepth_metric_*` 为度量，`zipdepth_base_*` 为相对）。因此即使进程以 `--depth-metric on` 启动，经 `POST /api/models` 换入的相对深度模型仍报告 `metric: false`。非深度项上的 `metric` 或非布尔值会使该项被跳过并记录警告。`models.catalogue` 为每个深度项报告确定后的 `metric`。
- YOLO entries may state `"classes": ["name0", "name1", ...]` (1..1000 non-empty strings, by class id). Class names (`class_name` of a detection, the debug overlay labels, the `mask_focus.label` match) come from, in order: the entry's `classes`; the sidecar file next to the model, its path with the last extension replaced by `.names` (`yolo11n_obb_cv181x_int8_fuse_qout.cvimodel` → `yolo11n_obb_cv181x_int8_fuse_qout.names`, one name per line, the format of sscma's `labels.txt`; blank lines skipped); the 80 COCO names for a `detect` or `segment` model with 80 classes (the yolo11n detection and segmentation models); else `class_<id>`. An OBB model never falls back to COCO: a DOTAv1 model without names reports `class_0` .. `class_14`, not `person`. A names count that differs from the model's class count is logged; ids beyond the list are `class_<id>`. The startup model uses the `classes` of the catalogue entry with its path. YOLO 目录项可给出 `"classes": ["name0", "name1", ...]`（1..1000 个非空字符串，按类别 id 排列）。类别名称（检测结果的 `class_name`、调试画面标签、`mask_focus.label` 匹配）依次取自：目录项的 `classes`；模型旁的同名文件，即把模型路径最后一个扩展名换成 `.names`（每行一个名称，与 sscma 的 `labels.txt` 格式相同，空行跳过）；80 类的 `detect` 或 `segment` 模型（yolo11n 检测与分割模型）使用 80 个 COCO 名称；否则为 `class_<id>`。OBB 模型不回退到 COCO：没有名称的 DOTAv1 模型报告 `class_0` .. `class_14`，而不是 `person`。名称数量与模型类别数不一致时记录日志；超出列表的 id 为 `class_<id>`。启动模型使用路径相同的目录项的 `classes`。
- The startup set is checked too: when the YOLO model and the depth model together exceed the budget, only the YOLO model is loaded and the log says why (`Depth model ... not loaded: ... need 69.5 MB ION, limit 62.9 MB`). This changes the behaviour of a segment + depth startup configuration, which earlier loaded both. 启动组合同样检查：YOLO 模型与深度模型合计超出预算时只加载 YOLO 模型，并在日志中说明原因。这改变了 segment + depth 启动配置的行为（之前两者都加载）。

`POST /api/models` on the provider (`:8090`; armd sends it for a [grasp task](agent_action_api.md#grasp-tasks--抓取任务) with `models` and does not proxy it):

提供端 `POST /api/models`（`:8090`；带 `models` 的[抓取任务](agent_action_api.md#grasp-tasks--抓取任务)由 armd 发送，armd 不代理此路由）：

| Body | Reply |
|---|---|
| `{"detector"?: name\|null, "depth"?: name\|null}` (absent = keep, `null` = unload) | 202 `{"api_version":2,"generation":n}`: accepted, applied by the frame thread |
| unknown name, a name of the wrong kind for its slot, other keys, no slot | 400 `{"error":"..."}` |
| the resident set with the request applied exceeds the budget | 409 `{"error":"ION_BUDGET","need_mb":69.5,"limit_mb":62.9}` |
| a swap is pending or running | 409 `{"error":"SWAP_BUSY"}` |

- One swap at a time, asynchronous: the reply does not wait for the load. The frame thread (the only user of the models) applies the pending swap at the start of its next frame, also in standby (a pending swap opens the camera gate as a trigger does); it re-checks the budget against the resident set, unloads the changed slots first, then loads each into a fresh model object (so the ION peak stays within the new set), reapplies the threshold, mask and scale settings, and publishes the new set with `generation` + 1. The control lock is not held while loading. 一次只处理一个切换，异步：回复不等待加载。帧线程（唯一使用模型的线程）在下一帧开始时执行待处理的切换，待机时也一样（待处理的切换像触发一样打开取帧闸门）；它按常驻模型重新检查预算，先卸载要更换的槽位，再把每个模型加载到新建的模型对象中（ION 峰值不超过新组合），重新应用阈值、掩码与缩放设置，然后以 `generation` + 1 发布新组合。加载期间不持有控制锁。
- Poll `GET /api/mode` until `models.generation` reaches the returned generation and `models.swap.state` is `idle` (done) or `failed`. A slot whose load failed stays empty (`error` `LOAD_FAILED:detector` / `LOAD_FAILED:depth`), the other slot keeps its new state, and the generation still advances. 轮询 `GET /api/mode`，直到 `models.generation` 达到返回的代号且 `models.swap.state` 为 `idle`（完成）或 `failed`。加载失败的槽位保持为空（`error` 为 `LOAD_FAILED:detector` / `LOAD_FAILED:depth`），另一槽位保持其新状态，代号照常增加。
- While the YOLO slot is empty, results report `pipelines.yolo.ran: false`, `available.yolo` is false and YOLO triggers, subscriptions and modes are refused with 409 `yolo unavailable`; with the depth slot empty, depth requests get 409 `depth unavailable`. Existing subscriptions stay registered and get no results for that pipeline; waiting triggers end with their timeout. YOLO 槽位为空时结果报告 `pipelines.yolo.ran: false`，`available.yolo` 为 false，YOLO 触发、订阅与模式以 409 `yolo unavailable` 拒绝；深度槽位为空时深度请求返回 409 `depth unavailable`。已有订阅保持注册，该流水线不再有结果；等待中的触发按其超时结束。
- `available`, `model` and `depth_model` in the mode body follow the resident set; every result carries `model_generation`. A pending swap is dropped at shutdown. 模式中的 `available`、`model` 与 `depth_model` 随常驻组合变化；每个结果带 `model_generation`。退出时丢弃待处理的切换。
- Not measured on the device yet: ION after each swap through the model wrappers, and the swap latency (raw cviruntime register/cleanup: 27.5 ms seg, 18 ms detect, ION back to 0 after 60 cycles). 尚未实机测量：经模型封装切换后的 ION 与切换耗时（裸 cviruntime 注册/释放：seg 27.5 ms、detect 18 ms，60 次后 ION 回到 0）。

The MuJoCo simulator's perception server has the same route and budget numbers (catalogue `seg`, `det`, `depth`; [simulation](simulation.md)). MuJoCo 仿真的感知服务提供同样的路由与预算数值（目录 `seg`、`det`、`depth`）。

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
