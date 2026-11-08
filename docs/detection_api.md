# Detection API / 识别接口

armd proxies the camera detector's on-demand inference API under its own origin (port 8765), so the page and external agents use one port. The upstream defaults to `http://127.0.0.1:8090`; set `ARMD_DETECTOR_URL` to change it.

armd 在自身端口（8765）下代理摄像头检测程序的按需推理接口，页面和外部 agent 只需访问一个端口。上游默认 `http://127.0.0.1:8090`，可用环境变量 `ARMD_DETECTOR_URL` 修改。

## Routes / 路由

| Route | Method | Upstream | Response |
|---|---|---|---|
| `/api/v1/detection/inference` | `GET` | `GET /api/inference` | `{"enabled":bool,"fps":n,"active_fps":n,"idle_fps":1}` |
| `/api/v1/detection/inference` | `POST` | `POST /api/inference` | Body `{"enabled":true\|false}`; same shape as GET. A bad body returns 400 `{"error":...}` |
| `/api/v1/detection/latest` | `GET` | `GET /api/results/latest` (query string passed through) | `{"enabled":bool,"frame_id":u32,"ts_ms":n,"age_ms":n,"inference_time_ms":n,"detections":[...],"timeout"?:true}` |
| `/api/v1/detection/trigger` | `POST` | `POST /api/trigger` (query string passed through) | `{"success":true,"frame_id":u32,"ts_ms":n,"age_ms":n,"inference_time_ms":n,"count":n,"markers":[...]}` or `{"success":false,"timeout":true,"count":0,"markers":[]}` |

- Status code and JSON body are passed through unchanged. 状态码和 JSON 原样透传。
- `latest?after=<frame_id>&wait_ms=<0..5000>` long-polls until a frame newer than `after` exists or `wait_ms` elapses (`"timeout":true`). The upstream caps `wait_ms` at 5000. armd waits `wait_ms` (capped at 5000) + 2 s for the upstream. 长轮询：等到比 `after` 新的帧或超时；armd 的上游超时为 `wait_ms`（上限 5000）+ 2 秒。
- `trigger?wait_ms=<0..5000>` (default 2500) runs one inference on a frame newer than the call and returns it, without changing the continuous `enabled` state, so it does not interfere with other clients that turned inference on. No fresh frame within `wait_ms` → `"success":false,"timeout":true`. armd waits `wait_ms` (default 2500, capped at 5000) + 2 s. Detector builds without this route return 404, which armd passes through; use the enable → `latest` → disable sequence below for those. 单次触发：对调用之后的新帧推理一次并返回，不改变持续推理开关，不影响其他已开启推理的客户端；`wait_ms` 内无新帧返回 `success:false`。armd 上游超时为 `wait_ms`（默认 2500，上限 5000）+ 2 秒。旧版检测程序无此路由时返回 404（原样透传），此时改用下方 开启 → `latest` → 关闭 流程。
- Detector unreachable or timed out: `502 {"error":"detector unreachable"}`. 检测程序不可达或超时返回 502。
- No authentication, like the other `/api/v1` routes. `POST` is subject to the same browser same-origin check (cross-origin `Origin` → 403). Use only on a trusted network. 与其他 `/api/v1` 路由一致，无鉴权；`POST` 同样做浏览器同源校验，仅限可信网络。

The page's **Detect** button (video HUD, top-left in expert mode) reads and toggles `/api/v1/detection/inference`. It shows "N/A" and is disabled when the route fails (for example an older detector build). Turning detection off clears the overlay.

The single-shot button and calibration capture call `/api/v1/detection/trigger`. On a 404 the page remembers that the detector lacks the route and uses enable → `latest` → disable instead, disabling inference afterwards only if the page itself enabled it.

页面视频区的「识别」按钮读取并切换该接口；接口不可用时显示「不可用」并禁用。关闭识别会清空检测框。

「单次识别」按钮和标定采样调用 `/api/v1/detection/trigger`；返回 404 时页面记住检测程序不支持该路由，改用 开启 → `latest` → 关闭，且只在推理是页面自己开启时才关闭。

## Agent loop example / Agent 调用示例

```sh
ARMD=http://<device-ip>:8765

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
