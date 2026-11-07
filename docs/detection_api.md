# Detection API / 识别接口

armd proxies the camera detector's on-demand inference API under its own origin (port 8765), so the page and external agents use one port. The upstream defaults to `http://127.0.0.1:8090`; set `ARMD_DETECTOR_URL` to change it.

armd 在自身端口（8765）下代理摄像头检测程序的按需推理接口，页面和外部 agent 只需访问一个端口。上游默认 `http://127.0.0.1:8090`，可用环境变量 `ARMD_DETECTOR_URL` 修改。

## Routes / 路由

| Route | Method | Upstream | Response |
|---|---|---|---|
| `/api/v1/detection/inference` | `GET` | `GET /api/inference` | `{"enabled":bool,"fps":n,"active_fps":n,"idle_fps":1}` |
| `/api/v1/detection/inference` | `POST` | `POST /api/inference` | Body `{"enabled":true\|false}`; same shape as GET. A bad body returns 400 `{"error":...}` |
| `/api/v1/detection/latest` | `GET` | `GET /api/results/latest` (query string passed through) | `{"enabled":bool,"frame_id":u32,"ts_ms":n,"age_ms":n,"inference_time_ms":n,"detections":[...],"timeout"?:true}` |

- Status code and JSON body are passed through unchanged. 状态码和 JSON 原样透传。
- `latest?after=<frame_id>&wait_ms=<0..5000>` long-polls until a frame newer than `after` exists or `wait_ms` elapses (`"timeout":true`). The upstream caps `wait_ms` at 5000. armd waits `wait_ms` (capped at 5000) + 2 s for the upstream. 长轮询：等到比 `after` 新的帧或超时；armd 的上游超时为 `wait_ms`（上限 5000）+ 2 秒。
- Detector unreachable or timed out: `502 {"error":"detector unreachable"}`. 检测程序不可达或超时返回 502。
- No authentication, like the other `/api/v1` routes. `POST` is subject to the same browser same-origin check (cross-origin `Origin` → 403). Use only on a trusted network. 与其他 `/api/v1` 路由一致，无鉴权；`POST` 同样做浏览器同源校验，仅限可信网络。

The page's **Detect** button (video HUD, top-left in expert mode) reads and toggles `/api/v1/detection/inference`. It shows "N/A" and is disabled when the route fails (for example an older detector build). Turning detection off clears the overlay.

页面视频区的「识别」按钮读取并切换该接口；接口不可用时显示「不可用」并禁用。关闭识别会清空检测框。

## Agent loop example / Agent 调用示例

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
