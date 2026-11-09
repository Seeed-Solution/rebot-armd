# vision/ (perception provider interface notes)

A kit's camera process is optional. armd does not run inference; it proxies a detector's
HTTP API so the cockpit and agents use one port (8765). Current routes (full contract in
[docs/detection_api.md](../../../docs/detection_api.md), code in `src/api/proxy.rs`):

| armd route | Upstream route your process serves | Purpose |
|---|---|---|
| `GET` / `POST /api/v1/detection/inference` | `GET` / `POST /api/inference` | read / set continuous inference (`{"enabled":bool}`) |
| `GET /api/v1/detection/latest` | `GET /api/results/latest?after=&wait_ms=` | latest result, long poll for a frame newer than `after` |
| `POST /api/v1/detection/trigger` | `POST /api/trigger?wait_ms=` | one inference on a frame newer than the call; does not change `enabled` |
| `GET /api/v1/detection/ws`, `/results` | WebSocket `ws://127.0.0.1:8001/results` | detection stream (fixed address, not configurable) |

- Upstream base: `ARMD_DETECTOR_URL` (default `http://127.0.0.1:8090`, `http://` only).
- Status code and JSON body are passed through unchanged; an unreachable upstream gives
  `502 {"error":"detector unreachable"}`. POST bodies are sent as `application/json`.
- armd waits `wait_ms` (capped at 5000; trigger default 2500) + 2 s for the upstream.
- A detector without `/api/trigger` returns 404; the cockpit then falls back to enable ->
  `latest` -> disable.
- Video is not proxied: the cockpit plays an H.264 WebSocket stream from
  `ws://<host>:8001/` directly (JMuxer).

`capabilities.vision.detection_proxy` in the schema is `true` in every armd build (the routes
exist); it does not tell whether your process is reachable.

Optional, perception API version 2 (the reCamera kit's `rebot-vision` implements it): serve
`GET /api/mode` with `"api_version": 2, "per_request_pipelines": true, "subscriptions": true`,
run the `{"pipelines": [...]}` a trigger body names, and serve `/api/subscribe`,
`/api/subscribe/{id}/keepalive`, `DELETE /api/subscribe/{id}` and `/api/subscriptions`. armd then
reports `capabilities.vision.per_request_pipelines` / `subscriptions`, the grasp triggers
`["yolo"]` per centring step, and the cockpit holds its own subscriptions instead of setting a
global mode. Contract: [docs/detection_api.md](../../../docs/detection_api.md).
