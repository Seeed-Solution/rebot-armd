"""Wrist camera stream and perception API for rebot-sim.

- Video: `ws://<host>:<video-port>/` (default 8001) as the reCamera debug stream: one binary
  message per H.264 access unit (Annex-B, SPS + PPS before every IDR) followed by the capture
  time as 8-byte little-endian unix ms. A new client starts at a forced IDR. `/scene` serves
  the fixed overview camera when it is enabled.
- HTTP `:<perception-port>` (default 8090): rebot-vision perception API version 2
  (rebot-armd docs/detection_api.md) with detections computed from the simulator's ground
  truth, plus `POST /sim/reset`, `GET /sim/state`, `GET /sim/camera`. With a depth model
  (`--depth metric|relative`, camera.DepthModel) the trigger also accepts the `depth` pipeline
  (trigger-only, as rebot-vision on demand): detections then carry disp_fg / disp_bg / depth_conf
  / length_px / jaw_clear and the body a `depth` object with table_samples, computed from the
  rendered depth and the ground-truth masks (camera.depth_result). Box noise does not change
  the depth fields; detections without a mask (false detections) get none.
"""
from __future__ import annotations

import itertools
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np

API_VERSION = 2
PIPELINES = ("yolo", "aruco")
MODES = ("off", "yolo", "aruco", "both")
MAX_SUBSCRIPTIONS = 16
IDLE_FPS = 1
MODE_PIPELINES = {"off": [], "yolo": ["yolo"], "aruco": ["aruco"], "both": ["yolo", "aruco"]}


def _box_iou(b1: list[float], b2: list[float]) -> float:
    ix0 = max(b1[0], b2[0])
    iy0 = max(b1[1], b2[1])
    ix1 = min(b1[2], b2[2])
    iy1 = min(b1[3], b2[3])
    iw = max(0.0, ix1 - ix0)
    ih = max(0.0, iy1 - iy0)
    inter = iw * ih
    a1 = max(0.0, b1[2] - b1[0]) * max(0.0, b1[3] - b1[1])
    a2 = max(0.0, b2[2] - b2[0]) * max(0.0, b2[3] - b2[1])
    union = a1 + a2 - inter
    return inter / union if union > 0 else 0.0


def _apply_mask_focus(detections: list[dict], focus: dict | None) -> list[dict]:
    out = [dict(d) for d in detections]
    for d in out:
        d.pop("mask_focus", None)
    if not focus or "box_norm" not in focus:
        return out
    fbox = focus["box_norm"]
    flabel = focus.get("label")
    best_iou = 0.099999
    best_det = None
    for d in out:
        if flabel and d.get("class_name") != flabel and d.get("name") != flabel:
            continue
        if "x" not in d or "y" not in d or "w" not in d or "h" not in d:
            continue
        cx, cy, w, h = d["x"], d["y"], d["w"], d["h"]
        dbox = [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]
        iou = _box_iou(fbox, dbox)
        if iou > best_iou:
            best_iou = iou
            best_det = d
    if best_det is not None:
        best_det["mask_focus"] = True
    return out


# --------------------------------------------------------------------------- video


class VideoStream:
    """Fan-out of encoded access units to WebSocket clients of one path."""

    QUEUE = 60

    def __init__(self):
        self.clients: list[queue.Queue] = []
        self.lock = threading.Lock()
        self.want_idr = True
        self.sent = 0

    def add(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(self.QUEUE)
        with self.lock:
            self.clients.append(q)
            self.want_idr = True
        return q

    def remove(self, q: queue.Queue) -> None:
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    @property
    def active(self) -> bool:
        return bool(self.clients)

    def take_idr_request(self) -> bool:
        with self.lock:
            want, self.want_idr = self.want_idr, False
        return want

    def publish(self, au: bytes, ts_ms: int, idr: bool) -> None:
        msg = au + int(ts_ms).to_bytes(8, "little")
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            if getattr(q, "started", False) is False:
                if not idr:
                    continue
                q.started = True
            try:
                q.put_nowait(msg)
            except queue.Full:  # slow client: drop its backlog, restart it at the next IDR
                with q.mutex:
                    q.queue.clear()
                q.started = False
                with self.lock:
                    self.want_idr = True
        self.sent += 1


def serve_video(host: str, port: int, streams: dict[str, VideoStream]):
    """Start the WebSocket server in a thread; returns the server."""
    from websockets.exceptions import ConnectionClosed
    from websockets.sync.server import serve

    def handler(ws):
        path = urlparse(ws.request.path).path.rstrip("/") or "/"
        stream = streams.get(path)
        if stream is None:
            ws.close(1008, "unknown path")
            return
        q = stream.add()
        try:
            while True:
                try:
                    msg = q.get(timeout=1.0)
                except queue.Empty:
                    continue
                ws.send(msg)
        except ConnectionClosed:
            pass
        finally:
            stream.remove(q)

    server = serve(handler, host, port, compression=None, max_size=2**20)
    threading.Thread(target=server.serve_forever, name="video-ws", daemon=True).start()
    return server


# --------------------------------------------------------------------------- perception


class Perception:
    """Perception API v2 state: default subscription (mode), subscriptions, frames, triggers."""

    def __init__(self, max_fps: int, depth=None):
        self.max_fps = max_fps
        self.depth_model = depth if depth is not None and depth.enabled else None
        # "depth" only with a depth model: without one every body is as before
        self.pipelines = PIPELINES + ("depth",) if self.depth_model else PIPELINES
        self.mode, self.enabled, self.last_active = "off", False, "yolo"
        self.subs: dict[str, dict] = {}
        self._ids = itertools.count(1)
        self.cond = threading.Condition()
        self.frame: dict | None = None  # newest rendered frame: id, ts_ms, mono, detections
        self.latest: dict | None = None  # newest published result
        self.triggers = 0
        self.pending: list[dict] = []  # registered triggers: {"called": mono, "pipelines": [...]}
        self.mask_focus: dict | None = None
        self._gate = 0.0  # mono of the last frame the continuous rate let through
        self.frames_in: list[float] = []
        self.results_out: list[float] = []

    # ---- subscriptions
    def _expire(self) -> None:
        now = time.monotonic()
        for sid in [s for s, v in self.subs.items() if v["expires"] <= now]:
            del self.subs[sid]
        if self.mask_focus is not None and self.mask_focus["expires"] <= now:
            self.mask_focus = None

    def mask_focus_body(self) -> dict | None:
        self._expire()
        if self.mask_focus is None:
            return None
        now = time.monotonic()
        left = max(0, int((self.mask_focus["expires"] - now) * 1000))
        out = {
            "box_norm": list(self.mask_focus["box_norm"]),
            "ttl_ms": self.mask_focus["ttl_ms"],
            "expires_in_ms": left,
            "remaining_ttl_ms": left,
        }
        if self.mask_focus.get("label"):
            out["label"] = self.mask_focus["label"]
        return out

    def continuous(self) -> dict:
        self._expire()
        pipes = set(MODE_PIPELINES[self.mode]) if self.enabled else set()
        fps = self.max_fps if pipes else 0  # every term is capped at max_fps
        for s in self.subs.values():
            pipes |= set(s["pipelines"])
            fps = max(fps, self.sub_fps(s))
        return {"active": bool(pipes), "pipelines": [p for p in PIPELINES if p in pipes],
                "fps": fps, "subscriptions": len(self.subs)}

    def sub_fps(self, s: dict) -> int:
        """Effective rate of a subscription: its requested rate under the current cap."""
        return min(s["requested_fps"], self.max_fps)

    def sub_body(self, s: dict) -> dict:
        return {"id": s["id"], "client": s["client"], "pipelines": s["pipelines"], "fps": self.sub_fps(s),
                "requested_fps": s["requested_fps"], "ttl_ms": s["ttl_ms"],
                "expires_in_ms": max(0, int((s["expires"] - time.monotonic()) * 1000))}

    # ---- frames
    def wants_depth(self, captured: float) -> bool:
        """A registered trigger asks for depth on a frame captured at `captured` (the plan of
        that frame will include it): the render thread then renders depth for it."""
        with self.cond:
            return self.depth_model is not None and any(
                "depth" in t["pipelines"] and t["called"] < captured for t in self.pending)

    def on_frame(self, frame_id: int, ts_ms: int, captured: float, detections: list[dict],
                 physics_time: float = 0.0, physics_step: int = 0, depth: dict | None = None) -> None:
        """A rendered wrist frame (called by the render thread); `captured` = time.monotonic()
        when the simulator state (physics step `physics_step` at `physics_time` s) was copied.
        `depth`: camera.depth_result of the same render (with its physics_step), or None."""
        now = time.monotonic()
        with self.cond:
            self.frame = {"frame_id": frame_id, "ts_ms": ts_ms, "mono": captured, "detections": detections,
                          "physics_time": round(physics_time, 6), "physics_step": physics_step, "depth": depth}
            self.frames_in = [t for t in self.frames_in if now - t < 2.0] + [now]
            # One plan per frame, as rebot-vision's frame_plan: the continuous pipelines when the
            # rate lets this frame through, plus those of every trigger registered before it.
            cont, plan = self.continuous(), set()
            if cont["active"] and now - self._gate >= 1.0 / max(cont["fps"], 1) - 0.005:
                plan |= set(cont["pipelines"])
                self._gate = now
            frame_focus = None
            for t in self.pending:
                if t["called"] < captured:
                    plan |= set(t["pipelines"])
                    if t.get("mask_focus"):
                        frame_focus = t["mask_focus"]
            if frame_focus is None:
                self._expire()
                if self.mask_focus is not None:
                    frame_focus = self.mask_focus
            if plan:
                self.latest = self.result(self.frame, [p for p in self.pipelines if p in plan], focus=frame_focus)
                self.results_out = [t for t in self.results_out if now - t < 2.0] + [now]
            self.cond.notify_all()

    def result(self, frame: dict, pipelines: list[str], focus: dict | None = None) -> dict:
        ran = {p: p in pipelines for p in self.pipelines}
        depth = frame.get("depth")
        if "depth" in ran:  # depth runs only on a frame whose depth render belongs to it
            ran["depth"] = ran["depth"] and depth is not None and depth.get("physics_step") == frame["physics_step"]
        detections = _apply_mask_focus(frame["detections"], focus) if ran["yolo"] else []
        if ran.get("depth"):
            detections = [{**d, **depth["objects"][d.get("name")]} if d.get("name") in depth["objects"] else d
                          for d in detections]
        body = {
            "api_version": API_VERSION, "success": True, "mode": self.mode, "frame_id": frame["frame_id"],
            "ts_ms": frame["ts_ms"], "mono": frame["mono"], "physics_time": frame["physics_time"],
            "physics_step": frame["physics_step"], "inference_time_ms": 0.1, "width": 640, "height": 360,
            "pipelines": {p: {"ran": ran[p], "ms": 0.1 if ran[p] else 0.0} for p in self.pipelines},
            "detections": detections,
            "count": 0, "family": "aruco4x4_50", "undistorted": False, "rejected": 0, "markers": [],
        }
        if ran.get("depth"):
            from .camera import DEPTH_GRID  # mujoco: imported only where depth runs

            gw, gh = DEPTH_GRID
            body["depth"] = {"metric_model": self.depth_model.metric, "grid_w": gw, "grid_h": gh,
                             "table_samples": [list(t) for t in depth["table_samples"]]}
        return body

    @staticmethod
    def public(body: dict) -> dict:
        out = {k: v for k, v in body.items() if k != "mono"}
        if "mono" in body:
            out["age_ms"] = int((time.monotonic() - body["mono"]) * 1000)
        return out

    def mode_body(self) -> dict:
        cont = self.continuous()
        now = time.monotonic()
        body = {
            "api_version": API_VERSION, "mode": self.mode, "modes": list(MODES),
            "pipelines": {"yolo": True, "aruco": True}, "last_active_mode": self.last_active,
            "enabled": self.enabled, "processing": cont["active"], "fps": cont["fps"] or IDLE_FPS,
            "active_fps": self.max_fps, "idle_fps": IDLE_FPS, "fps_range": [1, 15], "camera_fps": self.max_fps,
            "measured": {"input_fps": round(len([t for t in self.frames_in if now - t < 2.0]) / 2, 1),
                         "result_fps": round(len([t for t in self.results_out if now - t < 2.0]) / 2, 1)},
            "trigger": True, "per_request_pipelines": True, "subscriptions": True, "max_fps": self.max_fps,
            "continuous": cont, "available": {"yolo": True, "aruco": True},
            "model": {"name": "rebot-sim-ground-truth", "path": None, "mask_max_boxes": 3}, "family": "aruco4x4_50",
            "mask_focus": self.mask_focus_body(),
        }
        if self.depth_model:
            dm = self.depth_model
            body["pipelines"]["depth"] = body["available"]["depth"] = True
            body["depth_model"] = {"name": f"rebot-sim-depth-{dm.kind}", "path": f"sim:{dm.kind}", "metric": dm.metric}
        return body


class _Handler(BaseHTTPRequestHandler):
    server_version = "rebot-sim"
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):  # quiet
        pass

    @property
    def app(self) -> "PerceptionServer":
        return self.server.app  # type: ignore[attr-defined]

    def _send(self, code: int, body: dict) -> None:
        data = json.dumps(body, separators=(",", ":")).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict | None:
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b""
        if not raw.strip():
            return {}
        try:
            v = json.loads(raw)
        except ValueError:
            return None
        return v if isinstance(v, dict) else None

    def do_GET(self):
        self._route("GET")

    def do_POST(self):
        self._route("POST")

    def do_DELETE(self):
        self._route("DELETE")

    def _route(self, method: str) -> None:
        url = urlparse(self.path)
        qs = {k: v[-1] for k, v in parse_qs(url.query).items()}
        body = self._body() if method == "POST" else {}
        if body is None:
            return self._send(400, {"error": "body must be a JSON object"})
        try:
            code, out = self.app.dispatch(method, url.path.rstrip("/") or "/", qs, body)
        except ValueError as exc:
            code, out = 400, {"error": str(exc)}
        except TimeoutError as exc:  # the simulator's physics loop did not answer
            code, out = 504, {"error": str(exc)}
        except (OSError, EOFError) as exc:  # simulator pipe closed
            code, out = 503, {"error": f"simulator unavailable: {exc!r}"}
        self._send(code, out)


def _wait_ms(qs: dict, body: dict, default: int) -> int:
    v = body.get("wait_ms", qs.get("wait_ms", default))
    try:
        v = int(v)
    except (TypeError, ValueError):
        raise ValueError("wait_ms must be an integer") from None
    return max(0, min(5000, v))


def _pipelines(v, allowed: tuple[str, ...] = PIPELINES) -> list[str]:
    if not isinstance(v, list) or not v or any(p not in allowed for p in v):
        raise ValueError(f"pipelines must be a non-empty list of {' / '.join(allowed)}")
    return [p for p in allowed if p in v]


class PerceptionServer:
    """HTTP routes over a Perception state and the simulator (reset / state / camera info)."""

    def __init__(self, perception: Perception, sim, host: str, port: int):
        self.p, self.sim = perception, sim
        self.httpd = ThreadingHTTPServer((host, port), _Handler)
        self.httpd.daemon_threads = True
        self.httpd.app = self  # type: ignore[attr-defined]
        threading.Thread(target=self.httpd.serve_forever, name="perception-http", daemon=True).start()

    def dispatch(self, method: str, path: str, qs: dict, body: dict) -> tuple[int, dict]:
        p = self.p
        if (path == "/api/mode" or path == "/api/status") and method == "GET":
            with p.cond:
                return 200, p.mode_body()
        if path == "/api/mode" and method == "POST":
            if not body or not set(body) <= {"mode", "enabled", "fps"}:
                raise ValueError("body needs mode, enabled or fps")
            if "mode" in body and body["mode"] not in MODES:
                raise ValueError("bad mode")
            if "enabled" in body and not isinstance(body["enabled"], bool):
                raise ValueError("enabled must be a bool")
            if "fps" in body and (isinstance(body["fps"], bool) or not isinstance(body["fps"], int)
                                  or not 1 <= body["fps"] <= 15):
                raise ValueError("fps must be 1..15")
            with p.cond:  # the whole body is valid: apply it
                if "mode" in body:
                    p.mode = body["mode"]
                    if p.mode == "off":
                        p.enabled = False
                    else:
                        p.last_active = p.mode
                if "enabled" in body:
                    p.enabled = body["enabled"]
                    if p.enabled and p.mode == "off":
                        p.mode = p.last_active
                if "fps" in body:
                    p.max_fps = body["fps"]
                return 200, p.mode_body()
        if path == "/api/mask_focus":
            if method == "GET":
                with p.cond:
                    return 200, {"api_version": API_VERSION, "mask_focus": p.mask_focus_body()}
            if method == "POST":
                with p.cond:
                    if not body or (isinstance(body.get("ttl_ms"), int) and body["ttl_ms"] == 0):
                        p.mask_focus = None
                        return 200, {"api_version": API_VERSION, "mask_focus": None}
                    box_norm = body.get("box_norm")
                    if not isinstance(box_norm, list) or len(box_norm) != 4 or not all(isinstance(x, (int, float)) for x in box_norm):
                        raise ValueError("box_norm must be a list of 4 numbers")
                    ttl = body.get("ttl_ms", 2000)
                    if not isinstance(ttl, int):
                        raise ValueError("ttl_ms must be an integer")
                    ttl = max(100, min(10000, ttl))
                    label = body.get("label")
                    if label is not None and not isinstance(label, str):
                        raise ValueError("label must be a string")
                    p.mask_focus = {
                        "box_norm": [float(x) for x in box_norm],
                        "ttl_ms": ttl,
                        "expires": time.monotonic() + ttl / 1000.0,
                        "label": label if label else None,
                    }
                    return 200, {"api_version": API_VERSION, "mask_focus": p.mask_focus_body()}
        if path == "/api/inference":
            with p.cond:
                if method == "POST":
                    if not isinstance(body.get("enabled"), bool):
                        raise ValueError("enabled must be a bool")
                    p.enabled = body["enabled"]
                    if p.enabled and p.mode == "off":
                        p.mode = p.last_active
                return 200, {"api_version": API_VERSION, "enabled": p.enabled,
                             "fps": p.max_fps if p.enabled else IDLE_FPS, "active_fps": p.max_fps, "idle_fps": IDLE_FPS}
        if path == "/api/trigger" and method == "POST":
            return self.trigger(qs, body)
        if path == "/api/results/latest" and method == "GET":
            return self.latest(qs)
        if path == "/api/subscribe" and method == "POST":
            return self.subscribe(body)
        if path.startswith("/api/subscribe/"):
            parts = path.split("/")
            sid = parts[3]
            with p.cond:
                p._expire()
                s = p.subs.get(sid)
                if s is None:
                    return 404, {"api_version": API_VERSION, "error": "unknown subscription"}
                if method == "POST" and len(parts) == 5 and parts[4] == "keepalive":
                    s["expires"] = time.monotonic() + s["ttl_ms"] / 1000
                    return 200, {"api_version": API_VERSION, **p.sub_body(s), "continuous": p.continuous()}
                if method == "DELETE" and len(parts) == 4:
                    del p.subs[sid]
                    return 200, {"api_version": API_VERSION, "deleted": sid, "continuous": p.continuous()}
        if path == "/api/subscriptions" and method == "GET":
            with p.cond:
                cont = p.continuous()
                return 200, {"api_version": API_VERSION, "max_fps": p.max_fps, "max_subscriptions": MAX_SUBSCRIPTIONS,
                             "default": {"mode": p.mode, "enabled": p.enabled, "fps": p.max_fps},
                             "continuous": cont, "subscriptions": [p.sub_body(s) for s in p.subs.values()]}
        if path == "/sim/reset" and method == "POST":
            seed = body.get("seed")
            if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
                raise ValueError("seed must be a non-negative integer or null")
            out = self.sim.request_reset(seed)
            return (500 if "error" in out else 200), out
        if path == "/sim/state" and method == "GET":
            return 200, self.sim.sim_state()
        if path == "/sim/camera" and method == "GET":
            return 200, self.sim.camera_info()
        return 404, {"error": "not found"}

    def trigger(self, qs: dict, body: dict) -> tuple[int, dict]:
        p = self.p
        wait = _wait_ms(qs, body, 2500)
        with p.cond:
            req = _pipelines(body["pipelines"], p.pipelines) if "pipelines" in body else None
            pipes = req if req is not None else (MODE_PIPELINES[p.mode] or MODE_PIPELINES[p.last_active])
            called = time.monotonic()
            after = p.latest["frame_id"] if p.latest else 0
            focus = None
            if "mask_focus" in body and isinstance(body["mask_focus"], dict):
                mf = body["mask_focus"]
                box_norm = mf.get("box_norm")
                if isinstance(box_norm, list) and len(box_norm) == 4 and all(isinstance(x, (int, float)) for x in box_norm):
                    label = mf.get("label")
                    focus = {
                        "box_norm": [float(x) for x in box_norm],
                        "label": label if isinstance(label, str) and label else None,
                    }
            entry = {"called": called, "pipelines": pipes, "mask_focus": focus}
            p.pending.append(entry)  # the next frame's plan includes these pipelines

            def fresh() -> bool:
                r = p.latest
                return (r is not None and r["frame_id"] > after and r["mono"] > called
                        and all(r["pipelines"][x]["ran"] for x in pipes))

            try:
                ok = p.cond.wait_for(fresh, wait / 1000)
            finally:
                p.pending.remove(entry)
            if not ok:
                return 200, {"api_version": API_VERSION, "success": False, "timeout": True, "count": 0,
                             "detections": [], "markers": []}
            p.triggers += 1
            return 200, p.public({**p.latest, "requested": pipes})

    def latest(self, qs: dict) -> tuple[int, dict]:
        p = self.p
        wait = _wait_ms(qs, {}, 0)
        try:
            after = int(qs.get("after", 0))
        except ValueError:
            raise ValueError("after must be an integer") from None
        with p.cond:
            ok = p.cond.wait_for(lambda: p.latest is not None and p.latest["frame_id"] > after, wait / 1000)
            enabled = p.continuous()["active"]
            if p.latest is None:
                return 200, {"api_version": API_VERSION, "enabled": enabled, "frame_id": 0, "ts_ms": 0, "age_ms": 0,
                             "inference_time_ms": 0, "detections": [], "markers": [], "timeout": True}
            out = p.public(p.latest)
            out.pop("success", None)
            out["enabled"] = enabled
            if not ok:
                out["timeout"] = True
            return 200, out

    def subscribe(self, body: dict) -> tuple[int, dict]:
        p = self.p
        client = body.get("client")
        if not isinstance(client, str) or not 1 <= len(client) <= 64:
            raise ValueError("client must be 1..64 characters")
        pipes = _pipelines(body.get("pipelines"))
        fps = body.get("fps", p.max_fps)
        ttl = body.get("ttl_ms", 3000)
        if not isinstance(fps, int) or not 1 <= fps <= 15:
            raise ValueError("fps must be 1..15")
        if not isinstance(ttl, int) or not 500 <= ttl <= 60000:
            raise ValueError("ttl_ms must be 500..60000")
        with p.cond:
            p._expire()
            s = next((s for s in p.subs.values() if s["client"] == client or s["id"] == body.get("id")), None)
            if s is None:
                if len(p.subs) >= MAX_SUBSCRIPTIONS:
                    return 503, {"api_version": API_VERSION, "error": "too many subscriptions"}
                s = {"id": f"s{next(p._ids)}"}
                p.subs[s["id"]] = s
            s.update(client=client, pipelines=pipes, requested_fps=fps, ttl_ms=ttl,
                     expires=time.monotonic() + ttl / 1000)
            return 200, {"api_version": API_VERSION, **p.sub_body(s), "continuous": p.continuous()}


# --------------------------------------------------------------------------- render process
#
# The camera runs in a child process (spawned, so it gets its own GL context and GIL): on a
# loaded host a render thread in the simulator process cut the 500 Hz physics loop to about
# 220 Hz, while a separate rendering process left it at 500 Hz. The simulator publishes
# its state in shared memory under a multiprocessing lock and answers /sim requests over a pipe.

HDR = 5  # shared float64 slots: writes, sim time, camera frames, camera ms/frame, physics step


class SharedState:
    """Simulator state in shared memory: [writes, time, frames, frame_ms, step, qpos..., qvel...].
    `step` counts the physics states written, so a reader can tell whether physics advanced.
    Writes and reads hold `lock` (a multiprocessing lock: it orders the memory accesses across
    processes, which a seqlock on plain numpy stores does not)."""

    def __init__(self, nq: int, nv: int, name: str | None = None, lock=None):
        from multiprocessing import shared_memory

        size = (HDR + nq + nv) * 8
        self.owner = name is None  # the creator unlinks; an attached reader is not tracked
        self.shm = shared_memory.SharedMemory(name=name, create=self.owner, size=size, track=self.owner)
        self.arr = np.ndarray((HDR + nq + nv,), dtype=np.float64, buffer=self.shm.buf)
        self.nq, self.nv = nq, nv
        if lock is None:
            import multiprocessing as mp

            lock = mp.get_context("spawn").Lock()
        self.lock = lock
        if self.owner:
            self.arr[:] = 0.0

    def close(self) -> None:
        """Release the mapping; the owner also unlinks the segment. Idempotent."""
        if self.arr is None:
            return
        self.arr = None
        self.shm.close()
        if self.owner:
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass

    LOCK_TIMEOUT_S = 0.2  # a process that died holding the lock never releases it

    def write(self, data) -> None:
        """Publish `data`; raises TimeoutError when the lock is not had within LOCK_TIMEOUT_S."""
        a = self.arr
        if not self.lock.acquire(timeout=self.LOCK_TIMEOUT_S):
            raise TimeoutError("shared-state lock timeout")
        try:
            a[0] += 1
            a[1] = data.time
            a[4] += 1
            a[HDR:HDR + self.nq] = data.qpos
            a[HDR + self.nq:] = data.qvel
        finally:
            self.lock.release()

    def read(self, data) -> int | None:
        """Copy the state into `data`; returns its physics step, or None (and leaves `data`
        alone) when nothing was written yet or the lock was not had within LOCK_TIMEOUT_S."""
        a = self.arr
        if not self.lock.acquire(timeout=self.LOCK_TIMEOUT_S):
            return None
        try:
            if a[0] == 0:
                return None
            data.qpos[:], data.qvel[:], data.time = a[HDR:HDR + self.nq], a[HDR + self.nq:], a[1]
            return int(a[4])
        finally:
            self.lock.release()


class SimLink:
    """Child side of the simulator pipe: /sim/reset and /sim/state go to the physics loop.
    Requests are `(id, op, *args)`, replies `(id, body)`; a reply to an earlier request that
    timed out is dropped, so it never answers a later one."""

    def __init__(self, conn):
        self.conn, self.lock = conn, threading.Lock()
        self._ids = itertools.count(1)

    def call(self, msg: tuple, timeout: float = 3.0) -> dict:
        with self.lock:
            while self.conn.poll():  # late replies to timed-out requests
                self.conn.recv()
            rid = next(self._ids)
            self.conn.send((rid, *msg))
            deadline = time.monotonic() + timeout
            while True:
                left = deadline - time.monotonic()
                if left <= 0 or not self.conn.poll(left):
                    raise TimeoutError("simulator did not answer")
                got, body = self.conn.recv()
                if got == rid:
                    return body


class CameraRig:
    """Renders the wrist (and optionally scene) camera from the shared state, computes
    ground-truth detections, encodes and publishes. Runs in the camera process."""

    def __init__(self, cfg: dict, state: SharedState, link: SimLink):
        import mujoco

        from . import camera as cam
        from .model import load_model
        from .scene import Scene, parse_objects

        self.cfg, self.state, self.link = cfg, state, link
        self.fps, self.size = cfg["fps"], tuple(cfg["size"])
        self.mount = cam.CameraMount(*cfg["mount"])
        objects = parse_objects(cfg["objects"])
        self.model = load_model(objects=objects, camera=self.mount)
        self.data = mujoco.MjData(self.model)
        self.scene = Scene(self.model, objects)
        self.streams = {"/": VideoStream()}
        if cfg["scene_stream"]:
            self.streams["/scene"] = VideoStream()
        self.depth_model = cam.DepthModel(**cfg.get("depth", {}))
        self.perception = Perception(max_fps=min(15, self.fps), depth=self.depth_model)
        self.noise = cam.DetNoise(**cfg.get("noise", {}))
        # False: a plain detection model (no mask): detections carry no angle_deg / angle_conf /
        # grasp_width_px (the depth pipeline still reports its depth_* fields)
        self.mask_angle = cfg.get("mask_angle", True)
        self.labels = [(i, o.kind) for i, o in enumerate(objects)]  # what a false detection may be
        self.delayed: list[tuple] = []  # frames (on_frame arguments) waiting out noise.latency_ms
        self.frame_id = 0
        self.physics_step: int | None = None  # physics step of the newest frame
        self.skipped = 0  # camera ticks without a new physics state (nothing published)
        self.seg_failures = 0  # segmentation renders that failed (frame published without a result)
        self.seg_consecutive = 0
        self.seg_last_error: str | None = None
        self.render_ms: list[float] = []
        self.depth_ms: list[float] = []  # depth render + fields, frames that had a depth trigger
        self.depth_failures = 0  # depth renders with non-finite or non-positive Z (frame gets no depth)
        self.frame_depth: dict | None = None  # depth_result of the newest detect(want_depth=True)
        self.backend = os.environ.get("MUJOCO_GL", "default")
        self._cam = cam

    # PerceptionServer's simulator interface
    def request_reset(self, seed):
        return self.link.call(("reset", seed))

    def sim_state(self):
        """The simulator's state plus the camera's: the newest frame's physics time and how far
        it is behind the physics now (`stale` past two camera periods)."""
        out = self.link.call(("state",))
        with self.perception.cond:
            frame = self.perception.frame
        lag = float(self.state.arr[1]) - frame["physics_time"] if frame else None
        out["camera"] = {"frame_id": frame["frame_id"] if frame else 0,
                         "physics_time": frame["physics_time"] if frame else None,
                         "physics_step": frame["physics_step"] if frame else None,
                         "lag_s": None if lag is None else round(lag, 6), "skipped": self.skipped,
                         "seg_failures": self.seg_failures, "seg_last_error": self.seg_last_error,
                         "stale": frame is None or lag > 2.0 / self.fps}
        return out

    def camera_info(self) -> dict:
        cam, m = self._cam, self.mount
        r = cam.mount_rotation(self.model, m)
        return {
            "camera_mount": {"flip": m.flip, "rot_deg": m.rot_deg, "tilt_deg": m.tilt_deg,
                             "offset_m": [round(float(v), 5) for v in m.offset]},
            "hfov_deg": round(cam.HFOV_DEG, 3), "vfov_deg": round(cam.VFOV_DEG, 3),
            "stream_size": list(self.size), "fps": self.fps, "detection_size": list(cam.DETECTION_SIZE),
            "fx_px": round(cam.focal_px(cam.DETECTION_SIZE[1]), 2),
            "grip_pixel_norm_at": {f"{h:.2f}": cam.grip_pixel_norm(r, m.offset, h) for h in (0.10, 0.12, 0.15, 0.18, 0.20, 0.25, 0.30)},
            "render_backend": self.backend, "render": self.stats(),
            "depth": {"kind": self.depth_model.kind, "scale": self.depth_model.scale, "noise_rel": self.depth_model.noise_rel,
                      "a": self.depth_model.a, "b": self.depth_model.b, "conf": self.depth_model.conf,
                      "grid": list(cam.DEPTH_GRID), "failures": self.depth_failures},
        }

    def setup(self) -> None:
        """Create the renderers and encoders (raises when no GL backend or libx264 works)."""
        import mujoco

        w, h = self.size
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, w)
        # + 1: render_segmentation's canary row above the detection image
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, h, self._cam.DETECTION_SIZE[1] + 1)
        self.renderer = mujoco.Renderer(self.model, h, w)
        # plain renderer: render_segmentation sets the ID-colour flags, checks a canary pixel and decodes with bounds checks
        self.seg_renderer = mujoco.Renderer(self.model, self._cam.DETECTION_SIZE[1], self._cam.DETECTION_SIZE[0])
        self._cam.check_renderer_internals(self.seg_renderer)  # fail at start-up, not on the first frame
        self.opt = mujoco.MjvOption()
        self.opt.sitegroup[:] = 0
        if self.cfg["has_meshes"]:
            self.opt.geomgroup[3] = 0  # box primitives hidden behind the upstream meshes
        self.encoders = {path: self._cam.H264Encoder(w, h, self.fps) for path in self.streams}
        # one RGB and one segmentation render succeed before readiness is reported
        self.state.read(self.data)
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera="wrist", scene_option=self.opt)
        self.renderer.render()
        while self.detect() is None:  # raises after SEG_MAX_CONSECUTIVE failures
            pass

    SEG_MAX_CONSECUTIVE = 10

    def detect(self, want_depth: bool = False, physics_step: int = 0) -> list[dict] | None:
        """Ground-truth detections of the wrist view of self.data, or None when the segmentation
        render failed (logged and counted; the frame then gets no result). Raises RuntimeError
        after SEG_MAX_CONSECUTIVE failures in a row. `want_depth` (with a depth model): a second
        pass on the same scene renders depth, and self.frame_depth gets its depth_result tagged
        with `physics_step` (None when the depth was not finite)."""
        self.frame_depth = None
        r = self.seg_renderer
        r.update_scene(self.data, camera="wrist", scene_option=self.opt)
        try:
            ids = self._cam.render_segmentation(r)
        except Exception as exc:  # SegmentationError (bad readback) or a GL error
            self.seg_failures += 1
            self.seg_consecutive += 1
            self.seg_last_error = repr(exc)
            print(f"rebot-sim camera: segmentation failed on frame {self.frame_id + 1} "
                  f"({self.seg_consecutive} in a row, {self.seg_failures} total): {exc!r}; "
                  f"diag {getattr(exc, 'diag', {})} nbody {self.model.nbody} ngeom {self.model.ngeom} "
                  f"scene.ngeom {r.scene.ngeom}", file=sys.stderr, flush=True)
            if self.seg_consecutive >= self.SEG_MAX_CONSECUTIVE:
                raise RuntimeError(f"segmentation failed on {self.seg_consecutive} consecutive frames: {exc!r}") from exc
            return None
        self.seg_consecutive = 0
        if want_depth and self.depth_model.enabled:
            t0 = time.perf_counter()
            z = self._cam.render_depth(r)  # same mjvScene as the segmentation: same physics state
            d = self._cam.depth_result(ids, z, self.scene, self.depth_model, self.frame_id + 1)
            if d["finite"]:
                self.frame_depth = {**d, "physics_step": physics_step}
            else:
                self.depth_failures += 1
                print(f"rebot-sim camera: depth render not finite on frame {self.frame_id + 1}", file=sys.stderr, flush=True)
            self.depth_ms = (self.depth_ms + [(time.perf_counter() - t0) * 1000])[-100:]
        return self._cam.detections(ids, self.scene)

    def publish_frame(self, frame: tuple) -> None:
        """Hand a frame (on_frame's arguments) to perception, noise.latency_ms after its capture:
        the frame keeps its capture time, so its age_ms includes the delay."""
        if self.noise.latency_ms <= 0:
            self.perception.on_frame(*frame)
            return
        self.delayed.append(frame)
        now = time.monotonic()
        while self.delayed and now - self.delayed[0][2] >= self.noise.latency_ms / 1000.0:
            self.perception.on_frame(*self.delayed.pop(0))

    def run(self, parent_pid: int) -> None:
        import mujoco

        renderer, opt, encoders = self.renderer, self.opt, self.encoders
        period = 1.0 / self.fps
        next_t = time.perf_counter()
        while os.getppid() == parent_pid:
            t0 = time.perf_counter()
            step = self.state.read(self.data)
            if step is None or step == self.physics_step:  # no new physics: publish nothing
                self.skipped += 1
            else:
                self.physics_step = step
                captured = time.monotonic()
                ts_ms = int(time.time() * 1000)
                mujoco.mj_forward(self.model, self.data)
                dets = self.detect(self.perception.wants_depth(captured), step)
                self.frame_id += 1
                if dets is not None:  # a failed segmentation: the video frame goes out, no result
                    dets = self._cam.perturb(dets, self.noise, self.frame_id, self.labels)
                    if not self.mask_angle:
                        dets = [{k: v for k, v in d.items() if k not in ("angle_deg", "angle_conf", "grasp_width_px")}
                                for d in dets]
                    self.publish_frame((self.frame_id, ts_ms, captured, dets, self.data.time, step, self.frame_depth))
                for path, stream in self.streams.items():
                    if not stream.active:
                        continue
                    renderer.update_scene(self.data, camera="wrist" if path == "/" else "scene", scene_option=opt)
                    au = encoders[path].encode(renderer.render(), keyframe=stream.take_idr_request())
                    if au:
                        stream.publish(au, ts_ms, 5 in _nal_types(au))
                self.render_ms = (self.render_ms + [(time.perf_counter() - t0) * 1000])[-100:]
                self.state.arr[2], self.state.arr[3] = self.frame_id, float(np.mean(self.render_ms))
            next_t += period
            now = time.perf_counter()
            if next_t > now:
                time.sleep(next_t - now)
            else:
                next_t = now

    def stats(self) -> dict:
        ms = np.array(self.render_ms) if self.render_ms else np.zeros(1)
        out = {"frames": self.frame_id, "frame_ms_mean": round(float(ms.mean()), 2),
               "frame_ms_max": round(float(ms.max()), 2)}
        if self.depth_ms:
            out["depth_ms_mean"] = round(float(np.mean(self.depth_ms)), 2)
            out["depth_frames"] = len(self.depth_ms)
        return out


def camera_main(cfg: dict, shm_name: str, lock, nq: int, nv: int, conn, ready_conn, parent_pid: int) -> None:
    """Camera process entry: build the rig with its renderers and encoders, start the servers,
    then report readiness and render."""
    state = SharedState(nq, nv, shm_name, lock)
    try:
        rig = CameraRig(cfg, state, SimLink(conn))
        rig.setup()  # renderers and encoders before readiness is reported
        serve_video(cfg["host"], cfg["video_port"], rig.streams)
        PerceptionServer(rig.perception, rig, cfg["host"], cfg["perception_port"])
    except Exception as exc:  # no GL backend, no libx264, port in use
        ready_conn.send({"error": f"camera init failed: {exc!r}"})
        return
    ready_conn.send({"backend": rig.backend})
    rig.run(parent_pid)


def _nal_types(au: bytes) -> list[int]:
    out, i = [], au.find(b"\x00\x00\x01")
    while 0 <= i < len(au) - 3:
        out.append(au[i + 3] & 0x1F)
        i = au.find(b"\x00\x00\x01", i + 3)
    return out
