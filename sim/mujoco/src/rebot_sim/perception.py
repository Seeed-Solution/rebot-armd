"""Wrist camera stream and perception API for rebot-sim.

- Video: `ws://<host>:<video-port>/` (default 8001) as the reCamera debug stream: one binary
  message per H.264 access unit (Annex-B, SPS + PPS before every IDR) followed by the capture
  time as 8-byte little-endian unix ms. A new client starts at a forced IDR. `/scene` serves
  the fixed overview camera when it is enabled.
- HTTP `:<perception-port>` (default 8090): rebot-vision perception API version 2
  (rebot-armd docs/detection_api.md) with detections computed from the simulator's ground
  truth, plus `POST /sim/reset`, `GET /sim/state`, `GET /sim/camera`.
"""
from __future__ import annotations

import itertools
import json
import os
import queue
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


def _now_ms() -> int:
    return int(time.time() * 1000)


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

    def __init__(self, max_fps: int):
        self.max_fps = max_fps
        self.mode, self.enabled, self.last_active = "off", False, "yolo"
        self.subs: dict[str, dict] = {}
        self._ids = itertools.count(1)
        self.cond = threading.Condition()
        self.frame: dict | None = None  # newest rendered frame: id, ts_ms, mono, detections
        self.latest: dict | None = None  # newest published result
        self.triggers = 0
        self.pending: list[dict] = []  # registered triggers: {"called": mono, "pipelines": [...]}
        self._gate = 0.0  # mono of the last frame the continuous rate let through
        self.frames_in: list[float] = []
        self.results_out: list[float] = []

    # ---- subscriptions
    def _expire(self) -> None:
        now = time.monotonic()
        for sid in [s for s, v in self.subs.items() if v["expires"] <= now]:
            del self.subs[sid]

    def continuous(self) -> dict:
        self._expire()
        pipes = set(MODE_PIPELINES[self.mode]) if self.enabled else set()
        fps = self.max_fps if pipes else 0
        for s in self.subs.values():
            pipes |= set(s["pipelines"])
            fps = max(fps, self.sub_fps(s))
        active = bool(pipes)
        return {"active": active, "pipelines": [p for p in PIPELINES if p in pipes],
                "fps": min(fps, self.max_fps) if active else 0, "subscriptions": len(self.subs)}

    def sub_fps(self, s: dict) -> int:
        """Effective rate of a subscription: its requested rate under the current cap."""
        return min(s["requested_fps"], self.max_fps)

    def sub_body(self, s: dict) -> dict:
        return {"id": s["id"], "client": s["client"], "pipelines": s["pipelines"], "fps": self.sub_fps(s),
                "requested_fps": s["requested_fps"], "ttl_ms": s["ttl_ms"],
                "expires_in_ms": max(0, int((s["expires"] - time.monotonic()) * 1000))}

    # ---- frames
    def on_frame(self, frame_id: int, ts_ms: int, captured: float, detections: list[dict],
                 physics_time: float = 0.0, physics_step: int = 0) -> None:
        """A rendered wrist frame (called by the render thread); `captured` = time.monotonic()
        when the simulator state (physics step `physics_step` at `physics_time` s) was copied."""
        now = time.monotonic()
        with self.cond:
            self.frame = {"frame_id": frame_id, "ts_ms": ts_ms, "mono": captured, "detections": detections,
                          "physics_time": round(physics_time, 6), "physics_step": physics_step}
            self.frames_in = [t for t in self.frames_in if now - t < 2.0] + [now]
            # One plan per frame, as rebot-vision's frame_plan: the continuous pipelines when the
            # rate lets this frame through, plus those of every trigger registered before it.
            cont, plan = self.continuous(), set()
            if cont["active"] and now - self._gate >= 1.0 / max(cont["fps"], 1) - 0.005:
                plan |= set(cont["pipelines"])
                self._gate = now
            for t in self.pending:
                if t["called"] < captured:
                    plan |= set(t["pipelines"])
            if plan:
                self.latest = self.result(self.frame, [p for p in PIPELINES if p in plan])
                self.results_out = [t for t in self.results_out if now - t < 2.0] + [now]
            self.cond.notify_all()

    def result(self, frame: dict, pipelines: list[str]) -> dict:
        ran = {p: p in pipelines for p in PIPELINES}
        body = {
            "api_version": API_VERSION, "success": True, "mode": self.mode, "frame_id": frame["frame_id"],
            "ts_ms": frame["ts_ms"], "mono": frame["mono"], "physics_time": frame["physics_time"],
            "physics_step": frame["physics_step"], "inference_time_ms": 0.1, "width": 640, "height": 360,
            "pipelines": {p: {"ran": ran[p], "ms": 0.1 if ran[p] else 0.0} for p in PIPELINES},
            "detections": frame["detections"] if ran["yolo"] else [],
            "count": 0, "family": "aruco4x4_50", "undistorted": False, "rejected": 0, "markers": [],
        }
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
        return {
            "api_version": API_VERSION, "mode": self.mode, "modes": list(MODES),
            "pipelines": {"yolo": True, "aruco": True}, "last_active_mode": self.last_active,
            "enabled": self.enabled, "processing": cont["active"], "fps": cont["fps"] or IDLE_FPS,
            "active_fps": self.max_fps, "idle_fps": IDLE_FPS, "fps_range": [1, 15], "camera_fps": self.max_fps,
            "measured": {"input_fps": round(len([t for t in self.frames_in if now - t < 2.0]) / 2, 1),
                         "result_fps": round(len([t for t in self.results_out if now - t < 2.0]) / 2, 1)},
            "trigger": True, "per_request_pipelines": True, "subscriptions": True, "max_fps": self.max_fps,
            "continuous": cont, "available": {"yolo": True, "aruco": True},
            "model": {"name": "rebot-sim-ground-truth", "path": None}, "family": "aruco4x4_50",
        }


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


def _pipelines(v) -> list[str]:
    if not isinstance(v, list) or not v or any(p not in PIPELINES for p in v):
        raise ValueError("pipelines must be a non-empty list of yolo / aruco")
    return [p for p in PIPELINES if p in v]


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
        if path == "/api/mode" and method == "GET":
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
            req = _pipelines(body["pipelines"]) if "pipelines" in body else None
            pipes = req if req is not None else (MODE_PIPELINES[p.mode] or MODE_PIPELINES[p.last_active])
            called = time.monotonic()
            after = p.latest["frame_id"] if p.latest else 0
            entry = {"called": called, "pipelines": pipes}
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
# its state in shared memory (seqlock) and answers /sim requests over a pipe.

HDR = 5  # shared float64 slots: seq, sim time, camera frames, camera ms/frame, physics step


class SharedState:
    """Simulator state in shared memory: [seq, time, frames, frame_ms, step, qpos..., qvel...].
    `step` counts the physics states written, so a reader can tell whether physics advanced."""

    def __init__(self, nq: int, nv: int, name: str | None = None):
        from multiprocessing import shared_memory

        size = (HDR + nq + nv) * 8
        self.shm = shared_memory.SharedMemory(name=name, create=name is None, size=size)
        self.arr = np.ndarray((HDR + nq + nv,), dtype=np.float64, buffer=self.shm.buf)
        self.nq, self.nv = nq, nv
        if name is None:
            self.arr[:] = 0.0

    def write(self, data) -> None:
        a = self.arr
        a[0] += 1  # odd: writing
        a[1] = data.time
        a[4] += 1
        a[HDR:HDR + self.nq] = data.qpos
        a[HDR + self.nq:] = data.qvel
        a[0] += 1

    def read(self, data) -> int | None:
        """Copy a consistent state into `data`; returns its physics step, or None (and leaves
        `data` alone) when nothing was written yet or no consistent copy was had."""
        a = self.arr
        for _ in range(1000):
            s1 = a[0]
            if s1 % 2:
                continue
            qpos, qvel, t, step = a[HDR:HDR + self.nq].copy(), a[HDR + self.nq:].copy(), a[1], a[4]
            if a[0] == s1:
                if s1 == 0:
                    return None
                data.qpos[:], data.qvel[:], data.time = qpos, qvel, t
                return int(step)
        return None


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
        self.perception = Perception(max_fps=min(15, self.fps))
        self.frame_id = 0
        self.physics_step: int | None = None  # physics step of the newest frame
        self.skipped = 0  # camera ticks without a new physics state (nothing published)
        self.render_ms: list[float] = []
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
            "grip_pixel_norm_at": {f"{h:.2f}": cam.grip_pixel_norm(r, m.offset, h) for h in (0.10, 0.12, 0.15, 0.18, 0.20)},
            "render_backend": self.backend, "render": self.stats(),
        }

    def run(self, parent_pid: int) -> None:
        import mujoco

        w, h = self.size
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, w)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, h)
        renderer = mujoco.Renderer(self.model, h, w)
        seg_renderer = mujoco.Renderer(self.model, self._cam.DETECTION_SIZE[1], self._cam.DETECTION_SIZE[0])
        seg_renderer.enable_segmentation_rendering()
        opt = mujoco.MjvOption()
        opt.sitegroup[:] = 0
        if self.cfg["has_meshes"]:
            opt.geomgroup[3] = 0  # box primitives hidden behind the upstream meshes
        encoders = {path: self._cam.H264Encoder(w, h, self.fps) for path in self.streams}
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
                ts_ms = _now_ms()
                mujoco.mj_forward(self.model, self.data)
                seg_renderer.update_scene(self.data, camera="wrist", scene_option=opt)
                dets = self._cam.detections(self._cam.segmentation_ids(seg_renderer.render()), self.scene)
                self.frame_id += 1
                self.perception.on_frame(self.frame_id, ts_ms, captured, dets, self.data.time, step)
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
        return {"frames": self.frame_id, "frame_ms_mean": round(float(ms.mean()), 2),
                "frame_ms_max": round(float(ms.max()), 2)}


def camera_main(cfg: dict, shm_name: str, nq: int, nv: int, conn, ready_conn, parent_pid: int) -> None:
    """Camera process entry: build the rig, start the servers, report readiness, render."""
    state = SharedState(nq, nv, shm_name)
    try:
        rig = CameraRig(cfg, state, SimLink(conn))
        import mujoco

        mujoco.Renderer(rig.model, 8, 8).close()  # fails here when no GL backend works
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
