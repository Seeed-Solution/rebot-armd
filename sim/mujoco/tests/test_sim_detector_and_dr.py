"""Tests for optional real-detector evaluation mode and deterministic domain randomization."""

from __future__ import annotations

import json
import math
import os
import socket
import sys
import threading
import time
import types
import urllib.error
import urllib.request
from pathlib import Path

import mujoco
import numpy as np
import pytest
from test_armd_sim import free_port
from test_perception import http, set_arm

from rebot_sim.camera import DETECTION_SIZE, CameraMount
from rebot_sim.model import load_model
from rebot_sim.perception import (
    CameraRig,
    DomainRandomizationConfig,
    OnnxDetector,
    PerceptionServer,
    SharedState,
    SimDomainRandomizer,
    SimLink,
)
from rebot_sim.server import Simulator, main as sim_main


def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    dot = float(np.clip(abs(np.dot(q1, q2)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


class _InProcessRigHarness:
    """Runs CameraRig + PerceptionServer + Simulator in-process for deterministic RGB/DR/HTTP testing."""

    def __init__(
        self,
        *,
        objects: str = "cube,cup",
        detector_mode: str = "gt",
        detector_fn=None,
        detector_labels=None,
        dr_config: DomainRandomizationConfig | None = None,
        fps: int = 15,
    ):
        self.mount = CameraMount()
        self.sim = Simulator(
            free_port(socket.SOCK_DGRAM),
            objects=objects,
            camera_mount=self.mount,
            dr_config=dr_config,
            detector_mode=detector_mode,
            detector_labels=detector_labels,
        )
        self.shared = SharedState(self.sim.model.nq, self.sim.model.nv)
        self.sim.shared = self.shared
        self.shared.write(self.sim.data)

        import multiprocessing as mp

        parent_conn, child_conn = mp.Pipe()
        self.sim.cam_conn = parent_conn
        cfg = {
            "fps": fps,
            "size": [640, 360],
            "host": "127.0.0.1",
            "video_port": 0,
            "perception_port": 0,
            "scene_stream": False,
            "objects": objects,
            "mount": self.mount.ipc(),
            "has_meshes": False,
            "noise": {},
            "depth": {},
            "mask_angle": True,
            "detector_mode": detector_mode,
            "detector_labels": list(detector_labels or []),
            "dr": dr_config.to_dict() if dr_config is not None else None,
        }
        self.rig = CameraRig(cfg, self.shared, SimLink(child_conn), detector_fn=detector_fn)
        self._stop = threading.Event()
        self._step_physics = threading.Event()
        self._cam_ready = threading.Event()
        self._cam_err: list[BaseException] = []

        def _sim_loop():
            dt = self.sim.model.opt.timestep
            while not self._stop.is_set():
                if self._step_physics.is_set():
                    self.sim.step()
                self.sim.service_camera()
                time.sleep(dt)

        def _cam_loop():
            try:
                self.rig.setup()
            except BaseException as exc:
                self._cam_err.append(exc)
                self._cam_ready.set()
                return
            self._cam_ready.set()
            period = 1.0 / self.rig.fps
            while not self._stop.is_set():
                if self._step_physics.is_set():
                    self.rig.process_once()
                else:
                    self.rig._flush_delayed()
                if self.rig.delayed:
                    lat_s = self.rig._effective_latency_ms() / 1000.0
                    rem = max(0.002, lat_s - (time.monotonic() - self.rig.delayed[0][2]))
                    time.sleep(min(period, rem))
                else:
                    time.sleep(period * 0.5)

        self._sim_thread = threading.Thread(target=_sim_loop, daemon=True)
        self._cam_thread = threading.Thread(target=_cam_loop, daemon=True)
        self._sim_thread.start()
        self._cam_thread.start()
        self._cam_ready.wait(timeout=30)
        if self._cam_err:
            exc = self._cam_err[0]
            self.close()
            msg = repr(exc)
            if any(tok in msg for tok in ("EGL", "GL", "x264", "Codec", "av.", "display")):
                pytest.skip(f"GL/encoder unavailable: {msg}")
            raise exc
        self.srv = PerceptionServer(self.rig.perception, self.rig, "127.0.0.1", 0)
        self.port = self.srv.httpd.server_address[1]
        self.base = f"http://127.0.0.1:{self.port}"

    def start_loops(self, *, step_physics: bool = True) -> None:
        if step_physics:
            self._step_physics.set()
        else:
            self._step_physics.clear()

    def close(self) -> None:
        self._stop.set()
        if hasattr(self, "_cam_thread") and self._cam_thread is not None:
            self._cam_thread.join(timeout=2)
        if hasattr(self, "_sim_thread") and self._sim_thread is not None:
            self._sim_thread.join(timeout=2)
        if hasattr(self, "srv"):
            self.srv.httpd.shutdown()
            self.srv.httpd.server_close()
        self.sim.close()


def test_default_detector_mode_is_gt_and_dr_disabled():
    h = _InProcessRigHarness(objects="cube")
    try:
        code, status = http(h.base + "/api/status")
        assert code == 200
        assert status["detector_mode"] == "gt"
        assert status["model"]["name"] == "rebot-sim-ground-truth"

        code, st = http(h.base + "/sim/state")
        assert code == 200
        assert st["detector_mode"] == "gt"
        assert "dr" not in st
    finally:
        h.close()


def test_callable_detector_receives_rendered_rgb_and_serves_api_detect():
    seen_frames: list[np.ndarray] = []

    def spy_detector(rgb: np.ndarray) -> list[dict]:
        assert isinstance(rgb, np.ndarray)
        assert rgb.shape == (DETECTION_SIZE[1], DETECTION_SIZE[0], 3)
        assert rgb.dtype == np.uint8
        assert int(rgb.std()) > 0 or float(rgb.std()) > 1.0
        seen_frames.append(rgb.copy())
        return [
            {
                "class_id": 0,
                "label": "custom_cube",
                "confidence": 0.88,
                "box_norm": [0.25, 0.30, 0.45, 0.50],
            }
        ]

    h = _InProcessRigHarness(
        objects="cube",
        detector_mode="callable",
        detector_fn=spy_detector,
    )
    try:
        # Warmup in setup() called spy_detector once on the initial pose
        assert len(seen_frames) >= 1
        code, status = http(h.base + "/api/status")
        assert code == 200 and status["detector_mode"] == "callable"
        code, st = http(h.base + "/sim/state")
        assert code == 200 and st["detector_mode"] == "callable"

        # Move the arm so the rendered wrist camera image changes noticeably
        set_arm(h.sim.model, h.sim.data, [0.46, -0.63, -1.05, 1.48, 0.43, -0.66])
        h.sim.step()
        h.start_loops(step_physics=True)

        code, det_res = http(h.base + "/api/detect", {"pipelines": ["yolo"], "wait_ms": 2500})
        assert code == 200 and det_res["success"] is True
        assert det_res["pipelines"]["yolo"]["ran"] is True
        assert len(det_res["detections"]) == 1
        d = det_res["detections"][0]
        assert d["class_name"] == "custom_cube"
        assert d["format"] == "xywh_center"
        assert d["confidence"] == pytest.approx(0.88, abs=1e-4)
        assert d["x"] == pytest.approx(0.35, abs=1e-4)
        assert d["y"] == pytest.approx(0.40, abs=1e-4)
        assert d["w"] == pytest.approx(0.20, abs=1e-4)
        assert d["h"] == pytest.approx(0.20, abs=1e-4)

        code, latest = http(h.base + "/api/results/latest")
        assert code == 200
        assert latest["frame_id"] == det_res["frame_id"]
        assert latest["detections"] == det_res["detections"]

        # Verify the RGB frame actually changed after moving the arm from initial pose
        assert len(seen_frames) >= 2
        assert not np.array_equal(seen_frames[0], seen_frames[-1])
    finally:
        h.close()


def test_onnx_detector_validation_and_mocked_inference(tmp_path, monkeypatch):
    # Missing model path / missing callable fail cleanly and close SharedState
    st_err = SharedState(10, 10)
    try:
        with pytest.raises(ValueError, match="requires detector_model"):
            CameraRig(
                {
                    "fps": 15,
                    "size": [640, 360],
                    "mount": CameraMount().ipc(),
                    "scene_stream": False,
                    "has_meshes": False,
                    "detector_mode": "onnx",
                },
                st_err,
                SimLink(None),
            )
        with pytest.raises(ValueError, match="requires a callable"):
            CameraRig(
                {
                    "fps": 15,
                    "size": [640, 360],
                    "mount": CameraMount().ipc(),
                    "scene_stream": False,
                    "has_meshes": False,
                    "detector_mode": "callable",
                },
                st_err,
                SimLink(None),
            )
    finally:
        st_err.close()

    fake_onnx = tmp_path / "yolo.onnx"
    fake_onnx.write_bytes(b"fake-onnx")

    class _FakeInput:
        name = "images"
        shape = [1, 3, 640, 640]

    scripted_outputs: list[np.ndarray] = []

    class _FakeSession:
        def __init__(self, path, providers=None):
            self.path = path
            self.seen_tensors = []

        def get_inputs(self):
            return [_FakeInput()]

        def run(self, output_names, feed):
            tensor = feed["images"]
            assert tensor.shape == (1, 3, 640, 640)
            assert tensor.dtype == np.float32
            assert 0.0 <= float(tensor.min()) and float(tensor.max()) <= 1.0
            self.seen_tensors.append(tensor)
            return [scripted_outputs.pop(0)]

    fake_ort = types.SimpleNamespace(
        get_available_providers=lambda: ["CPUExecutionProvider"],
        InferenceSession=_FakeSession,
    )
    monkeypatch.setitem(sys.modules, "onnxruntime", fake_ort)

    rgb = np.full((360, 640, 3), 128, dtype=np.uint8)

    # 1. End-to-end NMS output (1, N, 6): [x1, y1, x2, y2, score, class_id]
    det_nms = OnnxDetector(fake_onnx, labels=["cube", "cup"], conf_threshold=0.25, iou_threshold=0.45)
    scripted_outputs.append(
        np.array(
            [
                [
                    [160.0, 260.0, 320.0, 380.0, 0.91, 0.0],
                    [162.0, 262.0, 318.0, 378.0, 0.70, 0.0],  # suppressed by NMS
                    [10.0, 150.0, 50.0, 200.0, 0.10, 1.0],  # below conf_threshold
                ]
            ],
            dtype=np.float32,
        )
    )
    out = det_nms.detect(rgb)
    assert len(out) == 1
    assert out[0]["class_name"] == "cube"
    assert out[0]["confidence"] == pytest.approx(0.91, abs=1e-3)
    assert out[0]["x"] == pytest.approx(240.0 / 640.0, abs=0.01)
    assert out[0]["y"] == pytest.approx(180.0 / 360.0, abs=0.01)

    # 2. YOLOv5 multi-class (1, N, 5+C): obj=0.9, cls=[0.1, 0.8] -> class 1 ("cup") @ 0.72
    det_v5_multi = OnnxDetector(
        fake_onnx, labels=["cube", "cup"], conf_threshold=0.25, iou_threshold=0.45, output_format="yolov5"
    )
    scripted_outputs.append(
        np.array(
            [
                [
                    [240.0, 320.0, 100.0, 80.0, 0.9, 0.1, 0.8],
                    [100.0, 200.0, 40.0, 40.0, 0.3, 0.5, 0.2],  # 0.15 < 0.25
                ]
            ],
            dtype=np.float32,
        )
    )
    out_v5 = det_v5_multi.detect(rgb)
    assert len(out_v5) == 1
    assert out_v5[0]["class_id"] == 1 and out_v5[0]["class_name"] == "cup"
    assert out_v5[0]["confidence"] == pytest.approx(0.72, abs=1e-4)

    # 3. YOLOv5 single-class (1, N, 6) auto-detected via 1 label (5+1=6): obj=0.9, cls=[0.8] -> class 0 @ 0.72
    det_v5_single = OnnxDetector(fake_onnx, labels=["cube"], conf_threshold=0.25, iou_threshold=0.45)
    scripted_outputs.append(
        np.array(
            [
                [
                    [320.0, 320.0, 80.0, 80.0, 0.9, 0.8],
                    [100.0, 200.0, 40.0, 40.0, 0.4, 0.4],  # 0.16 < 0.25
                ]
            ],
            dtype=np.float32,
        )
    )
    out_v5_1 = det_v5_single.detect(rgb)
    assert len(out_v5_1) == 1
    assert out_v5_1[0]["class_id"] == 0 and out_v5_1[0]["class_name"] == "cube"
    assert out_v5_1[0]["confidence"] == pytest.approx(0.72, abs=1e-4)

    # 4. YOLOv8 transposed (1, 4+C, N)
    det_v8 = OnnxDetector(fake_onnx, labels=["cube", "cup"], conf_threshold=0.25, iou_threshold=0.45)
    arr_v8 = np.zeros((1, 6, 8), dtype=np.float32)
    arr_v8[0, :, 0] = [320.0, 320.0, 60.0, 60.0, 0.2, 0.85]
    scripted_outputs.append(arr_v8)
    out_v8 = det_v8.detect(rgb)
    assert len(out_v8) == 1
    assert out_v8[0]["class_id"] == 1 and out_v8[0]["class_name"] == "cup"
    assert out_v8[0]["confidence"] == pytest.approx(0.85, abs=1e-4)


def test_deterministic_lighting_and_camera_extrinsics_dr():
    dr_cfg = DomainRandomizationConfig(
        enabled=True,
        lighting=True,
        camera_jitter_m=0.005,
        camera_jitter_deg=2.0,
    )
    h = _InProcessRigHarness(objects="cube,cup", dr_config=None)
    try:
        # Capture exact nominal baseline before DR
        base_diffuse = h.rig.model.light_diffuse.copy()
        base_specular = h.rig.model.light_specular.copy()
        base_ambient = h.rig.model.light_ambient.copy()
        base_lpos = h.rig.model.light_pos.copy()
        base_cpos = h.rig.model.cam_pos.copy()
        base_cquat = h.rig.model.cam_quat.copy()

        # Enable DR and reset with seed=42
        code, res42_a = http(h.base + "/sim/reset", {"seed": 42, "dr": dr_cfg.to_dict()})
        assert code == 200 and "dr" in res42_a and res42_a["dr"]["seed"] == 42
        snap_42_a = {
            "light_diffuse": h.rig.model.light_diffuse.copy(),
            "light_specular": h.rig.model.light_specular.copy(),
            "light_ambient": h.rig.model.light_ambient.copy(),
            "light_pos": h.rig.model.light_pos.copy(),
            "cam_pos": h.rig.model.cam_pos.copy(),
            "cam_quat": h.rig.model.cam_quat.copy(),
        }
        # Verify perturbations happened and stay strictly within bounds
        assert not np.array_equal(snap_42_a["light_diffuse"], base_diffuse)
        assert not np.array_equal(snap_42_a["cam_pos"], base_cpos)
        assert not np.array_equal(snap_42_a["cam_quat"], base_cquat)
        for i in range(h.rig.model.ncam):
            pos_delta = float(np.linalg.norm(snap_42_a["cam_pos"][i] - base_cpos[i]))
            ang_delta = _quat_angle_deg(snap_42_a["cam_quat"][i], base_cquat[i])
            assert pos_delta <= 0.005 + 1e-12
            assert ang_delta <= 2.0 + 1e-6

        # Reset with a different seed=77 -> different perturbations within bounds
        code, res77 = http(h.base + "/sim/reset", {"seed": 77})
        assert code == 200 and res77["dr"]["seed"] == 77
        assert not np.array_equal(h.rig.model.light_diffuse, snap_42_a["light_diffuse"])
        assert not np.array_equal(h.rig.model.cam_pos, snap_42_a["cam_pos"])
        assert not np.array_equal(h.rig.model.cam_quat, snap_42_a["cam_quat"])
        for i in range(h.rig.model.ncam):
            assert float(np.linalg.norm(h.rig.model.cam_pos[i] - base_cpos[i])) <= 0.005 + 1e-12
            assert _quat_angle_deg(h.rig.model.cam_quat[i], base_cquat[i]) <= 2.0 + 1e-6

        # Reset back to seed=42 -> bit-identical to snap_42_a (generation increments monotonically)
        code, res42_b = http(h.base + "/sim/reset", {"seed": 42})
        assert code == 200
        assert res42_b["dr"]["generation"] > res42_a["dr"]["generation"]
        assert {k: v for k, v in res42_b["dr"].items() if k != "generation"} == {
            k: v for k, v in res42_a["dr"].items() if k != "generation"
        }
        assert np.array_equal(h.rig.model.light_diffuse, snap_42_a["light_diffuse"])
        assert np.array_equal(h.rig.model.light_specular, snap_42_a["light_specular"])
        assert np.array_equal(h.rig.model.light_ambient, snap_42_a["light_ambient"])
        assert np.array_equal(h.rig.model.light_pos, snap_42_a["light_pos"])
        assert np.array_equal(h.rig.model.cam_pos, snap_42_a["cam_pos"])
        assert np.array_equal(h.rig.model.cam_quat, snap_42_a["cam_quat"])

        # Verify GET /sim/state surfaces active DR summary
        code, st_on = http(h.base + "/sim/state")
        assert code == 200 and st_on["dr"] == res42_b["dr"]
        json.dumps(st_on["dr"])

        # Disabling DR via POST /sim/dr restores exact bit-identical nominal baseline
        code, st_off = http(h.base + "/sim/dr", {"enabled": False})
        assert code == 200 and "dr" not in st_off
        assert np.array_equal(h.rig.model.light_diffuse, base_diffuse)
        assert np.array_equal(h.rig.model.light_specular, base_specular)
        assert np.array_equal(h.rig.model.light_ambient, base_ambient)
        assert np.array_equal(h.rig.model.light_pos, base_lpos)
        assert np.array_equal(h.rig.model.cam_pos, base_cpos)
        assert np.array_equal(h.rig.model.cam_quat, base_cquat)
    finally:
        h.close()


def test_latency_dr_increases_reported_frame_lag_deterministically():
    h = _InProcessRigHarness(objects="cube", fps=15)
    try:
        h.start_loops(step_physics=True)

        # Baseline (DR disabled): low age_ms and low lag_s
        code, d0 = http(h.base + "/api/detect", {"pipelines": ["yolo"], "wait_ms": 2500})
        assert code == 200 and d0["success"] is True
        assert d0["age_ms"] < 100
        code, st0 = http(h.base + "/sim/state")
        assert code == 200 and "dr" not in st0
        assert st0["camera"]["lag_s"] is not None and st0["camera"]["lag_s"] < 0.1

        # Enable latency DR (160 ms > 2/15s = 133.3 ms stale gate)
        code, st_dr = http(h.base + "/sim/dr", {"enabled": True, "latency_ms": 160.0, "seed": 5})
        assert code == 200 and st_dr["dr"]["latency_ms"] == 160.0

        code, d1 = http(h.base + "/api/detect", {"pipelines": ["yolo"], "wait_ms": 2500})
        assert code == 200 and d1["success"] is True
        assert d1["age_ms"] >= 150

        code, lat = http(h.base + "/api/results/latest")
        assert code == 200 and lat["age_ms"] >= 150

        code, st1 = http(h.base + "/sim/state")
        assert code == 200
        assert st1["camera"]["lag_s"] >= 0.15
        assert st1["camera"]["delivery_delay_ms"] >= 150.0
        assert st1["camera"]["stale"] is True

        # Seeded latency_ms_range determinism
        code, r_a = http(h.base + "/sim/dr", {"enabled": True, "latency_ms_range": [60.0, 140.0], "seed": 11})
        code, r_b = http(h.base + "/sim/dr", {"enabled": True, "latency_ms_range": [60.0, 140.0], "seed": 12})
        code, r_a2 = http(h.base + "/sim/dr", {"enabled": True, "latency_ms_range": [60.0, 140.0], "seed": 11})
        assert 60.0 <= r_a["dr"]["latency_ms"] <= 140.0
        assert 60.0 <= r_b["dr"]["latency_ms"] <= 140.0
        assert r_a["dr"]["latency_ms"] == r_a2["dr"]["latency_ms"]
        assert r_a["dr"]["latency_ms"] != r_b["dr"]["latency_ms"]

        # Disabling DR restores low latency
        code, st_off = http(h.base + "/sim/dr", {"enabled": False})
        assert code == 200 and "dr" not in st_off
        code, d2 = http(h.base + "/api/detect", {"pipelines": ["yolo"], "wait_ms": 2500})
        assert code == 200 and d2["success"] is True and d2["age_ms"] < 100
    finally:
        h.close()


def test_cli_flags_for_detector_and_dr_parse_and_validate():
    # --detector onnx without --detector-model is rejected
    with pytest.raises(SystemExit) as exc:
        sim_main(["--detector", "onnx"])
    assert exc.value.code == 2

    # Negative DR jitter/latency rejected
    with pytest.raises(SystemExit) as exc:
        sim_main(["--dr-camera-jitter-m", "-0.01"])
    assert exc.value.code == 2

    with pytest.raises(SystemExit) as exc:
        sim_main(["--dr-latency-ms", "-5"])
    assert exc.value.code == 2

    # Valid benchmark run with DR & detector flags exits cleanly
    sim_main([
        "--benchmark", "5",
        "--detector", "gt",
        "--detector-labels", "cube,cup",
        "--dr-lighting",
        "--dr-camera-jitter-m", "0.005",
        "--dr-camera-jitter-deg", "2.0",
        "--dr-latency-ms", "50",
    ])


def test_multiprocess_simulator_dr_over_http():
    """End-to-end check with spawned camera process: DR config propagates and updates /sim/state."""
    dr_cfg = DomainRandomizationConfig(
        enabled=True,
        lighting=True,
        camera_jitter_m=0.004,
        camera_jitter_deg=1.5,
        latency_ms=40.0,
    )
    sim = Simulator(
        free_port(socket.SOCK_DGRAM),
        objects="cube",
        seed=9,
        camera_mount=CameraMount(),
        dr_config=dr_cfg,
    )
    pport = free_port()
    ready = sim.start_camera(15, (640, 360), "127.0.0.1", free_port(), pport, objects="cube")
    if "error" in ready:
        err = ready["error"]
        sim.close()
        if any(tok in err for tok in ("EGL", "GL", "x264", "Codec", "av.")):
            pytest.skip(err)
        pytest.fail(f"unexpected camera startup failure: {err}")
    runner = threading.Thread(target=sim.run, daemon=True)
    runner.start()
    base = f"http://127.0.0.1:{pport}"
    try:
        code, st = http(base + "/sim/state")
        assert code == 200
        assert st["detector_mode"] == "gt"
        assert st["dr"]["enabled"] is True
        assert st["dr"]["seed"] == 9
        assert st["dr"]["latency_ms"] == 40.0

        code, r1 = http(base + "/sim/reset", {"seed": 21})
        code, r2 = http(base + "/sim/reset", {"seed": 21})
        assert code == 200 and r2["dr"]["generation"] > r1["dr"]["generation"]
        assert {k: v for k, v in r1["dr"].items() if k != "generation"} == {
            k: v for k, v in r2["dr"].items() if k != "generation"
        }

        # Unseeded reset (seed=None) with DR enabled still syncs identical seed & generation across processes
        code, r_unseeded = http(base + "/sim/reset", {})
        assert code == 200 and r_unseeded["dr"]["enabled"] is True
        assert isinstance(r_unseeded["dr"]["seed"], int)
        assert r_unseeded["dr"]["generation"] == sim.dr_generation
        assert r_unseeded["dr"]["seed"] == sim.dr_seed

        code, off = http(base + "/sim/dr", {"enabled": False})
        assert code == 200 and "dr" not in off
    finally:
        sim.stopping = True
        runner.join(timeout=2)
        sim.close()


def test_http_detector_trust_boundary_and_preregistered_switching():
    def custom_fn(rgb: np.ndarray) -> list[dict]:
        return [{"class_id": 0, "class_name": "pre_reg", "confidence": 0.95, "x": 0.5, "y": 0.5, "w": 0.1, "h": 0.1}]

    h = _InProcessRigHarness(objects="cube", detector_mode="gt")
    try:
        h.rig.register_detector("spy_v1", "callable", fn=custom_fn)
        h.start_loops(step_physics=True)

        # Dynamic import / model path / callable injection over HTTP MUST be forbidden (403)
        for payload in (
            {"mode": "callable", "callable": "os:system"},
            {"mode": "callable", "detector_callable": "builtins:eval"},
            {"mode": "callable", "module": "os"},
            {"mode": "onnx", "model_path": "/etc/passwd"},
            {"mode": "onnx", "detector_model": "/tmp/evil.onnx"},
            {"mode": "onnx", "model": "/tmp/evil.onnx"},
        ):
            code, body = http(h.base + "/sim/detector", payload)
            assert code == 403 and body["code"] == "FORBIDDEN", (payload, code, body)

        # Unknown key rejected with 400
        code, _ = http(h.base + "/sim/detector", {"mode": "gt", "extra": 1})
        assert code == 400

        # Switching to unregistered mode fails with 400
        code, _ = http(h.base + "/sim/detector", {"mode": "onnx"})
        assert code == 400

        # Switching between 'gt' and pre-registered 'spy_v1' succeeds
        code, sw = http(h.base + "/sim/detector", {"detector_id": "spy_v1"})
        assert code == 200 and sw["detector_mode"] == "callable"
        code, d_res = http(h.base + "/api/detect", {"pipelines": ["yolo"], "wait_ms": 2500})
        assert code == 200 and d_res["success"] is True
        assert d_res["detections"][0]["class_name"] == "pre_reg"

        code, sw_gt = http(h.base + "/sim/detector", {"mode": "gt"})
        assert code == 200 and sw_gt["detector_mode"] == "gt"
    finally:
        h.close()


def test_real_detector_depth_and_noise_do_not_leak_ground_truth(monkeypatch):
    def box_det(rgb: np.ndarray) -> list[dict]:
        return [{"class_id": 0, "class_name": "arbitrary_label", "confidence": 0.9, "x": 0.5, "y": 0.5, "w": 0.15, "h": 0.15}]

    # Build a harness with depth enabled and DetNoise false_rate=1.0
    mount = CameraMount()
    sim = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", camera_mount=mount, detector_mode="callable")
    shared = SharedState(sim.model.nq, sim.model.nv)
    sim.shared = shared
    shared.write(sim.data)
    import multiprocessing as mp

    parent_conn, child_conn = mp.Pipe()
    sim.cam_conn = parent_conn
    cfg = {
        "fps": 15,
        "size": [640, 360],
        "host": "127.0.0.1",
        "video_port": 0,
        "perception_port": 0,
        "scene_stream": False,
        "objects": "cube",
        "mount": mount.ipc(),
        "has_meshes": False,
        "noise": {"false_rate": 1.0, "seed": 3},
        "depth": {"kind": "metric"},
        "mask_angle": True,
        "detector_mode": "callable",
    }
    rig = CameraRig(cfg, shared, SimLink(child_conn), detector_fn=box_det)
    srv = None
    stop = threading.Event()
    cam_ready = threading.Event()
    cam_err: list[BaseException] = []

    def _sim_loop():
        while not stop.is_set():
            sim.step()
            sim.service_camera()
            time.sleep(0.002)

    def _cam_loop():
        try:
            rig.setup()
        except BaseException as exc:
            cam_err.append(exc)
            cam_ready.set()
            return
        cam_ready.set()
        while not stop.is_set():
            rig.process_once()
            time.sleep(0.02)

    st_t = threading.Thread(target=_sim_loop, daemon=True)
    ct_t = threading.Thread(target=_cam_loop, daemon=True)
    st_t.start()
    ct_t.start()
    cam_ready.wait(timeout=30)
    try:
        if cam_err:
            raise cam_err[0]
        # Forbid GT render_segmentation in callable mode!
        monkeypatch.setattr(
            rig._cam,
            "render_segmentation",
            lambda *a, **kw: (_ for _ in ()).throw(AssertionError("render_segmentation must not be called in callable mode")),
        )
        srv = PerceptionServer(rig.perception, rig, "127.0.0.1", 0)
        base = f"http://127.0.0.1:{srv.httpd.server_address[1]}"
        code, res = http(base + "/api/detect", {"pipelines": ["yolo", "depth"], "wait_ms": 2500})
        assert code == 200 and res["success"] is True
        assert res["pipelines"]["yolo"]["ran"] is True
        assert res["pipelines"]["depth"]["ran"] is True
        assert "depth" in res and len(res["depth"]["table_samples"]) > 0
        # M2: false_rate=1.0 did NOT leak GT labels into callable detections
        assert len(res["detections"]) == 1
        assert res["detections"][0]["class_name"] == "arbitrary_label"
        assert "disp_fg" in res["detections"][0] and "disp_bg" in res["detections"][0]
    finally:
        stop.set()
        ct_t.join(timeout=2)
        st_t.join(timeout=2)
        if srv is not None:
            srv.httpd.shutdown()
            srv.httpd.server_close()
        sim.close()


def test_detector_switch_discards_delayed_frames_and_run_cadence(monkeypatch):
    # M3: delayed frames from old detector generation are discarded on switch
    mount = CameraMount()
    model = load_model(camera=mount)
    st = SharedState(model.nq, model.nv)
    cfg = {
        "fps": 15,
        "size": [640, 360],
        "mount": mount.ipc(),
        "scene_stream": False,
        "has_meshes": False,
        "detector_mode": "gt",
        "dr": {"enabled": True, "latency_ms": 200.0},
    }
    rig = CameraRig(cfg, st, SimLink(None))
    try:
        old_det = [{"class_id": 0, "class_name": "stale_gt", "confidence": 0.9, "x": 0.5, "y": 0.5, "w": 0.1, "h": 0.1}]
        rig.publish_frame((1, 1000, time.monotonic() - 0.05, old_det, 0.0, 1), detector_generation=rig._detector_generation)
        assert len(rig.delayed) == 1

        rig.set_detector("callable", fn=lambda rgb: [])
        assert len(rig.delayed) == 0
        # Even a late publish_frame stamped with old generation 0 is rejected
        rig.publish_frame((2, 1001, time.monotonic() - 0.5, old_det, 0.0, 2), detector_generation=0)
        assert len(rig.delayed) == 0 and rig.perception.frame is None

        # M8: CameraRig.run() waking early to deliver delayed frame must NOT trigger premature process_once
        captures = []
        rig._dr_latency_ms = 30.0
        rig.fps = 5  # 200 ms capture period vs 30 ms delivery delay
        rig.delayed.append((10, 1010, time.monotonic(), [], 0.0, 10))
        rig._delayed_gens.append(rig._detector_generation)

        def fake_process_once():
            captures.append(time.perf_counter())
            return True

        monkeypatch.setattr(rig, "process_once", fake_process_once)
        real_getppid = os.getppid
        parent_pid = real_getppid()
        start_t = time.perf_counter()
        monkeypatch.setattr(os, "getppid", lambda: parent_pid if time.perf_counter() - start_t < 0.12 else -1)
        t = threading.Thread(target=rig.run, args=(parent_pid,), daemon=True)
        t.start()
        t.join(timeout=2)
        # Only 1 capture at t~0, even though delayed frame woke run() at t~30ms!
        assert len(captures) == 1
        assert rig.perception.frame is not None and rig.perception.frame["frame_id"] == 10
    finally:
        st.close()


def test_reset_atomicity_start_camera_cleanup_and_low_light_clamping():
    # N1: low-mounted light z=0.20 with jitter=0.05 stays near 0.20 (not pushed to 0.40)
    m = load_model(camera=CameraMount())
    m.light_pos[0] = [0.0, 0.0, 0.20]
    dr = SimDomainRandomizer(m, DomainRandomizationConfig(enabled=True, lighting=True, light_pos_jitter_m=0.05))
    dr.apply(m, seed=3)
    assert 0.14 <= float(m.light_pos[0, 2]) <= 0.26

    # M9 & M10: invalid seed (bool True) leaves scene & DR untouched; unpicklable callable cleans up before shm alloc
    sim = Simulator(free_port(socket.SOCK_DGRAM), objects="cube", seed=5)
    try:
        with pytest.raises(ValueError, match="non-negative integer"):
            sim.request_reset(True)  # type: ignore[arg-type]
        assert sim.scene.seed == 5

        def unpicklable(rgb):
            return []

        with pytest.raises(Exception):
            sim.start_camera(
                15, (640, 360), "127.0.0.1", free_port(), free_port(),
                detector_mode="callable", detector_callable=unpicklable,
            )
        assert sim.shared is None and sim.cam_conn is None and sim.cam_proc is None
    finally:
        sim.close()
