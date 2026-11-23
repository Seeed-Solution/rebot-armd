"""`GET /snapshot.jpg` on the video port (perception.SnapshotStore + serve_video): rebot-vision's
snapshot as armd's multi-view measurement reads it (640x640 letterbox of the 640x360 frame)."""

from __future__ import annotations

import io
import threading
import time
import urllib.request

import numpy as np
from PIL import Image
from test_armd_sim import free_port

from rebot_sim.perception import SnapshotStore, VideoStream, serve_video


def test_snapshot_route_serves_letterboxed_frames_that_differ_per_frame():
    snap = SnapshotStore()
    port = free_port()
    server = serve_video("127.0.0.1", port, {"/": VideoStream()}, snap)
    try:
        rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
        rgb[:, :, 0] = 200  # red frame

        # the rig renders while snapshots are asked for: feed two frames once the first request arrived
        def feed():
            while not snap.wanted():
                time.sleep(0.01)
            snap.put(rgb, 1)
            time.sleep(0.2)
            snap.put(rgb, 2)

        threading.Thread(target=feed, daemon=True).start()
        url = f"http://127.0.0.1:{port}/snapshot.jpg"
        with urllib.request.urlopen(url, timeout=5) as r:
            assert r.status == 200 and r.headers["Content-Type"] == "image/jpeg"
            first = r.read()
        img = Image.open(io.BytesIO(first))
        assert img.size == (640, 640)
        px = img.convert("RGB")
        assert px.getpixel((320, 320))[0] > 150  # the frame in the middle
        assert px.getpixel((320, 20)) == px.getpixel((320, 620))  # letterbox bars above and below
        time.sleep(0.4)
        with urllib.request.urlopen(url, timeout=5) as r:
            second = r.read()
        assert second != first  # same picture, next frame: the bytes differ (frame id in the JPEG comment)
        assert snap.wanted()
    finally:
        server.shutdown()
