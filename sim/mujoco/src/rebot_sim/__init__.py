"""MuJoCo simulator for armd `--follower sim`."""
import ctypes.util
import os
import sys

# Offscreen rendering backend, chosen before mujoco is imported (it reads MUJOCO_GL once):
# EGL when libEGL is present (GPU hosts), else OSMesa. An explicit MUJOCO_GL wins.
if "MUJOCO_GL" not in os.environ and sys.platform.startswith("linux"):
    if ctypes.util.find_library("EGL"):
        os.environ["MUJOCO_GL"] = "egl"
    elif ctypes.util.find_library("OSMesa"):
        os.environ["MUJOCO_GL"] = "osmesa"
