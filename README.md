# rebot-armd

<div align="center">

### Drive a 7-motor robot arm from a 40 mm camera

**`armd` is a 4 MB Rust runtime. On a single-core RISC-V camera chip it holds all 7 motors at 50 Hz with 7 % of the core and 3.7 MB of RAM, and serves a web cockpit and an AI-agent API, while the same chip streams video and runs object detection.**

[![Binary](https://img.shields.io/badge/binary-4.1_MB_static-brightgreen.svg?style=flat-square)](#hardware)
[![RAM](https://img.shields.io/badge/RAM-3.7_MB-brightgreen.svg?style=flat-square)](#hardware)
[![CPU](https://img.shields.io/badge/CPU-7%25_of_1_RISC--V_core-brightgreen.svg?style=flat-square)](#hardware)
[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg?style=flat-square&logo=rust)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)
[![CI](https://img.shields.io/github/actions/workflow/status/Seeed-Solution/rebot-armd/ci.yml?branch=main&style=flat-square&label=CI)](https://github.com/Seeed-Solution/rebot-armd/actions)
[![Target Arm](https://img.shields.io/badge/Robot-reBot_B601--DM-orange.svg?style=flat-square)](https://github.com/Seeed-Projects/reBot-DevArm)
[![Platform](https://img.shields.io/badge/Platform-Seeed_reCamera_SG2002-red.svg?style=flat-square)](https://www.seeedstudio.com)

English | [中文](./README.zh-CN.md)

<!-- TODO: Add demo GIF showing Web cockpit teleoperation here -->

</div>

---

`armd` is an arm-control runtime: a fixed-rate control loop with a safety supervisor, motor
drivers, and an HTTP/WebSocket server on port 8765, built as a single static binary (4.1 MiB
for RISC-V). It needs no Python, ROS or GPU on the controller.

The reference platform shows how small that controller can be: `armd` runs on a Seeed
reCamera (SG2002 RISC-V SoC, about 180 MB of RAM available to Linux) next to the camera's own
video and detection services, and drives the 7 Damiao motors of a reBot B601-DM over the CAN
bus of the reCamera B4 board. The same code builds and passes its tests on x86_64 Linux (CI);
other hosts and arms are listed under [Hardware](#hardware) with their verification status.

| | Measured on the reference platform (Seeed reCamera + reBot B601-DM) |
|---|---|
| Controller | Seeed reCamera: SG2002, **1 RISC-V core** (1 GHz per SoC spec), **180 MB** RAM visible to Linux, 40 × 40 mm camera body |
| `armd` binary | **4.1 MB**, static, no Python, ROS or GPU; installed as a **2.0 MB** `.deb` |
| `armd` memory | **3.7 MB** resident (VmRSS) |
| `armd` CPU | **7.0 %** of the core with all 7 motors torqued at 50 Hz; 4.5 % with torque off (10 s averages) |
| Whole chip, arm torqued | **31 %** busy, including the camera's video and detection, so about 69 % of the core is idle |
| Arm | **7** Damiao motors (6 joints + gripper) on CAN at 1 Mbit/s, **50 Hz** control loop |
| Same chip, same time | H.264 video, YOLO object detection and ArUco markers from `rebot-vision`, on demand (see [Vision](#vision-recamera-kit)) |

`armd` provides:

- a single-file web cockpit for phones, tablets and desktops (joysticks, Cartesian control,
  camera video, detection overlay, hand-eye calibration, tap-to-grasp);
- a REST API for observation and a WebSocket API for control, where actuator commands are
  accepted only from the connection that holds the control lease;
- high-level actions for agents (`move_to`, `move_relative`, `gripper`, `grasp`), used by
  [rebot-companion](https://github.com/Seeed-Solution/rebot-companion) and its MCP server;
- recording of joint state and commands to Parquet files;
- object detection (YOLO, on the TPU) and ArUco markers with 3D pose (on the CPU), run only
  when a caller asks for them ([Vision](#vision-recamera-kit));
- a gravity-compensation mode for hand-guided demonstrations, verified on the B601-DM.

> This software moves a real arm. Read [Safety](#safety) before running it on hardware.

**Names**: the repository is `rebot-armd`, the Cargo package and the binary are `armd`, and
the Debian package that installs it on the reCamera is `rebot-teleop` (init script
`/etc/init.d/K92rebot-teleop`, data under `/usr/share/rebot-teleop`).

---

## Contents

- [Hardware](#hardware)
- [Architecture](#architecture)
- [Install on reCamera](#install-on-recamera)
- [Build from source](#build-from-source)
- [Web cockpit](#web-cockpit)
- [Vision (reCamera kit)](#vision-recamera-kit)
- [API overview](#api-overview)
- [Agent and MCP integration](#agent-and-mcp-integration)
- [Episode recording (Parquet)](#episode-recording-parquet)
- [Configuration and CLI](#configuration-and-cli)
- [Wiring (reCamera B4 CAN board)](#wiring-recamera-b4-can-board)
- [Safety](#safety)
- [Development](#development)
- [Acknowledgements](#acknowledgements)
- [License](#license)

---

## Hardware

### Platforms

| Platform | Status |
|---|---|
| Seeed reCamera (SG2002, RISC-V, musl static build) with the B4 CAN board | verified with a reBot B601-DM (reference platform) |
| x86_64 Linux | builds and passes `cargo test` in CI, with the `mock` follower; not run against a real arm |
| Other Linux hosts with a SocketCAN interface (for example a USB-CAN adapter that appears as `can0`) or a serial bus | expected to work from the code; not verified |

### Arms

Verified on real hardware:

| Item | Details |
|---|---|
| Arm | reBot B601-DM: 6 arm joints and a gripper, all Damiao motors, 7 joints in armd (index 6 = gripper) |
| Controller | Seeed reCamera (SG2002) with the B4 CAN board, `can0` at 1 Mbit/s (the init script configures it) |
| Motor mode | Damiao `POS_VEL` (position and velocity limit), the default and only mode selectable from the CLI |

Under verification (drivers exist in the code; not run on a real arm, not supported yet):

| `--follower` | Driver | Bus | Notes |
|---|---|---|---|
| `rebot-rs` | RobStride (MIT mode, 29-bit CAN IDs) | SocketCAN | 6 joints; no enable confirmation, so automatic disable and fault clearing are refused |
| `so-arm101` | Feetech STS (SO-ARM101) | serial, 1,000,000 baud | 6 joints; position, velocity and temperature feedback, no torque or enable feedback |
| `fashionstar`, `star-arm102` | FashionStar | serial, 115,200 baud | 6 joints; the driver declares no feedback capabilities |

`mock` (in-process simulation, the default) is used by the tests and for UI work without an arm.

The protocol encoders of the drivers under verification are covered by unit tests
(`tests/test_protocol_vectors.rs`, `tests/test_driver_capabilities.rs`). They use the generic
6-joint profile without kinematics or a workspace, so Cartesian actions return `UNSUPPORTED`.
The Damiao driver also has an MIT mode with gravity feed-forward, but it can only be enabled
from code (`DamiaoArmDriver::with_control_mode`), not from the CLI or a profile. The
gravity-compensation mode switches J1–J6 to MIT at runtime and back (see
[Web cockpit](#web-cockpit)); that switch is verified on the B601-DM.

TODO / roadmap:

- [ ] Verify `rebot-rs` (RobStride), `so-arm101` (Feetech STS) and `fashionstar` / `star-arm102`
      (FashionStar) on real arms, with profiles that include kinematics and a workspace.
- [x] Verify the gravity-compensation mode (Damiao MIT switch, gravity feed-forward) on the B601-DM
      ([checklist](docs/control_safety_contract.md#hardware-verification-checklist--实机核实清单)).
- [ ] Leader-follower teleoperation (not implemented; `--leader` / `--leader-port` are rejected).

---

## Architecture

```text
 browser cockpit / rebot-companion / scripts
        │  HTTP GET (observation)      WebSocket /api/v1/ws (lease, commands, telemetry)
        ▼
┌───────────────────────────── armd :8765 ──────────────────────────────┐
│ axum HTTP/WS server ── detection proxy ──► camera detector :8090      │
│        │                                                               │
│ ArmSupervisor (one lock)                                               │
│   lease + epoch checks → planner (quintic / S-curve, Cartesian IK)     │
│   safety state machine: Booting, Idle, Armed, Moving, EStop, Fault     │
│   watchdog, joint limits, workspace box, speed caps, tracking error    │
│        │ control tick, 50 Hz default (SCHED_FIFO 50 when permitted)    │
│ driver: Damiao / RobStride (SocketCAN), Feetech / FashionStar (serial) │
└────────────────────────────────────────────────────────────────────────┘
        │ CAN 1 Mbit/s
        ▼
   reBot B601-DM motors
```

- **Control loop**: a fixed-period tick (`control_frequency_hz`, 50 Hz in the built-in
  profiles, accepted range 1..1000). A tick later than 80 ms latches a safety hold instead of
  continuing an old trajectory. Video is not handled by armd: the cockpit plays the camera's
  H.264 WebSocket stream (`ws://<host>:8001/`) directly with JMuxer.
- **Lease**: actuator authority belongs to the WebSocket connection that sent `acquire_lease`
  and keeps sending `heartbeat` (lease timeout 2500 ms). Every `command` carries the lease
  `generation`, the current safety `epoch_id` and a strictly increasing `request_id`. REST
  actuator routes (`torque`, `motion/*`, `action/*`, `clear_fault`) return
  `409 LEASE_REQUIRED` and do nothing. E-stop is the exception: it is accepted without a lease
  on REST and WebSocket. Another client can take over with `force`, after which motion is
  refused (`HANDOVER_SETTLING`) until the arm has settled.
- **Safety layers** (details in [docs/control_safety_contract.md](docs/control_safety_contract.md)):
  joint position, velocity and acceleration limits; temperature trip at `max_temperature`
  (70 °C in the built-in profiles); feedback freshness (250 ms); following-error fault
  (`max_error_rad` 0.5 for 500 ms) and `TRACKING_STALL` for actions; planned-motion speed cap
  (`planned_max_velocity`, 0.45 rad/s on rebot-dm); Cartesian workspace check (floor, box,
  reach); lease watchdog that parks the arm; refusal to cut torque away from the resting pose
  (`ARM_NOT_AT_RESTING_POSE`), so the arm is not dropped.

### Core, kits and agents

The repository has three layers: the **core** (`armd`, this crate), **kits** that adapt it to
one controller + arm + camera + mount (`kits/recamera-b601dm/` is the official reference kit,
`kits/_template/` the start for third-party kits), and the **agent layer**
(rebot-companion, the cockpit) that uses only armd's API. Core and kits interact through four
versioned contracts: the arm profile JSON (`--config`, `profile_version` 1), the perception
provider API (the detection proxy), capability discovery (`capabilities` in the observation
schema, version 1) and packaging. `./scripts/build_deb.sh --flavor lite` builds the core
alone, the default `--flavor recamera` the core with the reCamera kit. Details:
[docs/architecture.md](docs/architecture.md).

---

## Install on reCamera

Build the package (see [Cross-compile](#cross-compile-for-recamera)) or take a release `.deb`,
copy it to the camera and install it. `192.168.42.1` is the reCamera's address over USB;
when connected to the access point that this package starts, the camera is `192.168.16.1`.

```bash
scp dist/rebot-teleop_1.3.4_riscv64.deb root@192.168.42.1:/tmp/
ssh root@192.168.42.1 "dpkg -i /tmp/rebot-teleop_1.3.4_riscv64.deb && /etc/init.d/K92rebot-teleop restart"
```

The package installs:

| Path | Content |
|---|---|
| `/usr/bin/armd` (symlink `/usr/local/bin/armd`) | the daemon |
| `/usr/bin/rebot-vision` | camera process: H.264 video `:8001`, YOLO / ArUco API `:8090` ([kits/recamera-b601dm/vision](kits/recamera-b601dm/vision/)) |
| `/etc/init.d/K92rebot-teleop` (`postinst` adds `S92rebot-teleop` only on devices without the reCamera console) | init script: Wi-Fi, `can0` setup, start / stop of armd and rebot-vision |
| `/usr/share/rebot-teleop/` | cockpit `index.html`, URDF, app manifest `rebot-teleop.json` for the reCamera app list, arm profile `profile/rebot-b601dm.json` |
| `/userdata/local/apps/rebot-teleop.json` | copied by `postinst` |

The init script starts `armd --follower rebot-dm --follower-port can0 --port 8765` (plus
`--config /usr/share/rebot-teleop/profile/rebot-b601dm.json` when that file exists), sets
`can0` to 1 Mbit/s and starts a Wi-Fi access point on `wlan1` (SSID `reCamera-Arm-Teleop`,
default password `12345678`, address `192.168.16.1`). It can also join an existing Wi-Fi
network. These values are read from `/userdata/local/apps/rebot-teleop.config.json`
(`arm.follower_model`, `arm.serial_port`, `port`, `network.ap_ssid`, `network.ap_password`,
`network.ap_ip`, `network.ap_band`, `network.ap_channel`, `network.sta_ssid`,
`network.sta_password`). **Change the default access point password** before using the arm
outside a lab. Logs go to `/var/log/rebot-teleop.log`.

After armd the init script starts `rebot-vision` (nice 10, restarted with backoff by
`/usr/share/rebot-teleop/rebot-vision-run`, log `/var/log/rebot-vision.log`). Before that it
stops a camera app that is running (for example `yolo-detector` or `aruco-detector`) with that
app's own init script; other apps are not disabled or removed. Activating another camera app in
the reCamera console stops this app. Vision settings: `/userdata/rebot-teleop/vision.json`. The
YOLO model is not in the package; the model already on the device is used (see the vision
README). Without a model, YOLO modes are unavailable and ArUco works.

The access point uses 5 GHz by default (`network.ap_band: "5g"`, channel 44, 802.11n/ac,
20 MHz). Set `network.ap_band` to `"2g4"` for 2.4 GHz (channel 6). `network.ap_channel`
overrides the channel; `0` or unset keeps the default. If hostapd is not running on the 5 GHz
channel after start (checked with `iw dev wlan1 info`), the init script restarts it on
2.4 GHz channel 6 and writes the fallback to syslog.

Open the cockpit at `http://<camera-address>:8765/`.

armd reaches the vision process at `http://127.0.0.1:8090` (`ARMD_DETECTOR_URL` to change it).
With `--flavor lite` there is no vision process; detection, calibration and grasp are then
unavailable and the rest of the cockpit works.

---

## Build from source

Requirements: a Rust toolchain (stable, edition 2021). Node.js is only needed for the web
self-tests.

```bash
cargo build --release
cargo test

# Run with the simulated arm and open http://localhost:8765/
cargo run --release -- --follower mock
```

On Linux with a CAN adapter the real arm can be driven from a PC as well:
`cargo run --release -- --follower rebot-dm --follower-port can0` (bring `can0` up at
1 Mbit/s first). Only the reCamera setup has been tested.

### Cross-compile for reCamera

Target `riscv64gc-unknown-linux-musl`, statically linked. The linker is `rust-lld`; the C
runtime libraries come from the SG2002 musl toolchain in
[sophgo/host-tools](https://github.com/sophgo/host-tools) (`gcc/riscv64-linux-musl-x86_64`,
its `lib64v0p7_xthead/lp64d` libraries). Only the libraries are used, so the x86_64 toolchain
also works on an arm64 build host.

```bash
rustup target add riscv64gc-unknown-linux-musl
git clone --depth 1 https://github.com/sophgo/host-tools
export RECAMERA_TOOLCHAIN=$PWD/host-tools/gcc/riscv64-linux-musl-x86_64

RECAMERA_DOCKER=tpuc_recamera \
./scripts/build_deb.sh 1.3.4          # version argument optional (default: debian/control)
# -> target/riscv64gc-unknown-linux-musl/release/armd
# -> dist/rebot-teleop_1.3.4_riscv64.deb

./scripts/build_deb.sh --flavor lite  # armd only, no reCamera kit files
# -> dist/armd-lite_0.1.0_riscv64.tar.gz (armd, LICENSE, THIRD_PARTY_NOTICES.md)
```

`--flavor recamera` needs `rebot-vision`, a C++ program built against the SG2002 SDK:
`RECAMERA_DOCKER=<container>` builds it with `scripts/build_vision.sh` in a running
`sophgo/tpuc_dev` container that has the host-tools, the reCamera OS SDK and
`sscma-example-sg200x` (at the commit in `kits/recamera-b601dm/vision/sscma.lock`) under
`/recamera`; `REBOT_VISION_BIN=<path>` uses a prebuilt binary. Details:
[kits/recamera-b601dm/vision/README.md](kits/recamera-b601dm/vision/README.md).

`--flavor recamera` (the default) builds the reCamera runtime: armd plus the files of the
[reCamera B601-DM kit](kits/recamera-b601dm/) (init script, gallery manifest, URDF; sources in
`kits/recamera-b601dm/packaging/`). `--flavor lite` builds the minimal runtime: the same
`armd` binary in a tarball, without init script or kit files (version default: `Cargo.toml`).

`build_deb.sh` needs `cargo` and `dpkg-deb`. It builds `RUSTFLAGS` from `RECAMERA_TOOLCHAIN`
(the older names `REC_TOOLCHAIN` / `RISCV_SYSROOT` also work); if you set `RUSTFLAGS`
yourself it is used as is. `.cargo/config.toml` only holds the target-wide linker settings.

Sizes measured on the build for this README (release profile: `opt-level = "z"`, fat LTO,
stripped): `armd` 4,327,440 bytes (4.1 MiB), `.deb` 2,090,568 bytes (2.0 MiB, gzip). Memory
use on the device has not been measured for this README.

---

## Web cockpit

`src/api/index.html` is a single HTML file without external resources, embedded in the binary
and served at `/`. JMuxer (`/assets/jmuxer.min.js`) plays the camera video. The UI is in
Chinese and English (language switch in the header).

- **Simple / expert mode**: simple shows the video with the joysticks on top and, on the
  7-DOF arm, always drives in the view frame; expert adds the view / base frame switch, the
  joint / Cartesian mode switch, a side-view pose preview, link status, the joint state table,
  dataset recording and keyboard shortcuts.
- **Joint jog**: two virtual joysticks (J1/J2, J3/J5, gripper) and the keyboard.
- **Cartesian control** in two frames:
  - *view frame*: directions follow the camera image (the camera is mounted after J6), solved
    with a 6-joint IK in the browser;
  - *base frame*: directions follow the arm's base axes.
  The target follows the measured pose and is streamed over the WebSocket lease.
- **Detection overlay**: YOLO boxes and ArUco markers drawn on the video. Choose YOLO, ArUco
  or both; the page holds a vision subscription only while the overlay is on and the page is
  visible. Single-shot snapshot.
- **Hand-eye calibration drawer**: eye-in-hand (camera on the gripper) or eye-to-hand (fixed
  camera) with an ArUco marker; capture poses, solve, export JSON that rebot-companion's
  `HandEyeCalibration.load_result` reads.
- **Tap-to-grasp**: with detection on, tap a box and confirm; armd runs the `grasp` action
  (open, approach looking down, closed-loop image centring, align J6 to 0° / 90°, descend,
  close and check the hold, lift). Any joystick, key, stop or E-stop ends it. The grasp setup
  (table height, gripper pixel, approach and grasp heights, camera tilt) is stored by armd.
  Tested only in simulation and screenshot tests so far; see
  [docs/plan_tap_to_grasp.md](docs/plan_tap_to_grasp.md) for what still needs checking on the arm.
- **Gravity compensation / hand-guiding**: hold the switch for 0.6 s to make J1–J6 float (MIT
  mode, kp = 0, damping, URDF gravity torque as feed-forward, ramped in over 1 s); the gripper
  stays position-controlled. While it is on, sticks, keys and presets send nothing, the gripper
  keys send open / close actions, and a banner shows the ramp, the recording toggle and an exit
  button. Exit, stop, E-stop, lease loss, faults, stale feedback, overspeed and the
  other exit causes lock the arm at the measured pose; the reason is shown as a toast.
  Overspeed limits are 1.5 rad/s for J1–J3 and 3.0 rad/s for J4–J6. On the B601-DM, all five
  exit paths tested (exit, stop, takeover, E-stop, client disconnect) held the arm within
  1.5 mrad, and a 30 s hand-guided recording gave 1509 frames.
- **E-stop** button, fault display and clear, torque enable by press-and-hold, park.

The cockpit does not take the lease when it opens. It acquires it on the first control input
(jog, stick, key, Cartesian input, preset, enable, gripper, grasp, gravity compensation, park
or home) and releases it after 30 s without input, only while torque is off and no action,
gravity compensation, stream or motion is running: armd parks a torqued arm whose lease is
released. An agent can therefore take the lease while a cockpit page is open. When another
client holds the lease, the cockpit switches to observer mode, names the holder from the
observation's `lease_holder` (another cockpit page, the AI agent `rebot-mcp`, or the client's
label) and offers a takeover with confirmation. E-stop works without the lease.

## Vision (reCamera kit)

`rebot-vision` ([kits/recamera-b601dm/vision](kits/recamera-b601dm/vision/)) is a C++ process
in the `rebot-teleop` package. It owns the camera and provides:

- **Video**: H.264 over `ws://<host>:8001/`, played by the cockpit.
- **Object detection**: YOLO on the SG2002 TPU, using the YOLO11n model already on the device.
- **ArUco markers**: dictionary `aruco4x4_50`, each with its 3D pose in the camera frame (marker
  size `marker_size`, 5 cm by default). Hand-eye calibration uses them. A marker can be much
  smaller than the object it is stuck on.
- **On demand**: nothing is inferred until a caller asks.
  - One frame: `POST /api/v1/detection/trigger {"pipelines": ["yolo"]}`, `["aruco"]` or
    both. The grasp action sends one YOLO trigger per centring step, and calibration one
    ArUco trigger per captured pose.
  - Continuous: a subscription (`/api/v1/vision/subscribe`) with its pipelines and rate,
    kept alive by the caller and dropped when keep-alives stop. Several subscriptions run as
    their union, at no more than 5 results/s by default. The cockpit overlay uses one.

Measured on the reCamera with the arm's torque off (rebot-teleop 1.3.1, two runs):

| | Runs on | Time per frame | Whole chip at 5 results/s |
|---|---|---|---|
| Standby (no caller) | — | — | 20–26 % (vision process 6 %) |
| YOLO | TPU | 47 ms | 34 % |
| ArUco (640 × 360) | CPU | 37–57 ms (scene-dependent) | 53 % |
| YOLO + ArUco | TPU + CPU | — | 55 % |

A single trigger from standby returns in about 0.12–0.25 s (p50 0.13–0.15 s), measured from a
warm client; the first request of a newly started client adds about 0.4 s of connection set-up. Settings, the model search path
and the per-stage ArUco timing are in the
[vision README](kits/recamera-b601dm/vision/README.md).

---

## API overview

Default port 8765. Observation over REST; actuator commands only over the
WebSocket that holds the lease.

| Path | Method | Purpose |
|---|---|---|
| `/api/v1/observation` | GET | positions, velocities, torques, temperatures, safety state, `commanded_positions`, latest `action`, driver capabilities, `driver_range_check` (Damiao mapping ranges read from the motors at startup) |
| `/api/v1/observation/schema` | GET | DOF, joint names, limits, workspace, camera mount, `capabilities` ([fields](docs/architecture.md#3-capability-discovery)) |
| `/api/v1/ws` | WS | lease (`acquire_lease`, `heartbeat`), `command` / `command_ack` / `command_reject`, telemetry |
| `/api/v1/safety/e_stop` | POST | E-stop, no lease needed |
| `/api/v1/torque`, `/api/v1/motion/*`, `/api/v1/action/*`, `/api/v1/mode/gravity_comp`, `/api/v1/safety/clear_fault` | POST | return `409 LEASE_REQUIRED`; send the same paths as WS `command`s |
| `/api/v1/grasp/setup` | GET | grasp setup (changed with a WS command) |
| `/api/v1/dataset/record/start` / `save` / `discard` | POST | episode recording |
| `/api/v1/detection/latest`, `/trigger`, `/inference`, `/ws` | GET / POST / WS | detector proxy |

WebSocket sequence:

```json
{"type":"acquire_lease","client_id":"agent_node"}
{"type":"heartbeat","client_id":"agent_node","generation":1,"epoch_id":42}
{"type":"command","request_id":1,"generation":1,"epoch_id":42,"path":"/api/v1/action/move_to","body":{"position":[0.3,0.0,0.2]}}
{"type":"command_ack","request_id":1,"status":"accepted","action_id":5,"expected_duration_s":2.4}
```

Reference documents:

- [docs/agent_action_api.md](docs/agent_action_api.md): `move_to`, `move_relative` (base,
  tool and camera frames), `gripper`, `grasp`, the gravity-compensation mode command, action
  status and error codes;
- [docs/control_safety_contract.md](docs/control_safety_contract.md): lease, epochs, driver
  capabilities, timing defaults, speed caps, workspace;
- [docs/detection_api.md](docs/detection_api.md): detection routes and an agent loop example.

---

## Agent and MCP integration

[rebot-companion](https://github.com/Seeed-Solution/rebot-companion) is the host-side Python
SDK and stdio MCP server for this daemon. It holds the lease over the WebSocket, calls the
action API, takes snapshots from the camera stream and uses the detection proxy. Point it at
armd with `--armd http://192.168.42.1:8765`. Its README describes the MCP tools and the lease
rules for sharing the arm with the cockpit.

---

## Episode recording (Parquet)

While a recording is active, each control tick appends one frame; the frames are kept in
memory and written to an uncompressed Parquet file on `save`.

| Column | Type | Content |
|---|---|---|
| `timestamp` | Float64 | seconds since armd started |
| `observation.state` | List<Float32> | measured joint positions (rad) |
| `action` | List<Float32> | commanded joint positions (rad) |

```bash
curl -X POST http://192.168.42.1:8765/api/v1/dataset/record/start \
     -H 'Content-Type: application/json' -d '{"output_path":"/userdata/datasets/episode_001.parquet"}'
curl -X POST http://192.168.42.1:8765/api/v1/dataset/record/save      # or .../discard
```

In gravity-compensation mode the commanded position of J1–J6 follows the measured position,
so a hand-guided episode has `action` equal to `observation.state` for those joints.

Without `output_path` the file goes to `./data/episodes/<episode_id>.parquet` relative to the
working directory. The column names follow LeRobot's naming, but armd writes one plain
Parquet file per episode, not a LeRobot dataset (no `meta/` files, episode or frame indices,
or camera frames). Converting it to a LeRobot dataset is not provided here and training on it
has not been tested.

---

## Configuration and CLI

```text
armd [OPTIONS]
    --follower <MODEL>       rebot-dm, rebot-rs, so-arm101, fashionstar, star-arm102, mock (default)
    --follower-port <PORT>   CAN interface (default can0) or serial device (default /dev/ttyUSB0)
    --config <JSON>          ArmConfig profile (must match the follower's joint count)
    --port <PORT>            HTTP / WebSocket port (default 8765, or $PORT)
    -h, --help
```

Environment: `RUST_LOG` (log filter, default `armd=info,tower_http=info`), `PORT`,
`ARMD_DETECTOR_URL` (default `http://127.0.0.1:8090`), `ARMD_GRASP_SETUP` (grasp setup file,
default `/userdata/rebot-teleop/grasp_setup.json`).

Without `--config`, `rebot-dm` uses the built-in B601-DM profile (`ArmConfig::rebot_dm()` in
`src/model.rs`) and the other followers a generic 6-joint profile. A `--config` file is a
complete `ArmConfig` JSON. The same B601-DM profile is shipped as data in
[`kits/recamera-b601dm/profile/rebot-b601dm.json`](kits/recamera-b601dm/profile/rebot-b601dm.json)
(a test keeps it equal to `rebot_dm()`); the deb installs it at
`/usr/share/rebot-teleop/profile/rebot-b601dm.json` and the init script passes it with
`--config` when the follower is `rebot-dm`. Excerpt with the rebot-dm values:

```json
{
  "profile_version": 1,
  "dof": 7,
  "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
  "control_frequency_hz": 50,
  "lease_timeout_ms": 2500,
  "max_temperature": 70.0,
  "limits": [
    {"name": "joint_1", "min_position": -3.14159, "max_position": 3.14159, "max_velocity": 2.0, "max_acceleration": 5.0}
  ],
  "home_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0],
  "planned_max_velocity": 0.45,
  "workspace": {"floor_z_m": 0.0, "box_min_m": [-0.6, -0.6, -1.0], "box_max_m": [0.6, 0.6, 0.8], "max_reach_m": 0.52}
}
```

A profile without `kinematics` and `workspace` runs without the Cartesian workspace check and
without Cartesian actions; armd logs a warning at startup. Field list:
[docs/control_safety_contract.md](docs/control_safety_contract.md#configuration--配置).

---

## Wiring (reCamera B4 CAN board)

The B4 CAN board pinout (J2: power, ground, `CAN_H`, `CAN_L`) is part of the reCamera kit
documentation: [kits/recamera-b601dm/docs/](kits/recamera-b601dm/docs/README.md#wiring-recamera-b4-can-board).

---

## Safety

armd moves a real arm with enough force to injure people and damage objects. Use it at your
own risk; the MIT License comes without warranty.

- Keep people, especially hands and faces, out of the arm's reach while it is powered. Clear
  the workspace before enabling torque.
- Keep a hardware power cut-off within reach. The software E-stop (cockpit button,
  `POST /api/v1/safety/e_stop`, WS `estop`) depends on the network and the daemon; it does not
  replace a physical emergency stop.
- Lease watchdog: if the lease holder stops sending heartbeats for 2.5 s while the arm is
  torqued, armd parks the arm and turns torque off when the arm is healthy and has a resting
  pose; otherwise it holds position with torque on and latches `Fault(WatchdogTimeout)`. The
  park budget is max(10 s, deceleration + planned park duration + 0.8 s settle + 3 s), plus 5 s
  once when the arm is at the resting pose and only the stationary check is pending
  (`docs/control_safety_contract.md`, "Unload settle").
- Torque is not cut away from the resting pose (`ARM_NOT_AT_RESTING_POSE`); use park.
- Workspace box: on rebot-dm, servo targets and planned motion samples must keep the end_link
  origin, the 8 gripper-envelope corners and the camera point above the floor (`floor_z_m`),
  inside the box and within `max_reach_m`; violations are rejected with
  `422 WORKSPACE_VIOLATION`. The box is a coarse limit, not collision detection with objects
  on the table.
- Speed caps: planned motion and actions are capped at 0.45 rad/s per joint on rebot-dm;
  actions default to 0.05 m/s (max 0.15 m/s) and 0.5 rad/s (max 1.0 rad/s).
- Tracking-error fault: a following error above 0.5 rad for 500 ms latches a fault that names
  the joint; actions end earlier as `TRACKING_STALL`.
- Gravity compensation: entry needs an armed, stationary arm, the
  lease and fresh feedback. Every exit holds the measured pose with torque on, then switches
  back to POS_VEL; the lease watchdog park starts only after that. Exceptions: a thermal trip
  still disables torque, and with a complete CAN bus loss the motors keep their last MIT
  command (gravity feed-forward and damping) until the bus returns. Details:
  [docs/control_safety_contract.md](docs/control_safety_contract.md#gravity-compensation-hand-guiding--重力补偿拖动示教).
- The default access point password (`12345678`) and the unauthenticated HTTP/WebSocket API
  let anyone on that network control the arm and start recordings. Change the password and
  run the arm on a network you trust.

The values above are software defaults from the source (`src/model.rs`, `src/supervisor.rs`),
not certified safety functions.

---

## Development

```bash
cargo test                                 # 316 tests (2 ignored), mock arm, no hardware
cargo clippy --all-targets -- -D warnings
node scripts/view_ik_selftest.mjs          # cockpit view-frame IK
node scripts/calib_selftest.mjs            # cockpit hand-eye calibration (part c needs ../rebot-companion and uv)
node tests/test_rpc_error_contract.js      # cockpit RPC error handling
```

CI (`.github/workflows/ci.yml`) runs these, plus `sh -n` on the kit init scripts. `cargo fmt --check` is not enforced: the code is
not rustfmt-formatted yet. The RISC-V cross build is not in CI because it needs the SG2002
toolchain libraries. `test_damiao_smoothness::stale_posvel_gripper_recovery_uses_posvel_frame`
is timing-sensitive and can fail on a loaded machine; rerun it.

Playwright scripts (`scripts/*_shots.mjs`, `scripts/ux_walkthrough.mjs`,
`tests/test_*_playwright.js`, `tests/test_single_origin_e2e.js`) take cockpit screenshots;
they need Playwright (`PLAYWRIGHT=<module path>` to override) and a Chromium (`CHROME=<path>`),
and some need a running armd. The `scripts/*_shots.mjs` write to `.cache/` (or `OUT`), the
`tests/*.js` scripts to `tests/artifacts/`; both are ignored by git. The Python scripts in
`scripts/` are bench diagnostics: some send raw CAN frames to the motors, others drive a running
armd over its API. Run them only with the arm supported and the workspace clear.

See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).

---

## Acknowledgements

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm): arm hardware and the URDF
  this daemon's kinematics and gravity model are based on.
- [HuggingFace LeRobot](https://github.com/huggingface/lerobot): column naming of the recorded
  episodes.
- [JMuxer](https://github.com/samirkumardas/jmuxer): H.264 playback in the cockpit.

---

## License

Code: MIT, see [LICENSE](LICENSE). Kit hardware: the camera mount CAD in
[`kits/recamera-b601dm/hardware/`](kits/recamera-b601dm/hardware/) is CERN-OHL-W-2.0
(its own [LICENSE](kits/recamera-b601dm/hardware/LICENSE)). Third-party files keep their own
licenses, listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md): JMuxer (MIT) and the
reBot-DevArm URDF (CERN-OHL-W-2.0).
