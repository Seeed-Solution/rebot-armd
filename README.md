# rebot-armd

<div align="center">

### Arm control daemon and web cockpit for the reBot B601-DM arm on a Seeed reCamera

**A Rust daemon (`armd`) that drives the arm over CAN, enforces a lease-based safety model, serves a browser cockpit and records episodes to Parquet**

[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg?style=flat-square&logo=rust)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)
[![CI](https://img.shields.io/github/actions/workflow/status/Seeed-Solution/rebot-armd/ci.yml?branch=main&style=flat-square&label=CI)](https://github.com/Seeed-Solution/rebot-armd/actions)
[![Target Arm](https://img.shields.io/badge/Robot-reBot_B601--DM-orange.svg?style=flat-square)](https://github.com/Seeed-Projects/reBot-DevArm)
[![Platform](https://img.shields.io/badge/Platform-Seeed_reCamera_SG2002-red.svg?style=flat-square)](https://www.seeedstudio.com)

English | [中文](./README.zh-CN.md)

<!-- TODO: Add demo GIF showing Web cockpit teleoperation here -->

</div>

---

`armd` runs on the reCamera (SG2002, RISC-V) next to the camera's own services. It talks to
the arm's Damiao motors on the CAN bus of the reCamera B4 board, runs a fixed-rate control
loop with a safety supervisor, and exposes the arm over HTTP and WebSocket on port 8765:

- a single-file web cockpit for phones, tablets and desktops (joysticks, Cartesian control,
  camera video, detection overlay, hand-eye calibration, tap-to-grasp);
- a REST API for observation and a WebSocket API for control, where actuator commands are
  accepted only from the connection that holds the control lease;
- high-level actions for agents (`move_to`, `move_relative`, `gripper`, `grasp`), used by
  [rebot-companion](https://github.com/Seeed-Solution/rebot-companion) and its MCP server;
- recording of joint state and commands to Parquet files.

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

Tested:

| Item | Details |
|---|---|
| Arm | reBot B601-DM: 6 arm joints and a gripper, all Damiao motors, 7 joints in armd (index 6 = gripper) |
| Controller | Seeed reCamera (SG2002) with the B4 CAN board, `can0` at 1 Mbit/s (the init script configures it) |
| Motor mode | Damiao `POS_VEL` (position and velocity limit), the default and only mode selectable from the CLI |

Present in the source but **not tested on hardware**:

| `--follower` | Driver | Bus | Notes |
|---|---|---|---|
| `rebot-rs` | RobStride (MIT mode, 29-bit CAN IDs) | SocketCAN | 6 joints; no enable confirmation, so automatic disable and fault clearing are refused |
| `so-arm101` | Feetech STS (SO-ARM101) | serial, 1,000,000 baud | 6 joints; position, velocity and temperature feedback, no torque or enable feedback |
| `fashionstar`, `star-arm102` | FashionStar | serial, 115,200 baud | 6 joints; the driver declares no feedback capabilities |
| `mock` | in-process simulation | — | default; used by the tests and for UI work without an arm |

The protocol encoders of these drivers are covered by unit tests (`tests/test_protocol_vectors.rs`,
`tests/test_driver_capabilities.rs`), not by runs on a real arm. They use the generic 6-joint
profile without kinematics or a workspace, so Cartesian actions return `UNSUPPORTED`. The
Damiao driver also has an MIT mode with gravity feed-forward, but it can only be enabled from
code (`DamiaoArmDriver::with_control_mode`), not from the CLI or a profile. Leader-follower
teleoperation is not implemented; `--leader` / `--leader-port` are rejected.

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

---

## Install on reCamera

Build the package (see [Cross-compile](#cross-compile-for-recamera)) or take a release `.deb`,
copy it to the camera and install it. `192.168.42.1` is the reCamera's address over USB;
when connected to the access point that this package starts, the camera is `192.168.16.1`.

```bash
scp dist/rebot-teleop_1.0.0_riscv64.deb root@192.168.42.1:/tmp/
ssh root@192.168.42.1 "dpkg -i /tmp/rebot-teleop_1.0.0_riscv64.deb && /etc/init.d/K92rebot-teleop restart"
```

The package installs:

| Path | Content |
|---|---|
| `/usr/bin/armd` (symlink `/usr/local/bin/armd`) | the daemon |
| `/etc/init.d/K92rebot-teleop` (symlink `S92rebot-teleop`) | init script: Wi-Fi, `can0` setup, start / stop |
| `/usr/share/rebot-teleop/` | cockpit `index.html`, URDF, app manifest `rebot-teleop.json` for the reCamera app list |
| `/userdata/local/apps/rebot-teleop.json` | copied by `postinst` |

The init script starts `armd --follower rebot-dm --follower-port can0 --port 8765`, sets
`can0` to 1 Mbit/s and starts a Wi-Fi access point on `wlan1` (SSID `reCamera-Arm-Teleop`,
default password `12345678`, address `192.168.16.1`). It can also join an existing Wi-Fi
network. These values are read from `/userdata/local/apps/rebot-teleop.config.json`
(`arm.follower_model`, `arm.serial_port`, `port`, `network.ap_ssid`, `network.ap_password`,
`network.ap_ip`, `network.sta_ssid`, `network.sta_password`). **Change the default access
point password** before using the arm outside a lab. Logs go to `/var/log/rebot-teleop.log`.

Open the cockpit at `http://<camera-address>:8765/`.

Object detection in the cockpit needs the camera's detector service (`yolo-detector`,
`http://127.0.0.1:8090` by default, `ARMD_DETECTOR_URL` to change it); it is not part of this
package.

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

./scripts/build_deb.sh 1.0.0          # version argument optional (default: debian/control)
# -> target/riscv64gc-unknown-linux-musl/release/armd
# -> dist/rebot-teleop_1.0.0_riscv64.deb
```

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

- **Simple / expert mode**: simple shows the video with the joysticks and the view / base
  frame switch on top; expert adds the joint / Cartesian mode switch, a side-view pose preview,
  link status, the joint state table, dataset recording and keyboard shortcuts.
- **Joint jog**: two virtual joysticks (J1/J2, J3/J5, gripper) and the keyboard.
- **Cartesian control** in two frames:
  - *view frame*: directions follow the camera image (the camera is mounted after J6), solved
    with a 6-joint IK in the browser;
  - *base frame*: directions follow the arm's base axes.
  The target follows the measured pose and is streamed over the WebSocket lease.
- **Detection overlay**: boxes from the camera's detector drawn on the video; detection on /
  off and single-shot snapshot.
- **Hand-eye calibration drawer**: eye-in-hand (camera on the gripper) or eye-to-hand (fixed
  camera) with an ArUco marker; capture poses, solve, export JSON that rebot-companion's
  `HandEyeCalibration.load_result` reads.
- **Tap-to-grasp**: with detection on, tap a box and confirm; armd runs the `grasp` action
  (open, approach looking down, closed-loop image centring, align J6 to 0° / 90°, descend,
  close and check the hold, lift). Any joystick, key, stop or E-stop ends it. The grasp setup
  (table height, gripper pixel, approach and grasp heights, camera tilt) is stored by armd.
  Tested only in simulation and screenshot tests so far; see
  [docs/plan_tap_to_grasp.md](docs/plan_tap_to_grasp.md) for what still needs checking on the arm.
- **E-stop** button, fault display and clear, torque enable by press-and-hold, park.

When another client holds the lease, the cockpit switches to observer mode and offers a
takeover with confirmation.

---

## API overview

Default port 8765. Observation over REST; actuator commands only over the
WebSocket that holds the lease.

| Path | Method | Purpose |
|---|---|---|
| `/api/v1/observation` | GET | positions, velocities, torques, temperatures, safety state, `commanded_positions`, latest `action`, driver capabilities |
| `/api/v1/observation/schema` | GET | DOF, joint names, limits, workspace, camera mount |
| `/api/v1/ws` | WS | lease (`acquire_lease`, `heartbeat`), `command` / `command_ack` / `command_reject`, telemetry |
| `/api/v1/safety/e_stop` | POST | E-stop, no lease needed |
| `/api/v1/torque`, `/api/v1/motion/*`, `/api/v1/action/*`, `/api/v1/safety/clear_fault` | POST | return `409 LEASE_REQUIRED`; send the same paths as WS `command`s |
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
  tool and camera frames), `gripper`, `grasp`, action status and error codes;
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
complete `ArmConfig` JSON; excerpt with the rebot-dm values:

```json
{
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

From the B4 board documentation; this repository does not verify it. Check the silkscreen of
your board revision.

```text
              reCamera B4 CAN board
     [ RST/BOOT buttons ]             [ silkscreen CAN&POWER ]
    ┌─────────────────────────────────────────────────┐
    │              J2 (white 6-pin connector)          │
    │         ┌───┬───┬───┬───┬───────┬───────┐       │
    │         │ 1 │ 2 │ 3 │ 4 │   5   │   6   │       │
    │         └───┴───┴───┴───┴───┬───┴───┬───┘       │
    └─────────────────────────────┼───────┼───────────┘
                                CAN_H   CAN_L
```

J2 (SM06B-SRSS-TB, JST SH 1.0 mm, 6 pins):

- Pin 1 / 2: `VIN` (12-24 V DC in)
- Pin 3 / 4: `GND`
- Pin 5: `CAN_H` (arm red wire)
- Pin 6: `CAN_L` (arm black wire)

The board has an MCP2518FD SPI-CAN controller, an MCP2558FD transceiver and a 124 Ω
termination resistor. Keep motor power wiring away from pins 5/6 and connect the grounds of
the reCamera and the arm power supply (pins 3/4).

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
  park deadline is max(10 s, planned park duration + 3 s).
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
- The default access point password (`12345678`) and the unauthenticated HTTP/WebSocket API
  let anyone on that network control the arm and start recordings. Change the password and
  run the arm on a network you trust.

The values above are software defaults from the source (`src/model.rs`, `src/supervisor.rs`),
not certified safety functions.

---

## Development

```bash
cargo test                                 # 248 tests (1 ignored), mock arm, no hardware
cargo clippy --all-targets -- -D warnings
node scripts/view_ik_selftest.mjs          # cockpit view-frame IK
node scripts/calib_selftest.mjs            # cockpit hand-eye calibration (part c needs ../rebot-companion and uv)
node tests/test_rpc_error_contract.js      # cockpit RPC error handling
```

CI (`.github/workflows/ci.yml`) runs these. `cargo fmt --check` is not enforced: the code is
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

MIT, see [LICENSE](LICENSE). Third-party files keep their own licenses, listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md): JMuxer (MIT) and the reBot-DevArm URDF
(CERN-OHL-W-2.0).
