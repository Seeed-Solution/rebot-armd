# Architecture: core, kits and agents

English | [中文](architecture.zh-CN.md)

rebot-armd is split into three layers. The core runs on any supported controller; a kit adapts
it to one hardware combination; agents talk only to the core's API.

```text
 agent layer   rebot-companion (SDK, MCP server), browser cockpit, scripts
                     │ HTTP / WebSocket :8765 only
 core          armd: control loop, safety supervisor, drivers, API, detection proxy
                     │ contracts: arm profile, perception provider API,
                     │            capability discovery, packaging
 kits          kits/<name>/: profile, packaging, optional perception process,
               hardware, calibration, docs (one per controller + arm + camera + mount)
```

| Layer | What it is | Where |
|---|---|---|
| Core | `armd`, the minimal runtime. Hardware-agnostic within the drivers it ships (`--follower`: `rebot-dm`, `rebot-rs`, `so-arm101`, `fashionstar`, `star-arm102`, `mock`) | repository root: `src/`, `tests/`, `Cargo.toml` |
| Kits | One adaptation: controller + arm + camera + mount. Data and packaging; no changes to core code | `kits/recamera-b601dm/` (official), third-party kits in their own repositories |
| Agent layer | [rebot-companion](https://github.com/Seeed-Solution/rebot-companion) SDK and MCP server, the web cockpit, scripts | uses `/api/v1/*` and `/api/v1/ws` of armd only |

The core crate stays at the repository root (it was published there); kits are added beside it.

## Builds

| Build | Contents | Command | Output |
|---|---|---|---|
| Minimal runtime | core only | `./scripts/build_deb.sh --flavor lite` | `dist/armd-lite_<version>_riscv64.tar.gz`: `armd`, `LICENSE`, `THIRD_PARTY_NOTICES.md` |
| reCamera runtime | core + `kits/recamera-b601dm` | `./scripts/build_deb.sh` (`--flavor recamera`, default) | `dist/rebot-teleop_<version>_riscv64.deb` |

Both use the same `armd` binary (riscv64gc-unknown-linux-musl, static). The cockpit
(`src/api/index.html`) and JMuxer are embedded in the binary, so the lite build serves the
cockpit too. Host builds (`cargo build --release`) produce the core for x86_64 or other
targets; they are not packaged by the script.

## Contracts

Core and kits interact only through these four contracts. Versions are those in the code at
the time of writing.

### 1. Arm profile

The `ArmConfig` JSON loaded with `armd --config <file>` (`src/model.rs`). Without `--config`,
`rebot-dm` uses the built-in `ArmConfig::rebot_dm()` and the other followers a generic
6-joint profile.

- Version: `profile_version`, currently **1** (`ARM_PROFILE_VERSION`). A profile without the
  field is version 1; armd refuses newer versions at startup.
- Unknown top-level fields are ignored (the `grasp` section rejects unknown fields); every optional
  field has a serde default.

| Field | Meaning | Required |
|---|---|---|
| `profile_version` | contract version | no (default 1) |
| `dof`, `joint_names`, `limits` (`name`, `min_position`, `max_position`, `max_velocity`, `max_acceleration`), `home_positions` | joints and limits; `dof` must match the follower (`rebot-dm` 7, others 6, `mock` any) | yes |
| `lease_timeout_ms`, `max_temperature`, `control_frequency_hz` | lease watchdog, temperature trip, control rate (1..1000 Hz) | yes |
| `resting_pose` | pose and thresholds for safe torque release; null forbids automatic release | no |
| `following_error` | `max_error_rad`, `persistence_ms` | no (0.5 rad, 500 ms) |
| `max_jerk`, `planned_max_velocity` | trajectory jerk preference, planned-motion speed cap | no |
| `kinematics` | kinematic model (`RebotDevArm`); null disables Cartesian actions and the workspace check | no |
| `workspace` | Cartesian safety box (`floor_z_m`, `box_min_m`, `box_max_m`, `max_reach_m`, `check_points_end_m`) | no |
| `gravity_model` | gravity model (`RebotDevArm`) | no |
| `gravity_comp` | gravity-compensation policy (ramp, scale, damping, torque and speed limits, hold gains); null = unsupported | no |
| `damiao_ranges` | per-joint Damiao mapping range `{p_max, v_max, t_max}`; null = ±12.5 rad, ±30 rad/s, ±18 N·m | no |
| `gripper_policy` | gripper action (closed / open angles, holding gap and torque, squeeze, settle) | no |
| `grasp` | tap-to-grasp setup (`GraspSetup` in `src/grasp.rs`: heights, grip pixel, camera tilt, intrinsics placeholders, centring and timeout parameters) | no |
| `camera_mount` | camera extrinsics relative to end_link: `flip`, `rot_deg`, `tilt_deg`, `offset_m` | no (all zero) |

Details and units: [control_safety_contract.md](control_safety_contract.md#configuration--配置).

Not in the profile today:

- URDF: `gravity_model` selects a model compiled into armd. At startup armd checks the sha256
  of `reBot-DevArm_gripper.urdf` at fixed paths (`assets/`, `/usr/share/rebot-teleop/assets/`,
  `/usr/share/rebot-teleop/`); the reCamera kit's deb installs it. The path is not a profile
  field.
- The grasp setup can also come from `ARMD_GRASP_SETUP` (runtime changes, persisted).

The official kit ships the B601-DM profile as data,
`kits/recamera-b601dm/profile/rebot-b601dm.json`; `tests/test_arm_profile.rs` fails when it
differs from `rebot_dm()`.

### 2. Perception provider API

armd does not run inference. It proxies a detector's HTTP API under its own port so the
cockpit and agents use one origin (`src/api/proxy.rs`, [detection_api.md](detection_api.md)).

- Version: not versioned yet (v0 = the routes below, as of this document).
- Upstream: `ARMD_DETECTOR_URL` (default `http://127.0.0.1:8090`).

| armd route | Upstream | Purpose |
|---|---|---|
| `GET` / `POST /api/v1/detection/inference` | `/api/inference` | continuous inference on / off, rates |
| `GET /api/v1/detection/latest` | `/api/results/latest?after=&wait_ms=` | latest result, long poll |
| `POST /api/v1/detection/trigger` | `/api/trigger?wait_ms=` | one inference on a fresh frame, `enabled` unchanged |
| `GET /api/v1/detection/ws`, `/results` | `ws://127.0.0.1:8001/results` | detection stream (fixed address) |

Status and body pass through; an unreachable upstream gives 502. Video is not proxied: the
cockpit plays the camera's H.264 WebSocket stream (`ws://<host>:8001/`) directly. On the
reCamera the provider is today the camera's `yolo-detector` (or `aruco-detector`) service,
not part of the kit; phase 2 replaces it with the kit's `rebot-vision`
([dev_flow_spec_vision_runtime.md](dev_flow_spec_vision_runtime.md)), which adds mode
switching (`off | yolo | aruco | both`) and on-demand trigger in every mode.

### 3. Capability discovery

`GET /api/v1/observation/schema` carries `capabilities` (`Capabilities` in `src/model.rs`), so
the cockpit and MCP clients can adapt without probing routes. Schemas from older armd versions
have no `capabilities` object; it parses as all false, `version` 0.

- Version: `capabilities.version`, currently **1** (`CAPABILITIES_VERSION`).

| Field | True when |
|---|---|
| `profile_version` | (number) version of the loaded profile |
| `cartesian` | 7-DOF profile with `kinematics` (Cartesian actions) |
| `workspace_check` | `kinematics` and `workspace` set |
| `gripper` | `gripper_policy` set and a 7th joint exists (`/api/v1/action/gripper`) |
| `grasp` | 7-DOF, `kinematics`, `gripper_policy` and `grasp` set (`/api/v1/action/grasp`) |
| `gravity_comp` | profile, URDF check and driver support the mode; equals `observation.gravity_comp.supported` |
| `damiao_ranges` | the profile sets per-joint Damiao ranges (the startup check result is `observation.driver_range_check`) |
| `vision.detection_proxy` | always true: the detection routes exist; reachability of the provider is not checked |

Other parts of the schema already describe the arm: `dof`, `joint_names`, `limits`,
`workspace`, `camera_mount`, `kinematics`. Per-observation state stays in the observation
(`driver_capabilities`, `gravity_comp`, `driver_range_check`). Phase 2 adds vision modes to
`capabilities.vision` (version 2).

### 4. Packaging

A kit ships: the armd binary (unchanged core), its profile, optional sidecar processes
(perception provider), and a launch script that starts them and runs
`armd --follower <model> [--follower-port <port>] --config <profile>`.

- Version: not versioned in code; the layout below is version 1 of this document.
- armd reads `PORT`, `RUST_LOG`, `ARMD_DETECTOR_URL`, `ARMD_GRASP_SETUP` from the environment.

The official kit's package (`kits/recamera-b601dm/packaging/`, deb `rebot-teleop` 1.1.0):

| Installed path | Source in the repository |
|---|---|
| `/usr/bin/armd`, symlink `/usr/local/bin/armd` | core build |
| `/etc/init.d/K92rebot-teleop`, symlink `S92rebot-teleop` | `packaging/rootfs/etc/init.d/K92rebot-teleop` |
| `/usr/share/rebot-teleop/index.html` | `src/api/index.html` |
| `/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf` | `packaging/rootfs/usr/share/rebot-teleop/assets/` |
| `/usr/share/rebot-teleop/profile/rebot-b601dm.json` | `kits/recamera-b601dm/profile/` |
| `/usr/share/rebot-teleop/rebot-teleop.json` (+ `.md`, `.zh.md`) | gallery manifest for the reCamera console, `"requires": ["can"]` |
| `DEBIAN/control`, `postinst`, `prerm` | `packaging/debian/` |

The init script passes `--config` only for the `rebot-dm` follower and
only when the profile file exists.

## Repository layout

```text
src/, tests/, Cargo.toml     core crate (armd)
scripts/build_deb.sh         --flavor lite | recamera
docs/                        core docs and contracts (this file)
kits/
  _template/                 template and README for third-party kits
  recamera-b601dm/           official reference kit
    profile/                 arm profile JSON
    vision/                  phase 2: rebot-vision process (placeholder README now)
    packaging/               rebot-teleop deb sources
    hardware/                camera mount CAD, CERN-OHL-W-2.0 (own LICENSE)
    calibration/             camera tilt per mount, placeholder intrinsics, grasp setup defaults
    docs/                    wiring, assembly, verified capabilities
```

## Making a third-party kit

1. Start from [`kits/_template/`](../kits/_template/README.md) in your own repository.
2. Write a profile for your arm (`dof` must match a follower armd ships). Check it with the
   mock follower and read `capabilities` from the schema.
3. Optionally provide a perception process that serves the routes in contract 2 and point
   `ARMD_DETECTOR_URL` at it.
4. Package the armd release binary (lite tarball, or a core build for your target), the
   profile, the sidecars and a launch script.
5. State in your README the armd version, `profile_version`, `capabilities.version` and the
   perception routes you rely on, and what you verified on hardware.

A new motor protocol or kinematic model is a change to the core (a driver or model in
`src/`), not a kit.

## Official vs third party

| | Official | Third party |
|---|---|---|
| Where | this repository, `kits/` | the author's repository |
| Today | `kits/recamera-b601dm` (reCamera + B601-DM) | none listed |
| Built by | `scripts/build_deb.sh` | the kit's own scripts, from armd release artifacts |
| Tested by | this repository's CI (`cargo test`, profile and init-script tests) | the kit's authors |
| Licence | code MIT; kit hardware CERN-OHL-W-2.0 | the author's choice |
