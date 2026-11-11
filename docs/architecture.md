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
  field is version 1; armd refuses newer versions at startup. The URDF fields (`urdf`,
  `urdf_sha256`, `chain`, `gripper_joint`) and the `Urdf` model value are optional additions
  to version 1; a profile without them loads as before. An older armd rejects a profile that
  sets `kinematics` or `gravity_model` to `Urdf` (unknown variant).
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
| `speed_cap` | `linear_mps`, `angular_rps`, `joint_velocity_scale` (0, 1]: upper bound for every planned motion, min() with the request and `planned_max_velocity`; each entry optional | no |
| `collision` | collision guard: `enabled` (default false), `thresholds_nm` (J1..J6, placeholders until measured on a real arm), `persist_ticks`, `clear_ratio`, `resume_after_s`, `max_auto_resumes`, `escalate_window_s`; each entry optional | no |
| `kinematics` | kinematic model: `RebotDevArm` (built-in B601-DM, needs `dof` 7) or `Urdf` (the `chain` of `urdf`); null disables Cartesian actions and the workspace check | no |
| `workspace` | Cartesian safety box (`floor_z_m`, `box_min_m`, `box_max_m`, `max_reach_m`, `check_points_end_m`, `keepout_boxes` `[{name, min_m, max_m}]`, at most 16) | no |
| `gravity_model` | gravity model: `RebotDevArm` or `Urdf` (inertials of `urdf`); both model fields must name the same source | no |
| `urdf` | URDF file for `Urdf`, relative to the profile file; read once at startup | with `Urdf` |
| `urdf_sha256` | expected sha256 of `urdf`; startup fails on a mismatch | no |
| `chain` | `base_link`, `tip_link`, `joints` `[{urdf_joint, sign (±1, default 1), offset_rad (default 0)}]`: the arm joints in armd order = revolute joints from base to tip; URDF angle = sign × armd angle + offset_rad | with `Urdf` |
| `gripper_joint` | gripper joint index (after the arm joints); default: the joint after the arm joints when `dof` = chain length + 1, else none | no |
| `gravity_comp` | gravity-compensation policy (ramp, scale, damping, torque and speed limits, hold gains); null = unsupported | no |
| `damiao_ranges` | per-joint Damiao mapping range `{p_max, v_max, t_max}`; null = ±12.5 rad, ±30 rad/s, ±18 N·m | no |
| `gripper_policy` | gripper action (closed / open angles, holding gap and torque, squeeze, settle) | no |
| `grasp` | tap-to-grasp setup (`GraspSetup` in `src/grasp.rs`: heights, grip pixel, camera tilt, intrinsics placeholders, centring and timeout parameters) | no |
| `camera_mount` | camera extrinsics relative to end_link: `flip`, `rot_deg`, `tilt_deg`, `offset_m` | no (all zero) |

Details and units: [control_safety_contract.md](control_safety_contract.md#configuration--配置).

Not in the profile today:

- The built-in URDF check: with `RebotDevArm`, the model is compiled into armd and armd
  checks the sha256 of `reBot-DevArm_gripper.urdf` at fixed paths (`assets/`,
  `/usr/share/rebot-teleop/assets/`, `/usr/share/rebot-teleop/`); the reCamera kit's deb
  installs it. That path is not a profile field; a profile that wants a URDF from its own
  path uses `Urdf` + `urdf`.

Arm model with `Urdf` (`src/urdf.rs`, `src/chain.rs`): the chain `base_link -> tip_link`
with 1..8 revolute joints; fixed joints are folded in; links that hang off the chain
(fingers) count for gravity at joint position 0. The chain gives FK, the geometric Jacobian,
damped-least-squares IK and gravity torques. `tests/test_chain.rs` builds the chain from the
B601-DM URDF and compares it with the built-in model over random joint vectors (FK 6e-16 m,
IK 2e-12 rad, gravity 1e-14 N·m). What each feature needs:

| Feature | Needs |
|---|---|
| workspace check, pose TCP, gravity torques | any chain length; `dof` = chain length (+1 with a gripper) |
| residual torques, collision guard | 6 arm joints |
| Cartesian actions, grasp, place, gravity compensation | 6 arm joints, gripper at joint 7 (`dof` 7) |

The tip frame is the frame of the workspace check points and Cartesian targets; with a chain
the camera mount (`cam_mount`) uses the second-to-last joint as the wrist axis. Profiles:
[`profiles/rebot_b601_rs.json`](../profiles/rebot_b601_rs.json) (B601-RS, not verified on
hardware), template [`kits/_template/profile/example_urdf.json`](../kits/_template/profile/example_urdf.json).
- The grasp setup can also come from `ARMD_GRASP_SETUP` (runtime changes, persisted).
- Runtime keep-out boxes (`ARMD_KEEPOUT`, default `/userdata/rebot-teleop/keepout.json`)
  are added to `workspace.keepout_boxes` (they cannot remove a profile box); named poses are stored in `ARMD_POSES`
  (default `/userdata/rebot-teleop/poses.json`); the runtime collision guard setting in
  `ARMD_COLLISION` (default `/userdata/rebot-teleop/collision.json`) replaces `collision.enabled`
  when present. The files are written by one writer thread in order (unique temp file, fsync,
  rename, directory fsync; an older state never overwrites a newer one); an invalid file is
  logged at start and renamed to `<name>.bad-<unix seconds>`.

The official kit ships the B601-DM profile as data,
`kits/recamera-b601dm/profile/rebot-b601dm.json`; `tests/test_arm_profile.rs` fails when it
differs from `rebot_dm()`.

### 2. Perception provider API

armd does not run inference. It proxies a detector's HTTP API under its own port so the
cockpit and agents use one origin (`src/api/proxy.rs`, [detection_api.md](detection_api.md)).

- Version: the provider reports `"api_version"` in every `:8090` body. **2** is the API of the
  kit's `rebot-vision` from `rebot-teleop` 1.3.0: vision on demand per caller (triggers name their
  pipelines, continuous inference through subscriptions that expire without keep-alive, the mode
  as a default subscription). **1** is `rebot-vision` 1.2 (one global mode, trigger in every mode,
  combined result). Providers without the field (yolo-detector, aruco-detector) are version 0:
  the routes below without `/api/mode` and the subscription routes. armd publishes the
  provider's version as `capabilities.vision.api_version`.
- Upstream: `ARMD_DETECTOR_URL` (default `http://127.0.0.1:8090`).

| armd route | Upstream | Purpose |
|---|---|---|
| `GET` / `POST /api/v1/detection/inference` | `/api/inference` | continuous inference on / off, rates |
| `GET /api/v1/detection/latest` | `/api/results/latest?after=&wait_ms=` | latest result, long poll |
| `POST /api/v1/detection/trigger` | `/api/trigger?wait_ms=` (JSON body passed through) | one inference on a fresh frame; version 2 runs the body's `pipelines`; no continuous state changes |
| `GET` / `POST /api/v1/vision/mode` | `/api/mode` | mode `off` / `yolo` / `aruco` / `both`, continuous on / off and the rate in one request (version 1); in version 2 the default subscription, and `fps` is the cap of every subscription |
| `POST /api/v1/vision/subscribe` | `/api/subscribe` | start or update a caller's continuous inference (version 2) |
| `POST /api/v1/vision/subscribe/{id}/keepalive`, `DELETE /api/v1/vision/subscribe/{id}` | `/api/subscribe/{id}[/keepalive]` | keep a subscription alive, end it (version 2) |
| `GET /api/v1/vision/subscriptions` | `/api/subscriptions` | active subscriptions and what runs (version 2) |
| `GET /api/v1/detection/ws`, `/results` | `ws://127.0.0.1:8001/results` | detection stream (fixed address) |

Status and body pass through; an unreachable upstream gives 502. Video is not proxied: the
cockpit plays the camera's H.264 WebSocket stream (`ws://<host>:8001/`) directly. On the
reCamera the provider is the kit's `rebot-vision` (the camera's `yolo-detector` or
`aruco-detector` services also work as version 0 providers). `rebot-vision`
(`kits/recamera-b601dm/vision/`, design in [dev_flow_spec_vision_runtime.md](dev_flow_spec_vision_runtime.md))
is part of the package since `rebot-teleop` 1.2.0: one process owns the camera and runs YOLO (TPU) and ArUco (CPU)
on the same frame. From 1.3.0 (version 2) it infers nothing without a caller: a trigger runs the
pipelines it names, and continuous inference runs the union of the active subscriptions'
pipelines at their highest rate, capped at the configured rate. Results always carry both
`detections` and `markers`.

Callers do not change the global mode any more (with one global mode, the caller that restored
last won). The grasp action triggers `{"pipelines": ["yolo"]}` per centring step; the cockpit's
overlay holds its own subscription while it is on and the page is visible; the hand-eye
calibration drawer holds an ArUco subscription while open and captures with one `["aruco"]`
trigger per pose. With a version 1 provider the grasp still switches to `yolo` (`both` when
ArUco was on) and restores the previous mode afterwards, and the cockpit uses the mode; providers
without `/api/mode` (404) get the old inference on / off toggle.

### 3. Capability discovery

`GET /api/v1/observation/schema` carries `capabilities` (`Capabilities` in `src/model.rs`), so
the cockpit and MCP clients can adapt without probing routes. Schemas from older armd versions
have no `capabilities` object; it parses as all false, `version` 0.

- Version: `capabilities.version`, currently **6** (`CAPABILITIES_VERSION`). Version 2 added
  the `vision.*` fields after `detection_proxy`; version 3 added `vision.per_request_pipelines`
  and `vision.subscriptions`; version 4 added `servo_guard` (with the schema fields
  `servo_max_velocity` and `servo_max_lead_rad`, and `observation.motion_limit`); version 5 added
  `diagnostics`; version 6 added `obs_rate`, `fault_codes`, `poses`, `keepout`, `speed_cap`,
  `residual_torques` and `tick_stats` (with the schema field `speed_cap` and the observation
  fields `fault` and `residual_torques`), `place`, `collision_guard` (with the observation
  field `collision`), and `safety_describe`. Clients check each field and fall back
  when it is false or missing.

| Field | True when |
|---|---|
| `profile_version` | (number) version of the loaded profile |
| `cartesian` | `kinematics` with 6 arm joints and the gripper at joint 7 (Cartesian actions) |
| `workspace_check` | `kinematics` and `workspace` set |
| `gripper` | `gripper_policy` set and a 7th joint exists (`/api/v1/action/gripper`) |
| `grasp` | 7-DOF, `kinematics`, `gripper_policy` and `grasp` set (`/api/v1/action/grasp`) |
| `gravity_comp` | profile, URDF check and driver support the mode; equals `observation.gravity_comp.supported` |
| `damiao_ranges` | the profile sets per-joint Damiao ranges (the startup check result is `observation.driver_range_check`) |
| `servo_guard` | always true: armd caps servo streams (speed, lead), ends a blocked stream in a hold and publishes `observation.motion_limit` (version 4) |
| `diagnostics` | always true: `GET /api/v1/diagnostics/boots` and `/blackbox` exist; their `enabled` says whether this armd writes the files (version 5) |
| `obs_rate` | always true: `/api/v1/ws?obs_hz=N` limits observations on that connection (version 6) |
| `fault_codes` | always true: `observation.fault` is `{code, message, recoverable}` in Fault / EStop, else null (version 6) |
| `poses` | always true: `GET /api/v1/poses`, WS `/api/v1/poses/save`, `/api/v1/poses/delete`, action `move_to_pose` (version 6) |
| `keepout` | `workspace_check` is true: keep-out boxes are applied and `/api/v1/workspace/keepout` accepts a runtime list (version 6) |
| `speed_cap` | always true: the profile `speed_cap` is applied and published as `schema.speed_cap` (null when not set) (version 6) |
| `residual_torques` | the profile has a gravity model on a 6/7-DOF arm; `observation.residual_torques` is still null without torque feedback (version 6) |
| `tick_stats` | always true: `GET /api/v1/diagnostics/tick` exists (version 6) |
| `place` | 7-DOF, `kinematics` and `gripper_policy` set (`/api/v1/action/place`; version 6) |
| `collision_guard` | the profile has a gravity model on a 6/7-DOF arm and the driver reports torque: the `collision` profile section, `/api/v1/safety/collision` and `observation.collision` apply (the guard itself is off unless enabled; version 6) |
| `safety_describe` | always true: `GET /api/v1/safety/describe` lists limits, stops and faults generated from the running config (version 6) |
| `vision.detection_proxy` | always true: the detection routes exist; reachability of the provider is not checked |
| `vision.mode_endpoint` | (string) always `/api/v1/vision/mode` |
| `vision.modes` | (list) modes the provider reports (`off`, `yolo`, `aruco`, `both`); empty when it is unreachable or has no `/api/mode` |
| `vision.mode` | (string or null) the provider's current base mode |
| `vision.api_version` | (number or null) the provider's perception API version |
| `vision.trigger`, `vision.yolo_available` | (bool or null) trigger in every mode; YOLO model loaded |
| `vision.per_request_pipelines` | the provider runs the pipelines a trigger names (version 2) |
| `vision.subscriptions` | the provider serves `/api/v1/vision/subscribe` (version 2) |

The `vision.modes` / `mode` / `api_version` / `trigger` / `yolo_available` /
`per_request_pipelines` / `subscriptions` values come from the
provider's `GET /api/mode`: the schema handler uses the body of the last mode request proxied
through armd, or asks the provider (500 ms timeout) when that is older than 2 s.

Other parts of the schema already describe the arm: `dof`, `joint_names`, `limits`,
`workspace`, `camera_mount`, `kinematics`, `servo_max_velocity` (effective servo speed cap per
joint), `servo_max_lead_rad`, `speed_cap`. Per-observation state stays in the observation
(`driver_capabilities`, `gravity_comp`, `driver_range_check`, `motion_limit`, `fault`,
`residual_torques`, `collision`).

### 4. Packaging

A kit ships: the armd binary (unchanged core), its profile, optional sidecar processes
(perception provider), and a launch script that starts them and runs
`armd --follower <model> [--follower-port <port>] --config <profile>`.

- Version: not versioned in code; the layout below is version 1 of this document.
- armd reads `PORT`, `RUST_LOG`, `ARMD_DETECTOR_URL`, `ARMD_GRASP_SETUP`, `ARMD_KEEPOUT`, `ARMD_POSES`, `ARMD_COLLISION`, `ARMD_LOG_DIR`, `ARMD_BLACKBOX_DIR` from the environment.

The official kit's package (`kits/recamera-b601dm/packaging/`, deb `rebot-teleop` 1.3.4):

| Installed path | Source in the repository |
|---|---|
| `/usr/bin/armd`, symlink `/usr/local/bin/armd` | core build |
| `/usr/bin/rebot-vision` | `kits/recamera-b601dm/vision/`, built by `scripts/build_vision.sh` |
| `/etc/init.d/K92rebot-teleop` (postinst adds `S92rebot-teleop` only without the reCamera console) | `packaging/rootfs/etc/init.d/K92rebot-teleop` |
| `/usr/share/rebot-teleop/rebot-vision-run` (respawn loop, nice 10), `vision.json.default` | `packaging/rootfs/usr/share/rebot-teleop/` |
| `/usr/share/doc/rebot-teleop/` (`LICENSE`, `LICENSE-APACHE`, `THIRD_PARTY_NOTICES.md`) | repository root, `kits/recamera-b601dm/vision/LICENSE-APACHE` |
| `/usr/share/rebot-teleop/index.html` | `src/api/index.html` |
| `/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf` | `packaging/rootfs/usr/share/rebot-teleop/assets/` |
| `/usr/share/rebot-teleop/profile/rebot-b601dm.json` | `kits/recamera-b601dm/profile/` |
| `/usr/share/rebot-teleop/rebot-teleop.json` (+ `.md`, `.zh.md`) | gallery manifest for the reCamera console, `"requires": ["can"]` |
| `DEBIAN/control`, `postinst`, `prerm` | `packaging/debian/` |

The init script passes `--config` for the `rebot-dm` and `mock` followers, only when the
profile file exists, and `--follower-port` for every follower except `mock`.

## Repository layout

```text
src/, tests/, Cargo.toml     core crate (armd)
scripts/build_deb.sh         --flavor lite | recamera
docs/                        core docs and contracts (this file)
kits/
  _template/                 template and README for third-party kits
  recamera-b601dm/           official reference kit
    profile/                 arm profile JSON
    vision/                  rebot-vision process (C++, built against sscma-example-sg200x)
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
