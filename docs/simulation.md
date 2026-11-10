# Simulation

English | [中文](simulation.zh-CN.md)

`armd --follower sim` runs the same armd binary against a MuJoCo physics simulation of the reBot B601-DM instead of the CAN bus. The SDK, MCP server and cockpit connect to armd as usual; armd's own checks (joint limits, workspace, lease watchdog, gripper stall / holding, residual torques, bus-loss detection, and the collision guard on branches that have it) run on simulated positions, velocities and torques.

## Run

Requirements: Rust toolchain, [uv](https://docs.astral.sh/uv/). The simulator is a uv project in `sim/mujoco` (dependencies `mujoco`, `numpy`).

```bash
# terminal 1: simulator (500 Hz physics, UDP 47600)
cd sim/mujoco
uv run rebot-sim --port 47600 --headless

# terminal 2: armd with the B601-DM profile
cargo run --bin armd -- --follower sim --follower-port udp://127.0.0.1:47600 \
  --config kits/recamera-b601dm/profile/rebot-b601dm.json --log-dir off --blackbox-dir off
```

Open the cockpit at `http://127.0.0.1:8765/`. `--follower-port` defaults to `udp://127.0.0.1:47600`; without `--config` the built-in `rebot-dm` profile is used. Set `ARMD_POSES`, `ARMD_KEEPOUT` and `ARMD_GRASP_SETUP` to writable paths when `/userdata` does not exist.

`rebot-sim` options:

| option | effect |
|---|---|
| `--port`, `--host` | UDP address (default `127.0.0.1:47600`) |
| `--headless` | no viewer (the default) |
| `--viewer` | MuJoCo viewer; needs a display (`DISPLAY` / `WAYLAND_DISPLAY`) |
| `--stats S` | print step rate, joint positions and cube position every S seconds |
| `--duration S` | exit after S seconds |
| `--benchmark N` | time N physics steps without real-time pacing and exit |

`REBOT_DEVARM_DIR=<reBot-DevArm checkout>` adds the upstream link meshes (`Rebot_Arm_description/DM/meshes/collision`) as visuals for the viewer. Physics never uses them.

## Tests

```bash
cargo test --lib sim_driver            # driver against a fake UDP peer
cargo test --test test_sim_fk          # armd FK vs MuJoCo end_link poses
cd sim/mujoco && uv run pytest -s      # simulator + armd end to end, FK fixture current
```

The end-to-end test builds armd (`cargo build`) unless `ARMD_BIN` names a binary. It enables the arm, homes it, opens the gripper, runs `move_to` with the tool pointing down, checks J2 / J3 gravity torques, descends onto the 4 cm cube, closes with `grasp` (expects `holding: true`), lifts, kills the simulator and expects fault code `BUS_LOSS`.

## What is simulated

- **Scene** (`sim/mujoco/models/rebot_b601dm_scene.xml`): arm chain and link inertials from `assets/reBot-DevArm_gripper.urdf` (the numbers armd's gravity model and kinematics use), box collision primitives sized from the upstream DM meshes, table top at z = 0 (base plane, profile `grasp.table_z_m`), a 4 cm 30 g cube at (0.30, 0, 0.02) m. Arm links collide with the table and cube, not with each other.
- **Joint convention**: J1..J6 motor angle = URDF angle. J7 drives both jaws through the rack: opening per side = −q7 × 0.0425 / 5.7 m (q7 = 0 closed, −5.7 rad = 85 mm). The second jaw follows by an equality constraint. J2 and J3 have upper stops at 0 rad (URDF limits), so the arm rests folded at q = 0; the other ranges are the profile limits + 0.1 rad.
- **Motors** (physics 500 Hz):
  - POS_VEL (armd's normal mode): v_des = clip(25 · (q_target − q), ±vlim), torque = kv · (v_des − dq) + model bias torque (gravity, Coriolis) on J1..J6; vlim 2.0 rad/s as sent by armd.
  - MIT (per-joint switch used by gravity compensation): torque = kp · (q_target − q) + kd · (dq_target − dq) + τ_ff.
  - Disabled: no torque; the arm falls under gravity.
  - Torque limits 27 N·m (J1..J3), 7 N·m (J4..J6), 1.5 N·m (J7).
- **Feedback** (250 Hz): positions, velocities, total motor torque (servo + feed-forward, so gravity and payload load show up), temperatures constant 35 °C, enable state.

### Wire protocol (UDP, JSON, version 1)

One JSON object per datagram.

| direction | message |
|---|---|
| armd → sim | `{"v":1,"type":"cmd","seq":N,"enable":bool,"targets":[T × 7] \| null}` per control tick and on enable / disable (`targets: null`) |
| armd → sim | `{"v":1,"type":"poll","seq":0,...}` keep-alive every 20 ms without other traffic |
| sim → armd | `{"v":1,"type":"state","session":S,"seq":N,"ack":N,"enabled":bool,"q":[7],"dq":[7],"tau":[7],"temp":[7],"t":s}` to the sender of the last datagram |

`T` is `{"mode":"pos_vel","q":rad,"vlim":rad/s}` or `{"mode":"mit","q":rad,"dq":rad/s,"kp":N·m/rad,"kd":N·m·s/rad,"tau":N·m}`. `ack` is the `seq` of the last applied command; `session` is random per simulator start.

armd side (`src/sim_driver.rs`): a background thread receives state into a cache; driver calls inside the control tick do not wait on the network. State with a wrong version, wrong array length, non-finite values or a `seq` not above the last one of the same session is dropped. Feedback older than 200 ms makes `is_feedback_valid` false and the enable state Unknown; the supervisor latches `BUS_LOSS` after 500 ms without fresh feedback (`BUS_LOSS_TIMEOUT`), as on hardware. Enable waits up to 80 ms for a state that acknowledges the enable command (disable: 25 ms), like the Damiao handshake.

## Measured on this machine

Host spark-6124 (aarch64, Cortex-X925, 20 cores), MuJoCo 4.0.0, Python 3.13:

| measurement | value |
|---|---|
| unpaced physics (`rebot-sim --benchmark 20000`) | 36,960 and 37,045 steps/s (2 runs) |
| paced simulator (`--stats 2`) | 500.0–500.5 Hz |
| FK agreement armd vs MuJoCo (`test_sim_fk`, 7 joint vectors) | worst 0.0001 mm, 0.0000° |
| `move_to` [0.30, 0, 0.12] tool down, TCP error after settle | 0.04 mm |
| hold torques at that pose (J1..J7, N·m) | 0.01, 7.05, −5.90, −0.23, 0.01, 0.00, 0.00 |
| close on the cube | `holding: true`, gap 2.64 rad, τ7 1.00 N·m |
| time from simulator kill to `BUS_LOSS` | 0.51 s |

## Not simulated / limitations

- Motor dynamics are simplified: no current loop, no cogging, no CAN latency or frame loss model; the POS_VEL integrator is replaced by bias-torque feed-forward. Loop gains, joint damping (0.8 N·m·s/rad), friction loss (0.05 N·m) and rotor armature are guessed, not identified on the motors.
- Contact parameters and friction (jaws 1.5, cube 1.2) are guessed; collision shapes are boxes, not meshes. The links do not collide with each other.
- J7 torque scale: holding and stall thresholds (`gripper_policy.holding_torque_nm`, the 0.444 N·m stall latch) are met by construction of the 1.5 N·m limit and gains, not by a measured gripper model.
- No camera image yet: vision, `grasp` (point-and-grasp) and detection need the real camera or a separate image source.
- No motor faults (over-temperature, under-voltage); temperature is constant. Damiao register access (mapping-range check, CTRL_MODE writes) is not simulated: `driver_range_check` is null and MIT switching is confirmed locally.
- With the default `speed` 1.0 rad/s the jaw reached −5.699 rad from closed after about 5.9 s, too late for the 200 ms settle within the 6 s default `timeout_s`: the gripper action ended `GRIPPER_TIMEOUT`. The test opens at 2.0 rad/s. Not verified on the real gripper.
- The simulator runs on the host only; it is not packaged for the reCamera.
