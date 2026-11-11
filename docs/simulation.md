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
| `--damping D1,..,D6` | J1..J6 joint damping in N·m·s/rad instead of the scene values |
| `--frictionloss F1,..,F6` | J1..J6 Coulomb friction in N·m instead of the scene values |
| `--torque-report instant\|mean` | torque in state datagrams: instantaneous (default) or the mean over the last command period (sliding window; see Friction identification) |

`REBOT_DEVARM_DIR=<reBot-DevArm checkout>` adds the upstream link meshes (`Rebot_Arm_description/DM/meshes/collision`) as visuals for the viewer. Physics never uses them.

## Tests

```bash
cargo test --lib sim_driver            # driver against a fake UDP peer
cargo test --test test_sim_fk          # armd FK vs MuJoCo end_link poses
cd sim/mujoco && uv run pytest -s      # simulator + armd end to end, FK fixture current, sysid recovery
cd sim/mujoco && uv run pytest -s -m "not slow"   # without the 3.5 min sysid recovery test
```

The end-to-end test builds armd (`cargo build`) unless `ARMD_BIN` names a binary. It enables the arm, homes it, opens the gripper, runs `move_to` with the tool pointing down, checks J2 / J3 gravity torques, descends onto the 4 cm cube, closes with `grasp` (expects `holding: true`), lifts, kills the simulator and expects fault code `BUS_LOSS`.

## What is simulated

- **Scene** (`sim/mujoco/models/rebot_b601dm_scene.xml`): arm chain and link inertials from `assets/reBot-DevArm_gripper.urdf` (the numbers armd's gravity model and kinematics use), box collision primitives sized from the upstream DM meshes, table top at z = 0 (base plane, profile `grasp.table_z_m`), a 4 cm 30 g cube at (0.30, 0, 0.02) m. Arm links collide with the table and cube, not with each other.
- **Joint convention**: J1..J6 motor angle = URDF angle. J7 drives both jaws through the rack: opening per side = −q7 × 0.0425 / 5.7 m (q7 = 0 closed, −5.7 rad = 85 mm). The second jaw follows by an equality constraint. J2 and J3 have upper stops at 0 rad (URDF limits), so the arm rests folded at q = 0; the other ranges are the profile limits + 0.1 rad.
- **Motors** (physics 500 Hz):
  - POS_VEL (armd's normal mode): v_des = clip(25 · (q_target − q), ±vlim), torque = kv · (v_des − dq) + model bias torque (gravity, Coriolis) on J1..J6; vlim 2.0 rad/s as sent by armd.
  - MIT (per-joint switch used by gravity compensation): torque = kp · (q_target − q) + kd · (dq_target − dq) + τ_ff.
  - Disabled: no torque; the arm falls under gravity.
  - Torque limits = Damiao peak torque: 27 N·m (J1..J3), 7 N·m (J4..J7). The POS_VEL velocity target is capped at the motor's no-load speed (5.45 / 20.9 rad/s; armd sends vlim 2.0, so the cap does not bind in normal use).
  - Rotor inertia reflected through the gearbox as joint armature (table below).
- **Feedback** (250 Hz): positions, velocities, total motor torque (servo + feed-forward, so gravity and payload load show up), temperatures constant 35 °C, enable state.

### Motor parameters

Motors per joint from the reBot B601-DM BOM ([hardware/reBot_B601_DM/readme.md](https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/reBot_B601_DM/readme.md): DM4310(V4) × 4, DM4340P(V4) × 3; the v1.1 changelog moves joint 1 from 4310 to 4340P). The split J1..J3 = DM4340P, J4..J7 = DM4310 matches the PMAX / VMAX / TMAX registers read from the arm on 2026-11-09 (J1..J3 VMAX 10, TMAX 28; J4..J7 VMAX 30, TMAX 10; [control_safety_contract.md](control_safety_contract.md)) and the Seeed wiki defaults per model (4340P: VMAX 8, TMAX 28; 4310: VMAX 30, TMAX 10).

Sources: [S1] Seeed wiki, Damiao series parameter table, https://wiki.seeedstudio.com/damiao_series/ ; [S2] Damiao DM-J4340P-2EC user manual V1.0 (2025-01-10), https://doc.switch-science.com/media/files/5e033168-99f1-4d2d-b758-e03192a1f071.pdf ; [S3] Seeed product page DM4310 / DM4340P table, https://www.seeedstudio.com/Damiao-4310-Actuator-Motor-p-6823.html ; [S4] Damiao DM-J4310-2EC V1.1 user manual, https://sharingwin.com/wp-content/uploads/2025/09/DM-J4310-2EC-V1.1-Gear-Motor-User-Manual-V1.0.pdf (parameter table is an image; values taken from S1 / S3).

| parameter | J1..J3 DM-J4340P-2EC | J4..J7 DM-J4310-2EC V1.1 | source | used in the sim |
|---|---|---|---|---|
| gear ratio | 40 | 10 | S1, S2, S3 | armature |
| rated / peak torque (output) | 9 / 27 N·m | 3 / 7 N·m | S1, S2, S3 | `forcerange`, `TORQUE_LIMIT` = peak |
| rated / no-load speed at 24 V | 36 / 52 rpm | 120 / 200 rpm | S2, S3 | v_des cap 5.45 / 20.9 rad/s |
| torque constant (output side) | 4.074 N·m/A | 0.945 N·m/A | S1 | not used (no current model) |
| rotor inertia (motor side) | 2.0e-5 kg·m² | 1.8e-5 kg·m² | S1 | armature |
| armature = J_rotor × N² | 0.032 kg·m² | 0.0018 kg·m² | computed | joint `armature` (J1..J6) |
| J7 armature on the jaw slides | — | 0.0018 × (5.7 / 0.0425)² = 32.4 kg, 16.2 kg per jaw | computed | `finger_left` / `finger_right` `armature` |
| phase resistance / inductance | 0.88 Ω / 360 µH | 0.85 Ω / 345 µH (S3: 0.65 Ω) | S1, S2, S3 | not used |
| joint damping | 0.8 N·m·s/rad | 0.8 N·m·s/rad | assumption: reBot-DevArm DM MJCF default, no Damiao data | `damping` |
| joint friction (Coulomb) | 0.05 N·m | 0.05 N·m | assumption, no Damiao data | `frictionloss` |
| POS_VEL loop KPOS / KV | 25 1/s, 8 N·m·s/rad | 25 1/s, 1.5 (J7 0.5) N·m·s/rad | assumption (S1 lists a speed-loop KP in driver units, not convertible without the current-loop scaling) | `server.py` |
| jaw damping / friction | — | 2 N·s/m / 0.2 N | assumption | jaw joints |

Damping and Coulomb friction can be identified on the arm with `rebot-sysid` (below) and passed to the simulator with `--damping` / `--frictionloss`.

### Wire protocol (UDP, JSON, version 1)

One JSON object per datagram.

| direction | message |
|---|---|
| armd → sim | `{"v":1,"type":"cmd","seq":N,"enable":bool,"targets":[T × 7] \| null}` per control tick and on enable / disable (`targets: null`) |
| armd → sim | `{"v":1,"type":"poll","seq":0,...}` keep-alive every 20 ms without other traffic |
| sim → armd | `{"v":1,"type":"state","session":S,"seq":N,"ack":N,"enabled":bool,"q":[7],"dq":[7],"tau":[7],"temp":[7],"t":s}` to the sender of the last datagram |

`T` is `{"mode":"pos_vel","q":rad,"vlim":rad/s}` or `{"mode":"mit","q":rad,"dq":rad/s,"kp":N·m/rad,"kd":N·m·s/rad,"tau":N·m}`. `ack` is the `seq` of the last applied command; `session` is random per simulator start.

armd side (`src/sim_driver.rs`): a background thread receives state into a cache; driver calls inside the control tick do not wait on the network. State with a wrong version, wrong array length, non-finite values or a `seq` not above the last one of the same session is dropped. Feedback older than 200 ms makes `is_feedback_valid` false and the enable state Unknown; the supervisor latches `BUS_LOSS` after 500 ms without fresh feedback (`BUS_LOSS_TIMEOUT`), as on hardware. Enable waits up to 80 ms for a state that acknowledges the enable command (disable: 25 ms), like the Damiao handshake.

## Friction identification (`rebot-sysid`)

`rebot-sysid` (`sim/mujoco/src/rebot_sim/sysid.py`, console script of the `sim/mujoco` uv project) identifies Coulomb and viscous friction per joint J1..J6 through armd and suggests collision thresholds. It runs on a PC with numpy / matplotlib and connects to armd as an ordinary WebSocket client (`websockets`); nothing is installed on the reCamera. The same command works against `armd --follower sim` and the real arm.

Procedure:

1. Takes the control lease, refuses to start unless `--i-am-at-the-arm` is given, and asks for `yes` before moving (`--yes` skips the prompt). Refuses when torque is off unless `--enable` is given.
2. Optionally jogs to `--pose` (J1..J6, rad); otherwise sweeps around the current pose. This jog, the return to the start pose and the home jog take duration = largest J1..J6 change / `--approach-speed` (default 0.1 rad/s), at least 3 s. Every joint vector is checked against the schema `dof` before it is sent.
3. For each joint in `--joints`, with the other joints held: servo stream at 50 Hz, trapezoidal back-and-forth sweep ±`--amplitude` (default 0.15 rad, max 0.35) at each of `--speeds` (default 0.1, 0.2, 0.3 rad/s, max 0.6; at least two speeds, needed to separate Coulomb from viscous), `--cycles` (default 2) per speed, turnarounds at `--accel` (default 1.0 rad/s²). armd checks joint limits and the workspace on every frame.
4. Records q, dq, torque and `residual_torques` (torque − armd gravity model) from the observations (`obs_hz=50`). Aborts with `motion/stop` when a residual moves more than `--abort-nm` (default 3, 3, 3, 1, 1, 1 N·m) from its value at the start (default 4, 4, 4, 2.5, 2.5, 2.5 N·m; 1.0 on J4..J6 tripped on normal J4 friction swings on the real arm), on any fault / E-stop / lease loss, on Ctrl-C, and on any servo rejection. After an abort or error the joints whose sweeps had finished are fitted and written (`report.json` / `samples.json` field `aborted` holds the reason, `joints_finished` the joints); then, when the arm is torqued and armd is Armed or Moving, it jogs slowly back to the start pose (to the pose before the approach when the error came during the approach jog) and, with `--unload`, goes home and unloads as at the end of a run. In any other state the arm is not moved. The tool then exits with the error.
5. Returns to the start pose and fits, on the constant-velocity samples (velocity within 10 % of the sweep speed, position clear of the turnaround ramps: |q − q_start| ≤ amplitude − v²/(2·accel) − 0.15 s · v − 0.01 rad, so no acceleration torque enters). The velocity is taken from position differences over ±2 samples (80 ms), not from the reported velocity (see the sampling-phase section):
   `r = Fc · sign(v) + Fv · v + offset + kq · (q − q_start)`; `kq` absorbs gravity-model error that varies over the sweep. Per direction it also fits `r = a + b · v` (asymmetric friction), and a Coulomb-only fit `r = Fc · sign(v) + offset + kq · (q − q_start)` (`coulomb_only`). A fit with Fv < 0 (seen on the real arm) is marked `viscous_reliable: false` with a `viscous_note`.
6. Suggested collision threshold per joint = max |r − Coulomb-only fit(v, q)| over every recorded sample (Coulomb friction plus the residual spread, no viscous term; `collision.basis: coulomb_only`) (all sweeps, including turnarounds and the time the joint stands still while others move) × `--safety-factor` (default 1.5), at least `--threshold-floor` (0.1 N·m).

Output in `--out` (default `sysid-out/`): `report.json` (per joint `coulomb_nm`, `viscous_nm_s_rad`, `offset_nm`, `position_slope_nm_rad`, `rmse_nm`, `positive` / `negative` direction fits, `collision.max_abs_deviation_nm`, `collision.p99_abs_deviation_nm`, `collision.suggested_threshold_nm`), `samples.json` (raw observations and sweep windows), `friction_J<n>.png` (residual vs velocity with the fit). Without a gravity model armd reports no `residual_torques`; the tool then fits raw torques and warns.

### On the real arm

```bash
cd sim/mujoco
uv run rebot-sysid --url http://<recamera-ip>:8765 --i-am-at-the-arm \
  --pose 0,-1.0,-1.2,0.6,0,0 --joints 1,2,3,4,5,6 --out sysid-$(date +%Y%m%d)
```

- A person stays at the arm with the e-stop in reach for the whole run (about 30 s of motion per joint with the defaults). Ctrl-C stops the motion; armd keeps holding the pose.
- Choose `--pose` with every joint at least `--amplitude` away from its limit and the tool clear of the table and of anything the sweep can reach; J1 swings the whole arm by ±amplitude. Start with `--joints 6` and the defaults, look at the plot, then widen.
- Close the cockpit or any other client first: the tool needs the lease. At the end it returns to the start pose and releases the lease; armd then holds and latches the usual `WatchdogTimeout` fault for a torqued arm without a lease owner (clear it in the cockpit). `--unload` parks and turns torque off instead; with it `--return-home` (default on with `--unload`, `--no-return-home` turns it off) first jogs slowly to the profile home pose so that the park is a small move. The home pose is read from `home_positions` in `GET /api/v1/observation/schema`; armd does not publish that field today, so the home jog goes to the start pose (no move) and the park runs from there at armd's park speed. The gripper is not moved by the home jog.
- Speeds: use at most 0.08 rad/s for J1..J3 on the real arm (for example `--joints 1,2,3 --speeds 0.04,0.08`, then `--joints 4,5,6` with the defaults).
- Warm motors have lower friction than cold ones: note the motor temperatures (in `samples.json`) and repeat after the arm has run for a while if the thresholds are meant for continuous operation.
- The simulator values can be updated from the report: `rebot-sim --damping <viscous J1..J6> --frictionloss <coulomb J1..J6>`.

### What the fit measures, and the sampling-phase effect

armd ticks at 50 Hz: each tick it reads the motor feedback, then sends the next position target, so torque readings are taken at a fixed phase of the command period. While a joint moves at constant speed the target advances in steps of v / 50 rad per tick and the motor's position loop answers each step with a torque transient. In `rebot-sim` (instantaneous torque, assumed KPOS 25 / KV 8) the J1 torque at 0.1 rad/s swings between 0.28 and 0.65 N·m within each 20 ms tick around a mean of 0.450 N·m (= 0.3 Coulomb + 1.5 × 0.1 viscous); armd samples the 0.28–0.31 end of every cycle. The ripple scales with v, so the sampled torque underestimates the viscous term: with `--torque-report instant` the fit returned Fc 0.299 N·m (set 0.3) but Fv −0.18 N·m·s/rad (set 1.5) on J1. The reported velocity has the same phase bias (J6 at 0.3 rad/s reads 0.256 rad/s on average); positions sampled at a fixed phase still difference to the true mean velocity, which is why the fit uses position differences. With `--torque-report mean` (sliding mean over the last command period) the torque ripple averages out.

A mean torque lags the position it is paired with by about half a command period. On gravity-loaded joints the gravity term then changes by G′(q) · v · lag between the two, which adds to the fitted viscous coefficient with the sign of G′(q). Reporting the mean of the previous complete command interval (lag about 1.5 periods) gave Fv errors of +16 % (J2) and −21 % (J4) at an otherwise exact Fc; the sliding window reduces the lag to about half a period.

On the real arm the Damiao reply to each command carries the torque at that moment, so the same phase effect is possible; its size depends on the motor's internal loops and is not measured yet. The fitted Fc / Fv then describe the torque as armd reads it, which is what armd's residual-based checks see, and can differ from the physical friction. A Fv close to zero or negative on a joint indicates this effect. Needs verification on the hardware.

### Recovery in the simulator

`tests/test_sysid.py` (marker `slow`) starts `rebot-sim --torque-report mean` with damping 1.5, 1.2, 1.5, 0.3, 0.25, 0.3 N·m·s/rad and friction loss 0.3, 0.4, 0.3, 0.1, 0.12, 0.1 N·m, runs `rebot-sysid --pose 0,-1.6,-1.2,0.9,0,0` with the defaults against `armd --follower sim`, and requires every fitted Fc and Fv within 15 % of the set value.

Result (spark-6124, 2026-11-11, 3 speeds × 2 cycles, ±0.15 rad, 230 s test; thresholds on the Coulomb-only basis):

| joint | Fc fitted / set (N·m) | error | Fv fitted / set (N·m·s/rad) | error | suggested threshold (N·m) |
|---|---|---|---|---|---|
| J1 | 0.303 / 0.3 | +1.0 % | 1.479 / 1.5 | −1.4 % | 0.742 |
| J2 | 0.401 / 0.4 | +0.2 % | 1.269 / 1.2 | +5.7 % | 0.911 |
| J3 | 0.302 / 0.3 | +0.5 % | 1.458 / 1.5 | −2.8 % | 0.765 |
| J4 | 0.100 / 0.1 | +0.3 % | 0.276 / 0.3 | −8.0 % | 0.112 |
| J5 | 0.120 / 0.12 | +0.2 % | 0.240 / 0.25 | −4.2 % | 0.119 |
| J6 | 0.100 / 0.1 | 0.0 % | 0.300 / 0.3 | −0.2 % | 0.167 |

The thresholds include the turnaround acceleration torque (J1 swings the extended arm at 1 rad/s²), since that is part of normal motion; a lower `--accel` lowers them.

## Measured on this machine

Host spark-6124 (aarch64, Cortex-X925, 20 cores), MuJoCo 4.0.0, Python 3.13:

| measurement | value |
|---|---|
| unpaced physics (`rebot-sim --benchmark 20000`) | 36,960 and 37,045 steps/s (2 runs) |
| paced simulator (`--stats 2`) | 500.0–500.5 Hz |
| FK agreement armd vs MuJoCo (`test_sim_fk`, 7 joint vectors) | worst 0.0001 mm, 0.0000° |
| `move_to` [0.30, 0, 0.12] tool down, TCP error after settle | 0.02 mm (0.04 mm with the earlier assumed armature 0.05 / 0.01 and J7 limit 1.5 N·m) |
| hold torques at that pose (J1..J7, N·m) | 0.01, 7.06, −5.89, −0.23, 0.00, 0.00, 0.00 (before: 0.01, 7.05, −5.90, −0.23, 0.01, 0.00, 0.00) |
| gripper open at 2.0 rad/s (ack to done) | 3.56 s (before: 3.52 s) |
| close on the cube (`grasp: true`, ack to done) | 3.66 s, `holding: true`, gap 2.66 rad, τ7 1.00 N·m (before: 4.02 s, gap 2.64 rad, τ7 1.01 N·m) |
| time from simulator kill to `BUS_LOSS` | 0.51 s |

## Not simulated / limitations

- Motor dynamics are simplified: no current loop, no torque-speed curve, no cogging, no CAN latency or frame loss model; the POS_VEL integrator is replaced by bias-torque feed-forward. Torque limits, no-load speed and rotor inertia are datasheet values; loop gains, joint damping (0.8 N·m·s/rad) and friction loss (0.05 N·m) are assumptions until identified on the arm (`rebot-sysid`).
- Contact parameters and friction (jaws 1.5, cube 1.2) are guessed; collision shapes are boxes, not meshes. The links do not collide with each other.
- J7 torque scale: holding and stall thresholds (`gripper_policy.holding_torque_nm`, the 0.444 N·m stall latch) are met through the assumed velocity-loop gain (KV 0.5 × vlim 2.0 = 1.0 N·m squeeze, below the 7 N·m peak), not by a measured gripper model.
- No camera image yet: vision, `grasp` (point-and-grasp) and detection need the real camera or a separate image source.
- No motor faults (over-temperature, under-voltage); temperature is constant. Damiao register access (mapping-range check, CTRL_MODE writes) is not simulated: `driver_range_check` is null and MIT switching is confirmed locally.
- With the default `speed` 1.0 rad/s the jaw reached −5.699 rad from closed after about 5.9 s, too late for the 200 ms settle within the 6 s default `timeout_s`: the gripper action ended `GRIPPER_TIMEOUT`. The test opens at 2.0 rad/s. Not verified on the real gripper.
- The simulator runs on the host only; it is not packaged for the reCamera.
