# Agent action API / 动作接口

High-level arm actions for agents (MCP server, scripts). They run on the WebSocket `/api/v1/ws` under the same control lease, epoch and `request_id` rules as every other `command`. The REST paths `POST /api/v1/action/*` exist only to return `409 LEASE_REQUIRED` / `LEASE_ACTIVE`.

面向 agent（MCP server、脚本）的高层动作。走 WebSocket `/api/v1/ws` 的 `command`，租约、epoch、`request_id` 规则与其他指令相同。REST 路径 `POST /api/v1/action/*` 只返回 `409 LEASE_REQUIRED` / `LEASE_ACTIVE`。

Profiles without `kinematics` (any `--config` profile that omits it) return `UNSUPPORTED` for `move_to` / `move_relative`; without `gripper_policy`, `gripper` returns `UNSUPPORTED`. The built-in `rebot-dm` profile has all three.

未配置 `kinematics` 的 profile 对 `move_to` / `move_relative` 返回 `UNSUPPORTED`；未配置 `gripper_policy` 时 `gripper` 返回 `UNSUPPORTED`。内置 `rebot-dm` profile 三者都已配置。

## Message / 消息

```json
{"type":"command", "request_id":42, "generation":7, "epoch_id":3,
 "path":"/api/v1/action/move_to", "body":{"position":[0.30,0.0,0.25]}}
```

Accepted: `{"type":"command_ack", "request_id":42, "status":"accepted", "action_id":5, "expected_duration_s":2.4, ...}`. Rejected: `{"type":"command_reject", "request_id":42, "status":422, "code":"IK_UNREACHABLE", "message":"..."}`.

The existing `/api/v1/motion/jog`, `/motion/home`, `/motion/unload` commands and WS `preset` keep their bodies and now also create an action record (kind `joint_move`, `home`, `park`); their `command_ack` / `preset_ack` carry `action_id`.

已有的 jog / home / unload 指令与 WS `preset` 请求体不变，同时生成动作记录（`joint_move` / `home` / `park`），ack 带 `action_id`。

## Paths / 路径

| path | body |
|---|---|
| `/api/v1/action/move_to` | `{"position":[x,y,z], "orientation": {"rpy":[r,p,y]} \| {"quat":[w,x,y,z]} \| null, "speed": {"linear":m/s, "angular":rad/s}?, "tag": str?}` |
| `/api/v1/action/move_relative` | `{"frame":"base"\|"tool"\|"camera", "d":[dx,dy,dz]?, "dyaw":rad?, "dpitch":rad?, "droll":rad?, "level":bool?, "speed"?, "tag"?}` |
| `/api/v1/action/gripper` | `{"command":"open"\|"close"\|"move", "fraction":0..1? (move only; 0 closed, 1 open), "grasp":bool? (close only, default true), "speed":rad/s? (≤2.0, default 1.0), "timeout_s":? (default 6, ≤30), "tag"?}` |

- Poses are the end_link pose in base_link, metres and radians (URDF `assets/reBot-DevArm_gripper.urdf`; end_link x is the tool axis). `rpy` is URDF fixed-axis R = Rz(yaw)·Ry(pitch)·Rx(roll). `orientation: null` keeps the current orientation.
- Motor angle = URDF model angle for J1..J6 (the cockpit's `?iksign` / `?ikoff` overrides are not applied).
- `tag`: string ≤ 64 bytes, echoed in the action record.
- `width_m` for the gripper returns `UNSUPPORTED` until the jaw angle-to-width mapping is measured.

位姿为 end_link 在 base_link 下的位姿（米、弧度），end_link x 为工具轴。`rpy` 为 URDF 固定轴 Rz·Ry·Rx。`orientation: null` 保持当前姿态。J1..J6 电机角 = URDF 模型角。

### `move_relative` frames / 坐标系

| frame | translation `d` | rotations (about the end_link origin) |
|---|---|---|
| `base` | base axes | `dyaw` about world Z; `dpitch` about unit(Z × x_tool), positive tips the tool down; `droll` about tool x |
| `tool` | end_link axes | `droll` / `dpitch` / `dyaw` about end_link x / y / z (intrinsic, applied z·y·x) |
| `camera` | OpenCV camera axes (x right, y down, z optical) | `dpitch` > 0 looks down (−dpitch about x_cam); `dyaw` > 0 looks right (−dyaw about world Z); `droll` about z_cam. `level` (default true) then rolls x_cam horizontal unless the optical axis is within 15° of vertical |

The camera rotation is R_base_cam = R_base_end · R_end_cam, R_end_cam from `config.camera_mount` (`flip`, `rot_deg`, `tilt_deg`; defaults = cockpit defaults, optical axis = end_link x). `level` is only accepted for `camera`.

### Speed / 速度

Defaults linear 0.05 m/s (max 0.15), angular 0.5 rad/s (max 1.0). Joint speed is capped by `planned_max_velocity` (0.45 rad/s on rebot-dm), joint acceleration by min(limit, 3.0 rad/s²).

默认线速度 0.05 m/s（上限 0.15），角速度 0.5 rad/s（上限 1.0）。关节速度受 `planned_max_velocity`（rebot-dm 0.45 rad/s）限制，加速度上限 min(限位, 3.0 rad/s²)。

## Planning and execution / 规划与执行

1. Under the lock: lease, epoch, `request_id`, then preconditions — Armed/Moving, no pending re-arm / park / handover, no trajectory, no servo target, all commanded velocities < 1e-3 (else `MOTION_BUSY`).
2. Without the lock (blocking thread): straight line in position, slerp in rotation, one sample per 5 mm / 2° (≤ 400), DLS IK per sample seeded with the previous one, within joint limits − 0.02 rad. A joint change > 0.10 rad between samples is `IK_BRANCH_FLIP`; every sample passes the workspace rule. One quintic time scaling over the whole path.
3. Lock again: same lease and generation, epoch unchanged, still idle and commanded positions unchanged (< 1e-4 rad), else `PLAN_STALE`. Then the path is installed and acknowledged.

Execution: each tick advances path time by dt·rate with rate = clamp((0.45 − lag)/(0.45 − 0.38), 0, 1), lag = max over J1..J6 of |commanded − measured|. Geometry stays the same, only speed drops. Rate 0 for 1 s ends the action as `TRACKING_STALL` (commanded reset to measured, no Fault). After the path: settle until |commanded − measured| < 0.05 rad and |v| < 0.05 rad/s for 200 ms (`settled:true`), at most 2 s (`settled:false`). J7 holds its commanded value during arm paths.

Gripper: J1..J6 hold; J7 steps at `speed`. `open` / `move` are done at the target (±0.05 rad, still 200 ms) or on a stall. `close` with `grasp`: on contact (stall latch, or still 200 ms after moving) J7 holds min(measured + `squeeze_rad`, `closed_rad`) for `settle_ms`; `holding` = gap ≥ `holding_gap_rad` and median |τ7| ≥ `holding_torque_nm`. The hold target stays in `commanded_positions[6]` after the action. Timeout is `GRIPPER_TIMEOUT` (J7 held at measured). The existing stall protection still applies: a sustained |τ7| ≥ 0.8 N·m re-latch resets J7 to measured.

执行：每 tick 按 dt·rate 推进路径时间，滞后超过 0.38 rad 开始降速，0.45 rad 停止；持续 1 s 停止则 `TRACKING_STALL`（指令重置为实测，不进入 Fault）。夹爪闭合抓取：接触后保持 min(实测 + `squeeze_rad`, `closed_rad`) `settle_ms` 毫秒，按间隙和扭矩中位数判定 `holding`。

## Action status / 动作状态

Every observation carries `action` (latest record) and, for 2 s after a newer action replaced a running one, `previous_action`:

```json
"action": {"id":5, "kind":"move_to", "tag":"pick", "state":"running", "code":null, "message":null,
           "progress":0.42, "started_at":812.3, "finished_at":null,
           "holding":null, "gripper_gap_rad":null, "settled":null}
```

`kind`: `move_to`, `move_relative`, `gripper`, `joint_move`, `home`, `park`. `state`: `running`, `done`, `failed`, `cancelled`. `started_at` / `finished_at` use the observation `timestamp` clock. Clients block by waiting until `action.id == action_id` and `state != "running"`, or until `previous_action.id == action_id`.

A running action ends when its reference goes away:

| cause | state / code |
|---|---|
| E-stop | `failed` / `ESTOP_ACTIVE` |
| Fault | `failed` / `FAULT` (message = reason) |
| lease watchdog (park or `WatchdogTimeout`) | `failed` / `WATCHDOG` |
| lease released, preempted or connection closed | `cancelled` / `LEASE_LOST` |
| new jog / preset / home / park / action | `cancelled` / `SUPERSEDED` |
| `/motion/stop` | `cancelled` / `STOPPED` |
| park ended with torque on | `failed` / `UNLOAD_FAILED` |

Servo frames while an action runs are rejected with `MOTION_BUSY`. Jog, preset, home and park are accepted and supersede it.

## Errors / 错误码

| code | status |
|---|---|
| `IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION` | 422 |
| `MOTION_BUSY`, `PLAN_STALE` | 409 |
| `INVALID_COMMAND` (bad or missing params, path too long) | 400 |
| `UNSUPPORTED` | 422 |

Codes that only appear in `action.code`: `TRACKING_STALL`, `GRIPPER_TIMEOUT`, `ESTOP_ACTIVE`, `FAULT`, `WATCHDOG`, `LEASE_LOST`, `STOPPED`, `SUPERSEDED`, `UNLOAD_FAILED`.

## Needs verification on the device / 需实机核实

- POS_VEL lag gain K = 1.5, measured 2026-11-09 (basis of the 0.45 rad/s cap and the 0.38 / 0.45 rad band).
- `squeeze_rad` 0.15 and `holding_torque_nm` 0.12 come from the MIT-mode project; grip torque under POS_VEL depends on the motor's internal gains.
- `camera_mount` (`flip`, `tilt_deg`, `offset_m`) for camera-frame moves.
- Planning time on the SG2002: `cargo test --release --test test_actions bench_plan -- --ignored --nocapture`.
