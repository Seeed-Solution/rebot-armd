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
| `/api/v1/action/grasp` | `{"label":str, "box_norm":[x0,y0,x1,y1], "frame_id":int?, "ts_ms":number?, "overrides":{setup fields}?, "tag"?}` — see [Grasp](#grasp--点框抓取) |
| `/api/v1/grasp/setup` | grasp setup patch, see [Grasp setup](#grasp-setup--抓取设置) (not an action: acknowledged without `action_id`) |
| `/api/v1/action/move_to_pose` | `{"name":str, "include_gripper":bool? (default false), "speed_scale":0.1..1.0? (default 1.0), "tag"?}` — see [Named poses](#named-poses--命名姿态) |
| `/api/v1/action/place` | `{"pose":str? \| "position":[x,y,z]?, "orientation"?, "approach_height_m":0.02..0.30? (default 0.08), "release":bool? (default true), "retreat":bool? (default true), "speed_scale":0.1..1.0? (default 1.0), "tag"?}` — see [Place](#place--放置) |
| `/api/v1/poses/save`, `/api/v1/poses/delete` | `{"name":str, "positions":[dof]?}` / `{"name":str}` (not actions: acknowledged without `action_id`) |
| `/api/v1/workspace/keepout` | `{"boxes":[{"name":str, "min_m":[x,y,z], "max_m":[x,y,z]}]}` — see [Keep-out boxes](#keep-out-boxes--禁入盒) (not an action) |
| `/api/v1/action/replay` | `{"episode":str, "speed":0.5\|1.0? (default 0.5), "gripper":bool? (default true), "tag"?}` — see [Replay](#replay--回放) |

- Poses are the end_link pose in base_link, metres and radians (URDF `assets/reBot-DevArm_gripper.urdf`; end_link x is the tool axis). `rpy` is URDF fixed-axis R = Rz(yaw)·Ry(pitch)·Rx(roll). `orientation: null` keeps the current orientation.
- Motor angle = URDF model angle for J1..J5; URDF J6 = motor J6 − π/2 (B601-DM, profile `model_offsets_rad`, published as `model_joint_map` in `GET /api/v1/observation/schema`: URDF angle = `sign` × motor angle + `offset_rad`). Joint positions in requests, observations and stored poses are motor angles. The cockpit's `?iksign` / `?ikoff` overrides are not applied. URDF J6 = motor J6 − π/2 was found on 2026-11-12: at the rest pose (motor J6 = π/2) photos of the arm show the jaws opening left-right and the camera holder (end_link +z) on top, which URDF FK gives only at URDF J6 = 0. J1–J5 directions were checked on the arm on 2026-11-09.
- `tag`: string ≤ 64 bytes, echoed in the action record.
- `width_m` for the gripper returns `UNSUPPORTED` until the jaw angle-to-width mapping is measured.

位姿为 end_link 在 base_link 下的位姿（米、弧度），end_link x 为工具轴。`rpy` 为 URDF 固定轴 Rz·Ry·Rx。`orientation: null` 保持当前姿态。J1..J5 电机角 = URDF 模型角；URDF J6 = 电机 J6 − π/2（B601-DM，profile 字段 `model_offsets_rad`，在 `GET /api/v1/observation/schema` 中以 `model_joint_map` 发布：URDF 角 = `sign` × 电机角 + `offset_rad`）。请求、观测和已存点位中的关节角都是电机角。不使用 cockpit 的 `?iksign` / `?ikoff`。URDF J6 = 电机 J6 − π/2 为 2026-11-12 的结论：静置位姿（电机 J6 = π/2）下的真机照片中夹爪左右张合、相机支架（end_link +z）在上方，URDF 正运动学只在 URDF J6 = 0 时得到这一姿态。J1–J5 的方向于 2026-11-09 在真机上检查过。

### `move_relative` frames / 坐标系

| frame | translation `d` | rotations (about the end_link origin) |
|---|---|---|
| `base` | base axes | `dyaw` about world Z; `dpitch` about unit(Z × x_tool), positive tips the tool down; `droll` about tool x |
| `tool` | end_link axes | `droll` / `dpitch` / `dyaw` about end_link x / y / z (intrinsic, applied z·y·x) |
| `camera` | OpenCV camera axes (x right, y down, z optical) | `dpitch` > 0 looks down (−dpitch about x_cam); `dyaw` > 0 looks right (−dyaw about world Z); `droll` about z_cam. `level` (default true) then rolls x_cam horizontal unless the optical axis is within 15° of vertical |

The camera rotation is R_base_cam = R_base_end · R_end_cam, R_end_cam from `config.camera_mount` (`flip`, `rot_deg`, `tilt_deg`; defaults = cockpit defaults, optical axis = end_link x). With `rot_deg` 0 and `flip` false, image y (down) = −end_link z and image x = −end_link y: a camera above the jaws whose image is upright at the rest pose (the mount is referenced to J6 at its rest angle, motor π/2). `level` is only accepted for `camera`.

相机旋转 R_base_cam = R_base_end · R_end_cam，R_end_cam 来自 `config.camera_mount`（`flip`、`rot_deg`、`tilt_deg`；默认值同 cockpit，光轴 = end_link x）。`rot_deg` 0、`flip` false 时，图像 y（向下）= −end_link z，图像 x = −end_link y，即相机位于夹爪上方、静置位姿下图像正立（安装以 J6 静置角、电机 π/2 为参考）。`level` 只用于 `camera`。

### Speed / 速度

Defaults linear 0.05 m/s (max 0.15), angular 0.5 rad/s (max 1.0). Joint speed is capped by `planned_max_velocity` (0.45 rad/s on rebot-dm), joint acceleration by min(limit, 3.0 rad/s²).

A profile `speed_cap` (`linear_mps`, `angular_rps`, `joint_velocity_scale`) lowers these further: the request speed is accepted within the limits above, then min(request, cap) is planned, and the joint speed cap is multiplied by `joint_velocity_scale` (gripper excluded). It applies to every planned motion (actions, jog, home, park, `move_to_pose`), cannot be changed at runtime and is published as `schema.speed_cap`.

默认线速度 0.05 m/s（上限 0.15），角速度 0.5 rad/s（上限 1.0）。关节速度受 `planned_max_velocity`（rebot-dm 0.45 rad/s）限制，加速度上限 min(限位, 3.0 rad/s²)。

profile 的 `speed_cap`（`linear_mps`、`angular_rps`、`joint_velocity_scale`）进一步降低速度：请求值在上述范围内被接受，规划时取 min(请求值, 上限)，关节限速乘以 `joint_velocity_scale`（夹爪除外）。对所有规划运动生效（动作、jog、home、park、`move_to_pose`），运行时不可修改，发布在 `schema.speed_cap`。

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

`kind`: `move_to`, `move_relative`, `gripper`, `grasp`, `place`, `joint_move`, `home`, `park`, `move_to_pose`, `replay`. Planned motions carry `power_scale` when the profile has a power budget (see [Power budget](#power-budget--电源功率预算)). A `grasp` record also carries `phase`, `iteration` and `error_px`, and, for camera-down acquisition, `final_offset_px` and `acquire_height_m` (fields absent otherwise); a `place` record carries `phase` (all three are null for other kinds); a `replay` record carries `phase`, `episode` and `t_s` (the last two absent for other kinds). `state`: `running`, `done`, `failed`, `cancelled`. `started_at` / `finished_at` use the observation `timestamp` clock. Clients block by waiting until `action.id == action_id` and `state != "running"`, or until `previous_action.id == action_id`.

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

## Named poses / 命名姿态

`GET /api/v1/poses` (no lease) returns `{"poses":[{"name", "positions":[dof], "tcp":{"position":[x,y,z], "rpy":[r,p,y]} | null, "saved_at_ms"}]}`. `tcp` is the end_link pose at save time (null without `kinematics`); `saved_at_ms` is Unix time.

- `/api/v1/poses/save` (lease): `name` matches `^[A-Za-z0-9_-]{1,64}$`; `positions` defaults to the measured positions and must be within the joint limits. Saving an existing name replaces it. At most 64 poses. The ack carries `pose`.
- `/api/v1/poses/delete` (lease): an unknown name is `404 POSE_NOT_FOUND`.
- Both acks carry `persisted` (true / false with `persist_error`). The store is written atomically to `ARMD_POSES` (default `/userdata/rebot-teleop/poses.json`) after the supervisor lock is released. At start a valid file is loaded; an unreadable or invalid file is logged and the store starts empty; a malformed or invalid file is renamed to `poses.json.bad-<unix seconds>` so the next save cannot overwrite it.
- `/api/v1/action/move_to_pose`: joint-space quintic of J1..J6 (and J7 with `include_gripper`) to the stored positions, planned with the joint speed cap × `speed_scale`. Action kind `move_to_pose`; same preconditions as the other actions (`MOTION_BUSY` while a trajectory or servo stream runs) and the same cancellation causes. The path's verification samples (21, more with keep-out boxes: see [Keep-out boxes](#keep-out-boxes--禁入盒)) pass the joint limits and, with `kinematics` and `workspace`, the workspace rule including keep-out boxes. Errors: `POSE_NOT_FOUND` (404), `WORKSPACE_VIOLATION` (422), `INVALID_COMMAND` (400).

`GET /api/v1/poses`（无需租约）返回已保存的姿态。`/api/v1/poses/save`、`/api/v1/poses/delete` 需要租约：名称须匹配 `^[A-Za-z0-9_-]{1,64}$`，`positions` 默认取实测位置并须在关节限位内，同名覆盖，最多 64 个；删除不存在的名称返回 `404 POSE_NOT_FOUND`。文件在释放锁后原子写入 `ARMD_POSES`（默认 `/userdata/rebot-teleop/poses.json`）；启动时文件无效则记录日志、以空列表启动；格式错误或无效的文件重命名为 `poses.json.bad-<unix 秒>`，之后的保存不会覆盖它。`move_to_pose` 以关节空间五次多项式移动 J1..J6（`include_gripper` 时含 J7），速度为关节限速 × `speed_scale`，动作类型 `move_to_pose`，前置条件与取消规则同其他动作，路径的校验采样点（21 个，有禁入盒时更多，见“禁入盒”）须满足关节限位和工作空间规则（含禁入盒）。

## Keep-out boxes / 禁入盒

Axis-aligned boxes in the base frame that no workspace check point may enter. They are checked wherever the workspace rule applies (servo targets, every sample of Cartesian paths, the samples of jog / home / preset / `move_to_pose` quintics: 21, plus with keep-out boxes the samples of the sweep check described in the safety contract, at most 4000 added FK evaluations, else `WORKSPACE_VIOLATION`; servo and braking steps are checked the same way, at most 64 added evaluations per tick, else the stream stops in the recoverable hold; park is checked too: it is exempt from the floor, box and reach limits only, so a park through a box is `WORKSPACE_VIOLATION` and a watchdog park through a box holds position and latches `WATCHDOG_TIMEOUT`) on the same check points (end_link origin, `check_points_end_m`, camera offset). Entering a box is `WORKSPACE_VIOLATION` with `keepout '<name>'` in the message; as with the other limits, a move that starts inside a box and does not go deeper is allowed.

- Profile: `workspace.keepout_boxes` (default empty, at most 16).
- `GET /api/v1/workspace/keepout` returns `{"boxes":[{name, min_m, max_m, "source":"profile"|"runtime"}, ...], "source":"profile"|"runtime"}`: the boxes in force, profile boxes first; the top-level `source` is `runtime` once a runtime list was set.
- WS command `/api/v1/workspace/keepout` `{"boxes":[...]}` (lease) replaces the runtime list: names 1..64 bytes, finite, `min_m < max_m` on every axis, at most 16 boxes in force (profile + runtime), else `400 INVALID_COMMAND`. Profiles without `kinematics` and `workspace` return `UNSUPPORTED`. The boxes in force are the profile boxes plus the runtime list: a runtime list only adds boxes and cannot remove a profile box; `{"boxes": []}` clears the runtime list. While a trajectory, macro action, servo stream or park runs, or the arm still moves, the command is `409 MOTION_BUSY`; an action planned before a change is `409 PLAN_STALE` at install. It is written to `ARMD_KEEPOUT` (default `/userdata/rebot-teleop/keepout.json`, ack field `persisted`) and loaded at the next start. Acks of state-file commands (keep-out, poses, collision setting, grasp setup) are sent when the write finished, so they can arrive after replies to later messages; their `epoch_id`, `safety_state` and `is_torqued` are read when the ack is sent.

禁入盒为基座坐标系下的轴对齐盒，工作空间检查点不得进入。在所有应用工作空间规则的位置检查（有禁入盒时，关节空间轨迹与 servo / 制动步按安全契约中的扫掠检查验证，每条轨迹最多追加 4000 次 FK，超出返回 `WORKSPACE_VIOLATION`；每个 servo 周期最多追加 64 次，超出则 servo 流停在可恢复的保持状态），进入时返回 `WORKSPACE_VIOLATION`，消息中带 `keepout '<name>'`；起点已在盒内且不再深入的运动允许。profile 字段 `workspace.keepout_boxes`；`GET /api/v1/workspace/keepout` 返回列表与来源；WS 指令 `/api/v1/workspace/keepout` 需要租约，替换运行时列表（profile 与运行时合计最多 16 个），写入 `ARMD_KEEPOUT`（默认 `/userdata/rebot-teleop/keepout.json`），下次启动时加载。状态文件类指令（禁入盒、姿态、碰撞设置、抓取设置）的应答在写入完成后发送，可能晚于后续消息的应答；其中的 `epoch_id`、`safety_state`、`is_torqued` 取发送时的值。生效的禁入盒为 profile 列表加运行时列表：运行时列表只能增加禁入盒，不能删除 profile 中的禁入盒；轨迹、宏动作、servo 流或收纳进行中（或机械臂仍在运动）时该指令返回 `409 MOTION_BUSY`，变更前规划的动作在安装时返回 `409 PLAN_STALE`；`GET` 返回的每个盒带 `source`（`profile` 或 `runtime`）。

## Observation rate, faults and residual torques / 观测频率、故障与残差力矩

- `/api/v1/ws?obs_hz=N` (integer >= 0, clamped to 1..`control_frequency_hz`, so 0 and 1 mean 1 Hz; anything else: `400` before the upgrade) sends at most N observations per second on that connection, always the newest. Lease, ack, reject, `lease_lost` and `epoch_sync` messages are not decimated. `action` keeps the latest record and `previous_action` stays 2 s, so N ≥ 1 does not lose a terminal action state.
- `fault`: null, or `{"code", "message", "recoverable"}` while `safety_state` is Fault or EStop (`safety_state` keeps its shape). Codes: `ESTOP`, `BUS_LOSS`, `THERMAL_TRIP`, `JOINT_LIMIT`, `WATCHDOG_TIMEOUT`, `FOLLOWING_ERROR`, `CONTROL_DEADLINE`, `INVALID_TELEMETRY`, `DRIVER_WRITE_FAILED`, `HARDWARE_FAULT`, `ENABLE_FAILED`, `SERVO_NO_BRAKE`, `HANDOVER_UNSETTLED`, `GRAVITY_COMP_FAILED`, `COLLISION`, `OTHER`. `recoverable` is true only for `WATCHDOG_TIMEOUT`, `CONTROL_DEADLINE`, `HANDOVER_UNSETTLED`; it is a hint for client policies, armd never clears a fault by itself. In Fault / EStop, when every motor reports fresh feedback with enable state Disabled (e.g. a motor power reset during `BUS_LOSS`), `is_torqued` becomes false and the fault stays until `clear_fault`.
- `residual_torques`: measured torque minus model gravity torque for J1..J6 (N·m), null without a gravity model or valid torque feedback (also null with the mock follower). The residual raises no fault itself; the collision guard below uses it.

`?obs_hz=N` 限制该连接每秒最多 N 条观测（取 1..`control_frequency_hz`），租约、应答、拒绝、`lease_lost`、`epoch_sync` 不降频。`fault` 在 Fault / EStop 时为 `{code, message, recoverable}`，否则为 null；`recoverable` 仅对 `WATCHDOG_TIMEOUT`、`CONTROL_DEADLINE`、`HANDOVER_UNSETTLED` 为 true，只是给客户端的提示，armd 不会自行清除故障。Fault / EStop 期间若所有电机反馈新鲜且使能状态为 Disabled（例如 `BUS_LOSS` 期间电机掉电复位），`is_torqued` 变为 false，故障保持到 `clear_fault`。`residual_torques` 为 J1..J6 实测力矩减模型重力力矩，无重力模型或无有效力矩反馈时为 null，本身不触发故障，碰撞保护（见下）使用它。

## Collision guard / 碰撞保护

Contact detection with a protective stop, built on `residual_torques` (capability `collision_guard`: profile gravity model and a driver that reports torque). It is contact detection, not a certified safety function, and the default thresholds are placeholders until measured on a real arm, so it is **off by default**; thresholds must come from rebot-sysid. Joints in `collision.rest_exempt_joints` (default: those of `resting_pose.support_torque_limits`; B601-DM: J2, J3, J4) are not watched at the resting pose. Full rules: `docs/control_safety_contract.md` "Collision guard".

- Profile section `collision`: `{"enabled": false, "thresholds_nm": [6.0,6.0,6.0,2.5,2.5,2.5], "persist_ticks": 4, "clear_ratio": 0.5, "resume_after_s": 2.0, "max_auto_resumes": 3, "escalate_window_s": 60}` (all optional).
- `GET /api/v1/safety/collision` (no lease): `{"available", "enabled", "sensitivity", "thresholds_nm" (scaled), "profile_thresholds_nm", "persist_ticks", "clear_ratio", "resume_after_s", "max_auto_resumes", "escalate_window_s", "source": "profile"|"runtime", "active"}`.
- WS command `/api/v1/safety/collision` `{"enabled": bool?, "sensitivity": "low"|"medium"|"high"?}` (lease; at least one field; unknown fields or values are `400 INVALID_COMMAND`; enabling while unavailable is `422 UNSUPPORTED`). Sensitivity scales the thresholds ×1.5 / ×1.0 / ×0.7. The ack carries `collision` (the config above) and `persisted`; the setting is written to `ARMD_COLLISION` (default `/userdata/rebot-teleop/collision.json`) and loaded at the next start. Switching the guard off ends a hold at once.
- Trip: |residual| above the threshold for `persist_ticks` ticks on any of J1..J6, with torque on, Armed / Moving and gravity compensation off. The running action ends `failed` with code `COLLISION` (message names the joint and residual); the arm holds its measured pose with torque on. `safety_state` does not change.
- While `collision.active`, motion (jog, home, presets, park, servo, every action) is rejected with `409 COLLISION_HOLD`; e_stop, stop, clear_fault and torque off work. Entering gravity compensation is allowed and ends the hold.
- The hold ends by itself after every residual stays below `clear_ratio` × threshold for `resume_after_s`. The (`max_auto_resumes`+1)-th trip within `escalate_window_s` latches Fault code `COLLISION` (`recoverable: false`), which needs `clear_fault`.
- Observation `collision`: null while disabled or unavailable, else `{"active": bool, "joint": int|null (1-based), "residual_nm": float|null, "threshold_nm": float|null, "since_s": float|null, "resume_in_s": float|null, "trips_in_window": int, "enabled": true, "sensitivity": "medium"}`.

Client pattern: on an action failing with `COLLISION`, wait until `collision.active` is false (raise if `fault.code` is `COLLISION`), then retry the step once; on escalation stop.

基于 `residual_torques` 的接触检测与保护性停止（能力 `collision_guard`：profile 有重力模型且驱动报告力矩）。它是接触检测，不是经过认证的安全功能；默认阈值在真机实测前为占位值，因此**默认关闭**；阈值须来自 rebot-sysid。`collision.rest_exempt_joints` 中的关节（缺省时取 `resting_pose.support_torque_limits` 的关节；B601-DM 为 J2、J3、J4）在收纳位姿不参与检测。完整规则见 `docs/control_safety_contract.md`“碰撞保护”。profile 段 `collision` 字段同上；`GET /api/v1/safety/collision`（无需租约）返回含缩放后阈值的生效配置；WS 指令 `/api/v1/safety/collision`（需租约）设置 `enabled` 与 `sensitivity`（low / medium / high，阈值乘以 1.5 / 1.0 / 0.7），写入 `ARMD_COLLISION`（默认 `/userdata/rebot-teleop/collision.json`），下次启动加载；关闭保护会立即结束保持。触发后正在执行的动作以 `COLLISION` 失败，机械臂在实测位置保持力矩，`safety_state` 不变；保持期间运动类指令返回 `409 COLLISION_HOLD`，急停、停止、`clear_fault`、断力矩仍可用；可以进入重力补偿，进入时保持结束。残差低于 `clear_ratio` × 阈值持续 `resume_after_s` 后自动恢复；`escalate_window_s` 内第 `max_auto_resumes`+1 次触发锁存 Fault `COLLISION`，须 `clear_fault`。观测字段 `collision` 结构见上。

## Torque guard / 力矩保护

A joint whose measured |torque| stays at or above `ratio` × its TMAX (`damiao_ranges[j].t_max`) for `persist_ms` stops the motion (capability `torque_guard`, capabilities version 8; profile section `torque_guard`, absent = off). B601-DM: `ratio` 0.60, `persist_ms` 100, J1–J6 watched: 16.8 N·m on J1–J3, 6.0 N·m on J4–J6; the gripper is not watched. 0.60 was set on 2026-11-12 for the 24 V 6.5 A supply in use; 0.85 is the value intended with a 14.6 A supply. Full rules: `docs/control_safety_contract.md` "Torque guard".

- `schema.torque_guard`: `{"ratio", "persist_ms", "thresholds_nm": [per joint, null = not watched], "release_rad"}`; `GET /api/v1/safety/describe` lists it as `torque_limit_hold`.
- Trip (torqued, Armed / Moving, position control; not in gravity compensation or the park settle phase): the running action (planned motion, Cartesian action, macro, replay, park) ends `failed` with code `TORQUE_LIMIT` (message names the joint, torque and threshold); every motion ends; J1–J6 hold the measured pose with torque on, the gripper keeps its command. `safety_state` does not change, no Fault.
- `motion_limit` = `{"reason": "torque", "joint": <0-based>, "since_ms"}` while the hold lasts.
- While held: servo frames and jog / joint_move / home targets that move the held joint further in the push direction (sign of the torque at the trip) are clamped to the hold position for that joint, the other joints follow; moves the other way pass. The hold ends when the joint is 0.05 rad from the hold position with |torque| below the threshold, or on torque off, E-stop, Fault or gravity-compensation entry.

Client pattern: on `TORQUE_LIMIT`, do not retry the same goal; move the named joint back (opposite to the direction it was moving) and re-plan.

关节实测 |力矩| ≥ `ratio` × 该关节 TMAX（`damiao_ranges[j].t_max`）持续 `persist_ms` 时停止运动（能力 `torque_guard`，能力版本 8；profile 段 `torque_guard`，缺省为关闭）。B601-DM：`ratio` 0.60、`persist_ms` 100，检测 J1–J6：J1–J3 16.8 N·m，J4–J6 6.0 N·m；夹爪不检测。0.60 于 2026-11-12 按当前 24 V 6.5 A 电源设定，配 14.6 A 电源时预期值为 0.85。完整规则见 `docs/control_safety_contract.md`「力矩保护」。`schema.torque_guard` 结构见上，`GET /api/v1/safety/describe` 中为 `torque_limit_hold`。触发（力矩开启、Armed / Moving、位置控制；重力补偿和收纳稳定阶段不检测）后正在执行的动作（规划运动、笛卡尔动作、宏动作、回放、收纳）以 `TORQUE_LIMIT` 失败，所有运动结束，J1–J6 在实测位置保持力矩，夹爪指令不变，`safety_state` 不变，不锁存 Fault；保持期间 `motion_limit` 为 `{"reason": "torque", "joint", "since_ms"}`。保持期间 servo 帧与 jog / joint_move / home 目标若使该关节继续朝施力方向（触发时力矩的符号）越过保持位置，该关节被限制在保持位置，其他关节照常；反方向运动照常通过。关节离开保持位置 0.05 rad 且 |力矩| 低于阈值，或断力矩、急停、Fault、进入重力补偿时保持结束。客户端做法：收到 `TORQUE_LIMIT` 不要重试同一目标，先把该关节反向移出再重新规划。

## Power budget / 电源功率预算

armd estimates the motor supply current from the measured joint torques and velocities with datasheet motor parameters (capability `power_budget`, capabilities version 9; profile section `power_budget`, absent = off). There is no current sensor: the values are estimates, not measurements. B601-DM: Mean Well LRS-150-24, 6.5 A rated, trip ~8.5 A. Full model, parameters and rules: `docs/control_safety_contract.md` "Power budget".

- `observation.power` (null without a budget):

  ```json
  "power": {"valid": true, "bus_a": 1.42, "peak_a": 2.10, "rated_a": 6.5, "ratio": 0.218, "regen_w": 0.0,
            "per_joint_a": [0.02, 0.41, 0.33, 0.12, 0.02, 0.02, 0.02], "scale": 1.0, "limiting": false, "hold": false}
  ```

  `bus_a` estimated supply current (A, 0.25 s low-pass), `peak_a` its 5 s peak, `ratio` = `bus_a / rated_a`, `regen_w` mechanical power fed back by the motors (not credited), `per_joint_a` per-joint share (idle power included, the base load is not), `scale` speed scale the governor applies to every motion this tick, `limiting` the governor slows or holds, `hold` a power hold is in force. `valid: false` (numbers 0) without fresh torque feedback.
- Governor: above 80 % of the rating motions slow down (`scale` < 1, `motion_limit.reason` `power`); above 95 % for 100 ms while a joint pushes in its torque direction the running action ends `failed` with code `POWER_LIMIT`, every motion ends and the measured pose is held with torque on (`power.hold` true, `motion_limit` `{"reason": "power", "joint"}`); no Fault. During the hold, servo / jog targets that move a pushing joint further in its push direction are clamped; yielding targets pass. The hold ends after the estimate stays below 80 % for 300 ms. Not applied in gravity compensation.
- Planning: every planned motion (jog / joint_move, home, move_to_pose, move_to, move_relative, grasp and place sub-motions, replay, park) is checked with URDF inverse dynamics before it starts; if its estimated peak exceeds 75 % of the rating it is slowed (duration / scale). The action record carries `power_scale` (1 = as planned, < 1 slowed; absent without a budget and for the gripper action) and `expected_duration_s` includes the slowing. If even 20 % speed does not fit, the command is rejected with `422 POWER_BUDGET`.
- `GET /api/v1/diagnostics/power` (no lease): `{"enabled", "estimate_only", "observation", "fast_a", "raw_a", "peak_5s_a", "peak_60s_a", "max_a", "max_at_s", "over_rated_s", "idle_a", "governor": {"scale", "limited_s", "holds_total", "hold"}, "budget"}`; `{"enabled": false}` without a budget. `schema.power_budget` is the budget in force; `GET /api/v1/safety/describe` lists the hold as `power_limit_hold`.

Client pattern: on `POWER_LIMIT`, wait for `power.hold` to clear, then retry slower or with less load; on `POWER_BUDGET`, the motion cannot fit the supply at any allowed speed (e.g. the target pose needs more holding current than the budget): choose another target or a larger supply.

armd 根据实测关节力矩、速度和数据手册电机参数估算电机电源电流（能力 `power_budget`，能力版本 9；profile 段 `power_budget`，缺省为关闭）。没有电流传感器：数值是估算值，不是测量值。B601-DM：明纬 LRS-150-24，额定 6.5 A，约 8.5 A 触发。完整模型、参数与规则见 `docs/control_safety_contract.md`「电源功率预算」。`observation.power` 结构见上：`bus_a` 为估算电源电流（A，0.25 s 低通），`peak_a` 为其 5 s 峰值，`ratio` = `bus_a / rated_a`，`regen_w` 为电机回馈的机械功率（不抵扣），`per_joint_a` 为各关节份额（含待机功率，不含基础负载），`scale` 为本周期调速器对所有运动施加的速度比例，`limiting` 表示正在降速或保持，`hold` 表示功率保持生效；无新鲜力矩反馈时 `valid: false`（数值为 0），无预算时为 null。调速：超过额定值 80 % 时运动降速（`scale` < 1，`motion_limit.reason` 为 `power`）；超过 95 % 持续 100 ms 且有关节沿其力矩方向施力时，正在执行的动作以 `POWER_LIMIT` 失败，所有运动结束，在实测位姿保持力矩（`power.hold` 为 true，`motion_limit` 为 `{"reason": "power", "joint"}`），不锁存 Fault；保持期间使施力关节继续朝施力方向运动的 servo / jog 目标被限制，顺从方向照常通过；估算值低于 80 % 持续 300 ms 后保持结束；重力补偿期间不调速。规划：所有规划运动（jog / joint_move、home、move_to_pose、move_to、move_relative、抓取与放置子运动、回放、收纳）开始前用 URDF 逆动力学检查，估算峰值超过额定值 75 % 时放慢（时长 / 比例）；动作记录带 `power_scale`（1 为原规划，< 1 为已放慢；无预算或夹爪动作时缺省），`expected_duration_s` 已包含放慢；20 % 速度仍不满足时以 `422 POWER_BUDGET` 拒绝。`GET /api/v1/diagnostics/power`（无需租约）字段见上；`schema.power_budget` 为生效的预算；`GET /api/v1/safety/describe` 中保持为 `power_limit_hold`。客户端做法：收到 `POWER_LIMIT` 时等 `power.hold` 清除后放慢或减小负载再试；收到 `POWER_BUDGET` 表示任何允许速度下都超出电源（例如目标位姿所需保持电流超过预算），应换目标或换更大的电源。

## Safety self-description / 安全自描述

`GET /api/v1/safety/describe` (no lease; capability `safety_describe`) lists this armd's timing, joint and Cartesian limits, keep-out boxes, speed cap, stops (`e_stop`, `watchdog` with the outcome for this config, `bus_loss`, `control_deadline`, `following_error`, `thermal`, and `collision_protective_stop` with `collision_guard`), every fault code with its recovery steps, and hardware notes. Values come from the running config and the code's constants. A client or AI assistant reads it before planning motion and keeps within those limits. Field list: `docs/control_safety_contract.md` "Safety self-description".

`GET /api/v1/safety/describe`（无需租约；能力 `safety_describe`）列出本 armd 的时序、关节与笛卡尔限值、禁入盒、速度上限、停止方式（`e_stop`、按本配置给出结果的 `watchdog`、`bus_loss`、`control_deadline`、`following_error`、`thermal`，有 `collision_guard` 时另有 `collision_protective_stop`）、每个故障码及其恢复步骤和硬件说明。数值取自运行中的配置与代码常量。客户端或 AI 助手在规划运动前读取，并在这些限值内运行。字段列表见 `docs/control_safety_contract.md`“安全自描述”。

## Grasp / 点框抓取

`/api/v1/action/grasp` picks up an object the user selected in the camera image. The request names a detection box (normalised `box_norm` from the detector, its `label`, optionally the frame); the action does the rest and ends with the object lifted, or with a failure code.

`/api/v1/action/grasp` 抓取用户在画面中选中的物体：请求给出检测框（归一化 `box_norm`、类别 `label`，可选帧号），动作完成后物体被抬起，或以失败码结束。

Needs a 7-DOF profile with `kinematics`, `gripper_policy` and `grasp` (the built-in `rebot-dm` has all three); otherwise `UNSUPPORTED`.

### Before the ack / 应答之前

Under the lock: lease, epoch, request_id, body, then the same preconditions as other actions (`MOTION_BUSY` etc.). Outside the lock: the box centre ray (setup intrinsics, current commanded pose, `camera_mount`) is intersected with the plane z = `table_z_m + grasp_height_m` to estimate the target; the approach pose above it (look-down, or camera-down, see [Camera-down acquisition](#camera-down-acquisition--相机朝下采集)) is planned with the gripper open, and its path (with the camera-down transition) is checked against the [table clearance](#table-clearance--桌面间隙). A planning failure is a `command_reject` with the planner's code (`IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION`, 422); a ray that does not meet the table in front of the camera, or `acquire_camera_down: "on"` without `camera_mount.offset_m`, is `INVALID_COMMAND`. Then the action begins and the ack carries `action_id` and a rough `expected_duration_s`.

锁内校验租约、epoch、request_id、请求体和动作前置条件；锁外用框中心射线与平面 z = `table_z_m + grasp_height_m` 求交估计目标位置，并以夹爪张开状态规划接近位姿（俯视；相机朝下采集时为光轴竖直），接近路径（相机朝下时含转换路径）按[桌面间隙](#table-clearance--桌面间隙)检查。规划失败直接 `command_reject`（`IK_UNREACHABLE` / `IK_BRANCH_FLIP` / `WORKSPACE_VIOLATION`，422）；射线不与桌面相交、或 `acquire_camera_down: "on"` 而 `camera_mount.offset_m` 未设置，返回 `INVALID_COMMAND`。

### Phases / 阶段

An async task outside the supervisor lock and the control tick runs the phases; `action.phase` and `action.progress` report them:

| phase | what happens |
|---|---|
| `open` | gripper `open` |
| `approach` | move_to above the estimate at `table_z_m + approach_height_m`, tool axis (end_link x) straight down, heading kept (minimal rotation). Camera-down: optical axis straight down instead, lens centre above the estimate (below) |
| `centring` | closed loop: wait for frames captured `settle_delay_s` after the last motion settled, match the target, take the median centre of `median_frames` frames; done when within `centre_tol_px` of the grip pixel (camera-down: of the image centre) for `stable_frames` consecutive frames; otherwise step = `servo_gain` × (table point of the box centre − table point of the grip pixel / image centre), at most `max_step_m`, as a base-frame move_relative. `iteration` = corrections made, `error_px` = latest error. More than `max_iterations` corrections → `CENTRING_FAILED` |
| `transition` | camera-down only: one Cartesian move to the tool-vertical pose above the target's table point at approach height; then a re-check frame (below) |
| `align` | when the box's long / short side ratio ≥ `align_min_aspect` and the jaws (end_link y in the image) would close across the long side: roll ±90° about the tool axis (the sign that keeps J6 inside its limits) |
| `descend` | straight down to `table_z_m + grasp_height_m` at `descend_speed_mps` |
| `close` | gripper `close` with grasp detection; `holding` per `gripper_policy` |
| `release` | not holding: gripper `open`, then `lift`, then `failed` / `NOT_HOLDING` (`holding:false`); no retry |
| `lift` | straight up to `table_z_m + lift_height_m` |
| `done` | `state:"done"`, `holding:true`, `gripper_gap_rad`; the jaw keeps its hold target in `commanded_positions[6]` |

由锁外异步任务按阶段执行，`action.phase` / `action.progress` 报告当前阶段：`open` → `approach` → `centring` →（仅相机朝下采集）`transition` → `align` → `descend` → `close` →（未抓住）`release` → `lift` → `done`。对中阶段只用运动停稳之后拍到的帧；停稳之前拍到的帧不延长 `lost_timeout_s`（从请求和停稳时刻中较晚者起算），超时为 `TARGET_LOST`；像素误差按粗略内参和 FK 已知高度换算为米，再乘保守增益，所以内参或安装角不准只会多走几步，不会发散。

Target tracking: same label, centre within `match_dist_px` of the predicted centre or IoU > `match_iou`; the first match after the approach uses the wider `acquire_dist_px` around the projection of the estimate. No match in `lost_frames` fresh frames or for `lost_timeout_s` → `TARGET_LOST`; frames captured before the motion settled do not extend `lost_timeout_s` (counted from the later of the request and the settle time). No frames at all for max(3 s, `lost_timeout_s`) → `TARGET_LOST`; a detector error → `DETECTION_UNAVAILABLE`. If inference was off, the grasp turns it on and turns it off again when it ends (any outcome).

Every motion is a sub-motion of the one grasp record: it is installed under the lock only while the grasp is still the running action of the same lease and epoch, and sub-motion results go to the orchestrator, not to `action`. A motion longer than its planned duration × 2 + 5 s (gripper: expected duration + 10 s), capped at `phase_timeout_s`,, or the whole grasp longer than `timeout_s`, stops the motion and ends as `GRASP_TIMEOUT`. Sub-motion failures (`TRACKING_STALL`, `GRIPPER_TIMEOUT`, planner codes) end the grasp with that code.

Ending a grasp from outside:

| cause | state / code |
|---|---|
| `/motion/stop` (during a motion or between motions) | `cancelled` / `STOPPED` |
| new jog / preset / home / park / action | `cancelled` / `SUPERSEDED` |
| lease released, preempted or connection closed | `cancelled` / `LEASE_LOST` |
| E-stop / Fault / lease watchdog | `failed` / `ESTOP_ACTIVE` / `FAULT` / `WATCHDOG` |

The record ends in the same tick; a sub-motion still carrying the grasp is stopped, and the orchestrator issues nothing more. Servo frames are rejected with `MOTION_BUSY` for the whole grasp, including the waits between motions.

外部停止（停止、急停、租约丢失、被新指令替代）在同一 tick 内结束动作记录，仍在执行的子运动被停止，编排任务不再下发任何运动。抓取期间（包括两次运动之间的等待）servo 帧一律返回 `MOTION_BUSY`。

### Table clearance / 桌面间隙

Every grasp sub-motion, look-down and camera-down alike (`approach`, each `centring` step, `transition`, `align`, `descend`, `lift`), keeps the envelope at least 5 mm above `table_z_m` along its path. The envelope is the TCP (jaw tips), every `workspace.check_points_end_m` point and, when `camera_mount.offset_m` or `camera_mount.body_size_m` is set, the lens centre (`offset_m`; the end_link origin when only `body_size_m` is set, as the grasp camera model assumes) and the 8 corners of the camera body box (`body_size_m` or its default, see [Camera-down acquisition](#camera-down-acquisition--相机朝下采集)). The general workspace check compares against `workspace.floor_z_m`, not the table, and counts the lens centre only. Paths are sampled as the planner interpolates them (5 mm / 1.1°).

- Before the ack: the approach path from the commanded pose and, camera-down, the transition from the planned acquisition pose. A violation is a `command_reject` `WORKSPACE_VIOLATION` (422).
- During the grasp: every sub-motion, from the commanded pose to its goal, before it is installed. A violation ends the grasp `failed` / `WORKSPACE_VIOLATION` in that phase with nothing installed; the message names the point index, its z and the bound (`table_z_m + 0.005`). The descent is checked here, not before the ack.
- A path that starts below the bound may move as long as no envelope point goes lower than its own start height, and every point that starts above the bound stays above it, so the lift from the grasp height passes. The rule is checked per point: a rotation that lowers one point while another started lower is still rejected.

The bound is the same for every point, the TCP included. `grasp_height_m` is validated ≥ 0.005, so the tool-vertical jaw tips at the grasp height always satisfy it; the default `grasp_height_m` 0.015 leaves 10 mm for points below the jaw-tip plane. With the tool vertical a point's height relative to the TCP is minus its end_link x (the heading and the J6 alignment do not change it), so the descent fails when a point lies more than `grasp_height_m − 0.005` ahead of the jaw tips along the tool axis, for example a lens or camera body that reaches past the jaws. The kit check points have end_link x ≤ 0; with the v8 `offset_m` [−0.0731, 0.0005, 0.066] and the default body box the camera's foremost point is 58.6 mm behind the jaw-tip plane, so default v5 and v8 grasps are unchanged. Covered by `cargo test --test test_grasp look_down_descent` (lens 9 mm ahead: done; 14.5 mm ahead: `WORKSPACE_VIOLATION` in `descend`, TCP still at approach height).

抓取的每个子运动（俯视与相机朝下两种方式下的 `approach`、每个 `centring` 步、`transition`、`align`、`descend`、`lift`）沿整条路径都使包络点高于 `table_z_m` 至少 5 mm。包络点为 TCP（夹爪尖）、所有 `workspace.check_points_end_m` 点，以及在设置了 `camera_mount.offset_m` 或 `camera_mount.body_size_m` 时的镜头中心（取 `offset_m`；只设置 `body_size_m` 时按抓取相机模型取 end_link 原点）和相机机身方盒的 8 个角点（`body_size_m` 或其默认值）。通用工作空间检查比较的是 `workspace.floor_z_m` 而不是桌面，且只计镜头中心。路径按规划器的插值方式采样（5 mm / 1.1°）。应答前检查接近路径（相机朝下时还检查从规划的采集位姿出发的转换路径），违反时为 `command_reject` `WORKSPACE_VIOLATION`（422）；执行中每个子运动在下发前从当前指令位姿到目标再检查一次，违反时抓取以 `failed` / `WORKSPACE_VIOLATION` 结束于该阶段，不下发该运动，消息给出点序号、其 z 值和下限（`table_z_m + 0.005`）；下降只在此时检查，不在应答前检查。起点已低于下限的路径只要没有点变得更低即可执行，因此从抓取高度抬起总能通过。所有点（含 TCP）使用同一下限。`grasp_height_m` 校验为 ≥ 0.005，因此工具竖直时抓取高度处的夹爪尖总满足下限；默认 `grasp_height_m` 0.015 给夹爪尖平面以下的点留 10 mm。工具竖直时某点相对 TCP 的高度等于其 end_link x 的相反数（朝向和 J6 对齐不改变它），所以沿工具轴伸出夹爪尖超过 `grasp_height_m − 0.005` 的点（如伸出夹爪的镜头或机身）会使下降失败。套件检查点的 end_link x ≤ 0；v8 的 `offset_m` [−0.0731, 0.0005, 0.066] 加默认机身方盒时，相机最前端在夹爪尖平面后方 58.6 mm，因此默认的 v5、v8 抓取不变。测试：`cargo test --test test_grasp look_down_descent`（镜头伸出 9 mm：完成；伸出 14.5 mm：`descend` 阶段 `WORKSPACE_VIOLATION`，TCP 仍在接近高度）。

### Camera-down acquisition / 相机朝下采集

For a camera mount pitched toward the jaws (v8 holder: `camera_mount` `rot_deg` 0, `tilt_deg` −38), the table point under the jaw tips is outside the image at approach height when the tool points straight down, so the look-down centring above cannot run. The setup field `acquire_camera_down` selects the acquisition:

| value | acquisition |
|---|---|
| `auto` (default) | camera-down when \|`camera_tilt_deg`\| > 5° and `camera_mount.offset_m` is set; look-down otherwise |
| `on` | camera-down; without `camera_mount.offset_m` the request is `INVALID_COMMAND` |
| `off` | look-down |

Camera-down phases:

- `approach`: orientation = the look-down orientation rotated minimally so the optical axis (camera z) points straight down, i.e. the tool pitched by the mount tilt about x_cam, heading kept. Position: lens centre (FK + `camera_mount.offset_m`) above the estimate in x, y; TCP at `table_z_m + approach_height_m`.
- `centring`: the loop above with the image centre (`image_size_px` / 2) as the aim point instead of `grip_pixel_norm`; gains, limits, `stable_frames`, `max_iterations` unchanged.
- `transition`: the ray through the last matched box centre (within `centre_tol_px` of the image centre) is intersected with the plane z = `table_z_m + grasp_height_m`; one planned Cartesian move (IK, workspace and keep-out checks as every motion) goes to the tool-vertical pose with the TCP above that point at approach height. Then, when the camera model projects the target into the image at the new pose, one fresh frame verifies it before the descent: no match (`lost_frames` / `lost_timeout_s`, gate `acquire_dist_px` around the projection) → `TARGET_LOST`; a detector error → `DETECTION_UNAVAILABLE`; box centre more than `recheck_tol_px` from the projection → `TARGET_DISPLACED`. `final_offset_px` = [dx, dy] of the matched box centre minus `grip_pixel_norm` in pixels (a diagnostic; the displacement check uses the projection, so an uncalibrated grip pixel does not fail it). When the target is not expected in view (for example the v8 holder at a 0.15 m approach) nothing is checked and `final_offset_px` is absent.
- `align`, `descend`, `close`, `lift`: as above; J6 alignment uses the last box of the centring.

Table clearance at the camera-down acquisition (the rule for every phase is in [Table clearance](#table-clearance--桌面间隙)): the pitched gripper reaches below the TCP (with the kit check points up to 57 mm at a 38° pitch about the end_link z axis, 21 mm about the y axis), and the workspace check compares against `workspace.floor_z_m`, not the table. The acquisition therefore raises the TCP above `table_z_m + approach_height_m` when needed so that the TCP, every `workspace.check_points_end_m` point, the lens centre and the 8 corners of the camera body box stay at least 5 mm above `table_z_m` at the camera-down orientation and along the transition (the raised height is `action.acquire_height_m`, above the table; absent when not raised). The camera body box is `camera_mount.body_size_m` [width along x_cam, height along y_cam, depth behind the lens along −z_cam] (m, optional, each in (0, 0.2]); the default 0.055 x 0.047 x 0.040 is the reCamera housing (40 x 40 x 36.5 mm) in the v8 holder (walls, ears, support, top bar and rear lip from the holder's generator), lens at the centre of the front face. It is used for the grasp table clearance only, not in the general workspace check, so other motions are unchanged; the general check still counts the lens centre only. With the kit check points and the v8 mount the gripper points, not the body, set the raised height; a mount whose body reaches lower than the gripper is rejected or raised by the same rule. The approach path, each centring step and the transition path are checked against the same bound as every grasp sub-motion.

The accuracy of the transition depends on `camera_tilt_deg`, `camera_mount.rot_deg` and `offset_m`: an error in them moves the grasp point (in the simulated-camera test, a 2° tilt, 1° rot, 5 % focal length and (6, −4) px principal point error gave a 9 mm grasp point error at a 0.15 m approach). `expected_duration_s` adds the transition move and 1.5 s for the re-check frame. With \|`camera_tilt_deg`\| ≤ 5° (v5 mount) `auto` keeps the look-down acquisition unchanged. Validated in simulation only (`cargo test --test test_grasp camera_down`, `sim/mujoco/tests/test_camera_grasp.py` v8 cases); not run on the arm.

相机安装向夹爪方向俯仰时（v8 支架：`camera_mount` `rot_deg` 0、`tilt_deg` −38），工具竖直朝下时夹爪尖正下方的桌面点在接近高度处不在画面内，上面的俯视对中无法进行。抓取设置字段 `acquire_camera_down` 选择采集方式：`auto`（默认）在 \|`camera_tilt_deg`\| > 5° 且 `camera_mount.offset_m` 已设置时用相机朝下采集，否则俯视；`on` 始终相机朝下，`camera_mount.offset_m` 未设置时请求返回 `INVALID_COMMAND`；`off` 始终俯视。

相机朝下采集的阶段：`approach` 将俯视姿态做最小旋转使光轴竖直向下（工具绕 x_cam 俯仰安装倾角，朝向不变），镜头中心（FK + `camera_mount.offset_m`）在 x、y 上位于估计点正上方，TCP 高度为 `table_z_m + approach_height_m`；`centring` 与上面的闭环相同，目标点改为画面中心（`image_size_px` / 2），增益、步长上限、`stable_frames`、`max_iterations` 不变；`transition` 用最后一次匹配框中心（距画面中心不超过 `centre_tol_px`）的射线与平面 z = `table_z_m + grasp_height_m` 求交，一次规划的笛卡尔运动（与其他运动一样做 IK、工作空间和禁入区检查）移到 TCP 位于该点上方接近高度、工具竖直的位姿；之后若相机模型预测目标在新位姿的画面内，下降前取一帧新图复查：未匹配到（`lost_frames` / `lost_timeout_s`，以投影为中心、门限 `acquire_dist_px`）为 `TARGET_LOST`，检测器出错为 `DETECTION_UNAVAILABLE`，框中心距投影超过 `recheck_tol_px` 为 `TARGET_DISPLACED`；`final_offset_px` = 匹配框中心减 `grip_pixel_norm` 的像素差 [dx, dy]（诊断用；位移判断以投影为准，抓取像素未标定不会导致失败）；预测目标不在画面内时（如 v8 支架、0.15 m 接近高度）不做复查，`final_offset_px` 缺省；`align`、`descend`、`close`、`lift` 不变，J6 对齐使用对中阶段最后一个框。相机朝下采集时的桌面间隙（所有阶段的规则见[桌面间隙](#table-clearance--桌面间隙)）：俯仰后的夹爪会低于 TCP（套件检查点在绕 end_link z 轴俯仰 38° 时最多低 57 mm，绕 y 轴时 21 mm），而工作空间检查比较的是 `workspace.floor_z_m`，不是桌面。因此采集时在需要时把 TCP 抬高到 `table_z_m + approach_height_m` 以上，使 TCP、所有 `workspace.check_points_end_m` 点、镜头中心和相机机身方盒的 8 个角点在相机朝下姿态及整个转换过程中都高于 `table_z_m` 至少 5 mm（抬高后的高度见 `action.acquire_height_m`，相对桌面；未抬高时缺省）。相机机身方盒为 `camera_mount.body_size_m` [沿 x_cam 宽、沿 y_cam 高、沿 −z_cam 镜头后方深度]（米，可选，每项 (0, 0.2]），默认 0.055 x 0.047 x 0.040，即 reCamera 外壳（40 x 40 x 36.5 mm）装在 v8 支架内（壁厚、耳、底托、顶杆和后唇取自支架生成脚本），镜头位于前面中心。它只用于抓取的桌面间隙，不进入通用工作空间检查，因此其他运动不变；通用检查仍只计镜头中心。使用套件检查点和 v8 支架时，决定抬高量的是夹爪检查点而不是机身；机身比夹爪更低的安装按同一规则抬高或拒绝。接近路径、每个对中步和转换路径与抓取的其他子运动按同一下限检查。转换的精度取决于 `camera_tilt_deg`、`camera_mount.rot_deg` 和 `offset_m`（模拟相机测试中倾角差 2°、旋转差 1°、焦距差 5 %、主点偏 (6, −4) px，在 0.15 m 接近高度下抓取点偏差 9 mm）。`expected_duration_s` 增加转换运动时长和 1.5 s 复查帧。\|`camera_tilt_deg`\| ≤ 5°（v5 支架）时 `auto` 保持原俯视流程不变。仅在仿真中验证（`cargo test --test test_grasp camera_down`、`sim/mujoco/tests/test_camera_grasp.py` 的 v8 用例），未在真机上运行。

Example / 示例:

```json
{"type":"command", "request_id":51, "generation":7, "epoch_id":3, "path":"/api/v1/action/grasp",
 "body":{"label":"cup", "box_norm":[0.31,0.39,0.47,0.72], "ts_ms":1762650000000, "tag":"pick",
         "overrides":{"approach_height_m":0.17}}}
```

```json
"action": {"id":9, "kind":"grasp", "tag":"pick", "state":"running", "phase":"centring", "iteration":2, "error_px":14.3,
           "progress":0.3, "code":null, "holding":null, ...}
```

### Grasp setup / 抓取设置

`ArmConfig.grasp` holds the defaults; the runtime setup is read with `GET /api/v1/grasp/setup` (`{"supported":bool, "setup":{...}, "path":str|null}`) and changed with the WS command `/api/v1/grasp/setup` (body = fields to change; unknown fields, non-finite or out-of-range values → `INVALID_COMMAND`). `{"table_z_from_tcp": true}` sets `table_z_m` to the current commanded end_link height (jog the jaw tips onto the table first). The ack carries the new `setup` and `persisted: true|false` (`persist_error` on failure). Every field can also be overridden for one grasp with `overrides`.

Persistence: `ARMD_GRASP_SETUP` (default `/userdata/rebot-teleop/grasp_setup.json`), written atomically (temp file + rename, directory created). At startup a missing file keeps the defaults; an unreadable, malformed or invalid file is logged and ignored (defaults kept, file untouched); a partial file overrides only its fields.

`camera_tilt_deg` is `camera_mount.tilt_deg`: setting it changes the mount used by camera-frame moves and published in the schema. A change of `camera_tilt_deg` while a trajectory, macro action, servo stream or park runs is `409 MOTION_BUSY` (the camera point is part of the workspace check); other fields can change at any time. For the default mount (`flip:false`) a negative tilt turns the optical axis toward the jaws (image down).

设置默认值来自 `ArmConfig.grasp`；运行时用 `GET /api/v1/grasp/setup` 读取，用 WS 指令 `/api/v1/grasp/setup` 修改（`table_z_from_tcp: true` 以当前夹爪尖高度作为桌面高度），保存到 `ARMD_GRASP_SETUP`（默认 `/userdata/rebot-teleop/grasp_setup.json`）。文件缺失用默认值；文件损坏或数值无效时记日志并忽略。`camera_tilt_deg` 即 `camera_mount.tilt_deg`；轨迹、宏动作、servo 流或收纳进行中修改 `camera_tilt_deg` 返回 `409 MOTION_BUSY`（相机点属于工作空间检查），其他字段随时可改；默认安装（`flip:false`）下负值表示光轴朝夹爪方向（画面下方）倾斜。

| field | default | meaning |
|---|---|---|
| `table_z_m` | 0.0 | table surface, base frame (m) |
| `approach_height_m` / `grasp_height_m` / `lift_height_m` | 0.15 / 0.015 / 0.12 | TCP (end_link origin = jaw tips) above the table (m) |
| `grip_pixel_norm` | [0.5, 0.5] | where the table point under the jaw tips appears at approach height, looking down (camera-down: only for `final_offset_px`) |
| `camera_tilt_deg` | `camera_mount.tilt_deg` | camera tilt (deg) |
| `image_size_px`, `fx_px`, `fy_px` | [640, 360], 600, 600 | rough intrinsics for pixel → metric steps (placeholders) |
| `centre_tol_px`, `stable_frames`, `median_frames`, `max_iterations` | 10, 3, 3, 10 | centring done / give up |
| `servo_gain`, `max_step_m` | 0.6, 0.03 | step = gain × estimated error, at most max_step |
| `match_dist_px`, `match_iou`, `acquire_dist_px` | 35, 0.45, 160 | target matching gates |
| `lost_timeout_s`, `lost_frames`, `settle_delay_s` | 1.0, 5, 0.1 | target loss and frame freshness |
| `align_j6`, `align_min_aspect` | true, 1.2 | J6 alignment |
| `approach_speed_mps`, `descend_speed_mps` | 0.05, 0.03 | linear speeds |
| `phase_timeout_s`, `timeout_s` | 25, 120 | timeouts |
| `recheck_tol_px` | 40 | camera-down re-check: largest distance of the target from its projection after the transition |
| `acquire_camera_down` | `"auto"` | `"auto"`, `"on"` or `"off"`: [camera-down acquisition](#camera-down-acquisition--相机朝下采集); other values → `INVALID_COMMAND` |

The cockpit (`src/api/index.html`) uses this API: with Detect on, tap a box, confirm "Grasp"; the "Grasp setup" HUD button opens the setup drawer. After any `gripper` or `grasp` action the cockpit takes its gripper target from `commanded_positions[6]`, so taking over by teleop does not move the jaw.

驾驶舱：开启「识别」后点框并确认「抓取」；HUD 上的「抓取设置」打开设置抽屉。任何 `gripper` / `grasp` 动作之后，驾驶舱从 `commanded_positions[6]` 同步夹爪目标，遥操作接管时夹爪不会被拉回原开度。

### Real-arm calibration (v8 mount) / 真机标定（v8 支架）

The defaults below are simulation values. Measure them on the v8 mount before relying on them:

| value | default | how to measure |
|---|---|---|
| `camera_mount.offset_m` | none (sim tests: [−0.0731, 0.0005, 0.066]) | lens centre in end_link (m): measure from the jaw tips (end_link origin) with a ruler or take it from the CAD of the arm and the printed holder; check by jogging so the lens is above a table mark and comparing FK + offset with the mark |
| `camera_mount.body_size_m` | 0.055 x 0.047 x 0.040 | measure the printed holder with the camera installed (width along x_cam, height along y_cam, depth behind the lens) and confirm the lens sits at the centre of the front face; set the field when the box does not contain the holder |
| table margin | 5 mm (`TABLE_CLEARANCE_M`, `src/grasp.rs`) | set `table_z_m` with `table_z_from_tcp` at several points of the work area; the margin must exceed the spread of those readings plus the TCP height error at the grasp height. The margin is a constant: a larger one needs a code change |
| `recheck_tol_px` | 40 | hold the arm still over a static target, record the box centre of 100 consecutive frames, take the jitter (largest distance from the mean centre); set `recheck_tol_px` ≥ 3 × that jitter + the model error (distance between the projected and the detected centre after a transition with the target in place) |

下表默认值来自仿真，在 v8 支架上使用前须实测：`camera_mount.offset_m`（默认未设置，仿真测试用 [−0.0731, 0.0005, 0.066]）为 end_link 坐标系下的镜头中心（米），从夹爪尖（end_link 原点）用尺量或取自机械臂与打印支架的 CAD，再点动使镜头位于桌面标记上方、比较 FK + 偏移与标记位置来验证；`camera_mount.body_size_m`（默认 0.055 x 0.047 x 0.040）量装好相机的打印支架（沿 x_cam 宽、沿 y_cam 高、镜头后方深度），确认镜头位于前面中心，方盒包不住支架时设置该字段；桌面余量 5 mm（`src/grasp.rs` 的 `TABLE_CLEARANCE_M`）须大于在工作区内多点用 `table_z_from_tcp` 测得的 `table_z_m` 差值加抓取高度处的 TCP 高度误差，余量是常量，改大需改代码；`recheck_tol_px`（默认 40）：机械臂静止对准静止目标，记录连续 100 帧的框中心，取抖动（到平均中心的最大距离），设 `recheck_tol_px` ≥ 3 × 抖动 + 模型误差（目标不动时转换后投影中心与检测中心的距离）。

## Place / 放置

`/api/v1/action/place` puts a held object down at a named pose or at a base-frame position. Needs a 7-DOF profile with `kinematics` and `gripper_policy` (`capabilities.place`); otherwise `UNSUPPORTED`. No camera or grasp setup is involved.

`/api/v1/action/place` 把夹持的物体放到命名姿态或基座坐标系下的指定位置。需要 7 轴且设置了 `kinematics` 和 `gripper_policy`（`capabilities.place`），否则返回 `UNSUPPORTED`。不使用相机和抓取设置。

| field | default | meaning |
|---|---|---|
| `pose` | — | named pose; its stored TCP (`tcp.position`, `tcp.rpy`) is the place point and orientation. A pose saved without a TCP uses FK of its positions |
| `position` | — | place point `[x,y,z]` (end_link origin, base frame, m). Exactly one of `pose` / `position` |
| `orientation` | null | `{"rpy":[r,p,y]}` or `{"quat":[w,x,y,z]}`; null = the pose's orientation (`pose`) or the current one (`position`) |
| `approach_height_m` | 0.08 | 0.02..0.30; height above the place point for the approach and the retreat |
| `release` | true | open the gripper at the place point |
| `retreat` | true | move straight up by `approach_height_m` after the release |
| `speed_scale` | 1.0 | 0.1..1.0; every phase runs at 0.05 m/s × scale linear and 0.5 rad/s × scale angular (the `move_to` defaults), then the profile `speed_cap` |
| `tag` | null | string ≤ 64 bytes |

Phases (`action.phase`, `action.progress`):

| phase | progress | what happens |
|---|---|---|
| `lift` | 0.05 | straight up, orientation kept, to max(current z, target z + `approach_height_m`); no motion when the TCP is already within 1 mm of that height |
| `approach` | 0.2 | straight line to target + `approach_height_m` in z, rotating to the place orientation |
| `descend` | 0.55 | straight down to the place point |
| `open` | 0.75 | gripper `open` (only with `release`) |
| `retreat` | 0.85 | straight up by `approach_height_m` (only with `retreat`) |
| `done` | 1.0 | `state:"done"`; `holding:false` after a release, null otherwise |

Before the ack, outside the lock, lift, approach, descend and retreat are planned in sequence from the commanded state (each from the end of the previous one, retreat with the jaws open when `release`). A failure is a `command_reject` with the planner code (`IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION` incl. keep-out boxes with `keepout '<name>'` in the message, 422); an unknown `pose` is `POSE_NOT_FOUND` (404); bad fields are `INVALID_COMMAND`. Each phase is planned again from the commanded state when it runs. Place does not use the grasp setup's `table_z_m` or the [table clearance](#table-clearance--桌面间隙): its paths are checked against `workspace.floor_z_m` and the keep-out boxes only, so a place point (or a camera reaching below it) at the table is not rejected.

Execution uses the same orchestration as `grasp` (`src/macro_action.rs`): every motion is a sub-motion of the one `place` record, installed under the lock only while the action is still running, and the cancellation table of [Grasp](#phases--阶段) applies unchanged (`/motion/stop` → `cancelled` / `STOPPED`, new motion → `SUPERSEDED`, lease → `LEASE_LOST`, E-stop / Fault / watchdog → `failed`). Servo frames are rejected with `MOTION_BUSY` for the whole action. A sub-motion longer than its planned duration × 2 + 5 s (gripper: expected duration + 10 s), capped at 60 s, or the whole action longer than 2 × `expected_duration_s` + 20 s, stops the motion and ends as `PLACE_TIMEOUT`. Sub-motion failures (`TRACKING_STALL`, `GRIPPER_TIMEOUT`, planner codes) end the action with that code.

应答前在锁外按顺序规划抬升、接近、下降和撤离（每段从上一段终点开始；`release` 时撤离段按夹爪张开规划）。规划失败直接 `command_reject`（`IK_UNREACHABLE` / `IK_BRANCH_FLIP` / `WORKSPACE_VIOLATION`，含禁入盒，消息中带 `keepout '<name>'`，422）；`pose` 不存在返回 `POSE_NOT_FOUND`（404）；参数错误返回 `INVALID_COMMAND`。执行时每个阶段从当前指令位置重新规划。放置不使用抓取设置的 `table_z_m`，也不做[桌面间隙](#table-clearance--桌面间隙)检查：路径只按 `workspace.floor_z_m` 和禁入盒检查，放置点（或低于放置点的相机）贴近桌面时不会被拒绝。

阶段：`lift` 沿竖直方向抬升到 max(当前 z, 目标 z + `approach_height_m`)，已在该高度 1 mm 内则不运动；`approach` 直线移动到目标上方 `approach_height_m` 处并转到放置姿态（指定姿态、命名姿态的姿态或当前姿态）；`descend` 竖直下降到放置点；`open` 张开夹爪（仅 `release`）；`retreat` 竖直上升 `approach_height_m`（仅 `retreat`）；`done`。速度为 `move_to` 默认值（0.05 m/s、0.5 rad/s）乘以 `speed_scale`，再受 profile `speed_cap` 限制。执行与 `grasp` 共用编排模块（`src/macro_action.rs`），取消规则相同：`/motion/stop` 为 `cancelled` / `STOPPED`，新运动为 `SUPERSEDED`，租约丢失为 `LEASE_LOST`，急停 / 故障 / 看门狗为 `failed`。单个子运动超过规划时长 × 2 + 5 s（夹爪为预计时长 + 10 s，上限 60 s），或整个动作超过 2 × `expected_duration_s` + 20 s，停止运动并以 `PLACE_TIMEOUT` 结束。`release:false` 时夹爪保持原指令位置，`holding` 为 null。

Example / 示例:

```json
{"type":"command", "request_id":52, "generation":7, "epoch_id":3, "path":"/api/v1/action/place",
 "body":{"pose":"bin_a", "approach_height_m":0.06, "tag":"drop"}}
```

```json
"action": {"id":10, "kind":"place", "tag":"drop", "state":"running", "phase":"descend", "progress":0.55,
           "code":null, "holding":null, "iteration":null, "error_px":null, ...}
```

## Replay / 回放

Replays a saved episode (see the README, "Episode recording"). Episode files are managed over REST without a lease, like the other `/api/v1/dataset/*` routes; they move nothing. Browser requests must be same-origin (`DELETE` included).

| Route | Returns |
|---|---|
| `GET /api/v1/dataset/episodes` | `{"enabled", "dir", "episodes":[{"id", "frames", "duration_s", "size_bytes", "created_ms", "dof", "joint_min":[dof], "joint_max":[dof], "error"?}], "total_bytes", "max_bytes", "max_files"}`, newest first. `created_ms` is the file modification time (Unix ms). `error` is set for a file that cannot be read. |
| `DELETE /api/v1/dataset/episodes/{id}` | `{"status":"deleted", "episode_id"}`; `404 EPISODE_NOT_FOUND`, `400 INVALID_EPISODE_ID` |

WS command (lease, epoch, `request_id` as for every action):

```json
{"type":"command", "request_id":9, "generation":1, "epoch_id":3,
 "path":"/api/v1/action/replay", "body":{"episode":"teleop_20261112_101500", "speed":0.5, "gripper":true, "tag":"demo"}}
```

- `episode` (required): episode id, `^[A-Za-z0-9_-]{1,64}$`. `speed`: `0.5` (default) or `1.0`, a fraction of the recorded speed. `gripper` (default true): replay J7; false holds J7 at its current command. `tag` as for other actions.
- The ack carries `action_id` and `expected_duration_s` (approach + recorded duration / speed + 0.5 s).
- The episode file is read and checked on a blocking thread without the supervisor lock. Then, under the lock, epoch, workspace revision, busy rules and the commanded positions (< 1e-4 rad) are checked again (`PLAN_STALE` / the busy codes), the approach is planned and the action starts.

Checks before any motion (rejection, no motion):

| Check | Code (status) |
|---|---|
| fewer than 2 frames, or no time span | `REPLAY_EMPTY` (422) |
| a frame's `observation.state` length differs from the profile's joint count | `REPLAY_JOINT_COUNT` (422) |
| non-finite angle, timestamps not finite and non-decreasing, unreadable file | `REPLAY_INVALID_EPISODE` (422) |
| recorded duration over 600 s | `REPLAY_TOO_LONG` (422) |
| any frame outside the joint soft limits (J7 only with `gripper`) | `SOFT_LIMIT_EXCEEDED` (400) |
| per-joint speed of the resampled recording × `speed` above the planned-motion limit (`planning_limits()`: `planned_max_velocity` × `speed_cap.joint_velocity_scale`, 0.45 rad/s on rebot-dm; the J7 joint limit for the gripper). The message names the joint, the speed, the time and the highest playback speed that passes. The recording is not clipped. | `REPLAY_TOO_FAST` (422) |
| any resampled frame against the workspace rule of planned motions (baseline = the commanded pose at the request), and the keep-out sweep over the frames (at most 4000 added FK evaluations) | `WORKSPACE_VIOLATION` (422) |
| file missing; storage off | `EPISODE_NOT_FOUND` (404); `EPISODES_DISABLED` (409) |
| a recording is running or saving; gravity compensation; another trajectory, servo stream or motion | `MOTION_BUSY` (409) |
| collision hold; E-stop; fault; not armed | `COLLISION_HOLD` (409); `ESTOP_ACTIVE`; `FAULT_ACTIVE`; `NOT_ARMED` |

Execution:

1. `observation.state` is resampled by timestamp to the control rate (`control_frequency_hz`, linear between frames).
2. Approach: a joint-space quintic from the commanded pose to frame 0 at the planned-motion limits, checked like a jog (joint limits, 21 workspace samples, keep-out sweep).
3. Replay: each tick advances the recording time by dt × `speed` × rate, with the lag-aware rate of planned paths (full speed below 0.38 rad lag on J1–J6, stop at 0.45 rad). A rate at or below 0.1 for 1 s ends the action `failed` / `TRACKING_STALL` with commanded = measured (no Fault).
4. Settle at the last frame, as for other actions (`settled`).

Gripper with `gripper: true`: J7 follows the recording; while the stall latch holds the jaw (|τ7| high, jaw still), a recorded command further into the stall is held and a command in the opposite direction releases the latch (the servo / jog rule).

Action record: `kind` `replay`, `episode`, `phase` (`approach` → `replay` → `settle`), `t_s` (recording time reached, s; 0 during the approach), `progress` (`t_s` / recorded duration). Stop, E-stop, collision trip, fault, watchdog and lease loss end it with the codes of the table in [Action status](#action-status--动作状态); on lease loss (release, preemption, closed connection) armd also stops the replay reference, so the arm brakes and holds.

保存的录制片段可回放（录制见 README「示教片段录制」）。片段文件通过 REST 管理，与其他 `/api/v1/dataset/*` 路由一样无需租约（不产生运动）；浏览器请求（含 `DELETE`）须同源。`GET /api/v1/dataset/episodes` 按时间倒序返回片段（帧数、时长、大小、保存时间、各关节范围；无法读取的文件带 `error`）；`DELETE /api/v1/dataset/episodes/{id}` 删除片段，不存在返回 `404 EPISODE_NOT_FOUND`，id 不合法返回 `400 INVALID_EPISODE_ID`。

WS 指令 `/api/v1/action/replay`（需租约与 epoch）：`episode` 必填（`^[A-Za-z0-9_-]{1,64}$`）；`speed` 为 `0.5`（默认）或 `1.0`，相对录制速度；`gripper`（默认 true）回放 J7，false 时 J7 保持当前指令。ack 带 `action_id` 与 `expected_duration_s`。片段在阻塞线程中读取与检查，不持有监督器锁；安装前在锁内再次检查 epoch、工作空间版本、忙碌条件和指令位置（`PLAN_STALE` 或对应忙碌码）。

运动前的检查（不通过则拒绝，不运动）：帧数少于 2 或没有时长 `REPLAY_EMPTY`；关节数与 profile 不符 `REPLAY_JOINT_COUNT`；非有限值、时间戳非有限或递减、文件无法读取 `REPLAY_INVALID_EPISODE`；时长超过 600 s `REPLAY_TOO_LONG`；任一帧超出关节软限位（J7 仅在 `gripper` 时检查）`SOFT_LIMIT_EXCEEDED`；重采样后各关节速度 × `speed` 超过规划运动限速（`planning_limits()`，rebot-dm 为 0.45 rad/s；夹爪用 J7 关节限位）`REPLAY_TOO_FAST`，消息给出关节、速度、时刻与可通过的最高回放速度，不对录制做削峰；每个重采样帧按规划运动的工作空间规则检查（基准为请求时的指令位姿），并对禁入盒做扫掠检查（最多追加 4000 次 FK）`WORKSPACE_VIOLATION`；文件不存在 `EPISODE_NOT_FOUND`；存储关闭 `EPISODES_DISABLED`；录制进行或保存中、重力补偿中、其他轨迹 / servo 流 / 运动进行中 `MOTION_BUSY`；碰撞保护中 `COLLISION_HOLD`；急停、故障、未使能为对应错误码。

执行：按时间戳把 `observation.state` 线性重采样到控制频率；先以规划运动限速用关节空间五次多项式移动到第 0 帧（与 jog 相同的检查）；之后每个周期按 dt × `speed` × rate 推进录制时间，rate 与规划路径相同（J1–J6 滞后低于 0.38 rad 全速，0.45 rad 停止），rate ≤ 0.1 持续 1 s 结束为 `failed` / `TRACKING_STALL`，指令设为实测位置，不进入 Fault；最后在末帧做到位确认。`gripper: true` 时 J7 跟随录制；夹爪卡滞锁存期间，继续向卡滞方向的指令保持不动，反向指令解除锁存（与 servo / jog 规则相同）。动作记录：`kind` 为 `replay`，带 `episode`、`phase`（`approach` → `replay` → `settle`）、`t_s`（已回放到的录制时间，接近阶段为 0）与 `progress`。停止、急停、碰撞、故障、看门狗和租约丢失按“动作状态”一节的错误码结束；租约丢失（释放、被接管、连接关闭）时 armd 同时停止回放参考轨迹，机械臂制动并保持。

## Gravity compensation mode / 重力补偿模式

Verified on the reBot B601-DM (see the README). Full contract: [control_safety_contract.md](control_safety_contract.md#gravity-compensation-hand-guiding--重力补偿拖动示教).

```json
{"type":"command","request_id":7,"generation":1,"epoch_id":42,"path":"/api/v1/mode/gravity_comp","body":{"enable":true,"scale":1.0,"damping_kd":0.5}}
{"type":"command_ack","request_id":7,"status":"accepted","control_mode":"gravity_comp", ...}
{"type":"command","request_id":8,"generation":1,"epoch_id":42,"path":"/api/v1/mode/gravity_comp","body":{"enable":false}}
```

- `enable` (bool, required). `scale` (optional, within the profile's `scale_range`, rebot-dm 0.5–1.2) multiplies the model torque. `damping_kd` (optional) is one number for J1–J6 or six numbers, each 0 ≤ kd ≤ `damping_kd_max` (2.0). Out-of-range values: `400 INVALID_COMMAND`.
- `enable: true` while the mode is on is accepted without change; `enable: false` while it is off is accepted and does nothing.
- The ack does not mean the arm floats yet: follow `observation.gravity_comp.phase` (`ramp_in` → `active`). After `enable: false`, `control_mode` returns to `position` once the switch back to POS_VEL is confirmed (about 100 ms).
- Before sending, `observation.gravity_comp.blocker` gives the code the entry would be refused with (null when it would be accepted).
- While the mode is on: servo frames, `motion/jog`, `motion/home`, presets, `motion/unload`, `torque`, `move_to`, `move_relative` and `grasp` return `409 MOTION_BUSY`; `motion/stop` and E-stop end the mode; `action/gripper` works; recording (REST) works.
- `POST /api/v1/mode/gravity_comp` over REST returns `409 LEASE_REQUIRED` / `LEASE_ACTIVE` and does nothing.
- Why the mode ended: `observation.gravity_comp.last_exit.code` (see the contract for the list).

已在 reBot B601-DM 上实机验证（见 README）。完整契约见 [control_safety_contract.md](control_safety_contract.md#gravity-compensation-hand-guiding--重力补偿拖动示教)。`enable` 必填；`scale` 可选，须在 profile 的 `scale_range` 内（rebot-dm 为 0.5–1.2）；`damping_kd` 可选，一个数（J1–J6 共用）或六个数，每个 0 ≤ kd ≤ `damping_kd_max`（2.0），越界返回 `400 INVALID_COMMAND`。应答不代表已经漂浮，请看 `observation.gravity_comp.phase`（`ramp_in` → `active`）；`enable: false` 后切回 POS_VEL 确认（约 100 ms）时 `control_mode` 变回 `position`。发送前可读 `observation.gravity_comp.blocker`，得到会被拒绝的错误码（可进入时为 null）。模式开启时 servo 帧、`motion/jog`、`motion/home`、预设、`motion/unload`、`torque`、`move_to`、`move_relative`、`grasp` 返回 `409 MOTION_BUSY`；`motion/stop` 与急停结束本模式；`action/gripper` 与录制（REST）可用。REST 的 `POST /api/v1/mode/gravity_comp` 返回 `409 LEASE_REQUIRED` / `LEASE_ACTIVE`，不执行。模式结束原因见 `observation.gravity_comp.last_exit.code`。

## Diagnostics / 诊断

Read-only, no lease, `GET` only. They read the files armd writes under `--blackbox-dir`
(default `/userdata/rebot-teleop/blackbox`), which survive a reboot or power cut. Responses are
JSON, at most 2 MB (older lines are cut first, `truncated: true`). Without a black-box
directory both return `enabled: false` and empty lists.

| Path | Returns |
|---|---|
| `/api/v1/diagnostics/boots` | `{"enabled", "boot", "truncated", "boots": [...]}`: one entry per armd start, oldest first |
| `/api/v1/diagnostics/blackbox?seconds=N` | `{"enabled", "boot", "current_boot", "seconds", "truncated", "records": [...]}`: this start's records of the last N seconds (default 60, max 3600), oldest first |
| `/api/v1/diagnostics/blackbox?seconds=N&prev=1` | the same for the previous start: the seconds before the last reset or restart |
| `/api/v1/diagnostics/tick` | `{"window", "tick_us": {"p50","p99","max"}, "lock_us": {...}, "late_count", "since_boot_ticks"}` over the last 1000 control ticks (in memory, works without a black-box directory) |
| `/api/v1/diagnostics/power` | supply-current estimate, 5 s / 60 s peaks and governor state (in memory, see [Power budget](#power-budget--电源功率预算)) |

Boot entry (`boots.log`): `boot` (start counter, persisted in `boot_count`), `uptime_s` (kernel
uptime at armd start), `kernel_boot_id`, `same_kernel_boot` (false: the kernel booted since the
previous armd start; true: armd restarted without a reboot; null: unknown), `reset_reason` (SG2002
`cvi-base` power-on reason: `cold_boot`, `watchdog`, `suspend`, `warm_reset`; null when not
available; the driver reports unknown causes as `cold_boot`), `armd_version`, `prev_last` (the
last black-box record of the previous start, or null).

Black-box record (`bb.jsonl`): `boot`, `t_ms` (monotonic ms since armd start), `safety_state`,
`is_torqued`, `enable_state`, `lease_holder`, `control_mode`, `positions`,
`commanded_positions`, `velocities`, `torques`, `temperatures` (3 decimals), `feedback_valid`,
`feedback_age_ms`, `motor_errors`, `motion_limit`, `residual_torques` (3 decimals or null), `collision` (the observation field; omitted while null), `power` (`{"a": estimated supply current, "pk": its 5 s peak (A, 2 decimals), "s": governor scale, "hold": true only during a power hold}`; omitted without an estimate), `action` (latest action as `kind:state`),
`dropped` (samples dropped so far because the writer was busy). Rate: 5 Hz while torqued or a
joint moves faster than 0.05 rad/s, otherwise one line every 5 s. Each line is `fdatasync`ed
while torqued. Files: `bb.jsonl` rotated at 2 MiB, six old files kept (`.1` … `.6`); about 75 min
at 5 Hz (about 656 B per line, measured on the arm), about 30 h idle. The 2 MB response cap
holds about 10 min of 5 Hz records; request a shorter window to get it untruncated. armd has no CAN error counters, so the record carries none.

Tick timing: `tick_us` is the whole `ArmSupervisor::tick` call (waiting for the supervisor lock, the tick body and the telemetry broadcast), `lock_us` the time the tick holds the supervisor write lock, both in µs (nearest-rank percentiles). `late_count` counts ticks since start whose `dt` exceeded two control periods.

只读，无需租约，仅 `GET`。读取 armd 写在 `--blackbox-dir`（默认
`/userdata/rebot-teleop/blackbox`）下的文件，重启或断电后仍在。响应为 JSON，最大 2 MB（超出时先丢
最旧的行，`truncated: true`）。没有黑匣子目录时两者都返回 `enabled: false` 和空列表。`boots` 每次
armd 启动一条（含上一次启动的最后一条记录 `prev_last`、内核 `uptime_s`、`same_kernel_boot`、
SG2002 上电原因 `reset_reason`）；`blackbox?seconds=N` 返回本次启动最近 N 秒（默认 60，最大
3600）的记录，`&prev=1` 返回上一次启动的记录，即复位或重启前的那几秒。记录字段与采样率见上文英文部分。`/api/v1/diagnostics/tick` 返回最近 1000 个控制周期的耗时（`tick_us` 为整个周期调用，`lock_us` 为持有监督器写锁的时间，单位 µs）以及启动以来 `dt` 超过两个控制周期的次数 `late_count`。`/api/v1/diagnostics/power` 返回电源电流估算、5 s / 60 s 峰值与调速器状态（内存数据，见「电源功率预算」）；黑匣子记录的 `power` 字段为 `{"a": 估算电源电流, "pk": 5 s 峰值（A，2 位小数）, "s": 调速比例, "hold": 仅功率保持期间为 true}`，无估算时省略。

## Errors / 错误码

| code | status |
|---|---|
| `IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION`, `POWER_BUDGET` | 422 |
| `REPLAY_EMPTY`, `REPLAY_JOINT_COUNT`, `REPLAY_INVALID_EPISODE`, `REPLAY_TOO_LONG`, `REPLAY_TOO_FAST` | 422 |
| `SOFT_LIMIT_EXCEEDED`, `INVALID_EPISODE_ID`, `INVALID_OUTPUT_PATH` | 400 |
| `EPISODE_NOT_FOUND` | 404 |
| `EPISODES_DISABLED`, `EPISODE_EXISTS` | 409 |
| `EPISODES_FULL` (recording start / save) | 507 |
| `MOTION_BUSY`, `PLAN_STALE`, `COLLISION_HOLD` | 409 |
| `INVALID_COMMAND` (bad or missing params, path too long) | 400 |
| `UNSUPPORTED`, `GRAVITY_MODEL_UNAVAILABLE` | 422 |
| `NOT_ARMED` | 412 |
| `POSE_NOT_FOUND` | 404 |
| `GRAVITY_COMP_NOT_STATIONARY`, `GRAVITY_COMP_JOINT_LIMIT`, `GRAVITY_COMP_SWITCH_FAILED`, `FEEDBACK_STALE`, `LEASE_REQUIRED` (gravity compensation entry) | 409 |

Codes that only appear in `action.code`: `COLLISION`, `TORQUE_LIMIT`, `POWER_LIMIT`, `TRACKING_STALL`, `GRIPPER_TIMEOUT`, `ESTOP_ACTIVE`, `FAULT`, `WATCHDOG`, `LEASE_LOST`, `STOPPED`, `SUPERSEDED`, `UNLOAD_FAILED`; for `grasp` also `TARGET_LOST`, `TARGET_DISPLACED`, `CENTRING_FAILED`, `NOT_HOLDING`, `GRASP_TIMEOUT`, `DETECTION_UNAVAILABLE`, `INTERNAL`, and `IK_*` / `WORKSPACE_VIOLATION` / `PLAN_STALE` from a later phase; for `place` also `PLACE_TIMEOUT`, `INTERNAL`, and `IK_*` / `WORKSPACE_VIOLATION` / `PLAN_STALE` from a re-planned phase.

## Needs verification on the device / 需实机核实

- POS_VEL lag gain K = 1.5, measured 2026-11-09 (basis of the 0.45 rad/s cap and the 0.38 / 0.45 rad band).
- `squeeze_rad` 0.15 and `holding_torque_nm` 0.0667 (0.12 as read with the old TMAX 18 mapping of J7, converted to its real TMAX 10) come from the MIT-mode project; grip torque under POS_VEL depends on the motor's internal gains.
- `camera_mount` (`flip`, `tilt_deg`, `offset_m`) for camera-frame moves.
- Gravity compensation: the Damiao CTRL_MODE register write while enabled, its acknowledgement, the PMAX / VMAX / TMAX values of each B601-DM motor, the URDF torque against the measured hold torque, the POS_VEL transient when the feed-forward ends at the switch back, and the damping / overspeed defaults (checklist in the contract).
- Replay: tested in simulation only (first-order servo, `tests/test_replay.rs`); tracking of a replayed hand-guided episode under POS_VEL, and how often hand-guided episodes exceed the 0.45 rad/s planned-motion limit at 0.5x, are not measured on the arm.
- Planning time on the SG2002: `cargo test --release --test test_actions bench_plan -- --ignored --nocapture`.
- Power budget: the supply current against the estimate (clamp meter) at several poses, speeds and stalls; the datasheet motor parameters, `k_cu`, `efficiency`, idle and reCamera power; the LRS-150-24 trip current and time; power planning time on the SG2002 (`cargo test --release --test test_power_budget -- --nocapture` prints it).
- Grasp: `camera_tilt_deg` sign and value for the new mount, `camera_mount.offset_m`, `grip_pixel_norm`, `table_z_m`; detection rate and latency (`settle_delay_s`, `lost_*`); the jaw closing axis (assumed end_link y) for J6 alignment; `centre_tol_px` against detector jitter; `holding` thresholds under POS_VEL; camera-down acquisition with the v8 holder (mount tilt, rot and lens centre accuracy, reachability of the pitched approach); the v8 lens offset, body box, 5 mm table margin and `recheck_tol_px` per [Real-arm calibration](#real-arm-calibration-v8-mount--真机标定v8-支架). Simulated-camera tests: `cargo test --test test_grasp -- --nocapture`.
