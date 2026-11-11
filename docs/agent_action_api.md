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

`kind`: `move_to`, `move_relative`, `gripper`, `grasp`, `place`, `joint_move`, `home`, `park`, `move_to_pose`. A `grasp` record also carries `phase`, `iteration` and `error_px`, and, for camera-down acquisition, `final_offset_px` and `acquire_height_m` (fields absent otherwise); a `place` record carries `phase` (all three are null for other kinds). `state`: `running`, `done`, `failed`, `cancelled`. `started_at` / `finished_at` use the observation `timestamp` clock. Clients block by waiting until `action.id == action_id` and `state != "running"`, or until `previous_action.id == action_id`.

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
- `fault`: null, or `{"code", "message", "recoverable"}` while `safety_state` is Fault or EStop (`safety_state` keeps its shape). Codes: `ESTOP`, `BUS_LOSS`, `THERMAL_TRIP`, `JOINT_LIMIT`, `WATCHDOG_TIMEOUT`, `FOLLOWING_ERROR`, `CONTROL_DEADLINE`, `INVALID_TELEMETRY`, `DRIVER_WRITE_FAILED`, `HARDWARE_FAULT`, `ENABLE_FAILED`, `SERVO_NO_BRAKE`, `HANDOVER_UNSETTLED`, `GRAVITY_COMP_FAILED`, `COLLISION`, `OTHER`. `recoverable` is true only for `WATCHDOG_TIMEOUT`, `CONTROL_DEADLINE`, `HANDOVER_UNSETTLED`; it is a hint for client policies, armd never clears a fault by itself.
- `residual_torques`: measured torque minus model gravity torque for J1..J6 (N·m), null without a gravity model or valid torque feedback (also null with the mock follower). The residual raises no fault itself; the collision guard below uses it.

`?obs_hz=N` 限制该连接每秒最多 N 条观测（取 1..`control_frequency_hz`），租约、应答、拒绝、`lease_lost`、`epoch_sync` 不降频。`fault` 在 Fault / EStop 时为 `{code, message, recoverable}`，否则为 null；`recoverable` 仅对 `WATCHDOG_TIMEOUT`、`CONTROL_DEADLINE`、`HANDOVER_UNSETTLED` 为 true，只是给客户端的提示，armd 不会自行清除故障。`residual_torques` 为 J1..J6 实测力矩减模型重力力矩，无重力模型或无有效力矩反馈时为 null，本身不触发故障，碰撞保护（见下）使用它。

## Collision guard / 碰撞保护

Contact detection with a protective stop, built on `residual_torques` (capability `collision_guard`: profile gravity model and a driver that reports torque). It is contact detection, not a certified safety function, and the default thresholds are placeholders until measured on a real arm, so it is **off by default**; thresholds must come from rebot-sysid. Joints resting on their support (`resting_pose.support_torque_limits`) are not watched at the resting pose. Full rules: `docs/control_safety_contract.md` "Collision guard".

- Profile section `collision`: `{"enabled": false, "thresholds_nm": [6.0,6.0,6.0,2.5,2.5,2.5], "persist_ticks": 4, "clear_ratio": 0.5, "resume_after_s": 2.0, "max_auto_resumes": 3, "escalate_window_s": 60}` (all optional).
- `GET /api/v1/safety/collision` (no lease): `{"available", "enabled", "sensitivity", "thresholds_nm" (scaled), "profile_thresholds_nm", "persist_ticks", "clear_ratio", "resume_after_s", "max_auto_resumes", "escalate_window_s", "source": "profile"|"runtime", "active"}`.
- WS command `/api/v1/safety/collision` `{"enabled": bool?, "sensitivity": "low"|"medium"|"high"?}` (lease; at least one field; unknown fields or values are `400 INVALID_COMMAND`; enabling while unavailable is `422 UNSUPPORTED`). Sensitivity scales the thresholds ×1.5 / ×1.0 / ×0.7. The ack carries `collision` (the config above) and `persisted`; the setting is written to `ARMD_COLLISION` (default `/userdata/rebot-teleop/collision.json`) and loaded at the next start. Switching the guard off ends a hold at once.
- Trip: |residual| above the threshold for `persist_ticks` ticks on any of J1..J6, with torque on, Armed / Moving and gravity compensation off. The running action ends `failed` with code `COLLISION` (message names the joint and residual); the arm holds its measured pose with torque on. `safety_state` does not change.
- While `collision.active`, motion (jog, home, presets, park, servo, every action, gravity-compensation entry) is rejected with `409 COLLISION_HOLD`; e_stop, stop, clear_fault and torque off work.
- The hold ends by itself after every residual stays below `clear_ratio` × threshold for `resume_after_s`. The (`max_auto_resumes`+1)-th trip within `escalate_window_s` latches Fault code `COLLISION` (`recoverable: false`), which needs `clear_fault`.
- Observation `collision`: null while disabled or unavailable, else `{"active": bool, "joint": int|null (1-based), "residual_nm": float|null, "threshold_nm": float|null, "since_s": float|null, "resume_in_s": float|null, "trips_in_window": int, "enabled": true, "sensitivity": "medium"}`.

Client pattern: on an action failing with `COLLISION`, wait until `collision.active` is false (raise if `fault.code` is `COLLISION`), then retry the step once; on escalation stop.

基于 `residual_torques` 的接触检测与保护性停止（能力 `collision_guard`：profile 有重力模型且驱动报告力矩）。它是接触检测，不是经过认证的安全功能；默认阈值在真机实测前为占位值，因此**默认关闭**；阈值须来自 rebot-sysid。在收纳位姿靠在支撑上的关节（`resting_pose.support_torque_limits`）不参与检测。完整规则见 `docs/control_safety_contract.md`“碰撞保护”。profile 段 `collision` 字段同上；`GET /api/v1/safety/collision`（无需租约）返回含缩放后阈值的生效配置；WS 指令 `/api/v1/safety/collision`（需租约）设置 `enabled` 与 `sensitivity`（low / medium / high，阈值乘以 1.5 / 1.0 / 0.7），写入 `ARMD_COLLISION`（默认 `/userdata/rebot-teleop/collision.json`），下次启动加载；关闭保护会立即结束保持。触发后正在执行的动作以 `COLLISION` 失败，机械臂在实测位置保持力矩，`safety_state` 不变；保持期间运动类指令返回 `409 COLLISION_HOLD`，急停、停止、`clear_fault`、断力矩仍可用。残差低于 `clear_ratio` × 阈值持续 `resume_after_s` 后自动恢复；`escalate_window_s` 内第 `max_auto_resumes`+1 次触发锁存 Fault `COLLISION`，须 `clear_fault`。观测字段 `collision` 结构见上。

## Safety self-description / 安全自描述

`GET /api/v1/safety/describe` (no lease; capability `safety_describe`) lists this armd's timing, joint and Cartesian limits, keep-out boxes, speed cap, stops (`e_stop`, `watchdog` with the outcome for this config, `bus_loss`, `control_deadline`, `following_error`, `thermal`, and `collision_protective_stop` with `collision_guard`), every fault code with its recovery steps, and hardware notes. Values come from the running config and the code's constants. A client or AI assistant reads it before planning motion and keeps within those limits. Field list: `docs/control_safety_contract.md` "Safety self-description".

`GET /api/v1/safety/describe`（无需租约；能力 `safety_describe`）列出本 armd 的时序、关节与笛卡尔限值、禁入盒、速度上限、停止方式（`e_stop`、按本配置给出结果的 `watchdog`、`bus_loss`、`control_deadline`、`following_error`、`thermal`，有 `collision_guard` 时另有 `collision_protective_stop`）、每个故障码及其恢复步骤和硬件说明。数值取自运行中的配置与代码常量。客户端或 AI 助手在规划运动前读取，并在这些限值内运行。字段列表见 `docs/control_safety_contract.md`“安全自描述”。

## Grasp / 点框抓取

`/api/v1/action/grasp` picks up an object the user selected in the camera image. The request names a detection box (normalised `box_norm` from the detector, its `label`, optionally the frame); the action does the rest and ends with the object lifted, or with a failure code.

`/api/v1/action/grasp` 抓取用户在画面中选中的物体：请求给出检测框（归一化 `box_norm`、类别 `label`，可选帧号），动作完成后物体被抬起，或以失败码结束。

Needs a 7-DOF profile with `kinematics`, `gripper_policy` and `grasp` (the built-in `rebot-dm` has all three); otherwise `UNSUPPORTED`.

### Before the ack / 应答之前

Under the lock: lease, epoch, request_id, body, then the same preconditions as other actions (`MOTION_BUSY` etc.). Outside the lock: the box centre ray (setup intrinsics, current commanded pose, `camera_mount`) is intersected with the plane z = `table_z_m + grasp_height_m` to estimate the target; the approach pose above it (look-down, or camera-down, see [Camera-down acquisition](#camera-down-acquisition--相机朝下采集)) is planned with the gripper open. A planning failure is a `command_reject` with the planner's code (`IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION`, 422); a ray that does not meet the table in front of the camera, or `acquire_camera_down: "on"` without `camera_mount.offset_m`, is `INVALID_COMMAND`. Then the action begins and the ack carries `action_id` and a rough `expected_duration_s`.

锁内校验租约、epoch、request_id、请求体和动作前置条件；锁外用框中心射线与平面 z = `table_z_m + grasp_height_m` 求交估计目标位置，并以夹爪张开状态规划接近位姿（俯视；相机朝下采集时为光轴竖直）。规划失败直接 `command_reject`（`IK_UNREACHABLE` / `IK_BRANCH_FLIP` / `WORKSPACE_VIOLATION`，422）；射线不与桌面相交、或 `acquire_camera_down: "on"` 而 `camera_mount.offset_m` 未设置，返回 `INVALID_COMMAND`。

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

### Camera-down acquisition / 相机朝下采集

For a camera mount pitched toward the jaws (v8 holder: `camera_mount` `rot_deg` 90, `tilt_deg` −38), the table point under the jaw tips is outside the image at approach height when the tool points straight down, so the look-down centring above cannot run. The setup field `acquire_camera_down` selects the acquisition:

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

Table clearance (camera-down only): the pitched gripper reaches below the TCP (with the kit check points up to 57 mm at a 38° pitch about the end_link z axis, 21 mm about the y axis), and the workspace check compares against `workspace.floor_z_m`, not the table. The acquisition therefore raises the TCP above `table_z_m + approach_height_m` when needed so that the TCP, every `workspace.check_points_end_m` point, the lens centre and the 8 corners of the camera body box stay at least 5 mm above `table_z_m` at the camera-down orientation and along the transition (the raised height is `action.acquire_height_m`, above the table; absent when not raised). The camera body box is `camera_mount.body_size_m` [width along x_cam, height along y_cam, depth behind the lens along −z_cam] (m, optional, each in (0, 0.2]); the default 0.055 x 0.047 x 0.040 is the reCamera housing (40 x 40 x 36.5 mm) in the v8 holder (walls, ears, support, top bar and rear lip from the holder's generator), lens at the centre of the front face. It is used only for the camera-down clearance, not in the general workspace check, so the look-down grasp and other motions are unchanged; the general check still counts the lens centre only. With the kit check points and the v8 mount the gripper points, not the body, set the raised height; a mount whose body reaches lower than the gripper is rejected or raised by the same rule. The approach path, each centring step and the transition path are sampled (5 mm / 1.1°) against the same bound before they are issued; a violation is `WORKSPACE_VIOLATION` (a `command_reject` for the approach, the action's code later). A path that starts below the bound may move as long as no point goes lower.

The accuracy of the transition depends on `camera_tilt_deg`, `camera_mount.rot_deg` and `offset_m`: an error in them moves the grasp point (in the simulated-camera test, a 2° tilt, 1° rot, 5 % focal length and (6, −4) px principal point error gave a 9 mm grasp point error at a 0.15 m approach). `expected_duration_s` adds the transition move and 1.5 s for the re-check frame. With \|`camera_tilt_deg`\| ≤ 5° (v5 mount) `auto` keeps the look-down acquisition unchanged. Validated in simulation only (`cargo test --test test_grasp camera_down`, `sim/mujoco/tests/test_camera_grasp.py` v8 cases); not run on the arm.

相机安装向夹爪方向俯仰时（v8 支架：`camera_mount` `rot_deg` 90、`tilt_deg` −38），工具竖直朝下时夹爪尖正下方的桌面点在接近高度处不在画面内，上面的俯视对中无法进行。抓取设置字段 `acquire_camera_down` 选择采集方式：`auto`（默认）在 \|`camera_tilt_deg`\| > 5° 且 `camera_mount.offset_m` 已设置时用相机朝下采集，否则俯视；`on` 始终相机朝下，`camera_mount.offset_m` 未设置时请求返回 `INVALID_COMMAND`；`off` 始终俯视。

相机朝下采集的阶段：`approach` 将俯视姿态做最小旋转使光轴竖直向下（工具绕 x_cam 俯仰安装倾角，朝向不变），镜头中心（FK + `camera_mount.offset_m`）在 x、y 上位于估计点正上方，TCP 高度为 `table_z_m + approach_height_m`；`centring` 与上面的闭环相同，目标点改为画面中心（`image_size_px` / 2），增益、步长上限、`stable_frames`、`max_iterations` 不变；`transition` 用最后一次匹配框中心（距画面中心不超过 `centre_tol_px`）的射线与平面 z = `table_z_m + grasp_height_m` 求交，一次规划的笛卡尔运动（与其他运动一样做 IK、工作空间和禁入区检查）移到 TCP 位于该点上方接近高度、工具竖直的位姿；之后若相机模型预测目标在新位姿的画面内，下降前取一帧新图复查：未匹配到（`lost_frames` / `lost_timeout_s`，以投影为中心、门限 `acquire_dist_px`）为 `TARGET_LOST`，检测器出错为 `DETECTION_UNAVAILABLE`，框中心距投影超过 `recheck_tol_px` 为 `TARGET_DISPLACED`；`final_offset_px` = 匹配框中心减 `grip_pixel_norm` 的像素差 [dx, dy]（诊断用；位移判断以投影为准，抓取像素未标定不会导致失败）；预测目标不在画面内时（如 v8 支架、0.15 m 接近高度）不做复查，`final_offset_px` 缺省；`align`、`descend`、`close`、`lift` 不变，J6 对齐使用对中阶段最后一个框。桌面间隙（仅相机朝下采集）：俯仰后的夹爪会低于 TCP（套件检查点在绕 end_link z 轴俯仰 38° 时最多低 57 mm，绕 y 轴时 21 mm），而工作空间检查比较的是 `workspace.floor_z_m`，不是桌面。因此采集时在需要时把 TCP 抬高到 `table_z_m + approach_height_m` 以上，使 TCP、所有 `workspace.check_points_end_m` 点、镜头中心和相机机身方盒的 8 个角点在相机朝下姿态及整个转换过程中都高于 `table_z_m` 至少 5 mm（抬高后的高度见 `action.acquire_height_m`，相对桌面；未抬高时缺省）。相机机身方盒为 `camera_mount.body_size_m` [沿 x_cam 宽、沿 y_cam 高、沿 −z_cam 镜头后方深度]（米，可选，每项 (0, 0.2]），默认 0.055 x 0.047 x 0.040，即 reCamera 外壳（40 x 40 x 36.5 mm）装在 v8 支架内（壁厚、耳、底托、顶杆和后唇取自支架生成脚本），镜头位于前面中心。它只用于相机朝下采集的桌面间隙，不进入通用工作空间检查，因此俯视抓取和其他运动不变；通用检查仍只计镜头中心。使用套件检查点和 v8 支架时，决定抬高量的是夹爪检查点而不是机身；机身比夹爪更低的安装按同一规则抬高或拒绝。接近路径、每个对中步和转换路径在下发前按同一下限采样检查（5 mm / 1.1°），违反时为 `WORKSPACE_VIOLATION`（接近阶段为 `command_reject`，之后为动作失败码）。起点已低于下限的路径，只要没有点变得更低即可执行。转换的精度取决于 `camera_tilt_deg`、`camera_mount.rot_deg` 和 `offset_m`（模拟相机测试中倾角差 2°、旋转差 1°、焦距差 5 %、主点偏 (6, −4) px，在 0.15 m 接近高度下抓取点偏差 9 mm）。`expected_duration_s` 增加转换运动时长和 1.5 s 复查帧。\|`camera_tilt_deg`\| ≤ 5°（v5 支架）时 `auto` 保持原俯视流程不变。仅在仿真中验证（`cargo test --test test_grasp camera_down`、`sim/mujoco/tests/test_camera_grasp.py` 的 v8 用例），未在真机上运行。

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

Before the ack, outside the lock, lift, approach, descend and retreat are planned in sequence from the commanded state (each from the end of the previous one, retreat with the jaws open when `release`). A failure is a `command_reject` with the planner code (`IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION` incl. keep-out boxes with `keepout '<name>'` in the message, 422); an unknown `pose` is `POSE_NOT_FOUND` (404); bad fields are `INVALID_COMMAND`. Each phase is planned again from the commanded state when it runs.

Execution uses the same orchestration as `grasp` (`src/macro_action.rs`): every motion is a sub-motion of the one `place` record, installed under the lock only while the action is still running, and the cancellation table of [Grasp](#phases--阶段) applies unchanged (`/motion/stop` → `cancelled` / `STOPPED`, new motion → `SUPERSEDED`, lease → `LEASE_LOST`, E-stop / Fault / watchdog → `failed`). Servo frames are rejected with `MOTION_BUSY` for the whole action. A sub-motion longer than its planned duration × 2 + 5 s (gripper: expected duration + 10 s), capped at 60 s, or the whole action longer than 2 × `expected_duration_s` + 20 s, stops the motion and ends as `PLACE_TIMEOUT`. Sub-motion failures (`TRACKING_STALL`, `GRIPPER_TIMEOUT`, planner codes) end the action with that code.

应答前在锁外按顺序规划抬升、接近、下降和撤离（每段从上一段终点开始；`release` 时撤离段按夹爪张开规划）。规划失败直接 `command_reject`（`IK_UNREACHABLE` / `IK_BRANCH_FLIP` / `WORKSPACE_VIOLATION`，含禁入盒，消息中带 `keepout '<name>'`，422）；`pose` 不存在返回 `POSE_NOT_FOUND`（404）；参数错误返回 `INVALID_COMMAND`。执行时每个阶段从当前指令位置重新规划。

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

Boot entry (`boots.log`): `boot` (start counter, persisted in `boot_count`), `uptime_s` (kernel
uptime at armd start), `kernel_boot_id`, `same_kernel_boot` (false: the kernel booted since the
previous armd start; true: armd restarted without a reboot; null: unknown), `reset_reason` (SG2002
`cvi-base` power-on reason: `cold_boot`, `watchdog`, `suspend`, `warm_reset`; null when not
available; the driver reports unknown causes as `cold_boot`), `armd_version`, `prev_last` (the
last black-box record of the previous start, or null).

Black-box record (`bb.jsonl`): `boot`, `t_ms` (monotonic ms since armd start), `safety_state`,
`is_torqued`, `enable_state`, `lease_holder`, `control_mode`, `positions`,
`commanded_positions`, `velocities`, `torques`, `temperatures` (3 decimals), `feedback_valid`,
`feedback_age_ms`, `motor_errors`, `motion_limit`, `residual_torques` (3 decimals or null), `collision` (the observation field; omitted while null), `action` (latest action as `kind:state`),
`dropped` (samples dropped so far because the writer was busy). Rate: 5 Hz while torqued or a
joint moves faster than 0.05 rad/s, otherwise one line every 5 s. Each line is `fdatasync`ed
while torqued. Files: `bb.jsonl` rotated at 512 KiB, two old files kept (`.1`, `.2`); about 10 min
at 5 Hz (about 490 B per line), about 4 h idle. armd has no CAN error counters, so the record carries none.

Tick timing: `tick_us` is the whole `ArmSupervisor::tick` call (waiting for the supervisor lock, the tick body and the telemetry broadcast), `lock_us` the time the tick holds the supervisor write lock, both in µs (nearest-rank percentiles). `late_count` counts ticks since start whose `dt` exceeded two control periods.

只读，无需租约，仅 `GET`。读取 armd 写在 `--blackbox-dir`（默认
`/userdata/rebot-teleop/blackbox`）下的文件，重启或断电后仍在。响应为 JSON，最大 2 MB（超出时先丢
最旧的行，`truncated: true`）。没有黑匣子目录时两者都返回 `enabled: false` 和空列表。`boots` 每次
armd 启动一条（含上一次启动的最后一条记录 `prev_last`、内核 `uptime_s`、`same_kernel_boot`、
SG2002 上电原因 `reset_reason`）；`blackbox?seconds=N` 返回本次启动最近 N 秒（默认 60，最大
3600）的记录，`&prev=1` 返回上一次启动的记录，即复位或重启前的那几秒。记录字段与采样率见上文英文部分。`/api/v1/diagnostics/tick` 返回最近 1000 个控制周期的耗时（`tick_us` 为整个周期调用，`lock_us` 为持有监督器写锁的时间，单位 µs）以及启动以来 `dt` 超过两个控制周期的次数 `late_count`。

## Errors / 错误码

| code | status |
|---|---|
| `IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION` | 422 |
| `MOTION_BUSY`, `PLAN_STALE`, `COLLISION_HOLD` | 409 |
| `INVALID_COMMAND` (bad or missing params, path too long) | 400 |
| `UNSUPPORTED`, `GRAVITY_MODEL_UNAVAILABLE` | 422 |
| `NOT_ARMED` | 412 |
| `POSE_NOT_FOUND` | 404 |
| `GRAVITY_COMP_NOT_STATIONARY`, `GRAVITY_COMP_JOINT_LIMIT`, `GRAVITY_COMP_SWITCH_FAILED`, `FEEDBACK_STALE`, `LEASE_REQUIRED` (gravity compensation entry) | 409 |

Codes that only appear in `action.code`: `COLLISION`, `TRACKING_STALL`, `GRIPPER_TIMEOUT`, `ESTOP_ACTIVE`, `FAULT`, `WATCHDOG`, `LEASE_LOST`, `STOPPED`, `SUPERSEDED`, `UNLOAD_FAILED`; for `grasp` also `TARGET_LOST`, `TARGET_DISPLACED`, `CENTRING_FAILED`, `NOT_HOLDING`, `GRASP_TIMEOUT`, `DETECTION_UNAVAILABLE`, `INTERNAL`, and `IK_*` / `WORKSPACE_VIOLATION` / `PLAN_STALE` from a later phase; for `place` also `PLACE_TIMEOUT`, `INTERNAL`, and `IK_*` / `WORKSPACE_VIOLATION` / `PLAN_STALE` from a re-planned phase.

## Needs verification on the device / 需实机核实

- POS_VEL lag gain K = 1.5, measured 2026-11-09 (basis of the 0.45 rad/s cap and the 0.38 / 0.45 rad band).
- `squeeze_rad` 0.15 and `holding_torque_nm` 0.0667 (0.12 as read with the old TMAX 18 mapping of J7, converted to its real TMAX 10) come from the MIT-mode project; grip torque under POS_VEL depends on the motor's internal gains.
- `camera_mount` (`flip`, `tilt_deg`, `offset_m`) for camera-frame moves.
- Gravity compensation: the Damiao CTRL_MODE register write while enabled, its acknowledgement, the PMAX / VMAX / TMAX values of each B601-DM motor, the URDF torque against the measured hold torque, the POS_VEL transient when the feed-forward ends at the switch back, and the damping / overspeed defaults (checklist in the contract).
- Planning time on the SG2002: `cargo test --release --test test_actions bench_plan -- --ignored --nocapture`.
- Grasp: `camera_tilt_deg` sign and value for the new mount, `camera_mount.offset_m`, `grip_pixel_norm`, `table_z_m`; detection rate and latency (`settle_delay_s`, `lost_*`); the jaw closing axis (assumed end_link y) for J6 alignment; `centre_tol_px` against detector jitter; `holding` thresholds under POS_VEL; camera-down acquisition with the v8 holder (mount tilt, rot and lens centre accuracy, reachability of the pitched approach). Simulated-camera tests: `cargo test --test test_grasp -- --nocapture`.
