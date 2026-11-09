# Control and safety contract / 控制与安全契约

## Scope / 范围

This document describes the software contract visible in the current source tree. It is not a hardware acceptance report. No physical arm test is claimed; behavior also depends on a trusted control network, correct wiring, fresh telemetry, and an external emergency-stop circuit.

本文记录当前源码可验证的软件契约，不构成实物验收报告。当前未进行实物测试；行为还取决于可信控制网络、正确接线、新鲜遥测和外部急停回路。

## Control path / 控制路径

- Observation remains available through `GET /api/v1/observation` and its schema endpoint.
- Actuator authority belongs to the WebSocket connection that acquires the lease. Use `acquire_lease`, then periodic `heartbeat`, then strictly increasing `request_id` `command` messages carrying `generation` and the current `epoch_id`.
- A successful command returns `command_ack`; lease loss, stale epoch, missing generation, or safety rejection returns `command_reject`. `estop` is accepted without a lease and advances the safety epoch.
- REST actuator routes (`torque`, `heartbeat`, `motion/*`, and `clear_fault`) return a structured lease error and do not execute the action. This is intentional: a caller-supplied client ID cannot prove WebSocket connection ownership.

- 观测通过 `GET /api/v1/observation` 及 schema 接口提供。
- 执行器权限属于取得租约的 WebSocket 连接。流程为 `acquire_lease`、周期性 `heartbeat`、带 `generation` 和当前 `epoch_id` 且 `request_id` 严格递增的 `command`。
- 成功返回 `command_ack`；租约丢失、epoch 过期、缺少 generation 或安全策略拒绝时返回 `command_reject`。`estop` 不需要租约，执行后会推进安全 epoch。
- REST 执行器路由（`torque`、`heartbeat`、`motion/*`、`clear_fault`）返回结构化租约错误，不执行动作。

## Driver capability boundary / 驱动能力边界

The observation fields `driver_capabilities`, `enable_state`, and `feedback_valid` are authoritative. `Unknown` means that fresh hardware confirmation is unavailable; it is not equivalent to `Disabled`. The bundled drivers declare different feedback and enable capabilities. In particular, Damiao default `POS_VEL` does not send gravity torque feed-forward; gravity output requires MIT capability and the configured gravity model. The source does not claim four physical driver integrations have been accepted.

观测中的 `driver_capabilities`、`enable_state`、`feedback_valid` 是实际边界。`Unknown` 表示没有新鲜硬件确认，不等于 `Disabled`。各驱动反馈和使能能力不同。达妙默认 `POS_VEL` 不发送重力力矩前馈；只有 MIT 能力和重力模型均满足时才允许重力输出。源码不宣称四款驱动已完成实物验收。

When enable confirmation is unavailable or remains `Unknown`, automatic disable, fault clearing, and service exit are rejected; this includes the non-Damiao drivers whose `enable_feedback` capability is false. The API remains available for observation and recovery decisions. Normal stop holds the commanded/measured pose and does not home the arm or force-kill the service. Ordinary `home` remains a motion command; automatic `unload`/park is rejected before planning when `supports_automatic_unload` is false.

当使能确认不可用或仍为 `Unknown` 时，自动失能、清故障和服务退出会被拒绝；这包括 `enable_feedback` 为 false 的非达妙驱动。API 保持可用以便观测和恢复决策。普通 stop 保持当前指令/测量位置，不回零，也不强杀服务。普通 `home` 仍是可用的运动命令；当 `supports_automatic_unload` 为 false 时，自动 `unload`/park 会在规划前拒绝。

## Timing and calibration defaults / 时序与标定默认值

- The control loop has an 80 ms overrun/stall protection deadline; a late tick latches a safety hold instead of advancing an old trajectory.
- Separately, the Damiao enable handshake polls for up to 80 ms for confirmed motor state.
- Feedback freshness checks use a 250 ms limit in the supervisor and bundled serial/CAN drivers.
- `FollowingErrorPolicy` defaults to `max_error_rad = 0.5` and `persistence_ms = 500`.
- The Damiao profile sets `max_jerk = 50.0`; it is a soft preference and does not override braking or position limits. Other default profiles leave it unset.
- These are software defaults, not measured industrial timing guarantees.

- 控制循环有 80 ms 的超期/停滞保护期限；tick 过晚时锁存安全保持，不继续推进旧轨迹。
- 另有独立的达妙使能握手确认轮询上限 80 ms。
- supervisor 及内置串口/CAN 驱动的反馈新鲜度上限为 250 ms。
- `FollowingErrorPolicy` 默认 `max_error_rad = 0.5`、`persistence_ms = 500`。
- 达妙 profile 设置 `max_jerk = 50.0`；这是软偏好，不覆盖制动和位置约束，其他默认 profile 不设置该值。
- 这些是软件默认值，不是工业时序保证。

## Planned speed, workspace and actions / 规划速度、工作空间与动作

- `planned_max_velocity` (rebot-dm 0.7 rad/s, default unset) caps every joint except the 7-DOF gripper for jog, home, presets, park (including watchdog park) and servo stepping. It keeps the expected Damiao POS_VEL lag (≈ v / 2.2 rad, `POSVEL_LAG_GAIN`, an estimate) below the 0.5 rad following-error bound; `validate()` rejects a cap whose expected lag reaches the bound. Non-cockpit servo clients (for example the companion `servo_stream`) are bounded by the same cap.
- The watchdog park deadline is max(10 s, planned park duration + 3 s), so a park from a far J6 pose is not cut short.
- The tracking-error fault names the joint: `Persistent joint tracking error: joint N cmd … meas … rad`. Observations carry `commanded_positions`.
- Cartesian workspace (`workspace` + `kinematics`, 7-DOF only): the end_link origin, the 8 gripper-envelope corners and the optional camera point must stay above `floor_z_m`, inside the box and within `max_reach_m` of the shoulder axis. A servo target, or any of the 21 planner samples of a jog / home / preset, is rejected with `422 WORKSPACE_VIOLATION` when it violates and is worse than the start; moving out of a violation is allowed. Park is exempt and only logged. The schema publishes `workspace` and `camera_mount`. A `--config` profile without these fields runs without the check and logs a warning at startup.
- Actions (`docs/agent_action_api.md`): `move_to`, `move_relative` and `gripper` over the WS lease; every observation carries `action`. Servo frames during an action return `409 MOTION_BUSY`. Action paths slow down when lag exceeds 0.38 rad and end as `TRACKING_STALL` after 1 s at 0.45 rad, so a blocked action path ends before the generic tracking fault would latch.

- `planned_max_velocity`（rebot-dm 为 0.7 rad/s，默认不设）限制除 7 轴夹爪外所有关节在 jog、home、预设、收纳（含看门狗收纳）和 servo 步进中的速度，使达妙 POS_VEL 预计滞后（约 v / 2.2 rad，估计值）低于 0.5 rad 跟随误差阈值；预计滞后达到阈值的配置会被 `validate()` 拒绝。非驾驶舱的 servo 客户端同样受此限制。
- 看门狗收纳期限为 max(10 s, 规划时长 + 3 s)。
- 跟踪误差故障信息包含关节号与指令/实测值；观测数据包含 `commanded_positions`。
- 笛卡尔工作空间（需 `workspace` 与 `kinematics`，仅 7 轴）：end_link 原点、夹爪包络 8 个角点及可选相机点须高于 `floor_z_m`、位于包围盒内且距肩轴不超过 `max_reach_m`。servo 目标或 jog / home / 预设轨迹的 21 个采样点越界且比起点更差时返回 `422 WORKSPACE_VIOLATION`；从越界位姿向外移动允许。收纳不受限，只记日志。schema 公开 `workspace` 与 `camera_mount`。未配置这些字段的 `--config` profile 不做检查，启动时告警。
- 动作接口见 `docs/agent_action_api.md`。动作运行时 servo 帧返回 `409 MOTION_BUSY`；动作路径在滞后超过 0.38 rad 时降速，0.45 rad 持续 1 s 结束为 `TRACKING_STALL`。

## Configuration / 配置

`--config` loads a complete `ArmConfig` JSON. Core fields are `dof`, `joint_names`, `limits`, `home_positions`, `lease_timeout_ms`, `max_temperature`, and `control_frequency_hz`; optional fields are `resting_pose`, `following_error`, `max_jerk`, `gravity_model`, `planned_max_velocity`, `kinematics`, `workspace`, `camera_mount`, and `gripper_policy`. Follower DOF is fixed by model (`rebot-dm=7`; `rebot-rs`, `so-arm101`, `fashionstar`, and `star-arm102=6`), and mismatches are rejected. `--leader` and `--leader-port` are rejected because leader-following is not implemented.

`--config` 加载完整 `ArmConfig` JSON。核心字段为 `dof`、`joint_names`、`limits`、`home_positions`、`lease_timeout_ms`、`max_temperature`、`control_frequency_hz`；可选字段为 `resting_pose`、`following_error`、`max_jerk`、`gravity_model`、`planned_max_velocity`、`kinematics`、`workspace`、`camera_mount`、`gripper_policy`。follower 的 DOF 按型号固定（`rebot-dm=7`；`rebot-rs`、`so-arm101`、`fashionstar`、`star-arm102=6`），不匹配会拒绝启动。`--leader` 与 `--leader-port` 因主从功能未实现而被拒绝。
