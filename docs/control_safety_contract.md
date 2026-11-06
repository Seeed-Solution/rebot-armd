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

## Configuration / 配置

`--config` loads a complete `ArmConfig` JSON. Core fields are `dof`, `joint_names`, `limits`, `home_positions`, `lease_timeout_ms`, `max_temperature`, and `control_frequency_hz`; optional fields are `resting_pose`, `following_error`, `max_jerk`, and `gravity_model`. Follower DOF is fixed by model (`rebot-dm=7`; `rebot-rs`, `so-arm101`, `fashionstar`, and `star-arm102=6`), and mismatches are rejected. `--leader` and `--leader-port` are rejected because leader-following is not implemented.

`--config` 加载完整 `ArmConfig` JSON。核心字段为 `dof`、`joint_names`、`limits`、`home_positions`、`lease_timeout_ms`、`max_temperature`、`control_frequency_hz`；可选字段为 `resting_pose`、`following_error`、`max_jerk`、`gravity_model`。follower 的 DOF 按型号固定（`rebot-dm=7`；`rebot-rs`、`so-arm101`、`fashionstar`、`star-arm102=6`），不匹配会拒绝启动。`--leader` 与 `--leader-port` 因主从功能未实现而被拒绝。
