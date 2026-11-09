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

The observation fields `driver_capabilities`, `enable_state`, and `feedback_valid` are authoritative. `Unknown` means that fresh hardware confirmation is unavailable; it is not equivalent to `Disabled`. The bundled drivers declare different feedback and enable capabilities. In particular, Damiao default `POS_VEL` does not send gravity torque feed-forward; gravity output requires MIT capability and the configured gravity model. The gravity-compensation mode below switches J1–J6 to MIT only while it is on. The source does not claim four physical driver integrations have been accepted.

观测中的 `driver_capabilities`、`enable_state`、`feedback_valid` 是实际边界。`Unknown` 表示没有新鲜硬件确认，不等于 `Disabled`。各驱动反馈和使能能力不同。达妙默认 `POS_VEL` 不发送重力力矩前馈；只有 MIT 能力和重力模型均满足时才允许重力输出。下文的重力补偿模式只在开启期间把 J1–J6 切到 MIT。源码不宣称四款驱动已完成实物验收。

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

- `planned_max_velocity` (rebot-dm 0.45 rad/s, default unset) caps every joint except the 7-DOF gripper for planned motion: jog, home, presets, park (including watchdog park) and high-level actions. It keeps the Damiao POS_VEL lag (≈ v / 1.5 rad, `POSVEL_LAG_GAIN`, measured 2026-11-09: J3 trailed 0.458 rad at 0.7 rad/s) below the 0.5 rad following-error bound; `validate()` rejects a cap whose expected lag reaches the bound. Servo streaming and stream braking keep the joint limits (≤ 2.0 rad/s): stream clients bound their own lead (the cockpit keeps it under 0.2 rad).
- The watchdog park deadline is max(10 s, planned park duration + 3 s), so a park from a far J6 pose is not cut short.
- The tracking-error fault names the joint: `Persistent joint tracking error: joint N cmd … meas … rad`. Observations carry `commanded_positions`.
- Cartesian workspace (`workspace` + `kinematics`, 7-DOF only): the end_link origin, the 8 gripper-envelope corners and the optional camera point must stay above `floor_z_m`, inside the box and within `max_reach_m` of the shoulder axis. A servo target, or any of the 21 planner samples of a jog / home / preset, is rejected with `422 WORKSPACE_VIOLATION` when it violates and is worse than the start; moving out of a violation is allowed. Park is exempt and only logged. The schema publishes `workspace` and `camera_mount`. A `--config` profile without these fields runs without the check and logs a warning at startup.
- Actions (`docs/agent_action_api.md`): `move_to`, `move_relative` and `gripper` over the WS lease; every observation carries `action`. Servo frames during an action return `409 MOTION_BUSY`. Action paths slow down when lag exceeds 0.38 rad and end as `TRACKING_STALL` after 1 s at 0.45 rad, so a blocked action path ends before the generic tracking fault would latch.

- `planned_max_velocity`（rebot-dm 为 0.45 rad/s，默认不设）限制除 7 轴夹爪外所有关节在规划运动中的速度：jog、home、预设、收纳（含看门狗收纳）和高层动作，使达妙 POS_VEL 滞后（约 v / 1.5 rad，2026-11-09 实测：0.7 rad/s 时 J3 滞后 0.458 rad）低于 0.5 rad 跟随误差阈值；预计滞后达到阈值的配置会被 `validate()` 拒绝。servo 流式控制及其制动仍用关节限位（≤ 2.0 rad/s），由客户端自行限制领先量（驾驶舱保持在 0.2 rad 以内）。
- 看门狗收纳期限为 max(10 s, 规划时长 + 3 s)。
- 跟踪误差故障信息包含关节号与指令/实测值；观测数据包含 `commanded_positions`。
- 笛卡尔工作空间（需 `workspace` 与 `kinematics`，仅 7 轴）：end_link 原点、夹爪包络 8 个角点及可选相机点须高于 `floor_z_m`、位于包围盒内且距肩轴不超过 `max_reach_m`。servo 目标或 jog / home / 预设轨迹的 21 个采样点越界且比起点更差时返回 `422 WORKSPACE_VIOLATION`；从越界位姿向外移动允许。收纳不受限，只记日志。schema 公开 `workspace` 与 `camera_mount`。未配置这些字段的 `--config` profile 不做检查，启动时告警。
- 动作接口见 `docs/agent_action_api.md`。动作运行时 servo 帧返回 `409 MOTION_BUSY`；动作路径在滞后超过 0.38 rad 时降速，0.45 rad 持续 1 s 结束为 `TRACKING_STALL`。

## Gravity compensation (hand-guiding) / 重力补偿（拖动示教）

Status: implemented and tested against a simulated arm (`tests/test_gravity_comp.rs`, `tests/test_gravity_comp_api.rs`, `tests/test_damiao_mode_switch.rs`). **Not verified on hardware.** Code: `src/gravity_comp.rs`; parameters: `ArmConfig.gravity_comp` (`GravityCompPolicy`, set for rebot-dm, `None` elsewhere = unsupported).

- **Control law.** J1–J6 run in Damiao MIT mode with kp = 0, damping `damping_kd` and feed-forward τ = clamp(URDF gravity torque × `scale`, ±`torque_limits_nm`). The gripper stays in POS_VEL at its commanded position.
- **Mode switch (Damiao).** The control mode is register 10 (CTRL_MODE: 1 = MIT, 2 = POS_VEL), written in RAM with a `0x7FF` frame `[id_l, id_h, 0x55, 10, value u32 LE]`; MIT frames go to CAN ID = motor ID, POS_VEL frames to `0x100 + ID`. The motor is not disabled for the switch. The layout follows Damiao's DM_CAN reference library; register reads were confirmed on the B601-DM (2026-11-09), writes have not been checked on hardware. Sequence per joint: write CTRL_MODE, then the hold frame in the new mode immediately after it on the bus, then wait up to 50 ms for the write acknowledgement (`[id_l, id_h, 0x55, 10, value]` on the feedback ID). An unacknowledged joint is switched back and held with both a POS_VEL and an MIT frame; a joint that still cannot be confirmed keeps receiving both frames on every write and latches a fault. Before the first switch to MIT the driver reads PMAX / VMAX / TMAX (registers 21–23) and refuses the mode when they differ from that joint's configured range (`damiao_ranges`; rebot-dm: J1–J3 ±12.5 rad, ±10 rad/s, ±28 N·m, J4–J7 ±12.5 rad, ±30 rad/s, ±10 N·m; profiles without `damiao_ranges` use ±12.5 / ±30 / ±18 for every joint), because a different TMAX scales every torque command. Feedback is decoded with the same per-joint ranges. CTRL_MODE is not stored to flash; the next enable handshake restores POS_VEL on any joint left in another mode, and a motor power cycle returns to the flashed mode.
- **Entry** (`GRAVITY_COMP_*` or the usual codes otherwise): rebot-dm profile with the verified URDF and a driver that can switch joints (`UNSUPPORTED`); no E-stop or fault; torque on and state `Armed` (`NOT_ARMED`); no trajectory, servo stream, grasp or park (`MOTION_BUSY`); lease held (`LEASE_REQUIRED`); feedback younger than `feedback_max_age_ms` (`FEEDBACK_STALE`); every arm joint slower than `entry_max_velocity` (`GRAVITY_COMP_NOT_STATIONARY`); no joint within `joint_limit_margin_rad` of a limit (`GRAVITY_COMP_JOINT_LIMIT`); model torque computable (`GRAVITY_MODEL_UNAVAILABLE`). A failed switch is `GRAVITY_COMP_SWITCH_FAILED`.
- **Ramp-in.** The switch frame holds the measured pose with `hold_kp` / `hold_kd` and the torque the motor reported (the load the position loop was carrying). Over `ramp_in_s` the stiffness fades to 0, the damping to `damping_kd`, and the feed-forward from the reported torque to the model torque. A joint that moves more than `ramp_max_deviation_rad` from the entry pose during the ramp ends the mode (`GRAVITY_COMP_SAG`).
- **Exit.** Every exit captures the measured pose and at once sends an MIT stiff hold there (`hold_kp`, `hold_kd`, feed-forward unchanged); after `exit_hold_ms` J1–J6 are switched back to POS_VEL holding that pose. POS_VEL frames carry no torque, so the feed-forward ends with that switch and the motor's position loop carries the load. If the switch back is not acknowledged the MIT stiff hold continues, `GRAVITY_COMP_EXIT_FAILED` is latched as a fault (it cannot be cleared before POS_VEL is confirmed), and the switch is retried every 0.5 s.
- **Exit causes and codes** (`gravity_comp.last_exit.code`): `REQUESTED` (mode command), `STOPPED` (`/api/v1/motion/stop`), `ESTOP`, `FAULT`, `BUS_LOSS`, `FEEDBACK_STALE` (older than `feedback_max_age_ms`), `WATCHDOG` (no heartbeat for the lease timeout), `LEASE_LOST` (released, expired or taken over), `TORQUE_LOST`, `GRAVITY_COMP_OVERSPEED` (a joint faster than `max_speed_rad_s` for `overspeed_ticks` ticks, speed = max of reported velocity and position difference per tick), `GRAVITY_COMP_SAG`, `GRAVITY_COMP_JOINT_LIMIT`, `GRAVITY_MODEL_UNAVAILABLE`. None of them turns torque off. E-stop and bus-loss handling send the hold in the same call; the others act at the next control tick (≤ 20 ms at 50 Hz). Overspeed is reported as the exit reason, not as a fault.
- **Watchdog.** While the mode is on the lease watchdog does not start a park. A heartbeat gap first ends the mode (`WATCHDOG`, hold at the measured pose); once the arm is back in POS_VEL the existing watchdog logic runs on the next tick (park, or hold and `Fault(WatchdogTimeout)`).
- **Exceptions.** A thermal trip still disables torque, as in position mode. With a complete bus loss no hold frame reaches the motors: they keep executing the last MIT command (model feed-forward and damping) until the bus returns, and the switch back is retried.
- **Tracking error.** While floating the reference of J1–J6 is set to the measured position every tick, so intended hand motion produces no following error; during the exit the reference is the captured pose and the following-error check applies. Overspeed, hardware fault, feedback and thermal checks stay active.
- **Commands while on.** Servo frames, `jog`, `home`, presets, `unload`, `torque` and the `move_to` / `move_relative` / `grasp` actions return `409 MOTION_BUSY`. Accepted: the mode command, `motion/stop` (exits), E-stop (exits), `action/gripper`, grasp setup, recording (REST).
- **Recording.** Each tick still appends one frame; while floating `action` equals `observation.state` for J1–J6.
- **Observation.** `control_mode` (`position` | `gravity_comp`, any phase) and `gravity_comp`: `supported`, `blocker` (`{code, message}` or null), `phase` (`ramp_in`, `active`, `exiting`, `exit_failed`), `ramp` (0..1), `scale`, `damping_kd`, `tau_ff` (N·m sent to J1–J6), `last_exit` (`{code, message, at}`).
- **rebot-dm defaults** (software values, not measured): `ramp_in_s` 1.0, `scale` 1.0 (override range 0.5–1.2), `damping_kd` [0.4, 0.6, 0.5, 0.2, 0.15, 0.1] (override ≤ 2.0), `torque_limits_nm` [2, 12, 8.5, 3, 1.5, 0.5], `max_speed_rad_s` 1.5, `overspeed_ticks` 3, `hold_kp` [30, 30, 30, 15, 10, 8], `hold_kd` [1, 1, 1, 0.5, 0.3, 0.2], `exit_hold_ms` 100, `entry_max_velocity` 0.05, `feedback_max_age_ms` 100, `joint_limit_margin_rad` 0.05, `ramp_max_deviation_rad` 0.15. The URDF model torque reaches 16.3 N·m on J2 with the arm stretched out horizontally (grid maximum), above the 12 N·m clamp.

状态：已实现，并在仿真机械臂上测试（`tests/test_gravity_comp.rs`、`tests/test_gravity_comp_api.rs`、`tests/test_damiao_mode_switch.rs`）。**未经实机验证。** 代码在 `src/gravity_comp.rs`，参数在 `ArmConfig.gravity_comp`（rebot-dm 已设置，其他 profile 为 `None`，即不支持）。

- **控制律**：J1–J6 用达妙 MIT 模式，kp = 0，阻尼 `damping_kd`，前馈 τ = clamp(URDF 重力力矩 × `scale`, ±`torque_limits_nm`)。夹爪保持 POS_VEL，停在指令位置。
- **模式切换（达妙）**：控制模式是 10 号寄存器（CTRL_MODE：1 = MIT，2 = POS_VEL），用 `0x7FF` 帧 `[id_l, id_h, 0x55, 10, 值 u32 小端]` 写入 RAM；MIT 帧发往 CAN ID = 电机 ID，POS_VEL 帧发往 `0x100 + ID`。切换时不失能电机。帧格式依据达妙 DM_CAN 参考库；寄存器读取已在 B601-DM 上核对（2026-11-09），写入尚未在实机上核对。每个关节的顺序：写 CTRL_MODE，总线上紧接着发新模式下的保持帧，然后最多等待 50 ms 的写入应答（反馈 ID 上的 `[id_l, id_h, 0x55, 10, 值]`）。未应答的关节切回原模式，并同时用 POS_VEL 帧和 MIT 帧保持；仍无法确认的关节此后每次写入都收到两种帧，并锁存故障。第一次切到 MIT 前，驱动读取 PMAX / VMAX / TMAX（21–23 号寄存器），与该关节配置的范围（`damiao_ranges`；rebot-dm：J1–J3 为 ±12.5 rad、±10 rad/s、±28 N·m，J4–J7 为 ±12.5 rad、±30 rad/s、±10 N·m；未设置 `damiao_ranges` 的 profile 所有关节用 ±12.5 / ±30 / ±18）不一致时拒绝进入，因为 TMAX 不同会按比例改变每个力矩指令。反馈也按同一组分关节范围解码。CTRL_MODE 不写入 flash；下一次使能握手会把仍处于其他模式的关节恢复为 POS_VEL，电机断电重启后回到 flash 中的模式。
- **进入条件**（不满足时返回对应错误码）：rebot-dm profile、URDF 校验通过、驱动能切换关节模式（`UNSUPPORTED`）；无急停、无故障；力矩开启且状态为 `Armed`（`NOT_ARMED`）；没有轨迹、servo 流、抓取或收纳（`MOTION_BUSY`）；持有租约（`LEASE_REQUIRED`）；反馈不超过 `feedback_max_age_ms`（`FEEDBACK_STALE`）；各臂关节速度低于 `entry_max_velocity`（`GRAVITY_COMP_NOT_STATIONARY`）；没有关节在限位 `joint_limit_margin_rad` 以内（`GRAVITY_COMP_JOINT_LIMIT`）；能算出模型力矩（`GRAVITY_MODEL_UNAVAILABLE`）。切换失败返回 `GRAVITY_COMP_SWITCH_FAILED`。
- **渐入**：切换帧以 `hold_kp` / `hold_kd` 保持实测位姿，前馈取电机反馈的当前力矩（位置环正在承担的负载）。在 `ramp_in_s` 内刚度降到 0，阻尼过渡到 `damping_kd`，前馈从反馈力矩过渡到模型力矩。渐入期间任一关节偏离进入位姿超过 `ramp_max_deviation_rad` 即退出（`GRAVITY_COMP_SAG`）。
- **退出**：所有退出都先记录实测位姿，立即在该位姿发送 MIT 刚性保持（`hold_kp`、`hold_kd`，前馈不变）；`exit_hold_ms` 后 J1–J6 切回 POS_VEL 保持该位姿。POS_VEL 帧不带力矩，前馈随切换结束，之后由电机位置环承担负载。切回未被应答时继续 MIT 刚性保持，锁存故障 `GRAVITY_COMP_EXIT_FAILED`（POS_VEL 确认之前不能清除），每 0.5 s 重试一次切换。
- **退出原因与代码**（`gravity_comp.last_exit.code`）：`REQUESTED`（模式指令）、`STOPPED`（`/api/v1/motion/stop`）、`ESTOP`、`FAULT`、`BUS_LOSS`、`FEEDBACK_STALE`（超过 `feedback_max_age_ms`）、`WATCHDOG`（超过租约超时没有心跳）、`LEASE_LOST`（释放、过期或被接管）、`TORQUE_LOST`、`GRAVITY_COMP_OVERSPEED`（任一关节超过 `max_speed_rad_s` 连续 `overspeed_ticks` 个 tick，速度取反馈速度与每 tick 位置差分中的较大者）、`GRAVITY_COMP_SAG`、`GRAVITY_COMP_JOINT_LIMIT`、`GRAVITY_MODEL_UNAVAILABLE`。这些都不会断力矩。急停和总线中断在同一次调用中发出保持帧，其他原因在下一个控制 tick 生效（50 Hz 时 ≤ 20 ms）。超速作为退出原因上报，不锁存故障。
- **看门狗**：模式开启期间租约看门狗不启动收纳。心跳中断先退出本模式（`WATCHDOG`，在实测位姿保持）；回到 POS_VEL 后，下一个 tick 由原有看门狗逻辑处理（收纳，或保持并锁存 `Fault(WatchdogTimeout)`）。
- **例外**：过温保护仍会断力矩，与位置模式相同。总线完全中断时保持帧到不了电机，电机继续执行最后一条 MIT 指令（模型前馈加阻尼），直到总线恢复，期间持续重试切回。
- **跟随误差**：漂浮期间 J1–J6 的参考位置每个 tick 取实测值，用手推动不产生跟随误差；退出期间参考位置为记录的位姿，跟随误差检查照常生效。超速、硬件故障、反馈和温度检查一直有效。
- **模式开启时的指令**：servo 帧、`jog`、`home`、预设、`unload`、`torque` 以及 `move_to` / `move_relative` / `grasp` 动作返回 `409 MOTION_BUSY`。可用：模式指令、`motion/stop`（退出）、急停（退出）、`action/gripper`、抓取设置、录制（REST）。
- **录制**：每个 tick 照常追加一帧；漂浮期间 J1–J6 的 `action` 等于 `observation.state`。
- **观测**：`control_mode`（`position` | `gravity_comp`，任何阶段）与 `gravity_comp`：`supported`、`blocker`（`{code, message}` 或 null）、`phase`（`ramp_in`、`active`、`exiting`、`exit_failed`）、`ramp`（0..1）、`scale`、`damping_kd`、`tau_ff`（发给 J1–J6 的前馈，N·m）、`last_exit`（`{code, message, at}`）。
- **rebot-dm 默认值**（软件取值，未实测）：见上方英文列表。URDF 模型在机械臂水平伸直时 J2 力矩达 16.3 N·m（网格最大值），高于 12 N·m 的限幅。

### Hardware verification checklist / 实机核实清单

Ordered from no motion to full use. Keep a hand on the arm and the power cut-off within reach from step 4 on.

1. Arm unpowered or disabled: read registers 10 and 21–23 of every motor (`0x7FF`, `[id, 0, 0x33, rid, 0, 0, 0, 0]`) and compare PMAX / VMAX / TMAX with the driver ranges (±12.5 / ±30 / ±18). If they differ, configure matching ranges before anything else (entry is refused until then).
2. Disabled motor: write CTRL_MODE = 1 and back to 2 on one joint; confirm the acknowledgement layout and that the value reads back.
3. Arm supported at the resting pose, torque on, one joint (J6, lightest load): switch that joint to MIT with a stiff hold and back while logging position; confirm no motion beyond a few mrad and no disable.
4. Same pose: compare the URDF gravity torque with the reported holding torque of J2–J4 in POS_VEL at a few poses; adjust `scale` if needed.
5. Enter the mode with the arm held by hand at a low pose; watch the ramp (`tau_ff`, positions). Check that `GRAVITY_COMP_SAG` / overspeed exits end in a hold.
6. Exit (button, stop, E-stop, unplug the WebSocket client, lease takeover) and measure the sag when the feed-forward ends at the switch back to POS_VEL.
7. Pull the CAN connector briefly while floating (bus loss) with the arm supported; confirm the behaviour after reconnection.
8. Hand-guide over the workspace; tune `damping_kd`, `max_speed_rad_s`; record an episode and check the file.

1. 不上电或失能：读取每个电机的 10 号与 21–23 号寄存器（`0x7FF`，`[id, 0, 0x33, rid, 0, 0, 0, 0]`），将 PMAX / VMAX / TMAX 与驱动范围（±12.5 / ±30 / ±18）比较；不一致时先配置一致的范围（在此之前无法进入本模式）。
2. 电机失能：对一个关节写 CTRL_MODE = 1 再改回 2，确认应答格式和回读值。
3. 机械臂在收纳位有支撑、力矩开启，只对一个关节（负载最小的 J6）：切到 MIT 刚性保持再切回，同时记录位置，确认位移不超过几 mrad 且未失能。
4. 同一位姿：在几个位姿下比较 POS_VEL 时 J2–J4 的反馈保持力矩与 URDF 重力力矩，必要时调整 `scale`。
5. 用手扶住机械臂、在较低位姿进入本模式，观察渐入（`tau_ff`、位置）；确认 `GRAVITY_COMP_SAG` / 超速退出后进入保持。
6. 逐一测试退出（按钮、停止、急停、断开 WebSocket 客户端、被接管），测量切回 POS_VEL、前馈结束时的下沉量。
7. 机械臂有支撑时，在漂浮状态下短暂拔掉 CAN 插头（总线中断），确认恢复连接后的行为。
8. 在工作空间内拖动示教；调整 `damping_kd`、`max_speed_rad_s`；录制一个片段并检查文件。

## Configuration / 配置

`--config` loads a complete `ArmConfig` JSON. Core fields are `dof`, `joint_names`, `limits`, `home_positions`, `lease_timeout_ms`, `max_temperature`, and `control_frequency_hz`; optional fields are `resting_pose`, `following_error`, `max_jerk`, `gravity_model`, `planned_max_velocity`, `kinematics`, `workspace`, `camera_mount`, `gripper_policy`, `grasp`, `gravity_comp` (requires dof 7 and `gravity_model`), and `damiao_ranges` (one `{p_max, v_max, t_max}` per joint; the Damiao feedback decoding and MIT encoding range of each motor). Follower DOF is fixed by model (`rebot-dm=7`; `rebot-rs`, `so-arm101`, `fashionstar`, and `star-arm102=6`), and mismatches are rejected. `--leader` and `--leader-port` are rejected because leader-following is not implemented.

`--config` 加载完整 `ArmConfig` JSON。核心字段为 `dof`、`joint_names`、`limits`、`home_positions`、`lease_timeout_ms`、`max_temperature`、`control_frequency_hz`；可选字段为 `resting_pose`、`following_error`、`max_jerk`、`gravity_model`、`planned_max_velocity`、`kinematics`、`workspace`、`camera_mount`、`gripper_policy`、`grasp`、`gravity_comp`（要求 dof 为 7 且设置了 `gravity_model`）、`damiao_ranges`（每个关节一组 `{p_max, v_max, t_max}`，即该达妙电机的反馈解码与 MIT 编码范围）。follower 的 DOF 按型号固定（`rebot-dm=7`；`rebot-rs`、`so-arm101`、`fashionstar`、`star-arm102=6`），不匹配会拒绝启动。`--leader` 与 `--leader-port` 因主从功能未实现而被拒绝。
