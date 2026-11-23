# Gripper position-force mode (B601-DM J7)

[English](#english) | [中文](#中文)

## English

### What the driver does

Profile block `damiao_gripper` (B601-DM default):

| Field | Default | Meaning |
|---|---|---|
| `mode` | `pos_force` | `pos_force` = Damiao CTRL_MODE 4; `pos_vel` = legacy CTRL_MODE 2, no motor current cap |
| `current_ceiling_raw` | 460 | current limit on every J7 frame, in 1/10000 of the motor's maximum phase current |
| `close_speed_rad_s` | 0.30 | speed limit of closing frames |
| `calibrated` | false | the ceiling's torque equivalent was measured (needs `ceiling_torque_nm`) |
| `ceiling_torque_nm` | none | measured torque equivalent of the ceiling |
| `torque_reference` | none | `external` (independent force / torque reference) or `feedback_equivalent` (motor feedback only) |

Measurements on **2026-11-23** (office B601-DM, J7 `sw_ver 942747703`, build `1.6.0-dev8j7` / `027e46c`, method: motor feedback torque at stall on empty jaws, no external force/torque reference):

- Directly measured stall feedback-torque plateaus:
  - raw 220: `0.42 N·m` (jaw stalls at `−3.6 rad`);
  - raw 300: `0.55–0.57 N·m` (opens to `−5.64 rad`, stalls at `−5.55 rad` closing from full open).
  - Ratio from these two stall plateaus: **≈ 0.0019 N·m per raw unit**.
- Higher ceiling steps on the same unit (with ceilings extrapolated from `0.0019 N·m/raw`; in that run every close from `−5.69 rad` moved smoothly at `~0.30 rad/s` until stopped around `−3.2 rad` by the pre-fix 8.5 s timeout, so full-stroke close to `0.0 rad` was not yet observed):

| `current_ceiling_raw` | Extrapolated ceiling (feedback-equivalent) | Observation (`2026-11-23`, empty jaw) |
|---|---|---|
| 220 | — (measured stall plateau `0.42 N·m`) | stalls at `−3.6 rad` |
| 300 | — (measured stall plateau `0.55–0.57 N·m`) | opens to `−5.64 rad`, stalls at `−5.55 rad` closing from full open |
| 400 | `≈ 0.76 N·m` | opens to `−5.69 rad`; closes at `~0.30 rad/s`, peak `0.76 N·m` (at ceiling), stopped at `−3.21 rad` by 8.5 s timeout |
| 460 | `≈ 0.87 N·m` | opens to `−5.69 rad`; closes at `~0.30 rad/s`, peak `0.79 N·m`, stopped at `−3.19 rad` by 8.5 s timeout |
| 520 | `≈ 0.99 N·m` | opens to `−5.69 rad`; closes at `~0.30 rad/s`, peak `0.75 N·m`, stopped at `−3.19 rad` by 8.5 s timeout |

Earlier `pos_vel` runs measured free-motion torque up to `0.79 N·m` near full open. Default `current_ceiling_raw: 460` (`calibrated: false`) is therefore a **feedback-equivalent ≈ 0.87 N·m estimate**, not an external calibration.

- **Enable.** At a full enable with torque off, the driver drains buffered RX traffic while applying drained telemetry (bound to each motor's own CAN ID or shared master ID) to the feedback cache, then writes J7's CTRL_MODE to 4 in RAM (never flash) only on provably fresh (decode time and receive-queue bound ≤ 100 ms), stationary (|v| ≤ 0.05 rad/s), state-0 feedback. If J7 feedback is stale or unbounded at the mode-write precondition or before the pre-enable hold, the driver sends a `0xCC` refresh for J7 and keeps the handshake pending (within the 50 ms reply deadline, without widening the 100 ms freshness bound), failing closed if no timely post-request reply arrives.
  - It then requires the matching register acknowledgement and a separate readback. The readback request goes out after an observed-empty receive queue; only a read reply dequeued after that, carrying the motor's expected master CAN arbitration ID (configured `master_id`, or `0x10 + motor_id`, i.e. `0x17` for J7) and `RID_CTRL_MODE`, and arriving strictly before the 50 ms deadline counts.
  - Every later full enable (including after a host restart with J7 already in mode 4, verified during startup range checks only from post-boundary replies on the motor's expected master CAN ID) reads the mode back again, since a motor power cycle resets the RAM mode. Once readback confirms mode 4, the driver sends a capped `0x307` hold at the freshly measured J7 position (≤ `close_speed_rad_s`) before `CMD_ENABLE` so the motor cannot resume a stale firmware target. The already-enabled fast path also requires verified mode 4 and fresh ≤ 100 ms J7 feedback.
  - Any failure fails the enable: wrong value, wrong CAN ID or register ID, reply at or after the 50 ms deadline, missing/unbounded fresh measured position, or a request/hold that was not sent. Capped operation is then unavailable: target batches are refused before their first frame, and J7 never receives a frame of another mode.
- **Final writer.** Every J7 frame is `0x300 + id`: position f32, speed u16 in 0.01 rad/s (quantized downward via `f64` floor so wire speed never exceeds the limit), current u16 = `current_ceiling_raw`. This covers targets, holds, pre-enable holds, measured-position recovery holds, mode-switch holds, servo streams and policies.
  - The speed field is the requested speed (`JointTarget::velocity`, else the POS_VEL ceiling, at most 4 rad/s).
  - Speed may exceed `close_speed_rad_s` only when provably fresh (≤ 100 ms) finite feedback proves the frame moves the jaw strictly toward `open_rad`. Closing frames, holds (target == measured), explicit fast closes, and recovery holds stay capped at `close_speed_rad_s`. Once J7 feedback exceeds 100 ms (at batch entry, mid-batch, or across a `WouldBlock` retry) or a J7 target position/velocity is non-finite, stale-feedback recovery emits no J7 motion frame (read-only `0xCC` refresh only), `write_joint_targets` refuses to transmit a J7 frame, and enabled mode handovers back to `PosForce` fail before writing `CTRL_MODE`.
- **E-stop hold.** `StopPolicy::EStopLatch` locks onto `measured_positions` only when every joint reading is finite and the supervisor's own coherent feedback snapshot (`core.feedback()` decode instants and, for receive-queued drivers, `joint_feedback_queue_bounds`, evaluated against `driver.clock_now()` via non-blocking `try_lock`) is provably within the 250 ms feedback freshness bound; when the driver mutex is contended or that snapshot is stale, unread past 250 ms, or lacks a receive-queue bound (even if a driver-side drain later updated the driver's cache), E-stop completes without waiting on a lock and holds `commanded_positions` instead.
- **Unchanged.** J1–J6 frames are byte-identical to the legacy profile (test `final_writer_caps_every_j7_frame_and_j1_j6_frames_are_unchanged`). The host-side lead cap, contact regulator and software torque cap remain as the second layer.
- **Reporting.** `GET /api/v1/safety/describe` → `hardware.gripper_current_cap` returns:
  - `mode`, `current_raw`, `close_speed_rad_s`;
  - `active`: the mode was read back at the last enable;
  - `calibrated`, `ceiling_torque_nm`, `torque_reference`, `note`.

  The legacy profile reports `"no motor current cap (legacy POS_VEL gripper)"`.

Not yet verified on the device: full-stroke close to `0.0 rad` at raw 460 after the close-speed timeout fix, external force/torque calibration at the jaws, readback after a host/motor power-cycle restart, holding after a slip, and CAN loss while holding (`TIMEOUT=0`).

### Commissioning: `current_ceiling_raw` to N·m

Prerequisites:
- a guarded empty-jaw fixture;
- an accessible power cut;
- an independent force or torque reference at the jaws;
- a fixture-rated upper load and an independent abort threshold, set before energizing;
- the operator at the arm.

1. **Mode check.** With torque off, enable once. Confirm that `hardware.gripper_current_cap.active` is true and that the logs show "confirmed in position-force mode".
2. **Free travel.** At the lowest validated nonzero current that moves the jaw, open and close the empty jaw at 0.1, 0.3 and 0.5 rad/s. Record the measured speed against each setpoint, and the decoded torque.
3. **Stepped current on empty jaws.** From just short of jaw contact (or at the fixture), run brief bounded closes with stepped `current_ceiling_raw`. Start at the lowest value that moved the jaw in step 2 and increase in small steps (for example +20 raw), unloading between steps.
   - Record per step: raw command, speed field, jaw position, decoded torque, external load, peak and settled plateau.
4. **Stop conditions.** Stop on unexpected motion, stale feedback, heating, or reaching the fixture load bound.
5. **Repeat** at several jaw positions across the stroke.
6. **Select** the highest tested raw value whose upper measured torque, uncertainty included, stays within 1.0 N·m (or the gripper's rated limit if lower).
7. **Record it in the profile:**
   - `current_ceiling_raw`, `calibrated: true`, `ceiling_torque_nm`;
   - `torque_reference: "external"`. Without an external reference, use `"feedback_equivalent"` and label the result "feedback-equivalent torque".

A demonstrated motor current ceiling limits active drive effort. A bound on fingertip impact also needs measurements at the jaws across the supported speeds and geometries.

### Not handled here (gripper-baseline branch)

`feat/gripper-baseline` needs to adapt:
- **Baseline identity:** bind the learned baseline to `(mode, current_ceiling_raw, calibrated)`, and invalidate it when any of them changes.
- **Saturation verdict:** sustained clipping at the ceiling is inconclusive for free-motion learning, even when the software guard never trips. Keep torque-plus-gap and following-error evidence separate from saturation, because an empty jaw against its stop can draw capped current.
- **Zero check current:** zero check needs enough current to overcome free friction. Reject an early stall outside the expected stop window and leave the stored zero unchanged.

## 中文

### 驱动行为

Profile 块 `damiao_gripper`（B601-DM 默认）：

| 字段 | 默认 | 含义 |
|---|---|---|
| `mode` | `pos_force` | `pos_force` = 达妙 CTRL_MODE 4；`pos_vel` = 旧版 CTRL_MODE 2，无电机电流上限 |
| `current_ceiling_raw` | 460 | 每个 J7 帧的电流上限，单位为电机最大相电流的 1/10000 |
| `close_speed_rad_s` | 0.30 | 闭合帧的速度上限 |
| `calibrated` | false | 上限对应的力矩已实测（需要 `ceiling_torque_nm`） |
| `ceiling_torque_nm` | 无 | 上限对应的实测力矩 |
| `torque_reference` | 无 | `external`（独立力 / 力矩参考）或 `feedback_equivalent`（仅电机反馈） |

**2026-11-23** 实机测量记录（办公室 B601-DM，J7 `sw_ver 942747703`，构建 `1.6.0-dev8j7` / `027e46c`，方法：空夹爪堵转时的电机反馈力矩，无外部力/力矩参考）：

- 直接测得的堵转反馈力矩平台：
  - raw 220：`0.42 N·m`（夹爪在 `−3.6 rad` 处堵转）；
  - raw 300：`0.55–0.57 N·m`（可完全张开至 `−5.64 rad`，从全开闭合时在 `−5.55 rad` 处堵转）。
  - 由以上两档堵转平台折算比例为 **≈ 0.0019 N·m / raw**。
- 同一台设备上的更高档位测试（上限按 `0.0019 N·m / raw` 推算；该次测试中各档位从 `−5.69 rad` 闭合时均以约 `0.30 rad/s` 平稳运动，但受修复前的 8.5 s 超时限制在约 `−3.2 rad` 处停止，完整闭合至 `0.0 rad` 尚待修复后复测）：

| `current_ceiling_raw` | 推算上限（反馈等效） | 观测结果（`2026-11-23`，空夹爪） |
|---|---|---|
| 220 | —（实测堵转平台 `0.42 N·m`） | 在 `−3.6 rad` 处堵转 |
| 300 | —（实测堵转平台 `0.55–0.57 N·m`） | 可张开至 `−5.64 rad`，全开闭合在 `−5.55 rad` 处堵转 |
| 400 | `≈ 0.76 N·m` | 张开至 `−5.69 rad`；闭合速度 `~0.30 rad/s`，峰值 `0.76 N·m`（触及上限），因 8.5 s 超时停于 `−3.21 rad` |
| 460 | `≈ 0.87 N·m` | 张开至 `−5.69 rad`；闭合速度 `~0.30 rad/s`，峰值 `0.79 N·m`，因 8.5 s 超时停于 `−3.19 rad` |
| 520 | `≈ 0.99 N·m` | 张开至 `−5.69 rad`；闭合速度 `~0.30 rad/s`，峰值 `0.75 N·m`，因 8.5 s 超时停于 `−3.19 rad` |

此前 `pos_vel` 测试在接近全开位置测得自由运动力矩最高 `0.79 N·m`。因此默认 `current_ceiling_raw: 460`（`calibrated: false`）属于**反馈等效约 0.87 N·m 的估算值**，并非外部标定结果。

- **使能。** 完整使能时，驱动先清空接收队列并将排出的遥测帧（按各电机自身 CAN ID 或共享主站 ID 绑定）更新到反馈缓存，再在力矩关闭状态下把 J7 的 CTRL_MODE 写为 4（RAM，不写 flash）；前提是反馈可证新鲜（解码时间与接收队列边界均 ≤ 100 ms）、静止（|v| ≤ 0.05 rad/s）且状态为 0。若写模式前置检查或使能前保持检查时 J7 反馈不新鲜或无队列边界，驱动发送 J7 的 `0xCC` 刷新并保持握手挂起（沿用 50 ms 应答截止时间，不放宽 100 ms 新鲜度窗口），截止前未收到请求后新鲜回复则失效关闭。
  - 写入后要求收到匹配的寄存器应答，并单独回读。回读请求在观察到接收队列为空之后发出，仅接受此后出队、来源 CAN ID 等于电机预期主站 ID（配置的 `master_id`，或默认 `0x10 + motor_id`，J7 为 `0x17`）、寄存器号为 `RID_CTRL_MODE` 且严格在 50 ms 截止时间前到达的读回复。
  - 之后每次完整使能（含主机重启时 J7 已处于模式 4 的情况；启动量程检查仅凭接收边界后来自电机预期主站 CAN ID 的回复确认模式 4）都会重新回读，因为电机断电会复位 RAM 中的模式。回读确认模式 4 后，驱动先在新鲜实测的 J7 位置发出一帧受限 `0x307` 保持帧（速度 ≤ `close_speed_rad_s`），再发 `CMD_ENABLE`，避免电机恢复旧的固件目标。已使能快速路径同样要求模式 4 已验证且 J7 反馈 ≤ 100 ms 新鲜。
  - 任一失败都使使能失败：值不符、CAN ID 或寄存器号不符、在 50 ms 截止时间及之后到达、缺少有界新鲜实测位置、请求或保持帧未发出。此时受限运行不可用：目标批次在第一帧之前被拒，J7 不会收到其他模式的帧。
- **最终写入。** 每个 J7 帧都是 `0x300 + id`：位置 f32、速度 u16（0.01 rad/s，按 `f64` 向下取整确保线速不超过上限）、电流 u16 = `current_ceiling_raw`。覆盖目标、保持、使能前保持、实测位置恢复保持、模式切换保持、servo 流和策略。
  - 速度字段为请求速度（`JointTarget::velocity`，否则为 POS_VEL 上限，最大 4 rad/s）。
  - 仅当可证新鲜（≤ 100 ms）且有限的反馈证明帧将夹爪严格移向 `open_rad` 时才允许超过 `close_speed_rad_s`。闭合帧、保持帧（目标等于实测）、显式快速闭合及恢复保持均限制在 `close_speed_rad_s` 以内。J7 反馈超过 100 ms（含批次入口、批次中途或 `WouldBlock` 重试期间）或目标位置/速度非有限时，陈旧反馈恢复不再发送 J7 运动帧（仅发只读 `0xCC` 刷新），`write_joint_targets` 拒绝发送 J7 帧，且使能状态下切回 `PosForce` 的模式切换在写 `CTRL_MODE` 之前失败。
- **急停保持。** `StopPolicy::EStopLatch` 仅在所有关节实测位置有限且监管器自身一致性反馈快照（与 `measured_positions` 同步记录的 `core.feedback()` 解码时间以及带接收队列驱动的 `joint_feedback_queue_bounds`，经非阻塞 `try_lock` 按 `driver.clock_now()` 判定）可证在 250 ms 反馈新鲜度上限内时锁定到 `measured_positions`；若驱动互斥锁被占用，或快照陈旧、在队列中积压超 250 ms 或无接收队列边界（即使之后驱动侧排空刷新了驱动缓存），急停不等待锁，直接改用 `commanded_positions` 保持。
- **不变部分。** J1–J6 帧与旧版 profile 逐字节相同（测试 `final_writer_caps_every_j7_frame_and_j1_j6_frames_are_unchanged`）。主机侧领先量上限、接触调节和软件力矩上限保留为第二层。
- **上报。** `GET /api/v1/safety/describe` → `hardware.gripper_current_cap` 返回：
  - `mode`、`current_raw`、`close_speed_rad_s`；
  - `active`：上次使能时已回读到该模式；
  - `calibrated`、`ceiling_torque_nm`、`torque_reference`、`note`。

  旧版 profile 上报 `"no motor current cap (legacy POS_VEL gripper)"`。

尚未在设备上验证：闭合超时修复后 raw 460 全行程闭合至 `0.0 rad`、夹爪处外部力/力矩标定、主机或电机断电重启后的回读、打滑后的保持、保持期间 CAN 断开（`TIMEOUT=0`）。

### 标定：`current_ceiling_raw` 对应 N·m

前提：
- 有防护的空夹爪工装；
- 可随时断电；
- 夹爪处有独立的力或力矩参考；
- 上电前设定好工装额定的负载上限和独立的中止阈值；
- 操作员在机械臂旁。

1. **模式检查。** 力矩关闭时使能一次。确认 `hardware.gripper_current_cap.active` 为 true，日志中出现 "confirmed in position-force mode"。
2. **自由行程。** 以能驱动夹爪的最低已验证非零电流，在 0.1、0.3、0.5 rad/s 下张开、闭合空夹爪。记录各设定值对应的实测速度，以及解码力矩。
3. **空夹爪电流阶梯。** 从刚好未接触（或工装位置）开始，按阶梯调整 `current_ceiling_raw`，做短时、有界的闭合。从第 2 步中能驱动夹爪的最低值开始，小步增加（例如每步 +20 raw），每步之间卸载。
   - 每步记录：raw 指令、速度字段、夹爪位置、解码力矩、外部载荷、峰值和稳定平台值。
4. **停止条件。** 出现意外运动、反馈陈旧、发热或达到工装载荷上限时停止。
5. **重复**：在行程内多个夹爪位置重复。
6. **选定**：取测得力矩上界（含不确定度）不超过 1.0 N·m（或夹爪额定值，取较小者）的最高测试 raw 值。
7. **写入 profile：**
   - `current_ceiling_raw`、`calibrated: true`、`ceiling_torque_nm`；
   - `torque_reference: "external"`。没有外部参考时用 `"feedback_equivalent"`，结果标注为“反馈等效力矩”。

已验证的电机电流上限只限制主动驱动出力。要给出指尖冲击的上界，还需要在支持的速度和几何条件下于夹爪处实测。

### 不在此处处理（gripper-baseline 分支）

`feat/gripper-baseline` 需要适配：
- **基线身份：** 学习到的基线绑定 `(mode, current_ceiling_raw, calibrated)`，任一变化即失效。
- **饱和判定：** 持续在上限处削顶对自由运动学习属于不确定，即使软件保护从未触发。力矩加间隙证据、跟随误差证据与饱和分开，因为空夹爪顶在止挡上也可能达到上限电流。
- **零点检查电流：** 零点检查需要足够电流克服自由摩擦。在预期止挡窗口之外的过早堵转应拒绝，并保持已存零点不变。
