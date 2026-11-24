# 电机控制模式抽象

[English](motor_modes.md) | 中文

`armd` 按**指令语义**（`ControlMode`）对执行器控制建模，而非按电机型号或关节序号硬编码。各厂商的编解码器、模式切换握手与反馈能力在 profile 加载阶段绑定，且全部校验在打开底层传输之前完成。

> **第 1 步范围（当前分支）：** 本阶段仅落地 `ControlMode`、按模式区分的类型化限位、profile 字段与加载期校验、缺省字段向后兼容翻译、厂商能力表，以及达妙纯编解码函数（`encode_command` / `decode_feedback_bytes`）和单元测试。`src/driver.rs` 中的发送、使能、失能、保持与 emitter 路径暂不切换（见下文第 2–5 步迁移计划）。

---

## 1. 控制模式语义模型

核心类型定义在 `src/motor_mode.rs`（同时支持原生目标与 `wasm32-unknown-unknown` 编译），并由 `src/model.rs` 与 `src/lib.rs` 重导出。

| `ControlMode`（JSON） | 控制语义 | 类型化限位结构 | 必填模式限位 |
|---|---|---|---|
| `position` | 位置目标 | `PositionLimits` | 无必填项（可选 `max_velocity_rad_s`、`step_range` 范围 `0..=4095`、`interval_ms >= 1`、`power_limit >= 1`） |
| `position_velocity` | 位置目标 + 逐帧速度上限 | `PositionVelocityLimits` | `max_velocity_rad_s`（有限正数，且不超过厂商线协议 `v_max`） |
| `impedance` | 位置 + 速度 + `kp` + `kd` + 前馈力矩（MIT 阻抗） | `ImpedanceLimits` | `p_max`、`v_max`、`t_max`（有限正数对称映射范围），`kp_max`（`0..=500`），`kd_max`（`0..=5`），可选 `effort_limit_nm`（`(0, t_max]`） |
| `position_force` | 位置目标 + 逐帧速度上限 + 电机相电流上限 | `PositionForceLimits` | `current_ceiling_raw`（`1..=10000`），`close_speed_rad_s`（向下量化后 `>= 0.01 rad/s` 且 `<= 655.35 rad/s`），可选 `max_speed_rad_s`、`calibrated` / `ceiling_torque_nm` / `torque_reference`、`feedback_stale_fault_ms`（`200..=450`） |

### Profile 配置格式（`ArmConfig::control_mode`）

Profile 可声明可选顶层字段 `control_mode`（长度为 `dof` 的数组）。每个元素可为 `null`（由既有 follower 与 profile 配置自动翻译）或以下对象：

```json
{
  "control_mode": [
    {
      "mode": "position_velocity",
      "vendor": "damiao",
      "runtime_modes": ["impedance"],
      "limits": { "max_velocity_rad_s": 2.0 }
    },
    null,
    null,
    null,
    null,
    null,
    {
      "mode": "position_force",
      "vendor": "damiao",
      "runtime_modes": [],
      "limits": {
        "current_ceiling_raw": 460,
        "close_speed_rad_s": 0.3,
        "calibrated": false
      }
    }
  ]
}
```

- `limits` 严格按所声明的 `mode` 解析（`deny_unknown_fields`）；仅 `position` 允许省略 `limits`。
- `vendor` 可选；显式填写时在加载期对照当前 follower 与厂商能力表校验。
- `runtime_modes` 可选：省略时继承既有 `gravity_comp` / `joint_tracking` 的运行时切换权限；显式设为 `[]` 表示禁止该关节在运行时切换模式。

### 加载期校验（传输打开之前）

`src/main.rs`（`resolve_follower_config`）在绑定网络端口或打开 CAN / 串口传输之前调用 `ArmConfig::validate()` 与 `ArmConfig::resolve_control_modes_for_follower()`，拒绝以下配置：

- 长度与 `dof` 不一致；
- 缺少模式必填限位，或限位变体与 `mode` 不匹配；
- 厂商能力表不支持的 `(vendor, mode)` 或 `(vendor, base_mode -> runtime_mode)` 绑定；
- `control_mode` 与既有字段（`damiao_gripper`、`damiao_ranges`、`feetech`、`gravity_comp`、已启用的 MIT `joint_tracking`）冲突，或不同关节 / follower 间的 `vendor` 冲突；
- 非法线协议范围（非有限浮点数、关节限位超出 `[-p_max, p_max]` 或 FashionStar `i16` 0.1° 范围、`kp_max > 500`、`kd_max > 5`、`current_ceiling_raw` 超出 `1..=10000`、`POS_FORCE` 速度量化为 `0` 或超过 `655.35 rad/s`、12-bit `step_range` 超出 `0..=4095`，以及跨厂商误填字段如 Feetech 填 `interval_ms`/`power_limit` 或 FashionStar 填 `step_range`）。

### 缺省字段翻译与 `damiao_gripper` 兼容读取

当 `control_mode`（或其中某关节项）省略时，解析结果与现有全部随仓 profile（`kits/*/profile/*.json`、`profiles/*.json`）的行为逐项一致：

| Follower / Profile | 手臂关节 | 夹爪关节 | 运行时切换 |
|---|---|---|---|
| Damiao（`rebot-dm`，含 `damiao_gripper.mode = pos_force` 的 B601-DM） | `position_velocity`（`2.0 rad/s`） | `position_force`（兼容读取自 `damiao_gripper`：raw `460`、`0.30 rad/s`） | 配置了 `gravity_comp` 或已启用 MIT `joint_tracking` 的手臂关节允许切到 `impedance` |
| Damiao（旧版 B601-DM：无 `damiao_gripper` 或 `mode = pos_vel`） | `position_velocity`（`2.0 rad/s`） | `position_velocity`（`2.0 rad/s`） | 同上 |
| Feetech（`so-arm100`、`so-arm101`） | `position`（步数范围来自 `feetech.joints[i]`） | `position`（步数范围来自 `feetech.joints[i]`） | 无 |
| FashionStar（`fashionstar`、`star-arm102`） | `position`（`interval_ms = 20`、`power_limit = 1000`） | 无 | 无 |
| RobStride（`rebot-rs`，B601-RS） | `impedance`（`±4π rad`、`±30 rad/s`、`±18 N·m`、`kp <= 500`、`kd <= 5`） | 无 | 无 |

`ArmConfig::damiao_gripper` 保持为旧 profile 的兼容读取入口；同时提供 `ArmConfig::effective_damiao_gripper()` 统一读取 `damiao_gripper` 或 `control_mode` 声明的 `PositionForce` 夹爪参数。

### 达妙纯编解码抽离（第 1 步）

`src/protocol/damiao.rs` 新增纯函数、零分配（`Copy`）编解码接口，不改动现有驱动发送路径，也不削弱 `build_pos_force_frame` 的 `PosForceSendToken` 编译期封闭约束：

- `encode_command(motor_id, mode, payload, limits) -> Result<DamiaoEncodedFrame, DamiaoEncodeRefusal>`：将 `PositionVelocity`（`0x100 + id`）、`Impedance`（`id`）、`PositionForce`（`0x300 + id`）编码为 `{ can_id: u16, data: [u8; 8] }`；对 `ControlMode::Position`、模式/限位不匹配、非法 CAN ID、非有限值、非法限位、运动目标线速量化为 `0`、电流上限为 `0` 及非零速度保持帧全部 fail-closed 拒绝。
- `decode_feedback_bytes(data, limits) -> Result<DamiaoFeedback, DamiaoError>`：纯字节解码 8 字节达妙反馈帧，与 `unpack_feedback_frame` 逐字段一致。

---

## 2. 厂商能力表

完全依据仓库内 `src/protocol/*.rs` 与 `src/driver.rs` 整理（对应代码常量 `VENDOR_CAPABILITIES`，位于 `src/motor_mode.rs`）。仓库内未证实的协议能力或时序保证均标注为**需核实厂商文档（needs vendor-doc check）**。

| 厂商 | 协议源码 | `armd` 已支持的 `ControlMode` | 默认基础模式 | 模式选择与回读 | 运行时切换 | 反馈来源与运行时字段 | 需核实厂商文档（needs vendor-doc check） |
|---|---|---|---|---|---|---|---|
| **Damiao** | `src/protocol/damiao.rs` | `position_velocity`（`CTRL_MODE = 2`）、`impedance`（`CTRL_MODE = 1`）、`position_force`（`CTRL_MODE = 4`） | `position_velocity` | RAM 寄存器写（`0x7FF`、`0x55`、`RID_CTRL_MODE = 10`）+ 匹配 ACK + 独立回读（`0x33`）。J7 `POS_FORCE` 使能要求先具备新鲜（`<= 100 ms`）、静止（`|v| <= 0.05 rad/s`）、状态为 0 的反馈，写入并回读确认后再发零速预使能保持帧（见 `docs/gripper_pos_force.md`）。 | 手臂关节 `position_velocity <-> impedance` | 指令回复 + 只读 `0xCC` 刷新（`0x7FF`）。解码 `position`、`velocity`、`torque`、`t_mos`、`t_rotor`、`state_or_error`。 | `CTRL_MODE = 3`（VEL）帧格式与语义；各型号电机的模式支持矩阵；不同模式与固件下的反馈回复节奏与可靠性；`RID_TIMEOUT`（寄存器 9）单位（文档记为 `50 us`）、超时延迟及在 B601-DM 固件（`sw_ver 942747703`）上的实际响应。 |
| **Feetech STS3215** | `src/protocol/feetech.rs` | `position`（`INST_SYNC_WRITE` `0x83` 写 `REG_GOAL_POSITION` `0x2A`） | `position` | 上线配置读取 `REG_OPERATING_MODE`（`0x21`），不一致时在力矩关闭且解锁 `REG_LOCK = 0x37` 后写入 `0` 并回读核验。 | 否 | 有界分块 `INST_SYNC_READ`（`0x82`，自 `0x38` 读 8 字节）：`position`、`speed`、`load`、`voltage`、`temperature`（由负载折算的力矩仅作估算，非实测力矩）；`REG_TORQUE_ENABLE`（`0x28`）单独回读。 | `REG_OPERATING_MODE`（`0x21`）轮式/速度模式与 PWM 模式的寄存器值与指令语义；`REG_PRESENT_CURRENT_L`（`0x45`）的量纲与延迟（协议已定义，但不在当前 8 字节轮询内）。 |
| **RobStride** | `src/protocol/robstride.rs` | `impedance`（`TYPE_MOTION_CONTROL = 1` 29 位扩展 CAN 帧：载荷为 4 个 16-bit 大端 `pos`/`vel`/`kp`/`kd`，CAN ID 位 23..8 为 16-bit `torque`） | `impedance` | 固定运动控制帧类型（`TYPE_ENABLE = 3`、`TYPE_STOP = 4`）。协议已声明 `TYPE_PARAM_READ = 17` 与 `TYPE_PARAM_WRITE = 18` 常量。 | 否 | 排空 `TYPE_FEEDBACK = 2` 29 位扩展 CAN 反馈帧：`position`、`velocity`、`torque`、`mos_temperature`（`0.1 °C`）、`fault_flags`。 | 位置、速度、电流模式及其选择寄存器和经 `TYPE_PARAM_READ` / `TYPE_PARAM_WRITE` 的回读；Type-2 反馈应答配对、周期上报节奏与电流遥测；`TYPE_ENABLE` / `TYPE_STOP` 是否产生因果力矩状态证据；`TYPE_SET_ZERO`（6）掉电保持与保存参数序列（type 22）。 |
| **FashionStar** | `src/protocol/fashionstar.rs` | `position`（`CMD_SET_SERVO_ANGLE = 0x08`：`0.1°` `i16`、`interval_ms = 20`、`power = 1000`；独立 `CMD_SET_TORQUE = 0x1A`） | `position` | 隐式角度指令。 | 否 | 轮询发送 `CMD_QUERY_SERVO_ANGLE`（`0x0A`），仅更新 `position`。协议另定义了 `CMD_QUERY_STATUS`（`0x1E`）解码器（电压、温度、电流、状态），运行时主循环未轮询。 | 模式选择/回读及非位置模式；`CMD_SET_SERVO_ANGLE`（`0x08`）应答语义与 `power = 0` 含义；速度与力矩遥测支持情况。 |

---

## 3. 迁移计划（第 2–5 步）

后续按小 PR 渐进替换，每个 PR 须满足：

- `test_ws_replay` 对基线回放语料库 **0 差异**；
- 每条迁移路径保持字节级与帧顺序等价；
- `armd-control` 线程严格零堆分配、零 `SupervisorLock` 尝试（由 `test_rt_*_alloc` 与 `test_rt_lock_proof` 守护）；
- 凡涉及停止优先级或力矩证据语义变更（`OffConfirmed` / `OnConfirmed` / `Unknown` 与单一 `send(DisableRequested)` 关口），须与纯等价重构拆开独立评审。

### 第 2 步：达妙驱动统一（`POS_VEL`、`MIT`、`POS_FORCE`）

- 将 J7 单发射器模式（`pos_force_emitter`）泛化为固定容量的全关节逐周期 emitter，在 Legacy 与 RT Executor 双控制所有者上共用：所有者仅写入逐关节意图（`Target`、`Hold`、`Stop`、受限条件下的 `Freeze`；无有效证据时不发帧并上报 unheld），emitter 调用 `protocol::damiao::encode_command` 编码。
- 承重关节的 `Stop` / `Hold` 严格优先于 `0xCC` 刷新与夹爪任务，同时保持握手顺序（写 `CTRL_MODE` -> ACK -> 回读 -> 预使能保持 -> 使能）。
- 发送瞬间复查截止时间、TX 预算、停止代数、已验证模式与反馈新鲜度，丢弃被新意图覆盖的旧工作。
- 将期望模式与已验证模式分开跟踪（未验证切换阻断目标输出）；力矩证据（`OffConfirmed` / `OnConfirmed` / `Unknown`）与模式独立维护。
- **删除的特判代码：** `src/driver.rs` 中重复的 `POS_VEL` / `MIT` 目标与保持构帧路径及夹爪专用运行时模式分发。
- **保留：** `damiao_gripper` 兼容读取、`PosForce` 100 ms 新鲜度与启动回退语义、以及模式切换回读校验。

### 第 3 步：Feetech STS3215 迁移

- 将 `FeetechArmDriver` 的目标与保持写入迁入统一逐周期 emitter，保留调用方固定缓冲区内的单包 `INST_SYNC_WRITE` 串口合批（零堆分配）。
- **删除的特判代码：** `FeetechArmDriver` 内独立拼装位置写包的路径。
- **保留：** `configure_servos` 上线配置（`REG_OPERATING_MODE = 0x21` 在 `REG_LOCK` 下的读/写/核验）、串口 TX 回显过滤与分块 `SYNC_READ` 轮询。

### 第 4 步：RobStride 迁移

- 以 `ControlMode::Impedance` 与 `ImpedanceLimits` 将 `RobStrideArmDriver` 的直接 Type-1 目标发送循环切换为逐周期 emitter。
- **删除的特判代码：** 直接逐关节发送 Type-1 帧的循环。
- **保留：** `src/protocol/robstride.rs` 扩展 CAN 编解码与 Type-2 反馈排空逻辑。

### 第 5 步：FashionStar 迁移

- 将 `FashionStarArmDriver` 的直接角度发送循环切换为逐周期 emitter，并将 `src/protocol/fashionstar.rs` 的组包改为写入调用方缓冲区（消除热路径 `Vec` 分配）。
- **删除的特判代码：** 直接角度写循环与热路径堆分配组包调用。
- **保留：** 请求/应答帧头区分（`[0x12, 0x4C]` 与 `[0x05, 0x1C]`）与轮询角度查询匹配。

---

## 4. 验证矩阵

### 模拟 / 仿真（CI）

- 在 Legacy 与 RT Executor 双所有者下覆盖陈旧/缺失 RX、错误模式回读、构帧期间 STOP、部分 TX 背压，以及单关节不可用并存健康承重关节等场景。
- 断言精确线字节、帧顺序、力矩与模式证据转换、截止时间以及 `armd-control` 零堆分配和零锁尝试。

### 真机台架验证

| 目标硬件 | 必须完成的实机核验 |
|---|---|
| **B601-DM（办公室样机）** | 采集 `POS_VEL`、`MIT`、`POS_FORCE` 三模式的 CAN 帧与运动轨迹；核验总线拥塞下的带载保持；核验 J7 `0xCC` 刷新/冻结及外部力矩标定（`docs/gripper_pos_force.md` 当前堵转数据仅基于电机自身反馈）。 |
| **SO-ARM101（Feetech STS3215）** | 核验 `REG_OPERATING_MODE` 与 `REG_TORQUE_ENABLE` 回读、串口回显、sync-read 轮询节奏及带载保持/停止响应。 |
| **B601-RS（RobStride）** | 核验接线方向与关节限位（`profiles/rebot_b601_rs.json` 尚未经实机验证）、Type-2 实际反馈时序，以及 `TYPE_ENABLE` / `TYPE_STOP` 是否提供因果力矩状态证据。 |
| **FashionStar** | 在独立台架完成角度应答、功率/时间间隔语义与遥测核验后再声明硬件等价。 |
