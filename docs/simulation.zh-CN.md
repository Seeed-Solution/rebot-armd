# 仿真

[English](simulation.md) | 中文

`armd --follower sim` 让同一个 armd 二进制连接 reBot B601-DM 的 MuJoCo 物理仿真，而不是 CAN 总线。SDK、MCP server 和 cockpit 照常连接 armd；armd 自身的检查（关节限位、工作空间、租约看门狗、夹爪堵转 / 夹持判定、残差力矩、总线丢失检测，以及含碰撞守护的分支上的碰撞守护）基于仿真的位置、速度和力矩运行。

## 运行

需要 Rust 工具链和 [uv](https://docs.astral.sh/uv/)。仿真器是 `sim/mujoco` 下的 uv 项目（依赖 `mujoco`、`numpy`）。

```bash
# 终端 1：仿真器（500 Hz 物理，UDP 47600）
cd sim/mujoco
uv run rebot-sim --port 47600 --headless

# 终端 2：armd，使用 B601-DM profile
cargo run --bin armd -- --follower sim --follower-port udp://127.0.0.1:47600 \
  --config kits/recamera-b601dm/profile/rebot-b601dm.json --log-dir off --blackbox-dir off
```

cockpit 地址 `http://127.0.0.1:8765/`。`--follower-port` 默认 `udp://127.0.0.1:47600`；不带 `--config` 时使用内置 `rebot-dm` profile。没有 `/userdata` 的机器上，把 `ARMD_POSES`、`ARMD_KEEPOUT`、`ARMD_GRASP_SETUP` 设为可写路径。

`rebot-sim` 选项：

| 选项 | 作用 |
|---|---|
| `--port`、`--host` | UDP 地址（默认 `127.0.0.1:47600`） |
| `--headless` | 不开窗口（默认） |
| `--viewer` | MuJoCo 窗口，需要显示环境（`DISPLAY` / `WAYLAND_DISPLAY`） |
| `--stats S` | 每 S 秒打印步频、关节位置和方块位置 |
| `--duration S` | S 秒后退出 |
| `--benchmark N` | 不按实时节拍跑 N 步物理并计时后退出 |

`REBOT_DEVARM_DIR=<reBot-DevArm 仓库路径>` 时，把上游连杆网格（`Rebot_Arm_description/DM/meshes/collision`）作为可视几何加入窗口显示；物理计算不使用它们。

## 测试

```bash
cargo test --lib sim_driver            # 驱动 + 假 UDP 对端
cargo test --test test_sim_fk          # armd 正运动学 vs MuJoCo end_link 位姿
cd sim/mujoco && uv run pytest -s      # 仿真器 + armd 端到端，FK fixture 是否最新
```

端到端测试默认先 `cargo build`，设置 `ARMD_BIN` 时直接用该二进制。流程：上使能、回 home、张开夹爪、工具朝下 `move_to`、检查 J2 / J3 重力力矩、下降到 4 cm 方块、`grasp` 闭合（期望 `holding: true`）、抬起、杀掉仿真器，期望故障码 `BUS_LOSS`。

## 仿真内容

- **场景**（`sim/mujoco/models/rebot_b601dm_scene.xml`）：关节链与连杆惯量取自 `assets/reBot-DevArm_gripper.urdf`（armd 重力模型和运动学用的同一组参数）；碰撞体为按上游 DM 网格尺寸设置的方盒；桌面在 z = 0（底座平面，对应 profile `grasp.table_z_m`）；4 cm、30 g 方块位于 (0.30, 0, 0.02) m。连杆与桌面、方块碰撞，连杆之间不碰撞。
- **关节约定**：J1..J6 电机角 = URDF 角。J7 经齿条驱动两指：单侧开口 = −q7 × 0.0425 / 5.7 m（q7 = 0 闭合，−5.7 rad = 85 mm）。另一指由等式约束跟随。J2、J3 上限位在 0 rad（URDF 限位），机械臂在 q = 0 折叠静置；其余关节范围为 profile 限位 + 0.1 rad。
- **电机**（物理 500 Hz）：
  - POS_VEL（armd 常规模式）：v_des = clip(25 · (q_target − q), ±vlim)，力矩 = kv · (v_des − dq) + J1..J6 的模型偏置力矩（重力、科氏力）；vlim 为 armd 下发的 2.0 rad/s。
  - MIT（重力补偿按关节切换）：力矩 = kp · (q_target − q) + kd · (dq_target − dq) + τ_ff。
  - 失能：无力矩，机械臂在重力下下落。
  - 力矩上限 27 N·m（J1..J3）、7 N·m（J4..J6）、1.5 N·m（J7）。
- **反馈**（250 Hz）：位置、速度、电机总力矩（伺服 + 前馈，因此包含重力和负载）、温度恒定 35 °C、使能状态。

### 通信协议（UDP，JSON，版本 1）

每个数据报一个 JSON 对象。

| 方向 | 消息 |
|---|---|
| armd → sim | `{"v":1,"type":"cmd","seq":N,"enable":bool,"targets":[T × 7] \| null}`，每个控制周期及使能 / 失能时发送（`targets: null`） |
| armd → sim | `{"v":1,"type":"poll","seq":0,...}`，无其他报文时每 20 ms 一次保活 |
| sim → armd | `{"v":1,"type":"state","session":S,"seq":N,"ack":N,"enabled":bool,"q":[7],"dq":[7],"tau":[7],"temp":[7],"t":s}`，发往最近一个数据报的来源地址 |

`T` 为 `{"mode":"pos_vel","q":rad,"vlim":rad/s}` 或 `{"mode":"mit","q":rad,"dq":rad/s,"kp":N·m/rad,"kd":N·m·s/rad,"tau":N·m}`。`ack` 为最近应用的指令 `seq`；`session` 在每次启动仿真器时随机生成。

armd 侧（`src/sim_driver.rs`）：后台线程把状态收进缓存，控制周期内的驱动调用不等待网络。版本不符、数组长度不符、含非有限值、或同一 session 内 `seq` 未递增的状态报文被丢弃。反馈超过 200 ms 时 `is_feedback_valid` 为 false、使能状态为 Unknown；连续 500 ms（`BUS_LOSS_TIMEOUT`）无新反馈时监督器锁存 `BUS_LOSS`，与硬件一致。上使能最多等待 80 ms 收到确认该指令的状态（失能 25 ms），与 Damiao 握手相同。

## 本机实测

主机 spark-6124（aarch64，Cortex-X925，20 核），MuJoCo 4.0.0，Python 3.13：

| 项目 | 数值 |
|---|---|
| 不限速物理（`rebot-sim --benchmark 20000`） | 36,960 与 37,045 步/秒（2 次） |
| 实时节拍仿真器（`--stats 2`） | 500.0–500.5 Hz |
| armd 与 MuJoCo 正运动学一致性（`test_sim_fk`，7 组关节角） | 最大 0.0001 mm、0.0000° |
| 工具朝下 `move_to` [0.30, 0, 0.12]，稳定后 TCP 误差 | 0.04 mm |
| 该姿态保持力矩（J1..J7，N·m） | 0.01, 7.05, −5.90, −0.23, 0.01, 0.00, 0.00 |
| 闭合夹取方块 | `holding: true`，间隙 2.64 rad，τ7 1.00 N·m |
| 杀掉仿真器到 `BUS_LOSS` 的时间 | 0.51 s |

## 未仿真 / 限制

- 电机动力学为简化模型：无电流环、无齿槽转矩、无 CAN 延迟或丢帧模型；POS_VEL 积分环用偏置力矩前馈代替。环路增益、关节阻尼（0.8 N·m·s/rad）、摩擦（0.05 N·m）和转子惯量为估计值，未在电机上辨识。
- 接触参数与摩擦系数（夹指 1.5、方块 1.2）为估计值；碰撞体为方盒而非网格；连杆之间不碰撞。
- J7 力矩量级：夹持与堵转阈值（`gripper_policy.holding_torque_nm`、0.444 N·m 堵转锁存）由 1.5 N·m 上限和增益设定满足，并非实测夹爪模型。
- 暂无相机图像：视觉、`grasp`（点框抓取）和检测需要真实相机或另外的图像源。
- 不模拟电机故障（过温、欠压），温度恒定。不模拟 Damiao 寄存器访问（映射范围检查、CTRL_MODE 写入）：`driver_range_check` 为 null，MIT 切换在本地确认。
- 默认 `speed` 1.0 rad/s 时，夹爪从闭合到张开在约 5.9 s 后到达 −5.699 rad，来不及在 6 s 默认 `timeout_s` 内满足 200 ms 静止，夹爪动作以 `GRIPPER_TIMEOUT` 结束；测试中以 2.0 rad/s 张开。真实夹爪是否相同未核实。
- 仿真器只在主机上运行，未打包到 reCamera。
