# rebot-armd

<div align="center">

### 运行在 Seeed reCamera 上的 reBot B601-DM 机械臂控制守护进程与网页驾驶舱

**Rust 守护进程 `armd`：经 CAN 驱动机械臂，执行基于租约的安全模型，提供浏览器驾驶舱，并把示教片段录制为 Parquet**

[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg?style=flat-square&logo=rust)](https://www.rust-lang.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)
[![CI](https://img.shields.io/github/actions/workflow/status/Seeed-Solution/rebot-armd/ci.yml?branch=main&style=flat-square&label=CI)](https://github.com/Seeed-Solution/rebot-armd/actions)
[![Target Arm](https://img.shields.io/badge/Robot-reBot_B601--DM-orange.svg?style=flat-square)](https://github.com/Seeed-Projects/reBot-DevArm)
[![Platform](https://img.shields.io/badge/Platform-Seeed_reCamera_SG2002-red.svg?style=flat-square)](https://www.seeedstudio.com)

[English](./README.md) | 中文

<!-- TODO: Add demo GIF showing Web cockpit teleoperation here -->

</div>

---

`armd` 与摄像头自带的服务一起运行在 reCamera（SG2002，RISC-V）上。它通过 reCamera B4 底板的 CAN 总线与机械臂的达妙电机通信，以固定周期运行控制循环和安全监督器，并在 8765 端口通过 HTTP 与 WebSocket 提供：

- 单文件网页驾驶舱，适用于手机、平板和桌面（摇杆、笛卡尔控制、摄像头视频、识别叠加、手眼标定、点框抓取）；
- 用于观测的 REST API 和用于控制的 WebSocket API，执行器指令只接受持有控制租约的连接；
- 面向 agent 的高层动作（`move_to`、`move_relative`、`gripper`、`grasp`），由 [rebot-companion](https://github.com/Seeed-Solution/rebot-companion) 及其 MCP 服务器使用；
- 把关节状态与指令录制为 Parquet 文件。

> 本软件会驱动真实的机械臂。在硬件上运行前请先阅读 [安全](#安全)。

**命名**：仓库名 `rebot-armd`，Cargo 包与二进制名 `armd`，在 reCamera 上安装它的 Debian 包名 `rebot-teleop`（init 脚本 `/etc/init.d/K92rebot-teleop`，数据目录 `/usr/share/rebot-teleop`）。

---

## 目录

- [硬件](#硬件)
- [架构](#架构)
- [在 reCamera 上安装](#在-recamera-上安装)
- [从源码构建](#从源码构建)
- [网页驾驶舱](#网页驾驶舱)
- [API 概览](#api-概览)
- [Agent 与 MCP 集成](#agent-与-mcp-集成)
- [示教片段录制（Parquet）](#示教片段录制parquet)
- [配置与命令行](#配置与命令行)
- [接线（reCamera B4 CAN 底板）](#接线recamera-b4-can-底板)
- [安全](#安全)
- [开发](#开发)
- [致谢](#致谢)
- [许可证](#许可证)

---

## 硬件

已测试：

| 项目 | 说明 |
|---|---|
| 机械臂 | reBot B601-DM：6 个臂关节和 1 个夹爪，均为达妙电机；armd 中共 7 个关节（索引 6 为夹爪） |
| 控制器 | Seeed reCamera（SG2002）+ B4 CAN 底板，`can0` 1 Mbit/s（由 init 脚本配置） |
| 电机模式 | 达妙 `POS_VEL`（位置 + 速度上限），默认模式，也是命令行唯一可选的模式 |

源码中存在但**未在硬件上测试**：

| `--follower` | 驱动 | 总线 | 说明 |
|---|---|---|---|
| `rebot-rs` | RobStride（MIT 模式，29 位 CAN ID） | SocketCAN | 6 关节；无使能确认，因此自动断力矩和清除故障会被拒绝 |
| `so-arm101` | 飞特 STS（SO-ARM101） | 串口，1,000,000 波特 | 6 关节；有位置、速度、温度反馈，无力矩与使能反馈 |
| `fashionstar`、`star-arm102` | FashionStar | 串口，115,200 波特 | 6 关节；驱动未声明任何反馈能力 |
| `mock` | 进程内仿真 | — | 默认值；用于测试和无机械臂时的界面开发 |

这些驱动的协议编码由单元测试覆盖（`tests/test_protocol_vectors.rs`、`tests/test_driver_capabilities.rs`），没有在真实机械臂上运行过。它们使用通用 6 关节 profile，没有运动学和工作空间，笛卡尔动作返回 `UNSUPPORTED`。达妙驱动另有带重力前馈的 MIT 模式，但只能在代码中启用（`DamiaoArmDriver::with_control_mode`），命令行和 profile 都不能选择。主从示教未实现，`--leader` / `--leader-port` 会被拒绝。

---

## 架构

```text
 浏览器驾驶舱 / rebot-companion / 脚本
        │  HTTP GET（观测）           WebSocket /api/v1/ws（租约、指令、遥测）
        ▼
┌───────────────────────────── armd :8765 ──────────────────────────────┐
│ axum HTTP/WS 服务 ── 识别代理 ──► 摄像头检测服务 :8090                 │
│        │                                                               │
│ ArmSupervisor（单把锁）                                                │
│   租约 + epoch 校验 → 规划（五次多项式 / S 曲线、笛卡尔 IK）           │
│   安全状态机：Booting、Idle、Armed、Moving、EStop、Fault               │
│   看门狗、关节限位、工作空间盒、限速、跟踪误差                         │
│        │ 控制 tick，默认 50 Hz（权限允许时 SCHED_FIFO 50）             │
│ 驱动：达妙 / RobStride（SocketCAN），飞特 / FashionStar（串口）        │
└────────────────────────────────────────────────────────────────────────┘
        │ CAN 1 Mbit/s
        ▼
   reBot B601-DM 电机
```

- **控制循环**：固定周期 tick（`control_frequency_hz`，内置 profile 为 50 Hz，允许范围 1..1000）。tick 晚于 80 ms 时锁存安全保持，不继续执行旧轨迹。视频不经过 armd：驾驶舱用 JMuxer 直接播放摄像头的 H.264 WebSocket 流（`ws://<host>:8001/`）。
- **租约**：执行器权限属于发送了 `acquire_lease` 并持续发送 `heartbeat` 的 WebSocket 连接（租约超时 2500 ms）。每条 `command` 都带租约 `generation`、当前安全 `epoch_id` 和严格递增的 `request_id`。REST 执行器路由（`torque`、`motion/*`、`action/*`、`clear_fault`）返回 `409 LEASE_REQUIRED`，不执行动作。急停例外：REST 与 WebSocket 都不需要租约。其他客户端可用 `force` 接管，接管后在机械臂稳定前拒绝运动（`HANDOVER_SETTLING`）。
- **安全层**（详见 [docs/control_safety_contract.md](docs/control_safety_contract.md)）：关节位置、速度、加速度限位；`max_temperature` 温度保护（内置 profile 为 70 °C）；反馈新鲜度（250 ms）；跟随误差故障（`max_error_rad` 0.5，持续 500 ms）及动作的 `TRACKING_STALL`；规划运动限速（`planned_max_velocity`，rebot-dm 为 0.45 rad/s）；笛卡尔工作空间检查（地面、包围盒、伸展范围）；租约看门狗自动收纳；不在收纳位时拒绝断力矩（`ARM_NOT_AT_RESTING_POSE`），避免机械臂下坠。

---

## 在 reCamera 上安装

构建安装包（见 [交叉编译](#交叉编译到-recamera)）或使用发布的 `.deb`，复制到摄像头并安装。`192.168.42.1` 是 reCamera 的 USB 网络地址；连接本包启动的 Wi-Fi 热点时，摄像头地址为 `192.168.16.1`。

```bash
scp dist/rebot-teleop_1.0.0_riscv64.deb root@192.168.42.1:/tmp/
ssh root@192.168.42.1 "dpkg -i /tmp/rebot-teleop_1.0.0_riscv64.deb && /etc/init.d/K92rebot-teleop restart"
```

安装内容：

| 路径 | 内容 |
|---|---|
| `/usr/bin/armd`（软链接 `/usr/local/bin/armd`） | 守护进程 |
| `/etc/init.d/K92rebot-teleop`（软链接 `S92rebot-teleop`） | init 脚本：Wi-Fi、`can0` 配置、启动 / 停止 |
| `/usr/share/rebot-teleop/` | 驾驶舱 `index.html`、URDF、reCamera 应用列表用的应用清单 `rebot-teleop.json` |
| `/userdata/local/apps/rebot-teleop.json` | 由 `postinst` 复制 |

init 脚本启动 `armd --follower rebot-dm --follower-port can0 --port 8765`，把 `can0` 设为 1 Mbit/s，并在 `wlan1` 上启动 Wi-Fi 热点（SSID `reCamera-Arm-Teleop`，默认密码 `12345678`，地址 `192.168.16.1`），也可以连接已有的 Wi-Fi 网络。这些值从 `/userdata/local/apps/rebot-teleop.config.json` 读取（`arm.follower_model`、`arm.serial_port`、`port`、`network.ap_ssid`、`network.ap_password`、`network.ap_ip`、`network.sta_ssid`、`network.sta_password`）。在实验室以外使用前，**请修改默认热点密码**。日志在 `/var/log/rebot-teleop.log`。

驾驶舱地址：`http://<摄像头地址>:8765/`。

驾驶舱的识别功能需要摄像头上的检测服务（`yolo-detector`，默认 `http://127.0.0.1:8090`，可用 `ARMD_DETECTOR_URL` 修改），它不包含在本包中。

---

## 从源码构建

需要 Rust 工具链（stable，edition 2021）。Node.js 只在运行网页自测时需要。

```bash
cargo build --release
cargo test

# 用仿真机械臂运行，然后打开 http://localhost:8765/
cargo run --release -- --follower mock
```

在带 CAN 适配器的 Linux PC 上也可以驱动真实机械臂：`cargo run --release -- --follower rebot-dm --follower-port can0`（先把 `can0` 以 1 Mbit/s 启用）。目前只测试过 reCamera 方案。

### 交叉编译到 reCamera

目标 `riscv64gc-unknown-linux-musl`，静态链接。链接器为 `rust-lld`；C 运行时库来自 [sophgo/host-tools](https://github.com/sophgo/host-tools) 中的 SG2002 musl 工具链（`gcc/riscv64-linux-musl-x86_64` 下的 `lib64v0p7_xthead/lp64d` 库）。只用到其中的库文件，因此 x86_64 工具链在 arm64 构建主机上同样可用。

```bash
rustup target add riscv64gc-unknown-linux-musl
git clone --depth 1 https://github.com/sophgo/host-tools
export RECAMERA_TOOLCHAIN=$PWD/host-tools/gcc/riscv64-linux-musl-x86_64

./scripts/build_deb.sh 1.0.0          # 版本参数可省略（默认取 debian/control）
# -> target/riscv64gc-unknown-linux-musl/release/armd
# -> dist/rebot-teleop_1.0.0_riscv64.deb
```

`build_deb.sh` 需要 `cargo` 和 `dpkg-deb`。它根据 `RECAMERA_TOOLCHAIN` 生成 `RUSTFLAGS`（旧变量名 `REC_TOOLCHAIN` / `RISCV_SYSROOT` 仍可用）；如果自行设置了 `RUSTFLAGS`，则按原样使用。`.cargo/config.toml` 只保存目标级的链接器设置。

编写本文档时实测的构建大小（release profile：`opt-level = "z"`、fat LTO、strip）：`armd` 4,327,440 字节（4.1 MiB），`.deb` 2,090,568 字节（2.0 MiB，gzip）。设备上的内存占用未测。

---

## 网页驾驶舱

`src/api/index.html` 是不依赖外部资源的单个 HTML 文件，嵌入二进制并在 `/` 提供。JMuxer（`/assets/jmuxer.min.js`）播放摄像头视频。界面支持中文和英文（页头切换）。

- **简洁 / 专家模式**：简洁模式显示视频、摇杆和叠加在视频上的视角 / 基座坐标系切换；专家模式另有关节 / 笛卡尔模式切换、侧视姿态预览、链路状态、关节状态表、数据集录制和键盘快捷键。
- **关节点动**：两个虚拟摇杆（J1/J2、J3/J5、夹爪）以及键盘。
- **笛卡尔控制**，两种坐标系：
  - *视角坐标系*：方向跟随摄像头画面（摄像头安装在 J6 之后），在浏览器中用 6 关节 IK 求解；
  - *基座坐标系*：方向跟随机械臂基座坐标轴。
  目标跟随实测位姿，经 WebSocket 租约流式发送。
- **识别叠加**：在视频上绘制摄像头检测服务输出的检测框；识别开关与单次快照。
- **手眼标定抽屉**：眼在手上（摄像头装在夹爪上）或眼在手外（固定摄像头），使用 ArUco 标记；采集位姿、求解、导出 rebot-companion 的 `HandEyeCalibration.load_result` 可读取的 JSON。
- **点框抓取**：开启识别后点一个框并确认，armd 执行 `grasp` 动作（张开、俯视接近、闭环图像对中、J6 对齐 0° / 90°、下降、夹紧并判定、抬起）。任何摇杆、按键、停止或急停都会中止。抓取设置（桌面高度、夹爪位置像素、接近与抓取高度、相机俯角）由 armd 保存。目前只在仿真和截图测试中验证过；需要在真机上确认的项目见 [docs/plan_tap_to_grasp.md](docs/plan_tap_to_grasp.md)。
- **急停**按钮、故障显示与清除、长按使能力矩、收纳。

其他客户端持有租约时，驾驶舱切换为观察模式，并提供需确认的接管操作。

---

## API 概览

默认端口 8765。观测走 REST；执行器指令只能经持有租约的 WebSocket 发送。

| 路径 | 方法 | 用途 |
|---|---|---|
| `/api/v1/observation` | GET | 位置、速度、力矩、温度、安全状态、`commanded_positions`、最近的 `action`、驱动能力 |
| `/api/v1/observation/schema` | GET | 自由度、关节名、限位、工作空间、相机安装参数 |
| `/api/v1/ws` | WS | 租约（`acquire_lease`、`heartbeat`）、`command` / `command_ack` / `command_reject`、遥测 |
| `/api/v1/safety/e_stop` | POST | 急停，不需要租约 |
| `/api/v1/torque`、`/api/v1/motion/*`、`/api/v1/action/*`、`/api/v1/safety/clear_fault` | POST | 返回 `409 LEASE_REQUIRED`；请把相同路径作为 WS `command` 发送 |
| `/api/v1/grasp/setup` | GET | 抓取设置（通过 WS 指令修改） |
| `/api/v1/dataset/record/start` / `save` / `discard` | POST | 示教片段录制 |
| `/api/v1/detection/latest`、`/trigger`、`/inference`、`/ws` | GET / POST / WS | 检测服务代理 |

WebSocket 时序：

```json
{"type":"acquire_lease","client_id":"agent_node"}
{"type":"heartbeat","client_id":"agent_node","generation":1,"epoch_id":42}
{"type":"command","request_id":1,"generation":1,"epoch_id":42,"path":"/api/v1/action/move_to","body":{"position":[0.3,0.0,0.2]}}
{"type":"command_ack","request_id":1,"status":"accepted","action_id":5,"expected_duration_s":2.4}
```

参考文档：

- [docs/agent_action_api.md](docs/agent_action_api.md)：`move_to`、`move_relative`（基座、工具、相机坐标系）、`gripper`、`grasp`、动作状态与错误码；
- [docs/control_safety_contract.md](docs/control_safety_contract.md)：租约、epoch、驱动能力、时序默认值、限速、工作空间；
- [docs/detection_api.md](docs/detection_api.md)：识别路由与 agent 调用示例。

---

## Agent 与 MCP 集成

[rebot-companion](https://github.com/Seeed-Solution/rebot-companion) 是本守护进程的主机端 Python SDK 和 stdio MCP 服务器。它经 WebSocket 持有租约、调用动作 API、从摄像头视频流截图并使用识别代理。用 `--armd http://192.168.42.1:8765` 指向 armd。MCP 工具以及与驾驶舱共用机械臂时的租约规则见其 README。

---

## 示教片段录制（Parquet）

录制进行中时，每个控制 tick 追加一帧；帧保存在内存中，`save` 时写入未压缩的 Parquet 文件。

| 列 | 类型 | 内容 |
|---|---|---|
| `timestamp` | Float64 | armd 启动以来的秒数 |
| `observation.state` | List<Float32> | 实测关节位置（rad） |
| `action` | List<Float32> | 指令关节位置（rad） |

```bash
curl -X POST http://192.168.42.1:8765/api/v1/dataset/record/start \
     -H 'Content-Type: application/json' -d '{"output_path":"/userdata/datasets/episode_001.parquet"}'
curl -X POST http://192.168.42.1:8765/api/v1/dataset/record/save      # 或 .../discard
```

不传 `output_path` 时，文件写到工作目录下的 `./data/episodes/<episode_id>.parquet`。列名沿用 LeRobot 的命名，但 armd 每个片段写一个普通 Parquet 文件，不是 LeRobot 数据集（没有 `meta/` 文件、片段与帧索引，也没有相机画面）。本仓库不提供转换为 LeRobot 数据集的工具，也未测试用它训练。

---

## 配置与命令行

```text
armd [OPTIONS]
    --follower <MODEL>       rebot-dm, rebot-rs, so-arm101, fashionstar, star-arm102, mock（默认）
    --follower-port <PORT>   CAN 接口（默认 can0）或串口设备（默认 /dev/ttyUSB0）
    --config <JSON>          ArmConfig profile（关节数必须与 follower 一致）
    --port <PORT>            HTTP / WebSocket 端口（默认 8765，或 $PORT）
    -h, --help
```

环境变量：`RUST_LOG`（日志过滤，默认 `armd=info,tower_http=info`）、`PORT`、`ARMD_DETECTOR_URL`（默认 `http://127.0.0.1:8090`）、`ARMD_GRASP_SETUP`（抓取设置文件，默认 `/userdata/rebot-teleop/grasp_setup.json`）。

不带 `--config` 时，`rebot-dm` 使用内置 B601-DM profile（`src/model.rs` 中的 `ArmConfig::rebot_dm()`），其他 follower 使用通用 6 关节 profile。`--config` 文件是完整的 `ArmConfig` JSON；以下为 rebot-dm 取值的节选：

```json
{
  "dof": 7,
  "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"],
  "control_frequency_hz": 50,
  "lease_timeout_ms": 2500,
  "max_temperature": 70.0,
  "limits": [
    {"name": "joint_1", "min_position": -3.14159, "max_position": 3.14159, "max_velocity": 2.0, "max_acceleration": 5.0}
  ],
  "home_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0],
  "planned_max_velocity": 0.45,
  "workspace": {"floor_z_m": 0.0, "box_min_m": [-0.6, -0.6, -1.0], "box_max_m": [0.6, 0.6, 0.8], "max_reach_m": 0.52}
}
```

没有 `kinematics` 和 `workspace` 的 profile 不做笛卡尔工作空间检查，也不支持笛卡尔动作；armd 启动时会记录告警。字段列表见 [docs/control_safety_contract.md](docs/control_safety_contract.md#configuration--配置)。

---

## 接线（reCamera B4 CAN 底板）

以下信息来自 B4 底板资料，本仓库未做验证。请以手中底板版本的丝印为准。

```text
              reCamera B4 CAN 底板
     [ 按键 RST/BOOT ]               [ 丝印 CAN&POWER ]
    ┌─────────────────────────────────────────────────┐
    │              J2（白色 6-Pin 插座）               │
    │         ┌───┬───┬───┬───┬───────┬───────┐       │
    │         │ 1 │ 2 │ 3 │ 4 │   5   │   6   │       │
    │         └───┴───┴───┴───┴───┬───┴───┬───┘       │
    └─────────────────────────────┼───────┼───────────┘
                                CAN_H   CAN_L
```

J2（SM06B-SRSS-TB，JST SH 1.0 mm，6 Pin）：

- Pin 1 / 2：`VIN`（12-24 V 直流输入）
- Pin 3 / 4：`GND`
- Pin 5：`CAN_H`（接机械臂红线）
- Pin 6：`CAN_L`（接机械臂黑线）

底板带 MCP2518FD SPI-CAN 控制器、MCP2558FD 收发器和 124 Ω 终端电阻。电机动力线不要触碰 Pin 5/6，并将 reCamera 与机械臂电源的地（Pin 3/4）连通。

---

## 安全

armd 驱动的真实机械臂有足以伤人和损坏物品的力量。使用风险自负；MIT 许可证不提供任何担保。

- 通电期间，人员（尤其是手和脸）不要进入机械臂的运动范围。使能力矩前先清空工作区。
- 在手边保留硬件断电开关。软件急停（驾驶舱按钮、`POST /api/v1/safety/e_stop`、WS `estop`）依赖网络和守护进程，不能代替物理急停。
- 租约看门狗：机械臂带力矩时，租约持有者停止心跳 2.5 s 后，若机械臂状态正常且有收纳位，armd 自动收纳并断力矩；否则保持力矩原地保持，并锁存 `Fault(WatchdogTimeout)`。收纳期限为 max(10 s, 规划收纳时长 + 3 s)。
- 不在收纳位时不允许断力矩（`ARM_NOT_AT_RESTING_POSE`），请使用收纳。
- 工作空间盒：rebot-dm 上，servo 目标和规划运动采样点必须使 end_link 原点、夹爪包络 8 个角点和相机点高于地面（`floor_z_m`）、位于包围盒内且在 `max_reach_m` 之内；越界时返回 `422 WORKSPACE_VIOLATION`。包围盒是粗略限制，不检测与桌面物体的碰撞。
- 限速：rebot-dm 上规划运动与动作的单关节速度上限为 0.45 rad/s；动作默认 0.05 m/s（最大 0.15 m/s）和 0.5 rad/s（最大 1.0 rad/s）。
- 跟踪误差故障：跟随误差超过 0.5 rad 持续 500 ms 时锁存故障，信息中包含关节号；动作会更早以 `TRACKING_STALL` 结束。
- 默认热点密码（`12345678`）和未鉴权的 HTTP/WebSocket API 使该网络上的任何人都能控制机械臂和开始录制。请修改密码，并只在可信网络上运行。

以上数值是源码中的软件默认值（`src/model.rs`、`src/supervisor.rs`），不是经过认证的安全功能。

---

## 开发

```bash
cargo test                                 # 248 个测试（1 个 ignored），使用仿真机械臂，不需要硬件
cargo clippy --all-targets -- -D warnings
node scripts/view_ik_selftest.mjs          # 驾驶舱视角坐标系 IK
node scripts/calib_selftest.mjs            # 驾驶舱手眼标定（c 部分需要 ../rebot-companion 和 uv）
node tests/test_rpc_error_contract.js      # 驾驶舱 RPC 错误处理
```

CI（`.github/workflows/ci.yml`）运行以上检查。未强制 `cargo fmt --check`：代码尚未按 rustfmt 格式化。RISC-V 交叉编译不在 CI 中，因为它需要 SG2002 工具链的库文件。`test_damiao_smoothness::stale_posvel_gripper_recovery_uses_posvel_frame` 对时序敏感，机器负载高时可能失败，重跑即可。

Playwright 脚本（`scripts/*_shots.mjs`、`scripts/ux_walkthrough.mjs`、`tests/test_*_playwright.js`、`tests/test_single_origin_e2e.js`）用于驾驶舱截图；需要 Playwright（可用 `PLAYWRIGHT=<模块路径>` 指定）和 Chromium（`CHROME=<路径>`），部分脚本需要运行中的 armd。`scripts/*_shots.mjs` 写到 `.cache/`（或 `OUT`），`tests/*.js` 写到 `tests/artifacts/`，两者都已被 git 忽略。`scripts/` 中的 Python 脚本用于台架诊断：有的直接向电机发送 CAN 帧，有的通过 API 驱动运行中的 armd。只在机械臂有支撑、工作区已清空时运行。

另见 [CONTRIBUTING.md](CONTRIBUTING.md) 和 [SECURITY.md](SECURITY.md)。

---

## 致谢

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)：机械臂硬件，以及本项目运动学和重力模型所依据的 URDF。
- [HuggingFace LeRobot](https://github.com/huggingface/lerobot)：录制片段的列命名。
- [JMuxer](https://github.com/samirkumardas/jmuxer)：驾驶舱中的 H.264 播放。

---

## 许可证

MIT，见 [LICENSE](LICENSE)。第三方文件保留各自的许可证，列于 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)：JMuxer（MIT）和 reBot-DevArm URDF（CERN-OHL-W-2.0）。
