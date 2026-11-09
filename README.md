# rebot-armd

<div align="center">

### 🤖 专为 HuggingFace LeRobot 打造的 3.3MB Rust 极简机械臂驱动与原生 Parquet 录制底座

**3.3MB Rust HAL & native Parquet recorder for HuggingFace LeRobot on 256MB edge boards**

[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg?style=flat-square&logo=rust)](https://www.rust-lang.org)
[![HuggingFace LeRobot](https://img.shields.io/badge/LeRobot-Native_Parquet-ffcc00.svg?style=flat-square&logo=huggingface)](https://github.com/huggingface/lerobot)
[![Robots](https://img.shields.io/badge/Robots-SO--ARM100%20%7C%20Rebot-ff69b4.svg?style=flat-square)](https://github.com/TheRobotStudio/SO-ARM100)
[![Binary Size](https://img.shields.io/badge/Binary_Size-3.3_MB-brightgreen.svg?style=flat-square)](https://github.com/Seeed-Solution/rebot-armd)
[![Control Loop](https://img.shields.io/badge/Control_Loop-50Hz~100Hz-blue.svg?style=flat-square)](https://github.com/Seeed-Solution/rebot-armd)
[![Embodied AI](https://img.shields.io/badge/Embodied_AI-ACT_%7C_Diffusion_Ready-9cf.svg?style=flat-square)](https://github.com/Seeed-Solution/rebot-armd)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)

<!-- TODO: Add demo GIF showing Web cockpit teleoperation and Parquet recording here -->

</div>

---

## 目录

- [设计定位与对比](#-设计定位与对比)
- [核心特性](#-核心特性)
- [快速上手](#-快速上手-quickstart)
- [支持硬件与协议矩阵](#-支持硬件与协议矩阵)
- [LeRobot 数据集录制 (Native Parquet)](#-lerobot-数据集录制-native-parquet)
- [Agent 动作与安全工作空间 (Action API)](#-agent-动作与安全工作空间-action-api)
- [REST 与 WebSocket API 规范](#-rest--websocket-api-规范)
- [reCamera (B4 CAN 底板) 接线指南](#-recamera-b4-can-底板接线指南)
- [配置说明 (Configuration)](#-配置说明-configuration)
- [构建与 Debian 一键打包](#-构建与-debian-一键打包-risc-v--arm64)
- [命令行参数参考](#-命令行参数参考-cli-reference)
- [参与贡献 (Contributing)](#-参与贡献-contributing)
- [致谢 (Acknowledgements)](#-致谢-acknowledgements)
- [开源许可证](#-开源许可证)

---

## 💡 设计定位与对比

在具身智能（Embodied AI）与模仿学习工作流中，[HuggingFace LeRobot](https://github.com/huggingface/lerobot) 提供了模型训练（ACT、Diffusion Policy）的算法框架。然而在**现场示教与边缘部署阶段**，依赖完整 Python 环境通常面临以下约束：

- **环境体积**：现场示教若依赖 Python / PyTorch / Conda，需要数 GB 磁盘空间和数 GB 内存，难以直接运行在低算力单板或微型边缘芯片上；
- **调度延迟与确定性**：解释器垃圾回收与多线程调度在 50Hz~100Hz 高频总线同步通信时可能引入抖动；
- **总线协议适配**：不同品牌机械臂与电机（达妙 CAN、飞特半双工串口、RobStride CAN）缺乏统一解耦的硬件抽象层。

`rebot-armd` 提供纯 Rust 实现的独立硬件抽象底座：
- **3.3MB 单二进制文件**，运行时内存占用约 20MB；
- 可直接部署于低至 256MB RAM 的边缘硬件（如 **Seeed reCamera SG2002 RISC-V 芯片**）或单板电脑；
- 采集动作直接流式写入 Apache Parquet 格式，无需安装 Python 即可完成示教录制并对接 LeRobot 训练流水线；
- 内置零安装单文件 Web 触控驾驶舱与基于租约的安全状态机。

---

## 🚀 核心特性

- **零 Python 依赖直接录制**：底层集成 Apache Arrow & Parquet，以 50Hz~100Hz 频率实时将关节角度和指令流式压入 `.parquet` 文件，原生匹配 LeRobot ACT / Diffusion Policy 训练要求。
- **超轻量边缘部署**：单一静态编译二进制文件（约 3.3MB，Debian deb 包约 1.6MB），专为 256MB RAM 的 RISC-V 芯片及 Linux ARM64/x86_64 设计。
- **单文件 Web 触控驾驶舱**：内置独立 Web 前端，连上 Wi-Fi 或局域网即可通过移动端/桌面浏览器触控虚拟双摇杆遥控，带 2D 正向运动学（FK）骨骼姿态可视化与急停（E-Stop）。
- **租约式安全状态机**：actuator 执行权限绑定 WebSocket 控制租约，具备 epoch 校验、心跳看门狗自动安全收纳（Watchdog Park）、软限位、跟随误差报警与实时温度监控。
- **高层 Agent 动作与工作空间约束**：提供阻尼最小二乘（DLS）逆运动学笛卡尔直线插补（`move_to` / `move_relative`）、接触力矩自适应夹爪抓取（`gripper`）、tap-to-grasp 闭环图像居中抓取，以及桌面地面与包络盒软碰撞防护。
- **多总线多电机协议矩阵**：统一抽象达妙 DM4310 (CAN)、飞特 STS3215 (UART, SO-ARM100/101)、RobStride (CAN) 与本地 Mock 仿真。

---

## ⚡ 快速上手 (Quickstart)

### 1. 本地仿真模式 (Mock Teleop)
无需连接物理机械臂，可在 Linux、macOS 或 Windows 本地一秒启动仿真节点：

```bash
# 本地编译并运行 Mock 机械臂
cargo run --release -- --follower mock

# 打开浏览器进入嵌入式触控驾驶舱
open http://localhost:8765
```

### 2. 直连物理机械臂 (以达妙电机 CAN 驱动为例)
```bash
# 通过板载 CAN 接口直驱 6 自由度机械臂
cargo run --release -- --follower rebot-dm --follower-port can0
```

### 3. 主从示教模式 (规划中)
> [!NOTE]
> 主从示教（Leader-Follower Teleoperation）当前尚未实现。CLI 中传入 `--leader` 或 `--leader-port` 参数会被服务端明确拒绝。

---

## 🤖 支持硬件与协议矩阵

### 1. 机械臂型号支持

- **[SO-ARM100 / SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100)**：HuggingFace LeRobot 社区开源机械臂（飞特 STS3215 总线舵机，6-DoF）；
- **reBot B601-DM 系列**：基于达妙 DM4310 关节电机的 6-DoF 工业关节力控机械臂 + 1 轴电动夹爪（7-DoF）；
- **reBot-RS 系列**：基于 RobStride 关节模组的直驱/准直驱机器人臂（6-DoF）；
- **Star-Arm 示教器**：高精度手持示教臂（跟随从臂 profile 已定义，主臂模式规划中）；
- **Mock 仿真臂**：内置 6-DoF 软件仿真模型，供离线开发与 API 联调。

### 2. 电机通信协议与能力矩阵

| 电机品牌/型号 | 物理总线 | 默认通信速率 | 控制模式 | 核心数据帧与反馈特性 |
| :--- | :--- | :--- | :--- | :--- |
| **达妙 (Damiao / DM-J 系列)** | CAN 2.0A | 1 Mbps | 默认 POS_VEL；可选 MIT | 位置/速度/力矩/温度/使能反馈；默认 POS_VEL 模式不发送重力前馈，MIT 模式支持重力前馈 |
| **飞特 (Feetech / STS3215)** | 半双工 UART | 1,000,000 bps | 位置伺服 | 位置、速度、温度反馈；驱动能力不含使能状态确认与力矩反馈 |
| **RobStride (RS00/01/02)** | CAN 2.0B (29-bit 扩展帧) | 1 Mbps | MIT 模式 | 位置、速度、力矩、温度反馈与力矩前馈；使能状态确认能力为 false |
| **FashionStar (方成串口总线)** | 半双工 UART | 115,200 bps | 角度控制 | 当前驱动能力声明为空；数值占位不代表硬件测得反馈 |

---

## 📊 LeRobot 数据集录制 (Native Parquet)

`rebot-armd` 底层集成 Apache Arrow 与 Parquet 库，将动作帧直接写入标准 Parquet 文件，无需中间转换：

```text
[遥操作示教 / 摇杆直驱]
         │
         ▼ (按配置频率 50~100Hz 运行的控制循环)
┌─────────────────────────────────────────────────────────────┐
│ rebot-armd 原生 Parquet 录制器 (Snappy 压缩)                │
│                                                             │
│   • timestamp           : Float64 (微秒高精时间戳)          │
│   • observation.state   : List<Float32> (各轴当前实测物理弧度)│
│   • action              : List<Float32> (各轴目标下发控制弧度)│
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼ (生成 .parquet 文件)
┌─────────────────────────────────────────────────────────────┐
│                  HuggingFace LeRobot 生态                    │
│                                                             │
│   $ python lerobot/scripts/train.py \                       │
│       --policy.type act \                                   │
│       --dataset.repo_id /data/episode_001.parquet           │
└─────────────────────────────────────────────────────────────┘
```

### 录制控制 API
- **开始录制**：`POST /api/v1/dataset/record/start`，请求体：`{"output_path": "/data/episode_001.parquet"}`
- **保存分卷**：`POST /api/v1/dataset/record/save`
- **放弃本轮**：`POST /api/v1/dataset/record/discard`

---

## 🛡️ Agent 动作与安全工作空间 (Action API)

为方便上位机 AI Agent（如通过 [rebot-companion](https://github.com/Seeed-Solution/rebot-companion) MCP 服务）调度，`rebot-armd` 在 WebSocket 控制租约下内置高层动作规划接口：

### 1. 高层动作路径
- `/api/v1/action/move_to`：笛卡尔空间直线轨迹规划（位置直线插值 + 姿态 Slerp 插值 + 阻尼最小二乘 DLS IK）；
- `/api/v1/action/move_relative`：基于基座（`base`）、工具末端（`tool`）或相机坐标系（`camera`，支持视线水平自适应）的相对位移；
- `/api/v1/action/gripper`：夹爪控制（`open` / `close` / `move`，支持堵转与维持力矩自适应判定 `holding`）。

### 2. 工作空间与碰撞边界防护
内置安全监视器强制执行物理边界检查：
- **地面防护**：末端执行器原点、夹爪包围盒 8 个顶点及安装相机均必须保持在 `floor_z_m` 以上；
- **边界盒与最大伸展范围**：采样点必须在定义的工作空间盒内且在距离肩轴 `max_reach_m` 范围内；任何越界且远离安全区的轨迹采样将被以 `422 WORKSPACE_VIOLATION` 拒绝。

---

## 📡 REST 与 WebSocket API 规范

默认端口：`8765`（支持 CORS 跨域与移动端 Web 遥测）。观测数据通过 REST 获取；执行器控制必须使用持有租约的 WebSocket。

| 路径 | 方法 | 功能描述 | 约束说明 |
| :--- | :--- | :--- | :--- |
| `/api/v1/observation` | `GET` | 50Hz 遥测状态（位置、速度、力矩、温度、动作进度、安全状态） | 只读查询，无需租约 |
| `/api/v1/observation/schema` | `GET` | 机械臂自由度、关节极值限位与工作空间几何定义 | 只读查询 |
| `/api/v1/torque`、`/motion/*` | `POST` | REST 执行器端点 | 返回 `409 LEASE_REQUIRED` 错误，强制改用 WebSocket |
| `/api/v1/safety/e_stop` | `POST` | 紧急制动并切入 EStop 锁死状态 | 紧急安全接口，**无需租约** |
| `/api/v1/dataset/record/*` | `POST` | LeRobot 数据集片段录制开始、保存、废弃 | 控制 Parquet 持久化 |
| `/api/v1/ws` | `WS` | 50Hz 双向实时流（遥测下行、控制指令与心跳上行） | 需租约握手与递增 `request_id` |

### WebSocket 交互时序

```json
{"type":"acquire_lease","client_id":"agent_node"}
{"type":"heartbeat","client_id":"agent_node","generation":1,"epoch_id":42}
{"type":"command","request_id":1,"generation":1,"epoch_id":42,"path":"/api/v1/action/move_to","body":{"position":[0.3,0.0,0.2]}}
{"type":"command_ack","request_id":1,"status":"accepted","action_id":5,"expected_duration_s":2.4}
```

---

## 📐 reCamera (B4 CAN 底板) 接线指南

Seeed reCamera Gimbal 扩展底板（B4）板载 MCP2518FD 工业 SPI-CAN 控制器与 MCP2558FD 收发器，内置 **124Ω 终端电阻**。

```text
              reCamera B4 CAN 底板
     [ 按键 RST/BOOT ]               [ 丝印 CAN&POWER ]
    ┌─────────────────────────────────────────────────┐
    │                                                 │
    │              J2 (白色 6-Pin 插座)                │
    │         ┌───┬───┬───┬───┬───────┬───────┐       │
    │         │ 1 │ 2 │ 3 │ 4 │   5   │   6   │       │
    │         └───┴───┴───┴───┴───┬───┴───┬───┘       │
    └─────────────────────────────┼───────┼───────────┘
                                  │       │
                                CAN_H   CAN_L
                                  │       │
                                ┌─┴───────┴─┐
                                │ 红线   黑线│ (机械臂 CAN 信号线)
                                └───────────┘
```

#### J2 物理接口引脚定义 (SM06B-SRSS-TB, JST SH 1.0mm 6-Pin)：
- **Pin 1 / Pin 2**: `VIN` (12V ~ 24V DC IN，板载 TVS 浪涌保护，并联通流)
- **Pin 3 / Pin 4**: `GND` (功率与信号参考地)
- **Pin 5**: **`CAN_H`** (差分高，接机械臂红线)
- **Pin 6**: **`CAN_L`** (差分低，接机械臂黑线)

> [!CAUTION]
> 动力电源（如 24V 开关电源粗线）严禁触碰 Pin 5/6 信号线；reCamera 与机械臂控制板建议将 **Pin 3/4 (GND)** 连通共地以消除共模干扰。

---

## ⚙️ 配置说明 (Configuration)

可以通过 `--config <path.json>` 传入自定义 JSON 配置。核心字段说明：

```json
{
  "dof": 7,
  "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
  "control_frequency_hz": 50,
  "lease_timeout_ms": 2500,
  "max_temperature": 75.0,
  "limits": [
    {"name": "joint1", "min_position": -3.14, "max_position": 3.14, "max_velocity": 1.5, "max_acceleration": 3.0}
  ],
  "home_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 1.5708, 0.0],
  "planned_max_velocity": 0.45,
  "workspace": {
    "floor_z_m": 0.02,
    "max_reach_m": 0.65
  }
}
```

---

## 🛠️ 构建与 Debian 一键打包 (RISC-V / ARM64)

### 1. 本地编译与单测
```bash
cargo check
cargo test  # 运行全量协议向量与安全状态机单元测试
```

### 2. 交叉编译为 SG2002 RISC-V deb 安装包
```bash
chmod +x scripts/build_deb.sh
./scripts/build_deb.sh 1.0.0
# 产物输出至: dist/rebot-teleop_1.0.0_riscv64.deb (约 1.6 MB)
```

### 3. reCamera 设备端一键部署
```bash
scp dist/rebot-teleop_1.0.0_riscv64.deb root@192.168.16.1:/tmp/
ssh root@192.168.16.1 "dpkg -i /tmp/rebot-teleop_1.0.0_riscv64.deb && /etc/init.d/K92rebot-teleop restart"
```

---

## 💻 命令行参数参考 (CLI Reference)

```text
armd - Minimal Robotic Arm Daemon & Teleop HAL for LeRobot

USAGE:
    armd [OPTIONS]

OPTIONS:
    --follower <MODEL>       从动臂型号: rebot-dm, rebot-rs, so-arm101, fashionstar, star-arm102, mock [默认: mock]
    --follower-port <PORT>   从动臂通信端口: can0, /dev/ttyUSB0, /dev/ttyS1
    --port <PORT>            HTTP / WebSocket 服务端口 [默认: 8765]
    --config <PATH>          加载自定义 ArmConfig JSON 配置文件
    -h, --help               打印帮助信息
```

---

## 🤝 参与贡献 (Contributing)

欢迎提交 Issue 和 Pull Request 适配更多开源机械臂结构与电机协议：
1. Fork 本仓库并新建分支（如 `feature/my-robot-profile`）；
2. 遵循代码格式规范 (`cargo fmt` 与 `cargo clippy`)；
3. 增加对应协议和状态机的单元测试并确保通过 (`cargo test`)；
4. 提交清晰的 commit 记录。

---

## 💖 致谢 (Acknowledgements)

- **[HuggingFace LeRobot](https://github.com/huggingface/lerobot)**：具身智能与模仿学习优秀生态；
- **[TheRobotStudio SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)**：广受欢迎的低成本开源 3D 打印机械臂；
- **[Seeed Studio](https://www.seeedstudio.com/)**：reCamera SG2002 边缘视觉硬件与 reBot 机械臂平台支持。

---

## 📄 开源许可证

本项目遵循 [MIT 许可证](LICENSE) 开源。
