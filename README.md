# rebot-armd

<div align="center">

### 🤖 专为 HuggingFace LeRobot 与具身智能打造的 3.3MB 极简 Rust 机械臂驱动底座

**Ultra-Lightweight Rust HAL & Teleop Daemon for HuggingFace LeRobot | SO-ARM100 | Embodied AI | Edge RISC-V**

[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg?style=flat-square&logo=rust)](https://www.rust-lang.org)
[![HuggingFace LeRobot](https://img.shields.io/badge/LeRobot-Native_Parquet-ffcc00.svg?style=flat-square&logo=huggingface)](https://github.com/huggingface/lerobot)
[![Robots](https://img.shields.io/badge/Robots-SO--ARM100%20%7C%20Rebot-ff69b4.svg?style=flat-square)](https://github.com/TheRobotStudio/SO-ARM100)
[![Binary Size](https://img.shields.io/badge/Binary_Size-3.3_MB-brightgreen.svg?style=flat-square)](https://github.com)
[![Control Loop](https://img.shields.io/badge/Control_Loop-50Hz~100Hz-blue.svg?style=flat-square)](https://github.com)
[![Embodied AI](https://img.shields.io/badge/Embodied_AI-ACT_%7C_Diffusion_Ready-9cf.svg?style=flat-square)](https://github.com)
[![License](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)

[快速开始](#-5-秒极速上手-quickstart) • [为什么是-rebot-armd](#-为什么选择-rebot-armd-痛点颠覆) • [支持硬件与协议](#-支持的开源机器人与电机矩阵) • [LeRobot-示教采集](#-lerobot-数据集录制-native-parquet) • [API-规范](#-rest--websocket-api-接口) • [接线指南](#-recamera-b4-can-底板接线)

</div>

---

## 💡 为什么选择 rebot-armd？ (痛点颠覆)

在当前的具身智能（Embodied AI）与机器人学习（Robot Learning）开发中，主流框架如 **[HuggingFace LeRobot](https://github.com/huggingface/lerobot)** 为模型训练（ACT、Diffusion Policy、Aloha）提供了极为出色的算法生态。

但到了**现场示教与边缘部署阶段**，传统工作流往往痛点重重：
- ❌ **环境极其臃肿**：为了在现场采集示教数据，机械臂旁边必须拖着一台笨重工控机或笔记本，跑着十几个 GB 的 Python / Conda / PyTorch 依赖环境；
- ❌ **延迟抖动与通信不稳定**：Python GIL 与垃圾回收机制在高频（50Hz~100Hz）总线通信和多轴数据对齐时容易出现帧丢失；
- ❌ **硬件接入碎片化**：达妙（Damiao MIT 阻抗）、飞特（Feetech 串口总线）、RobStride 等不同总线协议难以统一解耦。

### `rebot-armd` 的运行范围：
> **提供统一驱动、安全状态机、WebSocket 控制租约与 LeRobot Parquet 录制接口。** 二进制大小、循环抖动和工业确定性取决于构建与现场环境，需单独测量。

无需配置任何 Python 环境，哪怕是在售价几十元、内存仅 256MB 的边缘 RISC-V 芯片（如 **Seeed reCamera SG2002**）或微型单板电脑上，插上机械臂即可独立运行：
1. **零 Python 依赖录制**：通过纯 Rust 流式将 50Hz 关节角度和控制指令压入标准 `.parquet` 文件，**采集完毕直接扔给 LeRobot 即可启动 ACT / Diffusion 训练**；
2. **零安装触控驾驶舱**：内置单文件 Web 前端，手机连上 Wi-Fi 热点即可通过虚拟双摇杆远程遥控，配合 2D 正向运动学（FK）骨骼姿态可视化与一键 E-Stop；
3. **有边界的安全策略**：控制租约、软限位、跟随误差、温度检查和 E-Stop；它们不能替代急停回路、机械限位或现场风险评估。

---

## ⚡ 5 秒极速上手 (Quickstart)

### 1. 本地无硬件仿真模式 (Mock Teleop)
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

### 3. 主从示教模式 (Leader-Follower Teleoperation，未实现)
连接主动臂（Leader）与从动臂（Follower）进行实时镜像同步与 LeRobot 示教：
```bash
cargo run --release -- \
  --leader star-arm102 --leader-port /dev/ttyACM0 \
  --follower so-arm100 --follower-port /dev/ttyUSB0
```
上述 `--leader`/`--leader-port` 参数会被明确拒绝；主从示教尚未实现，不要按此命令运行。

---

## 🤖 支持的开源机器人与电机矩阵

### 1. 社区热门具身开源机械臂
- **[SO-ARM100 / SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100)**：HuggingFace LeRobot 社区官方主推的低成本开源模仿学习机械臂（飞特 STS3215 总线舵机）；
- **Rebot-DM 系列**：基于达妙 DM4310 工业关节电机的 6-DoF 高动态力控机械臂；
- **Rebot-RS 系列**：基于 RobStride 关节模组的直驱/准直驱机器人臂；
- **Star-Arm 示教器**：高精度无阻力手持主从示教臂。

### 2. 电机通信协议细节

| 电机品牌/型号 | 物理总线 | 默认通信速率 | 控制模式 | 核心数据帧特性 |
| :--- | :--- | :--- | :--- | :--- |
| **达妙 (Damiao / DM-J 系列)** | CAN 2.0A | 1 Mbps | 默认 POS_VEL；可选 MIT | 位置/速度/力矩/温度/使能反馈；默认 POS_VEL 不发送重力力矩前馈，MIT 才支持重力前馈 |
| **飞特 (Feetech / STS3215)** | 半双工 UART | 1,000,000 bps | 位置伺服 | 位置、速度、温度反馈；驱动能力不含使能状态确认、力矩反馈或力矩前馈 |
| **RobStride (RS00/01/02)** | CAN 2.0B (29-bit 扩展帧) | 1 Mbps | MIT 模式 | 位置、速度、力矩、温度反馈与力矩前馈；使能状态确认能力为 false |
| **FashionStar (方成串口总线)** | 半双工 UART | 115,200 bps | 角度控制 | 当前驱动能力声明为空；数值占位不等于已测得反馈，不能据此自动失能或清故障 |

---

## 📊 LeRobot 数据集录制 (Native Parquet)

`rebot-armd` 在底层集成 Apache Arrow 与 Parquet 库，将动作帧写入 Parquet；与 Python/LeRobot 的兼容性仍应按实际数据集验证。

```
[遥操作示教 / 摇杆直驱]
         │
         ▼ (按配置频率运行的控制循环)
┌─────────────────────────────────────────────────────────────┐
│ rebot-armd 原生 Parquet 录制器 (Snappy 压缩)                │
│                                                             │
│   • timestamp           : Float64 (微秒高精时间戳)          │
│   • observation.state   : List<Float32> (6 轴当前实测物理弧度)│
│   • action              : List<Float32> (6 轴目标下发控制弧度)│
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼ (生成 .parquet 文件)
┌─────────────────────────────────────────────────────────────┐
│                  HuggingFace LeRobot 生态                    │
│                                                             │
│   $ python lerobot/scripts/train.py \                       │
│       --policy.type act \                                   │
│       --dataset.repo_id /data/episode_001.parquet           │
│                                                             │
│   将数据集交给 ACT / Diffusion Policy 前请按项目格式核验。 │
└─────────────────────────────────────────────────────────────┘
```

### 录制控制 API（随时触发）
- **开始录制**：`POST /api/v1/dataset/record/start` (`{"output_path": "/data/episode_001.parquet"}`)
- **保存分卷**：`POST /api/v1/dataset/record/save`
- **放弃本轮**：`POST /api/v1/dataset/record/discard`

---

## 📐 reCamera (B4 CAN 底板) 接线指南

Seeed reCamera Gimbal 扩展底板（B4）板载 MCP2518FD 工业 SPI-CAN 控制器与 MCP2558FD 收发器，内置 **124Ω 终端电阻**（无需外挂）。

```
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
> 机械臂强电动力电源（如 24V 开关电源粗线）严禁触碰 Pin 5/6 信号线；reCamera 与机械臂控制板建议将 **Pin 3/4 (GND)** 连通共地以消除共模干扰。

---

## 📡 REST & WebSocket API 接口

默认端口：`8765`（支持 CORS 跨域与移动端 Web 遥测）。观测可用 REST；执行器动作必须使用持有租约的 WebSocket。REST 执行器端点返回结构化错误，不会绕过租约；E-Stop 不需要租约。

| 路径 | 方法 | 功能描述 | 请求参数示例 |
| :--- | :--- | :--- | :--- |
| `/api/v1/observation` | `GET` | 50Hz 遥测状态（位置、速度、力矩、温度、安全状态） | 无 |
| `/api/v1/observation/schema` | `GET` | 自由度配置与物理软限位极值查询 | 无 |
| `/api/v1/torque`、`/heartbeat`、`/motion/*` | `POST` | REST 执行器端点；返回 `LEASE_REQUIRED`/`LEASE_ACTIVE` 错误并提示改用 WebSocket | 不执行动作 |
| `/api/v1/safety/e_stop` | `POST` | 紧急制动并切入 EStop 锁死状态 | 无 |
| `/api/v1/safety/clear_fault` | `POST` | REST 返回租约错误；通过 WebSocket command 执行，仍需新鲜反馈与非 Unknown 使能状态 |
| `/api/v1/dataset/record/start` | `POST` | 开启 LeRobot 数据集片段录制 | `{"output_path": "episode_01.parquet"}` |
| `/api/v1/dataset/record/save` | `POST` | 停止并持久化 Parquet 数据文件 | 无 |
| `/api/v1/ws` | `WS` | 50Hz 双向实时流（下行遥测、上行心跳） | WebSocket 连接 |

WebSocket 控制流程（`epoch_id` 为当前观测中的值，`generation` 由服务端返回）：

```json
{"type":"acquire_lease","client_id":"web"}
{"type":"heartbeat","client_id":"web","generation":7,"epoch_id":42}
{"type":"command","request_id":1,"generation":7,"epoch_id":42,"path":"/api/v1/motion/servo","body":{"targets":[0,0,0,0,0,0]}}
{"type":"command_ack","request_id":1,"status":"accepted","epoch_id":42,"safety_state":"Armed","is_torqued":true}
```

服务端先返回 `lease_granted`（含 `generation`）；心跳应持续发送并带当前 `epoch_id`。`request_id` 必须严格递增。E-Stop 可直接发送 `{"type":"estop"}`，返回 `epoch_sync`，之后须重新同步 epoch；租约丢失、epoch 不匹配或拒绝时返回 `command_reject`。

---

## 🛠️ 构建与 Debian 一键打包 (RISC-V / ARM64)

### 1. 本地检查与测试
```bash
cargo check
cargo test  # 49 项全量协议向量与安全状态机单元测试
```

### 2. 交叉编译与生成 deb 安装包
使用内置自动化脚本构建针对 SG2002 RISC-V 平台的极轻量 deb 包：

```bash
chmod +x scripts/build_deb.sh
./scripts/build_deb.sh 1.0.0
# 产物输出至: dist/rebot-teleop_1.0.0_riscv64.deb (1.6 MB)
```

### 3. reCamera 设备端一键部署运行
```bash
# 上传并安装
scp dist/rebot-teleop_1.0.0_riscv64.deb root@192.168.16.1:/tmp/
ssh root@192.168.16.1 "dpkg -i /tmp/rebot-teleop_1.0.0_riscv64.deb && /etc/init.d/K92rebot-teleop restart"
```
连接由设备广播的热点（默认 SSID: `reCamera-Arm-Teleop`，密码: `12345678`），浏览器打开 `http://192.168.16.1:8765/` 即可通过触控驾驶舱开玩！

---

## 💻 命令行启动参数 (CLI Reference)

```text
armd - Minimal Robotic Arm Daemon & Teleop HAL for LeRobot

USAGE:
    armd [OPTIONS]

OPTIONS:
    --follower <MODEL>       Follower arm model: rebot-rs, rebot-dm, so-arm101, fashionstar, star-arm102, mock [default: mock]
    --follower-port <PORT>   Follower interface: can0, /dev/ttyUSB0, /dev/ttyS1
    --leader <MODEL>         未实现；传入即拒绝启动
    --leader-port <PORT>     未实现；传入即拒绝启动
    --port <PORT>            HTTP / WebSocket service port [default: 8765]
    -h, --help               Print help information
```

`--config` 接受 JSON 文件路径，必须包含完整的 `ArmConfig` 核心字段：`dof`、`joint_names`、`limits`（每项含 `name`、`min_position`、`max_position`、`max_velocity`、`max_acceleration`）、`home_positions`、`lease_timeout_ms`、`max_temperature`、`control_frequency_hz`。可选字段为 `resting_pose`、`following_error`、`max_jerk`、`gravity_model`；若出现则必须满足对应校验。驱动 DOF 固定为：`rebot-dm=7`（6 轴加 gripper），`rebot-rs/so-arm101/fashionstar/star-arm102=6`，`mock` 不强制。配置 DOF 与 follower 不符会拒绝启动。

停止动作只停止运动并保持 API 以便确认状态，不回零，也不强杀未确认失能的服务。三种串口/CAN 驱动的反馈和使能确认能力不同，必须以观测字段为准。

---

## 📄 开源许可证

本项目遵循 [MIT 许可证](LICENSE) 开源。欢迎提交 PR 适配更多开源机械臂与电机协议！
