# rebot-armd

`rebot-armd` 是专为嵌入式 RISC-V 计算平台（如 Seeed reCamera SG2002）与通用 Linux 系统设计的轻量级机械臂控制守护进程与遥控 HAL（Hardware Abstraction Layer）。系统集成 50Hz/100Hz 实时控制循环、五次多项式软限位轨迹平滑规划、看门狗租约急停安全状态机、HuggingFace LeRobot 标准 Parquet 示教录制、以及嵌入式 Web 触控驾驶舱与 REST / WebSocket 遥测接口。

---

## 1. 系统架构与核心特性

- **跨平台运行时**：原生支持 `riscv64gc-unknown-linux-musl` 静态二进制交叉编译，同时支持 `x86_64` 与 `aarch64` Linux/macOS 开发与仿真。
- **确定性控制循环**：基于 Tokio 与 `tokio::time::interval` 实现高精度控制周期调度，内置跳周期补偿与时间差限幅。
- **多级安全防护状态机**：
  - 具备 `Booting`, `Idle`, `Armed`, `Moving`, `EStop`, `Fault` 6 种状态。
  - 软限位检测、关节角速度与加速度限幅、超温熔断（Thermal Trip）、通信掉线回退（BusLossFallback）、客户端看门狗租约（Watchdog Lease Timeout）。
- **轨迹规划引擎**：五次多项式（Quintic Polynomial）多关节轨迹插值算法，保证起始与终点位置、速度、加速度连续且均为零。
- **具身智能数据集录制**：原生录制符合 HuggingFace LeRobot 标准的 Parquet 格式，零外部 Python 依赖。
- **一体化 Web 驾驶舱**：内嵌单文件 HTML/JS 界面，支持移动端多点触控双虚拟摇杆、2D 正向运动学（FK）骨骼姿态渲染、长按使能防误触与一键 E-Stop 急停。

---

## 2. 支持的电机协议

| 电机品牌/型号 | 物理总线 | 默认速率 | 控制模式 | 核心数据帧结构 |
|---|---|---|---|---|
| **达妙 (Damiao / DM-J 系列)** | CAN 2.0A | 1 Mbps | MIT 阻抗控制 / 位置速度 | 8 字节标准帧，16-bit 位置量化，12-bit 速度、力矩、$K_p$、$K_d$ 阻抗参数 |
| **飞特 (Feetech / STS3215 系列)** | 半双工 UART 异步串口 | 1,000,000 bps | 位置伺服 / 扭矩控制 | 帧头 `0xFF 0xFF`，指令包含 `PING`, `READ`, `WRITE`, `SYNC_WRITE`，累加和反码校验 |
| **RobStride (RS 系列: RS00/RS01/RS02)** | CAN 2.0B (29-bit 扩展帧) | 1 Mbps | MIT 模式 / 运控反馈 | 29-bit CAN ID 封装指令类型、主控 ID 与从机 ID；8 字节负载量化位置、速度与刚度 |
| **FashionStar (方成串口总线舵机)** | 半双工 UART 异步串口 | 115,200 bps | 角度控制 / 阻尼模式 | 请求帧头 `0x12 0x4C`，响应帧头 `0x05 0x1C`，0.1° 整数分辨率 |

### 2.1 达妙 (Damiao) 电机控制与反馈

- **MIT 控制帧**（8 字节）：
  - 字节 0~1：目标位置 $p_{\text{des}}$（16-bit 量化，映射至 $[-12.5, 12.5]\,\text{rad}$）
  - 字节 2~3（高 4 位）：目标速度 $v_{\text{des}}$（12-bit 量化，映射至 $[-30.0, 30.0]\,\text{rad/s}$）
  - 字节 3（低 4 位）~4：位置刚度增益 $K_p$（12-bit 量化，映射至 $[0.0, 500.0]$）
  - 字节 5~6（高 4 位）：速度阻尼增益 $K_d$（12-bit 量化，映射至 $[0.0, 5.0]$）
  - 字节 6（低 4 位）~7：前馈力矩 $T_{\text{ff}}$（12-bit 量化，映射至 $[-10.0, 10.0]\,\text{Nm}$）
- **控制指令**：
  - 使能：`0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC`
  - 失能：`0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD`
  - 保存零点：`0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFE`
  - 清除故障：`0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFB`

### 2.2 飞特 (Feetech STS3215) 协议规范

- **通信时序**：单线半双工 UART 异步串行通信，支持 1ms 轮询周期。
- **校验和计算**：$\text{Checksum} = \sim(\text{ID} + \text{Length} + \text{Instruction} + \sum \text{Params}) \ \& \ 0\text{xFF}$。
- **寄存器地址**：
  - 扭矩开关 (`REG_TORQUE_ENABLE`): `0x28` (1 字节)
  - 目标位置 (`REG_GOAL_POSITION`): `0x2A` (2 字节小端无符号整数)
  - 当前位置 (`REG_PRESENT_POSITION`): `0x38` (2 字节小端无符号整数)
  - 当前温度 (`REG_PRESENT_TEMPERATURE`): `0x3F` (1 字节)
- **角度步数换算**：分辨率为 4096 步对应 $2\pi\,\text{rad}$（即 $\text{steps} = \text{rad} \times \frac{2048}{\pi} + \text{offset}$）。

### 2.3 RobStride 电机扩展 CAN 帧结构

- **29-bit CAN ID 编码**：
  - Bit 28~24：通信类型 Mode（Type 1: 运控模式, Type 3: 使能, Type 4: 停机, Type 17: 读参数）
  - Bit 23~8：数据区 / 目标主控 ID 与参数地址
  - Bit 7~0：电机通信 ID（从机 ID）
- **状态反馈解码**：Type 2 反馈帧解析当前角度（16-bit）、角速度（12-bit）、力矩（12-bit）、驱动温度（8-bit, 步进 0.1℃）及故障掩码。

---

## 3. 传输层模式 (Transport Modes)

| 传输模式 | 实现类 | 适用环境 | 特性说明 |
|---|---|---|---|
| **SocketCAN** | `SocketCanTransport` | Linux 原生网络 CAN 栈 (`can0`, `vcan0`) | 支持非阻塞 I/O 与微秒级硬件时间戳，适用于 reCamera B4 SPI-CAN 底板 |
| **SLCAN** | `SlcanTransport` / `SlcanCodec` | 字符设备 (`/dev/ttyUSB*`, `/dev/ttyACM*`) | Lawicel ASCII 帧编码协议（`t`/`T` 传输，`r`/`R` 远程帧），兼容 USB-CAN 转换器 |
| **Serial** | `SerialTransport` | POSIX / Windows 标准串口设备 | 原生支持 1ms 超低延迟超时配置与波特率设置（115200 ~ 1000000 bps） |
| **Mock** | `MockTransport` / `MockCanTransport` | 单元测试与无硬件 CI/CD 流水线 | 纯内存无锁环形队列回环传输，支持单测中的数据帧拦截与状态断言 |

---

## 4. reCamera B4 CAN 底板接口与引脚定义

Seeed reCamera Gimbal 扩展底板（B4）通过 60-Pin BTB 连接器与 SG2002 核心板互联，板载工业级 SPI-CAN 控制器与高速收发器。

```
+---------------------------------------------------------------------------------------+
|                                reCamera SG2002 核心板                                  |
|  [SDIO_CLK/3V3]     [SDIO_CMD/3V3]     [SDIO_D0/3V3]     [SDIO_D3/3V3]     [GPIO12]   |
+--------+------------------+------------------+------------------+--------------+------+
         | SCK              | MOSI             | MISO             | nCS          | INT
+--------v------------------v------------------v------------------v--------------v------+
|                         Microchip MCP2518FD (SPI-CAN FD 控制器)                        |
|                                晶振: 40 MHz (10ppm, 15pF)                              |
+------------------------------------+--------------------------------------------------+
                                     | TXCAN / RXCAN
+------------------------------------v--------------------------------------------------+
|                        Microchip MCP2558FD (CAN FD 高速收发器)                         |
|                 供电: CAN_5V (VDD) / CAN_3V3 (VIO), 待机控制: S 引脚                   |
+------------------------------------+--------------------------------------------------+
                                     | 差分线 (CAN_H / CAN_L)
                                     +---> 共模扼流圈: L3 PCAQ4520MB-142
                                     +---> 分裂终端电阻: 62Ω + 62Ω (中点 4.7nF 接地)
                                     +---> TVS 静电浪涌防护: TPD1E10B06DPYR / SMBJ24CA
                                     |
               +---------------------+---------------------+
               |                                           |
               v                                           v
      [J2: CAN & 宽压电源输入]                     [J3: 扩展 CAN 输出接口]
      (JST SH 1.0mm 6-Pin 卧贴)                   (JST GH 1.25mm 4-Pin 锁扣)
```

### 4.1 核心板 60-Pin BTB 互联引脚

| 核心板信号 (Net) | BTB 引脚号 | MCP2518FD 引脚 | 信号方向 | 功能描述 |
|---|---|---|---|---|
| `SDIO_CLK/3V3` | Pin 35 | SCK (Pin 10) | 核心板 -> CAN | SPI 时钟总线 (通过 R22 0Ω 串联) |
| `SDIO_CMD/3V3` | Pin 33 | SDI (Pin 11) | 核心板 -> CAN | SPI 主出从入 (MOSI, 通过 R24 0Ω) |
| `SDIO_D0/3V3`  | Pin 29 | SDO (Pin 12) | CAN -> 核心板 | SPI 主入从出 (MISO, 通过 R25 0Ω) |
| `SDIO_D3/3V3`  | Pin 31 | nCS (Pin 13) | 核心板 -> CAN | SPI 片选信号 (通过 R23 0Ω) |
| `GPIO12`        | Pin 25 | ~INT (Pin 9) | CAN -> 核心板 | 中断输入信号 (通过 R26 0Ω) |
| `SD0_PWR_EN`   | Pin 27 | EN (U4)      | 核心板 -> 电源 | CAN 3.3V 负载开关使能 (TPS22916) |

### 4.2 外部硬件物理接口引脚定义

#### J2: CAN & 宽压电源输入接口 (SM06B-SRSS-TB, JST SH 1.0mm 6-Pin)
| 引脚号 (Pin) | 信号名称 (Net) | 电气特性 | 功能描述 |
|---|---|---|---|
| **1** | `VIN` | 12V ~ 24V DC IN (最大 3A) | 宽压直流供电正极（经磁珠 FB5 隔离） |
| **2** | `VIN` | 12V ~ 24V DC IN (最大 3A) | 宽压直流供电正极（并联增强通流） |
| **3** | `GND` | 电源与信号参考地 | 功率电源地 |
| **4** | `GND` | 电源与信号参考地 | 功率电源地 |
| **5** | `CAN_H` | ISO 11898 差分高 | 工业 CAN 总线差分高信号线 |
| **6** | `CAN_L` | ISO 11898 差分低 | 工业 CAN 总线差分低信号线 |

#### J3: 扩展 CAN 接口 (GH-1.25-4A-WT, JST GH 1.25mm 4-Pin)
| 引脚号 (Pin) | 信号名称 (Net) | 电气特性 | 功能描述 |
|---|---|---|---|
| **1** | `CAN_5V` | 5.0V DC OUT (最大 1A) | 辅助外设 5V 稳压供电输出 |
| **2** | `GND` | 电源与信号参考地 | 信号地 |
| **3** | `CAN_H` | ISO 11898 差分高 | 工业 CAN 总线差分高信号线 |
| **4** | `CAN_L` | ISO 11898 差分低 | 工业 CAN 总线差分低信号线 |

---

## 5. REST & WebSocket API 接口规范

默认监听端口：`8765`。所有接口支持 CORS 跨域请求。

### 5.1 机械臂状态与遥测

#### `GET /api/v1/observation`
获取当前 6 自由度机械臂完整状态与安全上下文。

- **响应格式 (200 OK)**：
```json
{
  "timestamp": 1730560000.123,
  "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
  "measured_positions": [0.0, 0.523, -1.047, 0.0, 0.785, 0.0],
  "measured_velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "commanded_positions": [0.0, 0.523, -1.047, 0.0, 0.785, 0.0],
  "torques": [0.12, -0.45, 0.33, 0.05, 0.02, 0.01],
  "temperatures": [35.0, 36.5, 37.0, 34.0, 33.5, 32.0],
  "safety_state": "Armed",
  "epoch_id": 42
}
```

#### `GET /api/v1/observation/schema`
获取关节配置与物理软限位定义。

- **响应格式 (200 OK)**：
```json
{
  "dof": 6,
  "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
  "limits": [
    {
      "name": "joint_1",
      "min_position": -3.14159,
      "max_position": 3.14159,
      "max_velocity": 2.0,
      "max_acceleration": 5.0
    }
  ]
}
```

### 5.2 力矩使能与心跳租约

#### `POST /api/v1/torque`
控制机械臂电机力矩使能状态。

- **请求体**：
```json
{
  "enable": true
}
```
- **响应体 (200 OK)**：
```json
{
  "is_torqued": true,
  "safety_state": "Armed"
}
```

#### `POST /api/v1/heartbeat`
客户端心跳续期（超时自动回退锁定当前姿态并进入安全停止）。

- **请求体**：
```json
{
  "client_id": "teleop_controller_01",
  "epoch_id": 42
}
```
- **响应体 (200 OK)**：
```json
{
  "status": "ok",
  "epoch_id": 42
}
```

### 5.3 运动控制接口

#### `POST /api/v1/motion/jog`
点动插值运动（调用五次多项式规划器平滑运动）。

- **请求体**：
```json
{
  "target_positions": [0.1, 0.2, -0.5, 0.0, 0.3, 0.0],
  "duration_secs": 2.5,
  "epoch_id": 42
}
```

#### `POST /api/v1/motion/home`
安全回归零位（Home 姿态）。

- **请求体**：
```json
{
  "duration_secs": 3.0,
  "epoch_id": 42
}
```

#### `POST /api/v1/motion/servo`
50Hz 高频直接伺服控制（用于遥控摇杆与主从示教）。

- **请求体**：
```json
{
  "targets": [0.12, 0.25, -0.48, 0.01, 0.32, 0.0],
  "epoch_id": 42
}
```

### 5.4 安全控制接口

#### `POST /api/v1/safety/e_stop`
软件紧急停止（锁定当前角度，切入 EStop 闭锁状态）。

#### `POST /api/v1/safety/clear_fault`
清除故障并恢复至 Idle 状态。

### 5.5 数据集录制接口

#### `POST /api/v1/dataset/record/start`
启动 LeRobot 数据集录制片段。
- **请求体**：
```json
{
  "episode_id": "episode_20261102_001",
  "output_path": "/userdata/datasets/episode_001.parquet"
}
```

#### `POST /api/v1/dataset/record/save`
停止并持久化当前录制片段至 Parquet 文件。

#### `POST /api/v1/dataset/record/discard`
放弃当前录制数据并重置录制状态机。

### 5.6 WebSocket 实时双向流

- **连接路径**：`ws://<host>:8765/api/v1/ws`
- **下行数据**：服务端按 50Hz 持续推送 `ArmObservation` JSON 格式字符串。
- **上行数据**：客户端发送心跳维持帧：
  ```json
  {
    "type": "heartbeat",
    "client_id": "ws_client",
    "epoch_id": 42
  }
  ```

---

## 6. LeRobot Parquet 录制格式规范

`rebot-armd` 原生生成符合 HuggingFace [LeRobot](https://github.com/huggingface/lerobot) 标准规范的 Parquet 数据集文件，无第三方 Python 环境开销。

### Arrow Schema 结构定义

```text
Schema {
    fields: [
        Field { name: "timestamp", data_type: Float64, nullable: false },
        Field {
            name: "observation.state",
            data_type: List(Field { name: "item", data_type: Float32, nullable: true }),
            nullable: false
        },
        Field {
            name: "action",
            data_type: List(Field { name: "item", data_type: Float32, nullable: true }),
            nullable: false
        }
    ]
}
```

- **压缩与格式特征**：
  - 数据块压缩：Snappy（压缩比高、解压极快）。
  - 对齐协议：`observation.state` 存储当前从手测得物理弧度；`action` 存储对应控制周期下发的目标弧度。
  - 单条录制回读验证：原生提供 `write_frames_to_parquet` 与 `read_frames_from_parquet` 严格双向无损校验。

---

## 7. 编译、打包与部署指南

### 7.1 本地编译与单元测试

```bash
# 本地架构编译检查
cargo check

# 运行全套 49 个单元与集成测试（涵盖达妙、飞特、RobStride、SLCAN、多项式规划与 Parquet 编解码）
cargo test
```

### 7.2 RISC-V 目标交叉编译与 Debian 包打包

使用自动化打包脚本在 Linux ARM64 或 x86_64 宿主机上一键构建 `riscv64gc-unknown-linux-musl` 架构 Debian 包：

```bash
# 赋予执行权限并构建
chmod +x scripts/build_deb.sh
./scripts/build_deb.sh 1.0.0
```

- 脚本自动执行以下操作：
  1. 调用 `cargo build --release --target riscv64gc-unknown-linux-musl` 生成静态剥离二进制。
  2. 组装 `rootfs` 目录结构（包含 `/etc/init.d/K92rebot-teleop`, `/usr/share/rebot-teleop/`, `/usr/local/bin/armd`）。
  3. 动态计算 `Installed-Size` 并设置 root 所有权。
  4. 使用 `dpkg-deb -Zgzip` 生成符合 opkg 与 dpkg 规范的 `dist/rebot-teleop_1.0.0_riscv64.deb`。

### 7.3 reCamera 设备端部署与运行

```bash
# 1. 传输 deb 安装包至 reCamera
scp dist/rebot-teleop_1.0.0_riscv64.deb root@192.168.16.1:/tmp/

# 2. 在 reCamera 执行安装
dpkg -i /tmp/rebot-teleop_1.0.0_riscv64.deb

# 3. 启动后台服务 (自动启动 Wi-Fi 热点并加载硬件驱动)
/etc/init.d/K92rebot-teleop start

# 4. 查看服务状态与日志
/etc/init.d/K92rebot-teleop status
tail -f /var/log/rebot-teleop.log
```

服务启动后，手机或电脑连接由 reCamera 广播的热点（默认 SSID: `reCamera-Arm-Teleop`, 密码: `12345678`），在浏览器中打开 `http://192.168.16.1:8765/` 即可进入操控界面。

---

## 8. CLI 命令行参数参考

`armd` 支持直接在命令行指定硬件驱动型号与通信端口：

```text
armd - Minimal Robotic Arm Daemon & Teleop HAL

USAGE:
    armd [OPTIONS]

OPTIONS:
    --follower <MODEL>       Follower arm model: rebot-rs, rebot-dm, so-arm101, mock [default: mock]
    --follower-port <PORT>   Follower interface: can0, /dev/ttyUSB0, /dev/ttyS1
    --leader <MODEL>         Leader teleop arm model: star-arm102, so-arm100
    --leader-port <PORT>     Leader serial interface: /dev/ttyACM0, /dev/ttyUSB1
    --port <PORT>            HTTP / WebSocket service port [default: 8765]
    -h, --help               Print help information
```
