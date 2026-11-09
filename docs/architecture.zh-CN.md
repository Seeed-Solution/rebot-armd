# 架构：核心、套件与 agent

[English](architecture.md) | 中文

rebot-armd 分三层。核心可在任何受支持的控制器上运行；套件把核心适配到一种硬件组合；agent 只通过核心的 API 通信。

```text
 agent 层     rebot-companion（SDK、MCP 服务）、浏览器驾驶舱、脚本
                     │ 只用 HTTP / WebSocket :8765
 核心         armd：控制循环、安全监督、驱动、API、检测代理
                     │ 契约：机械臂 profile、感知服务 API、
                     │       能力发现、打包
 套件         kits/<name>/：profile、打包、可选的感知进程、
              硬件、标定、文档（每种 控制器 + 机械臂 + 相机 + 支架 组合一个）
```

| 层 | 内容 | 位置 |
|---|---|---|
| 核心 | `armd`，最小运行时。在其自带驱动范围内与硬件无关（`--follower`：`rebot-dm`、`rebot-rs`、`so-arm101`、`fashionstar`、`star-arm102`、`mock`） | 仓库根目录：`src/`、`tests/`、`Cargo.toml` |
| 套件 | 一种适配：控制器 + 机械臂 + 相机 + 支架。只提供数据和打包，不改核心代码 | `kits/recamera-b601dm/`（官方）；第三方套件在各自仓库 |
| agent 层 | [rebot-companion](https://github.com/Seeed-Solution/rebot-companion) SDK 与 MCP 服务、网页驾驶舱、脚本 | 只使用 armd 的 `/api/v1/*` 与 `/api/v1/ws` |

核心 crate 保留在仓库根目录（已按此结构公开发布），套件放在旁边。

## 两种构建

| 构建 | 内容 | 命令 | 产物 |
|---|---|---|---|
| 最小运行时 | 只有核心 | `./scripts/build_deb.sh --flavor lite` | `dist/armd-lite_<version>_riscv64.tar.gz`：`armd`、`LICENSE`、`THIRD_PARTY_NOTICES.md` |
| reCamera 运行时 | 核心 + `kits/recamera-b601dm` | `./scripts/build_deb.sh`（`--flavor recamera`，默认） | `dist/rebot-teleop_<version>_riscv64.deb` |

两者使用同一个 `armd` 二进制（riscv64gc-unknown-linux-musl，静态链接）。驾驶舱（`src/api/index.html`）和 JMuxer 嵌入在二进制中，所以 lite 构建同样提供驾驶舱。主机构建（`cargo build --release`）可产出 x86_64 等目标的核心，脚本不负责打包。

## 契约

核心与套件只通过以下四个契约交互。版本号为编写本文时代码中的值。

### 1. 机械臂 profile

用 `armd --config <file>` 加载的 `ArmConfig` JSON（`src/model.rs`）。不带 `--config` 时，`rebot-dm` 使用内置的 `ArmConfig::rebot_dm()`，其他 follower 使用通用 6 关节 profile。

- 版本：`profile_version`，当前为 **1**（`ARM_PROFILE_VERSION`）。没有该字段的 profile 视为版本 1；更高版本在启动时被拒绝。
- 顶层未知字段被忽略（`grasp` 段拒绝未知字段）；所有可选字段都有 serde 默认值。

| 字段 | 含义 | 必填 |
|---|---|---|
| `profile_version` | 契约版本 | 否（默认 1） |
| `dof`、`joint_names`、`limits`（`name`、`min_position`、`max_position`、`max_velocity`、`max_acceleration`）、`home_positions` | 关节与限位；`dof` 必须与 follower 一致（`rebot-dm` 为 7，其他为 6，`mock` 不限） | 是 |
| `lease_timeout_ms`、`max_temperature`、`control_frequency_hz` | 租约看门狗、温度保护、控制频率（1..1000 Hz） | 是 |
| `resting_pose` | 安全卸力的姿态与阈值；为 null 时禁止自动卸力 | 否 |
| `following_error` | `max_error_rad`、`persistence_ms` | 否（0.5 rad，500 ms） |
| `max_jerk`、`planned_max_velocity` | 轨迹加加速度偏好、规划运动限速 | 否 |
| `kinematics` | 运动学模型（`RebotDevArm`）；为 null 时关闭笛卡尔动作和工作空间检查 | 否 |
| `workspace` | 笛卡尔安全盒（`floor_z_m`、`box_min_m`、`box_max_m`、`max_reach_m`、`check_points_end_m`） | 否 |
| `gravity_model` | 重力模型（`RebotDevArm`） | 否 |
| `gravity_comp` | 重力补偿策略（渐入、比例、阻尼、力矩与速度限制、保持增益）；为 null 表示不支持 | 否 |
| `damiao_ranges` | 每个关节的达妙映射范围 `{p_max, v_max, t_max}`；为 null 时为 ±12.5 rad、±30 rad/s、±18 N·m | 否 |
| `gripper_policy` | 夹爪动作（闭合 / 张开角度、保持间隙与力矩、挤压、稳定时间） | 否 |
| `grasp` | 点框抓取设置（`src/grasp.rs` 的 `GraspSetup`：高度、夹爪像素点、相机俯角、内参占位值、对中与超时参数） | 否 |
| `camera_mount` | 相机相对 end_link 的外参：`flip`、`rot_deg`、`tilt_deg`、`offset_m` | 否（全为零） |

细节与单位：[control_safety_contract.md](control_safety_contract.md#configuration--配置)。

目前不在 profile 中：

- URDF：`gravity_model` 选择编译进 armd 的模型。启动时 armd 在固定路径（`assets/`、`/usr/share/rebot-teleop/assets/`、`/usr/share/rebot-teleop/`）校验 `reBot-DevArm_gripper.urdf` 的 sha256；reCamera 套件的 deb 负责安装该文件。路径不是 profile 字段。
- 抓取设置也可来自 `ARMD_GRASP_SETUP`（运行时修改并持久化）。

官方套件把 B601-DM profile 以数据形式提供：`kits/recamera-b601dm/profile/rebot-b601dm.json`；与 `rebot_dm()` 不一致时 `tests/test_arm_profile.rs` 失败。

### 2. 感知服务 API

armd 不做推理，而是在自身端口下代理检测程序的 HTTP API，使驾驶舱和 agent 只访问一个源（`src/api/proxy.rs`，[detection_api.md](detection_api.md)）。

- 版本：感知服务在 `:8090` 的每个响应中返回 `"api_version"`。**2** 为 `rebot-teleop` 1.3.0 起套件 `rebot-vision` 的接口：视觉按调用方按需运行（单次触发指定流水线，持续推理通过订阅进行、不续期即过期，模式作为默认订阅）。**1** 为 `rebot-vision` 1.2（一个全局模式、每种模式都支持单次触发、合并结果）。没有该字段的服务（yolo-detector、aruco-detector）视为版本 0：下表中除 `/api/mode` 和订阅路由外的路由。armd 将其作为 `capabilities.vision.api_version` 发布。
- 上游：`ARMD_DETECTOR_URL`（默认 `http://127.0.0.1:8090`）。

| armd 路由 | 上游 | 用途 |
|---|---|---|
| `GET` / `POST /api/v1/detection/inference` | `/api/inference` | 持续推理开关、帧率 |
| `GET /api/v1/detection/latest` | `/api/results/latest?after=&wait_ms=` | 最新结果，长轮询 |
| `POST /api/v1/detection/trigger` | `/api/trigger?wait_ms=`（JSON 请求体透传） | 对新帧推理一次；版本 2 运行请求体中的 `pipelines`；不改变任何持续推理 |
| `GET` / `POST /api/v1/vision/mode` | `/api/mode` | 模式 `off` / `yolo` / `aruco` / `both`，以及持续推理开关和帧率（版本 1）；版本 2 中为默认订阅，`fps` 为所有订阅的帧率上限 |
| `POST /api/v1/vision/subscribe` | `/api/subscribe` | 创建或更新某个调用方的持续推理（版本 2） |
| `POST /api/v1/vision/subscribe/{id}/keepalive`、`DELETE /api/v1/vision/subscribe/{id}` | `/api/subscribe/{id}[/keepalive]` | 续期、结束订阅（版本 2） |
| `GET /api/v1/vision/subscriptions` | `/api/subscriptions` | 有效订阅及实际运行内容（版本 2） |
| `GET /api/v1/detection/ws`、`/results` | `ws://127.0.0.1:8001/results` | 检测结果流（固定地址） |

状态码和响应体原样透传；上游不可达返回 502。视频不经代理：驾驶舱直接播放相机的 H.264 WebSocket 流（`ws://<host>:8001/`）。目前 reCamera 上的感知服务是相机自带的 `yolo-detector`（或 `aruco-detector`），不属于套件。套件自带的感知服务是 `rebot-vision`（`kits/recamera-b601dm/vision/`，设计见 [dev_flow_spec_vision_runtime.md](dev_flow_spec_vision_runtime.md)），随 `rebot-teleop` 1.2.0 发布：一个进程占用相机，在同一帧上运行 YOLO（TPU）和 ArUco（CPU）。1.3.0（版本 2）起没有调用方时不做推理：单次触发运行其指定的流水线；持续推理运行所有有效订阅流水线的并集，帧率取请求的最大值并受配置帧率限制。结果始终同时包含 `detections` 和 `markers`。

调用方不再修改全局模式（只有一个全局模式时，最后恢复的调用方生效）。抓取动作每次对准触发一次 `{"pipelines": ["yolo"]}`；驾驶舱叠加层开启且页面可见时持有自己的订阅；手眼标定抽屉打开期间持有一个 ArUco 订阅，每次采样用一次 `["aruco"]` 单次触发。上游为版本 1 时，抓取仍切换到 `yolo`（原来开着 ArUco 时为 `both`）并在结束后恢复，驾驶舱仍使用模式；没有 `/api/mode` 的服务（404）仍使用推理开关。

### 3. 能力发现

`GET /api/v1/observation/schema` 带有 `capabilities`（`src/model.rs` 中的 `Capabilities`），驾驶舱与 MCP 客户端可据此调整，无需探测路由。旧版 armd 的 schema 没有 `capabilities`，解析结果为全 false、`version` 为 0。

- 版本：`capabilities.version`，当前为 **3**（`CAPABILITIES_VERSION`）。版本 2 在 `detection_proxy` 之后加入了 `vision.*` 字段；版本 3 加入了 `vision.per_request_pipelines` 和 `vision.subscriptions`。

| 字段 | 为 true 的条件 |
|---|---|
| `profile_version` | （数值）已加载 profile 的版本 |
| `cartesian` | 7 轴 profile 且设置了 `kinematics`（笛卡尔动作） |
| `workspace_check` | 设置了 `kinematics` 和 `workspace` |
| `gripper` | 设置了 `gripper_policy` 且存在第 7 个关节（`/api/v1/action/gripper`） |
| `grasp` | 7 轴，且设置了 `kinematics`、`gripper_policy`、`grasp`（`/api/v1/action/grasp`） |
| `gravity_comp` | profile、URDF 校验和驱动都支持该模式；与 `observation.gravity_comp.supported` 相同 |
| `damiao_ranges` | profile 设置了分关节达妙范围（启动检查结果在 `observation.driver_range_check`） |
| `vision.detection_proxy` | 恒为 true：检测路由存在；不检查感知服务是否可达 |
| `vision.mode_endpoint` | （字符串）恒为 `/api/v1/vision/mode` |
| `vision.modes` | （列表）感知服务报告的模式；服务不可达或没有 `/api/mode` 时为空 |
| `vision.mode` | （字符串或 null）当前基础模式 |
| `vision.api_version` | （数字或 null）感知接口版本 |
| `vision.trigger`、`vision.yolo_available` | （布尔或 null）每种模式都支持触发；YOLO 模型已加载 |
| `vision.per_request_pipelines` | 感知服务运行单次触发指定的流水线（版本 2） |
| `vision.subscriptions` | 感知服务提供 `/api/v1/vision/subscribe`（版本 2） |

`vision.modes` 等字段来自感知服务的 `GET /api/mode`：schema 使用最近一次经 armd 代理的模式请求的响应，超过 2 秒则向服务查询（超时 500 ms）。

schema 的其他字段已描述机械臂：`dof`、`joint_names`、`limits`、`workspace`、`camera_mount`、`kinematics`。随观测变化的状态仍在观测中（`driver_capabilities`、`gravity_comp`、`driver_range_check`）。

### 4. 打包

套件提供：armd 二进制（核心不改）、profile、可选的旁路进程（感知服务）、启动脚本；启动脚本启动这些进程并运行 `armd --follower <model> [--follower-port <port>] --config <profile>`。

- 版本：代码中未版本化；下面的布局为本文档的第 1 版。
- armd 从环境变量读取 `PORT`、`RUST_LOG`、`ARMD_DETECTOR_URL`、`ARMD_GRASP_SETUP`。

官方套件的安装包（`kits/recamera-b601dm/packaging/`，deb `rebot-teleop` 1.2.0）：

| 安装路径 | 仓库中的来源 |
|---|---|
| `/usr/bin/armd`，符号链接 `/usr/local/bin/armd` | 核心构建 |
| `/usr/bin/rebot-vision` | `kits/recamera-b601dm/vision/`，由 `scripts/build_vision.sh` 构建 |
| `/etc/init.d/K92rebot-teleop`（postinst 仅在没有 reCamera 控制台时添加 `S92rebot-teleop`） | `packaging/rootfs/etc/init.d/K92rebot-teleop` |
| `/usr/share/rebot-teleop/rebot-vision-run`（重启循环，nice 10）、`vision.json.default` | `packaging/rootfs/usr/share/rebot-teleop/` |
| `/usr/share/doc/rebot-teleop/`（`LICENSE`、`LICENSE-APACHE`、`THIRD_PARTY_NOTICES.md`） | 仓库根目录、`kits/recamera-b601dm/vision/LICENSE-APACHE` |
| `/usr/share/rebot-teleop/index.html` | `src/api/index.html` |
| `/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf` | `packaging/rootfs/usr/share/rebot-teleop/assets/` |
| `/usr/share/rebot-teleop/profile/rebot-b601dm.json` | `kits/recamera-b601dm/profile/` |
| `/usr/share/rebot-teleop/rebot-teleop.json`（及 `.md`、`.zh.md`） | reCamera 控制台的应用列表 manifest，`"requires": ["can"]` |
| `DEBIAN/control`、`postinst`、`prerm` | `packaging/debian/` |

init 脚本只对 `rebot-dm` follower、且 profile 文件存在时才传 `--config`。

## 仓库布局

```text
src/, tests/, Cargo.toml     核心 crate（armd）
scripts/build_deb.sh         --flavor lite | recamera
docs/                        核心文档与契约（本文）
kits/
  _template/                 第三方套件模板与说明
  recamera-b601dm/           官方参考套件
    profile/                 机械臂 profile JSON
    vision/                  rebot-vision 进程（C++，以 sscma-example-sg200x 为 SDK 构建）
    packaging/               rebot-teleop deb 源文件
    hardware/                相机支架 CAD，CERN-OHL-W-2.0（单独 LICENSE）
    calibration/             各支架相机俯角、内参占位值、抓取设置默认值
    docs/                    接线、组装、已验证能力
```

## 制作第三方套件

1. 在自己的仓库中以 [`kits/_template/`](../kits/_template/README.md) 为起点。
2. 为机械臂编写 profile（`dof` 必须与 armd 自带的某个 follower 一致）。用 mock follower 启动检查，并从 schema 读取 `capabilities`。
3. 可选：提供实现契约 2 路由的感知进程，并把 `ARMD_DETECTOR_URL` 指向它。
4. 打包 armd 发布二进制（lite tarball，或为目标平台构建的核心）、profile、旁路进程和启动脚本。
5. 在 README 中写明依赖的 armd 版本、`profile_version`、`capabilities.version`、感知路由，以及在实机上验证过的内容。

新的电机协议或运动学模型属于核心改动（`src/` 中的驱动或模型），不属于套件。

## 官方与第三方

| | 官方 | 第三方 |
|---|---|---|
| 位置 | 本仓库 `kits/` | 作者自己的仓库 |
| 目前 | `kits/recamera-b601dm`（reCamera + B601-DM） | 暂无 |
| 构建 | `scripts/build_deb.sh` | 套件自己的脚本，基于 armd 发布产物 |
| 测试 | 本仓库 CI（`cargo test`、profile 与 init 脚本测试） | 套件作者 |
| 许可证 | 代码 MIT；套件硬件 CERN-OHL-W-2.0 | 作者自定 |
