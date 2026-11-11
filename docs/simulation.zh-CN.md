# 仿真

[English](simulation.md) | 中文

`armd --follower sim` 让同一个 armd 二进制连接 reBot B601-DM 的 MuJoCo 物理仿真，而不是 CAN 总线。SDK、MCP server 和 cockpit 照常连接 armd；armd 自身的检查（关节限位、工作空间、租约看门狗、夹爪堵转 / 夹持判定、残差力矩、总线丢失检测，以及含碰撞守护的分支上的碰撞守护）基于仿真的位置、速度和力矩运行。

## 运行

需要 Rust 工具链和 [uv](https://docs.astral.sh/uv/)。仿真器是 `sim/mujoco` 下的 uv 项目（依赖 `mujoco`、`numpy`、`websockets`，相机视频流另需 `av`（PyAV，含 libx264））。

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
| `--damping D1,..,D6` | 用给定值（N·m·s/rad）替换场景中 J1..J6 的关节阻尼 |
| `--frictionloss F1,..,F6` | 用给定值（N·m）替换场景中 J1..J6 的库仑摩擦 |
| `--objects K1,..` | 场景物体，`cube`、`cup`、`block` 中 1..3 个（默认 `cube`）；见「相机、感知与场景」 |
| `--seed N` | 物体摆放种子（默认：固定布局） |
| `--camera` | 腕部相机：H.264 视频流在 `--video-port`（8001），感知接口和 `/sim/*` 在 `--perception-port`（8090），绑定 `--http-host`（127.0.0.1） |
| `--fps N`、`--camera-size` | 相机帧率 1..30（默认 15）；视频尺寸 `1280x720`（默认）或 `640x360` |
| `--profile P`、`--camera-rot-deg`、`--camera-tilt-deg`、`--camera-offset X,Y,Z` | 相机安装：取 profile 的 `camera_mount`（默认套件 profile），再按这些参数覆盖 |
| `--scene-stream` | 另在 `ws://<host>:<video-port>/scene` 推送固定的全景相机 |
| `--torque-report instant\|mean` | 状态报文中的力矩：瞬时值（默认），或最近一个指令周期内的平均值（滑动窗口；见「摩擦辨识」） |

`REBOT_DEVARM_DIR=<reBot-DevArm 仓库路径>` 时，把上游连杆网格（`Rebot_Arm_description/DM/meshes/collision`）作为可视几何加入窗口显示；物理计算不使用它们。

## 测试

```bash
cargo test --lib sim_driver            # 驱动 + 假 UDP 对端
cargo test --test test_sim_fk          # armd 正运动学 vs MuJoCo end_link 位姿
cd sim/mujoco && uv run pytest -s      # 仿真器 + armd 端到端，FK fixture 是否最新，sysid 回收
cd sim/mujoco && uv run pytest -s -m "not slow"   # 跳过约 3.5 分钟的 sysid 回收测试和抓取端到端测试
```

`tests/test_camera.py` 检查视频流封装，并分别用 PyAV 和 rebot-companion 的 `snapshot` 解码器解码（需要 `REBOT_COMPANION_DIR` 或仓库旁的 checkout，否则跳过），检查检测框与物体投影中心、感知接口各路由和按种子复位。`tests/test_camera_grasp.py`（标记 `slow`）跑点框抓取端到端（见下）。

端到端测试默认先 `cargo build`，设置 `ARMD_BIN` 时直接用该二进制。流程：上使能、回 home、张开夹爪、工具朝下 `move_to`、检查 J2 / J3 重力力矩、下降到 4 cm 方块、`grasp` 闭合（期望 `holding: true`）、抬起、杀掉仿真器，期望故障码 `BUS_LOSS`。

## 仿真内容

- **场景**（`sim/mujoco/models/rebot_b601dm_scene.xml`）：关节链与连杆惯量取自 `assets/reBot-DevArm_gripper.urdf`（armd 重力模型和运动学用的同一组参数）；碰撞体为按上游 DM 网格尺寸设置的方盒；桌面在 z = 0（底座平面，对应 profile `grasp.table_z_m`）；4 cm、30 g 方块位于 (0.30, 0, 0.02) m。连杆与桌面、方块碰撞，连杆之间不碰撞。
- **关节约定**：J1..J6 电机角 = URDF 角。J7 经齿条驱动两指：单侧开口 = −q7 × 0.0425 / 5.7 m（q7 = 0 闭合，−5.7 rad = 85 mm）。另一指由等式约束跟随。J2、J3 上限位在 0 rad（URDF 限位），机械臂在 q = 0 折叠静置；其余关节范围为 profile 限位 + 0.1 rad。
- **电机**（物理 500 Hz）：
  - POS_VEL（armd 常规模式）：v_des = clip(25 · (q_target − q), ±vlim)，力矩 = kv · (v_des − dq) + J1..J6 的模型偏置力矩（重力、科氏力）；vlim 为 armd 下发的 2.0 rad/s。关节处于行程端点 0.005 rad 以内、目标也在该端点且重力把它压向该端点时，不加偏置力矩：由限位承担负载。折叠的 home 姿态下 J2 / J3 以这种方式靠在上限位（q = 0）上，电机力矩接近 0，与真机一致，armd 的下使能稳定检查（`support_torque_limits`）因此通过。
  - MIT（重力补偿按关节切换）：力矩 = kp · (q_target − q) + kd · (dq_target − dq) + τ_ff。
  - 失能：无力矩，机械臂在重力下下落。
  - 力矩上限 = 达妙峰值扭矩：27 N·m（J1..J3）、7 N·m（J4..J7）。POS_VEL 的速度目标限制在电机空载转速以内（5.45 / 20.9 rad/s；armd 下发 vlim 2.0，正常使用时该限制不起作用）。
  - 转子惯量经减速比折算为关节 armature（见下表）。
- **反馈**（250 Hz）：位置、速度、电机总力矩（伺服 + 前馈，因此包含重力和负载）、温度恒定 35 °C、使能状态。

### 电机参数

各关节电机来自 reBot B601-DM 的 BOM（[hardware/reBot_B601_DM/readme.md](https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/reBot_B601_DM/readme.md)：DM4310(V4) × 4、DM4340P(V4) × 3；v1.1 更新记录把关节 1 从 4310 改为 4340P）。J1..J3 = DM4340P、J4..J7 = DM4310 的划分与 2026-11-09 从机械臂读出的 PMAX / VMAX / TMAX 寄存器一致（J1..J3 VMAX 10、TMAX 28；J4..J7 VMAX 30、TMAX 10；见 [control_safety_contract.md](control_safety_contract.md)），也与 Seeed wiki 中各型号的默认值一致（4340P：VMAX 8、TMAX 28；4310：VMAX 30、TMAX 10）。

来源：[S1] Seeed wiki 达妙系列参数表 https://wiki.seeedstudio.com/damiao_series/ ；[S2] 达妙 DM-J4340P-2EC 使用说明书 V1.0（2025-01-10）https://doc.switch-science.com/media/files/5e033168-99f1-4d2d-b758-e03192a1f071.pdf ；[S3] Seeed 商品页 DM4310 / DM4340P 参数表 https://www.seeedstudio.com/Damiao-4310-Actuator-Motor-p-6823.html ；[S4] 达妙 DM-J4310-2EC V1.1 使用说明书 https://sharingwin.com/wp-content/uploads/2025/09/DM-J4310-2EC-V1.1-Gear-Motor-User-Manual-V1.0.pdf （参数表为图片，数值取自 S1 / S3）。

| 参数 | J1..J3 DM-J4340P-2EC | J4..J7 DM-J4310-2EC V1.1 | 来源 | 仿真中的用途 |
|---|---|---|---|---|
| 减速比 | 40 | 10 | S1、S2、S3 | armature |
| 额定 / 峰值扭矩（输出端） | 9 / 27 N·m | 3 / 7 N·m | S1、S2、S3 | `forcerange`、`TORQUE_LIMIT` = 峰值 |
| 24 V 额定 / 空载转速 | 36 / 52 rpm | 120 / 200 rpm | S2、S3 | v_des 上限 5.45 / 20.9 rad/s |
| 扭矩常数（输出端） | 4.074 N·m/A | 0.945 N·m/A | S1 | 未使用（无电流模型） |
| 转子惯量（电机侧） | 2.0e-5 kg·m² | 1.8e-5 kg·m² | S1 | armature |
| armature = J_rotor × N² | 0.032 kg·m² | 0.0018 kg·m² | 计算 | 关节 `armature`（J1..J6） |
| J7 折算到夹指滑轨的 armature | — | 0.0018 × (5.7 / 0.0425)² = 32.4 kg，每指 16.2 kg | 计算 | `finger_left` / `finger_right` 的 `armature` |
| 相电阻 / 相电感 | 0.88 Ω / 360 µH | 0.85 Ω / 345 µH（S3：0.65 Ω） | S1、S2、S3 | 未使用 |
| 关节阻尼 | 0.8 N·m·s/rad | 0.8 N·m·s/rad | 假设：reBot-DevArm DM MJCF 默认值，无达妙数据 | `damping` |
| 关节摩擦（库仑） | 0.05 N·m | 0.05 N·m | 假设，无达妙数据 | `frictionloss` |
| POS_VEL 环路 KPOS / KV | 25 1/s、8 N·m·s/rad | 25 1/s、1.5（J7 0.5）N·m·s/rad | 假设（S1 给出的速度环 KP 为驱动内部单位，缺少电流环标定无法换算） | `server.py` |
| 夹指阻尼 / 摩擦 | — | 2 N·s/m / 0.2 N | 假设 | 夹指关节 |

阻尼和库仑摩擦可用 `rebot-sysid`（见下文）在机械臂上辨识，再通过 `--damping` / `--frictionloss` 传给仿真器。

### 通信协议（UDP，JSON，版本 1）

每个数据报一个 JSON 对象。

| 方向 | 消息 |
|---|---|
| armd → sim | `{"v":1,"type":"cmd","seq":N,"enable":bool,"targets":[T × 7] \| null}`，每个控制周期及使能 / 失能时发送（`targets: null`） |
| armd → sim | `{"v":1,"type":"poll","seq":0,...}`，无其他报文时每 20 ms 一次保活 |
| sim → armd | `{"v":1,"type":"state","session":S,"seq":N,"ack":N,"enabled":bool,"q":[7],"dq":[7],"tau":[7],"temp":[7],"t":s}`，发往最近一个数据报的来源地址 |

`T` 为 `{"mode":"pos_vel","q":rad,"vlim":rad/s}` 或 `{"mode":"mit","q":rad,"dq":rad/s,"kp":N·m/rad,"kd":N·m·s/rad,"tau":N·m}`。`ack` 为最近应用的指令 `seq`；`session` 在每次启动仿真器时随机生成。

armd 侧（`src/sim_driver.rs`）：后台线程把状态收进缓存，控制周期内的驱动调用不等待网络。版本不符、数组长度不符、含非有限值、或同一 session 内 `seq` 未递增的状态报文被丢弃。反馈超过 200 ms 时 `is_feedback_valid` 为 false、使能状态为 Unknown；连续 500 ms（`BUS_LOSS_TIMEOUT`）无新反馈时监督器锁存 `BUS_LOSS`，与硬件一致。上使能最多等待 80 ms 收到确认该指令的状态（失能 25 ms），与 Damiao 握手相同。

## 相机、感知与场景

`rebot-sim --camera` 加入腕部相机，对外接口与 reCamera 套件一致，驾驶舱、armd 检测代理和 rebot-companion 客户端无需改动：

```bash
cd sim/mujoco
uv run rebot-sim --camera --objects cube,cup --seed 3 --stats 2
ARMD_DETECTOR_URL=http://127.0.0.1:8090 cargo run --bin armd -- --follower sim \
  --config kits/recamera-b601dm/profile/rebot-b601dm.json --log-dir off --blackbox-dir off
```

### 相机

- **安装**：沿用 armd 约定（`kinematics::cam_mount`，OpenCV 相机坐标轴）：R_end_cam = [x0 y0 z0] · Rz(`rot_deg`) · Rx(`tilt_deg`)，z0 = end_link x（工具轴），y0 = q6 = π/2 时 J5 轴在 end_link 中的方向。数值取 profile 的 `camera_mount`（套件 profile：v5，`rot_deg` 0，`tilt_deg` 0），可用 `--camera-rot-deg` / `--camera-tilt-deg` 覆盖（armd 符号：负值使光轴朝夹爪倾斜）。镜头中心为 `camera_mount.offset_m`；套件 profile 中为 null（未测量），此时仿真器使用 v8 支架假定的镜头中心 end_link (−0.0731, 0.0005, 0.0660) m（reBot-DevArm `hardware/camera-mounts/recamera-mounts/v8/README.txt` 第 4 节）。
- **光学**：针孔模型，方形像素，无畸变。VFOV 24.65°，HFOV 42.46°（OV5647 1080p 窗口 2.688 × 1.512 mm，f 3.46 mm；由 v8 README 推导，未用标定板测量）；640x360 下 fx = fy = 824 px。
- **与 v8 分析对照**：v8 支架（`--camera-rot-deg 90 --camera-tilt-deg -38`）、home 位姿、加载上游网格（`REBOT_DEVARM_DIR`）：闭合夹爪尖位于 1280x720 的像素 (648.5, 477.5)，夹爪占画面 10.8 %，最高行 472；v8 README 为 (648, 477)、10.8 %、472 行。不加载网格时夹爪显示为碰撞方盒（比收窄的指尖厚：占 22.5 %，最高行 337）；加载网格时方盒移到 geom 组 3，相机不渲染该组。
- **渲染**：MuJoCo `Renderer` 离屏渲染，运行在独立的相机进程中（由 `rebot-sim` 启动，同时运行视频流和 HTTP 服务）。`rebot_sim/__init__.py` 找到 libEGL 时设 `MUJOCO_GL=egl`，否则找到 libOSMesa 时设 `osmesa`；显式设置的 `MUJOCO_GL` 优先。物理循环每步把 qpos / qvel / 时间写入共享内存，写入时持有一个多进程锁，相机进程复制时也持有该锁；相机进程每帧复制到自己的 MjData，并通过管道把 `/sim/reset`、`/sim/state` 交给物理循环处理（每 10 步即 20 ms 轮询一次）。相机只在物理步数比上一帧前进时渲染新帧；结果带该帧的 `physics_time` 和 `physics_step`。相机进程在渲染器和编码器创建完成、且一次 RGB 渲染和一次分割渲染成功后才报告就绪。分割以 ID 颜色渲染，解码时做范围检查（超过场景 geom 数的值被拒绝）。分割失败的帧记录日志（帧号、取值、ngeom、nbody）并计数；该帧视频照常发送，但不产生感知结果，下一帧重试；连续 10 次失败时相机进程退出。间歇性越界读回（mujoco 4.0.0、EGL，出现在相机进程的最初几帧）的原因尚不清楚。相机进程退出或其管道关闭时，仿真器结束物理循环，`rebot-sim` 以退出码 1 退出；仿真器进程退出后相机进程随之退出。嵌入使用时 `Simulator.close()` 停止相机进程并释放管道、套接字和共享内存。
- 相机画面中隐藏 tcp 标记点。

### 视频流（`ws://<host>:8001/`）

格式同 reCamera 调试流（rebot-companion `snapshot.py`、驾驶舱 `setupWS`）：每个 H.264 访问单元一条二进制 WebSocket 消息，Annex-B，每个 IDR 前带 SPS + PPS，末尾 8 字节小端 unix 毫秒采集时间。编码器：PyAV 调 libx264（`ultrafast`、`zerolatency`、无 B 帧、GOP = 帧率、CRF 23）。新客户端从强制 IDR 开始；落后 60 条消息的客户端丢弃积压，从下一个 IDR 重新开始。只在有客户端连接时编码。

### 感知（`http://<host>:8090`）

rebot-vision 感知接口版本 2（[detection_api.md](detection_api.md)）：`GET/POST /api/mode`、`GET/POST /api/inference`、`POST /api/trigger`、`GET /api/results/latest?after&wait_ms`、`POST /api/subscribe`、`POST /api/subscribe/{id}/keepalive`、`DELETE /api/subscribe/{id}`、`GET /api/subscriptions`。每个渲染帧都计算真值检测：对腕部相机做一次 640x360 分割渲染，每个场景物体取其可见像素（至少 20 个）的外接框，因此计入遮挡和画面边缘。`class_name` 为物体类别（`cube`、`cup`、`block`），`name` 为物体名，`confidence` 0.99，`format` 为 `xywh_center`，按 640x360 归一化。单次触发返回请求之后复制状态的第一帧。`aruco` 流水线会运行，但场景中没有标记。结果按订阅帧率（最高 min(15, `--fps`)）发布到 `latest`。

### 场景与评估路由

- `--objects`：`cube`（4 cm、30 g，即原场景方块）、`cup`（圆柱 ⌀ 5 cm × 7 cm、50 g）、`block`（6 × 3 × 4 cm、40 g）中 1..3 个；同类重复时命名为 `cube_2` 等。不带 `--seed` 时第一个物体位于 (0.30, 0) m，其余位于 (0.24, ±0.12) m。带种子时位置在 x 0.22..0.36 m、y −0.14..0.14 m 内抽取，偏航 ±90°，中心间距至少 9 cm（`numpy.random.default_rng(seed)`；同一种子得到同一布局）。
- `POST /sim/reset {"seed": N|null}`：在下一个物理步按种子（非负整数，否则 400）摆放物体（静止），不移动机械臂。返回状态。物理循环 3 秒内未应答时 `/sim` 路由返回 504。
- `GET /sim/state`：时间、种子、关节位置、TCP（end_link 原点），以及每个物体的 `pos`、`quat`、`lifted_m`（高出静止高度的距离）、`contacts`（`jaws`：左 / 右夹指接触；`table`）和 `grasped`（两侧夹指接触且抬起超过 2 cm）。`camera`：最新帧的 `frame_id`、`physics_time`、`physics_step`，落后物理的 `lag_s`，无新物理状态而跳过的相机周期数 `skipped`，`stale`（落后超过两个相机周期），`seg_failures`（分割渲染失败次数）和 `seg_last_error`。
- `GET /sim/camera`：当前安装参数、视场角、640x360 下的 fx、`grip_pixel_norm_at`（工具竖直朝下时夹爪尖正下方桌面点在画面中的位置，按若干接近高度给出，即 armd `grasp.grip_pixel_norm` 应取的值）、渲染后端和每帧耗时。

### 仿真中的点框抓取

`tests/test_camera_grasp.py` 启动 `rebot-sim --camera` 和 `armd --follower sim`（`ARMD_DETECTOR_URL` 指向仿真器），回 home，工具朝下移到方块上方 0.20 m，经 armd 触发一次检测（`/api/v1/detection/trigger`），再以方块的框发送 `/api/v1/action/grasp`。armd 使用套件 profile 的副本，其中 `camera_mount.offset_m` 设为仿真器的镜头中心；请求用 `overrides` 覆盖 `grip_pixel_norm`（`/sim/camera` 中 0.15 m 接近高度的值）和 `fx_px` / `fy_px`（824）。之后抓取宏（`src/grasp.rs`）与真机流程相同：接近、在新触发的帧上对中、J6 对齐、下降、闭合、抬起。通过条件：动作 `done` 且 `holding: true`，`/sim/state` 中方块 `grasped`，抬起超过 5 cm，与 TCP 水平距离小于 3 cm。

结果（spark-6124，2026-11-11，`pytest -s tests/test_camera_grasp.py`，135 s 内 2 项通过）：

| 场景 | 观察位姿处的检测（640x360 归一化） | 对中修正次数 | 最终误差 | 抓取耗时 | 抬起后的方块 |
|---|---|---|---|---|---|
| 默认方块 (0.30, 0) | x 0.154, y 0.504, w 0.258, h 0.392 | 0 | 5.2 px | 29.4 s | `grasped`，抬起 0.105 m，两侧夹指接触 |
| `--objects cube,cup --seed 3`，方块 (0.232, −0.074)，偏航 64° | x 0.159, y 0.504, w 0.319, h 0.547（被画面边缘截断） | 2 | 9.3 px | 37.3 s | `grasped`，抬起 0.105 m，杯子未被碰到 |

默认场景中 armd 的相机模型与仿真器完全一致（安装、镜头中心、fx 相同），接近后即已对中。armd 本身不计算抓取像素和镜头中心；真机上它们来自抓取设置和 profile。

v8 支架（倾斜 −38°）：工具竖直朝下时，夹爪尖正下方桌面点在画面中的归一化 v 值为：距指尖 0 m 时 0.66，0.05 m 时 0.10，约 0.06 m 以外超出画面上沿（v < 0）（0.15 m 处 `grip_pixel_norm` 的 v = −0.40）。抓取宏的对中要求该点在接近高度处位于画面内（`grip_pixel_norm` ∈ [0, 1]，`approach_height_m` ≥ `grasp_height_m` + 0.02），因此 v8 安装不适用于当前的俯视抓取宏。此结论由安装几何计算得出，未在真机上验证。

## 摩擦辨识（`rebot-sysid`）

`rebot-sysid`（`sim/mujoco/src/rebot_sim/sysid.py`，`sim/mujoco` uv 项目的命令行脚本）通过 armd 辨识 J1..J6 各关节的库仑摩擦与粘滞摩擦，并给出碰撞阈值建议。它在 PC 上运行（numpy / matplotlib），以普通 WebSocket 客户端（`websockets`）连接 armd，reCamera 上无需安装任何东西。同一命令可用于 `armd --follower sim` 和真机。

流程：

1. 获取控制租约；不带 `--i-am-at-the-arm` 时拒绝启动；运动前要求输入 `yes`（`--yes` 跳过）。未上使能时拒绝运行，除非带 `--enable`。
2. 可选：先 jog 到 `--pose`（J1..J6，rad）；否则以当前姿态为中心。该 jog、回起始姿态和回 home 的 jog 时长 = J1..J6 最大变化量 / `--approach-speed`（默认 0.1 rad/s），至少 3 s。每个关节向量在发送前按 schema 的 `dof` 检查长度。
3. 对 `--joints` 中的每个关节（其余关节保持不动）：以 50 Hz servo 流做梯形往复扫动，幅度 ±`--amplitude`（默认 0.15 rad，上限 0.35），速度依次取 `--speeds`（默认 0.1、0.2、0.3 rad/s，上限 0.6；至少两档，区分库仑与粘滞项需要），每档 `--cycles`（默认 2）个来回，换向加速度 `--accel`（默认 1.0 rad/s²）。armd 对每一帧检查关节限位和工作空间。
4. 从观测（`obs_hz=50`）记录 q、dq、力矩和 `residual_torques`（力矩 − armd 重力模型）。以下情况发送 `motion/stop` 并中止：某关节残差相对开始时变化超过 `--abort-nm`（默认 4、4、4、2.5、2.5、2.5 N·m；真机上 J4..J6 取 1.0 时被 J4 的正常摩擦波动触发）、任何故障 / 急停 / 租约丢失、Ctrl-C、servo 被拒。中止或出错后，已完成扫动的关节照常拟合并写出结果（`report.json` / `samples.json` 的 `aborted` 字段为原因，`joints_finished` 为已完成的关节）；之后若机械臂已上使能且 armd 处于 Armed 或 Moving，以慢速 jog 回到起始姿态（错误发生在前往起始姿态的 jog 中时，回到该 jog 之前的姿态），带 `--unload` 时与正常结束一样回 home 并下使能。其他状态下不移动机械臂。随后工具以该错误退出。
5. 回到起始姿态，在匀速样本（速度与扫动速度相差 10 % 以内，位置避开换向加减速段：|q − q_start| ≤ amplitude − v²/(2·accel) − 0.15 s · v − 0.01 rad，使加速度力矩不进入拟合）上拟合。速度由 ±2 个样本（80 ms）的位置差分得到，不用上报的速度（见采样相位一节）：
   `r = Fc · sign(v) + Fv · v + offset + kq · (q − q_start)`；`kq` 吸收重力模型在扫动范围内随位置变化的误差。另按方向分别拟合 `r = a + b · v`（非对称摩擦），并做仅库仑项的拟合 `r = Fc · sign(v) + offset + kq · (q − q_start)`（`coulomb_only`）。Fv < 0 的拟合（真机上出现过）标记为 `viscous_reliable: false` 并附 `viscous_note`。
6. 每个关节的碰撞阈值建议 = 所有记录样本（全部扫动，含换向段以及该关节静止、其他关节运动的时段）上 |r − 仅库仑项拟合值(v, q)| 的最大值（库仑摩擦加残差离散度，不含粘滞项；`collision.basis: coulomb_only`） × `--safety-factor`（默认 1.5），且不低于 `--threshold-floor`（0.1 N·m）。

输出目录 `--out`（默认 `sysid-out/`）：`report.json`（每个关节的 `coulomb_nm`、`viscous_nm_s_rad`、`offset_nm`、`position_slope_nm_rad`、`rmse_nm`、`positive` / `negative` 分方向拟合、`collision.max_abs_deviation_nm`、`collision.p99_abs_deviation_nm`、`collision.suggested_threshold_nm`）、`samples.json`（原始观测和各扫动时间窗）、`friction_J<n>.png`（残差-速度散点及拟合线）。没有重力模型时 armd 不提供 `residual_torques`，工具改为拟合原始力矩并给出警告。

### 在真机上运行

```bash
cd sim/mujoco
uv run rebot-sysid --url http://<recamera-ip>:8765 --i-am-at-the-arm \
  --pose 0,-1.0,-1.2,0.6,0,0 --joints 1,2,3,4,5,6 --out sysid-$(date +%Y%m%d)
```

- 整个过程（默认参数下每个关节约 30 s 运动）必须有人守在机械臂旁，急停在手边。Ctrl-C 停止运动，armd 保持当前姿态。
- `--pose` 要让每个关节距离限位至少 `--amplitude`，工具远离桌面和扫动范围内的物体；J1 会让整条臂摆动 ±amplitude。先用 `--joints 6` 和默认参数跑一次，看过曲线再扩大范围。
- 先关闭 cockpit 等其他客户端：工具需要租约。结束时回到起始姿态并释放租约；之后 armd 保持姿态，并像其他已上使能但无租约持有者的情况一样锁存 `WatchdogTimeout` 故障（在 cockpit 中清除）。带 `--unload` 时改为停靠并下使能：工具先以 `--approach-speed` 慢速 jog 到 profile 的 home 姿态，只有此后机械臂处于 home 的 `resting_pose.position_tolerances` 范围内才调用 unload，使 armd 以其自身速度执行的停靠只是小幅移动。home 姿态和容差取自 `GET /api/v1/observation/schema` 的 `home_positions` 与 `resting_pose`；armd 未公开时，`--unload` 在任何运动之前即被拒绝。机械臂最终不在容差内时（包括出错后的恢复流程），工具不执行 unload：机械臂保持使能并保持姿态，工具打印处理说明并以非零状态退出。回 home 的 jog 不移动夹爪。
- 速度：真机上 J1..J3 不超过 0.08 rad/s（例如 `--joints 1,2,3 --speeds 0.04,0.08`，再用默认参数跑 `--joints 4,5,6`）。
- 电机温度升高后摩擦会变小：`samples.json` 中有电机温度；若阈值用于长时间运行，应在机械臂运行一段时间后重复辨识。
- 可用报告中的数值更新仿真器：`rebot-sim --damping <J1..J6 粘滞项> --frictionloss <J1..J6 库仑项>`。

### 拟合量的含义与采样相位效应

armd 以 50 Hz 运行：每个周期先读电机反馈，再发送下一个位置目标，因此力矩读数总在指令周期的同一相位采样。关节匀速运动时，目标每周期前进 v / 50 rad，电机位置环对每一步都产生力矩瞬态。在 `rebot-sim` 中（瞬时力矩，假设 KPOS 25 / KV 8），J1 以 0.1 rad/s 运动时力矩在每个 20 ms 周期内于 0.28 与 0.65 N·m 之间变化，均值 0.450 N·m（= 库仑 0.3 + 粘滞 1.5 × 0.1）；armd 每次都采到周期末尾的 0.28–0.31。该波动随 v 增大，所以采样力矩低估粘滞项：`--torque-report instant` 时 J1 的拟合结果为 Fc 0.299 N·m（设定 0.3），Fv −0.18 N·m·s/rad（设定 1.5）。上报的速度有同样的相位偏差（J6 在 0.3 rad/s 时平均读数为 0.256 rad/s）；而在固定相位采样的位置做差分仍得到真实平均速度，所以拟合使用位置差分。`--torque-report mean`（最近一个指令周期的滑动平均）时力矩波动被平均掉。

平均力矩相对与之配对的位置滞后约半个指令周期。对承受重力的关节，两者之间重力项变化 G′(q) · v · 滞后时间，按 G′(q) 的符号叠加到拟合的粘滞系数上。若报告上一个完整指令区间的平均值（滞后约 1.5 个周期），库仑项仍准确，但 Fv 误差为 +16 %（J2）和 −21 %（J4）；滑动窗口把滞后缩短到约半个周期。

真机上达妙对每条指令的应答携带该时刻的力矩，因此可能存在同样的相位效应；其大小取决于电机内部环路，尚未测量。此时拟合出的 Fc / Fv 描述的是 armd 读到的力矩，也就是 armd 基于残差的检查所看到的量，可能与物理摩擦不同。某关节 Fv 接近零或为负，说明存在这一效应。需在硬件上核实。

### 仿真中的回收精度

`tests/test_sysid.py`（标记 `slow`）以 `rebot-sim --torque-report mean` 启动仿真器，阻尼设为 1.5、1.2、1.5、0.3、0.25、0.3 N·m·s/rad，摩擦设为 0.3、0.4、0.3、0.1、0.12、0.1 N·m，用默认参数对 `armd --follower sim` 运行 `rebot-sysid --pose 0,-1.6,-1.2,0.9,0,0`，要求每个拟合的 Fc、Fv 与设定值相差不超过 15 %。

结果（spark-6124，2026-11-11，3 档速度 × 2 个来回，±0.15 rad，测试用时 230 s；阈值按仅库仑项的拟合计算）：

| 关节 | Fc 拟合 / 设定（N·m） | 误差 | Fv 拟合 / 设定（N·m·s/rad） | 误差 | 建议阈值（N·m） |
|---|---|---|---|---|---|
| J1 | 0.303 / 0.3 | +1.0 % | 1.479 / 1.5 | −1.4 % | 0.742 |
| J2 | 0.401 / 0.4 | +0.2 % | 1.269 / 1.2 | +5.7 % | 0.911 |
| J3 | 0.302 / 0.3 | +0.5 % | 1.458 / 1.5 | −2.8 % | 0.765 |
| J4 | 0.100 / 0.1 | +0.3 % | 0.276 / 0.3 | −8.0 % | 0.112 |
| J5 | 0.120 / 0.12 | +0.2 % | 0.240 / 0.25 | −4.2 % | 0.119 |
| J6 | 0.100 / 0.1 | 0.0 % | 0.300 / 0.3 | −0.2 % | 0.167 |

阈值包含换向时的加速度力矩（J1 以 1 rad/s² 带动伸展的整条臂），这属于正常运动的一部分；降低 `--accel` 可降低阈值。

## 本机实测

主机 spark-6124（aarch64，Cortex-X925，20 核），MuJoCo 4.0.0，Python 3.13：

| 项目 | 数值 |
|---|---|
| 不限速物理（`rebot-sim --benchmark 20000`） | 36,960 与 37,045 步/秒（2 次） |
| 实时节拍仿真器（`--stats 2`） | 500.0–500.5 Hz |
| armd 与 MuJoCo 正运动学一致性（`test_sim_fk`，7 组关节角） | 最大 0.0001 mm、0.0000° |
| 工具朝下 `move_to` [0.30, 0, 0.12]，稳定后 TCP 误差 | 0.02 mm（此前假设 armature 0.05 / 0.01、J7 上限 1.5 N·m 时为 0.04 mm） |
| 该姿态保持力矩（J1..J7，N·m） | 0.01, 7.06, −5.89, −0.23, 0.00, 0.00, 0.00（此前：0.01, 7.05, −5.90, −0.23, 0.01, 0.00, 0.00） |
| 以 2.0 rad/s 张开夹爪（应答到完成） | 3.56 s（此前 3.52 s） |
| 闭合夹取方块（`grasp: true`，应答到完成） | 3.66 s，`holding: true`，间隙 2.66 rad，τ7 1.00 N·m（此前 4.02 s、间隙 2.64 rad、τ7 1.01 N·m） |
| 杀掉仿真器到 `BUS_LOSS` 的时间 | 0.51 s |
| 相机进程，无视频客户端（状态复制、forward、640x360 分割渲染、检测；`--stats`） | 每帧 3.2 ms；物理 500.0 Hz |
| 相机 + 视频客户端，1280x720 @ 15 / 30 fps | 每帧 7.6 / 6.5 ms；收到 15.2 / 30.2 条消息/s，解码 90/91、180/181（rebot-companion `_PyAvDecoder`）；采集 → 接收中位数 8 / 9 ms；0.22 / 0.20 Mbit/s；物理 500.0 Hz |
| 相机 + 视频客户端，640x360 @ 30 fps | 每帧 4.5 ms；30.2 条消息/s；中位数 7 ms；0.09 Mbit/s |
| 渲染后端 | GB10 上的 EGL（`GL_RENDERER` "NVIDIA GB10/PCIe"，OpenGL 4.6.0 NVIDIA 580.159.03）；默认自由视角 1280x720 渲染 + 读回 1463 / 1622 帧/s（2 次）。OSMesa：本机未安装（找不到 libOSMesa；`MUJOCO_GL=osmesa` 在 PyOpenGL 中报错），未测 |
| 高负载主机（负载均值约 100，其他任务），相机在独立进程 | 物理 499–500 Hz。测量时分割步骤每帧仍有 68 ms 的 Python 计算（之后已修复）：相机作为仿真器进程内线程、负载 100–300 时，物理为 220–275 Hz。修复后未在高负载下复测 |

## 未仿真 / 限制

- 电机动力学为简化模型：无电流环、无转矩-转速曲线、无齿槽转矩、无 CAN 延迟或丢帧模型；POS_VEL 积分环用偏置力矩前馈代替。力矩上限、空载转速和转子惯量取自数据手册；环路增益、关节阻尼（0.8 N·m·s/rad）和摩擦（0.05 N·m）在机械臂上辨识（`rebot-sysid`）之前为假设值。
- 接触参数与摩擦系数（夹指 1.5、方块 1.2）为估计值；碰撞体为方盒而非网格；连杆之间不碰撞。
- J7 力矩量级：夹持与堵转阈值（`gripper_policy.holding_torque_nm`、0.444 N·m 堵转锁存）由假设的速度环增益满足（KV 0.5 × vlim 2.0 = 1.0 N·m 夹紧力矩，低于 7 N·m 峰值），并非实测夹爪模型。
- 相机：无畸变针孔模型，无曝光 / 噪声 / 运动模糊，MuJoCo 平光照，机械臂为方盒几何（只有设置 `REBOT_DEVARM_DIR` 时连杆和夹指才用上游网格）。镜头中心和视场角为推导值，非实测。检测为真值（框精确、置信度 0.99、无漏检），不检验基于真实图像训练的检测器。未模拟真实相机的 VI 镜像设置。
- 主机负载高时相机帧率低于 `--fps`（实测见下）；物理循环不受相机影响。
- 不模拟电机故障（过温、欠压），温度恒定。不模拟 Damiao 寄存器访问（映射范围检查、CTRL_MODE 写入）：`driver_range_check` 为 null，MIT 切换在本地确认。
- 默认 `speed` 1.0 rad/s 时，夹爪从闭合到张开在约 5.9 s 后到达 −5.699 rad，来不及在 6 s 默认 `timeout_s` 内满足 200 ms 静止，夹爪动作以 `GRIPPER_TIMEOUT` 结束；测试中以 2.0 rad/s 张开。真实夹爪是否相同未核实。
- 仿真器只在主机上运行，未打包到 reCamera。
