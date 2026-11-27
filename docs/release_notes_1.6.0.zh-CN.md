# rebot-armd 1.6.0 发布说明

[English](release_notes_1.6.0.md)

`rebot-armd`，用于 Seeed reCamera（SG2002）与 reBot B601-DM 机械臂。接续 1.4.0（约 1,470 个提交）。

1.6.0 是 `rel/1.6.0-rc7` 这条线（HEAD `dcc2173a`）。下文真机结果测自 rc7（`aebc6ea3`）和 rc7.2（rc7 加上之后的五个修复：`fix/init-stop-companions`、`fix/cockpit-stopped-quiet`、`fix/workspace-messages`、`fix/stamp-namespaces`、`fix/workspace-recovery`）。rc7.3 只增加驾驶舱“抓取方式”一行；最终 1.6.0 构建与 rc7.3 的唯一区别是默认值为 top-down。

## 发布文件

| 文件 | 说明 |
|---|---|
| `rebot-teleop_1.6.0_riscv64.deb` | reCamera 安装包：`armd`（机械臂控制，网页驾驶舱在 `:8765`）和 `rebot-vision`（H.264 视频，按需运行 YOLO、分割、ArUco 和深度），以及调优后的视觉模型（见下）。用 `opkg install` 或 reCamera 控制台安装。 |
| `armd-1.6.0-riscv64-musl` | 单独的 `armd` 二进制（静态链接，riscv64 musl）。 |
| `rebot-vision-1.6.0-riscv64` | 单独的视觉二进制。配合 `REBOT_VISION_BIN=<path> scripts/build_deb.sh` 可在没有 reCamera SDK 的机器上打包。 |
| `SHA256SUMS` | 以上三个文件的校验和。 |

相同文件也在 CDN：https://sensecraft-statics.seeed.cc/solution-app/rebot/releases/1.6.0/

OBB 模型（`yolo11n_obb_cv181x_int8_fuse_qout.cvimodel`）不在包内：用 `scripts/fetch_models.sh` 下载（清单和许可证见 `kits/recamera-b601dm/vision/models/models.tsv`），然后在机械臂空闲时重启 rebot-teleop。

## 新功能

### 实时控制回路

- 运动规划在 supervisor 锁之外运行。主机测试中，点动、`run_sequence` 等旧版 WebSocket 处理函数持锁时间最长 0.164 ms（48 个处理用例；`run_sequence` 之前持锁 2.6 ms）。
- 终态观测的捕获改为边沿触发（之前是电平触发，停留期间每个 tick 都重复）。上电和下电不再在使能轮询期间持有 supervisor 锁：主机上，同步耗时 35 ms 的旧版上电，持锁最长 339 µs，下电 409 µs。
- SG2002 优先级布局，由 init 脚本设置：`armd-control` SCHED_FIFO 79，`spi0` 及其中断线程 FIFO 80，`rebot-vision` 的 SDK 线程和 4 个未固定的 `cvitask` 线程为 70。厂商内核把非 `cvitask` 线程的优先级限制在 80 以内（`MAX_OPT_RT_PRIORITY`），并把固定的 `isp_pre`、`vpss`、`tpu`、`vc` 线程设为 85–99。在备用 reCamera（mock follower）上实测：唤醒延迟最大 5.6 ms（布局前 14.4 ms）。
- 两种 owner 上都有有界 CAN 发送预算，延后的帧跨周期顺延。
- 本版本的真机测量见“测试结果”。

### RT executor owner（默认）与 legacy 回退

- 默认由 executor 线程持有机械臂。启动 owner 的优先级：`armd --owner <auto|legacy|executor>`，然后 `ARMD_RT_OWNER`，然后配置 `arm.armd_owner`，最后 `auto`。
- 在 `auto` 下，已认证的 `rebot-dm` 或 `mock` follower 启动为 executor（`effective_owner = "executor"`，`owner_reason = "auto_qualified"`）。带 MIT 关节跟踪的配置（`joint_tracking` 中 `strategy = "mit"`）和未认证的 follower（`so-arm101`、`so-arm100`、`rebot-rs`、`fashionstar`、`star-arm102`、`sim`）启动为 `legacy`，启动日志有一行说明原因（`auto_fallback_mit_profile`、`auto_fallback_unqualified_follower`）。在这类配置上指定 `--owner executor` 会在打开硬件前被拒绝。
- 在 `auto` 下 executor 无法启动时，armd 回退到 legacy owner（真机上未测试强制启动失败，项目 D2）。回退和 owner 失败状态会显示在驾驶舱（见“驾驶舱”）。
- `effective_owner` 和 `owner_reason` 在 `GET /api/v1/observation`、`GET /api/v1/diagnostics` 和 `GET /api/v1/diagnostics/tick` 中。一个不受门控的 owner 状态接口列出只有 legacy owner 提供的路由；测试设备上只列出 `/api/v1/action/gripper_calibrate`。
- 使用 legacy owner：在 `/userdata/local/apps/rebot-teleop.config.json` 中把 `arm.armd_owner` 设为 `legacy`（或用 `--owner legacy` 启动 armd），然后重启 armd。依赖 legacy 专有行为的脚本和测试必须固定该设置。
- 本版本在 executor 上可用的功能：录制与数据集（含序列录制）、夹爪零位检查、重力补偿、视觉和相机路由、深度测量、标定会话以及 sandbox 路由。

### 重力补偿、拖动示教与 MIT 模式

- 重力补偿（拖动示教 / 手动引导）在两种 owner 上都可用。达妙 MIT 模式切换属于 executor 和 legacy 共用的模式切换层，切换期间发送看门狗连续性保持帧。
- MIT 模式下达妙通信超时后，armd 用两轮下电确认恢复电机，期间保持各关节。该恢复确认尚未在真机上验证（见“已知限制”）。
- MIT 关节跟踪（`joint_tracking`，`strategy = "mit"`）默认关闭；真机上未启用。
- 写 TIMEOUT 寄存器之前，armd 在未上电状态下收集休息位证据（K = 3 个采样）。测试机械臂上的达妙 TIMEOUT 读回：J1–J6 均为 2000 counts。

### 规划限位

- 配置可以带每个关节的仅规划限位（`planning_min`、`planning_max`；默认等于硬限位）。B601-DM 配置设 J2 最大 0.03 rad、J3 最大 0.04 rad，因为从折叠 home 位向正方向会碰到机械限位或机械臂本体。
- 限位用于由位姿计算关节的场合：笛卡尔 `move_to` 和 `move_relative`（直线和 `path="joint"`）、抓取和放置子运动、回差同侧趋近、自适应抓取以及 `measure_multiview` 的视角位姿，两种 owner 上相同。被拒绝的请求返回 `IK_UNREACHABLE`，并指明关节和限位。
- 不适用于点动、`move_joints`、`move_to_pose`、序列、servo/遥操作/策略流、回放、home/park、重力补偿以及实测位置和故障检查；这些仍使用硬限位。
- 限位列在 `GET /api/v1/safety/describe`（`limits[].planning_max_position`；测试设备上 J2 为 0.03、J3 为 0.04）和观测 schema 中。
- 这替代了 1.6.0-rc4 的已知限制：`move_to` (0.26, -0.08, 0.17) 规划出 J3 = +0.078 rad，触发 `TORQUE_LIMIT`（17.9 N·m）。

### 电源保护

- 电源电流估算和调速器（由电机反馈估算；机械臂没有电流传感器，数值是按数据手册电机参数得到的估算）：`observation.power`、`GET /api/v1/diagnostics/power`，驾驶舱有电源负载表。
- 静态过载告警（默认开启）：机械臂静止（关节速度低于 0.02 rad/s）且估算电源电流在额定电流的 hard 比例以上持续 200 ms 时，`observation.power.severity` 报告 warning；超过 `supply_trip_a` 持续 100 ms 报告 trip；低于 soft 比例持续 300 ms 后解除。warning 或 trip 生效期间（`static_restricted`），降低负载的运动允许，增加负载的运动被拒绝。驾驶舱显示横幅。
- 检测所有电机同步丢失反馈，并在使能前核验达妙 `CTRL_MODE` 和 `TIMEOUT` 寄存器。
- 静态负载准入（`power_budget.static_admission_fraction`，`STATIC_LOAD_LIMIT`）可用但默认关闭，因为电流估算尚未对照实测电源标定。开启方法：在配置中设置，例如 `"static_admission_fraction": 0.85`（6.5 A 电源下为 5.53 A）。

### 工作空间脱离

- 对已经越界的工作空间外框边（x、y、z）适用 5 mm 容差，刚好在边外的位姿可以回到框内。新的侵入、地面、禁区盒、最大伸展和自碰撞不变。两种 owner 行为一致。
- 工作空间限位的拒绝信息在两种 owner 和驾驶舱中使用同一个格式化器（中英文）。信息给出带标签的数值（实测值、限位、先前和新的深度，米，3 位小数），只对具名禁区盒使用“障碍”一词；起点已在工作空间外时，建议 Park 而不是 Home。返回路径无法检查时，信息会说明并建议 Park 加一小段点动。错误码仍为 `WORKSPACE_VIOLATION`。
- 工作空间外允许拖动示教。驾驶舱显示横幅，给出越界的边、深度和返回方向。退出拖动模式（用户退出）在机械臂回到框内之前被 `GC_EXIT_OUTSIDE_WORKSPACE` 拒绝，反馈过期时也被拒绝。停止、急停、租约丢失、看门狗、故障、owner 交还和关机仍会结束拖动模式。

### 下电安全

- 只在休息位下电：J1–J6 与 `resting_pose` 偏差不超过 0.05 rad，且反馈新鲜（K = 3 个采样）。不在休息位时请求被 `409 TORQUE_OFF_WOULD_DROP` 拒绝（原因包括 `stale_feedback`、`no_resting_pose`）。没有覆盖参数。例外：热保护触发，以及进程关闭或 panic。
- 请求的下电分多个 tick 下发，下电确认前所有关节保持实测位置保持。下电不会清除故障或急停锁存。
- 使能失败不会自动回滚下电，而是锁存不可恢复的 `ENABLE_FAILED`，并对每个反馈新鲜的关节保持实测位置保持。
- 使能期间和每个保持状态下都会刷新 J7 夹爪反馈，由覆盖所有关节的轮询调度器完成；休息位判定检查 J1–J6。
- 错误码：`TORQUE_OFF_WOULD_DROP`、`CAN_TX_STALLED`（CAN 帧延后超出有界发送预算）、`J7_FEEDBACK_STALE`（可恢复；上电时 J7 反馈超过 250 ms）、`DEADLINE_EXPIRED`。

### 视觉

- deb 内置调优后的模型（deb 从 11.1 MB 增至 23.1 MB）：`yolo11n_seg_384x640` fuse 分割（单独前向 32.0 ms，stock 640x640 模型为 71.2 ms；实时 `yolo.ms` p50 67.0–73.6 ms）和桌面场景 ZipDepth（留出集桌面绝对误差中位数 33.8 mm，逐帧平面拟合后 4.0 mm；真机 8/8 个姿态残差 4–19 mm）。两者同时使用 54.29 MB ION（62.9 MB 保留区剩余 8.6 MB）。
- 安装和启动时按清单检查模型文件，文件缺失或不一致时从内置副本修复。`GET /api/v1/vision/mode` 返回 `model_config`；测试设备上 `model_config_optimal` 为 true。
- 分割检测结果在 WebSocket 帧中带 `contour`（掩码外轮廓）。备用 reCamera 实测：CPU +0.04%，帧平均大小 196 B 增至 840 B。
- `POST /api/v1/vision/detector`（仅操作员，能力 `vision.detector_switch`）在 `detect` 和 `segment` 之间切换检测器。`POST /api/v1/vision/models`（仅操作员）切换模型变体和深度模型。

### 驾驶舱

- Owner 显示：状态区显示当前生效的 owner；owner 失败或回退到 legacy 时出现横幅。页面 schema 和能力在重连和 owner 恢复后重新加载。
- 当前 owner 或状态无法执行的控件在使用前就标为不可用，提示中给出原因和下一步。例如：完整夹爪行程标定在 executor 上禁用，文字说明把 `arm.armd_owner` 设为 `legacy` 并重启 armd；拖动示教按钮在使能前显示“先使能机械臂”。
- 简易模式：识别面板、检测/分割切换、带抓取和测量操作的物体卡片、抓取任务。驾驶舱只在 Enable 或 Take control 时获取控制租约；没有控制权时控件显示为不可用，状态标签显示 “Torque on · not in control”。
- 抓取方式：物体卡片有“抓取方式”一行，两个选项：top-down（默认）和 adaptive（允许倾斜趋近）。选择保存在浏览器中，并作为 `strategy` 随抓取请求发送。
- 专家模式：桌面和平板一种布局，手机竖屏单行布局。顶栏中的“简易 | 专家”开关切换两种模式。
- 手机布局：360 px 宽度下顶栏完整显示，不遮挡语言按钮；设置抽屉高度有界，不与悬浮控件重叠，急停按钮保持可点。
- 提示信息：控制权被其他客户端接管时显示“控制权已被其他设备接管，当前页面已切换为观察模式”并带“接管控制权”按钮；连接中断时显示控制权已释放，并要求检查现场后重新获取；park 后下电状态会说明力矩为何关闭；工作空间边缘拒绝给出部位、距离和方向。均有中英文。 点动松开后，驾驶舱不再显示“Stopped before motion began”（因停止而被拒绝的 servo 帧；其他 servo 拒绝仍会显示）。
- 驾驶舱提供 `/favicon.ico`。

### 感知、录制与夹爪

- Executor owner：视觉订阅与模式、相机快照、检测器切换、深度测量、标定会话、设备录制与数据集、序列录制以及 sandbox 路由在 executor 上可用。这些在 1.6.0-rc4 中返回 `503 EXECUTOR_ROUTING_PENDING` 或仅 legacy 可用。
- executor 上的录制在 J1 运动和视觉开启时测试过（见“测试结果”）。
- 夹爪零位检查在 executor 上可用。
- 张开方向堵转：只有力矩在张开方向时，才把张开中的夹爪判为堵转；在两种 owner 上，闭合堵转后的张开和移动有回归测试。
- 夹爪完整行程标定（`gripper_calibrate`）仍仅 legacy 可用（见“已知限制”）。
- `--cell` / 配置 `cell`：解析并校验 `rebot-cell/v1` 拓扑；`capabilities.cell` 和 `GET /api/v1/cell/status` 报告该拓扑。

### 策略与抓取

- 策略动作分块（`policy.json` 中的 `chunking`：`chunk_size` 1..128、`replan_every`、`temporal_ensemble_k`）在 legacy 和 executor 两种 owner 上都支持（`docs/policy_runtime.md`）。
- 自适应抓取（阶段 B）：`/api/v1/action/grasp` 的 `strategy: "adaptive"`，含候选选择和 dry run（能力 `grasp_select`、`grasp_select_exec`；6 自由度配置）。默认仍为 `legacy`。
- Reducer（shadow）：力矩状态 reducer 与当前使用的路径并行运行并记录分歧（`torque_shadow_enabled`，默认开启）。它不驱动力矩。

## 测试结果

设备：reCamera（SG2002）上的 reBot B601-DM，`rebot-teleop 1.6.0-rc7`（`armd` md5 `922b46b8…`，2026-11-27 安装，构建机和设备上的 md5 一致）。两种 owner 都跑了同样的 7 个步骤；运行日志中 0 次机械臂故障、0 次电机 `0xD` 错误、0 行 `WATCHDOG_BUDGET_INSUFFICIENT`，reducer 无法解释的分歧为 0（executor 496 次比较，legacy 208 次）。`can0`：0 次 bus-off，0 次总线错误；接口计数器（error-warn 2、error-pass 2、1 次 RX 错误、1 次 overrun）在运行前后相同。本构建的主机测试：2,798 个测试，0 个失败。

release 时间 = 一次 50 Hz 控制 release 的耗时；跳过 = 错过截止时间的 release。Owner 列：`executor` 由 `auto` 启动（`auto_qualified`），`legacy` 用 `--owner legacy` 启动。

| 步骤 | Executor：release 最大值（ms）/ 跳过 | Legacy：release 最大值（ms）/ 跳过 |
|---|---|---|
| 1 启动、owner 状态、TIMEOUT 读回（dry run） | 5.79 / 0 | 28.70 / 1 |
| 2 空闲，未上电，61 s | 4.60 / 0 | 4.97 / 0 |
| 3 使能、保持 30 s、休息位下电，3 个循环 | 14.36 / 0 | 12.94 / 0 |
| 4 J1 运动，视觉开启，录制开启（192.7 s / 196.1 s） | 14.59 / 0 | 12.92 / 0 |
| 5 检测、测量、抓取 dry run | 4.58 / 0 | 3.59 / 0 |
| 6 safety describe、schema、规划限位 | 2.28 / 0 | 2.30 / 0 |
| 7 5 fps 视觉下的浸泡，在首次跳过时被测试脚本中止（676.9 s / 233.3 s） | 20.22 / 1（33,949 次 release） | 28.97 / 1（11,768 次 release） |

- 步骤 3 中，executor 使能耗时 0.51–0.62 s，下电 0.30 s；legacy 分别为 0.45–0.50 s 和 0.31–0.55 s。
- 录制（步骤 4）：executor 录得 180.12 s、9,007 帧的 episode，legacy 为 186.80 s、9,341 帧；均未被丢弃。
- Legacy 步骤 1 的 28.70 ms 来自 `mit_timeout` dry run；legacy 步骤 7 的 28.97 ms 来自一次持续 28.4 ms 的配置读锁（`PublishedConfig::from_inner`）。
- executor 上的 15 分钟浸泡（J1 运动循环）：视觉关闭，release 最大 14.18 ms，0 次跳过，45,483 次 release；视觉 5 fps，release 最大 14.40 ms，0 次跳过，45,699 次 release（窗口 p99 中位数 6.82 ms 对 10.32 ms）。
- executor 上 J1 运动、视觉订阅和录制开启时的 CAN 抓包（185.3 s 活动窗口，135,142 帧，内核丢包 0，CAN 错误帧 0）：任一轴控制帧之间的最大间隔为 24.79 ms，电机看门狗要求 40 ms。所有轴的保持回复延迟：p50 0.68 ms，p99 15.43 ms，最大 28.10 ms。
- 驾驶舱，executor owner：12 种组合（360x780、390x844、1024x768；中文和英文；简易和专家），12 种全部通过（无横向溢出、顶栏无被截断的元素、抽屉打开时急停可点、无控制台 404、无页面错误）；使能、park、释放操作通过。
- executor 上通过的现场项目：C3（夹爪零位检查）、C5（机械臂上电状态下的抓取 dry run：返回可达性结论 `IK_BRANCH_FLIP`）、C6（规划限位：`move_to` (0.26, -0.08, 0.17) 在规划阶段被拒绝，机械臂未动）、C8（两个姿态组成的序列运行两次，6.64 s 和 7.62 s，无故障）、D1（rebot-teleop 热重启 2 次：就绪用时 22.6 s 和 22.3 s；设备重启 1 次：48.8 s；每次之后均为 executor、Idle、未上电）、D3（第二个客户端收到 `controller_busy`；第一个客户端连接被重置后，租约释放，机械臂在 5 s 内回到 Idle 并下电；两种情况的驾驶舱文案已在中英文下检查）。C9（完整夹爪行程标定在 executor 上禁用并显示原因）通过。A7（极端反馈位置）：所有关节限位下原始位置都高于 -12.1 rad 碰撞阈值，最小余量 5.82 rad。
- D2（强制 executor 启动失败）未测试：没有受支持的设置可以在真机 follower 上强制 executor 启动失败。
- D4（在休息位带力矩时停止）在 rc7 上失败：armd 拒绝退出并锁存急停，而 init 脚本已先停掉视觉和网页服务（502）。`fix/init-stop-companions` 使 armd 拒绝停止时所有进程继续运行（有主机测试覆盖，未在机械臂上重跑）。急停锁存本身仍存在（见“已知限制”）。
**rc7.2**（rc7 加上之后的五个修复），真机 B601-DM，executor 默认，2026-11-27：

- 主机门禁：nextest 2,805 / 2,805，`ws_replay` 0 差异，alloc/lock 门禁 141 / 141。rc7.3（只增加抓取方式一行）：nextest 2,805 / 2,805。
- 真机上的 stamp 回归：2,100 个 WebSocket servo 帧、穿插 300 条普通命令和 84 次 REST 读取，随后 `home()` 两次，均被接受。
- 15 分钟 J1 ±0.15 rad 浸泡，5 fps 视觉：45,363 次 release，最大 15.935 ms，p99 11.227 ms，0 次跳过，2 次 release 超过 15 ms，0 次超过 20 ms，0 故障。
- 休息位未上电时服务停止和启动：正常。
- 驾驶舱 Playwright 12 / 12；点动松开后错误提示为 0。
- 现场，由用户完成（rc7.2 / rc7.3）：top-down 抓取成功；拖动示教（hand-guide）正常；C6 和 C3 再次通过。C6/C3 的证据（`realarm-rc7-onsite-c6c3-*`）：`move_to` (0.26, -0.08, 0.17) 被 `IK_UNREACHABLE` 拒绝，指明 `joint_3`（超出规划限位 0.0400 rad 达 0.0455 rad），最大关节变化 0.0011 rad；夹爪零位检查通过（`stop_rad` 0.0698）。

## 改进

- `K92rebot-teleop stop` 只在 armd 退出之后才停止视觉、应用和 AP 守护。armd 拒绝停止时所有进程继续运行：不丢视频，不出 502（现场测试 D4 中发现）。
- 修复 executor 上驾驶舱点动后抓取、Home 等动作被 `STAMP_REUSED` 拒绝的问题：RT 内部 servo 请求 ID 现在有独立的重放跟踪器。修复前已用测试复现。
- `rebot-vision` 自行请求关闭后会重启。BusyBox 1.33.0 的 `ash` 在子进程退出后，在阻塞的 `wait` 中空转，导致 runner 不重启（视频一直中断，直到手动重启）。runner 现在轮询 `/proc`。
- 停止和急停路径：按连接的 WebSocket 处理拆分为多个功能模块，重放语料 35 个会话（0 差异）。
- 控制线程的分配和锁证明纳入测试套件（分配门禁、锁门禁）。
- 按厂商划分的电机控制模式（配置中每个关节的 `control_mode`，`docs/motor_modes.md`）。

## 已知限制

- **Executor release 预算。** 每个 release 的硬截止和“保持帧优先”的预算排序不在本版本内（计划在 1.6.1）。负载下的保持连续性：控制帧之间最大间隔 24.8 ms，电机看门狗要求 40 ms。检查部署时，读取 `GET /api/v1/diagnostics/tick` 中的 `skipped_releases`，并在 armd 日志中查找 `WATCHDOG_BUDGET_INSUFFICIENT`；“测试结果”所列步骤中两者均为 0，注明的情况除外。
- **MIT / 拖动示教超时恢复。** MIT 模式下达妙通信超时后，恢复确认尚未在真机上验证。如果出现故障，在机械臂处于休息位时执行 park 或停止，并在驾驶舱中清除故障。
- **Executor 5 fps 视觉下的浸泡。** 有一次 release 耗时 20.22 ms（33,949 次 release 中 1 次跳过，发生在 676.9 s）。视觉关闭时，15 分钟浸泡最大 14.18 ms，0 次跳过。视觉 5 fps 时，15 分钟浸泡最大 14.40 ms，0 次跳过。
- **Legacy owner（回退）。** 配置读锁竞争在一次浸泡中造成一次 28.97 ms 的 release。对时间敏感的工作使用 executor。
- **机械臂在工作空间外。** 拖动模式因安全退出而在工作空间外结束时，机械臂停在原处。例如 `GRAVITY_COMP_JOINT_LIMIT`（距关节限位 0.05 rad）。在工作空间外，Home 和大多数运动被拒绝；按 Park 返回。通用的返回规则计划在 1.6.1。
- **带力矩时停止服务。** 机械臂带力矩时（包括在休息位）停止服务（`K92rebot-teleop stop` 或 SIGTERM）会锁存急停并保持力矩；armd 继续运行。停止服务前先 Park（下电）。计划修复：1.6.2。
- **拒绝日志。** 命令拒绝尚未写入 armd 日志（计划在 1.6.1）。
- **Adaptive 抓取方式。** 它可能拒绝 top-down 抓取（`OPENING`）能接受的宽物体，例如接近最大开口的香蕉，原因是它加了固定的位置误差余量。出现这种情况时，把“抓取方式”切换为 top-down。计划在 1.6.1 统一。
- **进入手动引导时下沉。** 进入手动引导（拖动示教）时，机械臂可能短暂下沉约 1°。2026-11-27 实测：位置到重力补偿切换期间 J2 22 mrad、J4 18 mrad；J3 力矩有一个采样从 -6.6 降到 -2.3 N·m。会自行稳定。计划在 1.6.1 修复。
- **驾驶舱页面停止发送**（例如平板锁屏或浏览器转入后台）：连接超时，armd 释放控制权，看门狗让机械臂 park。操作期间保持屏幕常亮。
- **夹爪完整行程标定**仅 legacy 可用；在 executor 上按钮禁用，驾驶舱说明原因。标定方法：把 `arm.armd_owner` 设为 `legacy`，重启 armd，标定，然后改回 `auto`。
- **静态负载准入默认关闭**，因为电源估算尚未对照实测电源标定。静态过载告警和其他电源保护保持开启。
- **单视角 `measure`** 在测试机上返回 `table_contradicts_fk`（桌面与机械臂几何不一致），需重做桌面和手眼标定。使用 `measure` 之前，在驾驶舱的标定中重做。
- **测试机上的抓取**在桌面和手眼标定不准时可能被拒绝为不可达（单视角 `measure` 报告 `table_contradicts_fk`）。先重做桌面和手眼标定。
- **部分向下运动时 USB 链路复位**（硬件问题，调查中）。在设备上（`127.0.0.1`）运行程序，不要走 USB 链路。
- Reducer 仅为 shadow：力矩决策来自当前使用的路径。
- MIT 关节跟踪未开放用于多关节。
- 双臂和多臂支持不在本版本内。
- 本版本未测设备上的内存占用。
