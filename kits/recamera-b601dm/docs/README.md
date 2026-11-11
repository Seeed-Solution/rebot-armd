# reCamera B601-DM kit: wiring, assembly, verified capabilities

English and Chinese (中文在各节英文之后). Facts are taken from the repository README and
`docs/`; this page adds no new measurements.

## Wiring (reCamera B4 CAN board)

From the B4 board documentation; this repository does not verify it. Check the silkscreen of
your board revision.

```text
              reCamera B4 CAN board
     [ RST/BOOT buttons ]             [ silkscreen CAN&POWER ]
    ┌─────────────────────────────────────────────────┐
    │              J2 (white 6-pin connector)          │
    │         ┌───┬───┬───┬───┬───────┬───────┐       │
    │         │ 1 │ 2 │ 3 │ 4 │   5   │   6   │       │
    │         └───┴───┴───┴───┴───┬───┴───┬───┘       │
    └─────────────────────────────┼───────┼───────────┘
                                CAN_H   CAN_L
```

J2 (SM06B-SRSS-TB, JST SH 1.0 mm, 6 pins):

- Pin 1 / 2: `VIN` (12-24 V DC in)
- Pin 3 / 4: `GND`
- Pin 5: `CAN_H` (arm red wire)
- Pin 6: `CAN_L` (arm black wire)

The board has an MCP2518FD SPI-CAN controller, an MCP2558FD transceiver and a 124 Ω
termination resistor. Keep motor power wiring away from pins 5/6 and connect the grounds of
the reCamera and the arm power supply (pins 3/4).
The init script sets `can0` to 1 Mbit/s; the arm runs at 1 Mbit/s.

### 接线（reCamera B4 CAN 底板）

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
init 脚本把 `can0` 设为 1 Mbit/s，与机械臂一致。

## Assembly

1. Print the camera holder: v5 (in use) or a v7_tilt variant, body and top bar separately;
   files and the list of what is not validated in [../hardware/](../hardware/README.md).
2. Mount the reCamera in the holder as described in `hardware/v5/ASSEMBLY.txt` (remove the
   top bar, lower the camera in, insert the square nuts, attach the top bar; do not drive
   screws into the camera) and clamp the holder to the gripper. The camera sits after J6
   (eye-in-hand).
3. Wire the arm's CAN pair and power to J2 of the B4 board (above).
4. Install the `rebot-teleop` deb (`./scripts/build_deb.sh`, repository README section
   "Install on reCamera") and open `http://<camera-address>:8765/`.
5. With a v7_tilt mount, set the camera tilt (see [../calibration/](../calibration/README.md)).

### 组装

1. 打印相机支架：v5（当前使用）或 v7_tilt 某一角度，主体与顶条分开打印；文件与未验证项见 [../hardware/](../hardware/README.md)。
2. 按 `hardware/v5/ASSEMBLY.txt` 把 reCamera 装进支架（取下顶条、放入相机、从两侧塞入方螺母、装回顶条；螺丝不要拧进相机），再把支架夹到夹爪上。相机装在 J6 之后（眼在手上）。
3. 机械臂的 CAN 线与电源接到 B4 底板 J2（见上）。
4. 安装 `rebot-teleop` deb（`./scripts/build_deb.sh`，见仓库 README「在 reCamera 上安装」），打开 `http://<相机地址>:8765/`。
5. 使用 v7_tilt 支架时设置相机俯角（见 [../calibration/](../calibration/README.md)）。

## Verified capabilities

| Capability | Status | Source |
|---|---|---|
| armd on the reCamera B4 CAN board driving the B601-DM (7 Damiao motors, POS_VEL, 50 Hz) | verified on hardware | README "Hardware" |
| CPU 7.0 % / RAM 3.7 MB with all motors torqued at 50 Hz | measured on hardware | README intro table |
| Damiao mapping ranges per joint (J1-J3 ±12.5 / ±10 / ±28, J4-J7 ±12.5 / ±30 / ±10) | read from the motors 2026-11-09 | `src/model.rs`, docs/control_safety_contract.md |
| Gripper range (closed +0.03 rad, mechanical full open -5.87 rad) | measured 2026-11-06 | `src/model.rs` |
| POS_VEL lag gain 1.5 (basis of the 0.45 rad/s planned-speed cap) | measured 2026-11-09; listed under "needs verification" | docs/agent_action_api.md |
| Camera image upright with the v5 mount | observed on the arm | `hardware/v7_tilt/README.txt` |
| Gravity compensation (hand-guiding) | verified on hardware: exit paths held within 1.5 mrad, 30 s recording gave 1509 frames | README, docs/control_safety_contract.md checklist |
| Tap-to-grasp | simulation and screenshot tests only | README, docs/plan_tap_to_grasp.md |
| `camera_mount` (`tilt_deg`, `offset_m`) for camera-frame moves | needs verification | docs/agent_action_api.md |
| Planning time on the SG2002 | needs verification | docs/agent_action_api.md |
| v7_tilt mounts | not printed or fitted | `hardware/v7_tilt/README.txt` |

Object detection and ArUco come from `rebot-vision`, which is part of this kit's package
(README "Install on reCamera"); no separate camera app is needed.

### 已验证能力

| 能力 | 状态 | 来源 |
|---|---|---|
| armd 在 reCamera B4 CAN 底板上驱动 B601-DM（7 个达妙电机，POS_VEL，50 Hz） | 实机验证 | README「硬件」 |
| 全部电机上力 50 Hz 时 CPU 7.0 %、内存 3.7 MB | 实机测得 | README 开头表格 |
| 各关节达妙映射范围（J1-J3 ±12.5 / ±10 / ±28，J4-J7 ±12.5 / ±30 / ±10） | 2026-11-09 从电机读取 | `src/model.rs`、docs/control_safety_contract.md |
| 夹爪范围（闭合 +0.03 rad，机械全开 -5.87 rad） | 2026-11-06 实测 | `src/model.rs` |
| POS_VEL 滞后增益 1.5（0.45 rad/s 规划限速的依据） | 2026-11-09 实测，列在「需实机核实」 | docs/agent_action_api.md |
| v5 支架下画面方向为正 | 实机观察 | `hardware/v7_tilt/README.txt` |
| 重力补偿（拖动示教） | 实机验证：各退出路径位置偏差不超过 1.5 mrad，30 s 录制 1509 帧 | README、docs/control_safety_contract.md 清单 |
| 点框抓取 | 仅仿真与截图测试 | README、docs/plan_tap_to_grasp.md |
| 相机坐标系运动用的 `camera_mount`（`tilt_deg`、`offset_m`） | 需实机核实 | docs/agent_action_api.md |
| SG2002 上的规划耗时 | 需实机核实 | docs/agent_action_api.md |
| v7_tilt 支架 | 未打印、未试装 | `hardware/v7_tilt/README.txt` |

目标检测和 ArUco 由 `rebot-vision` 提供，包含在本套件的安装包内（README「在 reCamera 上安装」），无需另装相机应用。

## Run your own program

The reCamera gallery runs one app at a time and the camera is exclusive, so a user program runs
inside the rebot-teleop package, not as a separate gallery app. `rebot-app-run` runs it, restarts
it after a crash and keeps its log. The runner is covered by host tests
(`scripts/test_app_runner.sh`); it has not been run on a device yet.

1. Write the program with the stdlib-only `rebot` SDK (rebot-companion). The deb carries the
   `rebot/` package in `/usr/share/rebot-teleop/python` (the recamera build requires
   `REBOT_SDK_DIR=<rebot-companion checkout>`). Example: `/usr/share/rebot-teleop/examples/hello/main.py`.
2. Copy the program to `/userdata/rebot-apps/<name>/` (entry `main.py`).
3. Start it: `/etc/init.d/K92rebot-teleop app-start <name>`. Other commands: `app-stop`,
   `app-status`, `app-logs [n]` (default 50 lines).
4. Start at boot: in the rebot-teleop settings, group "Your Program", set `app.enabled` to `on` and
   `app.name` to `<name>` (`/userdata/local/apps/rebot-teleop.config.json`). `start` launches the
   program after armd; `stop` stops it first.

| Item | Value |
|---|---|
| Command | `nice -n 5 python3 -u <entry>`, working directory = the app directory |
| User | `recamera` (unprivileged; `REBOT_APP_USER` overrides it), never root; a missing user refuses the start. The program can read the app directory and `/userdata/rebot-teleop` but not write the armd state files there (root-owned); a program that writes files needs a directory writable by `recamera` |
| Environment | `REBOT_HOST=127.0.0.1`, `PYTHONPATH=/usr/share/rebot-teleop/python:<app dir>` |
| `app.json` (optional) | `{"entry": "main.py", "restart": "on-failure" \| "always" \| "never"}`; default `main.py`, `on-failure` |
| Restart | exit code 0 ends the runner with `on-failure`; other exits restart after 1, 2, 4 ... 30 s (back to 1 s after a run longer than 60 s); 10 starts within 5 minutes end it with state `given_up` |
| Stop | the program runs in its own session; SIGTERM to its process group, SIGKILL to the group after 10 s; processes left in the group when the program exits are killed |
| Log | `/userdata/rebot-teleop/logs/app-<name>.log`; above 512 KiB (checked before each start and every 30 s) it is copied to `.1`, `.1` to `.2`, and truncated; bytes written during the copy can be lost |
| State | `/tmp/rebot-app-<name>.state`: `state` (running, backoff, stopped, given_up), `pid`, `runner`, `starts`, `last_exit` |

### 运行自己的程序

reCamera 应用画廊一次只运行一个应用，摄像头为独占资源，因此用户程序由 rebot-teleop 软件包托管，
不作为独立的画廊应用。`rebot-app-run` 负责运行、崩溃后重启并保留日志。运行器已有主机端测试
（`scripts/test_app_runner.sh`），尚未在设备上运行。

1. 用仅依赖标准库的 `rebot` SDK（rebot-companion）编写程序。构建 recamera deb 时必须设置
   `REBOT_SDK_DIR=<rebot-companion 检出目录>`（未设置时构建停止），`rebot/` 包会装入 `/usr/share/rebot-teleop/python`。
   示例：`/usr/share/rebot-teleop/examples/hello/main.py`。
2. 将程序复制到 `/userdata/rebot-apps/<name>/`（入口 `main.py`）。
3. 启动：`/etc/init.d/K92rebot-teleop app-start <name>`。其他命令：`app-stop`、`app-status`、
   `app-logs [n]`（默认 50 行）。
4. 开机自启：在 rebot-teleop 设置的“自定义程序”组中，将 `app.enabled` 设为 `on`、`app.name`
   设为 `<name>`（`/userdata/local/apps/rebot-teleop.config.json`）。`start` 在 armd 之后启动该程序，
   `stop` 先停止该程序。

程序以非特权用户 `recamera` 运行（可用 `REBOT_APP_USER` 指定，不允许 root）；该用户不存在时不启动并记录日志。
程序可读取应用目录与 `/userdata/rebot-teleop`，但不能写入其中由 root 拥有的 armd 状态文件。程序在独立会话中运行，
停止时 SIGTERM 发送到整个进程组，10 秒后对进程组发送 SIGKILL；程序退出后遗留在进程组中的进程会被结束。

| 项目 | 取值 |
|---|---|
| 命令 | `nice -n 5 python3 -u <entry>`，工作目录为程序目录 |
| 环境变量 | `REBOT_HOST=127.0.0.1`，`PYTHONPATH=/usr/share/rebot-teleop/python:<程序目录>` |
| `app.json`（可选） | `{"entry": "main.py", "restart": "on-failure" \| "always" \| "never"}`；默认 `main.py`、`on-failure` |
| 重启 | `on-failure` 下退出码 0 即结束；其他退出按 1、2、4 ... 30 秒退避重启（运行超过 60 秒后回到 1 秒）；5 分钟内启动 10 次则以 `given_up` 状态结束 |
| 停止 | 向程序发 SIGTERM，10 秒后 SIGKILL |
| 日志 | `/userdata/rebot-teleop/logs/app-<name>.log`；超过 512 KiB（每次启动前及每 30 秒检查）时复制为 `.1`（原 `.1` 变 `.2`）并原地清空，复制期间写入的字节可能丢失 |
| 状态 | `/tmp/rebot-app-<name>.state`：`state`（running、backoff、stopped、given_up）、`pid`、`runner`、`starts`、`last_exit` |
