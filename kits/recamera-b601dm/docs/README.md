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
