# Calibration defaults

Starting values for the reCamera B601-DM kit. None of them is a measurement of a specific
camera or arm; `tests/test_arm_profile.rs` keeps them consistent with the arm profile.

| File | Contents | Status |
|---|---|---|
| [`camera_mounts.json`](camera_mounts.json) | `camera_mount` (profile field) per mount version: v5 and v7_tilt 20 / 25 / 30 deg | v5 = profile default; v7 values derived from the CAD pitch, sign not verified |
| [`grasp_setup.default.json`](grasp_setup.default.json) | tap-to-grasp setup, a copy of the profile's `grasp` section | defaults; the format armd reads from `ARMD_GRASP_SETUP` |

## Camera tilt

`camera_mount.tilt_deg` (also `camera_tilt_deg` in the grasp setup) is the camera tilt about
the camera x axis. For the default mount (`flip: false`) a negative tilt turns the optical
axis toward the jaws (image down) ([docs/agent_action_api.md](../../../docs/agent_action_api.md)).

| Mount | Mechanical pitch (CAD) | `tilt_deg` |
|---|---|---|
| v5 | 0 deg | 0 |
| v7_tilt20 | 20 deg down | -20 |
| v7_tilt25 | 25 deg down | -25 |
| v7_tilt30 | 30 deg down | -30 |

The v7 values apply the sign convention above to the CAD pitch; the sign and the value on a
mounted camera are listed as "needs verification" in docs/agent_action_api.md. `rot_deg` is
0 for all mounts: with v5 on the real arm the image was upright
(`hardware/v7_tilt/README.txt`). `offset_m` is not set (null): the CAD gives a lens position
in the MJCF `gripper_end` frame (`hardware/v7_tilt/README.txt` section 2.3), which has not
been converted to the end_link frame armd uses or checked on the arm.

To use a v7 mount, set `camera_tilt_deg` in the cockpit grasp settings (it also sets
`camera_mount.tilt_deg` and is stored in `ARMD_GRASP_SETUP`), or set `camera_mount.tilt_deg`
in a profile passed with `--config`.

## Intrinsics (placeholders)

`grasp_setup.default.json` holds `image_size_px` [640, 360], `fx_px` 600 and `fy_px` 600.
These are placeholders (`GraspSetup` in `src/grasp.rs`: "rough intrinsics used to turn pixel
errors into metric steps"); they are not a calibration of the reCamera lens. Grasp centring
uses them only to scale pixel errors into steps.

## Grasp setup

`grasp_setup.default.json` equals `ArmConfig::rebot_dm().grasp` (`GraspSetup::default()`).
Field meanings: [docs/agent_action_api.md](../../../docs/agent_action_api.md#grasp--点框抓取).
Values to check on the device (same document): `grip_pixel_norm`, `table_z_m`,
`settle_delay_s`, `lost_*`, `centre_tol_px`.

## Calibration wizard (measured values) / 标定向导（实测值）

The cockpit's calibration wizard replaces the placeholders above with measured values. It moves
the arm over a printed ArUco GridBoard on the table, solves intrinsics, hand-eye mount and table
height in the browser, and applies them through armd (WS `/api/v1/calibration/apply`; contract in
[docs/agent_action_api.md](../../../docs/agent_action_api.md#camera-calibration-wizard--相机标定向导)).
One apply writes:

| Where | What |
|---|---|
| rebot-vision `/userdata/rebot-teleop/vision.json` (via `PUT /api/calibration/intrinsics`) | `fx`, `fy`, `cx`, `cy`, `k1`, `k2`, `p1`, `p2`, `k3`, `marker_size` at 640 x 360 |
| `/userdata/rebot-teleop/calibration.json` (`ARMD_CALIBRATION`) | `camera_mount` with `q_end_cam` (R_end_cam, [w, x, y, z]) and `offset_m`, a copy of the intrinsics, `table_z_m`, `solved_at`, `rms`; optional `kinematics` (see "Kinematic fine calibration" below) |
| `/userdata/rebot-teleop/grasp_setup.json` (`ARMD_GRASP_SETUP`) | `fx_px`, `fy_px`, `cx_px`, `cy_px`, `image_size_px` [640, 360], `table_z_m`, `camera_tilt_deg` (derived from `q_end_cam`), `grip_pixel_norm` (computed by armd; left unchanged when the point under the jaws is outside the image, as with v8) |

Each apply first stores a backup in `/userdata/rebot-teleop/calibration-backups/<UTC time>/`;
WS `/api/v1/calibration/restore {"backup": "<name>"}` puts one back, `GET /api/v1/calibration`
lists them. While `calibration.json` holds `q_end_cam`, `camera_tilt_deg`, `fx_px`, `fy_px`,
`cx_px`, `cy_px` and `image_size_px` cannot be changed in the grasp settings (restore a backup or
recalibrate); `camera_mounts.json` and the profile's `flip` / `rot_deg` / `tilt_deg` are then
not used for the camera rotation. Lens distortion is used by rebot-vision's marker poses, not by
the grasp geometry.

If `calibration.json` does not parse or validate at startup, armd moves it aside
(`calibration.json.bad-<UTC time>`), uses the profile camera mount and resets the grasp setup's
camera model (`fx_px`, `fy_px`, `cx_px`, `cy_px`, `image_size_px`, `camera_tilt_deg`,
`grip_pixel_norm`) to the profile values, live and in `grasp_setup.json` (the previous file is
kept as `grasp_setup.json.pre-reset-<UTC time>`; `table_z_m` stays). Grasps then use the
uncalibrated model. `GET /api/v1/calibration` reports the cause in `load_error` until an apply or
a restore succeeds; restore the last backup or run the wizard again.

### Kinematic fine calibration / 运动学精校

The real arm's geometry differs from the URDF (measured on a B601-DM: J5 zero about −4°, J1 / J2
axes at 87.5° instead of 90°, link offsets of a few mm). The wizard's fine-calibration mode fits a
correction of the built-in chain from two board placements; `calibration.json` then holds:

```json
"kinematics": {
  "joints": [
    {"offset_rad": 0, "scale": 0, "tilt_rad": [0, 0], "origin_m": [0, 0, 0]},
    ... 6 entries, J1..J6 ...
  ],
  "fit": {"heldout_mm": [a, b], "train_mm": x, "views": n, "solved_at": "..."}
}
```

Per joint, walking the chain: the URDF joint origin, then the origin correction (tilt about x,
then y, by `tilt_rad`; translation `origin_m`), then the joint rotation by
`q * (1 + scale) + offset_rad` (q = model angle as in the schema's `model_joint_map`). J1 and J6
are written as zeros. Limits (armd refuses values outside): |`offset_rad`| ≤ 10°, |`scale`| ≤ 0.08,
|`tilt_rad`| ≤ 3° per axis, |`origin_m`| ≤ 0.03 m per component. Measured on the B601-DM (fit on
one board placement, checked on the other): 4.4 / 5.9 mm uncorrected, 2.0 / 2.2 mm with the full
correction.

- Applying or removing it (`"kinematics": null`) needs torque off and no action running, else
  `KINEMATICS_ARMED`; park the arm or turn torque off first. An apply without `kinematics` keeps
  the stored correction.
- armd uses it at once for FK, IK, workspace and keep-out checks, grasp and place, and publishes
  it as `kinematics_correction` in `/api/v1/observation/schema` and `GET /api/v1/calibration`.
  Gravity compensation and the power-budget dynamics stay on the URDF model.
- To undo: WS `/api/v1/calibration/restore` with the backup taken before the apply (it also puts
  the camera values of that time back), or an apply with `"kinematics": null`.
- A correction outside the limits or malformed in `calibration.json` at startup: the whole file
  is rejected as described above (`load_error`, uncorrected model, profile camera mount).
- A zero offset above 5° on a joint usually means the motor zero is off: re-zero the motor with
  the official tool (reBotArm_control_py `example/2_zero_and_read.py` / MotorBridge) or the
  cockpit's zero tool (below) and calibrate again.

真机几何与 URDF 存在差异（在一台 B601-DM 上实测：J5 零位约 −4°，J1 / J2 轴夹角 87.5° 而非 90°，连杆偏移数毫米）。向导的精校模式用两个标定板位置拟合内置运动链的修正，`calibration.json` 中保存为上面的 `kinematics` 结构（`joints` 为 J1..J6 共 6 项，`fit` 为拟合报告）。

沿运动链对每个关节依次为：URDF 关节原点，原点修正（先绕 x、再绕 y 旋转 `tilt_rad`，平移 `origin_m`），关节转角 `q * (1 + scale) + offset_rad`（q 为模型角，同 schema 的 `model_joint_map`）。J1 与 J6 写 0。范围（超出时 armd 拒绝）：|`offset_rad`| ≤ 10°，|`scale`| ≤ 0.08，|`tilt_rad`| 每轴 ≤ 3°，|`origin_m`| 每分量 ≤ 0.03 m。在 B601-DM 上实测（用一个标定板位置拟合、在另一个位置检验）：未修正 4.4 / 5.9 mm，完整修正 2.0 / 2.2 mm。

- 写入或删除修正（`"kinematics": null`）要求扭矩关闭且没有动作在执行，否则返回 `KINEMATICS_ARMED`；先收臂（park）或关闭扭矩。不带 `kinematics` 的 apply 保留已存的修正。
- armd 立即将其用于 FK、IK、工作空间与禁入盒检查、抓取与放置，并在 `/api/v1/observation/schema` 与 `GET /api/v1/calibration` 中以 `kinematics_correction` 发布。重力补偿与功率预算动力学仍使用 URDF 模型。
- 撤销：用 apply 之前的备份执行 WS `/api/v1/calibration/restore`（同时恢复当时的相机参数），或 apply `"kinematics": null`。
- 启动时 `calibration.json` 中的修正超出范围或格式错误：按上文处理整个文件（`load_error`、未修正模型、profile 相机安装）。
- 某个关节的零位偏移超过 5° 时，通常是电机零位有偏差：用官方工具（reBotArm_control_py `example/2_zero_and_read.py` / MotorBridge）或驾驶舱的零位标定工具（见下文）重新标定电机零位后再做标定。

### Joint zero / 关节零位

**Convention.** The B601-DM follows the official stack (Seeed-Projects/reBotArm_control_py:
URDF angle = motor angle): the zero is the folded rest pose with every joint as folded and the
gripper fully closed, so the arm reads 0 on every joint in the rest pose, and home = zero pose =
rest pose. The profile states this pose once as `zero_reference` (URDF angles); `home_positions`
and the park target follow from it, and `model_offsets_rad` is 0 on every joint. The kinematic
correction and the camera calibration are measured in these joint angles.

`calibration.json` `joint_zero` maps an arm to this convention without touching the motors:
one offset per joint (motor - joint), applied by armd at the driver boundary, so park, limits,
poses, FK / IK, the cockpit and the SDK see calibrated joint angles. Without it the offsets are 0.

**Legacy units.** A unit zeroed with the wrist rolled 90° reads J6 = ±90° in the rest pose.
armd checks this at startup and each time torque goes off and reports it as
`zero_convention: legacy_j6` (`GET /api/v1/calibration`, schema); the cockpit shows a warning
in **Settings > Tools** with a button to the zero tool. Until J6 is zeroed in the rest pose, park
rolls the wrist 90° and the 3D view / camera orientation is off by 90°. armd does not move the
arm because of the check.

**The zero tool.** **Settings > Tools > Zero** (WS `/api/v1/calibration/motor_zero`, contract in
[docs/agent_action_api.md](../../../docs/agent_action_api.md#calibration-tools-and-motor-zero--标定工具与电机零位)):

1. Torque off (the tool offers **Park and disable torque**); fold the arm into the rest pose by
   hand and hold it there, gripper fully closed: upper arm level pointing back, forearm folded
   back over it, gripper level pointing forward, jaws opening sideways, camera holder on top.
   The 3D view shows the rest pose as the ghost next to the current pose.
2. Choose the method:
   - **Software zero** (recommended): armd averages the readings over 0.5 s (the arm must stay
     still) and stores offset = reading - reference in `calibration.json` `joint_zero`. No flash
     write, works with every driver, can be undone (**Undo** in the result, or WS
     `/api/v1/calibration/joint_zero/restore {backup}`; a backup of `calibration.json` is taken
     first).
   - **Motor zero** (Damiao): armd disables every motor, waits for fresh feedback confirming them
     disabled, sends the Damiao set-zero frame per motor 0.1 s apart (the sequence of
     reBotArm_control_py `RebotArm.set_zero`) and sets the joint zero offsets of those joints to 0.
     The motors hold the new zero; it cannot be undone, only set again in the rest pose.
3. Select the joints (for a legacy unit: J6). A joint reading more than 45° from the rest pose
   needs **force** (a lost or legacy zero). After the zero each selected joint reads its reference
   (0 on the B601-DM); armd checks it within 10 mrad within 1 s. Torque stays off.
4. What becomes stale. The kinematic correction is removed (it was measured with the old zero)
   and the camera calibration is marked stale (`"stale": {"reason", "at"}`; grasp keeps working
   with it until the next camera or fine calibration). For joints whose reading moved by more
   than 0.5°: the saved poses (`poses.json`) are marked stale (the arm would reach a different
   physical pose; saving a pose again clears the mark) and, for the gripper joint, the gripper
   calibration (opening table / offset). Nothing is deleted. `GET /api/v1/calibration`
   `stale_list` lists them; the cockpit marks stale poses in the poses list. Recorded episodes
   and datasets are joint-space data and do not follow a zero change. Each run is appended to
   `calibration-backups/motor_zero.jsonl`.

Recommended order: zero → fine calibration → gripper calibration.

Measured on the arm: on 2026-11-19 the motor zero was run on all 7 joints of the office
B601-DM in the rest pose (readings before: J1..J5 about 0, J6 1.574 rad, gripper about 0; J6
needed force); with this convention its `joint_zero` is all 0. Not verified: whether the Damiao
zero survives a power cycle (`persist_verified: false` in `capabilities.calibration_tools`); the
official wiki says the B601-DM is calibrated once after assembly, motorbridge's Damiao notes say
the zero is lost at power-off for the gripper motor (multi-turn travel). The software zero is
tested against the mock and sim followers.

**约定。** B601-DM 采用官方约定（Seeed-Projects/reBotArm_control_py：URDF 角 = 电机角）：零位是折叠收纳姿态，各关节按收纳时折好，夹爪完全闭合；机械臂在收纳姿态每个关节读 0，回零姿态 = 零位姿态 = 收纳姿态。profile 用 `zero_reference`（URDF 角）给出这个姿态一次；`home_positions` 和收纳目标由它推出，`model_offsets_rad` 各关节为 0。运动学修正与相机标定都按这套关节角测得。

`calibration.json` 的 `joint_zero` 在不改电机的情况下把机械臂映射到这套约定：每个关节一个偏移（电机角 - 关节角），由 armd 在驱动边界应用，收纳、限位、点位、FK / IK、驾驶舱和 SDK 看到的都是标定后的关节角。没有这个字段时偏移为 0。

**旧零位的机械臂。** 在手腕转了 90° 的位置置零的机械臂，在收纳姿态 J6 读 ±90°。armd 在启动时和每次关闭使能后检查，在 `GET /api/v1/calibration` 与 schema 中报告为 `zero_convention: legacy_j6`；驾驶舱在 **设置 > 工具** 中显示提示，并提供打开零位标定的按钮。在收纳姿态重新设置 J6 零位之前，收纳会把手腕转 90°，3D 视图与相机方向差 90°。armd 不会因为这项检查移动机械臂。

**零位标定工具。** **设置 > 工具 > 零位标定**（WS `/api/v1/calibration/motor_zero`，接口说明见 [docs/agent_action_api.md](../../../docs/agent_action_api.md#calibration-tools-and-motor-zero--标定工具与电机零位)）：

1. 关闭使能（工具里有 **收纳并关闭使能**），用手把机械臂折叠到收纳姿态并保持住，夹爪完全闭合：大臂水平朝后、小臂折回压在大臂上、夹爪水平朝前、夹爪左右张合、相机支架在上。3D 视图用半透明模型显示收纳姿态，与当前姿态对照。
2. 选择方法：
   - **软件零位**（推荐）：armd 在 0.5 s 内取读数平均（机械臂须保持不动），把 偏移 = 读数 - 参考值 写入 `calibration.json` 的 `joint_zero`。不写电机 flash，任何驱动都可用，可以撤销（结果页的 **撤销**，或 WS `/api/v1/calibration/joint_zero/restore {backup}`；执行前先备份 `calibration.json`）。
   - **电机零位**（达妙）：armd 先失能所有电机，等到新鲜反馈确认已失能，逐个电机发送达妙置零帧，间隔 0.1 s（与 reBotArm_control_py `RebotArm.set_zero` 的顺序相同），并把这些关节的零位偏移设为 0。电机保存新零点，无法撤销，只能在收纳姿态重新设置。
3. 选择关节（旧零位的机械臂：J6）。读数偏离收纳姿态超过 45° 的关节需要勾选 **强制设置**（零位丢失或旧零位）。设置后所选关节读出参考值（B601-DM 为 0）；armd 在 1 s 内检查到 10 mrad 以内。使能保持关闭。
4. 哪些内容会过期。删除运动学修正（它按旧零位测得），相机标定标为过期（`"stale": {"reason", "at"}`；抓取继续使用，直到下一次相机标定或精校）。对读数变化超过 0.5° 的关节：已存点位（`poses.json`）标为过期（前往时实际到达的物理姿态不同；重新记录该点位即清除），夹爪关节则把夹爪标定（开口表 / 偏差）标为过期。不删除任何数据。`GET /api/v1/calibration` 的 `stale_list` 列出这些内容；驾驶舱在点位列表中标出过期点位。已录制的 episode 与数据集是关节空间数据，不随零位改动变化。每次执行追加到 `calibration-backups/motor_zero.jsonl`。

推荐顺序：零位标定 → 精校 → 夹爪标定。

实机记录：2026-11-19 在办公室的 B601-DM 上于收纳姿态对全部 7 个关节执行了电机置零（置零前读数：J1..J5 约 0，J6 1.574 rad，夹爪约 0；J6 需要强制设置）；按本约定其 `joint_zero` 全为 0。尚未验证：达妙电机零点掉电后是否保留（`capabilities.calibration_tools` 中 `persist_verified: false`）；官方 wiki 写 B601-DM 装配后只需标定一次，motorbridge 的达妙说明写夹爪电机（多圈行程）掉电后零点丢失。软件零位在 mock 与 sim 跟随器上测试。

Board (the wizard prints it; the simulator's `--aruco-board` uses the same layout): 4 columns x
3 rows (landscape, as the 16:9 image) of aruco4x4_50 markers, ids 0..11 in reading order
(id = row * 4 + col, row 0 at the top of the page, col 0 at the left), marker side 25 mm by
default, gap 5 mm (0.2 x side), board about 115 x 85 mm. Board frame: origin at the board
centre, x right on the page, y down the page, z into the table; the printed "up" arrow points
to the page top (board -y). Marker corners in rebot-vision's
order: bottom-left, bottom-right, top-right, top-left of the printed marker (marker frame x right,
y down).

Fine calibration mode (camera + arm geometry): the wizard captures two board placements (about 35
poses each, joint-space moves approached from the same side; about 5 minutes per placement on the
real arm with these motion rules) and, besides the values above, solves a correction of the arm's
geometry for J2..J5 (zero offset, angle scale, joint-axis tilt, joint-origin shift; J1 and J6
zero). Between the two captures the user moves the board to another spot (at least 10 cm away)
and captures a new start view. The correction is applied only when the held-out error (fit on one
placement, predict the other) is ≤ 3 mm both ways, the mean held-out error is ≥ 30 % below the
uncorrected model, and every parameter is inside armd's bounds (|offset| ≤ 10°, |scale| ≤ 0.08,
|tilt| ≤ 3°, |origin| ≤ 30 mm). The same apply then also writes `kinematics` (per joint
`offset_rad`, `scale`, `tilt_rad`, `origin_m`, plus the fit's held-out numbers) into
`calibration.json`; armd requires torque off for it. A J5 zero offset above 5° is reported with a
hint to re-zero that motor (J2..J4 offsets are not separately identifiable and get no hint) (reBotArm_control_py `example/2_zero_and_read.py` / MotorBridge). On data
recorded on this kit's arm (2026-11-18) the held-out error went from 2.76 / 3.64 mm to
2.01 / 2.24 mm.

Start view (the place step, and in fine calibration mode also the move step after placement A): the
camera has to see at least 6 whole board markers; "Capture start view" stays disabled until the live
marker count reaches 6 and says why (too few markers, hand-guiding active, arm moving, search
running). Three buttons bring the camera onto the board:

| Button | What it does |
|---|---|
| Jog | Hides the wizard panel and shows a small bar with the live marker count, "Capture start view" and "Back to wizard". The bar sits above the sticks, E-stop and Enable (bottom of the screen, or higher when they are in the way), so the cockpit sticks and Cartesian jog stay usable, also on a phone (390 x 844). |
| Move by hand | Enters gravity compensation with the same checks as the cockpit's hand-guide switch (no hold-to-enter). The button then reads "Exit and hold pose"; the start view can be captured only after the exit has locked the pose and the arm is still. |
| Search | Moves the camera over up to 24 candidate board centres on the table (radius 0.20-0.45 m, azimuth -70..+70°, camera 0.31 m above the table looking down), nearest to the camera's current view first; in the move step, candidates within 8 cm of placement A come last. Same move_to speed caps (0.03 m/s, 0.3 rad/s), path checks, skip rules (IK_UNREACHABLE, IK_BRANCH_FLIP, WORKSPACE_VIOLATION) and abort rules as a capture run; one ArUco trigger per candidate. At the first view with ≥ 6 board markers it re-aims once at the marker centroid and stops there. Stop aborts at once. |

The search was tested against the in-page simulation of `scripts/calib_wizard_shots.mjs` (board
found after 17 of 24 candidates, camera starting 0.36 m from the board; Stop during a search); not yet on
the real arm.

驾驶舱标定向导用实测值替换上面的占位值：机械臂在桌面上打印的 ArUco GridBoard 上方移动，在浏览器中求解内参、手眼安装与桌面高度，再通过 armd 写入（WS `/api/v1/calibration/apply`，接口见 [docs/agent_action_api.md](../../../docs/agent_action_api.md#camera-calibration-wizard--相机标定向导)）。一次写入包括：rebot-vision 的 `/userdata/rebot-teleop/vision.json`（经 `PUT /api/calibration/intrinsics`，640 x 360 下的 `fx`、`fy`、`cx`、`cy`、`k1`、`k2`、`p1`、`p2`、`k3`、`marker_size`）；`/userdata/rebot-teleop/calibration.json`（`ARMD_CALIBRATION`：含 `q_end_cam`（R_end_cam，[w, x, y, z]）与 `offset_m` 的 `camera_mount`、内参副本、`table_z_m`、`solved_at`、`rms`）；`/userdata/rebot-teleop/grasp_setup.json`（`ARMD_GRASP_SETUP`：`fx_px`、`fy_px`、`cx_px`、`cy_px`、`image_size_px` [640, 360]、`table_z_m`、由 `q_end_cam` 推导的 `camera_tilt_deg`、armd 计算的 `grip_pixel_norm`；夹爪下方的点在画面外时（如 v8）保持不变）。

每次写入前先在 `/userdata/rebot-teleop/calibration-backups/<UTC 时间>/` 保存备份；WS `/api/v1/calibration/restore {"backup": "<名称>"}` 恢复备份，`GET /api/v1/calibration` 列出备份。`calibration.json` 含 `q_end_cam` 期间，抓取设置中的 `camera_tilt_deg`、`fx_px`、`fy_px`、`cx_px`、`cy_px`、`image_size_px` 不能修改（恢复备份或重新标定）；相机旋转不再使用 `camera_mounts.json` 与 profile 的 `flip` / `rot_deg` / `tilt_deg`。镜头畸变用于 rebot-vision 的标记位姿，不用于抓取几何。

启动时 `calibration.json` 无法解析或校验不通过：armd 将其改名为 `calibration.json.bad-<UTC 时间>`，使用 profile 的相机安装，并把抓取设置的相机模型（`fx_px`、`fy_px`、`cx_px`、`cy_px`、`image_size_px`、`camera_tilt_deg`、`grip_pixel_norm`）重置为 profile 值（运行时及 `grasp_setup.json`；原文件保留为 `grasp_setup.json.pre-reset-<UTC 时间>`；`table_z_m` 不变）。之后抓取使用未标定的模型。`GET /api/v1/calibration` 的 `load_error` 给出原因，直到一次 apply 或 restore 成功；恢复最近的备份或重新运行向导。

标定板（向导生成打印页；仿真器 `--aruco-board` 使用相同布局）：4 列 x 3 行（横向，与 16:9 画面一致）aruco4x4_50 标记，id 0..11 按阅读顺序（id = 行 * 4 + 列，第 0 行在页面顶部，第 0 列在左侧），默认标记边长 25 mm，间隔 5 mm（0.2 倍边长），整板约 115 x 85 mm。标定板坐标系：原点在板中心，x 沿页面向右，y 沿页面向下，z 指向桌面内；打印的 "up" 箭头指向页面顶部（板 -y）。标记角点顺序同 rebot-vision：打印标记的左下、右下、右上、左上（标记坐标系 x 向右、y 向下）。

精校模式（相机 + 机械臂几何）：向导采集两个标定板位置（各约 35 个位姿，关节空间移动、从同一方向接近；按这套运动规则在真机上每个位置约 5 分钟），除上述数值外还求解 J2..J5 的几何修正（零位、角度比例、关节轴倾斜、关节原点偏移；J1、J6 为 0）。两次采集之间用户把标定板移到另一个位置（距原位置至少 10 cm）并采集新的起始视图。只有满足以下条件才写入修正：两个方向的留出误差（在一个位置拟合、预测另一个位置）都 ≤ 3 mm，留出误差平均值比未修正模型低 ≥ 30 %，所有参数在 armd 的范围内（|零位| ≤ 10°、|比例| ≤ 0.08、|倾斜| ≤ 3°、|原点| ≤ 30 mm）。同一次写入还把 `kinematics`（每个关节的 `offset_rad`、`scale`、`tilt_rad`、`origin_m`，以及拟合的留出误差）写进 `calibration.json`；armd 要求此时电机断电。J5 零位偏差超过 5° 时给出提示，建议重新校准该电机零位（J2..J4 的零位不能单独确定，不提示）（reBotArm_control_py `example/2_zero_and_read.py` / MotorBridge）。在本套件机械臂上录制的数据（2026-11-18）中，留出误差从 2.76 / 3.64 mm 降到 2.01 / 2.24 mm。

起始视图（摆放步骤；精校模式下位置 A 之后的移板步骤也一样）：相机需要看到至少 6 个完整的标定板标记；实时标记数达到 6 之前「采集起始视图」不可用，并显示原因（标记不足、拖动示教中、机械臂在动、正在自动寻找）。三个按钮用来让相机看到标定板：

| 按钮 | 作用 |
|---|---|
| 用摇杆移动 | 隐藏向导面板，显示一个小条：实时标记数、「采集起始视图」、「返回向导」。小条放在摇杆、急停和使能按钮上方（屏幕底部；被挡住时上移），驾驶舱摇杆与笛卡尔点动照常可用，手机（390 x 844）上也一样。 |
| 拖动示教 | 进入重力补偿，检查条件与驾驶舱的拖动示教开关相同（不需要按住进入）。按钮随后变为「退出并锁定位置」；退出、位置锁定且机械臂静止后才能采集起始视图。 |
| 自动寻找 | 让相机依次移到桌面上最多 24 个候选板中心上方（半径 0.20–0.45 m，方位 −70..+70°，相机在桌面上方 0.31 m、朝下），从离相机当前视野最近的开始；移板步骤中距位置 A 8 cm 以内的候选放在最后。速度上限（0.03 m/s、0.3 rad/s）、路径检查、跳过规则（IK_UNREACHABLE、IK_BRANCH_FLIP、WORKSPACE_VIOLATION）和中止规则与采集运行相同；每个候选触发一次 ArUco 识别。第一个看到 ≥ 6 个标记的视图处，按标记中心再对准一次后停下。停止按钮立即中止。 |

自动寻找在 `scripts/calib_wizard_shots.mjs` 的页面内仿真中测试（相机起始位置离标定板 0.36 m，第 17 / 24 个候选找到；寻找中按停止）；尚未在真机上测试。
