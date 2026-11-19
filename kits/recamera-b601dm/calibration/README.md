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
0 for all mounts: with v5 on the real arm the image was upright (documented in upstream
`Seeed-Projects/reBot-DevArm`, `hardware/camera-mounts/recamera-mounts/`; see also
[../hardware/](../hardware/README.md)). `offset_m` is not set (null): the CAD gives a lens
position in the MJCF `gripper_end` frame (upstream `reBot-DevArm` design notes), which has not
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
| `/userdata/rebot-teleop/calibration.json` (`ARMD_CALIBRATION`) | `camera_mount` with `q_end_cam` (R_end_cam, [w, x, y, z]) and `offset_m`, a copy of the intrinsics, `table_z_m`, `solved_at`, `rms` |
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

Board (the wizard prints it; the simulator's `--aruco-board` uses the same layout): 4 columns x
3 rows (landscape, as the 16:9 image) of aruco4x4_50 markers, ids 0..11 in reading order
(id = row * 4 + col, row 0 at the top of the page, col 0 at the left), marker side 25 mm by
default, gap 5 mm (0.2 x side), board about 115 x 85 mm. Board frame: origin at the board
centre, x right on the page, y down the page, z into the table; the printed "up" arrow points
to the page top (board -y). Marker corners in rebot-vision's
order: bottom-left, bottom-right, top-right, top-left of the printed marker (marker frame x right,
y down).

驾驶舱标定向导用实测值替换上面的占位值：机械臂在桌面上打印的 ArUco GridBoard 上方移动，在浏览器中求解内参、手眼安装与桌面高度，再通过 armd 写入（WS `/api/v1/calibration/apply`，接口见 [docs/agent_action_api.md](../../../docs/agent_action_api.md#camera-calibration-wizard--相机标定向导)）。一次写入包括：rebot-vision 的 `/userdata/rebot-teleop/vision.json`（经 `PUT /api/calibration/intrinsics`，640 x 360 下的 `fx`、`fy`、`cx`、`cy`、`k1`、`k2`、`p1`、`p2`、`k3`、`marker_size`）；`/userdata/rebot-teleop/calibration.json`（`ARMD_CALIBRATION`：含 `q_end_cam`（R_end_cam，[w, x, y, z]）与 `offset_m` 的 `camera_mount`、内参副本、`table_z_m`、`solved_at`、`rms`）；`/userdata/rebot-teleop/grasp_setup.json`（`ARMD_GRASP_SETUP`：`fx_px`、`fy_px`、`cx_px`、`cy_px`、`image_size_px` [640, 360]、`table_z_m`、由 `q_end_cam` 推导的 `camera_tilt_deg`、armd 计算的 `grip_pixel_norm`；夹爪下方的点在画面外时（如 v8）保持不变）。

每次写入前先在 `/userdata/rebot-teleop/calibration-backups/<UTC 时间>/` 保存备份；WS `/api/v1/calibration/restore {"backup": "<名称>"}` 恢复备份，`GET /api/v1/calibration` 列出备份。`calibration.json` 含 `q_end_cam` 期间，抓取设置中的 `camera_tilt_deg`、`fx_px`、`fy_px`、`cx_px`、`cy_px`、`image_size_px` 不能修改（恢复备份或重新标定）；相机旋转不再使用 `camera_mounts.json` 与 profile 的 `flip` / `rot_deg` / `tilt_deg`。镜头畸变用于 rebot-vision 的标记位姿，不用于抓取几何。

启动时 `calibration.json` 无法解析或校验不通过：armd 将其改名为 `calibration.json.bad-<UTC 时间>`，使用 profile 的相机安装，并把抓取设置的相机模型（`fx_px`、`fy_px`、`cx_px`、`cy_px`、`image_size_px`、`camera_tilt_deg`、`grip_pixel_norm`）重置为 profile 值（运行时及 `grasp_setup.json`；原文件保留为 `grasp_setup.json.pre-reset-<UTC 时间>`；`table_z_m` 不变）。之后抓取使用未标定的模型。`GET /api/v1/calibration` 的 `load_error` 给出原因，直到一次 apply 或 restore 成功；恢复最近的备份或重新运行向导。

标定板（向导生成打印页；仿真器 `--aruco-board` 使用相同布局）：4 列 x 3 行（横向，与 16:9 画面一致）aruco4x4_50 标记，id 0..11 按阅读顺序（id = 行 * 4 + 列，第 0 行在页面顶部，第 0 列在左侧），默认标记边长 25 mm，间隔 5 mm（0.2 倍边长），整板约 115 x 85 mm。标定板坐标系：原点在板中心，x 沿页面向右，y 沿页面向下，z 指向桌面内；打印的 "up" 箭头指向页面顶部（板 -y）。标记角点顺序同 rebot-vision：打印标记的左下、右下、右上、左上（标记坐标系 x 向右、y 向下）。
