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
