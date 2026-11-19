# reCamera mount for the reBot B601-DM gripper

Holder that clamps a Seeed reCamera (with the B3/B4 CAN rear cover) to the gripper of the
reBot B601-DM arm. All files in this directory are licensed under the CERN Open Hardware
Licence Version 2 - Weakly Reciprocal (SPDX: `CERN-OHL-W-2.0`, full text in
[LICENSE](LICENSE)), not under the MIT License of the rest of the repository.

## Versions in this directory

| Directory | Camera pitch | Status |
|---|---|---|
| [`v5/`](v5/) | 0 deg (optical axis forward) | in use on the reference arm |
| [`v7_tilt/`](v7_tilt/) | 20, 25, 30 deg down | 30 deg recommended when the camera has to see the jaw tips (see below); not printed or fitted |
| [`v8/`](v8/) | 38, 42 deg down | 38 deg recommended (42 deg alternative) for the real 1080p-crop FOV (`v8/README.txt`); not printed or fitted |

Earlier drafts (v1-v4, v6_tilt) are not included: v6_tilt was based on v4 and is obsolete
(`v7_tilt/README.txt`).

Why 30 deg (v7): in the pinhole check of `v7_tilt/validation_output.txt` (summarised in
`v7_tilt/README.txt`, section 2.3), the jaw tip falls inside the image for an assumed half
vertical FOV of 17 deg only at 25 and 30 deg, and 30 deg has the most margin (tip at 0.670
of the half image height; 20 deg: 1.235, outside). The FOV values in v7 are assumptions, not
measurements of the reCamera.

Why 38 and 42 deg (v8): the OV5647 sensor (2592x1944, f = 3.46 mm) streams 1920x1080p30 as a
1:1 centre crop (`74 % x 56 %` of the array, scaled down to 1280x720 and 640x360), giving a
derived FOV of 42.47 deg H x 24.65 deg V (half VFOV 12.33 deg, half HFOV 21.23 deg; `v8/README.txt`
section 1.1) instead of the 65 deg full-sensor spec or v7's assumed 17 deg half VFOV. With the
12.33 deg half VFOV, v7 30 deg puts the jaw tip at `v = +0.938` (0.3 % gripper coverage, rows
692-720 of 720), which drops out of frame with ~0.5 deg pitch error or 1-2 mm higher lens
position. In `v8/` (`v8/README.txt` sections 2-3):
- **38 deg (recommended)**: jaw tip at `v = +0.326` (row 477 of 720), 10.8 % closed gripper
  coverage (upper 65 % free), and stays within `v = -0.07 .. +0.72` (inside 80 % half-height)
  across +-5 mm lens height and +-2 deg pitch error.
- **42 deg (alternative)**: jaw tip near image centre (`v = +0.018`; `v = -0.38 .. +0.42` over
  the same +-5 mm / +-2 deg combinations), 17.7 % closed gripper coverage, image top edge
  29.7 deg below the gripper axis (vs 25.7 deg at 38 deg).
- **Print orientation variants** (`v8_tilt38_body`): `*_side_flat.stl` places the largest flat
  side wall on the bed (3537 mm2 contact area, oriented for strength along the arm bending axis);
  `*_cradle_flat.stl` rotates the body 38 deg so the camera cavity base lies flat on the bed
  (3167 mm2 contact area; see `v8/print_orientation_comparison.png`).

Each variant has two parts, printed separately: `*_body` and `*_top_bar`, as STEP and STL.
Assembly: `v5/ASSEMBLY.txt`; v7 print and assembly notes: `v7_tilt/README.txt` section 3;
v8 print and assembly notes follow v7 (`v8/README.txt` section 5).

| File | Contents |
|---|---|
| `v5/reCamera_B601_Mount_v5_{body,top_bar}.{step,stl}` | v5 parts |
| `v5/assembly_preview.png` | v5 preview on the gripper |
| `v5/generate_recamera_mount.py`, `v5/render_preview.py` | v5 generator and preview script |
| `v7_tilt/reCamera_B601_Mount_v7_tilt{20,25,30}_{body,top_bar}.{step,stl}` | v7 parts per tilt |
| `v7_tilt/assembly_preview_tilt{20,25,30}.png` | v7 previews |
| `v7_tilt/generate_recamera_mount.py [TILT]`, `render_preview.py [TILT]`, `validate_tilt.py TILT... \| v5` | v7 generator, preview, validation |
| `v7_tilt/README.txt`, `v7_tilt/validation_output.txt` | v7 design notes and raw validation output (includes the STL md5 sums) |
| `v8/reCamera_B601_Mount_v8_tilt{38,42}_{body,top_bar}.{step,stl}` | v8 parts per tilt (38 and 42 deg) |
| `v8/reCamera_B601_Mount_v8_tilt38_body_{cradle_flat,side_flat}.stl` | v8 38 deg body print orientation variants |
| `v8/assembly_preview_v8_tilt{38,42}.png`, `v8/print_orientation_comparison.png` | v8 assembly previews (with FOV cone) and print bed orientation comparison |
| `v8/camera_view_{v8_tilt38,v8_tilt42,v7_tilt30_realFOV,roll90_tilt30}.png` | MuJoCo simulated camera views at real 1080p-crop FOV |
| `v8/generate_recamera_mount.py [TILT]`, `v8/validate_v8.py TILT...`, `v8/render_preview.py [TILT]`, `v8/optics.py`, `v8/visibility.py`, `v8/camera_view.py` | v8 generator, validation, preview, optics model, visibility sweep, simulated camera view |
| `v8/README.txt`, `v8/validation_output.txt`, `v8/generate_output.txt`, `v8/visibility_output.txt`, `v8/camera_view_output.txt`, `v8/md5sums.txt` | v8 design notes, raw command outputs, and STEP/STL md5 sums |

The STL files in `v7_tilt/` match the md5 sums listed in `v7_tilt/validation_output.txt`, and the v8 STEP/STL files match `v8/md5sums.txt`.

## Origin of the clamp geometry

The clamp interface (mount y <= 0) of v5, v7, and v8 is copied from `D435_Gemini2_Mount.step` of
[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)
(`hardware/camera-mounts/b601-camera-mounts/D435_Gemini2_Mount.step`, CERN-OHL-W-2.0). The
validation reports 0 mm3 added or lost volume against that reference in the interface region.

## Regenerating the parts

The scripts need reference files that are **not** in this repository. Get them separately:

| File | Used by | Path the scripts expect | Where to get it |
|---|---|---|---|
| `D435_Gemini2_Mount.step` (sha256 `bf300894...6293b68`) | all generators, `validate_tilt.py`, `validate_v8.py` | `<script dir>/../../b601-camera-mounts/D435_Gemini2_Mount.step` | reBot-DevArm, `hardware/camera-mounts/b601-camera-mounts/` |
| `B3_CAN.step` (reCamera CAN rear cover) | all generators, `validate_tilt.py`, `validate_v8.py` | `<script dir>/../reference/B3_CAN.step` | not distributed here; its origin is not confirmed |
| `rs_arm.xml` (MJCF) and its meshes | `render_preview.py`, `validate_tilt.py` (gripper pose and RS meshes) | `<script dir>/../../../Rebot_Arm_description/RS/mjcf/rs_arm.xml`, `.../RS/meshes/` | reBot-DevArm, `Rebot_Arm_description/RS/` |
| `rebotarm_b601_stl.xml` (MJCF) and its meshes | `v8/optics.py` (used by `v8/validate_v8.py`, `v8/render_preview.py`, `v8/visibility.py`, `v8/camera_view.py`) | `<script dir>/../../../Rebot_Arm_description/DM/mjcf/rebotarm_b601_stl.xml`, `.../DM/meshes/` | reBot-DevArm, `Rebot_Arm_description/DM/` |

The relative paths come from the reBot-DevArm layout the scripts were written in
(`hardware/camera-mounts/recamera-mounts/<version>/`). The simplest way to regenerate is to
copy `v5/`, `v7_tilt/`, and `v8/` into `hardware/camera-mounts/recamera-mounts/` of a reBot-DevArm
checkout, add `recamera-mounts/reference/B3_CAN.step`, and run from the version directory
(command from `v7_tilt/README.txt`; for `v8/`, run `validate_v8.py 38 42` in the same environment):

```bash
timeout 600 uv run --python 3.11 --with build123d --with trimesh \
  --with matplotlib --with scipy --with numpy --with manifold3d \
  --with rtree python validate_tilt.py v5 20 25 30
```

`v7_tilt/generate_recamera_mount.py` and `v8/generate_recamera_mount.py` also read the v5 STEP files from `../v5/`.

## Not validated

From `v5/ASSEMBLY.txt` and `v7_tilt/README.txt` section 4:

- Physical fit on the arm and seating of the clamp (v7 has not been printed).
- Print tolerance, print orientation and supports; real fastener and nut fit.
- Real reCamera FOV, lens position inside the housing and distortion (the FOV values are inputs).
- Cable routing and connector access (keep-out boxes are checked; real overmolds and bend radii are not).
- Strength and stiffness, and retention under arm loads.
- Arm motion beyond the gripper body and jaws; the camera is modelled as a 40 x 40 x 36.5 mm box.

The camera mount extrinsics used by armd (`camera_mount` in the arm profile) are not derived
from these files; see [../calibration/](../calibration/).
