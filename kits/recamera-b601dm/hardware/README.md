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

Earlier drafts (v1-v4, v6_tilt) are not included: v6_tilt was based on v4 and is obsolete
(`v7_tilt/README.txt`).

Why 30 deg: in the pinhole check of `v7_tilt/validation_output.txt` (summarised in
`v7_tilt/README.txt`, section 2.3), the jaw tip falls inside the image for an assumed half
vertical FOV of 17 deg only at 25 and 30 deg, and 30 deg has the most margin (tip at 0.670
of the half image height; 20 deg: 1.235, outside). The FOV values are assumptions, not
measurements of the reCamera.

Each variant has two parts, printed separately: `*_body` and `*_top_bar`, as STEP and STL.
Assembly: `v5/ASSEMBLY.txt`; v7 print and assembly notes: `v7_tilt/README.txt` section 3.

| File | Contents |
|---|---|
| `v5/reCamera_B601_Mount_v5_{body,top_bar}.{step,stl}` | v5 parts |
| `v5/assembly_preview.png` | v5 preview on the gripper |
| `v5/generate_recamera_mount.py`, `v5/render_preview.py` | v5 generator and preview script |
| `v7_tilt/reCamera_B601_Mount_v7_tilt{20,25,30}_{body,top_bar}.{step,stl}` | v7 parts per tilt |
| `v7_tilt/assembly_preview_tilt{20,25,30}.png` | v7 previews |
| `v7_tilt/generate_recamera_mount.py [TILT]`, `render_preview.py [TILT]`, `validate_tilt.py TILT... \| v5` | v7 generator, preview, validation |
| `v7_tilt/README.txt`, `v7_tilt/validation_output.txt` | v7 design notes and raw validation output (includes the STL md5 sums) |

The STL files here match the md5 sums listed in `v7_tilt/validation_output.txt`.

## Origin of the clamp geometry

The clamp interface (mount y <= 0) of v5 and v7 is copied from `D435_Gemini2_Mount.step` of
[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)
(`hardware/camera-mounts/b601-camera-mounts/D435_Gemini2_Mount.step`, CERN-OHL-W-2.0). The
validation reports 0 mm3 added or lost volume against that reference in the interface region.

## Regenerating the parts

The scripts need reference files that are **not** in this repository. Get them separately:

| File | Used by | Path the scripts expect | Where to get it |
|---|---|---|---|
| `D435_Gemini2_Mount.step` (sha256 `bf300894...6293b68`) | both generators, `validate_tilt.py` | `<script dir>/../../b601-camera-mounts/D435_Gemini2_Mount.step` | reBot-DevArm, `hardware/camera-mounts/b601-camera-mounts/` |
| `B3_CAN.step` (reCamera CAN rear cover) | both generators, `validate_tilt.py` | `<script dir>/../reference/B3_CAN.step` | not distributed here; its origin is not confirmed |
| `rs_arm.xml` (MJCF) and its meshes | `render_preview.py`, `validate_tilt.py` (gripper pose and meshes) | `<script dir>/../../../Rebot_Arm_description/RS/mjcf/rs_arm.xml`, `.../RS/meshes/` | reBot-DevArm, `Rebot_Arm_description/RS/` |

The relative paths come from the reBot-DevArm layout the scripts were written in
(`hardware/camera-mounts/recamera-mounts/<version>/`). The simplest way to regenerate is to
copy `v5/` and `v7_tilt/` into `hardware/camera-mounts/recamera-mounts/` of a reBot-DevArm
checkout, add `recamera-mounts/reference/B3_CAN.step`, and run from the version directory
(command from `v7_tilt/README.txt`):

```bash
timeout 600 uv run --python 3.11 --with build123d --with trimesh \
  --with matplotlib --with scipy --with numpy --with manifold3d \
  --with rtree python validate_tilt.py v5 20 25 30
```

`v7_tilt/generate_recamera_mount.py` also reads the v5 STEP files from `../v5/`.

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
