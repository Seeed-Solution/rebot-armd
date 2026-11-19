# reCamera mount for the reBot B601-DM gripper

Holder that clamps a Seeed reCamera (with the B3/B4 CAN rear cover) to the gripper of the
reBot B601-DM arm. All files in this directory are licensed under the CERN Open Hardware
Licence Version 2 - Weakly Reciprocal (SPDX: `CERN-OHL-W-2.0`, full text in
[LICENSE](LICENSE)), not under the MIT License of the rest of the repository.

## Versions in this directory

| Directory | Camera pitch | Status |
|---|---|---|
| [`v5/`](v5/) | 0 deg (optical axis forward) | in use on the reference arm |
| [`v7_tilt/`](v7_tilt/) | 30 deg down | recommended v7 tilt when the camera has to see the jaw tips (see below); not printed or fitted |
| [`v8/`](v8/) | 38 deg down | recommended v8 tilt for the real 1080p-crop FOV (`v8/README.txt`); not printed or fitted |

Earlier drafts (v1-v4, v6_tilt) are not included: v6_tilt was based on v4 and is obsolete
(`v7_tilt/README.txt`). Only the recommended tilt per version (v7 30 deg, v8 38 deg) is kept
here; other angle variants (v7 20/25 deg, v8 42 deg), `*_cradle_flat.stl`, generation/validation
scripts, and raw output logs are stored in the upstream source repository
`Seeed-Projects/reBot-DevArm` (`hardware/camera-mounts/recamera-mounts/`).

Why 30 deg (v7): in the pinhole check summarised in `v7_tilt/README.txt` (section 2.3), the
jaw tip falls inside the image for an assumed half vertical FOV of 17 deg only at 25 and
30 deg, and 30 deg has the most margin (tip at 0.670 of the half image height; 20 deg: 1.235,
outside). The FOV values in v7 are assumptions, not measurements of the reCamera.

Why 38 deg (v8): the OV5647 sensor (2592x1944, f = 3.46 mm) streams 1920x1080p30 as a
1:1 centre crop (`74 % x 56 %` of the array, scaled down to 1280x720 and 640x360), giving a
derived FOV of 42.47 deg H x 24.65 deg V (half VFOV 12.33 deg, half HFOV 21.23 deg; `v8/README.txt`
section 1.1) instead of the 65 deg full-sensor spec or v7's assumed 17 deg half VFOV. With the
12.33 deg half VFOV, v7 30 deg puts the jaw tip at `v = +0.938` (0.3 % gripper coverage, rows
692-720 of 720), which drops out of frame with ~0.5 deg pitch error or 1-2 mm higher lens
position. In `v8/` (`v8/README.txt` sections 2-3):
- **38 deg (included)**: jaw tip at `v = +0.326` (row 477 of 720), 10.8 % closed gripper
  coverage (upper 65 % free), and stays within `v = -0.07 .. +0.72` (inside 80 % half-height)
  across +-5 mm lens height and +-2 deg pitch error.
- **42 deg (upstream only)**: jaw tip near image centre (`v = +0.018`; `v = -0.38 .. +0.42` over
  the same +-5 mm / +-2 deg combinations), 17.7 % closed gripper coverage, image top edge
  29.7 deg below the gripper axis (vs 25.7 deg at 38 deg).
- **Print orientation (`v8_tilt38_body_side_flat.stl`)**: places the largest flat side wall
  on the bed (3537 mm2 contact area, oriented for strength along the arm bending axis); the
  alternative `*_cradle_flat.stl` (3167 mm2 contact area) is kept upstream.

Each variant has two parts, printed separately: `*_body` and `*_top_bar`, as STEP and STL.
Assembly: `v5/ASSEMBLY.txt`; v7 print and assembly notes: `v7_tilt/README.txt` section 3;
v8 print and assembly notes follow v7 (`v8/README.txt` section 5).

| File | Contents |
|---|---|
| `v5/reCamera_B601_Mount_v5_{body,top_bar}.{step,stl}` | v5 parts (0 deg) |
| `v5/assembly_preview.png` | v5 preview on the gripper |
| `v5/ASSEMBLY.txt` | v5 print and assembly notes |
| `v7_tilt/reCamera_B601_Mount_v7_tilt30_{body,top_bar}.{step,stl}` | v7 30 deg parts |
| `v7_tilt/assembly_preview_tilt30.png` | v7 30 deg preview |
| `v7_tilt/README.txt` | v7 design, print, and assembly notes |
| `v8/reCamera_B601_Mount_v8_tilt38_{body,top_bar}.{step,stl}` | v8 38 deg parts |
| `v8/reCamera_B601_Mount_v8_tilt38_body_side_flat.stl` | v8 38 deg body oriented with the flat side wall on the print bed |
| `v8/assembly_preview_v8_tilt38.png` | v8 38 deg assembly preview (with FOV cone) |
| `v8/README.txt` | v8 design, print, and assembly notes |

Other angle variants (v7 20/25 deg, v8 42 deg), `*_cradle_flat.stl`, comparison and simulated
camera previews, generation/validation scripts, and raw output logs (`validation_output.txt`,
`generate_output.txt`, `visibility_output.txt`, `camera_view_output.txt`, `md5sums.txt`) are
stored in `Seeed-Projects/reBot-DevArm` (`hardware/camera-mounts/recamera-mounts/`).

## Origin of the clamp geometry

The clamp interface (mount y <= 0) of v5, v7, and v8 is copied from `D435_Gemini2_Mount.step` of
[reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm)
(`hardware/camera-mounts/b601-camera-mounts/D435_Gemini2_Mount.step`, CERN-OHL-W-2.0). The
validation reports 0 mm3 added or lost volume against that reference in the interface region.

## Regenerating the parts

The generation and validation scripts (`generate_recamera_mount.py`, `render_preview.py`,
`validate_tilt.py`, `validate_v8.py`, `optics.py`, `visibility.py`, `camera_view.py`) live in
`Seeed-Projects/reBot-DevArm` (`hardware/camera-mounts/recamera-mounts/<version>/`) and need
reference files that are **not** in this repository:

| File | Used by | Path the scripts expect | Where to get it |
|---|---|---|---|
| `D435_Gemini2_Mount.step` (sha256 `bf300894...6293b68`) | all generators, `validate_tilt.py`, `validate_v8.py` | `<script dir>/../../b601-camera-mounts/D435_Gemini2_Mount.step` | reBot-DevArm, `hardware/camera-mounts/b601-camera-mounts/` |
| `B3_CAN.step` (reCamera CAN rear cover) | all generators, `validate_tilt.py`, `validate_v8.py` | `<script dir>/../reference/B3_CAN.step` | not distributed here; its origin is not confirmed |
| `rs_arm.xml` (MJCF) and its meshes | `render_preview.py`, `validate_tilt.py` (gripper pose and RS meshes) | `<script dir>/../../../Rebot_Arm_description/RS/mjcf/rs_arm.xml`, `.../RS/meshes/` | reBot-DevArm, `Rebot_Arm_description/RS/` |
| `rebotarm_b601_stl.xml` (MJCF) and its meshes | `v8/optics.py` (used by `v8/validate_v8.py`, `v8/render_preview.py`, `v8/visibility.py`, `v8/camera_view.py`) | `<script dir>/../../../Rebot_Arm_description/DM/mjcf/rebotarm_b601_stl.xml`, `.../DM/meshes/` | reBot-DevArm, `Rebot_Arm_description/DM/` |

From a `reBot-DevArm` checkout under `hardware/camera-mounts/recamera-mounts/<version>/`, add
`recamera-mounts/reference/B3_CAN.step` and run from the version directory (command from
`v7_tilt/README.txt`; for `v8/`, run `validate_v8.py 38 42` in the same environment):

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
