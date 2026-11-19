# reCamera mount for the reBot B601-DM gripper

Holder that clamps a Seeed reCamera (with the B3/B4 CAN rear cover) to the gripper of the
reBot B601-DM arm. All files in this directory are licensed under the CERN Open Hardware
Licence Version 2 - Weakly Reciprocal (SPDX: `CERN-OHL-W-2.0`, full text in
[LICENSE](LICENSE)), not under the MIT License of the rest of the repository.

## Versions in this directory

| Directory | Camera pitch | Status |
|---|---|---|
| [`v5/`](v5/) | 0 deg (optical axis forward), 5 files | in use on the reference arm |
| [`v8/`](v8/) | 38 deg down, 5 files | recommended v8 tilt for the real 1080p-crop FOV; not printed or fitted |

Earlier drafts (`v1`–`v4`, `v6_tilt`), transitional `v7_tilt` drafts (20/25/30 deg), the
original-orientation `v8` body STL (`reCamera_B601_Mount_v8_tilt38_body.stl`), alternative angle
variants (v8 42 deg), `*_cradle_flat.stl`, generation/validation scripts, and full design notes
(`README.txt` / `ASSEMBLY.txt` and raw output logs) are all preserved in the upstream source
repository `Seeed-Projects/reBot-DevArm` (`hardware/camera-mounts/recamera-mounts/`).

Why 30 deg (upstream v7) vs 38 deg (`v8/`): the OV5647 sensor (2592x1944, f = 3.46 mm) streams
1920x1080p30 as a 1:1 centre crop (`74 % x 56 %` of the array, scaled down to 1280x720 and
640x360), giving a derived FOV of 42.47 deg H x 24.65 deg V (half VFOV 12.33 deg, half HFOV
21.23 deg) instead of the 65 deg full-sensor spec or v7's assumed 17 deg half VFOV. With the
12.33 deg half VFOV, v7 30 deg puts the jaw tip at `v = +0.938` (0.3 % gripper coverage, rows
692-720 of 720), which drops out of frame with ~0.5 deg pitch error or 1-2 mm higher lens
position. In `v8/`:
- **38 deg (included)**: jaw tip at `v = +0.326` (row 477 of 720), 10.8 % closed gripper
  coverage (upper 65 % free), and stays within `v = -0.07 .. +0.72` (inside 80 % half-height)
  across +-5 mm lens height and +-2 deg pitch error.
- **42 deg (upstream only)**: jaw tip near image centre (`v = +0.018`; `v = -0.38 .. +0.42` over
  the same +-5 mm / +-2 deg combinations), 17.7 % closed gripper coverage, image top edge
  29.7 deg below the gripper axis (vs 25.7 deg at 38 deg).
- **Print orientation (`v8_tilt38_body_side_flat.stl`)**: places the largest flat side wall
  on the bed (3537 mm2 contact area, oriented for strength along the arm bending axis); the
  original-orientation `v8_tilt38_body.stl` and `*_cradle_flat.stl` (3167 mm2 contact area) are
  kept upstream.

Each variant has two parts, printed separately: `*_body` and `*_top_bar`, provided as STEP and
STL (with `v8/` shipping the print-ready `*_body_side_flat.stl` plus `*_body.step`).

| File | Contents |
|---|---|
| `v5/reCamera_B601_Mount_v5_{body,top_bar}.{step,stl}` | v5 parts (0 deg, STEP and STL) |
| `v5/assembly_preview.png` | v5 preview on the gripper |
| `v8/reCamera_B601_Mount_v8_tilt38_body_side_flat.stl` | v8 38 deg body oriented with the flat side wall on the print bed |
| `v8/reCamera_B601_Mount_v8_tilt38_body.step` | v8 38 deg body CAD (STEP) |
| `v8/reCamera_B601_Mount_v8_tilt38_top_bar.{step,stl}` | v8 38 deg top bar (STEP and STL) |
| `v8/assembly_preview_v8_tilt38.png` | v8 38 deg assembly preview (with FOV cone) |

## Assembly

Both `v5/` and `v8/` share the same two-part retention geometry and fasteners:

- **Print separately**: print `*_body` and `*_top_bar` separately (`*_top_bar` is a flat 3 mm
  plate with ears, printed flat on its large face; for v8, `v8_tilt38_body_side_flat.stl` places
  the largest flat side wall on the bed). The four reCamera internal assembly holes are not used
  and are not reproduced in the holder.
- **Retention & orientation**: camera is held by the rear stop, front lower lip, front side
  guards, and removable top bar. Orient the camera with the CAN rear cover `-X` side facing up
  and `+/-Y` sides facing left/right (the rear-depth region remains open on those three faces; do
  not install in the old v4 roll orientation). Image orientation may require software rotation.
- **Assembly steps**: remove the top bar, lower the camera into the holder, insert square nuts
  from the outer side-open pockets (`5.8 mm` along depth, `2.6 mm` high), then attach the top bar
  using two fasteners in the **printed ears only** (`3.2 mm` clearance holes).
- **Fastener & fit precautions**: verify actual square nut and screw fit/length before
  tightening; **do not drive screws into the camera**. Thin non-slip pads may take up the
  nominal `0.3 mm` clearance; do not force an interference fit against the enclosure.
- **Clamp interface**: the clamp interface (`y <= 0`) and its fasteners are identical across v5
  and v8.

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
`recamera-mounts/reference/B3_CAN.step` and run from the version directory (for `v8/`, run
`validate_v8.py 38 42` in the same environment; for upstream `v7_tilt`, run `validate_tilt.py`):

```bash
timeout 600 uv run --python 3.11 --with build123d --with trimesh \
  --with matplotlib --with scipy --with numpy --with manifold3d \
  --with rtree python validate_tilt.py v5 20 25 30
```

`v7_tilt/generate_recamera_mount.py` and `v8/generate_recamera_mount.py` also read the v5 STEP files from `../v5/`.

## Not validated

From the upstream v5 and v7/v8 design notes:

- Physical fit on the arm and seating of the clamp (v8 has not been printed or fitted).
- Print tolerance, print orientation and supports; real fastener and nut fit.
- Real reCamera FOV, lens position inside the housing and distortion (the FOV values are inputs).
- Cable routing and connector access (keep-out boxes are checked; real overmolds and bend radii are not).
- Strength and stiffness, and retention under arm loads.
- Arm motion beyond the gripper body and jaws; the camera is modelled as a 40 x 40 x 36.5 mm box.

The camera mount extrinsics used by armd (`camera_mount` in the arm profile) are not derived
from these files; see [../calibration/](../calibration/).
