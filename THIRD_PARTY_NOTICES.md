# Third-party notices

rebot-armd code is licensed under the MIT License (see `LICENSE`). The kit hardware in
`kits/recamera-b601dm/hardware/` is licensed under CERN-OHL-W-2.0 (see that directory's
`LICENSE`). The files listed below come from other projects and stay under their original
licenses.

## JMuxer

- File: `src/api/vendor/jmuxer.min.js` (embedded in the `armd` binary and served at
  `/assets/jmuxer.min.js`; the web cockpit uses it to play the camera H.264 stream).
- Upstream: https://github.com/samirkumardas/jmuxer, npm `jmuxer@2.1.0`, file
  `dist/jmuxer.min.js`, unmodified. The header comment in the file gives the sha256 of the
  upstream file and how to check it.
- License: MIT. Copyright (c) 2018 Samir Das.

```text
(The MIT License)

Copyright (c) 2018 Samir Das <cse.samir@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
```

## reBot-DevArm robot description (URDF)

- Files:
  - `assets/reBot-DevArm_gripper.urdf`
  - `kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets/reBot-DevArm_gripper.urdf`
    (same file, installed by the `.deb`). armd does not parse it at runtime: it checks the
    file's sha256 at startup, and the gravity model in `src/gravity.rs` uses link
    parameters taken from it.
- Upstream: https://github.com/Seeed-Projects/reBot-DevArm (robot description of the
  reBot B601-DM arm).
- License: CERN Open Hardware Licence Version 2 - Weakly Reciprocal (CERN-OHL-W-2.0), the
  license of the upstream repository. Full text: https://ohwr.org/cern_ohl_w_v2.txt
- Modifications: the arm chain (`base_link` .. `end_link`) is taken from the upstream
  fixed-end URDF. The `end_link` visual is replaced by a gripper body mesh, and two
  prismatic fingers (`left_finger`, `right_finger` and their joints) are appended; the
  header comment describes the additions. These files
  are not covered by the MIT License of this repository.
- The URDF refers to mesh files (`package://reBot-DevArm_description_fixend/meshes/*.STL`)
  that are not included here. armd does not load meshes; the cockpit 3D view serves this URDF
  from `/assets/model/` and replaces those meshes with the decimated DM meshes listed below.

## three.js and urdf-loader (cockpit 3D view)

- File: `src/api/vendor/three-urdf.min.js.gz` (embedded in the `armd` binary and served at
  `/assets/three-urdf.min.js` with `Content-Encoding: gzip`; the cockpit loads it only when the
  3D view is opened). One minified ES module built by `scripts/vendor_three.mjs` with esbuild
  0.25.10 from npm `three@0.186.1` (the classes the view uses, `OrbitControls`, `STLLoader`) and
  `urdf-loader@0.13.1` (`src/URDFLoader.js`, `src/URDFClasses.js`). Modifications: bundled and
  minified; urdf-loader's import of three.js `ColladaLoader` is replaced by a stub that throws.
  sha256 of the `.gz` file: `c5ab604e2dfeb6daa9fe436e54d069515b53b45581c00ad021d7b2594fffb133`.
- three.js: https://github.com/mrdoob/three.js, License: MIT.

```text
The MIT License

Copyright © 2010-2026 three.js authors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

- urdf-loader: https://github.com/gkjohnson/urdf-loaders (`javascript/`), License: Apache-2.0
  (full text: `kits/recamera-b601dm/vision/LICENSE-APACHE`, the same Apache License 2.0).

```text
Copyright © 2020 California Institute of Technology. ALL RIGHTS
RESERVED. United States Government Sponsorship Acknowledged.
Neither the name of Caltech nor its operating division, the
Jet Propulsion Laboratory, nor the names of its contributors may be
used to endorse or promote products derived from this software
without specific prior written permission.
```

## reBot B601-DM visual meshes (cockpit 3D view)

- Files: `kits/recamera-b601dm/packaging/rootfs/usr/share/rebot-teleop/assets/meshes/` (34 STL
  files, `manifest.json`, `NOTICE.md`, `LICENSE-CERN-OHL-W-2.0.txt`; 2.8 MB), installed by the
  `.deb` to `/usr/share/rebot-teleop/assets/meshes/` and served by armd at
  `/assets/model/meshes/`. Not embedded in the binary and not part of the lite tarball.
- Upstream: https://github.com/Seeed-Projects/reBot-DevArm, `Rebot_Arm_description/DM/meshes/visual`
  and `meshes/shared`, commit `b2287b6`. Copyright 2026 Seeed Studio Co., Ltd.
- License: CERN-OHL-W-2.0; the full text is installed next to the meshes.
- Modifications (2026-11-11): simplified to about 12 % of the triangles (at least 300 per part) by
  `scripts/decimate_meshes.py`, `colored_` prefix removed, `manifest.json` added (part to link
  mapping and colours from the upstream URDF materials). See `NOTICE.md` in that directory.

## reBot B601-RS robot description (URDF)

- File: `assets/ReBot_Arm_RS.urdf`, read at startup by armd when a profile names it
  (`profiles/rebot_b601_rs.json`).
- Upstream: https://github.com/Seeed-Projects/reBot-DevArm,
  `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf` at commit `a3155aa`, unmodified
  (sha256 `3838ca8f9d586be22a89b33c7cbdd4abd43d4a92a2de377b294eef4c5bd7654d`).
- License: CERN-OHL-W-2.0, as the B601-DM URDF above. Not covered by the MIT License of this
  repository. The meshes it refers to are not included.

## reBot B601-DM MuJoCo scene (simulator)

- File: `sim/mujoco/models/rebot_b601dm_scene.xml`.
- Derived from: the arm chain and inertials of `assets/reBot-DevArm_gripper.urdf` (above) and
  the gripper collision extents of `Rebot_Arm_description/DM/mjcf/rebotarm_b601_colored.xml` of
  https://github.com/Seeed-Projects/reBot-DevArm.
- License: CERN-OHL-W-2.0 (https://ohwr.org/cern_ohl_w_v2.txt). Not covered by the MIT License
  of this repository. The file header records the modifications (box primitives instead of
  meshes, actuators, table and cube; 2026-11-11). No upstream meshes are included; the
  simulator loads them from a reBot-DevArm checkout named by `REBOT_DEVARM_DIR`, when set.

## reCamera mount CAD (kit hardware)

- Files: `kits/recamera-b601dm/hardware/` (v5 and v7_tilt STEP/STL, previews, generator,
  preview and validation scripts).
- Derived from: `hardware/camera-mounts/b601-camera-mounts/D435_Gemini2_Mount.step` of
  https://github.com/Seeed-Projects/reBot-DevArm; the clamp interface is copied from it.
- License: CERN-OHL-W-2.0, full text in `kits/recamera-b601dm/hardware/LICENSE` (official
  text from https://ohwr.org/cern_ohl_w_v2.txt). Not covered by the MIT License of this
  repository.
- The reference files the scripts read (`D435_Gemini2_Mount.step`, `B3_CAN.step`, the
  reBot-DevArm MJCF and meshes) are not included; see the directory's `README.md`.

## rebot-vision (reCamera runtime)

`rebot-vision` is the kit's camera process (`kits/recamera-b601dm/vision/`, installed as
`/usr/bin/rebot-vision` by the `rebot-teleop` deb). Its sources and the binary include code
from the projects below.

### sscma-example-sg200x application code (copied)

- Upstream: https://github.com/Seeed-Studio/sscma-example-sg200x (fork
  https://github.com/suharvest/sscma-example-sg200x, branch `main`), commit
  `9af8118e556c01872ed2cd583abd0c1191df9c6c`.
- License: Apache-2.0, text in `kits/recamera-b601dm/vision/LICENSE-APACHE` (installed as
  `/usr/share/doc/rebot-teleop/LICENSE-APACHE`). Copyright Seeed Technology Co., Ltd.
- Copied files (path in this repository <- path upstream). "unchanged" = byte-identical at
  the import commit; files marked "modified" carry a notice at the top (Apache-2.0 4(b)).

| This repository (`kits/recamera-b601dm/vision/`) | Upstream (`solutions/`) | State |
|---|---|---|
| `main/marker_filter.h`, `main/marker_filter.cpp` | `aruco-detector/main/` | unchanged |
| `main/camera_model.h`, `main/camera_model.cpp` | `aruco-detector/main/` | unchanged |
| `main/capture_time.h` | `aruco-detector/main/` | unchanged |
| `main/http_control.h`, `main/http_control.cpp` | `aruco-detector/main/` | modified |
| `main/detector.h`, `main/detector.cpp` | `yolo-detector/main/` | unchanged |
| `main/vision_control.h`, `main/vision_control.cpp` | `aruco-detector/main/inference_control.{h,cpp}` | modified (modes added) |
| `main/aruco_stage.h`, `main/aruco_stage.cpp` | `aruco-detector/main/main.cpp` (frame handler, family table) | modified |
| `main/main.cpp` | `aruco-detector/main/main.cpp`, `yolo-detector/main/main.cpp` | modified (merged) |
| `tests/marker_filter_test.cpp`, `tests/camera_model_test.cpp`, `tests/capture_time_test.cpp` | `aruco-detector/tests/` | unchanged |
| `tests/vision_control_test.cpp` | `aruco-detector/tests/inference_control_test.cpp` | modified |

`main/tag_pose.h`, `main/tag_pose.cpp` are a port of `orthogonal_iteration()` and
`estimate_tag_pose()` from `components/apriltag/apriltag_pose.c` of the same checkout
(AprilTag, BSD-2-Clause, Copyright (C) 2013-2016 The Regents of The University of Michigan;
licence text below).

`apriltag_patched/` holds modified copies of files of `components/apriltag` of the same
checkout (AprilTag, BSD-2-Clause, licence text below); the build uses them in place of the
SDK's files of the same name. Each file keeps its licence header and lists its changes below
it.

### Components compiled into the binary (not in this repository)

The build uses an `sscma-example-sg200x` checkout as an SDK (`SSCMA_ROOT`, pinned in
`kits/recamera-b601dm/vision/sscma.lock`). Statically linked from it:

| Component | License | Notice |
|---|---|---|
| `components/app_framework`, `debug_stream`, `rtsp_server`, `ha_mqtt`, `onvif_service`, `sophgo` | Apache-2.0 (repository license) | Copyright Seeed Technology Co., Ltd.; text in `LICENSE-APACHE` |
| `components/sscma-micro` | Apache-2.0 (`components/sscma-micro/sscma-micro/LICENSE`) | Copyright Seeed Technology Co., Ltd.; text in `LICENSE-APACHE` |
| `sscma-micro/3rdparty/eigen` (headers) | MPL-2.0 | unmodified upstream Eigen, source at https://gitlab.com/libeigen/eigen; MPL text at https://mozilla.org/MPL/2.0/ |
| `components/json` (nlohmann/json) | MIT, Copyright (c) 2013-2022 Niels Lohmann | MIT text as for JMuxer above, with that copyright line |
| `components/apriltag` (with `aruco/` dictionaries) | BSD-2-Clause | text below |
| `libwebsockets` (built by `components/libwebsockets/fetch_and_build.sh`, linked statically) | MIT | https://github.com/warmcat/libwebsockets/blob/main/LICENSE |

Linked dynamically against the device's copies and not shipped in the deb: Sophgo MPI /
cviruntime / TPU SDK libraries (`/mnt/system/lib`, vendor license), mosquitto (EPL-2.0 /
EDL-1.0, pulled in by `ha_mqtt`; MQTT is not used at runtime), OpenSSL / c-ares from the
reCamera OS.

YOLO model files (`.cvimodel`) are not in this repository or the deb. `rebot-vision` loads the
model already on the device (see `kits/recamera-b601dm/vision/README.md`); Ultralytics YOLO
weights are AGPL-3.0 unless another license applies to the converted file.

AprilTag (`components/apriltag`):

```text
Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
```

## Rust crates

The Rust dependencies are not vendored. Cargo downloads them from crates.io as listed in
`Cargo.toml` and pinned in `Cargo.lock`; each crate is under its own license (mostly MIT
and/or Apache-2.0). The license of every crate in the dependency tree can be listed with
[`cargo-license`](https://crates.io/crates/cargo-license) or checked with
[`cargo-deny`](https://crates.io/crates/cargo-deny). A statically linked `armd` binary
contains code from these crates and from the Rust standard library (MIT OR Apache-2.0)
and, for the reCamera build, musl libc (MIT).

Crates added for a specific feature:

- `roxmltree` 0.21 (MIT OR Apache-2.0, Copyright (c) 2018 Yevhenii Reizner): read-only
  XML tree used by `src/urdf.rs` to read the URDF named by a profile's `urdf` field. It has
  no dependencies of its own; a hand-written XML reader would be more code to review than
  the crate, and the URDF is only read once at startup.

## Not included

`third_party/` is listed in `.gitignore` and is not part of the repository. Older versions
of the package started a `go2rtc` process for video; the current package does not ship
`go2rtc` and its init script only stops a copy left by an older install.
