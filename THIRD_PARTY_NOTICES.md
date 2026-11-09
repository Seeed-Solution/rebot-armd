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
  that are not included here; armd does not load meshes. They are available from the
  upstream repository under the same license.

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

## Rust crates

The Rust dependencies are not vendored. Cargo downloads them from crates.io as listed in
`Cargo.toml` and pinned in `Cargo.lock`; each crate is under its own license (mostly MIT
and/or Apache-2.0). The license of every crate in the dependency tree can be listed with
[`cargo-license`](https://crates.io/crates/cargo-license) or checked with
[`cargo-deny`](https://crates.io/crates/cargo-deny). A statically linked `armd` binary
contains code from these crates and from the Rust standard library (MIT OR Apache-2.0)
and, for the reCamera build, musl libc (MIT).

## Not included

`third_party/` is listed in `.gitignore` and is not part of the repository. Older versions
of the package started a `go2rtc` process for video; the current package does not ship
`go2rtc` and its init script only stops a copy left by an older install.
