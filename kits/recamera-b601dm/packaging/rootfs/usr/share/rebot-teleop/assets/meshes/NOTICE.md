# reBot B601-DM visual meshes

Source: https://github.com/Seeed-Projects/reBot-DevArm, directory `Rebot_Arm_description/DM/meshes/visual` and
`Rebot_Arm_description/DM/meshes/shared`, commit b2287b6.
Copyright 2026 Seeed Studio Co., Ltd. Licensed under CERN-OHL-W-2.0 (`LICENSE-CERN-OHL-W-2.0.txt`).

Modifications (2026-11-12): each part was simplified with meshoptimizer to a surface deviation of at most
0.5 mm (from 430,912 to 49,336 triangles in total) by `scripts/decimate_meshes.py` of the armd repository,
and all parts were packed into `arm.bin` (vertex positions quantized to 16 bits per axis within each part's
bounding box, triangle indices). `manifest.json` maps the parts and their URDF material colours to the
links of `reBot-DevArm_gripper.urdf` and gives each part's offset in `arm.bin`; the byte layout is in the
docstring of that script. The unmodified source is the directory named above.
