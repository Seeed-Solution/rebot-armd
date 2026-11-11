# reBot B601-DM visual meshes

Source: https://github.com/Seeed-Projects/reBot-DevArm, directory `Rebot_Arm_description/DM/meshes/visual` and
`Rebot_Arm_description/DM/meshes/shared`, commit b2287b6.
Copyright 2026 Seeed Studio Co., Ltd. Licensed under CERN-OHL-W-2.0 (`LICENSE-CERN-OHL-W-2.0.txt`).

Modifications (2026-11-11): each part was simplified to about 12 % of its triangles (at least 300) with
`scripts/decimate_meshes.py` of the armd repository, the `colored_` file name prefix was removed, and
`manifest.json` maps the parts and their URDF material colours to the links of `reBot-DevArm_gripper.urdf`.
The unmodified source is the directory named above.
