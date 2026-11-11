# profile/

`example.json` is armd's generic 6-joint profile (`ArmConfig::default()`), written by
`UPDATE_PROFILE=1 cargo test --test test_arm_profile`; a test checks that it loads and
validates. Replace the joint names, limits and home pose with your arm's values. Optional
sections (`kinematics`, `workspace`, `gripper_policy`, `grasp`, `gravity_model`,
`gravity_comp`, `damiao_ranges`, `resting_pose`) turn on the matching features; armd reports
them in the schema's `capabilities`. `kinematics` and `gravity_model` accept `RebotDevArm`
(the built-in B601-DM model) or `Urdf` (below).

`example_urdf.json` takes the kinematic and gravity model from a URDF instead
(`"kinematics": "Urdf"`, `"gravity_model": "Urdf"`):

- `urdf`: the URDF file, relative to the profile file (put `my-arm.urdf` next to it).
- `urdf_sha256`: optional; armd refuses to start when the file's sha256 differs.
- `chain.base_link` / `chain.tip_link`: the arm inside the URDF. `tip_link` is the frame the
  workspace check points and Cartesian targets refer to.
- `chain.joints`: one entry per arm joint, in armd joint order, which must be the order of
  the revolute joints from `base_link` to `tip_link`. URDF angle = `sign` × armd angle +
  `offset_rad`; set them from the motors' zero positions and directions.
- `gripper_joint`: optional gripper joint index (after the arm joints). Without it, a
  profile with one joint more than the chain has its gripper at that joint.

Gravity torques use the URDF `<inertial>` masses and centres of mass of every link below
`base_link`. The Cartesian actions (`move_*`, grasp, place) and gravity compensation need 6
arm joints with the gripper at joint 7; the workspace check and the gravity model work for
any chain length. [`profiles/rebot_b601_rs.json`](../../../profiles/rebot_b601_rs.json) is a
complete example (B601-RS, not verified on hardware).
