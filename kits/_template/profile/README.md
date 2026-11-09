# profile/

`example.json` is armd's generic 6-joint profile (`ArmConfig::default()`), written by
`UPDATE_PROFILE=1 cargo test --test test_arm_profile`; a test checks that it loads and
validates. Replace the joint names, limits and home pose with your arm's values. Optional
sections (`kinematics`, `workspace`, `gripper_policy`, `grasp`, `gravity_model`,
`gravity_comp`, `damiao_ranges`, `resting_pose`) turn on the matching features; armd reports
them in the schema's `capabilities`. `kinematics` and `gravity_model` only accept
`RebotDevArm` today.
