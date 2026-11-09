# Contributing

## Setup

Requires a stable Rust toolchain (edition 2021). Node.js (20 or newer) runs the web cockpit
self-tests. Cross-compiling for the reCamera is described in the README
("Cross-compile for reCamera").

```bash
cargo build
cargo run -- --follower mock     # simulated arm, cockpit at http://localhost:8765/
```

## Checks

Run these before opening a pull request; CI (`.github/workflows/ci.yml`) runs the same.

```bash
cargo clippy --all-targets -- -D warnings
cargo test
node scripts/view_ik_selftest.mjs
node scripts/calib_selftest.mjs
node tests/test_rpc_error_contract.js
```

The tests use the mock arm and in-process servers; they do not need a robot arm or a camera.
`cargo fmt --check` is not enforced because the existing code is not rustfmt-formatted; do not
reformat files you do not otherwise change, so that diffs stay reviewable.

A change that affects motion, safety or a motor driver should also be tried on hardware. Say in
the pull request whether it was, on which arm and motors, and what was observed. Changes to the
safety behaviour should update `docs/control_safety_contract.md`; changes to actions should
update `docs/agent_action_api.md`.

If you change the kinematics in `src/kinematics.rs` or the cockpit IK, keep the fixtures in
`tests/fixtures/` in sync (`tests/fixtures/gen_vik_fixture.mjs` regenerates the cockpit IK
samples). rebot-companion ports the same kinematics.

## Commits

- One logical change per commit; keep unrelated changes apart.
- Subject in English, imperative mood, following the existing history:
  `type(scope): summary`, for example `fix(cockpit): sync the gripper target after grasp actions`.
  Types in use: `feat`, `fix`, `test`, `docs`, `ci`, `build`, `style`, `chore`.
- Explain in the body why the change is needed when it is not obvious from the diff.

## License

By contributing you agree that your contributions are licensed under the MIT License
(see `LICENSE`), except contributions to `kits/*/hardware/`, which are licensed under the
licence in that directory (CERN-OHL-W-2.0 for `kits/recamera-b601dm/hardware/`). Files
listed in `THIRD_PARTY_NOTICES.md` stay under their own licenses.
