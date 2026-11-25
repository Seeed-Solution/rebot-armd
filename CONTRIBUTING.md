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

### Faster test runs

[cargo-nextest](https://nexte.st) runs each test in its own process and in parallel across all
test binaries; `.config/nextest.toml` holds the profiles.

```bash
cargo install cargo-nextest --locked                    # once
cargo nextest run --workspace --profile quick           # iteration lane: excludes slow integration suites, 10 s slow warning
cargo nextest run --workspace --profile fast            # day to day (~1375 of ~1550 tests)
cargo nextest run --workspace                           # every test (test-threads = "num-cpus")
cargo nextest run --workspace --profile ci              # full suite before a merge, then `cargo test --workspace` once
cargo nextest run --workspace -j $(( $(nproc) * 2 ))    # more test processes than CPUs (see below)
```

- `quick` is the fast iteration lane: it excludes the slow motion, vision, replay and multi-view
  integration suites (`test_mview_gray_bundle`, `test_ws_replay`, `test_move_path_joint`,
  `test_place`, `test_grasp`, `test_replay`, `test_poses`, `test_yaw_outward`,
  `test_teach_sequences`, `test_measure_multiview`, `test_mview_stereo`, `test_calib_hover`,
  and `test_5dof_place_execution_lateral_target`) and warns after 10 s.
- `fast` leaves out the ten test binaries that drive simulated motions or the multi-view
  measurement in real time (`test_grasp`, `test_measure_multiview`, `test_mview_stereo`,
  `test_mview_gray_bundle`, `test_move_path_joint`, `test_place`, `test_calib_hover`,
  `test_yaw_outward`, `test_teach_sequences`, `test_poses`). Run `ci` (or the default profile,
  which uses `test-threads = "num-cpus"`) when a change touches grasp, place, measurement,
  calibration hover, replay or motion planning, and before a merge.
- No profile retries a failing test, except `ci` for the wall-clock-rate tests
  (`test_damiao_smoothness`, `test_obs_rate`, `test_kincal_timing`, `test_alloc_timing`), which also
  run one at a time. A test is reported SLOW after 30 s and killed after 3 min.
- Most of the slow tests wait for 50 Hz simulated motion rather than use the CPU, so more test
  processes than CPUs shorten the run. Measured on spark (20 cores), full run: 117.8 s with
  `-j 20`, 76.6 s with `-j 40`.

Build settings that keep the test cycle short (in `Cargo.toml` and `.cargo/config.toml`):

- aarch64 Linux hosts link with rustup's `rust-lld` (`scripts/host-linker-lld.sh`); x86_64 Linux
  does by default. Set `CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=cc` to use the system linker.
- Dev and test builds carry line tables only (file:line in panics and backtraces). For a debugger
  session: `CARGO_PROFILE_DEV_DEBUG=full cargo test ...`.
- Dependencies and `mview-stereo` build at opt-level 2 in dev and test builds; armd itself stays at
  opt-level 0, so a change in `src/` rebuilds as before.

Several worktrees: keep one `target/` per worktree (the default). A shared `CARGO_TARGET_DIR`
makes parallel builds wait for each other's lock, and `target/debug/armd`, which the tests start,
would be the binary of whichever worktree built last. sccache (`RUSTC_WRAPPER=sccache`) speeds up
rebuilds in the same `target/` but not a new worktree: the compiler arguments carry the target
directory's path. Measured on spark with sccache 0.19 (2026-11-23): a second worktree built after
a first one with a warm cache got 2 Rust cache hits of 285, also with both roots in
`SCCACHE_BASEDIRS`. Keep a worktree's `target/` across branch switches instead of deleting it, and
`cargo clean` worktrees that are merged (a test build's `target/` is about 25 GB).

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
