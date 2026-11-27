# rebot-armd 1.6.0 release notes

[中文](release_notes_1.6.0.zh-CN.md)

`rebot-armd` for the Seeed reCamera (SG2002) with the reBot B601-DM arm. Follows 1.4.0 (about 1,470 commits).

1.6.0 is the `rel/1.6.0-rc7` line (HEAD `dcc2173a`). Real-arm results below were measured on rc7 (`aebc6ea3`) and on rc7.2 (rc7 plus the five later fixes: `fix/init-stop-companions`, `fix/cockpit-stopped-quiet`, `fix/workspace-messages`, `fix/stamp-namespaces`, `fix/workspace-recovery`). rc7.3 adds only the cockpit "Grasp approach" row; the final 1.6.0 build differs from rc7.3 only in its top-down default.

## Assets

| File | What it is |
|---|---|
| `rebot-teleop_1.6.0_riscv64.deb` | The reCamera package: `armd` (arm control, web cockpit on `:8765`) and `rebot-vision` (H.264 video, YOLO, segmentation, ArUco and depth on demand), plus the tuned vision models (see below). Install with `opkg install` or from the reCamera console. |
| `armd-1.6.0-riscv64-musl` | The `armd` binary alone (static, riscv64 musl). |
| `rebot-vision-1.6.0-riscv64` | The vision binary alone. Use it with `REBOT_VISION_BIN=<path> scripts/build_deb.sh` to build the package without the reCamera SDK. |
| `SHA256SUMS` | Checksums of the three files. |

The same files are on the CDN: https://sensecraft-statics.seeed.cc/solution-app/rebot/releases/1.6.0/

The OBB model (`yolo11n_obb_cv181x_int8_fuse_qout.cvimodel`) is not in the package: `scripts/fetch_models.sh` downloads it (list and licenses in `kits/recamera-b601dm/vision/models/models.tsv`), then restart rebot-teleop with the arm idle.

## What's New

### Real-time control loop

- Motion planning runs outside the supervisor lock. Jog, `run_sequence` and the other legacy WebSocket handlers hold the lock for at most 0.164 ms in the host test (48 handler cases; `run_sequence` held it 2.6 ms before).
- The terminal-observation capture is edge-triggered (it was level-triggered and repeated every tick during a dwell). Torque enable and disable no longer hold the supervisor lock across the enable polling: on the host, a legacy torque enable that takes 35 ms synchronously holds the guard for at most 339 µs, and disable for 409 µs.
- SG2002 priority layout, applied by the init script: `armd-control` SCHED_FIFO 79, `spi0` and its IRQ thread FIFO 80, the `rebot-vision` SDK threads and the four unpinned `cvitask` threads at 70. The vendor kernel caps non-`cvitask` threads at priority 80 (`MAX_OPT_RT_PRIORITY`) and fixes the pinned `isp_pre`, `vpss`, `tpu` and `vc` threads at 85–99. Measured on a spare reCamera with a mock follower: wake lateness max 5.6 ms (14.4 ms before the layout).
- Bounded CAN TX budget with carry-over of deferred frames on both owners.
- Real-arm measurements of this release are in "Test results".

### RT executor owner (default) with legacy fallback

- The executor thread owns the arm by default. Startup owner precedence: `armd --owner <auto|legacy|executor>`, then `ARMD_RT_OWNER`, then config `arm.armd_owner`, then `auto`.
- Under `auto`, a qualified `rebot-dm` or `mock` follower starts on the executor (`effective_owner = "executor"`, `owner_reason = "auto_qualified"`). A profile with MIT joint tracking (`joint_tracking` with `strategy = "mit"`) and the unqualified followers (`so-arm101`, `so-arm100`, `rebot-rs`, `fashionstar`, `star-arm102`, `sim`) start on `legacy`; one startup log line gives the reason (`auto_fallback_mit_profile`, `auto_fallback_unqualified_follower`). `--owner executor` on such a profile is refused before the hardware is opened.
- If the executor cannot start under `auto`, armd falls back to the legacy owner (forced startup failure not tested on the real arm, item D2). The fallback and the failed-owner state are shown in the cockpit (see "Cockpit").
- `effective_owner` and `owner_reason` are in `GET /api/v1/observation`, `GET /api/v1/diagnostics` and `GET /api/v1/diagnostics/tick`. A never-gated owner status endpoint lists the routes that only the legacy owner serves; on the test device it lists `/api/v1/action/gripper_calibrate` and nothing else.
- To run the legacy owner: set `arm.armd_owner` to `legacy` in `/userdata/local/apps/rebot-teleop.config.json` (or start armd with `--owner legacy`) and restart armd. Scripts and tests that depend on legacy-only behaviour must pin it.
- Executor-owned features in this release: recording and datasets (including sequence recording), the gripper zero check, gravity compensation, vision and camera routes, depth measure, calibration sessions and sandbox routes.

### Gravity compensation, drag teach and MIT mode

- Gravity compensation (drag teach / hand guiding) runs on both owners. Damiao MIT mode switching is part of the executor and legacy mode-switch layer, with watchdog-continuity holds sent during the switch.
- After a Damiao communication timeout in MIT mode, armd recovers the motors with a two-round disable confirmation and holds the joints in between. This recovery confirmation is not verified on hardware (see "Known limitations").
- MIT joint tracking (`joint_tracking`, `strategy = "mit"`) stays off by default; it is not enabled on the real arm.
- Before the TIMEOUT register is written, armd collects rest-pose evidence on the disarmed arm (K = 3 samples). Damiao TIMEOUT readback on the test arm: J1–J6 all 2000 counts.

### Planning limits

- Profiles can carry per-joint planning-only limits (`planning_min`, `planning_max`; default = hard limits). The B601-DM profile sets J2 max 0.03 rad and J3 max 0.04 rad, because the + side from the folded home runs into a stop or the arm itself.
- They apply where joints are computed from a pose: Cartesian `move_to` and `move_relative` (straight and `path="joint"`), grasp and place sub-motions, the backlash approach, adaptive grasp and `measure_multiview` view poses, on both owners. A refused request returns `IK_UNREACHABLE` naming the joint and the limit.
- They do not apply to jog, `move_joints`, `move_to_pose`, sequences, servo/teleop/policy streams, replay, home/park, gravity compensation or the measured-position and fault checks; the hard limits apply there.
- The limits are listed in `GET /api/v1/safety/describe` (`limits[].planning_max_position`: J2 0.03, J3 0.04 on the test device) and in the observation schema.
- This replaces the 1.6.0-rc4 limitation where `move_to` (0.26, -0.08, 0.17) planned J3 = +0.078 rad and tripped `TORQUE_LIMIT` at 17.9 N·m.

### Power protection

- Supply-current estimate and governor (from motor feedback; there is no current sensor, so the values are estimates from datasheet motor parameters): `observation.power`, `GET /api/v1/diagnostics/power`, and a supply load meter in the cockpit.
- Static overload alarm (on by default): when the arm is stationary (joint speed below 0.02 rad/s) and the estimated supply current stays above the hard fraction of the rated current for 200 ms, `observation.power.severity` reports a warning; above `supply_trip_a` for 100 ms it reports a trip; it clears after 300 ms below the soft fraction. While a warning or trip is active (`static_restricted`), motion that lowers the load is allowed and motion that raises it is refused. The cockpit shows a banner.
- Synchronized feedback loss on all motors is detected, and the Damiao `CTRL_MODE` and `TIMEOUT` registers are verified before enable.
- Static-load admission (`power_budget.static_admission_fraction`, `STATIC_LOAD_LIMIT`) is available and off by default, because the estimate is not yet calibrated against a measured supply. To turn it on, set e.g. `"static_admission_fraction": 0.85` (5.53 A on the 6.5 A supply) in the profile.

### Workspace escape

- A 5 mm tolerance applies to outer workspace box edges (x, y, z) that are already violated, so a pose that sits just outside an edge can move back in. New intrusions, the floor, keep-out boxes, maximum reach and self-collision are unchanged. Behaviour is the same on both owners.
- Refusals of workspace limits use one formatter on both owners and in the cockpit (zh and en). The message gives labelled numbers (measured value, limit, previous and new depth, 3-decimal metres), says "obstacle" only for named keep-out boxes, and, when the start pose is already outside the workspace, suggests Park instead of Home. When the return path cannot be checked it says so and suggests Park plus a short jog. The code stays `WORKSPACE_VIOLATION`.
- Drag teach outside the workspace is allowed. The cockpit shows a banner with the edge, the depth and the way back. Leaving drag mode (user exit) is refused with `GC_EXIT_OUTSIDE_WORKSPACE` until the arm is back inside, and also when feedback is stale. Stop, E-stop, lease loss, watchdog, faults, owner handback and shutdown still end drag mode.

### Torque-off safety

- Torque is removed only at the rest pose: J1–J6 within 0.05 rad of `resting_pose`, with fresh feedback (K = 3 samples). Away from the rest pose the request is refused with `409 TORQUE_OFF_WOULD_DROP` (reasons include `stale_feedback` and `no_resting_pose`). There is no override parameter. Exceptions: a thermal trip and process shutdown or panic.
- A requested disable is dispatched over several ticks with measured-position holds kept on every joint until the disable is confirmed. Faults and E-stop latches are not cleared by a disable.
- A failed enable does not send an automatic rollback disable. It latches non-recoverable `ENABLE_FAILED` and keeps measured-position holds on every joint with fresh feedback.
- J7 gripper feedback is refreshed during enable and in every hold, from one round-robin scheduler for all joints; the rest gate checks J1–J6.
- Error codes: `TORQUE_OFF_WOULD_DROP`, `CAN_TX_STALLED` (CAN frames deferred beyond the bounded TX budget), `J7_FEEDBACK_STALE` (recoverable; J7 feedback older than 250 ms while powered), `DEADLINE_EXPIRED`.

### Vision

- The deb bundles the tuned models (deb size 11.1 MB to 23.1 MB): `yolo11n_seg_384x640` fuse segmentation (standalone forward 32.0 ms vs 71.2 ms for the stock 640x640 model; live `yolo.ms` p50 67.0–73.6 ms) and the tabletop ZipDepth (held-out table abs median 33.8 mm, 4.0 mm after a per-frame plane fit; on-device 8/8 poses with 4–19 mm residual). Both together use 54.29 MB of ION (8.6 MB free of the 62.9 MB carveout).
- Install and startup check the model files against the manifest and repair from the bundled copy when a file is missing or differs. `GET /api/v1/vision/mode` reports `model_config`; on the test device `model_config_optimal` was true.
- Segmentation detections carry `contour` (the exterior outline of the mask) in the WebSocket frames. Measured on the spare reCamera: CPU +0.04%, frame size 196 to 840 B mean.
- `POST /api/v1/vision/detector` (operator only, capability `vision.detector_switch`) switches the detector between `detect` and `segment`. `POST /api/v1/vision/models` (operator only) switches the model variant and the depth model.

### Cockpit

- Owner display: the status area shows the effective owner; a banner appears when the owner failed or fell back to legacy. The page schema and capabilities reload on reconnect and after an owner recovery.
- Controls that the current owner or state cannot run are marked unavailable before use, with the reason in the tooltip and the next step. Example: the full-stroke gripper calibration is disabled on the executor and the text says to set `arm.armd_owner` to `legacy` and restart armd; the drag-teach button reads "Enable the arm first" until the arm is enabled.
- Simple mode: recognition panel, detect/segment switch, object card with grasp and measure actions, grasp tasks. The cockpit takes the control lease only on Enable or Take control; controls show as unavailable without control, and the pill reads "Torque on · not in control".
- Grasp approach: the object card has a "Grasp approach" row with two choices, top-down (default) and adaptive (angled approaches allowed). The choice is stored in the browser and sent as `strategy` with the grasp request.
- Expert mode: one layout for desktop and tablet and a single-row layout for phone portrait. A Simple | Expert toggle in the header switches between the modes.
- Phone layout: the header fits at 360 px width without clipping the language button; the settings drawer is bounded in height, does not overlap the floating controls, and keeps the E-stop button reachable.
- Messages: a lease taken by another client shows "Control was taken over by another device, this page is now an observer" with a Take control button; a dropped connection shows that control was released and asks to re-take it after checking the arm; a park-off state explains why torque is off; workspace edge rejections give the part, distance and direction. All in zh and en. After a jog release the cockpit no longer shows "Stopped before motion began" (a servo frame refused because of the stop; other servo rejections are still shown).
- The cockpit serves `/favicon.ico`.

### Perception, recording and gripper

- Executor owner: vision subscribe and mode, camera snapshot, detector switch, depth measure, calibration sessions, device recording and datasets, sequence recording and sandbox routes work on the executor. These returned `503 EXECUTOR_ROUTING_PENDING` or were legacy-only in 1.6.0-rc4.
- Recording on the executor was tested with J1 motion and vision on (see "Test results").
- The gripper zero check runs on the executor.
- Opening-direction stall: a stall is counted for an opening gripper only when the torque is in the opening direction; opening or moving the gripper after a closed stall is covered by a regression test on both owners.
- The full-stroke gripper calibration (`gripper_calibrate`) stays legacy-only (see "Known limitations").
- `--cell` / config `cell`: a `rebot-cell/v1` topology is parsed and validated; `capabilities.cell` and `GET /api/v1/cell/status` report it.

### Policy and grasping

- Policy action chunking (`chunking` in `policy.json`: `chunk_size` 1..128, `replan_every`, `temporal_ensemble_k`) is supported on both the legacy and the executor owner (`docs/policy_runtime.md`).
- Adaptive grasp (phase B): `strategy: "adaptive"` on `/api/v1/action/grasp` with candidate selection and dry run (capabilities `grasp_select`, `grasp_select_exec`; 6-DOF profiles). `legacy` stays the default.
- Reducer (shadow): a torque-state reducer runs next to the active path and records divergences (`torque_shadow_enabled`, default on). It does not drive torque.

## Test results

Device: reBot B601-DM on reCamera (SG2002), `rebot-teleop 1.6.0-rc7` (`armd` md5 `922b46b8…`, installed 2026-11-27, md5 equal on the build host and the device). Both owners were run through the same 7 stages; 0 arm faults, 0 `0xD` motor errors, 0 `WATCHDOG_BUDGET_INSUFFICIENT` lines and 0 unexplained reducer divergences (496 comparisons on the executor, 208 on legacy) in the run logs. `can0`: 0 bus-off and 0 bus errors; the interface counters (error-warn 2, error-pass 2, 1 RX error, 1 overrun) were the same before and after the run. Host test suite for this build: 2,798 tests, 0 failures.

Release time = time of one 50 Hz control release; a skip is a release that missed its deadline. Owner column: `executor` was started by `auto` (`auto_qualified`), `legacy` with `--owner legacy`.

| Stage | Executor: release max (ms) / skips | Legacy: release max (ms) / skips |
|---|---|---|
| 1 startup, owner status, TIMEOUT readback (dry run) | 5.79 / 0 | 28.70 / 1 |
| 2 idle, torque off, 61 s | 4.60 / 0 | 4.97 / 0 |
| 3 enable, hold 30 s, torque off at rest, 3 cycles | 14.36 / 0 | 12.94 / 0 |
| 4 J1 motion, vision on, recording on (192.7 s / 196.1 s) | 14.59 / 0 | 12.92 / 0 |
| 5 detect, measure, grasp dry run | 4.58 / 0 | 3.59 / 0 |
| 6 safety describe, schema, planning limits | 2.28 / 0 | 2.30 / 0 |
| 7 soak with 5 fps vision, stopped by the harness at the first skip (676.9 s / 233.3 s) | 20.22 / 1 (33,949 releases) | 28.97 / 1 (11,768 releases) |

- Enable took 0.51–0.62 s and torque-off 0.30 s on the executor (stage 3); 0.45–0.50 s and 0.31–0.55 s on legacy.
- Recording (stage 4): an episode of 180.12 s with 9,007 frames on the executor, 186.80 s with 9,341 frames on legacy; none discarded.
- Legacy stage 1 (28.70 ms) came from the `mit_timeout` dry run; legacy stage 7 (28.97 ms) came from a configuration read lock held for 28.4 ms (`PublishedConfig::from_inner`).
- 15-minute soaks on the executor with J1 motion cycles: vision off, max release 14.18 ms, 0 skips, 45,483 releases; vision at 5 fps, max release 14.40 ms, 0 skips, 45,699 releases (window p99 median 6.82 ms vs 10.32 ms).
- CAN capture on the executor with J1 motion, vision subscription and recording on (185.3 s active window, 135,142 frames, 0 kernel drops, 0 CAN error frames): the largest gap between control frames on any axis was 24.79 ms against the 40 ms motor watchdog requirement. Hold-reply latency over all axes: p50 0.68 ms, p99 15.43 ms, max 28.10 ms.
- Cockpit, executor owner: 12 combinations (360x780, 390x844, 1024x768; zh and en; simple and expert), 12 of 12 passed (no horizontal overflow, no clipped header items, E-stop reachable with the drawer open, no console 404, no page errors); enable, park and release actions passed.
- On-site items passed on the executor: C3 (gripper zero check), C5 (grasp dry run with the arm torqued: returned a reachability verdict, `IK_BRANCH_FLIP`), C6 (planning limit: `move_to` (0.26, -0.08, 0.17) refused at planning, no motion), C8 (sequence of two poses run twice, 6.64 s and 7.62 s, no fault), D1 (2 warm restarts of rebot-teleop: 22.6 s and 22.3 s to ready; 1 device reboot: 48.8 s; executor, Idle, torque off after each), D3 (a second client got `controller_busy`; after the first client's connection was reset, the lease was released and the arm went to Idle with torque off within 5 s; cockpit wording for both cases checked in zh and en). C9 (full gripper stroke calibration disabled on the executor with the reason shown) passed. A7 (extreme feedback position): all joint limits keep the raw position above the -12.1 rad collision threshold, smallest margin 5.82 rad.
- D2 (forced executor startup failure) was not tested: no supported setting forces an executor startup failure on the real follower.
- D4 (stop while torqued at rest) failed on rc7: armd refused to exit and latched an E-stop, and the init script had already stopped vision and the web service (502). `fix/init-stop-companions` keeps all processes running when armd refuses to stop (covered by a host test, not re-run on the arm). The E-stop latch itself remains (see "Known limitations").

**rc7.2** (rc7 plus the five later fixes), real B601-DM, executor default, 2026-11-27:

- Host gates: nextest 2,805 of 2,805, `ws_replay` 0 diffs, alloc/lock gates 141 of 141. rc7.3 (adds only the grasp-approach row): nextest 2,805 of 2,805.
- Stamp regression on the real arm: 2,100 WebSocket servo frames, 300 interleaved ordinary commands and 84 REST reads, then `home()` twice; both accepted.
- 15-minute J1 ±0.15 rad soak with 5 fps vision: 45,363 releases, max 15.935 ms, p99 11.227 ms, 0 skips, 2 releases above 15 ms, 0 above 20 ms, 0 faults.
- Service stop and start at rest with torque off: clean.
- Cockpit Playwright 12 of 12; a jog release shows 0 error toasts.
- On-site, by the user (rc7.2 / rc7.3): top-down grasp succeeded; hand-guide (drag teach) worked; C6 and C3 passed again. Evidence for C6/C3 (`realarm-rc7-onsite-c6c3-*`): `move_to` (0.26, -0.08, 0.17) refused with `IK_UNREACHABLE` naming `joint_3` (0.0455 rad outside the planning limit 0.0400 rad), maximum joint change 0.0011 rad; gripper zero check passed (`stop_rad` 0.0698).

## Improvements

- `K92rebot-teleop stop` stops vision, the app and the AP guard only after armd has exited. When armd refuses to stop, everything keeps running: no lost video, no 502 (found in on-site test D4).
- Fixes `STAMP_REUSED` refusals of grasp, Home and other actions after cockpit jogging on the executor: RT-internal servo request IDs now have their own replay tracker. Reproduced by a test before the fix.
- `rebot-vision` restarts after it requests its own shutdown. BusyBox 1.33.0 `ash` spun in a blocking `wait` after the child exited, so the runner never restarted it (video stayed down until a manual restart). The runner now polls `/proc`.
- Stop and E-stop paths: per-connection WebSocket handling split into family modules, with a replay corpus of 35 sessions (0 diffs).
- Allocation and lock proofs on the control thread are part of the test suite (alloc gate, lock gate).
- Motor control modes by vendor (`control_mode` per joint in the profile, `docs/motor_modes.md`).

## Known limitations

- **Executor release budget.** A per-release hard cutoff and "hold frames first" budget ordering are not in this release (planned for 1.6.1). Hold continuity under load: the worst gap between control frames was 24.8 ms against the 40 ms motor watchdog requirement. To check a deployment, read `skipped_releases` in `GET /api/v1/diagnostics/tick` and look for `WATCHDOG_BUDGET_INSUFFICIENT` in the armd log; both were 0 in the stages listed under "Test results" except where stated.
- **MIT / drag-teach timeout recovery.** After a Damiao communication timeout in MIT mode, the recovery confirmation is not yet verified on hardware. If a fault appears, park or stop and clear it in the cockpit with the arm at rest.
- **Executor soak with 5 fps vision.** One release took 20.22 ms (1 skip in 33,949 releases, at 676.9 s). With vision off, the 15-minute soak had a max of 14.18 ms and 0 skips. With vision at 5 fps, the 15-minute soak had a max of 14.40 ms and 0 skips.
- **Legacy owner (fallback).** Configuration read-lock contention gave one 28.97 ms release in a soak. Use the executor for time-critical work.
- **Arm outside the workspace.** If drag mode ends through a safety exit while the arm is outside the workspace, the arm holds there. Example: `GRAVITY_COMP_JOINT_LIMIT` (0.05 rad inside a joint limit). From outside the workspace, Home and most moves are refused; press Park to return. A general return rule is planned for 1.6.1.
- **Service stop while torqued.** Stopping the service (`K92rebot-teleop stop` or SIGTERM) while the arm is torqued, also at rest, latches an E-stop and keeps holding; armd stays running. Park (torque off) before stopping the service. Planned fix: 1.6.2.
- **Refusal logging.** Command refusals are not yet written to the armd log (planned for 1.6.1).
- **Adaptive grasp approach.** It can refuse wide objects that a top-down grasp (`OPENING`) accepts, for example a banana close to the maximum opening, because it adds a fixed position-error margin. If that happens, switch the "Grasp approach" row to top-down. Unification is planned for 1.6.1.
- **Hand-guide entry sag.** On entering hand-guide (drag teach) the arm can sag briefly by about 1°. Measured on 2026-11-27: J2 22 mrad and J4 18 mrad during the position-to-gravity-compensation handover; J3 torque dipped from -6.6 to -2.3 N·m for one sample. It settles by itself. Fix planned for 1.6.1.
- **Cockpit page stops sending** (for example the tablet screen locks or the browser goes to the background): the connection times out, armd releases control and the watchdog parks the arm. Keep the screen on while operating.
- **Gripper full-stroke calibration** is legacy-only; on the executor the button is disabled and the cockpit shows why. To calibrate: set `arm.armd_owner` to `legacy`, restart armd, calibrate, then set it back to `auto`.
- **Static-load admission is off by default**, because the power estimate is not yet calibrated against a measured supply. The static overload alarm and the other power protections stay on.
- **Single-view `measure`** on the test unit returns `table_contradicts_fk` (table/arm-geometry mismatch) until the table and hand-eye calibration are redone. Redo them in the cockpit calibration before relying on `measure`.
- **Grasp on the test unit** can be refused as out of reach while the table and hand-eye calibration are off (single-view `measure` reports `table_contradicts_fk`). Redo the table and hand-eye calibration first.
- **USB link resets during some downward moves** (hardware, under investigation). Run programs on the device (`127.0.0.1`) rather than over the USB link.
- The reducer is shadow-only: torque decisions come from the active path.
- MIT joint tracking is not enabled for multi-joint use.
- Dual-arm and multi-arm support is not part of this release.
- Memory use on the device is not measured for this release.
