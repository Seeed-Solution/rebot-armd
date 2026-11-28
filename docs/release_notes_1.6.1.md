# rebot-armd 1.6.1 release notes

[中文](release_notes_1.6.1.zh-CN.md)

`rebot-armd` for the Seeed reCamera (SG2002) with the reBot B601-DM arm. Follows 1.6.0.

1.6.1 is the `v1.6.1` tag, built from `arch/1.6.1`. Unless a line says "measured on the real arm", results below are host or simulation results. Real-arm checks for this release are listed under "Not yet verified on the real arm".

## Assets

| File | What it is |
|---|---|
| `rebot-teleop_1.6.1_riscv64.deb` | The reCamera package (`armd`, `rebot-vision`, tuned vision models). Install with `opkg install` or from the reCamera console. |
| `armd-1.6.1-riscv64-musl` | The `armd` binary alone. |
| `rebot-vision-1.6.1-riscv64` | The vision binary alone. |
| `SHA256SUMS` | Checksums of the files above. |

## What's New

### Cockpit

- A dropped connection now shows a persistent "Connection lost · reconnecting" warning in the lease status line (and in the header badge on the phone expert layout). Retries run at 1, 2, 4, 8 s, then every 15 s, with a countdown and a "Reconnect now" button. The retry timer resets after the new socket delivers a valid observation.
- After reconnecting, the cockpit shows a release notice with the current arm state from fresh telemetry. It says "parked" only when the server reported the arm as not torqued. Control is not re-taken automatically: taking control again is an explicit action.
- Verified in the Playwright suite (prolonged outage, repeated failures, recovery, watchdog park; zero lease acquire frames before the explicit retake). Tablet suspend and resume in the target browser: real-arm / tablet verification pending (G7).

### Hotspot setting

- New setting `network.hotspot` (`on` | `off`, default `on`) in the reCamera console, rebot-teleop settings. With `off`, the service does not start the device's own access point or its guard process. If the rebot AP is already running when the service starts, it is left running and the log says that the setting takes effect after a reboot. It takes effect at service start, that is, after a reboot or a service restart; a running AP is not stopped at runtime. Firmware-owned APs are not touched.
- When `start` finds armd already running, the service logs that the hotspot setting applies after a reboot.
- Not verified on the device: SSH and cockpit access over USB (`192.168.42.1`) with Wi-Fi station down and the hotspot off, including USB DHCP (G4). Keep another way to reach the device until this is checked.

### Refusal logging

- Command, preset and servo refusal replies sent over WebSocket are written to the armd log. Per connection and per refusal code: the first occurrence is logged immediately, then at most once per 5 s, and the next line carries the number of suppressed refusals. `STOPPED` is logged at DEBUG, other codes at WARN. A line holds the connection id, reply type, code, status and the request id (when it is a valid number). Wire replies are unchanged. More than 64 distinct codes on one connection share one `OTHER` bucket.

### Grasp

- The adaptive strategy's `-top` candidate now runs through the same route as the legacy top-down grasp. Failures of the adaptive preflight for `-top` no longer refuse the grasp; they are reported as `selection.top_advisory {code, phase}` (omitted when there is none). A chord candidate is preferred over `-top` only when `-top`'s advisory check passed and the chord scores higher, or `-top` is deferred and the chord is checked. `-top` has no 1.5 s builder deadline.
- Jaw reserve is checked per side. Admission computes a signed interval of the object across the closing axis (an off-centre contour counts on the side it protrudes) and requires at least 3 mm reserve on each side after the hard-error margin. It applies at planning admission to the legacy top-down route, `-top` and the chord candidates, and when the adaptive path revalidates with the actual pose. A centred object gives the same result as the 1.6.0 scalar rule.
- Adaptive chord grasps re-check the jaw reserve with the actual wrist pose, and again with the pose used for the insertion move.
- Verified in simulation: plan parity sweep, dispatch parity with forced `-top` (same phase sequence), grasp suites 135/135 on the merged branch. Real-arm grasp qualification (wide and narrow objects, both strategies) is pending (G6, after table and hand-eye calibration E4).

### Workspace return rule

Applies on both owners.

- Floor, named keep-out boxes and self-collision have zero deepening allowance in every motion, including Park (the 1 mm Park tolerance no longer reaches these).
- Maximum-reach limits (end origin and TCP) get the same 5 mm temporary allowance as the outer workspace faces; each limit is tracked independently.
- A motion that starts with any limit already violated must end with every limit at or shallower than its start depth and at least one limit improved by more than 0.1 mm; otherwise it is refused with `WORKSPACE_VIOLATION` (no new error code). This covers jog, Home, `move_joints`, saved poses, joint-path and straight `move_to`, relative moves, and each motion in a sequence. For servo/teleop/policy streams, each step may not re-enter or add an intrusion, and the shallowest depth reached is kept across frames; stationary and braking steps are allowed.
- The evaluation starts from the measured pose; when the commanded start differs, the transition is checked too. Before a prepared motion is installed, the rule is checked again against the live measured start; a drift of more than 20 mrad on any arm joint returns `PLAN_STALE` (encoder noise is about 0.3 mrad on B601-DM).
- When Park's ramp-out would deepen a floor, keep-out or self-collision intrusion, the arm stops at the last checked target and holds with torque on; no fault is latched. Park fails with `WORKSPACE_VIOLATION`, and the cockpit names the blocking geometry: jog away from it, then Park again.
- A `WORKSPACE_VIOLATION` reply carries the blocking constraint, geometry and phase only when they belong to that request; otherwise it carries the code alone and the cockpit shows a generic hint.
- Verified in simulation: jog, Home, Park and `move_to` from the rest pose accepted on both owners; B601-DM and SO-ARM101 rest, home, park and inspect poses are inside the workspace; alloc and lock suites 147/147. Real-arm check pending (G10).

### Configuration read lock

- The legacy owner's published-configuration snapshot is built in three steps: capture under the supervisor lock (the driver capability through a non-blocking borrow), finalize outside the lock, and publish only if the generation is newer. In the host test the longest supervisor read hold is 20 µs. If the driver stays busy, the request retries for at most 50 ms, then returns `503 CONFIG_UNAVAILABLE`. A delayed capture cannot overwrite a newer snapshot or publish into the executor's epoch.
- This targets the 28.97 ms legacy release seen in the 1.6.0 soak (a 28.4 ms read hold in `PublishedConfig::from_inner`). The real arm runs the executor owner by default, so the legacy-owner hold was not measured on the real arm for this release.

### Attended test flag

- `armd --test-fail-executor-startup` forces the recoverable executor-to-legacy fallback, to test D2 from the 1.6.0 checklist. It is off by default, accepted only for a `rebot-dm` follower with the executor owner, validated before the hardware is opened, and logs an ERROR line while armed. The init script cannot arm it from a saved configuration value. Use it only with the user at the arm, and restart without the flag afterwards. Host tests check that ownership returns to legacy, ticks keep advancing and no torque-off command is sent. Real-arm hold without drop and the cockpit fallback banner: pending (G5).

## Behaviour changes that may surprise you

- **Wider pre-open.** The gripper pre-open opening is now at least the opening the jaw check needs: `max(previous pre-open, admission minimum opening)`. For a 40 mm object it rises from about 55 mm to about 66.5 mm, which leaves about 5.7 mm less clearance to neighbouring objects per side.
- **Stricter legacy admission.** A top-down grasp is refused with `OPENING` when either side has less than 3 mm reserve after the hard-error margin, also when the 1.6.0 path would have planned it. Until table and hand-eye calibration are redone (E4), a wide object such as the test banana can be refused by both strategies.
- **Adaptive no longer refuses what top-down accepts at planning.** In 1.6.0 adaptive could refuse a target that top-down planned; with `-top` on the legacy route it falls back to that route. The live width check during execution is unchanged from 1.6.0.
- **Moves that start outside the workspace must make net progress** (see "Workspace return rule"). A jog sideways that stays equally deep, or deeper, is refused with `WORKSPACE_VIOLATION`.
- **Park is refused if it would deepen** a floor, keep-out box or self-collision intrusion.
- **Standalone cockpit page removed from the deb.** The package no longer ships `/usr/share/rebot-teleop/index.html`. Open the cockpit at `http://<device>:8765`.
- **Hotspot off** takes effect after a reboot or service restart, not immediately.
- **Config reads can return `503 CONFIG_UNAVAILABLE`** on the legacy owner when the driver stays busy for more than 50 ms.
- **Numeric values in `rebot-teleop.config.json`** (ports and similar) are trimmed of surrounding whitespace before they are passed to armd.

## Internal changes

- Driver split with no behaviour change: `src/driver.rs` went from 30,709 to 2,054 lines. Legacy CAN-servo and bus-servo moves, the Feetech engine, poll and torque modules, and the Damiao emitter, commissioning, modes, registers, MIT, torque and RX/TX code moved into `driver/` modules behind facades; `driver::*` and `protocol::*` re-exports are kept. The common extractions (rest gate, feedback ages, stop checks) are not part of this release.
- Supervisor split with no behaviour change: `src/supervisor.rs` went from 24,077 to 6,769 lines; driver transitions, streams, actions and terminal capture were split; workspace evaluation moved to `supervisor/workspace.rs` and configuration publication to `supervisor/config/published.rs`. The RT release-path extractions and the executor split are not part of this release.
- The grasp module was split earlier in this line (planning, orchestrator, selection).
- Cockpit page source is split into files and assembled at build time (Rust assembler for armd, Node assembler for tests); the deb flavors build into separate cargo output directories.
- CI runs the Rust tests with `cargo nextest` and runs doctests as a separate step; test timing races (bus-loss test, perception route test) were fixed in the tests.
- Source-size limits (about 1,500 lines per file) are checked in CI with a ratchet allowlist; scanner tests for the RT and torque rules were updated to follow the moved code.
- Host gates on the release line (`4dfe5908`; the release commit adds a whitespace fix and the version bump): clippy 0 warnings, nextest 2,902 of 2,902, doctests 29/29, WebSocket replay 0 diffs, alloc/lock suites pass, riscv64 (with and without `xthead-ncc`) and wasm32 builds pass, source-size check passes, 7 cockpit browser tests pass.

## Measured on the real arm

B601-DM, reCamera SG2002, 1.6.1-rc1 (same code as the release except the version string), executor owner, vision running, no user at the arm:

| Run | Release time | Skipped deadlines | Control-thread lock attempts | MIT watchdog budget | Faults |
|---|---|---|---|---|---|
| J1 ±0.15 rad at speed scale 0.3, 4 cycles, with observation, WebSocket, blackbox and bulk HTTP load | max 14.42 ms, p99 10.28 ms | 0 | 0 | 38.8 ms (limit 45 ms) | 0 |
| 10 min idle soak (arm parked, torque off) with the same load | max 5.96 ms, p99 5.03 ms | 0 | 0 | 33.6 ms | 0 |

The 1.6.0 soak had seen a 20.22 ms release with vision running; these runs stay below 20 ms.

## Not yet verified on the real arm

These checks need the user at the arm. They were not run for 1.6.1 and are planned together with the 1.6.2 tests.

| Item | What is pending |
|---|---|
| G1, G2 | Hand-guide entry with synchronized CAN capture; J2 `KP_APR` trial. No change shipped for the entry sag. |
| G3 | Reproduce `GRAVITY_COMP_JOINT_LIMIT` at the far pose with per-tick logging. |
| G4 | Hotspot off: cold boot, then SSH and cockpit over USB. |
| G5 | `--test-fail-executor-startup`: hold without drop, fallback banner. |
| G6 | Grasp qualification after E4: wide and narrow objects, both strategies, 3 supervised trials each; pre-open 55 to about 66.5 mm effect on clearance. |
| G7 | Tablet idle, WebSocket timeout, reconnect warning, explicit retake. |
| G8 | Gripper pad geometry measurement (input for the 1.6.2 pad-engagement rule). |
| G9 | SO-ARM101 bench on the legacy owner with the 1.6.1 driver layout. |
| G10 | Workspace return rule: start outside the box or inside a keep-out box, jog back (accepted), jog further out (refused), Park from inside a keep-out. |
| Config lock | Maximum read hold on an idle legacy-owner run. |

## Known limitations

Carried over from 1.6.0 unless noted.

- **SO-ARM101 / Feetech on the executor** is not eligible in 1.6.1. `auto` keeps SO-ARM101 on `legacy`; `--owner executor` for it is refused. Deferred to 1.6.2.
- **Live grasp evidence.** The live width check during a legacy grasp is the 1.6.0 behaviour (it is skipped when no width was measured). A model that binds width, axis and centre to a fresh observation after the camera-down transition and the recentre is deferred to 1.6.2.
- **Pad engagement.** The pad engagement rule (about 8 mm) for both strategies is deferred to 1.6.2, after measurement G8.
- **Workspace return, parts not in 1.6.1.** Interval certification of a whole path, the cockpit "return path unverified" notice with a shorter-jog suggestion, and off-thread whole-plan certification are deferred to 1.6.2. Until then a return path that cannot be checked is refused with the 1.6.0 wording.
- **Cockpit "armd restarted" notice** in the reconnect message is deferred to 1.6.2 (the observation has no boot id).
- **Release-path and executor splits.** Moving the RT release-path code and splitting `executor.rs` are deferred to 1.6.2.
- **Executor release budget.** The per-release hard cutoff and "hold frames first" ordering are still not in this release (1.6.0 note).
- **Hand-guide entry sag** (about 1° for a moment) is not fixed. Diagnosis: a one-sample J3 torque gap at the POS_VEL to MIT switch; the J2 `KP_APR` trial (G2) comes first, the entry fix is planned for 1.6.2.
- **Service stop while torqued** still latches an E-stop (1.6.0 note). Park before stopping the service.
- **Park refused while still moving.** If Park is refused while the arm is moving and no braking step stays inside the workspace, the shared braking fallback latches `ServoNoBrake`; clear it in the cockpit.
- **Motor TIMEOUT after a power loss.** After the arm's motors lose power, their TIMEOUT registers read 0 and armd refuses to enable (`ENABLE_FAILED`, "TIMEOUT readback mismatch"). In the rc1 test this failed enable left J1–J6 torqued at rest with torque-off refused. Recovery with the arm at rest: disable the motors, restart armd, then rewrite TIMEOUT with `/api/v1/calibration/mit_timeout`. That rewrite checks the rest pose with 0.05 rad per joint while the profile's resting pose allows 0.08 rad, so a wrist roll between the two blocks it. Fixes are planned for 1.6.2.
- **Hotspot off** was not verified for USB-only access (G4).
- **Single-view `measure`** and some grasps on the test unit stay refused until table and hand-eye calibration are redone (E4).
- **USB link resets during some downward moves** (hardware, under investigation).
- Dual-arm and multi-arm support is not part of this release.

## Upgrade notes

- Install the deb with `opkg install` (same as 1.6.0). The owner default (`auto` selects the executor for `rebot-dm`) is unchanged.
- Open the cockpit at `:8765`; any bookmark or script that read `/usr/share/rebot-teleop/index.html` from the device must change.
- To turn the device hotspot off: set `network.hotspot` to `off` in the rebot-teleop settings and reboot. Make sure you can reach the device another way first (G4 is pending).
- Scripts that relied on a top-down grasp passing with less than 3 mm reserve per side, or on a pre-open near 55 mm for a 40 mm object, need a re-check on the arm (speed scale 0.3, user at the stop button).
- Programs that jog or run motions while the arm is outside the workspace now get `WORKSPACE_VIOLATION` unless the move makes net progress back; use Park (not from inside a keep-out box that it would deepen) or a jog towards the inside.
- Clients that read the grasp dry-run may see the new optional field `selection.top_advisory`.
- `--test-fail-executor-startup` is for attended tests only; do not put it in a service configuration.
