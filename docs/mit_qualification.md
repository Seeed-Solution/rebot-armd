# MIT tracking qualification (B601-DM)

MIT impedance tracking (joint tracking Phase 2, `src/mit.rs`) is disabled by default and is
refused at arm time until the joint is qualified on the device. This page lists what armd checks,
what the operator measures, and the record that ties the two together. The motor command timeout
of J4 has been measured on the arm (see [Current status](#current-status)); the comparison runs and
the gates have not been run.

## Current status

MIT tracking is off by default and is not enabled on the real arm.

**Startup control-loop owner (`auto` default) / 启动控制循环所有者（默认 `auto`）：**
`armd` resolves its startup owner with precedence `--owner <auto|legacy|executor>` > `ARMD_RT_OWNER` > config `arm.armd_owner` > `auto`. Under `auto`, qualified `rebot-dm` and `mock` profiles default to the synchronous RT executor (`effective_owner = "executor"`, `owner_reason = "auto_qualified"`), while MIT-configured profiles (`joint_tracking` with `strategy = "mit"`) and unqualified followers (`so-arm101`, `so-arm100`, `rebot-rs`, `fashionstar`, `star-arm102`, `sim`) resolve to `legacy` with one startup log line naming the reason (`auto_fallback_mit_profile` / `auto_fallback_unqualified_follower`). Requesting `executor` explicitly on an MIT-configured profile or unqualified follower is refused at startup before opening hardware. Inspect `effective_owner` and `owner_reason` via `GET /api/v1/observation`, `GET /api/v1/diagnostics`, or `GET /api/v1/diagnostics/tick`.

`armd` 按 `--owner <auto|legacy|executor>` > `ARMD_RT_OWNER` > 配置 `arm.armd_owner` > `auto` 的优先级解析启动所有者。在默认 `auto` 模式下，已认证的 `rebot-dm` 与 `mock` 默认运行同步实时执行器（`effective_owner = "executor"`、`owner_reason = "auto_qualified"`）；启用 MIT 的配置（`joint_tracking` 中含 `strategy = "mit"`）及未认证从手自动回退到 `legacy` 并输出一条带原因的启动日志（`auto_fallback_mit_profile` / `auto_fallback_unqualified_follower`）。在 MIT 配置或未认证从手上显式指定 `executor` 会在打开硬件前直接拒绝。可通过 `GET /api/v1/observation`、`GET /api/v1/diagnostics` 或 `GET /api/v1/diagnostics/tick` 查看 `effective_owner` 与 `owner_reason`。

Measured on J4 of a B601-DM (motor ID 4, sw_ver 942747703, 2026-11-22):

- The motor replies on MST_ID 0x14.
- TIMEOUT 2000 counts expired 96-98 ms after the last command, which matches 50 µs per count.
- 0xCC status polls do not refresh the timer.
- On expiry the motor reports state 0xD and disables itself (torque about 0). A clear-error command
  followed by a disable returns state 0.

Control tick on the device (rc6, joint moves with 20 Hz observation polling and vision running): the
worst tick was 42 ms. The arm gate needs control period + `worst_latency_ms` + 5 ms < 45 ms, that is
a worst latency below 20 ms at the 20 ms control period, so MIT is not enabled on the real arm.

### CAN TX timing and RT priority A/B (2026-11-23)

Measured on the SG2002 B601-DM (`can0` `mcp251xfd` at 1 Mbit/s, `pfifo_fast`, `tx_queue_len=1000`;
`spi0` and `irq/51-spi0.0` at `SCHED_FIFO` 50) under the same batch-1 load (joint moves, measure
loop, 20 Hz `GET /observation`):

| Build | Tick priority | `tx_us` p50 | `tx_us` p99 | `tx_us` max | Worst tick (lock wait) |
|---|---|---:|---:|---:|---|
| `dev6` (`feat/rt-integration 433200c`) | `SCHED_FIFO` 45 (default) | 3.75 ms | 14.7 ms | 19.2 ms | 44.6 ms (`ws.rs:716`) |
| `dev6`, `ARMD_RT_PRIORITY=50` | `SCHED_FIFO` 50 | 0.29 ms | 4.3 ms | 12.1 ms | 53.8 ms (`ws.rs:716` held 57.7 ms) |
| `dev7` (legacy tick, integ lineage) | `SCHED_FIFO` 50 | 0.33 ms | 0.66 ms | 0.97 ms | 37.2 ms (`ws.rs:1165` held 47.6 ms) |

- At `SCHED_FIFO` 45 (below `spi0` and `irq/51-spi0.0` at FIFO 50), each SocketCAN frame write wakes
  `spi0` and preempts the control thread (~26 involuntary context switches per tick, `lock_cpu_us`
  1.5–2.1 ms), charging SPI transfer wall time inside `tx_us`.
- Raising the control thread to `ARMD_RT_PRIORITY=50` (same FIFO priority as the SPI/CAN threads;
  FIFO does not preempt equal priority) reduced `tx_us` p50 from 3.75 ms to 0.29 ms and p99 from
  14.7 ms to 4.3 ms.
- Prior to the SG2002 priority layout below, `DEFAULT_PRIORITY` in `src/rt_core/sched.rs` stayed
  at `45` and `ARMD_RT_PRIORITY=50` was used for on-device qualification A/B runs.

### SG2002 RT priority layout (2026-11-24)

The vendor SG2002 kernel (`linux_5.10/kernel/sched/cvi_sched.h`) caps non-`cvitask` tasks at
priority `80` and pins `cvitask_isp_pre` / `cvitask_vpss_0` / `cvitask_vpss_1` at `90` and
`cvitask_tpu_wor` at `85`. On SG2002, `armd` defaults to `SCHED_FIFO 79`
(`DEFAULT_PRIORITY = 79` in `src/rt_core/sched.rs`, overridable via `ARMD_RT_PRIORITY`), and
`K92rebot-teleop` applies the complementary layout at startup and whenever `rebot-vision` restarts:

| Thread group | Policy / Priority | Notes |
|---|---:|---|
| Pinned `cvitask` (`cvitask_isp_pre`, `cvitask_vpss_0/1`, `cvitask_tpu_wor`) | `FIFO 90` / `RR 85` | Pinned by `cvi_sched.h`; untouched |
| `spi0` and SPI/CAN IRQ threads (`irq/*spi*`, `irq/*mcp251*`) | `SCHED_FIFO 80` | Raised by `K92rebot-teleop` (at vendor cap) |
| `armd-control` | `SCHED_FIFO 79` | Default in `armd` (`ARMD_RT_PRIORITY` overrides) |
| 4 unpinned `cvitask` (`cvitask_vc_wt2`, `cvitask_vc_bh2`, `cvitask_isp_bla`, `cvitask_isp_err`) | `70` (keep FIFO/RR) | Lowered from `90`–`95` by `K92rebot-teleop` |
| `rebot-vision` RT SDK threads (`ISP0_RUN`, `Thread_Venc0_Pr`, `Thread_Venc2_Pr`, `tz_event_handle`, `vi_dbg_handler`) | `SCHED_RR 70` | Lowered from `RR 80` by `K92rebot-teleop` |
| Non-SPI IRQ threads (`irq/*mmc*`, `irq/*cviusb*`, `irq/*cd-gpio*`) | `SCHED_FIFO 50` | Stock kernel priority; untouched |

Measured on the spare reCamera SG2002 (`ARMD_RT_OWNER=executor`, `--follower mock`, vision ON, all
loads, 90 s, run `iso4-j`, 2026-11-24):

| Layout | Wake lateness max | Wake `> 5 ms` | MIT watchdog budget (`< 45 ms`) | Vision |
|---|---:|---:|---:|---|
| Default (`armd-control` FIFO 45, stock `spi0`/vision/`cvitask`) | 14.4 ms | — | 52 ms (FAIL) | 15 fps |
| Target (`armd-control` FIFO 79, SPI FIFO 80, vision & unpinned `cvitask` 70) | 5.6 ms | 2× | 35.7 ms (< 45 ms) | 15 fps, 0 errors |
- Bounded per-tick CAN TX (`src/driver.rs`, `TxBudget`: default 16 attempts, 10 ms start-cutoff
  overridable via profile `tx_budget_ms <= period / 2`, 45 ms inclusive queued-frame expiry)
  removes the 3 × 100 µs `WouldBlock` retry sleeps, maps SocketCAN `ENOBUFS` to
  `TransportError::NoBufferSpace`, treats bounded deferral as carry-over on unfenced output for
  both legacy and RT executor owners (latching `CAN_TX_STALLED` only on 45 ms expiry or
  `tx_stall_ticks >= ceil(45 ms / period) + 1` consecutive deferred ticks), keeps MIT-fenced output
  as an explicit fail-closed exception (tripping the deadline fence immediately on deferral,
  partial batch, preflight, or flush failure and clearing cached/pending frames), and exposes
  per-frame wall and `CLOCK_THREAD_CPUTIME_ID` timing plus queue-pressure counters in
  `GET /api/v1/diagnostics/tick` (`tx.*`, `max_tx`, and `tx_*` fields on `max_tick` / `max_lock`).

### Measured B601-DM TX budget sizing (2026-11-24)

Run `qual-smoke-20261124T000133Z` (`1.6.0-dev11` / `df7487d`, SG2002 B601-DM, `can0` `mcp251xfd`
over SPI at 1 Mbit/s, `SCHED_FIFO` 45, ERROR-ACTIVE, `berr 0/0`, `bus-off 0`):

| Metric | p50 | p99 | max | Notes |
|---|---:|---:|---:|---|
| `tx_us` (TX phase) | 3.915 ms | 8.123 ms | 13.383 ms | 7-motor SPI CAN under `SCHED_FIFO` 45 |
| `tx.send_us` (1000-tick window) | 3.622 ms | 7.763 ms | 13.184 ms | `send_cpu_us`: p50 0.593 / p99 0.757 / max 1.259 ms |
| `tx.frame_max_us` | 1.058 ms | 6.104 ms | 11.554 ms | `max_tx` peak frame 11.381 ms; `max_tick` 7 frames in 3.114 ms (`frame_max_us` 0.994 ms) |
| Post-TX tail (`finish_us + obs_us + publish_us`) | 0.817 ms | 1.671 ms | — | `finish_us` 7 / 298 µs, `obs_us` 751 / 1,257 µs, `publish_us` 59 / 116 µs (p50 / p99) |

With the earlier 5 ms start-cutoff, healthy 7-motor + refresh batches tripped `TxDeferred` 6 s into
the load run during `home`, followed by 174 repeated hold write errors in Fault. Raising the default
frame-start cutoff (`DAMIAO_TX_MAX_DURATION` and `rebot-b601dm.json` `tx_budget_ms`) to **10 ms**
(keeping `DAMIAO_TX_MAX_FRAMES = 16` and `DAMIAO_TX_FRAME_MAX_AGE = 45 ms`, validated against
`tx_budget_ms <= period / 2`) sizes the start window for the common 7-target + 1-refresh case (~8 ms
with margin inside the 20 ms release period). On both the Legacy and RT Executor control owners, the
per-release TX cutoff is also bounded by the absolute release deadline minus the publication margin
(`tx_deadline = min(first_tx + tx_budget, release_deadline - margin)`, default
`DAMIAO_TX_RELEASE_PUBLISH_MARGIN = 2 ms`, covering the 1.671 ms p99 post-TX tail) so a late output
start stops starting new frames before the release ends and carries the remainder over. The cutoff
governs starting new `transmit()` calls (and any non-no-op `flush()`), not interrupting an in-flight
syscall: an isolated >10 ms SPI transmit tail (such as an 11.4 ms frame) finishes cleanly and defers
any remaining unstarted frames to the next tick instead of faulting.

## What armd enforces

| Check | Where | Result when it fails |
|---|---|---|
| A joint uses `"strategy": "mit"` with `hold_kp` / `hold_kd`, positive `kp`, `kd`, `effort_limit_nm`; gains at or below the profile's `gravity_comp.hold_kp` / `hold_kd`; `coulomb_nm`, `viscous_nm_s_rad · max_velocity`, `inertia_kg_m2 · max_acceleration` each within `effort_limit_nm` | profile load | enabled block rejected; a disabled block is ignored as in Phase 1 |
| `calibration.json` `mit_qualification` holds the joint, bound to profile, joint name, joint zero, motor ID, reply ID, ESC_ID, MST_ID, sw_ver, PMAX / VMAX / TMAX, TIMEOUT; not invalidated by a zero; `timeout_unit_verified` true; `measured_expiry_ms` > 0 and `tested_response` given; control period + `worst_latency_ms` + 5 ms < 45 ms | every arm and rearm, against a fresh register readback | the arm does not arm (`MIT_UNQUALIFIED`), no enable frame is sent |
| TIMEOUT readback nonzero and equal to the record | same | same |
| Profile gains and effort limit at or below the record's `max_kp`, `max_kd`, `effort_ceiling_nm` (and TMAX) | same | same |
| Handover only from fresh (< 45 ms), stationary, confirmed position control; mode ACK, hold transmit, flush and feedback newer than the ACK | first tick at rest after arming | MIT stays inactive, motions are refused; an unconfirmed rollback latches a fault and blocks clearance |
| Feedback age or gap between dynamic batches ≥ 45 ms, RX backlog, partial or failed batch, or MIT-fenced TX deferral (`TxRefusal::Deferred` / `TxBudgetExhausted`) | every tick (supervisor) and before every transmit (driver, `src/driver.rs`) | **Explicit exception to carry-over deferral:** MIT-fenced output trips the deadline fence immediately on deferral instead of carrying frames over across ticks; group cancelled, BUS_LOSS latched wherever the loss is detected (also over another fault), no further frame; read-only status polls only after the operator's fault clearance and 2 × `measured_expiry_ms` (at least 250 ms); reconnect needs a rearm (fresh readback, base modes restored); ages or gaps from 36 ms on are counted and logged as near misses |

**MIT-fenced deferral exception (by design) / MIT 围栏输出延后例外（按设计）：**
Unfenced output (`POS_VEL`, `POS_FORCE`, unfenced gravity-compensation hold/targets) carries
deferred CAN frames across ticks and latches `CAN_TX_STALLED` only on 45 ms frame expiry or
`tx_stall_ticks >= ceil(45 ms / period) + 1` consecutive deferred ticks (e.g. 4 at 50 Hz, 2 at
10 Hz). MIT-fenced output (`src/driver.rs`, `MitFenceTripReason::TxBudgetExhausted`) is the
explicit exception: any TX budget exhaustion or backpressure deferral trips the MIT deadline fence
immediately and clears cached/pending frames rather than carrying impedance targets across ticks,
so a deferred frame cannot outlive the motor's command-timeout fence.

非围栏输出（`POS_VEL`、`POS_FORCE`、无围栏重力补偿保持与目标）在发生有界发送延后时跨周期顺延，仅在帧龄达到 45 ms 过期或连续延后达到 `tx_stall_ticks >= ceil(45 ms / period) + 1` 个控制周期（例如 50 Hz 为 4、10 Hz 为 2）时锁存 `CAN_TX_STALLED`。带 MIT 截止围栏的输出（`src/driver.rs` 中的 `MitFenceTripReason::TxBudgetExhausted`）是按设计的明确例外：任意发送预算耗尽或背压延后都会立即触发 MIT 截止围栏并清空缓存与待发帧，不跨周期顺延，防止阻抗控制目标越过电机指令超时围栏。

TIMEOUT is RAM only. A power cycle resets it to 0 and the next arm refuses until it is
commissioned again.

## Recovery after a motor command timeout

When MIT tracking is lost and a motor's own TIMEOUT then expires, that motor disables itself and
reports error 0xD (CAN communication loss) while the other motors stay enabled.
`/api/v1/safety/clear_fault` (operator session) first reads fresh feedback from every motor and
decides from it (stale feedback: refused, nothing sent). Inside the quiet period after the loss
(at least 250 ms, 2 × `measured_expiry_ms`) it sends no frame at all and is refused with `409
MIT_QUIET_PERIOD` and the remaining milliseconds. In the expiry state it needs the
explicit body `{"mit_recovery": "torque_off_all"}` (without it: `409
MIT_RECOVERY_CONFIRM_REQUIRED`; the cockpit shows the warning and sends the confirmation only on
a second tap while the warning is shown, for the same fault). With it armd recovers under control: it sends the Damiao
clear-error command (`FF FF FF FF FF FF FF FB` on the motor's CAN ID) to every motor in an error
state, disables every motor and requires fresh state-0 feedback from each, and only then clears to
Idle. **Torque is then off on every joint**, like a park torque-off: the arm drops onto whatever
supports it. No MIT frame and no POS_VEL target is sent. A motor that does not confirm keeps the
fault latched and is named in the reply. The MIT group is reset; the qualification record is kept
and the next arm reads the TIMEOUT again.

Before an expiry test (step 1 of the device qualification), put the arm at a supported or rest
pose: the recovery turns torque off on all joints.

## Commissioning the motor command timeout

Operator session, torque off, load mechanically supported, operator at the power cutoff.

1. Read only: WS `/api/v1/calibration/mit_timeout` `{"joints": [4], "dry_run": true}`. Record the
   readback (IDs, sw_ver, CTRL_MODE, ranges, original TIMEOUT; 0 on the B601-DM on 2026-11-22).
2. Write: `{"joints": [4], "timeout_counts": 2000}`. Before writing, the commissioning precheck
   confirms torque is off, the arm is stationary, and J1–J6 hold K = 3 causal fresh rest-pose
   samples (actively polling read-only `0xCC` status frames on J1–J6 while disarmed if the idle
   1-slot refresh scheduler has not built a confirmed window, failing closed with zero writes on
   any off-rest or silent axis). armd writes RID 9 with the RAM opcode 0x55 (never 0xAA), checks
   the acknowledgement value and reply ID, and confirms with a separate 0x33 read.
3. The count is configurable (1..20000). 2000 counts = 100 ms with the documented unit of
   50 µs per count; on J4 (sw_ver 942747703) the timeout expired 96-98 ms after the last command
   (see [Current status](#current-status)). Other joints and firmware versions are not measured.
4. **Scope (J1–J6 only; J7 excluded):** `/api/v1/calibration/mit_timeout` accepts arm joints
   J1–J6 only and refuses J7 (the gripper). J7 runs in position-force mode (`CTRL_MODE = 4`,
   hardware current-capped) or legacy `POS_VEL` (`CTRL_MODE = 2`) and never switches to MIT
   impedance mode. Its safety contracts (`PosForceBootFallback::NoOutputUntilFresh` and MIT-fence
   isolation in `docs/gripper_pos_force.md`) rely on J7 keeping `TIMEOUT = 0` so a held workpiece
   is not dropped across a host restart or bus pause (on B601-DM firmware `sw_ver 942747703`,
   `0xCC` polls do not refresh the Damiao timeout and expiry disables the motor with error `0xD`);
   `RID_TIMEOUT` expiry in `CTRL_MODE = 4` is also uncharacterized on B601-DM firmware.

## Device qualification (one joint at a time)

Workspace clear, load supported, operator at the power cutoff. J4 alone first, then J1, then each
later joint individually. All register writes are RAM only.

1. **Timeout response.** On an isolated fixture, observe TX-only loss (CAN TX cut), RX-only loss
   and host loss (armd killed). Record for each: time from the last command to the motor response,
   the response (`disabled`, `energized_hold`, ...), torque, motion and restart behaviour, and which
   frames refresh the timer (MIT frame, POS_VEL frame, 0xCC status poll, register read). The
   measured expiry decides whether 50 µs per count is right (`timeout_unit_verified`).
2. **Load envelope.** Repeat the timeout test at the heaviest payload and the posture with the
   largest gravity load of the joint. A supported bench test alone does not show an acceptable
   unsupported drop; production needs a safe timeout response across the whole envelope.
3. **Comparison runs.** Identical bidirectional excursions with POS_VEL, `position_lead` and MIT at
   0.03, 0.06 and 0.10 rad/s, then a hold. Record nominal and wire output, measured state and
   feedback ages, clipping, current and temperature, and the worst-case loop latency
   (`worst_latency_ms`: scheduling, RX queue and TX to the wire; SocketCAN `flush` does not bound
   wire time, so measure it on the bus).
4. **Gates** (design section 4, the same metrics as `tests/test_mit_plant.rs`):
   - hold: within 2.0-4.5 s after each move, the maximum |Δq| over sample pairs at most 1 s apart
     ≤ 1.0 mrad for every creep3 pose and both repetitions; incomplete telemetry is a failure;
   - tracking: moving RMS nominal-vs-measured ≤ 50 % of the matched POS_VEL run;
   - endpoint: |error| ≤ 2 mrad at +1 s.
5. **Abort** on unexpected displacement, growing oscillation, clipping, a guard trip, a timing
   miss, an uncertain mode, or current / temperature beyond the limits. A failed gate leaves the
   joint disabled.

## Storing the record

WS `/api/v1/calibration/mit_qualification` `{"record": {...}}` (operator, torque off). armd reads
the motors again and stores the record only when it matches them; the file is written atomically,
then published. Fields per joint: `profile_id`, `joint`, `joint_name`, `zero_fingerprint`
(`zero:<hex bits of the joint zero offset>`), `motor_id`, `reply_id`, `esc_id`, `mst_id`, `sw_ver`,
`pmax`, `vmax`, `tmax`, `timeout_raw`, `timeout_unit_us`, `timeout_unit_verified`, `tested_response`,
`measured_expiry_ms`, `worst_latency_ms` (measured worst-case scheduling + RX queue + TX latency of
the control loop under load; step 3 records it), `load_envelope`, `max_kp`, `max_kd`,
`effort_ceiling_nm`, `date`, `trace_id`. A malformed or invalid record is ignored and flagged at load
(the rest of `calibration.json` is used) and refuses MIT. A zero of a qualified joint writes the
invalidation to disk before the zero; if that write fails, the zero is refused.
A motor or software zero of the joint marks the record invalidated; requalify after it.

## What the plant model says (not a measurement)

`tests/test_mit_plant.rs` models one wrist joint (gear 10, POS_VEL lag Gr/54, PI velocity loop,
14-bit encoder, stiction, Coulomb and viscous friction) and compares POS_VEL with MIT PD at the
J5 ceilings (kp 10, kd 0.3). Results depend on the assumed velocity-loop bandwidth and friction:

| Assumption | POS_VEL hold cycle | MIT hold | MIT RMS / POS_VEL | MIT endpoint |
|---|---|---|---|---|
| velocity loop 12 Hz, Coulomb 0.015 / stiction 0.045 N·m (development plant) | 1.91-2.29 mrad on all 14 trials | - | - | - |
| same loop, held out: Coulomb 0.012 / 0.016 N·m, stiction 2.5 / 3 × | ≥ 1.14 mrad | 0.00 mrad | ≤ 0.28 | ≤ 1.77 mrad |
| velocity loop 40 Hz, Coulomb 0.08 / stiction 0.20 N·m | 1.91 mrad | 0.00 mrad | 0.98 (fails) | 8.03 mrad (fails) |

The POS_VEL lag of the model is 0.684 s (gear 40) and 0.194 s (gear 10) against Gr/54 =
0.741 / 0.185 s; the creep2 fits were 0.68 (J1) and 0.17 (J4). A ±0.02 N·m gravity-model error
moves the held-out MIT endpoint from 1.17 mrad to 0.74 / 3.08 mrad.

Without an integrator the MIT endpoint error is bounded by (static friction + gravity-model
error) / kp: at kp 10, 0.02 N·m of either is 2 mrad. The device runs in step 3 decide whether the
endpoint gate holds; the model does not.
