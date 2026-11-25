# SG2002 Real-Time Qualification Harness (`scripts/rt_qual/`)

Harness for the Stage 3 §6 qualification gate on the SG2002 reCamera arm (`armd` 1.6 RT release).

## Topology and Components

| Script | Runs on | Dependencies | Output |
|---|---|---|---|
| `devstat.sh` | reCamera (SG2002) | busybox `sh`, `/proc`, `ip` | `/userdata/rt-qual/<run-id>/devstat.txt` |
| `load.py` | Office Raspberry Pi | Python 3.11+ stdlib + `rebot` SDK (`rebot-companion`) | `load.jsonl` |
| `analyze.py` | Spark workstation | Python 3.11+ stdlib only | Markdown qualification report |

## Gate Criteria (Stage 3 §6)

1. **Release latency**: every release `< 20 ms`, gated on full control-thread wall duration `release_us` (from wake right after the lateness sample through control, TX, usage collection, publication, dispatcher wake, sub-release wakes, cleanup, and stats recording/publication up to the next periodic sleep), the process-lifetime monotonic counter delta `releases_over_20ms == 0` (alongside `releases_over_10ms`, `releases_over_15ms`, and `release_us_max_ever`), attributable window peaks, and live `in_flight_release_us`. `tick_us` is retained for compatibility.
2. **Deadlines**: zero skipped deadlines (`skipped_releases` delta `== 0`, cumulative for both legacy `run_periodic` and executor owners; every deadline crossed by an overrunning release is committed immediately at release completion and on panic exit).
3. **Lock gate**: zero `SupervisorLock` attempts from `armd-control` (proxy in HTTP diagnostics: `wait_us.max == 0`, `waited_for == null`, `reader_hold_us.max == 0`, `writer_hold_us.max == 0`; authoritative proof via B1 `cargo test`). Legacy `tick_stats` publication is lock-free (`LockFreeTickStats` seqlock ring, `stats_wait_us == 0`).
4. **Allocation gate**: zero allocations/frees in release body (requires authoritative run-scoped `control_thread_alloc_ops` delta `== 0` under `ARMD_ALLOC_TIMING=1` for `PASS`; peak-sample breakdown `lock_alloc_calls` alone is treated as a proxy with an explicit `INCOMPLETE` gap, and absent/malformed breakdowns never default to `0`).
5. **MIT watchdog budget**: `20 ms + worst additional scheduling/RX/TX latency + 5 ms < 45 ms` (uses the maximum of windowed `wake_lateness_us.max`, any same-instance in-run `wake_late_max_ever` increase, and active `since_marker.wake_late_max`; marks `INCOMPLETE` if `diag_dropped > 0` or if a pre-run `wake_late_max_ever` high-water mark could mask an in-run wake peak across a `> 1000`-tick sample gap without an active `since_marker`).
6. **Zero faults**: no arm fault or E-stop during the run (`load.py` halts on fault and never clears faults).
7. **Cold-start verification**: repeat across cold starts and verify single-core `PREEMPT` / `HZ 250`, thread priorities (`armd-control` `SCHED_FIFO 79`, `spi0` / SPI-CAN IRQ threads `SCHED_FIFO 80`, unpinned `cvitask_*` and `rebot-vision` RT threads at `70`, pinned `cvitask_*` untouched at `85`/`90`), and `mlockall` (`memory_locked: true`, `VmLck > 0`).

## SG2002 RT Priority Layout

The vendor SG2002 kernel (`linux_5.10/kernel/sched/cvi_sched.h`) caps non-`cvitask` tasks at priority `80` and pins `cvitask_isp_pre`, `cvitask_vpss_0/1` (`90`) and `cvitask_tpu_wor` (`85`). `armd` defaults to `SCHED_FIFO 79` (`ARMD_RT_PRIORITY` overrides), and `/etc/init.d/K92rebot-teleop` applies the remaining layout on SG2002 at boot and on `rebot-vision` restart:

- `spi0` and SPI/CAN IRQ threads (`irq/*spi*`, `irq/*mcp251*`): `SCHED_FIFO 80`
- `armd-control`: `SCHED_FIFO 79`
- 4 unpinned `cvitask` threads (`cvitask_vc_wt2`, `cvitask_vc_bh2`, `cvitask_isp_bla`, `cvitask_isp_err`): `70`
- `rebot-vision` RT SDK threads (`ISP0_RUN`, `Thread_Venc0_Pr`, `Thread_Venc2_Pr`, `tz_event_handle`, `vi_dbg_handler`): `SCHED_RR 70`
- Pinned `cvitask` threads (`85`/`90`) and non-SPI IRQ threads (`50`): untouched

Measured on the spare reCamera SG2002 (2026-11-24, `ARMD_RT_OWNER=executor`, vision ON, all loads, 90 s, run `iso4-j`):

- Default layout (`armd-control` FIFO 45): wake lateness max `14.4 ms`; MIT budget fails at `52 ms`.
- Target layout (`armd-control` FIFO 79, SPI FIFO 80, vision & unpinned `cvitask` 70): wake lateness max `5.6 ms` (`>5 ms` 2×), MIT budget `35.7 ms < 45 ms`, vision `15 fps` with 0 errors.

## Baseline Marker (`POST /api/v1/diagnostics/tick/mark`)

Monotonic counters (`releases_started`, `releases_total`, `release_us_max_ever`, `releases_over_10ms`, `releases_over_15ms`, `releases_over_20ms`, `wake_late_max_ever`, `skipped_releases`, `diag_dropped`, `wake_lateness_over_{1,5,10}ms`) are written exclusively by the control thread and never reset. To capture per-run peak high-water marks after an earlier boot spike without resetting process-lifetime counters, call:

```sh
POST /api/v1/diagnostics/tick/mark
```

- **Auth / gating**: protected by `require_operator` and `same_origin` (rejects missing/invalid operator credentials when `OperatorAuth` is enforced, rejects cross-origin POSTs, and permits local same-origin calls when `OperatorAuth` is explicitly disabled). Allowed in both `legacy` and `executor` owner modes; returns `503` if no control loop is actively running or during ownership transfer.
- **Coherency**: returns `{"marker_id": N, "status": "pending"}`. On completion of the next release (after committing all of that release's duration, thresholds, and overrun skips), the control thread captures all cumulative baselines and zeroes `since_marker` peak trackers in a single seqlock transaction; subsequent `GET /api/v1/diagnostics/tick` responses expose `since_marker` with `"status": "active"` and coherent deltas plus `release_us_max` / `wake_late_max` over the post-marker interval. If the control loop exits before acknowledging a pending request, `GET` reports terminal `"status": "inactive"`.

## `INVALID` and `DIAGNOSTIC ONLY` Verdict Rules (`analyze.py`)

`analyze.py` applies strict verdict precedence:
- Legacy `tickload.json` input is **always** reported as **`DIAGNOSTIC ONLY`** (never `PASS`), while retaining proxy breakdown tables.
- For native `load.jsonl` captures: **process continuity invalidity** (`instance_id`, `boot_id`, or `process_start_unix_ns` changed mid-run -> **`INVALID`**, retaining any observed fault/breach `FAIL` rows in the gate table) > **`FAIL`** (verified in-run gate breach, including `releases_over_20ms` delta `> 0`, in-run `release_us_max_ever` / attributable window `release_us.max >= 20 ms`, live `in_flight_release_us >= 20 ms`, or `stop_reason == "fault"` / faulted `safety_state`) > **`INVALID`** (untrustworthy, interrupted, malformed, duplicate-key/nonstandard JSON, unverified terminal safety state, or incomplete capture evidence) > **`INCOMPLETE`** (valid run with remaining independent telemetry gaps such as `diag_dropped > 0`, missing `enqueue_to_wire_us`, missing `control_thread_lock_attempts`, proxy-only allocation evidence, or rollover-masked / unscoped MIT wake peaks whose conservative upper bound reaches `45 ms`) > **`PASS`**.

Missing or malformed fields **never default to `0`**. `load.py` installs a lock-free async-signal-safe `SIGINT`/`SIGTERM` handler, captures a synchronous `baseline` diagnostics sample before spawning any load workers (even with `--baseline-s=0`), verifies requested loads actually executed, stops background workers and revokes sampler writes after load ends, verifies pre-idle safety observation, persists an explicit `idle_tail_start` boundary sample, waits **`>= max(1.0 s, 2 * period * 10)`** of idle releases (`>= 10` monotonic `releases_total` completions), verifies post-idle terminal safety observation (`terminal_safety_verified == true`), settles the terminal `final` sample until `releases_total == releases_started` AND `in_flight_release_us == 0` (or an in-flight stall `>= 20 ms` is observed), and writes a terminal `summary` record with `stop_reason` strictly in `{"completed", "interrupted", "fault", "error"}`. `analyze.py` reports **`INVALID`** whenever any of the following hold:

1. **JSONL grammar or capture-boundary violation**: malformed JSON line, duplicate object keys at any depth, nonstandard `NaN`/`Infinity` constants, non-object JSON record, unknown record `type`, missing/duplicate/misplaced leading `meta` record, missing/duplicate/non-trailing `summary` record, missing or inconsistent `run_id` across records, `summary.stop_reason != "completed"` (e.g. `"interrupted"`, `"error"`, missing, or unknown), first sample phase `!= "baseline"`, missing/duplicate `idle_tail_start` boundary or insufficient idle-tail duration (`< max(1.0 s, 2 * period * 10)`) / `releases_total` progress (`< 10`), missing `meta.enabled_loads` or requested load with zero activity, unverified terminal safety state (`terminal_safety_verified != true` or missing `safety_state`), duplicate or missing trailing `phase == "final"` sample, fewer than 2 valid tick samples, zero releases completed, or any failed poll (`tick_error`, `poll_failed`, or null/non-object `tick`).
2. **Unsealed terminal release**: final sample leaves `releases_started > releases_total` OR `in_flight_release_us > 0` (`>= 20 ms` escalates to `FAIL`).
3. **Process / boot identity change or counter reset**: `boot_id`, `process_start_unix_ns`, or `instance_id` changed across samples (or is missing/placeholder), `owner_mode` changed mid-run, `loop_active == false`, or any monotonic counter decreased between adjacent samples. (Release builds use `panic = "abort"`, so a control-thread panic surfaces as a poll failure and/or changed process boot identity on restart.)
4. **Window rollover gap without monotonic counters**: the gap between adjacent samples exceeds `TICK_WINDOW` (`1000` ticks, via `since_boot_ticks`, `releases_total`, or elapsed wall time / nominal period) while required monotonic counters are missing.
5. **Missing, malformed, or contradictory required fields**: any sample lacks or has a malformed value for `instance_id`, non-placeholder `boot_id`, positive integer `process_start_unix_ns`, recognized `owner_mode` (`"legacy"` or `"executor"`), boolean `loop_active`, boolean `stats_snapshot_lapped`, `stats_writer_drops`, `in_flight_release_us`, `releases_started`, `releases_total`, `release_us` (`p50`/`p99`/`max`), `release_us_max_ever`, `releases_over_10ms`, `releases_over_15ms`, `releases_over_20ms`, `wake_late_max_ever`, `skipped_releases`, `since_boot_ticks`, `diag_dropped`, `wake_lateness_us` (`p50`/`p99`/`max`), `tick_us` (`p50`/`p99`/`max`), `period_us` (`p50`/`p99`/`max`), `lock_us.max`, `wait_us.max`, `reader_hold_us.max`, or `writer_hold_us.max`; or contradictory telemetry (such as `release_us >= 20 ms` while `releases_over_20ms == 0`).
6. **Lock-free snapshot coverage loss**: any sample reports `stats_snapshot_lapped == true` or `stats_writer_drops > 0`.

## Running a Qualification Pass

Pick a shared `RUN_ID` across Pi and reCamera (for example `RUN_ID=qual-$(date -u +%Y%m%dT%H%M%SZ)`). Keep Pi and reCamera clocks synchronized within ~1 s (NTP/chrony) so interval timestamps align.

### 1. Start `devstat.sh` on the reCamera

Copy `scripts/rt_qual/devstat.sh` to `/userdata/rt-qual/devstat.sh` and start it just before `load.py` (set `--duration` ~30 s longer than `load.py`):

```sh
RUN_ID=qual-20261124T080000Z
sh /userdata/rt-qual/devstat.sh \
  --run-id "$RUN_ID" \
  --out-dir /userdata/rt-qual \
  --interval 5 \
  --duration 210 \
  --max-bytes 16777216
```

`devstat.sh` writes to `/userdata/rt-qual/$RUN_ID/devstat.txt` and stops automatically at `--duration`, `--max-bytes`, or `SIGINT`/`SIGTERM`.

### 2. Run `load.py` on the office Pi

Export operator credentials if operator auth is enforced on the reCamera (never hardcode PINs/tokens in scripts):

```sh
RUN_ID=qual-20261124T080000Z
REBOT_HOST=192.168.42.1 \
REBOT_OPERATOR_PIN=<pin> \
REBOT_ALLOW_INSECURE_OPERATOR_AUTH=1 \
python3 scripts/rt_qual/load.py \
  --host 192.168.42.1 \
  --port 8765 \
  --run-id "$RUN_ID" \
  --duration 180 \
  --cycles 6 \
  --speed-scale 0.3 \
  --sample-interval 5 \
  --output "./rt-qual-${RUN_ID}/load.jsonl"
```

Recommended run lengths:
- **Smoke check**: `--duration 45 --cycles 2` (`devstat.sh --duration 60`).
- **Full qualification pass**: `--duration 180 --cycles 6` (repeat across at least 3 cold starts).
- **Isolation matrix (Stage 3 §4)**: toggle individual loads with `--no-motion`, `--no-obs-poll`, `--no-ws-stream`, `--no-measure`, `--no-record`, `--no-blackbox-log`, `--no-bulk-http` to isolate SD/eMMC vs USB gadget vs WS/HTTP preemption.

Safety defaults in `load.py`:
- `--speed-scale` is rejected if `> 0.3` or `<= 0`.
- Motion and recording run inside `with Arm(...) as arm:`.
- Any fault or `EStop`/`Fault` safety state stops all workers immediately without calling `clear_fault`, captures a 30 s blackbox tail, and exits with code `2`.

### 3. Pull artifacts to Spark and run `analyze.py`

```sh
python3 scripts/rt_qual/analyze.py \
  --load "./rt-qual-${RUN_ID}/load.jsonl" \
  --devstat "./rt-qual-${RUN_ID}/devstat.txt" \
  --output "./rt-qual-${RUN_ID}/report.md"
```

`analyze.py` also accepts legacy `tickload.json` directly (without `--devstat`):

```sh
python3 scripts/rt_qual/analyze.py --load /path/to/tickload.json
```

### Offline Validation Against Mock `armd`

```sh
cargo build --bin armd
./target/debug/armd --follower mock --port 18877 &
python3 scripts/rt_qual/load.py \
  --host 127.0.0.1 --port 18877 \
  --duration 30 --cycles 2 --motion-mode joint-j1 \
  --bulk-path / --output /tmp/mock_load.jsonl
python3 scripts/rt_qual/analyze.py --load /tmp/mock_load.jsonl
```

## Output Formats

### `load.jsonl`
Line-delimited JSON with three record types:
- `type="meta"`: run ID, target URL, rates, enabled load flags, motion config, `armd_version`/boot info from `GET /api/v1/diagnostics/boots`, and notes.
- `type="sample"`: sampled every `--sample-interval` seconds (plus final sample) containing `ts_unix`, `ts_iso`, `elapsed_s`, `phase` (`baseline`, `motion_setup`, `motion_record`, `full_load`, `cooldown`, `final`), raw `tick` object from `GET /api/v1/diagnostics/tick`, per-interval `rx_age_ms` (`p50`/`p99`/`max` of `feedback_age_ms` observed via REST/WS), load worker `counters`, `fault`, and `safety_state`.
- `type="summary"`: elapsed duration, `stop_reason`, final `fault`/`safety_state`, final `counters`, and `blackbox_tail_on_fault` (if faulted).

### `devstat.txt`
Section-delimited text:
- `=== META ===`: run ID, interval, duration/size caps, `can_iface`, `armd_pid`.
- `=== ONCE_START ===` .. `=== ONCE_END ===`: `uname -a`, `armd` package/HTTP version and cmdline, kernel config (`CONFIG_PREEMPT*`, `CONFIG_HZ*`, `NO_HZ`, `SMP`, `IRQ_FORCED_THREADING`), and startup scheduling policies/priorities (`chrt -p` + `/proc/.../stat`) for all `armd` threads and all `irq/*`, `spi*`, `ksoftirqd*`, and RT threads.
- `=== SAMPLE N ... ===` .. `=== SAMPLE_END ===`: periodic `/proc/loadavg`, `/proc/interrupts`, `/proc/softirqs`, `/proc/schedstat`, `ip -s -d link show can0`, per-thread `/proc/<armd_pid>/task/*/{stat,status,sched}` (including `VmLck`, `voluntary_ctxt_switches`, `nonvoluntary_ctxt_switches`), and compact IRQ/SPI thread CPU runtime snapshots.

### `report.md`
1. Stage 3 §6 Gate Verdict table and overall `PASS` / `FAIL` / `INVALID` / `INCOMPLETE` verdict.
2. Run and environment summary (`armd` version, `instance_id`, `owner_mode`, `release_us_max_ever`, `releases_total` / `releases_over_{10,15,20}ms` deltas, `wake_lateness_us` percentiles and `wake_late_max_ever` + `>1`/`>5`/`>10 ms` counts, `diag_dropped`, `since_marker`, kernel config, `control_thread` scheduling, `VmLck`, context switch deltas).
3. Release latency per load phase (`release_us` p50/p99/max, `tick_us` max, `lock_us`, `wait_us`, `period_us`, `wake_lateness_us` p99/max or legacy wake overrun proxy, `skipped_releases`/`late_count` delta, observed RX age).
4. Per-phase lock breakdown (`pre`, `rx`, `control`, `tx`, `finish`, `obs`, `publish`) and deduplicated worst-release Wall-vs-CPU table showing off-CPU preemption time, context switches, page faults, alloc calls, and lock holders.
5. On-device attribution (when `devstat.txt` is supplied): thread priorities, worst release intervals joined with IRQ/SoftIRQ deltas vs interval median, IRQ thread CPU deltas, and CAN error counter deltas.
6. Active diagnostic gaps list.

## Gaps

The following Stage 3 §6 metrics or controls are not exposed directly by `GET /api/v1/diagnostics/tick` or HTTP routes (`release_us`, `releases_total`, `release_us_max_ever`, `releases_over_{10,15,20}ms`, `wake_late_max_ever`, `skipped_releases`, `diag_dropped`, `wake_lateness_us`, `wake_lateness_over_{1,5,10}ms`, `instance_id`, `since_marker` via `POST /api/v1/diagnostics/tick/mark`, `owner_mode`, and `control_thread` scheduling are now exposed for both legacy and executor owners):

1. **Enqueue-to-wire CAN latency**: `lock_phases.tx_us` measures target encoding plus nonblocking SocketCAN `write()` syscall duration (`src/driver.rs`), not kernel qdisc, `spi0`, or MCP251xFD wire delivery time. Independent CAN bus timestamps or `SO_TIMESTAMPING` are required to close the MIT 45 ms wire budget.
2. **Per-IRQ runtime during a tick**: `/proc/interrupts` and `/proc/softirqs` report counts only, not handler duration. `devstat.sh` collects coarse threaded-IRQ CPU runtime (`se.sum_exec_runtime` / `utime+stime`) per sample interval, which does not include hard-IRQ time or per-tick overlapping timestamps (`ftrace` `irq_handler_entry`/`sched_switch` would be needed).
3. **In-loop RX age**: `GET /api/v1/diagnostics/tick` does not expose `feedback_age_ms`. `load.py` samples `feedback_age_ms` externally from `GET /api/v1/observation` and WS messages, which can miss transient single-tick peaks between external samples.
4. **Supervisor-lock attempt counter**: `SupervisorLock::control_thread_attempts` (`src/supervisor_lock.rs:93,159`) is not serialized in `GET /api/v1/diagnostics/tick`. Live runs check `wait_us.max == 0` and `waited_for == null` as proxies; authoritative proof relies on the B1 lock-proof tests.
5. **Full-release allocation counters**: `lock_alloc_calls` / `lock_alloc_us` / `lock_alloc_max_us` (`src/diagnostics/alloc.rs`) are zero unless `armd` is launched with `ARMD_ALLOC_TIMING=1`, and `TickSnapshot` only serializes them on `max_tick` and `max_lock` (not window-wide maxima or sums). `ExecutorStats::wake_alloc_calls` / `release_allocs` / `wake_allocs` (`src/rt_core/executor.rs`) are verified via the B2 allocation proof tests.
6. **Blackbox and log write-injection endpoint**: `armd` exposes no HTTP/WS endpoint to inject synthetic blackbox or log writes (`src/api/mod.rs`). Blackbox writes at 5 Hz automatically whenever torqued or moving (`src/diagnostics/blackbox.rs:28-38`); `load.py` supplements this with periodic `GET /api/v1/diagnostics/blackbox?seconds=60` flash reads and HTTP `TraceLayer` log traffic.
7. **Stop and fault trace endpoint**: `GET /api/v1/diagnostics/tick` does not include stop or fault traces. `load.py` records `fault`/`safety_state` and fetches a 30 s `GET /api/v1/diagnostics/blackbox` tail if a fault occurs.
