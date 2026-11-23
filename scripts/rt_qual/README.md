# SG2002 Real-Time Qualification Harness (`scripts/rt_qual/`)

Harness for the Stage 3 §6 qualification gate on the SG2002 reCamera arm (`armd` 1.6 RT release).

## Topology and Components

| Script | Runs on | Dependencies | Output |
|---|---|---|---|
| `devstat.sh` | reCamera (SG2002) | busybox `sh`, `/proc`, `ip` | `/userdata/rt-qual/<run-id>/devstat.txt` |
| `load.py` | Office Raspberry Pi | Python 3.11+ stdlib + `rebot` SDK (`rebot-companion`) | `load.jsonl` |
| `analyze.py` | Spark workstation | Python 3.11+ stdlib only | Markdown qualification report |

## Gate Criteria (Stage 3 §6)

1. **Release latency**: every measured release `< 20 ms` (`tick_us.max < 20000`).
2. **Deadlines**: zero skipped deadlines (`skipped_releases == 0`; proxy: `late_count` delta `== 0` and `period_us.max <= 2 * period`).
3. **Lock gate**: zero `SupervisorLock` attempts from `armd-control` (proxy in HTTP diagnostics: `wait_us.max == 0`, `waited_for == null`, `reader_hold_us.max == 0`, `writer_hold_us.max == 0`; authoritative proof via B1 `cargo test`).
4. **Allocation gate**: zero allocations/frees in release body (`lock_alloc_calls == 0` under `ARMD_ALLOC_TIMING=1`; authoritative proof via release allocation `cargo test`).
5. **MIT watchdog budget**: `20 ms + worst additional scheduling/RX/TX latency + 5 ms < 45 ms`.
6. **Zero faults**: no arm fault or E-stop during the run (`load.py` halts on fault and never clears faults).
7. **Cold-start verification**: repeat across cold starts and verify single-core `PREEMPT` / `HZ 250`, thread priorities (`armd-control` `SCHED_FIFO`, IRQ/SPI thread priorities), and `mlockall` (`memory_locked: true`, `VmLck > 0`).

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
1. Stage 3 §6 Gate Verdict table and overall `PASS` / `FAIL` / `INCOMPLETE` verdict.
2. Run and environment summary (`armd` version, kernel config, `control_thread` scheduling, `VmLck`, context switch deltas).
3. Release latency per load phase (`tick_us` p50/p99/max, `lock_us`, `wait_us`, `period_us`, wake overrun proxy, `late_count` delta, observed RX age).
4. Per-phase lock breakdown (`pre`, `rx`, `control`, `tx`, `finish`, `obs`, `publish`) and deduplicated worst-tick Wall-vs-CPU table showing off-CPU preemption time, context switches, page faults, alloc calls, and lock holders.
5. On-device attribution (when `devstat.txt` is supplied): thread priorities, worst release intervals joined with IRQ/SoftIRQ deltas vs interval median, IRQ thread CPU deltas, and CAN error counter deltas.
6. Active diagnostic gaps list.

## Gaps (Not Exposed on Current Branch `385f3a8`)

The following Stage 3 §6 metrics or controls are not exposed by `armd` on this branch and cannot be measured directly from `GET /api/v1/diagnostics/tick` or HTTP routes without changing `armd`:

1. **`skipped_releases` (skipped deadlines)**: `ExecutorStats::skipped_releases` (`src/rt_core/executor.rs:204`) and `diag_dropped` (`src/rt_core/executor.rs:206`) are internal `AtomicU64` counters not serialized in `TickSnapshot` (`src/diagnostics/tick.rs:209`). `GET /api/v1/diagnostics/tick` exposes only `late_count` (`dt > 2 * period`) and `period_us`.
2. **Wake lateness**: `TickSample` records start-to-start `period_us` (`dt`), not `actual_release_start - scheduled_release_ns`. `analyze.py` uses `max(0, period_us.max - nominal_period_us)` as a proxy, which conflates prior-tick overrun with wakeup delay.
3. **Enqueue-to-wire CAN latency**: `lock_phases.tx_us` measures target encoding plus nonblocking SocketCAN `write()` syscall duration (`src/driver.rs`), not kernel qdisc, `spi0`, or MCP251xFD wire delivery time. Independent CAN bus timestamps or `SO_TIMESTAMPING` are required to close the MIT 45 ms wire budget.
4. **Per-IRQ runtime during a tick**: `/proc/interrupts` and `/proc/softirqs` report counts only, not handler duration. `devstat.sh` collects coarse threaded-IRQ CPU runtime (`se.sum_exec_runtime` / `utime+stime`) per sample interval, which does not include hard-IRQ time or per-tick overlapping timestamps (`ftrace` `irq_handler_entry`/`sched_switch` would be needed).
5. **In-loop RX age**: `GET /api/v1/diagnostics/tick` does not expose `feedback_age_ms`. `load.py` samples `feedback_age_ms` externally from `GET /api/v1/observation` and WS messages, which can miss transient single-tick peaks between external samples.
6. **Supervisor-lock attempt counter**: `SupervisorLock::control_thread_attempts` (`src/supervisor_lock.rs:93,159`) is not serialized in `GET /api/v1/diagnostics/tick`. Live runs check `wait_us.max == 0` and `waited_for == null` as proxies; authoritative proof relies on the B1 lock-proof tests.
7. **Full-release allocation counters**: `lock_alloc_calls` / `lock_alloc_us` / `lock_alloc_max_us` (`src/diagnostics/alloc.rs`) are zero unless `armd` is launched with `ARMD_ALLOC_TIMING=1`, and `TickSnapshot` only serializes them on `max_tick` and `max_lock` (not window-wide maxima or sums). `ExecutorStats::wake_alloc_calls` (`src/rt_core/executor.rs:210`) is not exposed.
8. **Blackbox and log write-injection endpoint**: `armd` exposes no HTTP/WS endpoint to inject synthetic blackbox or log writes (`src/api/mod.rs`). Blackbox writes at 5 Hz automatically whenever torqued or moving (`src/diagnostics/blackbox.rs:28-38`); `load.py` supplements this with periodic `GET /api/v1/diagnostics/blackbox?seconds=60` flash reads and HTTP `TraceLayer` log traffic.
9. **Stop and fault trace endpoint**: `GET /api/v1/diagnostics/tick` does not include stop or fault traces. `load.py` records `fault`/`safety_state` and fetches a 30 s `GET /api/v1/diagnostics/blackbox` tail if a fault occurs.
