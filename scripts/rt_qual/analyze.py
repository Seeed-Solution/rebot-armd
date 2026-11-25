#!/usr/bin/env python3
"""Offline analyzer for SG2002 RT qualification runs (Stage 3 §6).

Reads load.jsonl (or legacy dev6 tickload.json) plus optional devstat.txt and
outputs a Markdown qualification report covering:
- release max/p99/p50 per load phase, wake lateness (or legacy period proxy), skipped releases,
  diagnostic ring drops (`diag_dropped`), RX age, enqueue-to-wire latency (if exposed), and
  per-phase lock breakdown with wall-vs-CPU;
- IRQ and softirq count deltas per interval joined to the worst release intervals;
- CAN error counter deltas;
- Pass/Fail verdict against the Stage 3 §6 gate criteria.

Python 3.11+ stdlib only.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

RELEASE_BUDGET_MS = 20.0
RELEASE_BUDGET_US = 20_000
TICK_WINDOW_CAPACITY = 1000
MIT_PERIOD_MS = 20.0
MIT_MARGIN_MS = 5.0
MIT_WATCHDOG_MS = 45.0

LOCK_PHASE_KEYS = (
    "pre_us",
    "rx_us",
    "control_us",
    "tx_us",
    "finish_us",
    "obs_us",
    "publish_us",
)

MONOTONIC_REQUIRED_KEYS = (
    "releases_total",
    "release_us_max_ever",
    "releases_over_10ms",
    "releases_over_15ms",
    "releases_over_20ms",
    "wake_late_max_ever",
    "skipped_releases",
    "since_boot_ticks",
    "diag_dropped",
)

PCT_FULL_REQUIRED_KEYS = (
    "release_us",
    "wake_lateness_us",
    "tick_us",
    "period_us",
)

PCT_MAX_REQUIRED_KEYS = (
    "lock_us",
    "wait_us",
    "reader_hold_us",
    "writer_hold_us",
)


RECOGNIZED_OWNER_MODES = ("legacy", "executor")
VALID_STOP_REASONS = ("completed", "interrupted", "fault", "error")
INVALID_BOOT_ID_PLACEHOLDERS = frozenset({"unknown", "unknown-boot", "none", "null", "n/a", "na"})
FAULT_SAFETY_STATES = frozenset({"Fault", "EStop", "E_STOP", "FAULT"})
KNOWN_LOAD_KEYS = frozenset(
    {"motion", "obs_poll", "ws_stream", "measure", "record", "blackbox_log", "bulk_http"}
)
MIN_IDLE_TAIL_S = 1.0
IDLE_TAIL_PERIOD_MULTIPLIER = 20.0
MIN_IDLE_TAIL_RELEASES = 10


def _strict_object_pairs_hook(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in pairs:
        if k in out:
            raise ValueError(f"duplicate JSON object key {k!r}")
        out[k] = v
    return out


def _strict_parse_constant(token: str) -> Any:
    raise ValueError(f"nonstandard JSON constant {token!r}")


def strict_json_loads(raw: str) -> Any:
    return json.loads(
        raw,
        object_pairs_hook=_strict_object_pairs_hook,
        parse_constant=_strict_parse_constant,
    )


def missing_requested_load_activity(enabled_loads: dict[str, bool], counters: dict[str, Any]) -> list[str]:
    """Return enabled load names with zero observed execution activity in `counters`."""
    missing: list[str] = []
    if enabled_loads.get("obs_poll"):
        obs_ok = get_nonneg_int(counters, "obs_ok")
        if not (obs_ok is not None and obs_ok > 0):
            missing.append("obs_poll")
    if enabled_loads.get("ws_stream"):
        ws_msgs = get_nonneg_int(counters, "ws_msgs") or 0
        ws_reconn = get_nonneg_int(counters, "ws_reconnects") or 0
        ws_err = counters.get("ws_err")
        if not (ws_msgs > 0 or ws_reconn > 0 or (isinstance(ws_err, dict) and bool(ws_err))):
            missing.append("ws_stream")
    if enabled_loads.get("measure"):
        m_ok = get_nonneg_int(counters, "measure_ok") or 0
        m_err = counters.get("measure_err")
        if not (m_ok > 0 or (isinstance(m_err, dict) and bool(m_err))):
            missing.append("measure")
    if enabled_loads.get("blackbox_log"):
        bb_ok = get_nonneg_int(counters, "blackbox_ok") or 0
        bb_err = counters.get("blackbox_err")
        if not (bb_ok > 0 or (isinstance(bb_err, dict) and bool(bb_err))):
            missing.append("blackbox_log")
    if enabled_loads.get("bulk_http"):
        b_ok = get_nonneg_int(counters, "bulk_ok") or 0
        b_err = counters.get("bulk_err")
        if not (b_ok > 0 or (isinstance(b_err, dict) and bool(b_err))):
            missing.append("bulk_http")
    if enabled_loads.get("motion"):
        moves = get_nonneg_int(counters, "motion_moves_done") or 0
        cycles = get_nonneg_int(counters, "motion_cycles_done") or 0
        if not (moves > 0 or cycles > 0):
            missing.append("motion")
    if enabled_loads.get("record"):
        rec_state = counters.get("record_state")
        if not (isinstance(rec_state, str) and rec_state not in ("disabled", "pending", "")):
            missing.append("record")
    return missing


def get_nonneg_int(d: dict[str, Any], key: str) -> int | None:
    v = d.get(key)
    if isinstance(v, int) and not isinstance(v, bool) and v >= 0:
        return int(v)
    return None


def get_positive_int(d: dict[str, Any], key: str) -> int | None:
    v = d.get(key)
    if isinstance(v, int) and not isinstance(v, bool) and v > 0:
        return int(v)
    return None


def get_strict_bool(d: dict[str, Any], key: str) -> bool | None:
    v = d.get(key)
    if isinstance(v, bool):
        return v
    return None


def get_pct(d: dict[str, Any], key: str, sub: str) -> int | None:
    obj = d.get(key)
    if not isinstance(obj, dict):
        return None
    return get_nonneg_int(obj, sub)


def get_nonempty_str(d: dict[str, Any], key: str) -> str | None:
    v = d.get(key)
    if isinstance(v, str) and v.strip():
        return v.strip()
    return None


def get_valid_boot_id(d: dict[str, Any]) -> str | None:
    s = get_nonempty_str(d, "boot_id")
    if s is None or s.lower() in INVALID_BOOT_ID_PLACEHOLDERS:
        return None
    return s


def sample_process_identity(tick: dict[str, Any]) -> tuple[str | None, str | None, int | None]:
    return (
        get_nonempty_str(tick, "instance_id"),
        get_valid_boot_id(tick),
        get_positive_int(tick, "process_start_unix_ns"),
    )


def identities_conflict(
    a: tuple[str | None, str | None, int | None],
    b: tuple[str | None, str | None, int | None],
) -> bool:
    return any(x is not None and y is not None and x != y for x, y in zip(a, b))


def us_to_ms(val: float | int | None) -> float | None:
    if val is None:
        return None
    return float(val) / 1000.0


def fmt_ms(val: float | None, digits: int = 2) -> str:
    if val is None:
        return "n/a"
    return f"{val:.{digits}f}"


def fmt_int(val: int | None) -> str:
    if val is None:
        return "n/a"
    return str(val)


def median_or_none(vals: list[float]) -> float | None:
    return statistics.median(vals) if vals else None


def max_or_none(vals: list[float]) -> float | None:
    return max(vals) if vals else None


def max_int_or_none(vals: list[int]) -> int | None:
    return max(vals) if vals else None


@dataclass
class CounterSeries:
    present_any: bool
    present_all: bool
    partial_missing: bool
    missing_count: int
    reset_detected: bool
    start: int | None
    end: int | None
    total_delta: int | None
    verified_in_run_delta: int | None
    per_sample_deltas: list[int | None]


def analyze_counter_series(samples: list[LoadSample], key: str) -> CounterSeries:
    """Chronologically track a cumulative monotonic integer field across all adjacent samples.

    - Requires >= 2 samples to compute a run delta (`total_delta = None` for 1-sample runs so
      process-lifetime counts are never mistaken for run deltas).
    - Never defaults missing/malformed entries to 0.
    - Detects counter resets (`cur < prev`) and cross-instance restarts (`instance_id` change):
      `verified_in_run_delta` sums ONLY strictly non-decreasing steps between adjacent samples
      sharing the same `instance_id` (both `None` or equal strings).
    """
    raw_vals: list[int | None] = [get_nonneg_int(s.tick, key) for s in samples]
    ident_vals = [sample_process_identity(s.tick) for s in samples]
    present_vals = [v for v in raw_vals if v is not None]
    missing_count = len(raw_vals) - len(present_vals)
    if not present_vals:
        return CounterSeries(False, False, False, len(raw_vals), False, None, None, None, None, [None] * len(samples))

    present_all = missing_count == 0
    partial_missing = missing_count > 0
    per_sample: list[int | None] = [None] * len(samples)

    if len(samples) < 2:
        return CounterSeries(
            present_any=True,
            present_all=present_all,
            partial_missing=partial_missing,
            missing_count=missing_count,
            reset_detected=False,
            start=present_vals[0],
            end=present_vals[-1],
            total_delta=None,
            verified_in_run_delta=None,
            per_sample_deltas=per_sample,
        )

    reset_detected = False
    verified_delta = 0
    any_verified_step = False
    prev_val: int | None = None
    prev_ident: tuple[str | None, str | None, int | None] | None = None
    for idx, (val, ident) in enumerate(zip(raw_vals, ident_vals)):
        if val is None:
            per_sample[idx] = None
            continue
        if prev_val is None:
            per_sample[idx] = 0
        elif prev_ident is not None and identities_conflict(prev_ident, ident):
            # Daemon / process / host identity changed: do not treat cross-restart difference as an in-run delta.
            per_sample[idx] = None
        elif val >= prev_val:
            step = val - prev_val
            per_sample[idx] = step
            verified_delta += step
            any_verified_step = True
        else:
            reset_detected = True
            per_sample[idx] = None
        prev_val = val
        prev_ident = ident

    total_delta = verified_delta if any_verified_step else None
    verified_in_run_delta = verified_delta if any_verified_step else None
    return CounterSeries(
        present_any=True,
        present_all=present_all,
        partial_missing=partial_missing,
        missing_count=missing_count,
        reset_detected=reset_detected,
        start=present_vals[0],
        end=present_vals[-1],
        total_delta=total_delta,
        verified_in_run_delta=verified_in_run_delta,
        per_sample_deltas=per_sample,
    )


@dataclass
class LoadSample:
    index: int
    ts_unix: float | None
    ts_iso: str | None
    elapsed_s: float | None
    phase: str
    tick: dict[str, Any]
    rx_age_ms: dict[str, Any] | None = None
    counters: dict[str, Any] = field(default_factory=dict)
    fault: str | None = None
    safety_state: str | None = None


@dataclass
class LoadRun:
    source_format: str
    meta: dict[str, Any]
    summary: dict[str, Any]
    samples: list[LoadSample]
    failed_polls: list[dict[str, Any]] = field(default_factory=list)
    has_summary: bool = False
    grammar_errors: list[str] = field(default_factory=list)
    final_sample_count: int = 0
    idle_tail_start_count: int = 0


def load_input(path: Path) -> LoadRun:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return LoadRun("jsonl", {}, {}, [], grammar_errors=[f"empty load file: {path}"])

    # Auto-detect legacy dev6 tickload.json (single JSON object with "ticks": [...]) vs native JSONL.
    if text.startswith("{"):
        try:
            obj = strict_json_loads(text)
            if isinstance(obj, dict) and isinstance(obj.get("ticks"), list):
                samples: list[LoadSample] = []
                failed_polls: list[dict[str, Any]] = []
                for idx, t in enumerate(obj["ticks"]):
                    if not isinstance(t, dict):
                        failed_polls.append({"line": idx + 1, "phase": "tickload", "error": "non-object tick entry"})
                        continue
                    samples.append(
                        LoadSample(
                            index=len(samples),
                            ts_unix=None,
                            ts_iso=None,
                            elapsed_s=float(idx * 5),
                            phase="tickload",
                            tick=t,
                            fault=None if str(obj.get("fault")) in ("None", "null", "") else str(obj.get("fault")),
                        )
                    )
                meta = {
                    "run_id": path.stem,
                    "source_file": str(path),
                    "note": "Converted from legacy tickload.json (unsealed diagnostic format).",
                }
                summary = {
                    "fault": None if str(obj.get("fault")) in ("None", "null", "") else str(obj.get("fault")),
                    "measure": obj.get("measure"),
                }
                return LoadRun("tickload_json", meta, summary, samples, failed_polls=failed_polls)
        except (json.JSONDecodeError, ValueError):
            pass

    meta: dict[str, Any] = {}
    summary: dict[str, Any] = {}
    has_summary = False
    meta_count = 0
    summary_count = 0
    final_sample_count = 0
    idle_tail_start_count = 0
    samples = []
    failed_polls = []
    grammar_errors: list[str] = []
    seen_records: list[tuple[int, str]] = []
    expected_run_id: str | None = None
    idx = 0

    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        try:
            rec = strict_json_loads(line)
        except (json.JSONDecodeError, ValueError) as exc:
            msg = exc.msg if isinstance(exc, json.JSONDecodeError) else str(exc)
            grammar_errors.append(f"malformed JSON at line {line_no}: {msg}")
            continue
        if not isinstance(rec, dict):
            grammar_errors.append(f"non-object JSON record at line {line_no}")
            continue
        rtype = rec.get("type")
        if rtype not in ("meta", "sample", "summary"):
            grammar_errors.append(f"unknown or missing record type at line {line_no}: {rtype!r}")
            continue

        rec_run_id = get_nonempty_str(rec, "run_id")
        if rec_run_id is None:
            grammar_errors.append(f"missing or empty run_id on {rtype} record at line {line_no}")
        elif expected_run_id is None:
            expected_run_id = rec_run_id
        elif rec_run_id != expected_run_id:
            grammar_errors.append(
                f"foreign run_id on {rtype} record at line {line_no}: {rec_run_id!r} != {expected_run_id!r}"
            )

        if summary_count > 0:
            grammar_errors.append(f"trailing {rtype} record after terminal summary at line {line_no}")

        seen_records.append((line_no, rtype))
        if rtype == "meta":
            meta_count += 1
            if len(seen_records) != 1:
                grammar_errors.append(f"misplaced meta record at line {line_no} (must be first record)")
            if meta_count > 1:
                grammar_errors.append(f"duplicate meta record at line {line_no}")
            else:
                meta = rec
        elif rtype == "summary":
            summary_count += 1
            if summary_count > 1:
                grammar_errors.append(f"duplicate summary record at line {line_no}")
            else:
                summary = rec
                has_summary = True
        else:
            tick_err = rec.get("tick_error")
            poll_failed = bool(rec.get("poll_failed")) or (tick_err is not None and str(tick_err).strip() != "")
            tick = rec.get("tick")
            phase_str = str(rec.get("phase") or "unknown")
            if phase_str == "final":
                final_sample_count += 1
            elif phase_str == "idle_tail_start":
                idle_tail_start_count += 1
            if poll_failed or not isinstance(tick, dict):
                failed_polls.append(
                    {
                        "line": line_no,
                        "phase": phase_str,
                        "elapsed_s": rec.get("elapsed_s"),
                        "error": str(tick_err) if tick_err else "missing or non-object tick payload",
                    }
                )
            if not isinstance(tick, dict):
                continue
            samples.append(
                LoadSample(
                    index=idx,
                    ts_unix=float(rec["ts_unix"]) if isinstance(rec.get("ts_unix"), (int, float)) and not isinstance(rec.get("ts_unix"), bool) else None,
                    ts_iso=rec.get("ts_iso") if isinstance(rec.get("ts_iso"), str) else None,
                    elapsed_s=float(rec["elapsed_s"]) if isinstance(rec.get("elapsed_s"), (int, float)) and not isinstance(rec.get("elapsed_s"), bool) else None,
                    phase=phase_str,
                    tick=tick,
                    rx_age_ms=rec.get("rx_age_ms") if isinstance(rec.get("rx_age_ms"), dict) else None,
                    counters=rec.get("counters") if isinstance(rec.get("counters"), dict) else {},
                    fault=rec.get("fault") if isinstance(rec.get("fault"), str) else None,
                    safety_state=rec.get("safety_state") if isinstance(rec.get("safety_state"), str) else None,
                )
            )
            idx += 1

    if meta_count == 0:
        grammar_errors.append("missing leading meta record")
    return LoadRun(
        "jsonl",
        meta,
        summary,
        samples,
        failed_polls=failed_polls,
        has_summary=has_summary,
        grammar_errors=grammar_errors,
        final_sample_count=final_sample_count,
        idle_tail_start_count=idle_tail_start_count,
    )


# ---- devstat.txt parsing ----------------------------------------------------

@dataclass
class CanStats:
    state: str | None = None
    berr_tx: int | None = None
    berr_rx: int | None = None
    restarts: int | None = None
    error_warning: int | None = None
    error_passive: int | None = None
    bus_off: int | None = None
    rx_packets: int | None = None
    rx_errors: int | None = None
    rx_dropped: int | None = None
    rx_overrun: int | None = None
    tx_packets: int | None = None
    tx_errors: int | None = None
    tx_dropped: int | None = None
    tx_carrier: int | None = None


@dataclass
class DevstatSample:
    index: int
    ts_unix: float | None
    ts_iso: str | None
    uptime_s: float | None
    loadavg: str | None = None
    interrupts: dict[str, int] = field(default_factory=dict)
    softirqs: dict[str, int] = field(default_factory=dict)
    schedstat_cpu_running_ns: int | None = None
    schedstat_cpu_waiting_ns: int | None = None
    schedstat_cpu_timeslices: int | None = None
    armd_tasks: dict[str, dict[str, Any]] = field(default_factory=dict)
    irq_tasks: dict[str, dict[str, Any]] = field(default_factory=dict)
    can: CanStats = field(default_factory=CanStats)


@dataclass
class DevstatRun:
    meta: dict[str, str]
    uname: str
    armd_version: list[str]
    kernel_config: list[str]
    armd_threads: list[str]
    irq_threads: list[str]
    samples: list[DevstatSample]
    end: dict[str, str]


def parse_interrupts_block(lines: list[str]) -> dict[str, int]:
    """Parse /proc/interrupts into {label: total_count_across_cpus}."""
    out: dict[str, int] = {}
    num_cpus: int | None = None
    for raw in lines:
        s = raw.strip()
        if not s:
            continue
        if s.startswith("CPU0"):
            num_cpus = len(s.split())
            continue
        if ":" not in s:
            continue
        irq_id, rest = s.split(":", 1)
        irq_id = irq_id.strip()
        toks = rest.split()
        if not toks:
            continue
        counts: list[int] = []
        desc_idx = 0
        limit = num_cpus if num_cpus is not None else len(toks)
        for i, tok in enumerate(toks):
            if len(counts) < limit and tok.isdigit():
                counts.append(int(tok))
                desc_idx = i + 1
            else:
                break
        if not counts:
            continue
        desc = " ".join(toks[desc_idx:]).strip()
        label = f"{irq_id}: {desc}" if desc else irq_id
        out[label] = sum(counts)
    return out


def parse_softirqs_block(lines: list[str]) -> dict[str, int]:
    out: dict[str, int] = {}
    for raw in lines:
        s = raw.strip()
        if not s or s.startswith("CPU0") or ":" not in s:
            continue
        name, rest = s.split(":", 1)
        nums = [int(t) for t in rest.split() if t.isdigit()]
        if nums:
            out[name.strip()] = sum(nums)
    return out


def parse_schedstat_block(lines: list[str]) -> tuple[int | None, int | None, int | None]:
    run_ns = 0
    wait_ns = 0
    slices = 0
    seen = False
    for raw in lines:
        toks = raw.strip().split()
        if len(toks) >= 10 and toks[0].startswith("cpu") and toks[0] != "cpu":
            try:
                # /proc/schedstat cpu line: cpuN yld_cnt 0 sched_cnt sched_goidle ttwu_cnt ttwu_local rq_cpu_time run_delay pcount
                run_ns += int(toks[7])
                wait_ns += int(toks[8])
                slices += int(toks[9])
                seen = True
            except ValueError:
                pass
    return (run_ns, wait_ns, slices) if seen else (None, None, None)


def parse_can_block(lines: list[str]) -> CanStats:
    cs = CanStats()
    text = "\n".join(lines)
    m = re.search(r"\bcan\s+state\s+([A-Z0-9_-]+)", text) or re.search(r"\bstate\s+([A-Z0-9_-]+)", text)
    if m:
        cs.state = m.group(1)
    m = re.search(r"berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)", text)
    if m:
        cs.berr_tx, cs.berr_rx = int(m.group(1)), int(m.group(2))
    # Line format from `ip -s -d link show can0`:
    #   re-started bus-errors arbit-lost error-warn error-pass bus-off
    #   0          0          0          27         38         0
    for idx, line in enumerate(lines):
        if "error-warn" in line and "bus-off" in line and idx + 1 < len(lines):
            hdr = line.split()
            vals = lines[idx + 1].split()
            if len(vals) >= len(hdr):
                kv = dict(zip(hdr, vals))
                for src, attr in (
                    ("re-started", "restarts"),
                    ("error-warn", "error_warning"),
                    ("error-pass", "error_passive"),
                    ("bus-off", "bus_off"),
                ):
                    if src in kv and kv[src].isdigit():
                        setattr(cs, attr, int(kv[src]))
        if line.strip().startswith("RX:") and idx + 1 < len(lines):
            vals = lines[idx + 1].split()
            if len(vals) >= 5 and all(v.isdigit() for v in vals[:5]):
                cs.rx_packets = int(vals[1])
                cs.rx_errors = int(vals[2])
                cs.rx_dropped = int(vals[3])
                cs.rx_overrun = int(vals[4])
        if line.strip().startswith("TX:") and idx + 1 < len(lines):
            vals = lines[idx + 1].split()
            if len(vals) >= 5 and all(v.isdigit() for v in vals[:5]):
                cs.tx_packets = int(vals[1])
                cs.tx_errors = int(vals[2])
                cs.tx_dropped = int(vals[3])
                cs.tx_carrier = int(vals[4])
    return cs


def parse_armd_tasks_block(lines: list[str]) -> dict[str, dict[str, Any]]:
    tasks: dict[str, dict[str, Any]] = {}
    cur_key: str | None = None
    for raw in lines:
        s = raw.strip()
        if not s or s == "armd_pid_not_found":
            continue
        if s.startswith("TASK "):
            m = re.match(r"TASK\s+tid=(\d+)\s+comm=(.+)", s)
            if m:
                tid, comm = m.group(1), m.group(2).strip()
                cur_key = f"{comm}:{tid}"
                tasks[cur_key] = {"tid": int(tid), "comm": comm}
            continue
        if cur_key is None:
            continue
        rec = tasks[cur_key]
        if s.startswith("VmLck:"):
            m = re.search(r"(\d+)", s)
            if m:
                rec["vm_lck_kb"] = int(m.group(1))
        elif s.startswith("VmRSS:"):
            m = re.search(r"(\d+)", s)
            if m:
                rec["vm_rss_kb"] = int(m.group(1))
        elif s.startswith("voluntary_ctxt_switches:"):
            m = re.search(r"(\d+)", s)
            if m:
                rec["vcsw"] = int(m.group(1))
        elif s.startswith("nonvoluntary_ctxt_switches:"):
            m = re.search(r"(\d+)", s)
            if m:
                rec["ivcsw"] = int(m.group(1))
        elif s.startswith("se.sum_exec_runtime"):
            parts = s.split(":", 1)
            if len(parts) == 2:
                try:
                    rec["sum_exec_ms"] = float(parts[1].strip())
                except ValueError:
                    pass
        elif s.startswith("se.statistics.wait_sum"):
            parts = s.split(":", 1)
            if len(parts) == 2:
                try:
                    rec["wait_sum_ms"] = float(parts[1].strip())
                except ValueError:
                    pass
        elif s.startswith("se.statistics.wait_max"):
            parts = s.split(":", 1)
            if len(parts) == 2:
                try:
                    rec["wait_max_ms"] = float(parts[1].strip())
                except ValueError:
                    pass
    return tasks


def parse_irq_tasks_block(lines: list[str]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    for raw in lines:
        s = raw.strip()
        if not s.startswith("tid="):
            continue
        m = re.match(r"tid=(\d+)\s+comm=(\S+)\s+utime=(\d+)\s+stime=(\d+)\s+sum_exec_ms=(\S+)", s)
        if not m:
            continue
        tid, comm, ut, st, ex = m.groups()
        rec: dict[str, Any] = {"tid": int(tid), "comm": comm, "ticks": int(ut) + int(st)}
        if ex != "na":
            try:
                rec["sum_exec_ms"] = float(ex)
            except ValueError:
                pass
        out[f"{comm} (tid {tid})"] = rec
    return out


def parse_devstat(path: Path) -> DevstatRun:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    meta: dict[str, str] = {}
    end: dict[str, str] = {}
    once_sections: dict[str, list[str]] = {}
    samples: list[DevstatSample] = []

    mode = "none"
    sub = ""
    cur_sample_meta: dict[str, str] = {}
    cur_sample_blocks: dict[str, list[str]] = {}

    def finalize_sample() -> None:
        if not cur_sample_meta:
            return
        idx = int(cur_sample_meta.get("idx", len(samples)))
        ts_u = float(cur_sample_meta["ts_unix"]) if "ts_unix" in cur_sample_meta else None
        up = float(cur_sample_meta["uptime"]) if "uptime" in cur_sample_meta else None
        run_ns, wait_ns, slices = parse_schedstat_block(cur_sample_blocks.get("schedstat", []))
        load_lines = cur_sample_blocks.get("loadavg", [])
        samples.append(
            DevstatSample(
                index=idx,
                ts_unix=ts_u,
                ts_iso=cur_sample_meta.get("ts_iso"),
                uptime_s=up,
                loadavg=load_lines[0].strip() if load_lines else None,
                interrupts=parse_interrupts_block(cur_sample_blocks.get("interrupts", [])),
                softirqs=parse_softirqs_block(cur_sample_blocks.get("softirqs", [])),
                schedstat_cpu_running_ns=run_ns,
                schedstat_cpu_waiting_ns=wait_ns,
                schedstat_cpu_timeslices=slices,
                armd_tasks=parse_armd_tasks_block(cur_sample_blocks.get("armd_tasks", [])),
                irq_tasks=parse_irq_tasks_block(cur_sample_blocks.get("irq_tasks", [])),
                can=parse_can_block(cur_sample_blocks.get("can_link", [])),
            )
        )

    for line in lines:
        if line == "=== META ===":
            mode = "meta"
            continue
        if line == "=== ONCE_START ===":
            mode = "once"
            sub = ""
            continue
        if line == "=== ONCE_END ===":
            mode = "none"
            continue
        if line.startswith("=== SAMPLE "):
            mode = "sample"
            sub = ""
            cur_sample_blocks = {}
            cur_sample_meta = {}
            m = re.match(r"=== SAMPLE (\d+)\s+(.*?)\s*===", line)
            if m:
                cur_sample_meta["idx"] = m.group(1)
                for kv in m.group(2).split():
                    if "=" in kv:
                        k, v = kv.split("=", 1)
                        cur_sample_meta[k] = v
            continue
        if line == "=== SAMPLE_END ===":
            finalize_sample()
            cur_sample_meta = {}
            cur_sample_blocks = {}
            mode = "none"
            continue
        if line == "=== END ===":
            mode = "end"
            continue

        if mode == "meta":
            if "=" in line:
                k, v = line.split("=", 1)
                meta[k.strip()] = v.strip()
        elif mode == "end":
            if "=" in line:
                k, v = line.split("=", 1)
                end[k.strip()] = v.strip()
        elif mode == "once":
            m = re.match(r"^---\s+([a-zA-Z0-9_]+)\s+---$", line.strip())
            if m:
                sub = m.group(1)
                once_sections.setdefault(sub, [])
            elif sub:
                once_sections[sub].append(line)
        elif mode == "sample":
            m = re.match(r"^---\s+([a-zA-Z0-9_]+)\s+---$", line.strip())
            if m:
                sub = m.group(1)
                cur_sample_blocks.setdefault(sub, [])
            elif sub:
                cur_sample_blocks[sub].append(line)

    return DevstatRun(
        meta=meta,
        uname=" ".join(once_sections.get("uname", [])).strip() or "unavailable",
        armd_version=[l for l in once_sections.get("armd_version", []) if l.strip()],
        kernel_config=[l for l in once_sections.get("kernel_config", []) if l.strip()],
        armd_threads=[l for l in once_sections.get("armd_threads", []) if l.strip()],
        irq_threads=[l for l in once_sections.get("irq_and_rt_threads", []) if l.strip()],
        samples=samples,
        end=end,
    )


# ---- Analysis & Report Generation -------------------------------------------

def infer_nominal_period_us(samples: list[LoadSample], override_us: int | None) -> int:
    if override_us is not None:
        return override_us
    p50s = [
        float(v)
        for s in samples
        if (v := get_pct(s.tick, "period_us", "p50")) is not None and v > 0
    ]
    if not p50s:
        return 20_000
    med = statistics.median(p50s)
    # Snap to nearest 1000 us (e.g. 20000 us for 50 Hz, 10000 us for 100 Hz).
    return max(1000, int(round(med / 1000.0) * 1000))


RELEASE_PHASE_KEYS = (
    "alloc_read",
    "ack_flush",
    "terminal",
    "guard_drop",
    "post",
    "publish",
)

SCHED_SWITCH_RE = re.compile(
    r"\[\d+\]\s+\S+\s+(?P<ts>\d+\.\d+):\s+sched_switch:\s+"
    r"prev_comm=(?P<prev_comm>.+?)\s+prev_pid=(?P<prev_pid>\d+)\s+"
    r"prev_prio=(?P<prev_prio>-?\d+)\s+prev_state=(?P<prev_state>\S+)\s+==>\s+"
    r"next_comm=(?P<next_comm>.+?)\s+next_pid=(?P<next_pid>\d+)\s+"
    r"next_prio=(?P<next_prio>-?\d+)\s*$"
)


def get_signed_int(d: dict[str, Any], key: str) -> int | None:
    v = d.get(key)
    if isinstance(v, int) and not isinstance(v, bool):
        return int(v)
    return None


@dataclass
class SchedSwitchOut:
    ts_out: float
    ts_in: float
    duration_us: int
    prev_state: str
    prev_prio: int
    next_comm: str
    next_pid: int
    next_prio: int

    @property
    def duration_ms(self) -> float:
        return self.duration_us / 1000.0


@dataclass
class SchedTraceAnalysis:
    control_tid: int
    min_duration_ms: float
    total_switch_events: int
    control_switch_outs: int
    long_switch_outs: list[SchedSwitchOut]


def parse_sched_trace(
    path: Path,
    control_tid: int,
    min_duration_ms: float = 2.0,
) -> SchedTraceAnalysis:
    """Parse Linux ftrace output and list switch-outs of `control_tid` strictly longer than `min_duration_ms`."""
    min_us = int(round(min_duration_ms * 1000.0))
    total_events = 0
    control_outs = 0
    long_outs: list[SchedSwitchOut] = []
    # Active switch-out state: (ts_out, prev_state, prev_prio, next_comm, next_pid, next_prio)
    pending_out: tuple[float, str, int, str, int, int] | None = None

    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if not line or line.startswith("#") or "sched_switch:" not in line:
                continue
            m = SCHED_SWITCH_RE.search(line)
            if not m:
                continue
            total_events += 1
            ts = float(m.group("ts"))
            prev_pid = int(m.group("prev_pid"))
            next_pid = int(m.group("next_pid"))
            prev_prio = int(m.group("prev_prio"))
            next_prio = int(m.group("next_prio"))
            prev_state = m.group("prev_state")
            next_comm = m.group("next_comm")

            if prev_pid == control_tid and next_pid != control_tid:
                control_outs += 1
                pending_out = (ts, prev_state, prev_prio, next_comm, next_pid, next_prio)
            elif next_pid == control_tid and pending_out is not None:
                ts_out, p_state, p_prio, n_comm, n_pid, n_prio = pending_out
                pending_out = None
                if ts >= ts_out:
                    dur_us = int(round((ts - ts_out) * 1_000_000.0))
                    if dur_us > min_us:
                        long_outs.append(
                            SchedSwitchOut(
                                ts_out=ts_out,
                                ts_in=ts,
                                duration_us=dur_us,
                                prev_state=p_state,
                                prev_prio=p_prio,
                                next_comm=n_comm,
                                next_pid=n_pid,
                                next_prio=n_prio,
                            )
                        )

    long_outs.sort(key=lambda e: e.duration_us, reverse=True)
    return SchedTraceAnalysis(
        control_tid=control_tid,
        min_duration_ms=min_duration_ms,
        total_switch_events=total_events,
        control_switch_outs=control_outs,
        long_switch_outs=long_outs,
    )


def render_sched_trace_section(trace: SchedTraceAnalysis, heading: str = "## 7. Scheduler Switch-Outs (`sched_trace`)") -> list[str]:
    out: list[str] = [
        heading,
        "",
        f"- **Target TID (`armd-control`)**: `{trace.control_tid}` | "
        f"**Threshold**: `> {trace.min_duration_ms:.2f} ms` | "
        f"**Total `sched_switch` events**: `{trace.total_switch_events}` | "
        f"**`armd-control` switch-outs**: `{trace.control_switch_outs}` | "
        f"**Over threshold**: `{len(trace.long_switch_outs)}`",
        "",
    ]
    if not trace.long_switch_outs:
        out.append(f"_No switch-outs of TID `{trace.control_tid}` longer than {trace.min_duration_ms:.2f} ms._")
        out.append("")
        return out
    out.append("| # | Out TS (s) | In TS (s) | Duration (ms) | Prev State | Next Task | Next PID | Next Prio |")
    out.append("|---:|---:|---:|---:|---|---|---:|---:|")
    for idx, ev in enumerate(trace.long_switch_outs, start=1):
        out.append(
            f"| {idx} | `{ev.ts_out:.6f}` | `{ev.ts_in:.6f}` | **{ev.duration_ms:.3f}** | "
            f"`{ev.prev_state}` | `{ev.next_comm}` | {ev.next_pid} | {ev.next_prio} |"
        )
    out.append("")
    return out


def collect_unique_breakdowns(samples: list[LoadSample]) -> list[tuple[str, str, int, dict[str, Any]]]:
    """Return deduplicated (kind, phase, sample_idx, breakdown) sorted by release_us/tick_us desc."""
    by_sig: dict[tuple[Any, ...], list[Any]] = {}
    for s in samples:
        for kind in ("max_release", "max_tick", "max_lock", "max_wait"):
            bd = s.tick.get(kind)
            if not isinstance(bd, dict):
                continue
            holder = bd.get("waited_for")
            phases_us = bd.get("phases_us")
            sig = (
                bd.get("release_us"),
                bd.get("tick_us"),
                bd.get("lock_us"),
                bd.get("wait_us"),
                bd.get("period_us"),
                bd.get("rx_us"),
                bd.get("control_us"),
                bd.get("tx_us"),
                bd.get("obs_us"),
                bd.get("lock_cpu_us"),
                bd.get("lock_ivcsw"),
                bd.get("mid_nvcsw"),
                bd.get("mid_nivcsw"),
                bd.get("handoff_us"),
                tuple(phases_us.get(k) for k in RELEASE_PHASE_KEYS) if isinstance(phases_us, dict) else None,
                holder.get("location") if isinstance(holder, dict) else None,
                holder.get("command") if isinstance(holder, dict) else None,
                holder.get("hold_us") if isinstance(holder, dict) else None,
                holder.get("released_before_lock_us") if isinstance(holder, dict) else None,
            )
            if sig not in by_sig:
                by_sig[sig] = [[kind], s.phase, s.index, bd]
            elif kind not in by_sig[sig][0]:
                by_sig[sig][0].append(kind)
    rows = [("+".join(v[0]), v[1], v[2], v[3]) for v in by_sig.values()]
    rows.sort(
        key=lambda r: (
            get_nonneg_int(r[3], "release_us") or get_nonneg_int(r[3], "tick_us") or 0,
            get_nonneg_int(r[3], "wait_us") or 0,
            get_nonneg_int(r[3], "lock_us") or 0,
        ),
        reverse=True,
    )
    return rows


def collect_unique_kind_breakdowns(
    samples: list[LoadSample],
    kind: str,
    sort_key: str,
) -> list[tuple[str, int, dict[str, Any]]]:
    """Return deduplicated (phase, sample_idx, breakdown) for one slot (`max_release` or `max_wait`)."""
    by_sig: dict[tuple[Any, ...], tuple[str, int, dict[str, Any]]] = {}
    for s in samples:
        bd = s.tick.get(kind)
        if not isinstance(bd, dict):
            continue
        holder = bd.get("waited_for")
        phases_us = bd.get("phases_us")
        sig = (
            bd.get("release_us"),
            bd.get("tick_us"),
            bd.get("wait_us"),
            bd.get("lock_us"),
            bd.get("mid_nvcsw"),
            bd.get("mid_nivcsw"),
            bd.get("handoff_us"),
            tuple(phases_us.get(k) for k in RELEASE_PHASE_KEYS) if isinstance(phases_us, dict) else None,
            holder.get("kind") if isinstance(holder, dict) else None,
            holder.get("location") if isinstance(holder, dict) else None,
            holder.get("command") if isinstance(holder, dict) else None,
            holder.get("hold_us") if isinstance(holder, dict) else None,
            holder.get("released_before_lock_us") if isinstance(holder, dict) else None,
        )
        if sig not in by_sig:
            by_sig[sig] = (s.phase, s.index, bd)
    rows = list(by_sig.values())
    rows.sort(
        key=lambda r: (
            get_nonneg_int(r[2], sort_key) or 0,
            get_nonneg_int(r[2], "release_us") or get_nonneg_int(r[2], "tick_us") or 0,
        ),
        reverse=True,
    )
    return rows


def find_armd_control_task(tasks: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
    for rec in tasks.values():
        if rec.get("comm") == "armd-control":
            return rec
    for rec in tasks.values():
        if "control" in str(rec.get("comm", "")):
            return rec
    return None


def compute_dict_deltas(a: dict[str, int], b: dict[str, int]) -> dict[str, int]:
    keys = set(a.keys()) | set(b.keys())
    deltas: dict[str, int] = {}
    for k in keys:
        d = b.get(k, 0) - a.get(k, 0)
        if d > 0:
            deltas[k] = d
    return deltas


def _render_all_failed_report(load_run: LoadRun, devstat: DevstatRun | None) -> str:
    run_id = load_run.meta.get("run_id") or (devstat.meta.get("run_id") if devstat else "unknown")
    n_fail = len(load_run.failed_polls)
    if load_run.source_format == "tickload_json":
        headline = "**Overall Verdict: DIAGNOSTIC ONLY (legacy tickload.json; no valid tick samples)**"
    elif n_fail > 0:
        headline = f"**Overall Verdict: INVALID (all {n_fail} poll(s) failed; no tick samples captured)**"
    elif load_run.grammar_errors:
        headline = f"**Overall Verdict: INVALID ({'; '.join(load_run.grammar_errors[:3])})**"
    else:
        headline = "**Overall Verdict: INVALID (no valid tick samples captured)**"
    out = [
        f"# SG2002 Real-Time Qualification Report (`{run_id}`)",
        "",
        headline,
        "",
        "## 1. Stage 3 §6 Gate Verdict",
        "",
        "| Criterion | Target | Measured / Evidence | Status |",
        "|---|---|---|---|",
        f"| Every release (`release_us`) | `< {RELEASE_BUDGET_MS:.0f} ms` | no valid tick samples ({n_fail} failed poll(s)) | INVALID |",
        f"| Zero skipped deadlines | `0` | no valid tick samples ({n_fail} failed poll(s)) | INVALID |",
        "",
        "## 2. Capture Errors / Failed Polls",
        "",
    ]
    for ge in load_run.grammar_errors[:20]:
        out.append(f"- **INVALID**: {ge}")
    for fp in load_run.failed_polls[:20]:
        out.append(f"- line {fp.get('line')}: phase=`{fp.get('phase')}`, error=`{fp.get('error')}`")
    out.append("")
    return "\n".join(out)


def analyze(
    load_run: LoadRun,
    devstat: DevstatRun | None,
    nominal_period_us: int | None = None,
    top_intervals: int = 5,
    sched_trace: SchedTraceAnalysis | None = None,
) -> str:
    samples = load_run.samples
    if not samples:
        return _render_all_failed_report(load_run, devstat)

    nom_period_us = infer_nominal_period_us(samples, nominal_period_us)
    nom_period_ms = nom_period_us / 1000.0
    n_samples = len(samples)

    # ---- Strict non-defaulting extraction (missing/malformed NEVER defaults to 0) ----
    def collect_pct(key: str, sub: str) -> tuple[list[int], int]:
        vals: list[int] = []
        missing = 0
        for s in samples:
            v = get_pct(s.tick, key, sub)
            if v is None:
                missing += 1
            else:
                vals.append(v)
        return vals, missing

    rel_max_vals, rel_max_miss = collect_pct("release_us", "max")
    rel_p99_vals, rel_p99_miss = collect_pct("release_us", "p99")
    rel_p50_vals, rel_p50_miss = collect_pct("release_us", "p50")
    has_release_us = bool(rel_max_vals)
    release_us_max_us = max_int_or_none(rel_max_vals)
    release_us_p99_max_us = max_int_or_none(rel_p99_vals)
    release_us_p50_med_us = median_or_none([float(v) for v in rel_p50_vals])

    tick_max_vals, tick_max_miss = collect_pct("tick_us", "max")
    tick_p99_vals, tick_p99_miss = collect_pct("tick_us", "p99")
    tick_p50_vals, tick_p50_miss = collect_pct("tick_us", "p50")
    tick_max_us = max_int_or_none(tick_max_vals)
    tick_p99_max_us = max_int_or_none(tick_p99_vals)
    tick_p50_med_us = median_or_none([float(v) for v in tick_p50_vals])

    lock_max_vals, lock_max_miss = collect_pct("lock_us", "max")
    wait_max_vals, wait_max_miss = collect_pct("wait_us", "max")
    reader_hold_vals, reader_hold_miss = collect_pct("reader_hold_us", "max")
    writer_hold_vals, writer_hold_miss = collect_pct("writer_hold_us", "max")
    period_max_vals, period_max_miss = collect_pct("period_us", "max")
    period_p99_vals, period_p99_miss = collect_pct("period_us", "p99")
    period_p50_vals, period_p50_miss = collect_pct("period_us", "p50")
    lock_max_us = max_int_or_none(lock_max_vals)
    wait_max_us = max_int_or_none(wait_max_vals)
    reader_hold_max_us = max_int_or_none(reader_hold_vals)
    writer_hold_max_us = max_int_or_none(writer_hold_vals)
    period_max_us = max_int_or_none(period_max_vals)
    period_p99_max_us = max_int_or_none(period_p99_vals)

    wake_max_vals, wake_max_miss = collect_pct("wake_lateness_us", "max")
    wake_p99_vals, wake_p99_miss = collect_pct("wake_lateness_us", "p99")
    wake_p50_vals, wake_p50_miss = collect_pct("wake_lateness_us", "p50")
    has_wake_lateness = bool(wake_max_vals)
    wake_lateness_partial = has_wake_lateness and (wake_max_miss > 0 or wake_p99_miss > 0 or wake_p50_miss > 0)
    wake_lateness_max_us = max_int_or_none(wake_max_vals)
    wake_lateness_p99_max_us = max_int_or_none(wake_p99_vals)
    wake_lateness_p50_med_us = median_or_none([float(v) for v in wake_p50_vals])

    # ---- Monotonic counter series ----
    since_boot_series = analyze_counter_series(samples, "since_boot_ticks")
    releases_started_series = analyze_counter_series(samples, "releases_started")
    releases_total_series = analyze_counter_series(samples, "releases_total")
    release_max_ever_series = analyze_counter_series(samples, "release_us_max_ever")
    rel_over_10ms = analyze_counter_series(samples, "releases_over_10ms")
    rel_over_15ms = analyze_counter_series(samples, "releases_over_15ms")
    rel_over_20ms = analyze_counter_series(samples, "releases_over_20ms")
    wake_late_max_ever_series = analyze_counter_series(samples, "wake_late_max_ever")
    late_series = analyze_counter_series(samples, "late_count")
    skipped_series = analyze_counter_series(samples, "skipped_releases")
    diag_series = analyze_counter_series(samples, "diag_dropped")
    wake_over_1ms = analyze_counter_series(samples, "wake_lateness_over_1ms")
    wake_over_5ms = analyze_counter_series(samples, "wake_lateness_over_5ms")
    wake_over_10ms = analyze_counter_series(samples, "wake_lateness_over_10ms")
    alloc_ops_series = analyze_counter_series(samples, "control_thread_alloc_ops")

    in_flight_raw = [get_nonneg_int(s.tick, "in_flight_release_us") for s in samples]
    in_flight_miss = sum(1 for v in in_flight_raw if v is None)
    in_flight_max_us = max_int_or_none([v for v in in_flight_raw if v is not None])

    first_ticks = since_boot_series.start
    last_ticks = since_boot_series.end
    release_count = (
        releases_total_series.verified_in_run_delta
        if releases_total_series.present_all and not releases_total_series.reset_detected
        else since_boot_series.verified_in_run_delta
    )

    first_late = late_series.start
    last_late = late_series.end
    late_delta = late_series.verified_in_run_delta
    late_any_nonzero = any((get_nonneg_int(s.tick, "late_count") or 0) > 0 for s in samples)

    has_skipped_releases = skipped_series.present_any
    skipped_start = skipped_series.start
    skipped_end = skipped_series.end
    skipped_delta = skipped_series.verified_in_run_delta
    skipped_coverage_gap = skipped_series.partial_missing or skipped_series.reset_detected

    has_diag_dropped = diag_series.present_any
    diag_dropped_start = diag_series.start
    diag_dropped_end = diag_series.end
    diag_dropped_delta = diag_series.verified_in_run_delta
    diag_samples_lost = bool(has_diag_dropped and diag_dropped_delta is not None and diag_dropped_delta > 0)
    diag_coverage_gap = (not has_diag_dropped) or diag_series.partial_missing or diag_series.reset_detected

    has_wake_over_1ms, wake_over_1ms_end, wake_over_1ms_delta = (
        wake_over_1ms.present_any,
        wake_over_1ms.end,
        wake_over_1ms.verified_in_run_delta,
    )
    has_wake_over_5ms, wake_over_5ms_end, wake_over_5ms_delta = (
        wake_over_5ms.present_any,
        wake_over_5ms.end,
        wake_over_5ms.verified_in_run_delta,
    )
    has_wake_over_10ms, wake_over_10ms_end, wake_over_10ms_delta = (
        wake_over_10ms.present_any,
        wake_over_10ms.end,
        wake_over_10ms.verified_in_run_delta,
    )

    # ---- Process / boot identity continuity, owner mode continuity, lock-free snapshot integrity ----
    instance_ids = [get_nonempty_str(s.tick, "instance_id") for s in samples]
    distinct_instance_ids = sorted({i for i in instance_ids if i is not None})
    instance_missing_count = sum(1 for i in instance_ids if i is None)
    instance_changed = len(distinct_instance_ids) > 1
    instance_id_latest = distinct_instance_ids[-1] if distinct_instance_ids else None

    boot_ids = [get_valid_boot_id(s.tick) for s in samples]
    distinct_boot_ids = sorted({b for b in boot_ids if b is not None})
    boot_id_missing_count = sum(1 for b in boot_ids if b is None)
    boot_id_changed = len(distinct_boot_ids) > 1
    boot_id_latest = distinct_boot_ids[-1] if distinct_boot_ids else None

    proc_starts = [get_positive_int(s.tick, "process_start_unix_ns") for s in samples]
    distinct_proc_starts = sorted({p for p in proc_starts if p is not None})
    proc_start_missing_count = sum(1 for p in proc_starts if p is None)
    proc_start_changed = len(distinct_proc_starts) > 1
    proc_start_latest = distinct_proc_starts[-1] if distinct_proc_starts else None

    process_identity_changed = instance_changed or boot_id_changed or proc_start_changed

    owner_modes = [get_nonempty_str(s.tick, "owner_mode") for s in samples]
    owner_invalid_count = sum(1 for m in owner_modes if m not in RECOGNIZED_OWNER_MODES)
    distinct_owner_modes = sorted({m for m in owner_modes if m is not None})
    owner_mode_changed = len(distinct_owner_modes) > 1
    owner_mode: str | None = None
    for m in reversed(owner_modes):
        if m is not None:
            owner_mode = m
            break

    lapped_flags = [get_strict_bool(s.tick, "stats_snapshot_lapped") for s in samples]
    lapped_missing_count = sum(1 for v in lapped_flags if v is None)
    snapshot_lapped_any = any(v is True for v in lapped_flags)

    loop_active_flags = [get_strict_bool(s.tick, "loop_active") for s in samples]
    loop_active_missing_count = sum(1 for v in loop_active_flags if v is None)
    loop_inactive_any = any(v is False for v in loop_active_flags)

    writer_drop_vals = [get_nonneg_int(s.tick, "stats_writer_drops") for s in samples]
    writer_drops_missing_count = sum(1 for v in writer_drop_vals if v is None)
    writer_drops_max = max_int_or_none([v for v in writer_drop_vals if v is not None])

    latest_since_marker: dict[str, Any] | None = None
    for s in reversed(samples):
        sm = s.tick.get("since_marker")
        if isinstance(sm, dict):
            latest_since_marker = sm
            break

    # ---- INVALIDity checks (fail-closed: never default missing to 0, never grant PASS) ----
    invalid_reasons: list[str] = list(load_run.grammar_errors)
    summary_stop_reason: str | None = None
    summary_stop_detail: str | None = None
    stop_reason_fault = False
    jsonl_capture_complete = load_run.source_format != "jsonl"
    terminal_safety_ok = load_run.source_format != "jsonl"
    if load_run.failed_polls:
        first_err = load_run.failed_polls[0].get("error", "poll failed")
        invalid_reasons.append(f"failed poll(s): {len(load_run.failed_polls)} poll failure(s) (first: {first_err})")
    if n_samples < 2:
        invalid_reasons.append(f"fewer than 2 tick samples ({n_samples}); cannot compute run delta")
    if load_run.source_format == "jsonl":
        jsonl_ok = True
        if not load_run.has_summary:
            jsonl_ok = False
            invalid_reasons.append("incomplete capture: missing terminal summary record")
        else:
            summary_stop_reason = get_nonempty_str(load_run.summary, "stop_reason")
            summary_stop_detail = get_nonempty_str(load_run.summary, "stop_detail")
            if summary_stop_reason == "completed":
                pass
            elif summary_stop_reason == "fault":
                stop_reason_fault = True
                jsonl_ok = False
            elif summary_stop_reason in ("interrupted", "error"):
                jsonl_ok = False
                detail_sfx = f" ({summary_stop_detail})" if summary_stop_detail else ""
                invalid_reasons.append(f"incomplete run: stop_reason='{summary_stop_reason}'{detail_sfx}")
            else:
                jsonl_ok = False
                invalid_reasons.append(
                    f"invalid or missing summary stop_reason: {load_run.summary.get('stop_reason')!r} "
                    f"(expected one of {VALID_STOP_REASONS})"
                )
        if samples[0].phase != "baseline":
            jsonl_ok = False
            invalid_reasons.append(
                f"incomplete capture: first sample phase is '{samples[0].phase}' (expected 'baseline')"
            )
        if load_run.final_sample_count > 1:
            jsonl_ok = False
            invalid_reasons.append(f"duplicate final samples ({load_run.final_sample_count}; expected exactly 1)")
        if samples[-1].phase != "final":
            jsonl_ok = False
            invalid_reasons.append(
                f"incomplete capture: last sample phase is '{samples[-1].phase}' (expected 'final')"
            )

        if load_run.has_summary and summary_stop_reason == "completed":
            # 1. Verify persisted idle-tail boundary (`idle_tail_start` -> `final`) with required
            #    wall duration >= max(1.0 s, 2 * period * 10) and monotonic `releases_total` progress.
            required_idle_tail_s = max(
                MIN_IDLE_TAIL_S, IDLE_TAIL_PERIOD_MULTIPLIER * (float(nom_period_us) / 1_000_000.0)
            )
            if load_run.idle_tail_start_count == 0:
                jsonl_ok = False
                invalid_reasons.append(
                    "incomplete capture: missing idle_tail_start boundary sample before final"
                )
            elif load_run.idle_tail_start_count > 1:
                jsonl_ok = False
                invalid_reasons.append(
                    f"duplicate idle_tail_start samples ({load_run.idle_tail_start_count}; expected exactly 1)"
                )
            elif n_samples < 3 or samples[-2].phase != "idle_tail_start":
                jsonl_ok = False
                invalid_reasons.append(
                    "incomplete capture: idle_tail_start sample must immediately precede final sample"
                )
            else:
                pre_idle = samples[-2]
                fin_idle = samples[-1]
                idle_dt_s: float | None = None
                if pre_idle.elapsed_s is not None and fin_idle.elapsed_s is not None:
                    idle_dt_s = fin_idle.elapsed_s - pre_idle.elapsed_s
                elif pre_idle.ts_unix is not None and fin_idle.ts_unix is not None:
                    idle_dt_s = fin_idle.ts_unix - pre_idle.ts_unix
                if idle_dt_s is None or idle_dt_s < required_idle_tail_s:
                    jsonl_ok = False
                    got_str = "missing" if idle_dt_s is None else f"{idle_dt_s:.3f}s"
                    invalid_reasons.append(
                        f"incomplete capture: idle tail duration {got_str} < required {required_idle_tail_s:.3f}s"
                    )
                pre_rel = get_nonneg_int(pre_idle.tick, "releases_total")
                fin_rel = get_nonneg_int(fin_idle.tick, "releases_total")
                if pre_rel is None or fin_rel is None:
                    jsonl_ok = False
                    invalid_reasons.append(
                        "incomplete capture: missing releases_total on idle_tail_start/final boundary"
                    )
                elif not process_identity_changed:
                    if (fin_rel - pre_rel) < MIN_IDLE_TAIL_RELEASES:
                        jsonl_ok = False
                        invalid_reasons.append(
                            f"incomplete capture: insufficient releases_total progress across idle tail "
                            f"({fin_rel - pre_rel} < {MIN_IDLE_TAIL_RELEASES})"
                        )
                    base_rel = get_nonneg_int(samples[0].tick, "releases_total")
                    if base_rel is not None and pre_rel <= base_rel:
                        jsonl_ok = False
                        invalid_reasons.append(
                            "incomplete capture: zero releases completed before idle tail "
                            f"(baseline={base_rel}, idle_tail_start={pre_rel})"
                        )

            # 2. Verify requested load configuration is declared and every enabled load ran.
            raw_enabled = load_run.meta.get("enabled_loads")
            if (
                not isinstance(raw_enabled, dict)
                or not raw_enabled
                or any(k not in KNOWN_LOAD_KEYS or not isinstance(v, bool) for k, v in raw_enabled.items())
            ):
                jsonl_ok = False
                invalid_reasons.append("incomplete capture: missing or malformed meta.enabled_loads")
            else:
                term_counters = (
                    load_run.summary.get("counters")
                    if isinstance(load_run.summary.get("counters"), dict)
                    else samples[-1].counters
                )
                unmet_loads = missing_requested_load_activity(raw_enabled, term_counters)
                if unmet_loads:
                    jsonl_ok = False
                    invalid_reasons.append(
                        f"incomplete capture: requested load(s) did not run ({', '.join(unmet_loads)})"
                    )

            # 3. Verify fresh post-idle terminal safety observation succeeded.
            ts_ver = get_strict_bool(load_run.summary, "terminal_safety_verified")
            ts_state = get_nonempty_str(load_run.summary, "safety_state")
            ts_err = load_run.summary.get("terminal_observation_error")
            ts_err_present = ts_err is not None and str(ts_err).strip() != ""
            if ts_ver is True and ts_state is not None and not ts_err_present:
                terminal_safety_ok = True
            else:
                terminal_safety_ok = False
                jsonl_ok = False
                invalid_reasons.append(
                    "unverified terminal safety state (missing or failed terminal observation)"
                )

        jsonl_capture_complete = jsonl_ok
    if instance_changed:
        invalid_reasons.append(f"daemon restart: instance_id changed ({' -> '.join(distinct_instance_ids)})")
    if boot_id_changed:
        invalid_reasons.append(f"host reboot: boot_id changed ({' -> '.join(distinct_boot_ids)})")
    if proc_start_changed:
        invalid_reasons.append(
            f"process restart: process_start_unix_ns changed ({' -> '.join(str(x) for x in distinct_proc_starts)})"
        )
    if owner_mode_changed:
        invalid_reasons.append(f"owner_mode changed mid-run ({' -> '.join(distinct_owner_modes)})")
    if loop_inactive_any:
        invalid_reasons.append("control loop inactive during capture (loop_active=false)")
    if snapshot_lapped_any:
        invalid_reasons.append("lock-free tick_stats snapshot lapped (stats_snapshot_lapped=true)")
    if writer_drops_max is not None and writer_drops_max > 0:
        invalid_reasons.append(f"lock-free tick_stats writer contention drops detected ({writer_drops_max})")

    # Check every monotonic counter for decreases (daemon restart / counter reset).
    monotonic_checks: list[tuple[str, CounterSeries]] = [
        ("releases_started", releases_started_series),
        ("releases_total", releases_total_series),
        ("since_boot_ticks", since_boot_series),
        ("release_us_max_ever", release_max_ever_series),
        ("releases_over_10ms", rel_over_10ms),
        ("releases_over_15ms", rel_over_15ms),
        ("releases_over_20ms", rel_over_20ms),
        ("wake_late_max_ever", wake_late_max_ever_series),
        ("skipped_releases", skipped_series),
        ("diag_dropped", diag_series),
        ("wake_lateness_over_1ms", wake_over_1ms),
        ("wake_lateness_over_5ms", wake_over_5ms),
        ("wake_lateness_over_10ms", wake_over_10ms),
        ("late_count", late_series),
        ("control_thread_alloc_ops", alloc_ops_series),
    ]
    for name, series in monotonic_checks:
        if series.reset_detected:
            invalid_reasons.append(f"counter decreased mid-run (restart/reset): {name}")

    # Release-start watermark consistency (all samples) & terminal completion coverage.
    terminal_in_flight_incomplete = False
    for s in samples:
        s_started = get_nonneg_int(s.tick, "releases_started")
        s_total = get_nonneg_int(s.tick, "releases_total")
        if s_started is not None and s_total is not None and s_total > s_started:
            terminal_in_flight_incomplete = True
            invalid_reasons.append(
                f"inconsistent release counters on sample {s.index}: releases_total ({s_total}) > releases_started ({s_started})"
            )
            break
    last_started = get_nonneg_int(samples[-1].tick, "releases_started")
    last_total = get_nonneg_int(samples[-1].tick, "releases_total")
    if last_started is not None and last_total is not None and last_started > last_total:
        terminal_in_flight_incomplete = True
        invalid_reasons.append(
            f"uncompleted in-flight release at final capture (releases_started={last_started} > releases_total={last_total})"
        )
    last_in_flight = get_nonneg_int(samples[-1].tick, "in_flight_release_us")
    if last_in_flight is not None and 0 < last_in_flight < RELEASE_BUDGET_US:
        terminal_in_flight_incomplete = True
        invalid_reasons.append(
            f"final sample captured during active in-flight release (in_flight_release_us={last_in_flight} us > 0)"
        )

    # Require positive release progress between first and last sample.
    if n_samples >= 2:
        if releases_total_series.present_all and not releases_total_series.reset_detected and not process_identity_changed:
            if releases_total_series.verified_in_run_delta == 0:
                invalid_reasons.append("zero releases completed between first and last sample (releases_total delta=0)")
        elif since_boot_series.present_all and not since_boot_series.reset_detected and not process_identity_changed:
            if since_boot_series.verified_in_run_delta == 0:
                invalid_reasons.append("zero releases completed between first and last sample (since_boot_ticks delta=0)")

    # Check window rollover gap when monotonic counters are missing.
    all_monotonic_present = all(
        s.present_all
        for s in (
            releases_started_series,
            releases_total_series,
            release_max_ever_series,
            rel_over_10ms,
            rel_over_15ms,
            rel_over_20ms,
            wake_late_max_ever_series,
            skipped_series,
        )
    )
    max_adjacent_gap_ticks = 0.0
    for i in range(1, n_samples):
        t0 = get_nonneg_int(samples[i - 1].tick, "since_boot_ticks")
        t1 = get_nonneg_int(samples[i].tick, "since_boot_ticks")
        if t0 is not None and t1 is not None and t1 >= t0:
            max_adjacent_gap_ticks = max(max_adjacent_gap_ticks, float(t1 - t0))
        r0 = get_nonneg_int(samples[i - 1].tick, "releases_total")
        r1 = get_nonneg_int(samples[i].tick, "releases_total")
        if r0 is not None and r1 is not None and r1 >= r0:
            max_adjacent_gap_ticks = max(max_adjacent_gap_ticks, float(r1 - r0))
        dt_s: float | None = None
        if samples[i - 1].elapsed_s is not None and samples[i].elapsed_s is not None:
            dt_s = samples[i].elapsed_s - samples[i - 1].elapsed_s
        elif samples[i - 1].ts_unix is not None and samples[i].ts_unix is not None:
            dt_s = samples[i].ts_unix - samples[i - 1].ts_unix
        if dt_s is not None and dt_s > 0.0:
            est_ticks = dt_s * (1_000_000.0 / float(nom_period_us))
            max_adjacent_gap_ticks = max(max_adjacent_gap_ticks, est_ticks)
    window_gap_without_monotonic = (not all_monotonic_present) and (max_adjacent_gap_ticks > TICK_WINDOW_CAPACITY)
    if window_gap_without_monotonic:
        invalid_reasons.append(
            f"sample gap ({max_adjacent_gap_ticks:.0f} ticks > window {TICK_WINDOW_CAPACITY}) "
            "with missing monotonic counters"
        )

    # Check every required field (strictly validating nested percentile keys and coherence metadata).
    missing_required_fields: list[str] = []
    if instance_missing_count > 0:
        missing_required_fields.append(f"instance_id ({instance_missing_count}/{n_samples})")
    if boot_id_missing_count > 0:
        missing_required_fields.append(f"boot_id ({boot_id_missing_count}/{n_samples})")
    if proc_start_missing_count > 0:
        missing_required_fields.append(f"process_start_unix_ns ({proc_start_missing_count}/{n_samples})")
    if owner_invalid_count > 0:
        missing_required_fields.append(f"owner_mode ({owner_invalid_count}/{n_samples})")
    if loop_active_missing_count > 0:
        missing_required_fields.append(f"loop_active ({loop_active_missing_count}/{n_samples})")
    if lapped_missing_count > 0:
        missing_required_fields.append(f"stats_snapshot_lapped ({lapped_missing_count}/{n_samples})")
    if writer_drops_missing_count > 0:
        missing_required_fields.append(f"stats_writer_drops ({writer_drops_missing_count}/{n_samples})")
    if in_flight_miss > 0:
        missing_required_fields.append(f"in_flight_release_us ({in_flight_miss}/{n_samples})")
    for key, series in (
        ("releases_started", releases_started_series),
        ("releases_total", releases_total_series),
        ("release_us_max_ever", release_max_ever_series),
        ("releases_over_10ms", rel_over_10ms),
        ("releases_over_15ms", rel_over_15ms),
        ("releases_over_20ms", rel_over_20ms),
        ("wake_late_max_ever", wake_late_max_ever_series),
        ("skipped_releases", skipped_series),
        ("since_boot_ticks", since_boot_series),
        ("diag_dropped", diag_series),
    ):
        if series.missing_count > 0:
            missing_required_fields.append(f"{key} ({series.missing_count}/{n_samples})")
    for key, m_p50, m_p99, m_max in (
        ("release_us", rel_p50_miss, rel_p99_miss, rel_max_miss),
        ("wake_lateness_us", wake_p50_miss, wake_p99_miss, wake_max_miss),
        ("tick_us", tick_p50_miss, tick_p99_miss, tick_max_miss),
        ("period_us", period_p50_miss, period_p99_miss, period_max_miss),
    ):
        worst_miss = max(m_p50, m_p99, m_max)
        if worst_miss > 0:
            missing_required_fields.append(f"{key} ({worst_miss}/{n_samples})")
    for key, m_max in (
        ("lock_us", lock_max_miss),
        ("wait_us", wait_max_miss),
        ("reader_hold_us", reader_hold_miss),
        ("writer_hold_us", writer_hold_miss),
    ):
        if m_max > 0:
            missing_required_fields.append(f"{key}.max ({m_max}/{n_samples})")
    if missing_required_fields:
        invalid_reasons.append("missing/malformed required field(s): " + ", ".join(missing_required_fields))

    # Detect contradictory release telemetry (e.g. >= 20 ms peak while releases_over_20ms == 0).
    contradiction_seen = False
    for s in samples:
        c20 = get_nonneg_int(s.tick, "releases_over_20ms")
        rmax = get_pct(s.tick, "release_us", "max")
        rever = get_nonneg_int(s.tick, "release_us_max_ever")
        if c20 == 0 and (
            (rmax is not None and rmax >= RELEASE_BUDGET_US)
            or (rever is not None and rever >= RELEASE_BUDGET_US)
        ):
            if not contradiction_seen:
                invalid_reasons.append(
                    "contradictory telemetry: release_us >= 20 ms observed while releases_over_20ms == 0"
                )
                contradiction_seen = True
        if rever is not None and rmax is not None and rever < rmax:
            if not contradiction_seen:
                invalid_reasons.append(
                    f"contradictory telemetry: release_us_max_ever ({rever}) < release_us.max ({rmax})"
                )
                contradiction_seen = True

    # Optional external telemetry fields (never default missing to 0).
    enq_vals: list[int] = []
    for s in samples:
        if "enqueue_to_wire_us" not in s.tick:
            continue
        raw_e = s.tick.get("enqueue_to_wire_us")
        v = get_nonneg_int(raw_e, "max") if isinstance(raw_e, dict) else get_nonneg_int(s.tick, "enqueue_to_wire_us")
        if v is not None:
            enq_vals.append(v)
    has_enqueue_to_wire = len(enq_vals) == n_samples
    enqueue_to_wire_max_us = max_int_or_none(enq_vals)

    lock_attempt_vals = [
        v for s in samples if (v := get_nonneg_int(s.tick, "control_thread_lock_attempts")) is not None
    ]
    has_lock_attempts = len(lock_attempt_vals) == n_samples
    lock_attempts_max = max_int_or_none(lock_attempt_vals)

    # Lock holders & allocation counts observed on max_release / max_tick / max_lock (never default missing to 0).
    breakdowns = collect_unique_breakdowns(samples)
    lock_wait_holders = [
        bd["waited_for"]
        for _, _, _, bd in breakdowns
        if isinstance(bd.get("waited_for"), dict)
    ]
    bd_alloc_vals = [
        v for _, _, _, bd in breakdowns if (v := get_nonneg_int(bd, "lock_alloc_calls")) is not None
    ]
    bd_alloc_malformed = len(breakdowns) - len(bd_alloc_vals)
    alloc_calls_max = max_int_or_none(bd_alloc_vals)

    # RX age observed via load.py polling / WS.
    rx_age_max_ms = max_or_none(
        [float(s.rx_age_ms["max"]) for s in samples if s.rx_age_ms and s.rx_age_ms.get("max") is not None]
    )
    rx_age_p99_ms = max_or_none(
        [float(s.rx_age_ms["p99"]) for s in samples if s.rx_age_ms and s.rx_age_ms.get("p99") is not None]
    )

    # Per-phase lock percentiles across all samples (strict non-defaulting).
    phase_stats: dict[str, dict[str, float | None]] = {}
    for pk in LOCK_PHASE_KEYS:
        p50s = [
            float(v)
            for s in samples
            if isinstance(s.tick.get("lock_phases"), dict)
            and (v := get_pct(s.tick["lock_phases"], pk, "p50")) is not None
        ]
        p99s = [
            float(v)
            for s in samples
            if isinstance(s.tick.get("lock_phases"), dict)
            and (v := get_pct(s.tick["lock_phases"], pk, "p99")) is not None
        ]
        maxs = [
            float(v)
            for s in samples
            if isinstance(s.tick.get("lock_phases"), dict)
            and (v := get_pct(s.tick["lock_phases"], pk, "max")) is not None
        ]
        phase_stats[pk] = {
            "p50_ms": us_to_ms(median_or_none(p50s)),
            "p99_ms": us_to_ms(max_or_none(p99s)),
            "max_ms": us_to_ms(max_or_none(maxs)),
        }

    # Same-process high-water-mark increases prove an in-run peak even after 1000-tick rollover.
    in_run_release_ever_us: int | None = None
    in_run_wake_ever_us: int | None = None
    prev_ident_hwm: tuple[str | None, str | None, int | None] | None = None
    prev_rel_ever: int | None = None
    prev_wake_ever: int | None = None
    for s in samples:
        cur_ident = sample_process_identity(s.tick)
        same_inst = (prev_ident_hwm is None) or (not identities_conflict(prev_ident_hwm, cur_ident))
        cur_rel_ever = get_nonneg_int(s.tick, "release_us_max_ever")
        cur_wake_ever = get_nonneg_int(s.tick, "wake_late_max_ever")
        if same_inst and prev_rel_ever is not None and cur_rel_ever is not None and cur_rel_ever > prev_rel_ever:
            in_run_release_ever_us = max(in_run_release_ever_us or 0, cur_rel_ever)
        if same_inst and prev_wake_ever is not None and cur_wake_ever is not None and cur_wake_ever > prev_wake_ever:
            in_run_wake_ever_us = max(in_run_wake_ever_us or 0, cur_wake_ever)
        if cur_rel_ever is not None:
            prev_rel_ever = cur_rel_ever
        if cur_wake_ever is not None:
            prev_wake_ever = cur_wake_ever
        prev_ident_hwm = cur_ident

    # Exact baseline-aligned marker check: marker must be active with releases_total_delta == 0
    # on samples[0] AND active with the SAME marker_id and releases_total_delta == run_releases_delta
    # on samples[-1]. Late or early markers never mask earlier in-run wake peaks.
    aligned_marker_wake_max_us: int | None = None
    if n_samples >= 2 and not process_identity_changed and not releases_total_series.reset_detected:
        run_rel_delta = releases_total_series.verified_in_run_delta
        sm0 = samples[0].tick.get("since_marker")
        sm_end = samples[-1].tick.get("since_marker")
        if (
            isinstance(sm0, dict)
            and isinstance(sm_end, dict)
            and sm0.get("status") == "active"
            and sm_end.get("status") == "active"
            and (m0 := get_positive_int(sm0, "marker_id")) is not None
            and get_positive_int(sm_end, "marker_id") == m0
            and get_nonneg_int(sm0, "releases_total_delta") == 0
            and run_rel_delta is not None
            and run_rel_delta > 0
            and get_nonneg_int(sm_end, "releases_total_delta") == run_rel_delta
            and (wm := get_nonneg_int(sm_end, "wake_late_max")) is not None
        ):
            aligned_marker_wake_max_us = wm

    # Conservative MIT wake attribution:
    # - Hard MIT FAIL triggers ONLY from provably in-run wake peaks (same-process wake_late_max_ever
    #   increase, baseline-aligned since_marker.wake_late_max, or strictly post-rollover windows
    #   whose 1000-tick span lies entirely after samples[0]).
    # - PASS requires that the conservative upper bound across ALL post-baseline windows (and any
    #   rollover-masked wake_late_max_ever.start when >1000-tick gap occurs without aligned marker)
    #   clears < 45 ms; otherwise MIT reports INCOMPLETE (never false FAIL, never false PASS).
    provable_wake_vals: list[int] = []
    if in_run_wake_ever_us is not None:
        provable_wake_vals.append(in_run_wake_ever_us)
    if aligned_marker_wake_max_us is not None:
        provable_wake_vals.append(aligned_marker_wake_max_us)
    upper_wake_vals: list[int] = list(provable_wake_vals)

    first_ident_w = sample_process_identity(samples[0].tick)
    first_ticks_w = get_nonneg_int(samples[0].tick, "since_boot_ticks")
    if n_samples >= 2:
        for s in samples[1:]:
            if identities_conflict(first_ident_w, sample_process_identity(s.tick)):
                break
            cur_w = get_pct(s.tick, "wake_lateness_us", "max")
            if cur_w is None:
                continue
            if aligned_marker_wake_max_us is None:
                upper_wake_vals.append(cur_w)
            cur_t = get_nonneg_int(s.tick, "since_boot_ticks")
            if first_ticks_w is not None and cur_t is not None and (cur_t - first_ticks_w) >= TICK_WINDOW_CAPACITY:
                provable_wake_vals.append(cur_w)
    elif wake_lateness_max_us is not None:
        upper_wake_vals.append(wake_lateness_max_us)

    provable_in_run_wake_us = max_int_or_none(provable_wake_vals)
    wake_rollover_masked = (
        aligned_marker_wake_max_us is None
        and max_adjacent_gap_ticks > TICK_WINDOW_CAPACITY
        and wake_late_max_ever_series.start is not None
        and (provable_in_run_wake_us is None or wake_late_max_ever_series.start > provable_in_run_wake_us)
    )
    if wake_rollover_masked and wake_late_max_ever_series.start is not None:
        upper_wake_vals.append(wake_late_max_ever_series.start)

    upper_bound_wake_us = (
        aligned_marker_wake_max_us
        if aligned_marker_wake_max_us is not None
        else max_int_or_none(upper_wake_vals)
    )

    # Wake lateness display value & MIT budget.
    if upper_bound_wake_us is not None:
        wake_extra_ms: float | None = upper_bound_wake_us / 1000.0
        if aligned_marker_wake_max_us is not None:
            wake_extra_label = "wake lateness (aligned marker)"
        elif provable_in_run_wake_us is not None and provable_in_run_wake_us >= upper_bound_wake_us:
            wake_extra_label = (
                "wake lateness (in-run HWM)"
                if (in_run_wake_ever_us is not None and (wake_lateness_max_us is None or in_run_wake_ever_us > wake_lateness_max_us))
                else "wake lateness"
            )
        else:
            wake_extra_label = "wake lateness"
    elif period_max_us is not None:
        wake_extra_ms = max(0.0, (period_max_us - nom_period_us) / 1000.0)
        wake_extra_label = "sched overrun proxy"
    else:
        wake_extra_ms = None
        wake_extra_label = "missing"

    bd_start_to_tx_vals = [
        (
            (get_nonneg_int(bd, "wait_us") or 0)
            + (get_nonneg_int(bd, "pre_us") or 0)
            + (get_nonneg_int(bd, "rx_us") or 0)
            + (get_nonneg_int(bd, "control_us") or 0)
            + (get_nonneg_int(bd, "tx_us") or 0)
        )
        / 1000.0
        for _, _, _, bd in breakdowns
    ]
    fallback_rel_us = max_int_or_none([v for v in (release_us_max_us, in_run_release_ever_us) if v is not None])
    start_to_tx_ms: float | None = (
        max(bd_start_to_tx_vals)
        if bd_start_to_tx_vals
        else (us_to_ms(fallback_rel_us) if fallback_rel_us is not None else us_to_ms(tick_max_us))
    )
    enq_ms = (enqueue_to_wire_max_us / 1000.0) if enqueue_to_wire_max_us is not None else 0.0
    if wake_extra_ms is not None and start_to_tx_ms is not None:
        worst_extra_ms: float | None = wake_extra_ms + start_to_tx_ms + enq_ms
        mit_total_ms: float | None = MIT_PERIOD_MS + worst_extra_ms + MIT_MARGIN_MS
    else:
        worst_extra_ms = None
        mit_total_ms = None

    if provable_in_run_wake_us is not None and start_to_tx_ms is not None:
        provable_mit_total_ms: float | None = (
            MIT_PERIOD_MS + (provable_in_run_wake_us / 1000.0) + start_to_tx_ms + enq_ms + MIT_MARGIN_MS
        )
    elif not has_wake_lateness and mit_total_ms is not None:
        provable_mit_total_ms = mit_total_ms
    else:
        provable_mit_total_ms = None

    # Control thread info (latest non-null).
    ctl_sched: dict[str, Any] = {}
    for s in reversed(samples):
        if isinstance(s.tick.get("control_thread"), dict):
            ctl_sched = s.tick["control_thread"]
            break

    faults_seen = sorted({s.fault for s in samples if s.fault})
    for s in samples:
        if s.safety_state in FAULT_SAFETY_STATES:
            tag = f"safety_state={s.safety_state}"
            if tag not in faults_seen:
                faults_seen.append(tag)
    sum_safety = get_nonempty_str(load_run.summary, "safety_state")
    if sum_safety in FAULT_SAFETY_STATES:
        tag = f"safety_state={sum_safety}"
        if tag not in faults_seen:
            faults_seen.append(tag)
    if load_run.summary.get("fault") and load_run.summary["fault"] not in faults_seen:
        faults_seen.append(str(load_run.summary["fault"]))
    if stop_reason_fault and not faults_seen:
        faults_seen.append(
            f"stop_reason=fault ({summary_stop_detail})" if summary_stop_detail else "stop_reason=fault"
        )

    # ---- Gate Evaluation ----
    # 1. Release latency (< 20 ms): gated on `release_us`, monotonic `releases_over_20ms` delta,
    #    in-run `release_us_max_ever` increases, attributable window `release_us.max`, and live
    #    `in_flight_release_us` (always checked even when `releases_over_20ms` is present).
    release_over_20_delta = rel_over_20ms.verified_in_run_delta
    release_over_15_delta = rel_over_15ms.verified_in_run_delta
    release_over_10_delta = rel_over_10ms.verified_in_run_delta
    in_flight_breach = in_flight_max_us is not None and in_flight_max_us >= RELEASE_BUDGET_US
    release_in_run_breach = False
    if release_over_20_delta is not None and release_over_20_delta > 0:
        release_in_run_breach = True
    if in_flight_breach:
        release_in_run_breach = True
    if in_run_release_ever_us is not None and in_run_release_ever_us >= RELEASE_BUDGET_US:
        release_in_run_breach = True
    if has_release_us and n_samples >= 2:
        first_rel = get_pct(samples[0].tick, "release_us", "max")
        first_t = get_nonneg_int(samples[0].tick, "since_boot_ticks")
        first_ident = sample_process_identity(samples[0].tick)
        for s in samples[1:]:
            cur_ident = sample_process_identity(s.tick)
            if identities_conflict(first_ident, cur_ident):
                break
            cur_rel = get_pct(s.tick, "release_us", "max")
            if cur_rel is not None and cur_rel >= RELEASE_BUDGET_US:
                if first_rel is not None and cur_rel > first_rel:
                    release_in_run_breach = True
                    break
                mr = s.tick.get("max_release")
                cur_t = get_nonneg_int(s.tick, "since_boot_ticks")
                if (
                    isinstance(mr, dict)
                    and (ta := get_nonneg_int(mr, "ticks_ago")) is not None
                    and first_t is not None
                    and cur_t is not None
                    and cur_t > first_t
                    and ta < (cur_t - first_t)
                ):
                    release_in_run_breach = True
                    break

    release_fields_valid = (
        has_release_us
        and rel_max_miss == 0
        and rel_p99_miss == 0
        and rel_p50_miss == 0
        and rel_over_20ms.present_all
        and rel_over_15ms.present_all
        and rel_over_10ms.present_all
        and release_max_ever_series.present_all
        and releases_started_series.present_all
        and releases_total_series.present_all
        and in_flight_miss == 0
        and not rel_over_20ms.reset_detected
        and not releases_started_series.reset_detected
        and not releases_total_series.reset_detected
        and not release_max_ever_series.reset_detected
        and not process_identity_changed
        and instance_missing_count == 0
        and boot_id_missing_count == 0
        and proc_start_missing_count == 0
        and not owner_mode_changed
        and owner_invalid_count == 0
        and loop_active_missing_count == 0
        and not loop_inactive_any
        and not load_run.failed_polls
        and not load_run.grammar_errors
        and lapped_missing_count == 0
        and not snapshot_lapped_any
        and writer_drops_missing_count == 0
        and (writer_drops_max is not None and writer_drops_max == 0)
        and not terminal_in_flight_incomplete
        and not contradiction_seen
        and jsonl_capture_complete
        and n_samples >= 2
        and release_over_20_delta is not None
        and (releases_total_series.verified_in_run_delta or 0) > 0
    )
    release_pass = not release_in_run_breach
    if release_in_run_breach:
        release_status = "FAIL"
    elif not release_fields_valid:
        release_status = "INVALID"
    else:
        release_status = "PASS"

    if has_release_us:
        release_detail = (
            f"`release_us.max` = **{fmt_ms(us_to_ms(release_us_max_us))} ms** "
            f"(p99 max {fmt_ms(us_to_ms(release_us_p99_max_us))} ms, p50 med {fmt_ms(us_to_ms(release_us_p50_med_us))} ms), "
            f"`release_us_max_ever`={fmt_ms(us_to_ms(release_max_ever_series.end))} ms, "
            f"`releases_over_20ms` Δ={fmt_int(release_over_20_delta)} "
            f"(`>=15ms` Δ={fmt_int(release_over_15_delta)}, `>=10ms` Δ={fmt_int(release_over_10_delta)})"
            + (f", `in_flight_release_us` max={fmt_ms(us_to_ms(in_flight_max_us))} ms" if in_flight_breach else "")
        )
    else:
        release_detail = (
            f"`release_us` missing (proxy `tick_us.max` = **{fmt_ms(us_to_ms(tick_max_us))} ms**, "
            f"p99 max {fmt_ms(us_to_ms(tick_p99_max_us))} ms, p50 med {fmt_ms(us_to_ms(tick_p50_med_us))} ms)"
        )

    # 2. Deadlines (`skipped_releases` delta == 0)
    if has_skipped_releases:
        skipped_breach = skipped_delta is not None and skipped_delta > 0
        skipped_valid = (
            skipped_series.present_all
            and not skipped_series.reset_detected
            and not process_identity_changed
            and instance_missing_count == 0
            and boot_id_missing_count == 0
            and proc_start_missing_count == 0
            and not owner_mode_changed
            and owner_invalid_count == 0
            and loop_active_missing_count == 0
            and not loop_inactive_any
            and lapped_missing_count == 0
            and not snapshot_lapped_any
            and writer_drops_missing_count == 0
            and (writer_drops_max is not None and writer_drops_max == 0)
            and not load_run.failed_polls
            and not load_run.grammar_errors
            and not terminal_in_flight_incomplete
            and jsonl_capture_complete
            and n_samples >= 2
            and skipped_delta is not None
        )
        skipped_pass = not skipped_breach
        if skipped_breach:
            skipped_status = "FAIL"
        elif not skipped_valid:
            skipped_status = "INVALID"
        else:
            skipped_status = "PASS"
        diag_note = f", diag_dropped delta={fmt_int(diag_dropped_delta)}" if has_diag_dropped else ""
        skipped_detail = (
            f"skipped_releases delta={fmt_int(skipped_delta)} (start={fmt_int(skipped_start)}, end={fmt_int(skipped_end)}), "
            f"late_count delta={fmt_int(late_delta)}{diag_note}"
        )
    else:
        proxy_ok = (
            (late_delta == 0 if late_delta is not None else not late_any_nonzero)
            and (period_max_us is not None and period_max_us <= (2 * nom_period_us))
        )
        skipped_pass = True  # Legacy proxy is not an authoritative hard-fail; missing field drives INVALID.
        skipped_status = "PASS (proxy)" if proxy_ok else "INVALID (proxy breach)"
        skipped_detail = (
            f"late_count delta={fmt_int(late_delta)} (start={fmt_int(first_late)}, end={fmt_int(last_late)}), "
            f"period_us.max={fmt_ms(us_to_ms(period_max_us))} ms "
            f"(missing required field `skipped_releases`)"
        )

    # 3. Lock gate (never defaults missing wait_us/reader_hold/writer_hold to 0).
    lock_fields_present = (
        wait_max_miss == 0
        and reader_hold_miss == 0
        and writer_hold_miss == 0
        and wait_max_us is not None
        and reader_hold_max_us is not None
        and writer_hold_max_us is not None
    )
    lock_wait_observed = (
        bool(lock_wait_holders)
        or (wait_max_us is not None and wait_max_us > 0)
        or (writer_hold_max_us is not None and writer_hold_max_us > 0)
        or (reader_hold_max_us is not None and reader_hold_max_us > 0)
    )
    if has_lock_attempts and lock_attempts_max is not None:
        lock_breach = (lock_attempts_max > 0) or lock_wait_observed
        lock_pass = not lock_breach
        if lock_breach:
            lock_status = "FAIL"
        elif not lock_fields_present:
            lock_status = "INVALID"
        else:
            lock_status = "PASS"
        lock_detail = f"control_thread_lock_attempts={lock_attempts_max}, wait_us.max={fmt_ms(us_to_ms(wait_max_us))} ms"
    else:
        lock_pass = not lock_wait_observed
        if lock_wait_observed:
            worst_h = max(lock_wait_holders, key=lambda h: h.get("hold_us", 0)) if lock_wait_holders else None
            holder_note = (
                f", worst waited_for={worst_h.get('kind')}@{worst_h.get('location')} ({fmt_ms(us_to_ms(worst_h.get('hold_us')))} ms)"
                if worst_h
                else ""
            )
            lock_status = "FAIL"
            lock_detail = (
                f"wait_us.max={fmt_ms(us_to_ms(wait_max_us))} ms, "
                f"writer_hold.max={fmt_ms(us_to_ms(writer_hold_max_us))} ms, "
                f"reader_hold.max={fmt_ms(us_to_ms(reader_hold_max_us))} ms{holder_note}"
            )
        elif not lock_fields_present:
            lock_status = "INVALID"
            lock_detail = (
                f"missing required lock timing field(s) "
                f"(wait_us.max={fmt_ms(us_to_ms(wait_max_us))} ms, "
                f"reader_hold.max={fmt_ms(us_to_ms(reader_hold_max_us))} ms, "
                f"writer_hold.max={fmt_ms(us_to_ms(writer_hold_max_us))} ms)"
            )
        else:
            lock_status = "PASS (proxy)"
            lock_detail = (
                "wait_us.max=0.00 ms, waited_for=none, reader/writer_hold=0 "
                "(gap: SupervisorLock::control_thread_attempts not exposed; confirm via B1 cargo test)"
            )

    alloc_timing_confirmed = (
        bool(samples) and all(s.tick.get("alloc_timing_enabled") is True for s in samples)
    )
    if not alloc_timing_confirmed and devstat is not None:
        alloc_timing_confirmed = any("ARMD_ALLOC_TIMING=1" in l for l in devstat.armd_version)
    alloc_ops_delta = alloc_ops_series.verified_in_run_delta
    has_authoritative_alloc = (
        alloc_ops_series.present_all
        and not alloc_ops_series.reset_detected
        and alloc_ops_delta is not None
        and alloc_timing_confirmed
    )
    alloc_breach = (
        (alloc_ops_delta is not None and alloc_ops_delta > 0)
        or (alloc_calls_max is not None and alloc_calls_max > 0)
    )
    if alloc_breach:
        alloc_pass = False
        alloc_evidence_complete = True
        alloc_status = "FAIL"
        alloc_detail = (
            f"control_thread_alloc_ops Δ={fmt_int(alloc_ops_delta)}, "
            f"lock_alloc_calls max={fmt_int(alloc_calls_max)} on max_release/max_tick/max_lock"
        )
    elif has_authoritative_alloc:
        alloc_pass = True
        alloc_evidence_complete = True
        alloc_status = "PASS"
        alloc_detail = (
            f"control_thread_alloc_ops Δ=0 (ARMD_ALLOC_TIMING=1 confirmed=True"
            + (f", breakdown lock_alloc_calls max={alloc_calls_max}" if alloc_calls_max is not None else "")
            + ")"
        )
    elif bd_alloc_vals and bd_alloc_malformed == 0:
        alloc_pass = True
        alloc_evidence_complete = False
        alloc_status = "PASS (proxy)"
        alloc_detail = (
            f"lock_alloc_calls=0 on peak breakdown(s) only "
            f"(gap: run-scoped control_thread_alloc_ops not exposed; "
            f"ARMD_ALLOC_TIMING=1 confirmed={alloc_timing_confirmed})"
        )
    else:
        alloc_pass = True
        alloc_evidence_complete = False
        alloc_status = "INCOMPLETE (missing/malformed allocation evidence)"
        alloc_detail = (
            f"no authoritative run-scoped allocation counter and "
            f"{'no breakdowns present' if not breakdowns else f'{bd_alloc_malformed}/{len(breakdowns)} malformed lock_alloc_calls'} "
            f"(ARMD_ALLOC_TIMING=1 confirmed={alloc_timing_confirmed})"
        )

    mit_observable_pass = True if provable_mit_total_ms is None else (provable_mit_total_ms < MIT_WATCHDOG_MS)
    mit_upper_ambiguous = (
        aligned_marker_wake_max_us is None
        and mit_observable_pass
        and mit_total_ms is not None
        and mit_total_ms >= MIT_WATCHDOG_MS
    )
    mit_gaps: list[str] = []
    if diag_samples_lost:
        mit_gaps.append(f"diag_dropped delta={diag_dropped_delta} lost tick/wake samples")
    if not has_diag_dropped:
        mit_gaps.append("diag_dropped not exposed")
    elif diag_series.partial_missing:
        mit_gaps.append("diag_dropped missing on some samples")
    if diag_series.reset_detected:
        mit_gaps.append("diag_dropped counter reset mid-run")
    if not has_wake_lateness:
        mit_gaps.append("wake_lateness_us not exposed (using period proxy)")
    elif wake_lateness_partial:
        mit_gaps.append("wake_lateness_us missing on some samples")
    if wake_rollover_masked and (mit_total_ms is None or mit_total_ms >= MIT_WATCHDOG_MS):
        mit_gaps.append(
            "pre-run wake_late_max_ever masks in-run wake peak across >1000-tick sample gap without baseline-aligned since_marker"
        )
    if mit_upper_ambiguous:
        mit_gaps.append(
            "unscoped rolling wake_lateness_us / pre-run wake_late_max_ever upper bound reaches MIT watchdog budget without baseline-aligned since_marker"
        )
    if not has_enqueue_to_wire:
        mit_gaps.append("enqueue-to-wire CAN latency not exposed")
    if not mit_observable_pass:
        mit_status = "FAIL"
    elif mit_total_ms is None:
        mit_status = "INVALID"
    elif diag_samples_lost:
        mit_status = "INCOMPLETE (diag_dropped > 0)"
    elif not mit_gaps:
        mit_status = "PASS"
    else:
        mit_status = "INCOMPLETE (observable PASS)"
    if mit_total_ms is not None and wake_extra_ms is not None and start_to_tx_ms is not None:
        mit_detail = (
            f"20.00 + {wake_extra_ms:.2f} ({wake_extra_label}) + {start_to_tx_ms:.2f} (start->TX) "
            f"+ {enq_ms:.2f} (enqueue->wire) + 5.00 = {mit_total_ms:.2f} ms vs 45.00 ms"
            + (f" (gap: {'; '.join(mit_gaps)})" if mit_gaps else "")
        )
    else:
        mit_detail = f"n/a (missing timing fields; gap: {'; '.join(mit_gaps)})"

    fault_pass = len(faults_seen) == 0
    if not fault_pass:
        fault_status = "FAIL"
        fault_detail = "; ".join(faults_seen)
    elif load_run.source_format == "jsonl" and not terminal_safety_ok:
        fault_status = "INVALID"
        fault_detail = "unverified terminal safety observation"
    else:
        fault_status = "PASS"
        fault_detail = "none"

    hard_fail = (
        (not release_pass)
        or (not skipped_pass)
        or (not lock_pass)
        or (not alloc_pass)
        or (not mit_observable_pass)
        or (not fault_pass)
    )
    any_gaps = (
        diag_samples_lost
        or (not has_lock_attempts)
        or (not has_enqueue_to_wire)
        or (not alloc_evidence_complete)
        or bool(mit_gaps)
    )
    continuity_reasons = [
        r for r in invalid_reasons if r.startswith(("daemon restart:", "host reboot:", "process restart:"))
    ]
    if load_run.source_format == "tickload_json":
        overall_verdict = "DIAGNOSTIC ONLY (legacy tickload.json; cannot certify Stage 3 §6 gate)"
    elif process_identity_changed:
        overall_verdict = f"INVALID ({'; '.join(continuity_reasons[:3])})"
    elif hard_fail:
        overall_verdict = "FAIL"
    elif invalid_reasons:
        overall_verdict = f"INVALID ({'; '.join(invalid_reasons[:3])})"
    elif diag_samples_lost:
        overall_verdict = f"INCOMPLETE (diagnostic samples dropped: diag_dropped delta={diag_dropped_delta})"
    elif any_gaps:
        overall_verdict = "INCOMPLETE (all observable criteria PASS; diagnostic gaps remain)"
    else:
        overall_verdict = "PASS"

    # ---- Build Markdown ----
    out: list[str] = []
    run_id = load_run.meta.get("run_id") or (devstat.meta.get("run_id") if devstat else "unknown")
    out.append(f"# SG2002 Real-Time Qualification Report (`{run_id}`)")
    out.append("")
    out.append(f"**Overall Verdict: {overall_verdict}**")
    out.append("")
    if invalid_reasons:
        out.append("### Validity Findings")
        out.append("")
        for r in invalid_reasons:
            out.append(f"- **INVALID**: {r}")
        out.append("")

    # 1. Gate Criteria Table
    out.append("## 1. Stage 3 §6 Gate Verdict")
    out.append("")
    out.append("| Criterion | Target | Measured / Evidence | Status |")
    out.append("|---|---|---|---|")
    out.append(
        f"| Every release (`release_us`) | `< {RELEASE_BUDGET_MS:.0f} ms` | "
        f"{release_detail} | "
        f"{release_status} |"
    )
    out.append(f"| Zero skipped deadlines | `0` | {skipped_detail} | {skipped_status} |")
    out.append(f"| Lock gate (zero supervisor-lock attempts) | `0` | {lock_detail} | {lock_status} |")
    out.append(f"| Allocation gate (zero release allocs) | `0` | {alloc_detail} | {alloc_status} |")
    out.append(f"| MIT watchdog budget | `20 + extra + 5 < 45 ms` | {mit_detail} | {mit_status} |")
    out.append(
        f"| Fault / stop trace check | `0 faults` | "
        f"{fault_detail} | {fault_status} |"
    )
    out.append("")

    # 2. Run & Environment Summary
    out.append("## 2. Run & Environment Summary")
    out.append("")
    armd_ver = (load_run.meta.get("armd") or {}).get("armd_version") or "unknown"
    if armd_ver == "unknown" and devstat:
        for l in devstat.armd_version:
            if l.startswith("Version:"):
                armd_ver = l.split(":", 1)[1].strip()
                break
    out.append(
        f"- **Input format**: `{load_run.source_format}` ({len(samples)} tick-stats snapshots, "
        f"{len(load_run.failed_polls)} failed poll(s), nominal period `{nom_period_ms:.1f} ms`)"
    )
    if load_run.meta.get("target"):
        out.append(f"- **Target**: `{load_run.meta['target']}` | **armd version**: `{armd_ver}`")
    if instance_id_latest is not None:
        out.append(f"- **Instance ID**: `{instance_id_latest}` (distinct={len(distinct_instance_ids)})")
    if boot_id_latest is not None or proc_start_latest is not None:
        out.append(
            f"- **Process boot identity**: `boot_id={boot_id_latest or 'missing'}` (distinct={len(distinct_boot_ids)}), "
            f"`process_start_unix_ns={fmt_int(proc_start_latest)}` (distinct={len(distinct_proc_starts)})"
        )
    if release_count is not None:
        out.append(
            f"- **Releases covered**: `{release_count}` "
            f"(`releases_total`: {fmt_int(releases_total_series.start)} -> {fmt_int(releases_total_series.end)}, "
            f"`since_boot_ticks`: {fmt_int(first_ticks)} -> {fmt_int(last_ticks)})"
        )
    if owner_mode is not None:
        out.append(f"- **Owner mode**: `{owner_mode}`")
    executor_wakes_series = analyze_counter_series(samples, "executor_wakes")
    if executor_wakes_series.present_any:
        out.append(
            f"- **Executor sub-release wakes (`executor_wakes`)**: delta=`{fmt_int(executor_wakes_series.verified_in_run_delta)}` "
            f"(start={fmt_int(executor_wakes_series.start)}, end={fmt_int(executor_wakes_series.end)})"
        )
    if has_lock_attempts or alloc_ops_series.present_any:
        exec_parts: list[str] = []
        if has_lock_attempts:
            exec_parts.append(f"`control_thread_lock_attempts`={fmt_int(lock_attempts_max)}")
        if alloc_ops_series.present_any:
            exec_parts.append(
                f"`control_thread_alloc_ops` Δ={fmt_int(alloc_ops_delta)} "
                f"(ARMD_ALLOC_TIMING=1 confirmed={alloc_timing_confirmed})"
            )
        out.append("- **Executor lock/alloc telemetry**: " + ", ".join(exec_parts))
    if samples:
        last_tick = samples[-1].tick
        obs_cache = last_tick.get("observation_cache")
        term_cap = last_tick.get("terminal_capture")
        extra_exec_parts: list[str] = []
        if isinstance(obs_cache, dict) and get_nonneg_int(obs_cache, "tick_frees") is not None:
            extra_exec_parts.append(f"`observation_cache.tick_frees`={obs_cache['tick_frees']}")
        if isinstance(term_cap, dict) and get_nonneg_int(term_cap, "captured") is not None:
            extra_exec_parts.append(
                f"`terminal_capture` captured={term_cap.get('captured')}, dropped={term_cap.get('dropped_full', 0)}"
            )
        if get_nonneg_int(last_tick, "safety_log_dropped") is not None:
            extra_exec_parts.append(f"`safety_log_dropped`={last_tick['safety_log_dropped']}")
        if get_nonneg_int(last_tick, "log_writer_dropped") is not None:
            extra_exec_parts.append(f"`log_writer_dropped`={last_tick['log_writer_dropped']}")
        if extra_exec_parts:
            out.append("- **Off-thread sink & cache counters**: " + ", ".join(extra_exec_parts))
    if has_release_us or release_max_ever_series.present_any:
        out.append(
            f"- **Full release duration (`release_us`)**: p50 med=`{fmt_ms(us_to_ms(release_us_p50_med_us))} ms`, "
            f"p99 max=`{fmt_ms(us_to_ms(release_us_p99_max_us))} ms`, "
            f"window max=`{fmt_ms(us_to_ms(release_us_max_us))} ms`, "
            f"`release_us_max_ever`=`{fmt_ms(us_to_ms(release_max_ever_series.end))} ms` | "
            f"`>=10ms` Δ={fmt_int(release_over_10_delta)}, "
            f"`>=15ms` Δ={fmt_int(release_over_15_delta)}, "
            f"`>=20ms` Δ={fmt_int(release_over_20_delta)}"
        )
    if has_wake_lateness or wake_late_max_ever_series.present_any:
        thresh_parts: list[str] = []
        if wake_late_max_ever_series.present_any:
            thresh_parts.append(f"`wake_late_max_ever`=`{fmt_ms(us_to_ms(wake_late_max_ever_series.end))} ms`")
        if has_wake_over_1ms:
            thresh_parts.append(f"`>1ms` Δ={fmt_int(wake_over_1ms_delta)} (total {fmt_int(wake_over_1ms_end)})")
        if has_wake_over_5ms:
            thresh_parts.append(f"`>5ms` Δ={fmt_int(wake_over_5ms_delta)} (total {fmt_int(wake_over_5ms_end)})")
        if has_wake_over_10ms:
            thresh_parts.append(f"`>10ms` Δ={fmt_int(wake_over_10ms_delta)} (total {fmt_int(wake_over_10ms_end)})")
        thresh_suffix = (" | " + ", ".join(thresh_parts)) if thresh_parts else ""
        out.append(
            f"- **Wake lateness (`wake_lateness_us`)**: p50 med=`{fmt_ms(us_to_ms(wake_lateness_p50_med_us))} ms`, "
            f"p99 max=`{fmt_ms(us_to_ms(wake_lateness_p99_max_us))} ms`, "
            f"max=`{fmt_ms(us_to_ms(wake_lateness_max_us))} ms`{thresh_suffix}"
        )
    if latest_since_marker is not None:
        out.append(
            f"- **Baseline marker (`since_marker`)**: id=`{latest_since_marker.get('marker_id')}`, "
            f"status=`{latest_since_marker.get('status')}`, "
            f"releases Δ=`{fmt_int(get_nonneg_int(latest_since_marker, 'releases_total_delta'))}`, "
            f"release_us_max=`{fmt_ms(us_to_ms(get_nonneg_int(latest_since_marker, 'release_us_max')))} ms`, "
            f"wake_late_max=`{fmt_ms(us_to_ms(get_nonneg_int(latest_since_marker, 'wake_late_max')))} ms`"
        )
    if has_diag_dropped:
        out.append(
            f"- **Diagnostic ring drops (`diag_dropped`)**: delta=`{fmt_int(diag_dropped_delta)}` "
            f"(start={fmt_int(diag_dropped_start)}, end={fmt_int(diag_dropped_end)})"
        )
    if ctl_sched:
        out.append(
            f"- **Control thread (`diagnostics/tick`)**: policy=`{ctl_sched.get('policy')}`, "
            f"priority=`{ctl_sched.get('priority')}` (requested `{ctl_sched.get('requested_priority')}`), "
            f"realtime=`{ctl_sched.get('realtime')}`, memory_locked=`{ctl_sched.get('memory_locked')}` (`{ctl_sched.get('memory_lock')}`)"
        )
    if load_run.meta.get("enabled_loads"):
        enabled_str = ", ".join(k for k, v in load_run.meta["enabled_loads"].items() if v)
        disabled_str = ", ".join(k for k, v in load_run.meta["enabled_loads"].items() if not v)
        out.append(f"- **Enabled loads**: {enabled_str or 'none'}" + (f" | **Disabled**: {disabled_str}" if disabled_str else ""))
    if load_run.summary.get("counters"):
        c = load_run.summary["counters"]
        out.append(
            f"- **Load counters**: obs_ok={c.get('obs_ok', 0)}, ws_msgs={c.get('ws_msgs', 0)}, "
            f"measure_ok={c.get('measure_ok', 0)} (err={c.get('measure_err', {})}), "
            f"bulk_ok={c.get('bulk_ok', 0)} ({c.get('bulk_bytes', 0)} B), "
            f"blackbox_ok={c.get('blackbox_ok', 0)}, cycles={c.get('motion_cycles_done', 0)}, "
            f"record={c.get('record_state')} (episode={c.get('record_episode_id')})"
        )
    elif load_run.summary.get("measure"):
        out.append(f"- **Measure summary**: `{json.dumps(load_run.summary['measure'])}`")
    if devstat:
        out.append(f"- **Kernel (`uname`)**: `{devstat.uname}`")
        kcfg = ", ".join(devstat.kernel_config) if devstat.kernel_config else "unavailable"
        out.append(f"- **Kernel config**: `{kcfg}`")
        if devstat.samples:
            ctl_first = find_armd_control_task(devstat.samples[0].armd_tasks)
            ctl_last = find_armd_control_task(devstat.samples[-1].armd_tasks)
            if ctl_first and ctl_last:
                vmlck = ctl_last.get("vm_lck_kb")
                vcsw_d = (ctl_last.get("vcsw", 0) - ctl_first.get("vcsw", 0)) if "vcsw" in ctl_last and "vcsw" in ctl_first else None
                ivcsw_d = (ctl_last.get("ivcsw", 0) - ctl_first.get("ivcsw", 0)) if "ivcsw" in ctl_last and "ivcsw" in ctl_first else None
                out.append(
                    f"- **`armd-control` (`/proc`)**: `VmLck={fmt_int(vmlck)} kB`, "
                    f"`voluntary_ctxt_switches` delta=`{fmt_int(vcsw_d)}`, `nonvoluntary_ctxt_switches` delta=`{fmt_int(ivcsw_d)}`"
                )
    out.append("")

    # 3. Release Metrics per Load Phase
    out.append("## 3. Release Latency per Load Phase")
    out.append("")
    phases_ordered: list[str] = []
    by_phase: dict[str, list[LoadSample]] = {}
    for s in samples:
        if s.phase not in by_phase:
            phases_ordered.append(s.phase)
            by_phase[s.phase] = []
        by_phase[s.phase].append(s)

    rel_col_key = "release_us" if has_release_us else "tick_us"
    wake_col_hdr = "`wake_lateness_us` p99/max (ms)" if has_wake_lateness else "Wake Overrun Proxy (ms)"
    skip_col_hdr = "`skipped`/`late` Δ" if has_skipped_releases else "`late_count` Δ"
    out.append(
        f"| Phase | Samples | `{rel_col_key}` p50 med (ms) | `{rel_col_key}` p99 max (ms) | `{rel_col_key}` max (ms) | "
        f"`lock_us` max (ms) | `wait_us` max (ms) | `period_us` max (ms) | {wake_col_hdr} | "
        f"{skip_col_hdr} | RX Age p99/max (ms) | Enqueue→Wire |"
    )
    out.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")

    def sum_step_deltas(grp: list[LoadSample], step_deltas: list[int | None]) -> int | None:
        vals = [step_deltas[x.index] for x in grp if step_deltas[x.index] is not None]
        return sum(vals) if vals else None

    def phase_row(name: str, grp: list[LoadSample]) -> str:
        t_p50 = us_to_ms(median_or_none([float(v) for x in grp if (v := get_pct(x.tick, rel_col_key, "p50")) is not None]))
        t_p99 = us_to_ms(max_or_none([float(v) for x in grp if (v := get_pct(x.tick, rel_col_key, "p99")) is not None]))
        t_max = us_to_ms(max_or_none([float(v) for x in grp if (v := get_pct(x.tick, rel_col_key, "max")) is not None]))
        l_max = us_to_ms(max_or_none([float(v) for x in grp if (v := get_pct(x.tick, "lock_us", "max")) is not None]))
        w_max = us_to_ms(max_or_none([float(v) for x in grp if (v := get_pct(x.tick, "wait_us", "max")) is not None]))
        p_max_us = max_or_none([float(v) for x in grp if (v := get_pct(x.tick, "period_us", "max")) is not None])
        p_max = us_to_ms(p_max_us)
        if has_wake_lateness:
            wl_p99 = us_to_ms(
                max_or_none([float(v) for x in grp if (v := get_pct(x.tick, "wake_lateness_us", "p99")) is not None])
            )
            wl_max = us_to_ms(
                max_or_none([float(v) for x in grp if (v := get_pct(x.tick, "wake_lateness_us", "max")) is not None])
            )
            wake_str = f"{fmt_ms(wl_p99)} / {fmt_ms(wl_max)}" if wl_max is not None else "n/a"
        else:
            wake_p = max(0.0, (p_max_us - nom_period_us) / 1000.0) if p_max_us is not None else None
            wake_str = fmt_ms(wake_p)
        ld = sum_step_deltas(grp, late_series.per_sample_deltas)
        if has_skipped_releases:
            sd = sum_step_deltas(grp, skipped_series.per_sample_deltas)
            skip_str = f"{fmt_int(sd)} / {fmt_int(ld)}"
        else:
            skip_str = fmt_int(ld)
        r_p99 = max_or_none([float(x.rx_age_ms["p99"]) for x in grp if x.rx_age_ms and x.rx_age_ms.get("p99") is not None])
        r_max = max_or_none([float(x.rx_age_ms["max"]) for x in grp if x.rx_age_ms and x.rx_age_ms.get("max") is not None])
        rx_str = f"{fmt_ms(r_p99, 1)} / {fmt_ms(r_max, 1)}" if r_max is not None else "n/a"
        enq_str = fmt_ms(us_to_ms(enqueue_to_wire_max_us)) + " ms" if has_enqueue_to_wire else "not exposed"
        return (
            f"| `{name}` | {len(grp)} | {fmt_ms(t_p50)} | {fmt_ms(t_p99)} | **{fmt_ms(t_max)}** | "
            f"{fmt_ms(l_max)} | {fmt_ms(w_max)} | {fmt_ms(p_max)} | {wake_str} | "
            f"{skip_str} | {rx_str} | {enq_str} |"
        )

    for ph in phases_ordered:
        out.append(phase_row(ph, by_phase[ph]))
    if len(phases_ordered) > 1:
        out.append(phase_row("ALL", samples))
    out.append("")

    # 4. Per-Phase Lock Breakdown & Wall vs CPU
    out.append("## 4. Per-Phase Breakdown & Wall-vs-CPU Preemption")
    out.append("")
    out.append("| Lock Phase | p50 med (ms) | p99 max (ms) | max (ms) |")
    out.append("|---|---:|---:|---:|")
    for pk in LOCK_PHASE_KEYS:
        st = phase_stats[pk]
        out.append(f"| `{pk}` | {fmt_ms(st['p50_ms'])} | {fmt_ms(st['p99_ms'])} | **{fmt_ms(st['max_ms'])}** |")
    out.append("")

    out.append("### Worst Measured Releases (Wall vs CPU)")
    out.append("")
    out.append(
        "| Kind | Phase | `release`/`tick` (ms) | `wait` (ms) | `lock` wall (ms) | `lock` CPU (ms) | Off-CPU (ms) | "
        "`rx`/`ctl`/`tx`/`obs`/`pub` (ms) | `lock_vcsw`/`ivcsw` | `maj`/`min` flt | Alloc calls | Lock Holder (`waited_for`) |"
    )
    out.append("|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---|")
    for kind, ph, _sidx, bd in breakdowns[:12]:
        lw = us_to_ms(get_nonneg_int(bd, "lock_us")) or 0.0
        lc = us_to_ms(get_nonneg_int(bd, "lock_cpu_us")) or 0.0
        off_cpu = max(0.0, lw - lc)
        sub_str = "/".join(
            fmt_ms(us_to_ms(get_nonneg_int(bd, k)), 1) for k in ("rx_us", "control_us", "tx_us", "obs_us", "publish_us")
        )
        csw_str = f"{bd.get('lock_vcsw', 0)}/{bd.get('lock_ivcsw', 0)}"
        flt_str = f"{bd.get('lock_majflt', 0)}/{bd.get('lock_minflt', 0)}"
        wf = bd.get("waited_for")
        if isinstance(wf, dict):
            wf_cmd = f" cmd=`{wf['command']}`" if wf.get("command") else ""
            wf_handoff = get_signed_int(bd, "handoff_us")
            if wf_handoff is None:
                wf_handoff = get_signed_int(wf, "released_before_lock_us")
            wf_hf = f" handoff={wf_handoff}us" if wf_handoff is not None else ""
            wf_str = (
                f"`{wf.get('kind')}@{wf.get('location')}`{wf_cmd} "
                f"({fmt_ms(us_to_ms(wf.get('hold_us')))} ms{wf_hf})"
            )
        else:
            wf_str = "none"
        rel_or_tick = get_nonneg_int(bd, "release_us")
        if rel_or_tick is None or rel_or_tick == 0:
            rel_or_tick = get_nonneg_int(bd, "tick_us")
        out.append(
            f"| `{kind}` | `{ph}` | **{fmt_ms(us_to_ms(rel_or_tick))}** | {fmt_ms(us_to_ms(get_nonneg_int(bd, 'wait_us')))} | "
            f"{fmt_ms(lw)} | {fmt_ms(lc)} | **{fmt_ms(off_cpu)}** | {sub_str} | {csw_str} | {flt_str} | "
            f"{bd.get('lock_alloc_calls', 0)} | {wf_str} |"
        )
    out.append("")

    max_release_rows = collect_unique_kind_breakdowns(samples, "max_release", "release_us")
    max_release_with_phases = [
        r for r in max_release_rows
        if isinstance(r[2].get("phases_us"), dict)
        or r[2].get("mid_nvcsw") is not None
        or r[2].get("mid_nivcsw") is not None
    ]
    if max_release_with_phases:
        out.append("### Release Phase Marks (`max_release`)")
        out.append("")
        out.append(
            "| Phase | `release` (ms) | `lock` (ms) | `alloc_read` (µs) | `ack_flush` (µs) | "
            "`terminal` (µs) | `guard_drop` (µs) | `post` (µs) | `publish` (µs) | `mid_nvcsw`/`mid_nivcsw` |"
        )
        out.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
        for ph, _sidx, bd in max_release_with_phases[:12]:
            pu = bd.get("phases_us") if isinstance(bd.get("phases_us"), dict) else {}
            mid_csw = f"{fmt_int(get_nonneg_int(bd, 'mid_nvcsw'))}/{fmt_int(get_nonneg_int(bd, 'mid_nivcsw'))}"
            out.append(
                f"| `{ph}` | **{fmt_ms(us_to_ms(get_nonneg_int(bd, 'release_us')))}** | "
                f"{fmt_ms(us_to_ms(get_nonneg_int(bd, 'lock_us')))} | "
                f"{fmt_int(get_nonneg_int(pu, 'alloc_read'))} | "
                f"{fmt_int(get_nonneg_int(pu, 'ack_flush'))} | "
                f"{fmt_int(get_nonneg_int(pu, 'terminal'))} | "
                f"{fmt_int(get_nonneg_int(pu, 'guard_drop'))} | "
                f"{fmt_int(get_nonneg_int(pu, 'post'))} | "
                f"{fmt_int(get_nonneg_int(pu, 'publish'))} | "
                f"{mid_csw} |"
            )
        out.append("")

    max_wait_rows = collect_unique_kind_breakdowns(samples, "max_wait", "wait_us")
    if max_wait_rows:
        out.append("### Worst Lock Wait Breakdown (`max_wait`)")
        out.append("")
        out.append(
            "| Phase | `wait_us` (ms) | `release` (ms) | Last Holder (`kind@location`) | "
            "Command | Holder Hold (ms) | Handoff (µs) |"
        )
        out.append("|---|---:|---:|---|---|---:|---:|")
        for ph, _sidx, bd in max_wait_rows[:12]:
            wf = bd.get("waited_for")
            if isinstance(wf, dict):
                holder_loc = f"`{wf.get('kind')}@{wf.get('location')}`"
                holder_cmd = f"`{wf['command']}`" if wf.get("command") else "n/a"
                holder_hold = fmt_ms(us_to_ms(get_nonneg_int(wf, "hold_us")))
                handoff = get_signed_int(bd, "handoff_us")
                if handoff is None:
                    handoff = get_signed_int(wf, "released_before_lock_us")
                handoff_str = fmt_int(handoff)
            else:
                holder_loc = "none"
                holder_cmd = "n/a"
                holder_hold = "n/a"
                handoff_str = "n/a"
            rel_or_tick = get_nonneg_int(bd, "release_us")
            if rel_or_tick is None or rel_or_tick == 0:
                rel_or_tick = get_nonneg_int(bd, "tick_us")
            out.append(
                f"| `{ph}` | **{fmt_ms(us_to_ms(get_nonneg_int(bd, 'wait_us')))}** | "
                f"{fmt_ms(us_to_ms(rel_or_tick))} | {holder_loc} | {holder_cmd} | "
                f"{holder_hold} | {handoff_str} |"
            )
        out.append("")

    # 5. Devstat Attribution (IRQ / SoftIRQ / CAN / Threads)
    if devstat is not None:
        out.append("## 5. On-Device Attribution (`devstat.txt`)")
        out.append("")

        # Thread priorities
        if devstat.armd_threads or devstat.irq_threads:
            out.append("### Thread Priorities (Startup Snapshot)")
            out.append("")
            out.append("```text")
            if devstat.armd_threads:
                out.append("[armd threads]")
                out.extend(devstat.armd_threads)
            if devstat.irq_threads:
                out.append("[irq / rt threads]")
                out.extend(devstat.irq_threads[:30])
                if len(devstat.irq_threads) > 30:
                    out.append(f"... ({len(devstat.irq_threads) - 30} more)")
            out.append("```")
            out.append("")

        # Compute per-interval deltas from devstat.
        ds = devstat.samples
        interval_rows: list[dict[str, Any]] = []
        irq_all_deltas: dict[str, list[int]] = {}
        softirq_all_deltas: dict[str, list[int]] = {}

        for i in range(1, len(ds)):
            prev_d, cur_d = ds[i - 1], ds[i]
            dt_s = (
                (cur_d.uptime_s - prev_d.uptime_s)
                if cur_d.uptime_s is not None and prev_d.uptime_s is not None
                else (
                    (cur_d.ts_unix - prev_d.ts_unix)
                    if cur_d.ts_unix is not None and prev_d.ts_unix is not None
                    else None
                )
            )
            irq_d = compute_dict_deltas(prev_d.interrupts, cur_d.interrupts)
            sirq_d = compute_dict_deltas(prev_d.softirqs, cur_d.softirqs)
            for k, v in irq_d.items():
                irq_all_deltas.setdefault(k, []).append(v)
            for k, v in sirq_d.items():
                softirq_all_deltas.setdefault(k, []).append(v)

            # Match load.jsonl samples falling in [prev_d.ts_unix, cur_d.ts_unix] (or by nearest index if no ts).
            matched_load: list[LoadSample] = []
            if prev_d.ts_unix is not None and cur_d.ts_unix is not None:
                for ls in samples:
                    if ls.ts_unix is not None and (prev_d.ts_unix - 1.0) <= ls.ts_unix <= (cur_d.ts_unix + 1.0):
                        matched_load.append(ls)
            elif i < len(samples):
                matched_load.append(samples[i])

            worst_tick_ms = max_or_none(
                [
                    ms
                    for ls in matched_load
                    if (
                        ms := us_to_ms(
                            get_pct(ls.tick, "release_us", "max")
                            if get_pct(ls.tick, "release_us", "max") is not None
                            else get_pct(ls.tick, "tick_us", "max")
                        )
                    )
                    is not None
                ]
            )
            worst_lock_ms = max_or_none(
                [
                    ms
                    for ls in matched_load
                    if (ms := us_to_ms(get_pct(ls.tick, "lock_us", "max"))) is not None
                ]
            )
            phases_hit = sorted({ls.phase for ls in matched_load})

            ctl_prev = find_armd_control_task(prev_d.armd_tasks)
            ctl_cur = find_armd_control_task(cur_d.armd_tasks)
            ctl_ivcsw_d = (
                (ctl_cur["ivcsw"] - ctl_prev["ivcsw"])
                if ctl_prev and ctl_cur and "ivcsw" in ctl_prev and "ivcsw" in ctl_cur
                else None
            )

            # IRQ thread CPU runtime deltas (ms) if available.
            irq_cpu_d: dict[str, float] = {}
            for tkey, rec in cur_d.irq_tasks.items():
                prec = prev_d.irq_tasks.get(tkey)
                if not prec:
                    continue
                if "sum_exec_ms" in rec and "sum_exec_ms" in prec:
                    delta_ms = rec["sum_exec_ms"] - prec["sum_exec_ms"]
                    if delta_ms > 0:
                        irq_cpu_d[tkey] = delta_ms

            interval_rows.append(
                {
                    "interval": f"{prev_d.index}->{cur_d.index}",
                    "ts_iso": cur_d.ts_iso or "n/a",
                    "dt_s": dt_s,
                    "phases": ",".join(phases_hit) if phases_hit else "unmatched",
                    "worst_tick_ms": worst_tick_ms,
                    "worst_lock_ms": worst_lock_ms,
                    "ctl_ivcsw_d": ctl_ivcsw_d,
                    "irq_d": irq_d,
                    "sirq_d": sirq_d,
                    "irq_cpu_d": irq_cpu_d,
                }
            )

        # Compute median IRQ delta across intervals so spikes stand out clearly.
        irq_medians = {k: statistics.median(v) for k, v in irq_all_deltas.items() if v}

        out.append("### Worst Release Intervals Joined with IRQ & SoftIRQ Deltas")
        out.append("")
        if interval_rows:
            ranked = sorted(
                interval_rows,
                key=lambda r: (r["worst_tick_ms"] if r["worst_tick_ms"] is not None else -1.0, sum(r["irq_d"].values())),
                reverse=True,
            )[:top_intervals]
            out.append(
                "| Devstat Interval | End UTC | Load Phase | `tick` max (ms) | `lock` max (ms) | `armd-control` `ivcsw` Δ | "
                "Top IRQ Deltas (vs median) | Top SoftIRQ Deltas | Top IRQ Thread CPU Δ (ms) |"
            )
            out.append("|---|---|---|---:|---:|---:|---|---|---|")
            for r in ranked:
                top_irqs = sorted(r["irq_d"].items(), key=lambda kv: kv[1], reverse=True)[:4]
                irq_txt = ", ".join(
                    f"`{k}`: +{v} (med {irq_medians.get(k, 0):.0f})" for k, v in top_irqs
                ) or "none"
                top_sirqs = sorted(r["sirq_d"].items(), key=lambda kv: kv[1], reverse=True)[:4]
                sirq_txt = ", ".join(f"`{k}`: +{v}" for k, v in top_sirqs) or "none"
                top_cpu = sorted(r["irq_cpu_d"].items(), key=lambda kv: kv[1], reverse=True)[:3]
                cpu_txt = ", ".join(f"`{k}`: +{v:.1f}" for k, v in top_cpu) or "n/a"
                out.append(
                    f"| `{r['interval']}` | `{r['ts_iso']}` | `{r['phases']}` | {fmt_ms(r['worst_tick_ms'])} | "
                    f"{fmt_ms(r['worst_lock_ms'])} | {fmt_int(r['ctl_ivcsw_d'])} | {irq_txt} | {sirq_txt} | {cpu_txt} |"
                )
        else:
            out.append("_Fewer than 2 devstat samples; cannot compute interval deltas._")
        out.append("")

        # Total IRQ & SoftIRQ delta summary across entire run.
        if len(ds) >= 2:
            tot_irq = compute_dict_deltas(ds[0].interrupts, ds[-1].interrupts)
            tot_sirq = compute_dict_deltas(ds[0].softirqs, ds[-1].softirqs)
            out.append("### Cumulative IRQ & SoftIRQ Deltas (Run Start -> End)")
            out.append("")
            out.append("| Source Type | Name | Total Delta |")
            out.append("|---|---|---:|")
            for k, v in sorted(tot_irq.items(), key=lambda kv: kv[1], reverse=True)[:10]:
                out.append(f"| IRQ | `{k}` | {v} |")
            for k, v in sorted(tot_sirq.items(), key=lambda kv: kv[1], reverse=True)[:8]:
                out.append(f"| SoftIRQ | `{k}` | {v} |")
            out.append("")

            # CAN Error Counter Deltas
            c0, c1 = ds[0].can, ds[-1].can
            out.append("### CAN Counter Deltas (`can0`)")
            out.append("")
            if c0.state is None and c1.state is None and c0.rx_packets is None:
                out.append("_CAN interface statistics unavailable in devstat.txt._")
            else:
                out.append(f"- **State**: start=`{c0.state}`, end=`{c1.state}`")
                out.append("")
                out.append("| Counter | Start | End | Delta |")
                out.append("|---|---:|---:|---:|")
                for label, attr in (
                    ("berr tx", "berr_tx"),
                    ("berr rx", "berr_rx"),
                    ("restarts", "restarts"),
                    ("error-warning", "error_warning"),
                    ("error-passive", "error_passive"),
                    ("bus-off", "bus_off"),
                    ("RX packets", "rx_packets"),
                    ("RX errors", "rx_errors"),
                    ("RX dropped", "rx_dropped"),
                    ("RX overrun", "rx_overrun"),
                    ("TX packets", "tx_packets"),
                    ("TX errors", "tx_errors"),
                    ("TX dropped", "tx_dropped"),
                    ("TX carrier", "tx_carrier"),
                ):
                    v0 = getattr(c0, attr)
                    v1 = getattr(c1, attr)
                    d = (v1 - v0) if v0 is not None and v1 is not None else None
                    out.append(f"| `{label}` | {fmt_int(v0)} | {fmt_int(v1)} | {fmt_int(d)} |")
            out.append("")

    # 6. Diagnostic Gaps
    gaps: list[str] = []
    if not has_release_us:
        gaps.append("`release_us`: full control-thread release duration is not present in input JSON.")
    if not all_monotonic_present:
        gaps.append(
            "`releases_total` / `release_us_max_ever` / `releases_over_{10,15,20}ms` / `wake_late_max_ever`: "
            "process-lifetime monotonic counters are missing on some or all samples."
        )
    if not has_skipped_releases:
        gaps.append(
            "`skipped_releases`: tracked in `ExecutorStats::skipped_releases` (`src/rt_core/executor.rs`), "
            "not serialized in `GET /api/v1/diagnostics/tick` (only `late_count` for `dt > 2 * period` is exposed)."
        )
    elif skipped_coverage_gap:
        gaps.append(
            "`skipped_releases`: partial presence or mid-run counter reset detected; deadline coverage is incomplete."
        )
    if not has_wake_lateness:
        gaps.append(
            "`wake_lateness_us`: scheduled release time vs actual wakeup offset is not in `TickSample`; "
            "only start-to-start `period_us` is exposed."
        )
    elif wake_lateness_partial:
        gaps.append("`wake_lateness_us`: missing on some samples; wake-lateness coverage is incomplete.")
    if not has_diag_dropped:
        gaps.append(
            "`diag_dropped`: not present in input JSON; diagnostic ring sample loss cannot be ruled out."
        )
    elif diag_series.partial_missing or diag_series.reset_detected:
        gaps.append(
            "`diag_dropped`: partial presence or mid-run counter reset detected; sample-loss coverage is incomplete."
        )
    if diag_samples_lost:
        gaps.append(
            f"`diag_dropped`: {diag_dropped_delta} diagnostic tick sample(s) were dropped by the full ring during this run; "
            "window maxima (`tick_us.max`, `wake_lateness_us.max`) may miss unrecorded peaks."
        )
    if not has_enqueue_to_wire:
        gaps.append(
            "`enqueue_to_wire_us`: `lock_phases.tx_us` covers encoding + nonblocking SocketCAN `write()` syscall only, "
            "not kernel qdisc/SPI/MCP251xFD wire delivery latency."
        )
    if not has_lock_attempts:
        gaps.append(
            "`control_thread_lock_attempts`: `SupervisorLock::control_thread_attempts` is not exposed in "
            "`GET /api/v1/diagnostics/tick` (verified via B1 tests; `wait_us`/`waited_for` are proxies)."
        )
    if not has_authoritative_alloc:
        gaps.append(
            "`lock_alloc_calls`: populated only when `armd` runs with `ARMD_ALLOC_TIMING=1`, and serialized only on "
            "`max_tick`/`max_lock` (not cumulative/window-wide); run-scoped `control_thread_alloc_ops` requires executor mode with `ARMD_ALLOC_TIMING=1`."
        )
    gaps.append(
        "`irq_runtime_ns`: `/proc/interrupts` exposes counts only; `/proc/<irq-tid>/sched` gives coarse threaded-IRQ "
        "CPU totals per sample interval, not hard-IRQ duration or per-tick overlapping timestamps."
    )
    if rx_age_max_ms is None:
        gaps.append(
            "`rx_age_ms`: not exposed inside `GET /api/v1/diagnostics/tick`; sample `feedback_age_ms` via "
            "`load.py` (`GET /api/v1/observation` / WS) for external RX age."
        )

    out.append("## 6. Diagnostic Gaps Observed in This Run")
    out.append("")
    for g in gaps:
        out.append(f"- {g}")
    out.append("")

    if owner_mode == "executor":
        out.append("## 7. Simulation-Only Qualification Coverage (Covered in Sim)")
        out.append("")
        out.append(
            "Fault-injection panic and teardown tests that alter hold targets or require mechanical support "
            "run on mock / Damiao `FakeBus` rather than unattended hardware:"
        )
        out.append("")
        out.append("| Qualification Item | Status | Evidence Suite |")
        out.append("|---|---|---|")
        out.append("| Control-thread panic -> preallocated zero-alloc SAFE-HOLD on measured positions | covered in sim | `test_drv_f1_lifecycle`, `test_drv_f1_alloc`, `test_rt_executor_hardening` |")
        out.append("| Non-control panic -> stop, hold, park-to-rest, disable at confirmed rest | covered in sim | `test_drv_f1_lifecycle` |")
        out.append("| Energized handback (`executor` -> `legacy`) and startup-failure legacy fallback | covered in sim | `test_drv_f1_lifecycle`, `test_g_real_follower` |")
        out.append("| Stalled teardown / startup-timeout SAFE-HOLD & deliberate ProcessExit fallback | covered in sim | `test_drv_f1_lifecycle`, `test_rt_executor_hardening` |")
        out.append("| Thermal trip protective disable path | covered in sim | `test_drv_f1_lifecycle`, `test_rt_executor_hardening` |")
        out.append("| Whole-release / sub-release-wake 0-alloc & 0-supervisor-lock proofs | covered in sim | `test_rt_whole_release_alloc`, `test_drv_f1_alloc`, `test_rt_lock_proof` |")
        out.append("| Damiao `FakeBus` executor E2E (enable, J1 motion, rest-pose torque-off) | covered in sim | `test_g_real_follower` |")
        out.append("")

    if sched_trace is not None:
        out.extend(render_sched_trace_section(sched_trace))

    return "\n".join(out)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Analyze SG2002 RT qualification load.jsonl + devstat.txt + ftrace sched_trace.")
    p.add_argument("--load", default=None, help="Path to load.jsonl (or legacy dev6 tickload.json)")
    p.add_argument("--devstat", default=None, help="Optional path to devstat.txt from reCamera")
    p.add_argument("--sched-trace", default=None, help="Optional path to ftrace sched_switch capture from sched_trace.sh")
    p.add_argument("--control-tid", type=int, default=None, help="TID of armd-control for --sched-trace analysis (auto-detected from --devstat if omitted)")
    p.add_argument("--min-switch-out-ms", type=float, default=2.0, help="Minimum switch-out duration (ms) to list from --sched-trace (default: 2.0)")
    p.add_argument("--output", default=None, help="Optional output Markdown path (default: stdout)")
    p.add_argument("--nominal-period-us", type=int, default=None, help="Nominal control period in us (default: inferred from period_us.p50)")
    p.add_argument("--top-intervals", type=int, default=5, help="Number of worst release intervals to join with IRQ deltas")
    args = p.parse_args(argv)
    if args.load is None and args.sched_trace is None:
        p.error("at least one of --load or --sched-trace is required")
    return args


def _resolve_control_tid(explicit_tid: int | None, devstat: DevstatRun | None) -> int | None:
    if explicit_tid is not None:
        return explicit_tid
    if devstat is not None and devstat.samples:
        ctl = find_armd_control_task(devstat.samples[0].armd_tasks)
        if ctl is not None and isinstance(ctl.get("tid"), int):
            return int(ctl["tid"])
    return None


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    devstat = parse_devstat(Path(args.devstat)) if args.devstat else None
    sched_trace: SchedTraceAnalysis | None = None
    if args.sched_trace:
        tid = _resolve_control_tid(args.control_tid, devstat)
        if tid is None:
            print(
                "Error: --control-tid is required with --sched-trace (or supply --devstat containing armd-control)",
                file=sys.stderr,
            )
            return 2
        sched_trace = parse_sched_trace(
            Path(args.sched_trace),
            control_tid=tid,
            min_duration_ms=args.min_switch_out_ms,
        )
    if args.load:
        load_run = load_input(Path(args.load))
        report = analyze(
            load_run,
            devstat,
            nominal_period_us=args.nominal_period_us,
            top_intervals=args.top_intervals,
            sched_trace=sched_trace,
        )
    else:
        assert sched_trace is not None
        report = "\n".join(
            render_sched_trace_section(
                sched_trace,
                heading="# Scheduler Switch-Out Report (`sched_trace`)",
            )
        )
    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(report, encoding="utf-8")
        print(f"Wrote report to {out_path}", file=sys.stderr)
    else:
        print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
