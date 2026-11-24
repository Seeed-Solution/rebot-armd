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


def cumulative_delta(
    samples: list[LoadSample], key: str
) -> tuple[bool, int | None, int | None, int | None]:
    """Return (present, start, end, delta) for a monotonic cumulative integer field in `tick`."""
    vals = [
        int(s.tick[key])
        for s in samples
        if isinstance(s.tick.get(key), int) and not isinstance(s.tick.get(key), bool)
    ]
    if not vals:
        return (False, None, None, None)
    start, end = vals[0], vals[-1]
    if len(vals) == 1:
        delta = end
    elif end >= start:
        delta = end - start
    else:
        # Daemon restarted mid-run; fall back to post-restart count.
        delta = end
    return (True, start, end, delta)


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


def load_input(path: Path) -> LoadRun:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"Empty load file: {path}")

    # Auto-detect dev6 tickload.json (single JSON object with "ticks": [...]) vs JSONL.
    if text.startswith("{"):
        try:
            obj = json.loads(text)
            if isinstance(obj, dict) and isinstance(obj.get("ticks"), list):
                samples: list[LoadSample] = []
                for idx, t in enumerate(obj["ticks"]):
                    if not isinstance(t, dict):
                        continue
                    samples.append(
                        LoadSample(
                            index=idx,
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
                    "note": "Converted from legacy tickload.json (no per-sample timestamps or phase tags).",
                }
                summary = {
                    "fault": None if str(obj.get("fault")) in ("None", "null", "") else str(obj.get("fault")),
                    "measure": obj.get("measure"),
                }
                return LoadRun("tickload_json", meta, summary, samples)
        except json.JSONDecodeError:
            pass

    meta = {}
    summary = {}
    samples = []
    idx = 0
    for line_no, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        rec = json.loads(line)
        if not isinstance(rec, dict):
            continue
        rtype = rec.get("type")
        if rtype == "meta":
            meta = rec
        elif rtype == "summary":
            summary = rec
        else:
            tick = rec.get("tick") if "tick" in rec else (rec if "tick_us" in rec else None)
            if not isinstance(tick, dict):
                continue
            samples.append(
                LoadSample(
                    index=idx,
                    ts_unix=float(rec["ts_unix"]) if rec.get("ts_unix") is not None else None,
                    ts_iso=rec.get("ts_iso"),
                    elapsed_s=float(rec["elapsed_s"]) if rec.get("elapsed_s") is not None else None,
                    phase=str(rec.get("phase") or "unknown"),
                    tick=tick,
                    rx_age_ms=rec.get("rx_age_ms") if isinstance(rec.get("rx_age_ms"), dict) else None,
                    counters=rec.get("counters") if isinstance(rec.get("counters"), dict) else {},
                    fault=rec.get("fault"),
                    safety_state=rec.get("safety_state"),
                )
            )
            idx += 1
    if not samples:
        raise ValueError(f"No tick samples found in {path}")
    return LoadRun("jsonl", meta, summary, samples)


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
        float(s.tick["period_us"]["p50"])
        for s in samples
        if isinstance(s.tick.get("period_us"), dict) and s.tick["period_us"].get("p50")
    ]
    if not p50s:
        return 20_000
    med = statistics.median(p50s)
    # Snap to nearest 1000 us (e.g. 20000 us for 50 Hz, 10000 us for 100 Hz).
    return max(1000, int(round(med / 1000.0) * 1000))


def collect_unique_breakdowns(samples: list[LoadSample]) -> list[tuple[str, str, int, dict[str, Any]]]:
    """Return deduplicated (kind, phase, sample_idx, breakdown) sorted by tick_us desc."""
    by_sig: dict[tuple[Any, ...], list[Any]] = {}
    for s in samples:
        for kind in ("max_tick", "max_lock"):
            bd = s.tick.get(kind)
            if not isinstance(bd, dict):
                continue
            holder = bd.get("waited_for")
            sig = (
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
                holder.get("location") if isinstance(holder, dict) else None,
                holder.get("hold_us") if isinstance(holder, dict) else None,
            )
            if sig not in by_sig:
                by_sig[sig] = [[kind], s.phase, s.index, bd]
            elif kind not in by_sig[sig][0]:
                by_sig[sig][0].append(kind)
    rows = [("+".join(v[0]), v[1], v[2], v[3]) for v in by_sig.values()]
    rows.sort(key=lambda r: (r[3].get("tick_us") or 0, r[3].get("lock_us") or 0), reverse=True)
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


def analyze(
    load_run: LoadRun,
    devstat: DevstatRun | None,
    nominal_period_us: int | None = None,
    top_intervals: int = 5,
) -> str:
    samples = load_run.samples
    nom_period_us = infer_nominal_period_us(samples, nominal_period_us)
    nom_period_ms = nom_period_us / 1000.0

    # Aggregate global metrics across all samples.
    tick_max_us = max((s.tick.get("tick_us", {}).get("max", 0) for s in samples), default=0)
    tick_p99_max_us = max((s.tick.get("tick_us", {}).get("p99", 0) for s in samples), default=0)
    tick_p50_med_us = median_or_none([float(s.tick["tick_us"]["p50"]) for s in samples if "tick_us" in s.tick])

    lock_max_us = max((s.tick.get("lock_us", {}).get("max", 0) for s in samples), default=0)
    wait_max_us = max((s.tick.get("wait_us", {}).get("max", 0) for s in samples), default=0)
    reader_hold_max_us = max((s.tick.get("reader_hold_us", {}).get("max", 0) for s in samples), default=0)
    writer_hold_max_us = max((s.tick.get("writer_hold_us", {}).get("max", 0) for s in samples), default=0)
    period_max_us = max((s.tick.get("period_us", {}).get("max", 0) for s in samples), default=0)
    period_p99_max_us = max((s.tick.get("period_us", {}).get("p99", 0) for s in samples), default=0)

    first_ticks = samples[0].tick.get("since_boot_ticks")
    last_ticks = samples[-1].tick.get("since_boot_ticks")
    release_count = (
        int(last_ticks) - int(first_ticks)
        if isinstance(first_ticks, int) and isinstance(last_ticks, int) and last_ticks >= first_ticks
        else None
    )

    first_late = samples[0].tick.get("late_count")
    last_late = samples[-1].tick.get("late_count")
    late_delta = (
        int(last_late) - int(first_late)
        if isinstance(first_late, int) and isinstance(last_late, int)
        else None
    )
    late_any_nonzero = any((s.tick.get("late_count") or 0) > 0 for s in samples)

    # Q1 executor/legacy timing fields (absent in older JSON; keep proxy fallbacks).
    has_skipped_releases, skipped_start, skipped_end, skipped_delta = cumulative_delta(samples, "skipped_releases")
    has_diag_dropped, diag_dropped_start, diag_dropped_end, diag_dropped_delta = cumulative_delta(samples, "diag_dropped")
    diag_samples_lost = bool(has_diag_dropped and diag_dropped_delta is not None and diag_dropped_delta > 0)

    wake_dicts = [s.tick["wake_lateness_us"] for s in samples if isinstance(s.tick.get("wake_lateness_us"), dict)]
    has_wake_lateness = bool(wake_dicts)
    wake_lateness_max_us = (
        max(int(w.get("max", 0)) for w in wake_dicts) if has_wake_lateness else None
    )
    wake_lateness_p99_max_us = (
        max(int(w.get("p99", 0)) for w in wake_dicts) if has_wake_lateness else None
    )
    wake_lateness_p50_med_us = (
        median_or_none([float(w["p50"]) for w in wake_dicts if w.get("p50") is not None])
        if has_wake_lateness
        else None
    )
    has_wake_over_1ms, _, wake_over_1ms_end, wake_over_1ms_delta = cumulative_delta(samples, "wake_lateness_over_1ms")
    has_wake_over_5ms, _, wake_over_5ms_end, wake_over_5ms_delta = cumulative_delta(samples, "wake_lateness_over_5ms")
    has_wake_over_10ms, _, wake_over_10ms_end, wake_over_10ms_delta = cumulative_delta(samples, "wake_lateness_over_10ms")

    owner_mode: str | None = None
    for s in reversed(samples):
        if isinstance(s.tick.get("owner_mode"), str) and s.tick["owner_mode"]:
            owner_mode = s.tick["owner_mode"]
            break

    has_enqueue_to_wire = any("enqueue_to_wire_us" in s.tick for s in samples)
    enqueue_to_wire_max_us = (
        max(
            (
                s.tick["enqueue_to_wire_us"]["max"]
                if isinstance(s.tick.get("enqueue_to_wire_us"), dict)
                else int(s.tick.get("enqueue_to_wire_us") or 0)
            )
            for s in samples
            if "enqueue_to_wire_us" in s.tick
        )
        if has_enqueue_to_wire
        else None
    )

    has_lock_attempts = any("control_thread_lock_attempts" in s.tick for s in samples)
    lock_attempts_max = (
        max(int(s.tick.get("control_thread_lock_attempts") or 0) for s in samples)
        if has_lock_attempts
        else None
    )

    # Lock holders observed on max_tick / max_lock.
    breakdowns = collect_unique_breakdowns(samples)
    lock_wait_holders = [
        bd["waited_for"]
        for _, _, _, bd in breakdowns
        if isinstance(bd.get("waited_for"), dict)
    ]
    alloc_calls_max = max(
        (int(bd.get("lock_alloc_calls") or 0) for _, _, _, bd in breakdowns),
        default=0,
    )

    # RX age observed via load.py polling / WS.
    rx_age_max_ms = max_or_none(
        [float(s.rx_age_ms["max"]) for s in samples if s.rx_age_ms and s.rx_age_ms.get("max") is not None]
    )
    rx_age_p99_ms = max_or_none(
        [float(s.rx_age_ms["p99"]) for s in samples if s.rx_age_ms and s.rx_age_ms.get("p99") is not None]
    )

    # Per-phase lock percentiles across all samples.
    phase_stats: dict[str, dict[str, float | None]] = {}
    for pk in LOCK_PHASE_KEYS:
        p50s = [float(s.tick["lock_phases"][pk]["p50"]) for s in samples if pk in s.tick.get("lock_phases", {})]
        p99s = [float(s.tick["lock_phases"][pk]["p99"]) for s in samples if pk in s.tick.get("lock_phases", {})]
        maxs = [float(s.tick["lock_phases"][pk]["max"]) for s in samples if pk in s.tick.get("lock_phases", {})]
        phase_stats[pk] = {
            "p50_ms": us_to_ms(median_or_none(p50s)),
            "p99_ms": us_to_ms(max_or_none(p99s)),
            "max_ms": us_to_ms(max_or_none(maxs)),
        }

    # Wake lateness (real Q1 field when present; legacy period_us proxy otherwise) & MIT budget.
    if has_wake_lateness and wake_lateness_max_us is not None:
        wake_extra_ms = wake_lateness_max_us / 1000.0
        wake_extra_label = "wake lateness"
    else:
        wake_extra_ms = max(0.0, (period_max_us - nom_period_us) / 1000.0)
        wake_extra_label = "sched overrun proxy"
    rx_max_ms = phase_stats["rx_us"]["max_ms"] or 0.0
    tx_max_ms = phase_stats["tx_us"]["max_ms"] or 0.0
    # Worst start-to-TX completion observed in any breakdown (wait + pre + rx + control + tx),
    # or tick_us.max if larger.
    start_to_tx_ms = max(
        (
            (
                (bd.get("wait_us") or 0)
                + (bd.get("pre_us") or 0)
                + (bd.get("rx_us") or 0)
                + (bd.get("control_us") or 0)
                + (bd.get("tx_us") or 0)
            )
            / 1000.0
            for _, _, _, bd in breakdowns
        ),
        default=us_to_ms(tick_max_us) or 0.0,
    )
    enq_ms = (enqueue_to_wire_max_us / 1000.0) if enqueue_to_wire_max_us is not None else 0.0
    worst_extra_ms = wake_extra_ms + start_to_tx_ms + enq_ms
    mit_total_ms = MIT_PERIOD_MS + worst_extra_ms + MIT_MARGIN_MS

    # Control thread info (latest non-null).
    ctl_sched: dict[str, Any] = {}
    for s in reversed(samples):
        if isinstance(s.tick.get("control_thread"), dict):
            ctl_sched = s.tick["control_thread"]
            break

    faults_seen = sorted({s.fault for s in samples if s.fault})
    if load_run.summary.get("fault") and load_run.summary["fault"] not in faults_seen:
        faults_seen.append(str(load_run.summary["fault"]))

    # ---- Gate Evaluation ----
    release_pass = (tick_max_us / 1000.0) < RELEASE_BUDGET_MS
    if not release_pass:
        release_status = "FAIL"
    elif diag_samples_lost:
        release_status = "INCOMPLETE (diag_dropped > 0)"
    else:
        release_status = "PASS"
    release_detail = (
        f"`tick_us.max` = **{fmt_ms(us_to_ms(tick_max_us))} ms** "
        f"(p99 max {fmt_ms(us_to_ms(tick_p99_max_us))} ms, p50 med {fmt_ms(us_to_ms(tick_p50_med_us))} ms)"
        + (f" (incomplete: `diag_dropped` delta={diag_dropped_delta})" if diag_samples_lost else "")
    )

    if has_skipped_releases and skipped_delta is not None:
        skipped_pass = skipped_delta == 0
        skipped_status = "PASS" if skipped_pass else "FAIL"
        diag_note = f", diag_dropped delta={diag_dropped_delta}" if has_diag_dropped else ""
        skipped_detail = (
            f"skipped_releases delta={skipped_delta} (start={fmt_int(skipped_start)}, end={fmt_int(skipped_end)}), "
            f"late_count delta={fmt_int(late_delta)}{diag_note}"
        )
    else:
        skipped_pass = (late_delta == 0 if late_delta is not None else not late_any_nonzero) and period_max_us <= (2 * nom_period_us)
        skipped_status = "PASS (proxy)" if skipped_pass else "FAIL"
        skipped_detail = (
            f"late_count delta={fmt_int(late_delta)} (start={fmt_int(first_late)}, end={fmt_int(last_late)}), "
            f"period_us.max={fmt_ms(us_to_ms(period_max_us))} ms "
            f"(gap: ExecutorStats::skipped_releases not exposed in /diagnostics/tick)"
        )

    # Lock gate: fail if any lock attempt counter > 0 or any tick waited for lock / non-zero wait_us (> 50 us indicates real lock wait, > 0 in RT executor).
    lock_wait_observed = bool(lock_wait_holders) or wait_max_us > 0 or writer_hold_max_us > 0 or reader_hold_max_us > 0
    if has_lock_attempts:
        lock_pass = (lock_attempts_max == 0) and not lock_wait_observed
        lock_status = "PASS" if lock_pass else "FAIL"
        lock_detail = f"control_thread_lock_attempts={lock_attempts_max}, wait_us.max={fmt_ms(us_to_ms(wait_max_us))} ms"
    else:
        lock_pass = not lock_wait_observed
        if lock_pass:
            lock_status = "PASS (proxy)"
            lock_detail = (
                "wait_us.max=0.00 ms, waited_for=none, reader/writer_hold=0 "
                "(gap: SupervisorLock::control_thread_attempts not exposed; confirm via B1 cargo test)"
            )
        else:
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

    alloc_timing_confirmed = False
    if devstat is not None:
        alloc_timing_confirmed = any("ARMD_ALLOC_TIMING=1" in l for l in devstat.armd_version)
    if alloc_calls_max > 0:
        alloc_pass = False
        alloc_status = "FAIL"
        alloc_detail = f"lock_alloc_calls max={alloc_calls_max} on max_tick/max_lock"
    else:
        alloc_pass = True
        alloc_status = "PASS (proxy)"
        alloc_detail = (
            f"lock_alloc_calls=0 on max_tick/max_lock "
            f"(gap: only reported on max_tick/max_lock and zero unless ARMD_ALLOC_TIMING=1; "
            f"ARMD_ALLOC_TIMING=1 confirmed={alloc_timing_confirmed})"
        )

    mit_observable_pass = mit_total_ms < MIT_WATCHDOG_MS
    mit_gaps: list[str] = []
    if diag_samples_lost:
        mit_gaps.append(f"diag_dropped delta={diag_dropped_delta} lost tick/wake samples")
    if not has_wake_lateness:
        mit_gaps.append("wake_lateness_us not exposed (using period proxy)")
    if not has_enqueue_to_wire:
        mit_gaps.append("enqueue-to-wire CAN latency not exposed")
    if not mit_observable_pass:
        mit_status = "FAIL"
    elif diag_samples_lost:
        mit_status = "INCOMPLETE (diag_dropped > 0)"
    elif not mit_gaps:
        mit_status = "PASS"
    else:
        mit_status = "INCOMPLETE (observable PASS)"
    mit_detail = (
        f"20.00 + {wake_extra_ms:.2f} ({wake_extra_label}) + {start_to_tx_ms:.2f} (start->TX) "
        f"+ {enq_ms:.2f} (enqueue->wire) + 5.00 = {mit_total_ms:.2f} ms vs 45.00 ms"
        + (f" (gap: {'; '.join(mit_gaps)})" if mit_gaps else "")
    )

    fault_pass = len(faults_seen) == 0

    hard_fail = (not release_pass) or (not skipped_pass) or (not lock_pass) or (not alloc_pass) or (not mit_observable_pass) or (not fault_pass)
    any_gaps = (
        (not has_skipped_releases)
        or (not has_wake_lateness)
        or diag_samples_lost
        or (not has_lock_attempts)
        or (not has_enqueue_to_wire)
        or (not alloc_timing_confirmed)
    )
    if hard_fail:
        overall_verdict = "FAIL"
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

    # 1. Gate Criteria Table
    out.append("## 1. Stage 3 §6 Gate Verdict")
    out.append("")
    out.append("| Criterion | Target | Measured / Evidence | Status |")
    out.append("|---|---|---|---|")
    out.append(
        f"| Every measured release | `< {RELEASE_BUDGET_MS:.0f} ms` | "
        f"{release_detail} | "
        f"{release_status} |"
    )
    out.append(f"| Zero skipped deadlines | `0` | {skipped_detail} | {skipped_status} |")
    out.append(f"| Lock gate (zero supervisor-lock attempts) | `0` | {lock_detail} | {lock_status} |")
    out.append(f"| Allocation gate (zero release allocs) | `0` | {alloc_detail} | {alloc_status} |")
    out.append(f"| MIT watchdog budget | `20 + extra + 5 < 45 ms` | {mit_detail} | {mit_status} |")
    out.append(
        f"| Fault / stop trace check | `0 faults` | "
        f"{'none' if fault_pass else '; '.join(faults_seen)} | {'PASS' if fault_pass else 'FAIL'} |"
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
    out.append(f"- **Input format**: `{load_run.source_format}` ({len(samples)} tick-stats snapshots, nominal period `{nom_period_ms:.1f} ms`)")
    if load_run.meta.get("target"):
        out.append(f"- **Target**: `{load_run.meta['target']}` | **armd version**: `{armd_ver}`")
    if release_count is not None:
        out.append(f"- **Releases covered**: `{release_count}` (`since_boot_ticks`: {first_ticks} -> {last_ticks})")
    if owner_mode is not None:
        out.append(f"- **Owner mode**: `{owner_mode}`")
    if has_wake_lateness:
        thresh_parts: list[str] = []
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

    wake_col_hdr = "`wake_lateness_us` p99/max (ms)" if has_wake_lateness else "Wake Overrun Proxy (ms)"
    skip_col_hdr = "`skipped`/`late` Δ" if has_skipped_releases else "`late_count` Δ"
    out.append(
        f"| Phase | Samples | `tick_us` p50 med (ms) | `tick_us` p99 max (ms) | `tick_us` max (ms) | "
        f"`lock_us` max (ms) | `wait_us` max (ms) | `period_us` max (ms) | {wake_col_hdr} | "
        f"{skip_col_hdr} | RX Age p99/max (ms) | Enqueue→Wire |"
    )
    out.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")

    def phase_row(name: str, grp: list[LoadSample], prev_sample: LoadSample | None = None) -> str:
        t_p50 = us_to_ms(median_or_none([float(x.tick["tick_us"]["p50"]) for x in grp if "tick_us" in x.tick]))
        t_p99 = us_to_ms(max_or_none([float(x.tick["tick_us"]["p99"]) for x in grp if "tick_us" in x.tick]))
        t_max = us_to_ms(max_or_none([float(x.tick["tick_us"]["max"]) for x in grp if "tick_us" in x.tick]))
        l_max = us_to_ms(max_or_none([float(x.tick["lock_us"]["max"]) for x in grp if "lock_us" in x.tick]))
        w_max = us_to_ms(max_or_none([float(x.tick["wait_us"]["max"]) for x in grp if "wait_us" in x.tick]))
        p_max_us = max_or_none([float(x.tick["period_us"]["max"]) for x in grp if "period_us" in x.tick])
        p_max = us_to_ms(p_max_us)
        if has_wake_lateness:
            wl_p99 = us_to_ms(
                max_or_none(
                    [
                        float(x.tick["wake_lateness_us"]["p99"])
                        for x in grp
                        if isinstance(x.tick.get("wake_lateness_us"), dict)
                        and x.tick["wake_lateness_us"].get("p99") is not None
                    ]
                )
            )
            wl_max = us_to_ms(
                max_or_none(
                    [
                        float(x.tick["wake_lateness_us"]["max"])
                        for x in grp
                        if isinstance(x.tick.get("wake_lateness_us"), dict)
                        and x.tick["wake_lateness_us"].get("max") is not None
                    ]
                )
            )
            wake_str = f"{fmt_ms(wl_p99)} / {fmt_ms(wl_max)}" if wl_max is not None else "n/a"
        else:
            wake_p = max(0.0, (p_max_us - nom_period_us) / 1000.0) if p_max_us is not None else None
            wake_str = fmt_ms(wake_p)
        l0 = prev_sample.tick.get("late_count") if prev_sample is not None else grp[0].tick.get("late_count")
        l1 = grp[-1].tick.get("late_count")
        ld = (int(l1) - int(l0)) if isinstance(l0, int) and isinstance(l1, int) and l1 >= l0 else None
        if has_skipped_releases:
            delta_grp = [prev_sample, *grp] if prev_sample is not None else grp
            _, _, _, sd = cumulative_delta(delta_grp, "skipped_releases")
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

    prev_phase_last: LoadSample | None = None
    for ph in phases_ordered:
        out.append(phase_row(ph, by_phase[ph], prev_phase_last))
        prev_phase_last = by_phase[ph][-1]
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

    out.append("### Worst Measured Ticks (Wall vs CPU)")
    out.append("")
    out.append(
        "| Kind | Phase | `tick` (ms) | `wait` (ms) | `lock` wall (ms) | `lock` CPU (ms) | Off-CPU (ms) | "
        "`rx`/`ctl`/`tx`/`obs`/`pub` (ms) | `lock_vcsw`/`ivcsw` | `maj`/`min` flt | Alloc calls | Lock Holder (`waited_for`) |"
    )
    out.append("|---|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---|")
    for kind, ph, _sidx, bd in breakdowns[:12]:
        lw = us_to_ms(bd.get("lock_us")) or 0.0
        lc = us_to_ms(bd.get("lock_cpu_us")) or 0.0
        off_cpu = max(0.0, lw - lc)
        sub_str = "/".join(
            fmt_ms(us_to_ms(bd.get(k)), 1) for k in ("rx_us", "control_us", "tx_us", "obs_us", "publish_us")
        )
        csw_str = f"{bd.get('lock_vcsw', 0)}/{bd.get('lock_ivcsw', 0)}"
        flt_str = f"{bd.get('lock_majflt', 0)}/{bd.get('lock_minflt', 0)}"
        wf = bd.get("waited_for")
        wf_str = (
            f"`{wf.get('kind')}@{wf.get('location')}` ({fmt_ms(us_to_ms(wf.get('hold_us')))} ms)"
            if isinstance(wf, dict)
            else "none"
        )
        out.append(
            f"| `{kind}` | `{ph}` | **{fmt_ms(us_to_ms(bd.get('tick_us')))}** | {fmt_ms(us_to_ms(bd.get('wait_us')))} | "
            f"{fmt_ms(lw)} | {fmt_ms(lc)} | **{fmt_ms(off_cpu)}** | {sub_str} | {csw_str} | {flt_str} | "
            f"{bd.get('lock_alloc_calls', 0)} | {wf_str} |"
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

            worst_tick_ms = max_or_none([us_to_ms(ls.tick.get("tick_us", {}).get("max")) or 0.0 for ls in matched_load])
            worst_lock_ms = max_or_none([us_to_ms(ls.tick.get("lock_us", {}).get("max")) or 0.0 for ls in matched_load])
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
    if not has_skipped_releases:
        gaps.append(
            "`skipped_releases`: tracked in `ExecutorStats::skipped_releases` (`src/rt_core/executor.rs`), "
            "not serialized in `GET /api/v1/diagnostics/tick` (only `late_count` for `dt > 2 * period` is exposed)."
        )
    if not has_wake_lateness:
        gaps.append(
            "`wake_lateness_us`: scheduled release time vs actual wakeup offset is not in `TickSample`; "
            "only start-to-start `period_us` is exposed."
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
    gaps.append(
        "`lock_alloc_calls`: populated only when `armd` runs with `ARMD_ALLOC_TIMING=1`, and serialized only on "
        "`max_tick`/`max_lock` (not cumulative/window-wide); `ExecutorStats::wake_alloc_calls` is internal."
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

    return "\n".join(out)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Analyze SG2002 RT qualification load.jsonl + devstat.txt.")
    p.add_argument("--load", required=True, help="Path to load.jsonl (or legacy dev6 tickload.json)")
    p.add_argument("--devstat", default=None, help="Optional path to devstat.txt from reCamera")
    p.add_argument("--output", default=None, help="Optional output Markdown path (default: stdout)")
    p.add_argument("--nominal-period-us", type=int, default=None, help="Nominal control period in us (default: inferred from period_us.p50)")
    p.add_argument("--top-intervals", type=int, default=5, help="Number of worst release intervals to join with IRQ deltas")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    load_run = load_input(Path(args.load))
    devstat = parse_devstat(Path(args.devstat)) if args.devstat else None
    report = analyze(
        load_run,
        devstat,
        nominal_period_us=args.nominal_period_us,
        top_intervals=args.top_intervals,
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
