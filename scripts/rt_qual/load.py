#!/usr/bin/env python3
"""SG2002 real-time qualification load driver (Stage 3 §6).

Runs on the office Raspberry Pi (Python 3.11+, stdlib + rebot SDK only).
Drives concurrent loads against armd and samples GET /api/v1/diagnostics/tick
every N seconds to a JSONL file.

Safety invariants:
- speed_scale is hard-capped at <= 0.3;
- motion and recording run inside `with Arm(...)`;
- stops immediately on any detected fault or E-stop and never clears faults;
- every worker loop is bounded by stop_event and a monotonic deadline.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import signal
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

try:
    from rebot import Arm
    from rebot import _ws as rebot_ws
    from rebot.client import ArmdClient
    from rebot.errors import RebotError
    HAS_REBOT_SDK = True
except ImportError:
    Arm = None  # type: ignore[assignment]
    rebot_ws = None  # type: ignore[assignment]
    ArmdClient = None  # type: ignore[assignment]

    class RebotError(Exception):  # type: ignore[no-redef]
        pass

    HAS_REBOT_SDK = False

MAX_SPEED_SCALE = 0.3
VALID_STOP_REASONS = ("completed", "interrupted", "fault", "error")
_STOP_REASON_PRIORITY = {"completed": 0, "error": 1, "interrupted": 2, "fault": 3}
MIN_IDLE_TAIL_S = 1.0
IDLE_TAIL_PERIOD_MULTIPLIER = 20.0  # 2 * period * 10 idle releases minimum
MIN_IDLE_TAIL_RELEASES = 10
SIGNAL_POLL_SLICE_S = 0.05

# Default J1 joint-path waypoints matching qual-ref/tickload.py (B601-RS workspace).
DEFAULT_RPY = (0.0, math.radians(41.0), 0.0)
DEFAULT_POINT_A = (0.255, 0.08, 0.43)
DEFAULT_POINT_B = (0.255, 0.12, 0.43)

BLACKBOX_LOG_NOTE = (
    "No documented armd endpoint injects synthetic blackbox or log writes. "
    "Blackbox writes at 5 Hz automatically while the arm is torqued or moving "
    "(src/diagnostics/blackbox.rs); this load adds periodic "
    "GET /api/v1/diagnostics/blackbox?seconds=60 reads and HTTP TraceLayer log traffic."
)


def utc_iso(ts: float) -> str:
    return dt.datetime.fromtimestamp(ts, tz=dt.timezone.utc).isoformat(timespec="milliseconds")


def percentile_nearest(values: list[float], p: int) -> float | None:
    if not values:
        return None
    s = sorted(values)
    idx = max(1, (len(s) * p + 99) // 100) - 1
    return s[idx]


def extract_fault_info(obs: Any) -> tuple[str | None, str | None]:
    """Return (fault_str, safety_state_str) from either an ArmStatus or raw dict."""
    if obs is None:
        return None, None
    if isinstance(obs, dict):
        safety_raw = obs.get("safety_state")
        fault_raw = obs.get("fault")
        if isinstance(safety_raw, dict):
            safety_str = next(iter(safety_raw.keys()), str(safety_raw))
        elif safety_raw is None:
            safety_str = None
        else:
            safety_str = str(safety_raw)
        fault_str = None if fault_raw is None else (
            json.dumps(fault_raw, sort_keys=True) if isinstance(fault_raw, (dict, list)) else str(fault_raw)
        )
        if fault_str is None and safety_str in ("Fault", "EStop", "E_STOP", "FAULT"):
            fault_str = f"safety_state={safety_str}"
        return fault_str, safety_str

    safety_str = getattr(obs, "safety_state", None)
    fault_raw = getattr(obs, "fault", None)
    fault_str = None if fault_raw is None else str(fault_raw)
    if fault_str is None and (getattr(obs, "faulted", False) or getattr(obs, "e_stopped", False)):
        fault_str = f"safety_state={safety_str}"
    return fault_str, str(safety_str) if safety_str is not None else None


def extract_rx_ages_ms(obs: Any) -> list[float]:
    raw = obs.get("feedback_age_ms") if isinstance(obs, dict) else getattr(obs, "feedback_age_ms", None)
    if not isinstance(raw, list):
        return []
    out: list[float] = []
    for item in raw:
        if isinstance(item, (int, float)) and math.isfinite(item) and item >= 0:
            out.append(float(item))
    return out


class SharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Lock-free pending signal slot written ONLY by the OS signal handler so signal delivery
        # never acquires `_lock` or `Event._cond` (prevents same-thread re-entry deadlock).
        self._pending_signal: int | None = None
        # Signals background load workers and periodic tick sampler to wind down.
        self.workers_stop = threading.Event()
        # Set ONLY by SIGINT/SIGTERM (applied in normal control flow) or an arm fault (never set
        # by normal load completion), so the post-load idle tail waits for the full >= 1 s window.
        self.interrupt_event = threading.Event()
        self.phase: str = "init"
        self.fault: str | None = None
        self.safety_state: str | None = None
        self.stop_reason: str = "completed"
        self.stop_detail: str | None = None
        self.rx_ages_ms: list[float] = []
        self.counters: dict[str, Any] = {
            "obs_ok": 0,
            "obs_err": {},
            "ws_msgs": 0,
            "ws_bytes": 0,
            "ws_reconnects": 0,
            "ws_err": {},
            "measure_ok": 0,
            "measure_err": {},
            "blackbox_ok": 0,
            "blackbox_err": {},
            "bulk_ok": 0,
            "bulk_bytes": 0,
            "bulk_err": {},
            "motion_cycles_done": 0,
            "motion_moves_done": 0,
            "record_state": "disabled",
            "record_episode_id": None,
        }

    def record_signal_lockfree(self, signum: int) -> None:
        """Async-signal-safe recorder: touches NO locks and NO threading.Event primitives."""
        if self._pending_signal is None:
            self._pending_signal = signum

    def apply_pending_signal(self) -> bool:
        """Apply any signal recorded by `record_signal_lockfree` in normal control flow."""
        sig = self._pending_signal
        if sig is not None:
            self.request_interrupt(f"signal:{sig}")
            return True
        return False

    def is_workers_stopped(self) -> bool:
        if self._pending_signal is not None:
            return True
        return self.workers_stop.is_set()

    def is_interrupted(self) -> bool:
        if self._pending_signal is not None:
            return True
        return self.interrupt_event.is_set()

    def wait_workers_stop(self, timeout_s: float) -> bool:
        """Wait for `workers_stop` or a lock-free pending signal in bounded slices."""
        if self.apply_pending_signal() or self.workers_stop.is_set():
            return True
        end = time.monotonic() + max(0.0, timeout_s)
        while True:
            rem = end - time.monotonic()
            if rem <= 0.0:
                return self.apply_pending_signal() or self.workers_stop.is_set()
            if self.workers_stop.wait(min(rem, SIGNAL_POLL_SLICE_S)):
                self.apply_pending_signal()
                return True
            if self.apply_pending_signal():
                return True

    def wait_interrupt(self, timeout_s: float) -> bool:
        """Wait for `interrupt_event` or a lock-free pending signal in bounded slices."""
        if self.apply_pending_signal() or self.interrupt_event.is_set():
            return True
        end = time.monotonic() + max(0.0, timeout_s)
        while True:
            rem = end - time.monotonic()
            if rem <= 0.0:
                return self.apply_pending_signal() or self.interrupt_event.is_set()
            if self.interrupt_event.wait(min(rem, SIGNAL_POLL_SLICE_S)):
                self.apply_pending_signal()
                return True
            if self.apply_pending_signal():
                return True

    def set_stop_reason(self, reason: str, detail: str | None = None) -> None:
        if reason not in VALID_STOP_REASONS:
            raise ValueError(f"invalid stop_reason: {reason!r}")
        with self._lock:
            cur_prio = _STOP_REASON_PRIORITY[self.stop_reason]
            new_prio = _STOP_REASON_PRIORITY[reason]
            if new_prio > cur_prio or (new_prio == cur_prio and self.stop_detail is None and detail is not None):
                self.stop_reason = reason
                self.stop_detail = detail

    def get_stop_reason(self) -> tuple[str, str | None]:
        self.apply_pending_signal()
        with self._lock:
            return self.stop_reason, self.stop_detail

    def request_interrupt(self, detail: str) -> None:
        self.set_stop_reason("interrupted", detail)
        self.workers_stop.set()
        self.interrupt_event.set()

    def clear_safety_state(self) -> None:
        with self._lock:
            self.safety_state = None

    def snapshot_counters(self) -> dict[str, Any]:
        with self._lock:
            return {
                k: (dict(v) if isinstance(v, dict) else v)
                for k, v in self.counters.items()
            }

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self.phase = phase

    def get_phase(self) -> str:
        with self._lock:
            return self.phase

    def inc(self, key: str, delta: int = 1) -> None:
        with self._lock:
            self.counters[key] += delta

    def inc_err(self, key: str, err_code: str) -> None:
        with self._lock:
            bucket = self.counters[key]
            bucket[err_code] = bucket.get(err_code, 0) + 1

    def set_counter(self, key: str, value: Any) -> None:
        with self._lock:
            self.counters[key] = value

    def record_rx_ages(self, ages: list[float]) -> None:
        if not ages:
            return
        with self._lock:
            # Keep bounded memory even at high poll + WS rates (cap 20k values/interval).
            if len(self.rx_ages_ms) < 20_000:
                self.rx_ages_ms.extend(ages)

    def note_observation(self, obs: Any) -> bool:
        """Update safety_state / rx_ages; return True if fault tripped."""
        self.record_rx_ages(extract_rx_ages_ms(obs))
        fault_str, safety_str = extract_fault_info(obs)
        tripped = False
        with self._lock:
            if safety_str is not None:
                self.safety_state = safety_str
            if fault_str is not None and self.fault is None:
                self.fault = fault_str
                self.stop_reason = "fault"
                self.stop_detail = fault_str
                tripped = True
            has_fault = self.fault is not None
        if tripped:
            self.workers_stop.set()
            self.interrupt_event.set()
        return has_fault

    def trip_fault(self, reason: str) -> None:
        with self._lock:
            if self.fault is None:
                self.fault = reason
                self.stop_reason = "fault"
                self.stop_detail = reason
        self.workers_stop.set()
        self.interrupt_event.set()

    def has_fault(self) -> bool:
        with self._lock:
            return self.fault is not None

    def drain_interval_snapshot(self) -> tuple[str, dict[str, Any] | None, dict[str, Any], str | None, str | None]:
        with self._lock:
            phase = self.phase
            ages = self.rx_ages_ms
            self.rx_ages_ms = []
            counters_copy = {
                k: (dict(v) if isinstance(v, dict) else v)
                for k, v in self.counters.items()
            }
            fault = self.fault
            safety = self.safety_state
        rx_summary = None
        if ages:
            rx_summary = {
                "count": len(ages),
                "p50": round(percentile_nearest(ages, 50) or 0.0, 3),
                "p99": round(percentile_nearest(ages, 99) or 0.0, 3),
                "max": round(max(ages), 3),
            }
        return phase, rx_summary, counters_copy, fault, safety


def http_get_bytes(url: str, timeout: float) -> tuple[int, bytes]:
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return int(resp.status), resp.read()


def http_get_json(url: str, timeout: float) -> Any:
    _, raw = http_get_bytes(url, timeout)
    return json.loads(raw.decode("utf-8"))


def err_tag(exc: BaseException) -> str:
    if isinstance(exc, urllib.error.HTTPError):
        return f"HTTP_{exc.code}"
    code = getattr(exc, "code", None)
    if code:
        return str(code)
    return type(exc).__name__


def build_base_urls(host: str, port: int) -> tuple[str, str]:
    if host.startswith(("http://", "https://")):
        parsed = urllib.parse.urlsplit(host)
        scheme = parsed.scheme
        hostname = parsed.hostname or "127.0.0.1"
        eff_port = parsed.port or port
    else:
        scheme = "http"
        hostname = host
        eff_port = port
    http_base = f"{scheme}://{hostname}:{eff_port}"
    ws_scheme = "wss" if scheme == "https" else "ws"
    ws_base = f"{ws_scheme}://{hostname}:{eff_port}"
    return http_base, ws_base


def run_obs_poll(base_url: str, hz: float, deadline: float, state: SharedState) -> None:
    url = f"{base_url}/api/v1/observation"
    period = 1.0 / max(hz, 0.1)
    next_t = time.monotonic()
    while not state.workers_stop.is_set() and time.monotonic() < deadline:
        try:
            obs = http_get_json(url, timeout=min(1.5, max(0.5, period * 4)))
            state.inc("obs_ok")
            state.note_observation(obs)
        except Exception as exc:
            state.inc_err("obs_err", err_tag(exc))
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            state.workers_stop.wait(min(sleep_s, max(0.0, deadline - time.monotonic())))
        else:
            next_t = time.monotonic()


def run_ws_stream(ws_base: str, ws_obs_hz: int | None, deadline: float, state: SharedState) -> None:
    path = "/api/v1/ws" if ws_obs_hz is None else f"/api/v1/ws?obs_hz={ws_obs_hz}"
    ws_url = f"{ws_base}{path}"
    while not state.workers_stop.is_set() and time.monotonic() < deadline:
        ws = None
        try:
            ws = rebot_ws.connect(ws_url, timeout=3.0, origin=True)
            while not state.workers_stop.is_set() and time.monotonic() < deadline:
                try:
                    msg = ws.recv(timeout=0.5)
                except TimeoutError:
                    continue
                state.inc("ws_msgs")
                state.inc("ws_bytes", len(msg.encode("utf-8") if isinstance(msg, str) else msg))
                if isinstance(msg, str) and msg.startswith("{"):
                    try:
                        parsed = json.loads(msg)
                        if isinstance(parsed, dict) and "safety_state" in parsed:
                            state.note_observation(parsed)
                    except json.JSONDecodeError:
                        pass
        except Exception as exc:
            if not state.workers_stop.is_set():
                state.inc_err("ws_err", err_tag(exc))
                state.inc("ws_reconnects")
                state.workers_stop.wait(0.5)
        finally:
            if ws is not None:
                try:
                    ws.close()
                except Exception:
                    pass


def run_measure_loop(
    base_url: str,
    deadline: float,
    state: SharedState,
    operator_pin: str | None,
    operator_token: str | None,
    allow_insecure_auth: bool,
) -> None:
    client = ArmdClient(
        base_url,
        timeout_sec=5.0,
        operator_pin=operator_pin,
        operator_token=operator_token,
        allow_insecure_operator_auth=allow_insecure_auth,
    )
    while not state.workers_stop.is_set() and time.monotonic() < deadline:
        try:
            client.measure(timeout=15.0)
            state.inc("measure_ok")
        except Exception as exc:
            state.inc_err("measure_err", err_tag(exc))
            state.workers_stop.wait(0.3)


def run_blackbox_log_loop(base_url: str, hz: float, deadline: float, state: SharedState) -> None:
    url = f"{base_url}/api/v1/diagnostics/blackbox?seconds=60"
    period = 1.0 / max(hz, 0.1)
    while not state.workers_stop.is_set() and time.monotonic() < deadline:
        try:
            http_get_bytes(url, timeout=3.0)
            state.inc("blackbox_ok")
        except Exception as exc:
            state.inc_err("blackbox_err", err_tag(exc))
        state.workers_stop.wait(min(period, max(0.0, deadline - time.monotonic())))


def run_bulk_http_loop(base_url: str, bulk_path: str, interval_s: float, deadline: float, state: SharedState) -> None:
    path = bulk_path if bulk_path.startswith("/") else "/" + bulk_path
    url = f"{base_url}{path}"
    while not state.workers_stop.is_set() and time.monotonic() < deadline:
        try:
            _, data = http_get_bytes(url, timeout=3.0)
            state.inc("bulk_ok")
            state.inc("bulk_bytes", len(data))
        except Exception as exc:
            state.inc_err("bulk_err", err_tag(exc))
            state.workers_stop.wait(min(max(interval_s, 0.2), max(0.0, deadline - time.monotonic())))
            continue
        if interval_s > 0:
            state.workers_stop.wait(min(interval_s, max(0.0, deadline - time.monotonic())))


class JsonlWriter:
    """Thread-safe JSONL writer with sampler write revocation before terminal records."""

    def __init__(self, fh: Any) -> None:
        self._fh = fh
        self._lock = threading.Lock()
        self._sampler_closed = False

    def write_from_sampler(self, record: dict[str, Any]) -> bool:
        line = json.dumps(record, separators=(",", ":"), sort_keys=False)
        with self._lock:
            if self._sampler_closed:
                return False
            self._fh.write(line + "\n")
            self._fh.flush()
            return True

    def close_sampler_writes(self) -> None:
        with self._lock:
            self._sampler_closed = True

    def write_terminal(self, record: dict[str, Any]) -> None:
        line = json.dumps(record, separators=(",", ":"), sort_keys=False)
        with self._lock:
            self._fh.write(line + "\n")
            self._fh.flush()


def sample_tick_once(
    base_url: str,
    run_id: str,
    t0: float,
    state: SharedState,
    enabled_loads: dict[str, bool],
    phase_override: str | None = None,
) -> dict[str, Any]:
    now_unix = time.time()
    elapsed = time.monotonic() - t0
    phase, rx_summary, counters, fault, safety = state.drain_interval_snapshot()
    if phase_override is not None:
        phase = phase_override
    tick_data: dict[str, Any] | None = None
    tick_err: str | None = None
    try:
        tick_data = http_get_json(f"{base_url}/api/v1/diagnostics/tick", timeout=2.0)
    except Exception as exc:
        tick_err = f"{err_tag(exc)}: {exc}"
    return {
        "type": "sample",
        "run_id": run_id,
        "ts_unix": round(now_unix, 6),
        "ts_iso": utc_iso(now_unix),
        "elapsed_s": round(elapsed, 6),
        "phase": phase,
        "enabled_loads": enabled_loads,
        "tick": tick_data,
        "tick_error": tick_err,
        "rx_age_ms": rx_summary,
        "counters": counters,
        "fault": fault,
        "safety_state": safety,
    }


def run_tick_sampler(
    base_url: str,
    run_id: str,
    t0: float,
    interval_s: float,
    deadline: float,
    state: SharedState,
    enabled_loads: dict[str, bool],
    writer: JsonlWriter,
    skip_initial: bool = False,
) -> None:
    if not skip_initial:
        rec = sample_tick_once(base_url, run_id, t0, state, enabled_loads)
        if not writer.write_from_sampler(rec):
            return
    next_t = time.monotonic() + interval_s
    while not state.workers_stop.is_set() and time.monotonic() < deadline:
        wait_s = min(max(0.0, next_t - time.monotonic()), max(0.0, deadline - time.monotonic()))
        if wait_s > 0:
            state.workers_stop.wait(wait_s)
        if state.workers_stop.is_set() or time.monotonic() >= deadline:
            break
        rec = sample_tick_once(base_url, run_id, t0, state, enabled_loads)
        if not writer.write_from_sampler(rec):
            break
        next_t += interval_s
        if next_t < time.monotonic():
            next_t = time.monotonic() + interval_s


def infer_nominal_period_s(tick: Any) -> float:
    if isinstance(tick, dict):
        p = tick.get("period_us")
        if isinstance(p, dict):
            p50 = p.get("p50")
            if isinstance(p50, int) and not isinstance(p50, bool) and p50 > 0:
                return max(0.001, min(1.0, p50 / 1_000_000.0))
    return 0.02


def extract_releases_total(tick: Any) -> int | None:
    """Strictly extract monotonic `releases_total` (never falls back to `since_boot_ticks`)."""
    if not isinstance(tick, dict):
        return None
    v = tick.get("releases_total")
    if isinstance(v, int) and not isinstance(v, bool) and v >= 0:
        return int(v)
    return None


def missing_requested_load_activity(enabled_loads: dict[str, bool], counters: dict[str, Any]) -> list[str]:
    """Return names of enabled loads that recorded zero execution activity."""
    missing: list[str] = []
    if enabled_loads.get("obs_poll"):
        if not (isinstance(counters.get("obs_ok"), int) and counters["obs_ok"] > 0):
            missing.append("obs_poll")
    if enabled_loads.get("ws_stream"):
        ws_ok = (counters.get("ws_msgs") or 0) > 0 or (counters.get("ws_reconnects") or 0) > 0 or bool(counters.get("ws_err"))
        if not ws_ok:
            missing.append("ws_stream")
    if enabled_loads.get("measure"):
        if not ((counters.get("measure_ok") or 0) > 0 or bool(counters.get("measure_err"))):
            missing.append("measure")
    if enabled_loads.get("blackbox_log"):
        if not ((counters.get("blackbox_ok") or 0) > 0 or bool(counters.get("blackbox_err"))):
            missing.append("blackbox_log")
    if enabled_loads.get("bulk_http"):
        if not ((counters.get("bulk_ok") or 0) > 0 or bool(counters.get("bulk_err"))):
            missing.append("bulk_http")
    if enabled_loads.get("motion"):
        if not ((counters.get("motion_moves_done") or 0) > 0 or (counters.get("motion_cycles_done") or 0) > 0):
            missing.append("motion")
    if enabled_loads.get("record"):
        if counters.get("record_state") in ("disabled", "pending", None):
            missing.append("record")
    return missing


def make_signal_handler(state: SharedState) -> Any:
    """Return an async-signal-safe handler that never acquires Python locks or Event primitives."""
    def _on_signal(signum: int, _frame: Any) -> None:
        state.record_signal_lockfree(signum)

    return _on_signal


def capture_settled_final_sample(
    base_url: str,
    run_id: str,
    t0: float,
    state: SharedState,
    enabled_loads: dict[str, bool],
    max_attempts: int = 8,
    settle_sleep_s: float = 0.005,
) -> dict[str, Any]:
    """Capture the terminal `final` diagnostics sample, settling briefly if an ordinary
    in-flight release (< 20 ms) is active when the poll fires so `releases_total == releases_started`
    and `in_flight_release_us == 0`. Immediately stops if a release is stalled >= 20 ms."""
    sample = sample_tick_once(base_url, run_id, t0, state, enabled_loads, phase_override="final")
    for _ in range(max_attempts - 1):
        if state.is_interrupted():
            state.apply_pending_signal()
            break
        tick = sample.get("tick")
        if not isinstance(tick, dict) or sample.get("tick_error"):
            break
        started = tick.get("releases_started")
        total = tick.get("releases_total")
        in_flight = tick.get("in_flight_release_us")
        in_flight_int = in_flight if isinstance(in_flight, int) and not isinstance(in_flight, bool) else None
        if in_flight_int is not None and in_flight_int >= 20_000:
            break
        has_open_count = (
            isinstance(started, int)
            and not isinstance(started, bool)
            and isinstance(total, int)
            and not isinstance(total, bool)
            and started > total
        )
        has_active_in_flight = in_flight_int is not None and in_flight_int > 0
        if not (has_open_count or has_active_in_flight):
            break
        if state.wait_interrupt(settle_sleep_s):
            break
        sample = sample_tick_once(base_url, run_id, t0, state, enabled_loads, phase_override="final")
    return sample


def check_arm_state(arm: Arm, state: SharedState) -> bool:
    """Return True if healthy, False if faulted or stopped."""
    if state.is_workers_stopped():
        state.apply_pending_signal()
        return False
    st = arm.state()
    if state.note_observation(st):
        return False
    return True


def execute_one_cycle(
    arm: Arm,
    motion_mode: str,
    speed_scale: float,
    dwell_s: float,
    j1_delta_rad: float,
    deadline: float,
    state: SharedState,
    home_joints: list[float] | None,
) -> bool:
    """Run one bounded motion cycle. Returns False if stopped/faulted/deadline."""
    if state.is_workers_stopped() or time.monotonic() >= deadline:
        state.apply_pending_signal()
        return False
    if motion_mode == "tickload":
        targets = (DEFAULT_POINT_B, DEFAULT_POINT_A)
        for tgt in targets:
            if not check_arm_state(arm, state) or time.monotonic() >= deadline:
                return False
            arm.move_to(tgt, rpy=DEFAULT_RPY, yaw="outward", path="joint", speed_scale=speed_scale)
            state.inc("motion_moves_done")
            if not check_arm_state(arm, state):
                return False
            if dwell_s > 0:
                state.wait_workers_stop(min(dwell_s, max(0.0, deadline - time.monotonic())))
    else:
        assert home_joints is not None
        p_plus = list(home_joints)
        p_minus = list(home_joints)
        p_plus[0] += j1_delta_rad
        p_minus[0] -= j1_delta_rad
        # Duration scales inversely with speed_scale (safe slow default >= 2.0 s).
        dur = max(1.5, 0.6 / max(speed_scale, 0.05))
        for tgt_j in (p_plus, p_minus):
            if not check_arm_state(arm, state) or time.monotonic() >= deadline:
                return False
            arm.move_joints(tgt_j, duration=dur)
            state.inc("motion_moves_done")
            if not check_arm_state(arm, state):
                return False
            if dwell_s > 0:
                state.wait_workers_stop(min(dwell_s, max(0.0, deadline - time.monotonic())))
    state.inc("motion_cycles_done")
    return True


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="SG2002 RT qualification load driver (Stage 3 §6).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--host", default=os.environ.get("REBOT_HOST", "127.0.0.1"), help="armd host or URL")
    p.add_argument("--port", type=int, default=8765, help="armd HTTP/WS port")
    p.add_argument("--run-id", default=None, help="Run identifier (default: UTC timestamp)")
    p.add_argument("--output", required=True, help="Output JSONL path")
    p.add_argument("--duration", type=float, default=120.0, help="Maximum total run duration in seconds")
    p.add_argument("--sample-interval", type=float, default=5.0, help="Seconds between GET /api/v1/diagnostics/tick samples")
    p.add_argument("--baseline-s", type=float, default=5.0, help="Idle baseline phase duration before starting load (0 disables)")
    p.add_argument("--cooldown-s", type=float, default=5.0, help="Post-motion cooldown duration before final sample")

    # Toggleable load pieces (all enabled by default).
    p.add_argument("--motion", action=argparse.BooleanOptionalAction, default=True, help="J1 joint moves (bounded cycles)")
    p.add_argument("--obs-poll", action=argparse.BooleanOptionalAction, default=True, help="20 Hz GET /api/v1/observation")
    p.add_argument("--ws-stream", action=argparse.BooleanOptionalAction, default=True, help="WebSocket /api/v1/ws observation subscriber")
    p.add_argument("--measure", action=argparse.BooleanOptionalAction, default=True, help="Continuous single-view measure loop")
    p.add_argument("--record", action=argparse.BooleanOptionalAction, default=True, help="Record one episode via arm.record()")
    p.add_argument("--blackbox-log", action=argparse.BooleanOptionalAction, default=True, help="Periodic blackbox reads + TraceLayer log pressure")
    p.add_argument("--bulk-http", action=argparse.BooleanOptionalAction, default=True, help="Bulk HTTP download loop (USB gadget traffic)")

    # Rates & motion parameters.
    p.add_argument("--speed-scale", type=float, default=0.3, help="Motion speed_scale (hard cap <= 0.3)")
    p.add_argument("--cycles", type=int, default=4, help="Maximum bounded motion cycles")
    p.add_argument("--dwell-s", type=float, default=2.0, help="Dwell seconds between waypoints")
    p.add_argument(
        "--motion-mode",
        choices=("tickload", "joint-j1"),
        default="tickload",
        help="tickload: Cartesian A/B joint-path moves as in qual-ref/tickload.py; joint-j1: direct J1 joint moves around home",
    )
    p.add_argument("--j1-delta-rad", type=float, default=0.15, help="J1 excursion (rad) when --motion-mode=joint-j1")
    p.add_argument("--obs-hz", type=float, default=20.0, help="GET /api/v1/observation polling rate (Hz)")
    p.add_argument("--ws-obs-hz", type=int, default=None, help="Optional ?obs_hz=N for WS subscriber (default: every tick)")
    p.add_argument("--blackbox-hz", type=float, default=1.0, help="GET /api/v1/diagnostics/blackbox polling rate (Hz)")
    p.add_argument("--bulk-path", default="/api/v1/camera/snapshot", help="HTTP path for bulk download loop")
    p.add_argument("--bulk-interval-s", type=float, default=0.05, help="Sleep seconds between bulk HTTP downloads")
    p.add_argument("--record-dataset", default="rt_qual", help="Dataset name for recorded episode")
    p.add_argument("--record-task", default="rt-qual-harness", help="Task label for recorded episode")
    p.add_argument("--record-seconds", type=float, default=15.0, help="Recording window when --no-motion is set")
    p.add_argument(
        "--allow-insecure-auth",
        action="store_true",
        default=os.environ.get("REBOT_ALLOW_INSECURE_OPERATOR_AUTH", "").lower() in ("1", "true", "yes"),
        help="Allow sending operator PIN/token over plain http:// to non-loopback host",
    )
    p.add_argument("--force-lease", action="store_true", default=False, help="Force-take arm lease if another client holds it")
    args = p.parse_args(argv)

    if not (0.0 < args.speed_scale <= MAX_SPEED_SCALE):
        p.error(f"--speed-scale must be in (0, {MAX_SPEED_SCALE}], got {args.speed_scale}")
    if args.duration <= 0:
        p.error("--duration must be > 0")
    if args.sample_interval <= 0:
        p.error("--sample-interval must be > 0")
    if args.cycles < 1:
        p.error("--cycles must be >= 1")
    if args.obs_hz <= 0:
        p.error("--obs-hz must be > 0")
    if args.blackbox_hz <= 0:
        p.error("--blackbox-hz must be > 0")
    if args.run_id is None:
        args.run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return args


def fetch_boot_meta(base_url: str) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    try:
        boots = http_get_json(f"{base_url}/api/v1/diagnostics/boots", timeout=2.0)
        meta["boots_enabled"] = boots.get("enabled")
        meta["boot"] = boots.get("boot")
        entries = boots.get("boots") or []
        if entries:
            latest = entries[-1]
            meta["armd_version"] = latest.get("armd_version")
            meta["kernel_boot_id"] = latest.get("kernel_boot_id")
            meta["reset_reason"] = latest.get("reset_reason")
    except Exception as exc:
        meta["boots_error"] = err_tag(exc)
    try:
        schema = http_get_json(f"{base_url}/api/v1/observation/schema", timeout=2.0)
        meta["dof"] = schema.get("dof")
        meta["joint_names"] = schema.get("joint_names")
    except Exception as exc:
        meta["schema_error"] = err_tag(exc)
    return meta


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not HAS_REBOT_SDK and (args.motion or args.record or args.ws_stream or args.measure):
        raise RuntimeError(
            "rebot SDK is required when --motion, --record, --ws-stream, or --measure is enabled"
        )
    http_base, ws_base = build_base_urls(args.host, args.port)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    enabled_loads = {
        "motion": args.motion,
        "obs_poll": args.obs_poll,
        "ws_stream": args.ws_stream,
        "measure": args.measure,
        "record": args.record,
        "blackbox_log": args.blackbox_log,
        "bulk_http": args.bulk_http,
    }
    state = SharedState()
    if args.record:
        state.set_counter("record_state", "pending")

    on_signal = make_signal_handler(state)
    prev_sigint = signal.signal(signal.SIGINT, on_signal)
    prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

    t0 = time.monotonic()
    deadline = t0 + args.duration
    started_unix = time.time()
    boot_meta = fetch_boot_meta(http_base)
    operator_pin = os.environ.get("REBOT_OPERATOR_PIN")
    operator_token = os.environ.get("REBOT_OPERATOR_TOKEN")

    workers: list[threading.Thread] = []
    sampler_thread: threading.Thread | None = None
    idle_wait_s = MIN_IDLE_TAIL_S
    idle_elapsed_s = 0.0
    requested_any_load = any(enabled_loads.values())
    load_entered = False
    terminal_safety_verified = False
    terminal_observation_error: str | None = None

    try:
        with out_path.open("w", encoding="utf-8") as fh:
            writer = JsonlWriter(fh)
            meta_record = {
                "type": "meta",
                "run_id": args.run_id,
                "ts_unix": round(started_unix, 6),
                "ts_iso": utc_iso(started_unix),
                "target": http_base,
                "duration_s": args.duration,
                "sample_interval_s": args.sample_interval,
                "enabled_loads": enabled_loads,
                "rates": {
                    "obs_hz": args.obs_hz if args.obs_poll else 0,
                    "ws_obs_hz": args.ws_obs_hz if args.ws_stream else 0,
                    "blackbox_hz": args.blackbox_hz if args.blackbox_log else 0,
                    "bulk_interval_s": args.bulk_interval_s if args.bulk_http else None,
                    "bulk_path": args.bulk_path if args.bulk_http else None,
                },
                "motion_config": {
                    "mode": args.motion_mode,
                    "speed_scale": args.speed_scale,
                    "cycles": args.cycles,
                    "dwell_s": args.dwell_s,
                },
                "armd": boot_meta,
                "blackbox_log_note": BLACKBOX_LOG_NOTE,
            }
            writer.write_terminal(meta_record)

            # Capture a synchronous baseline sample BEFORE starting any load workers (even when
            # --baseline-s=0) so early workload spikes cannot be absorbed into samples[0].
            state.set_phase("baseline")
            baseline_sample = sample_tick_once(http_base, args.run_id, t0, state, enabled_loads, phase_override="baseline")
            writer.write_terminal(baseline_sample)
            baseline_ok = isinstance(baseline_sample.get("tick"), dict) and not baseline_sample.get("tick_error")
            nominal_period_s = infer_nominal_period_s(baseline_sample.get("tick"))
            idle_wait_s = max(MIN_IDLE_TAIL_S, IDLE_TAIL_PERIOD_MULTIPLIER * nominal_period_s)
            if not baseline_ok:
                state.set_stop_reason("error", f"baseline_poll_failed:{baseline_sample.get('tick_error') or 'no_tick'}")
                state.workers_stop.set()
            else:
                state.set_phase("baseline" if args.baseline_s > 0 else "full_load")
                sampler_thread = threading.Thread(
                    target=run_tick_sampler,
                    args=(
                        http_base,
                        args.run_id,
                        t0,
                        args.sample_interval,
                        deadline,
                        state,
                        enabled_loads,
                        writer,
                        True,
                    ),
                    name="tick-sampler",
                    daemon=True,
                )
                sampler_thread.start()

            try:
                if args.baseline_s > 0 and not state.is_workers_stopped():
                    state.wait_workers_stop(min(args.baseline_s, max(0.0, deadline - time.monotonic())))

                state.apply_pending_signal()
                if requested_any_load and not state.is_workers_stopped() and time.monotonic() >= deadline:
                    state.set_stop_reason("error", "deadline_expired_before_load_start")
                    state.workers_stop.set()

                # Spawn background load threads.
                if not state.is_workers_stopped() and time.monotonic() < deadline:
                    load_entered = True
                    if args.obs_poll:
                        workers.append(threading.Thread(
                            target=run_obs_poll, args=(http_base, args.obs_hz, deadline, state),
                            name="obs-poll", daemon=True,
                        ))
                    if args.ws_stream:
                        workers.append(threading.Thread(
                            target=run_ws_stream, args=(ws_base, args.ws_obs_hz, deadline, state),
                            name="ws-stream", daemon=True,
                        ))
                    if args.measure:
                        workers.append(threading.Thread(
                            target=run_measure_loop,
                            args=(http_base, deadline, state, operator_pin, operator_token, args.allow_insecure_auth),
                            name="measure-loop", daemon=True,
                        ))
                    if args.blackbox_log:
                        workers.append(threading.Thread(
                            target=run_blackbox_log_loop, args=(http_base, args.blackbox_hz, deadline, state),
                            name="blackbox-log", daemon=True,
                        ))
                    if args.bulk_http:
                        workers.append(threading.Thread(
                            target=run_bulk_http_loop,
                            args=(http_base, args.bulk_path, args.bulk_interval_s, deadline, state),
                            name="bulk-http", daemon=True,
                        ))
                    for w in workers:
                        w.start()

                # Foreground motion + recording inside `with Arm(...)`.
                needs_arm = args.motion or args.record
                if needs_arm and not state.is_workers_stopped() and time.monotonic() < deadline:
                    arm_kwargs: dict[str, Any] = {
                        "host": http_base,
                        "enable": args.motion,
                        "park_on_exit": args.motion,
                        "force": args.force_lease,
                        "handle_sigterm": False,
                        "operator_pin": operator_pin,
                        "operator_token": operator_token,
                        "allow_insecure_operator_auth": args.allow_insecure_auth,
                    }
                    try:
                        with Arm(**arm_kwargs) as arm:
                            if not check_arm_state(arm, state):
                                raise RuntimeError(f"arm faulted on connect: {state.fault}")

                            home_joints: list[float] | None = None
                            if args.motion:
                                state.set_phase("motion_setup")
                                arm.home()
                                if not check_arm_state(arm, state):
                                    raise RuntimeError(f"arm faulted after home: {state.fault}")
                                home_joints = list(arm.state().joint_positions)
                                if args.motion_mode == "tickload" and time.monotonic() < deadline:
                                    arm.move_to(
                                        DEFAULT_POINT_A,
                                        rpy=DEFAULT_RPY,
                                        yaw="outward",
                                        path="joint",
                                        speed_scale=args.speed_scale,
                                    )
                                    state.inc("motion_moves_done")
                                    if not check_arm_state(arm, state):
                                        raise RuntimeError(f"arm faulted after move A: {state.fault}")
                                    if args.dwell_s > 0:
                                        state.wait_workers_stop(min(args.dwell_s, max(0.0, deadline - time.monotonic())))

                            cycle_idx = 0
                            # Record one episode (wraps cycle 0 when motion is enabled, else timed window).
                            if args.record and not state.is_workers_stopped() and time.monotonic() < deadline:
                                state.set_phase("motion_record" if args.motion else "record_load")
                                try:
                                    arm.create_dataset(args.record_dataset, exist_ok=True)
                                    with arm.record(dataset=args.record_dataset, task=args.record_task) as rec:
                                        state.set_counter("record_state", "recording")
                                        if args.motion and cycle_idx < args.cycles:
                                            ok = execute_one_cycle(
                                                arm, args.motion_mode, args.speed_scale, args.dwell_s,
                                                args.j1_delta_rad, deadline, state, home_joints,
                                            )
                                            cycle_idx += 1
                                            if not ok and state.has_fault():
                                                raise RuntimeError(f"arm faulted during recorded cycle: {state.fault}")
                                        else:
                                            rec_wait = min(args.record_seconds, max(0.0, deadline - time.monotonic()))
                                            state.wait_workers_stop(rec_wait)
                                    state.set_counter("record_episode_id", rec.episode_id)
                                    state.set_counter("record_state", "saved" if rec.episode_id else "discarded")
                                except RebotError as exc:
                                    # Non-fault recording unavailability (e.g. --episodes-dir off) is noted, not a fault.
                                    state.set_counter("record_state", f"unavailable:{err_tag(exc)}")
                                    if not check_arm_state(arm, state):
                                        raise

                            # Remaining motion cycles under full load.
                            if args.motion:
                                state.set_phase("full_load")
                                while cycle_idx < args.cycles and not state.is_workers_stopped() and time.monotonic() < deadline:
                                    ok = execute_one_cycle(
                                        arm, args.motion_mode, args.speed_scale, args.dwell_s,
                                        args.j1_delta_rad, deadline, state, home_joints,
                                    )
                                    cycle_idx += 1
                                    if not ok:
                                        break
                                if not state.has_fault() and not state.is_workers_stopped():
                                    state.set_phase("cooldown")
                                    arm.home()
                                    check_arm_state(arm, state)
                                    if args.cooldown_s > 0:
                                        state.wait_workers_stop(min(args.cooldown_s, max(0.0, deadline - time.monotonic())))
                            else:
                                state.set_phase("full_load")
                                remaining = max(0.0, deadline - time.monotonic())
                                if remaining > 0:
                                    state.wait_workers_stop(remaining)
                    except RebotError as exc:
                        # Check whether the arm faulted; never clear faults.
                        try:
                            obs = http_get_json(f"{http_base}/api/v1/observation", timeout=1.5)
                            state.note_observation(obs)
                        except Exception:
                            pass
                        if not state.has_fault():
                            state.trip_fault(f"{err_tag(exc)}: {exc}")
                    except RuntimeError as exc:
                        if not state.has_fault():
                            state.trip_fault(str(exc))
                elif not state.is_workers_stopped():
                    state.set_phase("full_load")
                    remaining = max(0.0, deadline - time.monotonic())
                    if remaining > 0:
                        state.wait_workers_stop(remaining)
            except KeyboardInterrupt:
                state.request_interrupt("KeyboardInterrupt")
            except Exception as exc:
                state.set_stop_reason("error", f"load_exception:{err_tag(exc)}: {exc}")

            state.apply_pending_signal()

            # Stop all background loads and periodic sampler cleanly, then revoke sampler writes
            # so a timed-out background sampler can never append records after idle_tail_start/final/summary.
            state.workers_stop.set()
            for w in workers:
                w.join(timeout=3.0)
            if sampler_thread is not None:
                sampler_thread.join(timeout=3.0)
            writer.close_sampler_writes()

            alive_threads = [t.name for t in workers if t.is_alive()]
            if sampler_thread is not None and sampler_thread.is_alive():
                alive_threads.append(sampler_thread.name)
            if alive_threads:
                state.set_stop_reason("error", f"threads_still_alive:{','.join(alive_threads)}")

            if state.get_stop_reason()[0] == "completed" and requested_any_load:
                if not load_entered:
                    state.set_stop_reason("error", "deadline_expired_before_load_start")
                else:
                    missing_loads = missing_requested_load_activity(enabled_loads, state.snapshot_counters())
                    if missing_loads:
                        state.set_stop_reason("error", f"requested_load_no_activity:{','.join(missing_loads)}")

            # Post-load idle release tail:
            # 1. Verify safety observation before entering idle tail (fail-closed on errors).
            # 2. Persist explicit `idle_tail_start` diagnostics boundary sample.
            # 3. Wait >= max(1.0 s, 2 * period * 10) of idle releases on `interrupt_event`.
            # 4. Verify terminal safety observation strictly AFTER idle tail (fail-closed; never
            #    retain stale healthy safety_state if the terminal observation request fails).
            # 5. Capture settled `final` sample and verify `releases_total` progressed across the tail.
            pre_idle_total: int | None = None
            pre_idle_elapsed_s: float | None = None
            reason_now, _ = state.get_stop_reason()
            if reason_now == "completed":
                try:
                    obs_pre = http_get_json(f"{http_base}/api/v1/observation", timeout=1.5)
                    state.note_observation(obs_pre)
                    _, pre_safety = extract_fault_info(obs_pre)
                    if pre_safety is None:
                        state.set_stop_reason("error", "pre_idle_observation_missing_safety_state")
                except Exception as exc:
                    state.clear_safety_state()
                    state.set_stop_reason("error", f"pre_idle_observation_failed:{err_tag(exc)}: {exc}")
                reason_now, _ = state.get_stop_reason()

            if reason_now == "completed":
                state.set_phase("idle_tail")
                pre_idle_sample = sample_tick_once(
                    http_base, args.run_id, t0, state, enabled_loads, phase_override="idle_tail_start"
                )
                idle_start = time.monotonic()
                writer.write_terminal(pre_idle_sample)
                if pre_idle_sample.get("tick_error") or not isinstance(pre_idle_sample.get("tick"), dict):
                    state.set_stop_reason(
                        "error",
                        f"pre_idle_poll_failed:{pre_idle_sample.get('tick_error') or 'no_tick'}",
                    )
                else:
                    pre_idle_total = extract_releases_total(pre_idle_sample.get("tick"))
                    pre_idle_elapsed_s = pre_idle_sample.get("elapsed_s")
                    if pre_idle_total is None:
                        state.set_stop_reason("error", "pre_idle_missing_releases_total")
                reason_now, _ = state.get_stop_reason()

                if reason_now == "completed":
                    idle_deadline = idle_start + idle_wait_s
                    try:
                        while not state.is_interrupted():
                            rem = idle_deadline - time.monotonic()
                            if rem <= 0.0:
                                break
                            state.wait_interrupt(rem)
                    except KeyboardInterrupt:
                        state.request_interrupt("KeyboardInterrupt")
                    state.apply_pending_signal()
                    idle_elapsed_s = time.monotonic() - idle_start
                    reason_now, _ = state.get_stop_reason()
                    if reason_now == "completed" and idle_elapsed_s < idle_wait_s:
                        state.set_stop_reason(
                            "error",
                            f"idle_tail_short:{idle_elapsed_s:.3f}s<{idle_wait_s:.3f}s",
                        )

            # Always poll terminal safety state after idle tail / shutdown; never swallow failures.
            try:
                obs_post = http_get_json(f"{http_base}/api/v1/observation", timeout=1.5)
                state.note_observation(obs_post)
                _, post_safety = extract_fault_info(obs_post)
                if post_safety is not None:
                    terminal_safety_verified = True
                else:
                    terminal_observation_error = "missing_safety_state"
                    state.clear_safety_state()
                    state.set_stop_reason("error", "terminal_observation_missing_safety_state")
            except Exception as exc:
                terminal_observation_error = f"{err_tag(exc)}: {exc}"
                state.clear_safety_state()
                state.set_stop_reason("error", f"terminal_observation_failed:{err_tag(exc)}: {exc}")

            final_sample = capture_settled_final_sample(http_base, args.run_id, t0, state, enabled_loads)
            if state.get_stop_reason()[0] == "completed":
                if final_sample.get("tick_error") or not isinstance(final_sample.get("tick"), dict):
                    state.set_stop_reason(
                        "error",
                        f"final_poll_failed:{final_sample.get('tick_error') or 'no_tick'}",
                    )
                else:
                    final_total = extract_releases_total(final_sample.get("tick"))
                    final_elapsed_s = final_sample.get("elapsed_s")
                    if (
                        pre_idle_total is None
                        or final_total is None
                        or (final_total - pre_idle_total) < MIN_IDLE_TAIL_RELEASES
                    ):
                        state.set_stop_reason(
                            "error",
                            f"idle_tail_insufficient_release_progress:{pre_idle_total}->{final_total}",
                        )
                    elif (
                        not isinstance(pre_idle_elapsed_s, (int, float))
                        or not isinstance(final_elapsed_s, (int, float))
                        or (final_elapsed_s - pre_idle_elapsed_s) < idle_wait_s
                    ):
                        state.set_stop_reason(
                            "error",
                            f"idle_tail_boundary_elapsed_short:{pre_idle_elapsed_s}->{final_elapsed_s}",
                        )
            writer.write_terminal(final_sample)

            blackbox_tail = None
            if state.has_fault():
                try:
                    blackbox_tail = http_get_json(f"{http_base}/api/v1/diagnostics/blackbox?seconds=30", timeout=2.0)
                except Exception:
                    blackbox_tail = None

            final_stop_reason, final_stop_detail = state.get_stop_reason()
            summary = {
                "type": "summary",
                "run_id": args.run_id,
                "ts_unix": round(time.time(), 6),
                "ts_iso": utc_iso(time.time()),
                "elapsed_s": round(time.monotonic() - t0, 6),
                "stop_reason": final_stop_reason,
                "stop_detail": final_stop_detail,
                "idle_tail_target_s": round(idle_wait_s, 3),
                "idle_tail_elapsed_s": round(idle_elapsed_s, 3),
                "terminal_safety_verified": terminal_safety_verified,
                "terminal_observation_error": terminal_observation_error,
                "fault": state.fault,
                "safety_state": state.safety_state,
                "counters": final_sample["counters"],
                "blackbox_tail_on_fault": blackbox_tail,
            }
            writer.write_terminal(summary)
    finally:
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)

    final_stop_reason, final_stop_detail = state.get_stop_reason()
    print(
        f"rt_qual load finished: run_id={args.run_id} elapsed={time.monotonic() - t0:.1f}s "
        f"stop_reason={final_stop_reason} stop_detail={final_stop_detail} "
        f"idle_tail={idle_elapsed_s:.2f}s/{idle_wait_s:.2f}s fault={state.fault} output={out_path}"
    )
    if final_stop_reason == "fault":
        return 2
    if final_stop_reason == "interrupted":
        return 130
    if final_stop_reason == "error":
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
