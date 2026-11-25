# Adaptive Grasp Phase B — Companion SDK & MCP Patch Specification

This document specifies the exact, `git apply`-verified changes for `/home/harvest/project/rebot-companion` (`rebot/client.py`, `rebot/arm.py`, `rebot_companion/mcp_server.py`) to expose `rebot-armd` Phase B adaptive grasp (`strategy="adaptive"`) while keeping legacy grasp as the default.

- **Target companion base commit**: `a863f9e` (`/home/harvest/project/rebot-companion`)
- **Scope note**: `rebot-companion` lives in a separate repository and is not edited from the `rebot-armd` worktree. Apply each fenced diff block below in `/home/harvest/project/rebot-companion` with `git apply`.

---

## 1. Wire Contract (`rebot-armd` Phase B)

### 1.1 Capabilities (`/api/v1/observation/schema`)

| Capability | Meaning | Profiles |
|---|---|---|
| `capabilities.grasp` | Legacy visual-servo grasp available | 5-DOF and 6-DOF profiles with gripper + grasp setup |
| `capabilities.grasp_select` | Adaptive candidate selection (`dry_run: true`, `strategy: "adaptive"`) | 6-DOF profiles with gripper + grasp setup |
| `capabilities.grasp_select_exec` | Live adaptive grasp execution (`strategy: "adaptive"`) | 6-DOF profiles with gripper + grasp setup (`false` on 5-DOF and `wasm32`) |

### 1.2 Transport & Request Body (`/api/v1/action/grasp`)

- **Live execution (`dry_run` omitted or `false`)**: WebSocket `/api/v1/ws` command (`path: "/api/v1/action/grasp"`) under the active control lease. REST `POST /api/v1/action/grasp` without `dry_run: true` returns HTTP `409` (`LEASE_REQUIRED`).
- **Dry-run (`dry_run: true`)**: Lease-free over REST `POST /api/v1/action/grasp` or WebSocket `/api/v1/ws` command (`path: "/api/v1/action/grasp"`).
- Existing fields remain unchanged; `"legacy"` remains the default when `strategy` is omitted.

Example adaptive request body (validated directly against `armd`'s `grasp::parse_grasp`, `grasp_select::parse_strategy`, `grasp_select::snapshot::parse_evidence`, `grasp_select::snapshot::parse_preferences`, and `grasp_select::snapshot::parse_budget`):

```json
{
  "label": "banana",
  "box_norm": [0.35, 0.40, 0.65, 0.58],
  "frame_id": 7,
  "ts_ms": 1730000000000.0,
  "dry_run": false,
  "strategy": "adaptive",
  "evidence": {
    "contour": [[0.36, 0.42], [0.64, 0.45], [0.62, 0.56], [0.38, 0.54]],
    "height_m": [0.032, 0.038],
    "payload_kg": 0.15
  },
  "pose_preferences": {
    "vertical_weight": 0.3,
    "approach_side": "any",
    "preferred_max_tilt_deg": 45.0,
    "hard": {
      "max_tilt_deg": 70.0
    }
  },
  "budget_ms": 500
}
```

Allowed `pose_preferences` keys:
- `vertical_weight`: float in `[0.0, 2.0]` (default `0.3`)
- `approach_side`: `"any" | "outward" | "inward" | "lateral"` (default `"any"`)
- `preferred_max_tilt_deg`: optional float in `[0.0, 90.0]`
- `hard`: optional object containing only `max_tilt_deg` (float in `[0.0, 90.0]`)

### 1.3 Dry-Run Reply (`dry_run: true`, `strategy: "adaptive"`)

Legacy (`strategy` omitted or `"legacy"`) returns the exact existing keys `{reachable, code, message}` (+ optional `task_supported`, `task_reason`).

Adaptive (`strategy: "adaptive"`) returns:
- `reachable: bool`
- `code: str | null`
- `message: str | null`
- `strategy: "adaptive"`
- `outcome: "selected" | "infeasible" | "insufficient_evidence" | "budget_exhausted"`
- `failing_phase: str | null` (present on failure, e.g. `"evidence"`, `"acquisition"`, `"transition"`, `"pre_open"`, `"insertion"`, `"closure"`, `"separation"`, `"lift"`, `"withdrawal"`, `"budget"`, `"selection"`)
- `hint: str` (present **only** when a checked alternative exists — e.g. checked legacy verdict or checked candidate with relaxed preference; omitted when no alternative was verified)
- `pregrasp_mode: "cartesian" | "swept_joint"` (when `outcome == "selected"`)
- `expected_duration_s: float` (when `outcome == "selected"`)
- `budget: {pregrasp_s, start_vision_s, pre_open_s, pregrasp_vision_s, insert_s, close_s, close_timeout_s, is_pos_force_close, separation_s, lift_s, withdrawal_s, post_lift_settle_s, nominal_total_s, recovery_total_s}` (when `outcome == "selected"`)
- `selected_candidate`, `checked_phases`, `margins`, `evidence`, `rejections`, `rejection_codes`, `alternatives`, `search`, `legacy: {reachable, code, message}`

### 1.4 Live Refusal & Terminal Semantics

- **Refusal before `BeginMacro`** (WS `command_reject` with `status: 422`): `code`, `message`, `failing_phase`, `outcome`, and optional `hint` (only when a checked alternative exists).
- **Terminal codes during execution** (lossless WS observation `action` terminal): `TARGET_LOST`, `TARGET_DISPLACED`, `DETECTION_UNAVAILABLE`, `PLAN_STALE`, `CALIBRATION_CHANGED`, `COLLISION`, `TRACKING_STALL`, `NOT_HOLDING`, `DROPPED`, `GRASP_TIMEOUT`, `LIFT_TORQUE`, `INSUFFICIENT_EVIDENCE`, `BUDGET_EXHAUSTED`, `IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION`.
- **No automatic retries**: on `NOT_HOLDING`, `armd` withdraws with jaws closed along `-insertion_dir` (never opening jaws at contact) and terminates failed. On `DROPPED`, `armd` terminates failed without regripping. On success, `action.holding == true` and `HeldObject` is committed for a subsequent `place(...)`.

---

## 2. Patch for `rebot/client.py`

### Summary of Changes
1. Add `ObservationSchema.require_grasp_select()` and `ObservationSchema.require_grasp_select_exec()` next to `require_grasp_tasks()`.
2. Add `_validate_grasp_strategy(strategy)` and `_build_grasp_body(...)` shared across `ArmdClient.grasp_dry_run*`, `ArmSession.grasp`, and `ArmSession.grasp_dry_run*`.
3. Extend `ArmdClient.grasp_dry_run(...)` (preserving its 3-tuple return type) and add `ArmdClient.grasp_dry_run_detail(...) -> dict` over lease-free REST (`POST /api/v1/action/grasp`, `dry_run: true`), gated on `grasp_select` when `strategy == "adaptive"`.
4. Extend `ArmSession.grasp(...)` (WS under control lease, gated on `grasp_select_exec` when `strategy == "adaptive"`), `ArmSession.grasp_dry_run(...)`, and add `ArmSession.grasp_dry_run_detail(...)` (WS dry-run, gated on `grasp_select` when `strategy == "adaptive"`).

### Unified Diff (`rebot/client.py`)

```diff
diff --git a/rebot/client.py b/rebot/client.py
index d15368a..f1c1428 100644
--- a/rebot/client.py
+++ b/rebot/client.py
@@ -954,6 +954,20 @@ class ObservationSchema:
                 "Grasp tasks need an armd that reports capability 'grasp_tasks'. Update rebot-armd."
             )

+    def require_grasp_select(self) -> None:
+        """Raise :class:`UnsupportedError` unless armd reports capability ``grasp_select: true``."""
+        if self.capabilities.get("grasp_select") is not True:
+            raise UnsupportedError(
+                "Adaptive grasp selection needs a 6-DOF armd that reports capability 'grasp_select'."
+            )
+
+    def require_grasp_select_exec(self) -> None:
+        """Raise :class:`UnsupportedError` unless armd reports capability ``grasp_select_exec: true``."""
+        if self.capabilities.get("grasp_select_exec") is not True:
+            raise UnsupportedError(
+                "Live adaptive grasp execution needs a 6-DOF armd that reports capability 'grasp_select_exec'."
+            )
+
     def has(self, capability: str, min_version: int = CAPABILITIES_DEVPLAT) -> bool:
         """``capabilities[capability]`` is true and ``capabilities.version >= min_version``."""
         return self.capabilities_version >= min_version and self.capabilities.get(capability) is True
@@ -1208,7 +1222,7 @@ class ArmdClient:
         """``GET /api/v1/gripper/calibration`` (no lease required)."""
         return dict(self._get("/api/v1/gripper/calibration") or {})

-    def grasp_dry_run(
+    def grasp_dry_run_detail(
         self,
         label: str,
         box_norm: Sequence[float],
@@ -1218,35 +1232,83 @@ class ArmdClient:
         tag: str | None = None,
         timeout: float | None = None,
         task: str | None = None,
-    ) -> tuple[bool, str | None, str | None]:
-        """Check grasp reachability over REST (no lease required).
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
+    ) -> dict:
+        """Check grasp reachability over REST (no lease required) and return the full reply dict.

-        With ``task`` (needs capability ``grasp_tasks``, else :class:`UnsupportedError`) an unsupported task
-        returns ``(False, "TASK_UNSUPPORTED", reason)``.
+        With ``strategy="adaptive"``, requires capability ``grasp_select`` and returns the full
+        adaptive selection/preflight payload (``outcome``, ``failing_phase``, optional checked
+        ``hint``, ``pregrasp_mode``, ``expected_duration_s``, ``selected_candidate``, ``budget``,
+        ``legacy``). Legacy remains the default when ``strategy`` is omitted or ``"legacy"``.
         """
-        if task is not None:
-            self.get_schema().require_grasp_tasks()
-        body: dict = {
-            "label": str(label),
-            "box_norm": [float(v) for v in box_norm],
-            "dry_run": True,
-        }
-        if frame_id is not None:
-            body["frame_id"] = int(frame_id)
-        if ts_ms is not None:
-            body["ts_ms"] = float(ts_ms)
-        if overrides is not None:
-            body["overrides"] = dict(overrides)
-        if tag is not None:
-            body["tag"] = str(tag)
-        if task is not None:
-            body["task"] = str(task)
+        strat = _validate_grasp_strategy(strategy)
+        if task is not None or strat == "adaptive":
+            schema = self.get_schema()
+            if task is not None:
+                schema.require_grasp_tasks()
+            if strat == "adaptive":
+                schema.require_grasp_select()
+        body = _build_grasp_body(
+            label=label,
+            box_norm=[float(v) for v in box_norm],
+            dry_run=True,
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            overrides=overrides,
+            tag=tag,
+            task=task,
+            strategy=strat,
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+            keep_empty_overrides=True,
+        )
         try:
             data = self._post("/api/v1/action/grasp", json=body, timeout=timeout)
         except _http.HttpStatusError as exc:
             data = _json_dict(exc.response)
             if data.get("code") != TASK_UNSUPPORTED:
                 raise
+        return dict(data)
+
+    def grasp_dry_run(
+        self,
+        label: str,
+        box_norm: Sequence[float],
+        frame_id: int | None = None,
+        ts_ms: float | None = None,
+        overrides: dict | None = None,
+        tag: str | None = None,
+        timeout: float | None = None,
+        task: str | None = None,
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
+    ) -> tuple[bool, str | None, str | None]:
+        """Check grasp reachability over REST (no lease required).
+
+        With ``task`` (needs capability ``grasp_tasks``, else :class:`UnsupportedError`) an unsupported task
+        returns ``(False, "TASK_UNSUPPORTED", reason)``. With ``strategy="adaptive"`` (needs capability
+        ``grasp_select``) runs adaptive selection/preflight without moving the arm.
+        """
+        data = self.grasp_dry_run_detail(
+            label=label,
+            box_norm=box_norm,
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            overrides=overrides,
+            tag=tag,
+            timeout=timeout,
+            task=task,
+            strategy=strategy,
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+        )
         return _dry_run_tuple(data)

     def measure(
@@ -1698,6 +1760,59 @@ class ArmdClient:
         return ArmSession(self, client_id=client_id, **kwargs)


+_GRASP_STRATEGIES = ("legacy", "adaptive")
+
+
+def _validate_grasp_strategy(strategy: str | None) -> str | None:
+    if strategy is None:
+        return None
+    if not isinstance(strategy, str) or strategy not in _GRASP_STRATEGIES:
+        raise ValueError(f"strategy must be one of {_GRASP_STRATEGIES}, got {strategy!r}")
+    return strategy
+
+
+def _build_grasp_body(
+    *,
+    label: str,
+    box_norm: list[float],
+    dry_run: bool,
+    frame_id: int | None = None,
+    ts_ms: float | None = None,
+    overrides: dict | None = None,
+    tag: str | None = None,
+    task: str | None = None,
+    strategy: str | None = None,
+    evidence: dict | None = None,
+    pose_preferences: dict | None = None,
+    budget_ms: int | None = None,
+    keep_empty_overrides: bool = False,
+) -> dict:
+    body: dict = {"label": str(label), "box_norm": box_norm}
+    if dry_run:
+        body["dry_run"] = True
+    if frame_id is not None:
+        body["frame_id"] = int(frame_id)
+    if ts_ms is not None:
+        body["ts_ms"] = float(ts_ms)
+    if overrides is not None and (keep_empty_overrides or overrides):
+        body["overrides"] = dict(overrides)
+    if tag is not None:
+        body["tag"] = str(tag)
+    if task is not None:
+        body["task"] = str(task)
+    if strategy is not None:
+        body["strategy"] = strategy
+    if evidence is not None:
+        body["evidence"] = dict(evidence)
+    if pose_preferences is not None:
+        body["pose_preferences"] = dict(pose_preferences)
+    if budget_ms is not None:
+        if isinstance(budget_ms, bool) or not isinstance(budget_ms, int):
+            raise ValueError(f"budget_ms must be an int, got {type(budget_ms).__name__}")
+        body["budget_ms"] = budget_ms
+    return body
+
+
 def _dry_run_tuple(data: dict) -> tuple[bool, str | None, str | None]:
     """``(reachable, code, message)`` from a dry-run reply; ``task_supported: false`` -> TASK_UNSUPPORTED."""
     if data.get("task_supported") is False:
@@ -2901,30 +3016,42 @@ class ArmSession:
         tag: str | None = None,
         timeout: float | None = None,
         task: str | None = None,
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
     ) -> ActionResult:
-        """``/api/v1/action/grasp``: armd centres on the box, descends, closes and lifts.
+        """``/api/v1/action/grasp`` over WebSocket under the active control lease.

         ``box_norm`` is ``[x0, y0, x1, y1]`` normalised to the detector image. The result's
         ``action.holding`` tells whether an object is in the gripper. Default timeout 120 s.
         ``task`` names a stored grasp task (capability ``grasp_tasks``, else :class:`UnsupportedError`);
         armd rejects it with ``TASK_UNSUPPORTED`` when the loaded vision model cannot serve it.
+        ``strategy="adaptive"`` selects and executes an adaptive 6-DOF grasp (requires capability
+        ``grasp_select_exec``); legacy remains the default when omitted.
         """
+        strat = _validate_grasp_strategy(strategy)
         if task is not None:
             self.schema().require_grasp_tasks()
-        body: dict = {"label": str(label), "box_norm": _vec(box_norm, 4, "box_norm")}
-        if frame_id is not None:
-            body["frame_id"] = int(frame_id)
-        if ts_ms is not None:
-            body["ts_ms"] = ts_ms
-        if overrides:
-            body["overrides"] = dict(overrides)
-        if tag is not None:
-            body["tag"] = str(tag)
-        if task is not None:
-            body["task"] = str(task)
+        if strat == "adaptive":
+            self.schema().require_grasp_select_exec()
+        body = _build_grasp_body(
+            label=label,
+            box_norm=_vec(box_norm, 4, "box_norm"),
+            dry_run=False,
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            overrides=overrides,
+            tag=tag,
+            task=task,
+            strategy=strat,
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+        )
         return self._action("/api/v1/action/grasp", body, 120.0 if timeout is None else timeout, legacy_wait=None)

-    def grasp_dry_run(
+    def grasp_dry_run_detail(
         self,
         label: str,
         box_norm: Sequence[float],
@@ -2934,35 +3061,79 @@ class ArmSession:
         tag: str | None = None,
         timeout: float | None = None,
         task: str | None = None,
-    ) -> tuple[bool, str | None, str | None]:
-        """Check grasp reachability over WebSocket (returns (reachable, code, message)).
-
-        With ``task`` (capability ``grasp_tasks``, else :class:`UnsupportedError`) an unsupported task
-        returns ``(False, "TASK_UNSUPPORTED", reason)``.
-        """
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
+    ) -> dict:
+        """Check grasp reachability over WebSocket and return the full ack dict."""
+        strat = _validate_grasp_strategy(strategy)
         if task is not None:
             self.schema().require_grasp_tasks()
-        body: dict = {
-            "label": str(label),
-            "box_norm": _vec(box_norm, 4, "box_norm"),
-            "dry_run": True,
-        }
-        if frame_id is not None:
-            body["frame_id"] = int(frame_id)
-        if ts_ms is not None:
-            body["ts_ms"] = float(ts_ms)
-        if overrides:
-            body["overrides"] = dict(overrides)
-        if tag is not None:
-            body["tag"] = str(tag)
-        if task is not None:
-            body["task"] = str(task)
+        if strat == "adaptive":
+            self.schema().require_grasp_select()
+        body = _build_grasp_body(
+            label=label,
+            box_norm=_vec(box_norm, 4, "box_norm"),
+            dry_run=True,
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            overrides=overrides,
+            tag=tag,
+            task=task,
+            strategy=strat,
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+        )
         try:
             ack = self.command("/api/v1/action/grasp", body, timeout=timeout)
         except ArmCommandRejected as exc:
             if exc.code != TASK_UNSUPPORTED:
                 raise
-            return False, TASK_UNSUPPORTED, str(exc.raw.get("message") or "grasp task not supported")
+            return {
+                "reachable": False,
+                "code": TASK_UNSUPPORTED,
+                "message": str(exc.raw.get("message") or "grasp task not supported"),
+                "task_supported": False,
+                "task_reason": exc.raw.get("message"),
+            }
+        return dict(ack)
+
+    def grasp_dry_run(
+        self,
+        label: str,
+        box_norm: Sequence[float],
+        frame_id: int | None = None,
+        ts_ms: float | None = None,
+        overrides: dict | None = None,
+        tag: str | None = None,
+        timeout: float | None = None,
+        task: str | None = None,
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
+    ) -> tuple[bool, str | None, str | None]:
+        """Check grasp reachability over WebSocket (returns (reachable, code, message)).
+
+        With ``task`` (capability ``grasp_tasks``, else :class:`UnsupportedError`) an unsupported task
+        returns ``(False, "TASK_UNSUPPORTED", reason)``.
+        """
+        ack = self.grasp_dry_run_detail(
+            label=label,
+            box_norm=box_norm,
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            overrides=overrides,
+            tag=tag,
+            timeout=timeout,
+            task=task,
+            strategy=strategy,
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+        )
         return _dry_run_tuple(ack)

     def measure(
```

---

## 3. Patch for `rebot/arm.py`

### Summary of Changes
1. Add `_HINTS` entries for Phase B refusal/terminal codes (`LIFT_TORQUE`, `INSUFFICIENT_EVIDENCE`, `BUDGET_EXHAUSTED`, `TARGET_DISPLACED`, `NOT_HOLDING`, `DROPPED`, `CALIBRATION_CHANGED`, `PLAN_STALE`), and surface server-checked `exc.raw["hint"]` in `_call_with_hints` when present.
2. Add `_merge_adaptive_evidence(target, evidence, strategy)` to forward oriented/metric `Detection` fields (`obb_corners_norm`, `angle_deg`, `angle_conf`, `height_m`) merged under caller-supplied `evidence`, and `_adaptive_kw(...)` so legacy callers pass unchanged kwargs to `session.grasp*`.
3. Extend `Arm.grasp`, `Arm.grasp_dry_run`, add `Arm.grasp_dry_run_detail`, and extend `Arm.grasp_any` with `strategy`, `evidence`, `pose_preferences`, and `budget_ms`.

### Unified Diff (`rebot/arm.py`)

```diff
diff --git a/rebot/arm.py b/rebot/arm.py
index fe8e6ad..e5db02d 100644
--- a/rebot/arm.py
+++ b/rebot/arm.py
@@ -109,6 +109,14 @@ _HINTS = {
         "use a smaller model or a lower fps (the message gives the highest fps the model holds)"
     ),
     "OBSERVATION_UNAVAILABLE": "observation not ready yet; wait and retry once",
+    "LIFT_TORQUE": "exceeds derated joint holding torque along separation or lift",
+    "INSUFFICIENT_EVIDENCE": "lacks enough geometry or payload evidence to clear adaptive safety checks",
+    "BUDGET_EXHAUSTED": "exhausted the adaptive search wall-time budget before finishing checks",
+    "TARGET_DISPLACED": "stopped because the target moved, rotated, or became ambiguous before insertion",
+    "NOT_HOLDING": "closed without holding the object and withdrew with jaws closed (no automatic retry)",
+    "DROPPED": "lost grip after lifting the object (no automatic retry)",
+    "CALIBRATION_CHANGED": "aborted because camera or arm calibration changed mid-grasp",
+    "PLAN_STALE": "aborted because arm state or workspace/grasp configuration changed after planning",
 }

 SENSITIVITIES = ("low", "medium", "high")
@@ -168,7 +176,12 @@ def _call_with_hints(fn: Callable[[], Any], what: str, hints: Optional[dict] = N
     try:
         return fn()
     except ArmCommandRejected as exc:
-        hint = _hint(exc.code, exc.message, hints)
+        server_hint = exc.raw.get("hint") if isinstance(exc.raw, dict) else None
+        hint = (
+            server_hint
+            if isinstance(server_hint, str) and server_hint
+            else _hint(exc.code, exc.message, hints)
+        )
         if hint:
             msg = f"{what} {hint} ({exc.message})" if exc.message else f"{what} {hint}"
             raw = dict(exc.raw, message=msg)
@@ -188,6 +201,46 @@ def _tag_kw(tag: Optional[str]) -> dict:
     """Forward ``tag`` only when provided so mocks asserting exact kwargs stay compatible."""
     return {"tag": tag} if tag is not None else {}

+
+def _merge_adaptive_evidence(
+    target: Union[Detection, str],
+    evidence: Optional[dict],
+    strategy: Optional[str],
+) -> Optional[dict]:
+    if strategy != "adaptive" and evidence is None:
+        return None
+    merged: dict[str, Any] = {}
+    if isinstance(target, Detection) and strategy == "adaptive":
+        if target.obb_corners_norm is not None:
+            merged["obb_corners_norm"] = [[float(u), float(v)] for u, v in target.obb_corners_norm]
+        if target.angle_deg is not None and target.angle_conf is not None:
+            merged["angle_deg"] = float(target.angle_deg)
+            merged["angle_conf"] = float(target.angle_conf)
+        if target.height_m is not None:
+            merged["height_m"] = float(target.height_m)
+    if evidence is not None:
+        merged.update(evidence)
+    return merged or None
+
+
+def _adaptive_kw(
+    strategy: Optional[str],
+    evidence: Optional[dict],
+    pose_preferences: Optional[dict],
+    budget_ms: Optional[int],
+) -> dict:
+    """Forward Phase B adaptive fields only when set so legacy mock tests stay compatible."""
+    kw: dict[str, Any] = {}
+    if strategy is not None:
+        kw["strategy"] = strategy
+    if evidence is not None:
+        kw["evidence"] = evidence
+    if pose_preferences is not None:
+        kw["pose_preferences"] = pose_preferences
+    if budget_ms is not None:
+        kw["budget_ms"] = budget_ms
+    return kw
+
 # Loop guard: time inside an Arm method counts as fed only up to the call's own budget plus this
 # margin (enable, command round trips). A call stuck past that trips the guard like a hung loop.
 GUARD_BUSY_MARGIN_S = 15.0
@@ -706,6 +759,10 @@ class Arm:
         timeout: Optional[float] = None,
         task: Optional[str] = None,
         tag: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
     ) -> ActionResult:
         """Pick up a detected object: ``arm.grasp(detection)`` or ``arm.grasp("cup", box=(x0, y0, x1, y1))``.

@@ -716,7 +773,12 @@ class Arm:
         ``task`` names a grasp task stored in armd. Without the ``grasp_tasks`` capability this raises
         :class:`UnsupportedError`; when the loaded vision model/depth cannot serve the task armd rejects the
         command with ``TASK_UNSUPPORTED`` (:class:`ArmCommandRejected`) before the arm moves.
+
+        ``strategy="adaptive"`` selects and executes an adaptive 6-DOF grasp (requires capability
+        ``grasp_select_exec``). Legacy remains the default when ``strategy`` is omitted.
         """
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ValueError(f"strategy must be 'legacy' or 'adaptive', got {strategy!r}")
         if isinstance(target, Detection):
             label, box_norm, frame_id, ts_ms = target.label, target.box_norm, target.frame_id, target.ts_ms
         else:
@@ -730,20 +792,62 @@ class Arm:
             )
         if task is not None:
             self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select_exec()
+        eff_evidence = _merge_adaptive_evidence(target, evidence, strategy)
+        akw = _adaptive_kw(strategy, eff_evidence, pose_preferences, budget_ms)
         return self._motion(
             lambda: self.session.grasp(
-                label, box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task, **_tag_kw(tag)
+                label, box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task, **_tag_kw(tag), **akw
             ),
             f"grasp of '{label}'",
             timeout,
         )

+    def grasp_dry_run_detail(
+        self,
+        target: Union[Detection, str],
+        box: Optional[Sequence[float]] = None,
+        timeout: Optional[float] = None,
+        task: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
+    ) -> dict:
+        """Check reachability without moving and return the full server reply dict."""
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ValueError(f"strategy must be 'legacy' or 'adaptive', got {strategy!r}")
+        if isinstance(target, Detection):
+            label, box_norm, frame_id, ts_ms = target.label, target.box_norm, target.frame_id, target.ts_ms
+        else:
+            if box is None:
+                raise ValueError(
+                    "grasp_dry_run_detail(label) needs box=(x0, y0, x1, y1); or pass a detection from arm.detect()"
+                )
+            label, box_norm, frame_id, ts_ms = str(target), tuple(box), None, None
+        if task is not None:
+            self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select()
+        eff_evidence = _merge_adaptive_evidence(target, evidence, strategy)
+        akw = _adaptive_kw(strategy, eff_evidence, pose_preferences, budget_ms)
+        return self._run(
+            lambda: self.session.grasp_dry_run_detail(
+                label=label, box_norm=box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task, **akw
+            )
+        )
+
     def grasp_dry_run(
         self,
         target: Union[Detection, str],
         box: Optional[Sequence[float]] = None,
         timeout: Optional[float] = None,
         task: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
     ) -> Tuple[bool, Optional[str], Optional[str]]:
         """Check reachability of a detection without moving the arm.

@@ -751,6 +855,8 @@ class Arm:
         :class:`UnsupportedError`) a task the loaded vision model/depth cannot serve returns
         ``(False, "TASK_UNSUPPORTED", reason)``.
         """
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ValueError(f"strategy must be 'legacy' or 'adaptive', got {strategy!r}")
         if isinstance(target, Detection):
             label, box_norm, frame_id, ts_ms = target.label, target.box_norm, target.frame_id, target.ts_ms
         else:
@@ -761,9 +867,13 @@ class Arm:
             label, box_norm, frame_id, ts_ms = str(target), tuple(box), None, None
         if task is not None:
             self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select()
+        eff_evidence = _merge_adaptive_evidence(target, evidence, strategy)
+        akw = _adaptive_kw(strategy, eff_evidence, pose_preferences, budget_ms)
         return self._run(
             lambda: self.session.grasp_dry_run(
-                label=label, box_norm=box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task
+                label=label, box_norm=box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task, **akw
             )
         )

@@ -774,6 +884,10 @@ class Arm:
         max_candidates: int = 5,
         timeout: Optional[float] = None,
         task: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
     ) -> ActionResult:
         """Detect objects, dry-run in score order, and grasp the first reachable one.

@@ -785,19 +899,25 @@ class Arm:
         ``TASK_UNSUPPORTED``, :class:`ActionFailed` with that code is raised at once. Without the
         ``grasp_tasks`` capability :class:`UnsupportedError` is raised before anything is sent.
         """
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ValueError(f"strategy must be 'legacy' or 'adaptive', got {strategy!r}")
         if task is not None:
             self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select()
+            self.schema.require_grasp_select_exec()
+        akw = _adaptive_kw(strategy, evidence, pose_preferences, budget_ms)
         candidates = self.detect(label=label, min_score=min_score)
         bounded_candidates = candidates[:max(0, int(max_candidates))]
         last_code: Optional[str] = None
         last_message: Optional[str] = None
         for d in bounded_candidates:
             self.feed()
-            reachable, code, message = self.grasp_dry_run(d, timeout=timeout, task=task)
+            reachable, code, message = self.grasp_dry_run(d, timeout=timeout, task=task, **akw)
             if code == "TASK_UNSUPPORTED" and not reachable:
                 raise ActionFailed(code, message or "grasp task not supported by the loaded vision model")
             if reachable:
-                return self.grasp(d, timeout=timeout, task=task)
+                return self.grasp(d, timeout=timeout, task=task, **akw)
             last_code = code
             last_message = message
         raise ActionFailed(
```

---

## 4. Patch for `rebot_companion/mcp_server.py`

### Summary of Changes
1. Add Phase B `HINTS` entries and preserve server-checked `hint`, `failing_phase`, and `outcome` from `ArmCommandRejected.raw` in `map_exception`.
2. Expose `"holding": a.holding if a else None` in `Controller._action_dict` so MCP callers observe `holding == true` after `grasp` before calling `place`.
3. Extend `Controller.grasp_dry_run` and `@tool grasp_dry_run` (lease-free REST) with optional `strategy`, `evidence`, `pose_preferences`, and `budget_ms`; preserve the exact legacy `{reachable, code, message}` payload when `strategy != "adaptive"`.
4. Add `Controller.grasp` and actuator `@tool grasp` (inside `if actuate:` alongside `place`), using `self.actuator()` over the WebSocket control lease, gating on `grasp` / `grasp_tasks` / `grasp_select_exec`, performing zero automatic retries, and returning `self._action_dict(res)` (with `holding`).

### Unified Diff (`rebot_companion/mcp_server.py`)

```diff
diff --git a/rebot_companion/mcp_server.py b/rebot_companion/mcp_server.py
index d7c7390..b75bbe9 100644
--- a/rebot_companion/mcp_server.py
+++ b/rebot_companion/mcp_server.py
@@ -191,6 +191,13 @@ HINTS = {
     "TOO_FEW_POINTS": "too few points on the object to triangulate; light it evenly, use more views or a larger "
     "baseline_m",
     "TARGET_LOST": "the object left the view; start where the whole object is in view or use a smaller baseline_m",
+    "LIFT_TORQUE": "exceeds derated joint holding torque along separation or lift",
+    "INSUFFICIENT_EVIDENCE": "lacks enough geometry or payload evidence to clear adaptive safety checks",
+    "BUDGET_EXHAUSTED": "exhausted the adaptive search wall-time budget before finishing checks",
+    "TARGET_DISPLACED": "target moved, rotated, or became ambiguous before insertion",
+    "NOT_HOLDING": "gripper closed without holding the object and withdrew with jaws closed (no automatic retry)",
+    "DROPPED": "object dropped after lift (no automatic retry)",
+    "GRASP_TIMEOUT": "grasp exceeded its time budget; stop was sent; check the scene with snapshot",
 }

 SINGLE_VIEW_NOTE = (
@@ -338,12 +345,18 @@ def map_exception(exc: BaseException) -> ToolFail | None:
         )
     if isinstance(exc, ArmCommandRejected):
         message = str(exc.raw.get("message") or exc.code)
-        hint = None
+        server_hint = exc.raw.get("hint") if isinstance(exc.raw, dict) else None
+        hint = server_hint if isinstance(server_hint, str) and server_hint else None
         if exc.code == "FAULT_ACTIVE":
             hint = "the arm is faulted; check get_state.fault_reason, then clear_fault and enable"
         elif exc.code in ("INVALID_PIN", "RATE_LIMITED"):
             hint = f"the operator PIN configured for rebot-mcp was refused; ask the user to check ${PIN_ENV}"
-        return ToolFail(exc.code, message, hint, **exc.violation_details())
+        extra = dict(exc.violation_details())
+        if isinstance(exc.raw, dict):
+            for k in ("failing_phase", "outcome"):
+                if exc.raw.get(k) is not None:
+                    extra[k] = exc.raw[k]
+        return ToolFail(exc.code, message, hint, **extra)
     if isinstance(exc, MotionTimeout):
         return ToolFail("MOTION_TIMEOUT", str(exc))
     if isinstance(exc, ArmFaultError):
@@ -1092,6 +1105,7 @@ class Controller:
             "action_id": a.id if a else None,
             "kind": a.kind if a else None,
             "state": a.state if a else "done",
+            "holding": a.holding if a else None,
             "settled": a.settled if a else None,
             "tcp_after": self.format_tcp(res.status.tcp, res.status.positions),
             "duration_s": _r(res.duration_s, 3),
@@ -1687,6 +1701,10 @@ class Controller:
         ts_ms: float | None = None,
         task: str | None = None,
         timeout_s: float | None = None,
+        strategy: str | None = None,
+        evidence: dict[str, Any] | None = None,
+        pose_preferences: dict[str, Any] | None = None,
+        budget_ms: int | None = None,
     ) -> dict[str, Any]:
         """Check grasp reachability over lease-free REST (POST /api/v1/action/grasp dry_run=true)."""
         if not isinstance(label, str) or not label.strip():
@@ -1700,15 +1718,108 @@ class Controller:
             )
         ):
             raise ToolFail("INVALID_ARGUMENT", "box_norm must be 4 finite numbers [x0, y0, x1, y1]")
-        reachable, code, message = self.client.grasp_dry_run(
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ToolFail("INVALID_ARGUMENT", "strategy must be 'legacy' or 'adaptive'")
+        if strategy != "adaptive":
+            kw: dict[str, Any] = {}
+            if strategy is not None:
+                kw["strategy"] = strategy
+            reachable, code, message = self.client.grasp_dry_run(
+                label,
+                [float(v) for v in box_norm],
+                frame_id=frame_id,
+                ts_ms=ts_ms,
+                timeout=timeout_s,
+                task=task,
+                **kw,
+            )
+            return {"reachable": bool(reachable), "code": code, "message": message}
+        data = self.client.grasp_dry_run_detail(
             label,
             [float(v) for v in box_norm],
             frame_id=frame_id,
             ts_ms=ts_ms,
             timeout=timeout_s,
             task=task,
+            strategy="adaptive",
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
         )
-        return {"reachable": bool(reachable), "code": code, "message": message}
+        out: dict[str, Any] = {
+            "reachable": bool(data.get("reachable", False)),
+            "code": data.get("code"),
+            "message": data.get("message"),
+            "strategy": "adaptive",
+            "outcome": data.get("outcome"),
+            "failing_phase": data.get("failing_phase"),
+            "legacy": data.get("legacy"),
+        }
+        if isinstance(data.get("hint"), str) and data["hint"]:
+            out["hint"] = data["hint"]
+        if data.get("selected_candidate") is not None:
+            out["selected_candidate"] = data["selected_candidate"]
+            out["pregrasp_mode"] = data.get("pregrasp_mode")
+            out["expected_duration_s"] = data.get("expected_duration_s")
+            out["margins"] = data.get("margins")
+            out["budget"] = data.get("budget")
+        return out
+
+    def grasp(
+        self,
+        label: str,
+        box_norm: list[float],
+        frame_id: int | None = None,
+        ts_ms: float | None = None,
+        task: str | None = None,
+        strategy: str | None = None,
+        evidence: dict[str, Any] | None = None,
+        pose_preferences: dict[str, Any] | None = None,
+        budget_ms: int | None = None,
+        timeout_s: float | None = None,
+    ) -> dict[str, Any]:
+        """Execute grasp over WebSocket under the active control lease (/api/v1/action/grasp). No automatic retries."""
+        self._require_control_allowed("grasp")
+        if not isinstance(label, str) or not label.strip():
+            raise ToolFail("INVALID_ARGUMENT", "label must be a non-empty string")
+        if (
+            not isinstance(box_norm, list)
+            or len(box_norm) != 4
+            or not all(
+                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
+                for v in box_norm
+            )
+        ):
+            raise ToolFail("INVALID_ARGUMENT", "box_norm must be 4 finite numbers [x0, y0, x1, y1]")
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ToolFail("INVALID_ARGUMENT", "strategy must be 'legacy' or 'adaptive'")
+        caps = self.schema().capabilities or {}
+        if caps and caps.get("grasp") is False:
+            raise ToolFail("UNSUPPORTED", "this armd profile does not support grasp")
+        if task is not None and caps.get("grasp_tasks") is not True:
+            raise ToolFail("UNSUPPORTED", "armd does not advertise capability 'grasp_tasks'")
+        if strategy == "adaptive" and caps.get("grasp_select_exec") is not True:
+            raise ToolFail("UNSUPPORTED", "armd does not advertise capability 'grasp_select_exec'")
+        s = self.actuator()
+        kw: dict[str, Any] = {}
+        if strategy is not None:
+            kw["strategy"] = strategy
+        if evidence is not None:
+            kw["evidence"] = evidence
+        if pose_preferences is not None:
+            kw["pose_preferences"] = pose_preferences
+        if budget_ms is not None:
+            kw["budget_ms"] = budget_ms
+        res = s.grasp(
+            label,
+            [float(v) for v in box_norm],
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            timeout=timeout_s,
+            task=task,
+            **kw,
+        )
+        return self._action_dict(res)

     # ---- sandbox tools (lease-free) ----------------------------------------

@@ -3051,7 +3162,12 @@ def build_server(
     @tool(
         "Check grasp reachability for a detected box [x0, y0, x1, y1] (normalised 0..1) without moving "
         "or acquiring a control lease (REST POST /api/v1/action/grasp with dry_run=true). "
-        "Returns {reachable, code, message}. Optional task needs armd capability 'grasp_tasks'."
+        "Default strategy='legacy' returns {reachable, code, message}. strategy='adaptive' (needs capability "
+        "'grasp_select') accepts optional evidence ({contour, obb_corners_norm, angle_deg, angle_conf, "
+        "height_m, payload_kg}), pose_preferences ({vertical_weight, approach_side, preferred_max_tilt_deg, "
+        "hard: {max_tilt_deg}}), and budget_ms, returning outcome, failing_phase, pregrasp_mode, "
+        "expected_duration_s, selected_candidate, legacy, and optional checked hint. Optional task needs "
+        "armd capability 'grasp_tasks'."
     )
     async def grasp_dry_run(
         label: str,
@@ -3060,8 +3176,24 @@ def build_server(
         ts_ms: float | None = None,
         task: str | None = None,
         timeout_s: Annotated[float | None, Field(gt=0, le=60)] = None,
+        strategy: Literal["legacy", "adaptive"] | None = None,
+        evidence: dict[str, Any] | None = None,
+        pose_preferences: dict[str, Any] | None = None,
+        budget_ms: Annotated[int | None, Field(ge=1, le=10000)] = None,
     ) -> dict:
-        return await run(ctrl.grasp_dry_run, label, box_norm, frame_id, ts_ms, task, timeout_s)
+        return await run(
+            ctrl.grasp_dry_run,
+            label,
+            box_norm,
+            frame_id,
+            ts_ms,
+            task,
+            timeout_s,
+            strategy,
+            evidence,
+            pose_preferences,
+            budget_ms,
+        )

     @tool(
         "Estimate object sizes from ONE camera view with the depth model (no motion, no lease; operator "
@@ -3383,6 +3515,40 @@ def build_server(
                 axes,
             )

+        @tool(
+            "Pick up a detected object [x0, y0, x1, y1] (normalised 0..1) over the WebSocket control lease "
+            "(/api/v1/action/grasp). Default strategy='legacy' preserves visual-servo grasp; "
+            "strategy='adaptive' (needs capability 'grasp_select_exec') plans and executes an adaptive 6-DOF "
+            "grasp with optional evidence, pose_preferences ({vertical_weight, approach_side, "
+            "preferred_max_tilt_deg, hard: {max_tilt_deg}}), and budget_ms. No automatic retries; on success "
+            "returns holding=true so a follow-up place call can put the object down. Blocks until done."
+        )
+        async def grasp(
+            label: str,
+            box_norm: Annotated[list[float], Field(min_length=4, max_length=4)],
+            frame_id: int | None = None,
+            ts_ms: float | None = None,
+            task: str | None = None,
+            strategy: Literal["legacy", "adaptive"] | None = None,
+            evidence: dict[str, Any] | None = None,
+            pose_preferences: dict[str, Any] | None = None,
+            budget_ms: Annotated[int | None, Field(ge=1, le=10000)] = None,
+            timeout_s: Annotated[float | None, Field(gt=0, le=120)] = None,
+        ) -> dict:
+            return await motion(
+                ctrl.grasp,
+                label,
+                box_norm,
+                frame_id,
+                ts_ms,
+                task,
+                strategy,
+                evidence,
+                pose_preferences,
+                budget_ms,
+                timeout_s,
+            )
+
         @tool(
             "Put the held object down: lift, move above the target, descend, open the gripper, retreat. Target: "
             "a saved pose name (its gripper position is the place point) or x, y, z in m (base frame). "
```

---

## 5. Verification Plan in `rebot-companion`

After applying the three patches on commit `a863f9e`:
1. `pytest tests/test_client.py tests/test_arm.py tests/test_mcp_server.py` — verify legacy `grasp` / `grasp_dry_run` mocks and payloads remain byte-for-byte compatible when `strategy` is omitted.
2. Add capability-gating tests:
   - `strategy="adaptive"` dry-run raises `UnsupportedError` when `capabilities.grasp_select` is `false` or absent.
   - `strategy="adaptive"` live `grasp` raises `UnsupportedError` (SDK) / `ToolFail("UNSUPPORTED")` (MCP) when `capabilities.grasp_select_exec` is `false` or absent (5-DOF or `wasm32`).
3. Verify `Arm.grasp(detection, strategy="adaptive")` merges `obb_corners_norm`/`angle_deg`/`angle_conf`/`height_m` under explicit `evidence`, MCP `grasp` returns `holding: true`, and `ToolFail` preserves `failing_phase` / `outcome` and only forwards server-checked `hint`.
