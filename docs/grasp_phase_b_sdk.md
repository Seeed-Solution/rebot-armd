# Adaptive Grasp Phase B — Companion SDK & MCP Patch Specification

This document specifies the exact, patch-ready changes for `/home/harvest/project/rebot-companion` (`rebot/client.py`, `rebot/arm.py`, `rebot_companion/mcp_server.py`) to expose `rebot-armd` Phase B adaptive grasp (`strategy="adaptive"`) while keeping legacy grasp as the default.

> Scope note: `rebot-companion` lives in a separate repository. Apply the patches below directly in `/home/harvest/project/rebot-companion` once `rebot-armd` Phase B is deployed.

---

## 1. Wire Contract (`rebot-armd` Phase B)

### 1.1 Capabilities (`/api/v1/observation/schema`)

| Capability | Meaning | Profiles |
|---|---|---|
| `capabilities.grasp` | Legacy visual-servo grasp available | 5-DOF and 6-DOF profiles with gripper + grasp setup |
| `capabilities.grasp_select` | Adaptive candidate selection (`dry_run: true`, `strategy: "adaptive"`) | 6-DOF profiles with gripper + grasp setup |
| `capabilities.grasp_select_exec` | Live adaptive grasp execution (`strategy: "adaptive"`) | 6-DOF profiles with gripper + grasp setup (`false` on 5-DOF and `wasm32`) |

### 1.2 Request Body (`/api/v1/action/grasp`, REST dry-run & WS live/dry-run)

Existing fields remain unchanged; `"legacy"` remains the default when `strategy` is omitted:

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
    "approach": "any",
    "max_tilt_deg": 70.0
  },
  "budget_ms": 500
}
```

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

- **Refusal before `BeginMacro`** (HTTP 422 / WS `command_reject`): `code`, `message`, `failing_phase`, `outcome`, and optional `hint` (only when a checked alternative exists).
- **Terminal codes during execution**: `TARGET_LOST`, `TARGET_DISPLACED`, `DETECTION_UNAVAILABLE`, `PLAN_STALE`, `CALIBRATION_CHANGED`, `COLLISION`, `TRACKING_STALL`, `NOT_HOLDING`, `DROPPED`, `GRASP_TIMEOUT`, `LIFT_TORQUE`, `INSUFFICIENT_EVIDENCE`, `BUDGET_EXHAUSTED`, `IK_UNREACHABLE`, `IK_BRANCH_FLIP`, `WORKSPACE_VIOLATION`.
- **No automatic retries**: on `NOT_HOLDING`, `armd` withdraws with jaws closed along `-insertion_dir` (never opening jaws at contact) and terminates failed. On `DROPPED`, `armd` terminates failed without regripping. On success, `action.holding == true` and `HeldObject` is committed for a subsequent `place(...)`.

---

## 2. Patch for `rebot/client.py`

### Summary of Changes
1. Add `ObservationSchema.require_grasp_select()` and `ObservationSchema.require_grasp_select_exec()` next to `require_grasp_tasks()` (line 950).
2. Add `_validate_grasp_strategy(strategy)` and `_build_grasp_body(...)` helper to eliminate duplication across `ArmdClient.grasp_dry_run*`, `ArmSession.grasp`, and `ArmSession.grasp_dry_run*`.
3. Extend `ArmdClient.grasp_dry_run(...)` with optional `strategy`, `evidence`, `pose_preferences`, `budget_ms` (preserving its 3-tuple return type) and add `ArmdClient.grasp_dry_run_detail(...) -> dict` returning the full server payload (`failing_phase`, `hint`, `outcome`, `pregrasp_mode`, `expected_duration_s`, `selected_candidate`, `budget`, `legacy`).
4. Extend `ArmSession.grasp(...)`, `ArmSession.grasp_dry_run(...)`, and add `ArmSession.grasp_dry_run_detail(...)` with the same parameters and capability gating (`require_grasp_select()` for adaptive dry-run, `require_grasp_select_exec()` for live adaptive grasp).

### Unified Diff (`rebot/client.py`)

```diff
--- a/rebot/client.py
+++ b/rebot/client.py
@@ -953,6 +953,20 @@ class ObservationSchema:
             raise UnsupportedError(
                 "Grasp tasks need an armd that reports capability 'grasp_tasks'. Update rebot-armd."
             )
+
+    def require_grasp_select(self) -> None:
+        """Raise :class:`UnsupportedError` unless armd reports capability ``grasp_select: true``."""
+        if self.capabilities.get("grasp_select") is not True:
+            raise UnsupportedError(
+                "Adaptive grasp selection needs a 6-DOF armd reporting capability 'grasp_select'."
+            )
+
+    def require_grasp_select_exec(self) -> None:
+        """Raise :class:`UnsupportedError` unless armd reports capability ``grasp_select_exec: true``."""
+        if self.capabilities.get("grasp_select_exec") is not True:
+            raise UnsupportedError(
+                "Live adaptive grasp execution needs a 6-DOF armd reporting capability 'grasp_select_exec'."
+            )
 
     def has(self, capability: str, min_version: int = CAPABILITIES_DEVPLAT) -> bool:
         """``capabilities[capability]`` is true and ``capabilities.version >= min_version``."""
@@ -1208,45 +1222,74 @@ class ArmdClient:
         """``GET /api/v1/gripper/calibration`` (no lease required)."""
         return dict(self._get("/api/v1/gripper/calibration") or {})
 
-    def grasp_dry_run(
+    def grasp_dry_run_detail(
         self,
         label: str,
         box_norm: Sequence[float],
         frame_id: int | None = None,
         ts_ms: float | None = None,
         overrides: dict | None = None,
         tag: str | None = None,
         timeout: float | None = None,
         task: str | None = None,
-    ) -> tuple[bool, str | None, str | None]:
-        """Check grasp reachability over REST (no lease required).
-
-        With ``task`` (needs capability ``grasp_tasks``, else :class:`UnsupportedError`) an unsupported task
-        returns ``(False, "TASK_UNSUPPORTED", reason)``.
-        """
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
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
+    ) -> dict:
+        """Check grasp reachability over REST (no lease required) and return the full reply dict.
+
+        When ``strategy == "adaptive"``, requires capability ``grasp_select`` and returns the
+        adaptive selection / preflight fields (``outcome``, ``failing_phase``, optional checked
+        ``hint``, ``pregrasp_mode``, ``expected_duration_s``, ``selected_candidate``, ``budget``,
+        ``legacy``). Legacy remains the default when ``strategy`` is omitted or ``"legacy"``.
+        """
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
+        """Check grasp reachability over REST (no lease required)."""
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
@@ -1698,6 +1741,47 @@ class ArmdClient:
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
+) -> dict:
+    body: dict = {"label": str(label), "box_norm": box_norm}
+    if dry_run:
+        body["dry_run"] = True
+    if frame_id is not None:
+        body["frame_id"] = int(frame_id)
+    if ts_ms is not None:
+        body["ts_ms"] = float(ts_ms)
+    if overrides:
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
+        body["budget_ms"] = int(budget_ms)
+    return body
+
+
 def _dry_run_tuple(data: dict) -> tuple[bool, str | None, str | None]:
     """``(reachable, code, message)`` from a dry-run reply; ``task_supported: false`` -> TASK_UNSUPPORTED."""
     if data.get("task_supported") is False:
@@ -2894,76 +2978,124 @@ class ArmSession:
     def grasp(
         self,
         label: str,
         box_norm: Sequence[float],
         frame_id: int | None = None,
         ts_ms: float | None = None,
         overrides: dict | None = None,
         tag: str | None = None,
         timeout: float | None = None,
         task: str | None = None,
+        strategy: str | None = None,
+        evidence: dict | None = None,
+        pose_preferences: dict | None = None,
+        budget_ms: int | None = None,
     ) -> ActionResult:
         """``/api/v1/action/grasp``: armd centres on the box, descends, closes and lifts.
 
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
         frame_id: int | None = None,
         ts_ms: float | None = None,
         overrides: dict | None = None,
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
+        """Check grasp reachability over WebSocket (returns (reachable, code, message))."""
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
```

---

## 3. Patch for `rebot/arm.py`

### Summary of Changes
1. Add `_HINTS` entries for Phase B terminal/refusal codes (`LIFT_TORQUE`, `INSUFFICIENT_EVIDENCE`, `BUDGET_EXHAUSTED`, `TARGET_DISPLACED`, `NOT_HOLDING`, `DROPPED`, `CALIBRATION_CHANGED`, `PLAN_STALE`). Do **not** fabricate unchecked alternative hints (surface server-checked `hint` only when `armd` sends it in `message`/`raw["hint"]`).
2. Add `_adaptive_evidence(target, evidence)` helper so passing a `Detection` with `strategy="adaptive"` forwards available oriented/metric evidence (`obb_corners_norm`, `angle_deg`, `angle_conf`, `height_m`) merged under any explicit `evidence` dict.
3. Extend `Arm.grasp`, `Arm.grasp_dry_run`, add `Arm.grasp_dry_run_detail`, and extend `Arm.grasp_any` with `strategy`, `evidence`, `pose_preferences`, `budget_ms`. Gate adaptive dry-run on `grasp_select` and live adaptive grasp on `grasp_select_exec` before dispatching.

### Unified Diff (`rebot/arm.py`)

```diff
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
+    "NOT_HOLDING": "closed without holding the object and withdrew safely without retrying",
+    "DROPPED": "lost grip after lifting the object (no automatic retry)",
+    "CALIBRATION_CHANGED": "aborted because camera or arm calibration changed mid-grasp",
+    "PLAN_STALE": "aborted because arm state or workspace/grasp configuration changed after planning",
 }
 
 SENSITIVITIES = ("low", "medium", "high")
@@ -155,6 +163,25 @@ def _tag_kw(tag: Optional[str]) -> dict:
     return {} if tag is None else {"tag": tag}
 
 
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
 class Arm:
@@ -702,11 +729,15 @@ class Arm:
     def grasp(
         self,
         target: Union[Detection, str],
         box: Optional[Sequence[float]] = None,
         timeout: Optional[float] = None,
         task: Optional[str] = None,
         tag: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
     ) -> ActionResult:
         """Pick up a detected object: ``arm.grasp(detection)`` or ``arm.grasp("cup", box=(x0, y0, x1, y1))``.
 
@@ -716,6 +747,9 @@ class Arm:
         ``task`` names a grasp task stored in armd. Without the ``grasp_tasks`` capability this raises
         :class:`UnsupportedError`; when the loaded vision model/depth cannot serve the task armd rejects the
         command with ``TASK_UNSUPPORTED`` (:class:`ArmCommandRejected`) before the arm moves.
+
+        ``strategy="adaptive"`` runs Phase B adaptive grasp selection and execution (requires capability
+        ``grasp_select_exec``). Legacy remains the default when ``strategy`` is omitted.
         """
         if isinstance(target, Detection):
             label, box_norm, frame_id, ts_ms = target.label, target.box_norm, target.frame_id, target.ts_ms
@@ -730,22 +764,45 @@ class Arm:
             )
         if task is not None:
             self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select_exec()
+        eff_evidence = _merge_adaptive_evidence(target, evidence, strategy)
         return self._motion(
             lambda: self.session.grasp(
-                label, box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task, **_tag_kw(tag)
+                label,
+                box_norm,
+                frame_id=frame_id,
+                ts_ms=ts_ms,
+                timeout=timeout,
+                task=task,
+                strategy=strategy,
+                evidence=eff_evidence,
+                pose_preferences=pose_preferences,
+                budget_ms=budget_ms,
+                **_tag_kw(tag),
             ),
             f"grasp of '{label}'",
             timeout,
         )
 
-    def grasp_dry_run(
+    def grasp_dry_run_detail(
         self,
         target: Union[Detection, str],
         box: Optional[Sequence[float]] = None,
         timeout: Optional[float] = None,
         task: Optional[str] = None,
-    ) -> Tuple[bool, Optional[str], Optional[str]]:
-        """Check reachability of a detection without moving the arm.
-
-        Returns (reachable, code, message). With ``task`` (capability ``grasp_tasks``, else
-        :class:`UnsupportedError`) a task the loaded vision model/depth cannot serve returns
-        ``(False, "TASK_UNSUPPORTED", reason)``.
-        """
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
+    ) -> dict:
+        """Check reachability without moving and return the full server reply dict."""
         if isinstance(target, Detection):
             label, box_norm, frame_id, ts_ms = target.label, target.box_norm, target.frame_id, target.ts_ms
         else:
@@ -760,21 +817,67 @@ class Arm:
             label, box_norm, frame_id, ts_ms = str(target), tuple(box), None, None
         if task is not None:
             self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select()
+        eff_evidence = _merge_adaptive_evidence(target, evidence, strategy)
         return self._run(
-            lambda: self.session.grasp_dry_run(
-                label=label, box_norm=box_norm, frame_id=frame_id, ts_ms=ts_ms, timeout=timeout, task=task
+            lambda: self.session.grasp_dry_run_detail(
+                label=label,
+                box_norm=box_norm,
+                frame_id=frame_id,
+                ts_ms=ts_ms,
+                timeout=timeout,
+                task=task,
+                strategy=strategy,
+                evidence=eff_evidence,
+                pose_preferences=pose_preferences,
+                budget_ms=budget_ms,
             )
         )
 
+    def grasp_dry_run(
+        self,
+        target: Union[Detection, str],
+        box: Optional[Sequence[float]] = None,
+        timeout: Optional[float] = None,
+        task: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
+    ) -> Tuple[bool, Optional[str], Optional[str]]:
+        """Check reachability of a detection without moving the arm."""
+        if isinstance(target, Detection):
+            label, box_norm, frame_id, ts_ms = target.label, target.box_norm, target.frame_id, target.ts_ms
+        else:
+            if box is None:
+                raise ValueError(
+                    "grasp_dry_run(label) needs box=(x0, y0, x1, y1); or pass a detection from arm.detect()"
+                )
+            label, box_norm, frame_id, ts_ms = str(target), tuple(box), None, None
+        if task is not None:
+            self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select()
+        eff_evidence = _merge_adaptive_evidence(target, evidence, strategy)
+        return self._run(
+            lambda: self.session.grasp_dry_run(
+                label=label,
+                box_norm=box_norm,
+                frame_id=frame_id,
+                ts_ms=ts_ms,
+                timeout=timeout,
+                task=task,
+                strategy=strategy,
+                evidence=eff_evidence,
+                pose_preferences=pose_preferences,
+                budget_ms=budget_ms,
+            )
+        )
+
     def grasp_any(
         self,
         label: Optional[str] = None,
         min_score: float = 0.3,
         max_candidates: int = 5,
         timeout: Optional[float] = None,
         task: Optional[str] = None,
+        strategy: Optional[str] = None,
+        evidence: Optional[dict] = None,
+        pose_preferences: Optional[dict] = None,
+        budget_ms: Optional[int] = None,
     ) -> ActionResult:
         """Detect objects, dry-run in score order, and grasp the first reachable one."""
         if task is not None:
             self.schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            self.schema.require_grasp_select()
+            self.schema.require_grasp_select_exec()
         candidates = self.detect(label=label, min_score=min_score)
         bounded_candidates = candidates[:max(0, int(max_candidates))]
         last_code: Optional[str] = None
         last_message: Optional[str] = None
         for d in bounded_candidates:
             self.feed()
-            reachable, code, message = self.grasp_dry_run(d, timeout=timeout, task=task)
+            reachable, code, message = self.grasp_dry_run(
+                d,
+                timeout=timeout,
+                task=task,
+                strategy=strategy,
+                evidence=evidence,
+                pose_preferences=pose_preferences,
+                budget_ms=budget_ms,
+            )
             if code == "TASK_UNSUPPORTED" and not reachable:
                 raise ActionFailed(code, message or "grasp task not supported by the loaded vision model")
             if reachable:
-                return self.grasp(d, timeout=timeout, task=task)
+                return self.grasp(
+                    d,
+                    timeout=timeout,
+                    task=task,
+                    strategy=strategy,
+                    evidence=evidence,
+                    pose_preferences=pose_preferences,
+                    budget_ms=budget_ms,
+                )
             last_code = code
             last_message = message
```

---

## 4. Patch for `rebot_companion/mcp_server.py`

### Summary of Changes
1. Extend `Controller.grasp_dry_run(...)` and `@tool grasp_dry_run` with optional `strategy` (`"legacy" | "adaptive"`), `evidence`, `pose_preferences`, and `budget_ms`.
   - When `strategy` is omitted or `"legacy"`, return the exact existing `{reachable, code, message}`.
   - When `strategy == "adaptive"`, call `self.client.grasp_dry_run_detail(...)` (which enforces capability `grasp_select`) and return `{reachable, code, message, strategy, outcome, failing_phase, pregrasp_mode, expected_duration_s, selected_candidate, margins, legacy}` plus `hint` **only** when `armd` returned a non-null checked `hint`.
2. Add `Controller.grasp(...)` and actuator `@tool grasp` (inside the `if actuate:` registration block alongside `place`), defaulting `strategy` to `"legacy"`:
   - Enforces `grasp` capability, `grasp_tasks` when `task` is set, and `grasp_select_exec` when `strategy == "adaptive"`.
   - Surfaces `failing_phase` and server-checked `hint` (if present in `ArmCommandRejected.raw`) on `ToolFail`.
   - Performs no automatic retries; returns the action dict including `holding` so a follow-up `place` call can consume the held object.

### Unified Diff (`rebot_companion/mcp_server.py`)

```diff
--- a/rebot_companion/mcp_server.py
+++ b/rebot_companion/mcp_server.py
@@ -1682,31 +1682,115 @@ class Controller:
     def grasp_dry_run(
         self,
         label: str,
         box_norm: list[float],
         frame_id: int | None = None,
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
             raise ToolFail("INVALID_ARGUMENT", "label must be a non-empty string")
         if (
             not isinstance(box_norm, list)
             or len(box_norm) != 4
             or not all(
                 isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                 for v in box_norm
             )
         ):
             raise ToolFail("INVALID_ARGUMENT", "box_norm must be 4 finite numbers [x0, y0, x1, y1]")
-        reachable, code, message = self.client.grasp_dry_run(
-            label,
-            [float(v) for v in box_norm],
-            frame_id=frame_id,
-            ts_ms=ts_ms,
-            timeout=timeout_s,
-            task=task,
-        )
-        return {"reachable": bool(reachable), "code": code, "message": message}
+        if strategy is not None and strategy not in ("legacy", "adaptive"):
+            raise ToolFail("INVALID_ARGUMENT", "strategy must be 'legacy' or 'adaptive'")
+        if strategy != "adaptive":
+            reachable, code, message = self.client.grasp_dry_run(
+                label,
+                [float(v) for v in box_norm],
+                frame_id=frame_id,
+                ts_ms=ts_ms,
+                timeout=timeout_s,
+                task=task,
+                strategy=strategy,
+            )
+            return {"reachable": bool(reachable), "code": code, "message": message}
+        data = self.client.grasp_dry_run_detail(
+            label,
+            [float(v) for v in box_norm],
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            timeout=timeout_s,
+            task=task,
+            strategy="adaptive",
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+        )
+        out: dict[str, Any] = {
+            "reachable": bool(data.get("reachable", False)),
+            "code": data.get("code"),
+            "message": data.get("message"),
+            "strategy": "adaptive",
+            "outcome": data.get("outcome"),
+            "failing_phase": data.get("failing_phase"),
+            "legacy": data.get("legacy"),
+        }
+        if data.get("hint"):
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
+        """Execute grasp over the active control lease (/api/v1/action/grasp). No automatic retries."""
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
+        session = self._need_control()
+        schema = session.schema()
+        if schema.capabilities.get("grasp") is False:
+            raise ToolFail("UNSUPPORTED", "this armd profile does not support grasp")
+        if task is not None:
+            schema.require_grasp_tasks()
+        if strategy == "adaptive":
+            schema.require_grasp_select_exec()
+        res = session.grasp(
+            label=label,
+            box_norm=[float(v) for v in box_norm],
+            frame_id=frame_id,
+            ts_ms=ts_ms,
+            task=task,
+            strategy=strategy,
+            evidence=evidence,
+            pose_preferences=pose_preferences,
+            budget_ms=budget_ms,
+            timeout=timeout_s,
+        )
+        return self._action_dict(res.action)
```

```diff
--- a/rebot_companion/mcp_server.py
+++ b/rebot_companion/mcp_server.py
@@ -3051,18 +3051,38 @@ def build_server(...):
     @tool(
         "Check grasp reachability for a detected box [x0, y0, x1, y1] (normalised 0..1) without moving "
         "or acquiring a control lease (REST POST /api/v1/action/grasp with dry_run=true). "
-        "Returns {reachable, code, message}. Optional task needs armd capability 'grasp_tasks'."
+        "Legacy strategy is the default and returns {reachable, code, message}. Set strategy='adaptive' "
+        "(needs 6-DOF capability 'grasp_select') to preflight oblique/swept-joint grasp candidates "
+        "with full route checks and return {reachable, code, message, strategy, outcome, failing_phase, "
+        "pregrasp_mode, expected_duration_s, selected_candidate, margins, budget, legacy} plus optional "
+        "checked hint. Optional task needs capability 'grasp_tasks'."
     )
     async def grasp_dry_run(
         label: str,
         box_norm: Annotated[list[float], Field(min_length=4, max_length=4)],
         frame_id: int | None = None,
         ts_ms: float | None = None,
         task: str | None = None,
         timeout_s: Annotated[float | None, Field(gt=0, le=60)] = None,
+        strategy: Literal["legacy", "adaptive"] | None = None,
+        evidence: dict[str, Any] | None = None,
+        pose_preferences: dict[str, Any] | None = None,
+        budget_ms: Annotated[int | None, Field(ge=10, le=5000)] = None,
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
@@ -3386,6 +3406,40 @@ def build_server(...):
+        @tool(
+            "MOVES THE ARM. Grasp a detected object [x0, y0, x1, y1] (normalised 0..1). Legacy visual-servo "
+            "is the default; set strategy='adaptive' (needs 6-DOF capability 'grasp_select_exec') to run "
+            "the preflighted Phase B adaptive grasp. No automatic retries; on success returns holding=true "
+            "ready for place. Call grasp_dry_run first when uncertain. Blocks until done."
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
+            budget_ms: Annotated[int | None, Field(ge=10, le=5000)] = None,
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
```

Also surface `failing_phase` and `hint` (only when present in `exc.raw`) inside `mcp_server.py`'s `ArmCommandRejected` -> `ToolFail` mapper so MCP callers see the exact failing phase and server-checked hint without inventing unchecked hints:

```diff
--- a/rebot_companion/mcp_server.py
+++ b/rebot_companion/mcp_server.py
@@ -112,6 +112,12 @@ def _map_error(exc: Exception) -> dict:
         if isinstance(exc, ArmCommandRejected):
             out = {"ok": False, "code": exc.code, "message": exc.message}
+            if exc.raw.get("failing_phase"):
+                out["failing_phase"] = exc.raw["failing_phase"]
+            if exc.raw.get("outcome"):
+                out["outcome"] = exc.raw["outcome"]
+            if exc.raw.get("hint"):
+                out["hint"] = exc.raw["hint"]
             return out
```

---

## 5. Companion Verification Checklist (when applying in `rebot-companion`)

1. **Capability gating**:
   - `capabilities.grasp_select == False` -> `grasp_dry_run(..., strategy="adaptive")` raises `UnsupportedError` before HTTP/WS traffic.
   - `capabilities.grasp_select == True` and `capabilities.grasp_select_exec == False` -> adaptive dry-run succeeds; live `grasp(..., strategy="adaptive")` raises `UnsupportedError`.
   - `strategy=None` and `strategy="legacy"` remain unaffected on both 5-DOF and 6-DOF profiles.
2. **Dry-run / live parity**:
   - `grasp_dry_run_detail(..., strategy="adaptive")` returns `failing_phase`, `outcome`, `pregrasp_mode`, `expected_duration_s`, `budget`, and includes `hint` **only** when `armd` returned a checked alternative.
   - `Arm.grasp_any(..., strategy="adaptive")` forwards the exact same `strategy`, `evidence`, `pose_preferences`, `budget_ms` from `grasp_dry_run` to `grasp`.
3. **MCP surface**:
   - `grasp_dry_run` legacy call keeps exact keys `{reachable, code, message}`.
   - `grasp_dry_run` adaptive call returns `failing_phase`, `outcome`, `selected_candidate`, `pregrasp_mode`, `expected_duration_s`, `margins`, `budget`, `legacy`, and optional checked `hint`.
   - `grasp` tool defaults to legacy, enforces `grasp_select_exec` when `strategy="adaptive"`, performs zero automatic retries, and returns `holding=true` consumable by `place`.
