# Policy runtime (`rebot-onnx-policy/v1`)

English | [中文](policy_runtime.zh-CN.md)

`armd` runs learned joint-position policies inside `src/policy_runtime/`. A policy pack is a directory containing `policy.json` and one model file (`.rmlp`, `.mlp.json`, or `.onnx`), loaded once into an immutable snapshot (`PolicySnapshot`) and bound to a deterministic SHA-256 `release_digest`. The digest hashes two framed entries in order — `"policy.json"` and the normalized POSIX relative model path — each formatted as `<path>\0<decimal_byte_len>\0<raw_bytes>`.

## `policy.json` fields (`rebot-onnx-policy/v1`)

`PolicySnapshot::load_from_dir` (`src/policy_runtime/manifest.rs`) enforces file-size limits at snapshot load: `policy.json` must be a non-empty regular file of at most `4 MiB` (`4,194,304` bytes), and `model_file` must be a non-empty regular file of at most `8 MiB + 4 KiB` (`8,392,704` bytes). `model_file` may be a nested relative path inside the policy directory (for example `weights/model.rmlp`), with no leading `/` or `\`, no `..`, no drive prefix, no NUL byte, no leading/trailing whitespace, and no symlink escaping the pack directory. `StrictPolicyContract::parse` validates the manifest fields below.

| Field | Type | Limits / values | Default / required |
|---|---|---|---|
| `format` | string | `"rebot-onnx-policy/v1"` | required |
| `name` | string | any string | `""` |
| `robot` | string \| null | non-empty string | `null` |
| `model_file` | string | safe relative path inside pack dir (nested subdirs allowed); size `1..=8,392,704` bytes enforced at snapshot load | `"model.onnx"` |
| `unit_mode` | string | `"rad"` or `"so101_norm"` | required |
| `action_dim` | integer | `6` or `7` | required |
| `action_space` | string | `"joint_positions"` (must match `outputs.action.space` if both set; `null` rejected) | required (top-level or `outputs.action.space`) |
| `outputs.action.space` | string | `"joint_positions"` (`null` rejected) | see `action_space` |
| `outputs.action.shape` | integer array | `[N]` or `[1, N]` where `N = action_dim * chunk_size`; unflattened `[K, D]` and `[1, K, D]` are rejected | optional |
| `inputs.state.shape` | integer array | `[action_dim]` or `[1, action_dim]` | optional |
| `joint_names` | string array \| null | `action_dim` unique non-empty names (must match `inputs.state.joint_names` if both non-null); `null` treated as absent | required (at least one of top-level or `inputs.state.joint_names`) |
| `inputs.state.joint_names` | string array \| null | `action_dim` unique non-empty names; `null` treated as absent | see `joint_names` |
| `normalization` | object \| null | optional container (`null` treated as absent; identity if omitted) | optional (`null`) |
| `normalization.state_mean`, `normalization.state_std` (or `normalization.state.{mean,std}`) | `f32[action_dim]` pair \| null | finite numbers; `std` strictly `> 0.0`; each pair must be provided together (both non-null or both null/absent); nested `normalization.state` may be `null`; flat and nested forms must match if both present | optional (identity) |
| `normalization.action_mean`, `normalization.action_std` (or `normalization.action.{mean,std}`) | `f32[action_dim]` pair \| null | finite numbers; `std` strictly `> 0.0`; each pair must be provided together (both non-null or both null/absent); nested `normalization.action` may be `null`; flat and nested forms must match if both present | optional (identity) |
| `fps` | number \| null | finite `f64 > 0.0`; when `chunk_size > 1`, `replan_every * (1000 / fps) <= 1000 ms` | `null` (runtime precedence: start request `fps`, then manifest `fps`, then `20.0`) |
| `max_step_rad` | number \| null | finite `f32 > 0.0` | `null` (unset at parse; no per-step cap beyond servo limits) |
| `max_first_step_rad` | number \| null | finite `f32 > 0.0` | `null` (unset at parse; runtime defaults to `0.25` rad and clamps declared values to `min(value, 0.25)`) |
| `vision` | object \| null | total input width `action_dim + vision_feature_dim <= 1024` | `null` |
| `chunking` | object \| null | see `chunking.*` rows below | `{chunk_size: 1, replan_every: 1, temporal_ensemble_k: null}` |

### `vision` fields (`vision.source == "detector"`)

| Field | Type | Limits / values | Default / required |
|---|---|---|---|
| `vision.source` | string | `"detector"` | required |
| `vision.layout` | string | `"top1_box_norm_v1"` (6 floats: `[present, cx, cy, w, h, score]`) | required |
| `vision.target_label` | string \| null | non-empty string | `null` (any label) |
| `vision.min_score` | number \| null | finite `f32` in `[0.0, 1.0]` | `0.3` |
| `vision.max_staleness_ms` | integer | `1..=500` | required |
| `vision.max_sync_skew_ms` | integer \| null | `1..=500` | `vision.max_staleness_ms` |
| `vision.on_target_lost` | string \| null | `"hold"`, `"zero_features"`, or `"stop"` | `"hold"` |
| `vision.on_stale` | string \| null | `"stop"` or `"hold"` | `"stop"` |
| `vision.require_model_generation` | bool \| null | boolean | `true` |

### `vision` fields (`vision.source == "video_embedding"`)

| Field | Type | Limits / values | Default / required |
|---|---|---|---|
| `vision.source` | string | `"video_embedding"` | required |
| `vision.layout` | string | `"embedding_v1"` | required |
| `vision.embedding_dim` | integer | `>= 1` | required |
| `vision.history_len` | integer | `>= 1`; `action_dim + embedding_dim * history_len <= 1024` | required |
| `vision.max_staleness_ms` | integer | `1..=500` | required |
| `vision.max_window_span_ms` | integer | `1..=2000`; must be `>= max_staleness_ms` when `history_len > 1` | required |
| `vision.fps_staleness_budget_ms` | integer \| null | `>= 1` | `null` (unset at parse; runtime defaults to `clamp(round(2000 / runner_fps), 50, 500)` ms) |
| `vision.encoder_artifact_sha256` | string | 64-character hex SHA-256 | required |
| `vision.preprocess_config_sha256` | string | 64-character hex SHA-256 | required |
| `vision.embedding_norm` | object \| null | `{mean, std}`: finite `f32[embedding_dim]`, `std > 0.0` | `null` |
| `vision.on_stale` | string \| null | `"stop"` or `"hold"` | `"stop"` |

### `chunking` fields

| Field | Type | Limits / values | Default / required |
|---|---|---|---|
| `chunking.chunk_size` | integer \| null | `1..=128`; `chunk_size * action_dim <= 1024` | `1` |
| `chunking.replan_every` | integer \| null | `1..=chunk_size`; when `chunk_size > 1`, `replan_every * (1000 / fps) <= 1000 ms` (single-step policies `chunk_size == 1` are exempt) | `chunk_size` |
| `chunking.temporal_ensemble_k` | number \| null | finite `f32` in `0.0..=100.0` | `null` (open-loop, no ensembling) |

## Backends

`load_backend` (`src/policy_runtime/mod.rs`) selects the backend from the artifact bytes and filename, and checks that `input_dim == action_dim + vision_feature_dim` and `output_dim == action_dim * chunk_size`.

| Backend | Artifact | Availability | Resource limits |
|---|---|---|---|
| `mlp` (`MlpBackend`) | Binary `RMLP\x01` (`.rmlp`) or JSON (`.mlp.json`, max `4 MiB`) | always compiled; zero heap allocation inside `MlpBackend::infer` | `MAX_LAYERS = 16`, `MAX_DIM = 1024`, `MAX_PARAMS = 2,000,000`, `MAX_MACS = 4,000,000`; activations `none`, `relu`, `tanh`, `gelu` |
| `tract-onnx` (`TractBackend`) | `.onnx` | Cargo feature `policy-tract` | static `f32` `[1, input_dim]` → `[1, action_dim * chunk_size]`; external tensor data, subgraphs, sparse tensors, and model-local functions forbidden; `MAX_MODEL_BYTES = 8 MiB + 4 KiB`, `MAX_ONNX_NODES = 512`, `MAX_ONNX_PARAM_ELEMENTS = 2,000,000`, `MAX_ONNX_ACTIVATION_ELEMENTS = 1,000,000` |

## Shadow vs live mode

Policies start via WebSocket `/api/v1/action/policy/start` with `mode` set to `"shadow"`, `"live_sim"`, or `"live"` (`"live_gated"` aliases `"live"`). Effective rate precedence is: start request `fps`, then manifest `fps`, then `DEFAULT_POLICY_FPS = 20.0`.

| Property | `shadow` | `live` / `live_sim` |
|---|---|---|
| Commands servo loop | no (publishes `observation.policy.last_shadow_target`) | yes, through `validate_and_filter_servo_targets` (soft limits, workspace, collision, torque guard) |
| Allowed `fps` | `1.0..=50.0` | `10.0..=50.0` |
| Start warm-up speed gate (`p50 > 0.5 * period_us`) | allowed (reports `too_slow: true` in start ACK) | refused with 422 `POLICY_TOO_SLOW` |
| `STALE_INPUT`, `max_first_step_rad`, `max_step_rad` | not enforced on shadow output | enforced before every commit |
| Real-arm authorization (`!policy_live_allowed()`, i.e. non-mock/non-sim) | active lease + operator session (required for `/api/v1/action/policy/start` in any mode on a real arm) | active lease + operator session + `i_am_at_the_arm: true`, verified `rebot-release-gate/v1` bound to `artifact_digest` and active `calibration_digest`, confirmed hardware feedback, and (for `production_release` tier) device qualification ledger |

## Input freshness fences

All freshness fences fail closed before committing a live target.

- **Proprioception (`FEEDBACK_UNCONFIRMED` / `STALE_INPUT`)**: on a real arm (`!policy_live_allowed()`), missing or unconfirmed driver feedback (including `policy_input_age_range_ms().is_none()`) terminates the run with 409 `FEEDBACK_UNCONFIRMED` before the `STALE_INPUT` check runs. When feedback is confirmed, capture-to-commit age is checked against `max_policy_input_age_ms(fps) = clamp(round(2000 / fps), 50, 200)` ms (200 ms at 10 fps, 100 ms at 20 fps, 80 ms at 25 fps, 50 ms at 50 fps); exceeding it (or passing an unknown age on a real arm) terminates the run with 409 `STALE_INPUT`.
- **Detector vision**: checks capture timestamp presence, `vision_age_ms <= vision.max_staleness_ms` (both before inference and before commit), `|vision_age_ms - prop_age_ms| <= vision.max_sync_skew_ms`, `model_generation` lock (`VISION_GENERATION_MISSING` / `VISION_GENERATION_CHANGED`), and target presence (`hold`, `zero_features`, or `stop` with `TARGET_LOST`).
- **Video embedding**: checks window freshness against `min(max_staleness_ms, effective_fps_budget_ms, 500)` ms (where unset `fps_staleness_budget_ms` defaults at runtime to `clamp(round(2000 / runner_fps), 50, 500)` ms) and span against `max_window_span_ms` (`<= 2000` ms), verifies bilateral clock handshake (`DEFAULT_HANDSHAKE_TTL_MS = 60,000` ms), and re-verifies the `InferenceTicket` (policy instance, lease generation, producer epoch, model generation, invalidation generation, clock handshake, and window deadlines) inside the commit critical section under `HubCommitGuard`.

## `max_first_step_rad` and `max_step_rad`

- **`max_first_step_rad`**: unset (`None`) at manifest parse when omitted or `null`. At `begin_policy_run`, the runtime defaults to `DEFAULT_MAX_FIRST_STEP_RAD = 0.25` rad and clamps any declared value to `min(value, 0.25)` rad. Checked on the first live step against `measured_positions` on every joint; exceeding it terminates the run with 422 `JUMP_LIMIT`.
- **`max_step_rad`**: optional manifest limit checked on every subsequent live step against the previous committed live target (`last_live_target`). Exceeding it terminates the run with 422 `STEP_LIMIT`.

## `policy_stats` and warm-up speed gate

Before `begin_policy_run`, `PreparedPolicy::warmup` runs up to `WARMUP_INFERENCES = 10` inferences, stopping after the inference that reaches or crosses `WARMUP_MAX_TOTAL = 5 s` (because an individual inference cannot be interrupted, total warm-up duration can exceed 5 s by up to one inference). The first sample is discarded when more than one ran. The start ACK (and `POLICY_TOO_SLOW` rejection) includes:

- `warmup_infer_us`: `{p50, max, n}` (microseconds)
- `too_slow`: `p50 > 0.5 * period_us` (`SPEED_GATE_PERIOD_FRACTION = 0.5`)
- `period_us`: `round(1e6 / fps)`
- `max_fps`: `floor(0.5 * 1e6 / max(p50, 1))`

During and after the run, `ActionStatus::policy_stats` is updated by `record_policy_step_timing` under the supervisor write lock **before** `commit_gate` and `policy_step_with_vision` submission:

- `steps`: runner inference/step cycles recorded (recorded before submission; includes cycles later skipped by `commit_gate` or rejected at submission)
- `overruns`: cycles whose tick-to-record duration `step_us` exceeded `period_us`
- `skipped`: cumulative `floor(step_us / period_us)` across overrunning cycles
- `period_us`: configured control period in microseconds
- `infer_us`: `{p50, p99, max}` in microseconds over a fixed 256-sample ring (`POLICY_STATS_RING = 256`, no per-step allocation)

## Action chunking

`armd` implements ACT-style action chunking and temporal ensembling (Zhao et al., 2023, *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*). When `chunking.chunk_size > 1`, the model outputs a flattened `[1, chunk_size * action_dim]` tensor containing `chunk_size` consecutive normalized (or raw, when `normalization` is omitted) joint targets in row-major order.

- **Parameters**:
  - `chunk_size` (`1..=128`, default `1`): number of future control periods predicted per inference.
  - `replan_every` (`1..=chunk_size`, default `chunk_size`): number of elapsed control periods between inferences.
  - `temporal_ensemble_k` (`0.0..=100.0`, finite as `f32`, default `null`): when `null`, each replan replaces the ring and executes the latest chunk open-loop. When set to `k`, overlapping predictions targeting the same control period are averaged with exponential weights:

    $$w_i = \exp(-k \cdot i), \quad a = \frac{\sum_{i=0}^{m-1} w_i \, a_i}{\sum_{i=0}^{m-1} w_i}$$

    where predictions $a_0, \dots, a_{m-1}$ are ordered **oldest first** ($i = 0$ is the oldest surviving prediction for that period).
- **Preallocated ring and runner-thread allocations**: `PreparedPolicy::from_backend` allocates the chunk buffer (`ChunkRing`) and its fixed-size source provenance table once (`chunk_size` slots × `1` or `chunk_size` predictions × `action_dim` floats; zero bytes when `chunk_size <= 1`), so pushing chunks, pruning, and slot selection do not reallocate the ring. Unit conversion and target construction (`rad_to_policy_units`, `policy_units_to_rad`, and cloning the `Vec<f32>` submitted to `policy_step_with_vision`) still allocate short `action_dim`-length vectors on the runner thread.
- **Two-stage execution and single final `commit_gate(now)`**:
  - Off the supervisor lock (`plan_chunk_step`), the runner prunes expired/invalidated ring entries, advances the ring to the sampled period, runs backend inference when the ring is empty or `sample_period - last_replan_period >= replan_every`, and stages the converted candidate target without consuming the ring slot.
  - Under the supervisor write lock (and `HubCommitGuard` when video embeddings are active), after release-gate/deployment and current-tick `InferenceTicket` checks, `commit_gate_now` runs as the **last step before `policy_step_with_vision`**, reading the monotonic clock once (`now`):
    1. Computes `commit_period` from `now` and the interval origin, selects that period's slot, and discards passed slots. If `commit_period` was already committed/passed or is missing from the ring, submits nothing this tick (dropping the buffer if missing).
    2. Checks every contributing source age of the selected slot against `now`: for predictions from a fresh inference on the current tick, uses the normal limits (`max_policy_input_age_ms(fps)`, `vision.max_staleness_ms`, embedding ticket deadline with `horizon_ms = 0`); for cached predictions from earlier ticks, uses `max_policy_input_age_ms(fps) + replan_every * period` (and `vision.max_staleness_ms + replan_every * period`, embedding deadline `+ replan_every * period`). On expiry, clears the whole chunk ring, skips submission on this tick without failing the run, and forces a replan on the next period.
    3. Checks provenance of every contributing source: detector `model_generation`, detector `DetectorTargetIdentity` (re-read from the latest frame at commit time: `TrackId` when reported by the detector, otherwise 128-bit SHA-256 label digest + normalized box quantized to `0..=1000` + `frame_id`, or `Absent`), and embedding `InferenceTicket` identity/invalidation/deadlines. On mismatch (including top target replacement while presence remains true), clears the whole chunk ring and skips submission.
  - No further clock reads or freshness/period checks run between `commit_gate_now` and `policy_step_with_vision` (`input_age_ms` and detector `post_vision_age_ms` are derived from the gate's `now` reading).
- **Whole-ring invalidation triggers**: in addition to `commit_gate` mismatches/expiries, the runner clears the entire chunk ring on stop or E-stop, fault, lease change or revocation, epoch change, shadow↔live mode switch (`sync_run_context`), vision `Hold` or `Stop` decisions, and `policy_step_with_vision` errors.
- **1000 ms open-loop horizon cap**: for `chunk_size > 1`, `replan_every * (1000 / fps)` must be `<= 1000 ms` (`MAX_OPEN_LOOP_HORIZON_MS`). Enforced during `StrictPolicyContract::parse` when `policy.json` specifies `fps`, and at `/api/v1/action/policy/start` and `spawn_policy_runner` against the effective `fps`. Single-step policies (`chunk_size == 1`) are exempt.
