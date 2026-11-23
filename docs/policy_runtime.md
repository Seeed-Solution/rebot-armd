# Policy runtime (`rebot-onnx-policy/v1`)

English | [中文](policy_runtime.zh-CN.md)

`armd` runs learned joint-position policies inside `src/policy_runtime/`. A policy pack is a directory containing `policy.json` and one model file (`.rmlp`, `.mlp.json`, or `.onnx`), loaded once into an immutable snapshot (`PolicySnapshot`) and bound to a deterministic SHA-256 `release_digest` computed over `policy.json` and the model bytes.

## `policy.json` fields (`rebot-onnx-policy/v1`)

`StrictPolicyContract::parse` (`src/policy_runtime/manifest.rs`) parses `policy.json` (max `4 MiB`). Files referenced by `model_file` must be safe relative paths inside the policy directory (no `/`, `\`, `..`, drive prefix, NUL byte, leading/trailing whitespace, or symlink escape) and at most `8 MiB + 4 KiB` (`8,392,704` bytes).

| Field | Type | Limits / values | Default / required |
|---|---|---|---|
| `format` | string | `"rebot-onnx-policy/v1"` | required |
| `name` | string | any string | `""` |
| `robot` | string \| null | non-empty string | `null` |
| `model_file` | string | safe relative path inside pack dir; size `1..=8,392,704` bytes | `"model.onnx"` |
| `unit_mode` | string | `"rad"` or `"so101_norm"` | required |
| `action_dim` | integer | `6` or `7` | required |
| `action_space` | string | `"joint_positions"` (must match `outputs.action.space` if both set) | required (top-level or `outputs.action.space`) |
| `outputs.action.space` | string | `"joint_positions"` | see `action_space` |
| `outputs.action.shape` | integer array | `[N]` or `[1, N]` where `N = action_dim * chunk_size`; unflattened `[K, D]` and `[1, K, D]` are rejected | optional |
| `inputs.state.shape` | integer array | `[action_dim]` or `[1, action_dim]` | optional |
| `joint_names` | string array | `action_dim` unique non-empty names (must match `inputs.state.joint_names` if both set) | required (top-level or `inputs.state.joint_names`) |
| `inputs.state.joint_names` | string array | `action_dim` unique non-empty names | see `joint_names` |
| `normalization.state_mean`, `normalization.state_std` (or `normalization.state.{mean,std}`) | `f32[action_dim]` pair | finite numbers; `std` strictly `> 0.0`; both must appear together; flat and nested forms must match if both present | optional (identity) |
| `normalization.action_mean`, `normalization.action_std` (or `normalization.action.{mean,std}`) | `f32[action_dim]` pair | finite numbers; `std` strictly `> 0.0`; both must appear together; flat and nested forms must match if both present | optional (identity) |
| `fps` | number \| null | finite `f64 > 0.0`; with `chunk_size > 1`, `replan_every * (1000 / fps) <= 1000 ms` | `null` (falls back to start request `fps`, else `20.0`) |
| `max_step_rad` | number \| null | finite `f32 > 0.0` | `null` (no per-step cap beyond servo limits) |
| `max_first_step_rad` | number \| null | finite `f32 > 0.0`; effective cap is `min(value, 0.25)` | `0.25` rad (`DEFAULT_MAX_FIRST_STEP_RAD`) |
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
| `vision.fps_staleness_budget_ms` | integer \| null | `>= 1` | `clamp(round(2000 / fps), 50, 500)` |
| `vision.encoder_artifact_sha256` | string | 64-character hex SHA-256 | required |
| `vision.preprocess_config_sha256` | string | 64-character hex SHA-256 | required |
| `vision.embedding_norm` | object \| null | `{mean, std}`: finite `f32[embedding_dim]`, `std > 0.0` | `null` |
| `vision.on_stale` | string \| null | `"stop"` or `"hold"` | `"stop"` |

### `chunking` fields

| Field | Type | Limits / values | Default / required |
|---|---|---|---|
| `chunking.chunk_size` | integer \| null | `1..=128`; `chunk_size * action_dim <= 1024` | `1` |
| `chunking.replan_every` | integer \| null | `1..=chunk_size`; `replan_every * (1000 / fps) <= 1000 ms` | `chunk_size` |
| `chunking.temporal_ensemble_k` | number \| null | finite `f32` in `0.0..=100.0` | `null` (open-loop, no ensembling) |

## Backends

`load_backend` (`src/policy_runtime/mod.rs`) selects the backend from the artifact bytes and filename, and checks that `input_dim == action_dim + vision_feature_dim` and `output_dim == action_dim * chunk_size`.

| Backend | Artifact | Availability | Resource limits |
|---|---|---|---|
| `mlp` (`MlpBackend`) | Binary `RMLP\x01` (`.rmlp`) or JSON (`.mlp.json`, max `4 MiB`) | always compiled; zero heap allocation at inference | `MAX_LAYERS = 16`, `MAX_DIM = 1024`, `MAX_PARAMS = 2,000,000`, `MAX_MACS = 4,000,000`; activations `none`, `relu`, `tanh`, `gelu` |
| `tract-onnx` (`TractBackend`) | `.onnx` | Cargo feature `policy-tract` | static `f32` `[1, input_dim]` → `[1, action_dim * chunk_size]`; external tensor data, subgraphs, sparse tensors, and model-local functions forbidden; `MAX_MODEL_BYTES = 8 MiB + 4 KiB`, `MAX_ONNX_NODES = 512`, `MAX_ONNX_PARAM_ELEMENTS = 2,000,000`, `MAX_ONNX_ACTIVATION_ELEMENTS = 1,000,000` |

## Shadow vs live mode

Policies start via WebSocket `/api/v1/action/policy/start` with `mode` set to `"shadow"`, `"live_sim"`, or `"live"` (`"live_gated"` aliases `"live"`).

| Property | `shadow` | `live` / `live_sim` |
|---|---|---|
| Commands servo loop | no (publishes `observation.policy.last_shadow_target`) | yes, through `validate_and_filter_servo_targets` (soft limits, workspace, collision, torque guard) |
| Allowed `fps` | `1.0..=50.0` | `10.0..=50.0` |
| Start warm-up speed gate (`p50 > 0.5 * period_us`) | allowed (reports `too_slow: true` in start ACK) | refused with 422 `POLICY_TOO_SLOW` |
| `STALE_INPUT`, `max_first_step_rad`, `max_step_rad` | not enforced on shadow output | enforced before every commit |
| Real-arm preconditions (`!policy_live_allowed()`, i.e. non-mock/non-sim) | none beyond active lease | operator session, `i_am_at_the_arm: true`, verified `rebot-release-gate/v1` bound to `artifact_digest` and active `calibration_digest`, confirmed hardware feedback, and (for `production_release` tier) device qualification ledger |

## Input freshness fences

All freshness fences fail closed before committing a live target.

- **Proprioception (`STALE_INPUT`)**: measured from driver feedback capture to target commit under the supervisor write lock. Limit is `max_policy_input_age_ms(fps) = clamp(round(2000 / fps), 50, 200)` ms (200 ms at 10 fps, 100 ms at 20 fps, 80 ms at 25 fps, 50 ms at 50 fps). A target older than this (or with unknown age on a real arm) terminates the run with `STALE_INPUT`.
- **Detector vision**: checks capture timestamp presence, `vision_age_ms <= vision.max_staleness_ms` (both before inference and before commit), `|vision_age_ms - prop_age_ms| <= vision.max_sync_skew_ms`, `model_generation` lock (`VISION_GENERATION_MISSING` / `VISION_GENERATION_CHANGED`), and target presence (`hold`, `zero_features`, or `stop` with `TARGET_LOST`).
- **Video embedding**: checks window freshness against `min(max_staleness_ms, fps_staleness_budget_ms, 500)` ms and span against `max_window_span_ms` (`<= 2000` ms), verifies bilateral clock handshake (`DEFAULT_HANDSHAKE_TTL_MS = 60,000` ms), and re-verifies the `InferenceTicket` (policy instance, lease generation, producer epoch, model generation, invalidation generation, clock handshake, and window deadlines) inside the commit critical section under `HubCommitGuard`.

## `max_first_step_rad` and `max_step_rad`

- **`max_first_step_rad`**: checked on the first live step against `measured_positions` on every joint. The effective limit is `min(manifest.max_first_step_rad, 0.25)` rad (`DEFAULT_MAX_FIRST_STEP_RAD = 0.25`), so a manifest can tighten the bound but never raise it above 0.25 rad. Exceeding it terminates the run with 422 `JUMP_LIMIT`.
- **`max_step_rad`**: optional manifest limit checked on every subsequent live step against the previous committed live target (`last_live_target`). Exceeding it terminates the run with 422 `STEP_LIMIT`.

## `policy_stats` and warm-up speed gate

Before `begin_policy_run`, `PreparedPolicy::warmup` runs up to `WARMUP_INFERENCES = 10` inferences (bounded by `WARMUP_MAX_TOTAL = 5 s`), discarding the first sample when more than one ran. The start ACK (and `POLICY_TOO_SLOW` rejection) includes:

- `warmup_infer_us`: `{p50, max, n}` (microseconds)
- `too_slow`: `p50 > 0.5 * period_us` (`SPEED_GATE_PERIOD_FRACTION = 0.5`)
- `period_us`: `round(1e6 / fps)`
- `max_fps`: `floor(0.5 * 1e6 / max(p50, 1))`

During and after the run, `ActionStatus::policy_stats` reports:

- `steps`: completed steps recorded by the runner
- `overruns`: steps whose tick-to-commit duration `step_us` exceeded `period_us`
- `skipped`: cumulative `floor(step_us / period_us)` across overrunning steps
- `period_us`: configured control period in microseconds
- `infer_us`: `{p50, p99, max}` in microseconds over a fixed 256-sample ring (`POLICY_STATS_RING = 256`, no per-step allocation)

## Action chunking

`armd` implements ACT-style action chunking and temporal ensembling (Zhao et al., 2023, *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*). When `chunking.chunk_size > 1`, the model outputs a flattened `[1, chunk_size * action_dim]` tensor containing `chunk_size` consecutive normalized joint targets in row-major order.

- **Parameters**:
  - `chunk_size` (`1..=128`, default `1`): number of future control periods predicted per inference.
  - `replan_every` (`1..=chunk_size`, default `chunk_size`): number of elapsed control periods between inferences.
  - `temporal_ensemble_k` (`0.0..=100.0`, default `null`): when `null`, each replan replaces the ring and executes the latest chunk open-loop. When set to `k`, overlapping predictions targeting the same control period are averaged with exponential weights:

    $$w_i = \exp(-k \cdot i), \quad a = \frac{\sum_{i=0}^{m-1} w_i \, a_i}{\sum_{i=0}^{m-1} w_i}$$

    where predictions $a_0, \dots, a_{m-1}$ are ordered **oldest first** ($i = 0$ is the oldest surviving prediction for that period).
- **Preallocated ring**: `PreparedPolicy::from_backend` allocates the chunk buffer (`ChunkRing`) and its fixed-size source provenance table once (zero heap allocations during replans or steps). `chunk_size == 1` allocates zero chunk-ring storage and keeps single-step behavior unchanged.
- **Elapsed-period indexing and commit-time alignment**:
  - The chunk timeline is indexed by elapsed control periods since the runner's interval origin, not by `step_infer` call count. Entries for skipped periods are discarded before taking or ensembling the current period, and replanning triggers when `periods_since_replan >= replan_every` or the buffer is exhausted.
  - Right before commit, the runner recomputes the current control period. If sampling/inference overran into a later period, passed slots are discarded and the action for the current period is submitted instead; if the buffer cannot cover the current period, nothing is submitted on that tick.
- **Source freshness and ring invalidation**:
  - Source-observation freshness is checked when a chunk is inferred, and every chunk retains its fixed-size provenance (`joint_captured_at` + driver feedback age, detector `vision_captured_at` / `vision_generation` / target presence, and video-embedding `InferenceTicket`).
  - A cached action may execute only while its source observation's age is `<= max_policy_input_age_ms(fps) + replan_every * period`. Once past that bound, the stale chunk is dropped (and a replan runs when no fresh prediction covers the current period).
  - The entire chunk ring is cleared on: stop or E-stop, fault, lease change or revocation, epoch change, source vision ticket invalidation (or detector target lost/replaced or model-generation change), and shadow↔live mode switches.
  - Current-tick input fences (`STALE_INPUT`, detector vision staleness, and `InferenceTicket` commit-section verification) continue to apply on every tick.
- **1000 ms open-loop horizon cap**: `replan_every * (1000 / fps)` must be `<= 1000 ms` (`MAX_OPEN_LOOP_HORIZON_MS`). Enforced during `StrictPolicyContract::parse` when `policy.json` specifies `fps`, and again at `/api/v1/action/policy/start` (and runner startup) against the effective `fps`.
