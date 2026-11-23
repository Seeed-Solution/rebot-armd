# 策略运行时（`rebot-onnx-policy/v1`）

[English](policy_runtime.md) | 中文

`armd` 在 `src/policy_runtime/` 中运行关节位置策略。一个策略包是由 `policy.json` 与单个模型文件（`.rmlp`、`.mlp.json` 或 `.onnx`）组成的目录；加载时一次性读入不可变快照（`PolicySnapshot`），并基于 `policy.json` 与模型字节计算确定性 SHA-256 `release_digest`。

## `policy.json` 字段（`rebot-onnx-policy/v1`）

`StrictPolicyContract::parse`（`src/policy_runtime/manifest.rs`）负责解析 `policy.json`（上限 `4 MiB`）。`model_file` 必须为策略目录内的安全相对路径（禁止 `/`、`\`、`..`、盘符、NUL 字节、首尾空白及符号链接逃逸），文件大小上限 `8 MiB + 4 KiB`（`8,392,704` 字节）。

| 字段 | 类型 | 限制 / 取值 | 默认值 / 必填 |
|---|---|---|---|
| `format` | 字符串 | `"rebot-onnx-policy/v1"` | 必填 |
| `name` | 字符串 | 任意字符串 | `""` |
| `robot` | 字符串 \| null | 非空字符串 | `null` |
| `model_file` | 字符串 | 包目录内安全相对路径；大小 `1..=8,392,704` 字节 | `"model.onnx"` |
| `unit_mode` | 字符串 | `"rad"` 或 `"so101_norm"` | 必填 |
| `action_dim` | 整数 | `6` 或 `7` | 必填 |
| `action_space` | 字符串 | `"joint_positions"`（与 `outputs.action.space` 同时出现时必须一致） | 必填（顶层或 `outputs.action.space`） |
| `outputs.action.space` | 字符串 | `"joint_positions"` | 见 `action_space` |
| `outputs.action.shape` | 整数数组 | `[N]` 或 `[1, N]`，其中 `N = action_dim * chunk_size`；拒绝未展平的 `[K, D]` 与 `[1, K, D]` | 可选 |
| `inputs.state.shape` | 整数数组 | `[action_dim]` 或 `[1, action_dim]` | 可选 |
| `joint_names` | 字符串数组 | 长度为 `action_dim` 的唯一非空关节名（与 `inputs.state.joint_names` 同时出现时必须一致） | 必填（顶层或 `inputs.state.joint_names`） |
| `inputs.state.joint_names` | 字符串数组 | 长度为 `action_dim` 的唯一非空关节名 | 见 `joint_names` |
| `normalization.state_mean`、`normalization.state_std`（或 `normalization.state.{mean,std}`） | `f32[action_dim]` 成对数组 | 有限数值；`std` 严格 `> 0.0`；必须成对出现；平铺与嵌套同时存在时数值必须一致 | 可选（恒等） |
| `normalization.action_mean`、`normalization.action_std`（或 `normalization.action.{mean,std}`） | `f32[action_dim]` 成对数组 | 有限数值；`std` 严格 `> 0.0`；必须成对出现；平铺与嵌套同时存在时数值必须一致 | 可选（恒等） |
| `fps` | 数值 \| null | 有限 `f64 > 0.0`；当 `chunk_size > 1` 时满足 `replan_every * (1000 / fps) <= 1000 ms` | `null`（启动时取请求 `fps`，否则 `20.0`） |
| `max_step_rad` | 数值 \| null | 有限 `f32 > 0.0` | `null`（仅受伺服常规限速约束） |
| `max_first_step_rad` | 数值 \| null | 有限 `f32 > 0.0`；实际生效上限为 `min(声明值, 0.25)` | `0.25` rad（`DEFAULT_MAX_FIRST_STEP_RAD`） |
| `vision` | 对象 \| null | 总输入维度 `action_dim + vision_feature_dim <= 1024` | `null` |
| `chunking` | 对象 \| null | 见下方 `chunking.*` 表 | `{chunk_size: 1, replan_every: 1, temporal_ensemble_k: null}` |

### `vision` 字段（`vision.source == "detector"`）

| 字段 | 类型 | 限制 / 取值 | 默认值 / 必填 |
|---|---|---|---|
| `vision.source` | 字符串 | `"detector"` | 必填 |
| `vision.layout` | 字符串 | `"top1_box_norm_v1"`（6 维浮点：`[present, cx, cy, w, h, score]`） | 必填 |
| `vision.target_label` | 字符串 \| null | 非空字符串 | `null`（不限标签） |
| `vision.min_score` | 数值 \| null | `[0.0, 1.0]` 内有限 `f32` | `0.3` |
| `vision.max_staleness_ms` | 整数 | `1..=500` | 必填 |
| `vision.max_sync_skew_ms` | 整数 \| null | `1..=500` | `vision.max_staleness_ms` |
| `vision.on_target_lost` | 字符串 \| null | `"hold"`、`"zero_features"` 或 `"stop"` | `"hold"` |
| `vision.on_stale` | 字符串 \| null | `"stop"` 或 `"hold"` | `"stop"` |
| `vision.require_model_generation` | 布尔 \| null | 布尔值 | `true` |

### `vision` 字段（`vision.source == "video_embedding"`）

| 字段 | 类型 | 限制 / 取值 | 默认值 / 必填 |
|---|---|---|---|
| `vision.source` | 字符串 | `"video_embedding"` | 必填 |
| `vision.layout` | 字符串 | `"embedding_v1"` | 必填 |
| `vision.embedding_dim` | 整数 | `>= 1` | 必填 |
| `vision.history_len` | 整数 | `>= 1`；`action_dim + embedding_dim * history_len <= 1024` | 必填 |
| `vision.max_staleness_ms` | 整数 | `1..=500` | 必填 |
| `vision.max_window_span_ms` | 整数 | `1..=2000`；`history_len > 1` 时必须 `>= max_staleness_ms` | 必填 |
| `vision.fps_staleness_budget_ms` | 整数 \| null | `>= 1` | `clamp(round(2000 / fps), 50, 500)` |
| `vision.encoder_artifact_sha256` | 字符串 | 64 位十六进制 SHA-256 | 必填 |
| `vision.preprocess_config_sha256` | 字符串 | 64 位十六进制 SHA-256 | 必填 |
| `vision.embedding_norm` | 对象 \| null | `{mean, std}`：有限 `f32[embedding_dim]`，`std > 0.0` | `null` |
| `vision.on_stale` | 字符串 \| null | `"stop"` 或 `"hold"` | `"stop"` |

### `chunking` 字段

| 字段 | 类型 | 限制 / 取值 | 默认值 / 必填 |
|---|---|---|---|
| `chunking.chunk_size` | 整数 \| null | `1..=128`；`chunk_size * action_dim <= 1024` | `1` |
| `chunking.replan_every` | 整数 \| null | `1..=chunk_size`；`replan_every * (1000 / fps) <= 1000 ms` | `chunk_size` |
| `chunking.temporal_ensemble_k` | 数值 \| null | `[0.0, 100.0]` 内有限 `f32` | `null`（开环执行，不做时序集成） |

## 推理后端

`load_backend`（`src/policy_runtime/mod.rs`）根据文件魔数与后缀选择后端，并校验 `input_dim == action_dim + vision_feature_dim` 与 `output_dim == action_dim * chunk_size`。

| 后端 | 产物格式 | 启用条件 | 资源预算与约束 |
|---|---|---|---|
| `mlp`（`MlpBackend`） | 二进制 `RMLP\x01`（`.rmlp`）或 JSON（`.mlp.json`，上限 `4 MiB`） | 默认内置；推理过程零堆分配 | `MAX_LAYERS = 16`、`MAX_DIM = 1024`、`MAX_PARAMS = 2,000,000`、`MAX_MACS = 4,000,000`；激活函数 `none`、`relu`、`tanh`、`gelu` |
| `tract-onnx`（`TractBackend`） | `.onnx` | 需启用 Cargo feature `policy-tract` | 静态 `f32` 输入 `[1, input_dim]` → 输出 `[1, action_dim * chunk_size]`；禁止外部权重数据、子图、稀疏张量与模型局部函数；`MAX_MODEL_BYTES = 8 MiB + 4 KiB`、`MAX_ONNX_NODES = 512`、`MAX_ONNX_PARAM_ELEMENTS = 2,000,000`、`MAX_ONNX_ACTIVATION_ELEMENTS = 1,000,000` |

## Shadow 模式与 Live 模式

策略通过 WebSocket `/api/v1/action/policy/start` 启动，`mode` 取 `"shadow"`、`"live_sim"` 或 `"live"`（`"live_gated"` 为 `"live"` 别名）。

| 项目 | `shadow` | `live` / `live_sim` |
|---|---|---|
| 下发伺服目标 | 否（写入 `observation.policy.last_shadow_target`） | 是，经 `validate_and_filter_servo_targets` 校验（软限位、工作空间、碰撞与力矩保护） |
| 允许的 `fps` | `1.0..=50.0` | `10.0..=50.0` |
| 启动预热速度门禁（`p50 > 0.5 * period_us`） | 允许启动（ACK 中标记 `too_slow: true`） | 拒绝启动，返回 422 `POLICY_TOO_SLOW` |
| `STALE_INPUT`、`max_first_step_rad`、`max_step_rad` | 不拦截 shadow 输出 | 每次提交前强制检查 |
| 真机前置门禁（`!policy_live_allowed()`，即非 mock / 非仿真） | 仅需有效租约 | 操作员会话、`i_am_at_the_arm: true`、绑定当前 `artifact_digest` 与 `calibration_digest` 的已验证 `rebot-release-gate/v1`、已确认硬件反馈；`production_release` 级别还需设备准入台账 |

## 输入新鲜度围栏

所有新鲜度检查均在提交实机目标前按 fail-closed 原则执行：

- **本体关节状态（`STALE_INPUT`）**：从驱动采集关节反馈到监督器写锁内提交目标的耗时。上限为 `max_policy_input_age_ms(fps) = clamp(round(2000 / fps), 50, 200)` ms（10 fps 为 200 ms，20 fps 为 100 ms，25 fps 为 80 ms，50 fps 为 50 ms）。超过该上限（或真机上报未知年龄）时立即以 `STALE_INPUT` 结束运行。
- **检测器视觉（`detector`）**：检查捕获时间戳存在性、推理前与提交前的 `vision_age_ms <= vision.max_staleness_ms`、视觉与本体时间差 `|vision_age_ms - prop_age_ms| <= vision.max_sync_skew_ms`、`model_generation` 锁定（`VISION_GENERATION_MISSING` / `VISION_GENERATION_CHANGED`）以及目标丢失策略（`hold`、`zero_features` 或 `stop` / `TARGET_LOST`）。
- **视频嵌入（`video_embedding`）**：检查最新帧年龄不超过 `min(max_staleness_ms, fps_staleness_budget_ms, 500)` ms、窗口跨度不超过 `max_window_span_ms`（`<= 2000` ms）、双边时钟握手有效（`DEFAULT_HANDSHAKE_TTL_MS = 60,000` ms），并在提交临界区持有 `HubCommitGuard` 重新核验 `InferenceTicket`（策略实例、租约代次、生产者 epoch、模型代次、失效代次、时钟握手与窗口截止时间）。

## `max_first_step_rad` 与 `max_step_rad`

- **`max_first_step_rad`**：在 live 模式第一步提交时逐关节与 `measured_positions` 比较。生效上限为 `min(manifest.max_first_step_rad, 0.25)` rad（`DEFAULT_MAX_FIRST_STEP_RAD = 0.25`），manifest 只能收紧，不能放宽超过 0.25 rad；超限以 422 `JUMP_LIMIT` 结束运行。
- **`max_step_rad`**：可选的相邻步跳变上限，在 live 模式后续每步逐关节与上一次提交的目标（`last_live_target`）比较；超限以 422 `STEP_LIMIT` 结束运行。

## `policy_stats` 与预热速度门禁

在 `begin_policy_run` 之前，`PreparedPolicy::warmup` 执行最多 `WARMUP_INFERENCES = 10` 次预热推理（总时长上限 `WARMUP_MAX_TOTAL = 5 s`），当运行超过 1 次时丢弃首样本。启动 ACK（及 `POLICY_TOO_SLOW` 拒绝响应）包含：

- `warmup_infer_us`：`{p50, max, n}`（微秒）
- `too_slow`：`p50 > 0.5 * period_us`（`SPEED_GATE_PERIOD_FRACTION = 0.5`）
- `period_us`：`round(1e6 / fps)`
- `max_fps`：`floor(0.5 * 1e6 / max(p50, 1))`

运行期间及结束后，`ActionStatus::policy_stats` 发布：

- `steps`：已记录的步数
- `overruns`：单步节拍到提交耗时 `step_us > period_us` 的超时次数
- `skipped`：超时步累计跳过的周期数 `floor(step_us / period_us)`
- `period_us`：当前控制周期（微秒）
- `infer_us`：最近 256 步（`POLICY_STATS_RING = 256`，固定环形缓冲、无单步分配）的推理耗时 `{p50, p99, max}`（微秒）

## 动作分块（Action chunking）

`armd` 实现 ACT 风格的动作分块与时序集成（Zhao et al., 2023, *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*）。当 `chunking.chunk_size > 1` 时，模型输出展平的 `[1, chunk_size * action_dim]` 张量，按行优先连续存放未来 `chunk_size` 个控制周期的归一化关节目标。

- **参数**：
  - `chunk_size`（`1..=128`，默认 `1`）：单次推理预测的未来控制周期数。
  - `replan_every`（`1..=chunk_size`，默认 `chunk_size`）：两次重规划之间相隔的控制周期数。
  - `temporal_ensemble_k`（`0.0..=100.0`，默认 `null`）：为 `null` 时每次重规划覆盖环形缓冲并开环执行最新块；设为 `k` 时对指向同一控制周期的重叠预测按指数权重加权平均：

    $$w_i = \exp(-k \cdot i), \quad a = \frac{\sum_{i=0}^{m-1} w_i \, a_i}{\sum_{i=0}^{m-1} w_i}$$

    其中预测序列 $a_0, \dots, a_{m-1}$ 按**最旧优先**排列（$i = 0$ 为该周期最早且仍有效的预测）。
- **预分配环形缓冲**：`PreparedPolicy::from_backend` 一次性分配 `ChunkRing` 及其固定容量的来源元数据表（重规划与单步执行均无堆分配）。`chunk_size == 1` 时不分配块缓冲，保持单步路径行为不变。
- **按已过控制周期索引与提交前重算**：
  - 块时间线按相对定时器原点的已过控制周期计数，而非按 `step_infer` 调用次数递增。消费当前周期前会先丢弃被跳过周期对应的条目；当距离上次重规划已过的周期数 `>= replan_every` 或缓冲耗尽时触发重规划。
  - 提交前在锁内重新计算当前控制周期；若采样或推理超时跨入后续周期，会丢弃已过期槽位并提交当前周期对应的动作；若缓冲无法覆盖当前周期，则本拍不提交目标。
- **来源观测新鲜度与清空规则**：
  - 推理新块时照常检查来源观测新鲜度；每个块在固定存储中保存来源身份（关节采样时间与驱动延迟、检测器帧时间/模型代次/目标存在状态、视频嵌入 `InferenceTicket`）。
  - 缓存动作仅在其来源观测年龄 `<= max_policy_input_age_ms(fps) + replan_every * period` 时可执行；超过该上限则丢弃过期块并重新规划。
  - 发生以下任一事件时立即清空整个块环形缓冲：停止 / 急停（E-stop）、故障（fault）、租约变更或失效、epoch 变化、来源视觉票据失效（或检测目标丢失/替换、模型代次变更），以及 shadow ↔ live 模式切换。
  - 当前拍的常规输入围栏（`STALE_INPUT`、视觉新鲜度与 `InferenceTicket` 临界区校验）在每拍继续生效。
- **1000 ms 开环时域上限**：要求 `replan_every * (1000 / fps) <= 1000 ms`（`MAX_OPEN_LOOP_HORIZON_MS`）。若 `policy.json` 声明了 `fps`，在 `StrictPolicyContract::parse` 时校验；在 `/api/v1/action/policy/start` 及 runner 启动时按生效 `fps` 再次校验并拒绝超限配置。
