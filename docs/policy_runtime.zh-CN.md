# 策略运行时（`rebot-onnx-policy/v1`）

[English](policy_runtime.md) | 中文

`armd` 在 `src/policy_runtime/` 中运行关节位置策略。一个策略包是由 `policy.json` 与单个模型文件（`.rmlp`、`.mlp.json` 或 `.onnx`）组成的目录；加载时一次性读入不可变快照（`PolicySnapshot`），并计算确定性 SHA-256 `release_digest`。摘要按顺序对 `"policy.json"` 和规范化后的 POSIX 相对模型路径两条记录分帧哈希，每条记录格式为 `<路径>\0<十进制字节长度>\0<原始字节>`。

## `policy.json` 字段（`rebot-onnx-policy/v1`）

文件大小上限在快照加载阶段（`PolicySnapshot::load_from_dir`，`src/policy_runtime/manifest.rs`）强制检查：`policy.json` 必须为非空常规文件且不超过 `4 MiB`（`4,194,304` 字节），`model_file` 必须为非空常规文件且不超过 `8 MiB + 4 KiB`（`8,392,704` 字节）。`model_file` 允许指向策略包内的嵌套相对子路径（例如 `weights/model.rmlp`），但禁止以 `/` 或 `\` 开头、包含 `..`、盘符、NUL 字节、首尾空白或通过符号链接逃逸包目录。`StrictPolicyContract::parse` 校验下表字段。

| 字段 | 类型 | 限制 / 取值 | 默认值 / 必填 |
|---|---|---|---|
| `format` | 字符串 | `"rebot-onnx-policy/v1"` | 必填 |
| `name` | 字符串 | 任意字符串 | `""` |
| `robot` | 字符串 \| null | 非空字符串 | `null` |
| `model_file` | 字符串 | 包目录内安全相对路径（允许嵌套子目录）；快照加载时检查大小 `1..=8,392,704` 字节 | `"model.onnx"` |
| `unit_mode` | 字符串 | `"rad"` 或 `"so101_norm"` | 必填 |
| `action_dim` | 整数 | `6` 或 `7` | 必填 |
| `action_space` | 字符串 | `"joint_positions"`（与 `outputs.action.space` 同时出现时必须一致；拒绝 `null`） | 必填（顶层或 `outputs.action.space`） |
| `outputs.action.space` | 字符串 | `"joint_positions"`（拒绝 `null`） | 见 `action_space` |
| `outputs.action.shape` | 整数数组 | `[N]` 或 `[1, N]`，其中 `N = action_dim * chunk_size`；拒绝未展平的 `[K, D]` 与 `[1, K, D]` | 可选 |
| `inputs.state.shape` | 整数数组 | `[action_dim]` 或 `[1, action_dim]` | 可选 |
| `joint_names` | 字符串数组 \| null | 长度为 `action_dim` 的唯一非空关节名（两者均非 null 时必须一致）；`null` 视为缺省 | 必填（顶层与 `inputs.state.joint_names` 至少提供一个非 null 列表） |
| `inputs.state.joint_names` | 字符串数组 \| null | 长度为 `action_dim` 的唯一非空关节名；`null` 视为缺省 | 见 `joint_names` |
| `normalization` | 对象 \| null | 可选容器（`null` 视为缺省；省略时不做归一化） | 可选（`null`） |
| `normalization.state_mean`、`normalization.state_std`（或 `normalization.state.{mean,std}`） | `f32[action_dim]` 成对数组 \| null | 有限数值；`std` 严格 `> 0.0`；每对必须同时非 null 或同时 null/缺省；嵌套对象 `normalization.state` 可为 `null`；平铺与嵌套同时存在时数值必须一致 | 可选（恒等） |
| `normalization.action_mean`、`normalization.action_std`（或 `normalization.action.{mean,std}`） | `f32[action_dim]` 成对数组 \| null | 有限数值；`std` 严格 `> 0.0`；每对必须同时非 null 或同时 null/缺省；嵌套对象 `normalization.action` 可为 `null`；平铺与嵌套同时存在时数值必须一致 | 可选（恒等） |
| `fps` | 数值 \| null | 有限 `f64 > 0.0`；当 `chunk_size > 1` 时满足 `replan_every * (1000 / fps) <= 1000 ms` | `null`（运行时优先级：启动请求 `fps` → manifest `fps` → `20.0`） |
| `max_step_rad` | 数值 \| null | 有限 `f32 > 0.0` | `null`（解析时不设值；仅受伺服常规限速约束） |
| `max_first_step_rad` | 数值 \| null | 有限 `f32 > 0.0` | `null`（解析时不设值；运行时默认 `0.25` rad，声明值钳位为 `min(声明值, 0.25)`） |
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
| `vision.fps_staleness_budget_ms` | 整数 \| null | `>= 1` | `null`（解析时不设值；运行时默认 `clamp(round(2000 / runner_fps), 50, 500)` ms） |
| `vision.encoder_artifact_sha256` | 字符串 | 64 位十六进制 SHA-256 | 必填 |
| `vision.preprocess_config_sha256` | 字符串 | 64 位十六进制 SHA-256 | 必填 |
| `vision.embedding_norm` | 对象 \| null | `{mean, std}`：有限 `f32[embedding_dim]`，`std > 0.0` | `null` |
| `vision.on_stale` | 字符串 \| null | `"stop"` 或 `"hold"` | `"stop"` |

### `chunking` 字段

| 字段 | 类型 | 限制 / 取值 | 默认值 / 必填 |
|---|---|---|---|
| `chunking.chunk_size` | 整数 \| null | `1..=128`；`chunk_size * action_dim <= 1024` | `1` |
| `chunking.replan_every` | 整数 \| null | `1..=chunk_size`；`chunk_size > 1` 时 `replan_every * (1000 / fps) <= 1000 ms`（`chunk_size == 1` 的单步策略不受此上限限制） | `chunk_size` |
| `chunking.temporal_ensemble_k` | 数值 \| null | `[0.0, 100.0]` 内有限 `f32` | `null`（开环执行，不做时序集成） |

## 推理后端

`load_backend`（`src/policy_runtime/mod.rs`）根据文件魔数与后缀选择后端，并校验 `input_dim == action_dim + vision_feature_dim` 与 `output_dim == action_dim * chunk_size`。

| 后端 | 产物格式 | 启用条件 | 资源预算与约束 |
|---|---|---|---|
| `mlp`（`MlpBackend`） | 二进制 `RMLP\x01`（`.rmlp`）或 JSON（`.mlp.json`，上限 `4 MiB`） | 默认内置；`MlpBackend::infer` 内部零堆分配 | `MAX_LAYERS = 16`、`MAX_DIM = 1024`、`MAX_PARAMS = 2,000,000`、`MAX_MACS = 4,000,000`；激活函数 `none`、`relu`、`tanh`、`gelu` |
| `tract-onnx`（`TractBackend`） | `.onnx` | 需启用 Cargo feature `policy-tract` | 静态 `f32` 输入 `[1, input_dim]` → 输出 `[1, action_dim * chunk_size]`；禁止外部权重数据、子图、稀疏张量与模型局部函数；`MAX_MODEL_BYTES = 8 MiB + 4 KiB`、`MAX_ONNX_NODES = 512`、`MAX_ONNX_PARAM_ELEMENTS = 2,000,000`、`MAX_ONNX_ACTIVATION_ELEMENTS = 1,000,000` |

## Shadow 模式与 Live 模式

策略通过 WebSocket `/api/v1/action/policy/start` 启动，`mode` 取 `"shadow"`、`"live_sim"` 或 `"live"`（`"live_gated"` 为 `"live"` 别名）。生效帧率优先级为：启动请求 `fps` → manifest `fps` → `DEFAULT_POLICY_FPS = 20.0`。

| 项目 | `shadow` | `live` / `live_sim` |
|---|---|---|
| 下发伺服目标 | 否（写入 `observation.policy.last_shadow_target`） | 是，经 `validate_and_filter_servo_targets` 校验（软限位、工作空间、碰撞与力矩保护） |
| 允许的 `fps` | `1.0..=50.0` | `10.0..=50.0` |
| 启动预热速度门禁（`p50 > 0.5 * period_us`） | 允许启动（ACK 中标记 `too_slow: true`） | 拒绝启动，返回 422 `POLICY_TOO_SLOW` |
| `STALE_INPUT`、`max_first_step_rad`、`max_step_rad` | 不拦截 shadow 输出 | 每次提交前强制检查 |
| 真机鉴权与前置门禁（`!policy_live_allowed()`，即非 mock / 非仿真） | 有效租约 + 操作员会话（真机上任何模式调用 `/api/v1/action/policy/start` 均需操作员授权） | 有效租约 + 操作员会话 + `i_am_at_the_arm: true`、绑定当前 `artifact_digest` 与 `calibration_digest` 的已验证 `rebot-release-gate/v1`、已确认硬件反馈；`production_release` 级别还需设备准入台账 |

## 输入新鲜度围栏

所有新鲜度检查均在提交实机目标前按 fail-closed 原则执行：

- **本体关节状态（`FEEDBACK_UNCONFIRMED` / `STALE_INPUT`）**：真机（`!policy_live_allowed()`）上若驱动反馈未确认或缺失反馈年龄（`policy_input_age_range_ms().is_none()`），在 `STALE_INPUT` 检查之前即以 409 `FEEDBACK_UNCONFIRMED` 结束运行。反馈已确认时，采集到提交的耗时上限为 `max_policy_input_age_ms(fps) = clamp(round(2000 / fps), 50, 200)` ms（10 fps 为 200 ms，20 fps 为 100 ms，25 fps 为 80 ms，50 fps 为 50 ms）；超限（或真机提交未知年龄）时以 409 `STALE_INPUT` 结束运行。
- **检测器视觉（`detector`）**：检查捕获时间戳存在性、推理前与提交前的 `vision_age_ms <= vision.max_staleness_ms`、视觉与本体时间差 `|vision_age_ms - prop_age_ms| <= vision.max_sync_skew_ms`、`model_generation` 锁定（`VISION_GENERATION_MISSING` / `VISION_GENERATION_CHANGED`）以及目标丢失策略（`hold`、`zero_features` 或 `stop` / `TARGET_LOST`）。
- **视频嵌入（`video_embedding`）**：检查最新帧年龄不超过 `min(max_staleness_ms, effective_fps_budget_ms, 500)` ms（`fps_staleness_budget_ms` 未配置时运行时默认 `clamp(round(2000 / runner_fps), 50, 500)` ms）、窗口跨度不超过 `max_window_span_ms`（`<= 2000` ms）、双边时钟握手有效（`DEFAULT_HANDSHAKE_TTL_MS = 60,000` ms），并在提交临界区持有 `HubCommitGuard` 重新核验 `InferenceTicket`（策略实例、租约代次、生产者 epoch、模型代次、失效代次、时钟握手与窗口截止时间）。

## `max_first_step_rad` 与 `max_step_rad`

- **`max_first_step_rad`**：省略或为 `null` 时解析阶段保持未设置（`None`）；在 `begin_policy_run` 阶段运行时默认取 `DEFAULT_MAX_FIRST_STEP_RAD = 0.25` rad，声明值钳位至 `min(声明值, 0.25)` rad。在 live 模式第一步提交时逐关节与 `measured_positions` 比较，超限以 422 `JUMP_LIMIT` 结束运行。
- **`max_step_rad`**：可选的相邻步跳变上限，在 live 模式后续每步逐关节与上一次提交的目标（`last_live_target`）比较；超限以 422 `STEP_LIMIT` 结束运行。

## `policy_stats` 与预热速度门禁

在 `begin_policy_run` 之前，`PreparedPolicy::warmup` 执行最多 `WARMUP_INFERENCES = 10` 次预热推理，在某次推理完成后若总耗时达到或超过 `WARMUP_MAX_TOTAL = 5 s` 即停止（单次推理不可中断，因此总预热耗时最多可超过 5 s 一次推理的时长）。运行超过 1 次时丢弃首样本。启动 ACK（及 `POLICY_TOO_SLOW` 拒绝响应）包含：

- `warmup_infer_us`：`{p50, max, n}`（微秒）
- `too_slow`：`p50 > 0.5 * period_us`（`SPEED_GATE_PERIOD_FRACTION = 0.5`）
- `period_us`：`round(1e6 / fps)`
- `max_fps`：`floor(0.5 * 1e6 / max(p50, 1))`

运行期间及结束后，`ActionStatus::policy_stats` 在监督器写锁内、**`commit_gate` 与 `policy_step_with_vision` 提交之前**由 `record_policy_step_timing` 记录：

- `steps`：已记录的推理/节拍周期数（提交前记录，包含后续被 `commit_gate` 跳过或提交被拒的周期）
- `overruns`：节拍开始到记录耗时 `step_us > period_us` 的超时次数
- `skipped`：超时周期累计跳过的周期数 `floor(step_us / period_us)`
- `period_us`：当前控制周期（微秒）
- `infer_us`：最近 256 步（`POLICY_STATS_RING = 256`，固定环形缓冲、无单步分配）的推理耗时 `{p50, p99, max}`（微秒）

## 动作分块（Action chunking）

`armd` 实现 ACT 风格的动作分块与时序集成（Zhao et al., 2023, *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*）。当 `chunking.chunk_size > 1` 时，模型输出展平的 `[1, chunk_size * action_dim]` 张量，按行优先连续存放未来 `chunk_size` 个控制周期的归一化关节目标（省略 `normalization` 时为原始单位目标）。

- **参数**：
  - `chunk_size`（`1..=128`，默认 `1`）：单次推理预测的未来控制周期数。
  - `replan_every`（`1..=chunk_size`，默认 `chunk_size`）：两次重规划之间相隔的控制周期数。
  - `temporal_ensemble_k`（`0.0..=100.0`，`f32` 有限，默认 `null`）：为 `null` 时每次重规划覆盖环形缓冲并开环执行最新块；设为 `k` 时对指向同一控制周期的重叠预测按指数权重加权平均：

    $$w_i = \exp(-k \cdot i), \quad a = \frac{\sum_{i=0}^{m-1} w_i \, a_i}{\sum_{i=0}^{m-1} w_i}$$

    其中预测序列 $a_0, \dots, a_{m-1}$ 按**最旧优先**排列（$i = 0$ 为该周期最早且仍有效的预测）。
- **预分配环形缓冲与 runner 线程分配**：`PreparedPolicy::from_backend` 一次性分配 `ChunkRing` 及其固定容量的来源元数据表（`chunk_size` 槽 × `1` 或 `chunk_size` 条重叠预测 × `action_dim` 浮点；`chunk_size <= 1` 时分配 0 字节），写入块、剪枝与槽位选择均不扩容环形缓冲。单位转换与目标构造（`rad_to_policy_units`、`policy_units_to_rad` 以及克隆传给 `policy_step_with_vision` 的 `Vec<f32>`）仍在 runner 线程上分配长度为 `action_dim` 的短向量。
- **两阶段执行与提交前唯一门禁 `commit_gate(now)`**：
  - 锁外规划阶段（`plan_chunk_step`）：清理过期/失效条目，将环形缓冲推进到当前采样周期；当缓冲为空或 `sample_period - last_replan_period >= replan_every` 时运行后端推理并写入新块，同时预转换当前采样周期的候选目标（不弹出槽位）。
  - 监督器写锁内（视频嵌入模式下同时持有 `HubCommitGuard`），在发布门禁/部署检查及当前拍 `InferenceTicket` 校验之后，将 `commit_gate_now` 作为**调用 `policy_step_with_vision` 前的最后一步**执行，读取一次单调时钟（`now`）：
    1. 按该次读数计算 `commit_period` 并选择对应槽位（丢弃已过槽位）；若该周期已提交/已过期或缓冲未能覆盖该周期，则本拍不提交目标（未覆盖时清空缓冲）。
    2. 按该次 `now` 校验所选槽位每一条贡献预测的来源年龄：本拍新推理的预测使用常规上限（`max_policy_input_age_ms(fps)`、`vision.max_staleness_ms`、嵌入票据原始截止时间 `horizon_ms = 0`）；早期缓存预测使用 `max_policy_input_age_ms(fps) + replan_every * period`（视觉为 `vision.max_staleness_ms + replan_every * period`，嵌入票据截止时间 `+ replan_every * period`）。若超期，则清空整个块环形缓冲并在本拍跳过提交（不终止运行），下一周期强制重新规划。
    3. 校验每条贡献预测的来源身份：检测器 `model_generation`、检测器目标身份 `DetectorTargetIdentity`（提交前重新读取最新帧：检测器提供 track id 时为 `TrackId`，否则为标签 128 位 SHA-256 摘要 + 量化到 `0..=1000` 的归一化框 + `frame_id`，无目标为 `Absent`）以及视频嵌入 `InferenceTicket` 的身份/失效代次/截止时间；任一项不匹配（包括目标保持存在但顶层目标被替换为另一目标）即清空整个块环形缓冲并跳过本拍提交。
  - `commit_gate_now` 决定之后到 `policy_step_with_vision` 之间不再做任何额外读时钟或新鲜度/周期检查（`input_age_ms` 与检测器 `post_vision_age_ms` 直接由门禁的 `now` 计算）。
- **整环清空触发条件**：除 `commit_gate` 失配/超期外，发生停止或急停（E-stop）、故障（fault）、租约变更或失效、epoch 变化、shadow ↔ live 模式切换（`sync_run_context`）、视觉 `Hold`/`Stop` 决策以及 `policy_step_with_vision` 返回错误时，均立即清空整个块环形缓冲。
- **1000 ms 开环时域上限**：`chunk_size > 1` 时要求 `replan_every * (1000 / fps) <= 1000 ms`（`MAX_OPEN_LOOP_HORIZON_MS`）。若 `policy.json` 声明了 `fps`，在 `StrictPolicyContract::parse` 时校验；在 `/api/v1/action/policy/start` 及 `spawn_policy_runner` 启动时按生效 `fps` 再次校验并拒绝超限配置。单步策略（`chunk_size == 1`）不受此上限限制。
