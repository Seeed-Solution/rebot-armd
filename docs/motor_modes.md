# Motor Control Modes

English | [中文](motor_modes.zh-CN.md)

`armd` models actuator control by **command semantics** (`ControlMode`) rather than motor model or joint index. Vendor codecs, mode-selection handshakes, and feedback capabilities bind to those semantics at profile load before any transport opens.

> **Step 1 scope (current branch):** `ControlMode`, typed per-mode limits, the profile schema and load-time validation, omitted-field translation, the vendor capability table, and the pure Damiao codec (`encode_command` / `decode_feedback_bytes`) are implemented and tested. Driver TX, enable, disable, hold, and emitter paths in `src/driver.rs` are not switched yet (scheduled for Steps 2–5 below).
>
> **Step 1 hardware startup guard:** Because `src/driver.rs` still reads legacy profile sections (`damiao_gripper`, `damiao_ranges`, `feetech`, and vendor defaults), `armd` startup runs `ArmConfig::validate_step1_driver_parity()` before opening any hardware transport. If an explicit `control_mode` declaration diverges from what the unmigrated Step-1 driver would actually apply (for example, removing `damiao_gripper` and putting a J7 current ceiling only in `control_mode`), startup fails closed instead of silently running without the declared mode or limit. Keep legacy profile fields matching explicit `control_mode` entries (or omit `control_mode`) until Steps 2–5 migrate each driver.

---

## 1. Semantic Control-Mode Model

Defined in `src/motor_mode.rs` (compiled on native targets and `wasm32-unknown-unknown`) and re-exported from `src/model.rs` and `src/lib.rs`.

| `ControlMode` (JSON) | Semantics | Typed Limits Struct | Required Mode-Specific Limits |
|---|---|---|---|
| `position` | Position setpoint | `PositionLimits` | None required (optional `max_velocity_rad_s`, `step_range` in `0..=4095`, `interval_ms >= 1`, `power_limit >= 1`) |
| `position_velocity` | Position setpoint + per-frame velocity ceiling | `PositionVelocityLimits` | `max_velocity_rad_s` (positive finite; within vendor wire `v_max`) |
| `impedance` | Position + velocity + `kp` + `kd` + feedforward torque (MIT-style) | `ImpedanceLimits` | `p_max`, `v_max`, `t_max` (positive finite symmetric wire ranges), `kp_max` (`0..=500`), `kd_max` (`0..=5`), optional `effort_limit_nm` (`(0, t_max]`) |
| `position_force` | Position setpoint + per-frame speed ceiling + motor current ceiling | `PositionForceLimits` | `current_ceiling_raw` (`1..=10000`), `close_speed_rad_s` (quantizes to `>= 0.01 rad/s`, `<= pos_force_max_wire_speed_f32()` i.e. `wire_speed_f32(65535)`), optional `max_speed_rad_s`, `calibrated` / `ceiling_torque_nm` / `torque_reference`, `feedback_stale_fault_ms` (`200..=450`) |

### Profile Schema (`ArmConfig::control_mode`)

Profiles may declare an optional top-level `control_mode` array of length `dof`. Each element is either `null` (translate from existing follower and profile settings) or an object:

```json
{
  "control_mode": [
    {
      "mode": "position_velocity",
      "vendor": "damiao",
      "runtime_modes": ["impedance"],
      "limits": { "max_velocity_rad_s": 2.0 }
    },
    null,
    null,
    null,
    null,
    null,
    {
      "mode": "position_force",
      "vendor": "damiao",
      "runtime_modes": [],
      "limits": {
        "current_ceiling_raw": 460,
        "close_speed_rad_s": 0.3,
        "calibrated": false
      }
    }
  ]
}
```

- `limits` is parsed strictly for the declared `mode` (`deny_unknown_fields`); omitting `limits` is accepted only for `position`.
- `vendor` is optional; when present, it is checked against the active follower binding and the vendor capability table.
- `runtime_modes` is optional: omitting it inherits runtime transitions from `gravity_comp` / `joint_tracking`, while `"runtime_modes": []` explicitly forbids runtime switching on that joint.

### Load-Time Validation (Before Transport Open)

`ArmConfig::validate()` and `ArmConfig::validate_step1_driver_parity()` run in `src/main.rs` (`resolve_follower_config`) before binding sockets or opening CAN/serial transports, rejecting:

- length mismatch (`control_mode.len() != dof`) or duplicate JSON keys inside `control_mode` / `limits`;
- missing mode-specific limits or limits whose variant does not match `mode`;
- unsupported `(vendor, mode)`, joint-role mismatches (Damiao `position_force` is gripper-only; Damiao `impedance` / runtime MIT transitions are arm-joint-only), or unsupported `(vendor, base_mode -> runtime_mode)` bindings;
- conflicts between `control_mode` and legacy profile blocks (`damiao_gripper`, `damiao_ranges`, `feetech`, `gravity_comp` effective effort/gain ceilings, enabled MIT `joint_tracking`), conflicting `vendor` declarations across joints / follower, or explicit impedance wire mappings disagreeing with commissioned/default driver mappings;
- Step-1 hardware parity drift (explicit `control_mode` diverging from what the unmigrated driver path would apply);
- invalid wire ranges (non-finite or quantization-overflowing endpoints, joint position limits exceeding `[-p_max, p_max]` or FashionStar `i16` tenth-degree span, `kp_max > 500`, `kd_max > 5`, `current_ceiling_raw` outside `1..=10000`, `POS_FORCE` speed quantizing to `0` or exceeding `pos_force_max_wire_speed_f32()`, 12-bit `step_range` outside `0..=4095`, cross-vendor fields such as Feetech `interval_ms`/`power_limit` or FashionStar `step_range`).

### Omitted-Field Translation and `damiao_gripper` Compatibility

When `control_mode` (or an individual entry) is omitted, resolution reproduces existing behaviour across all shipped profiles (`kits/*/profile/*.json`, `profiles/*.json`):

| Follower / Profile | Arm Joints | Gripper Joint | Runtime Transitions |
|---|---|---|---|
| Damiao (`rebot-dm`, B601-DM with `damiao_gripper.mode = pos_force`) | `position_velocity` (`2.0 rad/s`) | `position_force` (from `damiao_gripper`: raw `460`, `0.30 rad/s`) | `impedance` on arm joints when `gravity_comp` or enabled MIT `joint_tracking` applies |
| Damiao (legacy B601-DM: `damiao_gripper` absent or `mode = pos_vel`) | `position_velocity` (`2.0 rad/s`) | `position_velocity` (`2.0 rad/s`) | `impedance` on arm joints when `gravity_comp` or enabled MIT `joint_tracking` applies |
| Feetech (`so-arm100`, `so-arm101`) | `position` (step range from `feetech.joints[i]`) | `position` (step range from `feetech.joints[i]`) | none |
| FashionStar (`fashionstar`, `star-arm102`) | `position` (`interval_ms = 20`, `power_limit = 1000`) | none | none |
| RobStride (`rebot-rs`, B601-RS) | `impedance` (`±4π rad`, `±30 rad/s`, `±18 N·m`, `kp <= 500`, `kd <= 5`) | none | none |

`ArmConfig::damiao_gripper` remains supported as the compatibility reader for existing profiles; `ArmConfig::effective_damiao_gripper()` also exposes `PositionForce` gripper settings declared via `control_mode`.

### Pure Damiao Codec (Step 1 Extraction)

`src/protocol/damiao.rs` provides pure, zero-allocation (`Copy`) wire codecs without changing the driver send paths or weakening the compile-time `PosForceSendToken` seal on `build_pos_force_frame`:

- `encode_command(motor_id, mode, payload, limits) -> Result<DamiaoEncodedFrame, DamiaoEncodeRefusal>` encodes `PositionVelocity` (`0x100 + id`), `Impedance` (`id`), and `PositionForce` (`0x300 + id`) into `{ can_id: u16, data: [u8; 8] }`, rejecting `ControlMode::Position`, mode/limit mismatches, invalid CAN IDs, non-finite inputs, invalid limits, zero wire speed on moving targets, zero current limits, and non-zero hold velocities.
- `decode_feedback_bytes(data, limits) -> Result<DamiaoFeedback, DamiaoError>` decodes 8-byte Damiao feedback payloads identically to `unpack_feedback_frame`.

---

## 2. Vendor Capability Table

Derived from `src/protocol/*.rs` and `src/driver.rs` (`VENDOR_CAPABILITIES` in `src/motor_mode.rs`). Items not verified in the repository are explicitly marked **needs vendor-doc check**.

| Vendor | Source | Supported `ControlMode`s in `armd` | Default Base Mode | Mode Selection & Readback | Runtime Switch | Feedback Source & Runtime Fields | Needs Vendor-Doc Check |
|---|---|---|---|---|---|---|---|
| **Damiao** | `src/protocol/damiao.rs` | `position_velocity` (`CTRL_MODE = 2`), `impedance` (`CTRL_MODE = 1`), `position_force` (`CTRL_MODE = 4`, gripper joint only) | `position_velocity` | RAM write (`0x7FF`, `0x55`, `RID_CTRL_MODE = 10`) + matching ACK + separate readback (`0x33`). J7 `POS_FORCE` enable requires fresh (`<= 100 ms`), stationary (`|v| <= 0.05 rad/s`), state-0 feedback before mode write, readback, and pre-enable zero-velocity hold (`docs/gripper_pos_force.md`). | `position_velocity <-> impedance` on arm joints | Command reply + read-only `0xCC` refresh (`0x7FF`). Unpacks `position`, `velocity`, `torque`, `t_mos`, `t_rotor`, `state_or_error`. General freshness bound `250 ms`; `position_force` requires `<= 100 ms`. | `CTRL_MODE = 3` (VEL) frame layout and semantics; per-motor-model supported `CTRL_MODE` matrix; feedback reply cadence and guarantees across modes/firmware; `RID_TIMEOUT` (reg 9) unit (`50 us` documented), expiry latency, and motor response on B601-DM firmware (`sw_ver 942747703`). |
| **Feetech STS3215** | `src/protocol/feetech.rs` | `position` (`INST_SYNC_WRITE` `0x83` to `REG_GOAL_POSITION` `0x2A`) | `position` | Provisioning reads `REG_OPERATING_MODE` (`0x21`), writes `0` when different (torque off, `REG_LOCK = 0x37` unlocked), and verifies changed registers via readback. | No | Bounded chunked `INST_SYNC_READ` (`0x82`, 8 bytes from `0x38`): `position`, `speed`, `load`, `voltage`, `temperature` (load-derived torque is an estimate, not measured torque); separate `REG_TORQUE_ENABLE` (`0x28`) readback. | `REG_OPERATING_MODE` (`0x21`) values and command semantics for wheel/speed and PWM modes; `REG_PRESENT_CURRENT_L` (`0x45`) scaling and latency (defined in protocol, outside the 8-byte sync-read poll). |
| **RobStride** | `src/protocol/robstride.rs` | `impedance` (`TYPE_MOTION_CONTROL = 1` 29-bit extended CAN frame: 16-bit `pos`, `vel`, `kp`, `kd` in payload, 16-bit `torque` in CAN ID bits 23..8) | `impedance` | Fixed motion-control frame type (`TYPE_ENABLE = 3`, `TYPE_STOP = 4`). `TYPE_PARAM_READ = 17` and `TYPE_PARAM_WRITE = 18` constants declared. | No | Drains `TYPE_FEEDBACK = 2` 29-bit extended CAN frames: `position`, `velocity`, `torque`, `mos_temperature` (`0.1 °C`), `fault_flags`. | Position, speed, and current operating modes, selection registers, and readback via `TYPE_PARAM_READ` / `TYPE_PARAM_WRITE`; Type-2 feedback reply pairing, periodic reporting cadence, and current telemetry; whether `TYPE_ENABLE` / `TYPE_STOP` yield causal torque-state evidence; `TYPE_SET_ZERO` (6) persistence and parameter-save sequence (type 22). |
| **FashionStar** | `src/protocol/fashionstar.rs` | `position` (`CMD_SET_SERVO_ANGLE = 0x08`: `0.1 deg` `i16`, `interval_ms = 20`, `power = 1000`; separate `CMD_SET_TORQUE = 0x1A`) | `position` | Implicit angle command. | No | Round-robin explicit `CMD_QUERY_SERVO_ANGLE` (`0x0A`) updating `position` only. Protocol also defines `CMD_QUERY_STATUS` (`0x1E`) decoder (voltage, temp, current, status), not polled in the runtime loop. | Mode selection/readback and any non-position operating modes; `CMD_SET_SERVO_ANGLE` (`0x08`) ACK semantics and `power = 0` meaning; velocity and torque telemetry availability. |

---

## 3. Migration Plan (Steps 2–5)

Each migration step lands as small, behaviour-preserving PRs. Every PR must satisfy:

- `test_ws_replay` with **0 diffs** against the recorded baseline corpus;
- byte-level and frame-order parity for every migrated wire path;
- zero heap allocations and zero `SupervisorLock` attempts on `armd-control` (enforced by `test_rt_*_alloc` and `test_rt_lock_proof`);
- any change to stop-priority scheduling or torque-evidence semantics (`OffConfirmed` / `OnConfirmed` / `Unknown` and single `send(DisableRequested)` choke point) is reviewed in a separate PR from parity refactors.

### Step 2: Damiao Driver Unification (`POS_VEL`, `MIT`, `POS_FORCE`)

- Generalize the J7 tick-epilogue emitter pattern (`pos_force_emitter`) to a fixed-capacity per-tick emitter covering all joints on both owners (Legacy and RT Executor): owner stages per-joint intents (`Target`, `Hold`, `Stop`, `Freeze` only where qualified, no-evidence -> emit nothing and report unheld), emitter encodes via `protocol::damiao::encode_command`.
- Schedule load-bearing `Stop` / `Hold` frames ahead of `0xCC` refresh and gripper work while preserving handshake ordering (`CTRL_MODE` write -> ACK -> readback -> pre-enable hold -> enable).
- Recheck deadline, TX budget, stop generation, verified mode, and feedback freshness at transmit; discard superseded intents.
- Track desired vs verified mode separately (unverified switch blocks target output); keep torque evidence (`OffConfirmed` / `OnConfirmed` / `Unknown`) separate from mode.
- **Removed special-case code:** duplicate per-path `POS_VEL` / `MIT` target and hold frame builders and gripper-specific runtime mode dispatch in `src/driver.rs`.
- **Retained:** `damiao_gripper` compatibility reader, `PosForce` 100 ms freshness / boot-fallback rules, and mode-transition readback checks.

### Step 3: Feetech STS3215 Migration

- Route `FeetechArmDriver` target and hold output through the unified per-tick emitter, preserving single-packet `INST_SYNC_WRITE` serial batching in caller-owned fixed buffers (zero allocation).
- **Removed special-case code:** independent position-write assembly paths in `FeetechArmDriver`.
- **Retained:** `configure_servos` provisioning (`REG_OPERATING_MODE = 0x21` read/write/verify under `REG_LOCK`), serial TX-echo handling, and chunked `SYNC_READ` polling.

### Step 4: RobStride Migration

- Replace the direct Type-1 `write_joint_targets` loop in `RobStrideArmDriver` with the per-tick emitter using `ControlMode::Impedance` and `ImpedanceLimits`.
- **Removed special-case code:** direct per-joint Type-1 target loop.
- **Retained:** `src/protocol/robstride.rs` extended CAN codec and Type-2 feedback drain.

### Step 5: FashionStar Migration

- Replace the direct `CMD_SET_SERVO_ANGLE` loop in `FashionStarArmDriver` with the per-tick emitter and move packet encoding in `src/protocol/fashionstar.rs` to caller-provided fixed buffers (removing per-packet `Vec` allocations).
- **Removed special-case code:** direct angle-write loop and allocating packet builders on the hot path.
- **Retained:** request-echo filtering (`[0x12, 0x4C]` vs `[0x05, 0x1C]`) and round-robin angle query matching.

---

## 4. Verification Matrix

### Mock / Simulation (CI)

- Exercise both Legacy and RT Executor owners with stale/missing RX, mismatched mode readback, STOP during frame build, partial TX backpressure, and one unavailable joint beside healthy load-bearing joints.
- Assert exact wire bytes, frame ordering, torque/mode evidence transitions, deadlines, zero heap allocations, and zero `SupervisorLock` attempts on `armd-control`.

### Hardware Bench Verification

| Target Hardware | Required Bench Checks |
|---|---|
| **B601-DM (office arm)** | Capture CAN frames and motion across `POS_VEL`, `MIT`, and `POS_FORCE`; verify loaded holds under CAN congestion; verify J7 `0xCC` refresh / freeze and external current-to-jaw-force calibration (current stall table in `docs/gripper_pos_force.md` uses motor feedback only). |
| **SO-ARM101 (Feetech STS3215)** | Verify `REG_OPERATING_MODE` and `REG_TORQUE_ENABLE` readback, serial TX echo handling, sync-read poll cadence, and loaded hold/stop response. |
| **B601-RS (RobStride)** | Verify motor wiring signs and joint limits (`profiles/rebot_b601_rs.json` is unverified on hardware), actual Type-2 feedback timing, and whether `TYPE_ENABLE` / `TYPE_STOP` produce causal torque-state evidence. |
| **FashionStar** | Qualify on a dedicated bench (angle ACK, power/interval semantics, telemetry) before claiming hardware parity. |
