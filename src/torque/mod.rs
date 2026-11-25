//! Pure `TorqueController` policy reducer core (no owner or driver wiring yet).
//!
//! # Overview
//!
//! `src/torque/` isolates all torque ownership, rest-gate verification, hold issuance, enable
//! handshakes, thermal derating/preemption, protective damping, and FU-PANIC safety policy into
//! a deterministic, allocation-free, I/O-free, lock-free state machine:
//!
//! ```text
//! step(&mut TorqueState, Event) -> ActionSet
//! ```
//!
//! Both control owners (legacy `SupervisorInner` and RT `Executor`) will feed normalized
//! monotonic events into the same reducer and execute the emitted `ActionSet` through a single
//! capability-gated emitter per release.
//!
//! # Topology Model (Three Object Kinds)
//!
//! Fixed capacities (`MAX_CHANNELS = 32`, `MAX_DEVICES = 4`, `MAX_GROUPS = 8`) with overflow
//! rejected at profile load (`TorqueConfig::validate`):
//!
//! 1. **Logical joints / channels** (`0..32`): carry captured hold position, sample provenance,
//!    desired vs. verified [`crate::motor_mode::ControlMode`], last motion-write boundary,
//!    thermal derate state, and functional [`types::JointRole`]:
//!    - `LoadBearing`: requires full `K = 3` rest-pose gate before non-protective disable.
//!    - `Gripper(LegacyJ7)`: 3 fresh distinct causal samples across distinct releases, no rest
//!      position tolerance; stop / `FailedHold` never opens jaws or raises force.
//!    - `Gripper(ExplicitHand)`: requires explicit release authorization plus verified release
//!      evidence (`SupportedObject` or `VerifiedEmptyGrasp`); unknown evidence defers or refuses.
//!    - `ActiveTool(StopOutput | BoundedRetention)`: semantic safe output required at profile load.
//!    - `PassiveSensor`: observations only; never receives torque commands.
//! 2. **Command groups** (`0..8`): track [`types::TorqueIntent`], [`types::DriveState`],
//!    `stop_generation`, enable phase, protective latch, and disable transaction independently
//!    (disabling a hand group leaves arm holds active).
//! 3. **Physical disable recipients** (`0..32`): each grouped packet declares its full
//!    `grouped_packet_mask`; a grouped disable passes only when every actual recipient passes
//!    its role policy in the same release.
//!
//! # Drive States (CiA 402 + Repo Extensions)
//!
//! | State | `TorqueIntent` | Meaning |
//! |---|---|---|
//! | `SwitchOnDisabled` | `Off` | Idle / disabled; safe to start an enable handshake. |
//! | `ReadyToSwitchOn` | `On` | Enable started; awaiting handshake + mode + TIMEOUT verification. |
//! | `OperationEnabled` | `On` | Operational; motion allowed while unfenced. |
//! | `QuickStopActive` | `On` | Stopped (`stop -> hold`); every captured joint held every release. |
//! | `FaultReactionActive` | `On` | Active fault reaction (e.g., Tier-1 worker-panic park in flight). |
//! | `Fault` | `On` / `Off` | Latched fault; holds continue while `On` until rest-gated disable. |
//! | `FailedHold` | `On` | Failed / interrupted / expired enable (`ENABLE_FAILED`); no rollback. |
//! | `ProtectiveDamping` | `On` | Bounded MIT `kd`-only damping before `DisableProtective` (if configured & verified). |
//! | `ProtectiveDisabling` | `On` / `Off` | `DisableProtective` scheduled by next serviced release (`N = 1`), reserved TX budget. |
//! | `ProtectiveDisabled` | `Off` | Protective disable committed; protective latch stays active until cleared. |
//! | `SafeHold` | `On` | FU-PANIC Tier-2 caught control-thread panic; cuts only on thermal or panic bus-loss limit. |
//!
//! # Events
//!
//! - `Release`: advances `current_release_id` and monotonic `now_us`; expires stale captures,
//!   deadlines, damping, and bus-loss timers; emits holds, protective actions, repeats, and refreshes.
//! - `CausalSample`: validates clock monotonicity (rejects future timestamps without clamping),
//!   `<= 1 ms` clock-offset drift, poisoned-line silence, and post-motion settle (`> 20 ms`);
//!   updates anchored captures and feeds active `K = 3` rest windows. Whole-hand ACKs complete
//!   handshakes only and are never position evidence.
//! - `WriteResult`: per-motor outcomes from the emitter, correlated by `dispatch_id` /
//!   `attempt_id` + `kind` + `generation`; stationary holds preserve the motion-write boundary;
//!   the first correlated `Sent` or `Ambiguous` disable commits `TorqueIntent::Off`; zero writes
//!   keep `On`; 25 consecutive lock-contention releases escalate once and latch `DriverWriteFailed`.
//! - `HandshakeResult` (correlated by `attempt_id`), `EnableStart`, `DisableRequest`, `Stop`
//!   (Stop / E-Stop / Fault / Watchdog / LeaseLoss), `FaultClear`, `Thermal`,
//!   `MitOwnershipChange`, `ModeVerified`, `TimeoutReadback`, `MotionCommand`, `SerialTimeout`,
//!   `GraspStateUpdate`, `ParkAccepted` / `ParkSettled` (correlated by `park_id`), `Panic`,
//!   `ProcessExit`, `BusLoss`.
//!
//! # Actions & Priority
//!
//! `ActionSet` is fixed-capacity and zero-allocation, with strict emitter priority ordering via
//! [`types::ActionSet::visit_by_priority`]:
//! - `Protective` (`Damp`, `DisableProtective`)
//! - `StopHold` (`DisableRequested`, `Hold`, `RequestPark`)
//! - `Motion` (`Enable`, `MotionAllowed`)
//! - `RefreshRoutine` (`Refresh`, `DisableComplete`, `Refusal`, `EscalateContention`)
//!
//! # Invariants
//!
//! Every safety property is independently machine-checked on `(state, prev, event, actions)` by
//! [`check_invariants`], and write-time capability validity is exposed via
//! [`is_disable_authorized`]:
//!
//! - **Hold**: `TorqueIntent::On` while stopped or `FailedHold` emits a hold for every captured
//!   joint every release regardless of reported enable state; captures require finite measured
//!   positions `<= 100 ms` old; uncaptured/unverified joints are surfaced in telemetry; grippers
//!   never open or raise force on stop/`FailedHold`.
//! - **Rest gate**: all actual recipients pass in the same release (`K = 3` consecutive distinct
//!   samples across distinct releases, span `>= 20 ms`, strictly `> 20 ms` after last motion write);
//!   stationary holds preserve the motion boundary; invalid/disagreeing evidence resets the window;
//!   missing evidence defers up to `1 s`; definite non-rest refuses with `TORQUE_OFF_WOULD_DROP`.
//! - **Dispatch**: gate success authorizes one generation-bound dispatch in the current release;
//!   zero writes keep `TorqueIntent::On`; the first `Sent` or `Ambiguous` disable commits `Off`
//!   before any later hold or motion; bounded repeats (Damiao `3 × 20 ms`) stay inside the original
//!   deadline; motion and enable are fenced until completion; partial dispatch is surfaced explicitly.
//! - **Enable**: `EnableStart` sets `TorqueIntent::On` (after refusing TIMEOUT mismatch or active
//!   fences); failed/interrupted/expired enable enters `FailedHold` + `ENABLE_FAILED` with no
//!   rollback; reported enable state feeds telemetry and handshake completion only; `FaultClear`
//!   never dispatches a disable.
//! - **Thermal & Panic**: linear derate between `derate_start_c` and `trip_c` (default: none);
//!   thermal preempts everything and schedules protective disable by `N = 1` with reserved TX
//!   budget (preceded by MIT `kd`-only damping only when configured and verified); ordinary bus
//!   loss grants no protective authority; Tier-1 worker panic parks arm without releasing hand
//!   payload and keeps holding if park fails; Tier-2 caught control panic holds until thermal or
//!   panic bus-loss limit.
//!
//! # How Both Owners Feed the Reducer (PR1 Shadow Mode & PR2+ Live Wiring)
//!
//! 1. At profile load, both owners construct `TorqueConfig` from `ArmConfig` (defaulting omitted
//!    role/group/device fields to the existing topology) and call `TorqueState::new(config)`.
//! 2. At each control release, the owner converts monotonic `Instant` deltas into `u64`
//!    microseconds and feeds events in deterministic order:
//!    - `Event::Release { now_us }`
//!    - `Event::CausalSample` / `Event::Thermal` / `Event::SerialTimeout` / `Event::BusLoss` from RX
//!    - `Event::Stop` / `Event::EnableStart` / `Event::DisableRequest` / `Event::FaultClear` /
//!      `Event::MitOwnershipChange` / `Event::MotionCommand` / `Event::Panic` from control inputs
//!    - `Event::HandshakeResult` / `Event::ModeVerified` / `Event::TimeoutReadback` from driver polls
//! 3. In PR1 (shadow mode), owners compare `ActionSet` against existing logic, count and log
//!    divergences, and still let existing logic drive hardware.
//! 4. In PR2+, the single capability-gated emitter drains `ActionSet::visit_by_priority`, performs
//!    write-time rechecks (generation, authorization, mode, freshness, deadline), transmits within
//!    per-bus normal + reserved-safety quotas, and feeds the exact per-motor `Event::WriteResult`
//!    back into `step()`.

pub mod invariants;
pub mod reducer;
pub mod types;

pub use invariants::{check_invariants, Violation, ViolationKind};
pub use reducer::{compute_thermal_derate_scale, event_now_us, is_disable_authorized, step};
pub use types::*;
