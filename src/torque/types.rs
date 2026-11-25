//! Core data types, topology configuration, CiA 402 state model, events, actions, and refusal codes
//! for the pure [`crate::torque`] reducer.
//!
//! All structures use fixed-capacity arrays and bitsets (`MAX_CHANNELS = 32`, `MAX_DEVICES = 4`,
//! `MAX_GROUPS = 8`) and integer monotonic timestamps in microseconds (`u64`), with zero heap
//! allocation, zero I/O, and zero locks.

use serde::{Deserialize, Serialize};

use crate::motor_mode::ControlMode;

// =========================================================================
// Capacity & Timing Constants
// =========================================================================

/// Maximum logical actuator/sensor channels supported by one reducer instance.
pub const MAX_CHANNELS: usize = 32;

/// Maximum physical bus devices supported by one reducer instance.
pub const MAX_DEVICES: usize = 4;

/// Maximum independently switchable torque groups supported by one reducer instance.
pub const MAX_GROUPS: usize = 8;

/// Microseconds per millisecond.
pub const US_PER_MS: u64 = 1_000;

/// Maximum permitted age of a measured position sample to capture or keep a hold (100 ms).
pub const DEFAULT_CAPTURE_MAX_AGE_US: u64 = 100 * US_PER_MS;

/// Post-motion settle delay before rest-gate evidence is eligible (strictly `> 20 ms`).
pub const REST_SETTLE_US: u64 = 20 * US_PER_MS;

/// Minimum time span across the `K = 3` rest-gate window (`>= 20 ms`).
pub const REST_WINDOW_MIN_SPAN_US: u64 = 20 * US_PER_MS;

/// Consecutive distinct samples from distinct releases required by the rest gate (`K = 3`).
pub const REST_WINDOW_SAMPLES: u8 = 3;

/// Maximum / default lifetime of an explicit disable request / rest-gate defer window (1 s).
pub const DEFAULT_DISABLE_DEADLINE_US: u64 = 1_000 * US_PER_MS;

/// Default enable handshake deadline (500 ms).
pub const DEFAULT_ENABLE_DEADLINE_US: u64 = 500 * US_PER_MS;

/// Damiao bounded disable repeats (`3 × 20 ms`).
pub const DAMIAO_DISABLE_REPEATS: u8 = 3;

/// Spacing between Damiao disable repeats (20 ms).
pub const DAMIAO_DISABLE_REPEAT_INTERVAL_US: u64 = 20 * US_PER_MS;

/// Feetech serial timeout poisoned-line silence requirement (20 ms).
pub const FEETECH_POISON_SILENCE_US: u64 = 20 * US_PER_MS;

/// Maximum tolerated `CLOCK_REALTIME - CLOCK_MONOTONIC` offset drift (1 ms).
pub const CLOCK_OFFSET_MAX_DRIFT_US: u64 = US_PER_MS;

/// Consecutive serviced releases skipped due to hold lock contention before escalating (25).
pub const CONTENTION_ESCALATION_RELEASES: u16 = 25;

/// Maximum allowed Damiao Reg 9 `TIMEOUT` register count (`8000 × 50 µs = 400 ms`).
pub const DAMIAO_TIMEOUT_MAX_REG: u16 = 8_000;

/// Microseconds per Damiao Reg 9 `TIMEOUT` register count.
pub const DAMIAO_TIMEOUT_TICK_US: u64 = 50;

/// Default bus-loss limit in Tier-2 `SafeHold` before protective disable (500 ms).
pub const DEFAULT_PANIC_BUS_LOSS_LIMIT_US: u64 = 500 * US_PER_MS;

// =========================================================================
// Bitsets
// =========================================================================

/// 32-bit bitset of logical channels or physical disable recipients (`0..32`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ChannelMask(pub u32);

impl ChannelMask {
    pub const EMPTY: Self = Self(0);
    pub const ALL: Self = Self(u32::MAX);

    #[inline]
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    #[inline]
    pub const fn raw(self) -> u32 {
        self.0
    }

    #[inline]
    pub const fn bit(idx: u8) -> Self {
        if (idx as usize) < MAX_CHANNELS {
            Self(1u32 << idx)
        } else {
            Self::EMPTY
        }
    }

    #[inline]
    pub const fn from_count(count: u8) -> Self {
        if count == 0 {
            Self::EMPTY
        } else if (count as usize) >= MAX_CHANNELS {
            Self::ALL
        } else {
            Self((1u32 << count) - 1)
        }
    }

    #[inline]
    pub const fn contains(self, idx: u8) -> bool {
        (idx as usize) < MAX_CHANNELS && (self.0 & (1u32 << idx)) != 0
    }

    #[inline]
    pub fn insert(&mut self, idx: u8) {
        if (idx as usize) < MAX_CHANNELS {
            self.0 |= 1u32 << idx;
        }
    }

    #[inline]
    pub fn remove(&mut self, idx: u8) {
        if (idx as usize) < MAX_CHANNELS {
            self.0 &= !(1u32 << idx);
        }
    }

    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    #[inline]
    pub const fn intersect(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    #[inline]
    pub const fn difference(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    #[inline]
    pub const fn is_subset_of(self, superset: Self) -> bool {
        (self.0 & !superset.0) == 0
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    #[inline]
    pub const fn count(self) -> u32 {
        self.0.count_ones()
    }

    #[inline]
    pub fn iter(self) -> ChannelMaskIter {
        ChannelMaskIter(self.0)
    }
}

/// Zero-alloc iterator over set bit indices in a [`ChannelMask`].
#[derive(Debug, Clone, Copy)]
pub struct ChannelMaskIter(u32);

impl Iterator for ChannelMaskIter {
    type Item = u8;

    #[inline]
    fn next(&mut self) -> Option<u8> {
        if self.0 == 0 {
            return None;
        }
        let tz = self.0.trailing_zeros() as u8;
        self.0 &= self.0 - 1;
        Some(tz)
    }
}

/// 4-bit bitset of physical devices (`0..4`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct DeviceMask(pub u8);

impl DeviceMask {
    pub const EMPTY: Self = Self(0);
    pub const ALL: Self = Self((1u8 << MAX_DEVICES) - 1);

    #[inline]
    pub const fn bit(idx: u8) -> Self {
        if (idx as usize) < MAX_DEVICES {
            Self(1u8 << idx)
        } else {
            Self::EMPTY
        }
    }

    #[inline]
    pub const fn from_count(count: u8) -> Self {
        if count == 0 {
            Self::EMPTY
        } else if (count as usize) >= MAX_DEVICES {
            Self::ALL
        } else {
            Self((1u8 << count) - 1)
        }
    }

    #[inline]
    pub const fn contains(self, idx: u8) -> bool {
        (idx as usize) < MAX_DEVICES && (self.0 & (1u8 << idx)) != 0
    }

    #[inline]
    pub fn insert(&mut self, idx: u8) {
        if (idx as usize) < MAX_DEVICES {
            self.0 |= 1u8 << idx;
        }
    }

    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    #[inline]
    pub const fn intersect(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    #[inline]
    pub fn iter(self) -> SmallMaskIter {
        SmallMaskIter(self.0 & Self::ALL.0)
    }
}

/// 8-bit bitset of torque groups (`0..8`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct GroupMask(pub u8);

impl GroupMask {
    pub const EMPTY: Self = Self(0);
    pub const ALL: Self = Self(u8::MAX);

    #[inline]
    pub const fn bit(idx: u8) -> Self {
        if (idx as usize) < MAX_GROUPS {
            Self(1u8 << idx)
        } else {
            Self::EMPTY
        }
    }

    #[inline]
    pub const fn from_count(count: u8) -> Self {
        if count == 0 {
            Self::EMPTY
        } else if (count as usize) >= MAX_GROUPS {
            Self::ALL
        } else {
            Self((1u8 << count) - 1)
        }
    }

    #[inline]
    pub const fn contains(self, idx: u8) -> bool {
        (idx as usize) < MAX_GROUPS && (self.0 & (1u8 << idx)) != 0
    }

    #[inline]
    pub fn insert(&mut self, idx: u8) {
        if (idx as usize) < MAX_GROUPS {
            self.0 |= 1u8 << idx;
        }
    }

    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    #[inline]
    pub const fn intersect(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    #[inline]
    pub fn iter(self) -> SmallMaskIter {
        SmallMaskIter(self.0)
    }
}

/// Zero-alloc iterator over set bit indices in an 8-bit mask.
#[derive(Debug, Clone, Copy)]
pub struct SmallMaskIter(u8);

impl Iterator for SmallMaskIter {
    type Item = u8;

    #[inline]
    fn next(&mut self) -> Option<u8> {
        if self.0 == 0 {
            return None;
        }
        let tz = self.0.trailing_zeros() as u8;
        self.0 &= self.0 - 1;
        Some(tz)
    }
}

// =========================================================================
// CiA 402 + Repo Extension State Model
// =========================================================================

/// Decision-based torque ownership for one group (never inferred from motor enable feedback).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TorqueIntent {
    #[default]
    Off,
    On,
}

/// Per-group drive state following CiA 402 names plus repo safety/protective extensions.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriveState {
    /// CiA 402: disabled / idle (`TorqueIntent::Off`). Safe to begin enable.
    #[default]
    SwitchOnDisabled,
    /// CiA 402: enable handshake started and awaiting confirmation (`TorqueIntent::On`).
    ReadyToSwitchOn,
    /// CiA 402: enabled and operational (`TorqueIntent::On`). Motion permitted.
    OperationEnabled,
    /// CiA 402: stopped and actively holding (`stop -> hold`, `TorqueIntent::On`).
    QuickStopActive,
    /// CiA 402: active fault reaction (e.g., Tier-1 worker-panic park in flight).
    FaultReactionActive,
    /// CiA 402: latched fault (`EStop`, `BusLoss`, `Watchdog`, etc.). Holds continue while `TorqueIntent::On`.
    Fault,
    /// Repo extension: failed, interrupted, or expired enable (`TorqueIntent::On`, no rollback, `ENABLE_FAILED`).
    FailedHold,
    /// Protective extension: bounded MIT kd-only damping phase preceding `DisableProtective`.
    ProtectiveDamping,
    /// Protective extension: `DisableProtective` scheduled/in-flight with reserved TX budget.
    ProtectiveDisabling,
    /// Protective extension: protective disable committed (`TorqueIntent::Off`, protective latch active).
    ProtectiveDisabled,
    /// FU-PANIC Tier 2: caught panic on `armd-control` -> lock-free safe-hold loop.
    /// Cuts only on thermal trip or panic bus-loss limit.
    SafeHold,
}

impl DriveState {
    /// True when the group is in a stopped or safe-hold state that requires per-release holds
    /// whenever `TorqueIntent == On`.
    #[inline]
    pub const fn requires_holds_when_on(self) -> bool {
        matches!(
            self,
            Self::QuickStopActive | Self::Fault | Self::FailedHold | Self::SafeHold
        )
    }

    /// True when the group is in any protective state.
    #[inline]
    pub const fn is_protective(self) -> bool {
        matches!(
            self,
            Self::ProtectiveDamping | Self::ProtectiveDisabling | Self::ProtectiveDisabled
        )
    }
}

/// Enable handshake sub-phase tracked per group.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EnablePhase {
    #[default]
    Idle,
    Enabling,
    Validating,
    Armed,
    FailedHold,
}

// =========================================================================
// Roles, Vendors, Policies, and Topology Configuration
// =========================================================================

/// Vendor protocol family controlling rest-evidence qualification and transport timing semantics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VendorKind {
    #[default]
    Damiao,
    Feetech,
    RobStride,
    FashionStar,
    GenericGrouped,
}

/// Release policy for a [`JointRole::Gripper`] channel or recipient.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GripperReleasePolicy {
    /// Legacy J7 compatibility: 3 fresh distinct causal samples across distinct releases
    /// spanning `>= 20 ms` strictly `> 20 ms` after last motion write; no rest position tolerance.
    LegacyJ7,
    /// New gripper / dexterous hand: requires explicit release authorization PLUS verified
    /// release evidence (`SupportedObject` or `VerifiedEmptyGrasp`); unknown defers/refuses.
    ExplicitHand,
}

/// Semantic safe output for a [`JointRole::ActiveTool`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActiveToolSafeOutput {
    /// Stop output immediately on stop / FailedHold / disable (e.g., motorized screwdriver).
    StopOutput,
    /// Keep retention active up to `max_retention_us` while stopped so payloads are not dropped
    /// (e.g., vacuum suction), unless explicitly released or protective disable preempts.
    BoundedRetention { max_retention_us: u64 },
}

/// Functional safety role of a logical joint / physical recipient.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointRole {
    /// Load-bearing arm axis: full `K = 3` rest-pose gate before non-protective disable.
    #[default]
    LoadBearing,
    /// Gripper or dexterous hand axis: never opens jaws or raises force on stop/FailedHold.
    Gripper(GripperReleasePolicy),
    /// Active end-effector tool with a mandatory qualified safe response.
    ActiveTool(ActiveToolSafeOutput),
    /// Observation-only sensor: never receives hold, enable, or disable commands.
    PassiveSensor,
}

/// Qualification wrapper used during profile validation so an unqualified active tool is
/// rejected at profile load.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActiveToolConfig {
    #[default]
    NotActiveTool,
    Qualified(ActiveToolSafeOutput),
    Unqualified,
}

/// Causal release evidence reported for a gripper / dexterous hand channel.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReleaseEvidence {
    #[default]
    Unknown,
    /// Grasp is holding an object that is externally supported (safe to release).
    SupportedObject,
    /// Grasp is verified empty (safe to disable).
    VerifiedEmptyGrasp,
    /// Grasp is holding an unsupported payload (definite unsafe release -> `TORQUE_OFF_WOULD_DROP`).
    HoldingUnsupportedPayload,
}

/// Profile-configured MIT damping parameters preceding `DisableProtective` (thermal / exit).
/// Default `duration_us = 0` disables damping until measured on the real arm.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct DampingConfig {
    /// Damping phase duration in microseconds (`0` = no damping phase).
    pub duration_us: u64,
    /// Pure velocity damping gain (`kp = 0`, `kd = kd`).
    pub kd: f32,
    /// Bounded torque ceiling during damping (N·m).
    pub max_torque_nm: f32,
}

/// Profile-configured thermal derate and protective trip thresholds (°C).
/// Default `derate_start_c = None` preserves today's no-derate behavior until trip.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ThermalConfig {
    /// Optional temperature where linear torque/current derating begins (`1.0 -> 0.0` at `trip_c`).
    pub derate_start_c: Option<f32>,
    /// Trip temperature triggering immediate protective preemption and `DisableProtective`.
    pub trip_c: f32,
}

impl Default for ThermalConfig {
    fn default() -> Self {
        Self {
            derate_start_c: None,
            trip_c: 70.0,
        }
    }
}

/// Static configuration for one logical channel (`0..MAX_CHANNELS`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ChannelConfig {
    pub id: u8,
    pub device: u8,
    pub group: u8,
    pub recipient: u8,
    pub role: JointRole,
    pub active_tool_qualification: ActiveToolConfig,
    pub vendor: VendorKind,
    pub desired_mode: ControlMode,
    pub rest_position_rad: f32,
    pub rest_tolerance_rad: f32,
    pub rest_max_velocity_rad_s: Option<f32>,
    pub supports_mit_damping: bool,
    pub damping: DampingConfig,
    pub thermal: ThermalConfig,
    /// Configured Damiao Reg 9 `TIMEOUT` value (`1..=8000`, 50 µs/count).
    /// When `Some(v)`, enable is refused until current-session readback equals `v`.
    pub damiao_timeout_reg: Option<u16>,
    /// Freshness bound for hold capture and rest-window validity (`1..=100_000` µs).
    pub max_capture_age_us: u64,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            id: 0,
            device: 0,
            group: 0,
            recipient: 0,
            role: JointRole::LoadBearing,
            active_tool_qualification: ActiveToolConfig::NotActiveTool,
            vendor: VendorKind::Damiao,
            desired_mode: ControlMode::PositionVelocity,
            rest_position_rad: 0.0,
            rest_tolerance_rad: 0.08,
            rest_max_velocity_rad_s: Some(0.015),
            supports_mit_damping: false,
            damping: DampingConfig::default(),
            thermal: ThermalConfig::default(),
            damiao_timeout_reg: None,
            max_capture_age_us: DEFAULT_CAPTURE_MAX_AGE_US,
        }
    }
}

/// Static configuration for one physical disable recipient (`0..MAX_CHANNELS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecipientConfig {
    pub id: u8,
    pub device: u8,
    pub group: u8,
    /// Logical channels driven by this physical disable recipient.
    pub channels: ChannelMask,
    /// Full set of physical recipients affected when a disable packet addresses this recipient.
    /// Must include `self.id`; for grouped packets, lists all co-addressed recipients.
    pub grouped_packet_mask: ChannelMask,
    pub role: JointRole,
    pub vendor: VendorKind,
    /// Whether this recipient's vendor/driver is bench-qualified to supply rest-pose evidence.
    /// `false` for `VendorKind::FashionStar` until bench-qualified.
    pub rest_evidence_qualified: bool,
}

impl Default for RecipientConfig {
    fn default() -> Self {
        Self {
            id: 0,
            device: 0,
            group: 0,
            channels: ChannelMask::EMPTY,
            grouped_packet_mask: ChannelMask::EMPTY,
            role: JointRole::LoadBearing,
            vendor: VendorKind::Damiao,
            rest_evidence_qualified: true,
        }
    }
}

/// Static configuration for one independently switchable torque group (`0..MAX_GROUPS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GroupConfig {
    pub id: u8,
    /// Logical channels belonging to this group (must equal canonical channel ownership).
    pub channels: ChannelMask,
    /// Physical disable recipients belonging to this group (must equal canonical recipient ownership).
    pub recipients: ChannelMask,
    /// Devices spanned by this group (must equal canonical device ownership).
    pub devices: DeviceMask,
    /// Bounded disable repeat count (`1..=3` for Damiao, `1..=3` general).
    pub disable_repeats: u8,
    /// Spacing between bounded disable repeats (`>= 20 ms` for Damiao when `disable_repeats > 1`).
    pub disable_repeat_interval_us: u64,
    /// Single deadline for disable request rest-gate defer + bounded repeats (`1..=1_000_000` µs).
    pub disable_deadline_us: u64,
    /// Deadline for enable handshake (`> 0`).
    pub enable_deadline_us: u64,
    /// Whether Tier-1 worker panic may execute a park-to-rest trajectory on this group.
    pub has_park_path: bool,
}

impl Default for GroupConfig {
    fn default() -> Self {
        Self {
            id: 0,
            channels: ChannelMask::EMPTY,
            recipients: ChannelMask::EMPTY,
            devices: DeviceMask::EMPTY,
            disable_repeats: DAMIAO_DISABLE_REPEATS,
            disable_repeat_interval_us: DAMIAO_DISABLE_REPEAT_INTERVAL_US,
            disable_deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            enable_deadline_us: DEFAULT_ENABLE_DEADLINE_US,
            has_park_path: true,
        }
    }
}

/// Static configuration for one physical bus device (`0..MAX_DEVICES`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceConfig {
    pub id: u8,
    /// Shared physical bus identifier (`0..MAX_DEVICES`); devices sharing `bus_id` must agree on
    /// bus-level normal/reserved TX quotas.
    pub bus_id: u8,
    /// Shared normal TX frame quota per release on `bus_id`.
    pub tx_quota_per_release: u16,
    /// Shared reserved safety TX capacity per release on `bus_id` (protective damping / disable).
    pub reserved_safety_tx: u16,
    /// Device operation deadline (µs).
    pub deadline_us: u64,
    /// When true, a mode switch on any channel of this device fences every group touching this device
    /// until verified.
    pub device_wide_mode_switch: bool,
    /// Silence period required after a serial timeout before evidence is accepted (20 ms for Feetech).
    pub poison_silence_us: u64,
    /// Continuous bus-loss duration in Tier-2 `SafeHold` before protective disable is allowed.
    pub panic_bus_loss_limit_us: u64,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            id: 0,
            bus_id: 0,
            tx_quota_per_release: 16,
            reserved_safety_tx: 8,
            deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            device_wide_mode_switch: false,
            poison_silence_us: 0,
            panic_bus_loss_limit_us: DEFAULT_PANIC_BUS_LOSS_LIMIT_US,
        }
    }
}

/// Complete fixed-capacity topology and policy configuration for one reducer instance.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TorqueConfig {
    pub channel_count: u8,
    pub recipient_count: u8,
    pub group_count: u8,
    pub device_count: u8,
    pub channels: [ChannelConfig; MAX_CHANNELS],
    pub recipients: [RecipientConfig; MAX_CHANNELS],
    pub groups: [GroupConfig; MAX_GROUPS],
    pub devices: [DeviceConfig; MAX_DEVICES],
}

impl Default for TorqueConfig {
    fn default() -> Self {
        Self {
            channel_count: 0,
            recipient_count: 0,
            group_count: 0,
            device_count: 0,
            channels: [ChannelConfig::default(); MAX_CHANNELS],
            recipients: [RecipientConfig::default(); MAX_CHANNELS],
            groups: [GroupConfig::default(); MAX_GROUPS],
            devices: [DeviceConfig::default(); MAX_DEVICES],
        }
    }
}

/// Zero-allocation profile validation errors rejected at load time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConfigError {
    ChannelCapacityExceeded { count: u8, max: usize },
    RecipientCapacityExceeded { count: u8, max: usize },
    GroupCapacityExceeded { count: u8, max: usize },
    DeviceCapacityExceeded { count: u8, max: usize },
    EmptyTopology,
    InvalidChannelRef { channel: u8 },
    InvalidRecipientRef { recipient: u8 },
    InvalidGroupRef { group: u8 },
    InvalidDeviceRef { device: u8 },
    UnqualifiedActiveTool { channel: u8 },
    InvalidActiveToolRetention { channel: u8 },
    InvalidRestTolerance { channel: u8 },
    InvalidThermalConfig { channel: u8 },
    InvalidDampingConfig { channel: u8 },
    InvalidDamiaoTimeoutReg { channel: u8, reg: u16 },
    InvalidCaptureAge { channel: u8 },
    InvalidGroupedPacketMask { recipient: u8 },
    CrossGroupRecipientBleed { recipient: u8, target: u8 },
    DuplicateRecipientOwner { recipient: u8 },
    PassiveSensorListedAsRecipient { channel: u8, recipient: u8 },
    InvalidGroupTiming { group: u8 },
    InvalidDeviceBudget { device: u8 },
    SharedBusBudgetMismatch { bus_id: u8 },
    MismatchedChannelTopology { channel: u8 },
    MismatchedGroupTopology { group: u8 },
}

impl TorqueConfig {
    /// Validate capacity bounds, canonical group membership, role qualifications, timing bounds,
    /// and physical constraints.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if (self.channel_count as usize) > MAX_CHANNELS {
            return Err(ConfigError::ChannelCapacityExceeded {
                count: self.channel_count,
                max: MAX_CHANNELS,
            });
        }
        if (self.recipient_count as usize) > MAX_CHANNELS {
            return Err(ConfigError::RecipientCapacityExceeded {
                count: self.recipient_count,
                max: MAX_CHANNELS,
            });
        }
        if (self.group_count as usize) > MAX_GROUPS {
            return Err(ConfigError::GroupCapacityExceeded {
                count: self.group_count,
                max: MAX_GROUPS,
            });
        }
        if (self.device_count as usize) > MAX_DEVICES {
            return Err(ConfigError::DeviceCapacityExceeded {
                count: self.device_count,
                max: MAX_DEVICES,
            });
        }
        if self.channel_count == 0 || self.group_count == 0 || self.device_count == 0 {
            return Err(ConfigError::EmptyTopology);
        }

        let valid_channels = ChannelMask::from_count(self.channel_count);
        let valid_recipients = ChannelMask::from_count(self.recipient_count);
        let valid_devices = DeviceMask::from_count(self.device_count);

        // Validate devices and enforce consistent shared-bus budgets (M14).
        let mut bus_quota: [Option<(u16, u16)>; MAX_DEVICES] = [None; MAX_DEVICES];
        for d in 0..self.device_count {
            let dev = &self.devices[d as usize];
            if dev.id != d || (dev.bus_id as usize) >= MAX_DEVICES {
                return Err(ConfigError::InvalidDeviceRef { device: d });
            }
            if dev.tx_quota_per_release == 0
                || dev.reserved_safety_tx == 0
                || dev.deadline_us == 0
                || dev.panic_bus_loss_limit_us == 0
            {
                return Err(ConfigError::InvalidDeviceBudget { device: d });
            }
            let b = dev.bus_id as usize;
            match bus_quota[b] {
                None => bus_quota[b] = Some((dev.tx_quota_per_release, dev.reserved_safety_tx)),
                Some((q, r)) if q == dev.tx_quota_per_release && r == dev.reserved_safety_tx => {}
                Some(_) => return Err(ConfigError::SharedBusBudgetMismatch { bus_id: dev.bus_id }),
            }
        }

        // Canonical group membership derived from channel and recipient ownership (B4).
        let mut derived_group_channels = [ChannelMask::EMPTY; MAX_GROUPS];
        let mut derived_group_devices = [DeviceMask::EMPTY; MAX_GROUPS];
        let mut derived_group_recipients = [ChannelMask::EMPTY; MAX_GROUPS];
        let mut group_has_damiao = [false; MAX_GROUPS];

        for c in 0..self.channel_count {
            let ch = &self.channels[c as usize];
            if ch.id != c || ch.device >= self.device_count || ch.group >= self.group_count {
                return Err(ConfigError::InvalidChannelRef { channel: c });
            }
            derived_group_channels[ch.group as usize].insert(c);
            derived_group_devices[ch.group as usize].insert(ch.device);
            if matches!(ch.vendor, VendorKind::Damiao) {
                group_has_damiao[ch.group as usize] = true;
            }
            // M4: hold capture age ceiling <= 100 ms.
            if ch.max_capture_age_us == 0 || ch.max_capture_age_us > DEFAULT_CAPTURE_MAX_AGE_US {
                return Err(ConfigError::InvalidCaptureAge { channel: c });
            }
            if !ch.thermal.trip_c.is_finite() || ch.thermal.trip_c <= 0.0 {
                return Err(ConfigError::InvalidThermalConfig { channel: c });
            }
            if let Some(start_c) = ch.thermal.derate_start_c {
                if !start_c.is_finite() || start_c <= 0.0 || start_c >= ch.thermal.trip_c {
                    return Err(ConfigError::InvalidThermalConfig { channel: c });
                }
            }
            if ch.damping.duration_us > 0
                && (!ch.damping.kd.is_finite()
                    || ch.damping.kd <= 0.0
                    || !ch.damping.max_torque_nm.is_finite()
                    || ch.damping.max_torque_nm <= 0.0)
            {
                return Err(ConfigError::InvalidDampingConfig { channel: c });
            }
            if let Some(reg) = ch.damiao_timeout_reg {
                if reg == 0 || reg > DAMIAO_TIMEOUT_MAX_REG {
                    return Err(ConfigError::InvalidDamiaoTimeoutReg { channel: c, reg });
                }
            }
            match ch.role {
                JointRole::PassiveSensor => {
                    if self.recipient_count > 0
                        && ch.recipient < self.recipient_count
                        && self.recipients[ch.recipient as usize].channels.contains(c)
                    {
                        return Err(ConfigError::PassiveSensorListedAsRecipient {
                            channel: c,
                            recipient: ch.recipient,
                        });
                    }
                }
                JointRole::LoadBearing => {
                    if !ch.rest_position_rad.is_finite()
                        || !ch.rest_tolerance_rad.is_finite()
                        || ch.rest_tolerance_rad <= 0.0
                    {
                        return Err(ConfigError::InvalidRestTolerance { channel: c });
                    }
                    if let Some(v) = ch.rest_max_velocity_rad_s {
                        if !v.is_finite() || v <= 0.0 {
                            return Err(ConfigError::InvalidRestTolerance { channel: c });
                        }
                    }
                    self.validate_channel_recipient(c, ch)?;
                }
                JointRole::Gripper(_) => {
                    self.validate_channel_recipient(c, ch)?;
                }
                JointRole::ActiveTool(safe_out) => {
                    match ch.active_tool_qualification {
                        ActiveToolConfig::Qualified(q) if q == safe_out => {}
                        _ => return Err(ConfigError::UnqualifiedActiveTool { channel: c }),
                    }
                    if let ActiveToolSafeOutput::BoundedRetention { max_retention_us } = safe_out {
                        if max_retention_us == 0 {
                            return Err(ConfigError::InvalidActiveToolRetention { channel: c });
                        }
                    }
                    self.validate_channel_recipient(c, ch)?;
                }
            }
        }

        for r in 0..self.recipient_count {
            let rec = &self.recipients[r as usize];
            if rec.id != r
                || rec.device >= self.device_count
                || rec.group >= self.group_count
                || !rec.channels.is_subset_of(valid_channels)
                || rec.channels.is_empty()
            {
                return Err(ConfigError::InvalidRecipientRef { recipient: r });
            }
            if !rec.grouped_packet_mask.contains(r)
                || !rec.grouped_packet_mask.is_subset_of(valid_recipients)
            {
                return Err(ConfigError::InvalidGroupedPacketMask { recipient: r });
            }
            if matches!(rec.role, JointRole::PassiveSensor) {
                let first_ch = rec.channels.iter().next().unwrap_or(0);
                return Err(ConfigError::PassiveSensorListedAsRecipient {
                    channel: first_ch,
                    recipient: r,
                });
            }
            derived_group_recipients[rec.group as usize].insert(r);
            for target in rec.grouped_packet_mask.iter() {
                if self.recipients[target as usize].group != rec.group {
                    return Err(ConfigError::CrossGroupRecipientBleed {
                        recipient: r,
                        target,
                    });
                }
            }
            for c in rec.channels.iter() {
                let ch = &self.channels[c as usize];
                if ch.recipient != r || ch.role != rec.role || ch.group != rec.group {
                    return Err(ConfigError::MismatchedChannelTopology { channel: c });
                }
            }
        }

        let mut claimed_recipients = ChannelMask::EMPTY;
        for g in 0..self.group_count {
            let grp = &self.groups[g as usize];
            if grp.id != g
                || grp.channels.is_empty()
                || !grp.channels.is_subset_of(valid_channels)
                || !grp.recipients.is_subset_of(valid_recipients)
                || !grp.devices.intersect(valid_devices).eq(&grp.devices)
            {
                return Err(ConfigError::InvalidGroupRef { group: g });
            }
            // M9: enforce bounded 1 s request lifetime and Damiao 3 × 20 ms repeat bounds.
            if grp.disable_repeats == 0
                || grp.disable_repeats > DAMIAO_DISABLE_REPEATS
                || grp.disable_deadline_us == 0
                || grp.disable_deadline_us > DEFAULT_DISABLE_DEADLINE_US
                || grp.enable_deadline_us == 0
            {
                return Err(ConfigError::InvalidGroupTiming { group: g });
            }
            if group_has_damiao[g as usize]
                && grp.disable_repeats > 1
                && grp.disable_repeat_interval_us < DAMIAO_DISABLE_REPEAT_INTERVAL_US
            {
                return Err(ConfigError::InvalidGroupTiming { group: g });
            }
            // B4: declared group masks MUST exactly match canonical channel/recipient ownership.
            if grp.channels != derived_group_channels[g as usize]
                || grp.recipients != derived_group_recipients[g as usize]
                || grp.devices != derived_group_devices[g as usize]
            {
                return Err(ConfigError::MismatchedGroupTopology { group: g });
            }
            let overlap = claimed_recipients.intersect(grp.recipients);
            if !overlap.is_empty() {
                let first = overlap.iter().next().unwrap_or(0);
                return Err(ConfigError::DuplicateRecipientOwner { recipient: first });
            }
            claimed_recipients = claimed_recipients.union(grp.recipients);
        }

        Ok(())
    }

    fn validate_channel_recipient(&self, c: u8, ch: &ChannelConfig) -> Result<(), ConfigError> {
        if ch.recipient >= self.recipient_count {
            return Err(ConfigError::InvalidChannelRef { channel: c });
        }
        let rec = &self.recipients[ch.recipient as usize];
        if !rec.channels.contains(c) || rec.group != ch.group || rec.device != ch.device {
            return Err(ConfigError::MismatchedChannelTopology { channel: c });
        }
        Ok(())
    }

    /// Standard B601-DM 7-DOF topology (J1..J6 LoadBearing + J7 LegacyJ7 gripper, single group & device).
    pub fn rebot_dm_default() -> Self {
        let mut cfg = Self::default();
        cfg.channel_count = 7;
        cfg.recipient_count = 7;
        cfg.group_count = 1;
        cfg.device_count = 1;

        cfg.devices[0] = DeviceConfig {
            id: 0,
            bus_id: 0,
            tx_quota_per_release: 16,
            reserved_safety_tx: 8,
            deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            device_wide_mode_switch: false,
            poison_silence_us: 0,
            panic_bus_loss_limit_us: DEFAULT_PANIC_BUS_LOSS_LIMIT_US,
        };

        cfg.groups[0] = GroupConfig {
            id: 0,
            channels: ChannelMask::from_count(7),
            recipients: ChannelMask::from_count(7),
            devices: DeviceMask::bit(0),
            disable_repeats: DAMIAO_DISABLE_REPEATS,
            disable_repeat_interval_us: DAMIAO_DISABLE_REPEAT_INTERVAL_US,
            disable_deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            enable_deadline_us: DEFAULT_ENABLE_DEADLINE_US,
            has_park_path: true,
        };

        let tolerances = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08];
        for i in 0..7u8 {
            let is_gripper = i == 6;
            let role = if is_gripper {
                JointRole::Gripper(GripperReleasePolicy::LegacyJ7)
            } else {
                JointRole::LoadBearing
            };
            cfg.channels[i as usize] = ChannelConfig {
                id: i,
                device: 0,
                group: 0,
                recipient: i,
                role,
                active_tool_qualification: ActiveToolConfig::NotActiveTool,
                vendor: VendorKind::Damiao,
                desired_mode: if is_gripper {
                    ControlMode::PositionForce
                } else {
                    ControlMode::PositionVelocity
                },
                rest_position_rad: 0.0,
                rest_tolerance_rad: if is_gripper { 0.0 } else { tolerances[i as usize] },
                rest_max_velocity_rad_s: Some(if i < 3 { 0.005 } else { 0.015 }),
                supports_mit_damping: !is_gripper,
                damping: DampingConfig::default(),
                thermal: ThermalConfig::default(),
                damiao_timeout_reg: None,
                max_capture_age_us: DEFAULT_CAPTURE_MAX_AGE_US,
            };
            cfg.recipients[i as usize] = RecipientConfig {
                id: i,
                device: 0,
                group: 0,
                channels: ChannelMask::bit(i),
                grouped_packet_mask: ChannelMask::bit(i),
                role,
                vendor: VendorKind::Damiao,
                rest_evidence_qualified: true,
            };
        }
        cfg
    }

    /// Synthetic dual-group topology:
    /// - Group 0 (Device 0, Bus 0): 6-DOF LoadBearing Damiao arm (channels 0..6, recipients 0..6).
    /// - Group 1 (Device 1, Bus 1): 6-joint underactuated dexterous hand (channels 6..12) driven by
    ///   2 grouped physical disable recipients (recipients 6 and 7, grouped together in one packet
    ///   `grouped_packet_mask = {6, 7}`), `ExplicitHand` policy, `device_wide_mode_switch = true`.
    /// - Channel 12 (Device 1, Group 1): `PassiveSensor` tactile/force observation channel.
    pub fn arm_plus_underactuated_hand() -> Self {
        let mut cfg = Self::default();
        cfg.channel_count = 13;
        cfg.recipient_count = 8;
        cfg.group_count = 2;
        cfg.device_count = 2;

        cfg.devices[0] = DeviceConfig {
            id: 0,
            bus_id: 0,
            tx_quota_per_release: 16,
            reserved_safety_tx: 8,
            deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            device_wide_mode_switch: false,
            poison_silence_us: 0,
            panic_bus_loss_limit_us: DEFAULT_PANIC_BUS_LOSS_LIMIT_US,
        };
        cfg.devices[1] = DeviceConfig {
            id: 1,
            bus_id: 1,
            tx_quota_per_release: 8,
            reserved_safety_tx: 4,
            deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            device_wide_mode_switch: true,
            poison_silence_us: 0,
            panic_bus_loss_limit_us: DEFAULT_PANIC_BUS_LOSS_LIMIT_US,
        };

        let arm_channels = ChannelMask::from_count(6);
        let hand_channels = ChannelMask::from_raw(((1u32 << 7) - 1) << 6); // channels 6..=12
        let hand_recipients = ChannelMask::bit(6).union(ChannelMask::bit(7));

        cfg.groups[0] = GroupConfig {
            id: 0,
            channels: arm_channels,
            recipients: ChannelMask::from_count(6),
            devices: DeviceMask::bit(0),
            disable_repeats: DAMIAO_DISABLE_REPEATS,
            disable_repeat_interval_us: DAMIAO_DISABLE_REPEAT_INTERVAL_US,
            disable_deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            enable_deadline_us: DEFAULT_ENABLE_DEADLINE_US,
            has_park_path: true,
        };
        cfg.groups[1] = GroupConfig {
            id: 1,
            channels: hand_channels,
            recipients: hand_recipients,
            devices: DeviceMask::bit(1),
            disable_repeats: 1,
            disable_repeat_interval_us: DAMIAO_DISABLE_REPEAT_INTERVAL_US,
            disable_deadline_us: DEFAULT_DISABLE_DEADLINE_US,
            enable_deadline_us: DEFAULT_ENABLE_DEADLINE_US,
            has_park_path: false,
        };

        for i in 0..6u8 {
            cfg.channels[i as usize] = ChannelConfig {
                id: i,
                device: 0,
                group: 0,
                recipient: i,
                role: JointRole::LoadBearing,
                active_tool_qualification: ActiveToolConfig::NotActiveTool,
                vendor: VendorKind::Damiao,
                desired_mode: ControlMode::PositionVelocity,
                rest_position_rad: 0.0,
                rest_tolerance_rad: 0.08,
                rest_max_velocity_rad_s: Some(0.015),
                supports_mit_damping: true,
                damping: DampingConfig::default(),
                thermal: ThermalConfig::default(),
                damiao_timeout_reg: None,
                max_capture_age_us: DEFAULT_CAPTURE_MAX_AGE_US,
            };
            cfg.recipients[i as usize] = RecipientConfig {
                id: i,
                device: 0,
                group: 0,
                channels: ChannelMask::bit(i),
                grouped_packet_mask: ChannelMask::bit(i),
                role: JointRole::LoadBearing,
                vendor: VendorKind::Damiao,
                rest_evidence_qualified: true,
            };
        }

        // Underactuated hand: 6 finger joints across 2 physical tendon recipients (6: ch 6..9, 7: ch 9..12)
        for i in 6..12u8 {
            let rec = if i < 9 { 6 } else { 7 };
            cfg.channels[i as usize] = ChannelConfig {
                id: i,
                device: 1,
                group: 1,
                recipient: rec,
                role: JointRole::Gripper(GripperReleasePolicy::ExplicitHand),
                active_tool_qualification: ActiveToolConfig::NotActiveTool,
                vendor: VendorKind::GenericGrouped,
                desired_mode: ControlMode::PositionForce,
                rest_position_rad: 0.0,
                rest_tolerance_rad: 0.0,
                rest_max_velocity_rad_s: None,
                supports_mit_damping: false,
                damping: DampingConfig::default(),
                thermal: ThermalConfig::default(),
                damiao_timeout_reg: None,
                max_capture_age_us: DEFAULT_CAPTURE_MAX_AGE_US,
            };
        }
        // Passive tactile sensor on channel 12
        cfg.channels[12] = ChannelConfig {
            id: 12,
            device: 1,
            group: 1,
            recipient: u8::MAX,
            role: JointRole::PassiveSensor,
            active_tool_qualification: ActiveToolConfig::NotActiveTool,
            vendor: VendorKind::GenericGrouped,
            desired_mode: ControlMode::Position,
            rest_position_rad: 0.0,
            rest_tolerance_rad: 0.0,
            rest_max_velocity_rad_s: None,
            supports_mit_damping: false,
            damping: DampingConfig::default(),
            thermal: ThermalConfig::default(),
            damiao_timeout_reg: None,
            max_capture_age_us: DEFAULT_CAPTURE_MAX_AGE_US,
        };

        let rec6_channels = ChannelMask::bit(6)
            .union(ChannelMask::bit(7))
            .union(ChannelMask::bit(8));
        let rec7_channels = ChannelMask::bit(9)
            .union(ChannelMask::bit(10))
            .union(ChannelMask::bit(11));
        cfg.recipients[6] = RecipientConfig {
            id: 6,
            device: 1,
            group: 1,
            channels: rec6_channels,
            grouped_packet_mask: hand_recipients,
            role: JointRole::Gripper(GripperReleasePolicy::ExplicitHand),
            vendor: VendorKind::GenericGrouped,
            rest_evidence_qualified: true,
        };
        cfg.recipients[7] = RecipientConfig {
            id: 7,
            device: 1,
            group: 1,
            channels: rec7_channels,
            grouped_packet_mask: hand_recipients,
            role: JointRole::Gripper(GripperReleasePolicy::ExplicitHand),
            vendor: VendorKind::GenericGrouped,
            rest_evidence_qualified: true,
        };

        cfg
    }
}

// =========================================================================
// Fault & Refusal Codes
// =========================================================================

/// Fault kinds tracked inside the reducer's per-group fault latch bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum TorqueFaultKind {
    EStop = 0,
    ThermalTrip = 1,
    BusLoss = 2,
    WatchdogTimeout = 3,
    EnableFailed = 4,
    DriverWriteFailed = 5,
    HardwareFault = 6,
    WorkerPanic = 7,
    ControlPanic = 8,
    Other = 9,
}

impl TorqueFaultKind {
    #[inline]
    pub const fn bit(self) -> u16 {
        1u16 << (self as u8)
    }
}

/// Bitmask of latched [`TorqueFaultKind`] values for one group.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct TorqueFaultMask(pub u16);

impl TorqueFaultMask {
    pub const EMPTY: Self = Self(0);

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    #[inline]
    pub const fn contains(self, kind: TorqueFaultKind) -> bool {
        (self.0 & kind.bit()) != 0
    }

    #[inline]
    pub fn insert(&mut self, kind: TorqueFaultKind) {
        self.0 |= kind.bit();
    }

    #[inline]
    pub fn remove(&mut self, kind: TorqueFaultKind) {
        self.0 &= !kind.bit();
    }

    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Machine-readable refusal / rejection codes emitted by the reducer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RefusalCode {
    TorqueOffWouldDrop,
    EnableFailed,
    RestGateTimeout,
    RestEvidenceUnqualified,
    HandReleaseNotAuthorized,
    ReleaseEvidenceTimeout,
    ActiveToolReleaseNotAuthorized,
    TimeoutMismatch,
    DisableInProgress,
    ModeUnverified,
    FaultLatched,
    ProtectiveLatched,
    ThermalActive,
    InternalFault,
    ControlPanicSafeHold,
    PreemptedByStop,
    PreemptedByProtective,
    ZeroWritesRefused,
    RecipientMaskInvalid,
    AlreadyEnabled,
    NotEnabled,
    GroupOutOfRange,
}

impl RefusalCode {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::TorqueOffWouldDrop => "TORQUE_OFF_WOULD_DROP",
            Self::EnableFailed => "ENABLE_FAILED",
            Self::RestGateTimeout => "REST_GATE_TIMEOUT",
            Self::RestEvidenceUnqualified => "REST_EVIDENCE_UNQUALIFIED",
            Self::HandReleaseNotAuthorized => "HAND_RELEASE_NOT_AUTHORIZED",
            Self::ReleaseEvidenceTimeout => "RELEASE_EVIDENCE_TIMEOUT",
            Self::ActiveToolReleaseNotAuthorized => "ACTIVE_TOOL_RELEASE_NOT_AUTHORIZED",
            Self::TimeoutMismatch => "TIMEOUT_MISMATCH",
            Self::DisableInProgress => "DISABLE_IN_PROGRESS",
            Self::ModeUnverified => "MODE_UNVERIFIED",
            Self::FaultLatched => "FAULT_LATCHED",
            Self::ProtectiveLatched => "PROTECTIVE_LATCHED",
            Self::ThermalActive => "THERMAL_ACTIVE",
            Self::InternalFault => "INTERNAL_FAULT",
            Self::ControlPanicSafeHold => "CONTROL_PANIC_SAFE_HOLD",
            Self::PreemptedByStop => "PREEMPTED_BY_STOP",
            Self::PreemptedByProtective => "PREEMPTED_BY_PROTECTIVE",
            Self::ZeroWritesRefused => "ZERO_WRITES_REFUSED",
            Self::RecipientMaskInvalid => "RECIPIENT_MASK_INVALID",
            Self::AlreadyEnabled => "ALREADY_ENABLED",
            Self::NotEnabled => "NOT_ENABLED",
            Self::GroupOutOfRange => "GROUP_OUT_OF_RANGE",
        }
    }
}

// =========================================================================
// Runtime State: Per-Joint, Per-Recipient, Per-Group, Per-Device, Root
// =========================================================================

/// Per-channel runtime state.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JointRuntime {
    /// Captured hold position (rad). Present iff a finite causal sample `<= max_capture_age_us`
    /// is active; anchored until feedback goes stale.
    pub captured_position_rad: Option<f32>,
    /// Monotonic timestamp (µs) when `captured_position_rad` was captured.
    pub captured_at_us: Option<u64>,
    /// Last accepted causal sample identifier.
    pub last_sample_id: Option<u64>,
    /// Timestamp (µs) of last accepted causal sample.
    pub last_sample_us: Option<u64>,
    /// Release id in which last accepted causal sample arrived.
    pub last_sample_release_id: Option<u64>,
    /// Last accepted causal measured position (rad).
    pub last_measured_pos_rad: Option<f32>,
    /// Last accepted causal measured velocity (rad/s).
    pub last_measured_vel_rad_s: Option<f32>,
    /// Latest causal release evidence (for gripper/hand channels; cleared on grasp/motion change).
    pub last_release_evidence: ReleaseEvidence,
    /// Last sample_id consumed by the active rest-gate K-window for this channel (prevents replay, B8).
    pub last_window_sample_id: Option<u64>,
    /// Last release_id consumed by the active rest-gate K-window for this channel.
    pub last_window_release_id: Option<u64>,
    /// Monotonic timestamp (µs) of the last motion write completion (preserved by stationary holds).
    pub last_motion_write_us: Option<u64>,
    /// Desired semantic control mode.
    pub desired_mode: ControlMode,
    /// Hardware-verified control mode (`None` while unverified / mode switch in flight).
    pub verified_mode: Option<ControlMode>,
    /// Readback of Damiao Reg 9 `TIMEOUT` register.
    pub verified_timeout_reg: Option<u16>,
    /// Device session id in which `verified_timeout_reg` was read back (invalidated on session loss, M11).
    pub verified_timeout_session: Option<u64>,
    /// Reported motor enable bit from feedback (telemetry ONLY — never mutates torque policy, B2).
    pub reported_enabled: Option<bool>,
    /// Last finite motor temperature (°C).
    pub last_temperature_c: Option<f32>,
    /// Monotonic timestamp (µs) of the last accepted thermal observation (rejects out-of-order cool samples, M6).
    pub last_temperature_us: Option<u64>,
    /// Linear thermal derating multiplier in `0.0..=1.0` (`1.0` when no derate configured).
    pub thermal_derate_scale: f32,
    /// Optional bounded grasp setpoint for a gripper/hand channel.
    pub grasp_setpoint_rad: Option<f32>,
    /// Current grasp force ceiling (raw); stop and `FailedHold` never raise this value.
    pub grasp_force_limit_raw: Option<u16>,
    /// Timestamp when `ActiveTool(BoundedRetention)` entered stopped retention.
    pub retention_started_us: Option<u64>,
}

impl Default for JointRuntime {
    fn default() -> Self {
        Self {
            captured_position_rad: None,
            captured_at_us: None,
            last_sample_id: None,
            last_sample_us: None,
            last_sample_release_id: None,
            last_measured_pos_rad: None,
            last_measured_vel_rad_s: None,
            last_release_evidence: ReleaseEvidence::Unknown,
            last_window_sample_id: None,
            last_window_release_id: None,
            last_motion_write_us: None,
            desired_mode: ControlMode::PositionVelocity,
            verified_mode: None,
            verified_timeout_reg: None,
            verified_timeout_session: None,
            reported_enabled: None,
            last_temperature_c: None,
            last_temperature_us: None,
            thermal_derate_scale: 1.0,
            grasp_setpoint_rad: None,
            grasp_force_limit_raw: None,
            retention_started_us: None,
        }
    }
}

/// One complete, qualified per-release snapshot recorded in a recipient's 3-slot sliding window (B8).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RestSnapshot {
    pub release_id: u64,
    /// Earliest causal sample timestamp among mapped channels in this release.
    pub min_ts_us: u64,
    /// Latest causal sample timestamp among mapped channels in this release.
    pub max_ts_us: u64,
}

/// Per-recipient rest-gate `K = 3` sliding window of complete qualified snapshots (B8).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RestWindow {
    /// Number of valid consecutive snapshots currently stored (`0..=3`).
    pub count: u8,
    /// Sliding window of up to 3 consecutive qualified snapshots (oldest at index 0).
    pub snapshots: [RestSnapshot; 3],
    /// Channels of this recipient that have contributed an unused agreeing sample in `pending_release_id`.
    pub pending_agreed_channels: ChannelMask,
    /// Release id currently accumulating channel agreement.
    pub pending_release_id: u64,
    /// Minimum causal timestamp among channels agreed so far in `pending_release_id`.
    pub pending_min_ts_us: u64,
    /// Maximum causal timestamp among channels agreed so far in `pending_release_id`.
    pub pending_max_ts_us: u64,
}

impl RestWindow {
    pub const fn reset(&mut self) {
        self.count = 0;
        self.snapshots = [RestSnapshot {
            release_id: 0,
            min_ts_us: 0,
            max_ts_us: 0,
        }; 3];
        self.pending_agreed_channels = ChannelMask::EMPTY;
        self.pending_release_id = 0;
        self.pending_min_ts_us = 0;
        self.pending_max_ts_us = 0;
    }

    /// Conservative span across the 3 qualified snapshots (`newest.min_ts - oldest.max_ts`).
    #[inline]
    pub const fn span_us(&self) -> u64 {
        if self.count >= 2 {
            let newest = self.snapshots[(self.count - 1) as usize];
            let oldest = self.snapshots[0];
            newest.min_ts_us.saturating_sub(oldest.max_ts_us)
        } else {
            0
        }
    }

    #[inline]
    pub const fn oldest_ts_us(&self) -> u64 {
        if self.count > 0 {
            self.snapshots[0].min_ts_us
        } else {
            0
        }
    }

    #[inline]
    pub const fn latest_ts_us(&self) -> u64 {
        if self.count > 0 {
            self.snapshots[(self.count - 1) as usize].min_ts_us
        } else {
            0
        }
    }

    #[inline]
    pub const fn latest_release_id(&self) -> u64 {
        if self.count > 0 {
            self.snapshots[(self.count - 1) as usize].release_id
        } else {
            0
        }
    }

    pub fn push_snapshot(&mut self, snap: RestSnapshot) {
        if self.count < 3 {
            self.snapshots[self.count as usize] = snap;
            self.count += 1;
        } else {
            self.snapshots[0] = self.snapshots[1];
            self.snapshots[1] = self.snapshots[2];
            self.snapshots[2] = snap;
        }
    }
}

/// Why a disable transaction was initiated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisableOrigin {
    ExplicitRequest,
    WorkerPanicParkAtRest,
    ProtectiveThermal,
    ProtectiveProcessExit,
    ProtectivePanicBusLoss,
}

impl DisableOrigin {
    #[inline]
    pub const fn is_protective(self) -> bool {
        matches!(
            self,
            Self::ProtectiveThermal | Self::ProtectiveProcessExit | Self::ProtectivePanicBusLoss
        )
    }

    #[inline]
    pub const fn kind(self) -> DisableKind {
        if self.is_protective() {
            DisableKind::Protective
        } else {
            DisableKind::Requested
        }
    }
}

/// Wire disable class emitted to the capability-gated emitter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisableKind {
    Requested,
    Protective,
}

/// Current phase of an in-flight [`DisableTransaction`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DisablePhase {
    /// Collecting rest / release evidence across all actual recipients within the 1 s deadline.
    AwaitRestGate,
    /// Gate passed in `release_id`; one generation- and dispatch-bound write authorized for this release.
    AuthorizedForRelease {
        release_id: u64,
        generation: u64,
        dispatch_id: u64,
    },
    /// Protective MIT kd-only damping phase preceding `DisableProtective` (per-channel durations, M8).
    ProtectiveDamping {
        started_us: u64,
        max_end_us: u64,
        dampable_mask: ChannelMask,
    },
    /// Protective disable ready to dispatch on the next serviced release (`N = 1`) using reserved budget.
    ProtectivePendingDispatch,
    /// First disable write was sent or possibly-sent (`TorqueIntent::Off` committed);
    /// running bounded repeats within the original deadline.
    Dispatched,
}

/// Live unresolved wire disable dispatch awaiting per-recipient outcomes (B1).
/// Preserved across cancellation so stale/delayed wire receipts are correlated exactly once
/// and enable/motion remain fenced until any possibly-sent write is reconciled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PendingDisableDispatch {
    pub dispatch_id: u64,
    pub tx_id: u64,
    pub kind: DisableKind,
    pub origin: DisableOrigin,
    pub release_id: u64,
    pub generation: u64,
    pub target_recipients: ChannelMask,
    pub remaining_recipients: ChannelMask,
    pub sent_mask: ChannelMask,
    pub ambiguous_mask: ChannelMask,
    pub failed_mask: ChannelMask,
    pub deferred_mask: ChannelMask,
    pub rejected_stale_mask: ChannelMask,
    /// False if the owning transaction was cancelled/expired while this wire dispatch was in flight.
    pub tx_still_active: bool,
}

/// Per-group active disable transaction (at most one active per group).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisableTransaction {
    pub tx_id: u64,
    pub origin: DisableOrigin,
    pub phase: DisablePhase,
    pub started_us: u64,
    pub deadline_us: u64,
    pub auth_generation: u64,
    /// Full expanded set of physical recipients addressed by this disable (must cover group, B5).
    pub recipient_mask: ChannelMask,
    /// Explicit operator authorization for releasing a new hand / bounded-retention tool.
    pub explicit_release_authorized: bool,
    /// True once at least one disable write was confirmed `Sent` or `Ambiguous`.
    pub committed_off: bool,
    /// Number of distinct wire attempts completed (for bounded Damiao 3 × 20 ms repeats).
    pub repeats_completed: u8,
    /// Earliest monotonic timestamp (µs) for the next bounded repeat.
    pub next_repeat_at_us: u64,
    /// Release id in which the latest disable dispatch was emitted.
    pub last_dispatched_release_id: Option<u64>,
    /// Cumulative per-recipient outcomes across the transaction.
    pub sent_mask: ChannelMask,
    pub ambiguous_mask: ChannelMask,
    pub failed_mask: ChannelMask,
    pub deferred_mask: ChannelMask,
}

/// Active enable handshake tracking for a group in `DriveState::ReadyToSwitchOn` (B2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct EnableTransaction {
    pub attempt_id: u64,
    pub started_us: u64,
    pub deadline_us: u64,
    pub generation: u64,
    pub target_channels: ChannelMask,
    pub confirmed_channels: ChannelMask,
}

/// Pending Tier-1 worker-panic park capability bound to one unique `park_id` (B3, M1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PendingPark {
    pub park_id: u64,
    pub generation: u64,
    /// Holds continue until the owner explicitly accepts the park capability (`Event::ParkAccepted`).
    pub accepted: bool,
}

/// Per-group runtime state.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct GroupRuntime {
    pub torque_intent: TorqueIntent,
    pub drive_state: DriveState,
    pub stop_generation: u64,
    pub enable_phase: EnablePhase,
    pub enable_tx: Option<EnableTransaction>,
    pub disable_tx: Option<DisableTransaction>,
    /// Unresolved wire disable dispatch (B1); fences enable/motion until reconciled.
    pub pending_disable_dispatch: Option<PendingDisableDispatch>,
    /// Physical recipients confirmed `Sent` off since the last `EnableStart` (B10: tracks partial coverage).
    pub physically_sent_off_mask: ChannelMask,
    pub faults: TorqueFaultMask,
    pub protective_latch: bool,
    /// Channels in this group currently above their thermal trip threshold.
    pub thermal_tripped_mask: ChannelMask,
    /// Release id when thermal trip first latched (for checking N=1 protective scheduling).
    pub thermal_trip_release_id: Option<u64>,
    /// Live Tier-1 worker-panic park capability (`None` when no park is pending, B3/M1).
    pub pending_park: Option<PendingPark>,
}

/// Per-device runtime state.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceRuntime {
    /// Connection/power session id; incremented on bus loss/reset to invalidate cached TIMEOUT readbacks (M11).
    pub session_id: u64,
    /// Live device-wide mode transition fence (B13); cleared only once every channel on this device is verified.
    pub mode_fence_active: bool,
    /// Feetech/serial poisoned line: samples/polls before this timestamp are rejected.
    pub poisoned_until_us: u64,
    /// Per-device continuous bus-loss timer while any group on this device is in Tier-2 `SafeHold` (B11).
    pub safe_hold_bus_loss_since_us: Option<u64>,
    /// Consecutive serviced releases where stopped-hold writes were skipped due to lock contention (M10).
    pub consecutive_contention_skips: u16,
    /// True if a stopped-hold `LockContended` was observed in the current release (M10).
    pub hold_contended_in_release: bool,
    /// True if a stopped-hold write succeeded (`Sent`/`Ambiguous`) in the current release (M10).
    pub hold_succeeded_in_release: bool,
    /// True once contention has escalated (escalates once per streak, resets on hold success, M10).
    pub contention_escalated: bool,
}

/// Complete, self-contained reducer state (stack POD, zero heap allocation).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TorqueState {
    pub config: TorqueConfig,
    pub now_us: u64,
    pub current_release_id: u64,
    /// Monotonic capability id allocator for disable dispatches, enable attempts, and park requests.
    pub next_capability_id: u64,
    pub worker_panic_latched: bool,
    pub worker_panic_thread_tag: Option<u16>,
    pub control_panic_latched: bool,
    pub process_death_recorded: bool,
    pub joints: [JointRuntime; MAX_CHANNELS],
    pub rest_windows: [RestWindow; MAX_CHANNELS],
    pub groups: [GroupRuntime; MAX_GROUPS],
    pub devices: [DeviceRuntime; MAX_DEVICES],
}

impl TorqueState {
    /// Construct a validated initial reducer state in `SwitchOnDisabled` / `TorqueIntent::Off`.
    pub fn new(config: TorqueConfig) -> Result<Self, ConfigError> {
        config.validate()?;
        let mut state = Self {
            config,
            now_us: 0,
            current_release_id: 0,
            next_capability_id: 1,
            worker_panic_latched: false,
            worker_panic_thread_tag: None,
            control_panic_latched: false,
            process_death_recorded: false,
            joints: [JointRuntime::default(); MAX_CHANNELS],
            rest_windows: [RestWindow::default(); MAX_CHANNELS],
            groups: [GroupRuntime::default(); MAX_GROUPS],
            devices: [DeviceRuntime::default(); MAX_DEVICES],
        };
        for c in 0..config.channel_count {
            state.joints[c as usize].desired_mode = config.channels[c as usize].desired_mode;
        }
        Ok(state)
    }

    /// Allocate the next monotonic capability / attempt / dispatch identifier.
    #[inline]
    pub(crate) fn alloc_capability_id(&mut self) -> u64 {
        let id = self.next_capability_id;
        self.next_capability_id = self.next_capability_id.saturating_add(1);
        id
    }

    /// Mask of configured logical channels.
    #[inline]
    pub const fn valid_channels(&self) -> ChannelMask {
        ChannelMask::from_count(self.config.channel_count)
    }

    /// Mask of configured physical recipients.
    #[inline]
    pub const fn valid_recipients(&self) -> ChannelMask {
        ChannelMask::from_count(self.config.recipient_count)
    }

    /// Mask of configured groups.
    #[inline]
    pub const fn valid_groups(&self) -> GroupMask {
        GroupMask::from_count(self.config.group_count)
    }

    /// Mask of configured devices.
    #[inline]
    pub const fn valid_devices(&self) -> DeviceMask {
        DeviceMask::from_count(self.config.device_count)
    }
}

// =========================================================================
// Events
// =========================================================================

/// Causal feedback sample for one logical channel (or whole-hand ACK).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CausalSample {
    pub channel: u8,
    pub sample_id: u64,
    /// Kernel RX or monotonic poll reply timestamp (µs) in the reducer clock domain.
    pub rx_timestamp_us: u64,
    /// Monotonic clock reading (µs) at ingestion.
    pub now_us: u64,
    /// Observed `|delta(CLOCK_REALTIME - CLOCK_MONOTONIC)|` (µs). `> 1000` invalidates rest window.
    pub clock_offset_drift_us: u64,
    pub position_rad: f32,
    pub velocity_rad_s: Option<f32>,
    pub temperature_c: Option<f32>,
    /// Reported motor enable state (telemetry ONLY — never mutates torque policy or arms enable).
    pub reported_enabled: Option<bool>,
    /// Correlated write-done timestamp (µs) for serial/grouped reply correlation (required for Feetech, B7).
    pub correlated_after_write_done_us: Option<u64>,
    /// Monotonic timestamp (µs) when the correlated poll was issued (required for Feetech, B7: must be
    /// strictly `> last_motion_write_us + 20 ms` and `>= poisoned_until_us`).
    #[serde(default)]
    pub poll_issued_us: Option<u64>,
    /// Causal release evidence for gripper / dexterous hand channels (must be affirmative on qualifying sample, B6).
    pub release_evidence: Option<ReleaseEvidence>,
    /// True for a whole-device/hand ACK that carries no position evidence (never creates capture or rest evidence).
    pub is_handshake_ack_only: bool,
}

/// Kind of wire write whose outcome is reported back from the emitter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WriteKind {
    Hold,
    Motion,
    Enable,
    DisableRequested,
    DisableProtective,
    Damp,
    Refresh,
    ModeSwitch,
}

/// Per-motor / per-recipient outcome returned by the emitter for one write pass.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotorWriteOutcome {
    /// Not part of this write pass.
    #[default]
    NotTargeted,
    /// Confirmed transmitted on the wire.
    Sent,
    /// Ambiguous transmit (e.g., transport error after enqueue; motor may have received frame).
    Ambiguous,
    /// Deferred by TX budget or transport backpressure (guaranteed 0 wire bytes).
    Deferred,
    /// Rejected by write-time recheck (generation, authorization, mode, freshness, deadline).
    RejectedStale,
    /// Skipped due to lock contention (feeds 25-release stopped-hold contention escalation).
    LockContended,
    /// Definitive pre-transmit failure (0 wire bytes).
    Failed,
}

/// Per-motor write result returned by the emitter to the reducer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WriteResult {
    pub group: u8,
    pub device: u8,
    pub kind: WriteKind,
    /// Capability / dispatch / attempt identity returned on the corresponding emitted command (B1, B2).
    #[serde(default)]
    pub dispatch_id: u64,
    pub generation: u64,
    pub write_done_us: u64,
    pub now_us: u64,
    /// Target mask (channels for Motion/Hold/Enable/Damp/Refresh, physical recipients for Disable).
    pub target_mask: ChannelMask,
    pub outcomes: [MotorWriteOutcome; MAX_CHANNELS],
}

impl WriteResult {
    pub fn uniform(
        group: u8,
        device: u8,
        kind: WriteKind,
        dispatch_id: u64,
        generation: u64,
        write_done_us: u64,
        now_us: u64,
        target_mask: ChannelMask,
        outcome: MotorWriteOutcome,
    ) -> Self {
        let mut outcomes = [MotorWriteOutcome::NotTargeted; MAX_CHANNELS];
        for i in target_mask.iter() {
            outcomes[i as usize] = outcome;
        }
        Self {
            group,
            device,
            kind,
            dispatch_id,
            generation,
            write_done_us,
            now_us,
            target_mask,
            outcomes,
        }
    }
}

/// Enable handshake progress reported by the driver/emitter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HandshakeOutcome {
    Pending,
    Confirmed,
    Failed,
    FaultReported { channel: u8 },
}

/// Stop / safety trigger kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StopKind {
    Stop,
    EStop,
    Fault(TorqueFaultKind),
    Watchdog,
    LeaseLoss,
}

/// FU-PANIC severity tier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PanicTier {
    /// Tier 1: panic outside `armd-control`; control thread healthy.
    WorkerPanic { thread_tag: u16 },
    /// Tier 2: panic caught on `armd-control`; enter minimal lock-free safe-hold loop.
    CaughtControlPanic,
    /// Tier 3: process death (SIGKILL / OOM / abort); hardware TIMEOUT fallback only.
    ProcessDeath,
}

/// All input events processed by [`crate::torque::reducer::step`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Event {
    /// Start of a new control release at monotonic timestamp `now_us`.
    Release { now_us: u64 },
    /// Causal sensor / actuator sample (or whole-device handshake ACK).
    CausalSample(CausalSample),
    /// Per-motor wire result returned by the emitter.
    WriteResult(WriteResult),
    /// Enable handshake readback / completion progress bound to `attempt_id` (B2).
    HandshakeResult {
        group: u8,
        #[serde(default)]
        attempt_id: u64,
        now_us: u64,
        confirmed_mask: ChannelMask,
        outcome: HandshakeOutcome,
    },
    /// Request to enable torque on `group`.
    EnableStart { group: u8, now_us: u64 },
    /// Request to disable torque on `group` (subject to role rest/release gate).
    DisableRequest {
        group: u8,
        now_us: u64,
        explicit_release_authorized: bool,
        recipient_mask_override: Option<ChannelMask>,
    },
    /// Stop, E-stop, fault, watchdog, or lease loss targeting `groups`.
    Stop {
        groups: GroupMask,
        kind: StopKind,
        now_us: u64,
    },
    /// Operator fault clear targeting `groups` (never dispatches a disable).
    FaultClear { groups: GroupMask, now_us: u64 },
    /// Thermal observation for `channel`.
    Thermal {
        channel: u8,
        temperature_c: f32,
        now_us: u64,
    },
    /// MIT / control-mode ownership change (clears verified mode until readback confirms).
    MitOwnershipChange {
        group: u8,
        channels: ChannelMask,
        target_mode: ControlMode,
        device_wide: bool,
        now_us: u64,
    },
    /// Hardware readback verified `channel` is in `mode`.
    ModeVerified {
        channel: u8,
        mode: ControlMode,
        now_us: u64,
    },
    /// Damiao Reg 9 `TIMEOUT` readback confirmed `timeout_reg` on `channel` in the current device session.
    TimeoutReadback {
        channel: u8,
        timeout_reg: u16,
        now_us: u64,
    },
    /// Motion admission / fence check for `group` and `channels`.
    MotionCommand {
        group: u8,
        channels: ChannelMask,
        now_us: u64,
    },
    /// Serial / bus timeout (poisons Feetech line for 20 ms of silence).
    SerialTimeout { device: u8, now_us: u64 },
    /// Update bounded grasp setpoint / force ceiling on a gripper channel while operational.
    GraspStateUpdate {
        channel: u8,
        setpoint_rad: f32,
        force_limit_raw: Option<u16>,
        now_us: u64,
    },
    /// Owner accepted the pending Tier-1 worker-panic park capability `park_id` (M1).
    ParkAccepted {
        group: u8,
        park_id: u64,
        now_us: u64,
    },
    /// Outcome of a pending park-to-rest trajectory bound to `park_id` (B3).
    ParkSettled {
        group: u8,
        #[serde(default)]
        park_id: u64,
        succeeded: bool,
        now_us: u64,
    },
    /// FU-PANIC event at one of the three crash tiers.
    Panic { tier: PanicTier, now_us: u64 },
    /// Clean process exit requesting `DisableProtective` (preceded by damping if configured).
    ProcessExit { now_us: u64 },
    /// Ordinary bus loss on `device` (grants NO protective authority outside `SafeHold`).
    BusLoss { device: u8, now_us: u64 },
}

// =========================================================================
// Actions (Zero-Alloc Fixed-Capacity Output)
// =========================================================================

/// Strict emitter scheduling priority (`Protective > StopHold > Motion > RefreshRoutine`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum ActionPriority {
    Protective = 0,
    StopHold = 1,
    Motion = 2,
    RefreshRoutine = 3,
}

/// Role-specific hold / safe-output directive emitted for one channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldKind {
    /// Load-bearing measured-position hold.
    PositionHold,
    /// Gripper / hand freeze: never opens jaws and never raises force.
    GripperFreeze { max_force_limit_raw: Option<u16> },
    /// Active tool safe stop output (e.g. screwdriver zero velocity; emitted even without position capture, M5).
    ActiveToolStop,
    /// Active tool bounded retention (e.g. vacuum suction held within remaining budget).
    ActiveToolRetain { remaining_us: u64 },
}

/// Per-channel hold / safe-output intent emitted on a release.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HoldCommand {
    pub channel: u8,
    pub group: u8,
    pub device: u8,
    pub position_rad: f32,
    pub captured_at_us: u64,
    pub last_sample_us: u64,
    /// True for position-anchored holds; false for position-independent semantic tool stops (M5).
    pub uses_position_capture: bool,
    pub verified_mode: Option<ControlMode>,
    pub hold_kind: HoldKind,
    /// Thermal derating multiplier (`0.0..=1.0`).
    pub torque_scale: f32,
    pub generation: u64,
}

/// Per-channel MIT kd-only damping intent emitted during `ProtectiveDamping` (per-channel end_us, M8).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DampCommand {
    pub channel: u8,
    pub group: u8,
    pub device: u8,
    pub kp: f32,
    pub kd: f32,
    pub max_torque_nm: f32,
    pub end_us: u64,
    pub generation: u64,
    pub use_reserved_budget: bool,
}

/// Per-group enable command carrying its unique `attempt_id` (B2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct EnableCommand {
    pub group: u8,
    pub attempt_id: u64,
    pub channels: ChannelMask,
    pub generation: u64,
    pub deadline_us: u64,
}

/// Per-group typed disable command carrying its unique `dispatch_id` and per-device deadlines (B1, M14).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisableCommand {
    pub group: u8,
    pub tx_id: u64,
    pub dispatch_id: u64,
    pub kind: DisableKind,
    pub origin: DisableOrigin,
    pub recipients: ChannelMask,
    /// Distinct physical packets required to address `recipients` (grouped packets counted once, M14).
    pub packet_count: u16,
    pub generation: u64,
    pub release_id: u64,
    /// Effective deadline capped by both the group deadline and the tightest addressed device deadline (M14).
    pub deadline_us: u64,
    pub repeat_index: u8,
    pub use_reserved_budget: bool,
}

/// Per-group disable transaction completion report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisableCompletion {
    pub group: u8,
    pub tx_id: u64,
    pub origin: DisableOrigin,
    pub recipient_mask: ChannelMask,
    pub sent_mask: ChannelMask,
    pub ambiguous_mask: ChannelMask,
    pub failed_mask: ChannelMask,
    pub unsent_mask: ChannelMask,
    pub committed_off: bool,
    pub partial_or_ambiguous: bool,
    pub timed_out: bool,
    pub repeats_completed: u8,
}

/// Refusal emitted for one group (or an out-of-range group).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RefusalEvent {
    pub group: u8,
    pub code: RefusalCode,
    pub channel: Option<u8>,
}

/// Authorized motion pass-through carrying authoritative per-channel thermal derate scales (M7).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MotionAllowed {
    pub group: u8,
    pub channels: ChannelMask,
    pub generation: u64,
    pub thermal_derate_scales: [f32; MAX_CHANNELS],
}

/// Tier-1 worker-panic park capability emitted to the owner (B3, M1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ParkRequest {
    pub group: u8,
    pub park_id: u64,
    pub generation: u64,
}

/// Per-bus TX budget summary computed for the emitter on each step (M14: packet-accurate accounting).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BusBudgetSummary {
    pub normal_quota: u16,
    pub reserved_safety_quota: u16,
    pub protective_demand: u16,
    pub stop_hold_demand: u16,
    pub refresh_demand: u16,
}

/// Zero-allocation telemetry snapshot attached to every [`ActionSet`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TelemetrySnapshot {
    pub now_us: u64,
    pub release_id: u64,
    pub group_states: [DriveState; MAX_GROUPS],
    pub group_intents: [TorqueIntent; MAX_GROUPS],
    pub reported_enabled_mask: ChannelMask,
    pub unheld_mask: ChannelMask,
    pub mode_unverified_mask: ChannelMask,
    pub thermal_tripped_mask: ChannelMask,
    pub thermal_derate_scales: [f32; MAX_CHANNELS],
    pub bus_budgets: [BusBudgetSummary; MAX_DEVICES],
}

impl Default for TelemetrySnapshot {
    fn default() -> Self {
        Self {
            now_us: 0,
            release_id: 0,
            group_states: [DriveState::SwitchOnDisabled; MAX_GROUPS],
            group_intents: [TorqueIntent::Off; MAX_GROUPS],
            reported_enabled_mask: ChannelMask::EMPTY,
            unheld_mask: ChannelMask::EMPTY,
            mode_unverified_mask: ChannelMask::EMPTY,
            thermal_tripped_mask: ChannelMask::EMPTY,
            thermal_derate_scales: [1.0; MAX_CHANNELS],
            bus_budgets: [BusBudgetSummary::default(); MAX_DEVICES],
        }
    }
}

/// Unified action view yielded in strict priority order by [`ActionSet::visit_by_priority`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Action {
    Damp(DampCommand),
    Disable(DisableCommand),
    Hold(HoldCommand),
    RequestPark(ParkRequest),
    Enable(EnableCommand),
    MotionAllowed(MotionAllowed),
    Refresh(ChannelMask),
    DisableComplete(DisableCompletion),
    Refusal(RefusalEvent),
    EscalateContention { device: u8, consecutive_skips: u16 },
}

impl Action {
    /// Emitter priority tier of this action.
    #[inline]
    pub const fn priority(&self) -> ActionPriority {
        match self {
            Self::Damp(_) => ActionPriority::Protective,
            Self::Disable(d) if matches!(d.kind, DisableKind::Protective) => {
                ActionPriority::Protective
            }
            Self::Disable(_) | Self::Hold(_) | Self::RequestPark(_) => ActionPriority::StopHold,
            Self::Enable(_) | Self::MotionAllowed(_) => ActionPriority::Motion,
            Self::Refresh(_)
            | Self::DisableComplete(_)
            | Self::Refusal(_)
            | Self::EscalateContention { .. } => ActionPriority::RefreshRoutine,
        }
    }
}

/// Fixed-capacity, zero-allocation output produced by [`crate::torque::reducer::step`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ActionSet {
    pub holds: [Option<HoldCommand>; MAX_CHANNELS],
    pub damps: [Option<DampCommand>; MAX_CHANNELS],
    pub enables: [Option<EnableCommand>; MAX_GROUPS],
    pub disables: [Option<DisableCommand>; MAX_GROUPS],
    pub disable_completions: [Option<DisableCompletion>; MAX_GROUPS],
    pub refusals: [Option<RefusalEvent>; MAX_GROUPS],
    /// Dedicated refusal slot for out-of-range group ids (never dropped by `visit_by_priority`, m2).
    pub invalid_group_refusal: Option<RefusalEvent>,
    pub motion_allowed: [Option<MotionAllowed>; MAX_GROUPS],
    pub park_requests: [Option<ParkRequest>; MAX_GROUPS],
    pub refresh_requests: ChannelMask,
    pub contention_escalations: DeviceMask,
    pub telemetry: TelemetrySnapshot,
}

impl Default for ActionSet {
    fn default() -> Self {
        Self {
            holds: [None; MAX_CHANNELS],
            damps: [None; MAX_CHANNELS],
            enables: [None; MAX_GROUPS],
            disables: [None; MAX_GROUPS],
            disable_completions: [None; MAX_GROUPS],
            refusals: [None; MAX_GROUPS],
            invalid_group_refusal: None,
            motion_allowed: [None; MAX_GROUPS],
            park_requests: [None; MAX_GROUPS],
            refresh_requests: ChannelMask::EMPTY,
            contention_escalations: DeviceMask::EMPTY,
            telemetry: TelemetrySnapshot::default(),
        }
    }
}

impl ActionSet {
    #[inline]
    pub fn hold_for(&self, ch: u8) -> Option<HoldCommand> {
        self.holds.get(ch as usize).copied().flatten()
    }

    #[inline]
    pub fn damp_for(&self, ch: u8) -> Option<DampCommand> {
        self.damps.get(ch as usize).copied().flatten()
    }

    #[inline]
    pub fn enable_for(&self, group: u8) -> Option<EnableCommand> {
        self.enables.get(group as usize).copied().flatten()
    }

    #[inline]
    pub fn disable_for(&self, group: u8) -> Option<DisableCommand> {
        self.disables.get(group as usize).copied().flatten()
    }

    #[inline]
    pub fn refusal_for(&self, group: u8) -> Option<RefusalEvent> {
        self.refusals
            .get(group as usize)
            .copied()
            .flatten()
            .or_else(|| self.invalid_group_refusal.filter(|r| r.group == group))
    }

    #[inline]
    pub fn completion_for(&self, group: u8) -> Option<DisableCompletion> {
        self.disable_completions.get(group as usize).copied().flatten()
    }

    #[inline]
    pub fn park_for(&self, group: u8) -> Option<ParkRequest> {
        self.park_requests.get(group as usize).copied().flatten()
    }

    #[inline]
    pub fn any_disable(&self) -> bool {
        self.disables.iter().any(Option::is_some)
    }

    #[inline]
    pub fn any_hold(&self) -> bool {
        self.holds.iter().any(Option::is_some)
    }

    /// Visit every emitted action in strict priority order:
    /// `Protective > StopHold > Motion > RefreshRoutine` (zero allocation).
    pub fn visit_by_priority(&self, state: &TorqueState, mut f: impl FnMut(Action)) {
        // Tier 0: Protective (protective damping + protective disable)
        for ch in 0..state.config.channel_count {
            if let Some(d) = self.damps[ch as usize] {
                f(Action::Damp(d));
            }
        }
        for g in 0..state.config.group_count {
            if let Some(d) = self.disables[g as usize] {
                if matches!(d.kind, DisableKind::Protective) {
                    f(Action::Disable(d));
                }
            }
        }
        // Tier 1: Stop / Hold (requested disable, holds, park requests)
        for g in 0..state.config.group_count {
            if let Some(d) = self.disables[g as usize] {
                if matches!(d.kind, DisableKind::Requested) {
                    f(Action::Disable(d));
                }
            }
        }
        for ch in 0..state.config.channel_count {
            if let Some(h) = self.holds[ch as usize] {
                f(Action::Hold(h));
            }
        }
        for g in 0..state.config.group_count {
            if let Some(p) = self.park_requests[g as usize] {
                f(Action::RequestPark(p));
            }
        }
        // Tier 2: Motion (enable, motion allowed)
        for g in 0..state.config.group_count {
            if let Some(e) = self.enables[g as usize] {
                f(Action::Enable(e));
            }
            if let Some(m) = self.motion_allowed[g as usize] {
                f(Action::MotionAllowed(m));
            }
        }
        // Tier 3: Refresh / Routine / Completion / Refusal / Contention
        if !self.refresh_requests.is_empty() {
            f(Action::Refresh(self.refresh_requests));
        }
        for g in 0..state.config.group_count {
            if let Some(c) = self.disable_completions[g as usize] {
                f(Action::DisableComplete(c));
            }
        }
        for r in self.refusals.into_iter().flatten() {
            f(Action::Refusal(r));
        }
        if let Some(r) = self.invalid_group_refusal {
            f(Action::Refusal(r));
        }
        for d in self.contention_escalations.iter() {
            if d < state.config.device_count {
                f(Action::EscalateContention {
                    device: d,
                    consecutive_skips: state.devices[d as usize].consecutive_contention_skips,
                });
            }
        }
    }
}
