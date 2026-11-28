//! Core supervisor constants, state structs, and calibration/RX support types.

#[allow(unused_imports)]
use super::*;

// scaffold-impl-0
impl UnloadSettleSample {
    pub fn from_slices(t: f64, q: &[f32], v: &[f32]) -> Self {
        let dof = q.len().min(v.len()).min(crate::model::MAX_MOTION_DOF);
        let mut positions = [f32::NAN; crate::model::MAX_MOTION_DOF];
        let mut velocities = [f32::NAN; crate::model::MAX_MOTION_DOF];
        positions[..dof].copy_from_slice(&q[..dof]);
        velocities[..dof].copy_from_slice(&v[..dof]);
        Self { t, dof, positions, velocities }
    }
}

// scaffold-impl-1
impl UnloadSettle {
    pub fn with_capacity(cap: usize) -> Self {
        Self {
            samples: std::collections::VecDeque::with_capacity(cap),
            ..Self::default()
        }
    }
}

// scaffold-impl-2
impl TickOutput {
    /// True when the active J7 position-force gripper was stale (> 100 ms) during this pass
    /// (receiving a freeze frame instead of any new target); prevents J7-dependent action
    /// completions from committing on that tick.
    #[inline]
    pub(crate) fn j7_stale(&self) -> bool {
        matches!(
            self,
            Self::ActionDeadlineRefused { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Complete { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Deferred { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Expired { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
                | Self::Error { pf_status: crate::driver::PosForceTxStatus::Stale { .. }, .. }
        )
    }
}

// scaffold-impl-3
impl ZeroCheckPersistContext {
    /// Captures all heap-backed calibration persistence inputs off the RT completion path.
    pub fn from_inner(
        inner: &SupervisorInner,
        operator_auth: Option<(crate::auth::OperatorAuth, Option<String>)>,
    ) -> Self {
        let path = inner
            .gripper_calib_path
            .clone()
            .unwrap_or_else(crate::gripper_calib::path_from_env);
        let gripper_limit = inner
            .config
            .gripper_index()
            .and_then(|idx| inner.config.limits.get(idx))
            .cloned();
        Self {
            path,
            jaw_opening: inner.gripper_calib_jaw_opening.clone(),
            stale: inner.gripper_calib_stale.clone(),
            tool_id: inner.config.active_tool.id.clone(),
            tool_config_digest: inner.config.active_tool.config_digest.clone(),
            active_tool_is_stock: inner.config.active_tool.is_stock,
            base_gripper_policy: inner
                .base_gripper_policy
                .clone()
                .or_else(|| inner.config.gripper_policy.clone()),
            gripper_limit,
            operator_auth,
            sat_msg_buf: String::with_capacity(64),
        }
    }

    /// Serializes `GripperCalibFile` off the control thread into a [`crate::persist::PendingWrite`].
    pub fn serialize_write(&self, offset_rad: f32) -> crate::persist::PendingWrite {
        let file = crate::gripper_calib::GripperCalibFile {
            offset_rad: Some(offset_rad),
            jaw_opening: self.jaw_opening.clone(),
            ts: Some(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map_or(0, |d| d.as_secs()),
            ),
            stale: self.stale.clone(),
            tool_id: Some(self.tool_id.clone()),
            tool_config_digest: Some(self.tool_config_digest.clone()),
        };
        let text = serde_json::to_string_pretty(&file).unwrap_or_default();
        crate::persist::PendingWrite::new(self.path.clone(), text)
    }
}
