//! Damiao bounded target batch TX and target-frame validation helpers.

#[allow(unused_imports)]
use std::time::{Duration, Instant};

#[allow(unused_imports)]
use crate::driver::*;
#[allow(unused_imports)]
use crate::driver::can_servo::*;
#[allow(unused_imports)]
use crate::protocol::damiao;
#[allow(unused_imports)]
use crate::transport::{CanFrame, CanTransport};

impl<C: CanTransport> DamiaoArmDriver<C> {
    pub(in crate::driver::can_servo) fn build_pending_target_frame(&self, entry: &PendingTxFrame) -> Option<CanFrame> {
        if self.is_axis_mode_uncertain(entry.joint) {
            return None;
        }
        let active = self.joint_modes.get(entry.joint).copied().flatten();
        match entry.cmd {
            PendingTargetCmd::PosVel { position } => {
                if matches!(active, Some(DamiaoControlMode::Mit | DamiaoControlMode::PosForce)) {
                    return None;
                }
                Some(self.pos_vel_frame(entry.joint, position))
            }
            PendingTargetCmd::Mit { position, velocity, kp, kd, torque } => {
                if matches!(active, Some(DamiaoControlMode::PosVel | DamiaoControlMode::PosForce)) {
                    return None;
                }
                if !self.is_mit_hold_eligible(entry.joint) {
                    return None;
                }
                let target = JointTarget { position, velocity, kp, kd, torque };
                Some(self.mit_frame(entry.joint, &target))
            }
            PendingTargetCmd::RollbackHold { .. } | PendingTargetCmd::MeasuredHold { .. } => None,
        }
    }

    /// The frames of non-PosForce joint `i`'s target in its active/confirmed mode (one, or both
    /// kinds while its mode is unconfirmed), on the stack: the control tick's write does not
    /// allocate. Active `PosForce` joints are NEVER built here — they are emitted exclusively by
    /// the single J7 emitter ([`Self::emit_pos_force_j7_impl`]).
    pub(in crate::driver::can_servo) fn target_frames_checked(
        &self,
        i: usize,
        target: &JointTarget,
    ) -> Result<([Option<CanFrame>; 2], Option<usize>), ArmDriverError> {
        if self.is_axis_mode_uncertain(i) {
            return Ok(([None, None], None));
        }
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => Ok(([Some(self.pos_vel_frame(i, target.position)), None], None)),
            Some(DamiaoControlMode::Mit) => {
                if !self.is_mit_hold_eligible(i) {
                    return Ok(([None, None], None));
                }
                Ok(([Some(self.mit_frame(i, target)), None], None))
            }
            Some(DamiaoControlMode::PosForce) => Ok(([None, None], Some(i))),
            None if self.base_mode(i) == DamiaoControlMode::PosForce => {
                Err(ArmDriverError::PosForceModeUnconfirmed { joint: i })
            }
            None => Ok(([Some(self.pos_vel_frame(i, target.position)), Some(self.mit_frame(i, target))], None)),
        }
    }

    /// Unencoded target commands for non-PosForce arm joint `i` (stack-allocated, zero heap alloc)
    /// staged into [`PendingTxQueue`]. Active `PosForce` joints return `[None, None]` because
    /// they are emitted exclusively by the single J7 emitter ([`Self::emit_pos_force_j7_impl`]).
    pub(in crate::driver::can_servo) fn pending_target_cmds_checked(
        &self,
        i: usize,
        target: &JointTarget,
    ) -> Result<[Option<(u32, PendingTargetCmd)>; 2], ArmDriverError> {
        if self.is_axis_mode_uncertain(i) {
            return Ok([None, None]);
        }
        let id = self.motor_ids[i];
        match self.joint_modes[i] {
            Some(DamiaoControlMode::PosVel) => Ok([
                Some((0x100 + u32::from(id), PendingTargetCmd::PosVel { position: target.position })),
                None,
            ]),
            Some(DamiaoControlMode::Mit) => {
                if !self.is_mit_hold_eligible(i) {
                    return Ok([None, None]);
                }
                Ok([
                    Some((
                        u32::from(id),
                        PendingTargetCmd::Mit {
                            position: target.position,
                            velocity: target.velocity,
                            kp: target.kp,
                            kd: target.kd,
                            torque: target.torque,
                        },
                    )),
                    None,
                ])
            }
            Some(DamiaoControlMode::PosForce) => Ok([None, None]),
            None if self.base_mode(i) == DamiaoControlMode::PosForce => {
                Err(ArmDriverError::PosForceModeUnconfirmed { joint: i })
            }
            None => Ok([
                Some((0x100 + u32::from(id), PendingTargetCmd::PosVel { position: target.position })),
                Some((
                    u32::from(id),
                    PendingTargetCmd::Mit {
                        position: target.position,
                        velocity: target.velocity,
                        kp: target.kp,
                        kd: target.kd,
                        torque: target.torque,
                    },
                )),
            ]),
        }
    }

    /// A batch is refused before its first frame when the position-force gripper's mode is not
    /// confirmed, its target position or velocity is non-finite, or its requested speed encodes
    /// to 0 on the wire. When `allow_stale_pf` is `false` (MIT-fenced paths), stale J7 feedback
    /// also refuses the entire batch before frame 0; unfenced paths (`allow_stale_pf = true`)
    /// route stale J7 through [`Self::check_pos_force_freshness_for_tx`] so J1–J6 continue.
    pub(in crate::driver::can_servo) fn target_frames_refused(&self, targets: Option<&[JointTarget]>, allow_stale_pf: bool) -> Option<ArmDriverError> {
        let pf = self.pos_force.as_ref()?;
        let mode = self.joint_modes.get(pf.joint).copied().flatten();
        let allowed = (mode == Some(DamiaoControlMode::PosForce) && self.pos_force_verified)
            || mode == Some(DamiaoControlMode::Mit);
        if !allowed {
            return Some(ArmDriverError::PosForceModeUnconfirmed { joint: pf.joint });
        }
        if mode == Some(DamiaoControlMode::PosForce) {
            if let Some(target) = targets.and_then(|ts| ts.get(pf.joint)) {
                if let Err(err) = self.validate_pos_force_target_static(pf.joint, target) {
                    return Some(err);
                }
            }
            if !allow_stale_pf && !self.pos_force_feedback_valid(pf.joint, self.now()) {
                return Some(ArmDriverError::PosForceFeedbackStale { joint: pf.joint });
            }
        }
        None
    }

    pub(in crate::driver::can_servo) fn refusal_to_driver_err(r: TxRefusal) -> ArmDriverError {
        match r {
            TxRefusal::DeadlinePassed => ArmDriverError::Transport(TransportError::Timeout),
            TxRefusal::Deferred => ArmDriverError::Transport(TransportError::WouldBlock),
            TxRefusal::StopInhibited | TxRefusal::StopStampChanged => ArmDriverError::EstopInhibited,
            TxRefusal::TorqueOffWouldDrop | TxRefusal::DisableWaitEvidence => {
                ArmDriverError::TorqueOffWouldDrop
            }
            TxRefusal::DisableRequestExpired => ArmDriverError::DisableRequestExpired,
            TxRefusal::LeaseFenced => ArmDriverError::LeaseFenced,
            TxRefusal::EnableAuthorityExpired => ArmDriverError::EnableAuthorityExpired,
            TxRefusal::MitFenceTripped(reason) => ArmDriverError::MitFenceTripped(reason),
            TxRefusal::PosForceFeedbackExpired(joint) => {
                ArmDriverError::PosForceFeedbackStaleBeforeTransmit { joint }
            }
            TxRefusal::BuildRefused => ArmDriverError::HardwareFault("frame build refused (stale or invalid state)".into()),
            TxRefusal::Transport(e) => e.into(),
        }
    }

}
