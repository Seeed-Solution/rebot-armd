//! Unit tests for bounded panic disable and legacy torque timeout semantics.

use super::*;
use crate::driver::{ArmDriverError, TorqueOutcome, TorqueProgress};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};

impl ArmDriver for SelectiveMockDriver {
    crate::ext_accessors!(bus_recovery);
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn is_enabled(&self) -> bool { true }
    fn enable_state(&self) -> EnableState { EnableState::Unknown }
    fn is_feedback_valid(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> {
        Ok(Vec::new())
    }
    fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn begin_torque_budgeted(
        &mut self,
        _: bool,
        _: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> { vec![2] }
}

impl crate::driver::BusRecoveryExt for SelectiveMockDriver {
    fn panic_disable_step(
        &mut self,
        _first: bool,
        _deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.selective_calls.fetch_add(1, Ordering::SeqCst);
        if !self.sleep_on_step.is_zero() {
            std::thread::sleep(self.sleep_on_step);
        }
        Some(match &self.step_result {
            Ok(v) => Ok(v.clone()),
            Err(e) => Err(ArmDriverError::HardwareFault(e.to_string())),
        })
    }
}

impl ArmDriver for GenericMockDriver {
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn is_enabled(&self) -> bool { true }
    fn enable_state(&self) -> EnableState { EnableState::Unknown }
    fn is_feedback_valid(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> {
        Ok(Vec::new())
    }
    fn read_joint_states_budgeted(
        &mut self,
        out: &mut Vec<crate::model::JointState>,
        _deadline: Instant,
        _max_rx_frames: u16,
    ) -> Result<(u16, bool), ArmDriverError> {
        out.clear();
        Ok((0, false))
    }
    fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> {
        Ok(())
    }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn begin_torque_budgeted(
        &mut self,
        _: bool,
        _: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
        if !self.sleep_on_begin.is_zero() {
            std::thread::sleep(self.sleep_on_begin);
        }
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }
    fn unconfirmed_disable_joints(&self) -> Vec<usize> {
        vec![0]
    }
}

impl ArmDriver for ProbeThenGenericMockDriver {
    crate::ext_accessors!(bus_recovery);
    fn capabilities(&self) -> DriverCapabilities {
        DriverCapabilities {
            enable_feedback: true,
            ..DriverCapabilities::default()
        }
    }
    fn is_enabled(&self) -> bool { true }
    fn enable_state(&self) -> EnableState { EnableState::Unknown }
    fn is_feedback_valid(&self) -> bool { false }
    fn read_joint_states(&mut self) -> Result<Vec<crate::model::JointState>, ArmDriverError> { Ok(Vec::new()) }
    fn write_joint_targets(&mut self, _: &[crate::driver::JointTarget]) -> Result<(), ArmDriverError> { Ok(()) }
    fn set_torque(&mut self, _: bool) -> Result<(), ArmDriverError> { Ok(()) }
    fn begin_torque_budgeted(
        &mut self,
        _: bool,
        _: u16,
    ) -> Result<TorqueProgress, ArmDriverError> {
        self.generic_begin_calls.fetch_add(1, Ordering::SeqCst);
        Ok(TorqueProgress::Done(TorqueOutcome::Unconfirmed))
    }
}

impl crate::driver::BusRecoveryExt for ProbeThenGenericMockDriver {
    fn panic_disable_step(
        &mut self,
        _first: bool,
        _deadline: Instant,
    ) -> Option<Result<Vec<usize>, ArmDriverError>> {
        self.selective_calls.fetch_add(1, Ordering::SeqCst);
        if !self.sleep_on_probe.is_zero() {
            std::thread::sleep(self.sleep_on_probe);
        }
        None
    }
}

// scaffold-test-end
