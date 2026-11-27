//! Damiao CAN protocol facade (re-exported from `crate::driver::can_servo::damiao`).

pub use crate::driver::can_servo::damiao::codec::*;
pub(crate) use crate::driver::can_servo::damiao::codec::build_pos_force_frame;
pub use crate::driver::can_servo::damiao::table::*;
