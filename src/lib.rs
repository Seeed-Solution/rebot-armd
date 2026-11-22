#[cfg(not(target_arch = "wasm32"))] pub mod api;
#[cfg(not(target_arch = "wasm32"))] pub mod auth;
#[cfg(not(target_arch = "wasm32"))] pub mod calibration;
#[cfg(not(target_arch = "wasm32"))] pub mod calib_hover;
#[cfg(not(target_arch = "wasm32"))] pub mod calibration_sessions;
pub mod backlash;
pub mod chain;
pub mod collision;
#[cfg(not(target_arch = "wasm32"))] pub mod depth_measure;
#[cfg(not(target_arch = "wasm32"))] pub mod diagnostics;
#[cfg(not(target_arch = "wasm32"))] pub mod driver;
pub mod dynamics;
#[cfg(not(target_arch = "wasm32"))] pub mod estimate;
#[cfg(not(target_arch = "wasm32"))] pub mod grasp;
pub mod gravity;
#[cfg(not(target_arch = "wasm32"))] pub mod gravity_comp;
#[cfg(not(target_arch = "wasm32"))] pub mod gripper_calib;
#[cfg(not(target_arch = "wasm32"))] pub mod keepout;
#[cfg(not(target_arch = "wasm32"))] pub mod joint_zero;
pub mod kinematics;
#[cfg(not(target_arch = "wasm32"))] pub mod macro_action;
#[cfg(not(target_arch = "wasm32"))] pub mod measure_multiview;
#[cfg(not(target_arch = "wasm32"))] pub mod motor_zero;
#[cfg(not(target_arch = "wasm32"))] pub use ::mview_stereo;
pub mod mjcf_compose;
pub mod model;
pub mod motion;
#[cfg(not(target_arch = "wasm32"))] pub mod persist;
#[cfg(not(target_arch = "wasm32"))] pub mod place;
#[cfg(all(feature = "policy-runtime", not(target_arch = "wasm32")))] pub mod policy_runtime;
#[cfg(not(target_arch = "wasm32"))] pub mod poses;
pub mod power;
pub mod power_plan;
#[cfg(not(target_arch = "wasm32"))] pub mod protocol;
#[cfg(not(target_arch = "wasm32"))] pub mod recording;
#[cfg(not(target_arch = "wasm32"))] pub mod replay;
pub mod safety;
#[cfg(not(target_arch = "wasm32"))] pub mod safety_describe;
#[cfg(not(target_arch = "wasm32"))] pub mod sequences;
#[cfg(not(target_arch = "wasm32"))] pub mod sim_driver;
#[cfg(not(target_arch = "wasm32"))] pub mod supervisor;
#[cfg(not(target_arch = "wasm32"))] pub mod teach;
pub mod torque_guard;
pub mod tracking;
pub mod tool;
pub mod trajectory;
pub mod urdf;
#[cfg(not(target_arch = "wasm32"))] pub mod transport;
pub mod wasm_api;

#[cfg(not(target_arch = "wasm32"))] pub use api::{create_router, create_router_with_auth};
#[cfg(not(target_arch = "wasm32"))] pub use auth::OperatorAuth;
#[cfg(not(target_arch = "wasm32"))] pub use driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, FashionStarArmDriver, FeetechArmDriver,
    JointMode, JointTarget, RobStrideArmDriver,
};
pub use gravity::{compute_gravity_torques, GravityError};
#[cfg(not(target_arch = "wasm32"))]
pub use gravity::verify_urdf_sha256;
pub use model::{ArmConfig, ArmObservation, ARM_PROFILE_VERSION, JointLimit, JointState, ObservationSchema};
#[cfg(not(target_arch = "wasm32"))] pub use recording::{
    lerobot_arrow_schema, read_frames_from_parquet, write_frames_to_parquet, EpisodeRecorder,
    EpisodeState, RecordFrame,
};
pub use safety::{FaultCode, FaultInfo, FaultReason, SafetyError, SafetyManager, SafetyState, StopPolicy};
#[cfg(not(target_arch = "wasm32"))] pub use supervisor::{ArmError, ArmSupervisor, PreemptError, SupervisorInner};
pub use trajectory::{
    MultiJointQuinticTrajectory, QuinticPolynomial, TrajectoryError, TrajectoryPlanner,
    TrajectoryPoint,
};
#[cfg(not(target_arch = "wasm32"))] pub use transport::{
    AsyncTransport, CanFrame, CanTransport, DriverClock, MockCanTransport, MockClock, MockTransport,
    SerialTransport, SerialTransportConfig, SlcanCodec, SlcanTransport, Transport, TransportError,
};

#[cfg(all(not(target_arch = "wasm32"), target_os = "linux"))]
pub use transport::SocketCanTransport;
