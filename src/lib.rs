pub mod api;
pub mod driver;
pub mod gravity;
pub mod kinematics;
pub mod model;
pub mod motion;
pub mod protocol;
pub mod recording;
pub mod safety;
pub mod supervisor;
pub mod trajectory;
pub mod transport;

pub use api::create_router;
pub use driver::{
    ArmDriver, ArmDriverError, DamiaoArmDriver, FashionStarArmDriver, FeetechArmDriver,
    JointTarget, RobStrideArmDriver,
};
pub use gravity::{compute_gravity_torques, verify_urdf_sha256, GravityError};
pub use model::{ArmConfig, ArmObservation, JointLimit, JointState, ObservationSchema};
pub use recording::{
    lerobot_arrow_schema, read_frames_from_parquet, write_frames_to_parquet, EpisodeRecorder,
    EpisodeState, RecordFrame,
};
pub use safety::{FaultReason, SafetyError, SafetyManager, SafetyState, StopPolicy};
pub use supervisor::{ArmError, ArmSupervisor, PreemptError, SupervisorInner};
pub use trajectory::{
    MultiJointQuinticTrajectory, QuinticPolynomial, TrajectoryError, TrajectoryPlanner,
    TrajectoryPoint,
};
pub use transport::{
    AsyncTransport, CanFrame, CanTransport, MockCanTransport, MockTransport, SerialTransport,
    SerialTransportConfig, SlcanCodec, SlcanTransport, Transport, TransportError,
};

#[cfg(target_os = "linux")]
pub use transport::SocketCanTransport;
