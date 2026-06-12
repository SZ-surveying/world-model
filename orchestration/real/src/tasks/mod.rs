mod motor_debug;
mod registry;

use std::path::PathBuf;

use anyhow::Result;

use crate::config::{ProjectConfig, TaskConfig};

pub use motor_debug::build_plan as build_motor_debug_plan;
pub use motor_debug::{
    ARDUCOPTER_GUIDED_MODE_ID, CommandAckEvaluation, GuidedModeEvaluation,
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED, MAV_RESULT_FAILED, MavlinkStatusText,
    MotorDebugCommandLong, MotorDebugCommandPlan, MotorDebugOverrides, MotorDebugTask,
    OperatorConfirmations, REQUIRED_GUIDED_MODE_NAME, TASK_RESULT_SCHEMA_VERSION,
    arm_disarm_command, command_rejection_blockers, evaluate_command_ack, evaluate_guided_mode,
    motor_debug_command_plan,
};
pub use registry::Registry;

#[derive(Debug, Clone, Default)]
pub struct RunOptions {
    pub dry_run: bool,
    pub motor_debug: MotorDebugOverrides,
    pub operator_confirmations: OperatorConfirmations,
    pub artifact_dir: Option<PathBuf>,
    pub summary_path: Option<PathBuf>,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct ConfiguredTask {
    pub id: String,
    pub family: String,
    pub description: String,
    pub capabilities: Vec<String>,
}

#[async_trait::async_trait]
pub trait RealTask: Send + Sync {
    fn id(&self) -> &'static str;

    async fn run(
        &self,
        project: &ProjectConfig,
        config: &TaskConfig,
        options: RunOptions,
    ) -> Result<()>;
}
