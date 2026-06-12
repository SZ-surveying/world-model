use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use serde::Serialize;
use serde_json::{Value, json};
use time::OffsetDateTime;
use tracing::{info, instrument, warn};

use crate::config::{ProjectConfig, TaskConfig};
use crate::runtime::{MotorDebugRuntimeReport, MotorDebugRuntimeRequest, real_motor_debug_runtime};
use crate::tasks::{RealTask, RunOptions};
use crate::ui::{print_key_value, print_title};

pub const TASK_RESULT_SCHEMA_VERSION: &str = "navlab.runtime.task_result.v1";
pub const REQUIRED_GUIDED_MODE_NAME: &str = "GUIDED";
pub const ARDUCOPTER_GUIDED_MODE_ID: u32 = 4;
pub const MAV_CMD_COMPONENT_ARM_DISARM: u32 = 400;
pub const MAV_RESULT_ACCEPTED: u32 = 0;
pub const MAV_RESULT_FAILED: u32 = 4;

#[derive(Debug, Clone, Default)]
pub struct MotorDebugOverrides {
    pub motor_percent: Option<f64>,
    pub motor_sec: Option<f64>,
    pub motor_count: Option<u32>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct OperatorConfirmations {
    pub manual_takeover: bool,
    pub kill_switch: bool,
    pub safe_area: bool,
    pub no_props: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct OperatorSafetyEvaluation {
    pub ok: bool,
    pub blocked: bool,
    pub manual_takeover_confirmed: bool,
    pub kill_switch_confirmed: bool,
    pub safe_area_confirmed: bool,
    pub no_props_confirmed: bool,
    pub blockers: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct MotorDebugPlan {
    pub task_id: String,
    pub motor_percent: f64,
    pub motor_sec: f64,
    pub motor_count: u32,
    pub safety: MotorDebugSafety,
    pub claim: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct MotorDebugRuntimeSummary {
    pub schema_version: String,
    pub ok: bool,
    pub blocked: bool,
    pub blockers: Vec<String>,
    pub task: String,
    pub claim: String,
    pub no_takeoff: bool,
    pub requires_no_props: bool,
    pub guided_mode_required: bool,
    pub required_mode: String,
    pub spin_mode: String,
    pub throttle_command_claim: String,
    pub motor_percent: f64,
    pub motor_sec: f64,
    pub motor_count: u32,
    pub steps: Vec<Value>,
    pub shutdown: String,
    pub landing_claim: String,
    pub serial: String,
    pub connection_endpoint: String,
    pub baud: u32,
    pub arm_claim: String,
    pub takeoff_claim: String,
    pub guided_mode_claim: String,
    pub shutdown_claim: String,
    pub guided_mode: Value,
    pub command_plan: MotorDebugCommandPlan,
    pub acks: Vec<Value>,
    pub runtime_report: Option<MotorDebugRuntimeReport>,
}

#[derive(Debug, Clone, Serialize)]
pub struct MotorDebugSafety {
    pub confirm_manual_takeover: bool,
    pub confirm_kill_switch: bool,
    pub confirm_safe_area: bool,
    pub confirm_no_props: bool,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct GuidedModeEvaluation {
    pub ok: bool,
    pub mode_name: String,
    pub mode_id: u32,
    pub mode_mapping_has_mode: bool,
    pub mode_mapping_mode_id: Option<u32>,
    pub observed_mode_id: Option<u32>,
    pub blockers: Vec<String>,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct MotorDebugCommandLong {
    pub target_system: u8,
    pub target_component: u8,
    pub command: u32,
    pub confirmation: u8,
    pub param1: f64,
    pub param2: f64,
    pub param3: f64,
    pub param4: f64,
    pub param5: f64,
    pub param6: f64,
    pub param7: f64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct MotorDebugCommandPlan {
    pub guided_mode_id: u32,
    pub arm_command: MotorDebugCommandLong,
    pub disarm_command: MotorDebugCommandLong,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct MavlinkStatusText {
    pub severity: u8,
    pub text: String,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct CommandAckEvaluation {
    pub command: u32,
    pub result: u32,
    pub result_name: String,
    pub accepted: bool,
    pub result_param2: u32,
    pub status_text: Vec<MavlinkStatusText>,
}

pub struct MotorDebugTask;

#[async_trait::async_trait]
impl RealTask for MotorDebugTask {
    fn id(&self) -> &'static str {
        "motor-debug"
    }

    #[instrument(skip(self, project, config, options))]
    async fn run(
        &self,
        project: &ProjectConfig,
        config: &TaskConfig,
        options: RunOptions,
    ) -> Result<()> {
        if config.family != "real" {
            bail!(
                "motor-debug config family must be real, got {}",
                config.family
            );
        }
        let plan = build_plan(config, options.motor_debug.clone())?;
        info!(
            task_id = plan.task_id,
            motor_percent = plan.motor_percent,
            motor_sec = plan.motor_sec,
            motor_count = plan.motor_count,
            dry_run = options.dry_run,
            "prepared motor-debug task"
        );
        print_title("NavLab Real Motor Debug");
        print_key_value("runtime_mode", &project.runtime.mode);
        print_key_value("backend", &project.runtime.backend);
        print_key_value("dry_run", &options.dry_run.to_string());
        println!("{}", serde_json::to_string_pretty(&plan)?);
        if options.dry_run {
            return Ok(());
        }
        let safety = evaluate_operator_safety(&plan, &options.operator_confirmations);
        println!("{}", serde_json::to_string_pretty(&safety)?);
        if safety.blocked {
            bail!(
                "real motor-debug blocked before motor test: {}",
                safety.blockers.join(", ")
            );
        }
        let summary_path = resolve_summary_path(project, &options);
        let runtime_request = MotorDebugRuntimeRequest::new(
            mavlink_router_endpoint(project),
            plan.motor_percent,
            plan.motor_sec,
        );
        let runtime = real_motor_debug_runtime(&runtime_request);
        let summary = match runtime {
            Ok(report) => build_runtime_summary(project, &plan, report),
            Err(error) => build_runtime_error_summary(project, &plan, &error.to_string()),
        };
        write_runtime_summary(&summary_path, &summary)?;
        println!("{}", serde_json::to_string_pretty(&summary)?);
        if summary.ok {
            Ok(())
        } else {
            warn!("live motor-debug blocked");
            bail!(
                "live motor-debug blocked: {}; summary written to {}",
                summary.blockers.join(", "),
                summary_path.display()
            )
        }
    }
}

pub fn build_plan(config: &TaskConfig, overrides: MotorDebugOverrides) -> Result<MotorDebugPlan> {
    let motor_percent = overrides
        .motor_percent
        .unwrap_or_else(|| number_or_default(config.task.get("motor_percent"), 5.0));
    let motor_sec = overrides
        .motor_sec
        .unwrap_or_else(|| number_or_default(config.task.get("motor_sec"), 5.0));
    let motor_count = overrides
        .motor_count
        .unwrap_or_else(|| unsigned_or_default(config.task.get("motor_count"), 4));
    if motor_percent <= 0.0 {
        bail!("motor_percent must be positive");
    }
    if motor_sec <= 0.0 {
        bail!("motor_sec must be positive");
    }
    if motor_count == 0 {
        bail!("motor_count must be positive");
    }
    Ok(MotorDebugPlan {
        task_id: config.id.clone(),
        motor_percent,
        motor_sec,
        motor_count,
        safety: MotorDebugSafety {
            confirm_manual_takeover: bool_or_default(
                config.safety.get("confirm_manual_takeover"),
                true,
            ),
            confirm_kill_switch: bool_or_default(config.safety.get("confirm_kill_switch"), true),
            confirm_safe_area: bool_or_default(config.safety.get("confirm_safe_area"), true),
            confirm_no_props: bool_or_default(config.safety.get("confirm_no_props"), true),
        },
        claim: "dry_run_plan".to_string(),
    })
}

pub fn evaluate_operator_safety(
    plan: &MotorDebugPlan,
    confirmations: &OperatorConfirmations,
) -> OperatorSafetyEvaluation {
    let mut blockers = Vec::new();
    if plan.safety.confirm_manual_takeover && !confirmations.manual_takeover {
        blockers.push("operator_manual_takeover_not_confirmed".to_string());
    }
    if plan.safety.confirm_kill_switch && !confirmations.kill_switch {
        blockers.push("operator_kill_switch_not_confirmed".to_string());
    }
    if plan.safety.confirm_safe_area && !confirmations.safe_area {
        blockers.push("operator_safe_area_not_confirmed".to_string());
    }
    if plan.safety.confirm_no_props && !confirmations.no_props {
        blockers.push("operator_no_props_not_confirmed".to_string());
    }
    OperatorSafetyEvaluation {
        ok: blockers.is_empty(),
        blocked: !blockers.is_empty(),
        manual_takeover_confirmed: confirmations.manual_takeover,
        kill_switch_confirmed: confirmations.kill_switch,
        safe_area_confirmed: confirmations.safe_area,
        no_props_confirmed: confirmations.no_props,
        blockers,
    }
}

pub fn mavlink_router_endpoint(project: &ProjectConfig) -> String {
    let endpoint = project.prepare.mavlink_router_local_endpoint.trim();
    if ["udp:", "udpin:", "udpout:", "tcp:"]
        .iter()
        .any(|prefix| endpoint.starts_with(prefix))
    {
        return endpoint.to_string();
    }
    let (host, port) = endpoint.split_once(':').unwrap_or((endpoint, ""));
    format!(
        "udpin:{}:{}",
        if host.is_empty() { "127.0.0.1" } else { host },
        if port.is_empty() { "14550" } else { port }
    )
}

pub fn build_runtime_not_migrated_summary(
    project: &ProjectConfig,
    plan: &MotorDebugPlan,
) -> MotorDebugRuntimeSummary {
    MotorDebugRuntimeSummary {
        schema_version: TASK_RESULT_SCHEMA_VERSION.to_string(),
        ok: false,
        blocked: true,
        blockers: vec!["motor_debug_rust_mavlink_runtime_not_migrated".to_string()],
        task: "motor-debug".to_string(),
        claim: "runtime_blocked_not_migrated".to_string(),
        no_takeoff: true,
        requires_no_props: true,
        guided_mode_required: true,
        required_mode: REQUIRED_GUIDED_MODE_NAME.to_string(),
        spin_mode: "armed_idle".to_string(),
        throttle_command_claim: "not_sent".to_string(),
        motor_percent: plan.motor_percent,
        motor_sec: plan.motor_sec,
        motor_count: plan.motor_count,
        steps: vec![
            json!({"step": "arm", "claim": "start_all_motors_at_fcu_armed_idle"}),
            json!({"step": "hold", "duration_sec": plan.motor_sec}),
            json!({"step": "disarm", "claim": "stop_all_motors"}),
        ],
        shutdown: "send_disarm_after_idle_spin".to_string(),
        landing_claim: "not_evaluated_no_takeoff".to_string(),
        serial: project.prepare.mavlink_router_serial_port.clone(),
        connection_endpoint: mavlink_router_endpoint(project),
        baud: project.prepare.mavlink_router_baud,
        arm_claim: "not_requested".to_string(),
        takeoff_claim: "not_evaluated".to_string(),
        guided_mode_claim: "not_evaluated".to_string(),
        shutdown_claim: "not_evaluated".to_string(),
        guided_mode: json!({}),
        command_plan: motor_debug_command_plan(),
        acks: Vec::new(),
        runtime_report: None,
    }
}

pub fn build_runtime_summary(
    project: &ProjectConfig,
    plan: &MotorDebugPlan,
    report: MotorDebugRuntimeReport,
) -> MotorDebugRuntimeSummary {
    MotorDebugRuntimeSummary {
        schema_version: TASK_RESULT_SCHEMA_VERSION.to_string(),
        ok: report.ok,
        blocked: report.blocked,
        blockers: report.blockers.clone(),
        task: "motor-debug".to_string(),
        claim: if report.ok {
            "runtime_executed".to_string()
        } else {
            "runtime_blocked".to_string()
        },
        no_takeoff: true,
        requires_no_props: true,
        guided_mode_required: true,
        required_mode: REQUIRED_GUIDED_MODE_NAME.to_string(),
        spin_mode: "armed_idle".to_string(),
        throttle_command_claim: report.throttle_command_claim.clone(),
        motor_percent: plan.motor_percent,
        motor_sec: plan.motor_sec,
        motor_count: plan.motor_count,
        steps: vec![
            json!({"step": "set_guided", "claim": "send_mav_cmd_do_set_mode_guided"}),
            json!({"step": "arm", "claim": "start_all_motors_at_fcu_armed_idle"}),
            json!({"step": "hold", "duration_sec": plan.motor_sec}),
            json!({"step": "disarm", "claim": "stop_all_motors"}),
        ],
        shutdown: "send_disarm_after_idle_spin".to_string(),
        landing_claim: "not_evaluated_no_takeoff".to_string(),
        serial: project.prepare.mavlink_router_serial_port.clone(),
        connection_endpoint: report.connection_endpoint.clone(),
        baud: project.prepare.mavlink_router_baud,
        arm_claim: ack_claim(report.arm_ack.as_ref()),
        takeoff_claim: "not_evaluated".to_string(),
        guided_mode_claim: if report.guided_mode.ok {
            "guided_confirmed_by_ack_and_heartbeat".to_string()
        } else {
            "guided_not_confirmed".to_string()
        },
        shutdown_claim: report.shutdown_claim.clone(),
        guided_mode: serde_json::to_value(&report.guided_mode).unwrap_or_else(|_| json!({})),
        command_plan: motor_debug_command_plan(),
        acks: runtime_acks(&report),
        runtime_report: Some(report),
    }
}

pub fn build_runtime_error_summary(
    project: &ProjectConfig,
    plan: &MotorDebugPlan,
    error: &str,
) -> MotorDebugRuntimeSummary {
    let mut summary = build_runtime_not_migrated_summary(project, plan);
    summary.blockers = vec![format!("motor_debug_mavlink_runtime_error:{error}")];
    summary.claim = "runtime_error".to_string();
    summary
}

fn ack_claim(ack: Option<&CommandAckEvaluation>) -> String {
    match ack {
        Some(ack) if ack.accepted => "accepted".to_string(),
        Some(ack) => format!("rejected:{}", ack.result_name),
        None => "not_requested".to_string(),
    }
}

fn runtime_acks(report: &MotorDebugRuntimeReport) -> Vec<Value> {
    let mut acks = Vec::new();
    if let Some(ack) = &report.guided_mode.command_ack {
        acks.push(json!({"stage": "guided_mode", "ack": ack}));
    }
    if let Some(ack) = &report.arm_ack {
        acks.push(json!({"stage": "arm", "ack": ack}));
    }
    if let Some(ack) = &report.disarm_ack {
        acks.push(json!({"stage": "disarm", "ack": ack}));
    }
    acks
}

pub fn evaluate_guided_mode(
    mode_mapping: &BTreeMap<String, u32>,
    observed_custom_modes: &[u32],
) -> GuidedModeEvaluation {
    let mapped_mode = mode_mapping.get(REQUIRED_GUIDED_MODE_NAME).copied();
    let observed_mode_id = observed_custom_modes.last().copied();
    let ok = observed_custom_modes.contains(&ARDUCOPTER_GUIDED_MODE_ID);
    let blockers = if ok {
        Vec::new()
    } else {
        vec![format!(
            "motor_debug_guided_mode_not_observed:{}",
            REQUIRED_GUIDED_MODE_NAME
        )]
    };

    GuidedModeEvaluation {
        ok,
        mode_name: REQUIRED_GUIDED_MODE_NAME.to_string(),
        mode_id: ARDUCOPTER_GUIDED_MODE_ID,
        mode_mapping_has_mode: mapped_mode.is_some(),
        mode_mapping_mode_id: mapped_mode,
        observed_mode_id,
        blockers,
    }
}

pub fn motor_debug_command_plan() -> MotorDebugCommandPlan {
    MotorDebugCommandPlan {
        guided_mode_id: ARDUCOPTER_GUIDED_MODE_ID,
        arm_command: arm_disarm_command(1, 0, true),
        disarm_command: arm_disarm_command(1, 0, false),
    }
}

pub fn arm_disarm_command(
    target_system: u8,
    target_component: u8,
    arm: bool,
) -> MotorDebugCommandLong {
    MotorDebugCommandLong {
        target_system,
        target_component,
        command: MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: if arm { 1.0 } else { 0.0 },
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    }
}

pub fn evaluate_command_ack(
    command: u32,
    result: u32,
    result_param2: u32,
    status_text: Vec<MavlinkStatusText>,
) -> CommandAckEvaluation {
    CommandAckEvaluation {
        command,
        result,
        result_name: mav_result_name(result),
        accepted: result == MAV_RESULT_ACCEPTED,
        result_param2,
        status_text,
    }
}

pub fn command_rejection_blockers(prefix: &str, ack: &CommandAckEvaluation) -> Vec<String> {
    if ack.accepted {
        return Vec::new();
    }

    let mut blockers = vec![format!("{}:{}", prefix, ack.result_name)];
    blockers.extend(
        ack.status_text
            .iter()
            .map(|status| format!("{}_status:{}", prefix, status.text)),
    );
    blockers
}

fn mav_result_name(result: u32) -> String {
    match result {
        MAV_RESULT_ACCEPTED => "MAV_RESULT_ACCEPTED".to_string(),
        MAV_RESULT_FAILED => "MAV_RESULT_FAILED".to_string(),
        1 => "MAV_RESULT_TEMPORARILY_REJECTED".to_string(),
        2 => "MAV_RESULT_DENIED".to_string(),
        3 => "MAV_RESULT_UNSUPPORTED".to_string(),
        5 => "MAV_RESULT_IN_PROGRESS".to_string(),
        6 => "MAV_RESULT_CANCELLED".to_string(),
        value => format!("MAV_RESULT_UNKNOWN_{value}"),
    }
}

fn resolve_summary_path(project: &ProjectConfig, options: &RunOptions) -> PathBuf {
    if let Some(path) = &options.summary_path {
        return path.clone();
    }
    options
        .artifact_dir
        .clone()
        .unwrap_or_else(|| {
            project
                .paths
                .artifact_root
                .join("motor-debug")
                .join(run_id())
        })
        .join("summary.json")
}

fn write_runtime_summary(path: &Path, summary: &MotorDebugRuntimeSummary) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create motor-debug artifact dir {}", parent.display()))?;
    }
    fs::write(path, serde_json::to_string_pretty(summary)?)
        .with_context(|| format!("write motor-debug runtime summary {}", path.display()))
}

fn run_id() -> String {
    let now = OffsetDateTime::now_utc();
    format!(
        "{:04}{:02}{:02}T{:02}{:02}{:02}Z",
        now.year(),
        u8::from(now.month()),
        now.day(),
        now.hour(),
        now.minute(),
        now.second()
    )
}

fn number_or_default(value: Option<&Value>, default: f64) -> f64 {
    value.and_then(Value::as_f64).unwrap_or(default)
}

fn unsigned_or_default(value: Option<&Value>, default: u32) -> u32 {
    value
        .and_then(Value::as_u64)
        .and_then(|value| u32::try_from(value).ok())
        .unwrap_or(default)
}

fn bool_or_default(value: Option<&Value>, default: bool) -> bool {
    value.and_then(Value::as_bool).unwrap_or(default)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{
        CommonDoctorConfig, OrchestrationConfig, PathConfig, PreflightConfig, PrepareConfig,
        RuntimeConfig, SourceConfig, TaskDoctorConfig,
    };
    use rstest::rstest;
    use serde_json::json;

    fn task_config() -> TaskConfig {
        TaskConfig {
            id: "motor-debug".to_string(),
            family: "real".to_string(),
            description: "test".to_string(),
            capabilities: vec![],
            task: serde_json::from_value(json!({
                "motor_percent": 5.0,
                "motor_sec": 5.0,
                "motor_count": 4
            }))
            .expect("task map"),
            safety: serde_json::from_value(json!({
                "confirm_manual_takeover": true,
                "confirm_kill_switch": true,
                "confirm_safe_area": true,
                "confirm_no_props": true
            }))
            .expect("safety map"),
        }
    }

    fn project() -> ProjectConfig {
        ProjectConfig {
            orchestration: OrchestrationConfig {
                family: "real".to_string(),
                implementation: "rust".to_string(),
                contract_version: "navlab.orchestration.v1".to_string(),
            },
            runtime: RuntimeConfig {
                mode: "real".to_string(),
                backend: "process".to_string(),
            },
            paths: PathConfig {
                workspace_root: "../..".into(),
                artifact_root: "artifacts/real".into(),
                task_config_dir: "configs/tasks".into(),
            },
            sources: SourceConfig {
                scan_source_claim: "real_lidar_driver".to_string(),
                scan_source_topic: "/scan".to_string(),
                fcu_source_claim: "real_serial_mavlink_or_ardupilot_dds_bridge".to_string(),
                imu_source_claim: "real_fcu_or_sensor".to_string(),
                rangefinder_source_claim: "real_or_not_required".to_string(),
                slam_source_claim: "real_slam".to_string(),
                required_real_topics: vec![],
                forbidden_simulation_input_topics: vec![],
            },
            preflight: PreflightConfig::default(),
            prepare: PrepareConfig::default(),
            common_doctor: CommonDoctorConfig::default(),
            task_doctor: TaskDoctorConfig::default(),
        }
    }

    #[test]
    fn motor_debug_plan_uses_cli_overrides() {
        let plan = build_plan(
            &task_config(),
            MotorDebugOverrides {
                motor_percent: Some(6.0),
                motor_sec: Some(2.0),
                motor_count: Some(3),
            },
        )
        .expect("plan");
        assert_eq!(plan.motor_percent, 6.0);
        assert_eq!(plan.motor_sec, 2.0);
        assert_eq!(plan.motor_count, 3);
    }

    #[test]
    fn motor_debug_plan_json_contract_is_stable() {
        let plan = build_plan(
            &task_config(),
            MotorDebugOverrides {
                motor_percent: Some(6.0),
                motor_sec: Some(2.0),
                motor_count: Some(4),
            },
        )
        .expect("plan");

        insta::assert_json_snapshot!(plan, @r###"
        {
          "task_id": "motor-debug",
          "motor_percent": 6.0,
          "motor_sec": 2.0,
          "motor_count": 4,
          "safety": {
            "confirm_manual_takeover": true,
            "confirm_kill_switch": true,
            "confirm_safe_area": true,
            "confirm_no_props": true
          },
          "claim": "dry_run_plan"
        }
        "###);
    }

    #[test]
    fn motor_debug_operator_safety_blocks_missing_confirmations() {
        let plan = build_plan(&task_config(), MotorDebugOverrides::default()).expect("plan");
        let safety = evaluate_operator_safety(&plan, &OperatorConfirmations::default());

        assert!(!safety.ok);
        assert_eq!(
            safety.blockers,
            vec![
                "operator_manual_takeover_not_confirmed",
                "operator_kill_switch_not_confirmed",
                "operator_safe_area_not_confirmed",
                "operator_no_props_not_confirmed"
            ]
        );
    }

    #[test]
    fn motor_debug_operator_safety_passes_with_all_confirmations() {
        let plan = build_plan(&task_config(), MotorDebugOverrides::default()).expect("plan");
        let safety = evaluate_operator_safety(
            &plan,
            &OperatorConfirmations {
                manual_takeover: true,
                kill_switch: true,
                safe_area: true,
                no_props: true,
            },
        );

        assert!(safety.ok);
        assert!(safety.blockers.is_empty());
    }

    #[test]
    fn motor_debug_mavlink_router_endpoint_matches_python_policy() {
        let mut project = project();

        project.prepare.mavlink_router_local_endpoint = "127.0.0.1:14550".to_string();
        assert_eq!(mavlink_router_endpoint(&project), "udpin:127.0.0.1:14550");

        project.prepare.mavlink_router_local_endpoint = "tcp:127.0.0.1:14550".to_string();
        assert_eq!(mavlink_router_endpoint(&project), "tcp:127.0.0.1:14550");

        project.prepare.mavlink_router_local_endpoint = ":14555".to_string();
        assert_eq!(mavlink_router_endpoint(&project), "udpin:127.0.0.1:14555");
    }

    #[test]
    fn motor_debug_runtime_not_migrated_summary_contract_is_stable() {
        let plan = build_plan(
            &task_config(),
            MotorDebugOverrides {
                motor_percent: Some(6.0),
                motor_sec: Some(2.0),
                motor_count: Some(4),
            },
        )
        .expect("plan");
        let summary = build_runtime_not_migrated_summary(&project(), &plan);

        insta::assert_json_snapshot!(summary, @r###"
        {
          "schema_version": "navlab.runtime.task_result.v1",
          "ok": false,
          "blocked": true,
          "blockers": [
            "motor_debug_rust_mavlink_runtime_not_migrated"
          ],
          "task": "motor-debug",
          "claim": "runtime_blocked_not_migrated",
          "no_takeoff": true,
          "requires_no_props": true,
          "guided_mode_required": true,
          "required_mode": "GUIDED",
          "spin_mode": "armed_idle",
          "throttle_command_claim": "not_sent",
          "motor_percent": 6.0,
          "motor_sec": 2.0,
          "motor_count": 4,
          "steps": [
            {
              "claim": "start_all_motors_at_fcu_armed_idle",
              "step": "arm"
            },
            {
              "duration_sec": 2.0,
              "step": "hold"
            },
            {
              "claim": "stop_all_motors",
              "step": "disarm"
            }
          ],
          "shutdown": "send_disarm_after_idle_spin",
          "landing_claim": "not_evaluated_no_takeoff",
          "serial": "/dev/ttyUSB1",
          "connection_endpoint": "udpin:127.0.0.1:14550",
          "baud": 115200,
          "arm_claim": "not_requested",
          "takeoff_claim": "not_evaluated",
          "guided_mode_claim": "not_evaluated",
          "shutdown_claim": "not_evaluated",
          "guided_mode": {},
          "command_plan": {
            "guided_mode_id": 4,
            "arm_command": {
              "target_system": 1,
              "target_component": 0,
              "command": 400,
              "confirmation": 0,
              "param1": 1.0,
              "param2": 0.0,
              "param3": 0.0,
              "param4": 0.0,
              "param5": 0.0,
              "param6": 0.0,
              "param7": 0.0
            },
            "disarm_command": {
              "target_system": 1,
              "target_component": 0,
              "command": 400,
              "confirmation": 0,
              "param1": 0.0,
              "param2": 0.0,
              "param3": 0.0,
              "param4": 0.0,
              "param5": 0.0,
              "param6": 0.0,
              "param7": 0.0
            }
          },
          "acks": [],
          "runtime_report": null
        }
        "###);
    }

    #[test]
    fn motor_debug_guided_mode_evaluation_uses_arducopter_guided_id() {
        let mapping = BTreeMap::from([(REQUIRED_GUIDED_MODE_NAME.to_string(), 15)]);

        let result = evaluate_guided_mode(&mapping, &[0, ARDUCOPTER_GUIDED_MODE_ID]);

        assert!(result.ok);
        assert_eq!(result.mode_id, ARDUCOPTER_GUIDED_MODE_ID);
        assert_eq!(result.mode_mapping_mode_id, Some(15));
        assert_eq!(result.observed_mode_id, Some(ARDUCOPTER_GUIDED_MODE_ID));
        assert!(result.blockers.is_empty());
    }

    #[test]
    fn motor_debug_guided_mode_evaluation_blocks_when_guided_is_not_observed() {
        let mapping = BTreeMap::from([("STABILIZE".to_string(), 0)]);

        let result = evaluate_guided_mode(&mapping, &[0]);

        assert!(!result.ok);
        assert!(!result.mode_mapping_has_mode);
        assert_eq!(result.mode_mapping_mode_id, None);
        assert_eq!(result.observed_mode_id, Some(0));
        assert_eq!(
            result.blockers,
            vec!["motor_debug_guided_mode_not_observed:GUIDED"]
        );
    }

    #[test]
    fn motor_debug_command_plan_matches_python_arm_disarm_policy() {
        let plan = motor_debug_command_plan();

        assert_eq!(plan.guided_mode_id, 4);
        assert_eq!(plan.arm_command.command, MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(plan.arm_command.param1, 1.0);
        assert_eq!(plan.arm_command.param2, 0.0);
        assert_eq!(plan.disarm_command.command, MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(plan.disarm_command.param1, 0.0);
        assert_eq!(plan.disarm_command.param2, 0.0);
    }

    #[test]
    fn motor_debug_command_ack_blockers_include_result_and_statustext() {
        let ack = evaluate_command_ack(
            MAV_CMD_COMPONENT_ARM_DISARM,
            MAV_RESULT_FAILED,
            0,
            vec![MavlinkStatusText {
                severity: 3,
                text: "PreArm: safety switch".to_string(),
            }],
        );

        let blockers = command_rejection_blockers("motor_debug_arm_rejected", &ack);

        assert!(!ack.accepted);
        assert_eq!(ack.result_name, "MAV_RESULT_FAILED");
        assert_eq!(
            blockers,
            vec![
                "motor_debug_arm_rejected:MAV_RESULT_FAILED",
                "motor_debug_arm_rejected_status:PreArm: safety switch"
            ]
        );
    }

    #[rstest]
    #[case::bad_percent(
        MotorDebugOverrides {
            motor_percent: Some(0.0),
            ..MotorDebugOverrides::default()
        },
        "motor_percent must be positive"
    )]
    #[case::bad_duration(
        MotorDebugOverrides {
            motor_sec: Some(0.0),
            ..MotorDebugOverrides::default()
        },
        "motor_sec must be positive"
    )]
    #[case::bad_motor_count(
        MotorDebugOverrides {
            motor_count: Some(0),
            ..MotorDebugOverrides::default()
        },
        "motor_count must be positive"
    )]
    fn motor_debug_plan_rejects_invalid_values(
        #[case] overrides: MotorDebugOverrides,
        #[case] expected: &str,
    ) {
        let error = build_plan(&task_config(), overrides).expect_err("invalid plan");

        assert_eq!(error.to_string(), expected);
    }
}
