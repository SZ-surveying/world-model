use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use serde_json::{Value, json};

use crate::config::{ProjectConfig, TaskConfig};

pub const TASK_REQUEST_SCHEMA: &str = "navlab.orchestration.task_request.v1";
pub const TASK_RESULT_SCHEMA: &str = "navlab.orchestration.task_result.v1";
pub const DOCTOR_RESULT_SCHEMA: &str = "navlab.orchestration.doctor_result.v1";

pub fn task_request(
    project: &ProjectConfig,
    task: &TaskConfig,
    run_id: &str,
    artifact_dir: &Path,
) -> Value {
    json!({
        "schemaVersion": TASK_REQUEST_SCHEMA,
        "taskId": task.id,
        "runId": run_id,
        "runtimeMode": runtime_mode(project),
        "artifactDir": artifact_dir.display().to_string(),
        "capabilities": task.capabilities,
        "parameters": task.task,
        "sourceClaims": source_evidence(project),
    })
}

pub fn doctor_result(
    task_id: &str,
    ok: bool,
    blockers: &[String],
    checks: Vec<(&str, bool)>,
) -> Value {
    json!({
        "schemaVersion": DOCTOR_RESULT_SCHEMA,
        "taskId": task_id,
        "ok": ok,
        "blocked": !ok,
        "blockers": blocker_objects(blockers),
        "checks": checks.into_iter().map(|(name, ok)| {
            json!({
                "name": name,
                "ok": ok,
                "message": if ok { "" } else { "check failed" },
            })
        }).collect::<Vec<_>>(),
    })
}

pub struct TaskResultContractInput<'a> {
    pub project: &'a ProjectConfig,
    pub task_id: &'a str,
    pub run_id: &'a str,
    pub artifact_dir: &'a Path,
    pub summary_path: &'a Path,
    pub ok: bool,
    pub blockers: &'a [String],
    pub mavlink_acks: Vec<Value>,
    pub details: Value,
}

pub fn task_result(input: TaskResultContractInput<'_>) -> Value {
    json!({
        "schemaVersion": TASK_RESULT_SCHEMA,
        "taskId": input.task_id,
        "runId": input.run_id,
        "status": task_status(input.ok, input.blockers),
        "ok": input.ok,
        "blocked": !input.ok,
        "exitCode": if input.ok { 0 } else { 20 },
        "artifactDir": input.artifact_dir.display().to_string(),
        "summaryPath": input.summary_path.display().to_string(),
        "blockers": blocker_objects(input.blockers),
        "warnings": [],
        "sourceEvidence": source_evidence(input.project),
        "mavlinkAcks": input.mavlink_acks,
        "metrics": {},
        "evidence": {},
        "details": input.details,
    })
}

pub fn source_evidence(project: &ProjectConfig) -> Value {
    json!({
        "runtimeDomain": "RUNTIME_DOMAIN_REAL",
        "scanSource": project.sources.scan_source_claim,
        "imuSource": project.sources.imu_source_claim,
        "rangefinderSource": project.sources.rangefinder_source_claim,
        "slamSource": project.sources.slam_source_claim,
        "usesTruthAsControlInput": false,
    })
}

pub fn mavlink_ack(
    command: &str,
    result: &str,
    result_code: u32,
    accepted: bool,
    mode_before: &str,
    mode_after: &str,
    statustext: &str,
) -> Value {
    json!({
        "command": command,
        "result": result,
        "resultCode": result_code,
        "accepted": accepted,
        "modeBefore": mode_before,
        "modeAfter": mode_after,
        "statustext": statustext,
    })
}

pub fn write_json(path: &Path, value: &Value, label: &str) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .with_context(|| format!("create {label} dir {}", parent.display()))?;
    }
    fs::write(path, serde_json::to_string_pretty(value)?)
        .with_context(|| format!("write {label} {}", path.display()))
}

fn runtime_mode(project: &ProjectConfig) -> &'static str {
    if project.runtime.mode == "real" {
        "RUNTIME_MODE_REAL"
    } else {
        "RUNTIME_MODE_SIM"
    }
}

fn task_status(ok: bool, blockers: &[String]) -> &'static str {
    if ok {
        "TASK_STATUS_OK"
    } else if blockers.iter().any(|blocker| blocker.contains("error")) {
        "TASK_STATUS_ERROR"
    } else {
        "TASK_STATUS_BLOCKED"
    }
}

fn blocker_objects(blockers: &[String]) -> Vec<Value> {
    blockers
        .iter()
        .map(|blocker| {
            let code = blocker
                .split(':')
                .next()
                .filter(|value| !value.is_empty())
                .unwrap_or("unknown_blocker");
            json!({
                "code": code,
                "message": blocker,
                "source": blocker_source(code),
            })
        })
        .collect()
}

fn blocker_source(code: &str) -> &'static str {
    if code.contains("mavlink") || code.contains("ack") || code.contains("guided") {
        "mavlink"
    } else if code.contains("operator") {
        "operator"
    } else if code.contains("topic") || code.contains("source") {
        "sensors"
    } else if code.contains("prepare") || code.contains("preflight") {
        "runtime"
    } else {
        "doctor"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{
        CommonDoctorConfig, OrchestrationConfig, PathConfig, PreflightConfig, PrepareConfig,
        RuntimeConfig, SourceConfig, TaskDoctorConfig,
    };

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
                artifact_root: "../../artifacts/real".into(),
                task_config_dir: "configs/tasks".into(),
            },
            sources: SourceConfig {
                scan_source_claim: "ydlidar_x2".to_string(),
                scan_source_topic: "/scan".to_string(),
                fcu_source_claim: "real_serial_mavlink_or_ardupilot_dds_bridge".to_string(),
                imu_source_claim: "fcu_mavlink".to_string(),
                rangefinder_source_claim: "fcu_distance_sensor".to_string(),
                slam_source_claim: "cartographer".to_string(),
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
    fn source_evidence_matches_golden_real_claims() {
        let actual = source_evidence(&project());
        let golden: Value = serde_json::from_str(include_str!(
            "../../../contracts/examples/sensors/real_source_evidence.json"
        ))
        .expect("golden source evidence");

        assert_eq!(actual["runtimeDomain"], golden["runtimeDomain"]);
        assert_eq!(actual["scanSource"], golden["scanSource"]);
        assert_eq!(
            actual["usesTruthAsControlInput"],
            golden["usesTruthAsControlInput"]
        );
    }

    #[test]
    fn mavlink_ack_matches_golden_shape() {
        let actual = mavlink_ack(
            "ARM",
            "MAV_RESULT_FAILED",
            4,
            false,
            "GUIDED",
            "GUIDED",
            "Arm: RC not found",
        );
        let golden: Value = serde_json::from_str(include_str!(
            "../../../contracts/examples/safety/motor_debug_ack_failed.json"
        ))
        .expect("golden mavlink ack");

        assert_eq!(actual["command"], golden["command"]);
        assert_eq!(actual["accepted"], golden["accepted"]);
        assert_eq!(actual["resultCode"], golden["resultCode"]);
    }
}
