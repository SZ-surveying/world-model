use serde::Serialize;
use serde_json::Value;

pub const TASK_FSM_SCHEMA_VERSION: &str = "navlab.task_fsm.v1";

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct TaskFsmSummary {
    pub schema_version: String,
    pub task_id: String,
    pub fsm_name: String,
    pub mode: String,
    pub ok: bool,
    pub blocked: bool,
    pub current_state: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed_state: Option<String>,
    pub blockers: Vec<String>,
    pub states: Vec<TaskFsmState>,
    pub transitions: Vec<TaskFsmTransition>,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct TaskFsmState {
    pub state: String,
    pub entered_at: String,
    pub reason: String,
    pub evidence: Value,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct TaskFsmTransition {
    pub task_id: String,
    pub fsm_name: String,
    pub from_state: String,
    pub to_state: String,
    pub event: String,
    pub reason_code: String,
    pub ok: bool,
    pub blocked: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub blocker: Option<String>,
    pub evidence: Value,
    pub at: String,
}

pub fn task_fsm_state(
    state: impl Into<String>,
    entered_at: impl Into<String>,
    reason: impl Into<String>,
    evidence: Value,
) -> TaskFsmState {
    TaskFsmState {
        state: state.into(),
        entered_at: entered_at.into(),
        reason: reason.into(),
        evidence,
    }
}

#[derive(Debug, Clone)]
pub struct TaskFsmTransitionInput {
    pub task_id: String,
    pub fsm_name: String,
    pub from_state: String,
    pub to_state: String,
    pub event: String,
    pub reason_code: String,
    pub ok: bool,
    pub blocker: Option<String>,
    pub evidence: Value,
    pub at: String,
}

pub fn task_fsm_transition(input: TaskFsmTransitionInput) -> TaskFsmTransition {
    TaskFsmTransition {
        task_id: input.task_id,
        fsm_name: input.fsm_name,
        from_state: input.from_state,
        to_state: input.to_state,
        event: input.event,
        reason_code: input.reason_code,
        ok: input.ok,
        blocked: !input.ok,
        blocker: input.blocker,
        evidence: input.evidence,
        at: input.at,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn task_fsm_transition_marks_blocked_from_ok_flag() {
        let transition = task_fsm_transition(TaskFsmTransitionInput {
            task_id: "motor-debug".to_string(),
            fsm_name: "motor-debug".to_string(),
            from_state: "guided".to_string(),
            to_state: "armed".to_string(),
            event: "arm_confirmed".to_string(),
            reason_code: "motor_debug_arm_rejected".to_string(),
            ok: false,
            blocker: Some("motor_debug_arm_rejected:MAV_RESULT_FAILED".to_string()),
            evidence: json!({"ack": "failed"}),
            at: "2026-06-26T00:00:00Z".to_string(),
        });

        assert!(transition.blocked);
        assert_eq!(transition.from_state, "guided");
        assert_eq!(transition.to_state, "armed");
        assert_eq!(
            transition.blocker.as_deref(),
            Some("motor_debug_arm_rejected:MAV_RESULT_FAILED")
        );
    }
}
