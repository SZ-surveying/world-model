from __future__ import annotations

from navlab.common.companion.mission import TASK_FSM_SCHEMA_VERSION, TaskFsmRecorder


def test_task_fsm_recorder_builds_planned_summary() -> None:
    recorder = TaskFsmRecorder(
        task_id="motor-debug",
        fsm_name="motor-debug",
        mode="planned",
        initial_state="runtime_ready",
        reason="planned_dry_run_no_motor_side_effect",
        evidence={"motor_sec": 5.0},
    )

    recorder.transition(
        to_state="guided",
        event="guided_confirmed",
        reason_code="guided_ack_and_heartbeat_planned",
        ok=True,
        at="planned",
        evidence={"request": "MAV_CMD_DO_SET_MODE", "mode": "GUIDED"},
    )
    recorder.enter_state(
        state="guided",
        entered_at="planned",
        reason="planned_dry_run_no_motor_side_effect",
    )

    summary = recorder.summary().to_dict()

    assert summary["schema_version"] == TASK_FSM_SCHEMA_VERSION
    assert summary["task_id"] == "motor-debug"
    assert summary["mode"] == "planned"
    assert summary["ok"] is True
    assert summary["current_state"] == "guided"
    assert summary["states"][0]["state"] == "runtime_ready"
    assert summary["transitions"][0]["from_state"] == "runtime_ready"
    assert summary["transitions"][0]["to_state"] == "guided"
    assert summary["transitions"][0]["evidence"]["mode"] == "GUIDED"


def test_task_fsm_recorder_blocks_failed_transition() -> None:
    recorder = TaskFsmRecorder(
        task_id="motor-debug",
        fsm_name="motor-debug",
        mode="actual",
        initial_state="guided",
        entered_at="2026-06-26T00:00:00Z",
        reason="guided_ack_and_heartbeat_observed",
    )

    recorder.transition(
        to_state="armed",
        event="arm_confirmed",
        reason_code="motor_debug_arm_rejected",
        ok=False,
        blocker="motor_debug_arm_rejected:MAV_RESULT_FAILED",
        at="2026-06-26T00:00:01Z",
        evidence={"ack": {"result_name": "MAV_RESULT_FAILED"}},
    )

    summary = recorder.summary().to_dict()

    assert summary["ok"] is False
    assert summary["blocked"] is True
    assert summary["current_state"] == "guided"
    assert summary["failed_state"] == "armed"
    assert summary["blockers"] == ["motor_debug_arm_rejected:MAV_RESULT_FAILED"]
    assert summary["transitions"][0]["blocked"] is True
    assert summary["transitions"][0]["blocker"] == "motor_debug_arm_rejected:MAV_RESULT_FAILED"
