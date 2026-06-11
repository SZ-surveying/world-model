from __future__ import annotations

import json
from io import StringIO
from pathlib import Path

from rich.console import Console
from src.configs.run_config import RunConfig
from src.tasks.built_in import motor_debug as motor_debug_module
from src.tasks.built_in.motor_debug import (
    BuiltInMotorDebugTask,
    MotorDebugTaskConfig,
    mavlink_router_endpoint,
    motor_debug_runtime_command,
    print_motor_debug_run_start,
    print_motor_debug_summary,
)

from navlab.real.companion.nodes import motor_debug as motor_debug_runtime
from navlab.real.companion.nodes.motor_debug import (
    TASK_RESULT_SCHEMA_VERSION,
    build_motor_debug_plan,
    command_rejection_blockers,
    ensure_guided_mode,
    send_arm_disarm,
    wait_command_ack,
)


class _Heartbeat:
    def __init__(self, custom_mode: int) -> None:
        self.custom_mode = custom_mode


class _FakeMaster:
    def __init__(self, *, mapping: dict[str, int], heartbeats: list[_Heartbeat]) -> None:
        self._mapping = mapping
        self._heartbeats = heartbeats
        self.set_modes: list[int] = []

    def mode_mapping(self) -> dict[str, int]:
        return self._mapping

    def set_mode(self, mode_id: int) -> None:
        self.set_modes.append(mode_id)

    def recv_match(self, **_kwargs) -> _Heartbeat | None:  # noqa: ANN003
        if not self._heartbeats:
            return None
        return self._heartbeats.pop(0)


class _FakeMav:
    def __init__(self) -> None:
        self.command_long_calls: list[tuple[object, ...]] = []

    def command_long_send(self, *args: object) -> None:
        self.command_long_calls.append(args)


class _FakeMotorTestMaster:
    def __init__(self) -> None:
        self.mav = _FakeMav()


class _FakeMavlink:
    MAV_CMD_COMPONENT_ARM_DISARM = 400


class _FakeAck:
    command = 400
    result = 4
    result_param2 = 0

    def get_type(self) -> str:
        return "COMMAND_ACK"


class _FakeStatusText:
    severity = 3
    text = "PreArm: safety switch"

    def get_type(self) -> str:
        return "STATUSTEXT"


class _FakeMessageMaster:
    def __init__(self, messages: list[object]) -> None:
        self._messages = messages

    def recv_match(self, **_kwargs) -> object | None:  # noqa: ANN003
        if not self._messages:
            return None
        return self._messages.pop(0)


def test_motor_debug_plan_requires_guided_mode() -> None:
    plan = build_motor_debug_plan(motor_percent=5.0, motor_sec=1.0, motor_count=4)

    assert plan["ok"] is True
    assert plan["schema_version"] == TASK_RESULT_SCHEMA_VERSION
    assert plan["guided_mode_required"] is True
    assert plan["required_mode"] == "GUIDED"


def test_motor_debug_guided_gate_sets_and_observes_guided() -> None:
    master = _FakeMaster(mapping={"GUIDED": 4}, heartbeats=[_Heartbeat(0), _Heartbeat(4)])

    result = ensure_guided_mode(master, mode_name="GUIDED", timeout_sec=0.1)

    assert result["ok"] is True
    assert master.set_modes == [4]
    assert result["observed_mode_id"] == 4


def test_motor_debug_guided_gate_uses_arducopter_guided_id_when_mapping_is_wrong() -> None:
    master = _FakeMaster(mapping={"GUIDED": 15}, heartbeats=[_Heartbeat(0), _Heartbeat(4)])

    result = ensure_guided_mode(master, mode_name="GUIDED", timeout_sec=0.1)

    assert result["ok"] is True
    assert master.set_modes == [4]
    assert result["mode_id"] == 4
    assert result["mode_mapping_mode_id"] == 15
    assert result["observed_mode_id"] == 4


def test_motor_debug_guided_gate_uses_arducopter_guided_id_when_mapping_is_missing() -> None:
    master = _FakeMaster(mapping={"STABILIZE": 0}, heartbeats=[_Heartbeat(4)])

    result = ensure_guided_mode(master, mode_name="GUIDED", timeout_sec=0.1)

    assert result["ok"] is True
    assert result["mode_id"] == 4
    assert result["mode_mapping_has_mode"] is False
    assert master.set_modes == [4]


def test_motor_debug_sends_arm_then_disarm_commands(monkeypatch) -> None:  # noqa: ANN001
    master = _FakeMotorTestMaster()
    wait_commands: list[int] = []

    def fake_wait_command_ack(*_args, **_kwargs):  # noqa: ANN001
        wait_commands.append(_args[1])
        return {"command": _args[1], "result": 0, "accepted": True}

    monkeypatch.setattr(motor_debug_runtime, "wait_command_ack", fake_wait_command_ack)

    arm_ack = send_arm_disarm(master, 1, 0, _FakeMavlink, arm=True)
    disarm_ack = send_arm_disarm(master, 1, 0, _FakeMavlink, arm=False)

    assert arm_ack["accepted"] is True
    assert disarm_ack["accepted"] is True
    assert wait_commands == [400, 400]
    assert master.mav.command_long_calls == [
        (
            1,
            0,
            400,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        ),
        (
            1,
            0,
            400,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    ]


def test_motor_debug_command_ack_includes_mav_result_and_statustext() -> None:
    master = _FakeMessageMaster([_FakeStatusText(), _FakeAck()])

    ack = wait_command_ack(master, 400, timeout_sec=0.1)
    blockers = command_rejection_blockers(prefix="motor_debug_arm_rejected", ack=ack)

    assert ack["accepted"] is False
    assert ack["result"] == 4
    assert ack["result_name"] == "MAV_RESULT_FAILED"
    assert ack["status_text"] == [{"severity": 3, "text": "PreArm: safety switch"}]
    assert blockers == [
        "motor_debug_arm_rejected:MAV_RESULT_FAILED",
        "motor_debug_arm_rejected_status:PreArm: safety switch",
    ]


def test_motor_debug_command_ack_drains_statustext_after_failure() -> None:
    master = _FakeMessageMaster([_FakeAck(), _FakeStatusText()])

    ack = wait_command_ack(master, 400, timeout_sec=0.1)
    blockers = command_rejection_blockers(prefix="motor_debug_arm_rejected", ack=ack)

    assert ack["accepted"] is False
    assert ack["result_name"] == "MAV_RESULT_FAILED"
    assert ack["status_text"] == [{"severity": 3, "text": "PreArm: safety switch"}]
    assert blockers == [
        "motor_debug_arm_rejected:MAV_RESULT_FAILED",
        "motor_debug_arm_rejected_status:PreArm: safety switch",
    ]


def test_motor_debug_uses_own_task_config_and_real_prepare_common_config() -> None:
    task_config = MotorDebugTaskConfig.from_file()
    real_prepare_config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")

    assert task_config.path == Path("orchestration/configs/motor_debug.toml").resolve()
    assert task_config.motor_percent == 5.0
    assert task_config.motor_sec == 5.0
    assert task_config.motor_count == 4
    assert real_prepare_config.orchestration.task_config_path == Path(
        "orchestration/configs/real_prepare.toml"
    ).resolve()
    assert real_prepare_config.orchestration.real_prepare.mavlink_router_serial_port == "/dev/ttyUSB1"
    assert real_prepare_config.orchestration.real_prepare.mavlink_router_baud == 115200
    assert mavlink_router_endpoint(real_prepare_config) == "udpin:127.0.0.1:14550"


def test_motor_debug_runtime_command_uses_navlab_real_module(tmp_path: Path) -> None:
    command = motor_debug_runtime_command(
        serial="/dev/ttyUSB1",
        baud=115200,
        endpoint="udpin:127.0.0.1:14550",
        motor_percent=5.0,
        motor_sec=5.0,
        motor_count=4,
        summary_path=tmp_path / "summary.json",
    )

    assert command[1:4] == ["-m", "navlab.real.companion.nodes.motor_debug", "--serial"]
    assert "--summary-path" in command
    assert str(tmp_path / "summary.json") in command


def test_motor_debug_task_run_reads_task_config_and_real_prepare_common_config(monkeypatch, tmp_path) -> None:  # noqa: ANN001
    calls: list[dict[str, object]] = []

    def fake_sequence(**kwargs):  # noqa: ANN001
        config = kwargs["config"]
        calls.append(
            {
                "serial": config.orchestration.real_prepare.mavlink_router_serial_port,
                "baud": config.orchestration.real_prepare.mavlink_router_baud,
                "motor_percent": kwargs["motor_percent"],
                "motor_sec": kwargs["motor_sec"],
                "motor_count": kwargs["motor_count"],
            }
        )
        return {
            "ok": True,
            "blockers": [],
            "serial": config.orchestration.real_prepare.mavlink_router_serial_port,
            "connection_endpoint": "udpin:127.0.0.1:14550",
            "baud": config.orchestration.real_prepare.mavlink_router_baud,
            "motor_percent": kwargs["motor_percent"],
            "motor_sec": kwargs["motor_sec"],
            "motor_count": kwargs["motor_count"],
            "required_mode": "GUIDED",
            "guided_mode": {"ok": True},
            "shutdown_claim": "not_evaluated",
        }

    monkeypatch.setenv("ARTIFACT_DIR", str(tmp_path))
    monkeypatch.setattr(motor_debug_module, "run_motor_debug_sequence", fake_sequence)
    monkeypatch.setattr(
        motor_debug_module,
        "collect_motor_debug_common_state",
        lambda *_args, **_kwargs: {
            "active_source_set": "not_observed",
            "configured_external_nav_source_set": "SRC2",
            "observed_ekf_source_set": "not_observed",
            "ekf_source_set_switch": {"enabled": True, "target_source_set": "SRC2", "sent": True, "ack_result": 0},
            "ek3_src2": {"posxy": "6", "velxy": "6", "velz": "6", "yaw": "6", "posz": "1"},
            "gps_source_sets": ["SRC1"],
        },
    )

    stream = StringIO()
    rc = BuiltInMotorDebugTask().run(config_path="orchestration/config.real.toml", console=Console(file=stream))

    assert rc == 0
    output = stream.getvalue()
    assert "NavLab Real Motor Debug Run" in output
    assert "Guided gate" in output
    assert "run_stage" in output
    assert "MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=0" in output
    assert calls == [
        {
            "serial": "/dev/ttyUSB1",
            "baud": 115200,
            "motor_percent": 5.0,
            "motor_sec": 5.0,
            "motor_count": 4,
        }
    ]
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["schema_version"] == TASK_RESULT_SCHEMA_VERSION
    assert summary["logs"][0]["process"] == "companion"
    assert summary["logs"][0]["path"] == str(tmp_path / "logs" / "companion.log")
    assert summary["logs"][0]["entries"] >= 2
    assert summary["common_state"]["configured_external_nav_source_set"] == "SRC2"
    assert (tmp_path / "logs" / "companion.log").is_file()


def test_motor_debug_run_start_panel_shows_runtime_gate() -> None:
    stream = StringIO()
    console = Console(file=stream, force_terminal=False, width=100)
    config = RunConfig.from_config(config_path="orchestration/config.real.toml", task_name="real-prepare")
    task_config = MotorDebugTaskConfig.from_file()

    print_motor_debug_run_start(console, config=config, task_config=task_config, summary_path=Path("summary.json"))

    output = stream.getvalue()
    assert "NavLab Real Motor Debug Run" in output
    assert "/dev/ttyUSB1 @ 115200" in output
    assert "udpin:127.0.0.1:14550" in output
    assert "Guided gate" in output
    assert "run_stage" in output
    assert "MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=0" in output
    assert "MAV_CMD_COMPONENT_ARM_DISARM param1=0 param2=0" in output
    assert "┏" not in output
    assert "┃" not in output


def test_motor_debug_summary_uses_doctor_style_panels() -> None:
    stream = StringIO()
    console = Console(file=stream, force_terminal=False, width=100)
    summary = {
        "ok": False,
        "serial": "/dev/ttyUSB1",
        "connection_endpoint": "udpin:127.0.0.1:14550",
        "baud": 115200,
        "motor_count": 4,
        "motor_percent": 5.0,
        "motor_sec": 5.0,
        "required_mode": "GUIDED",
        "guided_mode": {"ok": None},
        "common_state": {
            "active_source_set": "not_observed",
            "configured_external_nav_source_set": "SRC2",
            "observed_ekf_source_set": "not_observed",
            "ekf_source_set_switch": {"enabled": True, "target_source_set": "SRC2", "sent": True, "ack_result": 0},
            "ek3_src2": {"posxy": "6", "velxy": "6", "velz": "6", "yaw": "6", "posz": "1"},
            "gps_source_sets": ["SRC1"],
        },
        "shutdown_claim": "not_evaluated",
        "blockers": ["motor_debug_failed:test"],
        "logs": [
            {
                "process": "companion",
                "path": "artifacts/ros/example/logs/companion.log",
                "entries": 3,
                "bytes": 180,
            }
        ],
    }

    print_motor_debug_summary(console, summary=summary, summary_path=Path("artifacts/ros/example/summary.json"))

    output = stream.getvalue()
    assert "NavLab Real Motor Debug" in output
    assert "Status" in output
    assert "Serial" in output
    assert "/dev/ttyUSB1 @ 115200" in output
    assert "udpin:127.0.0.1:14550" in output
    assert "ExternalNav config" in output
    assert "EKF active src" in output
    assert "SRC2" in output
    assert "EK3 SRC2" in output
    assert "velz=6" in output
    assert "GPS EKF src" in output
    assert "SRC1" in output
    assert "Blockers" in output
    assert "- motor_debug_failed:test" in output
    assert "Logs" in output
    assert "companion" in output
    assert "3 entries, 180 bytes" in output
    assert "artifacts/ros/example/logs/companion.log" in output
    assert "┏" not in output
    assert "┃" not in output
