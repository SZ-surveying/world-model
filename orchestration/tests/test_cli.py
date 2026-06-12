from __future__ import annotations

from types import SimpleNamespace

from src.cli import app
from typer.testing import CliRunner


def test_cli_exposes_only_doctor_and_run_wrapper() -> None:
    runner = CliRunner()

    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "build" not in result.output
    assert "doctor" in result.output
    assert "run" in result.output
    assert "hover" not in result.output
    assert "exploration" not in result.output
    assert "scan-robustness" not in result.output
    assert "motor-debug" not in result.output

    result = runner.invoke(app, ["run", "--help"])
    assert result.exit_code == 0
    assert "Built-in task to run" in result.output
    assert "--duration-sec" in result.output
    assert "--simulation-profile" in result.output
    assert "--live" in result.output
    assert "--live-profiles" in result.output
    assert "--dry-run" in result.output
    assert "--skip-doctor" in result.output
    assert "--confirm-no-props" in result.output
    assert "--motor-percent" in result.output


def test_real_preflight_doctor_is_not_a_top_level_command() -> None:
    runner = CliRunner()

    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "real-preflight-doctor" not in result.output

    result = runner.invoke(app, ["doctor", "--help"])
    assert result.exit_code == 0
    assert "[TASK_NAME]" not in result.output
    assert "hover, exploration, or scan-robustness" not in result.output


def test_runtime_doctor_stops_when_real_preflight_fails(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[dict[str, object]] = []

    def fake_run(**kwargs):  # noqa: ANN001
        calls.append(kwargs)
        return 7

    def fake_prepare(**kwargs):  # noqa: ANN001
        raise AssertionError("prepare must not start when preflight fails")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["doctor"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 7
    assert calls
    assert calls[0]["prompt_install"] is True
    assert calls[0]["force_install"] is False
    assert calls[0]["soft_dependency_warnings"] is True


def test_runtime_doctor_runs_preflight_prepare_common_doctor_and_stops(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_run(**kwargs):  # noqa: ANN001
        calls.append(f"preflight:force={kwargs['force_install']}")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        calls.append("common_doctor")
        return 0

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["doctor", "--force"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 0
    assert calls == ["preflight:force=True", "prepare:doctor", "common_doctor", "stop_prepare"]
    assert "Real doctor completed" in result.output


def test_runtime_doctor_stops_prepare_when_common_doctor_fails(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        calls.append("common_doctor")
        return 20

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["doctor"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert calls == ["preflight", "prepare:doctor", "common_doctor", "stop_prepare"]


def test_runtime_doctor_can_keep_prepare_running(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        calls.append("common_doctor")
        return 0

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["doctor", "--keep-prepare-running"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 0
    assert calls == ["preflight", "prepare:doctor", "common_doctor"]
    assert "prepare services are still running" in result.output


def test_run_wrapper_retires_python_simulation_tasks() -> None:
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover", "--duration-sec", "2.5", "--simulation-profile", "ideal"],
        env={"NAVLAB_RUNTIME_BACKEND": "docker", "NAVLAB_RUNTIME_MODE": "simulation"},
    )

    assert result.exit_code == 20
    assert "Python simulation tasks have been retired" in result.output
    assert "orchestration/sim/cmd/navlab-sim" in result.output


def test_run_wrapper_dry_run_does_not_restore_python_simulation_tasks() -> None:
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover", "--duration-sec", "2.5", "--simulation-profile", "ideal", "--dry-run"],
        env={"NAVLAB_RUNTIME_BACKEND": "docker", "NAVLAB_RUNTIME_MODE": "simulation"},
    )

    assert result.exit_code == 20
    assert "Python simulation tasks have been retired" in result.output


def test_run_wrapper_real_dry_run_stops_after_preflight_prepare_and_task_doctor(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        calls.append("common_doctor")
        return 0

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        calls.append(f"task_doctor:{kwargs['task_name']}")
        return 0

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "exploration", "--dry-run"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 0
    assert "runtime=process+real" in result.output
    assert "stopped after preflight, prepare, common doctor, and task doctor" in result.output
    assert calls == ["preflight", "prepare:exploration", "common_doctor", "task_doctor:exploration", "stop_prepare"]


def test_run_wrapper_blocks_when_real_preflight_fails(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    doctor_calls: list[dict[str, object]] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        doctor_calls.append(kwargs)
        return 7

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 7
    assert doctor_calls


def test_run_wrapper_real_motor_debug_dry_run_plans_without_spinning(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.built_in.motor_debug import BuiltInMotorDebugTask

    calls: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_motor_debug(self, **kwargs):  # noqa: ANN001
        raise AssertionError("motor-debug dry-run must not spin motors")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(BuiltInMotorDebugTask, "run", fake_motor_debug)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "motor-debug", "--dry-run", "--motor-percent", "4", "--motor-sec", "1.0"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 0
    assert calls == ["preflight"]
    assert "wrapper will not spin motors" in result.output
    assert "task=motor-debug" in result.output
    assert "spin_mode=armed_idle" in result.output
    assert "arm_command=MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=0" in result.output
    assert "disarm_command=MAV_CMD_COMPONENT_ARM_DISARM param1=0 param2=0" in result.output
    assert "hold_sec=1.0" in result.output
    assert "requires_no_props=True" in result.output
    assert "guided_mode_required=True" in result.output
    assert "required_mode=GUIDED" in result.output


def test_run_wrapper_real_motor_debug_requires_no_props_and_safety(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.built_in.motor_debug import BuiltInMotorDebugTask

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        return 0

    def fake_motor_debug(self, **kwargs):  # noqa: ANN001
        raise AssertionError("motor-debug must not run without confirmations")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(BuiltInMotorDebugTask, "run", fake_motor_debug)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "motor-debug"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert "Real motor-debug is blocked" in result.output
    assert "operator_manual_takeover_not_confirmed" in result.output
    assert "operator_kill_switch_not_confirmed" in result.output
    assert "operator_safe_area_not_confirmed" in result.output
    assert "operator_no_props_not_confirmed" in result.output


def test_run_wrapper_motor_debug_is_real_only() -> None:
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "motor-debug", "--dry-run"],
        env={"NAVLAB_RUNTIME_BACKEND": "docker", "NAVLAB_RUNTIME_MODE": "simulation"},
    )

    assert result.exit_code == 20
    assert "motor-debug is only supported with process+real runtime" in result.output


def test_run_wrapper_real_motor_debug_dispatches_with_confirmations(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.built_in.motor_debug import BuiltInMotorDebugTask

    calls: list[dict[str, object]] = []
    events: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        events.append("preflight")
        return 0

    def fake_motor_debug(self, **kwargs):  # noqa: ANN001
        events.append("motor-debug")
        calls.append(kwargs)
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        events.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        events.append("common-doctor")
        return 0

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        events.append(f"task-doctor:{kwargs['task_name']}")
        return 0

    def fake_stop(_result):  # noqa: ANN001
        events.append("stop-prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    monkeypatch.setattr(BuiltInMotorDebugTask, "run", fake_motor_debug)
    runner = CliRunner()

    result = runner.invoke(
        app,
        [
            "run",
            "motor-debug",
            "--confirm-manual-takeover",
            "--confirm-kill-switch",
            "--confirm-safe-area",
            "--confirm-no-props",
            "--motor-percent",
            "6",
            "--motor-sec",
            "2",
            "--motor-count",
            "4",
        ],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 0
    assert events == [
        "preflight",
        "prepare:motor-debug",
        "common-doctor",
        "task-doctor:motor-debug",
        "motor-debug",
        "stop-prepare",
    ]
    assert calls
    assert calls[0]["motor_percent"] == 6.0
    assert calls[0]["motor_sec"] == 2.0
    assert calls[0]["motor_count"] == 4


def test_run_wrapper_real_motor_debug_interrupt_stops_prepare(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.built_in.motor_debug import BuiltInMotorDebugTask

    events: list[str] = []

    def fake_doctor_run(**_kwargs):  # noqa: ANN001
        events.append("preflight")
        return 0

    def fake_motor_debug(self, **_kwargs):  # noqa: ANN001
        events.append("motor-debug")
        raise KeyboardInterrupt

    def fake_prepare(**kwargs):  # noqa: ANN001
        events.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**_kwargs):  # noqa: ANN001
        events.append("common-doctor")
        return 0

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        events.append(f"task-doctor:{kwargs['task_name']}")
        return 0

    def fake_stop(_result):  # noqa: ANN001
        events.append("stop-prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    monkeypatch.setattr(BuiltInMotorDebugTask, "run", fake_motor_debug)
    runner = CliRunner()

    result = runner.invoke(
        app,
        [
            "run",
            "motor-debug",
            "--confirm-manual-takeover",
            "--confirm-kill-switch",
            "--confirm-safe-area",
            "--confirm-no-props",
        ],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 130
    assert events == [
        "preflight",
        "prepare:motor-debug",
        "common-doctor",
        "task-doctor:motor-debug",
        "motor-debug",
        "stop-prepare",
    ]
    assert "WARN: real run interrupted by operator" in result.output
    assert "Real prepare services stopped after operator interrupt" in result.output


def test_run_wrapper_real_motor_debug_accepts_env_confirmations(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.built_in.motor_debug import BuiltInMotorDebugTask

    calls: list[dict[str, object]] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        return 0

    def fake_motor_debug(self, **kwargs):  # noqa: ANN001
        calls.append(kwargs)
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        return 0

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        return 0

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", lambda _result: None)
    monkeypatch.setattr(BuiltInMotorDebugTask, "run", fake_motor_debug)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "motor-debug"],
        env={
            "NAVLAB_RUNTIME_BACKEND": "process",
            "NAVLAB_RUNTIME_MODE": "real",
            "NAVLAB_CONFIRM_MANUAL_TAKEOVER": "true",
            "NAVLAB_CONFIRM_KILL_SWITCH": "true",
            "NAVLAB_CONFIRM_SAFE_AREA": "true",
            "NAVLAB_CONFIRM_NO_PROPS": "true",
        },
    )

    assert result.exit_code == 0
    assert calls
    assert calls[0]["motor_percent"] is None
    assert calls[0]["motor_sec"] is None
    assert calls[0]["motor_count"] is None


def test_run_wrapper_real_motor_debug_env_overrides_cli_confirmations(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.built_in.motor_debug import BuiltInMotorDebugTask

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        return 0

    def fake_motor_debug(self, **kwargs):  # noqa: ANN001
        raise AssertionError("env=false must override CLI confirmation flags")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(BuiltInMotorDebugTask, "run", fake_motor_debug)
    runner = CliRunner()

    result = runner.invoke(
        app,
        [
            "run",
            "motor-debug",
            "--confirm-manual-takeover",
            "--confirm-kill-switch",
            "--confirm-safe-area",
            "--confirm-no-props",
        ],
        env={
            "NAVLAB_RUNTIME_BACKEND": "process",
            "NAVLAB_RUNTIME_MODE": "real",
            "NAVLAB_CONFIRM_MANUAL_TAKEOVER": "false",
            "NAVLAB_CONFIRM_KILL_SWITCH": "true",
            "NAVLAB_CONFIRM_SAFE_AREA": "true",
            "NAVLAB_CONFIRM_NO_PROPS": "true",
        },
    )

    assert result.exit_code == 20
    assert "operator_manual_takeover_not_confirmed" in result.output


def test_run_wrapper_dispatches_real_prepare_and_task_doctor_in_order(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        assert kwargs["task_config_path"] is None
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        calls.append("common_doctor")
        return 0

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        calls.append(f"task_doctor:{kwargs['task_name']}")
        return 0

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover", "--confirm-manual-takeover", "--confirm-kill-switch", "--confirm-safe-area"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert calls == ["preflight", "prepare:hover", "common_doctor", "task_doctor:hover", "stop_prepare"]
    assert "companion startup" in result.output


def test_run_wrapper_skip_doctor_starts_at_task_doctor(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        raise AssertionError("preflight must be skipped")

    def fake_prepare(**kwargs):  # noqa: ANN001
        raise AssertionError("prepare must be skipped")

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        raise AssertionError("common doctor must be skipped")

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        calls.append(f"task_doctor:{kwargs['task_name']}")
        return 0

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    runner = CliRunner()

    result = runner.invoke(
        app,
        [
            "run",
            "hover",
            "--skip-doctor",
            "--confirm-manual-takeover",
            "--confirm-kill-switch",
            "--confirm-safe-area",
        ],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert calls == ["task_doctor:hover"]
    assert "Skipping real preflight, prepare, and common doctor." in result.output
    assert "companion startup" in result.output


def test_run_wrapper_blocks_real_flight_without_operator_safety(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_common_doctor(**kwargs):  # noqa: ANN001
        calls.append("common_doctor")
        return 0

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        calls.append(f"task_doctor:{kwargs['task_name']}")
        return 0

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_common_doctor", fake_common_doctor)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert calls == ["preflight", "prepare:hover", "common_doctor", "task_doctor:hover", "stop_prepare"]
    assert "operator safety missing" in result.output
    assert "operator_manual_takeover_not_confirmed" in result.output
    assert "operator_kill_switch_not_confirmed" in result.output
    assert "operator_safe_area_not_confirmed" in result.output


def test_run_wrapper_does_not_start_task_doctor_when_prepare_fails(monkeypatch) -> None:  # noqa: ANN001
    from src import cli

    calls: list[str] = []

    def fake_doctor_run(**kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append("prepare")
        return SimpleNamespace(return_code=20)

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        raise AssertionError("task doctor must not run when prepare failed")

    monkeypatch.setattr(cli, "run_real_preflight_doctor", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "exploration"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert calls == ["preflight", "prepare"]
