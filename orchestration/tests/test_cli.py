from __future__ import annotations

from types import SimpleNamespace

from src.cli import app
from typer.testing import CliRunner


def test_cli_exposes_only_build_doctor_and_run_wrapper() -> None:
    runner = CliRunner()

    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "build" in result.output
    assert "doctor" in result.output
    assert "run" in result.output
    assert "hover" not in result.output
    assert "exploration" not in result.output
    assert "scan-robustness" not in result.output

    result = runner.invoke(app, ["run", "--help"])
    assert result.exit_code == 0
    assert "Built-in task to run" in result.output
    assert "--duration-sec" in result.output
    assert "--simulation-profile" in result.output
    assert "--live" in result.output
    assert "--live-profiles" in result.output


def test_real_preflight_doctor_is_not_a_top_level_command() -> None:
    runner = CliRunner()

    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "real-preflight-doctor" not in result.output


def test_runtime_doctor_dispatches_real_preflight(monkeypatch) -> None:  # noqa: ANN001
    from src.tasks.real_preflight import RealPreflightDoctorTask

    calls: list[dict[str, object]] = []

    def fake_run(self, **kwargs):  # noqa: ANN001
        calls.append(kwargs)
        return 7

    monkeypatch.setattr(RealPreflightDoctorTask, "run", fake_run)
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


def test_runtime_doctor_force_dispatches_install_request(monkeypatch) -> None:  # noqa: ANN001
    from src.tasks.real_preflight import RealPreflightDoctorTask

    calls: list[dict[str, object]] = []

    def fake_run(self, **kwargs):  # noqa: ANN001
        calls.append(kwargs)
        return 0

    monkeypatch.setattr(RealPreflightDoctorTask, "run", fake_run)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["doctor", "--force"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 0
    assert calls[0]["force_install"] is True


def test_run_wrapper_dispatches_hover(monkeypatch) -> None:  # noqa: ANN001
    from src.tasks.hover import HoverAcceptanceTask

    calls: list[dict[str, object]] = []

    def fake_run(self, **kwargs):  # noqa: ANN001
        calls.append(kwargs)
        return 8

    monkeypatch.setattr(HoverAcceptanceTask, "run", fake_run)
    runner = CliRunner()

    result = runner.invoke(app, ["run", "hover", "--duration-sec", "2.5", "--simulation-profile", "ideal"])

    assert result.exit_code == 8
    assert calls
    assert calls[0]["duration_sec"] == 2.5
    assert calls[0]["simulation_profile"] == "ideal"


def test_run_wrapper_blocks_when_real_preflight_fails(monkeypatch) -> None:  # noqa: ANN001
    from src.tasks.doctor import DoctorTask
    from src.tasks.hover import HoverAcceptanceTask

    doctor_calls: list[dict[str, object]] = []

    def fake_doctor_run(self, **kwargs):  # noqa: ANN001
        doctor_calls.append(kwargs)
        return 7

    def fake_hover_run(self, **kwargs):  # noqa: ANN001
        raise AssertionError("real mode must not run hover acceptance directly")

    monkeypatch.setattr(DoctorTask, "run", fake_doctor_run)
    monkeypatch.setattr(HoverAcceptanceTask, "run", fake_hover_run)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 7
    assert doctor_calls


def test_run_wrapper_dispatches_real_prepare_and_task_doctor_in_order(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.doctor import DoctorTask

    calls: list[str] = []

    def fake_doctor_run(self, **kwargs):  # noqa: ANN001
        calls.append("preflight")
        assert kwargs["task_config_path"] is None
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append(f"prepare:{kwargs['task_name']}")
        return SimpleNamespace(return_code=0)

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        calls.append(f"task_doctor:{kwargs['task_name']}")
        return 0

    def fake_stop(result):  # noqa: ANN001
        calls.append("stop_prepare")

    monkeypatch.setattr(DoctorTask, "run", fake_doctor_run)
    monkeypatch.setattr(cli, "execute_real_prepare_phase", fake_prepare)
    monkeypatch.setattr(cli, "run_real_task_doctor", fake_task_doctor)
    monkeypatch.setattr(cli, "stop_real_prepare_phase", fake_stop)
    runner = CliRunner()

    result = runner.invoke(
        app,
        ["run", "hover"],
        env={"NAVLAB_RUNTIME_BACKEND": "process", "NAVLAB_RUNTIME_MODE": "real"},
    )

    assert result.exit_code == 20
    assert calls == ["preflight", "prepare:hover", "task_doctor:hover", "stop_prepare"]
    assert "companion startup" in result.output


def test_run_wrapper_does_not_start_task_doctor_when_prepare_fails(monkeypatch) -> None:  # noqa: ANN001
    from src import cli
    from src.tasks.doctor import DoctorTask

    calls: list[str] = []

    def fake_doctor_run(self, **kwargs):  # noqa: ANN001
        calls.append("preflight")
        return 0

    def fake_prepare(**kwargs):  # noqa: ANN001
        calls.append("prepare")
        return SimpleNamespace(return_code=20)

    def fake_task_doctor(**kwargs):  # noqa: ANN001
        raise AssertionError("task doctor must not run when prepare failed")

    monkeypatch.setattr(DoctorTask, "run", fake_doctor_run)
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
