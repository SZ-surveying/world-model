from __future__ import annotations

from typer.testing import CliRunner

from src.cli import app


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


def test_run_wrapper_dispatches_real_preflight_before_real_task(monkeypatch) -> None:  # noqa: ANN001
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
