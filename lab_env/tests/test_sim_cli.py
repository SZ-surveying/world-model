from __future__ import annotations

from pathlib import Path

from python_on_whales.exceptions import DockerException
from typer.testing import CliRunner

from lab_env import cli
from lab_env import sim_host
from lab_env.sim import runtime as sim_runtime

runner = CliRunner()
app = cli.app


class _FakeCompose:
    def __init__(self, exc: DockerException | None = None) -> None:
        self._exc = exc
        self.calls: list[tuple[str, list[str], bool]] = []
        self.up_calls: list[tuple[list[str], bool, bool]] = []
        self.down_calls = 0

    def execute(self, service: str, command: list[str], *, tty: bool) -> None:
        self.calls.append((service, command, tty))
        if self._exc is not None:
            raise self._exc

    def up(self, *, services: list[str], detach: bool, build: bool) -> None:
        self.up_calls.append((services, detach, build))

    def down(self) -> None:
        self.down_calls += 1


class _FakeClient:
    def __init__(self, exc: DockerException | None = None) -> None:
        self.compose = _FakeCompose(exc=exc)


def test_up_calls_runtime_exec(monkeypatch) -> None:
    calls: list[tuple[bool, str, str | None, float]] = []

    def fake_sim_up(
        *,
        markers: bool,
        mode: str,
        waypoint_file: str | None,
        timeout_sec: float,
    ) -> int:
        calls.append((markers, mode, waypoint_file, timeout_sec))
        return 0

    monkeypatch.setattr(sim_host, "sim_up", fake_sim_up)

    result = runner.invoke(app, ["sim", "up", "--no-markers"])

    assert result.exit_code == 0
    assert calls == [(False, "manual", None, 300.0)]


def test_up_auto_mode_passes_waypoint_file(monkeypatch, tmp_path: Path) -> None:
    calls: list[tuple[bool, str, str | None, float]] = []
    waypoint_file = tmp_path / "mission.yaml"
    waypoint_file.write_text("version: 1\ngoal:\n  x: 1.0\n  y: 0.0\n  z: 0.0\n", encoding="utf-8")

    def fake_sim_up(
        *,
        markers: bool,
        mode: str,
        waypoint_file: str | None,
        timeout_sec: float,
    ) -> int:
        calls.append((markers, mode, waypoint_file, timeout_sec))
        return 0

    monkeypatch.setattr(sim_host, "sim_up", fake_sim_up)

    result = runner.invoke(app, ["sim", "up", "--mode", "auto", "--waypoint-file", str(waypoint_file)])

    assert result.exit_code == 0
    assert calls == [(True, "auto", str(waypoint_file), 300.0)]


def test_up_auto_mode_passes_timeout(monkeypatch, tmp_path: Path) -> None:
    calls: list[tuple[bool, str, str | None, float]] = []
    waypoint_file = tmp_path / "mission.yaml"
    waypoint_file.write_text("version: 1\ngoal:\n  x: 1.0\n  y: 0.0\n  z: 0.0\n", encoding="utf-8")

    def fake_sim_up(
        *,
        markers: bool,
        mode: str,
        waypoint_file: str | None,
        timeout_sec: float,
    ) -> int:
        calls.append((markers, mode, waypoint_file, timeout_sec))
        return 0

    monkeypatch.setattr(sim_host, "sim_up", fake_sim_up)

    result = runner.invoke(
        app,
        ["sim", "up", "--mode", "auto", "--waypoint-file", str(waypoint_file), "--timeout-sec", "42"],
    )

    assert result.exit_code == 0
    assert calls == [(True, "auto", str(waypoint_file), 42.0)]


def test_down_calls_runtime_exec(monkeypatch) -> None:
    calls: list[str] = []

    def fake_sim_down() -> int:
        calls.append("down")
        return 0

    monkeypatch.setattr(sim_host, "sim_down", fake_sim_down)

    result = runner.invoke(app, ["sim", "down"])

    assert result.exit_code == 0
    assert calls == ["down"]


def test_exec_runtime_python_target_uses_compose_execute(monkeypatch) -> None:
    client = _FakeClient()
    monkeypatch.setattr(sim_host, "_sim_compose_client", lambda: client)

    exit_code = sim_host.exec_runtime_python_target(
        "pkg.module:run",
        ["--flag", "value"],
        record=True,
        label="consumer",
    )

    assert exit_code == 0
    assert client.compose.calls == [
        (
            "sim-runtime",
            [
                "bash",
                "/usr/local/bin/sim-runtime-env.sh",
                "python3",
                "-c",
                "from lab_env.sim.runtime import build_rosbag_options, invoke_python_target; "
                "raise SystemExit(invoke_python_target('pkg.module:run', ['--flag', 'value'], "
                "rosbag_options=build_rosbag_options(enabled=True, label='consumer')))",
            ],
            True,
        )
    ]


def test_exec_runtime_python_target_returns_compose_exit_code(monkeypatch) -> None:
    client = _FakeClient(exc=DockerException(["docker", "compose", "exec"], 17, b"", b"boom"))
    monkeypatch.setattr(sim_host, "_sim_compose_client", lambda: client)

    assert sim_host.exec_runtime_python_target("pkg.module:run") == 17


def test_build_rosbag_options_uses_sim_topic_profile(monkeypatch) -> None:
    monkeypatch.setenv("SESSION_ID", "demo-session")

    options = sim_runtime.build_rosbag_options(enabled=True, label="auto")

    assert options.enabled is True
    assert options.label == "auto"
    assert options.session_id == "demo-session"
    assert options.topic_file == sim_runtime._SIM_ROSBAG_TOPIC_FILE


def test_publish_cmd_vel_forward_preset_uses_ros2_topic_pub(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_exec_runtime_command(command: list[str]) -> int:
        calls.append(command)
        return 0

    monkeypatch.setattr(sim_host, "exec_runtime_command", fake_exec_runtime_command)

    assert sim_host.publish_cmd_vel_preset(preset="forward", duration=1.2, rate=5.0, linear_x=0.3) == 0
    assert calls == [
        [
            "ros2",
            "topic",
            "pub",
            "-r",
            "5.0",
            "--times",
            "6",
            "/planner/cmd_vel",
            "geometry_msgs/msg/Twist",
            "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}",
        ]
    ]


def test_publish_cmd_vel_stop_preset_uses_one_shot_pub(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_exec_runtime_command(command: list[str]) -> int:
        calls.append(command)
        return 0

    monkeypatch.setattr(sim_host, "exec_runtime_command", fake_exec_runtime_command)

    assert sim_host.publish_cmd_vel_preset(preset="stop") == 0
    assert calls == [
        [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/planner/cmd_vel",
            "geometry_msgs/msg/Twist",
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}",
        ]
    ]


def test_sim_up_uses_combined_profiles_and_restores_environment(monkeypatch) -> None:
    client = _FakeClient()
    captured_env: dict[str, str | None] = {}

    def fake_up(*, services: list[str], detach: bool, build: bool) -> None:
        captured_env["SIM_UP_MODE"] = sim_host.os.environ.get("SIM_UP_MODE")
        captured_env["SIM_AUTO_WAYPOINT_FILE"] = sim_host.os.environ.get("SIM_AUTO_WAYPOINT_FILE")
        captured_env["SIM_AUTO_ROSBAG_ENABLED"] = sim_host.os.environ.get("SIM_AUTO_ROSBAG_ENABLED")
        captured_env["SIM_AUTO_ROSBAG_LABEL"] = sim_host.os.environ.get("SIM_AUTO_ROSBAG_LABEL")
        captured_env["SIM_AUTO_RUN_ID"] = sim_host.os.environ.get("SIM_AUTO_RUN_ID")
        captured_env["SIM_AUTO_ARTIFACT_DIR"] = sim_host.os.environ.get("SIM_AUTO_ARTIFACT_DIR")
        captured_env["SIM_AUTO_LOG_FILE"] = sim_host.os.environ.get("SIM_AUTO_LOG_FILE")
        client.compose.up_calls.append((services, detach, build))

    client.compose.up = fake_up
    monkeypatch.setattr(sim_host, "_sim_compose_client", lambda: client)
    monkeypatch.delenv("SIM_MARKERS_AUTOSTART", raising=False)
    monkeypatch.delenv("SIM_UP_MODE", raising=False)
    monkeypatch.delenv("SIM_AUTO_WAYPOINT_FILE", raising=False)
    monkeypatch.delenv("SIM_AUTO_ROSBAG_ENABLED", raising=False)
    monkeypatch.delenv("SIM_AUTO_ROSBAG_LABEL", raising=False)

    assert sim_host.sim_up(markers=False, mode="manual") == 0
    assert client.compose.up_calls == [(["gazebo", "scan-bridge", "sim-runtime", "foxglove"], True, True)]
    assert captured_env == {
        "SIM_UP_MODE": "manual",
        "SIM_AUTO_WAYPOINT_FILE": "",
        "SIM_AUTO_ROSBAG_ENABLED": "false",
        "SIM_AUTO_ROSBAG_LABEL": sim_host._AUTO_ROSBAG_LABEL,
        "SIM_AUTO_RUN_ID": "",
        "SIM_AUTO_ARTIFACT_DIR": "",
        "SIM_AUTO_LOG_FILE": "",
    }
    assert "SIM_MARKERS_AUTOSTART" not in sim_host.os.environ
    assert "SIM_UP_MODE" not in sim_host.os.environ
    assert "SIM_AUTO_WAYPOINT_FILE" not in sim_host.os.environ
    assert "SIM_AUTO_ROSBAG_ENABLED" not in sim_host.os.environ
    assert "SIM_AUTO_ROSBAG_LABEL" not in sim_host.os.environ


def test_sim_up_auto_mode_sets_runtime_waypoint_env(monkeypatch, tmp_path: Path) -> None:
    client = _FakeClient()
    captured_env: dict[str, str | None] = {}
    waypoint_file = tmp_path / "mission.yaml"
    waypoint_file.write_text("version: 1\ngoal:\n  x: 1.0\n  y: 0.0\n  z: 0.0\n", encoding="utf-8")

    def fake_up(*, services: list[str], detach: bool, build: bool) -> None:
        captured_env["SIM_UP_MODE"] = sim_host.os.environ.get("SIM_UP_MODE")
        captured_env["SIM_AUTO_WAYPOINT_FILE"] = sim_host.os.environ.get("SIM_AUTO_WAYPOINT_FILE")
        captured_env["SIM_AUTO_ROSBAG_ENABLED"] = sim_host.os.environ.get("SIM_AUTO_ROSBAG_ENABLED")
        captured_env["SIM_AUTO_ROSBAG_LABEL"] = sim_host.os.environ.get("SIM_AUTO_ROSBAG_LABEL")
        captured_env["SIM_AUTO_RUN_ID"] = sim_host.os.environ.get("SIM_AUTO_RUN_ID")
        captured_env["SIM_AUTO_ARTIFACT_DIR"] = sim_host.os.environ.get("SIM_AUTO_ARTIFACT_DIR")
        captured_env["SIM_AUTO_LOG_FILE"] = sim_host.os.environ.get("SIM_AUTO_LOG_FILE")
        client.compose.up_calls.append((services, detach, build))

    client.compose.up = fake_up
    monkeypatch.setattr(sim_host, "_sim_compose_client", lambda: client)
    monkeypatch.setattr(sim_host, "load_straight_line_mission", lambda path: object())
    monkeypatch.setattr(sim_host, "_resolve_sim_runtime_path", lambda path: "/workspace/docs/sim/examples/demo.yaml")
    monkeypatch.setattr(
        sim_host,
        "_build_auto_artifact_paths",
        lambda session_id=None, run_id=None: (Path("/tmp/artifacts/ros/manual/auto_waypoint_follower/20260527_085852"), "/workspace/artifacts/ros/manual/auto_waypoint_follower/20260527_085852"),
    )
    monkeypatch.setattr(sim_host, "wait_for_auto_mission", lambda timeout_sec=300.0, status_topic="/sim/log": 0)
    monkeypatch.setattr(sim_host, "sim_down", lambda: 0)
    monkeypatch.setattr(sim_host, "configure_sim_logging", lambda **_kwargs: None)

    assert sim_host.sim_up(markers=True, mode="auto", waypoint_file=str(waypoint_file)) == 0
    assert client.compose.up_calls == [(["gazebo", "scan-bridge", "sim-runtime", "foxglove"], True, True)]
    assert captured_env == {
        "SIM_UP_MODE": "auto",
        "SIM_AUTO_WAYPOINT_FILE": "/workspace/docs/sim/examples/demo.yaml",
        "SIM_AUTO_ROSBAG_ENABLED": "true",
        "SIM_AUTO_ROSBAG_LABEL": sim_host._AUTO_ROSBAG_LABEL,
        "SIM_AUTO_RUN_ID": "20260527_085852",
        "SIM_AUTO_ARTIFACT_DIR": "/workspace/artifacts/ros/manual/auto_waypoint_follower/20260527_085852",
        "SIM_AUTO_LOG_FILE": "/workspace/artifacts/ros/manual/auto_waypoint_follower/20260527_085852/sim.log",
    }
    assert "SIM_UP_MODE" not in sim_host.os.environ
    assert "SIM_AUTO_WAYPOINT_FILE" not in sim_host.os.environ
    assert "SIM_AUTO_ROSBAG_ENABLED" not in sim_host.os.environ
    assert "SIM_AUTO_ROSBAG_LABEL" not in sim_host.os.environ
    assert "SIM_AUTO_RUN_ID" not in sim_host.os.environ
    assert "SIM_AUTO_ARTIFACT_DIR" not in sim_host.os.environ
    assert "SIM_AUTO_LOG_FILE" not in sim_host.os.environ


def test_sim_up_auto_mode_requires_waypoint_file(monkeypatch) -> None:
    try:
        sim_host.sim_up(markers=True, mode="auto")
    except ValueError as exc:
        assert str(exc) == "auto mode requires --waypoint-file"
    else:
        raise AssertionError("expected ValueError")


def test_sim_up_auto_mode_waits_and_tears_down(monkeypatch, tmp_path: Path) -> None:
    client = _FakeClient()
    waypoint_file = tmp_path / "mission.yaml"
    waypoint_file.write_text("version: 1\ngoal:\n  x: 1.0\n  y: 0.0\n  z: 0.0\n", encoding="utf-8")
    calls: list[tuple[str, object]] = []

    monkeypatch.setattr(sim_host, "_sim_compose_client", lambda: client)
    monkeypatch.setattr(sim_host, "load_straight_line_mission", lambda path: object())
    monkeypatch.setattr(sim_host, "_resolve_sim_runtime_path", lambda path: "/workspace/docs/sim/examples/demo.yaml")
    monkeypatch.setattr(
        sim_host,
        "_build_auto_artifact_paths",
        lambda session_id=None, run_id=None: (Path("/tmp/artifacts/ros/manual/auto_waypoint_follower/20260527_085852"), "/workspace/artifacts/ros/manual/auto_waypoint_follower/20260527_085852"),
    )
    monkeypatch.setattr(
        sim_host,
        "wait_for_auto_mission",
        lambda timeout_sec=300.0, status_topic="/sim/log": calls.append(("wait", timeout_sec)) or 0,
    )
    monkeypatch.setattr(sim_host, "sim_down", lambda: calls.append(("down", None)) or 0)
    monkeypatch.setattr(sim_host, "configure_sim_logging", lambda **_kwargs: None)

    assert sim_host.sim_up(markers=True, mode="auto", waypoint_file=str(waypoint_file)) == 0
    assert calls == [("wait", 300.0), ("down", None)]


def test_sim_down_uses_sim_compose_client(monkeypatch) -> None:
    client = _FakeClient()
    monkeypatch.setattr(sim_host, "_sim_compose_client", lambda: client)

    assert sim_host.sim_down() == 0
    assert client.compose.down_calls == 1
