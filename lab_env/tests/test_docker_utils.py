from __future__ import annotations

from lab_env import docker_utils


def test_exec_ros_command_uses_shared_ros_setup(monkeypatch) -> None:
    call: dict[str, str] = {}

    def fake_exec_shell_command_with_setup(container_name: str, *, setup: str, command: str) -> str:
        call["container_name"] = container_name
        call["setup"] = setup
        call["command"] = command
        return "ok"

    monkeypatch.setattr(docker_utils, "_exec_shell_command_with_setup", fake_exec_shell_command_with_setup)

    assert docker_utils._exec_ros_command("fast-lio", "ros2 topic list") == "ok"
    assert call == {
        "container_name": "fast-lio",
        "setup": docker_utils._ROS_ENV_SETUP,
        "command": "ros2 topic list",
    }


def test_count_nonempty_output_lines_ignores_blank_rows() -> None:
    assert docker_utils._count_nonempty_output_lines("\nalpha\n\n beta \n") == 2


def test_probe_ros_python_parses_json(monkeypatch) -> None:
    call: dict[str, str] = {}

    def fake_exec_ros_command(container_name: str, command: str) -> str:
        call["container_name"] = container_name
        call["command"] = command
        return '{"value": 1}'

    monkeypatch.setattr(docker_utils, "_exec_ros_command", fake_exec_ros_command)

    assert docker_utils._probe_ros_python("navlab-companion", 'print("hello")') == {"value": 1}
    assert call["container_name"] == "navlab-companion"
    assert call["command"].startswith("python3 -c ")


def test_probe_fast_lio_package_prefix_falls_back_when_probe_is_empty(monkeypatch) -> None:
    monkeypatch.setattr(docker_utils, "_probe_ros_python", lambda *_args, **_kwargs: {})

    assert docker_utils._probe_fast_lio_package_prefix("fast-lio") == "<missing>"
