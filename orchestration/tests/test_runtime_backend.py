from __future__ import annotations

import os
import sys
import time
from io import StringIO
from pathlib import Path

import pytest
from python_on_whales.exceptions import DockerException
from rich.console import Console
from src.config import RunConfig
from src.project_config import RuntimeConfig, load_orchestration_runtime_backend_config
from src.runtime import (
    DockerBackend,
    ProbeSpec,
    ProcessBackend,
    ProcessManager,
    RosbagSpec,
    ServiceSpec,
    VolumeMount,
    WorkspacePathMapper,
)
from src.runtime.errors import BackendConfigError, PathMappingError, RuntimeModeViolationError, ServiceStartError
from src.tasks.helpers import scan_integrity as scan_integrity_gate


class FakeDockerClient:
    def __init__(self) -> None:
        self.runs: list[dict[str, object]] = []
        self.waits: list[str] = []
        self.removes: list[dict[str, object]] = []

    def run(self, image, command, **kwargs):  # noqa: ANN001
        self.runs.append({"image": image, "command": command, **kwargs})
        return type("Container", (), {"id": "container-123"})()

    def wait(self, identifier):  # noqa: ANN001
        self.waits.append(identifier)
        return 0

    def remove(self, identifier, **kwargs):  # noqa: ANN001
        self.removes.append({"identifier": identifier, **kwargs})

    def logs(self, identifier, **kwargs):  # noqa: ANN001
        return f"logs:{identifier}:{kwargs['tail']}"


def test_service_spec_requires_process_command() -> None:
    with pytest.raises(BackendConfigError, match="process backend requires command"):
        ServiceSpec(name="companion", command=()).validate_for_process()


def test_service_spec_rejects_process_image() -> None:
    with pytest.raises(BackendConfigError, match="does not accept image"):
        ServiceSpec(name="companion", command=("echo", "ok"), image="image:tag").validate_for_process()


def test_service_spec_rejects_non_string_env_value() -> None:
    spec = ServiceSpec(name="companion", command=("echo", "ok"), env={"ROS_DOMAIN_ID": 85})  # type: ignore[dict-item]

    with pytest.raises(BackendConfigError, match="env ROS_DOMAIN_ID must be a string"):
        spec.validate_for_process()


def test_rosbag_spec_reads_topics_profile(tmp_path: Path) -> None:
    profile = tmp_path / "topics.txt"
    profile.write_text("# comment\n/scan\n\n/tf\n", encoding="utf-8")
    spec = RosbagSpec(name="bag", topics_profile=profile, output_path=tmp_path / "out")

    assert spec.topics() == ("/scan", "/tf")
    assert spec.command() == ("ros2", "bag", "record", "--storage", "mcap", "-o", str(tmp_path / "out"), "/scan", "/tf")


def test_rosbag_spec_missing_profile_fails(tmp_path: Path) -> None:
    spec = RosbagSpec(name="bag", topics_profile=tmp_path / "missing.txt", output_path=tmp_path / "out")

    with pytest.raises(BackendConfigError, match="missing topics profile"):
        spec.command()


def test_docker_backend_starts_service_with_explicit_spec(tmp_path: Path) -> None:
    fake = FakeDockerClient()
    backend = DockerBackend(client_factory=lambda: fake)
    handle = backend.start_service(
        ServiceSpec(
            name="slam",
            image="world-model/slam:latest",
            command=("bash", "-lc", "echo ok"),
            container_name="navlab-slam",
            env={"ROS_DOMAIN_ID": "85"},
            cwd="/workspace",
            volumes=(VolumeMount(source=tmp_path, target="/workspace"),),
            networks=("host",),
        )
    )

    assert handle.backend == "docker"
    assert handle.identifier == "navlab-slam"
    assert fake.runs[0]["image"] == "world-model/slam:latest"
    assert fake.runs[0]["envs"] == {"ROS_DOMAIN_ID": "85"}
    assert fake.runs[0]["volumes"] == [(tmp_path, "/workspace")]
    assert backend.wait(handle) == 0
    assert backend.logs(handle, tail=12) == "logs:navlab-slam:12"
    backend.stop(handle)
    assert fake.removes == [{"identifier": "navlab-slam", "force": True}]


def test_docker_backend_wraps_start_error() -> None:
    class FailingDockerClient(FakeDockerClient):
        def run(self, image, command, **kwargs):  # noqa: ANN001
            raise DockerException(["docker", "run"], 7, stderr=b"nope")

    backend = DockerBackend(client_factory=FailingDockerClient)

    with pytest.raises(ServiceStartError, match="docker service slam failed to start"):
        backend.start_service(ServiceSpec(name="slam", image="image", command=("cmd",)))


def test_process_backend_runs_probe_and_writes_log(tmp_path: Path) -> None:
    log_path = tmp_path / "probe.log"
    backend = ProcessBackend(default_log_dir=tmp_path)
    result = backend.run_probe(
        ProbeSpec(
            name="hello",
            command=(sys.executable, "-c", "print('hello process')"),
            timeout_sec=5,
            log_path=log_path,
        )
    )

    assert result.ok is True
    assert "hello process" in result.stdout
    assert "hello process" in log_path.read_text(encoding="utf-8")


def test_process_backend_starts_waits_and_tails_service_log(tmp_path: Path) -> None:
    backend = ProcessBackend(default_log_dir=tmp_path)
    handle = backend.start_service(
        ServiceSpec(
            name="short",
            command=(sys.executable, "-c", "print('line1'); print('line2')"),
            log_path=tmp_path / "short.log",
        )
    )

    assert handle.backend == "process"
    assert handle.pid is not None
    assert backend.wait(handle, timeout_sec=5) == 0
    assert backend.logs(handle, tail=1) == "line2"


def test_process_backend_wait_returns_nonzero_rc(tmp_path: Path) -> None:
    backend = ProcessBackend(default_log_dir=tmp_path)
    handle = backend.start_service(
        ServiceSpec(
            name="failing",
            command=(sys.executable, "-c", "raise SystemExit(13)"),
            log_path=tmp_path / "failing.log",
        )
    )

    assert backend.wait(handle, timeout_sec=5) == 13


def test_process_backend_dry_run_does_not_start_process(tmp_path: Path) -> None:
    backend = ProcessBackend(default_log_dir=tmp_path, dry_run=True)
    handle = backend.start_service(ServiceSpec(name="dry", command=("echo", "ok")))

    assert handle.identifier == "dry-run:dry"
    assert backend.wait(handle) == 0
    assert "command=echo ok" in (tmp_path / "dry.log").read_text(encoding="utf-8")


def test_process_backend_stop_kills_child_process_group(tmp_path: Path) -> None:
    child_code = "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"
    parent_code = (
        "import subprocess,sys,time;"
        f"p=subprocess.Popen([sys.executable, '-c', {child_code!r}]);"
        "print(p.pid, flush=True);"
        "time.sleep(30)"
    )
    backend = ProcessBackend(default_log_dir=tmp_path)
    handle = backend.start_service(
        ServiceSpec(
            name="child-group",
            command=(sys.executable, "-c", parent_code),
            log_path=tmp_path / "child-group.log",
        )
    )
    assert handle.pid is not None
    pgid = os.getpgid(handle.pid)

    deadline = time.monotonic() + 2.0
    while not (tmp_path / "child-group.log").read_text(encoding="utf-8").strip():
        assert time.monotonic() < deadline
        time.sleep(0.05)

    backend.stop(handle, timeout_sec=0.2)

    deadline = time.monotonic() + 2.0
    while _process_group_alive(pgid) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not _process_group_alive(pgid)


def _process_group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    return True


def test_process_manager_starts_waits_and_tails_logs(tmp_path: Path) -> None:
    manager = ProcessManager()
    managed = manager.start(
        name="manager-short",
        command=(sys.executable, "-c", "print('a'); print('b')"),
        cwd=None,
        env={},
        log_path=tmp_path / "manager-short.log",
    )

    assert managed.pid > 0
    assert managed.pgid is not None
    assert manager.wait(managed, timeout_sec=5) == 0
    assert manager.tail_logs(managed, tail=1) == "b"


def test_process_backend_delegates_service_lifecycle_to_manager(tmp_path: Path) -> None:
    class FakeManager:
        def __init__(self) -> None:
            self.started: list[dict[str, object]] = []
            self.waited: list[object] = []
            self.stopped: list[object] = []
            self.managed = None

        def start(self, **kwargs):  # noqa: ANN001
            self.started.append(kwargs)
            from src.runtime.process_manager import ManagedProcess

            self.managed = ManagedProcess(
                name=kwargs["name"],
                pid=12345,
                pgid=12345,
                command=tuple(kwargs["command"]),
                cwd=Path(kwargs["cwd"]) if kwargs["cwd"] is not None else None,
                log_path=kwargs["log_path"],
                started_at=1.0,
            )
            return self.managed

        def get(self, pid):  # noqa: ANN001
            assert pid == 12345
            return self.managed

        def wait(self, managed, *, timeout_sec=None):  # noqa: ANN001
            self.waited.append((managed, timeout_sec))
            return 7

        def terminate_group(self, managed, *, timeout_sec=5.0):  # noqa: ANN001
            self.stopped.append((managed, timeout_sec))

        def tail_logs(self, handle_or_path, *, tail=400):  # noqa: ANN001
            return f"tail:{tail}"

    manager = FakeManager()
    backend = ProcessBackend(default_log_dir=tmp_path, manager=manager)  # type: ignore[arg-type]
    handle = backend.start_service(
        ServiceSpec(
            name="managed",
            command=("echo", "ok"),
            cwd=tmp_path,
            env={"ROS_DOMAIN_ID": "85"},
        )
    )

    assert handle.pid == 12345
    assert manager.started[0]["name"] == "managed"
    assert manager.started[0]["cwd"] == tmp_path
    assert manager.started[0]["env"] == {"ROS_DOMAIN_ID": "85"}
    assert backend.wait(handle, timeout_sec=3) == 7
    backend.stop(handle, timeout_sec=2)
    assert manager.waited[0][1] == 3
    assert manager.stopped[0][1] == 2
    assert backend.logs(handle, tail=9) == "tail:9"


def test_workspace_path_mapper_maps_docker_workspace(tmp_path: Path) -> None:
    root = tmp_path / "repo"
    profile = root / "profiles" / "topics.txt"
    profile.parent.mkdir(parents=True)
    profile.write_text("/scan\n", encoding="utf-8")
    mapper = WorkspacePathMapper(host_root=root, backend_workspace_root="/workspace", backend="docker")

    assert mapper.backend_path(profile) == "/workspace/profiles/topics.txt"


def test_workspace_path_mapper_rejects_process_container_only_path(tmp_path: Path) -> None:
    mapper = WorkspacePathMapper(host_root=tmp_path, backend_workspace_root=str(tmp_path), backend="process")

    with pytest.raises(PathMappingError, match="container-only path"):
        mapper.backend_path(Path("/workspace/profiles/topics.txt"))


def test_load_runtime_backend_config_defaults_to_docker(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config_file = tmp_path / "config.toml"
    config_file.write_text("", encoding="utf-8")
    monkeypatch.delenv("NAVLAB_RUNTIME_BACKEND", raising=False)
    monkeypatch.delenv("NAVLAB_RUNTIME_MODE", raising=False)
    runtime = RuntimeConfig(
        lab_root=tmp_path,
        ardupilot_root=tmp_path / "ardupilot",
        mavlink_router_root=tmp_path / "mavlink-router",
        venv_path=tmp_path / ".venv",
        config_file=config_file,
        config_loaded=True,
    )

    config = load_orchestration_runtime_backend_config(runtime)

    assert config.backend.value == "docker"
    assert config.backend.source == "default"
    assert config.mode.value == "simulation"
    assert config.mode.source == "default"
    assert config.docker.workspace_container_path.value == "/workspace"
    assert config.real_sources.scan_source_claim.value == "real_lidar_driver"
    assert "/scan" in config.real_sources.required_real_topics


def test_load_runtime_backend_config_env_rejects_missing_process_services(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config_file = tmp_path / "config.toml"
    config_file.write_text(
        "[orchestration.runtime.process]\nrequire_explicit_services = true\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    runtime = RuntimeConfig(
        lab_root=tmp_path,
        ardupilot_root=tmp_path / "ardupilot",
        mavlink_router_root=tmp_path / "mavlink-router",
        venv_path=tmp_path / ".venv",
        config_file=config_file,
        config_loaded=True,
    )

    with pytest.raises(ValueError, match="process backend requires explicit services"):
        load_orchestration_runtime_backend_config(runtime)


def test_load_runtime_backend_config_parses_process_service(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config_file = tmp_path / "config.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = true
log_dir = "logs"

[orchestration.runtime.process.services.probe]
command = ["python", "-c", "print('ok')"]
cwd = "."
env = { ROS_DOMAIN_ID = "85" }

[orchestration.runtime.real.sources]
scan_source_claim = "real_x2"
scan_source_topic = "/scan"
fcu_source_claim = "real_ap"
imu_source_claim = "real_imu"
rangefinder_source_claim = "not_required"
slam_source_claim = "real_cartographer"
required_real_topics = ["/scan", "/tf"]
forbidden_simulation_input_topics = ["/gazebo/*"]
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    runtime = RuntimeConfig(
        lab_root=tmp_path,
        ardupilot_root=tmp_path / "ardupilot",
        mavlink_router_root=tmp_path / "mavlink-router",
        venv_path=tmp_path / ".venv",
        config_file=config_file,
        config_loaded=True,
    )

    config = load_orchestration_runtime_backend_config(runtime)

    assert config.backend.value == "process"
    assert config.mode.value == "real"
    assert config.process.log_dir == tmp_path / "logs"
    assert config.process.services["probe"].command == ("python", "-c", "print('ok')")
    assert config.process.services["probe"].cwd == tmp_path
    assert config.process.services["probe"].env == {"ROS_DOMAIN_ID": "85"}
    assert config.real_sources.scan_source_claim.value == "real_x2"
    assert config.real_sources.fcu_source_claim.value == "real_ap"
    assert config.real_sources.required_real_topics == ("/scan", "/tf")
    assert config.real_sources.forbidden_simulation_input_topics == ("/gazebo/*",)


def test_load_runtime_backend_config_rejects_unknown_backend(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config_file = tmp_path / "config.toml"
    config_file.write_text("", encoding="utf-8")
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "other")
    monkeypatch.delenv("NAVLAB_RUNTIME_MODE", raising=False)
    runtime = RuntimeConfig(
        lab_root=tmp_path,
        ardupilot_root=tmp_path / "ardupilot",
        mavlink_router_root=tmp_path / "mavlink-router",
        venv_path=tmp_path / ".venv",
        config_file=config_file,
        config_loaded=True,
    )

    with pytest.raises(ValueError, match="expected docker or process"):
        load_orchestration_runtime_backend_config(runtime)


def test_load_runtime_backend_config_rejects_process_simulation_combo(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config_file = tmp_path / "config.toml"
    config_file.write_text(
        "",
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "simulation")
    runtime = RuntimeConfig(
        lab_root=tmp_path,
        ardupilot_root=tmp_path / "ardupilot",
        mavlink_router_root=tmp_path / "mavlink-router",
        venv_path=tmp_path / ".venv",
        config_file=config_file,
        config_loaded=True,
    )

    with pytest.raises(ValueError, match="docker\\+simulation and process\\+real"):
        load_orchestration_runtime_backend_config(runtime)


def test_runtime_mode_policy_blocks_simulation_service_in_real_mode(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src import host

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime]
fail_on_mode_violation = true

[orchestration.runtime.process]
require_explicit_services = false
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(
        config_path=config_file,
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path / "artifact",
    )

    with pytest.raises(RuntimeModeViolationError, match="runtime_mode_violation:simulation_stack_requested"):
        host._compose_up(config)


def test_real_mode_blocks_simulation_overlay_and_official_baseline(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src import host
    from src.tasks.helpers.sensors import _write_p2_model_overlay

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime]
fail_on_mode_violation = true

[orchestration.runtime.process]
require_explicit_services = false
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(
        config_path=config_file,
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path / "artifact",
    )

    with pytest.raises(RuntimeModeViolationError, match="runtime_mode_violation:sdf_overlay_requested"):
        _write_p2_model_overlay(config, tmp_path / "overlay.sdf")
    with pytest.raises(RuntimeModeViolationError, match="runtime_mode_violation:official_baseline_requested"):
        host._start_official_baseline_container(config)


def test_real_preflight_summary_does_not_gate_on_ros_topics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import DependencyProbe, SerialMavlinkProbe, _build_real_preflight_summary

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[orchestration.runtime.real.sources]
required_real_topics = ["/scan", "/tf"]
forbidden_simulation_input_topics = ["/gazebo/*", "/sim/x2/status"]
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(
        config_path=config_file,
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path / "artifact",
    )

    summary = _build_real_preflight_summary(
        config,
        serial_mavlink_probe=SerialMavlinkProbe(
            summary={"enabled": False, "heartbeat_seen": False},
            blockers=(),
        ),
        dependency_probe=DependencyProbe(
            summary={
                "required_command_groups": [{"found": True}],
                "required_ros_packages": {},
                "required_python_modules": {},
            },
            blockers=(),
        ),
    )

    assert summary["ok"] is True
    assert summary["blockers"] == []
    assert "required_real_topics" not in summary["real_preflight"]
    assert "forbidden_simulation_input_topics" not in summary["real_preflight"]


def test_real_preflight_summary_blocks_non_process_real_runtime(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import DependencyProbe, SerialMavlinkProbe, _build_real_preflight_summary

    config_file = tmp_path / "simulation.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "docker")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "simulation")
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    summary = _build_real_preflight_summary(
        config,
        serial_mavlink_probe=SerialMavlinkProbe(summary={"enabled": False, "heartbeat_seen": False}, blockers=()),
        dependency_probe=DependencyProbe(
            summary={
                "required_command_groups": [{"found": True}],
                "required_ros_packages": {},
                "required_python_modules": {},
            },
            blockers=(),
        ),
    )

    assert summary["ok"] is False
    assert "runtime_backend_must_be_process:docker" in summary["blockers"]
    assert "runtime_mode_must_be_real:simulation" in summary["blockers"]


def test_real_preflight_loads_serial_mavlink_task_config() -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.real.toml",
        task_name="real-preflight-doctor",
        run_id="20260609_000000",
    )

    serial_mavlink = config.orchestration.real_preflight.serial_mavlink
    assert config.orchestration.real_preflight.valid_for_sec == 300.0
    assert config.orchestration.real_preflight.ros_distro == "humble"
    assert serial_mavlink.enabled is True
    assert serial_mavlink.port == "/dev/ttyUSB1"
    assert serial_mavlink.baud == 115200
    assert serial_mavlink.heartbeat_timeout_sec == 15.0
    assert serial_mavlink.required_messages == ("HEARTBEAT", "SYS_STATUS", "ATTITUDE")
    assert "DISTANCE_SENSOR" in serial_mavlink.optional_messages
    dependencies = config.orchestration.real_preflight.dependencies
    assert dependencies.required_command_groups == (("mavlink-routerd", "mavlink-router"), ("ros2",))
    assert "mavros" not in dependencies.required_ros_packages
    assert "mavros_msgs" not in dependencies.required_ros_packages
    assert "cartographer_ros" in dependencies.required_ros_packages
    assert "navlab_slam_bringup" in dependencies.required_ros_packages
    assert "ydlidar_ros2_driver" in dependencies.required_ros_packages
    assert dependencies.required_python_modules == ("navlab.companion.cli", "navlab.slam.cli")


def test_real_preflight_effective_dependencies_use_fcu_bridge_mode() -> None:
    from src.tasks.real_preflight import _effective_real_preflight_dependencies

    config = RunConfig.from_config(
        config_path="orchestration/config.real.toml",
        task_name="real-preflight-doctor",
        run_id="20260609_000000",
    )

    dependencies, blockers = _effective_real_preflight_dependencies(config)

    assert blockers == ()
    assert dependencies.required_command_groups == (("mavlink-routerd", "mavlink-router"), ("ros2",))
    assert "cartographer_ros" in dependencies.required_ros_packages
    assert "navlab_external_nav_bridge" in dependencies.required_ros_packages
    assert "ydlidar_ros2_driver" in dependencies.required_ros_packages
    assert "mavros" not in dependencies.required_ros_packages
    assert "mavros_msgs" not in dependencies.required_ros_packages


def test_real_preflight_dependency_probe_checks_host_real_hover_chain(monkeypatch: pytest.MonkeyPatch) -> None:
    from src.config import RealPreflightDependencyConfig
    from src.tasks import real_preflight

    def fake_which(command: str) -> str | None:
        if command == "mavlink-routerd":
            return "/usr/bin/mavlink-routerd"
        return None

    monkeypatch.setattr(real_preflight.shutil, "which", fake_which)
    settings = RealPreflightDependencyConfig(
        required_command_groups=(("mavlink-routerd", "mavlink-router"), ("ros2",)),
        required_ros_packages=("mavros", "navlab_slam_bringup"),
        required_python_modules=("navlab.companion.cli", "navlab.slam.cli"),
        required_process_services=("companion", "slam"),
    )

    probe = real_preflight._probe_real_preflight_dependencies(
        settings,
        ros_distro="humble",
        process_service_names=("companion",),
    )

    assert probe.summary["required_command_groups"][0]["found"] is True
    assert probe.summary["required_command_groups"][1]["found"] is False
    assert probe.summary["required_python_modules"]["navlab.companion.cli"] is True
    assert probe.summary["ros_distro"] == "humble"
    assert probe.summary["required_ros_packages"]["mavros"]["error"] == "ros2_distro_not_found:humble"
    assert "required_command_missing:ros2" in probe.blockers
    assert "required_process_service_missing:slam" in probe.blockers


def test_real_preflight_serial_probe_blocks_missing_port(tmp_path: Path) -> None:
    from src.tasks.real_preflight import _probe_serial_mavlink

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[serial_mavlink]
enabled = true
port = "/tmp/navlab_missing_fcu_serial"
baud = 115200
""".strip(),
        encoding="utf-8",
    )
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    result = _probe_serial_mavlink(config.orchestration.real_preflight.serial_mavlink)

    assert result.summary["enabled"] is True
    assert result.summary["serial_open_ok"] is False
    assert result.blockers == ("serial_port_missing:/tmp/navlab_missing_fcu_serial",)


def test_real_preflight_serial_probe_rejects_network_endpoint(tmp_path: Path) -> None:
    from src.tasks.real_preflight import _probe_serial_mavlink

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[serial_mavlink]
enabled = true
port = "udp:127.0.0.1:14550"
baud = 115200
""".strip(),
        encoding="utf-8",
    )
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    result = _probe_serial_mavlink(config.orchestration.real_preflight.serial_mavlink)

    assert result.summary["enabled"] is True
    assert result.blockers == ("serial_mavlink_endpoint_not_serial:udp:127.0.0.1:14550",)


def test_real_preflight_blocks_serial_probe_without_topic_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import DependencyProbe, SerialMavlinkProbe, _build_real_preflight_summary

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[orchestration.runtime.real.sources]
required_real_topics = ["/scan", "/ap/v1/status", "/ap/v1/pose/filtered"]
forbidden_simulation_input_topics = ["/gazebo/*"]
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    summary = _build_real_preflight_summary(
        config,
        serial_mavlink_probe=SerialMavlinkProbe(
            summary={"enabled": True, "heartbeat_seen": False},
            blockers=("serial_mavlink_heartbeat_missing",),
        ),
        dependency_probe=DependencyProbe(
            summary={
                "required_command_groups": [{"found": True}],
                "required_ros_packages": {},
                "required_python_modules": {},
            },
            blockers=(),
        ),
    )

    assert summary["ok"] is False
    assert "serial_mavlink_heartbeat_missing" in summary["blockers"]
    assert "fcu_topic_not_backed_by_serial_mavlink" not in summary["blockers"]


def test_real_preflight_allows_open_serial_before_prepare_heartbeat(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import DependencyProbe, SerialMavlinkProbe, _build_real_preflight_summary

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[real_prepare]
fcu_bridge_mode = "navlab_mavlink"
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    summary = _build_real_preflight_summary(
        config,
        serial_mavlink_probe=SerialMavlinkProbe(
            summary={"enabled": True, "serial_open_ok": True, "heartbeat_seen": False},
            blockers=(
                "serial_mavlink_heartbeat_missing",
                "serial_mavlink_required_message_missing:SYS_STATUS",
            ),
        ),
        dependency_probe=DependencyProbe(
            summary={
                "required_command_groups": [{"found": True}],
                "required_ros_packages": {},
                "required_python_modules": {},
            },
            blockers=(),
        ),
    )

    assert summary["ok"] is True
    assert summary["blockers"] == []
    assert summary["warnings"] == [
        "serial_mavlink_heartbeat_missing",
        "serial_mavlink_required_message_missing:SYS_STATUS",
    ]


def test_real_preflight_summary_schema_for_successful_serial_probe(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import DependencyProbe, SerialMavlinkProbe, _build_real_preflight_summary

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[orchestration.runtime.real.sources]
required_real_topics = ["/scan", "/tf"]
forbidden_simulation_input_topics = ["/gazebo/*"]

[real_preflight]
valid_for_sec = 120

[fcu_controller]
takeoff_alt_m = 0.4
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    serial_summary = {
        "enabled": True,
        "port": "/dev/ttyUSB1",
        "baud": 115200,
        "serial_open_ok": True,
        "heartbeat_seen": True,
        "system_id": 1,
        "component_id": 1,
        "autopilot": "MAV_AUTOPILOT_ARDUPILOTMEGA",
        "vehicle_type": "MAV_TYPE_QUADROTOR",
        "armed": False,
        "mode": "STABILIZE",
        "message_counts": {"HEARTBEAT": 1, "SYS_STATUS": 1, "ATTITUDE": 3},
    }
    summary = _build_real_preflight_summary(
        config,
        serial_mavlink_probe=SerialMavlinkProbe(summary=serial_summary, blockers=()),
        dependency_probe=DependencyProbe(
            summary={
                "required_command_groups": [{"found": True}, {"found": True}],
                "required_ros_packages": {"mavros": {"present": True}},
                "required_python_modules": {"navlab.companion.cli": True, "navlab.slam.cli": True},
            },
            blockers=(),
        ),
    )

    assert summary["ok"] is True
    assert summary["preflight_claim"] == "evaluated"
    assert summary["flight_claim"] == "not_evaluated"
    assert summary["landing_claim"] == "not_evaluated"
    assert summary["valid_for_sec"] == 120.0
    assert summary["real_preflight"]["serial_mavlink"]["heartbeat_seen"] is True


def test_real_preflight_softens_installable_ros_dependency_warnings() -> None:
    from src.tasks.real_preflight import _soften_installable_dependency_blockers

    summary = {
        "ok": False,
        "blocked": True,
        "blockers": ["required_command_missing:ros2", "required_ros_package_missing:mavros"],
        "real_preflight": {
            "dependencies": {
                "required_ros_packages": {"mavros": {"present": False}},
            },
        },
    }

    _soften_installable_dependency_blockers(summary)

    assert summary["ok"] is True
    assert summary["blocked"] is False
    assert summary["blockers"] == []
    assert summary["warnings"] == ["required_command_missing:ros2", "required_ros_package_missing:mavros"]
    assert summary["preflight_blockers"] == ["required_command_missing:ros2", "required_ros_package_missing:mavros"]


def test_real_preflight_dependency_install_uses_configured_ros_distro(monkeypatch: pytest.MonkeyPatch) -> None:
    from src.tasks import real_preflight

    commands: list[list[str]] = []

    def fake_run_install_command(command, *, console, result):  # noqa: ANN001
        commands.append(command)
        result["commands"].append(command)

    monkeypatch.setattr(real_preflight, "_run_install_command", fake_run_install_command)
    monkeypatch.setattr(real_preflight, "_elevated_command", lambda command: [command])
    monkeypatch.setattr(
        real_preflight.shutil,
        "which",
        lambda command: "/usr/bin/colcon" if command == "colcon" else None,
    )

    result = real_preflight._install_missing_real_preflight_dependencies(
        ["mavros", "mavros_msgs", "navlab_slam_bringup"],
        ros_distro="humble",
        console=Console(file=StringIO(), force_terminal=False),
    )

    assert result["ok"] is True
    assert result["ros_distro"] == "humble"
    assert any("ros-humble-mavros" in command for command in commands)
    assert any("ros-humble-mavros-msgs" in command for command in commands)
    assert not any("ros-jazzy" in item for command in commands for item in command)


def test_real_preflight_console_summary_prints_operator_keys(tmp_path: Path) -> None:
    from src.tasks.real_preflight import _print_real_preflight_console_summary

    output = StringIO()
    console = Console(file=output, force_terminal=False, width=120)
    summary = {
        "runtime_backend": "process",
        "runtime_mode": "real",
        "ros_distro": "humble",
        "blockers": ["serial_mavlink_heartbeat_missing"],
        "real_preflight": {
            "dependencies": {
                "required_command_groups": [{"found": True}, {"found": False}],
                "required_ros_packages": {"mavros": {"present": False}},
                "required_python_modules": {"navlab.companion.cli": True, "navlab.slam.cli": True},
            },
            "serial_mavlink": {
                "port": "/dev/ttyUSB1",
                "baud": 115200,
                "serial_open_ok": True,
                "heartbeat_seen": False,
                "system_id": 1,
                "component_id": 1,
                "autopilot": "MAV_AUTOPILOT_ARDUPILOTMEGA",
                "mode": "STABILIZE",
                "armed": False,
            },
        },
    }

    _print_real_preflight_console_summary(console, summary=summary, summary_path=tmp_path / "summary.json")

    text = output.getvalue()
    assert "Real Preflight Doctor" in text
    assert "Runtime" in text
    assert "ROS distro" in text
    assert "humble" in text
    assert "process+real" in text
    assert "Serial" in text
    assert "/dev/ttyUSB1 @ 115200" in text
    assert "heartbeat=False" in text
    assert "system=1, component=1" in text
    assert "Deps" in text
    assert "cmd=1/2" in text
    assert "ros=0/1" in text
    assert "py=2/2" in text
    assert "Blockers" in text
    assert "serial_mavlink_heartbeat_missing" in text
    assert str(tmp_path / "summary.json") in text


def test_real_preflight_summary_does_not_gate_on_source_claims(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import DependencyProbe, SerialMavlinkProbe, _build_real_preflight_summary

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime.process]
require_explicit_services = false

[orchestration.runtime.real.sources]
scan_source_claim = "real_lidar_driver"
scan_source_topic = "/lidar"
fcu_source_claim = "simulation_sitl_dds"
required_real_topics = ["/lidar"]
forbidden_simulation_input_topics = ["/gazebo/*"]
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.setenv("NAVLAB_RUNTIME_BACKEND", "process")
    monkeypatch.setenv("NAVLAB_RUNTIME_MODE", "real")
    config = RunConfig.from_config(config_path=config_file, run_id="20260609_000000")

    summary = _build_real_preflight_summary(
        config,
        serial_mavlink_probe=SerialMavlinkProbe(
            summary={"enabled": False, "heartbeat_seen": False},
            blockers=(),
        ),
        dependency_probe=DependencyProbe(
            summary={
                "required_command_groups": [{"found": True}],
                "required_ros_packages": {},
                "required_python_modules": {},
            },
            blockers=(),
        ),
    )

    assert summary["ok"] is True
    assert summary["blockers"] == []
    assert "source_claims" not in summary


def test_process_backend_merges_explicit_env(tmp_path: Path) -> None:
    backend = ProcessBackend(default_log_dir=tmp_path)
    result = backend.run_probe(
        ProbeSpec(
            name="env",
            command=(sys.executable, "-c", "import os; print(os.environ['NAVLAB_TEST_ENV'])"),
            env={"NAVLAB_TEST_ENV": "visible"},
            log_path=tmp_path / "env.log",
        )
    )

    assert result.stdout.strip() == "visible"
    assert os.environ.get("NAVLAB_TEST_ENV") is None


def test_capture_compose_service_log_uses_runtime_backend(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from types import SimpleNamespace

    from src import host
    from src.project_config import ComposeConfig, ValueWithSource

    calls: list[tuple[str, int]] = []

    class FakeBackend:
        def logs(self, handle, *, tail=400):  # noqa: ANN001
            calls.append((handle, tail))
            return "backend log"

    monkeypatch.setattr(host, "DockerBackend", FakeBackend)
    monkeypatch.setattr(host, "load_runtime_config", lambda: SimpleNamespace(lab_root=tmp_path))
    monkeypatch.setattr(
        host,
        "load_compose_config",
        lambda _runtime: ComposeConfig(
            compose_file=tmp_path / "compose.yaml",
            project_name=ValueWithSource("navlab", "test"),
            default_profile=ValueWithSource("base_env", "test"),
        ),
    )
    output_path = tmp_path / "sitl.log"

    host._capture_compose_service_log(config=SimpleNamespace(), service="sitl", output_path=output_path, tail=17)

    assert calls == [("navlab-sitl-1", 17)]
    assert output_path.read_text(encoding="utf-8") == "backend log"


def test_p10_rosbag_start_uses_runtime_backend(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config = RunConfig.from_config(
        config_path="orchestration/config.simulation.toml",
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path,
    )
    started: list[ServiceSpec] = []

    class FakeBackend:
        def start_service(self, spec: ServiceSpec) -> None:
            started.append(spec)

    monkeypatch.setattr(scan_integrity_gate, "DockerBackend", FakeBackend)

    scan_integrity_gate._start_p10_rosbag_recording(config, duration_sec=5.0)

    assert len(started) == 1
    spec = started[0]
    assert spec.name == "p10_rosbag"
    assert spec.container_name == scan_integrity_gate.P10_ROSBAG_CONTAINER
    assert spec.image == "world-model/navlab-official-baseline:latest"
    assert spec.networks == ("host",)
    assert spec.cwd == "/workspace"
    assert spec.env["PYTHONPATH"] == "/workspace"


def test_docker_backend_run_probe_uses_probe_spec_image_network_and_volume(tmp_path: Path) -> None:
    fake = FakeDockerClient()
    backend = DockerBackend(client_factory=lambda: fake)

    result = backend.run_probe(
        ProbeSpec(
            name="probe",
            image="navlab/probe:latest",
            command=("bash", "-lc", "echo probe"),
            container_name="navlab-probe",
            networks=("host",),
            volumes=(VolumeMount(tmp_path, "/workspace"),),
            cwd="/workspace",
            env={"ROS_DOMAIN_ID": "85"},
        )
    )

    assert result.ok is True
    assert fake.runs[0]["image"] == "navlab/probe:latest"
    assert fake.runs[0]["name"] == "navlab-probe"
    assert fake.runs[0]["networks"] == ["host"]
    assert fake.runs[0]["volumes"] == [(tmp_path, "/workspace")]
    assert fake.runs[0]["envs"] == {"ROS_DOMAIN_ID": "85"}


def test_host_ros_shell_capture_uses_probe_spec(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    from src import host

    config = RunConfig.from_config(
        config_path="orchestration/config.simulation.toml",
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path,
    )
    captured: list[ProbeSpec] = []

    class FakeBackend:
        def run_probe(self, spec: ProbeSpec):
            captured.append(spec)
            return type("Result", (), {"return_code": 0, "stdout": "probe ok"})()

    monkeypatch.setattr(host, "DockerBackend", FakeBackend)

    rc, output = host._docker_run_ros_shell_capture(
        config=config,
        image="world-model/navlab-official-baseline:latest",
        shell_command="ros2 topic list",
        name="topic-probe",
        network="host",
        envs={"EXTRA": "1"},
    )

    assert rc == 0
    assert output == "probe ok"
    assert len(captured) == 1
    spec = captured[0]
    assert spec.image == "world-model/navlab-official-baseline:latest"
    assert spec.container_name == "topic-probe"
    assert spec.networks == ("host",)
    assert spec.volumes == (VolumeMount(Path.cwd(), "/workspace"),)
    assert spec.env["EXTRA"] == "1"
    assert "ros2 topic list" in spec.command[2]


def test_p11_rosbag_start_uses_runtime_backend(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from src.tasks.helpers import scan_stabilization as scan_stabilization_gate

    config = RunConfig.from_config(
        config_path="orchestration/config.simulation.toml",
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path,
    )
    started: list[ServiceSpec] = []

    class FakeBackend:
        def start_service(self, spec: ServiceSpec) -> None:
            started.append(spec)

    monkeypatch.setattr(scan_stabilization_gate, "DockerBackend", FakeBackend)

    scan_stabilization_gate._start_p11_rosbag_recording(config, duration_sec=5.0)

    assert len(started) == 1
    spec = started[0]
    assert spec.name == "p11_rosbag"
    assert spec.container_name == scan_stabilization_gate.P11_ROSBAG_CONTAINER
    assert spec.image == "world-model/navlab-official-baseline:latest"
    assert spec.networks == ("host",)
    assert spec.cwd == "/workspace"
    assert spec.env["PYTHONPATH"] == "/workspace"


def test_p10_p11_p12_doctor_summaries_include_runtime_backend(tmp_path: Path) -> None:
    from src.tasks.helpers.scan_integrity import _build_p10_doctor_summary
    from src.tasks.helpers.scan_stabilization import _build_p11_doctor_summary
    from src.tasks.workflows.scan_robustness import _build_p12_doctor_summary

    config = RunConfig.from_config(
        config_path="orchestration/config.simulation.toml",
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path,
    )

    p10 = _build_p10_doctor_summary(config, runtime_config=tmp_path / "p10.toml", include_dependencies=False)
    p11 = _build_p11_doctor_summary(config, runtime_config=tmp_path / "p11.toml", include_dependencies=False)
    p12 = _build_p12_doctor_summary(config, runtime_config=tmp_path / "p12.toml")

    for summary in (p10, p11, p12):
        assert summary["runtime_backend"] == "docker"
        assert summary["runtime_mode"] == "simulation"
        assert summary["runtime_backend_summary"]["backend"] == "docker"
        assert summary["runtime_backend_summary"]["mode"] == "simulation"
        assert summary["runtime_backend_summary"]["backend_config_path"] == "orchestration/config.simulation.toml"
        assert summary["source_claims"]["scan"] == "gazebo_lidar_via_x2"
        assert summary["source_claims"]["scan_topic"] == "/scan"
