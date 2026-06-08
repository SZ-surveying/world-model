from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest
from python_on_whales.exceptions import DockerException
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
[orchestration.runtime]
backend = "process"
mode = "real"

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
        """
[orchestration.runtime]
backend = "process"
mode = "simulation"
""".strip(),
        encoding="utf-8",
    )
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
backend = "process"
mode = "real"
fail_on_mode_violation = true

[orchestration.runtime.process]
require_explicit_services = false
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.delenv("NAVLAB_RUNTIME_BACKEND", raising=False)
    monkeypatch.delenv("NAVLAB_RUNTIME_MODE", raising=False)
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
backend = "process"
mode = "real"
fail_on_mode_violation = true

[orchestration.runtime.process]
require_explicit_services = false
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.delenv("NAVLAB_RUNTIME_BACKEND", raising=False)
    monkeypatch.delenv("NAVLAB_RUNTIME_MODE", raising=False)
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


def test_real_preflight_summary_checks_required_and_forbidden_topics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.tasks.real_preflight import _build_real_preflight_summary

    config_file = tmp_path / "real.toml"
    config_file.write_text(
        """
[orchestration.runtime]
backend = "process"
mode = "real"

[orchestration.runtime.process]
require_explicit_services = false

[orchestration.runtime.real.sources]
required_real_topics = ["/scan", "/tf"]
forbidden_simulation_input_topics = ["/gazebo/*"]
""".strip(),
        encoding="utf-8",
    )
    monkeypatch.delenv("NAVLAB_RUNTIME_BACKEND", raising=False)
    monkeypatch.delenv("NAVLAB_RUNTIME_MODE", raising=False)
    config = RunConfig.from_config(
        config_path=config_file,
        duration_sec=45.0,
        run_id="20260608_000000",
        artifact_dir=tmp_path / "artifact",
    )

    summary = _build_real_preflight_summary(
        config,
        topics=("/scan", "/gazebo/default/world_stats"),
        topic_probe_error=None,
    )

    assert summary["ok"] is False
    assert "required_real_topic_missing:/tf" in summary["blockers"]
    assert "forbidden_simulation_topic_present:/gazebo/default/world_stats" in summary["blockers"]


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
        config_path="orchestration/config.toml",
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
        config_path="orchestration/config.toml",
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
        config_path="orchestration/config.toml",
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
    from src.tasks.workflows.scan_robustness import _build_p12_doctor_summary
    from src.tasks.helpers.scan_integrity import _build_p10_doctor_summary
    from src.tasks.helpers.scan_stabilization import _build_p11_doctor_summary

    config = RunConfig.from_config(
        config_path="orchestration/config.toml",
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
        assert summary["runtime_backend_summary"]["backend_config_path"] == "orchestration/config.toml"
        assert summary["source_claims"]["scan"] == "gazebo_lidar_via_x2"
        assert summary["source_claims"]["scan_topic"] == "/scan"
