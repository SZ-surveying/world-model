from __future__ import annotations

import io
from pathlib import Path

from rich.console import Console

from lab_env.config import load_navlab_images_config, load_runtime_config, resolve_navlab_image_tag
from lab_env.navlab.orchestration import host
from lab_env.navlab.orchestration.config import (
    NAVLAB_SERVICES,
    NAVLAB_STOP_SERVICES,
    OrchestrationConfig,
    RunConfig,
)
from lab_env.navlab.runtime.companion import CompanionLauncher
from lab_env.navlab.runtime.config import NodeConfig, RuntimeConfig


def test_navlab_runtime_config_loads_companion_nodes_from_toml() -> None:
    config = RuntimeConfig.load("profiles/navlab-gazebo.toml")

    assert config.imu_source_label == "fcu_mavlink_navlab"
    assert config.world_markers.autostart is True
    assert "/sim/markers" in config.world_markers.args
    assert config.scan_features.autostart is True
    assert config.pose_mirror.autostart is True
    assert config.pose_mirror.endpoint == "tcp:mavlink-router:5760"
    assert "--simulate-pose-from-mission-status" in config.pose_mirror.args
    assert config.external_nav_sender.args == ("--rate-hz", "20")
    assert config.mission.args == ("--simulate-mode-arm",)


def test_navlab_compose_env_contains_only_compose_level_config() -> None:
    config = OrchestrationConfig.load("profiles/navlab-gazebo.toml")
    image_config = load_navlab_images_config(load_runtime_config())
    env = config.compose_env()

    assert env["NAVLAB_CONFIG"] == "profiles/navlab-gazebo.toml"
    assert env["NAVLAB_COMPANION_IMAGE"] == image_config.companion.image()
    assert env["NAVLAB_SLAM_IMAGE"] == image_config.slam.image()
    assert env["NAVLAB_GAZEBO_SENSOR_IMAGE"] == image_config.gazebo_sensor.image()
    assert env["X2_MODE"] == "runtime"
    assert env["X2_SCAN_SOURCE"] == "x2_virtual_serial"
    assert env["X2_SCAN_IDEAL_TOPIC"] == "/scan_ideal"
    assert env["X2_SCAN_TOPIC"] == "/scan"
    assert env["X2_STATUS_TOPIC"] == "/sim/x2/status"
    assert env["SITL_IMAGE"] == "remote-sitl-lab/ardupilot-sitl:stage1-f10500ae45aa"
    assert "NAVLAB_POSE_MIRROR_EXTRA_ARGS" not in env
    assert "NAVLAB_MISSION_EXTRA_ARGS" not in env
    assert "SIM_UP_MODE" not in env
    assert "SIM_AUTO_ROSBAG_ENABLED" not in env
    assert config.foxglove_upload.enabled is True
    assert config.foxglove_upload.token_env == "FOXGLOVE_API_TOKEN"
    assert config.foxglove_upload.device_name == "navlab_companion_sitl_gazebo"
    assert config.slam.autostart is True
    assert config.slam.backend == "cartographer"
    assert config.slam.imu_source_topic == "/navlab/fcu_imu/data"
    assert config.sensor.scan_source == "x2_virtual_serial"
    assert config.sensor.acceptance_scan_source == "x2_virtual_serial_vendor_driver"


def test_navlab_images_config_drives_default_run_images() -> None:
    image_config = load_navlab_images_config(load_runtime_config())
    orchestration = OrchestrationConfig.load("profiles/navlab-gazebo.toml")

    assert image_config.companion.dockerfile.value == "docker/Dockerfile.companion"
    assert image_config.companion.context.value == "."
    assert image_config.companion.target.value == "navlab-companion"
    assert image_config.companion.repository.value == "world-model/navlab-companion"
    assert image_config.companion.tag_strategy.value == "latest"
    assert image_config.companion.image() == "world-model/navlab-companion:latest"
    assert image_config.slam.target.value == "navlab-slam-cartographer"
    assert image_config.slam.repository.value == "world-model/navlab-slam-cartographer"
    assert image_config.slam.image() == "world-model/navlab-slam-cartographer:latest"
    assert image_config.gazebo_sensor.target.value == "navlab-gazebo-sensor"
    assert image_config.gazebo_sensor.repository.value == "world-model/navlab-gazebo-sensor"
    assert image_config.gazebo_sensor.image() == "world-model/navlab-gazebo-sensor:latest"
    assert orchestration.companion_image == image_config.companion.image()
    assert orchestration.slam.image == image_config.slam.image()
    assert orchestration.sensor.image == image_config.gazebo_sensor.image()


def test_navlab_image_tag_cli_override_wins_over_strategy() -> None:
    image_config = load_navlab_images_config(load_runtime_config())

    assert resolve_navlab_image_tag("latest") == "latest"
    assert image_config.companion.image(cli_tag="manual-tag") == "world-model/navlab-companion:manual-tag"


def test_navlab_compose_services_do_not_include_sim_runtime() -> None:
    assert "sim-runtime" not in NAVLAB_SERVICES
    assert "sim-runtime" not in NAVLAB_STOP_SERVICES
    assert "scan-bridge" not in NAVLAB_SERVICES
    assert "gazebo-sensor" in NAVLAB_SERVICES


def test_navlab_run_config_is_derived_from_profile() -> None:
    config = RunConfig.from_profile(
        profile_path="profiles/navlab-gazebo.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
    )

    assert config.duration_sec == 45.0
    assert config.run_id == "20260603_000000"
    assert config.artifact_dir.as_posix() == "artifacts/ros/navlab_companion_sitl_gazebo/20260603_000000"


def test_companion_launcher_autostarts_sim_marker_and_scan_nodes(monkeypatch) -> None:
    config = RuntimeConfig(
        path=Path("profile.toml"),
        imu_source_label="fcu",
        world_markers=NodeConfig(autostart=True, args=("--topic", "/sim/markers")),
        scan_features=NodeConfig(autostart=True, args=("--features-topic", "/scan_features")),
        pose_mirror=NodeConfig(autostart=False),
        imu_bridge=NodeConfig(autostart=False),
        external_nav_sender=NodeConfig(autostart=False),
        mission=NodeConfig(autostart=False),
    )
    started: list[tuple[str, list[str]]] = []

    def fake_start_function(self: CompanionLauncher, name, target, argv):  # noqa: ANN001
        started.append((name, argv))

    monkeypatch.setattr(CompanionLauncher, "_start_function", fake_start_function)

    CompanionLauncher(config)._start_configured_processes()

    assert started == [
        ("world_marker_publisher", ["--topic", "/sim/markers"]),
        ("scan_features_publisher", ["--features-topic", "/scan_features"]),
    ]


def test_navlab_orchestration_starts_slam_container(monkeypatch) -> None:
    config = RunConfig.from_profile(
        profile_path="profiles/navlab-gazebo.toml",
        duration_sec=45.0,
        run_id="20260603_000000",
    )
    runs: list[dict[str, object]] = []

    class FakeDockerClient:
        def run(self, image, command, **kwargs):  # noqa: ANN001
            runs.append({"image": image, "command": command, **kwargs})

    monkeypatch.setattr(host, "DockerClient", FakeDockerClient)
    monkeypatch.setattr(host, "_gazebo_network_namespace", lambda: "container:gazebo-id")
    monkeypatch.setattr(host, "_remove_slam_container", lambda: None)

    host._start_slam_container(config)

    assert len(runs) == 1
    run = runs[0]
    assert run["image"] == "world-model/navlab-slam-cartographer:latest"
    assert run["name"] == "navlab-slam"
    assert run["networks"] == ["container:gazebo-id"]
    command = " ".join(run["command"])
    assert "ros2 launch indoor_bringup indoor_bringup.launch.py" in command
    assert "imu_source_topic:=/navlab/fcu_imu/data" in command


def test_navlab_image_build_uses_global_image_config(monkeypatch) -> None:
    builds: list[dict[str, object]] = []

    class FakeDockerClient:
        def build(self, context_path, **kwargs):  # noqa: ANN001
            builds.append({"context_path": context_path, **kwargs})
            return iter(["build log"])

    monkeypatch.setattr(host, "DockerClient", FakeDockerClient)
    console = Console(file=io.StringIO(), width=120)

    rc = host.build_navlab_images(kind="companion", tag="cli-tag", console=console)

    assert rc == 0
    assert len(builds) == 1
    build = builds[0]
    assert Path(build["context_path"]).resolve() == Path.cwd().resolve()
    assert Path(build["file"]).resolve() == (Path.cwd() / "docker/Dockerfile.companion").resolve()
    assert build["target"] == "navlab-companion"
    assert build["tags"] == "world-model/navlab-companion:cli-tag"
    assert build["stream_logs"] is True


def test_navlab_image_build_supports_gazebo_sensor(monkeypatch) -> None:
    builds: list[dict[str, object]] = []

    class FakeDockerClient:
        def build(self, context_path, **kwargs):  # noqa: ANN001
            builds.append({"context_path": context_path, **kwargs})
            return iter(["build log"])

    monkeypatch.setattr(host, "DockerClient", FakeDockerClient)
    console = Console(file=io.StringIO(), width=120)

    rc = host.build_navlab_images(kind="gazebo-sensor", tag="cli-tag", console=console)

    assert rc == 0
    assert len(builds) == 1
    build = builds[0]
    assert build["target"] == "navlab-gazebo-sensor"
    assert build["tags"] == "world-model/navlab-gazebo-sensor:cli-tag"
