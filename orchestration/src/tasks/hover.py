from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

from python_on_whales.exceptions import DockerException
from rich.console import Console

from src import host
from src.artifacts import finalize_navlab_artifact
from src.config import RunConfig
from src.foxglove_upload import upload_acceptance_rosbag
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class HoverAcceptanceTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "hover"
    TASK_DESCRIPTION: ClassVar[str] = "Run NavLab FCU GUIDED/arm/takeoff/hover acceptance."

    def run(
        self,
        *,
        config_path: str | Path | None = None,
        duration_sec: float = 90.0,
        console: Console | None = None,
    ) -> int:
        console = console or Console()
        config = RunConfig.from_config(config_path=config_path, duration_sec=duration_sec)
        config.artifact_dir.mkdir(parents=True, exist_ok=True)
        host._render_run_config(console, config)
        rc = 1
        try:
            console.print("[bold cyan]Starting SITL + Gazebo + companion stack[/bold cyan]")
            host._compose_up(config)
            host._start_slam_container(config)
            host._start_companion_container(config)
            console.print(f"[bold cyan]Running hover acceptance inside companion for {duration_sec:g}s[/bold cyan]")
            rc = host._docker_exec_runtime_command(
                module="navlab.companion.acceptance_cli",
                args=[
                    "execute-hover-acceptance",
                    "--artifact-dir",
                    str(config.artifact_dir),
                    "--duration-sec",
                    str(duration_sec),
                    "--rosbag-profile",
                    config.rosbag_profile,
                    "--companion-image",
                    config.companion_image,
                    "--scan-source",
                    config.scan_source,
                    "--config",
                    host._companion_runtime_config_path(),
                ],
            )
            host._capture_compose_service_log(
                config=config,
                service="sitl",
                output_path=config.artifact_dir / "sitl.log",
            )
        finally:
            host._capture_stack_logs(config=config)
            finalize_navlab_artifact(
                artifact_dir=config.artifact_dir,
                session_id=config.session_id,
                run_id=config.run_id,
                duration_sec=duration_sec,
                ros_domain_id=config.ros_domain_id,
                rosbag_profile=config.rosbag_profile,
                session_log_dir=host._session_log_dir(config),
                stage_label="FCU ExternalNav hover acceptance",
                control_mode="companion_mavlink_guided_arm_takeoff_hover",
            )
            host._remove_companion_container()
            host._remove_slam_container()
            try:
                host._compose_stop(config)
            except DockerException:
                pass
            upload = upload_acceptance_rosbag(config)
            upload_color = "green" if upload.ok else "yellow"
            console.print(f"[{upload_color}]Foxglove upload:[/{upload_color}] {upload.state} ({upload.reason})")
        color = "green" if rc == 0 else "red"
        console.print(f"[{color}]NavLab hover acceptance completed rc={rc}[/{color}]")
        console.print(f"[bold]Summary:[/bold] {config.artifact_dir / 'summary.json'}")
        console.print(f"[bold]SITL log:[/bold] {config.artifact_dir / 'sitl.log'}")
        return rc
