from __future__ import annotations

from typing import Annotated, cast

import typer
from rich.console import Console

from src.tasks.build import BuildTask, ImageKind
from src.tasks.built_in.exploration import BuiltInExplorationDoctorTask, BuiltInExplorationTask
from src.tasks.built_in.scan_robustness import BuiltInScanRobustnessDoctorTask, BuiltInScanRobustnessTask
from src.tasks.doctor import DoctorTask
from src.tasks.hover import HoverAcceptanceTask
from src.tasks.real_preflight import RealPreflightDoctorTask
from src.tasks.registry import TaskRegistry

app = typer.Typer(add_completion=False, no_args_is_help=True)
console = Console()


@app.command("build")
def image_build_command(
    kind: Annotated[
        str,
        typer.Argument(help="Image to build: companion, slam, gazebo-sensor, official-baseline, or all"),
    ] = "all",
    tag: Annotated[
        str | None,
        typer.Option("--tag", help="Override the configured NavLab image tag strategy"),
    ] = None,
) -> None:
    task = cast(BuildTask, TaskRegistry.create("build"))
    raise typer.Exit(task.run(kind=cast(ImageKind, kind), tag=tag, console=console))


@app.command("doctor")
def companion_doctor_command(
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
) -> None:
    task = cast(DoctorTask, TaskRegistry.create("doctor"))
    raise typer.Exit(task.run(config_path=orchestration_config, console=console))


@app.command("hover")
def hover_acceptance_command(
    duration_sec: Annotated[
        float | None,
        typer.Option("--duration-sec", help="Hover acceptance duration in seconds"),
    ] = None,
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="Hover task TOML path"),
    ] = None,
    simulation_profile: Annotated[
        str | None,
        typer.Option(
            "--simulation-profile",
            help="Gazebo/SITL Stage 1 profile: ideal or mild_disturbance",
        ),
    ] = None,
) -> None:
    task = cast(HoverAcceptanceTask, TaskRegistry.create("hover"))
    raise typer.Exit(
        task.run(
            config_path=orchestration_config,
            task_config_path=task_config,
            duration_sec=duration_sec,
            simulation_profile=simulation_profile,
            console=console,
        )
    )


@app.command("exploration-doctor")
def exploration_doctor_command(
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="P8 task TOML path"),
    ] = None,
) -> None:
    task = cast(BuiltInExplorationDoctorTask, TaskRegistry.create("exploration-doctor"))
    raise typer.Exit(task.run(config_path=orchestration_config, task_config_path=task_config, console=console))


@app.command("exploration")
def exploration_command(
    duration_sec: Annotated[
        float | None,
        typer.Option("--duration-sec", help="Built-in P8 movement/exploration duration in seconds"),
    ] = None,
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="P8 task TOML path"),
    ] = None,
    simulation_profile: Annotated[
        str | None,
        typer.Option(
            "--simulation-profile",
            help="Gazebo/SITL Stage 1 profile: ideal or mild_disturbance",
        ),
    ] = None,
) -> None:
    task = cast(BuiltInExplorationTask, TaskRegistry.create("exploration"))
    raise typer.Exit(
        task.run(
            config_path=orchestration_config,
            task_config_path=task_config,
            duration_sec=duration_sec,
            simulation_profile=simulation_profile,
            console=console,
        )
    )


@app.command("scan-robustness-doctor")
def scan_robustness_doctor_command(
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="P12 task TOML path"),
    ] = None,
) -> None:
    task = cast(BuiltInScanRobustnessDoctorTask, TaskRegistry.create("scan-robustness-doctor"))
    raise typer.Exit(task.run(config_path=orchestration_config, task_config_path=task_config, console=console))


@app.command("scan-robustness")
def scan_robustness_command(
    duration_sec: Annotated[
        float | None,
        typer.Option("--duration-sec", help="Built-in tilted-scan robustness duration budget in seconds"),
    ] = None,
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="P12 task TOML path"),
    ] = None,
    live: Annotated[
        bool | None,
        typer.Option(
            "--live/--profile-sweep-only",
            help="Run disturbed P8/P9 movement replay through scan robustness instead of profile sweep only",
        ),
    ] = None,
    live_profiles: Annotated[
        str | None,
        typer.Option("--live-profiles", help="Comma-separated disturbance profiles for live replay"),
    ] = None,
) -> None:
    task = cast(BuiltInScanRobustnessTask, TaskRegistry.create("scan-robustness"))
    profiles = None if live_profiles is None else tuple(item.strip() for item in live_profiles.split(",") if item.strip())
    raise typer.Exit(
        task.run(
            config_path=orchestration_config,
            task_config_path=task_config,
            duration_sec=duration_sec,
            live_replay=live,
            live_profiles=profiles,
            console=console,
        )
    )


@app.command("real-preflight-doctor")
def real_preflight_doctor_command(
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="Real preflight task TOML path"),
    ] = None,
) -> None:
    task = cast(RealPreflightDoctorTask, TaskRegistry.create("real-preflight-doctor"))
    raise typer.Exit(task.run(config_path=orchestration_config, task_config_path=task_config, console=console))


if __name__ == "__main__":
    app()
