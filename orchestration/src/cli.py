from __future__ import annotations

import os
from typing import Annotated, cast

import typer
from rich.console import Console

from src.tasks.build import BuildTask, ImageKind
from src.tasks.doctor import DoctorTask
from src.tasks.real_prepare import execute_real_prepare_phase, run_real_task_doctor, stop_real_prepare_phase
from src.tasks.registry import TaskRegistry

app = typer.Typer(add_completion=False, no_args_is_help=True)
console = Console()

RUN_TASKS = ("hover", "exploration", "scan-robustness")


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
def runtime_doctor_command(
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="Real preflight task TOML path when backend/mode is process+real"),
    ] = None,
    force: Annotated[
        bool,
        typer.Option("--force", help="Install missing doctor dependencies without confirmation."),
    ] = False,
) -> None:
    task = cast(DoctorTask, TaskRegistry.create("doctor"))
    raise typer.Exit(
        task.run(
            config_path=orchestration_config,
            task_config_path=task_config,
            console=console,
            prompt_install=True,
            force_install=force,
            soft_dependency_warnings=True,
        )
    )


@app.command("run")
def run_task_command(
    task_name: Annotated[
        str,
        typer.Argument(help="Built-in task to run: hover, exploration, or scan-robustness"),
    ],
    duration_sec: Annotated[
        float | None,
        typer.Option("--duration-sec", help="Task duration budget in seconds"),
    ] = None,
    orchestration_config: Annotated[
        str | None,
        typer.Option("--orchestration-config", "--config", help="NavLab orchestration TOML path"),
    ] = None,
    task_config: Annotated[
        str | None,
        typer.Option("--task-config", help="Built-in task TOML path"),
    ] = None,
    simulation_profile: Annotated[
        str | None,
        typer.Option(
            "--simulation-profile",
            help="Gazebo/SITL Stage 1 profile for hover/exploration: ideal or mild_disturbance",
        ),
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
    dry_run: Annotated[
        bool,
        typer.Option("--dry-run", help="Show what the wrapper would run without starting the task champion."),
    ] = False,
) -> None:
    normalized = task_name.strip()
    if normalized not in RUN_TASKS:
        supported = ", ".join(RUN_TASKS)
        console.print(f"[red]Unknown built-in task:[/red] {task_name}. Supported tasks: {supported}")
        raise typer.Exit(2)

    backend_mode = _runtime_backend_mode_from_env()

    if backend_mode == ("process", "real"):
        doctor = cast(DoctorTask, TaskRegistry.create("doctor"))
        rc = doctor.run(config_path=orchestration_config, task_config_path=None, console=console)
        if rc != 0:
            raise typer.Exit(rc)
        prepare_result = execute_real_prepare_phase(
            task_name=normalized,
            config_path=orchestration_config,
            console=console,
        )
        if prepare_result.return_code != 0:
            raise typer.Exit(prepare_result.return_code)
        try:
            rc = run_real_task_doctor(
                task_name=normalized,
                config_path=orchestration_config,
                task_config_path=task_config,
                console=console,
            )
            if rc != 0:
                raise typer.Exit(rc)
            if dry_run:
                _print_run_dry_run(
                    task_name=normalized,
                    backend_mode=backend_mode,
                    duration_sec=duration_sec,
                    orchestration_config=orchestration_config,
                    task_config=task_config,
                    simulation_profile=simulation_profile,
                    live=live,
                    live_profiles=live_profiles,
                )
                console.print("[yellow]Real dry run stopped after preflight, prepare, and task doctor.[/yellow]")
                raise typer.Exit(0)
            console.print(
                "[red]Real task run is blocked after task doctor:[/red] "
                "companion startup / arm / takeoff flight wrapper is not implemented yet."
            )
            raise typer.Exit(20)
        finally:
            stop_real_prepare_phase(prepare_result)

    if dry_run:
        _print_run_dry_run(
            task_name=normalized,
            backend_mode=backend_mode,
            duration_sec=duration_sec,
            orchestration_config=orchestration_config,
            task_config=task_config,
            simulation_profile=simulation_profile,
            live=live,
            live_profiles=live_profiles,
        )
        raise typer.Exit(0)
    if backend_mode != ("docker", "simulation"):
        console.print("[red]Unsupported runtime run combination:[/red] " f"{backend_mode[0]}+{backend_mode[1]}")
        raise typer.Exit(20)

    if normalized == "scan-robustness":
        profiles = (
            None if live_profiles is None else tuple(item.strip() for item in live_profiles.split(",") if item.strip())
        )
        if simulation_profile is not None:
            console.print("[red]--simulation-profile is only valid for hover and exploration[/red]")
            raise typer.Exit(2)
        task = TaskRegistry.create(normalized)
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

    if live is not None or live_profiles is not None:
        console.print("[red]--live and --live-profiles are only valid for scan-robustness[/red]")
        raise typer.Exit(2)

    task = TaskRegistry.create(normalized)
    raise typer.Exit(
        task.run(
            config_path=orchestration_config,
            task_config_path=task_config,
            duration_sec=duration_sec,
            simulation_profile=simulation_profile,
            console=console,
        )
    )


def _print_run_dry_run(
    *,
    task_name: str,
    backend_mode: tuple[str, str],
    duration_sec: float | None,
    orchestration_config: str | None,
    task_config: str | None,
    simulation_profile: str | None,
    live: bool | None,
    live_profiles: str | None,
) -> None:
    console.print("[yellow]Dry run:[/yellow] wrapper will not execute the task champion.")
    console.print(f"task={task_name}")
    console.print(f"runtime={backend_mode[0]}+{backend_mode[1]}")
    if duration_sec is not None:
        console.print(f"duration_sec={duration_sec}")
    if orchestration_config is not None:
        console.print(f"orchestration_config={orchestration_config}")
    if task_config is not None:
        console.print(f"task_config={task_config}")
    if simulation_profile is not None:
        console.print(f"simulation_profile={simulation_profile}")
    if live is not None:
        console.print(f"live={live}")
    if live_profiles is not None:
        console.print(f"live_profiles={live_profiles}")


def _runtime_backend_mode_from_env() -> tuple[str, str]:
    from src.project_config import DEFAULT_RUNTIME_BACKEND, DEFAULT_RUNTIME_MODE

    backend = os.environ.get("NAVLAB_RUNTIME_BACKEND", DEFAULT_RUNTIME_BACKEND).strip().lower()
    mode = os.environ.get("NAVLAB_RUNTIME_MODE", DEFAULT_RUNTIME_MODE).strip().lower()
    return backend, mode
