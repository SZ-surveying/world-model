from __future__ import annotations

from typing import Annotated, cast

import typer
from rich.console import Console

from src.tasks.acceptance import AcceptanceTask
from src.tasks.build import BuildTask, ImageKind
from src.tasks.doctor import DoctorTask
from src.tasks.hover import HoverAcceptanceTask
from src.tasks.hover_diagnostic import HoverDiagnosticTask
from src.tasks.hover_slam_diagnostic import HoverSlamDiagnosticTask
from src.tasks.official_baseline import OfficialBaselineAcceptanceTask, OfficialBaselineDoctorTask
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
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(DoctorTask, TaskRegistry.create("doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("acceptance")
def acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="Mission duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(AcceptanceTask, TaskRegistry.create("acceptance"))
    raise typer.Exit(
        task.run(config_path=config, duration_sec=duration_sec, console=console)
    )


@app.command("hover")
def hover_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="Hover acceptance duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(HoverAcceptanceTask, TaskRegistry.create("hover"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("hover-diagnostic")
def hover_diagnostic_command(
    duration_sec: Annotated[float, typer.Argument(help="Hover diagnostic duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(HoverDiagnosticTask, TaskRegistry.create("hover-diagnostic"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("hover-slam-diagnostic")
def hover_slam_diagnostic_command(
    duration_sec: Annotated[float, typer.Argument(help="SLAM hover diagnostic duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(HoverSlamDiagnosticTask, TaskRegistry.create("hover-slam-diagnostic"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("official-baseline-doctor")
def official_baseline_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(OfficialBaselineDoctorTask, TaskRegistry.create("official-baseline-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("official-baseline-acceptance")
def official_baseline_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="Official baseline graph check duration in seconds")] = 30.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(OfficialBaselineAcceptanceTask, TaskRegistry.create("official-baseline-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


if __name__ == "__main__":
    app()
