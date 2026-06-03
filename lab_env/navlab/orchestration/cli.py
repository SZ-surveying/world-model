from __future__ import annotations

from typing import Annotated, cast

import typer
from rich.console import Console

from lab_env.navlab.orchestration.host import (
    ImageKind,
    build_navlab_images,
    companion_doctor,
    orchestrate_companion_gazebo_acceptance,
)

app = typer.Typer(add_completion=False, no_args_is_help=True)
console = Console()


@app.command("build")
def image_build_command(
    kind: Annotated[
        str,
        typer.Argument(help="Image to build: companion, slam, gazebo-sensor, or all"),
    ] = "all",
    tag: Annotated[
        str | None,
        typer.Option("--tag", help="Override the configured NavLab image tag strategy"),
    ] = None,
) -> None:
    raise typer.Exit(build_navlab_images(kind=cast(ImageKind, kind), tag=tag, console=console))


@app.command("doctor")
def companion_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    raise typer.Exit(companion_doctor(config_path=config, console=console))


@app.command("acceptance")
def acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="Mission duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    raise typer.Exit(
        orchestrate_companion_gazebo_acceptance(
            config_path=config,
            duration_sec=duration_sec,
            console=console,
        )
    )
