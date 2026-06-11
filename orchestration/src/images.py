from __future__ import annotations

from pathlib import Path
from typing import Literal

from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console
from rich.table import Table

from src.configs.project_config import (
    NavLabImageConfig,
    NavLabImagesConfig,
    load_project_config,
)

ImageKind = Literal["companion", "slam", "gazebo-sensor", "official-baseline", "all"]


def _repo_path(runtime_root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return runtime_root / path


def _image_build_specs(
    image_config: NavLabImagesConfig,
    kind: ImageKind,
) -> tuple[tuple[str, NavLabImageConfig], ...]:
    specs = (
        ("companion", image_config.companion),
        ("slam", image_config.slam),
        ("gazebo-sensor", image_config.gazebo_sensor),
        ("official-baseline", image_config.official_baseline),
    )
    if kind == "all":
        return specs
    for spec in specs:
        if spec[0] == kind:
            return (spec,)
    raise ValueError(
        f"Invalid NavLab image kind '{kind}': expected companion, slam, gazebo-sensor, official-baseline, or all"
    )


def _render_image_build_config(
    console: Console,
    *,
    kind: ImageKind,
    runtime_root: Path,
    specs: tuple[tuple[str, NavLabImageConfig], ...],
    tag: str | None,
) -> None:
    table = Table(title="NavLab Image Build", show_header=False)
    table.add_column("Key", style="bold")
    table.add_column("Value")
    table.add_row("kind", kind)
    table.add_row("tag override", tag or "<none>")
    for label, image_config in specs:
        table.add_row(f"{label} context", str(_repo_path(runtime_root, image_config.context.value)))
        table.add_row(f"{label} dockerfile", str(_repo_path(runtime_root, image_config.dockerfile.value)))
        table.add_row(f"{label} target", image_config.target.value)
        table.add_row(f"{label} tag strategy", image_config.tag_strategy.value)
        table.add_row(f"{label} image", image_config.image(cli_tag=tag, cwd=runtime_root))
    console.print(table)


def run_image_build(*, kind: ImageKind = "all", tag: str | None = None, console: Console | None = None) -> int:
    console = console or Console()
    project_config = load_project_config()
    project_paths = project_config.paths
    image_config = project_config.images
    try:
        specs = _image_build_specs(image_config, kind)
        resolved_specs = tuple(
            (
                label,
                _repo_path(project_paths.lab_root, spec.context.value),
                _repo_path(project_paths.lab_root, spec.dockerfile.value),
                spec.target.value,
                spec.image(cli_tag=tag, cwd=project_paths.lab_root),
            )
            for label, spec in specs
        )
    except ValueError as exc:
        console.print(f"[red]{exc}[/red]")
        return 2
    _render_image_build_config(
        console,
        kind=kind,
        runtime_root=project_paths.lab_root,
        specs=specs,
        tag=tag,
    )
    try:
        for label, context_path, dockerfile, target, image in resolved_specs:
            console.print(f"[bold cyan]Building NavLab {label} image[/bold cyan] {image}")
            logs = DockerClient().build(
                context_path,
                file=dockerfile,
                target=target,
                tags=image,
                stream_logs=True,
            )
            if logs is not None:
                for line in logs:
                    console.print(str(line).rstrip())
    except DockerException as exc:
        console.print(f"[red]Docker build failed:[/red] {exc}")
        return exc.return_code or 1
    console.print("[green]NavLab image build completed[/green]")
    return 0
