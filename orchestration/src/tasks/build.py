from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar, Literal

from python_on_whales import DockerClient
from python_on_whales.exceptions import DockerException
from rich.console import Console
from rich.table import Table

from src.project_config import (
    NavLabImageConfig,
    NavLabImagesConfig,
    load_navlab_images_config,
    load_runtime_config,
)
from src.tasks.base import OrchestrationTask
from src.tasks.registry import TaskRegistry

ImageKind = Literal["companion", "slam", "gazebo-sensor", "all"]


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
    )
    if kind == "all":
        return specs
    for spec in specs:
        if spec[0] == kind:
            return (spec,)
    raise ValueError(f"Invalid NavLab image kind '{kind}': expected companion, slam, gazebo-sensor, or all")


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


@TaskRegistry.register
@dataclass(frozen=True, slots=True)
class BuildTask(OrchestrationTask):
    TASK_NAME: ClassVar[str] = "build"
    TASK_DESCRIPTION: ClassVar[str] = "Build one or more NavLab service images."

    def run(self, *, kind: ImageKind = "all", tag: str | None = None, console: Console | None = None) -> int:
        console = console or Console()
        runtime = load_runtime_config()
        image_config = load_navlab_images_config(runtime)
        try:
            specs = _image_build_specs(image_config, kind)
            resolved_specs = tuple(
                (
                    label,
                    _repo_path(runtime.lab_root, spec.context.value),
                    _repo_path(runtime.lab_root, spec.dockerfile.value),
                    spec.target.value,
                    spec.image(cli_tag=tag, cwd=runtime.lab_root),
                )
                for label, spec in specs
            )
        except ValueError as exc:
            console.print(f"[red]{exc}[/red]")
            return 2
        _render_image_build_config(
            console,
            kind=kind,
            runtime_root=runtime.lab_root,
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
