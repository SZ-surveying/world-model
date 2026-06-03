from __future__ import annotations

import os
from pathlib import Path
from typing import Annotated

import typer

from lab_env.config import DEFAULT_NAVLAB_COMPANION_IMAGE, load_navlab_images_config, load_runtime_config
from lab_env.navlab.runtime.acceptance import execute_companion_gazebo_acceptance
from lab_env.navlab.runtime.companion import launch_companion
from lab_env.navlab.runtime.config import RuntimeConfig
from lab_env.navlab.runtime.doctor import write_doctor_summary
from lab_env.navlab.runtime.logger import init_logger, logger

app = typer.Typer(add_completion=False, no_args_is_help=True)


def _default_config_path() -> Path:
    return Path(os.environ.get("NAVLAB_CONFIG", "/workspace/profiles/navlab-gazebo.toml"))


def _default_companion_image() -> str:
    try:
        runtime_config = load_runtime_config()
        return load_navlab_images_config(runtime_config).companion.image(cwd=runtime_config.lab_root)
    except (OSError, ValueError):
        return DEFAULT_NAVLAB_COMPANION_IMAGE


def _env_log_path(default: Path | None = None) -> Path | None:
    raw = os.environ.get("NAVLAB_RUNTIME_LOG")
    if raw:
        return Path(raw)
    return default


@app.command("launch-companion")
def launch_companion_command(
    config: Annotated[
        Path,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = _default_config_path(),
    log_file: Annotated[
        Path | None,
        typer.Option("--log-file", help="Runtime log file path"),
    ] = None,
) -> None:
    runtime_log = log_file or _env_log_path(Path("/tmp/navlab_companion.log"))
    init_logger(runtime_log)
    logger.info("Runtime log file: {}", runtime_log)
    raise typer.Exit(launch_companion(config_path=config))


@app.command("doctor")
def doctor_command(
    summary_file: Annotated[
        Path,
        typer.Option("--summary-file", help="Where to write the doctor summary JSON"),
    ],
    image: Annotated[
        str,
        typer.Option("--image", help="Companion image label recorded in the summary"),
    ] = _default_companion_image(),
    log_file: Annotated[
        Path | None,
        typer.Option("--log-file", help="Runtime log file path"),
    ] = None,
) -> None:
    runtime_log = log_file or _env_log_path(summary_file.parent / "doctor.log")
    init_logger(runtime_log)
    logger.info("Runtime log file: {}", runtime_log)
    raise typer.Exit(write_doctor_summary(summary_file=summary_file, image=image))


@app.command("execute-acceptance")
def execute_acceptance_command(
    artifact_dir: Annotated[
        Path,
        typer.Option("--artifact-dir", help="Artifact directory shared with the host"),
    ],
    duration_sec: Annotated[
        float,
        typer.Option("--duration-sec", help="Mission duration in seconds"),
    ],
    rosbag_profile: Annotated[
        Path,
        typer.Option("--rosbag-profile", help="Rosbag topic profile path"),
    ],
    companion_image: Annotated[
        str,
        typer.Option("--companion-image", help="Companion image label recorded in summaries"),
    ],
    scan_source: Annotated[
        str,
        typer.Option("--scan-source", help="Accepted scan source label recorded in summaries"),
    ] = "x2_virtual_serial_vendor_driver",
    config: Annotated[
        Path,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = _default_config_path(),
    log_file: Annotated[
        Path | None,
        typer.Option("--log-file", help="Runtime log file path"),
    ] = None,
) -> None:
    runtime_log = log_file or _env_log_path(artifact_dir / "runtime.log")
    init_logger(runtime_log)
    logger.info("Runtime log file: {}", runtime_log)
    raise typer.Exit(
        execute_companion_gazebo_acceptance(
            artifact_dir=artifact_dir,
            duration_sec=duration_sec,
            rosbag_profile_path=rosbag_profile,
            companion_image=companion_image,
            scan_source=scan_source,
            config=RuntimeConfig.load(config),
        )
    )


if __name__ == "__main__":
    app()
