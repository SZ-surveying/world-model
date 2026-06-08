from __future__ import annotations

from typing import Annotated, cast

import typer
from rich.console import Console

from src.tasks.acceptance import AcceptanceTask
from src.tasks.airframe_disturbance_gate import AirframeDisturbanceGateAcceptanceTask, AirframeDisturbanceGateDoctorTask
from src.tasks.build import BuildTask, ImageKind
from src.tasks.doctor import DoctorTask
from src.tasks.exploration_gate import ExplorationGateAcceptanceTask, ExplorationGateDoctorTask
from src.tasks.fcu_controller import FcuControllerAcceptanceTask, FcuControllerDoctorTask
from src.tasks.frame_contract import FrameContractAcceptanceTask, FrameContractDoctorTask
from src.tasks.hover import HoverAcceptanceTask
from src.tasks.hover_diagnostic import HoverDiagnosticTask
from src.tasks.hover_slam_diagnostic import HoverSlamDiagnosticTask
from src.tasks.motion_gate import MotionGateAcceptanceTask, MotionGateDoctorTask
from src.tasks.official_baseline import OfficialBaselineAcceptanceTask, OfficialBaselineDoctorTask
from src.tasks.official_maze_x2 import OfficialMazeX2AcceptanceTask
from src.tasks.rangefinder_imu import RangefinderImuAcceptanceTask, RangefinderImuDoctorTask
from src.tasks.registry import TaskRegistry
from src.tasks.scan_integrity_gate import ScanIntegrityGateAcceptanceTask, ScanIntegrityGateDoctorTask
from src.tasks.scan_stabilization_gate import ScanStabilizationGateAcceptanceTask, ScanStabilizationGateDoctorTask
from src.tasks.slam_hover import SlamHoverAcceptanceTask, SlamHoverDoctorTask
from src.tasks.slam_backend import SlamBackendAcceptanceTask, SlamBackendDoctorTask

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


@app.command("official-maze-x2-acceptance")
def official_maze_x2_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="Official maze + NavLab X2 acceptance duration in seconds")]
    = 45.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(OfficialMazeX2AcceptanceTask, TaskRegistry.create("official-maze-x2-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("rangefinder-imu-doctor")
def rangefinder_imu_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(RangefinderImuDoctorTask, TaskRegistry.create("rangefinder-imu-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("rangefinder-imu-acceptance")
def rangefinder_imu_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P2 rangefinder/IMU acceptance duration in seconds")] = 60.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(RangefinderImuAcceptanceTask, TaskRegistry.create("rangefinder-imu-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("slam-backend-doctor")
def slam_backend_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(SlamBackendDoctorTask, TaskRegistry.create("slam-backend-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("slam-backend-acceptance")
def slam_backend_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P3 SLAM backend quality acceptance duration in seconds")]
    = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(SlamBackendAcceptanceTask, TaskRegistry.create("slam-backend-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("fcu-controller-doctor")
def fcu_controller_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(FcuControllerDoctorTask, TaskRegistry.create("fcu-controller-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("fcu-controller-acceptance")
def fcu_controller_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P4 FCU controller acceptance duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(FcuControllerAcceptanceTask, TaskRegistry.create("fcu-controller-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("frame-contract-doctor")
def frame_contract_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(FrameContractDoctorTask, TaskRegistry.create("frame-contract-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("frame-contract-acceptance")
def frame_contract_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P5 frame contract acceptance duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(FrameContractAcceptanceTask, TaskRegistry.create("frame-contract-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("slam-hover-doctor")
def slam_hover_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(SlamHoverDoctorTask, TaskRegistry.create("slam-hover-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("slam-hover-acceptance")
def slam_hover_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P6 real SLAM hover acceptance duration in seconds")] = 90.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(SlamHoverAcceptanceTask, TaskRegistry.create("slam-hover-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("motion-gate-doctor")
def motion_gate_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(MotionGateDoctorTask, TaskRegistry.create("motion-gate-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("motion-gate-acceptance")
def motion_gate_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P7 official maze motion acceptance duration in seconds")]
    = 120.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(MotionGateAcceptanceTask, TaskRegistry.create("motion-gate-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("exploration-gate-doctor")
def exploration_gate_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(ExplorationGateDoctorTask, TaskRegistry.create("exploration-gate-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("exploration-gate-acceptance")
def exploration_gate_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P8 official maze exploration acceptance duration in seconds")]
    = 150.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(ExplorationGateAcceptanceTask, TaskRegistry.create("exploration-gate-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("exploration-replay-acceptance")
def exploration_replay_acceptance_command(
    duration_sec: Annotated[float, typer.Argument(help="P8 representative exploration replay duration in seconds")]
    = 180.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
    profile: Annotated[
        str,
        typer.Option("--profile", help="Replay profile: conservative or display"),
    ] = "conservative",
) -> None:
    task = cast(ExplorationGateAcceptanceTask, TaskRegistry.create("exploration-gate-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console, replay_profile=profile))


@app.command("scan-integrity-gate-doctor")
def scan_integrity_gate_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(ScanIntegrityGateDoctorTask, TaskRegistry.create("scan-integrity-gate-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("scan-integrity-gate-acceptance")
def scan_integrity_gate_acceptance_command(
    duration_sec: Annotated[
        float,
        typer.Argument(help="P10 body-fixed lidar scan integrity acceptance duration in seconds"),
    ] = 140.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(ScanIntegrityGateAcceptanceTask, TaskRegistry.create("scan-integrity-gate-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("scan-stabilization-gate-doctor")
def scan_stabilization_gate_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(ScanStabilizationGateDoctorTask, TaskRegistry.create("scan-stabilization-gate-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("scan-stabilization-gate-acceptance")
def scan_stabilization_gate_acceptance_command(
    duration_sec: Annotated[
        float,
        typer.Argument(help="P11 bounded 2D lidar scan stabilization acceptance duration in seconds"),
    ] = 240.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(ScanStabilizationGateAcceptanceTask, TaskRegistry.create("scan-stabilization-gate-acceptance"))
    raise typer.Exit(task.run(config_path=config, duration_sec=duration_sec, console=console))


@app.command("airframe-disturbance-gate-doctor")
def airframe_disturbance_gate_doctor_command(
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
) -> None:
    task = cast(AirframeDisturbanceGateDoctorTask, TaskRegistry.create("airframe-disturbance-gate-doctor"))
    raise typer.Exit(task.run(config_path=config, console=console))


@app.command("airframe-disturbance-gate-acceptance")
def airframe_disturbance_gate_acceptance_command(
    duration_sec: Annotated[
        float,
        typer.Argument(help="P12 airframe disturbance profile sweep duration budget in seconds"),
    ] = 240.0,
    config: Annotated[
        str | None,
        typer.Option("--config", help="NavLab TOML profile path"),
    ] = None,
    live: Annotated[
        bool,
        typer.Option("--live", help="Run the disturbed P9 replay through the P11 stabilization gate"),
    ] = False,
    live_profiles: Annotated[
        str,
        typer.Option("--live-profiles", help="Comma-separated P12 profiles to run through full live P9 replay"),
    ] = "",
) -> None:
    task = cast(AirframeDisturbanceGateAcceptanceTask, TaskRegistry.create("airframe-disturbance-gate-acceptance"))
    profiles = tuple(item.strip() for item in live_profiles.split(",") if item.strip())
    raise typer.Exit(
        task.run(
            config_path=config,
            duration_sec=duration_sec,
            live_replay=live,
            live_profiles=profiles,
            console=console,
        )
    )


if __name__ == "__main__":
    app()
