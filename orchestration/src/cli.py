from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Annotated, cast

import typer
from rich.console import Console

from src.config import load_motor_debug_task_config
from src.tasks.build import BuildTask, ImageKind
from src.tasks.built_in.motor_debug import build_motor_debug_plan
from src.tasks.doctor import DoctorTask
from src.tasks.real_prepare import (
    RealPreparePhaseResult,
    execute_real_prepare_phase,
    run_real_common_doctor,
    run_real_task_doctor,
    stop_real_prepare_phase,
)
from src.tasks.registry import TaskRegistry

app = typer.Typer(add_completion=False, no_args_is_help=True)
console = Console()

RUN_TASKS = ("hover", "exploration", "scan-robustness", "motor-debug")


@dataclass(frozen=True, slots=True)
class RealPrepareCommonDoctorChainResult:
    return_code: int
    prepare_result: RealPreparePhaseResult | None = None
    interrupted: bool = False


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
    keep_prepare_running: Annotated[
        bool,
        typer.Option(
            "--keep-prepare-running",
            help="Keep real prepare services running after doctor for Mission Planner debugging.",
        ),
    ] = False,
) -> None:
    backend_mode = _runtime_backend_mode_from_env()
    if backend_mode == ("process", "real"):
        raise typer.Exit(
            _run_real_doctor_chain(
                orchestration_config=orchestration_config,
                task_config=task_config,
                force=force,
                keep_prepare_running=keep_prepare_running,
            )
        )

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
        typer.Argument(help="Built-in task to run: hover, exploration, scan-robustness, or motor-debug"),
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
    skip_doctor: Annotated[
        bool,
        typer.Option(
            "--skip-doctor",
            help="Skip real preflight, prepare, and common doctor; run only task doctor and task execution.",
        ),
    ] = False,
    confirm_manual_takeover: Annotated[
        bool,
        typer.Option(
            "--confirm-manual-takeover",
            help="Confirm a trained operator can immediately take manual control before real flight.",
        ),
    ] = False,
    confirm_kill_switch: Annotated[
        bool,
        typer.Option(
            "--confirm-kill-switch",
            help="Confirm the physical/software kill switch is ready before real flight.",
        ),
    ] = False,
    confirm_safe_area: Annotated[
        bool,
        typer.Option(
            "--confirm-safe-area",
            help="Confirm the real flight area, standoff, battery, and props are safe.",
        ),
    ] = False,
    confirm_no_props: Annotated[
        bool,
        typer.Option("--confirm-no-props", help="Confirm propellers are removed before running motor-debug."),
    ] = False,
    motor_percent: Annotated[
        float | None,
        typer.Option("--motor-percent", help="motor-debug throttle percent, capped by the debug task."),
    ] = None,
    motor_sec: Annotated[
        float | None,
        typer.Option("--motor-sec", help="motor-debug spin duration per motor in seconds."),
    ] = None,
    motor_count: Annotated[
        int | None,
        typer.Option("--motor-count", help="motor-debug motor count to test sequentially."),
    ] = None,
) -> None:
    normalized = task_name.strip()
    if normalized not in RUN_TASKS:
        supported = ", ".join(RUN_TASKS)
        console.print(f"[red]Unknown built-in task:[/red] {task_name}. Supported tasks: {supported}")
        raise typer.Exit(2)

    backend_mode = _runtime_backend_mode_from_env()

    if backend_mode == ("process", "real"):
        manual_takeover_confirmed = _confirmation_value(
            "NAVLAB_CONFIRM_MANUAL_TAKEOVER",
            cli_value=confirm_manual_takeover,
        )
        kill_switch_confirmed = _confirmation_value(
            "NAVLAB_CONFIRM_KILL_SWITCH",
            cli_value=confirm_kill_switch,
        )
        safe_area_confirmed = _confirmation_value(
            "NAVLAB_CONFIRM_SAFE_AREA",
            cli_value=confirm_safe_area,
        )
        no_props_confirmed = _confirmation_value("NAVLAB_CONFIRM_NO_PROPS", cli_value=confirm_no_props)
        if normalized == "motor-debug":
            motor_debug_config = load_motor_debug_task_config(
                task_config_path=task_config,
                cli_motor_percent=motor_percent,
                cli_motor_sec=motor_sec,
                cli_motor_count=motor_count,
            )
            if dry_run:
                rc = _run_real_preflight_only(orchestration_config=orchestration_config, force=False)
                if rc != 0:
                    raise typer.Exit(rc)
                _print_motor_debug_dry_run(
                    motor_percent=motor_debug_config.motor_percent,
                    motor_sec=motor_debug_config.motor_sec,
                    motor_count=motor_debug_config.motor_count,
                    backend_mode=backend_mode,
                )
                raise typer.Exit(0)
            safety = _operator_safety_confirmation(
                manual_takeover=manual_takeover_confirmed,
                kill_switch=kill_switch_confirmed,
                safe_area=safe_area_confirmed,
            )
            no_props_blockers = [] if no_props_confirmed else ["operator_no_props_not_confirmed"]
            if safety["blocked"] or no_props_blockers:
                console.print("[red]Real motor-debug is blocked before motor test:[/red] safety confirmation missing.")
                for blocker in [*safety["blockers"], *no_props_blockers]:
                    console.print(f"blocker: {blocker}")
                raise typer.Exit(20)
            prepare_result: RealPreparePhaseResult | None = None
            if skip_doctor:
                console.print("[yellow]Skipping real preflight, prepare, and common doctor.[/yellow]")
            else:
                chain = _run_real_prepare_common_doctor_chain(
                    task_name=normalized,
                    orchestration_config=orchestration_config,
                    task_config=task_config,
                    force=False,
                    auto_cleanup=False,
                )
                if chain.return_code != 0 or chain.prepare_result is None:
                    if chain.prepare_result is not None:
                        stop_real_prepare_phase(chain.prepare_result)
                        if chain.interrupted:
                            _warn_real_prepare_stopped_after_interrupt()
                    raise typer.Exit(chain.return_code)
                prepare_result = chain.prepare_result
            interrupted = False
            try:
                rc = run_real_task_doctor(
                    task_name=normalized,
                    config_path=orchestration_config,
                    task_config_path=task_config,
                    console=console,
                )
                if rc != 0:
                    raise typer.Exit(rc)
                raise typer.Exit(
                    TaskRegistry.create("motor-debug").run(
                        config_path=orchestration_config,
                        task_config_path=task_config,
                        motor_percent=motor_percent,
                        motor_sec=motor_sec,
                        motor_count=motor_count,
                        console=console,
                    )
                )
            except KeyboardInterrupt:
                interrupted = True
                _warn_real_run_interrupted("motor-debug")
                raise typer.Exit(130) from None
            finally:
                if prepare_result is not None:
                    stop_real_prepare_phase(prepare_result)
                if interrupted and prepare_result is not None:
                    _warn_real_prepare_stopped_after_interrupt()
        prepare_result: RealPreparePhaseResult | None = None
        if skip_doctor:
            console.print("[yellow]Skipping real preflight, prepare, and common doctor.[/yellow]")
        else:
            chain = _run_real_prepare_common_doctor_chain(
                task_name=normalized,
                orchestration_config=orchestration_config,
                task_config=task_config,
                force=False,
                auto_cleanup=False,
            )
            if chain.return_code != 0 or chain.prepare_result is None:
                if chain.prepare_result is not None:
                    stop_real_prepare_phase(chain.prepare_result)
                    if chain.interrupted:
                        _warn_real_prepare_stopped_after_interrupt()
                raise typer.Exit(chain.return_code)
            prepare_result = chain.prepare_result
        interrupted = False
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
                console.print("[yellow]Real dry run stopped after preflight, prepare, common doctor, and task doctor.[/yellow]")
                raise typer.Exit(0)
            safety = _operator_safety_confirmation(
                manual_takeover=manual_takeover_confirmed,
                kill_switch=kill_switch_confirmed,
                safe_area=safe_area_confirmed,
            )
            if safety["blocked"]:
                console.print("[red]Real task run is blocked before arm/takeoff:[/red] operator safety missing.")
                for blocker in safety["blockers"]:
                    console.print(f"blocker: {blocker}")
                raise typer.Exit(20)
            console.print(
                "[red]Real task run is blocked after task doctor:[/red] "
                "companion startup / arm / takeoff flight wrapper is not implemented yet."
            )
            raise typer.Exit(20)
        except KeyboardInterrupt:
            interrupted = True
            _warn_real_run_interrupted(normalized)
            raise typer.Exit(130) from None
        finally:
            if prepare_result is not None:
                stop_real_prepare_phase(prepare_result)
            if interrupted and prepare_result is not None:
                _warn_real_prepare_stopped_after_interrupt()

    if normalized == "motor-debug":
        console.print("[red]motor-debug is only supported with process+real runtime.[/red]")
        raise typer.Exit(20)

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


def _run_real_doctor_chain(
    *,
    orchestration_config: str | None,
    task_config: str | None,
    force: bool,
    keep_prepare_running: bool,
) -> int:
    chain = _run_real_prepare_common_doctor_chain(
        task_name="doctor",
        orchestration_config=orchestration_config,
        task_config=task_config,
        force=force,
        auto_cleanup=not keep_prepare_running,
    )
    if chain.return_code == 0 and keep_prepare_running:
        console.print("[yellow]Real doctor completed; prepare services are still running.[/yellow]")
        console.print("[yellow]Stop them manually when debugging is done.[/yellow]")
    elif chain.return_code == 0:
        console.print("[green]Real doctor completed; prepare services stopped.[/green]")
    return chain.return_code


def _run_real_preflight_only(*, orchestration_config: str | None, force: bool) -> int:
    doctor = cast(DoctorTask, TaskRegistry.create("doctor"))
    return doctor.run(
        config_path=orchestration_config,
        task_config_path=None,
        console=console,
        prompt_install=True,
        force_install=force,
        soft_dependency_warnings=True,
    )


def _run_real_prepare_common_doctor_chain(
    *,
    task_name: str,
    orchestration_config: str | None,
    task_config: str | None,
    force: bool,
    auto_cleanup: bool,
) -> RealPrepareCommonDoctorChainResult:
    rc = _run_real_preflight_only(orchestration_config=orchestration_config, force=force)
    if rc != 0:
        return RealPrepareCommonDoctorChainResult(return_code=rc)
    prepare_result = execute_real_prepare_phase(
        task_name=task_name,
        config_path=orchestration_config,
        console=console,
    )
    if prepare_result.return_code != 0:
        if auto_cleanup:
            stop_real_prepare_phase(prepare_result)
        return RealPrepareCommonDoctorChainResult(return_code=prepare_result.return_code)
    interrupted = False
    try:
        rc = run_real_common_doctor(
            config_path=orchestration_config,
            task_config_path=task_config,
            task_name=task_name,
            console=console,
        )
    except KeyboardInterrupt:
        interrupted = True
        _warn_real_run_interrupted(task_name)
        rc = 130
    finally:
        if auto_cleanup:
            stop_real_prepare_phase(prepare_result)
            if interrupted:
                _warn_real_prepare_stopped_after_interrupt()
    return RealPrepareCommonDoctorChainResult(
        return_code=rc,
        prepare_result=None if auto_cleanup else prepare_result,
        interrupted=interrupted,
    )


def _warn_real_run_interrupted(task_name: str) -> None:
    console.print(
        "[yellow]WARN: real run interrupted by operator; stopping prepare services before exit.[/yellow]"
    )
    console.print(f"[yellow]Interrupted task:[/yellow] {task_name}")


def _warn_real_prepare_stopped_after_interrupt() -> None:
    console.print("[yellow]Real prepare services stopped after operator interrupt.[/yellow]")


def _print_motor_debug_dry_run(
    *,
    motor_percent: float,
    motor_sec: float,
    motor_count: int,
    backend_mode: tuple[str, str],
) -> None:
    plan = build_motor_debug_plan(motor_percent=motor_percent, motor_sec=motor_sec, motor_count=motor_count)
    console.print("[yellow]Dry run:[/yellow] wrapper will not spin motors.")
    console.print("task=motor-debug")
    console.print(f"runtime={backend_mode[0]}+{backend_mode[1]}")
    console.print("spin_mode=armed_idle")
    console.print("arm_command=MAV_CMD_COMPONENT_ARM_DISARM param1=1 param2=0")
    console.print("disarm_command=MAV_CMD_COMPONENT_ARM_DISARM param1=0 param2=0")
    console.print(f"hold_sec={motor_sec}")
    console.print(f"expected_motor_count={motor_count}")
    console.print("requires_no_props=True")
    console.print(f"guided_mode_required={plan['guided_mode_required']}")
    console.print(f"required_mode={plan['required_mode']}")
    console.print("takeoff_claim=not_evaluated")
    console.print("landing_claim=not_evaluated_no_takeoff")
    for blocker in plan["blockers"]:
        console.print(f"[red]blocker:[/red] {blocker}")


def _operator_safety_confirmation(
    *,
    manual_takeover: bool,
    kill_switch: bool,
    safe_area: bool,
) -> dict[str, object]:
    blockers = []
    if not manual_takeover:
        blockers.append("operator_manual_takeover_not_confirmed")
    if not kill_switch:
        blockers.append("operator_kill_switch_not_confirmed")
    if not safe_area:
        blockers.append("operator_safe_area_not_confirmed")
    return {
        "ok": not blockers,
        "blocked": bool(blockers),
        "manual_takeover_confirmed": manual_takeover,
        "kill_switch_confirmed": kill_switch,
        "safe_area_confirmed": safe_area,
        "blockers": blockers,
    }


def _confirmation_value(env_name: str, *, cli_value: bool) -> bool:
    raw = os.environ.get(env_name)
    if raw is None or not raw.strip():
        return cli_value
    return raw.strip().lower() == "true"


def _runtime_backend_mode_from_env() -> tuple[str, str]:
    from src.project_config import DEFAULT_RUNTIME_BACKEND, DEFAULT_RUNTIME_MODE

    backend = os.environ.get("NAVLAB_RUNTIME_BACKEND", DEFAULT_RUNTIME_BACKEND).strip().lower()
    mode = os.environ.get("NAVLAB_RUNTIME_MODE", DEFAULT_RUNTIME_MODE).strip().lower()
    return backend, mode
