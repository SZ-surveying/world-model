from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from lab_env.logging_utils import configure_sim_logging, logger
from lab_env.navlab.runtime.process_manager import ProcessManager
from lab_env.sim.sensors.x2.config import X2SensorRuntimeConfig


@dataclass(frozen=True, slots=True)
class X2SensorLaunchConfig:
    scan_source: str
    profile_path: Path
    virtual_serial_link: Path
    scan_ideal_topic: str
    scan_topic: str
    status_topic: str
    scan_frequency_hz: float
    sample_rate_hz: float
    range_min_m: float
    range_max_m: float
    static_range_m: float
    auto_start: bool
    range_noise_stddev_m: float
    dropout_rate: float
    random_seed: int | None

    @classmethod
    def from_runtime(cls, *, scan_source: str = "x2_virtual_serial") -> X2SensorLaunchConfig:
        runtime = X2SensorRuntimeConfig.load()
        return cls(
            scan_source=scan_source,
            profile_path=runtime.profile,
            virtual_serial_link=runtime.virtual_serial_link,
            scan_ideal_topic=runtime.scan_ideal_topic,
            scan_topic=runtime.scan_topic,
            status_topic=runtime.status_topic,
            scan_frequency_hz=runtime.scan_frequency_hz,
            sample_rate_hz=runtime.sample_rate_hz,
            range_min_m=runtime.range_min_m,
            range_max_m=runtime.range_max_m,
            static_range_m=1.5,
            auto_start=True,
            range_noise_stddev_m=0.0,
            dropout_rate=0.0,
            random_seed=None,
        )


def build_scan_ideal_bridge_command(config: X2SensorLaunchConfig) -> list[str]:
    return [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        f"{config.scan_ideal_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "--ros-args",
        "-p",
        "override_frame_id:=laser_frame",
    ]


def build_emulator_command(config: X2SensorLaunchConfig) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "lab_env.sim.sensors.x2.cli",
        "--virtual-serial-link",
        str(config.virtual_serial_link),
        "--status-topic",
        config.status_topic,
        "--scan-ideal-topic",
        config.scan_ideal_topic,
        "--profile-path",
        str(config.profile_path),
        "--scan-frequency-hz",
        str(config.scan_frequency_hz),
        "--sample-rate-hz",
        str(config.sample_rate_hz),
        "--range-min-m",
        str(config.range_min_m),
        "--range-max-m",
        str(config.range_max_m),
        "--static-range-m",
        str(config.static_range_m),
        "--range-noise-stddev-m",
        str(config.range_noise_stddev_m),
        "--dropout-rate",
        str(config.dropout_rate),
    ]
    if config.random_seed is not None:
        command.extend(["--random-seed", str(config.random_seed)])
    command.append("--auto-start" if config.auto_start else "--no-auto-start")
    return command


def build_vendor_driver_command(config: X2SensorLaunchConfig) -> list[str]:
    return [
        "ros2",
        "run",
        "ydlidar_ros2_driver",
        "ydlidar_ros2_driver_node",
        "--ros-args",
        "--params-file",
        str(config.profile_path),
    ]


class X2SensorRuntime:
    def __init__(self, config: X2SensorLaunchConfig) -> None:
        self._config = config
        self._manager = ProcessManager()
        self._stopping = False

    def start(self) -> int:
        logger.info("Starting X2 sensor runtime scan_source={}", self._config.scan_source)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)
        self._start_processes()
        try:
            while not self._stopping:
                for process in self._manager.processes:
                    rc = process.poll()
                    if rc is not None:
                        logger.warning("{} exited early rc={}; stopping sensor runtime", process.name, rc)
                        return rc or 1
                time.sleep(1.0)
        finally:
            self._manager.stop_all(timeout_sec=8.0)
        return 0

    def _start_processes(self) -> None:
        self._manager.start_subprocess("scan_ideal_bridge", build_scan_ideal_bridge_command(self._config))
        if self._config.scan_source != "x2_virtual_serial":
            logger.info("scan_source={} starts only the ideal scan bridge", self._config.scan_source)
            return
        self._manager.start_subprocess("x2_serial_emulator", build_emulator_command(self._config))
        time.sleep(0.5)
        self._manager.start_subprocess("ydlidar_ros2_driver", build_vendor_driver_command(self._config))

    def _handle_signal(self, signum: int, _frame: object) -> None:
        logger.info("Received signal {}; stopping X2 sensor runtime", signum)
        self._stopping = True


def _build_arg_parser() -> argparse.ArgumentParser:
    defaults = X2SensorRuntimeConfig.load()
    parser = argparse.ArgumentParser(description="Launch the NavLab Gazebo X2 sensor runtime.")
    parser.add_argument("--scan-source", choices=("gazebo_ideal", "x2_virtual_serial"), default="x2_virtual_serial")
    parser.add_argument("--virtual-serial-link", type=Path, default=defaults.virtual_serial_link)
    parser.add_argument("--status-topic", default=defaults.status_topic)
    parser.add_argument("--scan-ideal-topic", default=defaults.scan_ideal_topic)
    parser.add_argument("--scan-topic", default=defaults.scan_topic)
    parser.add_argument("--profile-path", type=Path, default=defaults.profile)
    parser.add_argument("--scan-frequency-hz", type=float, default=defaults.scan_frequency_hz)
    parser.add_argument("--sample-rate-hz", type=float, default=defaults.sample_rate_hz)
    parser.add_argument("--range-min-m", type=float, default=defaults.range_min_m)
    parser.add_argument("--range-max-m", type=float, default=defaults.range_max_m)
    parser.add_argument("--static-range-m", type=float, default=1.5)
    parser.add_argument("--range-noise-stddev-m", type=float, default=0.0)
    parser.add_argument("--dropout-rate", type=float, default=0.0)
    parser.add_argument("--random-seed", type=int)
    parser.add_argument("--auto-start", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--log-file", type=Path)
    return parser


def launch(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    configure_sim_logging(log_file=args.log_file)
    config = X2SensorLaunchConfig(
        scan_source=args.scan_source,
        profile_path=args.profile_path,
        virtual_serial_link=args.virtual_serial_link,
        scan_ideal_topic=args.scan_ideal_topic,
        scan_topic=args.scan_topic,
        status_topic=args.status_topic,
        scan_frequency_hz=args.scan_frequency_hz,
        sample_rate_hz=args.sample_rate_hz,
        range_min_m=args.range_min_m,
        range_max_m=args.range_max_m,
        static_range_m=args.static_range_m,
        auto_start=args.auto_start,
        range_noise_stddev_m=args.range_noise_stddev_m,
        dropout_rate=args.dropout_rate,
        random_seed=args.random_seed,
    )
    return X2SensorRuntime(config).start()


def main() -> int:
    return launch()


if __name__ == "__main__":
    raise SystemExit(main())
