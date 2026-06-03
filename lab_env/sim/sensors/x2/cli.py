from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime
from pathlib import Path

from lab_env.logging_utils import configure_sim_logging, logger
from lab_env.sim.sensors.x2.config import X2SensorRuntimeConfig
from lab_env.sim.sensors.x2.emulator import (
    X2SerialEmulator,
    X2SerialEmulatorConfig,
    build_static_scan_samples,
    samples_per_scan,
)
from lab_env.sim.sensors.x2.scan_source import IdealLaserScan, resample_ideal_scan_to_x2_samples


def _build_arg_parser() -> argparse.ArgumentParser:
    defaults = X2SensorRuntimeConfig.load()
    parser = argparse.ArgumentParser(description="Run the YDLidar X2 virtual serial emulator.")
    parser.add_argument("--virtual-serial-link", type=Path, default=defaults.virtual_serial_link)
    parser.add_argument("--status-topic", default=defaults.status_topic)
    parser.add_argument("--scan-ideal-topic", default=defaults.scan_ideal_topic)
    parser.add_argument("--profile-path", type=Path, default=defaults.profile)
    parser.add_argument("--scan-frequency-hz", type=float, default=defaults.scan_frequency_hz)
    parser.add_argument("--sample-rate-hz", type=float, default=defaults.sample_rate_hz)
    parser.add_argument("--range-min-m", type=float, default=defaults.range_min_m)
    parser.add_argument("--range-max-m", type=float, default=defaults.range_max_m)
    parser.add_argument("--static-range-m", type=float, default=1.5)
    parser.add_argument("--sample-count", type=int, default=0)
    parser.add_argument("--range-noise-stddev-m", type=float, default=0.0)
    parser.add_argument("--dropout-rate", type=float, default=0.0)
    parser.add_argument("--random-seed", type=int)
    parser.add_argument("--auto-start", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--driver-smoke", action="store_true", help="Run emulator + vendor driver smoke acceptance.")
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--startup-timeout-sec", type=float, default=20.0)
    parser.add_argument("--log-file", type=Path)
    return parser


def run(argv: list[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    configure_sim_logging(log_file=args.log_file)
    if args.driver_smoke:
        from lab_env.sim.sensors.x2.smoke import X2DriverSmokeConfig, execute_driver_smoke

        artifact_dir = args.artifact_dir or Path(
            os.environ.get(
                "ARTIFACT_DIR",
                f"artifacts/ros/x2_driver_smoke/{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            )
        )
        return execute_driver_smoke(
            X2DriverSmokeConfig(
                artifact_dir=artifact_dir,
                duration_sec=max(1.0, args.duration_sec or 15.0),
                profile_path=args.profile_path,
                virtual_serial_link=args.virtual_serial_link,
                status_topic=args.status_topic,
                scan_topic=X2SensorRuntimeConfig.load().scan_topic,
                scan_frequency_hz=args.scan_frequency_hz,
                sample_rate_hz=args.sample_rate_hz,
                range_min_m=args.range_min_m,
                range_max_m=args.range_max_m,
                static_range_m=args.static_range_m,
                startup_timeout_sec=args.startup_timeout_sec,
            )
        )

    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "x2 sensor CLI requires ROS2 Python packages. Run it inside the Gazebo/sensor runtime image."
        ) from exc

    sample_count = args.sample_count or samples_per_scan(
        sample_rate_hz=args.sample_rate_hz,
        scan_frequency_hz=args.scan_frequency_hz,
    )
    fallback_samples = build_static_scan_samples(sample_count=sample_count, range_m=args.static_range_m)
    emulator = X2SerialEmulator(
        X2SerialEmulatorConfig(
            virtual_serial_link=args.virtual_serial_link,
            scan_frequency_hz=args.scan_frequency_hz,
            sample_rate_hz=args.sample_rate_hz,
            range_min_m=args.range_min_m,
            range_max_m=args.range_max_m,
            status_topic=args.status_topic,
        )
    )

    class X2SerialEmulatorNode(Node):
        def __init__(self) -> None:
            super().__init__("x2_serial_emulator")
            self._publisher = self.create_publisher(String, args.status_topic, 10)
            self._latest_samples = fallback_samples
            self._latest_scan_ideal_monotonic_sec: float | None = None
            self._started_at = time.monotonic()
            emulator.open()
            if args.auto_start:
                emulator.start_scanning()
            self.create_subscription(LaserScan, args.scan_ideal_topic, self._handle_scan_ideal, 10)
            self.create_timer(0.02, self._poll_commands)
            self.create_timer(max(0.001, 1.0 / args.scan_frequency_hz), self._write_scan)
            self.create_timer(0.5, self._publish_status)
            if args.duration_sec > 0:
                self.create_timer(0.1, self._stop_when_done)
            logger.info(
                "x2 emulator opened link={} slave={} status_topic={}",
                args.virtual_serial_link,
                emulator.slave_path,
                args.status_topic,
            )

        def _handle_scan_ideal(self, message: LaserScan) -> None:
            samples = resample_ideal_scan_to_x2_samples(
                IdealLaserScan(
                    ranges=tuple(float(value) for value in message.ranges),
                    angle_min_rad=float(message.angle_min),
                    angle_increment_rad=float(message.angle_increment),
                ),
                sample_count=sample_count,
                range_min_m=args.range_min_m,
                range_max_m=args.range_max_m,
                noise_stddev_m=args.range_noise_stddev_m,
                dropout_rate=args.dropout_rate,
                random_seed=args.random_seed,
            )
            if samples:
                self._latest_samples = samples
                self._latest_scan_ideal_monotonic_sec = time.monotonic()

        def destroy_node(self) -> bool:
            emulator.close()
            return super().destroy_node()

        def _poll_commands(self) -> None:
            data = emulator.poll_commands()
            if data:
                logger.debug("x2 emulator read {} command bytes state={}", len(data), emulator.status().state)

        def _write_scan(self) -> None:
            byte_count = emulator.write_scan_once(self._latest_samples)
            if byte_count:
                logger.debug("x2 emulator wrote {} bytes", byte_count)

        def _publish_status(self) -> None:
            message = String()
            status = emulator.status_dict()
            status["scan_source"] = "gazebo_ideal" if self._latest_scan_ideal_monotonic_sec else "static_fallback"
            status["scan_ideal_topic"] = args.scan_ideal_topic
            status["latest_scan_ideal_age_sec"] = (
                None
                if self._latest_scan_ideal_monotonic_sec is None
                else time.monotonic() - self._latest_scan_ideal_monotonic_sec
            )
            message.data = json.dumps(status, ensure_ascii=True, sort_keys=True)
            self._publisher.publish(message)

        def _stop_when_done(self) -> None:
            if time.monotonic() - self._started_at >= args.duration_sec:
                raise SystemExit(0)

    rclpy.init(args=None)
    node = X2SerialEmulatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
