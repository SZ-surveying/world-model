from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from navlab.common.toml_values import (
    BoolWithSource,
    FloatWithSource,
    ValueWithSource,
    load_navlab_config,
    resolve_bool_value,
    resolve_float_value,
    resolve_str_value,
    section,
)
from navlab.gazebo_sensor.x2.emulator import X2SerialEmulatorConfig

DEFAULT_X2_PROTOCOL_ENABLED = False
DEFAULT_X2_SCAN_SOURCE = "x2_virtual_serial"
DEFAULT_X2_VENDOR_PROFILE = "/workspace/profiles/x2-vendor-sim.yaml"
DEFAULT_X2_VIRTUAL_SERIAL_LINK = "/tmp/navlab_x2"
DEFAULT_X2_SCAN_IDEAL_TOPIC = "/scan_ideal"
DEFAULT_X2_VENDOR_SCAN_TOPIC = "/navlab/x2/vendor_scan"
DEFAULT_X2_SCAN_TOPIC = "/scan"
DEFAULT_X2_STATUS_TOPIC = "/sim/x2/status"
DEFAULT_X2_SAMPLE_RATE_HZ = 3000.0
DEFAULT_X2_SCAN_FREQUENCY_HZ = 7.0
DEFAULT_X2_SCAN_FREQUENCY_MIN_HZ = 4.0
DEFAULT_X2_SCAN_FREQUENCY_MAX_HZ = 8.0
DEFAULT_X2_SCAN_FREQUENCY_JITTER_HZ = 0.0
DEFAULT_X2_RANGE_MIN_M = 0.1
DEFAULT_X2_RANGE_MAX_M = 8.0
DEFAULT_X2_STATIC_RANGE_M = 1.5
DEFAULT_X2_RANGE_NOISE_STDDEV_M = 0.0
DEFAULT_X2_RANGE_NOISE_STDDEV_PER_M = 0.0
DEFAULT_X2_DROPOUT_RATE = 0.0
DEFAULT_X2_AUTO_START = True
DEFAULT_DOWN_RANGEFINDER_ENABLED = True
DEFAULT_DOWN_RANGEFINDER_SCAN_IDEAL_TOPIC = "/rangefinder/down/scan_ideal"
DEFAULT_DOWN_RANGEFINDER_RANGE_TOPIC = "/rangefinder/down/range"
DEFAULT_DOWN_RANGEFINDER_STATUS_TOPIC = "/rangefinder/down/status"
DEFAULT_DOWN_RANGEFINDER_ENDPOINT = "udpout:mavlink-router:14550"
DEFAULT_DOWN_RANGEFINDER_FRAME_ID = "rangefinder_down_frame"
DEFAULT_DOWN_RANGEFINDER_MAVLINK_ORIENTATION = "MAV_SENSOR_ROTATION_PITCH_270"
DEFAULT_DOWN_RANGEFINDER_SOURCE_SYSTEM = 1
DEFAULT_DOWN_RANGEFINDER_SOURCE_COMPONENT = 158
DEFAULT_DOWN_RANGEFINDER_SENSOR_ID = 1
DEFAULT_DOWN_RANGEFINDER_RATE_HZ = 20.0
DEFAULT_DOWN_RANGEFINDER_MIN_M = 0.05
DEFAULT_DOWN_RANGEFINDER_MAX_M = 6.0
DEFAULT_DOWN_RANGEFINDER_COVARIANCE_CM = 2
DEFAULT_SCAN_INTEGRITY_ENABLED = False
DEFAULT_SCAN_INTEGRITY_INPUT_SCAN_TOPIC = "/navlab/x2/scan_normalized"
DEFAULT_SCAN_INTEGRITY_OUTPUT_SCAN_TOPIC = "/scan"
DEFAULT_SCAN_INTEGRITY_STATUS_TOPIC = "/navlab/scan_integrity/status"
DEFAULT_SCAN_INTEGRITY_EVENTS_TOPIC = "/navlab/scan_integrity/events"
DEFAULT_SCAN_INTEGRITY_FAULT_TOPIC = "/navlab/scan_integrity/fault_injection"
DEFAULT_SCAN_INTEGRITY_ATTITUDE_SOURCE_TOPIC = "/imu"
DEFAULT_SCAN_INTEGRITY_ATTITUDE_SOURCE_TYPE = "imu"
DEFAULT_SCAN_INTEGRITY_RANGE_TOPIC = "/rangefinder/down/range"
DEFAULT_SCAN_INTEGRITY_BASE_FRAME_ID = "base_link"
DEFAULT_SCAN_INTEGRITY_SCAN_FRAME_ID = "base_scan"
DEFAULT_SCAN_INTEGRITY_SOFT_TILT_DEG = 3.0
DEFAULT_SCAN_INTEGRITY_HARD_TILT_DEG = 6.0
DEFAULT_SCAN_INTEGRITY_MAX_DROPPED_SCAN_RATIO = 0.05
DEFAULT_SCAN_INTEGRITY_MAX_CLIPPED_BEAM_RATIO = 0.20
DEFAULT_SCAN_INTEGRITY_MAX_SCAN_ATTITUDE_OFFSET_MS = 50.0
DEFAULT_SCAN_INTEGRITY_MIN_ATTITUDE_RATE_HZ = 20.0
DEFAULT_SCAN_INTEGRITY_FLOOR_HIT_GUARD_RANGE_M = 8.0
DEFAULT_SCAN_INTEGRITY_MIN_LIDAR_HEIGHT_M = 0.25
DEFAULT_SCAN_INTEGRITY_MIN_DOWNWARD_RAY_Z = 0.05


@dataclass(slots=True)
class X2ProtocolConfig:
    enabled: BoolWithSource
    scan_source: ValueWithSource
    profile: ValueWithSource
    virtual_serial_link: ValueWithSource
    scan_ideal_topic: ValueWithSource
    vendor_scan_topic: ValueWithSource
    scan_topic: ValueWithSource
    status_topic: ValueWithSource
    sample_rate_hz: FloatWithSource
    scan_frequency_hz: FloatWithSource
    scan_frequency_min_hz: FloatWithSource
    scan_frequency_max_hz: FloatWithSource
    scan_frequency_jitter_hz: FloatWithSource
    range_min_m: FloatWithSource
    range_max_m: FloatWithSource
    static_range_m: FloatWithSource
    range_noise_stddev_m: FloatWithSource
    range_noise_stddev_per_m: FloatWithSource
    dropout_rate: FloatWithSource
    auto_start: BoolWithSource
    random_seed: ValueWithSource


@dataclass(frozen=True, slots=True)
class X2SensorRuntimeConfig:
    enabled: bool
    scan_source: str
    profile: Path
    virtual_serial_link: Path
    scan_ideal_topic: str
    vendor_scan_topic: str
    scan_topic: str
    status_topic: str
    sample_rate_hz: float
    scan_frequency_hz: float
    scan_frequency_min_hz: float
    scan_frequency_max_hz: float
    scan_frequency_jitter_hz: float
    range_min_m: float
    range_max_m: float
    static_range_m: float
    range_noise_stddev_m: float
    range_noise_stddev_per_m: float
    dropout_rate: float
    auto_start: bool
    random_seed: int | None

    @classmethod
    def load(cls) -> X2SensorRuntimeConfig:
        config = load_x2_protocol_config()
        return cls(
            enabled=config.enabled.value,
            scan_source=config.scan_source.value,
            profile=Path(config.profile.value),
            virtual_serial_link=Path(config.virtual_serial_link.value),
            scan_ideal_topic=config.scan_ideal_topic.value,
            vendor_scan_topic=config.vendor_scan_topic.value,
            scan_topic=config.scan_topic.value,
            status_topic=config.status_topic.value,
            sample_rate_hz=config.sample_rate_hz.value,
            scan_frequency_hz=config.scan_frequency_hz.value,
            scan_frequency_min_hz=config.scan_frequency_min_hz.value,
            scan_frequency_max_hz=config.scan_frequency_max_hz.value,
            scan_frequency_jitter_hz=config.scan_frequency_jitter_hz.value,
            range_min_m=config.range_min_m.value,
            range_max_m=config.range_max_m.value,
            static_range_m=config.static_range_m.value,
            range_noise_stddev_m=config.range_noise_stddev_m.value,
            range_noise_stddev_per_m=config.range_noise_stddev_per_m.value,
            dropout_rate=config.dropout_rate.value,
            auto_start=config.auto_start.value,
            random_seed=_optional_int(config.random_seed.value),
        )

    def emulator_config(self) -> X2SerialEmulatorConfig:
        return X2SerialEmulatorConfig(
            virtual_serial_link=self.virtual_serial_link,
            scan_frequency_hz=self.scan_frequency_hz,
            scan_frequency_min_hz=self.scan_frequency_min_hz,
            scan_frequency_max_hz=self.scan_frequency_max_hz,
            scan_frequency_jitter_hz=self.scan_frequency_jitter_hz,
            sample_rate_hz=self.sample_rate_hz,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
            status_topic=self.status_topic,
            random_seed=self.random_seed,
        )


@dataclass(slots=True)
class DownRangefinderConfig:
    enabled: BoolWithSource
    scan_ideal_topic: ValueWithSource
    range_topic: ValueWithSource
    status_topic: ValueWithSource
    endpoint: ValueWithSource
    frame_id: ValueWithSource
    mavlink_orientation: ValueWithSource
    source_system: ValueWithSource
    source_component: ValueWithSource
    sensor_id: ValueWithSource
    rate_hz: FloatWithSource
    min_distance_m: FloatWithSource
    max_distance_m: FloatWithSource
    covariance_cm: ValueWithSource
    model_pose: ValueWithSource
    model_update_rate_hz: FloatWithSource
    model_ray_count: ValueWithSource
    model_noise_stddev_m: FloatWithSource


@dataclass(frozen=True, slots=True)
class DownRangefinderRuntimeConfig:
    enabled: bool
    scan_ideal_topic: str
    range_topic: str
    status_topic: str
    endpoint: str
    frame_id: str
    mavlink_orientation: str
    source_system: int
    source_component: int
    sensor_id: int
    rate_hz: float
    min_distance_m: float
    max_distance_m: float
    covariance_cm: int

    @classmethod
    def load(cls) -> DownRangefinderRuntimeConfig:
        config = load_down_rangefinder_config()
        return cls(
            enabled=config.enabled.value,
            scan_ideal_topic=config.scan_ideal_topic.value,
            range_topic=config.range_topic.value,
            status_topic=config.status_topic.value,
            endpoint=config.endpoint.value,
            frame_id=config.frame_id.value,
            mavlink_orientation=config.mavlink_orientation.value,
            source_system=_required_int(config.source_system.value, key="source_system"),
            source_component=_required_int(config.source_component.value, key="source_component"),
            sensor_id=_required_int(config.sensor_id.value, key="sensor_id"),
            rate_hz=config.rate_hz.value,
            min_distance_m=config.min_distance_m.value,
            max_distance_m=config.max_distance_m.value,
            covariance_cm=_required_int(config.covariance_cm.value, key="covariance_cm"),
        )


@dataclass(slots=True)
class ScanIntegrityConfig:
    enabled: BoolWithSource
    input_scan_topic: ValueWithSource
    output_scan_topic: ValueWithSource
    status_topic: ValueWithSource
    events_topic: ValueWithSource
    fault_injection_topic: ValueWithSource
    attitude_source_topic: ValueWithSource
    attitude_source_type: ValueWithSource
    range_topic: ValueWithSource
    base_frame_id: ValueWithSource
    scan_frame_id: ValueWithSource
    soft_tilt_deg: FloatWithSource
    hard_tilt_deg: FloatWithSource
    max_dropped_scan_ratio: FloatWithSource
    max_clipped_beam_ratio: FloatWithSource
    max_scan_attitude_time_offset_ms: FloatWithSource
    min_attitude_rate_hz: FloatWithSource
    floor_hit_guard_range_m: FloatWithSource
    min_lidar_height_m: FloatWithSource
    min_downward_ray_z: FloatWithSource


@dataclass(frozen=True, slots=True)
class ScanIntegrityRuntimeConfig:
    enabled: bool
    input_scan_topic: str
    output_scan_topic: str
    status_topic: str
    events_topic: str
    fault_injection_topic: str
    attitude_source_topic: str
    attitude_source_type: str
    range_topic: str
    base_frame_id: str
    scan_frame_id: str
    soft_tilt_deg: float
    hard_tilt_deg: float
    max_dropped_scan_ratio: float
    max_clipped_beam_ratio: float
    max_scan_attitude_time_offset_ms: float
    min_attitude_rate_hz: float
    floor_hit_guard_range_m: float
    min_lidar_height_m: float
    min_downward_ray_z: float

    @classmethod
    def load(cls) -> ScanIntegrityRuntimeConfig:
        config = load_scan_integrity_config()
        return cls(
            enabled=config.enabled.value,
            input_scan_topic=config.input_scan_topic.value,
            output_scan_topic=config.output_scan_topic.value,
            status_topic=config.status_topic.value,
            events_topic=config.events_topic.value,
            fault_injection_topic=config.fault_injection_topic.value,
            attitude_source_topic=config.attitude_source_topic.value,
            attitude_source_type=config.attitude_source_type.value,
            range_topic=config.range_topic.value,
            base_frame_id=config.base_frame_id.value,
            scan_frame_id=config.scan_frame_id.value,
            soft_tilt_deg=config.soft_tilt_deg.value,
            hard_tilt_deg=config.hard_tilt_deg.value,
            max_dropped_scan_ratio=config.max_dropped_scan_ratio.value,
            max_clipped_beam_ratio=config.max_clipped_beam_ratio.value,
            max_scan_attitude_time_offset_ms=config.max_scan_attitude_time_offset_ms.value,
            min_attitude_rate_hz=config.min_attitude_rate_hz.value,
            floor_hit_guard_range_m=config.floor_hit_guard_range_m.value,
            min_lidar_height_m=config.min_lidar_height_m.value,
            min_downward_ray_z=config.min_downward_ray_z.value,
        )


def load_x2_protocol_config(path: str | Path | None = None) -> X2ProtocolConfig:
    config_file, config = load_navlab_config(path)
    raw_gazebo_sensor = section(config, "gazebo_sensor", path=config_file)
    raw_x2 = section(raw_gazebo_sensor, "x2_protocol", path=config_file, default={})
    return X2ProtocolConfig(
        enabled=resolve_bool_value(raw_x2, "enabled", DEFAULT_X2_PROTOCOL_ENABLED),
        scan_source=resolve_str_value(raw_x2, "scan_source", DEFAULT_X2_SCAN_SOURCE),
        profile=resolve_str_value(raw_x2, "profile", DEFAULT_X2_VENDOR_PROFILE),
        virtual_serial_link=resolve_str_value(raw_x2, "virtual_serial_link", DEFAULT_X2_VIRTUAL_SERIAL_LINK),
        scan_ideal_topic=resolve_str_value(raw_x2, "scan_ideal_topic", DEFAULT_X2_SCAN_IDEAL_TOPIC),
        vendor_scan_topic=resolve_str_value(raw_x2, "vendor_scan_topic", DEFAULT_X2_VENDOR_SCAN_TOPIC),
        scan_topic=resolve_str_value(raw_x2, "scan_topic", DEFAULT_X2_SCAN_TOPIC),
        status_topic=resolve_str_value(raw_x2, "status_topic", DEFAULT_X2_STATUS_TOPIC),
        sample_rate_hz=resolve_float_value(raw_x2, "sample_rate_hz", DEFAULT_X2_SAMPLE_RATE_HZ),
        scan_frequency_hz=resolve_float_value(raw_x2, "scan_frequency_hz", DEFAULT_X2_SCAN_FREQUENCY_HZ),
        scan_frequency_min_hz=resolve_float_value(
            raw_x2,
            "scan_frequency_min_hz",
            DEFAULT_X2_SCAN_FREQUENCY_MIN_HZ,
        ),
        scan_frequency_max_hz=resolve_float_value(
            raw_x2,
            "scan_frequency_max_hz",
            DEFAULT_X2_SCAN_FREQUENCY_MAX_HZ,
        ),
        scan_frequency_jitter_hz=resolve_float_value(
            raw_x2,
            "scan_frequency_jitter_hz",
            DEFAULT_X2_SCAN_FREQUENCY_JITTER_HZ,
        ),
        range_min_m=resolve_float_value(raw_x2, "range_min_m", DEFAULT_X2_RANGE_MIN_M),
        range_max_m=resolve_float_value(raw_x2, "range_max_m", DEFAULT_X2_RANGE_MAX_M),
        static_range_m=resolve_float_value(raw_x2, "static_range_m", DEFAULT_X2_STATIC_RANGE_M),
        range_noise_stddev_m=resolve_float_value(
            raw_x2,
            "range_noise_stddev_m",
            DEFAULT_X2_RANGE_NOISE_STDDEV_M,
        ),
        range_noise_stddev_per_m=resolve_float_value(
            raw_x2,
            "range_noise_stddev_per_m",
            DEFAULT_X2_RANGE_NOISE_STDDEV_PER_M,
        ),
        dropout_rate=resolve_float_value(raw_x2, "dropout_rate", DEFAULT_X2_DROPOUT_RATE),
        auto_start=resolve_bool_value(raw_x2, "auto_start", DEFAULT_X2_AUTO_START),
        random_seed=resolve_str_value(raw_x2, "random_seed", ""),
    )


def load_down_rangefinder_config(path: str | Path | None = None) -> DownRangefinderConfig:
    config_file, config = load_navlab_config(path)
    raw_gazebo_sensor = section(config, "gazebo_sensor", path=config_file)
    raw_rangefinder = section(raw_gazebo_sensor, "down_rangefinder", path=config_file, default={})
    return DownRangefinderConfig(
        enabled=resolve_bool_value(raw_rangefinder, "enabled", DEFAULT_DOWN_RANGEFINDER_ENABLED),
        scan_ideal_topic=resolve_str_value(
            raw_rangefinder,
            "scan_ideal_topic",
            DEFAULT_DOWN_RANGEFINDER_SCAN_IDEAL_TOPIC,
        ),
        range_topic=resolve_str_value(raw_rangefinder, "range_topic", DEFAULT_DOWN_RANGEFINDER_RANGE_TOPIC),
        status_topic=resolve_str_value(raw_rangefinder, "status_topic", DEFAULT_DOWN_RANGEFINDER_STATUS_TOPIC),
        endpoint=resolve_str_value(raw_rangefinder, "endpoint", DEFAULT_DOWN_RANGEFINDER_ENDPOINT),
        frame_id=resolve_str_value(raw_rangefinder, "frame_id", DEFAULT_DOWN_RANGEFINDER_FRAME_ID),
        mavlink_orientation=resolve_str_value(
            raw_rangefinder,
            "mavlink_orientation",
            DEFAULT_DOWN_RANGEFINDER_MAVLINK_ORIENTATION,
        ),
        source_system=resolve_str_value(
            raw_rangefinder,
            "source_system",
            str(DEFAULT_DOWN_RANGEFINDER_SOURCE_SYSTEM),
        ),
        source_component=resolve_str_value(
            raw_rangefinder,
            "source_component",
            str(DEFAULT_DOWN_RANGEFINDER_SOURCE_COMPONENT),
        ),
        sensor_id=resolve_str_value(raw_rangefinder, "sensor_id", str(DEFAULT_DOWN_RANGEFINDER_SENSOR_ID)),
        rate_hz=resolve_float_value(raw_rangefinder, "rate_hz", DEFAULT_DOWN_RANGEFINDER_RATE_HZ),
        min_distance_m=resolve_float_value(
            raw_rangefinder,
            "min_distance_m",
            DEFAULT_DOWN_RANGEFINDER_MIN_M,
        ),
        max_distance_m=resolve_float_value(
            raw_rangefinder,
            "max_distance_m",
            DEFAULT_DOWN_RANGEFINDER_MAX_M,
        ),
        covariance_cm=resolve_str_value(
            raw_rangefinder,
            "covariance_cm",
            str(DEFAULT_DOWN_RANGEFINDER_COVARIANCE_CM),
        ),
        model_pose=resolve_str_value(raw_rangefinder, "model_pose", "0 0 -0.02 0 1.5707963267948966 0"),
        model_update_rate_hz=resolve_float_value(raw_rangefinder, "model_update_rate_hz", 20.0),
        model_ray_count=resolve_str_value(raw_rangefinder, "model_ray_count", "1"),
        model_noise_stddev_m=resolve_float_value(raw_rangefinder, "model_noise_stddev_m", 0.0),
    )


def load_scan_integrity_config(path: str | Path | None = None) -> ScanIntegrityConfig:
    config_file, config = load_navlab_config(path)
    raw_gazebo_sensor = section(config, "gazebo_sensor", path=config_file)
    raw_integrity = section(raw_gazebo_sensor, "scan_integrity", path=config_file, default={})
    return ScanIntegrityConfig(
        enabled=resolve_bool_value(raw_integrity, "enabled", DEFAULT_SCAN_INTEGRITY_ENABLED),
        input_scan_topic=resolve_str_value(
            raw_integrity, "input_scan_topic", DEFAULT_SCAN_INTEGRITY_INPUT_SCAN_TOPIC
        ),
        output_scan_topic=resolve_str_value(
            raw_integrity, "output_scan_topic", DEFAULT_SCAN_INTEGRITY_OUTPUT_SCAN_TOPIC
        ),
        status_topic=resolve_str_value(raw_integrity, "status_topic", DEFAULT_SCAN_INTEGRITY_STATUS_TOPIC),
        events_topic=resolve_str_value(raw_integrity, "events_topic", DEFAULT_SCAN_INTEGRITY_EVENTS_TOPIC),
        fault_injection_topic=resolve_str_value(
            raw_integrity, "fault_injection_topic", DEFAULT_SCAN_INTEGRITY_FAULT_TOPIC
        ),
        attitude_source_topic=resolve_str_value(
            raw_integrity, "attitude_source_topic", DEFAULT_SCAN_INTEGRITY_ATTITUDE_SOURCE_TOPIC
        ),
        attitude_source_type=resolve_str_value(
            raw_integrity, "attitude_source_type", DEFAULT_SCAN_INTEGRITY_ATTITUDE_SOURCE_TYPE
        ),
        range_topic=resolve_str_value(raw_integrity, "range_topic", DEFAULT_SCAN_INTEGRITY_RANGE_TOPIC),
        base_frame_id=resolve_str_value(raw_integrity, "base_frame_id", DEFAULT_SCAN_INTEGRITY_BASE_FRAME_ID),
        scan_frame_id=resolve_str_value(raw_integrity, "scan_frame_id", DEFAULT_SCAN_INTEGRITY_SCAN_FRAME_ID),
        soft_tilt_deg=resolve_float_value(raw_integrity, "soft_tilt_deg", DEFAULT_SCAN_INTEGRITY_SOFT_TILT_DEG),
        hard_tilt_deg=resolve_float_value(raw_integrity, "hard_tilt_deg", DEFAULT_SCAN_INTEGRITY_HARD_TILT_DEG),
        max_dropped_scan_ratio=resolve_float_value(
            raw_integrity, "max_dropped_scan_ratio", DEFAULT_SCAN_INTEGRITY_MAX_DROPPED_SCAN_RATIO
        ),
        max_clipped_beam_ratio=resolve_float_value(
            raw_integrity, "max_clipped_beam_ratio", DEFAULT_SCAN_INTEGRITY_MAX_CLIPPED_BEAM_RATIO
        ),
        max_scan_attitude_time_offset_ms=resolve_float_value(
            raw_integrity,
            "max_scan_attitude_time_offset_ms",
            DEFAULT_SCAN_INTEGRITY_MAX_SCAN_ATTITUDE_OFFSET_MS,
        ),
        min_attitude_rate_hz=resolve_float_value(
            raw_integrity, "min_attitude_rate_hz", DEFAULT_SCAN_INTEGRITY_MIN_ATTITUDE_RATE_HZ
        ),
        floor_hit_guard_range_m=resolve_float_value(
            raw_integrity, "floor_hit_guard_range_m", DEFAULT_SCAN_INTEGRITY_FLOOR_HIT_GUARD_RANGE_M
        ),
        min_lidar_height_m=resolve_float_value(
            raw_integrity, "min_lidar_height_m", DEFAULT_SCAN_INTEGRITY_MIN_LIDAR_HEIGHT_M
        ),
        min_downward_ray_z=resolve_float_value(
            raw_integrity, "min_downward_ray_z", DEFAULT_SCAN_INTEGRITY_MIN_DOWNWARD_RAY_Z
        ),
    )


def _optional_int(value: str) -> int | None:
    if value == "":
        return None
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError("Invalid value for 'random_seed': expected an integer") from exc


def _required_int(value: str, *, key: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"Invalid value for '{key}': expected an integer") from exc
