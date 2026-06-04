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


@dataclass(slots=True)
class X2ProtocolConfig:
    enabled: BoolWithSource
    scan_source: ValueWithSource
    profile: ValueWithSource
    virtual_serial_link: ValueWithSource
    scan_ideal_topic: ValueWithSource
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


def _optional_int(value: str) -> int | None:
    if value == "":
        return None
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError("Invalid value for 'random_seed': expected an integer") from exc
