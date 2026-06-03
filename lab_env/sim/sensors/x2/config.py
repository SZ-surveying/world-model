from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from lab_env.config import load_runtime_config, load_x2_protocol_sim_config
from lab_env.sim.sensors.x2.emulator import X2SerialEmulatorConfig


@dataclass(frozen=True, slots=True)
class X2SensorRuntimeConfig:
    enabled: bool
    profile: Path
    virtual_serial_link: Path
    scan_ideal_topic: str
    scan_topic: str
    status_topic: str
    sample_rate_hz: float
    scan_frequency_hz: float
    scan_frequency_min_hz: float
    scan_frequency_max_hz: float
    range_min_m: float
    range_max_m: float

    @classmethod
    def load(cls) -> X2SensorRuntimeConfig:
        project_runtime = load_runtime_config()
        config = load_x2_protocol_sim_config(project_runtime)
        return cls(
            enabled=config.enabled.value,
            profile=Path(config.profile.value),
            virtual_serial_link=Path(config.virtual_serial_link.value),
            scan_ideal_topic=config.scan_ideal_topic.value,
            scan_topic=config.scan_topic.value,
            status_topic=config.status_topic.value,
            sample_rate_hz=config.sample_rate_hz.value,
            scan_frequency_hz=config.scan_frequency_hz.value,
            scan_frequency_min_hz=config.scan_frequency_min_hz.value,
            scan_frequency_max_hz=config.scan_frequency_max_hz.value,
            range_min_m=config.range_min_m.value,
            range_max_m=config.range_max_m.value,
        )

    def emulator_config(self) -> X2SerialEmulatorConfig:
        return X2SerialEmulatorConfig(
            virtual_serial_link=self.virtual_serial_link,
            scan_frequency_hz=self.scan_frequency_hz,
            sample_rate_hz=self.sample_rate_hz,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
            status_topic=self.status_topic,
        )
