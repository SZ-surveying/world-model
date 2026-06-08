from __future__ import annotations

from dataclasses import dataclass

from src.runtime.errors import RuntimeModeViolationError

RUNTIME_MODE_SIMULATION = "simulation"
RUNTIME_MODE_REAL = "real"

SERVICE_ROLE_GENERIC = "generic"
SERVICE_ROLE_SIMULATION_STACK = "simulation_stack"
SERVICE_ROLE_GAZEBO = "gazebo"
SERVICE_ROLE_SITL = "sitl"
SERVICE_ROLE_OFFICIAL_BASELINE = "official_baseline"
SERVICE_ROLE_GAZEBO_SENSOR = "gazebo_sensor"
SERVICE_ROLE_SDF_OVERLAY = "sdf_overlay"
SERVICE_ROLE_SIMULATION_ROSBAG = "simulation_rosbag"
SERVICE_ROLE_SIMULATION_PROBE = "simulation_probe"
SERVICE_ROLE_SIMULATION_RUNTIME = "simulation_runtime"
SERVICE_ROLE_REAL_ONLY = "real_only"

SIMULATION_ONLY_SERVICE_ROLES = frozenset(
    {
        SERVICE_ROLE_SIMULATION_STACK,
        SERVICE_ROLE_GAZEBO,
        SERVICE_ROLE_SITL,
        SERVICE_ROLE_OFFICIAL_BASELINE,
        SERVICE_ROLE_GAZEBO_SENSOR,
        SERVICE_ROLE_SDF_OVERLAY,
        SERVICE_ROLE_SIMULATION_ROSBAG,
        SERVICE_ROLE_SIMULATION_PROBE,
        SERVICE_ROLE_SIMULATION_RUNTIME,
    }
)
REAL_ONLY_SERVICE_ROLES = frozenset({SERVICE_ROLE_REAL_ONLY})


@dataclass(frozen=True, slots=True)
class RuntimeModePolicy:
    backend: str
    mode: str
    fail_on_mode_violation: bool = True

    def validate_backend_mode(self) -> None:
        if (self.backend, self.mode) in {("docker", RUNTIME_MODE_SIMULATION), ("process", RUNTIME_MODE_REAL)}:
            return
        self._violate(
            "unsupported_backend_mode",
            f"supported combinations are docker+simulation and process+real, got {self.backend}+{self.mode}",
        )

    def assert_service_allowed(self, *, service_name: str, service_role: str) -> None:
        self.validate_backend_mode()
        if self.mode == RUNTIME_MODE_REAL and service_role in SIMULATION_ONLY_SERVICE_ROLES:
            self._violate(
                f"{service_role}_requested",
                f"runtime mode real forbids simulation-only service {service_name!r}",
            )
        if self.mode == RUNTIME_MODE_SIMULATION and service_role in REAL_ONLY_SERVICE_ROLES:
            self._violate(
                f"{service_role}_requested",
                f"runtime mode simulation forbids real-only service {service_name!r}",
            )

    def assert_source_allowed(self, *, source_name: str, source_role: str) -> None:
        self.assert_service_allowed(service_name=source_name, service_role=source_role)

    def _violate(self, blocker: str, detail: str) -> None:
        if not self.fail_on_mode_violation:
            return
        raise RuntimeModeViolationError(f"runtime_mode_violation:{blocker}: {detail}")
