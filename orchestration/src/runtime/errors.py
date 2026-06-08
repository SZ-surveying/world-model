from __future__ import annotations


class RuntimeBackendError(RuntimeError):
    """Base error for runtime backend failures that should become explicit blockers."""


class BackendConfigError(RuntimeBackendError):
    """Raised when a backend is selected but its required config is missing or invalid."""


class RuntimeModeViolationError(RuntimeBackendError):
    """Raised when a runtime mode tries to use a forbidden service/source."""


class ServiceStartError(RuntimeBackendError):
    """Raised when a required service cannot be started."""


class ServiceWaitError(RuntimeBackendError):
    """Raised when a required service exits incorrectly or cannot be waited on."""


class PathMappingError(RuntimeBackendError):
    """Raised when a backend-visible path cannot be derived safely."""
