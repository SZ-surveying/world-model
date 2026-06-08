from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from src.runtime.errors import PathMappingError


@dataclass(frozen=True, slots=True)
class WorkspacePathMapper:
    host_root: Path
    backend_workspace_root: str
    backend: str

    def backend_path(self, path: Path) -> str:
        if self.backend == "process" and str(path).startswith("/workspace/"):
            raise PathMappingError(f"process backend cannot consume container-only path {path}")
        resolved_root = self.host_root.resolve()
        resolved_path = path.resolve()
        try:
            relative = resolved_path.relative_to(resolved_root)
        except ValueError:
            if self.backend == "process":
                return str(resolved_path)
            raise PathMappingError(f"path {path} is outside workspace root {self.host_root}")
        if self.backend == "process":
            return str(resolved_path)
        return str(Path(self.backend_workspace_root) / relative)
