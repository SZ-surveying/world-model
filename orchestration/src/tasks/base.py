from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any, ClassVar


class OrchestrationTask(ABC):
    TASK_NAME: ClassVar[str] = ""
    TASK_DESCRIPTION: ClassVar[str] = ""

    @property
    def description(self) -> str:
        return self.TASK_DESCRIPTION

    @abstractmethod
    def run(self, **kwargs: object) -> int:
        raise NotImplementedError

    def build_real_task_doctor(
        self,
        *,
        config: object,
        upstream: Mapping[str, Any],
    ) -> Mapping[str, Any] | None:
        return None
