from __future__ import annotations

from abc import ABC, abstractmethod
from typing import ClassVar


class OrchestrationTask(ABC):
    TASK_NAME: ClassVar[str] = ""
    TASK_DESCRIPTION: ClassVar[str] = ""

    @property
    def description(self) -> str:
        return self.TASK_DESCRIPTION

    @abstractmethod
    def run(self, **kwargs: object) -> int:
        raise NotImplementedError
