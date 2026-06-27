"""Pure task FSM summary helpers shared by mission runtimes."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

TASK_FSM_SCHEMA_VERSION = "navlab.task_fsm.v1"

JsonMap = Mapping[str, object]


@dataclass(frozen=True, slots=True)
class TaskFsmState:
    """One task FSM state entry for summary artifacts."""

    state: str
    entered_at: str
    reason: str
    evidence: JsonMap = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-serializable state payload."""

        return {
            "state": self.state,
            "entered_at": self.entered_at,
            "reason": self.reason,
            "evidence": dict(self.evidence),
        }


@dataclass(frozen=True, slots=True)
class TaskFsmTransition:
    """One coarse task FSM transition with command/topic evidence attached."""

    task_id: str
    fsm_name: str
    from_state: str
    to_state: str
    event: str
    reason_code: str
    ok: bool
    evidence: JsonMap = field(default_factory=dict)
    at: str = "unknown"
    blocker: str | None = None

    @property
    def blocked(self) -> bool:
        """Whether this transition blocks task progress."""

        return not self.ok

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-serializable transition payload."""

        payload: dict[str, object] = {
            "task_id": self.task_id,
            "fsm_name": self.fsm_name,
            "from_state": self.from_state,
            "to_state": self.to_state,
            "event": self.event,
            "reason_code": self.reason_code,
            "ok": self.ok,
            "blocked": self.blocked,
            "evidence": dict(self.evidence),
            "at": self.at,
        }
        if self.blocker is not None:
            payload["blocker"] = self.blocker
        return payload


@dataclass(frozen=True, slots=True)
class TaskFsmSummary:
    """Task FSM summary shape used inside task run summaries."""

    task_id: str
    fsm_name: str
    mode: str
    ok: bool
    current_state: str
    states: Sequence[TaskFsmState]
    transitions: Sequence[TaskFsmTransition]
    failed_state: str | None = None
    blockers: Sequence[str] = ()
    schema_version: str = TASK_FSM_SCHEMA_VERSION

    @property
    def blocked(self) -> bool:
        """Whether the FSM summary has blockers."""

        return not self.ok or bool(self.blockers)

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-serializable summary payload."""

        payload: dict[str, object] = {
            "schema_version": self.schema_version,
            "task_id": self.task_id,
            "fsm_name": self.fsm_name,
            "mode": self.mode,
            "ok": self.ok,
            "blocked": self.blocked,
            "current_state": self.current_state,
            "blockers": list(self.blockers),
            "states": [state.to_dict() for state in self.states],
            "transitions": [transition.to_dict() for transition in self.transitions],
        }
        if self.failed_state is not None:
            payload["failed_state"] = self.failed_state
        return payload


class TaskFsmRecorder:
    """Side-effect-free recorder for coarse task FSM summary artifacts."""

    def __init__(
        self,
        *,
        task_id: str,
        fsm_name: str,
        mode: str,
        initial_state: str,
        entered_at: str = "planned",
        reason: str = "planned",
        evidence: JsonMap | None = None,
    ) -> None:
        """Create a recorder at an initial coarse task state."""

        self._task_id = task_id
        self._fsm_name = fsm_name
        self._mode = mode
        self._states = [
            TaskFsmState(
                state=initial_state,
                entered_at=entered_at,
                reason=reason,
                evidence={} if evidence is None else dict(evidence),
            )
        ]
        self._transitions: list[TaskFsmTransition] = []
        self._current_state = initial_state
        self._failed_state: str | None = None
        self._blockers: list[str] = []

    @property
    def current_state(self) -> str:
        """Current coarse task state."""

        return self._current_state

    def enter_state(
        self,
        *,
        state: str,
        entered_at: str,
        reason: str,
        evidence: JsonMap | None = None,
    ) -> None:
        """Record entry into a coarse task state."""

        self._current_state = state
        self._states.append(
            TaskFsmState(
                state=state,
                entered_at=entered_at,
                reason=reason,
                evidence={} if evidence is None else dict(evidence),
            )
        )

    def transition(
        self,
        *,
        to_state: str,
        event: str,
        reason_code: str,
        ok: bool,
        at: str,
        evidence: JsonMap | None = None,
        blocker: str | None = None,
    ) -> None:
        """Record a transition from the current state."""

        transition = TaskFsmTransition(
            task_id=self._task_id,
            fsm_name=self._fsm_name,
            from_state=self._current_state,
            to_state=to_state,
            event=event,
            reason_code=reason_code,
            ok=ok,
            evidence={} if evidence is None else dict(evidence),
            at=at,
            blocker=blocker,
        )
        self._transitions.append(transition)
        if ok:
            self._current_state = to_state
            return
        self._failed_state = to_state
        if blocker is not None:
            self._blockers.append(blocker)

    def summary(self) -> TaskFsmSummary:
        """Build the immutable summary payload."""

        return TaskFsmSummary(
            task_id=self._task_id,
            fsm_name=self._fsm_name,
            mode=self._mode,
            ok=not self._blockers and self._failed_state is None,
            current_state=self._current_state,
            failed_state=self._failed_state,
            blockers=tuple(self._blockers),
            states=tuple(self._states),
            transitions=tuple(self._transitions),
        )
