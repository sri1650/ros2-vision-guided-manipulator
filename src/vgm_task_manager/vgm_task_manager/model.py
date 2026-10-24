from dataclasses import dataclass
from enum import Enum, auto


class TaskState(Enum):
    IDLE = auto()
    TARGET_SELECTED = auto()
    DISPATCHED = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELED = auto()


@dataclass(frozen=True)
class Candidate:
    object_id: str
    class_label: str
    confidence: float
    depth_m: float


class TaskCoordinator:
    """Deterministic task state and target-selection policy, independent of ROS."""

    def __init__(self, target_class: str, minimum_confidence: float = 0.75) -> None:
        if not target_class:
            raise ValueError("target_class must not be empty")
        if not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be within [0, 1]")
        self.target_class = target_class
        self.minimum_confidence = minimum_confidence
        self.state = TaskState.IDLE
        self.selected: Candidate | None = None

    def select(self, candidates: list[Candidate]) -> Candidate | None:
        if self.state is not TaskState.IDLE:
            return self.selected
        eligible = [
            item
            for item in candidates
            if item.class_label == self.target_class
            and item.confidence >= self.minimum_confidence
            and item.depth_m > 0.0
        ]
        if not eligible:
            return None
        self.selected = max(eligible, key=lambda item: (item.confidence, -item.depth_m))
        self.state = TaskState.TARGET_SELECTED
        return self.selected

    def mark_dispatched(self) -> None:
        self._transition(TaskState.TARGET_SELECTED, TaskState.DISPATCHED)

    def mark_running(self) -> None:
        self._transition(TaskState.DISPATCHED, TaskState.RUNNING)

    def complete(self, success: bool) -> None:
        if self.state not in {TaskState.DISPATCHED, TaskState.RUNNING}:
            raise RuntimeError(f"cannot complete task from {self.state.name}")
        self.state = TaskState.SUCCEEDED if success else TaskState.FAILED

    def cancel(self) -> None:
        if self.state in {TaskState.SUCCEEDED, TaskState.FAILED, TaskState.CANCELED}:
            raise RuntimeError(f"cannot cancel task from {self.state.name}")
        self.state = TaskState.CANCELED

    def reset(self) -> None:
        if self.state not in {TaskState.SUCCEEDED, TaskState.FAILED, TaskState.CANCELED}:
            raise RuntimeError("only a terminal task can be reset")
        self.selected = None
        self.state = TaskState.IDLE

    def _transition(self, expected: TaskState, target: TaskState) -> None:
        if self.state is not expected:
            raise RuntimeError(f"expected {expected.name}, got {self.state.name}")
        self.state = target

