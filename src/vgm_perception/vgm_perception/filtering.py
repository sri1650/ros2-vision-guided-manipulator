"""Small stateful filters for noisy object position measurements."""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from .models import PoseEstimate


class TemporalPositionFilter:
    """Per-object exponential position filter with stale-track eviction."""

    def __init__(self, alpha: float = 0.45, max_age_s: float = 0.75) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        self.alpha = float(alpha)
        self.max_age_s = float(max_age_s)
        self._tracks: dict[str, tuple[float, np.ndarray]] = {}

    def update(self, estimate: PoseEstimate, timestamp_s: float) -> PoseEstimate:
        self.expire(timestamp_s)
        measurement = np.asarray(estimate.position, dtype=np.float64)
        previous = self._tracks.get(estimate.object_id)
        if previous is None:
            filtered = measurement
        else:
            filtered = self.alpha * measurement + (1.0 - self.alpha) * previous[1]
        self._tracks[estimate.object_id] = (float(timestamp_s), filtered)
        return replace(estimate, position=tuple(float(value) for value in filtered))

    def expire(self, timestamp_s: float) -> None:
        stale = [
            object_id
            for object_id, (last_seen, _) in self._tracks.items()
            if timestamp_s - last_seen > self.max_age_s
        ]
        for object_id in stale:
            del self._tracks[object_id]

    @property
    def track_count(self) -> int:
        return len(self._tracks)
