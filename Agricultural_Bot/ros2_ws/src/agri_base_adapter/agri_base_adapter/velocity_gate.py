"""Dependency-light arbitration and safety logic for planar velocity commands."""

from dataclasses import dataclass
from math import isfinite
from typing import Dict, Iterable, Mapping, Tuple


PlanarTwist = Tuple[float, float, float]
ZERO_TWIST: PlanarTwist = (0.0, 0.0, 0.0)


@dataclass
class SourceState:
    """Last accepted command and its local reception time."""

    twist: PlanarTwist = ZERO_TWIST
    updated_at: float | None = None


class VelocityGateCore:
    """Select the highest-priority fresh source and enforce a motion lock."""

    def __init__(
        self,
        priorities: Iterable[str],
        timeouts: Mapping[str, float],
        limits: PlanarTwist,
    ) -> None:
        self.priorities = tuple(priorities)
        if not self.priorities or len(set(self.priorities)) != len(self.priorities):
            raise ValueError('priorities must contain unique source names')
        if set(self.priorities) != set(timeouts):
            raise ValueError('timeouts must define every source exactly once')
        self.timeouts = {name: float(timeouts[name]) for name in self.priorities}
        if any(not isfinite(value) or value <= 0.0
               for value in self.timeouts.values()):
            raise ValueError('source timeouts must be positive and finite')
        self.limits = tuple(float(value) for value in limits)
        if (len(self.limits) != 3 or
                not all(isfinite(value) and value >= 0.0 for value in self.limits)):
            raise ValueError('limits must contain three finite nonnegative values')
        self.sources: Dict[str, SourceState] = {
            name: SourceState() for name in self.priorities
        }
        self.locked = False
        self._last_now: float | None = None

    def _observe_time(self, now: float) -> bool:
        """Track local time and invalidate state after a backwards jump."""
        if not isfinite(now):
            return False
        moved_backwards = self._last_now is not None and now < self._last_now
        if moved_backwards:
            self.clear()
        self._last_now = float(now)
        return not moved_backwards

    def update(self, source: str, twist: PlanarTwist, now: float) -> bool:
        """Store a finite command, returning False when it is rejected."""
        if source not in self.sources:
            raise KeyError(source)
        values = tuple(float(value) for value in twist)
        if (len(values) != 3 or not self._observe_time(float(now)) or
                not all(isfinite(value) for value in values)):
            return False
        limited = tuple(
            max(-limit, min(limit, value)) if limit > 0.0 else 0.0
            for value, limit in zip(values, self.limits)
        )
        self.sources[source] = SourceState(limited, float(now))
        return True

    def clear_source(self, source: str) -> None:
        if source not in self.sources:
            raise KeyError(source)
        self.sources[source] = SourceState()

    def set_locked(self, locked: bool) -> None:
        """Change lock state and invalidate buffered commands in both directions."""
        self.locked = bool(locked)
        self.clear()

    def clear(self) -> None:
        for name in self.sources:
            self.sources[name] = SourceState()

    def select(self, now: float) -> Tuple[PlanarTwist, str]:
        """Return selected command and source label for the current time."""
        if not self._observe_time(float(now)):
            return ZERO_TWIST, 'clock_reset'
        if self.locked:
            return ZERO_TWIST, 'locked'
        for name in self.priorities:
            state = self.sources[name]
            if state.updated_at is None:
                continue
            age = float(now) - state.updated_at
            if 0.0 <= age <= self.timeouts[name]:
                return state.twist, name
        return ZERO_TWIST, 'watchdog'


def timestamp_is_current(
    stamp: float,
    now: float,
    maximum_age: float,
    future_skew: float,
) -> bool:
    """Check a required source timestamp against the local ROS clock."""
    values = (stamp, now, maximum_age, future_skew)
    if not all(isfinite(float(value)) for value in values):
        return False
    if stamp <= 0.0 or maximum_age <= 0.0 or future_skew < 0.0:
        return False
    age = now - stamp
    return -future_skew <= age <= maximum_age
