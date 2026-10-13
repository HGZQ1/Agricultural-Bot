"""Pure freshness and stopped-motion checks for the harvest handshake."""

from __future__ import annotations

import math


def stamp_ns(stamp) -> int:
    """Convert a ROS time message without assuming its clock origin."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def fresh_stamp(stamp: int, now: int, maximum_age_s: float) -> bool:
    """Require positive time with a bounded age and future skew."""
    return stamp > 0 and -0.05 <= (now - stamp) / 1e9 <= maximum_age_s


class StoppedMotion:
    """Require fresh measured velocity throughout a continuous ROS window."""

    def __init__(self, settle_s=0.5, max_age_s=0.35,
                 linear_limit=0.01, angular_limit=0.02):
        self.settle_s = settle_s
        self.max_age_s = max_age_s
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.last_stamp_ns = 0
        self.received_wall_s = None
        self.since_ns = None
        self.linear_speed = None
        self.angular_speed = None

    def update(self, stamp: int, now_ns: int, wall_s: float,
               linear_xyz, angular_z: float) -> bool:
        """Reject stale, repeated, future, nonfinite and moving samples."""
        values = (*linear_xyz, angular_z)
        valid = (all(math.isfinite(value) for value in values) and
                 fresh_stamp(stamp, now_ns, self.max_age_s) and
                 stamp > self.last_stamp_ns)
        previous_stamp = self.last_stamp_ns
        if not valid:
            self.since_ns = None
            return False
        self.last_stamp_ns = stamp
        self.received_wall_s = wall_s
        self.linear_speed = math.sqrt(sum(value * value for value in linear_xyz))
        self.angular_speed = abs(angular_z)
        if (self.linear_speed >= self.linear_limit or
                self.angular_speed >= self.angular_limit):
            self.since_ns = None
            return False
        if (self.since_ns is None or
                (stamp - previous_stamp) / 1e9 > self.max_age_s):
            self.since_ns = stamp
        return True

    def ready(self, now_ns: int, wall_s: float, since_ns: int = 0) -> bool:
        """Revoke parking permission on paused clocks or lost odometry."""
        if (self.since_ns is None or self.received_wall_s is None or
                wall_s - self.received_wall_s > self.max_age_s or
                not fresh_stamp(self.last_stamp_ns, now_ns, self.max_age_s)):
            return False
        return self.last_stamp_ns - max(self.since_ns, since_ns) >= int(
            self.settle_s * 1e9)


class WorkerLease:
    """Fresh acknowledged arm state, carrying the current action identity."""

    def __init__(self, maximum_age_s=0.75):
        self.maximum_age_s = maximum_age_s
        self.message = None
        self.received_wall_s = None
        self.last_stamp_ns = 0

    def update(self, message, now_ns: int, wall_s: float) -> bool:
        """Out-of-order or invalid heartbeats revoke the state lease."""
        stamp = stamp_ns(message.header.stamp)
        if (not fresh_stamp(stamp, now_ns, self.maximum_age_s) or
                stamp <= self.last_stamp_ns):
            self.message = None
            return False
        self.message = message
        self.last_stamp_ns = stamp
        self.received_wall_s = wall_s
        return True

    def fresh(self, now_ns: int, wall_s: float) -> bool:
        """Require both ROS timestamp validity and wall-clock heartbeat health."""
        return bool(
            self.message is not None and self.received_wall_s is not None and
            wall_s - self.received_wall_s <= self.maximum_age_s and
            fresh_stamp(self.last_stamp_ns, now_ns, self.maximum_age_s))

    def stowed(self, now_ns: int, wall_s: float, context=None,
               after_ns: int = 0) -> bool:
        """Match the work nonce and a fresh explicitly stowed phase."""
        if not self.fresh(now_ns, wall_s) or self.last_stamp_ns < after_ns:
            return False
        message = self.message
        if not message.arm_stowed or message.phase not in {'IDLE', 'STOWED'}:
            return False
        if context is None:
            return not any((message.run_id, message.waypoint_id, message.work_id))
        return (message.run_id, message.waypoint_id, message.work_id) == context
