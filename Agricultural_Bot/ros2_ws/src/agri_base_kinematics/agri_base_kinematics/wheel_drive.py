"""Limit normal drive changes after steering alignment, using ROS time."""

from math import isfinite


class WheelDriveLimiter:
    """Bound wheel acceleration without delaying a guarded zero output.

    One interpolation factor limits all wheels together, preserving the
    requested wheel-speed ratios when starting from rest. A stopped, disabled,
    or discontinuous-clock sample clears the output history immediately.
    """

    def __init__(self, wheel_count=4, max_acceleration=0.8, maximum_interval=0.25):
        if wheel_count <= 0:
            raise ValueError('wheel_count must be positive')
        if (not isfinite(max_acceleration) or max_acceleration <= 0.0 or
                not isfinite(maximum_interval) or maximum_interval <= 0.0):
            raise ValueError('wheel acceleration and maximum interval must be positive')
        self.wheel_count = wheel_count
        self.max_acceleration = float(max_acceleration)
        self.maximum_interval = float(maximum_interval)
        self.previous = [0.0] * wheel_count
        self.stamp_ns = None

    def step(self, target, stamp_ns, enabled=True, maximum_magnitudes=None):
        """Return physical rad/s commands at a ROS/SIM timestamp in ns.

        The caller supplies the original command and feedback freshness gates;
        this limiter neither retains a command nor extends its valid lifetime.
        All-zero targets always bypass the acceleration limit for locking and
        stopping. Nonzero normal changes, including reversals, remain limited.
        Optional current steering-speed caps take precedence over the ramp.
        """
        target = [float(value) for value in target]
        if len(target) != self.wheel_count or not all(isfinite(v) for v in target):
            raise ValueError('wheel targets must be complete and finite')
        caps = None
        if maximum_magnitudes is not None:
            caps = [float(value) for value in maximum_magnitudes]
            if (len(caps) != self.wheel_count or
                    not all(isfinite(v) and v >= 0.0 for v in caps)):
                raise ValueError('wheel speed caps must be complete, finite and nonnegative')
        dt = None if self.stamp_ns is None else (stamp_ns-self.stamp_ns)/1e9
        self.stamp_ns = stamp_ns
        if (not enabled or not any(target) or dt is None or
                not 0.0 < dt <= self.maximum_interval):
            self.previous = [0.0] * self.wheel_count
            return list(self.previous)
        difference = [value-previous for value, previous in zip(target, self.previous)]
        peak_change = max(abs(change) for change in difference)
        fraction = (1.0 if peak_change == 0.0 else
                    min(1.0, self.max_acceleration*dt/peak_change))
        self.previous = [previous+fraction*change
                         for previous, change in zip(self.previous, difference)]
        if caps is not None:
            # A deteriorating steering permit must not be delayed by a ramp.
            self.previous = [max(-cap, min(cap, value))
                             for value, cap in zip(self.previous, caps)]
        return list(self.previous)
