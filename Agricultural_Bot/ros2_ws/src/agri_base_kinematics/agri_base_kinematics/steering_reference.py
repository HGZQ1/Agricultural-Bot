"""Bound steering position-reference motion without changing measured feedback."""

from math import copysign, isfinite, sqrt


class SteeringReferenceLimiter:
    """Follow bounded joint headings using velocity/acceleration in SIM time.

    The inverse kinematics chooses the nearest feasible equivalent heading.
    These coordinates must not be wrapped through a revolute joint hard stop.
    Stops hold the fresh measured headings; missing feedback clears history.
    """

    def __init__(self, limits, max_velocity=.5, max_acceleration=1.,
                 maximum_interval=.25):
        self.limits = [tuple(float(v) for v in pair) for pair in limits]
        if (not self.limits or any(len(pair) != 2 or
                not all(isfinite(v) for v in pair) or pair[0] >= pair[1]
                for pair in self.limits)):
            raise ValueError('steering limits must be finite increasing pairs')
        if not all(isfinite(v) and v > 0. for v in
                   (max_velocity, max_acceleration, maximum_interval)):
            raise ValueError('steering motion limits and interval must be positive')
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.maximum_interval = float(maximum_interval)
        self.reset()

    def reset(self):
        self.position = None
        self.velocity = [0.] * len(self.limits)
        self.stamp_ns = None

    def step(self, target, measured, stamp_ns, enabled=True, hold=False):
        """Return references or None when no safe feedback/time is available.

        A normal target reversal decelerates before reversing and can pass a
        newly moved target. Position references remain inside the hard limits.
        Holding, feedback loss and clock discontinuities reset planned motion;
        those safety transitions intentionally bypass reference continuity.
        """
        if not enabled:
            self.reset()
            return None
        target, measured = list(target), list(measured)
        if (len(target) != len(self.limits) or len(measured) != len(self.limits) or
                not all(isfinite(v) for v in target + measured) or
                any(not low <= q <= high for q, (low, high) in
                    zip(measured, self.limits)) or
                any(not low <= q <= high for q, (low, high) in
                    zip(target, self.limits))):
            self.reset()
            return None
        if hold or self.position is None:
            self.position = list(measured)
            self.velocity = [0.] * len(self.limits)
            self.stamp_ns = stamp_ns
            return list(self.position)
        dt = (stamp_ns-self.stamp_ns)/1e9
        if not 0. < dt <= self.maximum_interval:
            self.reset()
            return None
        self.stamp_ns = stamp_ns
        acceleration_step = self.max_acceleration*dt

        def braking_speed(distance):
            # Leave room for this discrete position step and subsequent
            # deceleration; the final small step is clipped only if feasible.
            return max(0., sqrt(2.*self.max_acceleration*distance +
                               acceleration_step**2)-acceleration_step)

        next_position, next_velocity = [], []
        for q, v, goal, (low, high) in zip(
                self.position, self.velocity, target, self.limits):
            error = goal-q
            desired = (0. if error == 0. else copysign(
                min(self.max_velocity, braking_speed(abs(error))), error))
            minimum = max(-self.max_velocity, v-acceleration_step,
                          -braking_speed(q-low))
            maximum = min(self.max_velocity, v+acceleration_step,
                          braking_speed(high-q))
            if minimum > maximum + 1e-12:
                self.reset()
                return None
            planned = max(minimum, min(maximum, desired))
            if planned*error > 0. and abs(planned*dt) > abs(error):
                arrival_velocity = error/dt
                if minimum <= arrival_velocity <= maximum:
                    planned = arrival_velocity
            next_position.append(max(low, min(high, q+planned*dt)))
            next_velocity.append(planned)
        self.position, self.velocity = next_position, next_velocity
        return list(self.position)
