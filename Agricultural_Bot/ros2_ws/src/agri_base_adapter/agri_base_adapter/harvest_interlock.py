"""Short-lived arm-state permission for the final mobile-base velocity gate."""

from math import isfinite


class HarvestInterlock:
    """Require fresh, advancing, explicitly stowed worker heartbeats.

    This validates the worker's reported state; it does not measure arm joints.
    A valid heartbeat only permits an explicit unlock request, never unlocks.
    """

    def __init__(self, *, receipt_timeout: float = 0.75,
                 maximum_age: float = 0.5, future_skew: float = 0.05):
        self.receipt_timeout = float(receipt_timeout)
        self.maximum_age = float(maximum_age)
        self.future_skew = float(future_skew)
        if (not all(isfinite(value) for value in (
                self.receipt_timeout, self.maximum_age, self.future_skew)) or
                self.receipt_timeout <= 0.0 or self.maximum_age <= 0.0 or
                self.future_skew < 0.0):
            raise ValueError('harvest heartbeat leases must be finite and '
                             'positive, with nonnegative future skew')
        self._last_ros_now = None
        self._last_wall_now = None
        self._last_stamp = None
        self._last_report = None
        self._eligible_stamp = None
        self._received_at = None
        self.reason = 'no harvest-worker heartbeat received'

    def _invalidate(self, reason: str) -> bool:
        self._eligible_stamp = None
        self._received_at = None
        self.reason = reason
        return False

    def _observe_time(self, ros_now: float, wall_now: float) -> bool:
        if not all(isfinite(value) for value in (ros_now, wall_now)):
            return self._invalidate('invalid local clock')
        jumped = ((self._last_ros_now is not None and
                   ros_now < self._last_ros_now) or
                  (self._last_wall_now is not None and
                   wall_now < self._last_wall_now))
        self._last_ros_now = ros_now
        self._last_wall_now = wall_now
        if jumped:
            self._last_stamp = None
            self._last_report = None
            return self._invalidate('local clock moved backwards')
        return True

    def observe(self, *, stamp: float, ros_now: float, wall_now: float,
                arm_stowed: bool, phase: str, identity: tuple = ()) -> bool:
        """Ignore exact duplicates without renewing; revoke changed/old reports.

        Identity includes the worker message frame and work context. A report
        with the same stamp but different state must never reuse a stowed lease.
        """
        if not self._observe_time(ros_now, wall_now):
            return False
        if not isfinite(stamp) or stamp <= 0.0:
            return self._invalidate('worker stamp is zero or invalid')
        age = ros_now - stamp
        if not -self.future_skew <= age <= self.maximum_age:
            return self._invalidate('worker stamp is stale or future')
        report = (arm_stowed, phase, tuple(identity))
        if self._last_stamp is not None and stamp == self._last_stamp:
            if report != self._last_report:
                self._last_report = report
                return self._invalidate('worker state changed without timestamp increase')
            # A repeated safe report is not a new observation. In particular,
            # /clock pause and DDS retransmission cannot extend the wall lease.
            return self.permits_unlock(ros_now=ros_now, wall_now=wall_now)
        if self._last_stamp is not None and stamp < self._last_stamp:
            return self._invalidate('worker stamp did not increase')
        # Track deployed feedback too: an older STOWED message cannot undo it.
        self._last_stamp = stamp
        self._last_report = report
        if arm_stowed is not True or phase not in {'IDLE', 'STOWED'}:
            return self._invalidate('worker does not report IDLE/STOWED arm')
        self._eligible_stamp = stamp
        self._received_at = wall_now
        self.reason = 'fresh IDLE/STOWED arm feedback'
        return True

    def permits_unlock(self, *, ros_now: float, wall_now: float) -> bool:
        """Check both ROS message age and the wall-clock receipt watchdog."""
        if not self._observe_time(ros_now, wall_now):
            return False
        if self._eligible_stamp is None or self._received_at is None:
            return False
        if wall_now - self._received_at > self.receipt_timeout:
            return self._invalidate('harvest-worker heartbeat receipt expired')
        if not -self.future_skew <= ros_now - self._eligible_stamp <= self.maximum_age:
            return self._invalidate('harvest-worker heartbeat ROS age expired')
        return True
