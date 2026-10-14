"""Validate a received upstream reference for simulation-only r2 compensation.

Float64MultiArray has no source stamp. ros_ns is its real receipt time, never
an invented monotonic stamp or a timer refresh of a cached reference.
"""
from math import isfinite


class IdealSteeringCommand:
    joint_name = 'r2_steer_joint'
    index = 3

    def __init__(self, lower, upper, sim_timeout=.20, wall_timeout=.50,
                 feedback_max_age=.25, future_skew=.05, startup_tolerance=.03):
        self.lower, self.upper = float(lower), float(upper)
        if (not all(isfinite(x) for x in (lower, upper, sim_timeout, wall_timeout,
                feedback_max_age, future_skew, startup_tolerance)) or lower >= upper or
                min(sim_timeout, wall_timeout, feedback_max_age, startup_tolerance) <= 0 or
                future_skew < 0):
            raise ValueError('finite joint limits and positive watchdog bounds required')
        self.sim_timeout, self.wall_timeout = sim_timeout, wall_timeout
        self.feedback_max_age, self.future_skew = feedback_max_age, future_skew
        self.startup_tolerance = startup_tolerance
        self.last_stamp_ns = None
        self.last_target = None
        self.last_wall = None
        self.active = False
        self.reason = 'waiting for fresh upstream reference'
        self.last_ros_now = None
        self.last_wall_now = None

    def invalidate(self, reason):
        self.active = False
        self.reason = reason
        return None

    def _clock(self, ros_ns, wall_now):
        if (type(ros_ns) is not int or ros_ns <= 0 or
                not isinstance(wall_now, (float, int)) or type(wall_now) is bool or
                not isfinite(wall_now)):
            self.invalidate('invalid or zero clock')
            return False
        backward = ((self.last_ros_now is not None and ros_ns < self.last_ros_now) or
                    (self.last_wall_now is not None and wall_now < self.last_wall_now))
        self.last_ros_now, self.last_wall_now = ros_ns, wall_now
        if backward:
            self.last_stamp_ns = self.last_target = self.last_wall = None
            self.invalidate('clock moved backwards; fresh continuous reference required')
            return False
        return True

    def current(self, ros_ns, wall_now):
        if not self._clock(ros_ns, wall_now) or not self.active:
            return False
        if ((ros_ns-self.last_stamp_ns)/1e9 > self.sim_timeout or
                wall_now-self.last_wall > self.wall_timeout):
            self.invalidate('upstream reference lease expired')
            return False
        return True

    def accept(self, values, *, ros_ns, wall_now, actual, feedback_stamp_ns):
        if not self._clock(ros_ns, wall_now):
            return None
        try:
            valid = (len(values) == 4 and all(isfinite(x) for x in values) and
                     isfinite(actual) and self.lower <= actual <= self.upper and
                     self.lower <= values[self.index] <= self.upper)
        except (TypeError, ValueError):
            valid = False
        if not valid:
            return self.invalidate('incomplete, nonfinite or out-of-limit reference/feedback')
        if type(feedback_stamp_ns) is not int:
            return self.invalidate('joint feedback stamp must be an integer')
        feedback_age = (ros_ns-feedback_stamp_ns)/1e9
        if feedback_stamp_ns <= 0 or not -self.future_skew <= feedback_age <= self.feedback_max_age:
            return self.invalidate('joint feedback is stale, future or zero')
        target = float(values[self.index])
        if self.last_stamp_ns is not None and ros_ns == self.last_stamp_ns:
            if target != self.last_target:
                return self.invalidate('changed reference without clock advance')
            # No publication and no wall-lease extension for an exact duplicate.
            self.current(ros_ns, wall_now)
            return None
        if self.active:
            self.current(ros_ns, wall_now)
        if not self.active and abs(target-actual) > self.startup_tolerance:
            return self.invalidate('initial/reclaimed reference is discontinuous with actual joint')
        self.last_stamp_ns, self.last_target, self.last_wall = ros_ns, target, wall_now
        self.active, self.reason = True, 'fresh idealized r2 reference'
        return {'joint_name':self.joint_name, 'position_rad':target,
                'stamp_ns':ros_ns, 'simulation_only':True, 'idealized':True}
