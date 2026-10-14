"""Pure feedback and identity checks, separate from controller submission."""

import math
import threading


def fresh(stamp, now, age):
    return stamp > 0 and -50_000_000 <= now-stamp <= age*1e9


def clamp_gripper_measurement(value, maximum, epsilon=1e-6):
    """Remove numerical limit noise only; never turn a hold into a new grip."""
    if not math.isfinite(value) or value < -epsilon or value > maximum+epsilon:
        raise ValueError('measured finger position exceeds initialization hold bound')
    return max(0., min(maximum, value))


class JointEvidence:
    """Require measured positions and velocities continuously at the target."""

    def __init__(self, max_age=0.5):
        self.max_age = max_age
        self.values = {}
        self.stamp = 0
        self.wall = None
        self._settled = {}
        self.last_update = {}
        self._lock = threading.RLock()

    def update(self, names, positions, velocities, stamp, now, wall):
        with self._lock:
            previous_update = self.last_update
            reasons = []
            if len(names) != len(positions) or len(names) != len(velocities):
                reasons.append('incomplete_joint_arrays')
            if len(set(names)) != len(names):
                reasons.append('duplicate_joint_names')
            if not fresh(stamp, now, self.max_age):
                reasons.append('stale_zero_or_future_stamp')
            if stamp <= self.stamp:
                reasons.append('nonincreasing_stamp')
            if not all(math.isfinite(v) for v in (*positions, *velocities)):
                reasons.append('nonfinite_joint_measurement')
            self.last_update = {'accepted': not reasons, 'reasons': reasons,
                'stamp_ns': stamp, 'observed_clock_ns': now, 'wall': wall,
                'joint_count': len(names)}
            # An identical sample can be delivered twice between clock ticks.
            # It is not new evidence: retain the original receipt/stamp and
            # settlement, with no lease renewal. A changed measurement at the
            # same time, reordered older sample, invalid input or clock reversal
            # still withdraws evidence immediately below.
            exact_duplicate = (
                reasons == ['nonincreasing_stamp'] and stamp == self.stamp and
                bool(self.values) and self.wall is not None and
                0 <= wall-self.wall <= self.max_age and now >= self.stamp and
                now >= previous_update.get('observed_clock_ns', now) and
                wall >= previous_update.get('wall', wall) and
                dict(zip(names, zip(positions, velocities))) == self.values)
            if exact_duplicate:
                self.last_update['ignored_exact_duplicate'] = True
                return False
            if reasons:
                self.values = {}
                self._settled.clear()
                return False
            if self.wall is None or wall-self.wall > self.max_age:
                self._settled.clear()
            self.values = dict(zip(names, zip(positions, velocities)))
            self.stamp, self.wall = stamp, wall
            return True

    def pose_report(self, names, positions, now, wall, position_tolerance=0.03,
                    velocity_tolerance=0.03, settle=0.25):
        """Read evidence atomically without warming or clearing settlement."""
        key = (tuple(names), tuple(positions), position_tolerance, velocity_tolerance)
        with self._lock:
            complete = all(name in self.values for name in names)
            wall_age = None if self.wall is None else wall-self.wall
            ros_age = (now-self.stamp)/1e9
            wall_fresh = wall_age is not None and wall_age <= self.max_age
            ros_fresh = fresh(self.stamp, now, self.max_age)
            measured = {name: {'position': self.values[name][0],
                              'velocity': self.values[name][1],
                              'target': target,
                              'position_error': self.values[name][0]-target}
                        for name, target in zip(names, positions) if name in self.values}
            position_ok = complete and len(names) == len(positions) and all(
                abs(value['position_error']) <= position_tolerance for value in measured.values())
            velocity_ok = complete and all(
                abs(value['velocity']) <= velocity_tolerance for value in measured.values())
            since = self._settled.get(key)
            elapsed = None if since is None else max(0., wall-since)
            reasons = []
            if not complete: reasons.append('incomplete_joint_feedback')
            if len(names) != len(positions): reasons.append('target_size_mismatch')
            if not wall_fresh: reasons.append('wall_feedback_stale')
            if not ros_fresh: reasons.append('ros_feedback_stale_zero_or_future')
            if complete and not position_ok: reasons.append('position_outside_stow_tolerance')
            if complete and not velocity_ok: reasons.append('velocity_above_stopped_tolerance')
            if elapsed is None or elapsed < settle: reasons.append('settlement_not_confirmed')
            update = dict(self.last_update)
            update['reasons'] = list(update.get('reasons', []))
            if 'wall' in update:
                update['wall_age_s'] = wall-update.pop('wall')
            return {'passed': not reasons, 'reasons': reasons,
                'stamp_ns': self.stamp, 'observed_clock_ns': now,
                'wall_age_s': wall_age, 'ros_age_s': ros_age,
                'maximum_age_s': self.max_age, 'fresh': wall_fresh and ros_fresh,
                'complete': complete, 'position_tolerance_rad': position_tolerance,
                'velocity_tolerance_rps': velocity_tolerance,
                'settlement_required_wall_s': settle, 'settlement_elapsed_wall_s': elapsed,
                'joints': measured, 'last_update': update}

    def available(self, names, now, wall):
        with self._lock:
            return (self.wall is not None and wall-self.wall <= self.max_age and
                    fresh(self.stamp, now, self.max_age) and
                    all(name in self.values for name in names))

    def at(self, names, positions, now, wall, position_tolerance=0.03,
           velocity_tolerance=0.03, settle=0.25):
        key = (tuple(names), tuple(positions), position_tolerance, velocity_tolerance)
        with self._lock:
            if (not self.available(names, now, wall) or len(names) != len(positions) or
                    any(abs(self.values[name][0]-value) > position_tolerance or
                        abs(self.values[name][1]) > velocity_tolerance
                        for name, value in zip(names, positions))):
                self._settled.pop(key, None)
                return False
            since = self._settled.setdefault(key, wall)
            return wall-since >= settle

    def snapshot(self, now, wall):
        with self._lock:
            if not self.available(tuple(self.values), now, wall):
                raise ValueError('fresh complete joint feedback unavailable')
            return {name: values[0] for name, values in self.values.items()}

    def stationary(self, names, now, wall, velocity_tolerance=0.03, settle=0.25):
        """Fresh complete feedback must remain stopped, independent of pose."""
        key = ('stationary', tuple(names), velocity_tolerance)
        with self._lock:
            if (not self.available(names, now, wall) or
                    any(abs(self.values[name][1]) > velocity_tolerance for name in names)):
                self._settled.pop(key, None)
                return False
            since = self._settled.setdefault(key, wall)
            return wall-since >= settle


class ControllerGoalEvidence:
    """Two real controller status snapshots prevent false terminal on restart."""

    def __init__(self):
        self.states = {}
        self._lock = threading.RLock()

    def update(self, controller, goals):
        with self._lock:
            self.states[controller] = list(goals)

    def ready(self, owned_goal_ids=(), terminal_goal_ids=()):
        with self._lock:
            return (set(self.states) == {'arm', 'gripper'} and
                    all(status in {4, 5, 6} or
                        (status in {1, 2, 3} and identifier in owned_goal_ids) or
                        identifier in terminal_goal_ids
                        for goals in self.states.values() for identifier, status in goals))

    def terminal(self, terminal_goal_ids=()):
        with self._lock:
            return (set(self.states) == {'arm', 'gripper'} and
                    all(status in {4, 5, 6} or identifier in terminal_goal_ids
                        for goals in self.states.values() for identifier, status in goals))

    def idle_for_initialization(self, terminal_goal_ids=()):
        """Missing startup snapshots need real cancel/hold, never imply ready."""
        with self._lock:
            return all(status in {4, 5, 6} or identifier in terminal_goal_ids
                       for goals in self.states.values() for identifier, status in goals)

    def snapshot(self):
        with self._lock:
            return {label: [{'goal_id': identifier.hex(), 'status': status}
                            for identifier, status in goals]
                    for label, goals in self.states.items()}

    def terminal_identifiers(self):
        with self._lock:
            return {identifier for goals in self.states.values()
                    for identifier, status in goals if status in {4, 5, 6}}


def station_matches(route, station, request, now_ns, received_wall, wall,
                    maximum_age=0.5, accepting=False):
    """A STARTED acknowledgement binds a worker to the locked work nonce."""
    if not isinstance(route, dict) or not isinstance(station, dict):
        return False
    try:
        if (received_wall is None or wall-received_wall > maximum_age or
                not fresh(int(route.get('status_stamp_ns', 0)), now_ns, maximum_age) or
                not route.get('require_harvest_handshake') or
                route.get('state') != 'AT_STATION' or
                route.get('base_lock_observed') is not True or
                not station.get('active') or
                request.work_side not in {'left', 'right'} or
                route.get('run_id') != request.run_id or
                route.get('work_id') != request.work_id or
                route.get('current', {}).get('waypoint_id') != request.waypoint_id or
                any(station.get(key) != getattr(request, key) for key in
                    ('run_id', 'work_id', 'waypoint_id', 'plant_id', 'work_side')) or
                int(station.get('station_started_ns', 0)) <= 0):
            return False
        # Coordinator publishes STARTED, waits for acknowledgement, then sends.
        return (route.get('work_phase') == 'WORKING' and
                route.get('work_terminal_confirmed') is False)
    except (TypeError, ValueError, AttributeError):
        return False
