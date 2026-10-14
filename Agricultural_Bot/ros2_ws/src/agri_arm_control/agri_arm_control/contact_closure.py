"""Small successful joint steps stop on independently verified fruit contact."""

import math
import time

from action_msgs.msg import GoalStatus

from .controller import ControllerToleranceError, JointSettlementTimeout, MotionError


CONTACTS = {'NO_CONTACT': (False, False), 'LEFT_CONTACT': (True, False),
            'RIGHT_CONTACT': (False, True), 'CONTACT_SETTLING': (True, True),
            'CONTACT_CONFIRMED': (True, True)}


def next_finger_goals(actual, contacts, step, maximum):
    if (len(actual) != 2 or len(contacts) != 2 or not 0 < step <= .001 or
            not 0 < maximum <= .035 or
            not all(math.isfinite(value) and 0 <= value <= maximum for value in actual)):
        raise MotionError('invalid measured finger step or travel limit')
    target = [value if touched else min(maximum, value+step)
              for value, touched in zip(actual, contacts)]
    if all(abs(goal-value) < 1e-9 for value, goal in zip(actual, target)):
        raise MotionError('finger travel exhausted before confirmed double contact')
    return target


class ContactCloser:
    """Neither JTC abort nor a single touching finger establishes a grasp."""

    def __init__(self, arm, params, scene, diagnostic=None):
        self.arm, self.p, self.scene, self.diagnostic = arm, params, scene, diagnostic
        self._recovery_count = 0
        self._last_recovery = {}

    def _record(self, step, reply, positions, start):
        if self.diagnostic is not None:
            self.diagnostic({'step': step, 'state': reply.state,
                'actual_positions_m': list(positions),
                'wall_elapsed_s': time.monotonic()-start,
                'native_contact_message': getattr(reply, 'message', ''),
                'recovery_count': self._recovery_count,
                'last_recovery': dict(self._last_recovery)})

    def _query(self, deadline):
        self.arm._check()
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise MotionError('contact confirmation wall deadline exceeded')
        reply = self.scene('check_grip', timeout_s=min(5., remaining))
        self.arm._check()
        if time.monotonic() >= deadline:
            raise MotionError('contact confirmation wall deadline exceeded')
        if not reply.success or reply.state not in CONTACTS:
            raise MotionError(f'invalid physical grip contact query: {reply.state}; '
                              f'{getattr(reply, "message", "")}')
        return reply

    def _recover_contact_stop(self, error, deadline, step, start):
        """A failed closing step authorizes a query, never a successful grasp."""
        if isinstance(error, ControllerToleranceError) and (
                error.controller != 'gripper' or
                error.status != GoalStatus.STATUS_ABORTED or error.error_code not in {-4, -5}):
            raise error
        self.arm._check()
        self._recovery_count += 1
        self._last_recovery = {'error': str(error), 'error_type': type(error).__name__,
            'error_code': getattr(error, 'error_code', None),
            'controller_status': getattr(error, 'status', None),
            'controller_terminal_confirmed': False, 'accepted': False}
        if not self.arm.cancel_and_confirm():
            raise MotionError('contact-stop recovery requires confirmed controller terminal result') from error
        self._last_recovery['controller_terminal_confirmed'] = True
        self.arm._check()
        stopped_deadline = min(deadline, time.monotonic()+self.p.get('feedback_timeout_s', 3.))
        positions = self.arm.wait_gripper_stopped(stopped_deadline)
        reply = self._query(deadline)
        self._last_recovery.update(contact_state=reply.state,
                                   native_contact_message=getattr(reply, 'message', ''))
        self._record(step, reply, positions, start)
        if reply.state == 'NO_CONTACT':
            raise error
        if reply.state in {'CONTACT_CONFIRMED', 'CONTACT_SETTLING'}:
            confirmed = self._hold_confirmed_contacts(positions, deadline)
            self._last_recovery['accepted'] = bool(confirmed)
            self._last_recovery['outcome'] = 'DOUBLE_CONTACT_CONFIRMED'
            self._record(step, reply, positions, start)
            return confirmed
        # A single genuine contact may explain this closing step's stop. Hold
        # both measured positions with a successful real goal, then let the
        # next query freeze only the touching finger. It is not a grasp.
        self.arm.gripper_positions(positions, self.p['gripper_contact_step_s'],
                                   deadline_wall=deadline)
        self._last_recovery['accepted'] = True
        self._last_recovery['outcome'] = 'SINGLE_CONTACT_HELD_NOT_GRASPED'
        self._record(step, reply, positions, start)
        return False

    def _hold_confirmed_contacts(self, positions, deadline):
        self.arm.gripper_positions(positions, self.p['gripper_contact_step_s'],
                                   deadline_wall=deadline)
        settlement_deadline = min(deadline, time.monotonic()+5.)
        while True:
            held = self._query(settlement_deadline)
            if held.state == 'CONTACT_CONFIRMED':
                return True
            if held.state != 'CONTACT_SETTLING':
                raise MotionError('double physical contact lost after measured finger hold')
            time.sleep(min(.05, max(0., settlement_deadline-time.monotonic())))

    def close(self):
        start = time.monotonic()
        deadline = start+self.p['gripper_contact_timeout_s']
        for index in range(self.p['maximum_gripper_contact_steps']+1):
            self.arm._check()
            if time.monotonic() >= deadline:
                raise MotionError('contact-guided finger closure wall deadline exceeded')
            reply = self._query(deadline)
            positions = self.arm.measured_gripper_positions()
            self._record(index, reply, positions, start)
            if reply.state in {'CONTACT_CONFIRMED', 'CONTACT_SETTLING'}:
                # Replace any small-step terminal holding error with measured
                # current positions. Its real successful result is still required.
                return self._hold_confirmed_contacts(positions, deadline)
            if index >= self.p['maximum_gripper_contact_steps']:
                break
            goals = next_finger_goals(positions, CONTACTS[reply.state],
                                     self.p['gripper_contact_step_m'], self.p['gripper_max_m'])
            # A 2 mm generic gripper tolerance can acknowledge a 1 mm step
            # without moving. Closing steps need a stricter goal and measured
            # settlement tolerance; current-position holds retain their default.
            tolerance = min(self.p.get('gripper_position_tolerance_m', .002),
                            self.p['gripper_contact_step_m']/4.)
            try:
                self.arm.gripper_positions(goals, self.p['gripper_contact_step_s'],
                                           deadline_wall=deadline,
                                           position_tolerance_m=tolerance)
            except (ControllerToleranceError, JointSettlementTimeout) as error:
                if self._recover_contact_stop(error, deadline, index, start):
                    return True
        raise MotionError('contact-guided closure step budget exhausted without double contact')
