"""Small successful joint steps stop on independently verified fruit contact."""

import math
import time

from .controller import MotionError


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
            raise MotionError(f'invalid physical grip contact query: {reply.state}')
        return reply

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
            if self.diagnostic is not None:
                self.diagnostic({'step': index, 'state': reply.state,
                    'actual_positions_m': positions, 'wall_elapsed_s': time.monotonic()-start})
            if reply.state in {'CONTACT_CONFIRMED', 'CONTACT_SETTLING'}:
                # Replace any small-step terminal holding error with measured
                # current positions. Its real successful result is still required.
                return self._hold_confirmed_contacts(positions, deadline)
            if index >= self.p['maximum_gripper_contact_steps']:
                break
            goals = next_finger_goals(positions, CONTACTS[reply.state],
                                     self.p['gripper_contact_step_m'], self.p['gripper_max_m'])
            # Freeze a contacted finger at its actual value while the other
            # continues closing. Both real result and stopped feedback must pass.
            self.arm.gripper_positions(goals, self.p['gripper_contact_step_s'],
                                       deadline_wall=deadline)
        raise MotionError('contact-guided closure step budget exhausted without double contact')
