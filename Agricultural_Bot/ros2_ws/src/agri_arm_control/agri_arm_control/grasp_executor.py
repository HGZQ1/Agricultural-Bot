"""Verified steps adapted from the copied collaborator's grasp sequence."""

from .controller import MotionError
from .contact_closure import ContactCloser
from .geometry import basket_poses, grasp_candidates


class GraspExecutor:
    """Preserve the original sequence, but require every result and feedback."""

    def __init__(self, controller, params, to_planning, scene, phase, diagnostic=None,
                 contact_diagnostic=None):
        self.arm, self.p = controller, params
        self.to_planning, self.scene, self.phase = to_planning, scene, phase
        self.diagnostic = diagnostic
        self.contact_diagnostic = contact_diagnostic
        self.candidates_checked = []

    def _record_ik(self, entry, stage, solution):
        entry[stage] = {'accepted': solution is not None,
                        **getattr(self.arm, 'last_ik_status', {})}
        if self.diagnostic is not None:
            self.diagnostic(self.candidates_checked)

    def _phase(self, name):
        self.phase(name)
        self.arm._check()

    def execute(self, surface, side):
        self._phase('OPENING')
        self.arm.gripper(self.p['gripper_open_m'])
        self._phase('SELECTING_COLLISION_TARGET')
        self.scene('select')
        selected = None
        self._phase('CHECKING_IK')
        for index, candidate in enumerate(grasp_candidates(
                surface, side, self.p['pre_offset_m'], self.p['retreat_offset_m'],
                self.p['surface_centre_offset_m'])):
            if index >= self.p['maximum_ik_candidates']:
                break
            pre = self.to_planning(candidate['pre'], candidate['orientation'])
            centre = self.to_planning(candidate['centre'], candidate['orientation'])
            retreat = self.to_planning(candidate['retreat'], candidate['orientation'])
            entry = {'index': index, 'task': candidate,
                     'planning': {'pre': pre, 'centre': centre, 'retreat': retreat}}
            self.candidates_checked.append(entry)
            pre_ik = self.arm.ik(*pre)
            self._record_ik(entry, 'pre_ik', pre_ik)
            if pre_ik is None:
                continue
            centre_ik = self.arm.ik(*centre, seed=pre_ik)
            self._record_ik(entry, 'centre_ik', centre_ik)
            if centre_ik is None:
                continue
            retreat_ik = self.arm.ik(*retreat, seed=centre_ik)
            self._record_ik(entry, 'retreat_ik', retreat_ik)
            if retreat_ik is None:
                continue
            values = dict(zip(pre_ik.joint_state.name, pre_ik.joint_state.position))
            try:
                planned = self.arm.plan_joints([values[name] for name in self.arm.joints])
            except MotionError as exc:
                entry['plan_error'] = str(exc)
                if self.diagnostic is not None:
                    self.diagnostic(self.candidates_checked)
                self.arm._check()  # cancellation/parking loss is never an IK retry
                continue
            selected = (pre, centre, retreat, pre_ik, planned)
            break
        if selected is None:
            raise MotionError('no collision-aware pre/grasp/retreat IK candidate')
        pre, centre, retreat, pre_ik, planned = selected
        self._phase('PREGRASP')
        self.arm.move_pose(*pre, solution=pre_ik, planned=planned)
        self._phase('APPROACHING')
        self.arm.cartesian(*centre)
        self._phase('CLOSING')
        ContactCloser(self.arm, self.p, self.scene, self.contact_diagnostic).close()
        self._phase('ATTACHING')
        attached = self.scene('attach')
        if attached.state != 'ATTACHED':
            raise MotionError(f'fruit attachment unconfirmed: {attached.state}')
        self._phase('RETREATING')
        self.arm.cartesian(*retreat)
        drop, above = basket_poses(self.p['basket_drop_xyz_m'], self.p['basket_clearance_m'])
        # TCP +Z points down; task-frame +Z is vertical, unlike CAD base_link.
        drop_pose = self.to_planning(drop, self.p['basket_quaternion_xyzw'])
        above_pose = self.to_planning(above, self.p['basket_quaternion_xyzw'])
        self._phase('ABOVE_BASKET')
        self.arm.move_pose(*above_pose)
        self._phase('LOWERING')
        self.arm.cartesian(*drop_pose)
        self._phase('OPENING_OVER_BASKET')
        self.arm.gripper(self.p['gripper_open_m'])
        self._phase('RELEASING')
        released = self.scene('release')
        if released.state not in {'RELEASED', 'IN_BASKET'}:
            raise MotionError(f'fruit release unconfirmed: {released.state}')
        self._phase('LIFTING')
        self.arm.cartesian(*above_pose)
        self._phase('VERIFYING_BASKET')
        self.scene('wait_in_basket')
        self._phase('CLEARING_COLLISION_TARGET')
        self.scene('clear')
        self._phase('RETRACTING')
        self.arm.move_joints(self.p['stow_joints'])
        if not self.arm.stowed():
            raise MotionError('measured six-joint stow acknowledgement unavailable')
        return True
