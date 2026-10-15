"""Return from the basket using the actual previously reached scan branch.

All motion clients are fixtures; these tests never construct a ROS node.
"""

from types import SimpleNamespace as NS

import pytest

from agri_arm_control.controller import MotionError
from agri_arm_control.harvest_worker import HarvestArmWorker
from test_observation_scan import scan_fixture


def saved_scan_fixture(observations):
    worker, calls, solution = scan_fixture(observations)
    actual = [.123456789]  # Deliberately differs from the IK target's .1 rad.
    worker._scan_joint_cache = {}
    worker.arm.measured_arm_positions = lambda: list(actual)
    worker.arm.move_joints = lambda positions: calls.append(('saved_joints', list(positions)))
    return worker, calls, solution, actual


def target(name):
    return NS(target_id=name), NS()


def initial_scan(worker, selected):
    request = NS(work_side='left', harvest_until_clear=True)
    assert HarvestArmWorker._select_after_scan(worker, request) is selected
    assert worker._scan_height_m == pytest.approx(1.6)
    return request


def prevent_new_ik_branch(worker):
    worker.arm.ik = lambda *_args, **_kwargs: pytest.fail(
        'same-height return must reuse the measured joint branch')
    worker.arm.plan_joints = lambda *_args: pytest.fail(
        'scan return must not retry a newly solved IK branch')


def test_successful_scan_saves_actual_joints_without_replacing_them_with_ik_target():
    selected = target('first-red-fruit')
    worker, _, solution, actual = saved_scan_fixture([selected])

    initial_scan(worker, selected)

    assert worker._scan_joint_cache[('left', round(1.6, 9))] == [.123456789]
    assert worker._scan_joint_cache[('left', round(1.6, 9))] != list(
        solution.joint_state.position)
    actual[0] = 1.234  # The basket movement cannot overwrite the saved pose.
    assert worker._scan_joint_cache[('left', round(1.6, 9))] == [.123456789]


def test_return_reuses_saved_actual_pose_and_finds_another_fruit_at_the_same_height():
    first, second = target('first-red-fruit'), target('second-red-fruit')
    worker, calls, _, actual = saved_scan_fixture([first, second])
    request = initial_scan(worker, first)
    actual[0] = 1.234  # A different current configuration above the basket.
    prevent_new_ik_branch(worker)
    calls.clear()

    assert HarvestArmWorker._select_after_scan(
        worker, request, start_height=1.6, allow_empty=True) is second

    assert [call for call in calls if call[0] != 'phase'] == [
        ('saved_joints', [.123456789])]
    assert worker._scan_height_m == pytest.approx(1.6)
    assert worker._scan_diagnostics['selected_target_id'] == 'second-red-fruit'
    assert not worker._scan_diagnostics['scan_completed']


def test_return_observes_saved_height_before_descending_and_finishes_only_at_bottom():
    first = target('first-red-fruit')
    worker, calls, _, _ = saved_scan_fixture([first, None, None, None])
    request = initial_scan(worker, first)
    prevent_new_ik_branch(worker)
    calls.clear()

    assert HarvestArmWorker._select_after_scan(
        worker, request, start_height=1.6, allow_empty=True) is None

    motion = [call for call in calls if call[0] != 'phase']
    assert motion[0] == ('saved_joints', [.123456789])
    assert [call[0] for call in motion] == ['saved_joints', 'descending', 'descending']
    assert [call[1] for call in motion[1:]] == pytest.approx([1.5, 1.4])
    assert worker._scan_height_m == pytest.approx(1.4)
    assert worker._scan_diagnostics['bottom_observed']
    assert worker._scan_diagnostics['scan_completed']


def test_saved_pose_execution_failure_is_never_treated_as_a_completed_empty_plant():
    first = target('first-red-fruit')
    worker, calls, _, _ = saved_scan_fixture([first])
    request = initial_scan(worker, first)
    prevent_new_ik_branch(worker)
    observations = []
    worker._wait_scan_target = lambda *_args: observations.append('observed')

    def failed_return(_positions):
        # Match the text used for an unexecuted fresh-entry planning rejection;
        # a failure of the saved-pose motion must still propagate immediately.
        calls.append(('saved-return-failed',))
        raise MotionError('joint plan failed: saved scan return failed')

    worker.arm.move_joints = failed_return
    calls.clear()
    with pytest.raises(MotionError, match='saved scan return failed'):
        HarvestArmWorker._select_after_scan(
            worker, request, start_height=1.6, allow_empty=True)

    assert [call for call in calls if call[0] != 'phase'] == [('saved-return-failed',)]
    assert not observations
    assert not worker._scan_diagnostics['bottom_observed']
    assert not worker._scan_diagnostics['scan_completed']
