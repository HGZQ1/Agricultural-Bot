"""Vertical observation geometry and fail-stop scan scheduling regressions.

The scheduling fixtures never construct a ROS node or submit a trajectory.
"""

from types import SimpleNamespace as NS

import pytest

from agri_arm_control.controller import MotionError
from agri_arm_control.geometry import rotate
from agri_arm_control.harvest_worker import HarvestArmWorker
from agri_arm_control.observation_scan import (
    camera_pose, descending_heights, tcp_pose_for_camera,
)


def test_scan_includes_bottom_without_overshoot_for_short_last_interval():
    heights = descending_heights(1.6, .93, .1)
    assert heights == pytest.approx([1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1., .93])
    assert heights[-1] == .93
    assert all(first > second >= .93 for first, second in zip(heights, heights[1:]))


def test_exact_level_budget_and_one_height_are_valid():
    assert len(descending_heights(31., 0., 1.)) == 32
    assert descending_heights(.9, .9, .1) == [.9]
    with pytest.raises(ValueError, match='32'):
        descending_heights(32., 0., 1.)


@pytest.mark.parametrize('arguments', [
    (float('nan'), .9, .1), (1.6, float('inf'), .1),
    (1.6, .9, float('inf')), (.9, 1.6, .1), (1.6, .9, 0.), (1.6, .9, -.1),
])
def test_invalid_height_plan_cannot_generate_a_scan(arguments):
    with pytest.raises(ValueError):
        descending_heights(*arguments)


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_camera_preserves_horizontal_view_while_height_changes(side, sign):
    high, orientation = camera_pose(side, -.18, .3, 1.6)
    low, low_orientation = camera_pose(side, -.18, .3, .9)
    assert high[:2] == low[:2] == (-.18, sign*.3)
    assert orientation == low_orientation
    assert rotate(orientation, (0., 0., 1.)) == pytest.approx((0., sign, 0.))
    assert rotate(orientation, (0., 1., 0.)) == pytest.approx((0., 0., -1.))


def test_tcp_offset_is_rotated_by_camera_pose_before_translation():
    optical = camera_pose('left', -.18, .3, 1.6)
    tcp, orientation = tcp_pose_for_camera(*optical, (0., .084, .128), (0., 0., 0., 1.))
    assert tcp == pytest.approx((-.18, .428, 1.516))
    assert rotate(orientation, (0., 0., 1.)) == pytest.approx((0., 1., 0.))
    lower, _ = tcp_pose_for_camera(*camera_pose('left', -.18, .3, 1.5),
                                   (0., .084, .128), (0., 0., 0., 1.))
    assert lower[:2] == pytest.approx(tcp[:2])
    assert lower[2] == pytest.approx(tcp[2]-.1)


def scan_fixture(observations):
    calls = []
    solution = NS(joint_state=NS(name=['joint'], position=[.1]))
    arm = NS(joints=['joint'], terminal_confirmed=True, last_ik_status={},
             _check=lambda: None, ik=lambda *_args: solution,
             plan_joints=lambda _positions: object(),
             move_pose=lambda *args, **kwargs: calls.append(('entry', args[0][2])),
             measured_arm_positions=lambda: [.123],
             move_joints=lambda positions: calls.append(('saved-entry', positions)),
             cartesian=lambda *args: calls.append(('descending', args[0][2])))
    relative = NS(transform=NS(translation=NS(x=0., y=0., z=0.),
                               rotation=NS(x=0., y=0., z=0., w=1.)))
    responses = iter(observations)
    worker = NS(
        p={'scan_mode': 'vertical', 'scan_camera_frame': 'camera_optical_frame',
           'tip_link': 'grasp_tcp', 'task_frame': 'base_footprint',
           'scan_top_height_m': 1.6, 'scan_bottom_height_m': 1.4,
           'scan_height_step_m': .1, 'scan_camera_x_m': -.18,
           'scan_camera_lateral_m': .3, 'scan_dwell_s': 2.},
        arm=arm, tf=NS(lookup_transform=lambda *_args: relative),
        _set_phase=lambda phase: calls.append(('phase', phase)),
        _to_planning=lambda *pose: pose,
        get_logger=lambda: NS(info=lambda _message: None),
        _wait_scan_target=lambda *_args: next(responses),
    )
    return worker, calls, solution


def test_first_acceptable_red_target_ends_scan_without_more_motion():
    selected = (NS(target_id='ripe-one'), NS())
    worker, calls, _ = scan_fixture([selected])
    assert HarvestArmWorker._select_after_scan(worker, NS(work_side='left')) is selected
    assert [call[0] for call in calls] == ['phase', 'entry']
    assert worker._scan_diagnostics['selected_target_id'] == 'ripe-one'
    assert len(worker._scan_diagnostics['levels']) == 1


def test_scan_descends_only_after_current_level_has_no_target():
    selected = (NS(target_id='ripe-low'), NS())
    worker, calls, _ = scan_fixture([None, selected])
    assert HarvestArmWorker._select_after_scan(worker, NS(work_side='left')) is selected
    assert [call[0] for call in calls] == ['phase', 'entry', 'descending']
    assert calls[-1][1] == pytest.approx(1.5)


def test_unreachable_unexecuted_entry_can_be_skipped():
    selected = (NS(target_id='ripe-low'), NS())
    worker, calls, solution = scan_fixture([selected])
    answers = iter([None, solution])
    worker.arm.ik = lambda *_args: next(answers)
    assert HarvestArmWorker._select_after_scan(worker, NS(work_side='left')) is selected
    assert [call[0] for call in calls] == ['phase', 'entry']
    assert calls[-1][1] == pytest.approx(1.5)
    assert worker._scan_diagnostics['levels'][0]['state'] == 'SKIPPED_UNREACHABLE_ENTRY'


def test_entry_execution_failure_cannot_be_mistaken_for_skippable_plan_failure():
    worker, calls, _ = scan_fixture([])

    def fail_execution(*_args, **_kwargs):
        calls.append(('execution-failed', 1.6))
        # The text deliberately matches a skippable planning rejection. Its
        # origin after execution begins must still prevent any next level.
        raise MotionError('joint plan failed: execution was already submitted')

    worker.arm.move_pose = fail_execution
    with pytest.raises(MotionError, match='execution was already submitted'):
        HarvestArmWorker._select_after_scan(worker, NS(work_side='left'))
    assert [call[0] for call in calls] == ['phase', 'execution-failed']
    assert len(worker._scan_diagnostics['levels']) == 1


def test_partial_cartesian_failure_prevents_trying_another_height():
    worker, calls, _ = scan_fixture([None])

    def partial_execution(*args):
        calls.append(('partial-cartesian', args[0][2]))
        raise MotionError('Cartesian path incomplete: fraction=0.5000')

    worker.arm.cartesian = partial_execution
    with pytest.raises(MotionError, match='Cartesian path incomplete'):
        HarvestArmWorker._select_after_scan(worker, NS(work_side='left'))
    assert [call[0] for call in calls] == ['phase', 'entry', 'partial-cartesian']
    assert len(worker._scan_diagnostics['levels']) == 2


def test_observation_from_previous_height_is_not_reused(monkeypatch):
    import threading
    import agri_arm_control.harvest_worker as module

    now_ns = 2_000_000_000
    stale_stamp = NS(sec=1, nanosec=900_000_000)
    validated = []
    wall = [0.]
    array = NS(observation_valid=True, depth_stamp=stale_stamp,
               header=NS(stamp=stale_stamp), targets=[NS(target_id='old-height')])
    worker = NS(
        p={'target_max_age_s': .5}, arm=NS(_check=lambda: None),
        _mutex=threading.RLock(), _targets=array, _targets_wall=0.,
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=now_ns)),
        _validate_target=lambda *args: validated.append(args),
    )
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.time, 'sleep', lambda seconds: wall.__setitem__(0, wall[0]+seconds))
    assert HarvestArmWorker._wait_scan_target(worker, NS(work_side='left'), .2) is None
    assert validated == []


def test_whole_plant_scan_only_completes_after_observing_bottom():
    worker, calls, _ = scan_fixture([None, None, None])
    request = NS(work_side='left', harvest_until_clear=True)
    assert HarvestArmWorker._select_after_scan(worker, request, allow_empty=True) is None
    assert worker._scan_height_m == pytest.approx(1.4)
    assert worker._scan_diagnostics['bottom_observed']
    assert worker._scan_diagnostics['scan_completed']
    assert [entry['height_m'] for entry in worker._scan_diagnostics['levels']] == pytest.approx(
        [1.6, 1.5, 1.4])


def test_return_from_basket_resumes_at_saved_height_instead_of_top():
    worker, calls, _ = scan_fixture([None, None])
    worker._scan_diagnostics = {'levels': [], 'bottom_observed': False}
    request = NS(work_side='left', harvest_until_clear=True)
    assert HarvestArmWorker._select_after_scan(
        worker, request, start_height=1.5, allow_empty=True) is None
    assert ('phase', 'RETURNING_TO_SCAN') in calls
    assert [entry['height_m'] for entry in worker._scan_diagnostics['levels']] == pytest.approx(
        [1.5, 1.4])
    assert worker._scan_diagnostics['scan_completed']


def test_unreachable_bottom_cannot_be_reported_as_empty_plant():
    worker, _, _ = scan_fixture([])
    worker.arm.ik = lambda *_args: None
    with pytest.raises(MotionError, match='no reachable'):
        HarvestArmWorker._select_after_scan(
            worker, NS(work_side='left', harvest_until_clear=True), allow_empty=True)
    assert not worker._scan_diagnostics['bottom_observed']
    assert not worker._scan_diagnostics['scan_completed']


@pytest.mark.parametrize('valid,renew,completed', [(True, True, True),
                                                  (True, False, False),
                                                  (False, True, False)])
def test_empty_scan_needs_fresh_valid_images_not_camera_silence(
        monkeypatch, valid, renew, completed):
    import threading
    import agri_arm_control.harvest_worker as module

    wall = [0.]
    array = NS(observation_valid=valid, depth_stamp=NS(sec=2, nanosec=0),
               header=NS(stamp=NS(sec=2, nanosec=0)), targets=[])
    worker = NS(p={'target_max_age_s': .06}, arm=NS(_check=lambda: None),
                _mutex=threading.RLock(), _targets=array, _targets_wall=0.,
                get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000+int(wall[0]*1e9))))

    def sleep(seconds):
        wall[0] += seconds
        if renew:
            stamp = NS(sec=2, nanosec=int(wall[0]*1e9))
            array.depth_stamp = stamp
            array.header.stamp = stamp
            worker._targets_wall = wall[0]

    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.time, 'sleep', sleep)
    request = NS(work_side='left', harvest_until_clear=True)
    if completed:
        assert HarvestArmWorker._wait_scan_target(worker, request, .2) is None
    else:
        with pytest.raises(MotionError, match='plant clear is unconfirmed'):
            HarvestArmWorker._wait_scan_target(worker, request, .2)
