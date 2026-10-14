"""Ideal work keeps real zero velocity and nonce ownership, without stow leases."""

import threading
from types import SimpleNamespace as NS

import pytest
from rclpy.action import GoalResponse

from agri_arm_control.harvest_worker import HarvestArmWorker


def worker_fixture():
    request = NS(run_id='run', waypoint_id='point', work_id='work', plant_id='plant',
                 work_side='left', scan_before_selection=True)
    node = NS(_ideal_simulation=True, _mutex=threading.RLock(), _busy=False,
        _fault=True, _healthy=False, _request=request,
        _work_context=('run', 'point', 'work'), _base_velocity=(0., 0., 0., 0.),
        _used_work_ids=set(), _cancel=threading.Event(), p={'scan_mode': 'vertical'},
        _station={**vars(request), 'active': True},
        _route={'require_harvest_handshake': True, 'state': 'AT_STATION',
                'run_id': 'run', 'work_id': 'work', 'current': {'waypoint_id': 'point'},
                'work_phase': 'WORKING', 'work_terminal_confirmed': False},
        arm=NS(stowed=lambda: pytest.fail('ideal start must not require a stow lease'),
               reset_navigation_stow=lambda: None))
    node._parked = lambda: HarvestArmWorker._parked(node)
    node._ideal_station_matches = lambda goal: HarvestArmWorker._ideal_station_matches(node, goal)
    return node, request


def test_ideal_permit_ignores_health_fault_and_route_lease_but_requires_saved_nonce():
    node, _ = worker_fixture()
    assert HarvestArmWorker._permit(node)
    node._work_context = ('run', 'point', 'different-work')
    assert not HarvestArmWorker._permit(node)


@pytest.mark.parametrize('velocity', [None, (.011, 0., 0., 0.), (0., 0., 0., .02),
                                    (float('nan'), 0., 0., 0.)])
def test_ideal_work_cannot_continue_without_actual_finite_stopped_base(velocity):
    node, _ = worker_fixture()
    node._base_velocity = velocity
    assert not HarvestArmWorker._permit(node)


def test_new_ideal_work_can_start_from_faulted_health_without_claiming_stow():
    node, request = worker_fixture()
    assert HarvestArmWorker._goal(node, request) == GoalResponse.ACCEPT
    assert node._busy and not node._fault and node._phase == 'DEPLOYING'
    assert request.work_id in node._used_work_ids
    assert HarvestArmWorker._goal(node, request) == GoalResponse.REJECT


def test_ideal_work_still_rejects_wrong_plant_and_nonce():
    node, request = worker_fixture()
    node._station['plant_id'] = 'other-plant'
    assert HarvestArmWorker._goal(node, request) == GoalResponse.REJECT
    node._station['plant_id'] = request.plant_id
    node._route['work_id'] = 'old-work'
    assert HarvestArmWorker._goal(node, request) == GoalResponse.REJECT


def test_ideal_mode_does_not_skip_visual_quality_checks_for_direct_target():
    node, request = worker_fixture()
    request.scan_before_selection = False
    request.target, request.header = NS(), NS()
    from agri_arm_control.controller import MotionError
    node._validate_target = lambda *_args: (_ for _ in ()).throw(MotionError('unripe or invalid target'))
    assert HarvestArmWorker._goal(node, request) == GoalResponse.REJECT
