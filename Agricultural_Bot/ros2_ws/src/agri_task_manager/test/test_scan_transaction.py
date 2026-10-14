"""Keep camera deployment inside one acknowledged, parked work transaction."""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from agri_harvest_interfaces.action import HarvestWork
from agri_task_manager.harvest_coordinator import HarvestCoordinator
from builtin_interfaces.msg import Time


def _coordinator():
    coordinator = object.__new__(HarvestCoordinator)
    context = ('run', 'waypoint', 'work')
    coordinator._station_context = lambda require_permission=True: context
    coordinator._station = {
        'station_started_ns': 1_000_000_000,
        'plant_id': 'tomato_20', 'work_side': 'left',
    }
    coordinator._route = {'work_phase': 'SCANNING', 'work_terminal_confirmed': True}
    coordinator._work_context = None
    coordinator._handled_contexts = set()
    coordinator._pending_goal = None
    coordinator._send_future = None
    coordinator._request_started = None
    coordinator._target_history = {}
    coordinator._selected_target_id = 'previous-observation'
    coordinator._clock = SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=2), nanoseconds=2_000_000_000))
    coordinator.get_clock = lambda: coordinator._clock
    params = {'worker_select_targets_after_scan': True, 'selection_frame': 'base_footprint',
              'goal_response_timeout_s': 3.0, 'scan_timeout_s': 20.0}
    coordinator.get_parameter = lambda name: SimpleNamespace(value=params[name])
    reports, sent = [], []
    coordinator._report = lambda phase, **details: reports.append((phase, details))
    coordinator._publish_status = lambda: None

    def select_without_scan():
        raise AssertionError('camera is still stowed; selection must happen in worker')

    def send(goal, **kwargs):
        sent.append(goal)
        return SimpleNamespace(add_done_callback=lambda callback: None)

    coordinator._select_target = select_without_scan
    coordinator._client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=send)
    return coordinator, reports, sent


def test_scan_request_waits_for_executor_nonce_ack_and_carries_no_old_target():
    coordinator, reports, sent = _coordinator()
    coordinator._tick()
    assert sent == []
    assert reports[0][0] == 'STARTED'
    assert reports[0][1]['scan_before_selection'] is True
    assert coordinator._pending_goal.scan_before_selection is True
    assert coordinator._pending_goal.target.target_id == ''
    assert coordinator._pending_goal.plant_id == 'tomato_20'
    assert coordinator._pending_goal.work_side == 'left'
    # Merely being AT_STATION is insufficient: the executor must acknowledge
    # the STARTED report before the worker can move the camera.
    coordinator._route.update(work_phase='WORKING', work_terminal_confirmed=False)
    coordinator._tick()
    assert len(sent) == 1
    assert (sent[0].run_id, sent[0].waypoint_id, sent[0].work_id) == (
        'run', 'waypoint', 'work')


def test_revoked_station_before_nonce_ack_never_starts_scan():
    coordinator, reports, sent = _coordinator()
    coordinator._tick()
    coordinator._station_context = lambda require_permission=True: None
    coordinator._tick()
    assert sent == []
    assert coordinator._pending_goal is None
    assert reports[-1][0] == 'FAILED'
    assert reports[-1][1]['action_accepted'] is False
    assert reports[-1][1]['terminal_confirmed'] is True


def test_worker_selected_target_is_bound_to_station_and_simulation_report():
    coordinator, reports, _ = _coordinator()
    coordinator._work_context = ('run', 'waypoint', 'work')
    result = HarvestWork.Result(success=True, arm_stowed=True,
                                target_id='detector-session:7', sim_fruit_verified=True)
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=result))
    coordinator._result(future)
    assert reports[-1][0] == 'SUCCESS'
    assert reports[-1][1]['sim_fruit_verified'] is True
    assert coordinator._target_history['detector-session:7'] == {('run', 'waypoint')}
    assert coordinator._selected_target_id == 'detector-session:7'


def test_sim_fruit_claim_cannot_override_failed_stow():
    coordinator, reports, _ = _coordinator()
    coordinator._work_context = ('run', 'waypoint', 'work')
    result = HarvestWork.Result(success=True, arm_stowed=False,
                                target_id='detector-session:7', sim_fruit_verified=True)
    coordinator._result(SimpleNamespace(result=lambda: SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=result)))
    assert reports[-1][0] == 'FAILED'
    assert reports[-1][1]['sim_fruit_verified'] is False
