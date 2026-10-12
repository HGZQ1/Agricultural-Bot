"""Transit navigation stays separate from plant tasks and work handshakes."""

from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from agri_task_manager.field_route_executor import FieldRouteExecutor
from agri_task_manager.waypoint_schema import Pose2D, TransitWaypoint
from builtin_interfaces.msg import Time
from rclpy.task import Future


class _GoalHandle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancel = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        return self.cancel


class _Navigator:
    def __init__(self):
        self.goals = []
        self.acceptances = []
        self.handles = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        del feedback_callback
        future = Future()
        self.goals.append(goal)
        self.acceptances.append(future)
        self.handles.append(_GoalHandle())
        return future

    def accept(self, index):
        self.acceptances[index].set_result(self.handles[index])

    def finish(self, index, status=GoalStatus.STATUS_SUCCEEDED):
        self.handles[index].result.set_result(SimpleNamespace(status=status))


def _executor():
    executor = object.__new__(FieldRouteExecutor)
    waypoint = SimpleNamespace(
        waypoint_id='right_09', stop_id='right_stop_09',
        pose=Pose2D(7.3, -0.5, 0.0), goal_timeout_s=120.0,
        plant=SimpleNamespace(plant_id='tomato_39'), work_side='right',
        station_dwell_s=1.0, as_event=lambda: {'waypoint_id': 'right_09'})
    transits = (
        TransitWaypoint('headland_left', waypoint.waypoint_id,
                        Pose2D(7.9, 0.5, 0.0), 70.0),
        TransitWaypoint('headland_right', waypoint.waypoint_id,
                        Pose2D(8.4, -0.5, 0.0), 80.0),
    )
    executor._mission = SimpleNamespace(
        mission_id='side_stops', map_binding=SimpleNamespace(frame_id='map'),
        route_transits=transits, home=Pose2D(0.5, 0.0, 0.0),
        return_home=True)
    executor._waypoints = [waypoint]
    executor._index = 0
    executor._returning = False
    executor._last_completed_stop_id = None
    executor._active_transit = None
    executor._completed_transit_ids = set()
    executor._parking_goal_key = None
    executor._parking_retry_count = 0
    executor._parking_paused = False
    executor._parking_reused_pose = False
    executor._state = 'AT_STATION'
    executor._shutting_down = False
    executor._require_lock = True
    executor._require_station_ack = True
    executor._base_lock_observed = True
    executor._lock_pending = False
    executor._goal_generation = 0
    executor._goal_handle = None
    executor._goal_send_future = None
    executor._goal_result_future = None
    executor._goal_send_started_at = None
    executor._goal_started_at = None
    executor._after_cancel = None
    executor._cancel_started_at = None
    executor._cancel_request_generation = None
    executor._cancel_goal_terminal = False
    executor._cancel_lock_confirmed = False
    executor._paused_dwell_remaining = None
    executor._dwell_until = None
    executor._run_id = 'test'
    executor.events = []
    executor.work_stations = []
    executor.work_messages = []
    executor.gate_states = []
    executor._binding_error = lambda: None
    executor._publish_status = lambda: None
    executor._publish_point_tombstone = lambda reason: None
    executor._record = lambda event, extra=None: executor.events.append(
        (event, extra))
    executor.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    executor.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: Time()))
    executor._point_pub = SimpleNamespace(
        publish=executor.work_messages.append)

    def enter_station(point, reused_pose):
        FieldRouteExecutor._enter_station(executor, point, reused_pose)
        executor.work_stations.append(point.waypoint_id)

    executor._enter_station = enter_station

    def set_gate(locked, callback):
        executor._base_lock_observed = locked
        executor.gate_states.append(locked)
        callback()

    executor._unlock_then = lambda callback: set_gate(False, callback)
    executor._lock_then = lambda callback: set_gate(True, callback)
    executor._navigate = _Navigator()
    # These tests isolate transit ordering; actual post-lock TF verification
    # is exercised independently in test_post_lock_pose_gate.py.
    executor._begin_parking_check = executor._finish_parking_check
    return executor


def test_transits_execute_in_order_without_consuming_a_plant_or_work_ack():
    """Two intermediate goals must finish before the right-side plant goal."""
    executor = _executor()
    executor._advance_from_locked()
    assert executor._active_transit.transit_id == 'headland_left'
    assert executor._goal_timeout_s == 70.0
    assert executor._index == 0
    executor._navigate.accept(0)
    executor._navigate.finish(0)

    assert executor._active_transit.transit_id == 'headland_right'
    assert executor._goal_timeout_s == 80.0
    assert executor._index == 0
    assert executor.work_stations == []
    executor._navigate.accept(1)
    executor._navigate.finish(1)

    assert executor._active_transit is None
    assert executor._goal_timeout_s == 120.0
    assert executor._index == 0
    assert executor.work_stations == []
    executor._navigate.accept(2)
    executor._navigate.finish(2)

    assert executor.work_stations == ['right_09']
    assert executor._state == 'AT_STATION'
    assert executor._dwell_until is None
    assert len(executor.work_messages) == 1
    assert executor._completed_transit_ids == {
        'headland_left', 'headland_right'}
    assert executor.gate_states == [False, True, False, True, False, True]
    assert sum(event == 'at_harvest_point'
               for event, _ in executor.events) == 1


def test_pause_resumes_unfinished_transit_without_advancing_work_index():
    """Paused and canceled intermediate goals remain due on resume."""
    executor = _executor()
    executor._advance_from_locked()
    executor._navigate.accept(0)
    response = executor._pause_service(None, SimpleNamespace())
    assert response.success
    assert executor._state == 'CANCELING'
    assert executor._base_lock_observed is True
    executor._navigate.finish(0, GoalStatus.STATUS_CANCELED)
    assert executor._state == 'PAUSED'
    assert executor._completed_transit_ids == set()
    assert executor._index == 0

    response = executor._resume_service(None, SimpleNamespace())
    assert response.success
    assert len(executor._navigate.goals) == 2
    assert executor._active_transit.transit_id == 'headland_left'
    assert executor._navigate.goals[0].pose == (
        executor._navigate.goals[1].pose)
    assert executor.work_stations == []


def test_skip_cannot_bypass_required_side_change_transit():
    """A skip request cannot directly send a lateral plant approach."""
    executor = _executor()
    executor._advance_from_locked()
    executor._navigate.accept(0)
    response = executor._skip_service(None, SimpleNamespace())
    assert not response.success
    assert 'required transit' in response.message
    assert executor._index == 0
    assert executor._navigate.handles[0].cancel_count == 0
    assert len(executor._navigate.goals) == 1


def test_return_home_cancels_transit_and_never_marks_it_completed():
    """Operator return uses the normal cancel terminal and lock handshake."""
    executor = _executor()
    executor._advance_from_locked()
    executor._navigate.accept(0)
    response = executor._return_home_service(None, SimpleNamespace())
    assert response.success
    assert executor._base_lock_observed is True
    assert len(executor._navigate.goals) == 1
    executor._navigate.finish(0, GoalStatus.STATUS_CANCELED)

    assert executor._returning
    assert executor._active_transit is None
    assert executor._index == 0
    assert executor._completed_transit_ids == set()
    goal = executor._navigate.goals[1].pose.pose.position
    assert (goal.x, goal.y) == (0.5, 0.0)
    executor._navigate.accept(1)
    executor._navigate.finish(1)
    assert executor._state == 'COMPLETED'
    assert executor._base_lock_observed is True
    assert executor.work_stations == []
