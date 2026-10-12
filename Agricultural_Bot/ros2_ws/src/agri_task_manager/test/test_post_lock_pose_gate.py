"""Arrival requires a fresh, settled map pose after the motion gate locks."""

import json
import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

import agri_task_manager.field_route_executor as executor_module
from agri_task_manager.waypoint_schema import Pose2D, TransitWaypoint

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import pytest
from rclpy.task import Future
from std_msgs.msg import Bool
from test_field_route_executor import (
    _executor_for_flow, _FakeGoalHandle, _successful_lock_response)
from tf2_ros import TransformException


class _Clock:
    """Advance both clocks explicitly; the test never waits in real time."""

    def __init__(self):
        self.nanoseconds = 100_000_000_000
        self.wall = 200.0

    def now(self):
        return SimpleNamespace(
            nanoseconds=self.nanoseconds,
            to_msg=lambda: Time(
                sec=self.nanoseconds // 1_000_000_000,
                nanosec=self.nanoseconds % 1_000_000_000))

    def advance(self, seconds):
        self.nanoseconds += round(seconds * 1_000_000_000)
        self.wall += seconds


class _TfBuffer:
    """Expose actual stamped transforms, including unavailable/stale samples."""

    def __init__(self, clock, pose):
        self.clock = clock
        self.pose = pose
        self.age_s = 0.0
        self.stamp_ns = None
        self.available = True
        self.requests = []

    def lookup_transform(self, target, source, at_time, **kwargs):
        del at_time, kwargs
        self.requests.append((target, source))
        if not self.available:
            raise TransformException('map transform unavailable')
        stamp_ns = self.clock.nanoseconds - round(self.age_s * 1e9)
        if self.stamp_ns is not None:
            stamp_ns = self.stamp_ns
        transform = TransformStamped()
        transform.header.frame_id = target
        transform.child_frame_id = source
        transform.header.stamp.sec = stamp_ns // 1_000_000_000
        transform.header.stamp.nanosec = stamp_ns % 1_000_000_000
        transform.transform.translation.x = self.pose.x
        transform.transform.translation.y = self.pose.y
        transform.transform.rotation.z = math.sin(self.pose.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.pose.yaw / 2.0)
        return transform


class _Navigator:
    """Drive real executor action callbacks through controllable futures."""

    def __init__(self):
        self.goals = []
        self.acceptances = []
        self.handles = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        del feedback_callback
        acceptance = Future()
        self.goals.append(goal)
        self.acceptances.append(acceptance)
        self.handles.append(_FakeGoalHandle())
        return acceptance

    def arrive(self, index):
        self.acceptances[index].set_result(self.handles[index])
        self.handles[index].result_future.set_result(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED))


def _active_points(executor):
    return [json.loads(message.data) for message in executor.messages
            if json.loads(message.data).get('active')]


def _event_count(executor, name):
    return sum(event == name for event, _ in executor.events)


def _step(executor, seconds):
    executor.clock.advance(seconds)
    executor._tick()


def _executor(monkeypatch, *, kind='station', automatic_lock=True):
    """Use real mission methods and replace only ROS transports and clocks."""
    executor = _executor_for_flow()
    executor.clock = _Clock()
    monkeypatch.setattr(
        executor_module.time, 'monotonic', lambda: executor.clock.wall)
    executor.get_clock = lambda: executor.clock
    point = SimpleNamespace(
        waypoint_id='left_00', stop_id='left_stop_00',
        pose=Pose2D(1.2, 0.30, 0.0), goal_timeout_s=90.0,
        plant=SimpleNamespace(plant_id='tomato_00'), work_side='left',
        station_dwell_s=1.0, as_event=lambda: {'waypoint_id': 'left_00'})
    transit = TransitWaypoint(
        'headland', point.waypoint_id, Pose2D(2.0, 0.4, 0.0), 70.0)
    executor._waypoints = [point]
    executor._mission = SimpleNamespace(
        mission_id='parked_pose_test',
        map_binding=SimpleNamespace(frame_id='map'),
        route_transits=(transit,) if kind == 'transit' else (),
        home=Pose2D(0.5, 0.0, 0.0), return_home=True)
    executor._active_transit = None
    executor._completed_transit_ids = set()
    executor._last_completed_stop_id = None
    executor._require_station_ack = True
    executor._goal_kind = None
    executor._goal_pose = None
    executor._goal_label = ''
    executor._parking_goal_key = None
    executor._parking_retry_count = 0
    executor._parking_started_at = None
    executor._parking_started_ns = 0
    executor._parking_stable_since_ns = None
    executor._parking_stability_anchor = None
    executor._parking_paused = False
    executor._parking_reused_pose = False
    executor._parking_settle_s = 0.5
    executor._parking_timeout_s = 5.0
    executor._parking_tf_max_age_s = 0.35
    executor._parking_position_tolerance_m = 0.10
    executor._parking_yaw_tolerance_rad = math.radians(5.0)
    executor._parking_stability_position_m = 0.01
    executor._parking_stability_yaw_rad = math.radians(1.0)
    executor._parking_max_retries = 3
    executor._base_frame = 'base_footprint'
    executor._goal_response_timeout = 10.0
    executor._binding_error = lambda: None
    executor._dwell_until = None
    executor._navigate = _Navigator()
    executor._run_id = 'regression'
    executor.events = []
    executor.messages = []
    executor.gate_states = []
    executor._record = lambda event, extra=None: executor.events.append(
        (event, extra))
    executor._point_pub = SimpleNamespace(publish=executor.messages.append)
    pose = (transit.pose if kind == 'transit' else
            executor._mission.home if kind == 'home' else point.pose)
    executor._tf_buffer = _TfBuffer(executor.clock, pose)

    if automatic_lock:
        def set_gate(locked, callback):
            executor._base_lock_observed = locked
            executor._base_lock_confirmed = locked
            executor.gate_states.append(locked)
            callback()

        executor._unlock_then = lambda callback: set_gate(False, callback)
        executor._lock_then = lambda callback: set_gate(True, callback)

    return executor


def _start_goal(executor, kind):
    if kind == 'home':
        executor._start_return_from_locked()
    else:
        executor._advance_from_locked()


def _assert_no_arrival(executor, kind):
    assert _active_points(executor) == []
    assert executor._index == 0
    assert _event_count(executor, 'at_harvest_point') == 0
    assert _event_count(executor, 'transit_completed') == 0
    assert executor._completed_transit_ids == set()
    assert executor._state != 'COMPLETED'
    if kind == 'transit':
        assert executor._active_transit is not None


def test_parking_settle_starts_after_authoritative_lock_confirmation(monkeypatch):
    """Action success and a service reply cannot start work before lock feedback."""
    executor = _executor(monkeypatch, automatic_lock=False)
    _start_goal(executor, 'station')
    executor._lock_client.futures[0].set_result(_successful_lock_response())
    executor._lock_state_callback(Bool(data=False))
    executor._navigate.arrive(0)
    assert executor._state == 'LOCKING_STATION'
    assert executor._parking_started_at is None

    _step(executor, 1.0)
    executor._lock_client.futures[1].set_result(_successful_lock_response())
    assert executor._state == 'LOCKING_STATION'
    _assert_no_arrival(executor, 'station')
    executor._lock_state_callback(Bool(data=True))
    assert executor._state == 'PARKING_CHECK'
    executor._tick()
    _step(executor, 0.49)
    _assert_no_arrival(executor, 'station')
    _step(executor, 0.01)
    assert executor._state == 'AT_STATION'
    assert len(_active_points(executor)) == 1
    assert executor._base_lock_observed is True
    assert set(executor._tf_buffer.requests) == {('map', 'base_footprint')}


@pytest.mark.parametrize('kind', ['station', 'transit', 'home'])
def test_settled_map_pose_gates_all_navigation_arrival_semantics(monkeypatch, kind):
    """No task, transit advance, or home terminal is exposed before the pose gate."""
    executor = _executor(monkeypatch, kind=kind)
    _start_goal(executor, kind)
    executor._navigate.arrive(0)
    assert executor._state == 'PARKING_CHECK'
    _assert_no_arrival(executor, kind)
    response = executor._complete_current_service(None, SimpleNamespace())
    assert not response.success
    executor._tick()
    _step(executor, 0.49)
    _assert_no_arrival(executor, kind)
    _step(executor, 0.01)

    if kind == 'station':
        assert executor._state == 'AT_STATION'
        assert len(_active_points(executor)) == 1
        assert executor._dwell_until is None
        assert executor._base_lock_observed is True
    elif kind == 'transit':
        assert executor._completed_transit_ids == {'headland'}
        assert _event_count(executor, 'transit_completed') == 1
        assert len(executor._navigate.goals) == 2
        assert _active_points(executor) == []
        assert executor._index == 0
    else:
        assert executor._state == 'COMPLETED'
        assert executor._base_lock_observed is True
        assert _active_points(executor) == []
    _step(executor, 1.0)
    assert _event_count(executor, 'at_harvest_point') == (
        1 if kind == 'station' else 0)
    assert _event_count(executor, 'transit_completed') == (
        1 if kind == 'transit' else 0)


@pytest.mark.parametrize('kind', ['station', 'transit', 'home'])
@pytest.mark.parametrize('error', ['position', 'yaw'])
def test_pose_correction_retries_original_goal_without_consuming_work(
        monkeypatch, kind, error):
    """Correcting a locked stop retains its goal, route index, and work handshake."""
    executor = _executor(monkeypatch, kind=kind)
    _start_goal(executor, kind)
    original_pose = executor._tf_buffer.pose
    executor._tf_buffer.pose = Pose2D(
        original_pose.x + (0.101 if error == 'position' else 0.0),
        original_pose.y,
        original_pose.yaw + (math.radians(5.1) if error == 'yaw' else 0.0))
    executor._navigate.arrive(0)
    executor._tick()
    _step(executor, 0.5)

    assert len(executor._navigate.goals) == 2
    assert executor._navigate.goals[1].pose.pose == (
        executor._navigate.goals[0].pose.pose)
    assert executor._parking_retry_count == 1
    _assert_no_arrival(executor, kind)
    assert executor.gate_states == [False, True, False]
    executor._tf_buffer.pose = original_pose
    executor._navigate.arrive(1)
    executor._tick()
    _step(executor, 0.5)
    assert executor._index == 0
    if kind == 'station':
        assert executor._state == 'AT_STATION'
        assert len(_active_points(executor)) == 1
    elif kind == 'transit':
        assert executor._completed_transit_ids == {'headland'}
        assert len(executor._navigate.goals) == 3
        assert _active_points(executor) == []
    else:
        assert executor._state == 'COMPLETED'
        assert _active_points(executor) == []


@pytest.mark.parametrize('kind', ['station', 'transit', 'home'])
def test_persistent_pose_error_exhausts_retries_and_fails_locked(monkeypatch, kind):
    """Three correction goals form a finite budget and never complete a plant."""
    executor = _executor(monkeypatch, kind=kind)
    _start_goal(executor, kind)
    pose = executor._tf_buffer.pose
    executor._tf_buffer.pose = Pose2D(pose.x + 0.2, pose.y, pose.yaw)
    for goal_index in range(4):
        executor._navigate.arrive(goal_index)
        executor._tick()
        _step(executor, 0.5)

    assert executor._state == 'FAILED'
    assert executor._base_lock_observed is True
    assert executor._parking_retry_count == 3
    assert len(executor._navigate.goals) == 4
    assert executor._index == 0
    assert executor._completed_transit_ids == set()
    assert _active_points(executor) == []
    assert _event_count(executor, 'at_harvest_point') == 0
    assert _event_count(executor, 'transit_completed') == 0
    _step(executor, 10.0)
    assert len(executor._navigate.goals) == 4


@pytest.mark.parametrize('sample', ['missing', 'old', 'before_lock', 'future', 'zero'])
def test_unusable_map_tf_times_out_without_unlocking_or_publishing(monkeypatch, sample):
    """Unavailable or invalid TF cannot be treated as a successful parked pose."""
    executor = _executor(monkeypatch)
    _start_goal(executor, 'station')
    executor._navigate.arrive(0)
    if sample == 'missing':
        executor._tf_buffer.available = False
    elif sample == 'old':
        executor._tf_buffer.age_s = 0.36
    elif sample == 'before_lock':
        executor._tf_buffer.stamp_ns = executor.clock.nanoseconds - 1
    elif sample == 'future':
        executor._tf_buffer.age_s = -0.051
    else:
        executor._tf_buffer.stamp_ns = 0
    executor._tick()
    _step(executor, 0.5)
    _assert_no_arrival(executor, 'station')
    assert len(executor._navigate.goals) == 1
    _step(executor, 4.51)
    assert executor._state == 'FAILED'
    assert executor._base_lock_observed is True
    assert executor.gate_states == [False, True, True]
    assert executor._parking_retry_count == 0
    assert len(executor._navigate.goals) == 1
    assert _active_points(executor) == []


def test_yaw_tolerance_uses_shortest_map_angle_across_pi(monkeypatch):
    """Equivalent headings near the ±pi seam must not trigger correction loops."""
    executor = _executor(monkeypatch)
    executor._waypoints[0].pose = Pose2D(1.2, 0.3, math.pi - 0.02)
    _start_goal(executor, 'station')
    executor._tf_buffer.pose = Pose2D(1.2, 0.3, -math.pi + 0.02)
    executor._navigate.arrive(0)
    executor._tick()
    _step(executor, 0.5)
    assert executor._state == 'AT_STATION'
    assert executor._parking_retry_count == 0
    assert len(executor._navigate.goals) == 1


def test_localization_motion_restarts_stability_timer(monkeypatch):
    """A pose inside goal tolerance still needs half a second without drifting."""
    executor = _executor(monkeypatch)
    _start_goal(executor, 'station')
    executor._navigate.arrive(0)
    executor._tick()
    _step(executor, 0.4)
    executor._tf_buffer.pose = Pose2D(1.22, 0.3, 0.0)
    _step(executor, 0.1)
    _assert_no_arrival(executor, 'station')
    _step(executor, 0.49)
    _assert_no_arrival(executor, 'station')
    _step(executor, 0.01)
    assert executor._state == 'AT_STATION'
    assert len(_active_points(executor)) == 1


@pytest.mark.parametrize('kind', ['station', 'transit', 'home'])
def test_pause_during_pose_gate_rechecks_without_spending_correction_budget(
        monkeypatch, kind):
    """Resume cannot reuse the pre-pause stability interval or acknowledge work."""
    executor = _executor(monkeypatch, kind=kind)
    _start_goal(executor, kind)
    executor._navigate.arrive(0)
    requested_pose = executor._goal_pose
    executor._tick()
    _step(executor, 0.4)
    response = executor._pause_service(None, SimpleNamespace())
    assert response.success
    _step(executor, 3.0)
    _assert_no_arrival(executor, kind)
    assert executor._state == 'PAUSED'
    response = executor._resume_service(None, SimpleNamespace())
    assert response.success
    assert executor._state == 'PARKING_CHECK'
    assert executor._goal_pose == requested_pose
    assert executor._goal_kind == kind
    executor._tick()
    _step(executor, 0.49)
    _assert_no_arrival(executor, kind)
    _step(executor, 0.01)
    assert executor._parking_retry_count == 0
    if kind == 'station':
        assert executor._state == 'AT_STATION'
        assert len(_active_points(executor)) == 1
    elif kind == 'transit':
        assert executor._completed_transit_ids == {'headland'}
        assert executor._index == 0
        assert _active_points(executor) == []
    else:
        assert executor._state == 'COMPLETED'
        assert _active_points(executor) == []
    assert len(executor._navigate.goals) == (2 if kind == 'transit' else 1)


def test_shared_stop_does_not_bypass_pose_check_after_previous_work(monkeypatch):
    """Reusing a base stop must detect localization drift before its next task."""
    executor = _executor(monkeypatch)
    point = executor._waypoints[0]
    executor._last_completed_stop_id = point.stop_id
    executor._tf_buffer.pose = Pose2D(point.pose.x + 0.2, point.pose.y, 0.0)
    executor._advance_from_locked()
    assert executor._state == 'PARKING_CHECK'
    assert executor._navigate.goals == []
    _assert_no_arrival(executor, 'station')
    executor._tick()
    _step(executor, 0.5)
    assert len(executor._navigate.goals) == 1
    assert executor._index == 0
    assert _active_points(executor) == []


def test_lock_loss_during_verification_cannot_publish_a_station(monkeypatch):
    """A matching map pose cannot overrule authoritative unlocked feedback."""
    executor = _executor(monkeypatch)
    _start_goal(executor, 'station')
    executor._navigate.arrive(0)
    executor._tick()
    executor._lock_state_callback(Bool(data=False))
    assert executor._state == 'RECOVERING_LOCK'
    assert executor._lock_client.requests == [True]
    executor._lock_client.futures[0].set_result(_successful_lock_response())
    _step(executor, 0.5)
    assert executor._state == 'RECOVERING_LOCK'
    assert _active_points(executor) == []
    executor._lock_state_callback(Bool(data=True))
    assert executor._state == 'FAILED'
    assert executor._base_lock_observed is True
    assert len(executor._navigate.goals) == 1
    assert _active_points(executor) == []


def test_tf_dropout_invalidates_previous_stability_interval(monkeypatch):
    """Work requires a continuous valid interval after localization recovers."""
    executor = _executor(monkeypatch)
    _start_goal(executor, 'station')
    executor._navigate.arrive(0)
    executor._tick()
    _step(executor, 0.4)
    executor._tf_buffer.available = False
    _step(executor, 0.1)
    _assert_no_arrival(executor, 'station')
    executor._tf_buffer.available = True
    _step(executor, 0.1)
    _step(executor, 0.49)
    _assert_no_arrival(executor, 'station')
    _step(executor, 0.01)
    assert executor._state == 'AT_STATION'


def test_paused_simulation_does_not_spend_ros_settle_interval(monkeypatch):
    """Wall timeout still bounds the check when simulation time stops."""
    executor = _executor(monkeypatch)
    _start_goal(executor, 'station')
    executor._navigate.arrive(0)
    executor._tick()
    executor.clock.wall += 0.6
    executor._tick()
    _assert_no_arrival(executor, 'station')
    executor.clock.wall += 4.41
    executor._tick()
    assert executor._state == 'FAILED'
    assert executor._base_lock_observed is True
    assert executor._index == 0
    assert len(executor._navigate.goals) == 1


@pytest.mark.parametrize('kind', ['station', 'transit', 'home'])
def test_cancel_during_pose_verification_finishes_locked_without_work(
        monkeypatch, kind):
    """Canceling a successful but unverified goal suppresses its arrival task."""
    executor = _executor(monkeypatch, kind=kind)
    _start_goal(executor, kind)
    executor._navigate.arrive(0)
    executor._tick()
    response = executor._cancel_service(None, SimpleNamespace())
    assert response.success
    assert executor._state == 'CANCELED'
    assert executor._base_lock_observed is True
    _step(executor, 1.0)
    assert executor._state == 'CANCELED'
    assert executor._index == 0
    assert executor._completed_transit_ids == set()
    assert _event_count(executor, 'transit_completed') == 0
    assert _event_count(executor, 'at_harvest_point') == 0
    assert _active_points(executor) == []
    assert len(executor._navigate.goals) == 1


def test_return_home_during_transit_pose_verification_preserves_work_index(monkeypatch):
    """Operator return cannot mark an unverified headland transfer completed."""
    executor = _executor(monkeypatch, kind='transit')
    _start_goal(executor, 'transit')
    executor._navigate.arrive(0)
    executor._tick()
    response = executor._return_home_service(None, SimpleNamespace())
    assert response.success
    assert executor._returning
    assert executor._active_transit is None
    assert executor._completed_transit_ids == set()
    assert executor._index == 0
    assert executor._goal_kind == 'home'
    assert executor._goal_pose == executor._mission.home
    assert len(executor._navigate.goals) == 2
    home_goal = executor._navigate.goals[1].pose.pose.position
    assert (home_goal.x, home_goal.y) == (0.5, 0.0)
    assert _active_points(executor) == []
    executor._tf_buffer.pose = executor._mission.home
    executor._navigate.arrive(1)
    executor._tick()
    _step(executor, 0.5)
    assert executor._state == 'COMPLETED'
    assert executor._base_lock_observed is True
    assert executor._index == 0
    assert _event_count(executor, 'transit_completed') == 0


@pytest.mark.parametrize('kind', ['station', 'transit', 'home'])
def test_skip_during_pose_verification_only_advances_a_plant_once(monkeypatch, kind):
    """Skip handles pending work while keeping transit and home goals mandatory."""
    executor = _executor(monkeypatch, kind=kind)
    _start_goal(executor, kind)
    executor._navigate.arrive(0)
    executor._tick()
    response = executor._skip_service(None, SimpleNamespace())
    assert response.success == (kind == 'station')
    assert executor._completed_transit_ids == set()
    assert _active_points(executor) == []
    if kind == 'station':
        assert executor._index == 1
        assert executor._goal_kind == 'home'
        assert len(executor._navigate.goals) == 2
        assert _event_count(executor, 'harvest_point_skipped') == 1
        second_response = executor._skip_service(None, SimpleNamespace())
        assert not second_response.success
        _step(executor, 1.0)
        assert executor._index == 1
        assert _event_count(executor, 'harvest_point_skipped') == 1
    else:
        assert executor._index == 0
        assert executor._state == 'PARKING_CHECK'
        assert executor._goal_kind == kind
        assert len(executor._navigate.goals) == 1
        assert _event_count(executor, 'harvest_point_skipped') == 0
