"""Regression contracts for the field-route startup safety gates."""

import signal
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

import agri_task_manager.field_route_executor as executor_module
from agri_task_manager.field_route_executor import FieldRouteExecutor
from rclpy.task import Future
from std_msgs.msg import Bool


class _FakeAvailability:
    """Stand in for ROS clients used by the startup readiness check."""

    def __init__(self, available=True):
        self.available = available

    def wait_for_server(self, timeout_sec=0.0):
        """Return the configured action-server availability."""
        del timeout_sec
        return self.available

    def wait_for_service(self, timeout_sec=0.0):
        """Return the configured service availability."""
        del timeout_sec
        return self.available


def _executor_for_readiness(*, status_seen=False, active_goals=0):
    """Build the smallest executor double needed by ``_servers_ready``."""
    executor = object.__new__(FieldRouteExecutor)
    executor._require_live_map = True
    executor._live_map_valid = True
    executor._require_live_keepout = True
    executor._live_keepout_valid = True
    executor._navigate = _FakeAvailability()
    executor._require_lock = True
    executor._lock_client = _FakeAvailability()
    executor._require_idle_navigator = True
    executor._navigator_status_seen = status_seen
    executor._navigator_active_goals = active_goals
    return executor


def test_startup_does_not_require_a_preexisting_action_status_sample():
    """Allow a fresh Nav2 action server whose status topic is still silent."""
    executor = _executor_for_readiness()

    assert executor._servers_ready()


def test_startup_rejects_a_known_active_navigator_goal():
    """Keep a restarted mission from unlocking an already active goal."""
    executor = _executor_for_readiness(status_seen=True, active_goals=1)

    assert not executor._servers_ready()


class _FakeLockClient:
    """Capture asynchronous gate requests without a ROS graph."""

    def __init__(self):
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request.data)
        future = Future()
        self.futures.append(future)
        return future


class _FakeGoalHandle:
    """Expose independently controlled cancellation and result futures."""

    accepted = True

    def __init__(self):
        self.cancel_count = 0
        self.cancel_future = Future()
        self.result_future = Future()

    def cancel_goal_async(self):
        self.cancel_count += 1
        return self.cancel_future

    def get_result_async(self):
        return self.result_future


def _executor_for_flow():
    """Prepare actual executor methods with dependency-only test doubles."""
    executor = object.__new__(FieldRouteExecutor)
    executor._shutting_down = False
    executor._timer = SimpleNamespace(canceled=False)
    executor._timer.cancel = lambda: setattr(executor._timer, 'canceled', True)
    executor._state = 'AT_STATION'
    executor._detail = ''
    executor._index = 0
    executor._waypoints = []
    executor._require_lock = True
    executor._base_lock_observed = True
    executor._base_lock_confirmed = True
    executor._lock_client = _FakeLockClient()
    executor._lock_generation = 0
    executor._lock_pending = False
    executor._lock_target = None
    executor._lock_started_at = None
    executor._lock_service_succeeded = False
    executor._lock_callback = None
    executor._lock_recovery = False
    executor._lock_timeout = 3.0
    executor._lock_recovery_generation = 0
    executor._goal_generation = 0
    executor._goal_handle = None
    executor._goal_send_future = None
    executor._goal_result_future = None
    executor._goal_send_started_at = None
    executor._goal_started_at = None
    executor._goal_timeout_s = 120.0
    executor._cancel_timeout = 5.0
    executor._cancel_started_at = None
    executor._cancel_request_generation = None
    executor._cancel_goal_terminal = False
    executor._cancel_lock_confirmed = False
    executor._after_cancel = None
    executor._pending_terminal = None
    executor._start_pending = False
    executor._unsafe_active_goal = False
    executor._dwell_until = time.monotonic() - 1.0
    executor._returning = False
    executor.events = []
    executor._record = lambda event, extra=None: executor.events.append(event)
    executor._publish_status = lambda: None
    executor._publish_point_tombstone = lambda reason: None
    executor.get_logger = lambda: SimpleNamespace(
        info=lambda message: None,
        warning=lambda message: None,
        error=lambda message: None)
    return executor


def _successful_lock_response():
    return SimpleNamespace(success=True, message='gate state set')


def test_lock_transition_requires_service_success_and_matching_feedback():
    """Neither transport success nor the desired latched state is enough alone."""
    executor = _executor_for_flow()
    executor._base_lock_observed = False
    continued = []
    executor._lock_then(lambda: continued.append(True))

    executor._lock_client.futures[0].set_result(_successful_lock_response())
    assert continued == []
    assert executor._lock_pending

    executor._lock_state_callback(Bool(data=True))
    assert continued == [True]
    assert not executor._lock_pending


def test_cancel_waits_for_terminal_goal_even_after_cancel_acknowledgment():
    """A cancel acknowledgment must not resume or skip an executing goal."""
    executor = _executor_for_flow()
    executor._state = 'NAVIGATING'
    executor._base_lock_observed = False
    handle = _FakeGoalHandle()
    executor._goal_handle = handle
    outcomes = []
    executor._after_locked_cancel = outcomes.append

    executor._begin_cancel('skip')
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[1]))
    executor._lock_client.futures[0].set_result(_successful_lock_response())
    executor._lock_state_callback(Bool(data=True))
    assert outcomes == []
    assert executor._state == 'CANCELING'

    handle.result_future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED))
    executor._goal_result(handle.result_future, 0, 'test')
    assert outcomes == ['skip']
    assert executor._goal_handle is None


def test_stop_freezes_expired_station_dwell_while_waiting_for_lock(monkeypatch):
    """Shutdown's spinning waits must not let the mission advance again."""
    executor = _executor_for_flow()
    advanced = []
    executor._finish_station = lambda: advanced.append(True)
    monkeypatch.setattr(executor_module.rclpy, 'ok', lambda: True)

    def spin(_node, future, timeout_sec):
        del timeout_sec
        executor._tick()
        future.set_result(_successful_lock_response())

    monkeypatch.setattr(executor_module.rclpy, 'spin_until_future_complete', spin)
    executor.stop()

    assert executor._timer.canceled
    assert advanced == []
    assert executor._lock_client.requests == [True]
    assert executor._shutting_down


def test_stop_invalidates_late_unlock_response(monkeypatch):
    """An earlier unlock must not send a new goal during compensating lock."""
    executor = _executor_for_flow()
    continued = []
    executor._unlock_then(lambda: continued.append(True))
    old_unlock = executor._lock_client.futures[0]
    monkeypatch.setattr(executor_module.rclpy, 'ok', lambda: True)

    def spin(_node, future, timeout_sec):
        del timeout_sec
        old_unlock.set_result(_successful_lock_response())
        executor._lock_state_callback(Bool(data=False))
        future.set_result(_successful_lock_response())
        executor._lock_state_callback(Bool(data=True))

    monkeypatch.setattr(executor_module.rclpy, 'spin_until_future_complete', spin)
    executor.stop()

    assert executor._lock_client.requests == [False, True]
    assert continued == []
    assert executor._base_lock_observed is True


def test_stop_cancels_goal_accepted_during_shutdown(monkeypatch):
    """Track a late acceptance, cancel it, and suppress its arrival event."""
    executor = _executor_for_flow()
    accepted = Future()
    executor._goal_send_future = accepted
    accepted.add_done_callback(
        lambda future: executor._goal_response(future, 0, 'test'))
    handle = _FakeGoalHandle()
    entered = []
    executor._enter_station = lambda *args, **kwargs: entered.append(True)
    monkeypatch.setattr(executor_module.rclpy, 'ok', lambda: True)

    def spin(_node, future, timeout_sec):
        del timeout_sec
        if future is accepted:
            future.set_result(handle)
        elif future is handle.cancel_future:
            future.set_result(SimpleNamespace(goals_canceling=[1]))
        elif future is handle.result_future:
            future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        else:
            future.set_result(_successful_lock_response())

    monkeypatch.setattr(executor_module.rclpy, 'spin_until_future_complete', spin)
    executor.stop()

    assert handle.cancel_count == 1
    assert entered == []
    assert executor._goal_handle is None
    assert executor._lock_client.requests == [True]


def test_main_keeps_ros_context_alive_for_signal_cleanup(monkeypatch):
    """SIGTERM leaves ROS usable for stop; repeated SIGINT cannot interrupt it."""
    events = []
    handlers = {}
    original = object()

    def set_handler(signum, handler):
        previous = handlers.get(signum, original)
        handlers[signum] = handler
        return previous

    def initialize(**kwargs):
        assert kwargs['signal_handler_options'] == (
            executor_module.SignalHandlerOptions.NO)
        events.append('init')

    def spin(_node):
        handlers[signal.SIGTERM](signal.SIGTERM, None)

    def stop():
        handlers[signal.SIGINT](signal.SIGINT, None)
        events.append('stop')

    node = SimpleNamespace(
        stop=stop, destroy_node=lambda: events.append('destroy'))
    monkeypatch.setattr(executor_module.signal, 'signal', set_handler)
    monkeypatch.setattr(executor_module.rclpy, 'init', initialize)
    monkeypatch.setattr(executor_module.rclpy, 'spin', spin)
    monkeypatch.setattr(executor_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(
        executor_module.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(executor_module, 'FieldRouteExecutor', lambda: node)

    executor_module.main()

    assert events == ['init', 'stop', 'destroy', 'shutdown']
    assert handlers[signal.SIGINT] is original
    assert handlers[signal.SIGTERM] is original
