"""A slow simulator must not cause premature cancellation or unbounded waits."""

from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading

import pytest

from agri_arm_control import controller as controller_module
from agri_arm_control.controller import ArmController, MotionError
from agri_arm_control.deadline import TrajectoryDeadline


def budget():
    return TrajectoryDeadline(7., 1_000_000_000, 0., 30., 120., 5.)


def test_7_sim_second_trajectory_at_rtf_point2_can_run_over_30_wall_seconds():
    deadline = budget()
    assert deadline.update(8_000_000_000, 35.) is None
    assert deadline.report['elapsed_wall_s'] == 35.
    assert deadline.report['observed_realtime_factor'] == pytest.approx(.2)


def test_wall_cap_still_applies_if_simulation_is_extremely_slow():
    assert budget().update(2_000_000_000, 120.) == 'hard wall execution deadline reached'


def test_simulation_progress_deadline_still_rejects_nonterminal_controller():
    assert budget().update(12_000_000_000, 44.) == 'trajectory simulation-time deadline reached'


def test_clock_stall_and_backwards_jump_both_reject_execution():
    assert budget().update(1_000_000_000, 5.) == 'ROS clock stopped advancing during controller execution'
    assert budget().update(999_000_000, .1) == 'ROS time moved backwards during controller execution'


def test_overlong_trajectory_is_rejected_before_submission():
    with pytest.raises(ValueError, match='simulation-time bound'):
        TrajectoryDeadline(31., 1_000_000_000, 0., 30., 120., 5.)


@pytest.mark.parametrize('completion_wall_s,succeeds', [(35., True), (80., False)])
def test_actual_action_wait_uses_sim_progress_and_cancels_stalled_terminal(
        monkeypatch, completion_wall_s, succeeds):
    wall = [0.]
    wrapped = Future()
    cancels = []
    def cancel():
        cancels.append(True)
        if not wrapped.done():
            wrapped.set_result(NS(status=5, result=NS(error_code=0)))
    def sleep(_seconds):
        wall[0] += .5
        if wall[0] >= completion_wall_s and not wrapped.done():
            wrapped.set_result(NS(status=4, result=NS(error_code=0)))
    monkeypatch.setattr(controller_module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(controller_module.time, 'sleep', sleep)
    controller = ArmController.__new__(ArmController)
    controller.p = {'goal_response_timeout_s': 3., 'cancel_timeout_s': 5.,
                    'execution_clock_stall_s': 5., 'execution_margin_factor': 1.25,
                    'execution_margin_s': 2.}
    controller.node = NS(context=NS(ok=lambda: True),
                         get_clock=lambda: NS(now=lambda: NS(nanoseconds=int((1.+.2*wall[0])*1e9))),
                         get_logger=lambda: NS(info=lambda _message: None))
    controller.check_permit = lambda: True
    controller.cancellation = threading.Event()
    controller._mutex = threading.RLock()
    controller._active_handle = controller._active_result = None
    controller._uncertain_submission = False
    controller._execution_status = {}
    submission = Future()
    submission.set_result(NS(accepted=True, get_result_async=lambda: wrapped,
                             cancel_goal_async=cancel))
    client = NS(server_is_ready=lambda: True, send_goal_async=lambda _goal: submission)
    if succeeds:
        controller._action(client, NS(), 30., 'arm', lambda result: result.error_code == 0,
                           trajectory_duration_s=7., hard_wall_timeout_s=120.)
        assert not cancels
        assert controller.execution_status['elapsed_wall_s'] == 35.
        assert controller.execution_status['terminal_status'] == 4
    else:
        with pytest.raises(MotionError, match='simulation-time deadline'):
            controller._action(client, NS(), 30., 'arm', lambda result: result.error_code == 0,
                               trajectory_duration_s=7., hard_wall_timeout_s=120.)
        assert cancels and controller.execution_status['terminal_confirmed']
    assert controller.terminal_confirmed
