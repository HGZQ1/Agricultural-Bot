"""Exercise physical basket outcomes and clock handling without ROS nodes."""

import json
from types import SimpleNamespace as NS

import pytest

from agri_arm_control.controller import MotionError, WorkCanceled
from agri_arm_control.harvest_worker import HarvestArmWorker


def status(sim_time, state='RELEASED', **evidence):
    return NS(success=True, state=state, entity_id='tomato_20_fruit_2',
              message=json.dumps({'sim_time': sim_time, **evidence}))


def fixture(monkeypatch, responses, *, sim=True, wall_step=.1, ros_times=None):
    import agri_arm_control.harvest_worker as module
    wall, calls = [0.], []
    replies = iter(responses)
    ros_times = iter(ros_times or [0.])
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.time, 'sleep', lambda _seconds: wall.__setitem__(
        0, wall[0] + wall_step))
    worker = NS(p={'basket_verification_timeout_s': 8.},
                get_parameter=lambda _name: NS(value=sim),
                get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(next(ros_times)*1e9))))

    def scene(operation, **kwargs):
        assert operation == 'status'
        calls.append(kwargs)
        response = next(replies)
        if isinstance(response, Exception):
            raise response
        return response

    worker._scene = scene
    return worker, calls, wall


def test_slow_simulation_does_not_fail_after_eight_wall_seconds(monkeypatch):
    worker, calls, wall = fixture(monkeypatch, [status(100.), status(100.1),
        status(100.9, 'IN_BASKET', basket_inside=True, basket_stable_s=.5)], wall_step=5.)

    response = HarvestArmWorker._scene(worker, 'wait_in_basket')

    assert response.state == 'IN_BASKET' and len(calls) == 3
    assert wall[0] == 10. and worker._basket_diagnostics['wait_elapsed_s'] == pytest.approx(.9)
    assert worker._basket_diagnostics['wait_clock'] == 'simulation'


def test_basket_containment_diagnostic_alone_cannot_fake_native_success(monkeypatch):
    worker, calls, _ = fixture(monkeypatch, [status(10., basket_inside=True),
        status(18., basket_inside=True, basket_stable_s=.2,
               basket_position_cad_m=[-.124, .23362, .2442])])

    with pytest.raises(MotionError, match='state was not confirmed') as error:
        HarvestArmWorker._scene(worker, 'wait_in_basket')

    assert len(calls) == 2
    assert 'tomato_20_fruit_2' in str(error.value)
    assert 'basket_position_cad_m' in str(error.value)
    assert worker._basket_diagnostics['state'] == 'RELEASED'


def test_lost_fruit_is_reported_immediately_with_physical_evidence(monkeypatch):
    worker, calls, _ = fixture(monkeypatch, [status(10., 'LOST', basket_inside=False)])
    with pytest.raises(MotionError, match='lost outside the basket.*basket_inside'):
        HarvestArmWorker._scene(worker, 'wait_in_basket')
    assert len(calls) == 1


def test_paused_then_rewound_simulation_resets_physical_wait_budget(monkeypatch):
    worker, calls, _ = fixture(monkeypatch, [status(100.), status(100.), status(2.),
        status(9.), status(9.1, 'IN_BASKET')], wall_step=10.)
    assert HarvestArmWorker._scene(worker, 'wait_in_basket').state == 'IN_BASKET'
    assert len(calls) == 5
    assert worker._basket_diagnostics['wait_elapsed_s'] == pytest.approx(7.1)


@pytest.mark.parametrize('message', ['physical state', '[]', '{"sim_time": null}'])
def test_older_native_status_uses_ros_simulation_clock(monkeypatch, message):
    released = NS(state='RELEASED', entity_id='fruit', message=message)
    deposited = NS(state='IN_BASKET', entity_id='fruit', message=message)
    worker, _, _ = fixture(monkeypatch, [released, deposited], ros_times=[10., 11.])
    assert HarvestArmWorker._scene(worker, 'wait_in_basket').state == 'IN_BASKET'
    assert worker._basket_diagnostics['wait_elapsed_s'] == 1.


def test_non_simulation_backend_retains_wall_clock_timeout(monkeypatch):
    worker, calls, _ = fixture(monkeypatch, [status(10.), status(10.)],
                               sim=False, wall_step=8.)
    with pytest.raises(MotionError, match='state was not confirmed'):
        HarvestArmWorker._scene(worker, 'wait_in_basket')
    assert len(calls) == 2
    assert worker._basket_diagnostics['wait_clock'] == 'wall'


@pytest.mark.parametrize('failure', [WorkCanceled('canceled'), MotionError('status unavailable')])
def test_cancel_and_transport_failure_are_never_converted_to_success(monkeypatch, failure):
    worker, calls, _ = fixture(monkeypatch, [status(10.), failure])
    with pytest.raises(type(failure), match=str(failure)):
        HarvestArmWorker._scene(worker, 'wait_in_basket', check=False)
    assert calls == [{'check': False}, {'check': False}]
