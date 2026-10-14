"""Actual spawned process/GIL isolation, with no ROS or Gazebo transport."""
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from ideal_status_process import NativeStatusProcess


def busy_fake_worker(requests,replies,partition,service):
    # Simulate costly import/discovery before readiness, not an RPC deadline.
    time.sleep(.6)
    replies.send({'kind':'ready'})
    try:
        while True:
            req = requests.recv()
            end = time.monotonic()+(.65 if service=='hang' else .25)
            while time.monotonic()<end:  # holds this child's Python GIL
                pass
            packet = {'kind':'result','seq':req['seq'],
                'submitted_wall':req['submitted_wall'],'submitted_sim_ns':req['submitted_sim_ns'],
                'completed_wall':time.monotonic(),'data':{'sim_ns':req['submitted_sim_ns']}}
            if service=='error':packet['error']='bounded native timeout'
            if service=='wrong_seq':packet['seq']+=1
            replies.send(packet)
    except (EOFError,BrokenPipeError):
        pass
    finally:requests.close();replies.close()


def warmed(service='normal'):
    worker=NativeStatusProcess('offline-isolated',service,_worker=busy_fake_worker)
    end=time.monotonic()+4.
    while time.monotonic()<end and not worker.ready:
        worker.poll();time.sleep(.01)
    assert worker.ready
    return worker


def test_spawned_gil_holding_rpc_does_not_block_parent_feedback_and_has_one_inflight():
    worker=warmed()
    try:
        assert worker.submit(123_000_000_000)
        assert not worker.submit(124_000_000_000)  # no queue behind pending RPC
        times=[];packet=None;end=time.monotonic()+.5
        while time.monotonic()<end and packet is None:
            times.append(time.monotonic());packet=worker.poll();time.sleep(.01)
        assert packet is not None and packet['data']['sim_ns']==123_000_000_000
        assert len(times)>=15
        assert max(b-a for a,b in zip(times,times[1:]))<.15
        assert worker.inflight is None and worker.process.pid != __import__('os').getpid()
    finally:
        worker.close(wait=True)
    assert not worker.process.is_alive() and worker.closed
    assert not worker.submit(125_000_000_000) and worker.poll() is None


@pytest.mark.parametrize('service,expected',[
    ('error','RPC failed'),('wrong_seq','replayed'),('hang','deadline expired')])
def test_error_replay_and_hung_child_fail_without_new_query_or_lease_renewal(service,expected):
    worker=warmed(service)
    try:
        assert worker.submit(123_000_000_000)
        end=time.monotonic()+1.
        with pytest.raises(RuntimeError,match=expected):
            while time.monotonic()<end:
                worker.poll();time.sleep(.01)
        assert worker.sequence==1
    finally:worker.close(wait=True)


def test_delayed_parent_consumption_rejects_even_successful_child_reply():
    worker=warmed()
    try:
        worker.submit(123_000_000_000)
        time.sleep(.55)
        with pytest.raises(RuntimeError,match='expired'):
            worker.poll()
    finally:worker.close(wait=True)


def test_worker_exit_is_failure_and_cleanup_does_not_touch_other_processes():
    worker=warmed()
    try:
        worker.process.terminate();worker.process.join(timeout=1.)
        with pytest.raises(RuntimeError,match='pipe closed|subprocess exited'):
            worker.poll()
    finally:worker.close(wait=True)
