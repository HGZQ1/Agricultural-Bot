#!/usr/bin/env python3
"""Run the real Gazebo contact fixture and record verified physical outcomes."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from make_physics_fixture import main as make_fixture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--folded-base', action='store_true')
    args = parser.parse_args()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    world = args.report.with_suffix('.sdf'); make_fixture(world, args.folded_base)
    env = os.environ.copy(); env['GZ_PARTITION'] = f'agri_harvest_fixture_{os.getpid()}'
    service = '/world/harvest_fixture/harvest/command'
    identity = {'work_id': 'physics_fixture', 'target_id': 'synthetic_visual_fixture',
                'plant_id': 'tomato_0', 'position': [.17, 0, .5]}
    steps = {}

    def request(payload):
        result = subprocess.run(['gz', 'service', '-s', service, '--reqtype', 'gz.msgs.StringMsg',
            '--reptype', 'gz.msgs.StringMsg', '--timeout', '2500',
            '--req', 'data: ' + json.dumps(json.dumps(payload))], env=env,
            capture_output=True, text=True, timeout=3.5)
        match = re.search(r'^data:\s*("(?:[^"\\]|\\.)*")', result.stdout, re.MULTILINE)
        if not match: raise RuntimeError('Gazebo service unavailable')
        return json.loads(json.loads(match.group(1)))

    def scene(): return request({'operation': 'scene'})

    def wait(predicate, timeout=15):
        deadline = time.monotonic()+timeout
        state = None
        while time.monotonic() < deadline:
            try:
                state = scene()
                if predicate(state): return state
            except RuntimeError: pass
            time.sleep(.05)
        raise AssertionError('fixture physical condition not met before deadline: ' + json.dumps(state))

    def fingers(value):
        # A one-shot CLI publisher can exit before transport discovery finishes.
        # Retry its idempotent test command until actual joint feedback agrees.
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            state = scene()
            if all(abs(q-value)<.001 for q in state['finger_positions']): return state
            for index, side in enumerate(('left', 'right')):
                if abs(state['finger_positions'][index]-value) >= .001:
                    subprocess.run(['gz', 'topic', '-t', f'/fixture/{side}', '-m', 'gz.msgs.Double',
                        '-p', f'data: {value}'], env=env, capture_output=True, check=True, timeout=3)
        raise AssertionError('test command was not confirmed by actual finger feedback')

    log = args.report.with_suffix('.log').open('w')
    process = subprocess.Popen(['gz', 'sim', '-s', '-r', str(world.resolve())], env=env,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        steps['initial'] = wait(lambda s: s.get('success') and
            s['fruits'][0]['state']=='ON_PLANT' and s['sim_time']>.5)
        assert math.dist(steps['initial']['fruits'][0]['xyz'], [.17,0,.5]) < .001
        steps['wrong_plant'] = request({**identity, 'plant_id': 'other', 'operation': 'attach'})
        assert not steps['wrong_plant']['success']
        fingers(0)
        steps['open_fingers_attach'] = request({**identity, 'operation': 'attach'})
        assert not steps['open_fingers_attach']['success']
        fingers(.012)
        steps['attach'] = request({**identity, 'operation': 'attach'})
        assert steps['attach']['success'] and steps['attach']['state']=='ATTACHED'
        subprocess.run(['gz', 'topic', '-t', '/fixture/slide', '-m', 'gz.msgs.Double',
            '-p', 'data: .1'], env=env, capture_output=True, check=True, timeout=3)
        steps['follows_gripper'] = wait(lambda s: s['fruits'][0]['xyz'][2]>.58)
        assert steps['follows_gripper']['fruits'][0]['state']=='ATTACHED'
        steps['closed_fingers_release'] = request({**identity, 'operation': 'release'})
        assert not steps['closed_fingers_release']['success']
        fingers(0)
        steps['release'] = request({**identity, 'operation': 'release'})
        assert steps['release']['success'] and steps['release']['state']=='RELEASED'
        steps['basket'] = wait(lambda s: s['fruits'][0]['state']=='IN_BASKET')
        assert .23 < steps['basket']['fruits'][0]['xyz'][2] < .24
        steps['wrong_identity_status'] = request({**identity, 'target_id': 'other', 'operation': 'status'})
        assert not steps['wrong_identity_status']['success']
        steps['repeat_pick'] = request({**identity, 'operation': 'attach'})
        assert not steps['repeat_pick']['success']
        result = {'passed': True, 'scope': 'isolated Gazebo dynamic fruit fixture, no robot IK/vision validation',
                  'folded_base': args.folded_base,
                  'truth_is_test_fixture_only': True, 'steps': steps}
        args.report.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({'passed': True, 'report': str(args.report),
                          'basket_state': steps['basket']['fruits'][0]['state']}))
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=5)
        log.close()


if __name__ == '__main__': main()
