#!/usr/bin/env python3
"""Explicitly enable gripper / plant proxy filtering, preserving fruit contacts."""
import argparse
import json
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_prefix
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node, NodeOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enable', action='store_true', required=True)
    parser.add_argument('--gz-partition', required=True)
    parser.add_argument('--world-name', default='field')
    args, _ = parser.parse_known_args()
    if os.environ.get('GZ_PARTITION') != args.gz_partition:
        raise RuntimeError('explicit partition must match GZ_PARTITION')
    options = NodeOptions(); options.partition = args.gz_partition
    node = Node(options)
    service = f'/world/{args.world_name}/gripper_plant_pass_through/status'
    query = StringMsg(); query.data = json.dumps({'op': 'status'})
    ok, reply = node.request(service, query, StringMsg, StringMsg, 500)
    if not ok:
        library = (Path(get_package_prefix('agri_harvest_sim')) /
                   'lib/libagri_gripper_plant_pass_through.so')
        if not library.is_file():
            raise RuntimeError('build agri_harvest_sim first')
        request = EntityPlugin_V(); request.entity.name = args.world_name
        request.entity.type = Entity.WORLD
        plugin = request.plugins.add(); plugin.name = 'agri::GripperPlantPassThroughSystem'
        plugin.filename = str(library.resolve()); plugin.innerxml = '<enabled>true</enabled>'
        ok, response = node.request(f'/world/{args.world_name}/entity/system/add',
            request, EntityPlugin_V, Boolean, 3000)
        if ok and not response.data:
            raise RuntimeError('native gripper/plant filter load rejected')
        # A missing acknowledgement does not prove failure; query the single
        # requested instance instead of loading duplicate systems into the world.
    notice = time.monotonic() + 5.
    while True:
        ok, reply = node.request(service, query, StringMsg, StringMsg, 500)
        if ok:
            state = json.loads(reply.data)
            if (state.get('system') == 'agri::GripperPlantPassThroughSystem' and
                    state.get('enabled') is True and state.get('ready') is True):
                print(json.dumps(state), flush=True)
                return
        if time.monotonic() >= notice:
            print('Waiting for Gazebo plant and gripper collisions to initialize', flush=True)
            notice = time.monotonic() + 5.
        time.sleep(.05)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
