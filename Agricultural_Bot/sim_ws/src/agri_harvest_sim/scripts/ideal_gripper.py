#!/usr/bin/env python3
"""Follow JTC position commands with two simulation-only native finger resets."""

import argparse
import json
import math
import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_prefix
from control_msgs.msg import JointTrajectoryControllerState
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode, NodeOptions
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import String


FINGERS = ('gripper_left_finger_joint', 'gripper_right_finger_joint')


def commanded_fingers(message):
    """Map actual claimed controller output, independent of state joint order."""
    names = list(message.joint_names)
    positions = message.output.positions
    if len(positions) != len(names) or any(names.count(name) != 1 for name in FINGERS):
        raise ValueError('controller output requires the two named finger positions')
    targets = tuple(float(positions[names.index(name)]) for name in FINGERS)
    if not all(math.isfinite(value) for value in targets):
        raise ValueError('controller output contains a nonfinite finger position')
    return targets


class IdealGripper(Node):
    def __init__(self, args):
        super().__init__('ideal_gripper', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        if not args.ideal_simulation:
            raise RuntimeError('explicit --ideal-simulation required')
        if os.environ.get('GZ_PARTITION') != args.gz_partition:
            raise RuntimeError('explicit partition must match GZ_PARTITION')
        self.args = args
        options = NodeOptions()
        options.partition = args.gz_partition
        self.gz = GzNode(options)
        prefix = f'/world/{args.world_name}/ideal_gripper'
        self.command_publisher = self.gz.advertise(prefix + '/commands', Double_V)
        self.status_service = prefix + '/status'
        self.load_service = f'/world/{args.world_name}/entity/system/add'
        self.ready = False
        self.load_attempted = False
        self.latest = None
        self.source_stamp_ns = None
        self.sent = 0
        self.last_error = ''
        self.create_subscription(JointTrajectoryControllerState,
                                 args.controller_state_topic, self.receive,
                                 QoSProfile(depth=1))
        self.status_publisher = self.create_publisher(String, '~/status', 10)
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.publish_status, clock=self._steady)

    def receive(self, message):
        try:
            targets = commanded_fingers(message)
        except ValueError as error:
            detail = str(error)
            if detail != self.last_error:
                self.get_logger().warning(detail + '; retaining the previous finger command')
            self.last_error = detail
            return
        self.latest = targets
        self.source_stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        self.last_error = ''
        if self.ready:
            self.send(targets)

    def send(self, targets):
        message = Double_V()
        message.data.extend(targets)
        self.command_publisher.publish(message)
        self.sent += 1

    def native_status(self):
        query = StringMsg()
        query.data = json.dumps({'op': 'status'})
        ok, response = self.gz.request(self.status_service, query, StringMsg, StringMsg, 250)
        if not ok:
            return None
        state = json.loads(response.data)
        if (state.get('system') != 'agri::IdealGripperSystem' or
                state.get('enabled') is not True or state.get('ideal_simulation') is not True):
            raise RuntimeError('existing native gripper system is not enabled in ideal mode')
        return state

    def load_native(self):
        library = (Path(get_package_prefix('agri_harvest_sim')) /
                   'lib' / 'libagri_ideal_gripper_system.so')
        if not library.is_file():
            raise RuntimeError('build agri_harvest_sim before enabling ideal fingers')
        request = EntityPlugin_V()
        request.entity.name = self.args.world_name
        request.entity.type = Entity.WORLD
        plugin = request.plugins.add()
        plugin.name = 'agri::IdealGripperSystem'
        plugin.filename = str(library.resolve())
        plugin.innerxml = '<enabled>true</enabled><ideal_simulation>true</ideal_simulation>'
        self.load_attempted = True
        ok, response = self.gz.request(self.load_service,
                                      request, EntityPlugin_V, Boolean, 3000)
        if ok and not response.data:
            raise RuntimeError('Gazebo rejected the ideal finger system load')
        if not ok:
            # A delayed acknowledgement must not install a second actuator.
            self.get_logger().warning('load acknowledgement pending; waiting for native gripper status')

    def start(self):
        next_notice = 0.0
        while rclpy.ok():
            state = self.native_status()
            if state is None and not self.load_attempted:
                if not self.args.load_system:
                    raise RuntimeError('native gripper system absent; use --load-system')
                if self.load_service in self.gz.service_list():
                    self.load_native()
            elif state is not None and state.get('ready') is True:
                if tuple(state.get('joint_names', ())) != FINGERS:
                    raise RuntimeError('native ideal system does not own the two expected fingers')
                self.ready = True
                if self.latest is not None:
                    self.send(self.latest)
                self.get_logger().info('ideal gripper enabled: both fingers follow controller output positions')
                return
            if time.monotonic() >= next_notice:
                self.get_logger().info('waiting for Gazebo ideal finger system and joint feedback')
                next_notice = time.monotonic() + 5.0
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_status(self):
        message = String()
        message.data = json.dumps({
            'ideal_simulation': True, 'native_ready_at_startup': self.ready,
            'controller_state_topic': self.args.controller_state_topic,
            'command_source': 'JointTrajectoryControllerState.output.positions',
            'source_stamp_ns': self.source_stamp_ns, 'joint_names': FINGERS,
            'target_m': self.latest, 'sent_commands': self.sent,
            'last_input_error': self.last_error,
            'native_status_service': self.status_service,
            'feedback_source': 'Gazebo physics and existing /joint_states; no synthesized feedback',
        }, sort_keys=True)
        self.status_publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gz-partition', required=True)
    parser.add_argument('--world-name', default='field')
    parser.add_argument('--controller-state-topic', default='/gripper_controller/controller_state')
    parser.add_argument('--ideal-simulation', action='store_true', required=True)
    parser.add_argument('--load-system', action='store_true')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = IdealGripper(args)
        node.start()
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
