#!/usr/bin/env python3
"""Check real ROS parking/work transport in an empty, isolated DDS domain.

Navigation, localization, velocity feedback and visual observations are test
fixtures. The production route executor, coordinator, velocity gate and mock
worker are used. No arm or gripper controller command is published.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=198,
                        help='First isolated DDS domain; also reserves domain-1 and domain-2')
    parser.add_argument('--output', type=Path,
                        default=Path('artifacts/harvest_preflight_20261013/runtime_check.json'))
    parser.add_argument('--case', choices=('success', 'heartbeat_loss', 'stall_timeout'),
                        help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 102 <= args.domain <= 230:
        parser.error('choose an isolated domain from 102 through 230')
    return args


def run_case(args):
    """Keep all test nodes in one process but use actual DDS topics/actions."""
    import yaml
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TransformStamped, TwistStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from agri_harvest_interfaces.msg import HarvestWorkerState, TomatoTarget, TomatoTargetArray
    from agri_base_adapter.velocity_gate_node import VelocityGateNode
    from agri_task_manager.field_route_executor import FieldRouteExecutor
    from agri_task_manager.harvest_coordinator import HarvestCoordinator
    from agri_task_manager.mock_harvest_worker import MockHarvestWorker

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    source = Path(get_package_share_directory('agri_task_manager')) / 'config/missions/tomato_field_default_single_corridor.yaml'
    mission = yaml.safe_load(source.read_text())
    mission['mission_id'] += '_transport_fixture'
    for index, point in enumerate(mission['route']['waypoints']):
        point['enabled'] = index == 0
    mission['route']['transits'] = []
    mission['route']['return_home'] = False
    fixture_mission = output.parent / f'{args.case}_mission.yaml'
    fixture_mission.write_text(yaml.safe_dump(mission, sort_keys=False))
    failure_mode = 'none' if args.case == 'success' else (
        'heartbeat_loss' if args.case == 'heartbeat_loss' else 'stall')
    settings = {
        'base_velocity_gate:require_harvest_interlock': True,
        'field_route_executor:mission_file': str(fixture_mission),
        'field_route_executor:require_live_map': False,
        'field_route_executor:require_live_keepout': False,
        'field_route_executor:require_harvest_handshake': True,
        'field_route_executor:work_timeout_s': 3.0 if args.case == 'stall_timeout' else 12.0,
        'field_route_executor:log_directory': str(output.parent / args.case / 'mission_logs'),
        'mock_harvest_worker:failure_mode': failure_mode,
        'mock_harvest_worker:work_duration_s': 2.0,
        'mock_harvest_worker:retract_duration_s': 0.2,
        'harvest_coordinator:scan_timeout_s': 8.0,
        'harvest_coordinator:action_timeout_s': 10.0,
    }
    ros_args = ['--ros-args']
    for name, value in settings.items():
        encoded = ('true' if value else 'false') if isinstance(value, bool) else str(value)
        ros_args.extend(['-p', f'{name}:={encoded}'])
    rclpy.init(args=ros_args)
    # Match each production entrypoint's default rclpy.spin execution model.
    executor = SingleThreadedExecutor()
    nodes = []
    report = {'case': args.case, 'domain': args.domain, 'passed': False,
              'scope': 'real DDS production handshake nodes with artificial Nav2/TF/odom/targets',
              'live_map_topic_check': False, 'physical_motion_verified': False,
              'IK_or_actual_grasp_verified': False, 'checks': []}

    class Fixture(Node):
        def __init__(self):
            super().__init__('harvest_preflight_fixture')
            self.pose = [0.5, 0.0, 0.0]
            self.motion = 0.04
            self.target_mode = 'none'
            self.route = None
            self.station = None
            self.coordinator = None
            self.locked = None
            self.nav_succeeded_wall = None
            self.station_wall = None
            self.permit_wall = None
            self.zero_started_wall = None
            self.work_started_wall = None
            self.outputs = []
            self.work_reports = []
            self.worker_phase = None
            self._tf = TransformBroadcaster(self)
            self._static = StaticTransformBroadcaster(self)
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.rotation.w = 1.0
            self._static.sendTransform(transform)
            self._odom = self.create_publisher(Odometry, '/odom', 10)
            self._targets = self.create_publisher(TomatoTargetArray, '/agri_vision/tomato_targets_base', 10)
            self._manual = self.create_publisher(TwistStamped, '/cmd_vel', 10)
            self._unlock_topic = self.create_publisher(Bool, '/base_motion/lock_cmd', 10)
            self._reports = self.create_publisher(String, '/field_route_executor/work_report', 10)
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/field_route_executor/status', self._route, state_qos)
            self.create_subscription(String, '/field_route_executor/current_harvest_point', self._station, state_qos)
            self.create_subscription(String, '/harvest_coordinator/status', self._coordinator, state_qos)
            self.create_subscription(Bool, '/base_motion/locked', self._lock, state_qos)
            self.create_subscription(TwistStamped, '/cmd_vel_safe', self._output, 20)
            self.create_subscription(String, '/field_route_executor/work_report',
                                     lambda msg: self.work_reports.append(json.loads(msg.data)), 10)
            self.create_subscription(HarvestWorkerState, '/harvest_worker/state',
                                     lambda msg: setattr(self, 'worker_phase', msg.phase), 10)
            self.service_clients = {name: self.create_client(Trigger, f'/field_route_executor/{name}')
                                    for name in ('start', 'complete_current', 'skip', 'return_home', 'resume')}
            self.lock_client = self.create_client(SetBool, '/base_motion/set_lock')
            self._nav = ActionServer(self, NavigateToPose, '/navigate_to_pose',
                                     execute_callback=self._navigate,
                                     cancel_callback=lambda _handle: CancelResponse.ACCEPT)
            self.create_timer(0.05, self._publish_inputs)

        def _route(self, message):
            self.route = json.loads(message.data)
            if self.route.get('harvest_work_ready') and self.permit_wall is None:
                self.permit_wall = time.monotonic()

        def _station(self, message):
            event = json.loads(message.data)
            self.station = event
            if event.get('active') and self.station_wall is None:
                self.station_wall = time.monotonic()

        def _coordinator(self, message):
            self.coordinator = json.loads(message.data)
            if self.coordinator.get('selected_target_id') and self.work_started_wall is None:
                self.work_started_wall = time.monotonic()

        def _lock(self, message):
            self.locked = message.data

        def _output(self, message):
            self.outputs.append((time.monotonic(), message.twist.linear.x,
                                 message.twist.linear.y, message.twist.angular.z))

        def _navigate(self, handle):
            goal = handle.request.pose.pose
            q = goal.orientation
            self.pose = [goal.position.x, goal.position.y,
                         math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            self.nav_succeeded_wall = time.monotonic()
            handle.succeed()
            return NavigateToPose.Result()

        def _publish_inputs(self):
            stamp = self.get_clock().now().to_msg()
            x, y, yaw = self.pose
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = float(x)
            transform.transform.translation.y = float(y)
            transform.transform.rotation.z = math.sin(yaw/2)
            transform.transform.rotation.w = math.cos(yaw/2)
            self._tf.sendTransform(transform)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.twist.twist.linear.x = float(self.motion)
            self._odom.publish(odom)
            manual = TwistStamped()
            manual.header.stamp = stamp
            manual.header.frame_id = 'base_footprint'
            manual.twist.linear.x = 0.2
            self._manual.publish(manual)
            if self.target_mode == 'none':
                return
            array = TomatoTargetArray()
            array.header.stamp = stamp
            array.header.frame_id = 'base_footprint'
            array.depth_stamp = stamp
            array.observation_valid = True
            target = TomatoTarget()
            target.target_id = f'fixture_{args.case}_{self.target_mode}'
            target.class_name = 'tomato_G' if self.target_mode == 'green' else 'tomato_R'
            target.confidence = 0.95
            target.mature = self.target_mode != 'green'
            target.position.x = -0.17998
            target.position.y = -0.55 if self.target_mode == 'wrong_side' else 0.55
            target.position.z = 0.95
            target.depth_sample_count = 32
            target.depth_valid_fraction = 1.0
            target.depth_mad_m = 0.002
            target.segmentation_used = self.target_mode != 'no_segmentation'
            target.temporally_stable = self.target_mode != 'unstable'
            target.stable_frame_count = 1 if self.target_mode == 'unstable' else 5
            array.targets.append(target)
            self._targets.publish(array)

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def wait_for(predicate, description, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'timed out: {description}')

    def check(description, predicate, **evidence):
        if not predicate:
            raise AssertionError(description)
        report['checks'].append({'name': description, 'passed': True, **evidence})

    def call(client, request):
        wait_for(client.service_is_ready, client.srv_name, timeout=4.0)
        future = client.call_async(request)
        wait_for(future.done, f'response from {client.srv_name}', timeout=4.0)
        return future.result()

    try:
        # Refuse an occupied domain before any publisher capable of affecting
        # a mobile base is constructed. This process never kills foreign nodes.
        probe = Node(f'harvest_preflight_probe_{os.getpid()}')
        nodes.append(probe)
        executor.add_node(probe)
        spin_for(1.0)
        foreign = [name for name, namespace in probe.get_node_names_and_namespaces()
                   if name != probe.get_name()]
        if foreign:
            raise RuntimeError(f'DDS domain {args.domain} is occupied: {foreign}; choose another domain')
        check('isolated DDS domain was empty before test nodes', True)
        for factory in (VelocityGateNode, FieldRouteExecutor,
                        HarvestCoordinator, MockHarvestWorker, Fixture):
            node = factory()
            nodes.append(node)
            executor.add_node(node)
        fixture = nodes[-1]
        wait_for(lambda: fixture.route is not None and fixture.locked is True,
                 'locked initial route')
        spin_for(0.3)
        response = call(fixture.service_clients['start'], Trigger.Request())
        check('mission starts with fresh idle worker lease', response.success, message=response.message)
        wait_for(lambda: fixture.nav_succeeded_wall is not None, 'artificial Nav2 success')
        spin_for(0.8)
        check('Nav2 success while feedback still moves does not deliver a station',
              fixture.station is None or not fixture.station.get('active'))
        check('moving feedback cannot grant work permission', fixture.permit_wall is None)
        fixture.motion = 0.0
        fixture.zero_started_wall = time.monotonic()
        spin_for(0.4)
        check('less than 0.5 seconds of measured stop cannot deliver work',
              fixture.permit_wall is None)
        wait_for(lambda: fixture.route.get('harvest_work_ready'), 'continuous stop permission')
        elapsed = fixture.permit_wall - fixture.zero_started_wall
        check('fresh zero velocity held at least 0.5 seconds before permission', elapsed >= 0.5,
              elapsed_wall_s=elapsed)

        if args.case == 'success':
            for mode in ('green', 'unstable', 'wrong_side', 'no_segmentation'):
                fixture.target_mode = mode
                spin_for(0.35)
                check(f'{mode} target cannot start work', fixture.work_started_wall is None)
            spoof = {'run_id': fixture.station['run_id'],
                     'waypoint_id': fixture.station['waypoint_id'],
                     'work_id': 'wrong_nonce', 'phase': 'SUCCESS',
                     'stamp_ns': fixture.get_clock().now().nanoseconds,
                     'success': True, 'arm_stowed': True, 'terminal_confirmed': True}
            fixture._reports.publish(String(data=json.dumps(spoof)))
            spin_for(0.15)
            check('wrong work nonce cannot advance station',
                  fixture.route['waypoint_index'] == 0 and fixture.route['state'] == 'AT_STATION')
        fixture.target_mode = 'good'
        wait_for(lambda: fixture.work_started_wall is not None, 'accepted mature work target')
        wait_for(lambda: fixture.route.get('work_phase') == 'WORKING', 'work STARTED report')
        check('mature stable correct-side target selected',
              fixture.coordinator.get('selected_target_id') == f'fixture_{args.case}_good')
        if args.case == 'success':
            wait_for(lambda: fixture.worker_phase in {'DEPLOYING', 'SCANNING', 'WORKING'},
                     'actual mock worker deployment acknowledgement')
            for name in ('complete_current', 'skip', 'return_home', 'resume'):
                response = call(fixture.service_clients[name], Trigger.Request())
                check(f'active work rejects manual {name}', not response.success, message=response.message)
            response = call(fixture.lock_client, SetBool.Request(data=False))
            check('active worker rejects direct unlock service', not response.success, message=response.message)
            fixture._unlock_topic.publish(Bool(data=False))
            spin_for(0.15)
            check('active worker rejects direct unlock topic', fixture.locked is True)
            samples = [sample for sample in fixture.outputs
                       if sample[0] >= fixture.work_started_wall + 0.1]
            check('manual commands produce zero final output during work',
                  bool(samples) and all(max(abs(v) for v in sample[1:]) == 0.0 for sample in samples),
                  zero_output_samples=len(samples))
            wait_for(lambda: fixture.route.get('state') == 'COMPLETED',
                     'successful action plus fresh STOWED acknowledgement', timeout=8.0)
            check('success and fresh matching stow complete the one enabled station',
                  fixture.route['waypoint_index'] == 1 and fixture.locked is True)
            logs = list((output.parent / args.case / 'mission_logs').glob('*.jsonl'))
            events = [json.loads(line) for path in logs for line in path.read_text().splitlines()]
            check('wrong nonce rejection logged', any(event['event'] == 'work_report_rejected' for event in events))
            check('verified action/stow handoff logged', any(event['event'] == 'harvest_work_verified' for event in events))
        else:
            wait_for(lambda: fixture.route.get('state') == 'FAILED',
                     f'{args.case} fails closed', timeout=8.0)
            detail = fixture.route['detail']
            expected = 'heartbeat' if args.case == 'heartbeat_loss' else 'timeout'
            check(f'{args.case} failure reason recorded', expected in detail, detail=detail)
            check(f'{args.case} preserves final base lock', fixture.locked is True)
            response = call(fixture.service_clients['complete_current'], Trigger.Request())
            check(f'{args.case} cannot manually complete failed work', not response.success)
            samples = [sample for sample in fixture.outputs
                       if sample[0] >= fixture.work_started_wall + 0.1]
            check(f'{args.case} keeps manual commands at zero output',
                  bool(samples) and all(max(abs(v) for v in sample[1:]) == 0.0 for sample in samples),
                  zero_output_samples=len(samples))
        report['last_route_status'] = fixture.route
        report['last_coordinator_status'] = fixture.coordinator
        report['passed'] = True
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        traceback.print_exc()
        if nodes and isinstance(nodes[-1], Fixture):
            report['last_route_status'] = nodes[-1].route
            report['last_coordinator_status'] = nodes[-1].coordinator
    finally:
        # Only our own objects are touched; no user process or graph is killed.
        for node in nodes:
            if isinstance(node, HarvestCoordinator):
                node._cancel_action()
        if len(nodes) > 1:
            spin_for(0.35)
        for node in nodes:
            if isinstance(node, VelocityGateNode):
                node.stop()
        executor.shutdown(timeout_sec=0.5)
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    return 0 if report['passed'] else 1


def main():
    args = arguments()
    project = Path(__file__).resolve().parents[1]
    os.chdir(project)
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(args.output.parent / 'ros_logs')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_STATIC_PEERS', None)
    if args.case:
        return run_case(args)
    reports = []
    for index, case in enumerate(('success', 'heartbeat_loss', 'stall_timeout')):
        case_output = args.output.parent / f'{case}_check.json'
        case_log = args.output.parent / f'{case}.log'
        case_output.unlink(missing_ok=True)
        command = [sys.executable, str(Path(__file__).resolve()), '--case', case,
                   '--domain', str(args.domain-index), '--output', str(case_output)]
        with case_log.open('w') as stream:
            child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            try:
                returncode = child.wait(timeout=35.0)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
                returncode = 124
            finally:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
        if case_output.exists():
            report = json.loads(case_output.read_text())
        else:
            report = {'case': case, 'passed': False, 'error': 'case produced no report; inspect its log'}
        report['process_exit_code'] = returncode
        report['log_path'] = str(case_log)
        reports.append(report)
        print(f'{case}: {"PASS" if report.get("passed") and returncode == 0 else "FAIL"}', flush=True)
    result = {'passed': all(item.get('passed') and item['process_exit_code'] == 0 for item in reports),
              'scope': 'real DDS with fixture navigation/localization/depth targets; mock arm only',
              'actual_grasp_verified': False, 'real_localization_accuracy_verified': False,
              'cases': reports}
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(f'Report: {args.output}', flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
