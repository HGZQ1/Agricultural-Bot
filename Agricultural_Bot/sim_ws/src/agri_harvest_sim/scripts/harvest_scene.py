#!/usr/bin/env python3
"""Gazebo physical fruit verification and matching MoveIt obstacles.

World truth is confined to simulated contacts/collisions. Goals must originate
from a frozen visual observation. Gazebo's actual robot pose provides the
world/base relationship; map/world alignment is never assumed.
"""
import copy
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from agri_harvest_interfaces.srv import SimFruitCommand
from agri_harvest_interfaces.msg import HarvestWorkerState

from scene_geometry import inverse_point, match_observation, relative_pose, rotate
from gazebo_transport import GazeboClient


class HarvestScene(Node):
    # Exactly the canonical URDF's five arm-only links; joint6's child is the
    # gripper body, which must retain physical/planning contact. Tool/fingers, chassis,
    # basket, stems and other robot/self pairs are outside this demo permission.
    FRUIT_PASS_THROUGH_ARM_LINKS = (
        'arm_link1', 'arm_link2', 'arm_link3', 'arm_link4', 'arm_link5')
    PLANT_PASS_THROUGH_GRIPPER_LINKS = (
        'gripper_base_link', 'gripper_left_finger_link', 'gripper_right_finger_link')

    def __init__(self):
        super().__init__('harvest_scene')
        self.declare_parameter('metadata', '')
        self.declare_parameter('world_name', 'field')
        self.declare_parameter('gz_partition', 'agricultural_bot_tomato_field')
        self.declare_parameter('planning_frame', 'base_footprint')
        self.declare_parameter('touch_links', ['gripper_left_finger_link', 'gripper_right_finger_link'])
        self.declare_parameter('ground_touch_links', ['f1_wheel_link', 'f2_wheel_link',
                                                     'r1_wheel_link', 'r2_wheel_link'])
        self.declare_parameter('scene_update_period', 0.5)
        self.declare_parameter('gazebo_request_timeout_s', 5.0)
        self.declare_parameter('moveit_ack_timeout_s', 3.0)
        self.declare_parameter('command_lock_timeout_s', 0.5)
        self.declare_parameter('allow_cli_fallback', False)
        self.declare_parameter('worker_state_topic', '/harvest_worker/state')
        self.declare_parameter('fruit_arm_pass_through', False,
            ParameterDescriptor(read_only=True,
                description='Simulation demo: allow fruit/five-arm-only-link pairs in MoveIt only; '
                            'does not disable Gazebo physics or finger contacts'))
        self.fruit_arm_pass_through = self.get_parameter('fruit_arm_pass_through').value
        if type(self.fruit_arm_pass_through) is not bool:
            raise ValueError('fruit_arm_pass_through must be an explicit boolean')
        if self.fruit_arm_pass_through:
            self.get_logger().warning('Simulation demo fruit_arm_pass_through enabled for MoveIt: '
                'all fruits may overlap the five arm-only links. Finger, basket, stem, chassis '
                'and self-collision rules remain; Gazebo physical filtering is separate.')
        self.declare_parameter('gripper_plant_pass_through', False,
            ParameterDescriptor(read_only=True,
                description='Simulation demo: allow only plant body / three gripper-link '
                            'pairs in MoveIt; fruit and other environment contacts remain'))
        self.gripper_plant_pass_through = self.get_parameter('gripper_plant_pass_through').value
        if type(self.gripper_plant_pass_through) is not bool:
            raise ValueError('gripper_plant_pass_through must be an explicit boolean')
        if self.gripper_plant_pass_through:
            self.get_logger().warning('Simulation gripper/plant proxy pass-through enabled: '
                'leaves are visual-only; only plant stem boxes may overlap the gripper. '
                'Fruit, arm, basket, chassis and ground rules remain.')
        self.metadata = json.loads(Path(self.get_parameter('metadata').value).read_text())
        self.frame = self.get_parameter('planning_frame').value
        for name, upper in (('gazebo_request_timeout_s', 10.0),
                            ('moveit_ack_timeout_s', 10.0),
                            ('command_lock_timeout_s', 2.0)):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or not 0.0 < value <= upper:
                raise ValueError(f'{name} must be finite and in (0, {upper}] wall seconds')
            setattr(self, name, value)
        if self.frame != 'base_footprint':
            raise ValueError('this Gazebo bridge uses the URDF root base_footprint as its planning frame')
        self.gz_service = '/world/' + self.get_parameter('world_name').value + '/harvest/command'
        self.gz_client = GazeboClient(self.gz_service, self.get_parameter('gz_partition').value,
            self.gazebo_request_timeout_s, self.get_parameter('allow_cli_fallback').value)
        self.get_logger().info(f'Gazebo harvest transport: {self.gz_client.backend}, '
            f'partition={self.gz_client.actual_partition}, service={self.gz_service}')
        try:
            initial = self.gz_client.request({'operation': 'scene'})
            self.get_logger().info('Gazebo initial read-only probe: '
                f'success={bool(initial.get("success"))}, '
                f'fruits={len(initial.get("fruits", []))}, sim_time={initial.get("sim_time")}')
        except Exception as error:
            # This probe cannot ACK a collision scene or authorize arm movement.
            # The refresh/ready transactions still require a fresh MoveIt ACK.
            self.get_logger().warning('Gazebo initial read-only probe failed: '+str(error))
        self.group = ReentrantCallbackGroup()
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.group)
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=self.group)
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.tf, self)
        self.lock = threading.RLock()
        self.snapshot = None
        self.scene_ready = False
        self.contact = None
        self.prior_pairs = []
        self.attached_ids = set()
        self.worker_lease = None
        self.create_subscription(HarvestWorkerState, self.get_parameter('worker_state_topic').value,
            self.worker_state, 10, callback_group=self.group)
        self.create_service(SimFruitCommand, '/harvest_sim/command', self.sim_command, callback_group=self.group)
        self.create_service(SimFruitCommand, '/harvest_scene/contact', self.contact_command, callback_group=self.group)
        self.create_timer(self.get_parameter('scene_update_period').value, self.refresh,
                          callback_group=self.group)

    def gz(self, payload):
        return self.gz_client.request(payload)

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=0.1):
            raise RuntimeError('MoveIt planning scene service unavailable')
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(self.moveit_ack_timeout_s):
            future.cancel()
            raise RuntimeError('MoveIt planning scene acknowledgement timed out')
        return future.result()

    def apply_scene(self, scene):
        request = ApplyPlanningScene.Request(); request.scene = scene
        if not self.call(self.apply, request).success:
            raise RuntimeError('MoveIt rejected planning scene update')

    @staticmethod
    def pose(data):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = data['xyz']
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = data['xyzw']
        return pose

    def collision(self, object_id, data, primitive):
        obj = CollisionObject(); obj.id = object_id; obj.header.frame_id = self.frame
        obj.operation = CollisionObject.ADD
        obj.primitives = [primitive]; obj.primitive_poses = [self.pose(data)]
        return obj

    def update_scene(self, snapshot):
        if not snapshot.get('success') or 'base_pose' not in snapshot:
            raise RuntimeError(snapshot.get('message', 'actual Gazebo robot pose is unavailable'))
        if len(snapshot.get('fruits', [])) != len(self.metadata['fruits']):
            raise RuntimeError('Gazebo has not initialized every configured fruit/plant attachment')
        # MoveIt processes attached bodies before world geometry in a diff.
        # An attached ADD with explicit geometry removes its world object itself;
        # an additional world REMOVE would therefore fail even on first attach.
        # Read actual attachments (rather than a last-ACK cache), so an applied
        # update whose acknowledgement was lost also converges on the next refresh.
        query = GetPlanningScene.Request()
        query.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        current = self.call(self.get_scene, query).scene
        configured_ids = {'fruit_' + fruit['entity_id'] for fruit in snapshot['fruits']}
        actual_attached = {obj.object.id for obj in current.robot_state.attached_collision_objects
                           if obj.object.id in configured_ids}
        scene = PlanningScene(); scene.is_diff = True; scene.robot_state.is_diff = True
        base = snapshot['base_pose']
        geometry = self.metadata['plant_geometry']
        parameters = self.metadata['parameters']
        floor = SolidPrimitive(); floor.type = SolidPrimitive.BOX
        floor.dimensions = [float(parameters['ground_x']), float(parameters['ground_y']), 0.05]
        scene.world.collision_objects.append(self.collision('field_ground', relative_pose(base,
            {'xyz': [0,0,-0.025], 'xyzw': [0,0,0,1]}), floor))
        for plant in self.metadata['plants']:
            half = plant['yaw']/2
            pose = relative_pose(base, {'xyz': [plant['x'], plant['y'], geometry['stem_center_z']],
                                       'xyzw': [0, 0, math.sin(half), math.cos(half)]})
            shape = SolidPrimitive(); shape.type = SolidPrimitive.BOX
            shape.dimensions = geometry['stem_size']
            scene.world.collision_objects.append(self.collision('plant_' + plant['id'], pose, shape))
        now_attached = set()
        for fruit in snapshot['fruits']:
            object_id = 'fruit_' + fruit['entity_id']
            shape = SolidPrimitive(); shape.type = SolidPrimitive.SPHERE; shape.dimensions = [fruit['radius']]
            if fruit['state'] == 'ATTACHED':
                now_attached.add(object_id)
                attached = AttachedCollisionObject(); attached.link_name = 'gripper_base_link'
                attached.touch_links = self.get_parameter('touch_links').value
                attached.object = self.collision(object_id, relative_pose(snapshot['hand_pose'], fruit), shape)
                attached.object.header.frame_id = 'gripper_base_link'
                scene.robot_state.attached_collision_objects.append(attached)
            else:
                scene.world.collision_objects.append(self.collision(object_id, relative_pose(base, fruit), shape))
        for object_id in actual_attached - now_attached:
            attached = AttachedCollisionObject(); attached.link_name = 'gripper_base_link'
            attached.object.id = object_id; attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)
        # Four wheel/ground contacts are intentional. Keep ground collisions for
        # every arm/body link and preserve the current target's finger allowance.
        acm = copy.deepcopy(current.allowed_collision_matrix)
        ground = self.ensure_entry(acm, 'field_ground')
        for wheel in self.get_parameter('ground_touch_links').value:
            index = self.ensure_entry(acm, wheel)
            acm.entry_values[ground].enabled[index] = True
            acm.entry_values[index].enabled[ground] = True
        self.merge_fruit_arm_permissions(acm,
            {'fruit_' + fruit['entity_id'] for fruit in snapshot['fruits']})
        self.merge_gripper_plant_permissions(acm)
        scene.allowed_collision_matrix = acm
        self.apply_scene(scene)
        # Attached-body processing does not propagate every internal failure to
        # ApplyPlanningScene's bool. Verify actual membership after the ACK;
        # never report a native attachment/release as planning-ready on bool alone.
        verification = GetPlanningScene.Request()
        verification.components.components = (PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                               PlanningSceneComponents.WORLD_OBJECT_NAMES)
        applied = self.call(self.get_scene, verification).scene
        attached = {obj.object.id: obj for obj in applied.robot_state.attached_collision_objects
                    if obj.object.id in configured_ids}
        world_ids = {obj.id for obj in applied.world.collision_objects} & configured_ids
        if set(attached) != now_attached or world_ids != configured_ids - now_attached:
            raise RuntimeError('MoveIt fruit attachment/world membership verification failed')
        fruit_by_id = {'fruit_' + fruit['entity_id']: fruit for fruit in snapshot['fruits']}
        for object_id, obj in attached.items():
            primitives = obj.object.primitives
            if (obj.link_name != 'gripper_base_link' or len(primitives) != 1 or
                    primitives[0].type != SolidPrimitive.SPHERE or
                    len(primitives[0].dimensions) != 1 or
                    abs(primitives[0].dimensions[0] - fruit_by_id[object_id]['radius']) > 1e-9):
                raise RuntimeError('MoveIt held-fruit geometry verification failed')
        self.attached_ids = now_attached
        self.snapshot = snapshot
        self.scene_ready = True

    def refresh(self):
        # Skip a refresh while a command transaction owns the scene lock.
        if not self.lock.acquire(blocking=False):
            return
        try:
            if self.contact is not None and (self.worker_lease is None or
                    self.worker_lease[0] != self.contact[0][0] or
                    time.monotonic()-self.worker_lease[1] > 2.0):
                self.restore_contacts()
            self.update_scene(self.gz({'operation': 'scene'}))
        except Exception as error:
            self.scene_ready = False
            self.get_logger().warning(str(error), throttle_duration_sec=5)
        finally:
            self.lock.release()

    def worker_state(self, message):
        self.worker_lease = (message.work_id, time.monotonic())

    def base_point(self, request):
        point = [request.position.x, request.position.y, request.position.z]
        if not all(math.isfinite(value) for value in point):
            raise ValueError('observation position must be finite')
        if not request.header.frame_id:
            raise ValueError('observation frame is required')
        if request.header.frame_id == self.frame:
            return point
        transform = self.tf.lookup_transform(self.frame, request.header.frame_id,
            Time.from_msg(request.header.stamp), timeout=Duration(seconds=0.25)).transform
        q = transform.rotation
        rotated = rotate([q.x, q.y, q.z, q.w], point)
        return [rotated[0]+transform.translation.x, rotated[1]+transform.translation.y,
                rotated[2]+transform.translation.z]

    @staticmethod
    def fill(response, result):
        response.success = bool(result.get('success', False))
        response.state = str(result.get('state', 'UNAVAILABLE'))
        response.entity_id = str(result.get('entity_id', ''))
        response.message = str(result.get('message', ''))
        return response

    def sim_command(self, request, response):
        if not self.lock.acquire(timeout=self.command_lock_timeout_s):
            return self.fill(response, {'success': False, 'state': 'BUSY',
                'message': 'scene transaction busy; no command was sent to Gazebo'})
        try:
            try:
                if request.operation not in ('attach', 'release', 'status', 'check_grip'):
                    raise ValueError('operation must be attach/release/status/check_grip')
                if not request.target_id or not request.work_id or not request.plant_id:
                    raise ValueError('observation/work identity is required')
                payload = {'operation': request.operation, 'target_id': request.target_id,
                    'work_id': request.work_id, 'plant_id': request.plant_id,
                    'position': self.base_point(request) if request.operation in ('attach', 'check_grip') else [0.0,0.0,0.0]}
                if request.operation in ('attach', 'check_grip'):
                    identity = (request.work_id, request.target_id, request.plant_id)
                    if self.contact is None or self.contact[0] != identity:
                        raise ValueError('matching contact selection is required before grip check/attachment')
                    payload['expected_entity_id'] = self.contact[1][len('fruit_'):]
                result = self.gz(payload)
                if result.get('success') and request.operation in ('attach', 'release'):
                    # Do not allow subsequent planning before held-fruit obstacles are acknowledged.
                    self.update_scene(self.gz({'operation': 'scene'}))
                return self.fill(response, result)
            except Exception as error:
                return self.fill(response, {'success': False, 'message': str(error)})
        finally:
            self.lock.release()

    @staticmethod
    def ensure_entry(acm, name):
        from moveit_msgs.msg import AllowedCollisionEntry
        if name not in acm.entry_names:
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_names.append(name)
            row = AllowedCollisionEntry(); row.enabled = [False]*len(acm.entry_names)
            acm.entry_values.append(row)
        return acm.entry_names.index(name)

    def merge_fruit_arm_permissions(self, acm, current_fruit_names=()):
        """Own only fruit/five-arm pairs; preserve all other ACM permissions.

        Metadata IDs enumerate every fruit, including green/held/basket fruit.
        Existing fruit entries are included so a scene refresh, selection,
        contact rollback or node restart cannot retain an older demo policy.
        False explicitly revokes these demo pairs on a restart without touching
        finger allowances, robot self-collision or any non-fruit obstacle pair.
        This changes planning permission only, never native contact evidence.
        """
        fruit_names = {name for name in acm.entry_names if name.startswith('fruit_')}
        fruit_names.update(current_fruit_names)
        fruit_names.update('fruit_' + fruit['id'] for fruit in
            getattr(self, 'metadata', {}).get('fruits', []) if fruit.get('id'))
        if getattr(self, 'snapshot', None) is not None:
            fruit_names.update('fruit_' + fruit['entity_id'] for fruit in
                self.snapshot.get('fruits', []) if fruit.get('entity_id'))
        enabled = getattr(self, 'fruit_arm_pass_through', False)
        for fruit in sorted(fruit_names):
            index = self.ensure_entry(acm, fruit)
            for link in self.FRUIT_PASS_THROUGH_ARM_LINKS:
                other = self.ensure_entry(acm, link)
                acm.entry_values[index].enabled[other] = enabled
                acm.entry_values[other].enabled[index] = enabled

    def merge_gripper_plant_permissions(self, acm):
        """Own only plant proxy / three-gripper pairs, preserving fruit contacts.

        The default field has visual-only leaves and a narrow stem collision box.
        Keep the box in the scene for the arm, chassis and every other body;
        only the explicitly opted-in gripper/plant pairs are allowed here.
        """
        plant_names = {'plant_' + plant['id'] for plant in
            getattr(self, 'metadata', {}).get('plants', []) if plant.get('id')}
        enabled = getattr(self, 'gripper_plant_pass_through', False)
        for plant in sorted(plant_names):
            index = self.ensure_entry(acm, plant)
            for link in self.PLANT_PASS_THROUGH_GRIPPER_LINKS:
                other = self.ensure_entry(acm, link)
                acm.entry_values[index].enabled[other] = enabled
                acm.entry_values[other].enabled[index] = enabled

    def contact_command(self, request, response):
        if not self.lock.acquire(timeout=self.command_lock_timeout_s):
            return self.fill(response, {'success': False, 'state': 'BUSY',
                'message': 'scene transaction busy; no collision allowances were changed'})
        try:
            started_select = False
            try:
                if request.operation == 'ready':
                    if not request.work_id:
                        raise ValueError('work identity is required for scene readiness')
                    self.update_scene(self.gz({'operation': 'scene'}))
                    return self.fill(response, {'success': True, 'state': 'SCENE_READY',
                        'message': 'fresh actual robot/fruit poses and all field obstacles acknowledged by MoveIt'
                            + ('; simulation demo fruit/arm planning pass-through enabled'
                               if getattr(self, 'fruit_arm_pass_through', False) else '')})
                identity = (request.work_id, request.target_id, request.plant_id)
                if not all(identity):
                    raise ValueError('observation/work identity is required')
                if request.operation == 'select':
                    if self.contact is not None:
                        raise ValueError('a contact transaction is already active; clear it before selecting again')
                    if self.worker_lease is None or self.worker_lease[0] != request.work_id or time.monotonic()-self.worker_lease[1] > 2:
                        raise ValueError('fresh matching harvest worker heartbeat is required')
                    self.update_scene(self.gz({'operation': 'scene'}))
                    fruit = match_observation(self.snapshot, self.base_point(request), request.plant_id)
                    object_id = 'fruit_' + fruit['entity_id']
                elif request.operation == 'clear':
                    if self.contact is None:
                        return self.fill(response, {'success': True, 'state': 'CLEAR', 'message': 'no active contact allowance'})
                    if self.contact[0] != identity:
                        raise ValueError('contact identity mismatch')
                    object_id = self.contact[1]
                else:
                    raise ValueError('operation must be select/clear')
                query = GetPlanningScene.Request()
                query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                acm = copy.deepcopy(self.call(self.get_scene, query).scene.allowed_collision_matrix)
                pending_pairs = []
                if request.operation == 'select':
                    index = self.ensure_entry(acm, object_id)
                    for link in self.get_parameter('touch_links').value:
                        other = self.ensure_entry(acm, link)
                        previous = (bool(acm.entry_values[index].enabled[other]),
                                    bool(acm.entry_values[other].enabled[index]))
                        pending_pairs.append((object_id, link, previous))
                        acm.entry_values[index].enabled[other] = True
                        acm.entry_values[other].enabled[index] = True
                else:
                    for obj, link, previous in self.prior_pairs:
                        index, other = self.ensure_entry(acm, obj), self.ensure_entry(acm, link)
                        acm.entry_values[index].enabled[other] = previous[0]
                        acm.entry_values[other].enabled[index] = previous[1]
                self.merge_fruit_arm_permissions(acm)
                scene = PlanningScene(); scene.is_diff = True; scene.allowed_collision_matrix = acm
                if request.operation == 'select':
                    # Keep rollback identity even when the server applies an update
                    # but its acknowledgement times out.
                    self.contact = (identity, object_id); self.prior_pairs = pending_pairs
                    started_select = True
                self.apply_scene(scene)
                if request.operation == 'select':
                    return self.fill(response, {'success': True, 'state': 'CONTACT_SELECTED',
                        'entity_id': fruit['entity_id'], 'message': 'only selected fruit/finger contacts allowed'})
                self.contact = None; self.prior_pairs = []
                return self.fill(response, {'success': True, 'state': 'CLEAR', 'message': 'contact allowances restored'})
            except Exception as error:
                if started_select and self.contact is not None:
                    try:
                        self.restore_contacts()
                    except Exception:
                        self.worker_lease = None  # refresh retries rollback before allowing new work
                return self.fill(response, {'success': False, 'message': str(error)})
        finally:
            self.lock.release()

    def restore_contacts(self):
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = copy.deepcopy(self.call(self.get_scene, query).scene.allowed_collision_matrix)
        for obj, link, previous in self.prior_pairs:
            index, other = self.ensure_entry(acm, obj), self.ensure_entry(acm, link)
            acm.entry_values[index].enabled[other] = previous[0]
            acm.entry_values[other].enabled[index] = previous[1]
        self.merge_fruit_arm_permissions(acm)
        scene = PlanningScene(); scene.is_diff = True; scene.allowed_collision_matrix = acm
        self.apply_scene(scene)
        self.contact = None; self.prior_pairs = []


def main():
    rclpy.init()
    node = HarvestScene()
    executor = MultiThreadedExecutor(num_threads=6); executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
