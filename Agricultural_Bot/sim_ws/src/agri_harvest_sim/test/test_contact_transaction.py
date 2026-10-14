import copy
import importlib.util
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('agri_harvest_interfaces.srv')
from agri_harvest_interfaces.srv import SimFruitCommand
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix

SCRIPTS = Path(__file__).parents[1]/'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('harvest_scene_test', SCRIPTS/'harvest_scene.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)


def fixture():
    node = module.HarvestScene.__new__(module.HarvestScene)
    node.lock = threading.RLock(); node.contact = None; node.prior_pairs = []
    node.command_lock_timeout_s = 0.02
    node.worker_lease = ('work', time.monotonic())
    node.snapshot = {'base_pose': {'xyz': [0,0,0], 'xyzw': [0,0,0,1]}, 'fruits': [
        {'entity_id': 'tomato_0_fruit_0', 'plant_id': 'tomato_0', 'ripe': True,
         'state': 'ON_PLANT', 'radius': .03, 'xyz': [.5,0,1]}]}
    node.gz = lambda _: node.snapshot
    node.update_scene = lambda _: None
    node.base_point = lambda _: [.47,0,1]
    node.get_scene = object()
    node.get_parameter = lambda _: NS(value=['gripper_left_finger_link', 'gripper_right_finger_link'])
    acm = AllowedCollisionMatrix(); acm.entry_names = ['base_link', 'arm_link1']
    for _ in acm.entry_names:
        row = AllowedCollisionEntry(); row.enabled = [True, True]; acm.entry_values.append(row)
    node.current = acm
    node.actual_attached = []
    node.call = lambda *_: NS(scene=NS(allowed_collision_matrix=node.current,
        robot_state=NS(attached_collision_objects=node.actual_attached),
        world=NS(collision_objects=[NS(id='fruit_'+fruit['entity_id'])
            for fruit in node.snapshot['fruits'] if fruit['state'] != 'ATTACHED'])))
    node.apply_scene = lambda scene: setattr(node, 'current', copy.deepcopy(scene.allowed_collision_matrix))
    return node


def request(operation, work='work'):
    req = SimFruitCommand.Request(); req.operation = operation
    req.work_id = work; req.target_id = 'visual'; req.plant_id = 'tomato_0'
    return req


def test_contact_allowance_is_local_and_clear_restores_prior_collisions():
    node = fixture()
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    names = node.current.entry_names
    fruit = names.index('fruit_tomato_0_fruit_0')
    base = names.index('base_link')
    assert not node.current.entry_values[fruit].enabled[base]
    for finger in ('gripper_left_finger_link', 'gripper_right_finger_link'):
        assert node.current.entry_values[fruit].enabled[names.index(finger)]
    assert node.current.entry_values[0].enabled[1]
    assert node.contact_command(request('clear'), SimFruitCommand.Response()).success
    assert not any(node.current.entry_values[fruit].enabled)
    assert node.current.entry_values[0].enabled[1]


def test_foreign_work_cannot_clear_active_contact_and_missing_lease_blocks_select():
    node = fixture()
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    assert not node.contact_command(request('clear', 'other'), SimFruitCommand.Response()).success
    assert node.contact is not None
    node.restore_contacts()
    node.worker_lease = None
    assert not node.contact_command(request('select'), SimFruitCommand.Response()).success


def test_uncertain_select_acknowledgement_rolls_back_mutated_server_state():
    node = fixture()
    original = node.apply_scene
    calls = []
    def apply_then_timeout(scene):
        original(scene); calls.append(1)
        if len(calls)==1:
            raise RuntimeError('acknowledgement timeout after server side effect')
    node.apply_scene = apply_then_timeout
    assert not node.contact_command(request('select'), SimFruitCommand.Response()).success
    assert node.contact is None
    fruit = node.current.entry_names.index('fruit_tomato_0_fruit_0')
    assert not any(node.current.entry_values[fruit].enabled)


def test_scene_refresh_preserves_selected_fruit_and_limits_ground_allowance_to_wheels():
    node = fixture()
    node.frame = 'base_footprint'; node.attached_ids = set()
    node.metadata = {'plant_geometry': {'stem_center_z': 1, 'stem_size': [.06,.06,2]},
        'parameters': {'ground_x': 22, 'ground_y': 14},
        'plants': [{'id': 'tomato_0', 'x': .8, 'y': 0, 'yaw': 0}],
        'fruits': [{}]}
    node.snapshot['fruits'][0]['xyzw'] = [0,0,0,1]
    node.snapshot['success'] = True
    wheels = ['f1_wheel_link','f2_wheel_link','r1_wheel_link','r2_wheel_link']
    node.get_parameter = lambda name: NS(value=wheels if name=='ground_touch_links' else
        ['gripper_left_finger_link', 'gripper_right_finger_link'])
    node.update_scene = lambda snapshot: module.HarvestScene.update_scene(node, snapshot)
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    node.update_scene(node.snapshot)
    names = node.current.entry_names; ground = names.index('field_ground')
    for wheel in wheels: assert node.current.entry_values[ground].enabled[names.index(wheel)]
    assert not node.current.entry_values[ground].enabled[names.index('base_link')]
    assert not node.current.entry_values[ground].enabled[names.index('arm_link1')]
    fruit = names.index('fruit_tomato_0_fruit_0')
    assert node.current.entry_values[fruit].enabled[names.index('gripper_left_finger_link')]


def test_busy_transaction_returns_bounded_failure_without_physical_or_scene_side_effect():
    node = fixture()
    entered = threading.Event(); release = threading.Event()
    def hold_lock():
        with node.lock:
            entered.set(); release.wait(2)
    thread = threading.Thread(target=hold_lock); thread.start()
    entered.wait(1)
    node.gz = lambda _: pytest.fail('busy request must not call Gazebo')
    node.apply_scene = lambda _: pytest.fail('busy request must not mutate collisions')
    try:
        for callback, operation in ((node.contact_command, 'select'), (node.sim_command, 'attach')):
            response = callback(request(operation), SimFruitCommand.Response())
            assert not response.success and response.state == 'BUSY'
        assert node.contact is None
    finally:
        release.set(); thread.join(1)


def test_grip_check_is_readonly_and_binds_the_preselected_entity_and_work():
    node = fixture()
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    calls = []
    node.gz = lambda payload: calls.append(payload) or {
        'success': True, 'state': 'RIGHT_CONTACT', 'entity_id': 'tomato_0_fruit_0',
        'message': 'actual one-jaw physics contact'}
    node.update_scene = lambda _: pytest.fail('readonly grip check must not update the scene')
    before = copy.deepcopy(node.current)
    response = node.sim_command(request('check_grip'), SimFruitCommand.Response())
    assert response.success and response.state == 'RIGHT_CONTACT'
    assert len(calls) == 1 and calls[0]['operation'] == 'check_grip'
    assert calls[0]['expected_entity_id'] == 'tomato_0_fruit_0'
    assert calls[0]['position'] == [.47,0,1]
    assert node.current == before
    assert not node.sim_command(request('check_grip', 'foreign'), SimFruitCommand.Response()).success
    assert len(calls) == 1


def test_grip_check_rejects_missing_selection_without_sending_a_native_command():
    node = fixture()
    node.gz = lambda _: pytest.fail('unselected grip query must not reach native physics')
    response = node.sim_command(request('check_grip'), SimFruitCommand.Response())
    assert not response.success and 'matching contact selection' in response.message


def pass_through_fixture():
    node = fixture()
    node.fruit_arm_pass_through = True
    node.frame = 'base_footprint'; node.attached_ids = set()
    node.metadata = {'plant_geometry': {'stem_center_z': 1, 'stem_size': [.06,.06,2]},
        'parameters': {'ground_x': 22, 'ground_y': 14},
        'plants': [{'id': 'tomato_0', 'x': .8, 'y': 0, 'yaw': 0}],
        'fruits': [{'id': 'tomato_0_fruit_0'}, {'id': 'tomato_0_fruit_1'}]}
    node.snapshot['fruits'][0]['xyzw'] = [0,0,0,1]
    node.snapshot['fruits'].append({'entity_id': 'tomato_0_fruit_1',
        'plant_id': 'tomato_0', 'ripe': False, 'state': 'ON_PLANT',
        'radius': .03, 'xyz': [.8,0,1], 'xyzw': [0,0,0,1]})
    node.snapshot['success'] = True
    wheels = ['f1_wheel_link','f2_wheel_link','r1_wheel_link','r2_wheel_link']
    node.get_parameter = lambda name: NS(value=wheels if name=='ground_touch_links' else
        ['gripper_left_finger_link', 'gripper_right_finger_link'])
    node.update_scene = lambda snapshot: module.HarvestScene.update_scene(node, snapshot)
    for link in ('gripper_base_link', 'gripper_left_finger_link',
                 'gripper_right_finger_link', 'tomato_box_link', 'plant_tomato_0'):
        node.ensure_entry(node.current, link)
    return node


def pair(acm, first, second):
    return acm.entry_values[acm.entry_names.index(first)].enabled[acm.entry_names.index(second)]


def assert_arm_only_demo(node):
    assert node.FRUIT_PASS_THROUGH_ARM_LINKS == tuple('arm_link'+str(i) for i in range(1,6))
    for fruit in ('fruit_tomato_0_fruit_0', 'fruit_tomato_0_fruit_1'):
        for arm in node.FRUIT_PASS_THROUGH_ARM_LINKS:
            assert pair(node.current, fruit, arm) and pair(node.current, arm, fruit)
        for protected in ('base_link', 'gripper_base_link', 'tomato_box_link', 'plant_tomato_0'):
            assert not pair(node.current, fruit, protected)
    assert pair(node.current, 'base_link', 'arm_link1')  # existing self-collision rule
    assert not pair(node.current, 'field_ground', 'arm_link1')


def test_demo_all_fruits_arm_only_permissions_survive_refresh_and_selected_finger_clear():
    node = pass_through_fixture()
    node.update_scene(node.snapshot)
    assert_arm_only_demo(node)
    for fruit in ('fruit_tomato_0_fruit_0', 'fruit_tomato_0_fruit_1'):
        assert not pair(node.current, fruit, 'gripper_left_finger_link')
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    node.update_scene(node.snapshot)
    assert_arm_only_demo(node)
    assert pair(node.current, 'fruit_tomato_0_fruit_0', 'gripper_left_finger_link')
    assert not pair(node.current, 'fruit_tomato_0_fruit_1', 'gripper_left_finger_link')
    assert node.contact_command(request('clear'), SimFruitCommand.Response()).success
    node.update_scene(node.snapshot)
    assert_arm_only_demo(node)
    assert not pair(node.current, 'fruit_tomato_0_fruit_0', 'gripper_left_finger_link')


def test_demo_transaction_rollback_keeps_arm_policy_and_revokes_only_selected_fingers():
    node = pass_through_fixture(); node.update_scene(node.snapshot)
    original = node.apply_scene
    def uncertain_contact(scene):
        original(scene)
        if node.contact is not None and pair(node.current,
                'fruit_tomato_0_fruit_0', 'gripper_left_finger_link'):
            raise RuntimeError('contact acknowledgement lost after side effect')
    node.apply_scene = uncertain_contact
    assert not node.contact_command(request('select'), SimFruitCommand.Response()).success
    assert node.contact is None
    assert_arm_only_demo(node)
    assert not pair(node.current, 'fruit_tomato_0_fruit_0', 'gripper_left_finger_link')


def test_disabled_demo_reclaims_previous_arm_pairs_without_changing_selected_fingers():
    node = pass_through_fixture(); node.update_scene(node.snapshot)
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    node.fruit_arm_pass_through = False  # emulate next node's immutable startup setting
    node.update_scene(node.snapshot)
    for fruit in ('fruit_tomato_0_fruit_0', 'fruit_tomato_0_fruit_1'):
        for arm in node.FRUIT_PASS_THROUGH_ARM_LINKS:
            assert not pair(node.current, fruit, arm) and not pair(node.current, arm, fruit)
    assert pair(node.current, 'fruit_tomato_0_fruit_0', 'gripper_left_finger_link')
    assert not pair(node.current, 'fruit_tomato_0_fruit_1', 'gripper_left_finger_link')
    assert pair(node.current, 'base_link', 'arm_link1')


def attachment_server(node):
    """Model documented MoveIt robot-before-world diff and non-atomic failure.

    A full-geometry attached ADD takes the world object automatically, while
    a redundant later world REMOVE fails. Do not roll applied side effects back.
    """
    from moveit_msgs.msg import CollisionObject
    world = {'fruit_'+fruit['entity_id'] for fruit in node.snapshot['fruits']}
    bodies = {}
    updates = []
    def call(*_):
        return NS(scene=NS(allowed_collision_matrix=node.current,
            robot_state=NS(attached_collision_objects=list(bodies.values())),
            world=NS(collision_objects=[NS(id=object_id) for object_id in sorted(world)])))
    def apply(scene):
        updates.append(copy.deepcopy(scene))
        for obj in scene.robot_state.attached_collision_objects:
            if obj.object.operation == CollisionObject.ADD:
                assert obj.object.primitives, 'explicit geometry needed for idempotent attachment'
                world.discard(obj.object.id)
                bodies[obj.object.id] = copy.deepcopy(obj)
            elif obj.object.operation == CollisionObject.REMOVE:
                if obj.object.id not in bodies:
                    raise RuntimeError('missing attachment REMOVE')
                world.add(obj.object.id)
                del bodies[obj.object.id]
        for obj in scene.world.collision_objects:
            if obj.operation == CollisionObject.ADD:
                world.add(obj.id)
            elif obj.operation == CollisionObject.REMOVE:
                if obj.id not in world:
                    raise RuntimeError('missing world REMOVE after attached ADD')
                world.remove(obj.id)
        node.current = copy.deepcopy(scene.allowed_collision_matrix)
    node.call = call; node.apply_scene = apply
    return world, bodies, updates, apply


def test_first_attach_and_replayed_refresh_are_single_full_geometry_attachment():
    node = pass_through_fixture()
    world, bodies, updates, _ = attachment_server(node)
    node.snapshot['hand_pose'] = {'xyz':[.5,0,.97], 'xyzw':[0,0,0,1]}
    node.snapshot['fruits'][0]['state'] = 'ATTACHED'
    node.update_scene(node.snapshot)
    node.update_scene(node.snapshot)
    assert set(bodies) == {'fruit_tomato_0_fruit_0'}
    assert 'fruit_tomato_0_fruit_0' not in world
    assert len(bodies['fruit_tomato_0_fruit_0'].object.primitives) == 1
    assert bodies['fruit_tomato_0_fruit_0'].touch_links == [
        'gripper_left_finger_link','gripper_right_finger_link']
    assert all(obj.id != 'fruit_tomato_0_fruit_0'
               for scene in updates for obj in scene.world.collision_objects)
    assert_arm_only_demo(node)


def test_release_uses_actual_attachment_after_node_cache_loss_and_is_replayable():
    node = pass_through_fixture(); world, bodies, updates, _ = attachment_server(node)
    node.snapshot['hand_pose'] = {'xyz':[.5,0,.97], 'xyzw':[0,0,0,1]}
    node.snapshot['fruits'][0]['state'] = 'ATTACHED'
    node.update_scene(node.snapshot)
    node.attached_ids = set()  # restarted scene node has no last-ACK cache
    node.snapshot['fruits'][0]['state'] = 'IN_BASKET'
    node.update_scene(node.snapshot)
    node.update_scene(node.snapshot)
    assert bodies == {} and 'fruit_tomato_0_fruit_0' in world
    assert len(updates[-2].robot_state.attached_collision_objects) == 1
    assert not updates[-1].robot_state.attached_collision_objects


def test_side_effect_then_false_ack_fails_closed_but_next_refresh_reconciles_actual_attachment():
    node = pass_through_fixture(); world, bodies, _, actual_apply = attachment_server(node)
    node.snapshot['hand_pose'] = {'xyz':[.5,0,.97], 'xyzw':[0,0,0,1]}
    node.snapshot['fruits'][0]['state'] = 'ATTACHED'
    def false_ack(scene):
        actual_apply(scene)
        raise RuntimeError('MoveIt rejected planning scene update')
    node.apply_scene = false_ack
    with pytest.raises(RuntimeError, match='rejected'):
        node.update_scene(node.snapshot)
    assert bodies and node.attached_ids == set()
    node.apply_scene = actual_apply
    node.update_scene(node.snapshot)
    assert node.attached_ids == {'fruit_tomato_0_fruit_0'}
    assert 'fruit_tomato_0_fruit_0' not in world


def test_true_apply_ack_without_actual_attachment_is_not_accepted():
    node = pass_through_fixture(); attachment_server(node)
    node.snapshot['hand_pose'] = {'xyz':[.5,0,.97], 'xyzw':[0,0,0,1]}
    node.snapshot['fruits'][0]['state'] = 'ATTACHED'
    node.apply_scene = lambda _: None  # service bool true but body update not applied
    with pytest.raises(RuntimeError, match='membership verification'):
        node.update_scene(node.snapshot)
    assert node.attached_ids == set()


def test_native_rejected_attachment_cannot_trigger_planning_attachment_or_false_success():
    node = fixture()
    assert node.contact_command(request('select'), SimFruitCommand.Response()).success
    node.gz = lambda _: {'success':False, 'state':'RIGHT_CONTACT',
                        'message':'two settled physical contacts required'}
    node.update_scene = lambda _: pytest.fail('native reject must not attach in MoveIt')
    response = node.sim_command(request('attach'), SimFruitCommand.Response())
    assert not response.success and response.state == 'RIGHT_CONTACT'
