"""Geometry/control contracts shared by Gazebo, MoveIt and the harvest worker."""
from pathlib import Path
import math
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import xacro
import yaml


CONFIG = Path(__file__).resolve().parents[1]
DESCRIPTION = Path(get_package_share_directory('agri_robot_description'))


def model(controlled=True):
    return ET.fromstring(xacro.process_file(
        str(DESCRIPTION/'urdf'/'agri_robot.urdf.xacro'),
        mappings={'use_ros2_control': 'true' if controlled else 'false'}).toxml())


def vertices(name):
    data = (DESCRIPTION/'meshes'/'visual'/name).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    return np.array([struct.unpack_from('<9f', data, 84+50*i+12)
                     for i in range(count)]).reshape(-1, 3)


def test_finger_travel_preserves_clearance_instead_of_crossing_jaws():
    robot = model()
    left = robot.find("joint[@name='gripper_left_finger_joint']")
    right = robot.find("joint[@name='gripper_right_finger_joint']")
    assert left.find('mimic') is None and right.find('mimic') is None
    upper = float(left.find('limit').get('upper'))
    assert upper == float(right.find('limit').get('upper'))
    assert upper < .04
    lz = float(left.find('origin').get('xyz').split()[2])
    rz = float(right.find('origin').get('xyz').split()[2])
    inner_left = lz + vertices('gripper_left_finger_link.STL')[:, 2].max()
    inner_right = rz + vertices('gripper_right_finger_link.STL')[:, 2].min()
    assert inner_right-inner_left > .074
    assert inner_right-inner_left-2*upper > .004
    assert model(False).find("joint[@name='gripper_right_finger_joint']/mimic") is not None


def test_tcp_is_between_forward_jaws_with_outward_approach_axis():
    robot = model()
    tcp = robot.find("joint[@name='grasp_tcp_joint']")
    xyz = np.array([float(v) for v in tcp.find('origin').get('xyz').split()])
    rpy = [float(v) for v in tcp.find('origin').get('rpy').split()]
    assert tcp.find('parent').get('link') == 'tool0'
    assert np.allclose(xyz, [-.18, 0, 0])
    assert math.isclose(rpy[1], -math.pi/2)
    housing_min = vertices('gripper_base_link.STL')[:, 0].min()
    assert housing_min-xyz[0] > .05
    finger_min = vertices('gripper_left_finger_link.STL')[:, 0].min()-.169
    assert xyz[0]-finger_min > .035


def test_moveit_group_and_execution_interfaces_share_both_fingers():
    srdf = ET.parse(CONFIG/'config'/'agri_arm.srdf').getroot()
    assert srdf.find("group[@name='arm']/chain").get('tip_link') == 'grasp_tcp'
    exceptions = {(x.get('link1'), x.get('link2')) for x in srdf.findall('disable_collisions')}
    for arm in ['arm_link4', 'arm_link5']:
        for camera in ['camera_lens_link', 'camera_optical_frame']:
            assert (arm, camera) not in exceptions
    # Only the documented CAD mounting bracket contact has a local allowance.
    assert ('arm_link4', 'camera_link') not in exceptions
    assert ('arm_link5', 'camera_link') in exceptions
    control = yaml.safe_load((CONFIG/'config'/'ros2_controllers.yaml').read_text())
    execution = yaml.safe_load((CONFIG/'config'/'moveit_controllers.yaml').read_text())
    expected = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    assert control['gripper_controller']['ros__parameters']['joints'] == expected
    assert execution['moveit_simple_controller_manager']['gripper_controller']['joints'] == expected


def test_mock_wraps_canonical_model_without_a_second_geometry_snapshot():
    wrapper = ET.parse(CONFIG/'urdf'/'mock_robot.urdf.xacro').getroot()
    include = wrapper.find('{http://www.ros.org/wiki/xacro}include')
    assert include.get('filename').endswith('/urdf/agri_robot.urdf.xacro')
    assert wrapper.findall('link') == []
    assert not (CONFIG/'urdf'/'arm_full.urdf').exists()


def test_gazebo_and_moveit_gripper_action_contract_agree():
    # Source-tree check: sim_ws remains independently built; test its explicit
    # controller contract without importing a simulation package into ros2_ws.
    project = CONFIG.parents[2]
    sim_config = project/'sim_ws'/'src'/'agri_sim_control'/'config'/'controllers.yaml'
    actual = yaml.safe_load(sim_config.read_text())
    cfg = yaml.safe_load((CONFIG/'config'/'moveit_controllers.yaml').read_text())
    assert actual['controller_manager']['ros__parameters']['gripper_controller']['type'] == \
        'joint_trajectory_controller/JointTrajectoryController'
    assert actual['gripper_controller']['ros__parameters']['joints'] == \
        cfg['moveit_simple_controller_manager']['gripper_controller']['joints']
    assert actual['gripper_controller']['ros__parameters']['allow_partial_joints_goal'] is False
