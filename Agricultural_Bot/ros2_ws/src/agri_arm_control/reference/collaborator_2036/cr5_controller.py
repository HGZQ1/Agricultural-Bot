#!/usr/bin/env python3
"""MoveIt2 控制封装 + 夹爪控制"""
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2
from scipy.spatial.transform import Rotation as R
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def compute_grasp_orientation(normal):
    approach = -np.array(normal, dtype=float)
    approach = approach / np.linalg.norm(approach)
    z_axis = approach
    world_up = np.array([0, 0, 1])
    x_axis = world_up - np.dot(world_up, z_axis) * z_axis
    if np.linalg.norm(x_axis) < 0.1:
        world_x = np.array([1, 0, 0])
        x_axis = world_x - np.dot(world_x, z_axis) * z_axis
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
    return R.from_matrix(rot_matrix).as_quat().tolist()


class ArmController:
    def __init__(self, node, config):
        self.node = node
        self.moveit2 = MoveIt2(
            node,
            joint_names=config["joint_names"],
            base_link_name=config["base_link"],
            end_effector_name=config["end_effector"],
            group_name=config["group_name"],
        )
        self.moveit2.allowed_planning_time = 10.0
        self.moveit2.num_planning_attempts = 20
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        self.gripper_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self.gripper_joint = "gripper_left_finger_joint"

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True).start()

    def wait_ready(self, timeout=10.0):
        return self.moveit2.wait_for_joint_state(timeout_sec=timeout)

    def move_to_joints(self, joints, timeout=60.0):
        self.moveit2.move_to_configuration(joints)
        self.moveit2.wait_until_executed(timeout_sec=timeout)
        ok = self.moveit2.motion_succeeded
        self.node.get_logger().info(f"    关节移动: {'成功' if ok else '失败'}")
        return ok

    def move_to_pose(self, position, quat, timeout=60.0):
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat)
        self.moveit2.wait_until_executed(timeout_sec=timeout)
        ok = self.moveit2.motion_succeeded
        self.node.get_logger().info(f"    笛卡尔移动: {'成功' if ok else '失败'}")
        return ok

    def try_grasp_orientations(self, position, normal):
        self.node.get_logger().info("    使用默认姿态 [0.7071, 0, 0, 0.7071]")
        return [0.7071, 0.0, 0.0, 0.7071]

    def set_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().warn("夹爪控制器未就绪，跳过夹爪动作")
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [self.gripper_joint]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points.append(point)

        goal.trajectory = traj
        self.gripper_client.send_goal_async(goal)
        self.node.get_logger().info(f"    夹爪指令: {position}")
        return True

    def get_joint_state(self):
        return list(self.moveit2.joint_state.position)

    def get_pose(self):
        return self.moveit2.compute_fk()

    def shutdown(self):
        pass
