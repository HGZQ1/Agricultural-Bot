"""ROS 2 node for four-wheel-steering commands and wheel odometry."""

from math import cos, pi, sin
from typing import Dict

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .kinematics import (
    apply_joint_signs,
    FourWheelSteeringKinematics,
    normalize_angle,
)


class FourWheelSteeringNode(Node):
    """Bridge body velocity commands to four steering and four wheel joints."""

    def __init__(self) -> None:
        super().__init__('four_wheel_steering_node')
        self.declare_parameter('cmd_vel_type', 'twist_stamped')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('steering_command_topic', '/steering_controller/commands')
        self.declare_parameter('wheel_command_topic', '/wheel_controller/commands')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('wheel_radius', 0.127)
        self.declare_parameter('max_wheel_speed', 2.0)
        self.declare_parameter('steering_limits', [-pi, pi])
        self.declare_parameter('wheel_names', ['f1', 'f2', 'r1', 'r2'])
        self.declare_parameter('wheel_positions', [
            0.228220185, 0.173029217, 0.221645830, -0.173022364,
            -0.221740059, 0.170487567, -0.228314645, -0.170488836,
        ])
        self.declare_parameter('steering_joint_names', [
            'f1_steer_joint', 'f2_steer_joint',
            'r1_steer_joint', 'r2_steer_joint',
        ])
        self.declare_parameter('wheel_joint_names', [
            'f1_wheel_joint', 'f2_wheel_joint',
            'r1_wheel_joint', 'r2_wheel_joint',
        ])
        self.declare_parameter('steering_joint_signs', [-1.0, -1.0, -1.0, -1.0])
        self.declare_parameter('wheel_joint_signs', [1.0, 1.0, 1.0, 1.0])

        names = list(self.get_parameter('wheel_names').value)
        positions = list(self.get_parameter('wheel_positions').value)
        if len(names) != 4 or len(positions) != 8:
            raise ValueError(
                'wheel_names must have 4 values and wheel_positions 8 values')
        self._wheel_names = names
        self._steering_joint_names = list(self.get_parameter('steering_joint_names').value)
        self._wheel_joint_names = list(self.get_parameter('wheel_joint_names').value)
        self._steering_signs = [float(v) for v in self.get_parameter('steering_joint_signs').value]
        self._wheel_signs = [float(v) for v in self.get_parameter('wheel_joint_signs').value]
        if not all(len(values) == 4 for values in (
                self._steering_joint_names, self._wheel_joint_names,
                self._steering_signs, self._wheel_signs)):
            raise ValueError('all joint name and sign arrays must have 4 values')
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._kinematics = FourWheelSteeringKinematics(
            {names[i]: positions[2 * i:2 * i + 2] for i in range(4)},
            self._wheel_radius,
            float(self.get_parameter('max_wheel_speed').value),
            tuple(float(v) for v in self.get_parameter('steering_limits').value),
        )
        self._steering: Dict[str, float] = {name: 0.0 for name in names}
        self._wheel_velocity: Dict[str, float] = {name: 0.0 for name in names}
        self._twist = (0.0, 0.0, 0.0)
        self._last_command_time = self.get_clock().now()
        self._last_odom_time = self.get_clock().now()
        self._x = self._y = self._yaw = 0.0

        self._steering_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('steering_command_topic').value, 10)
        self._wheel_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter('wheel_command_topic').value, 10)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self._joint_sub = self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value,
            self._joint_state_cb, 20)
        cmd_type = self.get_parameter('cmd_vel_type').value.lower()
        if cmd_type == 'twist':
            self._cmd_sub = self.create_subscription(
                Twist, self.get_parameter('cmd_vel_topic').value,
                self._twist_cb, 20)
        elif cmd_type == 'twist_stamped':
            self._cmd_sub = self.create_subscription(
                TwistStamped, self.get_parameter('cmd_vel_topic').value,
                self._twist_stamped_cb, 20)
        else:
            raise ValueError("cmd_vel_type must be 'twist' or 'twist_stamped'")
        self._tf_broadcaster = (
            TransformBroadcaster(self)
            if self.get_parameter('publish_tf').value else None)
        update_period = 1.0 / float(self.get_parameter('update_rate').value)
        self._timer = self.create_timer(update_period, self._update)

    def _set_twist(self, msg: Twist) -> None:
        self._twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_command_time = self.get_clock().now()

    def _twist_cb(self, msg: Twist) -> None:
        self._set_twist(msg)

    def _twist_stamped_cb(self, msg: TwistStamped) -> None:
        self._set_twist(msg.twist)

    def _joint_state_cb(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if name in self._steering_joint_names and index < len(msg.position):
                wheel = self._wheel_names[self._steering_joint_names.index(name)]
                wheel_index = self._steering_joint_names.index(name)
                self._steering[wheel] = normalize_angle(
                    msg.position[index] * self._steering_signs[wheel_index])
            if name in self._wheel_joint_names and index < len(msg.velocity):
                wheel = self._wheel_names[self._wheel_joint_names.index(name)]
                wheel_index = self._wheel_joint_names.index(name)
                self._wheel_velocity[wheel] = (
                    msg.velocity[index] * self._wheel_signs[wheel_index])

    def _update(self) -> None:
        now = self.get_clock().now()
        command_age = (now - self._last_command_time).nanoseconds / 1e9
        if command_age > float(self.get_parameter('command_timeout').value):
            twist = (0.0, 0.0, 0.0)
        else:
            twist = self._twist
        commands = self._kinematics.inverse(*twist, current_steering=self._steering)
        steering_msg = Float64MultiArray(data=apply_joint_signs(
            [commands[name].steering for name in self._wheel_names],
            self._steering_signs))
        wheel_msg = Float64MultiArray(data=apply_joint_signs(
            [commands[name].velocity for name in self._wheel_names],
            self._wheel_signs))
        self._steering_pub.publish(steering_msg)
        self._wheel_pub.publish(wheel_msg)

        vx, vy, wz = self._kinematics.body_velocity(self._steering, self._wheel_velocity)
        dt = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now
        if 0.0 < dt < 1.0:
            self._x += (vx * cos(self._yaw) - vy * sin(self._yaw)) * dt
            self._y += (vx * sin(self._yaw) + vy * cos(self._yaw)) * dt
            self._yaw = normalize_angle(self._yaw + wz * dt)
        self._publish_odom(now, vx, vy, wz)

    def _publish_odom(self, stamp, vx: float, vy: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.get_parameter('odom_frame').value
        msg.child_frame_id = self.get_parameter('base_frame').value
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = cos(self._yaw / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self._odom_pub.publish(msg)
        if self._tf_broadcaster:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FourWheelSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
