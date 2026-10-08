"""ROS 2 node for four-wheel-steering commands and wheel odometry."""

from math import cos, isfinite, pi, sin
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
    integrate_planar_pose,
    physical_steering_to_joint,
    steering_alignment_scale,
    steering_feedback_to_physical,
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
        self.declare_parameter('command_max_age', 0.5)
        self.declare_parameter('max_future_stamp_skew', 0.05)
        self.declare_parameter('joint_state_timeout', 0.25)
        self.declare_parameter('wheel_radius', 0.127)
        self.declare_parameter('max_wheel_speed', 8.0)
        self.declare_parameter('steering_limits', [-pi, pi])
        self.declare_parameter('steering_full_speed_error', 0.05)
        self.declare_parameter('steering_stop_error', 0.35)
        self.declare_parameter(
            'pose_covariance_diagonal', [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.04])
        self.declare_parameter(
            'twist_covariance_diagonal', [0.02, 0.02, 1000000.0, 1000000.0, 1000000.0, 0.08])
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
        self.declare_parameter('steering_joint_offsets', [
            0.005596326794896633,
            -0.005638618281266784,
            0.005638618185346499,
            -0.005638618281326736,
        ])
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
        self._steering_offsets = [
            float(v) for v in self.get_parameter('steering_joint_offsets').value]
        self._wheel_signs = [float(v) for v in self.get_parameter('wheel_joint_signs').value]
        if not all(len(values) == 4 for values in (
                self._steering_joint_names, self._wheel_joint_names,
                self._steering_signs, self._steering_offsets, self._wheel_signs)):
            raise ValueError('all joint name, sign and offset arrays must have 4 values')
        if (not all(isfinite(value) and abs(abs(value) - 1.0) < 1e-9
                    for value in self._steering_signs + self._wheel_signs) or
                not all(isfinite(value) for value in self._steering_offsets)):
            raise ValueError('joint signs must be +/-1 and steering offsets finite')
        self._wheel_radius = float(self.get_parameter('wheel_radius').value)
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._command_max_age = float(self.get_parameter('command_max_age').value)
        self._future_stamp_skew = float(
            self.get_parameter('max_future_stamp_skew').value)
        self._joint_state_timeout = float(self.get_parameter('joint_state_timeout').value)
        if (not all(isfinite(value) and value > 0.0 for value in (
                self._command_timeout, self._command_max_age,
                self._joint_state_timeout)) or
                not isfinite(self._future_stamp_skew) or self._future_stamp_skew < 0.0):
            raise ValueError('timeouts must be positive and future stamp skew nonnegative')
        self._alignment_full_speed_error = float(
            self.get_parameter('steering_full_speed_error').value)
        self._alignment_stop_error = float(
            self.get_parameter('steering_stop_error').value)
        # Validate the thresholds before accepting commands.
        steering_alignment_scale(
            [], self._alignment_full_speed_error, self._alignment_stop_error)
        self._pose_covariance = self._covariance_from_diagonal(
            self.get_parameter('pose_covariance_diagonal').value,
            'pose_covariance_diagonal')
        self._twist_covariance = self._covariance_from_diagonal(
            self.get_parameter('twist_covariance_diagonal').value,
            'twist_covariance_diagonal')
        self._kinematics = FourWheelSteeringKinematics(
            {names[i]: positions[2 * i:2 * i + 2] for i in range(4)},
            self._wheel_radius,
            float(self.get_parameter('max_wheel_speed').value),
            tuple(float(v) for v in self.get_parameter('steering_limits').value),
        )
        self._steering: Dict[str, float] = {name: 0.0 for name in names}
        self._wheel_velocity: Dict[str, float] = {name: 0.0 for name in names}
        self._twist = (0.0, 0.0, 0.0)
        now = self.get_clock().now()
        self._last_command_time = now
        self._last_odom_time = now
        self._last_joint_state_time = now
        self._last_clock_ns = now.nanoseconds
        self._command_received = False
        self._joint_state_received = False
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
        update_rate = float(self.get_parameter('update_rate').value)
        if not isfinite(update_rate) or update_rate <= 0.0:
            raise ValueError('update_rate must be positive and finite')
        update_period = 1.0 / update_rate
        self._timer = self.create_timer(update_period, self._update)

    def _set_twist(self, msg: Twist) -> bool:
        twist = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
        if not all(isfinite(value) for value in twist):
            self.get_logger().warning('rejected nonfinite velocity command')
            return False
        self._twist = twist
        self._last_command_time = self.get_clock().now()
        self._command_received = True
        return True

    def _twist_cb(self, msg: Twist) -> None:
        self._set_twist(msg)

    def _twist_stamped_cb(self, msg: TwistStamped) -> None:
        expected_frame = str(self.get_parameter('base_frame').value)
        if msg.header.frame_id != expected_frame:
            self.get_logger().warning(
                f'rejected velocity command in frame {msg.header.frame_id!r}; '
                f'expected {expected_frame!r}')
            return
        if not self._stamp_is_current(
                msg.header.stamp, self._command_max_age, allow_zero=False):
            self.get_logger().warning('rejected stale, zero or future velocity timestamp')
            return
        self._set_twist(msg.twist)

    def _joint_state_cb(self, msg: JointState) -> None:
        if not self._stamp_is_current(
                msg.header.stamp, self._joint_state_timeout, allow_zero=True):
            return
        indices = {name: index for index, name in enumerate(msg.name)}
        try:
            steering_joint_values = [
                float(msg.position[indices[name]])
                for name in self._steering_joint_names
            ]
            wheel_joint_values = [
                float(msg.velocity[indices[name]])
                for name in self._wheel_joint_names
            ]
        except (KeyError, IndexError):
            return
        if not all(isfinite(value) for value in steering_joint_values + wheel_joint_values):
            self.get_logger().warning('rejected incomplete or nonfinite joint feedback')
            return
        physical_steering = steering_feedback_to_physical(
            steering_joint_values, self._steering_signs, self._steering_offsets)
        physical_velocity = apply_joint_signs(wheel_joint_values, self._wheel_signs)
        self._steering = dict(zip(self._wheel_names, physical_steering))
        self._wheel_velocity = dict(zip(self._wheel_names, physical_velocity))
        self._last_joint_state_time = self.get_clock().now()
        self._joint_state_received = True

    def _stamp_is_current(self, stamp, maximum_age: float, allow_zero: bool) -> bool:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns == 0:
            return allow_zero
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return -self._future_stamp_skew <= age <= maximum_age

    @staticmethod
    def _covariance_from_diagonal(values, name: str):
        diagonal = [float(value) for value in values]
        if (len(diagonal) != 6 or
                any(not isfinite(value) or value < 0.0 for value in diagonal)):
            raise ValueError(f'{name} must contain 6 finite nonnegative values')
        covariance = [0.0] * 36
        for index, value in enumerate(diagonal):
            covariance[index * 6 + index] = value
        return covariance

    def _update(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds < self._last_clock_ns:
            self.get_logger().warning(
                'ROS clock moved backwards; clearing command and odometry timing state')
            self._twist = (0.0, 0.0, 0.0)
            self._command_received = False
            self._joint_state_received = False
            self._last_command_time = now
            self._last_joint_state_time = now
            self._last_odom_time = now
        self._last_clock_ns = now.nanoseconds
        command_age = (now - self._last_command_time).nanoseconds / 1e9
        if (not self._command_received or command_age < 0.0 or
                command_age > self._command_timeout):
            twist = (0.0, 0.0, 0.0)
        else:
            twist = self._twist
        commands = self._kinematics.inverse(*twist, current_steering=self._steering)
        joint_state_age = (now - self._last_joint_state_time).nanoseconds / 1e9
        joint_state_fresh = (
            self._joint_state_received and 0.0 <= joint_state_age <= self._joint_state_timeout)
        alignment_scale = 0.0
        if joint_state_fresh:
            alignment_scale = steering_alignment_scale(
                [commands[name].steering - self._steering[name]
                 for name in self._wheel_names],
                self._alignment_full_speed_error,
                self._alignment_stop_error,
            )
        steering_msg = Float64MultiArray(data=physical_steering_to_joint(
            [commands[name].steering for name in self._wheel_names],
            self._steering_signs, self._steering_offsets))
        wheel_msg = Float64MultiArray(data=apply_joint_signs(
            [commands[name].velocity * alignment_scale for name in self._wheel_names],
            self._wheel_signs))
        self._steering_pub.publish(steering_msg)
        self._wheel_pub.publish(wheel_msg)

        dt = (now - self._last_odom_time).nanoseconds / 1e9
        self._last_odom_time = now
        if not joint_state_fresh:
            return
        vx, vy, wz = self._kinematics.body_velocity(self._steering, self._wheel_velocity)
        if 0.0 < dt < 1.0:
            self._x, self._y, self._yaw = integrate_planar_pose(
                self._x, self._y, self._yaw, vx, vy, wz, dt)
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
        msg.pose.covariance = self._pose_covariance
        msg.twist.covariance = self._twist_covariance
        self._odom_pub.publish(msg)
        if self._tf_broadcaster:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

    def stop(self) -> None:
        """Best-effort zero command for orderly process shutdown."""
        zero = Float64MultiArray(data=[0.0] * len(self._wheel_names))
        for _ in range(3):
            self._wheel_pub.publish(zero)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FourWheelSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
