"""Publish wheel odometry on the stable navigation-facing /odom interface."""

from copy import deepcopy
from math import isfinite

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def odometry_is_finite(message: Odometry) -> bool:
    """Reject invalid state before it reaches navigation consumers."""
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
        *message.pose.covariance,
        *message.twist.covariance,
    )
    quaternion_norm_squared = (
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w
    )
    return (all(isfinite(value) for value in values) and
            abs(quaternion_norm_squared - 1.0) <= 1e-3)


class OdomAdapterNode(Node):
    """Validate and republish source-specific wheel odometry as /odom."""

    def __init__(self) -> None:
        super().__init__('base_odom_adapter')
        self.declare_parameter('input_topic', '/wheel/odom')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('maximum_age', 0.5)
        self.declare_parameter('maximum_future_skew', 0.05)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        if input_topic == output_topic:
            raise ValueError('input_topic and output_topic must be distinct')
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._maximum_age = float(self.get_parameter('maximum_age').value)
        self._maximum_future_skew = float(
            self.get_parameter('maximum_future_skew').value)
        if self._maximum_age <= 0.0 or self._maximum_future_skew < 0.0:
            raise ValueError('maximum_age must be positive and future skew nonnegative')
        self._last_stamp_ns = 0
        self._last_clock_ns = self.get_clock().now().nanoseconds
        self._publisher = self.create_publisher(Odometry, output_topic, 20)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._on_odometry, 20)

    def _on_odometry(self, message: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._last_clock_ns:
            self._last_stamp_ns = 0
        self._last_clock_ns = now_ns
        stamp_ns = (int(message.header.stamp.sec) * 1_000_000_000 +
                    int(message.header.stamp.nanosec))
        age = (now_ns - stamp_ns) / 1e9
        if (stamp_ns <= self._last_stamp_ns or stamp_ns == 0 or
                age < -self._maximum_future_skew or age > self._maximum_age):
            self.get_logger().warning(
                'rejected nonmonotonic, zero, stale or future wheel odometry timestamp')
            return
        if not odometry_is_finite(message):
            self.get_logger().warning('rejected nonfinite wheel odometry')
            return
        if (message.header.frame_id != self._odom_frame or
                message.child_frame_id != self._base_frame):
            self.get_logger().warning(
                'rejected wheel odometry with unexpected frame pair '
                f'{message.header.frame_id} -> {message.child_frame_id}')
            return
        self._last_stamp_ns = stamp_ns
        self._publisher.publish(deepcopy(message))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
