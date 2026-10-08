"""ROS 2 TwistStamped arbitration, watchdog, limit and lock node."""

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .velocity_gate import timestamp_is_current, VelocityGateCore


class VelocityGateNode(Node):
    """Provide one guarded velocity output for teleoperation and navigation."""

    def __init__(self) -> None:
        super().__init__('base_velocity_gate')
        self.declare_parameter('manual_topic', '/cmd_vel')
        self.declare_parameter('navigation_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        self.declare_parameter('lock_command_topic', '/base_motion/lock_cmd')
        self.declare_parameter('lock_state_topic', '/base_motion/locked')
        self.declare_parameter('active_source_topic', '/base_motion/active_source')
        self.declare_parameter('lock_service', '/base_motion/set_lock')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('manual_timeout', 0.35)
        self.declare_parameter('navigation_timeout', 0.35)
        self.declare_parameter('maximum_input_age', 0.5)
        self.declare_parameter('maximum_future_skew', 0.05)
        self.declare_parameter('max_linear_x', 0.30)
        self.declare_parameter('max_linear_y', 0.0)
        self.declare_parameter('max_angular_z', 0.50)

        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        topics = [
            str(self.get_parameter(name).value)
            for name in ('manual_topic', 'navigation_topic', 'output_topic')
        ]
        if len(set(topics)) != len(topics):
            raise ValueError('manual, navigation and output topics must be distinct')

        self._core = VelocityGateCore(
            priorities=('manual', 'navigation'),
            timeouts={
                'manual': float(self.get_parameter('manual_timeout').value),
                'navigation': float(self.get_parameter('navigation_timeout').value),
            },
            limits=(
                float(self.get_parameter('max_linear_x').value),
                float(self.get_parameter('max_linear_y').value),
                float(self.get_parameter('max_angular_z').value),
            ),
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._maximum_input_age = float(
            self.get_parameter('maximum_input_age').value)
        self._maximum_future_skew = float(
            self.get_parameter('maximum_future_skew').value)
        if (self._maximum_input_age <= 0.0 or
                self._maximum_future_skew < 0.0):
            raise ValueError('input age must be positive and future skew nonnegative')
        self._last_source = None

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._output = self.create_publisher(
            TwistStamped, self.get_parameter('output_topic').value, 10)
        self._locked_state = self.create_publisher(
            Bool, self.get_parameter('lock_state_topic').value, state_qos)
        self._active_source = self.create_publisher(
            String, self.get_parameter('active_source_topic').value, state_qos)
        self._manual = self.create_subscription(
            TwistStamped, self.get_parameter('manual_topic').value,
            lambda message: self._on_command('manual', message), 20)
        self._navigation = self.create_subscription(
            TwistStamped, self.get_parameter('navigation_topic').value,
            lambda message: self._on_command('navigation', message), 20)
        self._lock_command = self.create_subscription(
            Bool, self.get_parameter('lock_command_topic').value,
            lambda message: self._set_locked(message.data), 10)
        self._lock_service = self.create_service(
            SetBool, self.get_parameter('lock_service').value,
            self._set_lock_service)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_selected)
        self._publish_state('watchdog')

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, source: str, message: TwistStamped) -> None:
        if message.header.frame_id != self._frame_id:
            self._core.clear_source(source)
            self.get_logger().warning(
                f'rejected {source} velocity in frame '
                f'{message.header.frame_id!r}; expected {self._frame_id!r}')
            return
        now = self._now_seconds()
        stamp = float(message.header.stamp.sec) + float(
            message.header.stamp.nanosec) / 1e9
        if not timestamp_is_current(
                stamp, now, self._maximum_input_age,
                self._maximum_future_skew):
            self._core.clear_source(source)
            self.get_logger().warning(
                f'rejected stale, zero or future {source} velocity timestamp')
            return
        accepted = self._core.update(
            source,
            (message.twist.linear.x, message.twist.linear.y, message.twist.angular.z),
            now,
        )
        if not accepted:
            self._core.clear_source(source)
            self.get_logger().warning(f'rejected nonfinite {source} velocity command')

    def _set_locked(self, locked: bool) -> None:
        self._core.set_locked(locked)
        self._publish_selected()

    def _set_lock_service(self, request, response):
        self._set_locked(request.data)
        response.success = True
        response.message = 'base motion locked' if request.data else 'base motion unlocked'
        return response

    def _publish_state(self, source: str) -> None:
        self._locked_state.publish(Bool(data=self._core.locked))
        if source != self._last_source:
            self._active_source.publish(String(data=source))
            self._last_source = source

    def _publish_selected(self) -> None:
        now = self.get_clock().now()
        twist, source = self._core.select(now.nanoseconds / 1e9)
        message = TwistStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self._frame_id
        message.twist.linear.x = twist[0]
        message.twist.linear.y = twist[1]
        message.twist.angular.z = twist[2]
        self._output.publish(message)
        self._publish_state(source)

    def stop(self) -> None:
        """Best-effort locked zero output during orderly shutdown."""
        self._core.set_locked(True)
        for _ in range(3):
            self._publish_selected()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityGateNode()
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
