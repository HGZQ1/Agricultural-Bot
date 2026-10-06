"""Express Gazebo RGB-D calibration using ROS integer pixel centres.

Gazebo's OpenGL projection uses image edge coordinates, so pixel (u, v)
samples (u + 0.5, v + 0.5). Subtracting that offset from K/P principal points
lets ordinary ROS backprojection recover those rays without editing images.
This node applies only to the simulation bridge's internal raw CameraInfo.
"""

from copy import deepcopy
import math
from numbers import Real

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


def _validate_offset(pixel_center_offset):
    if isinstance(pixel_center_offset, bool) or not isinstance(pixel_center_offset, Real) \
            or not math.isfinite(pixel_center_offset) or not 0 <= pixel_center_offset <= 1:
        raise ValueError('pixel_center_offset must be finite and between 0 and 1 pixel')


def _validate_topics(input_topic, output_topic):
    if not isinstance(input_topic, str) or not input_topic.strip() \
            or not isinstance(output_topic, str) or not output_topic.strip() \
            or input_topic == output_topic:
        raise ValueError('input_topic and output_topic must be distinct nonempty topics')


def normalize_camera_info(info: CameraInfo, pixel_center_offset: float = 0.5) -> CameraInfo:
    """Copy metadata and correct only the four K/P principal-point entries.

    Input remains unchanged. A zero offset returns an independent exact copy,
    which can disable this conversion for a renderer already using ROS pixel
    centres. Focal lengths, extrinsics, distortion, stamps and ROI stay intact.
    """
    _validate_offset(pixel_center_offset)
    if info.width <= 0 or info.height <= 0:
        raise ValueError('CameraInfo resolution must be positive')
    if not all(math.isfinite(value) for value in (*info.k, *info.p)):
        raise ValueError('CameraInfo K/P must contain finite values')
    if not all(value > 0 for value in (info.k[0], info.k[4], info.p[0], info.p[5])):
        raise ValueError('CameraInfo K/P focal lengths must be positive')
    output = deepcopy(info)
    for matrix, index, limit in (
        (output.k, 2, output.width), (output.k, 5, output.height),
        (output.p, 2, output.width), (output.p, 6, output.height),
    ):
        matrix[index] -= pixel_center_offset
        if not 0 <= matrix[index] < limit:
            raise ValueError('Corrected CameraInfo principal point lies outside the image')
    return output


class D405CameraInfoNormalizer(Node):
    def __init__(self):
        super().__init__('d405_camera_info_normalizer')
        defaults = {
            'input_topic': '/d405/color/camera_info/gz_raw',
            'output_topic': '/d405/color/camera_info',
            'pixel_center_offset': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        _validate_topics(input_topic, output_topic)
        # Include ROS remappings in the feedback-loop guard.
        _validate_topics(self.resolve_topic_name(input_topic), self.resolve_topic_name(output_topic))
        self._pixel_center_offset = self.get_parameter('pixel_center_offset').value
        _validate_offset(self._pixel_center_offset)
        self._publisher = self.create_publisher(CameraInfo, output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            CameraInfo, input_topic, self._on_info, qos_profile_sensor_data)

    def _on_info(self, info):
        try:
            output = normalize_camera_info(info, self._pixel_center_offset)
        except ValueError as error:
            self.get_logger().warning(f'Dropping Gazebo CameraInfo: {error}', throttle_duration_sec=5.0)
            return
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = D405CameraInfoNormalizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
