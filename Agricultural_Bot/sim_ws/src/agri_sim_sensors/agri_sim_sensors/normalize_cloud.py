"""Apply the MID-360 output contract without changing the official RGL preset.

The upstream preset has a few rays outside its nominal vertical field of view.
Filter in the sensor frame and copy whole point records so additional fields,
byte order, and per-point padding survive unchanged.
"""

from array import array
from copy import deepcopy
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


SENSOR_FRAME = 'mid360_sensor_frame'
ANGLE_TOLERANCE = 1e-6


def _validate_limits(frame_id, min_range, max_range, vertical_min_angle, vertical_max_angle):
    if frame_id != SENSOR_FRAME:
        raise ValueError(f'frame_id must be {SENSOR_FRAME!r}')
    values = (min_range, max_range, vertical_min_angle, vertical_max_angle)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Range and angle limits must be finite')
    if not 0.0 <= min_range < max_range:
        raise ValueError('Expected 0 <= min_range < max_range')
    if not -math.pi / 2 <= vertical_min_angle < vertical_max_angle <= math.pi / 2:
        raise ValueError('Expected -pi/2 <= vertical_min_angle < vertical_max_angle <= pi/2')


def normalize_cloud(
    cloud: PointCloud2,
    *,
    frame_id: str = SENSOR_FRAME,
    min_range: float = 0.1,
    max_range: float = 40.0,
    vertical_min_angle: float = math.radians(-7.0),
    vertical_max_angle: float = math.radians(52.0),
) -> PointCloud2:
    """Return an unorganized filtered cloud; never mutate or relabel the input.

    Coordinates may be FLOAT32 or FLOAT64 at arbitrary offsets. Row padding
    and big-endian payloads are supported. The output preserves each selected
    point's entire point_step bytes, including fields this node does not read.
    A frame mismatch or malformed XYZ layout raises ValueError.
    """
    _validate_limits(frame_id, min_range, max_range, vertical_min_angle, vertical_max_angle)
    if cloud.header.frame_id != frame_id:
        raise ValueError(f'Expected cloud frame {frame_id!r}, received {cloud.header.frame_id!r}')
    if cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('Invalid point_step or row_step')
    if len(cloud.data) < cloud.height * cloud.row_step:
        raise ValueError('PointCloud2 data is shorter than height * row_step')

    byte_order = '>' if cloud.is_bigendian else '<'
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        fields = [field for field in cloud.fields if field.name == name]
        if len(fields) != 1:
            raise ValueError(f'Expected exactly one {name!r} coordinate field')
        field = fields[0]
        if field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError(f'{name!r} must be a scalar FLOAT32 or FLOAT64')
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset + size > cloud.point_step:
            raise ValueError(f'{name!r} extends past point_step')
        formats.append(byte_order + ('f4' if size == 4 else 'f8'))
        offsets.append(field.offset)

    output = PointCloud2(
        header=deepcopy(cloud.header),
        height=1,
        width=0,
        fields=deepcopy(cloud.fields),
        is_bigendian=cloud.is_bigendian,
        point_step=cloud.point_step,
        row_step=0,
        is_dense=True,
    )
    if cloud.width == 0 or cloud.height == 0:
        return output

    xyz_dtype = np.dtype({
        'names': ['x', 'y', 'z'], 'formats': formats,
        'offsets': offsets, 'itemsize': cloud.point_step,
    })
    coordinates = np.ndarray(
        (cloud.height, cloud.width), dtype=xyz_dtype, buffer=cloud.data,
        strides=(cloud.row_step, cloud.point_step),
    )
    x, y, z = (coordinates[name].astype(np.float64, copy=False) for name in ('x', 'y', 'z'))
    with np.errstate(invalid='ignore', over='ignore'):
        horizontal_range = np.hypot(x, y)
        distance = np.hypot(horizontal_range, z)
        elevation = np.arctan2(z, horizontal_range)
    selected = (
        np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        & (distance >= min_range) & (distance <= max_range)
        & (elevation >= vertical_min_angle - ANGLE_TOLERANCE)
        & (elevation <= vertical_max_angle + ANGLE_TOLERANCE)
    )
    records = np.ndarray(
        (cloud.height, cloud.width, cloud.point_step), dtype=np.uint8,
        buffer=cloud.data, strides=(cloud.row_step, cloud.point_step, 1),
    )
    payload = array('B')
    payload.frombytes(records[selected].tobytes())
    output.width = int(np.count_nonzero(selected))
    output.row_step = output.width * output.point_step
    output.data = payload
    return output


class Mid360CloudFilter(Node):
    def __init__(self):
        super().__init__('mid360_cloud_filter')
        defaults = {
            'raw_topic': '/mid360/rgl_raw',
            'points_topic': '/mid360/points',
            'frame_id': SENSOR_FRAME,
            'min_range': 0.1,
            'max_range': 40.0,
            'vertical_min_angle': math.radians(-7.0),
            'vertical_max_angle': math.radians(52.0),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._filter_kwargs = {
            name: self.get_parameter(name).value
            for name in ('frame_id', 'min_range', 'max_range', 'vertical_min_angle', 'vertical_max_angle')
        }
        _validate_limits(**self._filter_kwargs)
        raw_topic = self.get_parameter('raw_topic').value
        points_topic = self.get_parameter('points_topic').value
        if not raw_topic or not points_topic or raw_topic == points_topic:
            raise ValueError('raw_topic and points_topic must be distinct nonempty topics')
        self._publisher = self.create_publisher(PointCloud2, points_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2, raw_topic, self._on_cloud, qos_profile_sensor_data,
        )

    def _on_cloud(self, cloud):
        try:
            output = normalize_cloud(cloud, **self._filter_kwargs)
        except ValueError as error:
            self.get_logger().warning(f'Dropping RGL cloud: {error}', throttle_duration_sec=5.0)
            return
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mid360CloudFilter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
