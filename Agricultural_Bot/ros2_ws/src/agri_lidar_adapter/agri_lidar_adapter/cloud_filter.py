"""
Transform and filter the MID-360 cloud for a planar navigation scan.

The node intentionally stops at a filtered PointCloud2.  Projection to
LaserScan is delegated to the maintained Jazzy ``pointcloud_to_laserscan``
package, so the same adapter can feed SLAM Toolbox, AMCL, and Nav2 without a
second project-specific scan implementation.
"""

from array import array
from copy import deepcopy
import math

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_INPUT = '/mid360/points'
DEFAULT_OUTPUT = '/mid360/navigation_points'
DEFAULT_SOURCE_FRAME = 'mid360_sensor_frame'
DEFAULT_TARGET_FRAME = 'mid360_scan_frame'
ANGLE_EPSILON = 1e-9
BOUNDARY_EPSILON = 1e-6


def _coordinate_dtype(cloud: PointCloud2) -> np.dtype:
    """Build an XYZ structured dtype respecting field offsets and endianness."""
    if cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('invalid PointCloud2 point_step or row_step')
    if len(cloud.data) < cloud.height * cloud.row_step:
        raise ValueError('PointCloud2 data is shorter than height * row_step')
    byte_order = '>' if cloud.is_bigendian else '<'
    names = []
    formats = []
    offsets = []
    for name in ('x', 'y', 'z'):
        matches = [field for field in cloud.fields if field.name == name]
        if len(matches) != 1:
            raise ValueError(f'expected exactly one {name!r} field')
        field = matches[0]
        if field.count != 1 or field.datatype not in (
                PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError(f'{name!r} must be a scalar FLOAT32 or FLOAT64')
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset < 0 or field.offset + size > cloud.point_step:
            raise ValueError(f'{name!r} extends past point_step')
        names.append(name)
        formats.append(byte_order + ('f4' if size == 4 else 'f8'))
        offsets.append(field.offset)
    return np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': cloud.point_step,
    })


def _records_and_coordinates(cloud: PointCloud2):
    """Return record bytes and coordinate arrays without assuming row packing."""
    dtype = _coordinate_dtype(cloud)
    if cloud.height == 0 or cloud.width == 0:
        shape = (cloud.height, cloud.width)
        empty = np.empty(shape, dtype=dtype)
        return (np.empty((cloud.height, cloud.width, cloud.point_step), dtype=np.uint8),
                empty)
    payload = memoryview(cloud.data)
    coordinates = np.ndarray(
        (cloud.height, cloud.width), dtype=dtype, buffer=payload,
        strides=(cloud.row_step, cloud.point_step),
    )
    records = np.ndarray(
        (cloud.height, cloud.width, cloud.point_step), dtype=np.uint8,
        buffer=payload,
        strides=(cloud.row_step, cloud.point_step, 1),
    )
    return records, coordinates


def _rotate_translate(xyz: np.ndarray, translation, quaternion) -> np.ndarray:
    """Apply a normalized ROS quaternion and translation to an ``N×3`` array."""
    tx, ty, tz = (float(value) for value in translation)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('transform quaternion is invalid')
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    # Matrix form avoids allocating one temporary 3x3 matrix per cloud.
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    result = np.empty_like(xyz, dtype=np.float64)
    result[:, 0] = ((1 - 2 * (yy + zz)) * x
                    + 2 * (xy - wz) * y
                    + 2 * (xz + wy) * z + tx)
    result[:, 1] = (2 * (xy + wz) * x
                    + (1 - 2 * (xx + zz)) * y
                    + 2 * (yz - wx) * z + ty)
    result[:, 2] = (2 * (xz - wy) * x
                    + 2 * (yz + wx) * y
                    + (1 - 2 * (xx + yy)) * z + tz)
    return result


def transform_from_message(transform: TransformStamped):
    """Extract numeric translation/quaternion tuples from a TF message."""
    t = transform.transform.translation
    q = transform.transform.rotation
    return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)


def _validate_options(options):
    numeric = (
        options.min_range, options.max_range, options.min_height,
        options.max_height, options.voxel_size,
        options.self_filter_x_min, options.self_filter_x_max,
        options.self_filter_y_min, options.self_filter_y_max,
        options.self_filter_z_min, options.self_filter_z_max,
    )
    if not all(math.isfinite(float(value)) for value in numeric):
        raise ValueError('filter bounds must be finite')
    if not 0 <= options.min_range < options.max_range:
        raise ValueError('expected 0 <= min_range < max_range')
    if not options.min_height < options.max_height:
        raise ValueError('expected min_height < max_height')
    if options.voxel_size < 0:
        raise ValueError('voxel_size must be non-negative')
    if not options.self_filter_x_min < options.self_filter_x_max:
        raise ValueError('self_filter_x_min must be below self_filter_x_max')
    if not options.self_filter_y_min < options.self_filter_y_max:
        raise ValueError('self_filter_y_min must be below self_filter_y_max')
    if not options.self_filter_z_min < options.self_filter_z_max:
        raise ValueError('self_filter_z_min must be below self_filter_z_max')


def filter_cloud(
    cloud: PointCloud2,
    *,
    target_frame: str,
    translation=(0.0, 0.0, 0.0),
    quaternion=(0.0, 0.0, 0.0, 1.0),
    min_range: float = 0.1,
    max_range: float = 40.0,
    min_height: float = -0.40,
    max_height: float = 0.40,
    self_filter_enabled: bool = False,
    self_filter_x_min: float = -0.75,
    self_filter_x_max: float = 0.55,
    self_filter_y_min: float = -0.55,
    self_filter_y_max: float = 0.55,
    self_filter_z_min: float = -0.55,
    self_filter_z_max: float = 0.45,
    voxel_size: float = 0.0,
) -> PointCloud2:
    """
    Transform and retain navigation-relevant point records.

    ``translation`` and ``quaternion`` describe target <- source.  The input
    record bytes are copied as a whole, then XYZ are replaced in the target
    frame; intensity/ring/time fields therefore remain available to later
    consumers.  Height, self-filter, and voxel bounds are all evaluated in the
    target frame.
    """
    if not target_frame:
        raise ValueError('target_frame must be non-empty')
    values = (
        float(min_range), float(max_range), float(min_height), float(max_height),
        float(self_filter_x_min), float(self_filter_x_max),
        float(self_filter_y_min), float(self_filter_y_max),
        float(self_filter_z_min), float(self_filter_z_max), float(voxel_size),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('filter bounds must be finite')
    if not 0 <= min_range < max_range:
        raise ValueError('expected 0 <= min_range < max_range')
    if not min_height < max_height:
        raise ValueError('expected min_height < max_height')
    if voxel_size < 0:
        raise ValueError('voxel_size must be non-negative')
    if not self_filter_x_min < self_filter_x_max:
        raise ValueError('self_filter_x_min must be below self_filter_x_max')
    if not self_filter_y_min < self_filter_y_max:
        raise ValueError('self_filter_y_min must be below self_filter_y_max')
    if not self_filter_z_min < self_filter_z_max:
        raise ValueError('self_filter_z_min must be below self_filter_z_max')

    records, coordinates = _records_and_coordinates(cloud)
    flat = np.column_stack((
        np.asarray(coordinates['x'], dtype=np.float64).reshape(-1),
        np.asarray(coordinates['y'], dtype=np.float64).reshape(-1),
        np.asarray(coordinates['z'], dtype=np.float64).reshape(-1),
    ))
    finite_input = np.isfinite(flat).all(axis=1)
    transformed = np.full(flat.shape, np.nan, dtype=np.float64)
    if np.any(finite_input):
        transformed[finite_input] = _rotate_translate(
            flat[finite_input], translation, quaternion)
    x, y, z = transformed[:, 0], transformed[:, 1], transformed[:, 2]
    with np.errstate(invalid='ignore', over='ignore'):
        distance = np.sqrt(x * x + y * y + z * z)
    selected = (
        np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        # PointCloud2 coordinates are commonly FLOAT32.  Include a small
        # tolerance so a configured boundary such as -0.40 m is not lost to
        # the one-ulp conversion from decimal configuration to binary data.
        & (distance >= min_range - BOUNDARY_EPSILON)
        & (distance <= max_range + BOUNDARY_EPSILON)
        & (z >= min_height - BOUNDARY_EPSILON)
        & (z <= max_height + BOUNDARY_EPSILON)
    )
    if self_filter_enabled:
        inside = (
            (x >= self_filter_x_min) & (x <= self_filter_x_max)
            & (y >= self_filter_y_min) & (y <= self_filter_y_max)
            & (z >= self_filter_z_min) & (z <= self_filter_z_max)
        )
        selected &= ~inside

    indices = np.flatnonzero(selected)
    if voxel_size > 0 and indices.size:
        keys = np.floor(transformed[indices] / voxel_size).astype(np.int64)
        # np.unique returns the first occurrence for each sorted key. Sort the
        # retained indices so output ordering remains the source scan order.
        _, first = np.unique(keys, axis=0, return_index=True)
        indices = np.sort(indices[first])

    selected_records = records.reshape(-1, cloud.point_step)[indices]
    payload = bytearray(selected_records.tobytes())
    if indices.size:
        output_coordinates = np.ndarray(
            (indices.size,), dtype=_coordinate_dtype(cloud), buffer=payload)
        output_coordinates['x'] = transformed[indices, 0]
        output_coordinates['y'] = transformed[indices, 1]
        output_coordinates['z'] = transformed[indices, 2]

    output = PointCloud2(
        header=deepcopy(cloud.header),
        height=1,
        width=int(indices.size),
        fields=deepcopy(cloud.fields),
        is_bigendian=cloud.is_bigendian,
        point_step=cloud.point_step,
        row_step=int(indices.size) * cloud.point_step,
        is_dense=True,
    )
    output.header.frame_id = target_frame
    output.data = array('B', payload)
    return output


class Mid360CloudFilter(Node):
    """ROS wrapper around :func:`filter_cloud` with timestamped TF lookup."""

    def __init__(self):
        super().__init__('mid360_navigation_cloud_filter')
        defaults = {
            'input_topic': DEFAULT_INPUT,
            'output_topic': DEFAULT_OUTPUT,
            'source_frame': DEFAULT_SOURCE_FRAME,
            'target_frame': DEFAULT_TARGET_FRAME,
            'min_range': 0.1,
            'max_range': 40.0,
            'min_height': -0.40,
            'max_height': 0.40,
            'self_filter_enabled': False,
            'self_filter_x_min': -0.75,
            'self_filter_x_max': 0.55,
            'self_filter_y_min': -0.55,
            'self_filter_y_max': 0.55,
            'self_filter_z_min': -0.55,
            'self_filter_z_max': 0.45,
            'voxel_size': 0.0,
            'tf_timeout': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.source_frame = str(self.get_parameter('source_frame').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self.self_filter_x_min = float(self.get_parameter('self_filter_x_min').value)
        self.self_filter_x_max = float(self.get_parameter('self_filter_x_max').value)
        self.self_filter_y_min = float(self.get_parameter('self_filter_y_min').value)
        self.self_filter_y_max = float(self.get_parameter('self_filter_y_max').value)
        self.self_filter_z_min = float(self.get_parameter('self_filter_z_min').value)
        self.self_filter_z_max = float(self.get_parameter('self_filter_z_max').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        if not self.input_topic or not self.output_topic or self.input_topic == self.output_topic:
            raise ValueError('input_topic and output_topic must be distinct non-empty topics')
        if not self.source_frame or not self.target_frame:
            raise ValueError('source_frame and target_frame must be non-empty')
        if self.tf_timeout <= 0 or not math.isfinite(self.tf_timeout):
            raise ValueError('tf_timeout must be positive and finite')

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, qos_profile_sensor_data)
        self._dropped_tf = 0

    def _lookup(self, cloud):
        source = cloud.header.frame_id or self.source_frame
        if source != self.source_frame:
            raise ValueError(
                f'expected cloud frame {self.source_frame!r}, received {source!r}')
        if source == self.target_frame:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        stamp = Time.from_msg(cloud.header.stamp)
        transform = self._tf_buffer.lookup_transform(
            self.target_frame, source, stamp,
            timeout=Duration(seconds=self.tf_timeout))
        return transform_from_message(transform)

    def _on_cloud(self, cloud):
        try:
            translation, quaternion = self._lookup(cloud)
            output = filter_cloud(
                cloud,
                target_frame=self.target_frame,
                translation=translation,
                quaternion=quaternion,
                min_range=self.min_range,
                max_range=self.max_range,
                min_height=self.min_height,
                max_height=self.max_height,
                self_filter_enabled=self.self_filter_enabled,
                self_filter_x_min=self.self_filter_x_min,
                self_filter_x_max=self.self_filter_x_max,
                self_filter_y_min=self.self_filter_y_min,
                self_filter_y_max=self.self_filter_y_max,
                self_filter_z_min=self.self_filter_z_min,
                self_filter_z_max=self.self_filter_z_max,
                voxel_size=self.voxel_size,
            )
        except (ValueError, TransformException) as error:
            self._dropped_tf += 1
            self.get_logger().warning(
                f'Dropping MID-360 cloud ({self._dropped_tf} total): {error}',
                throttle_duration_sec=5.0)
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
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # rclpy may deliver SIGINT while a service is being torn
                # down; shutdown remains successful even if destruction is
                # interrupted at that exact boundary.
                pass
        if rclpy.ok():
            rclpy.shutdown()
