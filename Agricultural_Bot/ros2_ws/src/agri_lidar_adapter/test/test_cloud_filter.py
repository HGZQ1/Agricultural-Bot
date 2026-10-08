import math
import struct

from agri_lidar_adapter.cloud_filter import filter_cloud
from builtin_interfaces.msg import Time
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def make_cloud(points, *, frame='mid360_sensor_frame', bigendian=False,
               point_step=16, padding=0):
    fields = [
        PointField(name='x', offset=0,
                   datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,
                   datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,
                   datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12,
                   datatype=PointField.FLOAT32, count=1),
    ]
    endian = '>' if bigendian else '<'
    data = bytearray()
    for index, point in enumerate(points):
        data.extend(struct.pack(endian + 'ffff', *point))
    row_step = len(points) * point_step + padding
    if padding:
        data.extend(b'\xA5' * padding)
    return PointCloud2(
        header=Header(stamp=Time(sec=3), frame_id=frame),
        height=1, width=len(points), fields=fields,
        is_bigendian=bigendian, point_step=point_step, row_step=row_step,
        is_dense=False, data=data,
    )


def read_xyz(cloud):
    endian = '>' if cloud.is_bigendian else '<'
    return [struct.unpack_from(endian + 'fff', cloud.data, i * cloud.point_step)
            for i in range(cloud.width)]


def test_transform_filter_preserves_origin_stamp_and_fields():
    cloud = make_cloud([(1.0, 0.0, 0.0, 7.0), (1.0, 0.0, 1.0, 8.0)])
    result = filter_cloud(
        cloud, target_frame='mid360_scan_frame',
        translation=(1.0, 2.0, 0.0),
        quaternion=(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)),
        min_height=-1.0, max_height=2.0)
    assert result.header.frame_id == 'mid360_scan_frame'
    assert result.header.stamp.sec == 3
    assert [field.name for field in result.fields] == [
        'x', 'y', 'z', 'intensity']
    assert read_xyz(result) == pytest.approx([(1.0, 3.0, 0.0), (1.0, 3.0, 1.0)])
    assert result.width == 2 and result.height == 1


def test_height_range_and_self_filter_are_applied_in_target_frame():
    cloud = make_cloud([
        (1.0, 0.0, -0.4, 1.0),  # boundary retained
        (1.0, 0.0, 0.41, 2.0),  # above slice
        (0.05, 0.0, 0.0, 3.0),  # below range
        (0.5, 0.0, 0.0, 4.0),   # self box
        (2.0, 0.0, 0.0, 5.0),   # retained
    ])
    result = filter_cloud(
        cloud, target_frame='mid360_scan_frame', min_range=0.1,
        max_range=3.0, min_height=-0.4, max_height=0.4,
        self_filter_enabled=True, self_filter_x_min=-0.1,
        self_filter_x_max=0.8, self_filter_y_min=-0.2,
        self_filter_y_max=0.2, self_filter_z_min=-0.5,
        self_filter_z_max=0.5)
    actual = read_xyz(result)
    assert len(actual) == 2
    assert actual[0] == pytest.approx((1.0, 0.0, -0.4), abs=2e-6)
    assert actual[1] == pytest.approx((2.0, 0.0, 0.0), abs=2e-6)


def test_voxel_filter_is_deterministic_and_keeps_source_order():
    cloud = make_cloud([
        (1.01, 0.01, 0.0, 1.0),
        (1.02, 0.02, 0.0, 2.0),
        (1.11, 0.01, 0.0, 3.0),
    ])
    result = filter_cloud(
        cloud, target_frame='mid360_scan_frame', min_height=-1.0,
        max_height=1.0, voxel_size=0.1)
    assert result.width == 2
    actual = read_xyz(result)
    expected = [(1.01, 0.01, 0.0), (1.11, 0.01, 0.0)]
    assert len(actual) == len(expected)
    for row, target in zip(actual, expected):
        assert row == pytest.approx(target, abs=2e-6)


def test_big_endian_and_nan_records_are_supported():
    cloud = make_cloud([
        (1.0, 2.0, 0.0, 4.0),
        (math.nan, 0.0, 0.0, 5.0),
    ], bigendian=True)
    result = filter_cloud(
        cloud, target_frame='mid360_scan_frame', min_height=-1.0, max_height=1.0)
    assert result.width == 1
    assert read_xyz(result) == pytest.approx([(1.0, 2.0, 0.0)])
    assert result.is_bigendian is True


@pytest.mark.parametrize('kwargs', [
    {'min_height': 1.0, 'max_height': 1.0},
    {'min_range': 2.0, 'max_range': 1.0},
    {'voxel_size': -0.1},
])
def test_invalid_filter_bounds_are_rejected(kwargs):
    cloud = make_cloud([(1.0, 0.0, 0.0, 1.0)])
    with pytest.raises(ValueError):
        filter_cloud(cloud, target_frame='mid360_scan_frame', **kwargs)
