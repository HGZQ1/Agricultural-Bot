"""Validate output geometry and preservation of real PointCloud2 layouts."""

from array import array
import math
import struct

import pytest
from sensor_msgs.msg import PointCloud2, PointField

from agri_sim_sensors.normalize_cloud import normalize_cloud


def _field(name, offset, datatype, count=1):
    return PointField(name=name, offset=offset, datatype=datatype, count=count)


def _cloud(records, *, fields=None, point_step=32, width=None, row_padding=b'', bigendian=False):
    width = len(records) if width is None else width
    height = 1 if width == 0 else len(records) // width
    payload = b''.join(
        b''.join(records[row * width:(row + 1) * width]) + row_padding
        for row in range(height)
    )
    msg = PointCloud2(
        height=height, width=width, point_step=point_step,
        row_step=width * point_step + len(row_padding),
        fields=fields or [
            _field('x', 0, PointField.FLOAT64), _field('y', 8, PointField.FLOAT64),
            _field('z', 16, PointField.FLOAT64), _field('intensity', 24, PointField.FLOAT32),
        ],
        is_bigendian=bigendian, data=array('B', payload), is_dense=False,
    )
    msg.header.frame_id = 'mid360_sensor_frame'
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    return msg


def _polar(distance, angle):
    return distance * math.cos(angle), 0.0, distance * math.sin(angle)


def _record(xyz, number):
    return struct.pack('<dddf4s', *xyz, number + 0.5, number.to_bytes(4, 'little'))


def test_geometric_limits_are_inclusive_and_nonfinite_xyz_is_removed():
    minimum, maximum = math.radians(-7), math.radians(52)
    positions = [
        (0.1, 0.0, 0.0), (40.0, 0.0, 0.0),
        _polar(2.0, minimum), _polar(2.0, maximum),
        _polar(2.0, minimum + 0.01), _polar(2.0, maximum - 0.01),
        (0.099, 0.0, 0.0), (40.01, 0.0, 0.0),
        _polar(2.0, minimum - 5e-6), _polar(2.0, maximum + 5e-6),
        (math.nan, 0.0, 0.0), (1.0, math.nan, 0.0), (1.0, 0.0, math.nan),
        (math.inf, 0.0, 0.0), (1.0, 0.0, -math.inf),
    ]
    records = [_record(xyz, index) for index, xyz in enumerate(positions)]
    result = normalize_cloud(_cloud(records))
    assert result.width == 6
    assert bytes(result.data) == b''.join(records[:6])
    assert result.height == 1 and result.row_step == 6 * 32 and result.is_dense


def test_float32_roundoff_at_vertical_limits_has_only_small_tolerance():
    minimum = math.radians(-7)
    maximum = math.radians(52)
    angles = [minimum, maximum, minimum - 0.5e-6, maximum + 0.5e-6,
              minimum - 2e-6, maximum + 2e-6]
    records = [struct.pack('<ffff', *_polar(2, angle), index) for index, angle in enumerate(angles)]
    fields = [_field(name, offset, PointField.FLOAT32)
              for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]]
    result = normalize_cloud(_cloud(records, fields=fields, point_step=16))
    assert result.width == 4
    assert bytes(result.data) == b''.join(records[:4])


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_row_padding_mixed_offsets_extra_fields_and_point_padding_survive(bigendian):
    order = '>' if bigendian else '<'
    fields = [
        _field('ring', 6, PointField.UINT16), _field('z', 20, PointField.FLOAT32),
        _field('intensity', 4, PointField.UINT16), _field('x', 8, PointField.FLOAT64),
        _field('y', 0, PointField.FLOAT32), _field('time', 16, PointField.FLOAT32),
    ]
    positions = [(1.0, 2.0, 0.0), (0.01, 0.0, 0.0),
                 (-3.0, 0.0, 1.0), (1.0, 0.0, 5.0)]
    records = []
    for index, (x, y, z) in enumerate(positions):
        record = bytearray(bytes([0xA0 + index]) * 32)
        struct.pack_into(order + 'fHHdff', record, 0, y, 123 + index, 11 + index, x, index / 10, z)
        records.append(bytes(record))
    cloud = _cloud(records, fields=fields, width=2, row_padding=b'ROWPAD!', bigendian=bigendian)
    before = bytes(cloud.data)
    result = normalize_cloud(cloud)
    assert result.width == 2 and result.height == 1 and result.row_step == 64
    assert bytes(result.data) == records[0] + records[2]
    assert result.fields == fields and result.is_bigendian == bigendian
    assert result.header == cloud.header and result.header is not cloud.header
    assert result.point_step == 32 and result.is_dense
    assert bytes(cloud.data) == before and not cloud.is_dense


def test_an_empty_selection_retains_the_message_contract():
    cloud = _cloud([_record((0.01, 0.0, 0.0), 7)])
    result = normalize_cloud(cloud)
    assert (result.height, result.width, result.row_step) == (1, 0, 0)
    assert bytes(result.data) == b''
    assert result.point_step == cloud.point_step and result.fields == cloud.fields
    assert result.header == cloud.header and result.is_dense
    empty = normalize_cloud(_cloud([]))
    assert empty.height == 1 and empty.width == 0 and bytes(empty.data) == b''


def test_a_different_frame_is_rejected_instead_of_relabelled():
    cloud = _cloud([_record((1.0, 0.0, 0.0), 0)])
    cloud.header.frame_id = 'base_link'
    with pytest.raises(ValueError, match='received.*base_link'):
        normalize_cloud(cloud)
    assert cloud.header.frame_id == 'base_link'


@pytest.mark.parametrize('problem', ['truncated', 'row_step', 'missing_x', 'integer_x', 'array_x'])
def test_invalid_cloud_layout_fails_clearly(problem):
    cloud = _cloud([_record((1.0, 0.0, 0.0), 0)])
    if problem == 'truncated':
        cloud.data = cloud.data[:-1]
    elif problem == 'row_step':
        cloud.row_step = 31
    elif problem == 'missing_x':
        cloud.fields = cloud.fields[1:]
    elif problem == 'integer_x':
        cloud.fields[0].datatype = PointField.INT32
    else:
        cloud.fields[0].count = 2
    with pytest.raises(ValueError):
        normalize_cloud(cloud)


@pytest.mark.parametrize('limits', [
    {'min_range': 40.0, 'max_range': 0.1}, {'min_range': math.nan},
    {'vertical_min_angle': 1.0, 'vertical_max_angle': 0.0}, {'frame_id': 'base_link'},
])
def test_invalid_filter_parameters_are_rejected(limits):
    with pytest.raises(ValueError):
        normalize_cloud(_cloud([_record((1.0, 0.0, 0.0), 0)]), **limits)
