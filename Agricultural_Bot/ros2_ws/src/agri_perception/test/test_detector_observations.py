"""Exercise the real ROS node/message/TF boundary with synthetic RGB-D input.

Only model inference is stubbed: these cases do not validate trained accuracy.
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('agri_harvest_interfaces.msg')
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

from agri_perception import tomato_detector


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def node(monkeypatch):
    result = SimpleNamespace(
        boxes=SimpleNamespace(xyxy=np.asarray([[17, 17, 47, 47]], dtype=float),
                              cls=np.asarray([0]), conf=np.asarray([0.9])),
        names={0: 'tomato_R', 1: 'tomato_G'},
        masks=SimpleNamespace(xy=[np.asarray([[17, 17], [47, 17], [47, 47], [17, 47]])]),
        plot=lambda: np.zeros((64, 64, 3), dtype=np.uint8),
    )
    # Provide len(boxes), as Ultralytics does, while retaining mutable classes.
    class _Boxes(SimpleNamespace):
        def __len__(self):
            return len(self.cls)
    result.boxes = _Boxes(**result.boxes.__dict__)
    model = SimpleNamespace(predict=lambda **kwargs: [result])
    monkeypatch.setattr(tomato_detector, 'YOLO', lambda _: model)
    rclpy.init(args=['--ros-args', '-p', 'model_path:=synthetic', '-p', 'use_sim_time:=true',
                    '-p', 'device:=cpu'])
    detector = tomato_detector.TomatoDetector()
    for name in ('_targets_pub', '_base_targets_pub', '_positions_pub',
                 '_base_positions_pub', '_annotated_pub', '_detections_pub'):
        setattr(detector, name, _Publisher())
    info = CameraInfo()
    info.width = info.height = 64
    info.header.frame_id = 'camera_optical_frame'
    info.k = [100.0, 0.0, 32.0, 0.0, 100.0, 32.0, 0.0, 0.0, 1.0]
    detector._camera_info_callback(info)
    detector._synthetic_result = result
    try:
        yield detector
    finally:
        detector.destroy_node()
        rclpy.shutdown()


def _images(node, stamp_s, depth_s=None):
    depth_s = stamp_s if depth_s is None else depth_s
    rgb = node._bridge.cv2_to_imgmsg(np.zeros((64, 64, 3), np.uint8), 'bgr8')
    depth = node._bridge.cv2_to_imgmsg(np.full((64, 64), 0.8, np.float32), '32FC1')
    for image, stamp in ((rgb, stamp_s), (depth, depth_s)):
        image.header.frame_id = 'camera_optical_frame'
        image.header.stamp = Time(seconds=stamp).to_msg()
    node.get_clock().set_ros_time_override(Time(seconds=max(stamp_s, depth_s)+0.01))
    return rgb, depth


def _transform(node, stamp_s, x):
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_optical_frame'
    transform.header.stamp = Time(seconds=stamp_s).to_msg()
    transform.transform.translation.x = float(x)
    transform.transform.rotation.w = 1.0
    node._tf_buffer.set_transform(transform, 'synthetic-test')


def test_capture_time_tf_uses_historical_transform_and_preserves_fields(node):
    _transform(node, 1.0, 1.0)
    _transform(node, 2.0, 9.0)
    rgb, depth = _images(node, 1.0)
    node._process_pair(rgb, depth)
    camera = node._targets_pub.messages[-1]
    base = node._base_targets_pub.messages[-1]
    assert camera.observation_valid and base.observation_valid
    assert base.targets[0].position.x == pytest.approx(1.0)
    assert base.targets[0].position.z == pytest.approx(0.8)
    assert base.targets[0].target_id == camera.targets[0].target_id
    assert base.targets[0].class_name == 'tomato_R'
    assert base.targets[0].confidence == pytest.approx(0.9)
    assert base.targets[0].segmentation_used
    assert base.header.stamp == rgb.header.stamp
    assert base.depth_stamp == depth.header.stamp
    assert not base.targets[0].temporally_stable


def test_maturity_stability_and_empty_frame_clear_previous_outputs(node):
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_optical_frame'
    transform.transform.rotation.w = 1.0
    node._tf_buffer.set_transform_static(transform, 'synthetic-test')
    for stamp in (1.0, 1.1, 1.2):
        node._process_pair(*_images(node, stamp))
    target = node._base_targets_pub.messages[-1].targets[0]
    assert target.temporally_stable and target.stable_frame_count == 3
    node._synthetic_result.boxes.cls = np.asarray([1])
    node._process_pair(*_images(node, 1.3))
    assert node._targets_pub.messages[-1].observation_valid
    assert node._targets_pub.messages[-1].targets == []
    assert node._base_positions_pub.messages[-1].poses == []
    assert node._detections_pub.messages[-1].detections[0].results[0].hypothesis.class_id == 'tomato_G'


def test_skew_expiration_and_tf_failure_publish_invalid_empty_arrays(node):
    node._process_pair(*_images(node, 1.0))
    assert node._targets_pub.messages[-1].targets
    assert not node._base_targets_pub.messages[-1].observation_valid
    assert node._base_targets_pub.messages[-1].targets == []
    assert node._base_targets_pub.messages[-1].rejection_reason.startswith('capture_time_tf_unavailable')
    node._process_pair(*_images(node, 1.1, 1.2))
    assert node._targets_pub.messages[-1].rejection_reason == 'rgb_depth_timestamp_skew'
    assert node._positions_pub.messages[-1].poses == []
    node._process_pair(*_images(node, 1.3))
    node.get_clock().set_ros_time_override(Time(seconds=2.0))
    node._watchdog_callback()
    assert node._targets_pub.messages[-1].rejection_reason == 'expired_observation'
    assert node._targets_pub.messages[-1].targets == []
    assert node._base_targets_pub.messages[-1].targets == []


def test_invalid_depth_and_calibration_frame_fail_closed(node):
    rgb, depth = _images(node, 1.0)
    depth = node._bridge.cv2_to_imgmsg(np.zeros((64, 64), np.uint16), '16UC1')
    depth.header = rgb.header
    node._process_pair(rgb, depth)
    assert node._targets_pub.messages[-1].rejection_reason == 'no_valid_depth_targets: insufficient_valid_depth'
    rgb, depth = _images(node, 1.1)
    depth.header.frame_id = 'wrong_optical_frame'
    node._process_pair(rgb, depth)
    assert node._targets_pub.messages[-1].rejection_reason == 'rgb_depth_optical_frame_mismatch'
    assert node._targets_pub.messages[-1].targets == []


def test_wall_watchdog_clears_targets_when_sim_clock_paused(node):
    node._process_pair(*_images(node, 1.0))
    node._last_pair_wall_ns -= 2_000_000_000
    node._watchdog_callback()
    assert node._targets_pub.messages[-1].rejection_reason == 'rgb_depth_stream_timeout'
    assert node._targets_pub.messages[-1].targets == []
