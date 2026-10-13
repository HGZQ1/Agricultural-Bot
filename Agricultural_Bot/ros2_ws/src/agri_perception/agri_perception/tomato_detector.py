"""Publish checked, capture-time RGB-D tomato surface observations."""

from __future__ import annotations

from copy import deepcopy
import math
import uuid
from typing import Any

from agri_harvest_interfaces.msg import TomatoTarget, TomatoTargetArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from agri_perception.target_quality import (
    DepthConfig, ImagePairBuffer, TargetTracker,
    observation_time_error, project_candidates,
)


def _stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


class TomatoDetector(Node):
    """The detector reports points and quality; it never commands the robot."""

    def __init__(self) -> None:
        super().__init__('tomato_detector')
        defaults = {
            'model_path': '', 'image_topic': '/d405/color/image_raw',
            'depth_topic': '/d405/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/d405/color/camera_info',
            'annotated_image_topic': '/agri_vision/annotated_image',
            'detections_topic': '/agri_vision/detections',
            'positions_topic': '/agri_vision/tomato_positions_camera',
            'base_positions_topic': '/agri_vision/tomato_positions_base',
            'targets_topic': '/agri_vision/tomato_targets_camera',
            'base_targets_topic': '/agri_vision/tomato_targets_base',
            'target_frame': 'base_link', 'confidence_threshold': 0.25,
            'image_size': 640, 'device': 0,
            'mature_only': True, 'mature_class': 'tomato_R',
            'depth_sample_count': 81, 'depth_sample_radius_px': 8.0,
            'depth_bbox_radius_fraction': 0.25, 'depth_sampling_seed': 42,
            'depth_min_m': 0.07, 'depth_max_m': 2.0,
            'depth_min_valid_samples': 15, 'depth_min_valid_fraction': 0.6,
            'depth_max_mad_m': 0.03, 'require_segmentation': False,
            'segmentation_erode_px': 1,
            'max_rgb_depth_skew_s': 0.05, 'max_observation_age_s': 0.5,
            'image_pair_queue_size': 10, 'stream_timeout_wall_s': 1.0,
            'stability_min_frames': 3, 'stability_window_s': 0.5,
            'stability_min_span_s': 0.1, 'stability_max_spread_m': 0.025,
            'track_match_distance_m': 0.08,
        }
        for name, value in defaults.items():
            if name == 'device':
                self.declare_parameter(name, value, ParameterDescriptor(dynamic_typing=True))
            else:
                self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        model_path = str(p('model_path'))
        if not model_path:
            raise ValueError('model_path is empty; pass the path to trained best.pt')
        self._confidence = float(p('confidence_threshold'))
        self._image_size = int(p('image_size'))
        device = p('device')
        self._device = int(device) if isinstance(device, int) else str(device)
        if not 0 < self._confidence <= 1 or self._image_size <= 0:
            raise ValueError('invalid inference confidence/image size')
        self._mature_only, self._mature_class = bool(p('mature_only')), str(p('mature_class'))
        self._seed = int(p('depth_sampling_seed'))
        self._erode_px = int(p('segmentation_erode_px'))
        self._max_skew_s = float(p('max_rgb_depth_skew_s'))
        self._max_age_s = float(p('max_observation_age_s'))
        self._wall_timeout_s = float(p('stream_timeout_wall_s'))
        if (self._seed < 0 or self._erode_px < 0
                or not 0 < self._wall_timeout_s < math.inf):
            raise ValueError('invalid sampling seed/erosion/stream timeout')
        self._depth_config = DepthConfig(
            sample_count=int(p('depth_sample_count')),
            radius_px=float(p('depth_sample_radius_px')),
            bbox_radius_fraction=float(p('depth_bbox_radius_fraction')),
            minimum_m=float(p('depth_min_m')), maximum_m=float(p('depth_max_m')),
            minimum_samples=int(p('depth_min_valid_samples')),
            minimum_valid_fraction=float(p('depth_min_valid_fraction')),
            maximum_mad_m=float(p('depth_max_mad_m')),
            require_segmentation=bool(p('require_segmentation')),
        )
        self._pair_buffer = ImagePairBuffer(self._max_skew_s, self._max_age_s,
                                           int(p('image_pair_queue_size')))
        self._tracker = TargetTracker(
            uuid.uuid4().hex[:12], int(p('stability_min_frames')),
            float(p('stability_window_s')), float(p('stability_max_spread_m')),
            float(p('track_match_distance_m')), float(p('stability_min_span_s')),
        )
        self._target_frame = str(p('target_frame'))
        if not self._target_frame or not self._mature_class:
            raise ValueError('target_frame and mature_class must be nonempty')
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self._model = YOLO(model_path)
        self._bridge = CvBridge()
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._camera_info: CameraInfo | None = None
        self._frame_count = 0
        self._last_header = Header()
        self._last_depth_stamp = None
        self._tracking_frame = ''
        self._last_observation_ns = 0
        self._last_ros_now = self.get_clock().now().nanoseconds
        # The steady watchdog clears targets even while /clock is paused.
        # Synchronization, capture age and TF still use the node ROS clock.
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_pair_wall_ns = self._watchdog_clock.now().nanoseconds
        self._last_rejection = ''
        self._annotated_pub = self.create_publisher(Image, str(p('annotated_image_topic')), 10)
        self._detections_pub = self.create_publisher(Detection2DArray, str(p('detections_topic')), 10)
        self._positions_pub = self.create_publisher(PoseArray, str(p('positions_topic')), 10)
        self._base_positions_pub = self.create_publisher(PoseArray, str(p('base_positions_topic')), 10)
        self._targets_pub = self.create_publisher(TomatoTargetArray, str(p('targets_topic')), 10)
        self._base_targets_pub = self.create_publisher(TomatoTargetArray, str(p('base_targets_topic')), 10)
        self._image_sub = self.create_subscription(Image, str(p('image_topic')), self._image_callback,
                                                   qos_profile_sensor_data)
        self._depth_sub = self.create_subscription(Image, str(p('depth_topic')), self._depth_callback,
                                                   qos_profile_sensor_data)
        self._info_sub = self.create_subscription(CameraInfo, str(p('camera_info_topic')),
                                                  self._camera_info_callback, qos_profile_sensor_data)
        self._watchdog = self.create_timer(0.1, self._watchdog_callback,
                                           clock=self._watchdog_clock)
        self.get_logger().info(
            f'Mature-only 3D: {self._mature_only} ({self._mature_class}); '
            f'capture-time TF target: {self._target_frame}; '
            'surface positions are not grasp poses')

    def _camera_info_callback(self, message):
        self._camera_info = message

    def _image_callback(self, message):
        self._receive('rgb', message)

    def _depth_callback(self, message):
        self._receive('depth', message)

    def _receive(self, kind, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._last_ros_now:
            self._pair_buffer.clear()
            self._tracker.clear()
            self._last_observation_ns = 0
            self._publish_invalid('clock_jump_back')
        self._last_ros_now = now_ns
        stamp_ns = _stamp_ns(message.header.stamp)
        reason = observation_time_error(stamp_ns, stamp_ns, now_ns,
                                        self._max_skew_s, self._max_age_s)
        if reason:
            self._publish_invalid(reason, message.header)
            return
        pair = self._pair_buffer.push(kind, stamp_ns, message, now_ns)
        if pair is not None:
            self._last_pair_wall_ns = self._watchdog_clock.now().nanoseconds
            self._process_pair(*pair)

    def _watchdog_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self._last_ros_now:
            self._pair_buffer.clear()
            self._tracker.clear()
            self._last_observation_ns = 0
            self._publish_invalid('clock_jump_back')
        self._last_ros_now = now_ns
        if (self._watchdog_clock.now().nanoseconds-self._last_pair_wall_ns
                > round(self._wall_timeout_s*1e9)):
            self._pair_buffer.clear()
            self._publish_invalid('rgb_depth_stream_timeout')
        elif (self._last_observation_ns
              and now_ns-self._last_observation_ns > round(self._max_age_s*1e9)):
            self._publish_invalid('expired_observation')

    def _process_pair(self, source, depth_message):
        self._last_header = deepcopy(source.header)
        self._last_depth_stamp = deepcopy(depth_message.header.stamp)
        reason = observation_time_error(_stamp_ns(source.header.stamp),
                                        _stamp_ns(depth_message.header.stamp),
                                        self.get_clock().now().nanoseconds,
                                        self._max_skew_s, self._max_age_s)
        if reason:
            self._publish_invalid(reason)
            return
        try:
            image = self._bridge.imgmsg_to_cv2(source, 'bgr8')
            result = self._model.predict(source=image, device=self._device,
                                         imgsz=self._image_size, conf=self._confidence,
                                         verbose=False)[0]
            annotated = self._bridge.cv2_to_imgmsg(result.plot(), 'bgr8')
            annotated.header = source.header
            self._annotated_pub.publish(annotated)
            self._detections_pub.publish(self._detection_message(result, source))
            camera_targets = self._target_message(result, source, depth_message)
            self._publish_camera(camera_targets)
            self._publish_base(self._base_target_message(camera_targets))
            self._last_observation_ns = min(_stamp_ns(source.header.stamp),
                                            _stamp_ns(depth_message.header.stamp))
            self._frame_count += 1
            if self._frame_count == 1 or self._frame_count % 30 == 0:
                self.get_logger().info(
                    f'Processed frame {self._frame_count}: {len(result.boxes)} detections, '
                    f'{len(camera_targets.targets)} checked 3D targets')
        except (CvBridgeError, RuntimeError, ValueError, IndexError, KeyError) as error:
            self._publish_invalid(f'vision_processing_failed: {error}')

    def _empty_array(self, header=None, reason=''):
        output = TomatoTargetArray()
        output.header = deepcopy(header if header is not None else self._last_header)
        if self._last_depth_stamp is not None:
            output.depth_stamp = deepcopy(self._last_depth_stamp)
        output.observation_valid = not bool(reason)
        output.rejection_reason = reason
        return output

    def _publish_invalid(self, reason, header=None):
        self._tracker.clear()
        output = self._empty_array(header, reason)
        self._publish_camera(output)
        base = deepcopy(output)
        base.header.frame_id = self._target_frame
        self._publish_base(base)
        if reason != self._last_rejection:
            self.get_logger().warning(f'3D observation rejected: {reason}')
            self._last_rejection = reason

    @staticmethod
    def _legacy_positions(targets):
        output = PoseArray()
        output.header = deepcopy(targets.header)
        for target in targets.targets:
            pose = Pose()
            pose.position = deepcopy(target.position)
            pose.orientation.w = 1.0  # Compatibility placeholder, not a grasp orientation.
            output.poses.append(pose)
        return output

    def _publish_camera(self, output):
        self._targets_pub.publish(output)
        self._positions_pub.publish(self._legacy_positions(output))

    def _publish_base(self, output):
        self._base_targets_pub.publish(output)
        self._base_positions_pub.publish(self._legacy_positions(output))

    def _segmentation_masks(self, result, shape):
        masks = getattr(result, 'masks', None)
        if masks is None:
            return None
        output = []
        # masks.xy are polygons in the original RGB image, unlike possibly
        # resized/letterboxed masks.data arrays.
        for polygon in masks.xy:
            mask = np.zeros(shape, dtype=np.uint8)
            points = np.asarray(polygon)
            if len(points) >= 3 and np.isfinite(points).all():
                cv2.fillPoly(mask, [np.round(points).astype(np.int32)], 1)
            if self._erode_px:
                size = self._erode_px*2+1
                mask = cv2.erode(mask, np.ones((size, size), dtype=np.uint8))
            output.append(mask.astype(bool))
        return output

    def _target_message(self, result: Any, source: Image, depth_message: Image):
        info = self._camera_info
        if info is None:
            self._tracker.clear()
            return self._empty_array(reason='camera_info_unavailable')
        frame = info.header.frame_id
        if not frame or frame != source.header.frame_id or frame != depth_message.header.frame_id:
            self._tracker.clear()
            return self._empty_array(reason='rgb_depth_optical_frame_mismatch')
        if (source.width != depth_message.width or source.height != depth_message.height
                or info.width != source.width or info.height != source.height):
            self._tracker.clear()
            return self._empty_array(reason='rgb_depth_calibration_size_mismatch')
        if depth_message.encoding not in ('16UC1', '32FC1'):
            self._tracker.clear()
            return self._empty_array(reason='unsupported_depth_encoding')
        depth = np.asarray(self._bridge.imgmsg_to_cv2(depth_message, 'passthrough'))
        if depth.ndim != 2 or depth.shape != (source.height, source.width):
            self._tracker.clear()
            return self._empty_array(reason='invalid_depth_shape')
        depth = depth.astype(np.float32)
        if depth_message.encoding == '16UC1':
            depth /= 1000.0
        candidates, rejected = project_candidates(
            result.boxes.xyxy.tolist(), result.boxes.cls.tolist(), result.boxes.conf.tolist(),
            result.names, depth, (info.k[0], info.k[4], info.k[2], info.k[5]),
            self._seed+self._frame_count, self._depth_config,
            self._segmentation_masks(result, depth.shape),
            self._mature_only, self._mature_class,
        )
        reason = observation_time_error(_stamp_ns(source.header.stamp),
                                        _stamp_ns(depth_message.header.stamp),
                                        self.get_clock().now().nanoseconds,
                                        self._max_skew_s, self._max_age_s)
        if reason or (rejected and not candidates):
            self._tracker.clear()
            return self._empty_array(reason=reason or 'no_valid_depth_targets: '+rejected[0][1])
        if frame != self._tracking_frame:
            self._tracker.clear()
            self._tracking_frame = frame
        output = self._empty_array(source.header)
        output.header.frame_id = frame
        output.depth_stamp = deepcopy(depth_message.header.stamp)
        for candidate, target_id, stable, frame_count in self._tracker.update(
                candidates, _stamp_ns(source.header.stamp)):
            target = TomatoTarget()
            target.target_id = target_id
            target.class_name = candidate.class_name
            target.confidence = float(candidate.confidence)
            target.detection_index = candidate.detection_index
            target.position.x, target.position.y, target.position.z = map(float, candidate.position)
            target.mature = candidate.class_name == self._mature_class
            target.depth_sample_count = candidate.depth.sample_count
            target.depth_valid_fraction = float(candidate.depth.valid_fraction)
            target.depth_mad_m = float(candidate.depth.mad_m)
            target.segmentation_used = candidate.depth.segmentation_used
            target.temporally_stable = stable
            target.stable_frame_count = frame_count
            output.targets.append(target)
        self._last_rejection = ''
        return output

    def _base_target_message(self, camera):
        output = deepcopy(camera)
        output.header.frame_id = self._target_frame
        if not camera.observation_valid or not camera.targets:
            return output
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, camera.header.frame_id,
                Time.from_msg(camera.header.stamp, clock_type=self.get_clock().clock_type))
        except TransformException as error:
            output.targets = []
            output.observation_valid = False
            output.rejection_reason = f'capture_time_tf_unavailable: {error}'
            return output
        translation, rotation = transform.transform.translation, transform.transform.rotation
        qx, qy, qz, qw = map(float, (rotation.x, rotation.y, rotation.z, rotation.w))
        norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
        if not math.isfinite(norm) or norm < 1e-12:
            output.targets = []
            output.observation_valid = False
            output.rejection_reason = 'invalid_tf_quaternion'
            return output
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        matrix = np.asarray([
            [1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
            [2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)],
            [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx*qx+qy*qy)],
        ])
        offset = np.asarray([translation.x, translation.y, translation.z], dtype=float)
        if not np.isfinite(offset).all():
            output.targets = []
            output.observation_valid = False
            output.rejection_reason = 'invalid_tf_translation'
            return output
        for target in output.targets:
            point = matrix @ np.asarray([target.position.x, target.position.y,
                                         target.position.z]) + offset
            target.position.x, target.position.y, target.position.z = map(float, point)
        return output

    @staticmethod
    def _detection_message(result: Any, source: Image):
        output = Detection2DArray()
        output.header = deepcopy(source.header)
        for xyxy, class_id, confidence in zip(result.boxes.xyxy.tolist(),
                                              result.boxes.cls.tolist(), result.boxes.conf.tolist()):
            xmin, ymin, xmax, ymax = map(float, xyxy)
            detection = Detection2D()
            detection.header = deepcopy(source.header)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(result.names[int(class_id)])
            hypothesis.hypothesis.score = float(confidence)
            detection.results.append(hypothesis)
            bbox = BoundingBox2D()
            bbox.center.position.x, bbox.center.position.y = (xmin+xmax)/2, (ymin+ymax)/2
            bbox.size_x, bbox.size_y = xmax-xmin, ymax-ymin
            detection.bbox = bbox
            output.detections.append(detection)
        return output


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TomatoDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError, RuntimeError) as error:
        if node is not None:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
