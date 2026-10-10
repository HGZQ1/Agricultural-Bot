"""Run the trained tomato segmentation model on a ROS 2 image stream."""

from __future__ import annotations

import math
from typing import Any

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class TomatoDetector(Node):
    """Subscribe to the D405 color stream and publish YOLO results."""

    def __init__(self) -> None:
        super().__init__('tomato_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/d405/color/image_raw')
        self.declare_parameter(
            'annotated_image_topic', '/agri_vision/annotated_image')
        self.declare_parameter('detections_topic', '/agri_vision/detections')
        self.declare_parameter(
            'depth_topic', '/d405/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/d405/color/camera_info')
        self.declare_parameter(
            'positions_topic', '/agri_vision/tomato_positions_camera')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter(
            'base_positions_topic', '/agri_vision/tomato_positions_base')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_size', 640)
        # ROS 2 parses ``device:=0`` as an integer parameter.  Keep the
        # default numeric so the launch file works, while accepting ``cpu``
        # or another Ultralytics device string as well.
        self.declare_parameter('device', 0)

        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            raise ValueError(
                'model_path is empty; pass the path to the trained best.pt')

        self._confidence = float(
            self.get_parameter('confidence_threshold').value)
        self._image_size = int(self.get_parameter('image_size').value)
        device_value = self.get_parameter('device').value
        self._device = (
            int(device_value)
            if isinstance(device_value, int)
            else str(device_value)
        )
        if not 0.0 < self._confidence <= 1.0:
            raise ValueError('confidence_threshold must be in (0, 1]')
        if self._image_size <= 0:
            raise ValueError('image_size must be positive')

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self._model = YOLO(model_path)
        self._bridge = CvBridge()
        self._frame_count = 0

        image_topic = str(self.get_parameter('image_topic').value)
        annotated_topic = str(
            self.get_parameter('annotated_image_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        camera_info_topic = str(
            self.get_parameter('camera_info_topic').value)
        positions_topic = str(self.get_parameter('positions_topic').value)
        target_frame = str(self.get_parameter('target_frame').value)
        base_positions_topic = str(
            self.get_parameter('base_positions_topic').value)

        self._annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self._detections_pub = self.create_publisher(
            Detection2DArray, detections_topic, 10)
        self._positions_pub = self.create_publisher(
            PoseArray, positions_topic, 10)
        self._base_positions_pub = self.create_publisher(
            PoseArray, base_positions_topic, 10)
        self._target_frame = target_frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_warning_count = 0
        self._latest_depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Subscribed to {image_topic}; publishing {annotated_topic} and '
            f'{detections_topic}; depth: {depth_topic}; '
            f'TF target: {target_frame}')

    def _depth_callback(self, message: Image) -> None:
        self._latest_depth = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _image_callback(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(message, 'bgr8')
            result = self._model.predict(
                source=image,
                device=self._device,
                imgsz=self._image_size,
                conf=self._confidence,
                verbose=False,
            )[0]
            annotated = result.plot()
            annotated_message = self._bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_message.header = message.header
            self._annotated_pub.publish(annotated_message)
            self._detections_pub.publish(
                self._detection_message(result, message))
            positions = self._positions_message(result, message)
            if positions is not None:
                self._positions_pub.publish(positions)
                base_positions = self._base_positions_message(positions)
                if base_positions is not None:
                    self._base_positions_pub.publish(base_positions)
        except (CvBridgeError, RuntimeError, ValueError) as error:
            self.get_logger().error(f'Vision callback failed: {error}')
            return

        self._frame_count += 1
        if self._frame_count == 1 or self._frame_count % 30 == 0:
            self.get_logger().info(
                f'Processed frame {self._frame_count}: '
                f'{len(result.boxes)} detections')

    def _positions_message(
            self, result: Any, source: Image) -> PoseArray | None:
        """Project detection centers into the aligned camera optical frame."""
        if self._latest_depth is None or self._camera_info is None:
            return None

        depth_message = self._latest_depth
        camera_info = self._camera_info
        try:
            depth = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.get_logger().error(f'Depth conversion failed: {error}')
            return None

        depth = np.asarray(depth)
        if depth.ndim != 2:
            return None
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32, copy=False)

        fx, fy, cx, cy = (
            float(camera_info.k[0]),
            float(camera_info.k[4]),
            float(camera_info.k[2]),
            float(camera_info.k[5]),
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in (fx, fy)):
            return None

        output = PoseArray()
        output.header = source.header
        output.header.frame_id = (
            camera_info.header.frame_id
            or depth_message.header.frame_id
            or source.header.frame_id)

        height, width = depth.shape
        for xyxy in result.boxes.xyxy.tolist():
            x_min, y_min, x_max, y_max = (float(value) for value in xyxy)
            u = int(round((x_min + x_max) / 2.0))
            v = int(round((y_min + y_max) / 2.0))
            if not (0 <= u < width and 0 <= v < height):
                continue

            u0, u1 = max(0, u - 2), min(width, u + 3)
            v0, v1 = max(0, v - 2), min(height, v + 3)
            samples = depth[v0:v1, u0:u1]
            valid = samples[np.isfinite(samples) & (samples > 0.05)
                             & (samples < 20.0)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))
            pose = Pose()
            pose.position.x = (u - cx) * z / fx
            pose.position.y = (v - cy) * z / fy
            pose.position.z = z
            pose.orientation.w = 1.0
            output.poses.append(pose)

        return output

    def _base_positions_message(self, camera_positions: PoseArray):
        """Transform camera-frame positions into the configured robot frame."""
        if not camera_positions.header.frame_id:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                camera_positions.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as error:
            self._tf_warning_count += 1
            if self._tf_warning_count == 1 or self._tf_warning_count % 100 == 0:
                self.get_logger().warning(
                    f'Waiting for TF {self._target_frame} <- '
                    f'{camera_positions.header.frame_id}: {error}')
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = (
            float(rotation.x), float(rotation.y),
            float(rotation.z), float(rotation.w))
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm < 1e-12:
            return None
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        output = PoseArray()
        output.header = camera_positions.header
        output.header.frame_id = self._target_frame
        for pose in camera_positions.poses:
            x, y, z = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            # Rotate the point by the TF quaternion, then apply translation.
            tx = ((1 - 2 * (qy * qy + qz * qz)) * x
                  + 2 * (qx * qy - qw * qz) * y
                  + 2 * (qx * qz + qw * qy) * z)
            ty = (2 * (qx * qy + qw * qz) * x
                  + (1 - 2 * (qx * qx + qz * qz)) * y
                  + 2 * (qy * qz - qw * qx) * z)
            tz = (2 * (qx * qz - qw * qy) * x
                  + 2 * (qy * qz + qw * qx) * y
                  + (1 - 2 * (qx * qx + qy * qy)) * z)
            transformed = Pose()
            transformed.position.x = tx + float(translation.x)
            transformed.position.y = ty + float(translation.y)
            transformed.position.z = tz + float(translation.z)
            transformed.orientation.w = 1.0
            output.poses.append(transformed)

        return output

    def _detection_message(self, result: Any, source: Image) -> Detection2DArray:
        output = Detection2DArray()
        output.header = source.header

        names = result.names
        boxes = result.boxes
        for xyxy, class_id, confidence in zip(
                boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist()):
            x_min, y_min, x_max, y_max = (float(value) for value in xyxy)
            detection = Detection2D()
            detection.header = source.header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(names[int(class_id)])
            hypothesis.hypothesis.score = float(confidence)
            detection.results.append(hypothesis)

            bbox = BoundingBox2D()
            bbox.center.position.x = (x_min + x_max) / 2.0
            bbox.center.position.y = (y_min + y_max) / 2.0
            bbox.size_x = x_max - x_min
            bbox.size_y = y_max - y_min
            detection.bbox = bbox
            output.detections.append(detection)

        return output


def main(args=None) -> None:
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
        rclpy.shutdown()
