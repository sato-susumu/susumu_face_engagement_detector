"""Face detection ROS 2 node with pluggable backends.

Backends are defined in ``susumu_face_engagement_detector.backends.detection``
and selected by the ``detection_backend`` parameter at startup.

Output topics:
    /face_detections              std_msgs/String         (LEGACY — pipe-delimited)
    /face_detections_vision       vision_msgs/Detection2DArray  (PREFERRED)
    /face_detection_markers       visualization_msgs/MarkerArray
    /face_detection_image         sensor_msgs/Image       (annotated)

The legacy String topic is preserved through Phase 5 (see docs/REVAMP_PLAN.md)
because downstream nodes still depend on it. Once the new pipeline (Phases
2–4) is in place the String topic will be removed.
"""
from typing import List, Tuple

import cv2
import numpy as np
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Header, String
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from .backends.detection import DetectionResult, make_backend


class FaceDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('face_detection_node')
        self._declare_params()
        self._bridge = CvBridge()

        # Backend selection — defaults to dlib_hog so the node behaves the same
        # out-of-the-box as before Phase 2.
        backend_name = self.get_parameter('detection_backend').value
        # Backwards compatibility: 'detection_model' (hog/cnn) was the old param.
        if backend_name == "":
            legacy = self.get_parameter('detection_model').value
            backend_name = "dlib_cnn" if legacy == "cnn" else "dlib_hog"
        self._detector = make_backend(
            backend_name,
            model_path=self.get_parameter('model_path').value,
            score_threshold=self.get_parameter('score_threshold').value,
            nms_threshold=self.get_parameter('nms_threshold').value,
        )

        # CPU負荷軽減のための制御変数
        self._max_fps = float(self.get_parameter('max_fps').value)
        self._min_interval = 1.0 / self._max_fps if self._max_fps > 0 else 0.0
        self._last_process_time = 0.0
        self._downsample_factor = float(self.get_parameter('downsample_factor').value)

        # 最新のフレームをキャッシュ (FPS制限時のみ)
        self._latest_frame = None

        topic = self.get_parameter('image_topic').value

        # QoS profile optimized for image streams
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, topic, self._on_image, image_qos)

        self._face_detection_pub = self.create_publisher(String, 'face_detections', qos_profile=10)
        self._detection2d_pub = self.create_publisher(
            Detection2DArray, 'face_detections_vision', qos_profile=10
        )

        # RViz可視化用パブリッシャー
        self._face_markers_pub = self.create_publisher(MarkerArray, 'face_detection_markers', qos_profile=10)
        self._annotated_image_pub = self.create_publisher(Image, 'face_detection_image', qos_profile=10)

        self._face_colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8),
            ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8),
            ColorRGBA(r=0.5, g=0.0, b=1.0, a=0.8),
        ]

        if self._max_fps > 0:
            self.create_timer(self._min_interval, self._timer_process)

        self.get_logger().info(
            f"Face Detection Node started — backend={self._detector.name}, input={topic}"
        )
        self.get_logger().info(
            f"Output topics: /face_detections, /face_detections_vision, "
            f"/face_detection_markers, /face_detection_image"
        )
        self.get_logger().info(
            f"Performance: max_fps={self._max_fps}, downsample={self._downsample_factor}"
        )

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/camera/color/image_raw'),
            # NEW (Phase 2): pluggable backend selection.
            ('detection_backend', ''),     # '' → fall back to detection_model
            ('model_path', ''),            # backend-specific (YuNet ONNX, …)
            ('score_threshold', 0.8),
            ('nms_threshold', 0.3),
            # LEGACY
            ('detection_model', 'hog'),
            ('max_fps', 5.0),
            ('downsample_factor', 0.5),
        ]:
            self.declare_parameter(name, default)

    def _on_image(self, msg: Image) -> None:
        if self._max_fps == 0:
            self._process_frame(msg)
            return
        self._latest_frame = msg

    def _timer_process(self) -> None:
        current_time = time.time()
        if current_time - self._last_process_time < self._min_interval:
            return
        if self._latest_frame is None:
            return
        self._process_frame(self._latest_frame)
        self._last_process_time = current_time
        self._latest_frame = None

    def _process_frame(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Downsample (optional) — backends operate on BGR ndarray.
        if self._downsample_factor < 1.0:
            small = cv2.resize(frame, None, fx=self._downsample_factor, fy=self._downsample_factor)
            result: DetectionResult = self._detector.detect(small)
            scale = 1.0 / self._downsample_factor
            scaled_boxes = [
                (int(top * scale), int(right * scale), int(bottom * scale),
                 int(left * scale), score)
                for top, right, bottom, left, score in result.boxes
            ]
            result = DetectionResult(boxes=scaled_boxes, encodings=result.encodings)
        else:
            result = self._detector.detect(frame)

        annotated_frame = frame.copy()
        marker_array = MarkerArray()
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for i, box in enumerate(result.boxes):
            top, right, bottom, left, score = box
            center_x = (left + right) / 2
            center_y = (top + bottom) / 2
            width = right - left
            height = bottom - top

            # Encoding may be unavailable for backends like YuNet — emit an
            # empty string in that slot so the legacy String contract holds.
            if result.encodings is not None and i < len(result.encodings):
                encoding_str = ','.join(map(str, result.encodings[i]))
            else:
                encoding_str = ''
            detection_data = (
                f"{i}|{center_x}|{center_y}|{width}|{height}|"
                f"{frame.shape[1]}|{frame.shape[0]}|{encoding_str}"
            )
            self._face_detection_pub.publish(String(data=detection_data))

            # vision_msgs/Detection2D
            det2d = Detection2D()
            det2d.header = msg.header
            det2d.id = str(i)
            bb = BoundingBox2D()
            bb.center.position.x = float(center_x)
            bb.center.position.y = float(center_y)
            bb.size_x = float(width)
            bb.size_y = float(height)
            det2d.bbox = bb
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "face"
            hyp.hypothesis.score = float(score)
            det2d.results.append(hyp)
            detection_array.detections.append(det2d)

            color = self._get_face_color_bgr(i)
            cv2.rectangle(annotated_frame, (left, top), (right, bottom), color, 2)
            label = f"Face {i}" + (f" {score:.2f}" if score < 1.0 else "")
            cv2.putText(annotated_frame, label, (left, max(0, top - 10)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            marker = self._create_face_marker(
                i, center_x, center_y, width, height,
                frame.shape[1], frame.shape[0], msg.header,
            )
            marker_array.markers.append(marker)

        # Always publish — empty array signals "image processed, no faces".
        self._detection2d_pub.publish(detection_array)

        try:
            annotated_msg = self._bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            annotated_msg.header = msg.header
            self._annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish annotated image: {e}')

        self._face_markers_pub.publish(marker_array)

    def _get_face_color_bgr(self, face_index: int) -> Tuple[int, int, int]:
        color = self._face_colors[face_index % len(self._face_colors)]
        return (int(color.b * 255), int(color.g * 255), int(color.r * 255))

    def _create_face_marker(self, face_id: int, center_x: float, center_y: float,
                           width: float, height: float, frame_width: int, frame_height: int,
                           header: Header) -> Marker:
        marker = Marker()
        marker.header = header
        marker.header.frame_id = "camera_link"
        marker.ns = "face_detections"
        marker.id = face_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 1.0
        marker.pose.position.y = -(center_x - frame_width / 2) * 0.002
        marker.pose.position.z = -(center_y - frame_height / 2) * 0.002
        marker.pose.orientation.w = 1.0

        marker.scale = Vector3(x=width * 0.001, y=height * 0.001, z=0.05)
        marker.color = self._face_colors[face_id % len(self._face_colors)]
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0
        return marker


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = FaceDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
