"""Facial expression ROS 2 node (Phase 3).

Subscribes to the camera and publishes hri_msgs/Expression for the most
prominent face (single-face for Phase 3; Phase 4 multi-face via person manager).

For each image:
    /humans/faces/expression        hri_msgs/Expression

The Expression message in hri_msgs is a string label only; per-class
probabilities are exposed on a sibling topic for downstream engagement scoring:
    /humans/faces/expression_scores  std_msgs/String  (JSON: {label: prob})
"""
from __future__ import annotations

import json
from typing import Optional

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from hri_msgs.msg import Expression
    _HRI_AVAILABLE = True
except ImportError:  # pragma: no cover
    _HRI_AVAILABLE = False

from .backends.expression import make_backend
from .backends.detection import make_backend as make_detection_backend


# Canonical -> hri_msgs/Expression vocabulary (REP-155 chose its own labels)
_CANONICAL_TO_HRI = {
    "neutral": "neutral",
    "happy": "happy",
    "sad": "sad",
    "anger": "angry",
    "surprise": "surprised",
    "fear": "scared",
    "disgust": "disgusted",
    "contempt": "annoyed",  # closest match in the hri vocabulary
}


class ExpressionNode(Node):
    def __init__(self) -> None:
        super().__init__('expression_node')
        self._declare_params()
        self._bridge = CvBridge()

        # Expression backend (default HSEmotion)
        backend_name = self.get_parameter('expression_backend').value or 'hsemotion'
        try:
            self._backend = make_backend(backend_name)
        except Exception as e:
            self.get_logger().error(f"Failed to load expression backend {backend_name!r}: {e}")
            raise

        # We need a face detector to crop. Reuse the detection backends — but
        # default to YuNet for speed; fall back to dlib_hog if no model given.
        det_backend = self.get_parameter('detection_backend').value or 'dlib_hog'
        det_kwargs = {
            'model_path': self.get_parameter('detection_model_path').value or None,
            'score_threshold': float(self.get_parameter('score_threshold').value),
            'nms_threshold': 0.3,
        }
        try:
            self._detector = make_detection_backend(det_backend, **det_kwargs)
        except Exception as e:
            self.get_logger().warn(f"detection backend {det_backend!r} unavailable ({e}); "
                                   "falling back to dlib_hog")
            self._detector = make_detection_backend('dlib_hog')

        topic = self.get_parameter('image_topic').value
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self._on_image, image_qos)

        self._expr_pub = (
            self.create_publisher(Expression, '/humans/faces/expression', qos_profile=10)
            if _HRI_AVAILABLE else None
        )
        self._scores_pub = self.create_publisher(
            String, '/humans/faces/expression_scores', qos_profile=10
        )

        self.get_logger().info(
            f"Expression Node started — backend={backend_name}, detection={det_backend}"
        )
        if not _HRI_AVAILABLE:
            self.get_logger().warn("hri_msgs not importable — /humans/faces/expression suppressed")

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/camera/color/image_raw'),
            ('expression_backend', 'hsemotion'),
            ('detection_backend', 'dlib_hog'),
            ('detection_model_path', ''),
            ('score_threshold', 0.8),
        ]:
            self.declare_parameter(name, default)

    def _largest_face_crop(self, bgr) -> Optional[tuple]:
        result = self._detector.detect(bgr)
        if not result.boxes:
            return None
        # Pick the largest by area.
        best = max(result.boxes, key=lambda b: (b[2] - b[0]) * (b[1] - b[3]))
        top, right, bottom, left, _score = best
        h, w = bgr.shape[:2]
        top = max(0, top); left = max(0, left)
        bottom = min(h, bottom); right = min(w, right)
        if bottom <= top or right <= left:
            return None
        crop = bgr[top:bottom, left:right]
        return crop

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        face = self._largest_face_crop(bgr)
        if face is None:
            return

        out = self._backend.predict(face)
        if out is None:
            return
        label, probs = out

        # JSON-encoded probabilities — coarse but type-safe wire format until
        # we land a custom msg in Phase 4.
        scores_msg = String(data=json.dumps({"label": label, "probabilities": probs}))
        self._scores_pub.publish(scores_msg)

        if self._expr_pub is not None:
            expr = Expression()
            expr.header = msg.header
            expr.expression = _CANONICAL_TO_HRI.get(label, "neutral")
            expr.confidence = float(probs.get(label, 0.0))
            self._expr_pub.publish(expr)


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = ExpressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
