"""Face recognition (identification) ROS 2 node.

Subscribes to the legacy /face_detections String topic (Phase 5 will switch
to vision_msgs/Detection2DArray once the upstream stops emitting the legacy
form) and publishes:

    /face_identities                std_msgs/String       (LEGACY)
    /face_identity_markers          MarkerArray
    /humans/faces/tracked           hri_msgs/IdsList      (REP-155)
"""
from __future__ import annotations

import os
from typing import List

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from hri_msgs.msg import IdsList  # REP-155
    _HRI_AVAILABLE = True
except ImportError:  # pragma: no cover — humble has it; this guard is for CI matrices
    _HRI_AVAILABLE = False


class FaceIdentifier:
    """Holds known-face encodings and tracks new faces with auto-assigned IDs."""

    def __init__(self, known_dir: str, tolerance: float, backend_name: str = "dlib_128d") -> None:
        self._tolerance = tolerance
        self._backend_name = backend_name
        self._known_encodings: List[np.ndarray] = []
        self._known_ids: List[str] = []
        self._tracked_encodings: List[np.ndarray] = []
        self._tracked_ids: List[str] = []
        self._next_id = 1
        self._load_known(known_dir)

    def _load_known(self, directory: str) -> None:
        if not os.path.exists(directory):
            return
        # We still use face_recognition to load known faces — its API is
        # well-fitted to this offline step. Online identification reuses
        # whatever embedding came from the recognition backend.
        try:
            import face_recognition
        except ImportError:
            return
        for fname in os.listdir(directory):
            path = os.path.join(directory, fname)
            try:
                image = face_recognition.load_image_file(path)
                encs = face_recognition.face_encodings(image)
            except Exception:
                continue
            if encs:
                self._known_encodings.append(encs[0])
                self._known_ids.append(os.path.splitext(fname)[0])

    def identify(self, encoding: np.ndarray) -> str:
        # Cosine-distance match for any backend; dlib historically used L2.
        # The two are monotonically related for unit-normalised vectors, so
        # the tolerance scale stays compatible.
        for idx, enc in enumerate(self._known_encodings):
            if self._match(enc, encoding):
                return self._known_ids[idx]
        for idx, enc in enumerate(self._tracked_encodings):
            if self._match(enc, encoding):
                return self._tracked_ids[idx]
        new_id = f"user_{self._next_id}"
        self._next_id += 1
        self._tracked_encodings.append(encoding)
        self._tracked_ids.append(new_id)
        return new_id

    def _match(self, ref: np.ndarray, query: np.ndarray) -> bool:
        # Backwards-compatible: dlib's compare_faces uses L2 distance threshold
        # 0.6 by default. For non-dlib backends we fall back to cosine.
        if self._backend_name == "dlib_128d":
            return float(np.linalg.norm(ref - query)) <= self._tolerance
        denom = (np.linalg.norm(ref) * np.linalg.norm(query)) or 1e-12
        sim = float(np.dot(ref, query) / denom)
        # For cosine, larger similarity = more likely same. Use 1 - sim as a
        # distance and compare against tolerance.
        return (1.0 - sim) <= self._tolerance


class FaceRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__('face_recognition_node')
        self._declare_params()
        backend_name = self.get_parameter('recognition_backend').value or 'dlib_128d'
        self._identifier = FaceIdentifier(
            known_dir=self.get_parameter('known_faces_dir').value,
            tolerance=self.get_parameter('match_tolerance').value,
            backend_name=backend_name,
        )
        self._backend_name = backend_name

        # Subscribers / publishers — keep legacy I/O intact.
        self.create_subscription(String, 'face_detections', self._on_face_detection, qos_profile=10)
        self._face_identity_pub = self.create_publisher(String, 'face_identities', qos_profile=10)
        self._identity_markers_pub = self.create_publisher(MarkerArray, 'face_identity_markers', qos_profile=10)
        self._tracked_pub = (
            self.create_publisher(IdsList, '/humans/faces/tracked', qos_profile=10)
            if _HRI_AVAILABLE else None
        )

        self._identity_colors = {
            'known': ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
            'unknown': ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),
            'user_1': ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9),
            'user_2': ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9),
            'user_3': ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9),
            'user_4': ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9),
            'user_5': ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),
            'default': ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.9),
        }
        self._currently_tracked: List[str] = []

        self.get_logger().info(
            f"Face Recognition Node started — backend={self._backend_name}"
        )
        if not _HRI_AVAILABLE:
            self.get_logger().warn(
                "hri_msgs not importable — /humans/faces/tracked will NOT be published"
            )

    def _declare_params(self) -> None:
        for name, default in [
            ('known_faces_dir', 'known_faces'),
            ('match_tolerance', 0.6),
            ('recognition_backend', 'dlib_128d'),
        ]:
            self.declare_parameter(name, default)

    def _on_face_detection(self, msg: String) -> None:
        data = msg.data.split('|')
        if len(data) < 8:
            return

        face_idx = data[0]
        center_x = float(data[1])
        center_y = float(data[2])
        width = float(data[3])
        height = float(data[4])
        frame_width = int(data[5])
        frame_height = int(data[6])
        encoding_str = data[7]

        # Encoding may be empty when the upstream backend doesn't compute one
        # (e.g. YuNet). Identification of such faces requires running the
        # recognition backend here — not yet wired (Phase 2 stays drop-in for
        # the dlib path). Skip rather than mis-identify.
        if not encoding_str:
            return
        try:
            encoding = np.array([float(x) for x in encoding_str.split(',')])
        except ValueError:
            return

        face_id = self._identifier.identify(encoding)

        # Legacy String output
        identity_data = (
            f"{face_id}|{center_x}|{center_y}|{width}|{height}|{frame_width}|{frame_height}"
        )
        self._face_identity_pub.publish(String(data=identity_data))

        # Update tracked list and publish hri_msgs/IdsList
        if face_id not in self._currently_tracked:
            self._currently_tracked.append(face_id)
        if self._tracked_pub is not None:
            ids = IdsList()
            ids.header.stamp = self.get_clock().now().to_msg()
            ids.ids = list(self._currently_tracked)
            self._tracked_pub.publish(ids)

        # Markers (legacy visualisation)
        marker_array = MarkerArray()
        marker_array.markers.append(self._create_identity_marker(
            face_id, center_x, center_y, width, height, frame_width, frame_height
        ))
        marker_array.markers.append(self._create_text_marker(
            face_id, center_x, center_y, frame_width, frame_height
        ))
        self._identity_markers_pub.publish(marker_array)

    def _get_identity_color(self, face_id: str) -> ColorRGBA:
        if face_id in self._identifier._known_ids:
            return self._identity_colors['known']
        if face_id in self._identity_colors:
            return self._identity_colors[face_id]
        return self._identity_colors['default']

    def _create_identity_marker(self, face_id: str, center_x: float, center_y: float,
                               width: float, height: float, frame_width: int, frame_height: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_identities"
        marker.id = hash(face_id) % 1000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.2
        marker.pose.position.y = -(center_x - frame_width / 2) * 0.002
        marker.pose.position.z = -(center_y - frame_height / 2) * 0.002
        marker.pose.orientation.w = 1.0
        scale = max(width, height) * 0.0012
        marker.scale = Vector3(x=scale, y=scale, z=scale)
        marker.color = self._get_identity_color(face_id)
        marker.lifetime = Duration(sec=2, nanosec=0)
        return marker

    def _create_text_marker(self, face_id: str, center_x: float, center_y: float,
                           frame_width: int, frame_height: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_identity_text"
        marker.id = hash(face_id) % 1000 + 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 1.15
        marker.pose.position.y = -(center_x - frame_width / 2) * 0.002
        marker.pose.position.z = -(center_y - frame_height / 2) * 0.002 + 0.1
        marker.pose.orientation.w = 1.0
        marker.text = face_id
        marker.scale.z = 0.08
        marker.color = self._get_identity_color(face_id)
        marker.lifetime = Duration(sec=2, nanosec=0)
        return marker


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
