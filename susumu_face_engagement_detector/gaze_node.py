"""New gaze direction ROS 2 node (Phase 3).

Publishes a 3-D unit gaze vector for the most prominent face in each frame:
    /humans/faces/gaze       geometry_msgs/Vector3Stamped

The vector is expressed in the camera optical frame (X right, Y down,
Z forward, as is convention for ROS image_geometry).

The legacy /gaze_status (gaze_analysis_node.py) is **not** subsumed by this
node — Phase 4's engagement_node will combine this gaze direction with head
pose / expression and produce the 5-state hri_msgs/EngagementLevel.
"""
from __future__ import annotations

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .backends.gaze import make_backend


class GazeNode(Node):
    def __init__(self) -> None:
        super().__init__('gaze_node')
        self._declare_params()
        self._bridge = CvBridge()
        backend_name = self.get_parameter('gaze_backend').value or 'mediapipe_iris'
        self._backend = make_backend(backend_name)
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_only_when_valid = self.get_parameter('publish_only_when_valid').value

        topic = self.get_parameter('image_topic').value
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self._on_image, image_qos)
        self._pub = self.create_publisher(Vector3Stamped, '/humans/faces/gaze', qos_profile=10)

        self.get_logger().info(
            f"Gaze Node started — backend={backend_name}, input={topic}"
        )

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/camera/color/image_raw'),
            ('gaze_backend', 'mediapipe_iris'),
            ('frame_id', 'camera_color_optical_frame'),
            ('publish_only_when_valid', True),
        ]:
            self.declare_parameter(name, default)

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        gaze = self._backend.estimate(bgr)
        if gaze is None:
            if self._publish_only_when_valid:
                return
            gaze = (0.0, 0.0, 1.0)

        out = Vector3Stamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame_id
        out.vector = Vector3(x=gaze[0], y=gaze[1], z=gaze[2])
        self._pub.publish(out)


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = GazeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
