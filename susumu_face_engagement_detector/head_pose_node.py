"""Head pose ROS 2 node.

Subscribes to a camera image and publishes:
    /humans/faces/head_pose      geometry_msgs/PoseStamped
                                  with quaternion encoding yaw/pitch/roll.

This is a single-face implementation for Phase 3. Multi-face routing through
hri_msgs/Person manager comes in Phase 4 — until then, the node operates on
the most prominent face in the frame.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .backends.headpose import make_backend


def _ypr_to_quaternion(yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple[float, float, float, float]:
    """Convert yaw/pitch/roll degrees to (x, y, z, w) quaternion.

    Uses Tait–Bryan ZYX intrinsic rotations (roll→pitch→yaw).
    """
    cy = math.cos(math.radians(yaw_deg) * 0.5)
    sy = math.sin(math.radians(yaw_deg) * 0.5)
    cp = math.cos(math.radians(pitch_deg) * 0.5)
    sp = math.sin(math.radians(pitch_deg) * 0.5)
    cr = math.cos(math.radians(roll_deg) * 0.5)
    sr = math.sin(math.radians(roll_deg) * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class HeadPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('head_pose_node')
        self._declare_params()
        self._bridge = CvBridge()
        backend_name = self.get_parameter('head_pose_backend').value or 'mediapipe_pnp'
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
        self._pose_pub = self.create_publisher(PoseStamped, '/humans/faces/head_pose', qos_profile=10)

        self.get_logger().info(
            f"Head Pose Node started — backend={backend_name}, input={topic}"
        )

    def _declare_params(self) -> None:
        for name, default in [
            ('image_topic', '/camera/color/image_raw'),
            ('head_pose_backend', 'mediapipe_pnp'),
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

        ypr = self._backend.estimate(bgr)
        if ypr is None:
            if self._publish_only_when_valid:
                return
            ypr = (0.0, 0.0, 0.0)

        yaw, pitch, roll = ypr
        qx, qy, qz, qw = _ypr_to_quaternion(yaw, pitch, roll)
        out = PoseStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame_id
        out.pose.position.x = 0.0
        out.pose.position.y = 0.0
        out.pose.position.z = 1.0  # placeholder until depth is wired
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self._pose_pub.publish(out)


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = HeadPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
