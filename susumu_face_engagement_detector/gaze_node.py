"""OpenVINO gaze-estimation ROS 2 node.

Publishes:
    /humans/faces/gaze    geometry_msgs/Vector3Stamped

This node estimates a single prominent face per frame. It does not publish a
heuristic gaze fallback; if the OpenVINO model or required eye/head-pose inputs
are unavailable, no gaze message is emitted by default.
"""
from __future__ import annotations

import os

from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .backends.gaze import make_backend as make_gaze_backend
from .backends.headpose import HeadPoseStabilizer, make_backend as make_headpose_backend


class GazeNode(Node):
    def __init__(self) -> None:
        super().__init__('gaze_node')
        self._declare_params()
        self._bridge = CvBridge()

        gaze_backend = self.get_parameter('gaze_backend').value or 'openvino_adas'
        self._gaze = make_gaze_backend(
            gaze_backend,
            model_path=self.get_parameter('gaze_model_path').value,
            device=self.get_parameter('gaze_device').value,
            min_eye_size=int(self.get_parameter('min_eye_size').value),
            max_head_yaw_deg=float(self.get_parameter('max_head_yaw_deg').value),
            max_head_pitch_deg=float(self.get_parameter('max_head_pitch_deg').value),
        )
        headpose_backend = self.get_parameter('head_pose_backend').value or 'mediapipe_pnp'
        self._headpose = make_headpose_backend(headpose_backend)
        self._stabilizer = HeadPoseStabilizer(
            ema_alpha=float(self.get_parameter('headpose_smoothing_alpha').value),
            max_jump_deg=float(self.get_parameter('headpose_max_jump_deg').value),
            reset_after_missed=int(self.get_parameter('headpose_reset_after_missed').value),
        )
        self._publish_only_when_valid = bool(self.get_parameter('publish_only_when_valid').value)
        self._frame_id = self.get_parameter('frame_id').value

        topic = self.get_parameter('image_topic').value
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self._on_image, image_qos)
        self._gaze_pub = self.create_publisher(Vector3Stamped, '/humans/faces/gaze', qos_profile=10)

        self.get_logger().info(
            f"Gaze Node started - backend={gaze_backend}, input={topic}"
        )

    def _declare_params(self) -> None:
        default_model = os.path.expanduser(
            '~/models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/'
            'gaze-estimation-adas-0002.xml'
        )
        for name, default in [
            ('image_topic', '/camera/color/image_raw'),
            ('gaze_backend', 'openvino_adas'),
            ('gaze_model_path', default_model),
            ('gaze_device', 'CPU'),
            ('head_pose_backend', 'mediapipe_pnp'),
            ('frame_id', 'camera_color_optical_frame'),
            ('publish_only_when_valid', True),
            ('min_eye_size', 8),
            ('max_head_yaw_deg', 45.0),
            ('max_head_pitch_deg', 35.0),
            ('headpose_smoothing_alpha', 0.35),
            ('headpose_max_jump_deg', 35.0),
            ('headpose_reset_after_missed', 10),
        ]:
            self.declare_parameter(name, default)

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        ypr = self._headpose.estimate(bgr)
        if ypr is None:
            self._stabilizer.mark_missed()
            if self._publish_only_when_valid:
                return
            ypr = (0.0, 0.0, 0.0)
        else:
            stable = self._stabilizer.update(ypr)
            if stable is not None:
                ypr = stable

        result = self._gaze.estimate(bgr, ypr)
        if result is None:
            if self._publish_only_when_valid:
                return
            vector = (0.0, 0.0, 0.0)
        else:
            vector = result.vector

        out = Vector3Stamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self._frame_id
        out.vector.x = vector[0]
        out.vector.y = vector[1]
        out.vector.z = vector[2]
        self._gaze_pub.publish(out)


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
