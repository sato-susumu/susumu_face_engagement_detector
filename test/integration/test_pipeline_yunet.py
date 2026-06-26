"""Verify the node end-to-end with the YuNet backend."""
from __future__ import annotations

import json
import os
import subprocess
import textwrap
from pathlib import Path

import pytest


ROS_SETUP = Path("/opt/ros/humble/setup.bash")
WIDER_ROOT = Path.home() / "datasets" / "wider_face"
SAMPLE_IMAGE = WIDER_ROOT / "WIDER_val" / "images" / "0--Parade" / "0_Parade_Parade_0_12.jpg"
YUNET_MODEL = Path.home() / "models" / "face_detection" / "face_detection_yunet_2023mar.onnx"

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not ROS_SETUP.exists(), reason="ROS 2 Humble setup.bash not found"),
    pytest.mark.skipif(not SAMPLE_IMAGE.exists(),
                       reason=f"need WIDER FACE val downloaded under {WIDER_ROOT}"),
    pytest.mark.skipif(not YUNET_MODEL.exists(),
                       reason=f"need YuNet weights at {YUNET_MODEL}"),
]


HARNESS_SCRIPT = textwrap.dedent(
    """\
    import json
    import sys
    import time

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray

    from susumu_face_engagement_detector.face_detection_node import FaceDetectionNode

    IMAGE_PATH = "{image_path}"


    class Harness(Node):
        def __init__(self):
            super().__init__('e2e_yunet')
            self.received = []
            self.create_subscription(
                Detection2DArray, 'face_detections_vision',
                lambda m: self.received.append(m), 10,
            )
            self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
            self.bridge = CvBridge()

        def publish_face(self):
            img = cv2.imread(IMAGE_PATH, cv2.IMREAD_COLOR)
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            self.pub.publish(msg)


    def main():
        rclpy.init(args=[
            '--ros-args',
            '-p', 'detection_backend:=yunet',
            '-p', 'model_path:={model_path}',
            '-p', 'downsample_factor:=1.0',
            '-p', 'max_fps:=0.0',
        ])
        detector = FaceDetectionNode()
        harness = Harness()
        ex = SingleThreadedExecutor()
        ex.add_node(detector)
        ex.add_node(harness)
        try:
            for _ in range(15):
                ex.spin_once(timeout_sec=0.02)
            harness.publish_face()
            end = time.time() + 10.0
            while time.time() < end and not harness.received:
                ex.spin_once(timeout_sec=0.05)
            result = {{
                'received_count': len(harness.received),
                'first_detections': (
                    len(harness.received[0].detections)
                    if harness.received else None
                ),
                'first_score': (
                    harness.received[0].detections[0].results[0].hypothesis.score
                    if harness.received and harness.received[0].detections else None
                ),
            }}
            print('RESULT_JSON=' + json.dumps(result))
            return 0
        finally:
            detector.destroy_node()
            harness.destroy_node()
            rclpy.shutdown()


    sys.exit(main())
    """
)


def test_yunet_backend_publishes_detection2d(tmp_path: Path) -> None:
    script = tmp_path / "harness_yunet.py"
    script.write_text(HARNESS_SCRIPT.format(
        image_path=str(SAMPLE_IMAGE),
        model_path=str(YUNET_MODEL),
    ))
    cmd = ["bash", "-c", f"source {ROS_SETUP} && python3 {script}"]
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", str((os.getpid() + 13) % 100 + 50))

    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30, env=env)

    payload = None
    for line in proc.stdout.splitlines():
        if line.startswith("RESULT_JSON="):
            payload = json.loads(line.split("=", 1)[1])
            break
    assert payload is not None, (
        f"no RESULT_JSON in output\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    assert payload["received_count"] >= 1
    assert payload["first_detections"] >= 1
    # YuNet returns real confidence scores (not 1.0 sentinel from dlib).
    assert payload["first_score"] is not None
    assert 0.0 < payload["first_score"] <= 1.0
