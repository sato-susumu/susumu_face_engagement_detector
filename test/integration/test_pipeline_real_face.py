"""End-to-end check that publishes a real face image and verifies the
detection node outputs a non-empty Detection2DArray.

Runs in a subprocess to avoid sys.modules contamination from legacy tests.
"""
from __future__ import annotations

import json
import os
import subprocess
import textwrap
from pathlib import Path

import pytest


ROS_SETUP = Path("/opt/ros/humble/setup.bash")
REPO_ROOT = Path(__file__).resolve().parents[2]
# Use a known-frontal WIDER FACE image. The repo previously shipped synthetic
# test_images/ that dlib HOG could not detect (0/8) — they were removed in
# favour of real WIDER FACE samples, which exposes real-world performance.
WIDER_ROOT = Path.home() / "datasets" / "wider_face"
SAMPLE_IMAGE = WIDER_ROOT / "WIDER_val" / "images" / "0--Parade" / "0_Parade_Parade_0_12.jpg"


pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not ROS_SETUP.exists(), reason="ROS 2 Humble setup.bash not found"),
    pytest.mark.skipif(not SAMPLE_IMAGE.exists(),
                       reason=f"need WIDER FACE val downloaded under {WIDER_ROOT}"),
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
            super().__init__('e2e_real_face')
            self.received = []
            self.create_subscription(
                Detection2DArray, 'face_detections_vision',
                lambda m: self.received.append(m), 10,
            )
            self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
            self.bridge = CvBridge()

        def publish_face(self):
            img = cv2.imread(IMAGE_PATH, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f'Failed to read {{IMAGE_PATH}}')
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            self.pub.publish(msg)


    def main():
        # Override parameters via the node's parameter overrides BEFORE
        # construction — the node fixes max_fps / downsample at __init__ time.
        rclpy.init(args=[
            '--ros-args',
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
                'first_bbox': None,
            }}
            if harness.received and harness.received[0].detections:
                d = harness.received[0].detections[0]
                result['first_bbox'] = {{
                    'cx': d.bbox.center.position.x,
                    'cy': d.bbox.center.position.y,
                    'sx': d.bbox.size_x,
                    'sy': d.bbox.size_y,
                    'class_id': d.results[0].hypothesis.class_id,
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


def test_real_face_image_produces_detection2d(tmp_path: Path) -> None:
    script = tmp_path / "harness_real.py"
    script.write_text(HARNESS_SCRIPT.format(image_path=str(SAMPLE_IMAGE)))
    cmd = ["bash", "-c", f"source {ROS_SETUP} && python3 {script}"]
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", str((os.getpid() + 7) % 100 + 50))

    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60, env=env)
    stdout, stderr = proc.stdout, proc.stderr

    payload = None
    for line in stdout.splitlines():
        if line.startswith("RESULT_JSON="):
            payload = json.loads(line.split("=", 1)[1])
            break
    assert payload is not None, (
        f"harness did not emit RESULT_JSON\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}"
    )
    assert payload["received_count"] >= 1, "no Detection2DArray received"
    assert payload["first_detections"] >= 1, (
        f"expected ≥1 face in {SAMPLE_IMAGE.name}, got {payload['first_detections']}"
    )
    bbox = payload["first_bbox"]
    assert bbox is not None
    assert bbox["class_id"] == "face"
    assert bbox["sx"] > 0 and bbox["sy"] > 0
