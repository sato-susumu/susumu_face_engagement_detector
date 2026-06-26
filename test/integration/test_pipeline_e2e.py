"""End-to-end smoke of the detection pipeline.

Runs the actual face_detection_node + a small subscriber in a subprocess
to escape the cv_bridge/cv2 MagicMock pollution that the legacy tests
inject into pytest's sys.modules. The harness checks that:

  * the new vision_msgs/Detection2DArray topic publishes at all
  * the empty-image case still publishes (an empty array, not nothing)

Real-image AP belongs to ``eval/runners/run_detection_eval.py``.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

# Locate dependencies up front so the subprocess uses the same Python.
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
pytestmark = pytest.mark.skipif(
    not ROS_SETUP.exists(), reason="ROS 2 Humble setup.bash not found"
)


HARNESS_SCRIPT = textwrap.dedent(
    """\
    import json
    import sys
    import time

    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray

    from susumu_face_engagement_detector.face_detection_node import FaceDetectionNode


    class Collector(Node):
        def __init__(self):
            super().__init__('e2e_collector')
            self.received = []
            self.create_subscription(
                Detection2DArray, 'face_detections_vision',
                lambda m: self.received.append(m), 10,
            )
            self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
            self.bridge = CvBridge()

        def publish_blank(self):
            img = np.zeros((120, 160, 3), dtype=np.uint8)
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            now = self.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = 'camera_color_optical_frame'
            self.pub.publish(msg)


    def main():
        rclpy.init()
        detector = FaceDetectionNode()
        collector = Collector()
        ex = SingleThreadedExecutor()
        ex.add_node(detector)
        ex.add_node(collector)
        try:
            for _ in range(10):
                ex.spin_once(timeout_sec=0.02)
            collector.publish_blank()
            end = time.time() + 5.0
            while time.time() < end and not collector.received:
                ex.spin_once(timeout_sec=0.05)
            result = {
                'received_count': len(collector.received),
                'first_detections': (
                    len(collector.received[0].detections)
                    if collector.received else None
                ),
            }
            print('RESULT_JSON=' + json.dumps(result))
            return 0 if collector.received else 1
        finally:
            detector.destroy_node()
            collector.destroy_node()
            rclpy.shutdown()


    sys.exit(main())
    """
)


@pytest.mark.integration
def test_blank_image_yields_empty_detection2darray(tmp_path: Path) -> None:
    """Spawn a fresh Python process so legacy test sys.modules patches don't leak in."""
    script = tmp_path / "harness.py"
    script.write_text(HARNESS_SCRIPT)

    # Bash -c to source ROS 2 then run python harness.
    cmd = [
        "bash",
        "-c",
        f"source {ROS_SETUP} && python3 {script}",
    ]
    env = dict(os.environ)
    # Use a unique domain id to keep parallel runs isolated.
    env.setdefault("ROS_DOMAIN_ID", str(os.getpid() % 100 + 50))

    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30, env=env)

    stdout = proc.stdout
    stderr = proc.stderr
    # Locate the JSON line emitted by the harness.
    payload = None
    for line in stdout.splitlines():
        if line.startswith("RESULT_JSON="):
            payload = json.loads(line.split("=", 1)[1])
            break

    assert payload is not None, (
        "harness did not emit RESULT_JSON.\n"
        f"--- STDOUT ---\n{stdout}\n--- STDERR ---\n{stderr}"
    )
    assert payload["received_count"] >= 1, (
        f"no Detection2DArray received. stderr:\n{stderr}"
    )
    # Blank image → empty detections array, but the message itself must publish.
    assert payload["first_detections"] == 0
