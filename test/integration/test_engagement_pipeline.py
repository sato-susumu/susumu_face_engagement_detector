"""End-to-end test of the new engagement pipeline.

Spawns engagement_node + a harness that feeds synthetic
expression / head_pose messages and verifies the engagement_status
topic publishes a non-UNKNOWN level.

Runs in a subprocess to avoid the legacy cv2/cv_bridge mock pollution.
"""
from __future__ import annotations

import json
import os
import subprocess
import textwrap
from pathlib import Path

import pytest


ROS_SETUP = Path("/opt/ros/humble/setup.bash")
pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not ROS_SETUP.exists(), reason="ROS 2 Humble setup.bash not found"),
]


HARNESS_SCRIPT = textwrap.dedent("""\
    import json
    import sys
    import time

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from hri_msgs.msg import EngagementLevel, Expression

    from susumu_face_engagement_detector.engagement_node import EngagementNode


    class Driver(Node):
        def __init__(self):
            super().__init__('engagement_driver')
            self.expr_pub = self.create_publisher(Expression, '/humans/faces/expression', 10)
            self.pose_pub = self.create_publisher(PoseStamped, '/humans/faces/head_pose', 10)
            self.received_levels = []
            self.create_subscription(
                EngagementLevel,
                '/humans/persons/default/engagement_status',
                lambda m: self.received_levels.append(m.level),
                10,
            )

        def publish_engaged_signals(self):
            expr = Expression()
            expr.header.stamp = self.get_clock().now().to_msg()
            expr.expression = 'neutral'  # maximum-weight emotion
            expr.confidence = 0.9
            self.expr_pub.publish(expr)

            p = PoseStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = 'camera_color_optical_frame'
            # Identity rotation == yaw=pitch=roll=0
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            self.pose_pub.publish(p)


    def main():
        # update_hz=20 so the scorer ticks fast enough to settle in ~2s.
        rclpy.init(args=[
            '--ros-args',
            '-p', 'update_hz:=20.0',
            '-p', 'engaged_hold:=2',
        ])
        engagement = EngagementNode()
        driver = Driver()
        ex = SingleThreadedExecutor()
        ex.add_node(engagement)
        ex.add_node(driver)
        try:
            # Pump connections.
            for _ in range(10):
                ex.spin_once(timeout_sec=0.02)
            end = time.time() + 5.0
            while time.time() < end:
                driver.publish_engaged_signals()
                for _ in range(3):
                    ex.spin_once(timeout_sec=0.02)

            result = {
                'received_count': len(driver.received_levels),
                'levels': driver.received_levels[-5:],
                'max_level': max(driver.received_levels) if driver.received_levels else None,
            }
            print('RESULT_JSON=' + json.dumps(result))
            return 0
        finally:
            engagement.destroy_node()
            driver.destroy_node()
            rclpy.shutdown()


    sys.exit(main())
    """)


def test_engagement_node_reaches_engaged_with_strong_signals(tmp_path: Path) -> None:
    script = tmp_path / "engagement_harness.py"
    script.write_text(HARNESS_SCRIPT)
    cmd = ["bash", "-c", f"source {ROS_SETUP} && python3 {script}"]
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", str((os.getpid() + 17) % 100 + 50))
    repo_root = Path(__file__).resolve().parents[2]
    env["PYTHONPATH"] = str(repo_root) + os.pathsep + env.get("PYTHONPATH", "")

    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30, env=env)
    payload = None
    for line in proc.stdout.splitlines():
        if line.startswith("RESULT_JSON="):
            payload = json.loads(line.split("=", 1)[1])
            break
    assert payload is not None, (
        f"no RESULT_JSON\nSTDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
    )
    assert payload["received_count"] >= 5
    # 3 = ENGAGED. The scorer must reach it given 5s of perfect inputs.
    assert payload["max_level"] == 3, (
        f"expected to reach ENGAGED (3), got max={payload['max_level']} "
        f"with tail levels={payload['levels']}"
    )
