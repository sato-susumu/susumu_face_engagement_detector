"""New engagement ROS 2 node (Phase 4) — concentration-index scoring.

Inputs:
    /humans/faces/expression             hri_msgs/Expression
    /humans/faces/expression_scores      std_msgs/String (JSON probabilities)
    /humans/faces/head_pose              geometry_msgs/PoseStamped

Output:
    /humans/persons/default/engagement_status  hri_msgs/EngagementLevel
    /humans/persons/default/engagement_score   std_msgs/Float32  (raw filtered score)
    /engagement_event                          std_msgs/String  (legacy DETECTED/LOST/ENGAGED/DISENGAGED)

This Phase 4 implementation operates on a single anonymous person
("default") — Phase 5 multi-face routing comes via hri_person_manager.
"""
from __future__ import annotations

import json
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from hri_msgs.msg import EngagementLevel, Expression
    _HRI_AVAILABLE = True
except ImportError:  # pragma: no cover
    _HRI_AVAILABLE = False

from .engagement import (
    EngagementConfig,
    EngagementInputs,
    EngagementScorer,
    state_name,
)


# Map hri_msgs/Expression label → canonical label used by the scorer.
_HRI_TO_CANONICAL = {
    "neutral": "neutral",
    "happy": "happy",
    "sad": "sad",
    "angry": "anger",
    "surprised": "surprise",
    "scared": "fear",
    "disgusted": "disgust",
    # anything we don't recognise falls back to "neutral" weight
}


def _quat_to_yaw_pitch_roll(qx, qy, qz, qw):
    """Convert quaternion to (yaw, pitch, roll) degrees (ZYX convention)."""
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return yaw, pitch, roll


class EngagementNode(Node):
    def __init__(self) -> None:
        super().__init__('engagement_node')
        self._declare_params()
        self._scorer = EngagementScorer(EngagementConfig(
            yaw_gate_deg=float(self.get_parameter('yaw_gate_deg').value),
            pitch_gate_deg=float(self.get_parameter('pitch_gate_deg').value),
            gate_penalty=float(self.get_parameter('gate_penalty').value),
            ema_alpha=float(self.get_parameter('ema_alpha').value),
            engaged_threshold=float(self.get_parameter('engaged_threshold').value),
            engaging_threshold=float(self.get_parameter('engaging_threshold').value),
            disengaging_threshold=float(self.get_parameter('disengaging_threshold').value),
            disengaged_threshold=float(self.get_parameter('disengaged_threshold').value),
            engaged_hold=int(self.get_parameter('engaged_hold').value),
            disengaged_hold=int(self.get_parameter('disengaged_hold').value),
        ))
        person_id = self.get_parameter('person_id').value
        self._person_id = person_id

        # State latched from inbound topics
        self._last_emotion: Optional[str] = None
        self._last_yaw: Optional[float] = None
        self._last_pitch: Optional[float] = None
        self._last_level = None

        # Subscriptions
        if _HRI_AVAILABLE:
            self.create_subscription(Expression, '/humans/faces/expression',
                                     self._on_expression, 10)
        self.create_subscription(String, '/humans/faces/expression_scores',
                                 self._on_expression_scores, 10)
        self.create_subscription(PoseStamped, '/humans/faces/head_pose',
                                 self._on_head_pose, 10)

        # Publishers
        if _HRI_AVAILABLE:
            self._level_pub = self.create_publisher(
                EngagementLevel,
                f'/humans/persons/{person_id}/engagement_status', 10,
            )
        else:
            self._level_pub = None
        self._score_pub = self.create_publisher(
            Float32, f'/humans/persons/{person_id}/engagement_score', 10
        )
        self._event_pub = self.create_publisher(String, 'engagement_event', 10)

        # Tick at fixed rate so the EMA + hysteresis always update.
        period = 1.0 / float(self.get_parameter('update_hz').value)
        self.create_timer(period, self._on_tick)

        self.get_logger().info(
            f"Engagement Node started — person_id={person_id}, "
            f"update_hz={self.get_parameter('update_hz').value}"
        )
        if not _HRI_AVAILABLE:
            self.get_logger().warn("hri_msgs not importable — EngagementLevel topic suppressed")

    def _declare_params(self) -> None:
        for name, default in [
            ('person_id', 'default'),
            ('update_hz', 5.0),
            ('yaw_gate_deg', 15.0),
            ('pitch_gate_deg', 10.0),
            ('gate_penalty', 0.3),
            ('ema_alpha', 0.3),
            ('engaged_threshold', 0.66),
            ('engaging_threshold', 0.33),
            ('disengaging_threshold', 0.20),
            ('disengaged_threshold', 0.10),
            ('engaged_hold', 4),
            ('disengaged_hold', 4),
        ]:
            self.declare_parameter(name, default)

    def _on_expression(self, msg) -> None:
        self._last_emotion = _HRI_TO_CANONICAL.get(msg.expression, "neutral")

    def _on_expression_scores(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        label = data.get("label")
        if label is None:
            return
        # If we already have a canonical label from /expression, keep it.
        if self._last_emotion is None:
            self._last_emotion = label

    def _on_head_pose(self, msg) -> None:
        q = msg.pose.orientation
        yaw, pitch, _roll = _quat_to_yaw_pitch_roll(q.x, q.y, q.z, q.w)
        self._last_yaw = yaw
        self._last_pitch = pitch

    def _on_tick(self) -> None:
        inputs = EngagementInputs(
            emotion_label=self._last_emotion,
            yaw_deg=self._last_yaw,
            pitch_deg=self._last_pitch,
        )
        state = self._scorer.update(inputs)

        # Score topic always publishes.
        self._score_pub.publish(Float32(data=float(state.score)))

        # EngagementLevel — publish if hri_msgs is present.
        if self._level_pub is not None:
            lvl = EngagementLevel()
            lvl.header.stamp = self.get_clock().now().to_msg()
            lvl.level = int(state.level)
            self._level_pub.publish(lvl)

        # Legacy event — only on transitions, in DETECTED/LOST/ENGAGED/DISENGAGED form.
        if state.level != self._last_level:
            self._publish_legacy_event(state.level)
            self._last_level = state.level

    def _publish_legacy_event(self, level: int) -> None:
        from .engagement import ENGAGED, DISENGAGED, ENGAGING, DISENGAGING, UNKNOWN
        if level == ENGAGED:
            self._event_pub.publish(String(data=f"{self._person_id}:ENGAGED"))
        elif level == DISENGAGED:
            self._event_pub.publish(String(data=f"{self._person_id}:DISENGAGED"))
        elif level == ENGAGING:
            self._event_pub.publish(String(data=f"{self._person_id}:DETECTED"))
        elif level == DISENGAGING:
            self._event_pub.publish(String(data=f"{self._person_id}:LOST"))
        # UNKNOWN intentionally produces no legacy event.


def main(args=None) -> None:
    import rclpy
    rclpy.init(args=args)
    node = EngagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
