"""Verify that vision_msgs / hri_msgs definitions used by this package import.

Phase 1 deliverable: the package now claims to publish vision_msgs
Detection2DArray (parallel to its legacy String output). This test fails
loudly if the message classes (or their dependencies) ever become
unavailable in the build environment.

These tests are dependency-light — they neither spin a ROS context nor
publish — so they run in CI without an active middleware.
"""
from __future__ import annotations


def test_vision_msgs_detection2darray_importable() -> None:
    from vision_msgs.msg import (
        Detection2D,
        Detection2DArray,
        BoundingBox2D,
        ObjectHypothesisWithPose,
    )
    # Smoke construction with required fields
    arr = Detection2DArray()
    d = Detection2D()
    d.id = "face_1"
    bb = BoundingBox2D()
    bb.center.position.x = 100.0
    bb.center.position.y = 200.0
    bb.size_x = 50.0
    bb.size_y = 60.0
    d.bbox = bb
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = "face"
    hyp.hypothesis.score = 0.95
    d.results.append(hyp)
    arr.detections.append(d)
    assert len(arr.detections) == 1
    assert arr.detections[0].results[0].hypothesis.score == 0.95


def test_hri_msgs_engagementlevel_has_5_states() -> None:
    """REP-155 / hri_msgs/EngagementLevel must define the five canonical states."""
    from hri_msgs.msg import EngagementLevel

    # The five constants we plan to publish from engagement_node in Phase 4.
    for name in ["UNKNOWN", "DISENGAGED", "ENGAGING", "ENGAGED", "DISENGAGING"]:
        assert hasattr(EngagementLevel, name), f"missing constant: {name}"


def test_hri_msgs_idslist_importable() -> None:
    from hri_msgs.msg import IdsList  # noqa: F401


def test_face_detection_node_module_loads() -> None:
    """The modified face_detection_node imports cleanly with new vision_msgs deps."""
    # Importing the node module should not raise.
    from susumu_face_engagement_detector import face_detection_node  # noqa: F401


def test_gaze_node_module_loads() -> None:
    from susumu_face_engagement_detector import gaze_node  # noqa: F401
