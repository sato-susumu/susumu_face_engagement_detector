"""Gaze backend factory + angle helper tests."""
import math

import pytest

from susumu_face_engagement_detector.backends.gaze import (
    angle_to_camera_axis,
    available_backends,
    make_backend,
)


def test_available_backends_lists_mediapipe_iris() -> None:
    assert "mediapipe_iris" in available_backends()


def test_unknown_backend_raises() -> None:
    with pytest.raises(ValueError, match="unknown gaze backend"):
        make_backend("nope")


def test_angle_forward_is_zero() -> None:
    assert angle_to_camera_axis((0.0, 0.0, 1.0)) == pytest.approx(0.0, abs=1e-6)


def test_angle_right_is_90() -> None:
    assert angle_to_camera_axis((1.0, 0.0, 0.0)) == pytest.approx(90.0, abs=1e-6)


def test_angle_backward_is_180() -> None:
    assert angle_to_camera_axis((0.0, 0.0, -1.0)) == pytest.approx(180.0, abs=1e-3)


def test_angle_diagonal() -> None:
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    assert angle_to_camera_axis((inv_sqrt2, 0.0, inv_sqrt2)) == pytest.approx(45.0, abs=1e-3)
