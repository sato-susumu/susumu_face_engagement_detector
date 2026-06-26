"""Sanity tests for the head pose backend factory."""
import numpy as np
import pytest

from susumu_face_engagement_detector.backends.headpose import (
    _rvec_to_ypr,
    available_backends,
    make_backend,
)


def test_available_backends_lists_mediapipe() -> None:
    assert "mediapipe_pnp" in available_backends()


def test_unknown_backend_raises() -> None:
    with pytest.raises(ValueError, match="unknown head pose backend"):
        make_backend("nonexistent")


def test_rvec_zero_is_zero_ypr() -> None:
    rvec = np.zeros((3, 1))
    yaw, pitch, roll = _rvec_to_ypr(rvec)
    assert yaw == pytest.approx(0.0, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-6)
    assert roll == pytest.approx(0.0, abs=1e-6)


def test_rvec_yaw_90deg() -> None:
    # Rotation of +90° around the Y axis (yaw right). Rodrigues vector is
    # axis * angle, so (0, pi/2, 0) — but the way we decompose puts that
    # into the 'yaw' slot.
    rvec = np.array([[0.0], [np.pi / 2], [0.0]])
    yaw, _pitch, _roll = _rvec_to_ypr(rvec)
    assert abs(abs(yaw) - 90.0) < 1.0
