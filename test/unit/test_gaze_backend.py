import numpy as np
import pytest

from susumu_face_engagement_detector.backends.gaze import (
    _to_nchw_bgr,
    _vector_to_angles,
    available_backends,
    make_backend,
)


def test_available_backends_lists_openvino_adas() -> None:
    assert "openvino_adas" in available_backends()


def test_unknown_backend_raises() -> None:
    with pytest.raises(ValueError, match="unknown gaze backend"):
        make_backend("does_not_exist")


def test_vector_to_angles_front() -> None:
    yaw, pitch = _vector_to_angles((0.0, 0.0, 1.0))
    assert yaw == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)


def test_vector_to_angles_right_and_up() -> None:
    yaw, pitch = _vector_to_angles((1.0, 1.0, 1.0))
    assert yaw == pytest.approx(45.0)
    assert pitch == pytest.approx(35.264, abs=1e-3)


def test_eye_preprocess_returns_nchw_float() -> None:
    image = np.zeros((10, 20, 3), dtype=np.uint8)
    blob = _to_nchw_bgr(image, (60, 60))
    assert blob.shape == (1, 3, 60, 60)
    assert blob.dtype == np.float32
