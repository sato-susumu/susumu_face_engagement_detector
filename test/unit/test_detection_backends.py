"""Backend factory + interface contract tests."""
from __future__ import annotations

from pathlib import Path

import pytest

from susumu_face_engagement_detector.backends.detection import (
    DetectionBackend,
    DetectionResult,
    available_backends,
    make_backend,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
YUNET_PATH = Path.home() / "models" / "face_detection" / "face_detection_yunet_2023mar.onnx"


def test_available_backends_includes_known_set() -> None:
    backends = available_backends()
    assert "dlib_hog" in backends
    assert "yunet" in backends


def test_unknown_backend_raises() -> None:
    with pytest.raises(ValueError, match="unknown detection backend"):
        make_backend("nonexistent")


def test_make_dlib_hog_returns_backend() -> None:
    pytest.importorskip("face_recognition")
    backend = make_backend("dlib_hog")
    assert isinstance(backend, DetectionBackend)
    assert backend.name == "dlib_hog"


def test_unused_kwargs_silently_dropped() -> None:
    """The factory must accept extra kwargs so the node can pass a config dict."""
    pytest.importorskip("face_recognition")
    # dlib_hog has no ctor args — must not raise with extras present.
    backend = make_backend("dlib_hog", model_path="/nonexistent", score_threshold=0.99)
    assert backend.name == "dlib_hog"


@pytest.mark.skipif(not YUNET_PATH.exists(), reason="YuNet weights not downloaded")
def test_yunet_detection_on_sample_image() -> None:
    import cv2
    backend = make_backend("yunet", model_path=str(YUNET_PATH))
    img_path = Path.home() / "datasets" / "wider_face" / "WIDER_val" / "images" / "0--Parade" / "0_Parade_Parade_0_12.jpg"
    if not img_path.exists():
        pytest.skip("WIDER FACE val not downloaded")
    img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
    result = backend.detect(img)
    assert isinstance(result, DetectionResult)
    assert len(result.boxes) >= 1
    # YuNet returns no encodings.
    assert result.encodings is None
    for (top, right, bottom, left, score) in result.boxes:
        assert 0.0 <= score <= 1.0
        assert right > left and bottom > top
