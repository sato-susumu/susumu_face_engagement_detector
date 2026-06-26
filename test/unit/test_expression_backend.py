"""Tests for the expression backend factory + label normaliser."""
from pathlib import Path

import numpy as np
import pytest

from susumu_face_engagement_detector.backends.expression import (
    CANONICAL_EMOTIONS,
    _normalise_label,
    available_backends,
    make_backend,
)


HSEMOTION_MODEL = Path.home() / ".hsemotion" / "enet_b0_8_best_afew.onnx"


def test_available_backends_lists_hsemotion() -> None:
    assert "hsemotion" in available_backends()


def test_unknown_backend_raises() -> None:
    with pytest.raises(ValueError, match="unknown expression backend"):
        make_backend("nope")


def test_canonical_emotions_includes_basics() -> None:
    for label in ("neutral", "happy", "sad", "anger", "surprise"):
        assert label in CANONICAL_EMOTIONS


def test_normalise_label_handles_aliases() -> None:
    assert _normalise_label("Happiness") == "happy"
    assert _normalise_label("Sadness") == "sad"
    assert _normalise_label("Neutral") == "neutral"
    assert _normalise_label("Angry") == "anger"


@pytest.mark.skipif(not HSEMOTION_MODEL.exists(),
                    reason="HSEmotion ONNX weights not cached under ~/.hsemotion/")
def test_hsemotion_predicts_on_synthetic_image() -> None:
    backend = make_backend("hsemotion")
    fake = np.full((100, 100, 3), 128, dtype=np.uint8)
    result = backend.predict(fake)
    assert result is not None
    label, probs = result
    assert isinstance(label, str)
    assert isinstance(probs, dict)
    assert abs(sum(probs.values()) - 1.0) < 0.05  # softmax sums to ~1
