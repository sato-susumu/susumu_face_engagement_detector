"""Facial expression recognition backends.

Each backend takes a BGR face crop and returns:
    (label_str, probabilities: dict[label_str, float])

HSEmotion is the primary backend — Apache 2.0, includes 8-class AffectNet
model weights that can be redistributed.
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

import cv2
import numpy as np


EmotionResult = Tuple[str, Dict[str, float]]


# Canonical label set for the engagement scoring layer. Backends normalise
# their own label vocabularies into this.
CANONICAL_EMOTIONS = (
    "neutral", "happy", "sad", "anger", "surprise", "fear", "disgust", "contempt",
)


class ExpressionBackend:
    name: str = "abstract"

    def predict(self, bgr_face: np.ndarray) -> Optional[EmotionResult]:
        raise NotImplementedError


def _normalise_label(label: str) -> str:
    """Map a backend-specific label to the canonical set."""
    low = label.strip().lower()
    aliases = {
        "happiness": "happy",
        "sadness": "sad",
        "angry": "anger",
        "anger": "anger",
        "surprise": "surprise",
    }
    canonical = aliases.get(low, low)
    if canonical in CANONICAL_EMOTIONS:
        return canonical
    return low  # leave as-is; engagement layer will treat unknown as neutral-weight


class HSEmotionBackend(ExpressionBackend):
    """HSEmotion ONNX EfficientNet — AffectNet 8-class."""

    name = "hsemotion"

    def __init__(self, model_name: str = "enet_b0_8_best_afew") -> None:
        from hsemotion_onnx.facial_emotions import HSEmotionRecognizer
        self._r = HSEmotionRecognizer(model_name=model_name)
        # Sorted by index — feed into dict in normalised form.
        self._labels = [self._r.idx_to_class[i] for i in sorted(self._r.idx_to_class)]

    def predict(self, bgr_face: np.ndarray) -> Optional[EmotionResult]:
        if bgr_face is None or bgr_face.size == 0:
            return None
        # HSEmotion accepts BGR ndarray directly.
        emotion, scores = self._r.predict_emotions(bgr_face, logits=False)
        scores = np.asarray(scores).flatten()
        if scores.size != len(self._labels):
            return None
        probs = {_normalise_label(lbl): float(s) for lbl, s in zip(self._labels, scores)}
        return _normalise_label(emotion), probs


_BACKENDS = {"hsemotion": HSEmotionBackend}


def make_backend(name: str, **kwargs) -> ExpressionBackend:
    if name not in _BACKENDS:
        raise ValueError(f"unknown expression backend: {name!r} (choices: {list(_BACKENDS)})")
    cls = _BACKENDS[name]
    import inspect
    sig = inspect.signature(cls.__init__)
    accepted = {p for p in sig.parameters if p != "self"}
    cleaned = {k: v for k, v in kwargs.items() if k in accepted and v is not None}
    return cls(**cleaned)


def available_backends() -> list:
    return list(_BACKENDS.keys())
