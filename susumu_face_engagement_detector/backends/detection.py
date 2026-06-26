"""Face detection backends with a uniform interface.

A backend takes a BGR ndarray and returns:
    (boxes_with_scores, encodings)
where:
    boxes_with_scores: list[(top, right, bottom, left, score)]
    encodings: list[ndarray] or None

dlib_hog/dlib_cnn produce 128-D encodings as a side product (face_recognition
embeds during detection). yunet/mediapipe do not; downstream
face_recognition_node must compute encodings separately when those backends
are active.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


BoxScore = Tuple[int, int, int, int, float]  # top, right, bottom, left, score


@dataclass
class DetectionResult:
    boxes: List[BoxScore]
    encodings: Optional[List[np.ndarray]]  # None when backend does not compute them


class DetectionBackend:
    """Abstract base — subclasses override detect()."""

    name: str = "abstract"

    def detect(self, bgr: np.ndarray) -> DetectionResult:
        raise NotImplementedError


class DlibHogBackend(DetectionBackend):
    name = "dlib_hog"

    def __init__(self) -> None:
        import face_recognition  # imported lazily for CI without dlib
        self._fr = face_recognition

    def detect(self, bgr: np.ndarray) -> DetectionResult:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        locations = self._fr.face_locations(rgb, model="hog")
        encodings = self._fr.face_encodings(rgb, locations) if locations else []
        boxes: List[BoxScore] = [(t, r, b, l, 1.0) for t, r, b, l in locations]
        return DetectionResult(boxes=boxes, encodings=encodings)


class DlibCnnBackend(DetectionBackend):
    name = "dlib_cnn"

    def __init__(self) -> None:
        import face_recognition
        self._fr = face_recognition

    def detect(self, bgr: np.ndarray) -> DetectionResult:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        locations = self._fr.face_locations(rgb, model="cnn")
        encodings = self._fr.face_encodings(rgb, locations) if locations else []
        boxes: List[BoxScore] = [(t, r, b, l, 1.0) for t, r, b, l in locations]
        return DetectionResult(boxes=boxes, encodings=encodings)


class YuNetBackend(DetectionBackend):
    """OpenCV YuNet ONNX — Apache 2.0, commercial OK."""

    name = "yunet"

    def __init__(self, model_path: str, score_threshold: float = 0.5,
                 nms_threshold: float = 0.3) -> None:
        if not model_path:
            raise ValueError(
                "yunet backend requires model_path "
                "(e.g. face_detection_yunet_2023mar.onnx from OpenCV Zoo)"
            )
        self._detector = cv2.FaceDetectorYN_create(
            model_path, "", (320, 320),
            score_threshold=score_threshold, nms_threshold=nms_threshold,
        )

    def detect(self, bgr: np.ndarray) -> DetectionResult:
        h, w = bgr.shape[:2]
        self._detector.setInputSize((w, h))
        _retval, faces = self._detector.detect(bgr)
        boxes: List[BoxScore] = []
        if faces is None:
            return DetectionResult(boxes=[], encodings=None)
        for f in faces:
            x, y, bw, bh = float(f[0]), float(f[1]), float(f[2]), float(f[3])
            score = float(f[-1])
            top, left = int(round(y)), int(round(x))
            bottom, right = int(round(y + bh)), int(round(x + bw))
            boxes.append((top, right, bottom, left, score))
        return DetectionResult(boxes=boxes, encodings=None)


_BACKENDS = {
    "dlib_hog": DlibHogBackend,
    "dlib_cnn": DlibCnnBackend,
    "yunet": YuNetBackend,
}


def make_backend(name: str, **kwargs) -> DetectionBackend:
    """Factory — instantiate the named backend with backend-specific kwargs.

    Unknown kwargs are silently dropped so the caller can pass a single config
    dict regardless of backend choice.
    """
    if name not in _BACKENDS:
        raise ValueError(f"unknown detection backend: {name!r} (choices: {list(_BACKENDS)})")
    cls = _BACKENDS[name]
    # Filter kwargs to those the constructor accepts.
    import inspect
    sig = inspect.signature(cls.__init__)
    accepted = {p for p in sig.parameters if p != "self"}
    cleaned = {k: v for k, v in kwargs.items() if k in accepted and v is not None}
    return cls(**cleaned)


def available_backends() -> List[str]:
    return list(_BACKENDS.keys())
