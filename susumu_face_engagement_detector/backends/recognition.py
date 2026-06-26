"""Face embedding (recognition) backends.

Each backend takes (rgb_image, bbox) — bbox in (top, right, bottom, left) px —
and returns a 1-D embedding ndarray, or None if no face is found.

Phase 2 ships dlib_128d only. MagFace / ArcFace (ONNX) are slotted in here
when their weights are downloaded.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np


BBox = Tuple[int, int, int, int]  # top, right, bottom, left


class RecognitionBackend:
    name: str = "abstract"
    embedding_dim: int = 0

    def embed(self, rgb: np.ndarray, bbox: BBox) -> Optional[np.ndarray]:
        raise NotImplementedError


class DlibBackend(RecognitionBackend):
    """face_recognition ResNet-34 128-D embeddings."""

    name = "dlib_128d"
    embedding_dim = 128

    def __init__(self) -> None:
        import face_recognition
        self._fr = face_recognition

    def embed(self, rgb: np.ndarray, bbox: BBox) -> Optional[np.ndarray]:
        encs = self._fr.face_encodings(rgb, [bbox])
        if not encs:
            return None
        return encs[0]


_BACKENDS = {"dlib_128d": DlibBackend}


def make_backend(name: str, **kwargs) -> RecognitionBackend:
    if name not in _BACKENDS:
        raise ValueError(f"unknown recognition backend: {name!r} (choices: {list(_BACKENDS)})")
    cls = _BACKENDS[name]
    import inspect
    sig = inspect.signature(cls.__init__)
    accepted = {p for p in sig.parameters if p != "self"}
    cleaned = {k: v for k, v in kwargs.items() if k in accepted and v is not None}
    return cls(**cleaned)


def available_backends() -> List[str]:
    return list(_BACKENDS.keys())
