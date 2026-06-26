"""Gaze direction estimation backends.

Each backend takes a BGR ndarray and returns a 3-D unit vector in the camera
optical frame (X right, Y down, Z forward). None if no face is found.

MediaPipe iris backend is a geometric approximation — it uses the iris centre
relative to the eye corners to derive a rough yaw/pitch, then combines that
with the head pose's forward vector. It is intentionally fast and
license-clean (Apache 2.0) for CPU pipelines. L2CS-Net (MIT, GPU) slots in
here as a follow-up.
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


GazeVec = Tuple[float, float, float]  # unit vector (x, y, z) in camera optical frame


# MediaPipe Face Mesh landmark IDs for the iris and eye corners.
# Left eye (subject's left, camera right):
_LEFT_IRIS_CENTER = 468
_LEFT_EYE_INNER = 133
_LEFT_EYE_OUTER = 33
# Right eye (subject's right, camera left):
_RIGHT_IRIS_CENTER = 473
_RIGHT_EYE_INNER = 362
_RIGHT_EYE_OUTER = 263


def _normalise(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    return v / n


class GazeBackend:
    name: str = "abstract"

    def estimate(self, bgr: np.ndarray) -> Optional[GazeVec]:
        raise NotImplementedError


class MediaPipeIrisBackend(GazeBackend):
    """Geometric gaze approximation from MediaPipe Face Mesh iris landmarks."""

    name = "mediapipe_iris"

    def __init__(self) -> None:
        import mediapipe as mp  # type: ignore
        self._mp = mp
        self._mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,  # required for iris landmarks 468-477
            min_detection_confidence=0.3,
            min_tracking_confidence=0.3,
        )

    def estimate(self, bgr: np.ndarray) -> Optional[GazeVec]:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self._mesh.process(rgb)
        if not res.multi_face_landmarks:
            return None
        lm = res.multi_face_landmarks[0].landmark

        # Compute normalised iris position within each eye [-1, +1] horizontal
        # and [-1, +1] vertical. -1 = looking outward, +1 = looking inward.
        def _iris_offset(iris_id: int, inner_id: int, outer_id: int) -> Tuple[float, float]:
            iris = np.array([lm[iris_id].x * w, lm[iris_id].y * h])
            inner = np.array([lm[inner_id].x * w, lm[inner_id].y * h])
            outer = np.array([lm[outer_id].x * w, lm[outer_id].y * h])
            eye_centre = (inner + outer) / 2.0
            eye_width = float(np.linalg.norm(inner - outer))
            if eye_width < 1.0:
                return 0.0, 0.0
            offset = iris - eye_centre
            # Horizontal: project on (outer - inner)/|...|
            horiz_axis = (outer - inner) / eye_width
            h_off = float(np.dot(offset, horiz_axis)) / (eye_width / 2.0)
            # Vertical: positive = down in image, but we want positive = up
            v_off = -float(offset[1]) / (eye_width / 2.0)
            return h_off, v_off

        lh, lv = _iris_offset(_LEFT_IRIS_CENTER, _LEFT_EYE_INNER, _LEFT_EYE_OUTER)
        rh, rv = _iris_offset(_RIGHT_IRIS_CENTER, _RIGHT_EYE_INNER, _RIGHT_EYE_OUTER)
        # Average both eyes — gives a reasonable gaze direction estimate.
        h_off = (lh + rh) / 2.0
        v_off = (lv + rv) / 2.0

        # Map iris offset to gaze angles. ~±0.5 offset corresponds to ~±30° gaze.
        # This is a coarse linear approximation — fine for distinguishing
        # "looking at camera" vs "looking away" but not for sub-degree accuracy.
        yaw_rad = float(np.clip(h_off, -1.0, 1.0)) * np.deg2rad(45.0)
        pitch_rad = float(np.clip(v_off, -1.0, 1.0)) * np.deg2rad(30.0)

        # Forward vector in camera optical frame (X right, Y down, Z forward)
        # is (0, 0, 1). Apply yaw then pitch.
        x = float(np.sin(yaw_rad))
        y = -float(np.sin(pitch_rad))
        z = float(np.cos(yaw_rad) * np.cos(pitch_rad))
        v = _normalise(np.array([x, y, z]))
        return float(v[0]), float(v[1]), float(v[2])


_BACKENDS = {"mediapipe_iris": MediaPipeIrisBackend}


def make_backend(name: str, **kwargs) -> GazeBackend:
    if name not in _BACKENDS:
        raise ValueError(f"unknown gaze backend: {name!r} (choices: {list(_BACKENDS)})")
    cls = _BACKENDS[name]
    import inspect
    sig = inspect.signature(cls.__init__)
    accepted = {p for p in sig.parameters if p != "self"}
    cleaned = {k: v for k, v in kwargs.items() if k in accepted and v is not None}
    return cls(**cleaned)


def available_backends() -> list:
    return list(_BACKENDS.keys())


def angle_to_camera_axis(gaze: GazeVec) -> float:
    """Return the angle in degrees between the gaze vector and the +Z axis."""
    v = _normalise(np.array(gaze))
    cos_a = float(np.clip(v[2], -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_a)))
