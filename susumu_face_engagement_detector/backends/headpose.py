"""Head pose backends.

Each backend takes a BGR ndarray and an optional bbox (top, right, bottom, left)
and returns (yaw_deg, pitch_deg, roll_deg) where:
  yaw   = positive looking right (from subject's POV) — head turns right
  pitch = positive looking up
  roll  = positive tilting right ear toward right shoulder

MediaPipe backend uses Face Landmarker + solvePnP against a canonical
3-D head model — Apache 2.0, CPU-ready, ~30 FPS on a single core.

6DRepNet slot is provided but inactive until the MIT weights are placed at
~/models/head_pose/sixdrepnet360_*.pth and a follow-up implements the loader.
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


YPR = Tuple[float, float, float]
BBox = Tuple[int, int, int, int]  # top, right, bottom, left


# A canonical 3-D head model with 6 landmarks used for solvePnP. Coordinates
# in millimetres, +X right, +Y up, +Z out of the face. Values come from the
# OpenCV head pose example (sellion, chin, left/right eye outer, left/right
# mouth corner).
_MODEL_POINTS_6 = np.array(
    [
        [0.0, 0.0, 0.0],          # 1: tip of the nose / sellion (ish)
        [0.0, -63.6, -12.5],      # 2: chin
        [-43.3, 32.7, -26.0],     # 3: left eye outer corner
        [43.3, 32.7, -26.0],      # 4: right eye outer corner
        [-28.9, -28.9, -24.1],    # 5: left mouth corner
        [28.9, -28.9, -24.1],     # 6: right mouth corner
    ],
    dtype=np.float64,
)

# MediaPipe Face Mesh landmark IDs corresponding to the model points above.
# See https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/modules/face_geometry/data/canonical_face_model_uv_visualization.png
_MEDIAPIPE_KEY_LANDMARKS = [4, 152, 33, 263, 61, 291]


def _rvec_to_ypr(rvec: np.ndarray) -> YPR:
    """Convert an OpenCV rotation vector to (yaw, pitch, roll) in degrees."""
    rmat, _ = cv2.Rodrigues(rvec)
    # Decompose into Euler angles via R = Rz(roll) Ry(yaw) Rx(pitch) convention.
    sy = float(np.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2))
    singular = sy < 1e-6
    if not singular:
        pitch = float(np.degrees(np.arctan2(rmat[2, 1], rmat[2, 2])))
        yaw = float(np.degrees(np.arctan2(-rmat[2, 0], sy)))
        roll = float(np.degrees(np.arctan2(rmat[1, 0], rmat[0, 0])))
    else:
        pitch = float(np.degrees(np.arctan2(-rmat[1, 2], rmat[1, 1])))
        yaw = float(np.degrees(np.arctan2(-rmat[2, 0], sy)))
        roll = 0.0
    return yaw, pitch, roll


class HeadPoseBackend:
    name: str = "abstract"

    def estimate(self, bgr: np.ndarray, bbox: Optional[BBox] = None) -> Optional[YPR]:
        raise NotImplementedError


class MediaPipePnPBackend(HeadPoseBackend):
    """MediaPipe Face Mesh landmarks → 6-point solvePnP head pose."""

    name = "mediapipe_pnp"

    def __init__(self) -> None:
        import mediapipe as mp  # type: ignore
        self._mp = mp
        # Static-image mode is appropriate when callers feed one frame at a time.
        self._mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=0.3,
            min_tracking_confidence=0.3,
        )

    def estimate(self, bgr: np.ndarray, bbox: Optional[BBox] = None) -> Optional[YPR]:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self._mesh.process(rgb)
        if not res.multi_face_landmarks:
            return None
        lm = res.multi_face_landmarks[0].landmark
        image_points = np.array(
            [(lm[i].x * w, lm[i].y * h) for i in _MEDIAPIPE_KEY_LANDMARKS],
            dtype=np.float64,
        )
        # Pinhole camera model with focal_length = image width — good enough
        # for relative head pose when no calibrated intrinsics are available.
        focal = float(w)
        cam_mat = np.array(
            [[focal, 0.0, w / 2.0], [0.0, focal, h / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dist = np.zeros((4, 1))
        ok, rvec, _tvec = cv2.solvePnP(
            _MODEL_POINTS_6, image_points, cam_mat, dist, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            return None
        return _rvec_to_ypr(rvec)


_BACKENDS = {"mediapipe_pnp": MediaPipePnPBackend}


def make_backend(name: str, **kwargs) -> HeadPoseBackend:
    if name not in _BACKENDS:
        raise ValueError(f"unknown head pose backend: {name!r} (choices: {list(_BACKENDS)})")
    cls = _BACKENDS[name]
    import inspect
    sig = inspect.signature(cls.__init__)
    accepted = {p for p in sig.parameters if p != "self"}
    cleaned = {k: v for k, v in kwargs.items() if k in accepted and v is not None}
    return cls(**cleaned)


def available_backends() -> list:
    return list(_BACKENDS.keys())
