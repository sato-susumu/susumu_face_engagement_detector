"""Gaze-estimation backends.

The active backend is OpenVINO's ``gaze-estimation-adas-0002`` model. It
requires left/right eye crops plus head-pose angles; this module only returns a
result when those inputs can be produced. It intentionally does not provide a
landmark-only heuristic fallback.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np


BBox = Tuple[int, int, int, int]  # top, right, bottom, left
YPR = Tuple[float, float, float]

# MediaPipe FaceMesh landmark groups. Naming follows the subject's left/right,
# not the image side.
_LEFT_EYE_LANDMARKS = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
_RIGHT_EYE_LANDMARKS = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]


@dataclass(frozen=True)
class EyeCrop:
    image: np.ndarray
    box: BBox


@dataclass(frozen=True)
class GazeResult:
    vector: Tuple[float, float, float]
    yaw_deg: float
    pitch_deg: float
    left_eye_box: BBox
    right_eye_box: BBox
    eye_center: Tuple[float, float]


def _vector_to_angles(vector: Sequence[float]) -> Tuple[float, float]:
    x, y, z = [float(v) for v in vector[:3]]
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError(f"invalid gaze vector: {vector!r}")
    yaw = math.degrees(math.atan2(x, z if abs(z) > 1e-6 else 1e-6))
    pitch = math.degrees(math.atan2(y, math.sqrt(x * x + z * z)))
    return yaw, pitch


def _clip_box(box: BBox, width: int, height: int) -> Optional[BBox]:
    top, right, bottom, left = box
    top = max(0, min(height - 1, int(round(top))))
    bottom = max(0, min(height, int(round(bottom))))
    left = max(0, min(width - 1, int(round(left))))
    right = max(0, min(width, int(round(right))))
    if right <= left or bottom <= top:
        return None
    return top, right, bottom, left


def _landmark_box(
    landmarks,
    ids: Sequence[int],
    width: int,
    height: int,
    padding_ratio: float,
) -> Optional[BBox]:
    points = np.array([(landmarks[i].x * width, landmarks[i].y * height) for i in ids], dtype=np.float32)
    if points.size == 0 or not np.isfinite(points).all():
        return None
    x, y, w, h = cv2.boundingRect(points)
    pad = int(round(max(w, h) * padding_ratio))
    return _clip_box((y - pad, x + w + pad, y + h + pad, x - pad), width, height)


def _crop_eye(
    bgr: np.ndarray,
    landmarks,
    ids: Sequence[int],
    padding_ratio: float,
    min_eye_size: int,
) -> Optional[EyeCrop]:
    h, w = bgr.shape[:2]
    box = _landmark_box(landmarks, ids, w, h, padding_ratio)
    if box is None:
        return None
    top, right, bottom, left = box
    if min(right - left, bottom - top) < min_eye_size:
        return None
    crop = bgr[top:bottom, left:right]
    if crop.size == 0:
        return None
    return EyeCrop(crop, box)


def _to_nchw_bgr(image: np.ndarray, size: Tuple[int, int]) -> np.ndarray:
    resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    return resized.transpose(2, 0, 1)[None, ...].astype(np.float32)


class GazeBackend:
    name: str = "abstract"

    def estimate(self, bgr: np.ndarray, head_pose_ypr: Optional[YPR]) -> Optional[GazeResult]:
        raise NotImplementedError


class OpenVINOGazeBackend(GazeBackend):
    """OpenVINO ``gaze-estimation-adas-0002`` backend."""

    name = "openvino_adas"

    def __init__(
        self,
        model_path: str,
        device: str = "CPU",
        eye_input_size: int = 60,
        min_eye_size: int = 8,
        eye_padding_ratio: float = 0.55,
        max_head_yaw_deg: float = 45.0,
        max_head_pitch_deg: float = 35.0,
    ) -> None:
        if not model_path:
            raise ValueError("gaze model_path is required for openvino_adas")
        path = Path(model_path).expanduser()
        if not path.exists():
            raise FileNotFoundError(path)

        try:
            from openvino import Core  # type: ignore
        except ImportError:  # pragma: no cover - depends on installed OpenVINO version
            from openvino.runtime import Core  # type: ignore

        import mediapipe as mp  # type: ignore

        self._core = Core()
        self._model = self._core.read_model(str(path))
        self._compiled = self._core.compile_model(self._model, device)
        self._infer_request = self._compiled.create_infer_request()
        self._inputs = self._resolve_inputs(self._compiled.inputs)
        self._output = self._compiled.outputs[0]
        self._eye_input_size = int(eye_input_size)
        self._min_eye_size = int(min_eye_size)
        self._eye_padding_ratio = float(eye_padding_ratio)
        self._max_head_yaw = float(max_head_yaw_deg)
        self._max_head_pitch = float(max_head_pitch_deg)
        self._mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    @staticmethod
    def _port_name(port) -> str:
        try:
            return str(port.get_any_name())
        except Exception:
            return str(getattr(port, "any_name", ""))

    @classmethod
    def _resolve_inputs(cls, ports) -> Dict[str, object]:
        resolved: Dict[str, object] = {}
        unresolved = []
        for port in ports:
            name = cls._port_name(port)
            lower = name.lower()
            if "left_eye" in lower:
                resolved["left_eye_image"] = port
            elif "right_eye" in lower:
                resolved["right_eye_image"] = port
            elif "head_pose" in lower:
                resolved["head_pose_angles"] = port
            else:
                unresolved.append(port)

        if "head_pose_angles" not in resolved:
            for port in unresolved:
                try:
                    shape = [int(dim) for dim in port.get_partial_shape() if dim.is_static]
                except Exception:
                    try:
                        shape = [int(dim) for dim in port.shape]
                    except Exception:
                        shape = []
                if len(shape) == 2 and shape[-1] == 3:
                    resolved["head_pose_angles"] = port
                    break

        missing = {"left_eye_image", "right_eye_image", "head_pose_angles"} - set(resolved)
        if missing:
            names = [cls._port_name(port) for port in ports]
            raise RuntimeError(f"gaze model inputs missing {sorted(missing)}; got {names}")
        return resolved

    def estimate(self, bgr: np.ndarray, head_pose_ypr: Optional[YPR]) -> Optional[GazeResult]:
        if head_pose_ypr is None or bgr.size == 0:
            return None
        yaw, pitch, roll = [float(v) for v in head_pose_ypr]
        if not all(math.isfinite(v) for v in (yaw, pitch, roll)):
            return None
        if abs(yaw) > self._max_head_yaw or abs(pitch) > self._max_head_pitch:
            return None

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mesh_result = self._mesh.process(rgb)
        if not mesh_result.multi_face_landmarks:
            return None
        landmarks = mesh_result.multi_face_landmarks[0].landmark

        left_eye = _crop_eye(
            bgr,
            landmarks,
            _LEFT_EYE_LANDMARKS,
            self._eye_padding_ratio,
            self._min_eye_size,
        )
        right_eye = _crop_eye(
            bgr,
            landmarks,
            _RIGHT_EYE_LANDMARKS,
            self._eye_padding_ratio,
            self._min_eye_size,
        )
        if left_eye is None or right_eye is None:
            return None

        input_size = (self._eye_input_size, self._eye_input_size)
        head_pose = np.array([[yaw, pitch, roll]], dtype=np.float32)
        inputs = {
            self._inputs["left_eye_image"]: _to_nchw_bgr(left_eye.image, input_size),
            self._inputs["right_eye_image"]: _to_nchw_bgr(right_eye.image, input_size),
            self._inputs["head_pose_angles"]: head_pose,
        }
        output = self._infer_request.infer(inputs)[self._output]
        vector_arr = np.asarray(output, dtype=np.float32).reshape(-1)[:3]
        norm = float(np.linalg.norm(vector_arr))
        if norm > 1e-6:
            vector_arr = vector_arr / norm
        vector = tuple(float(v) for v in vector_arr)
        gaze_yaw, gaze_pitch = _vector_to_angles(vector)

        left_center = ((left_eye.box[1] + left_eye.box[3]) / 2.0, (left_eye.box[0] + left_eye.box[2]) / 2.0)
        right_center = ((right_eye.box[1] + right_eye.box[3]) / 2.0, (right_eye.box[0] + right_eye.box[2]) / 2.0)
        eye_center = ((left_center[0] + right_center[0]) / 2.0, (left_center[1] + right_center[1]) / 2.0)

        return GazeResult(
            vector=vector,
            yaw_deg=gaze_yaw,
            pitch_deg=gaze_pitch,
            left_eye_box=left_eye.box,
            right_eye_box=right_eye.box,
            eye_center=eye_center,
        )


_BACKENDS = {"openvino_adas": OpenVINOGazeBackend}


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
