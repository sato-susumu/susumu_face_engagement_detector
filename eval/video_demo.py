"""Render the face-engagement pipeline onto a video file.

This module is intentionally ROS-free. It reuses the package backends directly
so a demo can be produced from a normal video file:

    python3 -m eval.video_demo --input input.webm --output outputs/runs/demo.mp4
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
import urllib.request
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from susumu_face_engagement_detector.backends.detection import make_backend as make_detection_backend
from susumu_face_engagement_detector.backends.expression import make_backend as make_expression_backend
from susumu_face_engagement_detector.backends.gaze import make_backend as make_gaze_backend
from susumu_face_engagement_detector.backends.headpose import (
    HeadPoseStabilizer,
    make_backend as make_headpose_backend,
    normalize_pose_angle,
)
from susumu_face_engagement_detector.backends.recognition import make_backend as make_recognition_backend
from susumu_face_engagement_detector.engagement import (
    EngagementInputs,
    EngagementScorer,
    state_name,
)


BBox = Tuple[int, int, int, int]  # top, right, bottom, left

DEFAULT_OUT_DIR = Path("outputs/runs/video_demo")
DEFAULT_OUT_PATH = DEFAULT_OUT_DIR / "annotated_engagement_demo.mp4"


@dataclass(frozen=True)
class DemoSample:
    key: str
    url: str
    page_url: str
    cache_name: str
    license_note: str

    @property
    def default_path(self) -> Path:
        return DEFAULT_OUT_DIR / self.cache_name


DEMO_SAMPLES: Dict[str, DemoSample] = {
    "cdc": DemoSample(
        key="cdc",
        url=(
            "https://commons.wikimedia.org/wiki/Special:Redirect/file/"
            "There_Isn%E2%80%99t_Just_One_Face_to_Breast_Cancer.webm"
        ),
        page_url=(
            "https://commons.wikimedia.org/wiki/"
            "File:There_Isn%E2%80%99t_Just_One_Face_to_Breast_Cancer.webm"
        ),
        cache_name="there_isnt_just_one_face_to_breast_cancer.webm",
        license_note=(
            "Public domain: CDC/HHS video hosted on Wikimedia Commons. "
            "Source page: https://commons.wikimedia.org/wiki/"
            "File:There_Isn%E2%80%99t_Just_One_Face_to_Breast_Cancer.webm"
        ),
    ),
    "multi_person": DemoSample(
        key="multi_person",
        url=(
            "https://commons.wikimedia.org/wiki/Special:Redirect/file/"
            "NASA_Astronauts_Discuss_Life_In_Space_With_Fox_News_"
            "%28iss073m262461622%29.webm"
        ),
        page_url=(
            "https://commons.wikimedia.org/wiki/"
            "File:NASA_Astronauts_Discuss_Life_In_Space_With_Fox_News_"
            "%28iss073m262461622%29.webm"
        ),
        cache_name="nasa_astronauts_discuss_life_in_space_multi_person.webm",
        license_note=(
            "Public domain: NASA Johnson Space Center video hosted on Wikimedia Commons. "
            "Source page: https://commons.wikimedia.org/wiki/"
            "File:NASA_Astronauts_Discuss_Life_In_Space_With_Fox_News_"
            "%28iss073m262461622%29.webm"
        ),
    ),
}


@dataclass
class FaceAnnotation:
    bbox: BBox
    detection_score: float
    identity: str
    expression: Optional[str] = None
    expression_score: Optional[float] = None
    yaw_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    roll_deg: Optional[float] = None
    gaze_vector: Optional[Tuple[float, float, float]] = None
    gaze_yaw_deg: Optional[float] = None
    gaze_pitch_deg: Optional[float] = None
    gaze_origin: Optional[Tuple[float, float]] = None
    engagement_score: Optional[float] = None
    engagement_state: Optional[str] = None


def _bbox_iou(a: BBox, b: BBox) -> float:
    a_top, a_right, a_bottom, a_left = a
    b_top, b_right, b_bottom, b_left = b
    inter_left = max(a_left, b_left)
    inter_top = max(a_top, b_top)
    inter_right = min(a_right, b_right)
    inter_bottom = min(a_bottom, b_bottom)
    if inter_right <= inter_left or inter_bottom <= inter_top:
        return 0.0
    inter = float((inter_right - inter_left) * (inter_bottom - inter_top))
    area_a = float((a_right - a_left) * (a_bottom - a_top))
    area_b = float((b_right - b_left) * (b_bottom - b_top))
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


class EmbeddingTracker:
    """Embedding-based identity assigner for offline demos.

    A face receives a ``user_*`` label only when a face embedding exists.
    Existing labels are reused only when embedding distance is within the
    configured threshold. Bboxes are retained for metadata and tie-breaking,
    not as identity evidence.
    """

    def __init__(
        self,
        known_dir: Optional[Path] = None,
        tolerance: float = 0.75,
        identity_margin: float = 0.15,
        max_missed_frames: int = 30,
        embedding_alpha: float = 0.25,
    ) -> None:
        self._tolerance = tolerance
        self._identity_margin = max(0.0, identity_margin)
        self._max_missed_frames = max_missed_frames
        self._embedding_alpha = embedding_alpha
        self._known_encodings: List[np.ndarray] = []
        self._known_ids: List[str] = []
        self._tracked_encodings: List[np.ndarray] = []
        self._tracked_ids: List[str] = []
        self._tracked_bboxes: List[Optional[BBox]] = []
        self._tracked_missed: List[int] = []
        self._identity_encodings: Dict[str, np.ndarray] = {}
        self._assigned_track_indices: set[int] = set()
        self._assigned_identities: set[str] = set()
        self._in_frame = False
        self._next_id = 1
        if known_dir is not None:
            self._load_known(known_dir)

    @property
    def tracked_count(self) -> int:
        return len(self._tracked_ids)

    @property
    def identity_count(self) -> int:
        return len(set(self._tracked_ids))

    @property
    def known_count(self) -> int:
        return len(self._known_ids)

    def start_frame(self, _frame_index: int) -> None:
        self._in_frame = True
        self._assigned_track_indices.clear()
        self._assigned_identities.clear()
        for idx in range(len(self._tracked_missed)):
            self._tracked_missed[idx] += 1

    def identify(self, encoding: Optional[np.ndarray], bbox: Optional[BBox] = None) -> str:
        if encoding is None:
            return "unidentified"
        if self._in_frame and self._assigned_identity_match(encoding) is not None:
            return "unidentified"

        match_idx = self._best_track_match(encoding, bbox)
        if match_idx is not None:
            self._update_track(match_idx, encoding, bbox)
            identity = self._tracked_ids[match_idx]
            self._mark_assigned(match_idx, identity)
            return identity

        identity = self._known_identity(encoding) or self._tracked_identity(encoding)
        if identity is None:
            identity = f"user_{self._next_id}"
            self._next_id += 1
        self._tracked_ids.append(identity)
        self._tracked_encodings.append(encoding.copy() if encoding is not None else np.array([], dtype=float))
        self._tracked_bboxes.append(bbox)
        self._tracked_missed.append(0)
        self._update_identity_encoding(identity, encoding)
        self._mark_assigned(len(self._tracked_ids) - 1, identity)
        return identity

    def _best_track_match(self, encoding: Optional[np.ndarray], bbox: Optional[BBox]) -> Optional[int]:
        if encoding is None:
            return None
        candidates: List[Tuple[float, float, int, int]] = []
        for idx, _tracked_id in enumerate(self._tracked_ids):
            if self._in_frame and idx in self._assigned_track_indices:
                continue
            if self._tracked_missed[idx] > self._max_missed_frames:
                continue

            iou = 0.0
            if bbox is not None and self._tracked_bboxes[idx] is not None:
                iou = _bbox_iou(bbox, self._tracked_bboxes[idx])
            distance = self._embedding_distance(self._tracked_encodings[idx], encoding)

            if distance is not None and distance <= self._tolerance:
                candidates.append((distance, 1.0 - iou, self._tracked_missed[idx], idx))

        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1], item[2]))
        best_distance = candidates[0][0]
        close_candidates = [
            item for item in candidates
            if item[0] <= best_distance + self._identity_margin
        ]

        # If embedding distances are effectively tied, prefer the track that
        # was just displayed at the same image location.
        if bbox is not None:
            continuous = [item for item in close_candidates if item[1] < 1.0]
            if continuous:
                continuous.sort(key=lambda item: (item[1], item[2], item[0]))
                return continuous[0][3]

        close_candidates.sort(key=lambda item: (item[0], item[1], item[2]))
        return close_candidates[0][3]

    def _update_track(self, idx: int, encoding: Optional[np.ndarray], bbox: Optional[BBox]) -> None:
        if bbox is not None:
            self._tracked_bboxes[idx] = bbox
        if encoding is not None:
            previous = self._tracked_encodings[idx]
            if previous.size == encoding.size:
                alpha = self._embedding_alpha
                self._tracked_encodings[idx] = (1.0 - alpha) * previous + alpha * encoding
            else:
                self._tracked_encodings[idx] = encoding.copy()
            self._update_identity_encoding(self._tracked_ids[idx], encoding)
        self._tracked_missed[idx] = 0

    def _known_identity(self, encoding: Optional[np.ndarray]) -> Optional[str]:
        if encoding is None:
            return None
        for known_id, known_encoding in zip(self._known_ids, self._known_encodings):
            if self._in_frame and known_id in self._assigned_identities:
                continue
            if self._match(known_encoding, encoding):
                return known_id
        return None

    def _tracked_identity(self, encoding: Optional[np.ndarray]) -> Optional[str]:
        if encoding is None:
            return None
        best: Optional[Tuple[float, str]] = None
        for identity, identity_encoding in self._identity_encodings.items():
            if self._in_frame and identity in self._assigned_identities:
                continue
            distance = self._embedding_distance(identity_encoding, encoding)
            if distance is None or distance > self._tolerance:
                continue
            if best is None or distance < best[0]:
                best = (distance, identity)
        return best[1] if best is not None else None

    def _assigned_identity_match(self, encoding: np.ndarray) -> Optional[str]:
        for identity in self._assigned_identities:
            identity_encoding = self._identity_encodings.get(identity)
            if identity_encoding is not None and self._match(identity_encoding, encoding):
                return identity
        return None

    def _mark_assigned(self, track_idx: int, identity: str) -> None:
        if not self._in_frame:
            return
        self._assigned_track_indices.add(track_idx)
        self._assigned_identities.add(identity)

    def _update_identity_encoding(self, identity: str, encoding: Optional[np.ndarray]) -> None:
        if encoding is None:
            return
        previous = self._identity_encodings.get(identity)
        if previous is not None and previous.size == encoding.size:
            alpha = self._embedding_alpha
            self._identity_encodings[identity] = (1.0 - alpha) * previous + alpha * encoding
        else:
            self._identity_encodings[identity] = encoding.copy()

    def _embedding_distance(
        self, reference: Optional[np.ndarray], query: Optional[np.ndarray]
    ) -> Optional[float]:
        if reference is None or query is None or reference.size == 0 or query.size == 0:
            return None
        if reference.size != query.size:
            return None
        return float(np.linalg.norm(reference - query))

    def _match(self, reference: np.ndarray, query: np.ndarray) -> bool:
        distance = self._embedding_distance(reference, query)
        return distance is not None and distance <= self._tolerance

    def _load_known(self, known_dir: Path) -> None:
        if not known_dir.exists():
            return
        try:
            import face_recognition
        except ImportError:
            print("warn: face_recognition is unavailable; known faces were not loaded", file=sys.stderr)
            return

        image_suffixes = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
        for path in sorted(known_dir.iterdir()):
            if path.suffix.lower() not in image_suffixes:
                continue
            try:
                image = face_recognition.load_image_file(str(path))
                encodings = face_recognition.face_encodings(image)
            except Exception as exc:
                print(f"warn: failed to load known face {path}: {exc}", file=sys.stderr)
                continue
            if not encodings:
                print(f"warn: no face found in known face image {path}", file=sys.stderr)
                continue
            self._known_encodings.append(encodings[0])
            self._known_ids.append(path.stem)


def _download_file(url: str, destination: Path) -> Path:
    if destination.exists() and destination.stat().st_size > 0:
        print(f"using existing sample video: {destination}")
        return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = destination.with_suffix(destination.suffix + ".tmp")
    req = urllib.request.Request(url, headers={"User-Agent": "susumu-video-demo/1.0"})
    print(f"downloading sample video: {url}")
    with urllib.request.urlopen(req, timeout=60) as response, tmp_path.open("wb") as fh:
        total = int(response.headers.get("Content-Length") or 0)
        copied = 0
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            fh.write(chunk)
            copied += len(chunk)
            if total:
                pct = copied * 100.0 / total
                print(f"  {copied / 1024 / 1024:.1f}/{total / 1024 / 1024:.1f} MiB ({pct:.0f}%)", end="\r")
    if total:
        print()
    tmp_path.replace(destination)
    print(f"wrote sample video: {destination}")
    return destination


def _resolve_input(input_arg: Optional[str], sample_name: str, sample_path: Optional[Path]) -> Tuple[Path, str]:
    sample = DEMO_SAMPLES[sample_name]
    cache_path = sample_path if sample_path is not None else sample.default_path

    if not input_arg:
        return _download_file(sample.url, cache_path), sample.license_note

    if input_arg.startswith(("http://", "https://")):
        suffix = Path(input_arg.split("?", 1)[0]).suffix or ".video"
        downloaded = cache_path.with_name(f"downloaded_input{suffix}")
        return _download_file(input_arg, downloaded), f"Downloaded from: {input_arg}"

    return Path(input_arg), "Local input video"


def _clamp_box(box: BBox, width: int, height: int) -> Optional[BBox]:
    top, right, bottom, left = box
    top = max(0, min(height - 1, int(round(top))))
    bottom = max(0, min(height, int(round(bottom))))
    left = max(0, min(width - 1, int(round(left))))
    right = max(0, min(width, int(round(right))))
    if right <= left or bottom <= top:
        return None
    return top, right, bottom, left


def _pad_box(box: BBox, width: int, height: int, ratio: float = 0.2) -> Optional[BBox]:
    top, right, bottom, left = box
    bw = right - left
    bh = bottom - top
    pad_x = int(round(bw * ratio))
    pad_y = int(round(bh * ratio))
    return _clamp_box((top - pad_y, right + pad_x, bottom + pad_y, left - pad_x), width, height)


def _crop(frame: np.ndarray, box: BBox) -> np.ndarray:
    top, right, bottom, left = box
    return frame[top:bottom, left:right]


def _resize_frame(frame: np.ndarray, resize_width: int) -> np.ndarray:
    if resize_width <= 0 or frame.shape[1] <= resize_width:
        return frame
    scale = resize_width / float(frame.shape[1])
    height = max(1, int(round(frame.shape[0] * scale)))
    return cv2.resize(frame, (resize_width, height), interpolation=cv2.INTER_AREA)


def _is_opencv_readable(path: Path) -> bool:
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            return False
        ok, frame = capture.read()
        return bool(ok and frame is not None and frame.size)
    finally:
        capture.release()


def _transcode_for_opencv(
    input_path: Path,
    start_seconds: float,
    max_seconds: float,
    resize_width: int,
) -> Path:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError(
            f"OpenCV cannot read {input_path} and ffmpeg is not available for transcoding"
        )

    start_key = int(round(max(0.0, start_seconds) * 1000.0))
    duration_key = int(round(max(0.0, max_seconds) * 1000.0))
    width_key = int(resize_width) if resize_width and resize_width > 0 else 0
    output_path = input_path.with_name(
        f"{input_path.stem}_opencv_s{start_key}_d{duration_key}_w{width_key}.mp4"
    )
    if output_path.exists() and output_path.stat().st_size > 0 and _is_opencv_readable(output_path):
        print(f"using existing OpenCV-readable video cache: {output_path}")
        return output_path

    cmd = [ffmpeg, "-y", "-v", "error"]
    if start_seconds and start_seconds > 0:
        cmd += ["-ss", f"{start_seconds:.3f}"]
    cmd += ["-i", str(input_path)]
    if max_seconds and max_seconds > 0:
        cmd += ["-t", f"{max_seconds:.3f}"]
    if resize_width and resize_width > 0:
        cmd += ["-vf", f"scale={int(resize_width)}:-2"]
    cmd += [
        "-an",
        "-c:v", "libx264",
        "-preset", "veryfast",
        "-crf", "23",
        "-pix_fmt", "yuv420p",
        str(output_path),
    ]
    print(f"transcoding for OpenCV: {output_path}")
    subprocess.run(cmd, check=True)
    return output_path


def _identity_color(identity: str) -> Tuple[int, int, int]:
    palette = [
        (36, 183, 255),
        (90, 210, 120),
        (255, 143, 88),
        (216, 128, 255),
        (255, 214, 86),
        (96, 196, 218),
        (114, 161, 255),
    ]
    idx = sum(ord(c) for c in identity) % len(palette)
    return palette[idx]


def _put_text_box(
    frame: np.ndarray,
    text: str,
    origin: Tuple[int, int],
    color: Tuple[int, int, int],
    scale: float = 0.55,
    thickness: int = 1,
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    x, y = origin
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = max(0, min(frame.shape[1] - tw - 8, x))
    y = max(th + 8, min(frame.shape[0] - baseline - 4, y))
    cv2.rectangle(frame, (x, y - th - 6), (x + tw + 8, y + baseline + 5), (16, 20, 24), -1)
    cv2.rectangle(frame, (x, y - th - 6), (x + tw + 8, y + baseline + 5), color, 1)
    cv2.putText(frame, text, (x + 4, y), font, scale, (245, 248, 250), thickness, cv2.LINE_AA)


def _format_optional(value: Optional[float], suffix: str = "", digits: int = 1) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}{suffix}"


def _head_pose_label(yaw: Optional[float], pitch: Optional[float]) -> str:
    if yaw is None or pitch is None:
        return "head n/a"
    visual_yaw = -yaw
    if abs(visual_yaw) <= 15.0 and abs(pitch) <= 10.0:
        return "head centered"
    parts = []
    if visual_yaw < -15.0:
        parts.append("left")
    elif visual_yaw > 15.0:
        parts.append("right")
    if pitch < -10.0:
        parts.append("down")
    elif pitch > 10.0:
        parts.append("up")
    return "head " + "/".join(parts or ["away"])


def _draw_direction_gauge(
    frame: np.ndarray,
    origin: Tuple[int, int],
    label: str,
    x_value: Optional[float],
    y_value: Optional[float],
    color: Tuple[int, int, int],
) -> None:
    x, y = origin
    size = 54
    x = max(0, min(frame.shape[1] - size - 1, x))
    y = max(0, min(frame.shape[0] - size - 16, y))
    centre = (x + size // 2, y + size // 2)
    cv2.rectangle(frame, (x, y), (x + size, y + size), (16, 20, 24), -1)
    cv2.rectangle(frame, (x, y), (x + size, y + size), color, 1)
    cv2.line(frame, (x + 6, centre[1]), (x + size - 6, centre[1]), (105, 115, 125), 1)
    cv2.line(frame, (centre[0], y + 6), (centre[0], y + size - 6), (105, 115, 125), 1)
    cv2.circle(frame, centre, 2, (180, 185, 190), -1)

    if x_value is not None and y_value is not None:
        px = int(round(centre[0] + np.clip(x_value, -1.0, 1.0) * (size // 2 - 8)))
        py = int(round(centre[1] + np.clip(y_value, -1.0, 1.0) * (size // 2 - 8)))
        cv2.circle(frame, (px, py), 5, color, -1)
        cv2.line(frame, centre, (px, py), color, 2)

    cv2.putText(
        frame,
        label,
        (x, y + size + 13),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (245, 248, 250),
        1,
        cv2.LINE_AA,
    )


def _draw_head_pose_arrow(
    frame: np.ndarray,
    box: BBox,
    yaw: Optional[float],
    pitch: Optional[float],
    color: Tuple[int, int, int],
) -> None:
    if yaw is None or pitch is None:
        return
    visual_yaw = -yaw
    top, right, bottom, left = box
    cx = int(round((left + right) / 2.0))
    cy = int(round((top + bottom) / 2.0))
    length = max(34, int(round((right - left) * 0.42)))
    dx = np.clip(visual_yaw / 60.0, -1.0, 1.0) * length
    dy = np.clip(-pitch / 45.0, -1.0, 1.0) * length
    end = (int(round(cx + dx)), int(round(cy + dy)))
    cv2.arrowedLine(frame, (cx, cy), end, color, 2, tipLength=0.25)
    cv2.putText(
        frame,
        "head",
        (end[0] + 4, end[1] - 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )


def _draw_gaze_arrow(
    frame: np.ndarray,
    box: BBox,
    origin: Optional[Tuple[float, float]],
    yaw: Optional[float],
    pitch: Optional[float],
    color: Tuple[int, int, int],
) -> None:
    if origin is None or yaw is None or pitch is None:
        return
    top, right, bottom, left = box
    ox = int(round(left + origin[0]))
    oy = int(round(top + origin[1]))
    face_width = max(1, right - left)
    length = max(34, int(round(face_width * 0.48)))
    dx = np.clip(math.tan(math.radians(yaw)), -1.0, 1.0) * length
    dy = -np.clip(math.tan(math.radians(pitch)), -1.0, 1.0) * length
    end = (int(round(ox + dx)), int(round(oy + dy)))
    cv2.arrowedLine(frame, (ox, oy), end, color, 2, tipLength=0.25)
    cv2.circle(frame, (ox, oy), 3, color, -1)
    cv2.putText(
        frame,
        "gaze",
        (end[0] + 4, end[1] - 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        color,
        1,
        cv2.LINE_AA,
    )


def _draw_annotation(frame: np.ndarray, ann: FaceAnnotation) -> None:
    top, right, bottom, left = ann.bbox
    face_width = right - left
    color = _identity_color(ann.identity)
    visual_yaw = None if ann.yaw_deg is None else -ann.yaw_deg
    cv2.rectangle(frame, (left, top), (right, bottom), color, 2)

    compact = face_width < 90
    if compact:
        header = f"{ann.identity} | det {ann.detection_score:.2f}"
    else:
        header = (
            f"{ann.identity} | det {ann.detection_score:.2f} | "
            f"{ann.engagement_state or 'UNKNOWN'} "
            f"{_format_optional(ann.engagement_score, digits=2)}"
        )
    if ann.identity.startswith("face_"):
        header = f"face | det {ann.detection_score:.2f}"
    _put_text_box(frame, header, (left, max(22, top - 8)), color, scale=0.5)
    if ann.identity.startswith("face_"):
        return
    if compact:
        _draw_head_pose_arrow(frame, ann.bbox, ann.yaw_deg, ann.pitch_deg, (80, 220, 120))
        _draw_gaze_arrow(frame, ann.bbox, ann.gaze_origin, ann.gaze_yaw_deg, ann.gaze_pitch_deg, (255, 216, 96))
        return

    lines = [
        f"expr: {ann.expression or 'n/a'}"
        + (f" {ann.expression_score:.2f}" if ann.expression_score is not None else ""),
        f"{_head_pose_label(ann.yaw_deg, ann.pitch_deg)} | "
        f"yaw {_format_optional(visual_yaw, 'deg')} "
        f"pitch {_format_optional(ann.pitch_deg, 'deg')}",
    ]
    if ann.gaze_vector is not None:
        lines.append(
            f"gaze yaw {_format_optional(ann.gaze_yaw_deg, 'deg')} "
            f"pitch {_format_optional(ann.gaze_pitch_deg, 'deg')}"
        )
    y = bottom + 22
    for line in lines:
        if y > frame.shape[0] - 8:
            break
        _put_text_box(frame, line, (left, y), color, scale=0.45)
        y += 22

    _draw_head_pose_arrow(frame, ann.bbox, ann.yaw_deg, ann.pitch_deg, (80, 220, 120))
    _draw_gaze_arrow(frame, ann.bbox, ann.gaze_origin, ann.gaze_yaw_deg, ann.gaze_pitch_deg, (255, 216, 96))
    if face_width >= 120:
        gauge_y = max(0, top - 74) if top > 88 else min(frame.shape[0] - 70, bottom + 78)
        gauge_x = min(frame.shape[1] - 56, max(0, right + 8))
        _draw_direction_gauge(
            frame,
            (gauge_x, gauge_y),
            "head",
            None if visual_yaw is None else visual_yaw / 60.0,
            None if ann.pitch_deg is None else -ann.pitch_deg / 45.0,
            (80, 220, 120),
        )


def _draw_frame_panel(
    frame: np.ndarray,
    frame_index: int,
    fps: float,
    faces: int,
    args: argparse.Namespace,
    elapsed_s: float,
) -> None:
    headpose_label = "off" if args.no_headpose else f"{args.headpose_backend}+stable"
    gaze_label = "off" if args.no_gaze else args.gaze_backend
    lines = [
        f"frame {frame_index} | {fps:.1f} fps source | faces {faces}",
        f"det={args.detection_backend} rec={args.recognition_backend} expr={args.expression_backend}",
        f"head={headpose_label} gaze={gaze_label} | elapsed {elapsed_s:.1f}s",
        "green=head pose cyan=gaze",
    ]
    x, y = 12, 24
    for line in lines:
        _put_text_box(frame, line, (x, y), (80, 185, 255), scale=0.48)
        y += 24


def _open_writer(output_path: Path, fps: float, size: Tuple[int, int]) -> cv2.VideoWriter:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    for codec in ("mp4v", "avc1", "MJPG"):
        writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*codec), fps, size)
        if writer.isOpened():
            return writer
        writer.release()
    raise RuntimeError(f"failed to open video writer for {output_path}")


def _build_annotations(
    frame: np.ndarray,
    detection_result,
    recognition_backend,
    expression_backend,
    headpose_backend,
    gaze_backend,
    headpose_stabilizers: DefaultDict[str, HeadPoseStabilizer],
    tracker: EmbeddingTracker,
    scorers: DefaultDict[str, EngagementScorer],
    min_identity_face_px: int,
    identity_score_threshold: float,
    identity_max_head_yaw_deg: float,
    identity_max_head_pitch_deg: float,
) -> List[FaceAnnotation]:
    height, width = frame.shape[:2]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    annotations: List[FaceAnnotation] = []

    for i, raw_box in enumerate(detection_result.boxes):
        box = _clamp_box(raw_box[:4], width, height)
        if box is None:
            continue
        top, right, bottom, left = box
        detection_score = float(raw_box[4])
        face_width = right - left
        face_height = bottom - top
        size_trackable = min(face_width, face_height) >= min_identity_face_px

        if not size_trackable:
            annotations.append(
                FaceAnnotation(
                    bbox=box,
                    detection_score=detection_score,
                    identity=f"face_{i}",
                )
            )
            continue

        crop_box = _pad_box(box, width, height)
        face_crop = _crop(frame, crop_box) if crop_box is not None else _crop(frame, box)

        expression = None
        expression_score = None
        if expression_backend is not None and face_crop.size:
            try:
                expression_result = expression_backend.predict(face_crop)
                if expression_result is not None:
                    expression, probs = expression_result
                    expression_score = float(probs.get(expression, max(probs.values()) if probs else 0.0))
            except Exception as exc:
                print(f"warn: expression failed for face {i}: {exc}", file=sys.stderr)

        ypr = None
        if headpose_backend is not None and face_crop.size:
            try:
                ypr = headpose_backend.estimate(face_crop)
            except Exception as exc:
                print(f"warn: head pose failed for face {i}: {exc}", file=sys.stderr)

        raw_yaw = normalize_pose_angle(ypr[0]) if ypr is not None else None
        raw_pitch = normalize_pose_angle(ypr[1]) if ypr is not None else None
        raw_roll = normalize_pose_angle(ypr[2]) if ypr is not None else None

        pose_allows_identity = True
        if headpose_backend is not None:
            pose_allows_identity = (
                raw_yaw is not None
                and raw_pitch is not None
                and abs(raw_yaw) <= identity_max_head_yaw_deg
                and abs(raw_pitch) <= identity_max_head_pitch_deg
            )
        identity_allowed = (
            detection_score >= identity_score_threshold
            and pose_allows_identity
        )

        embedding = None
        if identity_allowed:
            if detection_result.encodings is not None and i < len(detection_result.encodings):
                embedding = detection_result.encodings[i]
            elif recognition_backend is not None:
                try:
                    embedding = recognition_backend.embed(rgb, box)
                except Exception as exc:
                    print(f"warn: recognition failed for face {i}: {exc}", file=sys.stderr)

        identity = tracker.identify(embedding, box) if identity_allowed else "unidentified"

        stable_ypr = ypr
        if ypr is not None and identity != "unidentified":
            stable_ypr = headpose_stabilizers[identity].update(ypr)

        yaw = normalize_pose_angle(stable_ypr[0]) if stable_ypr is not None else raw_yaw
        pitch = normalize_pose_angle(stable_ypr[1]) if stable_ypr is not None else raw_pitch
        roll = normalize_pose_angle(stable_ypr[2]) if stable_ypr is not None else raw_roll

        gaze_result = None
        gaze_crop = _crop(frame, box)
        if (
            gaze_backend is not None
            and gaze_crop.size
            and yaw is not None
            and pitch is not None
            and roll is not None
        ):
            try:
                gaze_result = gaze_backend.estimate(gaze_crop, (yaw, pitch, roll))
            except Exception as exc:
                print(f"warn: gaze failed for face {i}: {exc}", file=sys.stderr)

        state = None
        if identity != "unidentified":
            state = scorers[identity].update(
                EngagementInputs(
                    emotion_label=expression,
                    yaw_deg=yaw,
                    pitch_deg=pitch,
                )
            )

        annotations.append(
            FaceAnnotation(
                bbox=box,
                detection_score=detection_score,
                identity=identity,
                expression=expression,
                expression_score=expression_score,
                yaw_deg=yaw,
                pitch_deg=pitch,
                roll_deg=roll,
                gaze_vector=gaze_result.vector if gaze_result is not None else None,
                gaze_yaw_deg=gaze_result.yaw_deg if gaze_result is not None else None,
                gaze_pitch_deg=gaze_result.pitch_deg if gaze_result is not None else None,
                gaze_origin=gaze_result.eye_center if gaze_result is not None else None,
                engagement_score=state.score if state is not None else None,
                engagement_state=state_name(state.level) if state is not None else "UNKNOWN",
            )
        )

    return annotations


def run(args: argparse.Namespace) -> Path:
    sample_path = Path(args.sample_path) if args.sample_path else None
    input_path, source_note = _resolve_input(args.input, args.sample, sample_path)
    if not input_path.exists():
        raise FileNotFoundError(input_path)

    original_input_path = input_path
    if not _is_opencv_readable(input_path):
        input_path = _transcode_for_opencv(
            input_path,
            start_seconds=args.start_seconds,
            max_seconds=args.max_seconds,
            resize_width=args.resize_width,
        )
        effective_start_seconds = 0.0
    else:
        effective_start_seconds = args.start_seconds

    output_path = Path(args.output)
    capture = cv2.VideoCapture(str(input_path))
    if not capture.isOpened():
        raise RuntimeError(f"failed to open input video: {input_path}")

    fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    if fps <= 0 or not math.isfinite(fps):
        fps = args.fallback_fps
    source_frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    if effective_start_seconds and effective_start_seconds > 0:
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(round(effective_start_seconds * fps)))
    max_frames = args.max_frames if args.max_frames and args.max_frames > 0 else None
    if args.max_seconds and args.max_seconds > 0:
        seconds_frames = max(1, int(math.ceil(args.max_seconds * fps)))
        max_frames = seconds_frames if max_frames is None else min(max_frames, seconds_frames)
    progress_every = max(1, int(args.progress_every))
    process_every = max(1, int(args.process_every))

    print(f"input:  {original_input_path}")
    if input_path != original_input_path:
        print(f"opencv input cache: {input_path}")
    print(f"output: {output_path}")
    print(f"source: {source_note}")

    detection_backend = make_detection_backend(
        args.detection_backend,
        model_path=args.model_path,
        score_threshold=args.score_threshold,
        nms_threshold=args.nms_threshold,
    )
    recognition_backend = None if args.no_recognition else make_recognition_backend(args.recognition_backend)
    expression_backend = None if args.no_expression else make_expression_backend(args.expression_backend)
    headpose_backend = None if args.no_headpose else make_headpose_backend(args.headpose_backend)
    gaze_backend = None
    gaze_enabled = not args.no_gaze
    if args.no_headpose and gaze_enabled:
        print("warn: gaze disabled because head pose is disabled", file=sys.stderr)
        gaze_enabled = False
        args.no_gaze = True
    if gaze_enabled:
        try:
            gaze_backend = make_gaze_backend(
                args.gaze_backend,
                model_path=args.gaze_model_path,
                device=args.gaze_device,
                min_eye_size=args.gaze_min_eye_px,
                max_head_yaw_deg=args.gaze_max_head_yaw_deg,
                max_head_pitch_deg=args.gaze_max_head_pitch_deg,
            )
        except Exception as exc:
            print(f"warn: gaze disabled: {exc}", file=sys.stderr)
            gaze_enabled = False
            args.no_gaze = True
    tracker = EmbeddingTracker(
        Path(args.known_faces_dir) if args.known_faces_dir else None,
        tolerance=args.match_tolerance,
        identity_margin=args.identity_margin,
        max_missed_frames=args.max_missed_frames,
        embedding_alpha=args.embedding_alpha,
    )
    scorers: DefaultDict[str, EngagementScorer] = defaultdict(EngagementScorer)
    headpose_stabilizers: DefaultDict[str, HeadPoseStabilizer] = defaultdict(
        lambda: HeadPoseStabilizer(
            ema_alpha=args.headpose_smoothing_alpha,
            max_jump_deg=args.headpose_max_jump_deg,
            reset_after_missed=args.headpose_reset_after_missed,
        )
    )

    writer = None
    frame_index = 0
    processed_frames = 0
    last_annotations: List[FaceAnnotation] = []
    started = time.monotonic()

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if max_frames is not None and processed_frames >= max_frames:
                break

            frame = _resize_frame(frame, args.resize_width)
            if writer is None:
                writer = _open_writer(output_path, fps, (frame.shape[1], frame.shape[0]))

            should_run_inference = process_every <= 1 or frame_index % process_every == 0
            if should_run_inference:
                tracker.start_frame(frame_index)
                detection_result = detection_backend.detect(frame)
                last_annotations = _build_annotations(
                    frame,
                    detection_result,
                    recognition_backend,
                    expression_backend,
                    headpose_backend,
                    gaze_backend,
                    headpose_stabilizers,
                    tracker,
                    scorers,
                    args.min_identity_face_px,
                    args.identity_score_threshold,
                    args.identity_max_head_yaw_deg,
                    args.identity_max_head_pitch_deg,
                )

            annotated = frame.copy()
            for ann in last_annotations:
                _draw_annotation(annotated, ann)
            elapsed = time.monotonic() - started
            _draw_frame_panel(annotated, frame_index, fps, len(last_annotations), args, elapsed)
            writer.write(annotated)

            processed_frames += 1
            frame_index += 1
            if processed_frames == 1 or processed_frames % progress_every == 0:
                total = max_frames or source_frame_count
                if total:
                    print(f"processed {processed_frames}/{total} frames")
                else:
                    print(f"processed {processed_frames} frames")
    finally:
        capture.release()
        if writer is not None:
            writer.release()

    if processed_frames == 0:
        raise RuntimeError("no frames were written")

    metadata: Dict[str, object] = {
        "input": str(original_input_path),
        "opencv_input": str(input_path) if input_path != original_input_path else None,
        "output": str(output_path),
        "sample": None if args.input else args.sample,
        "source": source_note,
        "frames_written": processed_frames,
        "fps": fps,
        "start_seconds": args.start_seconds,
        "detection_backend": args.detection_backend,
        "score_threshold": args.score_threshold,
        "recognition_backend": None if args.no_recognition else args.recognition_backend,
        "expression_backend": None if args.no_expression else args.expression_backend,
        "headpose_backend": None if args.no_headpose else args.headpose_backend,
        "headpose_stabilized": not args.no_headpose,
        "headpose_smoothing_alpha": args.headpose_smoothing_alpha,
        "headpose_max_jump_deg": args.headpose_max_jump_deg,
        "headpose_reset_after_missed": args.headpose_reset_after_missed,
        "gaze_backend": args.gaze_backend if gaze_enabled else None,
        "gaze_model_path": args.gaze_model_path if gaze_enabled else None,
        "gaze_device": args.gaze_device if gaze_enabled else None,
        "known_faces_loaded": tracker.known_count,
        "track_instances_created": tracker.tracked_count,
        "unique_identities_created": tracker.identity_count,
        "match_tolerance": args.match_tolerance,
        "identity_margin": args.identity_margin,
        "max_missed_frames": args.max_missed_frames,
        "min_identity_face_px": args.min_identity_face_px,
        "identity_score_threshold": args.identity_score_threshold,
        "identity_max_head_yaw_deg": args.identity_max_head_yaw_deg,
        "identity_max_head_pitch_deg": args.identity_max_head_pitch_deg,
    }
    sidecar = output_path.with_suffix(output_path.suffix + ".json")
    sidecar.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    print(f"wrote annotated video: {output_path}")
    print(f"wrote metadata:        {sidecar}")
    return output_path


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create an annotated face engagement video from a local video or a public-domain sample."
    )
    parser.add_argument("--input", help="Input video path or URL. If omitted, downloads the default public-domain sample.")
    parser.add_argument("--sample", default="cdc", choices=sorted(DEMO_SAMPLES), help="Built-in sample to use when --input is omitted.")
    parser.add_argument("--output", default=str(DEFAULT_OUT_PATH), help="Output MP4 path.")
    parser.add_argument("--sample-path", default="", help="Override where to cache the selected built-in sample video.")
    parser.add_argument("--max-seconds", type=float, default=20.0, help="Limit output duration in seconds. Use 0 for full video.")
    parser.add_argument("--start-seconds", type=float, default=0.0, help="Start processing at this timestamp in seconds.")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit output frame count. 0 means no explicit frame limit.")
    parser.add_argument("--resize-width", type=int, default=960, help="Downscale wider videos to this width. 0 disables resizing.")
    parser.add_argument("--fallback-fps", type=float, default=25.0, help="FPS to use if the input has no FPS metadata.")
    parser.add_argument("--progress-every", type=int, default=30, help="Print progress every N frames.")
    parser.add_argument("--process-every", type=int, default=1, help="Run inference every N frames and reuse overlays between runs.")

    parser.add_argument("--detection-backend", default="yunet", choices=["yunet", "dlib_hog", "dlib_cnn"])
    parser.add_argument("--model-path", default=str(Path.home() / "models/face_detection/face_detection_yunet_2023mar.onnx"))
    parser.add_argument("--score-threshold", type=float, default=0.8)
    parser.add_argument("--nms-threshold", type=float, default=0.3)

    parser.add_argument("--recognition-backend", default="dlib_128d", choices=["dlib_128d"])
    parser.add_argument("--known-faces-dir", default="", help="Optional directory of known face images, named by file stem.")
    parser.add_argument("--match-tolerance", type=float, default=0.75)
    parser.add_argument(
        "--identity-margin",
        type=float,
        default=0.15,
        help="When close identity candidates differ by at most this distance, prefer the previous track ID.",
    )
    parser.add_argument("--max-missed-frames", type=int, default=30)
    parser.add_argument("--embedding-alpha", type=float, default=0.25)
    parser.add_argument("--min-identity-face-px", type=int, default=110)
    parser.add_argument(
        "--identity-score-threshold",
        type=float,
        default=0.9,
        help="Minimum detector confidence required before assigning a user ID.",
    )
    parser.add_argument("--identity-max-head-yaw-deg", type=float, default=40.0)
    parser.add_argument("--identity-max-head-pitch-deg", type=float, default=25.0)
    parser.add_argument("--no-recognition", action="store_true")

    parser.add_argument("--expression-backend", default="hsemotion", choices=["hsemotion"])
    parser.add_argument("--headpose-backend", default="mediapipe_pnp", choices=["mediapipe_pnp"])
    parser.add_argument("--headpose-smoothing-alpha", type=float, default=0.35)
    parser.add_argument("--headpose-max-jump-deg", type=float, default=35.0)
    parser.add_argument("--headpose-reset-after-missed", type=int, default=10)
    parser.add_argument("--gaze-backend", default="openvino_adas", choices=["openvino_adas"])
    parser.add_argument(
        "--gaze-model-path",
        default=str(
            Path.home()
            / "models/gaze_estimation/intel/gaze-estimation-adas-0002/FP32/gaze-estimation-adas-0002.xml"
        ),
        help="OpenVINO gaze-estimation-adas-0002 XML path.",
    )
    parser.add_argument("--gaze-device", default="CPU")
    parser.add_argument("--gaze-min-eye-px", type=int, default=8)
    parser.add_argument("--gaze-max-head-yaw-deg", type=float, default=45.0)
    parser.add_argument("--gaze-max-head-pitch-deg", type=float, default=35.0)
    parser.add_argument("--no-expression", action="store_true")
    parser.add_argument("--no-headpose", action="store_true")
    parser.add_argument("--no-gaze", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        run(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
