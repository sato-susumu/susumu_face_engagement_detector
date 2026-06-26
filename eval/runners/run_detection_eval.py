"""Detection evaluation runner.

Two evaluation modes:

  (1) DIRECT (default): instantiate a detector backend and feed images
      directly. Fast and deterministic — ideal for baselines and CI.

  (2) ROSBAG: feed images via rosbag2 playback into the running ROS 2
      pipeline and collect the published Detection2DArray. Catches
      pipeline-level regressions (QoS, serialization, msg contract).
      Implemented as a follow-up in this same file via `--mode rosbag`.

The result JSON shape is stable so reports and CI diffs can compare versions
across time. Schema:

    {
      "backend": "<name>",
      "version": "<freeform>",
      "config": {...},
      "dataset": "wider_face_val",
      "iou_threshold": 0.5,
      "overall": {"ap": ..., "num_gt": ..., "num_pred": ...},
      "subsets": {"easy": {...}, "medium": {...}, "hard": {...}},
      "latency_ms": {"mean": ..., "p50": ..., "p95": ..., ...},
      "num_images": N,
      "elapsed_seconds": ...
    }
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Iterable, List

import cv2
import numpy as np

from eval.datasets.wider_face import (
    DEFAULT_ROOT,
    WiderFaceImage,
    load_subsets,
    load_val,
)
from eval.metrics.detection import (
    Box,
    FrameDetections,
    average_precision,
    split_wider_subsets,
)


# ---------------------------------------------------------------------------
# Detector backends
# ---------------------------------------------------------------------------


class DlibHogBackend:
    """Current baseline — face_recognition (dlib HOG)."""

    name = "dlib_hog"

    def __init__(self, **_kw) -> None:
        import face_recognition  # imported lazily so other backends don't pull it
        self._fr = face_recognition

    def detect(self, bgr: np.ndarray) -> List[tuple]:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        locations = self._fr.face_locations(rgb, model="hog")
        # dlib does not provide scores → use a fixed value.
        return [
            (Box(left, top, right - left, bottom - top), 1.0)
            for top, right, bottom, left in locations
        ]


class DlibCnnBackend:
    """face_recognition (dlib CNN)."""

    name = "dlib_cnn"

    def __init__(self, **_kw) -> None:
        import face_recognition
        self._fr = face_recognition

    def detect(self, bgr: np.ndarray) -> List[tuple]:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        locations = self._fr.face_locations(rgb, model="cnn")
        return [
            (Box(left, top, right - left, bottom - top), 1.0)
            for top, right, bottom, left in locations
        ]


class YuNetBackend:
    """OpenCV YuNet (Apache 2.0 — commercial OK)."""

    name = "yunet"

    def __init__(self, model_path: str | None = None, score_threshold: float = 0.5,
                 nms_threshold: float = 0.3, **_kw) -> None:
        if model_path is None:
            raise ValueError(
                "yunet backend requires --model pointing to "
                "face_detection_yunet_*.onnx from OpenCV Zoo"
            )
        self._detector = cv2.FaceDetectorYN_create(
            model_path, "", (320, 320),
            score_threshold=score_threshold, nms_threshold=nms_threshold,
        )

    def detect(self, bgr: np.ndarray) -> List[tuple]:
        h, w = bgr.shape[:2]
        self._detector.setInputSize((w, h))
        _retval, faces = self._detector.detect(bgr)
        out: List[tuple] = []
        if faces is None:
            return out
        for f in faces:
            x, y, bw, bh = float(f[0]), float(f[1]), float(f[2]), float(f[3])
            score = float(f[-1])
            out.append((Box(x, y, bw, bh), score))
        return out


class MediaPipeBackend:
    """MediaPipe Face Detector (Apache 2.0 — commercial OK)."""

    name = "mediapipe"

    def __init__(self, model_selection: int = 1, min_detection_confidence: float = 0.5, **_kw) -> None:
        import mediapipe as mp  # type: ignore
        self._mp = mp
        self._det = mp.solutions.face_detection.FaceDetection(
            model_selection=model_selection,
            min_detection_confidence=min_detection_confidence,
        )

    def detect(self, bgr: np.ndarray) -> List[tuple]:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self._det.process(rgb)
        out: List[tuple] = []
        if not res.detections:
            return out
        for d in res.detections:
            bbox = d.location_data.relative_bounding_box
            x = max(0.0, bbox.xmin) * w
            y = max(0.0, bbox.ymin) * h
            bw = bbox.width * w
            bh = bbox.height * h
            score = float(d.score[0]) if d.score else 1.0
            out.append((Box(x, y, bw, bh), score))
        return out


BACKENDS = {
    "dlib_hog": DlibHogBackend,
    "dlib_cnn": DlibCnnBackend,
    "yunet": YuNetBackend,
    "mediapipe": MediaPipeBackend,
}


# ---------------------------------------------------------------------------
# Evaluation loop
# ---------------------------------------------------------------------------


def _iter_images(
    images: List[WiderFaceImage], limit: int
) -> Iterable[WiderFaceImage]:
    yielded = 0
    for img in images:
        if not img.image_path.exists():
            continue
        yield img
        yielded += 1
        if limit and yielded >= limit:
            break


def run_direct(
    backend: object,
    images: List[WiderFaceImage],
    iou_threshold: float,
    limit: int = 0,
    progress_every: int = 100,
) -> dict:
    frames: List[FrameDetections] = []
    latencies_ms: List[float] = []
    start_total = time.perf_counter()
    n = 0
    for entry in _iter_images(images, limit):
        bgr = cv2.imread(str(entry.image_path), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        t0 = time.perf_counter()
        preds = backend.detect(bgr)
        elapsed = (time.perf_counter() - t0) * 1000.0
        latencies_ms.append(elapsed)
        frames.append(
            FrameDetections(
                image_id=entry.image_id,
                predictions=preds,
                ground_truth=list(entry.ground_truth),
                ignore=list(entry.ignore),
            )
        )
        n += 1
        if progress_every and n % progress_every == 0:
            so_far = time.perf_counter() - start_total
            print(
                f"  [{n} images] avg det {np.mean(latencies_ms):.1f} ms, "
                f"elapsed {so_far:.1f}s",
                flush=True,
            )

    elapsed_total = time.perf_counter() - start_total
    overall = average_precision(frames, iou_threshold=iou_threshold)

    subsets_def = load_subsets()  # may be empty if user did not provide JSON
    subsets_result: dict = {}
    if subsets_def:
        split = split_wider_subsets(frames, subsets_def)
        for diff, frs in split.items():
            subsets_result[diff] = average_precision(frs, iou_threshold=iou_threshold)

    lat = np.array(latencies_ms) if latencies_ms else np.array([0.0])
    return {
        "overall": overall,
        "subsets": subsets_result,
        "latency_ms": {
            "mean": float(np.mean(lat)),
            "std": float(np.std(lat)),
            "p50": float(np.percentile(lat, 50)),
            "p95": float(np.percentile(lat, 95)),
            "p99": float(np.percentile(lat, 99)),
            "min": float(np.min(lat)),
            "max": float(np.max(lat)),
        },
        "num_images": n,
        "elapsed_seconds": elapsed_total,
    }


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--backend", required=True, choices=list(BACKENDS.keys()))
    p.add_argument("--model", default=None, help="Path to model weights (backend-specific)")
    p.add_argument("--mode", choices=["direct", "rosbag"], default="direct")
    p.add_argument("--dataset", default="wider_face_val")
    p.add_argument("--dataset-root", type=Path, default=DEFAULT_ROOT)
    p.add_argument("--iou-threshold", type=float, default=0.5)
    p.add_argument("--limit", type=int, default=0, help="0=all images")
    p.add_argument("--output", type=Path, required=True, help="Path to write result JSON")
    p.add_argument("--score-threshold", type=float, default=0.5)
    p.add_argument("--nms-threshold", type=float, default=0.3)
    p.add_argument("--label", default=None, help="Free-form label for the run (defaults to backend)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    if args.mode == "rosbag":
        raise SystemExit(
            "rosbag mode is implemented in a follow-up; use --mode direct for now"
        )

    if args.dataset != "wider_face_val":
        raise SystemExit(f"only wider_face_val supported at this Phase, got {args.dataset!r}")
    images = load_val(args.dataset_root)
    print(f"[run_detection_eval] dataset: {len(images)} images under {args.dataset_root}")

    backend_cls = BACKENDS[args.backend]
    backend_kwargs = {
        "model_path": args.model,
        "score_threshold": args.score_threshold,
        "nms_threshold": args.nms_threshold,
    }
    backend = backend_cls(**backend_kwargs)
    print(f"[run_detection_eval] backend: {backend.name}")

    result = run_direct(
        backend=backend,
        images=images,
        iou_threshold=args.iou_threshold,
        limit=args.limit,
    )

    payload = {
        "backend": backend.name,
        "label": args.label or backend.name,
        "config": {
            "model": args.model,
            "score_threshold": args.score_threshold,
            "nms_threshold": args.nms_threshold,
        },
        "dataset": args.dataset,
        "iou_threshold": args.iou_threshold,
        **result,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2))
    print(f"[run_detection_eval] wrote {args.output}")
    print(f"  overall AP@{args.iou_threshold}: {payload['overall']['ap']:.4f}")
    print(f"  mean latency: {payload['latency_ms']['mean']:.1f} ms")


if __name__ == "__main__":
    main()
