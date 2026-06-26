"""Face detection metrics: IoU, Precision/Recall curves, Average Precision.

PASCAL VOC 2010+ all-points interpolation (WIDER FACE official evaluator uses
the same scheme). One class only ("face"), but the structure leaves room for
multi-class extension.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Box:
    """Axis-aligned bounding box in image pixels.

    (x, y) is the top-left corner, (w, h) the size in pixels.
    """

    x: float
    y: float
    w: float
    h: float

    @property
    def area(self) -> float:
        return max(0.0, self.w) * max(0.0, self.h)

    @property
    def x2(self) -> float:
        return self.x + self.w

    @property
    def y2(self) -> float:
        return self.y + self.h


def iou(a: Box, b: Box) -> float:
    """Intersection over Union between two boxes."""
    ix1 = max(a.x, b.x)
    iy1 = max(a.y, b.y)
    ix2 = min(a.x2, b.x2)
    iy2 = min(a.y2, b.y2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    union = a.area + b.area - inter
    if union <= 0.0:
        return 0.0
    return inter / union


@dataclass
class FrameDetections:
    """All predictions and ground-truth boxes for a single image."""

    image_id: str
    predictions: List[Tuple[Box, float]]  # (box, score)
    ground_truth: List[Box]
    # Per-GT-box ignore flags (e.g. WIDER "invalid" tag). False by default.
    ignore: List[bool] | None = None


def _match_predictions(
    preds: Sequence[Tuple[Box, float]],
    gts: Sequence[Box],
    ignore: Sequence[bool],
    iou_threshold: float,
) -> Tuple[List[int], List[int]]:
    """Greedy match predictions (sorted by score desc) to ground truth.

    Returns (tp_flags, fp_flags) of length len(preds) — each 1 if matched as TP/FP, else 0.
    Predictions matched to ignored GT are recorded as neither TP nor FP.
    """
    matched = [False] * len(gts)
    tp = [0] * len(preds)
    fp = [0] * len(preds)
    # Predictions are assumed already sorted by score descending.
    for i, (pbox, _score) in enumerate(preds):
        best_iou = 0.0
        best_j = -1
        for j, gbox in enumerate(gts):
            if matched[j]:
                continue
            cur = iou(pbox, gbox)
            if cur > best_iou:
                best_iou = cur
                best_j = j
        if best_j >= 0 and best_iou >= iou_threshold:
            if ignore[best_j]:
                continue  # neither TP nor FP
            matched[best_j] = True
            tp[i] = 1
        else:
            fp[i] = 1
    return tp, fp


def average_precision(
    frames: Iterable[FrameDetections],
    iou_threshold: float = 0.5,
) -> dict:
    """Compute Average Precision aggregated across frames.

    Uses PASCAL VOC 2010+ all-points (every distinct recall) interpolation:
    AP = sum over distinct recall levels of (recall_{i+1} - recall_i) * max_precision_after.
    """
    all_scores: List[float] = []
    all_tp: List[int] = []
    all_fp: List[int] = []
    total_gt = 0

    for frame in frames:
        ignore = frame.ignore if frame.ignore is not None else [False] * len(frame.ground_truth)
        # sort preds by score desc once per frame
        preds_sorted = sorted(frame.predictions, key=lambda x: -x[1])
        tp, fp = _match_predictions(preds_sorted, frame.ground_truth, ignore, iou_threshold)
        for (_b, s), t, f in zip(preds_sorted, tp, fp):
            all_scores.append(s)
            all_tp.append(t)
            all_fp.append(f)
        total_gt += sum(1 for ig in ignore if not ig)

    if total_gt == 0:
        return {"ap": float("nan"), "precision": [], "recall": [], "num_gt": 0, "num_pred": 0}

    if not all_scores:
        return {"ap": 0.0, "precision": [], "recall": [], "num_gt": total_gt, "num_pred": 0}

    order = np.argsort(-np.array(all_scores), kind="mergesort")
    tp_arr = np.array(all_tp, dtype=np.int64)[order]
    fp_arr = np.array(all_fp, dtype=np.int64)[order]

    tp_cum = np.cumsum(tp_arr)
    fp_cum = np.cumsum(fp_arr)
    recall = tp_cum / total_gt
    precision = tp_cum / np.maximum(tp_cum + fp_cum, 1)

    # VOC 2010+ all-points interpolation
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([0.0], precision, [0.0]))
    for i in range(len(mpre) - 1, 0, -1):
        mpre[i - 1] = max(mpre[i - 1], mpre[i])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    ap = float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))

    return {
        "ap": ap,
        "precision": precision.tolist(),
        "recall": recall.tolist(),
        "num_gt": int(total_gt),
        "num_pred": int(len(all_scores)),
    }


def split_wider_subsets(
    frames: Iterable[FrameDetections],
    difficulty_per_image: dict,
) -> dict:
    """Split frames by WIDER FACE difficulty (easy/medium/hard) for separate AP.

    difficulty_per_image: dict[image_id, set of difficulties this image is part of]
    e.g. {"0_Parade_marchingband_1_84": {"easy", "medium", "hard"}}
    """
    subsets: dict = {"easy": [], "medium": [], "hard": []}
    for f in frames:
        diffs = difficulty_per_image.get(f.image_id, {"easy", "medium", "hard"})
        for d in diffs:
            if d in subsets:
                subsets[d].append(f)
    return subsets
