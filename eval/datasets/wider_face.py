"""WIDER FACE dataset loader.

Official site: http://shuoyang1213.me/WIDERFACE/

The validation set + annotation pack must be downloaded manually due to the
dataset's Google Drive hosting (no stable direct URL). This module gives the
canonical paths and parses the txt-style ground truth.

Layout expected under DATASET_ROOT (default: ~/datasets/wider_face/):
    WIDER_val/
        images/<event>/<image>.jpg
    wider_face_split/
        wider_face_val_bbx_gt.txt
        wider_easy_val.mat        (subset lists, optional — we accept
        wider_medium_val.mat       a JSON of {"easy"|"medium"|"hard": [image_id, ...]}
        wider_hard_val.mat         exported alongside as wider_subsets_val.json)
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List

from eval.metrics.detection import Box, FrameDetections


DEFAULT_ROOT = Path(os.path.expanduser("~/datasets/wider_face"))

DOWNLOAD_INSTRUCTIONS = """\
WIDER FACE validation set must be downloaded manually.

  1. Visit http://shuoyang1213.me/WIDERFACE/
  2. Download:
     - WIDER Face Validation Images  (WIDER_val.zip, ~1.8 GB)
     - Face annotations              (wider_face_split.zip, ~3.5 MB)
  3. Extract both under: {root}
     Resulting layout:
       {root}/WIDER_val/images/<event>/<image>.jpg
       {root}/wider_face_split/wider_face_val_bbx_gt.txt
"""


@dataclass
class WiderFaceImage:
    """One WIDER FACE entry."""

    image_id: str          # "<event>/<filename_without_ext>"
    image_path: Path
    ground_truth: List[Box]
    ignore: List[bool]


def _parse_gt_file(gt_path: Path) -> List[WiderFaceImage]:
    """Parse WIDER FACE bbx_gt.txt format.

    Each entry is:
        <relative image path>
        <num boxes>
        x1 y1 w h blur expression illumination invalid occlusion pose
        ...

    "invalid" flag → ignore=True for that box.
    """
    entries: List[WiderFaceImage] = []
    lines = gt_path.read_text().splitlines()
    i = 0
    n = len(lines)
    while i < n:
        rel = lines[i].strip()
        if not rel:
            i += 1
            continue
        i += 1
        if i >= n:
            break
        try:
            num_boxes = int(lines[i].strip())
        except ValueError:
            i += 1
            continue
        i += 1
        boxes: List[Box] = []
        ignore: List[bool] = []
        # WIDER edge case: when num_boxes == 0, file still contains
        # exactly one zero-line "0 0 0 0 0 0 0 0 0 0". Skip it without
        # registering a box.
        if num_boxes == 0:
            if i < n and lines[i].strip().startswith("0"):
                i += 1
        else:
            for _ in range(num_boxes):
                if i >= n:
                    break
                parts = lines[i].split()
                i += 1
                if len(parts) < 4:
                    continue
                x, y, w, h = (float(p) for p in parts[:4])
                invalid = int(parts[7]) if len(parts) >= 8 else 0
                # WIDER conventions: drop zero-sized boxes (treat as ignore).
                if w <= 0 or h <= 0:
                    ig = True
                else:
                    ig = bool(invalid)
                boxes.append(Box(x, y, w, h))
                ignore.append(ig)

        image_id = rel.rsplit(".", 1)[0]
        entries.append(
            WiderFaceImage(
                image_id=image_id,
                image_path=Path("WIDER_val/images") / rel,
                ground_truth=boxes,
                ignore=ignore,
            )
        )
    return entries


def load_val(root: Path | str | None = None) -> List[WiderFaceImage]:
    """Load the validation set (images + GT)."""
    root = Path(root) if root is not None else DEFAULT_ROOT
    gt_path = root / "wider_face_split" / "wider_face_val_bbx_gt.txt"
    if not gt_path.exists():
        raise FileNotFoundError(
            DOWNLOAD_INSTRUCTIONS.format(root=root)
            + f"\nMissing: {gt_path}"
        )
    images = _parse_gt_file(gt_path)
    # Resolve image paths to absolute
    for img in images:
        img.image_path = root / img.image_path
    return images


def load_subsets(root: Path | str | None = None) -> dict:
    """Load easy/medium/hard subset definition.

    Official WIDER FACE distributes .mat files; we accept a pre-flattened JSON
    placed at {root}/wider_face_split/wider_subsets_val.json:
        {"easy": ["<event>/<image_id>", ...],
         "medium": [...], "hard": [...]}

    If the JSON is missing, returns an empty mapping (the metric layer will
    fall back to treating every image as part of every subset).
    """
    root = Path(root) if root is not None else DEFAULT_ROOT
    p = root / "wider_face_split" / "wider_subsets_val.json"
    if not p.exists():
        return {}
    with p.open() as f:
        data = json.load(f)
    inverted: dict = {}
    for diff, ids in data.items():
        for image_id in ids:
            inverted.setdefault(image_id, set()).add(diff)
    return inverted


def iter_for_publishing(images: List[WiderFaceImage]) -> Iterator[WiderFaceImage]:
    """Convenience iterator that yields entries with existing image files only."""
    for img in images:
        if img.image_path.exists():
            yield img


def to_frame_detections(images: List[WiderFaceImage]) -> List[FrameDetections]:
    """Wrap WIDER entries into FrameDetections with empty predictions.

    Predictions are filled in later by the evaluation runner once it collects
    Detection2DArray messages from the bag.
    """
    return [
        FrameDetections(
            image_id=img.image_id,
            predictions=[],
            ground_truth=list(img.ground_truth),
            ignore=list(img.ignore),
        )
        for img in images
    ]
