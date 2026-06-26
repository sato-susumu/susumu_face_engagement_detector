"""Face recognition (1:1 verification) metrics.

Given a list of pairs (embedding_a, embedding_b, same_person), produce:
  * verification accuracy at the best threshold
  * AUC and TAR@FAR (e.g. TAR@FAR=1e-3) — the metric LFW papers usually quote
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class Pair:
    a: np.ndarray
    b: np.ndarray
    same: bool


def cosine_distance(a: np.ndarray, b: np.ndarray) -> float:
    denom = (np.linalg.norm(a) * np.linalg.norm(b)) or 1e-12
    return 1.0 - float(np.dot(a, b) / denom)


def _scores_and_labels(pairs: Sequence[Pair]) -> Tuple[np.ndarray, np.ndarray]:
    # "score" = similarity (1 - cosine distance). Larger ↔ more likely same person.
    scores = np.array([1.0 - cosine_distance(p.a, p.b) for p in pairs], dtype=np.float64)
    labels = np.array([1 if p.same else 0 for p in pairs], dtype=np.int64)
    return scores, labels


def best_threshold_accuracy(pairs: Sequence[Pair]) -> dict:
    """Sweep thresholds and pick the one maximising accuracy."""
    if not pairs:
        return {"accuracy": float("nan"), "threshold": float("nan"), "num_pairs": 0}
    scores, labels = _scores_and_labels(pairs)
    thresholds = np.unique(scores)
    best_acc = 0.0
    best_t = 0.0
    for t in thresholds:
        pred = (scores >= t).astype(np.int64)
        acc = float(np.mean(pred == labels))
        if acc > best_acc:
            best_acc = acc
            best_t = float(t)
    return {
        "accuracy": best_acc,
        "threshold": best_t,
        "num_pairs": int(len(pairs)),
        "num_positive": int(np.sum(labels)),
        "num_negative": int(np.sum(1 - labels)),
    }


def tar_at_far(pairs: Sequence[Pair], target_fars: Sequence[float] = (1e-1, 1e-2, 1e-3)) -> dict:
    """True Acceptance Rate at given False Acceptance Rates.

    Returns a dict mapping each FAR (as a string) to the TAR achievable while
    keeping FAR <= target.
    """
    if not pairs:
        return {f"tar@far={f:g}": float("nan") for f in target_fars}
    scores, labels = _scores_and_labels(pairs)
    pos = scores[labels == 1]
    neg = scores[labels == 0]
    if len(neg) == 0:
        return {f"tar@far={f:g}": float("nan") for f in target_fars}

    out: dict = {}
    # Sort negatives descending to find the score that produces a given FAR.
    neg_sorted = np.sort(neg)[::-1]
    for far in target_fars:
        k = max(1, int(np.ceil(far * len(neg))))
        if k > len(neg_sorted):
            out[f"tar@far={far:g}"] = 1.0
            continue
        thr = neg_sorted[k - 1]  # the highest score we still call "negative"
        tar = float(np.mean(pos > thr))
        out[f"tar@far={far:g}"] = tar
    return out
