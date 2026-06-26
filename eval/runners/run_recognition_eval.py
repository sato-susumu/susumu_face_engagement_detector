"""1:1 face verification evaluation on LFW pairs.

Generates embeddings with the chosen backend, computes cosine similarity,
finds the best threshold, and reports verification accuracy + TAR@FAR.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

from eval.datasets.lfw import LfwPair, load_pairs
from eval.metrics.recognition import Pair, best_threshold_accuracy, tar_at_far


# ---------------------------------------------------------------------------
# Embedding backends
# ---------------------------------------------------------------------------


class DlibBackend:
    """face_recognition 128-D ResNet-34 embeddings (Boost SL / public domain)."""

    name = "dlib_128d"

    def __init__(self, **_kw) -> None:
        import face_recognition
        self._fr = face_recognition

    def embed(self, rgb: np.ndarray) -> Optional[np.ndarray]:
        """Compute one embedding; returns None if no face detected."""
        # LFW images are already tight crops — give face_recognition a hint
        # by passing the whole image as the single face location.
        h, w = rgb.shape[:2]
        loc = [(0, w, h, 0)]  # top, right, bottom, left
        encs = self._fr.face_encodings(rgb, loc)
        if not encs:
            return None
        return encs[0]


BACKENDS = {"dlib_128d": DlibBackend}


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------


def evaluate(backend, lfw_pairs: List[LfwPair], limit: int = 0,
             progress_every: int = 200) -> dict:
    matched_pairs: List[Pair] = []
    skipped = 0
    latencies_ms: List[float] = []
    start = time.perf_counter()
    for i, lp in enumerate(lfw_pairs):
        if limit and i >= limit:
            break
        t0 = time.perf_counter()
        ea = backend.embed(lp.image_a)
        eb = backend.embed(lp.image_b)
        latencies_ms.append((time.perf_counter() - t0) * 1000.0)
        if ea is None or eb is None:
            skipped += 1
            continue
        matched_pairs.append(Pair(a=ea, b=eb, same=lp.same))
        if progress_every and (i + 1) % progress_every == 0:
            print(f"  [{i+1} pairs] matched={len(matched_pairs)}, skipped={skipped}, "
                  f"avg embed {np.mean(latencies_ms):.1f} ms", flush=True)
    elapsed = time.perf_counter() - start

    acc = best_threshold_accuracy(matched_pairs)
    tar = tar_at_far(matched_pairs)
    lat = np.array(latencies_ms) if latencies_ms else np.array([0.0])
    return {
        "accuracy": acc,
        "tar_at_far": tar,
        "num_pairs_input": len(lfw_pairs) if not limit else min(limit, len(lfw_pairs)),
        "num_pairs_matched": len(matched_pairs),
        "num_skipped": skipped,
        "latency_ms": {
            "mean": float(np.mean(lat)),
            "p50": float(np.percentile(lat, 50)),
            "p95": float(np.percentile(lat, 95)),
            "p99": float(np.percentile(lat, 99)),
        },
        "elapsed_seconds": elapsed,
    }


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--backend", required=True, choices=list(BACKENDS))
    p.add_argument("--subset", default="test", choices=["train", "test", "10_folds"])
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--limit", type=int, default=0)
    p.add_argument("--label", default=None)
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    print(f"[run_recognition_eval] loading LFW {args.subset!r} pairs…", flush=True)
    pairs = load_pairs(subset=args.subset)
    print(f"[run_recognition_eval] {len(pairs)} pairs loaded", flush=True)

    backend = BACKENDS[args.backend]()
    print(f"[run_recognition_eval] backend: {backend.name}", flush=True)

    result = evaluate(backend, pairs, limit=args.limit)

    payload = {
        "backend": backend.name,
        "label": args.label or backend.name,
        "dataset": f"lfw_pairs_{args.subset}",
        **result,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2))
    print(f"[run_recognition_eval] wrote {args.output}")
    print(f"  accuracy: {payload['accuracy']['accuracy'] * 100:.2f}% @ thr={payload['accuracy']['threshold']:.3f}")
    print(f"  matched/input: {payload['num_pairs_matched']}/{payload['num_pairs_input']}")


if __name__ == "__main__":
    main()
