"""LFW (Labeled Faces in the Wild) dataset loader for verification benchmarks.

Uses scikit-learn's fetch_lfw_pairs, which auto-downloads to
~/scikit_learn_data/. LFW pairs come in 10 folds with 600 pairs each
(half same/half different).

The loader yields (image_a, image_b, same_person) triples — image arrays
are uint8 greyscale to make the embedding backend swap (dlib vs ArcFace
vs MagFace) transparent.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator

import numpy as np


@dataclass
class LfwPair:
    image_a: np.ndarray  # shape (H, W, 3) uint8
    image_b: np.ndarray
    same: bool
    pair_id: str


def load_pairs(subset: str = "test", color: bool = True) -> list[LfwPair]:
    """Load LFW pairs.

    subset: "train" | "test" | "10_folds"
    color:  True returns 3-channel RGB uint8; False returns 1-channel.

    Returns up to ~6,000 pairs (test subset = 1,000; 10_folds = 6,000).
    """
    from sklearn.datasets import fetch_lfw_pairs

    data = fetch_lfw_pairs(subset=subset, color=color, resize=1.0)
    # data.pairs shape: (n_pairs, 2, H, W) or (n_pairs, 2, H, W, 3)
    pairs = data.pairs
    labels = data.target  # 1 = same, 0 = different
    out: list[LfwPair] = []
    for i in range(pairs.shape[0]):
        ia = pairs[i, 0]
        ib = pairs[i, 1]
        # fetch_lfw_pairs returns float in [0, 1]; convert to uint8 for the
        # detection / embedding backends that expect 8-bit input.
        if ia.dtype != np.uint8:
            ia = np.clip(ia * 255.0, 0, 255).astype(np.uint8)
            ib = np.clip(ib * 255.0, 0, 255).astype(np.uint8)
        out.append(LfwPair(image_a=ia, image_b=ib, same=bool(labels[i]),
                           pair_id=f"{subset}_{i:05d}"))
    return out


def iter_pairs(subset: str = "test") -> Iterator[LfwPair]:
    yield from load_pairs(subset=subset)
