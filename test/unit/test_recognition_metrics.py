"""Sanity tests for recognition metrics."""
import math

import numpy as np
import pytest

from eval.metrics.recognition import Pair, best_threshold_accuracy, cosine_distance, tar_at_far


def _emb(n: int = 8, seed: int | None = None) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(n)
    return v / np.linalg.norm(v)


def test_cosine_distance_identical_is_zero() -> None:
    v = _emb(8, seed=1)
    assert cosine_distance(v, v) == pytest.approx(0.0, abs=1e-9)


def test_cosine_distance_orthogonal_is_one() -> None:
    a = np.array([1.0, 0.0, 0.0])
    b = np.array([0.0, 1.0, 0.0])
    assert cosine_distance(a, b) == pytest.approx(1.0)


def test_accuracy_perfectly_separable() -> None:
    rng = np.random.default_rng(0)
    pairs = []
    for _ in range(10):
        v = rng.standard_normal(8)
        v = v / np.linalg.norm(v)
        pairs.append(Pair(a=v, b=v.copy(), same=True))
    for _ in range(10):
        a = rng.standard_normal(8); a /= np.linalg.norm(a)
        b = -a
        pairs.append(Pair(a=a, b=b, same=False))
    res = best_threshold_accuracy(pairs)
    assert res["accuracy"] == 1.0
    assert res["num_pairs"] == 20


def test_accuracy_no_pairs_is_nan() -> None:
    res = best_threshold_accuracy([])
    assert math.isnan(res["accuracy"])
    assert res["num_pairs"] == 0


def test_tar_at_far_perfectly_separable_is_one() -> None:
    rng = np.random.default_rng(1)
    pairs = []
    for _ in range(50):
        v = rng.standard_normal(16); v /= np.linalg.norm(v)
        pairs.append(Pair(a=v, b=v.copy(), same=True))
    for _ in range(50):
        a = rng.standard_normal(16); a /= np.linalg.norm(a)
        b = -a
        pairs.append(Pair(a=a, b=b, same=False))
    out = tar_at_far(pairs, target_fars=(1e-1, 1e-2))
    assert out["tar@far=0.1"] == pytest.approx(1.0)
    assert out["tar@far=0.01"] == pytest.approx(1.0)
