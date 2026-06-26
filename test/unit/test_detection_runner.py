"""Smoke test for run_detection_eval against a tiny in-memory dataset.

Note: legacy tests under test/legacy/ patch ``sys.modules['cv2']`` at import
time. We isolate them under test/legacy and exclude that path from pytest
discovery (see pytest.ini), but to be robust we point this test at an
existing on-disk JPEG so cv2.imread inside the runner reads real bytes.
"""
from pathlib import Path

import pytest

from eval.datasets.wider_face import WiderFaceImage
from eval.metrics.detection import Box
from eval.runners.run_detection_eval import run_direct


SAMPLE_IMAGE = (
    Path.home() / "datasets" / "wider_face" / "WIDER_val"
    / "images" / "0--Parade" / "0_Parade_Parade_0_12.jpg"
)


pytestmark = pytest.mark.skipif(
    not SAMPLE_IMAGE.exists(), reason=f"missing {SAMPLE_IMAGE}"
)


class _StubBackend:
    """Returns boxes from a script — used to validate the runner end-to-end."""

    name = "stub"

    def __init__(self, script: dict) -> None:
        self._script = script
        self._calls = 0

    def detect(self, _bgr) -> list:
        self._calls += 1
        return list(self._script.get(self._calls - 1, []))


@pytest.fixture
def tiny_dataset() -> list:
    """Reuse an existing image 3 times — different IDs, same pixels."""
    return [
        WiderFaceImage(
            image_id=f"sample_{i}",
            image_path=SAMPLE_IMAGE,
            ground_truth=[Box(10, 10, 50, 50)],
            ignore=[False],
        )
        for i in range(3)
    ]


def test_runner_perfect_predictions_yield_ap_one(tiny_dataset: list) -> None:
    backend = _StubBackend({
        0: [(Box(10, 10, 50, 50), 0.99)],
        1: [(Box(10, 10, 50, 50), 0.99)],
        2: [(Box(10, 10, 50, 50), 0.99)],
    })
    result = run_direct(backend, tiny_dataset, iou_threshold=0.5, progress_every=0)
    assert result["overall"]["ap"] == pytest.approx(1.0)
    assert result["num_images"] == 3
    assert result["latency_ms"]["count" if "count" in result["latency_ms"] else "mean"] is not None


def test_runner_zero_predictions_yield_ap_zero(tiny_dataset: list) -> None:
    backend = _StubBackend({0: [], 1: [], 2: []})
    result = run_direct(backend, tiny_dataset, iou_threshold=0.5, progress_every=0)
    assert result["overall"]["ap"] == 0.0


def test_runner_limit_caps_image_count(tiny_dataset: list) -> None:
    backend = _StubBackend({i: [] for i in range(3)})
    result = run_direct(backend, tiny_dataset, iou_threshold=0.5, limit=2, progress_every=0)
    assert result["num_images"] == 2
