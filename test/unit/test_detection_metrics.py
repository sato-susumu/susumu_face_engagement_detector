"""Sanity tests for detection metrics."""
import math

import pytest

from eval.metrics.detection import Box, FrameDetections, average_precision, iou


def test_iou_identical_boxes_is_one() -> None:
    a = Box(10, 10, 50, 50)
    assert iou(a, a) == pytest.approx(1.0)


def test_iou_disjoint_is_zero() -> None:
    a = Box(0, 0, 10, 10)
    b = Box(100, 100, 10, 10)
    assert iou(a, b) == 0.0


def test_iou_half_overlap() -> None:
    a = Box(0, 0, 10, 10)
    b = Box(5, 0, 10, 10)  # inter 5x10, union 100+100-50
    assert iou(a, b) == pytest.approx(50.0 / 150.0)


def test_ap_perfect_detection_is_one() -> None:
    frames = [
        FrameDetections(
            image_id="img1",
            predictions=[(Box(10, 10, 50, 50), 0.99)],
            ground_truth=[Box(10, 10, 50, 50)],
        )
    ]
    res = average_precision(frames, iou_threshold=0.5)
    assert res["ap"] == pytest.approx(1.0)
    assert res["num_gt"] == 1
    assert res["num_pred"] == 1


def test_ap_missed_detection_is_zero() -> None:
    frames = [
        FrameDetections(
            image_id="img1",
            predictions=[],
            ground_truth=[Box(10, 10, 50, 50)],
        )
    ]
    res = average_precision(frames, iou_threshold=0.5)
    assert res["ap"] == 0.0


def test_ap_false_positive_only_is_zero() -> None:
    frames = [
        FrameDetections(
            image_id="img1",
            predictions=[(Box(0, 0, 10, 10), 0.9)],
            ground_truth=[Box(500, 500, 10, 10)],
        )
    ]
    res = average_precision(frames, iou_threshold=0.5)
    assert res["ap"] == 0.0


def test_ap_no_gt_is_nan() -> None:
    frames = [
        FrameDetections(image_id="img1", predictions=[], ground_truth=[])
    ]
    res = average_precision(frames, iou_threshold=0.5)
    assert math.isnan(res["ap"])


def test_ap_mixed_tp_fp() -> None:
    # 2 GT, 3 predictions: one perfect TP, one FP, one missed GT.
    frames = [
        FrameDetections(
            image_id="img1",
            predictions=[
                (Box(0, 0, 10, 10), 0.9),   # TP (matches GT1)
                (Box(500, 500, 10, 10), 0.8),  # FP
                (Box(0, 0, 10, 10), 0.7),   # FP — GT1 already matched
            ],
            ground_truth=[Box(0, 0, 10, 10), Box(100, 100, 10, 10)],
        )
    ]
    res = average_precision(frames, iou_threshold=0.5)
    # Recall reaches at most 0.5 (1 of 2 GT found). AP must be <= 0.5.
    assert 0.0 < res["ap"] <= 0.5
