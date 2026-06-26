"""Verify the WIDER FACE GT parser handles canonical edge cases."""
from pathlib import Path

import pytest

from eval.datasets.wider_face import _parse_gt_file, load_subsets


@pytest.fixture
def sample_gt(tmp_path: Path) -> Path:
    """Three entries: 2 valid boxes, 1 invalid box, 1 image with zero boxes."""
    text = (
        "0--Parade/0_Parade_marchingband_1_849.jpg\n"
        "2\n"
        "449 330 122 149 0 0 0 0 0 0\n"
        "10 20 30 40 0 0 0 1 0 0\n"   # invalid=1 → ignore
        "1--Handshaking/1_Handshaking_Handshaking_1_35.jpg\n"
        "1\n"
        "100 200 50 60 0 0 0 0 0 0\n"
        "2--Demonstration/2_Demonstration_Political_Rally_2_444.jpg\n"
        "0\n"
        "0 0 0 0 0 0 0 0 0 0\n"
    )
    p = tmp_path / "wider_face_val_bbx_gt.txt"
    p.write_text(text)
    return p


def test_parse_three_images(sample_gt: Path) -> None:
    entries = _parse_gt_file(sample_gt)
    assert len(entries) == 3


def test_invalid_flag_marks_ignore(sample_gt: Path) -> None:
    entries = _parse_gt_file(sample_gt)
    first = entries[0]
    assert len(first.ground_truth) == 2
    assert first.ignore == [False, True]


def test_zero_box_image_has_no_gt(sample_gt: Path) -> None:
    entries = _parse_gt_file(sample_gt)
    last = entries[2]
    assert last.image_id == "2--Demonstration/2_Demonstration_Political_Rally_2_444"
    assert last.ground_truth == []
    assert last.ignore == []


def test_image_id_strips_extension(sample_gt: Path) -> None:
    entries = _parse_gt_file(sample_gt)
    assert entries[0].image_id == "0--Parade/0_Parade_marchingband_1_849"


def test_subsets_missing_json_returns_empty(tmp_path: Path) -> None:
    # Create the wider_face_split dir but no JSON — must not raise.
    (tmp_path / "wider_face_split").mkdir()
    assert load_subsets(tmp_path) == {}
