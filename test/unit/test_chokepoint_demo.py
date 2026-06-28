import numpy as np

from eval.chokepoint_demo import GalleryEntry, _identify


def test_chokepoint_identify_prefers_previous_id_when_close_to_best():
    gallery = [
        GalleryEntry("0003", np.array([0.0, 0.0])),
        GalleryEntry("0012", np.array([0.11, 0.0])),
    ]
    result = _identify(
        gallery,
        np.array([0.06, 0.0]),
        tolerance=0.75,
        previous_id="0003",
        identity_margin=0.02,
    )

    assert result.nearest_id == "0012"
    assert result.pred_id == "0003"
    assert result.reason == "previous_margin"


def test_chokepoint_identify_uses_nearest_when_previous_id_is_not_close():
    gallery = [
        GalleryEntry("0003", np.array([0.0, 0.0])),
        GalleryEntry("0012", np.array([0.30, 0.0])),
    ]
    result = _identify(
        gallery,
        np.array([0.24, 0.0]),
        tolerance=0.75,
        previous_id="0003",
        identity_margin=0.02,
    )

    assert result.nearest_id == "0012"
    assert result.pred_id == "0012"
    assert result.reason == "nearest"
