import numpy as np

from eval.video_demo import DEMO_SAMPLES, EmbeddingTracker, _bbox_iou, _clamp_box, _pad_box, build_arg_parser
from susumu_face_engagement_detector.backends.headpose import normalize_pose_angle


def test_clamp_box_keeps_valid_pixels_inside_frame():
    assert _clamp_box((-5, 120, 80, -10), width=100, height=60) == (0, 100, 60, 0)
    assert _clamp_box((10, 5, 20, 15), width=100, height=60) is None


def test_video_demo_defaults_reject_low_confidence_faces():
    args = build_arg_parser().parse_args([])
    assert args.sample == "cdc"
    assert args.score_threshold == 0.8
    assert args.identity_margin == 0.15
    assert args.identity_score_threshold == 0.9
    assert args.identity_max_head_yaw_deg == 40.0
    assert args.identity_max_head_pitch_deg == 25.0
    assert args.gaze_backend == "openvino_adas"
    assert args.no_gaze is False


def test_video_demo_has_multi_person_sample():
    assert "multi_person" in DEMO_SAMPLES
    args = build_arg_parser().parse_args(["--sample", "multi_person"])
    assert args.sample == "multi_person"


def test_pad_box_expands_and_clamps_to_frame():
    assert _pad_box((10, 30, 30, 10), width=35, height=35, ratio=0.5) == (0, 35, 35, 0)


def test_embedding_tracker_reuses_matching_id():
    tracker = EmbeddingTracker(tolerance=0.6)
    first = np.zeros(128)
    near = np.zeros(128)
    near[0] = 0.1
    far = np.ones(128)

    assert tracker.identify(first) == "user_1"
    assert tracker.identify(near) == "user_1"
    assert tracker.identify(far) == "user_2"


def test_embedding_tracker_does_not_assign_without_embedding():
    tracker = EmbeddingTracker(tolerance=0.6)
    tracker.start_frame(0)
    assert tracker.identify(None, (10, 50, 60, 10)) == "unidentified"
    assert tracker.tracked_count == 0
    assert tracker.identity_count == 0


def test_embedding_tracker_does_not_reuse_id_by_bbox_only():
    tracker = EmbeddingTracker(tolerance=0.6)
    first = np.zeros(128)
    different_embedding = np.ones(128)

    tracker.start_frame(0)
    assert tracker.identify(first, (10, 50, 60, 10)) == "user_1"

    tracker.start_frame(1)
    assert tracker.identify(different_embedding, (12, 52, 62, 12)) == "user_2"


def test_embedding_tracker_reuses_identity_when_embedding_is_similar():
    tracker = EmbeddingTracker(tolerance=0.6)
    first = np.zeros(128)
    near = np.zeros(128)
    near[0] = 0.1

    assert tracker.identify(first, (10, 50, 60, 10)) == "user_1"
    assert tracker.identify(near, (100, 150, 160, 100)) == "user_1"
    assert tracker.tracked_count == 1
    assert tracker.identity_count == 1


def test_embedding_tracker_reuses_identity_across_frames():
    tracker = EmbeddingTracker(tolerance=0.6)
    first = np.zeros(128)
    near = np.zeros(128)
    near[0] = 0.1

    tracker.start_frame(0)
    assert tracker.identify(first, (10, 50, 60, 10)) == "user_1"

    tracker.start_frame(1)
    assert tracker.identify(near, (100, 150, 160, 100)) == "user_1"
    assert tracker.tracked_count == 1
    assert tracker.identity_count == 1


def test_embedding_tracker_prefers_previous_track_when_distances_are_close():
    tracker = EmbeddingTracker(tolerance=0.6, identity_margin=0.05)
    first = np.array([0.0, 0.0])
    second = np.array([0.70, 0.0])
    query = np.array([0.36, 0.0])

    tracker.start_frame(0)
    assert tracker.identify(first, (10, 50, 60, 10)) == "user_1"
    assert tracker.identify(second, (100, 150, 160, 100)) == "user_2"

    tracker.start_frame(1)
    assert tracker.identify(query, (12, 52, 62, 12)) == "user_1"


def test_embedding_tracker_does_not_reuse_same_user_in_one_frame():
    tracker = EmbeddingTracker(tolerance=0.6)
    first = np.zeros(128)
    duplicate = np.zeros(128)
    duplicate[0] = 0.1

    tracker.start_frame(0)
    assert tracker.identify(first, (10, 50, 60, 10)) == "user_1"
    assert tracker.identify(duplicate, (100, 150, 160, 100)) == "unidentified"
    assert tracker.tracked_count == 1
    assert tracker.identity_count == 1


def test_bbox_iou():
    assert _bbox_iou((0, 10, 10, 0), (0, 10, 10, 0)) == 1.0
    assert _bbox_iou((0, 10, 10, 0), (20, 30, 30, 20)) == 0.0


def test_fold_pose_angle_handles_euler_flips():
    assert normalize_pose_angle(-170.0) == 10.0
    assert normalize_pose_angle(170.0) == -10.0
