"""Engagement scorer (CI + EMA + hysteresis) tests."""
import pytest

from susumu_face_engagement_detector.engagement import (
    DISENGAGED,
    DISENGAGING,
    ENGAGED,
    ENGAGING,
    UNKNOWN,
    EngagementConfig,
    EngagementInputs,
    EngagementScorer,
)


def _full_engagement_inputs() -> EngagementInputs:
    return EngagementInputs(
        emotion_label="neutral",  # neutral has the max weight (0.9)
        gaze_angle_deg=5.0,        # well within gaze_full_deg
        yaw_deg=0.0,
        pitch_deg=0.0,
    )


def test_initial_state_is_unknown() -> None:
    scorer = EngagementScorer()
    assert scorer.state.level == UNKNOWN
    assert scorer.state.score == 0.0


def test_missing_modalities_keep_state_unchanged() -> None:
    scorer = EngagementScorer()
    out = scorer.update(EngagementInputs(emotion_label=None, gaze_angle_deg=None))
    assert out.level == UNKNOWN


def test_sustained_high_score_reaches_engaged() -> None:
    scorer = EngagementScorer()
    # Hammer the scorer until the EMA settles + the engaged hold trips.
    final = None
    for _ in range(40):
        final = scorer.update(_full_engagement_inputs())
    assert final is not None
    assert final.level == ENGAGED
    assert final.score >= 0.66


def test_intermediate_score_yields_engaging() -> None:
    scorer = EngagementScorer()
    # Use a less-weighted emotion (happy = 0.7/0.9 ≈ 0.78 max) AND half gaze.
    inputs = EngagementInputs(
        emotion_label="happy", gaze_angle_deg=25.0, yaw_deg=0.0, pitch_deg=0.0
    )
    final = None
    for _ in range(40):
        final = scorer.update(inputs)
    assert final is not None
    assert final.level in (ENGAGING, ENGAGED) or final.level == DISENGAGING
    # Score must sit in the engaging band, not at the engaged ceiling.
    assert final.score < 0.66


def test_pose_gate_demotes_engaged_when_yaw_too_large() -> None:
    scorer = EngagementScorer()
    # First bring it up to engaged…
    for _ in range(40):
        scorer.update(_full_engagement_inputs())
    assert scorer.state.level == ENGAGED
    # …then yaw goes off-axis: gate penalty kicks in, score drops.
    pose_off = EngagementInputs(
        emotion_label="neutral", gaze_angle_deg=5.0, yaw_deg=45.0, pitch_deg=0.0
    )
    final = None
    for _ in range(40):
        final = scorer.update(pose_off)
    assert final is not None
    assert final.level in (DISENGAGED, DISENGAGING)


def test_gaze_off_axis_pushes_to_disengaged() -> None:
    scorer = EngagementScorer()
    inputs = EngagementInputs(
        emotion_label="neutral", gaze_angle_deg=80.0, yaw_deg=0.0, pitch_deg=0.0
    )
    final = None
    for _ in range(40):
        final = scorer.update(inputs)
    assert final is not None
    assert final.level == DISENGAGED


def test_reset_returns_to_unknown() -> None:
    scorer = EngagementScorer()
    for _ in range(40):
        scorer.update(_full_engagement_inputs())
    assert scorer.state.level == ENGAGED
    scorer.reset()
    assert scorer.state.level == UNKNOWN
    assert scorer.state.score == 0.0


def test_custom_config_overrides_thresholds() -> None:
    # With a deliberately unreachable threshold (1.5), even the maximum-strength
    # signal must never cross it.
    cfg = EngagementConfig(engaged_threshold=1.5, engaging_threshold=1.2,
                           engaged_hold=1)
    scorer = EngagementScorer(config=cfg)
    final = None
    for _ in range(40):
        final = scorer.update(_full_engagement_inputs())
    assert final is not None
    assert final.level != ENGAGED
