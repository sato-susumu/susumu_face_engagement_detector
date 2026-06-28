"""Engagement scoring — pure Python, framework-free.

Combines:

  * emotion weight (per canonical label, see EMOTION_WEIGHTS)
  * head pose gate (yaw within ±yaw_gate_deg AND pitch within ±pitch_gate_deg)

Raw score r = emotion_weight / max(EMOTION_WEIGHTS.values())
filtered = EMA(r, α)
gated    = filtered if head-pose gate passes else filtered * gate_penalty

Five-state EngagementLevel is derived from `gated` with hysteresis to keep
the published label from flapping.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


EMOTION_WEIGHTS = {
    "neutral": 0.9,
    "happy": 0.7,
    "surprise": 0.6,
    "sad": 0.3,
    "anger": 0.2,
    "fear": 0.2,
    "disgust": 0.1,
    "contempt": 0.2,
}

# 5-state EngagementLevel values (mirror hri_msgs/EngagementLevel constants).
UNKNOWN = 0
DISENGAGED = 1
ENGAGING = 2
ENGAGED = 3
DISENGAGING = 4

_STATE_NAMES = {
    UNKNOWN: "UNKNOWN",
    DISENGAGED: "DISENGAGED",
    ENGAGING: "ENGAGING",
    ENGAGED: "ENGAGED",
    DISENGAGING: "DISENGAGING",
}


def state_name(level: int) -> str:
    return _STATE_NAMES.get(level, str(level))


@dataclass
class EngagementConfig:
    # Head pose gate — if either axis exceeds the limit, scale down.
    yaw_gate_deg: float = 15.0
    pitch_gate_deg: float = 10.0
    gate_penalty: float = 0.3
    # EMA smoothing factor for raw score.
    ema_alpha: float = 0.3
    # State-transition thresholds.
    engaged_threshold: float = 0.66
    engaging_threshold: float = 0.33
    disengaging_threshold: float = 0.20  # below this for a while → DISENGAGING
    disengaged_threshold: float = 0.10   # below this and sustained → DISENGAGED
    # Persistence (in number of consecutive updates) for entering each state.
    engaged_hold: int = 4
    disengaged_hold: int = 4


@dataclass
class EngagementState:
    score: float = 0.0
    level: int = UNKNOWN
    above_engaged_count: int = 0
    below_disengaged_count: int = 0
    last_update_seq: int = 0


@dataclass
class EngagementInputs:
    """Per-update measurements feeding the scorer."""
    emotion_label: Optional[str] = None
    yaw_deg: Optional[float] = None
    pitch_deg: Optional[float] = None


class EngagementScorer:
    """Stateful scorer — call update() per detection cycle."""

    def __init__(self, config: EngagementConfig | None = None) -> None:
        self.config = config or EngagementConfig()
        self.state = EngagementState()

    def update(self, inputs: EngagementInputs) -> EngagementState:
        cfg = self.config
        # Missing modalities → keep score at 0 and don't change state direction.
        if inputs.emotion_label is None:
            return self.state

        # Emotion weight (normalised by max)
        max_w = max(EMOTION_WEIGHTS.values())
        raw = EMOTION_WEIGHTS.get(inputs.emotion_label, 0.5) / max_w

        # EMA
        a = cfg.ema_alpha
        filt = a * raw + (1.0 - a) * self.state.score

        # Head pose gate
        pose_ok = True
        if inputs.yaw_deg is not None and abs(inputs.yaw_deg) > cfg.yaw_gate_deg:
            pose_ok = False
        if inputs.pitch_deg is not None and abs(inputs.pitch_deg) > cfg.pitch_gate_deg:
            pose_ok = False
        gated = filt if pose_ok else filt * cfg.gate_penalty

        self.state.score = gated
        self.state.level = self._classify(gated)
        self.state.last_update_seq += 1
        return self.state

    def _classify(self, score: float) -> int:
        cfg = self.config
        # Hysteresis counters
        if score >= cfg.engaged_threshold:
            self.state.above_engaged_count += 1
            self.state.below_disengaged_count = 0
        elif score < cfg.disengaged_threshold:
            self.state.below_disengaged_count += 1
            self.state.above_engaged_count = 0
        else:
            self.state.above_engaged_count = 0
            self.state.below_disengaged_count = 0

        current = self.state.level

        # Above the engaged band — climbing or steady.
        if score >= cfg.engaged_threshold:
            if self.state.above_engaged_count >= cfg.engaged_hold:
                return ENGAGED
            return ENGAGED if current == ENGAGED else ENGAGING

        # In the engaging band — climbing-from-below or descending-from-engaged.
        if score >= cfg.engaging_threshold:
            return DISENGAGING if current == ENGAGED else ENGAGING

        # Below engaging threshold.
        # Distinguish "still on the way up from nothing" (UNKNOWN/DISENGAGED)
        # from "lost a previously engaged state" — only the latter warrants
        # the DISENGAGING label.
        if current in (ENGAGED, ENGAGING, DISENGAGING):
            if score < cfg.disengaged_threshold and self.state.below_disengaged_count >= cfg.disengaged_hold:
                return DISENGAGED
            return DISENGAGING

        # current is UNKNOWN or DISENGAGED — stay there until we see enough signal.
        if score >= cfg.disengaging_threshold:
            return ENGAGING  # rising signal from a quiet state
        return DISENGAGED if score < cfg.disengaged_threshold else current

    def reset(self) -> None:
        self.state = EngagementState()
