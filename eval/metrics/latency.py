"""End-to-end latency between two ROS messages by header.stamp matching."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence

import numpy as np


def _stamp_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


@dataclass
class StampedEvent:
    image_stamp_ns: int   # the stamp copied from the input image
    receive_stamp_ns: int  # when this downstream message was received/written


def latency_ms(events: Sequence[StampedEvent]) -> dict:
    """Per-message latency = receive_stamp - image_stamp, in milliseconds.

    Matching is by image stamp (input header.stamp must be propagated through
    the pipeline). Returns p50/p95/p99/mean/std.
    """
    if not events:
        return {"count": 0}
    lat_ns = np.array(
        [max(0, e.receive_stamp_ns - e.image_stamp_ns) for e in events],
        dtype=np.float64,
    )
    lat_ms = lat_ns / 1e6
    return {
        "count": int(len(lat_ms)),
        "mean_ms": float(np.mean(lat_ms)),
        "std_ms": float(np.std(lat_ms)),
        "p50_ms": float(np.percentile(lat_ms, 50)),
        "p95_ms": float(np.percentile(lat_ms, 95)),
        "p99_ms": float(np.percentile(lat_ms, 99)),
        "min_ms": float(np.min(lat_ms)),
        "max_ms": float(np.max(lat_ms)),
    }
