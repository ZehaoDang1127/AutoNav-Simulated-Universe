"""Short-lived, frame-aware cache for observed lane-direction guidance."""
from __future__ import annotations
import math
from typing import Mapping
from .corridor_tangent import TangentEstimate


class GuidanceCache:
    """Never reuse a measurement after its source, frame or lifetime changes."""
    def __init__(self, ttl_s: float = .75):
        if not math.isfinite(ttl_s) or ttl_s <= 0:
            raise ValueError('guidance lifetime must be finite and positive')
        self.ttl_s = ttl_s
        self.estimate = None
        self.frame = None
        self.received_s = -math.inf

    def clear(self):
        self.estimate = self.frame = None
        self.received_s = -math.inf

    def update(self, estimate: TangentEstimate, frame: str, now: float):
        if (not isinstance(estimate, TangentEstimate) or not math.isfinite(estimate.yaw_rad)
                or not isinstance(frame, str) or not frame or not math.isfinite(now)):
            raise ValueError('invalid guidance observation')
        self.estimate, self.frame, self.received_s = estimate, frame, now

    def get(self, frame: str, now: float,
            source_fresh: Mapping[str, bool]) -> TangentEstimate | None:
        if (self.estimate is None or not isinstance(frame, str) or not frame
                or frame != self.frame or not math.isfinite(now)
                or not 0 <= now-self.received_s <= self.ttl_s
                or not source_fresh.get(self.estimate.source, False)):
            return None
        return self.estimate

    def reference(self, odom_direction: float, frame: str, now: float,
                  source_fresh: Mapping[str, bool]) -> float:
        if not math.isfinite(odom_direction):
            raise ValueError('reference direction must be finite')
        cached = self.get(frame, now, source_fresh)
        return cached.yaw_rad if cached is not None else odom_direction
