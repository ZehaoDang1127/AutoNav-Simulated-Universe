"""Hold self-drive scoring until the simulator's ground-truth pose stabilizes.

The launch must keep motion inhibited while this check runs. This is only a
startup synchronization, not an odometry filter once the referee starts.
"""
from __future__ import annotations

import math
import json


class StartupPoseGuard:
    def __init__(self, start_x, start_y, *, radius_m=0.3, duration_s=1.0,
                 max_gap_s=0.3, max_speed_mps=0.05):
        self.start_x, self.start_y = start_x, start_y
        self.radius, self.duration = radius_m, duration_s
        self.max_gap, self.max_speed = max_gap_s, max_speed_mps
        self.first = self.last = None
        self.anchor = None

    def update(self, x, y, speed, now):
        values = (x, y, speed, now)
        if (not all(math.isfinite(v) for v in values)
                or math.hypot(x-self.start_x, y-self.start_y) > self.radius
                or abs(speed) > self.max_speed
                or (self.last is not None and (now <= self.last or now-self.last > self.max_gap))
                or (self.anchor is not None and math.hypot(x-self.anchor[0], y-self.anchor[1]) > .08)):
            self.first = self.last = self.anchor = None
            return False
        if self.first is None:
            self.first, self.anchor = now, (x, y)
        self.last = now
        return now-self.first >= self.duration


class RefereeHealth:
    """Require current, authoritative referee scores throughout a run."""
    def __init__(self, timeout_s=2.5):
        self.timeout_s = timeout_s
        self.last_ok = -math.inf
        self.failed = False

    def update(self, data, now):
        try:
            score = json.loads(data)
            if score.get('failed') is True or score.get('finish_reached') is True:
                self.failed = True
            age = score.get('last_odom_age_s')
            if (score.get('failed') is False and score.get('finish_reached') is False
                    and score.get('odom_source') == 'primary'
                    and score.get('primary_odom_sample_count', 0) >= 3
                    and isinstance(age, (int, float)) and math.isfinite(age) and 0 <= age <= 1.5):
                self.last_ok = now
            else:
                self.last_ok = -math.inf
        except (TypeError, ValueError, AttributeError):
            self.last_ok = -math.inf

    def permitted(self, now):
        return not self.failed and 0 <= now-self.last_ok <= self.timeout_s
