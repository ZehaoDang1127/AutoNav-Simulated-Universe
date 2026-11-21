"""Estimate travel direction from observed local lane markings.

The input grids are local sensor observations, not a pre-authored track or
waypoints. A solid-line input must contain classified paint only; obstacles
from a combined occupancy grid cannot be assumed to represent a boundary.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .local_drive import LocalGrid


@dataclass(frozen=True)
class TangentEstimate:
    yaw_rad: float
    source: str
    confidence: float
    point_count: int


def _estimate(grid, x, y, reference_yaw, source, lookahead_m, fit_radius_m,
              max_distance_m, min_span_m, min_confidence, max_deviation_rad=None):
    if grid is None or not grid.valid():
        return None
    res = grid.resolution
    ox, oy, angle = grid.origin_x, grid.origin_y, grid.origin_yaw
    c, s = math.cos(angle), math.sin(angle)
    look_x = x + lookahead_m * math.cos(reference_yaw)
    look_y = y + lookahead_m * math.sin(reference_yaw)
    # Each marked cell contributes its centre in the common odometry frame.
    points = []
    for index, value in enumerate(grid.cells):
        # A classified line-layer costmap includes an inflated 1..99 halo.
        # Fit only observed lethal paint cells, not that broad soft field.
        if value < (100 if source == 'boundary' else 1):
            continue
        px = (index % grid.width + .5) * res
        py = (index // grid.width + .5) * res
        wx, wy = ox+c*px-s*py, oy+s*px+c*py
        if math.hypot(wx-look_x, wy-look_y) <= max_distance_m + fit_radius_m:
            points.append((wx, wy))
    if not points:
        return None
    nearest = min(points, key=lambda p: math.hypot(p[0]-look_x, p[1]-look_y))
    if math.hypot(nearest[0]-look_x, nearest[1]-look_y) > max_distance_m:
        return None
    fit = [(px, py) for px, py in points
           if math.hypot(px-nearest[0], py-nearest[1]) <= fit_radius_m]
    if len(fit) < 5:
        return None
    mx = sum(p[0] for p in fit)/len(fit)
    my = sum(p[1] for p in fit)/len(fit)
    xx = sum((px-mx)**2 for px, _ in fit)/len(fit)
    yy = sum((py-my)**2 for _, py in fit)/len(fit)
    xy = sum((px-mx)*(py-my) for px, py in fit)/len(fit)
    magnitude = math.hypot(xx-yy, 2*xy)
    total = xx+yy
    if total <= 1e-9:
        return None
    confidence = magnitude/total
    if confidence < min_confidence:
        return None
    yaw = .5*math.atan2(2*xy, xx-yy)
    ux, uy = math.cos(yaw), math.sin(yaw)
    projections = [(px-mx)*ux+(py-my)*uy for px, py in fit]
    if max(projections)-min(projections) < min_span_m:
        return None
    alignment = math.cos(yaw-reference_yaw)
    # An undirected stripe exactly perpendicular to the known motion carries
    # no evidence for which of its two directions is the forward direction.
    if abs(alignment) < .087:
        return None
    if alignment < 0:
        yaw += math.pi
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    if max_deviation_rad is not None:
        error = (yaw-reference_yaw+math.pi) % (2*math.pi)-math.pi
        # A lane cannot turn this far between two observations. Paint across
        # the road (a stop line, possibly bridged to a divider end) can.
        if abs(error) > max_deviation_rad:
            return None
    return TangentEstimate(yaw, source,
                           min(1.0, max(0.0, confidence)), len(fit))


def estimate_corridor_tangent(divider: LocalGrid | None, x: float, y: float,
                              reference_yaw: float, boundary: LocalGrid | None = None,
                              *, lookahead_m=1.0, fit_radius_m=1.15,
                              max_distance_m=2.25, min_span_m=.50,
                              min_confidence=.65,
                              max_deviation_rad=None) -> TangentEstimate | None:
    """Fit a nearby observed marking and orient it along previous motion.

    Unknown cells contribute no observations. Input directions must already
    share the grid frame. `boundary` is optional and must be a classified
    paint-only map, never the obstacle-bearing combined costmap. No state or
    stale-data handling is performed here: the caller owns timestamps.
    ``max_deviation_rad`` rejects a source whose fit differs from
    ``reference_yaw`` by more than this, so the next source is tried.
    """
    values = (x, y, reference_yaw, lookahead_m, fit_radius_m, max_distance_m,
              min_span_m, min_confidence)
    if max_deviation_rad is not None and not (math.isfinite(max_deviation_rad) and max_deviation_rad > 0):
        raise ValueError('invalid tangent estimation inputs')
    if (not all(math.isfinite(v) for v in values) or lookahead_m < 0 or fit_radius_m <= 0
            or max_distance_m <= 0 or min_span_m <= 0 or not 0 <= min_confidence <= 1):
        raise ValueError('invalid tangent estimation inputs')
    for grid, source in ((divider, 'divider'), (boundary, 'boundary')):
        result = _estimate(grid, x, y, reference_yaw, source, lookahead_m,
                           fit_radius_m, max_distance_m, min_span_m, min_confidence,
                           max_deviation_rad)
        if result is not None:
            return result
    return None
