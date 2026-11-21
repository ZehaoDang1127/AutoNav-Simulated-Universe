"""Lane-aware v2 of the Python reference map (not a Nav2 costmap plugin).

Lane IDs are public, observed semantics on the same grid as ``cells``. They
never replace obstacle/grade/boundary labels. A transition has a cost in
equivalent metres, independent of map resolution or divider paint thickness.
"""
from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass

from .perception import ReferenceCostmap

Cell = tuple[int, int]
Point = tuple[float, float]
HARD_LABELS = frozenset(("obstacle", "steep", "boundary"))


@dataclass(frozen=True)
class LaneCostConfig:
    lane_change_cost_m: float = 0.25
    clearance_weight: float = 3.0
    clearance_distance_m: float = 0.8
    unknown_multiplier: float = 1.15

    def __post_init__(self):
        for name in ("lane_change_cost_m", "clearance_weight", "clearance_distance_m"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if not math.isfinite(self.unknown_multiplier) or self.unknown_multiplier < 1:
            raise ValueError("unknown_multiplier must be finite and at least 1")


@dataclass(frozen=True)
class LanePlan:
    status: str
    points: tuple[Point, ...] = ()  # Includes exact start and goal.
    length_m: float = 0.0
    clearance_cost_m: float = 0.0
    unknown_cost_m: float = 0.0
    lane_changes: int = 0
    lane_change_cost_m: float = 0.0

    @property
    def total_cost_m(self) -> float:
        if self.status != "ok":
            return math.inf
        return (self.length_m + self.clearance_cost_m + self.unknown_cost_m
                + self.lane_change_cost_m)


class LaneAwareCostmap(ReferenceCostmap):
    """A* over (cell, last known lane), with independent hard and soft costs.

    A missing lane label does not erase lane history. Travelling through an
    unlabelled divider strip therefore cannot avoid a transition penalty.
    Footprint checking conservatively uses an axis-aligned square of half-size
    ``half_width_m`` swept along each segment; callers must supply a size that
    encloses their robot. No kinematic or dynamic feasibility is implied.
    """

    def __init__(self, grade_limit_percent: float = 20.0, resolution_m: float = 0.3,
                 config: LaneCostConfig | None = None):
        if not math.isfinite(resolution_m) or resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        super().__init__(grade_limit_percent, resolution_m)
        self.config = config or LaneCostConfig()
        self.lane_ids: dict[Cell, str] = {}
        self.last_plan = LanePlan("not_planned")

    def update(self, cloud, odom_x, odom_y, odom_yaw):
        boundaries = {cell for cell, label in self.cells.items() if label == "boundary"}
        elapsed = super().update(cloud, odom_x, odom_y, odom_yaw)
        for cell in boundaries:
            self.cells[cell] = "boundary"
        return elapsed

    def _hard_cells(self):
        return {cell for cell, label in self.cells.items() if label in HARD_LABELS}

    def segment_is_clear(self, start: Point, end: Point, half_width_m: float,
                         hard: set[Cell] | None = None) -> bool:
        """Slab intersection with occupied squares enlarged by the footprint."""
        if not math.isfinite(half_width_m) or half_width_m < 0:
            raise ValueError("half_width_m must be finite and nonnegative")
        if not all(math.isfinite(v) for point in (start, end) for v in point):
            raise ValueError("path coordinates must be finite")
        hard = self._hard_cells() if hard is None else hard
        res = self.resolution_m
        extent = half_width_m + res / 2
        lo_x = math.ceil((min(start[0], end[0]) - extent) / res)
        hi_x = math.floor((max(start[0], end[0]) + extent) / res)
        lo_y = math.ceil((min(start[1], end[1]) - extent) / res)
        hi_y = math.floor((max(start[1], end[1]) + extent) / res)
        for ix in range(lo_x, hi_x + 1):
            for iy in range(lo_y, hi_y + 1):
                if (ix, iy) not in hard:
                    continue
                t0, t1 = 0.0, 1.0
                for a, b, centre in zip(start, end, (ix * res, iy * res)):
                    delta = b - a
                    if abs(delta) < 1e-12:
                        if a < centre - extent or a > centre + extent:
                            t0, t1 = 1.0, 0.0
                            break
                    else:
                        near, far = sorted(((centre - extent - a) / delta,
                                            (centre + extent - a) / delta))
                        t0, t1 = max(t0, near), min(t1, far)
                if t0 <= t1:
                    return False
        return True

    def clearance_field(self, half_width_m: float) -> dict[Cell, float]:
        """Dimensionless soft cost per metre beyond the hard footprint buffer."""
        reach = self.config.clearance_distance_m
        if reach == 0 or self.config.clearance_weight == 0:
            return {}
        extent = half_width_m + self.resolution_m / 2
        radius = math.ceil((extent + reach) / self.resolution_m)
        kernel = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                distance = math.hypot(max(0, abs(dx) * self.resolution_m - extent),
                                      max(0, abs(dy) * self.resolution_m - extent))
                if distance < reach:
                    kernel.append((dx, dy, self.config.clearance_weight * (1 - distance / reach)**2))
        field: dict[Cell, float] = {}
        for x, y in self._hard_cells():
            for dx, dy, cost in kernel:
                cell = (x + dx, y + dy)
                field[cell] = max(field.get(cell, 0), cost)
        return field

    def plan(self, start: Point, goal: Point, half_width_m: float = 0.21) -> list[Point]:
        self.last_plan = self.plan_with_costs(start, goal, half_width_m)
        return list(self.last_plan.points[1:])

    def plan_with_costs(self, start: Point, goal: Point,
                        half_width_m: float = 0.21) -> LanePlan:
        # Validate coordinates and footprint even on empty maps.
        hard = self._hard_cells()
        if not self.segment_is_clear(start, start, half_width_m, hard):
            return LanePlan("start_blocked")
        if not self.segment_is_clear(goal, goal, half_width_m, hard):
            return LanePlan("goal_blocked")
        origin, target = self._cell(*start), self._cell(*goal)
        res, config = self.resolution_m, self.config
        soft = self.clearance_field(half_width_m)
        if origin == target:
            if not self.segment_is_clear(start, goal, half_width_m, hard):
                return LanePlan("connection_blocked")
            length = math.dist(start, goal)
            return LanePlan("ok", (start,) if start == goal else (start, goal), length,
                            length * soft.get(origin, 0),
                            length * (config.unknown_multiplier - 1) * (origin not in self.cells))
        # Anchor the search to the exact endpoints. Snapping the first point
        # back to its cell centre on every replan can make a moving robot loop.
        def centre(cell):
            if cell == origin:
                return start
            if cell == target:
                return goal
            return cell[0] * res, cell[1] * res
        # Expand only when the smaller search window cannot find a path.
        # This avoids rejecting feasible detours just beyond the original 3 m
        # window while keeping no-path searches bounded on an unbounded grid.
        clear_edges = {}
        moves = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
        for padding_m in (3.0, 6.0, 12.0):
            pad = math.ceil(padding_m / res)
            min_x, max_x = min(origin[0], target[0]) - pad, max(origin[0], target[0]) + pad
            min_y, max_y = min(origin[1], target[1]) - pad, max(origin[1], target[1]) + pad
            initial = (origin, self.lane_ids.get(origin))
            distance = {initial: 0.0}
            parent = {}
            counter = itertools.count()
            queue = [(0.0, next(counter), 0.0, initial)]
            reached_window_edge = False
            while queue:
                _, _, g, state = heapq.heappop(queue)
                if g != distance[state]:
                    continue
                node, lane = state
                if node[0] in (min_x, max_x) or node[1] in (min_y, max_y):
                    reached_window_edge = True
                if node == target:
                    states = [state]
                    while state != initial:
                        state = parent[state]
                        states.append(state)
                    states.reverse()
                    points = [start]
                    for point in [*(centre(s[0]) for s in states), goal]:
                        if point != points[-1]:
                            points.append(point)
                    length = clearance = unknown = 0.0
                    for a, b in zip(points, points[1:]):
                        step = math.dist(a, b)
                        ca, cb = self._cell(*a), self._cell(*b)
                        length += step
                        clearance += step * (soft.get(ca, 0) + soft.get(cb, 0)) / 2
                        unknown += step * (config.unknown_multiplier - 1) * (
                            (ca not in self.cells) + (cb not in self.cells)) / 2
                    changes = sum(a[1] is not None and b[1] is not None and a[1] != b[1]
                                  for a, b in zip(states, states[1:]))
                    return LanePlan("ok", tuple(points), length, clearance, unknown,
                                    changes, changes * config.lane_change_cost_m)
                for dx, dy in moves:
                    nxt = (node[0] + dx, node[1] + dy)
                    if not (min_x <= nxt[0] <= max_x and min_y <= nxt[1] <= max_y):
                        continue
                    edge = tuple(sorted((node, nxt)))
                    if edge not in clear_edges:
                        clear_edges[edge] = self.segment_is_clear(centre(node), centre(nxt), half_width_m, hard)
                    if not clear_edges[edge]:
                        continue
                    next_lane = self.lane_ids.get(nxt, lane)
                    change = lane is not None and next_lane is not None and lane != next_lane
                    step = math.dist(centre(node), centre(nxt))
                    traversal = step * (1 + (soft.get(node, 0) + soft.get(nxt, 0)) / 2
                                        + (config.unknown_multiplier - 1) * (
                                            (node not in self.cells) + (nxt not in self.cells)) / 2)
                    candidate = g + traversal + change * config.lane_change_cost_m
                    successor = (nxt, next_lane)
                    if candidate < distance.get(successor, math.inf):
                        distance[successor] = candidate
                        parent[successor] = state
                        heuristic = math.dist(centre(nxt), goal)
                        heapq.heappush(queue, (candidate + heuristic, next(counter), candidate, successor))
            # An enclosed reachable region cannot gain new cells by expanding.
            if not reached_window_edge:
                break
        return LanePlan("no_path")
