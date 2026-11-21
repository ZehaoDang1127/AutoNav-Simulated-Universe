"""Short-horizon velocity search over an observed ROS local costmap.

No map geometry, route, goals, or simulated truth enter the controller. Costmap
values follow nav_msgs/OccupancyGrid conventions: -1 unknown, 0 free, 100 lethal.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

from .lane_costmap import LaneCostConfig
from .divider_transition import observed_divider_transitions

try:
    import numpy as np
except ImportError:
    np = None


@dataclass(frozen=True)
class LocalGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: tuple[int, ...]

    def valid(self):
        return (self.width > 0 and self.height > 0 and math.isfinite(self.resolution)
                and self.resolution > 0 and len(self.cells) == self.width*self.height
                and all(math.isfinite(n) for n in (self.origin_x, self.origin_y, self.origin_yaw)))

    def local(self, x, y):
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        dx, dy = x-self.origin_x, y-self.origin_y
        return (c*dx+s*dy)/self.resolution, (-s*dx+c*dy)/self.resolution


@dataclass(frozen=True)
class DrivePlan:
    linear: float = 0.0
    angular: float = 0.0
    points: tuple[tuple[float, float], ...] = ()
    reason: str = 'blocked'
    second_angular: float = 0.0
    divider_crossings: int = 0
    divider_cost_m: float = 0.0


def integrated_clearance_cost(cost_samples, step_distance_m, weight=3.0):
    """Integrate normalized observed clearance costs by distance travelled."""
    return weight * sum(cost_samples) * step_distance_m


def integrated_heading_deviation(headings, guidance_yaw, step_distance_m, weight=0.6):
    """Cost heading departures along the whole path, not just at its endpoint.

    A paired turn can end aligned with the lane after a large sideways
    excursion. Integrating the wrapped heading error distinguishes it from a
    straighter candidate while leaving hard collision checks independent.
    """
    if guidance_yaw is None:
        return 0.0
    return weight * step_distance_m * sum(1.0 - math.cos(heading-guidance_yaw)
                                          for heading in headings)


def cell_is_unknown(grid: LocalGrid, x: float, y: float) -> bool:
    """Query the observed centre cell; out-of-map is handled as hard separately."""
    gx, gy = grid.local(x, y)
    ix, iy = math.floor(gx), math.floor(gy)
    return (0 <= ix < grid.width and 0 <= iy < grid.height
            and grid.cells[iy*grid.width+ix] < 0)


def unknown_traversal_cost(before_unknown, after_unknown, distance_m, multiplier=1.15):
    """Previous v2 objective: endpoint-averaged unknown premium per metre."""
    return distance_m * (multiplier-1.0) * (int(before_unknown)+int(after_unknown))/2


def divider_crossings(points, divider: LocalGrid | None) -> int:
    """Charge observed side-to-side transitions plus an unfinished entry.

    A touch followed by return to the original side is not a lane change. A
    candidate ending on the marking is provisionally charged to discourage a
    rolling-horizon plan from hiding an unfinished lane change at its endpoint.
    Starting on an unclassified stripe has no recoverable previous-side
    history; an unobserved or ambiguous marking cannot establish a crossing.
    """
    if divider is None or not divider.valid() or not points:
        return 0
    def marked(point):
        gx, gy = divider.local(*point)
        ix, iy = math.floor(gx), math.floor(gy)
        return (0 <= ix < divider.width and 0 <= iy < divider.height
                and divider.cells[iy*divider.width+ix] > 0)
    transitions = observed_divider_transitions(points, divider)
    pending_entry = not marked(points[0]) and marked(points[-1])
    return transitions + int(pending_entry)


class LocalDrivePlanner:
    """Evaluate forward arcs using the robot's filled, padded footprint.

    The body extends from -0.32 m to +0.77 m from base_link and is 0.82 m
    wide, matching full_track_baseline.yaml.  The extra margin is 0.10 m.
    Lethal and outside cells are impassable; unknown centre cells incur the
    previous v2 unknown-space cost. Inputs must already share one frame.
    """
    rear = -0.42
    front = 0.87
    half_width = 0.51
    # A longer sensor-local horizon gives the robot time to see and commit to
    # a lane change. Every sample, including the far endpoint, is still checked
    # against the live rolling costmap and out-of-map space is blocked.
    horizon = 6.0
    dt = 0.20
    hard_cost = 100

    def __init__(self, lane_change_cost_m=0.25):
        if not math.isfinite(lane_change_cost_m) or lane_change_cost_m < 0:
            raise ValueError('lane_change_cost_m must be finite and nonnegative')
        self.lane_change_cost_m = lane_change_cost_m
        previous = LaneCostConfig()
        self.clearance_weight = previous.clearance_weight
        self.unknown_multiplier = previous.unknown_multiplier

    def _footprint_cost(self, grid, x, y, yaw):
        # Check every potentially overlapping cell. Enlarging each body side by
        # the half-diagonal of a cell makes this conservative under rotation.
        gx, gy = grid.local(x, y)
        resolution = grid.resolution
        # Cover the largest possible translation and rotation between rollout
        # samples, as well as cells intersecting the sampled rectangle.
        sweep = (0.35 + 0.54 * math.hypot(self.front, self.half_width)) * self.dt / 2
        padding = resolution * math.sqrt(2) / 2 + sweep
        radius = math.hypot(max(abs(self.rear), self.front), self.half_width) + padding
        n = math.ceil(radius/resolution) + 1
        c, s = math.cos(yaw-grid.origin_yaw), math.sin(yaw-grid.origin_yaw)
        if np is not None:
            # Evaluate exactly the same conservative cell-centre rectangle
            # in bulk. The ROS container supplies NumPy; retain the loop
            # below for headless installations without it.
            if getattr(self, '_cached_grid', None) is not grid:
                self._cached_grid = grid
                self._cached_cells = np.asarray(grid.cells, dtype=np.int16).reshape(grid.height, grid.width)
            xs = np.arange(math.floor(gx)-n, math.floor(gx)+n+1)
            ys = np.arange(math.floor(gy)-n, math.floor(gy)+n+1)
            dx = (xs[None, :] + .5 - gx) * resolution
            dy = (ys[:, None] + .5 - gy) * resolution
            bx, by = c*dx+s*dy, -s*dx+c*dy
            mask = ((bx >= self.rear-padding) & (bx <= self.front+padding)
                    & (np.abs(by) <= self.half_width+padding))
            valid = ((xs[None, :] >= 0) & (xs[None, :] < grid.width)
                     & (ys[:, None] >= 0) & (ys[:, None] < grid.height))
            if np.any(mask & ~valid):
                return None
            clipped_x = np.clip(xs, 0, grid.width-1)
            clipped_y = np.clip(ys, 0, grid.height-1)
            values = self._cached_cells[np.ix_(clipped_y, clipped_x)][mask]
            if np.any(values >= self.hard_cost):
                return None
            return float(values.max(initial=0)) / 100.0
        maximum = 0
        for iy in range(math.floor(gy)-n, math.floor(gy)+n+1):
            for ix in range(math.floor(gx)-n, math.floor(gx)+n+1):
                dx, dy = (ix+0.5-gx)*resolution, (iy+0.5-gy)*resolution
                bx, by = c*dx+s*dy, -s*dx+c*dy
                if not (self.rear-padding <= bx <= self.front+padding and abs(by) <= self.half_width+padding):
                    continue
                if ix < 0 or iy < 0 or ix >= grid.width or iy >= grid.height:
                    return None
                value = grid.cells[iy*grid.width+ix]
                if value >= self.hard_cost:
                    return None
                maximum = max(maximum, value)
        return maximum / 100.0

    def plan(self, grid: LocalGrid, x: float, y: float, yaw: float,
             divider: LocalGrid | None = None, reference_yaw: float | None = None,
             guidance_yaw: float | None = None) -> DrivePlan:
        if not grid.valid() or not all(math.isfinite(v) for v in (x, y, yaw)):
            return DrivePlan(reason='invalid_input')
        initial_cost = self._footprint_cost(grid, x, y, yaw)
        if initial_cost is None:
            return DrivePlan(reason='footprint_blocked_or_unknown')
        best_score, best = -math.inf, None
        for linear in (0.35, 0.22):
            # Include paired turns that return to the original heading, so
            # avoiding an obstacle need not commit to a continuing circle.
            # Steady arcs remain available for bends in the observed road.
            candidates = [(0.0, 0.0)]
            # Finer arcs permit corrections on a straight lane without the
            # ~30 degree excursion of the former smallest 0.18 rad/s turn.
            # Retain the original larger turns for obstacles and bends.
            for angular in (0.06, -0.06, 0.12, -0.12, 0.18, -0.18,
                            0.36, -0.36, 0.54, -0.54):
                candidates.append((angular, -angular))
                if abs(angular) <= 0.36:
                    candidates.append((angular, angular))
            for first_angular, second_angular in candidates:
                px, py, heading = x, y, yaw
                samples, cost_samples, headings = [], [], []
                unknown_cost = 0.0
                previous_cost = initial_cost
                prev_unknown = cell_is_unknown(grid, px, py)
                steps = round(self.horizon/self.dt)
                for step in range(steps):
                    angular = first_angular if step < steps//2 else second_angular
                    # Midpoint integration; all samples test the filled body.
                    mid = heading + angular*self.dt/2
                    px += linear*self.dt*math.cos(mid)
                    py += linear*self.dt*math.sin(mid)
                    heading += angular*self.dt
                    if (reference_yaw is not None and abs(math.atan2(
                            math.sin(heading-reference_yaw), math.cos(heading-reference_yaw))) > 1.50):
                        break
                    cost = self._footprint_cost(grid, px, py, heading)
                    if cost is None:
                        break
                    next_unknown = cell_is_unknown(grid, px, py)
                    unknown_cost += unknown_traversal_cost(prev_unknown, next_unknown,
                                                           linear*self.dt,
                                                           self.unknown_multiplier)
                    prev_unknown = next_unknown
                    # Previous v2 integrates the mean cost at the segment's
                    # endpoints, so path cost is stable under resampling.
                    cost_samples.append((previous_cost+cost)/2)
                    previous_cost = cost
                    samples.append((px, py))
                    headings.append(heading)
                else:
                    # Reward actual displacement in the current forward
                    # direction, not distance travelled on a circular arc.
                    # Preserve the heading when a bypass permits it, while
                    # allowing gradual turns along curved track boundaries.
                    travel_yaw = yaw if guidance_yaw is None else guidance_yaw
                    forward = (px-x)*math.cos(travel_yaw) + (py-y)*math.sin(travel_yaw)
                    heading_error = abs(math.atan2(math.sin(heading-travel_yaw),
                                                   math.cos(heading-travel_yaw)))
                    crossings = divider_crossings([(x, y), *samples], divider)
                    crossing_cost = crossings * self.lane_change_cost_m
                    # Previous LaneCostConfig uses clearance_weight=3 per
                    # metre. ROS cells are already inflated and normalized, so
                    # this is an approximation rather than its analytic field.
                    clearance_cost = integrated_clearance_cost(cost_samples, linear*self.dt,
                                                               self.clearance_weight)
                    heading_cost = integrated_heading_deviation(
                        headings, guidance_yaw, linear*self.dt)
                    score = (forward - clearance_cost - unknown_cost
                             - heading_cost - 0.35*heading_error - 0.08*abs(first_angular)
                             - crossing_cost)
                    if score > best_score:
                        best_score = score
                        best = DrivePlan(linear, first_angular, tuple(samples), 'ok', second_angular,
                                         crossings, crossing_cost)
        return best or DrivePlan(reason='no_safe_forward_trajectory')

    def validate_maneuver(self, grid, x, y, yaw, linear, first_angular,
                          second_angular, first_remaining, second_remaining, divider=None,
                          reference_yaw=None):
        """Recheck every remaining phase against the newest local costmap."""
        if not grid.valid() or self._footprint_cost(grid, x, y, yaw) is None:
            return DrivePlan(reason='footprint_blocked_or_unknown')
        px, py, heading = x, y, yaw
        points = []
        for angular, duration in ((first_angular, first_remaining),
                                  (second_angular, second_remaining)):
            remaining = max(0.0, duration)
            while remaining > 1e-9:
                tick = min(self.dt, remaining)
                mid = heading + angular*tick/2
                px += linear*tick*math.cos(mid)
                py += linear*tick*math.sin(mid)
                heading += angular*tick
                if (reference_yaw is not None and abs(math.atan2(
                        math.sin(heading-reference_yaw), math.cos(heading-reference_yaw))) > 1.50):
                    return DrivePlan(reason='heading_limit')
                if self._footprint_cost(grid, px, py, heading) is None:
                    return DrivePlan(reason='maneuver_blocked')
                points.append((px, py))
                remaining -= tick
        angular = first_angular if first_remaining > 1e-9 else second_angular
        crossings = divider_crossings([(x, y), *points], divider)
        return DrivePlan(linear, angular, tuple(points), 'ok', second_angular,
                         crossings, crossings*self.lane_change_cost_m)


class ManeuverPlanner:
    """Follow forward arcs, with a bounded local-map-only lane-shift recovery."""

    def __init__(self, lane_change_cost_m=0.25):
        self.search = LocalDrivePlanner(lane_change_cost_m)
        self.maneuver = None
        self.recovery = None
        self.recovery_anchor = None
        self.recovery_failed = False
        self.direction_anchor = None

    def reset(self):
        self.maneuver = None
        self.recovery = None
        self.recovery_anchor = None
        self.recovery_failed = False
        self.direction_anchor = None

    @staticmethod
    def _angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    def _check_recovery(self, grid, x, y, yaw, base_yaw, side, shift,
                        phase, anchor_x, anchor_y, divider):
        """Sweep the remaining pivot, lateral advance, pivot back and exit.

        All poses are built relative to current odometry and checked against
        the current observed map. Each turn includes an overshoot reserve.
        """
        if not grid.valid() or self.search._footprint_cost(grid, x, y, yaw) is None:
            return None
        px, py, heading = x, y, yaw
        points = [(x, y)]
        soft_cost = 0.0
        unknown_cost = 0.0
        lateral = (-math.sin(base_yaw)*side, math.cos(base_yaw)*side)
        shifted = (x-anchor_x)*lateral[0]+(y-anchor_y)*lateral[1]
        remaining = max(0.0, shift-shifted)
        def turn_to(target):
            nonlocal heading, soft_cost
            angle = self._angle(target-heading)
            if abs(angle) < 1e-6:
                return True
            # Check possible braking overshoot, then retain the target angle.
            reserve = math.copysign(.08, angle)
            steps = max(1, math.ceil(abs(angle+reserve)/.04))
            for i in range(1, steps+1):
                cost = self.search._footprint_cost(grid, px, py,
                                                   heading+(angle+reserve)*i/steps)
                if cost is None:
                    return False
                # Approximate the swept distance of the outer footprint for
                # a rotation, rather than charging clearance per second.
                soft_cost += cost * abs(angle+reserve) / steps * math.hypot(
                    self.search.front, self.search.half_width)
            heading = target
            return True
        def forward(distance):
            nonlocal px, py, soft_cost, unknown_cost
            steps = max(1, math.ceil(distance/.04))
            for _ in range(steps):
                previous_unknown = cell_is_unknown(grid, px, py)
                px += distance/steps*math.cos(heading)
                py += distance/steps*math.sin(heading)
                cost = self.search._footprint_cost(grid, px, py, heading)
                if cost is None:
                    return False
                soft_cost += cost * distance / steps
                unknown_cost += unknown_traversal_cost(previous_unknown,
                                                       cell_is_unknown(grid, px, py),
                                                       distance/steps,
                                                       self.search.unknown_multiplier)
                points.append((px, py))
            return True
        target_out = self._angle(base_yaw+side*math.pi/2)
        if phase in ('turn_out', 'settle_out'):
            if not turn_to(target_out):
                return None
        if phase in ('turn_out', 'settle_out', 'advance', 'settle_advance'):
            if abs(self._angle(heading-target_out)) > .15:
                return None
            # Reserve space for the small distance travelled while braking.
            if not forward(remaining+.08):
                return None
        if not turn_to(base_yaw) or not forward(.75):
            return None
        crossings = divider_crossings(points, divider)
        return crossings, soft_cost, unknown_cost

    def _begin_recovery(self, grid, x, y, yaw, time_s, divider, reference_yaw):
        if self.recovery_failed or self.recovery_anchor is not None:
            return DrivePlan(reason='recovery_unavailable')
        # The robot may already have crossed into the other lane but be
        # facing the border. First try to return to its locally remembered
        # direction; a second lateral shift can otherwise drive it into that
        # border. The entire pivot and a forward exit must both be clear.
        delta = self._angle(reference_yaw-yaw)
        if abs(delta) >= .35:
            side = -1 if delta > 0 else 1
            checked = self._check_recovery(grid, x, y, yaw, reference_yaw, side, 0.0,
                                           'turn_back', x, y, divider)
            if checked is not None:
                crossings, _, _ = checked
                self.recovery_anchor = (x, y, reference_yaw)
                self.recovery = {'phase': 'turn_back', 'started': time_s,
                                 'yaw': reference_yaw, 'side': side, 'shift': 0.0,
                                 'x': x, 'y': y}
                return DrivePlan(0.0, -side*.25, ((x, y),), 'ok', 0.0,
                                 crossings, crossings*self.search.lane_change_cost_m)
        candidates = []
        for side in (1, -1):
            for shift in (1.6, 1.8, 2.0):
                checked = self._check_recovery(grid, x, y, yaw, yaw, side, shift,
                                               'turn_out', x, y, divider)
                if checked is not None:
                    crossings, soft_cost, unknown_cost = checked
                    candidates.append((shift + self.search.clearance_weight*soft_cost + unknown_cost
                                       + crossings*self.search.lane_change_cost_m,
                                       side, shift, crossings))
        if not candidates:
            self.recovery_failed = True
            return DrivePlan(reason='recovery_unavailable')
        _, side, shift, crossings = min(candidates)
        self.recovery_anchor = (x, y, yaw)
        self.recovery = {'phase': 'turn_out', 'started': time_s, 'yaw': yaw,
                         'side': side, 'shift': shift, 'x': x, 'y': y}
        return DrivePlan(0.0, side*.25, ((x, y),), 'ok', 0.0,
                         crossings, crossings*self.search.lane_change_cost_m)

    def _continue_recovery(self, grid, x, y, yaw, time_s, divider,
                           linear_speed_mps, angular_speed_radps, lateral_speed_mps):
        r = self.recovery
        valid_speed = (linear_speed_mps is not None and angular_speed_radps is not None
                       and lateral_speed_mps is not None and math.isfinite(linear_speed_mps)
                       and math.isfinite(angular_speed_radps) and math.isfinite(lateral_speed_mps))
        if not valid_speed or time_s < r['started'] or time_s-r['started'] > 25:
            self.recovery = None
            self.recovery_failed = True
            return DrivePlan(reason='recovery_unavailable')
        phase, side, base_yaw = r['phase'], r['side'], r['yaw']
        if ((phase in ('turn_out', 'turn_back') and abs(linear_speed_mps) > .05)
                or (phase == 'advance' and (abs(angular_speed_radps) > .15
                                            or abs(lateral_speed_mps) > .05))):
            self.recovery = None
            self.recovery_failed = True
            return DrivePlan(reason='recovery_unavailable')
        checked = self._check_recovery(grid, x, y, yaw, base_yaw, side, r['shift'],
                                       phase, r['x'], r['y'], divider)
        if checked is None:
            self.recovery = None
            self.recovery_failed = True
            return DrivePlan(reason='recovery_unavailable')
        crossings, _, _ = checked
        def change(name):
            r['phase'], r['started'] = name, time_s
        # Stop between phases so a requested pivot cannot begin while the
        # chassis is still translating or an advance while it is rotating.
        if phase == 'turn_out':
            remaining = self._angle(base_yaw+side*math.pi/2-yaw)*side
            if remaining <= .05:
                change('settle_out')
                return DrivePlan(reason='recovery_settling')
            return DrivePlan(0.0, side*.25, ((x, y),), 'ok', 0.0, crossings,
                             crossings*self.search.lane_change_cost_m)
        if phase == 'settle_out':
            if abs(linear_speed_mps) > .05 or abs(angular_speed_radps) > .05:
                return DrivePlan(reason='recovery_settling')
            change('advance')
            phase = 'advance'
        if phase == 'advance':
            lateral = (-math.sin(base_yaw)*side, math.cos(base_yaw)*side)
            shifted = (x-r['x'])*lateral[0]+(y-r['y'])*lateral[1]
            if shifted >= r['shift']-.05:
                change('settle_advance')
                return DrivePlan(reason='recovery_settling')
            return DrivePlan(.18, 0.0, ((x, y),), 'ok', 0.0, crossings,
                             crossings*self.search.lane_change_cost_m)
        if phase == 'settle_advance':
            if abs(linear_speed_mps) > .05 or abs(angular_speed_radps) > .05:
                return DrivePlan(reason='recovery_settling')
            change('turn_back')
            phase = 'turn_back'
        if phase == 'turn_back':
            remaining = self._angle(base_yaw-yaw)*(-side)
            if remaining <= .05:
                change('settle_back')
                return DrivePlan(reason='recovery_settling')
            return DrivePlan(0.0, -side*.25, ((x, y),), 'ok', 0.0, crossings,
                             crossings*self.search.lane_change_cost_m)
        if abs(linear_speed_mps) > .05 or abs(angular_speed_radps) > .05:
            return DrivePlan(reason='recovery_settling')
        self.recovery = None
        return None

    def plan(self, grid, x, y, yaw, time_s, divider=None,
             linear_speed_mps=None, angular_speed_radps=None, lateral_speed_mps=None,
             guidance_yaw=None):
        if self.direction_anchor is None:
            self.direction_anchor = (x, y, yaw)
        ax, ay, direction = self.direction_anchor
        # Advance the locally measured travel direction only after genuine
        # forward progress and modest heading change. This follows gradual
        # bends without repeatedly turning back while avoiding an obstacle.
        if (self.recovery is None and
                (x-ax)*math.cos(direction)+(y-ay)*math.sin(direction) >= 1.5 and
                abs(self._angle(yaw-direction)) <= .45):
            self.direction_anchor = (x, y, yaw)
            direction = yaw
        if self.recovery_anchor is not None and self.recovery is None:
            ax, ay, recovery_direction = self.recovery_anchor
            if (x-ax)*math.cos(recovery_direction)+(y-ay)*math.sin(recovery_direction) >= .5:
                self.recovery_anchor = None
                self.recovery_failed = False
        # A recent, observed local line supplies a corridor direction; the
        # odometry anchor remains the fallback when no classified paint is in
        # view. A semantic direction never overrides collision checks.
        if guidance_yaw is not None and math.isfinite(guidance_yaw):
            direction = guidance_yaw
        else:
            guidance_yaw = None
        if self.recovery:
            recovery = self._continue_recovery(grid, x, y, yaw, time_s, divider,
                                               linear_speed_mps, angular_speed_radps, lateral_speed_mps)
            if recovery is not None:
                return recovery
        if self.maneuver:
            started, linear, first, second, planned_crossings = self.maneuver
            elapsed = time_s-started
            if 0 <= elapsed < self.search.horizon:
                half = self.search.horizon/2
                plan = self.search.validate_maneuver(
                    grid, x, y, yaw, linear, first, second,
                    max(0., half-elapsed), min(half, self.search.horizon-elapsed), divider,
                    reference_yaw=direction)
                if plan.reason == 'ok' and plan.divider_crossings <= planned_crossings:
                    return plan
            self.maneuver = None
        plan = self.search.plan(grid, x, y, yaw, divider, reference_yaw=direction,
                                guidance_yaw=guidance_yaw)
        if plan.reason == 'ok' and abs(plan.angular) > 1e-9:
            self.maneuver = (time_s, plan.linear, plan.angular, plan.second_angular,
                             plan.divider_crossings)
        if plan.reason == 'no_safe_forward_trajectory':
            # Previous forward motion must have stopped before trying a pivot.
            if (linear_speed_mps is not None and angular_speed_radps is not None
                    and lateral_speed_mps is not None and math.isfinite(linear_speed_mps)
                    and math.isfinite(angular_speed_radps) and math.isfinite(lateral_speed_mps)
                    and abs(linear_speed_mps) <= .05 and abs(angular_speed_radps) <= .05
                    and abs(lateral_speed_mps) <= .05):
                return self._begin_recovery(grid, x, y, yaw, time_s, divider, direction)
        return plan
