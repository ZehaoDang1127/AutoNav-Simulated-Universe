"""Standalone waypoint-free closed-track simulation.

This is an authored, ground-truth prototype, not the ROS navigation stack.
The planner searches a finite forward horizon in track coordinates, balancing
clearance, divider proximity, lateral motion and lane changes.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .lane_costmap import LaneAwareCostmap


@dataclass(frozen=True)
class Track:
    width: float = 14.0
    height: float = 11.0
    outer: tuple = (0.5, 0.5, 13.5, 10.5)
    inner: tuple = (3.5, 3.5, 10.5, 7.5)
    divider: tuple = (2.0, 2.0, 12.0, 9.0)
    corner_radius: float = 1.4
    road_half_width: float = 1.5
    obstacles: tuple = ((8.0, 1.4, 0.42), (2.6, 5.0, 0.42))

    @property
    def length(self):
        return 2 * (10 - 2 * self.corner_radius) + 2 * (7 - 2 * self.corner_radius) + 2 * math.pi * self.corner_radius

    def center(self, distance, lateral=0.0):
        """CCW centerline and its left normal, starting on the bottom straight."""
        r = self.corner_radius
        segments = ((10-2*r, 'bottom'), (math.pi*r/2, 'br'),
                    (7-2*r, 'right'), (math.pi*r/2, 'tr'),
                    (10-2*r, 'top'), (math.pi*r/2, 'tl'),
                    (7-2*r, 'left'), (math.pi*r/2, 'bl'))
        d = distance % self.length
        for length, name in segments:
            if d <= length:
                break
            d -= length
        if name == 'bottom':
            x, y, yaw = 2+r+d, 2, 0.0
        elif name == 'right':
            x, y, yaw = 12, 2+r+d, math.pi/2
        elif name == 'top':
            x, y, yaw = 12-r-d, 9, math.pi
        elif name == 'left':
            x, y, yaw = 2, 9-r-d, 3*math.pi/2
        else:
            cx, cy, angle = {'br': (12-r, 2+r, -math.pi/2),
                             'tr': (12-r, 9-r, 0),
                             'tl': (2+r, 9-r, math.pi/2),
                             'bl': (2+r, 2+r, math.pi)}[name]
            angle += d/r
            x, y, yaw = cx+r*math.cos(angle), cy+r*math.sin(angle), angle+math.pi/2
        return x-lateral*math.sin(yaw), y+lateral*math.cos(yaw), yaw

    def signed_distance(self, x, y):
        r = self.corner_radius
        qx, qy = abs(x-7)-(5-r), abs(y-5.5)-(3.5-r)
        return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0) - r


class SelfDriveSimulation:
    """Kinematic point robot with a conservative circular collision footprint."""
    robot_radius = 0.30
    speed = 0.7
    sample_spacing = 0.2
    horizon = 5.0
    lateral_step = 0.15
    lateral_limit = 1.05

    def __init__(self, obstacles=None):
        self.track = Track() if obstacles is None else Track(obstacles=tuple(tuple(o) for o in obstacles))
        for obstacle in self.track.obstacles:
            if len(obstacle) != 3 or not all(math.isfinite(v) for v in obstacle) or obstacle[2] < 0:
                raise ValueError('obstacles must be finite (x, y, nonnegative radius) tuples')
        self.grid = LaneAwareCostmap(resolution_m=0.2)
        self.soft_costs = {}
        for ix in range(71):
            for iy in range(56):
                x, y = ix*.2, iy*.2
                border = self.track.road_half_width - abs(self.track.signed_distance(x, y))
                obstacle = min((math.hypot(x-ox, y-oy)-radius for ox, oy, radius in self.track.obstacles), default=100.0)
                clearance = min(border, obstacle)
                self.grid.cells[ix, iy] = 'boundary' if border <= 0 else 'obstacle' if obstacle <= 0 else 'free'
                self.grid.lane_ids[ix, iy] = 'inner' if self.track.signed_distance(x, y) < 0 else 'outer'
                if clearance > 0:
                    # The planner uses this same clearance penalty formula.
                    self.soft_costs[ix, iy] = 2.0 * max(0.0, 1.1-clearance)**2
        self.reset()

    def reset(self):
        self.progress = 0.0
        self.lateral = -0.6
        self.x, self.y, self.yaw = self.track.center(self.progress, self.lateral)
        self.trail = [(self.x, self.y)]
        self.plan = []
        self.status = 'running' if self.is_safe(self.x, self.y) else 'blocked'
        self.laps = self.lane_changes = self.collision_count = 0
        self.distance_m = self.time_s = 0.0
        self._last_lane = -1
        self._route = []
        self._route_age = 1.0

    def clearance(self, x, y):
        border = self.track.road_half_width - abs(self.track.signed_distance(x, y))
        obstacle = min((math.hypot(x-ox, y-oy)-r for ox, oy, r in self.track.obstacles), default=math.inf)
        return min(border, obstacle) - self.robot_radius

    def is_safe(self, x, y):
        return self.clearance(x, y) > 1e-6

    def _edge_safe(self, a, b):
        n = max(1, math.ceil(math.dist(a, b)/0.05))
        return all(self.is_safe(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(n+1))

    def _search(self):
        # Search only increasing progress; no waypoint or externally supplied goal.
        # Lateral states are offsets from the broken line. Negative/positive
        # offsets represent the two lanes. The full horizon must remain safe.
        offsets = [i*self.lateral_step for i in range(-7, 8)]
        states = {(round(self.lateral, 8), self._last_lane): (0.0, [])}
        count = round(self.horizon/self.sample_spacing)
        for i in range(1, count+1):
            s0 = self.progress+(i-1)*self.sample_spacing
            s1 = self.progress+i*self.sample_spacing
            next_states = {}
            for (previous, lane), (cost, path) in states.items():
                ax, ay, _ = self.track.center(s0, previous)
                for offset in offsets:
                    if abs(offset-previous) > self.lateral_step+1e-6:
                        continue
                    x, y, _ = self.track.center(s1, offset)
                    if not self._edge_safe((ax, ay), (x, y)):
                        continue
                    clearance = self.clearance(x, y)
                    near = max(0.0, 0.8-clearance)**2 * 2.0
                    divider = max(0.0, 0.5-abs(offset))**2 * 1.5
                    next_lane = 1 if offset > 1e-8 else -1 if offset < -1e-8 else lane
                    change = 0.25 if lane != next_lane else 0.0
                    lateral_cost = 0.18*abs(offset-previous)/self.lateral_step
                    total = cost + self.sample_spacing*(1+near+divider) + lateral_cost + change
                    key = (offset, next_lane)
                    if key not in next_states or total < next_states[key][0]:
                        next_states[key] = (total, path+[(s1, offset, x, y)])
            states = next_states
            if not states:
                return []
        return min(states.values(), key=lambda item: item[0])[1]

    def step(self, dt=0.1):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        if self.status != 'running':
            return
        # Bound integration steps so collision checks and replanning cannot be skipped.
        remaining = dt
        while remaining > 1e-9 and self.status == 'running':
            tick = min(remaining, 0.1)
            remaining -= tick
            if self._route_age >= 0.4 or not self._route:
                self._route = self._search()
                self._route_age = 0.0
                self.plan = [(self.x, self.y)] + [(p[2], p[3]) for p in self._route]
                if not self._route:
                    self.status = 'blocked'
                    break
            target_s = self.progress + self.speed*tick
            first = self._route[0]
            fraction = min(1.0, (target_s-self.progress)/(first[0]-self.progress))
            offset = self.lateral + fraction*(first[1]-self.lateral)
            x, y, tangent = self.track.center(target_s, offset)
            if not self._edge_safe((self.x, self.y), (x, y)):
                self.status = 'blocked'
                break
            distance = math.hypot(x-self.x, y-self.y)
            if distance > 1e-9:
                self.yaw = math.atan2(y-self.y, x-self.x)
            self.x, self.y, self.lateral, self.progress = x, y, offset, target_s
            self.distance_m += distance
            self.time_s += tick
            self.laps = int(self.progress/self.track.length)
            lane = 1 if offset > 0.08 else -1 if offset < -0.08 else self._last_lane
            if lane != self._last_lane:
                self.lane_changes += 1
                self._last_lane = lane
            self.trail.append((x, y))
            self._route_age += tick
            while self._route and self._route[0][0] <= self.progress+1e-9:
                self._route.pop(0)
