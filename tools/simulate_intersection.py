#!/usr/bin/env python3
"""Offline closed-loop check of junction recognition on the intersection track.

Evaluator-side harness, not the ROS stack: it renders the local grids the
self-drive controller reads (combined costmap, classified paint and
classified hard tape) from the authored track with a simple field-of-view
sensor model and memory, then runs the controller's own guidance, junction
manager and lane reference planner. A pure-pursuit tracker with first-order
drive lag stands in for Nav2's MPPI. Track geometry is used only to render
observations and to score the run; it never reaches the decision code.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))

from autonav_universe.corridor_tangent import estimate_corridor_tangent  # noqa: E402
from autonav_universe.guidance_cache import GuidanceCache  # noqa: E402
from autonav_universe.intersection import (GUIDANCE_MAX_DEVIATION_RAD,  # noqa: E402
                                           IntersectionManager, trim_to_stop)
from autonav_universe.lane_reference import LaneReferencePlanner, _Field  # noqa: E402
from autonav_universe.local_drive import LocalGrid  # noqa: E402
from autonav_universe.self_drive_divider import rasterize_observations  # noqa: E402

_spec = importlib.util.spec_from_file_location('generate_self_drive_track',
                                               ROOT/'tools/generate_self_drive_track.py')
track_tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(track_tool)

LIDAR_X = .6598
BODY = (-.32, .77, .41)            # rear, front, half width about base_link


def _segments(items):
    return [((i['start']['x_m'], i['start']['y_m']), (i['end']['x_m'], i['end']['y_m']), i['width_m'])
            for i in items]


def _point_segment(px, py, a, b):
    ax, ay = a
    dx, dy = b[0]-ax, b[1]-ay
    length2 = dx*dx+dy*dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/length2))
    return math.hypot(px-ax-t*dx, py-ay-t*dy)


class World:
    """Ground-truth tape rasterized once into a world cell index."""

    def __init__(self, resolution=.1, layout='intersection'):
        if layout in ('intersection', 'closed_branch'):
            self.course, self.track = track_tool.intersection_course_data()
            if layout == 'closed_branch':
                # Negative test: tape closes the branch mouth (no left road).
                x0, x1 = track_tool.BRANCH_X0, track_tool.BRANCH_X1
                self.course = dict(self.course, internal_lines=self.course['internal_lines']+[
                    track_tool._segment(f'closure_{k}', (x0+k*.8, track_tool.ROAD_Y1),
                                        (min(x1, x0+(k+1)*.8), track_tool.ROAD_Y1)) for k in range(6)])
        else:
            self.course = track_tool.course_data()[0]
            self.track = {'exits': {}}
        self.res = resolution
        self.hard = _segments(self.course['internal_lines'])
        self.paint = _segments(self.course['dashed_lines'])
        self.stop_lines = [s for i, s in zip(self.course['dashed_lines'], self.paint)
                           if i['name'].startswith('stop_line_')]
        self.hard_cells = self._rasterize(self.hard)
        self.obstacle_cells = set()
        r = resolution
        for item in self.course['obstacles']:
            ox, oy, radius = item['x_m'], item['y_m'], item['radius_m']
            for ix in range(math.floor((ox-radius)/r), math.floor((ox+radius)/r)+1):
                for iy in range(math.floor((oy-radius)/r), math.floor((oy+radius)/r)+1):
                    if math.hypot((ix+.5)*r-ox, (iy+.5)*r-oy) <= radius:
                        self.obstacle_cells.add((ix, iy))
        self.paint_cells = self._rasterize(self.paint) - self.hard_cells

    def _rasterize(self, segments):
        cells = set()
        r = self.res
        for a, b, width in segments:
            reach = width/2+r/2
            x0, x1 = min(a[0], b[0])-reach, max(a[0], b[0])+reach
            y0, y1 = min(a[1], b[1])-reach, max(a[1], b[1])+reach
            for ix in range(math.floor(x0/r), math.floor(x1/r)+1):
                for iy in range(math.floor(y0/r), math.floor(y1/r)+1):
                    if _point_segment((ix+.5)*r, (iy+.5)*r, a, b) <= reach:
                        cells.add((ix, iy))
        return cells


class Sensor:
    """Rolling local maps with field-of-view observation memory."""

    def __init__(self, world, size_m=8.0, fov_rad=math.radians(70), range_m=6.0, paint_ttl_s=4.0,
                 line_fov_rad=math.radians(54), line_range_m=4.5, bridge_paint=True):
        """Free space: planar scan (fov_rad, range_m). Tape and paint: the LiDAR
        line detector's narrower clearing cone (line_fov_rad, line_range_m)."""
        self.world, self.size_m, self.fov, self.range = world, size_m, fov_rad, range_m
        self.line_fov, self.line_range = line_fov_rad, line_range_m
        self.bridge_paint = bridge_paint
        self.paint_ttl = paint_ttl_s
        self.seen = set()          # observed world cells (free-space clearing)
        self.tape = set()          # observed hard tape cells (persisted)
        self.obstacles = set()     # observed obstacle cells (persisted)
        self.paint_seen = {}       # crossable paint cell -> last seen time

    def observe(self, x, y, yaw, now):
        r = self.world.res
        lx, ly = x+LIDAR_X*math.cos(yaw), y+LIDAR_X*math.sin(yaw)
        reach = self.range
        for ix in range(math.floor((lx-reach)/r), math.floor((lx+reach)/r)+1):
            for iy in range(math.floor((ly-reach)/r), math.floor((ly+reach)/r)+1):
                cx, cy = (ix+.5)*r-lx, (iy+.5)*r-ly
                if cx*cx+cy*cy > reach*reach:
                    continue
                bearing = abs((math.atan2(cy, cx)-yaw+math.pi) % (2*math.pi)-math.pi)
                if bearing > self.fov:
                    continue
                cell = (ix, iy)
                self.seen.add(cell)
                if bearing > self.line_fov or cx*cx+cy*cy > self.line_range**2:
                    continue
                if cell in self.world.hard_cells:
                    self.tape.add(cell)
                elif cell in self.world.obstacle_cells:
                    self.obstacles.add(cell)
                elif cell in self.world.paint_cells:
                    self.paint_seen[cell] = now

    def grids(self, x, y, now):
        r, n = self.world.res, round(self.size_m/self.world.res)
        ox = (math.floor(x/r)-n//2)*r
        oy = (math.floor(y/r)-n//2)*r
        bx, by = math.floor(ox/r+.5), math.floor(oy/r+.5)
        lethal, tape, paint, seen = [], [], [], []
        for iy in range(n):
            for ix in range(n):
                cell = (bx+ix, by+iy)
                tape.append(100 if cell in self.tape else 0)
                lethal.append(100 if cell in self.tape or cell in self.obstacles else 0)
                paint.append(100 if now-self.paint_seen.get(cell, -math.inf) <= self.paint_ttl else 0)
                seen.append(cell in self.seen)
        if self.bridge_paint:
            # The ROS divider node rasterizes LiDAR hits and joins hits up to
            # 0.8 m apart; sample hits sparsely and rasterize the same way.
            hits = [((bx+i % n+.5)*r, (by+i//n+.5)*r) for i, v in enumerate(paint)
                    if v and (i % n + i//n) % 3 == 0]
            paint = list(rasterize_observations(hits, n, n, r, (ox, oy, 0.0)))
        boundary = LocalGrid(n, n, r, ox, oy, 0.0, tuple(tape))
        distance = _Field(LocalGrid(n, n, r, ox, oy, 0.0, tuple(lethal))).distance
        cells = []
        for i in range(n*n):
            if lethal[i]:
                cells.append(100)
            elif not seen[i]:
                cells.append(-1)
            else:
                # Inflation-like soft cost (nav2 1.15 m radius, scaling 3.2).
                d = distance[i]
                cells.append(0 if d > 1.15 else max(1, min(98, round(98*math.exp(-3.2*max(0.0, d-.5))))))
        return (LocalGrid(n, n, r, ox, oy, 0.0, tuple(cells)), boundary,
                LocalGrid(n, n, r, ox, oy, 0.0, tuple(paint)))


def body_hits_tape(world, x, y, yaw):
    rear, front, half = BODY
    c, s = math.cos(yaw), math.sin(yaw)
    cx, cy = x+c*(rear+front)/2, y+s*(rear+front)/2
    hx, hy = (front-rear)/2, half
    for a, b, width in world.hard:
        # Sample the segment; tape is thin, 2 cm sampling suffices.
        length = math.dist(a, b)
        for k in range(max(1, math.ceil(length/.02))+1):
            t = k/max(1, math.ceil(length/.02))
            px, py = a[0]+(b[0]-a[0])*t-cx, a[1]+(b[1]-a[1])*t-cy
            bx, by = c*px+s*py, -s*px+c*py
            if abs(bx) <= hx+width/2 and abs(by) <= hy+width/2:
                return True
    return False


def run(turn='straight', resolution=.1, max_time_s=110.0, stop_hold_s=2.0, verbose=False,
        layout='intersection', start_offset=(0.0, 0.0, 0.0), schedule=(), yaw_rate_bias=0.0,
        centering_weight=None):
    """``schedule``: (time_s, command) turn requests sent during the run."""
    world = World(resolution, layout)
    sensor = Sensor(world)
    planner = (LaneReferencePlanner(.25) if centering_weight is None
               else LaneReferencePlanner(.25, centering_weight))
    manager = IntersectionManager(planner, stop_hold_s=stop_hold_s)
    if turn != 'straight':
        manager.set_request(turn, 'test')
    cache = GuidanceCache(.75)
    start = world.course['start']
    x, y, yaw = (start['x_m']+start_offset[0], start['y_m']+start_offset[1],
                 start['yaw_rad']+start_offset[2])
    pending = sorted(schedule)
    v = w = 0.0
    travel_yaw = None
    path, previous = (), None
    dt, plan_every = .05, 4
    trail, states = [], []
    openings_seen = 0
    tape_hits = 0
    stopped_at = None
    plan_ms = []
    exits = world.track['exits']
    result = {'turn_request': turn, 'exit': None}
    step = 0
    while step*dt < max_time_s:
        now = step*dt
        sensor.observe(x, y, yaw, now)
        while pending and pending[0][0] <= now:
            manager.set_request(pending.pop(0)[1], 'schedule')
        if step % plan_every == 0:
            costmap, boundary, divider = sensor.grids(x, y, now)
            fallback = travel_yaw if travel_yaw is not None else yaw
            sources = {'divider': True, 'boundary': True}
            expected = manager.expected_yaw(x, y)
            reference_yaw = (expected if expected is not None
                             else cache.reference(fallback, 'odom', now, sources))
            estimate = estimate_corridor_tangent(divider, x, y, reference_yaw, boundary,
                                                 max_deviation_rad=GUIDANCE_MAX_DEVIATION_RAD)
            if estimate is not None:
                cache.update(estimate, 'odom', now)
            guidance = cache.get('odom', now, sources)
            directive = manager.update(now, costmap, divider, x, y, yaw, v,
                                       guidance.yaw_rad if guidance else None, boundary)
            if directive.new_travel_yaw is not None:
                travel_yaw = directive.new_travel_yaw
                cache.clear()
                previous = None
            if manager.last_obs is not None and manager.last_obs.is_junction:
                openings_seen += 1
            if not states or states[-1][1] != manager.state:
                states.append((round(now, 2), manager.state, directive.reason))
            started = time.perf_counter()
            if directive.mode == 'hold':
                path = ()
            elif directive.mode == 'follow':
                path = directive.poses
            else:
                g = directive.guidance_yaw if directive.mode in ('locked', 'stop_at') else (
                    guidance.yaw_rad if guidance else None)
                if g is None:
                    path = ()
                else:
                    if directive.mode == 'normal':
                        travel_yaw = g
                    reference = planner.plan(costmap, x, y, yaw, g, divider, previous)
                    path = reference.poses if reference else ()
                    if directive.mode == 'stop_at' and path:
                        path = trim_to_stop(path, directive.stop_point, manager.theta)
                        if len(path) < 2:
                            path = ()
                    previous = path or None
            plan_ms.append(1000*(time.perf_counter()-started))
            if verbose and step % 20 == 0:
                print(f't={now:5.1f} x={x:6.2f} y={y:5.2f} yaw={math.degrees(yaw):6.1f} v={v:.2f} '
                      f'{manager.state}/{directive.reason} path={len(path)}', flush=True)
        # Pure pursuit on the current reference (stand-in for MPPI).
        target_v, target_w = 0.0, 0.0
        if len(path) >= 2:
            nearest = min(range(len(path)), key=lambda i: (path[i][0]-x)**2+(path[i][1]-y)**2)
            remaining = sum(math.dist(path[i][:2], path[i+1][:2]) for i in range(nearest, len(path)-1))
            look = nearest
            while look < len(path)-1 and math.dist(path[look][:2], (x, y)) < .6:
                look += 1
            tx, ty = path[look][:2]
            alpha = (math.atan2(ty-y, tx-x)-yaw+math.pi) % (2*math.pi)-math.pi
            distance = max(.05, math.dist((tx, ty), (x, y)))
            target_v = min(.5, math.sqrt(max(0.0, 2*.5*remaining)))
            if abs(alpha) > 1.2:
                target_v = 0.0
            target_w = max(-1.0, min(1.0, 2*max(target_v, .15)*math.sin(alpha)/distance))
            if remaining < .03:
                target_v = target_w = 0.0
        v += (target_v-v)*min(1.0, dt/.2)
        # yaw_rate_bias stands in for MPPI's small persistent tracking error.
        w += (target_w+(yaw_rate_bias if target_v > 0 else 0.0)-w)*min(1.0, dt/.18)
        x += v*math.cos(yaw)*dt
        y += v*math.sin(yaw)*dt
        yaw = (yaw+w*dt+math.pi) % (2*math.pi)-math.pi
        trail.append((round(x, 3), round(y, 3)))
        if body_hits_tape(world, x, y, yaw) if step % 2 == 0 else False:
            tape_hits += 1
        if manager.state == 'stopped' and stopped_at is None and abs(v) < .02:
            front = x+BODY[1]*math.cos(yaw)
            stopped_at = {'time_s': round(now, 2), 'x': round(x, 3), 'y': round(y, 3),
                          'front_x': round(front, 3)}
        for name, zone in exits.items():
            if math.hypot(x-zone['x_m'], y-zone['y_m']) <= zone['radius_m']:
                result['exit'] = name
        if result['exit']:
            break
        step += 1
    stop_line = next((s for s in world.stop_lines if s[0][0] < 12.0), ((None, None),))
    stopped_s = 0.0
    if states:
        times = [t for t, state, _ in states]
        for i, (t, state, _) in enumerate(states):
            if state == 'stopped':
                stopped_s = (times[i+1] if i+1 < len(times) else step*dt)-t
    result.update({
        'time_s': round(step*dt, 2), 'final_pose': [round(x, 3), round(y, 3), round(yaw, 3)],
        'tape_contacts': tape_hits, 'states': states, 'stopped': stopped_at,
        'stop_line_x': stop_line[0][0], 'stopped_state_s': round(stopped_s, 2),
        'route': manager.last_route or manager.route, 'junction_observations': openings_seen,
        'max_plan_ms': round(max(plan_ms), 1) if plan_ms else None,
        'mean_plan_ms': round(sum(plan_ms)/len(plan_ms), 1) if plan_ms else None,
        'trail': trail[::10]})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--turn', choices=('straight', 'left'), default='straight')
    parser.add_argument('--layout', choices=('intersection', 'loop', 'closed_branch'), default='intersection')
    parser.add_argument('--resolution', type=float, default=.1)
    parser.add_argument('--max-time', type=float, default=110.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()
    result = run(args.turn, args.resolution, args.max_time, verbose=args.verbose, layout=args.layout)
    summary = {k: v for k, v in result.items() if k != 'trail'}
    print(json.dumps(summary, indent=2))
    if args.output:
        args.output.write_text(json.dumps(result), encoding='utf-8')


if __name__ == '__main__':
    main()
