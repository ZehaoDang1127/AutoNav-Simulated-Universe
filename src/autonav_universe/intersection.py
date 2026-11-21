"""Recognise a road junction in the observed local cost maps and pick a route.

Self-drive has no map, route or waypoints, so a junction is recognised from
the live grids that the lane reference planner already reads. Every grid
must share one frame (the rolling local costmap's ``odom`` frame):

* ``costmap``: combined local costmap. 100 is observed hard tape or an
  obstacle, 0..99 is observed and non-lethal, -1 is unobserved.
* ``divider``: classified crossable paint (divider dashes and stop lines).

The pattern is measured in a road frame at base_link: ``u`` along the
observed road direction, ``v`` to its left.

* Stop line: crossable paint spanning the robot's own lane, across the road.
* Side opening: where a road edge should continue (one road width from the
  opposite edge), a run of observed, non-lethal cells at least 1.6 m long.
* Straight on: no hard line across both lanes beyond the robot.

:class:`IntersectionManager` turns repeated observations into one directive
per controller tick: drive normally, slow to a stop at the stop line, hold
there briefly, then cross straight with the road direction locked or follow
a left-turn path into the branch's right-hand lane. The route is chosen only
at a stop line; an opening seen without one is reported but not acted on. The turn request comes
from outside ('straight' or 'left'); without one the robot goes straight.
No course geometry, exits or simulated truth enter this module.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

TURN_COMMANDS = ('straight', 'left')
DEFAULT_TURN = 'straight'
DEFAULT_ROAD_WIDTH_M = 4.8
MIN_OPENING_M = 1.6
# Lane-direction fits further than this from the current travel direction
# are rejected (passed to estimate_corridor_tangent by the controller).
GUIDANCE_MAX_DEVIATION_RAD = .6


def parse_turn_command(value):
    """Return 'straight' or 'left', or None for anything else."""
    if not isinstance(value, str):
        return None
    value = value.strip().lower()
    return value if value in TURN_COMMANDS else None


def _wrap(angle):
    return (angle+math.pi) % (2*math.pi) - math.pi


class _Probe:
    """Cell lookup in a LocalGrid; None off the map."""

    def __init__(self, grid):
        self.grid = grid
        self.c, self.s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
        self.inv = 1.0/grid.resolution

    def __call__(self, x, y):
        g = self.grid
        dx, dy = x-g.origin_x, y-g.origin_y
        ix = math.floor((self.c*dx+self.s*dy)*self.inv)
        iy = math.floor((-self.s*dx+self.c*dy)*self.inv)
        if 0 <= ix < g.width and 0 <= iy < g.height:
            return g.cells[iy*g.width+ix]
        return None


@dataclass(frozen=True)
class Opening:
    """A gap in one road edge, in metres along the road from base_link."""
    start_m: float
    end_m: float | None          # far edge, when it has been observed
    visible_end_m: float


@dataclass(frozen=True)
class JunctionObservation:
    road_yaw: float
    right_edge_m: float | None   # lateral offset of the right road edge (< 0)
    left_edge_m: float | None    # lateral offset of the left road edge (> 0)
    road_width_m: float
    left: Opening | None
    right: Opening | None
    straight_open: bool
    stop_line_m: float | None    # along-road distance to own-lane stop line
    # Observed tape where the far edge would continue past ``branch_from_m``:
    # True = the edge is seen to continue (no branch), None = not visible.
    left_blocked: bool | None = None

    @property
    def is_junction(self):
        return self.left is not None or self.right is not None


def observe_junction(costmap, divider, x, y, road_yaw, road_width_m=DEFAULT_ROAD_WIDTH_M,
                     *, boundary=None, max_ahead_m=6.0, stop_range_m=4.5, branch_from_m=None):
    """Measure the junction pattern around base_link (x, y).

    ``boundary`` is the optional classified hard-tape grid. When given, tape
    is read from it, so an obstacle beside the lane cannot pose as a road
    edge; the combined costmap still decides observed versus unknown.
    """
    if costmap is None or not costmap.valid() or not all(
            math.isfinite(v) for v in (x, y, road_yaw, road_width_m)):
        raise ValueError('invalid junction observation inputs')
    probe = _Probe(costmap)
    tape = _Probe(boundary) if boundary is not None and boundary.valid() else None
    tc, ts = math.cos(road_yaw), math.sin(road_yaw)

    def raw_at(u, v, look=probe):
        return look(x+u*tc-v*ts, y+u*ts+v*tc)

    if tape is None:
        at = raw_at
    else:
        def at(u, v, look=None):
            if look is not None:
                return raw_at(u, v, look)
            value = raw_at(u, v)
            if value is None or value < 0:
                return value
            paint = raw_at(u, v, tape)
            return 100 if paint is not None and paint >= 100 else min(value, 99)

    step = max(costmap.resolution, .025)

    def edge(u, side):
        v = .3
        while v <= 5.0:
            value = at(u, side*v)
            if value is None or value < 0:
                return None
            if value >= 100:
                return side*v
            v += step
        return None

    def median(values):
        values = sorted(v for v in values if v is not None)
        if len(values) < 2:
            return None
        mid = len(values)//2
        return values[mid] if len(values) % 2 else (values[mid-1]+values[mid])/2

    stations_u = (-.4, 0.0, .4, .8)
    right = median(edge(u, -1) for u in stations_u)
    left = median(edge(u, 1) for u in stations_u)
    width = road_width_m
    if right is not None and left is not None and 3.0 <= left-right <= 7.0:
        width = left-right

    # Side openings. Raw station status at 0.1 m, then paint is dilated by
    # 0.2 m so sparse LiDAR tape returns cannot read as a gap.
    dilation = 2
    station_step = .1
    count = int(round((max_ahead_m+.6)/station_step))+1
    us = [-.6+i*station_step for i in range(count)]

    def band(base, side):
        """Dilated status per station of a band straddling an expected edge."""
        offsets = [base+side*k*step for k in range(-int(round(.35/step)), int(round(.25/step))+1)]
        raw = []
        for u in us:
            status = 'open'
            for v in offsets:
                value = at(u, v)
                if value is None or value < 0:
                    status = 'unknown'
                elif value >= 100:
                    status = 'paint'
                    break
            raw.append(status)
        return ['paint' if any(raw[j] == 'paint' for j in range(max(0, i-dilation),
                                                                 min(count, i+dilation+1)))
                else raw[i] for i in range(count)]

    def continues(base, side, u0, u1):
        """The opposite edge runs on as one straight line past the opening.

        A road curving away also leaves its inner edge empty, but then the
        outer edge bends too. Fit the edge offsets from behind the robot to
        the end of the opening and require a straight, well-covered line.
        """
        if base is None:
            return False
        samples = []
        stations = [u for u in us if -.4 <= u <= u1]
        for u in stations:
            # Nearest observed tape to the expected edge, within +/-0.45 m.
            best = None
            for k in range(int(round(.45/step))+1):
                for v in ((base+k*step,) if k == 0 else (base+k*step, base-k*step)):
                    value = at(u, v)
                    if value is not None and value >= 100:
                        best = v
                        break
                if best is not None:
                    break
            if best is not None:
                samples.append((u, best))
        covered = [u for u, _ in samples if u0 <= u <= u1]
        inside = [u for u in stations if u0 <= u <= u1]
        if len(samples) < 5 or not inside or len(covered) < .6*len(inside):
            return False
        n = len(samples)
        mu = sum(u for u, _ in samples)/n
        mv = sum(v for _, v in samples)/n
        suu = sum((u-mu)**2 for u, _ in samples)
        slope = sum((u-mu)*(v-mv) for u, v in samples)/suu if suu > 1e-9 else 0.0
        residuals = [v-mv-slope*(u-mu) for u, v in samples]
        return abs(slope) <= .1 and max(abs(r) for r in residuals) <= .12

    def scan(side):
        if side > 0:
            base, opposite = (right+width if right is not None else left), right
        else:
            base, opposite = (left-width if left is not None else right), left
        if base is None:
            return None
        status = band(base, side)
        found = None
        run_start = last_open = None
        gap = 0
        for i, u in enumerate(us+[math.inf]):
            st = status[i] if i < count else 'end'
            if st == 'open':
                if run_start is None:
                    run_start = u
                last_open, gap = u, 0
                continue
            if run_start is None:
                continue
            if st == 'unknown' and gap < 2:
                gap += 1
                continue
            if last_open-run_start+station_step >= MIN_OPENING_M:
                # Undo the paint dilation at observed edges.
                start = run_start-dilation*station_step if run_start > us[0]+1e-9 else run_start
                end = u+(dilation-1)*station_step if st == 'paint' else None
                found = Opening(round(start, 3), None if end is None else round(end, 3),
                                round(last_open, 3))
                break
            run_start, gap = None, 0
        if found is None:
            return None
        # A road curving away also leaves its inner edge empty; a junction
        # keeps the opposite edge straight across the gap.
        span_end = min(found.visible_end_m, found.start_m+2.0)
        if not continues(opposite, -side, max(found.start_m, -.4), span_end):
            return None
        return found

    left_opening = scan(1)
    right_opening = scan(-1)

    # Refutation, not confirmation: the far edge is often outside the LiDAR
    # line detector's reach, so absent tape proves little, but tape seen
    # where a left branch would start shows that there is no branch.
    left_blocked = None
    base = right+width if right is not None else left
    if branch_from_m is not None and base is not None:
        status = band(base, 1)
        span = [status[i] for i, u in enumerate(us) if branch_from_m <= u <= branch_from_m+2.0]
        seen = [st for st in span if st != 'unknown']
        if len(seen) >= 10:
            left_blocked = sum(st == 'paint' for st in seen) >= .6*len(seen)

    # A hard line across both lanes ahead ends the road (no straight route).
    lane_right = right if right is not None else -width/4
    across = [lane_right+.3+k*step for k in range(int((width-.6)/step)+1)]
    straight_open = True
    for i in range(int(round(1.0/station_step)), int(round(max_ahead_m/station_step))+1):
        u = i*station_step
        values = [at(u, v) for v in across]
        lethal = sum(1 for v in values if v is not None and v >= 100)
        if lethal >= .6*len(values):
            straight_open = False
            break

    stop_line = None
    if divider is not None and divider.valid():
        paint = _Probe(divider)
        lo, hi = lane_right+.25, lane_right+width/2-.3
        if hi-lo < .6:
            lo, hi = -.9, .9
        dv = max(divider.resolution, .025)
        lane = [lo+k*dv for k in range(int((hi-lo)/dv)+1)]
        du = max(divider.resolution/2, .02)
        for i in range(int(stop_range_m/du)+1):
            u = i*du
            hits = sum(1 for v in lane if (at(u, v, paint) or 0) > 0)
            if hits >= .5*len(lane):
                stop_line = round(u, 3)
                break

    return JunctionObservation(road_yaw, right, left, width, left_opening, right_opening,
                               straight_open, stop_line, left_blocked)


@dataclass(frozen=True)
class Directive:
    """What the controller should do this tick.

    mode: 'normal' (plan with observed guidance), 'stop_at' (plan, then trim
    the path at ``stop_point``), 'hold' (zero velocity), 'locked' (plan with
    ``guidance_yaw`` instead of the observed direction), 'follow' (send
    ``poses`` as the reference path).
    """
    mode: str
    reason: str
    guidance_yaw: float | None = None
    stop_point: tuple[float, float] | None = None
    poses: tuple[tuple[float, float, float], ...] = ()
    # Set once when a crossing ends: the new road direction to follow.
    new_travel_yaw: float | None = None


def trim_to_stop(poses, stop_point, yaw):
    """Keep path poses that do not pass ``stop_point`` along ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    kept = []
    for pose in poses:
        if (pose[0]-stop_point[0])*c+(pose[1]-stop_point[1])*s > 1e-9:
            break
        kept.append(pose)
    return tuple(kept)


def left_turn_poses(x, y, theta, target_u, radius, exit_m=3.0, spacing=.05):
    """Straight lead-in, quarter arc of ``radius``, then straight exit.

    The arc ends on the line u = ``target_u`` (road frame at x, y, theta),
    heading theta + pi/2. Returns (poses, index of the arc's last pose).
    """
    lead = target_u-radius
    if lead < -1e-9 or radius <= 0:
        raise ValueError('turn does not fit ahead of the robot')
    tc, ts = math.cos(theta), math.sin(theta)
    n = max(1, round(lead/spacing)) if lead > 1e-9 else 0
    road = [(lead*k/n, 0.0, 0.0) for k in range(n+1)] if n else [(0.0, 0.0, 0.0)]
    arc_n = max(2, round(radius*math.pi/2/spacing))
    for k in range(1, arc_n+1):
        phi = (math.pi/2)*k/arc_n
        road.append((lead+radius*math.sin(phi), radius-radius*math.cos(phi), phi))
    arc_end = len(road)-1
    exit_n = max(1, round(exit_m/spacing))
    for k in range(1, exit_n+1):
        road.append((target_u, radius+exit_m*k/exit_n, math.pi/2))
    poses = tuple((x+u*tc-v*ts, y+u*ts+v*tc, _wrap(theta+h)) for u, v, h in road)
    return poses, arc_end


class IntersectionManager:
    """Stop at the stop line, then take the requested route (default straight)."""
    front_m = .87                  # padded nose from base_link (reference planner)
    stop_margin_m = .25            # padded nose to the stop line when stopped
    stop_confirm = 2               # consecutive stop-line observations
    stop_range_m = 4.5
    arrive_tolerance_m = .10
    stopped_speed_mps = .05
    history = 5
    cooldown_m = 3.0               # no new junction until this far past one
    # The junction box is assumed to begin this far past the stop line when
    # no gap in the far edge has been observed (0.6 m on the generated course).
    junction_setback_m = .6
    blocked_fallback_s = 5.0       # left turn blocked this long: go straight
    turn_radii = (3.0, 2.6, 3.4, 2.2, 1.8)
    turn_exit_m = 4.0
    turn_check_m = 3.0             # re-validate this much of the turn ahead
    exit_heading_tolerance = .35
    # Observed road direction vs. committed exit, for this many ticks in a
    # row. 0.3 rad once handed back on a 16 degree stop-line fit (Gazebo).
    guidance_agreement = .12
    guidance_agreement_ticks = 3
    settle_limit_m = 4.0           # then hand back even without agreement

    def __init__(self, planner=None, stop_hold_s=2.0):
        if planner is None:
            from .lane_reference import LaneReferencePlanner
            planner = LaneReferencePlanner()
        if not math.isfinite(stop_hold_s) or stop_hold_s < 0:
            raise ValueError('stop hold must be finite and nonnegative')
        self.planner = planner
        self.stop_hold_s = float(stop_hold_s)
        self.request = DEFAULT_TURN
        self.request_source = 'default'
        self.road_width = DEFAULT_ROAD_WIDTH_M
        self.junctions_passed = 0
        self.last_route = None
        self.reset()

    # -- external input ---------------------------------------------------
    def set_request(self, value, source='topic'):
        """Standing request for every junction until changed. Returns success."""
        command = parse_turn_command(value)
        if command is None:
            return False
        self.request, self.request_source = command, source
        return True

    def reset(self):
        """Forget the current junction (e.g. after pause/stop); keep the request."""
        self.state = 'cruise'
        self.reason = 'cruise'
        self.theta = None
        self.stop_line = None
        self.stop_since = None
        self.stop_count = 0
        self.recent = deque(maxlen=self.history)
        self.route = None
        self.route_note = None
        self.turn = ()
        self.arc_end = 0
        self.straight = ()
        self.agree_ticks = 0
        self.cross_origin = None
        self.cross_length = 0.0
        self.cooldown_from = None
        self.last_obs = None
        self.boundary = None
        self.blocked_since = None

    # -- helpers ----------------------------------------------------------
    def _observe(self, costmap, divider, x, y, yaw):
        line = None
        if self.stop_line is not None:
            line = (self.stop_line[0]-x)*math.cos(yaw)+(self.stop_line[1]-y)*math.sin(yaw)
        obs = observe_junction(costmap, divider, x, y, yaw, self.road_width,
                               boundary=self.boundary, stop_range_m=self.stop_range_m,
                               branch_from_m=None if line is None else line+self.junction_setback_m)
        self.last_obs = obs
        tc, ts = math.cos(yaw), math.sin(yaw)
        left = None
        # A gap that starts before the stop line is a missed detection of the
        # far edge, not the junction.
        if obs.left is not None and (line is None or obs.left.start_m >= line-.2):
            width = (obs.left.end_m-obs.left.start_m) if obs.left.end_m is not None else None
            left = ((x+obs.left.start_m*tc, y+obs.left.start_m*ts), width)
        self.recent.append({'left': left, 'right': obs.right is not None,
                            'straight': obs.straight_open, 'left_blocked': obs.left_blocked})
        return obs

    def _left_opening(self):
        """Latest observed left opening (world start point, width or None)."""
        seen = [entry['left'] for entry in self.recent if entry['left'] is not None]
        if len(seen) < 2:
            return None
        start = seen[-1][0]
        widths = [w for _, w in seen if w is not None]
        return start, (max(widths) if widths else None)

    def options(self):
        """straight: bool. left: 'confirmed' (gap seen), 'blocked' (far edge
        seen continuing) or 'unknown' (far edge not visible)."""
        entries = list(self.recent)
        straight = (not entries or
                    sum(1 for e in entries if e['straight']) >= math.ceil(len(entries)/2))
        if self._left_opening() is not None:
            left = 'confirmed'
        elif sum(1 for e in entries if e.get('left_blocked')) >= 2:
            left = 'blocked'
        else:
            left = 'unknown'
        return {'straight': straight, 'left': left}

    def _hold(self, reason):
        self.reason = reason
        return Directive('hold', reason)

    def _along(self, x, y, point):
        return (point[0]-x)*math.cos(self.theta)+(point[1]-y)*math.sin(self.theta)

    # -- update -----------------------------------------------------------
    def update(self, now_s, costmap, divider, x, y, heading, speed_mps, road_yaw, boundary=None):
        """Return this tick's Directive. ``road_yaw`` is the observed direction or None.

        ``boundary`` is the optional classified hard-tape grid (same frame).
        """
        self.boundary = boundary
        if not all(math.isfinite(v) for v in (now_s, x, y, heading, speed_mps)):
            raise ValueError('invalid intersection manager inputs')
        if road_yaw is not None and not math.isfinite(road_yaw):
            road_yaw = None
        handler = {'cruise': self._cruise, 'approach': self._approach,
                   'stopped': self._stopped, 'crossing_straight': self._crossing_straight,
                   'crossing_left': self._crossing_left}[self.state]
        return handler(now_s, costmap, divider, x, y, heading, abs(speed_mps), road_yaw)

    def _cruise(self, now, costmap, divider, x, y, heading, speed, road_yaw):
        self.reason = 'cruise'
        if road_yaw is None:
            return Directive('normal', 'cruise')
        if self.cooldown_from is not None:
            if math.hypot(x-self.cooldown_from[0], y-self.cooldown_from[1]) < self.cooldown_m:
                self.reason = 'junction_cleared'
                return Directive('normal', self.reason)
            self.cooldown_from = None
            self.recent.clear()
        obs = self._observe(costmap, divider, x, y, road_yaw)
        if (not obs.is_junction and obs.left_edge_m is not None and obs.right_edge_m is not None
                and 3.0 <= obs.left_edge_m-obs.right_edge_m <= 7.0):
            self.road_width = .8*self.road_width+.2*(obs.left_edge_m-obs.right_edge_m)
        if obs.stop_line_m is not None:
            self.stop_count += 1
        else:
            self.stop_count = 0
        if self.stop_count >= self.stop_confirm:
            self.theta = road_yaw
            u = obs.stop_line_m
            self.stop_line = (x+u*math.cos(road_yaw), y+u*math.sin(road_yaw))
            self.state = 'approach'
            return self._approach(now, costmap, divider, x, y, heading, speed, road_yaw, observed=obs)
        # Openings without a stop line are reported in status only: the route
        # decision is made while stopped at the stop line.
        return Directive('normal', 'cruise')

    def _locked_guidance(self, road_yaw):
        if road_yaw is not None and abs(_wrap(road_yaw-self.theta)) < .25:
            return road_yaw
        return self.theta

    def _approach(self, now, costmap, divider, x, y, heading, speed, road_yaw, observed=None):
        guidance = self._locked_guidance(road_yaw)
        obs = observed or self._observe(costmap, divider, x, y, guidance)
        if obs.stop_line_m is not None:
            seen = (x+obs.stop_line_m*math.cos(guidance), y+obs.stop_line_m*math.sin(guidance))
            if math.dist(seen, self.stop_line) <= .6:
                self.stop_line = (.7*self.stop_line[0]+.3*seen[0], .7*self.stop_line[1]+.3*seen[1])
        remaining = self._along(x, y, self.stop_line)-self.front_m-self.stop_margin_m
        if remaining <= self.arrive_tolerance_m or (remaining < .35 and speed <= self.stopped_speed_mps):
            self.state = 'stopped'
            self.stop_since = now
            return self._hold('stopping_at_stop_line')
        c, s = math.cos(self.theta), math.sin(self.theta)
        reach = self.front_m+self.stop_margin_m
        stop_point = (self.stop_line[0]-reach*c, self.stop_line[1]-reach*s)
        self.reason = 'approaching_stop_line'
        return Directive('stop_at', self.reason, guidance_yaw=guidance, stop_point=stop_point)

    def _stopped(self, now, costmap, divider, x, y, heading, speed, road_yaw):
        self._observe(costmap, divider, x, y, self.theta)
        if speed > self.stopped_speed_mps:
            self.stop_since = now        # still settling: the hold starts at rest
            return self._hold('stopping_at_stop_line')
        if now-self.stop_since < self.stop_hold_s:
            return self._hold('stopped_at_stop_line')
        return self._commit(costmap, x, y, now)

    def _commit(self, costmap, x, y, now):
        options = self.options()
        want = self.request
        left_ok = options['left'] != 'blocked'
        blocked_too_long = (self.blocked_since is not None
                            and now-self.blocked_since >= self.blocked_fallback_s)
        if want == 'left' and left_ok and not (blocked_too_long and options['straight']):
            route = 'left'
            note = 'requested' if options['left'] == 'confirmed' else 'requested_branch_unconfirmed'
        elif options['straight']:
            route = 'straight'
            note = ('left_turn_blocked' if want == 'left' and blocked_too_long
                    else 'left_unavailable' if want == 'left'
                    else 'requested' if self.request_source != 'default' else 'default')
        elif left_ok:
            route, note = 'left', 'straight_unavailable'
        else:
            return self._hold('no_route_available')
        self.route, self.route_note = route, note
        opening = self._left_opening()
        if route == 'straight':
            far = 4.0
            if opening is not None:
                width = opening[1] or self.road_width
                far = self._along(x, y, opening[0])+width
            elif self.stop_line is not None:
                far = self._along(x, y, self.stop_line)+self.junction_setback_m+self.road_width
            self.state = 'crossing_straight'
            self.cross_origin = (x, y)
            self.cross_length = max(3.0, far+1.5)
            # A straight line through the junction from the robot's lane
            # position. Locking only the direction let the planner drift
            # sideways in the unmarked box (Gazebo 2026-11-18: 0.8 m).
            n = round((self.cross_length+2.0)/.05)
            c, s_ = math.cos(self.theta), math.sin(self.theta)
            line = tuple((x+c*k*.05, y+s_*k*.05, self.theta) for k in range(n+1))
            self.straight = line if self._turn_clear(costmap, line) else ()
            self.reason = 'crossing_straight'
            return self._crossing_straight(None, costmap, None, x, y, self.theta, 0.0, None)
        # Where the branch starts: the observed gap, else the stop line plus
        # the usual setback. Its width: the observed gap, else the road's.
        if opening is not None:
            start, width = self._along(x, y, opening[0]), opening[1] or self.road_width
        else:
            start, width = self._along(x, y, self.stop_line)+self.junction_setback_m, self.road_width
        # Right-hand traffic: the branch's outbound lane is its far half.
        target = start+.75*width
        poses = self._clear_turn(costmap, x, y, target)
        if not poses:
            self.state = 'stopped'
            self.stop_since = -math.inf  # retry the commit next tick
            if self.blocked_since is None:
                self.blocked_since = now
            return self._hold('left_turn_blocked')
        self.blocked_since = None
        self.turn, self.arc_end = poses
        self.state = 'crossing_left'
        self.reason = 'turning_left'
        return Directive('follow', self.reason, poses=self.turn)

    def _turn_clear(self, costmap, poses, start=0, length=None):
        """Padded-body check every 0.2 m; poses beyond the map are not judged."""
        field = self.planner.field(costmap)
        travelled, since_check, previous = 0.0, math.inf, None
        for pose in poses[start:]:
            if previous is not None:
                step = math.hypot(pose[0]-previous[0], pose[1]-previous[1])
                travelled += step
                since_check += step
            previous = pose
            if length is not None and travelled > length:
                break
            if since_check < .2:
                continue
            since_check = 0.0
            cost = self.planner._pose_cost(field, *pose)
            if cost == 'edge':
                break
            if cost is None:
                return False
        return True

    def _clear_turn(self, costmap, x, y, target):
        for radius in self.turn_radii:
            if target-radius < 0:
                continue
            poses, arc_end = left_turn_poses(x, y, self.theta, target, radius, self.turn_exit_m)
            if self._turn_clear(costmap, poses):
                return poses, arc_end
        return None

    def _finish(self, x, y, new_yaw=None):
        self.junctions_passed += 1
        self.last_route = self.route
        route, note = self.route, self.route_note
        self.reset()
        self.route, self.route_note = route, note   # keep for status
        self.cooldown_from = (x, y)
        self.reason = 'junction_cleared'
        return Directive('normal', self.reason, new_travel_yaw=new_yaw)

    def _guidance_agrees(self, road_yaw, yaw):
        if road_yaw is not None and abs(_wrap(road_yaw-yaw)) < self.guidance_agreement:
            self.agree_ticks += 1
        else:
            self.agree_ticks = 0
        return self.agree_ticks >= self.guidance_agreement_ticks

    def _crossing_straight(self, now, costmap, divider, x, y, heading, speed, road_yaw):
        progress = -self._along(x, y, self.cross_origin)
        if self.straight and progress < self.cross_length:
            line = self.straight
            nearest = min(range(len(line)), key=lambda i: (line[i][0]-x)**2+(line[i][1]-y)**2)
            if self._turn_clear(costmap, line, nearest, self.turn_check_m):
                self.reason = 'crossing_straight'
                return Directive('follow', self.reason, poses=line[nearest:])
            # Something on the line: let the planner steer around it.
            self.straight = ()
        if progress >= self.cross_length:
            # Hand back to observed guidance only once it sees the new road;
            # junction paint (stop line ends, corners) cannot be fitted.
            if (self._guidance_agrees(road_yaw, self.theta)
                    or progress >= self.cross_length+self.settle_limit_m):
                return self._finish(x, y, self.theta)
            self.reason = 'awaiting_lane_after_junction'
        else:
            self.reason = 'crossing_straight' if self.route == 'straight' else 'entering_branch'
        return Directive('locked', self.reason, guidance_yaw=self.theta)

    def _crossing_left(self, now, costmap, divider, x, y, heading, speed, road_yaw):
        poses = self.turn
        nearest = min(range(len(poses)), key=lambda i: (poses[i][0]-x)**2+(poses[i][1]-y)**2)
        exit_yaw = _wrap(self.theta+math.pi/2)
        aligned = nearest >= self.arc_end and abs(_wrap(heading-exit_yaw)) < self.exit_heading_tolerance
        if aligned and self._guidance_agrees(road_yaw, exit_yaw):
            return self._finish(x, y, exit_yaw)
        if nearest >= len(poses)-10:
            # Turn done but the branch's lane is not yet observed: continue on
            # the branch heading with the planner until it is.
            self.theta = exit_yaw
            self.state = 'crossing_straight'
            self.cross_origin = (x, y)
            self.cross_length = 0.0
            return self._crossing_straight(now, costmap, divider, x, y, heading, speed, road_yaw)
        if not self._turn_clear(costmap, poses, nearest, self.turn_check_m):
            return self._hold('left_turn_blocked')
        self.reason = 'turning_left'
        return Directive('follow', self.reason, poses=poses[nearest:])

    def expected_yaw(self, x, y):
        """Direction to seed lane-direction fitting while at a junction.

        None while cruising (the controller's own travel direction applies).
        At a junction the previous road direction is stale, and junction
        paint can point anywhere, so fitting is seeded with the committed
        heading: the road direction, or the turn path's tangent here.
        """
        if self.state == 'cruise':
            return None
        if self.state == 'crossing_left' and self.turn:
            nearest = min(range(len(self.turn)),
                          key=lambda i: (self.turn[i][0]-x)**2+(self.turn[i][1]-y)**2)
            return self.turn[nearest][2]
        return self.theta

    def status(self, now_s=None):
        """JSON-ready state for /self_drive/status."""
        obs = self.last_obs
        hold_left = None
        if self.state == 'stopped' and self.stop_since is not None and now_s is not None:
            hold_left = max(0.0, round(self.stop_hold_s-(now_s-self.stop_since), 2)) \
                if math.isfinite(self.stop_since) else 0.0
        return {
            'state': self.state, 'reason': self.reason,
            'turn_request': self.request, 'turn_request_source': self.request_source,
            'route': self.route, 'route_note': self.route_note,
            'options': self.options() if self.recent else None,
            'stop_line_m': obs.stop_line_m if obs else None,
            'left_opening_m': ([obs.left.start_m, obs.left.end_m] if obs and obs.left else None),
            'right_opening_m': ([obs.right.start_m, obs.right.end_m] if obs and obs.right else None),
            'road_width_m': round(self.road_width, 2),
            'hold_remaining_s': hold_left,
            'junctions_passed': self.junctions_passed,
        }
