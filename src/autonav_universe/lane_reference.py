"""Rolling lane reference path for Nav2's MPPI controller in self-drive mode.

The AutoNav_25-26 stack drives with ``nav2_mppi_controller::MPPIController``
following a Smac path to a goal. The self-drive track has no goal, so this
module supplies MPPI's path instead: a short beam search over the live ROS
local costmap, oriented by the observed corridor direction. It never issues a
velocity; MPPI samples and scores every command against the same costmap.

Costmap values follow nav_msgs/OccupancyGrid conventions: -1 unknown, 0 free,
100 lethal. All inputs must share one frame. No track geometry, route, goals
or simulated truth enter this planner.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .lane_costmap import LaneCostConfig

try:  # The ROS container provides SciPy; headless installs use the fallback.
    import numpy as _np
    from scipy.ndimage import distance_transform_edt as _edt
except ImportError:  # pragma: no cover - exercised only where SciPy exists
    _np = _edt = None


@dataclass(frozen=True)
class ReferencePath:
    """Poses are (x, y, yaw) in the costmap frame, spaced for Nav2 critics."""
    poses: tuple[tuple[float, float, float], ...] = ()
    reason: str = 'no_reference_path'
    # Lane changes across observed divider paint, including an unfinished one.
    divider_crossings: int = 0
    length_m: float = 0.0
    pivot_rad: float = 0.0

    def __bool__(self):
        return self.reason == 'ok'


def _wrap(angle):
    return (angle+math.pi) % (2*math.pi) - math.pi


class _Field:
    """Lethal-distance and soft-cost lookup for one OccupancyGrid snapshot."""

    def __init__(self, grid, conservative_scale=1.0, threshold=100, nearest=False):
        self.grid = grid
        self.width, self.height, self.res = grid.width, grid.height, grid.resolution
        self.cos = math.cos(grid.origin_yaw)
        self.sin = math.sin(grid.origin_yaw)
        self.cells = grid.cells
        # nearest[i]: flat index of the source cell closest to cell i (or -1).
        self.distance, self.nearest = self._distance(conservative_scale, threshold, nearest)

    def _distance(self, scale, threshold, want_nearest):
        w, h, res = self.width, self.height, self.res
        lethal = [value >= threshold for value in self.cells]
        if _edt is not None:
            mask = _np.asarray(lethal, dtype=bool).reshape(h, w)
            if not mask.any():
                return [math.inf] * (w*h), ([-1] * (w*h) if want_nearest else None)
            if not want_nearest:
                return (_edt(~mask) * res).ravel().tolist(), None
            d, (iy, ix) = _edt(~mask, return_indices=True)
            return (d * res).ravel().tolist(), (iy*w + ix).ravel().tolist()
        # Two-pass 8-neighbour chamfer, carrying the nearest source cell. It
        # can exceed the Euclidean distance by up to 8.24 %, so the caller
        # scales it down to stay conservative.
        inf = math.inf
        d = [0.0 if is_lethal else inf for is_lethal in lethal]
        src = [i if is_lethal else -1 for i, is_lethal in enumerate(lethal)]
        if all(value == inf for value in d):
            return d, (src if want_nearest else None)
        diag = math.sqrt(2)
        forward = ((-1, 0, 1.0), (-1, -1, diag), (0, -1, 1.0), (1, -1, diag))
        backward = ((1, 0, 1.0), (1, 1, diag), (0, 1, 1.0), (-1, 1, diag))
        for offsets, rows, cols in ((forward, range(h), range(w)),
                                    (backward, range(h-1, -1, -1), range(w-1, -1, -1))):
            for iy in rows:
                for ix in cols:
                    i = iy*w+ix
                    best = d[i]
                    if best == 0.0:
                        continue
                    for dx, dy, step in offsets:
                        jx, jy = ix+dx, iy+dy
                        if 0 <= jx < w and 0 <= jy < h:
                            j = jy*w+jx
                            if d[j]+step < best:
                                best = d[j]+step
                                src[i] = src[j]
                    d[i] = best
        factor = res / scale
        return [value*factor for value in d], (src if want_nearest else None)

    def centre(self, i):
        """World coordinates of cell i's centre."""
        gx = (i % self.width + .5)*self.res
        gy = (i // self.width + .5)*self.res
        return (self.grid.origin_x+self.cos*gx-self.sin*gy,
                self.grid.origin_y+self.sin*gx+self.cos*gy)

    def index(self, x, y):
        dx, dy = x-self.grid.origin_x, y-self.grid.origin_y
        ix = math.floor((self.cos*dx+self.sin*dy)/self.res)
        iy = math.floor((-self.sin*dx+self.cos*dy)/self.res)
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return iy*self.width+ix
        return -1


class LaneReferencePlanner:
    """Beam search for a short, padded-body-safe path along the lane.

    The padded body extends from -0.42 m to +0.87 m from base_link and is
    1.02 m wide: the 1.09 m by 0.82 m robot plus the local costmap's 0.10 m
    footprint padding. Each pose is checked with a grid of covering circles
    against the distance to observed lethal cells. Unknown cells are
    traversable at the previous v2 1.15x premium under the base_link cell.
    """
    rear, front, half_width = -0.42, 0.87, 0.51
    step_m = 0.20
    horizon_m = 3.2
    # Curvature up to 0.8 rad/m (1.25 m radius); the inner lane is 2.3 m.
    heading_steps = (0.0, 0.03, -0.03, 0.07, -0.07, 0.12, -0.12, 0.16, -0.16)
    # In-place turns available before the first forward step.
    pivot_steps = (0.35, -0.35, 0.7, -0.7, 1.05, -1.05, 1.4, -1.4)
    beam_width = 64
    lateral_bin_m = 0.20
    heading_bin_rad = 0.12
    min_length_m = 0.6
    output_spacing_m = 0.05      # Smac lattice paths in AutoNav_25-26 use 5 cm.
    max_heading_error = 1.50
    heading_weight = 0.6
    smooth_weight = 0.25
    pivot_weight = 0.6
    # Breaks ties between equivalent paths (e.g. passing left or right).
    # Offline laps: 0.8 held on to a stale plan and sharpened the first
    # lane change (40 deg peak); 0.2 gave 35 deg; 0 gave 30 deg but leaves
    # nothing to stop noisy re-plans flipping sides.
    hysteresis_weight = 0.2
    hysteresis_cap_m = 0.6
    divider_band_m = 0.25
    divider_exit_margin_m = 0.10
    terminal_heading_weight = 1.0
    # Lane centring. Hard tape carries a clearance cost reaching ~1 m into a
    # 2.4 m lane while crossable divider paint carries none, so the cheapest
    # place was beside the divider; with no restoring term a few degrees of
    # tracking error then drifted the robot onto it (Gazebo 2026-11-18). The
    # offset from the centre between observed tape and observed divider is
    # charged quadratically, capped so lane changes stay affordable.
    centering_lane_max_m = 2.6     # both markings must be this close
    centering_cap_m = 0.5

    def __init__(self, lane_change_cost_m=0.25, centering_weight=1.5):
        if not math.isfinite(lane_change_cost_m) or lane_change_cost_m < 0:
            raise ValueError('lane_change_cost_m must be finite and nonnegative')
        if not math.isfinite(centering_weight) or centering_weight < 0:
            raise ValueError('centering_weight must be finite and nonnegative')
        self.lane_change_cost_m = float(lane_change_cost_m)
        self.centering_weight = float(centering_weight)
        previous = LaneCostConfig()
        self.clearance_weight = previous.clearance_weight
        self.unknown_multiplier = previous.unknown_multiplier
        length = self.front-self.rear
        width = 2*self.half_width
        nx, ny = math.ceil(length/.35), math.ceil(width/.35)
        sx, sy = length/nx, width/ny
        self._samples = tuple((self.rear+(i+.5)*sx, -self.half_width+(j+.5)*sy)
                              for i in range(nx) for j in range(ny))
        self._sample_radius = math.hypot(sx, sy)/2
        self._field = None
        self._paint = None

    def field(self, grid):
        if self._field is None or self._field.grid is not grid:
            field = _Field(grid, 1.0 if _edt is not None else 1.0824)
            res = grid.resolution
            # Body sample offsets and the body's farthest point, in cells.
            field.samples_cells = tuple((bx/res, by/res) for bx, by in self._samples)
            field.reach_cells = math.hypot(max(-self.rear, self.front), self.half_width)/res
            self._field = field
        return self._field

    def paint_field(self, divider):
        """Distance to observed divider paint; never conservative-scaled."""
        if divider is None:
            return None
        if self._paint is None or self._paint.grid is not divider:
            self._paint = _Field(divider, threshold=1, nearest=True)
        return self._paint

    def _pose_cost(self, field, x, y, heading):
        """Return (soft cost 0..0.99, unknown) or None if blocked; 'edge' off-map.

        Collision uses the whole padded body. The soft cost reads only the
        base_link cell: Nav2's inflation layer is already sized from that
        frame's footprint, and MPPI's CostCritic scores the same point.
        Taking the maximum over the body would count the body twice.
        """
        res, width, height = field.res, field.width, field.height
        dx, dy = x-field.grid.origin_x, y-field.grid.origin_y
        gx = (field.cos*dx+field.sin*dy)/res
        gy = (-field.sin*dx+field.cos*dy)/res
        ix, iy = math.floor(gx), math.floor(gy)
        if not (0 <= ix < width and 0 <= iy < height):
            return 'edge'
        centre = iy*width+ix
        value = field.cells[centre]
        soft = (max(0, value)/100.0, value < 0)
        distance = field.distance
        reach = field.reach_cells
        # Whole padded body inside the map and farther from lethal than its
        # farthest point: one lookup decides it. Most lane poses take this.
        if (distance[centre] > (reach+2)*res and reach+1 <= gx <= width-reach-1
                and reach+1 <= gy <= height-reach-1):
            return soft
        angle = heading-field.grid.origin_yaw
        c, s = math.cos(angle), math.sin(angle)
        clearance = self._sample_radius + res*math.sqrt(2)
        for bx, by in field.samples_cells:
            jx = math.floor(gx+c*bx-s*by)
            jy = math.floor(gy+s*bx+c*by)
            if not (0 <= jx < width and 0 <= jy < height):
                return 'edge'
            if distance[jy*width+jx] <= clearance:
                return None
        return soft

    def _centering(self, field, paint, x, y):
        """Squared offset (capped) from the centre between tape and divider."""
        if paint is None or not self.centering_weight:
            return 0.0
        i, j = field.index(x, y), paint.index(x, y)
        if i < 0 or j < 0:
            return 0.0
        boundary, divider = field.distance[i], paint.distance[j]
        if boundary > self.centering_lane_max_m or divider > self.centering_lane_max_m:
            return 0.0
        offset = min(abs(boundary-divider)/2, self.centering_cap_m)
        return offset*offset

    # Divider-band states carried by each search node.
    OUTSIDE, UNKNOWN_SIDE, STARTED_INSIDE = 0, 2, 3   # +1/-1: entered from that side

    def _band(self, paint, x, y, normal):
        """Distance to observed paint, and which side of it the point lies on.

        The side is the sign of the offset from the nearest paint cell along
        the corridor normal; 0 when too close to tell.
        """
        i = paint.index(x, y)
        if i < 0:
            return math.inf, 0
        q = paint.nearest[i]
        if q < 0:
            return paint.distance[i], 0
        qx, qy = paint.centre(q)
        offset = normal[0]*(x-qx) + normal[1]*(y-qy)
        return paint.distance[i], (1 if offset > .02 else -1 if offset < -.02 else 0)

    def _start_band_state(self, paint, x, y):
        if paint is None:
            return self.OUTSIDE
        i = paint.index(x, y)
        inside = i >= 0 and paint.distance[i] <= self.divider_band_m
        # No recoverable previous side when starting on the marking.
        return self.STARTED_INSIDE if inside else self.OUTSIDE

    def _paint_entries(self, paint, x0, y0, x1, y1, state, normal):
        """Lane changes across observed divider paint along one segment.

        The band within ``divider_band_m`` of paint bridges the 0.45 m dash
        gaps, so a path cannot change lanes through a gap unnoticed. Entering
        the band charges a provisional crossing; leaving it on the side it
        entered from refunds it (a touch is not a lane change), and leaving
        on the other side keeps it. A path that ends inside the band keeps
        the charge, so a rolling plan cannot hide an unfinished lane change.
        Charging each band entry instead made shallow crossings, which can
        brush the wavy band edge twice, dearer than steep ones. Unobserved
        paint is never charged. Returns (change in crossings, new state).
        """
        if paint is None:
            return 0, self.OUTSIDE
        length = math.hypot(x1-x0, y1-y0)
        exit_distance = self.divider_band_m+self.divider_exit_margin_m
        if state == self.OUTSIDE:
            i = paint.index(x0, y0)
            # Every point of the segment lies within `length` of its start,
            # so a start this far from paint proves the whole step is outside
            # the band. Most lane steps end here.
            if i >= 0 and paint.distance[i]-length > exit_distance:
                return 0, state
        count = max(1, math.ceil(length/.05))
        delta = 0
        for k in range(1, count+1):
            px, py = x0+(x1-x0)*k/count, y0+(y1-y0)*k/count
            i = paint.index(px, py)
            distance = paint.distance[i] if i >= 0 else math.inf
            if state == self.OUTSIDE:
                if distance <= self.divider_band_m:
                    side = self._band(paint, px, py, normal)[1]
                    state = side if side else self.UNKNOWN_SIDE
                    delta += 1
            elif distance > exit_distance:
                # Leave the band only beyond an extra margin, so a path along
                # its edge does not flicker in and out.
                if state in (1, -1) and self._band(paint, px, py, normal)[1] == state:
                    delta -= 1
                state = self.OUTSIDE
        return delta, state

    def _still_clear(self, field, previous, x, y):
        """Whether the previous path ahead of the robot keeps the body clear."""
        nearest = min(range(len(previous)),
                      key=lambda i: math.hypot(previous[i][0]-x, previous[i][1]-y))
        last = None
        for px, py, heading in previous[nearest:]:
            if last is not None and math.hypot(px-last[0], py-last[1]) < self.step_m/2:
                continue
            last = (px, py)
            if self._pose_cost(field, px, py, heading) is None:
                return False
        return True

    @staticmethod
    def _hysteresis_lookup(previous, cap):
        if not previous:
            return None
        cell = .1
        table = {}
        reach = math.ceil(cap/cell)
        last = None
        for px, py, _ in previous:
            if last is not None and math.hypot(px-last[0], py-last[1]) < cell:
                continue
            last = (px, py)
            cx, cy = round(px/cell), round(py/cell)
            for iy in range(cy-reach, cy+reach+1):
                for ix in range(cx-reach, cx+reach+1):
                    dist = math.hypot(ix*cell-px, iy*cell-py)
                    if dist < table.get((ix, iy), cap):
                        table[(ix, iy)] = dist
        return lambda x, y: table.get((round(x/cell), round(y/cell)), cap)

    def plan(self, grid, x, y, yaw, guidance_yaw, divider=None, previous=None):
        """Plan from the base_link pose; ``previous`` is the last path's poses."""
        if (not grid.valid() or not all(math.isfinite(v) for v in (x, y, yaw))
                or guidance_yaw is None or not math.isfinite(guidance_yaw)):
            return ReferencePath(reason='invalid_input')
        if divider is not None and not divider.valid():
            divider = None
        field = self.field(grid)
        paint = self.paint_field(divider)
        start = self._pose_cost(field, x, y, yaw)
        if start is None or start == 'edge':
            return ReferencePath(reason='footprint_blocked_or_unknown')
        if previous and not self._still_clear(field, previous, x, y):
            # A new observation blocks the last path. Attraction to it would
            # delay the avoidance until the obstacle is close, so drop it.
            previous = None
        hysteresis = self._hysteresis_lookup(previous, self.hysteresis_cap_m)
        ds = self.step_m
        g = guidance_yaw
        gc, gs = math.cos(g), math.sin(g)
        normal = (-gs, gc)          # Left of the observed corridor direction.
        steps = round(self.horizon_m/ds)
        # Node: (score, x, y, heading, soft, unknown, band, parent, pivot, depth)
        tree = [(0.0, x, y, yaw, start[0], start[1], self._start_band_state(paint, x, y),
                 -1, 0.0, 0)]
        beam = [0]
        for pivot in self.pivot_steps:
            heading = yaw+pivot
            if abs(_wrap(heading-g)) > self.max_heading_error:
                continue
            ok = True
            for k in range(1, math.ceil(abs(pivot)/.1)+1):
                if self._pose_cost(field, x, y, yaw+pivot*min(1.0, k*.1/abs(pivot))) in (None, 'edge'):
                    ok = False
                    break
            if ok:
                tree.append((-self.pivot_weight*abs(pivot), x, y, heading, start[0], start[1],
                             tree[0][6], 0, pivot, 0))
                beam.append(len(tree)-1)
        complete = []
        for depth in range(1, steps+1):
            best = {}
            for parent in beam:
                score, px, py, heading, soft, unknown, band, _, pivot, _ = tree[parent]
                for turn in self.heading_steps:
                    new_heading = heading+turn
                    error = _wrap(new_heading-g)
                    if abs(error) > self.max_heading_error:
                        continue
                    mid = heading+turn/2
                    nx, ny = px+ds*math.cos(mid), py+ds*math.sin(mid)
                    cost = self._pose_cost(field, nx, ny, new_heading)
                    if cost is None:
                        continue
                    if cost == 'edge':
                        if depth > 1:
                            complete.append(parent)
                        continue
                    new_soft, new_unknown = cost
                    entries, marked = self._paint_entries(paint, px, py, nx, ny, band, normal)
                    # Units are metres, as in the previous v2 objective:
                    # progress along the observed corridor, less clearance,
                    # unknown-space, heading, smoothness and lane-change costs.
                    gain = (ds*math.cos(mid-g)
                            - self.heading_weight*ds*(1-math.cos(error))
                            - self.clearance_weight*ds*(soft+new_soft)/2
                            - ds*(self.unknown_multiplier-1)*(int(unknown)+int(new_unknown))/2
                            - self.smooth_weight*abs(turn)
                            - entries*self.lane_change_cost_m)
                    if hysteresis is not None:
                        gain -= self.hysteresis_weight*ds*hysteresis(nx, ny)
                    gain -= self.centering_weight*ds*self._centering(field, paint, nx, ny)
                    node = (score+gain, nx, ny, new_heading, new_soft, new_unknown,
                            marked, parent, pivot, depth)
                    rank = score+gain-self.terminal_heading_weight*abs(error)  # _rank(node)
                    # Deduplicate by offset across the corridor and heading.
                    lateral = gc*(ny-y)-gs*(nx-x)
                    key = (round(lateral/self.lateral_bin_m), round(error/self.heading_bin_rad))
                    if key not in best or best[key][0] < rank:
                        best[key] = (rank, node)
            if not best:
                break
            # One node per coarse (offset, heading) cell. A top-N over fine
            # cells fills with near-identical straight variants; when they
            # all meet an obstacle beyond the horizon, no early swerve is
            # left and the eventual avoidance has to be sharp.
            ranked = sorted(best.values(), key=lambda item: item[0],
                            reverse=True)[:self.beam_width]
            beam = []
            for _, node in ranked:
                tree.append(node)
                beam.append(len(tree)-1)
        complete.extend(beam if beam and tree[beam[0]][9] == steps else [])
        candidates = [i for i in set(complete) if tree[i][9]*ds >= self.min_length_m]
        if not candidates:
            # Every branch was blocked before the horizon: keep the best long
            # enough partial path, if any, so MPPI can approach and re-plan.
            candidates = [i for i in range(len(tree)) if tree[i][9]*ds >= self.min_length_m]
        if not candidates:
            return ReferencePath(reason='no_reference_path')
        chosen = max(candidates, key=lambda i: self._rank(tree[i], g))
        return self._result(tree, chosen, x, y, yaw, paint, normal)

    def _rank(self, node, guidance_yaw):
        """Score plus a terminal heading term, used for pruning and selection.

        A path need only be clear to the horizon, so without this a path
        ending mid-swerve ranks like one that has finished its lane change,
        and each re-plan defers the swerve until it must be sharp.
        """
        return node[0] - self.terminal_heading_weight*abs(_wrap(node[3]-guidance_yaw))

    def _result(self, tree, chosen, x, y, yaw, paint, normal):
        chain = []
        node = chosen
        while node >= 0:
            chain.append(tree[node])
            node = tree[node][7]
        chain.reverse()
        pivot = chain[-1][8]
        poses = [(x, y, yaw)]
        if abs(pivot) > 1e-9:
            count = max(1, math.ceil(abs(pivot)/.1))
            poses.extend((x, y, _wrap(yaw+pivot*k/count)) for k in range(1, count+1))
        forward = [entry for entry in chain if entry[9] > 0]
        # The root and an optional pivot node share the start position.
        previous = (x, y, chain[len(chain)-len(forward)-1][3])
        count = max(1, round(self.step_m/self.output_spacing_m))
        for entry in forward:
            _, nx, ny, heading = entry[:4]
            for k in range(1, count+1):
                t = k/count
                poses.append((previous[0]+(nx-previous[0])*t, previous[1]+(ny-previous[1])*t,
                              _wrap(previous[2]+_wrap(heading-previous[2])*t)))
            previous = (nx, ny, heading)
        band = self._start_band_state(paint, x, y)
        crossings = 0
        for (ax, ay, _), (bx, by, _) in zip(poses, poses[1:]):
            entries, band = self._paint_entries(paint, ax, ay, bx, by, band, normal)
            crossings += entries
        return ReferencePath(tuple(poses), 'ok', crossings, len(forward)*self.step_m, pivot)
