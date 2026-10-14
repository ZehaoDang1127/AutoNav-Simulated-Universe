"""Reference LiDAR cloud to observed costmap, in the sensor's local frame.

The PCA normal is compared only with sensor up. World geometry and private
terrain data are inaccessible here. Unknown map cells remain unknown.
"""
from __future__ import annotations

import heapq
import math
import statistics
import time
from collections import defaultdict


def _normal_from_pca(points: list[tuple[float, float, float]]) -> tuple[float, float, float] | None:
    if len(points) < 6:
        return None
    mean = tuple(statistics.fmean(point[i] for point in points) for i in range(3))
    covariance = [[sum((p[i]-mean[i])*(p[j]-mean[j]) for p in points)
                   for j in range(3)] for i in range(3)]
    vectors = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(20):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(covariance[ij[0]][ij[1]]))
        if abs(covariance[p][q]) < 1e-10:
            break
        theta = 0.5*math.atan2(2*covariance[p][q], covariance[q][q]-covariance[p][p])
        c, s = math.cos(theta), math.sin(theta)
        app, aqq, apq = covariance[p][p], covariance[q][q], covariance[p][q]
        covariance[p][p] = c*c*app - 2*c*s*apq + s*s*aqq
        covariance[q][q] = s*s*app + 2*c*s*apq + c*c*aqq
        covariance[p][q] = covariance[q][p] = 0.0
        for r in range(3):
            if r != p and r != q:
                arp, arq = covariance[r][p], covariance[r][q]
                covariance[r][p] = covariance[p][r] = c*arp-s*arq
                covariance[r][q] = covariance[q][r] = s*arp+c*arq
            vrp, vrq = vectors[r][p], vectors[r][q]
            vectors[r][p], vectors[r][q] = c*vrp-s*vrq, s*vrp+c*vrq
    eigenvalues = [covariance[i][i] for i in range(3)]
    order = sorted(range(3), key=eigenvalues.__getitem__)
    if eigenvalues[order[1]] <= max(1e-8, eigenvalues[order[2]]*0.01):
        return None  # points form a line, not a measured surface
    vector = tuple(vectors[i][order[0]] for i in range(3))
    length = math.sqrt(sum(v*v for v in vector))
    return tuple(v/length for v in vector) if length > 1e-12 else None


def pca_grade_percent(points: list[tuple[float, float, float]]) -> float | None:
    normal = _normal_from_pca(points)
    if normal is None:
        return None
    up = min(1.0, max(0.0, abs(normal[2])))
    return 100*math.tan(math.acos(up))


class ReferenceCostmap:
    def __init__(self, grade_limit_percent: float = 20.0, resolution_m: float = 0.3):
        self.grade_limit_percent = grade_limit_percent
        self.resolution_m = resolution_m
        self.cells: dict[tuple[int, int], str] = {}
        self.last_processing_ms = 0.0
        self.max_processing_ms = 0.0
        self.violations_60ms = 0

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return round(x/self.resolution_m), round(y/self.resolution_m)

    def update(self, cloud: tuple[tuple[float, float, float], ...],
               odom_x: float, odom_y: float, odom_yaw: float) -> float:
        start = time.perf_counter()
        local: dict[tuple[int, int], list[tuple[float, float, float]]] = defaultdict(list)
        for point in cloud:
            if len(point) == 3 and all(math.isfinite(v) for v in point):
                local[self._cell(point[0], point[1])].append(point)
        c, s = math.cos(odom_yaw), math.sin(odom_yaw)
        for key, own in local.items():
            neighborhood = [point for di in (-1, 0, 1) for dj in (-1, 0, 1)
                            for point in local.get((key[0]+di, key[1]+dj), ())]
            zs = sorted(p[2] for p in neighborhood)
            grade = pca_grade_percent(neighborhood)
            jump = (zs[-1]-zs[len(zs)//3]) if zs else 0.0
            if jump > 0.3 and zs[-1] > 0.2:
                label = "obstacle"
            elif grade is not None and grade > self.grade_limit_percent:
                label = "steep"
            else:
                label = "free"
            lx = statistics.fmean(p[0] for p in own)
            ly = statistics.fmean(p[1] for p in own)
            world_cell = self._cell(odom_x+c*lx-s*ly, odom_y+s*lx+c*ly)
            previous = self.cells.get(world_cell)
            if previous in ("obstacle", "steep") and label == "free":
                continue
            self.cells[world_cell] = label
        elapsed = (time.perf_counter()-start)*1000
        self.last_processing_ms = elapsed
        self.max_processing_ms = max(self.max_processing_ms, elapsed)
        if elapsed > 60:
            self.violations_60ms += 1
        return elapsed

    def plan(self, start: tuple[float, float], goal: tuple[float, float],
             half_width_m: float = 0.21) -> list[tuple[float, float]]:
        origin, target = self._cell(*start), self._cell(*goal)
        radius = max(1, math.ceil(half_width_m/self.resolution_m))
        blocked = set()
        for (x, y), label in self.cells.items():
            if label in ("obstacle", "steep"):
                for dx in range(-radius, radius+1):
                    for dy in range(-radius, radius+1):
                        if dx*dx+dy*dy <= radius*radius:
                            blocked.add((x+dx, y+dy))
        blocked.discard(origin)
        pad = math.ceil(3/self.resolution_m)
        min_x, max_x = min(origin[0], target[0])-pad, max(origin[0], target[0])+pad
        min_y, max_y = min(origin[1], target[1])-pad, max(origin[1], target[1])+pad
        queue = [(0.0, origin)]
        distance = {origin: 0.0}
        parent = {}
        while queue:
            _, node = heapq.heappop(queue)
            if node == target:
                result = []
                while node != origin:
                    result.append((node[0]*self.resolution_m, node[1]*self.resolution_m))
                    node = parent[node]
                result.reverse()
                return result
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = (node[0]+dx, node[1]+dy)
                if not (min_x <= nxt[0] <= max_x and min_y <= nxt[1] <= max_y):
                    continue
                if nxt in blocked:
                    continue
                # Unknown is traversable with an explicit exploration cost.
                cost = math.hypot(dx, dy) * (1.15 if nxt not in self.cells else 1.0)
                candidate = distance[node]+cost
                if candidate < distance.get(nxt, math.inf):
                    distance[nxt] = candidate
                    parent[nxt] = node
                    heuristic = math.hypot(nxt[0]-target[0], nxt[1]-target[1])
                    heapq.heappush(queue, (candidate+heuristic, nxt))
        return []
