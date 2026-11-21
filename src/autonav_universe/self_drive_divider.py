"""Pure local divider observation rasterization (no authored map input)."""
from __future__ import annotations
import math


def classified_divider(matches):
    """Hard paint takes precedence when multiple surfaces overlap."""
    return bool(matches) and all(getattr(tape, 'crossable', False) for tape in matches)


def rasterize_observations(points, width, height, resolution, origin, bridge_m=.8):
    """Rasterize observed hits and bridge short gaps between nearby hits.

    Points must already be in the rolling costmap frame. Cells not supported
    by recent observations remain zero; this does not assert they are free of
    unobserved paint. The collision grid remains completely separate.
    """
    if width <= 0 or height <= 0 or resolution <= 0:
        raise ValueError('invalid grid geometry')
    ox, oy, yaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    def cell(p):
        dx, dy = p[0]-ox, p[1]-oy
        return (c*dx+s*dy)/resolution, (-s*dx+c*dy)/resolution
    data = [0] * (width * height)
    projected = [cell(p) for p in points if len(p) >= 2 and all(math.isfinite(v) for v in p[:2])]
    def mark(a, b):
        n = max(1, math.ceil(math.dist(a,b)*2))
        for i in range(n+1):
            ix = math.floor(a[0] + (b[0]-a[0])*i/n)
            iy = math.floor(a[1] + (b[1]-a[1])*i/n)
            if 0 <= ix < width and 0 <= iy < height:
                data[iy*width+ix] = 100
    for i, a in enumerate(projected):
        mark(a, a)
        for b in projected[i+1:]:
            if math.dist(a,b)*resolution <= bridge_m:
                mark(a,b)
    return tuple(data)
