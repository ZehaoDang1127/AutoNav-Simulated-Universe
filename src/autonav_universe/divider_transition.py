"""Count only observed lane-divider side transitions, rather than tape touches."""
from __future__ import annotations
import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .local_drive import LocalGrid


def _local_normal(grid, gx, gy, radius_m=.6):
    """Fit an undirected normal to nearby observed divider cells only."""
    radius = max(2, math.ceil(radius_m / grid.resolution))
    center_x, center_y = math.floor(gx), math.floor(gy)
    support = []
    for iy in range(max(0, center_y-radius), min(grid.height, center_y+radius+1)):
        for ix in range(max(0, center_x-radius), min(grid.width, center_x+radius+1)):
            if grid.cells[iy*grid.width+ix] > 0 and math.hypot(ix+.5-gx, iy+.5-gy) <= radius:
                support.append((ix+.5, iy+.5))
    if len(support) < 3:
        return None
    mx = sum(p[0] for p in support)/len(support)
    my = sum(p[1] for p in support)/len(support)
    xx = sum((px-mx)**2 for px, _ in support)/len(support)
    yy = sum((py-my)**2 for _, py in support)/len(support)
    xy = sum((px-mx)*(py-my) for px, py in support)/len(support)
    total = xx+yy
    if total <= 1e-9 or math.hypot(xx-yy, 2*xy)/total < .5:
        return None
    angle=.5*math.atan2(2*xy,xx-yy)
    return mx, my, -math.sin(angle), math.cos(angle)


def observed_divider_transitions(points, divider: LocalGrid | None) -> int:
    """Charge when a path exits an observed strip on its opposite side.

    Unknown cells do not supply a lane identity. A touch and return, an
    unfinished entry, or an initially unclassified pose do not establish a
    transition. This routine uses only local observed divider cells; it does
    not infer unseen strip locations. Paths are sampled at half a grid cell.
    """
    if divider is None or not divider.valid() or not points:
        return 0
    def marked(p):
        ix, iy = math.floor(p[0]), math.floor(p[1])
        return (0 <= ix < divider.width and 0 <= iy < divider.height
                and divider.cells[iy*divider.width+ix] > 0)
    mapped=[divider.local(*p) for p in points]
    inside = marked(mapped[0])
    previous = mapped[0]
    entrance = None
    entry_location = None
    crossings = 0
    for start,end in zip(mapped,mapped[1:]):
        count = max(1, math.ceil(math.dist(start,end)*2))
        for i in range(1,count+1):
            p=(start[0]+(end[0]-start[0])*i/count,
               start[1]+(end[1]-start[1])*i/count)
            now_inside=marked(p)
            if now_inside and not inside:
                entrance, entry_location = previous, p
            elif inside and not now_inside:
                if entrance is not None:
                    model=_local_normal(divider, *entry_location)
                    if model is not None:
                        mx,my,nx,ny=model
                        before=(entrance[0]-mx)*nx+(entrance[1]-my)*ny
                        after=(p[0]-mx)*nx+(p[1]-my)*ny
                        if before*after < -1e-8:
                            crossings += 1
                entrance = entry_location = None
            inside,previous=now_inside,p
    return crossings
