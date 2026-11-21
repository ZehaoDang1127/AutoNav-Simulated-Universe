"""Reproducible authored lane-change maps and an SVG cost-map illustration."""
from __future__ import annotations

import json
import math
from dataclasses import asdict
from pathlib import Path

from .lane_costmap import LaneAwareCostmap, LaneCostConfig


def make_demo(case: str = "sketch", penalty: float = 0.25):
    if case not in ("sketch", "optional", "clear"):
        raise ValueError("case must be sketch, optional, or clear")
    grid = LaneAwareCostmap(resolution_m=0.1,
                           config=LaneCostConfig(lane_change_cost_m=penalty))
    start, goal = (4.3, 1.0), (1.3, 7.0)
    tire = (4.4, 5.8, 1.15)
    if case == "optional":
        start, goal, tire = (2.9, 1.0), (2.9, 7.0), (3.6, 4.1, 0.8)
    elif case == "clear":
        goal, tire = (4.3, 7.0), None
    for ix in range(61):
        for iy in range(81):
            label = "boundary" if ix in (0, 60) or iy in (0, 80) else "free"
            if tire and math.hypot(ix * 0.1 - tire[0], iy * 0.1 - tire[1]) <= tire[2] + 0.1 / math.sqrt(2):
                label = "obstacle"
            grid.cells[ix, iy] = label
            grid.lane_ids[ix, iy] = "left" if ix < 26 else "right"
    return grid, start, goal, tire


def demo_data(case="sketch", penalty=0.25):
    grid, start, goal, tire = make_demo(case, penalty)
    half_width = 0.28
    result = grid.plan_with_costs(start, goal, half_width)
    return grid, {
        "schema_version": 2, "source": "authored illustration, not sensor measurements",
        "case": case, "config": asdict(grid.config), "resolution_m": grid.resolution_m,
        "bounds_m": [0, 0, 6, 8], "divider_x_m": 2.55,
        "footprint_half_size_m": half_width, "start": start, "goal": goal, "tire": tire,
        "plan": {**asdict(result), "total_cost_m": result.total_cost_m if result.status == "ok" else None},
    }


def render_svg(grid, data):
    """Render the actual A* result and clearance field, without path smoothing."""
    scale = 66
    x = lambda v: 36 + v * scale
    y = lambda v: 558 - v * scale
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="700" height="610" viewBox="0 0 700 610" role="img" aria-labelledby="title desc">',
             '<title id="title">Lane-aware cost map v2</title>',
             '<desc id="desc">Hard tire and boundaries, graded clearance costs, a crossable divider and the computed grid path.</desc>',
             '<rect width="700" height="610" fill="#fafafa"/>',
             '<g font-family="Segoe UI,Arial,sans-serif" font-size="14" fill="#202830">',
             '<text x="36" y="22" font-size="18">Lane-aware cost map v2</text>']
    soft = grid.clearance_field(data["footprint_half_size_m"])
    size = grid.resolution_m * scale
    for (ix, iy), label in grid.cells.items():
        value = soft.get((ix, iy), 0)
        fill = "#7c1537" if label in ("boundary", "obstacle") else "#ffffff"
        if label == "free" and value > 0:
            fill = "#f1df6e" if value < 0.65 else "#f5aa55" if value < 2.3 else "#c96b5c"
        parts.append(f'<rect x="{x(ix*.1)-size/2:.2f}" y="{y(iy*.1)-size/2:.2f}" width="{size+.1:.2f}" height="{size+.1:.2f}" fill="{fill}"/>')
    parts.append(f'<path d="M{x(2.55)} {y(0.05)} V{y(7.95)}" stroke="#776aa5" stroke-width="2" stroke-dasharray="7 5"/>')
    points = data["plan"]["points"]
    if points:
        coords = " ".join(f"{x(px):.2f},{y(py):.2f}" for px, py in points)
        parts.append(f'<polyline points="{coords}" fill="none" stroke="#3156c7" stroke-width="4" stroke-linejoin="round"/>')
    sx, sy = data["start"]
    half = data["footprint_half_size_m"] * scale
    parts.append(f'<rect x="{x(sx)-half}" y="{y(sy)-half}" width="{2*half}" height="{2*half}" fill="#fff" stroke="#202830" stroke-width="2"/>')
    parts.append(f'<text x="{x(sx)}" y="{y(sy)+half+21}" text-anchor="middle">Robot</text>')
    gx, gy = data["goal"]
    parts.append(f'<rect x="{x(gx)-9}" y="{y(gy)-9}" width="18" height="18" fill="#e4f2e8" stroke="#2c824e" stroke-width="3"/>')
    parts.append(f'<text x="{x(gx)}" y="{y(gy)-18}" text-anchor="middle">Goal</text>')
    if data["tire"]:
        tx, ty, _ = data["tire"]
        parts.append(f'<text x="{x(tx)}" y="{y(ty)+5}" text-anchor="middle" fill="white">Tire</text>')
    legend = [("#7c1537", "Tire / outer boundary"), ("#c96b5c", "Footprint clearance"),
              ("#f5aa55", "Higher proximity cost"), ("#f1df6e", "Lower proximity cost"),
              ("#776aa5", "Crossable divider"), ("#3156c7", "Computed path")]
    for i, (color, label) in enumerate(legend):
        yy = 86 + i * 30
        parts.append(f'<rect x="465" y="{yy-12}" width="13" height="13" fill="{color}"/><text x="487" y="{yy}">{label}</text>')
    plan = data["plan"]
    for i, label in enumerate((f'Penalty: {data["config"]["lane_change_cost_m"]:g} m / change',
                               f'Lane changes: {plan["lane_changes"]}',
                               f'Added change cost: {plan["lane_change_cost_m"]:.2f} m',
                               f'Path length: {plan["length_m"]:.2f} m',
                               f'Status: {plan["status"]}')):
        parts.append(f'<text x="465" y="{320+i*27}">{label}</text>')
    parts.append('<text x="36" y="594" font-size="12">Authored sketch example · grid path · conservative square footprint · dimensions assumed</text></g></svg>')
    return "\n".join(parts)


def write_demo(output: Path, case="sketch", penalty=0.25):
    grid, data = demo_data(case, penalty)
    output.mkdir(parents=True, exist_ok=True)
    (output / "lane-costmap-v2.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
    (output / "lane-costmap-v2.svg").write_text(render_svg(grid, data), encoding="utf-8")
    return data
