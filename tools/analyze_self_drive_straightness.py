"""Measure lateral waviness in a selected straight portion of a saved ROS replay.

This is an evaluator-only tool: its coordinate bounds never reach the robot
controller. It fits a line to the longest contiguous run in a specified region.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re


def load_trail(path: Path):
    text = path.read_text(encoding='utf-8')
    if path.suffix.lower() == '.html':
        match = re.search(r'window\.__AUTONAV_EMBEDDED_STATE__ = (.*?);</script>', text)
        if not match:
            raise ValueError('Replay has no embedded state')
        data = json.loads(match.group(1))
    else:
        data = json.loads(text)
    trail = data.get('trail') if isinstance(data, dict) else data
    if not isinstance(trail, list):
        raise ValueError('Input does not contain a trail')
    return [(float(p[0]), float(p[1])) for p in trail]


def straightness(trail, axis, lower, upper, cross_lower=-math.inf, cross_upper=math.inf):
    if axis not in ('x', 'y') or not lower < upper or cross_lower >= cross_upper:
        raise ValueError('Invalid axis or bounds')
    index = 0 if axis == 'x' else 1
    segments, current = [], []
    for point in trail:
        along, across = point[index], point[1-index]
        if not math.isfinite(along) or not math.isfinite(across):
            raise ValueError('Trail contains nonfinite coordinates')
        if lower <= along <= upper and cross_lower <= across <= cross_upper:
            current.append((along, across))
        else:
            if current:
                segments.append(current)
            current = []
    if current:
        segments.append(current)
    if not segments:
        raise ValueError('No trail samples fall inside the selected region')
    segment = max(segments, key=lambda points: abs(points[-1][0]-points[0][0]))
    if len(segment) < 3:
        raise ValueError('At least three contiguous samples are required')
    n = len(segment)
    along_mean = sum(p[0] for p in segment)/n
    across_mean = sum(p[1] for p in segment)/n
    variance = sum((p[0]-along_mean)**2 for p in segment)
    if variance <= 1e-12:
        raise ValueError('Selected segment has no along-track extent')
    slope = sum((a-along_mean)*(b-across_mean) for a,b in segment)/variance
    residuals = [b-across_mean-slope*(a-along_mean) for a,b in segment]
    length = sum(math.dist(a,b) for a,b in zip(segment,segment[1:]))
    chord = math.dist(segment[0], segment[-1])
    if chord <= 1e-12:
        raise ValueError('Selected segment has no endpoint separation')
    return {'samples': n, 'along_progress_m': abs(segment[-1][0]-segment[0][0]),
            'path_length_m': length, 'endpoint_distance_m': chord,
            'extra_distance_percent': 100*(length/chord-1),
            'lateral_residual_rms_m': math.sqrt(sum(v*v for v in residuals)/n),
            'lateral_residual_peak_to_peak_m': max(residuals)-min(residuals),
            'lateral_end_to_end_m': segment[-1][1]-segment[0][1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay', type=Path)
    parser.add_argument('--axis', choices=('x', 'y'), default='x')
    parser.add_argument('--min', dest='lower', type=float, default=8.0)
    parser.add_argument('--max', dest='upper', type=float, default=18.0)
    parser.add_argument('--cross-min', type=float, default=8.0)
    parser.add_argument('--cross-max', type=float, default=math.inf)
    args = parser.parse_args()
    try:
        value = straightness(load_trail(args.replay), args.axis, args.lower, args.upper,
                             args.cross_min, args.cross_max)
    except (ValueError, OSError, TypeError, IndexError) as exc:
        parser.exit(2, f'Cannot evaluate straight segment: {exc}\n')
    print(json.dumps(value, indent=2))


if __name__ == '__main__':
    main()
