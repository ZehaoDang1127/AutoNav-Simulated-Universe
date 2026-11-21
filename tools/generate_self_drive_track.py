#!/usr/bin/env python3
"""Generate an authored self-drive track for the pinned ROS/Fortress harness.

Two layouts are available: the closed two-lane ``loop`` (default) and a
two-lane ``intersection``: a T-junction with a branch to the left and
crossable stop lines on each inbound lane.

The tape polygons in course.yaml drive simulated LiDAR tape returns. Crossable
paint is modeled with a distinct classified response by the sensor wrapper;
the controller receives observed paint, not the authored tape coordinates.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROBOT = {
    'base_link_to_nav_center_m': .225, 'lidar_x_from_base_link_m': .6598,
    'lidar_z_from_base_link_m': .2057, 'base_link_height_above_ground_m': .113,
    'gps_x_from_base_link_m': -.2122, 'gps_y_from_base_link_m': -.0001,
    'gps_z_from_base_link_m': .6616, 'wheel_track_m': .7233, 'wheel_radius_m': .1295,
    'physical_half_length_m': .545, 'physical_half_width_m': .41,
    'footprint_padding_m': .05, 'max_linear_speed_mps': .5,
    'max_angular_speed_radps': 1, 'cmd_latency_s': .08,
    'linear_time_constant_s': .2, 'angular_time_constant_s': .18,
}
# A 1.5 m diameter makes each tire wider than the 0.82 m vehicle, while
# preserving a wide passing corridor on the opposite side of the 4.8 m road.
OBSTACLES = ((13.0, 1.8, .75), (4.2, 10.0, .75))
OBSTACLE_HEIGHT_M = 1.0


def rounded_rectangle(x0, y0, x1, y1, radius, subdivisions=12):
    """Return a closed CCW polyline, with no repeated vertex."""
    points = []
    for cx, cy, angle in ((x1-radius, y0+radius, -math.pi/2),
                          (x1-radius, y1-radius, 0),
                          (x0+radius, y1-radius, math.pi/2),
                          (x0+radius, y0+radius, math.pi)):
        for i in range(subdivisions+1):
            a = angle + i*math.pi/(2*subdivisions)
            points.append((round(cx+radius*math.cos(a), 5), round(cy+radius*math.sin(a), 5)))
    return points


def edges(points):
    return list(zip(points, points[1:]+points[:1]))


def divider_paint(center):
    """Painted portions with a continuous dash phase around the course."""
    result = []
    distance = 0.0
    for a, b in edges(center):
        length = math.dist(a, b)
        offset = 0.0
        while offset < length-1e-9:
            phase = distance % 1.2
            chunk = min(length-offset, (.75-phase) if phase < .75 else (1.2-phase))
            if chunk < 1e-8:
                distance += 1e-7
                continue
            if phase < .75:
                start = tuple(a[k]+(b[k]-a[k])*offset/length for k in (0,1))
                end = tuple(a[k]+(b[k]-a[k])*(offset+chunk)/length for k in (0,1))
                if math.dist(start, end) > 1e-6:
                    result.append({'name': f'divider_{len(result):05}',
                                   'start': {'x_m': start[0], 'y_m': start[1]},
                                   'end': {'x_m': end[0], 'y_m': end[1]},
                                   'dash_length_m': 1.0, 'gap_length_m': .45,
                                   'width_m': .0762, 'allow_crossing': True})
            distance += chunk
            offset += chunk
    return result


def course_data():
    center = rounded_rectangle(3, 3, 23, 17, 3.5)
    outer = rounded_rectangle(.6, .6, 25.4, 19.4, 5.9)
    inner = rounded_rectangle(5.4, 5.4, 20.6, 14.6, 1.1)
    solids = []
    for boundary, points in (('outer', outer), ('inner', inner)):
        for i, (a, b) in enumerate(edges(points)):
            solids.append({'name': f'{boundary}_{i:03}', 'start': {'x_m': a[0], 'y_m': a[1]},
                           'end': {'x_m': b[0], 'y_m': b[1]}, 'width_m': .0762})
    return {
        'course_id': 'self_drive_track', 'description': 'Closed two-lane self-drive test track',
        'course_geometry': 'explicit_blender',
        'datum': {'latitude_deg': 37.2303, 'longitude_deg': -80.425, 'altitude_m': 650},
        'start': {'x_m': 6.5, 'y_m': 1.8, 'yaw_rad': 0},
        'finish': {'x_m': 6.5, 'y_m': 1.8, 'radius_m': 1.25},
        'robot': ROBOT, 'tape_width_m': .0762,
        'centerline': [{'x_m': x, 'y_m': y, 'width_m': 4.8} for x, y in center],
        'line_breaks': [{'segment_index': i, 'boundary': 'both', 'start_fraction': 0.0,
                         'end_fraction': 1.0} for i in range(len(center)-1)],
        'internal_lines': solids,
        'obstacles': [{'name': f'tire_{i}', 'type': 'post', 'x_m': x, 'y_m': y,
                       'radius_m': r, 'height_m': OBSTACLE_HEIGHT_M} for i, (x, y, r) in enumerate(OBSTACLES)],
        'ramps': [], 'speed_check': {'start_x_m': 6.5, 'end_distance_m': 13.4112,
            # This is a slow local-planning test, not the competition speed
            # challenge. Keep the referee's collision and boundary checks.
            'minimum_average_mps': 0.0, 'maximum_speed_mps': 2.2352,
            'blocking_stop_s': 3600.0, 'blocking_speed_mps': .02, 'blocking_progress_radius_m': .25},
        'mission_waypoints': [], 'analysis_stations': [], 'diagnostic_road_surfaces': [],
        'dashed_lines': divider_paint(center),
    }, center, outer, inner


def add_model(world, name, x, y, z, yaw, geometry, color, collision=False):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.7f}'
    link = ET.SubElement(model, 'link', name='link')
    def add_shape(parent):
        shape = ET.SubElement(parent, 'geometry')
        if geometry[0] == 'box':
            ET.SubElement(ET.SubElement(shape, 'box'), 'size').text = ' '.join(map(str, geometry[1:]))
        else:
            cylinder = ET.SubElement(shape, 'cylinder')
            ET.SubElement(cylinder, 'radius').text = str(geometry[1])
            ET.SubElement(cylinder, 'length').text = str(geometry[2])
    visual = ET.SubElement(link, 'visual', name='visual')
    add_shape(visual)
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = color
    ET.SubElement(material, 'diffuse').text = color
    if collision:
        add_shape(ET.SubElement(link, 'collision', name='collision'))


def add_visual_divider(world, center):
    """Add the broken center marking without adding collision or sensed tape."""
    existing = {model.get('name') for model in world.findall('model')}
    distance = 0.0
    index = 0
    for a, b in edges(center):
        length = math.dist(a, b)
        offset = 0.0
        while offset < length-1e-9:
            phase = distance % 1.2
            chunk = min(length-offset, (.75-phase) if phase < .75 else (1.2-phase))
            if chunk < 1e-8:
                distance += 1e-7
                continue
            if phase < .75:
                name = f'self_drive_divider_{index:05}'
                if name in existing:
                    raise ValueError(f'base world already contains {name}')
                start = tuple(a[k]+(b[k]-a[k])*offset/length for k in (0,1))
                end = tuple(a[k]+(b[k]-a[k])*(offset+chunk)/length for k in (0,1))
                dx, dy = end[0]-start[0], end[1]-start[1]
                add_model(world, name, (start[0]+end[0])/2, (start[1]+end[1])/2, .011,
                          math.atan2(dy, dx), ('box', round(math.hypot(dx,dy), 5), .0762, .012),
                          '.98 .98 .98 1')
                index += 1
            distance += chunk
            offset += chunk


def extend_base_world(base_text, center):
    """Keep the ROS-generated world, including its sensor-visible divider."""
    root = ET.fromstring(base_text)
    world = root.find('world')
    if world is None or world.get('name') != 'igvc_competition':
        raise ValueError('expected the pinned igvc_competition world')
    return base_text


def add_visual_paint(world, course):
    """Visual-only crossable paint for layouts without a closed divider."""
    existing = {model.get('name') for model in world.findall('model')}
    for index, item in enumerate(course['dashed_lines']):
        name = f'self_drive_divider_{index:05}'
        if name in existing:
            raise ValueError(f'base world already contains {name}')
        start = (item['start']['x_m'], item['start']['y_m'])
        end = (item['end']['x_m'], item['end']['y_m'])
        dx, dy = end[0]-start[0], end[1]-start[1]
        add_model(world, name, (start[0]+end[0])/2, (start[1]+end[1])/2, .011,
                  math.atan2(dy, dx), ('box', round(math.hypot(dx, dy), 5), item['width_m'], .012),
                  '.98 .98 .98 1')


def world_data(course, center):
    """Standalone world; ``center`` is None for layouts listing all paint."""
    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name='igvc_competition')
    for name in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors'):
        filename = {'Physics': 'physics', 'UserCommands': 'user-commands',
                    'SceneBroadcaster': 'scene-broadcaster', 'Sensors': 'sensors'}[name]
        plugin = ET.SubElement(world, 'plugin', filename=f'ignition-gazebo-{filename}-system',
                               name=f'ignition::gazebo::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 20 0 0 0'
    ET.SubElement(light, 'diffuse').text = '.8 .8 .75 1'
    ET.SubElement(light, 'direction').text = '-.35 .15 -.92'
    ET.SubElement(world, 'gravity').text = '0 0 -9.80665'
    ET.SubElement(world, 'magnetic_field').text = '6e-06 2.3e-05 -4.2e-05'
    physics = ET.SubElement(world, 'physics', name='default_physics', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '.01'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    sphere = ET.SubElement(world, 'spherical_coordinates')
    for key, val in (('surface_model', 'EARTH_WGS84'), ('latitude_deg','37.2303'),
                     ('longitude_deg','-80.425'), ('elevation','650'), ('heading_deg','0')):
        ET.SubElement(sphere, key).text = val
    add_model(world, 'ground', 13, 10, -.025, 0, ('box', 60, 60, .05), '.16 .16 .15 1', True)
    def line(name, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        if math.hypot(dx, dy) < 1e-6:
            return
        add_model(world, name, (a[0]+b[0])/2, (a[1]+b[1])/2, .011,
                  math.atan2(dy, dx), ('box', round(math.hypot(dx,dy), 5), .0762, .012),
                  '.98 .98 .98 1')
    for item in course['internal_lines']:
        line(item['name'], (item['start']['x_m'], item['start']['y_m']),
             (item['end']['x_m'], item['end']['y_m']))
    if center is None:
        add_visual_paint(world, course)
    else:
        add_visual_divider(world, center)
    for item in course['obstacles']:
        add_model(world, item['name'], item['x_m'], item['y_m'], item['height_m']/2, 0,
                  ('cylinder', item['radius_m'], item['height_m']), '.16 .16 .16 1', True)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode', xml_declaration=True)


# -- T-intersection (hand sketch, 2026-11-17) --------------------------------
# A horizontal two-lane road with a two-lane branch leaving to the north
# (left of the robot's eastbound travel). Right-hand traffic: each inbound
# lane has a crossable stop line just before the junction box. The robot
# starts eastbound in the south lane. The sketch's extra line across the
# outbound east lane is omitted: it would sit in the robot's straight route.
ROAD_X0, ROAD_X1 = 0.6, 30.0
ROAD_Y0, ROAD_Y1 = 2.0, 6.8          # 4.8 m road, as on the loop
BRANCH_X0, BRANCH_X1 = 12.0, 16.8    # 4.8 m branch
BRANCH_Y1 = 20.0
STOP_SETBACK_M = 0.6                 # stop line distance before the box
INTERSECTION_START = {'x_m': 3.0, 'y_m': 3.2, 'yaw_rad': 0.0}
# Evaluator-only route exits; the controller never reads them.
INTERSECTION_EXITS = {'straight': (27.0, 3.2), 'left': (15.6, 17.0)}
EXIT_RADIUS_M = 1.25
TAPE_WIDTH_M = .0762


def _segment(name, a, b, crossable=False):
    item = {'name': name, 'start': {'x_m': round(a[0], 5), 'y_m': round(a[1], 5)},
            'end': {'x_m': round(b[0], 5), 'y_m': round(b[1], 5)}, 'width_m': TAPE_WIDTH_M}
    if crossable:
        # One upstream dash per entry: the dash is never shorter than it.
        item.update({'dash_length_m': round(max(1.0, math.dist(a, b)+.01), 5),
                     'gap_length_m': .45, 'allow_crossing': True})
    return item


def intersection_lines():
    """Hard tape polylines: road edges, the two junction corners, end caps."""
    mid_y = (ROAD_Y0+ROAD_Y1)/2
    return {
        'south_edge': ((ROAD_X0, ROAD_Y0), (ROAD_X1, ROAD_Y0)),
        'west_corner': ((ROAD_X0, ROAD_Y1), (BRANCH_X0, ROAD_Y1), (BRANCH_X0, BRANCH_Y1)),
        'east_corner': ((ROAD_X1, ROAD_Y1), (BRANCH_X1, ROAD_Y1), (BRANCH_X1, BRANCH_Y1)),
        'west_end': ((ROAD_X0, ROAD_Y0), (ROAD_X0, ROAD_Y1)),
        'east_end': ((ROAD_X1, ROAD_Y0), (ROAD_X1, ROAD_Y1)),
        'north_end': ((BRANCH_X0, BRANCH_Y1), (BRANCH_X1, BRANCH_Y1)),
    }, mid_y


def intersection_paint():
    """Crossable paint: three dashed dividers and three stop lines."""
    lines, mid_y = intersection_lines()
    mid_x = (BRANCH_X0+BRANCH_X1)/2
    west_stop = BRANCH_X0-STOP_SETBACK_M
    east_stop = BRANCH_X1+STOP_SETBACK_M
    north_stop = ROAD_Y1+STOP_SETBACK_M
    dividers = {'west': ((ROAD_X0+.3, mid_y), (west_stop, mid_y)),
                'east': ((ROAD_X1-.3, mid_y), (east_stop, mid_y)),
                'north': ((mid_x, BRANCH_Y1-.3), (mid_x, north_stop))}
    # Each stop line spans its own inbound lane only (right-hand traffic).
    stops = {'eastbound': ((west_stop, ROAD_Y0+.05), (west_stop, mid_y)),
             'westbound': ((east_stop, mid_y), (east_stop, ROAD_Y1-.05)),
             'southbound': ((BRANCH_X0+.05, north_stop), (mid_x, north_stop))}
    return dividers, stops


def intersection_course_data():
    lines, mid_y = intersection_lines()
    dividers, stops = intersection_paint()
    solids = []
    for name, points in lines.items():
        for i, (a, b) in enumerate(zip(points, points[1:])):
            pieces = max(1, math.ceil(math.dist(a, b)/1.0))
            for k in range(pieces):
                p = tuple(a[j]+(b[j]-a[j])*k/pieces for j in (0, 1))
                q = tuple(a[j]+(b[j]-a[j])*(k+1)/pieces for j in (0, 1))
                solids.append(_segment(f'{name}_{i}_{k:03}', p, q))
    paint = []
    for name, (a, b) in dividers.items():
        length = math.dist(a, b)
        s = 0.0
        while s < length-1e-9:
            e = min(length, s+.75)
            p = tuple(a[j]+(b[j]-a[j])*s/length for j in (0, 1))
            q = tuple(a[j]+(b[j]-a[j])*e/length for j in (0, 1))
            if e-s > .05:
                paint.append(_segment(f'divider_{name}_{len(paint):05}', p, q, True))
            s += 1.2
    for name, (a, b) in stops.items():
        paint.append(_segment(f'stop_line_{name}', a, b, True))
    center = [(x, mid_y) for x in [ROAD_X0+i*2.0 for i in range(int((ROAD_X1-ROAD_X0)//2.0)+1)]]
    if center[-1][0] < ROAD_X1:
        center.append((ROAD_X1, mid_y))
    course = {
        'course_id': 'self_drive_intersection',
        'description': 'Two-lane T-intersection with crossable stop lines',
        'course_geometry': 'explicit_blender',
        'datum': {'latitude_deg': 37.2303, 'longitude_deg': -80.425, 'altitude_m': 650},
        'start': dict(INTERSECTION_START),
        # The referee scores one finish; the session also accepts the left exit.
        'finish': {'x_m': INTERSECTION_EXITS['straight'][0], 'y_m': INTERSECTION_EXITS['straight'][1],
                   'radius_m': EXIT_RADIUS_M},
        'robot': ROBOT, 'tape_width_m': TAPE_WIDTH_M,
        'centerline': [{'x_m': x, 'y_m': y, 'width_m': ROAD_Y1-ROAD_Y0} for x, y in center],
        'line_breaks': [{'segment_index': i, 'boundary': 'both', 'start_fraction': 0.0,
                         'end_fraction': 1.0} for i in range(len(center)-1)],
        'internal_lines': solids, 'obstacles': [], 'ramps': [],
        'speed_check': {'start_x_m': INTERSECTION_START['x_m']+.5, 'end_distance_m': 13.4112,
                        'minimum_average_mps': 0.0, 'maximum_speed_mps': 2.2352,
                        'blocking_stop_s': 3600.0, 'blocking_speed_mps': .02,
                        'blocking_progress_radius_m': .25},
        'mission_waypoints': [], 'analysis_stations': [], 'diagnostic_road_surfaces': [],
        'dashed_lines': paint,
    }
    track = {
        'layout': 'intersection',
        'lines': [list(points) for points in lines.values()],
        'dashed': [list(points) for points in dividers.values()],
        'stop_lines': [list(points) for points in stops.values()],
        'obstacles': [], 'robot': ROBOT, 'start': course['start'],
        'exits': {name: {'x_m': x, 'y_m': y, 'radius_m': EXIT_RADIUS_M}
                  for name, (x, y) in INTERSECTION_EXITS.items()},
        'extent': [0.0, 0.0, ROAD_X1+1.0, BRANCH_Y1+1.0],
    }
    return course, track


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--layout', choices=('loop', 'intersection'), default='loop')
    parser.add_argument('--base-world', type=Path,
                        help='extend the world generated by the installed ROS simulator')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base_text = args.base_world.read_text(encoding='utf-8') if args.base_world else None
    if args.layout == 'intersection':
        course, track = intersection_course_data()
        center = None
    else:
        course, center, outer, inner = course_data()
        track = {'layout': 'loop', 'center': center, 'outer': outer, 'inner': inner,
                 'obstacles': OBSTACLES, 'robot': ROBOT, 'start': course['start']}
    # JSON is valid YAML and avoids introducing a host-side YAML dependency.
    (args.output/'course.yaml').write_text(json.dumps(course, indent=2), encoding='utf-8')
    (args.output/'world.sdf').write_text(extend_base_world(base_text, center) if base_text is not None
                                         else world_data(course, center), encoding='utf-8')
    track.update({'divider_perception': 'classified simulated LiDAR reflectivity; observed locally',
                  'base_world_sha256': hashlib.sha256(base_text.encode('utf-8')).hexdigest() if base_text else None})
    (args.output/'track.json').write_text(json.dumps(track, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
