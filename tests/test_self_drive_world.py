"""Structural checks for the authored ROS self-drive track assets."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('generate_self_drive_track', ROOT/'tools/generate_self_drive_track.py')
track = importlib.util.module_from_spec(spec)
spec.loader.exec_module(track)


class SelfDriveWorldTests(unittest.TestCase):
    def test_world_uses_real_robot_and_crossable_visual_divider(self):
        course, center, outer, inner = track.course_data()
        world = ET.fromstring(track.world_data(course, center))
        names = {node.attrib['name'] for node in world.findall('.//model')}
        self.assertTrue(any(name.startswith('self_drive_divider_') for name in names))
        self.assertEqual(course['mission_waypoints'], [])
        self.assertEqual(course['speed_check']['minimum_average_mps'], 0.0)
        self.assertTrue(course['dashed_lines'])
        self.assertTrue(all(line.get('allow_crossing') is True for line in course['dashed_lines']))
        self.assertTrue(all(item['name'] in names for item in course['internal_lines']))
        self.assertEqual(course['robot']['physical_half_length_m'], .545)
        self.assertEqual(course['robot']['physical_half_width_m'], .41)
        for obstacle in course['obstacles']:
            self.assertIn(obstacle['name'], names)
            # ROS renders 'post' as a dark cylinder, rather than a cone whose
            # visible and sensed width tapers toward the top.
            self.assertEqual(obstacle['type'], 'post')
            self.assertEqual(obstacle['radius_m'], .75)
            self.assertEqual(obstacle['height_m'], 1.0)
            model = world.find(f".//model[@name='{obstacle['name']}']")
            self.assertEqual(float(model.find('.//visual/geometry/cylinder/radius').text),
                             obstacle['radius_m'])
            self.assertEqual(float(model.find('.//collision/geometry/cylinder/radius').text),
                             obstacle['radius_m'])
        self.assertEqual(world.find('.//world').attrib['name'], 'igvc_competition')

    def test_start_and_obstacles_fit_with_a_clear_passing_lane(self):
        course, *_ = track.course_data()
        # Both straights retain a clear passing corridor for the full 0.82 m
        # robot body and the local controller's 0.10 m padding on each side.
        hx = course['robot']['physical_half_width_m'] + .10
        ox, oy, radius = track.OBSTACLES[0]
        start = course['start']
        self.assertGreater(start['y_m'] - hx, .6)
        self.assertGreater(oy - radius - .6, 0)
        opposite_lane_y = 4.2
        self.assertGreater(opposite_lane_y - oy, radius + hx)
        self.assertLess(opposite_lane_y + hx, 5.4)
        self.assertGreater(5.4 - (oy + radius), 2*hx)
        left_x, _, left_radius = track.OBSTACLES[1]
        self.assertGreater(left_x - left_radius - .6, 2*hx)

    def test_ros_base_world_is_preserved(self):
        course, center, *_ = track.course_data()
        base = ET.fromstring(track.world_data(course, center))
        world = base.find('world')
        for model in list(world.findall('model')):
            if model.get('name', '').startswith('self_drive_divider_'):
                world.remove(model)
        before = {m.get('name'): ET.tostring(m) for m in world.findall('model')}
        extended = ET.fromstring(track.extend_base_world(ET.tostring(base, encoding='unicode'), center))
        after_world = extended.find('world')
        after = {m.get('name'): m for m in after_world.findall('model')}
        for name, xml in before.items():
            # Whitespace can be normalized when pretty-printing.
            self.assertEqual(ET.fromstring(xml).find('pose').text, after[name].find('pose').text)
        self.assertEqual(len(after), len(before))


class IntersectionWorldTests(unittest.TestCase):
    def test_stop_lines_are_crossable_and_span_inbound_lanes(self):
        course, track_data = track.intersection_course_data()
        stops = {item['name']: item for item in course['dashed_lines'] if item['name'].startswith('stop_line_')}
        self.assertEqual(set(stops), {'stop_line_eastbound', 'stop_line_westbound', 'stop_line_southbound'})
        for item in course['dashed_lines']:
            self.assertIs(item['allow_crossing'], True)
            # One upstream dash per entry, so no line is split by a gap.
            length = ((item['end']['x_m']-item['start']['x_m'])**2 + (item['end']['y_m']-item['start']['y_m'])**2)**.5
            self.assertGreaterEqual(item['dash_length_m'], length)
        east = stops['stop_line_eastbound']
        start = course['start']
        self.assertGreater(east['start']['x_m'], start['x_m'] + 5)
        # The robot's own (south) lane: between the south edge and the divider.
        lane = sorted((east['start']['y_m'], east['end']['y_m']))
        self.assertTrue(lane[0] < start['y_m'] < lane[1])
        self.assertEqual(course['obstacles'], [])
        self.assertEqual(set(track_data['exits']), {'straight', 'left'})

    def test_world_and_files_generate(self):
        import json
        import subprocess
        import sys
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([sys.executable, str(ROOT/'tools/generate_self_drive_track.py'),
                            '--output', directory, '--layout', 'intersection'], check=True)
            course = json.loads(Path(directory, 'course.yaml').read_text())
            data = json.loads(Path(directory, 'track.json').read_text())
            world = ET.parse(Path(directory, 'world.sdf')).getroot()
        names = {node.attrib['name'] for node in world.findall('.//model')}
        self.assertEqual(course['course_id'], 'self_drive_intersection')
        self.assertEqual(data['layout'], 'intersection')
        self.assertTrue(all(item['name'] in names for item in course['internal_lines']))
        self.assertEqual(sum(name.startswith('self_drive_divider_') for name in names),
                         len(course['dashed_lines']))


if __name__ == '__main__':
    unittest.main()
