import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

try:
    import yaml
except ImportError:
    yaml = None

from autonav_universe.full_track import FullTrackSession

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("prepare_lane_costmap", ROOT / "integrations/ros_humble_fortress/prepare_lane_costmap.py")
module = importlib.util.module_from_spec(spec) if yaml is not None else None
if module is not None:
    spec.loader.exec_module(module)


@unittest.skipIf(yaml is None, "ROS profile preparation requires PyYAML")
class RosLaneCostmapTests(unittest.TestCase):
    def setUp(self):
        self.base = yaml.safe_load((ROOT / "integrations/robot/full_track_baseline.yaml").read_text(encoding="utf-8"))
        self.course = {"course_id": "test", "course_geometry": "explicit_blender",
                       "internal_lines": [
                           {"name": "divider-edge-1", "start": {"x_m": 0, "y_m": -.04}, "end": {"x_m": 5, "y_m": -.04}},
                           {"name": "divider-edge-2", "start": {"x_m": 0, "y_m": .04}, "end": {"x_m": 5, "y_m": .04}},
                           {"name": "boundary", "start": {"x_m": 0, "y_m": 3}, "end": {"x_m": 5, "y_m": 3}}]}
        self.roles = {"schema_version": 1, "frame_id": "map", "course_id": "test",
                      "crossable_line_names": [], "divider_centerlines_m": []}

    def test_previous_profile_is_preserved_exactly(self):
        profile, metadata = module.prepare(self.base, self.course, self.roles, "1")
        self.assertEqual(profile, self.base)
        self.assertEqual(metadata["divider_segments"], 0)

    def test_default_v2_retains_all_tape_hard_and_all_existing_guards(self):
        profile, metadata = module.prepare(self.base, self.course, self.roles)
        self.assertEqual(metadata["semantic_status"], "all_tape_hard")
        self.assertEqual(metadata["hard_segments"], 3)
        self.assertEqual(profile["controller_server"], self.base["controller_server"])
        self.assertEqual(profile["bt_navigator"], self.base["bt_navigator"])
        for name in ("local_costmap", "global_costmap"):
            old = self.base[name][name]["ros__parameters"]
            new = profile[name][name]["ros__parameters"]
            self.assertEqual(new["footprint"], old["footprint"])
            self.assertEqual(new["footprint_padding"], old["footprint_padding"])
            self.assertEqual(new["plugins"], old["plugins"])
            for layer in new["plugins"]:
                if new[layer].get("plugin") == "line_layer::LineLayer":
                    self.assertTrue(new[layer]["quadratic_clearance"])
                    self.assertNotIn("lane_dividers", new[layer])

    def test_two_paint_edges_share_one_transition_centreline(self):
        self.roles.update(crossable_line_names=["divider-edge-1", "divider-edge-2"],
                          divider_centerlines_m=[[[0, 0], [5, 0]]])
        profile, metadata = module.prepare(self.base, self.course, self.roles)
        planner = profile["planner_server"]["ros__parameters"]["GridBased"]
        self.assertEqual(planner["lane_dividers"], [0., 0., 5., 0.])
        self.assertEqual(metadata["crossable_paint_segments"], 2)
        self.assertEqual(metadata["divider_segments"], 1)
        self.assertFalse(planner["smooth_path"])
        line = profile["global_costmap"]["global_costmap"]["ros__parameters"]["lidar_line_layer"]
        self.assertEqual(len(line["lane_dividers"]), 8)
        self.assertEqual(line["lane_hard_boundaries"], [0., 3., 5., 3.])
        self.assertNotIn("lane_dividers", self.base["planner_server"]["ros__parameters"]["GridBased"])

    def test_ambiguous_mismatched_and_malformed_annotations_are_rejected(self):
        for changes in ({"frame_id": "odom"}, {"crossable_line_names": ["missing"]},
                        {"crossable_line_names": ["divider-edge-1"]},
                        {"crossable_line_names": ["divider-edge-1"], "course_id": "other"},
                        {"classification_tolerance_m": math.nan},
                        {"divider_centerlines_m": [[[0, 0], [math.inf, 0]]]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                module.prepare(self.base, self.course, {**self.roles, **changes})
        for penalty in (-1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                module.prepare(self.base, self.course, self.roles, penalty=penalty)

    def test_dashed_divider_classifies_paint_and_keeps_gap_transition(self):
        self.course["dashed_lines"] = [{"name": "dash", "start": {"x_m": 0, "y_m": 0},
                                        "end": {"x_m": 3, "y_m": 0}, "dash_length_m": 1,
                                        "gap_length_m": .5, "allow_crossing": True}]
        self.roles.update(crossable_line_names=["dash"], divider_centerlines_m=[[[0, 0], [3, 0]]])
        profile, metadata = module.prepare(self.base, self.course, self.roles)
        line = profile["global_costmap"]["global_costmap"]["ros__parameters"]["lidar_line_layer"]
        self.assertEqual(line["lane_dividers"], [0, 0, 1, 0, 1.5, 0, 2.5, 0])
        self.assertEqual(metadata["hard_segments"], 3)
        self.assertEqual(profile["planner_server"]["ros__parameters"]["GridBased"]["lane_dividers"], [0, 0, 3, 0])
        for changes in ({"allow_crossing": False}, {"dash_length_m": 0},
                        {"gap_length_m": -1}, {"dash_length_m": math.nan}):
            bad = dict(self.course["dashed_lines"][0], **changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                module.prepare(self.base, {**self.course, "dashed_lines": [bad]}, self.roles)

    def test_ros_parameter_serialization_has_no_yaml_aliases(self):
        self.roles.update(crossable_line_names=["divider-edge-1", "divider-edge-2"],
                          divider_centerlines_m=[[[0, 0], [5, 0]]])
        profile, _ = module.prepare(self.base, self.course, self.roles)
        serialized = module.dump_profile(profile)
        self.assertFalse(any(isinstance(event, yaml.AliasEvent) for event in yaml.parse(serialized)))
        self.assertEqual(yaml.safe_load(serialized), profile)

    def test_integrated_divider_follows_course_and_keeps_hard_geometry(self):
        authored = json.loads((ROOT / "Mock_Course_Asset/course_semantics.json").read_text(encoding="utf-8"))
        dividers = authored["diagnostic_dashed_lines"]
        self.assertGreater(len(dividers), 10)
        self.assertFalse(authored.get("diagnostic_boundary_lines"))
        self.assertTrue(all(line.get("allow_crossing") is True for line in dividers))
        for a, b in zip(dividers, dividers[1:]):
            self.assertEqual(a["end"], b["start"])
        self.assertEqual(dividers[-1]["end"], dividers[0]["start"])
        course = {"course_id": "local_mock_course", "course_geometry": "explicit_blender",
                  "internal_lines": self.course["internal_lines"], "dashed_lines": dividers}
        roles = yaml.safe_load((ROOT / "integrations/robot/full_track_lane_semantics.yaml").read_text(encoding="utf-8"))
        profile, metadata = module.prepare(self.base, course, roles)
        self.assertEqual(metadata["hard_segments"], len(self.course["internal_lines"]))
        self.assertEqual(metadata["divider_segments"], len(dividers))
        self.assertGreater(metadata["crossable_paint_segments"], len(dividers))
        layer = profile["global_costmap"]["global_costmap"]["ros__parameters"]["lidar_line_layer"]
        self.assertEqual(len(layer["lane_hard_boundaries"]), 4 * len(self.course["internal_lines"]))

    def test_validation_omits_only_explicitly_crossable_paint(self):
        from tools.prepare_full_track import hard_geometry_course
        course = {"internal_lines": [{"name": "boundary"}], "obstacles": [{"name": "cone"}],
                  "dashed_lines": [{"name": "crossable", "allow_crossing": True},
                                   {"name": "hard", "allow_crossing": False},
                                   {"name": "unknown"}]}
        checked, count = hard_geometry_course(course)
        self.assertEqual(count, 1)
        self.assertEqual([line["name"] for line in checked["dashed_lines"]], ["hard", "unknown"])
        self.assertEqual(checked["internal_lines"], course["internal_lines"])
        self.assertEqual(checked["obstacles"], course["obstacles"])
        self.assertEqual(len(course["dashed_lines"]), 3)

    def test_v2_launcher_rejects_a_stale_ros_workspace_before_writing_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            for name, value in (("base", self.base), ("course", self.course), ("semantics", self.roles)):
                (folder / (name+".yaml")).write_text(yaml.safe_dump(value), encoding="utf-8")
            (folder / "workspace.json").write_text(json.dumps({"input_sha256": {}}), encoding="utf-8")
            import sys
            command = [sys.executable, str(ROOT / "integrations/ros_humble_fortress/prepare_lane_costmap.py")]
            for name in ("base", "course", "semantics"):
                command += ["--"+name, str(folder / (name+".yaml"))]
            command += ["--workspace-manifest", str(folder / "workspace.json"),
                        "--output", str(folder / "nav2.yaml"), "--metadata", str(folder / "meta.json")]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("run setup.cmd", result.stderr)
            self.assertFalse((folder / "nav2.yaml").exists())

    def test_v1_also_requires_referee_patch_for_crossable_dashes(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            course = dict(self.course, dashed_lines=[{
                "name": "dash", "start": {"x_m": 0, "y_m": 0},
                "end": {"x_m": 3, "y_m": 0}, "allow_crossing": True}])
            for name, value in (("base", self.base), ("course", course), ("semantics", self.roles)):
                (folder / (name + ".yaml")).write_text(yaml.safe_dump(value), encoding="utf-8")
            (folder / "workspace.json").write_text(json.dumps({"input_sha256": {}}), encoding="utf-8")
            import sys
            command = [sys.executable, str(ROOT / "integrations/ros_humble_fortress/prepare_lane_costmap.py")]
            for name in ("base", "course", "semantics"):
                command += ["--" + name, str(folder / (name + ".yaml"))]
            command += ["--version", "1", "--workspace-manifest", str(folder / "workspace.json"),
                        "--output", str(folder / "nav2.yaml"), "--metadata", str(folder / "meta.json")]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("run setup.cmd", result.stderr)
            self.assertFalse((folder / "nav2.yaml").exists())

    def test_desktop_records_version_and_rejects_bad_penalties(self):
        session = FullTrackSession(ROOT, ROOT / "runs/full-track-diagnostic", costmap_version="1", lane_change_cost_m=2)
        self.assertEqual(session.costmap_version, "1")
        self.assertEqual(session.lane_change_cost_m, 2)
        with self.assertRaises(ValueError):
            FullTrackSession(ROOT, ROOT, costmap_version="3")
        with self.assertRaises(ValueError):
            FullTrackSession(ROOT, ROOT, lane_change_cost_m=math.nan)

    def test_actual_portable_cpp_geometry(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("A C++ compiler is required for native lane geometry")
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / ("lane_test.exe" if os.name == "nt" else "lane_test")
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            str(ROOT / "tests/ros/lane_geometry_test.cpp"), "-o", str(executable)],
                           check=True, capture_output=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
            self.assertIn("passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
