"""Regression gate for the real local course asset and semantic overlay."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
import unittest

from tools.audit_course_export import audit
from tools.prepare_full_track import semantic_gaps, sha256


ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "Mock_Course_Asset"


class CoursePipelineTests(unittest.TestCase):
    def test_inventory_is_tied_to_the_real_blender_file(self):
        metadata_path = ASSET / "course_metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        semantics = json.loads((ASSET / "course_semantics.json").read_text(encoding="utf-8"))
        source_hash = sha256(ASSET / "Course.blend")
        self.assertTrue(audit(metadata_path)["valid_geometry_metadata"])
        self.assertEqual(metadata["source_sha256"], source_hash)
        self.assertEqual(semantics["source_sha256"], source_hash)
        self.assertEqual(metadata["schema_version"], 2)

    def test_missing_features_fail_the_asset_gate(self):
        metadata = json.loads((ASSET / "course_metadata.json").read_text(encoding="utf-8"))
        semantics = json.loads((ASSET / "course_semantics.json").read_text(encoding="utf-8"))
        five_goals = SimpleNamespace(mission_waypoints=[
            SimpleNamespace(label=str(i)) for i in range(1, 6)
        ])
        gaps = semantic_gaps(metadata, semantics, five_goals)
        self.assertTrue(any("pothole_objects" in gap for gap in gaps))
        self.assertTrue(any("finish_gate_objects" in gap for gap in gaps))
        self.assertTrue(any("5 GPS goals" in gap for gap in gaps))
        self.assertTrue(any("38.000 x 38.000" in gap for gap in gaps))
        # A cone must not satisfy a pothole role just by naming it in the overlay.
        semantics["pothole_objects"] = ["Cones.001"]
        gaps = semantic_gaps(metadata, semantics, five_goals)
        self.assertTrue(any("Cones.001 needs type=pothole" in gap for gap in gaps))


if __name__ == "__main__":
    unittest.main()
