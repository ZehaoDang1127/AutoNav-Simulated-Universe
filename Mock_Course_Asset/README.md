# Local AutoNav mock course

`Course.blend` is the local Blender source asset. It was audited with Blender
5.0.1 on 2026-09-28. The file is Zstandard-compressed and Blender 4.0.2 could
not open it. Its SHA-256 is recorded in `course_semantics.json`.

The scene is metric at scale 1 and contains 83 objects: a 38 x 38 m ground,
two line meshes, one ramp, 73 cones, one start marker, and five waypoint
markers. The ramp mesh is about 3.959 m long with a 0.297 m rise. Only the
first three waypoints have `waypoint_order`; the fourth and fifth use a generic
`prop` key. The fifth marker is near the start and is only a candidate finish
marker. `course_semantics.json` records these provisional roles.

The asset does **not** contain authored pothole discs, a No Man's Land region,
an internal boundary, a finish gate, or ordered referee checkpoints. The
ground exceeds the rules reference footprint. The current Blender course is
therefore a diagnostic scene, not the validated complete track described in
`FULL_TRACK_AUTONAV_PLAN.md`. The earlier description of GPS lat/lon, surface
tags, and spoofing zones was aspirational; those properties are absent from
this file.

## Reproduce the export

Use Blender 5.0.1 or later and the pinned simulator revision
`46b8e9aa953160b2d317fcc811e209fed46a52f4`. From the repository root:

```bash
git clone --filter=blob:none --no-checkout https://github.com/blobspire/autonav_sim.git runs/upstream-autonav-sim
git -C runs/upstream-autonav-sim checkout --detach 46b8e9aa953160b2d317fcc811e209fed46a52f4
git -C runs/upstream-autonav-sim apply ../../integrations/autonav_sim/validator_robot_profile.patch
git -C runs/upstream-autonav-sim apply ../../integrations/autonav_sim/runner_course_world.patch
git -C runs/upstream-autonav-sim apply ../../integrations/autonav_sim/crossable_dashed_tape.patch
# Python requirements for the upstream validator: numpy, scipy, PyYAML.
blender -b Mock_Course_Asset/Course.blend --python tools/export_course.py -- runs/course-export
blender -b Mock_Course_Asset/Course.blend --python tools/render_course_overview.py -- runs/course-export/overview.png
blender -b Mock_Course_Asset/Course.blend --python runs/upstream-autonav-sim/igvc_competition_sim/autoresearch/tools/export_blender_course.py -- --output runs/course-export/course.yaml --course-id local_mock_course
python -m tools.prepare_full_track \
  --source Mock_Course_Asset/Course.blend \
  --metadata runs/course-export/course_metadata.json \
  --course-yaml runs/course-export/course.yaml \
  --semantics Mock_Course_Asset/course_semantics.json \
  --sim-package runs/upstream-autonav-sim/igvc_competition_sim \
  --output runs/full-track-integrated-lanes
```

The upstream checkout path in this example is a local, ignored directory.
`prepare_full_track` reuses its course
loader and SDF generator, validates the source hash and semantic roles, and
writes `course.yaml`, `world.sdf`, `public_mission.json`, and
`course_manifest.json` from one diagnostic source. It does not pass the asset
gate while required features are missing. Use `--require-complete-assets` in
automation to get a nonzero exit code until the asset has all required roles.

The semantic overlay adds a crossable dashed divider that follows the existing
marked track through the obstacle course, including its open right-hand section.
The existing Blender tape remains hard, and the cones remain obstacles even
where they overlap divider paint. The generated paint uses 0.75 m dashes and
0.45 m gaps. It is included in the course YAML and SDF and is not a Blender
source mesh. The generated bundle and manifest include the overlay in their
hashes. The v2 profile classifies this paint as crossable and scores transitions
across its centreline, including through gaps. The simulator referee honors the
explicit `allow_crossing` flag after `setup.cmd` rebuilds the workspace; all
other tape remains a contact failure. Existing exported bundles need to be
regenerated.

The course was not designed as a two-lane road: parts of the marked corridor
are narrower than two robot widths, and the right-hand obstacle area has gaps in
its outer boundaries. Lane following and crossing behavior require ROS/Gazebo
validation; the overlay does not establish either.

The generated SDF uses the simulator's semantic world models. The GLB from
`export_course.py` is an inventory/visual artifact; it is not itself a Gazebo
world. The public mission file contains four provisional GPS targets and no
centerline, hidden obstacles, or scoring geometry. ROS/Gazebo behavior still
requires integration testing in a compatible Humble/Fortress environment.
The runner patch requires an explicit `WORLD` whenever `COURSE_CONFIG` is
custom and records their hashes; it does not add the missing readiness checks
or make the current asset release ready.

## Run the exported scene in ROS/Gazebo

After the export and `integrations/ros_humble_fortress/setup.cmd`, run
`integrations/ros_humble_fortress/run_full_track.cmd 900`. This verifies the
matching source/course/world hashes and tests all five exported markers in
the full local scene. It saves a readiness report, mission result, logs,
rosbag, and replay in a fresh `runs/full-track-*` directory. The observed
`full-track-20261003-03` attempt passed startup but stalled on its first leg
with 0/5 waypoints at simulation time 103.01 s; its stop gate passed. See
`integrations/ros_humble_fortress/README.md` for the launcher options.
