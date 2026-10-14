# AutoNav Universe prototype

This is the shared Python simulation path described in [INTEGRATION_PLAN.md](INTEGRATION_PLAN.md). It runs one robot state, one simulation clock, sensor models, a navigation plug-in process, wheel control, private scoring, and a recorded Qt viewer. The existing subsystem launchers are still available for their original experiments.

## Install and run

From the repository root with Python 3.10 or later:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[gui]"
.\.venv\Scripts\autonav-universe.exe validate-plugin --algorithm examples.my_navigation.policy:MyNavigator
.\.venv\Scripts\autonav-universe.exe run --scenario straight --algorithm examples.my_navigation.policy:MyNavigator --seed 1 --output runs/example
.\.venv\Scripts\autonav-universe.exe app
```

On macOS/Linux, replace `Scripts` with `bin`. Omit `[gui]` for the headless CLI. `run --gui` opens recorded truth replay after the episode; `app` has Run, Pause, Reset, algorithm, scenario, and seed controls with live sensor availability and truth-viewer motion.

## Compare algorithms

```powershell
.\.venv\Scripts\autonav-universe.exe benchmark --algorithm autonav_universe.algorithms:ReferenceNavigator --suite navigation-v1 --output runs/reference
.\.venv\Scripts\autonav-universe.exe benchmark --algorithm examples.my_navigation.policy:MyNavigator --suite navigation-v1 --output runs/candidate
.\.venv\Scripts\autonav-universe.exe compare runs/reference runs/candidate
.\.venv\Scripts\autonav-universe.exe replay runs/candidate/straight-1 --gui
```

`navigation-v1` is a **prototype suite** with three seeds and six authored scenarios, including an obstacle detour and both permitted and forbidden ramps. `gps-prototype` exercises turn bootstrap, blackout, bias, and odometry drift with `autonav_universe.algorithms:GPSFollower`. These are not the frozen release benchmarks in the plan.

The separate `planner-controller-prototype` suite runs the same six authored layouts with a fixed sensor-derived odometry estimate and LiDAR costmap. Run it with `--algorithm autonav_universe.algorithms:FixedPipelinePlanner`. Its results are a separate track and should be compared only with other runs of that suite. The planner/controller plug-in receives an immutable `Observation.reference` containing the estimated pose, odom-frame goal, observed costmap, pipeline version, LiDAR sample time, and reference perception time. Raw sensor samples and world geometry are not sent to that track. Reference perception time is measured in the runner and cannot be supplied by the plug-in.

`gps-lidar-prototype` runs an ordered GPS waypoint mission across a permitted ramp, around an obstacle, and through a GPS blackout. Use `--algorithm autonav_universe.algorithms:GPSLidarNavigator --suite gps-lidar-prototype`. The plug-in learns the geographic-to-odometry rotation from GPS and odometry motion, then projects the GPS goals into its LiDAR-derived local map. It is a simple demonstration estimator, not the legacy GPS EKF. The separate `gps-prototype` suite remains GPS-only and has no LiDAR.

Results contain a manifest, CSV summary, episode metrics, events/commands, and a separate truth replay file. `run --rich` additionally records every public observation. `compare` checks matching scenario, seed, API, and timing configurations. A benchmark returns a nonzero exit code when any episode fails, while saving every outcome.

Scoring uses private truth for contacts, forbidden terrain, course exits, and ordered goals. Arrival requires at least half of the oriented rectangular footprint area inside the goal circle; the overlap area is computed geometrically rather than sampled. Ramp traversals use separate entry and exit gates, so a detour around a forbidden ramp does not count as crossing it.

## Plug-in contract

Declare a `PluginManifest` and implement `reset(context)`, `step(observation) -> NavigationCommand`, and `close()` as in [policy.py](examples/my_navigation/policy.py). Select it with a `module:Class` entry point. The worker checks API version, track, required sensors, and requested update rate before launch. It receives only immutable [public API objects](src/autonav_universe/api.py); world state and scenario geometry remain in the runner. On the GPS-only track, odometry contains only local `(x,y)` and GPS fixes use named `latitude_deg`/`longitude_deg` fields. LiDAR points are `(x,y,z)` meters in the sensor frame. All samples have acquisition and delivery times; stale and invalid readings are explicit.

Pass plug-in settings with repeated `--param KEY=VALUE`; the example accepts `--param speed_mps=0.4`. These values are recorded in the manifest and delivered in `EpisodeContext.parameters`.

Commands are forward m/s and counterclockwise rad/s, with a simulation-time expiry. Invalid or timed-out actions produce explicit outcomes. The worker process is restarted for each episode. Wheel speed targets feed the shared PID and planar chassis; commands do not set pose. Delivered sensor snapshots include a `stale` flag, and the sensor delivery queue has a 256-sample limit with dropped samples recorded.

The simulation advances to exact sensor, controller (50 Hz), and navigation deadlines with integration steps no larger than 1/240 s. **Functional timing mode** waits for each plug-in result and bounds each call at two seconds. `--timing-mode deadline --budget-ms 50` paces simulation time against wall time, applies only on-time actions, records overruns/skipped observations, and keeps the previous valid command until its expiry. The two modes cannot be compared in one paired report. GUI rendering does not advance the simulation clock. Separate random streams keep GPS noise and plug-in seeds independent.

## Fidelity and current scope

The shared chassis and PID use provisional BT/SPEED parameters. The authored scenarios combine odometry, GPS faults, sparse sensor-frame LiDAR, ramps, obstacles, and ordered goals in one run. `ReferenceNavigator` builds an observed costmap with sensor-frame PCA grade checks, plans with A*, and has a small backup/spin recovery. `GPSLidarNavigator` adds GPS waypoint projection to that pipeline. The 60 ms perception metric is measured by the runner on the planner/controller track; a complete-navigation plug-in must instrument and report its own perception stage.

The current terrain is a simplified height field. It has no suspension, traction loss, tipping, contact impulse, or hardware-calibrated wheel forces. The legacy STL raycaster, GPS EKF, exact behavior tree, full ROS HUD, and Blender mock-course geometry have not yet been extracted into this engine. `combined.json` is an authored integration scenario, not a validated export of `Course.blend`. The Qt window is a new shared-run viewer; the existing GUI simulator's ROS-compatible HUD remains a separate legacy program.

For the Blender asset audit, run `blender -b Mock_Course_Asset/Course.blend --python tools/export_course.py -- runs/course-export`, then `python tools/audit_course_export.py runs/course-export/course_metadata.json`. The exporter preserves raw custom properties and writes a GLB; the audit checks transforms and meter units and lists property names. Converting that export into a runtime scenario requires checking the actual property schema first.

The Python evaluation release described by the plan still needs the full terrain/waypoint regression suites, validated mock-course export, legacy component parity checks, and a polished comparison view. The ROS/Gazebo connection is a later milestone and requires the actual robot workspace.
