# AutoNav-Simulated-Universe

Simulation and evaluation tools for AutoNav robot navigation. This repository
contains a shared Python navigation engine, a pinned ROS 2 Humble / Gazebo
Fortress integration, standalone subsystem simulators, and the Blender source
for the local mock course.

## Launch the full-course Qt workspace

After [Python/GUI setup](#quick-start-shared-python-engine), run this command
from the repository root in Windows PowerShell:

```powershell
.\Run_Full_Course.cmd
```

Preview opens without Docker. To start a simulation with **Run**, start Docker
Desktop's Linux engine and prepare the pinned ROS workspace using the
[desktop course guide](integrations/ros_humble_fortress/README.md#full-course-qt-workspace).

## Current status

The shared Python package is **version 0.1.0**. The ROS/Gazebo integration has
completed one autonomous run of the full local diagnostic course. The latest
recorded pass is **October 7, 2026**, attempt `full-track-fix-20261007-76`:

| Check | Recorded result |
|---|---|
| Ordered waypoints | 5/5 reached |
| Finish | Reached after all waypoints |
| Referee violations | Zero |
| Speed requirement | Passed |
| Stop gate | Five consecutive zero velocity commands |
| Distance / simulation time | 165.042 m / 378.0 s |
| Maximum speed | 0.5 m/s |

The October 14 Qt workspace now controls the real course backend. Live
Pause/Resume/Stop and fresh-reset checks pass, but its latest full-course retest reached 4/5
goals and stalled on the final return leg. Its stop gate passed. The earlier
diagnostic pass does not establish repeatable full-course completion.

This establishes one local-course diagnostic pass. Release acceptance still
requires repeated runs, a complete course and referee, sensor fidelity checks,
and end-to-end validation of the new full-course Qt control application. See
[implementation status](FULL_TRACK_IMPLEMENTATION_STATUS.md) for the evidence,
earlier attempts, and open gates. Recorded validation in the pinned Linux
container passed 32 Python tests and 30 ROS regression groups.

## Choose a workflow

The [ROS lane-aware cost map v2](docs/ros-lane-costmap.md) is wired into the
full-course launcher and Qt cost-map selector. Rebuild the pinned workspace
before use. All 872 Blender tape segments stay hard; the diagnostic course
adds a crossable dashed divider through the existing obstacle course, with a
0.25 m equivalent crossing penalty. Some sections are too narrow for two robot
widths, and the original right-side boundaries have gaps; the overlay does not
establish lane-following performance. The
[earlier comparison report](deliverables/ros-costmap-comparison-2026-10-23/comparison.md)
explains the clearance-field changes and validation limits. ROS compilation
and paired Gazebo laps remain pending because Docker's Linux engine was
unavailable. The separate [Python prototype](docs/lane-costmap-v2.md) and
`autonav-universe lane-costmap-demo` illustration remain available.

| Workflow | Use it for | Guide |
|---|---|---|
| Full-course Qt workspace | Preview the exported course and control a real ROS/Gazebo session | [Desktop course guide](integrations/ros_humble_fortress/README.md#full-course-qt-workspace) |
| Self-drive track | Test waypoint-free lane following and obstacle avoidance on a closed track | [Self-drive track](#self-drive-track) |
| Shared Python engine | Develop navigation plugins, run authored scenarios, compare algorithms, and replay episodes without ROS | [Universe guide](UNIVERSE_README.md) |
| ROS 2 / Gazebo Fortress | Exercise the actual robot navigation stack against the exported local course | [Integration guide](integrations/ros_humble_fortress/README.md) |
| Standalone simulators | Work on GPS, LiDAR, recovery behavior, wheel control, or the robot HUD independently | [Simulator commands](SIMULATOR_COMMANDS.md) |

The Python engine and ROS/Gazebo integration are separate execution paths.
The default `app` window controls the ROS/Gazebo course. Its Python scenario
lab tab runs the separate Python engine; `app --backend python` opens that lab
directly.

## Self-drive track

Install the GUI dependencies as described below, then open the **Self-drive
track** tab in the default `autonav-universe app` workspace. To open the mode
directly on Windows:

```powershell
.\.venv\Scripts\autonav-universe.exe self-drive
```

On macOS or Linux, run `./.venv/bin/autonav-universe self-drive` instead.
The tab launches a separate ROS 2 Humble / Gazebo Fortress session using the
same pinned Docker image and workspace as the full-course integration. Start
Docker Desktop's Linux engine and complete the
[ROS workspace setup](integrations/ros_humble_fortress/README.md) first. Run
starts a fresh session; Pause inhibits motion and Stop ends it.

The generated closed course has solid boundaries, two obstacles and a
crossable dashed divider. Velocity commands come from Nav2's
`nav2_mppi_controller::MPPIController`, the local controller of the
[AutoNav_25-26](https://github.com/KazakhStallion/AutoNav_25-26) robot stack.
The self-drive profile copies that robot's `FollowPath` block (critics,
weights, 56 × 0.05 s horizon, 1200 samples, 20 Hz) with two simulator
overrides: `vx_max` 0.50 m/s, the simulated drive's limit, and
`consider_footprint: true`, because this costmap defines the real body about
the rear axle while the robot profile checks only a 0.1 m point.
The robot follows a Smac path to a goal; this track has no waypoints, so
`self_drive_controller.py` sends MPPI a rolling 3.2 m reference path at 5 Hz
through `controller_server`'s FollowPath action. The path comes from a beam
search over the live `/local_costmap/costmap`, aligned with the travel
direction estimated from observed lane or boundary markings. Each pose keeps
the padded body clear of observed lethal cells, and the path may begin with an
in-place turn. MPPI scores its own rollouts against the same costmap.
The view shows the 8 m rolling ROS local map and the robot's 1.09 m by 0.82 m
footprint. Stale inputs, missing direction guidance, a blocked start pose or no
safe reference path cancel the FollowPath goal, publish zero velocity and
revoke the command gate's enable heartbeat.
Unknown cells remain visible as unknown; when the reference path's robot centre
enters them, the previous v2 1.15× traversal multiplier applies.
This permits travel into unseen space and does not guarantee it is clear.
Choose the **Broken-line crossing penalty** before starting a run. The value
is an equivalent path cost in metres per observed crossing; it remains
crossable even at the highest setting. The reference path pays it on entering
the 0.25 m band around observed divider paint, which spans the dash gaps, and
is refunded if it leaves the band on the side it entered: a touch is not a
crossing, and a path ending on the paint keeps the charge. Purple cells show recently observed
divider markings, separate from hard red collision cells. The simulator models
a distinct sensor response for the divider paint; the physical LiDAR cannot
necessarily classify identical paint this way. When markings are not observed,
their locations and crossing costs are unknown to this local planner.
The self-drive ROS profile restores the earlier v2 0.8 m quadratic tape
clearance and the baseline 1.15 m obstacle inflation (scaling 3.2). The
reference path uses the previous 3.0 clearance weight on the normalized ROS
grid cost at `base_link`, the point MPPI's CostCritic also scores.
The run ends after one scored lap because the upstream referee stops scoring
after the finish. On November 5, 2026, six MPPI ROS/Gazebo laps finished in
123–138 simulated seconds over 57–60 m at up to 0.50 m/s with no recorded
referee violations. The previous arc controller's lap on November 4 took
210 s over 66.3 m at its 0.35 m/s cap, with 65° heading excursions. The two
laps on the current code (`self-drive-ros-20261105T032423Z-58d771` and
`…032805Z-2fd0a2`, 123–125 s) peaked at 44–51° around the first tire and
50–53° around the second; the four earlier laps with stronger path hysteresis
peaked at 42–69° and 41–58°. MPPI tracking adds roughly 15° to the reference
path's own peak. Lap `…023007Z-93caff` failed on an inner-boundary tape crossing
at the top-left corner while the node was starving its TF callbacks; TF now
runs on a dedicated thread and grids are converted once per tick. A 12 m map
with 6 m obstacle marking mapped the tires about 1.8 m earlier but did not make
offline lane changes gentler and cost 1.5–2× the planning time, so the 8 m map
remains. Boundary detections can be intermittent, and these few laps are not
evidence of repeatability or hardware validation.

### T-intersection: stop, then straight or left

Choose **Track: T-intersection** in the Self-drive tab to drive a two-lane
T-junction with crossable stop lines. The robot recognises the junction from
its observed cost maps and stops for 2 s at its stop line. It then goes
straight, or turns left into the branch when **At junction** is set to
*Turn left*. The same request can be published on `/self_drive/turn_command`
(`std_msgs/String`, `straight` or `left`). The closed loop is unchanged: the
route is chosen only at a stop line. This mode has only been validated
offline (`tools/simulate_intersection.py`), not yet in Gazebo. See
[the intersection notes](docs/self-drive-intersection.md).

## Quick start: shared Python engine

Run these commands from the repository root. The shared engine requires
Python 3.10 or later; Python 3.12 is recommended when also using the standalone
simulators. The headless engine has no third-party runtime dependencies. The
optional GUI uses PyQt5.

### Windows PowerShell

```powershell
if (-not (Test-Path .\.venv\Scripts\python.exe)) {
    python -m venv .venv
}
.\.venv\Scripts\python.exe -m pip install -e ".[gui]"
.\.venv\Scripts\autonav-universe.exe validate-plugin --algorithm examples.my_navigation.policy:MyNavigator
.\.venv\Scripts\autonav-universe.exe run --scenario straight --seed 1 --output runs/example
.\.venv\Scripts\autonav-universe.exe app
```

### macOS / Linux

```bash
if [ ! -x .venv/bin/python ]; then
    python3 -m venv .venv
fi
./.venv/bin/python -m pip install -e '.[gui]'
./.venv/bin/autonav-universe validate-plugin --algorithm examples.my_navigation.policy:MyNavigator
./.venv/bin/autonav-universe run --scenario straight --seed 1 --output runs/example
./.venv/bin/autonav-universe app
```

For headless use, omit `[gui]` from the matching environment's install command
and omit `app`. `run` uses
`autonav_universe.algorithms:ReferenceNavigator` by default; select your own
implementation with `--algorithm module:Class`. The example plugin is in
[policy.py](examples/my_navigation/policy.py).

The default Qt workspace previews the exported Blender course and offers
Run, Pause/Resume, Stop, Reset, sensor health, estimated costmap/camera panels,
runtime logs, and saved results. On Windows, double-click `Run_Full_Course.cmd`
after setup. Docker Desktop's Linux engine and the pinned ROS workspace are
required for Run; preview does not require Docker. The course's asset gaps
remain visible in the workspace. Reset stops the active session, then the
next Run starts a new container with fresh localization, maps and referee.

The Python scenario lab offers scenario, algorithm, seed, and Run/Pause/Reset
controls. `run --gui` opens recorded replay after a Python episode. Algorithms
receive public observations and return velocity commands; the runner owns the
robot dynamics, sensor timing, private scoring, and truth replay. See the
[plugin contract and fidelity notes](UNIVERSE_README.md) before interpreting
benchmark results.

### Windows environment troubleshooting

Use `.\.venv\Scripts\python.exe -m pip` when launching the app from `.venv`.
A bare `pip` command may install into a different Python environment.

If `.venv` was recreated with a different Python version, old compiled Qt
components can remain installed for the previous version. Reinstall the GUI
dependencies with the environment's own interpreter:

```powershell
.\.venv\Scripts\python.exe -m pip install --force-reinstall -e ".[gui]"
.\.venv\Scripts\autonav-universe.exe app
```

### Benchmarks and replay

```powershell
.\.venv\Scripts\autonav-universe.exe benchmark --algorithm autonav_universe.algorithms:ReferenceNavigator --suite navigation-v1 --output runs/reference
.\.venv\Scripts\autonav-universe.exe benchmark --algorithm examples.my_navigation.policy:MyNavigator --suite navigation-v1 --output runs/candidate
.\.venv\Scripts\autonav-universe.exe compare runs/reference runs/candidate
.\.venv\Scripts\autonav-universe.exe replay runs/candidate/straight-1 --gui
```

On macOS/Linux, use `./.venv/bin/autonav-universe` for the same commands.

| Prototype suite | Scope | Matching built-in algorithm |
|---|---|---|
| `navigation-v1` | Six authored layouts: straight, turns, obstacle, permitted ramp, forbidden ramp, and combined | `ReferenceNavigator` |
| `gps-prototype` | GPS turn bootstrap, blackout, bias, and odometry drift | `GPSFollower` |
| `gps-lidar-prototype` | Ordered GPS goals with LiDAR, an obstacle, a ramp, and GPS blackout | `GPSLidarNavigator` |
| `planner-controller-prototype` | Six layouts with a fixed sensor-derived pose and costmap | `FixedPipelinePlanner` |

All four suites use seeds 1, 2, and 3 by default. Built-in algorithm entry
points use the prefix `autonav_universe.algorithms:`. These suites are still
prototypes; they are not the frozen release benchmarks in the integration plan.
Compare runs from matching suites and timing configurations. A failed episode
produces a nonzero benchmark exit code while retaining its results.

Runs save configuration manifests, episode metrics, events, and truth replay;
benchmarks also save CSV and Markdown summaries. `run --rich` records the full
public observations. The Python backend uses simplified planar dynamics and a
height field; it has no suspension, traction loss, tipping, or contact impulses.
Its scenarios do not contain the full Blender course geometry.

## ROS 2 Humble / Gazebo Fortress

The integration builds pinned simulator, robot, and Nav2 sources in a Docker
volume. It includes local wheel/gyro and global GPS localization, LiDAR-derived
mapping, footprint-aware planning and control, readiness checks, supervised
velocity commands, and course scoring. Navigation uses simulation time; the
stop gate uses a steady clock.

On Windows, start Docker Desktop's Linux engine before setup. The current run
launcher requests `--gpus all`, so the Docker runtime must support GPU access.
Download the LFS assets and build the workspace:

```powershell
git lfs install
git lfs pull
.\integrations\ros_humble_fortress\setup.cmd
```

Choose the diagnostic after setup:

```powershell
# Bounded compact-course diagnostic
.\integrations\ros_humble_fortress\run_compact_smoke.cmd 90

# Full local course: first generate the course/world bundle using the asset guide
.\integrations\ros_humble_fortress\run_full_track.cmd 900
```

The full-course launcher requires the matching generated `course.yaml`,
`world.sdf`, and manifest under `runs/full-track-integrated-lanes/`. Follow the
[course export instructions](Mock_Course_Asset/README.md) to create that bundle;
it is not included in a fresh clone. The launcher verifies source/course/world
hashes and waits for ROS readiness before dispatching the mission.

The Windows launcher opens a live browser course view and saves an offline
`replay.html`. Full runs also save a rosbag, result JSON, score, logs, and exact
input/build manifests in a fresh `runs/full-track-*` directory. Gazebo runs in
server-only mode. Mission completion and the stopping gate are reported
separately: a passed stop check alone does not establish a successful mission.

The October 7 pass belongs to the full local course. The compact-course
diagnostic has separate navigation failures and is not covered by that pass.
The [integration guide](integrations/ros_humble_fortress/README.md) documents
launcher options, container validation, and the navigation corrections.

## Standalone subsystem simulators

Each simulator keeps its own dependencies and virtual environment under
`<Sim>/simulated_world/`. Use Python 3.12 and install its `requirements.txt`;
the root shared-engine environment does not replace these environments.
[SIMULATOR_COMMANDS.md](SIMULATOR_COMMANDS.md) contains the complete Windows
setup, scenario flags, direct entry points, and benchmark commands.

| Simulator | What it exercises | Windows entry after setup |
|---|---|---|
| [LiDAR](LiDAR_Sim/RULES.md) | STL terrain, simulated point clouds, grade-based costmaps, and ramp crossings | `.\LiDAR_Sim\Run_LIDAR_SIM_direct.bat --threshold 20 --rays 2000` |
| [GPS](GPS_Sim/RULES.md) | GPS/odometry heading estimation without a magnetometer, waypoint missions, and interference | `.\GPS_Sim\Run_GPS_SIM_direct.bat --real --single --seed 7` |
| Behavior tree | Corridor/maze navigation and recovery transitions on Chaplygin sleigh dynamics | `.\BEHAVIOR_TREE_Sim\Run_BT_SIM_direct.bat --layout track --seed 7` |
| Speed control | Per-wheel PID tuning with a 0–5 MPH setpoint | `.\SPEED_Sim\simulated_world\.venv\Scripts\python.exe .\SPEED_Sim\simulated_world\speed_sim.py` |
| GUI/HUD | The robot HUD against synthetic ROS sensor topics | `.\GUI_Sim\Run_GUI.bat` |

The folders also provide macOS `.command` launchers. SPEED has a macOS launcher
and can be run directly through Python on Windows. The HUD simulator's
`Bake_Bag_Video` launchers render playback CSV or rosbag2 recordings to MP4.

For the standalone LiDAR GUI, click to place a robot, Shift-click to place a
goal, and press `G` to navigate. On macOS, the configuration launcher may
auto-start with default options; use `Run_LIDAR_SIM_direct.command` to pass
flags directly. A missing `Consolas` font on macOS/Linux falls back to the
system monospace font.

## Local course asset and release work

[Course.blend](Mock_Course_Asset/Course.blend) is the source for the local
diagnostic course. The audited scene contains 83 objects: a 38 × 38 m ground,
two line meshes, one ramp, 73 cones, a start marker, and five waypoint markers.
The export tools preserve object metadata and generate a paired course/world
bundle using the pinned upstream simulator. The SDF is generated from semantic
YAML rather than every Blender collision mesh.

![Top-down overview of the audited local Blender course](Mock_Course_Asset/overview.png)

The course still needs authored potholes, No Man's Land, an internal boundary,
an explicit finish gate, and ordered referee checkpoints. Waypoint/finish roles
are provisional, and the ground exceeds the reference footprint.
`--require-complete-assets` currently rejects the asset. The
[asset manifest](Mock_Course_Asset/course_asset_manifest.json) records the gaps.

Remaining release work includes:

- Three clean repeat runs, ten-seed evaluation, a rules-complete referee, and
  performance acceptance.
- Verified ramp contact and camera-visible marking geometry, hardware sensor
  calibration, camera line/pothole perception, and lane-to-GPS transitions. The
  current x86 container skips the upstream CUDA camera detector.
- Runtime validation and repeated-reset acceptance of the new ROS/Gazebo Qt
  workspace. Desktop sessions use a heartbeat interlock and supervised AUTO;
  the original command-line diagnostics retain their existing lifecycle.
- Full terrain/waypoint suites, legacy component parity checks, and a polished
  Python comparison view.

## Repository map and validation

```text
src/autonav_universe/          Shared Python engine, API, algorithms, CLI, viewer
examples/my_navigation/       Navigation plugin example
scenarios/                    Authored Python scenarios
benchmarks/                   Prototype suites and seed lists
integrations/                 Pinned ROS/Gazebo setup, profiles, and source patches
Mock_Course_Asset/            Blender source, metadata, semantics, and overview
tools/                        Course export, audit, and preparation tools
tests/                        Python checks and ROS/native regression groups
LiDAR_Sim/ GPS_Sim/           Standalone perception and GPS simulators
BEHAVIOR_TREE_Sim/ SPEED_Sim/  Standalone recovery and wheel-control simulators
GUI_Sim/                     Standalone robot HUD simulator
runs/                        Generated results, exports, logs, replays, and bags
```

After installing the shared package, run the Python checks from the repository
root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

On macOS/Linux, use `./.venv/bin/python`. One launcher lifecycle check requires
Linux and is skipped on Windows. ROS/native checks require the pinned
container and built workspace; see the integration guide.

Virtual environments, `runs/`, build outputs, temporary files, Python caches,
and generated recordings are ignored by Git. Recorded attempt bundles are
local outputs; the implementation report preserves their observed outcomes.
The `.stl` and `.blend` assets are tracked with Git LFS.

Further reading: [Python integration plan](INTEGRATION_PLAN.md),
[full-track plan](FULL_TRACK_AUTONAV_PLAN.md),
[implementation status](FULL_TRACK_IMPLEMENTATION_STATUS.md), and the pinned
upstream [robot](https://github.com/KazakhStallion/AutoNav_25-26) and
[simulator](https://github.com/blobspire/autonav_sim) repositories.
