# Pinned ROS 2 Humble / Gazebo Fortress integration

This directory contains reproducible **diagnostic compact and full local-course** runs.
It builds the pinned simulator and robot sources in a Docker volume and leaves
the repository's Python prototype intact. It is a bring-up stage of
`FULL_TRACK_AUTONAV_PLAN.md`, not the delivered full-track application.

## Windows setup

Start Docker Desktop's Linux engine, then run from PowerShell. The `.cmd`
launchers also work when PowerShell's execution policy blocks `.ps1` files:

```powershell
.\integrations\ros_humble_fortress\setup.cmd
.\integrations\ros_humble_fortress\run_compact_smoke.cmd 90
```

The setup script builds image `autonav-humble-fortress:20260928` from the
checksum-pinned OSRF Humble simulation image and builds 19 selected ROS packages
in volume `autonav-full-track-20260928`. It verifies the simulator and robot
commit SHAs and applies only the tracked integration patches. If an existing
checkout has a different commit or conflicting edits, setup stops without
resetting it.

The smoke script generates logs, `readiness.json`, input hashes, `result.json`,
score text, and `replay.html` in `runs/compact-ros-smoke/`. The `.cmd` launcher
opens a live top-down course view at `http://127.0.0.1:8765/` while the run is
active, then opens the saved replay when Docker exits. If the browser does not
open automatically, visit that address during the run. The replay can be opened
later directly from the run directory. This is a course and ROS telemetry view;
Gazebo's 3D client is still disabled in this Docker diagnostic.
The live URL closes when the Docker run ends. The saved `replay.html` works
offline and displays a `Saved replay` label without polling the closed server.

The `.cmd` wrapper invokes PowerShell with a process-only execution-policy
bypass, so it works when directly running `.ps1` scripts is blocked. The
PowerShell launcher also accepts `-RunName` to retain multiple attempts,
`-ViewerPort` if 8765 is occupied, and `-NoViewer` for headless runs. It starts
a velocity gate before Gazebo: Nav2 publishes
`/cmd_vel`, the gate publishes `/cmd_vel_supervised`, and the calibrated dynamics
adapter consumes only the latter. The gate requires fresh mission heartbeats
and Nav2 commands; it latches zero on referee failure. A steady-clock timer
keeps the stop behavior independent of simulation time.

This is still a diagnostic runner. Scored attempts have reached
`obstacle_gps` but failed later at either the internal no-cross line or the
ramp edge. The `.cmd` launcher returns a nonzero exit code for a mission
failure; inspect `runs/compact-ros-smoke/mission.log`, `final_score.txt`, and
`replay.html` for the exact result. The final terminal JSON reports
`mission_passed` and `safety_gate_passed` separately, including the first
failure pose. The replay marks that pose on the course. `postrun_gate.json`
with `"safety_gate_passed": true` only confirms that the gate issued zero
velocity commands after the mission.

The readiness probe waits for an advancing clock, fresh LiDAR, GPS, odometry,
map, TF chain, active Nav2 servers, and the waypoint action. The mission runner
starts only after this check, sends the course waypoints, cancels an active
action on failure or timeout, and stops its gate heartbeat. The shell watchdog
is an emergency process deadline after the ROS runner's own deadline.

## Full local-track test

After setup and the export described in `Mock_Course_Asset/README.md`, run:

```powershell
.\integrations\ros_humble_fortress\run_full_track.cmd 900
```

This selects `runs/full-track-diagnostic/course.yaml` and its matching
`world.sdf`: the full exported local Blender scene, with 73 cones, 872 tape
segments, one ramp, and all five exported GPS markers. It checks the source,
course, and world hashes and the geometry-check result before starting Gazebo.
The mission has a 900-second wall-time limit. Each invocation writes to a fresh
timestamped run directory, records a rosbag and costmap diagnostics, and opens
the live course view and saved replay. For an explicitly named headless run:

```powershell
.\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -NoViewer -RunName full-track-test-01 -MissionTimeoutSec 900
```

The larger world exposed two startup problems. The spawn helper now waits for
an advancing clock and verifies actual model odometry, retrying with entity
renaming disabled. Tape-distance sensor queries use a spatial index while
retaining the exact distance calculation; scanning all 872 segments per beam
had made LiDAR frames stale. A readiness failure now saves a failed result
without starting the mission.

The recorded run `runs/full-track-20261003-03/` passed startup but failed the
mission at simulation time 103.01 s with
`blocking_traffic_no_progress_over_60s`, near (0.193, -4.988 m). It traveled
6.778 m and reached 0/5 waypoints. The planner logged 122 lethal-start errors
and zero goal preemptions. The stop gate verified five consecutive zero
commands. `result.json`, `final_score.txt`, `stack.log`, `bag/`, and
`replay.html` preserve the result. Earlier attempts `full-track-20261003-01`
and `-02` failed startup before the mission.

The October 3 verification passed four ROS regression tests and 30 Python tests.
That full-scene attempt did not complete a lap.

## Full-track failure corrections (2026-10-05)

The obstacle mirror accepts lower observations within its forward clearing
view. It retains unseen cones even when a neighbor's inflation halo overlaps
their remembered marks. Tape mirrors can accept lower persisted halo costs
outside the view; their source includes line memory. UNKNOWN observations keep
memory. Mirror layers merge by maximum cost so soft tape cannot overwrite a
lethal cone from another layer. Runtime regressions cover these clearing rules.
The obstacle clearing sector and scan projections use `lidar_planar`, an upright
frame at the LiDAR's horizontal mount. A
body-centered sector can claim a near-side cone is visible even after it moves
behind the lidar's forward-offset mount; unavailable mount transforms inhibit
clearing.

The simulator's geometry canvas now publishes UNKNOWN rather than marking the
entire course and its outside margin FREE. The global obstacle mirror retains
FREE cells only in its configured forward clearing sector, filling UNKNOWN
master cells without lowering another layer's obstacle costs. FREE retention is
an opt-in parameter; camera and lidar tape mirrors keep their existing behavior.
The sector restricts the rolling-window background; it is not an occlusion map.
Remaining unknown planning cells receive cost 200, while the sensor canvas stays
UNKNOWN. Humble Smac's obstacle heuristic skips cost 255; the finite exploration
penalty supports distant goals and keeps observed FREE cells preferable. The
default `unknown_space_cost: -1` preserves existing plugin behavior elsewhere.

Full runs use a native Fortress IMU at 50 Hz. Its ROS adapter publishes angular
velocity in `base_link` and disables orientation and acceleration measurements.
The sensor-equipped URDF and runtime world are saved with each attempt;
regressions verify that robot and course geometry remain identical. Native tests
cover continuous sampling through turn, stop, and resume. The pose-derived gyro
is retained on `/igvc_sim/pose_gyro` for diagnostics when the native feed is active.
That fallback uses physics sample timestamps. Repeated held poses are skipped until
a stationary heartbeat is needed, avoiding zero/rate spikes caused by the
timer cadence. Each heartbeat advances the held-yaw time reference so the first
resumed turn is not averaged over the stopped interval. Physics callbacks publish
new samples immediately, with a lock protecting the yaw/timestamp pair and
derivative state. A continuous local EKF fuses wheel body velocity and gyro rate without
wheel-integrated position or yaw. A second EKF fuses projected GPS position and
local heading/velocity, owning map-to-odom while the local EKF owns odom-to-body.
Startup requires fresh IMU, projected GPS odometry, and both filter outputs.
The course's initial east-facing orientation supplies the simulated heading
convention; this is not a real-world heading calibration result.
GPS waypoints use navsat_transform's asynchronous `FromLL` service, sharing the
global filter's datum and map projection. Each waypoint projection is cached
for its active action and dispatched through the existing single Nav2 action.
This replaces the independent heading-based projection only when the GPS EKF
is enabled. The earlier projection moved the first target about 10 m as wheel
odometry drifted; regression tests preserve a fixed target under that drift
and reject responses for a replaced action.

MPPI and the velocity smoother now share the calibrated 0.50 m/s forward limit,
0.25 m/s reverse limit, and acceleration limits. MPPI permits the reverse
maneuvers generated by Smac. Stock inflation runs after tape and obstacle
merging and extends beyond the padded body's circumscribed radius. Planner and
controller recoveries use separate Wait servers with positive integer seconds,
as required by the Humble implementation.
Nav2 poses now use `base_link`, the drive axle for DiffDrive motion and odometry.
The same full body is expressed with asymmetric x extents -0.32..0.77 m and
y extents +-0.41 m. Local/global costmaps and all BT gates use 0.15 m padding
for measured pose/projection error; stock inflation covers the maximum padded
axle-relative extent with a 1.15 m radius.
The behavior tree plans directly to the refreshed mission goal. The earlier
GoalBender workaround was designed for forward-only control and could replace a
reachable distant waypoint with an unreachable point beside the robot. It is
removed for reversible MPPI. Swept-body checks run at the 20 Hz controller and
local-costmap cadence, avoiding repeated full-route checks at 100 Hz.

Full runs snapshot their course, world, launch script, navigation profile,
behavior tree, and dynamics settings. Editing integration inputs during a run
therefore cannot alter its launch or final report. Fifteen ROS regression groups cover
including actual EKF processes with biased wheel yaw/translation and the mapping
rules. All 32 Python tests also pass. Fifteen simulator and fourteen robot patches apply cleanly and remain
detectable on repeated setup; earlier mirror and gyro migrations also pass.
A pinned Nav2 1.1.20 Smac overlay is also built and checked. It preserves
continuous primitive poses and tests the filled footprint, including pure
rotations. Planning uses an immutable map copy so live sensor updates continue
during search; current-map BT guards remain enabled. Unsafe smoothing falls
back to the searched route. The manifest records the Nav2 source and library
hash. See `integrations/nav2_smac_planner/README.md` for the recorded replay.

The intermediate run `runs/full-track-fix-20261005-03/` cleared the original
first-leg stall and traveled into the upper course, but reached 0/5 waypoints
before its 900-second deadline. It retained no referee collision failure and
passed the stop gate. Its position estimate had drifted by over one metre,
prompting the GPS correction. Full-course validation of the final settings is
recorded separately; intermediate progress is not a completed lap.
Attempt `runs/full-track-fix-20261005-07/` traveled 148.26 m around the outside,
but timed out with 0/5 reached. Its physical referee feed had no fallback samples
or collision failure, and its stop gate passed. This exposed the optimistic
FREE canvas; full-course validation of its correction remains pending.
Attempt `runs/full-track-fix-20261005-12/` confirmed that the gyro no longer lost
heading across stops, but the live planner repeatedly failed its one-second
search budget and stopped without progress. Replaying the recorded map found a
route with no lethal cells inside the padded body. The full-course profile now
allows five seconds for search under simulator load, retaining all path guards.
The next diagnostic showed that longer search alone was insufficient: frozen
maps could exhaust the graph, and live paths could overlap newly published
geometry. It was stopped with a passing stop-gate probe before native IMU
validation. These diagnostic attempts reached 0/5 waypoints.
The native-gyro attempt `runs/full-track-fix-20261005-14/` stopped safely at a
padded-footprint planning stall, reaching 0/5 with no collision. PCA scans now
use the LiDAR's horizontal origin in `lidar_planar`, including its near blind
area, and the global map receives direct obstacle marks out to 8 m. Near-field
ray clearing stays within 3 m; the distant exploration penalty is retained.
Attempt `runs/full-track-fix-20261005-15/` passed the stop gate with no
collision, but failed the first-44-feet speed requirement at 0.128 m/s and
reached 0/5. Its bag records 62.8 seconds nearly stopped during the scored
104.39-second interval. The active global guard now checks a 2 m horizon while
new plans still require whole-path validation and measured-body checks remain
immediate. The local 1.5 m gate covers MPPI's maximum 1.4 m rollout. MPPI also
retains 2 m of path, keeping ordinary corridors outside its 1.2 m near-goal
threshold. An approaching-hazard regression verifies controller cancellation.
Strict scoring and calibrated speed limits remain unchanged; this correction
still needs full-course validation.
Attempt `runs/full-track-fix-20261005-16/` stopped without progress at 108.25 s
with 0/5 reached, no collision, and a passing stop gate. Its frozen-map replay
exposed 24 raw poses that the original planner accepted but the filled-body
guard rejected. The corrected replay has zero such overlaps; the full-course
result for the overlay is tracked separately.
Attempt 17 failed readiness after a lifecycle-service response timed out and
never moved. Attempt 18 passed readiness and the stop gate without a collision,
but failed the speed rule at 0.134 m/s, reaching 0/5. Global line layers now
receive complete observations before the rolling local map clips them, keeping
6 m LiDAR tape observations in fixed map-frame memory. They preserve UNKNOWN
outside observed line fields; LiDAR clearing uses `lidar_planar` and retains
blind/unseen marks. Attempt 19 tested a startup survey but failed at 0.126 m/s,
reaching 0/5 without collision and with a passing stop gate. The survey consumed
scored time and was removed. Recorded cone returns during turns exposed mixed
physical poses within individual synthetic scans. Sensor generation now uses
one acquisition pose and timestamp for full/PCA clouds and GPS, and prevents
overlapping scan callbacks. Strict speed scoring and calibrated dynamics are
unchanged. Full-course validation remains pending.

Attempt 20 failed the speed rule at 0.146 m/s, reaching 0/5 with a passing stop
gate and no collision. Far obstacle clearing now records visible free-space
rays out to the 6 m tape-detection horizon; the native obstacle-layer regression
checks occlusions and retention of unseen space. Attempt 21 still stalled at
91.54 s with 0/5. Residual turn-time scan errors matched physical poses
0.17–0.19 seconds before their stamps. The zero-stamped model pose bridge is
now replaced for sensor generation by a native Fortress physical odometry/pose
publisher with acquisition timestamps. This feed is confined to the simulator;
the navigation EKFs still use wheel velocity, gyro, and GPS. Scoring and robot
geometry remain unchanged. Full-course validation remains pending.

Attempt 22 verified native acquisition timing: median cone-surface error was
0.00586 m, with the worst per-frame 90th-percentile error 0.03784 m. It still
failed strict speed scoring at 0.265 m/s, reaching 0/5 with no collision and a
passing stop gate. Attempts 23 and 24 increased footprint padding but stalled;
the shared 0.15 m margin and 1.15 m inflation radius have been restored.
Attempt 24's frozen map reproduced a clear measured body rejected by rasterized
outline cells and start-pose snapping. The Smac overlay now matches the BT's
filled-cell collision rule, preserves continuous start coordinates, and checks
the turn to a safe lattice heading. The returned route includes the measured
pose and initial turn. Full-course validation remains pending; strict scoring,
calibrated speed limits, and all active route guards remain enabled.

Attempt 25 stalled with global hazards absent from the local controller map.
Local control now imports global lethal memory. Global mapping consumes direct
sensor observations and the UNKNOWN canvas rather than the combined local map,
avoiding occupied-cell feedback. Attempt 26 passed that stall but exceeded live
search deadlines farther along the leg. Occupied-row indexing and center-cost
scoring reduced its safe frozen-map search from 3.54 to 1.20 seconds. Current
Nav2 also scores valid Smac nodes by center cost; hard filled-body checks remain
enabled. Full-track waypoint and strict speed completion are still pending.

Attempt 27 improved to 0.375 m/s but still failed strict speed scoring, reaching
0/5 with no collision and a passing stop gate. Planner/controller retry waits
now use zero integer seconds to remove a fixed delay after each rejection;
Humble accepts this duration. The outer recovery still waits one second and
control remains stopped until all active body checks accept a route. Focused
launcher and safety-handoff regressions pass. Full-course validation is pending.

Attempt 28 stalled at 81.55 s after 4.716 m, reaching 0/5 with no collision and
a passing stop gate. Native MPPI regressions reproduced its footprint critic
accepting an interior lethal cell or an inscribed center with a clear outline.
The `autonav_filled_cost_critic` plugin now applies Smac's continuous filled-body
checks with the original MPPI scoring parameters. All 16 ROS test groups and
32 Python tests pass; the 16-package workspace includes the new plugin. Attempt
29 improved to 0.418 m/s but still failed strict speed scoring at 40.27 s,
reaching 0/5 with no collision and a passing stop gate.

MPPI now predicts the unchanged calibrated command delay, response time
constants, and acceleration bounds in every critic's rollout. Its response
regression reproduces an inertial body overlap missed by instantaneous turning;
latency, forward/reverse/stop response, changed speed limits, and command
selection are covered. The workspace builds 17 packages and the suite includes
17 ROS groups plus 32 Python tests. Full-track validation remains pending.

Attempt 30 failed at 0.289 m/s; pre-averaging sample clipping kept maximum
physical speed below 1 mph. The response model now preserves Gaussian samples
for the weighted update and saturates predicted actuator targets instead.
Physical calibration, final command limits, and strict scoring remain unchanged.

Attempt 31 restored 0.50 m/s cruising but failed at 0.394 m/s after guard pauses.
Recorded maps and a native regression identified shifted global-memory marks
in the rolling local map. `mirror_rolling_origin.patch` synchronizes layer and
master origins while preserving world positions, lethal marks, and observed
FREE cells across rolling/resizing. The full ROS suite passes with this change;
the next full-track run retains strict scoring and every body guard.

Attempt 32 failed strict speed at 0.175 m/s after a padded-body overlap caused
a long stall; it reached 0/5 with no collision and a passing stop gate. MPPI now
validates its final bounded, filtered physical rollout. An unsafe blend falls
back to a validated sample in score order, or the existing bounded failure
handling stops control. The new regression reproduces safe sampled turns whose
average collides. All 18 ROS groups and 32 Python tests pass, as do canonical
checks for 15 simulator, 15 robot, and 3 Nav2 patches. Attempt 33 avoided the
long stall but failed at 0.410 m/s, reaching 0/5 with no collision and a passing
stop gate. The profile now uses position-only PathAlign at offset 20
to defer alignment until bearing is established; PathAngle and all body gates
remain enabled. Attempt 34 passed strict speed and reached 4/5, then contacted
cone 9 at 581.11 s on its return leg; the stop gate passed. The local/global
obstacle layers now disable footprint clearing, which otherwise removes real
marks inside the padded body before guards can check them. A native regression
reproduces that deletion and checks preservation with clearing disabled.
Full-track validation of the obstacle-preservation setting remains pending.

Attempt 35 passed strict speed but contacted cone 35 at 61.67 s, reaching 0/5
with a passing stop gate. Surface-only marks left the previously unseen side
of the cone traversable. The obstacle-volume detector now fits round shapes
from classified LiDAR clusters and emits filled footprints for local/global
PointCloud2 marking. It has no course or physical-pose input; insufficient or
non-round clusters preserve their original observations. The cloud is projected
into the upright sensor-origin plane so inverted-mount heights cannot drop
filled marks below the costmap's cutoff. Readiness requires
the marking feed and bags save both its input and output. Focused regressions
and canonical checks pass; full-track validation remains pending.

Attempt 37 avoided contact through the speed interval but failed at 0.421 m/s.
Increasing path alignment in attempt 38 caused a padded-body stall beside
cone 22. The profile restores alignment weight 12 and adds a 0.10 m prediction
reserve beyond the live 0.15 m body padding. Global planning uses matching
0.25 m padding and 1.25 m inflation coverage; live BT guards remain unchanged.
Attempt 39 avoided contact but failed strict speed at 0.198 m/s after 34.78
scored seconds nearly stopped. Its live padded body was clear while the
extra prediction reserve was occupied, rejecting all departures. The extra
reserve now grows in four increments over 0.8 s, with the full live padded
body always checked. The center-cost shortcut based on the smaller live
footprint is disabled; the occupied index still skips empty regions. Sampled
and final MPPI rollouts use the same reserve schedule. Regressions cover safe
departure, stationary rejection, live overlap, and a free-center collision.
Strict full-track validation of this correction is pending.
Attempt 40 failed without progress at 82.70 s with no contact and a passing
stop gate. Its live padded body overlapped cone marks. The controller's pose
was gyro-fused but Humble defaulted its velocity input to raw `/odom`.
The profile now explicitly routes `/local_ekf/odom` to controller_server as
well as the BT and velocity smoother. Recorded native-gyro comparisons show
the fused input reduced mean turn-rate error from 0.0544 to 0.0178 rad/s.
Attempt 41 failed without progress at 79.85 s after 5.089 m with no contact
and a passing stop gate. Its gradual reserve admitted motion into an occupied
live body guard. That experiment is removed. Every predicted pose now checks
the full extra 0.10 m using the corrected occupied-cell check, with fused
controller velocity retained. Attempt 42 will test the full track.
Attempt 42 failed without progress at 77.65 s after 4.337 m, with no contact
and a passing stop gate. Its live padded body was clear while the added
reserve was occupied, blocking all initial rollouts. The extra 0.10 m now
adds a 3000-per-pose clearance cost. The live 0.15 m padded body and inscribed
center remain hard constraints in sampled and final validation. The planner
uses matching 0.15 m padding and retains 1.25 m soft inflation. Extra-only
occupancy raises cost while permitting safe departure; live-body occupancy
still fails the regressions. Attempt 43 will test the full track strictly.
Attempt 43 failed without progress at 83.56 s after 4.328 m, with no contact
and a passing stop gate. The local mirror could retain outdated projected
marks in grid-rasterization holes. The new `retain_source_history: false`
opt-in imports each complete global snapshot afresh after TF succeeds; its
source retains the actual observation history. A regression verifies stale
cells disappear and current and other-layer hazards remain. PathAngle now
corrects bearing above 0.15 rad rather than 1.0 rad. Hard body padding remains
0.15 m. Attempt 44 will test this correction with strict scoring.
Attempt 44 failed without progress at 83.55 s after 5.009 m, with no contact
and a passing stop gate. The sampled-control fallback bypassed the temporal
filter used for normal control. It now filters each candidate using copied
issued-command history, applies bounds, and validates the filtered physical
rollout. Internal retries and rejected candidates preserve issued history;
history advances once when an actual command is returned. The regression
checks a safe smoothed fallback, unchanged prior history, and blocked-body
rejection. All 19 ROS checks and 32 Python tests passed. Attempt 45 passed
the strict first-44-feet speed interval, then failed without progress at
111.93 s after 19.669 m, with no contact and a passing stop gate. Replay
shows occasional lethal cells in the extra navigation padding while the
referee's 0.05 m margin remains clear. Attempt 46 uses matching 0.10 m hard
padding in the planner, controller, and all BT guards, with another 0.15 m
of soft clearance preference. The navigation-geometry check passes; strict
referee scoring and physical calibration are unchanged.

Attempts 46–48 and 50 passed the strict speed interval but failed without
progress. Global obstacle storage, planning, and path guards now use continuous
odom, with native TF conversion of GPS goals. A 100 ms control interval retains
the 2.8 s horizon; recorded commands have 60 ms median and 90 ms 95th-percentile
simulation intervals. The viewer now respects separate path and goal frames.
Attempt 49 failed readiness because its best-effort subscription lost the large
latched map. Reliable transient-local map delivery now passes a late-subscriber
regression while still rejecting stale cached maps. Attempt 50 reached 24.835 m
without contact but failed the no-progress rule at 119.96 s.

Final MPPI validation now requires clear continuing and braking rollouts. The
braking check holds the first command for one control step, then accounts for
the velocity smoother's bounded ramp to zero, command latency, and calibrated
vehicle response. A regression verifies an otherwise safe turn with unsafe
braking clearance is replaced by a filtered alternative that passes both checks.
All 20 ROS checks and 32 Python tests passed the braking change. Attempt 51
passed strict speed scoring and cleared the earlier corner, then oscillated at
a later lattice rotation. It was interrupted for diagnosis at 341.01 s with no
referee violations, 0/5 waypoints, and a passed stop gate. The controller's
turn-stop experiment in attempt 52 failed strict speed scoring at 0.266 m/s
and was removed. PathAngleCritic now optionally scores a nearby lattice turn's
required body heading while retaining full translating lookahead. All 21 ROS
checks and 32 Python tests passed. Attempt 53 passed strict speed scoring and
reached 4/5 without contact, then failed without progress while planning the
final goal. Its requested heading overlapped stored lethal cells; changing only
arrival heading produced a clear recorded-map route. Optional clear goal-heading
selection passed all 21 ROS checks and 32 Python tests, with the actual goal
position unchanged. Attempt 54 passed speed scoring but failed without progress
at 0/5 with a clear body. The critic now ignores lattice alignment turns below
0.30 rad, preserving its normal translating bearing. All 21 ROS checks and 32
Python tests passed the code refinement. Attempt 55 also passed speed scoring
but stalled at 0/5. Attempt 56 restores the critic's 3000 extra-clearance
preference while retaining all hard body/braking checks and strict scoring.
The targeted geometry check passes and the input manifest was refreshed.
Attempt 56 passed speed scoring and reached 4/5 without contact, then failed
without progress while planning the final return. Its exact goal cell blocked
every heading, although a clear arrival cell was only 0.256 m away. Smac now
selects that cell within the existing 0.35 m world-position tolerance and uses
zero additional search tolerance there. The original captured request yields
a 1,238-pose route with zero filled-body overlaps in 2.94 s. The 17-package
build, all 21 ROS checks, all 32 Python tests, and clean/repeated patch
installation passed. Attempt 57 passed strict speed scoring and reached 4/5
without contact, then exhausted the final-return planner budget. A clear
recorded-map route needs 672,597 expansions and takes 4.53 s; the profile now
permits 1,000,000 expansions / 10 s per search. The targeted geometry check
passed and the manifest was refreshed. Attempt 58 passed speed scoring but
stalled at 0/5 when fresh observed marks entered the 0.10 m navigation padding.
Its physical/referee-sized bodies were clear, and native replay finds a safe
departure with 0.05 m padding. Planner/controller/BT now share that unchanged
referee-sized hard margin with 0.20 m extra soft reserve, retaining 0.25 m total
preferred clearance and braking validation. Targeted geometry, critic and
final-control checks passed. Attempt 59 passed speed scoring but contacted cone
032, so the shared 0.10 m hard margin / 0.15 m soft reserve is restored. The
native smoother's wall timer could advance its ramp while simulation time was
paused; both timer sites now use ROS time. MPPI receives actual supervised
command feedback and predicts normal smoother ramps plus queued transport
commands. The 18-package build, all 22 ROS checks, all 32 Python tests, and
clean/repeated patch installation passed. Strict attempt 60 in
`runs/full-track-fix-20261007-60` passed speed scoring but crossed tape while
backing into rear geometry absent from the observed map. The diagnostic profile
now uses forward MPPI commands and lattice paths, retaining in-place turns,
physical dynamics and strict scoring. Targeted geometry checks pass; strict
attempt 61 failed startup after the controller's lifecycle response timed out.
Fresh-container attempt 62 passed speed and reached 4/5 without contact, then
failed the no-progress rule at 266.05 s after 89.953 m. Its stop gate passed.
The recorded return requires 1,288,326 forward-only expansions: native replay
found a 1,132-pose route in 4.57 s with zero filled-body overlaps. The profile
now permits two million expansions with the same ten-second timeout; targeted
geometry checks passed and the manifest was refreshed. Attempt 63 passed speed
and reached 4/5 without contact, then stalled with valid return routes and
failed the no-progress rule at 259.72 s; its stop gate passed. Native calibrated
scoring on the recorded local map shows the 3000 extra-clearance preference
made holding still cheaper than a body/braking-safe departure (17.58 vs 20.76).
Preference 500 makes that departure cheaper (7.82). The profile now uses 500
for the extra 0.15 m reserve, retaining 0.10 m hard padding. Geometry, filled-body
and final-control checks passed. Strict attempt 64 in
`runs/full-track-fix-20261007-64` cleared the return stall but failed near the
finish on a false ramp-proximity departure at 288.23 s, reaching 4/5 after
132.451 m. The stop gate passed and no tape/cone contacts occurred. Its padded
body was wholly outside the deck, about one metre from the side edge. The
referee now checks actual finite side-edge contact under its existing dense
body sweep; five regressions retain real contact failures. Score schema 5
records the geometry correction, retaining speed/contact/no-progress/finish
thresholds. The 18-package build, all 23 ROS checks, all 32 Python tests, and
clean/repeated installation of 17 simulator / 16 robot / 5 Nav2 patches passed.
Strict attempt 65 passed speed and avoided contact, then made return detours
and failed the no-progress rule at 644.05 s after 275.732 m, reaching 4/5;
the stop gate passed. Its captured final request exhausts two million
expansions but succeeds with 3,057,867 expansions / 9.39 s and zero filled-body
overlaps. The profile now permits five million expansions / twenty seconds
while retaining every footprint and referee gate. Targeted geometry checks
passed. Strict attempt 66 passed speed and avoided contact but timed out on
waypoint 5 after 376.391 m / 852.0 s, reaching 4/5; its stop gate passed.
The native controller's wall-time loop was advancing MPPI more frequently than
its ROS-time prediction interval. A native regression reproduced commands while
the simulation clock was paused. `integrations/nav2_controller/ros_clock.patch`
now uses ROS-clock deadlines, handles clock jumps without bursts and permits
cancellation while paused. The stop gate retains its independent steady clock.
The 19-package build, all 25 ROS checks, all 32 Python tests and clean/repeated
installation of 17 simulator / 16 robot / 6 Nav2 patches passed. Strict attempt
67 passed speed and reached 4/5 without contact, then failed no-progress at
495.16 s after 163.292 m; its stop gate passed. The next native correction removes
a center-inflation shortcut that accepted a lethal continuous body corner;
the new inflated-field regression reproduces that failure. The 19-package build,
all 25 ROS / 32 Python checks and clean/repeated installation of 17 simulator /
16 robot / 7 Nav2 patches passed. Strict attempt 68 passed speed and reached
1/5 without contact, then failed no-progress at 189.97 s after 60.288 m; its
stop gate passed. The GPS handler completed waypoint one, but Fast DDS had
timed out sending its acceptance reply, so the mission runner never advanced.

`fastdds_services.xml` sets only the service reply writer's discovery blocking
window to five seconds. The pinned middleware otherwise uses a 100 ms window,
which can expire before the request client's reply reader is discovered.
The installed middleware regression reproduces the old timeout with a reader
discovered after 300 ms, verifies the native five-second setting and a real ROS
request/reply, and checks ordinary topic blocking time is unchanged. The runner
snapshots and hashes the XML with each run; `RMW_FASTRTPS_USE_QOS_FROM_XML` is
not enabled, retaining the pinned memory and publication-mode defaults.
See the [upstream service discovery race](https://github.com/ros2/rmw_fastrtps/issues/842)
and [Humble response implementation](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_response.cpp).
No navigation or scoring threshold changed. A complete strict lap remains unverified.
No complete lap has passed.

## Navigation diagnosis fixes (2026-10-03)

The installed workspace now dispatches each LOCAL/GPS leg through the tracked
`NavigateToPose` action only; corrections still use `/goal_update`. Planner
candidates are committed to the controller blackboard only after footprint
validation succeeds. The active filtered route and measured robot footprint
are checked during planning recovery too, including a 1.5 m local-costmap
lookahead; a failed check halts `FollowPath`.

Line-memory mirrors accept explicit lower nonzero costs from the line layer's
persisted observations, including beside the robot. FREE cells still require
the configured sensor clearing view, and UNKNOWN cells leave memory intact.
The obstacle mirror excludes exact lethal tape cells rather than the entire
soft tape halo, so nearby barrel obstacles remain visible to the planner.

`setup.cmd` applies the integration patches and rebuilds the affected packages.
The existing Docker volume has already been updated. To run the focused ROS
regressions and Python suite in that image, use:

```bash
bash /host/integrations/ros_humble_fortress/verify_navigation.sh
```

The second waypoint passed in both
`runs/compact-navigation-fix-20261003-02/` and
`runs/compact-navigation-fix-20261003-03/`, clearing the diagnosed x ≈ 7 m
stall. The latest attempt still failed at `tape_crossing:right_boundary_3`
on the following leg, with 2/4 waypoints reached. Its stop gate passed.
These fixes do not establish a complete lap.

## What this verifies

This profile uses simulated wheel velocity, gyro rate, GPS, online map harness,
and the LiDAR line detector. The referee alone scores physical truth. It does
not establish real-sensor calibration or CPU camera line and pothole detection.
The CUDA camera detector
is skipped by the x86 image because CUDAToolkit is absent. The compact launcher
selects the upstream compact course; the full-track launcher selects the
exported local `Course.blend` scene. That asset still lacks required semantic
features and fails the semantic release gate. See
`FULL_TRACK_IMPLEMENTATION_STATUS.md` for the latest observed run outcome and
remaining acceptance gates.
