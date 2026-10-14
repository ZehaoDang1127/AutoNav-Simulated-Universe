# Full-track implementation status (2026-10-07)

This records observed work against `FULL_TRACK_AUTONAV_PLAN.md`. Attempt 76
passed the autonomous full local-course diagnostic: **5/5 waypoints, finish,
zero violations, strict speed and stop gates**, after 165.042 m / 378.0 simulated
seconds. The one-click release, complete assets and repeatability suite remain
open. Earlier entries below preserve the evidence from intermediate attempts.

## Full-track failure correction (2026-10-05)

- Corrected stale lethal obstacle mirrors and maximum-cost layer merging.
  Explicit lower observations clear obsolete obstacle marks within the forward
  sensor view; soft tape costs cannot lower another layer's live obstacle cost.
  Tape mirrors accept lower persisted halo costs outside that view, while
  obstacle mirrors retain unseen cones even when a neighbor's halo overlaps.
  Runtime regressions cover these distinct clearing rules.
- Added angular-rate-only synthetic IMU measurements and a local wheel/gyro
  EKF. Added GPS projection and a global EKF to correct translation drift,
  with one owner for each transform. The simulated course starts facing east;
  no absolute physical pose or orientation is fed to these filters.
- Corrected gyro timestamp aliasing exposed by the full run. Finite differences
  use new physics sample timestamps; held samples do not create artificial zero
  readings between rate spikes. A stationary heartbeat keeps the filter fresh
  when Gazebo omits unchanged poses. Wrap, timing, held-pose, and real-EKF tests
  pass, including an upgrade check for the earlier gyro patch.
- Attempt `full-track-fix-20261005-11` contacted `Cones.022` at 64.32 s after
  5.207 m, reaching 0/5; its stop gate passed. The gyro integral and EKF retained
  approximately 16 degrees of heading error after stops. Stationary heartbeats
  had not advanced the finite-difference reference, so the first resumed turn
  was averaged over the entire stopped interval. Heartbeats now advance that
  reference; new physics callbacks publish immediately, with a lock protecting
  the yaw/timestamp pair and derivative state. The stop/resume regression
  reproduced the old 0.00495 rad/s reading for an actual 0.5 rad/s turn.
- Attempt `full-track-fix-20261005-12` had no collision and passed the stop
  gate, but failed for 60 seconds without progress at simulation time 88.35 s,
  reaching 0/5. The recorded gyro integral stayed within about one degree of
  physical heading after the stop; the EKF error was approximately three
  degrees. A replay of the final map found an 806-pose route after 53,152
  expansions, with no lethal cells inside the padded body. The live planner's
  one-second budget was a suspected contributor. The profile now allows five
  seconds for full-course searches under simulator load.
- Attempt `full-track-fix-20261005-13` continued to replan and eventually moved
  through the ramp area, but reached 0/5 before diagnostic interruption. Its
  stop gate passed and no collision was recorded. A live frozen-map replay also
  exhausted the search graph in under one second even with a 15-second budget,
  so extra planning time alone does not resolve the failure. Both raw and
  smoothed paths could overlap newly published lethal geometry; the guards
  rejected them. The bag showed only 586 gyro samples over the previous
  approximately 90-second run, with none arriving late relative to wheel or
  filter timestamps. Sparse pose differences and delayed stationary heartbeats
  allow rate-based prediction to continue turning after a physical stop.
- Added a native Fortress IMU at 50 Hz and a ROS adapter emitting body angular
  velocity only. Full runs save the sensor-equipped URDF and runtime world;
  tests confirm that their robot and course geometry stay identical. The old
  pose-derived feed remains available on a separate diagnostic topic. A native
  Gazebo regression passed turn, stop, and resume at 0.5 rad/s, with continuous
  sampling in every phase. Full-course validation remains pending.
- GPS waypoint dispatch now shares the global filter's `FromLL` projection.
  The independent heading-based projection had moved the first target from
  (32.359, -0.075) to (36.084, 9.216). Tests confirm that changing odometry or
  heading cannot move a cached target, and a replaced action cannot dispatch a
  late projection response. GPS arrival checks and cancellation are retained.
- Matched MPPI and smoother limits to calibrated dynamics and enabled MPPI
  reverse execution to match Smac. Inflated merged tape/obstacle geometry last
  with a radius covering the padded footprint. Corrected Humble's integer Wait
  durations and separated planner/controller recovery action servers.
- Removed the forward-only goal-bending workaround. It could replace the real
  waypoint with an unreachable nearby target after a reverse route appeared.
  The BT now plans directly to the refreshed mission goal and checks swept
  footprints at the controller/local-costmap cadence of 20 Hz.
- Full-run scripts and course/navigation inputs are saved per attempt. This
  avoids changes to a running Bash script and preserves exact runtime inputs.
- Changed the simulator's map canvas from FREE to UNKNOWN. The obstacle mirror
  optionally retains FREE cells in its configured forward clearing sector,
  fills only UNKNOWN master cells, and preserves other layers' obstacles.
  Rolling-window cells outside that sector remain unobserved. This prevents
  unseen space from having a lower cost than the sensed lane solely because
  the canvas predeclared the entire course and its outside margin FREE.
  Still-unobserved planning cells receive an optional cost of 200, below the
  inscribed threshold; the sensor canvas retains UNKNOWN. Humble Smac's obstacle
  heuristic skips cost 255, making the first UNKNOWN-only experiment unable to
  plan the distant goal within its one-second budget. The finite cost preserves
  preference for observed FREE while restoring that heuristic. Runtime merge
  regressions and installation checks pass after this correction.
- Fifteen ROS regression groups cover view-gated FREE persistence, safe merges,
  and the UNKNOWN canvas. The ROS tests include real local and global EKF
  processes under biased wheel yaw and translation. Fifteen simulator and fourteen
  robot canonical patches plus one pinned Nav2 overlay pass clean and repeated installation checks, including
  upgrades from earlier obstacle-mirror and gyro-timing patches.
- All 32 Python tests pass in the pinned Linux container. An initial run hit
  the 0.5-second worker startup timeout in the deadline regression; the isolated
  regression and subsequent complete Python suite passed without code changes.
- Intermediate attempt `full-track-fix-20261005-03` traveled into the upper
  course beyond the original stall, but timed out with 0/5 waypoints. Its referee
  recorded no collision failure and its stop gate passed. Over one metre of
  localization drift motivated the final GPS correction; this intermediate
  attempt does not establish full-lap completion.
- Attempt `full-track-fix-20261005-07` used the corrected GPS projection and gyro
  timing, but traveled 148.26 m around the outside and timed out at waypoint 1
  with 0/5 reached. All 10,845 referee odometry samples used the physical primary
  feed; there were no collision failures, and the stop gate passed. This run
  motivated the UNKNOWN/free-memory correction. A complete lap remains pending.
- Attempt `full-track-fix-20261005-08` was stopped after repeated planning
  failures exposed the UNKNOWN heuristic limitation. Container interruption
  cleaned up ROS before its final probe, leaving referee fields unavailable and
  the stop gate unverified. Its failed result and diagnostic note are preserved.
- Attempt `full-track-fix-20261005-09` produced routes with finite exploration
  costs but contacted `Cones.022` at 72.54 s after 6.974 m, reaching 0/5. The stop
  gate passed. Recorded maps show the cone disappearing outside the sensor view
  when soft inflation from a neighbor retired its remembered mark. The obstacle
  mirror now disables `decrease_nonzero_outside_view`; a regression preserves an
  unseen lethal cone while allowing lower observations within the front sector.
- Attempt `full-track-fix-20261005-10` avoided cone 22 but contacted `Cones.003`
  at 84.36 s after 8.49 m, reaching 0/5; the stop gate passed. The bag exposed an
  offset viewing-sector mismatch during a tight turn: at 74.36 s the cone was
  60.53 degrees from nav_center but 75.74 degrees from the lidar, outside its
  70-degree half-view. Clearing now uses the actual `lidar_footprint` transform
  and fails closed when that transform is unavailable.
- Planning, control, and recoveries now use the drive axle (`base_link`) to match
  DiffDrive velocity and rotation kinematics. The footprint stays the same full
  physical body, expressed as x=-0.32..0.77 m and y=+-0.41 m about that axle.
  Padding is 0.15 m for the measured pose/projection error, with inflation radius
  1.15 m covering its maximum axle-relative extent. All three BT gates share
  this footprint and frame; runtime and physical-profile regressions pass.
- Attempt `full-track-fix-20261005-14` used the native gyro, measured at 50 Hz
  with 20 ms intervals in a live probe. Local/global heading errors were about
  0.6/1.4 degrees after a turn. It still failed without progress at 97.39 s,
  reaching 0/5; no collision occurred and the stop gate passed. Its estimated
  padded body contained seven lethal cells, while the same body without
  extra padding contained none. Smac correctly rejected that starting pose.
- Corrected both PCA scan projections to an upright `lidar_planar` frame at
  the actual sensor's horizontal mount. Axle-origin rays could falsely observe
  a near-side cone after it moved behind the forward-mounted sensor. A runtime
  converter regression verifies sensor-relative range and excludes that blind
  area. The obstacle mirror uses the same frame and 140-degree viewing sector.
- Added direct PCA obstacle observations to the global map out to 8 m, beyond
  the 6 m local window, so the planner can see upcoming cone geometry sooner.
  Near-field ray clearing remains limited to 3 m; unobserved distant lane space
  retains its exploration penalty. Full-course validation remains pending.
- Attempt `full-track-fix-20261005-15` traveled through the ramp without a
  collision but failed the first-44-feet speed rule at 119.77 s: 0.128 m/s
  average, reaching 0/5. The stop gate passed. Recorded physical velocity
  shows 62.8 seconds nearly stopped out of 104.39 scored seconds. Active-route
  guards repeatedly cancelled motion as farther map geometry changed.
- Retained whole-path validation for candidate plans and measured-body checks
  on every controller tick. The active global guard now checks the next 2 m,
  with the local guard retaining 1.5 m; a runtime regression verifies that an
  approaching hazard halts control before its next tick. MPPI keeps 2 m of
  route, exceeding its maximum 1.4 m rollout and its 1.2 m near-goal threshold.
  The previous 1.2 m cutoff could disable its path-following incentive on an
  ordinary corridor. Strict scoring and calibrated speed limits are retained;
  a passing full-course speed check remains unverified.
- Attempt `full-track-fix-20261005-16` reached 9.166 m, then failed without
  progress at 108.25 s, reaching 0/5. It recorded no collision and passed the
  stop gate. A frozen-map search reproduced 24 raw poses containing lethal
  cells inside the padded body. Nav2's checker lost fractional pose/yaw and
  checked only the footprint outline; increasing angle bins alone did not
  resolve the mismatch.
- Added the pinned Humble Smac source overlay in `integrations/nav2_smac_planner`.
  It checks the filled body at continuous primitive poses, including rotations,
  keeps a per-search occupied-bounds cache, and falls back to the searched route
  when smoothing violates footprint clearance. It copies the search map before
  releasing the live costmap lock so sensors remain fresh during planning.
  The same frozen input now yields 921 poses with zero lethal body overlaps in
  4.51 seconds. Full-course validation of this overlay remains pending.
- Attempt `full-track-fix-20261005-17` never moved: the controller's lifecycle
  configure response timed out, leaving navigation inactive. Readiness failed
  and the stop gate passed. The same inputs were retried in attempt 18.
- Attempt `full-track-fix-20261005-18` traveled 13.797 m without collision,
  but failed the first-44-feet rule at 112.87 s with 0.134 m/s average, reaching
  0/5; its stop gate passed. Physical velocity reached the calibrated limit,
  but 65.28 of the 100.15 scored seconds were nearly stopped. The initial
  provisional route went south before turning north; farther tape observations
  were clipped by the 6 m rolling local costmap before the global mirrors saw them.
- Global camera/LiDAR line layers now consume the complete point observations,
  preserving the detector's 6 m view and fixed map-frame memory. They preserve
  UNKNOWN outside observed line fields and clear only in the sensor view. A
  runtime test covers far marks, stale visible clearing, unseen/blind-area
  retention, and preservation of another layer's obstacle. The local LiDAR
  layer uses the same sensor-origin clearing frame.
- Attempt `full-track-fix-20261005-19` tested the complete line observations
  and a startup survey. It failed the speed rule at 113.77 s with 0.126 m/s
  average, reaching 0/5 without collision; the stop gate passed. Physical
  movement during the survey started strict speed scoring. The survey was
  removed from the production BT after it consumed scored time without
  improving the initial route.
- Recorded PCA cone returns during turns were displaced by more than one metre
  from their physical surfaces at the scan timestamp. Synthetic scan generation
  read changing physical poses concurrently throughout ray casting and point
  conversion. `sensor_pose_snapshot.patch` captures one pose and acquisition
  timestamp for the full scan and PCA output, releases the lock before ray
  casting, prevents overlapping scan timer callbacks, and stamps GPS with its
  sampled pose time. Regressions advance the physical pose during generation
  and check that both cone and floor returns retain the acquisition geometry.
  Strict scoring and calibrated speed limits are unchanged. Full-course
  validation of this correction remains pending.
- Attempt `full-track-fix-20261006-20` used the sensor snapshots and reached
  0/5, failing the speed check at 110.09 s with 0.146 m/s average. No collision
  was recorded and the stop gate passed; 56.30 of 92 seconds were nearly stopped.
  The next profile records unoccluded free-space rays through the 6 m tape
  observation horizon, replacing the old 3 m clearing limit. A native obstacle
  layer regression verifies occlusion, bounds, unseen-space retention, fixed-map
  persistence, and preservation of another layer's obstacle.
- Attempt `full-track-fix-20261006-21` tested the longer free-space rays but
  failed after 60 seconds without progress at 91.54 s, reaching 0/5 without
  collision and with a passing stop gate. This does not establish a speed pass.
- Remaining large cone-return errors in attempt 20 matched physical poses
  0.17–0.19 s before the scan stamp, with residual median errors of 0.009–0.026 m.
  The bridged model entries in `Pose_V` carry zero timestamps; the harness had
  assigned receipt time to delayed physical poses. The new
  `physics_pose_timestamp.patch` uses Fortress's physical `OdometryPublisher`
  and its stamped pose message for synthetic sensor placement. It retains world
  coordinates and acquisition time, preserves robot/course geometry, and is
  confined to simulation sensor generation and diagnostic topics. Local/global
  navigation EKFs continue to consume wheel velocity, gyro, and GPS only.
  Full-course validation remains pending.
- Attempt `full-track-fix-20261006-22` verified the timestamp correction:
  recorded scan poses matched physical acquisition time within one 20 ms sample,
  with a median cone-surface error of 0.00586 m and worst per-frame 90th-percentile
  error of 0.03784 m. A live 748-pose route contained no lethal body overlaps.
  It still failed strict speed scoring at 68.38 s with 0.265 m/s average and
  0/5 waypoints; 18.34 of 50.6 scored seconds were nearly stopped. No collision
  occurred and the stop gate passed. Native gyro samples were not late relative
  to filter timestamps, so delayed IMU fusion was not supported by this bag.
- Attempts `full-track-fix-20261006-23` and `-24` tried 0.25 m padding, first
  for planning alone, then consistently for planning, control, and BT guards.
  Both failed after 60 seconds without progress, at 121.06 and 123.89 s,
  respectively, with 0/5 and passing stop gates. The experiment was reverted
  to the shared 0.15 m margin and 1.15 m stock inflation radius.
- Attempt 24's frozen map reproduced two false lethal-start conditions:
  rasterized outline cells could lie outside the filled padded polygon, and
  snapping the measured pose to the lattice cell/heading could introduce an
  overlap. The Smac overlay now uses the same filled-cell hard rule as the BT,
  retains continuous start coordinates, and checks the entire initial rotation
  before choosing a discrete heading. The measured pose is included in the
  returned path. Regressions preserve lethal interior, center inflation,
  unknown-space, and rotation-sweep rejection. Full-track strict speed and
  waypoint completion remain unverified for these changes.
- Attempt `full-track-fix-20261006-25` failed without progress at 90.43 s
  after 6.083 m, reaching 0/5 with no collision and a passing stop gate.
  Scan acquisition remained accurate (0.00094 m median cone-surface error).
  Its final global map had 17 lethal cell centers inside the extra padded body,
  while its local controller map had none. Both maps had no lethal cells inside
  the unpadded physical body. Local control now imports global lethal memory;
  global mapping receives direct sensor observations and no longer imports the
  combined local costmap. The UNKNOWN canvas supplies the finite exploration
  penalty without occupied-cell feedback. Focused runtime/configuration tests
  cover rear remembered hazards and retirement of an authoritative source mark.
- Attempt `full-track-fix-20261006-26` passed the previous stall but failed
  without progress at 118.61 s with 0/5. Its final local/global bodies were clear,
  and the frozen map yielded a safe route in 3.54 seconds. Live searches exceeded
  their five-second budget. Occupied-row indexing preserves 1,024 regression
  collision decisions; center-cost scoring avoids charging inflated footprint
  edges a second time. The same frozen search now takes 1.20 seconds with zero
  lethal overlaps. All 15 ROS groups and 32 Python tests passed after indexing;
  the subsequent center-cost change passed focused Smac/geometry and complete
  clean/repeated patch checks. Strict full-course validation is in progress.
- Attempt `full-track-fix-20261006-27` traversed the speed interval without
  collision but failed at 62.71 s with 0.375 m/s average, reaching 0/5 and
  passing the stop gate. Of 35.78 scored seconds, 4.15 were nearly stopped;
  repeated active-guard cancellations invoked fixed one-second controller waits.
  Local planner/controller retries now use zero integer-second Wait durations
  (supported by Humble), while the outer recovery wait remains one second.
  Controller retries are bounded at 32 so prompt retries can wait for a valid
  new route before escalating. All candidate and active body checks remain
  enabled; launcher, geometry, and unsafe-path-handoff regressions pass.
- Attempt `full-track-fix-20261006-28` failed without progress at 81.55 s
  after 4.716 m, reaching 0/5 with no collision and a passing stop gate.
  Removing retry delays alone did not resolve the stall. The controller's
  original footprint critic accepted lethal cells inside the body and an
  inscribed center when its rasterized outline was clear; both mismatches
  were reproduced in native MPPI regression fixtures.
- The new `FilledCostCritic` applies the same continuous filled-body and
  center checks as Smac while preserving the original MPPI weights, collision
  penalty, and unknown-space policy. Its occupied-cell cache refreshes under
  the controller's existing costmap lock. Plugin discovery, rejection of both
  mismatches, selection of a safe alternative, and cache clearing pass.
  The workspace builds 16 packages; all 16 ROS groups and 32 Python tests pass.
  Attempt `full-track-fix-20261006-29` improved strict speed to 0.418 m/s but
  still failed the 0.44704 m/s rule at 40.27 s, reaching 0/5. No collision
  occurred and the stop gate passed. Its 32.08-second scored interval included
  1.56 seconds nearly stopped and 3.71 seconds below 0.3 m/s.
- Attempt 29's half-second recorded-command replay exposed a physical response
  mismatch: mean position/yaw error was 0.041 m / 0.114 rad with instantaneous
  response and 0.020 m / 0.047 rad with the calibrated delay, time constants,
  and acceleration limits. An inertial turn regression reproduces a filled-body
  collision that instantaneous prediction misses. MPPI now offers a calibrated
  differential-drive model using the unchanged vehicle response parameters;
  every trajectory critic receives that prediction. The selected command
  begins at the first scored action because delay is explicitly modeled.
  The workspace builds 17 packages; 17 ROS groups and 32 Python tests pass.
  Strict full-track validation of this response model remains pending.
- Attempt `full-track-fix-20261006-30` failed strict speed at 0.289 m/s,
  simulation time 61.67 s, reaching 0/5 with no collision and a passing stop
  gate. Maximum speed was only 0.443 m/s. Clipping Gaussian perturbations before
  the MPPI weighted update introduced an inward command bias. The corrected
  response model preserves those perturbations and saturates predicted actuator
  targets; final issued commands retain their original bounds. Response,
  sampling, command-selection, and collision regressions cover this correction.
- Attempt `full-track-fix-20261006-31` restored 0.50 m/s cruising but failed
  the speed interval at 0.394 m/s and 40.13 s, reaching 0/5 with no collision
  and a passing stop gate. It spent 5.39 seconds nearly stopped. At those early
  rejections the latest global route was clear, but local lethal cells intersected
  it; their corresponding global cells were FREE or exploration cost, with
  nearest global lethal marks as far as 2.5 m away. Neither local line layer
  owned those first extra marks.
- A native rolling-map regression reproduced the cause: `LocalMirrorLayer`
  retained its initial grid origin while the local master rolled, painting
  remembered global hazards at displaced world positions. The layer now
  synchronizes its origin before updates and preserves cells by world position.
  The existing resize merge also treated UNKNOWN as greater than lethal and
  discarded observed FREE memory; both preservation cases are corrected.
  Forward/reverse rolling shifts and resizing regressions pass, together with
  all 17 ROS groups. Canonical setup checks pass for 15 simulator, 15 robot,
  and 2 Nav2 patches. Strict full-course validation is pending.
- Attempt `full-track-fix-20261006-32` failed strict speed at 0.175 m/s and
  83.37 s, reaching 0/5 with no collision and a passing stop gate. Its padded
  body overlapped a newly observed lethal cell near 4.4 m of progress and
  stalled for about 37 seconds. The unpadded physical body remained clear.
- The controller previously validated sampled trajectories but issued their
  weighted and filtered control sequence without checking its final rollout.
  A native regression reproduces two safe turns whose averaged straight path
  collides. The calibrated controller now bounds and validates the final
  sequence, substitutes the best validated sample if necessary, and uses its
  existing bounded failure handling when none is safe. Filter history records
  the command actually issued. All 18 ROS groups and 32 Python tests pass;
  canonical checks pass for 15 simulator, 15 robot, and 3 Nav2 patches.
  No full-lap pass has been established.
- Attempt `full-track-fix-20261006-33` avoided the long stall but failed strict
  speed at 0.410 m/s and 37.53 s, reaching 0/5 with no collision and a passing
  stop gate. Its 32.71 scored seconds included 2.05 nearly stopped, 3.41 below
  0.3 m/s, and 6.05 between 0.3 m/s and 1 mph; cruising reached 0.50 m/s.
  The initial turn and later heading corrections remain costly. The profile
  now delays PathAlign to offset 20 and scores position alone;
  PathAngle retains bearing control and all final/body collision checks stay
  enabled.
- Attempt `full-track-fix-20261006-34` passed the strict speed interval and
  reached 4/5 waypoints. It failed on the return leg at 581.11 s after
  147.148 m with `obstacle_contact:Cones.009`; the stop gate passed. All
  11,056 referee odometry samples used the primary physical feed. This is
  progress beyond the speed failure, not a complete lap.
- Both contact maps reported a clear padded body. Native ObstacleLayer
  defaults erase obstacle marks inside its configured padded footprint before
  merging costs. A regression reproduces deletion of a real mark in the extra
  padding and preserves it over repeated updates when footprint clearing is
  disabled. Local/global obstacle layers now disable this operation; the
  synthetic PCA feed excludes the robot already. Sensor ray clearing remains
  enabled. All 18 ROS groups and 32 Python tests pass, including repeated
  padded-body mark preservation and profile checks. Attempt 35 is testing
  this correction under the same strict referee rules.
- Attempt `full-track-fix-20261006-35` also passed strict speed, but contacted
  `Cones.035` at 61.67 s and 22.483 m, reaching 0/5; its stop gate passed.
  Footprint preservation alone does not resolve the cone-contact failure.
  Its retained map marks cover the cone's previously visible face rather than
  its solid interior and unseen side, leaving a body approach from the opposite
  side unchecked.
- `obstacle_volume.patch` adds a detector consuming classified LiDAR points
  only. It clusters surface returns, fits sufficiently conditioned circular
  shapes, and marks their filled footprints through PointCloud2 observations.
  Radius, center, and fit quality come from the observations; sparse, straight,
  ambiguous, or non-round clusters retain only their original returns. The
  original sensor-origin 140-degree sector, acquisition header, and ray clearing
  are retained. Readiness now requires this marking feed and bags save its input,
  output, clearing scan, and native IMU. Four focused regressions and clean/
  repeated installation checks pass for 16 simulator, 15 robot, and 3 Nav2
  patches. The 17-package build, all 19 ROS groups, and 32 Python tests pass.
  Attempt 36 contacted `Cones.022` at 13.67 s, reaching 0/5; its stop gate
  passed. Its new direct cloud exposed a height-projection mistake: fitted
  points from the inverted sensor could fall below the obstacle height cutoff.
  The detector now transforms into `lidar_planar`, applies the original height
  and viewing filters, and projects all marking footprints to z=0. Acquisition
  timestamps are preserved. The runtime regression covers the actual inverted
  sensor mount and verifies that the published marks lie on the ground plane.
  The 17-package build, 19 ROS groups, 32 Python tests, and complete canonical
  patch checks pass. Attempt 37 is testing the corrected projection with
  strict scoring; it failed the speed interval at 0.421 m/s and 41.12 s,
  reaching 0/5 without contact and with a passing stop gate. Its 31.86 scored
  seconds included 2.35 nearly stopped and two guard-induced turn pauses.
  The next profile trial raises positional PathAlign weight from 12 to 20
  to reduce corner cutting; body checks, scoring, and calibrated bounds stay
  enabled. The trial's full-track result is pending.
- Attempt `full-track-fix-20261006-38` stalled beside cone 22 and failed
  without progress at 78.71 s, reaching 0/5 without contact; the stop gate
  passed. A live frozen map contains nine lethal cells inside the extra
  padded body and none inside the physical body. Native gyro sampling stayed
  at 50 Hz with 20 ms maximum gaps, and pose errors were small.
- The profile restores PathAlign weight 12 and reserves 0.10 m beyond the
  live 0.15 m padding in FilledCostCritic's sampled and final rollouts. Smac
  plans with matching 0.25 m padding and 1.25 m inflation coverage. The live
  controller footprint and all BT body guards retain their 0.15 m margin.
  A scoring regression rejects a candidate inside the prediction reserve
  while retaining a safe alternative; the geometry check ties both reserves
  together. The 17-package build, 19 ROS groups, 32 Python tests, and all
  canonical patch checks pass.
- Attempt `full-track-fix-20261006-39` failed strict speed at 86.60 s with
  0.198 m/s average, reaching 0/5 without contact; its stop gate passed. It
  spent 34.78 scored seconds nearly stopped. Indexed bag replay shows the
  physical and live padded bodies clear throughout the main stall, while
  the extra prediction reserve contains one or two lethal cells. Requiring
  that full extra reserve at the initial predicted pose rejected every
  departure. The native center-cost shortcut also uses the live footprint's
  smaller circumscribed radius and can skip an enlarged-footprint collision.
- FilledCostCritic now grows its optional extra reserve in four increments
  over 0.8 s, retaining the full live padded body at every pose and the full
  extra reserve thereafter. Sampled and final control validation use the
  same schedule. It disables the mismatched center-cost shortcut and retains
  the occupied-row empty-region shortcut. Regressions cover a free-center
  reserve collision, a safe departure, a stationary rejected rollout, and
  a lethal live-body overlap. The 17-package build, all canonical patch
  checks, 19 ROS groups, and 32 Python tests pass. Attempt 40 is running the
  full track with strict scoring.
- Attempt `full-track-fix-20261006-40` failed the no-progress rule at 82.70 s
  after 4.429 m, reaching 0/5 without contact; its stop gate passed. Indexed
  replay confirms lethal cells inside the live padded body beside cone 22,
  so the growing extra reserve cannot resolve that overlap. The controller
  server had no explicit `odom_topic`: Humble's OdomSubscriber therefore
  used raw `/odom` velocity while its pose came from the gyro-fused EKF TF.
  At sampled native-gyro acquisition times from 8–24 s, wheel turn-rate error
  averaged 0.0544 rad/s and peaked at 0.1631; fused error averaged 0.0178 and
  peaked at 0.0578. The controller now explicitly uses `/local_ekf/odom`,
  matching its pose and the BT odometry. The geometry regression checks
  that connection.
- Attempt `full-track-fix-20261006-41` failed the no-progress rule at 79.85 s
  after 5.089 m, reaching 0/5 without contact; its stop gate passed. The live
  padded body overlapped one or two tape cells while the full prediction
  reserve overlapped several more. The gradual-reserve experiment is removed:
  the full extra 0.10 m now applies at every predicted pose, with the corrected
  occupied-cell check and fused velocity input retained. This avoids admitting
  an initial motion under a smaller envelope than the planner requires.
  Attempt 42 will retest the full track after rebuilding and verification.
- Attempt `full-track-fix-20261006-42` failed the no-progress rule at 77.65 s
  after 4.337 m, reaching 0/5 without contact; its stop gate passed. A live
  probe confirms controller_server subscribes to `/local_ekf/odom`. Indexed
  replay shows zero lethal cells in either live padded body during the stall,
  but two to four inside the added 0.10 m envelope. Making that envelope a
  hard condition again blocked every initial rollout and the planner.
- The added 0.10 m now contributes a 3000-per-pose clearance cost. Live
  padded-body and inscribed-center collisions remain hard failures; sampled
  and final control validation check the same hard body. Global planning
  again uses the matching 0.15 m hard padding, retaining 1.25 m soft inflation.
  Regressions require extra-only occupancy to raise cost while allowing safe
  departure, and require live-body occupancy to fail. Attempt 43 will retest
  the full track after rebuilding and verification.
- Attempt `full-track-fix-20261006-43` failed the no-progress rule at 83.56 s
  after 4.328 m, reaching 0/5 without contact; its stop gate passed. The
  initial stall contains three local lethal cells inside the 0.15 m padding
  and none inside the scored 0.05 m footprint. While stopped, imported local
  marks sometimes grow beyond the current global projection. The local
  mirror retained previous projected cells, including rasterization holes
  left by changed transforms or source grids.
- A `retain_source_history: false` opt-in now rebuilds the local import from
  each successfully transformed complete global-map snapshot. The global
  source retains observed hazards; local sensor memory and other layers
  retain their existing behavior. A coarser-source regression reproduces
  unvisited destination cells and verifies obsolete imported marks disappear,
  new hazards remain, and other-layer hazards are preserved. PathAngle's
  correction threshold changes from 1.0 to 0.15 rad to reduce body drift at
  cone edges. Hard padding remains 0.15 m throughout. All 19 ROS checks and
  32 Python tests passed before attempt 44.
- Attempt `full-track-fix-20261006-44` failed the no-progress rule at 83.55 s
  after 5.009 m, reaching 0/5 without contact; its stop gate passed. The
  snapshot import and earlier bearing correction did not prevent another
  padded-body stall.
- Final-command fallback had a separate smoothing path: the mean was
  temporally filtered, but sampled fallback sequences were issued raw. Each
  sampled candidate now receives the same filter, bounds, and physical-body
  revalidation. Filters use copied issued-command history; rejected candidates
  and internal retries cannot advance it. History advances once for the
  command actually returned. The regression requires a safe filtered fallback
  whose initial angular command is below its raw sample, preserves previous
  issued history during selection, and still rejects a fully blocked body.
  All 19 ROS checks and 32 Python tests passed before attempt 45.
- Attempt `full-track-fix-20261006-45` passed the strict first-44-feet speed
  interval, then failed the no-progress rule at 111.93 s after 19.669 m,
  reaching 0/5 without contact. Its stop gate passed. Replay shows occasional
  lethal cells inside the extra 0.15 m navigation padding while the referee's
  0.05 m margin remains clear. Hard navigation padding is now 0.10 m in both
  costmaps and all three BT gates, with a 0.15 m additional soft reserve to
  retain the 0.25 m preferred clearance. Referee scoring and physical
  calibration are unchanged. Navigation-geometry checks pass; attempt 46
  tested this with strict scoring.
- Attempt `full-track-fix-20261006-46` passed the strict speed interval but
  failed without progress at 112.27 s after 19.875 m, reaching 0/5 without
  contact; the stop gate passed. Stored global obstacle cells used the
  GPS-corrected map frame, and clearance changed while the robot was stationary.
  Global obstacle storage, planning, BT navigation, and all path guards now
  use continuous `odom`. GPS goals retain their map frame and the planner's
  native TF conversion. Geometry and path-handoff checks pass.
- Attempt `full-track-fix-20261006-47` passed the strict speed interval and
  the previous stall, then failed without progress at 124.73 s after 26.188 m,
  reaching 0/5 without contact; the stop gate passed. The turn moved from
  roughly 1.04 to 0.50 rad, entering the hard margin beside a cone. The
  controller repeatedly missed its 50 ms deadline. Attempt 48 uses a 100 ms
  control interval and 28 steps, preserving the 2.8 s prediction horizon.
  Geometry and all six launcher checks pass. Raw `/cmd_vel_nav` is now recorded
  to measure actual command cadence. The viewer preserves path/goal frames
  and aligns map and odom independently; the live frame probe and viewer syntax
  checks pass.
- Attempt `full-track-fix-20261006-48` passed the strict speed interval but
  failed without progress at 116.60 s after 25.257 m, reaching 0/5 without
  contact; the stop gate passed. Raw command cadence was 0.060 s median and
  0.090 s at the 95th percentile in simulation time, with 2% above 0.110 s.
  The same corner entered the hard padding after a turn away from the route.
  PathAngle's initial-heading threshold disabled future heading costs while
  initially aligned. Attempt 49 uses a zero threshold, weight 6, and clearance
  cost 9000 per affected pose. Geometry checks pass.
- Attempt `full-track-fix-20261006-49` did not start a mission because readiness
  never received `/map_padded`; the stop gate passed. The readiness map
  subscription now matches its reliable, transient-local publisher instead of
  best-effort sensor QoS. A late-subscriber check receives the full 880x920
  cached map without republishing; freshness requirements are unchanged.
  The persistent readiness regression also checks stale cached-map rejection.
- Attempt `full-track-fix-20261006-50` passed readiness and the strict speed
  interval, then failed without progress at 119.96 s after 24.835 m, reaching
  0/5 without contact; the stop gate passed. Increased heading/clearance costs
  did not prevent the corner stall. Final control validation now checks both
  the continuing plan and a braking path that holds the first command for one
  control interval, then ramps commands to zero using the matched command
  smoother limits before the calibrated latency and vehicle response. A
  filled-body regression places a cell clear of the continuing turn but inside
  its braking sweep; the command must be rejected without replacing the planned
  sequence, while still permitting a filtered alternative with both clear
  rollouts. The 17-package build, all 20 ROS checks, and 32 Python tests pass.
  Attempt 51 passed the strict speed interval and cleared the repeated x=13 m
  corner. It later oscillated near (25.3, -10.4), pursuing positions beyond a
  required lattice rotation. Diagnostic interruption at 341.01 s retained no
  referee violations, 0/5 waypoints, and a passed stop gate; it is not a lap pass.
- Attempt 52's stop-at-every-lattice-turn experiment passed 21 ROS and 32 Python
  tests but failed the strict first-44-feet speed rule at 106.07 s, averaging
  0.266 m/s. It reached 0/5 and passed the stop gate. The experiment was removed.
  PathAngleCritic instead optionally scores the required body heading of a
  nearby stationary yaw sequence while preserving full translating lookahead.
  It retains the original bearing score on other paths, including distant
  turns. The native heading regression checks required rotations, asymmetric
  reverse alternatives, duplicates, wrapped yaw, and near-goal turns. The
  17-package build, all 21 ROS checks, all 32 Python tests, and clean/repeated
  patch installation passed. Attempt 53 passed strict speed scoring and reached
  4/5 without contact, then failed the no-progress rule at 388.22 s after
  93.551 m; its stop gate passed. The final goal's requested body heading
  intersected stored lethal cells. A frame-correct replay exhausted 500,000
  expansions with that heading but found a 1,564-pose clear route in 0.28 s /
  18,130 expansions after changing only arrival heading. Smac now optionally
  chooses a clear arrival heading at the same goal cell; the GPS profile enables
  it because its goal checker accepts any yaw. Other profiles retain their
  heading constraint by default. The 17-package build, all 21 ROS checks, all
  32 Python tests, and clean/repeated patch installation passed. The native
  heading selector reproduces the clear route in 0.33 s with zero filled-body
  overlaps. Attempt 54 passed the strict speed interval but failed without
  progress at 131.86 s after 27.155 m, reaching 0/5 without contact; its stop
  gate passed. Recorded local/global bodies and preferred margins were clear.
  Tiny lattice-bin alignment turns were being scored as stationary goals while
  the path curved ahead. The critic now ignores rotations below 0.30 rad and
  preserves the original translating bearing there. Larger required rotations
  retain their explicit body heading. The 17-package build, all 21 ROS checks,
  all 32 Python tests, and clean/repeated patch installation passed.
  Attempt 55 also passed speed scoring but failed without progress at 119.55 s
  after 26.688 m, reaching 0/5 without contact; its stop gate passed. Excluding
  tiny alignment turns alone did not resolve the corner. The extra-clearance
  preference is restored from 9000 to the critic's default 3000 per affected
  pose, testing whether the stronger soft preference outweighed progress on
  clear-body turns. Hard body/braking checks, padding, speed caps, and referee
  scoring are unchanged. The targeted geometry check passed and the workspace
  manifest was refreshed. Attempt 56 passed strict speed scoring and reached
  4/5 without contact, then failed without progress at 245.79 s after 83.073 m;
  its stop gate passed. Every heading at its final goal cell was blocked, but
  a filled-body-clear cell existed 0.256 m from the requested target, inside
  the existing 0.35 m planner tolerance. Smac now explicitly selects the
  nearest clear arrival cell within that tolerance when all exact-cell
  headings are blocked. Search uses zero additional tolerance around a moved
  cell, preventing compounded tolerance. The unchanged captured request now
  produces a 1,238-pose route in 2.94 s / 415,862 expansions with zero filled-body
  overlaps and a final endpoint inside the original tolerance. The 17-package
  build, all 21 ROS checks, all 32 Python tests, and clean/repeated patch
  installation passed. Attempt 57 passed strict speed scoring and reached
  4/5 without contact, then failed without progress at 276.24 s after 88.249 m;
  its stop gate passed. Its final goal body was clear, but the original search
  exhausted 500,000 expansions. With a larger diagnostic budget, the same
  request produces a 1,237-pose route in 4.53 s / 672,597 expansions with zero
  filled-body overlaps. The profile now permits 1,000,000 expansions and 10 s
  per search. Geometry, collision checks, mission timeout, and referee limits
  are unchanged. The targeted geometry check passed and the manifest was
  refreshed. Attempt 58 passed strict speed scoring but stopped at 0/5 when
  two newly observed lethal cells entered its 0.10 m navigation padding. It
  failed without progress at 181.97 s after 47.576 m; its stop gate passed.
  The physical body and referee-sized 0.05 m margin were clear on both observed
  maps. Native replay rejects the 0.10 m start but produces a 268-pose clear
  departure in 0.43 s / 85,500 expansions with 0.05 m padding. Planner,
  controller, and all three BT guards now share the referee-sized 0.05 m hard
  margin. The extra soft reserve increases from 0.15 to 0.20 m, preserving
  0.25 m total preferred clearance and its 3000 cost. Braking checks and strict
  referee scoring are unchanged. The targeted geometry, filled-critic and
  final-control/braking checks passed; the manifest was refreshed. Strict
  full-track attempt 59 passed speed scoring but contacted cone 032 at 75.44 s
  after 25.104 m, reaching 0/5; its stop gate passed. The smaller hard margin
  is rejected: the shared 0.10 m hard margin / 0.15 m soft reserve is restored.
  Its command trace showed smoother increments exceeding the configured ramp
  per simulated second. The native smoother used wall timers with fixed ramp
  increments; a regression reproduced seven outputs while ROS time was paused.
  Both native timers now use the ROS clock. MPPI also receives supervised
  command feedback, holds a per-evaluation snapshot for queued commands during
  latency, and predicts normal commands through the 20 Hz smoother before
  actuator response. Stale feedback cannot reapply an old turn. The 18-package
  build, all 22 ROS checks, all 32 Python tests, and clean/repeated installation
  of 16 simulator / 16 robot / 5 Nav2 patches passed. Strict full-track attempt
  60 passed speed scoring but crossed tape while reversing at 151.55 s after
  46.867 m, reaching 0/5; its stop gate passed. The observed local/global maps
  had no lethal cells in even the preferred body at the crossing. Recorded
  commands backed at 0.1–0.2 m/s into rear geometry outside the forward sensor
  view. The diagnostic profile now uses nonnegative MPPI linear commands,
  forward path-angle preference, and forward-only lattice expansion; in-place
  turns remain available. Physical dynamics and strict scoring are unchanged.
  The targeted geometry check passed and the manifest was refreshed. Strict
  full-track attempt 61 did not start a mission: the controller configured,
  but its lifecycle-service response timed out and navigation stayed inactive.
  The stop gate passed. Fresh-container attempt 62 passed readiness and speed,
  reached 4/5 without contact, then failed the 60-second no-progress rule at
  266.05 s after 89.953 m; its stop gate passed. The recorded forward-only return
  request reproduced the one-million-expansion cap. A larger native replay
  found a 1,132-pose route in 1,288,326 expansions / 4.57 s with zero filled-body
  overlaps. The profile now permits two million expansions, retaining the
  ten-second timeout. Targeted geometry checks passed and the manifest was
  refreshed. Attempt 63 passed speed and reached 4/5 without contact, then
  failed the no-progress rule at 259.72 s near (27.219, 15.013); its stop gate
  passed. Return routes were available, but measured commands stayed near zero.
  Native calibrated scoring on its recorded local map gives standing still
  cost 17.58 and a body/braking-safe departure cost 20.76 with the 3000 reserve
  preference. With preference 500 the same departure costs 7.82; a gentle
  0.1 m/s departure also costs less than holding still. The profile now uses
  500 for the extra 0.15 m reserve, keeping the 0.10 m hard margin. Geometry,
  filled-body and final-control checks passed; the manifest was refreshed.
  Attempt 64 passed speed and cleared the previous return stall, then failed
  at 288.23 s after 132.451 m with 4/5 on `ramp_edge_departure:Ramp`; its stop
  gate passed and there were no tape/cone contacts. This is a referee proximity
  false positive: the whole padded body was outside the deck, with its nearest
  lateral corner about one metre beyond the actual side edge. The old ramp
  monitor classified any body within that extra band as departing the ramp.
  The canonical ramp referee correction uses finite side-edge/padded-body
  intersections under the existing dense sweep. Actual edge contact still
  fails; five regressions cover the recorded false positive and real crossings.
  Score schema 5 records this correction; speed/contact/no-progress/finish
  thresholds are retained, and navigation receives no course truth. The
  18-package build, all 23 ROS checks, all 32 Python tests, and clean/repeated
  installation of 17 simulator / 16 robot / 5 Nav2 patches passed. Strict
  attempt 65 passed speed and avoided all contact but made repeated return
  detours before failing the no-progress rule at 644.05 s after 275.732 m,
  reaching 4/5; its stop gate passed. Recorded local/global bodies were clear
  even with the 0.25 m preferred margin. Its exact final native replay exhausts
  two million expansions. A larger replay finds a 1,144-pose route in 3,057,867
  expansions / 9.39 s with zero filled-body overlaps. The profile now permits
  five million expansions / twenty seconds for simulator contention, retaining
  all footprint gates and the referee's 60-second no-progress limit. Targeted
  geometry checks passed and the manifest was refreshed. Strict attempt 66
  passed speed and avoided contact, but timed out on waypoint 5 after
  376.391 m / 852.0 s with 4/5 reached and a passing stop gate.
- Attempt 66's return loops exposed another timing mismatch: the native
  controller used wall time while MPPI's prediction and control-sequence shifts
  used ROS time. A real native-controller regression reproduced continued
  commands with the simulation clock frozen. The controller now waits for
  ROS-clock intervals, resets its deadline on backward jumps, skips missed
  intervals without a burst, and remains cancelable while paused. The independent
  steady-clock command gate is retained. The 19-package build, all 25 ROS checks,
  all 32 Python tests, and clean/repeated application of 17 simulator / 16 robot /
  6 Nav2 patches passed. Strict attempt 67 passed speed and reached 4/5 without
  contact, then failed the strict no-progress rule at 495.16 s after 163.292 m;
  its stop gate passed. The corrected clock cadence is retained.
- A native inflated-field regression exposed a planner/guard disagreement:
  the center-cost shortcut could accept a lethal filled-body corner when the
  continuous pose lay almost half a cell from the sampled inflation center.
  This was not covered by the prior tests' uniform cost-200 background.
  `filled_query_shortcut.patch` removes that shortcut; the immutable occupied-cell
  cache still skips empty body bounds safely. Center inflation remains the
  traversal cost, and the 0.10 m hard padding is unchanged. The 19-package build,
  all 25 ROS / 32 Python checks, and clean/repeated installation of 17 simulator /
  16 robot / 7 Nav2 patches passed. Strict attempt 68 passed speed and reached
  1/5 without contact, then failed no-progress at 189.97 s after 60.288 m;
  its stop gate passed. The GPS handler completed the first goal at about 130 s,
  but its acceptance reply had timed out in Fast DDS, leaving the mission runner
  waiting for an acknowledgement. A service-only XML profile increases the
  native reply-reader discovery window from 100 ms to five seconds. The native
  regression reproduces the old timeout with 300 ms delayed discovery, verifies
  the corrected installed writer QoS and successful ROS request/reply, and checks
  ordinary topic blocking time remains 100 ms. Each run snapshots and hashes
  this transport profile. No navigation or scoring threshold changed.
  No complete lap has passed.

Attempt 69 completed with working GPS acknowledgements, a passed strict speed
check, no referee violations, and a passed stop gate. It reached 4/5 waypoints
but timed out on waypoint five after 404.022 m / 991.0 simulated seconds.
Successive return replans moved the turning point away: remaining route length
grew from 7.51 m at 275.03 s to 14.54 m at 289.91 s. Frozen-map experiments
changing unseen-space costs did not alter the routes and were not applied.

`route_commitment.patch` optionally preserves an accepted shorter untraveled
route against a longer same-goal candidate, including forced periodic refreshes.
It compares projected progress on the old polyline, releases for a new goal or
frame, more than 0.30 m deviation, or recovery, and continues ticking both fresh
footprint guards. The full-track profile enables it with a 0.50 m length margin.
The native route-commitment and existing hazard-handoff regressions pass.
Clean/repeated installation of 17 simulator / 17 robot / 7 Nav2 patches and
all 27 ROS regression groups and all 32 Python tests pass. Strict attempt 70
reached 4/5 and passed speed/contact/stop checks, but still detoured; it was
interrupted for diagnosis at 491.0 s / 224.403 m. Comparing selected arrival
endpoints could mistake two cells within the planner's 0.35 m tolerance for
a new mission goal. The BT now passes the original `live_goal` request to the
decorator. A native regression covers shifted arrival cells for one request
and a changed request with the same endpoint. Full-course verification remains
pending. The final goal-identity correction passed all 27 ROS groups, all
32 Python tests, and clean/repeated patch installation. Strict attempt 71
reached 4/5 with no contact and passed speed/stop checks, then failed for
60 seconds without progress at 501.17 s. Its guards correctly rejected the
accepted northern turns as fresh tape arrived. Physical/sensed geometry
comparison confirms a long boundary, rather than an isolated short obstacle:
planning beyond its current sensed end repeatedly moved the crossing away.

`traveled_corridor.patch` adds optional return routing learned from estimated
odometry. A revisited goal first uses a private planning snapshot restricted
to previously traveled corridors; sensed hazard costs are preserved. An
preferred search falls back to ordinary sensed-map planning.
Frame changes and localization jumps clear the bounded history. The full-track
profile enables this policy. A native frozen replay at 282.3 s returned a
122.46 m route through traveled passages. This can make the return longer.
All 29 ROS groups, 32 Python tests, and clean/repeated installation of
17 simulator / 17 robot / 8 Nav2 overlays pass. Strict attempt 72 passed
speed/contact/stop checks and reached 4/5, then was interrupted for diagnosis
at 330.01 s / 146.546 m. Its initial preferred searches exceeded the new
eight-second bound. A replay found an 87.06 m learned return in 8.39 s under
simulator load, with zero filled-body conflicts. The preferred search now uses
the existing 20-second planner limit. The actual native planner replay returned
the known route after this correction; strict attempt 73 is running with saved
input snapshots. Attempt 73 still recirculated and was interrupted at
563.0 s / 210.707 m, reaching 4/5 with speed/contact/stop checks passed.
The recording ended at about 340 s after a live SQLite diagnostic read
conflicted with its writer; replay scripts now require a completed run and
reject requests beyond the recorded clock range. Results requested at 450 s
and 535 s therefore represented the last recorded 340 s input, not those
later poses. Replaying that input found a clear 117.74 m route at radius
1.5 m versus a clear 38.20 m route at 2.5 m. The latter leaves maneuvering
room around the unchanged padded body. The actual native planner and targeted
hazard/fallback/geometry regressions pass. Strict attempt 74 uses the wider
search corridor and uninterrupted recording. It reached the finish without
violations after 166.719 m / 376.0 s, passed strict speed and the five-zero-command
stop gate, and returned a successful GPS action sequence. Its result still
reported 4/5: the referee froze waypoint counting on entry to the 1.25 m finish
circle before reaching the overlapping 1.0 m final antenna marker. The completed
recording places the final physical antenna 0.948 m from that marker.

`ordered_finish.patch` requires all ordered waypoints before declaring finish,
keeping swept contact and speed checks active through the final approach with
unchanged scored radii. `gps_arrival_reference.patch` measures GPS arrival and
terminal distance at the antenna, using the same cached `FromLL` map projection
and TF as navigation. The old check used the internal base estimate and a
different first-fix datum. Missing projection or antenna TF cannot declare
arrival; native projection rotates the antenna lever arm back onto the base
estimate. All 30 ROS regression groups pass, as do clean/repeated installation
checks for 18 simulator / 18 robot / 8 Nav2 patches. Strict attempt 75 is running
with these corrections and immutable input snapshots. It reached 4/5 and passed
speed/contact/stop checks, then failed no-progress at 281.62 s after 93.255 m.
The old odom history mixed its earlier coordinates with a GPS-corrected current
goal: the 196 s request shifted to (-1.510, 1.471), outside the 1.5 m revisit
threshold, and an unrestricted shortcut replaced the learned route.

Travel history now uses `/global_ekf/odom` in `map` and is projected with current
TF into the planner frame for each search. An odom planner start cannot reset
that map history. A 3.5 m corridor accommodates frame shifts while preserving
all sensed hazard costs and body checks. The actual patched native planner
returns an 84.32 m learned route for attempt 75's 196 s input. Soft cost
experiments were not applied. All 30 ROS groups and clean/repeated patch checks
pass, including a global-history U-shaped route that rejects an untraveled
shortcut, changing TF, and safe unrestricted fallback around a new wall.
Strict attempt `full-track-fix-20261007-76` passed with immutable inputs:
5/5 ordered waypoints, finish reached, mission runner exit 0, no referee
violations, speed interval complete, and five consecutive zero-command samples
after stopping. It traveled 165.042 m over 378.0 simulated seconds at a maximum
0.5 m/s. All 7,584 scoring odometry samples came from the primary physical feed;
no fallback scoring was used. The recorder stopped cleanly and saved 306,218
messages. Its result, replay and exact input/build manifest are in
`runs/full-track-fix-20261007-76/`. This establishes one autonomous local-course
pass; the release repeatability suite remains open.

## Full local-track runtime test (2026-10-03)

- Added `run_full_track.cmd` and `run_full_track.sh` to run the exported local
  Blender scene with its matching world. The launcher verifies the source,
  course, and world hashes, requires the recorded geometry check, and keeps
  each attempt in a fresh output directory. The scene has 73 cones, 872 tape
  segments, one ramp, and five exported GPS waypoints.
- Fixed full-world startup: verify robot spawning with actual model odometry
  after the world clock advances; retry with renaming disabled. Added an
  index of width-expanded tape bounds to avoid testing every distant segment
  for every LiDAR query. Exact tape-distance classification is unchanged.
  Equivalence regressions cover segment endpoints, negative cell boundaries,
  zero-length segments, and all 872 full-course tape segments. Readiness
  failure now produces a failed result and skips mission dispatch.
- `runs/full-track-20261003-03/` passed all readiness checks, then failed at
  simulation time 103.01 s with `blocking_traffic_no_progress_over_60s`, base
  (0.193, -4.988), yaw -0.765 rad. It traveled 6.778 m and reached **0/5**
  waypoints. The planner reported 122 lethal-start errors and zero goal
  preemptions. The failure indicates a navigation/recovery stall on the first
  leg; this run does not establish its underlying costmap cause. The stop gate
  passed with five consecutive zero velocity commands.
- The run retains its result, readiness report, input/workspace hashes,
  ground-truth referee score, logs, costmap probe, rosbag, and offline replay.
  Attempts `full-track-20261003-01` and `-02` failed startup before a mission.
  The viewer now draws the authored full-course tape and rotated ramp.
- Verification passed: four focused ROS tests and 30 Python tests. All seven
  simulator patches and eight robot patches apply cleanly to the pinned
  sources and remain detectable on repeated setup. The local scene's missing
  semantic features remain visible in the manifest; no full-lap or competition
  acceptance pass is claimed.

## Navigation diagnosis fixes (2026-10-03)

- Removed the duplicate `/goal_pose` dispatch for both LOCAL and GPS goals.
  Each leg now uses one tracked `NavigateToPose` action, with in-flight
  corrections on `/goal_update`. Genuine terminal failures retain bounded
  redispatch through the same action route.
- Planner candidates use a separate blackboard value. Only successful swept
  footprint validation publishes the accepted path. A reactive controller
  check cancels `FollowPath` when its filtered path or the measured robot
  footprint becomes unsafe, even while planning is recovering. It also checks
  1.5 m ahead against the faster local costmap and skips traveled path poses.
- Corrected sticky lethal tape memory: explicit lower nonzero line-layer
  observations may update remembered halo cells beside the robot. Unobserved
  FREE cells still require the clearing-view gate, and UNKNOWN does not erase
  memory. The obstacle mirror masks exact lethal tape cells only, preserving
  real barrel marks where they overlap a soft tape halo.
- Rebuilt `gps_waypoint_handler`, `custom_behavior_tree_plugins`, and
  `local_mirror_layer` in the persistent pinned Docker workspace. Added runtime
  regressions for goal dispatch/corrections/retries, successful-only path
  handoff, unsafe-route cancellation, measured footprint checks, remaining-path
  pruning, local lookahead, tape-memory updates, and obstacle-mask thresholds.
  All three ROS regression executables/tests and all 26 existing Python tests
  pass. The eight robot patches and five simulator patches apply cleanly to
  pinned sources and remain detectable on a repeated setup run.
- The original diagnostic timed out at `obstacle_gps` with 1/4 waypoints. The
  fixed runs `compact-navigation-fix-20261003-02` and
  `compact-navigation-fix-20261003-03` both passed that waypoint. The latter
  recorded no duplicate goal preemptions but failed later at
  `tape_crossing:right_boundary_3`, simulation time 58.88 s, base pose
  (16.903, -2.382), yaw -1.884 rad, during ramp-goal pursuit. It reached 2/4
  waypoints and passed the five-zero-command stop gate. Full-lap navigation
  remains open.
  The latest log has zero `Starting point in lethal space` errors, compared
  with 344 in the original diagnostic, and zero goal preemptions, compared
  with 63. It retains two genuine terminal-failure redispatches.

## Implemented and verified

- Pinned simulator `46b8e9aa953160b2d317fcc811e209fed46a52f4` and robot
  `00875b391b9ea897681893ec063c99a3d9d32835` were inspected. The
  existing uncommitted Python prototype was left intact.
- Portable Blender 5.0.1 opened the actual `Course.blend`. The exported
  versioned object inventory, overview, source hash, provisional semantic
  roles, and asset gap report are in `Mock_Course_Asset/`.
- `tools/prepare_full_track.py` uses the pinned simulator course loader and
  world generator to emit matching YAML/SDF, four public target roles, and
  input hashes. The upstream feasibility validator's stale `Course.robot`
  reference was fixed by a tracked patch. The diagnostic local course passes
  its 0.05 m grid connectivity check (880 x 920 cells, 0.46 m inflation; five
  exported markers and finish candidate reachable). The strict asset gate
  correctly fails for the current Blender scene.
- The pinned official ROS Humble/Gazebo Fortress container image was extended
  with the x86 dependencies. The selected 14 robot and simulator packages
  built successfully. `integrations/ros_humble_fortress/setup.ps1` reproduces
  the pinned checkouts, patches, and build in a Docker volume.
- The dedicated Nav2 profile enables simulation time, a measured robot
  footprint, footprint-aware MPPI cost checking, direct Smac State Lattice
  planning, and an explicit swept-path BT check. The detection launch patch
  applies `use_sim_time` to the detector nodes.
- A compact-course ROS/Gazebo smoke stack passed a bounded readiness probe:
  advancing clock, fresh LiDAR/GPS/odometry/map, complete map-to-robot TF,
  active Nav2 servers, a waypoint action server, exactly one gate publisher,
  a dynamics subscriber, and fresh zero commands before Run.
- The mission runner now cancels an active waypoint on an internal wall-time
  deadline or referee failure. A separate steady-clock command gate sits
  between `/cmd_vel` and the Gazebo dynamics adapter and requires fresh Run
  heartbeats and Nav2 commands. In the 25-second bounded smoke run,
  `runs/compact-gate-probe-v3/`, the first local waypoint succeeded; the GPS
  leg timed out, the Nav2 goal was canceled, and the post-run probe observed
  five consecutive zero velocity commands. An 18-second run independently
  passed the strengthened gate readiness check and zero-command probe, but
  timed out at its first waypoint. Gate-state unit tests pass.
- The compact run exposed a heading-estimation error: comparing a windowed GPS
  course with instantaneous odometry yaw shifted distant goals sideways in
  turns. The handler now fits the world-to-odom rotation from paired GPS and
  odometry displacement over the same 30-sample window. The turning regression
  test passes in the pinned container.
- The global local-mirror layer now forgets an old lethal mark when a softer
  local cell is observed in its configured forward clearing view. The swept
  path check tests the whole padded footprint against lethal cells and the
  robot center against the inscribed band, avoiding double inflation while
  retaining a full-body tape/obstacle collision gate. Both C++ plugins rebuilt.
- The compact bootstrap waypoint's 1.0 m tolerance lets the GPS estimator
  initialize without spending roughly a minute trying to reach a point beside
  an observed barrel. The scored `runs/compact-footprint-fix/` run reached both
  `bootstrap_local` and `obstacle_gps`, crossed no boundary on those legs, and
  verified five zero commands after the later referee failure.
- The Windows compact launcher now opens a live browser course view showing
  the robot's ground-truth pose, traveled path, map-aligned Nav2 plan, mission
  targets, and referee state. It saves a self-contained `replay.html` and a
  `result.json` that distinguishes mission outcome from command-gate outcome.
  This is a diagnostic course view, not a Gazebo 3D client or the full-course
  Run application.
- The generated ramp-edge white strips are now reflected in the synthetic
  LiDAR intensity/reflector output, matching the world drawing. A geometry
  check confirms both ramp edges are classified as tape. The first scored
  retest still failed at the ramp, so this sensor correction is not a
  navigation completion claim.

## Gates still open

| Plan gate | Observed state |
|---|---|
| Validated local asset | The 83-object Blender scene has a 38 x 38 m ground, two line meshes, one ramp, 73 cones, start, and five markers. It does not author potholes, a No Man's Land region, an internal boundary, a finish gate, or ordered referee checkpoints. The fifth marker is only a finish candidate. Four GPS roles are provisional. `--require-complete-assets` fails. |
| Full world fidelity | The generated SDF represents semantic YAML, not every Blender collision mesh. Ramp contact and camera-visible marking geometry have not been verified in Gazebo. The upstream `official_full_loop.yaml` also fails the patched validator because its generated world is stale and its `line_less_gps_sections` requirement is missing. |
| Sensor-driven stack | The diagnostic now uses local wheel/gyro and global GPS EKFs with an initially east-facing robot, plus `/map_padded`. Real-sensor calibration, transient mapping, CPU camera lines, visual potholes, and lane-to-GPS state transitions are not integrated. The image lacks CUDAToolkit, so the upstream CUDA camera detector was skipped. |
| Navigation | Attempt 76 passed the autonomous local course: 5/5, finish, zero violations, strict speed and stop gates, 165.042 m / 378.0 s. The profile retains 0.10 m hard padding, preference 500, queued-command prediction, forward travel and schema-5 swept-contact scoring. Learned return history uses global GPS EKF coordinates projected through current TF; GPS arrival uses the antenna and matching datum. The controller/smoother use ROS time and the stop gate uses steady time. |
| Run application | Compact and full local-scene diagnostics have launchers, a live browser view, saved replay, and result bundle; the full run also retains a rosbag. Gazebo runs in server-only mode. There is no visible default full-course Run button, Pause/Resume/Stop/Reset UI, or validated three-repeat reset lifecycle. The gate is wired for diagnostic runs, but the simulator's `/autonomous_mode` harness still publishes true continuously. |
| Release suite | One full local-course diagnostic passed. Three clean repeats, ten-seed testing, a rules-complete referee and performance acceptance remain unverified. |

The diagnostic lap establishes autonomous completion of the current local
course. It does not verify the missing authored features or the release suite.

## Next implementation dependencies

Author the missing features and unambiguous mission roles in the Blender
source, regenerate the world and course as a single versioned pair, and verify
sensor observations. Replace the simulator pose/map shortcuts with the robot
estimator and observed map, add camera-derived pothole exclusion and lane
progress logic. The current local-course stall is resolved; compact-course
differences remain separate validation work. Implement the desktop Run
application and repeatability suite before treating the system as a release
candidate.
