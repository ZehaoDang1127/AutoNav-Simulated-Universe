# MPPI response and final-control overlays

`calibrated_response.patch` applies to Navigation2 1.1.20 at
`a097086719c88f781aa59788eca29ac6ca5e56db`. Bootstrap also builds this package.
`final_control_validation.patch` applies after the response patch.

The optional `CalibratedDiffDrive` motion model predicts command latency,
first-order linear/angular response, and acceleration limits before all
trajectory critics score the sampled body poses. The profile matches
`full_track_dynamics.yaml`; the vehicle's physical calibration and strict
referee rules are unchanged. The existing motion models remain available.
Predicted actuator response and the final command retain configured speed bounds.
Gaussian perturbations remain intact for the weighted MPPI update. Attempt 30
clipped them before averaging and failed at 0.289 m/s, with maximum speed only
0.443 m/s. Saturation now applies to predicted actuator targets, avoiding the
inward sampling bias while retaining physical limits.
The response reads current optimizer constraints, including changed speed limits.
The controller sends command zero for this model because latency is already
included in its prediction; the original models retain their command offset.

Attempt 29's half-second command replay measured mean position/yaw prediction
errors of 0.041 m / 0.114 rad with instantaneous response and 0.020 m / 0.047 rad
with calibrated response. This comparison uses recorded commands and measured
velocity; it does not prove future command prediction or a full-track pass.
Unobserved commands already in flight initialize to measured velocity.

`tests/ros/calibrated_motion_test.cpp` checks latency, acceleration, command
bounds, forward/reverse/stop convergence, changed limits, and a filled-body
collision during a turn reversal that instantaneous prediction misses.
The geometry regression ties prediction parameters to the actual dynamics file.

MPPI's weighted average and temporal filter can cross an obstacle even when
the sampled turns are safe. For the calibrated model, the controller now
reapplies command bounds and scores the final physical rollout with its actual
configured critics under the existing costmap lock. If that rollout collides,
it filters samples in score order using copied issued-command history, applies
bounds, and validates the filtered sequence before issuing it. Rejected
candidates and internal retries preserve the actual history. If none is safe,
the existing bounded failure/retry handling stops control. History advances
once for the actual issued action.

The chosen action must also permit a clear braking path. The controller holds
its first command for one prediction step, then ramps linear and angular
commands to zero at the matched velocity-smoother limits. The same calibrated
latency, vehicle inertia, critics, and locked costmap check the stopping body.
The original requested sequence is restored after this check. This closes the
gap where safe future planned turns were required to avoid contact after a
path guard could instead halt control. Candidate selection requires both checks.

`tests/ros/optimal_control_test.cpp` reproduces two safe turns whose straight
average collides, checks selection of a safe filtered bounded sample, verifies
candidate evaluation preserves issued history, rejects a fully blocked body,
and rejects a continuing turn whose smoothed braking sweep hits a cell.
Full-track attempts 45–48 and 50 passed the strict first-44-feet speed interval;
full waypoint completion remains unverified. Attempt 48 uses a 100 ms control
interval and 28 prediction steps, retaining the 2.8 s horizon after repeated
50 ms control-loop overruns. It records raw controller commands for cadence checks.

`lattice_maneuvers.patch` adds optional `consider_lattice_rotations` to
PathAngleCritic. At a stationary yaw sequence within 0.15 m, it scores the
required final body heading instead of a bearing to repeated/later positions.
Turns below configurable `lattice_rotation_min_angle` (0.30 rad in the profile)
retain the normal bearing score; these small lattice-bin alignment steps can
otherwise suppress the following curved motion.
That heading is not interchangeable with yaw + pi for the asymmetric body.
Ordinary translating paths retain their original bearing score. Full forward
lookahead remains available, and `enforce_path_inversion` stays disabled.

Attempt 51 passed strict speed scoring and cleared the earlier corner but
oscillated at a later lattice rotation. It was interrupted for diagnosis at
341.01 s, with 0/5 waypoints, no referee violations, and a passed stop gate.
Attempt 52's stop-at-every-turn experiment failed strict speed scoring at
0.266 m/s and was removed. `tests/ros/lattice_heading_test.cpp` compares native
heading scores for required rotations, asymmetric reverse alternatives,
unchanged duplicates, distant turns, wrapped yaw, and near-goal turns. The new
heading-score overlay passed all 21 ROS checks and 32 Python tests. Strict
full-track attempt 53 passed strict speed scoring and reached 4/5 without contact,
then failed while planning the final goal. Full waypoint completion remains unverified.
Attempt 54 passed speed scoring but stalled at an earlier clear-body corner.
The regression now checks that small lattice alignment turns preserve normal
translating bearing. All 21 ROS checks and 32 Python tests pass; strict full-track
attempt 55 also passed speed scoring but stalled at 0/5. Attempt 56 passed speed
scoring and reached 4/5 without contact with the restored 3000 extra-clearance
preference and all hard body/braking checks retained. It failed while planning
the final return. Smac's bounded arrival-cell selection passed the 17-package
build, all 21 ROS checks, and all 32 Python tests. Attempt 57 also passed strict
speed scoring and reached 4/5 without contact, then exhausted the final-return
planner budget. Attempt 58 passed speed scoring but stalled after a fresh
observation entered the extra hard padding. Attempt 59 tested a shared 0.05 m
hard margin with 0.20 m extra soft reserve, retaining 0.25 m total preferred
clearance. Attempt 59 passed speed scoring
but contacted cone 032, so the 0.10 m hard margin / 0.15 m soft reserve is
restored. The native smoother now uses ROS-clock timers; its regression
reproduced seven ramp updates while simulation time was paused before the fix.
The MPPI profile receives `/cmd_vel_supervised` feedback. One immutable command
snapshot per evaluation supplies the time-matched commands already in flight;
normal predicted commands pass through the 20 Hz smoother ramp before latency
and calibrated actuator response. Empty/stale feedback falls back to measured
velocity and cannot invent an old active turn. Default profiles without feedback
retain the previous response model. The 18-package build, all 22 ROS checks,
all 32 Python tests, and clean/repeated patch installation passed. Strict
attempt 60 passed speed scoring but crossed tape during prolonged reverse
travel into rear geometry absent from the observed map. The diagnostic profile
now uses nonnegative MPPI linear commands and forward path-angle preference,
with forward-only lattice planning and in-place turns still available. The
physical vehicle limits and scoring are unchanged. Targeted geometry checks
pass. Attempt 61 failed startup; attempt 62 passed speed and reached 4/5 without
contact, then exhausted the final-return planner expansion cap. Its recorded
forward-only route is clear with 1.29 million expansions, so the profile now
permits two million with the same ten-second timeout. Attempt 63 passed speed
and reached 4/5 without contact, then stalled despite valid return routes.
Native calibrated scoring on its recorded local map makes holding still
cheaper than safe departure with reserve cost 3000 (17.58 vs 20.76); reserve
cost 500 makes the same body/braking-safe departure cost 7.82. The profile
uses 500 for the extra 0.15 m reserve, retaining 0.10 m hard body padding.
Geometry, filled-body and final-control checks pass. Strict attempt 64
in `runs/full-track-fix-20261007-64` cleared the return stall and reached 4/5
without tape/cone contact, then hit a ramp-proximity false positive near the
finish. The referee correction retains actual swept ramp-edge contact failures.
All 23 ROS checks and 32 Python tests pass. Attempt 65 avoided contact but
exhausted another return search after repeated detours. Its exact recorded
request needs 3.06 million expansions / 9.39 s with a clear filled-body route;
the planner budget is now five million / twenty seconds. Strict attempt 66
passed speed and avoided contact but timed out on waypoint 5 after 376.391 m,
reaching 4/5 with a passing stop gate. Its native controller advanced on wall
time while MPPI predicted ROS-time intervals. The separate
`integrations/nav2_controller/ros_clock.patch` corrects that cadence and retains
paused-clock cancellation. The 19-package build and all 25 ROS / 32 Python
checks passed. Strict attempt 67 passed speed and reached 4/5 without contact,
then failed no-progress at 495.16 s after 163.292 m, with a passing stop gate.
The controller-clock correction is retained. A reproduced native planner
center-inflation shortcut is now corrected; the 19-package build and all
25 ROS / 32 Python checks passed. Strict attempt 68 passed speed and reached
1/5 without contact, then failed no-progress at 189.97 s after losing the GPS
action acceptance reply. Its stop gate passed. The runner now snapshots a
service-only Fast DDS discovery profile; navigation and scoring thresholds
remain unchanged. No complete lap has passed.
