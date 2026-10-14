# Prefer a learned corridor when returning to a known goal

The planner can route around the currently observed end of a long tape boundary.
As the robot approaches, newly observed tape closes that gap and the next plan
moves the crossing farther away. Attempts 69–71 showed this during the finish
return. Attempt 71's footprint guards correctly rejected successive turns;
route retention alone could not preserve an unsafe route.

`traveled_corridor.patch` records estimated odometry, sampled every 0.10 m.
The full-track profile uses the global GPS EKF in `map`; each search projects
the complete history through the current TF into the planning frame. A changing
GPS correction cannot mix current goal coordinates with old odom coordinates.
An odom-frame planner start also cannot reset a map-frame history.
For a goal within 1.5 m of a position visited at least 20 m of travel earlier,
Smac first searches a private snapshot restricted to a corridor around
that history. The full-track profile uses a 3.5 m radius to accommodate current
frame corrections and maneuvering around the roughly 1 m axle-centered body.
Hazard checks still govern
every pose. A frozen native replay of attempt 71's 282.3 s input found a
122.46 m route with the earlier 1.5 m radius through previously traveled passages instead of the moving
northern crossing. This preference can make a return substantially longer.
Attempt 73's last recorded input at about 340 s produced a 117.74 m route
with that narrow radius, versus 38.20 m at 2.5 m with zero filled-body conflicts.
Attempt 74 reached the finish with that 2.5 m radius but exposed premature
referee completion. Attempt 75 reached 4/5 and stopped safely at a blocked turn.
Its moving odom-frame goal temporarily lay more than 1.5 m from the old travel
history, disabling the return preference. Reprojecting global estimated history
with a 3.5 m radius yields an 84.32 m learned route on its 196 s native snapshot.
The actual patched planner's TF path reproduces that route. Soft corridor-cost
experiments were not applied.

The corridor adds exclusions to the search snapshot and never lowers sensed
hazard costs. The existing continuous body, smoothing, active route, braking,
and ground-truth referee checks remain authoritative. The preferred search uses
the existing 20-second planner limit and falls back to ordinary sensed-map
planning if unavailable. New goals
outside the history use ordinary planning directly. Frame changes and odometry
jumps over 2 m clear history; storage is bounded to 20,000 samples.

The option defaults to false. `full_track_baseline.yaml` enables it with
`/global_ekf/odom`, using neither course geometry nor physical truth as planner
input. Native regressions check revisit detection, speculative shortcuts,
new hazards, history resets, current-frame projection, a global U-shaped route
that must avoid an untraveled shortcut, and a new wall forcing a safe unrestricted detour.
Full-course results are recorded in
`FULL_TRACK_IMPLEMENTATION_STATUS.md`.

Strict attempt 76 completed all 5 waypoints and the finish with zero referee
violations, passed speed and stop gates, and clean recording. It traveled
165.042 m over 378.0 simulated seconds. This is one autonomous local-course
pass, with release repeatability still unverified.
