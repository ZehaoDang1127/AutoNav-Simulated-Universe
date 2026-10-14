# Humble Smac footprint correction

`continuous_footprint.patch` applies to Navigation2 1.1.20, commit
`a097086719c88f781aa59788eca29ac6ca5e56db`. Bootstrap builds only
`nav2_smac_planner` from that pinned source and records its revision, patch
hash, and installed lattice-library hash in the workspace manifest.

On the frozen map from `full-track-fix-20261005-16`, the original lattice
search produced 24 poses containing lethal cell centers inside the padded
body. Its checker truncates fractional grid positions and orientation bins,
and checks the footprint outline. The independent filled-body BT gate rejected
that route, causing repeated retries without progress.

The overlay retains fractional positions and yaw, checks lethal cell centers
throughout the polygon, and checks every primitive pose, including in-place
rotation. Center inflation and unknown-space policy are preserved. A per-search
occupied-cell prefix cache skips interior scans in clear bounding boxes; the
cache is explicitly refreshed while the planner holds the map lock.

The hard footprint rule uses cell centers throughout the filled padded body,
matching the BT gate. Adjacent cells with centers outside that body cannot
independently reject it. Valid nodes retain center inflation as their soft
traversal cost, as in current Nav2 Smac. Scoring the already-inflated padded
edge counted the body radius twice and could make sensed lanes costlier than
unobserved cells. Occupied columns are indexed per row on the immutable map,
preserving decisions while avoiding repeated scans of free bounding-box cells.
An October 6 regression reproduced that false rejection before the correction.
The planner also retains the measured continuous start position and checks the
full initial turn to a collision-free lattice heading. It includes the measured
pose in its output, so downstream checks validate that connection too. A frozen
map from attempt 24 rejected the snapped start despite a clear measured body;
the corrected start admits a route while preserving all filled-body checks.

On attempt 26's final frozen map, search fell from 3.54 seconds and 238,188
expansions to 1.20 seconds and 195,085 expansions after occupied-row indexing
and center-cost scoring. Both routes had zero lethal body overlaps. The
regression compares 1,024 cached/uncached collision decisions and checks that
footprint-edge inflation cannot replace center traversal cost.

The lattice planner copies an immutable search map before releasing that lock,
so live sensor updates continue during search. Controller guards still check
current maps before movement. If smoothing introduces a filled-footprint
collision, the planner retains the searched route. Whole-path candidate and
active measured-body safety checks remain enabled.

The corrected frozen-map search produced 921 poses with zero filled-body
lethal overlaps in 4.51 seconds, versus approximately 14 seconds before the
cache optimization. The synthetic ROS regression covers interior hazards,
fractional translation and yaw, pure rotation, unknown cells, cache refresh,
map bounds, and safe connections from continuous start poses. Full-course validation is recorded in
`FULL_TRACK_IMPLEMENTATION_STATUS.md`; a safe replay does not establish a lap.

Optional `allow_goal_heading_change` first preserves the requested goal cell while
choosing the closest collision-free lattice arrival heading. The full-track
GPS profile enables it because its goal checker accepts any yaw. Other profiles
retain the original heading constraint by default. When no heading at that cell
is clear, the selector tries the nearest filled-body-clear cell within the existing
world-position tolerance. A moved cell is searched with zero additional tolerance,
so positional tolerances cannot compound. If the entire permitted region is blocked,
the original search fallback remains available.

Attempt 53 passed strict speed scoring and reached 4/5 without contact, then
failed the no-progress rule at 388.22 s because its final goal's requested
heading intersected lethal cells. On that recorded map the original search
exhausted 500,000 expansions; changing only arrival heading yielded a clear
1,564-pose route in 0.28 s and 18,130 expansions. A runtime regression checks
preserving a clear requested heading, choosing a clear alternate, and rejecting
a fully blocked goal. All 21 ROS checks and 32 Python tests pass; strict
full-track attempt 54 passed speed scoring but stalled at an earlier clear-body
corner. All 21 ROS checks and 32 Python tests pass with the controller's
small-rotation refinement. Attempt 55 also passed speed scoring but stalled at
0/5. Attempt 56 passed speed scoring and reached 4/5 without contact, then failed
the no-progress rule on the final return. All exact-goal headings were blocked,
but a clear arrival cell existed 0.256 m away within the unchanged 0.35 m tolerance.
The bounded selector yields a 1,238-pose clear route to that cell in 2.94 s /
415,862 expansions on the original captured input. Full waypoint completion
remains unverified. The 17-package build, all 21 ROS checks, all 32 Python tests,
and clean/repeated patch installation passed. Attempt 57 passed strict speed
scoring and reached 4/5 without contact, then failed at the final return because
the 500,000-expansion cap was insufficient. Its clear recorded-map route needs
672,597 expansions, completing in 4.53 s with zero filled-body overlaps. The
profile then permitted 1,000,000 expansions / 10 s per search; targeted geometry
checks pass. Attempt 58 passed speed scoring but stalled at 0/5 when fresh marks
entered its extra navigation padding; its physical/referee-sized bodies were
clear. Recorded-map replay finds a 268-pose safe departure with 0.05 m padding,
tested in attempt 59. The 0.25 m preferred clearance, braking
validation and strict scoring remain. Attempt 59 passed speed scoring but
contacted cone 032, so the larger 0.10 m hard margin is restored. ROS-clock
smoothing and actual-command prediction passed the 18-package build, all 22 ROS
checks and all 32 Python tests. Attempt 60 passed speed scoring but crossed
unseen rear tape during reverse travel. The diagnostic profile now disables
reverse lattice expansion and reverse MPPI commands while retaining in-place
turns. Attempt 61 failed startup. Forward-only attempt 62 passed speed and
reached 4/5 without contact, then hit the one-million-expansion cap on the return.
Its captured request succeeds in 1,288,326 expansions / 4.57 s with 1,132 poses
and zero filled-body overlaps. The profile now permits two million expansions
with the same ten-second timeout. Targeted geometry checks pass; strict attempt
63 reached 4/5 without contact but stalled despite valid return routes. Its
recorded controller costs identify the extra-clearance preference as the next
issue; the hard planner footprint is retained. Strict attempt 64 tests the
softer controller preference with the same two-million / ten-second search.
Attempt 64 cleared that stall, then hit a corrected ramp-proximity referee
false positive. Attempt 65 avoided contact but exhausted two million
expansions on its final return request. A larger native replay finds a clear
1,144-pose route in 3,057,867 expansions / 9.39 s. The profile now permits five
million / twenty seconds to accommodate simulator contention. The 0.10 m
hard footprint and strict referee gates are retained. Strict attempt 66 passed
speed and avoided contact but timed out on waypoint 5, reaching 4/5 after
376.391 m. Its stop gate passed. The next correction makes the native controller
follow ROS-clock intervals rather than wall time; the 19-package build and all
25 ROS / 32 Python checks passed. Strict attempt 67 passed speed and reached
4/5 without contact, then failed no-progress at 495.16 s after 163.292 m;
its stop gate passed. The five-million / twenty-second planner budget is retained.

`filled_query_shortcut.patch` removes the center-cost early exit for polygonal
bodies. A fractional body query can lie almost half a cell from the sampled
inflation center; a center cost below the circumscribed threshold does not
certify every filled corner. The new native regression constructs the actual
inflated field and reproduces a lethal corner accepted by the old shortcut.
The occupied-cell cache continues to skip empty body bounds, preserving center
inflation as traversal cost and full padded polygons as hard collision geometry.
The 19-package build and all 25 ROS / 32 Python checks passed. Strict attempt 68
passed speed and reached 1/5 without contact, then failed no-progress at
189.97 s after a lost GPS action acceptance reply. Its stop gate passed.
The runner now snapshots a service-only Fast DDS discovery profile; navigation
and scoring thresholds remained unchanged for that attempt.

The return-loop correction is described in [traveled_corridor.md](traveled_corridor.md).
It records the estimated global GPS EKF history and transforms it through current
TF for each search, preserving sensed hazards. The full-track profile uses a
3.5 m corridor and retains shorter accepted routes while fresh footprint guards
continue checking them. GPS antenna arrival and ordered finish scoring are
described in [gps_arrival_reference.md](../robot/gps_arrival_reference.md).

Strict attempt 76 completed the local course autonomously: 5/5 waypoints,
finish, zero referee violations, passed speed and five-zero-command stop gates,
165.042 m / 378.0 simulated seconds. All 30 ROS regression groups and
clean/repeated patch installation passed. The saved result and replay are in
`runs/full-track-fix-20261007-76/`; release repeatability remains unverified.
