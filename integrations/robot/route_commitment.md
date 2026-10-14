# Keep progress through a return turn

`route_commitment.patch` adds an optional rule to `PathSignificantlyChanged`.
When its child is running, a candidate for the same goal does not replace the
accepted route if it is more than 0.50 m longer than the accepted route's
untraveled suffix. Progress is projected onto the accepted polyline; comparing
with its original total length would incorrectly accept increasingly long
detours as the robot moves.

The rule applies only within 0.30 m of that route and in the same coordinate
frame. The BT passes the original `live_goal` request so different arrival
cells within the planner tolerance cannot be mistaken for different goals.
A new goal, a shorter/comparable candidate, a significant deviation,
or a recovery halt releases the commitment. The option defaults to false;
the full-track profile enables it. A periodic update cannot override it.

The decorator still ticks the existing global/local footprint guards on every
BT cycle. They use fresh sensed maps and the measured body, stop unsafe motion,
and trigger recovery so a longer replacement can be accepted. Candidate
validation, padding, braking checks, speed limits and strict scoring are retained.

Attempt 69 reached 4/5 waypoints, passed speed and contact checks, then timed
out at waypoint five after 404.022 m. Its recorded plans moved the northern
return turn farther away during successive replans: remaining route length
increased from 7.51 m at 275.03 s to 14.54 m at 289.91 s. Native replays retain
the relevant body checks. Changes to the unseen-space cost did not change those
routes and were not applied.

Attempt 71 confirmed that retention alone is insufficient: the fresh guards
correctly rejected northern turns as the robot observed the continuation of
a long tape boundary. The planner's learned return-corridor preference is
described in `../nav2_smac_planner/traveled_corridor.md`.

The native regression covers suffix comparison, repeated moving turns,
periodic updates, shifted arrival cells for one request, better candidates,
changed goals/frames, route deviation,
recovery, and disabling the option. Full-run results are recorded in
`FULL_TRACK_IMPLEMENTATION_STATUS.md`.
