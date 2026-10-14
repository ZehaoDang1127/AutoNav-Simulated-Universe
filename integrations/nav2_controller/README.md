# Native controller ROS-clock cadence

`ros_clock.patch` applies to pinned Nav2 Humble 1.1.20 at
`a097086719c88f781aa59788eca29ac6ca5e56db`.

The controller action loop originally used `rclcpp::WallRate`. Its configured
10 Hz therefore advanced the MPPI control sequence every 100 ms of wall time,
even when less simulation time elapsed. Attempt 66's live command capture
contained 105 outputs over 6.11 seconds of ROS time. The native regression
also reproduced commands continuing while `/clock` was held constant.

The loop now waits on the node's clock. It runs once initially, then once per
configured ROS-time interval. Clock rewinds reset the deadline and large jumps
skip missed intervals without a burst of old commands. A one-millisecond wall
poll permits action cancellation and shutdown while simulation time is paused.
The costmap-readiness wait also observes cancellation. On a normal hardware
clock, the configured frequency retains its ordinary real-time meaning.

The command gate retains its independent steady-clock stop timer. Dynamics,
footprint validation and scoring thresholds are unchanged.

`tests/ros/test_controller_clock.py` runs the actual native controller executable
with an empty sensed costmap and the installed RPP plugin. It verifies paused
time, partial intervals, 10 Hz clock ticks, and cancellation to a zero command
without advancing the clock. `controller_rate_test.cpp` checks missed intervals
and backward clock jumps in the same rate helper used by the action loop.

Attempt 66 reached four of five waypoints with speed and stop-gate passes and
no contact violations, but timed out on the final waypoint after 376.391 m.
The 19-package build and all 25 ROS / 32 Python checks passed. Strict attempt 67
passed speed and reached 4/5 without contact, then failed no-progress at
495.16 s after 163.292 m; its stop gate passed. The clock correction is retained,
and a separate native planner collision shortcut is now corrected. Its 19-package
build and all 25 ROS / 32 Python checks passed. Strict attempt 68 passed speed
and reached 1/5 without contact, then failed no-progress at 189.97 s after a
lost GPS action acceptance reply. Its stop gate passed. The service-reply
discovery window is corrected separately; a complete strict lap remains unverified.
