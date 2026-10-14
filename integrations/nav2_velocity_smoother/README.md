# ROS-clock command smoothing

`ros_clock.patch` applies to the same pinned Navigation2 1.1.20 checkout as
Smac and MPPI. Bootstrap builds `nav2_velocity_smoother` and records its patch
and library hashes in the workspace manifest.

The native smoother used wall timers while command timeout, simulation motion,
and acceleration prediction used ROS time. With a slowed simulator, fixed
per-tick increments therefore produced a faster ramp per simulated second.
The native regression reproduced seven output updates while ROS time was held
constant. Both activation and frequency-change timers now use the node clock.
The configured 20 Hz rate, velocity limits and acceleration limits are unchanged.

`tests/ros/test_velocity_smoother_clock.py` runs the native lifecycle node in an
isolated ROS domain. It verifies no ramp updates during a paused clock, four
20 Hz simulation ticks, a change to 40 Hz, and deceleration on the new ticks.
The process is launched directly so cleanup terminates the actual native node.

The full-track MPPI profile also receives `/cmd_vel_supervised` as command
feedback. Prediction uses past commands during transport latency and the
same discrete smoother ramp before the calibrated actuator response. No
absolute physical pose or heading is fed to navigation. Full-track outcomes
are recorded separately in `FULL_TRACK_IMPLEMENTATION_STATUS.md`.
