# GPS arrival uses the antenna and the navigation datum

The handler's internal EKF estimates `base_link`, after subtracting the GPS
antenna lever arm. Its former arrival check compared that base position to a
GPS receiver target. With `use_navsat_projection` enabled, it also used a
different first-fix datum from the `FromLL` target sent to Nav2.

`gps_arrival_reference.patch` measures arrival and terminal distance using
`map -> gps_footprint` against the cached `FromLL` map target. Missing target
projection or antenna TF cannot declare arrival. In the native projection
mode, it adds the rotated antenna lever arm back to the internal base estimate.
Local goals keep their base reference. Waypoint radii remain unchanged.

Attempt 74 completed its GPS action sequence and stopped safely, but the referee
reported 4/5. Its larger finish circle froze waypoint counting before the final
antenna reached the marker. `../autonav_sim/ordered_finish.patch` requires all
ordered waypoints before declaring finish, so strict contact and speed checks
continue through that last approach. It changes neither waypoint nor finish
radii. The completed recording shows finish-circle entry at 373.50 s, with the
antenna still 1.446 m from the marker, and final-marker entry at 374.56 s. The
final physical antenna was 0.948 m from the 1 m marker; the premature freeze,
rather than that final position, caused the missing scored waypoint.

Regressions exercise independent datum drift, antenna offsets and rotation,
missing projection/TF, local goals, overlapping finish/waypoint circles, ordered
waypoints, and contact/speed checks continuing until all markers are reached.
