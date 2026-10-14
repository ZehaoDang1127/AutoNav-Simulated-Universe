"""Small reference plug-ins for interface and common-run demonstrations.

These policies consume only EpisodeContext and Observation. They are not a
replacement for the legacy GPS EKF, LiDAR PCA costmap or BT recovery tree.
"""
from __future__ import annotations

import math

from .api import EpisodeContext, NavigationCommand, Observation, SensorSample, PluginManifest
from .scenarios import latlon_to_meters
from .perception import ReferenceCostmap


def _angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _latest(observation: Observation, kind: str) -> SensorSample | None:
    return next((s for s in observation.samples if s.kind == kind and s.valid), None)


class WaypointFollower:
    """Odom-frame waypoint tracking with a local LiDAR avoidance reflex."""
    manifest = PluginManifest("1", "0.1.0", ("complete-navigation",),
                              ("odometry", "lidar"))

    def reset(self, context: EpisodeContext) -> None:
        if context.track != "complete-navigation":
            raise ValueError("WaypointFollower requires complete-navigation track")
        self.context = context

    def step(self, observation: Observation) -> NavigationCommand:
        odom = _latest(observation, "odometry")
        if odom is None or observation.time_s - odom.acquired_s > 0.2:
            return NavigationCommand(0, 0, observation.time_s+0.1)
        x, y, heading = odom.data[:3]
        goal = self.context.goals[observation.goal_index]
        delta_x, delta_y = goal.x_m-x, goal.y_m-y
        error = _angle(math.atan2(delta_y, delta_x)-heading)
        distance = math.hypot(delta_x, delta_y)
        turn = max(-1.6, min(1.6, 2.2*error))
        speed = min(0.8, max(0.15, distance*0.7)) if abs(error) < 0.9 else 0.0
        cloud = _latest(observation, "lidar")
        if cloud is not None and observation.time_s-cloud.acquired_s <= 0.15:
            threats = [(px, py) for px, py, pz in cloud.data
                       if 0 < px < 1.5 and abs(py) < 0.55 and pz > -0.12]
            if threats:
                nearest_x, nearest_y = min(threats, key=lambda p: p[0])
                speed = 0.0 if nearest_x < 0.7 else min(speed, 0.2)
                turn = -1.1 if nearest_y > 0 else 1.1
        return NavigationCommand(speed, turn, observation.time_s+0.15)

    def close(self) -> None:
        pass


class CautiousFollower(WaypointFollower):
    """Independent speed policy for plug-in swapping and paired comparisons."""

    def step(self, observation: Observation) -> NavigationCommand:
        command = super().step(observation)
        return NavigationCommand(command.linear_mps*0.55, command.angular_radps*0.8,
                                 command.expires_s)


class ReferenceNavigator:
    """Cloud-derived costmap, A* path, wheel-command tracking and recovery."""
    manifest = PluginManifest("1", "0.1.0", ("complete-navigation",),
                              ("odometry", "lidar"))

    def reset(self, context: EpisodeContext) -> None:
        if not ((context.track == "complete-navigation" and "lidar" in context.capabilities) or
                (context.track == "planner-controller" and
                 "reference-navigation" in context.capabilities)):
            raise ValueError("ReferenceNavigator requires LiDAR or reference navigation")
        self.context = context
        self.map = ReferenceCostmap(context.max_grade_percent)
        self.last_cloud_time = -1.0
        self.last_plan_time = -1.0
        self.path = []
        self.last_progress = None
        self.last_progress_time = 0.0
        self.recovery = "NORMAL_FOLLOWING"
        self.recovery_until = 0.0

    def step(self, observation: Observation) -> NavigationCommand:
        now = observation.time_s
        perception_ms = None
        if observation.reference is not None:
            reference = observation.reference
            if reference.pose is None:
                return NavigationCommand(0, 0, now+0.1)
            x, y, heading = (reference.pose.x_m, reference.pose.y_m,
                             reference.pose.yaw_rad)
            self.map.resolution_m = reference.costmap_resolution_m
            self.map.cells = {(ix, iy): label for ix, iy, label in reference.costmap_cells}
            goal = reference.goal
        else:
            odom = _latest(observation, "odometry")
            if odom is None or now-odom.acquired_s > 0.2:
                return NavigationCommand(0, 0, now+0.1)
            x, y, heading = odom.data[:3]
            cloud = _latest(observation, "lidar")
            if cloud is not None and cloud.acquired_s > self.last_cloud_time:
                perception_ms = self.map.update(cloud.data, x, y, heading)
                self.last_cloud_time = cloud.acquired_s
            goal = self.context.goals[observation.goal_index]
        target = self._target_for(goal, observation, (x, y, heading))
        if target is None:
            return self._command(self._bootstrap_speed(), 0, now, perception_ms)
        if now-self.last_plan_time >= 0.5 or self.last_plan_time < 0:
            # The observed map has sparse edge returns; reserve a full body
            # length around marked cells until a calibrated footprint planner
            # replaces this conservative buffer.
            self.path = self.map.plan((x, y), target, self.context.robot_length_m)
            self.last_plan_time = now
        if self.last_progress is None:
            self.last_progress = (x, y)
            self.last_progress_time = now
        elif math.hypot(x-self.last_progress[0], y-self.last_progress[1]) > 0.25:
            self.last_progress = (x, y)
            self.last_progress_time = now
        if self.recovery == "NORMAL_FOLLOWING" and now-self.last_progress_time > 3.0:
            self.recovery = "BACKUP_RECOVERY"
            self.recovery_until = now+1.2
        if self.recovery == "BACKUP_RECOVERY":
            if now >= self.recovery_until:
                self.recovery = "SPIN_RECOVERY"
                self.recovery_until = now+1.0
            else:
                return self._command(-0.25, 0, now, perception_ms)
        if self.recovery == "SPIN_RECOVERY":
            if now >= self.recovery_until:
                self.recovery = "NORMAL_FOLLOWING"
                self.last_progress = (x, y)
                self.last_progress_time = now
                self.last_plan_time = -1
            else:
                return self._command(0, 0.9, now, perception_ms)
        lookahead = target
        for point in self.path:
            if math.hypot(point[0]-x, point[1]-y) > 2.0:
                lookahead = point
                break
        error = _angle(math.atan2(lookahead[1]-y, lookahead[0]-x)-heading)
        speed = 0.7 if abs(error) < 0.7 else 0.0
        if math.hypot(target[0]-x, target[1]-y) < 1.0:
            speed = min(speed, 0.35)
        turn = max(-1.5, min(1.5, 2.2*error))
        return self._command(speed, turn, now, perception_ms)

    def _target_for(self, goal, observation, pose):
        if goal.frame != "odom" or goal.x_m is None or goal.y_m is None:
            raise ValueError("ReferenceNavigator requires an odom-frame goal")
        return goal.x_m, goal.y_m

    def _bootstrap_speed(self):
        return 0.0

    def _command(self, speed, turn, now, perception_ms):
        diagnostics = [("recovery_state", self.recovery),
                       ("planned_points", str(len(self.path)))]
        if perception_ms is not None:
            diagnostics.append(("perception_ms", f"{perception_ms:.6f}"))
        return NavigationCommand(speed, turn, now+0.15, tuple(diagnostics))

    def close(self) -> None:
        pass


class FixedPipelinePlanner(ReferenceNavigator):
    """Planner/controller example using the common reference perception."""
    manifest = PluginManifest("1", "0.1.0", ("planner-controller",),
                              ("reference-navigation",))


class GPSLidarNavigator(ReferenceNavigator):
    """Joint GPS waypoint and LiDAR navigation on one shared robot run."""
    manifest = PluginManifest("1", "0.1.0", ("complete-navigation",),
                              ("odometry", "gps", "lidar"))

    def reset(self, context: EpisodeContext) -> None:
        super().reset(context)
        if any(goal.frame != "gps" for goal in context.goals):
            raise ValueError("GPSLidarNavigator requires GPS waypoint goals")
        self.origin_fix = None
        self.origin_odom = None
        self.rotation = None
        self.last_gps_time = -1.0

    def _target_for(self, goal, observation, pose):
        x, y, _ = pose
        gps = _latest(observation, "gps")
        if (gps is not None and gps.acquired_s > self.last_gps_time and
                observation.time_s-gps.acquired_s <= 0.2):
            self.last_gps_time = gps.acquired_s
            fix = (gps.data.latitude_deg, gps.data.longitude_deg)
            if self.origin_fix is None:
                self.origin_fix = fix
                self.origin_odom = (x, y)
            else:
                east, north = latlon_to_meters(*fix, *self.origin_fix)
                dx, dy = x-self.origin_odom[0], y-self.origin_odom[1]
                travel = math.hypot(dx, dy)
                if travel >= 0.8 and math.hypot(east, north) >= 0.5:
                    measured = _angle(math.atan2(north, east)-math.atan2(dy, dx))
                    if self.rotation is None:
                        self.rotation = measured
                    elif abs(_angle(measured-self.rotation)) < 0.4:
                        self.rotation = _angle(self.rotation+0.12*_angle(measured-self.rotation))
        if self.rotation is None:
            return None
        east, north = latlon_to_meters(goal.latitude_deg, goal.longitude_deg,
                                       *self.origin_fix)
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return (self.origin_odom[0]+c*east+s*north,
                self.origin_odom[1]-s*east+c*north)

    def _bootstrap_speed(self):
        return 0.45 if self.origin_fix is not None else 0.0


class GPSFollower:
    """GPS-only demonstration: project geographic displacement into odom.

    The reference requires motion before the ENU-to-odom rotation can be
    observed; this simple estimator is intentionally less robust than GPSEKF.
    """
    manifest = PluginManifest("1", "0.1.0", ("gps-only",),
                              ("odometry", "gps"))

    def reset(self, context: EpisodeContext) -> None:
        if context.track != "gps-only":
            raise ValueError("GPSFollower requires gps-only track")
        self.context = context
        self.origin_fix = None
        self.origin_odom = None
        self.rotation = None
        self.last_odom = None
        self.heading = 0.0

    def step(self, observation: Observation) -> NavigationCommand:
        odom = _latest(observation, "odometry")
        gps = _latest(observation, "gps")
        if odom is None:
            return NavigationCommand(0, 0, observation.time_s+0.1)
        x, y = odom.data[:2]
        if self.last_odom is not None and math.hypot(x-self.last_odom[0], y-self.last_odom[1]) > 0.015:
            self.heading = math.atan2(y-self.last_odom[1], x-self.last_odom[0])
        self.last_odom = (x, y)
        if gps is not None and observation.time_s-gps.acquired_s < 0.2:
            lat, lon = gps.data.latitude_deg, gps.data.longitude_deg
            if self.origin_fix is None:
                self.origin_fix = (lat, lon)
                self.origin_odom = (x, y)
            else:
                east, north = latlon_to_meters(lat, lon, *self.origin_fix)
                dx, dy = x-self.origin_odom[0], y-self.origin_odom[1]
                if math.hypot(dx, dy) > 0.8 and math.hypot(east, north) > 0.8:
                    measurement = math.atan2(north, east)-math.atan2(dy, dx)
                    self.rotation = measurement if self.rotation is None else _angle(
                        self.rotation + 0.05*_angle(measurement-self.rotation))
        if self.rotation is None:
            return NavigationCommand(0.45, 0, observation.time_s+0.15)
        goal = self.context.goals[observation.goal_index]
        east, north = latlon_to_meters(goal.latitude_deg, goal.longitude_deg, *self.origin_fix)
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        target_x = self.origin_odom[0]+c*east+s*north
        target_y = self.origin_odom[1]-s*east+c*north
        error = _angle(math.atan2(target_y-y, target_x-x)-self.heading)
        turn = max(-1.4, min(1.4, error*2))
        # Keep a slow arc while turning: GPS-only odometry exposes position,
        # not yaw, so pure spinning gives the estimator no heading evidence.
        speed = 0.55 if abs(error) < 0.7 else 0.2
        return NavigationCommand(speed, turn, observation.time_s+0.15)

    def close(self) -> None:
        pass
