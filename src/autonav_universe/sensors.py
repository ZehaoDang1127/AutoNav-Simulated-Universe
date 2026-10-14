"""Clocked sensor models. Geometry queries exist only in this engine module."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .api import GPSFix, SensorSample
from .dynamics import RobotState
from .scenarios import Scenario, meters_to_latlon


def elevation(scenario: Scenario, x: float, y: float) -> float:
    return sum(r.elevation(x, y) for r in scenario.ramps)


@dataclass
class OdomSensor:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def integrate(self, state: RobotState, dt: float, scenario: Scenario) -> None:
        speed = state.linear_mps * scenario.odom_linear_scale
        self.x_m += speed * math.cos(self.yaw_rad) * dt
        self.y_m += speed * math.sin(self.yaw_rad) * dt
        self.yaw_rad += (state.angular_radps + scenario.odom_yaw_bias_radps) * dt
        self.yaw_rad = math.atan2(math.sin(self.yaw_rad), math.cos(self.yaw_rad))

    def sample(self, time_s: float, state: RobotState, gps_only: bool = False) -> SensorSample:
        if gps_only:
            return SensorSample("odometry", time_s, time_s, "odom", True,
                                (self.x_m, self.y_m), (0.02, 0.02))
        return SensorSample("odometry", time_s, time_s, "odom", True,
                            (self.x_m, self.y_m, self.yaw_rad, state.linear_mps, state.angular_radps),
                            (0.02, 0.02, 0.01))


def gps_sample(scenario: Scenario, state: RobotState, time_s: float, rng: random.Random) -> SensorSample:
    x, y = state.x_m, state.y_m
    valid = True
    for fault in scenario.gps_faults:
        if fault.contains(x, y):
            if fault.kind == "blackout":
                valid = False
            elif fault.kind == "bias":
                x += fault.bias_east_m
                y += fault.bias_north_m
            elif fault.kind == "multipath":
                x += rng.gauss(0, max(0.1, abs(fault.bias_east_m)))
                y += rng.gauss(0, max(0.1, abs(fault.bias_north_m)))
            else:
                raise ValueError(f"unknown GPS fault {fault.kind}")
    if valid:
        if rng.random() < scenario.gps_dropout_probability:
            valid = False
    if valid:
        x += rng.gauss(0, scenario.gps_noise_std_m)
        y += rng.gauss(0, scenario.gps_noise_std_m)
        lat, lon = meters_to_latlon(x, y, scenario)
        data = GPSFix(lat, lon)
    else:
        data = None
    return SensorSample("gps", time_s, time_s, "gps", valid, data,
                        (scenario.gps_noise_std_m, scenario.gps_noise_std_m))


def _circle_ray_distance(ox: float, oy: float, dx: float, dy: float,
                         cx: float, cy: float, radius: float) -> float | None:
    px, py = ox-cx, oy-cy
    b = px*dx + py*dy
    c = px*px + py*py - radius*radius
    disc = b*b-c
    if disc < 0:
        return None
    t = -b-math.sqrt(disc)
    return t if t > 0 else None


def lidar_sample(scenario: Scenario, state: RobotState, time_s: float,
                 range_m: float = 8.0) -> SensorSample:
    """Returns a sparse 3D sensor-local cloud; absent rays are absent points.

    Ground returns are sampled on radial rings, while upright obstacles are
    raycast. This is a provisional LiDAR model, not the legacy STL raycaster.
    """
    cloud: list[tuple[float, float, float]] = []
    heading = state.yaw_rad
    sensor_z = elevation(scenario, state.x_m, state.y_m) + 0.3
    local_grade = 0.0
    for ramp in scenario.ramps:
        if ramp.contains(state.x_m, state.y_m):
            local_grade = ramp.grade_percent / 100
            break
    tilt = math.atan(local_grade)
    ct, st = math.cos(tilt), math.sin(tilt)
    for i in range(72):
        angle = 2*math.pi*i/72
        direction = heading + angle
        dx, dy = math.cos(direction), math.sin(direction)
        nearest = range_m
        for obstacle in scenario.obstacles:
            distance = _circle_ray_distance(state.x_m, state.y_m, dx, dy,
                                            obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            if distance is not None:
                nearest = min(nearest, distance)
        if nearest < range_m:
            lx, ly = nearest*math.cos(angle), nearest*math.sin(angle)
            cloud.append((ct*lx + st*0.3, ly, -st*lx + ct*0.3))
        for step in range(16):
            radius = 0.5 + 0.25*step
            if radius >= nearest or radius > range_m:
                break
            wx, wy = state.x_m+radius*dx, state.y_m+radius*dy
            if not (0 <= wx <= scenario.width_m and 0 <= wy <= scenario.height_m):
                break
            z = elevation(scenario, wx, wy)-sensor_z
            lx, ly = radius*math.cos(angle), radius*math.sin(angle)
            cloud.append((ct*lx+st*z, ly, -st*lx+ct*z))
    return SensorSample("lidar", time_s, time_s, "lidar", True, tuple(cloud))
