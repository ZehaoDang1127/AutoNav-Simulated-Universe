"""Versioned, validated course descriptions; geometry stays private to the engine."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .api import Goal
from .dynamics import RobotProfile, RobotState, footprint_points


@dataclass(frozen=True)
class Circle:
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True)
class Ramp:
    x0_m: float
    x1_m: float
    y0_m: float
    y1_m: float
    grade_percent: float

    def contains(self, x: float, y: float) -> bool:
        return self.x0_m <= x <= self.x1_m and self.y0_m <= y <= self.y1_m

    def elevation(self, x: float, y: float) -> float:
        if not self.contains(x, y):
            return 0.0
        width = self.x1_m - self.x0_m
        return (x - self.x0_m) / width * self.grade_percent / 100 * width


@dataclass(frozen=True)
class GPSFault:
    kind: str
    x_m: float
    y_m: float
    radius_m: float
    bias_east_m: float = 0.0
    bias_north_m: float = 0.0

    def contains(self, x: float, y: float) -> bool:
        return math.hypot(x-self.x_m, y-self.y_m) <= self.radius_m


@dataclass(frozen=True)
class Scenario:
    name: str
    width_m: float
    height_m: float
    spawn: tuple[float, float, float]
    goals: tuple[tuple[float, float], ...]
    obstacles: tuple[Circle, ...] = ()
    ramps: tuple[Ramp, ...] = ()
    gps_faults: tuple[GPSFault, ...] = ()
    track: str = "complete-navigation"
    goal_frame: str = "odom"
    sensors: tuple[str, ...] = ("odometry", "gps", "lidar")
    max_time_s: float = 60.0
    arrival_radius_m: float = 0.6
    gps_origin_lat_deg: float = 37.23027
    gps_origin_lon_deg: float = -80.42504
    gps_noise_std_m: float = 0.15
    odom_rate_hz: float = 20.0
    gps_rate_hz: float = 10.0
    lidar_rate_hz: float = 20.0
    navigation_rate_hz: float = 20.0
    odom_latency_s: float = 0.0
    gps_latency_s: float = 0.0
    lidar_latency_s: float = 0.0
    gps_dropout_probability: float = 0.0
    odom_linear_scale: float = 1.0
    odom_yaw_bias_radps: float = 0.0
    grade_limit_percent: float = 20.0

    def public_goals(self) -> tuple[Goal, ...]:
        if self.track == "gps-only" or self.goal_frame == "gps":
            return tuple(Goal("gps", latitude_deg=meters_to_latlon(x, y, self)[0],
                              longitude_deg=meters_to_latlon(x, y, self)[1]) for x, y in self.goals)
        c, s = math.cos(self.spawn[2]), math.sin(self.spawn[2])
        return tuple(Goal("odom", x_m=(x-self.spawn[0])*c+(y-self.spawn[1])*s,
                          y_m=-(x-self.spawn[0])*s+(y-self.spawn[1])*c) for x, y in self.goals)

    def validate(self, profile: RobotProfile) -> None:
        if not self.name or self.width_m <= 0 or self.height_m <= 0 or self.max_time_s <= 0:
            raise ValueError("invalid scenario dimensions or duration")
        if self.track not in ("complete-navigation", "planner-controller", "gps-only"):
            raise ValueError("unsupported track")
        if self.goal_frame not in ("odom", "gps"):
            raise ValueError("unsupported goal frame")
        if self.track == "gps-only" and self.goal_frame != "gps":
            raise ValueError("GPS-only track requires GPS goals")
        if self.goal_frame == "gps" and "gps" not in self.sensors:
            raise ValueError("GPS goals require a GPS sensor")
        if self.track == "planner-controller" and self.goal_frame != "odom":
            raise ValueError("planner-controller prototype requires local goals")
        if not self.goals or len(set(self.goals)) != len(self.goals):
            raise ValueError("goals must be nonempty and distinct")
        if self.arrival_radius_m <= 0 or self.gps_noise_std_m < 0:
            raise ValueError("invalid sensor or arrival setting")
        if any(rate <= 0 or not math.isfinite(rate) for rate in
               (self.odom_rate_hz, self.gps_rate_hz, self.lidar_rate_hz, self.navigation_rate_hz)):
            raise ValueError("sensor and navigation rates must be positive")
        if any(latency < 0 or not math.isfinite(latency) for latency in
               (self.odom_latency_s, self.gps_latency_s, self.lidar_latency_s)):
            raise ValueError("sensor latency must be nonnegative")
        if not 0 <= self.gps_dropout_probability <= 1:
            raise ValueError("GPS dropout probability outside [0,1]")
        if self.track == "gps-only" and self.sensors != ("odometry", "gps"):
            raise ValueError("GPS-only track allows odometry and GPS only")
        if self.track == "planner-controller" and not {"odometry", "lidar"}.issubset(self.sensors):
            raise ValueError("planner-controller track requires odometry and LiDAR")
        if any(s not in ("odometry", "gps", "lidar") for s in self.sensors):
            raise ValueError("unknown sensor")
        if any(o.radius_m <= 0 for o in self.obstacles):
            raise ValueError("obstacle radius must be positive")
        if any(f.kind not in ("blackout", "bias", "multipath") or f.radius_m <= 0
               for f in self.gps_faults):
            raise ValueError("invalid GPS fault")
        for ramp in self.ramps:
            if ramp.x0_m >= ramp.x1_m or ramp.y0_m >= ramp.y1_m:
                raise ValueError("invalid ramp bounds")
        for x, y, yaw in (self.spawn,):
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError("invalid spawn")
        state = RobotState(*self.spawn)
        for x, y in footprint_points(state, profile):
            if not (0 < x < self.width_m and 0 < y < self.height_m):
                raise ValueError("robot footprint outside course at spawn")
            if any(r.contains(x, y) and abs(r.grade_percent) > self.grade_limit_percent
                   for r in self.ramps):
                raise ValueError("robot footprint on forbidden ramp at spawn")
        for o in self.obstacles:
            if _circle_footprint_contact(state, profile, o):
                raise ValueError("robot footprint collides at spawn")
        for x, y in self.goals:
            if not (0 < x < self.width_m and 0 < y < self.height_m):
                raise ValueError("goal outside course")
            if any(math.hypot(x-o.x_m, y-o.y_m) <= o.radius_m + profile.footprint_half_width_m
                   for o in self.obstacles):
                raise ValueError("goal inside obstacle")
            if any(r.contains(x, y) and abs(r.grade_percent) > self.grade_limit_percent
                   for r in self.ramps):
                raise ValueError("goal on forbidden ramp")
            if any(f.contains(x, y) for f in self.gps_faults):
                raise ValueError("goal inside GPS interference zone")


def _circle_footprint_contact(state: RobotState, profile: RobotProfile, circle: Circle) -> bool:
    dx, dy = circle.x_m-state.x_m, circle.y_m-state.y_m
    c, s = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
    forward, lateral = dx*c+dy*s, -dx*s+dy*c
    near_forward = max(-profile.footprint_back_m, min(profile.footprint_forward_m, forward))
    near_lateral = max(-profile.footprint_half_width_m, min(profile.footprint_half_width_m, lateral))
    return math.hypot(forward-near_forward, lateral-near_lateral) <= circle.radius_m


def meters_to_latlon(east: float, north: float, scenario: Scenario) -> tuple[float, float]:
    radius = 6_371_000.0
    lat = scenario.gps_origin_lat_deg + math.degrees(north / radius)
    lon = scenario.gps_origin_lon_deg + math.degrees(east / (radius * math.cos(math.radians(scenario.gps_origin_lat_deg))))
    return lat, lon


def latlon_to_meters(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    radius = 6_371_000.0
    return (math.radians(lon-origin_lon) * radius * math.cos(math.radians(origin_lat)),
            math.radians(lat-origin_lat) * radius)


def load_scenario(name_or_path: str) -> Scenario:
    path = Path(name_or_path)
    if not path.exists():
        path = Path(__file__).resolve().parents[2] / "scenarios" / (name_or_path + ".json")
    if not path.exists():
        path = Path(__file__).resolve().parent / "data" / "scenarios" / (name_or_path + ".json")
    if not path.exists():
        raise FileNotFoundError(f"scenario {name_or_path!r} not found")
    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema_version") != 1:
        raise ValueError("unsupported scenario schema")
    scenario = Scenario(
        name=raw["name"], width_m=float(raw["width_m"]), height_m=float(raw["height_m"]),
        spawn=tuple(map(float, raw["spawn"])), goals=tuple(tuple(map(float, g)) for g in raw["goals"]),
        obstacles=tuple(Circle(*map(float, o)) for o in raw.get("obstacles", [])),
        ramps=tuple(Ramp(*map(float, r)) for r in raw.get("ramps", [])),
        gps_faults=tuple(GPSFault(f["kind"], *map(float, f["center_radius"]),
                                  *map(float, f.get("bias_m", (0, 0)))) for f in raw.get("gps_faults", [])),
        track=raw.get("track", "complete-navigation"),
        goal_frame=raw.get("goal_frame", "gps" if raw.get("track") == "gps-only" else "odom"),
        sensors=tuple(raw.get("sensors", ["odometry", "gps", "lidar"])),
        max_time_s=float(raw.get("max_time_s", 60)),
        arrival_radius_m=float(raw.get("arrival_radius_m", 0.6)),
        gps_noise_std_m=float(raw.get("gps_noise_std_m", 0.15)),
        odom_rate_hz=float(raw.get("odom_rate_hz", 20)),
        gps_rate_hz=float(raw.get("gps_rate_hz", 10)),
        lidar_rate_hz=float(raw.get("lidar_rate_hz", 20)),
        navigation_rate_hz=float(raw.get("navigation_rate_hz", 20)),
        odom_latency_s=float(raw.get("odom_latency_s", 0)),
        gps_latency_s=float(raw.get("gps_latency_s", 0)),
        lidar_latency_s=float(raw.get("lidar_latency_s", 0)),
        gps_dropout_probability=float(raw.get("gps_dropout_probability", 0)),
        odom_linear_scale=float(raw.get("odom_linear_scale", 1.0)),
        odom_yaw_bias_radps=float(raw.get("odom_yaw_bias_radps", 0.0)),
        grade_limit_percent=float(raw.get("grade_limit_percent", 20.0)),
    )
    scenario.validate(RobotProfile())
    return scenario
