"""Version 1 public plug-in contract. No engine state crosses this boundary."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

API_VERSION = "1"


@dataclass(frozen=True)
class PluginManifest:
    api_version: str
    algorithm_version: str
    supported_tracks: tuple[str, ...]
    required_sensors: tuple[str, ...]
    requested_update_hz: float = 20.0


@dataclass(frozen=True)
class Pose2:
    x_m: float
    y_m: float
    yaw_rad: float
    frame: str


@dataclass(frozen=True)
class Goal:
    frame: str
    x_m: float | None = None
    y_m: float | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None


@dataclass(frozen=True)
class GPSFix:
    latitude_deg: float
    longitude_deg: float


@dataclass(frozen=True)
class SensorSample:
    kind: str
    acquired_s: float
    delivered_s: float
    frame: str
    valid: bool
    data: GPSFix | tuple[float, ...] | tuple[tuple[float, ...], ...] | None
    uncertainty: tuple[float, ...] = ()
    stale: bool = False


@dataclass(frozen=True)
class ReferenceNavigation:
    """Fixed, sensor-derived inputs for the planner/controller track."""
    pipeline_version: str
    pose: Pose2 | None
    goal: Goal
    costmap_resolution_m: float
    costmap_cells: tuple[tuple[int, int, str], ...]
    lidar_acquired_s: float | None
    perception_ms: float | None


@dataclass(frozen=True)
class EpisodeContext:
    api_version: str
    episode_id: str
    track: str
    robot_width_m: float
    robot_length_m: float
    track_width_m: float
    max_linear_mps: float
    max_angular_radps: float
    max_grade_percent: float
    navigation_rate_hz: float
    capabilities: tuple[str, ...]
    goals: tuple[Goal, ...]
    algorithm_seed: int
    parameters: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class Observation:
    episode_id: str
    sequence: int
    time_s: float
    dt_s: float
    goal_index: int
    samples: tuple[SensorSample, ...]
    reference: ReferenceNavigation | None = None


@dataclass(frozen=True)
class NavigationCommand:
    linear_mps: float
    angular_radps: float
    expires_s: float
    diagnostics: tuple[tuple[str, str], ...] = ()


class NavigationAlgorithm(Protocol):
    manifest: PluginManifest
    def reset(self, context: EpisodeContext) -> None: ...
    def step(self, observation: Observation) -> NavigationCommand: ...
    def close(self) -> None: ...
