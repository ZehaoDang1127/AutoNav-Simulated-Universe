"""One deterministic simulation clock, world state, sensors, controller and evaluator."""
from __future__ import annotations

import hashlib
import heapq
import math
import random
import threading
import time
from dataclasses import asdict, dataclass, replace
from typing import Callable

from .api import (API_VERSION, EpisodeContext, NavigationCommand, Observation,
                  Pose2, ReferenceNavigation, SensorSample)
from .dynamics import RobotProfile, RobotState, WheelPID, footprint_points
from .perception import ReferenceCostmap
from .scenarios import Scenario, _circle_footprint_contact
from .sensors import OdomSensor, gps_sample, lidar_sample
from .worker import AlgorithmWorker


@dataclass(frozen=True)
class EpisodeResult:
    episode_id: str
    scenario: str
    algorithm: str
    seed: int
    outcome: str
    time_s: float
    distance_m: float
    goals_reached: int
    total_goals: int
    goal_center_error_m: float
    footprint_goal_fraction: float
    contacts: int
    first_contact_time_s: float | None
    min_clearance_m: float | None
    ramp_traversals: int
    commands: int
    command_saturations: int
    tracking_error_rms_mps: float
    perception_max_ms: float | None
    perception_violations_60ms: int | None
    sensor_queue_drops: int
    deadline_overruns: int
    skipped_observations: int
    algorithm_wall_p50_ms: float
    algorithm_wall_p95_ms: float
    algorithm_wall_max_ms: float
    timing_mode: str


def _stream_seed(seed: int, label: str) -> int:
    return int.from_bytes(hashlib.sha256(f"{seed}:{label}".encode()).digest()[:8], "big")


def footprint_goal_fraction(state: RobotState, profile: RobotProfile,
                            goal: tuple[float, float], radius: float) -> float:
    """Exact circle/footprint intersection area divided by rectangle area."""
    c, s = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
    local = ((-profile.footprint_back_m, -profile.footprint_half_width_m),
             (profile.footprint_forward_m, -profile.footprint_half_width_m),
             (profile.footprint_forward_m, profile.footprint_half_width_m),
             (-profile.footprint_back_m, profile.footprint_half_width_m))
    vertices = [(state.x_m+x*c-y*s-goal[0], state.y_m+x*s+y*c-goal[1])
                for x, y in local]

    def cross(a, b):
        return a[0]*b[1]-a[1]*b[0]

    area = 0.0
    for a, b in zip(vertices, vertices[1:]+vertices[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        aa = dx*dx+dy*dy
        bb = 2*(a[0]*dx+a[1]*dy)
        cc = a[0]*a[0]+a[1]*a[1]-radius*radius
        roots = []
        discriminant = bb*bb-4*aa*cc
        if discriminant > 0 and aa > 0:
            root = math.sqrt(discriminant)
            roots = [t for t in ((-bb-root)/(2*aa), (-bb+root)/(2*aa))
                     if 0 < t < 1]
        marks = [0.0]+sorted(roots)+[1.0]
        for left, right in zip(marks, marks[1:]):
            u = (a[0]+left*dx, a[1]+left*dy)
            v = (a[0]+right*dx, a[1]+right*dy)
            mid = (a[0]+(left+right)*dx/2, a[1]+(left+right)*dy/2)
            wedge = cross(u, v)
            if mid[0]*mid[0]+mid[1]*mid[1] <= radius*radius:
                area += wedge/2
            else:
                area += radius*radius*math.atan2(wedge, u[0]*v[0]+u[1]*v[1])/2
    footprint_area = (profile.footprint_back_m+profile.footprint_forward_m) * (
        2*profile.footprint_half_width_m)
    return max(0.0, min(1.0, abs(area)/footprint_area))


class EpisodeRunner:
    def __init__(self, scenario: Scenario, algorithm: str, seed: int = 0,
                 algorithm_timeout_s: float = 2.0, capture_observations: bool = False,
                 parameters: tuple[tuple[str, str], ...] = (),
                 timing_mode: str = "functional", action_budget_s: float = 0.05):
        if timing_mode not in ("functional", "deadline"):
            raise ValueError("timing mode must be functional or deadline")
        if action_budget_s <= 0 or not math.isfinite(action_budget_s):
            raise ValueError("action budget must be positive")
        self.profile = RobotProfile()
        scenario.validate(self.profile)
        self.scenario, self.algorithm, self.seed = scenario, algorithm, seed
        self.parameters = parameters
        self.timing_mode = timing_mode
        self.action_budget_s = action_budget_s
        self._pending = None
        self.deadline_overruns = 0
        self.skipped_observations = 0
        self.observation_sequence = 0
        self.episode_id = f"{scenario.name}-{seed}"
        self.state = RobotState(*scenario.spawn)
        self.odom = OdomSensor()
        self.rng_gps = random.Random(_stream_seed(seed, "gps"))
        self.left_pid, self.right_pid = WheelPID(), WheelPID()
        self.time_s = 0.0
        self.goal_index = 0
        self.distance_m = 0.0
        self.contacts = 0
        self.first_contact_time_s: float | None = None
        self.min_clearance_m = math.inf
        self.ramp_traversals = 0
        self._ramp_entry: dict[int, str] = {}
        self._ramp_last: dict[int, str] = {}
        self.command_count = 0
        self.command_saturations = 0
        self.tracking_error_sq_sum = 0.0
        self.tracking_error_count = 0
        self.perception_ms: list[float] = []
        self.reference_costmap = (ReferenceCostmap(scenario.grade_limit_percent)
                                  if scenario.track == "planner-controller" else None)
        self._reference_cloud_time = -1.0
        self.algorithm_wall_ms: list[float] = []
        self.events: list[dict] = []
        self.truth: list[dict] = []
        self.observations: list[dict] = []
        self.capture_observations = capture_observations
        self.samples: dict[str, SensorSample] = {}
        self._delivery_queue: list[tuple[float, int, SensorSample]] = []
        self._sample_counter = 0
        self.sensor_queue_drops = 0
        self.command = NavigationCommand(0.0, 0.0, 0.0)
        self.applied = (0.0, 0.0)
        self.outcome: str | None = None
        self._last_nav_s = 0.0
        self._deadline = {"odometry": 0.0, "gps": 0.0, "lidar": 0.0,
                          "navigation": 0.0, "controller": 0.0, "record": 0.0}
        self._period = {"odometry": 1/scenario.odom_rate_hz,
                        "gps": 1/scenario.gps_rate_hz,
                        "lidar": 1/scenario.lidar_rate_hz,
                        "navigation": 1/scenario.navigation_rate_hz,
                        "controller": 0.02, "record": 0.1}
        self._worker = None
        self.plugin_manifest = None
        self._algorithm_timeout_s = algorithm_timeout_s

    def _context(self) -> EpisodeContext:
        p = self.profile
        return EpisodeContext(API_VERSION, self.episode_id, self.scenario.track,
                              2*p.footprint_half_width_m,
                              p.footprint_back_m+p.footprint_forward_m,
                              p.track_width_m, p.max_linear_mps, p.max_angular_radps,
                              self.scenario.grade_limit_percent,
                              self.scenario.navigation_rate_hz,
                              (("reference-navigation",) if self.reference_costmap is not None
                               else self.scenario.sensors), self.scenario.public_goals(),
                              _stream_seed(self.seed, "algorithm"), self.parameters)

    def _safety(self) -> None:
        state, scenario, profile = self.state, self.scenario, self.profile
        points = footprint_points(state, profile)
        boundary_clearance = min(min(x, scenario.width_m-x, y, scenario.height_m-y)
                                 for x, y in points)
        obstacle_clearance = math.inf
        c, s = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
        for obstacle in scenario.obstacles:
            dx, dy = obstacle.x_m-state.x_m, obstacle.y_m-state.y_m
            forward, lateral = dx*c+dy*s, -dx*s+dy*c
            near_forward = max(-profile.footprint_back_m,
                               min(profile.footprint_forward_m, forward))
            near_lateral = max(-profile.footprint_half_width_m,
                               min(profile.footprint_half_width_m, lateral))
            clearance = math.hypot(forward-near_forward, lateral-near_lateral)-obstacle.radius_m
            obstacle_clearance = min(obstacle_clearance, clearance)
        self.min_clearance_m = min(self.min_clearance_m, boundary_clearance, obstacle_clearance)
        if any(not (0 <= x <= scenario.width_m and 0 <= y <= scenario.height_m) for x, y in points):
            self.outcome = "out_of_bounds"
        elif any(_circle_footprint_contact(state, profile, o) for o in scenario.obstacles):
            self.contacts += 1
            self.first_contact_time_s = self.time_s
            self.outcome = "collision"
        elif any(r.contains(x, y) and abs(r.grade_percent) > scenario.grade_limit_percent
                 for r in scenario.ramps for x, y in points):
            self.outcome = "unsafe_terrain"

    def _advance(self, target_s: float) -> None:
        while self.time_s + 1e-10 < target_s and self.outcome is None:
            dt = min(1/240, target_s-self.time_s)
            before = (self.state.x_m, self.state.y_m)
            self.state.integrate(dt, self.profile)
            self.odom.integrate(self.state, dt, self.scenario)
            self.time_s += dt
            self.distance_m += math.hypot(self.state.x_m-before[0], self.state.y_m-before[1])
            self._safety()  # before any collision correction or goal credit
            if self.outcome is None:
                self._track_ramps()
        if self.outcome is None:
            self.time_s = target_s

    def _track_ramps(self) -> None:
        for index, ramp in enumerate(self.scenario.ramps):
            x, y = self.state.x_m, self.state.y_m
            side = ("inside" if ramp.contains(x, y) else
                    "left" if x < ramp.x0_m and ramp.y0_m <= y <= ramp.y1_m else
                    "right" if x > ramp.x1_m and ramp.y0_m <= y <= ramp.y1_m else "outside")
            previous = self._ramp_last.get(index, "outside")
            if side == "inside" and previous in ("left", "right"):
                self._ramp_entry[index] = previous
            elif side in ("left", "right") and previous == "inside":
                if self._ramp_entry.pop(index, None) == {"left": "right", "right": "left"}[side]:
                    self.ramp_traversals += 1
                    self.events.append({"time_s": self.time_s, "type": "ramp_traversal", "index": index})
            elif side == "outside":
                self._ramp_entry.pop(index, None)
            self._ramp_last[index] = side

    def _sample(self, name: str) -> None:
        if name not in self.scenario.sensors:
            return
        if name == "odometry":
            sample = self.odom.sample(self.time_s, self.state,
                                      self.scenario.track == "gps-only")
            latency = self.scenario.odom_latency_s
        elif name == "gps":
            sample = gps_sample(self.scenario, self.state, self.time_s, self.rng_gps)
            latency = self.scenario.gps_latency_s
        elif name == "lidar":
            sample = lidar_sample(self.scenario, self.state, self.time_s)
            latency = self.scenario.lidar_latency_s
        self._sample_counter += 1
        sample = replace(sample, delivered_s=self.time_s+latency)
        if len(self._delivery_queue) >= 256:
            _, _, dropped = heapq.heappop(self._delivery_queue)
            self.sensor_queue_drops += 1
            self.events.append({"time_s": self.time_s, "type": "sensor_queue_drop",
                                "kind": dropped.kind, "acquired_s": dropped.acquired_s})
        heapq.heappush(self._delivery_queue, (sample.delivered_s, self._sample_counter, sample))

    def _deliver(self) -> None:
        while self._delivery_queue and self._delivery_queue[0][0] <= self.time_s+1e-9:
            _, _, sample = heapq.heappop(self._delivery_queue)
            self.samples[sample.kind] = sample

    def _navigation(self) -> None:
        samples = tuple(replace(self.samples[k],
                                stale=self.time_s-self.samples[k].acquired_s >
                                2*self._period[k]+1e-9)
                        for k in sorted(self.samples))
        reference = self._reference_navigation() if self.reference_costmap is not None else None
        observation = Observation(self.episode_id, self.observation_sequence, self.time_s,
                                  self.time_s-self._last_nav_s, self.goal_index,
                                  () if reference is not None else samples, reference)
        self.observation_sequence += 1
        if self.capture_observations:
            self.observations.append(asdict(observation))
        self._last_nav_s = self.time_s
        if self.timing_mode == "deadline":
            if self._pending is None:
                try:
                    self._worker.submit(observation)
                    self._pending = (observation, time.perf_counter(), False)
                except Exception as exc:
                    self.outcome = "algorithm_error"
                    self.events.append({"time_s": self.time_s, "type": "algorithm_error",
                                        "message": str(exc)})
            else:
                self.skipped_observations += 1
                self.events.append({"time_s": self.time_s, "type": "observation_skipped",
                                    "sequence": observation.sequence})
            return
        start = time.perf_counter()
        try:
            requested = self._worker.step(observation)
        except TimeoutError as exc:
            self.outcome = "algorithm_timeout"
            self.events.append({"time_s": self.time_s, "type": "algorithm_timeout", "message": str(exc)})
            return
        except Exception as exc:
            self.outcome = "algorithm_error"
            self.events.append({"time_s": self.time_s, "type": "algorithm_error", "message": str(exc)})
            return
        wall_ms = (time.perf_counter()-start)*1000
        self._accept_command(requested, observation, wall_ms)

    def _reference_navigation(self) -> ReferenceNavigation:
        """Build the shared estimate and map using delivered public samples only."""
        odom = self.samples.get("odometry")
        cloud = self.samples.get("lidar")
        pose = None
        perception_ms = None
        if odom is not None and odom.valid and self.time_s-odom.acquired_s <= 0.2:
            x, y, yaw = odom.data[:3]
            pose = Pose2(x, y, yaw, "odom")
        if (pose is not None and cloud is not None and cloud.valid and
                cloud.acquired_s > self._reference_cloud_time and
                self.time_s-cloud.acquired_s <= 0.15):
            perception_ms = self.reference_costmap.update(
                cloud.data, pose.x_m, pose.y_m, pose.yaw_rad)
            self.perception_ms.append(perception_ms)
            self._reference_cloud_time = cloud.acquired_s
        return ReferenceNavigation(
            "pca-costmap-v1", pose, self.scenario.public_goals()[self.goal_index],
            self.reference_costmap.resolution_m,
            tuple((x, y, label) for (x, y), label in sorted(self.reference_costmap.cells.items())),
            self._reference_cloud_time if self._reference_cloud_time >= 0 else None,
            perception_ms)

    def _poll_deadline(self) -> None:
        if self._pending is None or self.outcome is not None:
            return
        observation, dispatched_wall, overrun_recorded = self._pending
        wall_elapsed = time.perf_counter()-dispatched_wall
        if wall_elapsed > self._algorithm_timeout_s:
            self.outcome = "algorithm_timeout"
            self.events.append({"time_s": self.time_s, "type": "algorithm_timeout",
                                "sequence": observation.sequence})
            return
        try:
            response = self._worker.poll()
        except Exception as exc:
            self.outcome = "algorithm_error"
            self.events.append({"time_s": self.time_s, "type": "algorithm_error",
                                "message": str(exc)})
            return
        if response is not None:
            requested, compute_s = response
            if (overrun_recorded or wall_elapsed > self.action_budget_s+1e-9 or
                    self.time_s > observation.time_s+self.action_budget_s+1e-9):
                if not overrun_recorded:
                    self.deadline_overruns += 1
                self.events.append({"time_s": self.time_s, "type": "late_action_discarded",
                                    "sequence": observation.sequence, "compute_ms": compute_s*1000})
            else:
                self._accept_command(requested, observation, compute_s*1000)
            self._pending = None
        elif self.time_s >= observation.time_s+self.action_budget_s-1e-9 and not overrun_recorded:
            self.deadline_overruns += 1
            self.events.append({"time_s": self.time_s, "type": "deadline_overrun",
                                "sequence": observation.sequence})
            self._pending = (observation, dispatched_wall, True)

    def _accept_command(self, requested: NavigationCommand,
                        observation: Observation, wall_ms: float) -> None:
        self.algorithm_wall_ms.append(wall_ms)
        values = (requested.linear_mps, requested.angular_radps, requested.expires_s)
        valid_values = all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
        valid_diagnostics = (isinstance(requested.diagnostics, tuple) and
                             len(requested.diagnostics) <= 64 and
                             all(isinstance(item, tuple) and len(item) == 2 and
                                 all(isinstance(part, str) and len(part) <= 4096 for part in item)
                                 for item in requested.diagnostics))
        if not valid_values or not valid_diagnostics or requested.expires_s <= self.time_s:
            self.outcome = "invalid_action"
            self.events.append({"time_s": self.time_s, "type": "invalid_action"})
            return
        p = self.profile
        linear = max(-p.max_linear_mps, min(p.max_linear_mps, requested.linear_mps))
        angular = max(-p.max_angular_radps, min(p.max_angular_radps, requested.angular_radps))
        if linear != requested.linear_mps or angular != requested.angular_radps:
            self.command_saturations += 1
        self.command = NavigationCommand(linear, angular, requested.expires_s, requested.diagnostics)
        diag = dict(requested.diagnostics)
        if self.reference_costmap is None and "perception_ms" in diag:
            try:
                elapsed = float(diag["perception_ms"])
                if math.isfinite(elapsed) and elapsed >= 0:
                    self.perception_ms.append(elapsed)
            except ValueError:
                pass
        self.command_count += 1
        self.events.append({"time_s": self.time_s, "type": "command", "sequence": observation.sequence,
                            "requested": asdict(requested), "applied": asdict(self.command),
                            "wall_ms": wall_ms})

    def _controller(self) -> None:
        linear, angular = (self.command.linear_mps, self.command.angular_radps)
        if self.time_s >= self.command.expires_s - 1e-9:
            linear, angular = 0.0, 0.0
        half = self.profile.track_width_m/2
        left_meas, right_meas = self.state.wheel_speeds(self.profile)
        self.tracking_error_sq_sum += ((linear-angular*half-left_meas)**2 +
                                       (linear+angular*half-right_meas)**2)/2
        self.tracking_error_count += 1
        self.state.left_force_n = self.left_pid.step(linear-angular*half, left_meas, 0.02,
                                                     self.profile.force_max_n)
        self.state.right_force_n = self.right_pid.step(linear+angular*half, right_meas, 0.02,
                                                       self.profile.force_max_n)
        self.applied = (linear, angular)

    def _check_goal(self) -> None:
        goal = self.scenario.goals[self.goal_index]
        if math.hypot(self.state.x_m-goal[0], self.state.y_m-goal[1]) > self.scenario.arrival_radius_m + 0.6:
            return
        fraction = footprint_goal_fraction(self.state, self.profile, goal,
                                           self.scenario.arrival_radius_m)
        if fraction >= 0.5:
            self.events.append({"time_s": self.time_s, "type": "goal", "index": self.goal_index,
                                "footprint_fraction": fraction})
            self.goal_index += 1
            if self.goal_index == len(self.scenario.goals):
                self.outcome = "success"

    def _record(self) -> None:
        self.truth.append({"time_s": round(self.time_s, 8), "x_m": self.state.x_m,
                           "y_m": self.state.y_m, "yaw_rad": self.state.yaw_rad,
                           "linear_mps": self.state.linear_mps, "angular_radps": self.state.angular_radps,
                           "goal_index": self.goal_index})

    def run(self, on_frame: Callable[["EpisodeRunner"], None] | None = None,
            running_event: threading.Event | None = None,
            stop_event: threading.Event | None = None,
            realtime: bool = False) -> EpisodeResult:
        try:
            try:
                self._worker = AlgorithmWorker(self.algorithm, self._context(), self._algorithm_timeout_s)
                self.plugin_manifest = self._worker.manifest
                self.events.append({"time_s": 0.0, "type": "plugin_manifest",
                                    "manifest": asdict(self.plugin_manifest)})
            except TimeoutError:
                self.outcome = "algorithm_timeout"
            except Exception as exc:
                self.outcome = "algorithm_error"
                self.events.append({"time_s": 0.0, "type": "algorithm_error", "message": str(exc)})
            if self.outcome is None:
                self._run_events(on_frame, running_event, stop_event, realtime)
        finally:
            if self._worker is not None:
                self._worker.close()
        goal = self.scenario.goals[min(self.goal_index, len(self.scenario.goals)-1)]
        error = math.hypot(self.state.x_m-goal[0], self.state.y_m-goal[1])
        fraction = footprint_goal_fraction(self.state, self.profile, goal,
                                           self.scenario.arrival_radius_m)
        timings = sorted(self.algorithm_wall_ms)
        p95 = timings[min(len(timings)-1, math.ceil(len(timings)*0.95)-1)] if timings else 0.0
        p50 = timings[min(len(timings)-1, math.ceil(len(timings)*0.5)-1)] if timings else 0.0
        return EpisodeResult(self.episode_id, self.scenario.name, self.algorithm, self.seed,
                             self.outcome or "timeout", self.time_s, self.distance_m,
                             self.goal_index, len(self.scenario.goals), error, fraction,
                             self.contacts, self.first_contact_time_s,
                             self.min_clearance_m if math.isfinite(self.min_clearance_m) else None,
                             self.ramp_traversals, self.command_count, self.command_saturations,
                             math.sqrt(self.tracking_error_sq_sum/self.tracking_error_count)
                             if self.tracking_error_count else 0.0,
                             max(self.perception_ms) if self.perception_ms else None,
                             sum(ms > 60 for ms in self.perception_ms) if self.perception_ms else None,
                             self.sensor_queue_drops, self.deadline_overruns,
                             self.skipped_observations,
                             p50, p95, max(timings, default=0.0), self.timing_mode)

    def _run_events(self, on_frame, running_event, stop_event, realtime) -> None:
        wall_start = time.perf_counter()
        while self.outcome is None:
            if stop_event is not None and stop_event.is_set():
                self.outcome = "cancelled"
                break
            if running_event is not None:
                wait_start = time.perf_counter()
                while not running_event.wait(0.05):
                    if stop_event is not None and stop_event.is_set():
                        self.outcome = "cancelled"
                        break
                wall_start += time.perf_counter()-wait_start
                if self.outcome is not None:
                    break
            delivery_time = self._delivery_queue[0][0] if self._delivery_queue else math.inf
            deadline_time = math.inf
            if self.timing_mode == "deadline" and self._pending is not None and not self._pending[2]:
                deadline_time = self._pending[0].time_s+self.action_budget_s
            target = min(*self._deadline.values(), delivery_time, deadline_time,
                         self.scenario.max_time_s)
            if self.timing_mode == "deadline":
                delay = wall_start+target-time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
            self._advance(target)
            if self.outcome is not None:
                break
            for name in ("odometry", "gps", "lidar"):
                if self._deadline[name] <= self.time_s + 1e-9:
                    self._sample(name)
                    self._deadline[name] += self._period[name]
            self._deliver()
            if self.timing_mode == "deadline":
                self._poll_deadline()
                if self.outcome is not None:
                    break
            for name in ("navigation", "controller", "record"):
                if self._deadline[name] <= self.time_s + 1e-9:
                    if name == "navigation":
                        self._navigation()
                    elif name == "controller":
                        self._controller()
                    else:
                        self._record()
                        if on_frame is not None:
                            on_frame(self)
                        if realtime and self.timing_mode == "functional":
                            delay = wall_start+self.time_s-time.perf_counter()
                            if delay > 0:
                                time.sleep(min(delay, 0.1))
                    self._deadline[name] += self._period[name]
                    if self.outcome is not None:
                        break
            if self.outcome is None:
                self._check_goal()
            if self.outcome is None and self.time_s >= self.scenario.max_time_s - 1e-9:
                self.outcome = "timeout"
        self._record()
