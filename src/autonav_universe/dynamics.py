"""BT/SPEED rear-axle sleigh and SPEED per-wheel PID, with shared parameters.

The 35 kg, 0.54 m track and 200 N envelope are provisional simulator values.
This planar model does not establish ramp traction, suspension, or tip safety.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotProfile:
    mass_kg: float = 35.0
    com_offset_m: float = 0.25
    wheelbase_m: float = 0.39
    track_width_m: float = 0.54
    footprint_half_width_m: float = 0.21
    footprint_back_m: float = 0.10
    footprint_forward_m: float = 0.44
    force_max_n: float = 200.0
    linear_damping: float = 6.0
    angular_damping: float = 2.0
    max_linear_mps: float = 2.2352
    max_angular_radps: float = 2.5

    @property
    def inertia_rear(self) -> float:
        length = self.footprint_back_m + self.footprint_forward_m
        width = 2 * self.footprint_half_width_m
        return self.mass_kg * (length * length + width * width) / 12 + self.mass_kg * self.com_offset_m**2


@dataclass
class WheelPID:
    kp: float = 120.0
    ki: float = 55.0
    kd: float = 28.0
    d_alpha: float = 0.35
    i_limit_n: float = 60.0
    integral: float = 0.0
    previous_measurement: float = 0.0
    filtered_derivative: float = 0.0

    def step(self, target: float, measured: float, dt: float, force_limit: float) -> float:
        error = target - measured
        derivative = (measured - self.previous_measurement) / dt if dt > 0 else 0.0
        self.filtered_derivative += self.d_alpha * (derivative - self.filtered_derivative)
        self.previous_measurement = measured
        cap = self.i_limit_n / self.ki if self.ki > 1e-9 else 0.0
        candidate = max(-cap, min(cap, self.integral + error * dt))
        raw = self.kp * error + self.ki * candidate - self.kd * self.filtered_derivative
        force = max(-force_limit, min(force_limit, raw))
        if not ((raw > force_limit and error > 0) or (raw < -force_limit and error < 0)):
            self.integral = candidate
        return force


@dataclass
class RobotState:
    x_m: float
    y_m: float
    yaw_rad: float
    linear_mps: float = 0.0
    angular_radps: float = 0.0
    left_force_n: float = 0.0
    right_force_n: float = 0.0

    def wheel_speeds(self, profile: RobotProfile) -> tuple[float, float]:
        half = profile.track_width_m / 2
        return self.linear_mps - self.angular_radps * half, self.linear_mps + self.angular_radps * half

    def integrate(self, dt: float, profile: RobotProfile, extra_damping: float = 0.0,
                  force_world: tuple[float, float] = (0.0, 0.0)) -> None:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        fx, fy = force_world
        along = fx * c + fy * s
        lateral = -fx * s + fy * c
        total = self.left_force_n + self.right_force_n
        torque = (self.right_force_n - self.left_force_n) * profile.track_width_m / 2
        du = (total + along + profile.mass_kg * profile.com_offset_m * self.angular_radps**2
              - (profile.linear_damping + extra_damping) * self.linear_mps) / profile.mass_kg
        dw = (torque + lateral * profile.com_offset_m
              - profile.mass_kg * profile.com_offset_m * self.linear_mps * self.angular_radps
              - (profile.angular_damping + 0.4 * extra_damping) * self.angular_radps) / profile.inertia_rear
        self.linear_mps += du * dt
        self.angular_radps += dw * dt
        self.x_m += self.linear_mps * c * dt
        self.y_m += self.linear_mps * s * dt
        self.yaw_rad = math.atan2(math.sin(self.yaw_rad + self.angular_radps * dt),
                                  math.cos(self.yaw_rad + self.angular_radps * dt))


def footprint_points(state: RobotState, profile: RobotProfile) -> tuple[tuple[float, float], ...]:
    c, s = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
    return tuple((state.x_m + forward*c - lateral*s, state.y_m + forward*s + lateral*c)
                 for forward in (-profile.footprint_back_m, profile.footprint_forward_m)
                 for lateral in (-profile.footprint_half_width_m, profile.footprint_half_width_m))
