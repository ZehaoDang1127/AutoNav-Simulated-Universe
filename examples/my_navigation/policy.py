"""Minimal external navigation plug-in. Run with --algorithm examples.my_navigation.policy:MyNavigator."""
from __future__ import annotations

import math

from autonav_universe.api import EpisodeContext, NavigationCommand, Observation, PluginManifest


class MyNavigator:
    manifest = PluginManifest("1", "0.1.0", ("complete-navigation",),
                              ("odometry",))
    def reset(self, context: EpisodeContext) -> None:
        if context.track != "complete-navigation":
            raise ValueError("this example needs local odometry goals")
        self.context = context
        self.speed_mps = min(context.max_linear_mps,
                             max(0.0, float(dict(context.parameters).get("speed_mps", "0.5"))))

    def step(self, observation: Observation) -> NavigationCommand:
        odom = next((s for s in observation.samples
                     if s.kind == "odometry" and s.valid
                     and observation.time_s-s.acquired_s < 0.2), None)
        if odom is None:
            return NavigationCommand(0, 0, observation.time_s+0.1)
        x, y, yaw = odom.data[:3]
        goal = self.context.goals[observation.goal_index]
        dx, dy = goal.x_m-x, goal.y_m-y
        error = math.atan2(math.sin(math.atan2(dy, dx)-yaw),
                           math.cos(math.atan2(dy, dx)-yaw))
        turn = max(-1.2, min(1.2, 2*error))
        speed = self.speed_mps if abs(error) < 0.5 else 0.0
        return NavigationCommand(speed, turn, observation.time_s+0.12)

    def close(self) -> None:
        pass
