"""Deliberately small worker fixtures used by integration tests."""
import time

from autonav_universe.api import NavigationCommand, PluginManifest


class Forward:
    manifest = PluginManifest("1", "test", ("complete-navigation", "gps-only"), ())
    def reset(self, context):
        assert not hasattr(context, "world")

    def step(self, observation):
        assert not hasattr(observation, "world")
        return NavigationCommand(0.5, 0, observation.time_s+0.1)

    def close(self):
        pass


class Invalid(Forward):
    def step(self, observation):
        return NavigationCommand(float("nan"), 0, observation.time_s+0.1)


class BadDiagnostics(Forward):
    def step(self, observation):
        return NavigationCommand(0.5, 0, observation.time_s+0.1, (("bad", object()),))


class Crash(Forward):
    def step(self, observation):
        raise RuntimeError("fixture failure")


class Hang(Forward):
    def step(self, observation):
        time.sleep(1.0)
        return NavigationCommand(0, 0, observation.time_s+0.1)


class Slow(Forward):
    def step(self, observation):
        time.sleep(0.10)
        return NavigationCommand(0.5, 0, observation.time_s+0.2)


class WrongVersion(Forward):
    manifest = PluginManifest("2", "test", ("complete-navigation",), ())


class NeedsLidar(Forward):
    manifest = PluginManifest("1", "test", ("gps-only",), ("lidar",))
