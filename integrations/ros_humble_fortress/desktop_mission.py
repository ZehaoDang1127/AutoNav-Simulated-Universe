"""Run the pinned mission client under desktop control and a sensor watchdog.

The command gate independently reads the same desktop lease. Pausing first
inhibits that gate, then freezes Gazebo physics; all mission wall budgets
exclude the acknowledged pause. Reset is a new container, never a pose jump.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from nav_msgs.msg import Odometry

from autonav_universe.full_track import ControlLease, MissionClock, read_json, write_json
from igvc_competition_sim.mission_runner import MissionRunner


class DesktopMission(MissionRunner):
    def __init__(self, args):
        self.folder = Path(os.environ["RUN_DIR"])
        self.lease = ControlLease()
        self.active_clock = MissionClock(time.monotonic())
        self.paused = False
        self.finishing = False
        self.control_failure = None
        self.phase = None
        self.last_tick = time.monotonic()
        self.last_sensor = {}
        self.watchdog_grace = time.monotonic() + 8.0
        self.last_clock_stamp = None
        world = ET.parse(os.environ["WORLD_OVERRIDE"]).getroot().find("world")
        self.world_name = world.attrib["name"]
        super().__init__(args.course_config, args.timeout_sec,
                         require_command_gate=True, total_timeout_sec=args.total_timeout_sec)
        for topic, kind in (("/scan_fullframe", LaserScan), ("/imu/data", Imu),
                            ("/gps_fix", NavSatFix), ("/local_ekf/odom", Odometry)):
            self.create_subscription(kind, topic, lambda msg, name=topic:
                                     self.last_sensor.__setitem__(name, time.monotonic()),
                                     qos_profile_sensor_data)
        self.create_subscription(ClockMessage, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_timer(0.1, self.control_tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.report("Ready")

    def on_clock(self, message):
        stamp = (message.clock.sec, message.clock.nanosec)
        if stamp != self.last_clock_stamp:
            self.last_sensor["/clock"] = time.monotonic()
        self.last_clock_stamp = stamp

    def report(self, phase, reason=None):
        if (phase, reason) == self.phase:
            return
        self.phase = (phase, reason)
        event = {"state": phase, "reason": reason, "wall_s": time.time()}
        write_json(self.folder / "desktop_status.json", event)
        with (self.folder / "desktop_events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event) + "\n")

    def set_physics_pause(self, pause):
        response = subprocess.run([
            "ign", "service", "-s", f"/world/{self.world_name}/control",
            "--reqtype", "ignition.msgs.WorldControl", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "2000", "--req", "pause: " + str(pause).lower()],
            capture_output=True, text=True, timeout=3)
        if response.returncode or "data: true" not in response.stdout:
            raise RuntimeError("Gazebo did not acknowledge " + ("Pause" if pause else "Resume"))

    def control_tick(self):
        now = time.monotonic()
        old_paused = self.active_clock.paused_s
        self.active_clock.update(now, self.paused)
        # The pinned run loop checks this raw deadline between waypoints.
        self.deadline_wall += self.active_clock.paused_s - old_paused
        desired = self.lease.update(read_json(self.folder / "desktop_control.json"), now)
        if desired == "stopped":
            self.control_failure = self.lease.reason or "desktop_control_unavailable"
        if not self.control_failure and desired == "paused" and not self.paused and not self.finishing:
            try:
                self.set_physics_pause(True)
                self.paused = True
                self.active_clock.update(time.monotonic(), True)
                self.report("Paused")
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.control_failure = str(exc)
        elif self.paused and (desired == "running" or self.control_failure or self.finishing):
            try:
                self.set_physics_pause(False)
                self.paused = False
                self.watchdog_grace = time.monotonic() + 8
                if self.mission_active:
                    self.report("Running")
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.control_failure = str(exc)
        if not self.paused and not self.control_failure:
            if now > self.watchdog_grace:
                missing = [name for name in ("/scan_fullframe", "/imu/data", "/gps_fix",
                                             "/local_ekf/odom", "/clock")
                           if now - self.last_sensor.get(name, 0) > 5]
                if missing:
                    self.control_failure = "required_sensor_stale: " + ", ".join(missing)
            if not self.control_failure and self.mission_active:
                self.report("Running")
        if self.control_failure:
            self.referee_failed = True  # Abort the client; never publish a private referee event.
            self.report("Stopped" if self.control_failure == "operator_stop" else "Failed",
                        self.control_failure)
        self._publish_enable()

    def _publish_enable(self):
        # Retain the mission runner's ownership of the Run heartbeat.
        from std_msgs.msg import Bool
        if rclpy.ok():
            permitted = (self.mission_active and not self.referee_failed
                         and not self.paused and not self.control_failure)
            self.enable_pub.publish(Bool(data=permitted))

    def _wait(self, future, timeout_sec):
        deadline_active = time.monotonic() - self.active_clock.paused_s + timeout_sec
        while rclpy.ok() and not future.done() and not self.referee_failed:
            now = time.monotonic()
            if not self.paused and (now - self.active_clock.paused_s >= deadline_active
                                    or now >= self.deadline_wall):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--course-config", required=True)
    parser.add_argument("--timeout-sec", type=float, default=900)
    parser.add_argument("--total-timeout-sec", type=float, default=900)
    parser.add_argument("--require-command-gate", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = DesktopMission(args)
    ok = False
    try:
        node.control_tick()
        if not node.control_failure:
            ok = node.run()
    except (KeyboardInterrupt, Exception) as exc:
        node.control_failure = str(exc) or "mission_interrupted"
        node.report("Failed", node.control_failure)
    finally:
        node.mission_active = False
        node.finishing = True
        # AUTO can abort an action before the 100 ms control timer consumes
        # Stop. Read the final command before classifying that action result.
        node.control_tick()
        node._publish_enable()
        node.cancel_active_goal()
        if not node.control_failure:
            node.report("Finishing" if ok else "Failed", None if ok else "mission_failed")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
