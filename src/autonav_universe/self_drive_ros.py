"""Lifecycle for the standalone ROS/Gazebo self-drive track."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import secrets
import socket
from pathlib import Path
import threading
import time
from urllib.request import urlopen
import uuid

from .full_track import FullTrackSession, read_json, repository_root, write_json
from .intersection import DEFAULT_TURN, parse_turn_command

TRACK_LAYOUTS = ("loop", "intersection")


def referee_failure(score):
    """Expose the latched referee reason rather than implying motion is enabled."""
    if not score or not score.get("failed"):
        return None
    details = score.get("failure_details") or []
    first = details[0] if details else {}
    reason = (first.get("reason") if isinstance(first, dict) else None) or score.get("first_failure") or "safety monitor failure"
    return "ROS safety monitor: " + str(reason)


def reserve_ros_domain():
    """Reserve a non-default DDS domain across concurrent desktop sessions.

    The local TCP socket is a host-only lock; ROS discovery itself runs in the
    containers. Keep it open until the corresponding container is removed.
    """
    start = secrets.randbelow(101)
    for offset in range(101):
        domain = 1 + (start + offset) % 101
        reservation = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                reservation.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            reservation.bind(('127.0.0.1', 37000 + domain))
            return domain, reservation
        except OSError:
            reservation.close()
    raise RuntimeError('No ROS domain is available for a new self-drive session.')


class SelfDriveRosSession(FullTrackSession):
    """Own one isolated container; run the monitor in a background thread.

    request() controls a separately monitored desktop lease. The ROS command
    gate independently inhibits robot motion when the lease is paused or lost.
    """
    def __init__(self, repo=None, image="autonav-humble-fortress:20260928",
                 volume="autonav-full-track-20260928", lane_change_cost_m=0.25,
                 track_layout="loop", turn_command=DEFAULT_TURN):
        if not isinstance(lane_change_cost_m, (int, float)) or not math.isfinite(lane_change_cost_m) or lane_change_cost_m < 0:
            raise ValueError('Broken-line crossing penalty must be finite and nonnegative')
        if track_layout not in TRACK_LAYOUTS:
            raise ValueError('Track layout must be loop or intersection')
        if parse_turn_command(turn_command) is None:
            raise ValueError('Turn command must be straight or left')
        root = Path(repo or repository_root()).resolve()
        super().__init__(root, root, image=image, volume=volume)
        self.lane_change_cost_m = float(lane_change_cost_m)
        self.track_layout = track_layout
        self.turn_command = parse_turn_command(turn_command)
        self.exits = None
        token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
        self.run_dir = root / "runs" / ("self-drive-ros-" + token)
        self.name = "autonav-self-drive-" + token.lower()

    def set_turn_command(self, value):
        """Standing turn request for junctions; reaches the robot within ~0.25 s."""
        command = parse_turn_command(value)
        if command is None:
            raise ValueError('Turn command must be straight or left')
        with self.lock:
            self.turn_command = command
            if self.run_dir.is_dir() and self.desired != "stopped":
                self._heartbeat()

    def _heartbeat(self) -> None:
        self.heartbeat += 1
        value = {"heartbeat": self.heartbeat, "desired": self.desired,
                 "turn_command": self.turn_command}
        if self.desired == "stopped" and self.control_error:
            value["reason"] = self.control_error
        write_json(self.run_dir / "desktop_control.json", value)

    def _exit_reached(self, pose):
        """Evaluator-side route completion from displayed truth (intersection only)."""
        if self.track_layout != "intersection" or not isinstance(pose, dict):
            return None
        if self.exits is None:
            track = read_json(self.run_dir / "track" / "track.json")
            self.exits = track.get("exits") or None
        try:
            x, y = float(pose["x"]), float(pose["y"])
        except (KeyError, TypeError, ValueError):
            return None
        for name, zone in (self.exits or {}).items():
            if math.hypot(x - zone["x_m"], y - zone["y_m"]) <= zone["radius_m"]:
                return name
        return None

    def run(self, notify):
        heartbeat_thread = None
        domain_reservation = None
        course_complete = False
        final_state, reason = "Failed", "startup_failed"
        try:
            self.run_dir.mkdir(parents=True, exist_ok=False)
            with self.lock:
                self._heartbeat()
            heartbeat_thread = threading.Thread(target=self._keep_alive, daemon=True)
            heartbeat_thread.start()
            notify({"state": "Starting", "message": "Checking Docker and the ROS workspace…",
                    "run_dir": str(self.run_dir)})
            if self.desired == "stopped":
                final_state, reason = "Stopped", "operator_stop"
                notify({"state": final_state, "message": reason, "run_dir": str(self.run_dir)})
                return
            try:
                docker_os = self.docker("info", "--format", "{{.OSType}}")
            except (RuntimeError, OSError) as exc:
                raise RuntimeError("Start Docker Desktop's Linux engine before Run. "
                                   f"Docker reported: {exc}") from exc
            if docker_os != "linux":
                raise RuntimeError("Start Docker Desktop's Linux engine before Run.")
            self.docker("image", "inspect", self.image)
            self.docker("volume", "inspect", self.volume)
            domain, domain_reservation = reserve_ros_domain()
            args = ["run", "-d", "--init", "--name", self.name,
                    "--label", f"autonav.desktop-session={self.name}", "--gpus", "all",
                    "--stop-timeout", "15", "-p", "127.0.0.1::8765",
                    "-v", f"{self.volume}:/ws", "-v", f"{self.repo}:/host",
                    "-e", f"RUN_DIR=/host/runs/{self.run_dir.name}",
                    "-e", f"ROS_DOMAIN_ID={domain}",
                    "-e", f"SELF_DRIVE_LANE_CHANGE_COST_M={self.lane_change_cost_m:g}",
                    "-e", f"SELF_DRIVE_TRACK={self.track_layout}",
                    "-e", f"SELF_DRIVE_TURN_COMMAND={self.turn_command}",
                    "-e", "SPECTATOR_PORT=8765", "-e", "DESKTOP_CONTROL=true",
                    self.image, "bash", "/host/integrations/ros_humble_fortress/run_self_drive_track.sh"]
            write_json(self.run_dir / "desktop_session.json", {
                "container": self.name, "image": self.image, "volume": self.volume,
                "ros_domain_id": domain,
                "lane_change_cost_m": self.lane_change_cost_m,
                "track_layout": self.track_layout, "initial_turn_command": self.turn_command,
                "created_utc": datetime.now(timezone.utc).isoformat(), "docker_arguments": args})
            self.launch_attempted = True
            self.docker(*args, timeout=45)
            self.container_started = True
            detail = json.loads(self.docker("inspect", self.name))[0]
            bindings = (detail.get("NetworkSettings", {}).get("Ports") or {}).get("8765/tcp") or []
            if not bindings or not bindings[0].get("HostPort"):
                logs = self.docker("logs", self.name, timeout=5)
                raise RuntimeError("Viewer port unavailable. " + (logs[-2000:] or "Check container logs."))
            self.url = f"http://127.0.0.1:{bindings[0]['HostPort']}/"
            stop_since = None
            done_text = ("Route complete; stopping the robot." if self.track_layout == "intersection"
                         else "Lap complete; stopping the robot.")
            while True:
                readiness = read_json(self.run_dir / "readiness.json")
                state = ("Failed" if self.control_error else "Completed" if course_complete else
                         "Paused" if self.desired == "paused" else
                         "Running" if readiness.get("ready") else "Starting")
                message = {"Starting": "Waiting for ROS, sensors and local costmap…",
                           "Running": "Nav2 MPPI follows the observed-lane reference path.",
                           "Paused": "Motion inhibited by the command gate.",
                           "Completed": done_text,
                           "Failed": self.control_error or "ROS track stopped."}[state]
                snapshot = {"state": state, "message": message, "readiness": readiness,
                            "run_dir": str(self.run_dir), "url": self.url}
                try:
                    with urlopen(self.url + "state", timeout=0.6) as response:
                        snapshot["telemetry"] = json.load(response)
                        controller = snapshot["telemetry"].get("self_drive_status") or {}
                        score = snapshot["telemetry"].get("score") or {}
                        failure = referee_failure(score)
                        if failure:
                            self.control_error = failure
                            snapshot["state"] = "Failed"
                            snapshot["message"] = self.control_error
                        elif score.get("finish_reached") is True or self._exit_reached(
                                snapshot["telemetry"].get("pose")):
                            course_complete = True
                            snapshot["state"] = "Completed"
                            exit_name = self._exit_reached(snapshot["telemetry"].get("pose"))
                            snapshot["message"] = (f"Route complete via the {exit_name} exit; stopping the robot."
                                                   if exit_name else done_text)
                        elif state == "Running":
                            if controller.get("state"):
                                snapshot["message"] = "MPPI self-drive: " + str(controller["state"])
                            else:
                                snapshot["state"] = "Starting"
                                snapshot["message"] = "Waiting for the MPPI self-drive controller…"
                except (OSError, ValueError):
                    snapshot["connection_lost"] = True
                notify(snapshot)
                if self.control_error or course_complete:
                    self.request("stopped")
                detail = json.loads(self.docker("inspect", self.name, timeout=5))[0]
                if not detail["State"]["Running"]:
                    code = detail["State"].get("ExitCode", 1)
                    if self.control_error:
                        final_state, reason = "Failed", self.control_error
                    elif course_complete:
                        final_state, reason = "Completed", None
                    elif self.desired == "stopped":
                        final_state, reason = "Stopped", "operator_stop"
                    else:
                        final_state, reason = "Failed", f"ROS track session exited (code {code}); see logs."
                    notify({"state": final_state, "message": reason or (
                        "Route complete." if self.track_layout == "intersection" else "Lap complete."),
                        "run_dir": str(self.run_dir)})
                    break
                if self.desired == "stopped":
                    stop_since = stop_since or time.monotonic()
                    if time.monotonic() - stop_since > 15:
                        self.docker("stop", "--time", "10", self.name, timeout=15)
                self.done.wait(0.5)
        except Exception as exc:
            final_state, reason = "Failed", str(exc)
            notify({"state": final_state, "message": reason, "run_dir": str(self.run_dir)})
        finally:
            try:
                self.request("stopped")
            except OSError:
                pass
            if self.launch_attempted and not self.container_started:
                try:
                    detail = json.loads(self.docker("inspect", self.name, timeout=5))[0]
                    labels = detail.get("Config", {}).get("Labels") or {}
                    self.container_started = labels.get("autonav.desktop-session") == self.name
                except Exception:
                    pass
            if self.container_started:
                try:
                    logs = self.docker("logs", self.name, timeout=5)
                    (self.run_dir / "container.log").write_text(logs, encoding="utf-8")
                except Exception:
                    pass
                try:
                    self.docker("rm", "-f", self.name, timeout=20)
                except Exception as exc:
                    final_state, reason = "Failed", f"Container cleanup failed: {exc}"
                    notify({"state": final_state, "message": reason, "run_dir": str(self.run_dir)})
            self.done.set()
            if heartbeat_thread:
                heartbeat_thread.join(timeout=1)
            if domain_reservation:
                domain_reservation.close()
            if self.run_dir.is_dir():
                write_json(self.run_dir / "desktop_host_result.json", {"state": final_state, "reason": reason})
