"""Desktop lifecycle for the real ROS/Fortress course, without Qt imports."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from urllib.request import urlopen
import uuid
import xml.etree.ElementTree as ET


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2), encoding="utf-8")
    # Docker Desktop can briefly hold a Windows bind-mounted file without
    # delete sharing. Retain atomic reads while retrying that transient lock.
    for attempt in range(20):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(0.02)


@dataclass
class ControlLease:
    """Fail closed on a missing desktop heartbeat; Stop cannot be resumed."""
    timeout_s: float = 2.0
    heartbeat: int = -1
    last_seen_s: float | None = None
    desired: str = "stopped"
    reason: str | None = None

    def update(self, value: dict, now_s: float) -> str:
        if self.reason:
            return "stopped"
        if self.last_seen_s is None:
            self.last_seen_s = now_s
        sequence, desired = value.get("heartbeat"), value.get("desired")
        if (type(sequence) is int and sequence > self.heartbeat
                and desired in ("running", "paused", "stopped")):
            self.heartbeat = sequence
            self.last_seen_s = now_s
            self.desired = desired
            if desired == "stopped":
                reason = value.get("reason")
                self.reason = reason[:512] if isinstance(reason, str) and reason else "operator_stop"
        if now_s - self.last_seen_s >= self.timeout_s:
            self.reason = "desktop_heartbeat_lost"
        return "stopped" if self.reason else self.desired


@dataclass
class MissionClock:
    """Exclude acknowledged pauses from mission wall-time budgets."""
    last_s: float
    paused: bool = False
    paused_s: float = 0.0

    def update(self, now_s: float, paused: bool) -> float:
        if self.paused:
            self.paused_s += max(0.0, now_s - self.last_s)
        self.last_s, self.paused = now_s, paused
        return now_s - self.paused_s


def load_bundle(repo: Path, bundle: Path) -> tuple[dict, dict]:
    """Use the same source/world pairing contract as the diagnostic launcher."""
    import yaml  # GUI extra; the headless engine still has no dependencies.
    manifest = read_json(bundle / "course_manifest.json")
    if not manifest:
        raise ValueError("Course bundle missing. Export the course using Mock_Course_Asset/README.md.")
    for path, key in ((repo / "Mock_Course_Asset/Course.blend", "source_asset_sha256"),
                      (bundle / "course.yaml", "course_yaml_sha256"),
                      (bundle / "world.sdf", "world_sdf_sha256")):
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest.get(key):
            raise ValueError(f"Stale or mismatched course bundle: {path.name}")
    ET.parse(bundle / "world.sdf")
    if not manifest.get("geometry_check", {}).get("passed"):
        raise ValueError("Course geometry validation has not passed.")
    course = yaml.safe_load((bundle / "course.yaml").read_text(encoding="utf-8"))
    if not isinstance(course, dict) or course.get("course_id") != manifest.get("course_id"):
        raise ValueError("Course ID differs from the bundle manifest.")
    return course, manifest


def repository_root() -> Path:
    for candidate in (Path.cwd(), *Path.cwd().parents, Path(__file__).resolve().parents[2]):
        if (candidate / "integrations/ros_humble_fortress/run_full_track.sh").is_file():
            return candidate
    return Path.cwd()


class FullTrackSession:
    """Own one isolated container and fresh output directory per Run.

    Run this object's monitor on a worker thread. Heartbeats have their own
    thread so HTTP/Docker timeouts cannot silently keep an old command alive.
    """
    def __init__(self, repo: Path, bundle: Path, timeout_s: int = 1800,
                 image: str = "autonav-humble-fortress:20260928",
                 volume: str = "autonav-full-track-20260928",
                 costmap_version: str = "2", lane_change_cost_m: float = .25):
        if costmap_version not in ("1", "2"):
            raise ValueError("Cost map version must be 1 or 2")
        if not math.isfinite(lane_change_cost_m) or lane_change_cost_m < 0:
            raise ValueError("Lane-change cost must be finite and nonnegative")
        self.costmap_version, self.lane_change_cost_m = costmap_version, lane_change_cost_m
        self.repo, self.bundle = repo.resolve(), bundle.resolve()
        self.timeout_s, self.image, self.volume = timeout_s, image, volume
        token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
        self.run_dir = self.repo / "runs" / ("full-track-app-" + token)
        self.name = "autonav-app-" + token.lower()
        self.desired = "running"
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.heartbeat = 0
        self.container_started = False
        self.launch_attempted = False
        self.url = None
        self.control_error = None

    def docker(self, *args: str, timeout: float = 15) -> str:
        process = subprocess.run(["docker", *args], capture_output=True, text=True,
                                 timeout=timeout, creationflags=(
                                     subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0))
        if process.returncode:
            raise RuntimeError(process.stderr.strip() or process.stdout.strip() or "Docker command failed")
        # `docker logs` sends the container's stderr to the CLI's stderr even
        # when the command succeeds. Startup errors often appear only there.
        if args and args[0] == "logs":
            return "\n".join(part.strip() for part in (process.stdout, process.stderr) if part.strip())
        return process.stdout.strip()

    def request(self, desired: str) -> None:
        if desired not in ("running", "paused", "stopped"):
            raise ValueError("Unknown session command")
        with self.lock:
            if self.desired == "stopped":
                return
            self.desired = desired
            if self.run_dir.is_dir():
                self._heartbeat()

    def _heartbeat(self) -> None:
        self.heartbeat += 1
        value = {"heartbeat": self.heartbeat, "desired": self.desired}
        if self.desired == "stopped" and self.control_error:
            value["reason"] = self.control_error
        write_json(self.run_dir / "desktop_control.json", value)

    def _keep_alive(self) -> None:
        while not self.done.wait(0.25):
            try:
                with self.lock:
                    self._heartbeat()
            except OSError as exc:
                # The container's independent lease expires if this disk stops working.
                if self.desired != "stopped":
                    self.control_error = f"desktop_heartbeat_write_failed: {exc}"
                return

    def run(self, notify) -> None:
        heartbeat_thread = None
        final_state, reason = "Failed", "startup_failed"
        try:
            load_bundle(self.repo, self.bundle)
            relative_bundle = self.bundle.relative_to(self.repo).as_posix()
            if self.timeout_s < 1:
                raise ValueError("Mission timeout must be positive")
            self.run_dir.mkdir(parents=True, exist_ok=False)
            with self.lock:
                self._heartbeat()
            heartbeat_thread = threading.Thread(target=self._keep_alive, daemon=True)
            heartbeat_thread.start()
            notify({"state": "Starting", "message": "Checking Docker and the pinned workspace…"})
            if self.docker("info", "--format", "{{.OSType}}") != "linux":
                raise RuntimeError("Start Docker Desktop's Linux engine before Run.")
            self.docker("image", "inspect", self.image)
            self.docker("volume", "inspect", self.volume)
            args = ["run", "-d", "--init", "--name", self.name,
                    "--label", f"autonav.desktop-session={self.name}", "--gpus", "all",
                    "--stop-timeout", "15", "-p", "127.0.0.1::8765",
                    "-v", f"{self.volume}:/ws", "-v", f"{self.repo}:/host",
                    "-e", f"RUN_DIR=/host/runs/{self.run_dir.name}",
                    "-e", f"COURSE_BUNDLE=/host/{relative_bundle}",
                    "-e", f"MISSION_TIMEOUT_SEC={self.timeout_s}",
                    "-e", f"COSTMAP_VERSION={self.costmap_version}",
                    "-e", f"LANE_CHANGE_COST_M={self.lane_change_cost_m}",
                    "-e", "SPECTATOR_PORT=8765", "-e", "DESKTOP_CONTROL=true",
                    self.image, "bash", "/host/integrations/ros_humble_fortress/run_full_track.sh"]
            write_json(self.run_dir / "desktop_session.json", {
                "container": self.name, "image": self.image, "volume": self.volume,
                "course_bundle": str(self.bundle), "timeout_active_wall_s": self.timeout_s,
                "costmap_version": self.costmap_version, "lane_change_cost_m": self.lane_change_cost_m,
                "created_utc": datetime.now(timezone.utc).isoformat(), "docker_arguments": args})
            # The launcher's fresh-directory check expects this directory not to
            # exist; desktop control files are deliberately the only exception.
            self.launch_attempted = True
            self.docker(*args, timeout=45)
            self.container_started = True
            detail = json.loads(self.docker("inspect", self.name))[0]
            bindings = (detail.get("NetworkSettings", {}).get("Ports") or {}).get("8765/tcp") or []
            if not bindings or not bindings[0].get("HostPort"):
                state = detail.get("State") or {}
                if not state.get("Running", False):
                    try:
                        logs = self.docker("logs", self.name, timeout=5)
                    except Exception:
                        logs = ""
                    exit_code = state.get("ExitCode", "unknown")
                    cause = logs[-2000:] or state.get("Error") or "No container logs available."
                    raise RuntimeError(f"Simulation container exited during startup (exit {exit_code}): {cause}")
                raise RuntimeError("Docker did not publish the simulation viewer port 8765/tcp.")
            port = bindings[0]["HostPort"]
            self.url = f"http://127.0.0.1:{port}/"
            stop_since = None
            while True:
                status = read_json(self.run_dir / "desktop_status.json")
                readiness = read_json(self.run_dir / "readiness.json")
                state = status.get("state", "Starting")
                descriptions = {"Starting": "Waiting for ROS readiness…", "Ready": "ROS readiness passed.",
                                "Running": "Autonomous course mission in progress.", "Paused": "Gazebo physics paused; motion inhibited.",
                                "Finishing": "Mission ended; verifying the stop gate and saving results…"}
                snapshot = {"state": state,
                            "message": status.get("reason") or descriptions.get(state, state),
                            "readiness": readiness, "run_dir": str(self.run_dir), "url": self.url}
                try:
                    with urlopen(self.url + "state", timeout=0.6) as response:
                        snapshot["telemetry"] = json.load(response)
                except (OSError, ValueError):
                    snapshot["connection_lost"] = True
                notify(snapshot)
                if self.control_error:
                    self.request("stopped")
                detail = json.loads(self.docker("inspect", self.name, timeout=5))[0]
                if not detail["State"]["Running"]:
                    result = read_json(self.run_dir / "result.json")
                    desktop = read_json(self.run_dir / "desktop_status.json")
                    if self.control_error:
                        final_state, reason = "Failed", self.control_error
                    elif desktop.get("reason") == "operator_stop" or self.desired == "stopped":
                        final_state, reason = "Stopped", "operator_stop"
                    elif result.get("mission_passed"):
                        final_state, reason = "Completed", None
                    else:
                        reason = result.get("reason") or desktop.get("reason") or "container_exited_before_result"
                    notify({"state": final_state, "message": reason or "Course completed; stop gate verified.",
                            "result": result, "run_dir": str(self.run_dir)})
                    break
                if self.desired == "stopped":
                    stop_since = stop_since or time.monotonic()
                    if time.monotonic() - stop_since > 15:
                        self.docker("stop", "--time", "10", self.name, timeout=15)
                self.done.wait(0.5)
        except Exception as exc:
            reason = str(exc)
            notify({"state": "Failed", "message": reason, "run_dir": str(self.run_dir)})
        finally:
            try:
                self.request("stopped")
            except OSError:
                pass  # Stop the container even when its bind mount becomes unwritable.
            if self.launch_attempted and not self.container_started:
                # The daemon may create a container before the Docker client
                # times out. Verify our unique ownership label before cleanup.
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
                    notify({"state": final_state, "message": reason})
            self.done.set()
            if heartbeat_thread:
                heartbeat_thread.join(timeout=1)
            if self.run_dir.is_dir():
                write_json(self.run_dir / "desktop_host_result.json",
                           {"state": final_state, "reason": reason})
