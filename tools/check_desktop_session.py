"""Opt-in host test of the real desktop ROS/Gazebo lifecycle.

Run with the GUI environment's Python after ROS workspace setup. By default
this checks Pause/Resume/Stop and a fresh reset. --complete-course also requires
the first attempt to finish every goal and pass its final stopping gate.
"""
import argparse
import json
from pathlib import Path
import time

from autonav_universe.full_track import FullTrackSession, read_json, repository_root


def verify(repo, bundle, timeout_s, complete_course=False):
    session = FullTrackSession(repo, bundle, timeout_s=timeout_s)
    phase, last_state = "starting", None
    running_since, pause_since, resumed_since = None, None, None
    pause_samples = []

    def changed(snapshot):
        nonlocal phase, last_state, running_since, pause_since, resumed_since
        now, state = time.monotonic(), snapshot["state"]
        if state != last_state:
            print(json.dumps({"run": session.run_dir.name, "state": state,
                              "message": snapshot.get("message")}), flush=True)
            last_state = state
        score = (snapshot.get("telemetry") or {}).get("score") or {}
        if phase == "starting" and state == "Running":
            running_since = running_since or now
            if now - running_since >= 5:
                session.request("paused")
                phase = "pausing"
        elif phase == "pausing" and state == "Paused":
            pause_since = pause_since or now
            if score.get("score_time_s") is not None:
                pause_samples.append(score["score_time_s"])
            if now - pause_since >= 4:
                if len(pause_samples) < 2 or max(pause_samples) - min(pause_samples) >= 0.2:
                    session.request("stopped")
                    raise RuntimeError(f"Pause did not freeze simulation time: {pause_samples}")
                print(json.dumps({"pause_clock_samples": pause_samples}), flush=True)
                session.request("running")
                phase = "resuming"
        elif phase == "resuming" and state == "Running":
            resumed_since = now
            phase = "resumed"
        elif phase == "resumed" and not complete_course and now - resumed_since >= 3:
            session.request("stopped")

    session.run(changed)
    result = read_json(session.run_dir / "result.json")
    print(json.dumps({"run_dir": str(session.run_dir), "result": result}), flush=True)
    if phase != "resumed" or not result.get("safety_gate_passed"):
        raise RuntimeError("Desktop lifecycle failed: " + json.dumps(result))
    if complete_course and not result.get("mission_passed"):
        raise RuntimeError("Full course failed: " + json.dumps(result))
    if not complete_course and result.get("reason") != "operator_stop":
        raise RuntimeError("Stop was not recorded: " + json.dumps(result))

    reset = FullTrackSession(repo, bundle, timeout_s=90)

    def reset_changed(snapshot):
        if snapshot["state"] == "Running":
            reset.request("stopped")

    reset.run(reset_changed)
    result = read_json(reset.run_dir / "result.json")
    print(json.dumps({"reset_run_dir": str(reset.run_dir), "result": result}), flush=True)
    if (reset.run_dir == session.run_dir or result.get("reason") != "operator_stop"
            or not result.get("safety_gate_passed")):
        raise RuntimeError("Fresh reset/Stop failed: " + json.dumps(result))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=repository_root())
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--timeout", type=int, default=1800)
    parser.add_argument("--complete-course", action="store_true")
    args = parser.parse_args()
    verify(args.repo, args.bundle or args.repo / "runs/full-track-diagnostic",
           args.timeout, args.complete_course)


if __name__ == "__main__":
    main()
