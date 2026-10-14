#!/usr/bin/env python3
"""Report a course mission result separately from the stop-gate probe."""

import argparse
import json
import re
from pathlib import Path


def read_json_line(path: Path) -> dict | None:
    if not path.exists():
        return None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                return value
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--score", type=Path, required=True)
    parser.add_argument("--gate", type=Path, required=True)
    parser.add_argument("--mission-log", type=Path, required=True)
    parser.add_argument("--mission-exit", type=int, required=True)
    parser.add_argument("--readiness", type=Path)
    args = parser.parse_args()

    score = read_json_line(args.score)
    gate = read_json_line(args.gate)
    readiness = None
    if args.readiness and args.readiness.exists():
        try:
            readiness = json.loads(args.readiness.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            readiness = {"ready": False, "missing": ["invalid readiness report"]}
    gate_passed = bool(gate and gate.get(
        "safety_gate_passed", gate.get("passed")))
    mission_log = (args.mission_log.read_text(encoding="utf-8", errors="replace")
                   if args.mission_log.exists() else "")
    mission_passed = bool(
        args.mission_exit == 0
        and score
        and not score.get("failed")
        and score.get("all_waypoints_reached")
        and score.get("finish_reached")
        and gate_passed
        and (readiness is None or readiness.get("ready"))
    )
    if readiness and not readiness.get("ready"):
        reason = "startup_readiness_failed"
    elif score and score.get("failures"):
        reason = str(score["failures"][0])
    elif not gate_passed:
        reason = "command_gate_probe_failed"
    elif args.mission_exit != 0:
        timeout = re.findall(r"\[([^]]+)\] (?:result|goal acceptance) timed out",
                             mission_log)
        if timeout:
            reason = f"waypoint_timeout:{timeout[-1]}"
        elif "total mission wall timeout" in mission_log:
            reason = "mission_wall_timeout"
        else:
            reason = f"mission_runner_exit_{args.mission_exit}"
    elif not score:
        reason = "referee_score_unavailable"
    elif not score.get("all_waypoints_reached"):
        reason = "waypoints_incomplete"
    elif not score.get("finish_reached"):
        reason = "finish_not_reached"
    else:
        reason = None
    result = {
        "mission_passed": mission_passed,
        "reason": reason,
        "mission_runner_exit_code": args.mission_exit,
        "safety_gate_passed": gate_passed,
        "waypoints_reached": score.get("waypoints_reached_count") if score else None,
        "waypoints_total": score.get("waypoints_total") if score else None,
        "finish_reached": score.get("finish_reached") if score else None,
        "first_failure": score.get("first_failure") if score else None,
        "course_id": score.get("course_id") if score else None,
        "startup_ready": readiness.get("ready") if readiness else None,
        "startup_missing": readiness.get("missing", []) if readiness else [],
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if mission_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
