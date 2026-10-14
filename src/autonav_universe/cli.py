"""Entry point for plug-in validation, episodes, paired suites and replay."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
from dataclasses import asdict, fields
from datetime import datetime, timezone
from pathlib import Path

from . import __version__
from .api import API_VERSION, Observation
from .runner import EpisodeResult, EpisodeRunner
from .dynamics import RobotProfile
from .scenarios import Scenario, load_scenario
from .worker import AlgorithmWorker

DEFAULT_ALGORITHM = "autonav_universe.algorithms:ReferenceNavigator"
PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def _git_info() -> dict:
    def git(*args):
        result = subprocess.run(["git", *args], cwd=PACKAGE_ROOT, capture_output=True,
                                text=True, check=False)
        return result.stdout.strip() if result.returncode == 0 else None
    package_dir = Path(__file__).resolve().parent
    files = sorted(package_dir.glob("*.py"))
    files += sorted((package_dir / "data" / "scenarios").glob("*.json"))
    digest = hashlib.sha256()
    for file in files:
        digest.update(str(file.relative_to(package_dir)).encode())
        digest.update(file.read_bytes())
    return {"commit": git("rev-parse", "HEAD"), "status": git("status", "--porcelain"),
            "source_sha256": digest.hexdigest()}


def _manifest(scenarios: list[Scenario], algorithm: str, seeds: list[int],
              parameters: tuple[tuple[str, str], ...] = (),
              timing_mode: str = "functional", action_budget_s: float = 0.05) -> dict:
    module_name = algorithm.partition(":")[0]
    plugin_sha256 = None
    try:
        if os.getcwd() not in sys.path:
            sys.path.insert(0, os.getcwd())
        spec = importlib.util.find_spec(module_name)
        if spec and spec.origin and Path(spec.origin).is_file():
            plugin_sha256 = hashlib.sha256(Path(spec.origin).read_bytes()).hexdigest()
    except (ImportError, OSError, ValueError):
        pass
    assets = {}
    scenario_dir = Path(__file__).resolve().parent / "data" / "scenarios"
    for path in sorted(scenario_dir.glob("*.json")):
        assets[f"scenarios/{path.name}"] = hashlib.sha256(path.read_bytes()).hexdigest()
    course = PACKAGE_ROOT / "Mock_Course_Asset" / "Course.blend"
    if course.is_file():
        assets[str(course.relative_to(PACKAGE_ROOT))] = hashlib.sha256(course.read_bytes()).hexdigest()
    return {"schema_version": 1, "simulator_version": __version__, "api_version": API_VERSION,
            "algorithm": algorithm, "algorithm_sha256": plugin_sha256,
            "parameters": dict(parameters),
            "seeds": seeds, "timing_mode": timing_mode,
            "action_budget_s": action_budget_s if timing_mode == "deadline" else None,
            "scenarios": [asdict(s) for s in scenarios], "robot_profile": asdict(RobotProfile()),
            "asset_sha256": assets, "source": _git_info(),
            "runtime": {"python": sys.version, "platform": platform.platform(),
                        "processor": platform.processor(),
                        "installed_packages": sorted(
                            f"{dist.metadata.get('Name', 'unknown')}=={dist.version}"
                            for dist in importlib.metadata.distributions())},
            "created_utc": datetime.now(timezone.utc).isoformat()}


def _write_episode(root: Path, runner: EpisodeRunner, result) -> None:
    folder = root / result.episode_id
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "metrics.json").write_text(json.dumps(asdict(result), indent=2), encoding="utf-8")
    (folder / "events.json").write_text(json.dumps(runner.events, indent=2), encoding="utf-8")
    (folder / "truth_replay.json").write_text(json.dumps(runner.truth), encoding="utf-8")
    if runner.capture_observations:
        (folder / "observations.json").write_text(json.dumps(runner.observations), encoding="utf-8")


def _run_one(scenario: Scenario, algorithm: str, seed: int, output: Path | None,
             rich: bool = False, parameters: tuple[tuple[str, str], ...] = (),
             timing_mode: str = "functional", action_budget_s: float = 0.05):
    runner = EpisodeRunner(scenario, algorithm, seed, capture_observations=rich,
                           parameters=parameters, timing_mode=timing_mode,
                           action_budget_s=action_budget_s)
    result = runner.run()
    if output is not None:
        manifest_path = output / "manifest.json"
        if runner.plugin_manifest is not None and manifest_path.is_file():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["plugin_manifest"] = asdict(runner.plugin_manifest)
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        _write_episode(output, runner, result)
    print(json.dumps(asdict(result)))
    return result


def _parse_seeds(value: str) -> list[int]:
    return [int(part) for part in value.split(",") if part]


def _parse_params(values: list[str]) -> tuple[tuple[str, str], ...]:
    result = []
    seen = set()
    for value in values:
        key, separator, val = value.partition("=")
        if not separator or not key or key in seen:
            raise ValueError("parameters must be unique KEY=VALUE entries")
        result.append((key, val))
        seen.add(key)
    return tuple(result)


def _suite(path_or_name: str) -> tuple[list[Scenario], list[int], list[dict]]:
    path = Path(path_or_name)
    if not path.exists():
        path = PACKAGE_ROOT / "benchmarks" / (path_or_name + ".json")
    if not path.exists():
        path = Path(__file__).resolve().parent / "data" / "benchmarks" / (path_or_name + ".json")
    raw = json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema_version") != 1:
        raise ValueError("unsupported suite schema")
    scenarios, invalid = [], []
    for name in raw["scenarios"]:
        try:
            scenarios.append(load_scenario(name))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            invalid.append({"scenario": name, "error": str(exc)})
    return scenarios, list(map(int, raw["seeds"])), invalid


def _write_summary(output: Path, results: list, invalid: list[dict]) -> None:
    rows = [asdict(r) for r in results]
    with (output / "episodes.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[field.name for field in fields(EpisodeResult)])
        writer.writeheader()
        writer.writerows(rows)
    summary = {"episodes": len(rows), "successes": sum(r["outcome"] == "success" for r in rows),
               "invalid_scenarios": len(invalid),
               "outcomes": {name: sum(r["outcome"] == name for r in rows)
                            for name in sorted({r["outcome"] for r in rows})}}
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (output / "invalid_scenarios.json").write_text(json.dumps(invalid, indent=2), encoding="utf-8")
    lines = ["# AutoNav Universe benchmark", "", f"Episodes: {len(rows)}",
             f"Successes: {summary['successes']}", f"Invalid scenarios: {len(invalid)}",
             "", "## Outcomes", ""]
    lines += [f"- {name}: {count}" for name, count in summary["outcomes"].items()]
    lines += ["", "## Episodes", "", "| Scenario | Seed | Outcome | Time (s) | Clearance (m) | Ramp traversals |",
              "|---|---:|---|---:|---:|---:|"]
    for row in rows:
        clearance = "n/a" if row["min_clearance_m"] is None else f"{row['min_clearance_m']:.3f}"
        lines.append(f"| {row['scenario']} | {row['seed']} | {row['outcome']} | "
                     f"{row['time_s']:.2f} | {clearance} | {row['ramp_traversals']} |")
    if invalid:
        lines += ["", "## Invalid scenarios", ""]
        lines += [f"- `{item['scenario']}`: {item['error']}" for item in invalid]
    (output / "report.md").write_text("\n".join(lines)+"\n", encoding="utf-8")


def _compare(left: Path, right: Path, output: Path | None = None) -> int:
    left_manifest = json.loads((left / "manifest.json").read_text(encoding="utf-8"))
    right_manifest = json.loads((right / "manifest.json").read_text(encoding="utf-8"))
    if left_manifest.get("invalid_scenarios") or right_manifest.get("invalid_scenarios"):
        raise ValueError("invalid scenarios must be resolved before comparison")
    for field in ("api_version", "timing_mode", "action_budget_s", "scenarios", "seeds", "robot_profile",
                  "asset_sha256"):
        if left_manifest.get(field) != right_manifest.get(field):
            raise ValueError(f"incompatible comparison: {field} differs")
    if left_manifest.get("source", {}).get("source_sha256") != right_manifest.get("source", {}).get("source_sha256"):
        raise ValueError("incompatible comparison: simulator source differs")
    def read(path):
        with (path / "episodes.csv").open(encoding="utf-8", newline="") as stream:
            return {(r["scenario"], r["seed"]): r for r in csv.DictReader(stream)}
    a, b = read(left), read(right)
    keys = sorted(a.keys() & b.keys())
    if not keys:
        raise ValueError("no paired scenario/seed episodes")
    print("scenario,seed,baseline,candidate,time_delta_s")
    for scenario, seed in keys:
        base, candidate = a[(scenario, seed)], b[(scenario, seed)]
        delta = float(candidate["time_s"])-float(base["time_s"])
        print(f"{scenario},{seed},{base['outcome']},{candidate['outcome']},{delta:.3f}")
    print(f"paired={len(keys)} baseline_success={sum(a[k]['outcome']=='success' for k in keys)} "
          f"candidate_success={sum(b[k]['outcome']=='success' for k in keys)}")
    if output is not None:
        lines = ["# AutoNav Universe paired comparison", "",
                 f"Baseline: `{left_manifest['algorithm']}`", f"Candidate: `{right_manifest['algorithm']}`",
                 f"Paired episodes: {len(keys)}", "",
                 "| Scenario | Seed | Baseline | Candidate | Time difference (s) |",
                 "|---|---:|---|---|---:|"]
        for scenario, seed in keys:
            base, candidate = a[(scenario, seed)], b[(scenario, seed)]
            delta = float(candidate["time_s"])-float(base["time_s"])
            lines.append(f"| {scenario} | {seed} | {base['outcome']} | {candidate['outcome']} | {delta:+.3f} |")
        lines += ["", f"Baseline successes: {sum(a[k]['outcome']=='success' for k in keys)}",
                  f"Candidate successes: {sum(b[k]['outcome']=='success' for k in keys)}"]
        output.write_text("\n".join(lines)+"\n", encoding="utf-8")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="autonav-universe")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("app", help="open interactive Qt application")
    validate = commands.add_parser("validate-plugin")
    validate.add_argument("--algorithm", required=True)
    validate.add_argument("--scenario", default="straight")
    validate.add_argument("--param", action="append", default=[])
    run = commands.add_parser("run")
    run.add_argument("--algorithm", default=DEFAULT_ALGORITHM)
    run.add_argument("--scenario", default="straight")
    run.add_argument("--seed", type=int, default=1)
    run.add_argument("--output", type=Path)
    run.add_argument("--rich", action="store_true", help="save full public sensor observations")
    run.add_argument("--param", action="append", default=[])
    run.add_argument("--timing-mode", choices=("functional", "deadline"), default="functional")
    run.add_argument("--budget-ms", type=float, default=50.0)
    run.add_argument("--gui", action="store_true", help="open course replay window after run")
    benchmark = commands.add_parser("benchmark")
    benchmark.add_argument("--algorithm", default=DEFAULT_ALGORITHM)
    benchmark.add_argument("--suite", default="navigation-v1")
    benchmark.add_argument("--seeds", type=_parse_seeds)
    benchmark.add_argument("--output", type=Path, required=True)
    benchmark.add_argument("--param", action="append", default=[])
    benchmark.add_argument("--timing-mode", choices=("functional", "deadline"), default="functional")
    benchmark.add_argument("--budget-ms", type=float, default=50.0)
    compare = commands.add_parser("compare")
    compare.add_argument("baseline", type=Path)
    compare.add_argument("candidate", type=Path)
    compare.add_argument("--output", type=Path)
    replay = commands.add_parser("replay")
    replay.add_argument("episode", type=Path)
    replay.add_argument("--gui", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.command == "app":
            from .ui import launch_app
            launch_app()
            return 0
        if args.command == "validate-plugin":
            scenario = load_scenario(args.scenario)
            runner = EpisodeRunner(scenario, args.algorithm, parameters=_parse_params(args.param))
            worker = AlgorithmWorker(args.algorithm, runner._context())
            try:
                response = worker.step(Observation(runner.episode_id, 0, 0.0, 0.0, 0, ()))
                if response.expires_s <= 0:
                    raise ValueError("plug-in returned an expired command")
            finally:
                worker.close()
            print(f"valid: {args.algorithm} for {scenario.track}")
            return 0
        if args.command == "run":
            scenario = load_scenario(args.scenario)
            parameters = _parse_params(args.param)
            budget_s = args.budget_ms/1000
            output = args.output
            if args.gui and output is None:
                output = PACKAGE_ROOT / "runs" / f"{scenario.name}-{args.seed}"
            if output is not None:
                output.mkdir(parents=True, exist_ok=True)
                (output / "manifest.json").write_text(json.dumps(_manifest([scenario], args.algorithm,
                                                                        [args.seed], parameters,
                                                                        args.timing_mode, budget_s), indent=2), encoding="utf-8")
            result = _run_one(scenario, args.algorithm, args.seed, output, args.rich, parameters,
                              args.timing_mode, budget_s)
            if args.gui:
                from .ui import show_replay
                show_replay(output / result.episode_id, scenario)
            return 0 if result.outcome == "success" else 1
        if args.command == "benchmark":
            scenarios, default_seeds, invalid = _suite(args.suite)
            seeds = args.seeds or default_seeds
            parameters = _parse_params(args.param)
            budget_s = args.budget_ms/1000
            args.output.mkdir(parents=True, exist_ok=True)
            manifest = _manifest(scenarios, args.algorithm, seeds, parameters,
                                 args.timing_mode, budget_s)
            manifest["invalid_scenarios"] = invalid
            (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            results = [_run_one(scenario, args.algorithm, seed, args.output,
                                parameters=parameters, timing_mode=args.timing_mode,
                                action_budget_s=budget_s)
                       for scenario in scenarios for seed in seeds]
            _write_summary(args.output, results, invalid)
            return 0 if not invalid and results and all(r.outcome == "success" for r in results) else 1
        if args.command == "compare":
            return _compare(args.baseline, args.candidate, args.output)
        if args.command == "replay":
            metrics = json.loads((args.episode / "metrics.json").read_text(encoding="utf-8"))
            print(json.dumps(metrics, indent=2))
            if args.gui:
                from .ui import show_replay
                show_replay(args.episode, load_scenario(metrics["scenario"]))
            return 0
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f"error: {exc}\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
