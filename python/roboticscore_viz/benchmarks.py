"""Turn benchmark JSON into the markdown report in docs/benchmarks/.

    python -m roboticscore_viz.benchmarks results/benchmarks.json results/micro.json

Only measured values are written; nothing is estimated or copied from elsewhere.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
from pathlib import Path

from .trace import REPO_ROOT

OUT_DIR = REPO_ROOT / "docs" / "benchmarks"


def cpu_name() -> str:
    """Marketing name of the CPU without host-identifying details."""
    if sys.platform == "win32":
        try:
            import winreg

            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0")
            return str(winreg.QueryValueEx(key, "ProcessorNameString")[0]).strip()
        except OSError:
            pass
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor() or "unknown"


def machine_metadata(results: dict) -> dict:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        commit = "unknown"
    return {
        "cpu": cpu_name(),
        "logical_cores": os.cpu_count(),
        "os": f"{platform.system()} {platform.release()}",
        "compiler": results.get("compiler", "unknown"),
        "build_type": results.get("build_type", "unknown"),
        "library_version": results.get("library_version", "unknown"),
        "commit": commit,
    }


def _fmt(v, digits=2):
    if isinstance(v, bool):
        return "yes" if v else "**no**"
    if isinstance(v, int):
        return f"{v:,}"
    if isinstance(v, float):
        if v != 0 and abs(v) < 10 ** (-digits):
            return f"{v:.1e}"
        return f"{v:,.{digits}f}"
    return str(v)


def table(rows: list[dict], columns: list[tuple[str, str, int]]) -> str:
    """Markdown table; columns are (key, header, decimals)."""
    head = "| " + " | ".join(h for _, h, _ in columns) + " |"
    align = "|" + "|".join("---" if i == 0 else "---:" for i in range(len(columns))) + "|"
    body = ["| " + " | ".join(_fmt(r.get(k, ""), d) for k, _, d in columns) + " |" for r in rows]
    return "\n".join([head, align, *body])


def build_report(results: dict, micro: dict | None) -> str:
    meta = machine_metadata(results)
    lines = [
        "# Benchmarks",
        "",
        "All numbers below were measured by `roboticscore_bench` and `roboticscore_microbenchmarks` on",
        "the machine described here. Timings are wall-clock medians and depend on the hardware;",
        "path lengths, success rates and errors are deterministic for the given seeds.",
        "",
        "| Machine | |",
        "|---|---|",
        *[f"| {k.replace('_', ' ')} | {v} |" for k, v in meta.items()],
        "",
        "Reproduce with `python scripts/benchmark/run_benchmarks.py` (Release build required).",
        "Fairness: every algorithm in a table uses the same map, start, goal, collision checker,",
        "seeds and machine. Sampling-planner rows aggregate "
        f"{results['sampling_planners'][0]['runs']} seeds (medians over successful runs).",
        "",
        "## Grid planners",
        "",
        "8-connected, 0.25 m cells, obstacles inflated by 0.3 m, no corner cutting.",
        "",
    ]
    for scenario in dict.fromkeys(r["scenario"] for r in results["grid_planners"]):
        rows = [r for r in results["grid_planners"] if r["scenario"] == scenario]
        lines += [
            f"**{scenario.replace('_', ' ')}**",
            "",
            table(
                rows,
                [
                    ("planner", "Planner", 0),
                    ("success", "Success", 0),
                    ("time_ms", "Time [ms]", 3),
                    ("path_length", "Path length [m]", 2),
                    ("expanded", "Expanded nodes", 0),
                    ("min_clearance", "Min clearance [m]", 2),
                ],
            ),
            "",
        ]
    lines += [
        "BFS minimises the number of moves, not length; weighted A* trades optimality (bounded by",
        "w) for fewer expansions; Theta* paths are any-angle and therefore shorter than grid paths.",
        "",
        "## Sampling-based planners",
        "",
        "Disc robot of radius 0.3 m, 3,000 iterations, 1 m steering step, PRM with 600 milestones.",
        "",
    ]
    for scenario in dict.fromkeys(r["scenario"] for r in results["sampling_planners"]):
        rows = [r for r in results["sampling_planners"] if r["scenario"] == scenario]
        lines += [
            f"**{scenario.replace('_', ' ')}**",
            "",
            table(
                rows,
                [
                    ("planner", "Planner", 0),
                    ("success_rate", "Success rate", 2),
                    ("time_ms", "Time [ms]", 2),
                    ("path_length", "Path length [m]", 2),
                    ("nodes", "Tree / roadmap nodes", 0),
                    ("collision_checks", "Collision checks", 0),
                    ("min_clearance", "Min clearance [m]", 2),
                ],
            ),
            "",
        ]
    lines += [
        "RRT and RRT-Connect stop at the first solution; RRT* and Informed RRT* use the whole",
        "iteration budget to improve it. PRM's fixed roadmap can miss the narrow passage.",
        "",
        "## Path-tracking controllers",
        "",
        "Kinematic bicycle (wheelbase 2.7 m, |delta| <= 0.6 rad), 20 Hz control, speed profile",
        "limited to 6 m/s, 1.5 m/s^2 and 2 m/s^2 lateral; vehicle starts at rest on the path.",
        "Errors exclude the first 2 s. Steering effort = integral of delta^2; rate energy =",
        "integral of (d delta/dt)^2.",
        "",
    ]
    for track in dict.fromkeys(r["track"] for r in results["controllers"]):
        rows = [r for r in results["controllers"] if r["track"] == track]
        lines += [
            f"**{track.replace('_', ' ')}**",
            "",
            table(
                rows,
                [
                    ("controller", "Controller", 0),
                    ("finished", "Finished", 0),
                    ("cross_track_rmse", "Cross-track RMSE [m]", 3),
                    ("max_cross_track", "Max cross-track [m]", 3),
                    ("heading_rmse", "Heading RMSE [rad]", 3),
                    ("steering_effort", "Steering effort", 3),
                    ("steering_rate_energy", "Steering-rate energy", 3),
                    ("compute_us_mean", "Compute [us/step]", 1),
                ],
            ),
            "",
        ]
    lines += [
        "## State estimation",
        "",
        "Unicycle robot, range-bearing landmarks, 300 steps at 10 Hz, medians over "
        f"{results['estimators'][0]['runs']} seeds. NEES/NIS are means over steps and runs; a",
        "consistent filter gives NEES close to 3 (state dimension) and NIS close to 2 (measurement",
        "dimension). The gated EKF rejects measurements with NIS above the 99% chi-square quantile.",
        "",
    ]
    for suite in dict.fromkeys(r["suite"] for r in results["estimators"]):
        rows = [r for r in results["estimators"] if r["suite"] == suite]
        lines += [
            f"**{suite.replace('_', ' ')}**",
            "",
            table(
                rows,
                [
                    ("filter", "Filter", 0),
                    ("position_rmse", "Position RMSE [m]", 3),
                    ("heading_rmse", "Heading RMSE [rad]", 4),
                    ("nees_mean", "Mean NEES", 2),
                    ("nis_mean", "Mean NIS", 2),
                    ("diverged_runs", "Diverged runs", 0),
                    ("step_time_ms", "Time [ms/step]", 4),
                ],
            ),
            "",
        ]
    lines += [
        "Without gating, the EKF and UKF are badly inconsistent under 5% outliers; the chi-square",
        "gate restores consistency. The particle filter is robust without gating but costs about",
        "20x more per step with 500 particles.",
        "",
        "## SLAM",
        "",
        "Landmark SLAM: 500 steps around a circle with 8 landmarks. Pose graph: two laps of a block",
        "with drifting odometry and simulated place recognition.",
        "",
        table(
            results["slam"],
            [
                ("method", "Method", 0),
                ("ate", "ATE [m]", 3),
                ("ate_odometry", "ATE odometry only [m]", 3),
                ("landmark_rmse", "Landmark RMSE [m]", 3),
                ("step_time_ms", "Time [ms/step]", 4),
                ("total_time_ms", "Total time [ms]", 2),
            ],
        ),
        "",
    ]
    if micro:
        rows = []
        for b in micro.get("benchmarks", []):
            if b.get("aggregate_name") == "median":
                rows.append(
                    {
                        "name": b["run_name"],
                        "time": b["real_time"],
                        "unit": b["time_unit"],
                        "cpu": b["cpu_time"],
                    }
                )
        lines += [
            "## Microbenchmarks",
            "",
            "Google Benchmark, 5 repetitions, medians.",
            "",
            table(
                rows,
                [
                    ("name", "Benchmark", 0),
                    ("time", "Real time", 1),
                    ("cpu", "CPU time", 1),
                    ("unit", "Unit", 0),
                ],
            ),
            "",
        ]
        big_o = [b for b in micro.get("benchmarks", []) if b.get("aggregate_name") == "BigO"]
        if big_o:
            lines += [
                "Empirical complexity of k-d tree nearest-neighbour queries (fitted by Google Benchmark): "
                + ", ".join(f"{b['run_name']}: {b['big_o']}" for b in big_o),
                "",
            ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print("usage: python -m roboticscore_viz.benchmarks results/benchmarks.json [results/micro.json]")
        return 2
    results = json.loads(Path(argv[0]).read_text(encoding="utf-8"))
    micro = json.loads(Path(argv[1]).read_text(encoding="utf-8")) if len(argv) > 1 else None
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "README.md").write_text(build_report(results, micro) + "\n", encoding="utf-8", newline="\n")
    small = {"machine": machine_metadata(results), **{k: v for k, v in results.items()}}
    (OUT_DIR / "results.json").write_text(json.dumps(small, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {OUT_DIR / 'README.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
