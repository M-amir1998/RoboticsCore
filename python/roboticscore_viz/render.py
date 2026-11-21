"""Reproducible rendering of the documentation animations.

    python -m roboticscore_viz.render astar          # one animation
    python -m roboticscore_viz.render all            # every animation
    python -m roboticscore_viz.render --list

Each animation is defined by (demo, seed, config); the C++ demo is executed, its trace is
rendered to docs/assets/<name>.gif, and docs/assets/manifest.json records how to reproduce it.
"""

from __future__ import annotations

import argparse
import importlib
import json
import subprocess
import sys
from dataclasses import dataclass

from .trace import REPO_ROOT, run_demo

ASSETS = REPO_ROOT / "docs" / "assets"
MANIFEST = ASSETS / "manifest.json"


@dataclass(frozen=True)
class Animation:
    name: str
    demo: str
    seed: int
    renderer: str
    config: str | None = None
    description: str = ""


ANIMATIONS = [
    Animation(
        "hero",
        "hero",
        3,
        "hero",
        "configs/integrated/hero.json",
        "Integrated mobile robot: EKF localisation, LiDAR mapping, Theta*, DWA",
    ),
    Animation(
        "astar",
        "astar",
        3,
        "astar",
        "configs/planning/astar.json",
        "Dijkstra vs A* expansion, Theta* any-angle path",
    ),
    Animation(
        "dstar_lite",
        "dstar_lite",
        3,
        "dstar_lite",
        "configs/planning/dstar_lite.json",
        "D* Lite repairing its plan after a wall appears",
    ),
    Animation(
        "rrt_star",
        "rrt_star",
        3,
        "rrt_star",
        "configs/planning/rrt_star.json",
        "RRT* growth and rewiring, Informed RRT* comparison",
    ),
    Animation(
        "parking",
        "parking",
        1,
        "parking",
        "configs/integrated/parking.json",
        "Hybrid A* parking plan tracked by MPC",
    ),
    Animation(
        "particle_filter",
        "particle_filter",
        3,
        "particle_filter",
        "configs/estimation/particle_filter.json",
        "Particle filter localisation from a poor prior",
    ),
    Animation(
        "ekf_localization",
        "ekf_localization",
        3,
        "ekf_localization",
        "configs/estimation/ekf_localization.json",
        "EKF landmark localisation vs dead reckoning",
    ),
    Animation("icp", "icp", 3, "icp", "configs/estimation/icp.json", "Point-to-point vs point-to-line ICP"),
    Animation(
        "pose_graph",
        "pose_graph",
        3,
        "pose_graph",
        "configs/slam/pose_graph.json",
        "Pose-graph SLAM loop closure",
    ),
    Animation(
        "controllers",
        "controllers",
        1,
        "controllers",
        "configs/control/controllers.json",
        "Pure Pursuit, Stanley, LQR and MPC on one track",
    ),
    Animation(
        "potential_field",
        "potential_field",
        1,
        "potential_field",
        None,
        "Potential field success and local-minimum failure",
    ),
    Animation(
        "rvo",
        "rvo",
        1,
        "rvo",
        "configs/planning/rvo.json",
        "Reciprocal velocity obstacles: eight agents swap places",
    ),
]


def _git_commit() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _load_manifest() -> dict:
    if MANIFEST.exists():
        return json.loads(MANIFEST.read_text(encoding="utf-8"))
    return {"assets": {}}


def render_one(anim: Animation, manifest: dict) -> None:
    config = REPO_ROOT / anim.config if anim.config else None
    trace = run_demo(anim.demo, anim.seed, config)
    module = importlib.import_module(f"roboticscore_viz.renderers.{anim.renderer}")
    output = ASSETS / f"{anim.name}.gif"
    info = module.render(trace, output)
    manifest["assets"][output.name] = {
        "description": anim.description,
        "generator": f"scripts/render/render_{anim.name}.py",
        "demo": anim.demo,
        "seed": anim.seed,
        "config": anim.config,
        "commit": _git_commit(),
        "bytes": info["bytes"],
        "frames": info["frames"],
    }
    print(f"{output.relative_to(REPO_ROOT)}: {info['bytes'] / 1e6:.2f} MB, {info['frames']} frames")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("names", nargs="*", help="animation names, or 'all'")
    parser.add_argument("--list", action="store_true", help="list available animations")
    args = parser.parse_args(argv)
    if args.list or not args.names:
        for a in ANIMATIONS:
            print(f"{a.name:18s} {a.description}")
        return 0
    wanted = [a.name for a in ANIMATIONS] if args.names == ["all"] else args.names
    by_name = {a.name: a for a in ANIMATIONS}
    unknown = [n for n in wanted if n not in by_name]
    if unknown:
        print(f"unknown animation(s): {', '.join(unknown)}", file=sys.stderr)
        return 2
    manifest = _load_manifest()
    for name in wanted:
        render_one(by_name[name], manifest)
    manifest["assets"] = dict(sorted(manifest["assets"].items()))
    ASSETS.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
