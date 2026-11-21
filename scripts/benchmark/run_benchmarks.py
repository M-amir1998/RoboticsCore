"""Run the scenario benchmarks and microbenchmarks (Release build) and regenerate
docs/benchmarks/README.md and docs/benchmarks/results.json.

    python scripts/benchmark/run_benchmarks.py [--build-dir build/release] [--runs 20]
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from roboticscore_viz import benchmarks  # noqa: E402

EXE = ".exe" if os.name == "nt" else ""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", default="build/release")
    parser.add_argument("--runs", type=int, default=20)
    args = parser.parse_args()
    build = ROOT / args.build_dir
    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    subprocess.run(
        [
            str(build / "apps" / f"roboticscore_bench{EXE}"),
            "--out",
            str(out / "benchmarks.json"),
            "--runs",
            str(args.runs),
        ],
        check=True,
    )
    subprocess.run(
        [
            str(build / "benchmarks" / f"roboticscore_microbenchmarks{EXE}"),
            "--benchmark_repetitions=5",
            "--benchmark_report_aggregates_only=true",
            f"--benchmark_out={out / 'micro.json'}",
            "--benchmark_out_format=json",
        ],
        check=True,
    )
    return benchmarks.main([str(out / "benchmarks.json"), str(out / "micro.json")])


if __name__ == "__main__":
    raise SystemExit(main())
