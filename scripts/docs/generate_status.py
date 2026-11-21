"""Generate the README status matrix from configs/algorithms.yaml.

    python scripts/docs/generate_status.py           # rewrite the block in README.md
    python scripts/docs/generate_status.py --check   # fail if stale or a referenced file is missing

The matrix lives between the markers <!-- status:begin --> and <!-- status:end --> in README.md.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
REGISTRY = REPO_ROOT / "configs" / "algorithms.yaml"
README = REPO_ROOT / "README.md"
BEGIN = "<!-- status:begin -->"
END = "<!-- status:end -->"
STATUSES = {"Tested", "Experimental", "Planned"}
BENCHMARKS = {None, "scenario", "micro"}


def load_registry() -> dict:
    return yaml.safe_load(REGISTRY.read_text(encoding="utf-8"))


def validate(registry: dict) -> list[str]:
    """Return a list of problems: unknown values or referenced files that do not exist."""
    problems = []
    categories = {c["id"] for c in registry["categories"]}
    for algo in registry["algorithms"]:
        name = algo["name"]
        if algo["category"] not in categories:
            problems.append(f"{name}: unknown category {algo['category']!r}")
        if algo["status"] not in STATUSES:
            problems.append(f"{name}: unknown status {algo['status']!r}")
        if algo.get("benchmark") not in BENCHMARKS:
            problems.append(f"{name}: unknown benchmark kind {algo['benchmark']!r}")
        if algo["status"] == "Planned":
            continue
        paths = [algo.get("header"), algo.get("docs"), *algo.get("tests", [])]
        if algo.get("demo"):
            paths.append(f"docs/assets/{algo['demo']}.gif")
        for rel in filter(None, paths):
            if not (REPO_ROOT / rel).exists():
                problems.append(f"{name}: missing file {rel}")
        if algo["status"] == "Tested" and not algo.get("tests"):
            problems.append(f"{name}: status Tested but no tests listed")
    return problems


def render_matrix(registry: dict) -> str:
    lines = [
        "| Area | Algorithm | Status | Tests | Demo | Benchmark |",
        "|---|---|---|---:|---|---|",
    ]
    titles = {c["id"]: c["title"] for c in registry["categories"]}
    previous = None
    for cat in registry["categories"]:
        for algo in (a for a in registry["algorithms"] if a["category"] == cat["id"]):
            area = titles[cat["id"]] if cat["id"] != previous else ""
            previous = cat["id"]
            name = f"[{algo['name']}]({algo['docs']})" if algo.get("docs") else algo["name"]
            tests = str(len(algo.get("tests", []))) if algo.get("tests") else "-"
            demo = f"[{algo['demo']}](docs/assets/{algo['demo']}.gif)" if algo.get("demo") else "-"
            bench = {"scenario": "[scenario](docs/benchmarks/README.md)", "micro": "micro"}.get(
                algo.get("benchmark"), "-"
            )
            lines.append(f"| {area} | {name} | {algo['status']} | {tests} | {demo} | {bench} |")
    lines += [
        "",
        "*Tested*: implemented, unit tested and documented. *Experimental*: implemented, limited "
        "validation. *Planned*: not implemented. Tests = number of test files covering the entry. "
        "Generated from [`configs/algorithms.yaml`](configs/algorithms.yaml) by "
        "`scripts/docs/generate_status.py`.",
    ]
    return "\n".join(lines)


def splice(readme: str, block: str) -> str:
    start = readme.index(BEGIN) + len(BEGIN)
    end = readme.index(END)
    return readme[:start] + "\n" + block + "\n" + readme[end:]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--check", action="store_true", help="verify instead of writing")
    args = parser.parse_args(argv)

    registry = load_registry()
    problems = validate(registry)
    for p in problems:
        print(f"error: {p}", file=sys.stderr)

    readme = README.read_text(encoding="utf-8")
    if BEGIN not in readme or END not in readme:
        print(f"error: README.md lacks the {BEGIN} / {END} markers", file=sys.stderr)
        return 1
    updated = splice(readme, render_matrix(registry))
    if args.check:
        if updated != readme:
            print(
                "error: README status matrix is stale; run scripts/docs/generate_status.py", file=sys.stderr
            )
            return 1
        return 1 if problems else 0
    README.write_text(updated, encoding="utf-8", newline="\n")
    print("updated README.md status matrix")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
