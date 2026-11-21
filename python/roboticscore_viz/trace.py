"""Running the C++ demo executable and loading its JSON traces."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_EXE = "roboticscore_demo.exe" if os.name == "nt" else "roboticscore_demo"


def find_demo_executable() -> Path:
    """Locate ``roboticscore_demo``: $ROBOTICSCORE_DEMO, PATH, then the usual build trees."""
    env = os.environ.get("ROBOTICSCORE_DEMO")
    if env:
        return Path(env)
    on_path = shutil.which("roboticscore_demo")
    if on_path:
        return Path(on_path)
    for preset in ("release", "ci-release", "clang-release", "gcc-release", "debug"):
        candidate = REPO_ROOT / "build" / preset / "apps" / _EXE
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "roboticscore_demo not found; build the 'release' preset or set ROBOTICSCORE_DEMO"
    )


def run_demo(name: str, seed: int = 1, config: Path | None = None) -> dict:
    """Run a demo and return its trace. The trace file lives only in a temporary directory."""
    exe = find_demo_executable()
    with tempfile.TemporaryDirectory(prefix="roboticscore_") as tmp:
        out = Path(tmp) / f"{name}.json"
        cmd = [str(exe), name, "--seed", str(seed), "--out", str(out)]
        if config is not None:
            cmd += ["--config", str(config)]
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        return json.loads(out.read_text(encoding="utf-8"))


def load_trace(path: Path) -> dict:
    """Load a trace previously written with ``roboticscore_demo --out``."""
    return json.loads(Path(path).read_text(encoding="utf-8"))
