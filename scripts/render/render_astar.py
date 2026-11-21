"""Render docs/assets/astar.gif from the C++ demo trace (see roboticscore_viz.render)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from roboticscore_viz.render import main

if __name__ == "__main__":
    raise SystemExit(main(["astar"]))
