"""Shared visual style: one palette and one set of figure defaults for every figure."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

# Palette (light background, muted obstacles, saturated algorithm output).
BACKGROUND = "#ffffff"
FREE = "#f8fafc"
OBSTACLE = "#334155"
OBSTACLE_EDGE = "#1e293b"
GRID_LINE = "#e2e8f0"
TEXT = "#0f172a"
MUTED = "#64748b"

TRUTH = "#0f172a"  # ground truth
ESTIMATE = "#ea580c"  # estimator output
PATH = "#2563eb"  # planned path
PATH_ALT = "#7c3aed"  # alternative planner / informed
EXPLORED = "#93c5fd"  # search frontier / tree
EXPLORED_DARK = "#60a5fa"
GOOD = "#16a34a"
BAD = "#dc2626"
DYNAMIC = "#e11d48"  # moving obstacles
LANDMARK = "#0d9488"
LIDAR = "#f43f5e"

CONTROLLER_COLORS = {
    "pure_pursuit": "#2563eb",
    "stanley": "#16a34a",
    "lqr": "#ea580c",
    "mpc": "#7c3aed",
}
CONTROLLER_LABELS = {
    "pure_pursuit": "Pure Pursuit",
    "stanley": "Stanley",
    "lqr": "LQR",
    "mpc": "MPC",
}


def apply() -> None:
    """Apply the RoboticsCore matplotlib defaults."""
    plt.rcParams.update(
        {
            "figure.facecolor": BACKGROUND,
            "axes.facecolor": BACKGROUND,
            "axes.edgecolor": GRID_LINE,
            "axes.labelcolor": TEXT,
            "axes.titlesize": 11,
            "axes.titleweight": "semibold",
            "axes.titlecolor": TEXT,
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "legend.fontsize": 8,
            "legend.frameon": True,
            "legend.framealpha": 0.92,
            "legend.edgecolor": GRID_LINE,
            "xtick.color": MUTED,
            "ytick.color": MUTED,
            "savefig.facecolor": BACKGROUND,
            "lines.solid_capstyle": "round",
        }
    )


def figure(width_px: int = 760, height_px: int = 460, dpi: int = 100):
    """Create a figure of the given pixel size with the shared style applied."""
    apply()
    fig = plt.figure(figsize=(width_px / dpi, height_px / dpi), dpi=dpi)
    return fig


def clean_axes(ax, bounds, title: str | None = None) -> None:
    """Equal-aspect axes clipped to ``bounds`` = (xmin, ymin, xmax, ymax) without tick noise."""
    ax.set_xlim(bounds[0], bounds[2])
    ax.set_ylim(bounds[1], bounds[3])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color(GRID_LINE)
    if title:
        ax.set_title(title, loc="left", pad=6)
