"""Reciprocal velocity obstacles: agents on a circle swap to antipodal positions."""

from __future__ import annotations

import colorsys
from pathlib import Path

import numpy as np
from matplotlib.patches import Circle

from .. import animation, style


def render(trace: dict, output: Path) -> dict:
    frames = [np.asarray(f) for f in trace["frames"]]
    goals = np.asarray(trace["goals"])
    radius = trace["radius"]
    n = len(goals)
    colors = [colorsys.hsv_to_rgb(i / n, 0.65, 0.85) for i in range(n)]
    fig = style.figure(540, 540)
    ax = fig.add_axes([0.03, 0.03, 0.94, 0.89])
    style.clean_axes(ax, (-6.5, -6.5, 6.5, 6.5), "Reciprocal velocity obstacles (8 agents)")
    for g, c in zip(goals, colors, strict=False):
        ax.scatter([g[0]], [g[1]], marker="x", s=40, color=c, zorder=2)
    trails = [ax.plot([], [], color=c, linewidth=1.0, alpha=0.6)[0] for c in colors]
    discs = [ax.add_patch(Circle((0, 0), radius, facecolor=c, edgecolor="white", zorder=5)) for c in colors]
    info = ax.text(
        0.01,
        0.01,
        "",
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )
    pos = np.stack(frames)
    indices = list(range(0, len(frames), 2))

    def draw_frame(i: int) -> None:
        for a in range(n):
            trails[a].set_data(pos[: i + 1, a, 0], pos[: i + 1, a, 1])
            discs[a].center = pos[i, a]
        diff = pos[i][:, None, :] - pos[i][None, :, :]
        d = np.linalg.norm(diff, axis=2) + np.eye(n) * 1e9
        info.set_text(f"min separation {d.min():.2f} m (contact at {2 * radius:.2f} m)")

    size = animation.render_animation(fig, draw_frame, indices, output, fps=15, colors=48, hold_last_s=2.0)
    return {"bytes": size, "frames": len(indices)}
