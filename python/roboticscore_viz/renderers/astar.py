"""Dijkstra vs A* expanding through the same cluttered grid; Theta* path for comparison."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.colors import ListedColormap

from .. import animation, draw, style

# 0 free, 1 obstacle, 2 expanded
_CMAP = ListedColormap([style.FREE, style.OBSTACLE, style.EXPLORED])


def render(trace: dict, output: Path) -> dict:
    grid = trace["grid"]
    w, h = grid["width"], grid["height"]
    occ = np.asarray(trace["occupancy"], dtype=float).reshape(h, w)
    runs = {"dijkstra": trace["dijkstra"], "astar": trace["astar"]}
    titles = {"dijkstra": "Dijkstra", "astar": "A* (octile heuristic)"}
    total = max(len(r["expansion_order"]) for r in runs.values())

    fig = style.figure(980, 380)
    axes = {
        "dijkstra": fig.add_axes([0.01, 0.06, 0.485, 0.84]),
        "astar": fig.add_axes([0.505, 0.06, 0.485, 0.84]),
    }
    images, counters, paths = {}, {}, {}
    extent_bounds = (
        grid["origin"][0],
        grid["origin"][1],
        grid["origin"][0] + w * grid["resolution"],
        grid["origin"][1] + h * grid["resolution"],
    )
    start = draw.cell_centers([trace["start"]], grid)[0]
    goal = draw.cell_centers([trace["goal"]], grid)[0]
    for key, ax in axes.items():
        style.clean_axes(ax, extent_bounds, titles[key])
        images[key] = draw.grid_image(ax, occ, grid, _CMAP, vmin=0, vmax=2)
        ax.scatter([start[0]], [start[1]], s=45, color=style.GOOD, zorder=8, edgecolor="white")
        draw.star(ax, goal, style.BAD)
        counters[key] = ax.text(
            0.99,
            0.02,
            "",
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            fontsize=9,
            color=style.TEXT,
            bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
        )
        (paths[key],) = ax.plot([], [], color=style.PATH, linewidth=2.4, zorder=7)
    theta_line = None
    fig.text(
        0.99,
        0.955,
        "same map, start and goal  |  0.25 m cells, 8-connected",
        fontsize=9,
        color=style.MUTED,
        ha="right",
    )

    frame_count = 70

    def draw_frame(i: int) -> None:
        nonlocal theta_line
        n = round(total * min(1.0, i / (frame_count - 1)))
        for key, run in runs.items():
            order = run["expansion_order"]
            k = min(n, len(order))
            img = occ.copy()
            if k:
                cells = np.asarray(order[:k])
                img[cells[:, 1], cells[:, 0]] = 2
            images[key].set_data(img)
            done = k >= len(order)
            counters[key].set_text(f"expanded {k:,}" + (f"  |  path {run['cost']:.1f} m" if done else ""))
            if done:
                pts = draw.cell_centers(run["path"], grid)
                paths[key].set_data(pts[:, 0], pts[:, 1])
        if i == frame_count - 1 and theta_line is None:
            pts = draw.cell_centers(trace["theta_star"]["path"], grid)
            (theta_line,) = axes["astar"].plot(
                pts[:, 0],
                pts[:, 1],
                color=style.PATH_ALT,
                linewidth=1.8,
                linestyle="--",
                zorder=7,
                label=f"Theta* {trace['theta_star']['cost']:.1f} m",
            )
            axes["astar"].legend(loc="upper left")

    size = animation.render_animation(
        fig, draw_frame, range(frame_count), output, fps=14, colors=48, hold_last_s=2.5
    )
    return {"bytes": size, "frames": frame_count}
