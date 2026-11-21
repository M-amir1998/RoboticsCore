"""D* Lite: the robot follows its plan, a wall appears, only affected states are repaired."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.colors import ListedColormap

from .. import animation, draw, style

# 0 free, 1 obstacle, 2 initial search, 3 new obstacle, 4 repaired states
_CMAP = ListedColormap([style.FREE, style.OBSTACLE, "#dbeafe", style.DYNAMIC, "#fcd34d"])


def render(trace: dict, output: Path) -> dict:
    grid = trace["grid"]
    w, h, res = grid["width"], grid["height"], grid["resolution"]
    base = np.asarray(trace["occupancy"], dtype=float).reshape(h, w)
    for x, y in trace["initial_expansions"]:
        if base[y, x] == 0:
            base[y, x] = 2
    steps = trace["steps"]
    insert = next(i for i, s in enumerate(steps) if s["new_obstacles"])
    replan = steps[insert]

    fig = style.figure(760, 520)
    ax = fig.add_axes([0.02, 0.03, 0.96, 0.88])
    bounds = (grid["origin"][0], grid["origin"][1], grid["origin"][0] + w * res, grid["origin"][1] + h * res)
    style.clean_axes(ax, bounds, "D* Lite: incremental replanning")
    image = draw.grid_image(ax, base, grid, _CMAP, vmin=0, vmax=4)
    (path_line,) = ax.plot([], [], color=style.PATH, linewidth=2.6, zorder=6)
    (trail,) = ax.plot([], [], color=style.TRUTH, linewidth=1.2, alpha=0.6, zorder=6)
    robot = ax.scatter([], [], s=70, color=style.TRUTH, edgecolor="white", zorder=8)
    draw.star(ax, draw.cell_centers([trace["goal"]], grid)[0], style.BAD)
    note = ax.text(
        0.01,
        0.98,
        "",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.35"},
    )

    # Show the replan step for longer so the repaired region can be seen.
    order = list(range(insert)) + [insert] * 12 + list(range(insert + 1, len(steps)))

    def draw_frame(i: int) -> None:
        k = order[i]
        img = base.copy()
        if k >= insert:
            for x, y in replan["replanned"]:
                if img[y, x] != 1:
                    img[y, x] = 4
            for x, y in replan["new_obstacles"]:
                img[y, x] = 3
        image.set_data(img)
        path = draw.cell_centers(steps[k]["path"], grid)
        path_line.set_data(path[:, 0], path[:, 1])
        visited = draw.cell_centers([s["robot"] for s in steps[: k + 1]], grid)
        trail.set_data(visited[:, 0], visited[:, 1])
        robot.set_offsets(visited[-1:])
        if k < insert:
            note.set_text(f"initial search: {len(trace['initial_expansions'])} expansions")
        else:
            note.set_text(
                f"wall detected -> repaired {replan['replan_expansions']} states "
                f"(initial search: {len(trace['initial_expansions'])})"
            )

    size = animation.render_animation(
        fig, draw_frame, range(len(order)), output, fps=8, colors=40, hold_last_s=2.0
    )
    return {"bytes": size, "frames": len(order)}
