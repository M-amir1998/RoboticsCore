"""RRT* growth with rewiring and the improving solution; Informed RRT* final tree for contrast."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection

from .. import animation, draw, style


def _segments(nodes, parents):
    pts = np.asarray([n[:2] for n in nodes])
    segs = [(pts[i], pts[p]) for i, p in enumerate(parents) if p >= 0]
    return segs


def render(trace: dict, output: Path) -> dict:
    nodes = trace["nodes"]
    events = trace["events"]
    env = trace["environment"]
    b = env["bounds"]
    informed = trace["informed"]

    fig = style.figure(980, 360)
    ax = fig.add_axes([0.01, 0.04, 0.485, 0.84])
    ax2 = fig.add_axes([0.505, 0.04, 0.485, 0.84])
    style.clean_axes(ax, b, "RRT*: growth, rewiring and solution cost")
    style.clean_axes(ax2, b, "Informed RRT*: same budget, focused samples")
    for a in (ax, ax2):
        draw.environment(a, env)
        a.scatter(*trace["start"], s=45, color=style.GOOD, edgecolor="white", zorder=8)
        draw.star(a, trace["goal"], style.BAD)
    tree = LineCollection([], colors=style.EXPLORED_DARK, linewidths=0.6, zorder=3)
    ax.add_collection(tree)
    (path_line,) = ax.plot([], [], color=style.PATH, linewidth=2.4, zorder=7)
    cost_text = ax.text(
        0.99,
        0.02,
        "",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )
    inf_nodes = informed["nodes"]
    ax2.add_collection(
        LineCollection(
            _segments(inf_nodes, [n[2] for n in inf_nodes]), colors="#c4b5fd", linewidths=0.6, zorder=3
        )
    )
    ip = np.asarray(informed["path"])
    ax2.plot(ip[:, 0], ip[:, 1], color=style.PATH_ALT, linewidth=2.4, zorder=7)
    ax2.text(
        0.99,
        0.02,
        f"cost {informed['cost_history'][-1][1]:.2f} m",
        transform=ax2.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )

    # Replay parent pointers through the event log (0 add, 1 rewire, 2 solution improved).
    last_iteration = events[-1][1] if events else 1
    frame_count = 80
    checkpoints = np.linspace(0, last_iteration, frame_count).astype(int)
    parents = [-1] * len(nodes)
    present = [False] * len(nodes)
    present[0] = True
    best_cost = None
    cursor = 0
    path = np.asarray(trace["path"])

    def draw_frame(i: int) -> None:
        nonlocal cursor, best_cost
        limit = checkpoints[i]
        while cursor < len(events) and events[cursor][1] <= limit:
            kind, _, node, value = events[cursor]
            if kind in (0, 1):
                parents[node] = value
                present[node] = True
            else:
                best_cost = value
            cursor += 1
        segs = [(nodes[k][:2], nodes[p][:2]) for k, p in enumerate(parents) if present[k] and p >= 0]
        tree.set_segments(segs)
        if i == frame_count - 1:
            path_line.set_data(path[:, 0], path[:, 1])
        text = f"iteration {limit:,}  |  {sum(present):,} nodes"
        if best_cost is not None:
            text += f"  |  best {best_cost:.2f} m"
        cost_text.set_text(text)

    size = animation.render_animation(
        fig, draw_frame, range(frame_count), output, fps=12, colors=48, hold_last_s=2.5
    )
    return {"bytes": size, "frames": frame_count}
