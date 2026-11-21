"""Four path trackers driving the same track side by side, with live cross-track errors."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .. import animation, draw, style

ORDER = ["pure_pursuit", "stanley", "lqr", "mpc"]
CAR = {"length": 4.5, "width": 1.9, "rear_overhang": 0.9}


def render(trace: dict, output: Path) -> dict:
    path = np.asarray(trace["path"])
    runs = {k: trace["runs"][k] for k in ORDER}
    lo, hi = path.min(0) - 4.0, path.max(0) + 4.0
    fig = style.figure(980, 520)
    ax = fig.add_axes([0.01, 0.34, 0.98, 0.58])
    style.clean_axes(
        ax, (lo[0], lo[1], hi[0], hi[1]), f"Path tracking on the '{trace['track'].replace('_', ' ')}' track"
    )
    ax.plot(path[:, 0], path[:, 1], color=style.MUTED, linewidth=6, alpha=0.18, zorder=1)
    ax.plot(
        path[:, 0], path[:, 1], color=style.MUTED, linewidth=0.8, linestyle="--", zorder=2, label="reference"
    )
    err_ax = fig.add_axes([0.07, 0.1, 0.91, 0.2])
    err_ax.set_ylabel("cross-track [m]")
    err_ax.set_xlabel("time [s]")
    err_ax.axhline(0.0, color=style.GRID_LINE, linewidth=1)
    longest = max(len(r["states"]) for r in runs.values())
    dt = 0.1  # trace stores every second 0.05 s control step
    err_ax.set_xlim(0, longest * dt)
    span = max(max(abs(e) for e in r["cross_track"][20:]) for r in runs.values())
    err_ax.set_ylim(-1.15 * span, 1.15 * span)
    trails, err_lines, cars = {}, {}, {}
    for key in ORDER:
        color = style.CONTROLLER_COLORS[key]
        (trails[key],) = ax.plot(
            [], [], color=color, linewidth=1.6, zorder=4, label=style.CONTROLLER_LABELS[key]
        )
        (err_lines[key],) = err_ax.plot([], [], color=color, linewidth=1.2)
        cars[key] = []
    ax.legend(loc="upper right", ncol=5)
    frames = [*list(range(0, longest, 3)), longest - 1]

    def draw_frame(i: int) -> None:
        for key, run in runs.items():
            states = np.asarray(run["states"])
            k = min(i, len(states) - 1)
            trails[key].set_data(states[: k + 1, 0], states[: k + 1, 1])
            cte = run["cross_track"][: k + 1]
            err_lines[key].set_data(np.arange(len(cte)) * dt, cte)
            for artist in cars[key]:
                artist.remove()
            cars[key] = draw.vehicle(ax, states[k][:3], CAR, style.CONTROLLER_COLORS[key], alpha=0.85)

    size = animation.render_animation(fig, draw_frame, frames, output, fps=15, colors=64, hold_last_s=2.5)
    return {"bytes": size, "frames": len(frames)}
