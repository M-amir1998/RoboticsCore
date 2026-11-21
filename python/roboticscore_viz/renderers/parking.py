"""Hybrid A* search into a parking slot, then MPC tracking with its prediction horizon."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .. import animation, draw, style


def render(trace: dict, output: Path) -> dict:
    env = trace["environment"]
    fp = trace["footprint"]
    planned = np.asarray(trace["planned"])
    expanded = np.asarray(trace["expanded"])
    driven = np.asarray(trace["driven"])
    predictions = {int(k): np.asarray(v) for k, v in trace["predictions"]}
    fig = style.figure(860, 560)
    ax = fig.add_axes([0.02, 0.03, 0.96, 0.88])
    style.clean_axes(ax, env["bounds"], "Car-like planning: Hybrid A* + MPC parking")
    draw.environment(ax, env)
    draw.vehicle(ax, trace["goal"], fp, style.GOOD, filled=False, zorder=3)
    expl = ax.scatter([], [], s=4, color=style.EXPLORED_DARK, alpha=0.6, zorder=3, label="expanded states")
    fwd = planned[planned[:, 3] > 0]
    rev = planned[planned[:, 3] < 0]
    (plan_fwd,) = ax.plot([], [], color=style.PATH, linewidth=2.2, zorder=4, label="plan (forward)")
    (plan_rev,) = ax.plot(
        [], [], color=style.PATH_ALT, linewidth=2.2, linestyle="--", zorder=4, label="plan (reverse)"
    )
    (trail,) = ax.plot([], [], color=style.ESTIMATE, linewidth=1.4, zorder=5, label="driven (MPC)")
    (pred,) = ax.plot(
        [],
        [],
        color=style.ESTIMATE,
        linewidth=0,
        marker="o",
        markersize=2.5,
        alpha=0.8,
        zorder=6,
        label="MPC prediction",
    )
    ax.legend(loc="lower right", ncol=2)
    info = ax.text(
        0.01,
        0.98,
        "",
        transform=ax.transAxes,
        fontsize=9,
        va="top",
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )
    car = []
    search_frames = 24
    drive_idx = [*list(range(0, len(driven), 3)), len(driven) - 1]
    schedule = [("search", i) for i in range(search_frames)] + [("drive", k) for k in drive_idx]
    pred_keys = np.array(sorted(predictions))

    def draw_frame(i: int) -> None:
        kind, k = schedule[i]
        for a in car:
            a.remove()
        car.clear()
        if kind == "search":
            m = int(len(expanded) * (k + 1) / search_frames)
            expl.set_offsets(expanded[:m])
            info.set_text(f"Hybrid A*: {m} expanded (x, y, yaw) states")
            if k == search_frames - 1:
                plan_fwd.set_data(fwd[:, 0], fwd[:, 1])
                plan_rev.set_data(rev[:, 0], rev[:, 1])
            car.extend(draw.vehicle(ax, trace["start"], fp, style.TRUTH))
            return
        trail.set_data(driven[: k + 1, 0], driven[: k + 1, 1])
        nearest = pred_keys[np.searchsorted(pred_keys, k, side="right") - 1]
        p = predictions[int(nearest)]
        pred.set_data(p[:, 0], p[:, 1]) if len(p) else pred.set_data([], [])
        car.extend(draw.vehicle(ax, driven[k][:3], fp, style.TRUTH))
        gear = "reverse" if driven[k][3] < -0.05 else ("forward" if driven[k][3] > 0.05 else "stopped")
        text = f"MPC tracking  |  speed {driven[k][3]:+.2f} m/s ({gear})"
        if k == len(driven) - 1:
            text = (
                f"parked: position error {trace['final_position_error'] * 100:.0f} cm, "
                f"heading error {np.degrees(trace['final_heading_error']):.1f} deg"
            )
        info.set_text(text)

    size = animation.render_animation(
        fig, draw_frame, range(len(schedule)), output, fps=15, colors=64, hold_last_s=3.0
    )
    return {"bytes": size, "frames": len(schedule)}
