"""Pose-graph SLAM: drifted odometry snaps onto the true loop once loop closures are optimised."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.collections import LineCollection

from .. import animation, style


def render(trace: dict, output: Path) -> dict:
    truth = np.asarray(trace["truth"])
    odom = np.asarray(trace["odometry"])
    iterations = [np.asarray(p) for p in trace["iterations"]]
    closures = trace["loop_closures"]
    allpts = np.vstack([truth[:, :2], odom[:, :2]])
    lo, hi = allpts.min(0) - 1.5, allpts.max(0) + 1.5
    fig = style.figure(760, 500)
    ax = fig.add_axes([0.02, 0.03, 0.96, 0.87])
    style.clean_axes(ax, (lo[0], lo[1], hi[0], hi[1]), "Pose-graph SLAM: loop closure with Gauss-Newton")
    ax.plot(truth[:, 0], truth[:, 1], color=style.TRUTH, linewidth=1.2, alpha=0.35, label="ground truth")
    edges = LineCollection([], colors=style.GOOD, linewidths=0.8, alpha=0.7, zorder=3)
    ax.add_collection(edges)
    (est,) = ax.plot(
        [],
        [],
        color=style.ESTIMATE,
        linewidth=2.0,
        marker="o",
        markersize=2.2,
        zorder=4,
        label="pose estimate",
    )
    ax.plot([], [], color=style.GOOD, linewidth=0.8, label="loop-closure constraints")
    ax.legend(loc="upper right")
    info = ax.text(
        0.01,
        0.01,
        "",
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )
    n = len(odom)
    grow = 40
    # Phase 1: the drifted trajectory is driven; phase 2: each optimisation iteration (eased).
    schedule = [("drive", int(k)) for k in np.linspace(2, n, grow)]
    schedule += [("closures", 0)] * 6
    for it in range(1, len(iterations)):
        for t in np.linspace(0.0, 1.0, 8)[1:]:
            schedule.append(("opt", (it, float(t))))

    def poses_for(step):
        kind, value = step
        if kind in ("drive", "closures"):
            return odom
        it, t = value
        return (1 - t) * iterations[it - 1] + t * iterations[it]

    def draw_frame(i: int) -> None:
        step = schedule[i]
        poses = poses_for(step)
        upto = step[1] if step[0] == "drive" else n
        est.set_data(poses[:upto, 0], poses[:upto, 1])
        if step[0] == "drive":
            edges.set_segments([])
            info.set_text(f"driving with odometry only   (nodes {upto})")
        else:
            edges.set_segments([(poses[a, :2], poses[b, :2]) for a, b in closures])
            if step[0] == "closures":
                info.set_text(f"{len(closures)} loop closures detected   ATE {trace['ate_before']:.2f} m")
            else:
                it, t = step[1]
                chi2 = trace["chi2"][it] if t == 1.0 else trace["chi2"][it - 1]
                text = f"Gauss-Newton iteration {it}   chi2 {chi2:,.0f}"
                if it == len(iterations) - 1 and t == 1.0:
                    text += f"   ATE {trace['ate_before']:.2f} m -> {trace['ate_after']:.2f} m"
                info.set_text(text)

    size = animation.render_animation(
        fig, draw_frame, range(len(schedule)), output, fps=12, colors=48, hold_last_s=3.0
    )
    return {"bytes": size, "frames": len(schedule)}
