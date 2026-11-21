"""Flagship animation: the whole mobile-robot stack running in one warehouse scenario.

noisy odometry + landmarks -> EKF localisation | LiDAR -> occupancy mapping |
Theta* global path -> lookahead target -> DWA (with moving obstacles) -> velocity commands
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle, Polygon

from .. import animation, draw, style

_MAP_CMAP = LinearSegmentedColormap.from_list(
    "occupancy", [(0.0, "#ffffff"), (0.5, "#e2e8f0"), (1.0, "#1e293b")]
)


def render(trace: dict, output: Path) -> dict:
    env = trace["environment"]
    grid = trace["grid"]
    frames = trace["frames"]
    maps = trace["maps"]
    landmarks = np.asarray(trace["landmarks"])
    route = np.asarray(trace["global_path"])

    fig = style.figure(1000, 500)
    ax = fig.add_axes([0.01, 0.04, 0.70, 0.86])
    b = env["bounds"]
    style.clean_axes(ax, b, "RoboticsCore: localisation, mapping, planning and control in one loop")
    map_img = draw.grid_image(ax, np.full(grid["width"] * grid["height"], 0.5), grid, _MAP_CMAP)
    for poly in env["polygons"]:  # true obstacles as thin outlines over the estimated map
        ax.add_patch(Polygon(poly, closed=True, fill=False, edgecolor=style.MUTED, linewidth=0.5, zorder=2))
    for x, y, r in env["circles"]:
        ax.add_patch(Circle((x, y), r, fill=False, edgecolor=style.MUTED, linewidth=0.5, zorder=2))
    ax.scatter(
        landmarks[:, 0], landmarks[:, 1], marker="^", s=40, color=style.LANDMARK, edgecolor="white", zorder=5
    )
    ax.plot(route[:, 0], route[:, 1], color=style.PATH, linewidth=1.6, alpha=0.75, zorder=3)
    draw.star(ax, trace["goal"], style.BAD, size=200)
    (truth_trail,) = ax.plot([], [], color=style.TRUTH, linewidth=1.3, zorder=4)
    lidar = ax.scatter([], [], s=2.5, color=style.LIDAR, zorder=4)
    (rollout,) = ax.plot([], [], color=style.GOOD, linewidth=2.0, zorder=6)
    target = ax.scatter([], [], s=30, marker="x", color=style.GOOD, zorder=7)
    transient = []

    # Side panel: pipeline legend and localisation error.
    side = fig.add_axes([0.73, 0.52, 0.26, 0.38])
    side.axis("off")
    rows = [
        (style.LANDMARK, "^", "landmarks + noisy odometry -> EKF"),
        (style.ESTIMATE, "o", "EKF estimate (2-sigma x10)"),
        (style.LIDAR, ".", "LiDAR returns -> occupancy map"),
        (style.PATH, "_", "Theta* global path"),
        (style.GOOD, "_", "DWA trajectory to lookahead"),
        (style.DYNAMIC, "o", "moving obstacles (predicted)"),
        (style.TRUTH, "o", "true robot"),
    ]
    for i, (color, marker, label) in enumerate(rows):
        y = 0.95 - i * 0.14
        side.scatter([0.03], [y], marker=marker, s=50, color=color)
        side.text(0.1, y, label, va="center", fontsize=8.5, color=style.TEXT)
    side.set_xlim(0, 1)
    side.set_ylim(0, 1)
    err_ax = fig.add_axes([0.765, 0.1, 0.22, 0.3])
    err_ax.set_title("localisation error", fontsize=9, loc="left")
    err_ax.set_xlabel("time [s]", fontsize=8)
    err_ax.set_ylabel("[cm]", fontsize=8)
    times = np.asarray([f["t"] for f in frames])
    errors = np.asarray(
        [100 * np.hypot(f["truth"][0] - f["estimate"][0], f["truth"][1] - f["estimate"][1]) for f in frames]
    )
    err_ax.set_xlim(0, times[-1])
    err_ax.set_ylim(0, max(1.0, errors.max() * 1.15))
    (err_line,) = err_ax.plot([], [], color=style.ESTIMATE, linewidth=1.2)
    status = ax.text(
        0.01,
        0.98,
        "",
        transform=ax.transAxes,
        fontsize=9,
        va="top",
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )

    map_frames = np.asarray([m["frame"] for m in maps])
    indices = [*list(range(0, len(frames), 4)), len(frames) - 1]

    def draw_frame(i: int) -> None:
        f = frames[i]
        for a in transient:
            a.remove()
        transient.clear()
        m = maps[int(np.searchsorted(map_frames, i, side="right")) - 1] if map_frames[0] <= i else None
        if m is not None:
            map_img.set_data(np.asarray(m["p"], dtype=float).reshape(grid["height"], grid["width"]) / 255.0)
        t = np.asarray([fr["truth"] for fr in frames[: i + 1]])
        truth_trail.set_data(t[:, 0], t[:, 1])
        hits = np.asarray(f["lidar"]) if f["lidar"] else np.zeros((0, 2))
        lidar.set_offsets(hits)
        r = np.asarray(f["rollout"]) if f["rollout"] else np.zeros((0, 2))
        rollout.set_data(r[:, 0] if len(r) else [], r[:, 1] if len(r) else [])
        target.set_offsets([f["target"]])
        for lid in f["observed"]:
            lm = landmarks[lid]
            transient.extend(
                ax.plot(
                    [f["truth"][0], lm[0]],
                    [f["truth"][1], lm[1]],
                    color=style.LANDMARK,
                    linewidth=0.5,
                    alpha=0.45,
                    zorder=3,
                )
            )
        for x, y, rad in f["dynamic"]:
            transient.append(
                ax.add_patch(
                    Circle((x, y), rad, facecolor=style.DYNAMIC, edgecolor="white", alpha=0.9, zorder=6)
                )
            )
        cov = [100 * c for c in f["covariance"]]  # 2-sigma ellipse scaled x10 in length
        transient.append(draw.covariance_ellipse(ax, f["estimate"], cov, style.ESTIMATE, zorder=7))
        transient.extend(draw.robot(ax, f["truth"], 0.3, style.TRUTH, zorder=8))
        err_line.set_data(times[: i + 1], errors[: i + 1])
        text = f"t = {f['t']:5.1f} s   map + EKF + Theta* + DWA"
        if i == len(frames) - 1:
            text = (
                f"goal reached in {f['t']:.1f} s   |   EKF RMSE {trace['position_rmse'] * 100:.1f} cm"
                f"   |   map accuracy {trace['map_accuracy'] * 100:.0f}%"
            )
        status.set_text(text)

    size = animation.render_animation(fig, draw_frame, indices, output, fps=15, colors=96, hold_last_s=3.0)
    return {"bytes": size, "frames": len(indices)}
