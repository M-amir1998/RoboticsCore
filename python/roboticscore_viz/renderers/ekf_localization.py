"""EKF landmark localisation vs odometry dead reckoning, with the 2-sigma position ellipse."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .. import animation, draw, style


def render(trace: dict, output: Path) -> dict:
    frames = trace["frames"]
    landmarks = np.asarray(trace["landmarks"])
    b = trace["bounds"]
    fig = style.figure(620, 560)
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.9])
    style.clean_axes(
        ax, (b[0] - 0.5, b[1] - 0.5, b[2] + 0.5, b[3] + 0.5), "EKF localisation with range-bearing landmarks"
    )
    ax.scatter(
        landmarks[:, 0],
        landmarks[:, 1],
        marker="^",
        s=70,
        color=style.LANDMARK,
        edgecolor="white",
        zorder=5,
        label="landmarks",
    )
    (truth,) = ax.plot([], [], color=style.TRUTH, linewidth=1.6, label="ground truth", zorder=4)
    (dead,) = ax.plot([], [], color=style.BAD, linewidth=1.2, linestyle=":", label="odometry only", zorder=4)
    (est,) = ax.plot(
        [], [], color=style.ESTIMATE, linewidth=1.4, linestyle="--", label="EKF estimate", zorder=4
    )
    ax.legend(loc="upper right")
    transient = []
    info = ax.text(
        0.01,
        0.01,
        "",
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )
    indices = animation.evenly_spaced(len(frames), 90)

    def draw_frame(i: int) -> None:
        for a in transient:
            a.remove()
        transient.clear()
        f = frames[i]
        seg = frames[: i + 1]
        for line, key in ((truth, "truth"), (dead, "dead_reckoning"), (est, "estimate")):
            arr = np.asarray([s[key] for s in seg])
            line.set_data(arr[:, 0], arr[:, 1])
        for lid in f["observed"]:
            lm = landmarks[lid]
            transient.extend(
                ax.plot(
                    [f["truth"][0], lm[0]],
                    [f["truth"][1], lm[1]],
                    color=style.LANDMARK,
                    linewidth=0.7,
                    alpha=0.5,
                    zorder=2,
                )
            )
        # Exaggerate the ellipse x5 so it is visible at this scale (stated in the label).
        cov = [5 * 5 * c for c in f["covariance"]]
        transient.append(draw.covariance_ellipse(ax, f["estimate"], cov, style.ESTIMATE))
        transient.extend(draw.robot(ax, f["truth"], 0.35, style.TRUTH))
        e_ekf = np.hypot(f["truth"][0] - f["estimate"][0], f["truth"][1] - f["estimate"][1])
        e_odo = np.hypot(f["truth"][0] - f["dead_reckoning"][0], f["truth"][1] - f["dead_reckoning"][1])
        info.set_text(f"error  EKF {e_ekf:4.2f} m   odometry {e_odo:4.2f} m   (ellipse: 2-sigma x5)")

    size = animation.render_animation(fig, draw_frame, indices, output, fps=12, colors=64, hold_last_s=2.0)
    return {"bytes": size, "frames": len(indices)}
