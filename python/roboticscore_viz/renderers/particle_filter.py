"""Particle filter localisation: particles collapse from a poor prior onto the true pose."""

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
        ax,
        (b[0] - 0.5, b[1] - 0.5, b[2] + 0.5, b[3] + 0.5),
        "Particle filter localisation (systematic resampling)",
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
    particles = ax.scatter([], [], s=3, color=style.EXPLORED_DARK, alpha=0.5, zorder=3, label="particles")
    (truth_trail,) = ax.plot([], [], color=style.TRUTH, linewidth=1.4, zorder=4, label="ground truth")
    (est_trail,) = ax.plot(
        [],
        [],
        color=style.ESTIMATE,
        linewidth=1.4,
        linestyle="--",
        zorder=4,
        label="estimate (weighted mean)",
    )
    rays = []
    dynamic = []
    info = ax.text(
        0.01,
        0.01,
        "",
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
    )
    ax.legend(loc="upper right", markerscale=2)
    indices = animation.evenly_spaced(len(frames), 80)

    def draw_frame(i: int) -> None:
        f = frames[i]
        for artist in rays + dynamic:
            artist.remove()
        rays.clear()
        dynamic.clear()
        p = np.asarray(f["particles"])
        particles.set_offsets(p[:, :2])
        t = np.asarray([fr["truth"] for fr in frames[: i + 1]])
        e = np.asarray([fr["estimate"] for fr in frames[: i + 1]])
        truth_trail.set_data(t[:, 0], t[:, 1])
        est_trail.set_data(e[:, 0], e[:, 1])
        for lid in f["observed"]:
            lm = landmarks[lid]
            rays.extend(
                ax.plot(
                    [f["truth"][0], lm[0]],
                    [f["truth"][1], lm[1]],
                    color=style.LANDMARK,
                    linewidth=0.7,
                    alpha=0.5,
                    zorder=2,
                )
            )
        dynamic.extend(draw.robot(ax, f["truth"], 0.35, style.TRUTH))
        dynamic.extend(draw.robot(ax, f["estimate"], 0.35, style.ESTIMATE, filled=False))
        err = np.hypot(f["truth"][0] - f["estimate"][0], f["truth"][1] - f["estimate"][1])
        info.set_text(f"t = {0.1 * (i + 1):4.1f} s   position error {err:5.2f} m   ESS {f['ess']:.0f}")

    size = animation.render_animation(fig, draw_frame, indices, output, fps=12, colors=64, hold_last_s=2.0)
    return {"bytes": size, "frames": len(indices)}
