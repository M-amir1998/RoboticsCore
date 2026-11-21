"""ICP iterations on the same scans: point-to-line converges in far fewer iterations than
point-to-point, which slides slowly along the walls."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .. import animation, style


def _apply(pose, pts):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pts @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])


def render(trace: dict, output: Path) -> dict:
    target = np.asarray(trace["target"])
    source = np.asarray(trace["source"])
    methods = [("point_to_point", "Point-to-point ICP"), ("point_to_line", "Point-to-line ICP")]
    fig = style.figure(900, 380)
    axes = [fig.add_axes([0.01, 0.04, 0.485, 0.84]), fig.add_axes([0.505, 0.04, 0.485, 0.84])]
    pad = 1.0
    lo = np.minimum(target.min(0), source.min(0)) - pad
    hi = np.maximum(target.max(0), source.max(0)) + pad
    artists, texts = [], []
    for ax, (_key, title) in zip(axes, methods, strict=False):
        style.clean_axes(ax, (lo[0], lo[1], hi[0], hi[1]), title)
        ax.scatter(target[:, 0], target[:, 1], s=6, color=style.TRUTH, label="target scan", zorder=3)
        artists.append(ax.scatter([], [], s=6, color=style.ESTIMATE, label="source scan", zorder=4))
        texts.append(
            ax.text(
                0.99,
                0.02,
                "",
                transform=ax.transAxes,
                ha="right",
                va="bottom",
                fontsize=9,
                bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
            )
        )
        ax.legend(loc="upper left", markerscale=2)
    histories = [trace[k]["history"] for k, _ in methods]
    frame_count = max(len(h) for h in histories) + 1

    def draw_frame(i: int) -> None:
        for art, text, hist in zip(artists, texts, histories, strict=False):
            k = min(i, len(hist) - 1)
            moved = _apply(hist[k], source)
            art.set_offsets(moved)
            d = np.min(np.linalg.norm(moved[:, None, :] - target[None, :, :], axis=2), axis=1)
            done = " (converged)" if k == len(hist) - 1 else ""
            text.set_text(f"iteration {k:2d}{done}   mean residual {d.mean() * 100:5.1f} cm")

    size = animation.render_animation(
        fig, draw_frame, range(frame_count), output, fps=4, colors=32, hold_last_s=2.5
    )
    return {"bytes": size, "frames": frame_count}
