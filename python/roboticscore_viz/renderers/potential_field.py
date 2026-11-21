"""Artificial potential field: success in open space, trapped in a U-shaped local minimum."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .. import animation, draw, style


def render(trace: dict, output: Path) -> dict:
    cases = trace["cases"]
    titles = ["Potential field: reaches the goal", "Potential field: trapped in a local minimum"]
    fig = style.figure(900, 430)
    axes = [fig.add_axes([0.01, 0.04, 0.485, 0.84]), fig.add_axes([0.505, 0.04, 0.485, 0.84])]
    markers, trails, notes = [], [], []
    xs = np.arange(0, 20.5, 0.5)
    for ax, case, title in zip(axes, cases, titles, strict=False):
        style.clean_axes(ax, case["environment"]["bounds"], title)
        field = np.log1p(np.asarray(case["field"]))
        ax.contourf(xs, xs, field, levels=18, cmap="Blues", alpha=0.55, zorder=0.5)
        draw.environment(ax, case["environment"])
        ax.patches[0].set_alpha(0.0)  # keep the field visible under the workspace rectangle
        draw.star(ax, case["goal"], style.BAD)
        (trail,) = ax.plot([], [], color=style.ESTIMATE, linewidth=2.0, zorder=6)
        trails.append(trail)
        markers.append(ax.scatter([], [], s=50, color=style.TRUTH, edgecolor="white", zorder=8))
        notes.append(
            ax.text(
                0.01,
                0.01,
                "",
                transform=ax.transAxes,
                fontsize=9,
                va="bottom",
                bbox={"facecolor": "white", "edgecolor": style.GRID_LINE, "boxstyle": "round,pad=0.3"},
            )
        )
    paths = [np.asarray(c["path"]) for c in cases]
    frame_count = 60

    def draw_frame(i: int) -> None:
        for path, trail, marker, note, case in zip(paths, trails, markers, notes, cases, strict=False):
            k = round((len(path) - 1) * i / (frame_count - 1))
            trail.set_data(path[: k + 1, 0], path[: k + 1, 1])
            marker.set_offsets(path[k : k + 1])
            if i == frame_count - 1:
                status = case["status"].replace("_", " ")
                note.set_text(f"status: {status}")

    size = animation.render_animation(
        fig, draw_frame, range(frame_count), output, fps=15, colors=64, hold_last_s=2.5
    )
    return {"bytes": size, "frames": frame_count}
