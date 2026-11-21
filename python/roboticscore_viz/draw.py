"""Drawing primitives shared by all renderers."""

from __future__ import annotations

import math

import numpy as np
from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle

from . import style


def environment(ax, env: dict, zorder: int = 2) -> None:
    """Draw workspace bounds, circular and polygonal obstacles."""
    b = env["bounds"]
    ax.add_patch(
        Rectangle(
            (b[0], b[1]),
            b[2] - b[0],
            b[3] - b[1],
            facecolor=style.FREE,
            edgecolor=style.OBSTACLE_EDGE,
            linewidth=1.2,
            zorder=0,
        )
    )
    for x, y, r in env.get("circles", []):
        ax.add_patch(
            Circle(
                (x, y),
                r,
                facecolor=style.OBSTACLE,
                edgecolor=style.OBSTACLE_EDGE,
                linewidth=0.6,
                zorder=zorder,
            )
        )
    for poly in env.get("polygons", []):
        ax.add_patch(
            Polygon(
                poly,
                closed=True,
                facecolor=style.OBSTACLE,
                edgecolor=style.OBSTACLE_EDGE,
                linewidth=0.6,
                zorder=zorder,
            )
        )


def robot(
    ax, pose, radius: float, color: str, zorder: int = 6, alpha: float = 1.0, filled: bool = True
) -> list:
    """Disc robot with a heading tick."""
    x, y, th = pose[0], pose[1], pose[2]
    artists = [
        ax.add_patch(
            Circle(
                (x, y),
                radius,
                facecolor=color if filled else "none",
                edgecolor=color,
                linewidth=1.4,
                alpha=alpha,
                zorder=zorder,
            )
        )
    ]
    hx, hy = x + 1.25 * radius * math.cos(th), y + 1.25 * radius * math.sin(th)
    line_color = "white" if filled else color
    artists += ax.plot([x, hx], [y, hy], color=line_color, linewidth=1.6, alpha=alpha, zorder=zorder + 1)
    return artists


def vehicle(
    ax, pose, footprint: dict, color: str, zorder: int = 6, alpha: float = 1.0, filled: bool = True
) -> list:
    """Rectangular car footprint for a rear-axle pose, with a windscreen marker."""
    x, y, th = pose[0], pose[1], pose[2]
    length, width, rear = footprint["length"], footprint["width"], footprint["rear_overhang"]
    c, s = math.cos(th), math.sin(th)
    local = np.array(
        [[-rear, -width / 2], [length - rear, -width / 2], [length - rear, width / 2], [-rear, width / 2]]
    )
    world = local @ np.array([[c, s], [-s, c]]) + np.array([x, y])
    artists = [
        ax.add_patch(
            Polygon(
                world,
                closed=True,
                facecolor=color if filled else "none",
                edgecolor=color,
                linewidth=1.2,
                alpha=alpha,
                zorder=zorder,
            )
        )
    ]
    front = np.array([[length - rear - 0.9, -width / 2 + 0.25], [length - rear - 0.9, width / 2 - 0.25]])
    fw = front @ np.array([[c, s], [-s, c]]) + np.array([x, y])
    artists += ax.plot(
        fw[:, 0], fw[:, 1], color="white" if filled else color, linewidth=1.5, alpha=alpha, zorder=zorder + 1
    )
    return artists


def covariance_ellipse(ax, mean, cov3, color: str, n_sigma: float = 2.0, zorder: int = 7):
    """2-sigma ellipse of a 2x2 covariance given as (xx, xy, yy)."""
    cov = np.array([[cov3[0], cov3[1]], [cov3[1], cov3[2]]])
    vals, vecs = np.linalg.eigh(cov)
    vals = np.clip(vals, 0.0, None)
    angle = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
    w, h = 2 * n_sigma * math.sqrt(vals[1]), 2 * n_sigma * math.sqrt(vals[0])
    return ax.add_patch(
        Ellipse(
            (mean[0], mean[1]),
            w,
            h,
            angle=angle,
            facecolor=color,
            edgecolor=color,
            alpha=0.18,
            linewidth=1.0,
            zorder=zorder,
        )
    )


def grid_image(ax, values, grid: dict, cmap, vmin=0.0, vmax=1.0, zorder: int = 1, alpha=1.0):
    """Show a row-major (y, x) array on the world extent of ``grid``."""
    arr = np.asarray(values, dtype=float).reshape(grid["height"], grid["width"])
    ox, oy = grid["origin"]
    res = grid["resolution"]
    extent = (ox, ox + grid["width"] * res, oy, oy + grid["height"] * res)
    return ax.imshow(
        arr,
        origin="lower",
        extent=extent,
        cmap=cmap,
        vmin=vmin,
        vmax=vmax,
        interpolation="nearest",
        zorder=zorder,
        alpha=alpha,
    )


def cell_centers(cells, grid: dict) -> np.ndarray:
    """World coordinates of integer cells [[x, y], ...]."""
    if len(cells) == 0:
        return np.zeros((0, 2))
    c = np.asarray(cells, dtype=float)
    res = grid["resolution"]
    return np.asarray(grid["origin"]) + (c + 0.5) * res


def star(ax, p, color: str, size: float = 160, zorder: int = 8):
    """Goal marker."""
    return ax.scatter(
        [p[0]], [p[1]], marker="*", s=size, color=color, edgecolor="white", linewidth=0.8, zorder=zorder
    )
