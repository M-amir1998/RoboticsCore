"""In-memory GIF encoding with a shared palette and an explicit file-size budget.

Frames are rasterised by matplotlib's Agg canvas straight into memory, quantised to one
palette shared by all frames (no dithering, so lines stay crisp and frames do not flicker),
and written once with Pillow. No intermediate frame files are created.
"""

from __future__ import annotations

import io
from collections.abc import Callable, Iterable
from pathlib import Path

import numpy as np
from PIL import Image

DEFAULT_BUDGET_BYTES = 5 * 1024 * 1024


class SizeBudgetError(RuntimeError):
    """Raised when an encoded animation is larger than its budget."""


def canvas_to_image(fig) -> Image.Image:
    """Render a matplotlib figure into a Pillow RGB image."""
    fig.canvas.draw()
    buffer = np.asarray(fig.canvas.buffer_rgba())
    return Image.fromarray(buffer[:, :, :3].copy())


def shared_palette(frames: list[Image.Image], colors: int) -> Image.Image:
    """Adaptive palette computed from a mosaic of evenly spaced frames."""
    samples = frames[:: max(1, len(frames) // 8)][:8]
    width, height = samples[0].size
    mosaic = Image.new("RGB", (width, height * len(samples)))
    for i, frame in enumerate(samples):
        mosaic.paste(frame, (0, i * height))
    return mosaic.quantize(colors=colors, method=Image.Quantize.MEDIANCUT)


def encode_gif(
    frames: list[Image.Image],
    fps: float,
    colors: int = 96,
    hold_last_s: float = 1.5,
) -> bytes:
    """Encode RGB frames as a looping GIF and return the bytes."""
    if not frames:
        raise ValueError("encode_gif: no frames")
    palette = shared_palette(frames, colors)
    quantised = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    frame_ms = round(1000.0 / fps)
    durations = [frame_ms] * len(quantised)
    durations[-1] = round(hold_last_s * 1000) + frame_ms
    out = io.BytesIO()
    quantised[0].save(
        out,
        format="GIF",
        save_all=True,
        append_images=quantised[1:],
        duration=durations,
        loop=0,
        optimize=True,
        disposal=1,
    )
    return out.getvalue()


def render_animation(
    fig,
    draw_frame: Callable[[int], None],
    frame_indices: Iterable[int],
    output: Path,
    fps: float = 15.0,
    colors: int = 96,
    budget_bytes: int = DEFAULT_BUDGET_BYTES,
    hold_last_s: float = 1.5,
) -> int:
    """Draw each frame with ``draw_frame(i)``, encode and write ``output``.

    Returns the file size in bytes. Raises SizeBudgetError (without writing) if the encoded
    animation exceeds ``budget_bytes``.
    """
    frames = []
    for i in frame_indices:
        draw_frame(i)
        frames.append(canvas_to_image(fig))
    data = encode_gif(frames, fps=fps, colors=colors, hold_last_s=hold_last_s)
    if len(data) > budget_bytes:
        raise SizeBudgetError(
            f"{output.name}: {len(data) / 1e6:.2f} MB exceeds budget {budget_bytes / 1e6:.2f} MB"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(data)
    return len(data)


def evenly_spaced(count: int, frames: int) -> list[int]:
    """``frames`` indices spread over ``range(count)``, always including the last index."""
    if count <= 0:
        return []
    if count <= frames:
        return list(range(count))
    idx = np.linspace(0, count - 1, frames).round().astype(int)
    return sorted(set(int(i) for i in idx))
