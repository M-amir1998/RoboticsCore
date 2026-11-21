"""Tests of the GIF encoder and render registry (no C++ build required)."""

import io

import numpy as np
import pytest
from PIL import Image

from roboticscore_viz import animation, style
from roboticscore_viz.render import ANIMATIONS


def _frames(n=6, size=(64, 48)):
    out = []
    for i in range(n):
        arr = np.full((size[1], size[0], 3), 255, dtype=np.uint8)
        arr[:, : (i + 1) * 8] = (37, 99, 235)
        out.append(Image.fromarray(arr))
    return out


def test_encode_gif_roundtrip_frames_and_loop():
    data = animation.encode_gif(_frames(), fps=10, colors=16)
    gif = Image.open(io.BytesIO(data))
    assert gif.format == "GIF"
    assert gif.n_frames == 6
    assert gif.info.get("loop") == 0


def test_render_animation_enforces_budget(tmp_path):
    fig = style.figure(120, 80)
    ax = fig.add_axes([0, 0, 1, 1])
    (line,) = ax.plot([], [])

    def draw(i):
        line.set_data(np.arange(i + 2), np.random.default_rng(i).random(i + 2))

    out = tmp_path / "x.gif"
    with pytest.raises(animation.SizeBudgetError):
        animation.render_animation(fig, draw, range(5), out, budget_bytes=10)
    assert not out.exists()
    size = animation.render_animation(fig, draw, range(5), out)
    assert out.stat().st_size == size


def test_evenly_spaced_includes_last_index():
    assert animation.evenly_spaced(10, 4)[-1] == 9
    assert animation.evenly_spaced(3, 10) == [0, 1, 2]
    assert animation.evenly_spaced(0, 5) == []


def test_animation_registry_is_consistent():
    names = [a.name for a in ANIMATIONS]
    assert len(names) == len(set(names))
    for a in ANIMATIONS:
        assert a.seed >= 0
        __import__(f"roboticscore_viz.renderers.{a.renderer}")
