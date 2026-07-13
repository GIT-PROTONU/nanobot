"""Offline unit tests for gpu_vision's pure helpers (no GL context / camera needed):

    pixi run python -m pytest src/web_control/test

update_novelty (the CPU EMA-background novelty score) and write_oled_mask_blob (the
OLED mask mirror's /dev/shm blob format), plus the re-binarize translate table.
Importing gpu_vision is safe off-robot -- the EGL/GLES libraries are only dlopen'd
when a GLContext is actually constructed.
"""
import json

from web_control.gpu_vision import (update_novelty, write_oled_mask_blob,
                                    NOVELTY_EMA_ALPHA, _OLED_THRESH_TABLE)


def _rgba(cells):
    """Build a flat RGBA byte list from [(r,g,b), ...] cells (alpha ignored)."""
    buf = []
    for r, g, b in cells:
        buf += [r, g, b, 255]
    return buf


def test_novelty_zero_on_identical_frame():
    cells = [(10, 20, 30), (200, 100, 50)]
    pixels = _rgba(cells)
    bg = [float(v) for cell in cells for v in cell]
    assert update_novelty(bg, pixels, len(cells)) == 0.0


def test_novelty_scores_change_and_habituates():
    n = 4
    dark = _rgba([(0, 0, 0)] * n)
    bright = _rgba([(255, 255, 255)] * n)
    bg = [0.0] * (n * 3)
    first = update_novelty(bg, bright, n)
    assert first == 1.0                              # maximal change vs. the background
    # Repeated exposure eases the background toward the new scene -> novelty decays
    # monotonically, and after long enough the change has fully habituated.
    prev = first
    for _ in range(3000):                            # ~3.3 min at 15 fps
        cur = update_novelty(bg, bright, n)
        assert cur <= prev
        prev = cur
    assert prev < 0.01                               # the new scene is now "normal"
    # And the ORIGINAL scene now reads as novel instead.
    assert update_novelty(bg, dark, n) > 0.95


def test_novelty_background_ema_rate():
    bg = [0.0, 0.0, 0.0]
    update_novelty(bg, _rgba([(255, 255, 255)]), 1)
    assert all(abs(v - 255 * NOVELTY_EMA_ALPHA) < 1e-9 for v in bg)


def test_thresh_table_binarizes():
    raw = bytes([0, 60, 127, 128, 200, 255]).translate(_OLED_THRESH_TABLE)
    assert list(raw) == [0, 0, 0, 255, 255, 255]


def test_oled_mask_blob_roundtrip(tmp_path):
    path = str(tmp_path / "nano_oled_mask.bin")
    raw = bytes([0, 255] * 32)
    write_oled_mask_blob(raw, 8, 8, seq=7, conf=0.125, path=path)
    with open(path, "rb") as f:
        header = json.loads(f.readline().decode())
        payload = f.read()
    assert header["w"] == 8 and header["h"] == 8 and header["seq"] == 7
    assert header["conf"] == 0.125 and "t" in header
    assert payload == raw
    assert not (tmp_path / "nano_oled_mask.bin.tmp").exists()   # atomic replace cleaned up
