import numpy as np

from white_balance import balance


def test_purple_table_becomes_neutral():
    image = np.full((50, 50, 3), (200, 180, 210), np.uint8)
    image[20:25, 20:25] = (220, 30, 30)  # a red cube keeps its hue
    out, gains = balance(image)
    table = out[0, 0].astype(int)
    assert max(table) - min(table) <= 2
    assert out[22, 22, 0] > 150 and out[22, 22, 1] < 60


def test_uniform_image_has_finite_gains():
    out, gains = balance(np.full((40, 40, 3), (120, 130, 140), np.uint8))
    assert np.all(np.isfinite(gains))
    assert out.dtype == np.uint8


def test_tiny_image_is_left_unchanged():
    image = np.full((5, 5, 3), (10, 200, 30), np.uint8)
    out, gains = balance(image)
    assert np.array_equal(out, image)
    assert np.all(gains == 1)
