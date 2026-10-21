import types

import numpy as np
import pytest

from lane_following.ros_image import image_msg_to_bgr


def msg(enc, arr, pad=0):
    h, w = arr.shape[:2]
    ch = 1 if arr.ndim == 2 else arr.shape[2]
    rows = np.zeros((h, w * ch + pad), np.uint8)
    rows[:, :w * ch] = arr.reshape(h, w * ch)
    return types.SimpleNamespace(encoding=enc, height=h, width=w, step=w * ch + pad, data=rows.tobytes())


def test_rgb_to_bgr_with_row_padding():
    rgb = np.zeros((4, 5, 3), np.uint8)
    rgb[..., 0] = 200  # red
    out = image_msg_to_bgr(msg('rgb8', rgb, pad=3))
    assert out.shape == (4, 5, 3) and (out[..., 2] == 200).all() and (out[..., 0] == 0).all()


def test_mono_and_bgra():
    assert image_msg_to_bgr(msg('mono8', np.full((3, 3), 7, np.uint8))).shape == (3, 3, 3)
    bgra = np.dstack([np.full((2, 2), v, np.uint8) for v in (1, 2, 3, 4)])
    assert image_msg_to_bgr(msg('bgra8', bgra))[0, 0].tolist() == [1, 2, 3]


def test_rejects_unsupported_and_truncated():
    with pytest.raises(ValueError):
        image_msg_to_bgr(msg('16UC1', np.zeros((2, 2), np.uint8)))
    m = msg('bgr8', np.zeros((4, 4, 3), np.uint8))
    m.data = m.data[:10]
    with pytest.raises(ValueError):
        image_msg_to_bgr(m)
