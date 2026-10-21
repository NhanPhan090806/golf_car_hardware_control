"""sensor_msgs/Image -> BGR numpy without cv_bridge (pure, testable)."""

import numpy as np

_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}


def image_msg_to_bgr(msg):
    """Return an owned HxWx3 uint8 BGR array; raise ValueError if unsupported."""
    enc = msg.encoding.lower()
    if enc not in _CHANNELS:
        raise ValueError(f'unsupported encoding {msg.encoding!r}; use one of {sorted(_CHANNELS)}')
    h, w, step, ch = int(msg.height), int(msg.width), int(msg.step), _CHANNELS[enc]
    if h <= 0 or w <= 0 or step < w * ch:
        raise ValueError(f'bad image geometry {w}x{h} step {step}')
    data = msg.data
    # rclpy delivers uint8[] as array.array, which exposes a buffer directly.
    buf = np.frombuffer(data if isinstance(data, (bytes, bytearray, memoryview)) or hasattr(data, 'typecode')
                        else bytes(data), np.uint8)
    if buf.size < h * step:
        raise ValueError(f'image data {buf.size} bytes < {h * step}')
    img = buf[:h * step].reshape(h, step)[:, :w * ch].reshape(h, w, ch)
    if enc == 'bgr8':
        out = img.copy()
    elif enc == 'rgb8':
        out = img[:, :, ::-1].copy()
    elif enc == 'bgra8':
        out = img[:, :, :3].copy()
    elif enc == 'rgba8':
        out = img[:, :, 2::-1].copy()
    else:
        out = np.repeat(img, 3, axis=2)
    return out
