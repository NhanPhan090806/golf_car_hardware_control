"""Synthetic road images with known lane geometry, for tests and demos.

Boundaries are drawn as x(y) = x_bottom + slope*(H-1-y) + curve*(H-1-y)^2,
so tests know the exact expected x at any row.
"""

import argparse
import dataclasses
import math
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclasses.dataclass
class Marking:
    x_bottom: float
    slope: float  # dx per pixel going up
    color: Tuple[int, int, int] = (235, 235, 235)  # BGR
    width_bottom: float = 10.0
    width_top: float = 4.0
    dashed: bool = False
    dash_len: float = 0.10  # fraction of H
    gap_len: float = 0.08
    dash_phase: float = 0.0  # fraction of H
    curve: float = 0.0
    double_gap: Optional[float] = None  # px at bottom; draws two stripes
    y_top_ratio: float = 0.55
    y_bottom_ratio: float = 1.0
    gaps: Sequence[Tuple[float, float]] = ()  # extra unpainted y-ratio intervals

    def x_at(self, y, H):
        d = (H - 1) - y
        return self.x_bottom + self.slope * d + self.curve * d * d


WHITE = (235, 235, 235)
YELLOW = (20, 190, 230)


def road(W=640, H=360, seed=0, noise=6.0):
    rng = np.random.default_rng(seed)
    img = np.empty((H, W, 3), np.uint8)
    horizon = int(0.5 * H)
    img[:horizon] = (200, 170, 130)  # sky
    base = np.linspace(70, 95, H - horizon)[:, None, None]
    img[horizon:] = np.clip(base + rng.normal(0, noise, (H - horizon, W, 1)), 0, 255).astype(np.uint8)
    return img


def draw(img, m: Marking):
    H = img.shape[0]
    y0, y1 = int(m.y_top_ratio * (H - 1)), int(m.y_bottom_ratio * (H - 1))
    period = (m.dash_len + m.gap_len) * H
    offsets = [0.0] if m.double_gap is None else [-0.5, 0.5]
    for y in range(y0, y1 + 1):
        t = (y - y0) / max(1, y1 - y0)
        w = m.width_top + (m.width_bottom - m.width_top) * t
        if m.dashed and ((H - 1 - y) / H * H + m.dash_phase * H) % period >= m.dash_len * H:
            continue
        if any(a * (H - 1) <= y <= b * (H - 1) for a, b in m.gaps):
            continue
        xc = m.x_at(y, H)
        for o in offsets:
            sep = 0.0 if m.double_gap is None else (m.double_gap * (0.4 + 0.6 * t) + w)
            x = xc + o * sep
            xa, xb = int(round(x - w / 2)), int(round(x + w / 2))
            xa, xb = max(0, xa), min(img.shape[1] - 1, xb)
            if xb > xa:
                img[y, xa:xb] = m.color
    return img


def scene(markings, W=640, H=360, seed=0):
    img = road(W, H, seed)
    for m in markings:
        draw(img, m)
    return img


def two_lane(W=640, H=360, left_x=170.0, right_x=470.0, left_dashed=False, right_dashed=False,
             converge=0.55, **kw):
    """Left/right boundaries converging toward a vanishing point."""
    vx = 0.5 * (left_x + right_x)
    dy = (H - 1) - 0.5 * (H - 1)
    lm = Marking(left_x, (vx - left_x) * converge / dy, dashed=left_dashed, **kw.pop('left', {}))
    rm = Marking(right_x, (vx - right_x) * converge / dy, dashed=right_dashed, **kw.pop('right', {}))
    return scene([lm, rm], W, H, kw.pop('seed', 0)), lm, rm


def moving_scene(t, W=640, H=360, amplitude=60.0, period_s=6.0, dashed_right=True):
    shift = amplitude * math.sin(2 * math.pi * t / period_s)
    phase = (t * 0.6) % 1.0
    img, _, _ = two_lane(W, H, 170 + shift, 470 + shift, right_dashed=dashed_right,
                         right={'dash_phase': phase}, seed=int(t * 20) % 7)
    return img


def main():
    ap = argparse.ArgumentParser(description='Write a synthetic lane video (no hardware needed).')
    ap.add_argument('output', help='output .avi/.mp4 path')
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--fps', type=float, default=20.0)
    a = ap.parse_args()
    fourcc = cv2.VideoWriter_fourcc(*('mp4v' if a.output.endswith('.mp4') else 'MJPG'))
    out = cv2.VideoWriter(a.output, fourcc, a.fps, (640, 360))
    if not out.isOpened():
        raise SystemExit(f'cannot open {a.output} for writing')
    for i in range(int(a.seconds * a.fps)):
        out.write(moving_scene(i / a.fps))
    out.release()
    print(f'wrote {a.output}')


if __name__ == '__main__':
    main()
