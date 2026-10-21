"""Shared annotated overlay used by every viewer (OpenCV window, browser, CLI)."""

import cv2
import numpy as np

from .detector import HELD, INTERPOLATED, MISSING, OBSERVED
from .pipeline import DEGRADED, INVALID, VALID

RED = (0, 0, 255)
GREEN = (0, 220, 0)
AMBER = (0, 170, 255)
CYAN = (255, 255, 0)
GREY = (150, 150, 150)
WHITE = (255, 255, 255)
STATE_COLOR = {VALID: GREEN, DEGRADED: AMBER, INVALID: RED, 'STALE': RED}
SIDE_COLOR = {OBSERVED: (255, 120, 0), INTERPOLATED: (255, 0, 200), HELD: AMBER, MISSING: GREY}


def _polyline(img, stripe, y0, y1, color, thickness, dashed=False):
    ys = np.arange(int(y0), int(y1) + 1, 2, dtype=np.float64)
    if len(ys) < 2:
        return
    pts = np.stack([stripe.x_at(ys), ys], axis=1).round().astype(np.int32)
    if not dashed:
        cv2.polylines(img, [pts], False, color, thickness, cv2.LINE_AA)
        return
    for i in range(0, len(pts) - 1, 6):
        cv2.polylines(img, [pts[i:i + 4]], False, color, thickness, cv2.LINE_AA)


def _diamond(img, x, y, r, color):
    pts = np.array([[x, y - r], [x + r, y], [x, y + r], [x - r, y]], np.int32)
    cv2.fillPoly(img, [pts], color, cv2.LINE_AA)
    cv2.polylines(img, [pts], True, (0, 0, 0), 1, cv2.LINE_AA)


def render(result, mode='PREVIEW', status=None, age_ms=None, rate_hz=None, show_stripes=True):
    """Return a new BGR image with boundaries, markers and status text.

    status overrides result.state for display (e.g. 'STALE'); a STALE or
    INVALID status never shows the green target.
    """
    det = result.detection
    if det is None:
        img = np.zeros((360, result.width or 640, 3), np.uint8)
    else:
        img = det.image.copy()
    H, W = img.shape[:2]
    status = status or result.state
    cv2.polylines(img, [det.roi_polygon] if det is not None else [], True, CYAN, 1, cv2.LINE_AA)

    used = {id(b.stripe) for b in (result.left, result.right) if b.stripe is not None}
    if det is not None and show_stripes:
        for s in det.stripes:
            if id(s) not in used:
                _polyline(img, s, s.y_min, s.y_max, GREY, 1)
                cv2.putText(img, f'{s.stripe_id}', (int(s.x_at(s.y_max)) + 3, s.y_max),
                            cv2.FONT_HERSHEY_PLAIN, 0.8, GREY, 1, cv2.LINE_AA)
    y_top = min([s.y_min for s in det.stripes], default=0) if det is not None else 0
    for b in (result.left, result.right):
        if b.stripe is None:
            continue
        color = SIDE_COLOR[b.state]
        s = b.stripe
        # Thick where paint was observed, thin dashed where the fit spans a gap.
        _polyline(img, s, max(y_top, s.y_min - 20), min(H - 1, s.y_max), color, 1, dashed=True)
        rows = s.support_rows
        breaks = np.nonzero(np.diff(rows) > 2)[0]
        for seg in np.split(rows, breaks + 1):
            _polyline(img, s, seg[0], seg[-1], color, 3)
        if b.group is not None and b.group.kind == 'double':
            for m in b.group.members:
                if m is not s:
                    _polyline(img, m, m.y_min, m.y_max, color, 1)
            gx = int(s.x_at(s.y_max))
            cv2.putText(img, f'G{b.group.group_id} double', (gx - 40, min(H - 4, s.y_max + 12)),
                        cv2.FONT_HERSHEY_PLAIN, 0.9, color, 1, cv2.LINE_AA)

    cx, cy = result.image_center
    ye = result.y_eval
    cv2.line(img, (int(round(cx)), 0), (int(round(cx)), H - 1), RED, 1, cv2.LINE_AA)
    cv2.line(img, (0, int(round(ye))), (W - 1, int(round(ye))), (90, 90, 90), 1)
    if result.x_lane is not None and status in (VALID, DEGRADED):
        xl = result.x_lane
        color = GREEN if status == VALID else AMBER
        cv2.line(img, (int(round(cx)), int(round(ye))), (int(round(xl)), int(round(ye))), color, 2, cv2.LINE_AA)
        for b in (result.left, result.right):
            cv2.circle(img, (int(round(b.x_eval)), int(round(ye))), 4, SIDE_COLOR[b.state], -1)
        _diamond(img, int(round(xl)), int(round(ye)), 8, color)
    cv2.circle(img, (int(round(cx)), int(round(cy))), 6, RED, -1, cv2.LINE_AA)
    _text_block(img, _lines(result, mode, status, age_ms, rate_hz), STATE_COLOR.get(status, WHITE))
    return img


def _fmt(v, spec, none='n/a'):
    return none if v is None else format(v, spec)


def _lines(result, mode, status, age_ms, rate_hz):
    s = result.steering
    age = 'static' if result.static else f'{_fmt(age_ms, ".0f")} ms'
    l, r = result.left, result.right
    lines = [
        f'{mode} | {status} | {result.reason} | frame age {age}',
        f'error: {_fmt(s and s.error_px, "+.1f")} px | alpha: {_fmt(s and s.alpha_effective, ".4f")} rad/px',
        f'raw: {_fmt(s and s.steer_raw_rad, "+.3f")} rad | output: {_fmt(s and s.steer_rad, "+.3f")} rad'
        f'{" (clipped)" if s is not None and s.clipped else ""} | requested, feedback n/a',
        f'processing: {result.width}x{result.height} | evaluation row: '
        f'{result.y_eval / max(1, result.height - 1):.2f} | {result.proc_ms:.1f} ms'
        f'{"" if rate_hz is None else f" | {rate_hz:.1f} Hz"}',
        f'L {l.state}{"" if l.state != HELD else f" {l.age_s * 1000:.0f}ms"} | '
        f'R {r.state}{"" if r.state != HELD else f" {r.age_s * 1000:.0f}ms"} | quality {result.quality:.2f}',
    ]
    if status == DEGRADED:
        lines.append('DEGRADED: held boundary, preview estimate only')
    return lines


def _text_block(img, lines, color):
    font, scale, th = cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1
    lh = 16
    w = max(cv2.getTextSize(t, font, scale, th)[0][0] for t in lines) + 10
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (min(img.shape[1] - 1, w), lh * len(lines) + 6), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, dst=img)
    for i, t in enumerate(lines):
        cv2.putText(img, t, (5, lh * (i + 1)), font, scale, color if i == 0 else WHITE, th, cv2.LINE_AA)
