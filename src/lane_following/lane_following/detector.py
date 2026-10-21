"""Classical lane-marking detection on one frame (pure OpenCV/NumPy, no ROS).

Stages, all in processing-image pixels (x right, y down):
  1. Resize to processing_width (aspect preserved) and mask the road ROI.
  2. Paint mask: horizontal top-hat on HSV value (paint is brighter than the
     asphalt next to it) AND a white-or-yellow HSV gate, then small morphology.
  3. Per-row runs of paint pixels give a stripe centre, width and colour.
  4. Runs are linked row to row into stripe pieces (one-to-one, slope aware).
  5. Pieces are chained across dash gaps by combined-fit residual, and each
     chain is fitted robustly as x = f(y) (linear, quadratic fallback).
  6. Parallel same-colour stripes at double-line spacing, judged row by row
     so perspective is handled, form one boundary group.

This replaces the plan's Canny + HoughLinesP stage: runs of the filled paint
mask give stripe centres directly, so the two Canny edges of one stripe never
need deduplicating, and dash gaps are bridged by explicit chaining instead of
large Hough gap settings.
"""

import dataclasses
import math
from typing import Dict, List, Tuple

import cv2
import numpy as np

from .config import DetectorConfig

OBSERVED = 'OBSERVED'
INTERPOLATED = 'INTERPOLATED'
HELD = 'HELD'
MISSING = 'MISSING'
STATE_RANK = {MISSING: 0, HELD: 1, INTERPOLATED: 2, OBSERVED: 3}


def _odd(n):
    n = max(1, int(round(n)))
    return n if n % 2 else n + 1


@dataclasses.dataclass(frozen=True, eq=False)
class Stripe:
    """One painted stripe (solid, or a chain of dashes) fitted as x = f(y)."""

    stripe_id: int
    coeffs: Tuple[float, ...]
    width_coeffs: Tuple[float, ...]
    support_rows: np.ndarray  # sorted unique rows with paint evidence
    y_min: int
    y_max: int
    rms_px: float
    color: str  # 'white' or 'yellow'
    style: str  # 'solid' or 'dashed'
    fit_mode: str  # 'linear' or 'quadratic'
    n_pieces: int

    def x_at(self, y):
        return np.polyval(self.coeffs, y)

    def width_at(self, y):
        return max(1.0, float(np.polyval(self.width_coeffs, y)))

    def support_state(self, y, max_extrap_px, max_interp_px):
        """OBSERVED / INTERPOLATED / MISSING for row y, from this frame only."""
        rows = self.support_rows
        i = int(np.searchsorted(rows, y))
        below = rows[i] - y if i < len(rows) else math.inf
        above = y - rows[i - 1] if i > 0 else math.inf
        if min(above, below) <= max_extrap_px:
            return OBSERVED
        if math.isfinite(above) and math.isfinite(below) and above + below <= max_interp_px:
            return INTERPOLATED
        return MISSING

    def summary(self):
        return {
            'id': self.stripe_id, 'coeffs': [round(c, 6) for c in self.coeffs],
            'y_range': [self.y_min, self.y_max], 'support_rows': int(len(self.support_rows)),
            'rms_px': round(self.rms_px, 3), 'color': self.color, 'style': self.style,
            'fit_mode': self.fit_mode, 'pieces': self.n_pieces,
        }


@dataclasses.dataclass(frozen=True, eq=False)
class BoundaryGroup:
    """One boundary: a single stripe, or parallel stripes forming a double line."""

    group_id: int
    members: Tuple[Stripe, ...]  # sorted left to right at the bottom of overlap
    kind: str  # 'single' or 'double'
    color: str
    ambiguous: bool
    grouping_score: float  # min fraction of rows at double spacing (1.0 for single)
    reason: str

    def x_at(self, y):
        return float(np.mean([m.x_at(y) for m in self.members]))

    def summary(self):
        return {
            'id': self.group_id, 'kind': self.kind, 'color': self.color,
            'ambiguous': self.ambiguous, 'grouping_score': round(self.grouping_score, 3),
            'reason': self.reason, 'members': [m.stripe_id for m in self.members],
        }


@dataclasses.dataclass(frozen=True, eq=False)
class Detection:
    """Everything one frame produced, at processing resolution."""

    image: np.ndarray  # resized BGR, read-only
    scale: float  # processing pixels per original pixel
    roi_polygon: np.ndarray  # int32 (N, 2)
    stripes: Tuple[Stripe, ...]
    groups: Tuple[BoundaryGroup, ...]
    paint_pixels: int
    rejected_chains: Dict[str, int]
    # Stripe-id pairs whose double spacing was inconclusive.
    ambiguous_pairs: Tuple[Tuple[int, int], ...] = ()

    @property
    def height(self):
        return self.image.shape[0]

    @property
    def width(self):
        return self.image.shape[1]


class _Piece:
    """Row-linked run sequence; becomes one dash or part of a solid line."""

    __slots__ = ('ys', 'xs', 'ws', 'yellow', 'last_y')

    def __init__(self, y, x, w, yellow):
        self.ys, self.xs, self.ws, self.yellow = [y], [x], [w], [yellow]
        self.last_y = y

    def add(self, y, x, w, yellow):
        self.ys.append(y)
        self.xs.append(x)
        self.ws.append(w)
        self.yellow.append(yellow)
        self.last_y = y

    def predict(self, y):
        # Rows are scanned bottom-up, so ys decrease. Local slope from up to
        # 6 rows back; zero slope until there is enough history.
        n = len(self.ys)
        if n < 3:
            return self.xs[-1]
        k = min(n - 1, 6)
        dy = self.ys[-1] - self.ys[-1 - k]
        slope = (self.xs[-1] - self.xs[-1 - k]) / dy if dy else 0.0
        return self.xs[-1] + slope * (y - self.ys[-1])


class LaneDetector:
    def __init__(self, config: DetectorConfig):
        self.config = config
        self._geom_key = None

    # -- geometry derived from the processing size -------------------------
    def _prepare(self, in_w, in_h):
        c = self.config
        W = c.processing_width
        H = max(32, int(round(in_h * W / in_w)))
        if self._geom_key == (in_w, in_h):
            return
        self._geom_key = (in_w, in_h)
        self.W, self.H, self.scale = W, H, W / in_w
        pts = np.array(c.roi, dtype=np.float64).reshape(-1, 2)
        self.roi_polygon = np.round(pts * [W - 1, H - 1]).astype(np.int32)
        self.roi_mask = np.zeros((H, W), np.uint8)
        cv2.fillPoly(self.roi_mask, [self.roi_polygon], 255)
        rows = np.nonzero(self.roi_mask.any(axis=1))[0]
        self.roi_y0, self.roi_y1 = int(rows.min()), int(rows.max())
        self.tophat_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (_odd(c.tophat_width_ratio * W), 1))
        close_h = _odd(c.vertical_close_ratio * H)
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, close_h)) if close_h > 1 else None
        # Lengths that must scale with the image use H; lateral ones use W.
        self.px = {
            'tophat_width': self.tophat_kernel.shape[1],
            'vertical_close': close_h,
            'cluster_max_gap': c.cluster_max_gap_ratio * W,
            'link_tolerance': max(2.0, c.link_tolerance_ratio * W),
            'association_residual': max(1.5, c.association_residual_ratio * W),
            'max_chain_gap': c.max_chain_gap_ratio * H,
            'min_support': max(4, int(round(c.min_support_ratio * H))),
            'min_span': max(6, int(round(c.min_span_ratio * H))),
            'max_fit_rms': max(1.0, c.max_fit_rms_ratio * W),
            'max_bow': c.max_bow_ratio * W,
            'max_extrapolation': max(2.0, c.max_extrapolation_ratio * H),
            'max_interpolation': c.max_interpolation_ratio * H,
            'max_run_width': max(6, int(round(c.tophat_width_ratio * W))),
        }

    def effective_pixels(self, in_w, in_h):
        """Pixel values derived from the normalized settings, for logging."""
        self._prepare(in_w, in_h)
        out = {'processing_size': [self.W, self.H], 'roi_rows': [self.roi_y0, self.roi_y1]}
        out.update({k: round(float(v), 2) for k, v in self.px.items()})
        return out

    # -- stage 2: paint mask ------------------------------------------------
    def paint_masks(self, image):
        """Return (paint, yellow) uint8 masks of the processing image."""
        c = self.config
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        tophat = cv2.morphologyEx(v, cv2.MORPH_TOPHAT, self.tophat_kernel)
        contrast = cv2.threshold(tophat, c.min_contrast - 1, 255, cv2.THRESH_BINARY)[1]
        white = cv2.inRange(hsv, (0, 0, c.white_min_value), (180, c.white_max_saturation, 255))
        yellow = cv2.inRange(hsv, (c.yellow_hue_min, c.yellow_min_saturation, c.yellow_min_value),
                             (c.yellow_hue_max, 255, 255))
        paint = cv2.bitwise_and(contrast, cv2.bitwise_or(white, yellow))
        paint = cv2.bitwise_and(paint, self.roi_mask)
        # Opening with a 1-pixel-high kernel removes speckle narrower than the
        # minimum run; a short vertical close bridges row dropouts in a stripe.
        if c.min_run_width_px > 1:
            paint = cv2.morphologyEx(paint, cv2.MORPH_OPEN,
                                     cv2.getStructuringElement(cv2.MORPH_RECT, (c.min_run_width_px, 1)))
        if self.close_kernel is not None:
            paint = cv2.morphologyEx(paint, cv2.MORPH_CLOSE, self.close_kernel)
        return paint, cv2.bitwise_and(yellow, paint)

    # -- stage 3: row runs --------------------------------------------------
    def _row_runs(self, paint, yellow):
        """Arrays (y, x_center, width, yellow_flag) of horizontal paint runs."""
        band = paint[self.roi_y0:self.roi_y1 + 1] > 0
        padded = np.zeros((band.shape[0], band.shape[1] + 2), np.int8)
        padded[:, 1:-1] = band
        d = np.diff(padded, axis=1)
        sy, sx = np.nonzero(d == 1)
        ey, ex = np.nonzero(d == -1)  # same row-major order as starts
        width = ex - sx
        keep = (width >= self.config.min_run_width_px) & (width <= self.px['max_run_width'])
        sy, sx, ex, width = sy[keep], sx[keep], ex[keep], width[keep]
        ycs = np.cumsum((yellow[self.roi_y0:self.roi_y1 + 1] > 0).astype(np.int32), axis=1)
        ycs = np.pad(ycs, ((0, 0), (1, 0)))
        yellow_frac = (ycs[sy, ex] - ycs[sy, sx]) / np.maximum(width, 1)
        return (sy + self.roi_y0, (sx + ex - 1) / 2.0, width.astype(np.float64), yellow_frac >= 0.5)

    # -- stage 4: link runs into pieces -------------------------------------
    def _link(self, ys, xs, ws, yl):
        c = self.config
        tol = self.px['link_tolerance']
        order = np.argsort(-ys, kind='stable')  # bottom row first
        ys, xs, ws, yl = ys[order], xs[order], ws[order], yl[order]
        bounds = np.nonzero(np.diff(ys))[0] + 1
        starts = np.concatenate(([0], bounds))
        ends = np.concatenate((bounds, [len(ys)]))
        active, done = [], []
        for s, e in zip(starts, ends):
            if s == e:
                continue
            y = int(ys[s])
            still = []
            for p in active:
                (still if p.last_y - y <= c.link_row_gap + 1 else done).append(p)
            active = still
            rx, rw, ry = xs[s:e], ws[s:e], yl[s:e]
            taken = np.zeros(e - s, bool)
            if active:
                pred = np.array([p.predict(y) for p in active])
                pw = np.array([p.ws[-1] for p in active])
                dist = np.abs(pred[:, None] - rx[None, :])
                lim = tol + 0.25 * np.maximum(pw[:, None], rw[None, :])
                dist[dist > lim] = np.inf
                # Greedy one-to-one assignment by distance: each run feeds at
                # most one piece, so close double stripes stay separate.
                pairs = np.argwhere(np.isfinite(dist))
                if len(pairs):
                    for pi, ri in pairs[np.argsort(dist[pairs[:, 0], pairs[:, 1]], kind='stable')]:
                        p = active[pi]
                        if taken[ri] or p.last_y == y:
                            continue
                        p.add(y, float(rx[ri]), float(rw[ri]), bool(ry[ri]))
                        taken[ri] = True
            for ri in np.nonzero(~taken)[0]:
                active.append(_Piece(y, float(rx[ri]), float(rw[ri]), bool(ry[ri])))
        done.extend(active)
        return [p for p in done if len(p.ys) >= c.min_piece_rows]

    # -- stage 5: chain pieces across dash gaps -----------------------------
    def _chain(self, pieces):
        assoc = self.px['association_residual']
        max_gap = self.px['max_chain_gap']
        max_bow = self.px['max_bow']
        items = []
        for p in pieces:
            y = np.asarray(p.ys, np.float64)
            items.append({'y': y, 'x': np.asarray(p.xs, np.float64), 'w': np.asarray(p.ws, np.float64),
                          'yellow': float(np.mean(p.yellow)) >= 0.5,
                          'y_min': float(y.min()), 'y_max': float(y.max())})
        items.sort(key=lambda it: -it['y_max'])
        chains = []
        for it in items:
            best, best_rms = None, math.inf
            for ch in chains:
                if ch['yellow'] != it['yellow']:
                    continue
                gap = ch['y_min'] - it['y_max']
                # Pieces are visited bottom-up, so a new piece is above or
                # beside a chain. Beside (overlapping rows) means a parallel
                # stripe, which must never join the same chain.
                if gap > max_gap or gap < -self.config.link_row_gap:
                    continue
                ends = it['y'][[0, -1]]
                if np.max(np.abs(np.polyval(ch['line'], ends) - it['x'][[0, -1]])) > \
                        3.0 * assoc + 0.5 * max(gap, 0.0):
                    continue
                y = np.concatenate((ch['y'], it['y']))
                x = np.concatenate((ch['x'], it['x']))
                span = y.max() - y.min()
                deg = 2 if self.config.fit_mode == 'auto' and ch['pieces'] >= 2 and span >= 0.35 * self.H else 1
                coef = np.polyfit(y, x, deg)
                if deg == 2 and abs(coef[0]) * span * span / 4.0 > max_bow:
                    continue
                r = x - np.polyval(coef, y)
                rms_new = float(np.sqrt(np.mean(r[len(ch['y']):] ** 2)))
                rms_all = float(np.sqrt(np.mean(r ** 2)))
                if rms_new <= assoc and rms_all <= assoc and rms_new < best_rms:
                    best, best_rms = ch, rms_new
            if best is None:
                ch = dict(it, pieces=1)
                chains.append(ch)
            else:
                ch = best
                ch['y'] = np.concatenate((ch['y'], it['y']))
                ch['x'] = np.concatenate((ch['x'], it['x']))
                ch['w'] = np.concatenate((ch['w'], it['w']))
                ch['y_min'] = min(ch['y_min'], it['y_min'])
                ch['pieces'] += 1
            ch['line'] = np.polyfit(ch['y'], ch['x'], 1) if np.ptp(ch['y']) > 0 else \
                np.array([0.0, float(np.mean(ch['x']))])
        return chains

    # -- stage 5b: robust fit of one chain ----------------------------------
    @staticmethod
    def _robust_fit(y, x, deg, tol):
        coef = np.polyfit(y, x, deg)
        r = np.abs(x - np.polyval(coef, y))
        inl = r <= max(tol, 2.5 * float(np.median(r)))
        if inl.sum() > deg + 2 and not inl.all():
            coef = np.polyfit(y[inl], x[inl], deg)
            r = np.abs(x - np.polyval(coef, y))
            inl = r <= max(tol, 2.5 * float(np.median(r[inl])))
        rms = float(np.sqrt(np.mean((x[inl] - np.polyval(coef, y[inl])) ** 2))) if inl.any() else math.inf
        return coef, inl, rms

    def _fit_stripe(self, ch, stripe_id, rejected):
        c = self.config
        px = self.px
        y, x, w = ch['y'], ch['x'], ch['w']
        rows = np.unique(y.astype(np.int64))
        if len(rows) < px['min_support']:
            rejected['support'] = rejected.get('support', 0) + 1
            return None
        span = float(y.max() - y.min())
        if span < px['min_span']:
            rejected['span'] = rejected.get('span', 0) + 1
            return None
        tol = px['max_fit_rms']
        coef, inl, rms = self._robust_fit(y, x, 1, tol)
        mode = 'linear'
        if rms > tol or inl.mean() < c.min_inlier_fraction:
            if c.fit_mode != 'auto' or span < 0.3 * self.H:
                rejected['residual'] = rejected.get('residual', 0) + 1
                return None
            coef, inl, rms = self._robust_fit(y, x, 2, tol)
            if rms > tol or inl.mean() < c.min_inlier_fraction or \
                    abs(coef[0]) * span * span / 4.0 > px['max_bow']:
                rejected['residual'] = rejected.get('residual', 0) + 1
                return None
            mode = 'quadratic'
        y_in = y[inl]
        y0, y1 = int(y_in.min()), int(y_in.max())
        dcoef = np.polyder(coef)
        if max(abs(np.polyval(dcoef, y0)), abs(np.polyval(dcoef, y1))) > c.max_abs_slope:
            rejected['slope'] = rejected.get('slope', 0) + 1
            return None
        wcoef = np.polyfit(y_in, w[inl], 1) if np.ptp(y_in) > 0 else np.array([0.0, float(np.mean(w))])
        support = np.unique(y_in.astype(np.int64))
        coverage = len(support) / float(y1 - y0 + 1)
        return Stripe(
            stripe_id=stripe_id, coeffs=tuple(float(v) for v in coef),
            width_coeffs=tuple(float(v) for v in wcoef), support_rows=support,
            y_min=y0, y_max=y1, rms_px=rms, color='yellow' if ch['yellow'] else 'white',
            style='solid' if coverage >= c.solid_support_fraction else 'dashed',
            fit_mode=mode, n_pieces=int(ch['pieces']))

    # -- stage 6: double-stripe grouping ------------------------------------
    def _pair_score(self, a, b):
        """Fraction of shared support rows where a and b sit at double spacing.

        The limit is judged per row from the local stripe width, so it
        follows perspective instead of one whole-image pixel gap.
        """
        if a.color != b.color:
            return None
        shared = np.intersect1d(a.support_rows, b.support_rows, assume_unique=True)
        # Need real side-by-side evidence: a short overlap where one stripe
        # ends and the next begins (curve split) is not a double line.
        if len(shared) < max(self.px['min_support'],
                             0.3 * min(len(a.support_rows), len(b.support_rows))):
            return None
        sep = np.abs(a.x_at(shared) - b.x_at(shared))
        wa = np.maximum(1.0, np.polyval(a.width_coeffs, shared))
        wb = np.maximum(1.0, np.polyval(b.width_coeffs, shared))
        gap = sep - 0.5 * (wa + wb)
        lim = np.minimum(self.config.cluster_gap_factor * np.maximum(wa, wb), self.px['cluster_max_gap'])
        return float(np.mean((gap > 0.0) & (gap <= lim)))

    def _group(self, stripes):
        c = self.config
        parent = list(range(len(stripes)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        ambiguous, scores = set(), {}
        self._ambiguous_pairs = []
        for i in range(len(stripes)):
            for j in range(i + 1, len(stripes)):
                f = self._pair_score(stripes[i], stripes[j])
                if f is None or f <= c.single_max_fraction:
                    continue
                if f >= c.double_min_fraction:
                    parent[find(i)] = find(j)
                    scores[i] = min(scores.get(i, 1.0), f)
                    scores[j] = min(scores.get(j, 1.0), f)
                else:
                    ambiguous.update((i, j))
                    self._ambiguous_pairs.append((stripes[i].stripe_id, stripes[j].stripe_id))
                    scores[i] = min(scores.get(i, 1.0), f)
                    scores[j] = min(scores.get(j, 1.0), f)
        buckets = {}
        for i in range(len(stripes)):
            buckets.setdefault(find(i), []).append(i)
        groups = []
        for gid, idx in enumerate(sorted(buckets.values(), key=lambda v: min(v))):
            members = [stripes[i] for i in idx]
            y_ref = 0.5 * (max(m.y_min for m in members) + min(m.y_max for m in members))
            members.sort(key=lambda m: float(m.x_at(y_ref)))
            amb = any(i in ambiguous for i in idx)
            score = min(scores.get(i, 1.0) for i in idx)
            kind = 'double' if len(members) > 1 else 'single'
            if amb:
                reason = 'AMBIGUOUS_GROUPING'
            elif kind == 'double':
                reason = 'DOUBLE_SPACING'
            else:
                reason = 'SINGLE'
            groups.append(BoundaryGroup(gid, tuple(members), kind, members[0].color, amb, score, reason))
        return groups

    # -- public entry -------------------------------------------------------
    def detect(self, image):
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 \
                or image.shape[2] != 3 or image.shape[0] < 16 or image.shape[1] < 16:
            raise ValueError('detector expects an HxWx3 uint8 BGR image of at least 16x16')
        in_h, in_w = image.shape[:2]
        self._prepare(in_w, in_h)
        if (in_w, in_h) == (self.W, self.H):
            proc = image.copy()
        else:
            interp = cv2.INTER_AREA if self.W < in_w else cv2.INTER_LINEAR
            proc = cv2.resize(image, (self.W, self.H), interpolation=interp)
        paint, yellow = self.paint_masks(proc)
        pieces = self._link(*self._row_runs(paint, yellow))
        rejected = {}
        stripes = []
        for ch in self._chain(pieces):
            s = self._fit_stripe(ch, len(stripes), rejected)
            if s is not None:
                stripes.append(s)
        groups = self._group(stripes)
        proc.setflags(write=False)
        return Detection(proc, self.scale, self.roi_polygon.copy(), tuple(stripes), tuple(groups),
                         int(np.count_nonzero(paint)), rejected, tuple(self._ambiguous_pairs))
