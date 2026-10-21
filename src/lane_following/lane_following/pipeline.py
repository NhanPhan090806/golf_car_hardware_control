"""Lane pair selection, short-term tracking, and immutable per-frame results.

Pure Python/NumPy; no ROS or UI. The same LanePipeline.process() call is used
for a saved image, a video frame, a camera frame, or a ROS image.

Result states:
  VALID     both boundaries supported in this frame (OBSERVED/INTERPOLATED)
  DEGRADED  one boundary HELD from an earlier frame; preview estimate only,
            never eligible for actuation
  INVALID   no supported, unambiguous lane pair (see `reason`)
Freshness (STALE) is decided later by the consumer from acquisition time, so
a result object never claims to be fresh by itself.
"""

import dataclasses
import math
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .config import DetectorConfig, SteeringConfig, TrackerConfig
from .detector import (HELD, INTERPOLATED, MISSING, OBSERVED, STATE_RANK, BoundaryGroup,
                       Detection, LaneDetector, Stripe)
from .steering import PixelGainController, SteeringOutput

VALID = 'VALID'
DEGRADED = 'DEGRADED'
INVALID = 'INVALID'


@dataclasses.dataclass(frozen=True)
class Boundary:
    side: str  # 'left' or 'right'
    state: str  # OBSERVED / INTERPOLATED / HELD / MISSING
    x_eval: Optional[float] = None
    stripe: Optional[Stripe] = None  # lane-facing reference stripe
    group: Optional[BoundaryGroup] = None
    observed_t: Optional[float] = None  # acquisition time of supporting frame
    age_s: float = 0.0  # 0 for current-frame support

    def summary(self):
        return {
            'side': self.side, 'state': self.state,
            'x_eval': None if self.x_eval is None else round(self.x_eval, 2),
            'age_s': round(self.age_s, 3),
            'reference_stripe': None if self.stripe is None else self.stripe.summary(),
            'group': None if self.group is None else self.group.summary(),
            'reference_convention': 'lane-facing stripe centre',
        }


@dataclasses.dataclass(frozen=True)
class LaneResult:
    frame_id: int
    source: str
    acq_t: float  # local monotonic acquisition time
    proc_t: float  # local monotonic time processing finished
    proc_ms: float
    width: int
    height: int
    original_size: Tuple[int, int]
    scale: float
    y_eval: float
    image_center: Tuple[float, float]
    state: str
    reason: str
    left: Boundary
    right: Boundary
    x_lane: Optional[float] = None
    lane_width_px: Optional[float] = None
    in_lane: Optional[bool] = None
    quality: float = 0.0
    steering: Optional[SteeringOutput] = None
    static: bool = False
    detection: Optional[Detection] = dataclasses.field(default=None, repr=False, compare=False)

    @property
    def actuation_eligible(self):
        """Phase A never actuates; this only marks results a future gate may use."""
        return self.state == VALID and not self.static and self.steering is not None

    def summary(self) -> Dict[str, Any]:
        s = self.steering
        return {
            'frame_id': self.frame_id, 'source': self.source, 'state': self.state,
            'reason': self.reason, 'static': self.static,
            'processing_size': [self.width, self.height], 'original_size': list(self.original_size),
            'scale': round(self.scale, 5), 'y_eval': round(self.y_eval, 2),
            'image_center': [round(v, 2) for v in self.image_center],
            'x_lane': None if self.x_lane is None else round(self.x_lane, 2),
            'lane_width_px': None if self.lane_width_px is None else round(self.lane_width_px, 2),
            'in_lane': self.in_lane, 'quality': round(self.quality, 3),
            'proc_ms': round(self.proc_ms, 2),
            'error_px': None if s is None else round(s.error_px, 2),
            'alpha_rad_per_px': None if s is None else s.alpha_effective,
            'steer_raw_rad': None if s is None else round(s.steer_raw_rad, 5),
            'steer_rad': None if s is None else round(s.steer_rad, 5),
            'clipped': None if s is None else s.clipped,
            'left': self.left.summary(), 'right': self.right.summary(),
            'stripes': [] if self.detection is None else [st.summary() for st in self.detection.stripes],
            'groups': [] if self.detection is None else [g.summary() for g in self.detection.groups],
            'rejected_chains': {} if self.detection is None else dict(self.detection.rejected_chains),
        }
@dataclasses.dataclass
class _SideTrack:
    stripe: Stripe
    group: BoundaryGroup
    x_eval: float
    observed_t: float


@dataclasses.dataclass(frozen=True)
class _Candidate:
    group: BoundaryGroup
    stripe: Stripe  # lane-facing supported member for this side
    x: float
    state: str


class LanePipeline:
    """detector -> pair selection/tracking -> steering, one frame at a time."""

    def __init__(self, detector_cfg: DetectorConfig, steering_cfg: SteeringConfig,
                 tracker_cfg: TrackerConfig):
        self.dcfg, self.scfg, self.tcfg = detector_cfg, steering_cfg, tracker_cfg
        self.detector = LaneDetector(detector_cfg)
        self.controller = PixelGainController(steering_cfg)
        self.reset('init')
        self._geom = None

    def reset(self, why='reset'):
        """Forget temporal state (seek, restart, source/resolution change)."""
        self._track = {'left': None, 'right': None}
        self._prev_x_lane = None
        self.last_reset_reason = why

    # -- candidate extraction ------------------------------------------------
    def _candidates(self, det: Detection, y_eval):
        """Per side, groups whose lane-facing member is supported at y_eval."""
        px = self.detector.px
        out = {'left': [], 'right': []}
        for g in det.groups:
            states = [(m, float(m.x_at(y_eval)),
                       m.support_state(y_eval, px['max_extrapolation'], px['max_interpolation']))
                      for m in g.members]
            states = [s for s in states if s[2] != MISSING and math.isfinite(s[1])]
            if not states:
                continue
            states.sort(key=lambda s: s[1])
            # As a left boundary the lane is to its right: rightmost member.
            m, x, st = states[-1]
            out['left'].append(_Candidate(g, m, x, st))
            m, x, st = states[0]
            out['right'].append(_Candidate(g, m, x, st))
        return out

    @staticmethod
    def _nearest(cands, x_target, limit):
        best, best_d = None, limit
        for c in cands:
            d = abs(c.x - x_target)
            if d <= best_d:
                best, best_d = c, d
        return best
    def _initial_pair(self, cands, W):
        """Pick the pair straddling the configured lane reference column.

        Uses the innermost supported boundary on each side, so a multi-lane
        scene yields the target lane, not the two outermost lines. If the
        reference column sits on a boundary the lane is ambiguous.
        """
        ref = self.dcfg.lane_reference_x_ratio * (W - 1)
        margin = self.dcfg.ambiguity_margin_ratio * W
        by_group = {}
        for side in ('left', 'right'):
            for c in cands[side]:
                by_group.setdefault(c.group.group_id, {})[side] = c
        for g in by_group.values():
            lo, hi = g['right'].x, g['left'].x
            if lo - margin < ref < hi + margin:
                return None, None, 'REFERENCE_ON_BOUNDARY'
        left = max((c for c in cands['left'] if c.x < ref), key=lambda c: c.x, default=None)
        right = min((c for c in cands['right'] if c.x > ref), key=lambda c: c.x, default=None)
        return left, right, None

    def _tracked_pair(self, cands, W):
        jump = self.tcfg.max_jump_ratio * W
        chosen = {}
        for side in ('left', 'right'):
            tr = self._track[side]
            chosen[side] = None if tr is None else self._nearest(cands[side], tr.x_eval, jump)
        # Re-acquire a side whose track expired, ordered against the other one.
        ref = self.dcfg.lane_reference_x_ratio * (W - 1)
        min_w = self.dcfg.min_lane_width_ratio * W
        for side, other in (('left', 'right'), ('right', 'left')):
            if self._track[side] is not None:
                continue
            anchor = chosen[other].x if chosen[other] is not None else (
                self._track[other].x_eval if self._track[other] is not None else None)
            if anchor is None:
                continue
            ok = [c for c in cands[side]
                  if (c.x < anchor - min_w if side == 'left' else c.x > anchor + min_w)]
            chosen[side] = min(ok, key=lambda c: abs(c.x - ref), default=None)
        return chosen['left'], chosen['right']

    def _boundary(self, side, cand, acq_t, y_eval, tracking):
        if cand is not None:
            return Boundary(side, cand.state, cand.x, cand.stripe, cand.group, acq_t, 0.0)
        tr = self._track[side]
        hold = self.tcfg.hold_duration_s
        if tracking and tr is not None and hold > 0.0:
            # Age is measured from the last frame that actually supported the
            # side; re-using the model never refreshes observed_t.
            age = acq_t - tr.observed_t
            if 0.0 <= age <= hold:
                return Boundary(side, HELD, float(tr.stripe.x_at(y_eval)), tr.stripe, tr.group,
                                tr.observed_t, age)
        return Boundary(side, MISSING)

    def _pair_reason(self, det, left, right, y_eval):
        c = self.dcfg
        W, H = det.width, det.height
        if left.state == MISSING and right.state == MISSING:
            return 'BOTH_MISSING'
        if left.state == MISSING:
            return 'LEFT_MISSING'
        if right.state == MISSING:
            return 'RIGHT_MISSING'
        if not (math.isfinite(left.x_eval) and math.isfinite(right.x_eval)):
            return 'NONFINITE'
        width = right.x_eval - left.x_eval
        if width <= 0:
            return 'CROSSED'
        if width < c.min_lane_width_ratio * W:
            return 'LANE_TOO_NARROW'
        if width > c.max_lane_width_ratio * W:
            return 'LANE_TOO_WIDE'
        pair = (left.stripe.stripe_id, right.stripe.stripe_id)
        if left.state != HELD and right.state != HELD and \
                (pair in det.ambiguous_pairs or pair[::-1] in det.ambiguous_pairs):
            return 'AMBIGUOUS_GROUPING'
        y_top = self.detector.roi_y0
        for off in c.check_row_offsets:
            y = min(H - 1.0, max(float(y_top), y_eval + off * (H - 1)))
            w = float(right.stripe.x_at(y) - left.stripe.x_at(y))
            if not math.isfinite(w) or w < 0.5 * c.min_lane_width_ratio * W:
                return 'INCONSISTENT_PAIR'
        return None

    def _quality(self, b: Boundary, H):
        px = self.detector.px
        if b.stripe is None:
            return 0.0
        support = min(1.0, len(b.stripe.support_rows) / (0.25 * H))
        fit = max(0.0, 1.0 - b.stripe.rms_px / px['max_fit_rms'])
        q = 0.6 * support + 0.4 * fit
        return 0.5 * q if b.state == HELD else q
    # -- main entry -----------------------------------------------------------
    def process(self, image, acq_t: Optional[float] = None, frame_id: int = 0,
                source: str = 'unknown', static: bool = False) -> LaneResult:
        """Run one frame. `acq_t` is the monotonic acquisition time.

        static=True is for single-image inspection: no temporal state is
        read or written, so a still image can never borrow history.
        """
        acq_t = time.monotonic() if acq_t is None else float(acq_t)
        t0 = time.monotonic()
        tracking = not static
        try:
            det = self.detector.detect(image)
        except (ValueError, TypeError) as exc:
            return self._invalid_frame(frame_id, source, acq_t, t0, static, f'BAD_FRAME: {exc}')
        geom = (image.shape[1], image.shape[0], det.width, det.height)
        if tracking and geom != self._geom:
            if self._geom is not None:
                self.reset('resolution change')
            self._geom = geom
        W, H = det.width, det.height
        y_eval = self.dcfg.evaluation_row_ratio * (H - 1)
        center = ((W - 1) / 2.0, (H - 1) / 2.0)
        cands = self._candidates(det, y_eval)

        have_track = tracking and (self._track['left'] is not None or self._track['right'] is not None)
        if have_track:
            lc, rc = self._tracked_pair(cands, W)
            sel_reason = None
        else:
            lc, rc, sel_reason = self._initial_pair(cands, W)
        left = self._boundary('left', lc, acq_t, y_eval, tracking)
        right = self._boundary('right', rc, acq_t, y_eval, tracking)
        reason = sel_reason or self._pair_reason(det, left, right, y_eval)

        steering, x_lane, width, in_lane, quality = None, None, None, None, 0.0
        if reason is None:
            x_lane = 0.5 * (left.x_eval + right.x_eval)
            width = right.x_eval - left.x_eval
            in_lane = left.x_eval < center[0] < right.x_eval
            steering = self.controller.compute(x_lane - center[0], W)
            continuity = 1.0
            if self._prev_x_lane is not None:
                continuity = max(0.0, 1.0 - abs(x_lane - self._prev_x_lane) /
                                 (self.tcfg.max_jump_ratio * W))
            quality = min(self._quality(left, H), self._quality(right, H)) * (0.5 + 0.5 * continuity)
            held = HELD in (left.state, right.state)
            state = DEGRADED if held else VALID
            reason = ('HELD_' + ('LEFT' if left.state == HELD else 'RIGHT')) if held else \
                ('OUT_OF_LANE' if not in_lane else 'OK')
        else:
            state = INVALID

        if tracking:
            self._update_tracks(left, right, acq_t, state != INVALID)
            self._prev_x_lane = x_lane if state != INVALID else self._prev_x_lane
        t1 = time.monotonic()
        return LaneResult(
            frame_id=frame_id, source=source, acq_t=acq_t, proc_t=t1, proc_ms=1000.0 * (t1 - t0),
            width=W, height=H, original_size=(image.shape[1], image.shape[0]), scale=det.scale,
            y_eval=y_eval, image_center=center, state=state, reason=reason, left=left, right=right,
            x_lane=x_lane, lane_width_px=width, in_lane=in_lane, quality=quality, steering=steering,
            static=static, detection=det)

    def _update_tracks(self, left, right, acq_t, pair_ok):
        for b in (left, right):
            tr = self._track[b.side]
            if pair_ok and b.state in (OBSERVED, INTERPOLATED):
                self._track[b.side] = _SideTrack(b.stripe, b.group, b.x_eval, acq_t)
            elif tr is not None and acq_t - tr.observed_t > self.tcfg.track_timeout_s:
                self._track[b.side] = None
        if self._track['left'] is None and self._track['right'] is None:
            self._prev_x_lane = None

    def _invalid_frame(self, frame_id, source, acq_t, t0, static, reason):
        t1 = time.monotonic()
        W = self.dcfg.processing_width
        return LaneResult(
            frame_id=frame_id, source=source, acq_t=acq_t, proc_t=t1, proc_ms=1000.0 * (t1 - t0),
            width=W, height=0, original_size=(0, 0), scale=0.0, y_eval=0.0,
            image_center=((W - 1) / 2.0, 0.0), state=INVALID, reason=reason,
            left=Boundary('left', MISSING), right=Boundary('right', MISSING), static=static)
