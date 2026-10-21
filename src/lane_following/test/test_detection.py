"""Detector + pair selection on synthetic scenes with known geometry."""

import numpy as np
import pytest

from lane_following import synthetic as sy
from lane_following.config import DetectorConfig, SteeringConfig, TrackerConfig
from lane_following.detector import INTERPOLATED, OBSERVED
from lane_following.pipeline import INVALID, VALID, LanePipeline

W, H = 640, 360
Y_EVAL = 0.75 * (H - 1)
CX = (W - 1) / 2.0
TOL = 3.0  # px


def pipe(**det):
    return LanePipeline(DetectorConfig(**det), SteeringConfig(), TrackerConfig())


def expected_error(lm, rm):
    return 0.5 * (lm.x_at(Y_EVAL, H) + rm.x_at(Y_EVAL, H)) - CX


@pytest.mark.parametrize('left_dashed,right_dashed', [(False, False), (False, True), (True, False), (True, True)])
@pytest.mark.parametrize('shift', [-50.0, 0.0, 50.0])
def test_solid_dashed_combinations(left_dashed, right_dashed, shift):
    img, lm, rm = sy.two_lane(left_x=170 + shift, right_x=470 + shift,
                              left_dashed=left_dashed, right_dashed=right_dashed)
    r = pipe().process(img, static=True)
    assert r.state == VALID, r.reason
    assert r.steering.error_px == pytest.approx(expected_error(lm, rm), abs=TOL)
    assert np.sign(r.steering.steer_rad) == np.sign(shift) or shift == 0.0
    styles = {r.left.stripe.style, r.right.stripe.style}
    assert ('dashed' in styles) == (left_dashed or right_dashed)


def test_paint_absent_exactly_at_eval_row_is_interpolated():
    gap = (0.70, 0.80)  # no paint across y_eval=0.75 on either side
    img, lm, rm = sy.two_lane(left={'gaps': [gap]}, right={'gaps': [gap]})
    r = pipe().process(img, static=True)
    assert r.state == VALID, r.reason
    assert r.left.state == INTERPOLATED and r.right.state == INTERPOLATED
    assert r.steering.error_px == pytest.approx(expected_error(lm, rm), abs=TOL)


def test_staggered_dash_gaps():
    img, lm, rm = sy.two_lane(left_dashed=True, right_dashed=True,
                              left={'dash_phase': 0.0}, right={'dash_phase': 0.09})
    r = pipe().process(img, static=True)
    assert r.state == VALID, r.reason
    assert {r.left.state, r.right.state} <= {OBSERVED, INTERPOLATED}


def test_dash_spacing_beyond_limit_is_not_bridged():
    # Long gaps with short dashes: no boundary should claim support at y_eval.
    img, _, _ = sy.two_lane(right_dashed=True, right={'dash_len': 0.04, 'gap_len': 0.40, 'dash_phase': 0.2})
    r = pipe().process(img, static=True)
    assert r.state == INVALID
    assert r.reason in ('RIGHT_MISSING', 'BOTH_MISSING')


@pytest.mark.parametrize('drop,reason', [('left', 'LEFT_MISSING'), ('right', 'RIGHT_MISSING')])
def test_one_boundary_absent_is_invalid_not_invented(drop, reason):
    _, lm, rm = sy.two_lane()
    img = sy.scene([rm] if drop == 'left' else [lm])
    r = pipe().process(img, static=True)
    assert r.state == INVALID and r.reason == reason
    assert r.steering is None and r.x_lane is None


def test_both_absent():
    r = pipe().process(sy.road(), static=True)
    assert r.state == INVALID and r.reason == 'BOTH_MISSING'


def test_curved_dash_chain_uses_quadratic_fallback():
    img, lm, rm = sy.two_lane(left_dashed=True, right_dashed=True,
                              left={'curve': 0.0022}, right={'curve': 0.0022})
    r = pipe().process(img, static=True)
    assert r.state == VALID, r.reason
    assert r.steering.error_px == pytest.approx(expected_error(lm, rm), abs=4.0)
    assert 'quadratic' in {r.left.stripe.fit_mode, r.right.stripe.fit_mode}


# -- double yellow and target-lane selection ---------------------------------

def three_marking_road(lane_ref, double_gap=10.0, center_x=300.0):
    """Left white edge, centre double yellow, right white edge."""
    vx, dy = 320.0, 0.5 * (H - 1)
    k = 0.55 / dy

    def m(x, **kw):
        return sy.Marking(x, (vx - x) * k, **kw)

    left = m(20.0)
    center = m(center_x, color=sy.YELLOW, width_bottom=9.0, width_top=4.0, double_gap=double_gap)
    right = m(620.0)
    img = sy.scene([left, center, right])
    return pipe(lane_reference_x_ratio=lane_ref).process(img, static=True), left, center, right


def test_double_yellow_right_lane_uses_lane_facing_stripe():
    r, left, center, right = three_marking_road(lane_ref=0.75)
    assert r.state == VALID, r.reason
    assert r.left.group.kind == 'double' and len(r.left.group.members) == 2
    # Left reference is the right-hand (lane-facing) yellow stripe.
    xs = sorted(m.x_at(Y_EVAL) for m in r.left.group.members)
    assert r.left.x_eval == pytest.approx(xs[1])
    assert r.right.x_eval == pytest.approx(right.x_at(Y_EVAL, H), abs=TOL)
    # Target lies in the driving lane, not in the narrow separator gap.
    assert r.x_lane > xs[1] + 20


def test_double_yellow_mirrored_left_lane():
    r, left, center, right = three_marking_road(lane_ref=0.2)
    assert r.state == VALID, r.reason
    assert r.right.group.kind == 'double'
    xs = sorted(m.x_at(Y_EVAL) for m in r.right.group.members)
    assert r.right.x_eval == pytest.approx(xs[0])
    assert r.left.x_eval == pytest.approx(left.x_at(Y_EVAL, H), abs=TOL)
    assert r.x_lane < xs[0] - 20


def test_reference_in_separator_gap_is_rejected():
    r, *_ = three_marking_road(lane_ref=300.0 / 639.0)
    assert r.state == INVALID and r.reason == 'REFERENCE_ON_BOUNDARY'


def test_single_wide_yellow_is_not_a_false_double():
    _, lm, rm = sy.two_lane()
    wide = sy.Marking(lm.x_bottom, lm.slope, color=sy.YELLOW, width_bottom=22.0, width_top=9.0)
    r = pipe().process(sy.scene([wide, rm]), static=True)
    assert r.state == VALID, r.reason
    assert r.left.group.kind == 'single'


def test_two_same_colour_lane_boundaries_stay_separate():
    img, lm, rm = sy.two_lane(left={'color': sy.YELLOW}, right={'color': sy.YELLOW})
    r = pipe().process(img, static=True)
    assert r.state == VALID, r.reason
    assert all(g.kind == 'single' for g in r.detection.groups)


def test_partially_occluded_double_member_reports_single():
    vx, dy = 320.0, 0.5 * (H - 1)
    k = 0.55 / dy
    a = sy.Marking(300.0, (vx - 300.0) * k, color=sy.YELLOW, width_bottom=9, width_top=4)
    b = sy.Marking(318.0, (vx - 318.0) * k, color=sy.YELLOW, width_bottom=9, width_top=4,
                   y_top_ratio=0.9)  # only a short visible piece
    right = sy.Marking(620.0, (vx - 620.0) * k)
    r = pipe(lane_reference_x_ratio=0.75).process(sy.scene([a, b, right]), static=True)
    # Not enough shared support to claim a double: no invented second stripe.
    assert r.left.group is None or r.left.group.kind == 'single'


def test_multi_lane_picks_lane_around_reference_not_outermost():
    vx, dy = 320.0, 0.5 * (H - 1)
    k = 0.55 / dy
    xs = [20.0, 230.0, 420.0, 620.0]
    marks = [sy.Marking(x, (vx - x) * k, dashed=(i in (1, 2))) for i, x in enumerate(xs)]
    r = pipe().process(sy.scene(marks), static=True)
    assert r.state == VALID, r.reason
    assert r.left.x_eval == pytest.approx(marks[1].x_at(Y_EVAL, H), abs=TOL)
    assert r.right.x_eval == pytest.approx(marks[2].x_at(Y_EVAL, H), abs=TOL)


def test_bad_frames_are_invalid_not_exceptions():
    p = pipe()
    for bad in (np.zeros((10, 10, 3), np.uint8), np.zeros((100, 100), np.uint8),
                np.zeros((100, 100, 3), np.float32)):
        r = p.process(bad, static=True)
        assert r.state == INVALID and r.reason.startswith('BAD_FRAME')
