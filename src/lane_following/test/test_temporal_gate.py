"""Tracking, HELD expiry, reset, and the per-tick freshness gate."""

import threading
import time

import numpy as np
import pytest

from lane_following import synthetic as sy
from lane_following.config import DetectorConfig, SteeringConfig, TrackerConfig
from lane_following.detector import HELD, MISSING
from lane_following.gate import STALE, decide
from lane_following.input_sources import LatestSlot, SourceThread, SyntheticSource
from lane_following.pipeline import DEGRADED, INVALID, VALID, LanePipeline

H = 360
Y_EVAL = 0.75 * (H - 1)


def pipe(hold=0.10):
    return LanePipeline(DetectorConfig(), SteeringConfig(), TrackerConfig(hold_duration_s=hold))


def frames():
    full, lm, rm = sy.two_lane()
    no_right = sy.scene([lm])
    return full, no_right, lm, rm


def test_brief_loss_is_held_then_recovers():
    full, no_right, *_ = frames()
    p = pipe()
    assert p.process(full, 0.00, 0).state == VALID
    r = p.process(no_right, 0.05, 1)
    assert r.state == DEGRADED and r.right.state == HELD
    assert r.right.age_s == pytest.approx(0.05)
    assert p.process(full, 0.10, 2).state == VALID


def test_hold_expires_by_elapsed_time_even_with_fresh_frames():
    full, no_right, *_ = frames()
    p = pipe(hold=0.10)
    p.process(full, 0.0, 0)
    states = [p.process(no_right, t, i + 1) for i, t in enumerate((0.04, 0.08, 0.12, 0.16))]
    assert [s.state for s in states[:2]] == [DEGRADED, DEGRADED]
    # Holding never refreshed the observation time, so it expires.
    assert states[2].state == INVALID and states[2].reason == 'RIGHT_MISSING'
    assert states[3].state == INVALID
    assert states[1].right.observed_t == pytest.approx(0.0)


def test_hold_disabled_with_zero():
    full, no_right, *_ = frames()
    p = pipe(hold=0.0)
    p.process(full, 0.0, 0)
    assert p.process(no_right, 0.02, 1).state == INVALID


def test_still_image_never_borrows_history():
    full, no_right, *_ = frames()
    p = pipe()
    p.process(full, 0.0, 0)
    r = p.process(no_right, 0.01, 1, static=True)
    assert r.state == INVALID and r.right.state == MISSING


def test_reset_clears_history():
    full, no_right, *_ = frames()
    p = pipe()
    p.process(full, 0.0, 0)
    p.reset('seek')
    assert p.process(no_right, 0.01, 1).state == INVALID


def test_resolution_change_resets():
    full, no_right, lm, _ = frames()
    p = pipe()
    p.process(full, 0.0, 0)
    import cv2
    small = cv2.resize(sy.scene([lm]), (480, 270))
    r = p.process(small, 0.01, 1)
    assert r.state == INVALID


def test_identity_preserved_target_does_not_jump_lane():
    # Lane drifts slowly; a new marking appears further right. Pair must stay.
    p = pipe()
    xs = []
    for i in range(10):
        s = 3.0 * i
        _, lm, rm = sy.two_lane(left_x=170 + s, right_x=470 + s)
        marks = [lm, rm]
        if i >= 5:
            marks.append(sy.Marking(rm.x_bottom + 150, rm.slope * 1.4))
        r = p.process(sy.scene(marks, seed=i), 0.05 * i, i)
        assert r.state == VALID, (i, r.reason)
        xs.append(r.right.x_eval)
    assert max(np.diff(xs)) < 10.0


def test_out_of_lane_is_flagged():
    # Car centre (image centre) is left of the selected lane.
    img, *_ = sy.two_lane(left_x=380, right_x=630, converge=0.3)
    p = LanePipeline(DetectorConfig(lane_reference_x_ratio=0.8), SteeringConfig(), TrackerConfig())
    r = p.process(img, static=True)
    assert r.state == VALID and r.reason == 'OUT_OF_LANE' and r.in_lane is False


# -- gate -----------------------------------------------------------------------

def valid_result(acq_t=100.0, static=False):
    img, *_ = sy.two_lane(left_x=120, right_x=420)
    r = pipe().process(img, acq_t, 0, static=static)
    assert r.state == VALID
    return r


def test_gate_fresh_valid_publishes_angle():
    r = valid_result()
    d = decide(r, 100.05, 0.15, vehicle_topic=False)
    assert d.status == VALID and d.publish_angle and d.steer_rad == r.steering.steer_rad
    assert d.steer_rad < 0  # lane midpoint is left of centre


def test_gate_stale_after_age_limit_and_repeat_does_not_refresh():
    r = valid_result()
    assert decide(r, 100.14, 0.15, False).status == VALID
    for t in (100.16, 100.5, 105.0):
        d = decide(r, t, 0.15, False)
        assert d.status == STALE and d.steer_rad == 0.0 and not d.publish_angle


def test_gate_stamp_age_counts():
    r = valid_result()
    d = decide(r, 100.01, 0.15, False, stamp_age_s=0.3)
    assert d.status == STALE


def test_gate_degraded_and_invalid_publish_neutral():
    full, no_right, *_ = frames()
    p = pipe()
    p.process(full, 0.0, 0)
    deg = p.process(no_right, 0.05, 1)
    d = decide(deg, 0.06, 0.15, False)
    assert d.status == DEGRADED and not d.publish_angle and d.steer_rad == 0.0
    inv = pipe().process(sy.road(), 0.0, 0)
    d = decide(inv, 0.01, 0.15, False)
    assert d.status == INVALID and d.steer_rad == 0.0


def test_gate_static_only_on_debug_topic():
    r = valid_result(static=True)
    assert decide(r, 100.01, 0.15, vehicle_topic=False).publish_angle
    d = decide(r, 100.01, 0.15, vehicle_topic=True)
    assert d.status == INVALID and not d.publish_angle


def test_gate_none_result():
    d = decide(None, 0.0, 0.15, False)
    assert not d.publish_angle and d.steer_rad == 0.0


# -- bounded buffering ---------------------------------------------------------------

def test_latest_slot_never_grows():
    slot = LatestSlot()
    for i in range(1000):
        slot.put(i)
    assert slot.take() == 999 and slot.take() is None and slot.dropped == 999


def test_source_thread_with_slow_consumer_keeps_only_latest():
    slot = LatestSlot()
    st = SourceThread(SyntheticSource(rate_hz=200.0), slot).start()
    time.sleep(0.3)
    st.stop()
    f = slot.take()
    assert f is not None and slot.take() is None
    assert slot.dropped > 10
    assert not f.image.flags.writeable


def test_gate_static_reaches_vehicle_only_in_bench():
    r = valid_result(static=True)
    assert not decide(r, 100.01, 0.15, vehicle_topic=True).publish_angle
    d = decide(r, 100.01, 0.15, vehicle_topic=True, allow_static=True)
    assert d.status == VALID and d.publish_angle and d.static
    # Freshness still applies to bench frames.
    assert decide(r, 100.5, 0.15, vehicle_topic=True, allow_static=True).status == STALE
