"""Phase B arming state machine (steering only), no ROS."""

import dataclasses

import pytest

from lane_following.arming import (ARMED, DISARMED, STOP_LATCHED, ArmingConfig, ArmingStateMachine,
                                   Preconditions, VehicleState)
from lane_following.gate import Decision

GOOD_PRE = Preconditions(live_source=True, stamp_checked=True, controller_requires_function_cmd=True,
                         sole_publisher=True)


def vs(t, **kw):
    base = dict(recv_t=t, hardware_online=True, hardware_fault=False, hardware_motion_allowed=True,
                function_motion_enabled=True, emergency_stop=False, command_timed_out=False,
                safe_command_active=False)
    base.update(kw)
    return VehicleState(**base)


def dec(status='VALID', steer=0.2, publish=True, static=False):
    return Decision(status, 'OK', steer, publish, 0.03, static)


def armed(now=0.0):
    sm = ArmingStateMachine(ArmingConfig())
    ok, msg = sm.request_arm(now, GOOD_PRE, vs(now), dec())
    assert ok, msg
    return sm


def test_starts_disarmed_and_publishes_nothing():
    sm = ArmingStateMachine(ArmingConfig())
    out = sm.step(0.0, dec(), vs(0.0), GOOD_PRE)
    assert out.state == DISARMED and not out.motion_enable and not out.publish_motion


@pytest.mark.parametrize('pre_kw,needle', [
    ({'live_source': False}, 'live'),
    ({'stamp_checked': False}, 'stamps'),
    ({'controller_requires_function_cmd': False}, 'require_function_command=false'),
    ({'controller_requires_function_cmd': None}, 'cannot read'),
    ({'sole_publisher': False}, 'another publisher'),
])
def test_arm_refused_on_preconditions(pre_kw, needle):
    sm = ArmingStateMachine(ArmingConfig())
    ok, msg = sm.request_arm(0.0, dataclasses.replace(GOOD_PRE, **pre_kw), vs(0.0), dec())
    assert not ok and needle in msg and sm.state == DISARMED


@pytest.mark.parametrize('vkw,needle', [
    ({'hardware_online': False}, 'offline'), ({'hardware_fault': True}, 'fault'),
    ({'hardware_motion_allowed': False}, 'HSH'), ({'emergency_stop': True}, 'emergency'),
])
def test_arm_refused_on_vehicle_state(vkw, needle):
    sm = ArmingStateMachine(ArmingConfig())
    ok, msg = sm.request_arm(0.0, GOOD_PRE, vs(0.0, **vkw), dec())
    assert not ok and needle in msg


def test_arm_refused_without_vehicle_state_or_with_stale_state():
    sm = ArmingStateMachine(ArmingConfig())
    assert not sm.request_arm(0.0, GOOD_PRE, None, dec())[0]
    assert not sm.request_arm(1.0, GOOD_PRE, vs(0.0), dec())[0]


@pytest.mark.parametrize('d', [None, dec('STALE', publish=False), dec('DEGRADED', publish=False),
                               dec('INVALID', publish=False), dec(static=True)])
def test_arm_refused_without_fresh_valid_lane(d):
    sm = ArmingStateMachine(ArmingConfig())
    assert not sm.request_arm(0.0, GOOD_PRE, vs(0.0), d)[0]


def test_does_not_wait_for_safe_command_inactive_before_arming():
    # The controller reports safe_command_active=true while disarmed; that
    # must not deadlock the arm request.
    sm = ArmingStateMachine(ArmingConfig())
    ok, _ = sm.request_arm(0.0, GOOD_PRE, vs(0.0, safe_command_active=True, function_motion_enabled=False), dec())
    assert ok


def test_armed_output_steering_only_and_clipped():
    sm = armed()
    out = sm.step(0.05, dec(steer=0.9), vs(0.05), GOOD_PRE)
    assert out.state == ARMED and out.motion_enable and out.publish_motion
    assert out.steer_rad == pytest.approx(0.5)  # hardware_steer_limit_rad
    assert sm.step(0.1, dec(steer=-0.9), vs(0.1), GOOD_PRE).steer_rad == pytest.approx(-0.5)


@pytest.mark.parametrize('bad', [
    dict(d=dec('STALE', publish=False)), dict(d=dec('DEGRADED', publish=False)),
    dict(d=dec('INVALID', publish=False)), dict(d=None),
    dict(v={'hardware_fault': True}), dict(v={'hardware_motion_allowed': False}),
    dict(v={'hardware_online': False}), dict(v={'emergency_stop': True}),
    dict(pre={'sole_publisher': False}), dict(pre={'controller_requires_function_cmd': None}),
])
def test_any_loss_of_eligibility_latches(bad):
    sm = armed()
    d = bad.get('d', dec()) if 'd' in bad else dec()
    v = vs(0.05, **bad.get('v', {}))
    pre = dataclasses.replace(GOOD_PRE, **bad.get('pre', {}))
    out = sm.step(0.05, d, v, pre)
    assert out.state == STOP_LATCHED and not out.motion_enable and not out.publish_motion


def test_vehicle_state_timeout_latches():
    sm = armed()
    out = sm.step(0.6, dec(), vs(0.0), GOOD_PRE)
    assert out.state == STOP_LATCHED and 'age' in sm.latch_cause


def test_controller_must_confirm_enable():
    sm = armed()
    # Within the confirm window safe_command_active may still be true.
    assert sm.step(0.3, dec(), vs(0.3, safe_command_active=True), GOOD_PRE).state == ARMED
    out = sm.step(0.6, dec(), vs(0.6, safe_command_active=True), GOOD_PRE)
    assert out.state == STOP_LATCHED and 'confirm' in sm.latch_cause


def test_no_automatic_rearm_and_reset_rules():
    sm = armed()
    sm.step(0.05, dec('INVALID', publish=False), vs(0.05), GOOD_PRE)
    assert sm.state == STOP_LATCHED
    # Lanes come back: still latched.
    assert sm.step(0.1, dec(), vs(0.1), GOOD_PRE).state == STOP_LATCHED
    assert not sm.request_arm(0.1, GOOD_PRE, vs(0.1), dec())[0]
    assert sm.request_disarm()[0] and sm.state == STOP_LATCHED
    # Reset refused while a fault persists, accepted once it clears.
    assert not sm.request_reset(0.2, vs(0.2, hardware_fault=True))[0]
    ok, _ = sm.request_reset(0.3, vs(0.3))
    assert ok and sm.state == DISARMED
    # Reset alone never arms.
    assert sm.step(0.35, dec(), vs(0.35), GOOD_PRE).state == DISARMED
    assert sm.request_arm(0.4, GOOD_PRE, vs(0.4), dec())[0]


def test_reset_allowed_while_hsh_parked():
    sm = armed()
    sm.request_stop()
    assert sm.request_reset(0.1, vs(0.1, hardware_motion_allowed=False))[0]


def test_operator_stop_and_disarm():
    sm = armed()
    assert sm.request_disarm()[0] and sm.state == DISARMED
    sm = armed()
    sm.request_stop('space key')
    assert sm.state == STOP_LATCHED and sm.latch_cause == 'space key'


def test_first_latch_cause_is_kept():
    sm = armed()
    sm.step(0.05, dec('STALE', publish=False), vs(0.05), GOOD_PRE)
    sm.request_stop('operator')
    assert sm.latch_cause.startswith('lane STALE')


@pytest.mark.parametrize('kw', [{'hardware_steer_limit_rad': 1.5}, {'vehicle_state_timeout_s': 0.0},
                                {'arm_confirm_timeout_s': float('nan')}])
def test_invalid_arming_config(kw):
    with pytest.raises(ValueError):
        ArmingConfig(**kw)


# -- bench mode (raised car, saved input) ------------------------------------------

BENCH_PRE = dataclasses.replace(GOOD_PRE, live_source=False, stamp_checked=False, bench=True)


def test_saved_input_refused_without_bench():
    sm = ArmingStateMachine(ArmingConfig())
    pre = dataclasses.replace(GOOD_PRE, live_source=False, stamp_checked=False)
    ok, msg = sm.request_arm(0.0, pre, vs(0.0), dec(static=True))
    assert not ok and 'live' in msg


def test_bench_arms_on_static_image_and_steers():
    sm = ArmingStateMachine(ArmingConfig(hardware_steer_limit_rad=0.2))
    ok, msg = sm.request_arm(0.0, BENCH_PRE, vs(0.0), dec(steer=0.13, static=True))
    assert ok, msg
    out = sm.step(0.05, dec(steer=0.13, static=True), vs(0.05), BENCH_PRE)
    assert out.state == ARMED and out.publish_motion and out.steer_rad == pytest.approx(0.13)
    assert sm.step(0.1, dec(steer=0.9, static=True), vs(0.1), BENCH_PRE).steer_rad == pytest.approx(0.2)


@pytest.mark.parametrize('kw', [{'controller_requires_function_cmd': False}, {'sole_publisher': False}])
def test_bench_keeps_the_other_preconditions(kw):
    sm = ArmingStateMachine(ArmingConfig())
    assert not sm.request_arm(0.0, dataclasses.replace(BENCH_PRE, **kw), vs(0.0), dec(static=True))[0]


def test_bench_still_latches_on_lane_loss_and_hsh():
    sm = ArmingStateMachine(ArmingConfig())
    assert sm.request_arm(0.0, BENCH_PRE, vs(0.0), dec(static=True))[0]
    assert sm.step(0.05, dec('INVALID', publish=False, static=True), vs(0.05), BENCH_PRE).state == STOP_LATCHED
    sm = ArmingStateMachine(ArmingConfig())
    sm.request_arm(0.0, BENCH_PRE, vs(0.0), dec(static=True))
    out = sm.step(0.05, dec(static=True), vs(0.05, hardware_motion_allowed=False), BENCH_PRE)
    assert out.state == STOP_LATCHED
