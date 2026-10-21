import math

import pytest

from lane_following import config as cfg
from lane_following.steering import PixelGainController


def ctl(**kw):
    return PixelGainController(cfg.SteeringConfig(**kw))


def test_centered_zero():
    out = ctl().compute(0.0, 640)
    assert out.steer_rad == 0.0 and not out.clipped


@pytest.mark.parametrize('err,expected', [(-50.0, -0.1), (50.0, 0.1)])
def test_sign_convention_negative_left_positive_right(err, expected):
    assert ctl().compute(err, 640).steer_rad == pytest.approx(expected)


def test_clip_at_limit():
    out = ctl().compute(600.0, 640)
    assert out.steer_raw_rad == pytest.approx(1.2)
    assert out.steer_rad == 1.0 and out.clipped
    assert ctl().compute(-600.0, 640).steer_rad == -1.0


def test_custom_limit_never_exceeds_hard_limit():
    assert ctl(steer_limit_rad=0.3).compute(1000.0, 640).steer_rad == pytest.approx(0.3)


def test_resolution_rescaling_preserves_fractional_offset():
    # 50 px at 640 wide and 100 px at 1280 wide are the same image fraction.
    c = ctl()
    assert c.compute(50.0, 640).steer_rad == pytest.approx(c.compute(100.0, 1280).steer_rad)
    assert c.effective_alpha(1280) == pytest.approx(0.001)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_error_rejected(bad):
    with pytest.raises(ValueError):
        ctl().compute(bad, 640)


@pytest.mark.parametrize('kw', [
    {'alpha_rad_per_px': -0.002}, {'alpha_rad_per_px': 0.0}, {'alpha_rad_per_px': math.nan},
    {'steer_limit_rad': 1.5}, {'steer_limit_rad': 0.0}, {'alpha_reference_width': 0},
])
def test_invalid_steering_config(kw):
    with pytest.raises(cfg.ConfigError):
        cfg.SteeringConfig(**kw)


@pytest.mark.parametrize('kw', [
    {'roi': (0.0, 0.0, 1.0, 0.0)},  # too few points
    {'roi': (0.0, 0.5, 0.5, 0.5, 1.0, 0.5)},  # zero area
    {'roi': (0.0, 1.2, 0.0, 0.8, 1.0, 0.8)},  # outside image
    {'evaluation_row_ratio': 0.0}, {'evaluation_row_ratio': 1.1},
    {'min_lane_width_ratio': 0.5, 'max_lane_width_ratio': 0.4},
    {'fit_mode': 'spline'}, {'processing_width': 100},
])
def test_invalid_detector_config(kw):
    with pytest.raises(cfg.ConfigError):
        cfg.DetectorConfig(**kw)


def test_hold_cannot_exceed_result_age():
    with pytest.raises(cfg.ConfigError):
        cfg.check_consistency(cfg.TrackerConfig(hold_duration_s=0.3, track_timeout_s=0.5),
                              cfg.RuntimeConfig(max_result_age_s=0.15))


def test_from_mapping_rejects_unknown_and_coerces():
    with pytest.raises(cfg.ConfigError):
        cfg.from_mapping(cfg.SteeringConfig, {'alpha': 0.1})
    s = cfg.from_mapping(cfg.SteeringConfig, {'steer_limit_rad': 1, 'alpha_reference_width': 640.0})
    assert isinstance(s.steer_limit_rad, float) and isinstance(s.alpha_reference_width, int)


def test_shipped_yaml_loads(tmp_path):
    import os
    here = os.path.dirname(__file__)
    c = cfg.load_yaml_sections(os.path.join(here, '..', 'config', 'lane_following.yaml'))
    assert c['detector'] == cfg.DetectorConfig()
    assert c['steering'] == cfg.SteeringConfig()
    assert c['other']['output']['motion_topic'] == '/lane_following/debug/motion_cmd'
