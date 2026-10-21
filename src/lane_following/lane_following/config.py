"""Validated, immutable configuration for the ROS-free lane pipeline.

All geometric settings are fractions of the processing image (width W,
height H) so they keep their meaning when processing_width changes. Invalid
values raise ConfigError instead of being silently corrected.
"""

import dataclasses
import math
from typing import Any, Dict, Mapping, Tuple

import yaml


class ConfigError(ValueError):
    pass


def _finite(name, value, lo=-math.inf, hi=math.inf, lo_open=False):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ConfigError(f'{name} must be a finite number, got {value!r}')
    if value < lo or value > hi or (lo_open and value == lo):
        raise ConfigError(f'{name}={value} outside {"(" if lo_open else "["}{lo}, {hi}]')


def _ratio(name, value, lo_open=False):
    _finite(name, value, 0.0, 1.0, lo_open)


@dataclasses.dataclass(frozen=True)
class DetectorConfig:
    processing_width: int = 640
    # Road polygon as flat [x0, y0, x1, y1, ...] in image fractions.
    roi: Tuple[float, ...] = (0.0, 0.98, 0.0, 0.80, 0.30, 0.58, 0.70, 0.58, 1.0, 0.80, 1.0, 0.98)
    # Paint mask: white top-hat on HSV V (paint brighter than nearby asphalt).
    tophat_width_ratio: float = 0.06
    min_contrast: int = 25
    white_max_saturation: int = 60
    white_min_value: int = 90
    yellow_hue_min: int = 12
    yellow_hue_max: int = 40
    yellow_min_saturation: int = 70
    yellow_min_value: int = 90
    # Vertical close bridges row dropouts but also fills the gap between
    # slanted double stripes; off by default.
    vertical_close_ratio: float = 0.0
    # Row runs and per-row grouping of double stripes.
    min_run_width_px: int = 2
    cluster_gap_factor: float = 2.0
    cluster_max_gap_ratio: float = 0.03
    # Linking runs into pieces and pieces into dash chains.
    link_row_gap: int = 3
    link_tolerance_ratio: float = 0.012
    min_piece_rows: int = 3
    association_residual_ratio: float = 0.012
    max_chain_gap_ratio: float = 0.25
    # Chain acceptance and fitting (fit x = f(y)).
    fit_mode: str = 'auto'  # 'linear' or 'auto' (quadratic fallback)
    min_support_ratio: float = 0.035
    min_span_ratio: float = 0.12
    max_fit_rms_ratio: float = 0.006
    max_abs_slope: float = 4.0
    max_bow_ratio: float = 0.06
    min_inlier_fraction: float = 0.6
    solid_support_fraction: float = 0.8
    # Double-stripe grouping fractions of resolvable rows.
    double_min_fraction: float = 0.5
    single_max_fraction: float = 0.2
    # Support classification around y_eval.
    max_extrapolation_ratio: float = 0.04
    max_interpolation_ratio: float = 0.25
    # Pair selection.
    evaluation_row_ratio: float = 0.75
    min_lane_width_ratio: float = 0.10
    max_lane_width_ratio: float = 0.90
    check_row_offsets: Tuple[float, ...] = (-0.08, 0.08)
    lane_reference_x_ratio: float = 0.5
    ambiguity_margin_ratio: float = 0.03

    def __post_init__(self):
        if not isinstance(self.processing_width, int) or not 160 <= self.processing_width <= 3840:
            raise ConfigError('processing_width must be an integer in [160, 3840]')
        if len(self.roi) < 6 or len(self.roi) % 2:
            raise ConfigError('roi needs at least three (x, y) pairs')
        for i, v in enumerate(self.roi):
            _ratio(f'roi[{i}]', v)
        xs, ys = self.roi[0::2], self.roi[1::2]
        area = 0.5 * abs(sum(xs[i] * ys[i - 1] - xs[i - 1] * ys[i] for i in range(len(xs))))
        if area < 0.01:
            raise ConfigError(f'roi polygon area {area:.4f} is too small (< 1% of the image)')
        if self.fit_mode not in ('linear', 'auto'):
            raise ConfigError("fit_mode must be 'linear' or 'auto'")
        for name in ('tophat_width_ratio', 'vertical_close_ratio', 'cluster_max_gap_ratio',
                     'link_tolerance_ratio', 'association_residual_ratio', 'max_chain_gap_ratio',
                     'min_support_ratio', 'min_span_ratio', 'max_fit_rms_ratio', 'max_bow_ratio',
                     'min_inlier_fraction', 'solid_support_fraction', 'double_min_fraction',
                     'max_extrapolation_ratio', 'max_interpolation_ratio', 'min_lane_width_ratio',
                     'max_lane_width_ratio', 'lane_reference_x_ratio', 'ambiguity_margin_ratio'):
            _ratio(name, getattr(self, name), lo_open=name not in ('vertical_close_ratio',
                                                                    'ambiguity_margin_ratio'))
        _ratio('single_max_fraction', self.single_max_fraction)
        _ratio('evaluation_row_ratio', self.evaluation_row_ratio, lo_open=True)
        for name in ('min_contrast', 'white_max_saturation', 'white_min_value', 'yellow_hue_min',
                     'yellow_hue_max', 'yellow_min_saturation', 'yellow_min_value',
                     'min_run_width_px', 'link_row_gap', 'min_piece_rows'):
            v = getattr(self, name)
            if not isinstance(v, int) or not 0 <= v <= 255:
                raise ConfigError(f'{name} must be an integer in [0, 255]')
        if self.yellow_hue_min >= self.yellow_hue_max or self.yellow_hue_max > 180:
            raise ConfigError('yellow hue range must satisfy min < max <= 180')
        _finite('cluster_gap_factor', self.cluster_gap_factor, 0.0, 20.0, lo_open=True)
        _finite('max_abs_slope', self.max_abs_slope, 0.0, 50.0, lo_open=True)
        if self.single_max_fraction >= self.double_min_fraction:
            raise ConfigError('single_max_fraction must be < double_min_fraction')
        if self.min_lane_width_ratio >= self.max_lane_width_ratio:
            raise ConfigError('min_lane_width_ratio must be < max_lane_width_ratio')
        for v in self.check_row_offsets:
            _finite('check_row_offsets', v, -0.5, 0.5)


@dataclasses.dataclass(frozen=True)
class SteeringConfig:
    alpha_rad_per_px: float = 0.002
    # Width at which alpha was tuned; alpha is rescaled so equal fractional
    # image offsets give equal steering at other processing widths.
    alpha_reference_width: int = 640
    steer_limit_rad: float = 1.0

    def __post_init__(self):
        _finite('alpha_rad_per_px', self.alpha_rad_per_px, 0.0, 1.0, lo_open=True)
        if not isinstance(self.alpha_reference_width, int) or self.alpha_reference_width <= 0:
            raise ConfigError('alpha_reference_width must be a positive integer')
        _finite('steer_limit_rad', self.steer_limit_rad, 0.0, 1.0, lo_open=True)


@dataclasses.dataclass(frozen=True)
class TrackerConfig:
    # Preview-only bounded hold of one boundary (DEGRADED). 0 disables it.
    hold_duration_s: float = 0.10
    track_timeout_s: float = 0.5
    max_jump_ratio: float = 0.08

    def __post_init__(self):
        _finite('hold_duration_s', self.hold_duration_s, 0.0, 5.0)
        _finite('track_timeout_s', self.track_timeout_s, 0.0, 10.0, lo_open=True)
        _ratio('max_jump_ratio', self.max_jump_ratio, lo_open=True)
        if self.hold_duration_s > self.track_timeout_s:
            raise ConfigError('hold_duration_s must not exceed track_timeout_s')


@dataclasses.dataclass(frozen=True)
class RuntimeConfig:
    publish_rate_hz: float = 20.0
    max_result_age_s: float = 0.15
    status_rate_hz: float = 2.0
    debug_rate_hz: float = 5.0

    def __post_init__(self):
        _finite('publish_rate_hz', self.publish_rate_hz, 1.0, 200.0)
        _finite('max_result_age_s', self.max_result_age_s, 0.0, 2.0, lo_open=True)
        _finite('status_rate_hz', self.status_rate_hz, 0.1, 50.0)
        _finite('debug_rate_hz', self.debug_rate_hz, 0.1, 30.0)


SECTIONS = {
    'detector': DetectorConfig,
    'steering': SteeringConfig,
    'tracker': TrackerConfig,
    'runtime': RuntimeConfig,
}


def from_mapping(cls, values: Mapping[str, Any]):
    """Build a config dataclass, rejecting unknown keys and coercing YAML types."""
    fields = {f.name: f for f in dataclasses.fields(cls)}
    unknown = set(values) - set(fields)
    if unknown:
        raise ConfigError(f'unknown {cls.__name__} keys: {sorted(unknown)}')
    kwargs = {}
    for key, value in values.items():
        default = fields[key].default
        if isinstance(default, tuple):
            value = tuple(float(v) for v in value)
        elif isinstance(default, bool):
            pass
        elif isinstance(default, int) and isinstance(value, float) and value.is_integer():
            value = int(value)
        elif isinstance(default, float) and isinstance(value, int) and not isinstance(value, bool):
            value = float(value)
        kwargs[key] = value
    return cls(**kwargs)


def check_consistency(tracker: TrackerConfig, runtime: RuntimeConfig):
    if tracker.hold_duration_s > runtime.max_result_age_s:
        raise ConfigError('tracker.hold_duration_s must not exceed runtime.max_result_age_s')


def load_yaml_sections(path: str, node_name: str = 'lane_following_node') -> Dict[str, Any]:
    """Return {section: config} from a ROS-style parameter YAML file.

    Keys outside the four pipeline sections (input, output, viewer) are
    returned untouched under 'other'.
    """
    with open(path, 'r', encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    params = doc.get(node_name, {}).get('ros__parameters', doc)
    out = {name: from_mapping(cls, params.get(name, {}) or {}) for name, cls in SECTIONS.items()}
    check_consistency(out['tracker'], out['runtime'])
    out['other'] = {k: v for k, v in params.items() if k not in SECTIONS}
    return out
