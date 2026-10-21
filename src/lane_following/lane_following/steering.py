"""Pixel-error to steering-request mapping (Phase A baseline, no ROS).

Sign convention for this vehicle: negative = left, positive = right.
error_px = x_lane - image_center_x, so a lane midpoint right of the image
center yields a positive (right) steering request.
"""

import dataclasses
import math

from .config import SteeringConfig

# Absolute bound on any published value, independent of configuration.
HARD_LIMIT_RAD = 1.0


@dataclasses.dataclass(frozen=True)
class SteeringOutput:
    error_px: float
    alpha_effective: float
    steer_raw_rad: float
    steer_rad: float
    clipped: bool


class PixelGainController:
    """steer = clip(alpha * error_px, +-steer_limit). Stateless by design."""

    name = 'pixel_gain'

    def __init__(self, config: SteeringConfig):
        self.config = config

    def effective_alpha(self, processing_width: int) -> float:
        if processing_width <= 0:
            raise ValueError('processing_width must be positive')
        return self.config.alpha_rad_per_px * self.config.alpha_reference_width / processing_width

    def compute(self, error_px: float, processing_width: int) -> SteeringOutput:
        if not math.isfinite(error_px):
            raise ValueError('error_px is not finite')
        alpha = self.effective_alpha(processing_width)
        raw = alpha * error_px
        limit = min(self.config.steer_limit_rad, HARD_LIMIT_RAD)
        steer = max(-limit, min(limit, raw))
        return SteeringOutput(error_px, alpha, raw, steer, steer != raw)
