"""Per-tick command decision: result validity + freshness -> published value.

Evaluated independently on every command timer tick from the latest result
snapshot, so a stalled worker or camera is noticed from elapsed time alone;
republishing never refreshes a result's age.
"""

import dataclasses
import math
from typing import Optional

from .pipeline import DEGRADED, INVALID, VALID

STALE = 'STALE'
NO_RESULT = 'NO_RESULT'


@dataclasses.dataclass(frozen=True)
class Decision:
    status: str  # VALID / DEGRADED / INVALID / STALE / NO_RESULT, plus STATIC flag below
    reason: str
    steer_rad: float  # value to publish; 0.0 unless a VALID, fresh result allows it
    publish_angle: bool
    age_s: Optional[float]
    static: bool


def decide(result, now: float, max_age_s: float, vehicle_topic: bool,
           stamp_age_s: Optional[float] = None, allow_static: bool = False) -> Decision:
    """Decide the published steering for this tick.

    - Only a VALID result younger than max_age_s may carry its angle.
    - DEGRADED (held boundary) and static-image results never reach a
      vehicle topic; on the debug topic a static result is labelled STATIC.
    - Everything else publishes a neutral 0.0 request with the reason in status.
    """
    if result is None:
        return Decision(NO_RESULT, 'no frame processed yet', 0.0, False, None, False)
    age = now - result.acq_t
    if stamp_age_s is not None and math.isfinite(stamp_age_s):
        age = max(age, stamp_age_s)
    if not math.isfinite(age) or age < -0.05:
        return Decision(STALE, 'clock error', 0.0, False, age, result.static)
    if result.state == INVALID:
        return Decision(INVALID, result.reason, 0.0, False, age, result.static)
    if age > max_age_s:
        return Decision(STALE, f'result age {age * 1000:.0f} ms > {max_age_s * 1000:.0f} ms',
                        0.0, False, age, result.static)
    s = result.steering
    if s is None or not math.isfinite(s.steer_rad) or abs(s.steer_rad) > 1.0:
        return Decision(INVALID, 'nonfinite or out-of-range steering', 0.0, False, age, result.static)
    if result.state == DEGRADED:
        return Decision(DEGRADED, result.reason, 0.0, False, age, result.static)
    if result.state != VALID:
        return Decision(INVALID, f'unexpected state {result.state}', 0.0, False, age, result.static)
    if result.static:
        # allow_static is only set by the explicit launch-time bench mode.
        if vehicle_topic and not allow_static:
            return Decision(INVALID, 'static image is never published to the vehicle topic',
                            0.0, False, age, True)
        return Decision(VALID, 'STATIC preview', float(s.steer_rad), True, age, True)
    return Decision(VALID, result.reason, float(s.steer_rad), True, age, False)
