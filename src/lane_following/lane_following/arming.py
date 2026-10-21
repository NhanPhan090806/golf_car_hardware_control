"""Phase B arming state machine (pure, no ROS). STEERING ONLY.

States:
  DISARMED      func_cmd motion_enable=false; no motion_cmd is published
  ARMED         func_cmd motion_enable=true; motion_cmd carries the lane
                steering request with throttle 0, brake 0, gear NEUTRAL
  STOP_LATCHED  same outputs as DISARMED; entered automatically on any
                loss of eligibility, cleared only by an explicit reset

Throttle and brake are never commanded: speed/brake are always 0.0 and gear
is always NEUTRAL, so the controller keeps drive relays open and the brake
axis IDLE. A stop therefore means motion_enable=false (controller safe
command: steering IDLE, brake IDLE, throttle 0), never the e-stop brake.

Nothing re-arms automatically. reset() only returns to DISARMED once the
latch cause has cleared; arming is always a separate operator request.
"""

import dataclasses
import math
from typing import Optional

from .gate import VALID

DISARMED = 'DISARMED'
ARMED = 'ARMED'
STOP_LATCHED = 'STOP_LATCHED'


@dataclasses.dataclass(frozen=True)
class ArmingConfig:
    vehicle_state_timeout_s: float = 0.5
    arm_confirm_timeout_s: float = 0.5  # controller must report enabled by then
    hardware_steer_limit_rad: float = 0.5  # extra clip on the published request

    def __post_init__(self):
        for name, hi in (('vehicle_state_timeout_s', 5.0), ('arm_confirm_timeout_s', 5.0),
                         ('hardware_steer_limit_rad', 1.0)):
            v = getattr(self, name)
            if not isinstance(v, (int, float)) or not math.isfinite(v) or not 0.0 < v <= hi:
                raise ValueError(f'{name} must be in (0, {hi}]')


@dataclasses.dataclass(frozen=True)
class VehicleState:
    """Latest /vehicle/state/system, stamped with local receive time."""

    recv_t: float
    hardware_online: bool
    hardware_fault: bool
    hardware_motion_allowed: bool
    function_motion_enabled: bool
    emergency_stop: bool
    command_timed_out: bool
    safe_command_active: bool


@dataclasses.dataclass(frozen=True)
class Preconditions:
    """Facts the ROS adapter checks outside the state machine."""

    live_source: bool  # camera / ROS topic, not file, loop or synthetic
    stamp_checked: bool  # ROS header stamps present and checked
    controller_requires_function_cmd: Optional[bool]  # None = could not query
    sole_publisher: bool  # this node is the only motion/func publisher
    # Launch-time bench mode: saved image/video/synthetic input may drive the
    # (raised, secured) car's steering. Every other check still applies.
    bench: bool = False

    def problems(self):
        out = []
        if not self.live_source and not self.bench:
            out.append('input is not a live camera/ROS stream')
        if not self.stamp_checked and not self.bench:
            out.append('image header stamps are not being checked')
        if self.controller_requires_function_cmd is None:
            out.append('cannot read vehicle_controller require_function_command')
        elif not self.controller_requires_function_cmd:
            out.append('vehicle_controller has require_function_command=false; use the lane profile')
        if not self.sole_publisher:
            out.append('another publisher exists on /vehicle/control/motion_cmd or func_cmd')
        return out


@dataclasses.dataclass(frozen=True)
class Output:
    state: str
    reason: str
    motion_enable: bool
    publish_motion: bool
    steer_rad: float  # only meaningful when publish_motion


def vehicle_problems(vs: Optional[VehicleState], now: float, timeout_s: float):
    if vs is None:
        return ['no /vehicle/state/system received']
    out = []
    age = now - vs.recv_t
    if not math.isfinite(age) or age > timeout_s or age < -0.05:
        out.append(f'vehicle state age {age * 1000:.0f} ms > {timeout_s * 1000:.0f} ms')
    if not vs.hardware_online:
        out.append('hardware offline')
    if vs.hardware_fault:
        out.append('hardware fault')
    if not vs.hardware_motion_allowed:
        out.append('HSH does not allow motion (AUTO switch / FSM not RUNNING)')
    if vs.emergency_stop:
        out.append('emergency stop active')
    return out


class ArmingStateMachine:
    def __init__(self, config: ArmingConfig):
        self.config = config
        self.state = DISARMED
        self.reason = 'startup'
        self.latch_cause = None
        self.armed_t = None
        self.last_problems = []

    # -- operator requests (return (accepted, message)) ----------------------
    def request_arm(self, now, pre: Preconditions, vs: Optional[VehicleState], decision):
        if self.state == STOP_LATCHED:
            return False, f'stop latched ({self.latch_cause}); reset first'
        if self.state == ARMED:
            return True, 'already armed'
        problems = pre.problems() + vehicle_problems(vs, now, self.config.vehicle_state_timeout_s)
        if decision is None or decision.status != VALID or not decision.publish_angle or \
                (decision.static and not pre.bench):
            problems.append('lane result is not fresh and VALID'
                            if decision is None else f'lane result {decision.status}: {decision.reason}')
        self.last_problems = problems
        if problems:
            return False, 'refused: ' + '; '.join(problems)
        self.state, self.reason, self.armed_t = ARMED, 'armed by operator', now
        return True, 'armed (steering only, throttle/brake 0, gear NEUTRAL)'

    def request_disarm(self):
        if self.state == STOP_LATCHED:
            return True, f'stop remains latched ({self.latch_cause}); use reset'
        self.state, self.reason, self.armed_t = DISARMED, 'disarmed by operator', None
        return True, 'disarmed'

    def request_stop(self, why='operator stop'):
        self._latch(why)
        return True, f'stop latched: {why}'

    def request_reset(self, now, vs: Optional[VehicleState]):
        if self.state != STOP_LATCHED:
            return True, f'nothing to reset ({self.state})'
        problems = vehicle_problems(vs, now, self.config.vehicle_state_timeout_s)
        # Offline/HSH-not-allowed may be the normal parked state; only refuse
        # while a fault or e-stop is still present.
        blocking = [p for p in problems if 'fault' in p or 'emergency' in p or 'age' in p or 'no /' in p]
        if blocking:
            return False, 'cause not cleared: ' + '; '.join(blocking)
        self.state, self.reason, self.latch_cause, self.armed_t = DISARMED, 'reset by operator', None, None
        return True, 'reset to DISARMED (arm again to continue)'

    def _latch(self, why):
        if self.state != STOP_LATCHED:
            self.latch_cause = why
        self.state, self.reason, self.armed_t = STOP_LATCHED, self.latch_cause, None

    # -- per tick --------------------------------------------------------------
    def step(self, now, decision, vs: Optional[VehicleState], pre: Preconditions) -> Output:
        if self.state == ARMED:
            cause = self._armed_violation(now, decision, vs, pre)
            if cause is not None:
                self._latch(cause)
        if self.state != ARMED:
            return Output(self.state, self.reason, False, False, 0.0)
        lim = self.config.hardware_steer_limit_rad
        steer = max(-lim, min(lim, float(decision.steer_rad)))
        return Output(ARMED, self.reason, True, True, steer)

    def _armed_violation(self, now, decision, vs, pre):
        if decision is None or decision.status != VALID or not decision.publish_angle:
            return f'lane {"NO_RESULT" if decision is None else decision.status}: ' \
                   f'{"" if decision is None else decision.reason}'
        if (decision.static and not pre.bench) or not math.isfinite(decision.steer_rad):
            return 'ineligible lane result'
        p = pre.problems()
        if p:
            return p[0]
        p = vehicle_problems(vs, now, self.config.vehicle_state_timeout_s)
        if p:
            return p[0]
        # Software ACK is not enough: the controller must report it actually
        # left its safe command within the confirm window.
        if now - self.armed_t > self.config.arm_confirm_timeout_s:
            if not vs.function_motion_enabled or vs.safe_command_active:
                return 'controller did not confirm motion enable (safe_command_active)'
        if vs.command_timed_out and now - self.armed_t > self.config.arm_confirm_timeout_s:
            return 'controller reports command timeout'
        return None
