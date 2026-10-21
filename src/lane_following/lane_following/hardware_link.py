"""Phase B ROS glue around ArmingStateMachine (steering only).

Owns /vehicle/control/func_cmd for the whole run (motion_enable=false unless
ARMED) and publishes /vehicle/control/motion_cmd only while ARMED, always
with speed 0, brake 0, gear NEUTRAL. If this process dies, the controller's
command timeout (require_function_command=true profile) removes motion;
nothing here relies on this process staying alive to stop.
"""

import threading
import time

from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from vehicle_msgs.msg import VehicleFunctionCommand, VehicleMotionCommand, VehicleSystemState, VehicleMotionState

from .arming import ARMED, ArmingConfig, ArmingStateMachine, Preconditions, VehicleState

MOTION_TOPIC = '/vehicle/control/motion_cmd'
FUNC_TOPIC = '/vehicle/control/func_cmd'
LIVE_MODES = ('camera', 'ros')


class HardwareLink:
    def __init__(self, node, qos_reliable, arming_cfg: ArmingConfig, input_mode, stamp_checked,
                 controller_node, io_output_mask, bench=False):
        self.node = node
        self.sm = ArmingStateMachine(arming_cfg)
        self.cfg = arming_cfg
        self.live_source = input_mode in LIVE_MODES
        self.bench = bool(bench)
        self.stamp_checked = stamp_checked
        self.io_output_mask = int(io_output_mask) & 0x1F  # aux relays 1..5 only
        self._lock = threading.Lock()
        self._vs = None
        self._motion_state = None
        self._controller_flag = None
        self._sole_publisher = False
        self._last_out = None
        self.motion_pub = node.create_publisher(VehicleMotionCommand, MOTION_TOPIC, qos_reliable)
        self.func_pub = node.create_publisher(VehicleFunctionCommand, FUNC_TOPIC, qos_reliable)
        node.create_subscription(VehicleSystemState, '/vehicle/state/system', self._on_system, 10)
        node.create_subscription(VehicleMotionState, '/vehicle/state/motion', self._on_motion, 10)
        self._param_client = node.create_client(GetParameters, f'/{controller_node}/get_parameters')
        self._controller_node = controller_node
        for name, cb in (('arm', self._srv_arm), ('disarm', self._srv_disarm),
                         ('stop', self._srv_stop), ('reset', self._srv_reset)):
            node.create_service(Trigger, f'/lane_following/{name}', cb)
        node.create_timer(0.5, self._slow_checks)

    # -- inputs ------------------------------------------------------------------
    def _on_system(self, m):
        vs = VehicleState(time.monotonic(), m.hardware_online, m.hardware_fault, m.hardware_motion_allowed,
                          m.function_motion_enabled, m.emergency_stop, m.command_timed_out,
                          m.safe_command_active)
        with self._lock:
            self._vs = vs

    def _on_motion(self, m):
        with self._lock:
            self._motion_state = (time.monotonic(), float(m.steering_angle_rad), bool(m.safe_command_active))

    def _slow_checks(self):
        # Exactly one publisher (this node) on each real command topic.
        sole = (self.node.count_publishers(MOTION_TOPIC) == 1 and
                self.node.count_publishers(FUNC_TOPIC) == 1)
        with self._lock:
            self._sole_publisher = sole
        if self._param_client.service_is_ready():
            req = GetParameters.Request(names=['require_function_command'])
            self._param_client.call_async(req).add_done_callback(self._on_param)
        else:
            with self._lock:
                self._controller_flag = None

    def _on_param(self, fut):
        try:
            values = fut.result().values
            flag = bool(values[0].bool_value) if values and values[0].type == 1 else None
        except Exception:  # service vanished mid-call
            flag = None
        with self._lock:
            self._controller_flag = flag

    def _pre(self):
        with self._lock:
            return Preconditions(self.live_source, self.stamp_checked, self._controller_flag,
                                 self._sole_publisher, self.bench), self._vs

    # -- services ----------------------------------------------------------------
    def _respond(self, resp, ok_msg):
        ok, msg = ok_msg
        resp.success, resp.message = bool(ok), msg
        self.node.get_logger().info(f'[{"BENCH " if self.bench else ""}{self.sm.state}] {msg}')
        return resp

    def _srv_arm(self, req, resp):
        pre, vs = self._pre()
        return self._respond(resp, self.sm.request_arm(time.monotonic(), pre, vs, self.node.current_decision()))

    def _srv_disarm(self, req, resp):
        return self._respond(resp, self.sm.request_disarm())

    def _srv_stop(self, req, resp):
        return self._respond(resp, self.sm.request_stop('operator stop'))

    def _srv_reset(self, req, resp):
        _, vs = self._pre()
        return self._respond(resp, self.sm.request_reset(time.monotonic(), vs))

    # -- per command tick ----------------------------------------------------------
    def tick(self, decision):
        pre, vs = self._pre()
        prev = self.sm.state
        out = self.sm.step(time.monotonic(), decision, vs, pre)
        if out.state != prev:
            log = self.node.get_logger().warning if out.state != ARMED else self.node.get_logger().info
            log(f'{prev} -> {out.state}: {out.reason}')
        f = VehicleFunctionCommand()
        f.motion_enable = bool(out.motion_enable)
        f.emergency_stop = False  # steering only: never trigger the e-stop brake
        f.io_output_mask = self.io_output_mask
        self.func_pub.publish(f)
        if out.publish_motion:
            m = VehicleMotionCommand()
            m.steering_angle_rad = float(out.steer_rad)
            m.speed_percentage = 0.0
            m.brake_percentage = 0.0
            m.gear = VehicleMotionCommand.GEAR_NEUTRAL
            self.motion_pub.publish(m)
        self._last_out = out
        return out

    def publish_safe_and_close(self):
        """Best effort on shutdown; the controller timeout is the real guard."""
        self.sm.request_disarm()
        f = VehicleFunctionCommand()
        f.motion_enable = False
        f.emergency_stop = False
        f.io_output_mask = self.io_output_mask
        for _ in range(3):
            self.func_pub.publish(f)

    def status(self):
        pre, vs = self._pre()
        with self._lock:
            ms = self._motion_state
        now = time.monotonic()
        out = self._last_out
        d = {
            'hw_state': self.sm.state, 'hw_reason': self.sm.reason, 'hw_bench': str(self.bench),
            'hw_steer_limit_rad': f'{self.cfg.hardware_steer_limit_rad:.3f}',
            'hw_latch_cause': str(self.sm.latch_cause),
            'hw_published_steer_rad': 'n/a' if out is None or not out.publish_motion else f'{out.steer_rad:+.4f}',
            'hw_throttle_brake_gear': '0 / 0 / NEUTRAL (fixed)',
            'hw_arm_blockers': '; '.join(pre.problems()) or 'none',
            'hw_controller_require_function_command': str(pre.controller_requires_function_cmd),
            'hw_sole_publisher': str(pre.sole_publisher),
        }
        if vs is None:
            d['vehicle_state'] = 'unavailable'
        else:
            d.update({'vehicle_state_age_ms': f'{(now - vs.recv_t) * 1000:.0f}',
                      'vehicle_online': str(vs.hardware_online), 'vehicle_fault': str(vs.hardware_fault),
                      'vehicle_motion_allowed': str(vs.hardware_motion_allowed),
                      'vehicle_function_enabled': str(vs.function_motion_enabled),
                      'vehicle_safe_command_active': str(vs.safe_command_active),
                      'vehicle_command_timed_out': str(vs.command_timed_out)})
        d['measured_steering_rad'] = 'unavailable' if ms is None else f'{ms[1]:+.4f} (age {(now - ms[0]) * 1000:.0f} ms)'
        return d

    def feedback_metadata(self):
        """Commanded and measured values with their own ages, for snapshots."""
        pre, vs = self._pre()
        with self._lock:
            ms = self._motion_state
        now = time.monotonic()
        out = self._last_out
        return {
            'hardware_state': self.sm.state,
            'commanded': None if out is None else {
                'steering_angle_rad': out.steer_rad if out.publish_motion else None,
                'speed_percentage': 0.0, 'brake_percentage': 0.0, 'gear': 'NEUTRAL',
                'motion_enable': out.motion_enable},
            'measured_steering_rad': None if ms is None else {'value': ms[1], 'age_s': round(now - ms[0], 3)},
            'system_state': None if vs is None else dict(
                vars(vs), age_s=round(now - vs.recv_t, 3), recv_t=None),
        }
