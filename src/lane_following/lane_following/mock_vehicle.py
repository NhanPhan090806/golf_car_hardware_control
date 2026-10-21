"""Mock vehicle_controller + camera for testing Phase B arming in WSL.

Imitates only the ROS contract the lane node sees: a node named
vehicle_controller with require_function_command, /vehicle/state/system and
/vehicle/state/motion at 50 Hz, and (optionally) a stamped synthetic camera
on /camera/image_raw. It does not model HSH, ODrive, relays or physics.

Refuses to run unless ROS_DOMAIN_ID is set to a non-zero value, so it can
never appear next to the real car on the default domain.

Fault injection at runtime:
  ros2 param set /vehicle_controller mock.motion_allowed false
  ros2 param set /vehicle_controller mock.fault true
  ros2 param set /vehicle_controller mock.publish_state false   # vehicle state goes stale
  ros2 param set /vehicle_controller mock.camera_rate_hz 0.0     # camera stalls
"""

import array
import os
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from vehicle_msgs.msg import VehicleFunctionCommand, VehicleMotionCommand, VehicleMotionState, VehicleSystemState

from .synthetic import moving_scene


class MockVehicle(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        for name, default in (('require_function_command', True), ('command_timeout_ms', 250),
                              ('mock.online', True), ('mock.fault', False), ('mock.motion_allowed', True),
                              ('mock.publish_state', True), ('mock.camera_rate_hz', 20.0),
                              ('mock.steer_rate_rad_s', 0.5)):
            self.declare_parameter(name, default)
        self.motion = None
        self.func = None
        self.steer = 0.0
        self.create_subscription(VehicleMotionCommand, '/vehicle/control/motion_cmd', self._on_motion, 1)
        self.create_subscription(VehicleFunctionCommand, '/vehicle/control/func_cmd', self._on_func, 1)
        self.sys_pub = self.create_publisher(VehicleSystemState, '/vehicle/state/system', 10)
        self.mot_pub = self.create_publisher(VehicleMotionState, '/vehicle/state/motion', 10)
        # Reliable so 690 kB frames are not dropped by small UDP buffers in WSL;
        # vehicle_integration.launch.py sets input.image_reliability:=reliable for mock.
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', QoSProfile(depth=1))
        self.create_timer(0.02, self._tick)
        self._t0 = time.monotonic()
        # The camera runs on its own thread so image encoding/transport
        # cannot delay the 50 Hz vehicle-state timer (and vice versa).
        self._stop = threading.Event()
        threading.Thread(target=self._camera_loop, name='mock_camera', daemon=True).start()
        self.get_logger().warning('MOCK vehicle (no hardware). Do not run next to the real car.')

    def _p(self, n):
        return self.get_parameter(n).value

    def _on_motion(self, m):
        self.motion = (time.monotonic(), m)

    def _on_func(self, m):
        self.func = (time.monotonic(), m)

    def _tick(self):
        now = time.monotonic()
        timeout = self._p('command_timeout_ms') / 1000.0
        motion_fresh = self.motion is not None and now - self.motion[0] <= timeout
        func_fresh = self.func is not None and now - self.func[0] <= timeout
        req_func = self._p('require_function_command')
        f = self.func[1] if self.func else None
        online, fault, allowed = self._p('mock.online'), self._p('mock.fault'), self._p('mock.motion_allowed')
        # Same safe rule shape as vehicle_controller.cpp update().
        safe = (not motion_fresh or (req_func and not func_fresh) or
                (f is not None and (f.emergency_stop or not f.motion_enable)) or
                not allowed or not online or fault)
        if not safe:
            target = max(-1.0, min(1.0, float(self.motion[1].steering_angle_rad)))
            step = self._p('mock.steer_rate_rad_s') * 0.02
            self.steer += max(-step, min(step, target - self.steer))
        stamp = self.get_clock().now().to_msg()
        if self._p('mock.publish_state'):
            s = VehicleSystemState()
            s.header.stamp = stamp
            s.hardware_online, s.hardware_fault, s.hardware_motion_allowed = online, fault, allowed
            s.function_motion_enabled = bool(f.motion_enable) if f else False
            s.emergency_stop = bool(f.emergency_stop) if f else False
            s.command_timed_out = not motion_fresh or (req_func and not func_fresh)
            s.safe_command_active = safe
            s.io_output_mask = (f.io_output_mask & 0x1F) if f else 0
            self.sys_pub.publish(s)
            m = VehicleMotionState()
            m.header.stamp = stamp
            m.steering_angle_rad = float(self.steer)
            m.gear = VehicleMotionCommand.GEAR_NEUTRAL
            m.safe_command_active = safe
            self.mot_pub.publish(m)

    def _camera_loop(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            rate = float(self._p('mock.camera_rate_hz'))
            if rate <= 0.0:
                self._stop.wait(0.05)
                next_t = time.monotonic()
                continue
            img = moving_scene(time.monotonic() - self._t0, amplitude=40.0)
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mock_camera'
            msg.height, msg.width = img.shape[:2]
            msg.encoding, msg.step = 'bgr8', img.shape[1] * 3
            # array.array avoids rclpy's per-element check on uint8[] (~50x faster)
            msg.data = array.array('B', img.tobytes())
            self.img_pub.publish(msg)
            next_t += 1.0 / rate
            self._stop.wait(max(0.0, next_t - time.monotonic()))


def main(args=None):
    if (os.environ.get('ROS_DOMAIN_ID') or '0') == '0':
        print('mock_vehicle refuses ROS_DOMAIN_ID=0/unset. Example: ROS_DOMAIN_ID=42 ros2 run ...',
              file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    node = MockVehicle()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
