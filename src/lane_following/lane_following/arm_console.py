"""Operator console for Phase B lane following (steering only).

Keys:  A arm   D disarm   Space/X stop (latch)   R reset latch   Q quit
Quitting or Ctrl-C sends disarm. Only one console should be used at a time.
The physical E-stop and AUTO/MANUAL switch remain the primary safety
controls; this console is not a substitute for them.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_srvs.srv import Trigger

KEYS = {'a': 'arm', 'd': 'disarm', ' ': 'stop', 'x': 'stop', 'r': 'reset'}
SHOW = ('hw_state', 'status', 'hw_published_steer_rad', 'measured_steering_rad', 'vehicle_motion_allowed',
        'vehicle_safe_command_active', 'hw_latch_cause', 'hw_arm_blockers')


class ArmConsole(Node):
    def __init__(self):
        super().__init__('lane_arm_console')
        self.clients_ = {n: self.create_client(Trigger, f'/lane_following/{n}')
                         for n in ('arm', 'disarm', 'stop', 'reset')}
        self.status = {}
        self.status_t = None
        self.create_subscription(DiagnosticArray, '/lane_following/status', self._on_status, 5)
        self.last_reply = 'press H for help'

    def _on_status(self, msg):
        if msg.status:
            self.status = {kv.key: kv.value for kv in msg.status[0].values}
            self.status['message'] = msg.status[0].message
            self.status_t = time.monotonic()

    def call(self, name):
        cli = self.clients_[name]
        if not cli.wait_for_service(timeout_sec=0.5):
            self.last_reply = f'{name}: service unavailable (is lane_following_node running with hardware.enabled?)'
            return
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        res = fut.result()
        self.last_reply = f'{name}: ' + ('no reply' if res is None else
                                         f'{"OK" if res.success else "REFUSED"} {res.message}')


def main(args=None):
    if not sys.stdin.isatty():
        print('lane_arm_console needs an interactive terminal; use the /lane_following/* services instead.')
        return
    rclpy.init(args=args)
    node = ArmConsole()
    old = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    print(__doc__)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                k = sys.stdin.read(1).lower()
                if k == 'q':
                    break
                if k == 'h':
                    print(__doc__)
                elif k in KEYS:
                    node.call(KEYS[k])
                    print(f'\n{node.last_reply}')
            age = 'no status' if node.status_t is None else f'{(time.monotonic() - node.status_t):.1f}s ago'
            line = ' | '.join(f'{k}={node.status.get(k, "?")}' for k in SHOW[:5])
            sys.stdout.write(f'\r[{age}] {line}'[:200].ljust(200))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.call('disarm')
        print(f'\n{node.last_reply}\nconsole closed')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
