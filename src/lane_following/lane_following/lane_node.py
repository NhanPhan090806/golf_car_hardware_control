"""ROS 2 adapter: input -> worker -> latest result -> 20 Hz preview command.

Threads:
  source thread (image/video/camera/synthetic) or ROS image callback
      -> frame LatestSlot (one frame, overwritten)
  worker thread: detection + steering -> result snapshot (lock-protected)
  debug thread (only while someone subscribes): render + JPEG at debug_rate_hz
  ROS timers: command (gate.decide on every tick), status diagnostics

Preview (default): publishes /lane_following/debug/motion_cmd with zero
throttle, zero brake and neutral gear, and never touches the vehicle topics.

Hardware mode (hardware.enabled:=true, Phase B, STEERING ONLY): additionally
runs HardwareLink, which owns /vehicle/control/func_cmd and publishes
/vehicle/control/motion_cmd only while ARMED. See hardware_link.py/arming.py.
"""

import math
import os
import threading
import time

import cv2
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from vehicle_msgs.msg import VehicleMotionCommand

import array
import json

from . import config as cfg
from .arming import ArmingConfig
from .gate import DEGRADED, NO_RESULT, STALE, VALID, decide
from .input_sources import Frame, LatestSlot, SourceThread, make_source
from .overlay import render
from .pipeline import LanePipeline
from .snapshot import FrameCache, SnapshotConfig, SnapshotWriter
from .ros_image import image_msg_to_bgr

DEBUG_TOPIC = '/lane_following/debug/motion_cmd'
VEHICLE_TOPIC = '/vehicle/control/motion_cmd'

NODE_PARAMS = {
    'input.mode': 'synthetic',  # image | video | camera | synthetic | ros
    'input.path': '',
    'input.rate_hz': 10.0,
    'input.loop': True,
    'input.topic': '/camera/image_raw',
    # Also count ROS header stamp age (needs synchronized clocks) toward the
    # freshness limit, so a late-delivered old image is still stale.
    'input.check_stamp_age': False,
    # best_effort matches most camera drivers; reliable avoids losing large
    # frames between processes when socket buffers are small (see README).
    'input.image_reliability': 'best_effort',
    'output.motion_topic': DEBUG_TOPIC,
    'output.allow_vehicle_topic': False,
    'debug.publish_image': True,
    'debug.jpeg_quality': 80,
    # Phase B hardware integration (steering only). Off by default.
    'hardware.enabled': False,
    'hardware.controller_node': 'vehicle_controller',
    'hardware.vehicle_state_timeout_s': 0.5,
    'hardware.arm_confirm_timeout_s': 0.5,
    'hardware.steer_limit_rad': 0.5,
    'hardware.io_output_mask': 0,  # aux relays 1..5 sent while this node owns func_cmd
    # Bench test on a raised car: allow image/video/synthetic input to arm.
    # Launch-time only (all hardware.* parameters need a restart).
    'hardware.bench': False,
    'hardware.bench_steer_limit_rad': 0.2,
    # Snapshot bundles, written only on request (viewer S/M keys or topic).
    'snapshot.enabled': True,
    'snapshot.directory': '~/lane_snapshots',
    'snapshot.queue_capacity': 4,
    'snapshot.max_bundles': 200,
    'snapshot.max_session_mb': 500.0,
    'snapshot.min_free_mb': 1024.0,
    'snapshot.frame_cache': 8,
}


def _qos(reliable, depth=1):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=depth)


class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__('lane_following_node')
        self.hw = None
        self.bench = False
        self._declare_all()
        self._cfg_lock = threading.Lock()
        self._configs = self._read_configs()
        self._pipeline_dirty = False
        self.add_on_set_parameters_callback(self._on_params)

        p = self._p
        self.mode = p('input.mode')
        self.motion_topic = p('output.motion_topic')
        self.vehicle_topic = self.motion_topic == VEHICLE_TOPIC
        self.hardware = bool(p('hardware.enabled'))
        if self.hardware and self.vehicle_topic:
            raise RuntimeError('hardware.enabled owns /vehicle/control/motion_cmd itself; '
                               'leave output.motion_topic on the debug topic')
        self.bench = self.hardware and bool(p('hardware.bench'))
        if self.bench:
            self.get_logger().warning(
                f'BENCH MODE: input.mode={self.mode} may drive the real steering when armed. '
                f'Car must be raised and secured; steering capped at '
                f'{min(p("hardware.steer_limit_rad"), p("hardware.bench_steer_limit_rad"))} rad.')
        elif self.hardware and self.mode not in ('camera', 'ros'):
            self.get_logger().warning(
                f'hardware.enabled with input.mode={self.mode}: arming will be refused (not a live source)')
        if self.vehicle_topic and not self._vehicle_topic_allowed():
            raise RuntimeError(
                f'refusing to publish {VEHICLE_TOPIC}: set output.allow_vehicle_topic:=true and use an '
                'isolated ROS_DOMAIN_ID (not 0) with no vehicle stack attached')

        self.frame_slot = LatestSlot()
        self._result_lock = threading.Lock()
        self._result = None
        self._stop = threading.Event()
        self._worker_stats = {'frames': 0, 'proc_ms': 0.0, 'rate_hz': 0.0, 'errors': 0, 'last_error': ''}
        self._last_stamp_age = None
        self._last_decision = None
        self._tick_times = []
        self._ros_frame_id = 0

        self.motion_pub = self.create_publisher(VehicleMotionCommand, self.motion_topic, _qos(True))
        self.status_pub = self.create_publisher(DiagnosticArray, '/lane_following/status', _qos(True, 5))
        self.debug_pub = None
        if p('debug.publish_image'):
            self.debug_pub = self.create_publisher(
                CompressedImage, '/lane_following/debug/image/compressed', _qos(False))

        if self.hardware:
            from .hardware_link import HardwareLink
            self.hw = HardwareLink(
                self, _qos(True), ArmingConfig(p('hardware.vehicle_state_timeout_s'),
                                               p('hardware.arm_confirm_timeout_s'),
                                               min(p('hardware.steer_limit_rad'),
                                                   p('hardware.bench_steer_limit_rad')) if self.bench
                                               else p('hardware.steer_limit_rad')),
                self.mode, self.mode == 'camera' or (self.mode == 'ros' and bool(p('input.check_stamp_age'))),
                p('hardware.controller_node'), p('hardware.io_output_mask'), bench=self.bench)

        self.frame_cache = None
        self.snapshots = None
        if p('snapshot.enabled'):
            sc = SnapshotConfig(p('snapshot.directory'), p('snapshot.queue_capacity'), p('snapshot.max_bundles'),
                                p('snapshot.max_session_mb'), p('snapshot.min_free_mb'), p('snapshot.frame_cache'))
            self.frame_cache = FrameCache(sc.frame_cache)
            self.snapshots = SnapshotWriter(sc, lambda r: render(r, self._mode_label()))
            self.snapshot_result_pub = self.create_publisher(String, '/lane_following/snapshot/result', 10)
            self.create_subscription(String, '/lane_following/snapshot/request', self._on_snapshot_request, 10)

        self.source_thread = None
        if self.mode == 'ros':
            rel = p('input.image_reliability')
            if rel not in ('best_effort', 'reliable'):
                raise ValueError("input.image_reliability must be 'best_effort' or 'reliable'")
            self.create_subscription(Image, p('input.topic'), self._on_image, _qos(rel == 'reliable'))
        else:
            src = make_source(self.mode, p('input.path'), p('input.rate_hz'), p('input.loop'))
            self.source_thread = SourceThread(src, self.frame_slot).start()

        self.worker = threading.Thread(target=self._work, name='lane_worker', daemon=True)
        self.worker.start()
        self.debug_thread = None
        if self.debug_pub is not None:
            self.debug_thread = threading.Thread(target=self._debug_loop, name='lane_debug', daemon=True)
            self.debug_thread.start()

        rt = self._configs['runtime']
        self.create_timer(1.0 / rt.publish_rate_hz, self._on_command_tick)
        self.create_timer(1.0 / rt.status_rate_hz, self._on_status_tick)
        det = self._configs['detector']
        self.get_logger().info(
            f'{"HARDWARE (steering only, DISARMED)" if self.hw else "PREVIEW"} mode, input={self.mode}, output={self.motion_topic}, '
            f'alpha={self._configs["steering"].alpha_rad_per_px} rad/px @ {det.processing_width}px, '
            f'eval row {det.evaluation_row_ratio}, max age {rt.max_result_age_s * 1000:.0f} ms')

    # -- parameters -----------------------------------------------------------
    def _p(self, name):
        return self.get_parameter(name).value

    def _declare_all(self):
        for name, default in NODE_PARAMS.items():
            self.declare_parameter(name, default)
        for section, cls in cfg.SECTIONS.items():
            for f in cfg.dataclasses.fields(cls):
                default = f.default
                if isinstance(default, tuple):
                    default = [float(v) for v in default]
                self.declare_parameter(f'{section}.{f.name}', default)

    def _section_values(self, section, overrides=None):
        cls = cfg.SECTIONS[section]
        vals = {}
        for f in cfg.dataclasses.fields(cls):
            key = f'{section}.{f.name}'
            vals[f.name] = overrides[key] if overrides and key in overrides else self._p(key)
        return vals

    def _read_configs(self, overrides=None):
        out = {s: cfg.from_mapping(c, self._section_values(s, overrides)) for s, c in cfg.SECTIONS.items()}
        cfg.check_consistency(out['tracker'], out['runtime'])
        return out

    def _on_params(self, params):
        overrides = {p.name: p.value for p in params}
        if self.hw is not None and self.hw.sm.state == 'ARMED':
            return SetParametersResult(successful=False, reason='disarm before changing parameters')
        if any(k in NODE_PARAMS for k in overrides) or any(k.startswith('runtime.') for k in overrides):
            return SetParametersResult(successful=False,
                                       reason='input/output/debug/runtime parameters need a restart')
        try:
            new = self._read_configs(overrides)
        except (cfg.ConfigError, TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        with self._cfg_lock:
            self._configs = new
            self._pipeline_dirty = True
        self.get_logger().info(f'parameters updated: {sorted(overrides)}; tracking reset')
        return SetParametersResult(successful=True)

    def _vehicle_topic_allowed(self):
        domain = os.environ.get('ROS_DOMAIN_ID', '0') or '0'
        return bool(self._p('output.allow_vehicle_topic')) and domain != '0'

    # -- input ----------------------------------------------------------------
    def _on_image(self, msg):
        # Only conversion and a slot write here; detection runs on the worker.
        now = time.monotonic()
        try:
            img = image_msg_to_bgr(msg)
        except ValueError as exc:
            self._worker_stats['errors'] += 1
            self._worker_stats['last_error'] = str(exc)
            return
        stamp = msg.header.stamp
        stamp_age = None
        if stamp.sec or stamp.nanosec:
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            stamp_age = ros_now - (stamp.sec + stamp.nanosec * 1e-9)
        img.setflags(write=False)
        self._ros_frame_id += 1
        self.frame_slot.put(Frame(img, now, self._ros_frame_id,
                                  f'ros:{msg.header.frame_id or "camera"}', False, stamp_age))

    # -- worker ---------------------------------------------------------------
    def _work(self):
        pipeline, epoch = None, None
        last_t = None
        while not self._stop.is_set():
            frame = self.frame_slot.take()
            if frame is None:
                self._stop.wait(0.002)
                continue
            with self._cfg_lock:
                if pipeline is None or self._pipeline_dirty:
                    c = self._configs
                    pipeline = LanePipeline(c['detector'], c['steering'], c['tracker'])
                    self._pipeline_dirty = False
            if epoch is not None and frame.epoch != epoch:
                pipeline.reset('source restart/seek')
            epoch = frame.epoch
            try:
                res = pipeline.process(frame.image, frame.acq_t, frame.frame_id, frame.source, frame.static)
            except Exception as exc:  # keep running; the gate will go STALE
                self._worker_stats['errors'] += 1
                self._worker_stats['last_error'] = f'{type(exc).__name__}: {exc}'
                continue
            if self.frame_cache is not None:
                self.frame_cache.put(res.frame_id, frame.image, res)
            with self._result_lock:
                self._result = res
                self._last_stamp_age = frame.stamp_age_s
            st = self._worker_stats
            st['frames'] += 1
            st['proc_ms'] = res.proc_ms
            if last_t is not None and res.proc_t > last_t:
                inst = 1.0 / (res.proc_t - last_t)
                st['rate_hz'] = inst if st['rate_hz'] == 0.0 else 0.9 * st['rate_hz'] + 0.1 * inst
            last_t = res.proc_t

    def _latest(self):
        with self._result_lock:
            return self._result, self._last_stamp_age

    # -- command tick ---------------------------------------------------------
    def _on_command_tick(self):
        now = time.monotonic()
        res, stamp_age = self._latest()
        effective_stamp_age = None
        if self._p('input.check_stamp_age') and stamp_age is not None and res is not None:
            # stamp age at reception, plus time spent since reception
            effective_stamp_age = stamp_age + (now - res.acq_t)
        d = decide(res, now, self._configs['runtime'].max_result_age_s,
                   self.vehicle_topic or self.hw is not None, effective_stamp_age,
                   allow_static=self.bench)
        msg = VehicleMotionCommand()
        msg.steering_angle_rad = float(d.steer_rad) if d.publish_angle and math.isfinite(d.steer_rad) else 0.0
        # Preview payload: explicit zeros, not a physical stop procedure.
        msg.speed_percentage = 0.0
        msg.brake_percentage = 0.0
        msg.gear = VehicleMotionCommand.GEAR_NEUTRAL
        self.motion_pub.publish(msg)
        self._last_decision = d
        if self.hw is not None:
            self.hw.tick(d)
        self._tick_times.append(now)
        if len(self._tick_times) > 50:
            del self._tick_times[:-50]

    # -- status ---------------------------------------------------------------
    def _on_status_tick(self):
        d = self._last_decision
        res, _ = self._latest()
        st = DiagnosticStatus()
        st.name = 'lane_following'
        st.hardware_id = self.mode
        status = d.status if d is not None else NO_RESULT
        st.level = DiagnosticStatus.OK if status == VALID else (
            DiagnosticStatus.WARN if status in (DEGRADED, NO_RESULT) else DiagnosticStatus.ERROR)
        st.message = f'{self._mode_label()} {status}{" STATIC" if d is not None and d.static else ""}: ' \
                     f'{d.reason if d else "starting"}'
        t = self._tick_times
        cmd_rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0
        kv = {
            'mode': self._mode_label(), 'status': status, 'output_topic': self.motion_topic,
            'result_age_ms': 'n/a' if d is None or d.age_s is None else f'{d.age_s * 1000:.1f}',
            'published_steer_rad': 'n/a' if d is None else f'{d.steer_rad:+.4f}',
            'command_rate_hz': f'{cmd_rate:.1f}',
            'processing_rate_hz': f'{self._worker_stats["rate_hz"]:.1f}',
            'processing_ms': f'{self._worker_stats["proc_ms"]:.1f}',
            'frames_processed': str(self._worker_stats['frames']),
            'frames_dropped': str(self.frame_slot.dropped),
            'worker_errors': str(self._worker_stats['errors']),
            'last_error': self._worker_stats['last_error'],
            'vehicle_feedback': 'unavailable (preview mode)',
        }
        if self.hw is not None:
            kv.pop('vehicle_feedback')
            kv.update(self.hw.status())
            if self.hw.sm.state != 'ARMED':
                st.level = max(st.level, DiagnosticStatus.WARN)
        if self.snapshots is not None:
            kv['snapshot_last'] = self.snapshots.last_message
            kv['snapshot_dir'] = self.snapshots.session_dir
        if self.source_thread is not None and self.source_thread.source.error:
            kv['source_error'] = self.source_thread.source.error
        if res is not None:
            s = res.steering
            kv.update({
                'result_state': res.state, 'result_reason': res.reason, 'frame_id': str(res.frame_id),
                'source': res.source,
                'error_px': 'n/a' if s is None else f'{s.error_px:+.2f}',
                'alpha_rad_per_px': f'{self._configs["steering"].alpha_rad_per_px}',
                'alpha_effective': 'n/a' if s is None else f'{s.alpha_effective:.6f}',
                'steer_raw_rad': 'n/a' if s is None else f'{s.steer_raw_rad:+.4f}',
                'steer_target_rad': 'n/a' if s is None else f'{s.steer_rad:+.4f}',
                'clipped': 'n/a' if s is None else str(s.clipped),
                'left_state': res.left.state, 'right_state': res.right.state,
                'lane_width_px': 'n/a' if res.lane_width_px is None else f'{res.lane_width_px:.1f}',
                'quality': f'{res.quality:.3f}',
                'processing_size': f'{res.width}x{res.height}',
                'evaluation_row_ratio': f'{self._configs["detector"].evaluation_row_ratio}',
            })
        st.values = [KeyValue(key=k, value=str(v)) for k, v in kv.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.status_pub.publish(arr)

    # -- debug image ----------------------------------------------------------
    def _debug_loop(self):
        period = 1.0 / self._configs['runtime'].debug_rate_hz
        quality = int(self._p('debug.jpeg_quality'))
        last_id = None
        while not self._stop.wait(period):
            if self.debug_pub.get_subscription_count() == 0:
                continue  # nobody is watching: skip render and JPEG encode
            res, _ = self._latest()
            d = self._last_decision
            if res is None or res.detection is None or (res.frame_id, res.acq_t) == last_id:
                continue
            last_id = (res.frame_id, res.acq_t)
            status = 'STALE' if d is not None and d.status == STALE else res.state
            age_ms = None if d is None or d.age_s is None else d.age_s * 1000
            img = render(res, self._mode_label(), status, age_ms, self._worker_stats['rate_hz'])
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            # frame_id carries the result frame id so viewers can request a
            # snapshot of exactly the frame they are displaying.
            msg.header.frame_id = f'{res.frame_id}|{res.source}'
            msg.format = 'jpeg'
            msg.data = array.array('B', buf.tobytes())
            self.debug_pub.publish(msg)

    def current_decision(self):
        return self._last_decision

    def _mode_label(self):
        if self.hw is None:
            return 'PREVIEW'
        return ('BENCH ' if self.bench else '') + self.hw.sm.state

    def _on_snapshot_request(self, msg):
        # Payload JSON: {"frame_id": int, "review": bool}. Encoding/writing
        # happens on the snapshot thread, never here.
        try:
            req = json.loads(msg.data or '{}')
            fid = int(req['frame_id'])
            review = bool(req.get('review', False))
        except (ValueError, KeyError, TypeError):
            self._snapshot_reply(False, 'bad request; expected {"frame_id": N, "review": false}', None)
            return
        item = self.frame_cache.get(fid)
        if item is None:
            self._snapshot_reply(False, f'frame {fid} no longer cached; press again', fid)
            return
        image, res = item
        extra = {'input_mode': self.mode, 'config': {s: cfg.dataclasses.asdict(c)
                                                     for s, c in self._configs.items()}}
        extra['vehicle'] = self.hw.feedback_metadata() if self.hw is not None else \
            {'hardware_state': 'PREVIEW', 'commanded': None, 'measured_steering_rad': None, 'system_state': None}
        extra['local_receive_t'] = res.acq_t
        extra['local_process_t'] = res.proc_t
        ok, text = self.snapshots.request(image, res, extra, review)
        self._snapshot_reply(ok, text, fid)

    def _snapshot_reply(self, ok, text, fid):
        out = String()
        out.data = json.dumps({'ok': ok, 'message': text, 'frame_id': fid,
                               'directory': self.snapshots.session_dir})
        self.snapshot_result_pub.publish(out)

    def shutdown(self):
        if self.hw is not None:
            self.hw.publish_safe_and_close()
        if self.snapshots is not None:
            self.snapshots.stop()
        self._stop.set()
        if self.source_thread is not None:
            self.source_thread.stop()
        self.worker.join(1.0)
        if self.debug_thread is not None:
            self.debug_thread.join(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneFollowingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
