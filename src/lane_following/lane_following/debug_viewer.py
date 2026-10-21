"""Optional viewer process: OpenCV window and/or localhost browser page.

Subscribes to the node's compressed overlay and status topics. It never
publishes vehicle commands and has no driving, arming or stop keys; closing it
does not affect the lane node. Snapshot keys (S save, M mark for review,
H help, Esc close) only ask the node to save the displayed frame; files are
written on the host running lane_following_node. Only the latest JPEG is kept, so slow browser clients
just miss frames.
"""

import argparse
import json
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

PAGE = b"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>lane_following preview</title>
<style>body{font-family:sans-serif;background:#111;color:#eee;margin:1em}
img{max-width:100%;border:1px solid #444}pre{background:#222;padding:.5em}</style></head>
<body><h1>lane_following preview</h1>
<p>This page has no vehicle controls. Snapshots save on the lane node host.</p>
<p><button id="bs" type="button">Save snapshot (S)</button>
<button id="bm" type="button">Mark for review (M)</button>
<button id="bh" type="button" aria-expanded="false" aria-controls="help">Help (H)</button></p>
<p id="snap" role="status" aria-live="polite"></p>
<div id="help" hidden><p>S: save the displayed frame. M: save with review_required. H: this help.
Esc: hide this help. Keys are ignored while typing in a field and while held down.</p></div>
<img src="/stream.mjpg" alt="Annotated lane detection stream">
<h2>Status</h2><pre id="s" aria-live="polite">waiting...</pre>
<script>
async function snap(review){const r=await fetch('/snapshot?review='+(review?1:0),{method:'POST'});
const j=await r.json();document.getElementById('snap').textContent=j.message;}
function help(show){const h=document.getElementById('help');h.hidden=!show;
document.getElementById('bh').setAttribute('aria-expanded',String(show));}
document.getElementById('bs').onclick=()=>snap(false);
document.getElementById('bm').onclick=()=>snap(true);
document.getElementById('bh').onclick=()=>help(document.getElementById('help').hidden);
document.addEventListener('keydown',e=>{if(e.repeat||e.ctrlKey||e.metaKey||e.altKey)return;
const t=e.target.tagName;if(t==='INPUT'||t==='TEXTAREA'||e.target.isContentEditable)return;
const k=e.key.toLowerCase();if(k==='s')snap(false);else if(k==='m')snap(true);
else if(k==='h')help(document.getElementById('help').hidden);else if(k==='escape')help(false);});
async function poll(){try{const r=await fetch('/status.json');
document.getElementById('s').textContent=JSON.stringify(await r.json(),null,1);}catch(e){}
setTimeout(poll,500);}poll();
</script></body></html>"""


class Latest:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.status = None, 0, {}
        self.frame_id = None
        self.snapshot = 'no snapshot yet'
        self.request_snapshot = None  # set by Viewer

    def set_jpeg(self, data, frame_id=None):
        with self.cond:
            self.jpeg, self.seq, self.frame_id = data, self.seq + 1, frame_id
            self.cond.notify_all()

    def wait_newer(self, seq, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seq, timeout)
            return self.jpeg, self.seq


def make_handler(latest, max_clients):
    sem = threading.BoundedSemaphore(max_clients)

    class Handler(BaseHTTPRequestHandler):
        timeout = 5.0  # a stalled client times out instead of piling up

        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, 'text/html; charset=utf-8', PAGE)
            elif self.path == '/status.json':
                self._send(200, 'application/json', json.dumps(latest.status).encode())
            elif self.path == '/stream.mjpg':
                self._stream()
            else:
                self._send(404, 'text/plain', b'not found')

        def do_POST(self):
            if not self.path.startswith('/snapshot'):
                self._send(404, 'text/plain', b'not found')
                return
            review = self.path.endswith('review=1')
            msg = latest.request_snapshot(review)
            self._send(200, 'application/json', json.dumps({'message': msg}).encode())

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            if not sem.acquire(blocking=False):
                self._send(503, 'text/plain', b'too many viewers')
                return
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                seq = -1
                while True:
                    jpeg, seq = latest.wait_newer(seq, 2.0)
                    if jpeg is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                                     str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass
            finally:
                sem.release()

    return Handler


class ThreadingServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Viewer(Node):
    def __init__(self, latest):
        super().__init__('lane_viewer')
        self.latest = latest
        self.declare_parameter('image_topic', '/lane_following/debug/image/compressed')
        self.declare_parameter('status_topic', '/lane_following/status')
        self.create_subscription(CompressedImage, self.get_parameter('image_topic').value,
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, self.get_parameter('status_topic').value,
                                 self._on_status, 5)
        self.snap_pub = self.create_publisher(String, '/lane_following/snapshot/request', 10)
        self.create_subscription(String, '/lane_following/snapshot/result', self._on_snap_result, 10)
        self._last_snap_t = 0.0
        latest.request_snapshot = self.request_snapshot

    def request_snapshot(self, review):
        now = time.monotonic()
        if now - self._last_snap_t < 0.5:  # debounce: one bundle per press
            return 'ignored (repeat within 0.5 s)'
        fid = self.latest.frame_id
        if fid is None:
            return 'no displayed frame yet'
        self._last_snap_t = now
        self.snap_pub.publish(String(data=json.dumps({'frame_id': fid, 'review': review})))
        self.latest.snapshot = f'Saving frame {fid}...'
        return self.latest.snapshot

    def _on_snap_result(self, msg):
        try:
            d = json.loads(msg.data)
            self.latest.snapshot = d.get('message', '')
        except ValueError:
            pass

    def _on_image(self, msg):
        fid = None
        head = msg.header.frame_id.split('|', 1)[0]
        if head.isdigit():
            fid = int(head)
        self.latest.set_jpeg(bytes(msg.data), fid)

    def _on_status(self, msg):
        if msg.status:
            st = msg.status[0]
            d = {'message': st.message, 'level': int.from_bytes(st.level, 'little')
                 if isinstance(st.level, bytes) else int(st.level)}
            d.update({kv.key: kv.value for kv in st.values})
            # The node reports async "Saved <path>" / failure via snapshot_last.
            if 'snapshot_last' in d and d['snapshot_last'] != 'no snapshot yet':
                self.latest.snapshot = d['snapshot_last']
            d['viewer_snapshot'] = self.latest.snapshot
            self.latest.status = d


def main(argv=None):
    ap = argparse.ArgumentParser(description='lane_following preview viewer')
    ap.add_argument('--opencv', action='store_true', help='show an OpenCV window')
    ap.add_argument('--web', action='store_true', help='serve http://HOST:PORT/')
    ap.add_argument('--host', default='127.0.0.1', help='bind address (default localhost only)')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--max-clients', type=int, default=3)
    args, ros_args = ap.parse_known_args(argv)
    if not (args.opencv or args.web):
        args.web = True

    rclpy.init(args=ros_args)
    latest = Latest()
    node = Viewer(latest)
    server = None
    if args.web:
        server = ThreadingServer((args.host, args.port), make_handler(latest, args.max_clients))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        node.get_logger().info(f'browser viewer on http://{args.host}:{args.port}/ (no vehicle controls)')
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    window = args.opencv
    try:
        if window:
            try:
                cv2.namedWindow('lane_following', cv2.WINDOW_NORMAL)
            except cv2.error as exc:
                node.get_logger().warning(f'no display for OpenCV window ({exc}); continuing headless')
                window = False
        seq = -1
        show_help = False
        while rclpy.ok():
            if not window:
                time.sleep(0.2)
                continue
            jpeg, seq = latest.wait_newer(seq, 0.1)
            if jpeg is not None:
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    _footer(img, latest.snapshot, show_help)
                    cv2.imshow('lane_following', img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('s'), ord('S'), ord('m'), ord('M')):
                node.request_snapshot(key in (ord('m'), ord('M')))
            elif key in (ord('h'), ord('H')):
                show_help = not show_help
            if key == 27 or cv2.getWindowProperty('lane_following', cv2.WND_PROP_VISIBLE) < 1:
                cv2.destroyAllWindows()
                window = False
                if server is None:
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server is not None:
            server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _footer(img, snap_text, show_help):
    lines = [f'snapshot: {snap_text}'[:100]]
    if show_help:
        lines += ['S save snapshot | M mark for review | H help | Esc close viewer',
                  'Viewer keys never arm, stop or drive the vehicle.']
    h = img.shape[0]
    for i, t in enumerate(reversed(lines)):
        y = h - 8 - 16 * i
        cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)


if __name__ == '__main__':
    main()
