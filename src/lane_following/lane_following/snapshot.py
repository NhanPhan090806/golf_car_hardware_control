"""Matched snapshot bundles (original.png, annotated.png, metadata.json).

Requests go through a bounded queue to one writer thread, so encoding and
disk I/O never run in the camera, worker, ROS callback or command timer
paths. Limits: queue capacity, bundles per session, bytes per session and
minimum free disk space. Nothing is ever deleted or overwritten; a full
queue or reached limit rejects the request visibly.
"""

import collections
import dataclasses
import datetime
import json
import os
import queue
import shutil
import threading
import time
import uuid
from typing import Any, Dict, Optional

import cv2


@dataclasses.dataclass(frozen=True)
class SnapshotConfig:
    directory: str = '~/lane_snapshots'
    queue_capacity: int = 4
    max_bundles: int = 200
    max_session_mb: float = 500.0
    min_free_mb: float = 1024.0
    frame_cache: int = 8


class FrameCache:
    """Small bounded map frame_id -> (original image, result) for capture."""

    def __init__(self, size):
        self._lock = threading.Lock()
        self._items = collections.OrderedDict()
        self._size = max(1, int(size))

    def put(self, frame_id, image, result):
        with self._lock:
            self._items[frame_id] = (image, result)
            self._items.move_to_end(frame_id)
            while len(self._items) > self._size:
                self._items.popitem(last=False)

    def get(self, frame_id):
        with self._lock:
            return self._items.get(frame_id)


class SnapshotWriter:
    def __init__(self, config: SnapshotConfig, render_fn):
        self.config = config
        self.render_fn = render_fn
        self.session_id = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:6]
        self.session_dir = os.path.join(os.path.expanduser(config.directory), self.session_id)
        self._q = queue.Queue(maxsize=max(1, config.queue_capacity))
        self._lock = threading.Lock()
        self.count = 0
        self.bytes = 0
        self.last_message = 'no snapshot yet'
        self._counter = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='snapshot_writer', daemon=True)
        self._thread.start()

    def request(self, image, result, extra: Dict[str, Any], review: bool):
        """Queue one bundle. Returns (accepted, message); never blocks."""
        with self._lock:
            if self.count >= self.config.max_bundles:
                return False, f'limit reached: {self.config.max_bundles} bundles this session'
            if self.bytes >= self.config.max_session_mb * 1e6:
                return False, f'limit reached: {self.config.max_session_mb:.0f} MB this session'
            self._counter += 1
            counter = self._counter
        try:
            self._q.put_nowait((image, result, extra, review, counter, time.time()))
        except queue.Full:
            return False, 'snapshot queue full; try again'
        return True, 'Saving...'

    def _free_mb(self, path):
        probe = path
        while not os.path.exists(probe):
            parent = os.path.dirname(probe)
            if parent == probe:
                break
            probe = parent
        return shutil.disk_usage(probe).free / 1e6

    def _run(self):
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            ok, msg = self._write(*item)
            with self._lock:
                self.last_message = msg if ok else f'FAILED: {msg}'

    def _write(self, image, result, extra, review, counter, req_wall):
        try:
            if self._free_mb(self.session_dir) < self.config.min_free_mb:
                return False, f'free disk below {self.config.min_free_mb:.0f} MB'
            stamp = datetime.datetime.utcfromtimestamp(req_wall).strftime('%Y%m%dT%H%M%S_%fZ')
            name = f'{stamp}_{result.frame_id}_{counter:04d}'
            final = os.path.join(self.session_dir, name)
            tmp = final + '.incomplete'
            os.makedirs(tmp, exist_ok=False)
            written = 0
            for fname, img in (('original.png', image), ('annotated.png', self.render_fn(result))):
                path = os.path.join(tmp, fname)
                if not cv2.imwrite(path, img):
                    return False, f'cannot write {path} (bundle left as {tmp})'
                written += os.path.getsize(path)
            meta = metadata(result, extra, review, req_wall, self.session_id)
            path = os.path.join(tmp, 'metadata.json')
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(meta, f, indent=1, default=str)
            written += os.path.getsize(path)
            os.rename(tmp, final)  # complete only once all three files exist
            with self._lock:
                self.count += 1
                self.bytes += written
            return True, f'Saved {final}'
        except OSError as exc:
            return False, str(exc)

    def stop(self):
        self._stop.set()
        self._thread.join(1.0)


def metadata(result, extra, review, req_wall, session_id) -> Dict[str, Any]:
    s = result.summary()
    held = 'HELD' in (result.left.state, result.right.state)
    s.update({
        'schema': 'lane_following.snapshot/1',
        'session_id': session_id,
        'review_required': bool(review),
        'snapshot_request_utc': datetime.datetime.utcfromtimestamp(req_wall).isoformat() + 'Z',
        'history_dependent': held,
        'note': 'HELD boundaries need the preceding frames to reproduce' if held else None,
    })
    s.update(extra)
    return s
