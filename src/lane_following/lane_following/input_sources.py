"""Frame sources and the bounded latest-value slot that decouples them.

Every source yields Frame objects stamped with a local monotonic acquisition
time. Nothing here queues frames: LatestSlot keeps exactly one frame and
overwrites it, so a slow consumer drops old frames instead of building a
backlog.
"""

import dataclasses
import glob
import os
import threading
import time
from typing import Iterator, Optional

import cv2
import numpy as np

IMAGE_EXTS = ('.png', '.jpg', '.jpeg', '.bmp', '.webp', '.tif', '.tiff')


@dataclasses.dataclass(frozen=True)
class Frame:
    image: np.ndarray  # BGR uint8, read-only, owned by this Frame
    acq_t: float  # time.monotonic() at acquisition
    frame_id: int
    source: str
    static: bool = False  # still image: no fresh acquisition
    stamp_age_s: Optional[float] = None  # producer-stamp age (ROS header), if known
    epoch: int = 0  # bumps on seek/restart; consumer resets tracking


def _own(image):
    image = np.ascontiguousarray(image)
    image.setflags(write=False)
    return image


class LatestSlot:
    """Single-element mailbox; put() replaces, get() returns newest or None."""

    def __init__(self):
        self._lock = threading.Lock()
        self._item = None
        self._seq = 0
        self.dropped = 0

    def put(self, item):
        with self._lock:
            if self._item is not None:
                self.dropped += 1
            self._item = item
            self._seq += 1

    def take(self):
        with self._lock:
            item, self._item = self._item, None
            return item

    def peek(self):
        with self._lock:
            return self._item, self._seq


class FrameSource:
    """Base: iterate frames; `stop()` is safe from another thread."""

    def __init__(self, name):
        self.name = name
        self._stop = threading.Event()
        self.error = None

    def stop(self):
        self._stop.set()

    def frames(self) -> Iterator[Frame]:
        raise NotImplementedError


class ImageSource(FrameSource):
    """One file, or every image in a directory, sorted by name.

    A directory replays as a sequence at `rate_hz` (useful for frame
    sequences such as 000000.png ...). A single image is re-emitted as a
    STATIC frame; it is never presented as a fresh observation.
    """

    def __init__(self, path, rate_hz=10.0, loop=False):
        super().__init__(f'image:{path}')
        if os.path.isdir(path):
            files = sorted(f for f in glob.glob(os.path.join(path, '*')) if f.lower().endswith(IMAGE_EXTS))
        else:
            files = [path]
        if not files:
            raise FileNotFoundError(f'no images at {path}')
        self.files, self.rate_hz, self.loop = files, float(rate_hz), loop
        self.static = len(files) == 1

    def frames(self):
        period = 1.0 / self.rate_hz
        fid, epoch = 0, 0
        cache = {}
        while not self._stop.is_set():
            for f in self.files:
                if self._stop.is_set():
                    return
                img = cache.get(f) if self.static else None
                if img is None:
                    img = cv2.imread(f, cv2.IMREAD_COLOR)
                    if img is None:
                        self.error = f'cannot read {f}'
                        continue
                    img = _own(img)
                    if self.static:
                        cache[f] = img
                yield Frame(img, time.monotonic(), fid, f'image:{os.path.basename(f)}', self.static, None, epoch)
                fid += 1
                self._stop.wait(period)
            if not (self.loop or self.static):
                return
            if not self.static:
                epoch += 1  # restarting a sequence is a seek


class VideoSource(FrameSource):
    """Video file (paced at its own FPS) or camera device index."""

    def __init__(self, path_or_index, loop=False, rate_hz=None):
        is_cam = isinstance(path_or_index, int) or str(path_or_index).isdigit()
        super().__init__(f'camera:{path_or_index}' if is_cam else f'video:{path_or_index}')
        self.target = int(path_or_index) if is_cam else str(path_or_index)
        self.is_camera, self.loop, self.rate_hz = is_cam, loop, rate_hz

    def frames(self):
        cap = cv2.VideoCapture(self.target)
        if not cap.isOpened():
            self.error = f'cannot open {self.target}'
            return
        fps = self.rate_hz or (cap.get(cv2.CAP_PROP_FPS) if not self.is_camera else 0) or 30.0
        period = 0.0 if self.is_camera else 1.0 / fps
        fid, epoch = 0, 0
        next_t = time.monotonic()
        try:
            while not self._stop.is_set():
                ok, img = cap.read()
                if not ok:
                    if self.loop and not self.is_camera:
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        epoch += 1
                        continue
                    self.error = 'end of stream' if not self.is_camera else 'camera read failed'
                    return
                yield Frame(_own(img), time.monotonic(), fid, self.name, False, None, epoch)
                fid += 1
                if period:
                    next_t += period
                    delay = next_t - time.monotonic()
                    if delay > 0:
                        self._stop.wait(delay)
                    else:
                        next_t = time.monotonic()
        finally:
            cap.release()


class SyntheticSource(FrameSource):
    """Procedural moving-lane video (see synthetic.py); no files needed."""

    def __init__(self, rate_hz=20.0, **scene):
        super().__init__('synthetic')
        self.rate_hz, self.scene = float(rate_hz), scene

    def frames(self):
        from .synthetic import moving_scene
        period = 1.0 / self.rate_hz
        fid = 0
        while not self._stop.is_set():
            img = moving_scene(fid / self.rate_hz, **self.scene)
            yield Frame(_own(img), time.monotonic(), fid, self.name)
            fid += 1
            self._stop.wait(period)


def make_source(mode, path='', rate_hz=10.0, loop=True):
    if mode == 'image':
        return ImageSource(path, rate_hz=rate_hz, loop=loop)
    if mode == 'video':
        return VideoSource(path, loop=loop)
    if mode == 'camera':
        return VideoSource(int(path or 0))
    if mode == 'synthetic':
        return SyntheticSource(rate_hz=rate_hz)
    raise ValueError(f"unknown input mode {mode!r}; use image, video, camera, synthetic or ros")


class SourceThread:
    """Runs a FrameSource into a LatestSlot on a daemon thread."""

    def __init__(self, source: FrameSource, slot: LatestSlot):
        self.source, self.slot = source, slot
        self.thread = threading.Thread(target=self._run, name=f'src:{source.name}', daemon=True)
        self.last_put_t = None
        self.finished = False

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        try:
            for frame in self.source.frames():
                self.slot.put(frame)
                self.last_put_t = frame.acq_t
        except Exception as exc:  # report, never take the process down
            self.source.error = f'{type(exc).__name__}: {exc}'
        finally:
            self.finished = True

    def stop(self, timeout=1.0):
        self.source.stop()
        self.thread.join(timeout)
