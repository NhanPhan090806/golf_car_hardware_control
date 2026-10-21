import json
import os
import time

import pytest

from lane_following import synthetic as sy
from lane_following.config import DetectorConfig, SteeringConfig, TrackerConfig
from lane_following.overlay import render
from lane_following.pipeline import LanePipeline
from lane_following.snapshot import FrameCache, SnapshotConfig, SnapshotWriter


def result(fid=7):
    img, *_ = sy.two_lane()
    return img, LanePipeline(DetectorConfig(), SteeringConfig(), TrackerConfig()).process(img, 1.0, fid)


def wait_for(writer, n, timeout=5.0):
    t = time.monotonic() + timeout
    while writer.count < n and time.monotonic() < t:
        time.sleep(0.02)


def test_bundle_has_three_matching_files(tmp_path):
    w = SnapshotWriter(SnapshotConfig(directory=str(tmp_path), min_free_mb=0.0), lambda r: render(r))
    img, r = result(fid=42)
    ok, msg = w.request(img, r, {'vehicle': {'measured_steering_rad': None}}, review=True)
    assert ok and msg == 'Saving...'
    wait_for(w, 1)
    w.stop()
    bundles = os.listdir(w.session_dir)
    assert len(bundles) == 1 and '_42_' in bundles[0] and not bundles[0].endswith('.incomplete')
    files = sorted(os.listdir(os.path.join(w.session_dir, bundles[0])))
    assert files == ['annotated.png', 'metadata.json', 'original.png']
    meta = json.load(open(os.path.join(w.session_dir, bundles[0], 'metadata.json')))
    assert meta['frame_id'] == 42 and meta['review_required'] is True
    assert meta['vehicle']['measured_steering_rad'] is None  # unavailable, not zero
    assert w.last_message.startswith('Saved ')


def test_names_unique_and_never_overwritten(tmp_path):
    w = SnapshotWriter(SnapshotConfig(directory=str(tmp_path), min_free_mb=0.0), lambda r: render(r))
    img, r = result()
    for _ in range(3):
        assert w.request(img, r, {}, False)[0]
        wait_for(w, w.count + 1)
    w.stop()
    assert len(set(os.listdir(w.session_dir))) == 3


def test_limits_and_full_queue_reject(tmp_path):
    w = SnapshotWriter(SnapshotConfig(directory=str(tmp_path), max_bundles=1, min_free_mb=0.0),
                       lambda r: render(r))
    img, r = result()
    assert w.request(img, r, {}, False)[0]
    wait_for(w, 1)
    ok, msg = w.request(img, r, {}, False)
    assert not ok and 'limit' in msg
    w.stop()

    slow = SnapshotWriter(SnapshotConfig(directory=str(tmp_path), queue_capacity=1, min_free_mb=0.0),
                          lambda r: (time.sleep(0.3), render(r))[1])
    results = [slow.request(img, r, {}, False)[0] for _ in range(4)]
    assert results[0] and not all(results)
    slow.stop()


def test_low_disk_reports_failure(tmp_path):
    w = SnapshotWriter(SnapshotConfig(directory=str(tmp_path), min_free_mb=1e12), lambda r: render(r))
    img, r = result()
    assert w.request(img, r, {}, False)[0]
    t = time.monotonic() + 3
    while w.last_message == 'no snapshot yet' and time.monotonic() < t:
        time.sleep(0.02)
    w.stop()
    assert w.last_message.startswith('FAILED') and w.count == 0


def test_frame_cache_bounded():
    c = FrameCache(3)
    for i in range(10):
        c.put(i, None, i)
    assert c.get(6) is None and c.get(9) == (None, 9)
