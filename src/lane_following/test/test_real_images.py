"""Smoke tests on the supplied photos (lane_detection_plan/test_lane_images).

These check that the detector does something sensible on real photographs;
they are not campus validation. Skipped when the images are not present,
for example in CI.
"""

import os

import cv2
import pytest

from lane_following.config import DetectorConfig, SteeringConfig, TrackerConfig
from lane_following.pipeline import INVALID, VALID, LanePipeline

IMG_DIR = os.environ.get('LANE_TEST_IMAGES', os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'lane_detection_plan', 'test_lane_images')))

pytestmark = pytest.mark.skipif(not os.path.isdir(IMG_DIR), reason=f'no test images at {IMG_DIR}')


def run(name, **det):
    img = cv2.imread(os.path.join(IMG_DIR, name))
    if img is None:
        pytest.skip(f'{name} missing')
    return LanePipeline(DetectorConfig(**det), SteeringConfig(), TrackerConfig()).process(img, static=True)


@pytest.mark.parametrize('name', ['000000.png', '000001.png', '000002.png', '000003.png'])
def test_double_yellow_sequence_frames(name):
    r = run(name)
    assert r.state == VALID, r.reason
    # Left boundary is the yellow centre line, right is the white dashed lane line.
    assert r.left.stripe.color == 'yellow'
    assert r.right.stripe.color == 'white'
    assert abs(r.steering.error_px) < 20


def test_double_yellow_resolves_at_native_width():
    r = run('000000.png', processing_width=1242)
    assert r.state == VALID, r.reason
    assert r.left.group.kind == 'double'
    xs = sorted(m.x_at(r.y_eval) for m in r.left.group.members)
    assert r.left.x_eval == pytest.approx(xs[-1])  # lane-facing stripe
    assert r.x_lane > xs[-1]


def test_highway_dashed_right_lane():
    r = run('lane-detection-using-machine-learning.webp')
    assert r.state == VALID, r.reason
    assert r.left.stripe.color == 'yellow'
    assert r.right.stripe.style == 'dashed'


def test_rural_road_valid():
    assert run('images (1).jpg').state == VALID


def test_vegetation_scene_does_not_fake_a_lane():
    # Yellow roadside vegetation must not yield a confident lane at image centre.
    r = run('images.jpg')
    assert r.state == INVALID or r.reason == 'OUT_OF_LANE'
