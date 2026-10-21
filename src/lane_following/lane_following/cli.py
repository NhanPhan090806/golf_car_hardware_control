"""Run the detector on saved images or a video without ROS.

Examples:
  lane_detect_image path/to/image.jpg --show
  lane_detect_image path/to/dir --out /tmp/lane_out      # annotated copies + JSON
  lane_detect_image clip.mp4 --video --show

Each image is processed as STATIC (no history), a directory as STATIC
images unless --sequence is given, and a video as a time sequence.
Nothing is written unless --out is passed.
"""

import argparse
import glob
import json
import os
import sys
import time

import cv2

from . import config as cfg
from .input_sources import IMAGE_EXTS
from .overlay import render
from .pipeline import LanePipeline


def _load_configs(path, overrides):
    if path:
        c = cfg.load_yaml_sections(path)
    else:
        c = {s: cls() for s, cls in cfg.SECTIONS.items()}
    for item in overrides:
        key, _, raw = item.partition('=')
        section, _, field = key.partition('.')
        if section not in cfg.SECTIONS or not field:
            raise SystemExit(f'--set expects section.field=value, got {item!r}')
        import yaml
        values = {f.name: getattr(c[section], f.name) for f in cfg.dataclasses.fields(c[section])}
        values[field] = yaml.safe_load(raw)
        c[section] = cfg.from_mapping(cfg.SECTIONS[section], values)
    cfg.check_consistency(c['tracker'], c['runtime'])
    return c


def _line(name, r):
    s = r.steering
    return (f'{name:45} {r.state:8} {r.reason:22} L={r.left.state:12} R={r.right.state:12} '
            f'err={"n/a" if s is None else f"{s.error_px:+7.1f}px"} '
            f'steer={"n/a" if s is None else f"{s.steer_rad:+.4f}rad"} {r.proc_ms:5.1f}ms')


def main(argv=None):
    ap = argparse.ArgumentParser(description='lane detection on images/video (no ROS, no hardware)')
    ap.add_argument('input', help='image file, directory of images, or video file')
    ap.add_argument('--config', help='lane_following.yaml (default: built-in defaults)')
    ap.add_argument('--set', action='append', default=[], metavar='SECTION.FIELD=VALUE',
                    help='override one setting, e.g. detector.lane_reference_x_ratio=0.35')
    ap.add_argument('--video', action='store_true', help='treat input as a video file')
    ap.add_argument('--sequence', action='store_true', help='treat a directory as a time sequence')
    ap.add_argument('--fps', type=float, default=10.0, help='sequence timing for --sequence')
    ap.add_argument('--show', action='store_true', help='OpenCV window; any key = next, Esc = quit')
    ap.add_argument('--out', help='write annotated PNG + JSON summaries to this directory')
    ap.add_argument('--json', action='store_true', help='print full JSON summary per frame')
    a = ap.parse_args(argv)

    c = _load_configs(a.config, a.set)
    pipe = LanePipeline(c['detector'], c['steering'], c['tracker'])
    if a.out:
        os.makedirs(a.out, exist_ok=True)

    def handle(name, img, acq_t, fid, static):
        r = pipe.process(img, acq_t, fid, name, static)
        print(_line(name, r))
        if a.json:
            print(json.dumps(r.summary(), indent=1))
        ann = None
        if a.out or a.show:
            ann = render(r, 'PREVIEW')
        if a.out:
            stem = os.path.splitext(os.path.basename(name))[0]
            cv2.imwrite(os.path.join(a.out, f'{stem}_annotated.png'), ann)
            with open(os.path.join(a.out, f'{stem}.json'), 'w', encoding='utf-8') as f:
                json.dump(r.summary(), f, indent=1)
        if a.show:
            cv2.imshow('lane_detect_image', ann)
            if (cv2.waitKey(0 if static else 1) & 0xFF) == 27:
                return False
        return True

    if a.video:
        cap = cv2.VideoCapture(a.input)
        if not cap.isOpened():
            raise SystemExit(f'cannot open video {a.input}')
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        fid = 0
        while True:
            ok, img = cap.read()
            if not ok or not handle(f'{os.path.basename(a.input)}#{fid}', img, fid / fps, fid, False):
                break
            fid += 1
        cap.release()
    else:
        files = sorted(f for f in glob.glob(os.path.join(a.input, '*')) if f.lower().endswith(IMAGE_EXTS)) \
            if os.path.isdir(a.input) else [a.input]
        if not files:
            raise SystemExit(f'no images found at {a.input}')
        t0 = time.monotonic()
        for i, f in enumerate(files):
            img = cv2.imread(f, cv2.IMREAD_COLOR)
            if img is None:
                print(f'{f}: unreadable', file=sys.stderr)
                continue
            static = not a.sequence
            if not handle(os.path.basename(f), img, t0 + i / a.fps, i, static):
                break
    if a.show:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
