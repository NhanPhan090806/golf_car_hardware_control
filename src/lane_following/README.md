# lane_following

Phase A preview, plus an optional Phase B steering-only hardware mode (see below).

Classical lane detection with a pixel-gain steering request:

```text
error_px = x_lane - image_center_x            # at y_eval = 0.75 * (H - 1)
steer    = clip(alpha * error_px, ±steer_limit) # negative = left, positive = right
```

By default (preview), the node publishes `/lane_following/debug/motion_cmd`
with zero throttle, zero brake, and neutral gear. It does not touch the vehicle
topics or depend on vehicle feedback. A
passing image test does not show stable closed-loop driving.

## Dependencies

```bash
sudo apt install python3-opencv   # Ubuntu 22.04: OpenCV 4.5.4 for system Python 3.10
```

Run with system Python, not the `ai_venv` venv. `cv_bridge` is not needed; `ros_image.py`
converts `bgr8`/`rgb8`/`bgra8`/`rgba8`/`mono8` images itself.

## Build and test

```bash
cd ~/robot_hardware_ws
colcon build --symlink-install --packages-up-to lane_following
source install/setup.bash
colcon test --packages-select lane_following --event-handlers console_direct+
colcon test-result --verbose
```

`test_real_images.py` uses `lane_detection_plan/test_lane_images/` and is skipped
when that folder is absent (set `LANE_TEST_IMAGES=/path` to override).

## Run

No ROS, single images or a folder (each image is STATIC, no history):

```bash
ros2 run lane_following lane_detect_image ~/golf_car_hardware_control/lane_detection_plan/test_lane_images --show
ros2 run lane_following lane_detect_image img.jpg --out /tmp/lane_out --json
ros2 run lane_following lane_detect_image img.jpg --set detector.lane_reference_x_ratio=0.35
```

`--out` is the only option that writes files.

ROS preview node:

```bash
ros2 launch lane_following offline.launch.py                               # synthetic stream
ros2 launch lane_following offline.launch.py mode:=image path:=/abs/dir rate_hz:=10.0
ros2 launch lane_following offline.launch.py mode:=video path:=/abs/clip.mp4 viewer:=true
ros2 launch lane_following camera_preview.launch.py source:=ros topic:=/camera/image_raw
ros2 run lane_following lane_viewer --web --opencv    # http://127.0.0.1:8080/
```

Inspect:

```bash
ros2 topic hz /lane_following/debug/motion_cmd      # ~20 Hz
ros2 topic echo /lane_following/debug/motion_cmd
ros2 topic echo /lane_following/status
```

## States

| State | Meaning | Published angle |
| --- | --- | --- |
| VALID | Both boundaries supported in this frame (OBSERVED or INTERPOLATED) and result age ≤ `max_result_age_s` | `steer` |
| DEGRADED | One boundary HELD from an earlier frame (≤ `hold_duration_s`), preview estimate only | 0.0 |
| INVALID | Missing/crossed/ambiguous pair, bad frame, or invalid value (`reason` in status) | 0.0 |
| STALE | Result older than `max_result_age_s` (checked on every 20 Hz tick) | 0.0 |

A single still image is labelled STATIC. It can never use tracking history,
and the node refuses to send it to the vehicle topic.

## Safety defaults

These are provisional software values, not demonstrated bounds.

- `output.motion_topic` defaults to the debug topic. `/vehicle/control/motion_cmd`
  is refused unless `output.allow_vehicle_topic:=true` and `ROS_DOMAIN_ID` is
  non-zero, which is meant for a message-contract check in an isolated domain only.
- `runtime.max_result_age_s` = 0.15 s. It is measured from frame acquisition,
  and republishing never refreshes it. Measure latency under load before relying on it.
- `tracker.hold_duration_s` = 0.10 s (must be ≤ the age limit, 0 disables). A held
  boundary's age counts from the last frame that actually saw it.
- `steering.alpha_rad_per_px` = 0.002 at 640 px wide is an example value, not
  a vehicle gain. Alpha is rescaled by `alpha_reference_width / processing_width`.
- Never run this alongside `vehicle_keyboard_teleop` on the same command topic.

## Phase B: hardware integration (steering only)

`vehicle_integration.launch.py` starts the existing hardware bringup with
`config/vehicle_controller_lane.yaml` (a copy of the shared controller config
with `require_function_command: true`) and the lane node with
`hardware.enabled:=true`. The default teleop bringup and its yaml are unchanged.

Throttle and brake are never commanded. While ARMED, every `motion_cmd` has
`speed_percentage 0`, `brake_percentage 0`, `gear NEUTRAL`. A stop is
`func_cmd.motion_enable=false` (the controller's normal safe command: steering
IDLE, brake IDLE, throttle 0). It never sets `emergency_stop`.

| State | func_cmd | motion_cmd |
| --- | --- | --- |
| DISARMED (start) | `motion_enable=false` at 20 Hz | not published |
| ARMED | `motion_enable=true` at 20 Hz | steering only, clipped to `hardware.steer_limit_rad` (0.5) |
| STOP_LATCHED | `motion_enable=false` | not published |

Arming is refused unless all of these hold: the input is a live camera/ROS
stream with header stamps checked; the controller reports
`require_function_command=true`; this node is the only publisher on both
command topics; `/vehicle/state/system` is fresh (≤ 0.5 s), online, fault-free,
HSH motion allowed, and no e-stop; and the lane result is fresh and VALID.
`safe_command_active` is not required to be false beforehand, so there's no
startup deadlock. Instead, the controller must report enabled within
`arm_confirm_timeout_s` (0.5 s), or the node latches.

While ARMED, any loss of those conditions latches STOP_LATCHED. That includes a
DEGRADED/STALE/INVALID lane, stale vehicle state, a fault, HSH revoking motion,
or a second publisher (for example teleop). Nothing re-arms automatically.
`reset` (refused while a fault or e-stop persists) returns to DISARMED, and you
must arm again. If the lane process dies, the controller's 250 ms command
timeout removes motion. That doesn't depend on this process.

Operator controls (the physical E-stop and AUTO/MANUAL switch stay primary):

```bash
ros2 run lane_following lane_arm_console          # A arm, D disarm, Space/X stop, R reset, Q quit (sends disarm)
ros2 service call /lane_following/arm std_srvs/srv/Trigger
ros2 service call /lane_following/stop std_srvs/srv/Trigger
ros2 service call /lane_following/reset std_srvs/srv/Trigger
ros2 service call /lane_following/disarm std_srvs/srv/Trigger
```

Parameter changes are rejected while ARMED.

### Mock test in WSL (no hardware)

```bash
export ROS_DOMAIN_ID=42            # mock_vehicle refuses domain 0
ros2 launch lane_following vehicle_integration.launch.py mock:=true
ros2 run lane_following lane_arm_console
ros2 param set /vehicle_controller mock.fault true          # also: mock.motion_allowed, mock.publish_state, mock.camera_rate_hz
```

`mock_vehicle` only imitates the controller's ROS contract (parameter, state
topics, timeout rule) and a stamped synthetic camera. It does not model HSH,
ODrive, relays or steering dynamics.

### Bench mode: saved images steer a raised car

For a car with no lanes in view (storeroom), `bench:=true` lets a saved image,
folder, video or the synthetic stream arm. This drives the full path: image →
detection → angle → `/vehicle/control/motion_cmd` → real steering.

```bash
ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=image path:=/abs/test.png
ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=synthetic rate_hz:=20.0   # sweeps L/R
ros2 run lane_following lane_arm_console
```

- `bench` is read at launch only. The status and console show `BENCH ARMED`/`BENCH DISARMED`.
- The cap is `bench_steer_limit_rad` (0.2 rad = 20% of full lock, about 14 motor turns).
  Raise it only after the sign/centre checks pass.
- Everything else still applies: manual arm, the lane controller profile, sole
  publisher, HSH permission (AUTO switch), fresh vehicle state, latching, and
  throttle/brake 0. An image with no valid lane can't arm.
- One image gives a constant angle. To test the stop path, use a folder that
  includes a no-lane image: arrival of that frame latches a stop and releases
  the steering to IDLE.

### On the car (B2 stationary test, not performed by this software)

Only with explicit authorization, the car raised/secured, a person at the
physical E-stop, and the AUTO/MANUAL switch in reach:

```bash
ros2 launch lane_following vehicle_integration.launch.py topic:=/camera/image_raw
# or, if the hardware stack is already running with the lane profile:
ros2 launch lane_following vehicle_integration.launch.py bringup:=false
```

Check, in order: the direction of a small positive request (should turn right),
centre, the 0.5 rad limit, that a stop and a covered camera return steering to
IDLE, and that killing the lane node times out. Raise `steer_limit_rad` only
after that. The plan's B3 driving trial needs a separate, reviewed throttle
change. This package cannot command throttle.

### Snapshots

The viewer keys `S` (save) and `M` (mark for review) and the browser buttons
save one bundle of the displayed frame on the lane node host:
`snapshot.directory/<session>/<utc>_<frame_id>_<n>/{original.png, annotated.png, metadata.json}`.
Writes go through a bounded queue with bundle, size and free-disk limits. A
press is rejected visibly instead of blocking, and nothing is overwritten or
deleted. Missing vehicle feedback is recorded as `null`.

### Image QoS and large frames

Best-effort transport of uncompressed 640x360 frames between processes lost
about two thirds of the frames in WSL with the default 212 kB socket buffers.
The mock therefore publishes reliable, and `mock:=true` sets
`input.image_reliability:=reliable`. For a real camera driver, either match its
QoS (`image_reliability:=reliable`) or raise buffers:
`sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608`.
Recheck on the Jetson.

## Detector notes

- Paint mask: horizontal top-hat on HSV V, gated by white/yellow HSV ranges.
  Per-row runs give stripe centres directly, so the plan's Canny + HoughLinesP
  step and its edge deduplication aren't needed.
- Runs are linked into pieces. Pieces are chained across dash gaps by fit
  residual and fitted as `x = f(y)` (linear, with a bounded quadratic fallback).
- Parallel same-colour stripes at double spacing, judged per row so it follows
  perspective, form one boundary group. The lane-facing member is the reference.
  Inconclusive spacing is reported as AMBIGUOUS_GROUPING.
- The initial lane is the pair straddling `lane_reference_x_ratio`. After that,
  each side is tracked within `max_jump_ratio`.
- At 640 px, the double yellow in `000000.png` (~6 px gap) merges into one
  stripe. The centre reference is then the merged stripe centre. Use
  `processing_width: 1242` to resolve both stripes.
- `images.jpg` (yellow roadside vegetation) is correctly rejected or flagged
  OUT_OF_LANE. Tune `roi` and the colour thresholds to the campus paint.
