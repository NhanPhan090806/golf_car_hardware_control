# Lightweight lane following implementation plan

Prepared 19 October 2026. This is a development and verification plan; it does not implement or launch vehicle control.

The target is a slow campus demonstration between two deliberately painted lane boundaries. Use Python, NumPy, OpenCV, and ROS 2 Humble on Ubuntu 22.04. The user has already built the existing workspace successfully in WSL2. No neural network, GPU inference, mapping, or navigation framework is needed for this first version.

The developer does not currently have the car or its camera/steering hardware. The immediate deliverable is therefore a complete hardware-free pixel-to-angle prototype. Vehicle calibration, physical feedback, and advanced controllers are later work; they must not become prerequisites for finishing this prototype.

## Development phases

| Phase | Scope | Hardware requirement |
| --- | --- | --- |
| A: implement now | Classical lane detection, pixel error times tunable gain, clipped command, ROS preview, optional OpenCV/browser viewers, image/video tests | None; an accessible webcam or ROS camera stream is optional |
| B: integrate when available | Actual camera recordings, steering-command calibration, vehicle feedback and stop-path verification, secured stationary tests, then a slow baseline trial | Car and appropriate test setup |
| C: evaluate later | PID/PD on pixel offset or metric-path Pure Pursuit, compared against the baseline in simulation and then on the car | Recorded video can test perception; meaningful controller evaluation needs a closed-loop simulator or the car |

Finish Phase A independently. Its result is a verified perception-to-command software pipeline, not a claim that the car follows a campus route reliably. PID and Pure Pursuit are possible later alternatives, not mandatory dependencies or a required sequence of upgrades.

## 1. Define what the first version must do

- Accept one saved image, a video file, a camera device, or an existing ROS `sensor_msgs/msg/Image` topic through selectable input adapters.
- Detect two lane boundaries, estimate their midpoint at a configurable image row, and calculate a signed horizontal pixel error.
- Support solid/solid, solid/dashed, dashed/solid, and dashed/dashed boundaries, including gaps at the evaluation row; aggregate visible dash evidence instead of requiring continuous paint.
- Recognize closely spaced double-yellow stripes as one separator/boundary group, rather than mistaking the narrow gap between them for the driving lane.
- Convert that error into a steering request using a tunable radians-per-pixel gain and clip the final result to [-1.0, +1.0].
- Use this vehicle's convention: negative = left, positive = right.
- Publish continuously during live operation, with explicit handling of missing lanes, stalled input, and stale results.
- Show optional visual feedback in an OpenCV window, a browser, or both. Save nothing to disk by default.
- Start in preview mode, with no publication to the real vehicle command topics.

This is an image-based proportional controller. It is a useful baseline, but it does not calculate a calibrated physical road-wheel angle from camera geometry. Passing an image test does not demonstrate stable closed-loop driving. Initially assume a forward-facing camera mounted centrally, a fixed processing resolution, slow forward motion, and straight or gently curving boundaries. Reverse driving and sharp curves are outside the first version.

Phase A specifically excludes PID integral/derivative terms, metric bird's-eye projection, wheelbase measurement, physical steering calibration, Pure Pursuit, MPC, and live vehicle feedback requirements. Keep steering logic replaceable so later controllers can reuse the input, detection, visualization, and ROS infrastructure.

## 2. Pixel geometry and steering contract

Use image coordinates with x increasing to the right and y increasing downward. All calculations and displayed pixel units refer to the processing image, initially 640 x 360. Preserve the camera aspect ratio and map any crop/resize coordinates consistently.

```text
image_center = ((W - 1) / 2, (H - 1) / 2)
y_eval       = evaluation_row_ratio * (H - 1)
x_left       = left_boundary(y_eval)
x_right      = right_boundary(y_eval)
x_lane       = (x_left + x_right) / 2
error_px     = x_lane - image_center.x
steer_raw    = alpha_rad_per_px * error_px
steer_target = clip(steer_raw, -steer_limit_rad, +steer_limit_rad)
```

The final published value must also be finite and clipped to [-1.0, +1.0]. Reject invalid configuration rather than silently accepting a negative gain, invalid ROI, or inconsistent limits.

| Lane midpoint relative to image center | Error | Steering request |
| --- | --- | --- |
| Left | Negative | Negative: steer left |
| Centered | Zero | Zero |
| Right | Positive | Positive: steer right |

Example only: alpha = 0.002 rad/pixel and error = +50 pixels gives +0.1 rad. This is arithmetic for an offline test, not a validated vehicle gain. Pixel-to-radian gain is empirical: perspective, camera mounting, speed, and steering calibration affect its useful value. Do not calculate it by simply converting camera field of view to radians per pixel.

Keep the requested red dot at the exact image center. Draw the green diamond at `(x_lane, y_eval)`. These can have different y coordinates: the error is their horizontal separation, not their Euclidean distance. Draw a vertical center reference and a horizontal measurement segment at y_eval to make that clear.

Start with evaluation_row_ratio = 0.75 and make it tunable. The exact center row can be near the horizon or outside the road ROI. If evaluating at 0.5 is requested, accept it only when both detected boundaries have sufficient support there; do not silently extrapolate far beyond visible lane evidence. The red dot is a visual reference for the assumed car centerline, not the physical location of the car in the image.

Keep processing resolution fixed while tuning alpha. If resolution changes, retune alpha or explicitly rescale it so equal fractional image offsets produce equal steering requests. Record the processing width next to the gain.

Optional refinements after the raw formula passes tests; these are not Phase A completion requirements:

- A small pixel deadband to reduce jitter near center.
- Time-based low-pass filtering and a steering slew limit, with both raw and final values visible.
- Reset filter state after loss/recovery or configuration changes; never allow smoothing to delay the invalid-input response.
- A future heading term if lateral error alone produces oscillation or poor curve tracking, evaluated in Phase C. A single-row midpoint can be centered while the car is pointing across the lane, so this controller has an inherent limitation.

## 3. Classical lane detection pipeline

For Phase A, start immediately with the supplied highway photograph, other saved road images, synthetic two-line images, and prerecorded video. Tune the paint masks/ROI on these inputs and label their source in results. Actual mounted-camera recordings and campus paint calibration belong to Phase B; their absence must not block development. The supplied photograph is an algorithm smoke test, not representative validation for the campus course.
**Note:** Apply/rely on opencv-python or predefined CPython as much as possbile for detection speed and ease of implementation. 
1. Resize with a documented transform; apply camera undistortion if calibration exists and distortion materially affects the boundaries.
2. Restrict processing to a configurable road polygon, initially roughly the lower half of the frame. Exclude sky, trees, dashboard, and irrelevant roadside objects.
3. Convert to HSV or HLS. Use configurable white/yellow paint masks, or a single mask matching the demo paint. Avoid relying on raw brightness alone.
4. Use small morphological opening/closing operations to remove isolated noise and bridge small gaps without merging separate boundaries.
5. Apply Canny to the masked road region and `HoughLinesP` to generate candidate line segments.
6. Reject short and implausibly horizontal segments. Cluster segments into boundary candidates, then robustly fit `x = a*y + b` using segment support. This form also handles nearly vertical boundaries.
7. Select an ordered left/right pair using road position, line support, plausible separation, and temporal continuity. Do not assume every left boundary has one fixed slope sign, or average all segments on each side indiscriminately.
8. Evaluate both boundaries at y_eval. Require x_left < x_right, plausible width, sufficient vertical support around the evaluation row, and a plausible frame-to-frame change. Paint need not intersect y_eval: interpolation between supported dash segments is allowed. Use the same checks at additional rows to reject crossing/inconsistent pairs.
9. Return a result containing frame identity/time, validity, invalid reason, boundary models, per-boundary observation/support state, target coordinates, lane width, error, and a documented quality score.

The quality score is a heuristic based on support, fit residuals, geometry, and continuity; do not present it as a calibrated probability. An offset car center may lie outside the selected pair: do not force the pair to straddle image center, which would hide large errors. Reject implausible targets and mark out-of-lane conditions separately.

**Note 2:** we will talk about the missing one line lane later.
Require supported estimates of both boundaries in version 1. Dashed markings are not automatically missing boundaries: several visible dashes can support one boundary model. A genuinely absent boundary must not be invented from an assumed lane width; that one-boundary reconstruction remains deferred as noted above. Start with straight fits; use the limited curved-fit fallback below only when the available evidence requires it.

### 3.1 Dashed-boundary support and fallbacks

These cases belong to Phase A image/video testing and keep exactly the same pixel-to-angle controller:

| Left boundary | Right boundary | Required behavior |
| --- | --- | --- |
| Solid | Solid | Fit each boundary from its supported marking pixels/segments |
| Solid | Dashed | Aggregate right-hand dashes into one boundary model; the solid left side is not a substitute for right-side evidence |
| Dashed | Solid | Same behavior with sides reversed |
| Dashed | Dashed | Fit each dash chain independently, then validate their ordered pair |

Use the following fallback order:

1. **Fit across visible dashes in the current frame.** Keep smaller paint components/short segments that could be individual dashes. Associate them by compatible direction, horizontal position at shared rows, and fit residuals. Use multiple vertical bands/sliding windows to collect the same boundary through gaps. Reject isolated road artifacts rather than assuming every short white segment is paint. The fitted model may cross an unpainted interval when supported by evidence on both sides of that interval.
2. **Bridge only limited gaps.** Tune `HoughLinesP` segment length/gap parameters and a small direction-aware morphology operation for the processing resolution. Bound the interpolation span relative to image height and check support/residuals. Do not use a huge closing kernel or gap threshold to merge neighboring lanes, both sides of a stripe, or unrelated markings. A sufficiently long visible dash can support a local fit; isolated tiny fragments without adequate vertical span cannot.
3. **Use a lightweight curved-fit fallback where justified.** If a straight fit has high residuals but enough consistent dash-chain support exists across the ROI, fit `x(y)` with a quadratic or piecewise model. Check ordering and width at several rows and limit extrapolation. Unsupported/sharp curves remain invalid. This improves perception for curved dashed examples without introducing PID, Pure Pursuit, or metric road projection.
4. **Track briefly through an evidence gap in video preview.** Maintain separate left/right identities and last-observed times, and search near the last accepted boundary on the next frame. An optional bounded hold may display a previous boundary during a brief gap, explicitly labeled HELD/DEGRADED. It must expire by elapsed observation time even if frames keep arriving; never refresh its observation timestamp merely by copying or refitting old data. Reset after expiry, input seek/restart, or source/resolution changes. A single still image has no temporal fallback unless prior frames actually exist.
5. **Invalidate when support is insufficient.** After the support/hold limits expire, remove the valid green target and report which boundary is unsupported. Do not keep a previous angle indefinitely, synthesize a boundary from fixed width, or switch to another lane to make detection appear successful.

Differentiate `OBSERVED` (adequate current-frame support), `INTERPOLATED` (adequate current-frame support around an unpainted interval), `HELD` (previous-frame evidence only), and `MISSING` for each boundary. A current-frame interpolated pair can be VALID. If either boundary is HELD, the overall result is DEGRADED and any angle is a preview estimate only; it is not eligible for real vehicle actuation in this plan. Preview hold duration is configurable and cannot exceed the general result-age limit; use zero to disable it. Phase B must separately validate any future permission to drive on temporally predicted boundaries rather than silently inheriting preview behavior.

Add configurable settings for minimum dash-component support, segment length/gap, association residual, vertical span, maximum interpolation/extrapolation gap, fit mode, and preview hold duration. Use normalized image fractions where appropriate; log the effective pixel values. Defaults are provisional until evaluated against the supplied images/video.

For multi-lane scenes such as the attached example, explicitly select the target lane pair using a configurable ROI/initial lane selection, then track that pair. Do not average every painted line or pick the two outermost lines. Group the parallel edges of one painted stripe, and closely spaced double-yellow markings used as one boundary, before choosing the pair; the two yellow stripes must not be mistaken for the two sides of the driving lane. These rules identify a lane to follow, not permission to cross a solid or dashed marking or choose a route at a junction.

### 3.2 Double-yellow markings and target-lane selection

Include the supplied double-yellow road photograph in Phase A test inputs. Here, two closely spaced yellow stripes form one center separator. They are two paint markings, not the left and right boundaries of a drivable lane. For the right-hand lane in this scene, select the lane-facing yellow stripe as the left boundary reference and the right white road-edge marking as the right boundary reference. The green midpoint must lie in that lane, not between the yellow stripes or between the two outer white road edges. Selection of the left-hand lane, if explicitly configured for a different test, uses the opposite side of the separator and the left white boundary.

Plan the detector in three levels: edge segments -> individual painted stripes -> boundary groups. Canny may produce two edges for each yellow stripe; deduplicate those before deciding that two separate yellow stripes exist. Group stripes only when their color, close spacing, local direction/curvature, and shared vertical support agree across multiple image bands. Because perspective changes spacing with image row, do not use one fixed whole-image pixel-gap threshold. Use row-dependent/normalized gap limits and compare separation with local stripe width and the selected lane width. Do not merge genuinely separate lane boundaries merely because both are yellow.

Store each group's member stripe models and select a consistent lane-facing stripe-center reference for the existing pixel-error calculation. For an ordinary single stripe use that stripe's center. This is an image-space reference convention, not a claim about legal lane geometry or metric road clearance; annotate/document it consistently. Expose paint color and single/double grouping separately from solid/dashed support, since a boundary group may contain mixed continuous and broken markings. If group membership or target-lane selection is ambiguous, report AMBIGUOUS/INVALID rather than silently choosing a narrow gap as the lane.

Show the two yellow stripe models, their shared boundary-group ID, and the selected left/right boundary references in the overlay. Save stripe membership, grouping score/reason, selected lane side, and boundary-reference convention in snapshot metadata. Grouping does not provide a missing opposite boundary; keep the deferred one-boundary reconstruction policy.

Acceptance cases: double-yellow plus right white boundary; the mirrored left-lane selection; a single wide yellow stripe whose two edges must not become a false double; two genuine lane boundaries of the same color that must remain separate; changing apparent yellow-pair spacing toward the horizon; a partially occluded group member; and an ambiguous grouping that must fail visibly. Where only one group member is supported, report that uncertainty instead of inventing a second stripe. Verify the green target never falls in the narrow separator gap when the selected target is the adjacent driving lane. The hill/crest in the supplied image can also limit visible support; mark unsupported distant fits invalid rather than extrapolating across the crest.

## 4. Proposed package and process structure

Add one new `ament_python` package, `lane_following`, under `robot_hardware_control_ws/src/`, alongside the existing repositories. Keep the existing hardware controller, CAN drivers, HSH, and shared-memory code unchanged during the perception baseline.

```text
lane_following/
  lane_following/
    input_sources.py       # Image, video, camera, ROS image adapters
    detector.py            # Pure OpenCV/NumPy detection
    steering.py            # Pure error-to-angle calculation
    lane_node.py           # ROS adapter, result freshness, command timer
    debug_viewer.py        # Optional OpenCV/browser display
  config/lane_following.yaml
  launch/offline.launch.py
  launch/camera_preview.launch.py
  launch/vehicle_integration.launch.py  # Phase B deliverable; not needed now
  test/
  README.md
```

These names are proposed deliverables, not existing executable commands. Implement the offline and preview paths in Phase A. The hardware launch and real command/enable adapter are deferred to Phase B; do not implement a full arming state machine merely to finish the current prototype.

Data flow:

```text
Selectable input -> bounded latest-frame slot -> detector + steering
                                               |
                                      timestamped latest result
                                        /                 \
                              command timer          optional overlay
                              /          \                |
                       preview topic   vehicle topics   debug viewer
                       (Phase A)       (Phase B)
```

Keep detector and steering logic free of ROS and UI dependencies so exactly the same functions handle a saved image and a live frame. A bounded capture/processing worker supplies immutable result snapshots; ROS callbacks and the publication timer must not read the camera, run detection, render windows, encode web video, or write files. A stalled worker must not prevent the timer from recognizing stale results. Synchronize snapshots explicitly; do not share a mutable NumPy frame unsafely between threads.

One Python process with bounded workers is sufficient initially, plus an optional viewer process. This is not a hard real-time design. If measured contention affects command cadence, split vision and command publication into separate ROS processes with a small, explicitly timestamped result message; do not use an unstamped float as the entire freshness contract.

Use `rclpy`, `numpy`, OpenCV, `cv_bridge`, `sensor_msgs`, `diagnostic_msgs`, and the existing `vehicle_msgs`. Optional web viewing may add Flask or an equivalent small HTTP server. Select the real camera driver after identifying the camera; do not assume a USB webcam driver will support a Jetson CSI camera.

## 5. ROS topics and command ownership

| Topic | Type | Purpose |
| --- | --- | --- |
| Configurable camera topic, e.g. `/camera/image_raw` | `sensor_msgs/msg/Image` | Existing camera driver input, if selected |
| `/lane_following/debug/motion_cmd` | `vehicle_msgs/msg/VehicleMotionCommand` | Preview steering requests; no hardware subscriber expected |
| `/lane_following/debug/image/compressed` | `sensor_msgs/msg/CompressedImage` | Optional annotated stream |
| `/lane_following/status` | `diagnostic_msgs/msg/DiagnosticArray` | Validity, reason, frame age, error, gain, raw/final steering, rate |
| `/vehicle/control/motion_cmd` | `vehicle_msgs/msg/VehicleMotionCommand` | Real vehicle motion commands, explicit hardware mode only |
| `/vehicle/control/func_cmd` | `vehicle_msgs/msg/VehicleFunctionCommand` | Enable/stop commands owned by the integration controller |
| `/vehicle/state/motion` | Existing `VehicleMotionState` | Measured feedback |
| `/vehicle/state/system` | Existing `VehicleSystemState` | Hardware permission, faults, timeout and safety state |

In Phase A, populate zero throttle, zero brake request, and neutral gear explicitly in the preview message; these are test payload values, not a physical stop procedure. Validity and timestamps remain in status/internal result metadata because `VehicleMotionCommand` has no header or validity field. Do not use the debug message by itself as a future control handoff without adding freshness information.

Phase A needs no messages on `/vehicle/state/*` and no hardware permission/enable handshake. By default use the debug topic above. To check the exact final topic name and message contract without hardware, allow a deliberate output-topic override to `/vehicle/control/motion_cmd` in an isolated test ROS domain with only an inspection subscriber. This is a message-delivery test, not vehicle integration; do not connect that test domain to the car. No function-command publisher or hardware bringup is required for it.

The message's `steering_angle_rad` field is a float32. Its numeric bound is enforced by software/configuration, not by the ROS message type. `speed_percentage` is a throttle percentage, not a closed-loop speed request in m/s. The developer must populate steering, throttle, brake, and gear deliberately; a steering-only update still replaces the other fields of this combined command.

Use best-effort, volatile, keep-last depth 1 for image subscriptions where compatible with the camera publisher. Use reliable, volatile, keep-last depth 1 for vehicle command publishers to match the current controller subscriptions. Verify QoS using `ros2 topic info -v` rather than assuming a visible topic means data is delivered. Keep debug image traffic best-effort and rate-limited.

Start command publication at 20 Hz (50 ms period); the existing configured timeout is 250 ms. This gives margin, but publish rate alone does not make stale perception safe. Every timer tick must independently check the age and validity of the underlying camera result. A repeated command must never reset that result's age.

There must be exactly one publisher controlling each real command topic during a run. Do not run keyboard teleop concurrently with autonomous publication. Future teleop/autonomy arbitration needs an explicit selector, not competing publishers.

## 6. Freshness, loss of lanes, and enable behavior

Phase A implements PREVIEW with VALID, DEGRADED, INVALID, and STALE result states. DEGRADED identifies the explicitly bounded dashed-boundary temporal hold described in section 3.1; it is not valid hardware control. It publishes no real enable requests and does not wait for physical feedback. For a single-image inspection, an explicit static-preview mode can display and republish the result while labeling it STATIC; live/video freshness tests must continue to enforce acquisition and result age. Replaying a still image is not evidence of fresh camera acquisition.

Phase B adds DISARMED, ARMED, and STOP_LATCHED integration states. Defaults remain hardware publication disabled, motion enable false, and throttle zero. A static image or looping video is never an eligible source for real actuation. In the rules below, hardware-state checks, emergency-stop publication, and operator arming/recovery apply to Phase B only.

- Timestamp frames at acquisition; preserve ROS image headers. Track local monotonic acquisition/processing times for watchdogs. For external ROS cameras, validate capture age using synchronized clocks as well as local reception age. A fresh arrival of an old buffered image is not a fresh observation.
- Process only the newest available frame; discard old queued frames. Detect stalled frame acquisition, not merely whether the camera handle remains open.
- Start with a 150 ms maximum result age as a provisional software value, then measure latency under load and review it for the actual demo speed. It is a tunable assumption, not a demonstrated stopping bound.
- Unsupported/invalid boundaries (including HELD preview-only estimates), nonfinite values, camera failure, expired perception, expired required vehicle state, or a hardware fault remove eligibility for motion. Current-frame interpolation across supported dash gaps is not itself a missing-boundary fault.
- In preview, show INVALID/STALE and suppress any apparently valid lane target. A held last target may be displayed only as visibly stale; never silently continue it as valid control.
- In hardware mode, latch a stop request, command zero throttle, and assert the integration controller's explicit emergency-stop function. Do not silently turn the last steering request into an indefinitely fresh command.
- Do not automatically re-arm after lanes return. Require deliberate operator reset/re-arm after the cause clears, and honor the vehicle's existing recovery procedure.
- Choose and verify brake/gear sequencing for the existing controller before actuation. Its emergency-stop branch actively applies braking; an application-level stop request still requires hardware validation. Do not invent a universal brake percentage or an untested automatic gear change.
- Monitor the hardware-reported online, fault, and motion-permitted state before enable. Do not make enable depend on `safe_command_active` already being false: that state may legitimately remain true while the controller is disarmed, creating a startup deadlock.

Phase B current-workspace finding: `vehicle_controller.yaml` has `require_function_command: false` with a bench-test comment. Before real integration, review an explicit hardware profile with that setting true and the required function-command heartbeat. Do not bypass HSH permission to make a mock test pass. This configuration does not need changing for Phase A preview.

Phase B documentation mismatch: the manual describes an active stop/brake response to command timeout. In the inspected controller source, an ordinary timeout reaches `write_safe_command()`, which removes throttle/drive relay outputs and sets brake and steering to IDLE. The explicit `emergency_stop` branch applies the brake. This is a controller-level observation; the complete physical behavior also depends on HSH and hardware. Verify the complete timeout path before moving tests. A watchdog process crash must not depend on that same process continuing to publish a stop. Resolving the physical behavior is not a Phase A completion requirement.

## 7. Visual feedback and storage policy

Render one shared overlay for both UI backends:

- Detected left/right boundaries and the selected ROI.
- Per-side support state (OBSERVED/INTERPOLATED/HELD/MISSING), support age and fit quality. Distinguish observed paint from fitted segments across gaps; show held models in amber and label them DEGRADED.
- Red filled dot at the exact image center.
- Green diamond at the detected midpoint on y_eval, only while valid.
- For a DEGRADED preview, an optional amber diamond may show the held estimate; never present it as the valid green target.
- A vertical image-center reference and a horizontal error segment at y_eval.
- Text: signed pixel error, alpha in rad/pixel, raw angle, final angle in radians, processing dimensions, evaluation row, valid/invalid reason, result age, measured processing rate, and PREVIEW/ARMED/STOP state.
- Label the value as a requested command angle. In Phase A show vehicle feedback as unavailable, not zero; in Phase B clearly distinguish the requested angle from measured feedback when both are shown.

Suggested display:

```text
PREVIEW | VALID | frame age 42 ms
error: +50.0 px | alpha: 0.0020 rad/px
raw: +0.100 rad | output: +0.100 rad
processing: 640x360 | evaluation row: 0.75
```

OpenCV UI uses `imshow` and nonblocking event polling in the viewer process. If no display is available, report that and continue headless. Browser UI serves a small monitoring page with an MJPEG stream and current status; retain only the latest encoded frame for streaming. Phase B adds the data-capture buttons below, but no vehicle enable buttons. Tune through validated ROS parameters/configuration; reject or defer control-affecting changes while armed.

Use independent flags for overlay publication, OpenCV viewing, browser viewing, and recording. Viewing a stream does not require writing images or video to disk. Defaults: all recording off, UI optional, and JPEG encoding skipped when no viewer/debug subscriber needs it. Start debug rendering/streaming at 5 Hz, independent of 20 Hz control publication. Bound queues, client count, and slow-client buffering so browser stalls cannot grow memory or block control.

Bind the web server to localhost initially. On the car, view it through SSH port forwarding. An explicit remote-bind configuration can be considered later if needed.

If recording is explicitly enabled, require an output location, rate/duration or size cap, and a documented retention policy. Stop recording at the cap rather than deleting arbitrary user files. Bound/rotate logs as well. A manual snapshot button/key should save exactly one frame.

### Phase B snapshot keys for later analysis

When the hardware is available, add the following controls to the focused OpenCV viewer and equivalent buttons/shortcuts to the browser viewer. The same capture feature may be tested earlier on saved images/video; it is not a new Phase A completion requirement.

| Key | Browser button | Action |
| --- | --- | --- |
| `S` | Save snapshot | Save one analysis bundle for the displayed detected frame |
| `M` | Mark for review | Save the same bundle with `review_required: true` for a bad detection or interesting scene |
| `H` | Help | Show the shortcut legend and configured snapshot directory |
| `Esc` | Close viewer | Close the OpenCV view or dismiss the browser overlay; this does not stop, disarm, or change vehicle commands |

Handle upper/lower case consistently. Accept shortcuts only while the viewer has focus, ignore browser shortcuts inside text fields, and debounce/ignore repeated keydown events so holding a key does not produce an image flood. Display the shortcut legend. These keys belong to the viewer and must never be forwarded to keyboard teleop or interpreted as driving commands. `M` marks a frame for offline review; it is not an emergency-stop control.

Each accepted snapshot request saves a matched bundle:

```text
<snapshot_dir>/<session_id>/<utc_timestamp>_<frame_id>_<counter>/
  original.png     # Original camera frame before resize, masking, or drawing
  annotated.png    # Detected boundaries, markers, and text at processing size
  metadata.json    # Capture identity, detector/controller configuration and result
```

The metadata includes source capture time, local receive/process time, snapshot-request time, frame ID, original/processing dimensions and transform, input source, ROI, paint/detection settings, boundary estimates, per-boundary support state/last-observed time, fit mode/residuals, interpolation/hold use, validity/reason, lane width, evaluation row, pixel error, alpha, raw/final steering request, clipping state, software version where available, and review flag. Include the latest available commanded gear/throttle/brake and measured steering/speed/system feedback with their own timestamps/ages. Missing feedback is `null`/unavailable, never an invented zero. Preserve enough configuration to rerun the detector against `original.png` later. For temporal HELD cases, mark the result as history-dependent; reproducing the tracker requires the preceding frame sequence, not just this saved still image.

Save exactly the source frame and result corresponding to the displayed frame ID, not a newer camera frame with an older overlay. Keep only a small bounded cache of matched source/result pairs for capture requests; if the requested frame has expired, report that and ask the operator to retry rather than silently substituting another frame. Saving invalid detections is allowed and useful: preserve their invalid reason and do not draw a fabricated valid target.

Perform snapshot encoding and disk writes in a separate bounded writer queue, outside camera processing, ROS callbacks, and the command timer. Configure queue capacity, maximum session bytes/count, and minimum free disk space. If the queue is full, storage is unavailable, or a limit is reached, reject the capture request visibly and continue control processing; do not block it or delete existing user data. Use unique names and never overwrite an earlier bundle. Report success only after all three files are written, and identify incomplete bundles if writing fails.

Show `Saving...`, then `Saved <bundle path>` or a concise failure message in either viewer. Browser requests save on the capture host (normally the Jetson); show that host/path explicitly, since this does not automatically download files to the laptop. Use the existing localhost/SSH-forwarded access for browser capture actions. Continuous recording remains off unless separately enabled; pressing `S` or `M` saves one bundle only.

## 8. What runs in each test mode

The first three rows are the current implementation scope. The remaining rows document optional or future integration work and must not block the prototype.

| Mode | Run | Hardware stack? | Evidence produced |
| --- | --- | --- | --- |
| Single image | Detector/steering CLI, optional viewer | No | Geometry, overlay, arithmetic |
| Offline ROS image/video | Proposed `offline.launch.py`, optional viewer, `ros2 topic echo/hz` | No | ROS output and replay behavior |
| Live camera preview | Camera driver or direct capture adapter, proposed `camera_preview.launch.py`, optional viewer | No | Live latency, robustness, stale detection |
| Controller unit tests | Existing vehicle controller tests with fake interfaces | No | Selected controller logic; not full bringup |
| Full mock integration, optional later | Dedicated fake hardware/state producer and separate mock launch | Mock only | Controller/FSM integration without physical actuation |
| Real vehicle | Working camera source, existing hardware bringup, proposed integration launch, optional remote viewer | Yes | Measured actuation and closed-loop behavior |

For Phase B reference only, real hardware bringup already exists:

```bash
ros2 launch vehicle_controller vehicle_hardware_bringup.launch.py
```

That launch starts the controller manager, joint-state broadcaster, vehicle controller, and HSH. Its URDF loads the ODrive, RS485, and relay plugins. Do not start duplicate controller managers/HSH instances or separately spawn a second copy of the vehicle controller. It is not an offline simulation launcher. LiDAR, SLAM, Nav2, and the legacy swerve launch files are unnecessary for this image-only baseline.

In WSL, start with image/video input. Live camera access is a separate device-access question: a successful colcon build does not establish that a Windows USB camera is available inside WSL. A camera feed from a ROS driver is an alternative. For the vehicle demo, prefer running perception and command publication locally on the Jetson; use the laptop/browser for observation.

After implementing the new package, build from the sourced Humble workspace with `colcon build --symlink-install --packages-up-to lane_following`, then source its `install/setup.bash`. Use the user's designated Python venv for Windows-side Python work; ROS execution in WSL must use a Python environment compatible with the installed Humble `rclpy`/`cv_bridge`, rather than attempting to execute the Windows venv inside Linux.

Useful inspection commands once preview publication exists:

```bash
ros2 topic echo /lane_following/debug/motion_cmd
ros2 topic hz /lane_following/debug/motion_cmd
ros2 topic echo /lane_following/status
ros2 topic info /vehicle/control/motion_cmd -v
ros2 topic echo /vehicle/state/system
ros2 topic echo /vehicle/state/motion
```

Preview requires no subscriber pretending to be the car. `echo` is sufficient to inspect messages. A mock vehicle is useful only when testing feedback/gating or controller integration. A fake CAN heartbeat alone is not a complete simulation of ODrive, RS485, relays, and vehicle motion.

## 9. Implementation order and acceptance checks

| Stage | Deliverable | Exit criteria |
| --- | --- | --- |
| A1 | Pure image detector and pixel-to-angle function | Correct pair selection and overlay for solid/solid, solid/dashed, dashed/solid, dashed/dashed supplied/synthetic images; supported gap interpolation and deterministic sign/clipping tests pass; no hardware needed |
| A2 | Video input and freshness logic; optional accessible live source | Bounded memory; no increasing backlog; dashed-boundary association/optional preview hold is labeled and expires; genuinely missing lanes, stalled input, and corrupt frames become invalid promptly |
| A3 | Humble node and preview launch | Intended QoS connects; output is approximately 20 Hz; angle is finite and bounded; stale input is visibly invalid; no vehicle feedback dependency |
| A4 | OpenCV/browser viewers | Both show the same result; viewers can be disabled; disk use stays flat with recording off; slow clients do not affect command cadence |
| B1 | Actual camera data, snapshot controls, and integration verification | Tune to camera/paint; verify matched original/annotated/metadata snapshot bundles and review markers; verify stop, enable, timeout, command ownership, and recovery behavior; mock/unit tests may support this |
| B2 | Secured stationary hardware verification | Physical steering sign, center, range mapping, brake/timeout behavior, and feedback verified under the vehicle test procedure |
| B3 | Slow closed-loop baseline trial | Pixel gain and evaluation row tuned incrementally; limits recorded; loss of lanes causes the validated stop response |
| C1 | Optional advanced-controller comparison | Evaluate PID/PD or Pure Pursuit against the same baseline in closed-loop simulation and then vehicle trials; retain only demonstrated improvements |

Phase A is complete when A1-A4 pass on available images/video in WSL2. It does not depend on B/C completion, a full mock vehicle, recordings from the real camera, or access to a Jetson. Record missing hardware evidence honestly instead of treating it as a software failure.

Phase B snapshot acceptance: one `S`/`M` action produces exactly one complete, uniquely named bundle; the original, overlay and metadata refer to the same frame; review marking and unavailable-feedback fields are correct; keyboard/browser capture behaves consistently; and queue-full, disk-limit and write-failure cases report failure without interrupting command timing. Confirm disk usage stays flat when neither snapshots nor recording are requested.

Required synthetic arithmetic examples at alpha = 0.002:

- Equal left/right offsets about image center -> error 0 -> steering 0.
- Midpoint 50 px left -> -0.1 rad; 50 px right -> +0.1 rad.
- Midpoint error +600 px -> +1.0 rad after clipping in the pure mapping test, even though detection geometry would reject an out-of-frame target.
- Missing/crossed boundaries, NaN values, invalid gain/ROI, and stale data -> invalid result, never a valid forward-driving command.
- Resolution rescaling preserves the documented gain convention; no silent doubling of steering after a width change.

Required dashed-lane tests: evaluate all four solid/dashed combinations, with paint absent exactly at y_eval but supported above/below it; staggered gaps on the two sides; dash spacing beyond the configured support limit; one/both completely absent boundaries; unrelated adjacent-lane dashes and double-yellow markings; and gently curved dash chains. In video, test a brief disappearance/reappearance, a prolonged loss, identity preservation, source seek/reset, and continuing fresh frames that contain no new boundary evidence. Assert that a held boundary cannot extend its own lifetime, unsupported history cannot initialize a still-image estimate, and the target does not jump to another lane. These tests validate detection/fallback behavior, not physical closed-loop tracking.

Use available test images/video with shadows, worn paint, one missing line, extra markings, different lighting, and camera vibration where available. Synthetic cases supplement missing datasets. Do not choose alpha by saturation alone: frequent clipping is a diagnostic of gain, geometry, or operating conditions, not evidence that control is good. In Phase A benchmark in WSL2 with UI off and on; measure 20 Hz command timing and result age rather than inferring them from average FPS. Repeat those measurements on the Jetson during Phase B. Run existing meaningful controller tests when integrating; a passing build is not physical verification.

Hardware tests are future steps, not actions authorized or performed by this planning document. Follow the repository's requirement for explicit authorization and a secured mechanism before closed-loop motion, homing, calibration, or raw CAN transmit tests.

## 10. Later controller options

Keep `steering.py` behind a small controller interface that consumes the current detection result and timestamp. Phase A implements only `pixel_gain`; do not add unused PID/MPC libraries or metric calibration dependencies now.

| Later option | Additional work | Evaluation requirement |
| --- | --- | --- |
| PID/PD on pixel offset | Time-aware error history, derivative filtering, integral anti-windup if integral action is used, reset on invalid/stale input, speed-dependent tuning as needed | Closed-loop simulator or vehicle; video playback alone cannot show how steering changes the next observation |
| Pure Pursuit | Camera/ground-plane calibration, local centerline in metres relative to the vehicle, wheelbase, lookahead selection, and calibrated conversion from road-wheel angle to this vehicle's command | Verify geometry and actuator mapping, then compare tracking and steering behavior in closed loop |
| MPC, optional much later | Validated motion/actuator model, state estimates, constraints, optimizer and fallback | Justify with measured limitations of simpler controllers; outside the current deliverable |

Clarification: PID on pixel offset uses the image error as feedback and does not intrinsically require a steering-angle sensor. Basic Pure Pursuit computes desired curvature/road-wheel angle from a local path and wheelbase; measured steering-angle feedback is not an input required by its basic formula either. Actual steering feedback is valuable for calibrating and verifying this car's response, particularly because its ROS command is mapped to motor turns. The reason to defer these controllers is the current lack of vehicle/closed-loop validation and, for Pure Pursuit, calibrated geometry and actuator mapping—not a universal requirement that both algorithms need an angle sensor.

When those prerequisites become available, compare a controller against the pixel baseline using lateral error, boundary departures, steering oscillation/rate, command saturation, latency, and loss-of-lane recovery at matched speeds/routes. Do not select a controller merely because its name sounds more advanced. Closed-loop simulation can begin before hardware returns, but its results must be labeled simulation evidence and depend on the model assumptions.

## 11. Developer handoff

Implement for an indefinite live stream from the first version. Single-image input is one adapter for debugging, not the main architecture. Keep capture, perception, command publication, and visualization decoupled by bounded latest-value buffers. Never create a growing list of frames, enqueue every frame for later processing, call `imwrite` per callback, or make UI/client availability a prerequisite for control execution. Preserve acquisition timestamps and invalid states all the way to the command decision.

Implement and deliver Phase A first, using available saved inputs. Architect for a future camera without requiring that camera to be present. Do not replace the requested `alpha * error_px` baseline with PID or Pure Pursuit, and do not wait for physical angle feedback, wheelbase, or metric ground-plane calibration. Keep later controller/integration work explicitly separate from current acceptance criteria.

Expose and document: input mode/topic/device, processing size, ROI, paint thresholds, morphology/edge/Hough settings, dash association/support/gap settings, fit mode, preview hold duration, pair-validity thresholds, evaluation row, alpha_rad_per_px, steering limit, optional deadband/filter/slew limit, publish rate, freshness limits, hardware enable mode, debug rates, UI switches, and bounded recording settings. In Phase B also document snapshot keys/buttons, capture host/directory, metadata schema, frame-cache/writer-queue limits, and storage caps. Defaults should allow useful preview immediately and require deliberate configuration before physical control. Leave demo throttle zero until vehicle testing establishes a suitable low-speed setting; percentage throttle is not a known speed.

## References and inspected evidence

- Existing local files: `vehicle_msgs/msg/VehicleMotionCommand.msg`, `VehicleFunctionCommand.msg`, `VehicleMotionState.msg`, and `VehicleSystemState.msg`; `vehicle_controller/config/vehicle_controller.yaml`; `vehicle_controller/src/vehicle_controller.cpp`; `vehicle_controller/launch/vehicle_hardware_bringup.launch.py`; `vehicle_controller/urdf/vehicle_hardware.urdf`; controller test registration; `Customer_Vehicle_Integration_Manual.docx`.
- [OpenCV thresholding with inRange](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html): configurable paint-color segmentation.
- [OpenCV Hough line transform](https://docs.opencv.org/4.x/d9/db0/tutorial_hough_lines.html): classical candidate-line generation.
- [ROS 2 Humble QoS documentation source](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst): sensor-data profiles and reliability compatibility.

No source code, ROS configuration, device setup, or hardware state was changed while preparing this plan. No runtime or hardware tests were performed.

## Revisions made during Phase A implementation (20 October 2026)

Implemented as `src/lane_following` (inside this repo, built from `~/robot_hardware_ws`). Deviations from the text above:

- **Detection stage (section 3, steps 5-6):** Canny + `HoughLinesP` replaced by per-row runs of the filled paint mask (top-hat on HSV V, white/yellow gate). Runs give stripe centres directly, so the two Canny edges of one stripe never need deduplicating, and dash gaps are bridged by explicit residual-checked chaining rather than large Hough gap settings. Still pure OpenCV/NumPy.
- **Morphology:** the default vertical close is off. On slanted paint it filled the gap between double-yellow stripes.
- **Double yellow at 640 px:** in the supplied `000000.png`, the two stripes are about 6 px apart at 640 px and merge into one stripe, so the reference is the merged centre. They resolve as a `double` group at `processing_width: 1242`. Choose the width for the campus paint in Phase B.
- **`cv_bridge` not used:** it is not installed, and `ros_image.py` converts `bgr8/rgb8/bgra8/rgba8/mono8` itself. **Flask not used:** the browser viewer is stdlib `http.server`, bound to 127.0.0.1.
- **Python environment (section 8):** everything ROS-related runs with system Python 3.10 in WSL. The `ai_venv` venv is not involved. OpenCV comes from `apt install python3-opencv`.
- **Vehicle-topic override (section 5):** refused unless `output.allow_vehicle_topic:=true` and `ROS_DOMAIN_ID` is non-zero.
- **Phase B finding:** `vehicle_controller` treats `steering_angle_rad` as a fraction of `max_steering_angle_rad` (1.0), mapped linearly to ±70 motor turns. It is a normalized steering command, not a calibrated road-wheel angle. Keep this in mind when tuning alpha.
- **Image replay:** replay intervals longer than `max_result_age_s` show STALE between frames. Use `rate_hz` ≥ 10 for directory playback.

## Phase B implementation notes (20 October 2026)

- **Scope reduced to steering only, at the user's request.** Throttle and brake are always 0 and gear NEUTRAL. The stop response is `motion_enable=false` (the controller's safe command), not `emergency_stop` and its brake. Section 6's "assert emergency stop" is therefore not used. B3 (slow driving) needs a separate, reviewed throttle change.
- **DISARMED/ARMED/STOP_LATCHED**, arming preconditions, the no-auto-re-arm rule and the operator console/services are implemented in `arming.py`, `hardware_link.py` and `arm_console.py`. The controller profile is `config/vehicle_controller_lane.yaml` (`require_function_command: true`). `vehicle_hardware_bringup.launch.py` gained a `controller_config` argument; its default is unchanged.
- **Snapshot keys and buttons** (S/M/H/Esc) and bounded bundle writing are implemented.
- **Verified with `mock_vehicle` only** (WSL, isolated ROS domain). It covered arming refusals, 20 Hz commands with throttle/brake/gear fixed, latching on fault, HSH revoke, camera stall, stale vehicle state and a second publisher, reset rules, and the controller timing out about 240 ms after the lane process was killed. Nothing was run on the car.
- **WSL finding:** best-effort raw 640x360 images lost about two thirds of frames between processes with the default 212 kB socket buffers. The README documents `input.image_reliability` and the sysctl fix. Recheck on the Jetson.
