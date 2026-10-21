# Run commands

All commands use system Python. Activate `ai_venv` (`workai`) only for the
standalone scripts in `tests/`. Build and run ROS from the workspace:

```bash
cd ~/robot_hardware_ws
source install/setup.bash
```

| Section | Hardware needed | Can anything move? |
| --- | --- | --- |
| [1. Setup and build](#1-setup-and-build) | None | No |
| [2. Tests](#2-tests) | None | No |
| [3. Lane detection, no ROS](#3-lane-detection-no-ros) | None | No |
| [4. Lane preview node](#4-lane-preview-node) | None (camera optional) | No |
| [5. Mock vehicle (WSL)](#5-mock-vehicle-wsl) | None | No (mock only) |
| [6. Vehicle stack and teleop](#6-vehicle-stack-and-teleop) | Car | Yes, operator-driven |
| [7. Lane following on the car](#7-lane-following-on-the-car) | Car | Steering only |
| [8. CAN and hardware diagnostics](#8-can-and-hardware-diagnostics) | CAN/RS485 adapters | Some send frames |

Sections 6–8 need the hardware procedure in `AGENTS.md`. Get explicit
authorization and secure the mechanism before any motion, homing, calibration
or raw CAN transmit. Never run `mock_vehicle` on the car's network.

## 1. Setup and build

First-time setup (Ubuntu 22.04 / WSL2):

```bash
cd ~/golf_car_hardware_control
./setup_env/install_all.sh                # everything, including WSL vcan modules
./setup_env/install_all.sh --skip-can     # skip the kernel module build and vcan
sudo apt install python3-opencv           # needed by lane_following
```

Individual steps (each can be re-run):

```bash
./setup_env/01_install_ros2_humble.sh
./setup_env/02_install_workspace_deps.sh
./setup_env/03_setup_python_venv.sh       # ~/ai_venv for tests/*.py
./setup_env/04_build_wsl_can_modules.sh   # WSL only: vcan, slcan, gs_usb
./setup_env/05_setup_vcan.sh [--persist]  # vcan0, vcan1
./setup_env/06_build_workspace.sh [--no-test]
```

Build:

```bash
deactivate 2>/dev/null; cd ~/robot_hardware_ws
colcon build --symlink-install                                        # everything
colcon build --symlink-install --packages-select lane_following       # one package
colcon build --symlink-install --packages-up-to lane_following        # with dependencies
source install/setup.bash
```

After pulling changes to `vehicle_msgs` or `hardware_ipc_common`, rebuild
everything so all processes share one message and shared-memory layout.

## 2. Tests

```bash
colcon test --packages-select lane_following vehicle_controller hardware_system_handler
colcon test-result --verbose
./build/vehicle_controller/test_vehicle_controller_sequence
./build/hardware_system_handler/test_system_behavior
```

The tests on `lane_detection_plan/test_lane_images` skip when the folder is
missing. Use `LANE_TEST_IMAGES=/path` to point at another copy.

## 3. Lane detection, no ROS

```bash
IMG=~/golf_car_hardware_control/lane_detection_plan/test_lane_images
ros2 run lane_following lane_detect_image $IMG --show                 # any key = next, Esc = quit
ros2 run lane_following lane_detect_image $IMG/000000.png --json      # full result
ros2 run lane_following lane_detect_image $IMG --sequence             # folder as video (tracking on)
ros2 run lane_following lane_detect_image clip.mp4 --video --show
ros2 run lane_following lane_detect_image $IMG --out /tmp/lane_out    # annotated PNG + JSON
ros2 run lane_following lane_detect_image $IMG/000000.png --set detector.processing_width=1242
ros2 run lane_following lane_synthetic_video /tmp/syn.avi --seconds 10
```

Use original camera frames, not screenshots of the viewer overlay.

## 4. Lane preview node

This publishes `/lane_following/debug/motion_cmd` only and never touches the vehicle topics.

```bash
ros2 launch lane_following offline.launch.py                                   # synthetic stream
ros2 launch lane_following offline.launch.py mode:=image path:=$IMG rate_hz:=10.0
ros2 launch lane_following offline.launch.py mode:=video path:=/abs/clip.mp4 viewer:=true
ros2 launch lane_following offline.launch.py viewer:=true viewer_mode:=--opencv
ros2 launch lane_following camera_preview.launch.py source:=camera device:=0 viewer:=true
ros2 launch lane_following camera_preview.launch.py source:=ros topic:=/camera/image_raw \
    image_reliability:=reliable viewer:=true
ros2 run lane_following lane_viewer --web --opencv        # browser: http://127.0.0.1:8080/
```

Inspect and tune:

```bash
ros2 topic hz /lane_following/debug/motion_cmd
ros2 topic echo /lane_following/debug/motion_cmd
ros2 topic echo /lane_following/status
ros2 param set /lane_following_node steering.alpha_rad_per_px 0.0015
ros2 param set /lane_following_node detector.evaluation_row_ratio 0.7
```

In the viewer, `S` saves a snapshot, `M` saves one marked for review, and `H`
shows help. Snapshots go to `~/lane_snapshots/` on the lane node host.

Webcam in WSL (untested). In Windows PowerShell as admin:
`winget install usbipd; usbipd list; usbipd bind --busid <ID>; usbipd attach --wsl --busid <ID>`,
then in WSL: `sudo modprobe uvcvideo && ls /dev/video*`.

If camera frames are slow between processes, raise the socket buffers:
`sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608`.

## 5. Mock vehicle (WSL)

This tests arming, stops and latching against a fake controller. Every terminal
must use the same non-zero domain.

```bash
export ROS_DOMAIN_ID=42
```

Not bench (live-camera path, the mock supplies a stamped camera):

```bash
ros2 launch lane_following vehicle_integration.launch.py mock:=true
```

Bench (saved input):

```bash
ros2 launch lane_following vehicle_integration.launch.py mock:=true bench:=true source:=image path:=$IMG/000000.png
ros2 launch lane_following vehicle_integration.launch.py mock:=true bench:=true source:=synthetic rate_hz:=20.0
```

Operate:

```bash
ros2 run lane_following lane_arm_console       # A arm, D disarm, Space/X stop, R reset, Q quit
ros2 service call /lane_following/arm std_srvs/srv/Trigger      # also: stop, reset, disarm
ros2 topic echo /vehicle/control/motion_cmd
ros2 topic echo /vehicle/control/func_cmd
ros2 topic echo /lane_following/status         # hw_state, hw_arm_blockers, hw_latch_cause
```

Inject faults (each should latch a stop):

```bash
ros2 param set /vehicle_controller mock.fault true
ros2 param set /vehicle_controller mock.motion_allowed false
ros2 param set /vehicle_controller mock.camera_rate_hz 0.0
ros2 param set /vehicle_controller mock.publish_state false
ros2 topic pub -r 5 /vehicle/control/func_cmd vehicle_msgs/msg/VehicleFunctionCommand "{motion_enable: false}"
```

## 6. Vehicle stack and teleop

These commands drive the real car, operated by hand. The default bringup uses
`vehicle_controller.yaml` (`require_function_command: false`, bench profile).

```bash
ros2 launch vehicle_controller vehicle_hardware_bringup.launch.py
ros2 launch vehicle_controller vehicle_hardware_bringup.launch.py controller_config:=/abs/other.yaml
ros2 run vehicle_keyboard_teleop keyboard_teleop
ros2 launch vehicle_keyboard_teleop vehicle_keyboard_teleop.launch.py
```

Teleop keys: `W/S` throttle, `A/D` steering ±2 turns, `C` centre, `R/F/N`
gear, `Space`/`Z` brake, `E` motion on/off, `X` e-stop, `1..5` aux relays, `Q` quit.

Monitor:

```bash
ros2 topic echo /vehicle/state/system
ros2 topic echo /vehicle/state/motion
ros2 run hardware_system_handler shm_viewer
```

## 7. Lane following on the car

This is steering only. Throttle and brake are always 0 and gear is NEUTRAL.
The node starts DISARMED and moves the steering only after manual arming, with
HSH allowing motion (AUTO switch, FSM running). Stop teleop first, or arming is
refused.

Preview on the car (cannot steer):

```bash
ros2 launch vehicle_controller vehicle_hardware_bringup.launch.py
ros2 launch lane_following camera_preview.launch.py source:=ros topic:=/camera/image_raw viewer:=true
ssh -L 8080:127.0.0.1:8080 <user>@<car>      # on the laptop, then open http://127.0.0.1:8080/
```

Bench mode: the car is raised and secured, there are no lanes in view, and the
input is saved. The steering cap is 0.2 rad (out of 1.0, about 14 motor turns).

```bash
scp test.png <user>@<car>:~/
ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=image path:=/home/<user>/test.png
ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=image path:=/abs/folder rate_hz:=2.0
ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=synthetic rate_hz:=20.0
ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=video path:=/abs/clip.mp4
ros2 launch lane_following vehicle_integration.launch.py bench:=true ... bench_steer_limit_rad:=0.3
ros2 run lane_following lane_arm_console
```

Not bench (live camera, normal operation):

```bash
ros2 launch lane_following vehicle_integration.launch.py topic:=/camera/image_raw
ros2 launch lane_following vehicle_integration.launch.py source:=camera device:=0
ros2 launch lane_following vehicle_integration.launch.py bringup:=false     # stack already running with the lane profile
ros2 launch lane_following vehicle_integration.launch.py steer_limit_rad:=0.3
ros2 run lane_following lane_arm_console
```

`vehicle_integration.launch.py` starts the bringup with
`lane_following/config/vehicle_controller_lane.yaml` (`require_function_command: true`).
With `bringup:=false`, the running stack must use that profile, or arming is refused.

First on-car checks, in order:

1. A positive request turns the wheels right (`images (1).jpg` gives +0.132 rad).
2. A left/right mirrored image turns them left.
3. The steering centres correctly.
4. The cap holds.
5. `Space`, MANUAL, a no-lane image, and killing the lane node each release the steering to IDLE.

## 8. CAN and hardware diagnostics

Virtual CAN (WSL or PC):

```bash
./setup_env/05_setup_vcan.sh
candump vcan0
cansend vcan0 123#DEADBEEF
```

Python tools (`workai` first):

```bash
workai
python3 tests/test_can_monitor.py --interface socketcan --port can0
python3 tests/test_can_monitor.py --port /dev/ttyACM0                    # slcan / CANable
python3 tests/test_can_payload_inspector.py --interface socketcan --port can0 --ids 500 580
python3 tests/can_bus_load.py can0 can1 --interval 1.0
python3 tests/test_can1_telemetry.py -i can1 [--ros]                     # speed + battery
python3 tests/test_modbus_ai8_ao8.py                                     # RS485 /dev/ttyUSB0
python3 tests/pot_relay_fun.py                                           # /dev/ttyUSB0 + can0
deactivate
```

Read each script before running it on the car. `test_modbus_ai8_ao8.py` and
`pot_relay_fun.py` write outputs.

ODrive tools (authorized, secured mechanism only):

```bash
ros2 run odrive_can_interface odrive_homing
ros2 run odrive_can_interface odrive_calib
```
