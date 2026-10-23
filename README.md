<img width="1068" height="135" alt="image" src="https://github.com/user-attachments/assets/e9d65366-629e-46fb-9fc8-bc58435565f8" /># golf_car_hardware_control

ROS 2 Humble hardware-control stack for the autonomous car (golf car) of the
**FPT-Automation LAB, FPT University Can Tho Campus**, forked from [CQDdon/robot_hardware_control](https://github.com/CQDdon/robot_hardware_control).

This repository is the low-level control layer: it talks to the car's motor
drivers, steering, brake, throttle, relays and sensors, decides when motion is
allowed, and exposes a small ROS command/state interface for higher-level
software such as camera lane following.

```text
operator / autonomy  ->  /vehicle/control/*  ->  vehicle_controller (ros2_control)
                     ->  hardware interfaces  ->  HSH safety FSM  ->  ODrive CAN / RS485 / relays  ->  car
```

The Hardware System Handler (HSH) is the authority on whether the car may
move. A hardware interface being active does not mean motion is permitted.

## Packages (`src/`)

| Package | Purpose |
| --- | --- |
| `odrive_can_interface` | ros2_control hardware plugin, SocketCAN transport and ODrive protocol (steering, brake and wheel axes) |
| `rs485_modbus_interface` | RS485/Modbus I/O: throttle voltage, steering potentiometer |
| `relay_can_interface` | ESP32 CAN relay module: drive/direction relays, AUTO/MANUAL switch, E-stop input |
| `hardware_ipc_common` | Shared-memory layouts and snapshot protocol used by HSH, the hardware interfaces and the viewers |
| `hardware_system_handler` | HSH node and state machine (tinyfsm); viewers and simulators for the shared-memory blocks |
| `vehicle_controller` | ros2_control controller and hardware bringup launch |
| `vehicle_msgs` | Messages for `/vehicle/control/*` and `/vehicle/state/*` |
| `vehicle_keyboard_teleop` | Terminal teleoperation |
| `lane_following` | Camera lane detection with steering-only autonomous control; see its [README](src/lane_following/README.md) |
| `tinyfsm` | Vendored state-machine library |

Other folders:

- `setup_env/`: scripts that set up Ubuntu 22.04 / WSL2 from scratch, including
  the vcan kernel modules for WSL.
- `tests/`: standalone CAN/RS485 diagnostic scripts (run with the `ai_venv` venv).
- `lane_detection_plan/`: the lane-following design plan and test images.

## ROS interface

| Topic | Type | Direction |
| --- | --- | --- |
| `/vehicle/control/motion_cmd` | `vehicle_msgs/VehicleMotionCommand` | Command: steering (−1.0 left … +1.0 right), throttle %, brake %, gear |
| `/vehicle/control/func_cmd` | `vehicle_msgs/VehicleFunctionCommand` | Command: motion enable, emergency stop, aux relays |
| `/vehicle/state/motion` | `vehicle_msgs/VehicleMotionState` | State: measured steering, speed, battery |
| `/vehicle/state/system` | `vehicle_msgs/VehicleSystemState` | State: online, fault, motion allowed, timeouts |

Only one publisher should own each command topic during a run (for example,
teleop or lane following, never both).

## Quick start

```bash
git clone https://github.com/NhanPhan090806/golf_car_hardware_control.git ~/golf_car_hardware_control
cd ~/golf_car_hardware_control
./setup_env/install_all.sh          # ROS 2 Humble, dependencies, venv, vcan, workspace build
```

The workspace lives in `~/robot_hardware_ws`, with this repo linked as
`src/robot_hardware_control`. Rebuild with:

```bash
cd ~/robot_hardware_ws
colcon build --symlink-install
source install/setup.bash
```

Use system Python for ROS. Activate `~/ai_venv` only for the scripts in `tests/`.

**[run.md](run.md)** lists every command: tests, the lane-following preview, the
mock vehicle, teleop, lane following on the car (bench and live camera), and
CAN diagnostics.

## Safety

This software moves a real vehicle.

- Run calibration, homing, closed-loop motion and raw CAN transmit tests only
  when they are authorized and the mechanism is secured (car raised, a person at
  the physical E-stop, the AUTO/MANUAL switch in reach).
- The physical E-stop and AUTO/MANUAL switch are the primary safety controls.
  Software stops do not replace them.
- Changes to shared memory, HSH state transitions, command gating, watchdogs,
  homing or unit conversion are high risk. Read [AGENTS.md](AGENTS.md) before
  changing them.

## Status

| Area | Status |
| --- | --- |
| Hardware stack and teleop | Developed and tested against the car hardware |
| Lane following: detection, preview, mock-vehicle tests | Done; checked in WSL without hardware |
| Lane following on the car | Steering only (throttle/brake fixed at 0); not yet verified on the car |
| Autonomous driving with throttle | Not implemented |

## License

No repository-wide license has been chosen yet. Individual packages declare
their own license in `package.xml`. Contact the AI & IoT Lab, FPT University
Can Tho Campus, before reusing this code.
