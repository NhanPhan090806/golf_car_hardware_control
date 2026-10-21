"""Phase B: lane following on the car, STEERING ONLY.

Starts the existing hardware bringup with the lane controller profile
(require_function_command: true) plus lane_following_node with
hardware.enabled. The node starts DISARMED; nothing moves until an operator
arms it (lane_arm_console or /lane_following/arm) AND HSH allows motion
(AUTO switch, FSM RUNNING). Throttle/brake are always 0, gear NEUTRAL.

  ros2 launch lane_following vehicle_integration.launch.py topic:=/camera/image_raw
  ros2 launch lane_following vehicle_integration.launch.py source:=camera device:=0
  ros2 launch lane_following vehicle_integration.launch.py bringup:=false   # stack already running

Bench test on a RAISED, SECURED car with no lanes in view (saved input):
  ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=image path:=/abs/test.png
  ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=synthetic  # sweeps left/right
  ros2 launch lane_following vehicle_integration.launch.py bench:=true source:=video path:=/abs/clip.mp4
Steering is capped at bench_steer_limit_rad (0.2). Still steering only, still
needs manual arming, HSH permission and the lane controller profile.

Mock test in WSL (no hardware):
  export ROS_DOMAIN_ID=42
  ros2 launch lane_following vehicle_integration.launch.py mock:=true

Do not run keyboard teleop at the same time; the node refuses to arm when
another publisher exists on the command topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lane_following')
    lane_cfg = os.path.join(share, 'config', 'lane_following.yaml')
    ctrl_cfg = os.path.join(share, 'config', 'vehicle_controller_lane.yaml')
    args = [
        DeclareLaunchArgument('config', default_value=lane_cfg),
        DeclareLaunchArgument('source', default_value='ros',
                              description='ros | camera; with bench:=true also image | video | synthetic'),
        DeclareLaunchArgument('path', default_value='', description='image/dir/video path for bench:=true'),
        DeclareLaunchArgument('rate_hz', default_value='10.0', description='image/synthetic replay rate'),
        DeclareLaunchArgument('bench', default_value='false',
                              description='allow saved input to steer a raised, secured car'),
        DeclareLaunchArgument('bench_steer_limit_rad', default_value='0.2'),
        DeclareLaunchArgument('topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('bringup', default_value='true',
                              description='start vehicle_hardware_bringup with the lane profile'),
        DeclareLaunchArgument('mock', default_value='false',
                              description='start mock_vehicle instead of hardware (needs ROS_DOMAIN_ID != 0)'),
        DeclareLaunchArgument('steer_limit_rad', default_value='0.5'),
        DeclareLaunchArgument('image_reliability', default_value='best_effort',
                              description='best_effort | reliable (mock:=true forces reliable)'),
    ]
    mock = LaunchConfiguration('mock')
    real_bringup = PythonExpression(["'", LaunchConfiguration('bringup'), "' == 'true' and '", mock, "' != 'true'"])
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_controller'), 'launch', 'vehicle_hardware_bringup.launch.py')),
        launch_arguments={'controller_config': ctrl_cfg}.items(),
        condition=IfCondition(real_bringup))
    mock_node = Node(package='lane_following', executable='mock_vehicle', name='vehicle_controller',
                     output='screen', condition=IfCondition(mock))
    lane = Node(
        package='lane_following', executable='lane_following_node', name='lane_following_node',
        output='screen',
        parameters=[LaunchConfiguration('config'), {
            'input.mode': ParameterValue(LaunchConfiguration('source'), value_type=str),
            'input.topic': ParameterValue(LaunchConfiguration('topic'), value_type=str),
            # path:= for bench files, otherwise the camera device index.
            'input.path': ParameterValue(PythonExpression(
                ["'", LaunchConfiguration('path'), "' or '", LaunchConfiguration('device'), "'"]), value_type=str),
            'input.rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
            'hardware.bench': ParameterValue(LaunchConfiguration('bench'), value_type=bool),
            'hardware.bench_steer_limit_rad': ParameterValue(LaunchConfiguration('bench_steer_limit_rad'),
                                                             value_type=float),
            'input.check_stamp_age': True,
            'input.loop': True,  # only matters for bench file input
            'input.image_reliability': ParameterValue(PythonExpression(
                ["'reliable' if '", mock, "' == 'true' else '", LaunchConfiguration('image_reliability'), "'"]),
                value_type=str),
            'hardware.enabled': True,
            'hardware.steer_limit_rad': ParameterValue(LaunchConfiguration('steer_limit_rad'), value_type=float),
        }])
    # Give the controller time to come up so the arm checks can see it.
    return LaunchDescription(args + [bringup, mock_node, TimerAction(period=3.0, actions=[lane])])
