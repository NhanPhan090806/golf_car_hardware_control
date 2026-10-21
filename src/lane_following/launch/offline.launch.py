"""Offline preview: saved image/dir, video file, or synthetic stream.

  ros2 launch lane_following offline.launch.py                       # synthetic
  ros2 launch lane_following offline.launch.py mode:=image path:=/abs/dir
  ros2 launch lane_following offline.launch.py mode:=video path:=/abs/clip.mp4
  ros2 launch lane_following offline.launch.py viewer:=true          # browser on :8080 (viewer_mode:=--opencv for a window)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lane_following')
    default_cfg = os.path.join(share, 'config', 'lane_following.yaml')
    args = [
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('mode', default_value='synthetic',
                              description='image | video | synthetic'),
        DeclareLaunchArgument('path', default_value=''),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('viewer_mode', default_value='--web', description='--web | --opencv'),
        DeclareLaunchArgument('viewer_port', default_value='8080'),
    ]
    node = Node(
        package='lane_following', executable='lane_following_node', name='lane_following_node',
        output='screen',
        parameters=[LaunchConfiguration('config'), {
            'input.mode': ParameterValue(LaunchConfiguration('mode'), value_type=str),
            'input.path': ParameterValue(LaunchConfiguration('path'), value_type=str),
            'input.rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
        }])
    viewer = Node(
        package='lane_following', executable='lane_viewer', name='lane_viewer', output='screen',
        arguments=[LaunchConfiguration('viewer_mode'), '--port', LaunchConfiguration('viewer_port')],
        condition=IfCondition(LaunchConfiguration('viewer')))
    return LaunchDescription(args + [node, viewer])
